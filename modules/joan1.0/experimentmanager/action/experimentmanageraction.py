from PyQt5 import QtWidgets

from core.joanmoduleaction import JoanModuleAction
from modules.joanmodules import JOANModules
from .condition import Condition
from .experiment import Experiment
import os, glob


class ExperimentManagerAction(JoanModuleAction):
    current_experiment: Experiment

    def __init__(self, millis=100):
        super().__init__(module=JOANModules.EXPERIMENT_MANAGER, use_state_machine_and_timer=False)

        self.current_experiment = None
        #TODO: We should definitely make a ROOT_PATH singleton somewhere in main so we dont have to do the following:
        cur_path = os.path.dirname(os.path.realpath(__file__))
        path = os.path.dirname(os.path.dirname(os.path.dirname(cur_path)))
        self.experiment_save_path = os.path.join(path, 'experiments/')

        self.active_condition = None
        self.active_condition_index = None

    def create_new_experiment(self, modules_to_include, save_path):
        # create the experiment
        self.current_experiment = Experiment(modules_to_include)
        self.current_experiment.set_from_current_settings(self.singleton_settings)
        self.experiment_save_path = save_path
        self.save_experiment()

        # update the gui
        self.module_dialog.update_gui()
        self.module_dialog.update_condition_lists()

        # and open the experiment dialog
        self.module_dialog.open_experiment_dialog()

    def save_experiment(self):
        if self.current_experiment:
            self.current_experiment.save_to_file(self.experiment_save_path)

    def load_experiment(self, file_path):
        self.experiment_save_path = file_path
        self.current_experiment = Experiment.load_from_file(file_path)
        self.module_dialog.update_gui()
        self.module_dialog.update_condition_lists()

    def activate_selected_condition(self, condition, condition_index):
        """
        To activate the condition, send the settings to the corresponding module (settings)
        :param condition:
        :param condition_index: self-explanatory
        :return:
        """

        for module, base_settings_dict in self.current_experiment.base_settings.items():
            module_settings_dict = base_settings_dict.copy()
            self._recursively_copy_dict(condition.diff[module], module_settings_dict)
            self.singleton_settings.get_settings(module).load_from_dict({str(module): module_settings_dict})

        self.active_condition = condition
        self.active_condition_index = condition_index

        return True

    def initialize_all(self):
        print(self.current_experiment.modules_included)
        if self.current_experiment:
            for module in self.current_experiment.modules_included:
                signals = self.singleton_signals.get_signals(module)
                signals.initialize_module.emit()

    def start_all(self):
        if self.current_experiment:
            for module in self.current_experiment.modules_included:
                signals = self.singleton_signals.get_signals(module)
                signals.start_module.emit()

    def stop_all(self):
        if self.current_experiment:
            for module in self.current_experiment.modules_included:
                signals = self.singleton_signals.get_signals(module)
                signals.stop_module.emit()

    def transition_to_next_condition(self):
        if not self.current_experiment:
            return False

        if not self.active_condition:
            self.active_condition_index = -1
        try:
            next_condition_or_transition = self.current_experiment.active_condition_sequence[self.active_condition_index + 1]
            if isinstance(next_condition_or_transition, Condition):
                return self.activate_selected_condition(next_condition_or_transition, self.active_condition_index + 1)
            else:
                transition = next_condition_or_transition
                transition.execute_before_new_condition_activation(self.current_experiment, self.active_condition)

                # search for the next condition
                added_index = 1
                while not isinstance(next_condition_or_transition, Condition):
                    try:
                        added_index += 1
                        next_condition_or_transition = self.current_experiment.active_condition_sequence[self.active_condition_index + added_index]
                    except IndexError:
                        # end of the list of conditions and transitions
                        return True
                    finally:
                        if added_index > 2:
                            QtWidgets.QMessageBox.warning(self.module_dialog, 'Warning', 'A sequence of multiple consecutive transitions was found. '
                                                                                         'This is illegal, only the first was executed, the others were '
                                                                                         'ignored.')

                if self.activate_selected_condition(next_condition_or_transition, self.active_condition_index + added_index):
                    transition.execute_after_new_condition_activation(self.current_experiment, self.active_condition)
                    return True
                else:
                    return False
        except IndexError:
            return False

    @staticmethod
    def _recursively_copy_dict(source, destination):
        for key, item in source.items():
            if isinstance(item, dict):
                try:
                    ExperimentManagerAction._recursively_copy_dict(item, destination[key])
                except KeyError:
                    destination[key] = {}
                    ExperimentManagerAction._recursively_copy_dict(item, destination[key])
            else:
                if item:  # check to avoid empty source list copying over filled destination list.
                    destination[key] = item
