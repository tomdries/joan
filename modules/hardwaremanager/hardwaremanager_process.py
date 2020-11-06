from core.module_process import ModuleProcess
from modules.hardwaremanager.hardwaremanager_inputtypes import HardwareInputTypes
from modules.joanmodules import JOANModules


class HardwareManagerProcess(ModuleProcess):
    """
    Overall process that inherits from ModuleProcess (will loop at the desired frequency)
    """
    def __init__(self, module: JOANModules, time_step_in_ms, news, settings, events, settings_singleton):
        super().__init__(module, time_step_in_ms=time_step_in_ms, news=news, settings=settings, events=events, settings_singleton= settings_singleton)
        self.daemon = False
        self.input_objects = {}

    def get_ready(self):
        # Create the objects that are in the settings here
        for key, value in self._settings_as_object.inputs.items():
            self.input_objects[key] = HardwareInputTypes(value.input_type).process(settings=value, shared_variables=self._module_shared_variables.inputs[key])

    def do_while_running(self):
        for inputs in self.input_objects:
            # will perform the mp input class for eaach available input
            self.input_objects[inputs].do()

