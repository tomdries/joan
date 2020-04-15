"""JOAN menu widget"""

import os
import sys

from PyQt5 import QtCore, QtWidgets, QtGui, uic

from modules.joanmenu.action.joanmenu import JOANMenuAction
from modules.joanmodules import JOANModules
from process import Control


class JOANMenuWidget(Control):
    """JOAN Menu widget"""

    app_is_quiting = QtCore.pyqtSignal()

    def __init__(self, *args, **kwargs):
        kwargs['millis'] = 'millis' in kwargs.keys() and kwargs['millis'] or 200
        kwargs['callback'] = []  # method will run each given millis

        Control.__init__(self, *args, **kwargs)

        # create action class
        # TODO Nu wordt nog de ...Widget geinstantieerd, maar straks wordt de ...Action de 'hoofdklasse', dan kan dit weg.
        self.action = JOANMenuAction(self)

        # path to resources folder
        self._path_resources = os.path.normpath(os.path.join(os.path.dirname(os.path.realpath(__file__)), "../../../", "resources"))
        self._path_modules = self.action.path_modules

        # setup window
        self.window = QtWidgets.QMainWindow()
        self.window.setWindowTitle('JOAN')
        self._main_widget = uic.loadUi(os.path.join(os.path.dirname(os.path.realpath(__file__)), "joanmenu.ui"))
        self._main_widget.lbl_master_state.setText(self.masterStateHandler.getCurrentState().name)
        self.window.setCentralWidget(self._main_widget)
        self.window.resize(400, 400)

        self._main_widget.btn_emergency.setIcon(QtGui.QIcon(QtGui.QPixmap(os.path.join(self._path_resources, "stop.png"))))
        self._main_widget.btn_emergency.clicked.connect(self.action.emergency)

        self._main_widget.btn_quit.setStyleSheet("background-color: darkred")
        self._main_widget.btn_quit.clicked.connect(self.action.quit)

        self._main_widget.btn_initialize_all.clicked.connect(self.action.initialize)
        self._main_widget.btn_start_all.clicked.connect(self.action.start)
        self._main_widget.btn_stop_all.clicked.connect(self.action.stop)

        # layout for the module groupbox
        # TODO Dit kan mooi in de UI ook al gezet worden, zie Joris' hardwaremanager
        self._layout_modules = QtWidgets.QVBoxLayout()
        self._main_widget.grpbox_modules.setLayout(self._layout_modules)

        # dictionary to store all the module widgets
        self._module_widgets = {}

        # add file menu
        self.file_menu = self.window.menuBar().addMenu('File')
        self.file_menu.addAction('Quit', self.quit)
        self.file_menu.addSeparator()
        self.file_menu.addAction('Add module...', self.process_menu_add_module)
        self.file_menu.addAction('Remove module...', self.process_menu_remove_module)

    def add_module(self, module: JOANModules, name=''):
        """Instantiate module, create a widget and add to main window"""
        # instantiate module (in Action)
        module_widget, module_action = self.action.add_module(module, name, parent=self.window)
        if not module_widget:
            return

        # create a 'tab' widget
        # TODO For now, the widget class keeps track of the tab widgets; but this 
        name = str(module)

        widget = uic.loadUi(os.path.join(os.path.dirname(os.path.realpath(__file__)), "modulewidget.ui"))
        widget.setObjectName(name)
        widget.grpbox.setTitle(name)

        if module is JOANModules.TEMPLATE:  # syntax is changed slightly in new example: wrapping show() in _show() is unnecessary
            widget.btn_show.clicked.connect(module_widget.show)
            widget.btn_close.clicked.connect(module_widget.close)

            module_action.moduleStateHandler.stateChanged.connect(lambda state: widget.lbl_state.setText(module_action.moduleStateHandler.getState(state).name))
        else:
            widget.btn_show.clicked.connect(module_widget._show)
            widget.btn_close.clicked.connect(module_widget._close)

            widget.lbl_state.setText(module_widget.moduleStateHandler.getCurrentState().name)
            module_widget.moduleStateHandler.stateChanged.connect(
                lambda state: widget.lbl_state.setText(module_widget.moduleStateHandler.getState(state).name)
            )

        # add it to the layout
        self._layout_modules.addWidget(widget)
        # self._main_widget.scrollArea.adjustSize()
        self.window.adjustSize()

        # and to the list
        self._module_widgets[name] = widget

    def process_menu_add_module(self):
        """Add module in menu clicked, add user-defined module"""
        path_module_dir = QtWidgets.QFileDialog.getExistingDirectory(
            self.window, caption="Select module directory", directory=self._path_modules, options=QtWidgets.QFileDialog.ShowDirsOnly
        )

        # extract module folder name
        module = '%s%s' % (os.path.basename(os.path.normpath(path_module_dir)), 'Widget')

        # add the module
        self.add_module(module)

    def process_menu_remove_module(self):
        """User hit remove module, ask them which one to remove"""
        name, _ = QtWidgets.QInputDialog.getItem(
            self.window, "Select module to remove", "Modules", list(self.action.instantiated_modules.keys())
        )

        # remove the module in action
        self.action.remove_module(name)

        # remove the widget in the main menu
        if name in self._module_widgets.keys():
            self._module_widgets[name].setParent(None)  # setting parent to None destroys the widget (garbage collector)
            del self._module_widgets[name]
            # adjust size
            self._main_widget.grpBoxModules.adjustSize()
            self._main_widget.adjustSize()
            self.window.adjustSize()

    def closeEvent(self, event):
        """redefined closeEvent"""
        # call our quit function
        self.action.quit()

        # if we end up here, it means we didn't want to quit
        # hence, ignore the event (for Qt)
        event.ignore()

