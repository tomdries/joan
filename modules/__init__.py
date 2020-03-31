# relative import used by sibling packages
# action classes, belonging to widgets (within the same module directory) are imported in the widget-class
#from .menu.widget.menu import MenuWidget
#from .datarecorder.widget.datarecorder import DatarecorderWidget
from .hardwarecommunication.widget.hardwarecommunication import HardwarecommunicationWidget
from .feedbackcontroller.widget.feedbackcontroller import FeedbackcontrollerWidget
from .siminterface.widget.siminterface import SiminterfaceWidget
from .trajectoryrecorder.widget.trajectoryrecorder import TrajectoryrecorderWidget

#from .steeringcommunication.widget.steeringcommunication import SteeringcommunicationWidget
#from .template.widget.template import TemplateWidget
#from .interface.widget.interface import InterfaceWidget