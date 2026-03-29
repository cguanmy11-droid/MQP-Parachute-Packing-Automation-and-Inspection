"""Widgets package for parachute GUI."""
from parachute_gui.widgets.state_machine_widget import StateMachineWidget
from parachute_gui.widgets.control_widgets import LoopSelectorWidget, ArmControlWidget
from parachute_gui.widgets.camera_widget import CameraWidget

__all__ = ['StateMachineWidget', 'LoopSelectorWidget', 'ArmControlWidget', 'CameraWidget']
