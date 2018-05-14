import os
import sys
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

from connection_monitor import RemoteMachine
from robot_monitor import MonitorConsole
from robot_monitor import ConsoleMonitorConfig

class MonitorPlugin(Plugin):

    def __init__(self, context):
        super(MonitorPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('Robot Monitor')
        # Create QWidget
        self._widget = MonitorConsole()
        config = ConsoleMonitorConfig()
        config.parse_config_file("config.cfg")
        for name, robot in config.items():
            print "Loading Robot:", name
            m = RemoteMachine(robot["dns"], robot["user"], robot["passwd"])
            self._widget.add_machine(m)

        # Get path to UI file which should be in the "resource" folder of this package
        # ui_file = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'resource', 'MyPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        # loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MonitorConsoleWidget')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
