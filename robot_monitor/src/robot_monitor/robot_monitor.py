#!/usr/bin/env python
import time
import os
import sys
from PyQt4 import QtGui
from PyQt4 import QtCore
from threading import Lock
import logging
import socket
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(":")

# from connection_monitor import RemoteMachine
from connectionless_monitor import RemoteMachine

class RemoteUpdateWorker(QtCore.QObject):
    callback_signal = QtCore.pyqtSignal(dict)
    def __init__(self, remote_machine, parent):
        QtCore.QObject.__init__(self)
        self._machine = remote_machine
        self._running = True
        parent._widgets["button_start"].clicked.connect(self.start_controller)
        parent._widgets["button_stop"].clicked.connect(self.stop_controller)

    def start_controller(self):
        try:
            self._machine.start_controller()
        except:
            logger.exception("Failed to start controller for %s" % self._machine._dns)

    def stop_controller(self):
        try:
            self._machine.stop_controller()
        except:
            logger.exception("Failed to stop controller for %s" % self._machine._dns)


    @QtCore.pyqtSlot()
    def do_update(self):
        while self._running:
            values = dict()
            values["ip"] = "IP: n/a"
            values["ping"] = "n/a"
            values["button_reset"] = False
            values["uptime"] = "n/a"
            values["cpu"] = 0.0
            values["mem"] =  {"used": 0,"total": 100}
            values["status"] = "Status: <span style='color:#aa0000;'>Disconnected</span>"
            values["cstatus"] = "Unknown"

            try:
                values["ip"] = "IP: %s" % self._machine.get_ip()
                ping = self._machine.ping()
                if float(ping['avgping']) > 5.0:
                    values["ping"] = "<span style='color:#aa0000;'>%s</span>" % ping['avgping']
                else:
                    values["ping"] = ping['avgping']
                self._machine.check_server()
            except socket.error as e:
                logger.error("socket error - %s - %s" % (self._machine._dns, str(e)))
            except:
                # If any of these fail, don't try to get any more information
                logger.exception("fail - %s" % self._machine._dns)
            else:

                values["status"] = "Status: <span style='color:#00aa00;'>Connected</span>"
                values["button_reset"] = True
                try:
                    telemetry = self._machine.get_telemetry()
                    cpu = telemetry["cpu"]
                    mem = telemetry["mem"]
                    uptime = telemetry["uptime"]
                    values["cpu"] = float(cpu['%usr']) + float(cpu['%sys'])
                    values["mem"] = mem
                    values["uptime"] = uptime #[3:].strip()
                except socket.error:
                    logger.error("Failed to get telemetry information for %s" % self._machine._dns)
                except:
                    logger.exception("Error processing telemetry for %s" % self._machine._dns)
                try:
                    values["cstatus"] = self._machine.controller_status()
                except: 
                    logger.error("Failed to get controller information for %s" % self._machine._dns)

            self.callback_signal.emit(values)
            time.sleep(1)


class MachineWidget(QtGui.QGroupBox):
    __update_widgets2_signal = QtCore.pyqtSignal(dict)
    __reboot_action_signal = QtCore.pyqtSignal()
    cancel_timer_signal = QtCore.pyqtSignal()
    def __init__(self, remote_machine, parent):
        self._parent = parent
        QtGui.QGroupBox.__init__(self, parent=self._parent)
        self._layout = QtGui.QVBoxLayout(self)
        self._machine = remote_machine
        self.machine_obj = None
        self._controller_status = ""
        self.setTitle(self._machine._dns)
        self._widgets = dict()  # Stores widgets that are drawn


        self.__update_widgets2_signal.connect(self.update_widgets2)
        self.__reboot_action_signal.connect(self.reboot_action)
        self._add_widget("label_connected", QtGui.QLabel("Status: Disconnected", self))
        # Force width size
        self._widgets["label_connected"].setMinimumWidth(230)
        self._widgets["label_connected"].setTextFormat(QtCore.Qt.RichText)
        self._add_widget("label_ip", QtGui.QLabel(self))
        self._add_widget("label_ping", QtGui.QLabel(self))
        self._widgets["label_ping"].setTextFormat(QtCore.Qt.RichText)
        self._add_widget("label_uptime", QtGui.QLabel(self))
        # Add CPU Info
        self._add_widget("label_cpu", QtGui.QLabel(self))
        self._add_widget("w_cpu", QtGui.QProgressBar(self))
        self._widgets["w_cpu"].setRange(0, 100)
        # Add Memory Info
        self._add_widget("label_mem", QtGui.QLabel(self))
        self._add_widget("w_memory", QtGui.QProgressBar(self))

        self._add_widget("button_reset", QtGui.QPushButton("Reboot",self))
        self._widgets["button_reset"].clicked.connect(self.__reboot_action_signal.emit)

        self._add_widget("label_controller", QtGui.QLabel(self))

        self._add_widget("button_start", QtGui.QPushButton("Start Controller",self))
        self._add_widget("button_stop", QtGui.QPushButton("Stop Controller",self))
#         self._widgets["button_start"].setEnabled(False)
        self._widgets["button_stop"].setEnabled(False)


        self.thread = QtCore.QThread(self)
        self.machine_obj = RemoteUpdateWorker(self._machine, self)
        self.machine_obj.callback_signal.connect(self.update_widgets2)
        self.machine_obj.moveToThread(self.thread)
        self.thread.started.connect(self.machine_obj.do_update)
        self.thread.start()

    def _add_widget(self, name, widget):
        """ adds widget to dictionary and puts it in the layout, in order """
        self._widgets[name] = widget
        self._layout.addWidget(self._widgets[name])

    def update_widgets2(self, values):
        self._widgets["button_reset"].setEnabled(values["button_reset"])
        self._widgets["label_connected"].setText(values["status"])
        self._widgets["label_ping"].setText("Latency: %s" % values["ping"])
        self._widgets["label_ip"].setText(values["ip"])
        self._widgets["label_uptime"].setText("Uptime: %s" % values["uptime"][3:])

        # Update CPU Value
        self._widgets["label_cpu"].setText("CPU: %.1f " % values["cpu"])
        self._widgets["w_cpu"].setValue(values["cpu"])

        # Update Memory Info
        self._widgets["label_mem"].setText("Memory: %s/%s" % (values["mem"]['used'],values["mem"]['total']))
        self._widgets["w_memory"].setRange(0, int(values["mem"]['total']))
        self._widgets["w_memory"].setValue(int(values["mem"]['used']))

        # Update Controller Info
        if not self._controller_status == values["cstatus"]:
            self._controller_status = values["cstatus"]
            if values["cstatus"] == "RUNNING":
                self._widgets["label_controller"].setText("Controller: <span style='color:#00aa00;'>RUNNING</span>")
                self._widgets["button_stop"].setEnabled(True)
                self._widgets["button_start"].setEnabled(False)
            elif values["cstatus"] == "STOPPED":
                self._widgets["label_controller"].setText("Controller: <span style='color:#b2b200;'>STOPPED</span>")
                self._widgets["button_start"].setEnabled(True)
                self._widgets["button_stop"].setEnabled(False)
            elif values["cstatus"] in ["FATAL", "EXITED"]:
                self._widgets["label_controller"].setText("Controller: <span style='color:#aa0000;'>%s</span>" % values["cstatus"])
                self._widgets["button_start"].setEnabled(True)
                self._widgets["button_stop"].setEnabled(False)
            else:
                if values["cstatus"] == "COMFAIL":
                    self._widgets["label_controller"].setText("Controller: <span style='color:#aa0000;'>%s</span>" % values["cstatus"])
                else:
                    self._widgets["label_controller"].setText("Controller: %s" % values["cstatus"])
                self._widgets["button_start"].setEnabled(False)
                self._widgets["button_stop"].setEnabled(False)


    def reboot_action(self):
        print "HAPPENIGN"
        msg = "Are you sure you want to reboot %s?" % self._machine._dns
        reply = QtGui.QMessageBox.question(self, 'Message', msg, 
                        QtGui.QMessageBox.Yes,
                        QtGui.QMessageBox.No)
        if reply == QtGui.QMessageBox.Yes:
            try:
                print "Rebooting %s" % self._machine._dns
                self._machine.reboot()
            except Exception as e:
                print "REBOOT FAILED: %s" % str(e)
        else:
            print "reboot aborted"

class MonitorConsole(QtGui.QWidget):
    """ Box that displays the different machines """
    def __init__(self):
        QtGui.QWidget.__init__(self)
        self._layout = QtGui.QHBoxLayout(self)
        self._machines = list()
        self.initUI()

    def add_machine(self, machine):
        new_machine = MachineWidget(machine, self)
        self._layout.addWidget(new_machine)
        self._machines.append(new_machine)

    def initUI(self):
        self.setGeometry(500,300,450,150)
        self.setWindowTitle("Robot Monitor Console")
        self.show()


class ConsoleMonitorConfig(dict):
    """ Loads arbitrary settings from a config file and from the command line.
    Settings are stored as config['section']['setting'] = value
    All names are automatically converted to lower case.
    """

    def _add_setting(self, section, setting, value):
        """ Adds a setting to the RobotConfig """
        section = section.lower()
        if section not in self.keys():
            self[section] = dict()
        self[section][setting] = value

    def parse_config_file(self, filename):
        """ Load arbitrary settings from config file"""
        import ConfigParser
        self.cfg_file = ConfigParser.ConfigParser()
        path = os.path.dirname(os.path.realpath(__file__))
        path = os.path.join(path, filename)
        try:
            self.cfg_file.readfp(open(path))
        except IOError as e:
            print "Unable to load config file '%s'" % path
            exit(0) 
        for section in self.cfg_file.sections():
            settings = self.cfg_file.options(section)
            for setting in settings:
                self._add_setting(section, setting, self.cfg_file.get(section, setting))

if __name__ == "__main__":
    a = QtGui.QApplication(sys.argv)
    w = MonitorConsole()

    config = ConsoleMonitorConfig()
    config.parse_config_file("config.cfg")

    for name, robot in config.items():
        print "Loading Robot:", name
#         m = RemoteMachine(robot["dns"], robot["user"], robot["passwd"])
        m = RemoteMachine(robot["dns"])
        w.add_machine(m)

    ret = a.exec_()
    sys.exit(ret)
