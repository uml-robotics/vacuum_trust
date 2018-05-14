#!/usr/bin/env python
# [db] dan@danbrooks.net 2017

import rospy
from std_msgs.msg import String
import sys
from python_qt_binding import QtGui,QtCore
from python_qt_binding.QtWidgets import QWidget, QLabel, QLineEdit, QPushButton, QApplication
from python_qt_binding.QtWidgets import QGridLayout
from data_logger.srv import start as DataLoggerStart
from std_srvs.srv import Empty as DataLoggerStop
from std_msgs.msg import String
import os
LOG_DEST="~/data/"

class LoggingWidget(QWidget):
    __logging_cb_signal = QtCore.pyqtSignal(String)
    __start_logging_signal = QtCore.pyqtSignal()
    __stop_logging_signal = QtCore.pyqtSignal()
    def __init__(self):
        QWidget.__init__(self)
        self._status = "unknown"
        self._layout = QGridLayout(self)
        self._widgets = dict()

        self.__logging_cb_signal.connect(self.logging_cb)
        self.__start_logging_signal.connect(self.start_logging)
        self.__stop_logging_signal.connect(self.stop_logging)
        self._add_widget("pid_label", QLabel("PID:",self))
        self._add_widget("pid", QLineEdit(self))
        self._widgets["pid"].setMaximumWidth(50)

        self._add_widget("label_status", QLabel(self))
        self._widgets["label_status"].setText("Status: Unknown")
        self._widgets["label_status"].setTextFormat(QtCore.Qt.RichText)
        self._add_widget("button_start", QPushButton("START", self))
        self._add_widget("button_stop", QPushButton("STOP", self))
        self._widgets["button_start"].clicked.connect(self.__start_logging_signal.emit)
        self._widgets["button_stop"].clicked.connect(self.__stop_logging_signal.emit)
        
        self._widgets["label_warning"] = QLabel(self)
        self._layout.addWidget(self._widgets["label_warning"],1, 0, 1, 2)
        self._widgets["label_warning"].setTextFormat(QtCore.Qt.RichText)

        rospy.Subscriber("/data_logger/status", String, self.__logging_cb_signal.emit)
        self._logging_start = rospy.ServiceProxy('/data_logger/start', DataLoggerStart)
        self._logging_stop = rospy.ServiceProxy('/data_logger/stop', DataLoggerStop)

    def _add_widget(self, name, widget):
        """ adds widget to dictionary and puts it in the layout, in order """
        self._widgets[name] = widget
        self._layout.addWidget(self._widgets[name], 0, len(self._widgets.keys())-1)

    def logging_cb(self, data):
        self._status = data.data
        if data.data == "stopped":
            self._widgets["button_stop"].setEnabled(False)
            self._widgets["button_start"].setEnabled(True)
            self._widgets["label_status"].setText("Status: %s" % data.data)
        elif data.data == "started":
            self._widgets["button_stop"].setEnabled(True)
            self._widgets["button_start"].setEnabled(False)
            self._widgets["label_status"].setText("Status: <span style='color:#00aa00;'>%s</span>" % data.data)


    def start_logging(self):
        pid = self._widgets["pid"].text()
        log_path = LOG_DEST+"%s" % pid
        log_path = os.path.abspath(os.path.expanduser(log_path))
        # Make the directory
        if not os.path.exists(log_path):
            print "Creating destination directory '%s'" % log_path
            os.makedirs(log_path)
        try:
            self._logging_start(log_path,"--all")
        except Exception as e:
            print "Failed to start logging service:"
            print str(e)
        
    def stop_logging(self):
        try:
            self._logging_stop()
        except Exception as e:
            print "Failed to stop logging service:"
            print str(e)



if __name__ == "__main__":
    rospy.init_node('LoggingInterface')
    a = QApplication(sys.argv)
    e = LoggingWidget()
    e.show()
    ret = a.exec_()
    rospy.signal_shutdown("Application Close")
    sys.exit(ret)

