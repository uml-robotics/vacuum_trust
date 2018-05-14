#!/usr/bin/env python

from reliable_rospy.reliable_node import ReliableNode
import sys
from PyQt4 import QtGui,QtCore
from std_msgs.msg import *

TOPIC_EXPERIMENT_TIME = "/experiment/time"

class StretchedLabel(QtGui.QLabel):
    """ Automatically resize Font to fit window http://stackoverflow.com/a/8806911 """
    def __init__(self, *args, **kwargs):
        QtGui.QLabel.__init__(self, *args, **kwargs)
        self.setSizePolicy(QtGui.QSizePolicy.Ignored, QtGui.QSizePolicy.Ignored)
        self.setAlignment(QtCore.Qt.AlignCenter)

    def resizeEvent(self, evt):
        font = self.font()
        font.setPixelSize(self.height() * 0.4)
        self.setFont(font)

class TimeDisplay(QtGui.QWidget):
    __time_update_signal = QtCore.pyqtSignal()
    def __init__(self):
        self.ros = ReliableNode('time_display', anonymous=True)
        self.ros.Subscriber(TOPIC_EXPERIMENT_TIME, Int32, lambda x: self._time_cb(x))
        QtGui.QWidget.__init__(self)
        self.setWindowTitle("Timer")
        self._layout = QtGui.QHBoxLayout(self)
        self._time_label = StretchedLabel(self)
        self._layout.addWidget(self._time_label)
        self._time_seconds = 0

        self.__time_update_signal.connect(self._update_label)
        self.__time_update_signal.emit()
        self.setGeometry(500,300,450,150)
        

    def _update_label(self):
        minutes = self._time_seconds / 60
        seconds = self._time_seconds % 60
        self._time_label.setText("%2.2d:%2.2d" % (minutes, seconds))

    def _time_cb(self, data):
        print "Received:", data
        self._time_seconds = int(data.data)
        self.__time_update_signal.emit()

    def closeEvent(self, event):
        print "Shutting down reliable node"
        self.ros.shutdown()


if __name__ == "__main__":
    a = QtGui.QApplication(sys.argv)
    e = TimeDisplay()
    e.show()
    e.showFullScreen()
    sys.exit(a.exec_())

