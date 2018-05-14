#!/usr/bin/env python

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import sys
import os

import console_monitor_config
import robot

COLORS = [ ('red', (255, 0, 0, 100)),      ('green', (0, 255, 0, 100)),  ('blue', (0, 0, 255, 100)),
           ('yellow', (255, 255, 0, 100)), ('cyan', (0, 255, 255, 100)), ('magenta', (255, 0, 255, 100))]

class PingMonitor(QtGui.QMainWindow):
    def __init__(self, fileName):
        super(PingMonitor, self).__init__()

        self._cfgDict = console_monitor_config.ConsoleMonitorConfig()
        self._robotArr = []

        self.initUI()
        self.cfgToUi(fileName)



    def cfgToUi(self, fileName):
        self._cfgDict.parse_config_file(fileName)

        count = 0
        for robotName in self._cfgDict.keys():
            #print "robot name: ", robotName, "curve color (rgb): ", COLORS[count][1]
            self._robotArr.append(robot.Robot(robotName, self._cfgDict[robotName]['address'], self._plotItem, COLORS[count][1]))
            count += 1

    def initUI(self):
        self.outerLayout = QtGui.QGridLayout()

        # Create and add the plot
        self._win = pg.GraphicsLayoutWidget()
        self._plotItem = self._win.addPlot()
        self.setCentralWidget(self._win)


        self._plotItem.addLegend()



    def update(self):
        idx = 0
        for robot in self._robotArr:
            robot.updateUi()
            if(robot._isSuccess):
                self._plotItem.legend.items[idx][1].item.setHtml('<div style="background:#141414; color: white;">' + robot._robotName + '</p>')
            else:
                self._plotItem.legend.items[idx][1].item.setHtml('<div style="background:#ff0000; color: white;">' + robot._robotName + '</p>')
            idx += 1



if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)

    ###

    pingMonitor = PingMonitor("res/config.cfg")
    pingMonitor.show()

    # Create & start timer, which will update the plots
    timer = QtCore.QTimer()
    timer.timeout.connect(pingMonitor.update)
    timer.start(1000)

    app.exec_()