#!/usr/bin/env python

"""
" File: wizard_controls.py
" Author: James T. Kuczynski <jkuczyns@cs.uml.edu>
" File Description: This ROS node receives data from one or more topics and
"                   visualizes it in graph(s), which are updated in real time.
"
" Created: 01/21/2017 by J.K.
" Last Modified: 02/01/2017 by J.K.
"""

import rospy
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import sys
import os
import console_monitor_config
import robot_widget

DEFAULT_FIELDS = ['battery_voltage', 'lifted', 'charging', 'docked', 'dustbin', 'cleaning', 'sci_error']

"""
" Main UI class, which widgets can be added to.
"""
class WizardControls(QtGui.QMainWindow):
    def __init__(self, fileName):
        super(WizardControls, self).__init__()

        self.setObjectName('wizard')
        self.setWindowTitle('Vacuum Wizard Controls')
        pg.setConfigOption('background', pg.mkColor(250, 250, 250, 100))  # Set PyQtGraph widgets to transparent

        self.cfgDict = console_monitor_config.ConsoleMonitorConfig()
        self.robotWidArr = []

        self.cfgToUi(fileName)

        # load the style file
        path = os.path.dirname(os.path.realpath(__file__))
        path = os.path.join(path, 'css/classic.css')
        try:
            with open(path, 'r') as myFile:
                style = myFile.read().replace('\n', '')
            self.setStyleSheet(style)
        except IOError as e:
            print "Unable to load css file '%s'" % path


    """
    " Load configuration files
    """
    def cfgToUi(self, fileName):
        self.cfgDict.parse_config_file(fileName)  # read file, text --> widgets

        # Find the max number of plots and UI buttons, so we know how much padding to use
        # for each RobotWidget
        maxPlotLen = 0
        maxBtnLen = 0
        for robotName in self.cfgDict.keys():
            currPlotLen = len(self.cfgDict[robotName]['buttons'].split(','))
            currBtnLen = len(self.cfgDict[robotName]['commands'].split(','))
            if currPlotLen > maxPlotLen:
                maxPlotLen = currPlotLen
            if currBtnLen > maxBtnLen:
                maxBtnLen = currBtnLen
        maxPlotLen += len(DEFAULT_FIELDS)

        # Take config data and generate a robot widget
        for robotName in self.cfgDict.keys():
            allRdFieldArr = list(DEFAULT_FIELDS) + self.cfgDict[robotName]['buttons'].split(',')
            btnArr = self.cfgDict[robotName]['buttons'].split(',')
            cmdArr = self.cfgDict[robotName]['commands'].split(',')
            allRdFieldArr.sort()
            btnArr.sort()
            cmdArr.sort()

            #print "robot, commands: ", robotName, ', ', cmdArr
            self.addRobot(robotName, allRdFieldArr, btnArr, cmdArr, maxPlotLen - len(allRdFieldArr), maxBtnLen - len(cmdArr))


    """
    " Adds a robot's elements to the UI
    """
    def addRobot(self, robotName, topicArr, tmpArr, btnArr, spacers, btnSpacers):
        self.robotWidArr.append(robot_widget.RobotWidget(robotName, topicArr, tmpArr, btnArr, spacers, btnSpacers))
        dock = QtGui.QDockWidget() # Put RobotWidget in a dock so it can be drag and dropped
        dock.setFeatures(QtGui.QDockWidget.DockWidgetFloatable | QtGui.QDockWidget.DockWidgetMovable)
        dock.setWindowTitle(self.robotWidArr[len(self.robotWidArr) - 1].robotName)
        dock.setWidget(self.robotWidArr[len(self.robotWidArr) - 1])
        self.addDockWidget(QtCore.Qt.BottomDockWidgetArea, dock)


    """
    " Update a robot's plots
    """
    def update(self):
        for robotWid in self.robotWidArr:
            robotWid.update()


    """
    " Kill the ROS thread(s) when the UI thread is killed by the user.
    """
    def closeEvent(self, event):
        rospy.signal_shutdown('UI requested shutdown')
        event.accept()



if __name__ == '__main__':
    rospy.init_node('graph_test2', anonymous=True)

    app = QtGui.QApplication(sys.argv)

    robot_widget.WizardParams.NUM_PLOT_POINTS=60    #number of data points visible
    robot_widget.WizardParams.PLOT_HEIGHT=20        # Graph height (pixels)

    wizardControls = WizardControls("config.cfg")
    wizardControls.show()

    # Create & start timer, which will update the plots
    timer = QtCore.QTimer()
    timer.timeout.connect(wizardControls.update)
    timer.start(1000)  # Update rate; will update the UI every 500 milliseconds

    app.exec_()

    rospy.spin()
