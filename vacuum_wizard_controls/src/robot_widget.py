#!/usr/bin/env python

"""
" File: robot_widget.py
" Author: James T. Kuczynski <jkuczyns@cs.uml.edu>
" File Description: This ROS node receives data from one or more topics and
"                   visualizes it in graph(s), which are updated in real time.
"
" Created: 01/21/2017 by J.K.
" Last Modified: 02/01/2017 by J.K.
"""

import rospy
from std_msgs.msg import String
from vacuum_experiment_msgs.msg import Telemetry, PhoneReply
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import numpy as np


class WizardParams():
    NUM_PLOT_POINTS = 60    # Number of points visible in a plot
    PLOT_HEIGHT = 20        # Height of each graph in pixels
    CURVE_COLOR = (0, 0, 0, 100) #r, g, b, transparency (100=no transparency)
    CURVE_2_COLOR = (255, 0, 0, 100)


"""
" Contains the widgets of a robot (i.e. robot name, graphs, and buttons)
"""
class RobotWidget(QtGui.QFrame):
    def __init__(self, title, topicArr, tmpArr, cmdBtnArr, spacers, btnSpacers):
        super(RobotWidget, self).__init__()

        self.setObjectName('robotWidget') # CSS object name
        self.robotName = title          # The name of the robot to be used as the title of the widget
        self.plotWidArr = []            # Array of PlotWidgets
        self.gotNewData = False         # FIXME: Should this be a int???
        self.gotNewAppData = False
        self.cmdBtnArr = cmdBtnArr      # Commands which can be sent to a robot
        self.topicArr = topicArr        # The custom buttons on a robot + default telemetry fields
        self.defaultTopicArr = tmpArr   # The default telemetry fields which every robot has
        self.cmdTupArr = []             # Array of tuples (QPushButton, Publisher)
        self.msgRingBuf = []            # Ring buffer of Telemetry msgs
        self.msgAppRingBuf = []
        self.msgRingMutex = QtCore.QMutex() # Mutex for the ring buffer
        self.msgAppRingMutex = QtCore.QMutex()
        self.btnLay = QtGui.QGridLayout() # Layout for the UI buttons
        self.isOddNumBtn = True         # Flag for formatting buttons
        self.outerLayout = QtGui.QGridLayout() # Base layout for the RobotWidget

        rospy.Subscriber('/' + self.robotName + '/telemetry', Telemetry, self.callback)  # Subscriber for this robot
        rospy.Subscriber('/' + self.robotName + '/phonereply', PhoneReply, self.app_callback)

        for topic in self.topicArr:
            if topic in self.defaultTopicArr:
                curvesNum = 2
            else:
                curvesNum = 1

            self.addPlot(title, topic, curvesNum)

        for i in range(spacers):
            space = QtGui.QSpacerItem(25, 100, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
            self.outerLayout.addItem(space, len(self.plotWidArr) + 1 + i, 0,
                                     QtCore.Qt.AlignHCenter | QtCore.Qt.AlignTop)

        #TODO:
        """
        for cmd in cmdBtnArr:
            self.addButton(cmd)
        """

        for i in range(btnSpacers):
            self.btnLay.setRowMinimumHeight((len(self.cmdTupArr) + i) / 2, 35)  # FIXME???

        self.outerLayout.setRowStretch(len(self.plotWidArr) + 1 + spacers, 1)
        self.outerLayout.addLayout(self.btnLay, len(self.plotWidArr) + 1 + spacers, 0,
                                   QtCore.Qt.AlignHCenter | QtCore.Qt.AlignBottom)
        self.setLayout(self.outerLayout)


    """
    " Adds a plot (graph) to this robot's widget
    """
    def addPlot(self, robotName, field, curvesNum):
        win = pg.GraphicsLayoutWidget()
        win.centralWidget.setContentsMargins(0, 0, 0, 0)
        wid = PlotWidget(robotName, field, Plot(win.addPlot(), field, curvesNum), win)
        self.plotWidArr.append(wid)
        self.outerLayout.addWidget(wid, len(self.plotWidArr), 0, QtCore.Qt.AlignHCenter | QtCore.Qt.AlignTop)


    """
    " ROS callback.  Gets Telemetry msgs
    """
    def callback(self, msg):
        self.msgRingMutex.lock()
        if len(self.msgRingBuf) == WizardParams.NUM_PLOT_POINTS:
            self.msgRingBuf[:-1] = self.msgRingBuf[1:]
            self.msgRingBuf[-1] = msg
        else:
            self.msgRingBuf.append(msg)
        self.msgRingMutex.unlock()

        self.msgToDict(msg)
        self.fakeAppCallback()

        self.gotNewData = True
        
        
    """
    " 
    """
    def app_callback(self, msg):
        self.msgAppRingMutex.lock()
        if len(self.msgAppRingBuf) == WizardParams.NUM_PLOT_POINTS:
            self.msgAppRingBuf[:-1] = self.msgAppRingBuf[1:]
            self.msgAppRingBuf[-1] = msg
        else:
            self.msgAppRingBuf.append(msg)
        self.msgAppRingMutex.unlock()
        
        self.msgAppToDict(msg)
        self.fakeCallback()

        self.gotNewAppData = True

    """
    " Updates the ring buffer even if no new data has been received from ROS.
    " If no new data, delete the first entry, shift, and then copy the last
    " entry.
    """
    def fakeCallback(self):
        self.msgRingMutex.lock()
        if len(self.msgRingBuf) > 0:
            msg = self.msgRingBuf[-1]
            self.msgRingBuf[:-1] = self.msgRingBuf[1:]
            self.msgRingBuf[-1] = msg
        else:
            # (edge case) If we have recieved NO valid msgs, must create a msg with all 0/False/empty
            msg = Telemetry()
            msg.battery_voltage = 0
            msg.lifted = False
            msg.charging = False
            msg.docked = False
            msg.dustbin = False
            msg.cleaning = False
            msg.sci_error = False
            msg.buttons = []
            
        self.msgToDict(msg)
        self.msgRingMutex.unlock()
        
        
    def fakeAppCallback(self):
        self.msgAppRingMutex.lock()
        if len(self.msgAppRingBuf) > 0:
            msg = self.msgAppRingBuf[-1]
            self.msgAppRingBuf[:-1] = self.msgAppRingBuf[1:]
            self.msgAppRingBuf[-1] = msg
        else:
            # (edge case) If we have recieved NO valid msgs, must create a msg with all 0/False/empty
            msg = PhoneReply()
            msg.buttons = []
            
        self.msgAppToDict(msg)
        self.msgAppRingMutex.unlock()


    """
    " Convert Telemetry msg to a dictionary
    """
    def msgToDict(self, msg):
        msgAsDict = {}
        msgAsDict['battery_voltage'] = msg.battery_voltage
        msgAsDict['lifted'] = msg.lifted
        msgAsDict['charging'] = msg.charging
        msgAsDict['docked'] = msg.docked
        msgAsDict['dustbin'] = msg.dustbin
        msgAsDict['cleaning'] = msg.cleaning
        msgAsDict['sci_error'] = msg.sci_error

        # Robot button presses
        for key in self.defaultTopicArr:
            if key in msg.buttons:
                msgAsDict[key] = True
            else:
                #print "setting to False: ", key
                msgAsDict[key] = False

        for plot in self.plotWidArr:
            plot.plot.callback(msgAsDict)  # TODO: don't pass whole dict
            
            
    def msgAppToDict(self, msg):
        msgAppAsDict = {}
        #Android button presses
        for key in self.defaultTopicArr:
            if key in msg.buttons:
                msgAppAsDict[key] = True
            else:
                msgAppAsDict[key] = False
                
        for plot in self.plotWidArr:
            plot.plot.app_callback(msgAppAsDict)  # TODO: don't pass whole dict


    """
    " Update each plot contained within this widget
    """
    def update(self):
        if not self.gotNewData:
            self.fakeCallback()
            
        if not self.gotNewAppData:
            self.fakeAppCallback()

        for plotWid in self.plotWidArr:
            plotWid.plot.update()

        self.gotNewData = False
        self.gotNewAppData = False


"""
" PyQtGraph's PlotItem is NOT a descendent of QWidget, so it CANNOT be
" directly added to a QT widget/layout.  Hence the reason this PlotWidget
" class is nessissary.
"""
class PlotWidget(QtGui.QFrame):
    def __init__(self, robotName, title, plot, win):
        super(PlotWidget, self).__init__()

        self.robotName = robotName
        self.titleBtn = QtGui.QPushButton(title)#QtGui.QLabel(title)    # Create title for plot
        self.titleBtn.setToolTip('/' + self.robotName + '/cmd')
        #TODO: fix this hack
        if title == 'sci_error':
            self.auxBtn = QtGui.QPushButton('baud_reset')
            self.auxBtn.setToolTip('/' + self.robotName + '/cmd')
            self.auxPub = rospy.Publisher('/' + self.robotName + '/cmd', String, queue_size=10)
            self.auxBtn.clicked.connect(lambda: self.auxBtnSlot(self.auxBtn.text()))

        self.titleBtn.clicked.connect(lambda: self.btnSlot(self.titleBtn.text()))
        self.pub = rospy.Publisher('/' + self.robotName + '/cmd', String, queue_size=10)
        font = self.titleBtn.font()            # Font object for the title
        font.setPointSize(8)
        self.titleBtn.setFont(font)
        self.plot = plot                    # The PyQtGraph object
        self.win = win                      # The PyQtGraph widget
        self.setObjectName('PlotWidget')
        self.outerLayout = QtGui.QGridLayout() # Base layout for this PlotWidget instance


        self.outerLayout.setContentsMargins(0, 0, 0, 0)
        if title != 'sci_error':
            self.outerLayout.addWidget(self.titleBtn, 0, 0, QtCore.Qt.AlignHCenter | QtCore.Qt.AlignTop)
        else:
            self.auxBtn.setFont(font)
            self.btnLay = QtGui.QHBoxLayout()
            self.btnLay.setContentsMargins(0, 0, 0, 0)
            self.btnLay.addWidget(self.titleBtn)
            self.btnLay.addWidget(self.auxBtn)
            self.outerLayout.addLayout(self.btnLay, 0, 0, QtCore.Qt.AlignHCenter | QtCore.Qt.AlignTop)

        self.outerLayout.addWidget(self.win, 1, 0, QtCore.Qt.AlignHCenter | QtCore.Qt.AlignTop)

        self.setToolTip(self.plot.tooltip)

        self.win.setMinimumHeight(WizardParams.PLOT_HEIGHT)
        self.win.setMaximumHeight(WizardParams.PLOT_HEIGHT)

        self.setLayout(self.outerLayout)


    """
    " QT slot triggered when a button is pressed.  The signal sends the
    " button's text, which is used as a ID.
    """
    def btnSlot(self, text):
        # Find publisher matching the text, and publish the command
        self.pub.publish(str(self.titleBtn.text()))


    """
    " QT slot triggered when the baud reset button is pressed.  The signal sends the
    " button's text, which is used as a ID.
    """
    def auxBtnSlot(self, text):
        # Find publisher matching the text, and publish the command
        self.auxPub.publish(str(self.auxBtn.text()))


"""
" Container class for a PyQtGraph's PlotItem and some QT boilerplate code.
" It also contains a subscriber to the topic which is generating the data
" which it will plot.
"""
class Plot():
    def __init__(self, plotItem, topic, curves=1):

        self.tooltip = topic                # name of the field this plot graphs
        self.data = []                      # list of data values to be graphed
        self.curves = curves
        if self.curves == 2:
            self.data2 = []
        self.plotItem = plotItem            # PyQtGraph PlotItem object
        self.plotItem.hideAxis('left')
        self.plotItem.hideAxis('bottom')
        self.plotItem.hideButtons()

        # self.plotItem.disableAutoRange()
        # Set the vertical scale of the plots
        if self.tooltip != 'battery_voltage':
            self.plotItem.setRange(None, None, (-0.025, 1.0))
        else:
            self.plotItem.setRange(None, None, (-0.25, 100.0))

        for i in range(WizardParams.NUM_PLOT_POINTS):  # length of data?
            #self.data.append(np.random.randint(0, 2))
            self.data.append(1)
        self.curve = self.plotItem.plot(self.data, pen=pg.mkPen(WizardParams.CURVE_COLOR), width=2)  # black, no transparency
        self.ptr1 = 0  # Um... TODO: figure out what exactly this is (axis?)
        if self.curves == 2:
            for i in range(WizardParams.NUM_PLOT_POINTS):  # length of data?
                #self.data2.append(np.random.randint(0, 2))
                self.data2.append(1)
            self.curve2 = self.plotItem.plot(self.data2, pen=pg.mkPen(WizardParams.CURVE_2_COLOR), width=2)  # red, no transparency


    """
    " Update the graph.  This is called whenever the callback is triggered.
    " @param num The new element to be inserted into the array and plotted
    """
    def update(self):
        self.curve.setData(self.data)
        self.curve.setPos(self.ptr1, 0)
        if self.curves == 2:
            self.curve2.setData(self.data2)
            self.curve2.setPos(self.ptr1, 0)


    """
    " Callback to get the data to graph
    " @param msg The ROS message
    """
    def callback(self, msg):
        if self.tooltip in msg:
            self.data[:-1] = self.data[1:]  # shift data in the array one sample left (see also: np.roll)
            self.data[-1] = msg[self.tooltip]
            self.ptr1 += 1
        #print "ptr1: ", self.ptr1

            
    def app_callback(self, msg):
        if (self.tooltip) in msg and self.curves == 2:
            self.data2[:-1] = self.data2[1:]  # shift data in the array one sample left (see also: np.roll)
            self.data2[-1] = msg[(self.tooltip)] #FIXME ???
            self.ptr1 += 1
