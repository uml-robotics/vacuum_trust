#!/usr/bin/env python

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import sys
import os, platform
import numpy as np
from socket import error as socket_error
import socket
import subprocess as sp
import threading
import xmlrpclib

import re


def ping_parse(ping_output):
    """ PingParser - parse ping output string into a dictionary
    Based on code from  https://github.com/ssteinerx/pingparser commit 80d7185
    Reorganized by [db] """
    host_matcher = re.compile(r'PING ([a-zA-Z0-9.\-]+) *\(')
    rslt_matcher = re.compile(r'(\d+) packets transmitted, (\d+) (?:packets )?received, (\d+\.?\d*)% packet loss')
    # Pull out round-trip min/avg/max/stddev = 49.042/49.042/49.042/0.000 ms
    minmax_matcher = re.compile(r'(\d+.\d+)/(\d+.\d+)/(\d+.\d+)/(\d+.\d+)')

    host = host_matcher.search(ping_output).groups()[0]
    sent, received, packet_loss = rslt_matcher.search(ping_output).groups()

    try:
        minping, avgping, maxping, jitter = minmax_matcher.search(ping_output).groups()
    except:
        minping = avgping = maxping = jitter = 'NaN'

    return {'host': host, 'sent': sent, 'received': received, 'packet_loss' : packet_loss,
            'minping': minping, 'avgping': avgping, 'maxping': maxping, 'jitter': jitter }


class Robot():
    def __init__(self, robotName, ipAddress, plotItemRef, penColor, dns=None):
        self._robotName = robotName
        self._ipAddress = ipAddress
        self._dns = robotName
        self._isSuccess = False
        self._timeout = 3
        self._data = []
        # HACK simulate data
        for i in range(100):  # length of data?
            #self._data.append(np.random.uniform(0.0, 2.0))
            self._data.append(0.0)
        self._curve = plotItemRef.plot(self._data, pen = pg.mkPen(penColor), width=2, name=robotName)
        self.ptr = 0


    """
    " Update UI
    """
    def updateUi(self):
        self._curve.setData(self._data)
        self._curve.setPos(self.ptr, 0)
        self.ping()

    """
    " Ping machine and update data ring buffer
    """

    def ping(self, n=1):
        """ Pings the host n times, returns output """
        socket.setdefaulttimeout(5)
        try:
            print "address: ", self._dns
            rslt = sp.check_output(("ping -c %d -W 1 %s" % (n, self._dns)).split(), stderr=sp.PIPE, universal_newlines=True)
            socket.setdefaulttimeout(self._timeout)
            pingStruct = ping_parse(rslt)
            #print "got ping data :) ", pingStruct
            #print "double?: ", pingStruct['avgping']

            self._data[:-1] = self._data[1:]  # shift data in the array one sample left (see also: np.roll)
            self._data[-1] = float(pingStruct['avgping'])

            self._isSuccess = True

        except sp.CalledProcessError as e:
            output = e.output
            self._data[:-1] = self._data[1:]  # shift data in the array one sample left (see also: np.roll)
            # DO NOT append new element
            print "ERROR: ", e.output

            self._isSuccess = False

        # FIXME: only the "master UI" should shift the graph
        self.ptr += 1