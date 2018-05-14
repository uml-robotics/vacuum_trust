#!/usr/bin/env python


# The problem is that rospy makes extensive use of global module variables, which
# are gross. Python does not have the functionality to "unload" a module, and 
# once rospy decides that ROS has been shutdown, there is no obvious way to undo
# that. Furthermore, there seems to be some lingering state that gets left behind
# after we loose a master.

import importlib
rospy = importlib.import_module("rospy")

import threading
import sys
from lagerlogger import LagerLogger


class ReliableTopicConn(object):
    """ This wraps topic connections (Publisher / Subscriber), and adds methods 
    for _start and _stop which can be called by a ReliableNode whenever ROSMaster
    comes or goes.
        """
    def __init__(self, topic_conn, *argv, **kwds):
        self.logr = LagerLogger("ReliableTopicConn")
        self.logr.console(LagerLogger.DEBUG)
        self.lock = threading.Lock()
        self._topic_conn = topic_conn
        self._argv = argv
        self._kwds = kwds
        self._conn = None
        self._name = argv[0]

    def _start(self):
        with self.lock:
            if self._conn:
                raise Exception("%s has already been initialized" % self._topic_conn.__name__)
            self.logr.debug("Starting %s %s" % (self._topic_conn.__name__, self._name))
            self._conn = self._topic_conn(*self._argv, **self._kwds)

    def _stop(self):
        if not self._conn:
            raise Exception("%s doesn't exist yet" % self._topic_conn.__name__)
        self.unregister()

    def get_num_connections(self):
        with self.lock:
            if not self._conn:
                raise Exception("%s not started" % self._topic_conn.__name__)
            return self._conn.get_num_connections()

    def unregister(self):
        with self.lock:
            self.logr.warn("Stopping %s %s" % (self._topic_conn.__name__, self._name))
            self._conn.unregister()
            self._conn = None


class ReliablePublisher(ReliableTopicConn):
    def __init__(self, *argv, **kwds):
        ReliableTopicConn.__init__(self, rospy.Publisher, *argv, **kwds)
        self.logr = LagerLogger("ReliablePublisher")
        self.logr.console(LagerLogger.DEBUG)

    def publish(self, *args, **kwds):
        with self.lock:
            if self._conn:
                self._conn.publish(*args, **kwds)
            else:
                self.logr.debug("ROS IS DOWN RIGHT NOW")
#             raise Exception("ROS IS DOWN RIGHT NOW")


class ReliableSubscriber(ReliableTopicConn):
    def __init__(self, *argv, **kwds):
        ReliableTopicConn.__init__(self, rospy.Subscriber, *argv, **kwds)





class ReliableNode(object):
    class Timer(object):
        """ Timer fires on an interval, but next cb does not fire until the 
        callback has completed. Can also be restarted without reinstantiating """
        def __init__(self, time, cb):
            self._time = time
            self._cb = cb
            self._timer = None
            self._running = True
            self._spin = True  # This is for the spin() function

        def start(self):
            self._running = True
            self._start_timer()

        def cancel(self):
            self._running = False
            self._stop_timer()

        def _start_timer(self):
            if not self._running:
                return
            if self._timer:
                raise Exception("Timer already started")
            self._timer = threading.Timer(self._time, self._cbfun)
            self._timer.start()

        def _stop_timer(self):
            if self._timer:
                self._timer.cancel()
            self._timer = None

        def _cbfun(self):
            self._stop_timer()
            self._cb()
            self._start_timer()

    def __init__(self, name, argv=None, anonymous=False, log_level=None, disable_rostime=False, disable_rosout=False, xmlrpc_port=0, tcpros_port=0):

        self.logr = LagerLogger("")
        self.logr.console(LagerLogger.DEBUG)
        self._enabled = False

        self._timer = ReliableNode.Timer(.1, self._timer_cb)
        self._name = name

        # State Information: Whether or not we are connected to the master
        self._state  = "disconnected"
        self._states = {"connected": self._state_connected,
                        "disconnected": self._state_disconnected
                        }

        # Topics to keep track of
        self._topic_connections = list()
        self._timer_connections = list()
        # Node settings
        self._settings = {"argv": argv, 
                            "anonymous": anonymous,
                            "log_level": log_level, 
                            "disable_rostime": disable_rostime, 
                            "disable_rosout": disable_rosout, 
                            "xmlrpc_port": xmlrpc_port,
                            "tcpros_port": tcpros_port}

        # Check for ROS Master, and connect if you see it
        # Start a timer so that we keep checking to make sure it is around.
        if self._check_connected():
            self._try_connect()
        self._timer.start()

    def enable(self):
        """ Allow node to connect to ROS if available """
        self.logr.info("Enabling RosNode")
        self._enabled = True

    def disable(self):
        """ Prevent node from connecting with ROS """
        self.logr.info("Disabling RosNode")
        self._enabled = False

    def _timer_cb(self):
        """ Timer callback to check whether or not the Master still exists """
        self._states[self._state]()

    def is_connected(self):
        if not self._state == "connected":
            return False
        return True

    def _check_connected(self):
        """ Checks to see if a roscore instance is running """
        if not self._enabled:
            return
        global rospy
        return rospy.core.rosgraph.is_master_online()

    def _try_connect(self):
        """ Attempt to establish a connection to ROS master """
        # The following three lines seem to work. Dont change them
        global rospy
        rospy = None
        rospy = importlib.import_module("rospy")
        self.logr.debug("Connecting to ROS")
        rospy.init_node(self._name, 
                        argv = self._settings["argv"],
                        anonymous=self._settings["anonymous"],
                        log_level=self._settings["log_level"],
                        disable_rostime = self._settings["disable_rostime"],
                        disable_rosout = self._settings["disable_rosout"],
                        disable_signals=True,
                        xmlrpc_port=self._settings["xmlrpc_port"],
                        tcpros_port=self._settings["tcpros_port"])
        for conn in self._topic_connections:
            conn._start()
        self._state = "connected"
        self.logr.info("Connected to ROS")


    def _state_disconnected(self):
        """ Disconnected State
        Check to see if a master is available, if so try to connect.
        """
        if not self._check_connected():
            return
        # We found a connection!
        self._try_connect()

    def _state_connected(self):
        if rospy.is_shutdown():
            #We are shutting down ROS. Close everything up and hibernate
            self.logr.warn("Heard ros shutdown call")
            self._shutdown_ros()
            self._state = "disconnected"
            return
        if not self._check_connected():
            # We have been disconnected
            self.logr.info("We have been disconnected")
            self._shutdown_ros()
            self._state = "disconnected"

    def _shutdown_ros(self):
        """ Shutdown all topics"""
        global rospy
        self.logr.warn("Shutting down ros topics")
        # Unregister Topics
        for conn in self._topic_connections:
            try:
                conn._stop()
            except Exception: pass
        rospy = None
        rospy = importlib.import_module("rospy")
        # The following command to signal shutdown should NOT be called.
        # We will never be able to undo it
        # rospy.core.signal_shutdown("reliable down!")

    def shutdown(self):
        """ Shutdown the reliable node """
        self._timer.cancel()
        self._shutdown_ros()
        for t in self._timer_connections:
            t.cancel()
        rospy.core.signal_shutdown("reliable down!")

    def Publisher(self, *argv, **kwds):
        """ replacement for rospy.Publisher() interface.
        ReliablePublishers "inherit" (not really) from rospy.Publisher(), but have a link back
        to the ReliableNode. They have the following characteristics
        - un-register themselves when the ROSMaster goes away or shutsdown
        - re-register themselves when the ROSMaster reappears
        - throw exceptions when you try to publish and ROSMaster is not around.
        """
        pub = ReliablePublisher(*argv, **kwds)
        self._topic_connections.append(pub)
        if self.is_connected():
            pub._start()
        return pub

    def Subscriber(self, *argv, **kwds):
        """ replacement call for rospy.Subscriber() """
        sub = ReliableSubscriber(*argv, **kwds)
        self._topic_connections.append(sub)
        if self.is_connected():
            sub._start()
        return sub

    def ReliableTimer(self, duration, callback):
        """ Makes a Timer that will be automatically killed when the process goes down"""
        t = ReliableNode.Timer(duration, callback)
        self._timer_connections.append(t)
        return t

    def spin(self):
        """ Blocks until keyboard interrupt or stop_spin() """
        self._spin = True
        try:
            while self._spin:
                time.sleep(0.5)
        except KeyboardInterrupt:
            pass

    def stop_spin(self):
        """ Can be used to programatically stop the spinning """
        self._spin = False


#TODO: rospy.loginfo! 
#TODO: ropsy.Rate
#TODO: rospy.Timer
#TODO: rospy.get_time()


