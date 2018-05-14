#!/usr/bin/env python

#pylint: disable=multiple-statements,trailing-whitespace,too-many-public-methods,too-few-public-methods
"""
Loads a robot controller from a file specified by robot.controller_file that 
implements a class called "RobotController".
"""
import rospy
import argparse
import os
import importlib
import imp
import inspect
import sys
import time
from reliable_rospy.reliable_node import ReliableNode
from std_msgs.msg import String, Bool
from vacuum_experiment_msgs.msg import Configuration
from vacuum_experiment_msgs.experiment_state import ExperimentState
# from behavior_profile import BehaviorProfile
from vacuum_experiment_msgs.behavior_profile import BehaviorProfile
from vacuum_experiment_msgs.controller_state import ControllerState
from phone_link import PhoneLink, FakePhoneLink
from progressions import ConversationStack
from lagerlogger import LagerLogger
import logging
# import hanging_threads
sys.dont_write_bytecode = True

class RobotConfig(dict):
    """ Loads arbitrary settings from a config file and from the command line.
    Settings are stored as config['section']['setting'] = value
    """
#     All names are automatically converted to lower case.
    def __init__(self, *args):
        dict.__init__(self, args)
        self.logger = LagerLogger("RobotConfig")
        self.logger.console(LagerLogger.DEBUG)

    def _add_setting(self, section, setting, value):
        """ Adds a setting to the RobotConfig """
#         section = section.lower()
        if section not in self.keys():
            self[section] = dict()
        self.logger.debug("Adding %s.%s: %s" % (section, setting, value))
        self[section][setting] = value

    def parse_config_file(self, filename):
        """ Load arbitrary settings from config file"""
        import ConfigParser
        self.cfg_file = ConfigParser.ConfigParser()
        self.cfg_file.optionxform = str  # Make options case sensitive
        path = os.path.dirname(os.path.realpath(__file__))
        path = os.path.join(path, filename)
        self.logger.debug("config path=%s"% path)
        try:
            self.cfg_file.readfp(open(path))
        except IOError as e:
            self.logger.error("Unable to load config file '%s'" % path)
            exit(0) 

        for section in self.cfg_file.sections():
            settings = self.cfg_file.options(section)
            for setting in settings:
                self._add_setting(section, setting, self.cfg_file.get(section, setting))

    def parse_args(self, unknown_args):
        """ Parse arbitrary command line args """
        arg_list = list()
        for arg in unknown_args:
            if arg.startswith(("-", "--")):
                if "." not in arg:
                    raise Exception("All arguments must have a '.' in their name, like 'Robot.setting'")
                arg_list.append(arg[2:])
                parser.add_argument(arg, type=str)
        opt_args = parser.parse_args(unknown_args)
        for arg in arg_list:
            section, setting = arg.split(".")
            self.logger.debug("Adding %s, %s from cmd line" % (section, setting))
            self._add_setting(section, setting, opt_args.__getattribute__(arg))

    def has(self, section, setting):
        """ test if the config includes a value for section setting """
        if section not in self.keys():
            return False
        if setting not in self[section].keys():
            return False
        return True

    def verify(self):
        """ Make sure we have at least the following settings"""
        if "robot" not in self.keys():
            raise Exception("No Section 'robot' in RobotConfig")
#         if "name" not in self["robot"]:
#             raise Exception("No robot.name specified in RobotConfig")
        if "controller_file" not in self['robot']:
            raise Exception("No robot.controller_file specified in RobotConfig")
#         if "ros_master_uri" not in self['robot']:
#             raise Exception("No robot.ros_master_uri specified in RobotConfig")
#         if "bluegigga_dev" not in self['robot']:
#             raise Exception("No robot.bluegigga_dev specified in RobotConfig")
        if "robot_dev" not in self['robot']:
            raise Exception("No robot.robot_dev specified in RobotConfig")
        
        
class Loggerator(object):
    def __init__(self, ros, config):
        self.logger = LagerLogger("Loggerator")
        self.logger.console(LagerLogger.DEBUG)
        self.ros = ros
        if not "logging" in config:
            self.logger.error("!!!!! NO LOGGING DEFINED !!!!!")
        self.publishers = dict()
        try:
            topics = [x.strip() for x in config["logging"]["topics"].split(",")]
            self.logger.info("Configuring logging on: %s" % topics)
            for topic in topics:
                try:
                    if config["logging"][topic] == "string":
                        topic_type = String
                    elif config["logging"][topic] == "bool":
                        topic_type = Bool
                    else:
                        topic_type = Bool
                except:
                    topic_type = Bool
                self.publishers[topic] = self.ros.Publisher("/logging/%s/%s" % (config["robot"]["name"], topic), topic_type, queue_size=10)
        except:
            self.logger.error("!!!!!!!!!!Error while parsing logging config!!!!!!!!!")

    def event(self, name, value=None):
        if value is None:
            value = True
        self.logger.debug("LOGGING: %s: %s" % (name,str(value)))
        self.publishers[name].publish(Bool(value))
 

class BaseRobotController(object):  #pylint: disable=abstract-class-not-used
    """ Manages ROS connection, experiment configuration, robot state, etc """
    def __init__(self, config):
        self.config = config
        self.robot_name = config['robot']['name']
        self.robot_model = config['bluegigga']['model']
        self.ros = ReliableNode(self.robot_name)

        # DEBUG LOGGING
        self.printer = LagerLogger("RobotBaseController")
        self.printer.console(LagerLogger.DEBUG)

        # ROS Logging
        self.logging = Loggerator(self.ros, config)

        # Robot Controller State
        self._controller_state_pub = self.ros.Publisher("/experiment/%s/state" % self.robot_name, String, queue_size=10)
        self._behavior_profile_pub = self.ros.Publisher("/experiment/%s/profile" % self.robot_name, String, queue_size=10)
        self._phone_support_pub = self.ros.Publisher("/experiment/%s/phone_support" % self.robot_name, Bool, queue_size=10)

        self.controller_state = ControllerState() # manual, ready, running
        self.controller_state.set_standby() 

        # Robot Behavior Experiment Profile
        self.behavior_profile = BehaviorProfile()
        self.behavior_profile.unset()

        # Startup bluetooth phone link
        if config.has('bluegigga', 'enabled') and config['bluegigga']['enabled'] == "True":
            self.printer.info("Using Bluetooth Hardware")
            self.phone_link = PhoneLink()
        else:
            self.printer.info("Faking bluetooth")
            self.phone_link = FakePhoneLink()
        self.conv_stack = ConversationStack()
        self.phone_link.set_name(self.robot_name)
        self.phone_link.set_model(config['bluegigga']['model'])
        self.phone_link.set_state("safe")
        self.phone_link.set_visibility(False)

        self.experiment_state = ExperimentState()
        # These need to come after the controller_state has been declared 
        self.ros.Subscriber('/experiment/config', Configuration, self._experiment_config_cb)
        self.ros.Subscriber('/experiment/state', String, self.experiment_state_cb)

        self._controller_state_timer = self.ros.ReliableTimer(0.5, self._controller_state_timer_cb)
        self._controller_state_timer.start()

    def initialize(self):
        """ Called once the controller's __init__ has been finished, to give
        the subclasses time to do things before firing up ros and the phone """
        self.ros.enable()
        self.phone_link.enable()

    def _controller_state_timer_cb(self):
        """ heatbeat that publishes the controller_state (overall state of controller) """
        self._controller_state_pub.publish(str(self.controller_state))
        self._behavior_profile_pub.publish(str(self.behavior_profile))
        self._phone_support_pub.publish(not self.phone_link.is_stealth())

    def experiment_state_cb(self, data):
        """ Receives experiment state from experiment controller """
        self.printer.info("Heard Experiment State: %s" % data.data)
        self.experiment_state.set(data.data)

        if self.experiment_state.is_standby():
            self.reset_controller()
        elif self.experiment_state.is_ready():
            pass  # Wait until we get a configuration
        elif self.experiment_state.is_running():
            # If we are in ready state, start things off!
            # Make sure we know what behavior to perform
            if not self.behavior_profile:
                self.printer.warn("Experiment Running, but still waiting for config")
                return
            if self.controller_state.is_standby() or self.controller_state.is_ready():
                self.start_run()
            elif self.controller_state.is_paused():
                self.controller_state.set_running()
        elif self.experiment_state.is_paused():
            if self.controller_state.is_standby() or self.controller_state.is_ready():
                self.start_run()
            self.controller_state.set_paused()
        elif self.experiment_state.is_complete():
#             self.phone_link.set_visibility(False)
#             self.controller_state.set_paused()
            self.reset_controller()

        # Do controller specific callbacks
        state_funs = {ExperimentState.STANDBY: self.experiment_state_standby,
                ExperimentState.READY: self.experiment_state_ready,
                ExperimentState.RUNNING: self.experiment_state_running,
                ExperimentState.PAUSED: self.experiment_state_paused,
                ExperimentState.COMPLETE: self.experiment_state_complete}
        state_funs[data.data]()

    def experiment_state_standby(self): pass
    def experiment_state_ready(self): pass
    def experiment_state_running(self): pass
    def experiment_state_paused(self): pass
    def experiment_state_complete(self): pass



    def _experiment_config_cb(self, config_msg):
        """ Receives the experiment configuration,
            calls user callback,
            configures the phone interface,
            sets controller state """
        # [db] this was causing a race condition when the controller would start up after experiment was running
#         if not (self.controller_state.is_standby() or self.controller_state.is_ready()):
        if (not (self.controller_state\
                or self.controller_state.is_standby()\
                or self.controller_state.is_ready() )\
                and self.behavior_profile):
            # We have received an experiment configuration at a strange time, ignore.
            self.printer.error("Ignoring received configuration while in state %s" % str(self.controller_state))
            return
        self.printer.info("Received Configuration: P=%d, C=%d, R=%d" % (config_msg.participant_id, config_msg.condition_id, config_msg.run_id))
        try:
            self.behavior_profile.get_robot_config(self.robot_name, config_msg.condition_id, config_msg.run_id) 
        except KeyError as e:
            self.printer.error("No Behavior Profile defined for '%s' in Cond '%d', Run '%d'" % (self.robot_name, config_msg.condition_id, config_msg.run_id))
            return

        self.logger.info("%s Loading Behavior Profile: %s" % (self.robot_name, self.behavior_profile))
    
        # Check phone support mode
        phone_support = {(1, 1): True,   (1, 2): False,
                         (2, 1): True,   (2, 2): False,
                         (3, 1): False,  (3, 2): True,
                         (4, 1): False,  (4, 2): True}
        if phone_support[(config_msg.condition_id, config_msg.run_id)]:
            self.phone_link.stealth_mode(False)
        else:
            self.phone_link.stealth_mode(True)

        # Initialize controller
#         self.experiment_init()
        if self.experiment_state:
            if self.experiment_state.get() in [ExperimentState.STANDBY, ExperimentState.READY]:
                self.controller_state.set_ready()
            elif self.experiment_state.is_running():
                self.logger.warn("Received configuration after experiment has started")
                self.logger.warn("Set to running")
                self.start_run()
            # If we are coming online while paused, start the robot and set to paused
            elif self.experiment_state.is_paused():
                self.logger.warn("Received configuration after experiment has started")
                self.logger.warn("Starting, then setting to pause")
                self.start_run()
                self.controller_state.set_paused()
            else:
                self.controller_state.set_standby()
        else:
            self.logger.warn("Received configuration, but no experiment state")





    def experiment_init(self):
        """ This is called after behavior_profile has been set! """
        raise NotImplementedError("this needs to be implemented!")

    def hw_pause(self):
        """ Make the robot stop moving """
        self.logger.debug("Pause called (no-op)")

    def reset_controller(self):
        self.hw_pause()
        # Enable the phone_link, but set it to invisible
        self.phone_link.set_state("safe")
        self.phone_link.stealth_mode(False)
        self.phone_link.set_visibility(False)
        self.phone_link.enable()
        # Reset the behavior profile, and put us into standby mode
        self.behavior_profile.unset()
        self.controller_state.set_standby()

    def shutdown(self):
        self.hw_pause()
        self.phone_link.set_visibility(False)
        self.ros.shutdown()
#         self.robot.terminate()
        time.sleep(2)
        self.phone_link.disable()

    def start_run(self):
        raise NotImplementedError()



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str)
    mainargs, others = parser.parse_known_args()

    rconfig = RobotConfig()
    # Load settings from config file
    if mainargs.config:
        rconfig.parse_config_file(mainargs.config)
    # Load command line args
    rconfig.parse_args(others)

    # Make sure we have all the mandatory settings
    rconfig.verify()

    # Setup ROS_MASTER_URI
    if "ros_master_uri" in rconfig["robot"]:
        os.environ['ROS_MASTER_URI'] = rconfig['robot']['ros_master_uri']

    # Load Robot Controller and pass RobotConfig into it
    logr = LagerLogger("main")
    logr.console(LagerLogger.DEBUG)
    logr.debug("=============================================================")
    logr.info(rconfig['robot']['controller_file'])
    cfg_file_path = os.path.dirname(os.path.realpath(__file__))
    cfg_file_path = os.path.join(cfg_file_path, rconfig['robot']['controller_file'])

    imp.load_source("rlib", cfg_file_path)
    from rlib import RobotController
    robotcontroller = RobotController(rconfig)
    print logging.Logger.manager.loggerDict.keys()
#     logging.getLogger('ReliableTopicConn').setLevel(LagerLogger.ERROR)
    if "loglevels" in rconfig:
        for name,level in rconfig["loglevels"].items():
            logr.debug("Setting log level for '%s' to '%s'" % (name,level))
            try:
                logging.getLogger(name).setLevel({"DEBUG": LagerLogger.DEBUG,
                                                "INFO": LagerLogger.INFO,
                                                "WARNING": LagerLogger.WARNING,
                                                "WARN": LagerLogger.WARN,
                                                "ERROR": LagerLogger.ERROR,
                                                "FATAL": LagerLogger.FATAL}[level])
            except:
                logr.warn("Unable to set log level '%s' for '%s'" % (level,name))

    robotcontroller.initialize()
#     logging.getLogger().setLevel(logging.DEBUG)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        robotcontroller.shutdown()
    logr.info("So long, and thanks for all the fish!\n===================================================================\n\n\n\n")

