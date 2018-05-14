import sys
from robot_controller import BaseRobotController
from vacuum_experiment_msgs.experiment_state import ExperimentState
from vacuum_experiment_msgs.msg import Telemetry
from std_msgs.msg import *
from vacuum_interfaces.timer import Timer
import time
from lagerlogger import LagerLogger
from progressions import ConversationStack
#                    _        
#                   | |       
#   _ __   ___  __ _| |_ ___  
#  | '_ \ / _ \/ _` | __/ _ \ 
#  | | | |  __/ (_| | || (_) |
#  |_| |_|\___|\__,_|\__\___/ 
                            
                            

class RobotController(BaseRobotController):
    def __init__(self, config):
        BaseRobotController.__init__(self, config)
        self.logger = LagerLogger()
        self.logger.console(LagerLogger.DEBUG)
        self.telemetry_pub = self.ros.Publisher("%s/telemetry" % self.robot_name, Telemetry, latch=True, queue_size=10)

        if config.has("stage","enabled") and config["stage"]["enabled"] == "True":
            self.logger.info("Using Stage")
            from vacuum_interfaces.stage import StageInterface
            self.robot = StageInterface(None, config)
        else:
            self.logger.debug("Using Hardware")
            from vacuum_interfaces.neato_interface import NeatoInterface
            self.robot = NeatoInterface(self.config)
            self.robot.set_telemetry_cb(self.telemetry_cb)

        m = self.conv_stack.add_message("This robot has been disabled due to problems"
                    " detected in the onboard electronics.<br/><br/>We apologize for the inconvenience.")
        self.phone_link.set_progression(self.conv_stack)


    def initialize(self):
        """ This is the only function executed explicitly by the robot controller """
        super(RobotController, self).initialize()
        self.logger.info("Robot Initialized")


    def shutdown(self):
        """ Runs when the robot controller is shutting down """
        self.logger.warning("Shutting down controller")
        super(RobotController, self).shutdown()
        self.robot.terminate()

    def reset_controller(self):
        self.logger.warn("Resetting controller")
        super(RobotController, self).reset_controller()

    def start_run(self):
        self.logger.info("starting run")
        # We assume that the robot needs
        if self.behavior_profile.is_disabled():
            self.phone_link.disable()
            self.logger.info("Ignoring start command")

        elif self.behavior_profile.is_dead():
            self.phone_link.set_state("off")
            self.phone_link.set_visibility(True)
            self.controller_state.set_running()

        else:
            self.logger.error("Received unexpected behavior profile")
            return

################################################################################

    def telemetry_cb(self, values):
        """ Sends telemetry information """
        t = Telemetry()
        t.lifted = values["lifted"]
        t.charging = values["charging"]
        t.docked = values["docked"]
        t.dustbin = values["dustbin"]
        t.cleaning = values["cleaning"]
        t.buttons = values["buttons"]
        t.battery_voltage = values["battery_voltage"]
        t.api_mode = "passive"
        self.telemetry_pub.publish(t)

