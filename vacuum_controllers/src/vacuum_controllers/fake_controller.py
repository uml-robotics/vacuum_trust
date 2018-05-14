import sys
from robot_controller import BaseRobotController
from vacuum_experiment_msgs.experiment_state import ExperimentState
from vacuum_experiment_msgs.msg import Telemetry
from std_msgs.msg import *
from vacuum_interfaces.timer import Timer
import time
from lagerlogger import LagerLogger
from progressions import ConversationStack

def test(text):
    print text

fake_telemetry = {"lifted": True,
                  "charging": False,
                  "docked": True,
                  "dustbin": False,
                  "cleaning": True,
                  "buttons": ["power"]}

class RobotController(BaseRobotController):
    def __init__(self, config):
        BaseRobotController.__init__(self, config)
        self.logger = LagerLogger("")
        self.logger.console(LagerLogger.DEBUG)
        self.conv_stack = ConversationStack()
        self.telemetry_pub = self.ros.Publisher("%s/telemetry" % self.robot_name, Telemetry, queue_size=10)

        if config.has("stage","enabled") and config["stage"]["enabled"] == "True":
            self.logger.info("Using Stage")
            from vacuum_interfaces.stage import StageInterface
            self.robot = StageInterface(None, config)
        else:
            self.logger.debug("Using Hardware")
            from vacuum_interfaces.fake_interface import FakeInterface
            self.robot = FakeInterface(self.config)
            self.robot.set_telemetry_cb(self.telemetry_cb)
        self.phone_link.set_visibility(False)

        m = self.conv_stack.add_message("This robot has been disabled due to problems detect in the onboard electronics. We apologize for the inconvenience.")
        m.add_response("safe", lambda: self.test_reply("safe"))
        m.add_response("dangerous", lambda: self.test_reply("dangerous"))
        m.add_response("help", lambda: self.test_reply("help"))
        m.add_response("off", lambda: self.test_reply("off"))
        self.phone_link.set_progression(self.conv_stack)

        self._telemetry_timer = Timer(1, lambda x=fake_telemetry: self.telemetry_cb(x))
        self._telemetry_timer.start()


    def initialize(self):
        """ This is the only function executed explicitly by the robot controller """
        self.logger.info("Robot Initialized")
        self.phone_link.set_visibility(True) # Start showing the robot even when the experiment is not running
        self.phone_link.enable()
        self.ros.enable()


    def experiment_init(self):
        """ Callbed by robot controller after a new config has been loaded """
        self.logger.info("%s Loaded Behavior Profile: %s" % (self.robot_name, self.behavior_profile))

#        if self.behavior_profile.is_dead():
#            # Queue up the state for this robot, but don't show it yet
#            self.phone_link.enable()
#            self.phone_link.set_state("off")


    def experiment_state_standby(self):
        self.reset_controller()
    def experiment_state_ready(self): pass
    def experiment_state_running(self):
        # If we are in ready state, start things off!
        if self.controller_state.is_ready():
            self.start_run()
        elif self.controller_state.is_paused():
            self.controller_state.set_running()
    def experiment_state_paused(self): 
        self.phone_link.set_visibility(True)
        self.controller_state.set_paused()
    def experiment_state_complete(self):
        self.phone_link.set_visibility(False)
        self.controller_state.set_paused()

    def shutdown(self):
        """ Runs when the robot controller is shutting down """
#         self.charging_timer.cancel()
#         time.sleep(.25)
#         self.robot.pause()

        self._telemetry_timer.cancel()
        time.sleep(.25)
        self.ros.shutdown()
        self.robot.terminate()
        self.phone_link.disable()

################################################################################
    def reset_controller(self):
        self.logger.warn("Resetting controller")
        self.phone_link.set_state("safe")
        self.phone_link.set_visibility(False)
        self.behavior_profile.unset()
        self.controller_state.set_standby()

    def start_run(self):
        self.logger.info("starting run")
        self.phone_link.set_visibility(True)
        
    def test_reply(self, state):
        self.phone_link.set_state(state)

    def clean(self):
        self.logger.warn("CLEAN?")

    def telemetry_cb(self, values):
        """ Sends telemetry information """
        self.logger.debug("telemetry")
        t = Telemetry()
        t.lifted = values["lifted"]
        t.charging = values["charging"]
        t.docked = values["docked"]
        t.dustbin = values["dustbin"]
        t.cleaning = values["cleaning"]
        t.buttons = values["buttons"]
        self.telemetry_pub.publish(t)
        self.logger.debug(str(t))

 
