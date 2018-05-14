import sys
from robot_controller import BaseRobotController
from vacuum_experiment_msgs.experiment_state import ExperimentState
from vacuum_experiment_msgs.enum_state import gen_enum_class
from vacuum_experiment_msgs.msg import Telemetry
# from vacuum_experiment_msgs.telemetry import TelemetryData
from std_msgs.msg import String
from vacuum_interfaces.timer import Timer
from lagerlogger import LagerLogger
from progressions import ConversationStack
import time
#      _                 _      _____  ___   ___  
#     (_)               | |    | ____|/ _ \ / _ \ 
#  ___ _ _ __ ___  _ __ | | ___| |__ | | | | | | |
# / __| | '_ ` _ \| '_ \| |/ _ \___ \| | | | | | |
# \__ \ | | | | | | |_) | |  __/___) | |_| | |_| |
# |___/_|_| |_| |_| .__/|_|\___|____/ \___/ \___/ 
#                 | |                             
#                 |_|                             
                                                          
class RobotController(BaseRobotController):
    def __init__(self, config):
        BaseRobotController.__init__(self, config)
        self.logger = LagerLogger("controller")
        self.logger.console(LagerLogger.DEBUG)
        self.conv_stack = ConversationStack()
        self.telemetry_pub = self.ros.Publisher("%s/telemetry" % self.robot_name, Telemetry, latch=True, queue_size=10)
        self.ros_cmd_sub = self.ros.Subscriber("%s/cmd" % self.robot_name, String, self.ros_cmd_cb)

        if config.has("stage","enabled") and config["stage"]["enabled"] == "True":
            self.logger.warn("using stage")
            from vacuum_interfaces.stage import StageInterface
            self.robot = StageInterface(None, config)
        else:
            self.logger.info("Using hardware")
            from vacuum_interfaces.roomba500_interface import Roomba500Interface
            self.robot = Roomba500Interface(self.config)
            self.robot.set_telemetry_cb(self.telemetry_cb)
        self.phone_link.set_visibility(False)

        self._telemetry_timer = Timer(5, lambda x=self.robot._telemetry: self.telemetry_cb(x))
        self._telemetry_timer.start()



        

    def initialize(self):
        """ This is the only function executed explicitly by the robot controller """
        self.robot.passive()
        self.logger.info("Robot Initialized")
        self.ros.enable()
        self.phone_link.enable()

    def experiment_init(self):
        """ Callbed by robot controller after a new config has been loaded """
        self.logger.info("%s Loading Behavior Profile: %s" % (self.robot_name, self.behavior_profile))

        if self.behavior_profile.is_disabled():
            self.phone_link.set_visibility(False)

        elif self.behavior_profile.is_easy():
            self.set_stack_idle()

        elif self.behavior_profile.is_help():
            self.phone_link.set_state("help")

        else:
            self.logger.error("Received unexpected behavior profile")
            return

        # Initialize controller
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



    def experiment_state_standby(self):
        self.reset_controller()
    def experiment_state_ready(self): pass
    def experiment_state_running(self):
        if not self.behavior_profile:
            self.logger.warn("Experiment Running, but still waiting for config")
            return
        # If we are in ready state, start things off!
        if self.controller_state.is_ready():
            self.controller_state.set_running()
            self.start_run()
        elif self.controller_state.is_paused():
            self.controller_state.set_running()
    def experiment_state_paused(self):
        self.controller_state.set_paused()
    def experiment_state_complete(self):
        self.phone_link.set_visibility(False)
        self.controller_state.set_paused()

    def shutdown(self):
        """ Runs when the robot controller is shutting down """
#         self.charging_timer.cancel()
        self.logger.warning("Shutting down controller")
        self._telemetry_timer.cancel()
        self.phone_link.set_visibility(False)
#         self.robot.pause()
        self.ros.shutdown()
        self.robot.terminate()
        time.sleep(2)
        self.phone_link.disable()

################################################################################
    def reset_controller(self):
        self.logger.warn("Resetting controller")

    def start_run(self):
        self.logger.info("starting run")

    def ros_cmd_cb(self, data):
        """ Receive commands from ROS """
        if data.data == "clean":
            self.logger.warn("ROS: clean()")
            self.robot.clean()
        elif data.data == "dock":
            self.logger.warn("ROS: dock()")
            self.robot.dock()
        elif data.data == "pause":
            self.logger.warn("ROS: pause()")
            self.robot.pause()



    def telemetry_cb(self, values):
        """ Sends telemetry information """
        t = Telemetry()
        t.battery_voltage = values["battery_voltage"]
        t.lifted = values["lifted"]
        t.charging = values["charging"]
        t.docked = values["docked"]
        t.dustbin = values["dustbin"]
        t.cleaning = values["cleaning"]
        t.buttons = values["buttons"]
        t.sci_error = values["sci_error"]
        t.api_mode = values["api_mode"]
        self.telemetry_pub.publish(t)


