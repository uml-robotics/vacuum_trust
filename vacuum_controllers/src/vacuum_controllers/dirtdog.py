from robot_controller import BaseRobotController
from vacuum_experiment_msgs.experiment_state import ExperimentState
from vacuum_experiment_msgs.enum_state import gen_enum_class
from std_msgs.msg import *
import time

#       _ _      _      _             
#      | (_)    | |    | |            
#    __| |_ _ __| |_ __| | ___   __ _ 
#   / _` | | '__| __/ _` |/ _ \ / _` |
#  | (_| | | |  | || (_| | (_) | (_| |
#   \__,_|_|_|   \__\__,_|\___/ \__, |
#                                __/ |
#                               |___/ 

class ActionState(gen_enum_class("ActionState", "cleaning", "docked", "docking", "cleaning_paused")):
    """ Dirtdog Behaviors """
    pass

   

class RobotController(BaseRobotController):
    def __init__(self, config):
        BaseRobotController.__init__(self, config)
        self.action_state = ActionState()
        self.action_state.set_cleaning_paused()

        if config.has("stage","enabled") and config["stage"]["enabled"] == "True":
            print "Using Stage"
            from vacuum_interfaces.stage import StageInterface
            self.robot = StageInterface(None, config)
        else:
            print "Using Hardware"
#             from vacuum_interfaces.roomba2 import RoombaInterface
            from vacuum_interfaces.dirtdog import DirtdogInterface
            self.robot = DirtdogInterface(self.config['robot']['robot_dev'])
            self.robot.set_buttons(power=self.power_button_cb)
            self.robot.set_wheeldrop_cb(wheeldrop_caster=self._caster_cb)
            self.robot.set_charging_cb(self.charging_cb)


    def experiment_init(self):
        print "%s Loaded Behavior Profile: %s" % (self.robot_name, self.behavior_profile)

    """
    Behaviors
        cleaning - wubbing about cleaning
        cleaning_paused - in the middle of cleaning, but currently stopped
        docking - heading back towards the dock
        docked - sitting on the charging station
    """
    def experiment_state_cb(self, data):
        print "Heard Experiment State: %s" % data.data
        # This should actually happen when we have the config loaded, not 
        # as a result of receiving the experiment state
        if data.data == ExperimentState.STANDBY:
            pass
        elif data.data == ExperimentState.READY:
            pass
        elif data.data == ExperimentState.RUNNING:
            if self.controller_state.is_offline():
                raise Exception("did not expect to be offline")
            elif self.controller_state.is_standby():
                raise Exception("did not expect to be standby")
            elif self.controller_state.is_manual():
                raise Exception("did not expect to be manual")
            elif self.controller_state.is_ready():
                #### BEGIN THE RUN
#                 self.start_run()
                self.controller_state.set_running()
            elif self.controller_state.is_running():
                print "Resuming Run"
                self.phone_link.set_state("dangerous")
#                 self.cmd_clean()
            else:
                raise Exception("Unknown controller state %s" % self.controller_state)

        elif data.data == ExperimentState.PAUSED:
            if self.controller_state.is_offline():
                raise Exception("did not expect to be offline")
            elif self.controller_state.is_standby():
                raise Exception("did not expect to be standby")
            elif self.controller_state.is_manual():
                raise Exception("did not expect to be manual")
            elif self.controller_state.is_ready():
                raise Exception("did not expect to be ready")
            elif self.controller_state.is_running():
                self.cmd_pause()

        elif data.data == ExperimentState.COMPLETE:
            if self.controller_state.is_offline():
                raise Exception("did not expect to be offline")
            elif self.controller_state.is_standby():
                raise Exception("did not expect to be standby")
            elif self.controller_state.is_manual():
                raise Exception("did not expect to be manual")
            elif self.controller_state.is_ready():
                raise Exception("did not expect to be ready")
            elif self.controller_state.is_running():
                self.cmd_pause()
                self.controller_state.set_standby()
                self.phone_link.set_state("off")
                self.phone_link.disable()
        else:
            raise Exception("Unknown experiment state %s" % data.data)

    def power_button_cb(self):
        self.logging.event("power_button")
        if not self.behavior_profile:
            print "Ignoring Button Press, no behavior profile loaded."
            return
        if self.behavior_profile.is_easy():
            if not self.action_state.is_cleaning():
                print "PWRBTN: Start Cleaning!"
                self.cmd_clean()
            else:
                print "PWRBTN: Pause"
                self.cmd_pause()
        elif self.behavior_profile.is_help():
            print "PWRBTN: TODO: Dirtbin!"
            self.cmd_pause()
        elif self.behavior_profile.is_dead():
            print "PWRBTN: TODO: Play Dead!"
            self.cmd_pause()
        else:
            print "PWRBTN: Robot is disabled"
            self.cmd_pause()
        print "current=",self.robot.robot.get_sensor("current")
        print "charging=",self.robot.robot.get_sensor("charging_state")

    def charging_cb(self, value):
        self.logging.event("charging_state", value)


#         if self.robot.is_cleaning():
#             print "Robot is cleaning!"
#         else:
#             print "Robot stop cleaning!"

    def _caster_cb(self, dropped):
        self.logging.event("lifted",dropped)
        print "Caster", "dropped" if dropped else "up"
        """ if the caster was down and comes back up, go to safe mode """
        if not dropped:
            if self.action_state.is_cleaning():
                print "END CLEANING (Reason: Picked up)"
                self.cmd_pause()

    def initialize(self):
        print "Initializing"
        time.sleep(1.25)
#         self.robot.robot.get_sensors()
#         self.init_state()
        self.action_state.set_cleaning_paused()
        print "UP!"

    def shutdown(self):
        self.robot.pause()
        self.ros.shutdown()
        self.robot.terminate()
        self.phone_link.disable()

    def start_run(self):
        if self.behavior_profile.is_disabled():
            print "Ignoring start command"
            return
        elif self.behavior_profile.is_easy() or self.behavior_profile.is_help():
            self.controller_state.set_running()
            self.phone_link.enable()
            self.cmd_clean()



#     def init_state(self):
#         """ Figure out what state you are in to initialize to """
#         if self.action_state:
#             raise Exception("Already Initialized state")
#         if self.robot.is_docked():
#             print "INITIALIZED ROBOT AS DOCKED"
#             self.action_state.set_docked()
#             self.phone_link.set_state("help")
#             return
#         # The robot is not docked, so we must be in another state.
#         if self.robot.is_cleaning():
#             print "INITIALIZING ROBOT TO CLEANING!?!?"
#             self.action_state.set_cleaning()
#             self.phone_link.set_state("dangerous")
#             return
#         
#         self.action_state.set_cleaning_paused()
#         self.phone_link.set_state("safe")
#         return


    def cmd_dock(self):
        """ Send the robot to its docking station """
        print "DIRTDOG_CONTROLLER: DOCK"
        if not self.action_state:
            raise Exception("Robot not initialized yet")
        if self.action_state.set_docked():
            print "Robot already docked"
            return
        
        self.robot.dock()
        self.phone_link.set_state("help")
        self.action_state.set_docking()
        return

    def cmd_pause(self):
        """ Tell the robot to pause what it is doing """
        print "DIRTDOG_CONTROLLER: PAUSE"
        self.robot.pause()
        self.phone_link.set_state("safe")
        self.action_state.set_cleaning_paused()
        print "TODO: phone progression"
        return


    def cmd_clean(self):
        """ Tell the robot to clean the floor """
        print "DIRTDOG_CONTROLLER: CLEAN"
        if not self.action_state:
            raise Exception("Robot not initialized yet")
        if self.action_state.is_cleaning():
            print "Robot already cleaning"
            return

        self.robot.clean()
        self.phone_link.set_state("dangerous")
        self.action_state.set_cleaning()
        return
 
