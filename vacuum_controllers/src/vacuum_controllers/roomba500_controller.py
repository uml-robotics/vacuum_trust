
#pylint: disable=multiple-statements,trailing-whitespace
import sys
from robot_controller import BaseRobotController
from std_msgs.msg import Bool
from vacuum_experiment_msgs.experiment_state import ExperimentState
from vacuum_experiment_msgs.enum_state import gen_enum_class
from vacuum_experiment_msgs.msg import Telemetry, PhoneReply
# from vacuum_experiment_msgs.telemetry import TelemetryData
from std_msgs.msg import String
from vacuum_interfaces.timer import Timer
from lagerlogger import LagerLogger
# from progressions import ConversationStack
import time
from behavior_manager import BehaviorStack,RobotBehavior
from threading import Lock
#                             _           _____  ___   ___  
#                            | |         | ____|/ _ \ / _ \ 
#   _ __ ___   ___  _ __ ___ | |__   __ _| |__ | | | | | | |
#  | '__/ _ \ / _ \| '_ ` _ \| '_ \ / _` |___ \| | | | | | |
#  | | | (_) | (_) | | | | | | |_) | (_| |___) | |_| | |_| |
#  |_|  \___/ \___/|_| |_| |_|_.__/ \__,_|____/ \___/ \___/ 
#                                                           
                                                          

class ResetBehavior(RobotBehavior):
    def __init__(self, controller):
        RobotBehavior.__init__(self, controller.behavior_stack)
        self.controller = controller
        self.controller.ros.Subscriber("/%s/fail/reset" % self.controller.robot_name, Bool, self._ros_ctrl)
        self.timer = None #Timer(7, self._timer_cb)
        # Repeats the warning sound every 15 seconds
        self.sound_timer = Timer(15, self._sound_timer_cb)
        # Flashes the ring light
        self.light_timer = Timer(1, self._light_timer_cb)
        self.ring_light_on = True

        self._last_buttons = None
        self._behavior_active = False

    def _init_timer(self):
        if self.timer:
            self.timer.cancel()
            self.timer = None
        self.timer = Timer(7, self._timer_cb)

    def start(self):
        if not self._behavior_active:
#             if self.timer:
#                 self.timer.cancel()
#                 self.timer = None
#                 self.timer = Timer(10, self._timer_cb)
            self._init_timer()
            self._behavior_active = True
            self.cmd_reset()
            self.controller.robot.robot.set_leds(["check"],255,255)
            self.controller.robot.robot.play_song(0)
            self.sound_timer.start()
            self.light_timer.start()
            RobotBehavior.start(self)
        else:
            self.logr.warn("Reset behavior already started")

    def cancel(self):
        self._behavior_active = False
        self.sound_timer.cancel()
        self.light_timer.cancel()
        RobotBehavior.cancel(self)
        self.controller.robot.pause()

    def _ros_ctrl(self, data):
        self.logr.debug("Heard Command from ROS: %s" % str(data.data))
        if data.data:
            self.start()
        else:
            self.cancel()

    def hw_buttons_cb(self, buttons):
        # Clean pressed down
        if ((not self._last_buttons) or (not (self._last_buttons["clean"] or self._last_buttons["spot"]))) and (buttons["clean"] or buttons["spot"]):
            self.logr.debug("Button Pressed, Starting Timer!")
            self.timer.start()
            self.light_timer.cancel()
            self.sound_timer.cancel()
            self.controller.robot.robot.set_leds(["check"],255,255)

        # Clean button lifted
        elif self._last_buttons and ((self._last_buttons["clean"] and not buttons["clean"]) or (self._last_buttons["spot"] and not buttons["spot"])):
            self.logr.debug("Button Released, Killing Timer!")
            self.timer.cancel()
            if not self._behavior_active:
                # End this behavior!
                self.cancel()
            else:
                try:
                    self.sound_timer.start()
                except:
                    self.sound_timer.cancel()
                    self.sound_timer.start()
                try:
                    self.light_timer.start()
                except:
                    self.light_timer.cancel()
                    self.light_timer.start()
        self._last_buttons = buttons

    def _timer_cb(self):
        """ This happens when the button has been held down for the correct length of time """
        self.logr.debug("Reset Achieved!")
        self.sound_timer.cancel()
        self.light_timer.cancel()
        if self._behavior_active:
            self.controller.robot.robot.play_song(2)
            self.controller.robot.robot.set_leds([],0,255)
            self.controller.phone_link.set_state("ok", update=False)
            if self.controller.conv_stack[-1].tag == "reset":
                self.controller.conv_stack[-1].set_selection("0")
            m = self.controller.conv_stack.add_message("Thank you for resetting me!")
            self.controller.phone_link.set_progression(self.controller.conv_stack)
            self._behavior_active = False
        self.timer._time = 1
#         self.controller.conv_stack.clear()
#         self.controller.conv_stack.add_message("Robot reset!")
#         self.controller.phone_link.set_state("ok", update=False)
#         self.controller.phone_link.set_progression(self.controller.conv_stack)
#         self.timer.cancel()

    def _sound_timer_cb(self):
            self.controller.robot.robot.play_song(0)

    def _light_timer_cb(self):
        if self.ring_light_on:
            self.controller.robot.robot.set_leds(["check"],255,0)
            self.ring_light_on = False
        else:
            self.controller.robot.robot.set_leds(["check"],255,255)
            self.ring_light_on = True


    def vacuum_state_cb(self, vacuum_state):
        self.cmd_reset()

    def cmd_clean(self):
        self.cmd_reset()

    def cmd_pause(self):
        self.cmd_reset()

    def cmd_dock(self):
        self.cmd_reset()

    def cmd_reset(self):
        self.controller.robot.control()
        if not self.controller.conv_stack or not self.controller.conv_stack[0].tag == "reset":
            self.controller.conv_stack.clear()
#             m = self.controller.conv_stack.add_message("Reset Required.<br/><br/>Please hold down the 'Clean' button for 10 seconds")
            m = self.controller.conv_stack.add_message("Hardware Reset Required. Could you please help me?<br/><br/>I need to be reset! Could you please hold down the 'Clean' button for 10 seconds.")
            m.tag = "reset"
            m.set_popup()
            m.add_response("Ok", self.reply_ok)
            m.add_response("Cancel", self.reply_cancel)
            self.controller.phone_link.set_state("help", update=False)
        self.controller.phone_link.set_progression(self.controller.conv_stack)

    def reply_ok(self):
        """ person replied 'ok' to request for help """
        if self.controller.conv_stack[-1].tag == "cancel":
            self.controller.conv_stack.pop()
        if self.controller.conv_stack[-1].tag == "reset":
            self.controller.conv_stack[-1].unset_popup()
            m = self.controller.conv_stack.add_message("Thank you! Waiting for reset...")
            m.tag = "ok"
        self.controller.phone_link.set_progression(self.controller.conv_stack)

    def reply_cancel(self):
        """ person replied 'cancel' to request for help """
        if self.controller.conv_stack[-1].tag == "ok":
            self.controller.conv_stack.pop()
        if self.controller.conv_stack[-1].tag == "reset":
            self.controller.conv_stack[-1].unset_popup()
        self.controller.phone_link.set_progression(self.controller.conv_stack)





class DustbinBehavior(RobotBehavior):
    def __init__(self, controller):
        RobotBehavior.__init__(self, controller.behavior_stack)
        self.controller = controller
        self.controller.ros.Subscriber("/%s/fail/dustbin" % self.controller.robot_name, Bool, self._ros_ctrl)
        # Repeats the warning sound every 15 seconds
        self.sound_timer = Timer(15, self._sound_timer_cb)
        # Flashes the ring light
        self.light_timer = Timer(1, self._light_timer_cb)
        self.debris_light_on = True
        self._active = False

    def _ros_ctrl(self, data):
        self.logr.debug("Heard Command from ROS: %s" % str(data.data))
        if data.data:
            self.start()
        else:
            self.cancel()

    def start(self):
        super(DustbinBehavior, self).start()
        if self.controller.robot.is_docked():
            self.begin_dustbin()
        elif not self.controller.robot.is_docking():
            self.logr.info("Telling robot to dock")
            self.controller.robot.dock()

    def cancel(self):
        self._active = False
        RobotBehavior.cancel(self)
        self.controller.robot.pause()
        self.sound_timer.cancel()
        self.light_timer.cancel()


    def begin_dustbin(self):
        """ called after the robot is on its dock, to start asking the user
        to empty the dustbin """
        if self._active:
            self.logr.warn("dustbin already begun!")
            return
        self._active = True
        if not self.controller.robot.is_docked():
            self.logr.error("oops")
        self.set_stack_idle_help()
        self.controller.robot.control()
        self.controller.robot.robot.set_leds(["debris"],128,255)
        self.controller.robot.robot.play_song(1)
        self.sound_timer.start()
        self.light_timer.start()




    def vacuum_state_cb(self, vacuum_state):
        if vacuum_state.is_cleaning():
            self.logr.warn("Intercepting Cleaning Activity, Telling robot to dock")
            self.controller.robot.pause()
        elif vacuum_state.is_idle():
            if self.controller.robot.is_lifted():
                self.begin_dustbin()
            else:
                self.controller.robot.dock()
        elif vacuum_state.is_docking():
            self.set_stack_docking()
#         elif vacuum_state.is_docked():
#             self.begin_dustbin()
#             self.controller.robot.control()
#             self.set_stack_idle_help()

    def docked_cb(self):

        if not self._active:
            self.begin_dustbin()
#         self.controller.robot.control()
#         self.set_stack_idle_help()

    def cmd_clean(self):
        self.logr.info("HW CLEAN BUTTON HEARD!")
#         self.controller.set_stack_cleaning(False)
#         self.cmd_dustbin()

    def cmd_dustbin(self):
        self.logr.debug("cmd_dustbin()")
        if self.controller.conv_stack and not self.controller.conv_stack[-1].tag in ["dustbin", "dustbin_response"]:
            m = self.controller.conv_stack.add_message("Cleaning Paused.<br/><br/>Please empty my dustbin.")
            m.tag = "dustbin"
            m.add_response("Ok", self.heard_ok)
            m.add_response("Cancel", self.controller.set_stack_idle)
            self.controller.phone_link.set_state("help", update=False)
            self.controller.phone_link.set_progression(self.controller.conv_stack)

    def heard_ok(self):
        self.logr.info("User agreed!")
        if not self.controller.conv_stack[-1].tag == "dustbin":
            self.logr.error("That last thing on the conversation stack was not the dustbin")
            self.controller.phone_link.set_progression(self.controller.conv_stack)
            return
        m = self.controller.conv_stack.add_message("Waiting for dustbin to be emptied.")
        m.tag = "dustbin_response"
        self.controller.phone_link.set_progression(self.controller.conv_stack)

    def dustbin_cb(self, detected):
        if not detected:
            self.logr.debug("dustbin removed")
            self.sound_timer.cancel()
            self.light_timer.cancel()
            self.controller.robot.robot.set_leds([],0,255)
            return
        self.logr.debug("dustbin replaced")
        self.controller.robot.robot.play_song(2)
        m = self.controller.conv_stack.add_message("Thank you! Begin Cleaning?")
        m.add_response("Ok", self.controller.cmd_clean)
        m.add_response("Cancel", self.controller.set_stack_idle)
        self.controller.phone_link.set_state("ok", update=False)
        self.controller.phone_link.set_progression(self.controller.conv_stack)
        self._active = False
        self.cancel()

    def set_stack_idle_help(self):
        self.logr.debug("set_stack_idle()")
        self.controller.conv_stack.clear()
        docked = self.controller.robot.is_docked()
        state = "Docked" if docked else "Idle"
        m = self.controller.conv_stack.add_message("%s.<br/><br/>Please empty my dustbin." % state)
        m.tag = "dustbin"
        m.add_response("Ok", self.heard_ok)
        m.add_response("Cancel", self.controller.set_stack_idle)
#         if not docked:
#             m.add_response("Dock", self.controller.behavior_stack.cmd_dock)
        self.controller.phone_link.set_state("help", update=False)
        self.controller.phone_link.set_progression(self.controller.conv_stack)

    def set_stack_docking(self):
        """ special docking message that prevents re-cleaning """
        self.logr.debug("set_stack_docking()")
        self.controller.conv_stack.clear()
        m = self.controller.conv_stack.add_message("Looking for base station...")
        self.controller.phone_link.set_state("ok", update=False)
        self.controller.phone_link.set_progression(self.controller.conv_stack)



    def _sound_timer_cb(self):
            self.controller.robot.robot.play_song(1)

    def _light_timer_cb(self):
        if self.debris_light_on:
            self.controller.robot.robot.set_leds(["debris"],64,128)
            self.debris_light_on = False
        else:
            self.controller.robot.robot.set_leds([],64,128)
            self.debris_light_on = True



class NormalBehavior(RobotBehavior):
    def __init__(self, controller):
        RobotBehavior.__init__(self, controller.behavior_stack)
        self.controller = controller
        self._cleaning_timer = Timer(120, self._cleaning_timeout)
        self._buttons_lock = Lock()
        self._last_buttons = list()

    def cancel(self):
        self._cleaning_timer.cancel()
        RobotBehavior.cancel(self)

    def vacuum_state_cb(self, vacuum_state):
        """ Listen to state from the robot and set the phone response """
        if vacuum_state.is_idle() or vacuum_state.is_docked():
            self.logr.debug("canceling the cleaning timer")
            self._cleaning_timer.cancel()
            self.controller.set_stack_idle()
        elif vacuum_state.is_cleaning():
            self.controller.set_stack_cleaning()
            self.logr.debug("starting the cleaning timer")
            self._cleaning_timer.start()
        elif vacuum_state.is_docking():
            self.logr.debug("canceling the cleaning timer")
            self._cleaning_timer.cancel()
            self.controller.set_stack_docking()

    def dustbin_cb(self, detected):
        pass

    def hw_buttons_cb(self, buttons):
        with self._buttons_lock:
            if (("clean" in buttons and "clean" not in self._last_buttons) or ("spot" in buttons and "spot" not in self._last_buttons)) and self.controller.robot.is_charging():
                self.logr.debug("hw press: making sure we leave the dock")
                import threading
                threading.Timer(1, self._check_clean_started).start()
            self._last_buttons = buttons

    def _cleaning_timeout(self):
        self.logr.info("Cleaning timeout hit!")
        if self.controller.behavior_profile.is_easy():
            self.controller.behavior_dustbin.start()
        else:
            self.controller.robot.dock()
        self.logr.debug("canceling the cleaning timer")
        self._cleaning_timer.cancel()

    def ros_cmd_cb(self, data):
        """ Receive commands from ROS """
        if data.data == "clean":
            self.logr.warn("ROS: clean()")
            self.cmd_clean()
        elif data.data == "dock":
            self.logr.warn("ROS: dock()")
            self.cmd_dock()

    def cmd_dock(self):
        """ Send the robot to its docking station """
        self.logr.info("dock()")
        # Do not call passive first. it stops a cleaning robot, which then will not dock
        self.controller.robot.dock()  # This will cause the idle stack to get set
        self.controller.set_stack_docking()

    def docked_cb(self):
        self.logr.info("Robot has docked")

    def cmd_pause(self):
        """ Tell the robot to pause what it is doing """
        self.logr.info("cmd_pause()")
        self.controller.robot.pause()

    def cmd_clean(self):
        """ Tell the robot to clean the floor """
        self.logr.info("clean()")
        self.controller.robot.passive()
        time.sleep(.50)
        self.controller.robot.clean()
        # Roomba600 fix for going quite while charging
        # The problem is we have to literally press the button twice, sometimes.
        # If we are charging, check a second later to see if we are not longer charging.
        # If the clean command was successful, we won't be charging. If it was
        # not successful, we will still be charging and need to press clean again.
        if self.controller.robot.is_charging():
            self.logr.debug("making sure we leave the dock")
            import threading
            threading.Timer(1, self._check_clean_started).start()

    def _check_clean_started(self):
        self.logr.debug("Checking that we left the dock!")
        if self.controller.robot.is_charging():
            self.logr.info("Robot is still charging, so pressing clean again!")
            self.controller.robot.clean(no_state_change=True)



class RobotController(BaseRobotController):
    def __init__(self, config):
        BaseRobotController.__init__(self, config)
        self.logger = LagerLogger("controller")
        self.logger.console(LagerLogger.DEBUG)

        self.telemetry_pub = self.ros.Publisher("%s/telemetry" % self.robot_name, Telemetry, latch=True, queue_size=10)
        self.phonebutton_pub = self.ros.Publisher("%s/phonereply" % self.robot_name, PhoneReply, latch=True, queue_size=10)

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
            self.robot.reg_sensor_cb("charging_sources_available", self.docked_cb, lambda old,new: (not old["base"] == new["base"]) and new["base"])
            self.robot.reg_dustbin_cb(self.dustbin_cb)
            self.robot.reg_buttons_cb(self.hw_buttons_cb)
            self.robot.reg_vacuum_state_cb(self.vacuum_state_cb)

        self._telemetry_timer = Timer(5, lambda x=self.robot._telemetry: self.telemetry_cb(x))
        self._telemetry_timer.start()

        self.behavior_stack = BehaviorStack()
        self.behavior_normal = NormalBehavior(self)
        self.behavior_dustbin = DustbinBehavior(self)
        self.behavior_reset = ResetBehavior(self)

        self.behavior_normal.start()


        

    def initialize(self):
        """ This is the only function executed explicitly by the robot controller """
        super(RobotController, self).initialize()
        self.robot.passive()
        self.logger.info("Robot Initialized")
	self.robot.robot.set_song(0, [(79,4),(78,4),(76,4),(62,16),(0,32),(91,8),(91,8),(91,8),(91,8),(91,8),(91,8),(91,8),(91,8)]) 
	self.robot.robot.set_song(1, [(79,4),(78,4),(76,4),(62,16),(0,32),(90,4),(91,4),(93,4),(0,4),(90,4),(91,4),(93,4)]) 
        self.robot.robot.set_song(2, [(84,4)])


    def shutdown(self):
        """ Runs when the robot controller is shutting down """
        self.logger.warning("Shutting down controller")
        super(RobotController, self).shutdown()
        self._telemetry_timer.cancel()
        if self.behavior_dustbin:
            self.behavior_dustbin.cancel()
        if self.behavior_reset:
            self.behavior_reset.cancel()
        self.behavior_normal.cancel()
        self.robot.terminate()

    def reset_controller(self):
        self.logger.warn("Resetting controller")
        if self.behavior_dustbin:
            self.behavior_dustbin.cancel()
        if self.behavior_reset:
            self.behavior_reset.cancel()
        self.behavior_normal.cancel()
        self.behavior_normal.start()
        super(RobotController, self).reset_controller()

    def start_run(self):
        self.logger.info("starting run")
        # We assume that the robot needs
        # [db] removed because it undoes the reset behavior on boot
#         self.robot.passive()

        if self.phone_link.is_stealth():
            # [db] this is a hack to work around the robotlink software showing
            # popups even when visibility is set to false
            self.logger.info("Shutting down bluetooth because we are in stealth")
            self.phone_link.disable()

        if self.behavior_profile.is_disabled():
            # phone link should already be set to invisible, this is redundant
            self.phone_link.set_visibility(False)
            self.robot.passive()
            self.phone_link.disable()
            self.logger.info("Ignoring start command")

        elif self.behavior_profile.is_easy():
            self.set_stack_idle()
            self.controller_state.set_running()
            self.robot.passive()
            self.phone_link.set_visibility(True)

        elif self.behavior_profile.is_help():
            self.behavior_reset.start()
            self.controller_state.set_running()
            self.phone_link.set_visibility(True)

        else:
            self.logger.error("Received unexpected behavior profile")
            return



################################################################################




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

    def vacuum_state_cb(self, vacuum_state):
        """ Listen to state from the robot and set the phone response """
        if self.controller_state.is_running() or self.controller_state.is_paused():
            self.behavior_stack.vacuum_state_cb(vacuum_state)

        # If the robot is supposed to be paused and gets jostled causing it to
        # turn on, make it stop
        elif self.controller_state.is_standby() or self.controller_state.is_ready():
            if vacuum_state.is_cleaning():
                self.hw_pause()

    def hw_buttons_cb(self, buttons):
        if self.controller_state.is_running() or self.controller_state.is_paused():
            self.behavior_stack.hw_buttons_cb(buttons)

    def dustbin_cb(self, detected):
        if self.controller_state.is_running() or self.controller_state.is_paused():
            self.behavior_stack.dustbin_cb(detected)

    def ros_cmd_cb(self, data):
        """ Receive commands from ROS """
        self.logger.debug("ros_cmd_cb(%s)" % data.data)
        if data.data == "sci":
            self.robot.keep_alive()
        elif data.data == "baud":
            self.robot.baud_fix()
        elif data.data == "play_song0":
            self.robot.robot.play_song(0)
        elif data.data == "play_song1":
            self.robot.robot.play_song(1)
#         elif data.data == "bump":
#             self.robot.robot.sci.wake()
        else:
            self.behavior_stack.ros_cmd_cb(data)

    def cmd_dock(self):
        """ Send the robot to its docking station """
        if self.controller_state.is_running() or self.controller_state.is_paused():
            self.pub_phone_buttons("dock")
            self.behavior_stack.cmd_dock()

    def docked_cb(self):
        if self.controller_state.is_running() or self.controller_state.is_paused():
            self.behavior_stack.docked_cb()

    def cmd_pause(self):
        """ Tell the robot to pause what it is doing - this is a behavior so what this means is up to interpertation"""
        if self.controller_state.is_running() or self.controller_state.is_paused():
            self.behavior_stack.cmd_pause()

    def hw_pause(self):
        """ Tell the robot hardware to stop moving """
        self.logger.debug("hw_pause()")
        self.robot.pause()

    def cmd_clean(self):
        """ Tell the robot to clean the floor """
        if self.controller_state.is_running() or self.controller_state.is_paused():
            self.pub_phone_buttons("clean")
            self.behavior_stack.cmd_clean()

    def cmd_noop(self):
        self.logger.debug("cmd_noop()")

    def pub_phone_buttons(self, button=None):
        data = PhoneReply()
        if button:
            data.buttons.append(button)
        self.phonebutton_pub.publish(data)
        data = None
        data = PhoneReply()
        self.phonebutton_pub.publish(data)


####################$$$$$$$$$$$$$$$$$$$$$$$$$$####################
    def set_stack_idle(self):
        self.logger.debug("set_stack_idle()")
        self.conv_stack.clear()
        docked = self.robot.is_docked()
        state = "Docked" if docked else "Idle"
        m = self.conv_stack.add_message("%s. Waiting for Job." % state)
        m.add_response("Clean", self.cmd_clean)
        if not docked:
            m.add_response("Dock", self.cmd_dock)
        self.phone_link.set_state("ok", update=False)
        self.phone_link.set_progression(self.conv_stack)

    def set_stack_cleaning(self, update=True):
        self.logger.debug("set_stack_cleaning()")
        self.conv_stack.clear()
        m = self.conv_stack.add_message("Received command: Cleaning")
        m.add_response("Dock", self.cmd_dock)
        m.add_response("Pause", self.cmd_pause)
        self.phone_link.set_state("safe", update=False)
        self.phone_link.set_progression(self.conv_stack, update=update)

    def set_stack_docking(self):
        self.logger.debug("set_stack_docking()")
        self.conv_stack.clear()
        m = self.conv_stack.add_message("Looking for base station...")
        m.add_response("Clean", self.cmd_clean)
        m.add_response("Pause", self.cmd_pause)
        self.phone_link.set_state("ok", update=False)
        self.phone_link.set_progression(self.conv_stack)


