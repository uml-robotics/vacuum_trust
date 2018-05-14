
import sys
from vacuum_interfaces.generic_vacuum import VacuumInterface
import time
from threading import Thread, Lock
import copy
import RPi.GPIO as GPIO
from vacuum_interfaces.gpio_sensor import gpio_sensor
from vacuum_interfaces.timer import Timer
from vacuum_interfaces.advanced_roomba import AdvancedRoomba, SensorStoreException
from create_driver import SerialCommandInterface,DriverError, BadDataLengthError
from vacuum_experiment_msgs.telemetry import TelemetryData
from vacuum_experiment_msgs.enum_state import gen_enum_class
from vacuum_experiment_msgs.vacuum_state import VacuumState
from lagerlogger import LagerLogger
from spinner import Spinner
#                            ___    __     __  
#                   /)      /     /   )  /   ) 
#  __  _________   (/_ _   /__   /   /  /   /  
# / (_(_)(_) // (_/_) (_(_____) (__ /  (__ /   
#                                              
                                             

DEVICE_DETECT = 7
DUSTBIN = 16


class APIButtonPress(object):
    def __init__(self, button_name, button_tracker):
        self.button_name = button_name
        self.button_tracker = button_tracker
        self.logr = LagerLogger("APIButtonPress(%s)" % button_name)
        self.logr.console(LagerLogger.DEBUG)
        self._waiting_for_event = False
        self._lock = Lock()
        self._timeout = 40 # Should work out to be a full two seconds

    def __call__(self, button_fun):
        # Notify button tracker to give us the next instance of the following command
        if not self._lock.acquire(False):
            self.logr.fatal("Cant acquire lock!")
            return
        self._waiting_for_event = True
        try:
            self.button_tracker.consume_event(self.button_name, self.button_cb)
        except Exception as e:
            self.logr.error(e)
            self._lock.release()
            return
        # Press button over API
        button_fun()
        # Wait to consume the button press
        timeout = 0
        while self._waiting_for_event:
            if timeout >= self._timeout:
                self.logr.error("Timeout waiting to hear button")
                self._waiting_for_event = False
                self.button_tracker.cancel_consume(self.button_name)
                break
            timeout += 1
            time.sleep(.05)
        self._lock.release()

    def button_cb(self):
        self._waiting_for_event = False


class ButtonTracker(object):
    """ Tracks changes to button states, reports them to user callbacks. ALso used to remove API button presses 
    from robot data before their values ever get reported """
    def __init__(self):
        self._api_callbacks = dict()
        self._user_callbacks = list()
        self.logr = LagerLogger("ButtonTracker")
        self.logr.console(LagerLogger.DEBUG)
        self._lock = Lock()
        # This stores the values we will pass to callbacks
        self._button_dict = dict()
        # Cache what the last data looked like (e.g. ['clean','spot'])
        # This is different from _button_dict in that button dict can changed by removing values, while
        # this is a pure snapshot of the incoming data
        self._last_data = list()  

    def update(self, button_dict):
        """ called just after the hardware reports what buttons have been pressed, removes buttons that
        were 'pressed through the API' so that we only report hardware presses. """
        new_data = [k for k,v in button_dict.items() if v]
        changed = True if not set(new_data) == set(self._last_data) else False
        if not changed:
            return

        cb_value = None
        with self._lock:
            # Save the snapshot so we will know when it changes again, even if we modify the data.
            self._last_data = new_data
            for name,pressed in button_dict.items():
                if pressed and name in self._api_callbacks:
                    # Do callback and pretend button was never pressed
                    self._api_callbacks.pop(name)()
                    self.logr.debug("discarding anticapated button '%s'" % name)
                    pressed = False
                # Save the value of new or changed buttons
                if (name not in self._button_dict) or (not self._button_dict[name] == pressed):
                    self._button_dict[name] = pressed
            cb_value = copy.copy(self._button_dict)
        cb_threads = list()
        for cb in self._user_callbacks:
            cb_threads.append(Thread(target=cb,args=(cb_value,)))
            cb_threads[-1].start()
#             cb(cb_value)
        wait = 2.0
        for cb in cb_threads:
            cb.join(wait)
            if cb.is_alive():
                self.logr.warn("Long running hw button callback!")
#             else:
#                 self.logr.debug("dead")
#                 import traceback
#                 traceback.print_stack()
#                 sys.stdout.flush()
            wait = .1

    def get_buttons(self):
        with self._lock:
            return copy.copy(self._button_dict)

    def consume_event(self, button_name, cbfun):
        """ register a button name to consume, does so in the form of removing it from list to pass on and
        calling this associated callback (takes no arguments). Used by APIButtonCall """
        with self._lock:
            if button_name in self._api_callbacks:
                raise Exception("Button '%s' already registered!" % button_name)
            self._api_callbacks[button_name] = cbfun

    def cancel_consume(self, button_name):
        self.logr.debug("canceling consume event for '%s'" % button_name)
        with self._lock:
            if button_name in self._api_callbacks:
                self._api_callbacks.pop(button_name)
            else:
                self.logr.warn("no api_callback for '%s' for found" % button_name)

    def reg_update_callback(self, fun):
        """ register a user callback to call whenever a button gets pressed """
        self._user_callbacks.append(fun)

# class CleanButtonSafety(object):
#     """ Tracks when the clean button gets pressed to prevent baud being changed """
#     def __init__(self, interface):
#         self.logr = LagerLogger("CleanButtonTracker")
#         self.logr.console(LagerLogger.DEBUG)
#         self._interface = interface
#         self._last_value = None
#         self._timer = None
#         self._control_mode = False
# 
#     def set_value(self, pressed):
#         self.logr.warn("set_value(%s)" % str(pressed))
#         if not self._last_value and pressed:
#             oi_mode = self._interface.get_sensor("oi_mode")
#             if not oi_mode == "passive":
#                 self._last_value =  pressed
#                 return
#             self.logr.debug("Starting clean button timer")
#             self._timer = Timer(5.0, self._timer_cb)
#             self._timer.start()
#         elif self._last_value and not pressed:
#             if self._timer:
#                 self.logr.debug("canceling clean button timer")
#                 self._timer.cancel()
#                 self._timer = None
#             if self._control_mode:
#                 self.logr.debug("Restoring passive mode!")
#                 self._interface.pause()
#                 self._control_mode = False
#         self._last_value = pressed
# 
#     def _timer_cb(self):
#         self.logr.error("Clean button got held down too long! Mitigating the problem!")
#         self._control_mode = True
# #         self._interface.control()
#         self._interface.robot.sci.power()
#         self._timer.cancel()
# 


class Roomba500Interface(VacuumInterface):
    def __init__(self, config):
        VacuumInterface.__init__(self)
        self.logr = LagerLogger("")
        self._vacuum_state = VacuumState()
        self._vacuum_state.set_idle()
        self._spinner = Spinner()
        self.logr.console(LagerLogger.DEBUG)
        self._dev_path = config['robot']['robot_dev']

        # Tracks button presses to prevent API calls from slipping out
        self._button_tracker = ButtonTracker()  

        # Keep Alive Code
        #setup GPIO to reference pins based the board
        GPIO.setmode(GPIO.BOARD) 
       #device_detect pin is the pin we set low to turn on the robot, we must 
       #set it to out mode for rasing high and low
        GPIO.setup(DEVICE_DETECT, GPIO.OUT) 
        self._keep_alive()


        self.robot = AdvancedRoomba(config)
        self.robot.start(self._dev_path, 115200)
        self.robot.passive()

        self._telemetry = TelemetryData()
        self._button_tracker = ButtonTracker()
        self._api_button_clean = APIButtonPress("clean", self._button_tracker)
        self._api_button_dock = APIButtonPress("dock", self._button_tracker)

        #keep alive thread
        self._keep_alive_timer = Timer(60, self._keep_alive)
        self._keep_alive_timer.start()

        # Dust-bin detection
        gpio_sensor(DUSTBIN, self.gpio_dustbin_cb)
        self._dustbin_user_cb_funs = list()

        # Callback functions
        self._callbacks = dict()  # name=(cmp_fun, cb)
        self._cleaning_user_cb_funs = list()

        # Prevent Baud change
#         self._clean_button_safety = CleanButtonSafety(self)

        # Use button presses to track vacuum state
        self._button_tracker.reg_update_callback(self._hw_buttons_cb)
        self.reg_cleaning_cb(self._sweeper_cb)


        # Detect Docking
        self.reg_sensor_cb("charging_sources_available", self._docked_cb, lambda old,new: (not old["base"] == new["base"]))

        # Detect Lifts
        self.reg_sensor_cb("bumps_wheeldrops", self._lifted_cb, 
                lambda old,new: any([((not old[x]) and new[x]) for x in ["wheeldrop_right", "wheeldrop_left"]]) )
        self.reg_sensor_cb("bumps_wheeldrops", self._dropped_cb, 
                lambda old,new: any([((not new[x]) and old[x]) for x in ["wheeldrop_right", "wheeldrop_left"]]) )


        self._sensor_update_timer = Timer(.1, self.poll_sensors_cb)       
        self._sensor_update_timer.start()

    def reg_vacuum_state_cb(self, cb):
        """ register a callback for when the vacuum state changes """
        self._vacuum_state.callbacks.append(cb)

    def reg_buttons_cb(self, cb):
        self._button_tracker.reg_update_callback(cb)

    def reg_sensor_cb(self, sensor_name, cb, cmp_fun=None):
        if sensor_name == "buttons":
            self.logr.error("cannot log buttons using this callback - use dedicated reg_button_cb!")
            return
        if sensor_name not in self._callbacks.keys():
            self._callbacks[sensor_name] = list()
        if cmp_fun is None:
            cmp_fun = lambda old,new: not old.__eq__(new)
        if (cmp_fun, cb) in self._callbacks[sensor_name]:
            self.logr.warn("THIS HAS ALREADY BEEN DONE!?")
        self._callbacks[sensor_name].append((cmp_fun, cb))

    def reg_dustbin_cb(self, cb):
        self._dustbin_user_cb_funs.append(cb)

    def reg_cleaning_cb(self, cb):
        """" callbacks need to take an argument """
        self._cleaning_user_cb_funs.append(cb)

    def _docked_cb(self):
        base = self.robot.get_sensor("charging_sources_available")["base"]
        if base:
            self._vacuum_state.set_docked()

    def _lifted_cb(self):
        """ gets called each time a wheel leaves the ground """
        self.logr.debug("Lifted!")
        if not self._vacuum_state.is_idle():
            self._vacuum_state.set_idle()


    def _dropped_cb(self):
        """ gets called every time a wheel is set back down """
        self.logr.debug("droped!")
        if not self._vacuum_state.is_idle():
            self._vacuum_state.set_idle()



    def baud_fix(self):
#         self.robot.sci = SerialCommandInterface(self._dev_path, 19200)
        with self.robot.sci.lock:
            self.logr.warn("baud_fix()!")
            self._keep_alive()
            time.sleep(0.20)
            self.robot.sci.ser.baudrate = 19200
            self.robot.sci.start()
            time.sleep(0.20)
            try:
                self.robot.change_baud_rate(115200)
            except Exception as e:
                self.logr.error("change_baud_rate %s: %s" % (str(type(e)), e))
            try:
                self.robot.sci.start()
            except Exception as e:
                self.logr.error("start %s: %s" % (str(type(e)), e))
            time.sleep(.15)
            self.logr.warn("baud_fix() done.")
            sys.stdout.flush()


    def _hw_buttons_cb(self, values):
        """ called whenever a person presses one of the hardware buttons on the robot """
        buttons = [k for k,v in values.items() if v]
#         self._clean_button_safety.set_value(True if "clean" in buttons else False)
        # Dont do anything if we are just lifting up off a button
        # This isn't quite right, but is probably good enough
        if not buttons: 
            return
        self.logr.debug("HW Button Pressed: %s" % str(buttons))
        oi_mode = self.get_sensor("oi_mode")
        if not oi_mode == "passive":
            self.logr.debug("Not setting vacuum_state in control mode '%s'" % oi_mode)
            return
        if self._telemetry["lifted"]:
            self.logr.debug("Ignoring buttons because lifted")
            return
        if len(buttons) > 1:
            self.logr.error("Heard too many button states at once %s, auto cancelling." % str(buttons))
            self.robot.pause()
        if self._vacuum_state.is_idle() or self._vacuum_state.is_docked():
            if "clean" in buttons:
                self._vacuum_state.set_cleaning()
            elif "spot" in buttons:
                self._vacuum_state.set_cleaning()
                # [db] we don't seperately track spot cleaning anymore 
#                 self._vacuum_state.set_spot_cleaning()
            elif "dock" in buttons:
                self._vacuum_state.set_docking()
            # [db] this happens if "day" or "hour" gets pressed...
            else:
                self.logr.warn("Unhandled last button: %s" % str(buttons))
        else:
            if self.is_docked():
                self._vacuum_state.set_docked()
            else:
                self._vacuum_state.set_idle()




    def _sweeper_cb(self, cleaning):
        """ gets called when the sweeper either starts or stops """
        self.logr.debug("sweeper_cb(%s)" % str(cleaning))
        if not self._vacuum_state:
            self.logr.warn("Sweeper changed state, but vacuum_state is unknown")
            return
        vacuum_state = self._vacuum_state.get()
        if cleaning and (vacuum_state not in ["cleaning","spot_cleaning","docking"]):
            self.logr.error("Sweeper started, but was in state '%s'. Assuming Cleaning." % vacuum_state)
            self._vacuum_state.set_cleaning()
        elif not cleaning and (vacuum_state not in ["idle","docking","docked"]):
            if self._telemetry["docked"]:
                self._vacuum_state.set_docked()
            else:
                self._vacuum_state.set_idle()

    def gpio_dustbin_cb(self, value):
        """ update the dustbin status """
        if not value == self._telemetry["dustbin"]:
            self._telemetry["dustbin"] = value
            if self._dustbin_user_cb_funs:
                for cb in self._dustbin_user_cb_funs:
                    cb(value)

    def get_sensor(self, key):
        if key == "buttons":
            return self._button_tracker.get_buttons()
        return self.robot.get_sensor(key)

    def passive(self):
        # This gets called when the controller boots up, and just before the clean cmd
        # Do NOT set the vacuum state explicitly here, because it should get overwritten
        # moments later and will just cause problems. If you want vacuum_state set, use
        # pause() instead.
        self.logr.info("passive mode")
        self.robot.passive()

    def control(self):
        self.keep_alive()
        self.logr.info("control mode")
        self.robot.control()

    def is_docking(self):
        return self._vacuum_state.is_docking()

    def is_docked(self):
        return self._vacuum_state.is_docked()

#     def is_on_dock(self):
#         return self._telemetry["docked"]

    def is_lifted(self):
        drops = self.robot.get_sensor("bumps_wheeldrops")
        return (drops["wheeldrop_right"] or drops["wheeldrop_left"])

    def is_charging(self):
        return self._telemetry["charging"]

    def is_cleaning(self):
        return self._vacuum_state.is_cleaning() or self._vacuum_state.is_spot_cleaning()

    def dock(self):
        """ Sends robot to the dock. Send twice if cleaning, once if not cleaning"""
        if self.is_cleaning():
            self.logr.info("dock() {while cleaning}")
            self._vacuum_state.set_docking()
            self.robot.pause()
            time.sleep(0.5)
        else:
            self.logr.info("dock() {while paused}")
        self._api_button_dock(self.robot.dock)
        self._vacuum_state.set_docking()

    def clean(self, no_state_change=None):
        self.logr.info("clean()")
        if (self._vacuum_state.is_cleaning() or self._vacuum_state.is_spot_cleaning()) and no_state_change is None:
            self.logr.error("Already cleaning. Ignoring command")
            return
        self._api_button_clean(self.robot.clean)
        if no_state_change is None:
            self._vacuum_state.set_cleaning()

    def pause(self):
        self.logr.info("pause()")
        if self._telemetry["docked"]:
            self._vacuum_state.set_docked()
        else:
            self._vacuum_state.set_idle()
        self.robot.pause()

    def keep_alive(self):
        self._keep_alive()

    def _keep_alive(self):
        """ Keep alive timer callback. Throws a gpio pin up and down  """
        self.logr.debug("SCI Keep Alive")
        GPIO.output(DEVICE_DETECT, GPIO.LOW)
        time.sleep(0.1)
        GPIO.output(DEVICE_DETECT, GPIO.HIGH)

    def poll_sensors_cb(self):
        """ Poll sensors from hardware and do callbacks"""
        self._spinner.spin()
        old_sensors = dict()
        for sensor_name in self._callbacks.keys():
            try:
                old_sensors[sensor_name] = self.robot.get_sensor(sensor_name)
            except:
                old_sensors[sensor_name] = None
        
        try:
            self.robot.update_sensors()
        except BadDataLengthError as e:
            self.logr.error(e)
            return
        except DriverError as e:
            self.logr.warn(e)
            self._telemetry["sci_error"] = True
            self._keep_alive()
            return
        except Exception as e:
            self.logr.error(e)
            return

        # Update Buttons
        self._button_tracker.update(self.robot.get_sensor("buttons"))

        # Update telemetry
        self._update_telemetry()

        # Do callbacks
        for sensor_name, cb_list in self._callbacks.items():
            for (cmp_fun, cb) in cb_list:
                new_sensor_val = self.robot.get_sensor(sensor_name)
                if old_sensors[sensor_name] is None or cmp_fun(old_sensors[sensor_name], new_sensor_val):
                    cb()



    def _update_telemetry(self):
        # If sci error has been set (by poll_sensors_cb), undo it
        if self._telemetry["sci_error"]:
            self._telemetry["sci_error"] = False
        # Compare charging status
        try:
            charging = self.robot.get_sensor("charging_state")
            chargingbool = False if charging in ["waiting", "not_charging", "error"] else True
            if not chargingbool == self._telemetry["charging"]:
                self._telemetry["charging"] = chargingbool
                if charging == "error":
                    self.logr.error("Charging: Fault")
                elif charging == "waiting":
                    self.logr.warn("Charging: waiting")
                elif charging == "trickle_charging":
                    self.logr.info("Charging: trickle")
                elif charging == "charging":
                    self.logr.info("Charging: Charging")
                elif charging == "charge_recovery":
                    self.logr.info("Charging: Recovery")
                elif charging == "not_charging":
                    self.logr.info("Charging: Not Charging")
                else:
                    self.logr.error("Charging: Received unknown state '%s'" % str(charging))
        except SensorStoreException as e:
            self.logr.warn(e)
        except Exception as e:
            self.logr.warn(e)
            self.logr.warn("Not updating telemetry this time")
        # See if we are on the dock
        try:
            sources = self.robot.get_sensor("charging_sources_available")
            if not sources["base"] == self._telemetry["docked"]:
                self._telemetry["docked"] = sources["base"]
        except SensorStoreException as e:
            self.logr.warn(e)
        # see if the robot has been picked up
        try:
            wheel_drops = self.robot.get_sensor("bumps_wheeldrops")
            lifted = (wheel_drops["wheeldrop_left"] or wheel_drops["wheeldrop_right"])
            if not lifted == self._telemetry["lifted"]:
                self._telemetry["lifted"] = lifted
        except SensorStoreException as e:
            self.logr.warn(e)
        # Check if the sweeper brush is running (that means we are cleaning)

        try:
            # Eva has a lower current than the other robots - dips into 80s sometimes
            # so this is a good threshold, I think
            cleaning = self.robot.get_sensor("main_brush_current") >= 65
#             if cleaning:
#                 print self.robot.get_sensor("main_brush_current")
            if not cleaning == self._telemetry["cleaning"]:
                self._telemetry["cleaning"] = cleaning
                # Do cleaning cb if it is registered.
                if self._cleaning_user_cb_funs:
                    for cb in self._cleaning_user_cb_funs:
                        cb(cleaning)
        except SensorStoreException as e:
            self.logr.warn(e)
        # Check the status of button presses
        try:
            buttons = self._button_tracker.get_buttons()
            buttons = [key for key, pressed in buttons.items() if pressed]
            if not set(self._telemetry["buttons"]) == set(buttons):
                self._telemetry["buttons"] = buttons
        except SensorStoreException as e:
            self.logr.warn(e)
        # Check the robot voltage
        try:
            voltage = self.robot.get_sensor("voltage")/1000.0
            if abs(voltage - self._telemetry["battery_voltage"]) >= 0.1 :
                self._telemetry["battery_voltage"] = voltage
        except SensorStoreException as e:
            self.logr.warn(e)
        # Set api mode
        try:
            oi_mode = self.robot.get_sensor("oi_mode")
            if not oi_mode == self._telemetry["api_mode"]:
                self._telemetry["api_mode"] = oi_mode
        except SensorStoreException as e:
            self.logr.warn(e)


    def set_telemetry_cb(self, fun):
        self._telemetry.set_telemetry_update_cb(fun)

    def terminate(self):
        self._keep_alive_timer.cancel()
        self.logr.info("keep alive killed")
        self._sensor_update_timer.cancel()
        self.logr.info("killed sensor update timer")
        time.sleep(1)
        GPIO.cleanup()
        self.robot.close()
        self.logr.info("Clean shutdown achieved - cheers!" )
