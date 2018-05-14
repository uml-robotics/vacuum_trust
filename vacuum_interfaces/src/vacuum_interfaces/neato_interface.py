from vacuum_interfaces.generic_vacuum import VacuumInterface
import time
from threading import Lock
import copy
import RPi.GPIO as GPIO
from vacuum_interfaces.timer import Timer

from vacuum_experiment_msgs.telemetry import TelemetryData
from lagerlogger import LagerLogger
from neato_driver import NeatoDriver 

class NeatoInterface(VacuumInterface):
    def __init__(self, config):
        VacuumInterface.__init__(self)
        self.logr = LagerLogger("")
        self.logr.console(LagerLogger.DEBUG)
        self._dev_path = config['robot']['robot_dev']
        self.robot = NeatoDriver(self._dev_path)
        self.robot.passive()
        self._callbacks = dict()
        self.reg_sensor_cb("buttons", self._buttons_cb,
                lambda old,new: not set([k for k,v in old.items() if v]) == set([k for k,v in new.items() if v]))
        self._telemetry = TelemetryData()
        self._sensor_update_timer = Timer(.001, self.poll_sensors_cb)       
        self._sensor_update_timer.start()

    def reg_sensor_cb(self, sensor_name, cb, cmp_fun=None):
        if sensor_name not in self._callbacks.keys():
            self._callbacks[sensor_name] = list()
        if cmp_fun is None:
            cmp_fun = lambda old,new: not old.__eq__(new)
        if (cmp_fun, cb) in self._callbacks[sensor_name]:
            self.logr.warn("THIS HAS ALREADY BEEN DONE!?")
        self._callbacks[sensor_name] = (cmp_fun, cb)

    def poll_sensors_cb(self):
        """ Poll sensors from hardware and do callbacks"""
        old_sensors = dict()
        for sensor_name in self._callbacks.keys():
            try:
                old_sensors[sensor_name] = self.robot.get_sensor(sensor_name)
            except:
                old_sensors[sensor_name] = None

        try:
            self.robot.update_sensors()
        except Exception as e:
            self.logr.error(e)
            return

        # Update telemetry
        self._update_telemetry()

        # Do callbacks
        for sensor_name, (cmp_fun, cb) in self._callbacks.items():
            new_sensor_val = self.robot.get_sensor(sensor_name)
            if old_sensors[sensor_name] is None or cmp_fun(old_sensors[sensor_name], new_sensor_val):
                cb()

    def get_sensor(self, key):
        return self.robot.get_sensor(key)

    def passive(self):
        self.robot.passive()

    def is_docked(self):
        #return bool(self.robot.get_sensor("ExtPwrPresent")) and not\
        #       bool(self.robot.get_sensor("SNSR_DC_JACK_CONNECT"))
        voltage = self.robot.get_sensor("VExtV")
        return (voltage > 20 and voltage < 23.5)

    def is_charging(self):
        #return bool(self.robot.get_sensor("ExtPwrPresent"))
        return (self.robot.get_sensor("VExtV") > 20)

    def is_cleaning(self):
        return False

    def dustbin_in(self):
        return bool(self.robot.get_sensor("SNSR_DUSTBIN_IS_IN"))

    def _update_telemetry(self):
        charging = self.is_charging()
        if not charging == self._telemetry["charging"]:
            self._telemetry["charging"] = charging
        # See if we are on the dock
        # sources = self.robot.get_sensor("charging_sources_available")
        docked = self.is_docked()
        if not docked == self._telemetry["docked"]:
            self._telemetry["docked"] = docked
        # see if the robot has been picked up

        wheel_drops = self.robot.get_sensor("bumps_wheeldrops")
        lifted = (wheel_drops["wheeldrop_left"] or wheel_drops["wheeldrop_right"])
        if not lifted == self._telemetry["lifted"]:
            self._telemetry["lifted"] = lifted

        dustbin = self.dustbin_in()
        if not dustbin == self._telemetry["dustbin"]:
            self._telemetry["dustbin"] = dustbin
        
        # Check if the sweeper brush is running (that means we are cleaning)
        cleaning = False
        if not cleaning == self._telemetry["cleaning"]:
            self._telemetry["cleaning"] = cleaning
        # Check the status of button presses

        buttons = self.robot.get_sensor("buttons")
        buttons = [key for key, pressed in buttons.items() if pressed]
        if not set(self._telemetry["buttons"]) == set(buttons):
            self._telemetry["buttons"] = buttons

        # Check the robot voltage
#         voltage = self.robot.get_sensor("BatteryVoltageInmV")
        voltage = self.robot.get_sensor("VBattV")
        if voltage is None:
            voltage = 0
        #voltage /= 1000
        if abs(voltage - self._telemetry["battery_voltage"]) >= .2: #40.0 :
            self._telemetry["battery_voltage"] = voltage

        

    def set_telemetry_cb(self, fun):
        self._telemetry.set_telemetry_update_cb(fun)

    def terminate(self):
        self._sensor_update_timer.cancel()
        time.sleep(.5)
        self.robot.close()
        self.logr.info("Clean shutdown achieved - cheers!" )

    def _buttons_cb(self):
	# Save the last button or buttons to be pressed.
        # Note: This gets called when buttons are released too, so make
        # sure something is actually pressed we didn't just release
        buttons = self.robot.get_sensor("buttons")
        new_buttons = [k for k,v in buttons.items() if v]
        if new_buttons:
            self._lastbuttons = new_buttons
        #print self._lastbuttons
