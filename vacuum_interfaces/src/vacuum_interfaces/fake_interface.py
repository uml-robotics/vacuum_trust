
from generic_vacuum import VacuumInterface
import copy
import sys
import time
import threading
from timer import Timer
import logging
from threading import Lock
DROPWHEEL=29
DUSTBIN = 16
POWER=33
SPOT=31
CLEAN=35
MAX=37
BASE=18

# def gpio_sensor(pin, cb_fun):
#     """ Register a callback whenever gpio pin changes state """
#     def _cb(channel):
#         if not channel == pin:
#             print "heard back strange channel %d for cb pin %d" % (channel, pin)
#         cb_fun(not bool(GPIO.input(pin)))
#     GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
#     GPIO.add_event_detect(pin, GPIO.BOTH, callback=_cb)
#     return


class FakeInterface(VacuumInterface):
    def __init__(self, config):
        VacuumInterface.__init__(self)
        self.config = config
        self._callbacks = dict()
        self._telemetry_cb = None

        self._telemetry_lock = Lock()
        self._telemetry = {"charging": False,
                            "docked": False, 
                            "lifted": False,
                            "dustbin": True,
                            "cleaning": False,
                            "battery_voltage": -1,
                            "buttons": list()}

    def set_telemetry_cb(self, fun):
        self._telemetry_cb = fun

#    def gpio_wheel_cb(self, value):
#        value = not value
#        with self._telemetry_lock:
#            self._telemetry["lifted"] = value
#        if self._telemetry_cb:
#            tvalue = copy.copy(self._telemetry)
#            self._telemetry_cb(tvalue)
#        print "Wheel=", value #(not bool(GPIO.input(BASE)))
#        sys.stdout.flush()
# 
#
#    def gpio_dock_cb(self, value):
#        with self._telemetry_lock:
#            self._telemetry["docked"] = value
#        if self._telemetry_cb:
#            tvalue = copy.copy(self._telemetry)
#            self._telemetry_cb(tvalue)
#        print "Dock=", value #(not bool(GPIO.input(BASE)))
#        sys.stdout.flush()
#    
#
#    def gpio_dustbin_cb(self, value):
#        with self._telemetry_lock:
#            self._telemetry["dustbin"] = value
#        if self._telemetry_cb:
#            tvalue = copy.copy(self._telemetry)
#            self._telemetry_cb(tvalue)
#        print "dustbin=", value
#        sys.stdout.flush()
#
#    def gpio_powerbtn_cb(self, value):
#        self.gpio_button_cb("power", value)
#        print "Power=", value
#        sys.stdout.flush()
#
#    def gpio_spot_cb(self, pressed):
#        self.gpio_button_cb("spot", pressed)
#
#    def gpio_clean_cb(self, value):
#        self.gpio_button_cb("clean", value)
#
#    def gpio_max_cb(self, value):
#        self.gpio_button_cb("max", value)
#
#    def gpio_button_cb(self, name, pressed):
#        with self._telemetry_lock:
#            if pressed and name not in self._telemetry["buttons"]:
#                self._telemetry["buttons"].append(name)
#            if (not pressed) and name in self._telemetry["buttons"]:
#                self._telemetry["buttons"].remove(name)
#            tvalue = copy.copy(self._telemetry)
#        if self._telemetry_cb:
#            self._telemetry_cb(tvalue)
#        print "%s button=%s" % (name, str(pressed))


    def reg_sensor_cb(self, sensor_name, cb, cmp_fun=None):
        if sensor_name not in self._callbacks.keys():
            self._callbacks[sensor_name] = list()
        if cmp_fun is None:
            cmp_fun = lambda old,new: not old.__eq__(new)
        if (cmp_fun, cb) in self._callbacks[sensor_name]:
            print "THIS HAS ALREADY BEEN DONE!?"
        self._callbacks[sensor_name] = (cmp_fun, cb)

    def is_docked(self):
        return self._dock

    def is_charging(self):
        return False

    def is_cleaning(self):
        return False

#     def dock(self): pass
#     def clean(self): pass
#     def pause(self): pass

    def terminate(self):
        #GPIO.cleanup()
        print "Clean shutdown achieved - cheers!" 
