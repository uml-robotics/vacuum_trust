
from generic_vacuum import VacuumInterface
import copy
import sys
import time
import threading
import RPi.GPIO as GPIO
from gpio_sensor import gpio_sensor
from timer import Timer
import logging
from threading import Lock
from vacuum_experiment_msgs.telemetry import TelemetryData
from lagerlogger import LagerLogger

logr = LagerLogger("")
logr.console(LagerLogger.DEBUG)
#
#    /) ,                           
#  _(/    _   _  ____ _   _  __     
# (_(__(_/_)_(__(_) (/___(/_/ (_(_/_
#                              .-/  
#                             (_/   

DROPWHEEL=29
DUSTBIN = 16
POWER=33
SPOT=31
CLEAN=35
MAX=37
BASE=18

def gpio_sensor(pin, cb_fun):
    """ Register a callback whenever gpio pin changes state """
    def _cb(channel):
        if not channel == pin:
            logr.warn("heard back strange channel %d for cb pin %d" % (channel, pin))
        cb_fun(not bool(GPIO.input(pin)))
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(pin, GPIO.BOTH, callback=_cb)
    return


class DiscoveryInterface(VacuumInterface):
    def __init__(self, config):
        VacuumInterface.__init__(self)
        self.config = config
        self._callbacks = dict()
        self._telemetry = TelemetryData()

        #setup GPIO to reference pins based the board
        GPIO.setmode(GPIO.BOARD) 
        gpio_sensor(DROPWHEEL, self.gpio_wheel_cb)
        gpio_sensor(BASE, self.gpio_dock_cb)
        gpio_sensor(POWER, self.gpio_powerbtn_cb)
        gpio_sensor(DUSTBIN, self.gpio_dustbin_cb)
        gpio_sensor(SPOT, self.gpio_spot_cb)
        gpio_sensor(CLEAN, self.gpio_clean_cb)
        gpio_sensor(MAX, self.gpio_max_cb)

    def set_telemetry_cb(self, fun):
        self._telemetry.set_telemetry_update_cb(fun)

    def gpio_wheel_cb(self, value):
        value = not value
        if not value == self._telemetry["lifted"]:
            self._telemetry["lifted"] = value
        logr.debug("Wheel= %s" % str(value)) #(not bool(GPIO.input(BASE)))
        sys.stdout.flush()
 

    def gpio_dock_cb(self, value):
        if not value == self._telemetry["docked"]:
            self._telemetry["docked"] = value
        logr.debug("Dock= %s" % str(value)) #(not bool(GPIO.input(BASE)))
        sys.stdout.flush()
    

    def gpio_dustbin_cb(self, value):
        if not value == self._telemetry["dustbin"]:
            self._telemetry["dustbin"] = value
        logr.debug("dustbin= %s" % str(value))
        sys.stdout.flush()

    def gpio_powerbtn_cb(self, value):
        self.gpio_button_cb("power", value)

    def gpio_spot_cb(self, pressed):
        self.gpio_button_cb("spot", pressed)

    def gpio_clean_cb(self, value):
        self.gpio_button_cb("clean", value)

    def gpio_max_cb(self, value):
        self.gpio_button_cb("max", value)

    def gpio_button_cb(self, name, pressed):
        buttons = self._telemetry["buttons"]
        buttons = buttons if buttons else list()  # if buttons=None, make it a list
        if pressed and name not in buttons:
            buttons.append(name)
        if (not pressed) and name in buttons:
            buttons.remove(name)
        self._telemetry["buttons"] = buttons
        logr.debug("%s button=%s" % (name, str(pressed)))


    def reg_sensor_cb(self, sensor_name, cb, cmp_fun=None):
        if sensor_name not in self._callbacks.keys():
            self._callbacks[sensor_name] = list()
        if cmp_fun is None:
            cmp_fun = lambda old,new: not old.__eq__(new)
        if (cmp_fun, cb) in self._callbacks[sensor_name]:
            logr.warn("THIS HAS ALREADY BEEN DONE!?")
        self._callbacks[sensor_name] = (cmp_fun, cb)

    def is_docked(self):
        return self._dock

    def is_charging(self):
        return False

    def is_cleaning(self):
        return False

    def terminate(self):
        GPIO.cleanup()
        logr.info("Clean shutdown achieved - cheers!")
