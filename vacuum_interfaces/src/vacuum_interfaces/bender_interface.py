
from vacuum_interfaces.generic_vacuum import VacuumInterface
import time
from threading import Lock
import copy
import RPi.GPIO as GPIO
from vacuum_interfaces.gpio_sensor import gpio_sensor
from vacuum_interfaces.timer import Timer
from vacuum_interfaces.advanced_roomba import AdvancedRoomba
from create_driver import DriverError, BadDataLengthError
from vacuum_experiment_msgs.telemetry import TelemetryData
from lagerlogger import LagerLogger
#
#    /)           /)       
#   (/_  _ __   _(/  _  __ 
#  /_) _(/_/ (_(_(__(/_/ (_
#                          
                        



# class HWState(gen_enum_class("RoombaHWState","init","safe","passive")): 
#     """ Enum State for tracking roomba's hw configuration """
#     pass
# 
# class RoombaHWState(HWState):
#     def __init__(self):
#         HWState.__init__(self)


DEVICE_DETECT = 7
DUSTBIN = 16


class BenderInterface(VacuumInterface):
    def __init__(self, config):
        VacuumInterface.__init__(self)
        self.logr = LagerLogger("")
        self.logr.console(LagerLogger.DEBUG)
        self._dev_path = config['robot']['robot_dev']
        self.robot = AdvancedRoomba(config)
        self.robot.start(self._dev_path, 115200)
        self.robot.passive()

        self._telemetry = TelemetryData()

        # Keep Alive Code
        #setup GPIO to reference pins based the board
        GPIO.setmode(GPIO.BOARD) 
        #device_detect pin is the pin we set low to turn on the robot, we must 
        #set it to out mode for rasing high and low
        GPIO.setup(DEVICE_DETECT, GPIO.OUT) 

        #keep alive thread
        self._keep_alive_timer = Timer(1, self._keep_alive)       
        self._keep_alive_timer.start()

        # Dust-bin detection
        gpio_sensor(DUSTBIN, self.gpio_dustbin_cb)

        # Callback functions
        self._callbacks = dict()  # name=(cmp_fun, cb)
 
        self._sensor_update_timer = Timer(.1, self.poll_sensors_cb)       
        self._sensor_update_timer.start()


    def reg_sensor_cb(self, sensor_name, cb, cmp_fun=None):
        if sensor_name not in self._callbacks.keys():
            self._callbacks[sensor_name] = list()
        if cmp_fun is None:
            cmp_fun = lambda old,new: not old.__eq__(new)
        if (cmp_fun, cb) in self._callbacks[sensor_name]:
            self.logr.warn("THIS HAS ALREADY BEEN DONE!?")
        self._callbacks[sensor_name] = (cmp_fun, cb)

    def gpio_dustbin_cb(self, value):
        """ update the dustbin status """
        if not value == self._telemetry["dustbin"]:
            self._telemetry["dustbin"] = value

    def get_sensor(self, key):
        return self.robot.get_sensor(key)

    def passive(self):
        self.robot.passive()

    def is_docked(self):
        return self._telemetry["docked"]

    def is_charging(self):
        return self._telemetry["charging"]

    def is_cleaning(self):
        return self._telemetry["cleaning"]

    def dock(self):
        self.robot.dock()

    def clean(self):
        self.robot.clean()

    def pause(self):
        self.robot.pause()

    def _keep_alive(self):
        """ Keep alive timer callback. Throws a gpio pin up and down  """
        GPIO.output(DEVICE_DETECT, GPIO.LOW)
        time.sleep(0.1)
        GPIO.output(DEVICE_DETECT, GPIO.HIGH)

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
        except BadDataLengthError as e:
            self.logr.error(e)
#             print "Restarting SCI into control() mode"
#             self.robot.control()
            return
        except DriverError as e:
            self.logr.warn(e)
            self._telemetry["sci_error"] = True
        except Exception as e:
            self.logri.error(e)
#             print "Restarting SCI into control() mode"
#             self.robot.control()
            # WHEN THIS HAPPENS - the robot seems to have turned off.
            # On dirtdog this would happen when the person presses
            # the power button while cleaning.
            return

        # Update telemetry
        self._update_telemetry()

        # Do callbacks
        for sensor_name, (cmp_fun, cb) in self._callbacks.items():
            new_sensor_val = self.robot.get_sensor(sensor_name)
            if old_sensors[sensor_name] is None or cmp_fun(old_sensors[sensor_name], new_sensor_val):
                cb()


    def _update_telemetry(self):
        # If sci error has been set (by poll_sensors_cb), undo it
        if self._telemetry["sci_error"]:
            self._telemetry["sci_error"] = False
        # Compare charging status
        charging = self.robot.get_sensor("charging_state")
        charging = False if charging in ["not_charging", "error"] else True
        if not charging == self._telemetry["charging"]:
            self._telemetry["charging"] = charging
        # See if we are on the dock
        sources = self.robot.get_sensor("charging_sources_available")
        if not sources["base"] == self._telemetry["docked"]:
            self._telemetry["docked"] = sources["base"]
        # see if the robot has been picked up
        wheel_drops = self.robot.get_sensor("bumps_wheeldrops")
        lifted = (wheel_drops["wheeldrop_left"] or wheel_drops["wheeldrop_right"])
        if not lifted == self._telemetry["lifted"]:
            self._telemetry["lifted"] = lifted
        # Check if the sweeper brush is running (that means we are cleaning)
        cleaning = self.robot.get_sensor("main_brush_current") >= 1
        if not cleaning == self._telemetry["cleaning"]:
            self._telemetry["cleaning"] = cleaning
        # Check the status of button presses
        buttons = self.robot.get_sensor("buttons")
        buttons = [key for key, pressed in buttons.items() if pressed]
        if not set(self._telemetry["buttons"]) == set(buttons):
            self._telemetry["buttons"] = buttons
        # Check the robot voltage
        voltage = self.robot.get_sensor("voltage")/1000.0
        if abs(voltage - self._telemetry["battery_voltage"]) >= 0.1 :
            self._telemetry["battery_voltage"] = voltage


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
