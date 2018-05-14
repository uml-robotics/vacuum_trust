
from generic_vacuum import VacuumInterface
import time
import threading
import serial
import RPi.GPIO as GPIO
from timer import Timer
from advanced_roomba import AdvancedRoomba
from create_driver import DriverError, BadDataLengthError
import logging
# logging.basicConfig(level=logging.DEBUG)
# class HWState(gen_enum_class("RoombaHWState","init","safe","passive")): 
#     """ Enum State for tracking roomba's hw configuration """
#     pass
# 
# class RoombaHWState(HWState):
#     def __init__(self):
#         HWState.__init__(self)


DEVICE_DETECT = 7

class DirtdogInterface(VacuumInterface):
    def __init__(self, dev_path):
        VacuumInterface.__init__(self)
        self._dev_path = dev_path
        self.robot = AdvancedRoomba()
        self.robot.start(dev_path, 57600)
        self.robot.control()

        # Keep Alive Code
        #setup GPIO to reference pins based the board
        GPIO.setmode(GPIO.BOARD) 
        #device_detect pin is the pin we set low to turn on the robot, we must 
        #set it to out mode for rasing high and low
        GPIO.setup(DEVICE_DETECT, GPIO.OUT) 
        #keep alive thread
        self._keep_alive_timer = Timer(1, self._keep_alive)       
        self._keep_alive_timer.start()

        # Callback functions
        self.button_cbs = None
        self.wheeldrop_cbs = None
        self.charging_cbs = None

        self._sensor_update_timer = Timer(.1, self.poll_sensors_cb)       
        self._sensor_update_timer.start()

    def set_buttons(self, **kwargs):
        self.button_cbs=kwargs

    def set_wheeldrop_cb(self, **kwargs):
        self.wheeldrop_cbs=kwargs

    def set_charging_cb(self, *args):
        self.charging_cbs = args

    def is_docked(self):
        return self.is_charging()

    def is_charging(self):
        charge_state = self.robot.get_sensor("charging_state")
        if charge_state == "error":
            print "Something terrible has happened to charging"
            return False
        if charge_state ==  "not_charging":
            return False
        return True

    def is_cleaning(self):
        if self.robot.get_sensor("current") < -1000:
            return True
        return False

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
        old_buttons = self.robot.get_sensor("buttons")
        old_bumps_wheeldrops = self.robot.get_sensor("bumps_wheeldrops")
        try:
            old_charging_state = self.robot.get_sensor("charging_state")
        except:
            print "this fails the first time around"
            old_charging_state = "charging_error"
        try:
            self.robot.update_sensors()
        except BadDataLengthError as e:
            print e
            print "Restarting SCI into control() mode"
            self.robot.control()
            return
        except Exception as e:
            print e
            print "Restarting SCI into control() mode"
            self.robot.control()
            # WHEN THIS HAPPENS - the robot seems to have turned off.
            # On dirtdog this would happen when the person presses
            # the power button while cleaning.
            return

        new_buttons = self.robot.get_sensor("buttons")
        new_bumps_wheeldrops = self.robot.get_sensor("bumps_wheeldrops")
        new_charging_state = self.robot.get_sensor("charging_state")

        # Do button Callbacks
        for button in new_buttons.keys():
            if not old_buttons[button] and new_buttons[button]:
                self.button_cbs[button]()

        # Do wheeldrop callbacks
        for key,fun in self.wheeldrop_cbs.items():
            if not old_bumps_wheeldrops[key] == new_bumps_wheeldrops[key]:
                fun(new_bumps_wheeldrops[key])

        # Charging CBS
        if not old_charging_state == new_charging_state:
            for fun in self.charging_cbs:
                fun(new_charging_state)


    def terminate(self):
        self._keep_alive_timer.cancel()
        print("keep alive killed")
        self._sensor_update_timer.cancel()
        print("killed sensor update timer")
        time.sleep(1)
        GPIO.cleanup()
        self.robot.close()
        print "Clean shutdown achieved - cheers!" 
