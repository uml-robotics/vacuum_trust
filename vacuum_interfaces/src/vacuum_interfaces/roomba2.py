
from generic_vacuum import VacuumInterface
import time
import threading
import serial
import RPi.GPIO as GPIO
from timer import Timer
from advanced_roomba import AdvancedRoomba
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

class RoombaInterface(VacuumInterface):
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

        self._sensor_update_timer = Timer(.1, self.robot.update_sensors)       
        self._sensor_update_timer.start()

    def set_buttons(self, **kwargs):
        self.robot.set_buttons(**kwargs)

    def set_wheeldrop_cb(self, **kwargs):
        self.robot.set_wheeldrop_cb(**kwargs)

    def is_docked(self):
        return self.is_charging()

    def is_charging(self):
        charge_state = self.robot.get_sensor("charging")
        if charge_state == "error":
            print "Something terrible has happened to charging"
            return False
        if charge_state ==  "not_charging":
            return False
        return True

    def is_cleaning(self):
        if self.get_current() < 1000:
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

    def terminate(self):
        self._keep_alive_timer.cancel()
        print("keep alive killed")
        self._sensor_update_timer.cancel()
        print("killed sensor update timer")
        time.sleep(1)
        GPIO.cleanup()
        self.robot.close()
        print "Clean shutdown achieved - cheers!" 
