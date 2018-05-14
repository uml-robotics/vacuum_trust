from roomba import RoombaSharedInterface as BaseRI
from enum_state import gen_enum_class
import Queue
import serial
import RPi.GPIO as GPIO
import struct
import time
import threading
# from multiprocessing import Pool
from timer import Timer
#import numpy 
import sys

DEVICE_DETECT = 7
debug = False

class RoombaHWState(gen_enum_class("RoombaHWState","init","safe","passive")): pass




class RoombaDirtdogInterface(BaseRI):
    def __init__(self, device): 
        BaseRI.__init__(self,device, 57600)
        self.hwstate = RoombaHWState()
        self.hwstate.set_init()

#         self.robot.write(struct.pack('ii', 128, 130)) #enable robot 
#         self.hwstate.set_safemode()
    
	#self._poll_sensor_timer = Timer(0.15, self._poll_sensor_data_passer) 
        #self._poll_sensor_timer.start()
#         self.robot_init()
      
    def robot_init(self):
        """ Initialize the robot into safe (controlable) mode """
        if not self.hwstate.is_init():
            raise Exception("Trying to init the robot, but already in init")
        print "INITIALIZING ROBOT SERIAL"

        wait = True
        while wait:
            self.robot.write(struct.pack('ii', 128, 130)) #enable robot 
            time.sleep(.1)
            if self._sensor_data:
                wait = False
        self.hwstate.set_safe()
        print "Robot Initialized"



    def _poll_sensors(self):
        print "polled"
        sys.stdout.flush()
        with self._opcode_lock:
            try:
                self.robot.write(struct.pack('i', 142))
            except Error:
                print("Error occured while getting sensor data => " + str(Error)) 
        time.sleep(0.1) 
        try:
            with self._sensor_data_lock:
                if debug:
                    print "Num of bytes in buffer: " + str(self.robot.inWaiting())
                self._sensor_data = []
                if self.robot.inWaiting() > 26:
                    self.robot.read(self.robot.inWaiting() - 26) # dump any extra bytes, sensor data should be the last 26
                if self.robot.inWaiting() < 26:
                     self.robot.read(self.robot.inWaiting())
                     raise Exception("packet too small")
                self._sensor_data = self.robot.read(self.robot.inWaiting())
                if debug:
                    print "length of data array: " + str(len(self._sensor_data))
                #start new process here
                newdata = struct.unpack('B', self._sensor_data[11])[0]
                if self._old_button_state != newdata: 
                    self.get_button_function(self._old_button_state, newdata)
                    self._old_button_state = newdata 
                   # struct.pack('B', 2)
        except Exception as error:
            print("Error getting data: ", str(error))
            self.hwstate.set_init()
            self.robot_init()

    def get_button_function(self, old_state, new_state):
        #get diff trigger dif's
        
        #print("old val: " + str(old_state) + ", new val " + str(new_state))
        diff = old_state ^ new_state
        diff = diff & new_state
        try:
            if int(diff) == 8:
                self.buttons["power"]()
        except Exception as error:
            print("Function not implemented: " + str(error))
         

    def is_docked(self):
        """ returns True if the robot is docked, else False """
        with self._sensor_data_lock:
            val = struct.unpack('B',self._sensor_data[16])[0]
            if val == 2 or val == 3 or val == 4:
                return True
            return False

    def is_cleaning(self):
        """ returns True if the robot is cleaning, else False """
        if self.get_current() < -1000:
            self.hwstate.set_passive()
            return True
        return False
