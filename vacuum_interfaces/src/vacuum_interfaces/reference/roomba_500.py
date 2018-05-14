from roomba import RoombaSharedInterface as BaseRI
import Queue
import serial
import RPi.GPIO as GPIO
import struct
import time
import threading
from multiprocessing import Pool
from timer import Timer
#import numpy 

DUSTBIN = 16
DEVICE_DETECT = 7
debug = False


class Roomba500Interface(BaseRI):
    def __init__(self, device): 
        BaseRI.__init__(self,device, 115200)
        self.robot.write(struct.pack('BB', *[128, 131])) #enable robot

        self._dustbin_open = False #open meaning the dustbin has been removed
        self._dustbin_timer = Timer(0.01, self._dustbin_cb)
        self._dustbin_timer.start()

        GPIO.setup(DUSTBIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def _dustbin_cb(self): 
        self._dustbin_open = GPIO.input(DUSTBIN) 

    def _poll_sensors(self):
        with self._opcode_lock:
            try:
                self.robot.write(struct.pack('BB', *[142, 6]))
            except Error:
                print("Error occured while getting sensor data => " + str(Error)) 
        time.sleep(0.2) 
        try:
            with self._sensor_data_lock:
                if debug:
                    print "Num of bytes in buffer: " + str(self.robot.inWaiting())
                self._sensor_data = []
                if self.robot.inWaiting() > 52:
                    self.robot.read(self.robot.inWaiting() - 52) # dump any extra bytes, sensor data should be the last 26
                if self.robot.inWaiting() < 52:
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

    def get_button_function(self, old_state, new_state):
        #get diff trigger dif's
        
        #print("old val: " + str(old_state) + ", new val " + str(new_state))
        diff = old_state ^ new_state
        diff = diff & new_state
        try:
            if int(diff) == 1:
                self.buttons["Clean"]()
            if int(diff) == 2:
                self.buttons["Spot"]()
            if int(diff) == 4:
                self.buttons["Dock"]()
            if int(diff) == 8:
                self.buttons["Minute"]()
            if int(diff) == 16:
                self.buttons["Hour"]()
            if int(diff) == 32:
                self.buttons["Day"]()
            if int(diff) == 64:
                self.buttons["Schedule"]()
            if int(diff) == 128:
                self.buttons["Clock"]()
        except Exception as error:
            print("Function not implemented: " + str(error))
         
    #def poll_sensor_data_passer(self):
    #    self._job_queue.put(self._poll_sensor_data_cb)


    def is_docked(self):
        """ returns True if the robot is docked, else False """
        val = struct.unpack('B', self._sensor_data[39])[0]
        if val == 2 or val == 3:
            return True
        else:
            return False     
        pass

    
    def is_cleaning(self):
        """ returns True if the robot is cleaning, else False """
        if self.get_current() < -800:
            return True
        else:
            return False

    def dustbin_open(self):
        return self._dustbin_open

        pass
    def dock_passed(self):
        with self._opcode_lock:
            print("500's dock")
            self.robot.write(struct.pack('i', 143))
            time.sleep(0.2)
            self.robot.write(struct.pack('i', 143))
        pass 
