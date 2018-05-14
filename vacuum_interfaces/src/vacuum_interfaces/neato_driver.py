import sys
import time 
import threading
#import struct
import copy
import math
import logging
import serial
from threading import Lock
import spinner

SERIAL_TIMEOUT = 3

SENSOR_GROUP_PACKET = ( 
   "GETDIGITALSENSORS",
#     "GETANALOGSENSORS",
    "GETCHARGER",
    "GETBUTTONS"
)

CHARGING_SOURCES=("internal","base")

class FloatDict(dict):
    def __getitem__(self, key):
        return float(super(FloatDict,self).__getitem__(key))

class NeatoDriver(object):
    def __init__(self, dev_path):
        #init serial to dev path
        self.spinner = spinner.Spinner()
        self._sensor_data_lock = threading.Lock()
        self._serial_lock = threading.Lock()
        self._serial_device = serial.Serial(port=dev_path, timeout=SERIAL_TIMEOUT)
        self._sensor_data = FloatDict()

    def get_sensor(self, sensor_name):
        with self._sensor_data_lock:
            if sensor_name is "buttons":
                return {"soft_key":self._sensor_data["BTN_SOFT_KEY"],
                        "scroll_up":self._sensor_data["BTN_SCROLL_UP"], 
                        "start":self._sensor_data["BTN_START"],
                        "back":self._sensor_data["BTN_BACK"],
                        "scroll_down":self._sensor_data["BTN_SCROLL_DOWN"]}
            if sensor_name is "charging_sources_available":
                return CHARGING_SOURCES #not implemented propperly
            if sensor_name is "bumps_wheeldrops":
                return { "wheeldrop_left": bool(self._sensor_data["SNSR_LEFT_WHEEL_EXTENDED"]), "wheeldrop_right": bool(self._sensor_data["SNSR_RIGHT_WHEEL_EXTENDED"])} 
            return self._sensor_data[sensor_name]
     
    def update_sensors(self):
        self.spinner.spin()
        with self._serial_lock:
            dict_vals = FloatDict()
            for command in SENSOR_GROUP_PACKET:
                self._serial_device.write(command + "\n")
                self._serial_device.flush()
                data = list()
                while True:
                    time.sleep(0.01)
                    w = self._serial_device.inWaiting()
                    if w > 0:
                        data.append(self._serial_device.read(w))
                        if data[-1][-1] == '\x1a':
                            break

                data = "".join(data).strip()
#                 if not data[:len(command)] == command:
#                     print "BAD START for '%s'! '%s'" % (command,data)
                try:
                    dvals = FloatDict([x.split(',') for x in data.split("\r\n")[2:-1]])
                    dict_vals.update(dvals)
                except Exception as e:
                    print type(e), e
                    print data
        with self._sensor_data_lock:
            self._sensor_data = dict_vals

    def passive(self):
        print "Setting RObot to passive read"
        sys.stdout.flush()
        self._serial_device.write("testmode on\n")
        self._serial_device.flush()
        time.sleep(.25)
        self._serial_device.read(self._serial_device.inWaiting())

    def close(self):
        self._serial_device.write("testmode off\n")
        self._serial_device.flush()
        time.sleep(.25)
        self._serial_device.read(self._serial_device.inWaiting())
        self._serial_device.close()
