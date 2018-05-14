""" Expands upon the turtlebot create_drive code """

import time
import threading
import struct
import copy
import math
from create_driver import Roomba as BasicRoomba
from create_driver import SENSOR_GROUP_PACKET_LENGTHS
from create_driver import BadDataLengthError
from lagerlogger import LagerLogger

# This defines the list of sensors sent back by the roomba device, in order of appearance.
# Some, but not all of these might be present in various devices
SENSORS=\
    "bumps_wheeldrops, wall, cliff_left, cliff_front_left, cliff_front_right, cliff_right,"\
    "virtual_wall, motor_overcurrents, dirt_detector_left, dirt_detector_right,"\
    "remote_opcode, buttons, distance, angle, charging_state, voltage, current,"\
    "temperature, charge, capacity, wall_signal, cliff_left_signal, cliff_front_left_signal,"\
    "cliff_front_right_signal, cliff_right_signal, user_digital_inputs, user_analog_input,"\
    "charging_sources_available, oi_mode, song_number,song_playing, number_of_stream_packets,"\
    "requested_velocity, requested_radius, requested_right_velocity, requested_left_velocity,"\
    "encoder_counts_left, encoder_counts_right, light_bumper, light_bump_left,"\
    "light_bump_front_left, light_bump_center_left, light_bump_center_right,"\
    "light_bump_front_right, light_bump_right, ir_opcode_left, ir_opcode_right,"\
    "left_motor_current, right_motor_current, main_brush_current, side_brush_current, statis"

# Breakout bits orders for certain packet structures
BUMPS_WHEELDROPS=["bump_right","bump_left","wheeldrop_right","wheeldrop_left","wheeldrop_caster"]
BUTTON_NAMES=["max","clean","spot","power"] # Gets Overwritten by config
CHARGING_STATES={0:"not_charging", 1: "charge_recovery", 2: "charging", 3: "trickle_charging", 4: "waiting", 5: "error"}
CHARGING_SOURCES=["internal","base"]
OI_MODE=["off","passive","safe","full"]

SENSOR_CASTS={"wall": bool, "cliff_left": bool, 
                "cliff_front_left": bool, 
                "cliff_front_right": bool,
                "cliff_front_right": bool,
                "cliff_right": bool,
                "virtual_wall": bool,
                # Motor Overcurrents
                "dirt_detect_left": bool,
                "dirt_detect_right": bool,
                # Remote command
#                 "buttons": lambda x: x,
                "distance": int,
                "angle": int,
                "charging_state": lambda x: CHARGING_STATES[x],
#                 "bumps_wheeldrops": lambda x: x,
#                 "current": lambda x: x,
                "requested_velocity": int,
                "main_brush_current": int,
                "side_brush_current": int,
                "voltage": int,
                "oi_mode": lambda x: OI_MODE[x]}

class SensorStoreException(Exception): pass

class SensorStore(object):
    """ Casts sensor values upon request """

    def __init__(self, data):
        self.data = data
        self.__process_bit_flags("bumps_wheeldrops", BUMPS_WHEELDROPS)
        self.__process_bit_flags("buttons", BUTTON_NAMES)
        if "charging_sources_available" in self.data.keys():
            self.__process_bit_flags("charging_sources_available", CHARGING_SOURCES)

    def __process_bit_flags(self, data_name, bit_names):
        """ Process individual bits into a dictionary of bools """
        if data_name in self.data.keys():
            self.data[data_name] = dict(zip(bit_names,[(self.data[data_name]&(1<<i))!=0 for i in range(len(bit_names))]))
        else:
            self.data[data_name] = dict(zip(bit_names,[False]*len(bit_names)))

    def get(self, name):
        if name not in self.data.keys():
            raise SensorStoreException("Data %s not collected!, have %s" % (name, ",".join(self.data.keys())))
        if name in SENSOR_CASTS.keys():
            return copy.copy(SENSOR_CASTS[name](self.data[name]))
        return copy.copy(self.data[name])

    def __getattr__(self, name):
        return self.get(name)

class AdvancedRoomba(BasicRoomba):
    """ A more appropriate elaboration of the Roomba defined by Turtlebot """
    def __init__(self, config):
        global BUTTON_NAMES
        BasicRoomba.__init__(self)
        self._config = config
        self._packet_id = int(config["hwinterface"]["update_packet_id"])
        BUTTON_NAMES = [x.strip() for x in config["hwinterface"]["buttons"].split(",")]
        # Sensor Data Structures - the first is used for the create (and the 500?)
#         self._sensor_struct = struct.Struct(">12B2hBHhb7HBH5B4h2HB6H2B4hb")
#         self._sensor_struct = struct.Struct(">12B2hBHhb2H") 
        self._sensor_struct = struct.Struct(config["hwinterface"]["sensor_struct"])
        self._sensor_names = SENSORS.replace(" ","").split(",")
        self._sensor_data_lock = threading.Lock()
        self._sensor_data = SensorStore(dict()) # populated by update_sensors
        
        # Use safe mode, not full mode
        self.safe = False

#     def control(self):
#         """ overrides BasicRoombas control """
#         self.passive()
#         self.sci.control()
#         time.sleep(0.5)

    def power_off(self):
        """ tells the roomba to power off """
        self.sci.power()

    def dock(self):
        BasicRoomba.dock(self)

    def clean(self):
        self.sci.start()
        time.sleep(0.5)
        self.sci.clean()

    def pause(self):
#         self.control()
        self.sci.full()
        time.sleep(0.5)
        self.passive()

    def update_sensors(self):
#         packet_id = 0 #100 # Get all the things
        with self.sci.lock:
            length = None
#             try:
            self.sci.flush_input()
            self.sci.sensors(self._packet_id)
            length = SENSOR_GROUP_PACKET_LENGTHS[self._packet_id]
            data = self.sci.read(length)
            vals = self._sensor_struct.unpack(data[0:length])
#             except BadDataLengthError as e:
#                 print e
#                 print "Restarting SCI into control() mode"
#                 self.sci.control()
#                 return
#             except Exception as e:
#                 print e
#                 print "Restarting SCI into control() mode"
#                 self.sci.control()
#                 # WHEN THIS HAPPENS - the robot seems to have turned off.
#                 # On dirtdog this would happen when the person presses
#                 # the power button while cleaning.
#                 return

        with self._sensor_data_lock:
            self._sensor_data = SensorStore(dict(zip(self._sensor_names[0:len(vals)], vals)))


    def get_sensor(self, name):
        with self._sensor_data_lock:
            return self._sensor_data.get(name)

    def close(self):
        self.passive()
        self.sci.ser.close()

    def set_song(self, song_num, song_def):
        """ song def [(note,length),(note,length)...] length = fraction of a second """
        song_len = len(song_def)
        song_def = [item for sublist in song_def for item in sublist]
        song = [song_num, song_len] + song_def
        self.sci.song(*song)

    def play_song(self, song_num):
        """ only available during full or safe mode """
        self.sci.play(song_num)

    def set_leds(self, leds, color, intinsity):
        """ leds is a [list], power color, power intinisty (only in full) """
        led_bits = 0
        for led in leds:
            led_bits = led_bits | (1<<{"debris": 0, "spot": 1, "dock": 2, "check": 3}[led])
        data = [led_bits, color, intinsity]
        self.sci.leds(*data)

