#!/usr/bin/env python
import importlib
import errno
import socket
import sys
from threading import Thread
import time

import xmltodict

from payload_pipe import PayloadPipe
from robot_state import RobotState
from state_manager import StateManager

DEBUG_CONFIG = "configs/robot-lab1.xml"

def load_xml_to_dict():
    config_file_name = 'configs/' + socket.gethostname() + '.xml'
    try:
        with open(config_file_name, 'r') as content_file:
            xml = content_file.read()
    except IOError as e:
        # Uses a debug config if a robot behavior under the current hostname
        # does not exist.
        if e.errno == errno.ENOENT:
            with open(DEBUG_CONFIG, 'r') as content_file:
                xml = content_file.read()
        else:
            raise
    return xmltodict.parse(xml)['robot']

def start_host_robot(config, sm):
    mod_name = str(config['mod_name'])
    robot_module = importlib.import_module("behaviors." + mod_name)
    behavior_thread = Thread(target=robot_module.behavior, args=(sm, ))
    behavior_thread.daemon = True
    behavior_thread.start()

pipe = PayloadPipe()
pipe.set_visibility(True)
config = load_xml_to_dict()
sm = StateManager(
        pipe,
        RobotState(
            config['name'],
            config['model'],
            config['state'],
            config['progression']))

start_host_robot(config, sm)
pipe.start_spinning()
while True:
    try:
        time.sleep(0.1)
    except KeyboardInterrupt:
        pipe.stop_spinning()
        pipe.disable()
        sys.exit()
