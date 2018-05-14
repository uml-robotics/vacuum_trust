#!/usr/bin/env python

import rospy
from vacuum_experiment_msgs.msg import Telemetry

import numpy as np
import random

import sys


def talker(topic):
    pub = rospy.Publisher('/'+topic+'/telemetry', Telemetry, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(4) # 4hz
    while not rospy.is_shutdown():
        msg = Telemetry()
        msg.battery_voltage = np.random.randint(0, 100)
        msg.lifted = bool(random.getrandbits(1))
        msg.charging = bool(random.getrandbits(1))
        msg.docked = bool(random.getrandbits(1))
        msg.dustbin = bool(random.getrandbits(1))
        msg.cleaning = bool(random.getrandbits(1))
        msg.sci_error = bool(random.getrandbits(1))
        msg.buttons = ['power'] # Power is always on, max is always off

        # Randomly select clean, spot, or both
        if bool(random.getrandbits(1)):
            msg.buttons.append('clean')
        if bool(random.getrandbits(1)):
            msg.buttons.append('spot')


        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':

    if len(sys.argv) != 2:
        print "rosrun vacuum_wizard_controls telemetry_chatter.py [robot-name]"
        exit()

    try:
        talker(sys.argv[1])
    except rospy.ROSInterruptException:
        pass
