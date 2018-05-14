#!/usr/bin/env python

import rospy
from vacuum_experiment_msgs.msg import PhoneReply

import numpy as np
import random

import sys


def talker(topic):
    pub = rospy.Publisher('/'+topic+'/phonereply', PhoneReply, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(2) # 2hz
    while not rospy.is_shutdown():
        msg = PhoneReply()
        msg.buttons = ['max'] # Power is always on, max is always off

        # Randomly select clean, spot, or both
        if bool(random.getrandbits(1)):
            msg.buttons.append('clean')
        if bool(random.getrandbits(1)):
            msg.buttons.append('spot')


        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':

    if len(sys.argv) != 2:
        print "rosrun vacuum_wizard_controls phonereply_chatter.py [robot-name]"
        exit()

    try:
        talker(sys.argv[1])
    except rospy.ROSInterruptException:
        pass
