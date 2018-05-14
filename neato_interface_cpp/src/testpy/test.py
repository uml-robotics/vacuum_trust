#!/usr/bin/env python
from neato_ros_cpp import neato_ros_cpp
from neato_cpp import neato_cpp
import time

#import neato_cpp

def name(a, b):
	print a
	print b

print "I will say hello\n"

neato_cpp.init("/dev/ttyACM0")
neato_ros_cpp.init(name)
neato_ros_cpp.set_debug(1)
neato_cpp.set_debug(1)

print "hello\n"

y = neato_cpp.readLidarData()
time.sleep(0.25)
x = neato_cpp.readMotorCommands()

while True:
	print "tick"
	neato_ros_cpp.pub_laser(y)
	neato_ros_cpp.pub_transform(x[0], x[1])
	time.sleep(0.25)
	x = neato_cpp.readMotorCommands()
        time.sleep(0.25)
        y = neato_cpp.readLidarData()
