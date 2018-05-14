#!/usr/bin/env python
from neato_ros_cpp import neato_ros_cpp
import time

def cmd_vel_cb(data, data2):
    print "Got things %s %s" % (data, data2)
#     print data
#     print data2

if __name__ == "__main__":
    neato_ros_cpp.init(cmd_vel_cb)
    try:
        while True:
            print "tick"
            time.sleep(1)
    except KeyboardInterrupt:
        print "shut down"
        neato_ros_cpp.dispose()
        print "out"

