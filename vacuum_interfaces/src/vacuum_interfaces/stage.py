#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import *
from generic_vacuum import VacuumInterface
import random


class StageInterface(VacuumInterface):
    def __init__(self, device="/dev/ttyUSB0", config=None):
        VacuumInterface.__init__(self)
        if not config:
            print "FUCK FUCK FUCK"
        self.config = config
        rospy.Subscriber(config["stage"]["laser_topic"], LaserScan, lambda x: self.laser_cb(x))
        self._motors_pub = rospy.Publisher(config["stage"]["motor_topic"], Twist, queue_size=10)
        self.cmd = "fwd" # back, turn
        self.behaviors =  {"fwd": self.drive_fwd,
                            "back": self.drive_back,
                            "turn": self.drive_turn,
                            "turning": lambda x: 1+2}
        self.state = "docked" # "cleaning" "pause"


    def get_charger(self):
        self._xv11.getCharger()
        if not "ChargingActive" in self._xv11.state.keys():
            raise Exception("Failed to retreive charging status")
        # TODO: I don't know what ChargingActive's values can be, so this probably
        # needs to be fixed
        if self._xv11.state["ChargingActive"] == "1":
            return True
        return False

    def is_docked(self):
        """ returns True if neato is on its dock, else False """
        return self.state == "docked"

    def is_cleaning(self):
        """ returns True if neato is cleaning, else False """
        return self.state == "cleaning"

    def dock(self):
        """ tell neato to go to its dock """
        pass

    def pause(self):
        self.state = "pause"
        rospy.sleep(0.25)

    def clean(self):
        """ tell neato to start cleaning """
        self.state = "cleaning"
 

    def beep(self):
        """ tell neato to make a beeping noise """
        pass

    def laser_cb(self, data):
        if self.is_cleaning():
            num_pts = len(data.ranges)
            width = 1000
            d = min(data.ranges[(num_pts/2)-(width/2):(num_pts/2)+(width/2)])
            self.behaviors[self.cmd](d)
        if self.state == "pause" or self.state == "docked":
            self._motors_pub.publish(Twist(linear=Vector3(x=0),angular=Vector3(z=0)))

    def drive_fwd(self, dist):
        if dist > .3:
            self._motors_pub.publish(Twist(linear=Vector3(x=.2),angular=Vector3(z=0)))
        else:
            print "%s: back" % self.config['robot']['name']
            self.cmd = "back"

    def drive_back(self, dist):
        if dist < .32:
            self._motors_pub.publish(Twist(linear=Vector3(x=-.05),angular=Vector3(z=0)))
        else:
            print "%s: turn" % self.config['robot']['name']
            self.cmd = "turn"

    def drive_turn(self, dist):
        self.cmd = "turning"
        v = random.choice([0.3,-.3])
        self._motors_pub.publish(Twist(angular=Vector3(z=v)))
        d = random.randrange(5,10)
        rospy.Timer(rospy.Duration(d), lambda x: self.drive_stopturn(x),oneshot=True)


    def drive_stopturn(self, x):
        print "%s: fwd" % self.config['robot']['name']
        self.cmd="fwd"


