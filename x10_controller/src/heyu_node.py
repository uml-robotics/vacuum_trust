#!/usr/bin/env python

import roslib; roslib.load_manifest('heyu_X10')
import rospy

import subprocess

from heyu_X10.srv import GetStatus, GetStatusResponse, GetStatusRequest, SendCommand, SendCommandResponse

class HeyuAppliance(object):
    def __init__(self, name, alias):
        self.name = name
        self.alias = alias
        print("Shit happened")
        rospy.logerr("Shit happened __init__")
        self.global_heyu_path = rospy.get_param("~heyu_bin_path","/usr/local/bin/heyu")
        self.off_command_pref = self.global_heyu_path + " off "
        self.on_command_pref = self.global_heyu_path + " on "
        self.status_command_pref = self.global_heyu_path + " onstate "

        status_topic = "heyu/" + name + "/state"
        command_topic = "heyu/" + name + "/command"
        self.status_srv = rospy.Service(status_topic, GetStatus, self.check_state)
        self.command_srv = rospy.Service(command_topic, SendCommand, self.run_command)

    def run_command(self, req):
        '''
        Starts or stop the appliance, then return its status.
        '''
        if req.start: #received true => turn the appliance on
            rospy.loginfo("Turning "+self.name+" ("+self.alias+") on")
            cmd = self.on_command_pref + self.name
            process = subprocess.Popen(cmd.split(' '))
            process = process.communicate()
        else: #received false => turn it off
            rospy.loginfo("Turning "+self.name+" ("+self.alias+") off")
            cmd = self.off_command_pref + self.name
            process = subprocess.Popen(cmd.split(' '))
            process = process.communicate()
        status = self.check_state(GetStatusRequest())
        return SendCommandResponse(status.status)


    def check_state(self, req):
        '''
        check_state is the callback from the service GetStatus
        to check the current state for this Appliance.
        @return True if the appliance is on, False if off.
        '''
        cmd = self.status_command_pref + self.name
        process = subprocess.Popen(cmd.split(' '), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        process = process.communicate()

        if "1" in process[0]:
            return GetStatusResponse(True)
        else:
            return GetStatusResponse(False)

class HeyuX10RosNode(object):
    def __init__(self):
        self.heyus = {}
        self.config_file_path = rospy.get_param("~heyu_config_path", "/etc/heyu/x10.conf")
        self.get_heyus(self.config_file_path)

    def get_heyus(self, config_file_path):
        f = open(config_file_path)
        for line in f.readlines():
            splitted = line.split()
            if len(splitted) == 0: 
                continue

            if splitted[0] != "ALIAS":
                # not a appliance
                continue
            app_name = splitted[1]
            alias = splitted[2]
            self.heyus[app_name] = HeyuAppliance(app_name, alias)
            rospy.loginfo("New heyu detected: ("+alias+") "+app_name)

    def start(self):
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('heyu_x10_node', anonymous=True)
    rospy.logerr("Shit happened")
    heyu_node = HeyuX10RosNode()
    heyu_node.start()
