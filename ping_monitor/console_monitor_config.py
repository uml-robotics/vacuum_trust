#!/usr/bin/env python

import os

class ConsoleMonitorConfig(dict):
    """ Loads arbitrary settings from a config file and from the command line.
    Settings are stored as config['section']['setting'] = value
    All names are automatically converted to lower case.
    """

    def _add_setting(self, section, setting, value):
        """ Adds a setting to the RobotConfig """
        section = section.lower()
        if section not in self.keys():
            self[section] = dict()
        self[section][setting] = value

    def parse_config_file(self, filename):
        """ Load arbitrary settings from config file"""
        import ConfigParser
        self.cfg_file = ConfigParser.ConfigParser()
        path = os.path.dirname(os.path.realpath(__file__))
        path = os.path.join(path, filename)
        try:
            self.cfg_file.readfp(open(path))
        except IOError as e:
            print "Unable to load config file '%s'" % path
            exit(0) 
        for section in self.cfg_file.sections():
            settings = self.cfg_file.options(section)
            for setting in settings:
                self._add_setting(section, setting, self.cfg_file.get(section, setting))

"""
config = ConsoleMonitorConfig()
config.parse_config_file("config.cfg")

for robot in config.keys():
    print "robot: ", robot
    topicArr = config[robot]['buttons'].split(',')
    btnArr = config[robot]['commands'].split(',')
    for topic in topicArr:
        print "\ttopic: ", topic
    for btn in btnArr:
        print "\tbtn: ", btn




config2 = ConsoleMonitorConfig()
config2.parse_config_file("cmd_topic_mapping.cfg")

for robot in config2.keys():
    print "robot: ", robot
    topicArr = config2[robot]['topic'].split(',')
    btnArr = config2[robot]['cmd'].split(',')
    for topic in topicArr:
        print "\ttopic: ", topic
    for btn in btnArr:
        print "\tbtn: ", btn
"""
