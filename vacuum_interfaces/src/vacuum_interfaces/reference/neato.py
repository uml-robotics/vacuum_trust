#!/usr/bin/env python
from neato_driver.neato_driver import xv11
from generic_vacuum import VacuumInterface


class NeatoInterface(VacuumInterface):
    def __init__(self, device="/dev/ttyUSB0"):
        VacuumInterface.__init__(self)
        self.dev = device
        self._xv11 = xv11(port=self.dev)

    def _neato_cmd(self, data):
        """ Send an api command to neato, return the response (if any) as a list.
        This is for implementing functionality not included in the 'neato_driver'
        provided by the ROS package.
        """
        self._xv11.port.write(data + "\n")
        return self._xv11.port.readlines()

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
        pass

    def is_cleaning(self):
        """ returns True if neato is cleaning, else False """
        pass

    def dock(self):
        """ tell neato to go to its dock """
        pass

    def clean(self):
        """ tell neato to start cleaning """
        pass

    def beep(self):
        """ tell neato to make a beeping noise """
        pass

