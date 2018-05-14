# 'workingneato' robot controller
from neato_controller import NeatoController

class RobotController(NeatoController):
    def __init__(self, config):
        NeatoController.__init__(self, config)
        self.state = None
    """
    States:
        cleaning - wubbing about cleaning
        cleaning_paused - in the middle of cleaning, but currently stopped
        docking - heading back towards the dock
        docked - sitting on the charging station
    """

    def initialize(self):
        self.init_state()

    def init_state(self):
        """ Figure out what state you are in to initialize to """
        if self.state:
            raise Exception("Already Initialized state")
        if self.robot.is_docked():
            self.state = "docked"
            self.robot.beep()
            return
        # The robot is not docked, so we must be in another state.
        if self.robot.is_cleaning():
            self.state = "cleaning"
            self.robot.beep()
            return
        
        self.state = "cleaning"
        self.state = "cleaning_paused"
        return


    def cmd_dock(self):
        """ Send the robot to its docking station """
        if not self.state:
            raise Exception("Robot not initialized yet")
        if self.state == "docked":
            print "Robot already docked"
            return
        
        self.robot.dock()
        self.state = "docking"
        return

    def cmd_clean(self):
        """ Tell the robot to clean the floor """
        if not self.state:
            raise Exception("Robot not initialized yet")
        if self.state == "cleaning":
            print "Robot already cleaning"
            return

        self.robot.clean()
        self.state = "cleaning"
        return
 
