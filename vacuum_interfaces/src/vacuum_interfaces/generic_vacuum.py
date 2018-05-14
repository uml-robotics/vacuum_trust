class VacuumInterface(object):
    """ These are generic functions that should be implemented for each type of robot.
    They should be overloaded in the roomba.py and neato.py files.
    """
    def start_cleaning(self):
        """ Tell robot to begin performing its cleaning action """
        raise NotImplementedError()

    def pause_cleaning(self):
        """ Tell robot to stop/pause its cleaning action, but don't go home """
        raise NotImplementedError()

    def end_cleaning(self):
        """ Tell robot to return to its charger (Might have to be done w/ ROS on neato?) """
        raise NotImplementedError()

    def get_buttons(self):
        """ returns the state of the buttons on the device """
        raise NotImplementedError()

    def get_charger(self):
        """ returns True if connected to charger, otherwise false """
        raise NotImplementedError()
