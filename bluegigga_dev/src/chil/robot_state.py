import copy
import bga_msgs

class RobotState(object):

    def __init__(self, name, model, state, progression=None):
        self.name = name
        self.model = model
        self.state = state
        self.progression = progression

    def update_msg(self):
        return bga_msgs.update_msg(
            self.name,
            self.model,
            self.state,
            self.progression)

    def copy(self):
        return copy.deepcopy(self)

    def __eq__(self, other):
        return (self.name == other.name
                and self.model == other.model
                and self.state == other.state
                and self.progression == other.progression)

    def __str__(self):
        return (self.name + " " + self.model + " " + str(self.state) + " "
                + str(self.progression))
