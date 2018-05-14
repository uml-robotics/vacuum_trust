class RobotBehavior(object):

    def __init__(self, sm):
        self._sm = sm

    def behavior(self):
        raise NotImplementedError("Please Implement this method")
