from vacuum_experiment_msgs.enum_state import gen_enum_class
from lagerlogger import LagerLogger
import copy
class VacuumState(gen_enum_class("VacuumStateEnum", "idle", "cleaning", "spot_cleaning", "docking", "docked", "pausing")): 
    def __init__(self):
        super(VacuumState, self).__init__()
        self.logr = LagerLogger()
        self.logr.console(LagerLogger.DEBUG)
        self.callbacks = list()

    def set(self, state):
        old = super(VacuumState, self).get()
        super(VacuumState, self).set(state)
        if not old:
            old = "unknown"
        self.logr.info("%s >> %s" % (old, state))
        if self.callbacks:
            for cb in self.callbacks:
                try:
                    cb(copy.copy(self))
                except Exception as e:
                    self.logr.error(e)


