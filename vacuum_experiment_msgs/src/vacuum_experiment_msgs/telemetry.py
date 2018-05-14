from threading import Lock
from vacuum_experiment_msgs.msg import Telemetry
import copy
from lagerlogger import LagerLogger

class TelemetryData(object):
    def __init__(self):
        self._telemetry_cb = None
        self.logr = LagerLogger("TelemetryData")
        self.logr.console(LagerLogger.DEBUG)
        self._telemetry_lock = Lock()
        self._telemetry = {"charging": False,
                            "docked": False,
                            "lifted": False,
                            "dustbin": True,
                            "cleaning": False,
                            "battery_voltage": 0,
                            "sci_error": False,
                            "api_mode": "unknown",
                            "buttons": list()}

    def __getitem__(self, key):
        if key not in self._telemetry.keys():
            raise Exception("MemberValue not found in TelemetryData: %s" % key)
        with self._telemetry_lock:
            return copy.copy(self._telemetry[key])

    def __setitem__(self, key, value):
        if key not in self._telemetry.keys():
            raise Exception("MemberValue not found in TelemetryData: %s" % key)

#         print key,value
        with self._telemetry_lock:
            self._telemetry[key] = copy.copy(value)
#             value = copy.copy(self._telemetry)
#         print value
        try:
            if self._telemetry_cb:
                self._telemetry_cb(self)
        except Exception as e:
            self.logr.error("%s: %s" % (str(type(e)), e))

    def set_telemetry_update_cb(self, fun):
        self._telemetry_cb = fun
    
    def get_ros_msg(self):
        """ returns the ros Telemetry message """
        t = Telemetry()
        with self._telemetry_lock:
            for key, value in self._telemetry.items():
                t.__setattr__(key, value)
        return t
