"""

phone_link = BLEPhoneLink("/dev/ttyACM0")
phone_link.is_enabled()
phone_link.enable()
phone_link.disable()
phone_link.set_name("BB-8")
phone_link.set_model("atrvjr")
phone_link.set_state("ok")
phone_link.get_clients() -> ["00:11:22...", "aa:bb:cc..."]
phone_link.

"""
from chil.payload_pipe import PayloadPipe
from chil.robot_state import RobotState as CHILRobotState
from chil.state_manager import StateManager as CHILStateManager
from chil import bga_msgs
from chil import bga_schema
from lagerlogger import LagerLogger
from progressions import ConversationStack
import sys



class FakePhoneLink(object):
    """ Implements the same phone interface as teh real phone link """
    def __init__(self):
        self.logger = LagerLogger("FakePhoneLink")
        self.logger.console(LagerLogger.DEBUG)
        self.logger.warn("~~~ WARNING: USING FAKE PHONE LINK ~~~")
        self._state = "dangerous"
        self._enabled = False
    def is_enabled(self):
        return self._enabled
    def set_visibility(self, val): pass
    def enable(self):
        self.logger.debug("enable()")
        self.logger.warn("~~~ WARNING: USING FAKE PHONE LINK ~~~")
        self._enabled = True
    def disable(self):
        self.logger.debug("disable()")
        self.logger.warn("~~~ WARNING: USING FAKE PHONE LINK ~~~")
        self._enabled = False
    def set_name(self, value):  pass
    def set_model(self, value):  pass
    def set_state(self, value, update=True):  self._state = value
    def set_progression(self, progression, update=True): pass

class PhoneLink(object):
    def __init__(self):
        # Robot Data 
        self._name = "unknown"
        self._model = "unknown"
        self._state = "dangerous"
        self._progression = None
        self.logger = LagerLogger("")
        self.logger.console(LagerLogger.DEBUG)

        # Self state
        self._enabled = False
        self._stealth = False

        # Components
        self._pipe = PayloadPipe()
        self._pipe._lower_level.stop()
        self._chil_state = self._build_state()
        self._sm = CHILStateManager(self._pipe, self._chil_state)
        self._sm.subscribe_to_receive(self._receiver_cb)


    def is_enabled(self):
        """ checks to see if we are advertising ourselves as a robot """
        return self._enabled

    def is_stealth(self):
        return self._stealth

    def stealth_mode(self, enabled):
        """ Do not ever let robot be visible when stealth is enabled """
        self.logger.info("stealth_mode(%s)" % str(enabled))
        self._stealth = enabled
        if enabled:
            self._pipe.set_visibility(False)


    def set_visibility(self, val):
        """ Sets whether or not the phone interface will display the data being sent"""
        if self._stealth:
            self.logger.info("bluetooth set_visibility(%s/steathmode)" % str(val))
            self._pipe.set_visibility(False)
            return
        else:
            self.logger.info("bluetooth set_visibility(%s)" % str(val))
            sys.stdout.flush()

        self._pipe.set_visibility(val)
        # TODO: This is a hack to force visibility updates to make robots disappear...
        # This should really be fixed on the phone side?
#         if val == False:
#             self.set_state("dangerous")

    def enable(self):
        """ Enables advertising and connecting to us """
        if self._enabled:
            self.logger.debug("bluetooth enabled (again)")
            return
        else:
            self.logger.info("bluetooth enable()")
        sys.stdout.flush()
        self._pipe.enable()
        self._pipe.start_spinning()
        self._enabled = True

    def disable(self):
        """ disables advertising and connecting to us """
        if not self._enabled:
            self.logger.debug("Already disabled")
            return
        else:
            self.logger.debug("bluetooth disable()")
        sys.stdout.flush()
        self._pipe.set_visibility(False)
        self._pipe.disable()
        self._pipe.stop_spinning()
        self._enabled = False

    def set_name(self, name):
        """ sets the robots name """
        self.logger.debug("set_name(%s)" % name)
        self._name = name
        self._chil_state = self._build_state()

    def set_model(self, model):
        """ set the robots model """
        self.logger.debug("set_model(%s)" % model)
        self._model = model
        self._chil_state = self._build_state()

    def set_state(self, state, update=True):
        """ set the robots state"""
        self.logger.debug("set_state(%s)" % state)
        if state not in ["ok", "safe", "help", "dangerous", "off"]:
            raise Exception("Unknown state for chil: %s" % state)
        self._state = state
        self._chil_state = self._build_state()
        if update:
            self._sm.update_client_state(self._chil_state)

    def set_progression(self, progression, update=True):
        self.logger.debug("set_progression()")
#         self.logger.debug("set_progression(%s)" % str(progression))
        self._progression = progression if progression else None
        self._chil_state = self._build_state()
        if update:
            self._sm.update_client_state(self._chil_state)


    def _build_state(self):
        """ constructs and returns a CHILRobotState object """
        return CHILRobotState(self._name, self._model, self._state, self._progression)

    def _receiver_cb(self, payload):
        """ receive data coming back from phone """
        if 'msgtype' in payload and payload['msgtype'] == bga_msgs.REPLY_TYPE:
            bga_schema.validate_object(payload, bga_schema.reply_schema)
            msgid = int(payload["responses"][0]["id"])
            rspid = int(payload["responses"][0]["value"])
            self.logger.info("Phone Sent Response '%s'" % self._progression[msgid]["responses"][rspid]["value"])
            sys.stdout.flush()
            self._progression[msgid].set_selection(str(rspid))
            self._progression.run_cb(msgid,rspid)
            # handle the reply value:
            # e.g. update our progression with that selection and handle
        else:
            self.logger.error("received malformed payload '%s'." % payload)
            sys.stdout.flush()

if __name__ == "__main__":
    link = PhoneLink()
    link.set_name("test_robot")
    link.set_model("test_model")
    link.set_state("safe")
    link.enable()
    import time
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        link.disable()
