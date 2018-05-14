import bga_msgs
from lagerlogger import LagerLogger

class StateManager(object):
    def __init__(self, pipe, robot_state):
        self.logr = LagerLogger()
        self.logr.console(LagerLogger.DEBUG)
        self._pipe = pipe
        self._robot_state = robot_state
        self.connected_client = None

        pipe.subscribe_to_connect(self.on_connect_cb)
        pipe.subscribe_to_disconnect(self.on_disconnect_cb)

    def on_connect_cb(self, who):
        """Determines if a state update is necessary upon client connection."""
        self.connected_client = who

    def on_disconnect_cb(self):
        self.connected_client = None

#     def update_client_state(self, robot_state=None):
#         """Sends the currently connected client a state update. Also will change
#         the StateManager's internal state if robot_state != None.
#         """
#         if robot_state != None:
#             self._robot_state = robot_state
#         state_msg = self._robot_state.update_msg()
#         self._pipe.set_adv_cache_data(state_msg)
#         self._pipe.write(state_msg)

    def update_client_state(self, robot_state):
        """ Accepts a new state to publish, sets the checksum, copies data into pipe """
        self.logr.info("update_client_state()")
        self._robot_state = robot_state
        state_msg = self._robot_state.update_msg()
        self._pipe.set_adv_cache_data(state_msg)
        self._pipe.write(state_msg)

    def subscribe_to_receive(self, on_receive_cb):
        self._pipe.subscribe_to_receive(on_receive_cb)

    def robot_state(self):
        return self._robot_state.copy()
