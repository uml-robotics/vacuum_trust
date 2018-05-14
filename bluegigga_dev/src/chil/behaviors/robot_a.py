import time
from robot_behavior import RobotBehavior

# puts us in the directory above to import
import sys
import os.path
sys.path.append(
        os.path.abspath(
            os.path.join(os.path.dirname(__file__), os.path.pardir)))
from robot_state import RobotState
import bga_msgs
import bga_schema

class ProgEleBuilder(object):
    next_msgid = 0

    @staticmethod
    def idle_prog_ele():
        progression_element = bga_msgs.prog_element(
                msgid=str(ProgEleBuilder.next_msgid),
                content="Idling.",
                responses=[bga_msgs.dict_element("TV", "Start Vacuuming"),
                    bga_msgs.dict_element("TC", "Start Charging")])
        ProgEleBuilder.next_msgid += 1
        return progression_element

    @staticmethod
    def vacuum_prog_ele():
        progression_element = bga_msgs.prog_element(
                msgid=str(ProgEleBuilder.next_msgid),
                content="Vacuuming.",
                responses=[bga_msgs.dict_element("PV", "Stop Vacuuming"),
                    bga_msgs.dict_element("TC", "Start Charging")])
        ProgEleBuilder.next_msgid += 1
        return progression_element

    @staticmethod
    def charge_prog_ele():
        progression_element = bga_msgs.prog_element(
                msgid=str(ProgEleBuilder.next_msgid),
                content="Charging.",
                responses=[bga_msgs.dict_element("TV", "Start Vacuuming"),
                    bga_msgs.dict_element("PC", "Stop Charging")])
        ProgEleBuilder.next_msgid += 1
        return progression_element

class ProgressionState:
    IDLE = 1
    VACUUM = 2
    CHARGE = 3

class RobotA(RobotBehavior):

    def __init__(self, sm):
        RobotBehavior.__init__(self, sm)
        self._sm.subscribe_to_receive(self.receive_cb)
        self._robot_state = self._sm.robot_state()
        self._robot_state.progression = [ProgEleBuilder.idle_prog_ele()]
        self._prog_state = ProgressionState.IDLE

    def behavior(self):
        states = ['ok', 'safe', 'help', 'dangerous', 'off']
        state_index = 0
        while True:
            self._robot_state.state = states[state_index % len(states)]
            self._sm.update_client_state(self._robot_state)
            print self._sm.robot_state().state
            state_index += 1
            time.sleep(17)

    def receive_cb(self, payload):
        if 'msgtype' in payload and payload['msgtype'] == bga_msgs.REPLY_TYPE:
            print payload
            bga_schema.validate_object(payload, bga_schema.reply_schema)
            if len(payload['responses']) > 0:
                response = payload['responses'][0]
                latest_prog_ele = self._robot_state.progression[-1]
                if latest_prog_ele['msgid'] == response['id']:
                    for answer_pair in latest_prog_ele['responses']:
                        if answer_pair['id'] == response['value']:
                            # then the selection is valid, because it targets
                            # the most recent progression element, and because
                            # the selection belongs to that progression
                            # element's responses array.

                            # stores the selection
                            latest_prog_ele['selection'] = response['value']

                            if self._prog_state == ProgressionState.IDLE:
                                # Start Vacuuming
                                if answer_pair['id'] == 'TV':
                                    self._vacuum()
                                # Start Charging
                                elif answer_pair['id'] == 'TC':
                                    self._charge()
                            elif self._prog_state == ProgressionState.VACUUM:
                                # Stop Vacuuming
                                if answer_pair['id'] == 'PV':
                                    self._idle()
                                # Start Charging
                                elif answer_pair['id'] == 'TC':
                                    self._charge()
                            elif self._prog_state == ProgressionState.CHARGE:
                                # Start Vacuuming
                                if answer_pair['id'] == 'TV':
                                    self._vacuum()
                                # Stop Charging
                                elif answer_pair['id'] == 'PC':
                                    self._idle()
                            self._sm.update_client_state(self._robot_state)
                            break
                else:
                    print "received selection for old progression element"
        elif 'msgtype' not in payload:
            raise ValueError(
                    'received malformed payload, \'msgtype\' not found.')

    def _idle(self):
        self._prog_state = ProgressionState.IDLE
        self._robot_state.progression += [ProgEleBuilder.idle_prog_ele()]

    def _charge(self):
        self._prog_state = ProgressionState.CHARGE
        self._robot_state.progression += [ProgEleBuilder.charge_prog_ele()]

    def _vacuum(self):
        self._prog_state = ProgressionState.VACUUM
        self._robot_state.progression += [ProgEleBuilder.vacuum_prog_ele()]

def behavior(sm):
    RobotA(sm).behavior()
