""""""
from bitarray import bitarray
import bga_constants, time

class BatchBuilder(object):
    """"""
    WAIT_TIME = .15

    def __init__(self, total_frames_left):
        if total_frames_left < bga_constants.BATCH_SIZE:
            self._final_size = total_frames_left
        else:
            self._final_size = bga_constants.BATCH_SIZE
        self._size = 0
        self._frames = [False] * self._final_size
        self._last_received = None
        self._accept_resends = False

    def add_frame(self, frame):
        """Adds a frame to the batch. Returns a boolean which describes the
        batch transfer's completeness."""
        self._last_received = time.time()
        frame_index = frame[0]
        if frame_index >= bga_constants.BATCH_SIZE and self._accept_resends:
            # disables the most significant bit of the byte
            frame_index = frame_index - 128
        if (frame_index < bga_constants.BATCH_SIZE
                and self._frames[frame_index] is False):
            self._frames[frame_index] = frame
            self._size += 1
        return self._size == self._final_size

    def is_waiting(self):
        return (self._size != self._final_size
                and self._last_received != None
                and time.time() - self._last_received > BatchBuilder.WAIT_TIME)

    def get_missed_frames_pkt(self):
        self._last_received = time.time()
        self._accept_resends = True
        missed_frames_pkt = bitarray(self._final_size + 1)
        missed_frames_pkt.setall(False)
        for index in range(0, self._final_size):
            if self._frames[index] is False:
                missed_frames_pkt[0] = True
                missed_frames_pkt[index + 1] = True
        return missed_frames_pkt

    def build(self):
        return self._frames
