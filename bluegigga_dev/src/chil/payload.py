""""""
import bga_constants

class Payload(object):
    """Converts a string into a batch-based payload."""

    def __init__(self, raw_payload):
        """Processes a string into integer-based batches and stores the number \
                of frames in the payload."""
        self._segmented_payload = []
        self._size = 0
        payload_len = len(raw_payload)
        byte_index = 0
        while byte_index < payload_len:
            seq_num = 0
            batch = []
            while (seq_num < bga_constants.BATCH_SIZE
                    and byte_index < payload_len):
                byte_buffer = [seq_num]
                j = 0
                while (j < bga_constants.FRAME_SIZE - 1
                        and byte_index + j < payload_len):
                    byte_buffer.append(ord(raw_payload[byte_index + j]))
                    j += 1
                batch.append(byte_buffer)
                byte_index += bga_constants.FRAME_SIZE - 1
                seq_num += 1
            self._size += seq_num
            self._segmented_payload.append(batch)

    def size(self):
        """Returns the amount of frames in the segmented payload."""
        return self._size

    def segmented_payload(self):
        """Returns the segmented payload."""
        return self._segmented_payload
