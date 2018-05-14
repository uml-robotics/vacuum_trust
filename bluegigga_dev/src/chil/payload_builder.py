""""""
import json

class PayloadBuilder(object):
    """Converts a string into a batch-based payload."""

    def __init__(self, final_size):
        self._final_size = final_size
        self._size = 0
        self._batches = []

    def add_batch(self, batch):
        batch_size = len(batch)

        # batch too large to fit in the payload <wrong batch received>
        if batch_size + self._size > self._final_size:
            raise ValueError(
                    'Size mismatch on received batch within PayloadBuilder.')
        else:
            self._size += batch_size
            self._batches.append(batch)

    def is_complete(self):
        return self._final_size == self._size

    def build(self):
        if not self.is_complete():
            raise RuntimeError('Cannot build an incomplete payload.')
        json_ascii = ''
        for batch in self._batches:
            for frame in batch:
                for byte in frame[1:]:
                    json_ascii += chr(byte)
        return json.loads(json_ascii)

    def final_size(self):
        return self._final_size

    def size(self):
        return self._size
