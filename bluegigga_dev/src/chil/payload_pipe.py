""""""
import sys
import binascii
import copy
import time
from threading import Lock
from lagerlogger import LagerLogger
from bg_serial import BgSerial
from payload import Payload
from payload_builder import PayloadBuilder
from timer import Timer


# [db] added to try to make threaded receive callbacks
from threading import Lock, Thread
def threaderized(fun, *args, **kwargs):
    t = Thread(target=fun, args=args, kwargs=kwargs)
    t.start()
    return t


class PayloadPipe(object):
    """Interacts with the bga layer to send and receive information."""

    def __init__(self):
        """Basic initialization."""
        self._batch_index = 0
        self.logr = LagerLogger("")
        self.logr.console(LagerLogger.DEBUG)
        self._lower_level = BgSerial(self)
        self._lower_level.subscribe_to_connect(self._on_notify_connect_cb)
        self._lower_level.subscribe_to_disconnect(self._on_disconnect_cb)
        self._lower_level.subscribe_to_transfer(self._on_transfer_cb)
        self._lower_level.subscribe_to_receive_batch(self._on_receive_batch_cb)
        self._last_batch_sent = None
        self._who_connected = None
        self._current_payload_out = None
        self._next_payload_out = None
        self._payload_builder_in = None
        self._on_receive_cbs = []
        self._on_connect_cbs = []
        self._on_disconnect_cbs = []
        self._spin_timer = Timer(0.01, self._lower_level.spin_once)
        self._write_in_progress_lock = Lock()
        self._next_payload_lock = Lock()
        self._payload_builder_in_lock = Lock()

    def write(self, raw_payload, who=None):
        """Writes payload to the _current_payload_out which is
        automatically transferred once the client requests the size of
        the payload.
        """
        with self._next_payload_lock:
            self._next_payload_out = Payload(raw_payload)

    def send_next_payload(self):
        """Uses the bga layer to send the batches in the payload."""
        if self._current_payload_out != None:
            for batch in (self._current_payload_out.segmented_payload()):
                self._last_batch_sent = batch
                if self._lower_level.write_batch(batch) == False:
                    # Then the payload's transfer failed to write because there
                    # was a disconnect. So we don't pop the last message from
                    # the queue.
                    self.logr.error("Disconnect occurred in the middle of a write.")
                    self._write_in_progress_lock.release()
                    return
            self._current_payload_out = None
            self._write_in_progress_lock.release()

    def start_spinning(self):
        """Starts a thread to processes the bga-layer's callback queue."""
        self._spin_timer.start()

    def stop_spinning(self):
        """Stops processing the bga-layer's callback queue."""
        self._spin_timer.cancel()

    def set_visibility(self, visibility):
        """Signals the bga-layer to enable/disable its visibility to clients at
        a software level."""
        self._lower_level.set_adv_visibility_data(visibility)

    # expects cache to be a string
    def set_adv_cache_data(self, cache):
        cache_hex = "%08x" % (binascii.crc32(cache) & 0xFFFFFFFF)
        self.logr.info("NEW CHECKSUM set_adv_cache_data(%s)" % cache_hex)
#         import traceback
#         traceback.print_stack()
        byte_array = []
        while len(cache_hex) != 0:
            byte_array.insert(0, int(cache_hex[-2:], 16))
            cache_hex = cache_hex[:-2]
        self._lower_level.set_adv_cache_data(byte_array)

    def get_last_batch_sent(self):
        """Required by the lower-level when they are resending lost packets."""
        return self._last_batch_sent

    def prepare_next_payload(self):
        """Called when a read is performed on the lower level."""
        with self._next_payload_lock:
            permission_acquired = self._write_in_progress_lock.acquire(False)
            if (self._next_payload_out != None and permission_acquired):
                self._current_payload_out = copy.deepcopy(self._next_payload_out)
            return permission_acquired;

    def get_payload_size(self):
        """Retrieves the number of frames that make up the current payload."""
        if self._current_payload_out != None:
            return self._current_payload_out.size()
        else:
            return 0

    def total_frames_remaining(self, who):
        """Retrieves the amount of frames needed to complete the newest payload
        from 'who'."""
        with self._payload_builder_in_lock:
            if self._payload_builder_in != None:
                return self._payload_builder_in.final_size() - self._payload_builder_in.size()
            else:
                return 0

    def enable(self):
        """Signals the bg_serial layer to stop advertising."""
        self._lower_level._start_advertising()

    def disable(self):
        """Signals the bg_serial layer to stop advertising."""
        self._lower_level.stop()

    def _on_notify_connect_cb(self, who):
        """ This begins the client reading data from the phone """
        self.logr.debug("%s sent notify connect" % who) 
        self._who_connected = who
        for on_connect_cb in self._on_connect_cbs:
            on_connect_cb(who)

    def _on_disconnect_cb(self):
        self.logr.debug("disconnected")
        for on_disconnect_cb in self._on_disconnect_cbs:
            on_disconnect_cb()

    def _on_transfer_cb(self, who, payload_size):
        with self._payload_builder_in_lock:
            self.logr.debug("Phone wants to send data of size %d" % payload_size)
            self._payload_builder_in = PayloadBuilder(payload_size)
            sys.stdout.flush()

    def _on_receive_batch_cb(self, who, batch):
        with self._payload_builder_in_lock:
            self.logr.debug("Received batch - %s" % binascii.crc32(str(batch)))
            sys.stdout.flush()
            if not self._payload_builder_in:
                self.logr.error("FUCK ME SIDEWAYS")
                sys.stdout.flush()
                return

            self._payload_builder_in.add_batch(batch)
            if self._payload_builder_in.is_complete():
                # Tell bg_serial to not accept batch data anymore
                self._lower_level._reading_batch = False
                self.logr.debug("Payload Complete - running callbacks")
                sys.stdout.flush()
                json_obj = self._payload_builder_in.build()
                threads = list()
                self.logr.debug("Starting %d callback threads" % len(self._on_receive_cbs))
                for receive_cb in self._on_receive_cbs:
#                     receive_cb(json_obj)
                    threads.append(threaderized(receive_cb, json_obj))
                for t in threads:
                    t.join(3.0)
                    if t.is_alive():
                        self.logr.warn("Long running callback, can't wait!")

                self._payload_builder_in = None
                self.logr.debug("Client data send completed - payloadbuilder = none")
                self.logr.info("Signaling allclear to phone")
                sys.stdout.flush()

    def subscribe_to_receive(self, receive_cb):
        self._on_receive_cbs.append(receive_cb)

    def subscribe_to_connect(self, on_connect_cb):
        self._on_connect_cbs.append(on_connect_cb)

    def subscribe_to_disconnect(self, on_disconnect_cb):
        self._on_disconnect_cbs.append(on_disconnect_cb)
