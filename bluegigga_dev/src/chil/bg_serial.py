#!/usr/bin/env python
# [cm] 20160906 christopher_munroe1@student.uml.edu
# [db] 20160805 dan@cs.uml.edu
# bga_ibeacon.py
from threading import Lock, Thread


from batch_builder import BatchBuilder
import bga_constants, bglib, pprint, serial, struct, time
import sys
from lagerlogger import LagerLogger

class Spinner():
    def __init__(self):
        self.__blip = "|"
    def spin(self):
        p = { "|": "/",
              "/": "-",
              "-": "\\",
              "\\": "|"}
        self.__blip = p[self.__blip]
        print "\b\b"+self.__blip,
        sys.stdout.flush()
spinner = Spinner()


def threaderized(fun, *args, **kwargs):
    t = Thread(target=fun, args=args, kwargs=kwargs)
    t.start()

ADV_MAX = 69
ADV_MIN = 56
ADV_DATA = [
        # header stuff
        0x02, 0x01, 0x06,

        # uuid adword
        0x11, 0x06, 0x0, 0x0, 0x18, 0x0, 0x30, 0xde, 0x46, 0x30, 0x9b, 0x59,
        0x27, 0x22, 0x8d, 0x45, 0xbf, 0x11,

        # manufacturer adword
        0x08, 0xff, 0xd6, 0x9d,
            # visibility byte (hidden by default)
            0x01,
            # cache checksum adword (0 by default)
            0x0, 0x0, 0x0, 0x0]
VISIBILITY_BIT_INDEX = 25
CACHE_BIT_INDEX = 26

class BgSerial(object):
    pp = pprint.PrettyPrinter(indent=4)

    def __init__(self, payload_pipe, dev='/dev/bluegiga', debug=False):
        self._lock = Lock()
        self.logr = LagerLogger("")
        self.logr.console(LagerLogger.DEBUG)
        self._attributes_status = 0
        self._ble = bglib.BGLib()
        self._ble.packet_mode = False
        self._connected_addr = None
        self._conn_handle = None
        self._notified_handle = None
        self.adv_data = list(ADV_DATA) # clones list, initially set to be hidden

        # SHOULD BE STATELESS, WHAT IS THIS???????????
        # Used to retrieve:
        #   (for writing)
        #   - The total payload size, we need this at the low-level because the
        #     client requests this before every payload transfer.
        #   - The last batch sent. We need this because we need to resend missed
        #     packets, but we have no knowledge of the payload.
        #
        #   (for receiving)
        #   - sends the received messages
        self._payload_pipe = payload_pipe

        # SHOULD BE STATELESS, WHAT IS THIS???????????
        # Used to signal when the (bga server -> client) client signals the
        # entire batch was received successfully.
        self._is_batch_transfer_complete = False
        self._reading_batch = False
        self._batch_builder = None

        self._ser = serial.Serial(port=dev, baudrate=115200, timeout=1)
        self._ser.flushInput()
        self._ser.flushOutput()

        self._on_connect_cbs = []
        self._on_transfer_cbs = []
        self._on_receive_batch_cbs = []
        self._on_disconnect_cbs = []

        #self._ble.ble_evt_attributes_user_read_request += self._read_cb
        #self._ble.ble_evt_attributes_value += self._write_cb
        #self._ble.ble_evt_attributes_status += self._attributes_status_cb
        #self._ble.ble_evt_connection_disconnected += self._disconnect_cb
        #self._ble.ble_evt_connection_status += self._conn_interval_cb
        self._ble.ble_evt_attributes_user_read_request += lambda x,y: threaderized(self._read_cb, x, y)
        self._ble.ble_evt_attributes_value += lambda x,y: threaderized(self._write_cb, x, y)
        self._ble.ble_evt_attributes_status += lambda x,y: threaderized(self._attributes_status_cb, x, y)
        self._ble.ble_evt_connection_disconnected += lambda x,y: threaderized(self._disconnect_cb, x, y)
        self._ble.ble_evt_connection_status += lambda x,y: threaderized(self._conn_interval_cb, x, y)
#         if debug == True:
#             self._setup_debug_callbacks()

        self.stop()
        self._start_advertising()

    def _start_advertising(self):
        """Sets necessary attributes for advertising and begins advertising."""
        self._ble.send_command(
                self._ser,
                self._ble.ble_cmd_gap_set_adv_parameters(
                    ADV_MIN,
                    ADV_MAX,
                    7))
        time.sleep(bga_constants.NOTIFY_RATE)

        self.update_adv_data()

        self.logr.debug("Entering advertisement mode...")
        sys.stdout.flush()
        self._ble.send_command(
                self._ser,
                self._ble.ble_cmd_gap_set_mode(0x04, 0x02))
        time.sleep(bga_constants.NOTIFY_RATE)

    # takes a list of 4 bytes, with each byte represented in hex.
    def set_adv_cache_data(self, cache):
        if len(cache) == 4:
            for i in range(0, 4):
                self.adv_data[CACHE_BIT_INDEX + i] = cache[i]
            self.update_adv_data()

    def set_adv_visibility_data(self, visible):
        self.adv_data[VISIBILITY_BIT_INDEX] = int(not visible)
        self.update_adv_data()

    def update_adv_data(self):
        self._ble.send_command(
                self._ser,
                self._ble.ble_cmd_gap_set_adv_data(0, self.adv_data))
        time.sleep(bga_constants.NOTIFY_RATE)
#        try:
#            self.check_activity(self._ser, 1)
#        except serial.SerialException:
#            print "Warning: Unable to set advertising data at this time."

    def spin(self):
        while True:
            self.spin_once()

    def spin_once(self):
#         spinner.spin()
        self.check_activity(self._ser)
        # Timeout - are we missing frames?
        if self._reading_batch and self._batch_builder and self._batch_builder.is_waiting():
            self._request_missed_frames_or_proceed()

    def check_activity(self, *args):
        with self._lock:
            self._ble.check_activity(*args)

    def write_batch(self, batch):
        """Writes a batch out to the currently connected client. Blocks
        the thread until the client signals the entire batch was
        received successfully.
        """
        self.logr.debug("Writing Batch")
        sys.stdout.flush()
        for frame in batch:
            if self._notified_handle != None:
                self._ble.send_command(
                        self._ser,
                        self._ble.ble_cmd_attributes_write(
                            self._notified_handle,
                            0,
                            frame))

                #self._ble.check_activity(self._ser)
                time.sleep(bga_constants.NOTIFY_RATE*2)


        self.logr.debug("Waiting for transfer to complete")
        sys.stdout.flush()
        while (self._is_batch_transfer_complete == False
                and self._connected_addr != None):
            time.sleep(bga_constants.NOTIFY_RATE)
            #self._ble.check_activity(self._ser)
        self.logr.debug("transfer to complete")
        sys.stdout.flush()
        self._is_batch_transfer_complete = False
        return self._connected_addr != None

    def stop(self):
        """Stops connections, advertising, and scanning."""
        # Disconnects
        self._ble.send_command(
                self._ser,
                self._ble.ble_cmd_connection_disconnect(0))
        time.sleep(bga_constants.NOTIFY_RATE)

        # Stops advertising mode
        self._ble.send_command(self._ser, self._ble.ble_cmd_gap_set_mode(0, 0))
        time.sleep(bga_constants.NOTIFY_RATE)

        # Stops scanning
        self._ble.send_command(self._ser, self._ble.ble_cmd_gap_end_procedure())
        time.sleep(bga_constants.NOTIFY_RATE)

    def _read_cb(self, sender, args):
        """"When the client requests to read from the server, this is fired."""
        self.logr.debug("Got Read Callback")
        sys.stdout.flush()
        if args['handle'] == bga_constants.PKT_OUT_HANDLE:
            self.logr.debug("_read_cb() PKT_OUT_HANDLE")
            prepare_permission = self._payload_pipe.prepare_next_payload()
            payload_size = self._payload_pipe.get_payload_size()
            if prepare_permission and payload_size > 0:
                self.logr.debug("_read_cb() Sending payload size")
                sys.stdout.flush()
                self._ble.send_command(
                        self._ser,
                        self._ble.ble_cmd_attributes_user_read_response(
                            args['connection'],
                            0,
                            [ord(c) for c in
                                struct.pack(
                                    "!I",
                                    payload_size)]))
                #self.check_activity(self._ser, 1)
                time.sleep(bga_constants.NOTIFY_RATE*2)
                self._payload_pipe.send_next_payload()
            elif not prepare_permission:
                self.logr.debug("_read_cb() tried to read during a transfer")
                sys.stdout.flush()
                self._ble.send_command(
                        self._ser,
                        self._ble.ble_cmd_attributes_user_read_response(
                            args['connection'],
                            0,
                            [0, 0, 0, -1]))
                time.sleep(bga_constants.NOTIFY_RATE)
            else:
                self.logr.debug("_read_cb() Doing the other thing")
                sys.stdout.flush()
                self._ble.send_command(
                        self._ser,
                        self._ble.ble_cmd_attributes_user_read_response(
                            args['connection'],
                            0,
                            [0, 0, 0, 0]))
                time.sleep(bga_constants.NOTIFY_RATE)

    def _write_cb(self, sender, args):
        """When the client writes to the server, this is fired."""
        # Acknowledges any write_with_response messages.
        sys.stdout.flush()
        if args['reason'] == 2:
            self._ble.send_command(
                    self._ser,
                    self._ble.ble_cmd_attributes_user_write_response(
                        self._conn_handle, 0))
            time.sleep(bga_constants.NOTIFY_RATE)

        # Stores packets sent from the client.
        if args['handle'] == bga_constants.PKT_IN_HANDLE:
            if not self._reading_batch:
                self.logr.error(" ~!~!~!~!~ This should never happen ~!~!~!~!~!~")
                return
            # when batch transfer complete, notify the client
            if self._batch_builder.add_frame(args['value']):
                self._request_missed_frames_or_proceed()

        # Processes a missed_pkt request (from the client)
        elif args['handle'] == bga_constants.MISSED_PKT_OUT_HANDLE:
            if self._payload_pipe.get_payload_size() > 0:
                missed_frames = ''
                for b in args['value']:
                    missed_frames += bin(b)[2:].zfill(8)
                self._is_batch_transfer_complete = \
                        self._resend_missed_frames(missed_frames)

        # Reads the size of an upcoming payload (from the client).
        elif args['handle'] == bga_constants.PKT_IN_COUNT_HANDLE:
            # converts a little endian hex to decimal
            total = 0
            byte_mult = 1
            for b in args['value']:
                if b == 0:
                    continue
                else:
                    total += b * byte_mult
                    byte_mult <<= 8
            self._reading_batch = True
            self._batch_builder = BatchBuilder(total)
            for on_transfer_cb in self._on_transfer_cbs:
                on_transfer_cb(self._connected_addr, total)

    def _resend_missed_frames(self, missed_frames):
        if len(missed_frames) > 0 and missed_frames[0] == '0':
            return True
        last_batch = self._payload_pipe.get_last_batch_sent()
        frame_index = 0
        while frame_index < len(last_batch):
            if (missed_frames[frame_index + 1] == '1'):
                bit_enabled_missed_frame = last_batch[frame_index]
                bit_enabled_missed_frame[0] += 128
                self._ble.send_command(
                    self._ser,
                    self._ble.ble_cmd_attributes_write(
                        self._notified_handle,
                        0,
                        bit_enabled_missed_frame))
                time.sleep(bga_constants.NOTIFY_RATE)
            frame_index += 1
        return False

    def _attributes_status_cb(self, sender, args):
        """Fired when we a client subscribes through notify/indicate."""
        if args['handle'] == bga_constants.PKT_OUT_HANDLE:
            if args['flags'] == 0:
                self._notified_handle = None
            elif args['flags'] == bga_constants.NOTIFY_FLAG:
                self._notified_handle = args['handle']
                for on_connect_cb in self._on_connect_cbs:
                    on_connect_cb(self._connected_addr)
            else:
                self.logr.error("invalid attribute status flag")
                return
            self._attributes_status = args['flags']
        elif args['handle'] == bga_constants.MISSED_PKT_IN_HANDLE:
            if args['flags'] == bga_constants.INDICATE_FLAG:
                self._in_missed_frame_handle = args['handle']

    def _request_missed_frames_or_proceed(self):
        missed_frames_pkt = self._batch_builder.get_missed_frames_pkt()
        if missed_frames_pkt[0] == False:
            self.logr.debug("Batch Complete")
            batch = self._batch_builder.build()
            for on_receive_batch_cb in self._on_receive_batch_cbs:
                on_receive_batch_cb(self._connected_addr, batch)
            total_frames_remaining = self._payload_pipe.total_frames_remaining(self._connected_addr)
            if total_frames_remaining > 0:
                self._batch_builder = BatchBuilder(total_frames_remaining)
            else:
                self._batch_builder = None

        self._ble.send_command(
                self._ser,
                self._ble.ble_cmd_attributes_write(
                    self._in_missed_frame_handle,
                    0,
                    [ord(num) for num in list(missed_frames_pkt.tobytes())]))
        time.sleep(bga_constants.NOTIFY_RATE)

    def _disconnect_cb(self, sender, args):
        """Called when a connection ends, re-enables peripheral advertising."""
        self._ble.send_command(
                self._ser,
                self._ble.ble_cmd_gap_set_mode(0x04, 0x02))
        time.sleep(bga_constants.NOTIFY_RATE)
        self._conn_handle = None
        self._connected_addr = None
        self._notified_handle = None

        for disconnect_cb in self._on_disconnect_cbs:
            disconnect_cb()

    def _conn_interval_cb(self, sender, args):
        """Suggests low connection interval, the central ultimately decides."""
        low = 6
        high = 10
        if args['conn_interval'] > high or args['conn_interval'] < low:
            self._ble.send_command(
                    self._ser,
                    self._ble.ble_cmd_connection_update(
                        0,
                        low,
                        high,
                        0,
                        2000))
            time.sleep(bga_constants.NOTIFY_RATE)
        self._conn_handle = args['connection']
        str_address = BgSerial._address_arr_to_str(args['address'])
        if self._connected_addr != str_address:
            self.logr.debug("Connected (actually)")
            self._connected_addr = str_address

    @classmethod
    def _debug_args_cb(cls, sender, args):
        """Prints all info about a RSP/EVT in a JSON-like format."""
        cls.pp.pprint(args)

    def subscribe_to_connect(self, on_connect_cb):
        if callable(on_connect_cb):
            self._on_connect_cbs.append(on_connect_cb)
        else:
            raise TypeError('on_connect callback provided was not a function.')

    def subscribe_to_transfer(self, on_transfer_cb):
        if callable(on_transfer_cb):
            self._on_transfer_cbs.append(on_transfer_cb)
        else:
            raise TypeError('on_transfer callback provided was not a function.')

    def subscribe_to_receive_batch(self, on_receive_batch_cb):
        if callable(on_receive_batch_cb):
            self._on_receive_batch_cbs.append(on_receive_batch_cb)
        else:
            raise TypeError(
                    'on_receive_batch callback provided was not a function.')

    def subscribe_to_disconnect(self, on_disconnect_cb):
        if callable(on_disconnect_cb):
            self._on_disconnect_cbs.append(on_disconnect_cb)
        else:
            raise TypeError(
                    'on_disconnect callback provided was not a function.')

    @staticmethod
    def _address_arr_to_str(address):
        str_address = ''
        for byte in address:
            str_byte = str(hex(byte))[2:]
            if len(str_byte) == 1:
                str_byte = '0' + str_byte
            if str_address != '':
                str_byte += ':'
            str_address = str_byte + str_address
        return str_address

    def _setup_debug_callbacks(self):
        """Registers the debug callback for all RSPs and EVTs."""
        self._ble.ble_rsp_system_reset += self._debug_args_cb
        self._ble.ble_rsp_system_hello += self._debug_args_cb
        self._ble.ble_rsp_system_address_get += self._debug_args_cb
        self._ble.ble_rsp_system_reg_write += self._debug_args_cb
        self._ble.ble_rsp_system_reg_read += self._debug_args_cb
        self._ble.ble_rsp_system_get_counters += self._debug_args_cb
        self._ble.ble_rsp_system_get_connections += self._debug_args_cb
        self._ble.ble_rsp_system_read_memory += self._debug_args_cb
        self._ble.ble_rsp_system_get_info += self._debug_args_cb
        self._ble.ble_rsp_system_endpoint_tx += self._debug_args_cb
        self._ble.ble_rsp_system_whitelist_append += self._debug_args_cb
        self._ble.ble_rsp_system_whitelist_remove += self._debug_args_cb
        self._ble.ble_rsp_system_whitelist_clear += self._debug_args_cb
        self._ble.ble_rsp_system_endpoint_rx += self._debug_args_cb
        self._ble.ble_rsp_system_endpoint_set_watermarks += self._debug_args_cb
        self._ble.ble_rsp_flash_ps_defrag += self._debug_args_cb
        self._ble.ble_rsp_flash_ps_dump += self._debug_args_cb
        self._ble.ble_rsp_flash_ps_erase_all += self._debug_args_cb
        self._ble.ble_rsp_flash_ps_save += self._debug_args_cb
        self._ble.ble_rsp_flash_ps_load += self._debug_args_cb
        self._ble.ble_rsp_flash_ps_erase += self._debug_args_cb
        self._ble.ble_rsp_flash_erase_page += self._debug_args_cb
        self._ble.ble_rsp_flash_write_words += self._debug_args_cb
        self._ble.ble_rsp_attributes_write += self._debug_args_cb
        self._ble.ble_rsp_attributes_read += self._debug_args_cb
        self._ble.ble_rsp_attributes_read_type += self._debug_args_cb
        self._ble.ble_rsp_attributes_user_read_response += self._debug_args_cb
        self._ble.ble_rsp_attributes_user_write_response += self._debug_args_cb
        self._ble.ble_rsp_connection_disconnect += self._debug_args_cb
        self._ble.ble_rsp_connection_get_rssi += self._debug_args_cb
        self._ble.ble_rsp_connection_update += self._debug_args_cb
        self._ble.ble_rsp_connection_version_update += self._debug_args_cb
        self._ble.ble_rsp_connection_channel_map_get += self._debug_args_cb
        self._ble.ble_rsp_connection_channel_map_set += self._debug_args_cb
        self._ble.ble_rsp_connection_features_get += self._debug_args_cb
        self._ble.ble_rsp_connection_get_status += self._debug_args_cb
        self._ble.ble_rsp_connection_raw_tx += self._debug_args_cb
        self._ble.ble_rsp_attclient_find_by_type_value += self._debug_args_cb
        self._ble.ble_rsp_attclient_read_by_group_type += self._debug_args_cb
        self._ble.ble_rsp_attclient_read_by_type += self._debug_args_cb
        self._ble.ble_rsp_attclient_find_information += self._debug_args_cb
        self._ble.ble_rsp_attclient_read_by_handle += self._debug_args_cb
        self._ble.ble_rsp_attclient_attribute_write += self._debug_args_cb
        self._ble.ble_rsp_attclient_write_command += self._debug_args_cb
        self._ble.ble_rsp_attclient_indicate_confirm += self._debug_args_cb
        self._ble.ble_rsp_attclient_read_long += self._debug_args_cb
        self._ble.ble_rsp_attclient_prepare_write += self._debug_args_cb
        self._ble.ble_rsp_attclient_execute_write += self._debug_args_cb
        self._ble.ble_rsp_attclient_read_multiple += self._debug_args_cb
        self._ble.ble_rsp_sm_encrypt_start += self._debug_args_cb
        self._ble.ble_rsp_sm_set_bondable_mode += self._debug_args_cb
        self._ble.ble_rsp_sm_delete_bonding += self._debug_args_cb
        self._ble.ble_rsp_sm_set_parameters += self._debug_args_cb
        self._ble.ble_rsp_sm_passkey_entry += self._debug_args_cb
        self._ble.ble_rsp_sm_get_bonds += self._debug_args_cb
        self._ble.ble_rsp_sm_set_oob_data += self._debug_args_cb
        self._ble.ble_rsp_gap_set_privacy_flags += self._debug_args_cb
        self._ble.ble_rsp_gap_set_mode += self._debug_args_cb
        self._ble.ble_rsp_gap_discover += self._debug_args_cb
        self._ble.ble_rsp_gap_connect_direct += self._debug_args_cb
        self._ble.ble_rsp_gap_end_procedure += self._debug_args_cb
        self._ble.ble_rsp_gap_connect_selective += self._debug_args_cb
        self._ble.ble_rsp_gap_set_filtering += self._debug_args_cb
        self._ble.ble_rsp_gap_set_scan_parameters += self._debug_args_cb
        self._ble.ble_rsp_gap_set_adv_parameters += self._debug_args_cb
        self._ble.ble_rsp_gap_set_adv_data += self._debug_args_cb
        self._ble.ble_rsp_gap_set_directed_connectable_mode += self._debug_args_cb
        self._ble.ble_rsp_hardware_io_port_config_irq += self._debug_args_cb
        self._ble.ble_rsp_hardware_set_soft_timer += self._debug_args_cb
        self._ble.ble_rsp_hardware_adc_read += self._debug_args_cb
        self._ble.ble_rsp_hardware_io_port_config_direction += self._debug_args_cb
        self._ble.ble_rsp_hardware_io_port_config_function += self._debug_args_cb
        self._ble.ble_rsp_hardware_io_port_config_pull += self._debug_args_cb
        self._ble.ble_rsp_hardware_io_port_write += self._debug_args_cb
        self._ble.ble_rsp_hardware_io_port_read += self._debug_args_cb
        self._ble.ble_rsp_hardware_spi_config += self._debug_args_cb
        self._ble.ble_rsp_hardware_spi_transfer += self._debug_args_cb
        self._ble.ble_rsp_hardware_i2c_read += self._debug_args_cb
        self._ble.ble_rsp_hardware_i2c_write += self._debug_args_cb
        self._ble.ble_rsp_hardware_set_txpower += self._debug_args_cb
        self._ble.ble_rsp_hardware_timer_comparator += self._debug_args_cb
        self._ble.ble_rsp_test_phy_tx += self._debug_args_cb
        self._ble.ble_rsp_test_phy_rx += self._debug_args_cb
        self._ble.ble_rsp_test_phy_end += self._debug_args_cb
        self._ble.ble_rsp_test_phy_reset += self._debug_args_cb
        self._ble.ble_rsp_test_get_channel_map += self._debug_args_cb
        self._ble.ble_rsp_test_debug += self._debug_args_cb

        self._ble.ble_evt_system_boot += self._debug_args_cb
        self._ble.ble_evt_system_debug += self._debug_args_cb
        self._ble.ble_evt_system_endpoint_watermark_rx += self._debug_args_cb
        self._ble.ble_evt_system_endpoint_watermark_tx += self._debug_args_cb
        self._ble.ble_evt_system_script_failure += self._debug_args_cb
        self._ble.ble_evt_system_no_license_key += self._debug_args_cb
        self._ble.ble_evt_flash_ps_key += self._debug_args_cb
        self._ble.ble_evt_attributes_value += self._debug_args_cb
        self._ble.ble_evt_attributes_user_read_request += self._debug_args_cb
        self._ble.ble_evt_attributes_status += self._debug_args_cb
        self._ble.ble_evt_connection_status += self._debug_args_cb
        self._ble.ble_evt_connection_version_ind += self._debug_args_cb
        self._ble.ble_evt_connection_feature_ind += self._debug_args_cb
        self._ble.ble_evt_connection_raw_rx += self._debug_args_cb
        self._ble.ble_evt_connection_disconnected += self._debug_args_cb
        self._ble.ble_evt_attclient_indicated += self._debug_args_cb
        self._ble.ble_evt_attclient_procedure_completed += self._debug_args_cb
        self._ble.ble_evt_attclient_group_found += self._debug_args_cb
        self._ble.ble_evt_attclient_attribute_found += self._debug_args_cb
        self._ble.ble_evt_attclient_find_information_found += self._debug_args_cb
        self._ble.ble_evt_attclient_attribute_value += self._debug_args_cb
        self._ble.ble_evt_attclient_read_multiple_response += self._debug_args_cb
        self._ble.ble_evt_sm_smp_data += self._debug_args_cb
        self._ble.ble_evt_sm_bonding_fail += self._debug_args_cb
        self._ble.ble_evt_sm_passkey_display += self._debug_args_cb
        self._ble.ble_evt_sm_passkey_request += self._debug_args_cb
        self._ble.ble_evt_sm_bond_status += self._debug_args_cb
        self._ble.ble_evt_gap_mode_changed += self._debug_args_cb
        self._ble.ble_evt_hardware_io_port_status += self._debug_args_cb
        self._ble.ble_evt_hardware_soft_timer += self._debug_args_cb
        self._ble.ble_evt_hardware_adc_result += self._debug_args_cb
        self._ble.ble_evt_gap_scan_response += self._debug_args_cb
