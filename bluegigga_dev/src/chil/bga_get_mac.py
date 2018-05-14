#!/usr/bin/env python
# [db] 20160805 dan@cs.uml.edu 
# Retreives the MAC address from a Bluegigga device
import bglib, serial, time, argparse

def ble_cb(sender, args):
    print ":".join("%02x" % b for b in args["address"])

def main(devpath):
    port = devpath
    baud = 115200
    packet_mode = False
    ble = bglib.BGLib()
    ble.packet_mode = packet_mode
    ble.ble_rsp_system_address_get += ble_cb
    
    ser = serial.Serial(port=port, baudrate=baud, timeout=1)
    ser.flushInput()
    ser.flushOutput()

    ble.send_command(ser, ble.ble_cmd_system_address_get())
    ble.check_activity(ser, 1)
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser("bga_get_mac.py")
    parser.add_argument('--dev', type=str, default='/dev/ttyACM0')
    args = parser.parse_args()
    main(args.dev)
