"""Basic constants for the bluegigga client/server"""

# Modifies how payloads are constructed and at what rate.
FRAME_SIZE = 20
BATCH_SIZE = 128
NOTIFY_RATE = 0.01

# Flag's defined by bglib manual
NOTIFY_FLAG = 1
INDICATE_FLAG = 2

# GATT characteristic's handle
PKT_OUT_HANDLE = 15
MISSED_PKT_OUT_HANDLE = 19
PKT_IN_HANDLE = 23
MISSED_PKT_IN_HANDLE = 26
PKT_IN_COUNT_HANDLE = 30
