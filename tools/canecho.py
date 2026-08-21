#!/usr/bin/env python3
"""Bench echo node for S4 step 1.

Opens the slcan adapter at 500 kbit/s, prints every frame on the bus,
and echoes the board's 0x100 frame back with the same payload so the
bootloader's round-trip check passes over the real wire.
"""
import sys

import can

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/tty.usbmodem208836A54B341"

bus = can.Bus(interface="slcan", channel=PORT, bitrate=500000)
print(f"listening on {PORT} at 500 kbit/s, ctrl-C to stop")
try:
    while True:
        msg = bus.recv()
        print(msg)
        if msg.arbitration_id == 0x100:
            bus.send(can.Message(arbitration_id=0x100, data=msg.data,
                                 is_extended_id=False))
            print("echoed back")
except KeyboardInterrupt:
    pass
finally:
    bus.shutdown()
