#!/usr/bin/env python3
"""
hello.py - send one HELLO frame and wait for the ACK.

The smallest possible exercise of the framing: one frame out, one frame in.

    .venv/bin/python tools/hello.py /dev/tty.usbmodem1103
    .venv/bin/python tools/hello.py /dev/tty.usbmodem1103 --corrupt

--corrupt flips one CRC bit; the device should drop the frame and reply with
nothing, then still accept the next good one.

Must be sent inside the bootloader's 3 s opening window -- press reset first.
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import g4b_frame as fr                                       # noqa: E402


def main():
    ap = argparse.ArgumentParser(description="send HELLO, expect ACK")
    ap.add_argument("port", help="e.g. /dev/tty.usbmodem1103")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--corrupt", action="store_true",
                    help="flip a CRC bit; the device should drop the frame")
    args = ap.parse_args()

    with fr.open_port(args.port, args.baud) as ser:
        frame = fr.build(fr.MSG_HELLO, corrupt=args.corrupt)
        ser.write(frame)
        print("sent  HELLO%s  %s"
              % (" (corrupted)" if args.corrupt else "", frame.hex()))

        got = fr.read_frame(ser)
        print("recv  %s" % fr.describe(got))

        if args.corrupt:
            return 0 if got is None else 1
        return 0 if got and got[0] == fr.MSG_ACK else 1


if __name__ == "__main__":
    sys.exit(main())
