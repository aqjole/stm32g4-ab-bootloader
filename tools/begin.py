#!/usr/bin/env python3
"""
begin.py - send BEGIN and wait for the slot erase to finish.

BEGIN's payload is the 32-byte image header pack.py already wrote, taken
straight off the front of the .img -- no new format, and the device runs the
same checks it uses when validating a slot at boot.

    .venv/bin/python tools/begin.py /dev/tty.usbmodem1103 app_b.img

The device chooses the target slot itself: always the one that is NOT active.
So hand it the .img linked for that slot. With active=A, that is app_b.img.

Must be sent inside the bootloader's 3 s opening window -- press reset first.
"""

import argparse
import os
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import g4b_frame as fr                                       # noqa: E402

HDR_SIZE = 32
HDR_STRUCT = "<IHHIIII8s"


def show_header(hdr):
    magic, hdr_ver, flags, img_len, entry, img_ver, crc, _ = \
        struct.unpack(HDR_STRUCT, hdr)
    print("  magic    0x%08X %s" % (magic, "ok" if magic == 0x54423447 else "BAD"))
    print("  version  %d.%d.%d   (header format v%d)"
          % ((img_ver >> 16) & 0xFF, (img_ver >> 8) & 0xFF, img_ver & 0xFF, hdr_ver))
    print("  img_len  %d bytes" % img_len)
    print("  entry    0x%08X" % entry)
    print("  crc32    0x%08X" % crc)
    return img_len


def main():
    ap = argparse.ArgumentParser(description="send BEGIN, wait for slot erase")
    ap.add_argument("port", help="e.g. /dev/tty.usbmodem1103")
    ap.add_argument("image", help="the .img whose header to announce")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--timeout", type=float, default=10.0,
                    help="erasing 26 pages takes about a second; be generous")
    args = ap.parse_args()

    with open(args.image, "rb") as f:
        hdr = f.read(HDR_SIZE)
    if len(hdr) < HDR_SIZE:
        sys.exit("error: %s is shorter than a header" % args.image)

    print("announcing %s:" % args.image)
    show_header(hdr)
    print()

    with fr.open_port(args.port, args.baud, timeout=args.timeout) as ser:
        ser.write(fr.build(fr.MSG_BEGIN, hdr))
        print("sent  BEGIN (%d B payload)" % len(hdr))

        t0 = time.monotonic()
        got = fr.read_frame(ser)
        dt = time.monotonic() - t0

        print("recv  %s   after %.2f s" % (fr.describe(got), dt))

        if got is None:
            print("\nno reply. Did you press reset first? The device only listens")
            print("for 3 s after boot.")
            return 1
        return 0 if got[0] == fr.MSG_ACK else 1


if __name__ == "__main__":
    sys.exit(main())
