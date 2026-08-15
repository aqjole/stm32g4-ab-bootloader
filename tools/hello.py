#!/usr/bin/env python3
"""
hello.py - send one HELLO frame and wait for the ACK.

The smallest possible exercise of the framing: one frame out, one frame in.
Everything in S3 sits on top of this working.

    python3 tools/hello.py /dev/tty.usbmodem11403
    python3 tools/hello.py /dev/tty.usbmodem11403 --corrupt   # expect no reply

Frame:  [SOF][len lo][len hi][type][payload...][crc32 le]
        len counts payload only; the CRC covers len + type + payload.

Constants must match shared/g4b_proto.h. Change one, change both.
"""

import argparse
import struct
import sys
import zlib

try:
    import serial
except ImportError:
    sys.exit("error: pyserial missing. Run:\n"
             "  python3 -m venv .venv && .venv/bin/pip install pyserial\n"
             "then use .venv/bin/python instead of python3")

SOF = 0x7E
MAX_PAYLOAD = 256

MSG_HELLO = 0x01
MSG_BEGIN = 0x02
MSG_CHUNK = 0x03
MSG_END = 0x04
MSG_ACK = 0x80
MSG_NACK = 0x81

NAMES = {MSG_HELLO: "HELLO", MSG_BEGIN: "BEGIN", MSG_CHUNK: "CHUNK",
         MSG_END: "END", MSG_ACK: "ACK", MSG_NACK: "NACK"}

NACK_REASONS = {0x01: "bad crc", 0x02: "bad len", 0x03: "bad type",
                0x04: "bad seq", 0x05: "flash error", 0x06: "not ready"}


def build(msg_type, payload=b"", corrupt=False):
    """Frame `payload` as `msg_type`. corrupt=True flips a CRC bit."""
    body = struct.pack("<HB", len(payload), msg_type) + payload
    crc = zlib.crc32(body) & 0xFFFFFFFF
    if corrupt:
        crc ^= 1
    return bytes([SOF]) + body + struct.pack("<I", crc)


def read_frame(ser):
    """Return (type, payload), or None once the port goes quiet.

    Hunts for SOF exactly like the device. The board's human-readable log
    shares this wire, so most of what arrives is not a frame at all.

    A false sync -- a 0x7E that was payload or log text rather than a real
    SOF -- must resume hunting, not give up: the genuine frame may be right
    behind it. Only running out of input ends the search.
    """
    while True:
        b = ser.read(1)
        if not b:
            return None                       # port quiet: nothing more coming
        if b[0] != SOF:
            continue                          # not a frame start

        hdr = ser.read(3)
        if len(hdr) < 3:
            return None
        length, msg_type = struct.unpack("<HB", hdr)

        if length > MAX_PAYLOAD:
            continue                          # false sync -> resync

        payload = ser.read(length) if length else b""
        if len(payload) < length:
            return None

        raw = ser.read(4)
        if len(raw) < 4:
            return None
        if struct.unpack("<I", raw)[0] != (zlib.crc32(hdr + payload) & 0xFFFFFFFF):
            continue                          # corrupt or false sync -> resync

        return msg_type, payload


def main():
    ap = argparse.ArgumentParser(description="send HELLO, expect ACK")
    ap.add_argument("port", help="e.g. /dev/tty.usbmodem11403")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--corrupt", action="store_true",
                    help="flip a CRC bit; the device should drop the frame")
    args = ap.parse_args()

    with serial.Serial(args.port, args.baud, timeout=1) as ser:
        frame = build(MSG_HELLO, corrupt=args.corrupt)
        ser.write(frame)
        print("sent  HELLO%s  %s" % (" (corrupted)" if args.corrupt else "",
                                     frame.hex()))

        got = read_frame(ser)
        if got is None:
            print("recv  nothing")
            return 1 if not args.corrupt else 0

        msg_type, payload = got
        name = NAMES.get(msg_type, "0x%02X" % msg_type)
        if msg_type == MSG_NACK and payload:
            print("recv  NACK (%s)" % NACK_REASONS.get(payload[0], "?"))
        else:
            print("recv  %s%s" % (name, "  " + payload.hex() if payload else ""))
        return 0 if msg_type == MSG_ACK else 1


if __name__ == "__main__":
    sys.exit(main())
