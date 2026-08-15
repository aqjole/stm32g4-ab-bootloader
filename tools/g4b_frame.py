#!/usr/bin/env python3
"""
g4b_frame.py - the wire protocol, host side.

Shared by hello.py, begin.py and update.py. 

Frame:  [SOF][len lo][len hi][type][payload...][crc32 le]
        len counts payload only; the CRC covers len + type + payload, not the
        SOF, which is a resync marker rather than data.
"""

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
    """Return (type, payload), or None once the port goes quiet."""
    while True:
        b = ser.read(1)
        if not b:
            return None
        if b[0] != SOF:
            continue

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


def describe(got):
    """Human-readable one-liner for a (type, payload) tuple, or None."""
    if got is None:
        return "nothing"
    msg_type, payload = got
    if msg_type == MSG_NACK and payload:
        return "NACK (%s)" % NACK_REASONS.get(payload[0], "reason 0x%02X" % payload[0])
    name = NAMES.get(msg_type, "0x%02X" % msg_type)
    return name + ("  " + payload.hex() if payload else "")


def open_port(port, baud=115200, timeout=1.0):
    return serial.Serial(port, baud, timeout=timeout)
