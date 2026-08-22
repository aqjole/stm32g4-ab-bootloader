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
MSG_BOOT = 0x05
MSG_ACK = 0x80
MSG_NACK = 0x81

NAMES = {MSG_HELLO: "HELLO", MSG_BEGIN: "BEGIN", MSG_CHUNK: "CHUNK",
         MSG_END: "END", MSG_BOOT: "BOOT", MSG_ACK: "ACK", MSG_NACK: "NACK"}

NACK_REASONS = {0x01: "bad crc", 0x02: "bad len", 0x03: "bad type",
                0x04: "bad seq", 0x05: "flash error", 0x06: "not ready",
                0x07: "wrong slot"}


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


CAN_ID_HOST2DEV = 0x7E0        # tester -> ECU, the classic UDS pair
CAN_ID_DEV2HOST = 0x7E8        # response id = request id + 8


class IsotpPipe:
    """ISO-TP (ISO 15765-2) over the slcan adapter, dressed as a serial port.

    write() sends one ISO-TP message; the can-isotp stack handles SF/FF/CF
    and obeys the device's flow control. read(n) hands out received message
    bytes with pyserial semantics -- waits up to `timeout`, may return fewer.
    """

    def __init__(self, channel, bitrate=500000, timeout=1.0):
        try:
            import can
            import isotp
        except ImportError:
            sys.exit("error: python-can or can-isotp missing. Run:\n"
                     "  .venv/bin/pip install python-can can-isotp")
        # default sleep_after_open is a hard 2 s, for adapters that reboot
        # on serial open; the CANable does not, and 2 s eats most of the
        # bootloader's 3 s listening window
        self.bus = can.Bus(interface="slcan", channel=channel,
                           bitrate=bitrate, sleep_after_open=0.3)
        addr = isotp.Address(isotp.AddressingMode.Normal_11bits,
                             txid=CAN_ID_HOST2DEV, rxid=CAN_ID_DEV2HOST)
        self.stack = isotp.CanStack(bus=self.bus, address=addr,
                                    params={"stmin": 0, "blocksize": 0,
                                            "rx_flowcontrol_timeout": 1000,
                                            "rx_consecutive_frame_timeout": 1000})
        self.timeout = timeout
        self._buf = bytearray()

    def write(self, data):
        import time
        self.stack.send(bytes(data))
        deadline = time.monotonic() + 5.0
        # no sleep: process() paces itself off the device's STmin (0 here),
        # and any sleep multiplied by ~37 CFs per chunk caps the throughput
        while self.stack.transmitting() and time.monotonic() < deadline:
            self.stack.process()

    def read(self, n=1):
        import time
        deadline = time.monotonic() + self.timeout
        while len(self._buf) < n and time.monotonic() < deadline:
            self.stack.process()
            msg = self.stack.recv()
            if msg is not None:
                self._buf.extend(msg)
            else:
                time.sleep(0.0005)
        out = bytes(self._buf[:n])
        del self._buf[:n]
        return out

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.bus.shutdown()


def open_can(channel, bitrate=500000, timeout=1.0):
    return IsotpPipe(channel, bitrate, timeout)
