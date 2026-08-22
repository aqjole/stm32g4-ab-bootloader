#!/usr/bin/env python3
"""
uds_update.py - flash the inactive slot over CAN as a UDS programming sequence.

    .venv/bin/python tools/uds_update.py /dev/tty.usbmodem208836A54B341 app_a.img --reset

ISO 14229 over ISO 15765-2: 0x10 programming session, 0x34 RequestDownload
(address = slot base; the device refuses any other address), 0x36
TransferData with block sequence counters, 0x37 RequestTransferExit (the
device verifies from flash), and with --reset a 0x11 ECUReset that arms the
A/B gamble exactly like BOOT did.

Start the command, then press reset within a second or two -- the device
listens for 3 s after boot.

Honest note: no 0x27 SecurityAccess. Real ECUs gate all of this behind a
seed/key exchange.
"""

import argparse
import os
import struct
import sys
import time

SLOT_A = 0x08005000
SLOT_B = 0x08012000


def main():
    ap = argparse.ArgumentParser(description="UDS programming sequence over CAN")
    ap.add_argument("port", help="slcan adapter, e.g. /dev/tty.usbmodem...")
    ap.add_argument("image", help=".img to download (header included)")
    ap.add_argument("--reset", action="store_true",
                    help="after 0x37, send 0x11 ECUReset: set pending and gamble")
    args = ap.parse_args()

    import can
    import isotp
    import udsoncan
    from udsoncan.client import Client
    from udsoncan.connections import PythonIsoTpConnection
    from udsoncan.services import ECUReset
    from udsoncan import MemoryLocation

    with open(args.image, "rb") as f:
        img = f.read()

    # the header's entry field names the slot this image was linked for;
    # the device independently enforces inactive-slot-only
    entry = struct.unpack_from("<I", img, 12)[0]
    slot = SLOT_A if SLOT_A <= entry < SLOT_B else SLOT_B
    print("image %s: %d bytes, linked for slot %s (0x%08X)"
          % (args.image, len(img), "A" if slot == SLOT_A else "B", slot))

    bus = can.Bus(interface="slcan", channel=args.port, bitrate=500000,
                  sleep_after_open=0.3)
    addr = isotp.Address(isotp.AddressingMode.Normal_11bits,
                         txid=0x7E0, rxid=0x7E8)
    stack = isotp.CanStack(bus=bus, address=addr,
                           params={"stmin": 0, "blocksize": 0,
                                   "rx_flowcontrol_timeout": 1000,
                                   "rx_consecutive_frame_timeout": 1000})
    conn = PythonIsoTpConnection(stack)

    config = dict(udsoncan.configs.default_client_config)
    config["p2_timeout"] = 2.0        # normal responses
    config["p2_star_timeout"] = 6.0   # after 0x78: the erase runs here
    config["exception_on_negative_response"] = True

    from udsoncan.exceptions import NegativeResponseException, TimeoutException
    try:
        run_session(Client, MemoryLocation, ECUReset, conn, config,
                    img, slot, args.reset)
    except NegativeResponseException as e:
        # the device saying "no" properly is a protocol outcome, not a crash
        print("refused: %s (0x%02X)"
              % (e.response.code_name, e.response.code))
        return 1
    except TimeoutException:
        print("no response. Did you press reset? The device listens 3 s after boot.")
        return 1
    finally:
        bus.shutdown()
    return 0


def run_session(Client, MemoryLocation, ECUReset, conn, config,
                img, slot, do_reset):
    with Client(conn, config=config) as client:
        client.change_session(2)
        print("0x10 -> programming session")

        resp = client.request_download(MemoryLocation(
            address=slot, memorysize=len(img),
            address_format=32, memorysize_format=32))
        block = resp.service_data.max_length - 2
        block -= block % 8
        print("0x34 -> ACK after erase, block %d B" % block)

        t0 = time.monotonic()
        seq = 1
        for i in range(0, len(img), block):
            client.transfer_data(seq & 0xFF, img[i:i + block])
            seq += 1
        dt = time.monotonic() - t0
        print("0x36 x%d -> %d B in %.2f s (%.1f KB/s)"
              % (seq - 1, len(img), dt, len(img) / dt / 1024.0))

        client.request_transfer_exit()
        print("0x37 -> verified from flash")

        if do_reset:
            client.ecu_reset(ECUReset.ResetType.hardReset)
            print("0x11 -> pending set, device resetting: the gamble is on")


if __name__ == "__main__":
    sys.exit(main())
