#!/usr/bin/env python3
"""
update.py - BEGIN, then stream the whole .img in CHUNKs.

    .venv/bin/python tools/update.py /dev/tty.usbmodem1103 app_b.img

Each chunk is [seq u16 le][<=248 B data]; the device computes the flash
address as seq*248 and ACKs. One retry per chunk covers a lost ACK -- the
device recognises the duplicate and re-ACKs without touching flash.

--dup N resends chunk N once after its ACK, to prove the duplicate path.
Must start inside the 3 s window -- press reset first.
"""

import argparse
import os
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import g4b_frame as fr                                       # noqa: E402

CHUNK_DATA = 248        # must match G4B_CHUNK_DATA


def send_chunk(ser, seq, data):
    ser.write(fr.build(fr.MSG_CHUNK, struct.pack("<H", seq) + data))
    return fr.read_frame(ser)


def main():
    ap = argparse.ArgumentParser(description="BEGIN + stream CHUNKs")
    ap.add_argument("port")
    ap.add_argument("image")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--dup", type=int, default=None,
                    help="resend this chunk once (device should just re-ACK)")
    ap.add_argument("--stop-after", type=int, default=None,
                    help="send only N chunks, then END -- device must NACK")
    ap.add_argument("--boot", action="store_true",
                    help="after END ACKs, send BOOT: set pending and reset")
    args = ap.parse_args()

    with open(args.image, "rb") as f:
        img = f.read()
    print("image %s: %d bytes = %d chunks"
          % (args.image, len(img), (len(img) + CHUNK_DATA - 1) // CHUNK_DATA))

    with fr.open_port(args.port, args.baud, timeout=10.0) as ser:
        ser.write(fr.build(fr.MSG_BEGIN, img[:32]))
        got = fr.read_frame(ser)
        print("BEGIN -> %s" % fr.describe(got))
        if got is None or got[0] != fr.MSG_ACK:
            return 1

        t0 = time.monotonic()

        for seq in range(0, (len(img) + CHUNK_DATA - 1) // CHUNK_DATA):
            if args.stop_after is not None and seq >= args.stop_after:
                print("stopping early at chunk %d" % seq)
                break
            data = img[seq * CHUNK_DATA:(seq + 1) * CHUNK_DATA]

            got = send_chunk(ser, seq, data)
            if got is None:                      # lost frame or lost ACK
                print("chunk %d: no reply, retrying" % seq)
                got = send_chunk(ser, seq, data)

            if got is None or got[0] != fr.MSG_ACK:
                print("chunk %d -> %s" % (seq, fr.describe(got)))
                return 1

            if args.dup is not None and seq == args.dup:
                got = send_chunk(ser, seq, data)
                print("chunk %d duplicate -> %s (flash untouched)"
                      % (seq, fr.describe(got)))
                if got is None or got[0] != fr.MSG_ACK:
                    return 1

        dt = time.monotonic() - t0
        print("streamed in %.2f s" % dt)

        ser.write(fr.build(fr.MSG_END))
        got = fr.read_frame(ser)
        print("END  -> %s" % fr.describe(got))

        ok = got is not None and got[0] == fr.MSG_ACK

        if ok and args.boot:
            ser.write(fr.build(fr.MSG_BOOT))
            got = fr.read_frame(ser)
            print("BOOT -> %s" % fr.describe(got))
            ok = got is not None and got[0] == fr.MSG_ACK

        if args.stop_after is not None:
            return 0 if not ok else 1      # incomplete: NACK is the pass
        return 0 if ok else 1

if __name__ == "__main__":
    sys.exit(main())