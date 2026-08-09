#!/usr/bin/env python3
"""
pack.py - G4Boot image packer.

Takes a raw application .bin produced by objcopy, pads it, computes a CRC-32,
and prepends a 512-byte header region so the result can be written straight
into a slot.

    python pack.py app.bin --slot A --version 1.0.0 -o app_a.img
    python pack.py --info app_a.img

Constants below MUST match image_header.h. If you change one, change both.
"""

import argparse
import struct
import sys
import zlib

HDR_MAGIC = 0x54423447          # "G4BT" as stored, little-endian
HDR_VERSION = 1
HDR_RESERVED = 512              # bytes reserved at the slot start
HDR_STRUCT = "<IHHIIII8s"       # magic, hdr_ver, flags, len, entry, ver, crc, reserved[2]
HDR_SIZE = struct.calcsize(HDR_STRUCT)   # 32

SLOT_SIZE = 52 * 1024
APP_MAX = SLOT_SIZE - HDR_RESERVED       # 52736

SLOT_BASE = {"A": 0x08005000, "B": 0x08012000}
ERASED = 0xFF

# reserved[2] stays erased, not zeroed. Flash only clears bits, so writing 0x00
# here would make these growth fields unprogrammable without a page erase.
RESERVED_FILL = bytes([ERASED]) * 8

assert HDR_SIZE == 32, "header struct drifted from image_header.h"


def parse_version(text):
    parts = text.split(".")
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("version must be major.minor.patch")
    try:
        maj, minr, pat = (int(p) for p in parts)
    except ValueError:
        raise argparse.ArgumentTypeError("version fields must be integers")
    for p in (maj, minr, pat):
        if not 0 <= p <= 255:
            raise argparse.ArgumentTypeError("version fields must be 0..255")
    return (maj << 16) | (minr << 8) | pat


def fmt_version(v):
    return "%d.%d.%d" % ((v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF)


def pack(args):
    payload = bytearray(open(args.binary, "rb").read())
    raw_len = len(payload)

    if raw_len < 8:
        sys.exit("error: %s is only %d bytes - that is not an image"
                 % (args.binary, raw_len))

    # Flash programs 64 bits at a time. Pad with the erased value so the
    # padding costs nothing to write and both sides checksum identical bytes.
    while len(payload) % 8:
        payload.append(ERASED)

    if len(payload) > APP_MAX:
        sys.exit("error: image is %d bytes, slot holds %d"
                 % (len(payload), APP_MAX))

    msp, entry = struct.unpack("<II", bytes(payload[:8]))
    app_base = SLOT_BASE[args.slot] + HDR_RESERVED
    target = entry & ~1

    if not (entry & 1):
        sys.exit("error: entry 0x%08X has bit 0 clear - not a Thumb address. "
                 "Wrong file, or not a Cortex-M image." % entry)

    if not app_base <= target < app_base + len(payload):
        sys.exit("error: entry 0x%08X is outside slot %s (0x%08X..0x%08X).\n"
                 "       This binary was linked for a different slot."
                 % (entry, args.slot, app_base, app_base + len(payload) - 1))

    if not 0x20000000 <= msp <= 0x20008000:
        print("warning: initial MSP 0x%08X is not in this part's SRAM" % msp,
              file=sys.stderr)

    crc = zlib.crc32(bytes(payload)) & 0xFFFFFFFF

    header = struct.pack(HDR_STRUCT, HDR_MAGIC, HDR_VERSION, 0,
                         len(payload), entry, args.version, crc,
                         RESERVED_FILL)
    header += bytes([ERASED]) * (HDR_RESERVED - HDR_SIZE)

    with open(args.output, "wb") as f:
        f.write(header)
        f.write(payload)

    print("packed  %s -> %s" % (args.binary, args.output))
    print("  slot        %s   (header 0x%08X, app 0x%08X)"
          % (args.slot, SLOT_BASE[args.slot], app_base))
    print("  payload     %d bytes (%d raw + %d pad), %.1f%% of slot"
          % (len(payload), raw_len, len(payload) - raw_len,
             100.0 * len(payload) / APP_MAX))
    print("  entry       0x%08X" % entry)
    print("  initial MSP 0x%08X" % msp)
    print("  version     %s" % fmt_version(args.version))
    print("  crc32       0x%08X" % crc)
    print("  total       %d bytes to write" % (HDR_RESERVED + len(payload)))


def info(args):
    data = open(args.image, "rb").read()
    if len(data) < HDR_RESERVED:
        sys.exit("error: file is shorter than the header region")

    (magic, hdr_ver, flags, img_len,
     entry, img_ver, crc, reserved) = struct.unpack(HDR_STRUCT, data[:HDR_SIZE])

    print("  magic       0x%08X %s" % (magic, "ok" if magic == HDR_MAGIC else "BAD"))
    print("  hdr_version %d" % hdr_ver)
    print("  flags       0x%04X" % flags)
    print("  img_len     %d" % img_len)
    print("  entry       0x%08X" % entry)
    print("  version     %s" % fmt_version(img_ver))
    print("  crc32       0x%08X" % crc)
    if reserved != RESERVED_FILL:
        print("  reserved    %s  (expected %s)"
              % (reserved.hex(), RESERVED_FILL.hex()))

    payload = data[HDR_RESERVED:HDR_RESERVED + img_len]
    if len(payload) < img_len:
        print("  payload     TRUNCATED - %d of %d bytes" % (len(payload), img_len))
        return
    actual = zlib.crc32(payload) & 0xFFFFFFFF
    print("  recomputed  0x%08X %s"
          % (actual, "ok" if actual == crc else "MISMATCH"))


def main():
    ap = argparse.ArgumentParser(description="G4Boot image packer")
    ap.add_argument("--info", metavar="IMAGE", dest="image",
                    help="dump and verify an existing .img instead of packing")
    ap.add_argument("binary", nargs="?", help="raw .bin from objcopy")
    ap.add_argument("--slot", choices=("A", "B"),
                    help="slot this binary was linked for")
    ap.add_argument("--version", type=parse_version, default=parse_version("0.1.0"),
                    help="image version, major.minor.patch (default 0.1.0)")
    ap.add_argument("-o", "--output", help="output .img path")
    args = ap.parse_args()

    if args.image:
        info(args)
        return
    if not (args.binary and args.slot and args.output):
        ap.error("need BINARY, --slot and -o (or use --info)")
    pack(args)


if __name__ == "__main__":
    main()
