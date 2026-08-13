#!/usr/bin/env python3
"""
check_map.py - fail the build if the memory map is declared inconsistently.

The same addresses are stated independently in three places:

    shared/image_header.h   what the firmware believes
    bl/bl.ld, app/app_*.ld  where the linker actually puts code
    tools/pack.py           what the host packer validates against

Nothing forces them to agree. Change one and forget another and everything
still compiles, links and packs -- then fails on the chip, where the only
symptom is a bootloader refusing to jump or a hard fault. This script is the
missing cross-check. Run from the repo root; `make` runs it before building.
"""

import functools
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))

PAGE = 2048
errors = []
checks = 0


def check(label, a, b, fmt="0x%08X"):
    """Assert two independently-declared values agree."""
    global checks
    checks += 1
    if a != b:
        errors.append("%-38s %s != %s" % (label, fmt % a, fmt % b))


# ---------------------------------------------------------------- the header
_hdr_src = open(os.path.join(ROOT, "shared/image_header.h")).read()
_defines = {}
for _name, _body in re.findall(r"^#define\s+(G4B_[A-Z0-9_]+)\s+(.+)$", _hdr_src, re.M):
    _body = re.sub(r"/\*.*?\*/", "", _body).strip()
    if _body:
        _defines[_name] = _body


@functools.lru_cache(None)
def hdr(name):
    """Evaluate a G4B_* macro from image_header.h."""
    expr = _defines[name]
    expr = re.sub(r"\b(0[xX][0-9a-fA-F]+|[0-9]+)[uU]\b", r"\1", expr)
    for ref in set(re.findall(r"\bG4B_[A-Z0-9_]+\b", expr)):
        expr = re.sub(r"\b%s\b" % ref, str(hdr(ref)), expr)
    return eval(expr, {}, {})


# ---------------------------------------------------------- the linker scripts
def memory_regions(path):
    """Parse a MEMORY{} block into {name: (origin, length)}."""
    src = open(os.path.join(ROOT, path)).read()
    block = re.search(r"MEMORY\s*\{(.*?)\}", src, re.S)
    if not block:
        sys.exit("error: %s has no MEMORY block" % path)
    out = {}
    for name, origin, length in re.findall(
        r"(\w+)\s*\([rwx!ai]+\)\s*:\s*ORIGIN\s*=\s*([^,]+),\s*LENGTH\s*=\s*(\S+)",
        block.group(1),
    ):
        out[name] = (_num(origin), _num(length))
    return out


def _num(tok):
    tok = tok.strip().rstrip(",")
    mult = 1
    if tok[-1] in "kK":
        mult, tok = 1024, tok[:-1]
    elif tok[-1] in "mM":
        mult, tok = 1024 * 1024, tok[:-1]
    return int(tok, 0) * mult


# ------------------------------------------------------------------ the packer
import pack  # noqa: E402


# ---------------------------------------------------------------------- checks
def main():
    # header <-> packer
    check("magic  header/pack", hdr("G4B_HDR_MAGIC"), pack.HDR_MAGIC)
    check("hdr_version  header/pack", hdr("G4B_HDR_VERSION"), pack.HDR_VERSION, "%d")
    check("hdr_reserved  header/pack", hdr("G4B_HDR_RESERVED"), pack.HDR_RESERVED, "%d")
    check("slot_size  header/pack", hdr("G4B_SLOT_SIZE"), pack.SLOT_SIZE, "%d")
    check("app_max  header/pack", hdr("G4B_APP_MAX_SIZE"), pack.APP_MAX, "%d")
    check("slot A base  header/pack", hdr("G4B_SLOT_A_BASE"), pack.SLOT_BASE["A"])
    check("slot B base  header/pack", hdr("G4B_SLOT_B_BASE"), pack.SLOT_BASE["B"])
    check("struct size  C/pack", 32, pack.HDR_SIZE, "%d")

    # header <-> linker scripts
    for path, origin_macro, length_macro in (
        ("bl/bl.ld",     "G4B_BL_BASE",         "G4B_BL_SIZE"),
        ("app/app_a.ld", "G4B_SLOT_A_APP_BASE", "G4B_APP_MAX_SIZE"),
        ("app/app_b.ld", "G4B_SLOT_B_APP_BASE", "G4B_APP_MAX_SIZE"),
    ):
        mem = memory_regions(path)
        if "FLASH" not in mem:
            errors.append("%-38s no FLASH region" % path)
            continue
        origin, length = mem["FLASH"]
        check("%s FLASH origin" % path, hdr(origin_macro), origin)
        check("%s FLASH length" % path, hdr(length_macro), length, "%d")

    # .noinit must land at one address in every image -- the app writes the
    # boot-request word there and the bootloader reads it back after a reset.
    noinit = {}
    for path in ("bl/bl.ld", "app/app_a.ld", "app/app_b.ld"):
        mem = memory_regions(path)
        if "NOINIT" not in mem:
            errors.append("%-38s no NOINIT region" % path)
            continue
        noinit[path] = mem["NOINIT"]
        ram_o, ram_l = mem["RAM"]
        check("%s RAM+NOINIT contiguous" % path, ram_o + ram_l, mem["NOINIT"][0])
    if len(set(noinit.values())) > 1:
        errors.append("NOINIT differs across images: %s" %
                      {k: "0x%08X" % v[0] for k, v in noinit.items()})
    else:
        global checks
        checks += 1

    # internal geometry
    check("slot A end == slot B base",
          hdr("G4B_SLOT_A_BASE") + hdr("G4B_SLOT_SIZE"), hdr("G4B_SLOT_B_BASE"))
    check("bl end == state page 0",
          hdr("G4B_BL_BASE") + hdr("G4B_BL_SIZE"), hdr("G4B_STATE0_BASE"))
    check("state page 0 + 1 page == state page 1",
          hdr("G4B_STATE0_BASE") + PAGE, hdr("G4B_STATE1_BASE"))
    check("state page 1 + 1 page == slot A",
          hdr("G4B_STATE1_BASE") + PAGE, hdr("G4B_SLOT_A_BASE"))
    check("app_max == slot - header",
          hdr("G4B_SLOT_SIZE") - hdr("G4B_HDR_RESERVED"), hdr("G4B_APP_MAX_SIZE"), "%d")

    # erase granularity: every region boundary must sit on a 2 KB page
    for name in ("G4B_BL_BASE", "G4B_STATE0_BASE", "G4B_STATE1_BASE",
                 "G4B_SLOT_A_BASE", "G4B_SLOT_B_BASE"):
        checks += 1
        if hdr(name) % PAGE:
            errors.append("%-38s 0x%08X not on a 2 KB page boundary"
                          % (name, hdr(name)))

    # VTOR alignment: the vector table must be aligned to a power of two >= its
    # size. 512 covers this part's 118-entry table.
    for name in ("G4B_SLOT_A_APP_BASE", "G4B_SLOT_B_APP_BASE"):
        checks += 1
        if hdr(name) % 512:
            errors.append("%-38s 0x%08X not 512-aligned -- VTOR would misbehave"
                          % (name, hdr(name)))

    if errors:
        print("memory map INCONSISTENT (%d checks):" % checks, file=sys.stderr)
        for e in errors:
            print("  " + e, file=sys.stderr)
        print("\nimage_header.h, the .ld files and pack.py must agree.",
              file=sys.stderr)
        return 1

    print("memory map consistent (%d checks)" % checks)
    return 0


if __name__ == "__main__":
    sys.exit(main())
