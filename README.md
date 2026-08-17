# G4Boot

A/B firmware-update bootloader for STM32G431KB (NUCLEO-G431KB), 16 KB.
Firmware arrives over UART into the inactive slot, is verified from flash,
booted under a 5 s watchdog, and must confirm itself; otherwise the previous
firmware is restored automatically in ~25 s.

All numbers measured on hardware. All failure paths exercised, including a
deliberately defective image left to roll back unattended.

```
host                         device
----                         ------
BEGIN (32 B header)   ->     validate everything, erase inactive slot    ACK
CHUNK xN (<=248 B)    ->     program doublewords at slot + seq*248       ACK each
END                   ->     re-read flash, run boot-time validation     ACK
BOOT                  ->     record pending=target, reset
                             try+1, arm IWDG, jump      (3 attempts max)
                             app confirms itself, or rollback
```

Device listens 3 s after reset; any valid frame in that window enters update
mode.

## Memory map

128 KB single-bank flash, 2 KB pages:

| region                      | address                 | size      |
|-----------------------------|-------------------------|-----------|
| bootloader                  | `0x08000000`            | 16 KB     |
| boot state, 2 pages (erase unit) | `0x08004000/0x08004800` | 2 KB each |
| slot A (512 B header + app) | `0x08005000`            | 52 KB     |
| slot B (same layout)        | `0x08012000`            | 52 KB     |

- app linked twice (`app_a.ld`, `app_b.ld`): addresses fixed at link time, a
  slot is a link target, not a copy destination
- map declared independently in `image_header.h`, 3 linker scripts, `pack.py`;
  toolchain enforces nothing, so `tools/check_map.py` cross-checks all (30
  checks) before every build

## Boot state record

16 bytes: `active`, `pending`, `try_count`, `confirmed`, seq, CRC. Two flash
doublewords, ping-ponged across two pages: write to stale page with seq+1, so
power loss at any instant leaves the previous record valid.

Three rules:

1. **`active` changes only when a running app confirms itself.** Transfer
   proves bytes arrived; only a running image proves it runs. App derives its
   slot from `SCB->VTOR`, confirms only if `pending == me`; also blocks
   confirmation by an image executing from the wrong slot.
2. **Attempt counted before the jump.** After the jump no bootloader code
   exists; a hung app reports nothing. Counting first makes silence itself
   consume an attempt. Counting write fails -> no jump.
3. **Bootloader arms the watchdog, not the app.** An app-started watchdog sits
   behind the exact hangs it should catch. Armed once, immediately before
   jumping to a pending image: confirmed boots run watchdog-free, bootloader
   never runs under it.

Rollback = one 16-byte state write. Previous image never moved.

## Validation

Destructive operations preceded by all applicable checks; each exercised on
hardware:

- BEGIN validates header (magic, version, bounded length, entry inside target
  slot) before the 570 ms erase. Wrong-slot image refused with slot intact:
  CRC cannot detect bytes at the wrong address, and a jump through the wrong
  vector table lands in code that executes
- retransmitted CHUNK (lost ACK) re-ACKed without touching flash: programming
  a doubleword twice corrupts ECC, NMI on every later read. Retry idempotent
- END re-reads the memory-mapped slot, runs the same validation as boot:
  passing END implies passing boot by construction. Frame CRCs verify the
  wire, END verifies the flash
- interrupted transfer: partial slot fails CRC, cannot boot, next BEGIN
  erases it

## Rollback demonstration

`make app_b_bad`: passes every static check (correct slot, valid CRC,
well-formed header), hangs at runtime. The failure class only a watchdog
detects. Delivered over UART, then unattended:

```
trying pending B, attempt 1 of 3
G4Boot app 0.1.0  vtor 0x08012200
simulating a hang: no kick, no confirm
                                        (5 s of silence)
G4Boot bl 0.1.0
reset cause: iwdg
trying pending B, attempt 2 of 3
...
trying pending B, attempt 3 of 3
...
pending B: 3 tries, never confirmed -- rolling back
state says active=A
booting slot A
```

~25 s from defective image to restored firmware, no intervention. State pages
read back as a log: activation request, three counted attempts, pending
cleared.

## Measurements

170 MHz, `-Og`, 115200 baud:

| quantity                         | value                    |
|----------------------------------|--------------------------|
| erase, one page                  | ~22 ms                   |
| erase, whole slot (26 pages)     | ~570 ms                  |
| transfer + program, 11.6 KB image | 1.23 s (9.2 KB/s)       |
| END verification (hardware CRC)  | < 10 ms                  |
| update, BEGIN to new app banner  | ~6 s (~8 s to confirmed) |
| rollback, hang to previous app   | ~25 s, unattended        |
| bootloader size                  | 12236 B of 16384 (74.7%) |

Fitting 16 KB required: `vsnprintf` -> minimal formatter (saved 2.3 KB), HAL
UART -> direct register access (saved 4.1 KB; `HAL_UART_Init` anchors helpers
`--gc-sections` cannot discard).

## Build and usage

Requires `arm-none-eabi-gcc` (tested 15.3.rel1), `st-flash`, Python 3 +
`pyserial` in a venv.

```
make images                 # bootloader, both app variants, packed .img files
st-flash --reset write bl/build/bl.bin 0x08000000
st-flash --reset write app_a.img 0x08005000        # initial firmware, by wire

.venv/bin/python tools/update.py PORT app_b.img --boot     # all later updates, by UART
```

Also in `tools/`: `hello.py` (framing test, `--corrupt` proves CRC rejection),
`begin.py` (erase timing), `pack.py --info`, `g4b_frame.py` (single shared
wire-format implementation).

## Repository layout

```
bl/        bootloader (CubeMX project; most HAL removed)
app/       one app source, linked twice via app_a.ld / app_b.ld
shared/    image_header.h, g4b_proto.h, g4b_state.[ch], compiled into all three targets
tools/     pack.py, check_map.py, update.py, g4b_frame.py, ...
```

## Limitations

- CRC-32 is integrity, not authenticity: no image signing, any host with UART
  access can update the board
- UART only; framing is transport-agnostic, FDCAN planned second
- bench project: numbers describe this board, not a product
