/*
 * image_header.h - G4Boot shared definitions
 *
 * Included by BOTH firmware projects (bootloader and application) and mirrored
 * by tools/pack.py. If you change anything here, change pack.py in the same
 * commit or the two sides will disagree silently.
 *
 * Target: STM32G431KB - 128 KB flash, SINGLE bank, 2 KB pages, 64 pages.
 */

#ifndef IMAGE_HEADER_H
#define IMAGE_HEADER_H

#include <stdint.h>

/* ------------------------------------------------------------------ */
/* Flash map                                                          */
/* ------------------------------------------------------------------ */
/*
 *  0x0800_0000  +----------------------------+  pages 0-7    16 KB
 *               |  bootloader                |
 *  0x0800_4000  +----------------------------+  page 8        2 KB
 *               |  boot state, copy 0        |
 *  0x0800_4800  +----------------------------+  page 9        2 KB
 *               |  boot state, copy 1        |
 *  0x0800_5000  +----------------------------+  pages 10-35  52 KB
 *               |  slot A header    (512 B)  |
 *  0x0800_5200  |  slot A application        |
 *  0x0801_2000  +----------------------------+  pages 36-61  52 KB
 *               |  slot B header    (512 B)  |
 *  0x0801_2200  |  slot B application        |
 *  0x0801_F000  +----------------------------+  pages 62-63   4 KB
 *               |  spare                     |
 *  0x0802_0000  +----------------------------+
 *
 */

#define G4B_FLASH_BASE        0x08000000u
#define G4B_PAGE_SIZE         2048u
#define G4B_FLASH_SIZE        (128u * 1024u)

#define G4B_BL_BASE           0x08000000u
#define G4B_BL_SIZE           (16u * 1024u)

#define G4B_STATE0_BASE       0x08004000u   /* page 8  */
#define G4B_STATE1_BASE       0x08004800u   /* page 9  */

#define G4B_SLOT_SIZE         (52u * 1024u)
#define G4B_HDR_RESERVED      512u

#define G4B_SLOT_A_BASE       0x08005000u
#define G4B_SLOT_B_BASE       0x08012000u
#define G4B_SLOT_A_APP_BASE   (G4B_SLOT_A_BASE + G4B_HDR_RESERVED)  /* 0x08005200 */
#define G4B_SLOT_B_APP_BASE   (G4B_SLOT_B_BASE + G4B_HDR_RESERVED)  /* 0x08012200 */
#define G4B_APP_MAX_SIZE      (G4B_SLOT_SIZE - G4B_HDR_RESERVED)    /* 52736 bytes */

#define G4B_SLOT_A            0u
#define G4B_SLOT_B            1u

static inline uint32_t g4b_slot_base(uint32_t slot)
{
    return (slot == G4B_SLOT_B) ? G4B_SLOT_B_BASE : G4B_SLOT_A_BASE;
}

static inline uint32_t g4b_slot_app_base(uint32_t slot)
{
    return g4b_slot_base(slot) + G4B_HDR_RESERVED;
}

/* ------------------------------------------------------------------ */
/* Image header                                                       */
/* ------------------------------------------------------------------ */

#define G4B_HDR_MAGIC     0x54423447u   /* bytes 47 34 42 54 = "G4BT" */
#define G4B_HDR_VERSION   1u

/* flags - room to grow; nothing consumes these yet */
#define G4B_FLAG_NONE     0x0000u

typedef struct __attribute__((packed)) {
    uint32_t magic;        /* G4B_HDR_MAGIC                                    */
    uint16_t hdr_version;  /* G4B_HDR_VERSION                                  */
    uint16_t flags;
    uint32_t img_len;      /* payload bytes, always a multiple of 8            */
    uint32_t entry;        /* app reset vector, incl. Thumb bit                */
    uint32_t img_version;  /* G4B_VERSION(major, minor, patch)                 */
    uint32_t crc32;        /* zlib CRC-32 over img_len payload bytes           */
    uint32_t reserved[2];  /* 0xFFFFFFFF - keeps the struct at 32 bytes        */
} image_header_t;

_Static_assert(sizeof(image_header_t) == 32,
               "image_header_t must be 32 bytes - check packing");

#define G4B_VERSION(maj, min, pat) \
    ((((uint32_t)(maj) & 0xFFu) << 16) | \
     (((uint32_t)(min) & 0xFFu) <<  8) | \
      ((uint32_t)(pat) & 0xFFu))

#define G4B_VERSION_MAJOR(v)  (((v) >> 16) & 0xFFu)
#define G4B_VERSION_MINOR(v)  (((v) >>  8) & 0xFFu)
#define G4B_VERSION_PATCH(v)  ( (v)        & 0xFFu)

/* ------------------------------------------------------------------ */
/* Bootloader entry request                                           */
/* ------------------------------------------------------------------ */
/*
 * Lives in .noinit so the C startup code does not touch it. The application
 * writes it and resets; the bootloader reads it and MUST clear it before
 * acting on it, or the device never boots an application again.
 */
#define G4B_BOOT_MAGIC_STAY   0xB0071BADu

/* ------------------------------------------------------------------ */
/* Boot state record                                                  */
/* ------------------------------------------------------------------ */
/*
 * Lives in the two ping-pong pages. Flash must be erased before it can be
 * written and an erase leaves 0xFF, so with a single page there is a window
 * where a power cut destroys the only copy. Two pages: read both, take the
 * valid one with the highest seq; to write, erase the OTHER page and write
 * there with seq + 1. There is never an instant with zero valid records.
 */

#define G4B_STATE_MAGIC   0x54533447u   /* bytes 47 34 53 54 = "G4ST" */
#define G4B_SLOT_NONE     0xFFu         /* "no pending slot" -- also the erased value */

typedef struct __attribute__((packed)) {
    uint32_t magic;      /* G4B_STATE_MAGIC                          */
    uint32_t seq;        /* higher wins; +1 on every write           */
    uint8_t  active;     /* G4B_SLOT_A or G4B_SLOT_B                 */
    uint8_t  pending;    /* slot on trial, or G4B_SLOT_NONE          */
    uint8_t  try_count;  /* boot attempts left for pending           */
    uint8_t  confirmed;  /* 1 once the running image has reported in */
    uint32_t crc32;      /* zlib CRC-32 over the 12 bytes above      */
} boot_state_t;

_Static_assert(sizeof(boot_state_t) == 16,
               "boot_state_t must be 16 bytes - two doublewords");

#endif /* IMAGE_HEADER_H */
