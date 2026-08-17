#ifndef G4B_STATE_H
#define G4B_STATE_H

#include <stdbool.h>
#include <stdint.h>
#include "image_header.h"

/* Shared by bl and app: the boot state record lives in two ping-pong pages
   and both sides must agree on every byte of how it is read and written. */
void     g4b_crc_init(void);
uint32_t g4b_crc32(const void *data, uint32_t len);
bool     g4b_state_read(uint32_t page_base, boot_state_t *out);
bool     g4b_state_write(uint32_t page_base, uint8_t active, uint8_t pending,
                         uint8_t try_count, uint8_t confirmed, uint32_t seq);
bool     g4b_state_load(boot_state_t *out, uint32_t *stale_page);

#endif