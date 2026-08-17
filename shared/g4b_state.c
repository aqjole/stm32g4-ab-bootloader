#include "main.h"          /* resolves per-project: HAL + Error_Handler */
#include "g4b_log.h"
#include "g4b_state.h"
#include <stddef.h>
#include <string.h>

static CRC_HandleTypeDef hcrc;

void g4b_crc_init(void)
{
  /* HAL_CRC_MspInit is __weak and nothing defines it. Without this the peripheral is
     unclocked: HAL_CRC_Init still returns HAL_OK and every result is garbage. */
  __HAL_RCC_CRC_CLK_ENABLE();

  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse    = DEFAULT_POLYNOMIAL_ENABLE;   /* 0x04C11DB7 */
  hcrc.Init.DefaultInitValueUse     = DEFAULT_INIT_VALUE_ENABLE;   /* 0xFFFFFFFF */
  hcrc.Init.InputDataInversionMode  = CRC_INPUTDATA_INVERSION_BYTE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_ENABLE;
  hcrc.InputDataFormat              = CRC_INPUTDATA_FORMAT_BYTES;

  if (HAL_CRC_Init(&hcrc) != HAL_OK) {
    Error_Handler();
  }
}

/* zlib-compatible CRC32. The hardware has no final-XOR stage, so that last
   step of the standard is done here. */
uint32_t g4b_crc32(const void *data, uint32_t len)
{
  return HAL_CRC_Calculate(&hcrc, (const uint32_t *)data, len) ^ 0xFFFFFFFFu;
}

bool g4b_state_read(uint32_t page_base, boot_state_t *out)
{
  const boot_state_t *s = (const boot_state_t *)page_base;

  g4b_printf("state @0x%08lX: ", (unsigned long)page_base);

  if (s->magic == 0xFFFFFFFFu) {
    g4b_printf("erased\r\n");
    return false;
  }

  if (s->magic != G4B_STATE_MAGIC) {
    g4b_printf("not a record (magic 0x%08lX)\r\n", (unsigned long)s->magic);
    return false;
  }

  uint32_t actual = g4b_crc32(s, offsetof(boot_state_t, crc32));
  if (actual != s->crc32) {
    g4b_printf("crc bad (stored 0x%08lX, computed 0x%08lX)\r\n",
               (unsigned long)s->crc32, (unsigned long)actual);
    return false;
  }

  g4b_printf("seq %lu active %u pending %u try %u confirmed %u\r\n",
             (unsigned long)s->seq, (unsigned)s->active,
             (unsigned)s->pending, (unsigned)s->try_count,
             (unsigned)s->confirmed);

  /* Copy out of flash into RAM so the caller holds a stable snapshot. */
  if (out != NULL) {
    memcpy(out, s, sizeof *out);
  }
  return true;
}

/* Erase `page_base` and program one boot state record into it.
   Returns false on any flash error. */
bool g4b_state_write(uint32_t page_base, uint8_t active,
                     uint8_t pending, uint8_t try_count,
                     uint8_t confirmed, uint32_t seq)
{
  /* Erasing is destructive and a miscomputed page index lands on the
     bootloader itself. Refuse anything that is not a state page. */
  if (page_base != G4B_STATE0_BASE && page_base != G4B_STATE1_BASE) {
    g4b_printf("refusing to erase 0x%08lX\r\n", (unsigned long)page_base);
    return false;
  }

  boot_state_t rec;
  rec.magic     = G4B_STATE_MAGIC;
  rec.seq       = seq;
  rec.active    = active;
  rec.pending   = pending;
  rec.try_count = try_count;
  rec.confirmed = confirmed;
  rec.crc32     = g4b_crc32(&rec, offsetof(boot_state_t, crc32));

  uint64_t words[2];
  memcpy(words, &rec, sizeof rec);

  bool ok = true;
  HAL_FLASH_Unlock();

  FLASH_EraseInitTypeDef e = {
    .TypeErase = FLASH_TYPEERASE_PAGES,
    .Banks     = FLASH_BANK_1,
    .Page      = (page_base - FLASH_BASE) / FLASH_PAGE_SIZE,  /* index, not address */
    .NbPages   = 1u
  };
  uint32_t page_error = 0u;

  if (HAL_FLASHEx_Erase(&e, &page_error) != HAL_OK) {
    g4b_printf("erase failed, page %lu, err 0x%08lX\r\n",
               (unsigned long)page_error, (unsigned long)HAL_FLASH_GetError());
    ok = false;
  }

  for (uint32_t i = 0u; ok && i < 2u; i++) {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                          page_base + i * 8u, words[i]) != HAL_OK) {
      g4b_printf("program failed @0x%08lX, err 0x%08lX\r\n",
                 (unsigned long)(page_base + i * 8u),
                 (unsigned long)HAL_FLASH_GetError());
      ok = false;
    }
  }

  HAL_FLASH_Lock();   /* single exit point: locked on every path */
  return ok;
}

bool g4b_state_load(boot_state_t *out, uint32_t *stale_page)
{
  boot_state_t s0, s1;

  bool ok0 = g4b_state_read(G4B_STATE0_BASE, &s0);
  bool ok1 = g4b_state_read(G4B_STATE1_BASE, &s1);

  if (!ok0 && !ok1) {
    *stale_page = G4B_STATE0_BASE;
    return false;
  }

  if (ok0 && !ok1) {
    *out = s0;
    *stale_page = G4B_STATE1_BASE;
    return true;
  }

  if (!ok0 && ok1) {
    *out = s1;
    *stale_page = G4B_STATE0_BASE;
    return true;
  }

  if (s0.seq >= s1.seq) {
    *out = s0;
    *stale_page = G4B_STATE1_BASE;
  } else {
    *out = s1;
    *stale_page = G4B_STATE0_BASE;
  }
  return true;
}
