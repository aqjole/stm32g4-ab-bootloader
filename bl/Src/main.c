/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "image_header.h"
#include "g4b_proto.h"
#include "g4b_log.h"
#include <stdarg.h>
#include <stdbool.h> 
#include <stddef.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Survives a warm reset -- app writes G4B_BOOT_MAGIC_STAY here and resets to
   ask the bootloader to stay put. Same address in bl, app_a and app_b. */
__attribute__((section(".noinit"))) uint32_t g4b_boot_request;
CRC_HandleTypeDef hcrc;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void g4b_tx_bytes(const uint8_t *p, uint32_t len)
{
  for (uint32_t i = 0u; i < len; i++) {
    while ((USART2->ISR & USART_ISR_TXE) == 0u) { }
    USART2->TDR = p[i];
  }
  /* Wait for the last byte to fully leave the shift register. Without this,
     an ACK followed by NVIC_SystemReset() gets cut off mid-byte. */
  while ((USART2->ISR & USART_ISR_TC) == 0u) { }
}

void g4b_printf(const char *fmt, ...)
{
  char buf[160];
  char *p = buf;
  char *const end = buf + sizeof buf;
  va_list ap;
  va_start(ap, fmt);

  for (const char *f = fmt; *f != '\0' && p < end; f++) {
    if (*f != '%') { *p++ = *f; continue; }

    f++;                                        /* past '%' */
    unsigned pad = 0u;                          /* "08" -> 8; only zero-pad */
    while (*f >= '0' && *f <= '9') { pad = pad * 10u + (unsigned)(*f - '0'); f++; }
    while (*f == 'l' || *f == 'h') { f++; }     /* accept, ignore: all 32-bit */

    char tmp[10];
    unsigned n = 0u;
    unsigned long v;

    switch (*f) {
    case 's': {
      const char *s = va_arg(ap, const char *);
      while (*s != '\0' && p < end) { *p++ = *s++; }
      break;
    }
    case 'x':
    case 'X': {
      const char *digits = (*f == 'X') ? "0123456789ABCDEF" : "0123456789abcdef";
      v = va_arg(ap, unsigned long);
      do { tmp[n++] = digits[v & 0xFuL]; v >>= 4; } while (v != 0uL && n < sizeof tmp);
      while (n < pad && n < sizeof tmp) { tmp[n++] = '0'; }
      while (n > 0u && p < end) { *p++ = tmp[--n]; }
      break;
    }
    case 'u': {
      v = va_arg(ap, unsigned long);
      do { tmp[n++] = (char)('0' + (v % 10uL)); v /= 10uL; } while (v != 0uL && n < sizeof tmp);
      while (n < pad && n < sizeof tmp) { tmp[n++] = '0'; }
      while (n > 0u && p < end) { *p++ = tmp[--n]; }
      break;
    }
    case '%':   *p++ = '%'; break;
    case '\0':  f--;        break;              /* trailing '%': let the loop end */
    default:    *p++ = *f;  break;              /* unknown: emit it literally */
    }
  }

  va_end(ap);
  g4b_tx_bytes((const uint8_t *)buf, (uint32_t)(p - buf));
}

static void g4b_crc_init(void)
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
static uint32_t g4b_crc32(const void *data, uint32_t len)
{
  return HAL_CRC_Calculate(&hcrc, (const uint32_t *)data, len) ^ 0xFFFFFFFFu;
}

static uint8_t g4b_tx[G4B_MAX_PAYLOAD + G4B_FRAME_OVERHEAD];

static void g4b_frame_send(uint8_t type, const void *payload, uint16_t len)
{
  g4b_tx[0] = G4B_SOF;
  g4b_tx[1] = (uint8_t)(len & 0xFFu);
  g4b_tx[2] = (uint8_t)(len >> 8);
  g4b_tx[3] = type;

  if (len > 0u && payload != NULL) {
    memcpy(&g4b_tx[4], payload, len);
  }

  /* CRC over len + type + payload, i.e. everything after the SOF */
  uint32_t crc = g4b_crc32(&g4b_tx[1], 3u + len);
  g4b_tx[4u + len + 0u] = (uint8_t)(crc);
  g4b_tx[4u + len + 1u] = (uint8_t)(crc >> 8);
  g4b_tx[4u + len + 2u] = (uint8_t)(crc >> 16);
  g4b_tx[4u + len + 3u] = (uint8_t)(crc >> 24);

  g4b_tx_bytes(g4b_tx, 8u + len);
}

static uint8_t  g4b_rx[3u + G4B_MAX_PAYLOAD];
static uint16_t g4b_rx_len;
static uint8_t  g4b_rx_type;
#define G4B_RX_PAYLOAD (&g4b_rx[3])

typedef enum { G4B_RX_OK, G4B_RX_TIMEOUT, G4B_RX_BAD } g4b_rx_result_t;

static bool g4b_rx_byte(uint8_t *b, uint32_t ms)
{
  uint32_t start = HAL_GetTick();
  for (;;) {
    if (USART2->ISR & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE)) {
      USART2->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF;
    }
    if (USART2->ISR & USART_ISR_RXNE) {
      *b = (uint8_t)USART2->RDR;          /* reading RDR clears RXNE */
      return true;
    }
    if ((HAL_GetTick() - start) >= ms) { return false; }
  }
}

static uint32_t g4b_rx_dropped;   /* frames discarded during the last recv */

static g4b_rx_result_t g4b_frame_recv(uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();
  g4b_rx_dropped = 0u;

  while ((HAL_GetTick() - start) < timeout_ms) {
    uint8_t b;

    /* 1. Hunt for SOF. */
    if (!g4b_rx_byte(&b, 10u) || b != G4B_SOF) { continue; }

    /* 2. Header. */
    bool got = true;
    for (uint32_t i = 0u; i < 3u && got; i++) {
      got = g4b_rx_byte(&g4b_rx[i], 50u);
    }
    if (!got) { g4b_rx_dropped++; continue; }

    uint16_t len = (uint16_t)g4b_rx[0] | ((uint16_t)g4b_rx[1] << 8);

    if (len > G4B_MAX_PAYLOAD) { g4b_rx_dropped++; continue; }

    /* 3. Payload. */
    for (uint16_t i = 0u; i < len && got; i++) {
      got = g4b_rx_byte(&g4b_rx[3u + i], 50u);
    }
    if (!got) { g4b_rx_dropped++; continue; }

    /* 4. CRC. */
    uint8_t c[4];
    for (uint32_t i = 0u; i < 4u && got; i++) {
      got = g4b_rx_byte(&c[i], 50u);
    }
    if (!got) { g4b_rx_dropped++; continue; }

    uint32_t want = (uint32_t)c[0] | ((uint32_t)c[1] << 8)
                  | ((uint32_t)c[2] << 16) | ((uint32_t)c[3] << 24);

    if (g4b_crc32(g4b_rx, 3u + len) != want) { g4b_rx_dropped++; continue; }

    g4b_rx_len  = len;
    g4b_rx_type = g4b_rx[2];
    return G4B_RX_OK;
  }

  return (g4b_rx_dropped > 0u) ? G4B_RX_BAD : G4B_RX_TIMEOUT;
}

/* Read and validate the record in `page_base`. */
static bool g4b_state_read(uint32_t page_base, boot_state_t *out)
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
static bool g4b_state_write(uint32_t page_base, uint8_t active,
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

static bool g4b_state_load(boot_state_t *out, uint32_t *stale_page)
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

static image_header_t g4b_up_hdr;    /* header the image must match at END  */
static uint32_t       g4b_up_slot;   /* slot base being written             */
static uint32_t       g4b_up_offset; /* payload bytes programmed so far     */
static bool           g4b_up_open;   /* BEGIN accepted, chunks welcome      */
static uint16_t       g4b_up_seq;    /* next chunk we expect               */

static bool g4b_slot_erase(uint32_t slot_base)
{
  if (slot_base != G4B_SLOT_A_BASE && slot_base != G4B_SLOT_B_BASE) {
    g4b_printf("refusing to erase 0x%08lX\r\n", (unsigned long)slot_base);
    return false;
  }

  uint32_t first = (slot_base - FLASH_BASE) / FLASH_PAGE_SIZE;   /* 10 or 36 */
  uint32_t pages = G4B_SLOT_SIZE / FLASH_PAGE_SIZE;              /* 26 */

  g4b_printf("erasing %lu pages from %lu\r\n",
             (unsigned long)pages, (unsigned long)first);

  uint32_t t0 = HAL_GetTick();
  HAL_FLASH_Unlock();

  FLASH_EraseInitTypeDef e = {
    .TypeErase = FLASH_TYPEERASE_PAGES,
    .Banks     = FLASH_BANK_1,
    .Page      = first,
    .NbPages   = pages
  };
  uint32_t page_error = 0u;
  bool ok = (HAL_FLASHEx_Erase(&e, &page_error) == HAL_OK);

  if (!ok) {
    g4b_printf("erase failed at page %lu err 0x%08lX\r\n",
               (unsigned long)page_error, (unsigned long)HAL_FLASH_GetError());
  }
  HAL_FLASH_Lock();

  g4b_printf("erase %s, %lu ms\r\n", ok ? "ok" : "FAILED",
             (unsigned long)(HAL_GetTick() - t0));
  return ok;
}

/* Program `len` bytes at `addr`, one doubleword at a time. `len` must be a
   multiple of 8. */
static bool g4b_flash_write(uint32_t addr, const uint8_t *data, uint32_t len)
{
  bool ok = true;

  HAL_FLASH_Unlock();
  for (uint32_t i = 0u; i < len && ok; i += 8u) {
    uint64_t dw;
    memcpy(&dw, &data[i], 8u);   /* payload sits at g4b_rx[5]: unaligned */
    ok = (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + i, dw) == HAL_OK);
  }
  HAL_FLASH_Lock();

  if (!ok) {
    g4b_printf("prog failed @0x%08lX err 0x%08lX\r\n",
               (unsigned long)addr, (unsigned long)HAL_FLASH_GetError());
  }
  return ok;
}

static void g4b_handle_begin(const boot_state_t *st)
{
  uint8_t why;

  if (g4b_rx_len != sizeof(image_header_t)) {
    why = G4B_NACK_BAD_LEN;
    g4b_frame_send(G4B_MSG_NACK, &why, 1u);
    return;
  }
  memcpy(&g4b_up_hdr, G4B_RX_PAYLOAD, sizeof g4b_up_hdr);

  if (g4b_up_hdr.magic != G4B_HDR_MAGIC ||
      g4b_up_hdr.hdr_version != G4B_HDR_VERSION) {
    why = G4B_NACK_BAD_TYPE;
    g4b_frame_send(G4B_MSG_NACK, &why, 1u);
    return;
  }

  if (g4b_up_hdr.img_len == 0u ||
      g4b_up_hdr.img_len > G4B_APP_MAX_SIZE ||
      (g4b_up_hdr.img_len % 8u) != 0u) {
    g4b_printf("begin: bad length %lu\r\n", (unsigned long)g4b_up_hdr.img_len);
    why = G4B_NACK_BAD_LEN;
    g4b_frame_send(G4B_MSG_NACK, &why, 1u);
    return;
  }

  g4b_up_slot = (st->active == G4B_SLOT_B) ? G4B_SLOT_A_BASE : G4B_SLOT_B_BASE;

  g4b_printf("begin: %lu B v%lu.%lu.%lu crc 0x%08lX -> slot %s\r\n",
             (unsigned long)g4b_up_hdr.img_len,
             (unsigned long)G4B_VERSION_MAJOR(g4b_up_hdr.img_version),
             (unsigned long)G4B_VERSION_MINOR(g4b_up_hdr.img_version),
             (unsigned long)G4B_VERSION_PATCH(g4b_up_hdr.img_version),
             (unsigned long)g4b_up_hdr.crc32,
             (g4b_up_slot == G4B_SLOT_B_BASE) ? "B" : "A");

  if (!g4b_slot_erase(g4b_up_slot)) {
    why = G4B_NACK_FLASH;
    g4b_frame_send(G4B_MSG_NACK, &why, 1u);
    return;
  }

  g4b_up_offset = 0u;
  g4b_up_seq    = 0u;
  g4b_up_open   = true;
  g4b_frame_send(G4B_MSG_ACK, NULL, 0u);
}

static void g4b_handle_chunk(void)
{
  uint8_t why;

  if (!g4b_up_open) {
    why = G4B_NACK_NOT_READY;                 /* no BEGIN, nowhere to write  */
    g4b_frame_send(G4B_MSG_NACK, &why, 1u);
    return;
  }

  /* seq(2) + at least one doubleword; data a multiple of 8, at most 248 */
  uint32_t data_len = (g4b_rx_len >= 2u) ? (g4b_rx_len - 2u) : 0u;
  if (data_len < 8u || data_len > G4B_CHUNK_DATA || (data_len % 8u) != 0u) {
    why = G4B_NACK_BAD_LEN;
    g4b_frame_send(G4B_MSG_NACK, &why, 1u);
    return;
  }

  uint16_t seq = (uint16_t)G4B_RX_PAYLOAD[0]
               | (uint16_t)((uint16_t)G4B_RX_PAYLOAD[1] << 8);

  /* Flash must not be touched twice (ECC); it only needs its receipt. */
  if (g4b_up_seq > 0u && seq == g4b_up_seq - 1u) {
    g4b_frame_send(G4B_MSG_ACK, NULL, 0u);
    return;
  }

  uint32_t offset = (uint32_t)seq * G4B_CHUNK_DATA;

  if (seq != g4b_up_seq ||
      offset + data_len > G4B_HDR_RESERVED + g4b_up_hdr.img_len) {
    g4b_printf("chunk: seq %lu want %lu\r\n",
               (unsigned long)seq, (unsigned long)g4b_up_seq);
    g4b_up_open = false;
    why = G4B_NACK_BAD_SEQ;
    g4b_frame_send(G4B_MSG_NACK, &why, 1u);
    return;
  }

  if (!g4b_flash_write(g4b_up_slot + offset, &G4B_RX_PAYLOAD[2], data_len)) {
    g4b_up_open = false;
    why = G4B_NACK_FLASH;
    g4b_frame_send(G4B_MSG_NACK, &why, 1u);
    return;
  }

  g4b_up_seq++;
  g4b_up_offset = offset + data_len;
  g4b_frame_send(G4B_MSG_ACK, NULL, 0u);
}

static bool g4b_slot_valid(uint32_t slot_base)
{
  /* Flash is memory-mapped: point a struct at the address and read it. */
  const image_header_t *h = (const image_header_t *)slot_base;

  g4b_printf("slot @0x%08lX: ", (unsigned long)slot_base);

  if (h->magic != G4B_HDR_MAGIC) {
    g4b_printf("no image (magic 0x%08lX)\r\n", (unsigned long)h->magic);
    return false;
  }

  if (h->hdr_version != G4B_HDR_VERSION) {
    g4b_printf("header v%u, I speak v%u\r\n",
               (unsigned)h->hdr_version, (unsigned)G4B_HDR_VERSION);
    return false;
  }

  if (h->img_len == 0u || h->img_len > G4B_APP_MAX_SIZE || (h->img_len % 8u) != 0u) {
    g4b_printf("bad length %lu\r\n", (unsigned long)h->img_len);
    return false;
  }

  const void *payload = (const void *)(slot_base + G4B_HDR_RESERVED);
  uint32_t actual = g4b_crc32(payload, h->img_len);

  if (actual != h->crc32) {
    g4b_printf("CRC mismatch: header 0x%08lX, flash 0x%08lX\r\n",
               (unsigned long)h->crc32, (unsigned long)actual);
    return false;
  }

  g4b_printf("valid, %lu B, v%lu.%lu.%lu, crc 0x%08lX\r\n",
             (unsigned long)h->img_len,
             (unsigned long)G4B_VERSION_MAJOR(h->img_version),
             (unsigned long)G4B_VERSION_MINOR(h->img_version),
             (unsigned long)G4B_VERSION_PATCH(h->img_version),
             (unsigned long)actual);
  return true;
}

static void g4b_handle_end(void)
{
  uint8_t why;

  if (!g4b_up_open) {
    why = G4B_NACK_NOT_READY;
    g4b_frame_send(G4B_MSG_NACK, &why, 1u);
    return;
  }
  g4b_up_open = false;

  if (g4b_rx_len != 0u) {
    why = G4B_NACK_BAD_LEN;
    g4b_frame_send(G4B_MSG_NACK, &why, 1u);
    return;
  }

  uint32_t want = G4B_HDR_RESERVED + g4b_up_hdr.img_len;
  if (g4b_up_offset != want) {
    g4b_printf("end: got %lu of %lu B\r\n",
               (unsigned long)g4b_up_offset, (unsigned long)want);
    why = G4B_NACK_BAD_LEN;
    g4b_frame_send(G4B_MSG_NACK, &why, 1u);
    return;
  }

  if (memcmp((const void *)g4b_up_slot, &g4b_up_hdr, sizeof g4b_up_hdr) != 0) {
    why = G4B_NACK_BAD_CRC;
    g4b_frame_send(G4B_MSG_NACK, &why, 1u);
    return;
  }

  /* Re-read from the memory-mapped slot and run the boot-time validation. */
  if (!g4b_slot_valid(g4b_up_slot)) {
    why = G4B_NACK_BAD_CRC;
    g4b_frame_send(G4B_MSG_NACK, &why, 1u);
    return;
  }

  g4b_frame_send(G4B_MSG_ACK, NULL, 0u);
}

/* Hand control to the image in `slot_base`. Never returns. */
__attribute__((noreturn))
static void g4b_jump_to_slot(uint32_t slot_base)
{
  uint32_t app_base = slot_base + G4B_HDR_RESERVED;      /* skip the 512 B header */
  uint32_t sp = *(volatile uint32_t *)(app_base + 0u);
  uint32_t pc = *(volatile uint32_t *)(app_base + 4u);

  g4b_printf("  sp 0x%08lX  pc 0x%08lX\r\n",
             (unsigned long)sp, (unsigned long)pc);

  if (sp < 0x20000000u || sp > 0x20008000u) {
    g4b_printf("  refusing: sp not in SRAM -- slot looks empty\r\n");
    while (1) { }
  }

  g4b_printf("jumping\r\n\r\n");
  HAL_Delay(5);                       /* let the last byte leave the shift register */

  __disable_irq();

  HAL_RCC_DeInit();
  HAL_DeInit();

  SysTick->CTRL = 0u;
  SysTick->LOAD = 0u;
  SysTick->VAL  = 0u;

  /* Disable AND unpend every source. A pending USART2 interrupt left over from
     the bootloader would fire the instant interrupts come back on, vectoring
     into an app handler whose peripheral is not initialised yet. */
  for (uint32_t i = 0u; i < 8u; i++) {
    NVIC->ICER[i] = 0xFFFFFFFFu;
    NVIC->ICPR[i] = 0xFFFFFFFFu;
  }

  SCB->VTOR = app_base;
  __DSB();
  __ISB();
  __enable_irq();

  /* Set MSP and branch in one asm block: writing MSP invalidates every
     stack-resident local, so `pc` must be pinned in a register across the
     switch. __set_MSP() followed by a C call works at -Og and breaks at -O2. */
  __asm volatile ("msr msp, %0 \n bx %1" :: "r" (sp), "r" (pc));

  __builtin_unreachable();
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  g4b_printf("\r\n\r\nG4Boot bl 0.1.0  %s %s\r\n", __DATE__, __TIME__);

  g4b_crc_init();
  uint32_t chk = g4b_crc32("123456789", 9u);
  g4b_printf("CRC32 check 0x%08lX (expect 0xCBF43926)\r\n", (unsigned long)chk);
  
  boot_state_t st;
  uint32_t stale = G4B_STATE0_BASE;

  if (!g4b_state_load(&st, &stale)) {
    g4b_printf("no state -- seeding active=A\r\n");
    st.magic     = G4B_STATE_MAGIC;
    st.seq       = 1u;
    st.active    = G4B_SLOT_A;
    st.pending   = G4B_SLOT_NONE;
    st.try_count = 0u;
    st.confirmed = 1u;
    if (!g4b_state_write(stale, st.active, st.pending,
                         st.try_count, st.confirmed, st.seq)) {
      g4b_printf("seed failed -- continuing on defaults\r\n");
    }
  }

  g4b_printf("listening for a frame (3 s)\r\n");
  g4b_rx_result_t r = g4b_frame_recv(3000u);

  if (r == G4B_RX_OK) {
    g4b_printf("update mode -- not booting\r\n");

    for (;;) {
      switch (g4b_rx_type) {
      case G4B_MSG_HELLO:
        g4b_frame_send(G4B_MSG_ACK, NULL, 0u);
        break;

      case G4B_MSG_BEGIN:
        g4b_handle_begin(&st);
        break;

      case G4B_MSG_CHUNK:
        g4b_handle_chunk();
        break;

      case G4B_MSG_END:
        g4b_handle_end();
        break;

      default: {
        uint8_t why = G4B_NACK_BAD_TYPE;
        g4b_frame_send(G4B_MSG_NACK, &why, 1u);
        break;
      }
      }

      if (g4b_frame_recv(30000u) != G4B_RX_OK) {
        g4b_printf("host gone -- resetting\r\n");
        NVIC_SystemReset();
      }
    }
  }

  if (r == G4B_RX_BAD) {
    g4b_printf("bad frame -- dropped\r\n");
  } else {
    g4b_printf("no frame\r\n");
  }

  bool     want_b = (st.active == G4B_SLOT_B);
  uint32_t first  = want_b ? G4B_SLOT_B_BASE : G4B_SLOT_A_BASE;
  uint32_t second = want_b ? G4B_SLOT_A_BASE : G4B_SLOT_B_BASE;

  g4b_printf("state says active=%s\r\n", want_b ? "B" : "A");

  if (g4b_slot_valid(first)) {
    g4b_printf("booting slot %s\r\n", want_b ? "B" : "A");
    g4b_jump_to_slot(first);
  }
  else if (g4b_slot_valid(second)) {
    g4b_printf("booting slot %s (fallback)\r\n", want_b ? "A" : "B");
    g4b_jump_to_slot(second);
  }
  else {
    g4b_printf("no bootable image in either slot -- halting\r\n");
    while (1) { }
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  /* Direct register init: BRR + CR1 replace HAL_UART_Init and the ~3 KB it
     anchors (UART_SetConfig, AdvFeatureConfig, uart_ex, PeriphCLKConfig).
     Reset defaults already give 8N1, oversampling 16, no FIFO, and
     USART2SEL = PCLK1, so only the non-defaults are written. */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART2_CLK_ENABLE();

  GPIO_InitTypeDef g = {
    .Pin = USART2_TX_Pin | USART2_RX_Pin,
    .Mode = GPIO_MODE_AF_PP,
    .Pull = GPIO_NOPULL,
    .Speed = GPIO_SPEED_FREQ_LOW,
    .Alternate = GPIO_AF7_USART2,
  };
  HAL_GPIO_Init(GPIOA, &g);

  USART2->BRR = (HAL_RCC_GetPCLK1Freq() + 115200u / 2u) / 115200u;
  USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
