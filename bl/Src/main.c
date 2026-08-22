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
#include "g4b_state.h"
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
static FDCAN_HandleTypeDef hfdcan1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_FDCAN1_Init(void);
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

/* --- CAN <-> byte-stream shim --------------------------------------------
   Parcels in, bytes out. Logs stay UART-only. */
#define G4B_CAN_ID_HOST2DEV  0x7E0u   /* tester -> ECU, the classic UDS pair */
#define G4B_CAN_ID_DEV2HOST  0x7E8u   /* response id = request id + 8        */

/* ISO-TP reassembles whole messages here; a completed message is a UDS
   request. */
static uint8_t  g4b_uds_buf[520];
static uint16_t g4b_uds_len;        /* reassembly write index            */
static bool     g4b_uds_busy;       /* re-entry guard for the dispatcher */
static void g4b_uds_handle(const uint8_t *m, uint16_t len);

static uint8_t  g4b_uds_session = 1u;   /* 1 default, 2 programming      */
static bool     g4b_uds_dl_open;
static uint8_t  g4b_uds_bsc;            /* next expected block counter   */
static uint32_t g4b_uds_slot;
static uint32_t g4b_uds_off;
static uint32_t g4b_uds_size;           /* announced total, header incl. */
static bool     g4b_uds_active;         /* any valid request seen        */
static uint32_t g4b_uds_last_ms;
static const boot_state_t *g4b_uds_boot_st;
static uint32_t g4b_uds_boot_stale;

/* ISO-TP receive state: a message in progress between FF and last CF */
static uint16_t g4b_itp_expected;   /* payload bytes still to come        */
static uint8_t  g4b_itp_seq;        /* next CF sequence number, 0..15     */
/* last Flow Control seen, for the TX side's wait */
static uint8_t  g4b_itp_fc[3];
static bool     g4b_itp_fc_fresh;

static void g4b_can_send(const uint8_t *d, uint32_t n)
{
  FDCAN_TxHeaderTypeDef h = {0};
  h.Identifier = G4B_CAN_ID_DEV2HOST;
  h.IdType = FDCAN_STANDARD_ID;
  h.TxFrameType = FDCAN_DATA_FRAME;
  h.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  h.BitRateSwitch = FDCAN_BRS_OFF;
  h.FDFormat = FDCAN_CLASSIC_CAN;
  h.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  h.DataLength = n;

  while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0u) { }
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &h, d);
}

static void g4b_uds_put(const uint8_t *d, uint32_t n)
{
  if ((uint32_t)g4b_uds_len + n > sizeof g4b_uds_buf) { return; }
  memcpy(&g4b_uds_buf[g4b_uds_len], d, n);
  g4b_uds_len += (uint16_t)n;
}

static void g4b_uds_complete(void)
{
  if (!g4b_uds_busy) {
    g4b_uds_busy = true;
    g4b_uds_handle(g4b_uds_buf, g4b_uds_len);
    g4b_uds_busy = false;
  }
}

static void g4b_itp_send_fc(uint8_t status)   /* 0 = CTS, 2 = overflow */
{
  /* BS 0 = no block limit, STmin 0 = no gap: the ring plus lockstep
     protocol above make throttling unnecessary on this side */
  uint8_t fc[3] = {(uint8_t)(0x30u | status), 0u, 0u};
  g4b_can_send(fc, 3u);
}

/* Drain the peripheral's 3-deep RX FIFO into the ring. */
static void g4b_can_pump(void)
{
  FDCAN_RxHeaderTypeDef h;
  uint8_t d[8];

  while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0u) {
    if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &h, d) != HAL_OK) {
      return;
    }
    if (h.Identifier != G4B_CAN_ID_HOST2DEV || h.DataLength == 0u) { continue; }

    uint8_t pci = d[0] >> 4;

    if (pci == 0x0u) {                  /* Single Frame: whole message     */
      uint8_t n = d[0] & 0x0Fu;
      if (n == 0u || n > h.DataLength - 1u) { continue; }
      g4b_uds_len = 0u;
      g4b_uds_put(&d[1], n);
      g4b_itp_expected = 0u;            /* SF aborts any half-done message */
      g4b_uds_complete();

    } else if (pci == 0x1u) {           /* First Frame: opens a long one   */
      uint16_t total = (uint16_t)((d[0] & 0x0Fu) << 8) | d[1];
      if (total <= 7u || h.DataLength < 8u) { continue; }
      if (total > sizeof g4b_uds_buf) {
        g4b_itp_send_fc(2u);            /* overflow: refuse BEFORE the CFs */
        continue;
      }
      g4b_uds_len = 0u;
      g4b_uds_put(&d[2], 6u);
      g4b_itp_expected = total - 6u;
      g4b_itp_seq = 1u;
      g4b_itp_send_fc(0u);              /* clear to send */

    } else if (pci == 0x2u) {           /* Consecutive Frame               */
      if (g4b_itp_expected == 0u) { continue; }
      if ((d[0] & 0x0Fu) != g4b_itp_seq) {
        g4b_itp_expected = 0u;
        continue;
      }
      g4b_itp_seq = (g4b_itp_seq + 1u) & 0x0Fu;
      uint32_t n = (g4b_itp_expected < 7u) ? g4b_itp_expected : 7u;
      if (n > (uint32_t)h.DataLength - 1u) { n = (uint32_t)h.DataLength - 1u; }
      g4b_uds_put(&d[1], n);
      g4b_itp_expected -= (uint16_t)n;
      if (g4b_itp_expected == 0u) { g4b_uds_complete(); }

    } else {                            /* Flow Control: stash for TX side */
      g4b_itp_fc[0] = d[0];
      g4b_itp_fc[1] = (h.DataLength > 1u) ? d[1] : 0u;
      g4b_itp_fc[2] = (h.DataLength > 2u) ? d[2] : 0u;
      g4b_itp_fc_fresh = true;
    }
  }
}

static void g4b_can_tx_bytes(const uint8_t *p, uint32_t len)
{
  uint8_t f[8];

  if (len <= 7u) {                      /* Single Frame */
    f[0] = (uint8_t)len;
    memcpy(&f[1], p, len);
    g4b_can_send(f, len + 1u);
  } else {                              /* FF, then FC gates the CFs */
    f[0] = (uint8_t)(0x10u | (len >> 8));
    f[1] = (uint8_t)len;
    memcpy(&f[2], p, 6u);
    g4b_can_send(f, 8u);
    p += 6u;
    len -= 6u;

    uint8_t seq = 1u;
    uint8_t bs = 0u;
    uint8_t stmin = 0u;
    uint32_t in_block = 0u;
    bool need_fc = true;

    while (len > 0u) {
      if (need_fc) {
        g4b_itp_fc_fresh = false;
        uint32_t t0 = HAL_GetTick();
        while (!g4b_itp_fc_fresh && HAL_GetTick() - t0 < 1000u) {
          g4b_can_pump();
        }
        if (!g4b_itp_fc_fresh || (g4b_itp_fc[0] & 0x0Fu) != 0u) { return; }
        bs = g4b_itp_fc[1];
        stmin = g4b_itp_fc[2];
        /* 0xF1..F9 encode 100..900 us; rounding up to 1 ms is legal */
        if (stmin > 0x7Fu) { stmin = 1u; }
        need_fc = false;
        in_block = 0u;
      }

      uint32_t n = (len < 7u) ? len : 7u;
      f[0] = (uint8_t)(0x20u | seq);
      memcpy(&f[1], p, n);
      g4b_can_send(f, n + 1u);
      seq = (seq + 1u) & 0x0Fu;
      p += n;
      len -= n;

      if (stmin > 0u && len > 0u) { HAL_Delay(stmin); }
      if (bs > 0u && ++in_block == bs && len > 0u) { need_fc = true; }
    }
  }
  /* Same issue as the UART TC wait: queued is not transmitted, and
     BOOT's ACK is followed by NVIC_SystemReset(). Bounded, unlike the
     UART wait -- an unACKed CAN frame retries forever on a deaf bus,
     and a spin on that would hang the bootloader. */
  uint32_t t0 = HAL_GetTick();
  while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) < 3u &&
         HAL_GetTick() - t0 < 10u) { }
}

/* Arm the independent watchdog: ~5 s at LSI/256. */
static void g4b_iwdg_start(void)
{
  IWDG->KR  = 0xCCCCu;                    /* start the countdown            */
  IWDG->KR  = 0x5555u;                    /* unlock PR/RLR                  */
  IWDG->PR  = 6u;                         /* LSI/256 = 125 ticks per second */
  IWDG->RLR = 624u;                       /* (624+1)/125 = 5.0 s            */
  while (IWDG->SR != 0u) { }              /* wait for the writes to land    */
  IWDG->KR  = 0xAAAAu;                    /* load the reload value          */
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

    g4b_can_pump();
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

  if (g4b_up_hdr.entry < g4b_up_slot + G4B_HDR_RESERVED ||
      g4b_up_hdr.entry >= g4b_up_slot + G4B_SLOT_SIZE ||
      (g4b_up_hdr.entry & 1u) == 0u) {
    g4b_printf("begin: entry 0x%08lX is not slot %s code -- wrong image?\r\n",
               (unsigned long)g4b_up_hdr.entry,
               (g4b_up_slot == G4B_SLOT_B_BASE) ? "B" : "A");
    why = G4B_NACK_WRONG_SLOT;
    g4b_frame_send(G4B_MSG_NACK, &why, 1u);
    return;
  }

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

  if (h->entry < slot_base + G4B_HDR_RESERVED ||
      h->entry >= slot_base + G4B_SLOT_SIZE ||
      (h->entry & 1u) == 0u) {
    g4b_printf("entry 0x%08lX not in this slot\r\n", (unsigned long)h->entry);
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

static void g4b_handle_boot(const boot_state_t *st, uint32_t stale)
{
  uint8_t why;
  uint8_t target = (st->active == G4B_SLOT_B) ? G4B_SLOT_A : G4B_SLOT_B;

  if (g4b_rx_len != 0u) {
    why = G4B_NACK_BAD_LEN;
    g4b_frame_send(G4B_MSG_NACK, &why, 1u);
    return;
  }

  if (!g4b_slot_valid(g4b_slot_base(target))) {
    why = G4B_NACK_BAD_CRC;
    g4b_frame_send(G4B_MSG_NACK, &why, 1u);
    return;
  }

  if (!g4b_state_write(stale, st->active, target, 0u,
                       st->confirmed, st->seq + 1u)) {
    why = G4B_NACK_FLASH;
    g4b_frame_send(G4B_MSG_NACK, &why, 1u);
    return;
  }

  g4b_printf("pending=%s -- resetting\r\n", (target == G4B_SLOT_B) ? "B" : "A");
  g4b_frame_send(G4B_MSG_ACK, NULL, 0u);   /* TC-wait guarantees it escapes */
  NVIC_SystemReset();
}

/* --- UDS server (ISO 14229), the six services of a programming session --- */
static void g4b_uds_nrc(uint8_t sid, uint8_t code)
{
  uint8_t r[3] = {0x7Fu, sid, code};
  g4b_can_tx_bytes(r, 3u);
}

static void g4b_uds_handle(const uint8_t *m, uint16_t len)
{
  if (len == 0u) { return; }
  g4b_uds_active  = true;
  g4b_uds_last_ms = HAL_GetTick();

  const boot_state_t *st = g4b_uds_boot_st;
  uint8_t sid = m[0];

  switch (sid) {
  case 0x10u: {                             /* DiagnosticSessionControl */
    if (len != 2u) { g4b_uds_nrc(sid, 0x13u); return; }
    if (m[1] != 1u && m[1] != 2u) { g4b_uds_nrc(sid, 0x12u); return; }
    g4b_uds_session = m[1];
    g4b_uds_dl_open = false;
    /* sessionParameterRecord: P2 = 50 ms, P2* = 5000 ms (in 10 ms units) */
    uint8_t r[6] = {0x50u, m[1], 0x00u, 0x32u, 0x01u, 0xF4u};
    g4b_can_tx_bytes(r, 6u);
    break;
  }

  case 0x3Eu: {                             /* TesterPresent */
    if (len != 2u) { g4b_uds_nrc(sid, 0x13u); return; }
    if ((m[1] & 0x7Fu) != 0u) { g4b_uds_nrc(sid, 0x12u); return; }
    if ((m[1] & 0x80u) == 0u) {             /* bit 7 = suppress response */
      uint8_t r[2] = {0x7Eu, 0x00u};
      g4b_can_tx_bytes(r, 2u);
    }
    break;
  }

  case 0x34u: {                             /* RequestDownload */
    if (g4b_uds_session != 2u) { g4b_uds_nrc(sid, 0x7Fu); return; }
    if (len != 11u) { g4b_uds_nrc(sid, 0x13u); return; }
    if (m[1] != 0x00u || m[2] != 0x44u) { g4b_uds_nrc(sid, 0x31u); return; }

    uint32_t addr = ((uint32_t)m[3] << 24) | ((uint32_t)m[4] << 16)
                  | ((uint32_t)m[5] << 8)  |  (uint32_t)m[6];
    uint32_t size = ((uint32_t)m[7] << 24) | ((uint32_t)m[8] << 16)
                  | ((uint32_t)m[9] << 8)  |  (uint32_t)m[10];

    uint32_t want = (st->active == G4B_SLOT_B) ? G4B_SLOT_A_BASE
                                               : G4B_SLOT_B_BASE;
    /* position is part of validity: only the inactive slot base is
       writable */
    if (addr != want ||
        size <= G4B_HDR_RESERVED || (size % 8u) != 0u ||
        size > G4B_HDR_RESERVED + G4B_APP_MAX_SIZE) {
      g4b_uds_nrc(sid, 0x31u);
      return;
    }

    g4b_uds_nrc(sid, 0x78u);                /* responsePending: erase ahead */
    if (!g4b_slot_erase(addr)) { g4b_uds_nrc(sid, 0x72u); return; }

    g4b_uds_slot = addr;
    g4b_uds_size = size;
    g4b_uds_off  = 0u;
    g4b_uds_bsc  = 1u;
    g4b_uds_dl_open = true;
    /* maxNumberOfBlockLength 514 = 512 data + SID + counter */
    uint8_t r[4] = {0x74u, 0x20u, 0x02u, 0x02u};
    g4b_can_tx_bytes(r, 4u);
    break;
  }

  case 0x36u: {                             /* TransferData */
    if (!g4b_uds_dl_open) { g4b_uds_nrc(sid, 0x24u); return; }
    uint32_t data = (uint32_t)len - 2u;
    if (len < 10u || (data % 8u) != 0u) { g4b_uds_nrc(sid, 0x13u); return; }

    if (m[1] == (uint8_t)(g4b_uds_bsc - 1u)) {
      uint8_t r[2] = {0x76u, m[1]};         /* duplicate: receipt only, the
                                               flash was already touched   */
      g4b_can_tx_bytes(r, 2u);
      return;
    }
    if (m[1] != g4b_uds_bsc) {
      g4b_uds_dl_open = false;
      g4b_uds_nrc(sid, 0x73u);
      return;
    }
    if (g4b_uds_off + data > g4b_uds_size) {
      g4b_uds_dl_open = false;
      g4b_uds_nrc(sid, 0x31u);
      return;
    }
    if (!g4b_flash_write(g4b_uds_slot + g4b_uds_off, &m[2], data)) {
      g4b_uds_dl_open = false;
      g4b_uds_nrc(sid, 0x72u);
      return;
    }
    g4b_uds_off += data;
    uint8_t r[2] = {0x76u, g4b_uds_bsc};
    g4b_uds_bsc++;                          /* u8 wrap 0xFF -> 0x00 is spec */
    g4b_can_tx_bytes(r, 2u);
    break;
  }

  case 0x37u: {                             /* RequestTransferExit */
    if (!g4b_uds_dl_open) { g4b_uds_nrc(sid, 0x24u); return; }
    g4b_uds_dl_open = false;
    if (len != 1u) { g4b_uds_nrc(sid, 0x13u); return; }
    if (g4b_uds_off != g4b_uds_size || !g4b_slot_valid(g4b_uds_slot)) {
      g4b_uds_nrc(sid, 0x72u);
      return;
    }
    uint8_t r[1] = {0x77u};
    g4b_can_tx_bytes(r, 1u);
    break;
  }

  case 0x11u: {                             /* ECUReset: the gamble */
    if (len != 2u) { g4b_uds_nrc(sid, 0x13u); return; }
    if (m[1] != 1u) { g4b_uds_nrc(sid, 0x12u); return; }
    uint8_t target = (st->active == G4B_SLOT_B) ? G4B_SLOT_A : G4B_SLOT_B;
    if (!g4b_slot_valid(g4b_slot_base(target)) ||
        !g4b_state_write(g4b_uds_boot_stale, st->active, target, 0u,
                         st->confirmed, st->seq + 1u)) {
      g4b_uds_nrc(sid, 0x22u);              /* conditionsNotCorrect */
      return;
    }
    g4b_printf("pending=%s -- resetting (uds)\r\n",
               (target == G4B_SLOT_B) ? "B" : "A");
    uint8_t r[2] = {0x51u, 0x01u};
    g4b_can_tx_bytes(r, 2u);                /* drain wait guarantees escape */
    NVIC_SystemReset();
    break;
  }

  default:
    g4b_uds_nrc(sid, 0x11u);                /* serviceNotSupported */
    break;
  }
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
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */
  g4b_printf("\r\n\r\nG4Boot bl 0.1.0  %s %s\r\n", __DATE__, __TIME__);
  
  uint32_t csr = RCC->CSR;
  g4b_printf("reset cause:%s%s%s%s%s\r\n",
              (csr & RCC_CSR_IWDGRSTF) ? " iwdg" : "",
              (csr & RCC_CSR_SFTRSTF)  ? " soft" : "",
              (csr & RCC_CSR_BORRSTF)  ? " bor"  : "",
              (csr & RCC_CSR_OBLRSTF)  ? " obl"  : "",
              (csr & RCC_CSR_PINRSTF)  ? " pin"  : "");
  RCC->CSR = csr | RCC_CSR_RMVF;
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

  g4b_uds_boot_st    = &st;
  g4b_uds_boot_stale = stale;

  g4b_printf("listening for a frame (3 s)\r\n");
  g4b_rx_result_t r = g4b_frame_recv(3000u);

  if (r == G4B_RX_OK || g4b_uds_active) {
    g4b_printf("update mode -- not booting\r\n");

    for (;;) {
      if (r == G4B_RX_OK) {
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
        case G4B_MSG_BOOT:
          g4b_handle_boot(&st, stale);
          break;

        default: {
          uint8_t why = G4B_NACK_BAD_TYPE;
          g4b_frame_send(G4B_MSG_NACK, &why, 1u);
          break;
        }
        }
      }

      r = g4b_frame_recv(30000u);
      if (r != G4B_RX_OK &&
          (HAL_GetTick() - g4b_uds_last_ms) >= 30000u) {
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

  if (st.pending != G4B_SLOT_NONE) {
    const char *ps = (st.pending == G4B_SLOT_B) ? "B" : "A";

    if (st.try_count >= G4B_TRY_LIMIT) {
      g4b_printf("pending %s: %u tries, never confirmed -- rolling back\r\n",
                 ps, (unsigned)st.try_count);
      g4b_state_write(stale, st.active, G4B_SLOT_NONE, 0u,
                      st.confirmed, st.seq + 1u);
    } else if (g4b_slot_valid(g4b_slot_base(st.pending))) {
      if (g4b_state_write(stale, st.active, st.pending,
                          (uint8_t)(st.try_count + 1u),
                          st.confirmed, st.seq + 1u)) {
        g4b_printf("trying pending %s, attempt %u of %u\r\n",
                   ps, (unsigned)(st.try_count + 1u), (unsigned)G4B_TRY_LIMIT);
        g4b_iwdg_start();                 /* from here, confirm or die */
        g4b_jump_to_slot(g4b_slot_base(st.pending));
      }
    } else {
      g4b_printf("pending %s invalid -- clearing\r\n", ps);
      g4b_state_write(stale, st.active, G4B_SLOT_NONE, 0u,
                      st.confirmed, st.seq + 1u);
    }
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

static void MX_FDCAN1_Init(void)
{
  /* FDCANSEL resets to HSE, which this board does not have -- with the
     default the peripheral has no clock and Init times out silently. */
  __HAL_RCC_FDCAN_CONFIG(RCC_FDCANCLKSOURCE_PCLK1);
  __HAL_RCC_FDCAN_CLK_ENABLE();

  /* PA12 = FDCAN1_TX, PA11 = FDCAN1_RX */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef g = {
    .Pin = GPIO_PIN_11 | GPIO_PIN_12,
    .Mode = GPIO_MODE_AF_PP,
    .Pull = GPIO_NOPULL,
    .Speed = GPIO_SPEED_FREQ_HIGH,
    .Alternate = GPIO_AF9_FDCAN1,
  };
  HAL_GPIO_Init(GPIOA, &g);

  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  /* 500 kbit/s from 170 MHz PCLK1: /10 = 17 MHz tq, 34 tq/bit = 1+29+4 */
  hfdcan1.Init.NominalPrescaler = 10u;
  hfdcan1.Init.NominalSyncJumpWidth = 4u;
  hfdcan1.Init.NominalTimeSeg1 = 29u;
  hfdcan1.Init.NominalTimeSeg2 = 4u;
  /* data-phase timing: unused in classic mode, must still be legal */
  hfdcan1.Init.DataPrescaler = 5u;
  hfdcan1.Init.DataSyncJumpWidth = 3u;
  hfdcan1.Init.DataTimeSeg1 = 13u;
  hfdcan1.Init.DataTimeSeg2 = 3u;
  hfdcan1.Init.StdFiltersNbr = 0u;
  hfdcan1.Init.ExtFiltersNbr = 0u;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
    g4b_printf("fdcan init failed\r\n");
    return;
  }

  /* no ID filters yet: every standard frame lands in RX FIFO 0 */
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0,
                               FDCAN_ACCEPT_IN_RX_FIFO0,
                               FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
  HAL_FDCAN_Start(&hfdcan1);
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
