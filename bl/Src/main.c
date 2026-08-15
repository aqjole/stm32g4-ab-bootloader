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
UART_HandleTypeDef huart2;

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
  HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)(p - buf), HAL_MAX_DELAY);
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

static void g4b_state_dump(uint32_t page_base, const char *label)
{
  const boot_state_t *s = (const boot_state_t *)page_base;

  g4b_printf("%s @0x%08lX: ", label, (unsigned long)page_base);

  if (s->magic == 0xFFFFFFFFu) {
    g4b_printf("erased\r\n");
    return;
  }

  if (s->magic != G4B_STATE_MAGIC) {
    g4b_printf("not a record (magic 0x%08lX)\r\n", (unsigned long)s->magic);
    return;
  }

  uint32_t actual = g4b_crc32(s, offsetof(boot_state_t, crc32));
  if (actual != s->crc32) {
    g4b_printf("crc bad (stored 0x%08lX, computed 0x%08lX)\r\n",
               (unsigned long)s->crc32, (unsigned long)actual);
    return;
  }

  g4b_printf("seq %lu active %u pending %u try %u confirmed %u\r\n",
             (unsigned long)s->seq, (unsigned)s->active,
             (unsigned)s->pending, (unsigned)s->try_count,
             (unsigned)s->confirmed);
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
  
  g4b_state_dump(G4B_STATE0_BASE, "state0");
  g4b_state_dump(G4B_STATE1_BASE, "state1");

  g4b_state_write(G4B_STATE0_BASE, G4B_SLOT_A, G4B_SLOT_NONE, 0u, 1u, 1u);
  g4b_state_dump(G4B_STATE0_BASE, "state0 after");

  if (g4b_slot_valid(G4B_SLOT_A_BASE)) {
    g4b_printf("booting slot A\r\n");
    g4b_jump_to_slot(G4B_SLOT_A_BASE);
  }
  else if (g4b_slot_valid(G4B_SLOT_B_BASE)) {
    g4b_printf("booting slot B\r\n");
    g4b_jump_to_slot(G4B_SLOT_B_BASE);
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

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
