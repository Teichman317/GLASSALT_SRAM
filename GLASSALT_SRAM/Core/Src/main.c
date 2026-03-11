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
#include "string.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <stdio.h>
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
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x2007c000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x2007c0a0
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x2007c000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x2007c0a0))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
#endif

ETH_TxPacketConfig TxConfig;

DMA2D_HandleTypeDef hdma2d;

ETH_HandleTypeDef heth;

I2C_HandleTypeDef hi2c4;

LTDC_HandleTypeDef hltdc;

QSPI_HandleTypeDef hqspi;

SD_HandleTypeDef hsd1;
DMA_HandleTypeDef hdma_sdmmc1_rx;
DMA_HandleTypeDef hdma_sdmmc1_tx;

SPI_HandleTypeDef hspi6;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

SDRAM_HandleTypeDef hsdram1;

/* USER CODE BEGIN PV */
#define IMG_WIDTH   480
#define IMG_HEIGHT  480
#define IMG_BPP     2                              /* RGB565 = 2 bytes/pixel */
#define TOTAL_BYTES (IMG_WIDTH * IMG_HEIGHT * IMG_BPP)  /* 460800 */

/* Framebuffer in SDRAM */
uint8_t *image_buffer = (uint8_t *)0xC0000000UL;

/* LTDC Layer 1 register block */
#define L1  ((LTDC_Layer_TypeDef *)LTDC_Layer1_BASE)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_FMC_Init(void);
static void MX_ETH_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_DMA2D_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_LTDC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C4_Init(void);
static void MX_SPI6_Init(void);
/* USER CODE BEGIN PFP */
static void MX_Custom_LTDC_Init(void);
static void Init_LCD_Display(void);
static void LCD_WriteCommand(uint8_t cmd);
static void LCD_WriteParameter(uint8_t param);
static int  LoadImageFromSD_Mirrored(const char *filename);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* MPU region 0 (4GB no-access, SubRegionDisable=0x87) blocks sub-regions 3-6,
   * which covers FMC control registers (0xA0000000) and SDRAM (0xC0000000).
   * Add higher-priority regions to allow access to those two ranges. */
  {
    MPU_Region_InitTypeDef r = {0};
    HAL_MPU_Disable();

    /* Region 1: FMC control registers 0xA0000000, 64KB, Device memory */
    r.Enable             = MPU_REGION_ENABLE;
    r.Number             = MPU_REGION_NUMBER1;
    r.BaseAddress        = 0xA0000000;
    r.Size               = MPU_REGION_SIZE_64KB;
    r.SubRegionDisable   = 0x00;
    r.TypeExtField       = MPU_TEX_LEVEL0;
    r.AccessPermission   = MPU_REGION_FULL_ACCESS;
    r.DisableExec        = MPU_INSTRUCTION_ACCESS_DISABLE;
    r.IsShareable        = MPU_ACCESS_SHAREABLE;
    r.IsCacheable        = MPU_ACCESS_NOT_CACHEABLE;
    r.IsBufferable       = MPU_ACCESS_BUFFERABLE;
    HAL_MPU_ConfigRegion(&r);

    /* Region 2: SDRAM 0xC0000000, 32MB, Normal write-through */
    r.Number             = MPU_REGION_NUMBER2;
    r.BaseAddress        = 0xC0000000;
    r.Size               = MPU_REGION_SIZE_32MB;
    r.IsShareable        = MPU_ACCESS_NOT_SHAREABLE;
    r.IsCacheable        = MPU_ACCESS_CACHEABLE;
    r.IsBufferable       = MPU_ACCESS_NOT_BUFFERABLE; /* write-through, no write-allocate */
    HAL_MPU_ConfigRegion(&r);

    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
  }

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_DEVICE_Init(); /* USB first — later inits must not block enumeration */
  MX_FMC_Init();
  MX_ETH_Init();
  MX_SDMMC1_SD_Init();
  MX_DMA2D_Init();
  MX_QUADSPI_Init();
  MX_LTDC_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_I2C4_Init();
  MX_SPI6_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  /* Configure PD7 as push-pull output for debug toggling */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* Diagnostic: 3 fast blinks = reached post-init, USB should be active */
  for (int i = 0; i < 3; i++) {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_Delay(100);
  }
  HAL_Delay(1000); /* Wait for USB enumeration on host */


  /* Comprehensive SDRAM test — new chip, no masking */
  {
    volatile uint32_t *sdram = (volatile uint32_t *)0xC0000000UL;
    char msg[110];
    uint32_t ok = 1;

    /* === Test 1: Data bus — walking 1s at address 0 === */
    CDC_Transmit_FS((uint8_t *)"TEST1: Data bus (walking 1s)\r\n", 29);
    HAL_Delay(5);
    for (int bit = 0; bit < 32; bit++) {
      uint32_t pat = 1UL << bit;
      sdram[0] = pat;
      __DSB();
      SCB_InvalidateDCache();
      uint32_t got = sdram[0];
      uint32_t diff = (pat ^ got);
      if (diff) {
        snprintf(msg, sizeof(msg), "  D%d FAIL: wrote %08lX read %08lX\r\n",
                 bit, (unsigned long)pat, (unsigned long)got);
        CDC_Transmit_FS((uint8_t *)msg, (uint16_t)strlen(msg));
        HAL_Delay(5);
        ok = 0;
      }
    }
    if (ok) {
      CDC_Transmit_FS((uint8_t *)"  Data bus OK\r\n", 15);
      HAL_Delay(5);
    }

    /* === Test 2: Address bus — power-of-2 offsets, 8MB range ===
     * FMC mapping for 16-bit bus, 8-col, 12-row, 4-bank:
     *   byte_addr[8:1]   → Column A[7:0]
     *   byte_addr[20:9]  → Row A[11:0]
     *   byte_addr[22:21] → Bank BA[1:0]
     * Word offsets that exercise each SDRAM signal:
     *   0x1..0x40      → Column A0..A7
     *   0x80..0x40000  → Row A0..A11
     *   0x80000        → BA0
     *   0x100000       → BA1
     */
    CDC_Transmit_FS((uint8_t *)"TEST2: Address bus (8MB range)\r\n", 31);
    HAL_Delay(5);
    ok = 1;
    const uint32_t addrTestMax = 2UL * 1024UL * 1024UL; /* 2M words = 8MB */

    /* Seed address 0 */
    sdram[0] = 0xAAAAAAAAUL;
    /* Write unique pattern at each power-of-2 word offset */
    for (uint32_t offset = 1; offset < addrTestMax; offset <<= 1)
      sdram[offset] = ~offset;
    __DSB();
    SCB_InvalidateDCache();

    /* Check address 0 wasn't overwritten (alias detection) */
    {
      uint32_t got = sdram[0];
      uint32_t diff = (got ^ 0xAAAAAAAAUL);
      if (diff) {
        snprintf(msg, sizeof(msg), "  Addr 0 alias! got %08lX\r\n",
                 (unsigned long)got);
        CDC_Transmit_FS((uint8_t *)msg, (uint16_t)strlen(msg));
        HAL_Delay(5);
        ok = 0;
      }
    }

    /* Check each power-of-2 offset, label which SDRAM signal it tests */
    for (uint32_t offset = 1; offset < addrTestMax; offset <<= 1) {
      uint32_t expected = ~offset;
      uint32_t got = sdram[offset];
      uint32_t diff = (expected ^ got);

      /* Identify which SDRAM signal this offset exercises */
      const char *sig = "?";
      uint32_t byteOff = offset * 4;
      /* 8-col-bit mapping: byte[8:1]→Col, byte[20:9]→Row, byte[22:21]→Bank */
      if      (byteOff == 0x00000004UL) sig = "ColA1";
      else if (byteOff == 0x00000008UL) sig = "ColA2";
      else if (byteOff == 0x00000010UL) sig = "ColA3";
      else if (byteOff == 0x00000020UL) sig = "ColA4";
      else if (byteOff == 0x00000040UL) sig = "ColA5";
      else if (byteOff == 0x00000080UL) sig = "ColA6";
      else if (byteOff == 0x00000100UL) sig = "ColA7";
      else if (byteOff == 0x00000200UL) sig = "RowA0";
      else if (byteOff == 0x00000400UL) sig = "RowA1";
      else if (byteOff == 0x00000800UL) sig = "RowA2";
      else if (byteOff == 0x00001000UL) sig = "RowA3";
      else if (byteOff == 0x00002000UL) sig = "RowA4";
      else if (byteOff == 0x00004000UL) sig = "RowA5";
      else if (byteOff == 0x00008000UL) sig = "RowA6";
      else if (byteOff == 0x00010000UL) sig = "RowA7";
      else if (byteOff == 0x00020000UL) sig = "RowA8";
      else if (byteOff == 0x00040000UL) sig = "RowA9";
      else if (byteOff == 0x00080000UL) sig = "RowA10";
      else if (byteOff == 0x00100000UL) sig = "RowA11";
      else if (byteOff == 0x00200000UL) sig = "BA0";
      else if (byteOff == 0x00400000UL) sig = "BA1";

      if (diff) {
        snprintf(msg, sizeof(msg), "  %s FAIL @0x%lX: exp %08lX got %08lX\r\n",
                 sig, (unsigned long)byteOff,
                 (unsigned long)expected, (unsigned long)got);
        CDC_Transmit_FS((uint8_t *)msg, (uint16_t)strlen(msg));
        HAL_Delay(5);
        ok = 0;
      } else {
        snprintf(msg, sizeof(msg), "  %s OK\r\n", sig);
        CDC_Transmit_FS((uint8_t *)msg, (uint16_t)strlen(msg));
        HAL_Delay(5);
      }
    }

    /* === Test 3: Pattern sweep — full 1MB, masking A8 alias === */
    CDC_Transmit_FS((uint8_t *)"TEST3: 1MB pattern sweep\r\n", 25);
    HAL_Delay(5);
    const uint32_t sweepWords = 256UL * 1024UL;
    uint32_t errors = 0;
    uint32_t errBitOr = 0;
    for (uint32_t i = 0; i < sweepWords; i++)
      sdram[i] = i ^ 0xA5A5A5A5UL;
    __DSB();
    SCB_InvalidateDCache();
    for (uint32_t i = 0; i < sweepWords; i++) {
      uint32_t expected = i ^ 0xA5A5A5A5UL;
      uint32_t got = sdram[i];
      uint32_t diff = (expected ^ got);
      if (diff) {
        errBitOr |= diff;
        errors++;
        if (errors <= 3) {
          snprintf(msg, sizeof(msg), "  ERR[%lu] exp %08lX got %08lX xor %08lX\r\n",
                   (unsigned long)i, (unsigned long)expected,
                   (unsigned long)got, (unsigned long)diff);
          CDC_Transmit_FS((uint8_t *)msg, (uint16_t)strlen(msg));
          HAL_Delay(5);
        }
      }
    }
    snprintf(msg, sizeof(msg), "  Sweep: %lu errs, failBits=%08lX\r\n",
             (unsigned long)errors, (unsigned long)errBitOr);
    CDC_Transmit_FS((uint8_t *)msg, (uint16_t)strlen(msg));
    HAL_Delay(5);

    /* === Test 4: Targeted A8 read/write isolation ===
     * A8 is column bit 8.  With 16-bit bus, col A8 = byte_addr bit 9
     * → word offset 0x80 (bit 7 of word address).
     * We test whether A8 fails on write, read, or both. */
    CDC_Transmit_FS((uint8_t *)"TEST4: A8 isolation (read vs write)\r\n", 36);
    HAL_Delay(5);
    {
      /* Pick two addresses differing only in A8 (column bit 8) */
      const uint32_t base = 0;       /* A8=0 */
      const uint32_t a8   = 0x80;    /* A8=1 (word offset) */

      /* Sub-test A: write to both, read both back immediately */
      sdram[base] = 0x11111111UL;
      sdram[a8]   = 0x22222222UL;
      __DSB(); SCB_InvalidateDCache();
      uint32_t rb = sdram[base];
      uint32_t ra = sdram[a8];

      snprintf(msg, sizeof(msg),
        "  Write base=0x11111111, a8=0x22222222\r\n");
      CDC_Transmit_FS((uint8_t *)msg, (uint16_t)strlen(msg)); HAL_Delay(5);
      snprintf(msg, sizeof(msg),
        "  Read  base=%08lX  a8=%08lX\r\n",
        (unsigned long)rb, (unsigned long)ra);
      CDC_Transmit_FS((uint8_t *)msg, (uint16_t)strlen(msg)); HAL_Delay(5);

      if (rb == 0x11111111UL && ra == 0x22222222UL) {
        CDC_Transmit_FS((uint8_t *)"  Sub-A: PASS — both distinct\r\n", 31);
      } else if (rb == ra) {
        CDC_Transmit_FS((uint8_t *)"  Sub-A: FAIL — aliased (same value)\r\n", 38);
        ok = 0;
      } else {
        CDC_Transmit_FS((uint8_t *)"  Sub-A: FAIL — corruption\r\n", 28);
        ok = 0;
      }
      HAL_Delay(5);

      /* Sub-test B: write base, then write a8, then check if base survived
       * If A8 stuck low on WRITE, writing to a8 overwrites base */
      sdram[base] = 0xDEADBEEFUL;
      __DSB(); SCB_InvalidateDCache();
      sdram[a8]   = 0xCAFEBABEUL;
      __DSB(); SCB_InvalidateDCache();
      rb = sdram[base];

      snprintf(msg, sizeof(msg),
        "  Wrote base=DEADBEEF, then a8=CAFEBABE\r\n");
      CDC_Transmit_FS((uint8_t *)msg, (uint16_t)strlen(msg)); HAL_Delay(5);
      snprintf(msg, sizeof(msg),
        "  Re-read base=%08lX  (expect DEADBEEF)\r\n", (unsigned long)rb);
      CDC_Transmit_FS((uint8_t *)msg, (uint16_t)strlen(msg)); HAL_Delay(5);

      if (rb == 0xDEADBEEFUL)
        CDC_Transmit_FS((uint8_t *)"  Sub-B: PASS — write A8 works\r\n", 32);
      else if (rb == 0xCAFEBABEUL) {
        CDC_Transmit_FS((uint8_t *)"  Sub-B: FAIL — A8 stuck on WRITE\r\n", 35);
        ok = 0;
      } else {
        snprintf(msg, sizeof(msg), "  Sub-B: FAIL — unexpected %08lX\r\n",
                 (unsigned long)rb);
        CDC_Transmit_FS((uint8_t *)msg, (uint16_t)strlen(msg));
        ok = 0;
      }
      HAL_Delay(5);

      /* Sub-test C: write distinct values, read only via a8 address
       * If A8 stuck low on READ, reading a8 returns base's value */
      sdram[base] = 0x55550000UL;
      sdram[a8]   = 0x0000AAAUL;
      __DSB(); SCB_InvalidateDCache();
      ra = sdram[a8];

      snprintf(msg, sizeof(msg),
        "  Wrote base=55550000, a8=00000AAA\r\n");
      CDC_Transmit_FS((uint8_t *)msg, (uint16_t)strlen(msg)); HAL_Delay(5);
      snprintf(msg, sizeof(msg),
        "  Read  a8=%08lX  (expect 00000AAA)\r\n", (unsigned long)ra);
      CDC_Transmit_FS((uint8_t *)msg, (uint16_t)strlen(msg)); HAL_Delay(5);

      if (ra == 0x00000AAAUL)
        CDC_Transmit_FS((uint8_t *)"  Sub-C: PASS — read A8 works\r\n", 31);
      else if (ra == 0x55550000UL) {
        CDC_Transmit_FS((uint8_t *)"  Sub-C: FAIL — A8 stuck on READ\r\n", 34);
        ok = 0;
      } else {
        snprintf(msg, sizeof(msg), "  Sub-C: FAIL — unexpected %08lX\r\n",
                 (unsigned long)ra);
        CDC_Transmit_FS((uint8_t *)msg, (uint16_t)strlen(msg));
        ok = 0;
      }
      HAL_Delay(5);

      /* Sub-test D: repeat with row-address A8 (word offset 0x20000)
       * to also check row A8 which is the same physical pin PF14 */
      const uint32_t rowA8 = 0x20000UL;  /* byte offset 0x80000 → row A8 */
      sdram[0]     = 0x12345678UL;
      sdram[rowA8] = 0x9ABCDEF0UL;
      __DSB(); SCB_InvalidateDCache();
      rb = sdram[0];
      ra = sdram[rowA8];

      snprintf(msg, sizeof(msg),
        "  RowA8: base=%08lX(exp 12345678)  rowA8=%08lX(exp 9ABCDEF0)\r\n",
        (unsigned long)rb, (unsigned long)ra);
      CDC_Transmit_FS((uint8_t *)msg, (uint16_t)strlen(msg)); HAL_Delay(5);

      if (rb == 0x12345678UL && ra == 0x9ABCDEF0UL)
        CDC_Transmit_FS((uint8_t *)"  Sub-D: PASS — row A8 OK\r\n", 27);
      else {
        CDC_Transmit_FS((uint8_t *)"  Sub-D: FAIL — row A8 problem\r\n", 32);
        ok = 0;
      }
      HAL_Delay(5);
    }

    /* === Summary === */
    if (errors == 0 && ok)
      CDC_Transmit_FS((uint8_t *)"SDRAM: ALL PASS\r\n", 17);
    else
      CDC_Transmit_FS((uint8_t *)"SDRAM: ERRORS found\r\n", 21);
  }

  /* ===== LCD Display Init ===== */
  CDC_Transmit_FS((uint8_t *)"LCD: init SPI display...\r\n", 25);
  HAL_Delay(10);

  /* SPI6 sanity check — verify HAL_SPI_Transmit works */
  {
    uint16_t test = 0x0011;  /* Sleep Out command */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_StatusTypeDef rc = HAL_SPI_Transmit(&hspi6, (uint8_t *)&test, 1, 1000);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
    char smsg[40];
    snprintf(smsg, sizeof(smsg), "SPI6 test: rc=%d state=%d\r\n", (int)rc, (int)hspi6.State);
    CDC_Transmit_FS((uint8_t *)smsg, (uint16_t)strlen(smsg));
    HAL_Delay(10);
  }

  Init_LCD_Display();

  /* Fill framebuffer with solid color test pattern (bypass SD for now) */
  {
    uint16_t *fb = (uint16_t *)image_buffer;
    uint16_t color = 0xF800;  /* Red in RGB565 */
    for (int i = 0; i < IMG_WIDTH * IMG_HEIGHT; i++) fb[i] = color;
    CDC_Transmit_FS((uint8_t *)"LCD: filled red test pattern\r\n", 29);
    HAL_Delay(10);
  }
  MX_Custom_LTDC_Init();
  CDC_Transmit_FS((uint8_t *)"LCD: LTDC started\r\n", 19);
  HAL_Delay(10);

  /* ETH diagnostics: report init result and read PHY ID registers */
  HAL_Delay(50); /* let CDC drain before ETH diag */
  {
    char emsg[80];
    snprintf(emsg, sizeof(emsg), "ETH: gState=%lu\r\n", (unsigned long)heth.gState);
    CDC_Transmit_FS((uint8_t *)emsg, (uint16_t)strlen(emsg));
    HAL_Delay(20);

    #define PHY_ADDR  0x00U
    uint32_t regval = 0;

    /* Try reading PHY ID regardless of init state */
    HAL_StatusTypeDef rc;
    rc = HAL_ETH_ReadPHYRegister(&heth, PHY_ADDR, 2, &regval);
    snprintf(emsg, sizeof(emsg), "ETH: PHY ID1=0x%04lX (rc=%d)\r\n",
             (unsigned long)regval, (int)rc);
    CDC_Transmit_FS((uint8_t *)emsg, (uint16_t)strlen(emsg));
    HAL_Delay(20);

    rc = HAL_ETH_ReadPHYRegister(&heth, PHY_ADDR, 3, &regval);
    snprintf(emsg, sizeof(emsg), "ETH: PHY ID2=0x%04lX (rc=%d)\r\n",
             (unsigned long)regval, (int)rc);
    CDC_Transmit_FS((uint8_t *)emsg, (uint16_t)strlen(emsg));
    HAL_Delay(20);

    rc = HAL_ETH_ReadPHYRegister(&heth, PHY_ADDR, 1, &regval);
    snprintf(emsg, sizeof(emsg), "ETH: BSR=0x%04lX rc=%d (%s)\r\n",
             (unsigned long)regval, (int)rc,
             (regval & 0x0004) ? "link up" : "no link");
    CDC_Transmit_FS((uint8_t *)emsg, (uint16_t)strlen(emsg));
    HAL_Delay(20);

    /* Also scan PHY addresses 0-3 in case strapping is different */
    for (uint32_t addr = 0; addr <= 3; addr++) {
      uint32_t id1 = 0, id2 = 0;
      HAL_ETH_ReadPHYRegister(&heth, addr, 2, &id1);
      HAL_ETH_ReadPHYRegister(&heth, addr, 3, &id2);
      if (id1 != 0x0000 && id1 != 0xFFFF) {
        snprintf(emsg, sizeof(emsg), "ETH: PHY@%lu ID=%04lX:%04lX\r\n",
                 (unsigned long)addr, (unsigned long)id1, (unsigned long)id2);
        CDC_Transmit_FS((uint8_t *)emsg, (uint16_t)strlen(emsg));
        HAL_Delay(20);
      }
    }
  }

  /* Start backlight PWM on TIM4 CH2 (PD13 -> TPS61043 CTRL pin)
   * TIM4 clk = 108 MHz, Period = 65535 -> ~1648 Hz PWM
   * 50% duty cycle for initial test */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 32768);

  /* FATFS: directory listing disabled — SD card init hanging, debug separately */
  #if 0
  {
    static FATFS fs;
    DIR dir;
    FILINFO fno;
    FRESULT fr;
    char fsmsg[80];

    fr = f_mount(&fs, "", 1);
    if (fr == FR_OK) {
      CDC_Transmit_FS((uint8_t *)"FS: mounted\r\n", 13);
      HAL_Delay(10);
      fr = f_opendir(&dir, "/");
      if (fr == FR_OK) {
        while (1) {
          fr = f_readdir(&dir, &fno);
          if (fr != FR_OK || fno.fname[0] == 0) break;
          snprintf(fsmsg, sizeof(fsmsg), "%s %s  %lu bytes\r\n",
                   (fno.fattrib & AM_DIR) ? "[DIR] " : "[FILE]",
                   fno.fname, (unsigned long)fno.fsize);
          CDC_Transmit_FS((uint8_t *)fsmsg, (uint16_t)strlen(fsmsg));
          HAL_Delay(10);
        }
        f_closedir(&dir);
        CDC_Transmit_FS((uint8_t *)"FS: done\r\n", 10);
      } else {
        snprintf(fsmsg, sizeof(fsmsg), "FS: opendir failed %d\r\n", (int)fr);
        CDC_Transmit_FS((uint8_t *)fsmsg, (uint16_t)strlen(fsmsg));
      }
    } else {
      snprintf(fsmsg, sizeof(fsmsg), "FS: mount failed %d\r\n", (int)fr);
      CDC_Transmit_FS((uint8_t *)fsmsg, (uint16_t)strlen(fsmsg));
    }
    f_mount(NULL, "", 0);
  }
  #endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Continuous toggle — frequency encodes CDC result:
       200 Hz = OK (0), 400 Hz = BUSY (1), 800 Hz = FAIL (2) */
    {
      uint8_t cdc_rc = CDC_Transmit_FS((uint8_t *)"Hello World!\r\n", 14);
      uint32_t half_period;
      if (cdc_rc == 0)      half_period = 2500;  /* 2.5ms half → 200 Hz */
      else if (cdc_rc == 1) half_period = 1250;  /* 1.25ms half → 400 Hz */
      else                  half_period = 625;   /* 0.625ms half → 800 Hz */

      /* Toggle for ~500ms at that frequency, then re-check */
      uint32_t start = HAL_GetTick();
      while (HAL_GetTick() - start < 500) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
        for (volatile uint32_t d = 0; d < half_period * 27; d++);  /* ~us at 216MHz */
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
        for (volatile uint32_t d = 0; d < half_period * 27; d++);
      }
    }
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB4444;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB4444;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  hdma2d.LayerCfg[1].AlphaInverted = DMA2D_REGULAR_ALPHA;
  hdma2d.LayerCfg[1].RedBlueSwap = DMA2D_RB_REGULAR;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */
  /* Drive PA1 HIGH briefly before ETH init — workaround for REFCLKO/nRST interaction */
  {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef geth = {0};
    geth.Pin   = GPIO_PIN_1;
    geth.Mode  = GPIO_MODE_OUTPUT_PP;
    geth.Pull  = GPIO_NOPULL;
    geth.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &geth);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_Delay(10);
  }
  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    /* Non-fatal: ETH PHY not operational. NOTE: CubeMX regen restores Error_Handler() here — re-apply after every regen. */
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x20404768;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};
  LTDC_LayerCfgTypeDef pLayerCfg1 = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 7;
  hltdc.Init.VerticalSync = 3;
  hltdc.Init.AccumulatedHBP = 14;
  hltdc.Init.AccumulatedVBP = 5;
  hltdc.Init.AccumulatedActiveW = 654;
  hltdc.Init.AccumulatedActiveH = 485;
  hltdc.Init.TotalWidth = 660;
  hltdc.Init.TotalHeigh = 487;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 0;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 0;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg.Alpha = 0;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = 0;
  pLayerCfg.ImageWidth = 0;
  pLayerCfg.ImageHeight = 0;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg1.WindowX0 = 0;
  pLayerCfg1.WindowX1 = 0;
  pLayerCfg1.WindowY0 = 0;
  pLayerCfg1.WindowY1 = 0;
  pLayerCfg1.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg1.Alpha = 0;
  pLayerCfg1.Alpha0 = 0;
  pLayerCfg1.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg1.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg1.FBStartAdress = 0;
  pLayerCfg1.ImageWidth = 0;
  pLayerCfg1.ImageHeight = 0;
  pLayerCfg1.Backcolor.Blue = 0;
  pLayerCfg1.Backcolor.Green = 0;
  pLayerCfg1.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg1, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */
  /* GPIO speed now fixed directly in MSP (VERY_HIGH for all LTDC pins) */

  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 255;
  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.FlashSize = 1;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;
  hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 4;  /* Slow down SDMMC clock for reliable card init */
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * @brief SPI6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI6_Init(void)
{

  /* USER CODE BEGIN SPI6_Init 0 */

  /* USER CODE END SPI6_Init 0 */

  /* USER CODE BEGIN SPI6_Init 1 */

  /* USER CODE END SPI6_Init 1 */
  /* SPI6 parameter configuration*/
  hspi6.Instance = SPI6;
  hspi6.Init.Mode = SPI_MODE_MASTER;
  hspi6.Init.Direction = SPI_DIRECTION_1LINE;        /* TX-only simplex */
  hspi6.Init.DataSize = SPI_DATASIZE_9BIT;           /* 9-bit: bit8=D/C flag */
  hspi6.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi6.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi6.Init.NSS = SPI_NSS_SOFT;
  hspi6.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; /* APB2/16 ≈ 6.75 MHz */
  hspi6.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi6.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi6.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi6.Init.CRCPolynomial = 7;
  hspi6.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi6.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;       /* Manual CS via GPIO */
  if (HAL_SPI_Init(&hspi6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI6_Init 2 */

  /* USER CODE END SPI6_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK1;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_ENABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_2;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 2;
  SdramTiming.ExitSelfRefreshDelay = 8;
  SdramTiming.SelfRefreshTime = 5;
  SdramTiming.RowCycleDelay = 7;
  SdramTiming.WriteRecoveryTime = 2;
  SdramTiming.RPDelay = 2;
  SdramTiming.RCDDelay = 16;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */
  {
    FMC_SDRAM_CommandTypeDef cmd = {0};

    /* Step 1: Clock Configuration Enable */
    cmd.CommandMode           = FMC_SDRAM_CMD_CLK_ENABLE;
    cmd.CommandTarget         = FMC_SDRAM_CMD_TARGET_BANK1;
    cmd.AutoRefreshNumber     = 1;
    cmd.ModeRegisterDefinition = 0;
    HAL_SDRAM_SendCommand(&hsdram1, &cmd, 100);
    HAL_Delay(1); /* datasheet requires min 100 us after CLK_ENABLE */

    /* Step 2: Precharge All Banks */
    cmd.CommandMode = FMC_SDRAM_CMD_PALL;
    HAL_SDRAM_SendCommand(&hsdram1, &cmd, 100);

    /* Step 3: Two Auto-Refresh cycles minimum (use 8 to be safe) */
    cmd.CommandMode       = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
    cmd.AutoRefreshNumber = 8;
    HAL_SDRAM_SendCommand(&hsdram1, &cmd, 100);

    /* Step 4: Load Mode Register
     * CAS Latency=2, Burst Length=1, Sequential, Write Burst=Single Location */
    cmd.CommandMode            = FMC_SDRAM_CMD_LOAD_MODE;
    cmd.AutoRefreshNumber      = 1;
    cmd.ModeRegisterDefinition = 0x0230; /* BL=1, CAS=3, Write Burst=Single */
    HAL_SDRAM_SendCommand(&hsdram1, &cmd, 100);

    /* Step 5: Set auto-refresh rate
     * tREF=64ms, rows=4096, SDRAM_CLK=108MHz
     * COUNT = (64ms / 4096) * 108MHz - 20 = 1667 */
    HAL_SDRAM_ProgramRefreshRate(&hsdram1, 1667);
  }
  /* USER CODE END FMC_Init 2 */
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TP_RESET_PD11_GPIO_Port, TP_RESET_PD11_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PG2_LDC_RESET_GPIO_Port, PG2_LDC_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PA15_SPI2_CS_GPIO_Port, PA15_SPI2_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SW_RESET_PE3_Pin SW_CNTR_PE4_Pin SW_TEST_PE5_Pin */
  GPIO_InitStruct.Pin = SW_RESET_PE3_Pin|SW_CNTR_PE4_Pin|SW_TEST_PE5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : TP_RESET_PD11_Pin */
  GPIO_InitStruct.Pin = TP_RESET_PD11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TP_RESET_PD11_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PG2_LDC_RESET_Pin */
  GPIO_InitStruct.Pin = PG2_LDC_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PG2_LDC_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TP_INT_PG3_Pin */
  GPIO_InitStruct.Pin = TP_INT_PG3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TP_INT_PG3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15_SPI2_CS_Pin */
  GPIO_InitStruct.Pin = PA15_SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PA15_SPI2_CS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* ===== ST7701S 9-bit SPI helpers (SPI6, CS=PA15, Reset=PG2) ===== */

static void LCD_WriteCommand(uint8_t cmd)
{
    uint16_t frame = (uint16_t)cmd & 0x00FF;  /* bit8=0 → command */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi6, (uint8_t *)&frame, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
}

static void LCD_WriteParameter(uint8_t param)
{
    uint16_t frame = ((uint16_t)param & 0x00FF) | (1 << 8);  /* bit8=1 → data */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi6, (uint8_t *)&frame, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
}

/* ===== ST7701S display register init (proven values — do not change) ===== */

static void Init_LCD_Display(void)
{
    /* Reset display */
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_Delay(120);

    LCD_WriteCommand(0x11);  HAL_Delay(120);
    LCD_WriteCommand(0x29);  HAL_Delay(20);

    /* Page 0x13: Power configuration */
    LCD_WriteCommand(0xFF);
    LCD_WriteParameter(0x77); LCD_WriteParameter(0x01);
    LCD_WriteParameter(0x00); LCD_WriteParameter(0x00);
    LCD_WriteParameter(0x13);
    LCD_WriteCommand(0xEF); LCD_WriteParameter(0x08);

    /* Page 0x10: Display driving settings */
    LCD_WriteCommand(0xFF);
    LCD_WriteParameter(0x77); LCD_WriteParameter(0x01);
    LCD_WriteParameter(0x00); LCD_WriteParameter(0x00);
    LCD_WriteParameter(0x10);

    LCD_WriteCommand(0xC0); LCD_WriteParameter(0x3B); LCD_WriteParameter(0x00);
    LCD_WriteCommand(0xC1); LCD_WriteParameter(0x10); LCD_WriteParameter(0x0C);
    LCD_WriteCommand(0xC2); LCD_WriteParameter(0x07); LCD_WriteParameter(0x0A);
    LCD_WriteCommand(0xC7); LCD_WriteParameter(0x04);
    LCD_WriteCommand(0xCC); LCD_WriteParameter(0x10);
    LCD_WriteCommand(0xCD); LCD_WriteParameter(0x08);

    /* Gamma positive */
    LCD_WriteCommand(0xB0);
    { uint8_t g[] = {0x05,0x12,0x98,0x0E,0x0F,0x07,0x07,0x09,0x09,0x23,0x05,0x52,0x0F,0x67,0x2C,0x11};
      for (int i = 0; i < (int)sizeof(g); i++) LCD_WriteParameter(g[i]); }

    /* Gamma negative */
    LCD_WriteCommand(0xB1);
    { uint8_t g[] = {0x0B,0x11,0x97,0x0C,0x12,0x06,0x06,0x08,0x08,0x22,0x03,0x51,0x11,0x66,0x2B,0x0F};
      for (int i = 0; i < (int)sizeof(g); i++) LCD_WriteParameter(g[i]); }

    /* Page 0x11: Interface and timing */
    LCD_WriteCommand(0xFF);
    LCD_WriteParameter(0x77); LCD_WriteParameter(0x01);
    LCD_WriteParameter(0x00); LCD_WriteParameter(0x00);
    LCD_WriteParameter(0x11);

    LCD_WriteCommand(0xB0); LCD_WriteParameter(0x00);
    LCD_WriteCommand(0xB1); LCD_WriteParameter(0x3E);
    LCD_WriteCommand(0xB2); LCD_WriteParameter(0x81);
    LCD_WriteCommand(0xB3); LCD_WriteParameter(0x80);
    LCD_WriteCommand(0xB5); LCD_WriteParameter(0x4E);
    LCD_WriteCommand(0xB7); LCD_WriteParameter(0x85);
    LCD_WriteCommand(0xB8); LCD_WriteParameter(0x20);
    LCD_WriteCommand(0xC1); LCD_WriteParameter(0x78);
    LCD_WriteCommand(0xC2); LCD_WriteParameter(0x78);
    LCD_WriteCommand(0xD0); LCD_WriteParameter(0x88);

    /* Panel timing and drive strength */
    LCD_WriteCommand(0xE0);
    { uint8_t p[] = {0x00,0x00,0x02};
      for (int i = 0; i < (int)sizeof(p); i++) LCD_WriteParameter(p[i]); }

    LCD_WriteCommand(0xE1);
    { uint8_t p[] = {0x06,0x30,0x08,0x30,0x05,0x30,0x07,0x30,0x00,0x33,0x33};
      for (int i = 0; i < (int)sizeof(p); i++) LCD_WriteParameter(p[i]); }

    LCD_WriteCommand(0xE2);
    { uint8_t p[] = {0x11,0x11,0x33,0x33,0xF4,0x00,0x00,0x00,0xF4,0x00,0x00,0x00};
      for (int i = 0; i < (int)sizeof(p); i++) LCD_WriteParameter(p[i]); }

    LCD_WriteCommand(0xE3);
    LCD_WriteParameter(0x00); LCD_WriteParameter(0x00);
    LCD_WriteParameter(0x11); LCD_WriteParameter(0x11);

    LCD_WriteCommand(0xE4);
    LCD_WriteParameter(0x44); LCD_WriteParameter(0x44);

    LCD_WriteCommand(0xE5);
    { uint8_t p[] = {0x0D,0xF5,0x30,0xF0,0x0F,0xF7,0x30,0xF0,0x09,0xF1,0x30,0xF0,0x0B,0xF3,0x30,0xF0};
      for (int i = 0; i < (int)sizeof(p); i++) LCD_WriteParameter(p[i]); }

    LCD_WriteCommand(0xE6);
    LCD_WriteParameter(0x00); LCD_WriteParameter(0x00);
    LCD_WriteParameter(0x11); LCD_WriteParameter(0x11);

    LCD_WriteCommand(0xE7);
    LCD_WriteParameter(0x44); LCD_WriteParameter(0x44);

    LCD_WriteCommand(0xE8);
    { uint8_t p[] = {0x0C,0xF4,0x30,0xF0,0x0E,0xF6,0x30,0xF0,0x08,0xF0,0x30,0xF0,0x0A,0xF2,0x30,0xF0};
      for (int i = 0; i < (int)sizeof(p); i++) LCD_WriteParameter(p[i]); }

    LCD_WriteCommand(0xE9); LCD_WriteParameter(0x36); LCD_WriteParameter(0x01);

    LCD_WriteCommand(0xEB);
    { uint8_t p[] = {0x00,0x01,0xE4,0xE4,0x44,0x88,0x40};
      for (int i = 0; i < (int)sizeof(p); i++) LCD_WriteParameter(p[i]); }

    LCD_WriteCommand(0xED);
    { uint8_t p[] = {0xFF,0x10,0xAF,0x76,0x54,0x2B,0xCF,0xFF,0xFF,0xFC,0xB2,0x45,0x67,0xFA,0x01,0xFF};
      for (int i = 0; i < (int)sizeof(p); i++) LCD_WriteParameter(p[i]); }

    LCD_WriteCommand(0xEF);
    LCD_WriteParameter(0x08); LCD_WriteParameter(0x08);

    LCD_WriteCommand(0xEF);
    LCD_WriteParameter(0x08); LCD_WriteParameter(0x08);
    LCD_WriteParameter(0x08); LCD_WriteParameter(0x45);
    LCD_WriteParameter(0x3F); LCD_WriteParameter(0x54);

    /* Return to default page */
    LCD_WriteCommand(0xFF);
    LCD_WriteParameter(0x77); LCD_WriteParameter(0x01);
    LCD_WriteParameter(0x00); LCD_WriteParameter(0x00);
    LCD_WriteParameter(0x00);

    LCD_WriteCommand(0x11); HAL_Delay(120);
    LCD_WriteCommand(0x3A); LCD_WriteParameter(0x66);  /* 18-bit RGB666 interface */
    LCD_WriteCommand(0x36); LCD_WriteParameter(0x00);  /* Default scan direction */
    LCD_WriteCommand(0x35); LCD_WriteParameter(0x00);  /* Tearing effect ON */
    LCD_WriteCommand(0x29);                            /* Display ON */
}

/* ===== Custom LTDC init with proven timing (do not change values) ===== */

static void MX_Custom_LTDC_Init(void)
{
    LTDC_LayerCfgTypeDef pLayerCfg = {0};

    hltdc.Instance = LTDC;
    hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
    hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
    hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
    hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
    /* Typical ST7701S 480x480 timing — 24 MHz pixel clock → ~60 Hz */
    hltdc.Init.HorizontalSync = 9;     /* HSW = 10 */
    hltdc.Init.VerticalSync = 1;       /* VSW = 2 */
    hltdc.Init.AccumulatedHBP = 19;    /* HSW+HBP = 10+10 = 20 */
    hltdc.Init.AccumulatedVBP = 15;    /* VSW+VBP = 2+14 = 16 */
    hltdc.Init.AccumulatedActiveW = 499;  /* +480 = 500 */
    hltdc.Init.AccumulatedActiveH = 495;  /* +480 = 496 */
    hltdc.Init.TotalWidth = 519;       /* +20 HFP = 520 */
    hltdc.Init.TotalHeigh = 511;       /* +16 VFP = 512 */
    hltdc.Init.Backcolor.Blue = 0;
    hltdc.Init.Backcolor.Green = 0;
    hltdc.Init.Backcolor.Red = 0;
    if (HAL_LTDC_Init(&hltdc) != HAL_OK)
    {
        Error_Handler();
    }

    /* Disable layer 1 (left over from CubeMX init with garbage FBStartAddr=0) */
    __HAL_LTDC_LAYER_DISABLE(&hltdc, 1);

    pLayerCfg.WindowX0 = 0;
    pLayerCfg.WindowX1 = 480;  /* HAL uses exclusive end */
    pLayerCfg.WindowY0 = 0;
    pLayerCfg.WindowY1 = 480;
    pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
    pLayerCfg.Alpha = 255;
    pLayerCfg.Alpha0 = 0;
    pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
    pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
    pLayerCfg.FBStartAdress = (uint32_t)image_buffer;
    pLayerCfg.ImageWidth = IMG_WIDTH;
    pLayerCfg.ImageHeight = IMG_HEIGHT;
    pLayerCfg.Backcolor.Blue = 0;
    pLayerCfg.Backcolor.Green = 0;
    pLayerCfg.Backcolor.Red = 0;
    if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
    {
        Error_Handler();
    }

    __HAL_LTDC_RELOAD_IMMEDIATE_CONFIG(&hltdc);
}

/* ===== Load image.bin from SD card into SDRAM (mirrored) ===== */

static int LoadImageFromSD_Mirrored(const char *filename)
{
    FRESULT fr;
    static FATFS fs;
    FIL file;
    UINT bytes_read;
    char msg[80];

    hsd1.State = HAL_SD_STATE_READY;

    fr = f_mount(&fs, "", 1);
    if (fr != FR_OK) {
        snprintf(msg, sizeof(msg), "LCD: mount fail %d\r\n", (int)fr);
        CDC_Transmit_FS((uint8_t *)msg, (uint16_t)strlen(msg));
        HAL_Delay(10);
        return -1;
    }

    fr = f_open(&file, filename, FA_READ);
    if (fr != FR_OK) {
        snprintf(msg, sizeof(msg), "LCD: open fail %s err %d\r\n", filename, (int)fr);
        CDC_Transmit_FS((uint8_t *)msg, (uint16_t)strlen(msg));
        HAL_Delay(10);
        f_mount(NULL, "", 0);
        return -1;
    }

    fr = f_read(&file, image_buffer, TOTAL_BYTES, &bytes_read);
    f_close(&file);
    f_mount(NULL, "", 0);

    if (fr != FR_OK || bytes_read != TOTAL_BYTES) {
        snprintf(msg, sizeof(msg), "LCD: read fail %d got %lu exp %lu\r\n",
                 (int)fr, (unsigned long)bytes_read, (unsigned long)TOTAL_BYTES);
        CDC_Transmit_FS((uint8_t *)msg, (uint16_t)strlen(msg));
        HAL_Delay(10);
        return -1;
    }

    /* Mirror image horizontally (in-place) */
    for (int y = 0; y < IMG_HEIGHT; y++) {
        uint8_t *row = &image_buffer[y * IMG_WIDTH * IMG_BPP];
        for (int x = 0; x < IMG_WIDTH / 2; x++) {
            int left  = x * 2;
            int right = (IMG_WIDTH - 1 - x) * 2;
            uint8_t t0 = row[left];     uint8_t t1 = row[left + 1];
            row[left]     = row[right];   row[left + 1]  = row[right + 1];
            row[right]    = t0;           row[right + 1] = t1;
        }
    }

    snprintf(msg, sizeof(msg), "LCD: loaded %s (%lu bytes)\r\n",
             filename, (unsigned long)bytes_read);
    CDC_Transmit_FS((uint8_t *)msg, (uint16_t)strlen(msg));
    HAL_Delay(10);
    return 0;
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* TEMP DIAG: IRQ left enabled so USB can still enumerate if fault is post-USB-init */
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
