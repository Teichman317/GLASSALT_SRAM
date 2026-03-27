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
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PAGE_SIZE   256
#define BLOCK_SIZE  (64 * 1024)
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

/* Pointer image in SDRAM (after main framebuffer) */
#define PTR_WIDTH   63
#define PTR_HEIGHT  240
#define PTR_BPP     2
#define PTR_BYTES   (PTR_WIDTH * PTR_HEIGHT * PTR_BPP)  /* 30720 */
uint8_t *pointer_buffer = (uint8_t *)(0xC0000000UL + 480*480*2);  /* right after image_buffer */

/* Rotated pointer overlay — full display size so pointer can point any direction */
#define OVL_WIDTH   480
#define OVL_HEIGHT  480
#define OVL_BYTES   (OVL_WIDTH * OVL_HEIGHT * 2)
uint8_t *overlay_buf0 = (uint8_t *)(0xC0000000UL + 480*480*2 + PTR_WIDTH*PTR_HEIGHT*2);
uint8_t *overlay_buf1 = (uint8_t *)(0xC0000000UL + 480*480*2 + PTR_WIDTH*PTR_HEIGHT*2 + 480*480*2);
uint8_t *overlay_front;  /* buffer LTDC is reading */
uint8_t *overlay_back;   /* buffer CPU is rendering into */

/* 100s counter drum strip in SDRAM (after overlay buffers) */
#define STRIP_WIDTH   28
#define STRIP_HEIGHT  567
#define STRIP_BPP     2
#define STRIP_BYTES   (STRIP_WIDTH * STRIP_HEIGHT * STRIP_BPP)
#define STRIP_DIGITS  10
#define DIGIT_HEIGHT  ((float)STRIP_HEIGHT / STRIP_DIGITS)  /* 56.7 px */
uint8_t *strip_buffer = (uint8_t *)(0xC0000000UL + 480*480*2 + PTR_WIDTH*PTR_HEIGHT*2 + 480*480*2*2);

/* Counter window position in the 480x480 background (mirrored image!) */
#define DRUM_X        288   /* left edge — mirrored side */
#define DRUM_Y        174   /* top edge — expanded vertically */
#define DRUM_VIS_H    104   /* visible height of counter aperture (2x) */

/* Pivot point within pointer source image */
#define PIVOT_X  32
#define PIVOT_Y  50

/* LTDC Layer 2 register block (layer index 1) */
#define L2  ((LTDC_Layer_TypeDef *)LTDC_Layer2_BASE)
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
static int  LoadImageFromSD(const char *filename, uint8_t *dest, uint32_t size);
static void fill_checkerboard_8bit(uint8_t *buf);
static void rotate_pointer(float angle_rad);
static void update_drum(float value);
void QSPI_MemoryMapped_Read6B(void);
void QSPI_MemoryMapped_ReadEB(void);
void QSPI_MemoryMapped_Read0B(void);
void QSPI_ExitXIP(void);
uint8_t QSPI_ReadSR1(uint8_t *sr1);
uint8_t QSPI_WaitForReady(uint32_t timeout);
uint8_t QSPI_ReadStatus(void);
void QSPI_DisableQPI(void);
uint8_t QSPI_EraseBlock64K_Diag(uint32_t offset);
uint8_t QSPI_CheckErased(uint32_t addr);
void QSPI_Reset(void);
void QSPI_MemoryMapped_DeActivate(void);
void QSPI_Enter4ByteAddrMode(void);
void QSPI_Exit4ByteAddrMode(void);
int WriteImageToFlash(uint32_t flash_addr);
uint8_t QSPI_WritePage(uint8_t* data, uint32_t addr, uint32_t len);
uint8_t QSPI_Read(uint8_t* buf, uint32_t addr, uint32_t len);
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

    /* Region 3: QSPI memory-mapped 0x90000000, 8MB, read-only, cacheable */
    r.Number             = MPU_REGION_NUMBER3;
    r.BaseAddress        = 0x90000000;
    r.Size               = MPU_REGION_SIZE_8MB;
    r.AccessPermission   = MPU_REGION_FULL_ACCESS;
    r.DisableExec        = MPU_INSTRUCTION_ACCESS_DISABLE;
    r.IsShareable        = MPU_ACCESS_NOT_SHAREABLE;
    r.IsCacheable        = MPU_ACCESS_CACHEABLE;
    r.IsBufferable       = MPU_ACCESS_NOT_BUFFERABLE; /* write-through */
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

  /* ===== QSPI NOR Flash Test ===== */
  {
    char qmsg[120];

    CDC_Transmit_FS((uint8_t *)"QSPI: Starting NOR flash test...\r\n", 34);
    HAL_Delay(10);

    /* Step 1: Reset flash and check it responds */
    QSPI_Reset();
    HAL_Delay(10);
    QSPI_DisableQPI();  /* ensure SPI mode */

    /* Read JEDEC ID (0x9F) */
    {
      uint8_t id[3] = {0};
      QSPI_CommandTypeDef cmd = {0};
      cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
      cmd.Instruction     = 0x9F;
      cmd.DataMode        = QSPI_DATA_1_LINE;
      cmd.NbData          = 3;
      if (HAL_QSPI_Command(&hqspi, &cmd, HAL_MAX_DELAY) == HAL_OK) {
        HAL_QSPI_Receive(&hqspi, id, HAL_MAX_DELAY);
      }
      snprintf(qmsg, sizeof(qmsg), "QSPI: JEDEC ID = %02X %02X %02X (expect 01 60 17 for S25FL064L)\r\n",
               id[0], id[1], id[2]);
      CDC_Transmit_FS((uint8_t *)qmsg, (uint16_t)strlen(qmsg));
      HAL_Delay(10);

      if (id[0] == 0x00 || id[0] == 0xFF) {
        CDC_Transmit_FS((uint8_t *)"QSPI: No response from flash — check wiring\r\n", 46);
        HAL_Delay(10);
        goto qspi_test_done;
      }
    }

    /* Step 2: Load image.bin from SD into SDRAM */
    CDC_Transmit_FS((uint8_t *)"QSPI: Loading image.bin from SD...\r\n", 36);
    HAL_Delay(10);
    if (LoadImageFromSD("image.bin", image_buffer, TOTAL_BYTES) != 0) {
      CDC_Transmit_FS((uint8_t *)"QSPI: SD load failed — skipping flash test\r\n", 45);
      HAL_Delay(10);
      goto qspi_test_done;
    }

    /* Print first 20 bytes from SDRAM (source) */
    {
      snprintf(qmsg, sizeof(qmsg), "QSPI: SD first 20:");
      int pos = strlen(qmsg);
      for (int i = 0; i < 20; i++) {
        pos += snprintf(qmsg + pos, sizeof(qmsg) - pos, " %02X", image_buffer[i]);
      }
      pos += snprintf(qmsg + pos, sizeof(qmsg) - pos, "\r\n");
      CDC_Transmit_FS((uint8_t *)qmsg, (uint16_t)pos);
      HAL_Delay(10);

      snprintf(qmsg, sizeof(qmsg), "QSPI: SD last 20: ");
      pos = strlen(qmsg);
      for (int i = TOTAL_BYTES - 20; i < (int)TOTAL_BYTES; i++) {
        pos += snprintf(qmsg + pos, sizeof(qmsg) - pos, " %02X", image_buffer[i]);
      }
      pos += snprintf(qmsg + pos, sizeof(qmsg) - pos, "\r\n");
      CDC_Transmit_FS((uint8_t *)qmsg, (uint16_t)pos);
      HAL_Delay(10);
    }

    /* Step 3: Write full image to flash */
    CDC_Transmit_FS((uint8_t *)"QSPI: Writing image to flash...\r\n", 33);
    HAL_Delay(10);
    if (WriteImageToFlash(0) != 0) {
      CDC_Transmit_FS((uint8_t *)"QSPI: Flash write FAILED\r\n", 26);
      HAL_Delay(10);
      goto qspi_test_done;
    }
    CDC_Transmit_FS((uint8_t *)"QSPI: Write complete\r\n", 22);
    HAL_Delay(10);

    /* Step 4: Read back via memory-mapped mode and verify */
    {
      QSPI_MemoryMapped_Read0B();
      SCB_InvalidateDCache();

      volatile uint8_t *flash = (volatile uint8_t *)0x90000000UL;
      int pos;

      /* First 20 bytes */
      snprintf(qmsg, sizeof(qmsg), "QSPI: Flash first 20:");
      pos = strlen(qmsg);
      for (int i = 0; i < 20; i++)
        pos += snprintf(qmsg + pos, sizeof(qmsg) - pos, " %02X", flash[i]);
      pos += snprintf(qmsg + pos, sizeof(qmsg) - pos, "\r\n");
      CDC_Transmit_FS((uint8_t *)qmsg, (uint16_t)pos);
      HAL_Delay(10);

      /* Spot-check at 25%, 50%, 75%, and end */
      {
        const uint32_t checkpoints[] = {
          TOTAL_BYTES / 4,
          TOTAL_BYTES / 2,
          (TOTAL_BYTES * 3) / 4,
          TOTAL_BYTES - 20
        };
        const char *labels[] = { "25%", "50%", "75%", "end" };

        for (int c = 0; c < 4; c++) {
          uint32_t off = checkpoints[c] & ~1U;
          snprintf(qmsg, sizeof(qmsg), "QSPI: @%s (0x%05lX):", labels[c], (unsigned long)off);
          pos = strlen(qmsg);
          for (int i = 0; i < 20; i++)
            pos += snprintf(qmsg + pos, sizeof(qmsg) - pos, " %02X", flash[off + i]);
          pos += snprintf(qmsg + pos, sizeof(qmsg) - pos, "\r\n");
          CDC_Transmit_FS((uint8_t *)qmsg, (uint16_t)pos);
          HAL_Delay(10);
        }
      }

      /* Quick byte-level verify: count mismatches against byte-swapped source */
      {
        uint32_t errors = 0;
        uint32_t first_err_addr = 0;
        for (uint32_t i = 0; i < TOTAL_BYTES; i += 2) {
          uint8_t exp_lo = image_buffer[i + 1]; /* swapped */
          uint8_t exp_hi = image_buffer[i];
          if (flash[i] != exp_lo || flash[i + 1] != exp_hi) {
            if (errors == 0) first_err_addr = i;
            errors++;
          }
        }
        snprintf(qmsg, sizeof(qmsg), "QSPI: Verify: %lu pixel errors", (unsigned long)errors);
        if (errors > 0) {
          int len = strlen(qmsg);
          snprintf(qmsg + len, sizeof(qmsg) - len, ", first @0x%05lX", (unsigned long)first_err_addr);
        }
        strncat(qmsg, "\r\n", sizeof(qmsg) - strlen(qmsg) - 1);
        CDC_Transmit_FS((uint8_t *)qmsg, (uint16_t)strlen(qmsg));
        HAL_Delay(10);
      }

      QSPI_MemoryMapped_DeActivate();
    }

    CDC_Transmit_FS((uint8_t *)"QSPI: Test complete\r\n", 21);
    HAL_Delay(10);
  }
  qspi_test_done:

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

  /* Try loading image from SD card, fall back to checkerboard */
  if (LoadImageFromSD_Mirrored("image.bin") != 0) {
    CDC_Transmit_FS((uint8_t *)"LCD: SD failed, using checkerboard\r\n", 36);
    HAL_Delay(10);
    fill_checkerboard_8bit(image_buffer);
  }

  /* Check pointer.bin file size and load */
  {
    static FATFS fs;
    FIL fp;
    char pmsg[80];
    hsd1.State = HAL_SD_STATE_RESET;
    HAL_SD_Init(&hsd1);
    HAL_SD_ConfigWideBusOperation(&hsd1, SDMMC_BUS_WIDE_4B);
    if (f_mount(&fs, "", 1) == FR_OK) {
      if (f_open(&fp, "pointer.bin", FA_READ) == FR_OK) {
        uint32_t fsize = f_size(&fp);
        snprintf(pmsg, sizeof(pmsg), "PTR: file size = %lu bytes\r\n", (unsigned long)fsize);
        CDC_Transmit_FS((uint8_t *)pmsg, (uint16_t)strlen(pmsg));
        HAL_Delay(10);
        f_close(&fp);
      }
      f_mount(NULL, "", 0);
    }
  }
  if (LoadImageFromSD("pointer.bin", pointer_buffer, PTR_BYTES) != 0) {
    CDC_Transmit_FS((uint8_t *)"PTR: load failed\r\n", 18);
    HAL_Delay(10);
  } else {
    /* ARGB4444 — no byte-swap needed */
    /* Init double-buffer pointers and render initial frame */
    overlay_front = overlay_buf0;
    overlay_back  = overlay_buf1;
    rotate_pointer(0.0f);
  }

  /* Load 100s counter drum strip */
  if (LoadImageFromSD("wheel100.bin", strip_buffer, STRIP_BYTES) != 0) {
    CDC_Transmit_FS((uint8_t *)"STRIP: load failed\r\n", 20);
    HAL_Delay(10);
  } else {
    CDC_Transmit_FS((uint8_t *)"STRIP: loaded OK\r\n", 18);
    HAL_Delay(10);
    /* Verify strip data in SDRAM */
    {
      char vmsg[80];
      uint16_t *sp = (uint16_t *)strip_buffer;
      SCB_InvalidateDCache_by_Addr((uint32_t *)strip_buffer, 256);
      snprintf(vmsg, sizeof(vmsg), "STRIP[0]=%04X [1]=%04X [28]=%04X [280]=%04X\r\n",
               sp[0], sp[1], sp[28], sp[280]);
      CDC_Transmit_FS((uint8_t *)vmsg, (uint16_t)strlen(vmsg));
      HAL_Delay(10);
    }
    /* Render initial drum position */
    update_drum(0.0f);
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
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 60000); //32768

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
    /* Continuous slow rotation with synchronized drum counter */
    {
      static float angle = 0.0f;
      static uint32_t last_frame = 0;
      if (HAL_GetTick() - last_frame >= 25) {  /* 25 ms = 40 fps */
        angle += 0.0174533f;  /* +1° per frame → same rotation speed, 2x smoother */
        if (angle >= 6.2831853f) angle -= 6.2831853f;
        rotate_pointer(angle);

        /* Map pointer angle to 0-9.999 drum value.
           angle=0 → 12 o'clock → digit 0, clockwise positive */
        float drum_val = 10.0f - (angle / 6.2831853f * 10.0f) + 5.0f;
        while (drum_val >= 10.0f) drum_val -= 10.0f;
        update_drum(drum_val);

        last_frame = HAL_GetTick();
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
  hqspi.Init.ClockPrescaler = 1;
  hqspi.Init.FifoThreshold = 4;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.FlashSize = 23;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_2_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;
  hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */
  hqspi.Init.FlashSize = 23;
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
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_1;
  /* SdramTiming — IS42S32400J-6TL @ 108 MHz (9.26 ns/clk)
   * tMRD = 2 clk, tXSR = 72ns→8clk, tRAS = 42ns→5clk,
   * tRC  = 60ns→7clk, tWR = 2clk, tRP = 18ns→2clk, tRCD = 18ns→2clk */
  SdramTiming.LoadToActiveDelay = 2;     /* tMRD */
  SdramTiming.ExitSelfRefreshDelay = 8;  /* tXSR */
  SdramTiming.SelfRefreshTime = 5;       /* tRAS */
  SdramTiming.RowCycleDelay = 7;         /* tRC  */
  SdramTiming.WriteRecoveryTime = 2;     /* tWR  */
  SdramTiming.RPDelay = 2;              /* tRP  */
  SdramTiming.RCDDelay = 2;             /* tRCD — was 16! */

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

    /* --- Layer 0: background image (480x480 RGB565) --- */
    __HAL_LTDC_LAYER_DISABLE(&hltdc, 1);

    pLayerCfg.WindowX0 = 0;
    pLayerCfg.WindowX1 = 480;
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

    /* --- Layer 1: rotated pointer overlay (full screen, ARGB4444 with alpha) --- */
    {
        LTDC_LayerCfgTypeDef ptrCfg = {0};
        ptrCfg.WindowX0 = 0;
        ptrCfg.WindowX1 = 480;
        ptrCfg.WindowY0 = 0;
        ptrCfg.WindowY1 = 480;
        ptrCfg.PixelFormat = LTDC_PIXEL_FORMAT_ARGB4444;
        ptrCfg.Alpha = 255;
        ptrCfg.Alpha0 = 0;
        ptrCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
        ptrCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
        ptrCfg.FBStartAdress = (uint32_t)overlay_front;
        ptrCfg.ImageWidth = OVL_WIDTH;
        ptrCfg.ImageHeight = OVL_HEIGHT;
        ptrCfg.Backcolor.Blue = 0;
        ptrCfg.Backcolor.Green = 0;
        ptrCfg.Backcolor.Red = 0;
        if (HAL_LTDC_ConfigLayer(&hltdc, &ptrCfg, 1) != HAL_OK)
        {
            Error_Handler();
        }
        /* No color keying — alpha channel handles transparency */
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

    /* Re-init SD peripheral — MX_SDMMC1_SD_Init only sets struct fields */
    hsd1.State = HAL_SD_STATE_RESET;
    HAL_StatusTypeDef sd_rc = HAL_SD_Init(&hsd1);
    if (sd_rc != HAL_OK) {
        snprintf(msg, sizeof(msg), "LCD: SD init rc=%d err=0x%lX state=%d\r\n",
                 (int)sd_rc, (unsigned long)hsd1.ErrorCode, (int)hsd1.State);
        CDC_Transmit_FS((uint8_t *)msg, (uint16_t)strlen(msg));
        HAL_Delay(10);
        /* Retry once after a delay — card may need more power-up time */
        HAL_Delay(500);
        hsd1.State = HAL_SD_STATE_RESET;
        sd_rc = HAL_SD_Init(&hsd1);
        if (sd_rc != HAL_OK) {
            snprintf(msg, sizeof(msg), "LCD: SD retry rc=%d err=0x%lX\r\n",
                     (int)sd_rc, (unsigned long)hsd1.ErrorCode);
            CDC_Transmit_FS((uint8_t *)msg, (uint16_t)strlen(msg));
            HAL_Delay(10);
            return -1;
        }
    }
    /* Widen to 4-bit bus after init */
    HAL_SD_ConfigWideBusOperation(&hsd1, SDMMC_BUS_WIDE_4B);

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

    /* Read in chunks through internal RAM to avoid DMA-to-SDRAM issues */
    {
        static uint8_t chunk[4096] __attribute__((aligned(4)));
        uint32_t remaining = TOTAL_BYTES;
        uint32_t offset = 0;
        while (remaining > 0) {
            uint32_t to_read = (remaining > sizeof(chunk)) ? sizeof(chunk) : remaining;
            fr = f_read(&file, chunk, to_read, &bytes_read);
            if (fr != FR_OK || bytes_read == 0) {
                snprintf(msg, sizeof(msg), "LCD: read fail %d at %lu got %lu\r\n",
                         (int)fr, (unsigned long)offset, (unsigned long)bytes_read);
                CDC_Transmit_FS((uint8_t *)msg, (uint16_t)strlen(msg));
                HAL_Delay(10);
                f_close(&file);
                f_mount(NULL, "", 0);
                return -1;
            }
            memcpy(&image_buffer[offset], chunk, bytes_read);
            offset += bytes_read;
            remaining -= bytes_read;
        }
    }
    f_close(&file);
    f_mount(NULL, "", 0);

    snprintf(msg, sizeof(msg), "LCD: read OK (%lu bytes)\r\n", (unsigned long)TOTAL_BYTES);
    CDC_Transmit_FS((uint8_t *)msg, (uint16_t)strlen(msg));
    HAL_Delay(10);

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

/* ===== Checkerboard test pattern (RGB565, 40-pixel squares) ===== */

static void fill_checkerboard_8bit(uint8_t *buf)
{
    uint16_t *fb = (uint16_t *)buf;
    for (int y = 0; y < IMG_HEIGHT; y++) {
        for (int x = 0; x < IMG_WIDTH; x++) {
            uint16_t color = ((x / 40 + y / 40) % 2) ? 0x0000 : 0xFFFF;
            fb[y * IMG_WIDTH + x] = color;
        }
    }
}

/* ===== Generic SD card image loader (chunked read to SDRAM) ===== */

static int LoadImageFromSD(const char *filename, uint8_t *dest, uint32_t size)
{
    FRESULT fr;
    static FATFS fs;
    FIL file;
    UINT bytes_read;
    char msg[80];

    hsd1.State = HAL_SD_STATE_RESET;
    if (HAL_SD_Init(&hsd1) != HAL_OK) {
        HAL_Delay(500);
        hsd1.State = HAL_SD_STATE_RESET;
        if (HAL_SD_Init(&hsd1) != HAL_OK) {
            snprintf(msg, sizeof(msg), "SD: init fail for %s\r\n", filename);
            CDC_Transmit_FS((uint8_t *)msg, (uint16_t)strlen(msg));
            HAL_Delay(10);
            return -1;
        }
    }
    HAL_SD_ConfigWideBusOperation(&hsd1, SDMMC_BUS_WIDE_4B);

    fr = f_mount(&fs, "", 1);
    if (fr != FR_OK) {
        snprintf(msg, sizeof(msg), "SD: mount fail %d for %s\r\n", (int)fr, filename);
        CDC_Transmit_FS((uint8_t *)msg, (uint16_t)strlen(msg));
        HAL_Delay(10);
        return -1;
    }

    fr = f_open(&file, filename, FA_READ);
    if (fr != FR_OK) {
        snprintf(msg, sizeof(msg), "SD: open fail %s err %d\r\n", filename, (int)fr);
        CDC_Transmit_FS((uint8_t *)msg, (uint16_t)strlen(msg));
        HAL_Delay(10);
        f_mount(NULL, "", 0);
        return -1;
    }

    static uint8_t chunk[4096] __attribute__((aligned(4)));
    uint32_t remaining = size;
    uint32_t offset = 0;
    while (remaining > 0) {
        uint32_t to_read = (remaining > sizeof(chunk)) ? sizeof(chunk) : remaining;
        fr = f_read(&file, chunk, to_read, &bytes_read);
        if (fr != FR_OK || bytes_read == 0) {
            snprintf(msg, sizeof(msg), "SD: read fail %s at %lu\r\n", filename, (unsigned long)offset);
            CDC_Transmit_FS((uint8_t *)msg, (uint16_t)strlen(msg));
            HAL_Delay(10);
            f_close(&file);
            f_mount(NULL, "", 0);
            return -1;
        }
        memcpy(&dest[offset], chunk, bytes_read);
        offset += bytes_read;
        remaining -= bytes_read;
    }
    f_close(&file);
    f_mount(NULL, "", 0);

    snprintf(msg, sizeof(msg), "SD: loaded %s (%lu bytes)\r\n", filename, (unsigned long)size);
    CDC_Transmit_FS((uint8_t *)msg, (uint16_t)strlen(msg));
    HAL_Delay(10);
    return 0;
}

/* ===== Rotate pointer source into full-screen overlay buffer ===== */

/* Bounding box of last-rendered pointer per overlay buffer */
static int bbox_x0[2], bbox_y0[2], bbox_x1[2], bbox_y1[2];
static int bbox_init = 0;

static void rotate_pointer(float angle_rad)
{
    uint16_t *src = (uint16_t *)pointer_buffer;
    uint16_t *dst = (uint16_t *)overlay_back;
    float cosA = cosf(angle_rad);
    float sinA = sinf(angle_rad);

    /* Display center = rotation center */
    const int cx = 240;
    const int cy = 240;

    /* Which buffer index is the back buffer? */
    int bi = (overlay_back == overlay_buf0) ? 0 : 1;

    /* Clear the previous bounding box in this back buffer (CPU memset, cache-safe) */
    if (bbox_init) {
        int bx0 = bbox_x0[bi], by0 = bbox_y0[bi];
        int bw = bbox_x1[bi] - bx0 + 1;
        for (int y = by0; y <= bbox_y1[bi]; y++) {
            memset(&dst[y * OVL_WIDTH + bx0], 0, bw * 2);
        }
    } else {
        /* First call — clear both buffers fully */
        memset(overlay_buf0, 0, OVL_BYTES);
        memset(overlay_buf1, 0, OVL_BYTES);
        bbox_init = 1;
    }

    /* Compute bounding box of rotated pointer in destination space.
       Transform all 4 corners of the source rect and take min/max. */
    int corners_sx[4] = {0, PTR_WIDTH - 1, 0,             PTR_WIDTH - 1};
    int corners_sy[4] = {0, 0,             PTR_HEIGHT - 1, PTR_HEIGHT - 1};
    int nx0 = OVL_WIDTH, ny0 = OVL_HEIGHT, nx1 = 0, ny1 = 0;
    for (int c = 0; c < 4; c++) {
        float fx = (float)(corners_sx[c] - PIVOT_X);
        float fy = (float)(corners_sy[c] - PIVOT_Y);
        int dx = cx + (int)(fx * cosA - fy * sinA);
        int dy = cy + (int)(fx * sinA + fy * cosA);
        if (dx < nx0) nx0 = dx;
        if (dx > nx1) nx1 = dx;
        if (dy < ny0) ny0 = dy;
        if (dy > ny1) ny1 = dy;
    }
    /* Clamp to overlay bounds */
    if (nx0 < 0) nx0 = 0;
    if (ny0 < 0) ny0 = 0;
    if (nx1 >= OVL_WIDTH)  nx1 = OVL_WIDTH - 1;
    if (ny1 >= OVL_HEIGHT) ny1 = OVL_HEIGHT - 1;

    /* Inverse mapping: for each destination pixel in the bounding box,
       look up the source pixel. Write every pixel — opaque source or
       transparent zero — so no stale data remains. */
    for (int dy = ny0; dy <= ny1; dy++) {
        for (int dx = nx0; dx <= nx1; dx++) {
            /* Reverse-rotate destination back to source coords */
            float fx = (float)(dx - cx);
            float fy = (float)(dy - cy);
            int sx = PIVOT_X + (int)(fx * cosA + fy * sinA + 0.5f);
            int sy = PIVOT_Y + (int)(-fx * sinA + fy * cosA + 0.5f);

            uint16_t pixel = 0x0000;  /* default: transparent (alpha=0) */
            if (sx >= 0 && sx < PTR_WIDTH && sy >= 0 && sy < PTR_HEIGHT) {
                pixel = src[sy * PTR_WIDTH + sx];
            }
            dst[dy * OVL_WIDTH + dx] = pixel;
        }
    }

    /* Save bounding box for this buffer's next clear */
    bbox_x0[bi] = nx0; bbox_y0[bi] = ny0;
    bbox_x1[bi] = nx1; bbox_y1[bi] = ny1;

    /* Flush the rendered region from D-cache so LTDC sees it */
    if (nx1 >= nx0 && ny1 >= ny0) {
        SCB_CleanDCache_by_Addr((uint32_t *)&dst[ny0 * OVL_WIDTH],
                                (ny1 - ny0 + 1) * OVL_WIDTH * 2);
    }

    /* Point Layer 2 CFBAR to the newly rendered back buffer */
    L2->CFBAR = (uint32_t)overlay_back;

    /* Request reload on next vertical blanking — hardware applies the new
       framebuffer address atomically at the VSync boundary, tear-free */
    LTDC->SRCR = LTDC_SRCR_VBR;

    /* Wait until the hardware has applied the reload (VBR bit clears) */
    while (LTDC->SRCR & LTDC_SRCR_VBR) {}

    /* Swap buffer pointers */
    uint8_t *tmp = overlay_front;
    overlay_front = overlay_back;
    overlay_back = tmp;
}

/* ===== Update 100s counter drum — blit strip window into Layer 0 background ===== */

static void update_drum(float value)
{
    /* value: 0.0 to 9.999 — fractional for smooth scrolling */
    uint16_t *bg  = (uint16_t *)image_buffer;
    uint16_t *strip = (uint16_t *)strip_buffer;

    /* Calculate y-offset into the strip */
    float y_offset_f = value * DIGIT_HEIGHT;
    int y_offset = (int)y_offset_f;

    /* Center the visible window on the current digit */
    int strip_y_start = y_offset - DRUM_VIS_H / 2 + (int)(DIGIT_HEIGHT / 2);

    /* Blit visible rows from strip into background framebuffer */
    for (int row = 0; row < DRUM_VIS_H; row++) {
        int sy = strip_y_start + row;

        /* Wrap around for seamless 9→0 transition */
        while (sy < 0)            sy += STRIP_HEIGHT;
        while (sy >= STRIP_HEIGHT) sy -= STRIP_HEIGHT;

        int bg_y = DRUM_Y + row;
        if (bg_y < 0 || bg_y >= IMG_HEIGHT) continue;

        for (int col = 0; col < STRIP_WIDTH; col++) {
            int bg_x = DRUM_X + col;
            if (bg_x < 0 || bg_x >= IMG_WIDTH) continue;

            /* Mirror horizontally to match flipped background */
            bg[bg_y * IMG_WIDTH + bg_x] = strip[sy * STRIP_WIDTH + (STRIP_WIDTH - 1 - col)];
        }
    }

    /* Flush the modified background region from D-cache */
    SCB_CleanDCache_by_Addr((uint32_t *)&bg[DRUM_Y * IMG_WIDTH],
                            DRUM_VIS_H * IMG_WIDTH * 2);
}

/* ===== QSPI Exit XIP Mode ===== */

void QSPI_ExitXIP(void)
{
    QSPI_CommandTypeDef cmd = {0};

    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.Instruction     = 0xFF;   // XIP Exit command
    cmd.AddressMode     = QSPI_ADDRESS_NONE;
    cmd.DataMode        = QSPI_DATA_NONE;

    HAL_QSPI_Command(&hqspi, &cmd, HAL_QPSI_TIMEOUT_DEFAULT_VALUE);
}

/* ===== QSPI Read SR1 ===== */

uint8_t QSPI_ReadSR1(uint8_t *sr1)
{
    QSPI_CommandTypeDef cmd = {0};

    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.Instruction     = 0x05; // Read Status Register 1
    cmd.AddressMode     = QSPI_ADDRESS_NONE;
    cmd.DataMode        = QSPI_DATA_1_LINE;
    cmd.NbData          = 1;

    if (HAL_QSPI_Command(&hqspi, &cmd, HAL_MAX_DELAY) != HAL_OK)
        return 1;

    if (HAL_QSPI_Receive(&hqspi, sr1, HAL_MAX_DELAY) != HAL_OK)
        return 1;

    return 0;
}

/* ===== QSPI Wait For Ready ===== */

uint8_t QSPI_WaitForReady(uint32_t timeout)
{
    QSPI_CommandTypeDef cmd = {0};
    uint8_t sr1 = 0;
    uint32_t start = HAL_GetTick();

    while (HAL_GetTick() - start < timeout) {

        cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
        cmd.Instruction     = 0x05;  // Read SR1
        cmd.AddressMode     = QSPI_ADDRESS_NONE;
        cmd.DataMode        = QSPI_DATA_1_LINE;
        cmd.NbData          = 1;

        if (HAL_QSPI_Command(&hqspi, &cmd, 100) != HAL_OK) {
            printf("  CMD error in WaitForReady, state=%d\r\n", hqspi.State);
            return 1;
        }
        if (HAL_QSPI_Receive(&hqspi, &sr1, 100) != HAL_OK) {
            printf("  RX error in WaitForReady, state=%d\r\n", hqspi.State);
            return 1;
        }

        printf("  SR1 in WaitForReady: 0x%02X\r\n", sr1);

        if ((sr1 & 0x01) == 0) {
            printf("  Ready, exiting WaitForReady\r\n");
            return 0;
        }
    }

    printf("  TIMEOUT WAITING FOR READY, last SR1=0x%02X\r\n", sr1);
    return 1;
}

/* ===== QSPI Status & Configuration ===== */

uint8_t QSPI_ReadStatus(void)
{
    uint8_t status = 0;
    QSPI_CommandTypeDef cmd = {0};

    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.Instruction     = 0x05;  // RDSR1
    cmd.DataMode        = QSPI_DATA_1_LINE;
    cmd.NbData          = 1;

    if (HAL_QSPI_Command(&hqspi, &cmd, HAL_MAX_DELAY) == HAL_OK) {
        HAL_QSPI_Receive(&hqspi, &status, HAL_MAX_DELAY);
    } else {
        printf("QSPI_ReadStatus: Command failed\r\n");
    }

    return status;
}

void QSPI_DisableQPI(void)
{
    uint8_t cr2;
    QSPI_CommandTypeDef cmd = {0};

    // Read CR2
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.Instruction = 0x71;
    cmd.AddressMode = QSPI_ADDRESS_1_LINE;
    cmd.Address = 0x000000;  // CR2 at offset 0
    cmd.DataMode = QSPI_DATA_1_LINE;
    cmd.NbData = 1;
    HAL_QSPI_Command(&hqspi, &cmd, HAL_MAX_DELAY);
    HAL_QSPI_Receive(&hqspi, &cr2, HAL_MAX_DELAY);

    printf("CR2 before: 0x%02X\r\n", cr2);

    if (cr2 & 0x08)  // QPI bit
    {
        printf("Disabling QPI mode...\r\n");

        // WREN
        cmd.Instruction = 0x06;
        cmd.DataMode = QSPI_DATA_NONE;
        HAL_QSPI_Command(&hqspi, &cmd, HAL_MAX_DELAY);

        // Write CR2
        uint8_t write_cr2 = cr2 & ~0x08;
        cmd.Instruction = 0x72;
        cmd.Address = 0x000000;
        cmd.DataMode = QSPI_DATA_1_LINE;
        cmd.NbData = 1;
        HAL_QSPI_Command(&hqspi, &cmd, HAL_MAX_DELAY);
        HAL_QSPI_Transmit(&hqspi, &write_cr2, HAL_MAX_DELAY);

        // Wait
        while (QSPI_ReadStatus() & 0x01);
        printf("QPI Disabled\r\n");
    }
}

/* ===== QSPI 64K Block Erase with Diagnostics ===== */

uint8_t QSPI_EraseBlock64K_Diag(uint32_t offset)
{
    printf("ERASE A: entered function\r\n");
    printf("=== QSPI 64K ERASE @ 0x%06lX ===\r\n", offset);

    if (offset & 0xFFFF) {
        printf("ERROR: Offset not 64K aligned\r\n");
        return 1;
    }

    // STEP 0 - EXIT XIP MODE (CRITICAL FOR CYPRESS FL-L)
    printf("Step 0: Sending XIP Exit (FFh)...\r\n");
    QSPI_ExitXIP();
    printf("XIP Exit sent.\r\n");

    printf("Step 0.1: Forcing QSPI state to READY...\r\n");
    hqspi.State = HAL_QSPI_STATE_READY;
    printf("QSPI state now = %d\r\n", hqspi.State);

    // STEP 1 - WAIT FOR FLASH READY
    printf("Step 1: Waiting for flash ready...\r\n");
    if (QSPI_WaitForReady(20000) != 0) {
        printf("ERROR: Flash not ready before erase\r\n");
        return 1;
    }
    printf("Flash ready.\r\n");

    // STEP 2 - SEND WREN
    printf("Step 2: Sending WREN (06h)...\r\n");

    QSPI_CommandTypeDef cmd = {0};
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.Instruction     = 0x06;
    cmd.AddressMode     = QSPI_ADDRESS_NONE;
    cmd.DataMode        = QSPI_DATA_NONE;

    if (HAL_QSPI_Command(&hqspi, &cmd, 100) != HAL_OK) {
        printf("ERROR: Failed to send WREN\r\n");
        return 1;
    }
    printf("WREN sent.\r\n");

    // STEP 3 - CONFIRM WEL=1
    printf("Step 3: Checking WEL bit...\r\n");
    uint8_t sr = 0;

    if (QSPI_ReadSR1(&sr) != 0) {
        printf("ERROR: Could not read SR1\r\n");
        return 1;
    }

    printf("SR1 = 0x%02X\r\n", sr);

    if ((sr & 0x02) == 0) {
        printf("ERROR: WEL bit not set after WREN\r\n");
        return 1;
    }
    printf("WEL=1 confirmed.\r\n");

    // STEP 4 - SEND 64K ERASE
    printf("Step 4: Sending 64K erase (D8h)...\r\n");

    memset(&cmd, 0, sizeof(cmd));
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.Instruction     = 0xD8;
    cmd.AddressMode     = QSPI_ADDRESS_1_LINE;
    cmd.AddressSize     = QSPI_ADDRESS_24_BITS;
    cmd.Address         = offset;
    cmd.DataMode        = QSPI_DATA_NONE;

    if (HAL_QSPI_Command(&hqspi, &cmd, 100) != HAL_OK) {
        printf("ERROR: Failed to send erase command\r\n");
        return 1;
    }
    printf("Erase command sent.\r\n");

    // STEP 5 - WAIT FOR ERASE COMPLETE
    printf("Step 5: Waiting for erase to finish...\r\n");

    if (QSPI_WaitForReady(200000) != 0) {
        printf("ERROR: Timeout waiting for erase completion\r\n");
        return 1;
    }

    printf("Erase completed successfully.\r\n");
    return 0;
}

uint8_t QSPI_CheckErased(uint32_t addr)
{
    uint8_t buf[16];
    QSPI_CommandTypeDef cmd = {0};

    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.Instruction     = 0x6B;
    cmd.AddressMode     = QSPI_ADDRESS_1_LINE;
    cmd.Address         = addr - 0x90000000;
    cmd.AddressSize     = QSPI_ADDRESS_24_BITS;
    cmd.DataMode        = QSPI_DATA_4_LINES;
    cmd.DummyCycles     = 8;
    cmd.NbData          = 16;
    cmd.SIOOMode        = QSPI_SIOO_INST_EVERY_CMD;

    HAL_QSPI_Command(&hqspi, &cmd, HAL_MAX_DELAY);
    HAL_QSPI_Receive(&hqspi, buf, HAL_MAX_DELAY);

    printf("ERASE CHECK: ");
    for (int i = 0; i < 16; i++) printf("%02X ", buf[i]);
    printf("\r\n");

    for (int i = 0; i < 16; i++) {
        if (buf[i] != 0xFF) {
            printf("ERASE FAILED (not 0xFF)\r\n");
            return 1;
        }
    }
    printf("ERASE PASSED (all 0xFF)\r\n");
    return 0;
}

void QSPI_Reset(void)
{
    QSPI_CommandTypeDef cmd = {0};
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.Instruction     = 0xFF;  // Reset
    HAL_QSPI_Command(&hqspi, &cmd, HAL_MAX_DELAY);
    HAL_Delay(10);
    printf("HARD RESET SENT (0xFF)\r\n");
}

void QSPI_MemoryMapped_DeActivate(void)
{
    if (HAL_QSPI_Abort(&hqspi) != HAL_OK) {
        printf("QSPI abort failed\r\n");
    }

    if (QSPI_WaitForReady(1000) != 0) {
        printf("QSPI not ready after abort\r\n");
    }

    QSPI_CommandTypeDef cmd = {0};
    cmd.InstructionMode = QSPI_INSTRUCTION_NONE;
    cmd.AddressMode     = QSPI_ADDRESS_NONE;
    cmd.DataMode        = QSPI_DATA_NONE;
    cmd.DummyCycles     = 0;

    HAL_QSPI_Command(&hqspi, &cmd, HAL_MAX_DELAY);

    printf("QSPI memory-mapped mode deactivated\r\n");
}

void QSPI_Enter4ByteAddrMode(void)
{
    QSPI_CommandTypeDef cmd = {0};

    if (QSPI_WaitForReady(1000) != 0) {
        printf("Device not ready for 4-byte mode\r\n");
        return;
    }

    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.Instruction     = 0x06; // WREN
    cmd.AddressMode     = QSPI_ADDRESS_NONE;
    cmd.DataMode        = QSPI_DATA_NONE;
    if (HAL_QSPI_Command(&hqspi, &cmd, HAL_MAX_DELAY) != HAL_OK) {
        printf("WREN failed\r\n");
        return;
    }

    cmd.Instruction     = 0xB7; // Enter 4-byte address mode
    if (HAL_QSPI_Command(&hqspi, &cmd, HAL_MAX_DELAY) != HAL_OK) {
        printf("Enter 4-byte mode failed\r\n");
        return;
    }

    printf("Flash now in 4-byte address mode\r\n");
}

void QSPI_Exit4ByteAddrMode(void)
{
    QSPI_CommandTypeDef cmd = {0};

    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.Instruction     = 0xE9; // Exit 4-byte address mode
    cmd.AddressMode     = QSPI_ADDRESS_NONE;
    cmd.DataMode        = QSPI_DATA_NONE;
    HAL_QSPI_Command(&hqspi, &cmd, HAL_MAX_DELAY);

    printf("Flash back to 3-byte address mode\r\n");
}

/* ===== QSPI Page Write & Read ===== */

uint8_t QSPI_WritePage(uint8_t* data, uint32_t addr, uint32_t len)
{
  static uint32_t page_count = 0;
  page_count++;

  printf("  Writing page %lu (addr 0x%08lX, %lu bytes)...\n\r", page_count, addr, len);

  if (QSPI_WaitForReady(1000) != 0) return 1;

  QSPI_CommandTypeDef cmd = {0};

  // WREN
  cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  cmd.Instruction     = 0x06;
  cmd.AddressMode     = QSPI_ADDRESS_NONE;
  cmd.DataMode        = QSPI_DATA_NONE;
  if (HAL_QSPI_Command(&hqspi, &cmd, HAL_MAX_DELAY) != HAL_OK) return 1;

  // PAGE PROGRAM (0x02) — 1-line address + 1-line data
  cmd.Instruction = 0x02;
  cmd.AddressMode = QSPI_ADDRESS_1_LINE;
  cmd.Address     = addr;
  cmd.AddressSize = QSPI_ADDRESS_24_BITS;
  cmd.DataMode    = QSPI_DATA_1_LINE;
  cmd.NbData      = len;

  if (HAL_QSPI_Command(&hqspi, &cmd, HAL_MAX_DELAY) != HAL_OK) return 1;

  if (HAL_QSPI_Transmit(&hqspi, data, HAL_MAX_DELAY) != HAL_OK) {
    return 1;
  }

  // AFTER WRITE, CLEAR ERRORS — fresh cmd struct, instruction-only
  {
    QSPI_CommandTypeDef clr = {0};
    clr.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    clr.Instruction     = 0x30;  // CLSR
    clr.AddressMode     = QSPI_ADDRESS_NONE;
    clr.DataMode        = QSPI_DATA_NONE;
    HAL_QSPI_Command(&hqspi, &clr, HAL_MAX_DELAY);
  }

  if (QSPI_WaitForReady(10000) != 0) return 1;

  printf("  Page %lu written\n\r", page_count);
  return 0;
}

/* Normal read (0x03) - works in any mode */
uint8_t QSPI_Read(uint8_t* buf, uint32_t addr, uint32_t len)
{
    if (QSPI_WaitForReady(1000) != 0) return 1;

    QSPI_CommandTypeDef cmd = {0};
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.Instruction     = 0x03;
    cmd.AddressMode     = QSPI_ADDRESS_1_LINE;
    cmd.AddressSize     = QSPI_ADDRESS_24_BITS;
    cmd.Address         = addr - 0x90000000;
    cmd.DataMode        = QSPI_DATA_1_LINE;
    cmd.DummyCycles     = 0;
    cmd.NbData          = len;

    HAL_StatusTypeDef status = HAL_QSPI_Command(&hqspi, &cmd, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        printf("QSPI_Command failed: %d\r\n", status);
        return 1;
    }

    return HAL_QSPI_Receive(&hqspi, buf, HAL_MAX_DELAY);
}

/* ===== QSPI Write Image to Flash ===== */

int WriteImageToFlash(uint32_t flash_addr)
{
    uint32_t bytes_remaining = TOTAL_BYTES;
    uint32_t current_addr = flash_addr;
    uint32_t buffer_offset = 0;

    printf("Writing %lu bytes to flash at 0x%08lX...\r\n", TOTAL_BYTES, flash_addr);

    static uint8_t swap_buf[PAGE_SIZE];

    while (bytes_remaining > 0) {

        // === ERASE CURRENT 64KB BLOCK IF NEEDED ===
        uint32_t block_start = current_addr & ~(BLOCK_SIZE - 1);
        if ((current_addr & (BLOCK_SIZE - 1)) == 0) {
            printf("  Erasing 64KB block at 0x%08lX...\r\n", block_start);
            if (QSPI_EraseBlock64K_Diag(block_start) != 0) {
                printf("  ERASE FAILED\r\n");
                return -1;
            }
        }

        // === PREPARE ONE PAGE (256 bytes max) ===
        uint32_t page_len = (bytes_remaining > PAGE_SIZE) ? PAGE_SIZE : bytes_remaining;

        // Copy + swap each 16-bit pixel
        for (uint32_t i = 0; i < page_len; i += 2) {
            uint8_t lo = image_buffer[buffer_offset + i];
            uint8_t hi = image_buffer[buffer_offset + i + 1];
            swap_buf[i]     = hi;
            swap_buf[i + 1] = lo;
        }

        // === WRITE SWAPPED PAGE TO FLASH ===
        if (QSPI_WritePage(swap_buf, current_addr, page_len) != 0) {
            printf("  PAGE WRITE FAILED at 0x%08lX\r\n", current_addr);
            return -1;
        }

        buffer_offset += page_len;
        current_addr  += page_len;
        bytes_remaining -= page_len;
    }

    printf("IMAGE FULLY WRITTEN & MAPPED\r\n");
    return 0;
}

/* ===== QSPI Memory-Mapped Mode (0x6B Quad Output Read, 1-1-4) ===== */

void QSPI_MemoryMapped_Read6B(void)
{
    QSPI_CommandTypeDef cmd = {0};

    cmd.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    cmd.Instruction       = 0x6B;
    cmd.AddressMode       = QSPI_ADDRESS_1_LINE;
    cmd.AddressSize       = QSPI_ADDRESS_24_BITS;
    cmd.DataMode          = QSPI_DATA_4_LINES;
    cmd.DummyCycles       = 10;
    cmd.DdrMode           = QSPI_DDR_MODE_DISABLE;
    cmd.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    HAL_QSPI_Command(&hqspi, &cmd, HAL_MAX_DELAY);
    QSPI_MemoryMappedTypeDef cfg = {0};
    cfg.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE;
    HAL_QSPI_MemoryMapped(&hqspi, &cmd, &cfg);

    printf("QSPI memory-mapped mode enabled (0x6B 1-1-4)\n\r");
}

/* ===== QSPI Memory-Mapped Mode (0xEB Quad I/O Fast Read, 1-4-4) ===== */

void QSPI_MemoryMapped_ReadEB(void)
{
    QSPI_CommandTypeDef s_command = {0};
    QSPI_MemoryMappedTypeDef s_mem_mapped_cfg = {0};

    s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    s_command.Instruction       = 0xEB;

    s_command.AddressMode       = QSPI_ADDRESS_4_LINES;
    s_command.AddressSize       = QSPI_ADDRESS_24_BITS;

    s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_4_LINES;
    s_command.AlternateBytesSize= QSPI_ALTERNATE_BYTES_8_BITS;
    s_command.AlternateBytes    = 0x00;

    s_command.DummyCycles       = 6;

    s_command.DataMode          = QSPI_DATA_4_LINES;

    s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    s_mem_mapped_cfg.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE;

    HAL_QSPI_MemoryMapped(&hqspi, &s_command, &s_mem_mapped_cfg);
}

/* ===== QSPI Memory-Mapped Mode (0x0B Fast Read, 1-1-1) ===== */

void QSPI_MemoryMapped_Read0B(void)
{
    QSPI_CommandTypeDef      cmd = {0};
    QSPI_MemoryMappedTypeDef cfg = {0};

    cmd.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    cmd.Instruction       = 0x0B;

    cmd.AddressMode       = QSPI_ADDRESS_1_LINE;
    cmd.AddressSize       = QSPI_ADDRESS_24_BITS;

    cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;

    cmd.DummyCycles       = 8;

    cmd.DataMode          = QSPI_DATA_1_LINE;

    cmd.DdrMode           = QSPI_DDR_MODE_DISABLE;
    cmd.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    cfg.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE;

    if (HAL_QSPI_MemoryMapped(&hqspi, &cmd, &cfg) != HAL_OK) {
        printf("Memory-mapped 0x0B failed\r\n");
        Error_Handler();
    }

    printf("QSPI memory-mapped mode enabled (0x0B 1-1-1)\r\n");
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
