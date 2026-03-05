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
  MX_USB_DEVICE_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  /* Configure PD7 as push-pull output for debug toggling */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  HAL_Delay(1000); /* Wait for USB enumeration on host */

  /* Comprehensive SDRAM test — mask A8 alias (known bad) */
  {
    volatile uint32_t *sdram = (volatile uint32_t *)0xC0000000UL;
    /* A8 stuck causes bit 7 errors in pattern sweep — mask both half-words */
    const uint32_t A8_MASK = ~0x00800080UL;
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
      uint32_t diff = (pat ^ got) & A8_MASK;
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

    /* === Test 2: Address bus — power-of-2 offsets, full 16MB range ===
     * FMC mapping for 16-bit bus, 9-col, 12-row, 4-bank:
     *   byte_addr[9:1]   → Column A[8:0]
     *   byte_addr[21:10] → Row A[11:0]
     *   byte_addr[23:22] → Bank BA[1:0]
     * Word offsets that exercise each SDRAM signal:
     *   0x1..0x80     → Column A0..A8  (PF0-PF5, PF12-PF15)
     *   0x100..0x80000 → Row A0..A11   (PF0-PF5, PF12-PF15, PG0-PG1)
     *   0x100000       → BA0           (PG4)
     *   0x200000       → BA1           (PG5)
     */
    CDC_Transmit_FS((uint8_t *)"TEST2: Address bus (16MB range)\r\n", 32);
    HAL_Delay(5);
    ok = 1;
    const uint32_t addrTestMax = 4UL * 1024UL * 1024UL; /* 4M words = 16MB */

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
      uint32_t diff = (got ^ 0xAAAAAAAAUL) & A8_MASK;
      if (diff) {
        snprintf(msg, sizeof(msg), "  Addr 0 alias! got %08lX (A8 known bad)\r\n",
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
      uint32_t diff = (expected ^ got) & A8_MASK;

      /* Identify which SDRAM signal this offset exercises */
      const char *sig = "?";
      uint32_t byteOff = offset * 4;
      if      (byteOff == 0x00000004UL) sig = "ColA1";
      else if (byteOff == 0x00000008UL) sig = "ColA2";
      else if (byteOff == 0x00000010UL) sig = "ColA3";
      else if (byteOff == 0x00000020UL) sig = "ColA4";
      else if (byteOff == 0x00000040UL) sig = "ColA5";
      else if (byteOff == 0x00000080UL) sig = "ColA6";
      else if (byteOff == 0x00000100UL) sig = "ColA7";
      else if (byteOff == 0x00000200UL) sig = "ColA8";
      else if (byteOff == 0x00000400UL) sig = "RowA0";
      else if (byteOff == 0x00000800UL) sig = "RowA1";
      else if (byteOff == 0x00001000UL) sig = "RowA2";
      else if (byteOff == 0x00002000UL) sig = "RowA3";
      else if (byteOff == 0x00004000UL) sig = "RowA4";
      else if (byteOff == 0x00008000UL) sig = "RowA5";
      else if (byteOff == 0x00010000UL) sig = "RowA6";
      else if (byteOff == 0x00020000UL) sig = "RowA7";
      else if (byteOff == 0x00040000UL) sig = "RowA8";
      else if (byteOff == 0x00080000UL) sig = "RowA9";
      else if (byteOff == 0x00100000UL) sig = "RowA10";
      else if (byteOff == 0x00200000UL) sig = "RowA11";
      else if (byteOff == 0x00400000UL) sig = "BA0";
      else if (byteOff == 0x00800000UL) sig = "BA1";

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
      uint32_t diff = (expected ^ got) & A8_MASK;
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

    /* === Summary === */
    if (errors == 0 && ok)
      CDC_Transmit_FS((uint8_t *)"SDRAM: ALL PASS (A8 known bad)\r\n", 32);
    else
      CDC_Transmit_FS((uint8_t *)"SDRAM: ERRORS found — see above\r\n", 33);
  }

  /* ETH diagnostics: report init result and read PHY ID registers */
  {
    char emsg[64];
    if (heth.gState == HAL_ETH_STATE_READY) {
      CDC_Transmit_FS((uint8_t *)"ETH: init OK\r\n", 14);
      HAL_Delay(10);
      uint32_t regval = 0;
      /* PHY ID1 should be 0x0007, ID2 = 0xC130 for LAN8742A */
      /* LAN8742A PHY address set by PHYAD[2:0] strapping — default 0x00 */
      #define PHY_ADDR  0x00U
      HAL_ETH_ReadPHYRegister(&heth, PHY_ADDR, 2, &regval);
      snprintf(emsg, sizeof(emsg), "ETH: PHY ID1=0x%04lX\r\n", regval);
      CDC_Transmit_FS((uint8_t *)emsg, (uint16_t)strlen(emsg));
      HAL_Delay(10);
      HAL_ETH_ReadPHYRegister(&heth, PHY_ADDR, 3, &regval);
      snprintf(emsg, sizeof(emsg), "ETH: PHY ID2=0x%04lX\r\n", regval);
      CDC_Transmit_FS((uint8_t *)emsg, (uint16_t)strlen(emsg));
      HAL_Delay(10);
      HAL_ETH_ReadPHYRegister(&heth, PHY_ADDR, 1, &regval);
      snprintf(emsg, sizeof(emsg), "ETH: BSR=0x%04lX (%s)\r\n", regval,
               (regval & 0x0004) ? "link up" : "no link");
      CDC_Transmit_FS((uint8_t *)emsg, (uint16_t)strlen(emsg));
    } else {
      snprintf(emsg, sizeof(emsg), "ETH: init FAIL state=%lu\r\n", (uint32_t)heth.gState);
      CDC_Transmit_FS((uint8_t *)emsg, (uint16_t)strlen(emsg));
    }
    HAL_Delay(10);
  }

  /* Start backlight PWM on TIM4 CH2 (PD13 -> TPS61043 CTRL pin)
   * TIM4 clk = 108 MHz, Period = 65535 -> ~1648 Hz PWM
   * 50% duty cycle for initial test */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 32768);

  /* FATFS: mount and list root directory */
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
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_7);
    CDC_Transmit_FS((uint8_t *)"Hello World!\r\n", 14);
    HAL_Delay(1000);
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
  return; /* TEMP DIAG: skip ETH init to check if it blocks USB */
  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */
  /* Release LAN8742A nRST: PA1 (RMII_REF_CLK) connects via R48 to nRST; R47 pulls to 3.3V.
   * PHY REFCLKO is LOW during reset, overriding R47 and holding nRST LOW.
   * Drive PA1 HIGH briefly to break the loop, then HAL_ETH_MspInit (called inside
   * HAL_ETH_Init below) immediately reconfigures PA1 as AF11, ending contention. */
  {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef geth = {0};
    geth.Pin   = GPIO_PIN_1;
    geth.Mode  = GPIO_MODE_OUTPUT_PP;
    geth.Pull  = GPIO_NOPULL;
    geth.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &geth);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_Delay(10); /* 10ms: >>100us nRST min, crystal startup ~1-5ms */
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
  hsd1.Init.ClockDiv = 0;
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
  hspi6.Init.Direction = SPI_DIRECTION_2LINES;
  hspi6.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi6.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi6.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi6.Init.NSS = SPI_NSS_SOFT;
  hspi6.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi6.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi6.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi6.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi6.Init.CRCPolynomial = 7;
  hspi6.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi6.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
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
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_9;
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
