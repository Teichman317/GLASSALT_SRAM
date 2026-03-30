# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

GLASSALT_SRAM is an STM32F767ZIT6 (Cortex-M7, LQFP144) embedded firmware project generated and maintained with STM32CubeMX 6.16.1 and built with STM32CubeIDE using GCC. The HAL library version is STM32Cube FW_F7 V1.17.4.

## Build & Flash

This project uses the STM32CubeIDE (Eclipse CDT) build system — there is no standalone Makefile. Build and flash operations are performed from within STM32CubeIDE:

- **Build**: Project > Build Project (or Ctrl+B)
- **Flash/Debug**: Run > Debug (F11) via ST-Link SWD (PA13=SWDIO, PA14=SWCLK)
- **Linker scripts**: `STM32F767ZITX_FLASH.ld` (normal operation) and `STM32F767ZITX_RAM.ld` (RAM execution)

Internal memory: 2 MB Flash @ `0x08000000`, 512 KB RAM @ `0x20000000`. Stack = 1 KB, Heap = 512 B.

## STM32CubeMX Code Generation Rules

**Critical**: STM32CubeMX regenerates `Core/Src/main.c`, `Core/Src/stm32f7xx_hal_msp.c`, and other generated files. All user code **must** be placed inside the designated comment markers:

```c
/* USER CODE BEGIN <tag> */
  // your code here
/* USER CODE END <tag> */
```

Code placed outside these markers will be **overwritten** the next time CubeMX regenerates. The `.ioc` file (`GLASSALT_SRAM.ioc`) is the CubeMX project source — edit peripheral configuration there, not in the generated C files.

## Architecture

### Peripheral Configuration (`Core/Src/main.c`)

The `main()` function initializes peripherals in this order:
1. `MPU_Config()` — configures MPU: Region 0 covers 4 GB with no-access (SubRegionDisable=0x87 enables only selected sub-regions)
2. `HAL_Init()` / `SystemClock_Config()` — HSE 8 MHz → PLL → 216 MHz SYSCLK (overdrive)
3. Peripheral inits: GPIO → DMA → FMC (SDRAM) → ETH → SDMMC1 → DMA2D → QUADSPI → LTDC → TIM2 → TIM4 → I2C4 → SPI6 → USB_OTG_FS

### MSP Layer (`Core/Src/stm32f7xx_hal_msp.c`)

GPIO alternate function mappings and DMA linkage for each peripheral live here. When adding a new peripheral, CubeMX generates the corresponding `HAL_XXX_MspInit` / `HAL_XXX_MspDeInit` callbacks here.

### Peripheral Summary

| Peripheral | Purpose | Key Pins |
|---|---|---|
| FMC/SDRAM | External 16-bit SDRAM, Bank 1, 12-bit addr, 4 banks | PF0–PF5,PF11–PF15, PG0,PG1,PG4,PG5,PG8,PG15, PD0,PD1,PD8–PD10,PD14,PD15, PE0,PE1,PE7–PE15, PC0,PC2,PC3 |
| LTDC | ST7701S 480×480 round LCD, 2 layers (L0: RGB565 background, L1: ARGB4444 overlay) | ~24 LTDC GPIO pins |
| DMA2D | GPU for framebuffer blits (ARGB4444) | — |
| QUADSPI | External quad-SPI flash, single bank | PB2(CLK), PB6(NCS), PE2,PF6,PF8,PF9(IO0–3) |
| SDMMC1 | SD card, 4-bit bus, DMA2 Stream3/6 | PC8–PC12, PD2 |
| ETH | Ethernet RMII | PA1,PA2,PA7, PC1,PC4,PC5, PB12,PB13, PG11 |
| TIM2 | Quadrature encoder interface (CH1+CH2) | PA0(CH1/ETR), PB3(CH2) |
| TIM4 | PWM output CH2 (likely backlight) | PD13 |
| I2C4 | Touch panel (likely FT-series) | PD12(SCL), PB7(SDA) |
| SPI6 | TX-only simplex master @ 8 Mbit/s | PG13(SCK), PG14(MOSI), PA15(CS) |
| USB_OTG_FS | USB device only | PA11(DM), PA12(DP) |

### GPIO Signals

| Label | Pin | Direction | Init State | Purpose |
|---|---|---|---|---|
| `SW_RESET_PE3` | PE3 | Input, pull-up | — | Reset button |
| `SW_CNTR_PE4` | PE4 | Input, pull-up | — | Counter/encoder button |
| `SW_TEST_PE5` | PE5 | Input, pull-up | — | Test button |
| `TP_RESET_PD11` | PD11 | Output | HIGH | Touch panel reset |
| `PG2_LDC_RESET` | PG2 | Output | LOW | LCD controller reset |
| `TP_INT_PG3` | PG3 | EXTI falling edge | — | Touch panel interrupt |
| `PA15_SPI2_CS` | PA15 | Output | LOW | SPI chip select |

### Clock Tree

- Source: HSE 8 MHz (external crystal)
- PLL: M=8, N=432, P=2 → **SYSCLK = 216 MHz** (overdrive enabled)
- APB1 = SYSCLK/4 = 54 MHz, APB2 = SYSCLK/2 = 108 MHz
- PLLSAI: N=192, R=4, DivR=2 → **LTDC pixel clock = 24 MHz**
- LTDC total frame: 520×512 = 266,240 clocks → **~90 Hz actual refresh**
- SDMMC1 clocked from 48 MHz (PLL48CLK: PLLQ=9 → 432/9 = 48 MHz)

#### 120 Hz Upgrade Plan (target: new custom boards, ~2026-04-13)

Customer requirement: VR headset passthrough cameras see retrace/tearing artifacts on physical 3" round ST7701S LCD panels. Fix: raise refresh to 120 Hz to eliminate beat frequency with 90-120 Hz headset cameras.

**Change:** PLLSAIR 4→3 → pixel clock 32 MHz → 32M/(520×512) = **~120 Hz**. One-line change in `stm32f7xx_hal_msp.c` (HAL_LTDC_MspInit). Bandwidth OK: ~128 MB/s LTDC reads vs ~400 MB/s SDRAM capacity at 216 MHz. See detailed notes in the source file.

### Project Files

- `GLASSALT_SRAM.ioc` — CubeMX configuration (source of truth for peripheral setup)
- `GLASSALT_SRAM.pdf` — Hardware schematic
- `GLASSALT_SRAM net lables from cubemx.txt` — Net label export from CubeMX (pin-to-signal reference)
- `Core/Inc/main.h` — GPIO pin/port macro definitions (`SW_RESET_PE3_Pin`, etc.)
