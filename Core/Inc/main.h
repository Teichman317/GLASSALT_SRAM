/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SW_RESET_PE3_Pin GPIO_PIN_3
#define SW_RESET_PE3_GPIO_Port GPIOE
#define SW_CNTR_PE4_Pin GPIO_PIN_4
#define SW_CNTR_PE4_GPIO_Port GPIOE
#define SW_TEST_PE5_Pin GPIO_PIN_5
#define SW_TEST_PE5_GPIO_Port GPIOE
#define PB15_ENC_SW_Pin GPIO_PIN_15
#define PB15_ENC_SW_GPIO_Port GPIOB
#define TP_RESET_PD11_Pin GPIO_PIN_11
#define TP_RESET_PD11_GPIO_Port GPIOD
#define PG2_LDC_RESET_Pin GPIO_PIN_2
#define PG2_LDC_RESET_GPIO_Port GPIOG
#define TP_INT_PG3_Pin GPIO_PIN_3
#define TP_INT_PG3_GPIO_Port GPIOG
#define PA15_SPI2_CS_Pin GPIO_PIN_15
#define PA15_SPI2_CS_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
