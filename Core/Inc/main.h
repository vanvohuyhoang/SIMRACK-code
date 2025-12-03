/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
// SIM Card pins (ISO 7816)
#define SIM_IO_Pin GPIO_PIN_13           // PE13 - Bidirectional I/O
#define SIM_IO_GPIO_Port GPIOE
#define SIM_RST_Pin GPIO_PIN_15         // PE15 - Reset control
#define SIM_RST_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */


// TODO: Add VDD_SIM power enable pin from your schematic
// Example (uncomment and modify based on your hardware):
// #define VDD_SIM_EN_Pin GPIO_PIN_X
// #define VDD_SIM_EN_GPIO_Port GPIOX
//
// Then in SIM_Init(), add:
// HAL_GPIO_WritePin(VDD_SIM_EN_GPIO_Port, VDD_SIM_EN_Pin, GPIO_PIN_SET);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
