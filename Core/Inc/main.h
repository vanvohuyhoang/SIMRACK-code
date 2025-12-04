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

/* ============================================================================
 * SIM Card HAL - Inline macros for timing-critical operations
 * ============================================================================ */

// DWT (Data Watchpoint and Trace) addresses for cycle-accurate delays
#define DWT_CONTROL_REG ((volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT_REG  ((volatile uint32_t *)0xE0001004)

// Inline delay using DWT cycle counter
static inline void SIM_HAL_DelayUs(uint32_t us) {
    uint32_t cycles = (SystemCoreClock / 1000000) * us;
    uint32_t start = *DWT_CYCCNT_REG;
    while ((*DWT_CYCCNT_REG - start) < cycles);
}

// Inline GPIO operations
static inline void SIM_HAL_WriteIO(uint8_t level) {
    if (level) {
        SIM_IO_GPIO_Port->BSRR = SIM_IO_Pin;      // Set HIGH
    } else {
        SIM_IO_GPIO_Port->BSRR = (uint32_t)SIM_IO_Pin << 16;  // Set LOW
    }
}

static inline uint8_t SIM_HAL_ReadIO(void) {
    return (SIM_IO_GPIO_Port->IDR & SIM_IO_Pin) ? 1 : 0;
}

static inline void SIM_HAL_WriteRST(uint8_t level) {
    HAL_GPIO_WritePin(SIM_RST_GPIO_Port, SIM_RST_Pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// These can remain as regular functions (not timing-critical)
extern void SIM_HAL_DelayMs(uint32_t ms);
extern void SIM_HAL_DebugPrint(const char *str);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
