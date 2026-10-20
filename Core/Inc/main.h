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
#include "stm32h7xx_hal.h"

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
#define RF_DIO1_Pin GPIO_PIN_2
#define RF_DIO1_GPIO_Port GPIOE
#define RF_DIO1_EXTI_IRQn EXTI2_IRQn
#define RF_BUSY_Pin GPIO_PIN_3
#define RF_BUSY_GPIO_Port GPIOE
#define RF_RXEN_Pin GPIO_PIN_6
#define RF_RXEN_GPIO_Port GPIOE
#define LedError_Pin GPIO_PIN_5
#define LedError_GPIO_Port GPIOF
#define RF_NRST_Pin GPIO_PIN_10
#define RF_NRST_GPIO_Port GPIOF
#define LedErrorH3_Pin GPIO_PIN_3
#define LedErrorH3_GPIO_Port GPIOH
#define SPI6_NSS_Pin GPIO_PIN_4
#define SPI6_NSS_GPIO_Port GPIOA
#define GYRO_EXTL1_Pin GPIO_PIN_10
#define GYRO_EXTL1_GPIO_Port GPIOE
#define GYRO_EXTL1_EXTI_IRQn EXTI15_10_IRQn
#define SPI4_NSS_Pin GPIO_PIN_11
#define SPI4_NSS_GPIO_Port GPIOE
#define BMP_INT_Pin GPIO_PIN_11
#define BMP_INT_GPIO_Port GPIOH
#define BMP_INT_EXTI_IRQn EXTI15_10_IRQn
#define QMC_DRDY_Pin GPIO_PIN_13
#define QMC_DRDY_GPIO_Port GPIOB
#define QMC_DRDY_EXTI_IRQn EXTI15_10_IRQn
#define LED_FAIL_Pin GPIO_PIN_10
#define LED_FAIL_GPIO_Port GPIOD
#define POWERUP_Pin GPIO_PIN_15
#define POWERUP_GPIO_Port GPIOH
#define SPI2_NSS_Pin GPIO_PIN_0
#define SPI2_NSS_GPIO_Port GPIOI
#define SDMMC1_CD_Pin GPIO_PIN_7
#define SDMMC1_CD_GPIO_Port GPIOD
#define SPI1_NSS_Pin GPIO_PIN_10
#define SPI1_NSS_GPIO_Port GPIOG
#define VL53_EVENT1_Pin GPIO_PIN_8
#define VL53_EVENT1_GPIO_Port GPIOB
#define VL53_EVENT1_EXTI_IRQn EXTI9_5_IRQn
#define PMW_MOTION_Pin GPIO_PIN_9
#define PMW_MOTION_GPIO_Port GPIOB
#define PMW_MOTION_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
