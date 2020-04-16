/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>

#include "usb_device.h"

#include "api.h"
#include "apiConfiguration.h"
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
#define LED_ONBOARD_Pin GPIO_PIN_13
#define LED_ONBOARD_GPIO_Port GPIOC
#define LIDAR_TX_Pin GPIO_PIN_2
#define LIDAR_TX_GPIO_Port GPIOA
#define LIDAR_RX_Pin GPIO_PIN_3
#define LIDAR_RX_GPIO_Port GPIOA
#define INH34_Pin GPIO_PIN_4
#define INH34_GPIO_Port GPIOA
#define INH12_Pin GPIO_PIN_5
#define INH12_GPIO_Port GPIOA
#define PWM4_Pin GPIO_PIN_6
#define PWM4_GPIO_Port GPIOA
#define PWM3_Pin GPIO_PIN_7
#define PWM3_GPIO_Port GPIOA
#define PWM2_Pin GPIO_PIN_0
#define PWM2_GPIO_Port GPIOB
#define PWM1_Pin GPIO_PIN_1
#define PWM1_GPIO_Port GPIOB
#define GPS_TX_Pin GPIO_PIN_10
#define GPS_TX_GPIO_Port GPIOB
#define GPS_RX_Pin GPIO_PIN_11
#define GPS_RX_GPIO_Port GPIOB
#define KVH_TX_Pin GPIO_PIN_9
#define KVH_TX_GPIO_Port GPIOA
#define KVH_RX_Pin GPIO_PIN_10
#define KVH_RX_GPIO_Port GPIOA
#define LIDAR_PWM_Pin GPIO_PIN_8
#define LIDAR_PWM_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
