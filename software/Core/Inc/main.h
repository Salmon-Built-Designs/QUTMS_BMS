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
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
//#define BMS_DEBUG_HEARTBEAT
//#define BMS_DEBUG_BALANCING
//#define BMS_DISABLE_PRINT_TEMPS
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TEMP2_Pin GPIO_PIN_3
#define TEMP2_GPIO_Port GPIOA
#define TEMP_SOC_Pin GPIO_PIN_4
#define TEMP_SOC_GPIO_Port GPIOA
#define nALARM_Pin GPIO_PIN_5
#define nALARM_GPIO_Port GPIOA
#define ID0_Pin GPIO_PIN_0
#define ID0_GPIO_Port GPIOB
#define ID1_Pin GPIO_PIN_1
#define ID1_GPIO_Port GPIOB
#define ID2_Pin GPIO_PIN_2
#define ID2_GPIO_Port GPIOB
#define ID3_Pin GPIO_PIN_10
#define ID3_GPIO_Port GPIOB
#define CELL_ALERT_Pin GPIO_PIN_11
#define CELL_ALERT_GPIO_Port GPIOB
#define LED0_Pin GPIO_PIN_14
#define LED0_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_15
#define LED1_GPIO_Port GPIOB
#define TEMP1_Pin GPIO_PIN_15
#define TEMP1_GPIO_Port GPIOA
#define TEMP3_Pin GPIO_PIN_3
#define TEMP3_GPIO_Port GPIOB
#define TEMP4_Pin GPIO_PIN_4
#define TEMP4_GPIO_Port GPIOB
#define BQ_POWER_Pin GPIO_PIN_9
#define BQ_POWER_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define TICKS_SECOND 2000
#define NUM_VOLTAGE_GROUPS 3
#define NUM_VOLTAGES 10

#define NUM_TEMP_BLOCKS 2
#define NUM_TEMPS_PER_BLOCK 7

#define OVER_VOLTAGE 3700
#define UNDER_VOLTAGE 2400

// 30 seconds
#define HEARTBEAT_TIMEOUT 30000

// corresponds to 40 seconds, with voltages read at 250ms
#define NUM_BAD_VOLTAGE_COUNT 160

extern bool take_voltage_reading;
extern bool update_balancing;

#define BALANCING_GROUP_1 0b10101
#define BALANCING_GROUP_2 0b01010

// balance anything more than 100mv above the average
#define BALANCING_THRESHOLD 100


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
