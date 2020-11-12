/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>

#include "bq769x0.h"

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

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef TxHeader;

uint8_t TxData[8];
uint32_t TxMailbox;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
	char msg[256];

	TxHeader.ExtId = 0x02;
	TxHeader.IDE = CAN_ID_EXT;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 2;
	TxHeader.TransmitGlobalTime = DISABLE;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
 // MX_CAN_Init();
  /* USER CODE BEGIN 2 */


	//Configure_CAN(&hcan);

	sprintf(msg, "startup.\r\n");
	if(HAL_UART_Transmit(&huart1, (uint8_t*) msg, strlen((char*) msg),
			HAL_MAX_DELAY) != HAL_OK) {
		//Error_Handler();
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	HAL_StatusTypeDef BQ_result = HAL_BUSY;

	int idx = 0;
	uint8_t sys_stat = 0;
	HAL_StatusTypeDef result = bq769x0_reg_read_byte(&hi2c1, BQ_SYS_STAT,
			&sys_stat);

	if (result != HAL_OK) {
		sprintf(msg, "error reading sys_stat.\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t*) msg, strlen((char*) msg),
				HAL_MAX_DELAY);
	} else {
		sprintf(msg, "sys_stat: %d\r\n", sys_stat);
		HAL_UART_Transmit(&huart1, (uint8_t*) msg, strlen((char*) msg),
				HAL_MAX_DELAY);

		if (sys_stat > 0) {
			uint8_t clear = sys_stat;// & 0b00010011;
			// SCD
			HAL_StatusTypeDef result = bq769x0_reg_write_byte(&hi2c1,
					BQ_SYS_STAT, clear);
			sprintf(msg, "result: %d\r\n", result);
			HAL_UART_Transmit(&huart1, (uint8_t*) msg, strlen((char*) msg),
					HAL_MAX_DELAY);
		}
	}

	result = bq769x0_reg_read_byte(&hi2c1, BQ_SYS_STAT, &sys_stat);
	if (result != HAL_OK) {
		sprintf(msg, "error reading sys_stat.\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t*) msg, strlen((char*) msg),
				HAL_MAX_DELAY);
	} else {
		sprintf(msg, "sys_stat: %d\r\n", sys_stat);
		HAL_UART_Transmit(&huart1, (uint8_t*) msg, strlen((char*) msg),
				HAL_MAX_DELAY);
	}


	uint8_t sysctl2reg = 0;
	result = bq769x0_reg_read_byte(&hi2c1, BQ_SYS_CTRL2, &sysctl2reg);
	sprintf(msg, "result: %d, sys_ctrl2: %d.\r\n", result, sysctl2reg);
	HAL_UART_Transmit(&huart1, (uint8_t*) msg, strlen((char*) msg),
			HAL_MAX_DELAY);

	// enable DSG
	BQ_result = bq769x0_set_DSG(&hi2c1, 1);
		if (BQ_result != HAL_OK) {
			// error, couldn't talk to BQ chip??
			Error_Handler();
		}

		HAL_Delay(1000);

		sysctl2reg = 0;
		result = bq769x0_reg_read_byte(&hi2c1, BQ_SYS_CTRL2, &sysctl2reg);
		sprintf(msg, "result: %d, sys_ctrl2: %d.\r\n", result, sysctl2reg);
		HAL_UART_Transmit(&huart1, (uint8_t*) msg, strlen((char*) msg),
				HAL_MAX_DELAY);

	MX_CAN_Init();
	Configure_CAN(&hcan);


	TxData[0] = 0;


	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

		TxData[0]++;
		TxData[1] = 2;

		// Request transmisison
		if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox)
				!= HAL_OK) {
			Error_Handler();
		}



		HAL_Delay(1000);

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	char msg[20];
	sprintf(msg, "error.\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*) msg, strlen((char*) msg),
			HAL_MAX_DELAY);
	while(1) {

	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
