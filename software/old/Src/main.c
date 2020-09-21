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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void PrintI2CStatus(uint8_t status);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define BMS_NUM_CELLS 10
static uint8_t cells[BMS_NUM_CELLS] = { 0, 1, 2, 4, 5, 6, 7, 8, 9, 10 };
static uint16_t cell_voltages[BMS_NUM_CELLS];



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	char msg[20];

	bq769x0_config_t bq769x0_config;
	bq769x0_config.ovp = 0;
	bq769x0_config.uvp = 0;

	uint8_t status = 0;


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
  /* USER CODE BEGIN 2 */
	char str[50] ={0};
	bq769x0_boot(GPIOB, GPIO_PIN_14);

	if(bq769x0_configure(hi2c1, bq769x0_config) < 0) {
		sprintf(msg, "ERROR: BQ Failed to Config.\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t*)msg,
					 strlen((char*)msg), HAL_MAX_DELAY);
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	// BQ7 main part
	if(bq769x0_read_status(hi2c1, &status) != HAL_OK) {
		sprintf(msg, "ERROR: Failed get Status\n\r");
		HAL_UART_Transmit(&huart1, (uint8_t*)msg,
					strlen(msg), HAL_MAX_DELAY);
	} else {
		sprintf(msg, "CC_Ready: %d;\n\rXReady: %d;\n\rALERT: %d\n\r",
				status<<BQ769X0_REG_STAT_CC_READY,
				status<<BQ769X0_REG_STAT_DEVICE_XREADY,
				status<<BQ769X0_REG_STAT_OVRD_ALERT);
		HAL_UART_Transmit(&huart1, (uint8_t*)msg,
						strlen(msg), HAL_MAX_DELAY);
		sprintf(msg, "UV: %d;\n\rOV: %d;\n\rSCD: %d\n\rOCD: %d\n\r",
						status<<BQ769X0_REG_STAT_UV,
						status<<BQ769X0_REG_STAT_OV,
						status<<BQ769X0_REG_STAT_SCD,
						status<<BQ769X0_REG_STAT_OCD);
				HAL_UART_Transmit(&huart1, (uint8_t*)msg,
								strlen(msg), HAL_MAX_DELAY);
		PrintI2CStatus(status);
	}
	if (bq769x0_status_error(status)) {
		sprintf(msg, "Err: Status\n\r");
		HAL_UART_Transmit(&huart1, (uint8_t*)msg,
					strlen(msg), HAL_MAX_DELAY);
	}
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		sprintf(str, "Single Loop: \n\r");
		HAL_UART_Transmit(&huart1, (uint8_t*)str,
					strlen(str), HAL_MAX_DELAY);

		// Getting Voltage values from BQ Board
		sprintf(msg, "Cells:\n\r");
		HAL_UART_Transmit(&huart1, (uint8_t*)msg,
					strlen(msg), HAL_MAX_DELAY);
		for (int i = 0; i < BMS_NUM_CELLS; i++) {
			if(bq769x0_read_voltage(hi2c1, cells[i],
					&cell_voltages[i]) != HAL_OK) {
				sprintf(msg, "Err c%d: voltage\n\r", i);
				HAL_UART_Transmit(&huart1, (uint8_t*)msg,
							strlen(msg), HAL_MAX_DELAY);
			} else {
				sprintf(msg, "V%d: %hu\n\r", i, cell_voltages[i]);
				HAL_UART_Transmit(&huart1, (uint8_t*)msg,
							strlen(msg), HAL_MAX_DELAY);
			}
		}
		sprintf(msg, "\n\r\n\r");
		HAL_UART_Transmit(&huart1, (uint8_t*)msg,
					strlen(msg), HAL_MAX_DELAY);

		HAL_Delay(3000);

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

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void PrintI2CStatus(uint8_t status){
	char msg[50];
	sprintf(msg, "\t\tStatusMessage\n\r");
	HAL_UART_Transmit(&huart1, (uint8_t*)msg,
				strlen(msg), HAL_MAX_DELAY);
	int i = 0;
	for (unsigned int mask = 0x80; mask != 0; mask >>= 1) {
		switch(i){
			case 0:
				sprintf(msg, "STAT_OCD:");
				HAL_UART_Transmit(&huart1, (uint8_t*)msg,
								strlen(msg), HAL_MAX_DELAY);
				break;
			case 1:
				sprintf(msg, "STAT_SCD:");
				HAL_UART_Transmit(&huart1, (uint8_t*)msg,
								strlen(msg), HAL_MAX_DELAY);
				break;
			case 2:
				sprintf(msg, "STAT_OV:");
				HAL_UART_Transmit(&huart1, (uint8_t*)msg,
								strlen(msg), HAL_MAX_DELAY);
				break;
			case 3:
				sprintf(msg, "STAT_UV:");
				HAL_UART_Transmit(&huart1, (uint8_t*)msg,
								strlen(msg), HAL_MAX_DELAY);
				break;
			case 4:
				sprintf(msg, "STAT_OCRD ALERT:");
				HAL_UART_Transmit(&huart1, (uint8_t*)msg,
								strlen(msg), HAL_MAX_DELAY);
				break;
			case 5:
				sprintf(msg, "STAT_DEV_XREADY:");
				HAL_UART_Transmit(&huart1, (uint8_t*)msg,
								strlen(msg), HAL_MAX_DELAY);
				break;
			case 7:
				sprintf(msg, "STAT_CC_READY:");
				HAL_UART_Transmit(&huart1, (uint8_t*)msg,
								strlen(msg), HAL_MAX_DELAY);
				break;
		}
		i++;
		if(status & mask) {
			sprintf(msg, "%d\n\r",1);
			HAL_UART_Transmit(&huart1, (uint8_t*)msg,
							strlen(msg), HAL_MAX_DELAY);
		} else {
			sprintf(msg, "%d\n\r",0);
			HAL_UART_Transmit(&huart1, (uint8_t*)msg,
							strlen(msg), HAL_MAX_DELAY);
		}
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
