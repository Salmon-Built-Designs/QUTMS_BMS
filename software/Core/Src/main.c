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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "bq769x0.h"
#include "BMS_CAN_Messages.h"
#include "temp_sensor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define N_CELLS	10
#define OVER_VOLTAGE 4000
#define UNDER_VOLTAGE 2000

// OVRD_ALERT, SCD, OCD
#define SYS_STAT_FLAG_BITS 0b00111111
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
bool CAN_error = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void SystemClock_8MHz_Config(void);

uint8_t startup_procedure();

uint8_t GetHardwareID();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t group_voltages[4];
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

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
//	  MX_CAN_Init();
	MX_TIM2_Init();
	MX_TIM1_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */

	if (!startup_procedure()) {
		// failed to start up
		Error_Handler();
	}

	// start as LOW to be good
	HAL_GPIO_WritePin(nALARM_GPIO_Port, nALARM_Pin, GPIO_PIN_RESET);

	// force temp soc to be high (need for temp reading laterz)
	HAL_GPIO_WritePin(TEMP_SOC_GPIO_Port, TEMP_SOC_Pin, GPIO_PIN_SET);

	HAL_StatusTypeDef BQ_result = HAL_BUSY;
	//uint8_t sys_stat = 0;

	// read BMS ID
	uint8_t bms_id = 0; //GetHardwareID();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	uint32_t txMailbox = 0;
	BMS_TransmitVoltage_t voltage_msg;
	BMS_TransmitTemperature_t temp_msg;
	BMS_BadCellVoltage_t voltage_error_msg;
	BMS_BadCellTemperature_t temp_error_msg;
	CAN_TxHeaderTypeDef header = { 0 };
	header.IDE = CAN_ID_EXT;
	header.RTR = CAN_RTR_DATA;
	header.TransmitGlobalTime = DISABLE;
	temp_reading current_temp_reading = { 0 };
	uint16_t temp_error = 0;
	uint16_t volt_error = 0;

	HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);

	// start CAN heartbeat timer
	uint8_t CAN_count_between_heartbeats = 0;

	HAL_Delay(1000);

	// initialize CAN
	// flag so we can go into shipping from CAN error
	CAN_error = true;
	MX_CAN_Init();
	Configure_CAN(&hcan);
	CAN_error = false;

	HAL_Delay(500);

	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		HAL_Delay(100);

		bms_id = GetHardwareID();

		/*
		 if (HAL_GPIO_ReadPin(CELL_ALERT_GPIO_Port, CELL_ALERT_Pin)) {
		 // detected an error
		 // pin is high whats going on
		 sys_stat = 0;
		 BQ_result = bq769x0_reg_read_byte(&hi2c1, BQ_SYS_STAT, &sys_stat);

		 if (BQ_result !=// HAL_OK) {
		 // error, couldn't talk to BQ chip??
		 Error_Handler();
		 }

		 HAL_UART_Transmit(&huart1, &sys_stat, sizeof(uint8_t),
		 HAL_MAX_DELAY);

		 if (sys_stat & BQ_SYS_OV) {
		 // OVER VOLTAGE
		 // send error message
		 voltage_error_msg = Compose_BMS_BadCellVoltage(bms_id, 0, 0);
		 header.ExtId = voltage_error_msg.id;
		 header.DLC = sizeof(voltage_error_msg.data);
		 HAL_CAN_AddTxMessage(&hcan, &header, voltage_error_msg.data, &txMailbox);
		 }

		 HAL_GPIO_WritePin(nALARM_GPIO_Port, nALARM_Pin, GPIO_PIN_SET);

		 // disable DSG
		 //BQ_result = bq769x0_set_DSG(&hi2c1, 0);
		 }
		 */
		CAN_count_between_heartbeats++;

		if (CAN_count_between_heartbeats > 60) {
			BQ_result = bq769x0_set_DSG(&hi2c1, 0);
			BQ_result = bq769x0_enter_shipping_mode(&hi2c1);
			// this turns off the board lmao
		}

		//HAL_UART_Transmit(&huart1, &CAN_count_between_heartbeats, sizeof(uint8_t),	HAL_MAX_DELAY);

		while (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0) {
			CAN_MSG_Generic_t msg;
			HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &(msg.header), msg.data);

			CAN_count_between_heartbeats = 0;  // reset CAN heartbeat timer
		}

		get_temp_reading();
		//HAL_Delay(500); // simulate temp_reading
		current_temp_reading = parse_temp_readings(raw_temp_readings,
				&temp_error);

		if (temp_error > 0) {
			temp_error_msg = Compose_BMS_BadCellTemperature(bms_id, 0,
					current_temp_reading.temps[0]);
			header.ExtId = temp_error_msg.id;
			header.DLC = sizeof(temp_error_msg.data);
			HAL_CAN_AddTxMessage(&hcan, &header, temp_error_msg.data,
					&txMailbox);
		}

		for (int i = 0; i < NUM_TEMPS; i++) {
			HAL_UART_Transmit(&huart1, &current_temp_reading.temps[i], 1,
			HAL_MAX_DELAY);
		}

		HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

		// send temps

		// send temp block 1
		temp_msg = Compose_BMS_TransmitTemperature(bms_id, 0,
				&current_temp_reading.temps[0]);
		header.ExtId = temp_msg.id;
		header.DLC = sizeof(temp_msg.data);
		HAL_CAN_AddTxMessage(&hcan, &header, temp_msg.data, &txMailbox);

		// send temp block 2
		temp_msg = Compose_BMS_TransmitTemperature(bms_id, 1,
				&current_temp_reading.temps[6]);
		header.ExtId = temp_msg.id;
		header.DLC = sizeof(temp_msg.data);
		HAL_CAN_AddTxMessage(&hcan, &header, temp_msg.data, &txMailbox);

		HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

		for (int i = 0; i < 3; i++) {
			// read and transmit all voltages
			BQ_result = bq769x0_read_voltage_group(&hi2c1, i, group_voltages);
			if (BQ_result != HAL_OK) {
				Error_Handler();
			}

			//uint8_t idx = i;
			//HAL_UART_Transmit(&huart1, &idx, 1, HAL_MAX_DELAY);

			volt_error = 0;

			for (int j = 0; j < 4; j++) {
				if ((group_voltages[j] > OVER_VOLTAGE)
						|| (group_voltages[j] < UNDER_VOLTAGE)) {
					volt_error |= (1 << (i * 4 + j));
				}
				uint8_t volt = (float) group_voltages[j] / 100;
				HAL_UART_Transmit(&huart1, &volt, 1,
				HAL_MAX_DELAY);

				if ((volt_error & 0x3FF) > 0) {
					voltage_error_msg = Compose_BMS_BadCellVoltage(bms_id,
							i * 4 + j, group_voltages[j]);
					header.ExtId = voltage_error_msg.id;
					header.DLC = sizeof(voltage_error_msg.data);
					HAL_CAN_AddTxMessage(&hcan, &header, voltage_error_msg.data,
							&txMailbox);

					// voltage error means set alarm line
					//HAL_GPIO_WritePin(nALARM_GPIO_Port, nALARM_Pin, GPIO_PIN_SET);
				}
			}

			// transmit voltage block
			voltage_msg = Compose_BMS_TransmitVoltage(bms_id, i,
					group_voltages);
			header.ExtId = voltage_msg.id;
			header.DLC = sizeof(voltage_msg.data);
			HAL_CAN_AddTxMessage(&hcan, &header, voltage_msg.data, &txMailbox);

		}

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV8;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1
			| RCC_PERIPHCLK_I2C1;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
/**
 * @brief System Clock Configuration for 8MHz based on generated config.
 * @retval None
 */
void SystemClock_8MHz_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI; // (0x00000002U)
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;				  // (0x1UL << (0U))
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;	// (0x10U)
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;			   // (0x00000000U)
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;	// (0x00000002U)|(0x00000001U)|(0x00000004U)
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;	// (0x00000000U)
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;// (0x00000000U) //DIVIDER is primary diff
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;		// (0x00000000U)

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {// {struct}, (0x00000000U)
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1
			| RCC_PERIPHCLK_I2C1;	// (0x00000001U)| (0x00000020U)
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;	// (0x00000000U)
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;	// (0x00000000U)
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/*
 * Return the ID set but Hardware.
 * TODO: Requires usage of all pins.
 */
uint8_t GetHardwareID() {
	uint8_t hardware_ID = 0;
	hardware_ID |= HAL_GPIO_ReadPin(ID0_GPIO_Port, ID0_Pin)
			| (HAL_GPIO_ReadPin(ID1_GPIO_Port, ID1_Pin) << 1)
			| (HAL_GPIO_ReadPin(ID2_GPIO_Port, ID2_Pin) << 2)
			| (HAL_GPIO_ReadPin(ID3_GPIO_Port, ID3_Pin) << 3);

	return hardware_ID;
}

uint8_t startup_procedure() {
	// force temp soc to be high
	HAL_GPIO_WritePin(TEMP_SOC_GPIO_Port, TEMP_SOC_Pin, GPIO_PIN_SET);

	// turn both LEDs off for debugging (auto go on from GPIO init)
	HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

	// check BQ chip
	HAL_StatusTypeDef BQ_result = HAL_BUSY;
	uint8_t sys_stat = 0;

	// set under and over voltage
	uint16_t voltage_limit = UNDER_VOLTAGE;
	if (bq769x0_set_under_voltage(&hi2c1, voltage_limit) != HAL_OK) {
		Error_Handler();
	}

	voltage_limit = OVER_VOLTAGE;
	if (bq769x0_set_over_voltage(&hi2c1, voltage_limit) != HAL_OK) {
		Error_Handler();
	}

	// get initial sys_stat register
	if (bq769x0_reg_read_byte(&hi2c1, BQ_SYS_STAT, &sys_stat) != HAL_OK) {
		// error, couldn't talk to BQ chip??
		Error_Handler();
	}

	HAL_UART_Transmit(&huart1, &sys_stat, sizeof(uint8_t), HAL_MAX_DELAY);

	// check value from sys_stat
	// do this after setting over and under voltage
	// so if we got an error caused by incorrect calibration if should stay cleared
	if ((sys_stat & SYS_STAT_FLAG_BITS) > 0) {
		// error has been detected potentially

		// clear read cycle to phase out old values
		sys_stat = sys_stat & SYS_STAT_FLAG_BITS;
		bq769x0_reg_write_byte(&hi2c1, BQ_SYS_STAT, sys_stat);
		HAL_UART_Transmit(&huart1, &sys_stat, sizeof(uint8_t), HAL_MAX_DELAY);

		bq769x0_reg_read_byte(&hi2c1, BQ_SYS_STAT, &sys_stat);
		HAL_UART_Transmit(&huart1, &sys_stat, sizeof(uint8_t), HAL_MAX_DELAY);

		// clear and read final register values

		sys_stat = sys_stat & SYS_STAT_FLAG_BITS;
		bq769x0_reg_write_byte(&hi2c1, BQ_SYS_STAT, sys_stat);
		HAL_UART_Transmit(&huart1, &sys_stat, sizeof(uint8_t), HAL_MAX_DELAY);

		bq769x0_reg_read_byte(&hi2c1, BQ_SYS_STAT, &sys_stat);
		HAL_UART_Transmit(&huart1, &sys_stat, sizeof(uint8_t), HAL_MAX_DELAY);

		for (int i = 0; i < 3; i++) {
			// read and transmit all voltages
			if (bq769x0_read_voltage_group(&hi2c1, i, group_voltages)
					!= HAL_OK) {
				Error_Handler();
			}
		}
		/*
		 if ((BQ_result != HAL_OK) || (sys_stat & SYS_STAT_FLAG_BITS) != 0) {
		 // couldn't talk to BQ or theres still an error
		 Error_Handler();
		 }
		 */
	}

	// everything is fine so turn the LEDs back on
	HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED0_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

	// enable DSG
	BQ_result = bq769x0_set_DSG(&hi2c1, 1);
	if (BQ_result != HAL_OK) {
		// error, couldn't talk to BQ chip??
		Error_Handler();
	}

	// full clock speed
	SystemClock_8MHz_Config();

	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_USART1_UART_Init();
	MX_TIM2_Init();
	MX_TIM1_Init();
	MX_TIM3_Init();

	//bqpower = HAL_GPIO_ReadPin(BQ_POWER_GPIO_Port, BQ_POWER_Pin);
	//HAL_UART_Transmit(&huart1, &bqpower, sizeof(uint8_t), HAL_MAX_DELAY);

	// everything is fine
	return 1;
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */

	char value = 'q';
	HAL_UART_Transmit(&huart1, (uint8_t*) &value, sizeof(char),
	HAL_MAX_DELAY);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);

	if (CAN_error) {
		// couldn't start up cuz not in car so just turn off lol
		bq769x0_set_DSG(&hi2c1, 0);
		bq769x0_enter_shipping_mode(&hi2c1);
	}

	/* User can add his own implementation to report the HAL error return state */
	while (1) {
		HAL_Delay(100);
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
