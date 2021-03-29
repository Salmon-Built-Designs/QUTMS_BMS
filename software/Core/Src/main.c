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
#include "QUTMS_can.h"
#include "BMS_CAN_Messages.h"
#include "temp_sensor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// OVRD_ALERT, SCD, OCD
#define SYS_STAT_FLAG_BITS 0b00111111

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
bool CAN_error = false;
uint8_t voltage_error_count[10];

bool take_voltage_reading;
bool update_balancing;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void SystemClock_1MHz_Config(void);

uint8_t startup_procedure();

uint8_t GetHardwareID();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t group_voltages[4];
uint16_t current_voltages[10];

bool balancing_mode = false;

char uart_buff[80];
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	// init 1mhz clock
	SystemClock_1MHz_Config();

	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_USART1_UART_Init_1MHz();

	// exit out of shipping mode
	if (!startup_procedure()) {
		// failed to start up
		Error_Handler();
	}

	// proceed at full clock speed
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_CAN_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

	// start as LOW to be good
	HAL_GPIO_WritePin(nALARM_GPIO_Port, nALARM_Pin, GPIO_PIN_RESET);

	// force temp soc to be high (need for temp reading laterz)
	HAL_GPIO_WritePin(TEMP_SOC_GPIO_Port, TEMP_SOC_Pin, GPIO_PIN_SET);

	HAL_StatusTypeDef BQ_result = HAL_BUSY;
	//uint8_t sys_stat = 0;

	// read BMS ID
	uint8_t bms_id = GetHardwareID();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	uint32_t txMailbox = 0;
	BMS_TransmitVoltage_t voltage_msg;
	BMS_TransmitTemperature_t temp_msg;
	BMS_BadCellVoltage_t voltage_error_msg;
	BMS_BadCellTemperature_t temp_error_msg;
	BMS_TransmitBalancing_t balancing_msg;
	CAN_TxHeaderTypeDef header = { 0 };
	header.IDE = CAN_ID_EXT;
	header.RTR = CAN_RTR_DATA;
	header.TransmitGlobalTime = DISABLE;
	temp_reading current_temp_reading = { 0 };
	uint16_t temp_error = 0;
	//uint16_t volt_error = 0;

	uint16_t average_voltage = 0;

	HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);

	// start CAN heartbeat timer
	uint32_t CAN_count_between_heartbeats = HAL_GetTick();

	// initialize CAN
	// flag so we can go into shipping from CAN error
	CAN_error = true;
	MX_CAN_Init();

	// small delay to wait for all BMS to turn CAN on before we start the peripheral properly
	HAL_Delay(500);

	Configure_CAN(&hcan);
	CAN_error = false;

	HAL_Delay(250);

	// reset voltage error count
	memset(voltage_error_count, 0, 10);

	bool reading_temperature = false;

	// start voltage timer
	HAL_TIM_Base_Start_IT(&htim14);

#ifdef BMS_DEBUG_BALANCING
	balancing_mode = true;
	HAL_TIM_Base_Start_IT(&htim16);
	update_balancing = true;
#endif

	bq769x0_reset_cell_balancing(&hi2c1);
	//bq769x0_set_cell_balancing(&hi2c1, 1, 1);

	sprintf(uart_buff, "\r\nStart\r\n");
	HAL_UART_Transmit(&huart1, uart_buff, strlen(uart_buff), HAL_MAX_DELAY);

	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

		bms_id = GetHardwareID();

		// check alarm pin
		if (HAL_GPIO_ReadPin(CELL_ALERT_GPIO_Port, CELL_ALERT_Pin)) {
			// detected an error, lets read the stat register
			uint8_t sys_stat = 0;
			BQ_result = bq769x0_reg_read_byte(&hi2c1, BQ_SYS_STAT, &sys_stat);

			if (BQ_result != HAL_OK) {
				// error, couldn't talk to BQ chip??
				Error_Handler();
			}

			sprintf(uart_buff, "CELL ALERT - sys_stat: %d\r\n", sys_stat);
			HAL_UART_Transmit(&huart1, uart_buff, strlen(uart_buff), HAL_MAX_DELAY);

			if (sys_stat) {
				// just clear the stat register, voltage errors should be picked up by us
				BQ_result = bq769x0_reg_write_byte(&hi2c1, BQ_SYS_STAT, sys_stat);
				// if this happened reset dsg on just in case
				BQ_result = bq769x0_set_DSG(&hi2c1, 1);

			}
		}

		// check CAN
		while (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0) {
			CAN_MSG_Generic_t msg;
			HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &(msg.header), msg.data);

			if (msg.header.ExtId == AMS_HeartbeatResponse_ID) {
				CAN_count_between_heartbeats = HAL_GetTick();  // reset CAN heartbeat timer
			}

			// check if other bms in series is broadcasting average voltage
			// TODO:

			// check for enable balancing
			else if (msg.header.ExtId == BMS_ChargeEnabled_ID) {
				CAN_count_between_heartbeats = HAL_GetTick();  // reset CAN heartbeat timer

				if (balancing_mode == false) {
					// just entered balancing mode, so start balancing timer
					HAL_TIM_Base_Start_IT(&htim16);

					// first iteration so also skip waiting for the timer lmao
					update_balancing = true;
				}

				balancing_mode = true;
			}
		}

#ifndef BMS_DEBUG_HEARTBEAT
		// check if heartbeat has expired
		if ((HAL_GetTick() - CAN_count_between_heartbeats) > HEARTBEAT_TIMEOUT) {
			sprintf(uart_buff, "Heartbeat timer expired (been %d ms), turning off...\r\n", (HAL_GetTick() - CAN_count_between_heartbeats));
			HAL_UART_Transmit(&huart1, uart_buff, strlen(uart_buff), HAL_MAX_DELAY);

			// haven't heard from the AMS in 30 seconds, so turn off
			BQ_result = bq769x0_set_DSG(&hi2c1, 0);
			BQ_result = bq769x0_enter_shipping_mode(&hi2c1);

			// small delay so we dont spam printing the timer expired
			HAL_Delay(100);
		}

#endif

		// check if time for voltage reading
		if (take_voltage_reading /*&& ( (balancing_mode && update_balancing) || (!balancing_mode) )*/) {

			average_voltage = 0;

			sprintf(uart_buff, "Voltages:");
			HAL_UART_Transmit(&huart1, uart_buff, strlen(uart_buff), HAL_MAX_DELAY);

			// read and transmit all voltages
			for (int i = 0; i < NUM_VOLTAGE_GROUPS; i++) {
				// read voltages
				BQ_result = bq769x0_read_voltage_group(&hi2c1, i, group_voltages);
				if (BQ_result != HAL_OK) {
					Error_Handler();
				}

				// copy voltages into main voltage block
				for (int j = 0; ((j < 4) && ((i * 4 + j) < 10)); j++) {
					current_voltages[i * 4 + j] = group_voltages[j];

					// sum all voltages
					average_voltage += group_voltages[j];

					// update error count if needed
					if ((group_voltages[j] > OVER_VOLTAGE) || (group_voltages[j] < UNDER_VOLTAGE)) {
						voltage_error_count[i * 4 + j]++;
					} else {
						voltage_error_count[i * 4 + j] = 0;
					}

					// send voltage on uart
					sprintf(uart_buff, " %d", group_voltages[j]);
					HAL_UART_Transmit(&huart1, uart_buff, strlen(uart_buff), HAL_MAX_DELAY);

				}

				// transmit voltage block
				voltage_msg = Compose_BMS_TransmitVoltage(bms_id, i, group_voltages);
				header.ExtId = voltage_msg.id;
				header.DLC = sizeof(voltage_msg.data);
				HAL_CAN_AddTxMessage(&hcan, &header, voltage_msg.data, &txMailbox);
			}

			// calculate average voltage
			average_voltage = ((float) average_voltage / NUM_VOLTAGES);

			sprintf(uart_buff, "\tAV: %d\r\n", average_voltage);
			HAL_UART_Transmit(&huart1, uart_buff, strlen(uart_buff), HAL_MAX_DELAY);

			// all voltages have been read, now check for any errors
			for (int i = 0; i < NUM_VOLTAGES; i++) {
				if (voltage_error_count[i] > NUM_BAD_VOLTAGE_COUNT) {
					// bad voltage, broadcast on CAN
					voltage_error_msg = Compose_BMS_BadCellVoltage(bms_id, i, current_voltages[i]);
					header.ExtId = voltage_error_msg.id;
					header.DLC = sizeof(voltage_error_msg.data);
					HAL_CAN_AddTxMessage(&hcan, &header, voltage_error_msg.data, &txMailbox);

					sprintf(uart_buff, "Bad Voltage at %d: %d\r\n", i, current_voltages[i]);
					HAL_UART_Transmit(&huart1, uart_buff, strlen(uart_buff), HAL_MAX_DELAY);

					// trip alarm line to let AMS know
					HAL_GPIO_WritePin(nALARM_GPIO_Port, nALARM_Pin, GPIO_PIN_SET);

					break;
				}
			}

			// clear flag - wait for next reading period
			take_voltage_reading = false;
		}

		// check state of temperature reading
		if (!reading_temperature) {
			// not currently reading temperature, so start a reading
			start_temp_reading();

			reading_temperature = true;
		} else {
			// is reading finished?
			if (finished_temp_reading()) {
				finish_temp_reading();
				reading_temperature = false;
				bool invalid_reading = invalid_temp_reading();

				if (!invalid_reading) {
					// got a successful temperature reading, so now parse and transmit

					// parse reading
					current_temp_reading = parse_temp_readings(raw_temp_readings, &temp_error);

#ifndef BMS_DISABLE_PRINT_TEMPS
					sprintf(uart_buff, "Temps");
					HAL_UART_Transmit(&huart1, uart_buff, strlen(uart_buff), HAL_MAX_DELAY);
#endif
					// transmit readings
					for (int i = 0; i < NUM_TEMP_BLOCKS; i++) {
						temp_msg = Compose_BMS_TransmitTemperature(bms_id, i, &current_temp_reading.temps[i * NUM_TEMPS_PER_BLOCK]);
						header.ExtId = temp_msg.id;
						header.DLC = sizeof(temp_msg.data);
						HAL_CAN_AddTxMessage(&hcan, &header, temp_msg.data, &txMailbox);
					}

					// sent a temp reading, so flash the LEDs
					HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
					HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

#ifndef BMS_DISABLE_PRINT_TEMPS
					for (int i = 0; i < NUM_TEMPS; i++) {
						sprintf(uart_buff, " %d", current_temp_reading.temps[i]);
						HAL_UART_Transmit(&huart1, uart_buff, strlen(uart_buff), HAL_MAX_DELAY);
					}

					sprintf(uart_buff, "\r\n");
					HAL_UART_Transmit(&huart1, uart_buff, strlen(uart_buff), HAL_MAX_DELAY);
#endif

					// dump temperatures over uart
					//HAL_UART_Transmit(&huart1, current_temp_reading.temps, NUM_TEMPS, HAL_MAX_DELAY);

					// check for temperature errors
					if (temp_error > 0) {
						// find first temperature that has an issue
						for (int i = 0; i < NUM_TEMPS; i++) {
							if (current_temp_reading.temps[i] > DANGER_TEMP) {
								temp_error_msg = Compose_BMS_BadCellTemperature(bms_id, i, current_temp_reading.temps[i]);
								header.ExtId = temp_error_msg.id;
								header.DLC = sizeof(temp_error_msg.data);
								HAL_CAN_AddTxMessage(&hcan, &header, temp_error_msg.data, &txMailbox);

								sprintf(uart_buff, "Bad Temp at %d: %d\r\n", i, current_temp_reading.temps[i]);
								HAL_UART_Transmit(&huart1, uart_buff, strlen(uart_buff), HAL_MAX_DELAY);

								// only send first bad temp, so exit out
								break;
							}
						}
					}
				}
			}
		}

		// check if time to update balancing
		if (balancing_mode && update_balancing) {
			// start by clearing all balancing registers
			//bq769x0_reset_cell_balancing(&hi2c1);

			// check temperature
			// TODO:

			// calculate delta of each voltage to average
			int deltas[NUM_VOLTAGES];

			sprintf(uart_buff, "B Deltas:\t");
			HAL_UART_Transmit(&huart1, uart_buff, strlen(uart_buff), HAL_MAX_DELAY);

			for (int i = 0; i < NUM_VOLTAGES; i++) {
				deltas[i] = average_voltage - current_voltages[i];

				// if it's less than the average we don't care about it
				if (deltas[i] < 0) {
					deltas[i] = 0;
				}

				sprintf(uart_buff, " %d", deltas[i]);
				HAL_UART_Transmit(&huart1, uart_buff, strlen(uart_buff), HAL_MAX_DELAY);
			}

			sprintf(uart_buff, "\r\n");
			HAL_UART_Transmit(&huart1, uart_buff, strlen(uart_buff), HAL_MAX_DELAY);

			sprintf(uart_buff, "Balancing:\t");
			HAL_UART_Transmit(&huart1, uart_buff, strlen(uart_buff), HAL_MAX_DELAY);

			uint16_t balancing_state = 0;

			// balance each group of 5 cells separately
			for (int i = 0; i < 2; i++) {
				// determine which cluster needs more balancing
				int sum_group_1 = deltas[(i * 5)] + deltas[(i * 5) + 2] + deltas[(i * 5) + 4];
				int sum_group_2 = deltas[(i * 5) + 1] + deltas[(i * 5) + 3];

				uint8_t balancing_idx = 0;

				// flag appropriate cells for balancing
				if (sum_group_1 > sum_group_2) {
					if (deltas[(i * 5)] > BALANCING_THRESHOLD) {
						balancing_idx |= 1 << 0;
					}

					if (deltas[(i * 5) + 2] > BALANCING_THRESHOLD) {
						balancing_idx |= 1 << 2;
					}

					if (deltas[(i * 5) + 4] > BALANCING_THRESHOLD) {
						balancing_idx |= 1 << 4;
					}
				} else {
					if (deltas[(i * 5) + 1] > BALANCING_THRESHOLD) {
						balancing_idx |= 1 << 1;
					}

					if (deltas[(i * 5) + 3] > BALANCING_THRESHOLD) {
						balancing_idx |= 1 << 3;
					}
				}

				balancing_state |= balancing_idx << (5 * i);

				// set balancing for this group
				bq769x0_set_cell_balancing_reg(&hi2c1, i, balancing_idx);

				for (int i = 0; i < 5; i++) {
					sprintf(uart_buff, " %d", (balancing_idx >> i) & 1);
					HAL_UART_Transmit(&huart1, &uart_buff, strlen(uart_buff), HAL_MAX_DELAY);
				}
			}

			sprintf(uart_buff, "\r\n");
			HAL_UART_Transmit(&huart1, &uart_buff, strlen(uart_buff), HAL_MAX_DELAY);

			// send CAN message to reflect balancing update
			balancing_msg = Compose_BMS_TransmitBalancing(bms_id, average_voltage, balancing_state);
			header.ExtId = balancing_msg.id;
			header.DLC = sizeof(balancing_msg.data);
			HAL_CAN_AddTxMessage(&hcan, &header, balancing_msg.data, &txMailbox);

			// clear flag
			update_balancing = false;
		}

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
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
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
/**
 * @brief System Clock Configuration for 8MHz based on generated config.
 * @retval None
 */
void SystemClock_1MHz_Config(void) {

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
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV8;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_I2C1;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/*
 * Return the ID set but Hardware.
 */
uint8_t GetHardwareID() {
	uint8_t hardware_ID = 0;
	hardware_ID |= HAL_GPIO_ReadPin(ID0_GPIO_Port, ID0_Pin) | (HAL_GPIO_ReadPin(ID1_GPIO_Port, ID1_Pin) << 1) | (HAL_GPIO_ReadPin(ID2_GPIO_Port, ID2_Pin) << 2)
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

	sprintf(uart_buff, "Initial - sys_stat: %d\r\n", sys_stat);
	HAL_UART_Transmit(&huart1, uart_buff, strlen(uart_buff), HAL_MAX_DELAY);

	//HAL_UART_Transmit(&huart1, &sys_stat, sizeof(uint8_t), HAL_MAX_DELAY);

	// check value from sys_stat
	// do this after setting over and under voltage
	// so if we got an error caused by incorrect calibration if should stay cleared
	if ((sys_stat & SYS_STAT_FLAG_BITS) > 0) {
		// error has been detected potentially

		// clear read cycle to phase out old values
		sys_stat = sys_stat & SYS_STAT_FLAG_BITS;
		bq769x0_reg_write_byte(&hi2c1, BQ_SYS_STAT, sys_stat);

		sprintf(uart_buff, "WARNING - writing_0 sys_stat: %d: %d\r\n", sys_stat);
		HAL_UART_Transmit(&huart1, uart_buff, strlen(uart_buff), HAL_MAX_DELAY);

		//HAL_UART_Transmit(&huart1, &sys_stat, sizeof(uint8_t), HAL_MAX_DELAY);

		bq769x0_reg_read_byte(&hi2c1, BQ_SYS_STAT, &sys_stat);
		sprintf(uart_buff, "WARNING - current sys_stat: %d: %d\r\n", sys_stat);
		HAL_UART_Transmit(&huart1, uart_buff, strlen(uart_buff), HAL_MAX_DELAY);
		// HAL_UART_Transmit(&huart1, &sys_stat, sizeof(uint8_t), HAL_MAX_DELAY);

		// clear and read final register values

		sys_stat = sys_stat & SYS_STAT_FLAG_BITS;
		bq769x0_reg_write_byte(&hi2c1, BQ_SYS_STAT, sys_stat);

		sprintf(uart_buff, "WARNING - writing_1 sys_stat: %d: %d\r\n", sys_stat);
		HAL_UART_Transmit(&huart1, uart_buff, strlen(uart_buff), HAL_MAX_DELAY);

		//HAL_UART_Transmit(&huart1, &sys_stat, sizeof(uint8_t), HAL_MAX_DELAY);

		bq769x0_reg_read_byte(&hi2c1, BQ_SYS_STAT, &sys_stat);

		sprintf(uart_buff, "WARNING - final sys_stat: %d: %d\r\n", sys_stat);
		HAL_UART_Transmit(&huart1, uart_buff, strlen(uart_buff), HAL_MAX_DELAY);

		//HAL_UART_Transmit(&huart1, &sys_stat, sizeof(uint8_t), HAL_MAX_DELAY);

		for (int i = 0; i < 3; i++) {
			// read and transmit all voltages
			if (bq769x0_read_voltage_group(&hi2c1, i, group_voltages) != HAL_OK) {
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

	// everything is fine
	return 1;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */

	sprintf(uart_buff, "Error\r\n");
	HAL_UART_Transmit(&huart1, uart_buff, strlen(uart_buff), HAL_MAX_DELAY);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);

	//if (CAN_error) {
	// found error so just turn off lol
	//bq769x0_set_DSG(&hi2c1, 0);
	//bq769x0_enter_shipping_mode(&hi2c1);
//	}

	/* User can add his own implementation to report the HAL error return state */
	/*while (1) {
	 HAL_Delay(100);
	 }*/
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
