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
#include "wwdg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "bq769x0.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct TMP05 {
	float TEMP1[4];
	float TEMP2[4];
	float TEMP3[3];
	float TEMP4[4];
	float TEMP5[4];
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define N_CELLS	10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char msgBuffer[256];

CAN_TxHeaderTypeDef TxHeader;

uint8_t TxData[8];

uint16_t voltage_read[N_CELLS] = {0};

volatile struct TMP05 TMP05_Readings;
extern int tempsegment;
long int temp_high0, temp_low0;		// Figure out replacement. struct in struct.
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void SystemClock_8MHz_Config(void);
void delay_us (uint16_t us);

void send_Pulse(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
float TMP05_PeriodsToTMPs(long int high, long int low);

char *FloatToStr(float x);
void bq769x0_PrintStatusRegister(uint8_t stat);
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

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  	// SystemClock_8MHz_Config();	// Uncomment to skip shipment.
    MX_USART1_UART_Init();
    sprintf(msgBuffer, "DEBUG: Waiting Exit From ShipMod");
	if(HAL_UART_Transmit(&huart1, (uint8_t*) msgBuffer, strlen((char*) msgBuffer),
			HAL_MAX_DELAY) != HAL_OK) Error_Handler();
	HAL_Delay(300);
	// If Ship mode exited restore to 8MHz and external oscillator
	SystemClock_8MHz_Config();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  //MX_USART1_UART_Init();
  MX_CAN_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_WWDG_Init();
  /* USER CODE BEGIN 2 */
    // Set initial BQ variables.
    uint8_t sys_stat = 0;
    HAL_StatusTypeDef BQ_result = HAL_BUSY;

	// Initialize CAN BUS ID
    TxHeader.ExtId = 0x02;	//TODO: Get ID from pins
	TxHeader.IDE = CAN_ID_EXT;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 2;
	TxHeader.TransmitGlobalTime = DISABLE;

	// Start the us Timer. Hold enabling TIM3
	HAL_TIM_Base_Start(&htim1);
	//HAL_TIM_Base_Start(&htim3);
	HAL_Delay(1);

	sprintf(msgBuffer, "DEBUG: BQ, CAN, TMP set.\r\n");
	if(HAL_UART_Transmit(&huart1, (uint8_t*) msgBuffer, strlen((char*) msgBuffer),
			HAL_MAX_DELAY) != HAL_OK) Error_Handler();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	// Get status from BQ
	HAL_Delay(1);	//t_i2c_startup = 1ms Delay
	BQ_result = bq769x0_reg_read_byte(&hi2c1, BQ_SYS_STAT,
			&sys_stat);

	if (BQ_result != HAL_OK) {
		sprintf(msgBuffer, "ERROR: BQ reading sys_stat.\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t*) msgBuffer, strlen((char*) msgBuffer),
				HAL_MAX_DELAY);
	} else {
		sprintf(msgBuffer, "DEBUG: BQ sys_stat: %d\r\n", sys_stat);
		HAL_UART_Transmit(&huart1, (uint8_t*) msgBuffer, strlen((char*) msgBuffer),
				HAL_MAX_DELAY);
		// DEBUG: Print Sys stat register
		bq769x0_PrintStatusRegister(sys_stat);

		// Clear SYS STAT to allow
		if (sys_stat > 0) {
			uint8_t clear = sys_stat;// & 0b00010011;
			// SCD
			BQ_result = bq769x0_reg_write_byte(&hi2c1,
					BQ_SYS_STAT, clear);
			sprintf(msgBuffer, "result: %d\r\n", BQ_result);
			HAL_UART_Transmit(&huart1, (uint8_t*) msgBuffer, strlen((char*) msgBuffer),
					HAL_MAX_DELAY);
		}
	}

	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_GPIO_WritePin(GPIOB, LED0_Pin, GPIO_PIN_RESET);
		for (int i = 0; i < N_CELLS; i++) {

			// Start by reading Voltage of 1 cell
			BQ_result = bq769x0_read_voltage(&hi2c1, i, &voltage_read[i]);
			if (BQ_result != HAL_OK) {
				sprintf(msgBuffer, "ERROR: Voltage %d Unsuccessful.\r\n", i);
				HAL_UART_Transmit(&huart1, (uint8_t*) msgBuffer, strlen((char*) msgBuffer),
						HAL_MAX_DELAY);
			} else {
				// Perform temperature readings.
				HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_RESET);
				HAL_TIM_Base_Start(&htim3);			// Figure out if necessary to
				//HAL_TIM_Base_Start_IT(&htim3);	//starts interrupt.

				tempsegment = 0;
				send_Pulse(GPIOA, GPIO_PIN_1);
				//TODO: This is a place for Watch DoG timer.
				while(tempsegment < 7) {}		// 2 Sensors took 4(5) iterations; 3 Sensors 6(7)

				//TODO: Incorrect assignment. Fix later
				TMP05_Readings.TEMP1[i] += TMP05_PeriodsToTMPs(temp_high0, temp_low0);
				TMP05_Readings.TEMP2[i] += TMP05_PeriodsToTMPs(temp_high0, temp_low0);
				TMP05_Readings.TEMP3[i] += TMP05_PeriodsToTMPs(temp_high0, temp_low0);
				TMP05_Readings.TEMP4[i] += TMP05_PeriodsToTMPs(temp_high0, temp_low0);
				TMP05_Readings.TEMP5[i] += TMP05_PeriodsToTMPs(temp_high0, temp_low0);

				HAL_TIM_Base_Stop(&htim3);
				//HAL_TIM_Base_Stop_IT(&htim3);
				HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_SET);
			}

			// Transmit one voltage over CAN-bus
			sprintf(msgBuffer, "Voltage %d: %d.\r\n", i, voltage_read[i]);
			HAL_UART_Transmit(&huart1, (uint8_t*) msgBuffer,
					strlen((char*) msgBuffer), HAL_MAX_DELAY);
		}
		HAL_GPIO_WritePin(GPIOB, LED0_Pin, GPIO_PIN_SET);

		int i = 0;
		sprintf(msgBuffer, "Temp1: %s C ::: Temp2: %s C ::: Temp3: %s "
				"::: Temp4: %s C ::: Temp5: %s  C.\n",
				FloatToStr(TMP05_Readings.TEMP1[i]/N_CELLS),
				FloatToStr(TMP05_Readings.TEMP2[i]/N_CELLS),
				FloatToStr(TMP05_Readings.TEMP3[i]/N_CELLS),
				FloatToStr(TMP05_Readings.TEMP4[i]/N_CELLS),
				FloatToStr(TMP05_Readings.TEMP5[i]/N_CELLS)
				);

		HAL_UART_Transmit(&huart1, (uint8_t*)msgBuffer,
					 strlen((char*)msgBuffer), HAL_MAX_DELAY);

		//TODO: Continue with decision on Balancing.

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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV8;
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
/**
  * @brief System Clock Configuration for 8MHz based on generated config.
  * @retval None
  */
void SystemClock_8MHz_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Initializes the RCC Oscillators according to the specified parameters
	* in the RCC_OscInitTypeDef structure.
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI; // (0x00000002U)
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;				   // (0x1UL << (0U))
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;	// (0x10U)
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;			   // (0x00000000U)
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
							  |RCC_CLOCKTYPE_PCLK1;		    // (0x00000002U)|(0x00000001U)|(0x00000004U)
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;	// (0x00000000U)
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;		// (0x00000000U)
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;		// (0x00000000U)

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {	// {struct}, (0x00000000U)
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;	// (0x00000001U)| (0x00000020U)
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;					// (0x00000000U)
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;						// (0x00000000U)
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}
/*
 * Microseconds delay. With 48MHz  - 1MHz.
 */
void delay_us (uint16_t us) {
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
}

/*
 * Datasheet claims that in daisy chain the pulse have to be from Low to High.
 * Withing time 20nS > delay > 25us.
 */
void send_Pulse(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
	// Send Pulse
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
	delay_us(2);
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}

/*
 * Converts periods of ADC into Celsius values. In what time value?
 */
float TMP05_PeriodsToTMPs(long int high, long int low) {
	return 421-(751*((((float)high)/1000)/(((float)low)/1000)));
}

/*
 * Inhereted from Atmega controllers.
 * Converts float reading to String, which later can be send
 * through UART.
 *
 * @param 	- x any floating number, either negative or positive
 * @return	- Same float as a string.
 */
char *FloatToStr(float x) {
	char *tmpSign = (x < 0) ? "-" : "";
	float tmpVal = (x < 0) ? -x : x;

	int tmpInt1 = tmpVal;                  // Get the integer (678).
	float tmpFrac = tmpVal - tmpInt1;      // Get fraction (0.0123).
	int tmpInt2 = trunc(tmpFrac * 10000);  // Turn into integer (123).

	sprintf (msgBuffer, "%s%d.%04d\n", tmpSign, tmpInt1, tmpInt2);

	return msgBuffer;
}

/*
 * Similar to MAX14920 helps to visualise Values of the register with references.
 * Uses a Masking approach to get each bit of 8bit variables and prints as a
 * string over HAL.
 *
 * @param	- stat unsigned 8 bit variable
 */
void bq769x0_PrintStatusRegister(uint8_t stat) {
	sprintf(msgBuffer, "\n\n\r ||||||||||||||||||||||||||\n SYS_STAT: ");
	HAL_UART_Transmit(&huart1, (uint8_t*) msgBuffer, strlen((char*) msgBuffer),
			HAL_MAX_DELAY);
	int i = 0;
	for (unsigned int mask = 0x80; mask != 0; mask >>= 1) {
		switch (i) {
			case 0:		// 1: Fresh Coulomb Counter reading is available.
				sprintf (msgBuffer, " CC_READY:");
				break;
			case 1:		// Reserved. Do not us.
				sprintf (msgBuffer, " RSVD:");
				break;
			case 2:		// 1: Internal chip fault detected.
				sprintf (msgBuffer, " XREADY:");
				break;
			case 3:		// 1: External override detected.
				sprintf (msgBuffer, " ALERT:");
				break;
			case 4:		// 1: UnderVoltage Detected
				sprintf (msgBuffer, " UV:");
				break;
			case 5:		// 1: OverVoltage Detected
				sprintf (msgBuffer, " OV:");
				break;
			case 6:		// 1: Short circuit in discharge fault Detected
				sprintf (msgBuffer, " SCD:");
				break;
			case 7:		// 1: Over current in discharge fault Detected
				sprintf (msgBuffer, " OCD:");
				break;
		}
		HAL_UART_Transmit(&huart1, (uint8_t*) msgBuffer,
					strlen((char*) msgBuffer), HAL_MAX_DELAY);

		if (stat & mask) sprintf (msgBuffer, "%d", 1);	// bit is 1
		else 			 sprintf (msgBuffer, "%d", 0);	// bit is 0

		HAL_UART_Transmit(&huart1, (uint8_t*) msgBuffer,
							strlen((char*) msgBuffer), HAL_MAX_DELAY);
		i++;
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
	while(1) {
		sprintf(msgBuffer, "Error Handler Loop.\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t*) msgBuffer, strlen((char*) msgBuffer),
				HAL_MAX_DELAY);
		HAL_Delay(1000);
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
