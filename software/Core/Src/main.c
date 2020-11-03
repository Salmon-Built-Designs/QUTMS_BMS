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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct TMP {
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

CAN_TxHeaderTypeDef 	TxHeader;
uint8_t					TxData[8];
uint32_t 				TxMailbox;

uint16_t voltage_read[N_CELLS] = {0};
uint16_t overalVoltage = 0;

volatile struct TMP TMP05_Readings;

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

uint16_t GetHardwareID();
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
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  //MX_I2C1_Init();
 // MX_USART1_UART_Init();
  ///MX_CAN_Init();
  //MX_TIM1_Init();
  /* USER CODE BEGIN 2 */


  SystemClock_8MHz_Config();
  MX_GPIO_Init();
  HAL_GPIO_WritePin(GPIOB, LED0_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_SET);

      HAL_Delay(10);

  HAL_GPIO_WritePin(GPIOB, LED0_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_RESET);

  MX_CAN_Init();

  HAL_GPIO_WritePin(GPIOB, LED0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_SET);


	// Initialize CAN BUS ID
    //Configure_CAN(&hcan);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_GPIO_WritePin(GPIOB, LED0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_SET);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(GPIOB, LED0_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_RESET);
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
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;		// (0x00000000U) //DIVIDER is primary diff
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
	//__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	//while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
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
 * Return the ID set but Hardware.
 * TODO: Requires usage of all pins.
 */
uint16_t GetHardwareID() {
	return 0;//(HAL_GPIO_ReadPin(GPIOB, ID1_Pin));
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	HAL_GPIO_WritePin(GPIOB, LED0_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_RESET);
	/* User can add his own implementation to report the HAL error return state */
	while(1) {
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
