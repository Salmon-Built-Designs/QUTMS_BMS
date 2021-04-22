/*
 * temp_sensor.c
 *
 *  Created on: Nov 10, 2020
 *      Author: Calvin Johnson
 */

#include "temp_sensor.h"
#include "main.h"
#include "tim.h"
#include <string.h>

// TODO: match this to physical number correctly
uint8_t num_temp_readings[NUM_TEMP_LINES] = { 9, 7, 7, 9, 5 };
uint8_t num_readings[NUM_TEMP_LINES];
long raw_temp_readings[NUM_TEMP_LINES][MAX_NUM_READINGS];

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	uint32_t channel1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
	uint32_t channel2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
	uint32_t channel4 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);

	//HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
	if (htim->Instance == TIM3) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
			// temp1
			if (num_readings[TEMP_LINE_4] < num_temp_readings[TEMP_LINE_4]) {
				raw_temp_readings[TEMP_LINE_4][num_readings[TEMP_LINE_4]] = channel1;
				//uint32_t timer_value = __HAL_TIM_GET_COUNTER(&htim3);
				num_readings[TEMP_LINE_4]++;
				/*if (num_readings[TEMP_LINE_4] < num_temp_readings[TEMP_LINE_4]) {
				 //raw_temp_readings[TEMP_LINE_4].times[num_readings[TEMP_LINE_4]] =	timer_value;
				 __HAL_TIM_SET_COUNTER(&htim3, 0);
				 }*/

			}

		} else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
			// temp5 - balancing board
			if (num_readings[TEMP_LINE_5] < num_temp_readings[TEMP_LINE_5]) {
				raw_temp_readings[TEMP_LINE_5][num_readings[TEMP_LINE_5]] = channel2;
				//uint32_t timer_value = __HAL_TIM_GET_COUNTER(&htim3);
				num_readings[TEMP_LINE_5]++;

			}
		}
	} else if (htim->Instance == TIM2) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
			// temp1
			if (num_readings[TEMP_LINE_1] < num_temp_readings[TEMP_LINE_1]) {
				raw_temp_readings[TEMP_LINE_1][num_readings[TEMP_LINE_1]] = channel1;
				num_readings[TEMP_LINE_1]++;
			}

		} else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
			// temp2
			//HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
			if (num_readings[TEMP_LINE_2] < num_temp_readings[TEMP_LINE_2]) {
				raw_temp_readings[TEMP_LINE_2][num_readings[TEMP_LINE_2]] = channel4;
				num_readings[TEMP_LINE_2]++;
			}
			//__HAL_TIM_SET_COUNTER(&htim2,0);

		} else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
			// temp3
			if (num_readings[TEMP_LINE_3] < num_temp_readings[TEMP_LINE_3]) {
				raw_temp_readings[TEMP_LINE_3][num_readings[TEMP_LINE_3]] = channel2;
				num_readings[TEMP_LINE_3]++;
			}
		}
	}

}

void temp_sensor_init() {
	//HAL_TIM_RegisterCallback(&htim2, HAL_TIM_IC_CAPTURE_CB_ID, Timer2_IC_CaptureCallback);
	//HAL_TIM_RegisterCallback(&htim3, HAL_TIM_IC_CAPTURE_CB_ID, Timer3_IC_CaptureCallback);
}

temp_reading parse_temp_readings(long raw_readings[NUM_TEMP_LINES][MAX_NUM_READINGS], uint16_t *error) {
	*error = 0;
	temp_reading reading = { 0 };
	int temp_num = 0;
	for (int i = 0; i < NUM_TEMP_LINES; i++) {
		int num_temp_vals = (num_temp_readings[i] - 1) / 2;
		for (int j = 0; j < num_temp_vals; j++) {
			long th = 0;
			long tl = 0;
			th = raw_readings[i][j * 2 + 1] - raw_readings[i][j * 2];
			tl = raw_readings[i][j * 2 + 2] - raw_readings[i][j * 2 + 1];
			if (tl == 0 || th == 0) {
				reading.temps[temp_num] = 0;
			} else {
				reading.temps[temp_num] = 421 - (751 * ((double) th / tl));
				// cheeky check for bad temps
				if (reading.temps[temp_num] > DANGER_TEMP) {
					*error |= (1 << temp_num);
				}

			}
			temp_num++;

		}
	}
	return reading;
}

void delay_us(uint16_t us) {
	__HAL_TIM_SET_COUNTER(&htim1, 0);  // set the counter value to 0
	// timer 1 ticks at 0.5us
	while (__HAL_TIM_GET_COUNTER(&htim1) < (us)) {
	}  // wait for the counter to reach the us input in the parameter
}

uint32_t start_time_temp;
bool invalid_reading = false;

void start_temp_reading() {
	start_time_temp = HAL_GetTick();
	// reset all temps to be zero
	memset(raw_temp_readings, 0, sizeof(long) * MAX_NUM_READINGS * NUM_TEMP_LINES);

	HAL_TIM_Base_Start(&htim1);

	delay_us(10);

	raw_temp_readings[TEMP_LINE_1][0] = 0;
	raw_temp_readings[TEMP_LINE_2][0] = 0;
	raw_temp_readings[TEMP_LINE_3][0] = 0;
	raw_temp_readings[TEMP_LINE_4][0] = 0;
	raw_temp_readings[TEMP_LINE_5][0] = 0;

	num_readings[0] = 1;
	num_readings[1] = 1;
	num_readings[2] = 1;
	num_readings[3] = 1;
	num_readings[4] = 1;

	// set low
	HAL_GPIO_WritePin(TEMP_SOC_GPIO_Port, TEMP_SOC_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

	//delay_us(40);
	HAL_Delay(1);

	// set high
	HAL_GPIO_WritePin(TEMP_SOC_GPIO_Port, TEMP_SOC_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	//HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	delay_us(9);

	// start channel interrupts
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);

	__HAL_TIM_SET_COUNTER(&htim2, 0);
	__HAL_TIM_SET_COUNTER(&htim3, 0);

	// set low - start reading
	HAL_GPIO_WritePin(TEMP_SOC_GPIO_Port, TEMP_SOC_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
	//HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
}

void finish_temp_reading() {
	// raise pin high to signify finished
	HAL_GPIO_WritePin(TEMP_SOC_GPIO_Port, TEMP_SOC_Pin, GPIO_PIN_SET);

	HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_2);
	HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_4);
	HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_2);
	HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_1);

	HAL_TIM_Base_Stop(&htim1);
}

bool finished_temp_reading() {
	// have all the temp readings finished?
	if ((num_readings[TEMP_LINE_1] >= num_temp_readings[TEMP_LINE_1]) && (num_readings[TEMP_LINE_2] >= num_temp_readings[TEMP_LINE_2]) && (num_readings[TEMP_LINE_3] >= num_temp_readings[TEMP_LINE_3])
			&& (num_readings[TEMP_LINE_4] >= num_temp_readings[TEMP_LINE_4])
			// only check last temperature sensor if we're actively balancing (otherwise board might not be there lol)
			/*
			 && (balancing_mode && (num_readings[TEMP_LINE_5] >= num_temp_readings[TEMP_LINE_5]))
			 */
			) {
		invalid_reading = false;
		return true;
	}

	// has it been a second since we started the temp reading?
	if ((HAL_GetTick() - start_time_temp) > TICKS_SECOND) {
		// it's taken too long so lets just clear everything
		invalid_reading = true;
		return true;
	}

	return false;
}

bool invalid_temp_reading() {
	return invalid_reading;
}
