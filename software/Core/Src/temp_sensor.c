/*
 * temp_sensor.c
 *
 *  Created on: Nov 10, 2020
 *      Author: Calvin Johnson
 */

#include "temp_sensor.h"
#include "tim.h"

//GPIO_TypeDef *ports[5] = { TEMP2_GPIO_Port/*TEMP1_GPIO_Port*/, TEMP2_GPIO_Port,
//		TEMP3_GPIO_Port, TEMP4_GPIO_Port, TEMP5_GPIO_Port };
//uint16_t pins[5] = { TEMP2_Pin/*TEMP1_Pin*/, TEMP2_Pin, TEMP3_Pin, TEMP4_Pin,
//		TEMP5_Pin };

// TODO: match this to physical number correctly
uint8_t num_temp_readings[4] = { 9, 7, 9, 7 };
uint8_t num_readings[4];
raw_temp_reading raw_temp_readings[4];


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	//HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
	/*if (htim->Instance == TIM2) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
			// temp1

		} else*/ if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
			// temp2
			//HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
			if (num_readings[0] < 9) {
				raw_temp_readings[0].times[num_readings[0]] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
				num_readings[0]++;
			}
			//__HAL_TIM_SET_COUNTER(&htim2,0);
			/*
		} else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
			// temp3
		}*/
	}
}

void temp_sensor_init() {
	//HAL_TIM_RegisterCallback(&htim2, HAL_TIM_IC_CAPTURE_CB_ID, Timer2_IC_CaptureCallback);
	//HAL_TIM_RegisterCallback(&htim3, HAL_TIM_IC_CAPTURE_CB_ID, Timer3_IC_CaptureCallback);
}

temp_reading parse_temp_readings(raw_temp_reading raw_readings[4]) {
	temp_reading reading = { 0 };
	int temp_num = 0;
	for (int i = 0; i < 4; i++) {
		int num_temp_vals = (num_temp_readings[i] - 1) / 2;
		for (int j = 0; j < num_temp_vals; j++) {
			long th = raw_readings[i].times[j * 2 + 1]
					- raw_readings[i].times[j * 2];
			long tl = raw_readings[i].times[j * 2 + 2]
					- raw_readings[i].times[j * 2 + 1];
			if (tl == 0 || th == 0) {
				reading.temps[temp_num] = 0;
			} else {
				reading.temps[temp_num] = 421 - (751 * ((double) th / tl));
			}
			temp_num++;

		}
	}
	return reading;
}




