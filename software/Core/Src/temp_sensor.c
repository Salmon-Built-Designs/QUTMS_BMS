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

raw_temp_reading raw_temp_readings[4];

void Timer2_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		// temp1

	} else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
		// temp2
		//HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
	} else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
		// temp3
	}
}

void Timer3_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		// temp4
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
	if (htim->Instance = TIM2) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
			// temp1

		} else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
			// temp2
			//HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
		} else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
			// temp3
		}
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

/*
 void get_temp_reading() {
 memset(raw_temp_readings, 0, sizeof(raw_temp_reading)*4);

 uint8_t read_count[4];
 uint8_t prev_state[4];
 int i = 0;
 for(i = 0; i < 4; i++) {
 read_count[i] = 0;
 // set prev state to start at LOW so first iteration records time
 prev_state[i] = 0;
 }

 uint8_t value = 0;

 // send pulse
 // low
 HAL_GPIO_WritePin(TEMP_SOC_GPIO_Port, TEMP_SOC_Pin, GPIO_PIN_RESET);
 // 20us delay TODO: WITH TIMER
 delay_us (50);

 // high
 HAL_GPIO_WritePin(TEMP_SOC_GPIO_Port, TEMP_SOC_Pin, GPIO_PIN_SET);
 // 20us delay TODO: WITH TIMER
 delay_us (5);

 // low - start reading
 HAL_GPIO_WritePin(TEMP_SOC_GPIO_Port, TEMP_SOC_Pin, GPIO_PIN_RESET);


 while(1)  {
 if (read_count[0] >= num_temp_readings[0]) {
 //break_loop = true;
 break;
 }
 for (i = 0; i < 1; i++) {
 value = HAL_GPIO_ReadPin(ports[i], pins[i]);

 if (read_count[i] >= num_temp_readings[i]) {
 continue;
 }

 if (value != prev_state[i]) {
 // set to current time
 // 1ms precision is fine for now, can do more precise later
 raw_temp_readings[i].times[read_count[i]] = HAL_GetTick();
 read_count[i]++;
 prev_state[i] = value;
 }
 }
 }

 // pull pin high again
 HAL_GPIO_WritePin(TEMP_SOC_GPIO_Port, TEMP_SOC_Pin, GPIO_PIN_SET);

 }
 */
