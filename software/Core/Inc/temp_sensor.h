/*
 * temp_sensor.h
 *
 *  Created on: Nov 10, 2020
 *      Author: Calvin Johnson
 */

#ifndef INC_TEMP_SENSOR_H_
#define INC_TEMP_SENSOR_H_

#include "main.h"

#define MAX_NUM_READINGS 9
#define NUM_TEMPS 14
#define NUM_TEMP_LINES 4


#define DANGER_TEMP 50

// GPIO DEFINES
#define TEMP1_PORT GPIOA
#define TEMP2_PORT GPIOA
#define TEMP3_PORT GPIOB
#define TEMP4_PORT GPIOB

#define TEMP1_PIN GPIO_PIN_15
#define TEMP2_PIN GPIO_PIN_3
#define TEMP3_PIN GPIO_PIN_3
#define TEMP4_PIN GPIO_PIN_4

#define TEMP1_EXT EXTI4_15_IRQn
#define TEMP2_EXT EXTI2_3_IRQn
#define TEMP3_EXT EXTI2_3_IRQn
#define TEMP4_EXT EXTI4_15_IRQn

enum temp_lines {
	TEMP_LINE_1 = 0,
	TEMP_LINE_2 = 1,
	TEMP_LINE_3 = 2,
	TEMP_LINE_4 = 3
};

struct temp_reading {
	uint8_t temps[NUM_TEMPS];
};

typedef struct temp_reading temp_reading;

void temp_sensor_init();
void start_temp_reading();
void finish_temp_reading();
bool finished_temp_reading();
bool invalid_temp_reading();

temp_reading parse_temp_readings(long raw_readings[NUM_TEMP_LINES][MAX_NUM_READINGS], uint16_t *error);
void delay_us (uint16_t us);

extern uint8_t num_readings[NUM_TEMP_LINES];
extern long raw_temp_readings[NUM_TEMP_LINES][MAX_NUM_READINGS];

#endif
