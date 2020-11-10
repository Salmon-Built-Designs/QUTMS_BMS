/*
 * temp_sensor.h
 *
 *  Created on: Nov 10, 2020
 *      Author: Calvin Johnson
 */

#ifndef INC_TEMP_SENSOR_H_
#define INC_TEMP_SENSOR_H_

#include "main.h"

#define NUM_READINGS 9
#define NUM_TEMPS 12

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



struct raw_temp_reading {
	long times[NUM_READINGS];
};

typedef struct raw_temp_reading raw_temp_reading;

struct temp_reading {
	uint8_t temps[NUM_TEMPS];
};

typedef struct temp_reading temp_reading;

void temp_sensor_init();
void read_temperatures(uint8_t line);
temp_reading parse_temp_readings(raw_temp_reading raw_readings[4]);

#endif
