/*
 * bq769x0.h
 *
 *  Created on: Sep 11, 2020
 *      Author: Calvin Johnson
 */

#ifndef INC_BQ769X0_H_
#define INC_BQ769X0_H_

#include "stm32f0xx_hal.h"

// defines
#define BQ_I2C_ADDRESS 0x18

// registers
#define BQ_SYS_STAT 0x00
#define BQ_SYS_CTRL2 0x05
#define BQ_OV_TRIP 0x09
#define BQ_UV_TRIP 0x0A
#define BQ769X0_REG_VC1_HI 0x0C
#define BQ_BAT_HI 0x2A
#define BQ_BAT_LO 0x2B
#define BQ_ADCOFFSET 0x51
#define BQ_ADCGAIN1 0x50
#define BQ_ADCGAIN2 0x59

// structs

// variables

// functions
HAL_StatusTypeDef bq769x0_reg_write_byte(I2C_HandleTypeDef *hi2c, uint8_t reg,
		uint8_t value);
HAL_StatusTypeDef bq769x0_reg_write_bytes(I2C_HandleTypeDef *hi2c, uint8_t reg,
		uint8_t *buffer, size_t length);
HAL_StatusTypeDef bq769x0_reg_read_byte(I2C_HandleTypeDef *hi2c, uint8_t reg,
		uint8_t *value);
HAL_StatusTypeDef bq769x0_reg_read_bytes(I2C_HandleTypeDef *hi2c, uint8_t reg,
		uint8_t *buffer, size_t length);

HAL_StatusTypeDef bq769x0_read_gain_and_offset(I2C_HandleTypeDef *hi2c,
		uint8_t *gain, uint8_t *offset);
HAL_StatusTypeDef bq769x0_read_voltage(I2C_HandleTypeDef *hi2c, int cell,
		uint16_t *voltage);
HAL_StatusTypeDef bq769x0_read_pack_voltage(I2C_HandleTypeDef *hi2c, int total_cells,
		uint16_t *voltage);

HAL_StatusTypeDef bq769x0_set_under_voltage(I2C_HandleTypeDef *hi2c, uint16_t under_voltage);
HAL_StatusTypeDef bq769x0_set_over_voltage(I2C_HandleTypeDef *hi2c, uint16_t over_voltage);

HAL_StatusTypeDef bq769x0_set_DSG(I2C_HandleTypeDef *hi2c, uint8_t value);


#endif /* INC_BQ769X0_H_ */
