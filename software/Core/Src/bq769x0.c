/*
 * bq769x0.c
 *
 *  Created on: Sep 11, 2020
 *      Author: Calvin Johnson
 */

// BQ chip is bq7693006DBT

#include "bq769x0.h"
#include <stdlib.h>

#define BQ_READ 1
#define BQ_WRITE 0

#define TIMEOUT_MINUTE 60000

static uint8_t adc_voltage_gain = 0;
static uint8_t adc_voltage_offset = 0;

HAL_StatusTypeDef bq769x0_reg_write_byte(I2C_HandleTypeDef *hi2c, uint8_t reg,
		uint8_t value) {
	uint8_t address = BQ_I2C_ADDRESS << 1 | 0;

	// no crc as chip doesn't support it

	uint8_t data[2] = { reg, value };

	return HAL_I2C_Master_Transmit(hi2c, address, data, sizeof(data),
			TIMEOUT_MINUTE);
}

HAL_StatusTypeDef bq769x0_reg_write_bytes(I2C_HandleTypeDef *hi2c, uint8_t reg,
		uint8_t *buffer, size_t length) {
	uint8_t address = BQ_I2C_ADDRESS << 1 | 0;

	// no crc as chip doesn't support it

	// transmit register as first byte, then all of buffer according to length

	uint8_t dataLength = 1 + length;
	uint8_t *data = (uint8_t*) calloc(dataLength, sizeof(uint8_t));
	data[0] = reg;

	memcpy(data + 1, buffer, length);

	HAL_StatusTypeDef result = HAL_I2C_Master_Transmit(hi2c, address, data,
			dataLength, TIMEOUT_MINUTE);

	// clean up dynamic memory
	free(data);

	return result;
}

HAL_StatusTypeDef bq769x0_reg_read_byte(I2C_HandleTypeDef *hi2c, uint8_t reg,
		uint8_t *value) {
	uint8_t address = BQ_I2C_ADDRESS << 1 | 0;
	HAL_StatusTypeDef result = HAL_I2C_Master_Transmit(hi2c, address, &reg, 1,
			TIMEOUT_MINUTE);

	if (result != HAL_OK) {
		// unable to send read request correctly
		return result;
	} else {
		return HAL_I2C_Master_Receive(hi2c, address, value, 1, TIMEOUT_MINUTE);
	}
}

HAL_StatusTypeDef bq769x0_reg_read_bytes(I2C_HandleTypeDef *hi2c, uint8_t reg,
		uint8_t *buffer, size_t length) {
	uint8_t address = BQ_I2C_ADDRESS << 1 | 0;
	HAL_StatusTypeDef result = HAL_I2C_Master_Transmit(hi2c, address, &reg, 1,
			TIMEOUT_MINUTE);

	if (result != HAL_OK) {
		// unable to send read request correctly
		return result;
	} else {
		return HAL_I2C_Master_Receive(hi2c, address, buffer, length,
				TIMEOUT_MINUTE);
	}
}

HAL_StatusTypeDef bq769x0_read_gain_and_offset(I2C_HandleTypeDef *hi2c,
		uint8_t *gain, uint8_t *offset) {
	uint8_t reading = 0;
	HAL_StatusTypeDef result;

	// ADCOFFSET[7:0] = OFFSET[7:0]
	result = bq769x0_reg_read_byte(&hi2c1, BQ_ADCOFFSET, &reading);
	if (result != HAL_OK) {
		*gain = 0;
		*offset = 0;
		return result;
	}

	*offset = reading;
	reading = 0;

	// ADCGAIN2[7:5] = GAIN[2:0]
	result = bq769x0_reg_read_byte(&hi2c1, BQ_ADCGAIN2, &reading);
	if (result != HAL_OK) {
		*gain = 0;
		*offset = 0;
		return result;
	}

	*gain = (reading & 0b11100000) >> 5;
	reading = 0;

	// ADCGAIN1[3:2] = GAIN[4:3]
	result = bq769x0_reg_read_byte(&hi2c1, BQ_ADCGAIN1, &reading);
	if (result != HAL_OK) {
		*gain = 0;
		*offset = 0;
		return result;
	}

	*gain |= ((reading & 0b00001100) << 1);

	adc_voltage_gain = *gain;
	adc_voltage_offset = *offset;

	return HAL_OK;
}

HAL_StatusTypeDef bq769x0_read_voltage(I2C_HandleTypeDef *hi2c, int cell,
		uint16_t *voltage) {
	if (adc_voltage_gain == 0 || adc_voltage_offset == 0) {
		bq769x0_read_gain_and_offset(hi2c, &adc_voltage_gain,
				&adc_voltage_offset);
	}

	uint8_t reg = BQ769X0_REG_VC1_HI + cell * 2;
	uint8_t buffer[2];

	// FIXME
	HAL_StatusTypeDef ret = bq769x0_reg_read_bytes(hi2c, reg, buffer, 2);
	if (ret != HAL_OK) {
		//SYS_LOG_ERR("failed to read cell %d voltage", cell);
		return ret;
	}

	uint16_t adc_value = ((buffer[0] & 0b00111111) << 8) | buffer[1];
	*voltage = adc_value * (365 + adc_voltage_gain) / 1000 + adc_voltage_offset;

	return 0;
}
