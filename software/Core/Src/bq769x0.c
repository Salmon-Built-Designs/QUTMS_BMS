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




HAL_StatusTypeDef bq769x0_reg_write_byte(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value) {
	uint8_t address = BQ_I2C_ADDRESS << 1 | 0;

	// TODO: do we need crc? datasheet says board doesn't support it?
	//uint8_t crc = 0x00;

	uint8_t data[2] = {reg, value};

	return HAL_I2C_Master_Transmit(hi2c, address, data, sizeof(data), TIMEOUT_MINUTE);
}

HAL_StatusTypeDef bq769x0_reg_write_bytes(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *buffer, size_t length) {
	uint8_t address = BQ_I2C_ADDRESS << 1 | 0;

	// TODO: do we need crc? datasheet says board doesn't support it?
	//uint8_t crc = 0x00;

	// transmit register as first byte, then all of buffer according to length

	uint8_t dataLength = 1+length;
	uint8_t *data = (uint8_t*)calloc(dataLength, sizeof(uint8_t));
	data[0] = reg;

	memcpy(data+1, buffer, length);

	HAL_StatusTypeDef result = HAL_I2C_Master_Transmit(hi2c, address, data, dataLength, TIMEOUT_MINUTE);

	// clean up dynamic memory
	free(data);

	return result;
}


HAL_StatusTypeDef bq769x0_reg_read_byte(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *value) {
	uint8_t address = BQ_I2C_ADDRESS << 1 | 0;
	HAL_StatusTypeDef result = HAL_I2C_Master_Transmit(hi2c, address, &reg, 1, TIMEOUT_MINUTE);

	if (result != HAL_OK) {
		// unable to send read request correctly
		return result;
	} else {
		return HAL_I2C_Master_Receive(hi2c, address, value, 1, TIMEOUT_MINUTE);
	}
}

HAL_StatusTypeDef bq769x0_reg_read_bytes(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *buffer, size_t length) {
	uint8_t address = BQ_I2C_ADDRESS << 1 | 0;
	HAL_StatusTypeDef result = HAL_I2C_Master_Transmit(hi2c, address, &reg, 1, TIMEOUT_MINUTE);

	if (result != HAL_OK) {
		// unable to send read request correctly
		return result;
	} else {
		return HAL_I2C_Master_Receive(hi2c, address, buffer, length, TIMEOUT_MINUTE);
	}
}

HAL_StatusTypeDef bq769x0_read_voltage(I2C_HandleTypeDef *hi2c, int cell, uint16_t *voltage, uint8_t gain, uint8_t offset) {
	uint8_t reg = BQ769X0_REG_VC1_HI + cell * 2;
	uint8_t buffer[2];

	// FIXME
	HAL_StatusTypeDef ret = bq769x0_reg_read_bytes(hi2c, reg, buffer, 2);
	if(ret != HAL_OK) {
		//SYS_LOG_ERR("failed to read cell %d voltage", cell);
		return ret;
	}

	uint16_t adc_value = ((buffer[0] & 0b00111111) << 8) | buffer[1];
	*voltage = adc_value * (365+gain) / 1000 + offset;

	return 0;
}
