/*
 * bq769x0.c
 *
 *  Created on: Sep 11, 2020
 *      Author: Calvin Johnson
 */

// BQ chip is bq7693006DBT
#include "bq769x0.h"
#include <stdlib.h>
#include <string.h>

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
	result = bq769x0_reg_read_byte(hi2c, BQ_ADCOFFSET, &reading);
	if (result != HAL_OK) {
		*gain = 0;
		*offset = 0;
		return result;
	}

	*offset = reading;
	reading = 0;

	// ADCGAIN2[7:5] = GAIN[2:0]
	result = bq769x0_reg_read_byte(hi2c, BQ_ADCGAIN2, &reading);
	if (result != HAL_OK) {
		*gain = 0;
		*offset = 0;
		return result;
	}

	*gain = (reading & 0b11100000) >> 5;
	reading = 0;

	// ADCGAIN1[3:2] = GAIN[4:3]
	result = bq769x0_reg_read_byte(hi2c, BQ_ADCGAIN1, &reading);
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

	return HAL_OK;
}

HAL_StatusTypeDef bq769x0_read_pack_voltage(I2C_HandleTypeDef *hi2c,
		int total_cells, uint16_t *voltage) {
	if (adc_voltage_gain == 0 || adc_voltage_offset == 0) {
		bq769x0_read_gain_and_offset(hi2c, &adc_voltage_gain,
				&adc_voltage_offset);
	}

	uint8_t buffer = 0;

	// FIXME
	HAL_StatusTypeDef ret = bq769x0_reg_read_byte(hi2c, BQ_BAT_HI, &buffer);
	if (ret != HAL_OK) {
		//SYS_LOG_ERR("failed to read cell %d voltage", cell);
		*voltage = 0;
		return ret;
	}
	uint16_t rawPackVoltage = buffer << 8;
	ret = bq769x0_reg_read_byte(hi2c, BQ_BAT_LO, &buffer);
	if (ret != HAL_OK) {
		//SYS_LOG_ERR("failed to read cell %d voltage", cell);
		*voltage = 0;
		return ret;
	}
	rawPackVoltage |= buffer;

	*voltage = (4 * adc_voltage_gain * rawPackVoltage)
			+ (total_cells * adc_voltage_offset);

	return HAL_OK;

}

HAL_StatusTypeDef bq769x0_set_under_voltage(I2C_HandleTypeDef *hi2c, uint16_t under_voltage) {
	if (adc_voltage_gain == 0 || adc_voltage_offset == 0) {
			bq769x0_read_gain_and_offset(hi2c, &adc_voltage_gain,
					&adc_voltage_offset);
		}

	uint16_t UV_TRIP_FULL = (under_voltage - adc_voltage_offset) / adc_voltage_gain;
	uint8_t UV_TRIP = ((UV_TRIP_FULL >> 4) & 0b11111111);

	return bq769x0_reg_write_byte(hi2c, BQ_UV_TRIP, UV_TRIP);
}

HAL_StatusTypeDef bq769x0_set_over_voltage(I2C_HandleTypeDef *hi2c, uint16_t over_voltage) {
	if (adc_voltage_gain == 0 || adc_voltage_offset == 0) {
			bq769x0_read_gain_and_offset(hi2c, &adc_voltage_gain,
					&adc_voltage_offset);
		}

	uint16_t OV_TRIP_FULL = (over_voltage - adc_voltage_offset) / adc_voltage_gain;
	uint8_t OV_TRIP = ((OV_TRIP_FULL >> 4) & 0b11111111);

	return bq769x0_reg_write_byte(hi2c, BQ_OV_TRIP, OV_TRIP);
}

HAL_StatusTypeDef bq769x0_set_DSG(I2C_HandleTypeDef *hi2c, uint8_t value) {
	uint8_t currentReg = 0;
	HAL_StatusTypeDef result = bq769x0_reg_read_byte(hi2c, BQ_SYS_CTRL2,
			&currentReg);

	if (result != HAL_OK) {
		return result;
	}

	// clear DSG_ON
	currentReg &= ~(1<<1);

	// set DSG VALUE
	currentReg |= ((1 & value) << 1);

	// write back to register
	return bq769x0_reg_write_byte(hi2c, BQ_SYS_CTRL2, currentReg);
}

HAL_StatusTypeDef bq769x0_set_CHG(I2C_HandleTypeDef *hi2c, uint8_t value) {
	uint8_t currentReg = 0;
	HAL_StatusTypeDef result = bq769x0_reg_read_byte(hi2c, BQ_SYS_CTRL2,
			&currentReg);

	if (result != HAL_OK) {
		return result;
	}

	// clear CHG_ON
	currentReg &= ~(1<<0);

	// set CHG VALUE
	currentReg |= ((1 & value) << 0);

	// write back to register
	return bq769x0_reg_write_byte(hi2c, BQ_SYS_CTRL2, currentReg);
}

HAL_StatusTypeDef bq769x0_set_CC_mode(I2C_HandleTypeDef *hi2c, uint8_t mode) {
	uint8_t currentReg = 0;
	HAL_StatusTypeDef result = bq769x0_reg_read_byte(hi2c, BQ_SYS_CTRL2,
				&currentReg);

	if (result != HAL_OK) {
			return result;
		}

	// clear CC_EN and CC_ONESHOT
	currentReg &= ~(1<<5);
	currentReg &= ~(1<<6);

	// set mode
	if(mode == BQ_CC_ALWAYSON) {
		currentReg |= 1<<6;
	}
	else {
		currentReg |= 1<<5;
	}

	// write back to register
	return bq769x0_reg_write_byte(hi2c, BQ_SYS_CTRL2, currentReg);
}

HAL_StatusTypeDef bq769x0_read_CC(I2C_HandleTypeDef *hi2c, uint16_t *voltage) {
	uint8_t buffer = 0;

	HAL_StatusTypeDef ret = bq769x0_reg_read_byte(hi2c, BQ_CC_HI, &buffer);
		if (ret != HAL_OK) {
			*voltage = 0;
			return ret;
		}

	uint16_t adc_value = buffer << 8;

	ret = bq769x0_reg_read_byte(hi2c, BQ_CC_LO, &buffer);
	if (ret != HAL_OK) {
		*voltage = 0;
		return ret;
	}

	adc_value |= buffer;

	// converting to two`s complement
	if(adc_value & (1<<15)) {
		adc_value = (~adc_value + 1) * -1;
	}

	*voltage = adc_value * 8.44; // (currently in micro-volts)

	return HAL_OK;
}


//Not finished, just need to set BQ_CELLBAL1 as reg1 and BQ_CELLBAL2 as reg2

/**
 * cell_num - 0-9
 *
 */
HAL_StatusTypeDef bq769x0_set_cell_balancing(I2C_HandleTypeDef *hi2c,
		uint8_t cell_num, uint8_t state) {

	state = state & 1;
	uint8_t reg = 0;
	uint8_t bit_num;

	if (cell_num < 5) {
		reg = BQ_CELLBAL1;
		bit_num = cell_num;
	} else if (cell_num < 10) {
		reg = BQ_CELLBAL2;
		bit_num = cell_num - 5;
	}

	uint8_t cur_value = 0;
	HAL_StatusTypeDef res = bq769x0_reg_read_byte(hi2c, reg, &cur_value);

	if (res != HAL_OK) {
		return res;
	}

	// clear current value
	cur_value = cur_value & ~(1<<bit_num);

	// set bit
	cur_value = cur_value | (state << bit_num);

	res = bq769x0_reg_write_byte(hi2c, reg, cur_value);

	return res;
}

HAL_StatusTypeDef bq769x0_reset_cell_balancing(I2C_HandleTypeDef *hi2c) {
	HAL_StatusTypeDef res = bq769x0_reg_write_byte(hi2c, BQ_CELLBAL1, 0);
	res = bq769x0_reg_write_byte(hi2c, BQ_CELLBAL2, 0);

	return res;

}


