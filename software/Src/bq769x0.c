/*
 * bq769x0.c
 *
 *  Created on: 25Apr.,2020
 *      Author: Malora
 */

//#include "i2c.h"
#include <string.h>
#include "stm32f0xx_hal.h"

#include "bq769x0.h"

// used to be an int8_t;
static uint8_t adc_offset;
static uint16_t adc_gain;

HAL_StatusTypeDef bq769x0_reg_write_byte(I2C_HandleTypeDef hi2c,
										 uint8_t reg, uint8_t value);
HAL_StatusTypeDef bq769x0_reg_read(I2C_HandleTypeDef hi2c,
				uint8_t reg, uint8_t *buffer, size_t length);
HAL_StatusTypeDef bq769x0_reg_read_byte(I2C_HandleTypeDef hi2c,
				uint8_t reg, uint8_t *value);
HAL_StatusTypeDef bq769x0_reg_update_byte(I2C_HandleTypeDef hi2c,
				uint8_t reg, uint8_t mask, uint8_t value);
uint8_t crc8(const uint8_t *buffer, size_t length, unsigned char key);
HAL_StatusTypeDef i2c_burst_read(I2C_HandleTypeDef hi2c, uint16_t addr,
		uint8_t reg, uint8_t value, uint8_t *buffer, size_t length);

int bq769x0_init(char *i2c_device) {
//  int rc;
//
//  SYS_LOG_INF("initializing...");
//
//  i2c = device_get_binding(i2c_device);
//
//  if (i2c == NULL) {
//    SYS_LOG_ERR("could not find %s", i2c_device);
//    return -EINVAL;
//  }
//
//  static union dev_config i2c_cfg = {
//    .raw = 0,
//    .bits = {
//      .use_10_bit_addr = 0,
//      .is_master_device = 1,
//      .speed = I2C_SPEED_STANDARD,
//    },
//  };
//
//  rc = i2c_configure(i2c, i2c_cfg.raw);
//  if (rc < 0) {
//    SYS_LOG_ERR("could not configure I2C_0");
//    return rc;
//  }
//
//  SYS_LOG_INF("initialized");
	return 0;
}

void bq769x0_boot(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
	//SYS_LOG_INF("booting...");
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
	// datasheet says wait at least 2ms
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);

	//gpio_pin_configure(gpio, boot_pin, GPIO_DIR_IN);

	// datasheets says wait at least 10ms
	HAL_Delay(10);

	//SYS_LOG_INF("booted");
	//return 0;
}

int bq769x0_configure(I2C_HandleTypeDef hi2c, bq769x0_config_t config) {
	//SYS_LOG_INF("configuring...");
	// test I2C communication
	uint8_t cfg;

	HAL_StatusTypeDef ret = bq769x0_reg_write_byte(hi2c, BQ769X0_REG_CFG, BQ769X0_CFG_VALUE);
	if ( ret != HAL_OK ) {
		//SYS_LOG_ERR("failed to write CFG register");
		return ret;
	} else {
  		ret = bq769x0_reg_read_byte(hi2c, BQ769X0_REG_CFG, &cfg);
  		if ( ret != HAL_OK ) {
  			//SYS_LOG_ERR("failed to read CFG register");
  			return ret;
		} else {
			if (cfg != BQ769X0_CFG_VALUE) {
				//SYS_LOG_ERR("invalid CFG value");
				return -EINVAL;
			}
			// enable ADC
			uint8_t adc_en = 1 << BQ769X0_REG_CTRL1_ADC_EN;
			ret = bq769x0_reg_update_byte(hi2c, BQ769X0_REG_SYS_CTRL1, adc_en, adc_en);
			if ( ret != HAL_OK ) {
				//SYS_LOG_ERR("failed to enable ADC");
				return 0;
			}
			// enable continous current monitoring
			uint8_t cc_en = 1 << BQ769X0_REG_CTRL2_CC_EN;
			ret = bq769x0_reg_update_byte(hi2c, BQ769X0_REG_SYS_CTRL2, cc_en, cc_en);
			if ( ret != HAL_OK ) {
				//SYS_LOG_ERR("failed to enable CC");
				return 0;
			}
			// read ADC offset
			ret = bq769x0_reg_read_byte(hi2c, BQ769X0_REG_ADCOFFSET, &adc_offset);
			if ( ret != HAL_OK ) {
				//SYS_LOG_ERR("failed to read ADCOFFSET");
				return 0;
			}

			// read ADC gain
			uint8_t adc_gain1;
			uint8_t adc_gain2;
			ret = bq769x0_reg_read_byte(hi2c, BQ769X0_REG_ADCGAIN1, &adc_gain1);
			if ( ret != HAL_OK ) {
				//SYS_LOG_ERR("failed to read ADCGAIN1");
				return 0;
			}
			ret = bq769x0_reg_read_byte(hi2c, BQ769X0_REG_ADCGAIN2, &adc_gain2);
			if ( ret != HAL_OK ) {
				//SYS_LOG_ERR("failed to read ADCGAIN2");
				return 0;
			}
			adc_gain = 365 + (((adc_gain1 & 0b00001100) << 1) | ((adc_gain2 & 0b11100000) >> 5));

			// clear all alerts
			ret = bq769x0_reg_write_byte(hi2c, BQ769X0_REG_SYS_STAT, 0b10111111);
			if ( ret != HAL_OK ) {
				//SYS_LOG_ERR("failed to clear alerts");
				return 0;
			}

			// set under-voltage protection
			uint8_t uvp = ((((config.uvp - adc_offset) * 1000 / adc_gain) >> 4) & 0xFF) + 1;
			ret = bq769x0_reg_write_byte(hi2c, BQ769X0_REG_UV_TRIP, uvp);
			if ( ret != HAL_OK ) {
				//SYS_LOG_ERR("failed to write UV_TRIP");
				return 0;
			}

			// set over-voltage protection
			uint8_t ovp = (((config.ovp - adc_offset) * 1000 / adc_gain) >> 4) & 0xFF;
			ret = bq769x0_reg_write_byte(hi2c, BQ769X0_REG_OV_TRIP, ovp);
			if ( ret != HAL_OK ) {
				//SYS_LOG_ERR("failed to write OV_TRIP");
				return 0;
			}

			// set short-circuit protection
			uint8_t protect1 = 0;
			protect1 |= (1 << BQ769X0_REG_PROTECT1_RSNS); // set RSNS = 1
			protect1 |= (0x2 << BQ769X0_REG_PROTECT1_SCD_D); // 200uS delay
			protect1 |= (0x7 << BQ769X0_REG_PROTECT1_SCD_T); // 200mV across 0.005ohm = 40A
			ret = bq769x0_reg_write_byte(hi2c, BQ769X0_REG_PROTECT1, protect1);
			if ( ret != HAL_OK ) {
				//SYS_LOG_ERR("failed to write PROTECT1");
				return 0;
			}

			// set over-current discharge protection
			uint8_t protect2 = 0;
			protect2 |= (0x0 << BQ769X0_REG_PROTECT2_OCD_D); // 8ms delay
			protect2 |= (0x6 << BQ769X0_REG_PROTECT2_OCD_T); // 50mV across 0.005ohm = 10A
			ret = bq769x0_reg_write_byte(hi2c, BQ769X0_REG_PROTECT2, protect2);
			if ( ret != HAL_OK ) {
				//SYS_LOG_ERR("failed to write PROTECT2");
				return 0;
			}

			// set uv and ov delay
			uint8_t protect3 = 0;
			protect3 |= (0x1 << BQ769X0_REG_PROTECT3_UV_D); // 4s
			protect3 |= (0x1 << BQ769X0_REG_PROTECT3_OV_D); // 2s
			ret = bq769x0_reg_write_byte(hi2c, BQ769X0_REG_PROTECT3, protect3);
			if ( ret != HAL_OK ) {
				//SYS_LOG_ERR("failed to write PROTECT3");
				return 0;
			}

			//SYS_LOG_INF("configured");
			return 1;
		}
  	}
}

HAL_StatusTypeDef bq769x0_read_voltage(I2C_HandleTypeDef hi2c, int cell, uint16_t *voltage) {
	uint8_t reg = BQ769X0_REG_VC1_HI + cell * 2;
	uint8_t buffer[2];

	// FIXME
	HAL_StatusTypeDef ret = bq769x0_reg_read(hi2c, reg, buffer, 2);
	if(ret != HAL_OK) {
		//SYS_LOG_ERR("failed to read cell %d voltage", cell);
		return ret;
	}

	uint16_t adc_value = ((buffer[0] & 0b00111111) << 8) | buffer[1];
	*voltage = adc_value * adc_gain / 1000 + adc_offset;

	return 0;
}

HAL_StatusTypeDef bq769x0_read_current(I2C_HandleTypeDef hi2c, int16_t *current) {
	HAL_StatusTypeDef ret;

	// read CC
	uint8_t msb;
	uint8_t lsb;

	ret = bq769x0_reg_read_byte(hi2c, BQ769X0_REG_CC_HI, &msb);
	if (ret != HAL_OK) {
		//SYS_LOG_ERR("failed to read CC_HI");
		return ret;
	}

	ret = bq769x0_reg_read_byte(hi2c, BQ769X0_REG_CC_LO, &lsb);
	if (ret != HAL_OK) {
		//SYS_LOG_ERR("failed to read CC_LO");
		return ret;
	}

	uint16_t adc_value = (msb << 8) | lsb;
	*current = (int16_t) adc_value * 8.44 / 5.0;

	// clear CC_READY
	uint8_t cc_ready = 1 << BQ769X0_REG_STAT_CC_READY;
	ret = bq769x0_reg_write_byte(hi2c, BQ769X0_REG_SYS_STAT, cc_ready);
	if (ret != HAL_OK) {
		//SYS_LOG_ERR("failed to clear CC_READY");
		return ret;
	}

	return HAL_OK;
}

HAL_StatusTypeDef bq769x0_read_status(I2C_HandleTypeDef hi2c, uint8_t *status) {
	HAL_StatusTypeDef ret = bq769x0_reg_read_byte(hi2c, BQ769X0_REG_SYS_STAT, status);
	if (ret != HAL_OK) {
		//SYS_LOG_ERR("failed to read status");
		return ret;
	}
	return HAL_OK;
}

HAL_StatusTypeDef bq769x0_clear_status(I2C_HandleTypeDef hi2c, uint8_t bit) {
	uint8_t mask = 1 << bit;
	HAL_StatusTypeDef ret = bq769x0_reg_write_byte(hi2c, BQ769X0_REG_SYS_STAT, mask);
	if (ret != HAL_OK) {
		//SYS_LOG_ERR("failed to clear status: %d", mask);
		return ret;
	}
	return 0;
}

uint8_t bq769x0_status_error(uint8_t status) {
  // mask out CC_READY (not considered an error)
  return status & 0b01111111;
}

uint8_t bq769x0_cc_ready(uint8_t status) {
  return status & 0b10000000;
}

//int bq769x0_enable_discharging(void) {
//  int rc;
//
//  uint8_t dsg_on = 1 << BQ769X0_REG_CTRL2_DSG_ON;
//  rc = bq769x0_reg_update_byte(BQ769X0_REG_SYS_CTRL2, dsg_on, dsg_on);
//  if (rc < 0) {
//    SYS_LOG_ERR("failed to enable discharging");
//    return rc;
//  }
//
//  return 0;
//}

//int bq769x0_enable_charging(void) {
//  int rc;
//
//  uint8_t chg_on = 1 << BQ769X0_REG_CTRL2_CHG_ON;
//  rc = bq769x0_reg_update_byte(BQ769X0_REG_SYS_CTRL2, chg_on, chg_on);
//  if (rc < 0) {
//    SYS_LOG_ERR("failed to enable charging");
//    return rc;
//  }
//
//  return 0;
//}

//int bq769x0_balance_cell(int8_t cell) {
//  int rc;
//
//  if (cell == -1) {
//    rc = bq769x0_reg_write_byte(BQ769X0_REG_CELLBAL1, 0);
//    if (rc < 0) {
//      SYS_LOG_ERR("failed to disable cell balancing");
//      return rc;
//    }
//  } else {
//    rc = bq769x0_reg_write_byte(BQ769X0_REG_CELLBAL1, 1 << cell);
//    if (rc < 0) {
//      SYS_LOG_ERR("failed to enable cell balancing");
//      return rc;
//    }
//  }
//
//  return 0;
//}

// helpers

HAL_StatusTypeDef bq769x0_reg_write_byte(I2C_HandleTypeDef hi2c,
										 uint8_t reg, uint8_t value) {
	uint8_t data[3] = { (BQ769X0_ADDR << 1) | 0, reg, value };
	uint8_t crc = crc8(data, sizeof(data), 0x07);

	uint8_t DataBuffer[3] = { reg, value, crc };
	return HAL_I2C_Master_Transmit(&hi2c, BQ769X0_ADDR,
						DataBuffer, sizeof(DataBuffer), HAL_MAX_DELAY);
}

HAL_StatusTypeDef bq769x0_reg_read(I2C_HandleTypeDef hi2c,
				uint8_t reg, uint8_t *buffer, size_t length) {
	uint8_t result[length * 2];
	unsigned char CRCInput[2];
	unsigned char CRCa = 0;

	HAL_StatusTypeDef ret = i2c_burst_read(hi2c, BQ769X0_ADDR, reg, 0, result, length * 2);
	//HAL_I2C_Master_Receive(&hi2c, BQ769X0_ADDR, result,
	//				length * 2, HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
		//SYS_LOG_ERR("failed to read register");
		return ret;
	} else {
		for (int i = 0; i < (length * 2); i += 2) {
			// TODO: check CRCs
			buffer[i / 2] = result[i];
			CRCInput[0] = (BQ769X0_ADDR << 1) + 1;
			CRCInput[1] = result[0];

			CRCa = crc8(CRCInput, 2, CRC_KEY);

			if (CRCa != ReadData[1])
				return -1;

			*buffer = result[0];
		}
		return 0;
	}
}

HAL_StatusTypeDef bq769x0_reg_read_byte(I2C_HandleTypeDef hi2c,
					uint8_t reg, uint8_t *value) {
  return bq769x0_reg_read(hi2c, reg, value, 1);
}

HAL_StatusTypeDef bq769x0_reg_update_byte(I2C_HandleTypeDef hi2c,
				uint8_t reg, uint8_t mask, uint8_t value) {
	uint8_t old_value, new_value;
	HAL_StatusTypeDef ret = bq769x0_reg_read_byte(hi2c, reg, &old_value);
	if(ret != HAL_OK) return ret;
	else {
		new_value = (old_value & ~mask) | (value & mask);
		if (new_value == old_value) return 0;
		return bq769x0_reg_write_byte(hi2c, reg, new_value);
	}
}

uint8_t crc8(const uint8_t *buffer, size_t length, unsigned char key) {
  uint8_t crc = 0;

  for (int i = 0; i < length; i++) {
    uint8_t data = crc ^ buffer[i];

    for (int i = 0; i < 8; i++ ) {
      if (data & 0x80) {
        data <<= 1;
        data ^= key;
      } else {
        data <<= 1;
      }
    }

    crc = data;
  }

  return crc;
}

HAL_StatusTypeDef i2c_burst_read(I2C_HandleTypeDef hi2c, uint16_t addr,
	uint8_t reg, uint8_t value, uint8_t *buffer, size_t length) {

	uint8_t data[3] = { (addr << 1) | 0, reg, value };
	uint8_t crc = crc8(data, sizeof(data), 0x07);

	uint8_t DataBuffer[3] = { reg, value, crc };

	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(&hi2c, addr,
						DataBuffer, sizeof(DataBuffer), HAL_MAX_DELAY);
	if ( ret != HAL_OK ) return ret;
	else
		return HAL_I2C_Master_Receive(&hi2c, addr, buffer, length, HAL_MAX_DELAY);
}
