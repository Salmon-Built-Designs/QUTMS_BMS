/*
 * bq769x0.h
 *
 *  Created on: 25Apr.,2020
 *      Author: Malora
 */

#ifndef BQ769X0_H_
#define BQ769X0_H_

#define BQ769X0_ADDR 0x08
#define BQ769X0_CFG_VALUE 0x19

// registers							// D7:        D6:        D5:        D4:        D3:        D2:        D1:        D0:
#define BQ769X0_REG_SYS_STAT 0x00		// CC_READY,  RSVD, DEVICE_XREADY,  OVRD_ALERT,UV,        OV,        SCD,       OCD
//    Cell Balance
#define BQ769X0_REG_CELLBAL1 0x01		// RSVD       RSVD       RSVD       CB<5:1>
#define BQ769X0_REG_CELLBAL2 0x02		// Only valid for BQ76930
//    Control registers
#define BQ769X0_REG_SYS_CTRL1 0x04
#define BQ769X0_REG_SYS_CTRL2 0x05
//    Protection
#define BQ769X0_REG_PROTECT1 0x06
#define BQ769X0_REG_PROTECT2 0x07
#define BQ769X0_REG_PROTECT3 0x08
//    ????
#define BQ769X0_REG_OV_TRIP 0x09
#define BQ769X0_REG_UV_TRIP 0x0A
#define BQ769X0_REG_VC1_HI 0x0C

#define BQ769X0_REG_CFG 0x0B
#define BQ769X0_REG_BAT_HI 0x2A
#define BQ769X0_REG_BAT_LO 0x2B
#define BQ769X0_REG_CC_HI 0x32
#define BQ769X0_REG_CC_LO 0x33
//    ADC Conf & Offset
#define BQ769X0_REG_ADCGAIN1 0x50
#define BQ769X0_REG_ADCOFFSET 0x51
#define BQ769X0_REG_ADCGAIN2 0x59
//    Voltage Control
#define BQ769X0_VC1_HI_BYTE 0x0C
#define BQ769X0_VC1_LO_BYTE 0x0D
#define BQ769X0_VC2_HI_BYTE 0x0E
#define BQ769X0_VC2_LO_BYTE 0x0F
#define BQ769X0_VC3_HI_BYTE 0x10
#define BQ769X0_VC3_LO_BYTE 0x11
#define BQ769X0_VC4_HI_BYTE 0x12
#define BQ769X0_VC4_LO_BYTE 0x13
#define BQ769X0_VC5_HI_BYTE 0x14
#define BQ769X0_VC5_LO_BYTE 0x15
#define BQ769X0_VC6_HI_BYTE 0x16
#define BQ769X0_VC6_LO_BYTE 0x17
#define BQ769X0_VC7_HI_BYTE 0x18
#define BQ769X0_VC7_LO_BYTE 0x19
#define BQ769X0_VC8_HI_BYTE 0x1A
#define BQ769X0_VC8_LO_BYTE 0x1B
#define BQ769X0_VC9_HI_BYTE 0x1C
#define BQ769X0_VC9_LO_BYTE 0x1D
#define BQ769X0_VC10_HI_BYTE 0x1E
#define BQ769X0_VC10_LO_BYTE 0x1F

// bits
#define BQ769X0_REG_CTRL1_ADC_EN 4
// Status Bits
#define BQ769X0_REG_STAT_OCD 0
#define BQ769X0_REG_STAT_SCD 1
#define BQ769X0_REG_STAT_OV 2
#define BQ769X0_REG_STAT_UV 3
#define BQ769X0_REG_STAT_OVRD_ALERT 4
#define BQ769X0_REG_STAT_DEVICE_XREADY 5
#define BQ769X0_REG_STAT_CC_READY 7
// Control bits
#define BQ769X0_REG_CTRL2_CHG_ON 0
#define BQ769X0_REG_CTRL2_DSG_ON 1
#define BQ769X0_REG_CTRL2_CC_ONESHOT 5
#define BQ769X0_REG_CTRL2_CC_EN 6
#define BQ769X0_REG_CTRL2_DELAY_DIS 7
// Protection 1 bits
#define BQ769X0_REG_PROTECT1_SCD_T 0
#define BQ769X0_REG_PROTECT1_SCD_D 3
#define BQ769X0_REG_PROTECT1_RSNS 7
// Protection 2 bits
#define BQ769X0_REG_PROTECT2_OCD_T 0
#define BQ769X0_REG_PROTECT2_OCD_D 4
// Protection 3 bits
#define BQ769X0_REG_PROTECT3_OV_D 4
#define BQ769X0_REG_PROTECT3_UV_D 6

//#define BQ769X0_STATUS_OCD 0
//#define BQ769X0_STATUS_SCD 1
//#define BQ769X0_STATUS_OV 2
//#define BQ769X0_STATUS_UV 3

#define EINVAL  22
#define CRC_KEY 7

typedef struct {
  uint16_t uvp; // mV
  uint16_t ovp; // mV
} bq769x0_config_t;

int bq769x0_init(char *i2c_device);
void bq769x0_boot(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
int bq769x0_configure(I2C_HandleTypeDef hi2c, bq769x0_config_t config);
HAL_StatusTypeDef bq769x0_read_voltage(I2C_HandleTypeDef hi2c, int cell, uint16_t *voltage);
HAL_StatusTypeDef bq769x0_read_current(I2C_HandleTypeDef hi2c, int16_t *current);
HAL_StatusTypeDef bq769x0_read_status(I2C_HandleTypeDef hi2c, uint8_t *status);
HAL_StatusTypeDef bq769x0_clear_status(I2C_HandleTypeDef hi2c, uint8_t bit);
uint8_t bq769x0_status_error(uint8_t status);
uint8_t bq769x0_cc_ready(uint8_t status);
//int bq769x0_enable_discharging(void);
//int bq769x0_enable_charging(void);
//int bq769x0_balance_cell(int8_t cell);

#endif /* BQ769X0_H_ */
