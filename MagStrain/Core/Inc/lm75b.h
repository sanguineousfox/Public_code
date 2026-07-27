/**
@file    lm75b.h
@brief   Драйвер температурного датчика LM75B через I2C2
*/
#ifndef LM75B_H
#define LM75B_H
#include "stm32f1xx_hal.h"
#include <stdint.h>

#define LM75B_DEFAULT_ADDRESS       (0x48 << 1)
#define LM75B_REG_TEMP              0x00
#define LM75B_REG_CONF              0x01
#define LM75B_REG_THYST             0x02
#define LM75B_REG_TOS               0x03

HAL_StatusTypeDef LM75B_Init(uint8_t dev_address);
HAL_StatusTypeDef LM75B_ReadTemperature(uint8_t dev_address, float *temp);
HAL_StatusTypeDef LM75B_ReadRawTemperature(uint8_t dev_address, uint16_t *raw_temp);
HAL_StatusTypeDef LM75B_ReadConfig(uint8_t dev_address, uint8_t *config);
HAL_StatusTypeDef LM75B_WriteConfig(uint8_t dev_address, uint8_t config);

#endif
