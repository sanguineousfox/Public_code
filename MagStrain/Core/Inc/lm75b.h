/**
  ******************************************************************************
  * @file    lm75b.h
  * @author  ram223
  * @brief   Драйвер температурного датчика LM75B (CJMCU75) через I2C2
  ******************************************************************************
  */

#ifndef LM75B_H
#define LM75B_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

// Адрес LM75B по умолчанию (A0=A1=A2=GND)
#define LM75B_DEFAULT_ADDRESS       (0x48 << 1)  // 7-bit to 8-bit

// Регистры LM75B
#define LM75B_REG_TEMP              0x00  // Temperature register
#define LM75B_REG_CONF              0x01  // Configuration register
#define LM75B_REG_THYST             0x02  // Hysteresis register
#define LM75B_REG_TOS               0x03  // OS register

HAL_StatusTypeDef LM75B_Init(uint8_t dev_address);
HAL_StatusTypeDef LM75B_ReadTemperature(uint8_t dev_address, float *temp);
HAL_StatusTypeDef LM75B_ReadRawTemperature(uint8_t dev_address, uint16_t *raw_temp);
HAL_StatusTypeDef LM75B_ReadConfig(uint8_t dev_address, uint8_t *config);
HAL_StatusTypeDef LM75B_WriteConfig(uint8_t dev_address, uint8_t config);

#endif /* LM75B_H */
