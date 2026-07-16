/**
  ******************************************************************************
  * @file    at24c02.h
  * @author  ram223
  * @brief   Драйвер EEPROM AT24C02 (2Kbit = 256 bytes) через I2C2
  ******************************************************************************
  */

#ifndef AT24C02_H
#define AT24C02_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

// Адрес AT24C02 по умолчанию (A0=A1=A2=GND)
#define AT24C02_DEFAULT_ADDRESS     (0x50 << 1)  // 7-bit to 8-bit

// Параметры памяти
#define AT24C02_SIZE                256   // 256 bytes (2Kbit)
#define AT24C02_PAGE_SIZE           8     // Page size для записи
#define AT24C02_WRITE_DELAY_MS      5     // Задержка после записи (таймаут цикла записи)

HAL_StatusTypeDef AT24C02_Init(uint8_t dev_address);
HAL_StatusTypeDef AT24C02_ReadByte(uint8_t dev_address, uint16_t mem_address, uint8_t *data);
HAL_StatusTypeDef AT24C02_ReadBytes(uint8_t dev_address, uint16_t mem_address, uint8_t *data, uint16_t size);
HAL_StatusTypeDef AT24C02_WriteByte(uint8_t dev_address, uint16_t mem_address, uint8_t data);
HAL_StatusTypeDef AT24C02_WriteBytes(uint8_t dev_address, uint16_t mem_address, uint8_t *data, uint16_t size);
HAL_StatusTypeDef AT24C02_WaitReady(uint8_t dev_address);

#endif /* AT24C02_H */
