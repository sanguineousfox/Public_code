/**
@file    at24c64.h
@brief   Драйвер EEPROM AT24C64 через I2C2
*/
#ifndef AT24C64_H
#define AT24C64_H
#include "stm32f1xx_hal.h"
#include <stdint.h>

#define AT24C64_DEFAULT_ADDRESS     (0x51 << 1)
#define AT24C64_SIZE                8192
#define AT24C64_PAGE_SIZE           32
#define AT24C64_WRITE_DELAY_MS      5

#define AT24C64_WP_PIN              GPIO_PIN_8
#define AT24C64_WP_PORT             GPIOB

#define AT24C64_WP_ENABLE_WRITE()   HAL_GPIO_WritePin(AT24C64_WP_PORT, AT24C64_WP_PIN, GPIO_PIN_RESET)
#define AT24C64_WP_ENABLE_PROTECT() HAL_GPIO_WritePin(AT24C64_WP_PORT, AT24C64_WP_PIN, GPIO_PIN_SET)

#define EEPROM_MAGIC_ADDR           0x0000
#define EEPROM_MAGIC_VALUE          0xDEADBEEF
#define EEPROM_FLOAT_BASE           0x0010
#define EEPROM_INT_BASE             0x0100
#define EEPROM_REGS_BASE            0x0200

HAL_StatusTypeDef AT24C64_Init(uint8_t dev_address);
HAL_StatusTypeDef AT24C64_ReadByte(uint8_t dev_address, uint16_t mem_address, uint8_t *data);
HAL_StatusTypeDef AT24C64_ReadBytes(uint8_t dev_address, uint16_t mem_address, uint8_t *data, uint16_t size);
HAL_StatusTypeDef AT24C64_WriteByte(uint8_t dev_address, uint16_t mem_address, uint8_t data);
HAL_StatusTypeDef AT24C64_WriteBytes(uint8_t dev_address, uint16_t mem_address, uint8_t *data, uint16_t size);
HAL_StatusTypeDef AT24C64_WaitReady(uint8_t dev_address);
HAL_StatusTypeDef AT24C64_SaveFloatParam(uint16_t mb_addr, float value);
HAL_StatusTypeDef AT24C64_LoadFloatParam(uint16_t mb_addr, float *value);
HAL_StatusTypeDef AT24C64_SaveIntParam(uint16_t mb_addr, uint16_t value);
HAL_StatusTypeDef AT24C64_LoadIntParam(uint16_t mb_addr, uint16_t *value);
HAL_StatusTypeDef AT24C64_SaveAllRegisters(uint16_t *regs, uint16_t count);
HAL_StatusTypeDef AT24C64_LoadAllRegisters(uint16_t *regs, uint16_t count);
HAL_StatusTypeDef AT24C64_Format(void);
uint8_t AT24C64_IsFormatted(void);

#endif
