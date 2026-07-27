/**
 * @file    at24c64.h
 * @brief   Низкоуровневый драйвер EEPROM AT24C64 через I2C2.
 */
#ifndef AT24C64_H
#define AT24C64_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdint.h>

#define AT24C64_DEFAULT_ADDRESS      (0x51U << 1)
#define AT24C64_SIZE_BYTES           8192U
#define AT24C64_PAGE_SIZE_BYTES      32U
#define AT24C64_READY_TIMEOUT_MS     25U
#define AT24C64_IO_TIMEOUT_MS        100U

#define AT24C64_WP_PIN               GPIO_PIN_8
#define AT24C64_WP_PORT              GPIOB

HAL_StatusTypeDef AT24C64_Init(uint8_t device_address);
HAL_StatusTypeDef AT24C64_IsReady(uint8_t device_address);
/* Быстрая проверка готовности: один опрос с коротким таймаутом. */
HAL_StatusTypeDef AT24C64_PollReady(uint8_t device_address);
/* Запуск записи одной страницы без ожидания внутреннего цикла EEPROM. */
HAL_StatusTypeDef AT24C64_WritePageBegin(uint8_t device_address,
                                         uint16_t memory_address,
                                         const uint8_t *data,
                                         uint16_t size);

HAL_StatusTypeDef AT24C64_ReadByte(uint8_t device_address,
                                   uint16_t memory_address,
                                   uint8_t *value);
HAL_StatusTypeDef AT24C64_ReadBytes(uint8_t device_address,
                                    uint16_t memory_address,
                                    uint8_t *data,
                                    uint16_t size);

HAL_StatusTypeDef AT24C64_WriteByte(uint8_t device_address,
                                    uint16_t memory_address,
                                    uint8_t value);
HAL_StatusTypeDef AT24C64_WriteBytes(uint8_t device_address,
                                     uint16_t memory_address,
                                     const uint8_t *data,
                                     uint16_t size);

#ifdef __cplusplus
}
#endif

#endif /* AT24C64_H */
