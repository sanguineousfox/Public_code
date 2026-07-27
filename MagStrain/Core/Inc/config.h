/**
 * @file config.h
 * @brief Совместимый интерфейс конфигурации.
 *
 * Настройки Modbus хранятся единственным модулем params_storage через modbus.
 * Запись во Flash удалена, чтобы не было второго источника адреса устройства.
 */
#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

#define DEFAULT_DEVICE_ADDRESS  1U
#define MODBUS_BAUDRATE         19200U
#define DEBUG_BAUDRATE          115200U

void Config_Init(void);
uint8_t Config_GetAddress(void);
void Config_SetAddress(uint8_t new_address);
void Config_SaveToFlash(void);
void Config_LoadFromFlash(void);
void Config_FactoryReset(void);

#endif /* CONFIG_H */
