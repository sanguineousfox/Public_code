/** @file config.c @brief Совместимый фасад над настройками Modbus. */
#include "config.h"
#include "modbus.h"

void Config_Init(void)
{
    /* ModBus_Init() выполняет загрузку единственного хранилища параметров. */
}

uint8_t Config_GetAddress(void)
{
    return ModBus_GetDeviceAddress();
}

void Config_SetAddress(uint8_t new_address)
{
    if (new_address >= 1U && new_address <= 247U) {
        ModBus_SetParameter_Int(MB_ADDR_MB_ADDR_SET, new_address);
        ModBus_SetParameter_Float(MB_ADDR_DEVICE_ADDR, (float)new_address);
    }
}

void Config_SaveToFlash(void)
{
    ModBus_ForceSaveToEEPROM();
}

void Config_LoadFromFlash(void)
{
    /* Оставлено для совместимости; загрузка выполняется ModBus_Init(). */
}

void Config_FactoryReset(void)
{
    Config_SetAddress(DEFAULT_DEVICE_ADDRESS);
    ModBus_SetParameter_Int(MB_ADDR_MB_BAUD_SET, MODBUS_BAUDRATE);
    ModBus_SetParameter_Float(MB_ADDR_BAUD_RATE, (float)MODBUS_BAUDRATE);
    ModBus_ForceSaveToEEPROM();
}
