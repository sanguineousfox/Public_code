/**
@file    config.c
@brief   Реализация конфигурации устройства
*/
#include "config.h"
#include <string.h>

static DeviceConfig_t device_config;
#define CONFIG_MAGIC_NUMBER     0x55AA55AA

static uint32_t CalculateCRC32(const uint8_t *data, uint32_t length)
{
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint32_t j = 0; j < 8; j++) {
            if (crc & 0x00000001) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
        }
    }
    return ~crc;
}

void Config_Init(void)
{
    Config_LoadFromFlash();
    uint32_t stored_crc = device_config.crc32;
    device_config.crc32 = 0;
    uint32_t calculated_crc = CalculateCRC32((uint8_t*)&device_config, sizeof(DeviceConfig_t));
    if (device_config.magic != CONFIG_MAGIC_NUMBER ||
        stored_crc != calculated_crc ||
        device_config.device_address < 1 ||
        device_config.device_address > 247) {
        Config_FactoryReset();
    }
}

uint8_t Config_GetAddress(void)
{
    return device_config.device_address;
}

void Config_SetAddress(uint8_t new_address)
{
    if (new_address >= 1 && new_address <= 247) {
        device_config.device_address = new_address;
        device_config.crc32 = 0;
        device_config.crc32 = CalculateCRC32((uint8_t*)&device_config, sizeof(DeviceConfig_t));
        Config_SaveToFlash();
    }
}

void Config_SaveToFlash(void)
{
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef erase_init;
    uint32_t page_error;
    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.PageAddress = CONFIG_FLASH_ADDRESS;
    erase_init.NbPages = 1;
    HAL_FLASHEx_Erase(&erase_init, &page_error);
    uint32_t* src = (uint32_t*)&device_config;
    uint32_t addr = CONFIG_FLASH_ADDRESS;
    for (uint32_t i = 0; i < sizeof(DeviceConfig_t); i += 4) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, *src);
        addr += 4;
        src++;
    }
    HAL_FLASH_Lock();
}

void Config_LoadFromFlash(void)
{
    DeviceConfig_t *flash_config = (DeviceConfig_t*)CONFIG_FLASH_ADDRESS;
    memcpy(&device_config, flash_config, sizeof(DeviceConfig_t));
}

void Config_FactoryReset(void)
{
    device_config.magic = CONFIG_MAGIC_NUMBER;
    device_config.device_address = DEFAULT_DEVICE_ADDRESS;
    memset(device_config.reserved, 0, sizeof(device_config.reserved));
    device_config.crc32 = 0;
    device_config.crc32 = CalculateCRC32((uint8_t*)&device_config, sizeof(DeviceConfig_t));
    Config_SaveToFlash();
}
