#include "config.h"
#include <string.h>

// Глобальная конфигурация устройства
static DeviceConfig_t device_config;

// Магическое число
#define CONFIG_MAGIC_NUMBER     0x55AA55AA

/**
  * @brief  Расчет CRC32
  */
static uint32_t CalculateCRC32(const uint8_t* data, uint32_t length)
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

/**
  * @brief  Инициализация конфигурации
  */
void Config_Init(void)
{
    // Загружаем конфигурацию из Flash
    Config_LoadFromFlash();
    
    // Проверяем валидность конфигурации
    uint32_t stored_crc = device_config.crc32;
    device_config.crc32 = 0;
    uint32_t calculated_crc = CalculateCRC32((uint8_t*)&device_config, sizeof(DeviceConfig_t));
    
    if (device_config.magic != CONFIG_MAGIC_NUMBER || 
        stored_crc != calculated_crc ||
        device_config.device_address < 1 || 
        device_config.device_address > 247) {
        
        // Конфигурация невалидна, используем значения по умолчанию
        Config_FactoryReset();
    }
}

/**
  * @brief  Получение текущего адреса устройства
  */
uint8_t Config_GetAddress(void)
{
    return device_config.device_address;
}

/**
  * @brief  Установка нового адреса устройства
  */
void Config_SetAddress(uint8_t new_address)
{
    if (new_address >= 1 && new_address <= 247) {
        device_config.device_address = new_address;
        
        // Пересчитываем CRC
        device_config.crc32 = 0;
        device_config.crc32 = CalculateCRC32((uint8_t*)&device_config, sizeof(DeviceConfig_t));
        
        // Сохраняем в Flash
        Config_SaveToFlash();
    }
}

/**
  * @brief  Сохранение конфигурации в Flash
  */
void Config_SaveToFlash(void)
{
    // Разблокируем Flash
    HAL_FLASH_Unlock();
    
    // Настраиваем стирание страницы
    FLASH_EraseInitTypeDef erase_init;
    uint32_t page_error;
    
    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.PageAddress = CONFIG_FLASH_ADDRESS;
    erase_init.NbPages = 1;
    
    // Стираем страницу
    HAL_FLASHEx_Erase(&erase_init, &page_error);
    
    // Записываем конфигурацию
    uint32_t* src = (uint32_t*)&device_config;
    uint32_t addr = CONFIG_FLASH_ADDRESS;
    
    for (uint32_t i = 0; i < sizeof(DeviceConfig_t); i += 4) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, *src);
        addr += 4;
        src++;
    }
    
    // Блокируем Flash
    HAL_FLASH_Lock();
}

/**
  * @brief  Загрузка конфигурации из Flash
  */
void Config_LoadFromFlash(void)
{
    DeviceConfig_t* flash_config = (DeviceConfig_t*)CONFIG_FLASH_ADDRESS;
    memcpy(&device_config, flash_config, sizeof(DeviceConfig_t));
}

/**
  * @brief  Сброс конфигурации к заводским настройкам
  */
void Config_FactoryReset(void)
{
    device_config.magic = CONFIG_MAGIC_NUMBER;
    device_config.device_address = DEFAULT_DEVICE_ADDRESS;
    memset(device_config.reserved, 0, sizeof(device_config.reserved));
    
    // Рассчитываем CRC
    device_config.crc32 = 0;
    device_config.crc32 = CalculateCRC32((uint8_t*)&device_config, sizeof(DeviceConfig_t));
    
    // Сохраняем
    Config_SaveToFlash();
}