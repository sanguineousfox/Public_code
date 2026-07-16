/**
  ******************************************************************************
  * @file    at24c02.c
  * @author  ram223
  * @brief   Драйвер EEPROM AT24C02 (2Kbit) через I2C2
  ******************************************************************************
  */

#include "at24c02.h"
#include "i2c_config.h"  // Для доступа к hi2c2
#include "main.h"        // Для USART2_Print

/**
  * @brief Инициализация AT24C02 (проверка доступности устройства)
  */
HAL_StatusTypeDef AT24C02_Init(uint8_t dev_address)
{
    // Проверяем доступность путём чтения первого байта
    uint8_t dummy = 0;
    return HAL_I2C_Mem_Read(&hi2c2, dev_address, 0x00, I2C_MEMADD_SIZE_8BIT, &dummy, 1, 100);
}

/**
  * @brief Чтение одного байта из EEPROM
  */
HAL_StatusTypeDef AT24C02_ReadByte(uint8_t dev_address, uint16_t mem_address, uint8_t *data)
{
    if (data == NULL || mem_address >= AT24C02_SIZE)
        return HAL_ERROR;
    
    return HAL_I2C_Mem_Read(&hi2c2, dev_address, (uint16_t)(mem_address & 0x00FF),
                            I2C_MEMADD_SIZE_8BIT, data, 1, 100);
}

/**
  * @brief Чтение нескольких байт из EEPROM
  */
HAL_StatusTypeDef AT24C02_ReadBytes(uint8_t dev_address, uint16_t mem_address, uint8_t *data, uint16_t size)
{
    if (data == NULL || mem_address >= AT24C02_SIZE ||
        (mem_address + size) > AT24C02_SIZE) 
        return HAL_ERROR;
    
    return HAL_I2C_Mem_Read(&hi2c2, dev_address, (uint16_t)(mem_address & 0x00FF),
                            I2C_MEMADD_SIZE_8BIT, data, size, 100);
}

/**
  * @brief Запись одного байта в EEPROM
  */
HAL_StatusTypeDef AT24C02_WriteByte(uint8_t dev_address, uint16_t mem_address, uint8_t data)
{
    if (mem_address >= AT24C02_SIZE)
        return HAL_ERROR;
    
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c2, dev_address,
                                                 (uint16_t)(mem_address & 0x00FF), 
                                                 I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    if (status == HAL_OK)
    {
        // Ждём завершения внутреннего цикла записи EEPROM
        HAL_Delay(AT24C02_WRITE_DELAY_MS);
    }
    return status;
}

/**
  * @brief Запись нескольких байт в EEPROM (с обработкой границ страниц)
  */
HAL_StatusTypeDef AT24C02_WriteBytes(uint8_t dev_address, uint16_t mem_address, uint8_t *data, uint16_t size)
{
    if (data == NULL || mem_address >= AT24C02_SIZE ||
        (mem_address + size) > AT24C02_SIZE || size == 0) 
        return HAL_ERROR;
    
    uint16_t written = 0;
    uint16_t addr = mem_address;
    
    while (written < size)
    {
        // Вычисляем оставшееся место до конца текущей страницы
        uint16_t page_offset = addr % AT24C02_PAGE_SIZE;
        uint16_t space_in_page = AT24C02_PAGE_SIZE - page_offset;
        uint16_t chunk_size = (size - written) < space_in_page ? (size - written) : space_in_page;
        
        // Записываем чанк
        HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c2, dev_address,
                                                     (uint16_t)(addr & 0x00FF), 
                                                     I2C_MEMADD_SIZE_8BIT, 
                                                     &data[written], chunk_size, 100);
        if (status != HAL_OK)
            return status;
        
        // Ждём завершения цикла записи
        HAL_Delay(AT24C02_WRITE_DELAY_MS);
        
        written += chunk_size;
        addr += chunk_size;
    }
    
    return HAL_OK;
}

/**
  * @brief Активное ожидание готовности EEPROM (поллинг)
  */
HAL_StatusTypeDef AT24C02_WaitReady(uint8_t dev_address)
{
    uint32_t timeout = 10;  // максимум 10 попыток
    while (timeout--)
    {
        // Пытаемся выполнить "dummy" чтение
        uint8_t dummy = 0;
        if (HAL_I2C_Mem_Read(&hi2c2, dev_address, 0x00, I2C_MEMADD_SIZE_8BIT, &dummy, 1, 50) == HAL_OK)
        {
            return HAL_OK;  // EEPROM готова
        }
        HAL_Delay(1);
    }
    return HAL_ERROR;  // Таймаут
}
