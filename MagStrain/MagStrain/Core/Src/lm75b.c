/**
  ******************************************************************************
  * @file    lm75b.c
  * @author  ram223
  * @brief   Драйвер температурного датчика LM75B (CJMCU75) через I2C2
  ******************************************************************************
  */

#include "lm75b.h"
#include "i2c_config.h"  // Для доступа к hi2c2
#include "main.h"        // Для USART2_Print

/**
  * @brief Инициализация LM75B (проверка доступности устройства)
  */
HAL_StatusTypeDef LM75B_Init(uint8_t dev_address)
{
    // Проверяем доступность устройства путём чтения регистра температуры
    uint8_t dummy[2] = {0};
    return HAL_I2C_Mem_Read(&hi2c2, dev_address, LM75B_REG_TEMP,
                            I2C_MEMADD_SIZE_8BIT, dummy, 2, 100);
}

/**
  * @brief Чтение "сырого" 16-битного значения температуры
  */
HAL_StatusTypeDef LM75B_ReadRawTemperature(uint8_t dev_address, uint16_t *raw_temp)
{
    if (raw_temp == NULL) return HAL_ERROR;
    
    uint8_t data[2] = {0};
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c2, dev_address, LM75B_REG_TEMP,
                                                I2C_MEMADD_SIZE_8BIT, data, 2, 100);
    if (status == HAL_OK)
    {
        *raw_temp = ((uint16_t)data[0] << 8) | data[1];
    }
    return status;
}

/**
  * @brief Чтение температуры в градусах Цельсия с точностью 0.125°C
  */
HAL_StatusTypeDef LM75B_ReadTemperature(uint8_t dev_address, float *temp)
{
    if (temp == NULL) return HAL_ERROR;
    
    uint16_t raw_temp = 0;
    HAL_StatusTypeDef status = LM75B_ReadRawTemperature(dev_address, &raw_temp);
    if (status == HAL_OK)
    {
        // LM75B: 11 бит точности, шаг 0.125°C
        // Биты 15-5 содержат значение температуры в дополнительном коде
        int16_t temp_raw = (int16_t)(raw_temp & 0xFFE0);  // Оставляем только 11 бит
        *temp = (float)temp_raw / 256.0f;  // 256 = 2^8
    }
    return status;
}

/**
  * @brief Чтение регистра конфигурации
  */
HAL_StatusTypeDef LM75B_ReadConfig(uint8_t dev_address, uint8_t *config)
{
    if (config == NULL) return HAL_ERROR;
    
    return HAL_I2C_Mem_Read(&hi2c2, dev_address, LM75B_REG_CONF,
                            I2C_MEMADD_SIZE_8BIT, config, 1, 100);
}

/**
  * @brief Запись регистра конфигурации
  */
HAL_StatusTypeDef LM75B_WriteConfig(uint8_t dev_address, uint8_t config)
{
    // Очищаем зарезервированные биты 5-7
    config &= 0x1F;
    
    return HAL_I2C_Mem_Write(&hi2c2, dev_address, LM75B_REG_CONF,
                             I2C_MEMADD_SIZE_8BIT, &config, 1, 100);
}
