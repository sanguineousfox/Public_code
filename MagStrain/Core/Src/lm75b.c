/**
@file    lm75b.c
@brief   Драйвер температурного датчика LM75B через I2C2
*/
#include "lm75b.h"
#include "i2c_config.h"
#include "main.h"

HAL_StatusTypeDef LM75B_Init(uint8_t dev_address)
{
    uint8_t dummy[2] = {0};
    return HAL_I2C_Mem_Read(&hi2c2, dev_address, LM75B_REG_TEMP,
        I2C_MEMADD_SIZE_8BIT, dummy, 2, 100);
}

HAL_StatusTypeDef LM75B_ReadRawTemperature(uint8_t dev_address, uint16_t *raw_temp)
{
    if (raw_temp == NULL) return HAL_ERROR;
    uint8_t data[2] = {0};
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c2, dev_address, LM75B_REG_TEMP,
        I2C_MEMADD_SIZE_8BIT, data, 2, 100);
    if (status == HAL_OK) {
        *raw_temp = ((uint16_t)data[0] << 8) | data[1];
    }
    return status;
}

HAL_StatusTypeDef LM75B_ReadTemperature(uint8_t dev_address, float *temp)
{
    if (temp == NULL) return HAL_ERROR;
    uint16_t raw_temp = 0;
    HAL_StatusTypeDef status = LM75B_ReadRawTemperature(dev_address, &raw_temp);
    if (status == HAL_OK) {
        int16_t temp_raw = (int16_t)(raw_temp & 0xFFE0);
        *temp = (float)temp_raw / 256.0f;
    }
    return status;
}

HAL_StatusTypeDef LM75B_ReadConfig(uint8_t dev_address, uint8_t *config)
{
    if (config == NULL) return HAL_ERROR;
    return HAL_I2C_Mem_Read(&hi2c2, dev_address, LM75B_REG_CONF,
        I2C_MEMADD_SIZE_8BIT, config, 1, 100);
}

HAL_StatusTypeDef LM75B_WriteConfig(uint8_t dev_address, uint8_t config)
{
    config &= 0x1F;
    return HAL_I2C_Mem_Write(&hi2c2, dev_address, LM75B_REG_CONF,
        I2C_MEMADD_SIZE_8BIT, &config, 1, 100);
}
