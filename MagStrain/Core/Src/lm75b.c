/**
 * @file    lm75b.c
 * @brief   Драйвер температурного датчика LM75B через I2C2.
 */
#include "lm75b.h"
#include "i2c_config.h"

#define LM75B_I2C_TIMEOUT_MS  10U

HAL_StatusTypeDef LM75B_Init(uint8_t device_address)
{
    uint8_t data[2] = {0U, 0U};

    return HAL_I2C_Mem_Read(&hi2c2,
                            device_address,
                            LM75B_REG_TEMP,
                            I2C_MEMADD_SIZE_8BIT,
                            data,
                            sizeof(data),
                            LM75B_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef LM75B_ReadRawTemperature(uint8_t device_address,
                                           uint16_t *raw_temperature)
{
    uint8_t data[2] = {0U, 0U};
    HAL_StatusTypeDef status;

    if (raw_temperature == NULL) {
        return HAL_ERROR;
    }

    status = HAL_I2C_Mem_Read(&hi2c2,
                              device_address,
                              LM75B_REG_TEMP,
                              I2C_MEMADD_SIZE_8BIT,
                              data,
                              sizeof(data),
                              LM75B_I2C_TIMEOUT_MS);
    if (status == HAL_OK) {
        *raw_temperature = ((uint16_t)data[0] << 8) | data[1];
    }

    return status;
}

HAL_StatusTypeDef LM75B_ReadTemperature(uint8_t device_address,
                                        float *temperature_c)
{
    uint16_t raw = 0U;
    HAL_StatusTypeDef status;

    if (temperature_c == NULL) {
        return HAL_ERROR;
    }

    status = LM75B_ReadRawTemperature(device_address, &raw);
    if (status == HAL_OK) {
        /* LM75B: 11-разрядное значение, шаг 0,125 °C, знак в старшем бите. */
        int16_t signed_raw = (int16_t)(raw & 0xFFE0U);
        *temperature_c = (float)signed_raw / 256.0f;
    }

    return status;
}

HAL_StatusTypeDef LM75B_ReadConfig(uint8_t device_address, uint8_t *config)
{
    if (config == NULL) {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Read(&hi2c2,
                            device_address,
                            LM75B_REG_CONF,
                            I2C_MEMADD_SIZE_8BIT,
                            config,
                            1U,
                            LM75B_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef LM75B_WriteConfig(uint8_t device_address, uint8_t config)
{
    config &= 0x1FU;

    return HAL_I2C_Mem_Write(&hi2c2,
                             device_address,
                             LM75B_REG_CONF,
                             I2C_MEMADD_SIZE_8BIT,
                             &config,
                             1U,
                             LM75B_I2C_TIMEOUT_MS);
}
