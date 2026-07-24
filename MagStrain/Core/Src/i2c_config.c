/**
@file    i2c_config.c
@brief   Конфигурация I2C2
*/
#include "i2c_config.h"
#include "main.h"

I2C_HandleTypeDef hi2c2;

HAL_StatusTypeDef MX_I2C2_Init(void)
{
    __HAL_RCC_I2C2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    hi2c2.Instance = I2C2;
    hi2c2.Init.ClockSpeed = 100000;
    hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1 = 0;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
        return HAL_ERROR;
    }
    volatile uint32_t status = I2C2->SR1;
    (void)status;
    status = I2C2->SR2;
    (void)status;
    return HAL_OK;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C2) {
        HAL_I2C_DeInit(hi2c);
        MX_I2C2_Init();
    }
}

HAL_StatusTypeDef I2C_CheckDevice(uint8_t dev_address)
{
    return HAL_I2C_IsDeviceReady(&hi2c2, dev_address, 2, 10);
}

HAL_StatusTypeDef I2C_ReadByte(uint8_t dev_address, uint8_t reg_address, uint8_t *data)
{
    return HAL_I2C_Mem_Read(&hi2c2, dev_address, reg_address,
        I2C_MEMADD_SIZE_8BIT, data, 1, 100);
}

HAL_StatusTypeDef I2C_WriteByte(uint8_t dev_address, uint8_t reg_address, uint8_t data)
{
    return HAL_I2C_Mem_Write(&hi2c2, dev_address, reg_address,
        I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

HAL_StatusTypeDef I2C_ReadMultiple(uint8_t dev_address, uint8_t reg_address,
    uint8_t *data, uint16_t size)
{
    return HAL_I2C_Mem_Read(&hi2c2, dev_address, reg_address,
        I2C_MEMADD_SIZE_8BIT, data, size, 100);
}
