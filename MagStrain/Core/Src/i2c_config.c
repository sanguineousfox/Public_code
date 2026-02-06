/**
  ******************************************************************************
  * @file    i2c_config.c
  * @author  ram223
  * @brief   Инициализация I2C2 для STM32F103C8T6 (PB10=SCL, PB11=SDA)
  ******************************************************************************
  */

#include "i2c_config.h"
#include "stm32f1xx_hal.h"
#include "main.h"  // Для доступа к USART2_Print

// Глобальный хендл I2C2 (только одно определение во всём проекте!)
I2C_HandleTypeDef hi2c2;

/**
  * @brief Инициализация I2C2 (PB10=SCL, PB11=SDA)
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef MX_I2C2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* === ШАГ 1: Включение тактирования === */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C2_CLK_ENABLE();

    /* === ШАГ 2: Настройка пинов PB10 (SCL) и PB11 (SDA) === */
    GPIO_InitStruct.Pin = I2C2_SCL_PIN | I2C2_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;  // Alternate Function Open Drain (ОБЯЗАТЕЛЬНО!)
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* === ШАГ 3: Конфигурация I2C2 === */
    hi2c2.Instance = I2C2;
    hi2c2.Init.ClockSpeed = I2C2_CLOCK_SPEED;
    hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1 = 0;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c2) != HAL_OK)
    {
        USART2_Print("[I2C] КРИТИЧЕСКАЯ ОШИБКА: HAL_I2C_Init FAILED!\r\n");
        return HAL_ERROR;
    }

    /* === ШАГ 4: Проверка наличия устройств на шине === */
    uint8_t dummy = 0;
    
    // Проверка LM75B (адрес 0x48)
    if (HAL_I2C_IsDeviceReady(&hi2c2, (0x48 << 1), 2, 10) == HAL_OK)
    {
        USART2_Print("[I2C] LM75B обнаружен на адресе 0x48\r\n");
    }
    else
    {
        USART2_Print("[I2C] ВНИМАНИЕ: LM75B НЕ ОБНАРУЖЕН на 0x48\r\n");
    }
    
    // Проверка AT24C02 (адрес 0x50)
    if (HAL_I2C_IsDeviceReady(&hi2c2, (0x50 << 1), 2, 10) == HAL_OK)
    {
        USART2_Print("[I2C] AT24C02 обнаружен на адресе 0x50\r\n");
    }
    else
    {
        USART2_Print("[I2C] ВНИМАНИЕ: AT24C02 НЕ ОБНАРУЖЕН на 0x50\r\n");
    }

    USART2_Print("[I2C] I2C2 инициализирован: PB10=SCL, PB11=SDA @ 100 kHz\r\n");
    return HAL_OK;
}

/**
  * @brief  Callback инициализации MSP для I2C2
  * @param  hi2c: указатель на структуру I2C_HandleTypeDef
  * @retval None
  */
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
    if(hi2c->Instance == I2C2)
    {
        /* Включение прерываний */
        HAL_NVIC_SetPriority(I2C2_EV_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
        HAL_NVIC_SetPriority(I2C2_ER_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);
    }
}
