/* USER CODE BEGIN Header */
/**
  @file           : i2c_config.c
  @brief          : Конфигурация I2C для ПМП-201Е
                   : I2C2 используется для подключения датчика температуры LM75B
*/
/* USER CODE END Header */

#include "i2c_config.h"
#include "main.h"

/* ==========================================================================
   ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ
   ========================================================================== */
I2C_HandleTypeDef hi2c2;

/* ==========================================================================
   ФУНКЦИЯ: Инициализация I2C2 (PB10=SCL, PB11=SDA)
   ========================================================================== */
HAL_StatusTypeDef MX_I2C2_Init(void)
{
    /* Тактирование I2C2 и GPIOB */
    __HAL_RCC_I2C2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    /* Настройка GPIO: PB10=I2C2_SCL, PB11=I2C2_SDA (Open-Drain) */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Конфигурация I2C2 */
    hi2c2.Instance = I2C2;
    hi2c2.Init.ClockSpeed = 100000;           /* 100 кГц (стандартный режим) */
    hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1 = 0;               /* Ведущий режим (адрес не нужен) */
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
        return HAL_ERROR;
    }

    /* Для STM32F1xx не требуется дополнительная настройка фильтров */
    /* I2C_AnalogFilterTypeDef существует только в F4/F7 сериях */
    
    /* Чтение статуса для очистки флагов ошибки */
    volatile uint32_t status = I2C2->SR1;
    (void)status;
    status = I2C2->SR2;
    (void)status;

    return HAL_OK;
}

/* ==========================================================================
   CALLBACK: Обработка ошибок I2C
   ========================================================================== */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C2) {
        /* Сброс I2C при ошибке */
        HAL_I2C_DeInit(hi2c);
        MX_I2C2_Init();
    }
}

/* ==========================================================================
   ФУНКЦИЯ: Проверка наличия устройства на шине I2C
   ========================================================================== */
HAL_StatusTypeDef I2C_CheckDevice(uint8_t dev_address)
{
    return HAL_I2C_IsDeviceReady(&hi2c2, dev_address, 2, 10);
}

/* ==========================================================================
   ФУНКЦИЯ: Чтение байта из регистра устройства
   ========================================================================== */
HAL_StatusTypeDef I2C_ReadByte(uint8_t dev_address, uint8_t reg_address, uint8_t *data)
{
    return HAL_I2C_Mem_Read(&hi2c2, dev_address, reg_address,
                            I2C_MEMADD_SIZE_8BIT, data, 1, 100);
}

/* ==========================================================================
   ФУНКЦИЯ: Запись байта в регистр устройства
   ========================================================================== */
HAL_StatusTypeDef I2C_WriteByte(uint8_t dev_address, uint8_t reg_address, uint8_t data)
{
    return HAL_I2C_Mem_Write(&hi2c2, dev_address, reg_address,
                             I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

/* ==========================================================================
   ФУНКЦИЯ: Чтение нескольких байт подряд
   ========================================================================== */
HAL_StatusTypeDef I2C_ReadMultiple(uint8_t dev_address, uint8_t reg_address,
                                   uint8_t *data, uint16_t size)
{
    return HAL_I2C_Mem_Read(&hi2c2, dev_address, reg_address,
                            I2C_MEMADD_SIZE_8BIT, data, size, 100);
}
