/**
@file    i2c_config.h
@brief   Конфигурация I2C2 для STM32F103C8T6
*/
#ifndef I2C_CONFIG_H
#define I2C_CONFIG_H
#include "stm32f1xx_hal.h"

#define I2C2_SCL_PIN                GPIO_PIN_10
#define I2C2_SCL_GPIO_PORT          GPIOB
#define I2C2_SDA_PIN                GPIO_PIN_11
#define I2C2_SDA_GPIO_PORT          GPIOB
#define I2C2_CLOCK_SPEED            100000

extern I2C_HandleTypeDef hi2c2;

HAL_StatusTypeDef MX_I2C2_Init(void);

#endif
