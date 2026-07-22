/**
  ******************************************************************************
  * @file    i2c_config.h
  * @author  ram223
  * @brief   Конфигурация I2C2 для STM32F103C8T6 (PB10=SCL, PB11=SDA)
  ******************************************************************************
  */

#ifndef I2C_CONFIG_H
#define I2C_CONFIG_H

#include "stm32f1xx_hal.h"

// Пины I2C2 для STM32F103C8T6 (стандартное подключение!)
#define I2C2_SCL_PIN                GPIO_PIN_10
#define I2C2_SCL_GPIO_PORT          GPIOB

#define I2C2_SDA_PIN                GPIO_PIN_11
#define I2C2_SDA_GPIO_PORT          GPIOB

// Скорость шины (стандартная 100 kHz)
#define I2C2_CLOCK_SPEED            100000

// Внешняя переменная (определена в i2c_config.c)
extern I2C_HandleTypeDef hi2c2;

/**
  * @brief Инициализация I2C2 (PB10=SCL, PB11=SDA)
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef MX_I2C2_Init(void);

/* Добавьте эти прототипы в i2c_config.h */
HAL_StatusTypeDef I2C_CheckDevice(uint8_t dev_address);
HAL_StatusTypeDef I2C_ReadByte(uint8_t dev_address, uint8_t reg_address, uint8_t *data);
HAL_StatusTypeDef I2C_WriteByte(uint8_t dev_address, uint8_t reg_address, uint8_t data);
HAL_StatusTypeDef I2C_ReadMultiple(uint8_t dev_address, uint8_t reg_address, uint8_t *data, uint16_t size);
void I2C_ScanBus(void);

#endif /* I2C_CONFIG_H */
