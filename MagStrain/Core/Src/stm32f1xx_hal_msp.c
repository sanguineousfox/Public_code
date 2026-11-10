/**
 * @file    stm32f1xx_hal_msp.c
 * @brief   Инициализация низкоуровневых ресурсов STM32F103.
 */
#include "main.h"

void HAL_MspInit(void)
{
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();

    /* Освобождаем PB3/PB4/PA15, оставляя SWD для отладки. */
    __HAL_AFIO_REMAP_SWJ_NOJTAG();
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
    GPIO_InitTypeDef gpio = {0};

    if (hadc == NULL) {
        return;
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Mode = GPIO_MODE_ANALOG;

    if (hadc->Instance == ADC1) {
        __HAL_RCC_ADC1_CLK_ENABLE();
        gpio.Pin = Read_24V_Pin;
        HAL_GPIO_Init(Read_24V_GPIO_Port, &gpio);
    } else if (hadc->Instance == ADC2) {
        __HAL_RCC_ADC2_CLK_ENABLE();
        gpio.Pin = Read_12V_Pin | Read_5V_Pin;
        HAL_GPIO_Init(GPIOA, &gpio);
    }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{
    if (hadc == NULL) {
        return;
    }

    if (hadc->Instance == ADC1) {
        __HAL_RCC_ADC1_CLK_DISABLE();
        HAL_GPIO_DeInit(Read_24V_GPIO_Port, Read_24V_Pin);
    } else if (hadc->Instance == ADC2) {
        __HAL_RCC_ADC2_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, Read_12V_Pin | Read_5V_Pin);
    }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
    if (hi2c != NULL && hi2c->Instance == I2C2) {
        __HAL_RCC_I2C2_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10 | GPIO_PIN_11);
    }
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef gpio = {0};

    if (huart == NULL) {
        return;
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();

    if (huart->Instance == USART1) {
        __HAL_RCC_USART1_CLK_ENABLE();

        gpio.Pin = GPIO_PIN_9;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &gpio);

        gpio.Pin = GPIO_PIN_10;
        gpio.Mode = GPIO_MODE_INPUT;
        gpio.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &gpio);
#if (USART2_DEBUG_ENABLED != 0U)
    } else if (huart->Instance == USART2) {
        __HAL_RCC_USART2_CLK_ENABLE();

        gpio.Pin = GPIO_PIN_2;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &gpio);

        gpio.Pin = GPIO_PIN_3;
        gpio.Mode = GPIO_MODE_INPUT;
        gpio.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &gpio);

        /* USART2 — только отладка; ниже TIM3 и Modbus. */
        HAL_NVIC_SetPriority(USART2_IRQn, 3U, 0U);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
#endif
    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
    if (huart == NULL) {
        return;
    }

    if (huart->Instance == USART1) {
        __HAL_RCC_USART1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9 | GPIO_PIN_10);
#if (USART2_DEBUG_ENABLED != 0U)
    } else if (huart->Instance == USART2) {
        HAL_NVIC_DisableIRQ(USART2_IRQn);
        __HAL_RCC_USART2_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2 | GPIO_PIN_3);
#endif
    }
}
