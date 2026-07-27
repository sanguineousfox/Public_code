/** @file rs485.c @brief Управление приемопередатчиком RS-485. */
#include "rs485.h"
#include "main.h"

#define RS485_DE_ASSERT_DELAY_CYCLES     150U
#define RS485_DE_RELEASE_DELAY_CYCLES    300U

static void DelayCycles(volatile uint32_t cycles)
{
    while (cycles-- > 0U) {
        __NOP();
    }
}

void RS485_Init(void)
{
    HAL_GPIO_WritePin(RS485_CTRL_GPIO_Port,
                      RS485_CTRL_Pin,
                      GPIO_PIN_RESET);
}

HAL_StatusTypeDef RS485_Transmit(UART_HandleTypeDef *uart,
                                const uint8_t *data,
                                uint16_t length,
                                uint32_t timeout_ms)
{
    HAL_StatusTypeDef status;
    uint32_t start;

    if (uart == NULL || data == NULL || length == 0U) {
        return HAL_ERROR;
    }

    HAL_GPIO_WritePin(RS485_CTRL_GPIO_Port,
                      RS485_CTRL_Pin,
                      GPIO_PIN_SET);
    DelayCycles(RS485_DE_ASSERT_DELAY_CYCLES);

    status = HAL_UART_Transmit(uart, (uint8_t *)data, length, timeout_ms);

    if (status == HAL_OK) {
        start = HAL_GetTick();
        while (__HAL_UART_GET_FLAG(uart, UART_FLAG_TC) == RESET) {
            if ((uint32_t)(HAL_GetTick() - start) >= timeout_ms) {
                status = HAL_TIMEOUT;
                break;
            }
        }
    }

    DelayCycles(RS485_DE_RELEASE_DELAY_CYCLES);
    HAL_GPIO_WritePin(RS485_CTRL_GPIO_Port,
                      RS485_CTRL_Pin,
                      GPIO_PIN_RESET);
    return status;
}
