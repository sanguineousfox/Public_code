/** @file rs485.h @brief Управление приемопередатчиком RS-485. */
#ifndef RS485_H
#define RS485_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdint.h>

void RS485_Init(void);
HAL_StatusTypeDef RS485_Transmit(UART_HandleTypeDef *uart,
                                const uint8_t *data,
                                uint16_t length,
                                uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* RS485_H */
