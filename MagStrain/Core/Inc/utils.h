/** @file utils.h @brief Строковые утилиты и неблокирующий вывод USART2. */
#ifndef UTILS_H
#define UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

void uint32_to_dec_str(uint32_t value, char *buffer);
void float_to_str(float value, char *buffer, int decimals);

void USART2_Print(const char *text);
void USART2_PrintNum(uint32_t value);
void USART2_PrintInt(int32_t value);
void USART2_PrintFloat(float value);
void USART2_PrintHexByte(uint8_t value);
void USART2_PrintHexBuffer(const uint8_t *buffer, uint16_t length);

void USART2_BufInit(void);
void USART2_BufPrint(const char *text);
void USART2_BufPrintInt(int32_t value);
void USART2_BufPrintFloat(float value);
void USART2_BufFlush(void);

/* Неблокирующая очередь отладочного UART. */
void USART2_TxProcess(void);
void USART2_TxCpltCallback(UART_HandleTypeDef *huart);
void USART2_TxErrorCallback(UART_HandleTypeDef *huart);
bool USART2_TxIsIdle(void);
uint16_t USART2_TxPendingBytes(void);
uint32_t USART2_TxDroppedBytes(void);

void USART2_PrintModBusCommand(const uint8_t *data, uint16_t length);
void USART2_PrintModBusResponse(const uint8_t *data, uint16_t length);
void ModBus_DebugFrame(const uint8_t *frame,
                       uint16_t length,
                       const char *prefix);

#ifdef __cplusplus
}
#endif

#endif /* UTILS_H */
