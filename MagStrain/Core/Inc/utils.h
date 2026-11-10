/** @file utils.h @brief Строковые утилиты и неблокирующий вывод USART2. */
#ifndef UTILS_H
#define UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "build_options.h"
#include <stdbool.h>
#include <stdint.h>

#if (USART2_DEBUG_ENABLED != 0U) || defined(UTILS_IMPLEMENTATION)
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
#else
/*
 * При отключенной отладке вызовы уничтожаются препроцессором непосредственно
 * в вызывающих модулях. Это важно: строковые литералы [LEVEL]/[STAT]/[CAL]
 * также не попадают во Flash. Функциональная логика при этом не меняется.
 */
#define USART2_Print(...)                 ((void)0)
#define USART2_PrintNum(...)              ((void)0)
#define USART2_PrintInt(...)              ((void)0)
#define USART2_PrintFloat(...)            ((void)0)
#define USART2_PrintHexByte(...)          ((void)0)
#define USART2_PrintHexBuffer(...)        ((void)0)
#define USART2_BufInit(...)               ((void)0)
#define USART2_BufPrint(...)              ((void)0)
#define USART2_BufPrintInt(...)           ((void)0)
#define USART2_BufPrintFloat(...)         ((void)0)
#define USART2_BufFlush(...)              ((void)0)
#define USART2_TxProcess(...)             ((void)0)
#define USART2_TxCpltCallback(...)        ((void)0)
#define USART2_TxErrorCallback(...)       ((void)0)
#define USART2_TxIsIdle()                 (true)
#define USART2_TxPendingBytes()           ((uint16_t)0U)
#define USART2_TxDroppedBytes()           ((uint32_t)0U)
#define USART2_PrintModBusCommand(...)    ((void)0)
#define USART2_PrintModBusResponse(...)   ((void)0)
#define ModBus_DebugFrame(...)            ((void)0)
#endif

#ifdef __cplusplus
}
#endif

#endif /* UTILS_H */
