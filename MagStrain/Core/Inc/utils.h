#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>

/* Функции для работы со строками */
void uint32_to_dec_str(uint32_t value, char* buffer);
void float_to_str(float value, char* buffer, int decimals);
void USART2_Print(const char* str);
void USART2_PrintNum(uint32_t num);
void USART2_PrintHexByte(uint8_t byte);
void USART2_PrintHexBuffer(const uint8_t* buffer, uint16_t length);
void USART2_PrintModBusCommand(const uint8_t* data, uint16_t length);
void USART2_PrintModBusResponse(const uint8_t* data, uint16_t length);
void ModBus_DebugFrame(const uint8_t* frame, uint16_t length, const char* prefix);

#endif /* UTILS_H */