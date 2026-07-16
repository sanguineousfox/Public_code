#include "utils.h"
#include "main.h"
#include <string.h>

/* Внешние переменные */
extern UART_HandleTypeDef huart2;

/* Буферы для преобразования чисел в строки */
static char str_buffer[32];

/**
  * @brief  Вывод строки через USART2
  */
void USART2_Print(const char* str)
{
    if (str == NULL) return;
    HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 100);
}

/**
  * @brief  Вывод числа через USART2
  */
void USART2_PrintNum(uint32_t num)
{
    /* Преобразование числа в строку */
    if (num == 0) {
        USART2_Print("0");
        return;
    }
    
    char temp[16];
    int i = 0;
    
    while (num > 0 && i < 15) {
        temp[i++] = (num % 10) + '0';
        num /= 10;
    }
    
    /* Вывод в обратном порядке */
    for (int j = i - 1; j >= 0; j--) {
        HAL_UART_Transmit(&huart2, (uint8_t*)&temp[j], 1, 100);
    }
}

/**
  * @brief  Вывод одного байта в hex формате на USART2
  */
void USART2_PrintHexByte(uint8_t byte)
{
    const char hex_chars[] = "0123456789ABCDEF";
    str_buffer[0] = hex_chars[(byte >> 4) & 0x0F];
    str_buffer[1] = hex_chars[byte & 0x0F];
    str_buffer[2] = '\0';
    USART2_Print(str_buffer);
}

/**
  * @brief  Вывод буфера в hex формате на USART2
  */
void USART2_PrintHexBuffer(const uint8_t* buffer, uint16_t length)
{
    for (uint16_t i = 0; i < length; i++) {
        USART2_PrintHexByte(buffer[i]);
        if (i < length - 1) {
            USART2_Print(" ");
        }
    }
}

/**
  * @brief  Вывод ModBus команды в формате QModMaster на USART2
  */
void USART2_PrintModBusCommand(const uint8_t* data, uint16_t length)
{
    if (length == 0) return;

    USART2_Print("Sys > ");

    /* Получаем текущее время */
    uint32_t tick = HAL_GetTick();
    uint32_t hours = tick / 3600000;
    uint32_t mins = (tick % 3600000) / 60000;
    uint32_t secs = (tick % 60000) / 1000;
    uint32_t ms = tick % 1000;

    /* Выводим время в формате HH:MM:SS:mmm */
    if (hours < 10) USART2_Print("0");
    USART2_PrintNum(hours);
    USART2_Print(":");
    
    if (mins < 10) USART2_Print("0");
    USART2_PrintNum(mins);
    USART2_Print(":");
    
    if (secs < 10) USART2_Print("0");
    USART2_PrintNum(secs);
    USART2_Print(":");
    
    if (ms < 100) USART2_Print("0");
    if (ms < 10) USART2_Print("0");
    USART2_PrintNum(ms);
    USART2_Print(" - ");

    /* Выводим данные команды */
    USART2_PrintHexBuffer(data, length);
    USART2_Print("\r\n");
}

/**
  * @brief  Вывод ModBus ответа на USART2
  */
void USART2_PrintModBusResponse(const uint8_t* data, uint16_t length)
{
    if (length == 0) return;

    USART2_Print("[RTU]>Tx > ");

    /* Получаем текущее время */
    uint32_t tick = HAL_GetTick();
    uint32_t hours = tick / 3600000;
    uint32_t mins = (tick % 3600000) / 60000;
    uint32_t secs = (tick % 60000) / 1000;
    uint32_t ms = tick % 1000;

    /* Выводим время в формате HH:MM:SS:mmm */
    if (hours < 10) USART2_Print("0");
    USART2_PrintNum(hours);
    USART2_Print(":");
    
    if (mins < 10) USART2_Print("0");
    USART2_PrintNum(mins);
    USART2_Print(":");
    
    if (secs < 10) USART2_Print("0");
    USART2_PrintNum(secs);
    USART2_Print(":");
    
    if (ms < 100) USART2_Print("0");
    if (ms < 10) USART2_Print("0");
    USART2_PrintNum(ms);
    USART2_Print(" - ");

    /* Выводим данные ответа */
    USART2_PrintHexBuffer(data, length);
    USART2_Print("\r\n");
}

/**
  * @brief  Вывод ModBus фрейма в детальном формате
  */
void ModBus_DebugFrame(const uint8_t* frame, uint16_t length, const char* prefix)
{
    if (length == 0) return;

    USART2_Print("[MODBUS] ");
    USART2_Print(prefix);
    USART2_Print(": ");

    /* Вывод hex */
    for(uint16_t i = 0; i < length; i++) {
        USART2_PrintHexByte(frame[i]);
        USART2_Print(" ");
    }

    USART2_Print(" | ");

    /* Парсинг фрейма */
    if (length >= 3) {
        uint8_t addr = frame[0];
        uint8_t func = frame[1];

        USART2_Print("Addr=");
        USART2_PrintNum(addr);
        USART2_Print(" Func=");
        USART2_PrintHexByte(func);

        switch(func) {
            case 0x03: /* Read Holding Registers */
                if (length >= 8) {
                    uint16_t start = (frame[2] << 8) | frame[3];
                    uint16_t count = (frame[4] << 8) | frame[5];
                    USART2_Print(" ReadHold Start=");
                    USART2_PrintNum(start);
                    USART2_Print(" Count=");
                    USART2_PrintNum(count);
                }
                break;

            case 0x04: /* Read Input Registers */
                if (length >= 8) {
                    uint16_t start = (frame[2] << 8) | frame[3];
                    uint16_t count = (frame[4] << 8) | frame[5];
                    USART2_Print(" ReadInput Start=");
                    USART2_PrintNum(start);
                    USART2_Print(" Count=");
                    USART2_PrintNum(count);
                }
                break;

            case 0x06: /* Write Single Register */
                if (length >= 8) {
                    uint16_t reg = (frame[2] << 8) | frame[3];
                    uint16_t value = (frame[4] << 8) | frame[5];
                    USART2_Print(" WriteSingle Reg=");
                    USART2_PrintNum(reg);
                    USART2_Print(" Value=");
                    USART2_PrintNum(value);
                }
                break;

            case 0x10: /* Write Multiple Registers */
                if (length >= 9) {
                    uint16_t start = (frame[2] << 8) | frame[3];
                    uint16_t count = (frame[4] << 8) | frame[5];
                    uint8_t byte_count = frame[6];
                    USART2_Print(" WriteMulti Start=");
                    USART2_PrintNum(start);
                    USART2_Print(" Count=");
                    USART2_PrintNum(count);
                    USART2_Print(" Bytes=");
                    USART2_PrintNum(byte_count);
                }
                break;

            case 0x83: /* Exception for 0x03 */
                USART2_Print(" Exception(0x03) Code=");
                if (length >= 3) {
                    USART2_PrintHexByte(frame[2]);
                }
                break;

            case 0x84: /* Exception for 0x04 */
                USART2_Print(" Exception(0x04) Code=");
                if (length >= 3) {
                    USART2_PrintHexByte(frame[2]);
                }
                break;

            case 0x86: /* Exception for 0x06 */
                USART2_Print(" Exception(0x06) Code=");
                if (length >= 3) {
                    USART2_PrintHexByte(frame[2]);
                }
                break;

            case 0x90: /* Exception for 0x10 */
                USART2_Print(" Exception(0x10) Code=");
                if (length >= 3) {
                    USART2_PrintHexByte(frame[2]);
                }
                break;

            default:
                USART2_Print(" Unknown function");
                break;
        }
    }

    /* Проверка CRC */
    if (length >= 2) {
        uint16_t recv_crc = (frame[length-1] << 8) | frame[length-2];
        USART2_Print(" CRC=");
        USART2_PrintHexByte((recv_crc >> 8) & 0xFF);
        USART2_PrintHexByte(recv_crc & 0xFF);
    }

    USART2_Print("\r\n");
}

/**
  * @brief  Преобразование uint32 в строку
  */
void uint32_to_dec_str(uint32_t value, char* buffer)
{
    if (value == 0) {
        buffer[0] = '0';
        buffer[1] = '\0';
        return;
    }
    
    char temp[16];
    int i = 0;
    
    while (value > 0 && i < 15) {
        temp[i++] = (value % 10) + '0';
        value /= 10;
    }
    
    for (int j = 0; j < i; j++) {
        buffer[j] = temp[i - j - 1];
    }
    buffer[i] = '\0';
}

/**
  * @brief  Преобразование float в строку
  */
void float_to_str(float value, char* buffer, int decimals)
{
    int offset = 0;
    char temp_buffer[32];

    if (value < 0) {
        temp_buffer[0] = '-';
        value = -value;
        offset = 1;
    }

    int int_part = (int)value;
    float frac_part = value - (float)int_part;

    /* Преобразуем целую часть */
    if (int_part == 0) {
        temp_buffer[offset] = '0';
        offset += 1;
    } else {
        int i = 0;
        while (int_part > 0 && i < 15) {
            temp_buffer[offset + i] = (int_part % 10) + '0';
            int_part /= 10;
            i++;
        }
        
        /* Переворачиваем цифры */
        for (int j = 0; j < i/2; j++) {
            char temp = temp_buffer[offset + j];
            temp_buffer[offset + j] = temp_buffer[offset + i - j - 1];
            temp_buffer[offset + i - j - 1] = temp;
        }
        offset += i;
    }

    /* Добавляем дробную часть */
    if (decimals > 0) {
        temp_buffer[offset] = '.';
        offset++;
        
        for (int i = 0; i < decimals; i++) {
            frac_part *= 10.0f;
            int digit = (int)frac_part;
            temp_buffer[offset + i] = digit + '0';
            frac_part -= (float)digit;
        }
        temp_buffer[offset + decimals] = '\0';
    } else {
        temp_buffer[offset] = '\0';
    }

    /* Копируем в выходной буфер */
    strcpy(buffer, temp_buffer);
}