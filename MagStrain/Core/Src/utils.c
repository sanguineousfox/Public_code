/** @file utils.c @brief Строковые утилиты и неблокирующий вывод USART2. */
#include "utils.h"
#include "main.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define DEBUG_BUILD_BUFFER_SIZE  512U
#define DEBUG_TX_QUEUE_SIZE      2048U

static char debug_build_buffer[DEBUG_BUILD_BUFFER_SIZE];
static uint16_t debug_build_length = 0U;

static uint8_t debug_tx_queue[DEBUG_TX_QUEUE_SIZE];
static volatile uint16_t debug_tx_head = 0U;
static volatile uint16_t debug_tx_tail = 0U;
static volatile uint16_t debug_tx_active_length = 0U;
static volatile uint8_t debug_tx_busy = 0U;
static volatile uint32_t debug_tx_dropped = 0U;

static uint32_t SignedMagnitude(int32_t value)
{
    return (value < 0) ? (uint32_t)(-(int64_t)value) : (uint32_t)value;
}

void uint32_to_dec_str(uint32_t value, char *buffer)
{
    char reversed[11];
    uint8_t count = 0U;
    uint8_t i;

    if (buffer == NULL) return;

    do {
        reversed[count++] = (char)('0' + value % 10U);
        value /= 10U;
    } while (value > 0U && count < sizeof(reversed));

    for (i = 0U; i < count; ++i) {
        buffer[i] = reversed[count - i - 1U];
    }
    buffer[count] = '\0';
}

void float_to_str(float value, char *buffer, int decimals)
{
    uint32_t scale = 1U;
    uint32_t integer_part;
    uint32_t fraction_part;
    uint8_t offset = 0U;
    char integer_text[12];
    int i;

    if (buffer == NULL) return;

    if (!isfinite(value)) {
        strcpy(buffer, isnan(value) ? "nan" : ((value < 0.0f) ? "-inf" : "inf"));
        return;
    }

    if (decimals < 0) decimals = 0;
    if (decimals > 6) decimals = 6;

    for (i = 0; i < decimals; ++i) scale *= 10U;

    if (value < 0.0f) {
        buffer[offset++] = '-';
        value = -value;
    }

    integer_part = (uint32_t)value;
    fraction_part = (uint32_t)(((value - (float)integer_part) * (float)scale) + 0.5f);
    if (fraction_part >= scale && decimals > 0) {
        integer_part++;
        fraction_part = 0U;
    }

    uint32_to_dec_str(integer_part, integer_text);
    strcpy(&buffer[offset], integer_text);
    offset = (uint8_t)(offset + strlen(integer_text));

    if (decimals > 0) {
        buffer[offset++] = '.';
        for (i = decimals - 1; i >= 0; --i) {
            uint32_t divisor = 1U;
            int j;
            for (j = 0; j < i; ++j) divisor *= 10U;
            buffer[offset++] = (char)('0' + (fraction_part / divisor) % 10U);
        }
    }
    buffer[offset] = '\0';
}

static uint16_t QueueFree(uint16_t head, uint16_t tail)
{
    if (head >= tail) {
        return (uint16_t)(DEBUG_TX_QUEUE_SIZE - (head - tail) - 1U);
    }
    return (uint16_t)(tail - head - 1U);
}

/* Вызывается либо с запрещенными IRQ, либо из USART2 IRQ. */
static void DebugTxKickLocked(void)
{
    uint16_t head;
    uint16_t tail;
    uint16_t length;

    if (debug_tx_busy != 0U) return;

    head = debug_tx_head;
    tail = debug_tx_tail;
    if (head == tail) return;

    length = (head > tail) ?
        (uint16_t)(head - tail) :
        (uint16_t)(DEBUG_TX_QUEUE_SIZE - tail);

    debug_tx_active_length = length;
    debug_tx_busy = 1U;

    if (HAL_UART_Transmit_IT(&huart2, &debug_tx_queue[tail], length) != HAL_OK) {
        debug_tx_busy = 0U;
        debug_tx_active_length = 0U;
    }
}

static void DebugTxEnqueue(const uint8_t *data, uint16_t length)
{
    uint16_t head;
    uint16_t tail;
    uint16_t free_space;
    uint16_t accepted;
    uint16_t i;

    if (data == NULL || length == 0U) return;

    head = debug_tx_head;
    tail = debug_tx_tail;
    free_space = QueueFree(head, tail);
    accepted = (length <= free_space) ? length : free_space;

    for (i = 0U; i < accepted; ++i) {
        debug_tx_queue[head] = data[i];
        head++;
        if (head >= DEBUG_TX_QUEUE_SIZE) head = 0U;
    }

    /* Публикуем head только после копирования данных. */
    debug_tx_head = head;
    if (accepted < length) {
        debug_tx_dropped += (uint32_t)(length - accepted);
    }

    USART2_TxProcess();
}

void USART2_TxProcess(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    DebugTxKickLocked();
    if (primask == 0U) {
        __enable_irq();
    }
}

void USART2_TxCpltCallback(UART_HandleTypeDef *huart)
{
    uint16_t tail;

    if (huart == NULL || huart->Instance != USART2) return;

    tail = (uint16_t)(debug_tx_tail + debug_tx_active_length);
    if (tail >= DEBUG_TX_QUEUE_SIZE) {
        tail = (uint16_t)(tail - DEBUG_TX_QUEUE_SIZE);
    }
    debug_tx_tail = tail;
    debug_tx_active_length = 0U;
    debug_tx_busy = 0U;
    DebugTxKickLocked();
}

void USART2_TxErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == NULL || huart->Instance != USART2) return;
    debug_tx_active_length = 0U;
    debug_tx_busy = 0U;
    DebugTxKickLocked();
}

bool USART2_TxIsIdle(void)
{
    return debug_tx_busy == 0U && debug_tx_head == debug_tx_tail;
}

uint16_t USART2_TxPendingBytes(void)
{
    uint16_t head = debug_tx_head;
    uint16_t tail = debug_tx_tail;

    if (head >= tail) return (uint16_t)(head - tail);
    return (uint16_t)(DEBUG_TX_QUEUE_SIZE - tail + head);
}

uint32_t USART2_TxDroppedBytes(void)
{
    return debug_tx_dropped;
}

void USART2_Print(const char *text)
{
    if (text == NULL) return;
    DebugTxEnqueue((const uint8_t *)text, (uint16_t)strlen(text));
}

void USART2_PrintNum(uint32_t value)
{
    char text[12];
    uint32_to_dec_str(value, text);
    USART2_Print(text);
}

void USART2_PrintInt(int32_t value)
{
    if (value < 0) USART2_Print("-");
    USART2_PrintNum(SignedMagnitude(value));
}

void USART2_PrintFloat(float value)
{
    char text[24];
    float_to_str(value, text, 2);
    USART2_Print(text);
}

void USART2_PrintHexByte(uint8_t value)
{
    static const char hex[] = "0123456789ABCDEF";
    char text[3];
    text[0] = hex[(value >> 4) & 0x0FU];
    text[1] = hex[value & 0x0FU];
    text[2] = '\0';
    USART2_Print(text);
}

void USART2_PrintHexBuffer(const uint8_t *buffer, uint16_t length)
{
    uint16_t i;
    if (buffer == NULL) return;
    for (i = 0U; i < length; ++i) {
        USART2_PrintHexByte(buffer[i]);
        if (i + 1U < length) USART2_Print(" ");
    }
}

void USART2_BufInit(void)
{
    debug_build_length = 0U;
}

static void BufferPutChar(char value)
{
    if (debug_build_length >= DEBUG_BUILD_BUFFER_SIZE) {
        USART2_BufFlush();
    }
    debug_build_buffer[debug_build_length++] = value;
}

void USART2_BufPrint(const char *text)
{
    if (text == NULL) return;
    while (*text != '\0') BufferPutChar(*text++);
}

void USART2_BufPrintInt(int32_t value)
{
    char text[12];
    uint32_t magnitude = SignedMagnitude(value);
    if (value < 0) BufferPutChar('-');
    uint32_to_dec_str(magnitude, text);
    USART2_BufPrint(text);
}

void USART2_BufPrintFloat(float value)
{
    char text[24];
    float_to_str(value, text, 2);
    USART2_BufPrint(text);
}

void USART2_BufFlush(void)
{
    if (debug_build_length == 0U) return;
    DebugTxEnqueue((const uint8_t *)debug_build_buffer, debug_build_length);
    debug_build_length = 0U;
}

static void PrintTimestamp(void)
{
    uint32_t tick = HAL_GetTick();
    uint32_t hours = tick / 3600000UL;
    uint32_t minutes = (tick / 60000UL) % 60UL;
    uint32_t seconds = (tick / 1000UL) % 60UL;
    uint32_t milliseconds = tick % 1000UL;

    if (hours < 10U) USART2_Print("0");
    USART2_PrintNum(hours);
    USART2_Print(":");
    if (minutes < 10U) USART2_Print("0");
    USART2_PrintNum(minutes);
    USART2_Print(":");
    if (seconds < 10U) USART2_Print("0");
    USART2_PrintNum(seconds);
    USART2_Print(":");
    if (milliseconds < 100U) USART2_Print("0");
    if (milliseconds < 10U) USART2_Print("0");
    USART2_PrintNum(milliseconds);
}

static void PrintFrame(const char *prefix, const uint8_t *data, uint16_t length)
{
    if (data == NULL || length == 0U) return;
    USART2_Print(prefix);
    PrintTimestamp();
    USART2_Print(" - ");
    USART2_PrintHexBuffer(data, length);
    USART2_Print("\r\n");
}

void USART2_PrintModBusCommand(const uint8_t *data, uint16_t length)
{
    PrintFrame("[RTU]>Rx > ", data, length);
}

void USART2_PrintModBusResponse(const uint8_t *data, uint16_t length)
{
    PrintFrame("[RTU]>Tx > ", data, length);
}

void ModBus_DebugFrame(const uint8_t *frame, uint16_t length, const char *prefix)
{
    USART2_Print("[MODBUS] ");
    USART2_Print((prefix != NULL) ? prefix : "FRAME");
    USART2_Print(": ");
    USART2_PrintHexBuffer(frame, length);
    USART2_Print("\r\n");
}
