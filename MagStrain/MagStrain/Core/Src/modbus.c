/* USER CODE BEGIN Header */
/*
  @file           : modbus.c
  @brief          : ModBus RTU Implementation for STM32F1.
  @description    :
          - Чтение/запись Holding/Input Registers.
          - Управление линией RS485 (TX/RX).
          - Обработка CRC и ошибок.
*/
/* USER CODE END Header */

#include "modbus.h"
#include "main.h"
#include <string.h>

/* ============================================================================
   ВСПОМОГАТЕЛЬНАЯ ФУНКЦИЯ ДЛЯ ПРЕОБРАЗОВАНИЯ ЧИСЛА В СТРОКУ.
============================================================================ */
__attribute__((unused))
static void modbus_uint32_to_str(uint32_t value, char *buffer)
{
    if (value == 0U) {
        buffer[0] = '0';
        buffer[1] = '\0';
        return;
    }
    char temp[16];
    int i = 0;
    while (value > 0U && i < 15) {
        temp[i++] = (char)((value % 10U) + '0');
        value /= 10U;
    }
    for (int j = 0; j < i; j++) {
        buffer[j] = temp[i - j - 1];
    }
    buffer[i] = '\0';
}

/* ============================================================================
   СТРУКТУРА ДАННЫХ ДЛЯ МОДУЛЯ MODBUS.
============================================================================ */
typedef struct {
    uint16_t holding_regs[HOLD_HOLDING_REG_COUNT];
    uint16_t input_regs[REG_INPUT_REG_COUNT];
    uint8_t device_address;
    uint32_t rx_timeout_ms;
    uint16_t rx_index;
    uint8_t rx_buffer[MODBUS_BUFFER_SIZE];
    uint8_t rx_byte;
    uint8_t rx_active;
    uint8_t rx_complete;
    uint8_t rx_error;
    uint32_t rx_byte_count;
    uint32_t last_byte_time_ms;
} ModBus_Struct;

static ModBus_Struct modbus_instance;

/* Внешние переменные для доступа к UART1 из main.c. */
extern UART_HandleTypeDef huart1;
extern volatile uint8_t modbus_tx_active;

/* ============================================================================
   CRC16 ALGORITHM
============================================================================ */
uint16_t ModBus_CRC16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint8_t bit;
    while(length--) {
        crc ^= *data++;
        for(bit = 0; bit < 8; bit++) {
            if(crc & 0x0001U) {
                crc = (crc >> 1U) ^ 0xA001U;
            } else {
                crc >>= 1U;
            }
        }
    }
    return crc;
}

/* ============================================================================
   ОТПРАВКА ИСКЛЮЧЕНИЯ ПРИ ОШИБКЕ
============================================================================ */
static void ModBus_SendException(uint8_t function, uint8_t exception_code)
{
    uint8_t response[5];
    response[0] = modbus_instance.device_address;
    response[1] = function | 0x80U;
    response[2] = exception_code;
    uint16_t crc = ModBus_CRC16(response, 3U);
    response[3] = crc & 0xFFU;
    response[4] = (crc >> 8) & 0xFFU;

    if (!modbus_tx_active) {
        ModBus_PrepareForTransmit();
    }
    HAL_UART_Transmit(&huart1, response, 5, 200U);
}

/* ============================================================================
   ЧТЕНИЕ HOLDING REGISTERS (Функция 0x03).
============================================================================ */
static void ModBus_ReadHoldingRegisters(uint16_t start_addr, uint16_t reg_count)
{
    if (start_addr >= HOLD_HOLDING_REG_COUNT) {
        ModBus_SendException(MODBUS_READ_HOLDING_REGISTERS, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
        return;
    }
    if ((reg_count == 0U) || (reg_count > 125U)) {
        ModBus_SendException(MODBUS_READ_HOLDING_REGISTERS, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
        return;
    }
    if ((start_addr + reg_count) > HOLD_HOLDING_REG_COUNT) {
        ModBus_SendException(MODBUS_READ_HOLDING_REGISTERS, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
        return;
    }

    uint8_t response[256];
    uint16_t index = 0U;
    uint16_t bytes_to_send = reg_count * 2U;

    response[index++] = modbus_instance.device_address;
    response[index++] = MODBUS_READ_HOLDING_REGISTERS;
    response[index++] = bytes_to_send;

    for(uint16_t i = 0U; i < reg_count; i++) {
        uint16_t reg_value = modbus_instance.holding_regs[start_addr + i];
        response[index++] = (reg_value >> 8) & 0xFFU;
        response[index++] = reg_value & 0xFFU;
    }

    uint16_t crc = ModBus_CRC16(response, index);
    response[index++] = crc & 0xFFU;
    response[index++] = (crc >> 8) & 0xFFU;

    if (!modbus_tx_active) {
        ModBus_PrepareForTransmit();
    }
    HAL_UART_Transmit(&huart1, response, index, 200U);
}

/* ============================================================================
   ЧТЕНИЕ INPUT REGISTERS (Функция 0x04).
============================================================================ */
static void ModBus_ReadInputRegisters(uint16_t start_addr, uint16_t reg_count)
{
    if (start_addr >= REG_INPUT_REG_COUNT) {
        ModBus_SendException(MODBUS_READ_INPUT_REGISTERS, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
        return;
    }
    if ((reg_count == 0U) || (reg_count > 125U)) {
        ModBus_SendException(MODBUS_READ_INPUT_REGISTERS, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
        return;
    }
    if ((start_addr + reg_count) > REG_INPUT_REG_COUNT) {
        ModBus_SendException(MODBUS_READ_INPUT_REGISTERS, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
        return;
    }

    uint8_t response[256];
    uint16_t index = 0U;
    uint16_t bytes_to_send = reg_count * 2U;

    response[index++] = modbus_instance.device_address;
    response[index++] = MODBUS_READ_INPUT_REGISTERS;
    response[index++] = bytes_to_send;

    for(uint16_t i = 0U; i < reg_count; i++) {
        uint16_t reg_value = modbus_instance.input_regs[start_addr + i];
        response[index++] = (reg_value >> 8) & 0xFFU;
        response[index++] = reg_value & 0xFFU;
    }

    uint16_t crc = ModBus_CRC16(response, index);
    response[index++] = crc & 0xFFU;
    response[index++] = (crc >> 8) & 0xFFU;

    if (!modbus_tx_active) {
        ModBus_PrepareForTransmit();
    }
    HAL_UART_Transmit(&huart1, response, index, 200U);
}

/* ============================================================================
   ЗАПИСЬ ОДНОГО REGISTRA (Функция 0x06).
============================================================================ */
static void ModBus_WriteSingleRegister(uint16_t reg_addr, uint16_t value)
{
    if (reg_addr >= HOLD_HOLDING_REG_COUNT) {
        ModBus_SendException(MODBUS_WRITE_SINGLE_REGISTER, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
        return;
    }

    modbus_instance.holding_regs[reg_addr] = value;

    if (reg_addr == HOLD_DEVICE_ADDR) {
        uint8_t new_addr = (uint8_t)(value & 0xFFU);
        if ((new_addr >= 1U) && (new_addr <= 247U)) {
            modbus_instance.device_address = new_addr;
        }
    }

    uint8_t response_local[8];
    response_local[0] = modbus_instance.device_address;
    response_local[1] = MODBUS_WRITE_SINGLE_REGISTER;
    response_local[2] = (reg_addr >> 8) & 0xFFU;
    response_local[3] = reg_addr & 0xFFU;
    response_local[4] = (value >> 8) & 0xFFU;
    response_local[5] = value & 0xFFU;
    uint16_t crc = ModBus_CRC16(response_local, 6);
    response_local[6] = crc & 0xFFU;
    response_local[7] = (crc >> 8) & 0xFFU;

    if (!modbus_tx_active) {
        ModBus_PrepareForTransmit();
    }
    HAL_UART_Transmit(&huart1, response_local, 8U, 200U);
}

/* ============================================================================
   ЗАПИСЬ НЕСКОЛЬКИХ REGISTERS (Функция 0x10).
============================================================================ */
static void ModBus_WriteMultipleRegisters(uint16_t start_addr, uint16_t reg_count, uint8_t *data)
{
    if (start_addr >= HOLD_HOLDING_REG_COUNT) {
        ModBus_SendException(MODBUS_WRITE_MULTIPLE_REGISTERS, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
        return;
    }
    if ((reg_count == 0U) || (reg_count > 123U)) {
        ModBus_SendException(MODBUS_WRITE_MULTIPLE_REGISTERS, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
        return;
    }
    if ((start_addr + reg_count) > HOLD_HOLDING_REG_COUNT) {
        ModBus_SendException(MODBUS_WRITE_MULTIPLE_REGISTERS, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
        return;
    }

    for(uint16_t i = 0U; i < reg_count; i++) {
        uint16_t value = ((uint32_t)data[i*2] << 8) | data[i*2 + 1];
        modbus_instance.holding_regs[start_addr + i] = (uint16_t)value;

        if ((start_addr + i) == HOLD_DEVICE_ADDR) {
            uint8_t new_addr = (uint8_t)(value & 0xFFU);
            if ((new_addr >= 1U) && (new_addr <= 247U)) {
                modbus_instance.device_address = new_addr;
            }
        }
    }

    uint8_t response_local[9];
    response_local[0] = modbus_instance.device_address;
    response_local[1] = MODBUS_WRITE_MULTIPLE_REGISTERS;
    response_local[2] = (start_addr >> 8) & 0xFFU;
    response_local[3] = start_addr & 0xFFU;
    response_local[4] = (reg_count >> 8) & 0xFFU;
    response_local[5] = reg_count & 0xFFU;
    response_local[6] = data[0];
    uint16_t crc = ModBus_CRC16(response_local, 7);
    response_local[7] = crc & 0xFFU;
    response_local[8] = (crc >> 8) & 0xFFU;

    if (!modbus_tx_active) {
        ModBus_PrepareForTransmit();
    }
    HAL_UART_Transmit(&huart1, response_local, 9U, 200U);
}

/* ============================================================================
   ОБРАБОТКА ПРИНЯТОГО ФРЕЙМА
============================================================================ */
static void ModBus_ProcessFrame(void)
{
    if (modbus_instance.rx_index < 6U) {
        modbus_instance.rx_index = 0U;
        return;
    }

    uint16_t received_crc = ((uint32_t)modbus_instance.rx_buffer[modbus_instance.rx_index - 1U] << 8) |
                             (uint32_t)modbus_instance.rx_buffer[modbus_instance.rx_index - 2U];
    uint16_t calculated_crc = ModBus_CRC16(modbus_instance.rx_buffer, modbus_instance.rx_index - 2U);

    if (received_crc != calculated_crc) {
        modbus_instance.rx_index = 0U;
        modbus_instance.rx_error = 1U;
        return;
    }

    uint8_t device_addr = modbus_instance.rx_buffer[0];
    if ((device_addr != modbus_instance.device_address) && (device_addr != 0U)) {
        modbus_instance.rx_index = 0U;
        return;
    }

    switch(modbus_instance.rx_buffer[1]) {
        case MODBUS_READ_HOLDING_REGISTERS:
            if (modbus_instance.rx_index >= 8U) {
                uint16_t start_addr = ((uint32_t)modbus_instance.rx_buffer[2] << 8) | modbus_instance.rx_buffer[3];
                uint16_t reg_count = ((uint32_t)modbus_instance.rx_buffer[4] << 8) | modbus_instance.rx_buffer[5];
                ModBus_ReadHoldingRegisters(start_addr, reg_count);
            }
            break;
        case MODBUS_READ_INPUT_REGISTERS:
            if (modbus_instance.rx_index >= 8U) {
                uint16_t start_addr = ((uint32_t)modbus_instance.rx_buffer[2] << 8) | modbus_instance.rx_buffer[3];
                uint16_t reg_count = ((uint32_t)modbus_instance.rx_buffer[4] << 8) | modbus_instance.rx_buffer[5];
                ModBus_ReadInputRegisters(start_addr, reg_count);
            }
            break;
        case MODBUS_WRITE_SINGLE_REGISTER:
            if (modbus_instance.rx_index >= 8U) {
                uint16_t reg_addr = ((uint32_t)modbus_instance.rx_buffer[2] << 8) | modbus_instance.rx_buffer[3];
                uint16_t reg_value = ((uint32_t)modbus_instance.rx_buffer[4] << 8) | modbus_instance.rx_buffer[5];
                ModBus_WriteSingleRegister(reg_addr, reg_value);
            }
            break;
        case MODBUS_WRITE_MULTIPLE_REGISTERS:
            if (modbus_instance.rx_index >= 9U) {
                uint16_t start_addr = ((uint32_t)modbus_instance.rx_buffer[2] << 8) | modbus_instance.rx_buffer[3];
                uint16_t reg_count = ((uint32_t)modbus_instance.rx_buffer[4] << 8) | modbus_instance.rx_buffer[5];
                uint8_t byte_count = modbus_instance.rx_buffer[6];
                if ((byte_count == reg_count * 2U) && (modbus_instance.rx_index == 7U + byte_count + 2U)) {
                    ModBus_WriteMultipleRegisters(start_addr, reg_count, &modbus_instance.rx_buffer[7]);
                } else {
                    ModBus_SendException(0x10U, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
                }
            }
            break;
        default:
            ModBus_SendException(modbus_instance.rx_buffer[1], MODBUS_EXCEPTION_ILLEGAL_FUNCTION);
            break;
    }

    modbus_instance.rx_index = 0U;
    modbus_instance.rx_complete = 0U;
    modbus_instance.rx_error = 0U;
}

/* ============================================================================
   ИНИЦИАЛИЗАЦИЯ MODBUS
============================================================================ */
void ModBus_Init(void)
{
    memset(&modbus_instance, 0U, sizeof(modbus_instance));
    modbus_instance.device_address = MODBUS_DEFAULT_ADDRESS;
    modbus_instance.rx_active = 1U;
    modbus_instance.rx_complete = 0U;
    modbus_instance.rx_error = 0U;
    modbus_instance.rx_timeout_ms = 2U;
    modbus_instance.rx_byte_count = 0U;
    modbus_instance.last_byte_time_ms = HAL_GetTick();

    for (int i = 0; i < HOLD_HOLDING_REG_COUNT; i++) {
        modbus_instance.holding_regs[i] = 0U;
    }
    modbus_instance.holding_regs[HOLD_DEVICE_ADDR] = MODBUS_DEFAULT_ADDRESS;
    modbus_instance.holding_regs[HOLD_BAUDRATE] = MODBUS_BAUDRATE;
    modbus_instance.holding_regs[HOLD_PARITY] = 0U;
    modbus_instance.holding_regs[HOLD_STOP_BITS] = 1U;

    for (int i = 0; i < REG_INPUT_REG_COUNT; i++) {
        modbus_instance.input_regs[i] = 0xFFFFU;
    }

    HAL_UART_Receive_IT(&huart1, &modbus_instance.rx_byte, 1U);
}

/* ============================================================================
   CALLBACK ПРИЕМ ДАННЫХ UART
============================================================================ */
void ModBus_RxCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        ModBus_RxByte(modbus_instance.rx_byte);
        HAL_UART_Receive_IT(&huart1, &modbus_instance.rx_byte, 1U);
    }
}

/* ============================================================================
   ОБРАБОТКА ПРИНЯТОГО БАЙТА
============================================================================ */
void ModBus_RxByte(uint8_t byte)
{
    uint32_t current_time = HAL_GetTick();

    if ((current_time - modbus_instance.last_byte_time_ms) > MODBUS_INTER_FRAME_TIMEOUT_MS) {
        modbus_instance.rx_index = 0U;
    }
    modbus_instance.last_byte_time_ms = current_time;

    if (modbus_instance.rx_active && modbus_instance.rx_index < MODBUS_BUFFER_SIZE) {
        modbus_instance.rx_buffer[modbus_instance.rx_index] = byte;
        modbus_instance.rx_index++;
        if (modbus_instance.rx_index >= MODBUS_BUFFER_SIZE) {
            modbus_instance.rx_index = 0U;
            modbus_instance.rx_error = 1U;
        }
    } else {
        modbus_instance.rx_error = 1U;
        modbus_instance.rx_index = 0U;
    }
}

/* ============================================================================
   ОБРАБОТКА MODBUS (Main Loop)
============================================================================ */
void ModBus_Process(void)
{
    static uint32_t last_process_time = 0U;
    uint32_t current_time = HAL_GetTick();

    if ((current_time - last_process_time) < 10U) {
        return;
    }
    last_process_time = current_time;

    if ((modbus_instance.rx_index > 0U) && ((current_time - modbus_instance.last_byte_time_ms) > MODBUS_RESPONSE_TIMEOUT_MS)) {
        ModBus_ProcessFrame();
    }
}

/* ============================================================================
   ОБНОВЛЕНИЕ ДАННЫХ НАПРЯЖЕНИЙ
============================================================================ */
void ModBus_UpdateVoltages(float vdda, float v24, float v12, float v5,
                           uint8_t vdda_err, uint8_t v24_err, uint8_t v12_err, uint8_t v5_err)
{
    modbus_instance.input_regs[REG_24V] = (v24_err ? 0xFFFFU : (uint16_t)(v24 * 10.0f + 0.5f));
    modbus_instance.input_regs[REG_12V] = (v12_err ? 0xFFFFU : (uint16_t)(v12 * 10.0f + 0.5f));
    modbus_instance.input_regs[REG_5V] = (v5_err ? 0xFFFFU : (uint16_t)(v5 * 10.0f + 0.5f));
    modbus_instance.input_regs[REG_VDDA] = (vdda_err ? 0xFFFFU : (uint16_t)(vdda * 10.0f + 0.5f));
}

/* ============================================================================
   ОБНОВЛЕНИЕ ДАННЫХ ИЗМЕРЕНИЙ
============================================================================ */
void ModBus_UpdateMeasurements(float tof_us, float temperature,
                               uint8_t tof_err, uint8_t temp_err, uint8_t status)
{
    static uint16_t counter = 0U;

    modbus_instance.input_regs[REG_TOF] = (tof_err ? 0xFFFFU : (uint16_t)(tof_us * 10.0f + 0.5f));
    modbus_instance.input_regs[REG_TEMPERATURE] = (temp_err ? 0xFFFFU : (uint16_t)(temperature * 10.0f + 0.5f));
    modbus_instance.input_regs[REG_STATUS] = status;
    modbus_instance.input_regs[REG_COUNTER] = counter++;

    if (counter > 65535U) {
        counter = 0U;
    }

    uint32_t timestamp = HAL_GetTick();
    modbus_instance.input_regs[REG_TIMESTAMP_HIGH] = ((timestamp >> 16U) & 0xFFFFU);
    modbus_instance.input_regs[REG_TIMESTAMP_LOW] = (timestamp & 0xFFFFU);
}

/* ============================================================================
   ПОДГОТОВКА К ПЕРЕДАЧЕ (Переключение RS485 в режим TX)
============================================================================ */
void ModBus_PrepareForTransmit(void)
{
    RS485_SET_TRANSMIT();
    modbus_tx_active = 1;
    for (volatile int i = 0; i < 150; i++) __NOP();
}

/* ============================================================================
   ЧТЕНИЕ FLOAT ПАРАМЕТРА ИЗ HOLDING REGISTERS
   Float хранится в двух регистрах (32 бита): addr (старшее) и addr+1 (младшее)
============================================================================ */
float ModBus_GetParameter_Float(uint8_t addr)
{
    if (addr >= (HOLD_HOLDING_REG_COUNT - 1)) {
        return 0.0f;
    }

    uint32_t raw = ((uint32_t)modbus_instance.holding_regs[addr] << 16) |
                   modbus_instance.holding_regs[addr + 1];

    float value;
    memcpy(&value, &raw, sizeof(float));

    return value;
}

/* ============================================================================
   ЗАПИСЬ FLOAT ПАРАМЕТРА В HOLDING REGISTERS
============================================================================ */
void ModBus_SetParameter_Float(uint8_t addr, float value)
{
    if (addr >= (HOLD_HOLDING_REG_COUNT - 1)) {
        return;
    }

    uint32_t raw;
    memcpy(&raw, &value, sizeof(float));

    modbus_instance.holding_regs[addr] = (uint16_t)((raw >> 16) & 0xFFFF);
    modbus_instance.holding_regs[addr + 1] = (uint16_t)(raw & 0xFFFF);
}

/* ============================================================================
   ПОЛУЧЕНИЕ ДЛИНЫ ВОЛНОВОДА (из регистра 10-11)
============================================================================ */
float ModBus_GetWaveguideLength(void)
{
    // Длина волновода хранится в регистрах 10-11 (float, 32 бита)
    return ModBus_GetParameter_Float(10);
}

/* ============================================================================
   ОБНОВЛЕНИЕ ВЕРСИИ ПРОШИВКИ (в регистр 12)
============================================================================ */
void ModBus_UpdateFirmwareVersion(uint16_t version)
{
    modbus_instance.input_regs[12] = version;
}

/* ============================================================================
   ГЕТТЕР АДРЕСА УСТРОЙСТВА
============================================================================ */
uint8_t ModBus_GetDeviceAddress(void)
{
    return modbus_instance.device_address;
}
