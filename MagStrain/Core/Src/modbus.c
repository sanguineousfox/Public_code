#include "modbus.h"
#include "main.h"
#include "utils.h"
#include <string.h>

/* Структура данных ModBus */
typedef struct {
    uint16_t holding_regs[HOLD_HOLDING_REG_COUNT];
    uint16_t input_regs[REG_INPUT_REG_COUNT];
    uint8_t device_address;
    uint16_t rx_index;
    uint8_t rx_buffer[MODBUS_BUFFER_SIZE];
    uint8_t rx_byte;
    uint32_t last_byte_time;
} ModBus_Struct;

static ModBus_Struct modbus;

/* Внешние переменные */
extern UART_HandleTypeDef huart1;
extern void USART2_Print(const char* str);

/* CRC16 */
uint16_t ModBus_CRC16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    while(length--) {
        crc ^= *data++;
        for(uint8_t bit = 0; bit < 8; bit++) {
            if(crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/* Отправка исключения */
static void ModBus_SendException(uint8_t function, uint8_t exception_code)
{
    uint8_t response[5];
    response[0] = modbus.device_address;
    response[1] = function | 0x80;
    response[2] = exception_code;
    uint16_t crc = ModBus_CRC16(response, 3);
    response[3] = crc & 0xFF;
    response[4] = (crc >> 8) & 0xFF;

    /* Передача с управлением RS485 */
    ModBus_TransmitFrame(response, 5);
}

/* Float -> 2 регистра */
static void FloatToRegisters(float value, uint16_t *reg_high, uint16_t *reg_low)
{
    union {
        float f;
        uint32_t u32;
    } converter;
    converter.f = value;
    *reg_high = (converter.u32 >> 16) & 0xFFFF;
    *reg_low = converter.u32 & 0xFFFF;
}

/* Чтение Holding Registers (0x03) */
static void ModBus_ReadHoldingRegisters(uint16_t start_addr, uint16_t reg_count)
{
    if (start_addr >= HOLD_HOLDING_REG_COUNT) {
        ModBus_SendException(0x03, 0x02);
        return;
    }
    if (reg_count == 0 || reg_count > 125) {
        ModBus_SendException(0x03, 0x03);
        return;
    }
    if (start_addr + reg_count > HOLD_HOLDING_REG_COUNT) {
        ModBus_SendException(0x03, 0x02);
        return;
    }

    uint8_t response[256];
    uint16_t index = 0;
    response[index++] = modbus.device_address;
    response[index++] = 0x03;
    response[index++] = reg_count * 2;

    for(uint16_t i = 0; i < reg_count; i++) {
        uint16_t val = modbus.holding_regs[start_addr + i];
        response[index++] = (val >> 8) & 0xFF;
        response[index++] = val & 0xFF;
    }

    uint16_t crc = ModBus_CRC16(response, index);
    response[index++] = crc & 0xFF;
    response[index++] = (crc >> 8) & 0xFF;

    ModBus_TransmitFrame(response, index);
}

/* Чтение Input Registers (0x04) */
static void ModBus_ReadInputRegisters(uint16_t start_addr, uint16_t reg_count)
{
    if (start_addr >= REG_INPUT_REG_COUNT) {
        ModBus_SendException(0x04, 0x02);
        return;
    }
    if (reg_count == 0 || reg_count > 125) {
        ModBus_SendException(0x04, 0x03);
        return;
    }
    if (start_addr + reg_count > REG_INPUT_REG_COUNT) {
        ModBus_SendException(0x04, 0x02);
        return;
    }

    uint8_t response[256];
    uint16_t index = 0;
    response[index++] = modbus.device_address;
    response[index++] = 0x04;
    response[index++] = reg_count * 2;

    for(uint16_t i = 0; i < reg_count; i++) {
        uint16_t val = modbus.input_regs[start_addr + i];
        response[index++] = (val >> 8) & 0xFF;
        response[index++] = val & 0xFF;
    }

    uint16_t crc = ModBus_CRC16(response, index);
    response[index++] = crc & 0xFF;
    response[index++] = (crc >> 8) & 0xFF;

    ModBus_TransmitFrame(response, index);
}

/* Запись одного регистра (0x06) */
static void ModBus_WriteSingleRegister(uint16_t reg_addr, uint16_t value)
{
    if (reg_addr >= HOLD_HOLDING_REG_COUNT) {
        ModBus_SendException(0x06, 0x02);
        return;
    }

    modbus.holding_regs[reg_addr] = value;

    uint8_t response[8];
    uint16_t index = 0;
    response[index++] = modbus.device_address;
    response[index++] = 0x06;
    response[index++] = (reg_addr >> 8) & 0xFF;
    response[index++] = reg_addr & 0xFF;
    response[index++] = (value >> 8) & 0xFF;
    response[index++] = value & 0xFF;

    uint16_t crc = ModBus_CRC16(response, index);
    response[index++] = crc & 0xFF;
    response[index++] = (crc >> 8) & 0xFF;

    ModBus_TransmitFrame(response, index);
}

/* Обработка фрейма */
static void ModBus_ProcessFrame(void)
{
    if (modbus.rx_index < 4) {
        modbus.rx_index = 0;
        return;
    }

    /* Проверка CRC */
    uint16_t received_crc = (modbus.rx_buffer[modbus.rx_index - 1] << 8) |
                           modbus.rx_buffer[modbus.rx_index - 2];
    uint16_t calculated_crc = ModBus_CRC16(modbus.rx_buffer, modbus.rx_index - 2);

    if (received_crc != calculated_crc) {
        modbus.rx_index = 0;
        return;
    }

    /* Проверка адреса */
    uint8_t device_addr = modbus.rx_buffer[0];
    if (device_addr != modbus.device_address && device_addr != 0) {
        modbus.rx_index = 0;
        return;
    }

    /* Обработка функций */
    switch(modbus.rx_buffer[1]) {
        case 0x03:
            if (modbus.rx_index >= 8) {
                uint16_t start_addr = (modbus.rx_buffer[2] << 8) | modbus.rx_buffer[3];
                uint16_t reg_count = (modbus.rx_buffer[4] << 8) | modbus.rx_buffer[5];
                ModBus_ReadHoldingRegisters(start_addr, reg_count);
            }
            break;

        case 0x04:
            if (modbus.rx_index >= 8) {
                uint16_t start_addr = (modbus.rx_buffer[2] << 8) | modbus.rx_buffer[3];
                uint16_t reg_count = (modbus.rx_buffer[4] << 8) | modbus.rx_buffer[5];
                ModBus_ReadInputRegisters(start_addr, reg_count);
            }
            break;

        case 0x06:
            if (modbus.rx_index >= 8) {
                uint16_t reg_addr = (modbus.rx_buffer[2] << 8) | modbus.rx_buffer[3];
                uint16_t reg_value = (modbus.rx_buffer[4] << 8) | modbus.rx_buffer[5];
                ModBus_WriteSingleRegister(reg_addr, reg_value);
            }
            break;

        default:
            ModBus_SendException(modbus.rx_buffer[1], 0x01);
            break;
    }

    modbus.rx_index = 0;
}

/* Инициализация */
void ModBus_Init(void)
{
    memset(&modbus, 0, sizeof(modbus));
    modbus.device_address = 1;
    modbus.last_byte_time = HAL_GetTick();

    /* Инициализация Holding регистров */
    for (int i = 0; i < HOLD_HOLDING_REG_COUNT; i++) {
        modbus.holding_regs[i] = 0;
    }
    modbus.holding_regs[0] = 1;  /* Адрес устройства */

    /* Инициализация Input регистров */
    for (int i = 0; i < REG_INPUT_REG_COUNT; i++) {
        modbus.input_regs[i] = 0;
    }

    /* Стартовые значения напряжений */
    FloatToRegisters(3.3f, &modbus.input_regs[0], &modbus.input_regs[1]);  /* VDDA */
    FloatToRegisters(24.0f, &modbus.input_regs[2], &modbus.input_regs[3]); /* 24V */
    FloatToRegisters(12.0f, &modbus.input_regs[4], &modbus.input_regs[5]); /* 12V */
    FloatToRegisters(5.0f, &modbus.input_regs[6], &modbus.input_regs[7]);  /* 5V */

    /* Запуск приема */
    HAL_UART_Receive_IT(&huart1, &modbus.rx_byte, 1);

    USART2_Print("[MODBUS] Init OK, Addr=1\r\n");
}

/* Callback приема */
void ModBus_RxCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        uint32_t current_time = HAL_GetTick();

        /* Проверка межкадрового интервала (3.5 символа ~ 4мс) */
        if (current_time - modbus.last_byte_time > 4) {
            modbus.rx_index = 0;  /* Новый фрейм */
        }
        modbus.last_byte_time = current_time;

        /* Сохранение байта */
        if (modbus.rx_index < MODBUS_BUFFER_SIZE) {
            modbus.rx_buffer[modbus.rx_index++] = modbus.rx_byte;
        }

        /* Прием следующего байта */
        HAL_UART_Receive_IT(&huart1, &modbus.rx_byte, 1);
    }
}

/* Обработка в цикле */
void ModBus_Process(void)
{
    if (modbus.rx_index > 0) {
        uint32_t current_time = HAL_GetTick();

        /* Таймаут конца фрейма (5мс) */
        if (current_time - modbus.last_byte_time > 5) {
            ModBus_ProcessFrame();
        }
    }
}

/* Обновление напряжений */
void ModBus_UpdateVoltages(float vdda, float v24, float v12, float v5)
{
    FloatToRegisters(vdda, &modbus.input_regs[0], &modbus.input_regs[1]);
    FloatToRegisters(v24, &modbus.input_regs[2], &modbus.input_regs[3]);
    FloatToRegisters(v12, &modbus.input_regs[4], &modbus.input_regs[5]);
    FloatToRegisters(v5, &modbus.input_regs[6], &modbus.input_regs[7]);
}

/* Обновление измерений */
void ModBus_UpdateMeasurements(float level, float temp, float waveguide)
{
    FloatToRegisters(level, &modbus.input_regs[10], &modbus.input_regs[11]);
    FloatToRegisters(temp, &modbus.input_regs[12], &modbus.input_regs[13]);
    FloatToRegisters(waveguide, &modbus.input_regs[14], &modbus.input_regs[15]);
}

/* Обновление версии */
void ModBus_UpdateFirmwareVersion(uint16_t version)
{
    modbus.holding_regs[10] = version;
}
