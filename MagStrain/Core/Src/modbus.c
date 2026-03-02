/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : modbus.c
  * @brief          : Modbus RTU implementation (ПМП-201Е)
  *                 : Поддержка чтения (0x03, 0x04) и записи (0x06, 0x10)
  ******************************************************************************
  */
/* USER CODE END Header */

#include "modbus.h"
#include "main.h"
#include <string.h>

/* ==========================================================================
 * КОНФИГУРАЦИЯ ПАМЯТИ
 * ========================================================================== */
#define MODBUS_REG_ARRAY_SIZE   256  /* Достаточно для адресов до 2512 */

/* ==========================================================================
 * СТРУКТУРА ДАННЫХ MODBUS
 * ========================================================================== */
typedef struct {
    uint16_t regs[MODBUS_REG_ARRAY_SIZE];
    uint8_t device_address;
    uint16_t rx_index;
    uint8_t rx_buffer[MODBUS_BUFFER_SIZE];
    uint8_t rx_byte;
    uint32_t last_byte_time;
} ModBus_Struct;

static ModBus_Struct modbus;

/* ==========================================================================
 * ПАРАМЕТРЫ ПО УМОЛЧАНИЮ
 * ========================================================================== */

typedef struct {
    uint16_t address;
    float default_value;
} Param_Float_Default;

static const Param_Float_Default default_float_params[] = {
    {MB_ADDR_WAVEGUIDE_LEN,   3000.0f},   /* 2096: Длина звукопровода текущая (Lc), м */
    {MB_ADDR_CAL_LOW_LVL,     0.0f},      /* 2000: Нижняя контрольная калибровочная точка уровня (h_), м */
    {MB_ADDR_CAL_HIGH_LVL,    100.0f},    /* 2002: Верхняя контрольная калибровочная точка уровня (h¯), м */
    {MB_ADDR_PROBE_DEPTH,     0.0f},      /* 2004: Глубина погружения поплавка уровня (d1), м */
    {MB_ADDR_TANK_HEIGHT,     10000.0f},  /* 2010: Высота(диаметр) резервуара (H), м */
    {MB_ADDR_DAMPING_TIME,    10.0f},     /* 2086: Постоянная времени демпфирования измерений уровня (dt), с */
    {MB_ADDR_POLL_PERIOD,     10.0f},     /* 2088: Период опроса (пользовательский параметр), с ★ */
    {MB_ADDR_LEVEL_OFFSET,    0.0f},      /* 2120: Поправка измерений уровня (dh), м */
    {MB_ADDR_THRESH_LVL,      50.0f},     /* 2040: Порог обнуления показаний уровня (d7), м */
};

#define DEFAULT_FLOAT_PARAMS_COUNT (sizeof(default_float_params) / sizeof(default_float_params[0]))

#define DEFAULT_FLOAT_PARAMS_COUNT (sizeof(default_float_params) / sizeof(default_float_params[0]))

typedef struct {
    uint16_t address;
    uint16_t default_value;
} Param_Int_Default;

static const Param_Int_Default default_int_params[] = {
    {MB_ADDR_MB_ADDR_SET,     1},
    {MB_ADDR_MB_BAUD_SET,     19200},
    {MB_ADDR_MEDIUM_TYPE,     1},
    {MB_ADDR_UNIT_LEVEL,      0},
    {MB_ADDR_UNIT_TEMP,       0},
    {MB_ADDR_FW_VERSION,      100},
};

#define DEFAULT_INT_PARAMS_COUNT (sizeof(default_int_params) / sizeof(default_int_params[0]))

/* Внешние переменные */
extern UART_HandleTypeDef huart1;

/* ==========================================================================
 * ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
 * ========================================================================== */

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

static float RegistersToFloat(uint16_t reg_high, uint16_t reg_low)
{
    union {
        float f;
        uint32_t u32;
    } converter;
    converter.u32 = ((uint32_t)reg_high << 16) | reg_low;
    return converter.f;
}

/* ==========================================================================
 * ПРЕОБРАЗОВАНИЕ АДРЕСОВ
 * ========================================================================== */

static uint16_t ModBus_AddressToIndex(uint16_t addr)
{
    /* Группа 3 и 4: Адреса 2000-2418 -> Индексы 0-209 */
    if (addr >= 2000 && addr <= 2418) {
        uint16_t idx = (addr - 2000) / 2;
        if (idx < MODBUS_REG_ARRAY_SIZE) {
            return idx;
        }
    }

    /* Группа 2: Адреса 1000-1040 -> Индексы 0-20 */
    if (addr >= 1000 && addr <= 1040) {
        uint16_t idx = (addr - 1000) / 2;
        if (idx < 32) {
            return idx;
        }
    }

    return 0xFFFF;
}

/* ==========================================================================
 * ФУНКЦИИ ДЛЯ РАБОТЫ С ПАРАМЕТРАМИ
 * ========================================================================== */

float ModBus_GetWaveguideLength(void)
{
    uint16_t idx = ModBus_AddressToIndex(MB_ADDR_WAVEGUIDE_LEN);
    if (idx != 0xFFFF) {
        return RegistersToFloat(modbus.regs[idx], modbus.regs[idx + 1]);
    }
    return 6000.0f;
}

void ModBus_SetWaveguideLength(float length_mm)
{
    if (length_mm >= 100.0f && length_mm <= 50000.0f) {
        uint16_t idx = ModBus_AddressToIndex(MB_ADDR_WAVEGUIDE_LEN);
        if (idx != 0xFFFF) {
            FloatToRegisters(length_mm, &modbus.regs[idx], &modbus.regs[idx + 1]);
        }
    }
}

float ModBus_GetParameter_Float(uint16_t addr)
{
    uint16_t idx = ModBus_AddressToIndex(addr);
    if (idx != 0xFFFF && (idx + 1) < MODBUS_REG_ARRAY_SIZE) {
        return RegistersToFloat(modbus.regs[idx], modbus.regs[idx + 1]);
    }
    return 0.0f;
}

void ModBus_SetParameter_Float(uint16_t addr, float value)
{
    uint16_t idx = ModBus_AddressToIndex(addr);
    if (idx != 0xFFFF && (idx + 1) < MODBUS_REG_ARRAY_SIZE) {
        FloatToRegisters(value, &modbus.regs[idx], &modbus.regs[idx + 1]);
    }
}

uint16_t ModBus_GetParameter_Int(uint16_t addr)
{
    uint16_t idx = ModBus_AddressToIndex(addr);
    if (idx != 0xFFFF && idx < MODBUS_REG_ARRAY_SIZE) {
        return modbus.regs[idx];
    }
    return 0;
}

void ModBus_SetParameter_Int(uint16_t addr, uint16_t value)
{
    uint16_t idx = ModBus_AddressToIndex(addr);
    if (idx != 0xFFFF && idx < MODBUS_REG_ARRAY_SIZE) {
        modbus.regs[idx] = value;
    }
}

/* ==========================================================================
 * ФУНКЦИИ MODBUS
 * ========================================================================== */

static void ModBus_SendException(uint8_t function, uint8_t exception_code)
{
    uint8_t response[5];
    response[0] = modbus.device_address;
    response[1] = function | 0x80;
    response[2] = exception_code;
    uint16_t crc = ModBus_CRC16(response, 3);
    response[3] = crc & 0xFF;
    response[4] = (crc >> 8) & 0xFF;

    ModBus_TransmitFrame(response, 5);
}

/* Чтение Holding Registers (0x03) */
static void ModBus_ReadHoldingRegisters(uint16_t start_addr, uint16_t reg_count)
{
    if (start_addr < 2000 || start_addr > 2418) {
        ModBus_SendException(0x03, 0x02);
        return;
    }

    if (reg_count == 0 || reg_count > 125) {
        ModBus_SendException(0x03, 0x03);
        return;
    }

    uint16_t idx = ModBus_AddressToIndex(start_addr);
    if (idx == 0xFFFF || (idx + reg_count) > MODBUS_REG_ARRAY_SIZE) {
        ModBus_SendException(0x03, 0x02);
        return;
    }

    uint8_t response[256];
    uint16_t index = 0;
    response[index++] = modbus.device_address;
    response[index++] = 0x03;
    response[index++] = reg_count * 2;

    for(uint16_t i = 0; i < reg_count; i++) {
        uint16_t val = modbus.regs[idx + i];
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
    if (start_addr < 1000 || start_addr > 1040) {
        ModBus_SendException(0x04, 0x02);
        return;
    }

    if (reg_count == 0 || reg_count > 125) {
        ModBus_SendException(0x04, 0x03);
        return;
    }

    uint16_t idx = ModBus_AddressToIndex(start_addr);
    if (idx == 0xFFFF || (idx + reg_count) > 32) {
        ModBus_SendException(0x04, 0x02);
        return;
    }

    uint8_t response[256];
    uint16_t index = 0;
    response[index++] = modbus.device_address;
    response[index++] = 0x04;
    response[index++] = reg_count * 2;

    for(uint16_t i = 0; i < reg_count; i++) {
        uint16_t val = modbus.regs[idx + i];
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
    if (reg_addr < 2000 || reg_addr > 2418) {
        ModBus_SendException(0x06, 0x02);
        return;
    }

    uint16_t idx = ModBus_AddressToIndex(reg_addr);
    if (idx == 0xFFFF || idx >= MODBUS_REG_ARRAY_SIZE) {
        ModBus_SendException(0x06, 0x02);
        return;
    }

    modbus.regs[idx] = value;

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

/* Запись нескольких регистров (0x10) ★ НОВОЕ ★ */
static void ModBus_WriteMultipleRegisters(uint16_t start_addr, uint16_t reg_count, uint8_t *data)
{
    if (start_addr < 2000 || start_addr > 2418) {
        ModBus_SendException(0x10, 0x02);
        return;
    }

    if (reg_count == 0 || reg_count > 123) {
        ModBus_SendException(0x10, 0x03);
        return;
    }

    uint16_t idx = ModBus_AddressToIndex(start_addr);
    if (idx == 0xFFFF || (idx + reg_count) > MODBUS_REG_ARRAY_SIZE) {
        ModBus_SendException(0x10, 0x02);
        return;
    }

    /* Запись данных */
    for(uint16_t i = 0; i < reg_count; i++) {
        uint16_t val = (data[i * 2] << 8) | data[i * 2 + 1];
        modbus.regs[idx + i] = val;
    }

    /* Ответ: эхо запроса */
    uint8_t response[8];
    uint16_t index = 0;
    response[index++] = modbus.device_address;
    response[index++] = 0x10;
    response[index++] = (start_addr >> 8) & 0xFF;
    response[index++] = start_addr & 0xFF;
    response[index++] = (reg_count >> 8) & 0xFF;
    response[index++] = reg_count & 0xFF;

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
        case 0x03:  /* Read Holding Registers */
            if (modbus.rx_index >= 8) {
                uint16_t start_addr = (modbus.rx_buffer[2] << 8) | modbus.rx_buffer[3];
                uint16_t reg_count = (modbus.rx_buffer[4] << 8) | modbus.rx_buffer[5];
                ModBus_ReadHoldingRegisters(start_addr, reg_count);
            }
            break;

        case 0x04:  /* Read Input Registers */
            if (modbus.rx_index >= 8) {
                uint16_t start_addr = (modbus.rx_buffer[2] << 8) | modbus.rx_buffer[3];
                uint16_t reg_count = (modbus.rx_buffer[4] << 8) | modbus.rx_buffer[5];
                ModBus_ReadInputRegisters(start_addr, reg_count);
            }
            break;

        case 0x06:  /* Write Single Register */
            if (modbus.rx_index >= 8) {
                uint16_t reg_addr = (modbus.rx_buffer[2] << 8) | modbus.rx_buffer[3];
                uint16_t reg_value = (modbus.rx_buffer[4] << 8) | modbus.rx_buffer[5];
                ModBus_WriteSingleRegister(reg_addr, reg_value);
            }
            break;

        case 0x10:  /* Write Multiple Registers ★ НОВОЕ ★ */
            if (modbus.rx_index >= 9) {
                uint16_t start_addr = (modbus.rx_buffer[2] << 8) | modbus.rx_buffer[3];
                uint16_t reg_count = (modbus.rx_buffer[4] << 8) | modbus.rx_buffer[5];
                uint8_t byte_count = modbus.rx_buffer[6];
                /* Данные начинаются с индекса 7 */
                ModBus_WriteMultipleRegisters(start_addr, reg_count, &modbus.rx_buffer[7]);
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
    modbus.device_address = MODBUS_DEFAULT_ADDRESS;
    modbus.last_byte_time = HAL_GetTick();

    /* Очистка всех регистров */
    for (int i = 0; i < MODBUS_REG_ARRAY_SIZE; i++) {
        modbus.regs[i] = 0;
    }

    /* Запись параметров по умолчанию */
    for (int i = 0; i < DEFAULT_FLOAT_PARAMS_COUNT; i++) {
        ModBus_SetParameter_Float(default_float_params[i].address, default_float_params[i].default_value);
    }

    for (int i = 0; i < DEFAULT_INT_PARAMS_COUNT; i++) {
        ModBus_SetParameter_Int(default_int_params[i].address, default_int_params[i].default_value);
    }

    /* Запуск приема */
    HAL_UART_Receive_IT(&huart1, &modbus.rx_byte, 1);
}

/* Callback приема */
void ModBus_RxCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        uint32_t current_time = HAL_GetTick();

        if (current_time - modbus.last_byte_time > 4) {
            modbus.rx_index = 0;
        }
        modbus.last_byte_time = current_time;

        if (modbus.rx_index < MODBUS_BUFFER_SIZE) {
            modbus.rx_buffer[modbus.rx_index++] = modbus.rx_byte;
        }

        HAL_UART_Receive_IT(&huart1, &modbus.rx_byte, 1);
    }
}

/* Обработка в цикле */
void ModBus_Process(void)
{
    if (modbus.rx_index > 0) {
        uint32_t current_time = HAL_GetTick();

        if (current_time - modbus.last_byte_time > 5) {
            ModBus_ProcessFrame();
        }
    }
}

/* Обновление напряжений */
void ModBus_UpdateVoltages(float vdda, float v24, float v12, float v5)
{
    FloatToRegisters(vdda, &modbus.regs[25], &modbus.regs[26]);
    FloatToRegisters(v24, &modbus.regs[27], &modbus.regs[28]);
    FloatToRegisters(v12, &modbus.regs[29], &modbus.regs[30]);
    FloatToRegisters(v5,  &modbus.regs[31], &modbus.regs[32]);
}

/* Обновление измерений */
void ModBus_UpdateMeasurements(float level, float temp, float waveguide)
{
    FloatToRegisters(level, &modbus.regs[0], &modbus.regs[1]);
    FloatToRegisters(temp, &modbus.regs[2], &modbus.regs[3]);

    float level_pct = 0.0f;
    if (waveguide > 0.0f) {
        level_pct = (level / waveguide) * 100.0f;
    }
    FloatToRegisters(level_pct, &modbus.regs[4], &modbus.regs[5]);

    ModBus_SetWaveguideLength(waveguide);
}

/* Обновление версии */
void ModBus_UpdateFirmwareVersion(uint16_t version)
{
    ModBus_SetParameter_Int(MB_ADDR_FW_VERSION, version);
}
