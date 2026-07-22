/**
 * @file           : modbus.c
 * @brief          : Modbus RTU implementation с EEPROM и обработкой ошибок
 */
#include "modbus.h"
#include "main.h"
#include "at24c64.h"
#include <string.h>

/* ==========================================================================
   КОНФИГУРАЦИЯ
   ========================================================================== */
#define MODBUS_REG_ARRAY_SIZE   260
#define INPUT_REGS_COUNT        128
#define HOLDING_REGS_COUNT      250

/* ==========================================================================
   СТРУКТУРА MODBUS
   ========================================================================== */
typedef struct {
    uint16_t regs[MODBUS_REG_ARRAY_SIZE];
    uint8_t  device_address;
    uint16_t rx_index;
    uint8_t  rx_buffer[MODBUS_BUFFER_SIZE];
    uint8_t  rx_byte;
    uint32_t last_byte_time;
} ModBus_Struct;

static ModBus_Struct modbus;

/* ==========================================================================
   EEPROM: отложенная запись
   ========================================================================== */
#define EEPROM_WRITE_INTERVAL_MS    1000
static volatile uint8_t  eeprom_dirty             = 0;
static volatile uint32_t eeprom_last_write_time   = 0;
static volatile uint8_t  eeprom_write_in_progress = 0;

/* ==========================================================================
   ПАРАМЕТРЫ ПО УМОЛЧАНИЮ
   ========================================================================== */
typedef struct { uint16_t address; float default_value; } Param_Float_Default;

static const Param_Float_Default default_float_params[] = {
    {MB_ADDR_WAVEGUIDE_LEN, 1.2f},    /* 1200 мм */
    {MB_ADDR_CAL_LOW_LVL,   0.1f},    /* 100 мм - ВЕРХНЯЯ точка (полный бак) */
    {MB_ADDR_CAL_HIGH_LVL,  1.0f},    /* 1000 мм - НИЖНЯЯ точка (пустой бак) */
    {MB_ADDR_PROBE_DEPTH,   0.0f},
    {MB_ADDR_TANK_HEIGHT,   0.95f},
    {MB_ADDR_DAMPING_TIME,  10.0f},
    {MB_ADDR_POLL_PERIOD,   1.0f},    /* 1 секунда */
    {MB_ADDR_LEVEL_OFFSET,  0.0f},
    {MB_ADDR_THRESH_LVL,    0.9f},
};

#define DEFAULT_FLOAT_PARAMS_COUNT (sizeof(default_float_params) / sizeof(default_float_params[0]))

typedef struct { uint16_t address; uint16_t default_value; } Param_Int_Default;

static const Param_Int_Default default_int_params[] = {
    {MB_ADDR_MB_ADDR_SET,   1},
    {MB_ADDR_MB_BAUD_SET,   19200},
    {MB_ADDR_MEDIUM_TYPE,   1},
    {MB_ADDR_UNIT_LEVEL,    0},
    {MB_ADDR_UNIT_TEMP,     0},
    {MB_ADDR_FW_VERSION,    100},
};

#define DEFAULT_INT_PARAMS_COUNT (sizeof(default_int_params) / sizeof(default_int_params[0]))

extern UART_HandleTypeDef huart1;

/* ==========================================================================
   CRC16
   ========================================================================== */
uint16_t ModBus_CRC16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    while (length--) {
        crc ^= *data++;
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else crc >>= 1;
        }
    }
    return crc;
}

/* ==========================================================================
   Конвертация float
   ========================================================================== */
static void FloatToRegisters(float value, uint16_t *reg_high, uint16_t *reg_low)
{
    union { float f; uint32_t u32; uint8_t bytes[4]; } converter;
    converter.f = value;
    *reg_high = (converter.u32 >> 16) & 0xFFFF;
    *reg_low  = converter.u32 & 0xFFFF;
}

static float RegistersToFloat(uint16_t reg_high, uint16_t reg_low)
{
    union { float f; uint32_t u32; } converter;
    converter.u32 = ((uint32_t)reg_high << 16) | reg_low;
    return converter.f;
}

/* ==========================================================================
   Адресация регистров
   ========================================================================== */
static uint16_t ModBus_AddressToIndex(uint16_t addr)
{
    if (addr >= 2000 && addr <= 2498) {
        uint16_t idx = (addr - 2000) / 2;
        if (idx < HOLDING_REGS_COUNT) return idx;
    }
    if (addr >= 999 && addr <= 1125) {
        uint16_t idx = (addr - 999) / 2;
        if (idx < INPUT_REGS_COUNT) return idx;
    }
    if (addr == 3000) return 250;
    if (addr == 3002) return 251;
    return 0xFFFF;
}

/* Экспорт для внешних модулей (temp_sensors, graduation) */
uint16_t ModBus_AddressToIndex_External(uint16_t addr)
{
    return ModBus_AddressToIndex(addr);
}

static uint8_t IsPersistentAddress(uint16_t addr)
{
    return (addr >= 2000 && addr <= 2498) ? 1 : 0;
}

/* ==========================================================================
   EEPROM ФУНКЦИИ
   ========================================================================== */
static void EEPROM_FlushIfNeeded(void)
{
    if (!eeprom_dirty || eeprom_write_in_progress) return;

    uint32_t now = HAL_GetTick();
    if ((now - eeprom_last_write_time) < EEPROM_WRITE_INTERVAL_MS) return;

    eeprom_write_in_progress = 1;
    eeprom_dirty = 0;
    eeprom_last_write_time = now;

    AT24C64_SaveAllRegisters(modbus.regs, HOLDING_REGS_COUNT);
    eeprom_write_in_progress = 0;
}

static void EEPROM_Initialize(void)
{
    if (AT24C64_Init(AT24C64_DEFAULT_ADDRESS) != HAL_OK) {
        USART2_Print("[EEPROM] AT24C64 НЕ НАЙДЕН! Работаем без сохранения.\r\n");
        return;
    }

    USART2_Print("[EEPROM] AT24C64 найдена (0x51)\r\n");

    if (!AT24C64_IsFormatted()) {
        USART2_Print("[EEPROM] Форматирование...\r\n");
        AT24C64_Format();

        for (int i = 0; i < DEFAULT_FLOAT_PARAMS_COUNT; i++)
            AT24C64_SaveFloatParam(default_float_params[i].address, default_float_params[i].default_value);
        for (int i = 0; i < DEFAULT_INT_PARAMS_COUNT; i++)
            AT24C64_SaveIntParam(default_int_params[i].address, default_int_params[i].default_value);

        AT24C64_SaveAllRegisters(modbus.regs, HOLDING_REGS_COUNT);
        USART2_Print("[EEPROM] Записаны дефолтные значения\r\n");
    } else {
        USART2_Print("[EEPROM] Загрузка параметров...\r\n");
        AT24C64_LoadAllRegisters(modbus.regs, HOLDING_REGS_COUNT);
    }

    USART2_Print("[EEPROM] Параметры:\r\n");
    USART2_Print("  Waveguide: ");
    USART2_BufInit();
    USART2_BufPrintFloat(ModBus_GetWaveguideLength());
    USART2_BufPrint(" м\r\n");

    float h_low = ModBus_GetParameter_Float(MB_ADDR_CAL_LOW_LVL);
    USART2_Print("  h_low (верхняя точка): ");
    USART2_BufInit();
    USART2_BufPrintFloat(h_low);
    USART2_BufPrint(" м (");
    USART2_BufPrintFloat(h_low * 1000.0f);
    USART2_BufPrint(" мм)\r\n");

    float h_high = ModBus_GetParameter_Float(MB_ADDR_CAL_HIGH_LVL);
    USART2_Print("  h_high (нижняя точка): ");
    USART2_BufPrintFloat(h_high);
    USART2_BufPrint(" м (");
    USART2_BufPrintFloat(h_high * 1000.0f);
    USART2_BufPrint(" мм)\r\n");
    USART2_BufFlush();
}

/* ==========================================================================
   ФУНКЦИИ ПАРАМЕТРОВ
   ========================================================================== */
float ModBus_GetWaveguideLength(void)
{
    uint16_t idx = ModBus_AddressToIndex(MB_ADDR_WAVEGUIDE_LEN);
    if (idx != 0xFFFF) {
        float value = RegistersToFloat(modbus.regs[idx], modbus.regs[idx + 1]);
        if (value < 0.1f || value > 50.0f) return 1.2f;
        return value;
    }
    return 1.2f;
}

void ModBus_SetWaveguideLength(float length_m)
{
    if (length_m >= 0.1f && length_m <= 50.0f) {
        uint16_t idx = ModBus_AddressToIndex(MB_ADDR_WAVEGUIDE_LEN);
        if (idx != 0xFFFF) {
            FloatToRegisters(length_m, &modbus.regs[idx], &modbus.regs[idx + 1]);
            eeprom_dirty = 1;
        }
    }
}

float ModBus_GetParameter_Float(uint16_t addr)
{
    uint16_t idx = ModBus_AddressToIndex(addr);
    if (idx != 0xFFFF && (idx + 1) < MODBUS_REG_ARRAY_SIZE)
        return RegistersToFloat(modbus.regs[idx], modbus.regs[idx + 1]);
    return 0.0f;
}

void ModBus_SetParameter_Float(uint16_t addr, float value)
{
    uint16_t idx = ModBus_AddressToIndex(addr);
    if (idx != 0xFFFF && (idx + 1) < MODBUS_REG_ARRAY_SIZE) {
        FloatToRegisters(value, &modbus.regs[idx], &modbus.regs[idx + 1]);
        if (IsPersistentAddress(addr)) eeprom_dirty = 1;
    }
}

uint16_t ModBus_GetParameter_Int(uint16_t addr)
{
    uint16_t idx = ModBus_AddressToIndex(addr);
    if (idx != 0xFFFF && idx < MODBUS_REG_ARRAY_SIZE) return modbus.regs[idx];
    return 0;
}

void ModBus_SetParameter_Int(uint16_t addr, uint16_t value)
{
    uint16_t idx = ModBus_AddressToIndex(addr);
    if (idx != 0xFFFF && idx < MODBUS_REG_ARRAY_SIZE) {
        modbus.regs[idx] = value;
        if (IsPersistentAddress(addr)) eeprom_dirty = 1;
    }
}

uint32_t ModBus_GetPulseWidthIterations(void) { return 10; }

/* ==========================================================================
   MODBUS ФУНКЦИИ
   ========================================================================== */
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

static void ModBus_ReadHoldingRegisters(uint16_t start_addr, uint16_t reg_count)
{
    if (!((start_addr >= 2000 && start_addr <= 2498) ||
          start_addr == 3000 || start_addr == 3002)) {
        ModBus_SendException(0x03, 0x02); return;
    }
    if (reg_count == 0 || reg_count > 125) {
        ModBus_SendException(0x03, 0x03); return;
    }

    uint16_t idx = ModBus_AddressToIndex(start_addr);
    if (start_addr == 3000 || start_addr == 3002) {
        if (idx == 0xFFFF || (idx + reg_count) > MODBUS_REG_ARRAY_SIZE) {
            ModBus_SendException(0x03, 0x02); return;
        }
    } else {
        if (idx == 0xFFFF || (idx + reg_count) > HOLDING_REGS_COUNT) {
            ModBus_SendException(0x03, 0x02); return;
        }
    }

    uint8_t response[256];
    uint16_t index = 0;

    response[index++] = modbus.device_address;
    response[index++] = 0x03;
    response[index++] = reg_count * 2;

    for (uint16_t i = 0; i < reg_count; i++) {
        uint16_t val = modbus.regs[idx + i];
        response[index++] = (val >> 8) & 0xFF;
        response[index++] = val & 0xFF;
    }

    uint16_t crc = ModBus_CRC16(response, index);
    response[index++] = crc & 0xFF;
    response[index++] = (crc >> 8) & 0xFF;

    ModBus_TransmitFrame(response, index);
}

static void ModBus_ReadInputRegisters(uint16_t start_addr, uint16_t reg_count)
{
    if (start_addr < 999 || start_addr > 1125) { ModBus_SendException(0x04, 0x02); return; }
    if (reg_count == 0 || reg_count > 125) { ModBus_SendException(0x04, 0x03); return; }

    uint16_t idx = ModBus_AddressToIndex(start_addr);
    if (idx == 0xFFFF || (idx + reg_count) > INPUT_REGS_COUNT) { ModBus_SendException(0x04, 0x02); return; }

    uint8_t response[256];
    uint16_t index = 0;

    response[index++] = modbus.device_address;
    response[index++] = 0x04;
    response[index++] = reg_count * 2;

    for (uint16_t i = 0; i < reg_count; i++) {
        uint16_t val = modbus.regs[idx + i];
        response[index++] = (val >> 8) & 0xFF;
        response[index++] = val & 0xFF;
    }

    uint16_t crc = ModBus_CRC16(response, index);
    response[index++] = crc & 0xFF;
    response[index++] = (crc >> 8) & 0xFF;

    ModBus_TransmitFrame(response, index);
}

static void ModBus_WriteSingleRegister(uint16_t reg_addr, uint16_t value)
{
    if (!((reg_addr >= 2000 && reg_addr <= 2498) ||
          reg_addr == 3000 || reg_addr == 3002)) {
        ModBus_SendException(0x06, 0x02); return;
    }

    uint16_t idx = ModBus_AddressToIndex(reg_addr);
    if (idx == 0xFFFF || idx >= MODBUS_REG_ARRAY_SIZE) {
        ModBus_SendException(0x06, 0x02); return;
    }

    modbus.regs[idx] = value;
    if (IsPersistentAddress(reg_addr)) eeprom_dirty = 1;

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

static void ModBus_WriteMultipleRegisters(uint16_t start_addr, uint16_t reg_count, uint8_t *data)
{
    if (!((start_addr >= 2000 && start_addr <= 2498) ||
          start_addr == 3000 || start_addr == 3002)) {
        ModBus_SendException(0x10, 0x02); return;
    }
    if (reg_count == 0 || reg_count > 123) { ModBus_SendException(0x10, 0x03); return; }

    uint16_t idx = ModBus_AddressToIndex(start_addr);
    if (idx == 0xFFFF || (idx + reg_count) > HOLDING_REGS_COUNT) { ModBus_SendException(0x10, 0x02); return; }

    for (uint16_t i = 0; i < reg_count; i++) {
        uint16_t val = ((uint16_t)data[i * 2] << 8) | data[i * 2 + 1];
        modbus.regs[idx + i] = val;
    }

    if (IsPersistentAddress(start_addr)) eeprom_dirty = 1;

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

static void ModBus_ProcessFrame(void)
{
    if (modbus.rx_index < 4) { modbus.rx_index = 0; return; }

    uint16_t received_crc = ((uint16_t)modbus.rx_buffer[modbus.rx_index - 1] << 8) |
                            modbus.rx_buffer[modbus.rx_index - 2];
    uint16_t calculated_crc = ModBus_CRC16(modbus.rx_buffer, modbus.rx_index - 2);

    if (received_crc != calculated_crc) { modbus.rx_index = 0; return; }

    uint8_t device_addr = modbus.rx_buffer[0];
    if (device_addr != modbus.device_address && device_addr != 0) {
        modbus.rx_index = 0; return;
    }

    switch (modbus.rx_buffer[1]) {
        case 0x03:
            if (modbus.rx_index >= 8) {
                uint16_t start_addr = ((uint16_t)modbus.rx_buffer[2] << 8) | modbus.rx_buffer[3];
                uint16_t reg_count = ((uint16_t)modbus.rx_buffer[4] << 8) | modbus.rx_buffer[5];
                ModBus_ReadHoldingRegisters(start_addr, reg_count);
            }
            break;
        case 0x04:
            if (modbus.rx_index >= 8) {
                uint16_t start_addr = ((uint16_t)modbus.rx_buffer[2] << 8) | modbus.rx_buffer[3];
                uint16_t reg_count = ((uint16_t)modbus.rx_buffer[4] << 8) | modbus.rx_buffer[5];
                ModBus_ReadInputRegisters(start_addr, reg_count);
            }
            break;
        case 0x06:
            if (modbus.rx_index >= 8) {
                uint16_t reg_addr = ((uint16_t)modbus.rx_buffer[2] << 8) | modbus.rx_buffer[3];
                uint16_t reg_value = ((uint16_t)modbus.rx_buffer[4] << 8) | modbus.rx_buffer[5];
                ModBus_WriteSingleRegister(reg_addr, reg_value);
            }
            break;
        case 0x10:
            if (modbus.rx_index >= 9) {
                uint16_t start_addr = ((uint16_t)modbus.rx_buffer[2] << 8) | modbus.rx_buffer[3];
                uint16_t reg_count = ((uint16_t)modbus.rx_buffer[4] << 8) | modbus.rx_buffer[5];
                ModBus_WriteMultipleRegisters(start_addr, reg_count, &modbus.rx_buffer[7]);
            }
            break;
        default:
            ModBus_SendException(modbus.rx_buffer[1], 0x01);
            break;
    }

    modbus.rx_index = 0;
}

/* ==========================================================================
   ИНИЦИАЛИЗАЦИЯ
   ========================================================================== */
void ModBus_Init(void)
{
    memset(&modbus, 0, sizeof(modbus));
    modbus.device_address = MODBUS_DEFAULT_ADDRESS;
    modbus.last_byte_time = HAL_GetTick();

    EEPROM_Initialize();

    for (int i = 0; i < DEFAULT_FLOAT_PARAMS_COUNT; i++) {
        uint16_t idx = ModBus_AddressToIndex(default_float_params[i].address);
        if (idx != 0xFFFF) {
            float cur = RegistersToFloat(modbus.regs[idx], modbus.regs[idx + 1]);
            if (cur != cur || cur < -1e9f || cur > 1e9f) {
                ModBus_SetParameter_Float(default_float_params[i].address, default_float_params[i].default_value);
            }
        }
    }

    for (int i = 0; i < DEFAULT_INT_PARAMS_COUNT; i++) {
        uint16_t idx = ModBus_AddressToIndex(default_int_params[i].address);
        if (idx != 0xFFFF && modbus.regs[idx] == 0)
            ModBus_SetParameter_Int(default_int_params[i].address, default_int_params[i].default_value);
    }

    eeprom_dirty = 0;
    HAL_UART_Receive_IT(&huart1, &modbus.rx_byte, 1);
}

/* ==========================================================================
   CALLBACK ПРИЕМА
   ========================================================================== */
void ModBus_RxCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        uint32_t current_time = HAL_GetTick();

        if (current_time - modbus.last_byte_time > 2) {
            modbus.rx_index = 0;
        }
        modbus.last_byte_time = current_time;

        if (modbus.rx_index < MODBUS_BUFFER_SIZE) {
            modbus.rx_buffer[modbus.rx_index++] = modbus.rx_byte;
        }

        HAL_UART_Receive_IT(&huart1, &modbus.rx_byte, 1);
    }
}

void ModBus_RestartRx(void) { HAL_UART_Receive_IT(&huart1, &modbus.rx_byte, 1); }

/* ==========================================================================
   ОБРАБОТКА MODBUS
   ========================================================================== */
void ModBus_Process(void)
{
    if (modbus.rx_index > 0) {
        uint32_t current_time = HAL_GetTick();
        if (current_time - modbus.last_byte_time > 5) {
            ModBus_ProcessFrame();
        }
    }
    EEPROM_FlushIfNeeded();
}

/* ==========================================================================
   ОБНОВЛЕНИЕ ДАННЫХ
   ========================================================================== */
void ModBus_UpdateVoltages(float vdda, float v24, float v12, float v5)
{
    FloatToRegisters(vdda, &modbus.regs[50], &modbus.regs[51]);
    FloatToRegisters(v24,  &modbus.regs[52], &modbus.regs[53]);
    FloatToRegisters(v12,  &modbus.regs[54], &modbus.regs[55]);
    FloatToRegisters(v5,   &modbus.regs[56], &modbus.regs[57]);
}

void ModBus_UpdateMeasurements(float level, float temp, float waveguide)
{
    FloatToRegisters(level, &modbus.regs[0], &modbus.regs[1]);
    FloatToRegisters(temp,  &modbus.regs[2], &modbus.regs[3]);

    float level_pct = (waveguide > 0.0f) ? (level / waveguide) * 100.0f : 0.0f;
    FloatToRegisters(level_pct, &modbus.regs[4], &modbus.regs[5]);
}

void ModBus_UpdateFirmwareVersion(uint16_t version)
{
    ModBus_SetParameter_Int(MB_ADDR_FW_VERSION, version);
}

void ModBus_ForceSaveToEEPROM(void)
{
    if (eeprom_dirty) {
        AT24C64_SaveAllRegisters(modbus.regs, HOLDING_REGS_COUNT);
        eeprom_dirty = 0;
        eeprom_last_write_time = HAL_GetTick();
    }
}
