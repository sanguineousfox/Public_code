#ifndef CONFIG_H
#define CONFIG_H

#include "stm32f1xx.h"

// ============================================
// КОНФИГУРАЦИЯ УСТРОЙСТВА
// ============================================

// Базовый адрес устройства по умолчанию
#define DEFAULT_DEVICE_ADDRESS     1     // Адрес устройства (1-247)

// Параметры Modbus (USART1)
#define MODBUS_BAUDRATE            9600  // Скорость Modbus
#define MODBUS_PARITY              UART_PARITY_NONE
#define MODBUS_STOP_BITS           UART_STOPBITS_1

// Параметры отладки (USART2)
#define DEBUG_BAUDRATE             115200
#define DEBUG_PARITY               UART_PARITY_NONE
#define DEBUG_STOP_BITS            UART_STOPBITS_1

// Адреса в энергонезависимой памяти (Flash)
#define CONFIG_FLASH_ADDRESS       0x0801FC00  // Адрес для хранения конфигурации

// Структура конфигурации
#pragma pack(push, 1)
typedef struct {
    uint32_t magic;           // Магическое число (0x55AA55AA)
    uint8_t  device_address;  // Текущий адрес устройства
    uint8_t  reserved[3];     // Зарезервировано для будущего использования
    uint32_t crc32;           // Контрольная сумма
} DeviceConfig_t;
#pragma pack(pop)

// ============================================
// КАРТА РЕГИСТРОВ MODBUS
// ============================================
typedef enum {
    REG_STATUS = 0,           // Статус устройства
    REG_DEVICE_ADDRESS = 1,   // Адрес устройства
    REG_PAUSE1_LOW = 2,       // Пауза 1-2, младшее слово
    REG_PAUSE1_HIGH = 3,      // Пауза 1-2, старшее слово
    REG_PAUSE2_LOW = 4,       // Пауза 2-3, младшее слово
    REG_PAUSE2_HIGH = 5,      // Пауза 2-3, старшее слово
    REG_FREQ_LOW = 6,         // Частота, младшее слово
    REG_FREQ_HIGH = 7,        // Частота, старшее слово
    REG_VDDA = 8,             // Напряжение VDDA (мВ)
    REG_V24_LOW = 9,          // Напряжение 24V, младшее слово
    REG_V24_HIGH = 10,        // Напряжение 24V, старшее слово
    REG_V12_LOW = 11,         // Напряжение 12V, младшее слово
    REG_V12_HIGH = 12,        // Напряжение 12V, старшее слово
    REG_V5_LOW = 13,          // Напряжение 5V, младшее слово
    REG_V5_HIGH = 14,         // Напряжение 5V, старшее слово
    REG_ERROR_COUNT = 15,     // Счетчик ошибок
    
    REG_COUNT                 // Общее количество регистров
} ModbusRegisters;

// ============================================
// СТАТУС УСТРОЙСТВА
// ============================================
typedef enum {
    STATUS_READY      = 0x0001,  // Устройство готово
    STATUS_FREQ_VALID = 0x0002,  // Измерение частоты выполнено
    STATUS_VOLT_VALID = 0x0004,  // Измерение напряжений выполнено
    STATUS_MODBUS_ERR = 0x0008,  // Ошибка Modbus
    STATUS_MEAS_ERR   = 0x0010,  // Ошибка измерения
    STATUS_ADC_ERR    = 0x0020,  // Ошибка ADC
    STATUS_CONFIG_ERR = 0x0040   // Ошибка конфигурации
} DeviceStatus;

// Системные константы
#define TIMER_CLOCK_HZ          72000000.0f
#define ADC_VREFINT_V           1.45f

// Функции для работы с конфигурацией
void Config_Init(void);
uint8_t Config_GetAddress(void);
void Config_SetAddress(uint8_t new_address);
void Config_SaveToFlash(void);
void Config_LoadFromFlash(void);
void Config_FactoryReset(void);

#endif /* CONFIG_H */