/**
@file    config.h
@brief   Конфигурация устройства
*/
#ifndef CONFIG_H
#define CONFIG_H
#include "stm32f1xx.h"

#define DEFAULT_DEVICE_ADDRESS     1
#define MODBUS_BAUDRATE            9600
#define MODBUS_PARITY              UART_PARITY_NONE
#define MODBUS_STOP_BITS           UART_STOPBITS_1
#define DEBUG_BAUDRATE             115200
#define DEBUG_PARITY               UART_PARITY_NONE
#define DEBUG_STOP_BITS            UART_STOPBITS_1
#define CONFIG_FLASH_ADDRESS       0x0801FC00

#pragma pack(push, 1)
typedef struct {
    uint32_t magic;
    uint8_t  device_address;
    uint8_t  reserved[3];
    uint32_t crc32;
} DeviceConfig_t;
#pragma pack(pop)

typedef enum {
    REG_STATUS = 0,
    REG_DEVICE_ADDRESS = 1,
    REG_PAUSE1_LOW = 2,
    REG_PAUSE1_HIGH = 3,
    REG_PAUSE2_LOW = 4,
    REG_PAUSE2_HIGH = 5,
    REG_FREQ_LOW = 6,
    REG_FREQ_HIGH = 7,
    REG_VDDA = 8,
    REG_V24_LOW = 9,
    REG_V24_HIGH = 10,
    REG_V12_LOW = 11,
    REG_V12_HIGH = 12,
    REG_V5_LOW = 13,
    REG_V5_HIGH = 14,
    REG_ERROR_COUNT = 15,
    REG_COUNT
} ModbusRegisters;

typedef enum {
    STATUS_READY      = 0x0001,
    STATUS_FREQ_VALID = 0x0002,
    STATUS_VOLT_VALID = 0x0004,
    STATUS_MODBUS_ERR = 0x0008,
    STATUS_MEAS_ERR   = 0x0010,
    STATUS_ADC_ERR    = 0x0020,
    STATUS_CONFIG_ERR = 0x0040
} DeviceStatus;

#define TIMER_CLOCK_HZ          72000000.0f
#define ADC_VREFINT_V           1.45f

void Config_Init(void);
uint8_t Config_GetAddress(void);
void Config_SetAddress(uint8_t new_address);
void Config_SaveToFlash(void);
void Config_LoadFromFlash(void);
void Config_FactoryReset(void);

#endif
