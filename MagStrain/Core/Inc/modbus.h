#ifndef MODBUS_H
#define MODBUS_H

#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "utils.h"

/* Конфигурация ModBus ------------------------------------------------------*/
#define MODBUS_DEFAULT_ADDRESS   1
#define MODBUS_BAUDRATE          9600
#define MODBUS_BUFFER_SIZE       256
#define MODBUS_CRC_INIT          0xFFFF

/* Увеличенные таймауты для стабильной работы */
#define MODBUS_RESPONSE_TIMEOUT_MS  200
#define MODBUS_INTER_FRAME_TIMEOUT_MS 10
#define MODBUS_RETRY_COUNT         3

/* Коды функций ModBus ------------------------------------------------------*/
#define MODBUS_READ_HOLDING_REGISTERS    0x03
#define MODBUS_READ_INPUT_REGISTERS      0x04
#define MODBUS_WRITE_SINGLE_REGISTER     0x06
#define MODBUS_WRITE_MULTIPLE_REGISTERS  0x10

/* Коды исключений ----------------------------------------------------------*/
#define MODBUS_EXCEPTION_ILLEGAL_FUNCTION     0x01
#define MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS 0x02
#define MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE   0x03

/* Адреса Input регистров (только чтение) -----------------------------------*/
#define REG_VDDA_HIGH        0
#define REG_VDDA_LOW         1
#define REG_24V_HIGH         2
#define REG_24V_LOW          3
#define REG_12V_HIGH         4
#define REG_12V_LOW          5
#define REG_5V_HIGH          6
#define REG_5V_LOW           7
#define REG_PERIOD1_HIGH     8
#define REG_PERIOD1_LOW      9
#define REG_PERIOD2_HIGH     10
#define REG_PERIOD2_LOW      11
#define REG_FREQ_HIGH        12
#define REG_FREQ_LOW         13
#define REG_STATUS           14
#define REG_COUNTER          15
#define REG_TIMESTAMP_HIGH   16
#define REG_TIMESTAMP_LOW    17
#define REG_INPUT_REG_COUNT  18

/* Адреса Holding регистров (чтение/запись) ---------------------------------*/
#define HOLD_DEVICE_ADDR     0
#define HOLD_BAUDRATE        1
#define HOLD_PARITY          2
#define HOLD_STOP_BITS       3
#define HOLD_HOLDING_REG_COUNT 32

/* Прототипы функций --------------------------------------------------------*/
void ModBus_Init(void);
void ModBus_Process(void);
void ModBus_UpdateVoltages(float vdda, float v24, float v12, float v5);
void ModBus_UpdateMeasurements(float p1, float p2, float freq, uint8_t status);
void ModBus_RxCallback(UART_HandleTypeDef *huart);
void ModBus_RxByte(uint8_t byte);
uint16_t ModBus_CRC16(const uint8_t *data, uint16_t length);

#endif /* MODBUS_H */
