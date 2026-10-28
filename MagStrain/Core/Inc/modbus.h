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

/* Адреса Input регистров (только чтение) - ИЗМЕНЕНО НА 16-БИТНЫЕ ЗНАЧЕНИЯ С ОБРАБОТКОЙ ОШИБОК */
#define REG_24V              0  // 24V * 10 (0.1V точность) или 0xFFFF при ошибке
#define REG_12V              1  // 12V * 10
#define REG_5V               2  // 5V * 10
#define REG_VDDA             3  // VDDA * 10
#define REG_TOF              4  // Время пролёта * 10 (0.1 мкс) или 0xFFFF при ошибке
#define REG_TEMPERATURE      5  // Температура * 10 (0.1 °C) или 0xFFFF при ошибке
#define REG_STATUS           6  // Статус измерения (бит 0: захват сигнала)
#define REG_COUNTER          7  // Счётчик измерений
#define REG_TIMESTAMP_HIGH   8  // Временная метка (старшие 16 бит)
#define REG_TIMESTAMP_LOW    9  // Временная метка (младшие 16 бит)
#define REG_INPUT_REG_COUNT  10

/* Адреса Holding регистров (чтение/запись) ---------------------------------*/
#define HOLD_DEVICE_ADDR     0
#define HOLD_BAUDRATE        1
#define HOLD_PARITY          2
#define HOLD_STOP_BITS       3
#define HOLD_HOLDING_REG_COUNT 32

/* Прототипы функций --------------------------------------------------------*/
void ModBus_Init(void);
void ModBus_Process(void);
/* ИЗМЕНЕНО: Добавлены флаги ошибок для обработки вывода FF */
void ModBus_UpdateVoltages(float vdda, float v24, float v12, float v5,
                           uint8_t vdda_err, uint8_t v24_err, uint8_t v12_err, uint8_t v5_err);
void ModBus_UpdateMeasurements(float tof_us, float temperature,
                               uint8_t tof_err, uint8_t temp_err, uint8_t status);
void ModBus_RxCallback(UART_HandleTypeDef *huart);
void ModBus_RxByte(uint8_t byte);
uint16_t ModBus_CRC16(const uint8_t *data, uint16_t length);
void ModBus_TransmitFrame(uint8_t *frame, uint16_t len);
void ModBus_PrepareForTransmit(void);  // Добавлен прототип
#endif /* MODBUS_H */
