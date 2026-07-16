#ifndef MODBUS_H
#define MODBUS_H

#include "stm32f1xx_hal.h"
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
   КОНФИГУРАЦИЯ MODBUS
============================================================================ */
#define MODBUS_DEFAULT_ADDRESS      1       // Адрес устройства по умолчанию
#define MODBUS_BAUDRATE             19200   // Скорость обмена
#define MODBUS_BUFFER_SIZE          256     // Размер буфера приёма
#define MODBUS_RESPONSE_TIMEOUT_MS  3       // Таймаут ответа (мс)
#define MODBUS_INTER_FRAME_TIMEOUT_MS 2     // Таймаут между кадрами (мс)
#define HOLD_HOLDING_REG_COUNT      32      // Количество Holding Registers
#define REG_INPUT_REG_COUNT         16      // Количество Input Registers

/* ============================================================================
   АДРЕСА HOLDING REGISTERS (для чтения/записи)
============================================================================ */
#define HOLD_DEVICE_ADDR            0       // Адрес устройства
#define HOLD_BAUDRATE               1       // Скорость обмена
#define HOLD_PARITY                 2       // Чётность
#define HOLD_STOP_BITS              3       // Стоповые биты
#define MB_ADDR_POLL_PERIOD         5       // Период опроса (сек)
#define MB_ADDR_CAL_LOW_LVL         7       // Нижний уровень калибровки
#define MB_ADDR_CAL_HIGH_LVL        8       // Верхний уровень калибровки

/* ============================================================================
   АДРЕСА INPUT REGISTERS (только для чтения)
============================================================================ */
#define REG_24V                     0       // Напряжение 24V
#define REG_12V                     1       // Напряжение 12V
#define REG_5V                      2       // Напряжение 5V
#define REG_VDDA                    3       // Напряжение VDDA
#define REG_TOF                     4       // Время пролёта (ToF)
#define REG_TEMPERATURE             5       // Температура
#define REG_STATUS                  6       // Статус устройства
#define REG_COUNTER                 7       // Счётчик измерений
#define REG_TIMESTAMP_HIGH          8       // Временная метка (старшее слово)
#define REG_TIMESTAMP_LOW           9       // Временная метка (младшее слово)

/* ============================================================================
   КОДЫ ФУНКЦИЙ MODBUS
============================================================================ */
#define MODBUS_READ_HOLDING_REGISTERS       0x03U
#define MODBUS_READ_INPUT_REGISTERS         0x04U
#define MODBUS_WRITE_SINGLE_REGISTER        0x06U
#define MODBUS_WRITE_MULTIPLE_REGISTERS     0x10U

/* ============================================================================
   КОДЫ ИСКЛЮЧЕНИЙ (Exception Codes)
============================================================================ */
#define MODBUS_EXCEPTION_ILLEGAL_FUNCTION           0x01U
#define MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS       0x02U
#define MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE         0x03U
#define MODBUS_EXCEPTION_SLAVE_DEVICE_FAILURE       0x04U

/* ============================================================================
   МАКРОСЫ УПРАВЛЕНИЯ RS-485
============================================================================ */
#define RS485_CTRL_PIN          GPIO_PIN_0
#define RS485_CTRL_PORT         GPIOB

#define RS485_SET_TRANSMIT()    HAL_GPIO_WritePin(RS485_CTRL_PORT, RS485_CTRL_PIN, GPIO_PIN_SET)
#define RS485_SET_RECEIVE()     HAL_GPIO_WritePin(RS485_CTRL_PORT, RS485_CTRL_PIN, GPIO_PIN_RESET)

/* ============================================================================
   ПРОТОТИПЫ ФУНКЦИЙ
============================================================================ */
void ModBus_Init(void);
void ModBus_Process(void);
void ModBus_RxCallback(UART_HandleTypeDef *huart);
void ModBus_RxByte(uint8_t byte);
uint16_t ModBus_CRC16(const uint8_t *data, uint16_t length);
uint8_t ModBus_GetDeviceAddress(void);

// Обновление данных
void ModBus_UpdateVoltages(float vdda, float v24, float v12, float v5,
                           uint8_t vdda_err, uint8_t v24_err, uint8_t v12_err, uint8_t v5_err);
void ModBus_UpdateMeasurements(float tof_us, float temperature,
                               uint8_t tof_err, uint8_t temp_err, uint8_t status);
void ModBus_UpdateFirmwareVersion(uint16_t version);

// Работа с параметрами (float)
float ModBus_GetParameter_Float(uint8_t addr);
void ModBus_SetParameter_Float(uint8_t addr, float value);
float ModBus_GetWaveguideLength(void);

// Подготовка к передаче
void ModBus_PrepareForTransmit(void);

#endif /* MODBUS_H */
