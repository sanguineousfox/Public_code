/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * @description    : Система измерения времени пролёта магнитострикционного датчика
  *                   - Импульс: точно 10.00 мкс (54 итерации)
  *                   - Период измерений: загружается из конфигурации (по умолчанию 10 сек)
  *                   - Красный светодиод (PB13) горит 1 сек при захвате сигнала
  *                   - Добавлена поддержка I2C2: LM75B (температура) и AT24C02 (EEPROM)
  *                   - ИСПРАВЛЕНО: Управление линией RS485 через PB0 с надёжным возвратом в приём
  *                   - ДОБАВЛЕНО: Вывод параметров через Modbus с обработкой ошибок (FF при ошибках)
  *                   - ДОБАВЛЕНО: Загрузка конфигурации из EEPROM при старте
  *                   - ДОБАВЛЕНО: Сохранение конфигурации в EEPROM через Modbus (регистр 6 = 0xCAFE)
  *
  * РЕГИСТРЫ КОНФИГУРАЦИИ (Holding Registers, функция 0x03 / 0x06 / 0x10):
  * ============================================================================
  * Адрес | Назначение          | Диапазон / Значение     | Примечание
  * ------|---------------------|-------------------------|-------------------
  *   0   | Адрес устройства    | 1-247                   | Автоматически применяется
  *   1   | Калибровка 24В      | 920 = 9.20              | Коэффициент × 100
  *   2   | Калибровка 12В      | 460 = 4.60              | Коэффициент × 100
  *   3   | Калибровка 5В       | 200 = 2.00              | Коэффициент × 100
  *   4   | Таймаут TOF         | 10 = 10 мс              | Диапазон 1-1000 мс
  *   5   | Период измерений    | 10000 = 10 сек          | Диапазон 1000-60000 мс
  *   6   | Сохранить в EEPROM  | 51966 (0xCAFE)          | Запись этого значения сохраняет ВСЮ конфигурацию
  * 7-8   | Счётчик загрузок    | Только чтение           | 32-битное значение (регистры 7=старшие, 8=младшие)
  * ============================================================================
  *
  * РЕГИСТРЫ ИЗМЕРЕНИЙ (Input Registers, функция 0x04):
  * ============================================================================
  * Адрес | Назначение          | Формат                  | Примечание
  * ------|---------------------|-------------------------|-------------------
  *   0   | Напряжение 24В      | Значение × 10 (0.1 В)   | 0xFFFF = ошибка
  *   1   | Напряжение 12В      | Значение × 10 (0.1 В)   | 0xFFFF = ошибка
  *   2   | Напряжение 5В       | Значение × 10 (0.1 В)   | 0xFFFF = ошибка
  *   3   | Напряжение VDDA     | Значение × 10 (0.1 В)   | 0xFFFF = ошибка
  *   4   | Время пролёта       | Значение × 10 (0.1 мкс) | 0xFFFF = ошибка/таймаут
  *   5   | Температура         | Значение × 10 (0.1 °C)  | 0xFFFF = ошибка датчика
  *   6   | Статус измерения    | Бит 0: захват сигнала   | 1 = сигнал захвачен
  *   7   | Счётчик измерений   | 16-битное значение      | Инкрементируется каждый цикл
  * 8-9   | Временная метка     | 32-битное значение мс   | Регистры 8=старшие, 9=младшие
  * ============================================================================
  ******************************************************************************
  */
/* USER CODE END Header */



#include "modbus.h"
#include "main.h"
#include "utils.h"
#include <string.h>
#include <math.h>

/* Структура данных ModBus ------------------------------------------------*/
typedef struct {
    uint16_t holding_regs[HOLD_HOLDING_REG_COUNT];  // Holding регистры
    uint16_t input_regs[REG_INPUT_REG_COUNT];       // Input регистры

    uint8_t device_address;                         // Адрес устройства
    uint32_t rx_timeout;                            // Таймаут приема
    uint16_t rx_index;                              // Индекс приема
    uint8_t rx_buffer[MODBUS_BUFFER_SIZE];          // Буфер приема
    uint8_t rx_byte;                                // Текущий принятый байт
    uint8_t rx_active;                              // Флаг активности приема
    uint8_t rx_complete;                            // Флаг завершения приема фрейма
    uint8_t rx_error;                               // Флаг ошибки приема
    uint32_t rx_byte_count;                         // Счетчик принятых байт
    uint32_t last_byte_time;                        // Время приема последнего байта
    uint8_t tx_buffer[256];                         // Буфер передачи
    uint16_t tx_index;                              // Индекс передачи
} ModBus_Struct;

static ModBus_Struct modbus;

/* Внешние переменные -----------------------------------------------------*/
extern UART_HandleTypeDef huart1;
extern volatile uint8_t modbus_tx_active;  // Флаг состояния передачи из main.c

/* Вспомогательная функция для преобразования числа в строку */
static void modbus_uint32_to_str(uint32_t value, char* buffer)
{
    if (value == 0) {
        buffer[0] = '0';
        buffer[1] = '\0';
        return;
    }

    char temp[16];
    int i = 0;

    while (value > 0 && i < 15) {
        temp[i++] = (value % 10) + '0';
        value /= 10;
    }

    for (int j = 0; j < i; j++) {
        buffer[j] = temp[i - j - 1];
    }
    buffer[i] = '\0';
}

/* Функции для работы с CRC16 ---------------------------------------------*/
uint16_t ModBus_CRC16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = MODBUS_CRC_INIT;
    uint8_t bit;

    while(length--) {
        crc ^= *data++;

        for(bit = 0; bit < 8; bit++) {
            if(crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}

/* Отправка исключения ----------------------------------------------------*/
static void ModBus_SendException(uint8_t function, uint8_t exception_code)
{
    uint8_t response[5];
    uint16_t crc;

    response[0] = modbus.device_address;
    response[1] = function | 0x80;
    response[2] = exception_code;

    crc = ModBus_CRC16(response, 3);
    response[3] = crc & 0xFF;
    response[4] = (crc >> 8) & 0xFF;

    USART2_PrintModBusResponse(response, 5);

    if (!modbus_tx_active) {
        ModBus_PrepareForTransmit();
    }

    HAL_UART_Transmit(&huart1, response, 5, 200);
}

/* Чтение holding регистров (функция 0x03) --------------------------------*/
static void ModBus_ReadHoldingRegisters(uint16_t start_addr, uint16_t reg_count)
{
    if (start_addr >= HOLD_HOLDING_REG_COUNT) {
        ModBus_SendException(0x03, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
        return;
    }

    if (reg_count == 0 || reg_count > 125) {
        ModBus_SendException(0x03, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
        return;
    }

    if (start_addr + reg_count > HOLD_HOLDING_REG_COUNT) {
        ModBus_SendException(0x03, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
        return;
    }

    uint8_t response[256];
    uint16_t index = 0;
    uint16_t bytes_to_send = reg_count * 2;

    response[index++] = modbus.device_address;
    response[index++] = MODBUS_READ_HOLDING_REGISTERS;
    response[index++] = bytes_to_send;

    for(uint16_t i = 0; i < reg_count; i++) {
        uint16_t reg_value = modbus.holding_regs[start_addr + i];
        response[index++] = (reg_value >> 8) & 0xFF;
        response[index++] = reg_value & 0xFF;
    }

    uint16_t crc = ModBus_CRC16(response, index);
    response[index++] = crc & 0xFF;
    response[index++] = (crc >> 8) & 0xFF;

    USART2_PrintModBusResponse(response, index);

    if (!modbus_tx_active) {
        ModBus_PrepareForTransmit();
    }

    HAL_UART_Transmit(&huart1, response, index, 200);
}

/* Чтение input регистров (функция 0x04) ---------------------------------*/
static void ModBus_ReadInputRegisters(uint16_t start_addr, uint16_t reg_count)
{
    if (start_addr >= REG_INPUT_REG_COUNT) {
        ModBus_SendException(0x04, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
        return;
    }

    if (reg_count == 0 || reg_count > 125) {
        ModBus_SendException(0x04, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
        return;
    }

    if (start_addr + reg_count > REG_INPUT_REG_COUNT) {
        ModBus_SendException(0x04, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
        return;
    }

    uint8_t response[256];
    uint16_t index = 0;
    uint16_t bytes_to_send = reg_count * 2;

    response[index++] = modbus.device_address;
    response[index++] = MODBUS_READ_INPUT_REGISTERS;
    response[index++] = bytes_to_send;

    for(uint16_t i = 0; i < reg_count; i++) {
        uint16_t reg_value = modbus.input_regs[start_addr + i];
        response[index++] = (reg_value >> 8) & 0xFF;
        response[index++] = reg_value & 0xFF;
    }

    uint16_t crc = ModBus_CRC16(response, index);
    response[index++] = crc & 0xFF;
    response[index++] = (crc >> 8) & 0xFF;

    USART2_PrintModBusResponse(response, index);

    if (!modbus_tx_active) {
        ModBus_PrepareForTransmit();
    }

    HAL_UART_Transmit(&huart1, response, index, 200);
}

/* Запись одного регистра (функция 0x06) ----------------------------------*/
static void ModBus_WriteSingleRegister(uint16_t reg_addr, uint16_t value)
{
    if (reg_addr >= HOLD_HOLDING_REG_COUNT) {
        ModBus_SendException(0x06, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
        return;
    }

    modbus.holding_regs[reg_addr] = value;

    if (reg_addr == HOLD_DEVICE_ADDR) {
        uint8_t new_addr = (uint8_t)(value & 0xFF);
        if (new_addr >= 1 && new_addr <= 247) {
            modbus.device_address = new_addr;
            char num_str[10];
            USART2_Print("[MODBUS] Device address changed to ");
            modbus_uint32_to_str(new_addr, num_str);
            USART2_Print(num_str);
            USART2_Print("\r\n");
        }
    }
    
    uint8_t response[8];
    uint16_t index = 0;

    response[index++] = modbus.device_address;
    response[index++] = MODBUS_WRITE_SINGLE_REGISTER;
    response[index++] = (reg_addr >> 8) & 0xFF;
    response[index++] = reg_addr & 0xFF;
    response[index++] = (value >> 8) & 0xFF;
    response[index++] = value & 0xFF;

    uint16_t crc = ModBus_CRC16(response, index);
    response[index++] = crc & 0xFF;
    response[index++] = (crc >> 8) & 0xFF;

    USART2_PrintModBusResponse(response, index);

    if (!modbus_tx_active) {
        ModBus_PrepareForTransmit();
    }

    HAL_UART_Transmit(&huart1, response, index, 200);
}

/* Запись нескольких регистров (функция 0x10) -----------------------------*/
static void ModBus_WriteMultipleRegisters(uint16_t start_addr, uint16_t reg_count, uint8_t *data)
{
    if (start_addr >= HOLD_HOLDING_REG_COUNT) {
        ModBus_SendException(0x10, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
        return;
    }

    if (reg_count == 0 || reg_count > 123) {
        ModBus_SendException(0x10, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
        return;
    }

    if (start_addr + reg_count > HOLD_HOLDING_REG_COUNT) {
        ModBus_SendException(0x10, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
        return;
    }

    for(uint16_t i = 0; i < reg_count; i++) {
        uint16_t value = (data[i*2] << 8) | data[i*2 + 1];
        modbus.holding_regs[start_addr + i] = value;

        if ((start_addr + i) == HOLD_DEVICE_ADDR) {
            uint8_t new_addr = (uint8_t)(value & 0xFF);
            if (new_addr >= 1 && new_addr <= 247) {
                modbus.device_address = new_addr;
                char num_str[10];
                USART2_Print("[MODBUS] Device address changed to ");
                modbus_uint32_to_str(new_addr, num_str);
                USART2_Print(num_str);
                USART2_Print("\r\n");
            }
        }
    }

    uint8_t response[8];
    uint16_t index = 0;

    response[index++] = modbus.device_address;
    response[index++] = MODBUS_WRITE_MULTIPLE_REGISTERS;
    response[index++] = (start_addr >> 8) & 0xFF;
    response[index++] = start_addr & 0xFF;
    response[index++] = (reg_count >> 8) & 0xFF;
    response[index++] = reg_count & 0xFF;

    uint16_t crc = ModBus_CRC16(response, index);
    response[index++] = crc & 0xFF;
    response[index++] = (crc >> 8) & 0xFF;

    USART2_PrintModBusResponse(response, index);

    if (!modbus_tx_active) {
        ModBus_PrepareForTransmit();
    }

    HAL_UART_Transmit(&huart1, response, index, 200);
}

/* Обработка принятого фрейма ---------------------------------------------*/
static void ModBus_ProcessFrame(void)
{
    if (modbus.rx_index < 4) {
        USART2_Print("[MODBUS] Frame too short, ignoring\r\n");
        modbus.rx_index = 0;
        return;
    }

    uint16_t received_crc = (modbus.rx_buffer[modbus.rx_index - 1] << 8) |
                           modbus.rx_buffer[modbus.rx_index - 2];
    uint16_t calculated_crc = ModBus_CRC16(modbus.rx_buffer, modbus.rx_index - 2);

    if (received_crc != calculated_crc) {
        ModBus_DebugFrame(modbus.rx_buffer, modbus.rx_index, "RX CRC ERROR");
        modbus.rx_index = 0;
        modbus.rx_error = 1;
        return;
    }

    uint8_t device_addr = modbus.rx_buffer[0];
    if (device_addr != modbus.device_address && device_addr != 0) {
        modbus.rx_index = 0;
        return;
    }

    USART2_PrintModBusCommand(modbus.rx_buffer, modbus.rx_index);

    switch(modbus.rx_buffer[1]) {
        case MODBUS_READ_HOLDING_REGISTERS:
            if (modbus.rx_index >= 8) {
                uint16_t start_addr = (modbus.rx_buffer[2] << 8) | modbus.rx_buffer[3];
                uint16_t reg_count = (modbus.rx_buffer[4] << 8) | modbus.rx_buffer[5];
                ModBus_ReadHoldingRegisters(start_addr, reg_count);
            }
            break;

        case MODBUS_READ_INPUT_REGISTERS:
            if (modbus.rx_index >= 8) {
                uint16_t start_addr = (modbus.rx_buffer[2] << 8) | modbus.rx_buffer[3];
                uint16_t reg_count = (modbus.rx_buffer[4] << 8) | modbus.rx_buffer[5];
                ModBus_ReadInputRegisters(start_addr, reg_count);
            }
            break;

        case MODBUS_WRITE_SINGLE_REGISTER:
            if (modbus.rx_index >= 8) {
                uint16_t reg_addr = (modbus.rx_buffer[2] << 8) | modbus.rx_buffer[3];
                uint16_t reg_value = (modbus.rx_buffer[4] << 8) | modbus.rx_buffer[5];
                ModBus_WriteSingleRegister(reg_addr, reg_value);
            }
            break;

        case MODBUS_WRITE_MULTIPLE_REGISTERS:
            if (modbus.rx_index >= 9) {
                uint16_t start_addr = (modbus.rx_buffer[2] << 8) | modbus.rx_buffer[3];
                uint16_t reg_count = (modbus.rx_buffer[4] << 8) | modbus.rx_buffer[5];
                uint8_t byte_count = modbus.rx_buffer[6];

                if (byte_count == reg_count * 2 &&
                    modbus.rx_index == 7 + byte_count + 2) {
                    ModBus_WriteMultipleRegisters(start_addr, reg_count,
                                                 &modbus.rx_buffer[7]);
                } else {
                    ModBus_SendException(0x10, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
                }
            }
            break;

        default:
            ModBus_SendException(modbus.rx_buffer[1], MODBUS_EXCEPTION_ILLEGAL_FUNCTION);
            break;
    }

    modbus.rx_index = 0;
    modbus.rx_complete = 0;
    modbus.rx_error = 0;
}

/* Инициализация ModBus ---------------------------------------------------*/
void ModBus_Init(void)
{
    memset(&modbus, 0, sizeof(modbus));
    modbus.device_address = MODBUS_DEFAULT_ADDRESS;
    modbus.rx_active = 1;
    modbus.rx_complete = 0;
    modbus.rx_error = 0;
    modbus.rx_byte_count = 0;
    modbus.last_byte_time = HAL_GetTick();

    for (int i = 0; i < HOLD_HOLDING_REG_COUNT; i++) {
        modbus.holding_regs[i] = 0;
    }

    modbus.holding_regs[HOLD_DEVICE_ADDR] = MODBUS_DEFAULT_ADDRESS;
    modbus.holding_regs[HOLD_BAUDRATE] = MODBUS_BAUDRATE;
    modbus.holding_regs[HOLD_PARITY] = 0;
    modbus.holding_regs[HOLD_STOP_BITS] = 1;

    // Инициализация регистров ошибками по умолчанию (FFFF)
    for (int i = 0; i < REG_INPUT_REG_COUNT; i++) {
        modbus.input_regs[i] = 0xFFFF;
    }

    HAL_UART_Receive_IT(&huart1, &modbus.rx_byte, 1);

    USART2_Print("[MODBUS] ModBus initialized\r\n");
    USART2_Print("[MODBUS] Address: ");
    char num_str[10];
    modbus_uint32_to_str(MODBUS_DEFAULT_ADDRESS, num_str);
    USART2_Print(num_str);
    USART2_Print(", Baudrate: ");
    modbus_uint32_to_str(MODBUS_BAUDRATE, num_str);
    USART2_Print(num_str);
    USART2_Print(", Input registers: ");
    modbus_uint32_to_str(REG_INPUT_REG_COUNT, num_str);
    USART2_Print(num_str);
    USART2_Print("\r\n");
}

/* Callback приема данных UART --------------------------------------------*/
void ModBus_RxCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        ModBus_RxByte(modbus.rx_byte);
        HAL_UART_Receive_IT(&huart1, &modbus.rx_byte, 1);
    }
}

/* Прием одного байта данных ---------------------------------------------*/
void ModBus_RxByte(uint8_t byte)
{
    uint32_t current_time = HAL_GetTick();

    if (current_time - modbus.last_byte_time > MODBUS_INTER_FRAME_TIMEOUT_MS) {
        modbus.rx_index = 0;
    }
    modbus.last_byte_time = current_time;

    if (modbus.rx_active && modbus.rx_index < MODBUS_BUFFER_SIZE) {
        modbus.rx_buffer[modbus.rx_index] = byte;
        modbus.rx_index++;

        if (modbus.rx_index >= MODBUS_BUFFER_SIZE) {
            modbus.rx_index = 0;
            modbus.rx_error = 1;
        }
    } else {
        modbus.rx_error = 1;
        modbus.rx_index = 0;
    }
}

/* Обработка ModBus (вызывать в цикле) -----------------------------------*/
void ModBus_Process(void)
{
    static uint32_t last_process_time = 0;
    uint32_t current_time = HAL_GetTick();

    if (current_time - last_process_time < 10) {
        return;
    }
    last_process_time = current_time;

    if (modbus.rx_index > 0 && (current_time - modbus.last_byte_time > MODBUS_RESPONSE_TIMEOUT_MS)) {
        ModBus_ProcessFrame();
    }
}

/* Обновление данных напряжений с обработкой ошибок ------------------------*/
void ModBus_UpdateVoltages(float vdda, float v24, float v12, float v5,
                           uint8_t vdda_err, uint8_t v24_err, uint8_t v12_err, uint8_t v5_err)
{
    modbus.input_regs[REG_24V] = v24_err ? 0xFFFF : (uint16_t)(v24 * 10.0f + 0.5f);
    modbus.input_regs[REG_12V] = v12_err ? 0xFFFF : (uint16_t)(v12 * 10.0f + 0.5f);
    modbus.input_regs[REG_5V] = v5_err ? 0xFFFF : (uint16_t)(v5 * 10.0f + 0.5f);
    modbus.input_regs[REG_VDDA] = vdda_err ? 0xFFFF : (uint16_t)(vdda * 10.0f + 0.5f);
}

/* Обновление данных измерений с обработкой ошибок -------------------------*/
void ModBus_UpdateMeasurements(float tof_us, float temperature,
                               uint8_t tof_err, uint8_t temp_err, uint8_t status)
{
    static uint16_t counter = 0;

    modbus.input_regs[REG_TOF] = tof_err ? 0xFFFF : (uint16_t)(tof_us * 10.0f + 0.5f);
    modbus.input_regs[REG_TEMPERATURE] = temp_err ? 0xFFFF : (uint16_t)(temperature * 10.0f + 0.5f);
    modbus.input_regs[REG_STATUS] = status;
    modbus.input_regs[REG_COUNTER] = counter++;
    if (counter > 65535) counter = 0;

    uint32_t timestamp = HAL_GetTick();
    modbus.input_regs[REG_TIMESTAMP_HIGH] = (timestamp >> 16) & 0xFFFF;
    modbus.input_regs[REG_TIMESTAMP_LOW] = timestamp & 0xFFFF;
}
