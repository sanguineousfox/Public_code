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

/* Вспомогательная функция для преобразования числа в строку в modbus.c */
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

    // Вывод ответа в формате QModMaster
    USART2_PrintModBusResponse(response, 5);

    HAL_UART_Transmit(&huart1, response, 5, 200);
}

/* Преобразование float в два регистра ------------------------------------*/
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

    // Вывод ответа в формате QModMaster
    USART2_PrintModBusResponse(response, index);

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

    // Вывод ответа в формате QModMaster
    USART2_PrintModBusResponse(response, index);

    HAL_UART_Transmit(&huart1, response, index, 200);
}

/* Запись одного регистра (функция 0x06) ----------------------------------*/
static void ModBus_WriteSingleRegister(uint16_t reg_addr, uint16_t value)
{
    if (reg_addr >= HOLD_HOLDING_REG_COUNT) {
        ModBus_SendException(0x06, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
        return;
    }

    // Записываем значение в регистр
    modbus.holding_regs[reg_addr] = value;

    // Если изменился адрес устройства, обновляем его
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
    
    // Отправляем ответ (эхо запроса)
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

    // Вывод ответа в формате QModMaster
    USART2_PrintModBusResponse(response, index);

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

    // Записываем данные в регистры
    for(uint16_t i = 0; i < reg_count; i++) {
        uint16_t value = (data[i*2] << 8) | data[i*2 + 1];
        modbus.holding_regs[start_addr + i] = value;

        // Если изменился адрес устройства (регистр 0), обновляем его
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

    // Отправляем ответ
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

    // Вывод ответа в формате QModMaster
    USART2_PrintModBusResponse(response, index);

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

    // Проверяем CRC
    uint16_t received_crc = (modbus.rx_buffer[modbus.rx_index - 1] << 8) |
                           modbus.rx_buffer[modbus.rx_index - 2];
    uint16_t calculated_crc = ModBus_CRC16(modbus.rx_buffer, modbus.rx_index - 2);

    if (received_crc != calculated_crc) {
        ModBus_DebugFrame(modbus.rx_buffer, modbus.rx_index, "RX CRC ERROR");
        modbus.rx_index = 0;
        modbus.rx_error = 1;
        return;
    }

    // Проверяем адрес устройства
    uint8_t device_addr = modbus.rx_buffer[0];
    if (device_addr != modbus.device_address && device_addr != 0) {
        modbus.rx_index = 0;
        return;
    }

    // Выводим принятую команду в формате как в QModMaster
    USART2_PrintModBusCommand(modbus.rx_buffer, modbus.rx_index);

    // Обработка в зависимости от функции
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

                // Проверяем соответствие количества байт
                if (byte_count == reg_count * 2 &&
                    modbus.rx_index == 7 + byte_count + 2) { // +2 для CRC
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
    // Инициализация ModBus структуры
    memset(&modbus, 0, sizeof(modbus));
    modbus.device_address = MODBUS_DEFAULT_ADDRESS;
    modbus.rx_active = 1;
    modbus.rx_complete = 0;
    modbus.rx_error = 0;
    modbus.rx_byte_count = 0;
    modbus.last_byte_time = HAL_GetTick();

    // Инициализация holding регистров нулями
    for (int i = 0; i < HOLD_HOLDING_REG_COUNT; i++) {
        modbus.holding_regs[i] = 0;
    }

    // Установка начальных значений регистров
    modbus.holding_regs[HOLD_DEVICE_ADDR] = MODBUS_DEFAULT_ADDRESS;
    modbus.holding_regs[HOLD_BAUDRATE] = MODBUS_BAUDRATE;
    modbus.holding_regs[HOLD_PARITY] = 0;
    modbus.holding_regs[HOLD_STOP_BITS] = 1;

    // Настройка начальных значений input регистров
    FloatToRegisters(3.3f, &modbus.input_regs[REG_VDDA_HIGH], &modbus.input_regs[REG_VDDA_LOW]);
    FloatToRegisters(24.0f, &modbus.input_regs[REG_24V_HIGH], &modbus.input_regs[REG_24V_LOW]);
    FloatToRegisters(12.0f, &modbus.input_regs[REG_12V_HIGH], &modbus.input_regs[REG_12V_LOW]);
    FloatToRegisters(5.0f, &modbus.input_regs[REG_5V_HIGH], &modbus.input_regs[REG_5V_LOW]);

    // Установка значений по умолчанию для измерений
    modbus.input_regs[REG_STATUS] = 0;
    modbus.input_regs[REG_COUNTER] = 0;
    modbus.input_regs[REG_TIMESTAMP_HIGH] = 0;
    modbus.input_regs[REG_TIMESTAMP_LOW] = 0;

    // Запуск приема по прерыванию
    HAL_UART_Receive_IT(&huart1, &modbus.rx_byte, 1);

    USART2_Print("[MODBUS] ModBus initialized\r\n");
    USART2_Print("[MODBUS] Address: ");
    char num_str[10];
    modbus_uint32_to_str(MODBUS_DEFAULT_ADDRESS, num_str);
    USART2_Print(num_str);
    USART2_Print(", Baudrate: ");
    modbus_uint32_to_str(MODBUS_BAUDRATE, num_str);
    USART2_Print(num_str);
    USART2_Print(", Holding registers: ");
    modbus_uint32_to_str(HOLD_HOLDING_REG_COUNT, num_str);
    USART2_Print(num_str);
    USART2_Print("\r\n");
}

/* Callback приема данных UART --------------------------------------------*/
void ModBus_RxCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        // Вызываем обработку принятого байта
        ModBus_RxByte(modbus.rx_byte);

        // Запускаем прием следующего байта
        HAL_UART_Receive_IT(&huart1, &modbus.rx_byte, 1);
    }
}

/* Прием одного байта данных ---------------------------------------------*/
void ModBus_RxByte(uint8_t byte)
{
    uint32_t current_time = HAL_GetTick();

    // Проверяем межкадровый интервал (3.5 символа при 9600 бод ~ 4 мс)
    if (current_time - modbus.last_byte_time > MODBUS_INTER_FRAME_TIMEOUT_MS) {
        // Новый фрейм, сбрасываем буфер
        modbus.rx_index = 0;
    }
    modbus.last_byte_time = current_time;

    // Проверяем, что прием активен и есть место в буфере
    if (modbus.rx_active && modbus.rx_index < MODBUS_BUFFER_SIZE) {
        // Сохраняем принятый байт
        modbus.rx_buffer[modbus.rx_index] = byte;
        modbus.rx_index++;

        // Если индекс превысил размер буфера, сбрасываем
        if (modbus.rx_index >= MODBUS_BUFFER_SIZE) {
            modbus.rx_index = 0;
            modbus.rx_error = 1;
        }
    } else {
        // Переполнение буфера
        modbus.rx_error = 1;
        modbus.rx_index = 0;
    }
}

/* Обработка ModBus (вызывать в цикле) -----------------------------------*/
void ModBus_Process(void)
{
    static uint32_t last_process_time = 0;
    uint32_t current_time = HAL_GetTick();

    // Обработка не чаще чем раз в 10 мс
    if (current_time - last_process_time < 10) {
        return;
    }
    last_process_time = current_time;

    // Если принят хотя бы 1 байт и прошло время таймаута, обрабатываем фрейм
    if (modbus.rx_index > 0 && (current_time - modbus.last_byte_time > MODBUS_RESPONSE_TIMEOUT_MS)) {
        // Принудительная обработка фрейма
        ModBus_ProcessFrame();
    }
}

/* Обновление данных напряжений -------------------------------------------*/
void ModBus_UpdateVoltages(float vdda, float v24, float v12, float v5)
{
    FloatToRegisters(vdda, &modbus.input_regs[REG_VDDA_HIGH], &modbus.input_regs[REG_VDDA_LOW]);
    FloatToRegisters(v24, &modbus.input_regs[REG_24V_HIGH], &modbus.input_regs[REG_24V_LOW]);
    FloatToRegisters(v12, &modbus.input_regs[REG_12V_HIGH], &modbus.input_regs[REG_12V_LOW]);
    FloatToRegisters(v5, &modbus.input_regs[REG_5V_HIGH], &modbus.input_regs[REG_5V_LOW]);
}

/* Обновление данных измерений --------------------------------------------*/
void ModBus_UpdateMeasurements(float p1, float p2, float freq, uint8_t status)
{
    static uint16_t counter = 0;

    FloatToRegisters(p1, &modbus.input_regs[REG_PERIOD1_HIGH], &modbus.input_regs[REG_PERIOD1_LOW]);
    FloatToRegisters(p2, &modbus.input_regs[REG_PERIOD2_HIGH], &modbus.input_regs[REG_PERIOD2_LOW]);
    FloatToRegisters(freq, &modbus.input_regs[REG_FREQ_HIGH], &modbus.input_regs[REG_FREQ_LOW]);

    modbus.input_regs[REG_STATUS] = status;
    modbus.input_regs[REG_COUNTER] = counter++;
    if (counter > 65535) counter = 0;

    uint32_t timestamp = HAL_GetTick();
    modbus.input_regs[REG_TIMESTAMP_HIGH] = (timestamp >> 16) & 0xFFFF;
    modbus.input_regs[REG_TIMESTAMP_LOW] = timestamp & 0xFFFF;
}
