/**
 * @file           : main.c
 * @brief          : Основной цикл магнитострикционного уровнемера
 *                   - Буферизованный вывод в USART2
 *                   - Интеграция с AT24C64 EEPROM
 *                   - Калибровка по двум точкам (команды 01, 02, 03, 04)
 *                   - Интеграция с модулями temp_sensors и graduation
 */
#include "main.h"
#include "stm32f1xx_it.h"
#include "utils.h"
#include "i2c_config.h"
#include "modbus.h"
#include "at24c64.h"
#include "temp_sensors.h"
#include "graduation.h"
#include <stdint.h>
#include <string.h>
#include <math.h>

/* ==========================================================================
   КОНСТАНТЫ И МАКРОСЫ
   ========================================================================== */
#define TIMER_CLOCK_HZ          72000000.0f
#define TOF_TICK_US             0.0972f
#define VREFINT_CAL_ADDR        0x1FFFF7BA
#define VREFINT_CAL_VALUE       ((uint16_t *)VREFINT_CAL_ADDR)
#define ADC_SAMPLES             16
#define MEAS_TIMEOUT_MS         50
#define DIV_24V_FACTOR          9.4f
#define DIV_12V_FACTOR          4.0f
#define DIV_5V_FACTOR           2.0f
#define PULSE_PERIOD_MS_DEFAULT 1000
#define SOUND_SPEED_MPS         3370.0f
#define STAT_HISTORY_SIZE       11
#define LED_RED_PIN             GPIO_PIN_13
#define LED_BLUE_PIN            GPIO_PIN_12
#define LED_RED_ON_TIME_MS      1000
#define LED_RED_ON              GPIO_PIN_SET
#define LED_RED_OFF             GPIO_PIN_RESET
#define LED_BLUE_ON             GPIO_PIN_SET
#define LED_BLUE_OFF            GPIO_PIN_RESET
#define PULSE_DELAY_ITERATIONS  10
#define DELAY_AFTER_PULSE_ITER  97
#define SWITCH_HOLD_ITERATIONS  12000
#define SWITCH_PIN              GPIO_PIN_7
#define SWITCH_PORT             GPIOB
#define MODBUS_SILENCE_TIME_MS  4
#define RS485_CTRL_PIN          GPIO_PIN_0
#define RS485_CTRL_PORT         GPIOB
#define FIRMWARE_VERSION        100

/* Ограничения периода опроса в МИЛЛИСЕКУНДАХ */
#define MIN_POLL_PERIOD_MS      100
#define MAX_POLL_PERIOD_MS      60000
#define MIN_CLICK_WIDTH_US      14.0f
#define MAX_CLICK_WIDTH_US      24.0f

/* ==========================================================================
   ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ
   ========================================================================== */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

volatile uint32_t tof_capture_value = 0;
volatile uint8_t tof_measurement_done = 0;
volatile uint8_t tof_timeout = 0;
volatile uint8_t signal_captured = 0;
volatile uint32_t captured_pulses[MAX_CAPTURED_PULSES];
volatile uint8_t capture_count = 0;
volatile uint8_t expected_pulse_pairs = MAX_PULSE_PAIRS;
volatile uint32_t last_capture_cnt = 0;
volatile uint8_t modbus_tx_active = 0;

static uint32_t measurement_history[STAT_HISTORY_SIZE] = {0};
static uint8_t stat_index = 0;
static uint8_t stat_count = 0;
static uint8_t stat_ready = 0;

static float current_vdda = 3.3f;
static float current_24v = 24.0f;
static float current_12v = 12.0f;
static float current_5v = 5.0f;
static float current_temperature = 0.0f;
static uint32_t current_poll_period_ms = PULSE_PERIOD_MS_DEFAULT;

static float prev_vdda = 0.0f;
static float prev_24v = 0.0f;
static float prev_12v = 0.0f;

static uint8_t v24_error = 0;
static uint8_t v12_error = 0;
static uint8_t v5_error = 0;
static uint8_t vdda_error = 0;

/* ==========================================================================
   МАКРОСЫ УПРАВЛЕНИЯ RS-485
   ========================================================================== */
#define RS485_SET_TRANSMIT() HAL_GPIO_WritePin(RS485_CTRL_PORT, RS485_CTRL_PIN, GPIO_PIN_SET)
#define RS485_SET_RECEIVE()  HAL_GPIO_WritePin(RS485_CTRL_PORT, RS485_CTRL_PIN, GPIO_PIN_RESET)

/* ==========================================================================
   БУФЕРНЫЙ ВЫВОД В USART2
   ========================================================================== */
#define USART2_TX_BUF_SIZE      512
static char tx_buf[USART2_TX_BUF_SIZE];
static uint16_t tx_buf_len = 0;

void USART2_BufInit(void) { tx_buf_len = 0; }

void USART2_BufFlush(void)
{
    if (tx_buf_len > 0) {
        HAL_UART_Transmit(&huart2, (uint8_t*)tx_buf, tx_buf_len, 200);
        tx_buf_len = 0;
    }
}

static void buf_putc(char c)
{
    if (tx_buf_len >= (USART2_TX_BUF_SIZE - 1)) {
        USART2_BufFlush();
    }
    tx_buf[tx_buf_len++] = c;
}

void USART2_BufPrint(const char *str)
{
    while (*str) { buf_putc(*str++); }
}

void USART2_BufPrintInt(int32_t val)
{
    char tmp[12];
    int8_t i = 0;
    if (val < 0) { buf_putc('-'); val = -val; }
    if (val == 0) { buf_putc('0'); return; }
    while (val > 0) { tmp[i++] = (val % 10) + '0'; val /= 10; }
    for (int8_t j = i - 1; j >= 0; j--) buf_putc(tmp[j]);
}

void USART2_BufPrintFloat(float val)
{
    int32_t int_part = (int32_t)val;
    float frac = val - (float)int_part;
    if (frac < 0) frac = -frac;
    int32_t frac_part = (int32_t)(frac * 100.0f + 0.5f);
    if (frac_part >= 100) frac_part = 0;
    USART2_BufPrintInt(int_part);
    buf_putc('.');
    if (frac_part < 10) buf_putc('0');
    USART2_BufPrintInt(frac_part);
}


/* ==========================================================================
   ПРОТОТИПЫ ЛОКАЛЬНЫХ ФУНКЦИЙ
   ========================================================================== */
static void USART2_PrintInt(int32_t val);
static void USART2_PrintFloat(float val);
static void Check_Voltage_Change(const char *name, float new_val, float old_val, float *store_val);
static void Update_Poll_Period_From_Modbus(void);
static void Stat_AddValue(uint32_t value);
static float Stat_CalculateTrimmedAverage(void);
static void Stat_ClearHistory(void);

/* ==========================================================================
   КАЛИБРОВКА: Расчет расстояния
   ========================================================================== */
static float Calculate_Position(float tof_us)
{
    float h_low   = ModBus_GetParameter_Float(MB_ADDR_CAL_LOW_LVL);
    float h_high  = ModBus_GetParameter_Float(MB_ADDR_CAL_HIGH_LVL);
    float C1_mm   = ModBus_GetParameter_Float(MB_ADDR_CAL_C1);
    float C2_mm   = ModBus_GetParameter_Float(MB_ADDR_CAL_C2);

    float measured_distance_mm = tof_us * 0.001f * SOUND_SPEED_MPS;

    if (C1_mm > 0.0f && C2_mm > 0.0f && C1_mm > C2_mm && h_high > h_low) {
        float position_m = h_low + (C1_mm - measured_distance_mm) * (h_high - h_low) / (C1_mm - C2_mm);
        if (position_m < 0.0f) position_m = 0.0f;
        if (position_m > h_high * 1.1f) position_m = h_high * 1.1f;
        return position_m * 1000.0f;
    } else {
        return measured_distance_mm;
    }
}

/* ==========================================================================
   КАЛИБРОВКА: Обработка команд управления
   ========================================================================== */
void Process_Calibration_Command(uint16_t cmd)
{
    /* === Обработка команд градуировочной таблицы === */
    if (cmd >= 300 && cmd <= 302) {
        float param = ModBus_GetParameter_Float(MB_ADDR_COMMAND_PARAM);
        Grad_ProcessCommand(cmd, param);
        ModBus_SetParameter_Int(MB_ADDR_COMMAND, 0);
        return;
    }

    switch (cmd) {
        case 03: {
            float waveguide_len = ModBus_GetWaveguideLength();
            USART2_Print("[CAL] Команда 03: Длина звукопровода = ");
            USART2_BufInit();
            USART2_BufPrintFloat(waveguide_len * 1000.0f);
            USART2_BufPrint(" мм\r\n");
            USART2_BufFlush();
            break;
        }
        case 02: {
            uint32_t measurement = measure_time_of_flight();
            if (measurement > 0 && !tof_timeout) {
                float tof_us = (float)measurement * TOF_TICK_US;
                float distance_mm = tof_us * 0.001f * SOUND_SPEED_MPS;
                float h_high = ModBus_GetParameter_Float(MB_ADDR_CAL_HIGH_LVL);
                ModBus_SetParameter_Float(MB_ADDR_CAL_C2, distance_mm);
                USART2_Print("[CAL] Команда 02: ВЕРХНЯЯ точка (полный бак, 100%)\r\n");
                USART2_BufInit();
                USART2_BufPrint("      C2 = ");
                USART2_BufPrintFloat(distance_mm);
                USART2_BufPrint(" мм (ToF = ");
                USART2_BufPrintFloat(tof_us);
                USART2_BufPrint(" мкс, h¯ = ");
                USART2_BufPrintFloat(h_high * 1000.0f);
                USART2_BufPrint(" мм)\r\n");
                USART2_BufFlush();
            } else {
                USART2_Print("[CAL] Команда 02: ОШИБКА измерения!\r\n");
            }
            break;
        }
        case 01: {
            uint32_t measurement = measure_time_of_flight();
            if (measurement > 0 && !tof_timeout) {
                float tof_us = (float)measurement * TOF_TICK_US;
                float distance_mm = tof_us * 0.001f * SOUND_SPEED_MPS;
                float h_low = ModBus_GetParameter_Float(MB_ADDR_CAL_LOW_LVL);
                ModBus_SetParameter_Float(MB_ADDR_CAL_C1, distance_mm);
                USART2_Print("[CAL] Команда 01: НИЖНЯЯ точка (пустой бак, 0%)\r\n");
                USART2_BufInit();
                USART2_BufPrint("      C1 = ");
                USART2_BufPrintFloat(distance_mm);
                USART2_BufPrint(" мм (ToF = ");
                USART2_BufPrintFloat(tof_us);
                USART2_BufPrint(" мкс, h_ = ");
                USART2_BufPrintFloat(h_low * 1000.0f);
                USART2_BufPrint(" мм)\r\n");
                USART2_BufFlush();
            } else {
                USART2_Print("[CAL] Команда 01: ОШИБКА измерения!\r\n");
            }
            break;
        }
        case 04:
            USART2_Print("[CAL] Команда 04: Разность высот магнитов\r\n");
            break;
        default:
            break;
    }

    ModBus_SetParameter_Int(MB_ADDR_COMMAND, 0);
}

/* ==========================================================================
   СТАТИСТИКА
   ========================================================================== */
static void Stat_AddValue(uint32_t value)
{
    measurement_history[stat_index] = value;
    stat_index++;
    if (stat_index >= STAT_HISTORY_SIZE) {
        stat_index = 0;
        stat_ready = 1;
    }
    if (stat_count < STAT_HISTORY_SIZE) {
        stat_count++;
    }
}

static float Stat_CalculateTrimmedAverage(void)
{
    if (stat_count < 3) return 0.0f;

    uint32_t sum = 0;
    uint32_t min_val = 0xFFFFFFFF;
    uint32_t max_val = 0;
    uint8_t count = stat_count;
    uint8_t valid_count = 0;

    for (uint8_t i = 0; i < count; i++) {
        uint32_t val = measurement_history[i];
        if (val > 0) {
            sum += val;
            if (val < min_val) min_val = val;
            if (val > max_val) max_val = val;
            valid_count++;
        }
    }

    if (valid_count >= 3) {
        sum -= min_val;
        sum -= max_val;
        valid_count -= 2;
    }

    if (valid_count == 0) return 0.0f;
    return (float)sum / (float)valid_count;
}

static void Stat_ClearHistory(void)
{
    for (uint8_t i = 0; i < STAT_HISTORY_SIZE; i++) {
        measurement_history[i] = 0;
    }
    stat_index = 0;
    stat_count = 0;
    stat_ready = 0;
}

/* ==========================================================================
   ФУНКЦИЯ: Передача кадра Modbus
   ========================================================================== */
void ModBus_TransmitFrame(uint8_t *frame, uint16_t len)
{
    if (len == 0) return;

    uint32_t start_wait = HAL_GetTick();
    while (modbus_tx_active) {
        if (HAL_GetTick() - start_wait > 100) break;
    }

    RS485_SET_TRANSMIT();
    modbus_tx_active = 1;

    for (volatile int i = 0; i < 150; i++) __NOP();

    HAL_UART_Transmit(&huart1, frame, len, 100);

    uint32_t timeout_start = HAL_GetTick();
    while ((USART1->SR & USART_SR_TC) == 0) {
        if (HAL_GetTick() - timeout_start > 50) break;
    }

    for (volatile int i = 0; i < 800; i++) __NOP();

    RS485_SET_RECEIVE();
    modbus_tx_active = 0;
}

/* ==========================================================================
   ФУНКЦИИ: Вывод через USART2
   ========================================================================== */
static void USART2_PrintInt(int32_t val)
{
    char buf[12];
    int8_t i = 0, len = 0;
    if (val < 0) { USART2_Print("-"); val = -val; }
    if (val == 0) { USART2_Print("0"); return; }
    do {
        buf[i++] = (val % 10) + '0';
        val /= 10;
    } while (val > 0);
    len = i;
    for (i = len - 1; i >= 0; i--) {
        char c = buf[i];
        HAL_UART_Transmit(&huart2, (uint8_t*)&c, 1, 10);
    }
}

static void USART2_PrintFloat(float val)
{
    int32_t int_part = (int32_t)val;
    float frac = val - (float)int_part;
    if (frac < 0) frac = -frac;
    int32_t frac_part = (int32_t)(frac * 100.0f + 0.5f);
    if (frac_part >= 100) frac_part = 0;
    USART2_PrintInt(int_part);
    USART2_Print(".");
    if (frac_part < 10) USART2_Print("0");
    USART2_PrintInt(frac_part);
}

/* ==========================================================================
   ФУНКЦИЯ: Отслеживание изменения напряжений
   ========================================================================== */
static void Check_Voltage_Change(const char *name, float new_val, float old_val, float *store_val)
{
    float diff = new_val - old_val;
    if (diff < 0) diff = -diff;

    if (diff > 0.1f || old_val == 0.0f) {
        USART2_Print("[VOLT] ");
        USART2_Print(name);
        USART2_Print(": ");
        USART2_PrintFloat(new_val);
        USART2_Print(" В (изм: ");
        if (old_val > 0.0f) { USART2_PrintFloat(diff); }
        else { USART2_Print("init"); }
        USART2_Print(")\r\n");
        *store_val = new_val;
    }
}

/* ==========================================================================
   ФУНКЦИЯ: Чтение периода опроса из Modbus
   ========================================================================== */
static void Update_Poll_Period_From_Modbus(void)
{
    uint16_t period_ms = ModBus_GetParameter_Int(MB_ADDR_POLL_PERIOD);

    if (period_ms < MIN_POLL_PERIOD_MS) {
        period_ms = MIN_POLL_PERIOD_MS;
        ModBus_SetParameter_Int(MB_ADDR_POLL_PERIOD, MIN_POLL_PERIOD_MS);
    }
    if (period_ms > MAX_POLL_PERIOD_MS) {
        period_ms = MAX_POLL_PERIOD_MS;
        ModBus_SetParameter_Int(MB_ADDR_POLL_PERIOD, MAX_POLL_PERIOD_MS);
    }

    current_poll_period_ms = period_ms;
}

/* ==========================================================================
   ФУНКЦИЯ: Обработка результатов измерения
   ========================================================================== */
void Process_Measurement_Results(float tof_us, float position_mm, uint8_t signal_captured)
{
    float waveguide_len = ModBus_GetWaveguideLength();
    float cal_low = ModBus_GetParameter_Float(MB_ADDR_CAL_LOW_LVL);
    float cal_high = ModBus_GetParameter_Float(MB_ADDR_CAL_HIGH_LVL);
    float C1_mm = ModBus_GetParameter_Float(MB_ADDR_CAL_C1);
    float C2_mm = ModBus_GetParameter_Float(MB_ADDR_CAL_C2);

    float waveguide_len_mm = waveguide_len * 1000.0f;
    float cal_low_mm = cal_low * 1000.0f;
    float cal_high_mm = cal_high * 1000.0f;

    if (cal_low_mm < 0.0f || cal_low_mm > 50000.0f || cal_low_mm != cal_low_mm) {
        cal_low_mm = 100.0f;
    }
    if (cal_high_mm < 0.0f || cal_high_mm > 50000.0f || cal_high_mm != cal_high_mm) {
        cal_high_mm = 1000.0f;
    }

    if (position_mm < 0.0f) position_mm = 0.0f;
    if (position_mm > waveguide_len_mm) position_mm = waveguide_len_mm;

    USART2_BufInit();

    if (signal_captured) {
        USART2_BufPrint("\r\n===========================================================\r\n");
        USART2_BufPrint("                    РЕЗУЛЬТАТЫ ИЗМЕРЕНИЯ                   \r\n");
        USART2_BufPrint("===========================================================\r\n");

        if (C1_mm > 0.0f && C2_mm > 0.0f && C1_mm > C2_mm) {
            USART2_BufPrint("[CAL] Калибровка АКТИВНА\r\n");
            USART2_BufPrint("      C1 = ");
            USART2_BufPrintFloat(C1_mm);
            USART2_BufPrint(" мм (нижняя точка, пустой бак)\r\n");
            USART2_BufPrint("      C2 = ");
            USART2_BufPrintFloat(C2_mm);
            USART2_BufPrint(" мм (верхняя точка, полный бак)\r\n");
        } else {
            USART2_BufPrint("[CAL] Калибровка НЕ проведена (используется скорость звука)\r\n");
        }

        USART2_BufPrint("[DBG] Захвачено импульсов: ");
        USART2_BufPrintInt(capture_count);
        USART2_BufPrint("\r\n");

        if (capture_count >= 2) {
            USART2_BufPrint("  Импульс[0] = ");
            USART2_BufPrintInt(captured_pulses[0]);
            USART2_BufPrint(" тиков (");
            USART2_BufPrintFloat((float)captured_pulses[0] * TOF_TICK_US);
            USART2_BufPrint(" мкс)\r\n");
            USART2_BufPrint("  Импульс[1] = ");
            USART2_BufPrintInt(captured_pulses[1]);
            USART2_BufPrint(" тиков (");
            USART2_BufPrintFloat((float)captured_pulses[1] * TOF_TICK_US);
            USART2_BufPrint(" мкс)\r\n");

            float width_ticks = (float)(captured_pulses[1] - captured_pulses[0]);
            float width_time_us = width_ticks * TOF_TICK_US;

            USART2_BufPrint("  Разница: ");
            USART2_BufPrintFloat(width_ticks);
            USART2_BufPrint(" тиков = ");
            USART2_BufPrintFloat(width_time_us);
            USART2_BufPrint(" мкс\r\n");

            if (width_time_us < MIN_CLICK_WIDTH_US || width_time_us > MAX_CLICK_WIDTH_US) {
                USART2_BufPrint("  X ОТБРОШЕНО: пауза вне диапазона\r\n");
            } else {
                USART2_BufPrint("  V ВАЛИДНАЯ ПАРА\r\n");

                float tof_time_us = (float)captured_pulses[0] * TOF_TICK_US;
                float measured_distance_mm = tof_time_us * 0.001f * SOUND_SPEED_MPS;
                float position = Calculate_Position(tof_time_us);

                float level_percent = 0.0f;
                if (cal_low_mm > 0.0f && cal_high_mm > cal_low_mm) {
                    level_percent = ((cal_high_mm - position) / (cal_high_mm - cal_low_mm)) * 100.0f;
                }
                if (level_percent > 100.0f) level_percent = 100.0f;
                if (level_percent < 0.0f) level_percent = 0.0f;

                USART2_BufPrint("  ToF = ");
                USART2_BufPrintFloat(tof_time_us);
                USART2_BufPrint(" мкс | Расстояние = ");
                USART2_BufPrintFloat(measured_distance_mm);
                USART2_BufPrint(" мм\r\n");
                USART2_BufPrint("  Уровень заполнения = ");
                USART2_BufPrintFloat(level_percent);
                USART2_BufPrint("%\r\n");

                /* === РАСЧЕТ ОБЪЕМА ЧЕРЕЗ ГРАДУИРОВОЧНУЮ ТАБЛИЦУ === */
                float tank_height = ModBus_GetParameter_Float(MB_ADDR_TANK_HEIGHT);
                float tank_volume = ModBus_GetParameter_Float(MB_ADDR_TANK_VOLUME);
                GradTankType_t tank_type = (GradTankType_t)(int)ModBus_GetParameter_Float(MB_ADDR_TANK_GEOM);

                float volume_m3 = 0.0f;
                if (tank_type == GRAD_TYPE_BY_TABLE && Grad_IsValid()) {
                    volume_m3 = Grad_InterpolateVolume(position / 1000.0f);
                } else {
                    volume_m3 = Grad_CalculateVolume(tank_type, position / 1000.0f,
                                                      tank_height, tank_volume);
                }

                /* Обновляем регистры Modbus */
                ModBus_SetParameter_Float(MB_ADDR_VOLUME, volume_m3);
                ModBus_SetParameter_Int(MB_ADDR_VOLUME_INT, (uint16_t)(volume_m3 * 100.0f));

                USART2_BufPrint("  Объем = ");
                USART2_BufPrintFloat(volume_m3);
                USART2_BufPrint(" м³ (тип резервуара: ");
                USART2_BufPrintInt((int32_t)tank_type);
                USART2_BufPrint(")\r\n");
            }
        }

        USART2_BufPrint("-----------------------------------------------------------\r\n");
        USART2_BufPrint("  Волновод: ");
        USART2_BufPrintFloat(waveguide_len_mm);
        USART2_BufPrint(" мм | Температура: ");
        USART2_BufPrintFloat(current_temperature);
        USART2_BufPrint(" C\r\n");
        USART2_BufPrint("  Калибровка: ");
        USART2_BufPrintFloat(cal_low_mm);
        USART2_BufPrint(" .. ");
        USART2_BufPrintFloat(cal_high_mm);
        USART2_BufPrint(" мм\r\n");
        USART2_BufPrint("  Период опроса: ");
        USART2_BufPrintInt(current_poll_period_ms);
        USART2_BufPrint(" мс\r\n");
        USART2_BufPrint("===========================================================\r\n");
    } else {
        USART2_BufPrint("[ИЗМ] Сигнал не захвачен (таймаут)\r\n");
    }

    USART2_BufFlush();
}

/* ==========================================================================
   MAIN: Точка входа
   ========================================================================== */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();

    if (MX_I2C2_Init() == HAL_OK) {
        USART2_Print("[I2C2] Инициализирована (PB10=SCL, PB11=SDA)\r\n");
        current_temperature = 0.0f;
    } else {
        USART2_Print("[I2C2] ОШИБКА инициализации!\r\n");
    }

    TIM3_InputCapture_Init();

    USART2_Print("[ИНИЦ] Калибровка АЦП...\r\n");
    if (HAL_ADCEx_Calibration_Start(&hadc1) == HAL_OK) USART2_Print("[ИНИЦ] АЦП1 OK\r\n");
    else { v24_error = 1; v12_error = 1; v5_error = 1; vdda_error = 1; }

    if (HAL_ADCEx_Calibration_Start(&hadc2) == HAL_OK) USART2_Print("[ИНИЦ] АЦП2 OK\r\n");
    else { v12_error = 1; v5_error = 1; }

    ModBus_Init();
    ModBus_UpdateFirmwareVersion(FIRMWARE_VERSION);

    /* === ИНИЦИАЛИЗАЦИЯ НОВЫХ МОДУЛЕЙ === */
    TempSensors_Init();
    Grad_Init();

    Stat_ClearHistory();

    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
    __enable_irq();

    Read_All_Voltages();
    Read_Temperature();
    ModBus_UpdateVoltages(current_vdda, current_24v, current_12v, current_5v);
    Update_Poll_Period_From_Modbus();

    USART2_BufInit();
    USART2_BufPrint("Modbus: Addr=1, Baud=19200\r\n");
    USART2_BufPrint("[DBG] Период: ");

    if (current_poll_period_ms >= 1000) {
        USART2_BufPrintInt(current_poll_period_ms / 1000);
        USART2_BufPrint(" сек\r\n");
    } else {
        USART2_BufPrintInt(current_poll_period_ms);
        USART2_BufPrint(" мс\r\n");
    }

    float C1 = ModBus_GetParameter_Float(MB_ADDR_CAL_C1);
    float C2 = ModBus_GetParameter_Float(MB_ADDR_CAL_C2);
    if (C1 > 0.0f && C2 > 0.0f && C1 > C2) {
        USART2_BufPrint("[CAL] Калибровка найдена: C1=");
        USART2_BufPrintFloat(C1);
        USART2_BufPrint(" мм, C2=");
        USART2_BufPrintFloat(C2);
        USART2_BufPrint(" мм\r\n");
    } else {
        USART2_BufPrint("[CAL] Калибровка НЕ найдена (используется скорость звука)\r\n");
    }
    USART2_BufFlush();

    uint32_t last_measure_time = 0;
    uint32_t last_debug_time = 0;
    uint32_t led_red_off_time = 0;
    uint32_t last_temp_read_time = 0;
    uint8_t blue_led_state = 0;
    uint8_t red_led_state = 0;

    while (1) {
        /* Мигание синим светодиодом раз в секунду */
        if (HAL_GetTick() - last_debug_time >= 1000) {
            last_debug_time = HAL_GetTick();
            blue_led_state = !blue_led_state;
            HAL_GPIO_WritePin(GPIOB, LED_BLUE_PIN, blue_led_state ? LED_BLUE_ON : LED_BLUE_OFF);
        }

        /* Гашение красного светодиода */
        if (red_led_state && (HAL_GetTick() - led_red_off_time >= LED_RED_ON_TIME_MS)) {
            HAL_GPIO_WritePin(GPIOB, LED_RED_PIN, LED_RED_OFF);
            red_led_state = 0;
        }

        /* === Периодический опрос датчиков температуры (каждые 2 секунды) === */
        if (HAL_GetTick() - last_temp_read_time >= 2000) {
            last_temp_read_time = HAL_GetTick();
            Read_Temperature();
        }

        /* Проверка команд калибровки через Modbus */
        uint16_t cmd = ModBus_GetParameter_Int(MB_ADDR_COMMAND);
        if (cmd > 0) {
            USART2_Print("[CAL] Получена команда: ");
            USART2_PrintInt(cmd);
            USART2_Print("\r\n");
            Process_Calibration_Command(cmd);
        }

        /* Измерение уровня */
        if (HAL_GetTick() - last_measure_time >= current_poll_period_ms) {
            last_measure_time = HAL_GetTick();
            Update_Poll_Period_From_Modbus();

            HAL_GPIO_WritePin(GPIOB, LED_RED_PIN, LED_RED_OFF);
            red_led_state = 0;
            signal_captured = 0;

            uint32_t measurement = measure_time_of_flight();
            float tof_us = 0.0f;
            float position_mm = 0.0f;

            if (measurement > 0 && !tof_timeout) {
                Stat_AddValue(measurement);
                if (stat_ready) {
                    float avg_ticks = Stat_CalculateTrimmedAverage();
                    if (avg_ticks > 0.0f) {
                        tof_us = avg_ticks * TOF_TICK_US;
                        position_mm = Calculate_Position(tof_us);

                        USART2_BufInit();
                        USART2_BufPrint("[STAT] Среднее: ");
                        USART2_BufPrintFloat(avg_ticks);
                        USART2_BufPrint(" тиков -> ");
                        USART2_BufPrintFloat(position_mm);
                        USART2_BufPrint(" мм\r\n");
                        USART2_BufFlush();

                        HAL_GPIO_WritePin(GPIOB, LED_RED_PIN, LED_RED_ON);
                        red_led_state = 1;
                        led_red_off_time = HAL_GetTick();
                        signal_captured = 1;
                        Stat_ClearHistory();

                        ModBus_UpdateMeasurements(position_mm, current_temperature, ModBus_GetWaveguideLength());
                    }
                }
            }

            if (stat_ready || signal_captured) {
                Process_Measurement_Results(tof_us, position_mm, signal_captured);
            }
        }

        Read_All_Voltages();
        ModBus_UpdateVoltages(current_vdda, current_24v, current_12v, current_5v);
        ModBus_Process();

        if (modbus_tx_active && (HAL_GetTick() - last_measure_time > 100)) {
            RS485_SET_RECEIVE();
            modbus_tx_active = 0;
        }

        HAL_Delay(1);
    }
}

/* ==========================================================================
   CALLBACK: Приём Modbus через USART1
   ========================================================================== */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        ModBus_RxCallback(huart);
    }
}

/* ==========================================================================
   ФУНКЦИЯ: Инициализация TIM3 для Input Capture
   ========================================================================== */
void TIM3_InputCapture_Init(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_AFIO_REMAP_TIM3_PARTIAL();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    TIM3->CR1 = 0;
    TIM3->CR2 = 0;
    TIM3->PSC = 6;
    TIM3->ARR = 0xFFFF;
    TIM3->CNT = 0;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->CCMR2 = (0x1 << 12) | (0x1 << 8);
    TIM3->CCER = TIM_CCER_CC4E;
    TIM3->DIER = 0;
    TIM3->SR = 0;
    TIM3->CR1 = TIM_CR1_CEN;
}

/* ==========================================================================
   ФУНКЦИЯ: Генерация импульса и запуск измерения ToF
   ========================================================================== */
void generate_pulse_and_measure(void)
{
    tof_measurement_done = 0;
    tof_timeout = 0;
    tof_capture_value = 0;
    capture_count = 0;
    last_capture_cnt = 0;

    for (uint8_t i = 0; i < MAX_CAPTURED_PULSES; i++) {
        captured_pulses[i] = 0;
    }

    TIM3->SR = 0;
    TIM3->CNT = 0;
    __DSB();
    TIM3->CR1 |= TIM_CR1_CEN;
    __NOP();

    HAL_GPIO_WritePin(SWITCH_PORT, SWITCH_PIN, GPIO_PIN_RESET);
    GPIOB->BSRR = GPIO_PIN_5;

    for (volatile uint32_t i = 0; i < PULSE_DELAY_ITERATIONS*5; i++) __NOP();

    GPIOB->BRR = GPIO_PIN_5;

    for (volatile uint32_t i = 0; i < DELAY_AFTER_PULSE_ITER; i++) __NOP();

    HAL_GPIO_WritePin(SWITCH_PORT, SWITCH_PIN, GPIO_PIN_SET);
}

/* ==========================================================================
   ФУНКЦИЯ: Измерение времени пролёта (ToF)
   ========================================================================== */
uint32_t measure_time_of_flight(void)
{
    TIM3->SR = 0;
    TIM3->DIER |= TIM_DIER_CC4IE;

    generate_pulse_and_measure();

    uint32_t start_wait = HAL_GetTick();
    while (!tof_measurement_done && !tof_timeout) {
        if ((HAL_GetTick() - start_wait) >= MEAS_TIMEOUT_MS) {
            tof_timeout = 1;
            TIM3->CR1 &= ~TIM_CR1_CEN;
            break;
        }
        __NOP();
    }

    TIM3->CR1 &= ~TIM_CR1_CEN;
    TIM3->SR = 0;

    for (volatile uint32_t i = 0; i < SWITCH_HOLD_ITERATIONS; i++) __NOP();
    HAL_GPIO_WritePin(SWITCH_PORT, SWITCH_PIN, GPIO_PIN_RESET);

    return (capture_count >= 2) ? captured_pulses[0] : 0;
}

/* ==========================================================================
   ФУНКЦИЯ: Опрос датчиков температуры (интеграция с temp_sensors)
   ========================================================================== */
void Read_Temperature(void)
{
    /* Опрос всех датчиков */
    TempSensors_ReadAll();

    /* Получаем текущий уровень для расчета средних температур */
    float level_m = ModBus_GetParameter_Float(MB_ADDR_LEVEL) / 1000.0f;
    float sep_level = ModBus_GetParameter_Float(MB_ADDR_LEVEL_SEP);

    /* Расчет средних температур */
    TempSensors_CalculateAverages(level_m, sep_level);

    /* Обновляем глобальную переменную (для обратной совместимости) */
    TempSensorsState_t *state = TempSensors_GetState();
    if (!isnan(state->avg_liquid_temp)) {
        current_temperature = state->avg_liquid_temp;
    }
}

/* ==========================================================================
   ФУНКЦИЯ: Чтение всех напряжений через АЦП
   ========================================================================== */
void Read_All_Voltages(void)
{
    uint32_t adc_raw_vdda = 0, adc_raw_24v = 0, adc_raw_12v = 0, adc_raw_5v = 0;

    ADC1->CR2 |= ADC_CR2_TSVREFE;
    HAL_Delay(10);
    adc_raw_vdda = Read_ADC_Average(&hadc1, ADC_CHANNEL_VREFINT, ADC_SAMPLETIME_239CYCLES_5, ADC_SAMPLES);
    ADC1->CR2 &= ~ADC_CR2_TSVREFE;
    HAL_Delay(1);

    if (adc_raw_vdda > 1000 && adc_raw_vdda < 2000 && *VREFINT_CAL_VALUE > 1000 && *VREFINT_CAL_VALUE < 2000) {
        current_vdda = 3.3f * (float)(*VREFINT_CAL_VALUE) / (float)adc_raw_vdda;
        vdda_error = 0;
    } else { current_vdda = 3.3f; vdda_error = 1; }

    adc_raw_24v = Read_ADC_Average(&hadc1, ADC_CHANNEL_0, ADC_SAMPLETIME_239CYCLES_5, ADC_SAMPLES);
    if (adc_raw_24v > 100 && adc_raw_24v < 4000) {
        current_24v = ((float)adc_raw_24v * current_vdda / 4095.0f) * DIV_24V_FACTOR;
        v24_error = 0;
    } else { current_24v = 24.0f; v24_error = 1; }

    adc_raw_12v = Read_ADC_Average(&hadc2, ADC_CHANNEL_1, ADC_SAMPLETIME_239CYCLES_5, ADC_SAMPLES);
    if (adc_raw_12v > 100 && adc_raw_12v < 4000) {
        current_12v = ((float)adc_raw_12v * current_vdda / 4095.0f) * DIV_12V_FACTOR;
        v12_error = 0;
    } else { current_12v = 12.0f; v12_error = 1; }

    adc_raw_5v = Read_ADC_Average(&hadc2, ADC_CHANNEL_5, ADC_SAMPLETIME_239CYCLES_5, ADC_SAMPLES);
    if (adc_raw_5v > 100 && adc_raw_5v < 4000) {
        current_5v = ((float)adc_raw_5v * current_vdda / 4095.0f) * DIV_5V_FACTOR;
        v5_error = 0;
    } else { current_5v = 5.0f; v5_error = 1; }

    Check_Voltage_Change("VDDA", current_vdda, prev_vdda, &prev_vdda);
    Check_Voltage_Change("+24V", current_24v, prev_24v, &prev_24v);
    Check_Voltage_Change("+12V", current_12v, prev_12v, &prev_12v);
}

/* ==========================================================================
   ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ АЦП
   ========================================================================== */
uint32_t Read_ADC_Single(ADC_HandleTypeDef *hadc, uint32_t channel, uint32_t sampling_time)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = sampling_time;

    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) return 0;

    HAL_ADC_Start(hadc);
    if (HAL_ADC_PollForConversion(hadc, 10) != HAL_OK) {
        HAL_ADC_Stop(hadc);
        return 0;
    }

    uint32_t val = HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);

    return val;
}

uint32_t Read_ADC_Average(ADC_HandleTypeDef *hadc, uint32_t channel, uint32_t sampling_time, uint8_t samples)
{
    uint32_t sum = 0, valid = 0;
    for (uint8_t i = 0; i < samples; i++) {
        uint32_t v = Read_ADC_Single(hadc, channel, sampling_time);
        if (v > 100 && v < 4000) { sum += v; valid++; }
        HAL_Delay(1);
    }
    return (valid > 0) ? sum / valid : 0;
}

/* ==========================================================================
   СИСТЕМНЫЕ ФУНКЦИИ
   ========================================================================== */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                   RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
}

void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_5 | SWITCH_PIN | LED_RED_PIN | LED_BLUE_PIN | RS485_CTRL_PIN;
    GPIO_InitStruct.Mode = (GPIO_InitStruct.Pin == GPIO_PIN_1) ? GPIO_MODE_INPUT : GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = (GPIO_InitStruct.Pin == GPIO_PIN_1 || GPIO_InitStruct.Pin == GPIO_PIN_5 ||
                             GPIO_InitStruct.Pin == SWITCH_PIN || GPIO_InitStruct.Pin == RS485_CTRL_PIN)
                            ? GPIO_SPEED_FREQ_HIGH : GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(SWITCH_PORT, SWITCH_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, LED_RED_PIN, LED_RED_OFF);
    HAL_GPIO_WritePin(GPIOB, LED_BLUE_PIN, LED_BLUE_OFF);
    HAL_GPIO_WritePin(RS485_CTRL_PORT, RS485_CTRL_PIN, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 19200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);
}

void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);
}

void MX_ADC1_Init(void)
{
    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    HAL_ADC_Init(&hadc1);
}

void MX_ADC2_Init(void)
{
    hadc2.Instance = ADC2;
    hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc2.Init.ContinuousConvMode = DISABLE;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc2.Init.NbrOfConversion = 1;
    HAL_ADC_Init(&hadc2);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {
        HAL_GPIO_TogglePin(GPIOB, LED_RED_PIN);
        HAL_Delay(200);
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { }
#endif
