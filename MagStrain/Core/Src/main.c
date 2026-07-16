/* USER CODE BEGIN Header */
/*
 * @file           : main.c
 * @brief          : Основной цикл ПМП-201Е
 *                   - Буферизованный вывод в USART2 (оптимизация скорости)
 *                   - Интеграция с AT24C64 EEPROM (адрес 0x51)
 *                   - LM75B отключён (температура = 0°C)
 *                   - Мёртвое время 60 мкс в TIM3_IRQHandler
 */
/* USER CODE END Header */

#include "main.h"
#include "stm32f1xx_it.h"
#include "utils.h"
#include "i2c_config.h"
#include "modbus.h"
#include "at24c64.h"
#include <stdint.h>
#include <string.h>

/* ==========================================================================
КОНСТАНТЫ И МАКРОСЫ
========================================================================== */
#define TIMER_CLOCK_HZ          72000000.0f
#define TOF_TICK_US             0.0972f
#define VREFINT_CAL_ADDR        0x1FFFF7BA
#define VREFINT_CAL_VALUE       ((uint16_t *)VREFINT_CAL_ADDR)
#define ADC_SAMPLES             16
#define MEAS_TIMEOUT_MS         50
#define DIV_24V_FACTOR          13.0f
#define DIV_12V_FACTOR          4.0f
#define DIV_5V_FACTOR           2.0f
#define PULSE_PERIOD_MS_DEFAULT 1000
#define SOUND_SPEED_MPS         4900.0f

/* === СТАТИСТИКА ИЗМЕРЕНИЙ === */
#define STAT_HISTORY_SIZE       11

/* === СВЕТОДИОДЫ === */
#define LED_RED_PIN             GPIO_PIN_13
#define LED_BLUE_PIN            GPIO_PIN_12
#define LED_RED_ON_TIME_MS      1000
#define LED_RED_ON              GPIO_PIN_SET
#define LED_RED_OFF             GPIO_PIN_RESET
#define LED_BLUE_ON             GPIO_PIN_SET
#define LED_BLUE_OFF            GPIO_PIN_RESET

/* === Импульсы и коммутация === */
#define PULSE_DELAY_ITERATIONS  10
#define DELAY_AFTER_PULSE_ITER  97
#define SWITCH_HOLD_ITERATIONS  12000
#define SWITCH_PIN              GPIO_PIN_7
#define SWITCH_PORT             GPIOB

/* === Modbus RS-485 === */
#define MODBUS_SILENCE_TIME_MS  4
#define RS485_CTRL_PIN          GPIO_PIN_0
#define RS485_CTRL_PORT         GPIOB
#define FIRMWARE_VERSION        100

/* === Ограничения периода опроса === */
#define MIN_POLL_PERIOD_MS      1000
#define MAX_POLL_PERIOD_MS      60000

/* ★ ПОРОГИ ШИРИНЫ ИМПУЛЬСА НА CLIK (между 1-м и 2-м импульсами значащей пары) ★ */
#define MIN_CLICK_WIDTH_US      14.0f
#define MAX_CLICK_WIDTH_US      30.0f

//#define TEST_MODE   1

/* ==========================================================================
ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ
========================================================================== */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

/* --- Состояние захвата ToF --- */
volatile uint32_t tof_capture_value = 0;
volatile uint8_t tof_measurement_done = 0;
volatile uint8_t tof_timeout = 0;
volatile uint8_t signal_captured = 0;
volatile uint32_t captured_pulses[MAX_CAPTURED_PULSES];
volatile uint8_t capture_count = 0;
volatile uint8_t expected_pulse_pairs = MAX_PULSE_PAIRS;
volatile uint32_t last_capture_cnt = 0;

/* --- Modbus TX флаг --- */
volatile uint8_t modbus_tx_active = 0;

/* === СТАТИСТИКА: История измерений === */
static uint32_t measurement_history[STAT_HISTORY_SIZE] = {0};
static uint8_t stat_index = 0;
static uint8_t stat_count = 0;
static uint8_t stat_ready = 0;

/* --- Текущие значения --- */
static float current_vdda = 3.3f;
static float current_24v = 24.0f;
static float current_12v = 12.0f;
static float current_5v = 5.0f;
static float current_temperature = 0.0f;  /* LM75B отключён */
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
БУФЕРНЫЙ ВЫВОД В USART2 (оптимизация скорости)
★ ВАЖНО: функции объявлены БЕЗ static, т.к. прототипы в main.h без static
========================================================================== */
#define USART2_TX_BUF_SIZE      512
static char tx_buf[USART2_TX_BUF_SIZE];
static uint16_t tx_buf_len = 0;

void USART2_BufInit(void)
{
    tx_buf_len = 0;
}

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
    while (*str) {
        buf_putc(*str++);
    }
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
СТАТИСТИКА: Добавление значения в историю
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

/* ==========================================================================
СТАТИСТИКА: Расчёт усреднённого значения (отброс мин/макс)
========================================================================== */
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

/* ==========================================================================
СТАТИСТИКА: Очистка истории (сброс для нового цикла измерений)
========================================================================== */
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
ФУНКЦИИ: Вывод через USART2 (по-символьно, для простых сообщений)
========================================================================== */
static void USART2_PrintInt(int32_t val)
{
    char buf[12];
    int8_t i = 0, len = 0;

    if (val < 0) {
        USART2_Print("-");
        val = -val;
    }

    if (val == 0) {
        USART2_Print("0");
        return;
    }

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

    if (diff > 0.1f || old_val == 0.0f)
    {
        USART2_Print("[VOLT] ");
        USART2_Print(name);
        USART2_Print(": ");
        USART2_PrintFloat(new_val);
        USART2_Print(" В (изм: ");
        if (old_val > 0.0f) {
            USART2_PrintFloat(diff);
        } else {
            USART2_Print("init");
        }
        USART2_Print(")\r\n");
        *store_val = new_val;
    }
}

/* ==========================================================================
ФУНКЦИЯ: Чтение периода опроса из Modbus
========================================================================== */
static void Update_Poll_Period_From_Modbus(void)
{
    float period_sec = ModBus_GetParameter_Float(MB_ADDR_POLL_PERIOD);
    uint32_t period_ms = (uint32_t)(period_sec * 1000.0f);

    if (period_ms < MIN_POLL_PERIOD_MS) {
        period_ms = MIN_POLL_PERIOD_MS;
        ModBus_SetParameter_Float(MB_ADDR_POLL_PERIOD, (float)MIN_POLL_PERIOD_MS / 1000.0f);
    }
    if (period_ms > MAX_POLL_PERIOD_MS) {
        period_ms = MAX_POLL_PERIOD_MS;
        ModBus_SetParameter_Float(MB_ADDR_POLL_PERIOD, (float)MAX_POLL_PERIOD_MS / 1000.0f);
    }

    current_poll_period_ms = period_ms;
}

/* ==========================================================================
ФУНКЦИЯ: Обработка результатов измерения (БУФЕРИЗОВАННЫЙ ВЫВОД)
========================================================================== */
void Process_Measurement_Results(float tof_us, float position_mm, uint8_t signal_captured)
{
    float waveguide_len = ModBus_GetWaveguideLength();
    float cal_low = ModBus_GetParameter_Float(MB_ADDR_CAL_LOW_LVL);
    float cal_high = ModBus_GetParameter_Float(MB_ADDR_CAL_HIGH_LVL);

    if (cal_low < 0.1f || cal_low > 100.0f) {
        cal_low = 0.1f;
    }
    if (cal_high < 1.0f || cal_high > 50000.0f) {
        cal_high = 3.0f;
    }
    if (position_mm < 0.0f) {
        position_mm = 0.0f;
    }
    if (position_mm > waveguide_len) {
        position_mm = waveguide_len;
    }

    ModBus_UpdateMeasurements(position_mm, current_temperature, waveguide_len);
    ModBus_UpdateVoltages(current_vdda, current_24v, current_12v, current_5v);
    ModBus_UpdateFirmwareVersion(FIRMWARE_VERSION);

    /* Буферизованный вывод */
    USART2_BufInit();

    if (signal_captured)
    {
        USART2_BufPrint("\r\n");
        USART2_BufPrint("===========================================================\r\n");
        USART2_BufPrint("                    РЕЗУЛЬТАТЫ ИЗМЕРЕНИЯ                   \r\n");
        USART2_BufPrint("===========================================================\r\n");
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
                USART2_BufPrint("  X ОТБРОШЕНО: пауза ");
                USART2_BufPrintFloat(width_time_us);
                USART2_BufPrint(" мкс вне диапазона ");
                USART2_BufPrintFloat(MIN_CLICK_WIDTH_US);
                USART2_BufPrint("-");
                USART2_BufPrintFloat(MAX_CLICK_WIDTH_US);
                USART2_BufPrint(" мкс\r\n");
            } else {
                USART2_BufPrint("  V ВАЛИДНАЯ ПАРА\r\n");

                float tof_time_us = (float)captured_pulses[0] * TOF_TICK_US;
                float position = (tof_time_us * 0.001f * SOUND_SPEED_MPS) / 2.0f;
                float level_percent = 0.0f;
                if (waveguide_len > 0.0f) {
                    level_percent = (position / waveguide_len) * 100.0f;
                }
                if (level_percent > 100.0f) {
                    level_percent = 100.0f;
                }

                USART2_BufPrint("  ToF = ");
                USART2_BufPrintFloat(tof_time_us);
                USART2_BufPrint(" мкс | Уровень = ");
                USART2_BufPrintFloat(position);
                USART2_BufPrint(" мм (");
                USART2_BufPrintFloat(level_percent);
                USART2_BufPrint("%)\r\n");
            }
        }

        USART2_BufPrint("-----------------------------------------------------------\r\n");
        USART2_BufPrint("  Волновод: ");
        USART2_BufPrintFloat(waveguide_len);
        USART2_BufPrint(" мм | Температура: ");
        USART2_BufPrintFloat(current_temperature);
        USART2_BufPrint(" C\r\n");
        USART2_BufPrint("  Калибровка: ");
        USART2_BufPrintFloat(cal_low * 1000.0f);
        USART2_BufPrint(" .. ");
        USART2_BufPrintFloat(cal_high * 1000.0f);
        USART2_BufPrint(" мм\r\n");
        USART2_BufPrint("  Период: ");
        USART2_BufPrintInt(current_poll_period_ms / 1000);
        USART2_BufPrint(" сек | Магнитов: ");
        USART2_BufPrintInt(MAX_PULSE_PAIRS);
        USART2_BufPrint("\r\n");
        USART2_BufPrint("===========================================================\r\n");
    }
    else
    {
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

    /* Инициализация I2C2 для EEPROM */
    if (MX_I2C2_Init() == HAL_OK) {
        USART2_Print("[I2C2] Инициализирована (PB10=SCL, PB11=SDA)\r\n");
        /* LM75B не подключён - температура будет 0 */
        current_temperature = 0.0f;
    } else {
        USART2_Print("[I2C2] ОШИБКА инициализации!\r\n");
    }

    TIM3_InputCapture_Init();

    USART2_Print("[ИНИЦ] Калибровка АЦП...\r\n");
    if (HAL_ADCEx_Calibration_Start(&hadc1) == HAL_OK) {
        USART2_Print("[ИНИЦ] АЦП1 откалиброван успешно\r\n");
    } else {
        USART2_Print("[ИНИЦ] Калибровка АЦП1 НЕ УДАЛАСЬ!\r\n");
        v24_error = 1; v12_error = 1; v5_error = 1; vdda_error = 1;
    }
    if (HAL_ADCEx_Calibration_Start(&hadc2) == HAL_OK) {
        USART2_Print("[ИНИЦ] АЦП2 откалиброван успешно\r\n");
    } else {
        USART2_Print("[ИНИЦ] Калибровка АЦП2 НЕ УДАЛАСЬ!\r\n");
        v12_error = 1; v5_error = 1;
    }

    if (HAL_ADC_Init(&hadc1) != HAL_OK || HAL_ADC_Init(&hadc2) != HAL_OK) {
        USART2_Print("[ИНИЦ] Инициализация АЦП НЕ УДАЛАСЬ!\r\n");
        v24_error = 1; v12_error = 1; v5_error = 1; vdda_error = 1;
    }

    /* Инициализация Modbus + загрузка параметров из EEPROM AT24C64 */
    ModBus_Init();
    ModBus_UpdateFirmwareVersion(FIRMWARE_VERSION);

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

#ifdef TEST_MODE
    USART2_Print("=== РЕЖИМ ТЕСТА ЛИНИИ ФИКСАЦИИ АКТИВЕН ===\r\n");
#endif

    /* Стартовый лог (буферизованный) */
    USART2_BufInit();
    USART2_BufPrint("Адрес modbus: 1, скорость: 19200 бод.\r\n");
    USART2_BufPrint("[DBG] Период измерений:  ");
    USART2_BufPrintInt(current_poll_period_ms / 1000);
    USART2_BufPrint(" сек, Мёртвое окно:  ");
    USART2_BufPrintInt((uint32_t)(BLANKING_WINDOW_TICKS * TOF_TICK_US));
    USART2_BufPrint(" мкс, Магнитов:  ");
    USART2_BufPrintInt(MAX_PULSE_PAIRS);
    USART2_BufPrint(", Статистика:  ");
    USART2_BufPrintInt(STAT_HISTORY_SIZE);
    USART2_BufPrint(" значений (сброс после вывода)\r\n");
    USART2_BufPrint("[DBG] Порог ширины на CLIK:  ");
    USART2_BufPrintFloat(MIN_CLICK_WIDTH_US);
    USART2_BufPrint(" - ");
    USART2_BufPrintFloat(MAX_CLICK_WIDTH_US);
    USART2_BufPrint(" мкс\r\n");
    USART2_BufPrint("[EEPROM] Параметры загружены из AT24C64 (0x51)\r\n");
    USART2_BufFlush();

    uint32_t last_measure_time = 0;
    uint32_t last_debug_time = 0;
    uint32_t led_red_off_time = 0;
    uint8_t blue_led_state = 0;
    uint8_t red_led_state = 0;

    while (1)
    {
        if (HAL_GetTick() - last_debug_time >= 1000) {
            last_debug_time = HAL_GetTick();
            blue_led_state = !blue_led_state;
            HAL_GPIO_WritePin(GPIOB, LED_BLUE_PIN, blue_led_state ? LED_BLUE_ON : LED_BLUE_OFF);
        }

        if (red_led_state && (HAL_GetTick() - led_red_off_time >= LED_RED_ON_TIME_MS)) {
            HAL_GPIO_WritePin(GPIOB, LED_RED_PIN, LED_RED_OFF);
            red_led_state = 0;
        }

        if (HAL_GetTick() - last_measure_time >= current_poll_period_ms) {
            last_measure_time = HAL_GetTick();
            Update_Poll_Period_From_Modbus();

            HAL_GPIO_WritePin(GPIOB, LED_RED_PIN, LED_RED_OFF);
            red_led_state = 0;

            signal_captured = 0;
            uint32_t measurement;

#ifdef TEST_MODE
            measurement = measure_time_of_flight_test();
#else
            measurement = measure_time_of_flight();
#endif

            float tof_us = 0.0f;
            float position_mm = 0.0f;

            if (measurement > 0 && !tof_timeout) {
                Stat_AddValue(measurement);

                if (stat_ready) {
                    float avg_ticks = Stat_CalculateTrimmedAverage();
                    if (avg_ticks > 0.0f) {
                        tof_us = avg_ticks * TOF_TICK_US;
                        position_mm = (tof_us * 0.001f * SOUND_SPEED_MPS) / 2.0f;

                        /* Статистика — буферизованно */
                        USART2_BufInit();
                        USART2_BufPrint("[STAT] Среднее: ");
                        USART2_BufPrintFloat(avg_ticks);
                        USART2_BufPrint(" тиков (");
                        USART2_BufPrintFloat(tof_us);
                        USART2_BufPrint(" мкс) | Накоплено: ");
                        USART2_BufPrintInt(stat_count);
                        USART2_BufPrint("/");
                        USART2_BufPrintInt(STAT_HISTORY_SIZE);
                        USART2_BufPrint(" -> СБРОС\r\n");
                        USART2_BufFlush();

                        HAL_GPIO_WritePin(GPIOB, LED_RED_PIN, LED_RED_ON);
                        red_led_state = 1;
                        led_red_off_time = HAL_GetTick();

                        signal_captured = 1;
                        Stat_ClearHistory();
                    }
                }
            }

            if (stat_ready || signal_captured) {
                Process_Measurement_Results(tof_us, position_mm, signal_captured);
            }
        }

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
ОБРАБОТЧИК ПРЕРЫВАНИЯ TIM3 (Input Capture Channel 4)
★ МЁРТВОЕ ВРЕМЯ 60 мкс для игнорирования паразитного импульса
========================================================================== */


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

    /* CC4S = 01 (канал 4 как вход), IC4PSC = 00, IC4F = 0001 (фильтр) */
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

    /* Ширина импульса 10 мкс */
    for (volatile uint32_t i = 0; i < PULSE_DELAY_ITERATIONS*4; i++) __NOP();

    GPIOB->BRR = GPIO_PIN_5;
    for (volatile uint32_t i = 0; i < DELAY_AFTER_PULSE_ITER; i++) __NOP();

    HAL_GPIO_WritePin(SWITCH_PORT, SWITCH_PIN, GPIO_PIN_SET);
}

/* ==========================================================================
ФУНКЦИЯ: Измерение времени пролёта (ToF) - ОБЫЧНЫЙ РЕЖИМ
========================================================================== */
uint32_t measure_time_of_flight(void)
{
    /* Включаем прерывание ДО генерации импульса */
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
ФУНКЦИЯ: Измерение времени пролёта (ToF) - ТЕСТОВЫЙ РЕЖИМ
========================================================================== */
uint32_t measure_time_of_flight_test(void)
{
    generate_pulse_and_measure();

    uint32_t start_wait = HAL_GetTick();
    TIM3->SR = 0;
    TIM3->DIER |= TIM_DIER_CC4IE;

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
ФУНКЦИЯ: Чтение температуры (LM75B отключён)
========================================================================== */
void Read_Temperature(void)
{
    /* LM75B не подключён - возвращаем 0 */
    current_temperature = 0.0f;
}

/* ==========================================================================
ФУНКЦИЯ: Чтение всех напряжений через АЦП
========================================================================== */
void Read_All_Voltages(void)
{
    uint32_t adc_raw_vdda = 0;
    uint32_t adc_raw_24v = 0;
    uint32_t adc_raw_12v = 0;
    uint32_t adc_raw_5v = 0;

    ADC1->CR2 |= ADC_CR2_TSVREFE;
    HAL_Delay(10);
    adc_raw_vdda = Read_ADC_Average(&hadc1, ADC_CHANNEL_VREFINT, ADC_SAMPLETIME_239CYCLES_5, ADC_SAMPLES);
    ADC1->CR2 &= ~ADC_CR2_TSVREFE;
    HAL_Delay(1);

    if (adc_raw_vdda > 1000 && adc_raw_vdda < 2000 && *VREFINT_CAL_VALUE > 1000 && *VREFINT_CAL_VALUE < 2000) {
        current_vdda = 3.3f * (float)(*VREFINT_CAL_VALUE) / (float)adc_raw_vdda;
        vdda_error = 0;
    } else {
        current_vdda = 3.3f;
        vdda_error = 1;
    }

    adc_raw_24v = Read_ADC_Average(&hadc1, ADC_CHANNEL_0, ADC_SAMPLETIME_239CYCLES_5, ADC_SAMPLES);
    if (adc_raw_24v > 100 && adc_raw_24v < 4000) {
        float adc_voltage = (float)adc_raw_24v * current_vdda / 4095.0f;
        current_24v = adc_voltage * DIV_24V_FACTOR;
        v24_error = 0;
    } else {
        current_24v = 24.0f;
        v24_error = 1;
    }

    adc_raw_12v = Read_ADC_Average(&hadc2, ADC_CHANNEL_1, ADC_SAMPLETIME_239CYCLES_5, ADC_SAMPLES);
    if (adc_raw_12v > 100 && adc_raw_12v < 4000) {
        float adc_voltage = (float)adc_raw_12v * current_vdda / 4095.0f;
        current_12v = adc_voltage * DIV_12V_FACTOR;
        v12_error = 0;
    } else {
        current_12v = 12.0f;
        v12_error = 1;
    }

    adc_raw_5v = Read_ADC_Average(&hadc2, ADC_CHANNEL_5, ADC_SAMPLETIME_239CYCLES_5, ADC_SAMPLES);
    if (adc_raw_5v > 100 && adc_raw_5v < 4000) {
        float adc_voltage = (float)adc_raw_5v * current_vdda / 4095.0f;
        current_5v = adc_voltage * DIV_5V_FACTOR;
        v5_error = 0;
    } else {
        current_5v = 5.0f;
        v5_error = 1;
    }

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
        if (v > 100 && v < 4000) {
            sum += v;
            valid++;
        }
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

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
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

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SWITCH_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SWITCH_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(SWITCH_PORT, SWITCH_PIN, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = LED_RED_PIN | LED_BLUE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, LED_RED_PIN, LED_RED_OFF);
    HAL_GPIO_WritePin(GPIOB, LED_BLUE_PIN, LED_BLUE_OFF);

    GPIO_InitStruct.Pin = RS485_CTRL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(RS485_CTRL_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(RS485_CTRL_PORT, RS485_CTRL_PIN, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* WP пин для AT24C64 (PB8) - настраивается в AT24C64_Init() */
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
