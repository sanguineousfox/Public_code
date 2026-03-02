/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "stm32f1xx_it.h"
#include "utils.h"
#include "i2c_config.h"
#include "lm75b.h"
#include "modbus.h"
#include <stdint.h>
#include <string.h>

/* Private define ------------------------------------------------------------*/
/* === ТАЙМЕР === */
#define TIMER_CLOCK_HZ      72000000.0f           /* Частота таймера 72 МГц */
#define TOF_TICK_US         0.0972f               /* Длительность 1 тика таймера в мкс (72МГц/7) */

/* === АЦП === */
#define VREFINT_CAL_ADDR    ((uint16_t*)0x1FFFF7BA) /* Адрес калибровочного значения VREFINT */
#define VREFINT_CAL_VALUE   (*VREFINT_CAL_ADDR)     /* Калибровочное значение VREFINT */
#define ADC_SAMPLES         16                      /* Количество отсчётов АЦП для усреднения */

/* === ТАЙМАУТЫ === */
#define MEAS_TIMEOUT_MS     50                      /* Таймаут ожидания сигнала в мс */

/* === ДЕЛИТЕЛИ НАПРЯЖЕНИЯ === */
#define DIV_24V_FACTOR      9.2f                    /* Коэффициент делителя 24В */
#define DIV_12V_FACTOR      4.6f                    /* Коэффициент делителя 12В */
#define DIV_5V_FACTOR       2.0f                    /* Коэффициент делителя 5В */

/* === ПЕРИОД ИЗМЕРЕНИЙ === */
#define PULSE_PERIOD_MS     10000                   /* Период между измерениями в мс (10 сек) */

/* === СКОРОСТЬ ЗВУКА === */
#define SOUND_SPEED_MPS     2800.0f                 /* Скорость звука в волноводе м/с */

/* === СВЕТОДИОДЫ === */
#define LED_RED_PIN         GPIO_PIN_13             /* Пин красного светодиода */
#define LED_BLUE_PIN        GPIO_PIN_12             /* Пин синего светодиода */
#define LED_RED_ON_TIME_MS  1000                    /* Время горения красного светодиода в мс */

/* === ИМПУЛЬС === */
#define PULSE_DELAY_ITERATIONS 45                   /* Длительность импульса в тактах */
#define SWITCH_PIN          GPIO_PIN_7              /* Пин управления ключом */
#define SWITCH_PORT         GPIOB                   /* Порт ключа */
#define DELAY_AFTER_PULSE_ITER  97                  /* Задержка после импульса в тактах */
#define SWITCH_HOLD_ITERATIONS 12000                /* Время удержания ключа в тактах */

/* === ВОЛНОВОД === */
#define WAVEGUIDE_LENGTH_MM 6000                    /* Длина волновода в мм */

/* === MODBUS === */
#define MODBUS_SILENCE_TIME_MS 4                    /* Пауза тишины Modbus в мс */
#define RS485_CTRL_PIN      GPIO_PIN_0              /* Пин управления RS485 */
#define RS485_CTRL_PORT     GPIOB                   /* Порт управления RS485 */

/* === ВЕРСИЯ === */
#define FIRMWARE_VERSION    100                     /* Версия прошивки */

/* === СОСТОЯНИЯ === */
#define LED_RED_ON          GPIO_PIN_SET
#define LED_RED_OFF         GPIO_PIN_RESET
#define LED_BLUE_ON         GPIO_PIN_SET
#define LED_BLUE_OFF        GPIO_PIN_RESET

/* === МЁРТВОЕ ОКНО === */
/* 50 мкс = ~515 тиков @ 10.28 MHz (72MHz / 7) */
/* Увеличено с 45 до 50 мкс чтобы отсечь 11 мкс помехи */
#define BLANKING_WINDOW_TICKS   515                 /* Мёртвое окно в тиках таймера (50 мкс) */

/* === УСРЕДНЕНИЕ === */
/* Количество измерений для усреднения (помогает отсечь выбросы) */
#define MEASUREMENT_AVG_COUNT   5                   /* Увеличено с 3 до 5 для стабильности */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

volatile uint32_t tof_capture_value = 0;
volatile uint8_t tof_measurement_done = 0;
volatile uint8_t tof_timeout = 0;
volatile uint8_t signal_captured = 0;

/* Текущие значения */
static float current_vdda = 3.3f;
static float current_24v = 24.0f;
static float current_12v = 12.0f;
static float current_5v = 5.0f;
static float current_temperature = 0.0f;

/* Предыдущие значения для отслеживания изменений */
static float prev_vdda = 0.0f;
static float prev_24v = 0.0f;
static float prev_12v = 0.0f;
static float prev_5v = 0.0f;

static uint8_t lm75b_initialized = 0;
static uint8_t v24_error = 0;
static uint8_t v12_error = 0;
static uint8_t v5_error = 0;
static uint8_t vdda_error = 0;
static uint8_t temp_error = 0;

volatile uint8_t modbus_tx_active = 0;

#define RS485_SET_TRANSMIT() HAL_GPIO_WritePin(RS485_CTRL_PORT, RS485_CTRL_PIN, GPIO_PIN_SET)
#define RS485_SET_RECEIVE()  HAL_GPIO_WritePin(RS485_CTRL_PORT, RS485_CTRL_PIN, GPIO_PIN_RESET)

/* USER CODE BEGIN 0 */
void ModBus_TransmitFrame(uint8_t *frame, uint16_t len)
{
    if (len == 0) return;

    /* Ждем пока линия не освободится (защита от коллизий) */
    uint32_t start_wait = HAL_GetTick();
    while (modbus_tx_active) {
        if (HAL_GetTick() - start_wait > 100) break;
    }

    RS485_SET_TRANSMIT();
    modbus_tx_active = 1;

    /* Небольшая задержка перед передачей (направление шины) */
    for (volatile int i = 0; i < 150; i++) __NOP();

    HAL_UART_Transmit(&huart1, frame, len, 100);

    /* Ждем окончания передачи (TC flag) */
    uint32_t timeout_start = HAL_GetTick();
    while ((USART1->SR & USART_SR_TC) == 0) {
        if (HAL_GetTick() - timeout_start > 50) break;
    }

    /* Пауза тишины Modbus (3.5 символа) */
    for (volatile int i = 0; i < 800; i++) __NOP();

    RS485_SET_RECEIVE();
    modbus_tx_active = 0;
}

/* Вспомогательная функция для вывода целого числа */
static void USART2_PrintInt(int32_t val)
{
    char buf[12];
    int8_t i = 0;
    int8_t len = 0;

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

/* Вспомогательная функция для вывода float (2 знака) */
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

/* Проверка и вывод изменения напряжения */
static void Check_Voltage_Change(const char* name, float new_val, float old_val, float* store_val)
{
    float diff = new_val - old_val;
    if (diff < 0) diff = -diff;

    /* Если изменилось более чем на 0.1В или это первое измерение */
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
/* USER CODE END 0 */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
void Error_Handler(void);
void TIM3_InputCapture_Init(void);
void generate_pulse_and_measure(void);
uint32_t measure_time_of_flight(void);
void Read_All_Voltages(void);
void Read_Temperature(void);
uint32_t Read_ADC_Single(ADC_HandleTypeDef* hadc, uint32_t channel, uint32_t sampling_time);
uint32_t Read_ADC_Average(ADC_HandleTypeDef* hadc, uint32_t channel, uint32_t sampling_time, uint8_t samples);

/* USER CODE BEGIN 1 */
void Process_Measurement_Results(float tof_us, float position_mm, uint8_t signal_captured)
{
    ModBus_UpdateMeasurements(position_mm, current_temperature, WAVEGUIDE_LENGTH_MM);
    ModBus_UpdateVoltages(current_24v, current_12v, current_5v, current_vdda);
    ModBus_UpdateFirmwareVersion(FIRMWARE_VERSION);

    if (signal_captured)
    {
        USART2_Print("[ИЗМ] ToF: ");
        USART2_PrintFloat(tof_us);
        USART2_Print(" мкс, Уровень: ");
        USART2_PrintFloat(position_mm);
        USART2_Print(" мм, Температура: ");
        USART2_PrintFloat(current_temperature);
        USART2_Print(" C\r\n");
    }
    else
    {
        USART2_Print("[ИЗМ] Сигнал не захвачен (таймаут)\r\n");
    }
}
/* USER CODE END 1 */

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();

    /* Инициализация I2C2 и датчиков температуры */
    if (MX_I2C2_Init() == HAL_OK) {
        if (LM75B_Init(LM75B_DEFAULT_ADDRESS) == HAL_OK) {
            lm75b_initialized = 1;
            USART2_Print("LM75B: обнаружен 1 датчик\r\n");
        } else {
            USART2_Print("LM75B: НЕ ОБНАРУЖЕН датчик температуры\r\n");
            temp_error = 1;
        }
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

    /* Инициализация Modbus */
    ModBus_Init();
    ModBus_UpdateFirmwareVersion(FIRMWARE_VERSION);

    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
    __enable_irq();

    /* Первое измерение */
    Read_All_Voltages();
    Read_Temperature();
    ModBus_UpdateVoltages(current_vdda, current_24v, current_12v, current_5v);

    USART2_Print("ПМП-201Е запущен. Адрес модбас: 1, скорость: 19200 бод.\r\n");
    USART2_Print("[DBG] Период измерений: ");
    USART2_PrintInt(PULSE_PERIOD_MS / 1000);
    USART2_Print(" сек, Мёртвое окно: ");
    USART2_PrintInt((uint32_t)(BLANKING_WINDOW_TICKS * TOF_TICK_US));
    USART2_Print(" мкс\r\n");

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

        /* Измерение уровня каждые 10 сек */
        if (HAL_GetTick() - last_measure_time >= PULSE_PERIOD_MS) {
            last_measure_time = HAL_GetTick();

            HAL_GPIO_WritePin(GPIOB, LED_RED_PIN, LED_RED_OFF);
            red_led_state = 0;
            signal_captured = 0;

            /* === УСРЕДНЕНИЕ ИЗМЕРЕНИЙ === */
            uint32_t total_ticks = 0;
            uint8_t valid_count = 0;
            uint32_t measurements[MEASUREMENT_AVG_COUNT];

            for (uint8_t i = 0; i < MEASUREMENT_AVG_COUNT; i++) {
                measurements[i] = measure_time_of_flight();
                if (measurements[i] > 0 && !tof_timeout) {
                    total_ticks += measurements[i];
                    valid_count++;
                }
                if (i < MEASUREMENT_AVG_COUNT - 1) {
                    HAL_Delay(10);
                }
            }

            float tof_us = 0.0f;
            float position_mm = 0.0f;

            if (valid_count > 0) {
                /* === МЕДИАНА ДЛЯ ОТСЕЧКИ ВЫБРОСОВ === */
                /* Сортируем массив для нахождения медианы */
                for (uint8_t i = 0; i < valid_count - 1; i++) {
                    for (uint8_t j = 0; j < valid_count - i - 1; j++) {
                        if (measurements[j] > measurements[j + 1]) {
                            uint32_t temp = measurements[j];
                            measurements[j] = measurements[j + 1];
                            measurements[j + 1] = temp;
                        }
                    }
                }
                /* Берём медиану (средний элемент) */
                uint32_t median_ticks = measurements[valid_count / 2];

                tof_us = median_ticks * TOF_TICK_US;
                position_mm = (tof_us * 0.001f * SOUND_SPEED_MPS) / 2.0f;

                USART2_Print("[AVG] ");
                USART2_PrintInt(valid_count);
                USART2_Print("/");
                USART2_PrintInt(MEASUREMENT_AVG_COUNT);
                USART2_Print(" valid, Median: ");
                USART2_PrintInt(median_ticks);
                USART2_Print(" тиков (");
                USART2_PrintFloat(tof_us);
                USART2_Print(" мкс)\r\n");

                HAL_GPIO_WritePin(GPIOB, LED_RED_PIN, LED_RED_ON);
                red_led_state = 1;
                led_red_off_time = HAL_GetTick();
                signal_captured = 1;
            }

            Process_Measurement_Results(tof_us, position_mm, signal_captured);
        }

        /* Обработка Modbus */
        ModBus_Process();

        /* Защита от зависания передачи */
        if (modbus_tx_active && (HAL_GetTick() - last_measure_time > 100)) {
            RS485_SET_RECEIVE();
            modbus_tx_active = 0;
        }

        HAL_Delay(1);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        ModBus_RxCallback(huart);
    }
}

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

    /* Цифровой фильтр IC4F=0101 (8 событий @ fDTS/8 ≈ 0.9 мкс) */
    TIM3->CCMR2 = (0x1 << 12) | (0x5 << 8);

    TIM3->CCER = TIM_CCER_CC4E | TIM_CCER_CC4P;
    /* ЗАХВАТ ОТКЛЮЧЕН до выхода из мёртвого окна! */
    TIM3->DIER = 0;
    TIM3->SR = 0;
    TIM3->CR1 = TIM_CR1_CEN;  /* Таймер запущен сразу после инициализации */
}

void generate_pulse_and_measure(void)
{
    tof_measurement_done = 0;
    tof_timeout = 0;
    tof_capture_value = 0;

    /* === КРИТИЧНО: ПОЛНЫЙ СБРОС ПЕРЕД ИЗМЕРЕНИЕМ === */
    TIM3->DIER &= ~TIM_DIER_CC4IE;  /* Отключаем прерывание ПЕРЕД сбросом */
    TIM3->SR = 0;                    /* Сбрасываем все флаги */
    TIM3->CNT = 0;                   /* Сбрасываем счётчик */
    __DSB();                         /* Барьер памяти для гарантии порядка операций */

    /* Убеждаемся что таймер запущен */
    TIM3->CR1 |= TIM_CR1_CEN;
    __NOP();

    HAL_GPIO_WritePin(SWITCH_PORT, SWITCH_PIN, GPIO_PIN_RESET);

    GPIOB->BSRR = GPIO_PIN_5;
    for (volatile uint32_t i = 0; i < PULSE_DELAY_ITERATIONS; i++) __NOP();
    GPIOB->BRR = GPIO_PIN_5;

    for (volatile uint32_t i = 0; i < DELAY_AFTER_PULSE_ITER; i++) __NOP();

    HAL_GPIO_WritePin(SWITCH_PORT, SWITCH_PIN, GPIO_PIN_SET);
}

uint32_t measure_time_of_flight(void)
{
    generate_pulse_and_measure();

    uint32_t start_wait = HAL_GetTick();

    /* === ЖДЁМ ВЫХОДА ИЗ МЁРТВОГО ОКНА === */
    volatile uint32_t cnt_value;
    uint32_t blank_timeout = 0;

    do {
        cnt_value = TIM3->CNT;
        if ((HAL_GetTick() - start_wait) >= 2) {  /* 2 мс достаточно для 50 мкс */
            blank_timeout = 1;
            break;
        }
    } while (cnt_value < BLANKING_WINDOW_TICKS);

    /* === МЁРТВОЕ ОКНО ПРОЙДЕНО — ВКЛЮЧАЕМ ЗАХВАТ === */
    TIM3->SR = 0;                  /* Ещё раз сбрасываем флаги перед включением */
    TIM3->DIER |= TIM_DIER_CC4IE;  /* Включаем прерывание ТОЛЬКО после мёртвого окна */

    /* === ЖДЁМ ЗАХВАТА === */
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

    return tof_capture_value;
}

void Read_Temperature(void)
{
    if (!lm75b_initialized) {
        current_temperature = -127.0f;
        temp_error = 1;
        return;
    }

    uint16_t raw_temp = 0;
    if (LM75B_ReadRawTemperature(LM75B_DEFAULT_ADDRESS, &raw_temp) != HAL_OK) {
        current_temperature = -127.0f;
        temp_error = 1;
        USART2_Print("[I2C] Ошибка чтения температуры.\r\n");
    } else {
        temp_error = 0;
        current_temperature = (float)((int16_t)raw_temp) / 256.0f;
    }
}

void Read_All_Voltages(void)
{
    uint32_t adc_raw_vdda = 0;
    uint32_t adc_raw_24v = 0;
    uint32_t adc_raw_12v = 0;
    uint32_t adc_raw_5v = 0;

    /* --- VDDA --- */
    ADC1->CR2 |= ADC_CR2_TSVREFE;
    HAL_Delay(10);
    adc_raw_vdda = Read_ADC_Average(&hadc1, ADC_CHANNEL_VREFINT, ADC_SAMPLETIME_239CYCLES_5, ADC_SAMPLES);
    ADC1->CR2 &= ~ADC_CR2_TSVREFE;
    HAL_Delay(1);

    if (adc_raw_vdda > 1000 && adc_raw_vdda < 2000 && VREFINT_CAL_VALUE > 1000 && VREFINT_CAL_VALUE < 2000) {
        current_vdda = 3.3f * (float)VREFINT_CAL_VALUE / (float)adc_raw_vdda;
        vdda_error = 0;
    } else {
        current_vdda = 3.3f;
        vdda_error = 1;
    }

    /* --- 24В --- */
    adc_raw_24v = Read_ADC_Average(&hadc1, ADC_CHANNEL_0, ADC_SAMPLETIME_239CYCLES_5, ADC_SAMPLES);
    if (adc_raw_24v > 100 && adc_raw_24v < 4000) {
        float adc_voltage = (float)adc_raw_24v * current_vdda / 4095.0f;
        current_24v = adc_voltage * DIV_24V_FACTOR;
        v24_error = 0;
    } else {
        current_24v = 24.0f;
        v24_error = 1;
    }

    /* --- 12В --- */
    adc_raw_12v = Read_ADC_Average(&hadc2, ADC_CHANNEL_1, ADC_SAMPLETIME_239CYCLES_5, ADC_SAMPLES);
    if (adc_raw_12v > 100 && adc_raw_12v < 4000) {
        float adc_voltage = (float)adc_raw_12v * current_vdda / 4095.0f;
        current_12v = adc_voltage * DIV_12V_FACTOR;
        v12_error = 0;
    } else {
        current_12v = 12.0f;
        v12_error = 1;
    }

    /* --- 5В --- */
    adc_raw_5v = Read_ADC_Average(&hadc2, ADC_CHANNEL_5, ADC_SAMPLETIME_239CYCLES_5, ADC_SAMPLES);
    if (adc_raw_5v > 100 && adc_raw_5v < 4000) {
        float adc_voltage = (float)adc_raw_5v * current_vdda / 4095.0f;
        current_5v = adc_voltage * DIV_5V_FACTOR;
        v5_error = 0;
    } else {
        current_5v = 5.0f;
        v5_error = 1;
    }

    /* === ВЫВОД ИЗМЕНЕНИЙ НАПРЯЖЕНИЙ === */
    Check_Voltage_Change("VDDA", current_vdda, prev_vdda, &prev_vdda);
    Check_Voltage_Change("+24V", current_24v, prev_24v, &prev_24v);
    Check_Voltage_Change("+12V", current_12v, prev_12v, &prev_12v);
    Check_Voltage_Change("+5V ", current_5v, prev_5v, &prev_5v);
}

uint32_t Read_ADC_Single(ADC_HandleTypeDef* hadc, uint32_t channel, uint32_t sampling_time)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = sampling_time;
    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) {
        return 0;
    }
    HAL_ADC_Start(hadc);
    if (HAL_ADC_PollForConversion(hadc, 10) != HAL_OK) {
        HAL_ADC_Stop(hadc);
        return 0;
    }
    uint32_t adc_value = HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);
    return adc_value;
}

uint32_t Read_ADC_Average(ADC_HandleTypeDef* hadc, uint32_t channel, uint32_t sampling_time, uint8_t samples)
{
    uint32_t sum = 0;
    uint8_t valid_samples = 0;
    for (uint8_t i = 0; i < samples; i++) {
        uint32_t value = Read_ADC_Single(hadc, channel, sampling_time);
        if (value > 100 && value < 4000) {
            sum += value;
            valid_samples++;
        }
        HAL_Delay(1);
    }
    if (valid_samples == 0) {
        return 0;
    }
    return sum / valid_samples;
}

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

static void MX_GPIO_Init(void)
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
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(RS485_CTRL_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(RS485_CTRL_PORT, RS485_CTRL_PIN, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void MX_USART1_UART_Init(void)
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

static void MX_USART2_UART_Init(void)
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

static void MX_ADC1_Init(void)
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

static void MX_ADC2_Init(void)
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
