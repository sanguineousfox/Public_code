/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
#include "utils.h"
#include <stdint.h>
#include <string.h>
#include <math.h>

/* Private define ------------------------------------------------------------*/
#define TIMER_CLOCK_HZ  72000000.0f  // 72 МГц после PLL
#define VREFINT_CAL    (*((uint16_t*)0x1FFFF7BA))  // Factory calibration @ 3.3V
#define VREFINT_VOLTAGE 1.23f  // Reference voltage of VREFINT
#define ADC_SAMPLES     16      // Количество выборок для усреднения

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;  // Для ModBus (USART1)
UART_HandleTypeDef huart2;  // Для отладки (USART2)
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
TIM_HandleTypeDef htim4;    // Для генерации импульсов

// Глобальные переменные для захвата частоты
volatile uint32_t capture_times[3] = {0};
volatile uint8_t capture_index = 0;
volatile uint8_t measurement_done = 0;
volatile uint16_t last_capture = 0;  // Для обработки переполнения

// Переменные для хранения измерений
static float current_vdda = 3.3f;
static float current_24v = 24.0f;
static float current_12v = 12.0f;
static float current_5v = 5.0f;

// Флаги инициализации ADC
static uint8_t adc1_initialized = 0;
static uint8_t adc2_initialized = 0;

// Буфер для преобразования float в строку
static char float_buffer[16];

// Переменные для генерации импульсов
volatile uint8_t pulse_enabled = 1;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM4_Init(void);
void Error_Handler(void);

void TIM3_InputCapture_Init(void);
void start_capture(void);
void print_results(void);
void reset_capture(void);
void Read_All_Voltages(void);
uint32_t Read_ADC_Single(ADC_HandleTypeDef* hadc, uint32_t channel, uint32_t sampling_time);
uint32_t Read_ADC_Average(ADC_HandleTypeDef* hadc, uint32_t channel, uint32_t sampling_time, uint8_t samples);
void TIM4_PWM_Init_Simple(void);
void Start_Pulse_Generator(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* MCU Configuration--------------------------------------------------------*/
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_TIM4_Init();

    /* Калибровка ADC */
    USART2_Print("[INIT] Calibrating ADC1...\r\n");
    if (HAL_ADCEx_Calibration_Start(&hadc1) == HAL_OK) {
        adc1_initialized = 1;
        USART2_Print("[INIT] ADC1 calibrated OK\r\n");
    } else {
        USART2_Print("[INIT] ADC1 calibration FAILED!\r\n");
    }

    USART2_Print("[INIT] Calibrating ADC2...\r\n");
    if (HAL_ADCEx_Calibration_Start(&hadc2) == HAL_OK) {
        adc2_initialized = 1;
        USART2_Print("[INIT] ADC2 calibrated OK\r\n");
    } else {
        USART2_Print("[INIT] ADC2 calibration FAILED!\r\n");
    }

    /* Инициализация ModBus */
    ModBus_Init();

    /* Задержка для стабилизации */
    HAL_Delay(1000);

    /* Инициализация захвата частоты */
    TIM3_InputCapture_Init();

    /* Инициализация и запуск генератора импульсов (ПРОСТОЙ ВАРИАНТ) */
    TIM4_PWM_Init_Simple();
    Start_Pulse_Generator();

    /* Настройка приоритетов прерываний */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);

    /* Включаем глобальные прерывания */
    __enable_irq();

    /* Вывод информации о системе */
    USART2_Print("\r\n========================================\r\n");
    USART2_Print("       STM32 Frequency Meter System       \r\n");
    USART2_Print("========================================\r\n");
    USART2_Print("System initialized\r\n");
    USART2_Print("USART2: Debug console (115200)\r\n");
    USART2_Print("USART1: ModBus RTU (9600, 8N1)\r\n");
    USART2_Print("Device address: 1\r\n");
    USART2_Print("ADC samples: ");
    USART2_PrintNum(ADC_SAMPLES);
    USART2_Print("\r\nPulse generator: PB5, 10 us @ 10 Hz\r\n");
    USART2_Print("========================================\r\n\r\n");

    /* Тестовое измерение напряжений при запуске */
    USART2_Print("[INIT] Testing ADC measurements...\r\n");
    Read_All_Voltages();

    USART2_Print("[INIT] VDDA: ");
    float_to_str(current_vdda, float_buffer, 3);
    USART2_Print(float_buffer);
    USART2_Print(" V\r\n");

    USART2_Print("[INIT] 24V:  ");
    float_to_str(current_24v, float_buffer, 3);
    USART2_Print(float_buffer);
    USART2_Print(" V\r\n");

    USART2_Print("[INIT] 12V:  ");
    float_to_str(current_12v, float_buffer, 3);
    USART2_Print(float_buffer);
    USART2_Print(" V\r\n");

    USART2_Print("[INIT] 5V:   ");
    float_to_str(current_5v, float_buffer, 3);
    USART2_Print(float_buffer);
    USART2_Print(" V\r\n");

    USART2_Print("[INIT] ADC test completed\r\n\r\n");

    /* Основные переменные цикла */
    uint32_t last_measure_time = 0;
    uint32_t last_debug_time = 0;
    uint8_t led_state = 0;
    /* Infinite loop */
    while (1)
    {
    	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    	//HAL_Delay(0.1);
    	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
    	HAL_Delay(100);
        /* Мигание светодиодом каждые 500 мс */
        if (HAL_GetTick() - last_debug_time >= 500)
        {
            last_debug_time = HAL_GetTick();
            led_state = !led_state;
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, led_state ? GPIO_PIN_RESET : GPIO_PIN_SET);
        }

        /* Чтение напряжений и измерение частоты каждую секунду */
        if (HAL_GetTick() - last_measure_time >= 1000)
        {
            last_measure_time = HAL_GetTick();

            /* Шаг 1: Измерение напряжений */
            Read_All_Voltages();

            /* Шаг 2: Обновление данных в ModBus регистрах */
            ModBus_UpdateVoltages(current_vdda, current_24v, current_12v, current_5v);

            /* Шаг 3: Вывод напряжений на отладку */
            USART2_Print("\r\n--- VOLTAGES ---\r\n");

            USART2_Print("VDDA = ");
            float_to_str(current_vdda, float_buffer, 3);
            USART2_Print(float_buffer);
            USART2_Print(" V\r\n");

            USART2_Print("24V  = ");
            float_to_str(current_24v, float_buffer, 3);
            USART2_Print(float_buffer);
            USART2_Print(" V\r\n");

            USART2_Print("12V  = ");
            float_to_str(current_12v, float_buffer, 3);
            USART2_Print(float_buffer);
            USART2_Print(" V\r\n");

            USART2_Print("5V   = ");
            float_to_str(current_5v, float_buffer, 3);
            USART2_Print(float_buffer);
            USART2_Print(" V\r\n");

            /* Шаг 4: Измерение частоты */
            reset_capture();
            start_capture();

            /* Ожидание завершения измерения частоты */
            uint32_t start_wait = HAL_GetTick();
            while (!measurement_done && ((HAL_GetTick() - start_wait) < 1000)) {
                ModBus_Process();
                __NOP();
            }

            /* Обработка результатов измерения частоты */
            if (measurement_done && capture_index >= 3) {
                /* Расчет периодов */
                uint32_t period1_ticks = capture_times[1] - capture_times[0];
                uint32_t period2_ticks = capture_times[2] - capture_times[1];

                float tick_duration_us = 1000000.0f / (TIMER_CLOCK_HZ / 7.0f);
                float period1_us = period1_ticks * tick_duration_us;
                float period2_us = period2_ticks * tick_duration_us;
                float avg_freq_mhz = 0.0f;
                uint8_t meas_status = 0;

                /* Проверяем минимальную паузу 1.5 мкс */
                if (period1_us >= 1.5f && period2_us >= 1.5f) {
                    float avg_period_us = (period1_us + period2_us) / 2.0f;
                    if (avg_period_us > 0.0f) {
                        avg_freq_mhz = 1.0f / (avg_period_us * 0.000001f);
                        meas_status = 1;
                    }
                }

                /* Обновляем ModBus регистры */
                ModBus_UpdateMeasurements(period1_us, period2_us, avg_freq_mhz, meas_status);

                /* Вывод результатов на отладку */
                print_results();
            } else {
                /* Ошибка измерения частоты */
                USART2_Print("\r\nERROR: Measurement timeout, captures=");
                USART2_PrintNum(capture_index);
                USART2_Print("\r\n");

                /* Обнуление регистров измерений в ModBus */
                ModBus_UpdateMeasurements(0, 0, 0, 0);
            }
        }

        /* Обработка ModBus */
        ModBus_Process();

        /* Минимальная задержка */
        HAL_Delay(1);
    }
}

/**
  * @brief  Callback завершения приема UART
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        ModBus_RxCallback(huart);
    }
}

/**
  * @brief  Инициализация TIM3 для захвата частоты
  */
void TIM3_InputCapture_Init(void)
{
    /* Включаем тактирование */
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    /* Частичное перемаппирование TIM3 для PB1 (TIM3_CH4) */
    __HAL_AFIO_REMAP_TIM3_PARTIAL();

    /* Настройка GPIO для PB1 (вход захвата) */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Сброс таймера */
    TIM3->CR1 = 0;
    TIM3->CR2 = 0;
    TIM3->PSC = 6;          /* Предделитель: 72MHz / (6+1) = 10.2857MHz */
    TIM3->ARR = 0xFFFF;     /* Максимальный период */
    TIM3->CNT = 0;

    /* Применяем настройки (генерируем событие обновления) */
    TIM3->EGR = TIM_EGR_UG;

    /* Настройка канала 4 (PB1) для захвата по фронту */
    TIM3->CCMR2 = TIM_CCMR2_CC4S_0; /* CC4S = 01 (вход TI4) */
    TIM3->CCMR2 &= ~(0xF << 12);    /* Очистка битов фильтра */
    TIM3->CCMR2 &= ~(0x3 << 10);    /* Без предделителя */

    /* Включаем захват по фронту (без инверсии) */
    TIM3->CCER = TIM_CCER_CC4E;

    /* Включаем прерывание по захвату */
    TIM3->DIER = TIM_DIER_CC4IE;

    /* Очищаем все флаги прерываний */
    TIM3->SR = 0;
}

/**
  * @brief  Начало захвата частоты
  */
void start_capture(void)
{
    /* Сброс переменных */
    capture_index = 0;
    measurement_done = 0;
    last_capture = 0;

    /* Очищаем флаги прерываний */
    TIM3->SR = 0;

    /* Сбрасываем счетчик */
    TIM3->CNT = 0;

    /* Включаем прерывание захвата */
    TIM3->DIER |= TIM_DIER_CC4IE;

    /* Запускаем таймер */
    TIM3->CR1 |= TIM_CR1_CEN;
}

/**
  * @brief  Сброс состояния захвата
  */
void reset_capture(void)
{
    capture_index = 0;
    measurement_done = 0;
    last_capture = 0;
    for (int i = 0; i < 3; i++) capture_times[i] = 0;
}

/**
  * @brief  Вывод результатов измерения частоты
  */
//void print_results(void)
//{
//    /* Периоды в тиках */
//    uint32_t period1_ticks = capture_times[1] - capture_times[0];
//    uint32_t period2_ticks = capture_times[2] - capture_times[1];
//
//    /* Разрешение таймера: 1/(72MHz/(PSC+1)) = 1/10.2857MHz = 0.09722 мкс */
//    float tick_duration_us = 1000000.0f / (TIMER_CLOCK_HZ / 7.0f);
//    float period1_us = period1_ticks * tick_duration_us;
//    float period2_us = period2_ticks * tick_duration_us;
//
//    USART2_Print("\r\n--- FREQUENCY MEASUREMENT ---\r\n");
//
//    /* Отладочный вывод тиков */
//    USART2_Print("Period1: ");
//    USART2_PrintNum(period1_ticks);
//    USART2_Print(" ticks, Period2: ");
//    USART2_PrintNum(period2_ticks);
//    USART2_Print(" ticks\r\n");
//
//    /* Проверка на минимальную паузу 1.5 мкс */
//    USART2_Print("Period 1-2: ");
//    if (period1_us < 1.5f) {
//        USART2_Print("<1.5 us (");
//        float_to_str(period1_us, float_buffer, 3);
//        USART2_Print(float_buffer);
//        USART2_Print(" us)\r\n");
//    } else {
//        float_to_str(period1_us, float_buffer, 3);
//        USART2_Print(float_buffer);
//        USART2_Print(" us\r\n");
//    }
//
//    USART2_Print("Period 2-3: ");
//    if (period2_us < 1.5f) {
//        USART2_Print("<1.5 us (");
//        float_to_str(period2_us, float_buffer, 3);
//        USART2_Print(float_buffer);
//        USART2_Print(" us)\r\n");
//    } else {
//        float_to_str(period2_us, float_buffer, 3);
//        USART2_Print(float_buffer);
//        USART2_Print(" us\r\n");
//    }
//
//    /* Средний период (только если обе паузы >= 1.5 мкс) */
//    if (period1_us >= 1.5f && period2_us >= 1.5f) {
//        float avg_period_us = (period1_us + period2_us) / 2.0f;
//        if (avg_period_us > 0.0f) {
//        	float avg_freq_hz = 1.0f / (avg_period_us * 0.000001f);   // Частота в Гц
//        	float avg_freq_MHz = avg_freq_hz / 1000000.0f;            // Перевод в МГц
//            USART2_Print("Avg frequency: ");
//            float_to_str(avg_freq_MHz, float_buffer, 3);
//            USART2_Print(float_buffer);
//            USART2_Print(" MHz\r\n");
//        }
//    } else {
//        USART2_Print("Avg frequency: NOT CALCULATED (period < 1.5 us)\r\n");
//    }
//}

/**
  * @brief  ПРОСТАЯ инициализация TIM4 для генерации импульсов на PB5
  */
void TIM4_PWM_Init_Simple(void)
{
    /* Включаем тактирование TIM4 */
    __HAL_RCC_TIM4_CLK_ENABLE();

    /* Настройка таймера вручную */
    TIM4->CR1 = 0;
    TIM4->CR2 = 0;
    TIM4->PSC = 72 - 1;        // Предделитель: 72MHz / 72 = 1MHz (1 тик = 1 мкс)
    TIM4->ARR = 100000 - 1;    // Период: 100000 мкс = 100 мс (10 Гц)
    TIM4->CNT = 0;

    /* Настройка канала 2 (PB7) в режиме PWM1 */
    // Внимание: на STM32F103C8T6 PB5 = TIM4_CH1, а не TIM4_CH2!
    // TIM4_CH1 на PB6, TIM4_CH2 на PB7, TIM4_CH3 на PB8, TIM4_CH4 на PB9
    // PB5 - это TIM3_CH2 или TIM2_CH1

    /* Давайте используем PB6 (TIM4_CH1) или PB7 (TIM4_CH2) */
    // Я выберу PB6 (TIM4_CH1) для простоты
    // Но сначала проверим, что в MX_GPIO_Init настроен PB5 как выход

    /* Настраиваем канал 1 в режиме PWM1 */
    TIM4->CCMR1 &= ~TIM_CCMR1_OC1M;
    TIM4->CCMR1 |= (0x6 << 4);  // PWM mode 1
    TIM4->CCMR1 |= TIM_CCMR1_OC1PE;  // Preload enable

    /* Настраиваем выход */
    TIM4->CCER |= TIM_CCER_CC1E;  // Включаем канал 1
    TIM4->CCER &= ~TIM_CCER_CC1P; // Активный высокий уровень

    /* Устанавливаем ширину импульса: 10 мкс */
    TIM4->CCR1 = 10 - 1;  // 10 мкс импульс

    /* Включаем предзагрузку ARR */
    TIM4->CR1 |= TIM_CR1_ARPE;

    /* Применяем настройки */
    TIM4->EGR = TIM_EGR_UG;
}

/**
  * @brief  Запуск генератора импульсов
  */
void Start_Pulse_Generator(void)
{
    /* Запускаем таймер */
    TIM4->CR1 |= TIM_CR1_CEN;

    USART2_Print("[PULSE] Generator started on PB6 (TIM4_CH1)\r\n");
    USART2_Print("[PULSE] 10 us pulse, 100 ms period (10 Hz)\r\n");
    USART2_Print("[PULSE] Duty cycle: 0.01%\r\n");
}

/**
  * @brief  Чтение одного значения ADC
  */
uint32_t Read_ADC_Single(ADC_HandleTypeDef* hadc, uint32_t channel, uint32_t sampling_time)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    /* Настройка канала */
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = sampling_time;

    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) {
        return 0;
    }

    /* Запуск преобразования */
    HAL_ADC_Start(hadc);

    /* Ожидание завершения преобразования */
    if (HAL_ADC_PollForConversion(hadc, 10) != HAL_OK) {
        HAL_ADC_Stop(hadc);
        return 0;
    }

    /* Получение результата */
    uint32_t adc_value = HAL_ADC_GetValue(hadc);

    /* Остановка ADC */
    HAL_ADC_Stop(hadc);

    return adc_value;
}

/**
  * @brief  Чтение среднего значения ADC
  */
uint32_t Read_ADC_Average(ADC_HandleTypeDef* hadc, uint32_t channel, uint32_t sampling_time, uint8_t samples)
{
    uint32_t sum = 0;
    uint8_t valid_samples = 0;

    for (uint8_t i = 0; i < samples; i++) {
        uint32_t value = Read_ADC_Single(hadc, channel, sampling_time);
        if (value > 0 && value < 4096) {
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

/**
  * @brief  Чтение всех напряжений
  */
void Read_All_Voltages(void)
{
    /* Проверка инициализации ADC */
    if (!adc1_initialized || !adc2_initialized) {
        USART2_Print("[ADC] ERROR: ADC not initialized!\r\n");
        return;
    }

    uint32_t adc_raw_vdda = 0;
    uint32_t adc_raw_24v = 0;
    uint32_t adc_raw_12v = 0;
    uint32_t adc_raw_5v = 0;

    /* --- Измерение VREFINT для калибровки VDDA --- */

    /* Включаем внутренний источник опорного напряжения */
    ADC1->CR2 |= ADC_CR2_TSVREFE;
    HAL_Delay(10);

    /* Измерение VREFINT (канал 17) */
    adc_raw_vdda = Read_ADC_Average(&hadc1, ADC_CHANNEL_VREFINT, ADC_SAMPLETIME_239CYCLES_5, ADC_SAMPLES);

    /* Выключаем VREFINT */
    ADC1->CR2 &= ~ADC_CR2_TSVREFE;
    HAL_Delay(1);

    USART2_Print("[ADC] Raw VREFINT: ");
    USART2_PrintNum(adc_raw_vdda);
    USART2_Print("\r\n");

    if (adc_raw_vdda == 0) {
        current_vdda = 3.3f;
        USART2_Print("[ADC] WARNING: VREFINT = 0! Using default 3.3V\r\n");
    } else if (adc_raw_vdda > 4095) {
        current_vdda = 3.3f;
        USART2_Print("[ADC] WARNING: VREFINT > 4095! Using default 3.3V\r\n");
    } else {
        /* Расчет VDDA: VREFINT_CAL * VREFINT_VOLTAGE / ADC_reading */
        current_vdda = (VREFINT_VOLTAGE * VREFINT_CAL) / (float)adc_raw_vdda;
    }

    /* --- 24V (PA0, канал 0, ADC1) --- */
    adc_raw_24v = Read_ADC_Average(&hadc1, ADC_CHANNEL_0, ADC_SAMPLETIME_239CYCLES_5, ADC_SAMPLES);

    USART2_Print("[ADC] Raw 24V: ");
    USART2_PrintNum(adc_raw_24v);
    USART2_Print("\r\n");

    /* Коэффициент делителя для 24В: (150k+10k)/10k = 16 */
    if (adc_raw_24v > 0 && adc_raw_24v < 4096) {
        current_24v = (float)adc_raw_24v * current_vdda / 4095.0f * 16.0f;
    } else {
        current_24v = 24.0f;
        USART2_Print("[ADC] WARNING: 24V measurement invalid, using default\r\n");
    }

    /* --- 12V (PA1, канал 1, ADC2) --- */
    adc_raw_12v = Read_ADC_Average(&hadc2, ADC_CHANNEL_1, ADC_SAMPLETIME_239CYCLES_5, ADC_SAMPLES);

    USART2_Print("[ADC] Raw 12V: ");
    USART2_PrintNum(adc_raw_12v);
    USART2_Print("\r\n");

    /* Коэффициент делителя для 12В: (82k+10k)/10k = 9.2 */
    if (adc_raw_12v > 0 && adc_raw_12v < 4096) {
        current_12v = (float)adc_raw_12v * current_vdda / 4095.0f * 9.2f;
    } else {
        current_12v = 12.0f;
        USART2_Print("[ADC] WARNING: 12V measurement invalid, using default\r\n");
    }

    /* --- 5V (PA5, канал 5, ADC2) --- */
    adc_raw_5v = Read_ADC_Average(&hadc2, ADC_CHANNEL_5, ADC_SAMPLETIME_239CYCLES_5, ADC_SAMPLES);

    USART2_Print("[ADC] Raw 5V: ");
    USART2_PrintNum(adc_raw_5v);
    USART2_Print("\r\n");

    /* Коэффициент делителя для 5В: (20k+10k)/10k = 3 */
    if (adc_raw_5v > 0 && adc_raw_5v < 4096) {
        current_5v = (float)adc_raw_5v * current_vdda / 4095.0f * 3.0f;
    } else {
        current_5v = 5.0f;
        USART2_Print("[ADC] WARNING: 5V measurement invalid, using default\r\n");
    }

    /* Ограничение значений */
    if (current_vdda < 2.0f || current_vdda > 4.0f) {
        current_vdda = 3.3f;
        USART2_Print("[ADC] ERROR: VDDA out of range, reset\r\n");
    }
    if (current_24v < 10.0f || current_24v > 30.0f) {
        current_24v = 24.0f;
        USART2_Print("[ADC] ERROR: 24V out of range, reset\r\n");
    }
    if (current_12v < 5.0f || current_12v > 15.0f) {
        current_12v = 12.0f;
        USART2_Print("[ADC] ERROR: 12V out of range, reset\r\n");
    }
    if (current_5v < 3.0f || current_5v > 6.0f) {
        current_5v = 5.0f;
        USART2_Print("[ADC] ERROR: 5V out of range, reset\r\n");
    }
}

/**
  * @brief System Clock Configuration
  */
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
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

    /* Настройка светодиода на PC13 */
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* Настройка аналоговых входов */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Настройка PB1 для захвата частоты (TIM3_CH4) */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Настройка PB5 как выход для генерации импульсов (ручной режим) */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Настройка PB6 как альтернативной функции для TIM4_CH1 */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Настройка USART1 (PA9-TX, PA10-RX) */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Настройка USART2 (PA2-TX, PA3-RX) */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
  * @brief USART1 Initialization Function (ModBus)
  */
static void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 9600;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief USART2 Initialization Function (Debug)
  */
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
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief ADC1 Initialization Function
  */
static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief ADC2 Initialization Function
  */
static void MX_ADC2_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc2.Instance = ADC2;
    hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc2.Init.ContinuousConvMode = DISABLE;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc2.Init.NbrOfConversion = 1;
    if (HAL_ADC_Init(&hadc2) != HAL_OK)
    {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief TIM4 Initialization Function (генерация импульсов)
  */
static void MX_TIM4_Init(void)
{
    /* Включаем тактирование TIM4 */
    __HAL_RCC_TIM4_CLK_ENABLE();

    /* Базовая инициализация таймера */
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 72 - 1;  // 72 МГц / 72 = 1 МГц (1 тик = 1 мкс)
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 100000 - 1;  // 100000 мкс = 100 мс (10 Гц)
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
    {
        Error_Handler();
    }

    /* Инициализация канала 1 в режиме PWM */
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 10 - 1;  // Ширина импульса 10 мкс
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(100);
    }
}
