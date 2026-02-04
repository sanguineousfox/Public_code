/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * @description    : Magnetostrictive sensor time-of-flight measurement
  *                   - Measurement cycle: 10 seconds
  *                   - Red LED (PB13) lights up when signal captured on CLIK (PB1)
  *                   - Blue LED (PB12) blinks to indicate system activity
  *                   - CRITICAL: Optimized for 2 us signal capture (no latency)
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
#define VREFINT_CAL_VALUE   (*(uint16_t*)0x1FFFF7BA)  // Правильное чтение калибровки (~1500-1550)
#define ADC_SAMPLES     16      // Количество выборок для усреднения
#define MEAS_TIMEOUT_MS 10      // Таймаут измерения времени пролёта (10 мс)

/* ЧАСТОТНЫЕ КОЭФФИЦИЕНТЫ ДЕЛИТЕЛЕЙ */
#define DIV_24V_FACTOR  9.1f    // Резисторы ~81к+10к
#define DIV_12V_FACTOR  4.5f    // Резисторы ~35к+10к
#define DIV_5V_FACTOR   2.0f    // Резисторы 10к+10к

/* МАГНИТОСТРИКЦИОННЫЙ ДАТЧИК */
#define PULSE_WIDTH_US  10      // Ширина генерируемого импульса (мкс)
#define PULSE_PERIOD_MS 10000   // Период измерения = 10 секунд
#define SOUND_SPEED_MPS 2800.0f // Скорость звука в волноводе (м/с)
#define TOF_TICK_US     (1000000.0f / (TIMER_CLOCK_HZ / 7.0f))  // 0.09722 мкс на тик

/* АКТИВНЫЕ УРОВНИ */
#define POWER_5V_ON     GPIO_PIN_RESET  // Низкий уровень = питание ВКЛЮЧЕНО (постоянно!)
#define PULSE_HIGH      GPIO_PIN_SET    // Высокий уровень = импульс АКТИВЕН
#define PULSE_LOW       GPIO_PIN_RESET  // Низкий уровень = импульс НЕ АКТИВЕН
#define LED_RED_ON      GPIO_PIN_SET    // Красный светодиод: включён = HIGH
#define LED_RED_OFF     GPIO_PIN_RESET  // Красный светодиод: выключен = LOW
#define LED_BLUE_ON     GPIO_PIN_SET    // Синий светодиод: включён = HIGH
#define LED_BLUE_OFF    GPIO_PIN_RESET  // Синий светодиод: выключен = LOW

/* СВЕТОДИОДЫ */
#define LED_RED_PIN     GPIO_PIN_13  // PB13 = красный светодиод
#define LED_BLUE_PIN    GPIO_PIN_12  // PB12 = синий светодиод

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;  // Для ModBus (USART1)
UART_HandleTypeDef huart2;  // Для отладки (USART2)
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

// Глобальные переменные для измерения времени пролёта
volatile uint32_t tof_capture_value = 0;    // Значение захвата таймера
volatile uint8_t tof_measurement_done = 0;  // Флаг завершения измерения
volatile uint8_t tof_timeout = 0;           // Флаг таймаута
volatile uint8_t signal_captured = 0;       // Флаг: был ли захвачен сигнал на CLIK

// Переменные для хранения измерений
static float current_vdda = 3.3f;
static float current_24v = 24.0f;
static float current_12v = 12.0f;
static float current_5v = 5.0f;

// Флаги инициализации ADC
static uint8_t adc1_initialized = 0;
static uint8_t adc2_initialized = 0;

// Буфер для преобразования float в строку
static char float_buffer[32];

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
void Error_Handler(void);

void TIM3_InputCapture_Init(void);
void generate_pulse_and_start_timer(void);
uint32_t measure_time_of_flight(void);
void print_tof_results(uint32_t tof_ticks, float tof_us, float position_mm, uint8_t captured);
void Read_All_Voltages(void);
uint32_t Read_ADC_Single(ADC_HandleTypeDef* hadc, uint32_t channel, uint32_t sampling_time);
uint32_t Read_ADC_Average(ADC_HandleTypeDef* hadc, uint32_t channel, uint32_t sampling_time, uint8_t samples);

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

    /* Инициализация захвата времени пролёта (PB1 = TIM3_CH4 = CLIK) */
    TIM3_InputCapture_Init();

    /* Калибровка ADC */
    USART2_Print("[ИНИЦ] Калибровка ADC1...\r\n");
    if (HAL_ADCEx_Calibration_Start(&hadc1) == HAL_OK) {
        adc1_initialized = 1;
        USART2_Print("[ИНИЦ] ADC1 откалиброван успешно\r\n");
    } else {
        USART2_Print("[ИНИЦ] Калибровка ADC1 НЕ УДАЛАСЬ!\r\n");
    }

    USART2_Print("[ИНИЦ] Калибровка ADC2...\r\n");
    if (HAL_ADCEx_Calibration_Start(&hadc2) == HAL_OK) {
        adc2_initialized = 1;
        USART2_Print("[ИНИЦ] ADC2 откалиброван успешно\r\n");
    } else {
        USART2_Print("[ИНИЦ] Калибровка ADC2 НЕ УДАЛАСЬ!\r\n");
    }

    /* Инициализация ModBus */
    ModBus_Init();

    /* Задержка для стабилизации */
    HAL_Delay(1000);

    /* Настройка приоритетов прерываний */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);  // МАКСИМАЛЬНЫЙ ПРИОРИТЕТ для захвата!
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);

    /* Включаем глобальные прерывания */
    __enable_irq();

    /* Вывод информации о системе */
    USART2_Print("\r\n========================================\r\n");
    USART2_Print(" Система измерения времени пролёта (TOF)\r\n");
    USART2_Print("========================================\r\n");
    USART2_Print("Система инициализирована\r\n");
    USART2_Print("USART2: Отладочный вывод (115200)\r\n");
    USART2_Print("USART1: ModBus RTU (9600, 8N1)\r\n");
    USART2_Print("Адрес устройства: 1\r\n");
    USART2_Print("Интервал измерений: 10 секунд\r\n");
    USART2_Print("Индикация: Красный LED (PB13) = сигнал захвачен на CLIK\r\n");
    USART2_Print("           Синий LED (PB12) = мигает (работа системы)\r\n");
    USART2_Print("VREFINT_CAL: ");
    USART2_PrintNum(VREFINT_CAL_VALUE);
    USART2_Print("\r\nПины:\r\n");
    USART2_Print("  PB5 (Gen_Impuls):  выход, импульс 10 мкс\r\n");
    USART2_Print("  PB6 (Power_5V):    выход, постоянно LOW (питание ВКЛ)\r\n");
    USART2_Print("  PB1 (CLIK):        вход, TIM3_CH4\r\n");
    USART2_Print("  PB13 (LED_RED):    выход, загорается при захвате сигнала\r\n");
    USART2_Print("  PB12 (LED_BLUE):   выход, мигает (1 Гц)\r\n");
    USART2_Print("========================================\r\n\r\n");

    /* Тестовое измерение напряжений при запуске */
    Read_All_Voltages();

    USART2_Print("[ИНИЦ] --- РЕЗУЛЬТАТЫ ИЗМЕРЕНИЙ ---\r\n");
    USART2_Print("[ИНИЦ] VDDA = ");
    float_to_str(current_vdda, float_buffer, 3);
    USART2_Print(float_buffer);
    USART2_Print(" В\r\n");

    USART2_Print("[ИНИЦ] 24В  = ");
    float_to_str(current_24v, float_buffer, 2);
    USART2_Print(float_buffer);
    USART2_Print(" В\r\n");

    USART2_Print("[ИНИЦ] 12В  = ");
    float_to_str(current_12v, float_buffer, 2);
    USART2_Print(float_buffer);
    USART2_Print(" В\r\n");

    USART2_Print("[ИНИЦ] 5В   = ");
    float_to_str(current_5v, float_buffer, 2);
    USART2_Print(float_buffer);
    USART2_Print(" В\r\n");

    /* Основные переменные цикла */
    uint32_t last_measure_time = 0;
    uint32_t last_debug_time = 0;
    uint8_t blue_led_state = 0;

    /* Бесконечный цикл */
    while (1)
    {
        /* Мигание синим светодиодом каждую секунду (индикация работы) */
        if (HAL_GetTick() - last_debug_time >= 1000)
        {
            last_debug_time = HAL_GetTick();
            blue_led_state = !blue_led_state;
            HAL_GPIO_WritePin(GPIOB, LED_BLUE_PIN, blue_led_state ? LED_BLUE_ON : LED_BLUE_OFF);
        }

        /* Измерение каждые 10 секунд */
        if (HAL_GetTick() - last_measure_time >= PULSE_PERIOD_MS)
        {
            last_measure_time = HAL_GetTick();

            /* === НАЧАЛО НОВОГО ЦИКЛА ИЗМЕРЕНИЯ === */
            USART2_Print("\r\n========================================\r\n");
            USART2_Print("ЦИКЛ ИЗМЕРЕНИЯ ЗАПУЩЕН (каждые 10 сек)\r\n");
            USART2_Print("========================================\r\n");

            /* Сброс индикации: погасить красный светодиод */
            HAL_GPIO_WritePin(GPIOB, LED_RED_PIN, LED_RED_OFF);
            signal_captured = 0;

            /* Шаг 1: Измерение напряжений */
            Read_All_Voltages();
            ModBus_UpdateVoltages(current_vdda, current_24v, current_12v, current_5v);

            /* Шаг 2: Генерация импульса и измерение времени пролёта */
            uint32_t tof_ticks = measure_time_of_flight();
            float tof_us = tof_ticks * TOF_TICK_US;

            /* Коррекция: вычитаем время импульса (10 мкс) */
            if (tof_ticks > (10.0f / TOF_TICK_US)) {
                tof_us -= 10.0f;
            }

            float position_mm = (tof_us * 0.001f * SOUND_SPEED_MPS) / 2.0f;  // /2 так как туда-обратно

            /* Шаг 3: Индикация результата */
            if (tof_ticks > 0 && tof_measurement_done) {
                /* Сигнал успешно захвачен — зажечь красный светодиод */
                HAL_GPIO_WritePin(GPIOB, LED_RED_PIN, LED_RED_ON);
                signal_captured = 1;
                USART2_Print("[УСПЕХ] Сигнал захвачен на CLIK (PB1)!\r\n");
            } else {
                /* Сигнал НЕ захвачен — красный светодиод остаётся погашенным */
                HAL_GPIO_WritePin(GPIOB, LED_RED_PIN, LED_RED_OFF);
                signal_captured = 0;
                USART2_Print("[ОШИБКА] Сигнал НЕ захвачен на CLIK (PB1)!\r\n");
            }

            /* Обновление данных в ModBus */
            ModBus_UpdateMeasurements(tof_us, position_mm, 0, signal_captured ? 1 : 0);

            /* Вывод результатов */
            USART2_Print("\r\n--- РЕЗУЛЬТАТЫ ИЗМЕРЕНИЯ ---\r\n");
            USART2_Print("Напряжения: VDDA=");
            float_to_str(current_vdda, float_buffer, 2);
            USART2_Print(float_buffer);
            USART2_Print("В 24В=");
            float_to_str(current_24v, float_buffer, 1);
            USART2_Print(float_buffer);
            USART2_Print("В 12В=");
            float_to_str(current_12v, float_buffer, 1);
            USART2_Print(float_buffer);
            USART2_Print("В 5В=");
            float_to_str(current_5v, float_buffer, 1);
            USART2_Print(float_buffer);
            USART2_Print("В\r\n");

            print_tof_results(tof_ticks, tof_us, position_mm, signal_captured);

            /* Информация о следующем цикле */
            USART2_Print("\r\nСледующий цикл измерения через 10 секунд...\r\n");
            USART2_Print("========================================\r\n");
        }

        /* Обработка ModBus */
        ModBus_Process();
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
  * @brief  Инициализация TIM3 для измерения времени пролёта (канал 4, PB1 = CLIK)
  */
void TIM3_InputCapture_Init(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    /* КРИТИЧЕСКИ ВАЖНО: Перемаппирование ДО настройки GPIO! */
    __HAL_AFIO_REMAP_TIM3_PARTIAL();  // PB1 = TIM3_CH4 (частичное перемаппирование)

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;  // PB1 как цифровой вход (CLIK)
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Настройка таймера */
    TIM3->CR1 = 0;
    TIM3->CR2 = 0;
    TIM3->PSC = 6;          // 72 МГц / 7 = 10.2857 МГц → 1 тик = 0.09722 мкс
    TIM3->ARR = 0xFFFF;     // Максимальный период (~6.7 мс)
    TIM3->CNT = 0;
    TIM3->EGR = TIM_EGR_UG; // Применить настройки

    /* Настройка канала 4 для захвата по фронту */
    TIM3->CCMR2 = 0;
    TIM3->CCMR2 |= (0x1 << 12);     // CC4S = 01 (вход от TI4/PB1)
    TIM3->CCMR2 &= ~(0x3 << 10);    // Без предделителя захвата
    TIM3->CCMR2 &= ~(0xF << 8);     // Фильтр = 0 (МАКСИМАЛЬНАЯ СКОРОСТЬ!)

    TIM3->CCER = TIM_CCER_CC4E;     // Включить захват по фронту на канале 4
    TIM3->DIER = TIM_DIER_CC4IE;    // Только прерывание захвата
    TIM3->SR = 0;                   // Очистить флаги
}

/**
  * @brief  КРИТИЧЕСКИ ВАЖНО: Таймер запускается ДО генерации импульса!
  *         Это позволяет захватить очень короткие задержки (2 мкс)
  */
void generate_pulse_and_start_timer(void)
{
    /* Сброс флагов измерения */
    tof_measurement_done = 0;
    tof_timeout = 0;
    tof_capture_value = 0;
    TIM3->SR = 0;
    TIM3->CNT = 0;  // Сброс счётчика

    /* === КРИТИЧЕСКИ ВАЖНО: ЗАПУСК ТАЙМЕРА ДО ИМПУЛЬСА === */
    TIM3->CR1 |= TIM_CR1_CEN;  // Запуск таймера (готов к захвату СРАЗУ!)

    /* МИНИМАЛЬНАЯ ЗАДЕРЖКА ДЛЯ СТАБИЛИЗАЦИИ ТАЙМЕРА (1 такт) */
    __asm__("NOP");  // ТОЛЬКО 1 такт задержки!

    /* Генерация импульса 10 мкс на PB5 (Gen_Impuls) */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, PULSE_HIGH);

    /* ТОЧНАЯ ЗАДЕРЖКА 10 МКС */
    for (volatile uint32_t i = 0; i < 54; i++) {
        __NOP();
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, PULSE_LOW);

    /* Таймер уже работает — ничего не делаем */
}

/**
  * @brief  Измерение времени пролёта
  * @retval Значение таймера в тиках (0 при таймауте)
  */
uint32_t measure_time_of_flight(void)
{
    /* Генерация импульса с предварительным запуском таймера */
    generate_pulse_and_start_timer();

    /* Ожидание завершения измерения или таймаута */
    uint32_t start_wait = HAL_GetTick();
    while (!tof_measurement_done && !tof_timeout) {
        if ((HAL_GetTick() - start_wait) >= MEAS_TIMEOUT_MS) {
            tof_timeout = 1;
            TIM3->CR1 &= ~TIM_CR1_CEN;  // Остановить таймер
            break;
        }
        ModBus_Process();
        __NOP();
    }

    /* Остановка таймера */
    TIM3->CR1 &= ~TIM_CR1_CEN;
    TIM3->SR = 0;

    return tof_capture_value;
}

/**
  * @brief  Вывод результатов измерения времени пролёта
  */
void print_tof_results(uint32_t tof_ticks, float tof_us, float position_mm, uint8_t captured)
{
    if (captured) {
        USART2_Print("[ИНДИКАЦИЯ] Красный светодиод (PB13): ВКЛ (сигнал захвачен)\r\n");
    } else {
        USART2_Print("[ИНДИКАЦИЯ] Красный светодиод (PB13): ВЫКЛ (сигнал НЕ захвачен)\r\n");
    }

    USART2_Print("Время пролёта: ");
    if (captured) {
        float_to_str(tof_us, float_buffer, 2);
        USART2_Print(float_buffer);
        USART2_Print(" мкс (");
        USART2_PrintNum(tof_ticks);
        USART2_Print(" тиков)\r\n");
    } else {
        USART2_Print("НЕ ИЗМЕРЕНО (таймаут)\r\n");
    }

    if (captured) {
        USART2_Print("Положение магнита: ");
        float_to_str(position_mm, float_buffer, 1);
        USART2_Print(float_buffer);
        USART2_Print(" мм от начала волновода\r\n");
    }
}

/**
  * @brief  Чтение одного значения ADC
  */
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

/**
  * @brief  Чтение среднего значения ADC
  */
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

/**
  * @brief  Чтение всех напряжений
  */
void Read_All_Voltages(void)
{
    if (!adc1_initialized || !adc2_initialized) {
        USART2_Print("[ADC] ОШИБКА: ADC не инициализирован!\r\n");
        return;
    }

    uint32_t adc_raw_vdda = 0;
    uint32_t adc_raw_24v = 0;
    uint32_t adc_raw_12v = 0;
    uint32_t adc_raw_5v = 0;

    /* --- Измерение VREFINT для калибровки VDDA --- */
    ADC1->CR2 |= ADC_CR2_TSVREFE;
    HAL_Delay(10);
    adc_raw_vdda = Read_ADC_Average(&hadc1, ADC_CHANNEL_VREFINT, ADC_SAMPLETIME_239CYCLES_5, ADC_SAMPLES);
    ADC1->CR2 &= ~ADC_CR2_TSVREFE;
    HAL_Delay(1);

    /* Расчёт VDDA */
    if (adc_raw_vdda > 1000 && adc_raw_vdda < 2000 && VREFINT_CAL_VALUE > 1000 && VREFINT_CAL_VALUE < 2000) {
        current_vdda = 3.3f * (float)VREFINT_CAL_VALUE / (float)adc_raw_vdda;
    } else {
        current_vdda = 3.3f;
    }

    /* --- 24В (PA0, ADC1) --- */
    adc_raw_24v = Read_ADC_Average(&hadc1, ADC_CHANNEL_0, ADC_SAMPLETIME_239CYCLES_5, ADC_SAMPLES);
    if (adc_raw_24v > 100 && adc_raw_24v < 4000) {
        float adc_voltage = (float)adc_raw_24v * current_vdda / 4095.0f;
        current_24v = adc_voltage * DIV_24V_FACTOR;
    } else {
        current_24v = 24.0f;
    }

    /* --- 12В (PA1, ADC2) --- */
    adc_raw_12v = Read_ADC_Average(&hadc2, ADC_CHANNEL_1, ADC_SAMPLETIME_239CYCLES_5, ADC_SAMPLES);
    if (adc_raw_12v > 100 && adc_raw_12v < 4000) {
        float adc_voltage = (float)adc_raw_12v * current_vdda / 4095.0f;
        current_12v = adc_voltage * DIV_12V_FACTOR;
    } else {
        current_12v = 12.0f;
    }

    /* --- 5В (PA5, ADC2) --- */
    adc_raw_5v = Read_ADC_Average(&hadc2, ADC_CHANNEL_5, ADC_SAMPLETIME_239CYCLES_5, ADC_SAMPLES);
    if (adc_raw_5v > 100 && adc_raw_5v < 4000) {
        float adc_voltage = (float)adc_raw_5v * current_vdda / 4095.0f;
        current_5v = adc_voltage * DIV_5V_FACTOR;
    } else {
        current_5v = 5.0f;
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

    /* Настройка кварца 32768 Гц на PC14/PC15 */
    GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* Настройка аналоговых входов */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Настройка PB1 для захвата времени пролёта (CLIK - TIM3_CH4) */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Настройка PB5 как цифрового выхода для генерации импульсов (Gen_Impuls) */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Настройка PB6 как выхода управления питанием 5В (активный низкий уровень) */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Настройка красного светодиода на PB13 (загорается при захвате сигнала) */
    GPIO_InitStruct.Pin = LED_RED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, LED_RED_PIN, LED_RED_OFF);  // Изначально ВЫКЛ

    /* Настройка синего светодиода на PB12 (мигает для индикации работы) */
    GPIO_InitStruct.Pin = LED_BLUE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, LED_BLUE_PIN, LED_BLUE_OFF);  // Изначально ВЫКЛ

    /* ВАЖНО: Питание 5В постоянно ВКЛЮЧЕНО (низкий уровень на PB6) */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, POWER_5V_ON);  // LOW = питание ВКЛ

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
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        /* Мигание красным светодиодом при критической ошибке */
        HAL_GPIO_TogglePin(GPIOB, LED_RED_PIN);
        HAL_Delay(200);
    }
}
