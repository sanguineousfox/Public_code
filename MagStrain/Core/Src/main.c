/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file           : main.c
* @brief          : Main program body
* @description    : Адаптация под документацию ПМП-201Е (СЕНС.421411.028 РЭ)
*                   - Измерение уровня через магнитострикцию
*                   - Генерация импульса на PB5 (Gen_Impuls)
*                   - Управление транзисторами через PB7 (Switch_In_impuls):
*                     * Перед импульсом: выключить (LOW)
*                     * Через 15-20 мкс после импульса: включить (HIGH)
*                     * Удерживать включённым минимум 500 мкс (или вычисленное время прохождения сигнала по волноводу)
*                     * Непосредственно перед повторной отправкой: выключить (LOW)
*                   - PB1 всегда в режиме входа (захват времени пролёта)
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
#include <math.h>

/* Private define ------------------------------------------------------------*/
#define TIMER_CLOCK_HZ      72000000.0f
#define VREFINT_CAL_ADDR    ((uint16_t*)0x1FFFF7BA)
#define VREFINT_CAL_VALUE   (*VREFINT_CAL_ADDR)
#define ADC_SAMPLES         16
#define MEAS_TIMEOUT_MS     10
#define DIV_24V_FACTOR      9.2f
#define DIV_12V_FACTOR      4.6f
#define DIV_5V_FACTOR       2.0f
#define PULSE_WIDTH_US      10
#define PULSE_PERIOD_MS     10000
#define SOUND_SPEED_MPS     2800.0f
#define TOF_TICK_US         (1000000.0f / (TIMER_CLOCK_HZ / 7.0f))
#define LED_RED_ON          GPIO_PIN_SET
#define LED_RED_OFF         GPIO_PIN_RESET
#define LED_BLUE_ON         GPIO_PIN_SET
#define LED_BLUE_OFF        GPIO_PIN_RESET
#define LED_RED_PIN         GPIO_PIN_13
#define LED_BLUE_PIN        GPIO_PIN_12
#define LED_RED_ON_TIME_MS  1000
#define PULSE_DELAY_ITERATIONS 45
#define SWITCH_PIN          GPIO_PIN_7    // PB7 - управление транзисторами (Switch_In_impuls)
#define SWITCH_PORT         GPIOB
#define DELAY_AFTER_PULSE_ITER  97        // 18 мкс (15-20 мкс после импульса)
// Минимальное время удержания транзистора закрытым (500 мкс = 2700 итераций при 72 МГц)
// Используем вычисляемое значение на основе длины волновода
#define WAVEGUIDE_LENGTH_MM 6000          // Длина волновода в мм (настраивается)
#define SWITCH_HOLD_ITERATIONS ((WAVEGUIDE_LENGTH_MM * 1931) / 1000)  // Масштабируем для компиляции
// #define SWITCH_HOLD_ITERATIONS 965   // Для 500 мм (0.179 мс)
// #define SWITCH_HOLD_ITERATIONS 1931  // Для 1000 мм (0.358 мс)
// #define SWITCH_HOLD_ITERATIONS 3862  // Для 2000 мм (0.716 мс)
// #define SWITCH_HOLD_ITERATIONS 5793  // Для 3000 мм (1.07 мс)
// #define SWITCH_HOLD_ITERATIONS 7724  // Для 4000 мм (1.43 мс)
// #define SWITCH_HOLD_ITERATIONS 9655  // Для 5000 мм (1.79 мс)
// #define SWITCH_HOLD_ITERATIONS 11586 // Для 6000 мм (2.14 мс)
#define SWITCH_HOLD_ITERATIONS 12000  // 2.22 мс (достаточно для 6000 мм)
/* ПРИМЕЧАНИЕ: ДЛЯ ДЛИНЫ ВОЛНОВОДА 6000 мм ВРЕМЯ ПРОХОЖДЕНИЯ = 2.14 мс */
#define MODBUS_SILENCE_TIME_MS 4
#define RS485_CTRL_PIN      GPIO_PIN_0
#define RS485_CTRL_PORT     GPIOB
#define FIRMWARE_VERSION    100

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
volatile uint32_t tof_capture_value = 0;
volatile uint8_t tof_measurement_done = 0;
volatile uint8_t tof_timeout = 0;
volatile uint8_t signal_captured = 0;
static float current_vdda = 3.3f;
static float current_24v = 24.0f;
static float current_12v = 12.0f;
static float current_5v = 5.0f;
static float current_temperature = 0.0f;
static uint8_t lm75b_initialized = 0;
static uint8_t v24_error = 0;
static uint8_t v12_error = 0;
static uint8_t v5_error = 0;
static uint8_t vdda_error = 0;
static uint8_t temp_error = 0;
static char float_buffer[32];
volatile uint8_t modbus_tx_active = 0;

#define RS485_SET_TRANSMIT() HAL_GPIO_WritePin(RS485_CTRL_PORT, RS485_CTRL_PIN, GPIO_PIN_SET)
#define RS485_SET_RECEIVE()  HAL_GPIO_WritePin(RS485_CTRL_PORT, RS485_CTRL_PIN, GPIO_PIN_RESET)

/* USER CODE BEGIN 0 */
void ModBus_TransmitFrame(uint8_t *frame, uint16_t len)
{
    if (len == 0) return;
    RS485_SET_TRANSMIT();
    modbus_tx_active = 1;
    for (volatile int i = 0; i < 150; i++) __NOP();
    HAL_UART_Transmit(&huart1, frame, len, 1000);
    uint32_t timeout_start = HAL_GetTick();
    while ((USART1->SR & USART_SR_TC) == 0) {
        if (HAL_GetTick() - timeout_start > 50) break;
    }
    HAL_Delay(MODBUS_SILENCE_TIME_MS);
    RS485_SET_RECEIVE();
    modbus_tx_active = 0;
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
/* Вынесенная функция для вывода результатов и обновления регистров Modbus */
void Process_Measurement_Results(float tof_us, float position_mm, uint8_t signal_captured)
{
    /* Обновление напряжений (ТОЛЬКО 4 аргумента - без флагов ошибок) */
    Read_All_Voltages();
    ModBus_UpdateVoltages(current_vdda, current_24v, current_12v, current_5v);

    /* Измерение температуры */
    Read_Temperature();

    /* Обновление ВСЕХ измерений через единую функцию (если объявлена в modbus.h) */
    /* Если ModBus_UpdateMeasurements не объявлена, закомментируйте эту строку */
    // ModBus_UpdateMeasurements(tof_us, current_temperature,
    //                          (signal_captured ? 0 : 1), temp_error, signal_captured ? 1 : 0);

    /* Вывод результатов в отладочный порт */
    USART2_Print("Измерение: ");

    // Напряжения
    USART2_Print("V=");
    float_to_str(current_24v, float_buffer, 1);
    USART2_Print(float_buffer);
    USART2_Print("/");
    float_to_str(current_12v, float_buffer, 1);
    USART2_Print(float_buffer);
    USART2_Print("/");
    float_to_str(current_5v, float_buffer, 1);
    USART2_Print(float_buffer);
    USART2_Print("V ");

    // Температура
    if (lm75b_initialized) {
        USART2_Print("T=");
        float_to_str(current_temperature, float_buffer, 1);
        USART2_Print(float_buffer);
        USART2_Print("C ");
    }

    // Уровень
    if (signal_captured) {
        USART2_Print("LEVEL=");
        float_to_str(position_mm, float_buffer, 0);
        USART2_Print(float_buffer);
        USART2_Print("mm (");
        float_to_str(position_mm * 0.001f, float_buffer, 3);
        USART2_Print(float_buffer);
        USART2_Print("m)");
    } else {
        USART2_Print("LEVEL=timeout");
    }
    USART2_Print("\r\n");
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
        /* Используем ОДИН датчик через базовую функцию (вместо несуществующей LM75B_Init_All) */
        if (LM75B_Init(LM75B_DEFAULT_ADDRESS) == HAL_OK) {
            lm75b_initialized = 1;
            USART2_Print("LM75B: обнаружен 1 датчик\r\n");
        } else {
            USART2_Print("LM75B: НЕ ОБНАРУЖЕН датчик температуры\r\n");
            temp_error = 1;
        }
    }

    /* Инициализация захвата времени пролёта (PB1 = TIM3_CH4 = CLIK) */
    TIM3_InputCapture_Init();

    /* Калибровка АЦП */
    USART2_Print("[ИНИЦ] Калибровка АЦП...\r\n");
    if (HAL_ADCEx_Calibration_Start(&hadc1) == HAL_OK) {
        USART2_Print("[ИНИЦ] АЦП1 откалиброван успешно\r\n");
    } else {
        USART2_Print("[ИНИЦ] Калибровка АЦП1 НЕ УДАЛАСЬ!\r\n");
        v24_error = 1;
        v12_error = 1;
        v5_error = 1;
        vdda_error = 1;
    }

    if (HAL_ADCEx_Calibration_Start(&hadc2) == HAL_OK) {
        USART2_Print("[ИНИЦ] АЦП2 откалиброван успешно\r\n");
    } else {
        USART2_Print("[ИНИЦ] Калибровка АЦП2 НЕ УДАЛАСЬ!\r\n");
        v12_error = 1;
        v5_error = 1;
    }

    /* Инициализация модуля АЦП */
    if (HAL_ADC_Init(&hadc1) != HAL_OK || HAL_ADC_Init(&hadc2) != HAL_OK) {
        USART2_Print("[ИНИЦ] Инициализация АЦП НЕ УДАЛАСЬ!\r\n");
        v24_error = 1;
        v12_error = 1;
        v5_error = 1;
        vdda_error = 1;
    }

    /* Инициализация модуля модбас */
    ModBus_Init();
    // ModBus_UpdateFirmwareVersion(FIRMWARE_VERSION); // Убрано - функция не объявлена

    /* Настройка прерываний */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
    __enable_irq();

    /* Первое измерение напряжений и температуры */
    Read_All_Voltages();
    Read_Temperature();
    ModBus_UpdateVoltages(current_vdda, current_24v, current_12v, current_5v);

    USART2_Print("ПМП-201Е запущен. Адрес модбас: 1, скорость: 19200 бод, период измерений: 10 сек.\r\n");

    /* Основные переменные цикла */
    uint32_t last_measure_time = 0;
    uint32_t last_debug_time = 0;
    uint32_t led_red_off_time = 0;
    uint8_t blue_led_state = 0;
    uint8_t red_led_state = 0;

    while (1)
    {
        /* Мигание синим светодиодом каждую секунду */
        if (HAL_GetTick() - last_debug_time >= 1000) {
            last_debug_time = HAL_GetTick();
            blue_led_state = !blue_led_state;
            HAL_GPIO_WritePin(GPIOB, LED_BLUE_PIN, blue_led_state ? LED_BLUE_ON : LED_BLUE_OFF);
        }

        /* Автоматическое выключение красного светодиода */
        if (red_led_state && (HAL_GetTick() - led_red_off_time >= LED_RED_ON_TIME_MS)) {
            HAL_GPIO_WritePin(GPIOB, LED_RED_PIN, LED_RED_OFF);
            red_led_state = 0;
        }

        /* Измерение каждые 10 секунд - КРИТИЧЕСКИ ВАЖНЫЙ ЦИКЛ */
        if (HAL_GetTick() - last_measure_time >= PULSE_PERIOD_MS) {
            last_measure_time = HAL_GetTick();

            /* Сброс индикации */
            HAL_GPIO_WritePin(GPIOB, LED_RED_PIN, LED_RED_OFF);
            red_led_state = 0;
            signal_captured = 0;

            /* === КРИТИЧЕСКИ ВАЖНЫЙ ЦИКЛ ИЗМЕРЕНИЯ === */
            /* Только управление транзисторами, генерация импульса и фиксация */
            uint32_t tof_ticks = measure_time_of_flight();
            float tof_us = tof_ticks * TOF_TICK_US;
            float position_mm = 0.0f;

            if (tof_ticks > 0 && !tof_timeout) {
                if (tof_us > 10.0f) tof_us -= 10.0f;
                position_mm = (tof_us * 0.001f * SOUND_SPEED_MPS) / 2.0f;
                HAL_GPIO_WritePin(GPIOB, LED_RED_PIN, LED_RED_ON);
                red_led_state = 1;
                led_red_off_time = HAL_GetTick();
                signal_captured = 1;
            }

            /* === ВЫНЕСЕННАЯ ОБРАБОТКА РЕЗУЛЬТАТОВ === */
            Process_Measurement_Results(tof_us, position_mm, signal_captured);
        }

        /* Обработка запросов модбас */
        ModBus_Process();

        /* Защита от зависания передачи */
        if (modbus_tx_active && (HAL_GetTick() - last_measure_time > 100)) {
            RS485_SET_RECEIVE();
            modbus_tx_active = 0;
        }

        HAL_Delay(1);
    }
}

/* Callback для приёма данных по модбас */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        ModBus_RxCallback(huart);
    }
}

/* Инициализация таймера для захвата времени пролёта */
void TIM3_InputCapture_Init(void)
{
	   __HAL_RCC_TIM3_CLK_ENABLE();
	    __HAL_RCC_GPIOB_CLK_ENABLE();
	    __HAL_RCC_AFIO_CLK_ENABLE();
	    __HAL_AFIO_REMAP_TIM3_PARTIAL();  // TIM3_CH4 на PB1 (частичный ремап)

	    /* --- Настройка PB1 как входа с высоким импедансом --- */
	    GPIO_InitTypeDef GPIO_InitStruct = {0};
	    GPIO_InitStruct.Pin = GPIO_PIN_1;
	    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;      // Вход без подтяжки
	    GPIO_InitStruct.Pull = GPIO_NOPULL;          // Сигнал уже имеет уровень от внешней схемы
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	    /* --- Настройка TIM3 в режиме Input Capture --- */
	    TIM3->CR1 = 0;                    // Сброс регистра управления
	    TIM3->CR2 = 0;                    // Сброс CR2
	    TIM3->PSC = 6;                    // Prescaler = 6 → тактирование 72 МГц / 7 = ~10.286 МГц (1 тик ≈ 97.2 нс)
	    TIM3->ARR = 0xFFFF;               // Авто-перезагрузка на максимум (16 бит)
	    TIM3->CNT = 0;                    // Сброс счётчика
	    TIM3->EGR = TIM_EGR_UG;           // Генерация события обновления для применения настроек

	    /* --- Настройка канала 4 (TIM3_CH4 = PB1) ---
	     * CCMR2, биты 12-13 (CC4S): 01 = входной сигнал с TI4
	     * Полярность захвата настраивается в CCER */
	    TIM3->CCMR2 = (0x1 << 12);        // CC4S = 01: IC4 mapped on TI4

	    /* --- Полярность захвата: КРИТИЧЕСКОЕ ИЗМЕНЕНИЕ ---
	     * TIM_CCER_CC4E  (бит 12) = 1 : включить захват канала 4
	     * TIM_CCER_CC4P  (бит 13) = 1 : захват по нисходящему фронту (HIGH→LOW)
	     * Без CC4P захват был бы по восходящему фронту (по умолчанию) */
	    TIM3->CCER = TIM_CCER_CC4E | TIM_CCER_CC4P;  // Falling edge capture

	    TIM3->DIER = TIM_DIER_CC4IE;      // Включить прерывание по захвату канала 4
	    TIM3->SR = 0;
}

/* Генерация импульса и запуск измерения С УПРАВЛЕНИЕМ транзисторами через PB7 */
void generate_pulse_and_measure(void)
{
    tof_measurement_done = 0;
    tof_timeout = 0;
    tof_capture_value = 0;
    TIM3->SR = 0;
    TIM3->CNT = 0;
    TIM3->CR1 |= TIM_CR1_CEN;
    __NOP();

    /* ГАРАНТИРОВАННО ВЫКЛЮЧАЕМ транзистор ПЕРЕД ИМПУЛЬСОМ (PB7 = LOW) */
    HAL_GPIO_WritePin(SWITCH_PORT, SWITCH_PIN, GPIO_PIN_RESET);

    /* ГЕНЕРАЦИЯ ИМПУЛЬСА 10 МКС НА Gen_Impuls (PB5) */
    GPIOB->BSRR = GPIO_PIN_5;
    for (volatile uint32_t i = 0; i < PULSE_DELAY_ITERATIONS; i++) __NOP();
    GPIOB->BRR = GPIO_PIN_5;

    /* ЗАДЕРЖКА 15-20 МКС (18 МКС) ПОСЛЕ ИМПУЛЬСА */
    for (volatile uint32_t i = 0; i < DELAY_AFTER_PULSE_ITER; i++) __NOP();

    /* ВКЛЮЧАЕМ транзистор через 15-20 мкс после импульса (PB7 = HIGH) */
    HAL_GPIO_WritePin(SWITCH_PORT, SWITCH_PIN, GPIO_PIN_SET);
}

/* Измерение времени пролёта С УПРАВЛЕНИЕМ транзисторами через PB7 */
uint32_t measure_time_of_flight(void)
{
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

    /* УДЕРЖИВАЕМ транзистор ВКЛЮЧЁННЫМ в течение времени прохождения сигнала по волноводу */
    /* Для длины 6000 мм: время = 2.14 мс = 11586 итераций */
    /* Используем фиксированное значение 12000 итераций (2.22 мс) для надёжности */
    for (volatile uint32_t i = 0; i < SWITCH_HOLD_ITERATIONS; i++) __NOP();

    /* ВЫКЛЮЧАЕМ транзистор НЕПОСРЕДСТВЕННО ПЕРЕД ПОВТОРНОЙ ОТПРАВКОЙ (в начале следующего цикла) */
    /* В следующем цикле измерения транзистор будет выключен в начале generate_pulse_and_measure() */
    HAL_GPIO_WritePin(SWITCH_PORT, SWITCH_PIN, GPIO_PIN_RESET);

    return tof_capture_value;
}

/* Чтение температуры с датчика (ОДИН датчик через базовую функцию) */
void Read_Temperature(void)
{
    if (!lm75b_initialized) {
        current_temperature = -127.0f;
        temp_error = 1;
        return;
    }

    /* Используем базовую функцию чтения (вместо несуществующей LM75B_ReadAllTemperatures) */
    if (LM75B_ReadRawTemperature(LM75B_DEFAULT_ADDRESS, &current_temperature) != HAL_OK) {
        current_temperature = -127.0f;
        temp_error = 1;
        USART2_Print("[I2C] Ошибка чтения температуры.\r\n");
    } else {
        temp_error = 0;
    }
}

/* ============================================================================
 * ИЗМЕРЕНИЕ НАПРЯЖЕНИЙ С КАЛИБРОВКОЙ VREFINT
 * ========================================================================== */
void Read_All_Voltages(void)
{
    uint32_t adc_raw_vdda = 0;
    uint32_t adc_raw_24v = 0;
    uint32_t adc_raw_12v = 0;
    uint32_t adc_raw_5v = 0;

    /* --- КАЛИБРОВКА VDDA ЧЕРЕЗ ВНУТРЕННИЙ ИСТОЧНИК ОПОРНОГО НАПРЯЖЕНИЯ --- */
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

    /* --- ИЗМЕРЕНИЕ 24В (PA0, ADC1) --- */
    adc_raw_24v = Read_ADC_Average(&hadc1, ADC_CHANNEL_0, ADC_SAMPLETIME_239CYCLES_5, ADC_SAMPLES);
    if (adc_raw_24v > 100 && adc_raw_24v < 4000) {
        float adc_voltage = (float)adc_raw_24v * current_vdda / 4095.0f;
        current_24v = adc_voltage * DIV_24V_FACTOR;
        v24_error = 0;
    } else {
        current_24v = 24.0f;
        v24_error = 1;
    }

    /* --- ИЗМЕРЕНИЕ 12В (PA1, ADC2) --- */
    adc_raw_12v = Read_ADC_Average(&hadc2, ADC_CHANNEL_1, ADC_SAMPLETIME_239CYCLES_5, ADC_SAMPLES);
    if (adc_raw_12v > 100 && adc_raw_12v < 4000) {
        float adc_voltage = (float)adc_raw_12v * current_vdda / 4095.0f;
        current_12v = adc_voltage * DIV_12V_FACTOR;
        v12_error = 0;
    } else {
        current_12v = 12.0f;
        v12_error = 1;
    }

    /* --- ИЗМЕРЕНИЕ 5В (PA5, ADC2) --- */
    adc_raw_5v = Read_ADC_Average(&hadc2, ADC_CHANNEL_5, ADC_SAMPLETIME_239CYCLES_5, ADC_SAMPLES);
    if (adc_raw_5v > 100 && adc_raw_5v < 4000) {
        float adc_voltage = (float)adc_raw_5v * current_vdda / 4095.0f;
        current_5v = adc_voltage * DIV_5V_FACTOR;
        v5_error = 0;
    } else {
        current_5v = 5.0f;
        v5_error = 1;
    }
}

/* ============================================================================
 * ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ДЛЯ АЦП
 * ========================================================================== */
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

/* Системная конфигурация тактирования */
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

/* Инициализация портов ввода-вывода */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    // UART1 (ModBus RS485)
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // UART2 (Отладка)
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // TIM3_CH4 (Вход захвата - PB1 = CLIK)
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Импульсный выход (PB5 = Gen_Impuls)
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Switch_In_impuls (PB7) - управление транзисторами
    GPIO_InitStruct.Pin = SWITCH_PIN;  // PB7
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SWITCH_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(SWITCH_PORT, SWITCH_PIN, GPIO_PIN_RESET); // Начальное состояние: ВЫКЛ

    // Светодиоды
    GPIO_InitStruct.Pin = LED_RED_PIN | LED_BLUE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, LED_RED_PIN, LED_RED_OFF);
    HAL_GPIO_WritePin(GPIOB, LED_BLUE_PIN, LED_BLUE_OFF);

    // Управление направлением линии RS485
    GPIO_InitStruct.Pin = RS485_CTRL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(RS485_CTRL_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(RS485_CTRL_PORT, RS485_CTRL_PIN, GPIO_PIN_RESET);

    // I2C2 (PB10=SCL, PB11=SDA)
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* Инициализация модуля обмена данными 1 */
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

/* Инициализация модуля обмена данными 2 */
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

/* Инициализация аналого-цифрового преобразователя 1 */
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

/* Инициализация аналого-цифрового преобразователя 2 */
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

/* Обработчик ошибок */
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
