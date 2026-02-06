/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Система измерения времени пролёта магнитострикционного датчика
  *                   с измерением температуры (LM75B) и сохранением данных (AT24C02)
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "stm32f1xx_it.h"
#include "utils.h"
#include "i2c_config.h"
#include "lm75b.h"
#include "at24c02.h"
#include <stdint.h>
#include <string.h>
#include <math.h>

/* === ТАКТИРОВАНИЕ И КАЛИБРОВКА === */
#define TIMER_CLOCK_HZ  72000000.0f                 // Тактовая частота ядра после PLL
#define VREFINT_CAL_ADDR  ((uint16_t*)0x1FFFF7BA)   // Адрес заводской калибровки VREFINT при 3.3В
#define VREFINT_CAL_VALUE (*VREFINT_CAL_ADDR)       // Значение калибровки (обычно ~1260 при 3.3В)
#define ADC_SAMPLES     16                          // Количество выборок для усреднения АЦП
#define MEAS_TIMEOUT_MS 10                          // Таймаут измерения времени пролёта

/* === КОЭФФИЦИЕНТЫ ДЕЛИТЕЛЕЙ НАПРЯЖЕНИЯ ===
   Формула расчёта реального напряжения:
   V_реал = V_адц * (R1 + R2) / R2

   Где:
   - V_адц = (значение_АЦП / 4095) * VDDA
   - R1, R2 - резисторы делителя

   Подобранные экспериментально коэффициенты: */
#define DIV_24V_FACTOR  9.2f    // Делитель 24В: ~81кОм + 10кОм → коэффициент 9.1, с учётом погрешностей = 9.2
#define DIV_12V_FACTOR  4.6f    // Делитель 12В: ~35кОм + 10кОм → коэффициент 4.5, с учётом погрешностей = 4.6
#define DIV_5V_FACTOR   2.92f   // Делитель 5В: ~4.7кОм + 10кОм → теоретический коэффициент 3.13, фактический = 2.92

/* === ПАРАМЕТРЫ МАГНИТОСТРИКЦИОННОГО ДАТЧИКА === */
#define PULSE_WIDTH_US  10          // Ширина генерируемого импульса возбуждения (мкс)
#define PULSE_PERIOD_MS 10000       // Период измерений (10 секунд)
#define SOUND_SPEED_MPS 2800.0f     // Скорость звука в волноводе магнитострикционного датчика (м/с)
                                    // Зависит от материала волновода (обычно 2700-2900 м/с для железа)
#define TOF_TICK_US     (1000000.0f / (TIMER_CLOCK_HZ / 7.0f))  // Длительность тика таймера в мкс
                                    // При предделителе 6: 72 МГц / (6+1) = 10.2857 МГц → 1 тик = 0.09722 мкс

/* === УРОВНИ СИГНАЛОВ === */
#define POWER_5V_ON     GPIO_PIN_RESET  // Активный низкий уровень для питания 5В (постоянно включено)
#define PULSE_HIGH      GPIO_PIN_SET    // Высокий уровень = импульс возбуждения АКТИВЕН
#define PULSE_LOW       GPIO_PIN_RESET  // Низкий уровень = импульс возбуждения НЕ АКТИВЕН
#define LED_RED_ON      GPIO_PIN_SET    // Красный светодиод: включён = HIGH
#define LED_RED_OFF     GPIO_PIN_RESET  // Красный светодиод: выключен = LOW
#define LED_BLUE_ON     GPIO_PIN_SET    // Синий светодиод: включён = HIGH
#define LED_BLUE_OFF    GPIO_PIN_RESET  // Синий светодиод: выключен = LOW

/* === ПИНЫ СВЕТОДИОДОВ === */
#define LED_RED_PIN     GPIO_PIN_13  // PB13 = красный светодиод (горит 1 сек при захвате сигнала)
#define LED_BLUE_PIN    GPIO_PIN_12  // PB12 = синий светодиод (мигает 1 Гц - индикация работы)

#define LED_RED_ON_TIME_MS 1000  // Время горения красного светодиода после захвата сигнала
#define PULSE_DELAY_ITERATIONS 54  // Количество итераций для задержки 10 мкс при 72 МГц (-O0)
                                   // Экспериментально подобрано: 54 * ~185 нс = 10.0 мкс

/* === АДРЕСА УСТРОЙСТВ I2C2 ===
   I2C2 использует пины PB10 (SCL) и PB11 (SDA) на STM32F103C8T6 */
#define LM75B_ADDR      LM75B_DEFAULT_ADDRESS   // 0x90 (8-bit) = 0x48 (7-bit) - датчик температуры
#define AT24C02_ADDR    AT24C02_DEFAULT_ADDRESS // 0xA0 (8-bit) = 0x50 (7-bit) - EEPROM 256 байт
#define EEPROM_TEMP_ADDR 0x20                   // Адрес в EEPROM для сохранения последней температуры
#define EEPROM_BOOT_COUNT_ADDR 0x00             // Адрес для счётчика загрузок системы

/* === ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ === */
UART_HandleTypeDef huart1;  // USART1 для ModBus RTU (9600 бод)
UART_HandleTypeDef huart2;  // USART2 для отладочного вывода (115200 бод)
ADC_HandleTypeDef hadc1;    // ADC1 для измерения 24В и VREFINT
ADC_HandleTypeDef hadc2;    // ADC2 для измерения 12В и 5В

// Переменные для измерения времени пролёта (Time-of-Flight)
volatile uint32_t tof_capture_value = 0;    // Значение счётчика таймера при захвате фронта сигнала
volatile uint8_t tof_measurement_done = 0;  // Флаг завершения измерения (устанавливается в прерывании TIM3)
volatile uint8_t tof_timeout = 0;           // Флаг таймаута измерения
volatile uint8_t signal_captured = 0;       // Флаг успешного захвата сигнала на входе CLIK (PB1)

// Результаты измерений
static float current_vdda = 3.3f;    // Напряжение питания АЦП (калибруется через VREFINT)
static float current_24v = 24.0f;    // Напряжение основного питания 24В
static float current_12v = 12.0f;    // Напряжение питания 12В
static float current_5v = 5.0f;      // Напряжение питания 5В
static float current_temperature = 0.0f;  // Температура от датчика LM75B (°C)

// Флаги инициализации периферии
static uint8_t adc1_initialized = 0;
static uint8_t adc2_initialized = 0;
static uint8_t i2c_initialized = 0;      // I2C2 инициализирован
static uint8_t lm75b_initialized = 0;    // Датчик температуры обнаружен
static uint8_t at24c02_initialized = 0;  // EEPROM обнаружен

static char float_buffer[32];  // Буфер для преобразования float в строку

/* === ПРОТОТИПЫ ФУНКЦИЙ === */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
void Error_Handler(void);

void TIM3_InputCapture_Init(void);          // Инициализация таймера для захвата времени пролёта
void generate_pulse_and_measure(void);      // Генерация импульса + запуск измерения
uint32_t measure_time_of_flight(void);      // Полный цикл измерения времени пролёта
void print_tof_results(uint32_t tof_ticks, float tof_us, float position_mm, uint8_t captured);
void Read_All_Voltages(void);               // Измерение всех напряжений системы
uint32_t Read_ADC_Single(ADC_HandleTypeDef* hadc, uint32_t channel, uint32_t sampling_time);
uint32_t Read_ADC_Average(ADC_HandleTypeDef* hadc, uint32_t channel, uint32_t sampling_time, uint8_t samples);

void Read_Temperature(void);                // Чтение температуры с LM75B
void Read_Boot_Count(void);                 // Чтение и инкремент счётчика загрузок из EEPROM

/**
  * @brief  Главная функция программы
  * @retval нет
  */
int main(void)
{
    /* === ИНИЦИАЛИЗАЦИЯ МИКРОКОНТРОЛЛЕРА === */
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();

    /* === ИНИЦИАЛИЗАЦИЯ I2C2 ДЛЯ ДАТЧИКОВ ===
       Используем I2C2 (PB10=SCL, PB11=SDA) - стандартное подключение для STM32F103C8T6
       ВАЖНО: На линиях SCL/SDA должны быть подтяжки 4.7кОм к 3.3В */
    USART2_Print("Инициализация I2C2 (PB10=SCL, PB11=SDA)...\r\n");
    if (MX_I2C2_Init() == HAL_OK) {
        i2c_initialized = 1;
        USART2_Print("I2C2 готов\r\n");

        /* Инициализация датчика температуры LM75B */
        if (LM75B_Init(LM75B_ADDR) == HAL_OK) {
            lm75b_initialized = 1;
            USART2_Print("LM75B обнаружен\r\n");
        } else {
            USART2_Print("LM75B не найден\r\n");
        }

        /* Инициализация EEPROM AT24C02 */
        if (AT24C02_Init(AT24C02_ADDR) == HAL_OK) {
            at24c02_initialized = 1;
            USART2_Print("AT24C02 обнаружен\r\n");
            Read_Boot_Count();  // Инкремент счётчика загрузок
        } else {
            USART2_Print("AT24C02 не найден\r\n");
        }
    }

    /* === ИНИЦИАЛИЗАЦИЯ ЗАХВАТА ВРЕМЕНИ ПРОЛЁТА ===
       Используется TIM3, канал 4 (вход PB1 = CLIK)
       Таймер настроен на частоту 10.2857 МГц (предделитель 6 от 72 МГц) */
    TIM3_InputCapture_Init();

    /* === КАЛИБРОВКА АЦП === */
    if (HAL_ADCEx_Calibration_Start(&hadc1) == HAL_OK) {
        adc1_initialized = 1;
    }
    if (HAL_ADCEx_Calibration_Start(&hadc2) == HAL_OK) {
        adc2_initialized = 1;
    }

    /* === ИНИЦИАЛИЗАЦИЯ ПРОТОКОЛА MODBUS === */
    ModBus_Init();
    HAL_Delay(1000);

    /* === НАСТРОЙКА ПРИОРИТЕТОВ ПРЕРЫВАНИЙ ===
       Приоритеты: 0 - самый высокий, 15 - самый низкий
       Важно: прерывание таймера захвата должно иметь высокий приоритет для точного измерения */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);  // МАКСИМАЛЬНЫЙ ПРИОРИТЕТ для захвата сигнала!
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);

    __enable_irq();  // Включение глобальных прерываний

    /* === ВЫВОД ИНФОРМАЦИИ О СИСТЕМЕ === */
    USART2_Print("\r\n=== СИСТЕМА ЗАПУЩЕНА ===\r\n");
    USART2_Print("Измерения каждые 10 секунд\r\n");
    USART2_Print("PB13 - красный светодиод (сигнал захвачен)\r\n");
    USART2_Print("PB12 - синий светодиод (работа системы)\r\n");
    USART2_Print("========================\r\n\r\n");

    /* === ПЕРВОНАЧАЛЬНОЕ ИЗМЕРЕНИЕ ПАРАМЕТРОВ === */
    Read_All_Voltages();
    if (lm75b_initialized) {
        Read_Temperature();
    }

    /* === ПЕРЕМЕННЫЕ ЦИКЛА ИЗМЕРЕНИЙ === */
    uint32_t last_measure_time = 0;     // Время последнего измерения
    uint32_t last_debug_time = 0;       // Время последнего мигания синим светодиодом
    uint32_t led_red_off_time = 0;      // Время автоматического выключения красного светодиода
    uint8_t blue_led_state = 0;         // Состояние синего светодиода (0=выкл, 1=вкл)
    uint8_t red_led_state = 0;          // Состояние красного светодиода

    /* === ГЛАВНЫЙ ЦИКЛ ПРОГРАММЫ === */
    while (1)
    {
        /* Мигание синим светодиодом 1 Гц - индикация работы системы */
        if (HAL_GetTick() - last_debug_time >= 1000) {
            last_debug_time = HAL_GetTick();
            blue_led_state = !blue_led_state;
            HAL_GPIO_WritePin(GPIOB, LED_BLUE_PIN, blue_led_state ? LED_BLUE_ON : LED_BLUE_OFF);
        }

        /* Автоматическое выключение красного светодиода через 1 секунду после захвата */
        if (red_led_state && (HAL_GetTick() - led_red_off_time >= LED_RED_ON_TIME_MS)) {
            HAL_GPIO_WritePin(GPIOB, LED_RED_PIN, LED_RED_OFF);
            red_led_state = 0;
        }

        /* Цикл измерений каждые 10 секунд */
        if (HAL_GetTick() - last_measure_time >= PULSE_PERIOD_MS) {
            last_measure_time = HAL_GetTick();

            /* Сброс индикации перед новым измерением */
            HAL_GPIO_WritePin(GPIOB, LED_RED_PIN, LED_RED_OFF);
            red_led_state = 0;
            signal_captured = 0;

            /* Шаг 1: Измерение напряжений питания */
            Read_All_Voltages();
            ModBus_UpdateVoltages(current_vdda, current_24v, current_12v, current_5v);

            /* Шаг 2: Измерение температуры и сохранение в EEPROM */
            if (lm75b_initialized) {
                Read_Temperature();
                /* Сохранение температуры в EEPROM с точностью 0.1°C:
                   Значение умножается на 10 и сохраняется как 16-битное целое */
                if (at24c02_initialized) {
                    int16_t temp_raw = (int16_t)(current_temperature * 10.0f);
                    AT24C02_WriteByte(AT24C02_ADDR, EEPROM_TEMP_ADDR, (uint8_t)(temp_raw >> 8));
                    AT24C02_WriteByte(AT24C02_ADDR, EEPROM_TEMP_ADDR + 1, (uint8_t)(temp_raw & 0xFF));
                }
            }

            /* Шаг 3: Генерация импульса и измерение времени пролёта */
            uint32_t tof_ticks = measure_time_of_flight();  // Время в тиках таймера
            float tof_us = tof_ticks * TOF_TICK_US;         // Перевод в микросекунды

            /* Коррекция: вычитаем длительность генерируемого импульса (10 мкс) */
            if (tof_ticks > (10.0f / TOF_TICK_US)) {
                tof_us -= 10.0f;
            }

            /* Расчёт положения магнита в волноводе:
               Формула: положение = (время_пролёта * скорость_звука) / 2
               Деление на 2 - потому что сигнал проходит путь туда и обратно */
            float position_mm = (tof_us * 0.001f * SOUND_SPEED_MPS) / 2.0f;

            /* Шаг 4: Индикация результата захвата */
            if (tof_ticks > 0 && tof_measurement_done) {
                /* Сигнал успешно захвачен - включаем красный светодиод на 1 сек */
                HAL_GPIO_WritePin(GPIOB, LED_RED_PIN, LED_RED_ON);
                red_led_state = 1;
                led_red_off_time = HAL_GetTick();
                signal_captured = 1;
            } else {
                signal_captured = 0;
            }

            /* Обновление данных для ModBus */
            ModBus_UpdateMeasurements(tof_us, position_mm, current_temperature, signal_captured ? 1 : 0);

            /* Вывод результатов измерения */
            USART2_Print("=== ИЗМЕРЕНИЕ ===\r\n");
            USART2_Print("Напряжения: ");
            float_to_str(current_vdda, float_buffer, 2);
            USART2_Print(float_buffer);
            USART2_Print("V ");
            float_to_str(current_24v, float_buffer, 1);
            USART2_Print(float_buffer);
            USART2_Print("V ");
            float_to_str(current_12v, float_buffer, 1);
            USART2_Print(float_buffer);
            USART2_Print("V ");
            float_to_str(current_5v, float_buffer, 1);
            USART2_Print(float_buffer);
            USART2_Print("V\r\n");

            if (lm75b_initialized) {
                USART2_Print("Температура: ");
                float_to_str(current_temperature, float_buffer, 1);
                USART2_Print(float_buffer);
                USART2_Print(" C\r\n");
            }

            if (signal_captured) {
                USART2_Print("Время пролёта: ");
                float_to_str(tof_us, float_buffer, 2);
                USART2_Print(float_buffer);
                USART2_Print(" мкс\r\n");
                USART2_Print("Положение: ");
                float_to_str(position_mm, float_buffer, 1);
                USART2_Print(float_buffer);
                USART2_Print(" мм\r\n");
            } else {
                USART2_Print("Сигнал не захвачен\r\n");
            }
            USART2_Print("================\r\n");
        }

        /* Обработка ModBus запросов */
        ModBus_Process();
        HAL_Delay(1);
    }
}

/**
  * @brief  Callback приёма данных по UART
  * @param  huart: указатель на структуру UART_HandleTypeDef
  * @retval нет
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        ModBus_RxCallback(huart);  // Обработка входящих ModBus команд
    }
}

/**
  * @brief  Инициализация TIM3 для захвата времени пролёта
  * @details Используется частичное перемаппирование: PB1 = TIM3_CH4
  *          Предделитель = 6 → тактовая частота таймера = 72 МГц / 7 = 10.2857 МГц
  *          Разрешение: 1 тик = 0.09722 мкс
  */
void TIM3_InputCapture_Init(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    /* КРИТИЧЕСКИ ВАЖНО: Перемаппирование ДО настройки GPIO!
       Частичное перемаппирование: PB1 вместо PA3 для TIM3_CH4 */
    __HAL_AFIO_REMAP_TIM3_PARTIAL();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;  // PB1 как цифровой вход для сигнала CLIK
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Настройка таймера TIM3 */
    TIM3->CR1 = 0;
    TIM3->CR2 = 0;
    TIM3->PSC = 6;          // Предделитель: 72 МГц / (6+1) = 10.2857 МГц
    TIM3->ARR = 0xFFFF;     // Автоперезагрузка (максимальный период ~6.7 мс)
    TIM3->CNT = 0;          // Сброс счётчика
    TIM3->EGR = TIM_EGR_UG; // Применение настроек

    /* Настройка канала 4 для захвата по фронту */
    TIM3->CCMR2 = 0;
    TIM3->CCMR2 |= (0x1 << 12);     // CC4S = 01: вход от TI4 (PB1)
    TIM3->CCMR2 &= ~(0x3 << 10);    // IC4PSC = 00: без предделителя захвата
    TIM3->CCMR2 &= ~(0xF << 8);     // IC4F = 0000: фильтр отключён (максимальная скорость)

    TIM3->CCER = TIM_CCER_CC4E;     // Включить захват по фронту на канале 4
    TIM3->DIER = TIM_DIER_CC4IE;    // Разрешить прерывание по захвату
    TIM3->SR = 0;                   // Очистить флаги прерываний
}

/**
  * @brief  Генерация импульса возбуждения и запуск измерения
  * @details КРИТИЧЕСКИ ВАЖНО: Таймер запускается ДО генерации импульса!
  *          Это позволяет захватить очень короткие задержки (~2 мкс) отражённого сигнала.
  *          Если запустить таймер после импульса — короткие задержки будут пропущены.
  */
void generate_pulse_and_measure(void)
{
    /* Сброс флагов и счётчика перед новым измерением */
    tof_measurement_done = 0;
    tof_timeout = 0;
    tof_capture_value = 0;
    TIM3->SR = 0;
    TIM3->CNT = 0;

    /* ЗАПУСК ТАЙМЕРА ДО ГЕНЕРАЦИИ ИМПУЛЬСА (критически важно!) */
    TIM3->CR1 |= TIM_CR1_CEN;
    __NOP();  // Минимальная задержка для стабилизации

    /* Генерация импульса 10 мкс на PB5 (Gen_Impuls) - прямой доступ к регистрам для скорости */
    GPIOB->BSRR = GPIO_PIN_5;  // Установить PB5 = HIGH (начало импульса)

    /* Точная задержка 10 мкс (54 итерации = 10.0 мкс при -O0 и 72 МГц) */
    for (volatile uint32_t i = 0; i < PULSE_DELAY_ITERATIONS; i++) {
        __NOP();
    }

    GPIOB->BRR = GPIO_PIN_5;   // Сбросить PB5 = LOW (конец импульса)
    /* Таймер продолжает работать — ждём захвата отражённого сигнала в прерывании */
}

/**
  * @brief  Измерение времени пролёта отражённого сигнала
  * @retval Время в тиках таймера (0 при таймауте)
  */
uint32_t measure_time_of_flight(void)
{
    /* Генерация импульса с предварительным запуском таймера */
    generate_pulse_and_measure();

    /* Ожидание завершения измерения или таймаута (10 мс) */
    uint32_t start_wait = HAL_GetTick();
    while (!tof_measurement_done && !tof_timeout) {
        if ((HAL_GetTick() - start_wait) >= MEAS_TIMEOUT_MS) {
            tof_timeout = 1;
            TIM3->CR1 &= ~TIM_CR1_CEN;  // Остановить таймер при таймауте
            break;
        }
        __NOP();
    }

    /* Остановка таймера и сброс флагов */
    TIM3->CR1 &= ~TIM_CR1_CEN;
    TIM3->SR = 0;

    return tof_capture_value;
}

/**
  * @brief  Вывод результатов измерения (заглушка - вывод реализован в основном цикле)
  */
void print_tof_results(uint32_t tof_ticks, float tof_us, float position_mm, uint8_t captured)
{
}

/**
  * @brief  Чтение температуры с датчика LM75B
  * @details LM75B возвращает 16-битное значение:
  *          Биты 15-5: знаковое значение температуры в дополнительном коде (шаг 0.125°C)
  *          Биты 4-0: дробная часть (всегда 0 для чтения)
  *          Формат: [15:5] = целая часть * 32, [4:0] = 0
  *          Пример: 0x0190 = 0000 0001 1001 0000 = +25.0°C (0x0190 >> 5 = 25)
  */
void Read_Temperature(void)
{
    if (!lm75b_initialized || !i2c_initialized) {
        current_temperature = -127.0f;  // Специальное значение ошибки
        return;
    }

    if (LM75B_ReadTemperature(LM75B_ADDR, &current_temperature) != HAL_OK) {
        current_temperature = -127.0f;
    }
}

/**
  * @brief  Чтение и инкремент счётчика загрузок из EEPROM
  * @details Счётчик хранится в первом байте EEPROM (адрес 0x00)
  *          При каждой загрузке значение увеличивается на 1 (с переполнением через 255)
  */
void Read_Boot_Count(void)
{
    if (!at24c02_initialized || !i2c_initialized) return;

    uint8_t boot_count = 0;
    if (AT24C02_ReadByte(AT24C02_ADDR, EEPROM_BOOT_COUNT_ADDR, &boot_count) == HAL_OK) {
        boot_count++;
        AT24C02_WriteByte(AT24C02_ADDR, EEPROM_BOOT_COUNT_ADDR, boot_count);
    }
}

/**
  * @brief  Чтение одного значения АЦП
  * @param  hadc: указатель на структуру ADC_HandleTypeDef
  * @param  channel: номер канала АЦП
  * @param  sampling_time: время выборки
  * @retval Значение АЦП (0-4095)
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
  * @brief  Чтение усреднённого значения АЦП
  * @param  hadc: указатель на структуру ADC_HandleTypeDef
  * @param  channel: номер канала АЦП
  * @param  sampling_time: время выборки
  * @param  samples: количество выборок для усреднения
  * @retval Усреднённое значение АЦП (0-4095)
  */
uint32_t Read_ADC_Average(ADC_HandleTypeDef* hadc, uint32_t channel, uint32_t sampling_time, uint8_t samples)
{
    uint32_t sum = 0;
    uint8_t valid_samples = 0;

    for (uint8_t i = 0; i < samples; i++) {
        uint32_t value = Read_ADC_Single(hadc, channel, sampling_time);
        /* Фильтрация некорректных значений (менее 100 или более 4000) */
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
  * @brief  Измерение всех напряжений системы
  * @details Порядок измерения:
  *          1. Калибровка VDDA через внутренний источник VREFINT
  *          2. Измерение 24В через делитель на PA0 (ADC1)
  *          3. Измерение 12В через делитель на PA1 (ADC2)
  *          4. Измерение 5В через делитель на PA5 (ADC2)
  *
  *          Формула калибровки VDDA:
  *          VDDA = 3.3В * VREFINT_CAL / ADC_VREFINT
  *          где VREFINT_CAL - заводское значение при 3.3В (адрес 0x1FFFF7BA)
  *
  *          Формула расчёта напряжения:
  *          V_реал = (ADC_value / 4095) * VDDA * DIV_FACTOR
  */
void Read_All_Voltages(void)
{
    if (!adc1_initialized || !adc2_initialized) {
        return;
    }

    uint32_t adc_raw_vdda = 0;
    uint32_t adc_raw_24v = 0;
    uint32_t adc_raw_12v = 0;
    uint32_t adc_raw_5v = 0;

    /* === ШАГ 1: КАЛИБРОВКА VDDA ЧЕРЕЗ VREFINT ===
       Включаем внутренний источник опорного напряжения (1.20В) */
    ADC1->CR2 |= ADC_CR2_TSVREFE;
    HAL_Delay(10);
    adc_raw_vdda = Read_ADC_Average(&hadc1, ADC_CHANNEL_VREFINT, ADC_SAMPLETIME_239CYCLES_5, ADC_SAMPLES);
    ADC1->CR2 &= ~ADC_CR2_TSVREFE;
    HAL_Delay(1);

    /* Расчёт реального VDDA по формуле:
       VDDA = 3.3В * VREFINT_CAL / ADC_VREFINT */
    if (adc_raw_vdda > 1000 && adc_raw_vdda < 2000 && VREFINT_CAL_VALUE > 1000 && VREFINT_CAL_VALUE < 2000) {
        current_vdda = 3.3f * (float)VREFINT_CAL_VALUE / (float)adc_raw_vdda;
    } else {
        current_vdda = 3.3f;  // Значение по умолчанию при ошибке калибровки
    }

    /* === ШАГ 2: ИЗМЕРЕНИЕ 24В (PA0, ADC1) === */
    adc_raw_24v = Read_ADC_Average(&hadc1, ADC_CHANNEL_0, ADC_SAMPLETIME_239CYCLES_5, ADC_SAMPLES);
    if (adc_raw_24v > 100 && adc_raw_24v < 4000) {
        float adc_voltage = (float)adc_raw_24v * current_vdda / 4095.0f;
        current_24v = adc_voltage * DIV_24V_FACTOR;
    } else {
        current_24v = 24.0f;
    }

    /* === ШАГ 3: ИЗМЕРЕНИЕ 12В (PA1, ADC2) === */
    adc_raw_12v = Read_ADC_Average(&hadc2, ADC_CHANNEL_1, ADC_SAMPLETIME_239CYCLES_5, ADC_SAMPLES);
    if (adc_raw_12v > 100 && adc_raw_12v < 4000) {
        float adc_voltage = (float)adc_raw_12v * current_vdda / 4095.0f;
        current_12v = adc_voltage * DIV_12V_FACTOR;
    } else {
        current_12v = 12.0f;
    }

    /* === ШАГ 4: ИЗМЕРЕНИЕ 5В (PA5, ADC2) === */
    adc_raw_5v = Read_ADC_Average(&hadc2, ADC_CHANNEL_5, ADC_SAMPLETIME_239CYCLES_5, ADC_SAMPLES);
    if (adc_raw_5v > 100 && adc_raw_5v < 4000) {
        float adc_voltage = (float)adc_raw_5v * current_vdda / 4095.0f;
        current_5v = adc_voltage * DIV_5V_FACTOR;
    } else {
        current_5v = 5.0f;
    }
}

/**
  * @brief  Конфигурация системы тактирования
  * @details Используется внешний кварц 8 МГц с PLL:
  *          HSE = 8 МГц → PLL (×9) = 72 МГц → системная частота
  *          АПВ1 (таймеры) = 72 МГц / 2 = 36 МГц
  *          АПВ2 (АЦП) = 72 МГц / 6 = 12 МГц (макс. для АЦП STM32F1)
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
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;  // 8 МГц × 9 = 72 МГц
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;   // 72/2 = 36 МГц для таймеров
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }

    /* Тактирование АЦП: 72 МГц / 6 = 12 МГц (максимальная частота для АЦП STM32F1) */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief  Инициализация GPIO
  * @details Назначение пинов:
  *          PB1  - вход захвата сигнала CLIK (TIM3_CH4)
  *          PB5  - выход генерации импульса возбуждения
  *          PB6  - выход управления питанием 5В (активный низкий)
  *          PB10 - SCL шины I2C2
  *          PB11 - SDA шины I2C2
  *          PB12 - синий светодиод (индикация работы)
  *          PB13 - красный светодиод (индикация захвата сигнала)
  *          PA0  - вход АЦП для 24В
  *          PA1  - вход АЦП для 12В
  *          PA5  - вход АЦП для 5В
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    /* Кварц 32768 Гц на PC14/PC15 (не используется, но инициализирован для совместимости) */
    GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* Аналоговые входы АЦП */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Вход захвата сигнала CLIK (PB1 = TIM3_CH4) */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Выход генерации импульса возбуждения (PB5) */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Выход управления питанием 5В (активный низкий уровень) */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Шина I2C2: PB10=SCL, PB11=SDA (ОБЯЗАТЕЛЬНО режим Open Drain!) */
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;  // Открытый сток для шины I2C
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Светодиоды индикации */
    GPIO_InitStruct.Pin = LED_RED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, LED_RED_PIN, LED_RED_OFF);

    GPIO_InitStruct.Pin = LED_BLUE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, LED_BLUE_PIN, LED_BLUE_OFF);

    /* Включение питания 5В (низкий уровень = ВКЛ) */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, POWER_5V_ON);

    /* USART1 (ModBus): PA9=TX, PA10=RX */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 (отладка): PA2=TX, PA3=RX */
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
  * @brief  Инициализация USART1 (ModBus RTU)
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
  * @brief  Инициализация USART2 (отладочный вывод)
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
  * @brief  Инициализация ADC1 (измерение 24В и VREFINT)
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
  * @brief  Инициализация ADC2 (измерение 12В и 5В)
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
  * @brief  Обработчик критических ошибок
  * @details Мигание красным светодиодом при невозможности продолжения работы
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        HAL_GPIO_TogglePin(GPIOB, LED_RED_PIN);
        HAL_Delay(200);
    }
}
