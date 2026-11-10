/**
 * @file    main.h
 * @brief   Общие определения аппаратной платформы и измерительного тракта.
 *
 * ВАЖНО ДЛЯ НАСТРОЙКИ ФИКСАЦИИ:
 * Все физические временные границы захвата собраны в одном месте ниже.
 * Если материал волновода изменится, в первую очередь корректируются
 * CAPTURE_PAIR_INTERVAL_MIN_US и CAPTURE_PAIR_INTERVAL_MAX_US.
 */
#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* ADC */
#define Read_24V_Pin                GPIO_PIN_0
#define Read_24V_GPIO_Port          GPIOA
#define Read_12V_Pin                GPIO_PIN_1
#define Read_12V_GPIO_Port          GPIOA
#define Read_5V_Pin                 GPIO_PIN_5
#define Read_5V_GPIO_Port           GPIOA

/* Измерительный тракт */
#define CLIK_Pin                    GPIO_PIN_1
#define CLIK_GPIO_Port              GPIOB
#define Gen_Impuls_Pin              GPIO_PIN_5
#define Gen_Impuls_GPIO_Port        GPIOB
#define Switch_In_impuls_Pin        GPIO_PIN_7
#define Switch_In_impuls_GPIO_Port  GPIOB

/* Индикация */
#define LED_BLUE_Pin                GPIO_PIN_12
#define LED_BLUE_GPIO_Port          GPIOB
#define LED_RED_Pin                 GPIO_PIN_13
#define LED_RED_GPIO_Port           GPIOB

/* RS-485 */
#define RS485_CTRL_Pin              GPIO_PIN_0
#define RS485_CTRL_GPIO_Port        GPIOB

/* Дополнительные цепи */
#define Check_OPA552_Pin            GPIO_PIN_8
#define Check_OPA552_GPIO_Port      GPIOA
#define ON_VCC_5_Pin                GPIO_PIN_6
#define ON_VCC_5_GPIO_Port          GPIOB

/* ==========================================================================
 * ПАРАМЕТРЫ АППАРАТНОЙ ФИКСАЦИИ TIM3 CH4
 * ========================================================================== */

/*
 * TIM3 тактируется от 72 МГц.
 * При PSC=0 частота счётчика равна 72 МГц:
 *
 *     72 000 000 / (0 + 1) = 72 000 000 Гц.
 *
 * Один тик равен приблизительно 0,013888889 мкс, то есть 13,889 нс.
 * В TIM3_InputCapture_Init() обязательно должно быть:
 *
 *     TIM3->PSC = 0U;
 */
#define TIM3_CAPTURE_FREQUENCY_HZ   72000000UL
#define TIM3_CAPTURE_TICK_US        0.013888889f

/*
 * Постоянная задержка электронного тракта от момента запуска PB5 до
 * срабатывания цифрового входа TIM3, не относящаяся к распространению
 * крутильной волны по звукопроводу.
 *
 * В неё входят приёмная катушка, усилительные каскады, фильтры и достижение
 * порога цифрового входа STM32.
 *
 * Значение вычитается только при переводе сырого ToF в расстояние.
 * Из интервала t2-t1 оно не вычитается, поскольку одинаковая задержка
 * присутствует в обоих фронтах и взаимно сокращается.
 */
#define ELECTRONICS_DELAY_US        18.00f

/*
 * Верхняя невалидная зона звукопровода около электронной головки.
 * Значение должно соответствовать фактической конструкции звукопровода.
 */
#define UPPER_INVALID_ZONE_MM       220.00f

/*
 * Калибровка измерительного канала по пяти опорным положениям.
 *
 * Команды 01 и 02 задают реальные пределы измерения 0 % и 100 %.
 * Команды 11/12/13 задают промежуточные риски 260/520/780 мм.
 *
 * Во всех точках сохраняется сырое время ToFraw. После изменения частоты
 * TIM3 калибровку необходимо выполнить заново, поскольку сохранённые времена
 * были получены при другой дискретности таймера.
 *
 * Полная последовательность:
 *   01 -> 11 -> 12 -> 13 -> 02.
 */
#define SENSOR_CAL_LEVEL_260_MM       260.00f
#define SENSOR_CAL_LEVEL_520_MM       520.00f
#define SENSOR_CAL_LEVEL_780_MM       780.00f
#define SENSOR_CAL_POINT_LOW_BIT      0x0001U
#define SENSOR_CAL_POINT_260_BIT      0x0002U
#define SENSOR_CAL_POINT_520_BIT      0x0004U
#define SENSOR_CAL_POINT_780_BIT      0x0008U
#define SENSOR_CAL_POINT_HIGH_BIT     0x0010U
#define SENSOR_CAL_FULL_MASK          0x001FU
#define SENSOR_CAL_STORAGE_TAG        0xA500U
#define SENSOR_CAL_STORAGE_TAG_MASK   0xFF00U

/*
 * Перевод микросекунд в тики TIM3.
 *
 * При частоте 72 МГц промежуточное произведение может превышать диапазон
 * uint32_t. Например:
 *
 *     100 мкс * 72 000 000 Гц = 7 200 000 000.
 *
 * Поэтому промежуточное вычисление обязательно выполняется в uint64_t.
 */
#define CAPTURE_US_TO_TICKS_CEIL(us)                                  \
    ((uint32_t)((((uint64_t)(us) *                                    \
                  (uint64_t)TIM3_CAPTURE_FREQUENCY_HZ) +               \
                 999999ULL) / 1000000ULL))

#define CAPTURE_US_TO_TICKS_FLOOR(us)                                 \
    ((uint32_t)(((uint64_t)(us) *                                     \
                 (uint64_t)TIM3_CAPTURE_FREQUENCY_HZ) /                \
                1000000ULL))


/* Перевод наносекунд в тики и в тысячные доли тика. */
#define CAPTURE_NS_TO_TICKS_CEIL(ns)                                  \
    ((uint32_t)((((uint64_t)(ns) *                                    \
                  (uint64_t)TIM3_CAPTURE_FREQUENCY_HZ) +               \
                 999999999ULL) / 1000000000ULL))

#define CAPTURE_NS_TO_MILLI_TICKS(ns)                                 \
    ((uint32_t)((((uint64_t)(ns) *                                    \
                  (uint64_t)TIM3_CAPTURE_FREQUENCY_HZ) +               \
                 999999ULL) / 1000000ULL))

/*
 * Первые 80 мкс после задающего импульса закрыты для фиксации.
 *
 * Значение 100 мкс оказалось слишком большим: в верхней части рабочего
 * диапазона истинный первый импульс приходит примерно через 97...99 мкс и
 * отбрасывался как помеха. После этого вторичный импульс принимался за t1,
 * из-за чего точка 780 мм сохранялась неверно и участок 520...780 мм
 * получал ошибочный наклон.
 */
#define CAPTURE_BLANKING_TIME_US       80U
#define BLANKING_WINDOW_TICKS          \
    CAPTURE_US_TO_TICKS_CEIL(CAPTURE_BLANKING_TIME_US)

/*
 * Внешняя схема формирует два отдельных цифровых импульса.
 * TIM3 фиксирует нарастающий фронт каждого импульса.
 *
 * Первый захват — время первого импульса t1.
 * Второй захват — время второго импульса t2.
 * Валидность пары определяется интервалом t2 - t1.
 */
#define CAPTURE_PAIR_INTERVAL_MIN_US   12U
#define CAPTURE_PAIR_INTERVAL_MAX_US   26U
#define MIN_CLICK_WIDTH_TICKS          \
    CAPTURE_US_TO_TICKS_CEIL(CAPTURE_PAIR_INTERVAL_MIN_US)
#define MAX_CLICK_WIDTH_TICKS          \
    CAPTURE_US_TO_TICKS_FLOOR(CAPTURE_PAIR_INTERVAL_MAX_US)

/*
 * К расчётному времени прохождения по всей длине волновода добавляется запас.
 * Он допускает задержку аналогового тракта, но отсекает поздние ложные пары.
 */
#define CAPTURE_MAX_TOF_MARGIN_US      60U

/*
 * Частота возбуждения волновода установлена 20 Гц.
 * Период между фронтами PB5 не может быть меньше 50 мс.
 * При появлении звона или роста ошибок пары период можно увеличить через
 * Modbus-регистр 2088 без перекомпиляции.
 */
#define EXCITATION_FREQUENCY_HZ        20U
#define EXCITATION_PERIOD_MS           \
    (1000U / EXCITATION_FREQUENCY_HZ)

/*
 * После первого импульса второй должен появиться не позднее верхней границы
 * интервала пары. Дополнительные 5 мкс оставлены как запас.
 */
#define CAPTURE_SECOND_PULSE_GRACE_US  5U
#define CAPTURE_SECOND_PULSE_TIMEOUT_TICKS                          \
    CAPTURE_US_TO_TICKS_CEIL(CAPTURE_PAIR_INTERVAL_MAX_US +        \
                             CAPTURE_SECOND_PULSE_GRACE_US)

/*
 * Фильтр использует 20 последовательных запусков. При частоте 20 Гц
 * длительность окна равна примерно одной секунде: шум уменьшается, а
 * запаздывание при движении компенсируется линейной аппроксимацией.
 */
#define MEASUREMENT_REQUIRED_SAMPLES   20U

/* Статус быстрого снимка измерений */
#define MEASUREMENT_STATUS_VALID         0x0001U
#define MEASUREMENT_STATUS_SINGLE_PULSE  0x0002U
#define MEASUREMENT_STATUS_COIL_FAULT    0x0004U

/* Коды ошибок Modbus-регистра 2416 */
#define MEASUREMENT_ERROR_NONE           0x0000U
#define MEASUREMENT_ERROR_CAPTURE_COIL   0x0101U

/*
 * Грубая защита от переключения на другой импульс. Точный отбор выбросов
 * выполняется по медиане и MAD внутри MeasurementStatistics_Calculate().
 */
#define MEASUREMENT_MAX_SPREAD_US        4U
#define MEASUREMENT_MAX_SPREAD_TICKS     \
    CAPTURE_US_TO_TICKS_CEIL(MEASUREMENT_MAX_SPREAD_US)

/*
 * Минимальный порог отклонения от медианы — 125 нс, то есть 9 тиков TIM3
 * при 72 МГц. Если MAD больше, фактический порог равен 4*MAD.
 */
#define MEASUREMENT_OUTLIER_MIN_GATE_NS  125U
#define MEASUREMENT_OUTLIER_MIN_GATE_TICKS \
    CAPTURE_NS_TO_TICKS_CEIL(MEASUREMENT_OUTLIER_MIN_GATE_NS)

/*
 * Адаптивная линейная аппроксимация.
 * При тренде меньше 10 нс/отсчёт публикуется робастное среднее.
 * При тренде больше 50 нс/отсчёт публикуется оценка на конце окна.
 * Между границами выполняется плавное смешивание результатов.
 */
#define MEASUREMENT_APPROX_SLOPE_LOW_NS_PER_SAMPLE   10U
#define MEASUREMENT_APPROX_SLOPE_HIGH_NS_PER_SAMPLE  50U
#define MEASUREMENT_APPROX_SLOPE_LOW_MILLI_TICKS \
    CAPTURE_NS_TO_MILLI_TICKS( \
        MEASUREMENT_APPROX_SLOPE_LOW_NS_PER_SAMPLE)
#define MEASUREMENT_APPROX_SLOPE_HIGH_MILLI_TICKS \
    CAPTURE_NS_TO_MILLI_TICKS( \
        MEASUREMENT_APPROX_SLOPE_HIGH_NS_PER_SAMPLE)

/*
 * Последняя защита готового уровня. После робастного окна скачок более
 * 0,45 мм должен подтвердиться тремя последовательными результатами.
 */
#define LEVEL_JUMP_CONFIRM_THRESHOLD_MM  0.45f
#define LEVEL_JUMP_CONFIRM_TOLERANCE_MM  0.20f
#define LEVEL_JUMP_CONFIRM_COUNT         3U

#define MAX_PULSE_PAIRS                  1U
#define MAX_CAPTURED_PULSES              (MAX_PULSE_PAIRS * 2U)

/* ==========================================================================
 * ГЛОБАЛЬНЫЕ ОБЪЕКТЫ
 * ========================================================================== */

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

extern volatile uint32_t captured_pulses[MAX_CAPTURED_PULSES];
extern volatile uint8_t capture_count;
extern volatile uint8_t tof_measurement_done;
extern volatile uint8_t tof_timeout;

/*
 * Верхняя допустимая граница времени первого сформированного импульса.
 * Значение рассчитывается перед измерением по длине волновода и скорости.
 */
extern volatile uint32_t capture_max_tof_ticks;

/* ==========================================================================
 * ПРОТОТИПЫ
 * ========================================================================== */

void Error_Handler(void);
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_ADC1_Init(void);
void MX_ADC2_Init(void);
HAL_StatusTypeDef MX_I2C2_Init(void);

void TIM3_InputCapture_Init(void);
void generate_pulse_and_measure(void);
uint32_t measure_time_of_flight(void);
uint32_t measure_time_of_flight_test(void);

void Read_All_Voltages(void);
void Read_Temperature(void);

void Process_Measurement_Results(float tof_us,
                                 float position_mm,
                                 uint8_t signal_was_captured);

/**
 * @brief Выполняет команду управления.
 *
 * Для команд 01, 02 и 223 итоговый код 90 устанавливается только после
 * подтверждённой записи EEPROM.
 */
void Process_Calibration_Command(uint16_t command);

uint32_t Read_ADC_Single(ADC_HandleTypeDef *hadc,
                         uint32_t channel,
                         uint32_t sampling_time);

uint32_t Read_ADC_Average(ADC_HandleTypeDef *hadc,
                          uint32_t channel,
                          uint32_t sampling_time,
                          uint8_t samples);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */
