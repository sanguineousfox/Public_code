/**
@file           : main.c
@brief          : Основной цикл магнитострикционного уровнемера
*/
#include "main.h"
#include "stm32f1xx_it.h"
#include "utils.h"
#include "i2c_config.h"
#include "modbus.h"
#include "temp_sensors.h"
#include "graduation.h"
#include "measurement_statistics.h"
#include "measurement_snapshot.h"
#include <stdint.h>
#include <string.h>
#include <math.h>

/* ==========================================================================
КОНСТАНТЫ И МАКРОСЫ
========================================================================== */
#define TIMER_CLOCK_HZ          72000000.0f
#define TOF_TICK_US             TIM3_CAPTURE_TICK_US
#define VREFINT_CAL_ADDR        0x1FFFF7BA
#define VREFINT_CAL_VALUE       ((uint16_t *)VREFINT_CAL_ADDR)
#define ADC_SAMPLES             16
#define MEAS_TIMEOUT_MIN_MS     3U
#define MEAS_TIMEOUT_MAX_MS     20U
#define MEAS_TIMEOUT_MARGIN_MS  2.0f
#define DIV_24V_FACTOR          9.4f
#define DIV_12V_FACTOR          4.0f
#define DIV_5V_FACTOR           2.0f
#define PULSE_PERIOD_MS_DEFAULT 100
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
#define FIRMWARE_VERSION        116
#define CALIBRATION_SAMPLE_MAX_AGE_MS 500U
#define MIN_POLL_PERIOD_MS      EXCITATION_PERIOD_MS
#define MAX_POLL_PERIOD_MS      60000
#define EEPROM_MEASUREMENT_GUARD_MS 5U

/*
 * Результат одного физического запуска. Время первого импульса всегда
 * пригодно для расчета ToF. Второй импульс используется как аппаратное
 * подтверждение исправности катушки фиксации и качества сформированной пары.
 */
typedef enum {
    LAUNCH_NO_INPUT_PULSE = 0,
    LAUNCH_VALID_PAIR,
    LAUNCH_SINGLE_PULSE_NO_SECOND,
    LAUNCH_SINGLE_PULSE_BAD_INTERVAL
} LaunchQuality_t;

typedef struct {
    uint32_t tof_ticks;
    LaunchQuality_t quality;
} LaunchResult_t;

/* ==========================================================================
ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ
========================================================================== */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
volatile uint8_t tof_measurement_done = 0;
volatile uint8_t tof_timeout = 0;
volatile uint8_t signal_captured = 0;
volatile uint32_t captured_pulses[MAX_CAPTURED_PULSES];
volatile uint8_t capture_count = 0;
/* Динамическая верхняя граница первого сформированного импульса. ISR использует ее для
 * немедленного отсечения физически невозможных поздних импульсов. */
volatile uint32_t capture_max_tof_ticks = 0xFFFFU;
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

/*
 * Состояние генератора и скользящего статистического окна.
 * last_excitation_time_ms гарантирует не более 10 задающих импульсов в секунду.
 * measurement_pair_fallback[] хранит признак работы по одному импульсу для
 * каждого элемента окна. Авария снимается только после 11 последовательных
 * запусков с полноценной парой, поэтому сообщение не мигает при единичных сбоях.
 */
static uint32_t last_excitation_time_ms = 0U;
static uint8_t excitation_time_initialized = 0U;
static uint32_t measurement_window[MEASUREMENT_REQUIRED_SAMPLES] = {0U};
static uint8_t measurement_pair_fallback[MEASUREMENT_REQUIRED_SAMPLES] = {0U};
static uint8_t measurement_window_count = 0U;
static uint8_t measurement_window_index = 0U;
static uint8_t measurement_fallback_count = 0U;
static uint8_t capture_coil_fault_active = 0U;
static uint16_t current_measurement_status = MEASUREMENT_STATUS_VALID;

/*
 * Активный режим таблицы калибровки:
 * 0 — таблица не используется;
 * 3 — используются точки 260/520/780 мм;
 * 5 — используются границы 0/100 % и три промежуточные точки.
 *
 * Значение обновляется внутри InterpolateCalibratedLevel() и применяется
 * только для диагностического вывода USART2.
 */
static uint8_t last_sensor_cal_mode = 0U;

/* Последнее стабильное значение, опубликованное штатным 10-Гц циклом.
 * Калибровочные команды не запускают отдельный импульс: они сохраняют именно
 * этот отфильтрованный результат. Это исключает лишний запуск вне периода
 * 100 мс и не позволяет записать точку по переходному или звенящему отклику. */
static uint32_t last_stable_tof_ticks = 0U;
static uint32_t last_stable_tof_time_ms = 0U;
static uint8_t last_stable_tof_valid = 0U;

/*
 * Команды 01/02 и команда 223 завершаются кодом 90 только после
 * подтвержденной записи требуемой редакции параметров в AT24C64. Пока идет
 * фоновая запись, адрес 3000 остается 85.
 */
static uint8_t command_save_pending = 0U;
static uint16_t command_save_code = 0U;

/* ==========================================================================
ПРОТОТИПЫ СТАТИЧЕСКИХ ФУНКЦИЙ
========================================================================== */
static void Check_Voltage_Change(const char *name, float new_val, float old_val, float *store_val);
static void Update_Poll_Period_From_Modbus(void);
static uint32_t GetMeasurementTimeoutMs(void);
static void PrepareCaptureWindow(void);
static void WaitForNextExcitationSlot(void);
static LaunchResult_t MeasureSingleLaunch(void);
static void UpdateCaptureCoilFault(uint8_t fault_active);
static void AddMeasurementToWindow(uint32_t tof_ticks, uint8_t used_single_pulse);
static void StartCommandSave(uint16_t command);
static void ProcessCommandSaveResult(void);
static float CorrectTofForElectronicsDelay(float raw_tof_us);
static float DistanceFromRawTofMm(float raw_tof_us);
static float GetUsableLevelHeightMm(void);
static uint8_t CalculateWaveSpeedFromThreePoints(float *speed_m_s);
static float InterpolateCalibratedLevel(float raw_tof_us);
static uint8_t GetStableCalibrationSample(float *tof_us, float *distance_mm);
static uint8_t RestoreThreePointCalibrationState(void);

/* ==========================================================================
ФИЛЬТРАЦИЯ И РАСЧЕТЫ
========================================================================== */
/**
 * @brief Удаляет из сырого времени измерения постоянную задержку электроники.
 *
 * Таймер TIM3 считает время от задающего импульса PB5 до цифрового фронта
 * после всей приемной цепи. Поэтому сырой ToF содержит две составляющие:
 *
 *     raw_tof = propagation_tof + electronics_delay.
 *
 * Проверка пары t2-t1 выполняется раньше в ISR и этой коррекции не требует:
 * одинаковая задержка присутствует в обоих фронтах и взаимно сокращается.
 */
static float CorrectTofForElectronicsDelay(float raw_tof_us)
{
    if (!isfinite(raw_tof_us) || raw_tof_us <= ELECTRONICS_DELAY_US) {
        return 0.0f;
    }

    return raw_tof_us - ELECTRONICS_DELAY_US;
}

/**
 * @brief Переводит сырой ToF в физическое расстояние до магнита.
 *
 * При единицах м/с и мкс произведение c * t / 1000 дает миллиметры.
 * Скорость берется из Modbus 2094...2095 и может быть откалибрована без
 * перекомпиляции. Постоянная задержка электроники удаляется до умножения.
 */
static float DistanceFromRawTofMm(float raw_tof_us)
{
    float propagation_tof_us = CorrectTofForElectronicsDelay(raw_tof_us);

    if (propagation_tof_us <= 0.0f) {
        return 0.0f;
    }

    return propagation_tof_us * 0.001f * ModBus_GetMaterialWaveSpeed();
}

/**
 * @brief Возвращает полезную измеряемую высоту уровня от дна.
 *
 * Общая длина звукопровода включает верхнюю невалидную зону около
 * электронной головки. Эта зона не является частью рабочего диапазона:
 * магнит на её границе соответствует максимальному уровню, а не полной
 * геометрической длине звукопровода.
 */
static float GetUsableLevelHeightMm(void)
{
    float waveguide_mm = ModBus_GetWaveguideLength() * 1000.0f;

    if (!isfinite(waveguide_mm) || waveguide_mm <= UPPER_INVALID_ZONE_MM) {
        return 0.0f;
    }

    return waveguide_mm - UPPER_INVALID_ZONE_MM;
}

/**
 * @brief Восстанавливает состояние трёхточечной калибровки после загрузки EEPROM.
 *
 * В ранних версиях точки 2100/2102/2104 могли быть успешно сохранены, а
 * служебная маска 2106 — отсутствовать или не содержать тег 0xA500. Тогда
 * значения ToFraw переживали перезапуск, но основной расчёт переходил в
 * CAL=FALLBACK.
 *
 * Восстановление выполняется только когда служебного тега вообще нет. Если
 * тег присутствует, но биты точек сброшены командой 01, это считается
 * намеренным началом нового цикла и старые точки не активируются.
 *
 * @return 1, если маска была восстановлена в RAM; иначе 0.
 */
static uint8_t RestoreThreePointCalibrationState(void)
{
    const uint16_t three_point_mask =
        SENSOR_CAL_POINT_260_BIT |
        SENSOR_CAL_POINT_520_BIT |
        SENSOR_CAL_POINT_780_BIT;
    uint16_t stored_mask =
        ModBus_GetParameter_Int(MB_ADDR_SENSOR_CAL_MASK);
    float tof_260_us;
    float tof_520_us;
    float tof_780_us;
    uint16_t normalized_mask;

    /* Тег уже присутствует: состояние маски считается осознанным и не
     * реконструируется по оставшимся в EEPROM значениям. */
    if ((stored_mask & SENSOR_CAL_STORAGE_TAG_MASK) == SENSOR_CAL_STORAGE_TAG) {
        return 0U;
    }

    tof_260_us = ModBus_GetParameter_Float(MB_ADDR_SENSOR_CAL_260);
    tof_520_us = ModBus_GetParameter_Float(MB_ADDR_SENSOR_CAL_520);
    tof_780_us = ModBus_GetParameter_Float(MB_ADDR_SENSOR_CAL_780);

    if (!isfinite(tof_260_us) || !isfinite(tof_520_us) ||
        !isfinite(tof_780_us) ||
        !(tof_260_us > tof_520_us && tof_520_us > tof_780_us) ||
        tof_780_us <= 0.0f) {
        return 0U;
    }

    normalized_mask = (uint16_t)(SENSOR_CAL_STORAGE_TAG |
                                 three_point_mask |
                                 (stored_mask &
                                  (SENSOR_CAL_POINT_LOW_BIT |
                                   SENSOR_CAL_POINT_HIGH_BIT)));
    ModBus_SetParameter_Int(MB_ADDR_SENSOR_CAL_MASK, normalized_mask);
    return 1U;
}

/**
 * @brief Рассчитывает фактическую скорость волны по точкам 260/520/780 мм.
 *
 * В EEPROM сохраняются только исходные калибровочные времена ToFraw.
 * Рассчитанная скорость ни в EEPROM, ни в Modbus 2094...2095 не записывается:
 * функция заново вычисляет её для диагностического вывода USART2.
 *
 * Для каждой контрольной высоты известна координата магнита от электроники:
 *
 *     distance_i = waveguide_length - level_i.
 *
 * Постоянная задержка усилителей присутствует во всех трёх ToFraw одинаково
 * и поэтому не влияет на наклон зависимости distance(ToF). Скорость находится
 * методом наименьших квадратов по всем трём точкам, а не по одной паре.
 */
static uint8_t CalculateWaveSpeedFromThreePoints(float *speed_m_s)
{
    const uint16_t required_bits =
        SENSOR_CAL_POINT_260_BIT |
        SENSOR_CAL_POINT_520_BIT |
        SENSOR_CAL_POINT_780_BIT;
    uint16_t stored_mask;
    float tof_us[3];
    float distance_mm[3];
    float waveguide_mm;
    float tof_mean;
    float distance_mean;
    float covariance = 0.0f;
    float variance = 0.0f;
    float slope_mm_per_us;
    float calculated_speed_m_s;
    uint8_t i;

    if (speed_m_s == NULL) {
        return 0U;
    }

    stored_mask = ModBus_GetParameter_Int(MB_ADDR_SENSOR_CAL_MASK);
    /* Три точки являются достаточным признаком готовой временной калибровки.
     * Проверка не зависит от служебного тега: так читаются данные ранней
     * версии, где точки уже сохранялись, а маска могла быть записана без
     * старшего байта 0xA5. После старта RestoreThreePointCalibrationState()
     * нормализует такую маску в RAM. */
    if ((stored_mask & required_bits) != required_bits) {
        return 0U;
    }

    tof_us[0] = ModBus_GetParameter_Float(MB_ADDR_SENSOR_CAL_260);
    tof_us[1] = ModBus_GetParameter_Float(MB_ADDR_SENSOR_CAL_520);
    tof_us[2] = ModBus_GetParameter_Float(MB_ADDR_SENSOR_CAL_780);
    waveguide_mm = ModBus_GetWaveguideLength() * 1000.0f;

    if (!isfinite(waveguide_mm) || waveguide_mm <= SENSOR_CAL_LEVEL_780_MM ||
        !isfinite(tof_us[0]) || !isfinite(tof_us[1]) ||
        !isfinite(tof_us[2]) ||
        !(tof_us[0] > tof_us[1] && tof_us[1] > tof_us[2])) {
        return 0U;
    }

    distance_mm[0] = waveguide_mm - SENSOR_CAL_LEVEL_260_MM;
    distance_mm[1] = waveguide_mm - SENSOR_CAL_LEVEL_520_MM;
    distance_mm[2] = waveguide_mm - SENSOR_CAL_LEVEL_780_MM;

    tof_mean = (tof_us[0] + tof_us[1] + tof_us[2]) / 3.0f;
    distance_mean =
        (distance_mm[0] + distance_mm[1] + distance_mm[2]) / 3.0f;

    for (i = 0U; i < 3U; ++i) {
        float dt = tof_us[i] - tof_mean;
        float dd = distance_mm[i] - distance_mean;
        covariance += dt * dd;
        variance += dt * dt;
    }

    if (!isfinite(covariance) || !isfinite(variance) || variance <= 0.000001f) {
        return 0U;
    }

    slope_mm_per_us = covariance / variance;
    calculated_speed_m_s = slope_mm_per_us * 1000.0f;

    /* Широкие границы защищают USART от NaN/мусора, не навязывая марку
     * сплава. Реальное диагностическое значение ожидается около 2...4 км/с. */
    if (!isfinite(calculated_speed_m_s) ||
        calculated_speed_m_s < 500.0f ||
        calculated_speed_m_s > 10000.0f) {
        return 0U;
    }

    *speed_m_s = calculated_speed_m_s;
    return 1U;
}

/**
 * @brief Рассчитывает уровень по двум пределам и трём промежуточным точкам.
 *
 * Опорные точки:
 *   команда 01 -> 0 % (h_low);
 *   команда 11 -> 260 мм;
 *   команда 12 -> 520 мм;
 *   команда 13 -> 780 мм;
 *   команда 02 -> 100 % (h_high, но не выше полезной длины звукопровода).
 *
 * При наличии всех пяти точек между соседними значениями выполняется
 * кусочно-линейная интерполяция, а за пределами 0/100 % результат насыщается.
 *
 * Если после перезапуска доступны только сохранённые команды 11/12/13,
 * включается трёхточечный режим: внутри 260...780 мм выполняется интерполяция,
 * а снаружи — продолжение ближайшего участка с ограничением физическим
 * диапазоном звукопровода. Поэтому маска 0x0E/0x0F уже активирует CAL=3P.
 */
static float InterpolateCalibratedLevel(float raw_tof_us)
{
    const uint16_t three_point_mask =
        SENSOR_CAL_POINT_260_BIT |
        SENSOR_CAL_POINT_520_BIT |
        SENSOR_CAL_POINT_780_BIT;
    uint16_t stored_mask = ModBus_GetParameter_Int(MB_ADDR_SENSOR_CAL_MASK);
    float tof_low_us;
    float tof_260_us;
    float tof_520_us;
    float tof_780_us;
    float tof_high_us;
    float level_low_mm;
    float level_high_mm;
    float usable_height_mm;
    float x0;
    float x1;
    float y0;
    float y1;
    float result_mm;
    uint8_t full_table_valid = 0U;

    last_sensor_cal_mode = 0U;

    /* Для основного временного режима достаточно точек 260/520/780 мм.
     * Команды 01 и 02 задают штатные границы 0/100 %, но их отсутствие
     * не должно блокировать восстановление трёхточечной таблицы после
     * перезагрузки. */
    /* Для CAL=3P достаточно наличия трёх битов 260/520/780.
     * Служебный тег 0xA5 проверяется и восстанавливается при старте, но его
     * отсутствие само по себе больше не блокирует уже сохранённые точки. */
    if ((stored_mask & three_point_mask) != three_point_mask) {
        return -1.0f;
    }

    tof_260_us = ModBus_GetParameter_Float(MB_ADDR_SENSOR_CAL_260);
    tof_520_us = ModBus_GetParameter_Float(MB_ADDR_SENSOR_CAL_520);
    tof_780_us = ModBus_GetParameter_Float(MB_ADDR_SENSOR_CAL_780);
    usable_height_mm = GetUsableLevelHeightMm();

    if (!isfinite(raw_tof_us) || raw_tof_us <= 0.0f ||
        !isfinite(tof_260_us) || !isfinite(tof_520_us) ||
        !isfinite(tof_780_us) ||
        !(tof_260_us > tof_520_us && tof_520_us > tof_780_us) ||
        !isfinite(usable_height_mm) || usable_height_mm <= 0.0f) {
        return -1.0f;
    }

    /* Если выполнены также команды 01 и 02 и все пять точек монотонны,
     * используем полную таблицу с жёсткими границами 0 и 100 %. */
    if ((stored_mask & SENSOR_CAL_STORAGE_TAG_MASK) == SENSOR_CAL_STORAGE_TAG &&
        (stored_mask & SENSOR_CAL_FULL_MASK) == SENSOR_CAL_FULL_MASK) {
        tof_low_us = ModBus_GetParameter_Float(MB_ADDR_SENSOR_CAL_LOW_TOF);
        tof_high_us = ModBus_GetParameter_Float(MB_ADDR_SENSOR_CAL_HIGH_TOF);
        level_low_mm =
            ModBus_GetParameter_Float(MB_ADDR_CAL_LOW_LVL) * 1000.0f;
        level_high_mm =
            ModBus_GetParameter_Float(MB_ADDR_CAL_HIGH_LVL) * 1000.0f;

        if (!isfinite(level_low_mm) || level_low_mm < 0.0f) {
            level_low_mm = 0.0f;
        }
        if (!isfinite(level_high_mm) || level_high_mm <= level_low_mm ||
            level_high_mm > usable_height_mm) {
            level_high_mm = usable_height_mm;
        }

        if (isfinite(tof_low_us) && isfinite(tof_high_us) &&
            tof_low_us > tof_260_us &&
            tof_780_us > tof_high_us &&
            level_low_mm < SENSOR_CAL_LEVEL_260_MM &&
            SENSOR_CAL_LEVEL_780_MM < level_high_mm) {
            full_table_valid = 1U;
        }
    }

    if (full_table_valid != 0U) {
        last_sensor_cal_mode = 5U;

        if (raw_tof_us >= tof_low_us) {
            return level_low_mm;
        }
        if (raw_tof_us <= tof_high_us) {
            return level_high_mm;
        }

        if (raw_tof_us >= tof_260_us) {
            x0 = tof_low_us;
            x1 = tof_260_us;
            y0 = level_low_mm;
            y1 = SENSOR_CAL_LEVEL_260_MM;
        } else if (raw_tof_us >= tof_520_us) {
            x0 = tof_260_us;
            x1 = tof_520_us;
            y0 = SENSOR_CAL_LEVEL_260_MM;
            y1 = SENSOR_CAL_LEVEL_520_MM;
        } else if (raw_tof_us >= tof_780_us) {
            x0 = tof_520_us;
            x1 = tof_780_us;
            y0 = SENSOR_CAL_LEVEL_520_MM;
            y1 = SENSOR_CAL_LEVEL_780_MM;
        } else {
            x0 = tof_780_us;
            x1 = tof_high_us;
            y0 = SENSOR_CAL_LEVEL_780_MM;
            y1 = level_high_mm;
        }
    } else {
        /* Трёхточечный режим. Между точками выполняется интерполяция,
         * ниже 260 и выше 780 мм — продолжение ближайшего участка. Итог
         * обязательно ограничивается физическим диапазоном звукопровода. */
        last_sensor_cal_mode = 3U;

        if (raw_tof_us >= tof_520_us) {
            x0 = tof_260_us;
            x1 = tof_520_us;
            y0 = SENSOR_CAL_LEVEL_260_MM;
            y1 = SENSOR_CAL_LEVEL_520_MM;
        } else {
            x0 = tof_520_us;
            x1 = tof_780_us;
            y0 = SENSOR_CAL_LEVEL_520_MM;
            y1 = SENSOR_CAL_LEVEL_780_MM;
        }
    }

    if (x0 <= x1) {
        last_sensor_cal_mode = 0U;
        return -1.0f;
    }

    result_mm = y0 + (x0 - raw_tof_us) * (y1 - y0) / (x0 - x1);

    if (!isfinite(result_mm)) {
        last_sensor_cal_mode = 0U;
        return -1.0f;
    }
    if (result_mm < 0.0f) {
        result_mm = 0.0f;
    }
    if (result_mm > usable_height_mm) {
        result_mm = usable_height_mm;
    }

    return result_mm;
}

/**
 * @brief Возвращает последнее стабильное измерение для записи точки калибровки.
 *
 * Калибровочная команда не должна сама вызывать measure_time_of_flight():
 * такой вызов формирует дополнительный задающий импульс вне штатного периода
 * 100 мс и может сохранить переходный отклик. Здесь используется ровно тот
 * отфильтрованный ToF, по которому уже рассчитаны LEVEL и DistTop в консоли.
 *
 * Значение считается пригодным только пока оно свежее. Если после перемещения
 * магнита статистическое окно ещё не стабилизировалось, новые валидные данные
 * не публикуются, возраст превышает предел и команда завершается ошибкой.
 */
static uint8_t GetStableCalibrationSample(float *tof_us, float *distance_mm)
{
    uint32_t age_ms;
    float local_tof_us;
    float local_distance_mm;

    if (tof_us == NULL || distance_mm == NULL || last_stable_tof_valid == 0U) {
        return 0U;
    }

    age_ms = (uint32_t)(HAL_GetTick() - last_stable_tof_time_ms);
    if (age_ms > CALIBRATION_SAMPLE_MAX_AGE_MS || last_stable_tof_ticks == 0U) {
        return 0U;
    }

    local_tof_us = (float)last_stable_tof_ticks * TOF_TICK_US;
    local_distance_mm = DistanceFromRawTofMm(local_tof_us);
    if (!isfinite(local_tof_us) || !isfinite(local_distance_mm) ||
        local_tof_us <= 0.0f || local_distance_mm <= 0.0f) {
        return 0U;
    }

    *tof_us = local_tof_us;
    *distance_mm = local_distance_mm;
    return 1U;
}

/**
 * @brief Переводит время пролёта в уровень жидкости, отсчитанный от дна.
 *
 * Приоритет расчета:
 * 1. Полная таблица ToFraw: 0 %, 260, 520, 780 мм и 100 %.
 * 2. Трёхточечная таблица ToFraw: 260, 520 и 780 мм.
 * 3. Штатная двухточечная калибровка 0%/100% по C1/C2.
 * 4. Геометрический расчёт по скорости и длине звукопровода.
 */
static float Calculate_Position(float raw_tof_us)
{
    float h_low_m = ModBus_GetParameter_Float(MB_ADDR_CAL_LOW_LVL);
    float h_high_m = ModBus_GetParameter_Float(MB_ADDR_CAL_HIGH_LVL);
    float C1_mm = ModBus_GetParameter_Float(MB_ADDR_CAL_C1);
    float C2_mm = ModBus_GetParameter_Float(MB_ADDR_CAL_C2);
    float measured_distance_mm = DistanceFromRawTofMm(raw_tof_us);
    float usable_height_mm = GetUsableLevelHeightMm();
    float configured_high_mm = h_high_m * 1000.0f;
    float effective_high_mm;
    float level_mm;

    effective_high_mm = configured_high_mm;
    if (!isfinite(effective_high_mm) || effective_high_mm <= 0.0f ||
        effective_high_mm > usable_height_mm) {
        effective_high_mm = usable_height_mm;
    }

    level_mm = InterpolateCalibratedLevel(raw_tof_us);
    if (level_mm >= 0.0f) {
        if (level_mm > usable_height_mm) {
            level_mm = usable_height_mm;
        }
        return level_mm;
    }

    if (C1_mm > 0.0f && C2_mm > 0.0f && C1_mm > C2_mm &&
        effective_high_mm > h_low_m * 1000.0f) {
        level_mm =
            h_low_m * 1000.0f +
            (C1_mm - measured_distance_mm) *
            (effective_high_mm - h_low_m * 1000.0f) /
            (C1_mm - C2_mm);

        if (level_mm < h_low_m * 1000.0f) {
            level_mm = h_low_m * 1000.0f;
        }
        if (level_mm > effective_high_mm) {
            level_mm = effective_high_mm;
        }
        return level_mm;
    }

    level_mm =
        ModBus_GetWaveguideLength() * 1000.0f -
        measured_distance_mm;

    if (level_mm < 0.0f) {
        level_mm = 0.0f;
    }
    if (level_mm > usable_height_mm) {
        level_mm = usable_height_mm;
    }

    return level_mm;
}


/**
 * @brief Запускает обязательное сохранение текущей команды.
 *
 * Для 01/02 и 11/12/13 новые точки калибровки к этому моменту уже записаны в RAM и помечены как
 * persistent. Для 223 сохраняется текущая пользовательская конфигурация.
 * Регистр 3000 остается равным 85 до подтверждения записи AT24C64.
 */
static void StartCommandSave(uint16_t command)
{
    command_save_pending = 1U;
    command_save_code = command;
    ModBus_ForceSaveToEEPROM();
}

/**
 * @brief Завершает команду по фактическому результату EEPROM.
 *
 * COMPLETE -> 3000=90: новые коэффициенты переживут перезапуск питания.
 * ERROR    -> 3000=0: измерение прошло, но калибровку нельзя считать
 *             завершенной, потому что EEPROM не подтвердила запись.
 * PENDING/BUSY -> 3000 остается 85.
 */
static void ProcessCommandSaveResult(void)
{
    ModBus_StorageSaveStatus_t state;

    if (command_save_pending == 0U) {
        return;
    }

    state = ModBus_GetStorageSaveStatus();

    if (state == MODBUS_STORAGE_SAVE_COMPLETE) {
        ModBus_SetParameter_Int(MB_ADDR_COMMAND, 90U);
        USART2_BufInit();
        USART2_BufPrint("[CAL] Команда ");
        USART2_BufPrintInt(command_save_code);
        USART2_BufPrint(": данные сохранены в EEPROM, статус 90\r\n");
        USART2_BufFlush();

        command_save_pending = 0U;
        command_save_code = 0U;
        ModBus_ClearStorageSaveStatus();
    } else if (state == MODBUS_STORAGE_SAVE_ERROR) {
        ModBus_SetParameter_Int(MB_ADDR_COMMAND, 0U);
        USART2_BufInit();
        USART2_BufPrint("[CAL] Команда ");
        USART2_BufPrintInt(command_save_code);
        USART2_BufPrint(": ОШИБКА сохранения EEPROM, статус 0\r\n");
        USART2_BufFlush();

        command_save_pending = 0U;
        command_save_code = 0U;
        ModBus_ClearStorageSaveStatus();
    }
}

void Process_Calibration_Command(uint16_t cmd)
{
    uint8_t command_succeeded = 1U;
    uint8_t requires_eeprom_commit = 0U;

    if (cmd >= 300U && cmd <= 302U) {
        float param = ModBus_GetParameter_Float(MB_ADDR_COMMAND_PARAM);
        /* Grad_ProcessCommand() сам записывает итог 90 или 0 в адрес 3000. */
        Grad_ProcessCommand(cmd, param);
        return;
    }

    switch (cmd) {
        case 3: {
            float waveguide_len = ModBus_GetWaveguideLength();
            USART2_Print("[CAL] Команда 03: Длина звукопровода = ");
            USART2_BufInit();
            USART2_BufPrintFloat(waveguide_len * 1000.0f);
            USART2_BufPrint(" мм\r\n");
            USART2_BufFlush();
            break;
        }
        case 2: {
            float tof_us;
            float distance_mm;
            if (GetStableCalibrationSample(&tof_us, &distance_mm) != 0U) {
                uint16_t mask =
                    ModBus_GetParameter_Int(MB_ADDR_SENSOR_CAL_MASK);
                float h_high =
                    ModBus_GetParameter_Float(MB_ADDR_CAL_HIGH_LVL);
                float tof_780 =
                    ModBus_GetParameter_Float(MB_ADDR_SENSOR_CAL_780);

                if ((mask & SENSOR_CAL_STORAGE_TAG_MASK) != SENSOR_CAL_STORAGE_TAG ||
                    (mask & (SENSOR_CAL_POINT_LOW_BIT |
                             SENSOR_CAL_POINT_260_BIT |
                             SENSOR_CAL_POINT_520_BIT |
                             SENSOR_CAL_POINT_780_BIT)) !=
                            (SENSOR_CAL_POINT_LOW_BIT |
                             SENSOR_CAL_POINT_260_BIT |
                             SENSOR_CAL_POINT_520_BIT |
                             SENSOR_CAL_POINT_780_BIT)) {
                    command_succeeded = 0U;
                    USART2_Print("[CAL] Команда 02: сначала выполните 01, 11, 12 и 13\r\n");
                } else if (!isfinite(tof_780) || tof_us >= tof_780) {
                    command_succeeded = 0U;
                    USART2_BufInit();
                    USART2_BufPrint("[CAL] Команда 02: ОШИБКА монотонности, ToF100=");
                    USART2_BufPrintFloat(tof_us);
                    USART2_BufPrint(" мкс должен быть меньше ToF780=");
                    USART2_BufPrintFloat(tof_780);
                    USART2_BufPrint(" мкс. Возможен переход на ложный импульс в верхней зоне.\r\n");
                    USART2_BufFlush();
                } else {
                    ModBus_SetParameter_Float(MB_ADDR_CAL_C2, distance_mm);
                    ModBus_SetParameter_Float(MB_ADDR_SENSOR_CAL_HIGH_TOF,
                                              tof_us);
                    ModBus_SetParameter_Int(
                        MB_ADDR_SENSOR_CAL_MASK,
                        (uint16_t)(mask | SENSOR_CAL_POINT_HIGH_BIT));
                    requires_eeprom_commit = 1U;

                    USART2_BufInit();
                    USART2_BufPrint("[CAL] Команда 02: сохранена точка 100 %, ToFraw=");
                    USART2_BufPrintFloat(tof_us);
                    USART2_BufPrint(" мкс, h_high=");
                    USART2_BufPrintFloat(h_high * 1000.0f);
                    USART2_BufPrint(" мм, CalMask=31\r\n");
                    USART2_BufFlush();
                }
            } else {
                command_succeeded = 0U;
                USART2_Print("[CAL] Команда 02: нет свежего стабильного измерения; подождите стабилизации\r\n");
            }
            break;
        }
        case 1: {
            float tof_us;
            float distance_mm;
            if (GetStableCalibrationSample(&tof_us, &distance_mm) != 0U) {
                ModBus_SetParameter_Float(MB_ADDR_CAL_C1, distance_mm);
                ModBus_SetParameter_Float(MB_ADDR_SENSOR_CAL_LOW_TOF,
                                          tof_us);

                /* Команда 01 начинает новый полный цикл калибровки.
                 * Старые промежуточные и верхняя точки остаются в EEPROM,
                 * но становятся невалидными до повторного набора маски. */
                ModBus_SetParameter_Int(
                    MB_ADDR_SENSOR_CAL_MASK,
                    (uint16_t)(SENSOR_CAL_STORAGE_TAG |
                               SENSOR_CAL_POINT_LOW_BIT));
                requires_eeprom_commit = 1U;

                USART2_BufInit();
                USART2_BufPrint("[CAL] Команда 01: сохранена точка 0 %, ToFraw=");
                USART2_BufPrintFloat(tof_us);
                USART2_BufPrint(" мкс. Новый цикл, CalMask=1\r\n");
                USART2_BufFlush();
            } else {
                command_succeeded = 0U;
                USART2_Print("[CAL] Команда 01: нет свежего стабильного измерения; подождите стабилизации\r\n");
            }
            break;
        }
        case 11:
        case 12:
        case 13: {
            float tof_us;
            float distance_mm;
            if (GetStableCalibrationSample(&tof_us, &distance_mm) != 0U) {
                uint16_t mask =
                    ModBus_GetParameter_Int(MB_ADDR_SENSOR_CAL_MASK);
                uint16_t address;
                uint16_t bit;
                float reference_level_mm;
                float previous_tof_us = 0.0f;
                uint16_t required_previous_bit = 0U;

                if (cmd == 11U) {
                    address = MB_ADDR_SENSOR_CAL_260;
                    bit = SENSOR_CAL_POINT_260_BIT;
                    reference_level_mm = SENSOR_CAL_LEVEL_260_MM;

                    /* Команда 11 начинает самостоятельный цикл CAL=3P.
                     * Команда 01 для этого режима не обязательна. Старые
                     * точки 520/780 и верхняя граница становятся невалидны. */
                    mask = SENSOR_CAL_STORAGE_TAG;
                } else if (cmd == 12U) {
                    address = MB_ADDR_SENSOR_CAL_520;
                    bit = SENSOR_CAL_POINT_520_BIT;
                    reference_level_mm = SENSOR_CAL_LEVEL_520_MM;
                    previous_tof_us =
                        ModBus_GetParameter_Float(MB_ADDR_SENSOR_CAL_260);
                    required_previous_bit = SENSOR_CAL_POINT_260_BIT;

                    /* Нормализация ранней маски без тега. */
                    if ((mask & SENSOR_CAL_STORAGE_TAG_MASK) !=
                        SENSOR_CAL_STORAGE_TAG) {
                        mask = (uint16_t)(SENSOR_CAL_STORAGE_TAG |
                                          (mask & SENSOR_CAL_FULL_MASK));
                    }
                } else {
                    address = MB_ADDR_SENSOR_CAL_780;
                    bit = SENSOR_CAL_POINT_780_BIT;
                    reference_level_mm = SENSOR_CAL_LEVEL_780_MM;
                    previous_tof_us =
                        ModBus_GetParameter_Float(MB_ADDR_SENSOR_CAL_520);
                    required_previous_bit = SENSOR_CAL_POINT_520_BIT;

                    if ((mask & SENSOR_CAL_STORAGE_TAG_MASK) !=
                        SENSOR_CAL_STORAGE_TAG) {
                        mask = (uint16_t)(SENSOR_CAL_STORAGE_TAG |
                                          (mask & SENSOR_CAL_FULL_MASK));
                    }
                }

                if (required_previous_bit != 0U &&
                    (mask & required_previous_bit) == 0U) {
                    command_succeeded = 0U;
                    USART2_Print("[CAL] Нарушен порядок команд 11 -> 12 -> 13\r\n");
                    break;
                }

                if (required_previous_bit != 0U &&
                    (!isfinite(previous_tof_us) || tof_us >= previous_tof_us)) {
                    command_succeeded = 0U;
                    USART2_BufInit();
                    USART2_BufPrint("[CAL] ОШИБКА монотонности: новый ToFraw=");
                    USART2_BufPrintFloat(tof_us);
                    USART2_BufPrint(" мкс должен быть меньше предыдущего ");
                    USART2_BufPrintFloat(previous_tof_us);
                    USART2_BufPrint(" мкс\r\n");
                    USART2_BufFlush();
                    break;
                }

                ModBus_SetParameter_Float(address, tof_us);
                ModBus_SetParameter_Int(MB_ADDR_SENSOR_CAL_MASK,
                                        (uint16_t)(mask | bit));
                requires_eeprom_commit = 1U;

                USART2_BufInit();
                USART2_BufPrint("[CAL] Команда ");
                USART2_BufPrintInt(cmd);
                USART2_BufPrint(": уровень ");
                USART2_BufPrintFloat(reference_level_mm);
                USART2_BufPrint(" мм, сохранён ToFraw = ");
                USART2_BufPrintFloat(tof_us);
                USART2_BufPrint(" мкс (DistRaw = ");
                USART2_BufPrintFloat(distance_mm);
                USART2_BufPrint(" мм), CalMask=");
                USART2_BufPrintInt((uint16_t)((mask | bit) &
                                              SENSOR_CAL_FULL_MASK));
                USART2_BufPrint("\r\n");
                USART2_BufFlush();
            } else {
                command_succeeded = 0U;
                USART2_Print("[CAL] Промежуточная точка: нет свежего стабильного измерения; подождите стабилизации\r\n");
            }
            break;
        }

        case 223:
            /*
             * Таблица Е.10: сохранить пользовательские настройки.
             * Здесь не выполняется синхронная запись: адрес 3000 остается 85,
             * а итог 90/0 формируется после ответа фонового драйвера AT24C64.
             */
            requires_eeprom_commit = 1U;
            USART2_Print("[CFG] Команда 223: сохранение пользовательских настроек\r\n");
            break;

        case 4:
            USART2_Print("[CAL] Команда 04: Разность высот магнитов\r\n");
            break;
        default:
            command_succeeded = 0U;
            break;
    }

    /*
     * Для 01/11/12/13/02 и 223 код 90 нельзя выдавать до подтверждения EEPROM:
     * питание может исчезнуть во время фоновой постраничной записи. Поэтому
     * команда остается в состоянии 85 и завершается только в
     * ProcessCommandSaveResult().
     */
    if (command_succeeded == 0U) {
        ModBus_SetParameter_Int(MB_ADDR_COMMAND, 0U);
    } else if (requires_eeprom_commit != 0U) {
        ModBus_SetParameter_Int(MB_ADDR_COMMAND, 85U);
        StartCommandSave(cmd);
    } else {
        ModBus_SetParameter_Int(MB_ADDR_COMMAND, 90U);
    }
}

/* ==========================================================================
ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ПЕЧАТИ
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

static void Update_Poll_Period_From_Modbus(void)
{
    float period_value = ModBus_GetParameter_Float(MB_ADDR_POLL_PERIOD);

    if (!isfinite(period_value)) {
        period_value = (float)PULSE_PERIOD_MS_DEFAULT;
    }
    if (period_value < (float)MIN_POLL_PERIOD_MS) {
        period_value = (float)MIN_POLL_PERIOD_MS;
    }
    if (period_value > (float)MAX_POLL_PERIOD_MS) {
        period_value = (float)MAX_POLL_PERIOD_MS;
    }

    current_poll_period_ms = (uint32_t)(period_value + 0.5f);
    ModBus_SetParameter_Float(MB_ADDR_POLL_PERIOD,
                              (float)current_poll_period_ms);
}

static uint32_t GetMeasurementTimeoutMs(void)
{
    float length_m = ModBus_GetWaveguideLength();
    float speed_m_s = ModBus_GetMaterialWaveSpeed();
    float expected_ms;
    uint32_t timeout_ms;

    if (!isfinite(length_m) || length_m <= 0.0f) length_m = MODBUS_DEFAULT_WAVEGUIDE_LENGTH_M;
    if (!isfinite(speed_m_s) || speed_m_s < 100.0f) speed_m_s = MODBUS_DEFAULT_MATERIAL_WAVE_SPEED_MPS;

    expected_ms = (length_m / speed_m_s) * 1000.0f +
                  ELECTRONICS_DELAY_US / 1000.0f;
    timeout_ms = (uint32_t)ceilf(expected_ms + MEAS_TIMEOUT_MARGIN_MS);

    if (timeout_ms < MEAS_TIMEOUT_MIN_MS) timeout_ms = MEAS_TIMEOUT_MIN_MS;
    if (timeout_ms > MEAS_TIMEOUT_MAX_MS) timeout_ms = MEAS_TIMEOUT_MAX_MS;
    return timeout_ms;
}

/* ==========================================================================
ОБРАБОТКА РЕЗУЛЬТАТОВ ИЗМЕРЕНИЯ
========================================================================== */
/**
 * @brief Читает float32 непосредственно из быстрого Modbus-снимка.
 *
 * Снимок 1000..1007 хранит слова в документированном порядке ПМП-201Е:
 * младшее слово по базовому адресу, старшее — по следующему. Отладочная
 * консоль обязана показывать именно тот уровень и процент, которые реально
 * читает внешняя утилита, поэтому для печати выполняется обратная сборка
 * float32 из опубликованных слов, а не используется промежуточная локальная
 * переменная расчёта.
 *
 * @param base_address Базовый адрес float32 в диапазоне 1000..1007.
 * @param value        Указатель для результата.
 * @return true, если оба слова атомарно прочитаны из снимка.
 */
static bool ReadPublishedFloat32(uint16_t base_address, float *value)
{
    uint16_t words[2];
    uint32_t raw;

    if (value == NULL ||
        !MeasurementSnapshot_ReadRange(base_address, 2U, words)) {
        return false;
    }

    raw = (uint32_t)words[0] | ((uint32_t)words[1] << 16);
    memcpy(value, &raw, sizeof(raw));
    return isfinite(*value);
}

void Process_Measurement_Results(float tof_us,
                                 float position_mm,
                                 uint8_t signal_was_captured)
{
    float waveguide_len_m;
    float waveguide_len_mm;
    float cal_low_mm;
    float cal_high_mm;
    float level_percent = 0.0f;
    float tank_height;
    float tank_volume;
    float tank_geom_raw;
    float volume_m3 = 0.0f;
    float wave_speed;
    float calculated_wave_speed = 0.0f;
    uint8_t calculated_wave_speed_valid;
    float corrected_tof_us;
    float published_level_mm;
    float published_percent;
    float distance_from_top_raw_mm;
    float distance_from_top_cal_mm;
    float usable_height_mm;
    uint16_t sensor_cal_mask;
    uint8_t sensor_cal_mode;
    GradTankType_t tank_type = GRAD_TYPE_VERTICAL;

    if (!signal_was_captured || !isfinite(tof_us) || tof_us <= 0.0f) {
        return;
    }

    waveguide_len_m = ModBus_GetWaveguideLength();
    waveguide_len_mm = waveguide_len_m * 1000.0f;
    usable_height_mm = GetUsableLevelHeightMm();
    wave_speed = ModBus_GetMaterialWaveSpeed();
    calculated_wave_speed_valid =
        CalculateWaveSpeedFromThreePoints(&calculated_wave_speed);
    corrected_tof_us = CorrectTofForElectronicsDelay(tof_us);
    distance_from_top_raw_mm = DistanceFromRawTofMm(tof_us);
    cal_low_mm = ModBus_GetParameter_Float(MB_ADDR_CAL_LOW_LVL) * 1000.0f;
    cal_high_mm = ModBus_GetParameter_Float(MB_ADDR_CAL_HIGH_LVL) * 1000.0f;
    if (!isfinite(cal_high_mm) || cal_high_mm > usable_height_mm) {
        cal_high_mm = usable_height_mm;
    }

    if (!isfinite(position_mm) || position_mm < 0.0f) position_mm = 0.0f;
    if (position_mm > waveguide_len_mm) position_mm = waveguide_len_mm;

    /* position_mm уже является уровнем, отсчитанным от дна. Поэтому
     * процент заполнения возрастает вместе с position_mm. Повторно
     * инвертировать значение нельзя: это и было причиной показания около
     * 97 % при фактически почти пустом метровом звукопроводе. */
    if (isfinite(cal_low_mm) && isfinite(cal_high_mm) &&
        cal_high_mm > cal_low_mm) {
        level_percent =
            ((position_mm - cal_low_mm) /
             (cal_high_mm - cal_low_mm)) * 100.0f;
    } else if (usable_height_mm > 0.0f) {
        level_percent = position_mm / usable_height_mm * 100.0f;
    }

    if (level_percent < 0.0f) level_percent = 0.0f;
    if (level_percent > 100.0f) level_percent = 100.0f;

    tank_height = ModBus_GetParameter_Float(MB_ADDR_TANK_HEIGHT);
    tank_volume = ModBus_GetParameter_Float(MB_ADDR_TANK_VOLUME);
    tank_geom_raw = ModBus_GetParameter_Float(MB_ADDR_TANK_GEOM);

    if (isfinite(tank_geom_raw) && tank_geom_raw >= 0.0f &&
        tank_geom_raw <= 3.0f) {
        tank_type = (GradTankType_t)((int)tank_geom_raw);
    }

    if (tank_type == GRAD_TYPE_BY_TABLE && Grad_IsValid()) {
        volume_m3 = Grad_InterpolateVolume(position_mm / 1000.0f);
    } else {
        volume_m3 = Grad_CalculateVolume(tank_type,
                                         position_mm / 1000.0f,
                                         tank_height,
                                         tank_volume);
    }
    if (!isfinite(volume_m3) || volume_m3 < 0.0f) volume_m3 = 0.0f;

    /* Самый первый шаг — публикация готового снимка в RAM. Обработчик
     * Modbus читает 1000..1007 непосредственно из этого снимка. */
    ModBus_PublishLiveMeasurements(position_mm,
                                   current_temperature,
                                   level_percent,
                                   volume_m3,
                                   current_measurement_status);

    /* Консоль и внешняя утилита должны показывать один и тот же снимок.
     * Обычно прочитанные значения совпадают с position_mm/level_percent.
     * Если снимок временно недоступен, используем локальные значения как
     * безопасный резерв, не влияющий на Modbus и измерительный расчёт. */
    published_level_mm = position_mm;
    published_percent = level_percent;
    (void)ReadPublishedFloat32(MB_ADDR_LEVEL, &published_level_mm);
    (void)ReadPublishedFloat32(MB_ADDR_PERCENT, &published_percent);

    /* DistTopCal — удобная для линейки координата от электроники, полученная
     * из уже откалиброванного уровня. DistRaw оставлен отдельно для оценки
     * исходной модели скорости и задержки электронного тракта. */
    distance_from_top_cal_mm = waveguide_len_mm - published_level_mm;
    if (distance_from_top_cal_mm < 0.0f) distance_from_top_cal_mm = 0.0f;
    if (distance_from_top_cal_mm > waveguide_len_mm) {
        distance_from_top_cal_mm = waveguide_len_mm;
    }
    sensor_cal_mask = ModBus_GetParameter_Int(MB_ADDR_SENSOR_CAL_MASK);
    (void)InterpolateCalibratedLevel(tof_us);
    sensor_cal_mode = last_sensor_cal_mode;

    ModBus_SetParameter_Int(MB_ADDR_LEVEL_INT,
                            (uint16_t)(int16_t)position_mm);
    ModBus_SetParameter_Int(MB_ADDR_TEMP_INT,
                            (uint16_t)(int16_t)(current_temperature * 100.0f));
    ModBus_SetParameter_Int(MB_ADDR_PERCENT_INT,
                            (uint16_t)(int16_t)(level_percent * 100.0f));
    ModBus_SetParameter_Int(MB_ADDR_VOLUME_INT,
                            (uint16_t)(volume_m3 * 100.0f));

    /* Если запрос уже принят, ответ формируется до отладочного текста. */
    ModBus_Process();

    /* USART2 — только отладка. При активном запросе строка пропускается,
     * чтобы форматирование не добавляло задержку к ответу Modbus. */
    if (!ModBus_CommunicationIsBusy()) {
        USART2_BufInit();
        USART2_BufPrint("[LEVEL] ");
        USART2_BufPrintFloat(published_level_mm);
        USART2_BufPrint(" mm | ");
        USART2_BufPrintFloat(published_percent);
        USART2_BufPrint(" % | DistTopCal=");
        USART2_BufPrintFloat(distance_from_top_cal_mm);
        USART2_BufPrint(" mm | DistRaw=");
        USART2_BufPrintFloat(distance_from_top_raw_mm);
        USART2_BufPrint(" mm | CalMask=");
        USART2_BufPrintInt((uint16_t)(sensor_cal_mask & SENSOR_CAL_FULL_MASK));
        if (sensor_cal_mode == 5U) {
            USART2_BufPrint(" CAL=5P");
        } else if (sensor_cal_mode == 3U) {
            USART2_BufPrint(" CAL=3P");
        } else {
            USART2_BufPrint(" CAL=FALLBACK");
        }
        USART2_BufPrint(" | ToFraw=");
        USART2_BufPrintFloat(tof_us);
        USART2_BufPrint(" us | ToF=");
        USART2_BufPrintFloat(corrected_tof_us);
        USART2_BufPrint(" us | T=");
        USART2_BufPrintFloat(current_temperature);
        USART2_BufPrint(" C | cCalc3P=");
        if (calculated_wave_speed_valid != 0U) {
            USART2_BufPrintFloat(calculated_wave_speed);
            USART2_BufPrint(" m/s");
        } else {
            USART2_BufPrint("N/A");
        }
        USART2_BufPrint(" | cCfg=");
        USART2_BufPrintFloat(wave_speed);
        USART2_BufPrint(" m/s");
        if (capture_coil_fault_active != 0U) {
            USART2_BufPrint(" | MODE=1P COIL_FAULT");
        } else {
            USART2_BufPrint(" | MODE=PAIR");
        }
        USART2_BufPrint("\r\n");
        USART2_BufFlush();
    }
}

/* ==========================================================================
MAIN FUNCTION
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
    if (HAL_ADCEx_Calibration_Start(&hadc1) == HAL_OK)
        USART2_Print("[ИНИЦ] АЦП1 OK\r\n");
    else {
        v24_error = 1; v12_error = 1; v5_error = 1; vdda_error = 1;
    }

    if (HAL_ADCEx_Calibration_Start(&hadc2) == HAL_OK)
        USART2_Print("[ИНИЦ] АЦП2 OK\r\n");
    else {
        v12_error = 1; v5_error = 1;
    }

    ModBus_Init();

    /* Восстанавливаем старую трёхточечную таблицу, если сами ToFraw
     * загрузились из EEPROM, а служебная маска ранней версии отсутствует. */
    {
        uint8_t cal_mask_recovered = RestoreThreePointCalibrationState();
        if (cal_mask_recovered != 0U) {
            USART2_Print("[CAL] Маска 3P восстановлена по ToFraw из EEPROM\r\n");
        }
    }

    /* После включения авария катушки фиксации считается снятой. Она будет
     * установлена при первом запуске без корректного второго импульса. */
    ModBus_SetParameter_Int(MB_ADDR_ERROR_CODE, MEASUREMENT_ERROR_NONE);
    current_measurement_status = MEASUREMENT_STATUS_VALID;

    ModBus_UpdateFirmwareVersion(FIRMWARE_VERSION);
    TempSensors_Init();
    Grad_Init();

    /* Приоритеты реального времени:
     * TIM3 capture — самый высокий: импульс нельзя задерживать даже Modbus.
     * USART1 Modbus — выше всех фоновых задач и отладочного USART2.
     * USART2 — только отладка, самый низкий приоритет. */
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_NVIC_SetPriority(USART2_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    __enable_irq();

    Read_All_Voltages();
    Read_Temperature();
    ModBus_UpdateVoltages(current_vdda, current_24v, current_12v, current_5v);
    Update_Poll_Period_From_Modbus();

    USART2_BufInit();
    USART2_BufPrint("Modbus: Addr=");
    USART2_BufPrintInt(ModBus_GetDeviceAddress());
    USART2_BufPrint(", Baud=");
    USART2_BufPrintInt(ModBus_GetParameter_Int(MB_ADDR_MB_BAUD_SET));
    USART2_BufPrint("\r\n");
    USART2_BufPrint("[DBG] Период: ");
    if (current_poll_period_ms >= 1000) {
        USART2_BufPrintInt(current_poll_period_ms / 1000);
        USART2_BufPrint(" сек\r\n");
    } else {
        USART2_BufPrintInt(current_poll_period_ms);
        USART2_BufPrint(" мс\r\n");
    }
    USART2_BufPrint("[DBG] Скорость волны в материале: ");
    USART2_BufPrintFloat(ModBus_GetMaterialWaveSpeed());
    USART2_BufPrint(" м/с (регистры 2094-2095)\r\n");

    {
        uint16_t cal_mask = ModBus_GetParameter_Int(MB_ADDR_SENSOR_CAL_MASK);
        float C1 = ModBus_GetParameter_Float(MB_ADDR_CAL_C1);
        float C2 = ModBus_GetParameter_Float(MB_ADDR_CAL_C2);
        float tof_260 = ModBus_GetParameter_Float(MB_ADDR_SENSOR_CAL_260);
        float tof_520 = ModBus_GetParameter_Float(MB_ADDR_SENSOR_CAL_520);
        float tof_780 = ModBus_GetParameter_Float(MB_ADDR_SENSOR_CAL_780);

        const uint16_t three_point_mask =
            SENSOR_CAL_POINT_260_BIT |
            SENSOR_CAL_POINT_520_BIT |
            SENSOR_CAL_POINT_780_BIT;
        float tof_probe =
            ModBus_GetParameter_Float(MB_ADDR_SENSOR_CAL_520);

        (void)InterpolateCalibratedLevel(tof_probe);

        USART2_BufPrint("[CAL] EEPROM: mask=");
        USART2_BufPrintInt(cal_mask);
        USART2_BufPrint(", bits=");
        USART2_BufPrintInt((uint16_t)(cal_mask & SENSOR_CAL_FULL_MASK));
        USART2_BufPrint(", T260=");
        USART2_BufPrintFloat(tof_260);
        USART2_BufPrint(", T520=");
        USART2_BufPrintFloat(tof_520);
        USART2_BufPrint(", T780=");
        USART2_BufPrintFloat(tof_780);
        USART2_BufPrint(" мкс\r\n");

        if (last_sensor_cal_mode == 5U) {
            USART2_BufPrint("[CAL] Активна калибровка 5P по ToFraw: 0%/260/520/780/100%\r\n");
        } else if (last_sensor_cal_mode == 3U &&
                   (cal_mask & SENSOR_CAL_STORAGE_TAG_MASK) == SENSOR_CAL_STORAGE_TAG &&
                   (cal_mask & three_point_mask) == three_point_mask) {
            USART2_BufPrint("[CAL] Активна калибровка 3P по ToFraw: 260/520/780 мм\r\n");
        } else if (C1 > 0.0f && C2 > 0.0f && C1 > C2) {
            USART2_BufPrint("[CAL] Активна резервная калибровка 0/100%: C1=");
            USART2_BufPrintFloat(C1);
            USART2_BufPrint(" мм, C2=");
            USART2_BufPrintFloat(C2);
            USART2_BufPrint(" мм\r\n");
        } else {
            USART2_BufPrint("[CAL] Калибровка не завершена: используется расчёт по скорости\r\n");
        }
    }
    USART2_BufFlush();

    uint32_t last_measure_time = 0;
    uint32_t last_debug_time = 0;
    uint32_t led_red_off_time = 0;
    uint32_t last_temp_read_time = 0;
    uint32_t last_voltage_read_time = 0;
    uint32_t last_measure_error_time = 0;
    uint8_t blue_led_state = 0;
    uint8_t red_led_state = 0;

    while (1) {
        uint32_t now;
        uint32_t elapsed_since_measure;
        uint32_t time_to_next_measure;
        bool measurement_processed = false;
        bool allow_storage;

        /* Modbus-кадры обрабатываются быстро; EEPROM здесь не пишется. */
        ModBus_Process();

        /* Проверяем обязательную запись 01/02/223. Функция не блокирует
         * цикл и только переносит результат EEPROM в адрес 3000. */
        ProcessCommandSaveResult();

        USART2_TxProcess();
        now = HAL_GetTick();

        if ((uint32_t)(now - last_debug_time) >= 1000U) {
            last_debug_time = now;
            blue_led_state = !blue_led_state;
            HAL_GPIO_WritePin(GPIOB, LED_BLUE_PIN,
                              blue_led_state ? LED_BLUE_ON : LED_BLUE_OFF);
        }

        if (red_led_state &&
            (uint32_t)(now - led_red_off_time) >= LED_RED_ON_TIME_MS) {
            HAL_GPIO_WritePin(GPIOB, LED_RED_PIN, LED_RED_OFF);
            red_led_state = 0U;
        }

        if ((uint32_t)(now - last_temp_read_time) >= 2000U &&
            !ModBus_CommunicationIsBusy()) {
            Read_Temperature();
            last_temp_read_time = HAL_GetTick();
        }

        {
            uint16_t cmd = ModBus_GetParameter_Int(MB_ADDR_COMMAND);
            /* 0/85/90/99 являются кодами результата, а не номерами команд. */
            if (command_save_pending == 0U &&
                cmd != 0U && cmd != 85U && cmd != 90U && cmd != 99U) {
                USART2_Print("[CAL] Получена команда: ");
                USART2_PrintInt(cmd);
                USART2_Print("\r\n");
                ModBus_SetParameter_Int(MB_ADDR_COMMAND, 85U);
                Process_Calibration_Command(cmd);
            }
        }

        if ((uint32_t)(now - last_measure_time) >= current_poll_period_ms) {
            uint32_t measurement;
            float raw_tof_us;
            float raw_position_mm;
            float position_mm;

            last_measure_time = now;
            Update_Poll_Period_From_Modbus();
            HAL_GPIO_WritePin(GPIOB, LED_RED_PIN, LED_RED_OFF);
            red_led_state = 0U;
            signal_captured = 0U;

            measurement = measure_time_of_flight();
            if (measurement > 0U && !tof_timeout) {
                /* measure_time_of_flight() выполняет один физический запуск.
                 * После заполнения скользящего окна возвращается среднее 11->9;
                 * при первоначальном заполнении возвращается текущий t1.
                 * Дополнительный EMA намеренно не применяется. */
                raw_tof_us = (float)measurement * TOF_TICK_US;

                /* Запоминаем именно стабильный результат штатного цикла.
                 * Команды 01/02/11/12/13 сохранят этот же ToF без отдельного
                 * импульса и без расхождения с отображаемым значением. */
                last_stable_tof_ticks = measurement;
                last_stable_tof_time_ms = HAL_GetTick();
                last_stable_tof_valid = 1U;

                raw_position_mm = Calculate_Position(raw_tof_us);
                position_mm = raw_position_mm;

                if (position_mm >= 0.0f && isfinite(position_mm)) {
                    signal_captured = 1U;
                    measurement_processed = true;

                    Process_Measurement_Results(raw_tof_us,
                                                position_mm,
                                                signal_captured);

                    HAL_GPIO_WritePin(GPIOB, LED_RED_PIN, LED_RED_ON);
                    red_led_state = 1U;
                    led_red_off_time = HAL_GetTick();
                }
            } else if ((uint32_t)(now - last_measure_error_time) >= 1000U) {
                last_measure_error_time = now;
                USART2_Print("[MEAS] Нет валидного импульса\r\n");
            }
        }

        now = HAL_GetTick();
        if ((uint32_t)(now - last_voltage_read_time) >= 1000U &&
            !ModBus_CommunicationIsBusy()) {
            Read_All_Voltages();
            last_voltage_read_time = HAL_GetTick();
            ModBus_UpdateVoltages(current_vdda,
                                  current_24v,
                                  current_12v,
                                  current_5v);
        }

        /* Приоритет: измерение/вывод -> Modbus -> только затем одна операция
         * фоновой записи EEPROM. Пока USART2 передает данные, EEPROM стоит. */
        USART2_TxProcess();
        now = HAL_GetTick();
        elapsed_since_measure = (uint32_t)(now - last_measure_time);
        time_to_next_measure =
            (elapsed_since_measure < current_poll_period_ms) ?
            (current_poll_period_ms - elapsed_since_measure) : 0U;

        /*
         * EEPROM запускается только без активного Modbus-кадра и вдали от
         * следующего 10-Гц измерения. Обычная отложенная запись дополнительно
         * ждет освобождения USART2. Для обязательной фиксации C1/C2 отладочный
         * вывод не имеет права бесконечно откладывать запись, поэтому состояния
         * PENDING/BUSY получают приоритет над USART2.
         */
        {
            ModBus_StorageSaveStatus_t save_state =
                ModBus_GetStorageSaveStatus();
            bool command_save_has_priority =
                (save_state == MODBUS_STORAGE_SAVE_PENDING) ||
                (save_state == MODBUS_STORAGE_SAVE_BUSY);

            allow_storage = !measurement_processed &&
                            !ModBus_CommunicationIsBusy() &&
                            (command_save_has_priority || USART2_TxIsIdle()) &&
                            time_to_next_measure > EEPROM_MEASUREMENT_GUARD_MS;
        }
        ModBus_StorageProcess(allow_storage);

        /* После первого байта запроса цикл больше не засыпает. */
        if (!ModBus_CommunicationIsBusy()) {
            HAL_Delay(1U);
        }
    }
}

/* ==========================================================================
CALLBACKS И НИЗКОУРОВНЕВЫЕ ФУНКЦИИ
========================================================================== */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        ModBus_RxCallback(huart);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    USART2_TxCpltCallback(huart);
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

    /*
     * Channel 4 Input Capture.
     * Захват выполняется ТОЛЬКО по нарастающему импульсу каждого из двух
     * сформированных внешней схемой импульсов. Биты CC4P/CC4NP явно
     * очищены и далее в программе не переключаются.
     */
    TIM3->CCMR2 = (1U << 8) |   /* CC4S = 01: TI4 input */
                  (2U << 12);  /* IC4F = 0010: digital filter */
    TIM3->CCER = TIM_CCER_CC4E; /* CC4P=0, CC4NP=0: rising edge only */
    TIM3->DIER = 0;
    TIM3->SR = 0;
    TIM3->CR1 = TIM_CR1_CEN;
}

/**
 * @brief Формирует один короткий задающий импульс и запускает окно фиксации.
 *
 * Последовательность синхронизирована так, чтобы нулевой отсчет TIM3 находился
 * максимально близко к переднему импульсу задающего импульса PB5. От этого
 * напрямую зависит абсолютная точность ToF.
 */
void generate_pulse_and_measure(void)
{
    uint8_t i;

    tof_measurement_done = 0U;
    tof_timeout = 0U;
    capture_count = 0U;

    for (i = 0U; i < MAX_CAPTURED_PULSES; ++i) {
        captured_pulses[i] = 0U;
    }

    /* Сначала переводим аналоговый ключ в исходное состояние. */
    HAL_GPIO_WritePin(SWITCH_PORT, SWITCH_PIN, GPIO_PIN_RESET);

    /* Счетчик обнуляется непосредственно перед импульсом PB5. Благодаря этому
     * BLANKING_WINDOW_TICKS отсчитывает именно первые 100 мкс после отправки,
     * а не время выполнения подготовительного кода. */
    TIM3->CR1 &= ~TIM_CR1_CEN;
    TIM3->SR = 0U;
    TIM3->CNT = 0U;
    __DSB();
    TIM3->CR1 |= TIM_CR1_CEN;
    __DSB();

    /* Задающий импульс. Прямая запись BSRR/BRR используется вместо HAL,
     * чтобы длительность импульсов была минимальной и повторяемой. */
    GPIOB->BSRR = Gen_Impuls_Pin;
    for (volatile uint32_t delay = 0U;
         delay < (PULSE_DELAY_ITERATIONS * 5U);
         ++delay) {
        __NOP();
    }
    GPIOB->BRR = Gen_Impuls_Pin;

    /* Небольшая аппаратная задержка до переключения приемного тракта. */
    for (volatile uint32_t delay = 0U;
         delay < DELAY_AFTER_PULSE_ITER;
         ++delay) {
        __NOP();
    }
    HAL_GPIO_WritePin(SWITCH_PORT, SWITCH_PIN, GPIO_PIN_SET);
}

/**
 * @brief Рассчитывает физически допустимое окно прихода первого сформированного импульса.
 *
 * Нижняя граница фиксирована: первые 100 мкс подавлены как наводка.
 * Верхняя граница получается из длины волновода и скорости волны:
 *
 *     t_max = L / c + задержка электроники + запас.
 *
 * Для L=1,0 м, c=2841,37 м/с и задержки 18 мкс получаем примерно
 * 352 мкс + 18 мкс + 60 мкс запаса. Поэтому ложные пары на 850–970 мкс
 * больше не могут быть приняты.
 */
static void PrepareCaptureWindow(void)
{
    float length_m = ModBus_GetWaveguideLength();
    float speed_m_s = ModBus_GetMaterialWaveSpeed();
    float maximum_tof_us;
    uint32_t maximum_ticks;

    if (!isfinite(length_m) || length_m <= 0.0f) {
        length_m = MODBUS_DEFAULT_WAVEGUIDE_LENGTH_M;
    }
    if (!isfinite(speed_m_s) || speed_m_s < 100.0f) {
        speed_m_s = MODBUS_DEFAULT_MATERIAL_WAVE_SPEED_MPS;
    }

    maximum_tof_us =
        (length_m / speed_m_s) * 1000000.0f +
        ELECTRONICS_DELAY_US +
        (float)CAPTURE_MAX_TOF_MARGIN_US;

    /* Верхняя граница должна оставлять место и для второго сформированного импульса пары. */
    if (maximum_tof_us <
        (float)(CAPTURE_BLANKING_TIME_US + CAPTURE_PAIR_INTERVAL_MAX_US + 10U)) {
        maximum_tof_us =
            (float)(CAPTURE_BLANKING_TIME_US +
                    CAPTURE_PAIR_INTERVAL_MAX_US + 10U);
    }

    maximum_ticks =
        (uint32_t)ceilf(maximum_tof_us / TIM3_CAPTURE_TICK_US);

    /* TIM3 работает как 16-битный счетчик. Значение 0xFFFF оставляем
     * резервным, чтобы сравнение в ISR не зависело от переполнения. */
    if (maximum_ticks > 0xFFFEU) {
        maximum_ticks = 0xFFFEU;
    }

    capture_max_tof_ticks = maximum_ticks;
}

/**
 * @brief Ожидает разрешенный момент следующего задающего импульса.
 *
 * Генерация ограничена 10 Гц независимо от того, кто вызвал измерение:
 * основной цикл, команда калибровки или повторный запуск. Во время ожидания
 * рабочий Modbus и неблокирующий USART2 продолжают обслуживаться.
 */
static void WaitForNextExcitationSlot(void)
{
    if (excitation_time_initialized != 0U) {
        while ((uint32_t)(HAL_GetTick() - last_excitation_time_ms) <
               EXCITATION_PERIOD_MS) {
            ModBus_Process();
            USART2_TxProcess();
            __NOP();
        }
    }

    last_excitation_time_ms = HAL_GetTick();
    excitation_time_initialized = 1U;
}

/**
 * @brief Устанавливает или снимает аварию катушки фиксации.
 *
 * Авария означает, что первый входной импульс присутствует и уровень можно
 * измерять, но второй подтверждающий импульс отсутствует либо пришел вне
 * диапазона 14...26 мкс. При аварии расчет продолжается по времени первого
 * импульса, а в Modbus-регистр 2416 записывается код 0x0101.
 *
 * Функция печатает сообщение только при изменении состояния, поэтому USART2
 * не заполняется одинаковыми строками на каждом запуске.
 */
static void UpdateCaptureCoilFault(uint8_t fault_active)
{
    fault_active = (fault_active != 0U) ? 1U : 0U;

    current_measurement_status = MEASUREMENT_STATUS_VALID;
    if (fault_active != 0U) {
        current_measurement_status |=
            MEASUREMENT_STATUS_SINGLE_PULSE |
            MEASUREMENT_STATUS_COIL_FAULT;
    }

    if (fault_active == capture_coil_fault_active) {
        return;
    }

    capture_coil_fault_active = fault_active;

    if (fault_active != 0U) {
        ModBus_SetParameter_Int(MB_ADDR_ERROR_CODE,
                                MEASUREMENT_ERROR_CAPTURE_COIL);
        if (!ModBus_CommunicationIsBusy()) {
            USART2_Print(
                "[ALARM] Неисправна катушка фиксации: второй импульс "
                "отсутствует/неверен. Измерение продолжается по первому "
                "импульсу.\r\n");
        }
    } else {
        ModBus_SetParameter_Int(MB_ADDR_ERROR_CODE,
                                MEASUREMENT_ERROR_NONE);
        if (!ModBus_CommunicationIsBusy()) {
            USART2_Print(
                "[ALARM] Катушка фиксации восстановлена: получены 11 "
                "последовательных корректных пар.\r\n");
        }
    }
}

/**
 * @brief Добавляет новый ToF в скользящее окно из 11 запусков.
 *
 * После заполнения окна самый старый элемент заменяется новым. Одновременно
 * ведется количество запусков, где использовался только первый импульс.
 * Пока хотя бы один такой запуск остается в последних 11 измерениях, авария
 * катушки остается активной. Для снятия аварии нужны 11 корректных пар подряд.
 */
static void AddMeasurementToWindow(uint32_t tof_ticks,
                                   uint8_t used_single_pulse)
{
    used_single_pulse = (used_single_pulse != 0U) ? 1U : 0U;

    if (measurement_window_count == MEASUREMENT_REQUIRED_SAMPLES) {
        if (measurement_pair_fallback[measurement_window_index] != 0U &&
            measurement_fallback_count > 0U) {
            --measurement_fallback_count;
        }
    } else {
        ++measurement_window_count;
    }

    measurement_window[measurement_window_index] = tof_ticks;
    measurement_pair_fallback[measurement_window_index] = used_single_pulse;

    if (used_single_pulse != 0U) {
        ++measurement_fallback_count;
    }

    ++measurement_window_index;
    if (measurement_window_index >= MEASUREMENT_REQUIRED_SAMPLES) {
        measurement_window_index = 0U;
    }

    UpdateCaptureCoilFault(measurement_fallback_count > 0U);
}

/**
 * @brief Выполняет один физический запуск измерительного тракта.
 *
 * Штатный режим:
 *  - после защитного окна 100 мкс фиксируется первый импульс t1;
 *  - второй импульс t2 подтверждает пару, если t2-t1 = 14...26 мкс;
 *  - для расчета ToF всегда используется t1.
 *
 * Аварийный режим:
 *  - если второй импульс не появился за 26 мкс + 5 мкс запаса либо его
 *    интервал неверен, t1 все равно возвращается как результат;
 *  - качество запуска помечается как работа по одному импульсу;
 *  - отсутствие первого импульса остается полной ошибкой измерения.
 */
static LaunchResult_t MeasureSingleLaunch(void)
{
    LaunchResult_t result = {0U, LAUNCH_NO_INPUT_PULSE};
    uint32_t start_wait;
    uint32_t timeout_ms;

    /* Этот вызов является единственной точкой ограничения частоты PB5. */
    WaitForNextExcitationSlot();

    TIM3->SR = 0U;
    TIM3->DIER |= TIM_DIER_CC4IE;
    generate_pulse_and_measure();

    start_wait = HAL_GetTick();
    timeout_ms = GetMeasurementTimeoutMs();

    while (tof_measurement_done == 0U && tof_timeout == 0U) {
        ModBus_Process();

        /* Если первый импульс уже зафиксирован, не ждем полный миллисекундный
         * таймаут. После 26 мкс + 5 мкс второй импульс считается пропавшим.
         * Первый импульс при этом остается полноценным измерением ToF. */
        if (capture_count == 1U) {
            uint32_t elapsed_ticks =
                (uint16_t)((uint16_t)TIM3->CNT -
                           (uint16_t)captured_pulses[0]);

            if (elapsed_ticks > CAPTURE_SECOND_PULSE_TIMEOUT_TICKS) {
                TIM3->DIER &= ~TIM_DIER_CC4IE;
                tof_measurement_done = 1U;
                break;
            }
        }

        if ((uint32_t)(HAL_GetTick() - start_wait) >= timeout_ms) {
            tof_timeout = 1U;
            break;
        }
        __NOP();
    }

    TIM3->DIER &= ~TIM_DIER_CC4IE;
    TIM3->CR1 &= ~TIM_CR1_CEN;
    TIM3->SR = 0U;

    /* Сохраняем проверенную последовательность управления аналоговым ключом. */
    for (volatile uint32_t delay = 0U;
         delay < SWITCH_HOLD_ITERATIONS;
         ++delay) {
        __NOP();
    }
    HAL_GPIO_WritePin(SWITCH_PORT, SWITCH_PIN, GPIO_PIN_RESET);

    /* Без первого импульса расстояние определить невозможно. */
    if (capture_count == 0U ||
        captured_pulses[0] < BLANKING_WINDOW_TICKS ||
        captured_pulses[0] > capture_max_tof_ticks) {
        return result;
    }

    result.tof_ticks = captured_pulses[0];
    tof_timeout = 0U;

    if (capture_count >= 2U) {
        uint32_t interval_ticks =
            captured_pulses[1] - captured_pulses[0];

        if (interval_ticks >= MIN_CLICK_WIDTH_TICKS &&
            interval_ticks <= MAX_CLICK_WIDTH_TICKS) {
            result.quality = LAUNCH_VALID_PAIR;
        } else {
            /* Второй импульс есть, но аппаратная пара нарушена. Для уровня
             * используем t1, а неисправность отмечаем аварийным статусом. */
            result.quality = LAUNCH_SINGLE_PULSE_BAD_INTERVAL;
        }
    } else {
        /* Второй импульс физически не появился. Продолжаем по одному t1. */
        result.quality = LAUNCH_SINGLE_PULSE_NO_SECOND;
    }

    return result;
}

/**
 * @brief Обновляет скользящую статистику уровня одним запуском на каждый вызов.
 *
 * В предыдущих версиях 11...22 импульса отправлялись одной быстрой пачкой,
 * что давало фактическую частоту около 100 Гц. Теперь каждый вызов формирует
 * ровно один импульс, а глобальный ограничитель выдерживает минимум 100 мс.
 *
 * Первые десять запусков заполняют окно и возвращают текущий t1 без задержки.
 * Начиная с одиннадцатого запуска результат обновляется каждые 100 мс:
 *  - сортируются последние 11 значений;
 *  - удаляются один минимум и один максимум;
 *  - усредняются оставшиеся девять.
 */
uint32_t measure_time_of_flight(void)
{
    LaunchResult_t launch;
    MeasurementStatisticsResult_t statistics;
    uint32_t ordered_samples[MEASUREMENT_REQUIRED_SAMPLES];
    uint8_t i;
    uint8_t used_single_pulse;

    PrepareCaptureWindow();
    launch = MeasureSingleLaunch();

    if (launch.tof_ticks == 0U) {
        tof_timeout = 1U;
        return 0U;
    }

    used_single_pulse =
        (launch.quality == LAUNCH_VALID_PAIR) ? 0U : 1U;
    AddMeasurementToWindow(launch.tof_ticks, used_single_pulse);

    /* Во время заполнения окна не задерживаем появление уровня на 1,1 секунды.
     * Публикуется текущее значение первого импульса; после заполнения автоматически
     * включается требуемая статистика 11 -> убрать min/max -> среднее 9. */
    if (measurement_window_count < MEASUREMENT_REQUIRED_SAMPLES) {
        tof_timeout = 0U;
        return launch.tof_ticks;
    }

    /* Кольцевой порядок не важен для сортировки, поэтому достаточно скопировать
     * все 11 элементов окна в локальный массив. */
    for (i = 0U; i < MEASUREMENT_REQUIRED_SAMPLES; ++i) {
        ordered_samples[i] = measurement_window[i];
    }

    if (!MeasurementStatistics_Calculate(
            ordered_samples,
            MEASUREMENT_REQUIRED_SAMPLES,
            MEASUREMENT_MAX_SPREAD_TICKS,
            &statistics)) {
        tof_timeout = 1U;

        if (!ModBus_CommunicationIsBusy()) {
            USART2_BufInit();
            USART2_BufPrint("[STAT] rejected: spread=");
            USART2_BufPrintFloat(
                (float)statistics.spread_ticks * TIM3_CAPTURE_TICK_US);
            USART2_BufPrint(" us, limit=");
            USART2_BufPrintInt(MEASUREMENT_MAX_SPREAD_US);
            USART2_BufPrint(" us, single=");
            USART2_BufPrintInt(measurement_fallback_count);
            USART2_BufPrint("/11\r\n");
            USART2_BufFlush();
        }
        return 0U;
    }

    tof_timeout = 0U;

    if (!ModBus_CommunicationIsBusy()) {
        USART2_BufInit();
        USART2_BufPrint("[STAT] accepted: 11->9, spread=");
        USART2_BufPrintFloat(
            (float)statistics.spread_ticks * TIM3_CAPTURE_TICK_US);
        USART2_BufPrint(" us, pair=");
        USART2_BufPrintInt(
            MEASUREMENT_REQUIRED_SAMPLES - measurement_fallback_count);
        USART2_BufPrint("/11, single=");
        USART2_BufPrintInt(measurement_fallback_count);
        USART2_BufPrint("/11\r\n");
        USART2_BufFlush();
    }

    return statistics.mean_ticks;
}

void Read_Temperature(void)
{
    TempSensors_ReadAll();
    float level_m = ModBus_GetParameter_Float(MB_ADDR_LEVEL) / 1000.0f;
    float sep_level = ModBus_GetParameter_Float(MB_ADDR_LEVEL_SEP);
    TempSensors_CalculateAverages(level_m, sep_level);

    TempSensorsState_t *state = TempSensors_GetState();

    /* Обновляем общую температуру только валидным результатом датчиков. */
    if (isfinite(state->avg_liquid_temp)) {
        current_temperature = state->avg_liquid_temp;
    }
    /* Если датчики не дали значение - оставляем текущую температуру */
}

static void DelayWithModBusService(uint32_t delay_ms)
{
    uint32_t start = HAL_GetTick();

    while ((uint32_t)(HAL_GetTick() - start) < delay_ms) {
        ModBus_Process();
        USART2_TxProcess();
        __NOP();
    }
}

void Read_All_Voltages(void)
{
    uint32_t adc_raw_vdda = 0, adc_raw_24v = 0, adc_raw_12v = 0, adc_raw_5v = 0;

    ADC1->CR2 |= ADC_CR2_TSVREFE;
    DelayWithModBusService(10U);
    adc_raw_vdda = Read_ADC_Average(&hadc1, ADC_CHANNEL_VREFINT, ADC_SAMPLETIME_239CYCLES_5, ADC_SAMPLES);
    ADC1->CR2 &= ~ADC_CR2_TSVREFE;
    DelayWithModBusService(1U);

    if (adc_raw_vdda > 1000 && adc_raw_vdda < 2000 && *VREFINT_CAL_VALUE > 1000 && *VREFINT_CAL_VALUE < 2000) {
        current_vdda = 3.3f * (float)(*VREFINT_CAL_VALUE) / (float)adc_raw_vdda;
        vdda_error = 0;
    } else {
        current_vdda = 3.3f;
        vdda_error = 1;
    }

    adc_raw_24v = Read_ADC_Average(&hadc1, ADC_CHANNEL_0, ADC_SAMPLETIME_239CYCLES_5, ADC_SAMPLES);
    if (adc_raw_24v > 100 && adc_raw_24v < 4000) {
        current_24v = ((float)adc_raw_24v * current_vdda / 4095.0f) * DIV_24V_FACTOR;
        v24_error = 0;
    } else {
        current_24v = 24.0f;
        v24_error = 1;
    }

    adc_raw_12v = Read_ADC_Average(&hadc2, ADC_CHANNEL_1, ADC_SAMPLETIME_239CYCLES_5, ADC_SAMPLES);
    if (adc_raw_12v > 100 && adc_raw_12v < 4000) {
        current_12v = ((float)adc_raw_12v * current_vdda / 4095.0f) * DIV_12V_FACTOR;
        v12_error = 0;
    } else {
        current_12v = 12.0f;
        v12_error = 1;
    }

    adc_raw_5v = Read_ADC_Average(&hadc2, ADC_CHANNEL_5, ADC_SAMPLETIME_239CYCLES_5, ADC_SAMPLES);
    if (adc_raw_5v > 100 && adc_raw_5v < 4000) {
        current_5v = ((float)adc_raw_5v * current_vdda / 4095.0f) * DIV_5V_FACTOR;
        v5_error = 0;
    } else {
        current_5v = 5.0f;
        v5_error = 1;
    }

    Check_Voltage_Change("VDDA", current_vdda, prev_vdda, &prev_vdda);
    Check_Voltage_Change("+24V", current_24v, prev_24v, &prev_24v);
    Check_Voltage_Change("+12V", current_12v, prev_12v, &prev_12v);
}

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
    }
    return (valid > 0) ? sum / valid : 0;
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
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();
}

void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    HAL_GPIO_WritePin(Gen_Impuls_GPIO_Port, Gen_Impuls_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Switch_In_impuls_GPIO_Port,
                      Switch_In_impuls_Pin,
                      GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RS485_CTRL_GPIO_Port, RS485_CTRL_Pin, GPIO_PIN_RESET);

    gpio.Pin = Gen_Impuls_Pin | Switch_In_impuls_Pin | RS485_CTRL_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin = LED_RED_Pin | LED_BLUE_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin = ON_VCC_5_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ON_VCC_5_GPIO_Port, &gpio);
    HAL_GPIO_WritePin(ON_VCC_5_GPIO_Port, ON_VCC_5_Pin, GPIO_PIN_SET);

    gpio.Pin = Check_OPA552_Pin;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(Check_OPA552_GPIO_Port, &gpio);
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
    if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
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
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
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
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();
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
    if (HAL_ADC_Init(&hadc2) != HAL_OK) Error_Handler();
}

void Error_Handler(void)
{
    while (1) {
        HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
        HAL_Delay(200U);
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { }
#endif
