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
#include "params_storage.h"
#include <stdint.h>
#include <string.h>
#include <math.h>

/* ==========================================================================
КОНСТАНТЫ И МАКРОСЫ
========================================================================== */
#define TIMER_CLOCK_HZ          72000000.0f /* Частота таймерной базы/ядра, используемая в расчетах времени. */
#define TOF_TICK_US             TIM3_CAPTURE_TICK_US /* Длительность одного отсчета TIM3 в микросекундах. */
#define VREFINT_CAL_ADDR        0x1FFFF7BA /* Адрес заводской калибровочной константы внутреннего VREF STM32F103. */
#define VREFINT_CAL_VALUE       ((uint16_t *)VREFINT_CAL_ADDR) /* Указатель на заводское значение VREFINT. */
#define ADC_SAMPLES             16 /* Число выборок ADC, усредняемых при контроле напряжений питания. */
#define MEAS_TIMEOUT_MIN_MS     3U /* Минимальное время ожидания входного импульса после запуска, мс. */
#define MEAS_TIMEOUT_MAX_MS     20U /* Максимальное время ожидания входного импульса, мс. */
#define MEAS_TIMEOUT_MARGIN_MS  2.0f /* Дополнительный запас к расчетному времени пролета волны, мс. */
#define DIV_24V_FACTOR          9.3f /* Коэффициент делителя ADC канала линии +24 В: Uвх = Uadc * 9,4. */
#define DIV_12V_FACTOR          4.0f /* Коэффициент делителя ADC канала линии +12 В: Uвх = Uadc * 4,0. */
#define PULSE_PERIOD_MS_DEFAULT EXCITATION_PERIOD_MS_DEFAULT /* Период возбуждения после сброса настроек: 100 мс = 10 Гц. */
#define LED_RED_PIN             GPIO_PIN_13 /* Красный светодиод на PB13. */
#define LED_BLUE_PIN            GPIO_PIN_12 /* Синий светодиод на PB12. */
#define LED_RED_ON_TIME_MS      1000 /* Длительность включения красного LED после события, мс. */
#define LED_RED_ON              GPIO_PIN_SET /* Активный логический уровень красного светодиода. */
#define LED_RED_OFF             GPIO_PIN_RESET /* Неактивный логический уровень красного светодиода. */
#define LED_BLUE_ON             GPIO_PIN_SET /* Активный логический уровень синего светодиода. */
#define LED_BLUE_OFF            GPIO_PIN_RESET /* Неактивный логический уровень синего светодиода. */
/* PULSE_DELAY_ITERATIONS удален в v135: ширина PB5 теперь задается точно через TIM3 и Modbus 2116...2117. */
#define DELAY_AFTER_PULSE_ITER  97 /* Короткая NOP-задержка между спадом PB5 и переключением аналогового ключа. */
#define SWITCH_HOLD_ITERATIONS  12000 /* NOP-задержка удержания приемного ключа перед возвратом в исходное состояние. */
#define SWITCH_PIN              GPIO_PIN_7 /* Управляющий вывод аналогового ключа приемного тракта. */
#define SWITCH_PORT             GPIOB /* GPIO-порт управляющего вывода аналогового ключа. */
#define FIRMWARE_VERSION        136 /* Номер версии ПО, публикуемый через информационный регистр Modbus. */
#define CALIBRATION_SAMPLE_MAX_AGE_MS 500U /* Резерв старой оконной калибровки: максимальный возраст выборки; fresh-capture сейчас не использует. */
#define CALIBRATION_STABLE_WINDOWS_REQUIRED 10U /* Резерв старой оконной калибровки: требуемые устойчивые окна; fresh-capture сейчас не использует. */
#define CALIBRATION_MIN_ACCEPTED_SAMPLES 16U /* Резерв старой оконной калибровки: минимум отсчетов; fresh-capture сейчас не использует. */

/*
 * Для записи калибровочной координаты используются только запуски PAIR,
 * поскольку в режиме 1P единственный принятый фронт может принадлежать
 * второй катушке и быть сдвинут примерно на межимпульсный интервал.
 *
 * В верхней части звукопровода полноценная пара появляется реже. Поэтому
 * одиночные запуски теперь ПРОПУСКАЮТСЯ, но не обнуляют уже набранные PAIR.
 * Это позволяет получить правильный t1 первой катушки даже при 80...90 %
 * запусков 1P.
 */
#define CALIBRATION_PAIR_SAMPLE_TARGET   8U
#define CALIBRATION_PAIR_TRIM_COUNT      1U
#define CALIBRATION_CAPTURE_MAX_ATTEMPTS 160U

/*
 * Допустимый диапазон расчётной скорости между соседними контрольными
 * точками. Проверка не участвует в рабочем пересчёте уровня: она только
 * запрещает случайно записать команду 12 или 13 не в той физической точке.
 * Для текущего звукопровода ожидается примерно 2700...3000 м/с.
 */
#define CALIBRATION_SEGMENT_SPEED_MIN_MPS 2000.0f
#define CALIBRATION_SEGMENT_SPEED_MAX_MPS 4000.0f
/* 0,5 мкс при текущей частоте TIM3. Используется только для записи
 * калибровочной точки и не ограничивает обычное измерение уровня. */
#define CALIBRATION_MAX_SPREAD_TICKS \
    ((uint32_t)(((uint64_t)TIM3_CAPTURE_FREQUENCY_HZ + 1999999ULL) / \
                2000000ULL))
#define MIN_POLL_PERIOD_MS      50U /* Максимальная разрешенная частота 20 Гц. */
#define MAX_POLL_PERIOD_MS      60000U /* Минимальная частота примерно 0,0167 Гц. */
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
static float current_temperature = 0.0f;
static uint32_t current_poll_period_ms = PULSE_PERIOD_MS_DEFAULT;
static float prev_vdda = 0.0f;
static float prev_24v = 0.0f;
static float prev_12v = 0.0f;
static uint8_t v24_error = 0;
static uint8_t v12_error = 0;
static uint8_t vdda_error = 0;

/*
 * Состояние генератора и скользящего статистического окна.
 * last_excitation_time_ms выдерживает текущий период возбуждения из Modbus между импульсами.
 * measurement_pair_fallback[] хранит признак работы по одному импульсу для
 * каждого элемента окна. Авария снимается только после полного окна последовательных
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
 * 4 — те же три точки с активными границами анализа 01/02.
 *
 * Значение обновляется внутри InterpolateCalibratedLevel() и применяется
 * только для диагностического вывода USART2.
 */
static uint8_t last_sensor_cal_mode = 0U;

/* Последнее стабильное значение штатного измерительного цикла.
 * Оно сохраняется для диагностики, но команды 01/02/11/12/13 в версии 130
 * формируют собственную свежую серию подтверждённых PAIR и не используют старый кэш. */

/*
 * Выходной фильтр одиночных скачков уровня.
 *
 * Основная статистика работает в тиках ToF и удаляет по три крайних
 * значения с каждой стороны. Этот дополнительный уровень защиты действует
 * уже после калибровочного преобразования ToF -> мм и не позволяет одному
 * редкому помеховому результату попасть в Modbus.
 *
 * Малые изменения принимаются сразу. Изменение более
 * LEVEL_JUMP_CONFIRM_THRESHOLD_MM должно повториться три раза подряд.
 * Поэтому реальный скачок уровня принимается не позднее чем примерно через
 * 300 мс при частоте измерения 10 Гц, а одиночный выброс отбрасывается.
 */
static float published_filtered_level_mm = 0.0f;
static float pending_level_mm = 0.0f;
static uint8_t published_level_initialized = 0U;
static uint8_t pending_level_count = 0U;
static uint8_t pending_spike_reported = 0U;

/*
 * Команды 01/02 и команда 223 завершаются кодом 90 только после
 * подтвержденной записи требуемой редакции параметров в AT24C64. Пока идет
 * фоновая запись, адрес 3000 остается 85.
 */
static uint8_t command_save_pending = 0U;
static uint16_t command_save_code = 0U;

/*
 * Отложенный программный перезапуск для диагностики EEPROM.
 * Команда 224 перезапускает только STM32 через NVIC_SystemReset(), поэтому
 * питание AT24C64 остаётся включённым. Это позволяет отличить ошибку
 * программного формата/CRC от повреждения данных при снятии питания.
 */
static uint8_t software_reset_pending = 0U;
static uint32_t software_reset_requested_ms = 0U;

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
static float ConfirmMeasuredLevel(float candidate_mm);
static uint8_t RestoreThreePointCalibrationState(void);
static void ProcessSoftwareReset(void);

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
    uint16_t stored_bits =
        (uint16_t)(stored_mask & SENSOR_CAL_FULL_MASK);
    float tof_260_us =
        ModBus_GetParameter_Float(MB_ADDR_SENSOR_CAL_260);
    float tof_520_us =
        ModBus_GetParameter_Float(MB_ADDR_SENSOR_CAL_520);
    float tof_780_us =
        ModBus_GetParameter_Float(MB_ADDR_SENSOR_CAL_780);
    uint16_t normalized_mask;

    /*
     * Важна не только служебная сигнатура 0xA500, но и три реально
     * сохранённых значения ToFraw. Ранний выход только по наличию сигнатуры
     * был ошибкой: маска могла сохраниться как 0xA500/0xA502/0xA506, тогда
     * правильные T260/T520/T780 оставались в EEPROM, но CAL=3P не включался.
     */
    if (!isfinite(tof_260_us) ||
        !isfinite(tof_520_us) ||
        !isfinite(tof_780_us) ||
        tof_780_us <= 0.0f ||
        !(tof_260_us > tof_520_us &&
          tof_520_us > tof_780_us)) {
        return 0U;
    }

    /*
     * Если сигнатура и все три бита уже присутствуют, таблица загружена
     * полностью и менять её состояние не требуется.
     */
    if ((stored_mask & SENSOR_CAL_STORAGE_TAG_MASK) ==
            SENSOR_CAL_STORAGE_TAG &&
        (stored_bits & three_point_mask) == three_point_mask) {
        return 0U;
    }

    normalized_mask =
        (uint16_t)(SENSOR_CAL_STORAGE_TAG | three_point_mask);

    /*
     * Биты 0 % и 100 % сохраняются только из записи с правильной сигнатурой.
     * Их окончательная пригодность всё равно проверяется внутри
     * InterpolateCalibratedLevel() по монотонности всех пяти точек.
     */
    if ((stored_mask & SENSOR_CAL_STORAGE_TAG_MASK) ==
        SENSOR_CAL_STORAGE_TAG) {
        normalized_mask |=
            (uint16_t)(stored_bits &
                (SENSOR_CAL_POINT_LOW_BIT |
                 SENSOR_CAL_POINT_HIGH_BIT));
    }

    ModBus_SetParameter_Int(MB_ADDR_SENSOR_CAL_MASK,
                            normalized_mask);
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

    /*
     * Готовность расчёта определяется самими тремя значениями ToFraw,
     * а не служебной маской. Маска могла не обновиться после записи одного
     * из параметров, хотя все три точки уже находятся в RAM/EEPROM.
     */
    tof_us[0] = ModBus_GetParameter_Float(MB_ADDR_SENSOR_CAL_260);
    tof_us[1] = ModBus_GetParameter_Float(MB_ADDR_SENSOR_CAL_520);
    tof_us[2] = ModBus_GetParameter_Float(MB_ADDR_SENSOR_CAL_780);
    waveguide_mm = ModBus_GetWaveguideLength() * 1000.0f;

    if (!isfinite(waveguide_mm) || waveguide_mm <= SENSOR_CAL_LEVEL_780_MM ||
        !isfinite(tof_us[0]) || !isfinite(tof_us[1]) ||
        !isfinite(tof_us[2]) || tof_us[2] <= 0.0f ||
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

    if (!isfinite(calculated_speed_m_s) ||
        calculated_speed_m_s < 500.0f ||
        calculated_speed_m_s > 10000.0f) {
        return 0U;
    }

    *speed_m_s = calculated_speed_m_s;
    return 1U;
}

/**
 * @brief Рассчитывает уровень только по трём калибровочным точкам ToFraw.
 *
 * Команды 11, 12 и 13 являются единственными точками, формирующими наклон
 * функции ToF -> уровень:
 *
 *   11 -> 260 мм;
 *   12 -> 520 мм;
 *   13 -> 780 мм.
 *
 * Команды 01 и 02 не участвуют в интерполяции и не меняют коэффициент
 * пересчёта. Они сохраняют только нижнюю и верхнюю границы анализа. При выходе
 * сырого ToF за сохранённую границу результат насыщается соответствующим
 * уровнем h_low/h_high.
 *
 * Между 260 и 520 мм используется первый линейный участок, между 520 и
 * 780 мм — второй. Ниже 260 и выше 780 мм продолжается ближайший участок,
 * после чего результат ограничивается границами 01/02 и физической полезной
 * длиной звукопровода.
 */
static float InterpolateCalibratedLevel(float raw_tof_us)
{
    const uint16_t three_point_mask =
        SENSOR_CAL_POINT_260_BIT |
        SENSOR_CAL_POINT_520_BIT |
        SENSOR_CAL_POINT_780_BIT;
    const uint16_t boundary_mask =
        SENSOR_CAL_POINT_LOW_BIT |
        SENSOR_CAL_POINT_HIGH_BIT;
    uint16_t stored_mask = ModBus_GetParameter_Int(MB_ADDR_SENSOR_CAL_MASK);
    uint16_t normalized_mask;
    float tof_260_us;
    float tof_520_us;
    float tof_780_us;
    float tof_low_us = 0.0f;
    float tof_high_us = 0.0f;
    float level_low_mm;
    float level_high_mm;
    float usable_height_mm;
    float x0;
    float x1;
    float y0;
    float y1;
    float result_mm;
    uint8_t low_bound_valid = 0U;
    uint8_t high_bound_valid = 0U;

    last_sensor_cal_mode = 0U;

    /*
     * Сначала читаем сами калибровочные точки. Они являются источником
     * истины. Ранее функция немедленно переходила в FALLBACK при неполной
     * маске, даже когда T260/T520/T780 были корректно сохранены.
     */
    tof_260_us = ModBus_GetParameter_Float(MB_ADDR_SENSOR_CAL_260);
    tof_520_us = ModBus_GetParameter_Float(MB_ADDR_SENSOR_CAL_520);
    tof_780_us = ModBus_GetParameter_Float(MB_ADDR_SENSOR_CAL_780);
    usable_height_mm = GetUsableLevelHeightMm();

    if (!isfinite(raw_tof_us) || raw_tof_us <= 0.0f ||
        !isfinite(tof_260_us) || !isfinite(tof_520_us) ||
        !isfinite(tof_780_us) || tof_780_us <= 0.0f ||
        !(tof_260_us > tof_520_us && tof_520_us > tof_780_us) ||
        !isfinite(usable_height_mm) || usable_height_mm <= 0.0f) {
        return -1.0f;
    }

    /*
     * Если три точки корректны, таблица активируется независимо от состояния
     * служебной маски. Маска автоматически нормализуется в RAM. Биты границ
     * 01/02 сохраняются, но сами границы не участвуют в наклоне интерполяции.
     */
    normalized_mask = (uint16_t)(SENSOR_CAL_STORAGE_TAG |
                                 three_point_mask |
                                 (stored_mask & boundary_mask));

    if ((stored_mask & SENSOR_CAL_STORAGE_TAG_MASK) !=
            SENSOR_CAL_STORAGE_TAG ||
        (stored_mask & three_point_mask) != three_point_mask) {
        ModBus_SetParameter_Int(MB_ADDR_SENSOR_CAL_MASK, normalized_mask);
        stored_mask = normalized_mask;
    }

    level_low_mm =
        ModBus_GetParameter_Float(MB_ADDR_CAL_LOW_LVL) * 1000.0f;
    level_high_mm =
        ModBus_GetParameter_Float(MB_ADDR_CAL_HIGH_LVL) * 1000.0f;

    if (!isfinite(level_low_mm) || level_low_mm < 0.0f ||
        level_low_mm >= SENSOR_CAL_LEVEL_260_MM) {
        level_low_mm = 0.0f;
    }
    if (!isfinite(level_high_mm) ||
        level_high_mm <= SENSOR_CAL_LEVEL_780_MM ||
        level_high_mm > usable_height_mm) {
        level_high_mm = usable_height_mm;
    }

    /* Команда 01 задаёт только дальнюю границу анализа. */
    if ((stored_mask & SENSOR_CAL_POINT_LOW_BIT) != 0U) {
        tof_low_us =
            ModBus_GetParameter_Float(MB_ADDR_SENSOR_CAL_LOW_TOF);

        if (isfinite(tof_low_us) && tof_low_us > tof_260_us) {
            low_bound_valid = 1U;
        }
    }

    /* Команда 02 задаёт только ближнюю границу анализа. */
    if ((stored_mask & SENSOR_CAL_POINT_HIGH_BIT) != 0U) {
        tof_high_us =
            ModBus_GetParameter_Float(MB_ADDR_SENSOR_CAL_HIGH_TOF);

        if (isfinite(tof_high_us) && tof_high_us > 0.0f &&
            tof_high_us < tof_780_us) {
            high_bound_valid = 1U;
        }
    }

    /*
     * Кусочно-линейная интерполяция:
     *   T260...T520 -> 260...520 мм;
     *   T520...T780 -> 520...780 мм.
     * За пределами трёх точек продолжается ближайший участок.
     */
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

    if (x0 <= x1) {
        return -1.0f;
    }

    result_mm =
        y0 + (x0 - raw_tof_us) * (y1 - y0) / (x0 - x1);

    if (!isfinite(result_mm)) {
        return -1.0f;
    }

    /* Насыщение по сохранённым границам анализа 01/02. */
    if (low_bound_valid != 0U) {
        if (raw_tof_us >= tof_low_us || result_mm < level_low_mm) {
            result_mm = level_low_mm;
        }
    }
    if (high_bound_valid != 0U) {
        if (raw_tof_us <= tof_high_us || result_mm > level_high_mm) {
            result_mm = level_high_mm;
        }
    }

    if (result_mm < 0.0f) {
        result_mm = 0.0f;
    }
    if (result_mm > usable_height_mm) {
        result_mm = usable_height_mm;
    }

    last_sensor_cal_mode =
        (low_bound_valid != 0U || high_bound_valid != 0U) ? 4U : 3U;

    return result_mm;
}

/**
 * @brief Формирует свежую калибровочную точку по редким корректным PAIR.
 *
 * Главная ошибка предыдущей версии заключалась в требовании получить
 * двадцать PAIR подряд. В верхней части звукопровода второй импульс часто
 * не фиксируется, поэтому любой 1P обнулял уже набранную серию и команды
 * 13/02 практически всегда завершались CAP FAIL.
 *
 * В этой версии:
 *  - после команды выполняются только новые физические запуски;
 *  - в калибровку попадает исключительно t1 из LAUNCH_VALID_PAIR;
 *  - запуски 1P и запуски без входного импульса просто пропускаются;
 *  - ранее набранные корректные PAIR не удаляются;
 *  - собираются восемь PAIR, затем отбрасываются минимум и максимум;
 *  - среднее оставшихся шести t1 сохраняется как калибровочная точка;
 *  - допустимый разброс после отсечения остаётся не более 0,5 мкс.
 *
 * Использовать одиночный 1P непосредственно как калибровочный t1 нельзя:
 * когда первая катушка не сформировала фронт, единственный принятый импульс
 * может оказаться импульсом второй катушки. Для обычного аварийного измерения
 * 1P по-прежнему разрешён, но калибровка ждёт редкие подтверждённые PAIR.
 */
static uint8_t GetStableCalibrationSample(float *tof_us, float *distance_mm)
{
    uint32_t cal_samples[CALIBRATION_PAIR_SAMPLE_TARGET];
    uint64_t sum_ticks = 0ULL;
    uint32_t mean_ticks;
    uint32_t spread_ticks;
    uint32_t key;
    uint8_t sample_count = 0U;
    uint8_t single_count = 0U;
    uint8_t no_input_count = 0U;
    uint16_t attempts = 0U;
    uint8_t i;
    uint8_t j;
    float local_tof_us;
    float local_distance_mm;

    if (tof_us == NULL || distance_mm == NULL) {
        return 0U;
    }

    USART2_Print("[CAL] CAP\r\n");

    while (sample_count < CALIBRATION_PAIR_SAMPLE_TARGET &&
           attempts < CALIBRATION_CAPTURE_MAX_ATTEMPTS) {
        LaunchResult_t launch;

        PrepareCaptureWindow();
        launch = MeasureSingleLaunch();
        ++attempts;

        if (launch.tof_ticks == 0U) {
            ++no_input_count;
            continue;
        }

#if (EMERGENCY_SINGLE_PULSE_DEBUG_MODE != 0U)
        /*
         * АВАРИЙНАЯ СТЕНДОВАЯ ОТЛАДКА.
         * При физически отсутствующем втором входном импульсе калибровка
         * набирает только свежие t1 после получения команды. Никакой проверки
         * PAIR и никакого смешивания со старым рабочим окном здесь нет.
         */
        if (launch.quality != LAUNCH_VALID_PAIR) {
            ++single_count;
        }
        cal_samples[sample_count] = launch.tof_ticks;
        ++sample_count;
#else
        /*
         * ШТАТНЫЙ РЕЖИМ.
         * В калибровку допускается только t1, аппаратно подтвержденный
         * корректным вторым импульсом пары.
         */
        if (launch.quality != LAUNCH_VALID_PAIR) {
            ++single_count;
            continue;
        }

        cal_samples[sample_count] = launch.tof_ticks;
        ++sample_count;
#endif
    }

    if (sample_count < CALIBRATION_PAIR_SAMPLE_TARGET) {
        USART2_BufInit();
#if (EMERGENCY_SINGLE_PULSE_DEBUG_MODE != 0U)
        USART2_BufPrint("[CAL] 1P ");
#else
        USART2_BufPrint("[CAL] PAIR ");
#endif
        USART2_BufPrintInt(sample_count);
        USART2_BufPrint("/8 A=");
        USART2_BufPrintInt(attempts);
        USART2_BufPrint(" 1P=");
        USART2_BufPrintInt(single_count);
        USART2_BufPrint(" N=");
        USART2_BufPrintInt(no_input_count);
        USART2_BufPrint("\r\n");
        USART2_BufFlush();
        return 0U;
    }

    /* Сортировка восьми значений по возрастанию методом вставок. */
    for (i = 1U; i < CALIBRATION_PAIR_SAMPLE_TARGET; ++i) {
        key = cal_samples[i];
        j = i;

        while (j > 0U && cal_samples[j - 1U] > key) {
            cal_samples[j] = cal_samples[j - 1U];
            --j;
        }
        cal_samples[j] = key;
    }

    /* Отбрасываем один минимум и один максимум. */
    spread_ticks =
        cal_samples[CALIBRATION_PAIR_SAMPLE_TARGET -
                    CALIBRATION_PAIR_TRIM_COUNT - 1U] -
        cal_samples[CALIBRATION_PAIR_TRIM_COUNT];

    if (spread_ticks > CALIBRATION_MAX_SPREAD_TICKS) {
        USART2_BufInit();
        USART2_BufPrint("[CAL] S=");
        USART2_BufPrintFloat(
            (float)spread_ticks * TIM3_CAPTURE_TICK_US);
        USART2_BufPrint("\r\n");
        USART2_BufFlush();
        return 0U;
    }

    for (i = CALIBRATION_PAIR_TRIM_COUNT;
         i < (CALIBRATION_PAIR_SAMPLE_TARGET -
              CALIBRATION_PAIR_TRIM_COUNT);
         ++i) {
        sum_ticks += cal_samples[i];
    }

    mean_ticks = (uint32_t)(
        (sum_ticks +
         ((CALIBRATION_PAIR_SAMPLE_TARGET -
           2U * CALIBRATION_PAIR_TRIM_COUNT) / 2U)) /
        (CALIBRATION_PAIR_SAMPLE_TARGET -
         2U * CALIBRATION_PAIR_TRIM_COUNT));

    local_tof_us = (float)mean_ticks * TIM3_CAPTURE_TICK_US;
    local_distance_mm = DistanceFromRawTofMm(local_tof_us);

    if (!isfinite(local_tof_us) || !isfinite(local_distance_mm) ||
        local_tof_us <= 0.0f || local_distance_mm <= 0.0f) {
        USART2_Print("[CAL] VAL\r\n");
        return 0U;
    }

    /* Специальная серия не должна попадать в рабочее скользящее окно. */
    memset(measurement_window, 0, sizeof(measurement_window));
    memset(measurement_pair_fallback, 0,
           sizeof(measurement_pair_fallback));
    measurement_window_count = 0U;
    measurement_window_index = 0U;
    measurement_fallback_count = 0U;
    UpdateCaptureCoilFault(0U);

    *tof_us = local_tof_us;
    *distance_mm = local_distance_mm;

    USART2_BufInit();
    USART2_BufPrint("[CAL] OK T=");
    USART2_BufPrintFloat(local_tof_us);
    USART2_BufPrint(" A=");
    USART2_BufPrintInt(attempts);
#if (EMERGENCY_SINGLE_PULSE_DEBUG_MODE != 0U)
    USART2_BufPrint(" EMG1P");
#else
    USART2_BufPrint(" 1P=");
    USART2_BufPrintInt(single_count);
#endif
    USART2_BufPrint(" s=");
    USART2_BufPrintFloat(
        (float)spread_ticks * TIM3_CAPTURE_TICK_US);
    USART2_BufPrint("\r\n");
    USART2_BufFlush();

    return 1U;
}


/**
 * @brief Подавляет одиночные скачки уже рассчитанного уровня.
 *
 * Функция не участвует в калибровке и не изменяет сохранённый ToFraw.
 * Она применяется только к готовому кандидату уровня перед публикацией в
 * Modbus и USART2.
 *
 * Алгоритм:
 *  - первое корректное значение принимается сразу;
 *  - изменение не более LEVEL_JUMP_CONFIRM_THRESHOLD_MM принимается сразу;
 *  - больший скачок временно удерживается как кандидат;
 *  - три последовательных кандидата с той же стороны от опубликованного
 *    значения подтверждают реальное перемещение;
 *  - если следующий результат вернулся к прежнему уровню, одиночный скачок
 *    считается помехой и отбрасывается.
 *
 * Проверка «с той же стороны» позволяет не замораживать показания при
 * реальном непрерывном перемещении поплавка. Допуск
 * LEVEL_JUMP_CONFIRM_TOLERANCE_MM используется для устойчивого ступенчатого
 * изменения, но не является обязательным при монотонном движении.
 */
static float ConfirmMeasuredLevel(float candidate_mm)
{
    float jump_mm;
    float pending_difference_mm;
    uint8_t same_side;

    if (!isfinite(candidate_mm)) {
        return published_filtered_level_mm;
    }

    if (published_level_initialized == 0U) {
        published_filtered_level_mm = candidate_mm;
        pending_level_mm = candidate_mm;
        pending_level_count = 0U;
        pending_spike_reported = 0U;
        published_level_initialized = 1U;
        return candidate_mm;
    }

    jump_mm = fabsf(candidate_mm - published_filtered_level_mm);

    /* Обычные небольшие изменения принимаются без дополнительной задержки. */
    if (jump_mm <= LEVEL_JUMP_CONFIRM_THRESHOLD_MM) {
        if (pending_level_count != 0U && pending_spike_reported != 0U &&
            !ModBus_CommunicationIsBusy()) {
            USART2_Print("[SPK] DROP\r\n");
        }

        published_filtered_level_mm = candidate_mm;
        pending_level_mm = candidate_mm;
        pending_level_count = 0U;
        pending_spike_reported = 0U;
        return candidate_mm;
    }

    /* Первый большой скачок только запоминается. */
    if (pending_level_count == 0U) {
        pending_level_mm = candidate_mm;
        pending_level_count = 1U;
        pending_spike_reported = 1U;

        if (!ModBus_CommunicationIsBusy()) {
            USART2_BufInit();
            USART2_BufPrint("[SPK] c=");
            USART2_BufPrintFloat(candidate_mm);
            USART2_BufPrint(" p=");
            USART2_BufPrintFloat(published_filtered_level_mm);
            USART2_BufPrint(" c=1/");
            USART2_BufPrintInt(LEVEL_JUMP_CONFIRM_COUNT);
            USART2_BufPrint("\r\n");
            USART2_BufFlush();
        }

        return published_filtered_level_mm;
    }

    pending_difference_mm = fabsf(candidate_mm - pending_level_mm);
    same_side =
        ((candidate_mm > published_filtered_level_mm &&
          pending_level_mm > published_filtered_level_mm) ||
         (candidate_mm < published_filtered_level_mm &&
          pending_level_mm < published_filtered_level_mm)) ? 1U : 0U;

    /* Подтверждаем либо устойчивую ступень, либо последовательное движение
     * в одну сторону. Смена стороны начинает подтверждение заново. */
    if (pending_difference_mm <= LEVEL_JUMP_CONFIRM_TOLERANCE_MM ||
        same_side != 0U) {
        pending_level_mm = candidate_mm;
        if (pending_level_count < 0xFFU) {
            ++pending_level_count;
        }
    } else {
        pending_level_mm = candidate_mm;
        pending_level_count = 1U;
    }

    if (pending_level_count >= LEVEL_JUMP_CONFIRM_COUNT) {
        float previous_level_mm = published_filtered_level_mm;

        published_filtered_level_mm = candidate_mm;
        pending_level_mm = candidate_mm;
        pending_level_count = 0U;
        pending_spike_reported = 0U;

        if (!ModBus_CommunicationIsBusy()) {
            USART2_BufInit();
            USART2_BufPrint("[LVL] JUMP ");
            USART2_BufPrintFloat(previous_level_mm);
            USART2_BufPrint(" -> ");
            USART2_BufPrintFloat(candidate_mm);
            USART2_BufPrint(" mm\r\n");
            USART2_BufFlush();
        }
    }

    return published_filtered_level_mm;
}

/**
 * @brief Переводит время пролёта в уровень жидкости, отсчитанный от дна.
 *
 * Приоритет расчёта:
 * 1. Трёхточечная таблица ToFraw команд 11/12/13:
 *    260, 520 и 780 мм. Команды 01/02 задают только границы анализа.
 * 2. Геометрический аварийный расчёт по скорости волны и длине звукопровода.
 *
 * Параметры C1/C2 сохранены в карте Modbus для совместимости с ПМП-201Е,
 * но намеренно не участвуют в измерительном пересчёте. Ранее оставшийся
 * двухточечный участок C1/C2 создавал скрытое смещение около 20 мм, когда
 * трёхточечная таблица временно не активировалась.
 */
static float Calculate_Position(float raw_tof_us)
{
    float measured_distance_mm = DistanceFromRawTofMm(raw_tof_us);
    float usable_height_mm = GetUsableLevelHeightMm();
    float level_mm;

    /* Основной и единственный калиброванный режим — точки 11/12/13. */
    level_mm = InterpolateCalibratedLevel(raw_tof_us);
    if (isfinite(level_mm) && level_mm >= 0.0f) {
        if (level_mm > usable_height_mm) {
            level_mm = usable_height_mm;
        }
        return level_mm;
    }

    /*
     * Аварийный расчёт используется только при отсутствующей либо
     * немонотонной таблице 11/12/13. C1/C2 здесь не применяются.
     */
    level_mm =
        ModBus_GetWaveguideLength() * 1000.0f -
        measured_distance_mm;

    if (!isfinite(level_mm) || level_mm < 0.0f) {
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
 * Для 01/02 сохраняются границы анализа, для 11/12/13 — точки калибровки.
 * Все значения к этому моменту уже записаны в RAM и помечены как
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
        USART2_BufPrint("[CAL] ");
        USART2_BufPrintInt(command_save_code);
        USART2_BufPrint(": SAVE OK\r\n");
        USART2_BufFlush();

        command_save_pending = 0U;
        command_save_code = 0U;
        ModBus_ClearStorageSaveStatus();
    } else if (state == MODBUS_STORAGE_SAVE_ERROR) {
        ModBus_SetParameter_Int(MB_ADDR_COMMAND, 0U);
        USART2_BufInit();
        USART2_BufPrint("[CAL] ");
        USART2_BufPrintInt(command_save_code);
        USART2_BufPrint(": SAVE ERR=");
        USART2_BufPrint(
            ParamsStorage_ErrorToString(ParamsStorage_GetLastError()));
        USART2_BufPrint("\r\n");
        USART2_BufFlush();

        command_save_pending = 0U;
        command_save_code = 0U;
        ModBus_ClearStorageSaveStatus();
    }
}

/**
 * @brief Выполняет безопасный программный перезапуск STM32 по команде 224.
 *
 * Сброс откладывается минимум на 500 мс, чтобы завершились передача Modbus,
 * диагностическое сообщение USART2 и возможная фоновая запись AT24C64.
 * NVIC_SystemReset() перезапускает ядро и периферию STM32, но не снимает
 * питание с внешней EEPROM.
 */
static void ProcessSoftwareReset(void)
{
    if (software_reset_pending == 0U) {
        return;
    }

    if ((uint32_t)(HAL_GetTick() - software_reset_requested_ms) < 500U) {
        return;
    }

    if (ModBus_CommunicationIsBusy() ||
        ModBus_StorageIsBusy() ||
        !USART2_TxIsIdle()) {
        return;
    }

    __disable_irq();
    NVIC_SystemReset();

    while (1) {
        /* После NVIC_SystemReset() выполнение сюда не должно вернуться. */
    }
}

/**
 * @brief Проверяет правдоподобие участка калибровки по расчётной скорости.
 *
 * Формула использует разность уровней 260 мм и разность ToF соседних точек.
 * Проверка нужна только против ошибочной команды, например когда команда 12
 * была отправлена при физическом положении 780 мм. В таком случае скорость
 * получается около 1500 м/с, и точка не должна попадать в EEPROM.
 */
static uint8_t CalibrationSegmentIsPlausible(float previous_tof_us,
                                              float current_tof_us)
{
    float delta_t_us;
    float speed_mps;

    if (!isfinite(previous_tof_us) || !isfinite(current_tof_us)) {
        return 0U;
    }

    delta_t_us = previous_tof_us - current_tof_us;
    if (delta_t_us <= 0.0f) {
        return 0U;
    }

    /* 260 мм / delta_t[мкс] = 260000 / delta_t м/с. */
    speed_mps = 260000.0f / delta_t_us;

    return (speed_mps >= CALIBRATION_SEGMENT_SPEED_MIN_MPS &&
            speed_mps <= CALIBRATION_SEGMENT_SPEED_MAX_MPS) ? 1U : 0U;
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
            USART2_Print("[CAL] L=");
            USART2_BufInit();
            USART2_BufPrintFloat(waveguide_len * 1000.0f);
            USART2_BufPrint(" mm\r\n");
            USART2_BufFlush();
            break;
        }
        case 2: {
            float tof_us;
            float distance_mm;

            if (GetStableCalibrationSample(&tof_us, &distance_mm) != 0U) {
                uint16_t mask =
                    ModBus_GetParameter_Int(MB_ADDR_SENSOR_CAL_MASK);
                float tof_780 =
                    ModBus_GetParameter_Float(MB_ADDR_SENSOR_CAL_780);

                if ((mask & SENSOR_CAL_STORAGE_TAG_MASK) !=
                    SENSOR_CAL_STORAGE_TAG) {
                    mask = (uint16_t)(SENSOR_CAL_STORAGE_TAG |
                                      (mask & SENSOR_CAL_FULL_MASK));
                }

                /* Команда 02 задаёт только верхнюю границу анализа.
                 * Она не меняет C2 и не становится четвёртым узлом формулы. */
                if ((mask & SENSOR_CAL_POINT_780_BIT) != 0U &&
                    (!isfinite(tof_780) || tof_us >= tof_780)) {
                    command_succeeded = 0U;
                    USART2_Print("[CAL] 02 BND\r\n");
                } else {
                    ModBus_SetParameter_Float(
                        MB_ADDR_SENSOR_CAL_HIGH_TOF, tof_us);
                    ModBus_SetParameter_Int(
                        MB_ADDR_SENSOR_CAL_MASK,
                        (uint16_t)(mask | SENSOR_CAL_POINT_HIGH_BIT));
                    requires_eeprom_commit = 1U;

                    USART2_BufInit();
                    USART2_BufPrint("[CAL] 02 T=");
                    USART2_BufPrintFloat(tof_us);
                    USART2_BufPrint(" D=");
                    USART2_BufPrintFloat(distance_mm);
                    USART2_BufPrint("\r\n");
                    USART2_BufFlush();
                }
            } else {
                command_succeeded = 0U;
                USART2_Print("[CAL] 02 FAIL\r\n");
            }
            break;
        }
        case 1: {
            float tof_us;
            float distance_mm;

            if (GetStableCalibrationSample(&tof_us, &distance_mm) != 0U) {
                uint16_t mask =
                    ModBus_GetParameter_Int(MB_ADDR_SENSOR_CAL_MASK);
                float tof_260 =
                    ModBus_GetParameter_Float(MB_ADDR_SENSOR_CAL_260);

                if ((mask & SENSOR_CAL_STORAGE_TAG_MASK) !=
                    SENSOR_CAL_STORAGE_TAG) {
                    mask = (uint16_t)(SENSOR_CAL_STORAGE_TAG |
                                      (mask & SENSOR_CAL_FULL_MASK));
                }

                /* Команда 01 задаёт только нижнюю границу анализа.
                 * Она не меняет C1 и не очищает точки 11/12/13. */
                if ((mask & SENSOR_CAL_POINT_260_BIT) != 0U &&
                    (!isfinite(tof_260) || tof_us <= tof_260)) {
                    command_succeeded = 0U;
                    USART2_Print("[CAL] 01 BND\r\n");
                } else {
                    ModBus_SetParameter_Float(
                        MB_ADDR_SENSOR_CAL_LOW_TOF, tof_us);
                    ModBus_SetParameter_Int(
                        MB_ADDR_SENSOR_CAL_MASK,
                        (uint16_t)(mask | SENSOR_CAL_POINT_LOW_BIT));
                    requires_eeprom_commit = 1U;

                    USART2_BufInit();
                    USART2_BufPrint("[CAL] 01 T=");
                    USART2_BufPrintFloat(tof_us);
                    USART2_BufPrint(" D=");
                    USART2_BufPrintFloat(distance_mm);
                    USART2_BufPrint("\r\n");
                    USART2_BufFlush();
                }
            } else {
                command_succeeded = 0U;
                USART2_Print("[CAL] 01 FAIL\r\n");
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

                    /*
                     * Команда 11 начинает новый цикл промежуточной
                     * калибровки, но не удаляет границы 01/02. Эти границы
                     * не участвуют в наклоне формулы и могут быть заданы до
                     * либо после команд 11/12/13.
                     */
                    if ((mask & SENSOR_CAL_STORAGE_TAG_MASK) !=
                        SENSOR_CAL_STORAGE_TAG) {
                        mask = (uint16_t)(SENSOR_CAL_STORAGE_TAG |
                                          (mask & SENSOR_CAL_FULL_MASK));
                    }

                    mask = (uint16_t)(SENSOR_CAL_STORAGE_TAG |
                        (mask & (SENSOR_CAL_POINT_LOW_BIT |
                                 SENSOR_CAL_POINT_HIGH_BIT)));

                    /* Старые точки 520/780 удаляются, чтобы незавершённый
                     * новый цикл не был принят за готовую таблицу. */
                    ModBus_SetParameter_Float(
                        MB_ADDR_SENSOR_CAL_520, 0.0f);
                    ModBus_SetParameter_Float(
                        MB_ADDR_SENSOR_CAL_780, 0.0f);
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
                    USART2_Print("[CAL] ORDER\r\n");
                    break;
                }

                if (required_previous_bit != 0U &&
                    (!isfinite(previous_tof_us) || tof_us >= previous_tof_us)) {
                    command_succeeded = 0U;
                    USART2_BufInit();
                    USART2_BufPrint("[CAL] MONO T=");
                    USART2_BufPrintFloat(tof_us);
                    USART2_BufPrint(">=");
                    USART2_BufPrintFloat(previous_tof_us);
                    USART2_BufPrint("\r\n");
                    USART2_BufFlush();
                    break;
                }

                /*
                 * Команды 12 и 13 должны соответствовать соседним точкам,
                 * разнесённым ровно на 260 мм. Ранее код проверял только
                 * монотонность, поэтому команда 12, случайно выполненная в
                 * положении 780 мм, принималась и записывала T520≈111 мкс.
                 */
                if (required_previous_bit != 0U &&
                    CalibrationSegmentIsPlausible(previous_tof_us, tof_us) == 0U) {
                    command_succeeded = 0U;
                    USART2_Print("[CAL] SPD\r\n");
                    break;
                }

                ModBus_SetParameter_Float(address, tof_us);

                /*
                 * После команды 13 принудительно формируем полную маску 3P.
                 * Это исключает ситуацию, когда T260/T520/T780 записаны, но
                 * один из служебных битов потерян и расчёт остаётся FALLBACK.
                 */
                if (cmd == 13U) {
                    mask = (uint16_t)(SENSOR_CAL_STORAGE_TAG |
                        (mask & (SENSOR_CAL_POINT_LOW_BIT |
                                 SENSOR_CAL_POINT_HIGH_BIT)) |
                        SENSOR_CAL_POINT_260_BIT |
                        SENSOR_CAL_POINT_520_BIT |
                        SENSOR_CAL_POINT_780_BIT);
                } else {
                    mask = (uint16_t)(mask | bit);
                }

                ModBus_SetParameter_Int(MB_ADDR_SENSOR_CAL_MASK, mask);
                requires_eeprom_commit = 1U;

                USART2_BufInit();
                USART2_BufPrint("[CAL] ");
                USART2_BufPrintInt(cmd);
                USART2_BufPrint(" L=");
                USART2_BufPrintFloat(reference_level_mm);
                USART2_BufPrint(" T=");
                USART2_BufPrintFloat(tof_us);
                USART2_BufPrint(" D=");
                USART2_BufPrintFloat(distance_mm);
                USART2_BufPrint(" M=");
                USART2_BufPrintInt((uint16_t)(mask &
                                              SENSOR_CAL_FULL_MASK));
                USART2_BufPrint("\r\n");
                USART2_BufFlush();
            } else {
                command_succeeded = 0U;
                USART2_Print("[CAL] CAP FAIL\r\n");
            }
            break;
        }

        case 224:
            /*
             * Диагностический программный перезапуск STM32.
             * Параметры автоматически не сохраняются: перед командой 224
             * необходимо дождаться статуса 90 от команды 223 либо от
             * последней калибровочной команды.
             */
            ModBus_SetParameter_Int(MB_ADDR_COMMAND, 90U);
            USART2_Print(
                "[SYS] RESET\r\n");
            software_reset_requested_ms = HAL_GetTick();
            software_reset_pending = 1U;
            return;

        case 223:
            /*
             * Таблица Е.10: сохранить пользовательские настройки.
             * Здесь не выполняется синхронная запись: адрес 3000 остается 85,
             * а итог 90/0 формируется после ответа фонового драйвера AT24C64.
             */
            requires_eeprom_commit = 1U;
            USART2_Print("[CFG] SAVE\r\n");
            break;

        case 4:
            USART2_Print("[CAL] CMD04\r\n");
            break;
        default:
            command_succeeded = 0U;
            break;
    }

    /*
     * Для 01/02, 11/12/13 и 223 код 90 нельзя выдавать до подтверждения EEPROM:
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
        USART2_Print(" V (d=");
        if (old_val > 0.0f) { USART2_PrintFloat(diff); }
        else { USART2_Print("init"); }
        USART2_Print(")\r\n");
        *store_val = new_val;
    }
}

static void Update_Poll_Period_From_Modbus(void)
{
    static uint8_t rate_sync_initialized = 0U;
    static float last_period_ms = 0.0f;
    static float last_frequency_hz = 0.0f;
    float period_value = ModBus_GetParameter_Float(MB_ADDR_POLL_PERIOD);
    float frequency_value =
        ModBus_GetParameter_Float(MB_ADDR_EXCITATION_FREQUENCY);
    uint8_t period_changed;
    uint8_t frequency_changed;

    /*
     * 2088...2089 хранит совместимый период в мс и сохраняется в EEPROM.
     * 2118...2119 — удобное оперативное представление того же параметра в Гц.
     * После загрузки EEPROM период имеет приоритет, поэтому старые настройки
     * автоматически продолжают работать и пересчитываются в частоту.
     */
    if (!isfinite(period_value)) {
        period_value = (float)PULSE_PERIOD_MS_DEFAULT;
    }
    if (period_value < (float)MIN_POLL_PERIOD_MS) {
        period_value = (float)MIN_POLL_PERIOD_MS;
    }
    if (period_value > (float)MAX_POLL_PERIOD_MS) {
        period_value = (float)MAX_POLL_PERIOD_MS;
    }

    if (rate_sync_initialized == 0U) {
        current_poll_period_ms = (uint32_t)(period_value + 0.5f);
        frequency_value = 1000.0f / (float)current_poll_period_ms;
        ModBus_SetParameter_Float(MB_ADDR_POLL_PERIOD,
                                  (float)current_poll_period_ms);
        ModBus_SetParameter_Float(MB_ADDR_EXCITATION_FREQUENCY,
                                  frequency_value);
        last_period_ms = (float)current_poll_period_ms;
        last_frequency_hz = frequency_value;
        rate_sync_initialized = 1U;
        return;
    }

    period_changed =
        (fabsf(period_value - last_period_ms) > 0.01f) ? 1U : 0U;
    frequency_changed =
        (isfinite(frequency_value) &&
         fabsf(frequency_value - last_frequency_hz) > 0.0001f) ? 1U : 0U;

    if (frequency_changed != 0U && period_changed == 0U) {
        /* Запись по 2118: прямое задание частоты в Гц. */
        if (frequency_value < MODBUS_MIN_EXCITATION_FREQUENCY_HZ) {
            frequency_value = MODBUS_MIN_EXCITATION_FREQUENCY_HZ;
        }
        if (frequency_value > MODBUS_MAX_EXCITATION_FREQUENCY_HZ) {
            frequency_value = MODBUS_MAX_EXCITATION_FREQUENCY_HZ;
        }
        period_value = 1000.0f / frequency_value;
    }

    /* Если изменены оба значения одновременно, совместимый адрес 2088 имеет
     * приоритет. Это сохраняет поведение существующего внешнего ПО. */
    if (period_value < (float)MIN_POLL_PERIOD_MS) {
        period_value = (float)MIN_POLL_PERIOD_MS;
    }
    if (period_value > (float)MAX_POLL_PERIOD_MS) {
        period_value = (float)MAX_POLL_PERIOD_MS;
    }

    current_poll_period_ms = (uint32_t)(period_value + 0.5f);
    frequency_value = 1000.0f / (float)current_poll_period_ms;

    ModBus_SetParameter_Float(MB_ADDR_POLL_PERIOD,
                              (float)current_poll_period_ms);
    ModBus_SetParameter_Float(MB_ADDR_EXCITATION_FREQUENCY,
                              frequency_value);

    last_period_ms = (float)current_poll_period_ms;
    last_frequency_hz = frequency_value;
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
        USART2_BufPrint(" | DR=");
        USART2_BufPrintFloat(distance_from_top_raw_mm);
        USART2_BufPrint(" | M=");
        USART2_BufPrintInt((uint16_t)(sensor_cal_mask & SENSOR_CAL_FULL_MASK));
        if (sensor_cal_mode == 4U) {
            USART2_BufPrint(" C=3P+B");
        } else if (sensor_cal_mode == 3U) {
            USART2_BufPrint(" C=3P");
        } else {
            USART2_BufPrint(" C=RAW");
        }
        USART2_BufPrint(" | ToFraw=");
        USART2_BufPrintFloat(tof_us);
        USART2_BufPrint(" | T=");
        USART2_BufPrintFloat(corrected_tof_us);
        USART2_BufPrint(" | Tc=");
        USART2_BufPrintFloat(current_temperature);
        USART2_BufPrint(" | c3=");
        if (calculated_wave_speed_valid != 0U) {
            USART2_BufPrintFloat(calculated_wave_speed);
            USART2_BufPrint("");
        } else {
            USART2_BufPrint("N/A");
        }
        USART2_BufPrint(" | cc=");
        USART2_BufPrintFloat(wave_speed);
        USART2_BufPrint("");
#if (EMERGENCY_SINGLE_PULSE_DEBUG_MODE != 0U)
        USART2_BufPrint(" | EMG1P");
#else
        if (capture_coil_fault_active != 0U) {
            USART2_BufPrint(" | 1P CF");
        } else {
            USART2_BufPrint(" | PAIR");
        }
#endif
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
#if (USART2_DEBUG_ENABLED != 0U)
    MX_USART2_UART_Init();
#endif
    MX_ADC1_Init();
    MX_ADC2_Init();

    if (MX_I2C2_Init() == HAL_OK) {
        USART2_Print("[I2C] OK\r\n");
        current_temperature = 0.0f;
    } else {
        USART2_Print("[I2C] ERR\r\n");
    }

    TIM3_InputCapture_Init();

    USART2_Print("[ADC] INIT\r\n");
    if (HAL_ADCEx_Calibration_Start(&hadc1) == HAL_OK)
        USART2_Print("[ADC1] OK\r\n");
    else {
        v24_error = 1; v12_error = 1; vdda_error = 1;
    }

    if (HAL_ADCEx_Calibration_Start(&hadc2) == HAL_OK)
        USART2_Print("[ADC2] OK\r\n");
    else {
        v12_error = 1;
    }

    ModBus_Init();

    /*
     * Нормализуем состояние трёхточечной таблицы после загрузки EEPROM.
     * Восстановление выполняется по полной монотонной тройке ToFraw даже
     * тогда, когда сигнатура 0xA500 присутствует, но биты маски неполны.
     */
    {
        uint8_t cal_mask_recovered = RestoreThreePointCalibrationState();
        if (cal_mask_recovered != 0U) {
            USART2_Print("[CAL] MASK OK\r\n");
        }
    }

    /* После включения авария катушки фиксации считается снятой. Она будет
     * установлена при первом запуске без корректного второго импульса. */
    ModBus_SetParameter_Int(MB_ADDR_ERROR_CODE, MEASUREMENT_ERROR_NONE);
    current_measurement_status = MEASUREMENT_STATUS_VALID;

    ModBus_UpdateFirmwareVersion(FIRMWARE_VERSION);
#if (EMERGENCY_SINGLE_PULSE_DEBUG_MODE != 0U)
    USART2_Print("[DBG] EMERGENCY 1P MODE\r\n");
#endif
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
#if (USART2_DEBUG_ENABLED != 0U)
    HAL_NVIC_SetPriority(USART2_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
#endif
    __enable_irq();

    Read_All_Voltages();
    Read_Temperature();
    ModBus_UpdateVoltages(current_24v, current_12v);
    Update_Poll_Period_From_Modbus();

    USART2_BufInit();
    USART2_BufPrint("MB A=");
    USART2_BufPrintInt(ModBus_GetDeviceAddress());
    USART2_BufPrint(", Baud=");
    USART2_BufPrintInt(ModBus_GetParameter_Int(MB_ADDR_MB_BAUD_SET));
    USART2_BufPrint("\r\n");
    USART2_BufPrint("[DBG] P=");
    if (current_poll_period_ms >= 1000) {
        USART2_BufPrintInt(current_poll_period_ms / 1000);
        USART2_BufPrint(" s\r\n");
    } else {
        USART2_BufPrintInt(current_poll_period_ms);
        USART2_BufPrint(" ms\r\n");
    }
    USART2_BufPrint("[DBG] c=");
    USART2_BufPrintFloat(ModBus_GetMaterialWaveSpeed());
    USART2_BufPrint(" m/s\r\n");

    {
        uint16_t cal_mask = ModBus_GetParameter_Int(MB_ADDR_SENSOR_CAL_MASK);
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

        USART2_BufPrint("[CAL] EE m=");
        USART2_BufPrintInt(cal_mask);
        USART2_BufPrint(", bits=");
        USART2_BufPrintInt((uint16_t)(cal_mask & SENSOR_CAL_FULL_MASK));
        USART2_BufPrint(", T260=");
        USART2_BufPrintFloat(tof_260);
        USART2_BufPrint(", T520=");
        USART2_BufPrintFloat(tof_520);
        USART2_BufPrint(", T780=");
        USART2_BufPrintFloat(tof_780);
        USART2_BufPrint("\r\n");

        if (last_sensor_cal_mode == 4U) {
            USART2_BufPrint("[CAL] MODE=3P+B\r\n");
        } else if (last_sensor_cal_mode == 3U &&
                   (cal_mask & SENSOR_CAL_STORAGE_TAG_MASK) == SENSOR_CAL_STORAGE_TAG &&
                   (cal_mask & three_point_mask) == three_point_mask) {
            USART2_BufPrint("[CAL] MODE=3P\r\n");
        } else {
            USART2_BufPrint("[CAL] MODE=RAW\r\n");
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

        /* Проверяем обязательную запись 01/02/11/12/13/223. Функция не блокирует
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
                USART2_Print("[CAL] CMD=");
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
                 * После заполнения скользящего окна возвращается робастное
                 * среднее 11->5;
                 * при первоначальном заполнении возвращается текущий t1.
                 * Дополнительный EMA намеренно не применяется. */
                raw_tof_us = (float)measurement * TOF_TICK_US;

                /* Отдельный калибровочный ToF обновляется внутри
                 * measure_time_of_flight() только по устойчивому MEAN. */

                raw_position_mm = Calculate_Position(raw_tof_us);

                /*
                 * Калибровка и геометрия формируют кандидат уровня.
                 * Перед публикацией одиночные скачки подтверждаются тремя
                 * последовательными результатами. Сырой ToF для команд
                 * 11/12/13 остаётся неизменным и хранится до этого фильтра.
                 */
                position_mm = ConfirmMeasuredLevel(raw_position_mm);

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
                USART2_Print("[MEAS] NOPULSE\r\n");
            }
        }

        now = HAL_GetTick();
        if ((uint32_t)(now - last_voltage_read_time) >= 1000U &&
            !ModBus_CommunicationIsBusy()) {
            Read_All_Voltages();
            last_voltage_read_time = HAL_GetTick();
            ModBus_UpdateVoltages(current_24v,
                                  current_12v);
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
         * следующего измерения. Обычная отложенная запись дополнительно
         * ждет освобождения USART2. Для обязательной фиксации калибровочных точек отладочный
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

        /*
         * Команда 224 обрабатывается после Modbus и EEPROM. Системный сброс
         * не должен обрывать ответ внешней программе или постраничную запись.
         */
        ProcessSoftwareReset();

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
    TIM3->PSC = 0U;
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
    float pulse_width_us;
    uint16_t pulse_start_tick;
    uint16_t pulse_width_ticks;
    uint32_t saved_primask;

    /*
     * 2116...2117: длительность задающего импульса PB5, float32, мкс.
     * Параметр читается перед каждым запуском, поэтому новое значение Modbus
     * начинает действовать со следующего импульса без перезапуска.
     *
     * Ширина формируется непосредственно по TIM3 (72 МГц), а не программным
     * NOP-циклом. Поэтому оптимизация -O0/-Og/-Os больше не меняет длительность.
     */
    pulse_width_us = ModBus_GetParameter_Float(MB_ADDR_EXCITATION_PULSE_WIDTH);
    if (!isfinite(pulse_width_us) ||
        pulse_width_us < MODBUS_MIN_EXCITATION_PULSE_WIDTH_US ||
        pulse_width_us > MODBUS_MAX_EXCITATION_PULSE_WIDTH_US) {
        pulse_width_us = MODBUS_DEFAULT_EXCITATION_PULSE_WIDTH_US;
    }
    pulse_width_ticks = (uint16_t)(pulse_width_us *
        ((float)TIM3_CAPTURE_FREQUENCY_HZ / 1000000.0f) + 0.5f);
    if (pulse_width_ticks == 0U) {
        pulse_width_ticks = 1U;
    }

    tof_measurement_done = 0U;
    tof_timeout = 0U;
    capture_count = 0U;

    for (i = 0U; i < MAX_CAPTURED_PULSES; ++i) {
        captured_pulses[i] = 0U;
    }

    /* Сначала переводим аналоговый ключ в исходное состояние. */
    HAL_GPIO_WritePin(SWITCH_PORT, SWITCH_PIN, GPIO_PIN_RESET);

    /* Счетчик обнуляется непосредственно перед импульсом PB5. Благодаря этому
     * BLANKING_WINDOW_TICKS отсчитывает именно первые 80 мкс после отправки,
     * а не время выполнения подготовительного кода. */
    TIM3->CR1 &= ~TIM_CR1_CEN;
    TIM3->SR = 0U;
    TIM3->CNT = 0U;
    __DSB();
    TIM3->CR1 |= TIM_CR1_CEN;
    __DSB();

    /*
     * Задающий импульс PB5. На время высокого уровня запрещаем IRQ максимум
     * на 30 мкс: это исключает растяжение импульса обработчиком USART/SysTick.
     * Сам TIM3 при запрещенных IRQ продолжает считать, поэтому длительность
     * задается аппаратной временной базой 72 МГц. Исходный PRIMASK сохраняется.
     */
    saved_primask = __get_PRIMASK();
    __disable_irq();
    GPIOB->BSRR = Gen_Impuls_Pin;
    pulse_start_tick = (uint16_t)TIM3->CNT;
    while ((uint16_t)((uint16_t)TIM3->CNT - pulse_start_tick) <
           pulse_width_ticks) {
        __NOP();
    }
    GPIOB->BRR = Gen_Impuls_Pin;
    if (saved_primask == 0U) {
        __enable_irq();
    }

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
 * Нижняя граница фиксирована: первые 80 мкс подавлены как наводка.
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
 * Генерация ограничена текущей частотой из Modbus независимо от того, кто вызвал измерение:
 * основной цикл, команда калибровки или повторный запуск. Во время ожидания
 * рабочий Modbus и неблокирующий USART2 продолжают обслуживаться.
 */
static void WaitForNextExcitationSlot(void)
{
    if (excitation_time_initialized != 0U) {
        while ((uint32_t)(HAL_GetTick() - last_excitation_time_ms) <
               current_poll_period_ms) {
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
#if (EMERGENCY_SINGLE_PULSE_DEBUG_MODE != 0U)
    /* В аварийной стендовой сборке отсутствие второго импульса ожидаемо. */
    fault_active = 0U;
#else
    fault_active = (fault_active != 0U) ? 1U : 0U;
#endif

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
                "[COIL] ERR\r\n");
        }
    } else {
        ModBus_SetParameter_Int(MB_ADDR_ERROR_CODE,
                                MEASUREMENT_ERROR_NONE);
        if (!ModBus_CommunicationIsBusy()) {
            USART2_Print(
                "[COIL] OK\r\n");
        }
    }
}

/**
 * @brief Добавляет новый ToF в скользящее окно из 20 запусков.
 *
 * После заполнения окна самый старый элемент заменяется новым. Одновременно
 * ведется количество запусков, где использовался только первый импульс.
 * Пока хотя бы один такой запуск остается в последних 20 измерениях, авария
 * катушки остается активной. Для снятия аварии нужно полное окно корректных пар подряд.
 */
static void AddMeasurementToWindow(uint32_t tof_ticks,
                                   uint8_t used_single_pulse)
{
#if (EMERGENCY_SINGLE_PULSE_DEBUG_MODE != 0U)
    /* PAIR/1P не участвует в аварийной отладке: окно содержит только t1. */
    used_single_pulse = 0U;
#else
    used_single_pulse = (used_single_pulse != 0U) ? 1U : 0U;
#endif

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
 *  - после защитного окна 80 мкс фиксируется первый импульс t1;
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

#if (EMERGENCY_SINGLE_PULSE_DEBUG_MODE != 0U)
    /*
     * Аварийная отладка: первый валидный t1 является достаточным результатом.
     * quality оставляем SINGLE только как диагностический факт; рабочее окно
     * и калибровка при этом не требуют PAIR.
     */
    result.quality = LAUNCH_SINGLE_PULSE_NO_SECOND;
    return result;
#else
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
#endif

    return result;
}

/**
 * @brief Обновляет скользящую статистику уровня одним запуском на каждый вызов.
 *
 * Каждый вызов формирует один импульс. Глобальный ограничитель использует
 * текущий период 2088 / частоту 2118; аппаратный максимум ограничен 20 Гц.
 *
 * После заполнения окна из 20 запусков выполняются медианный MAD-фильтр и
 * адаптивная линейная аппроксимация. При неподвижном уровне используется
 * робастное среднее, при движении — оценка на конце временного окна.
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

#if (EMERGENCY_SINGLE_PULSE_DEBUG_MODE != 0U)
    used_single_pulse = 0U;
#else
    used_single_pulse =
        (launch.quality == LAUNCH_VALID_PAIR) ? 0U : 1U;
#endif
    AddMeasurementToWindow(launch.tof_ticks, used_single_pulse);

    /*
     * При частоте по умолчанию 10 Гц окно заполняется примерно за две секунды.
     * До заполнения публикуется текущий t1, чтобы после включения уровень появился сразу.
     */
    if (measurement_window_count < MEASUREMENT_REQUIRED_SAMPLES) {
        tof_timeout = 0U;
        return launch.tof_ticks;
    }

    /*
     * Для линейной аппроксимации порядок отсчётов принципиален.
     * measurement_window_index указывает на ячейку, которая будет заменена
     * следующей, то есть на самый старый элемент полного кольцевого окна.
     */
    for (i = 0U; i < MEASUREMENT_REQUIRED_SAMPLES; ++i) {
        uint8_t source_index = (uint8_t)(measurement_window_index + i);
        if (source_index >= MEASUREMENT_REQUIRED_SAMPLES) {
            source_index =
                (uint8_t)(source_index - MEASUREMENT_REQUIRED_SAMPLES);
        }
        ordered_samples[i] = measurement_window[source_index];
    }

    if (!MeasurementStatistics_Calculate(
            ordered_samples,
            MEASUREMENT_REQUIRED_SAMPLES,
            MEASUREMENT_MAX_SPREAD_TICKS,
            MEASUREMENT_OUTLIER_MIN_GATE_TICKS,
            MEASUREMENT_APPROX_SLOPE_LOW_MILLI_TICKS,
            MEASUREMENT_APPROX_SLOPE_HIGH_MILLI_TICKS,
            &statistics)) {
        tof_timeout = 1U;

        if (!ModBus_CommunicationIsBusy()) {
            USART2_BufInit();
            USART2_BufPrint("[STAT] REJ u=");
            USART2_BufPrintInt(statistics.used_samples);
            USART2_BufPrint("/");
            USART2_BufPrintInt(MEASUREMENT_REQUIRED_SAMPLES);
            USART2_BufPrint(" m=");
            USART2_BufPrintFloat(
                (float)statistics.mad_ticks * TIM3_CAPTURE_TICK_US);
            USART2_BufPrint(" s=");
            USART2_BufPrintFloat(
                (float)statistics.spread_ticks * TIM3_CAPTURE_TICK_US);
            USART2_BufPrint(" 1p=");
            USART2_BufPrintInt(measurement_fallback_count);
            USART2_BufPrint("/");
            USART2_BufPrintInt(MEASUREMENT_REQUIRED_SAMPLES);
            USART2_BufPrint("\r\n");
            USART2_BufFlush();
        }
        return 0U;
    }

    tof_timeout = 0U;

    if (!ModBus_CommunicationIsBusy()) {
        float trend_us_per_second =
            ((float)statistics.slope_milli_ticks_per_sample / 1000.0f) *
            TIM3_CAPTURE_TICK_US *
            (1000.0f / (float)current_poll_period_ms);

        USART2_BufInit();
        USART2_BufPrint("[STAT] OK 20>");
        USART2_BufPrintInt(statistics.used_samples);
        USART2_BufPrint(" m=");
        USART2_BufPrintFloat(
            (float)statistics.mad_ticks * TIM3_CAPTURE_TICK_US);
        USART2_BufPrint(" s=");
        USART2_BufPrintFloat(
            (float)statistics.spread_ticks * TIM3_CAPTURE_TICK_US);
        USART2_BufPrint(" tr=");
        USART2_BufPrintFloat(trend_us_per_second);
        USART2_BufPrint(" ap=");
        USART2_BufPrint(statistics.regression_used != 0U ? "LIN" : "MEAN");
        USART2_BufPrint(" p=");
        USART2_BufPrintInt(
            MEASUREMENT_REQUIRED_SAMPLES - measurement_fallback_count);
        USART2_BufPrint("/");
        USART2_BufPrintInt(MEASUREMENT_REQUIRED_SAMPLES);
        USART2_BufPrint(", single=");
        USART2_BufPrintInt(measurement_fallback_count);
        USART2_BufPrint("/");
        USART2_BufPrintInt(MEASUREMENT_REQUIRED_SAMPLES);
        USART2_BufPrint("\r\n");
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
    uint32_t adc_raw_vdda = 0, adc_raw_24v = 0, adc_raw_12v = 0;

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
