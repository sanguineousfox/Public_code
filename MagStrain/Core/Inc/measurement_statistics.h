/**
 * @file    measurement_statistics.h
 * @brief   Робастная обработка и адаптивная линейная аппроксимация ToF.
 *
 * Алгоритм рассчитан на скользящее окно из 20 последовательных измерений:
 *  1. вычисляется медиана времени первого импульса t1;
 *  2. вычисляется медианное абсолютное отклонение MAD;
 *  3. значения, удалённые от медианы дальше робастного порога, исключаются;
 *  4. по оставшимся отсчётам вычисляются среднее и линейная регрессия;
 *  5. при неподвижном уровне публикуется робастное среднее с минимальным шумом;
 *  6. при устойчивом движении результат плавно приближается к значению
 *     регрессии в конце окна, уменьшая запаздывание фильтра.
 *
 * Обычная полиномиальная аппроксимация без предварительного исключения
 * выбросов здесь намеренно не используется: одиночный ложный фронт способен
 * заметно изменить коэффициенты полинома и создать дополнительный всплеск.
 */
#ifndef MEASUREMENT_STATISTICS_H
#define MEASUREMENT_STATISTICS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define MEASUREMENT_STATISTICS_SAMPLE_COUNT       20U
#define MEASUREMENT_STATISTICS_MIN_ACCEPTED       12U
#define MEASUREMENT_STATISTICS_MAD_FACTOR         4U
#define MEASUREMENT_STATISTICS_BLEND_SCALE        1000U
#define MEASUREMENT_LEVEL_BELOW_ZERO_TOLERANCE_MM 2.0f
#define MEASUREMENT_PULSE_WINDOW_COUNT 10U

typedef struct {
    uint16_t first_ticks;
    uint16_t second_ticks;
    bool valid_pair;
} MeasurementPulseSample_t;

typedef struct {
    MeasurementPulseSample_t samples[MEASUREMENT_PULSE_WINDOW_COUNT];
    uint32_t last_motion_ms;
    uint16_t sequence;
    bool moving;
} MeasurementPulseWindow_t;

enum {
    MEASUREMENT_PULSE_MODE_SINGLE = 0,
    MEASUREMENT_PULSE_MODE_PAIR = 1,
    MEASUREMENT_PULSE_MODE_AUTO = 2
};

enum {
    MEASUREMENT_PULSE_STATE_SINGLE = 0,
    MEASUREMENT_PULSE_STATE_PAIR = 1,
    MEASUREMENT_PULSE_STATE_MOVING = 2
};

uint16_t MeasurementStatistics_UpdatePulseWindow(MeasurementPulseWindow_t *window,
    MeasurementPulseSample_t sample, uint16_t mode, uint32_t now_ms);

typedef struct {
    uint32_t accepted[MEASUREMENT_PULSE_WINDOW_COUNT];
    uint32_t initial[5];
    uint32_t candidate_sum;
    uint16_t candidate;
    uint8_t accepted_count;
    uint8_t initial_count;
    uint8_t matches;
    uint8_t misses;
} MeasurementT1Filter_t;

typedef enum {
    MEASUREMENT_T1_PENDING,
    MEASUREMENT_T1_ACCEPTED,
    MEASUREMENT_T1_STEP
} MeasurementT1Decision_t;

MeasurementT1Decision_t MeasurementStatistics_FilterT1(MeasurementT1Filter_t *filter,
    const MeasurementPulseWindow_t *window, uint32_t *accepted_ticks);

typedef struct {
    float mean_ticks;         /* Робастное среднее или адаптивная аппроксимация. */
    float robust_mean_ticks;  /* Среднее принятых отсчётов с дробной частью. */
    uint32_t median_ticks;     /* Медиана полного окна из 20 значений. */
    uint32_t mad_ticks;        /* Медианное абсолютное отклонение. */
    uint32_t spread_ticks;     /* Разброс принятых отсчётов. */
    int32_t slope_milli_ticks_per_sample; /* Наклон регрессии, 0,001 тика/отсчёт. */
    uint8_t input_samples;     /* Количество входных отсчётов. */
    uint8_t used_samples;      /* Количество отсчётов после MAD-фильтра. */
    uint8_t regression_used;   /* 1, если в итог добавлена трендовая составляющая. */
    bool recent_window_used;   /* Подтверждённое движение или новый стабильный уровень. */
} MeasurementStatisticsResult_t;

/**
 * @brief Формирует робастную оценку текущего ToF по окну из 20 отсчётов.
 *
 * @param samples                         Отсчёты t1 в хронологическом порядке.
 * @param sample_count                    Должен быть равен 20.
 * @param maximum_accepted_spread_ticks   Грубый предел разброса после MAD.
 * @param minimum_outlier_gate_ticks      Минимальный порог отклонения от медианы.
 * @param slope_low_milli_ticks_per_sample Ниже этого наклона используется среднее.
 * @param slope_high_milli_ticks_per_sample Выше этого наклона используется конец регрессии.
 * @param result                          Результат и диагностические параметры.
 *
 * @return true  — оценка сформирована и может публиковаться;
 * @return false — валидных отсчётов недостаточно либо окно нестабильно.
 */
bool MeasurementStatistics_Calculate(
    const uint32_t *samples,
    uint8_t sample_count,
    uint32_t maximum_accepted_spread_ticks,
    uint32_t minimum_outlier_gate_ticks,
    uint32_t slope_low_milli_ticks_per_sample,
    uint32_t slope_high_milli_ticks_per_sample,
    MeasurementStatisticsResult_t *result);

bool MeasurementStatistics_Track(
    const uint32_t *samples,
    uint8_t sample_count,
    uint32_t maximum_accepted_spread_ticks,
    uint32_t minimum_outlier_gate_ticks,
    uint32_t slope_low_milli_ticks_per_sample,
    uint32_t slope_high_milli_ticks_per_sample,
    MeasurementStatisticsResult_t *result);

bool MeasurementStatistics_CanHold(bool pulse_captured, bool level_available,
                                   uint32_t age_ms, uint32_t hold_ms);

float MeasurementStatistics_SmoothEstimate(float previous_ticks,
                                           const MeasurementStatisticsResult_t *result,
                                           uint32_t period_ms);

/* Returns a signed estimate, or NAN outside the working range and its
 * tolerance. Clamp negative results to zero after filtering. */
float MeasurementStatistics_FilterRangeEdges(
    float previous_mm, float candidate_mm, float range_mm,
    float upper_overrun_mm, uint32_t period_ms);

float MeasurementStatistics_LevelPercent(float level_mm, float nominal_upper_mm);
bool MeasurementStatistics_OverflowActive(bool previous, float level_mm,
                                          float nominal_upper_mm, float alarm_margin_mm);

#ifdef __cplusplus
}
#endif

#endif /* MEASUREMENT_STATISTICS_H */
