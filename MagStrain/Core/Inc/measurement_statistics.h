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

typedef struct {
    uint32_t mean_ticks;       /* Итог: робастное среднее или адаптивная аппроксимация. */
    uint32_t robust_mean_ticks;/* Среднее всех принятых после MAD отсчётов. */
    uint32_t median_ticks;     /* Медиана полного окна из 20 значений. */
    uint32_t mad_ticks;        /* Медианное абсолютное отклонение. */
    uint32_t spread_ticks;     /* Разброс принятых отсчётов. */
    int32_t slope_milli_ticks_per_sample; /* Наклон регрессии, 0,001 тика/отсчёт. */
    uint8_t input_samples;     /* Количество входных отсчётов. */
    uint8_t used_samples;      /* Количество отсчётов после MAD-фильтра. */
    uint8_t regression_used;   /* 1, если в итог добавлена трендовая составляющая. */
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

#ifdef __cplusplus
}
#endif

#endif /* MEASUREMENT_STATISTICS_H */
