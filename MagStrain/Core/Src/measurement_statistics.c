/**
 * @file    measurement_statistics.c
 * @brief   Робастная обработка 20 измерений ToF и линейная аппроксимация.
 */
#include "measurement_statistics.h"

#include <stddef.h>
#include <string.h>

/** @brief Сортировка небольшого массива по возрастанию методом вставок. */
static void SortAscending(uint32_t *values, uint8_t count)
{
    uint8_t i;

    for (i = 1U; i < count; ++i) {
        uint32_t key = values[i];
        uint8_t position = i;

        while (position > 0U && values[position - 1U] > key) {
            values[position] = values[position - 1U];
            --position;
        }
        values[position] = key;
    }
}

/** @brief Медиана чётного массива из 20 уже отсортированных значений. */
static uint32_t MedianOfTwenty(const uint32_t *sorted)
{
    uint64_t middle_sum =
        (uint64_t)sorted[MEASUREMENT_STATISTICS_SAMPLE_COUNT / 2U - 1U] +
        (uint64_t)sorted[MEASUREMENT_STATISTICS_SAMPLE_COUNT / 2U];

    return (uint32_t)((middle_sum + 1U) / 2U);
}

/** @brief Деление signed int64 с округлением к ближайшему целому. */
static int64_t DivideRoundedSigned(int64_t numerator, int64_t denominator)
{
    if (denominator <= 0) {
        return 0;
    }

    if (numerator >= 0) {
        return (numerator + denominator / 2) / denominator;
    }

    return -((-numerator + denominator / 2) / denominator);
}

bool MeasurementStatistics_Calculate(
    const uint32_t *samples,
    uint8_t sample_count,
    uint32_t maximum_accepted_spread_ticks,
    uint32_t minimum_outlier_gate_ticks,
    uint32_t slope_low_milli_ticks_per_sample,
    uint32_t slope_high_milli_ticks_per_sample,
    MeasurementStatisticsResult_t *result)
{
    uint32_t sorted[MEASUREMENT_STATISTICS_SAMPLE_COUNT];
    uint32_t deviations[MEASUREMENT_STATISTICS_SAMPLE_COUNT];
    uint32_t median_ticks;
    uint32_t mad_ticks;
    uint32_t outlier_gate_ticks;
    uint32_t accepted_min = UINT32_MAX;
    uint32_t accepted_max = 0U;
    uint64_t sum_x = 0U;
    uint64_t sum_y = 0U;
    uint64_t sum_xx = 0U;
    uint64_t sum_xy = 0U;
    uint8_t accepted_count = 0U;
    uint8_t i;
    int64_t denominator;
    int64_t slope_numerator;
    int64_t intercept_numerator;
    int64_t endpoint_numerator;
    int64_t endpoint_ticks;
    int64_t robust_mean_ticks;
    int64_t final_ticks;
    int64_t endpoint_delta;
    uint64_t slope_abs_milli;
    uint32_t blend = 0U;

    if (samples == NULL || result == NULL) {
        return false;
    }

    memset(result, 0, sizeof(*result));
    result->input_samples = sample_count;

    if (sample_count != MEASUREMENT_STATISTICS_SAMPLE_COUNT) {
        return false;
    }

    memcpy(sorted, samples, sizeof(sorted));
    SortAscending(sorted, MEASUREMENT_STATISTICS_SAMPLE_COUNT);
    median_ticks = MedianOfTwenty(sorted);

    for (i = 0U; i < MEASUREMENT_STATISTICS_SAMPLE_COUNT; ++i) {
        deviations[i] = (samples[i] >= median_ticks) ?
            (samples[i] - median_ticks) :
            (median_ticks - samples[i]);
    }

    SortAscending(deviations, MEASUREMENT_STATISTICS_SAMPLE_COUNT);
    mad_ticks = MedianOfTwenty(deviations);

    outlier_gate_ticks = mad_ticks * MEASUREMENT_STATISTICS_MAD_FACTOR;
    if (outlier_gate_ticks < minimum_outlier_gate_ticks) {
        outlier_gate_ticks = minimum_outlier_gate_ticks;
    }

    /*
     * Отсчёты обрабатываются в исходном хронологическом порядке.
     * Индекс i используется как время для линейной регрессии.
     */
    for (i = 0U; i < MEASUREMENT_STATISTICS_SAMPLE_COUNT; ++i) {
        uint32_t deviation = (samples[i] >= median_ticks) ?
            (samples[i] - median_ticks) :
            (median_ticks - samples[i]);

        if (deviation > outlier_gate_ticks) {
            continue;
        }

        if (samples[i] < accepted_min) {
            accepted_min = samples[i];
        }
        if (samples[i] > accepted_max) {
            accepted_max = samples[i];
        }

        sum_x += i;
        sum_y += samples[i];
        sum_xx += (uint32_t)i * (uint32_t)i;
        sum_xy += (uint64_t)i * (uint64_t)samples[i];
        ++accepted_count;
    }

    result->median_ticks = median_ticks;
    result->mad_ticks = mad_ticks;
    result->used_samples = accepted_count;

    if (accepted_count < MEASUREMENT_STATISTICS_MIN_ACCEPTED) {
        return false;
    }

    result->spread_ticks = accepted_max - accepted_min;
    if (result->spread_ticks > maximum_accepted_spread_ticks) {
        return false;
    }

    robust_mean_ticks = DivideRoundedSigned(
        (int64_t)sum_y,
        (int64_t)accepted_count);
    result->robust_mean_ticks = (uint32_t)robust_mean_ticks;

    /*
     * Метод наименьших квадратов для y = a + b*x:
     *
     * b = (n*sum(x*y) - sum(x)*sum(y)) /
     *     (n*sum(x*x) - sum(x)^2)
     *
     * При отсутствии выраженного тренда итогом остаётся робастное среднее.
     * При движении поплавка добавляется прогноз к последнему индексу окна.
     */
    denominator =
        (int64_t)accepted_count * (int64_t)sum_xx -
        (int64_t)sum_x * (int64_t)sum_x;

    if (denominator <= 0) {
        result->mean_ticks = result->robust_mean_ticks;
        return true;
    }

    slope_numerator =
        (int64_t)accepted_count * (int64_t)sum_xy -
        (int64_t)sum_x * (int64_t)sum_y;

    intercept_numerator =
        (int64_t)sum_y * (int64_t)sum_xx -
        (int64_t)sum_x * (int64_t)sum_xy;

    endpoint_numerator = intercept_numerator +
        slope_numerator *
        (int64_t)(MEASUREMENT_STATISTICS_SAMPLE_COUNT - 1U);

    endpoint_ticks = DivideRoundedSigned(endpoint_numerator, denominator);

    if (endpoint_ticks < 0) {
        endpoint_ticks = 0;
    }
    if (endpoint_ticks > 0xFFFF) {
        endpoint_ticks = 0xFFFF;
    }

    slope_abs_milli =
        ((uint64_t)((slope_numerator < 0) ?
                    -slope_numerator : slope_numerator) *
         MEASUREMENT_STATISTICS_BLEND_SCALE) /
        (uint64_t)denominator;

    result->slope_milli_ticks_per_sample =
        (int32_t)DivideRoundedSigned(
            slope_numerator *
                (int64_t)MEASUREMENT_STATISTICS_BLEND_SCALE,
            denominator);

    if (slope_high_milli_ticks_per_sample <=
        slope_low_milli_ticks_per_sample) {
        slope_high_milli_ticks_per_sample =
            slope_low_milli_ticks_per_sample + 1U;
    }

    if (slope_abs_milli <= slope_low_milli_ticks_per_sample) {
        blend = 0U;
    } else if (slope_abs_milli >= slope_high_milli_ticks_per_sample) {
        blend = MEASUREMENT_STATISTICS_BLEND_SCALE;
    } else {
        blend = (uint32_t)(
            (slope_abs_milli - slope_low_milli_ticks_per_sample) *
            MEASUREMENT_STATISTICS_BLEND_SCALE /
            (slope_high_milli_ticks_per_sample -
             slope_low_milli_ticks_per_sample));
    }

    endpoint_delta = endpoint_ticks - robust_mean_ticks;
    final_ticks = robust_mean_ticks + DivideRoundedSigned(
        endpoint_delta * (int64_t)blend,
        MEASUREMENT_STATISTICS_BLEND_SCALE);

    if (final_ticks < 0) {
        final_ticks = 0;
    }
    if (final_ticks > 0xFFFF) {
        final_ticks = 0xFFFF;
    }

    result->regression_used = (blend != 0U) ? 1U : 0U;
    result->mean_ticks = (uint32_t)final_ticks;
    return true;
}
