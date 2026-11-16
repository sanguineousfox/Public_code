/**
 * @file    measurement_statistics.c
 * @brief   Робастная обработка 20 измерений ToF и линейная аппроксимация.
 */
#include "measurement_statistics.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static bool PulseWindowShowsMotion(const MeasurementPulseWindow_t *window, unsigned count)
{
    const MeasurementPulseSample_t *samples =
        window->samples + MEASUREMENT_PULSE_WINDOW_COUNT - count;
    const unsigned last = count - 1U;
    int32_t travel = (int32_t)samples[last].first_ticks - samples[0].first_ticks;
    int32_t direction = travel >= 0 ? 1 : -1;
    travel *= direction;
    /* 64 тика TIM3 — около 3 мм на текущем волноводе. */
    if (travel < 64 || samples[0].first_ticks == 0U) return false;

    unsigned forward = 0U;
    for (unsigned i = 1U; i <= last; ++i) {
        int32_t step = direction *
            ((int32_t)samples[i].first_ticks - samples[i - 1U].first_ticks);
        if (samples[i].first_ticks == 0U || step < -9 || step * 2 > travel) return false;
        if (step > 3) ++forward;
    }
    return forward > 3U * last / 4U;
}

uint16_t MeasurementStatistics_UpdatePulseWindow(MeasurementPulseWindow_t *window,
    MeasurementPulseSample_t sample, uint16_t mode, uint32_t now_ms)
{
    memmove(window->samples, window->samples + 1U,
            sizeof(window->samples) - sizeof(sample));
    window->samples[MEASUREMENT_PULSE_WINDOW_COUNT - 1U] = sample;
    ++window->sequence;

    if (mode != MEASUREMENT_PULSE_MODE_AUTO || sample.first_ticks == 0U) {
        window->moving = false;
    } else if (PulseWindowShowsMotion(window, MEASUREMENT_PULSE_WINDOW_COUNT)) {
        window->moving = true;
        window->last_motion_ms = now_ms;
    } else if (now_ms - window->last_motion_ms >= 1000U) {
        window->moving = false;
    }

    if (mode == MEASUREMENT_PULSE_MODE_SINGLE) return MEASUREMENT_PULSE_STATE_SINGLE;
    return window->moving ? MEASUREMENT_PULSE_STATE_MOVING : MEASUREMENT_PULSE_STATE_PAIR;
}

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

static uint32_t TickDistance(uint32_t t1, uint32_t t2)
{
    return t1 >= t2 ? t1 - t2 : t2 - t1;
}

static uint32_t MedianTicks(uint32_t *samples, uint8_t count)
{
    SortAscending(samples, count);
    return (samples[(count - 1U) / 2U] + samples[count / 2U] + 1U) / 2U;
}

static void RememberT1(MeasurementT1Filter_t *filter, uint32_t ticks)
{
    if (filter->accepted_count == MEASUREMENT_PULSE_WINDOW_COUNT) {
        memmove(filter->accepted, filter->accepted + 1U,
                sizeof(filter->accepted) - sizeof(ticks));
        --filter->accepted_count;
    }
    filter->accepted[filter->accepted_count++] = ticks;
}

MeasurementT1Decision_t MeasurementStatistics_FilterT1(MeasurementT1Filter_t *filter,
    const MeasurementPulseWindow_t *window, uint32_t *accepted_ticks)
{
    const unsigned last = MEASUREMENT_PULSE_WINDOW_COUNT - 1U;
    const uint16_t ticks = window->samples[last].first_ticks;
    if (ticks == 0U) {
        filter->candidate = 0U;
        filter->initial_count = 0U;
        return MEASUREMENT_T1_PENDING;
    }

    if (filter->accepted_count == 0U) {
        filter->initial[filter->initial_count++] = ticks;
        if (filter->initial_count < 5U) return MEASUREMENT_T1_PENDING;
        *accepted_ticks = MedianTicks(filter->initial, 5U);
        RememberT1(filter, *accepted_ticks);
        return MEASUREMENT_T1_ACCEPTED;
    }

    uint32_t values[MEASUREMENT_PULSE_WINDOW_COUNT];
    uint8_t count = filter->accepted_count;
    memcpy(values, filter->accepted, count * sizeof(values[0]));
    uint32_t baseline = MedianTicks(values, count);
    for (unsigned i = 0U; i < count; ++i) {
        values[i] = TickDistance(values[i], baseline);
    }
    uint32_t gate = MEASUREMENT_STATISTICS_MAD_FACTOR * MedianTicks(values, count);
    if (gate < 9U) gate = 9U;
    bool near_baseline = TickDistance(ticks, baseline) <= gate;

    if (!near_baseline && PulseWindowShowsMotion(window, 5U)) {
        filter->candidate = 0U;
        RememberT1(filter, ticks);
        *accepted_ticks = ticks;
        return MEASUREMENT_T1_ACCEPTED;
    }

    if (filter->candidate != 0U) {
        uint32_t distance = TickDistance(ticks, filter->candidate);
        if (distance <= gate && distance < TickDistance(ticks, baseline)) {
            filter->candidate_sum += ticks;
            if (++filter->matches == 4U) {
                *accepted_ticks = (filter->candidate_sum + 2U) / 5U;
                filter->candidate = 0U;
                filter->accepted_count = 0U;
                RememberT1(filter, *accepted_ticks);
                return MEASUREMENT_T1_STEP;
            }
        } else if (++filter->misses == 2U) {
            filter->candidate = 0U;
        }
        if (filter->candidate != 0U) return MEASUREMENT_T1_PENDING;
    }

    if (!near_baseline) {
        filter->candidate = ticks;
        filter->candidate_sum = ticks;
        filter->matches = 0U;
        filter->misses = 0U;
        return MEASUREMENT_T1_PENDING;
    }
    RememberT1(filter, ticks);
    *accepted_ticks = ticks;
    return MEASUREMENT_T1_ACCEPTED;
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

    result->robust_mean_ticks = (float)sum_y / (float)accepted_count;

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

    float endpoint_ticks = (float)endpoint_numerator / (float)denominator;

    if (endpoint_ticks < 0.0f) {
        endpoint_ticks = 0.0f;
    }
    if (endpoint_ticks > 65535.0f) {
        endpoint_ticks = 65535.0f;
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

    result->regression_used = (blend != 0U) ? 1U : 0U;
    result->mean_ticks = result->robust_mean_ticks +
        (endpoint_ticks - result->robust_mean_ticks) *
        ((float)blend / (float)MEASUREMENT_STATISTICS_BLEND_SCALE);
    return true;
}

bool MeasurementStatistics_Track(
    const uint32_t *samples,
    uint8_t sample_count,
    uint32_t maximum_accepted_spread_ticks,
    uint32_t minimum_outlier_gate_ticks,
    uint32_t slope_low_milli_ticks_per_sample,
    uint32_t slope_high_milli_ticks_per_sample,
    MeasurementStatisticsResult_t *result)
{
    bool full_window_valid = MeasurementStatistics_Calculate(
        samples, sample_count, maximum_accepted_spread_ticks,
        minimum_outlier_gate_ticks, slope_low_milli_ticks_per_sample,
        slope_high_milli_ticks_per_sample, result);
    if (samples == NULL || result == NULL ||
        sample_count != MEASUREMENT_STATISTICS_SAMPLE_COUNT) {
        return false;
    }

    /* Прогноз длинного окна усиливает дрейф у края окна. В покое
     * используем робастное среднее, а быстрый ход подтверждаем отдельно. */
    if (full_window_valid) {
        result->mean_ticks = result->robust_mean_ticks;
        result->regression_used = 0U;
    }

    enum { recent_count = 5 };
    const uint32_t *recent = &samples[sample_count - recent_count];
    float noise_gate = (float)minimum_outlier_gate_ticks;
    /* 64 тика при 72 МГц соответствуют примерно 3 мм на рабочем волноводе. */
    float motion_gate = fmaxf(64.0f, 4.0f * noise_gate);
    float direction = recent[recent_count - 1U] >= recent[0] ? 1.0f : -1.0f;
    float recent_min = (float)recent[0];
    float recent_max = (float)recent[0];
    float sum = (float)recent[0];
    float min_delta = INFINITY;
    float max_delta = -INFINITY;
    for (uint8_t i = 1U; i < recent_count; ++i) {
        float value = (float)recent[i];
        float delta = direction * (value - (float)recent[i - 1U]);
        recent_min = fminf(recent_min, value);
        recent_max = fmaxf(recent_max, value);
        min_delta = fminf(min_delta, delta);
        max_delta = fmaxf(max_delta, delta);
        sum += value;
    }
    bool settled = recent_min > 0.0f && recent_max - recent_min <= noise_gate;
    bool moving = recent_min > 0.0f && recent_max - recent_min >= motion_gate &&
        min_delta > noise_gate && max_delta <= 3.0f * min_delta + noise_gate;
    if (!moving && !settled) {
        return full_window_valid;
    }

    float recent_estimate = moving ? (float)recent[recent_count - 1U] : sum / (float)recent_count;
    if (!moving && full_window_valid &&
        fabsf(recent_estimate - result->robust_mean_ticks) < motion_gate) {
        return true;
    }
    result->mean_ticks = recent_estimate;
    result->recent_window_used = true;
    return true;
}

float MeasurementStatistics_SmoothEstimate(float previous_ticks,
                                           const MeasurementStatisticsResult_t *result,
                                           uint32_t period_ms)
{
    if (result->recent_window_used || !isfinite(previous_ticks)) {
        return result->mean_ticks;
    }
    float period = (float)period_ms;
    return previous_ticks + (result->mean_ticks - previous_ticks) *
        (period / (period + 1500.0f));
}

bool MeasurementStatistics_CanHold(bool pulse_captured, bool level_available,
                                   uint32_t age_ms, uint32_t hold_ms)
{
    return pulse_captured && level_available && age_ms < hold_ms;
}

float MeasurementStatistics_FilterRangeEdges(
    float previous_mm, float candidate_mm, float range_mm,
    float upper_overrun_mm, uint32_t period_ms)
{
    if (!isfinite(range_mm) || range_mm <= 0.0f || !isfinite(candidate_mm)) {
        return NAN;
    }

    const float edge_band_mm = fminf(3.0f, range_mm * 0.25f);
    const float upper_tolerance_mm = fminf(1.0f, edge_band_mm);
    float upper_limit_mm = range_mm + fmaxf(upper_tolerance_mm, upper_overrun_mm);
    if (candidate_mm < -MEASUREMENT_LEVEL_BELOW_ZERO_TOLERANCE_MM ||
        candidate_mm > upper_limit_mm) {
        return NAN;
    }

    bool same_low_edge = candidate_mm <= edge_band_mm && previous_mm <= edge_band_mm;
    bool same_high_edge = candidate_mm >= range_mm - edge_band_mm &&
                          previous_mm >= range_mm - edge_band_mm &&
                          candidate_mm < range_mm + upper_tolerance_mm &&
                          previous_mm < range_mm + upper_tolerance_mm;
    if (same_low_edge || same_high_edge) {
        float delta_mm = candidate_mm - previous_mm;
        /* Выброс за край учитывается в среднем, но не ускоряет фильтр. */
        float previous_in_range = fminf(range_mm, fmaxf(0.0f, previous_mm));
        float candidate_in_range = fminf(range_mm, fmaxf(0.0f, candidate_mm));
        float motion = fminf(1.0f, fmaxf(0.0f,
            (fabsf(candidate_in_range - previous_in_range) - 0.8f) / 0.5f));
        float time_constant_ms = 6000.0f * (1.0f - motion);
        float period = fmaxf(1.0f, (float)period_ms);
        return previous_mm + delta_mm * (period / (period + time_constant_ms));
    }
    return candidate_mm;
}

float MeasurementStatistics_LevelPercent(float level_mm, float nominal_upper_mm)
{
    if (!isfinite(level_mm) || level_mm < 0.0f ||
        !isfinite(nominal_upper_mm) || nominal_upper_mm <= 0.0f) {
        return NAN;
    }
    return (level_mm / nominal_upper_mm) * 100.0f;
}

bool MeasurementStatistics_OverflowActive(bool previous, float level_mm,
                                          float nominal_upper_mm, float alarm_margin_mm)
{
    if (!isfinite(level_mm)) return previous;
    return previous ? level_mm > nominal_upper_mm :
        level_mm >= nominal_upper_mm + alarm_margin_mm;
}
