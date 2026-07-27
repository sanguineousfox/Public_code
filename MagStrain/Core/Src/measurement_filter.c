/** @file measurement_filter.c @brief Быстрый адаптивный фильтр уровня. */
#include "measurement_filter.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static float AbsoluteFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static void ResetToValue(MeasurementFilter_t *filter, float value)
{
    uint8_t i;

    for (i = 0U; i < MEASUREMENT_FILTER_WINDOW_SIZE; ++i) {
        filter->samples[i] = value;
    }

    filter->filtered_value = value;
    filter->sample_count = 1U;
    filter->write_index = 1U;
    filter->initialized = 1U;
}

static float MedianOfSamples(const MeasurementFilter_t *filter)
{
    float sorted[MEASUREMENT_FILTER_WINDOW_SIZE];
    uint8_t count = filter->sample_count;
    uint8_t i;
    uint8_t j;

    if (count == 0U) {
        return filter->filtered_value;
    }

    if (count > MEASUREMENT_FILTER_WINDOW_SIZE) {
        count = MEASUREMENT_FILTER_WINDOW_SIZE;
    }

    for (i = 0U; i < count; ++i) {
        sorted[i] = filter->samples[i];
    }

    for (i = 1U; i < count; ++i) {
        float key = sorted[i];
        j = i;
        while (j > 0U && sorted[j - 1U] > key) {
            sorted[j] = sorted[j - 1U];
            --j;
        }
        sorted[j] = key;
    }

    if ((count & 1U) != 0U) {
        return sorted[count / 2U];
    }

    return (sorted[(count / 2U) - 1U] + sorted[count / 2U]) * 0.5f;
}

void MeasurementFilter_Init(MeasurementFilter_t *filter)
{
    if (filter == NULL) {
        return;
    }

    memset(filter, 0, sizeof(*filter));
}

float MeasurementFilter_Update(MeasurementFilter_t *filter,
                               float raw_value,
                               float step_threshold,
                               float smoothing_alpha)
{
    float median;
    float deviation;

    if (filter == NULL || !isfinite(raw_value)) {
        return 0.0f;
    }

    if (!isfinite(step_threshold) || step_threshold < 0.0f) {
        step_threshold = 0.0f;
    }

    if (!isfinite(smoothing_alpha)) {
        smoothing_alpha = 1.0f;
    }
    if (smoothing_alpha < 0.0f) smoothing_alpha = 0.0f;
    if (smoothing_alpha > 1.0f) smoothing_alpha = 1.0f;

    if (filter->initialized == 0U) {
        ResetToValue(filter, raw_value);
        return raw_value;
    }

    deviation = AbsoluteFloat(raw_value - filter->filtered_value);

    /* Реальное заметное перемещение: сбрасываем старую историю и сразу
     * публикуем новое положение. Именно это убирает секундный "хвост". */
    if (deviation >= step_threshold) {
        ResetToValue(filter, raw_value);
        return raw_value;
    }

    filter->samples[filter->write_index] = raw_value;
    filter->write_index++;
    if (filter->write_index >= MEASUREMENT_FILTER_WINDOW_SIZE) {
        filter->write_index = 0U;
    }
    if (filter->sample_count < MEASUREMENT_FILTER_WINDOW_SIZE) {
        filter->sample_count++;
    }

    median = MedianOfSamples(filter);
    filter->filtered_value +=
        smoothing_alpha * (median - filter->filtered_value);

    return filter->filtered_value;
}
