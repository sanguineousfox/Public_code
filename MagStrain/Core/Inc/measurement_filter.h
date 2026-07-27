/**
 * @file    measurement_filter.h
 * @brief   Быстрый адаптивный фильтр уровня.
 *
 * Малый шум сглаживается медианой и EMA. Большое реальное перемещение
 * определяется как ступень и публикуется сразу, без ожидания заполнения
 * длинного статистического окна.
 */
#ifndef MEASUREMENT_FILTER_H
#define MEASUREMENT_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define MEASUREMENT_FILTER_WINDOW_SIZE  5U

typedef struct {
    float samples[MEASUREMENT_FILTER_WINDOW_SIZE];
    float filtered_value;
    uint8_t sample_count;
    uint8_t write_index;
    uint8_t initialized;
} MeasurementFilter_t;

void MeasurementFilter_Init(MeasurementFilter_t *filter);

/**
 * @param raw_value            Новое валидное измерение, мм.
 * @param step_threshold       Порог быстрого перемещения, мм.
 * @param smoothing_alpha      Коэффициент EMA для медленных изменений 0..1.
 * @return Быстрое отфильтрованное значение.
 */
float MeasurementFilter_Update(MeasurementFilter_t *filter,
                               float raw_value,
                               float step_threshold,
                               float smoothing_alpha);

#ifdef __cplusplus
}
#endif

#endif /* MEASUREMENT_FILTER_H */
