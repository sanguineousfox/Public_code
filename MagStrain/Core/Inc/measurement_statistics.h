/**
 * @file    measurement_statistics.h
 * @brief   Статистическая обработка серии измерений времени пролета.
 *
 * Алгоритм специально оставлен простым и предсказуемым:
 *  1. получаем ровно 11 валидных значений времени первого импульса t1;
 *  2. сортируем значения;
 *  3. удаляем одно минимальное и одно максимальное;
 *  4. вычисляем среднее арифметическое оставшихся девяти значений;
 *  5. проверяем разброс этих девяти значений.
 *
 * Если разброс слишком велик, результат НЕ публикуется. Это принципиально:
 * предыдущая версия выбирала «плотную группу» даже при огромном разбросе и
 * могла усреднить разные отражения волны, создавая скачки уровня.
 */
#ifndef MEASUREMENT_STATISTICS_H
#define MEASUREMENT_STATISTICS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define MEASUREMENT_STATISTICS_SAMPLE_COUNT  11U

typedef struct {
    uint32_t mean_ticks;      /* Среднее девяти значений после удаления min/max. */
    uint32_t spread_ticks;    /* Разность между максимумом и минимумом из этих девяти. */
    uint8_t  input_samples;   /* Количество значений, переданных в функцию. */
    uint8_t  used_samples;    /* Всегда 9 для принятой серии из 11 значений. */
} MeasurementStatisticsResult_t;

/**
 * @brief Рассчитывает итоговое значение по серии из 11 валидных измерений.
 *
 * @param samples                       Массив значений t1 в тиках TIM3.
 * @param sample_count                  Должен быть равен 11.
 * @param maximum_trimmed_spread_ticks Максимально допустимый разброс после
 *                                      удаления одного min и одного max.
 * @param result                        Структура результата и диагностики.
 *
 * @return true  — серия стабильна, mean_ticks можно публиковать;
 * @return false — серия неполная или нестабильная, старое значение уровня
 *                 должно остаться в Modbus без изменений.
 */
bool MeasurementStatistics_Calculate(
    const uint32_t *samples,
    uint8_t sample_count,
    uint32_t maximum_trimmed_spread_ticks,
    MeasurementStatisticsResult_t *result);

#ifdef __cplusplus
}
#endif

#endif /* MEASUREMENT_STATISTICS_H */
