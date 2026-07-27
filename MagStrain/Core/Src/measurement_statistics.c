/**
 * @file    measurement_statistics.c
 * @brief   Простая и проверяемая статистика 11 измерений ToF.
 */
#include "measurement_statistics.h"

#include <stddef.h>
#include <string.h>

/**
 * @brief Сортирует небольшой массив по возрастанию методом вставок.
 *
 * В серии всего 11 элементов. Для такого количества insertion sort имеет
 * маленький код, не использует динамическую память и работает предсказуемо.
 */
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

/**
 * @brief Вычисляет среднее арифметическое с округлением к ближайшему тику.
 *
 * Сумма хранится в uint64_t, поэтому переполнение невозможно даже при
 * максимальных 16-битных значениях TIM3.
 */
static uint32_t CalculateRoundedMean(const uint32_t *values,
                                     uint8_t first,
                                     uint8_t count)
{
    uint64_t sum = 0U;
    uint8_t i;

    for (i = 0U; i < count; ++i) {
        sum += values[first + i];
    }

    return (uint32_t)((sum + (uint64_t)(count / 2U)) /
                      (uint64_t)count);
}

bool MeasurementStatistics_Calculate(
    const uint32_t *samples,
    uint8_t sample_count,
    uint32_t maximum_trimmed_spread_ticks,
    MeasurementStatisticsResult_t *result)
{
    uint32_t sorted[MEASUREMENT_STATISTICS_SAMPLE_COUNT];
    uint32_t trimmed_spread;

    if (samples == NULL || result == NULL) {
        return false;
    }

    memset(result, 0, sizeof(*result));
    result->input_samples = sample_count;

    /* По требованиям алгоритма результат строится только по полным 11
     * валидным измерениям. Неполная серия не должна менять уровень. */
    if (sample_count != MEASUREMENT_STATISTICS_SAMPLE_COUNT) {
        return false;
    }

    memcpy(sorted, samples, sizeof(sorted));
    SortAscending(sorted, MEASUREMENT_STATISTICS_SAMPLE_COUNT);

    /* sorted[0] и sorted[10] — удаляемые минимум и максимум.
     * В итог входят элементы sorted[1] ... sorted[9], всего девять. */
    trimmed_spread = sorted[9U] - sorted[1U];

    result->spread_ticks = trimmed_spread;
    result->used_samples = 9U;

    /* Критическое отличие от v8.1:
     * если после удаления крайних значений серия все равно нестабильна,
     * мы НЕ ищем произвольную группу из семи и НЕ публикуем среднее.
     * Старое корректное значение остается в RAM/Modbus до следующей серии. */
    if (trimmed_spread > maximum_trimmed_spread_ticks) {
        return false;
    }

    result->mean_ticks = CalculateRoundedMean(sorted, 1U, 9U);
    return true;
}
