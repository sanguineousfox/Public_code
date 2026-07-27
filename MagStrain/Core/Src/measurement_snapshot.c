/** @file measurement_snapshot.c @brief Быстрый снимок измерений в RAM. */
#include "measurement_snapshot.h"

#include <string.h>

typedef struct {
    volatile uint32_t sequence;
    volatile uint16_t words[MEASUREMENT_SNAPSHOT_WORD_COUNT];
    volatile uint32_t timestamp_ms;
    volatile uint16_t status;
    volatile uint8_t valid;
} MeasurementSnapshotState_t;

static MeasurementSnapshotState_t snapshot;

static void FloatToWords(float value, uint16_t *high_word, uint16_t *low_word)
{
    uint32_t raw;

    memcpy(&raw, &value, sizeof(raw));
    *high_word = (uint16_t)(raw >> 16);
    *low_word = (uint16_t)raw;
}

void MeasurementSnapshot_Init(void)
{
    memset((void *)&snapshot, 0, sizeof(snapshot));
}

void MeasurementSnapshot_Publish(float level_mm,
                                 float temperature_c,
                                 float percent,
                                 float volume_m3,
                                 uint16_t status,
                                 uint32_t timestamp_ms)
{
    uint16_t prepared[MEASUREMENT_SNAPSHOT_WORD_COUNT];
    uint8_t i;

    FloatToWords(level_mm,      &prepared[0], &prepared[1]);
    FloatToWords(temperature_c, &prepared[2], &prepared[3]);
    FloatToWords(percent,       &prepared[4], &prepared[5]);
    FloatToWords(volume_m3,     &prepared[6], &prepared[7]);

    /* Нечётное sequence означает обновление. Чётное — готовый снимок. */
    snapshot.sequence++;

    for (i = 0U; i < MEASUREMENT_SNAPSHOT_WORD_COUNT; ++i) {
        snapshot.words[i] = prepared[i];
    }

    snapshot.status = status;
    snapshot.timestamp_ms = timestamp_ms;
    snapshot.valid = 1U;

    snapshot.sequence++;
}

bool MeasurementSnapshot_ReadRange(uint16_t start_address,
                                   uint16_t register_count,
                                   uint16_t *destination)
{
    uint16_t offset;
    uint32_t sequence_before;
    uint32_t sequence_after;
    uint8_t attempt;
    uint16_t i;

    if (destination == NULL || register_count == 0U ||
        start_address < MEASUREMENT_SNAPSHOT_FIRST_ADDRESS ||
        snapshot.valid == 0U) {
        return false;
    }

    offset = (uint16_t)(start_address - MEASUREMENT_SNAPSHOT_FIRST_ADDRESS);
    if ((uint32_t)offset + register_count > MEASUREMENT_SNAPSHOT_WORD_COUNT) {
        return false;
    }

    for (attempt = 0U; attempt < 4U; ++attempt) {
        sequence_before = snapshot.sequence;
        if ((sequence_before & 1U) != 0U) {
            continue;
        }

        for (i = 0U; i < register_count; ++i) {
            destination[i] = snapshot.words[offset + i];
        }

        sequence_after = snapshot.sequence;
        if (sequence_before == sequence_after &&
            (sequence_after & 1U) == 0U) {
            return true;
        }
    }

    return false;
}

bool MeasurementSnapshot_ReadWord(uint16_t address, uint16_t *value)
{
    return MeasurementSnapshot_ReadRange(address, 1U, value);
}

bool MeasurementSnapshot_IsValid(void)
{
    return snapshot.valid != 0U;
}

uint32_t MeasurementSnapshot_GetTimestamp(void)
{
    return snapshot.timestamp_ms;
}

uint16_t MeasurementSnapshot_GetStatus(void)
{
    return snapshot.status;
}
