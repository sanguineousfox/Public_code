/** @file measurement_snapshot.c @brief Быстрый снимок измерений в RAM. */
#include "measurement_snapshot.h"

#include <math.h>
#include <string.h>

typedef struct {
    volatile uint32_t sequence;
    volatile uint16_t words[MEASUREMENT_SNAPSHOT_WORD_COUNT];
    volatile uint32_t timestamp_ms;
    volatile uint16_t status;
    volatile uint8_t valid;
} MeasurementSnapshotState_t;

static MeasurementSnapshotState_t snapshot;

static struct {
    uint32_t boundary_ms;
    uint32_t sequence;
    uint16_t points[MEASUREMENT_HISTORY_CAPACITY][MEASUREMENT_HISTORY_POINT_WORDS];
    uint16_t count;
    uint16_t next;
    bool started;
} history;

static bool RecentLevelAt(uint32_t time_ms)
{
    return (snapshot.status & MEASUREMENT_STATUS_VALID) != 0U &&
           (uint32_t)(time_ms - snapshot.timestamp_ms) <= MEASUREMENT_LEVEL_MAX_AGE_MS;
}

void MeasurementSnapshot_StartHistory(uint32_t now_ms)
{
    history.boundary_ms = now_ms;
    history.sequence = 0U;
    history.count = 0U;
    history.next = 0U;
    history.started = true;
}

void MeasurementSnapshot_RecordDue(uint32_t now_ms)
{
    if (!history.started) {
        return;
    }

    uint32_t due = (now_ms - history.boundary_ms) / MEASUREMENT_HISTORY_PERIOD_MS;
    if (due > MEASUREMENT_HISTORY_CAPACITY) {
        /* Earlier points would be overwritten immediately. Keep their sequence numbers. */
        uint32_t skipped = due - MEASUREMENT_HISTORY_CAPACITY;
        history.boundary_ms += skipped * MEASUREMENT_HISTORY_PERIOD_MS;
        history.sequence += skipped;
        due = MEASUREMENT_HISTORY_CAPACITY;
    }
    while (due > 0U) {
        history.boundary_ms += MEASUREMENT_HISTORY_PERIOD_MS;
        uint16_t *point = history.points[history.next];
        bool recent = RecentLevelAt(history.boundary_ms);
        point[0] = recent ? snapshot.words[0] : UINT16_MAX;
        point[1] = recent ? snapshot.words[1] : UINT16_MAX;
        point[2] = recent ? snapshot.status :
            snapshot.status & (uint16_t)~(MEASUREMENT_STATUS_VALID | MEASUREMENT_STATUS_HELD);
        history.next = (history.next + 1U) % MEASUREMENT_HISTORY_CAPACITY;
        if (history.count < MEASUREMENT_HISTORY_CAPACITY) {
            ++history.count;
        }
        ++history.sequence;
        --due;
    }
}

uint16_t MeasurementSnapshot_ReadHistoryWord(uint16_t offset)
{
    if (offset == 0U) return (uint16_t)history.sequence;
    if (offset == 1U) return (uint16_t)(history.sequence >> 16);
    if (offset == 2U) return history.count;

    offset -= MEASUREMENT_HISTORY_HEADER_WORDS;
    uint16_t age = offset / MEASUREMENT_HISTORY_POINT_WORDS;
    uint16_t word = offset % MEASUREMENT_HISTORY_POINT_WORDS;
    if (age >= history.count) {
        return word < 2U ? UINT16_MAX : 0U;
    }
    uint16_t index = (history.next + MEASUREMENT_HISTORY_CAPACITY - 1U - age) %
                     MEASUREMENT_HISTORY_CAPACITY;
    return history.points[index][word];
}

bool MeasurementSnapshot_HoldLevel(uint32_t now_ms)
{
    MeasurementSnapshot_RecordDue(now_ms);
    if (!RecentLevelAt(now_ms)) {
        return false;
    }
    snapshot.status |= MEASUREMENT_STATUS_HELD;
    return true;
}

static void FloatToDocumentWords(float value,
                                 uint16_t *base_low_word,
                                 uint16_t *next_high_word)
{
    uint32_t raw = UINT32_MAX;

    if (isfinite(value)) {
        memcpy(&raw, &value, sizeof(raw));
    }

    /* Таблица Е.3: младшее 16-битное слово лежит по базовому адресу,
     * старшее слово - по следующему адресу. */
    *base_low_word = (uint16_t)raw;
    *next_high_word = (uint16_t)(raw >> 16);
}

void MeasurementSnapshot_Init(void)
{
    memset((void *)&snapshot, 0, sizeof(snapshot));
    history.started = false;
    history.count = 0U;
    history.sequence = 0U;
}

void MeasurementSnapshot_Publish(float level_mm,
                                 float temperature_c,
                                 float percent,
                                 float volume_m3,
                                 float mass_kg,
                                 float density_kg_m3,
                                 uint16_t status,
                                 uint32_t timestamp_ms)
{
    uint16_t prepared[MEASUREMENT_SNAPSHOT_WORD_COUNT];
    uint8_t i;

    MeasurementSnapshot_RecordDue(timestamp_ms);

    FloatToDocumentWords(level_mm,      &prepared[0], &prepared[1]);
    FloatToDocumentWords(temperature_c, &prepared[2], &prepared[3]);
    FloatToDocumentWords(percent,       &prepared[4], &prepared[5]);
    FloatToDocumentWords(volume_m3,     &prepared[6], &prepared[7]);
    FloatToDocumentWords(mass_kg,       &prepared[8], &prepared[9]);
    FloatToDocumentWords(density_kg_m3, &prepared[10], &prepared[11]);
    /* Один поплавок: весь измеренный объём относится к основному продукту. */
    prepared[12] = prepared[6];
    prepared[13] = prepared[7];

    /* Нечётное sequence означает обновление. Чётное — готовый снимок. */
    snapshot.sequence++;

    for (i = 0U; i < MEASUREMENT_SNAPSHOT_WORD_COUNT; ++i) {
        snapshot.words[i] = prepared[i];
    }

    snapshot.status = isfinite(level_mm) ? status :
        status & (uint16_t)~(MEASUREMENT_STATUS_VALID | MEASUREMENT_STATUS_HELD);
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
