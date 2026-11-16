#include "liquid_properties.h"
#include "measurement_snapshot.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static float DecodeFloat(const uint16_t *words)
{
    uint32_t raw = (uint32_t)words[0] | ((uint32_t)words[1] << 16);
    float result;
    memcpy(&result, &raw, sizeof(result));
    return result;
}

static void TestSecondHistory(void)
{
    const uint32_t start = UINT32_MAX - 500U;
    const uint16_t valid = MEASUREMENT_STATUS_VALID;
    MeasurementSnapshot_Init();
    MeasurementSnapshot_StartHistory(start);

    MeasurementSnapshot_Publish(100.0f, NAN, 0.0f, 0.0f, 0.0f, NAN, valid, start + 700U);
    assert(MeasurementSnapshot_HoldLevel(start + 900U));
    assert(MeasurementSnapshot_GetTimestamp() == start + 700U);
    assert(MeasurementSnapshot_GetStatus() == (valid | MEASUREMENT_STATUS_HELD));
    MeasurementSnapshot_RecordDue(start + 999U);
    assert(MeasurementSnapshot_ReadHistoryWord(2U) == 0U);
    MeasurementSnapshot_RecordDue(start + 1000U);
    assert(MeasurementSnapshot_ReadHistoryWord(0U) == 1U);
    assert(MeasurementSnapshot_ReadHistoryWord(2U) == 1U);
    assert(MeasurementSnapshot_ReadHistoryWord(5U) == (valid | MEASUREMENT_STATUS_HELD));
    uint16_t held[] = {MeasurementSnapshot_ReadHistoryWord(3U),
                       MeasurementSnapshot_ReadHistoryWord(4U)};
    assert(DecodeFloat(held) == 100.0f);

    assert(MeasurementSnapshot_HoldLevel(start + 1700U));
    assert(!MeasurementSnapshot_HoldLevel(start + 1701U));
    MeasurementSnapshot_RecordDue(start + 2010U);
    assert(MeasurementSnapshot_ReadHistoryWord(0U) == 2U);
    assert(MeasurementSnapshot_ReadHistoryWord(3U) == UINT16_MAX);
    assert(MeasurementSnapshot_ReadHistoryWord(4U) == UINT16_MAX);
    assert(MeasurementSnapshot_ReadHistoryWord(5U) == 0U);

    MeasurementSnapshot_Publish(11.0f, NAN, 0.0f, 0.0f, 0.0f, NAN, valid, start + 2900U);
    MeasurementSnapshot_Publish(12.0f, NAN, 0.0f, 0.0f, 0.0f, NAN, valid, start + 3010U);
    uint16_t taken[] = {MeasurementSnapshot_ReadHistoryWord(3U),
                        MeasurementSnapshot_ReadHistoryWord(4U)};
    assert(DecodeFloat(taken) == 11.0f);
    assert(MeasurementSnapshot_ReadHistoryWord(5U) == valid);
    MeasurementSnapshot_RecordDue(start + 40037U);
    assert(MeasurementSnapshot_ReadHistoryWord(0U) == 40U);
    assert(MeasurementSnapshot_ReadHistoryWord(2U) == MEASUREMENT_HISTORY_CAPACITY);
    for (unsigned offset = 3U; offset < MEASUREMENT_HISTORY_WORDS; offset += 3U) {
        assert(MeasurementSnapshot_ReadHistoryWord(offset) == UINT16_MAX);
        assert(MeasurementSnapshot_ReadHistoryWord(offset + 2U) == 0U);
    }
    MeasurementSnapshot_RecordDue(start + 40999U);
    assert(MeasurementSnapshot_ReadHistoryWord(0U) == 40U);
    MeasurementSnapshot_RecordDue(start + 41000U);
    assert(MeasurementSnapshot_ReadHistoryWord(0U) == 41U);
    puts("PASS: 1 Hz history, bounded hold, missed service recovery and tick wrap");
}

int main(void)
{
    float density = Liquid_CalculateDensity(NAN, 0.0f, 0.0f, 0.0f);
    assert(fabsf(density - 998.20426f) < 0.001f);
    assert(fabsf(Liquid_CalculateDensity(25.0f, 0.0f, 0.0f, 0.0f) -
                 997.04506f) < 0.001f);
    assert(isfinite(Liquid_CalculateDensity(0.0f, 0.0f, 0.0f, 0.0f)));
    assert(isfinite(Liquid_CalculateDensity(40.0f, 0.0f, 0.0f, 0.0f)));
    assert(isnan(Liquid_CalculateDensity(41.0f, 0.0f, 0.0f, 0.0f)));
    assert(Liquid_CalculateDensity(NAN, 850.0f, 20.0f, 1.0f) == 850.0f);
    assert(fabsf(Liquid_CalculateDensity(30.0f, 850.0f, 20.0f, 1.0f) -
                 850.0f / 1.01f) < 0.001f);
    assert(isnan(Liquid_CalculateDensity(0.0f, 850.0f, 20.0f, 50.0f)));

    const float volume = 0.1798788f;
    float mass = Liquid_CalculateMass(volume, density);
    assert(fabsf(mass - 179.5558f) < 0.001f);
    assert(Liquid_CalculateMass(0.0f, density) == 0.0f);
    assert(isnan(Liquid_CalculateMass(NAN, density)));
    assert(isnan(Liquid_CalculateMass(-0.001f, density)));
    assert(isnan(Liquid_CalculateMass(volume, NAN)));
    float standard_density = Liquid_CalculateDensity(15.0f, 0.0f, 0.0f, 0.0f);
    assert(fabsf(standard_density - 999.09955f) < 0.001f);
    assert(fabsf(Liquid_CalculateVolume(mass, standard_density) - 0.1797176f) < 0.000001f);
    assert(fabsf(Liquid_CalculateVolume(mass, density) - volume) < 0.0000001f);
    assert(Liquid_CalculateVolume(0.0f, density) == 0.0f);
    assert(isnan(Liquid_CalculateVolume(mass, 0.0f)));
    assert(isnan(Liquid_CalculateVolume(NAN, density)));

    uint16_t words[MEASUREMENT_SNAPSHOT_WORD_COUNT];
    MeasurementSnapshot_Init();
    assert(!MeasurementSnapshot_ReadRange(1000U, MEASUREMENT_SNAPSHOT_WORD_COUNT, words));
    const float percent = 100.0f;
    MeasurementSnapshot_Publish(700.0f, NAN, percent, volume, mass, density, 1U, 100U);
    assert(MeasurementSnapshot_ReadRange(1000U, MEASUREMENT_SNAPSHOT_WORD_COUNT, words));
    assert(DecodeFloat(&words[0]) == 700.0f);
    assert(words[2] == UINT16_MAX && words[3] == UINT16_MAX);
    assert(DecodeFloat(&words[4]) == percent);
    assert(DecodeFloat(&words[6]) == volume);
    assert(DecodeFloat(&words[8]) == mass);
    assert(DecodeFloat(&words[10]) == density);
    assert(DecodeFloat(&words[12]) == volume);
    MeasurementSnapshot_Publish(NAN, NAN, NAN, NAN, NAN, NAN, 0U, 200U);
    assert(MeasurementSnapshot_ReadRange(1008U, 6U, words));
    for (unsigned i = 0U; i < 6U; ++i) {
        assert(words[i] == UINT16_MAX);
    }
    TestSecondHistory();
    puts("PASS: water reference, temperature correction, volume and mass, live Modbus words");
}
