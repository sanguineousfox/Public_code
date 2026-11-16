#include "modbus.h"
#include "measurement_statistics.h"
#include "sensor_calibration_grid.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static float float_registers[6044U - 6000U];
static uint16_t int_registers[2];

float ModBus_GetParameter_Float(uint16_t address)
{
    return float_registers[address - 6000U];
}

void ModBus_SetParameter_Float(uint16_t address, float value)
{
    float_registers[address - 6000U] = value;
}

uint16_t ModBus_GetParameter_Int(uint16_t address)
{
    return int_registers[address - 6008U];
}

void ModBus_SetParameter_Int(uint16_t address, uint16_t value)
{
    int_registers[address - 6008U] = value;
}

static void CheckFloat(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.000001f);
}

static MeasurementT1Decision_t FeedT1(MeasurementT1Filter_t *filter,
    MeasurementPulseWindow_t *window, uint16_t ticks, uint32_t *accepted)
{
    MeasurementStatistics_UpdatePulseWindow(window, (MeasurementPulseSample_t){ticks, 0U, false},
        MEASUREMENT_PULSE_MODE_SINGLE, 0U);
    return MeasurementStatistics_FilterT1(filter, window, accepted);
}

static void CheckT1Spikes(void)
{
    MeasurementT1Filter_t filter = {0};
    MeasurementPulseWindow_t window = {0};
    uint32_t accepted = 0U;
    for (unsigned i = 0U; i < 15U; ++i) {
        uint16_t ticks = i == 2U ? 11950U : 12000U;
        MeasurementT1Decision_t decision = FeedT1(&filter, &window, ticks, &accepted);
        assert(decision == (i < 4U ? MEASUREMENT_T1_PENDING : MEASUREMENT_T1_ACCEPTED));
        if (i >= 4U) assert(accepted == 12000U);
    }

    /* A short burst remains in the raw trace and never enters the mean. */
    for (unsigned i = 0U; i < 3U; ++i) {
        assert(FeedT1(&filter, &window, 11954U, &accepted) == MEASUREMENT_T1_PENDING);
    }
    assert(window.samples[9].first_ticks == 11954U && accepted == 12000U);
    assert(FeedT1(&filter, &window, 12000U, &accepted) == MEASUREMENT_T1_PENDING);
    assert(FeedT1(&filter, &window, 12000U, &accepted) == MEASUREMENT_T1_ACCEPTED);
    assert(accepted == 12000U);

    /* A real step of roughly 1 mm passes with four of five confirmations. */
    const uint16_t step[] = {11980U, 11979U, 12000U, 11980U, 11981U, 11980U};
    for (unsigned i = 0U; i < 6U; ++i) {
        MeasurementT1Decision_t decision = FeedT1(&filter, &window, step[i], &accepted);
        assert(decision == (i == 5U ? MEASUREMENT_T1_STEP : MEASUREMENT_T1_PENDING));
    }
    assert(accepted == 11980U);
    assert(FeedT1(&filter, &window, 11977U, &accepted) == MEASUREMENT_T1_ACCEPTED);
    assert(accepted == 11977U);

    /* Missing t1 discards the candidate. */
    assert(FeedT1(&filter, &window, 11800U, &accepted) == MEASUREMENT_T1_PENDING);
    assert(FeedT1(&filter, &window, 0U, &accepted) == MEASUREMENT_T1_PENDING);
    assert(FeedT1(&filter, &window, 11980U, &accepted) == MEASUREMENT_T1_ACCEPTED);

    for (int direction = -1; direction <= 1; direction += 2) {
        filter = (MeasurementT1Filter_t){0};
        window = (MeasurementPulseWindow_t){0};
        for (unsigned i = 0U; i < 15U; ++i) {
            FeedT1(&filter, &window, 12000U, &accepted);
        }
        for (int i = 1; i <= 12; ++i) {
            uint16_t ticks = (uint16_t)(12000 + direction * 22 * i);
            MeasurementT1Decision_t decision = FeedT1(&filter, &window, ticks, &accepted);
            if (i >= 4) assert(decision == MEASUREMENT_T1_ACCEPTED && accepted == ticks);
        }
    }
}

static void CheckPulseModes(void)
{
    MeasurementPulseWindow_t window = {0};
    uint16_t mode = MEASUREMENT_PULSE_MODE_AUTO;
    MeasurementPulseSample_t sample = {12000U, 13100U, true};
    for (unsigned i = 0U; i < MEASUREMENT_PULSE_WINDOW_COUNT; ++i) {
        assert(MeasurementStatistics_UpdatePulseWindow(&window, sample, mode, i * 50U) ==
               MEASUREMENT_PULSE_STATE_PAIR);
    }
    assert(window.sequence == 10U);
    assert(window.samples[0].second_ticks == 13100U);

    /* A single early edge or a missing second edge cannot enable bypass. */
    assert(MeasurementStatistics_UpdatePulseWindow(&window,
           (MeasurementPulseSample_t){11945U, 0U, false}, mode, 500U) ==
           MEASUREMENT_PULSE_STATE_PAIR);
    assert(MeasurementStatistics_UpdatePulseWindow(&window,
           (MeasurementPulseSample_t){12000U, 0U, false}, mode, 550U) ==
           MEASUREMENT_PULSE_STATE_PAIR);

    for (int direction = -1; direction <= 1; direction += 2) {
        window = (MeasurementPulseWindow_t){0};
        for (unsigned i = 0U; i < MEASUREMENT_PULSE_WINDOW_COUNT; ++i) {
            sample = (MeasurementPulseSample_t){(uint16_t)(12000 + direction * 16 * (int)i), 0U, false};
            uint16_t state = MeasurementStatistics_UpdatePulseWindow(&window, sample, mode, i * 50U);
            assert(state == (i == 9U ? MEASUREMENT_PULSE_STATE_MOVING : MEASUREMENT_PULSE_STATE_PAIR));
        }
        for (uint32_t now = 500U; now <= 900U; now += 50U) {
            assert(MeasurementStatistics_UpdatePulseWindow(&window, sample, mode, now) ==
                   MEASUREMENT_PULSE_STATE_MOVING);
        }
        assert(MeasurementStatistics_UpdatePulseWindow(&window, sample, mode, 1750U) ==
               MEASUREMENT_PULSE_STATE_PAIR);
        window.moving = true;
        window.last_motion_ms = 1750U;
        assert(MeasurementStatistics_UpdatePulseWindow(&window, (MeasurementPulseSample_t){0},
               mode, 1800U) == MEASUREMENT_PULSE_STATE_PAIR);
        assert(MeasurementStatistics_UpdatePulseWindow(&window, sample,
               MEASUREMENT_PULSE_MODE_SINGLE, 1850U) == MEASUREMENT_PULSE_STATE_SINGLE);
    }
}

static void CheckFastMotion(void)
{
    const uint8_t count = MEASUREMENT_STATISTICS_SAMPLE_COUNT;
    const uint32_t spread = 288U;
    const uint32_t noise = 9U;
    uint32_t samples[MEASUREMENT_STATISTICS_SAMPLE_COUNT];
    MeasurementStatisticsResult_t result;

    /* 80 ticks per launch at 10 Hz is about 40 mm/s with the factory grid. */
    for (int direction = -1; direction <= 1; direction += 2) {
        for (unsigned i = 0U; i < count; ++i) {
            samples[i] = (uint32_t)(12000 + direction * 80 * (int)i);
        }
        assert(!MeasurementStatistics_Calculate(samples, count, spread, noise, 720U, 3600U, &result));
        assert(MeasurementStatistics_Track(samples, count, spread, noise, 720U, 3600U, &result));
        assert(result.recent_window_used);
        CheckFloat(result.mean_ticks, (float)samples[count - 1U]);
    }

    for (unsigned i = 0U; i < count; ++i) {
        samples[i] = 12000U + i % 2U;
    }
    assert(MeasurementStatistics_Track(samples, count, spread, noise, 720U, 3600U, &result));
    assert(!result.recent_window_used);
    CheckFloat(result.mean_ticks, 12000.5f);
    samples[count - 1U] = 15000U;
    assert(MeasurementStatistics_Track(samples, count, spread, noise, 720U, 3600U, &result));
    assert(!result.recent_window_used);
    assert(fabsf(result.mean_ticks - 12000.5f) < 1.0f);

    /* Новый уровень после резкого перемещения подтверждён пятью замерами. */
    samples[count - 5U] = 15001U;
    samples[count - 4U] = 15002U;
    samples[count - 3U] = 15000U;
    samples[count - 2U] = 15002U;
    assert(MeasurementStatistics_Track(samples, count, spread, noise, 720U, 3600U, &result));
    assert(result.recent_window_used);
    assert(fabsf(result.mean_ticks - 15001.0f) < 1.0f);
    CheckFloat(MeasurementStatistics_SmoothEstimate(12000.0f, &result, 100U),
               result.mean_ticks);

    /* Последовательность шума около 2 мм на неподвижном датчике.
     * Старое короткое окно принимало два согласованных приращения. */
    for (unsigned i = 0U; i < count; ++i) {
        samples[i] = 12000U;
    }
    samples[count - 3U] = 12010U;
    samples[count - 2U] = 12025U;
    samples[count - 1U] = 12040U;
    assert(MeasurementStatistics_Track(samples, count, spread, noise, 720U, 3600U, &result));
    assert(!result.recent_window_used && !result.regression_used);
    assert(fabsf(result.mean_ticks - 12000.0f) < 2.0f);

    /* При малом, но устойчивом смещении показание постепенно доходит
     * до нового уровня без зоны нечувствительности. */
    float filtered = 12000.0f;
    for (unsigned i = 0U; i < count; ++i) {
        samples[i] = 12020U;
    }
    assert(MeasurementStatistics_Track(samples, count, spread, noise, 720U, 3600U, &result));
    assert(!result.recent_window_used);
    for (unsigned i = 0U; i < 100U; ++i) {
        filtered = MeasurementStatistics_SmoothEstimate(filtered, &result, 100U);
    }
    assert(fabsf(filtered - 12020.0f) < 0.1f);

    for (unsigned i = 0U; i < count; ++i) {
        samples[i] = (i % 2U == 0U) ? 12000U : 15000U;
    }
    assert(!MeasurementStatistics_Track(samples, count, spread, noise, 720U, 3600U, &result));
    assert(MeasurementStatistics_CanHold(true, true, 200U, 300U));
    assert(!MeasurementStatistics_CanHold(true, true, 300U, 300U));
    assert(!MeasurementStatistics_CanHold(false, true, 100U, 300U));
    assert(!MeasurementStatistics_CanHold(true, false, 100U, 300U));
}

static void CheckRangeEdgeNoise(float tof_us, float expected_mm)
{
    const float range_mm = 700.0f;
    float filtered_mm = NAN;
    for (unsigned i = 0U; i < 300U; ++i) {
        /* About +/-0.7 mm around the actual calibrated endpoint. */
        float noisy_tof_us = tof_us + (i % 2U == 0U ? 0.2f : -0.2f);
        float candidate_mm = SensorCalGrid_Interpolate(noisy_tof_us) * 1000.0f;
        assert(fabsf(candidate_mm - expected_mm) > 0.65f);
        filtered_mm = MeasurementStatistics_FilterRangeEdges(
            filtered_mm, candidate_mm, range_mm, 20.0f, 100U);
        assert(isfinite(filtered_mm));
        if (i >= 220U) {
            assert(fabsf(filtered_mm - expected_mm) < 0.07f);
        }
    }

    /* Slowly varying error persists for seconds instead of alternating
     * every sample. Use a positive mean offset relative to the endpoint. */
    float direction = expected_mm == 0.0f ? 1.0f : -1.0f;
    filtered_mm = expected_mm + direction * 0.4f;
    for (unsigned i = 0U; i < 640U; ++i) {
        float candidate_mm = expected_mm + direction *
            ((i / 40U) % 2U == 0U ? 0.1f : 0.7f);
        filtered_mm = MeasurementStatistics_FilterRangeEdges(
            filtered_mm, candidate_mm, range_mm, 20.0f, 100U);
        assert(fabsf(filtered_mm - (expected_mm + direction * 0.4f)) < 0.15f);
    }
    float moved_mm = expected_mm + direction * 2.5f;
    CheckFloat(MeasurementStatistics_FilterRangeEdges(
        filtered_mm, moved_mm, range_mm, 20.0f, 100U), moved_mm);
}

int main(void)
{
    CheckT1Spikes();
    CheckPulseModes();
    CheckFastMotion();
    assert(!SensorCalGrid_Enabled());
    assert(isnan(SensorCalGrid_Interpolate(230.0f)));
    SensorCalibrationGrid grid = {
        .low_m = 0.05f, .high_m = 0.75f, .step_m = 0.15f, .interior = 4U
    };
    assert(SensorCalGrid_Configure(grid));
    CheckFloat(ModBus_GetParameter_Float(MB_ADDR_SENSOR_GRID_DISTANCE), 0.70f);
    CheckFloat(ModBus_GetParameter_Float(MB_ADDR_SENSOR_GRID_STEP), 0.15f);
    CheckFloat(ModBus_GetParameter_Float(MB_ADDR_SENSOR_GRID_LAST_STEP), 0.10f);
    assert(!SensorCalGrid_Configure((SensorCalibrationGrid){ .low_m = 1.0f, .high_m = 0.5f }));
    assert(!SensorCalGrid_Configure((SensorCalibrationGrid){ .high_m = 1.0f, .interior = 11U }));
    assert(!SensorCalGrid_Configure((SensorCalibrationGrid){ .high_m = 0.6f, .step_m = 0.15f, .interior = 4U }));
    assert(!SensorCalGrid_Configure((SensorCalibrationGrid){ .high_m = 0.5f, .step_m = 0.15f, .interior = 4U }));
    assert(!SensorCalGrid_Capture(12U, 360.0f));
    assert(SensorCalGrid_Capture(1U, 450.0f));
    assert(!SensorCalGrid_Capture(11U, 445.0f));
    assert(!SensorCalGrid_Capture(2U, 200.0f));
    for (uint16_t i = 0; i < grid.interior; ++i) {
        assert(SensorCalGrid_Capture(11U + i, 450.0f - 50.0f * (i + 1U)));
    }
    assert(isnan(SensorCalGrid_Interpolate(325.0f)));
    assert(!SensorCalGrid_Capture(2U, 190.0f));
    assert(SensorCalGrid_Capture(2U, 220.0f));
    CheckFloat(SensorCalGrid_Interpolate(325.0f), 0.425f);
    CheckFloat(SensorCalGrid_Interpolate(250.0f), 0.65f);
    CheckFloat(SensorCalGrid_Interpolate(235.0f), 0.69939427f);
    CheckFloat(SensorCalGrid_Interpolate(220.0f), 0.75f);
    CheckFloat(SensorCalGrid_Interpolate(450.0f), 0.05f);
    assert(SensorCalGrid_Interpolate(500.0f) < grid.low_m);
    assert(SensorCalGrid_Interpolate(150.0f) > grid.high_m);
    assert(SensorCalGrid_Configure(grid));
    CheckFloat(SensorCalGrid_Interpolate(235.0f), 0.69939427f);
    grid.step_m = 0.14f;
    assert(SensorCalGrid_Configure(grid));
    assert(isnan(SensorCalGrid_Interpolate(325.0f)));
    assert(SensorCalGrid_Capture(1U, 450.0f));
    for (uint16_t i = 0; i < grid.interior; ++i) {
        assert(SensorCalGrid_Capture(11U + i, 450.0f - 50.0f * (i + 1U)));
    }
    assert(SensorCalGrid_Capture(2U, 200.0f));
    /* A v149.4 EEPROM image has no stored nominal step. */
    ModBus_SetParameter_Float(MB_ADDR_SENSOR_GRID_STEP, 0.0f);
    SensorCalGrid_UpdateDerived();
    CheckFloat(SensorCalGrid_EffectiveStep(SensorCalGrid_Read()), 0.14f);
    CheckFloat(ModBus_GetParameter_Float(MB_ADDR_SENSOR_GRID_LAST_STEP), 0.14f);
    CheckFloat(SensorCalGrid_Interpolate(325.0f), 0.40f);
    assert(SensorCalGrid_Capture(12U, 351.0f));
    assert(isnan(SensorCalGrid_Interpolate(325.0f)));

    grid.interior = 0U;
    grid.step_m = 0.15f;
    assert(SensorCalGrid_Configure(grid));
    CheckFloat(ModBus_GetParameter_Float(MB_ADDR_SENSOR_GRID_LAST_STEP), 0.70f);
    assert(!SensorCalGrid_Capture(11U, 350.0f));
    assert(!SensorCalGrid_Capture(2U, 200.0f));
    assert(SensorCalGrid_Capture(1U, 450.0f));
    assert(SensorCalGrid_Capture(2U, 200.0f));
    CheckFloat(SensorCalGrid_Interpolate(325.0f), 0.40f);

    grid.interior = SENSOR_GRID_MAX_INTERIOR;
    grid.low_m = 0.0f;
    grid.high_m = 0.77f;
    grid.step_m = 0.07f;
    assert(SensorCalGrid_Configure(grid));
    assert(SensorCalGrid_Capture(1U, 450.0f));
    for (uint16_t i = 0; i < grid.interior; ++i) {
        assert(SensorCalGrid_Capture(11U + i, 450.0f - 25.0f * (i + 1U)));
    }
    assert(SensorCalGrid_Capture(2U, 175.0f));
    CheckFloat(SensorCalGrid_Interpolate(312.5f), 0.385f);

    grid.high_m = MODBUS_DEFAULT_WORKING_HEIGHT_M;
    grid.step_m = 0.15f;
    grid.interior = 4U;
    assert(SensorCalGrid_Configure(grid));
    CheckFloat(ModBus_GetParameter_Float(MB_ADDR_SENSOR_GRID_DISTANCE), 0.70f);
    CheckFloat(ModBus_GetParameter_Float(MB_ADDR_SENSOR_GRID_LAST_STEP), 0.10f);
    assert(isnan(SensorCalGrid_Interpolate(225.0f)));
    assert(SensorCalGrid_Capture(1U, 350.0f));
    for (uint16_t i = 0U; i < grid.interior; ++i) {
        assert(SensorCalGrid_Capture(11U + i, 350.0f - 50.0f * (i + 1U)));
    }
    assert(!SensorCalGrid_Capture(2U, 95.0f));
    assert(SensorCalGrid_Capture(2U, 117.0f));
    CheckFloat(SensorCalGrid_Interpolate(350.0f), 0.0f);
    CheckFloat(SensorCalGrid_Interpolate(225.0f), 0.375f);
    CheckFloat(SensorCalGrid_Interpolate(133.5f), 0.64994145f);
    CheckFloat(SensorCalGrid_Interpolate(117.0f), 0.70f);
    assert(SensorCalGrid_Interpolate(351.0f) < 0.0f);
    assert(SensorCalGrid_Interpolate(99.0f) > grid.high_m);

    const float factory_us[] = {
        285.2986145019531f, 243.01158142089844f, 199.1273193359375f,
        157.10879516601562f, 112.8657455444336f, 84.1388931274414f
    };
    assert(SensorCalGrid_Capture(1U, factory_us[0]));
    for (uint16_t i = 1U; i <= grid.interior; ++i) {
        assert(SensorCalGrid_Capture(10U + i, factory_us[i]));
    }
    assert(SensorCalGrid_Capture(2U, factory_us[5]));
    for (uint16_t i = 0U; i < 5U; ++i) {
        float low = (float)i * grid.step_m;
        float high = i == 4U ? grid.high_m : (float)(i + 1U) * grid.step_m;
        CheckFloat(SensorCalGrid_Interpolate(factory_us[i]), low);
        for (uint16_t j = 1U; j < 128U; ++j) {
            float time_us = factory_us[i] -
                (factory_us[i] - factory_us[i + 1U]) * ((float)j / 128.0f);
            float level = SensorCalGrid_Interpolate(time_us);
            assert(level >= low && level <= high);
            low = level;
        }
        CheckFloat(SensorCalGrid_Interpolate(factory_us[i + 1U]), high);
    }
    CheckRangeEdgeNoise(factory_us[0], 0.0f);
    CheckRangeEdgeNoise(factory_us[5], 700.0f);
    CheckFloat(MeasurementStatistics_FilterRangeEdges(NAN, -2.0f, 700.0f, 20.0f, 100U), -2.0f);
    assert(isnan(MeasurementStatistics_FilterRangeEdges(0.0f, -2.01f, 700.0f, 20.0f, 100U)));
    float lower_tof_us = factory_us[0] + 1.8f * (factory_us[0] - factory_us[1]) / 150.0f;
    float below_zero_mm = SensorCalGrid_Interpolate(lower_tof_us) * 1000.0f;
    assert(fabsf(below_zero_mm + 1.8f) < 0.001f);
    float near_zero_mm = MeasurementStatistics_FilterRangeEdges(
        0.0f, below_zero_mm, 700.0f, 20.0f, 100U);
    assert(isfinite(near_zero_mm) && near_zero_mm < 0.0f);
    CheckFloat(fmaxf(0.0f, near_zero_mm), 0.0f);
    CheckFloat(MeasurementStatistics_FilterRangeEdges(
        near_zero_mm, 2.0f, 700.0f, 20.0f, 100U), 2.0f);
    CheckFloat(MeasurementStatistics_FilterRangeEdges(699.8f, 701.0f, 700.0f, 20.0f, 100U), 701.0f);
    CheckFloat(MeasurementStatistics_FilterRangeEdges(700.0f, 720.0f, 700.0f, 20.0f, 100U), 720.0f);
    assert(isnan(MeasurementStatistics_FilterRangeEdges(720.0f, 720.1f, 700.0f, 20.0f, 100U)));
    CheckFloat(MeasurementStatistics_LevelPercent(0.0f, 700.0f), 0.0f);
    CheckFloat(MeasurementStatistics_LevelPercent(350.0f, 700.0f), 50.0f);
    CheckFloat(MeasurementStatistics_LevelPercent(700.0f, 700.0f), 100.0f);
    CheckFloat(MeasurementStatistics_LevelPercent(600.0f, 600.0f), 100.0f);
    assert(fabsf(MeasurementStatistics_LevelPercent(720.0f, 700.0f) - 102.85714f) < 0.0001f);
    assert(isnan(MeasurementStatistics_LevelPercent(700.0f, 0.0f)));
    assert(!MeasurementStatistics_OverflowActive(false, 700.9f, 700.0f, 1.0f));
    assert(MeasurementStatistics_OverflowActive(false, 701.0f, 700.0f, 1.0f));
    assert(MeasurementStatistics_OverflowActive(true, 700.1f, 700.0f, 1.0f));
    assert(MeasurementStatistics_OverflowActive(true, NAN, 700.0f, 1.0f));
    assert(!MeasurementStatistics_OverflowActive(true, 700.0f, 700.0f, 1.0f));
    assert(!MeasurementStatistics_OverflowActive(false, NAN, 700.0f, 1.0f));

    /* Проход через верхнюю точку с заводскими временами при импульсе 5 мкс. */
    float upper_slope_mm_per_us = 100.0f / (factory_us[4] - factory_us[5]);
    float overrun_tof_us = factory_us[5] - 19.0f / upper_slope_mm_per_us;
    assert(overrun_tof_us > 70.0f);
    float overrun_mm = SensorCalGrid_Interpolate(overrun_tof_us) * 1000.0f;
    assert(fabsf(overrun_mm - 719.0f) < 0.001f);
    assert(MeasurementStatistics_FilterRangeEdges(700.0f, overrun_mm,
                                                  700.0f, 20.0f, 100U) == overrun_mm);
    assert(isnan(MeasurementStatistics_FilterRangeEdges(0.0f, NAN, 700.0f, 20.0f, 100U)));
    CheckFloat(MeasurementStatistics_FilterRangeEdges(NAN, 0.7f, 700.0f, 20.0f, 100U), 0.7f);
    CheckFloat(MeasurementStatistics_FilterRangeEdges(0.0f, 3.1f, 700.0f, 20.0f, 100U), 3.1f);
    CheckFloat(MeasurementStatistics_FilterRangeEdges(700.0f, 696.0f, 700.0f, 20.0f, 100U), 696.0f);
    CheckFloat(MeasurementStatistics_FilterRangeEdges(10.0f, 0.0f, 700.0f, 20.0f, 100U), 0.0f);
    float steady_mm = 0.0f;
    for (unsigned i = 0U; i < 450U; ++i) {
        steady_mm = MeasurementStatistics_FilterRangeEdges(steady_mm, 0.7f, 700.0f, 20.0f, 100U);
    }
    assert(fabsf(steady_mm - 0.7f) < 0.001f);
    ModBus_SetParameter_Float(MB_ADDR_SENSOR_GRID_TOF(4U), 0.0f);
    assert(isnan(SensorCalGrid_Interpolate(factory_us[1])));
    puts("PASS: fast rise/fall, outliers, hold timeout, calibration grid and range edge noise");
}
