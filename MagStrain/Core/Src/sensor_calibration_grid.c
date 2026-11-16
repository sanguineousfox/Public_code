#include "sensor_calibration_grid.h"
#include "modbus.h"
#include <math.h>

SensorCalibrationGrid SensorCalGrid_Read(void)
{
    return (SensorCalibrationGrid){
        .low_m = ModBus_GetParameter_Float(MB_ADDR_SENSOR_GRID_LOW),
        .high_m = ModBus_GetParameter_Float(MB_ADDR_SENSOR_GRID_HIGH),
        .step_m = ModBus_GetParameter_Float(MB_ADDR_SENSOR_GRID_STEP),
        .interior = ModBus_GetParameter_Int(MB_ADDR_SENSOR_GRID_INTERIOR)
    };
}

float SensorCalGrid_EffectiveStep(SensorCalibrationGrid grid)
{
    if (grid.step_m != 0.0f) return grid.step_m;
    return (grid.high_m - grid.low_m) / (float)(grid.interior + 1U);
}

static float LastStepM(SensorCalibrationGrid grid, float step_m)
{
    return (grid.high_m - grid.low_m) - (float)grid.interior * step_m;
}

bool SensorCalGrid_Enabled(void)
{
    return ModBus_GetParameter_Int(MB_ADDR_SENSOR_GRID_ENABLED) == 1U;
}

void SensorCalGrid_UpdateDerived(void)
{
    SensorCalibrationGrid grid = SensorCalGrid_Read();

    ModBus_SetParameter_Float(MB_ADDR_SENSOR_GRID_DISTANCE, grid.high_m - grid.low_m);
    ModBus_SetParameter_Float(MB_ADDR_SENSOR_GRID_LAST_STEP,
                              LastStepM(grid, SensorCalGrid_EffectiveStep(grid)));
}

static void ClearCaptures(void)
{
    for (uint16_t i = 0U; i < SENSOR_GRID_MAX_INTERIOR + 2U; ++i) {
        ModBus_SetParameter_Float(MB_ADDR_SENSOR_GRID_TOF(i), 0.0f);
    }
}

bool SensorCalGrid_Configure(SensorCalibrationGrid grid)
{
    if (!isfinite(grid.low_m) || !isfinite(grid.high_m) || !isfinite(grid.step_m) ||
        grid.low_m < 0.0f || grid.high_m <= grid.low_m ||
        grid.high_m > 50.0f || grid.step_m < 0.0f || grid.step_m > 50.0f ||
        grid.interior > SENSOR_GRID_MAX_INTERIOR) {
        return false;
    }

    float step_m = SensorCalGrid_EffectiveStep(grid);
    if (!(LastStepM(grid, step_m) > 0.0f) ||
        grid.low_m + (float)grid.interior * step_m >= grid.high_m) {
        return false;
    }

    SensorCalibrationGrid previous = SensorCalGrid_Read();
    if (!SensorCalGrid_Enabled() || previous.low_m != grid.low_m ||
        previous.high_m != grid.high_m || previous.interior != grid.interior ||
        (grid.interior != 0U && SensorCalGrid_EffectiveStep(previous) != step_m)) {
        ClearCaptures();
    }
    ModBus_SetParameter_Float(MB_ADDR_SENSOR_GRID_LOW, grid.low_m);
    ModBus_SetParameter_Float(MB_ADDR_SENSOR_GRID_HIGH, grid.high_m);
    ModBus_SetParameter_Float(MB_ADDR_SENSOR_GRID_STEP, grid.step_m);
    ModBus_SetParameter_Int(MB_ADDR_SENSOR_GRID_INTERIOR, grid.interior);
    ModBus_SetParameter_Int(MB_ADDR_SENSOR_GRID_ENABLED, 1U);
    SensorCalGrid_UpdateDerived();
    return true;
}

bool SensorCalGrid_Capture(uint16_t command, float tof_us)
{
    SensorCalibrationGrid grid = SensorCalGrid_Read();
    uint16_t last = grid.interior + 1U;
    uint16_t index;

    if (!SensorCalGrid_Enabled() || !isfinite(tof_us) ||
        tof_us <= 0.0f || tof_us > 50000.0f ||
        grid.interior > SENSOR_GRID_MAX_INTERIOR) {
        return false;
    }

    if (command == 1U) {
        ClearCaptures();
        index = 0U;
    } else {
        if (command == 2U) {
            index = last;
        } else if (command >= 11U && command < 11U + grid.interior) {
            index = command - 10U;
        } else {
            return false;
        }

        float previous_us = ModBus_GetParameter_Float(MB_ADDR_SENSOR_GRID_TOF(index - 1U));
        float delta_us = previous_us - tof_us;
        float step_m = SensorCalGrid_EffectiveStep(grid);
        float interval_m = index == last ? LastStepM(grid, step_m) : step_m;
        if (!(delta_us > 0.0f)) return false;
        float speed_m_s = interval_m * 1000000.0f / delta_us;
        if (speed_m_s < 2000.0f || speed_m_s > 4000.0f) return false;

        /* Capturing a point again starts the remaining part of the sequence. */
        for (uint16_t i = index + 1U; i <= last; ++i) {
            ModBus_SetParameter_Float(MB_ADDR_SENSOR_GRID_TOF(i), 0.0f);
        }
    }

    ModBus_SetParameter_Float(MB_ADDR_SENSOR_GRID_TOF(index), tof_us);
    return true;
}

/* PCHIP: взвешенное гармоническое среднее наклонов соседних участков.
 * Ось времени направлена от большего ToF к меньшему, оба наклона > 0. */
static float InteriorSlope(float previous_width, float next_width,
                           float previous_slope, float next_slope)
{
    float previous_weight = 2.0f * next_width + previous_width;
    float next_weight = next_width + 2.0f * previous_width;
    return (previous_weight + next_weight) /
        (previous_weight / previous_slope + next_weight / next_slope);
}

float SensorCalGrid_Interpolate(float tof_us)
{
    if (!SensorCalGrid_Enabled() || !isfinite(tof_us) || tof_us <= 0.0f) return NAN;

    SensorCalibrationGrid grid = SensorCalGrid_Read();
    if (grid.interior > SENSOR_GRID_MAX_INTERIOR || !isfinite(grid.low_m) ||
        !isfinite(grid.high_m) || !(grid.high_m > grid.low_m)) {
        return NAN;
    }

    uint16_t last = grid.interior + 1U;
    float step_m = SensorCalGrid_EffectiveStep(grid);
    float last_step_m = LastStepM(grid, step_m);
    if (!(step_m > 0.0f && last_step_m > 0.0f)) return NAN;

    float times[SENSOR_GRID_MAX_INTERIOR + 2U];
    times[0] = ModBus_GetParameter_Float(MB_ADDR_SENSOR_GRID_TOF(0U));
    if (!isfinite(times[0])) return NAN;

    uint16_t segment = 0U;
    for (uint16_t i = 1U; i <= last; ++i) {
        times[i] = ModBus_GetParameter_Float(MB_ADDR_SENSOR_GRID_TOF(i));
        if (!(times[i - 1U] > times[i] && times[i] > 0.0f)) return NAN;
        if (tof_us < times[i] && i < last) {
            segment = i;
        }
    }

    float left_m = grid.low_m + (float)segment * step_m;
    bool final_segment = segment + 1U == last;
    float right_m = final_segment ? grid.high_m : grid.low_m + (float)(segment + 1U) * step_m;
    float rise = right_m - left_m;
    float width = times[segment] - times[segment + 1U];
    float offset = times[segment] - tof_us;

    /* Линейное продолжение крайних участков для проверки рабочего хода. */
    if (tof_us >= times[0]) return grid.low_m + offset * (rise / width);
    if (tof_us <= times[last]) {
        return grid.high_m + (times[last] - tof_us) * (rise / width);
    }
    if (tof_us == times[segment + 1U]) return right_m;
    if (last == 1U) return left_m + (offset / width) * rise;

    float slope = rise / width;
    float left_slope = slope;
    float right_slope = slope;
    if (segment > 0U) {
        float previous_width = times[segment - 1U] - times[segment];
        float previous_m = grid.low_m + (float)(segment - 1U) * step_m;
        left_slope = InteriorSlope(previous_width, width,
                                   (left_m - previous_m) / previous_width, slope);
    }
    if (!final_segment) {
        float next_width = times[segment + 1U] - times[segment + 2U];
        float next_m = segment + 2U == last ?
            grid.high_m : grid.low_m + (float)(segment + 2U) * step_m;
        right_slope = InteriorSlope(width, next_width, slope,
                                    (next_m - right_m) / next_width);
    }

    float t = offset / width;
    float u = 1.0f - t;
    float level = left_m + t * t * (3.0f - 2.0f * t) * rise +
        t * u * width * (u * left_slope - t * right_slope);
    return fminf(right_m, fmaxf(left_m, level));
}
