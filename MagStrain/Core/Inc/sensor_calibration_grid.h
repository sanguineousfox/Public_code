#ifndef SENSOR_CALIBRATION_GRID_H
#define SENSOR_CALIBRATION_GRID_H

#include <stdbool.h>
#include <stdint.h>

#define SENSOR_GRID_MAX_INTERIOR 10U

typedef struct {
    float low_m;
    float high_m;
    float step_m;  /* Zero selects the legacy equally spaced grid. */
    uint16_t interior;
} SensorCalibrationGrid;

SensorCalibrationGrid SensorCalGrid_Read(void);
float SensorCalGrid_EffectiveStep(SensorCalibrationGrid grid);
bool SensorCalGrid_Enabled(void);
bool SensorCalGrid_Configure(SensorCalibrationGrid grid);
void SensorCalGrid_UpdateDerived(void);
bool SensorCalGrid_Capture(uint16_t command, float tof_us);
float SensorCalGrid_Interpolate(float tof_us);

#endif
