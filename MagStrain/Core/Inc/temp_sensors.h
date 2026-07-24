/**
@file    temp_sensors.h
@brief   Модуль опроса до 8 датчиков температуры LM75B по шине I2C
*/
#ifndef TEMP_SENSORS_H
#define TEMP_SENSORS_H
#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define TEMP_SENSORS_MAX_COUNT      8
#define TEMP_SENSOR_SCAN_TIMEOUT_MS 50
#define TEMP_SENSOR_READ_TIMEOUT_MS 100
#define LM75B_BASE_ADDRESS          0x48
#define TEMP_MIN_VALID              (-55.0f)
#define TEMP_MAX_VALID              (130.0f)
#define TEMP_ERROR_VALUE            0xFFFFFFFF

typedef struct {
    uint8_t  address;
    uint8_t  index;
    float    height_m;
    float    temperature_c;
    float    density_kg_m3;
    bool     present;
    bool     valid;
    uint32_t error_count;
} TempSensor_t;

typedef struct {
    TempSensor_t sensors[TEMP_SENSORS_MAX_COUNT];
    uint8_t      detected_count;
    float        avg_liquid_temp;
    float        avg_vapor_temp;
    float        temp_at_density;
    bool         scan_done;
    uint32_t     last_scan_time_ms;
    uint32_t     last_read_time_ms;
} TempSensorsState_t;

void TempSensors_Init(void);
uint8_t TempSensors_ScanBus(void);
void TempSensors_ReadAll(void);
void TempSensors_CalculateAverages(float level_m, float density_level);
void TempSensors_LoadHeightsFromEEPROM(void);
void TempSensors_SaveHeightsToEEPROM(void);
void TempSensors_UpdateModbusRegisters(void);
TempSensorsState_t* TempSensors_GetState(void);
bool TempSensors_IsPresent(uint8_t index);
float TempSensors_GetTemperature(uint8_t index);
uint8_t TempSensors_GetDetectedCount(void);

#ifdef __cplusplus
}
#endif
#endif
