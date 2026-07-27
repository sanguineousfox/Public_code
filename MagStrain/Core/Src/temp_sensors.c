/** @file temp_sensors.c @brief Опрос до 8 датчиков LM75B. */
#include "temp_sensors.h"
#include "i2c_config.h"
#include "lm75b.h"
#include "modbus.h"
#include "utils.h"

#include <math.h>
#include <string.h>

static TempSensorsState_t state;

static const uint16_t height_registers[TEMP_SENSORS_MAX_COUNT] = {
    MB_ADDR_TEMP_SENS_1_H, MB_ADDR_TEMP_SENS_2_H,
    MB_ADDR_TEMP_SENS_3_H, MB_ADDR_TEMP_SENS_4_H,
    MB_ADDR_TEMP_SENS_5_H, MB_ADDR_TEMP_SENS_6_H,
    MB_ADDR_TEMP_SENS_7_H, MB_ADDR_TEMP_SENS_8_H
};

static const uint16_t temperature_registers[TEMP_SENSORS_MAX_COUNT] = {
    MB_ADDR_TEMP_SENS_1_V, MB_ADDR_TEMP_SENS_2_V,
    MB_ADDR_TEMP_SENS_3_V, MB_ADDR_TEMP_SENS_4_V,
    MB_ADDR_TEMP_SENS_5_V, MB_ADDR_TEMP_SENS_6_V,
    MB_ADDR_TEMP_SENS_7_V, MB_ADDR_TEMP_SENS_8_V
};

static const uint16_t density_registers[TEMP_SENSORS_MAX_COUNT] = {
    MB_ADDR_DENS_SENS_1, MB_ADDR_DENS_SENS_2,
    MB_ADDR_DENS_SENS_3, MB_ADDR_DENS_SENS_4,
    MB_ADDR_DENS_SENS_5, MB_ADDR_DENS_SENS_6,
    MB_ADDR_DENS_SENS_7, MB_ADDR_DENS_SENS_8
};

static bool DevicePresent(uint8_t address)
{
    return HAL_I2C_IsDeviceReady(&hi2c2,
                                 address,
                                 2U,
                                 TEMP_SENSOR_SCAN_TIMEOUT_MS) == HAL_OK;
}

static bool ReadTemperature(uint8_t address, float *temperature)
{
    if (temperature == NULL ||
        LM75B_ReadTemperature(address, temperature) != HAL_OK) {
        return false;
    }

    return isfinite(*temperature) &&
           *temperature >= TEMP_MIN_VALID &&
           *temperature <= TEMP_MAX_VALID;
}

static void PublishInvalid(uint16_t address)
{
    ModBus_SetParameter_Float(address, NAN);
}

static void ResetSensor(uint8_t index)
{
    state.sensors[index].address = (uint8_t)((LM75B_BASE_ADDRESS + index) << 1);
    state.sensors[index].index = (uint8_t)(index + 1U);
    state.sensors[index].temperature_c = NAN;
    state.sensors[index].density_kg_m3 = NAN;
    state.sensors[index].present = false;
    state.sensors[index].valid = false;
    state.sensors[index].error_count = 0U;
}

uint8_t TempSensors_ScanBus(void)
{
    uint8_t found = 0U;
    uint8_t i;

    for (i = 0U; i < TEMP_SENSORS_MAX_COUNT; ++i) {
        uint8_t address = (uint8_t)((LM75B_BASE_ADDRESS + i) << 1);
        ResetSensor(i);
        state.sensors[i].address = address;
        if (DevicePresent(address)) {
            state.sensors[i].present = true;
            found++;
        }
        /* Сканирование I2C не должно задерживать уже принятый Modbus-кадр. */
        ModBus_Process();
    }

    state.detected_count = found;
    state.scan_done = true;
    state.last_scan_time_ms = HAL_GetTick();
    ModBus_SetParameter_Float(MB_ADDR_TEMP_SENS_COUNT, (float)found);
    return found;
}

void TempSensors_LoadHeightsFromEEPROM(void)
{
    float waveguide = ModBus_GetWaveguideLength();
    float step = waveguide / (float)TEMP_SENSORS_MAX_COUNT;
    uint8_t i;

    for (i = 0U; i < TEMP_SENSORS_MAX_COUNT; ++i) {
        float height = ModBus_GetParameter_Float(height_registers[i]);
        if (!isfinite(height) || height < 0.0f || height > 50.0f) {
            height = 0.060f + step * (float)i;
            ModBus_SetParameter_Float(height_registers[i], height);
        }
        state.sensors[i].height_m = height;
    }
}

void TempSensors_Init(void)
{
    memset(&state, 0, sizeof(state));
    state.avg_liquid_temp = NAN;
    state.avg_vapor_temp = NAN;
    state.temp_at_density = NAN;

    TempSensors_LoadHeightsFromEEPROM();
    (void)TempSensors_ScanBus();

    USART2_BufInit();
    USART2_BufPrint("[TEMP] Обнаружено датчиков: ");
    USART2_BufPrintInt(state.detected_count);
    USART2_BufPrint(" из 8\r\n");
    USART2_BufFlush();
}

void TempSensors_ReadAll(void)
{
    uint8_t i;
    uint8_t present_count = 0U;

    for (i = 0U; i < TEMP_SENSORS_MAX_COUNT; ++i) {
        TempSensor_t *sensor = &state.sensors[i];
        float temperature;

        if (!sensor->present) {
            PublishInvalid(temperature_registers[i]);
            ModBus_Process();
            continue;
        }

        if (ReadTemperature(sensor->address, &temperature)) {
            sensor->temperature_c = temperature;
            sensor->valid = true;
            sensor->error_count = 0U;
            present_count++;
            ModBus_SetParameter_Float(temperature_registers[i], temperature);
        } else {
            sensor->valid = false;
            sensor->temperature_c = NAN;
            sensor->error_count++;
            PublishInvalid(temperature_registers[i]);
            if (sensor->error_count >= TEMP_SENSOR_MAX_ERRORS) {
                sensor->present = false;
            } else {
                present_count++;
            }
        }

        /* Между транзакциями I2C немедленно обслуживаем Modbus. */
        ModBus_Process();
    }

    state.detected_count = present_count;
    state.last_read_time_ms = HAL_GetTick();
    ModBus_SetParameter_Float(MB_ADDR_TEMP_SENS_COUNT, (float)present_count);
}

void TempSensors_CalculateAverages(float level_m, float density_level_m)
{
    float liquid_sum = 0.0f;
    float vapor_sum = 0.0f;
    uint8_t liquid_count = 0U;
    uint8_t vapor_count = 0U;
    float nearest_temperature = NAN;
    float nearest_distance = INFINITY;
    uint8_t i;

    if (!isfinite(density_level_m)) {
        density_level_m = ModBus_GetWaveguideLength() -
                          ModBus_GetParameter_Float(MB_ADDR_PROBE_DEPTH);
    }

    for (i = 0U; i < TEMP_SENSORS_MAX_COUNT; ++i) {
        const TempSensor_t *sensor = &state.sensors[i];
        float distance;

        if (!sensor->present || !sensor->valid ||
            !isfinite(sensor->temperature_c)) {
            continue;
        }

        if (sensor->height_m <= level_m) {
            liquid_sum += sensor->temperature_c;
            liquid_count++;
        } else {
            vapor_sum += sensor->temperature_c;
            vapor_count++;
        }

        distance = fabsf(sensor->height_m - density_level_m);
        if (distance < nearest_distance) {
            nearest_distance = distance;
            nearest_temperature = sensor->temperature_c;
        }
    }

    state.avg_liquid_temp = (liquid_count > 0U) ?
        liquid_sum / (float)liquid_count : NAN;
    state.avg_vapor_temp = (vapor_count > 0U) ?
        vapor_sum / (float)vapor_count : NAN;
    state.temp_at_density = nearest_temperature;

    if (isfinite(state.avg_liquid_temp)) {
        ModBus_SetParameter_Float(MB_ADDR_TEMP, state.avg_liquid_temp);
        ModBus_SetParameter_Int(MB_ADDR_TEMP_INT,
            (uint16_t)(int16_t)(state.avg_liquid_temp * 100.0f));
    }
    if (isfinite(state.avg_vapor_temp)) {
        ModBus_SetParameter_Float(MB_ADDR_TEMP_VAPOR, state.avg_vapor_temp);
        ModBus_SetParameter_Int(MB_ADDR_TEMP_VAPOR_INT,
            (uint16_t)(int16_t)(state.avg_vapor_temp * 100.0f));
    }
    if (isfinite(state.temp_at_density)) {
        ModBus_SetParameter_Float(MB_ADDR_TEMP_DENS, state.temp_at_density);
        ModBus_SetParameter_Int(MB_ADDR_TEMP_DENS_INT,
            (uint16_t)(int16_t)(state.temp_at_density * 100.0f));
    }
}

void TempSensors_SaveHeightsToEEPROM(void)
{
    uint8_t i;
    for (i = 0U; i < TEMP_SENSORS_MAX_COUNT; ++i) {
        ModBus_SetParameter_Float(height_registers[i],
                                  state.sensors[i].height_m);
    }
    ModBus_ForceSaveToEEPROM();
}

void TempSensors_UpdateModbusRegisters(void)
{
    uint8_t i;
    for (i = 0U; i < TEMP_SENSORS_MAX_COUNT; ++i) {
        ModBus_SetParameter_Float(height_registers[i],
                                  state.sensors[i].height_m);
        if (state.sensors[i].present && state.sensors[i].valid) {
            ModBus_SetParameter_Float(temperature_registers[i],
                                      state.sensors[i].temperature_c);
            ModBus_SetParameter_Float(density_registers[i],
                                      state.sensors[i].density_kg_m3);
        } else {
            PublishInvalid(temperature_registers[i]);
            PublishInvalid(density_registers[i]);
        }
    }
}

TempSensorsState_t *TempSensors_GetState(void)
{
    return &state;
}

bool TempSensors_IsPresent(uint8_t index)
{
    return index < TEMP_SENSORS_MAX_COUNT && state.sensors[index].present;
}

float TempSensors_GetTemperature(uint8_t index)
{
    if (index >= TEMP_SENSORS_MAX_COUNT ||
        !state.sensors[index].present ||
        !state.sensors[index].valid) {
        return NAN;
    }
    return state.sensors[index].temperature_c;
}

uint8_t TempSensors_GetDetectedCount(void)
{
    return state.detected_count;
}
