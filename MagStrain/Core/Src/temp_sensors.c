/**
 * @file    temp_sensors.c
 * @brief   Реализация модуля опроса 8 датчиков температуры LM75B
 */
#include "temp_sensors.h"
#include "lm75b.h"
#include "modbus.h"
#include "main.h"
#include <string.h>
#include <math.h>

/* ==========================================================================
   ЛОКАЛЬНЫЕ ПЕРЕМЕННЫЕ
   ========================================================================== */
static TempSensorsState_t ts_state;

/* Внешний доступ к I2C */
extern I2C_HandleTypeDef hi2c2;

/* ==========================================================================
   ВНУТРЕННИЕ ФУНКЦИИ
   ========================================================================== */

/**
 * @brief Проверка наличия устройства по адресу
 */
static bool check_device_present(uint8_t address_8bit)
{
    uint8_t dummy = 0;
    HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(&hi2c2, address_8bit,
                                                      2, TEMP_SENSOR_SCAN_TIMEOUT_MS);
    return (status == HAL_OK);
}

/**
 * @brief Чтение температуры одного датчика с защитой от ошибок
 */
static bool read_sensor_temperature(uint8_t address_8bit, float *temp)
{
    if (LM75B_ReadTemperature(address_8bit, temp) != HAL_OK) {
        return false;
    }

    /* Валидация диапазона */
    if (*temp < TEMP_MIN_VALID || *temp > TEMP_MAX_VALID || isnan(*temp)) {
        return false;
    }
    return true;
}

/**
 * @brief Обновление регистра Modbus float32 для датчика температуры
 */
static void update_temp_register(uint8_t sensor_idx, float value)
{
    /* Адреса регистров: 2600, 2602, 2604, ..., 2614 */
    uint16_t mb_addr = 2600 + (sensor_idx * 2);
    ModBus_SetParameter_Float(mb_addr, value);
}

/**
 * @brief Обновление регистра Modbus float32 для высоты датчика
 */
static void update_height_register(uint8_t sensor_idx, float value)
{
    /* Адреса регистров: 2500, 2502, 2504, ..., 2514 */
    uint16_t mb_addr = 2500 + (sensor_idx * 2);
    ModBus_SetParameter_Float(mb_addr, value);
}

/**
 * @brief Обновление регистра Modbus float32 для плотности по датчику
 */
static void update_density_register(uint8_t sensor_idx, float value)
{
    /* Адреса регистров: 2700, 2702, 2704, ..., 2714 */
    uint16_t mb_addr = 2700 + (sensor_idx * 2);
    ModBus_SetParameter_Float(mb_addr, value);
}

/**
 * @brief Запись "ошибочного" значения в регистр (0xFFFFFFFF)
 */
static void write_error_value(uint16_t mb_addr)
{
    uint16_t idx = ModBus_AddressToIndex_External(mb_addr);
    if (idx != 0xFFFF && (idx + 1) < 260) {
        /* Записываем 0xFFFFFFFF как float (NaN) */
        uint32_t err_val = 0xFFFFFFFF;
        float err_float;
        memcpy(&err_float, &err_val, 4);
        ModBus_SetParameter_Float(mb_addr, err_float);
    }
}

/* ==========================================================================
   ПУБЛИЧНЫЕ ФУНКЦИИ
   ========================================================================== */

void TempSensors_Init(void)
{
    memset(&ts_state, 0, sizeof(ts_state));

    /* Инициализируем все датчики как отсутствующие */
    for (uint8_t i = 0; i < TEMP_SENSORS_MAX_COUNT; i++) {
        ts_state.sensors[i].index = i + 1;
        ts_state.sensors[i].present = false;
        ts_state.sensors[i].valid = false;
        ts_state.sensors[i].temperature_c = NAN;
        ts_state.sensors[i].density_kg_m3 = NAN;
        ts_state.sensors[i].height_m = 0.0f;
    }

    ts_state.avg_liquid_temp = NAN;
    ts_state.avg_vapor_temp = NAN;
    ts_state.temp_at_density = NAN;

    USART2_Print("[TEMP] Инициализация модуля датчиков...\r\n");

    /* Сканируем шину I2C */
    ts_state.detected_count = TempSensors_ScanBus();
    ts_state.scan_done = true;
    ts_state.last_scan_time_ms = HAL_GetTick();

    /* Загружаем высоты из EEPROM */
    TempSensors_LoadHeightsFromEEPROM();

    USART2_BufInit();
    USART2_BufPrint("[TEMP] Обнаружено датчиков: ");
    USART2_BufPrintInt(ts_state.detected_count);
    USART2_BufPrint(" из ");
    USART2_BufPrintInt(TEMP_SENSORS_MAX_COUNT);
    USART2_BufPrint("\r\n");

    for (uint8_t i = 0; i < TEMP_SENSORS_MAX_COUNT; i++) {
        if (ts_state.sensors[i].present) {
            USART2_BufPrint("  [");
            USART2_BufPrintInt(i + 1);
            USART2_BufPrint("] Addr=0x");
            /* Вывод адреса в hex */
            char hex_buf[8];
            uint8_t addr = ts_state.sensors[i].address >> 1;
            hex_buf[0] = (addr >> 4) < 10 ? '0' + (addr >> 4) : 'A' + (addr >> 4) - 10;
            hex_buf[1] = (addr & 0x0F) < 10 ? '0' + (addr & 0x0F) : 'A' + (addr & 0x0F) - 10;
            hex_buf[2] = '\0';
            USART2_BufPrint(hex_buf);
            USART2_BufPrint(", H=");
            USART2_BufPrintFloat(ts_state.sensors[i].height_m);
            USART2_BufPrint(" м\r\n");
        }
    }
    USART2_BufFlush();
}

uint8_t TempSensors_ScanBus(void)
{
    uint8_t found = 0;

    /* LM75B имеет адреса 0x48 - 0x4F (7-bit) */
    for (uint8_t i = 0; i < TEMP_SENSORS_MAX_COUNT; i++) {
        uint8_t addr_7bit = LM75B_BASE_ADDRESS + i;
        uint8_t addr_8bit = addr_7bit << 1;

        if (check_device_present(addr_8bit)) {
            ts_state.sensors[found].address = addr_8bit;
            ts_state.sensors[found].index = found + 1;
            ts_state.sensors[found].present = true;
            ts_state.sensors[found].valid = false;
            ts_state.sensors[found].error_count = 0;
            found++;
        }
    }

    return found;
}

void TempSensors_ReadAll(void)
{
    uint32_t now = HAL_GetTick();

    for (uint8_t i = 0; i < TEMP_SENSORS_MAX_COUNT; i++) {
        if (!ts_state.sensors[i].present) {
            /* Записываем NaN в регистр для отсутствующего датчика */
            write_error_value(2600 + (i * 2));
            continue;
        }

        float temp;
        if (read_sensor_temperature(ts_state.sensors[i].address, &temp)) {
            ts_state.sensors[i].temperature_c = temp;
            ts_state.sensors[i].valid = true;
            ts_state.sensors[i].error_count = 0;
            update_temp_register(i, temp);
        } else {
            /* Ошибка чтения */
            ts_state.sensors[i].error_count++;
            ts_state.sensors[i].valid = false;

            /* После 10 ошибок подряд - помечаем как отсутствующий */
            if (ts_state.sensors[i].error_count > 10) {
                ts_state.sensors[i].present = false;
            }

            write_error_value(2600 + (i * 2));
        }
    }

    ts_state.last_read_time_ms = now;
}

void TempSensors_CalculateAverages(float level_m, float density_level)
{
    /* Расчет средней температуры жидкости (датчики ниже уровня) */
    float sum_liquid = 0.0f;
    uint8_t count_liquid = 0;

    /* Расчет средней температуры паровой фазы (датчики выше уровня) */
    float sum_vapor = 0.0f;
    uint8_t count_vapor = 0;

    /* Температура в точке измерения плотности */
    float temp_at_dens = NAN;
    float min_dist_to_density = 1e9f;

    /* Получаем высоту поплавка плотности из Modbus */
    float waveguide_len = ModBus_GetWaveguideLength();
    float probe_depth = ModBus_GetParameter_Float(MB_ADDR_PROBE_DEPTH);
    float density_float_level = waveguide_len - probe_depth; /* Примерная высота поплавка плотности */

    for (uint8_t i = 0; i < TEMP_SENSORS_MAX_COUNT; i++) {
        if (!ts_state.sensors[i].present || !ts_state.sensors[i].valid) {
            continue;
        }

        float h = ts_state.sensors[i].height_m;
        float t = ts_state.sensors[i].temperature_c;

        if (h <= level_m) {
            /* Датчик в жидкости */
            sum_liquid += t;
            count_liquid++;
        } else {
            /* Датчик в паровой фазе */
            sum_vapor += t;
            count_vapor++;
        }

        /* Ищем ближайший к поплавку плотности датчик */
        float dist = fabsf(h - density_float_level);
        if (dist < min_dist_to_density) {
            min_dist_to_density = dist;
            temp_at_dens = t;
        }
    }

    /* Записываем результаты */
    if (count_liquid > 0) {
        ts_state.avg_liquid_temp = sum_liquid / (float)count_liquid;
    } else {
        ts_state.avg_liquid_temp = NAN;
    }

    if (count_vapor > 0) {
        ts_state.avg_vapor_temp = sum_vapor / (float)count_vapor;
    } else {
        ts_state.avg_vapor_temp = NAN;
    }

    ts_state.temp_at_density = temp_at_dens;

    /* Обновляем регистры Modbus для средних температур */
    if (!isnan(ts_state.avg_liquid_temp)) {
        ModBus_SetParameter_Float(MB_ADDR_TEMP, ts_state.avg_liquid_temp);
        /* int16 регистр: температура * 100 */
        ModBus_SetParameter_Int(MB_ADDR_TEMP_INT,
                                (uint16_t)(ts_state.avg_liquid_temp * 100.0f));
    }

    if (!isnan(ts_state.avg_vapor_temp)) {
        ModBus_SetParameter_Float(MB_ADDR_TEMP_VAPOR, ts_state.avg_vapor_temp);
        ModBus_SetParameter_Int(MB_ADDR_TEMP_VAPOR_INT,
                                (uint16_t)(ts_state.avg_vapor_temp * 100.0f));
    }

    if (!isnan(ts_state.temp_at_density)) {
        ModBus_SetParameter_Float(MB_ADDR_TEMP_DENS, ts_state.temp_at_density);
        ModBus_SetParameter_Int(MB_ADDR_TEMP_DENS_INT,
                                (uint16_t)(ts_state.temp_at_density * 100.0f));
    }

    /* Обновляем количество датчиков в регистре 2414 */
    ModBus_SetParameter_Float(MB_ADDR_TEMP_SENS_COUNT, (float)ts_state.detected_count);
}

void TempSensors_LoadHeightsFromEEPROM(void)
{
    for (uint8_t i = 0; i < TEMP_SENSORS_MAX_COUNT; i++) {
        uint16_t mb_addr = 2500 + (i * 2);
        float height = ModBus_GetParameter_Float(mb_addr);

        /* Валидация: высота должна быть в разумных пределах */
        if (height >= 0.0f && height <= 10.0f && !isnan(height)) {
            ts_state.sensors[i].height_m = height;
        } else {
            /* Если в EEPROM мусор - вычисляем по формуле из ПМП */
            /* hti = ht1 + ht * (i-1), где ht1 ≈ 60 мм, ht ≈ L/8 */
            float waveguide = ModBus_GetWaveguideLength();
            float ht1 = 0.060f;  /* 60 мм */
            float ht_step = waveguide / 8.0f;
            ts_state.sensors[i].height_m = ht1 + ht_step * (float)i;

            /* Сохраняем вычисленное значение */
            ModBus_SetParameter_Float(mb_addr, ts_state.sensors[i].height_m);
        }

        update_height_register(i, ts_state.sensors[i].height_m);
    }
}

void TempSensors_SaveHeightsToEEPROM(void)
{
    for (uint8_t i = 0; i < TEMP_SENSORS_MAX_COUNT; i++) {
        uint16_t mb_addr = 2500 + (i * 2);
        ModBus_SetParameter_Float(mb_addr, ts_state.sensors[i].height_m);
    }
    /* EEPROM запишется автоматически через dirty flag */
}

void TempSensors_UpdateModbusRegisters(void)
{
    for (uint8_t i = 0; i < TEMP_SENSORS_MAX_COUNT; i++) {
        if (ts_state.sensors[i].present && ts_state.sensors[i].valid) {
            update_temp_register(i, ts_state.sensors[i].temperature_c);
            update_height_register(i, ts_state.sensors[i].height_m);
            update_density_register(i, ts_state.sensors[i].density_kg_m3);
        } else {
            write_error_value(2600 + (i * 2));
        }
    }
}

TempSensorsState_t* TempSensors_GetState(void)
{
    return &ts_state;
}

bool TempSensors_IsPresent(uint8_t index)
{
    if (index >= TEMP_SENSORS_MAX_COUNT) return false;
    return ts_state.sensors[index].present;
}

float TempSensors_GetTemperature(uint8_t index)
{
    if (index >= TEMP_SENSORS_MAX_COUNT) return NAN;
    if (!ts_state.sensors[index].present || !ts_state.sensors[index].valid) return NAN;
    return ts_state.sensors[index].temperature_c;
}

uint8_t TempSensors_GetDetectedCount(void)
{
    return ts_state.detected_count;
}
