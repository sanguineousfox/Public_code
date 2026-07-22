#include "calc.h"
#include "modbus.h"
#include "main.h"
#include <math.h>
#include <string.h>

/* ==========================================================================
ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ
========================================================================== */
static GradTable_t grad_table = {0};
static bool grad_table_valid = false;

/* Датчики температуры */
#define MAX_TEMP_SENSORS        8
typedef struct {
    float   height;     /* Высота установки (м) */
    float   temperature; /* Температура (°C) */
    bool    valid;      /* Флаг валидности */
} TempSensor_t;

static TempSensor_t temp_sensors[MAX_TEMP_SENSORS] = {0};
static uint8_t temp_sensors_count = 0;

/* ==========================================================================
ГРАДУИРОВОЧНАЯ ТАБЛИЦА
========================================================================== */

/**
 * @brief Инициализация градуировочной таблицы
 */
void GradTable_Init(void)
{
    /* Читаем параметры из ModBus регистров */
    uint16_t points = (uint16_t)ModBus_GetParameter_Float(MB_ADDR_CALIB_POINTS);

    if (points > 0 && points <= MAX_GRAD_POINTS) {
        grad_table.points_count = points;
        grad_table.start_height = ModBus_GetParameter_Float(MB_ADDR_CAL_LOW_LVL);
        grad_table.step = ModBus_GetParameter_Float(MB_ADDR_TANK_HEIGHT) / (float)(points - 1);
        grad_table.tank_height = ModBus_GetParameter_Float(MB_ADDR_TANK_HEIGHT);
        grad_table.tank_volume = ModBus_GetParameter_Float(MB_ADDR_TANK_VOLUME);

        /* Читаем объемы из регистров ModBus (адреса 32778-38778) */
        for (uint16_t i = 0; i < points; i++) {
            uint16_t reg_addr = GRAD_TABLE_START_ADDR + 10 + (i * 2); /* 32778 + i*2 */
            grad_table.volumes[i] = ModBus_GetParameter_Float(reg_addr);
        }

        grad_table_valid = true;
    } else {
        grad_table_valid = false;
    }
}

/**
 * @brief Очистка градуировочной таблицы
 */
void GradTable_Clear(void)
{
    memset(&grad_table, 0, sizeof(grad_table));
    grad_table_valid = false;
}

/**
 * @brief Установка точки в градуировочной таблице
 */
bool GradTable_SetPoint(uint16_t index, float volume_m3)
{
    if (index >= MAX_GRAD_POINTS) {
        return false;
    }

    grad_table.volumes[index] = volume_m3;

    if (index >= grad_table.points_count) {
        grad_table.points_count = index + 1;
    }

    grad_table_valid = true;
    return true;
}

/**
 * @brief Получение точки из градуировочной таблицы
 */
float GradTable_GetPoint(uint16_t index)
{
    if (index >= MAX_GRAD_POINTS || !grad_table_valid) {
        return 0.0f;
    }

    return grad_table.volumes[index];
}

/**
 * @brief Проверка валидности таблицы
 */
bool GradTable_IsValid(void)
{
    return grad_table_valid;
}

/**
 * @brief Получение количества точек
 */
uint16_t GradTable_GetPointsCount(void)
{
    return grad_table.points_count;
}

/**
 * @brief Расчет линейной градуировочной таблицы
 */
void GradTable_CalculateLinear(float start_volume, float end_volume)
{
    if (grad_table.points_count < 2) {
        return;
    }

    float step_volume = (end_volume - start_volume) / (float)(grad_table.points_count - 1);

    for (uint16_t i = 0; i < grad_table.points_count; i++) {
        grad_table.volumes[i] = start_volume + (float)i * step_volume;
    }

    grad_table_valid = true;
}

/* ==========================================================================
РАСЧЕТ ОБЪЕМА
========================================================================== */

/**
 * @brief Расчет объема для вертикального резервуара
 */
float Calc_Volume_Vertical(float level_m, float diameter_m, float tank_volume)
{
    if (diameter_m <= 0.0f || tank_volume <= 0.0f) {
        return 0.0f;
    }

    float radius = diameter_m / 2.0f;
    float cross_section_area = 3.14159265f * radius * radius;

    /* Объем = площадь сечения × уровень */
    float volume = cross_section_area * level_m;

    /* Ограничиваем полным объемом резервуара */
    if (volume > tank_volume) {
        volume = tank_volume;
    }

    return volume;
}

/**
 * @brief Расчет объема для горизонтального резервуара с плоскими днищами
 * Формула для частично заполненного горизонтального цилиндра
 */
float Calc_Volume_HorizontalFlat(float level_m, float diameter_m, float length_m)
{
    if (diameter_m <= 0.0f || length_m <= 0.0f) {
        return 0.0f;
    }

    float radius = diameter_m / 2.0f;

    /* Если уровень выше диаметра, ограничиваем */
    if (level_m >= diameter_m) {
        return 3.14159265f * radius * radius * length_m;
    }

    /* Если уровень нулевой или отрицательный */
    if (level_m <= 0.0f) {
        return 0.0f;
    }

    /* Расчет площади сегмента круга */
    float h = level_m;
    float r = radius;

    /* Площадь сегмента: A = r²·arccos((r-h)/r) - (r-h)·√(2rh-h²) */
    float term1 = r * r * acosf((r - h) / r);
    float term2 = (r - h) * sqrtf(2.0f * r * h - h * h);
    float segment_area = term1 - term2;

    /* Объем = площадь сегмента × длина */
    return segment_area * length_m;
}

/**
 * @brief Расчет объема для горизонтального резервуара с эллиптическими днищами
 */
float Calc_Volume_HorizontalEllipse(float level_m, float diameter_m, float length_m)
{
    /* Для эллиптических днищ используем упрощенную формулу */
    float cylinder_volume = Calc_Volume_HorizontalFlat(level_m, diameter_m, length_m);

    /* Добавляем объем эллиптических днищ (приблизительно 10% от объема цилиндра) */
    float end_caps_volume = cylinder_volume * 0.1f;

    return cylinder_volume + end_caps_volume;
}

/**
 * @brief Расчет объема по градуировочной таблице (линейная интерполяция)
 */
float Calc_Volume_FromTable(float level_m)
{
    if (!grad_table_valid || grad_table.points_count < 2) {
        return 0.0f;
    }

    /* Находим индекс точки */
    float relative_level = (level_m - grad_table.start_height) / grad_table.step;

    if (relative_level < 0.0f) {
        return 0.0f;
    }

    if (relative_level >= (float)(grad_table.points_count - 1)) {
        return grad_table.volumes[grad_table.points_count - 1];
    }

    uint16_t idx1 = (uint16_t)relative_level;
    uint16_t idx2 = idx1 + 1;
    float fraction = relative_level - (float)idx1;

    /* Линейная интерполяция */
    float volume = grad_table.volumes[idx1] +
                   fraction * (grad_table.volumes[idx2] - grad_table.volumes[idx1]);

    return volume;
}

/**
 * @brief Основной расчет объема
 */
float Calc_Volume(float level_m, TankType_t tank_type)
{
    float tank_height = ModBus_GetParameter_Float(MB_ADDR_TANK_HEIGHT);
    float tank_volume = ModBus_GetParameter_Float(MB_ADDR_TANK_VOLUME);

    switch (tank_type) {
        case TANK_VERTICAL:
            return Calc_Volume_Vertical(level_m, tank_height, tank_volume);

        case TANK_HORIZ_FLAT:
            return Calc_Volume_HorizontalFlat(level_m, tank_height, tank_height);

        case TANK_GRAD_TABLE:
            return Calc_Volume_FromTable(level_m);

        case TANK_HORIZ_ELLIPSE:
            return Calc_Volume_HorizontalEllipse(level_m, tank_height, tank_height);

        default:
            return 0.0f;
    }
}

/* ==========================================================================
РАСЧЕТ ПЛОТНОСТИ
========================================================================== */

/**
 * @brief Расчет плотности для произвольной жидкости
 */
float Calc_Density_Arbitrary(float density_orig, float temp_orig, float temp_current, float expansion_coef)
{
    if (expansion_coef <= 0.0f) {
        return density_orig;
    }

    float delta_temp = temp_current - temp_orig;
    float density = density_orig / (1.0f + expansion_coef * delta_temp);

    return density;
}

/**
 * @brief Расчет плотности для нефтепродуктов (упрощенно по ГОСТ 8.587)
 */
float Calc_Density_Petroleum(float density_orig, float temp_orig, float temp_current)
{
    /* Упрощенный расчет с коэффициентом расширения 0.0008 1/°C */
    return Calc_Density_Arbitrary(density_orig, temp_orig, temp_current, 0.0008f);
}

/**
 * @brief Расчет плотности для СУГ по ГОСТ 28656
 */
float Calc_Density_LPG(float propane, float butane, float isobutane, float temp)
{
    /* Плотности компонентов при 15°C (кг/м³) */
    const float density_propane = 493.0f;
    const float density_butane = 573.0f;
    const float density_isobutane = 549.0f;

    /* Сумма долей должна быть 100% */
    float total = propane + butane + isobutane;
    if (total <= 0.0f) {
        return 0.0f;
    }

    /* Взвешенная средняя плотность */
    float density = (propane * density_propane +
                     butane * density_butane +
                     isobutane * density_isobutane) / total;

    /* Коррекция на температуру (упрощенно) */
    float temp_correction = 1.0f - 0.002f * (temp - 15.0f);

    return density * temp_correction;
}

/**
 * @brief Основной расчет плотности
 */
float Calc_Density(float density_orig, float temp_orig, float temp_current,
                   float expansion_coef, uint8_t medium_type)
{
    switch (medium_type) {
        case 0: /* Произвольная жидкость */
            return Calc_Density_Arbitrary(density_orig, temp_orig, temp_current, expansion_coef);

        case 1: /* Нефтепродукты */
            return Calc_Density_Petroleum(density_orig, temp_orig, temp_current);

        case 2: /* СУГ */
            {
                float propane = ModBus_GetParameter_Float(MB_ADDR_PROPANE_RATIO);
                float butane = ModBus_GetParameter_Float(MB_ADDR_BUTANE_RATIO);
                float isobutane = ModBus_GetParameter_Float(MB_ADDR_ISOBUTANE_RATIO);
                return Calc_Density_LPG(propane, butane, isobutane, temp_current);
            }

        default:
            return density_orig;
    }
}

/* ==========================================================================
РАСЧЕТ МАССЫ
========================================================================== */

/**
 * @brief Расчет массы
 */
float Calc_Mass(float volume, float density)
{
    return volume * density;
}

/* ==========================================================================
СТАНДАРТНЫЕ УСЛОВИЯ
========================================================================== */

/**
 * @brief Приведение объема к стандартным условиям
 */
float Calc_Volume_Standard(float volume, float temp, float temp_standard)
{
    if (temp <= 0.0f) {
        return volume;
    }

    /* Приведение к стандартной температуре (15 или 20 °C) */
    float correction = (273.15f + temp_standard) / (273.15f + temp);

    return volume * correction;
}

/**
 * @brief Приведение плотности к стандартным условиям
 */
float Calc_Density_Standard(float density, float temp, float temp_standard)
{
    if (temp <= 0.0f) {
        return density;
    }

    /* Обратная коррекция для плотности */
    float correction = (273.15f + temp) / (273.15f + temp_standard);

    return density * correction;
}

/* ==========================================================================
ТЕМПЕРАТУРА
========================================================================== */

/**
 * @brief Инициализация модуля температуры
 */
void Calc_Temperature_Init(void)
{
    memset(temp_sensors, 0, sizeof(temp_sensors));
    temp_sensors_count = 0;
}

/**
 * @brief Обновление данных датчика температуры
 */
void Calc_Temperature_Update(uint8_t sensor_num, float height, float temp)
{
    if (sensor_num >= MAX_TEMP_SENSORS) {
        return;
    }

    temp_sensors[sensor_num].height = height;
    temp_sensors[sensor_num].temperature = temp;
    temp_sensors[sensor_num].valid = true;

    if (sensor_num >= temp_sensors_count) {
        temp_sensors_count = sensor_num + 1;
    }
}

/**
 * @brief Получение средней температуры жидкости
 */
float Calc_Temperature_GetLiquid(void)
{
    float sum = 0.0f;
    uint8_t count = 0;

    /* Получаем текущий уровень */
    float level = ModBus_GetParameter_Float(MB_ADDR_LEVEL);

    /* Усредняем датчики ниже уровня */
    for (uint8_t i = 0; i < temp_sensors_count; i++) {
        if (temp_sensors[i].valid && temp_sensors[i].height <= level) {
            sum += temp_sensors[i].temperature;
            count++;
        }
    }

    return (count > 0) ? (sum / (float)count) : 0.0f;
}

/**
 * @brief Получение средней температуры паровой фазы
 */
float Calc_Temperature_GetVapor(void)
{
    float sum = 0.0f;
    uint8_t count = 0;

    /* Получаем текущий уровень */
    float level = ModBus_GetParameter_Float(MB_ADDR_LEVEL);

    /* Усредняем датчики выше уровня */
    for (uint8_t i = 0; i < temp_sensors_count; i++) {
        if (temp_sensors[i].valid && temp_sensors[i].height > level) {
            sum += temp_sensors[i].temperature;
            count++;
        }
    }

    return (count > 0) ? (sum / (float)count) : 0.0f;
}

/**
 * @brief Получение количества датчиков температуры
 */
uint8_t Calc_Temperature_GetCount(void)
{
    return temp_sensors_count;
}

/* ==========================================================================
ДЕМПФИРОВАНИЕ
========================================================================== */

/**
 * @brief Экспоненциальное сглаживание
 */
float Calc_Damping(float current_value, float prev_value, float dt, float time_constant)
{
    if (time_constant <= 0.0f || dt <= 0.0f) {
        return current_value;
    }

    float alpha = dt / time_constant;
    if (alpha > 1.0f) {
        alpha = 1.0f;
    }

    return prev_value + alpha * (current_value - prev_value);
}

/* ==========================================================================
ОСНОВНОЙ ЦИКЛ РАСЧЕТОВ
========================================================================== */

/**
 * @brief Выполнение всех расчетов
 */
void Calc_All(void)
{
    /* Получаем параметры */
    float level = ModBus_GetParameter_Float(MB_ADDR_LEVEL);
    float temp = Calc_Temperature_GetLiquid();
    TankType_t tank_type = (TankType_t)(uint16_t)ModBus_GetParameter_Float(MB_ADDR_TANK_GEOM);

    /* Расчет объема */
    float volume = Calc_Volume(level, tank_type);
    ModBus_SetParameter_Float(MB_ADDR_VOLUME, volume);

    /* Расчет плотности */
    float density_orig = ModBus_GetParameter_Float(MB_ADDR_DENSITY_ORIG);
    float temp_orig = ModBus_GetParameter_Float(MB_ADDR_TEMP_ORIG);
    float expansion_coef = ModBus_GetParameter_Float(MB_ADDR_EXPANSION_COEF);
    uint8_t medium_type = (uint8_t)ModBus_GetParameter_Float(MB_ADDR_MEDIUM_TYPE);

    float density = Calc_Density(density_orig, temp_orig, temp, expansion_coef, medium_type);
    ModBus_SetParameter_Float(MB_ADDR_DENSITY, density);

    /* Расчет массы */
    float mass = Calc_Mass(volume, density);
    ModBus_SetParameter_Float(MB_ADDR_MASS, mass);

    /* Приведение к стандартным условиям */
    float temp_std = ModBus_GetParameter_Float(MB_ADDR_TEMP_STD);
    float volume_std = Calc_Volume_Standard(volume, temp, temp_std);
    float density_std = Calc_Density_Standard(density, temp, temp_std);

    ModBus_SetParameter_Float(MB_ADDR_VOLUME_STD, volume_std);
    ModBus_SetParameter_Float(MB_ADDR_DENSITY_STD, density_std);

    /* Процент заполнения */
    float tank_height = ModBus_GetParameter_Float(MB_ADDR_TANK_HEIGHT);
    float percent = (tank_height > 0.0f) ? (level / tank_height * 100.0f) : 0.0f;
    ModBus_SetParameter_Float(MB_ADDR_PERCENT, percent);
}
