#ifndef __CALC_H
#define __CALC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ==========================================================================
КОНСТАНТЫ
========================================================================== */
#define MAX_GRAD_POINTS         3001        /* Максимальное количество точек градуировки */
#define GRAD_TABLE_START_ADDR   32768       /* Начальный адрес градуировочной таблицы в ModBus */

/* ==========================================================================
ТИПЫ РЕЗЕРВУАРОВ
========================================================================== */
typedef enum {
    TANK_VERTICAL = 0,              /* Вертикальный цилиндрический */
    TANK_HORIZ_FLAT = 1,            /* Горизонтальный с плоскими днищами */
    TANK_GRAD_TABLE = 2,            /* По градуировочной таблице */
    TANK_HORIZ_ELLIPSE = 3          /* Горизонтальный с эллиптическими днищами */
} TankType_t;

/* ==========================================================================
СТРУКТУРА ГРАДУИРОВОЧНОЙ ТАБЛИЦЫ
========================================================================== */
typedef struct {
    uint16_t    points_count;           /* Количество точек */
    float       start_height;           /* Начальная высота (м) */
    float       step;                   /* Шаг по высоте (м) */
    float       tank_height;            /* Высота/диаметр резервуара (м) */
    float       tank_volume;            /* Полный объем резервуара (м³) */
    float       volumes[MAX_GRAD_POINTS]; /* Объемы в каждой точке (м³) */
} GradTable_t;

/* ==========================================================================
ПРОТОТИПЫ ФУНКЦИЙ: ОБЪЕМ
========================================================================== */
float Calc_Volume_Vertical(float level_m, float diameter_m, float tank_volume);
float Calc_Volume_HorizontalFlat(float level_m, float diameter_m, float length_m);
float Calc_Volume_HorizontalEllipse(float level_m, float diameter_m, float length_m);
float Calc_Volume_FromTable(float level_m);
float Calc_Volume(float level_m, TankType_t tank_type);

/* ==========================================================================
ПРОТОТИПЫ ФУНКЦИЙ: ГРАДУИРОВОЧНАЯ ТАБЛИЦА
========================================================================== */
void GradTable_Init(void);
void GradTable_Clear(void);
bool GradTable_SetPoint(uint16_t index, float volume_m3);
float GradTable_GetPoint(uint16_t index);
bool GradTable_IsValid(void);
uint16_t GradTable_GetPointsCount(void);
void GradTable_CalculateLinear(float start_volume, float end_volume);

/* ==========================================================================
ПРОТОТИПЫ ФУНКЦИЙ: ПЛОТНОСТЬ
========================================================================== */
float Calc_Density_Arbitrary(float density_orig, float temp_orig, float temp_current, float expansion_coef);
float Calc_Density_Petroleum(float density_orig, float temp_orig, float temp_current);
float Calc_Density_LPG(float propane, float butane, float isobutane, float temp);
float Calc_Density(float density_orig, float temp_orig, float temp_current, float expansion_coef, uint8_t medium_type);

/* ==========================================================================
ПРОТОТИПЫ ФУНКЦИЙ: МАССА
========================================================================== */
float Calc_Mass(float volume, float density);

/* ==========================================================================
ПРОТОТИПЫ ФУНКЦИЙ: СТАНДАРТНЫЕ УСЛОВИЯ
========================================================================== */
float Calc_Volume_Standard(float volume, float temp, float temp_standard);
float Calc_Density_Standard(float density, float temp, float temp_standard);

/* ==========================================================================
ПРОТОТИПЫ ФУНКЦИЙ: ТЕМПЕРАТУРА
========================================================================== */
void Calc_Temperature_Init(void);
float Calc_Temperature_GetLiquid(void);
float Calc_Temperature_GetVapor(void);
uint8_t Calc_Temperature_GetCount(void);
void Calc_Temperature_Update(uint8_t sensor_num, float height, float temp);

/* ==========================================================================
ПРОТОТИПЫ ФУНКЦИЙ: ДЕМПФИРОВАНИЕ
========================================================================== */
float Calc_Damping(float current_value, float prev_value, float dt, float time_constant);

/* ==========================================================================
ПРОТОТИПЫ ФУНКЦИЙ: ИНИЦИАЛИЗАЦИЯ
========================================================================== */
void Calc_All(void);

#ifdef __cplusplus
}
#endif

#endif /* __CALC_H */
