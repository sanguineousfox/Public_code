/* USER CODE BEGIN Header */
/**
  @file           : modbus.h
  @brief          : Заголовочный файл Modbus RTU
                   : Карта регистров с подробными комментариями
*/
/* USER CODE END Header */
#ifndef __MODBUS_H
#define __MODBUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

/* ==========================================================================
   НАСТРОЙКИ MODBUS
   ========================================================================== */
#define MODBUS_DEFAULT_ADDRESS      1           /* Адрес устройства по умолчанию */
#define MODBUS_BUFFER_SIZE          256         /* Размер буфера приема/передачи */
#define HOLD_HOLDING_REG_COUNT      64          /* Количество Holding регистров */
#define REG_INPUT_REG_COUNT         64          /* Количество Input регистров */

/* ==========================================================================
   КАРТА РЕГИСТРОВ
   ========================================================================== */
/* --- ГРУППА 1: ВХОДНЫЕ РЕГИСТРЫ (INT16) (Адреса 1-37) --- */
#define MB_ADDR_LEVEL_INT       1           /* Уровень жидкости (h), мм */
#define MB_ADDR_TEMP_INT        2           /* Температура жидкости (tº), 0.01 ºС */
#define MB_ADDR_LEVEL_PCT_INT   3           /* Процентное заполнение объёма резервуара (%), 0.01% */
#define MB_ADDR_VOLUME_INT      4           /* Объём жидкости (U), 10 дм³ */
#define MB_ADDR_MASS_INT        5           /* Масса продукта (G), 10 кг */
#define MB_ADDR_DENSITY_INT     6           /* Плотность (r), 0.1 кг/м³ */
#define MB_ADDR_VOL_MAIN_INT    7           /* Объём основного продукта (U1), 10 дм³ */
#define MB_ADDR_LEVEL_SEP_INT   8           /* Уровень раздела сред (h2), мм */
#define MB_ADDR_TEMP_VAPOR_INT  10          /* Температура паровой фазы СУГ (t¯), 0.01 ºС */
#define MB_ADDR_MASS_VAPOR_INT  11          /* Масса паровой фазы СУГ (G¯), 10 кг */
#define MB_ADDR_MASS_LIQ_INT    12          /* Масса жидкой фазы СУГ (G_), 10 кг */
#define MB_ADDR_VOL_STD_INT     17          /* Объём, приведённый к стандартным условиям (Ut), 10 дм³ */
#define MB_ADDR_DENS_STD_INT    18          /* Плотность, приведённая к стандартным условиям (rt), 0.1 кг/м³ */
#define MB_ADDR_DENS_MEAS_INT   19          /* Измеренная плотность (ri), 0.1 кг/м³ */
#define MB_ADDR_TEMP_DENS_INT   20          /* Температура при измерении плотности (tr), 0.01 ºС */
#define MB_ADDR_VOL_SEP_INT     21          /* Объём жидкости под разделом сред (U2), 10 дм³ */
#define MB_ADDR_ERROR_REL_INT   31          /* Относительная погрешность измерений массы (G), 0.01% */
#define MB_ADDR_MB_ADDR_SET     35          /* Адрес преобразователя в сети Modbus (AA) */
#define MB_ADDR_MB_BAUD_SET     36          /* Скорость передачи данных по интерфейсу RS-485 (rS) */
#define MB_ADDR_MB_PARITY_SET   37          /* Режим контроля чётности и количество стоповых бит (rP) */

/* --- ГРУППА 2: ИЗМЕРЯЕМЫЕ ПАРАМЕТРЫ (FLOAT32) (Адреса 1000-1040) --- */
#define MB_ADDR_LEVEL_FLT       1000        /* Уровень жидкости (h), единицы задаются в регистре Eh (2160) */
#define MB_ADDR_TEMP_FLT        1002        /* Температура жидкости (tº), единицы задаются в регистре Et (2162) */
#define MB_ADDR_LEVEL_PCT_FLT   1004        /* Процентное заполнение объёма резервуара (%), % */
#define MB_ADDR_VOLUME_FLT      1006        /* Объём жидкости (U), единицы задаются в регистре EU (2164) */
#define MB_ADDR_MASS_FLT        1008        /* Масса продукта (G), единицы задаются в регистре EG (2166) */
#define MB_ADDR_DENSITY_FLT     1010        /* Плотность (r), единицы задаются в регистре Er (2168) */
#define MB_ADDR_VOL_MAIN_FLT    1012        /* Объём основного продукта (U1), единицы задаются в регистре EU (2164) */
#define MB_ADDR_LEVEL_SEP_FLT   1014        /* Уровень раздела сред (h2), единицы задаются в регистре Eh (2160) */
#define MB_ADDR_TEMP_VAPOR_FLT  1018        /* Температура паровой фазы СУГ (t¯), единицы задаются в регистре Et (2162) */
#define MB_ADDR_MASS_VAPOR_FLT  1020        /* Масса паровой фазы СУГ (G¯), единицы задаются в регистре EG (2166) */
#define MB_ADDR_MASS_LIQ_FLT    1022        /* Масса жидкой фазы СУГ (G_), единицы задаются в регистре EG (2166) */
#define MB_ADDR_VOL_STD_FLT     1032        /* Объём, приведённый к стандартным условиям (Ut), единицы задаются в регистре EU (2164) */
#define MB_ADDR_DENS_STD_FLT    1034        /* Плотность, приведённая к стандартным условиям (rt), единицы задаются в регистре Er (2168) */
#define MB_ADDR_DENS_MEAS_FLT   1036        /* Измеренная плотность (ri), единицы задаются в регистре Er (2168) */
#define MB_ADDR_TEMP_DENS_FLT   1038        /* Температура при измерении плотности (tr), единицы задаются в регистре Et (2162) */
#define MB_ADDR_VOL_SEP_FLT     1040        /* Объём жидкости под разделом сред (U2), единицы задаются в регистре EU (2164) */

/* --- ГРУППА 3: НАСТРОЙКИ И КАЛИБРОВКА (FLOAT32) (Адреса 2000-2096) --- */
#define MB_ADDR_CAL_LOW_LVL     2000        /* Нижняя контрольная калибровочная точка уровня (h_), м */
#define MB_ADDR_CAL_HIGH_LVL    2002        /* Верхняя контрольная калибровочная точка уровня (h¯), м */
#define MB_ADDR_PROBE_DEPTH     2004        /* Глубина погружения поплавка уровня (d1), м */
#define MB_ADDR_TANK_BOTTOM     2006        /* Отступ от дна резервуара (d0), м */
#define MB_ADDR_TANK_SHAPE      2008        /* Способ расчёта объёма жидкости (Gr) */
#define MB_ADDR_TANK_HEIGHT     2010        /* Высота (диаметр) резервуара (H), м */
#define MB_ADDR_TANK_VOL        2012        /* Объём резервуара (U), м³ */
#define MB_ADDR_TABLE_POINTS    2014        /* Количество точек в градуировочной таблице, штук */
#define MB_ADDR_CAL_LOW_DENS    2018        /* Нижняя контрольная точка плотности (r_), кг/м³ */
#define MB_ADDR_CAL_HIGH_DENS   2020        /* Верхняя контрольная точка плотности (r¯), кг/м³ */
#define MB_ADDR_EXPANSION_COEF  2022        /* Коэффициент объёмного расширения (Lo), 10⁻³ /ºС */
#define MB_ADDR_DENS_INIT       2024        /* Исходная плотность (ro), кг/м³ */
#define MB_ADDR_TEMP_INIT_DENS  2026        /* Температура, соответствующая исходной плотности (to), ºС */
#define MB_ADDR_PROBE_DEPTH_DENS 2028       /* Глубина погружения поплавка раздела сред (d2), м */
#define MB_ADDR_MASS_FRACTION   2030        /* Массовая доля пропана (Pr), % */
#define MB_ADDR_THRESH_LOW_DENS 2036        /* Нижний порог корректного измерения плотности (d3) */
#define MB_ADDR_THRESH_SEP_LVL  2038        /* Порог обнуления показаний уровня раздела сред (d6), м */
#define MB_ADDR_THRESH_LVL      2040        /* Порог обнуления показаний уровня (d7), м */
#define MB_ADDR_MAGNET_DIFF     2042        /* Разность высот установки магнитов поплавков уровня и раздела сред (d8), м */
#define MB_ADDR_MASS_PROPANE    2050        /* Массовая доля бутана (Pb), % */
#define MB_ADDR_THRESH_DENS_UP  2052        /* Верхний порог корректного измерения плотности (d9) */
#define MB_ADDR_MASS_ISOBUTANE  2054        /* Массовая доля изобутана (Pi), % */
#define MB_ADDR_ERR_REL_MASS    2056        /* Относительная погрешность измерений массы (G), % */
#define MB_ADDR_ERR_REL_TANK    2060        /* Пределы допускаемой относительной погрешности измерений вместимости резервуара (δt), % */
#define MB_ADDR_TEMP_STD        2062        /* Температура стандартных условий (tS), ºС */
#define MB_ADDR_BAUDRATE_FLT    2064        /* Скорость передачи данных по интерфейсу RS-485 (rS) */
#define MB_ADDR_PARITY_FLT      2066        /* Режим контроля чётности и количество стоповых бит (rP) */
#define MB_ADDR_MB_ADDR_FLT     2068        /* Адрес преобразователя в сети Modbus (AA) */
#define MB_ADDR_DAMPING_TIME    2086        /* Постоянная времени демпфирования измерений уровня (dt), с */
#define MB_ADDR_POLL_PERIOD     2088        /* Период опроса, с (пользовательский параметр) */
#define MB_ADDR_VOL_STD_15      2090        /* Объём, приведённый к 15 ºС (UF), единицы задаются в регистре EU (2164) */
#define MB_ADDR_DENS_STD_15     2092        /* Плотность, приведённая к 15 ºС (rF), единицы задаются в регистре Er (2168) */
#define MB_ADDR_PULSE_WIDTH_US  2094        /* ★ Ширина задающего импульса, мкс (новый параметр) */
#define MB_ADDR_WAVEGUIDE_LEN   2096        /* Длина звукопровода текущая (Lc), м ★ КЛЮЧЕВОЙ ПАРАМЕТР */

/* --- ГРУППА 4: ДОП. НАСТРОЙКИ И ДАННЫЕ (Адреса 2098-2418) --- */
#define MB_ADDR_WAVEGUIDE_ERR   2098        /* Относительное отклонение длины звукопровода (д), % */
#define MB_ADDR_LEVEL_OFFSET    2120        /* Поправка измерений уровня (dh), м */
#define MB_ADDR_DENS_OFFSET     2148        /* Поправка измерений плотности (dr), кг/м³ */
#define MB_ADDR_MEDIUM_TYPE     2154        /* Контролируемая среда (cE): 0-произвольная, 1-нефтепродукты, 2-СУГ */
#define MB_ADDR_EXPANSION_COEF2 2156        /* Температурный коэффициент линейного расширения материала стенки резервуара (ct), 10⁻⁶ 1/ºС */
#define MB_ADDR_UNIT_LEVEL      2160        /* Единицы измерения при отображении уровня (Eh): 8-мм, 9-м */
#define MB_ADDR_UNIT_TEMP       2162        /* Единицы измерения при отображении температуры (Et): 24-ºС */
#define MB_ADDR_UNIT_VOL        2164        /* Единицы измерения при отображении объёма (EU): 39-дал, 40-л, 41-м³ */
#define MB_ADDR_UNIT_MASS       2166        /* Единицы измерения при отображении массы (EG): 55-т, 56-кг */
#define MB_ADDR_UNIT_DENS       2168        /* Единицы измерения при отображении плотности (Er): 72-г/см³, 73-кг/м³, 74-т/м³ */
#define MB_ADDR_SERIAL_HIGH     2298        /* Старшие разряды заводского номера (S1) */
#define MB_ADDR_SERIAL_LOW      2300        /* Младшие разряды заводского номера (S2) */
#define MB_ADDR_DAMPING_DENS    2312        /* Постоянная времени демпфирования измерений плотности (dd), с */
#define MB_ADDR_CAL_PARAM_C1    2386        /* Калибровочный параметр, соответствующий h_ (C1) */
#define MB_ADDR_CAL_PARAM_C2    2388        /* Калибровочный параметр, соответствующий h¯ (C2) */
#define MB_ADDR_DIST_D1         2392        /* Контрольное расстояние, соответствующее r_ (d4) */
#define MB_ADDR_DIST_D5         2394        /* Контрольное расстояние, соответствующее r¯ (d5) */
#define MB_ADDR_TEMP_SENS_COUNT 2414        /* Количество датчиков температуры, штук */
#define MB_ADDR_ERROR_CODE      2416        /* Код ошибки преобразователя (Er) */
#define MB_ADDR_MB_ADDR_LINE    2418        /* Адрес преобразователя в линии СЕНС (Ad) */

/* --- ГРУППА 5: ВЕРСИИ И ДАТЧИКИ (Адреса 2420-3000) --- */
#define MB_ADDR_FW_VERSION      2420        /* Версия программы контроллера преобразователя (Pn) */
#define MB_ADDR_ADMIN_PASS      2426        /* Пароль администратора (P1) */
#define MB_ADDR_ERR_DELAY       2438        /* Время задержки реакции на ошибку (F), с */
#define MB_ADDR_TEMP_H1         2500        /* Высота установки 1-ого датчика температуры (1.ht), м */
#define MB_ADDR_TEMP_H2         2502        /* Высота установки 2-ого датчика температуры (2.ht), м */
#define MB_ADDR_TEMP_H3         2504        /* Высота установки 3-ого датчика температуры (3.ht), м */
#define MB_ADDR_TEMP_H4         2506        /* Высота установки 4-ого датчика температуры (4.ht), м */
#define MB_ADDR_TEMP_H5         2508        /* Высота установки 5-ого датчика температуры (5.ht), м */
#define MB_ADDR_TEMP_H6         2510        /* Высота установки 6-ого датчика температуры (6.ht), м */
#define MB_ADDR_TEMP_H7         2512        /* Высота установки 7-ого датчика температуры (7.ht), м */
#define MB_ADDR_TEMP_H8         2514        /* Высота установки 8-ого датчика температуры (8.ht), м */
#define MB_ADDR_TEMP_VAL_1      2600        /* Температура 1-ого датчика температуры (1.ºС), ºС */
#define MB_ADDR_TEMP_VAL_2      2602        /* Температура 2-ого датчика температуры (2.ºС), ºС */
#define MB_ADDR_TEMP_VAL_3      2604        /* Температура 3-ого датчика температуры (3.ºС), ºС */
#define MB_ADDR_TEMP_VAL_4      2606        /* Температура 4-ого датчика температуры (4.ºС), ºС */
#define MB_ADDR_TEMP_VAL_5      2608        /* Температура 5-ого датчика температуры (5.ºС), ºС */
#define MB_ADDR_TEMP_VAL_6      2610        /* Температура 6-ого датчика температуры (6.ºС), ºС */
#define MB_ADDR_TEMP_VAL_7      2612        /* Температура 7-ого датчика температуры (7.ºС), ºС */
#define MB_ADDR_TEMP_VAL_8      2614        /* Температура 8-ого датчика температуры (8.ºС), ºС */
#define MB_ADDR_DENS_VAL_1      2700        /* Плотность для 1-ого датчика температуры (1.r), единицы задаются в регистре Er (2168) */
#define MB_ADDR_DENS_VAL_2      2702        /* Плотность для 2-ого датчика температуры (2.r), единицы задаются в регистре Er (2168) */
#define MB_ADDR_DENS_VAL_3      2704        /* Плотность для 3-ого датчика температуры (3.r), единицы задаются в регистре Er (2168) */
#define MB_ADDR_DENS_VAL_4      2706        /* Плотность для 4-ого датчика температуры (4.r), единицы задаются в регистре Er (2168) */
#define MB_ADDR_DENS_VAL_5      2708        /* Плотность для 5-ого датчика температуры (5.r), единицы задаются в регистре Er (2168) */
#define MB_ADDR_DENS_VAL_6      2710        /* Плотность для 6-ого датчика температуры (6.r), единицы задаются в регистре Er (2168) */
#define MB_ADDR_DENS_VAL_7      2712        /* Плотность для 7-ого датчика температуры (7.r), единицы задаются в регистре Er (2168) */
#define MB_ADDR_DENS_VAL_8      2714        /* Плотность для 8-ого датчика температуры (8.r), единицы задаются в регистре Er (2168) */
#define MB_ADDR_COMMAND_REG     3000        /* Ввод команд управления. Запись значения N запускает выполнение команды с номером N */

/* --- ГРУППА 6: КОМАНДЫ И ТАБЛИЦЫ (Адреса 3002-38778) --- */
#define MB_ADDR_COMMAND_PARAM   3002        /* Параметр команды управления */
#define MB_ADDR_DISPLAY_REG_1   4000        /* Регистр отображения №1 */
#define MB_ADDR_DISPLAY_REG_125 4124        /* Регистр отображения №125 */
#define MB_ADDR_LINK_REG_1      5000        /* Адрес для регистра отображения №1 */
#define MB_ADDR_LINK_REG_125    5124        /* Адрес для регистра отображения №125 */
#define MB_ADDR_GRAD_POINTS     32768       /* Количество точек в градуировочной таблице, штук */
#define MB_ADDR_GRAD_H_START    32770       /* Начальная высота градуировки, м */
#define MB_ADDR_GRAD_STEP_H     32772       /* Шаг градуировки по уровню, м */
#define MB_ADDR_GRAD_TANK_H     32774       /* Высота (диаметр) резервуара, м */
#define MB_ADDR_GRAD_TANK_VOL   32776       /* Объём резервуара, м³ */
#define MB_ADDR_GRAD_VOL_1      32778       /* Объём в 1-й точке градуировки, м³ */
#define MB_ADDR_GRAD_VOL_2      32780       /* Объём во 2-й точке градуировки, м³ */
#define MB_ADDR_GRAD_VOL_3001   38778       /* Объём в 3001-й точке градуировки, м³ */

/* ==========================================================================
   ПРОТОТИПЫ ФУНКЦИЙ
   ========================================================================== */
void ModBus_Init(void);
void ModBus_Process(void);
void ModBus_RxCallback(UART_HandleTypeDef *huart);
void ModBus_UpdateVoltages(float vdda, float v24, float v12, float v5);
void ModBus_UpdateMeasurements(float level, float temp, float waveguide);
void ModBus_UpdateFirmwareVersion(uint16_t version);

/* === ФУНКЦИИ ДЛЯ РАБОТЫ С ПАРАМЕТРАМИ === */
float ModBus_GetWaveguideLength(void);
void ModBus_SetWaveguideLength(float length_mm);
float ModBus_GetParameter_Float(uint16_t addr);
void ModBus_SetParameter_Float(uint16_t addr, float value);
uint16_t ModBus_GetParameter_Int(uint16_t addr);
void ModBus_SetParameter_Int(uint16_t addr, uint16_t value);

/* === НОВАЯ ФУНКЦИЯ: Получение ширины импульса === */
uint32_t ModBus_GetPulseWidthIterations(void);

extern void ModBus_TransmitFrame(uint8_t *frame, uint16_t len);

#ifdef __cplusplus
}
#endif
#endif /* __MODBUS_H */
