/**
 * @file           : modbus.h
 * @brief          : Заголовочный файл Modbus RTU для
 */
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
#define MODBUS_DEFAULT_ADDRESS      1
#define MODBUS_BUFFER_SIZE          256

/* ==========================================================================
   КАРТА РЕГИСТРОВ (согласно таблице Е.4 документации  )
   ========================================================================== */

/* === Регистры int16 (адреса 1-37) === */
#define MB_ADDR_LEVEL_INT           1       /* Уровень жидкости (h) */
#define MB_ADDR_TEMP_INT            2       /* Температура (tº) */
#define MB_ADDR_PERCENT_INT         3       /* Процент заполнения (%) */
#define MB_ADDR_VOLUME_INT          4       /* Объем (U) */
#define MB_ADDR_MASS_INT            5       /* Масса (G) */
#define MB_ADDR_DENSITY_INT         6       /* Плотность (r) */
#define MB_ADDR_VOLUME_MAIN_INT     7       /* Объем основного продукта (U1) */
#define MB_ADDR_LEVEL_INT_SEP       8       /* Уровень раздела сред (h2) */
#define MB_ADDR_TEMP_VAPOR_INT      10      /* Температура паровой фазы (t¯) */
#define MB_ADDR_MASS_VAPOR_INT      11      /* Масса паровой фазы (G¯) */
#define MB_ADDR_MASS_LIQ_INT        12      /* Масса жидкой фазы (G_) */
#define MB_ADDR_VOLUME_STD_INT      17      /* Объем к станд. условиям (Ut) */
#define MB_ADDR_DENSITY_STD_INT     18      /* Плотность к станд. условиям (rt) */
#define MB_ADDR_DENSITY_MEAS_INT    19      /* Измеренная плотность (ri) */
#define MB_ADDR_TEMP_DENS_INT       20      /* Температура при измерении плотности (tr) */
#define MB_ADDR_VOLUME_SEP_INT      21      /* Объем под разделом сред (U2) */
#define MB_ADDR_MASS_ERROR_INT      31      /* Относит. погрешность массы (G) */
#define MB_ADDR_MB_ADDR_SET         35      /* Адрес в сети Modbus (AA) */
#define MB_ADDR_MB_BAUD_SET         36      /* Скорость передачи (rS) */
#define MB_ADDR_MB_PARITY_SET       37      /* Режим четности (rP) */

/* === Регистры float32 (адреса 1000-1040) === */
#define MB_ADDR_LEVEL               1000    /* Уровень жидкости (h) */
#define MB_ADDR_TEMP                1002    /* Температура (tº) */
#define MB_ADDR_PERCENT             1004    /* Процент заполнения (%) */
#define MB_ADDR_VOLUME              1006    /* Объем (U) */
#define MB_ADDR_MASS                1008    /* Масса (G) */
#define MB_ADDR_DENSITY             1010    /* Плотность (r) */
#define MB_ADDR_VOLUME_MAIN         1012    /* Объем основного продукта (U1) */
#define MB_ADDR_LEVEL_SEP           1014    /* Уровень раздела сред (h2) */
#define MB_ADDR_TEMP_VAPOR          1018    /* Температура паровой фазы (t¯) */
#define MB_ADDR_MASS_VAPOR          1020    /* Масса паровой фазы (G¯) */
#define MB_ADDR_MASS_LIQ            1022    /* Масса жидкой фазы (G_) */
#define MB_ADDR_VOLUME_STD          1032    /* Объем к станд. условиям (Ut) */
#define MB_ADDR_DENSITY_STD         1034    /* Плотность к станд. условиям (rt) */
#define MB_ADDR_DENSITY_MEAS        1036    /* Измеренная плотность (ri) */
#define MB_ADDR_TEMP_DENS           1038    /* Температура при измерении плотности (tr) */
#define MB_ADDR_VOLUME_SEP          1040    /* Объем под разделом сред (U2) */

/* === Регистры float32 настроечные (адреса 2000-2168) === */
#define MB_ADDR_CAL_LOW_LVL         2000    /* Нижняя контрольная точка уровня (h_) */
#define MB_ADDR_CAL_HIGH_LVL        2002    /* Верхняя контрольная точка уровня (h¯) */
#define MB_ADDR_PROBE_DEPTH         2004    /* Глубина погружения поплавка (d1) */
#define MB_ADDR_LEVEL_OFFSET        2006    /* Отступ от дна резервуара (d0) */
#define MB_ADDR_TANK_GEOM           2008    /* Способ расчета объема (Gr) */
#define MB_ADDR_TANK_HEIGHT         2010    /* Высота/диаметр резервуара (H) */
#define MB_ADDR_TANK_VOLUME         2012    /* Объем резервуара (U) */
#define MB_ADDR_CALIB_POINTS        2014    /* Количество точек градуировки */
#define MB_ADDR_DENSITY_LOW         2018    /* Нижняя контрольная точка плотности (r_) */
#define MB_ADDR_DENSITY_HIGH        2020    /* Верхняя контрольная точка плотности (r¯) */
#define MB_ADDR_EXPANSION_COEF      2022    /* Коэфф. объемного расширения (Lo) */
#define MB_ADDR_DENSITY_ORIG        2024    /* Исходная плотность (ro) */
#define MB_ADDR_TEMP_ORIG           2026    /* Температура исходной плотности (to) */
#define MB_ADDR_SEP_DEPTH           2028    /* Глубина погружения поплавка раздела сред (d2) */
#define MB_ADDR_PROPANE_RATIO       2030    /* Массовая доля пропана (Pr) */
#define MB_ADDR_DENSITY_MIN         2036    /* Нижний порог измерения плотности (d3) */
#define MB_ADDR_LEVEL_ZERO_SEP      2038    /* Порог обнуления уровня раздела сред (d6) */
#define MB_ADDR_THRESH_LVL          2040    /* Порог обнуления уровня (d7) */
#define MB_ADDR_MAGNET_DIFF         2042    /* Разность высот магнитов (d8) */
#define MB_ADDR_BUTANE_RATIO        2050    /* Массовая доля бутана (Pb) */
#define MB_ADDR_DENSITY_MAX         2052    /* Верхний порог измерения плотности (d9) */
#define MB_ADDR_ISOBUTANE_RATIO     2054    /* Массовая доля изобутана (Pi) */
#define MB_ADDR_MASS_REL_ERROR      2056    /* Относит. погрешность массы (G) */
#define MB_ADDR_TANK_ERROR          2060    /* Относит. погрешность градуировки (t) */
#define MB_ADDR_TEMP_STD            2062    /* Температура станд. условий (tS) */
#define MB_ADDR_BAUD_RATE           2064    /* Скорость передачи (rS) */
#define MB_ADDR_PARITY              2066    /* Режим четности (rP) */
#define MB_ADDR_DEVICE_ADDR         2068    /* Адрес устройства (AA) */
#define MB_ADDR_DAMPING_TIME        2086    /* Постоянная времени демпфирования уровня (dt) */
#define MB_ADDR_POLL_PERIOD         2088    /* Период опроса (мс), int16 */
#define MB_ADDR_VOLUME_15C          2090    /* Объем, приведенный к 15°C (UF) */
#define MB_ADDR_DENSITY_15C         2092    /* Плотность, приведенная к 15°C (rF) */
#define MB_ADDR_WAVEGUIDE_LEN       2096    /* Длина звукопровода (Lc) */
#define MB_ADDR_WAVEGUIDE_DEV       2098    /* Отклонение длины звукопровода (д) */
#define MB_ADDR_LEVEL_CORR          2120    /* Поправка измерений уровня (dh) */
#define MB_ADDR_DENSITY_CORR        2148    /* Поправка измерений плотности (dr) */
#define MB_ADDR_MEDIUM_TYPE         2154    /* Контролируемая среда (cE) */
#define MB_ADDR_TANK_EXPANSION      2156    /* Темп. коэфф. расширения материала (ct) */
#define MB_ADDR_UNIT_LEVEL          2160    /* Единицы измерения уровня (Eh) */
#define MB_ADDR_UNIT_TEMP           2162    /* Единицы измерения температуры (Et) */
#define MB_ADDR_UNIT_VOLUME         2164    /* Единицы измерения объема (EU) */
#define MB_ADDR_UNIT_MASS           2166    /* Единицы измерения массы (EG) */
#define MB_ADDR_UNIT_DENSITY        2168    /* Единицы измерения плотности (Er) */

/* === Регистры информации (адреса 2298-2438) === */
#define MB_ADDR_SERIAL_HI           2298    /* Старшие разряды заводского номера (S1) */
#define MB_ADDR_SERIAL_LO           2300    /* Младшие разряды заводского номера (S2) */
#define MB_ADDR_DAMPING_DENS        2312    /* Постоянная времени демпфирования плотности (dd) */
#define MB_ADDR_CAL_C1              2386    /* Калибровочный параметр C1 */
#define MB_ADDR_CAL_C2              2388    /* Калибровочный параметр C2 */
#define MB_ADDR_CAL_D4              2392    /* Контрольное расстояние d4 */
#define MB_ADDR_CAL_D5              2394    /* Контрольное расстояние d5 */
#define MB_ADDR_TEMP_SENS_COUNT     2414    /* Количество датчиков температуры */
#define MB_ADDR_ERROR_CODE          2416    /* Код ошибки (Er) */
#define MB_ADDR_SENS_ADDR           2418    /* Адрес в линии СЕНС (Ad) */
#define MB_ADDR_FW_VERSION          2420    /* Версия программы (Pn) */
#define MB_ADDR_ADMIN_PASS          2426    /* Пароль администратора (P1) */
#define MB_ADDR_ERROR_DELAY         2438    /* Время задержки реакции на ошибку (F) */

/* === Регистры датчиков температуры (адреса 2500-2714) === */
#define MB_ADDR_TEMP_SENS_1_H       2500
#define MB_ADDR_TEMP_SENS_2_H       2502
#define MB_ADDR_TEMP_SENS_3_H       2504
#define MB_ADDR_TEMP_SENS_4_H       2506
#define MB_ADDR_TEMP_SENS_5_H       2508
#define MB_ADDR_TEMP_SENS_6_H       2510
#define MB_ADDR_TEMP_SENS_7_H       2512
#define MB_ADDR_TEMP_SENS_8_H       2514
#define MB_ADDR_TEMP_SENS_1_V       2600
#define MB_ADDR_TEMP_SENS_2_V       2602
#define MB_ADDR_TEMP_SENS_3_V       2604
#define MB_ADDR_TEMP_SENS_4_V       2606
#define MB_ADDR_TEMP_SENS_5_V       2608
#define MB_ADDR_TEMP_SENS_6_V       2610
#define MB_ADDR_TEMP_SENS_7_V       2612
#define MB_ADDR_TEMP_SENS_8_V       2614
#define MB_ADDR_DENS_SENS_1         2700
#define MB_ADDR_DENS_SENS_2         2702
#define MB_ADDR_DENS_SENS_3         2704
#define MB_ADDR_DENS_SENS_4         2706
#define MB_ADDR_DENS_SENS_5         2708
#define MB_ADDR_DENS_SENS_6         2710
#define MB_ADDR_DENS_SENS_7         2712
#define MB_ADDR_DENS_SENS_8         2714

/* === Регистры управления (адреса 3000-3002) === */
#define MB_ADDR_COMMAND             3000    /* Ввод команд управления */
#define MB_ADDR_COMMAND_PARAM       3002    /* Параметр команды управления */

/* ==========================================================================
   ПРОТОТИПЫ ФУНКЦИЙ
   ========================================================================== */
void ModBus_Init(void);
void ModBus_Process(void);
void ModBus_RxCallback(UART_HandleTypeDef *huart);
void ModBus_UpdateVoltages(float vdda, float v24, float v12, float v5);
void ModBus_UpdateMeasurements(float level, float temp, float waveguide);
void ModBus_UpdateFirmwareVersion(uint16_t version);
void ModBus_ForceSaveToEEPROM(void);
void ModBus_RestartRx(void);

/* === ФУНКЦИИ ДЛЯ РАБОТЫ С ПАРАМЕТРАМИ === */
float ModBus_GetWaveguideLength(void);
void ModBus_SetWaveguideLength(float length_m);
float ModBus_GetParameter_Float(uint16_t addr);
void ModBus_SetParameter_Float(uint16_t addr, float value);
uint16_t ModBus_GetParameter_Int(uint16_t addr);
void ModBus_SetParameter_Int(uint16_t addr, uint16_t value);
uint32_t ModBus_GetPulseWidthIterations(void);

/* === ЭКСПОРТ ДЛЯ ВНЕШНИХ МОДУЛЕЙ (temp_sensors, graduation) === */
uint16_t ModBus_AddressToIndex_External(uint16_t addr);

extern void ModBus_TransmitFrame(uint8_t *frame, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __MODBUS_H */
