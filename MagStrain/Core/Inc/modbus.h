/* USER CODE BEGIN Header */
/*
@file           : modbus.h
@brief          : Заголовочный файл Modbus RTU для магнитострикционного уровнемера
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
#define MODBUS_DEFAULT_ADDRESS      1
#define MODBUS_BUFFER_SIZE          256

/* ==========================================================================
КАРТА РЕГИСТРОВ (полная, согласно таблице Е.4)
========================================================================== */

/* === Регистры int16 (адреса 1-37) === */
#define MB_ADDR_LEVEL_INT           1
#define MB_ADDR_TEMP_INT            2
#define MB_ADDR_PERCENT_INT         3
#define MB_ADDR_VOLUME_INT          4
#define MB_ADDR_MASS_INT            5
#define MB_ADDR_DENSITY_INT         6
#define MB_ADDR_VOLUME_MAIN_INT     7
#define MB_ADDR_LEVEL_INT_SEP       8
#define MB_ADDR_TEMP_VAPOR_INT      10
#define MB_ADDR_MASS_VAPOR_INT      11
#define MB_ADDR_MASS_LIQ_INT        12
#define MB_ADDR_VOLUME_STD_INT      17
#define MB_ADDR_DENSITY_STD_INT     18
#define MB_ADDR_DENSITY_MEAS_INT    19
#define MB_ADDR_TEMP_DENS_INT       20
#define MB_ADDR_VOLUME_SEP_INT      21
#define MB_ADDR_MASS_ERROR_INT      31
#define MB_ADDR_MB_ADDR_SET         35
#define MB_ADDR_MB_BAUD_SET         36
#define MB_ADDR_MB_PARITY_SET       37

/* === Регистры float32 (адреса 1000-1040) === */
#define MB_ADDR_LEVEL               1000
#define MB_ADDR_TEMP                1002
#define MB_ADDR_PERCENT             1004
#define MB_ADDR_VOLUME              1006
#define MB_ADDR_MASS                1008
#define MB_ADDR_DENSITY             1010
#define MB_ADDR_VOLUME_MAIN         1012
#define MB_ADDR_LEVEL_SEP           1014
#define MB_ADDR_TEMP_VAPOR          1018
#define MB_ADDR_MASS_VAPOR          1020
#define MB_ADDR_MASS_LIQ            1022
#define MB_ADDR_VOLUME_STD          1032
#define MB_ADDR_DENSITY_STD         1034
#define MB_ADDR_DENSITY_MEAS        1036
#define MB_ADDR_TEMP_DENS           1038
#define MB_ADDR_VOLUME_SEP          1040

/* === Регистры float32 настроечные (адреса 2000-2168) === */
#define MB_ADDR_CAL_LOW_LVL         2000
#define MB_ADDR_CAL_HIGH_LVL        2002
#define MB_ADDR_PROBE_DEPTH         2004
#define MB_ADDR_LEVEL_OFFSET        2006
#define MB_ADDR_TANK_GEOM           2008
#define MB_ADDR_TANK_HEIGHT         2010
#define MB_ADDR_TANK_VOLUME         2012
#define MB_ADDR_CALIB_POINTS        2014
#define MB_ADDR_DENSITY_LOW         2018
#define MB_ADDR_DENSITY_HIGH        2020
#define MB_ADDR_EXPANSION_COEF      2022
#define MB_ADDR_DENSITY_ORIG        2024
#define MB_ADDR_TEMP_ORIG           2026
#define MB_ADDR_SEP_DEPTH           2028
#define MB_ADDR_PROPANE_RATIO       2030
#define MB_ADDR_DENSITY_MIN         2036
#define MB_ADDR_LEVEL_ZERO_SEP      2038
#define MB_ADDR_THRESH_LVL          2040
#define MB_ADDR_MAGNET_DIFF         2042
#define MB_ADDR_BUTANE_RATIO        2050
#define MB_ADDR_DENSITY_MAX         2052
#define MB_ADDR_ISOBUTANE_RATIO     2054
#define MB_ADDR_MASS_REL_ERROR      2056
#define MB_ADDR_TANK_ERROR          2060
#define MB_ADDR_TEMP_STD            2062
#define MB_ADDR_BAUD_RATE           2064
#define MB_ADDR_PARITY              2066
#define MB_ADDR_DEVICE_ADDR         2068
#define MB_ADDR_DAMPING_TIME        2086
#define MB_ADDR_POLL_PERIOD         2088
#define MB_ADDR_VOLUME_15C          2090
#define MB_ADDR_DENSITY_15C         2092
#define MB_ADDR_SOUND_SPEED         2094    /* Скорость звука в волноводе, м/с */
#define MB_ADDR_WAVEGUIDE_LEN       2096
#define MB_ADDR_WAVEGUIDE_DEV       2098
#define MB_ADDR_LEVEL_CORR          2120
#define MB_ADDR_DENSITY_CORR        2148
#define MB_ADDR_MEDIUM_TYPE         2154
#define MB_ADDR_TANK_EXPANSION      2156
#define MB_ADDR_UNIT_LEVEL          2160
#define MB_ADDR_UNIT_TEMP           2162
#define MB_ADDR_UNIT_VOLUME         2164
#define MB_ADDR_UNIT_MASS           2166
#define MB_ADDR_UNIT_DENSITY        2168

/* === Регистры информации (адреса 2298-2438) === */
#define MB_ADDR_SERIAL_HI           2298
#define MB_ADDR_SERIAL_LO           2300
#define MB_ADDR_DAMPING_DENS        2312
#define MB_ADDR_CAL_C1              2386
#define MB_ADDR_CAL_C2              2388
#define MB_ADDR_CAL_D4              2392
#define MB_ADDR_CAL_D5              2394
#define MB_ADDR_TEMP_SENS_COUNT     2414
#define MB_ADDR_ERROR_CODE          2416
#define MB_ADDR_SENS_ADDR           2418
#define MB_ADDR_FW_VERSION          2420
#define MB_ADDR_ADMIN_PASS          2426
#define MB_ADDR_ERROR_DELAY         2438

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
#define MB_ADDR_COMMAND             3000
#define MB_ADDR_COMMAND_PARAM       3002

/* === Регистры отображения (адреса 4000-4124 и таблица 5000-5124) === */
#define MB_ADDR_DISPLAY_BASE        4000
#define MB_ADDR_DISPLAY_COUNT       125
#define MB_ADDR_DISPLAY_TABLE_BASE  5000

/* === Градуировочная таблица (адреса 32768-38778) === */
#define MB_ADDR_GRAD_POINTS         32768
#define MB_ADDR_GRAD_START          32770
#define MB_ADDR_GRAD_STEP           32772
#define MB_ADDR_GRAD_TANK_H         32774
#define MB_ADDR_GRAD_TANK_V         32776
#define MB_ADDR_GRAD_V1             32778

/* ==========================================================================
КОДЫ КОМАНД УПРАВЛЕНИЯ (регистр 3000)
========================================================================== */
#define CMD_CAL_LOW_LVL             1
#define CMD_CAL_HIGH_LVL            2
#define CMD_WAVEGUIDE_LEN           3
#define CMD_MAGNET_DIFF             4
#define CMD_CAL_LOW_DENS            5
#define CMD_CAL_HIGH_DENS           6
#define CMD_EMUL_OFF                200
#define CMD_EMUL_ON                 201
#define CMD_CORR_OFF                210
#define CMD_CORR_ON                 211
#define CMD_RESTORE_USER            222
#define CMD_SAVE_USER               223
#define CMD_RESTORE_FACTORY         224
#define CMD_ACCESS_USER             230
#define CMD_ACCESS_ADMIN            231

/* ==========================================================================
РЕЗУЛЬТАТЫ ВЫПОЛНЕНИЯ КОМАНД
========================================================================== */
#define CMD_RESULT_REFUSE           0
#define CMD_RESULT_RUNNING          85
#define CMD_RESULT_DONE             90
#define CMD_RESULT_NONE             99

/* ==========================================================================
ТИПЫ СРЕДЫ (cE)
========================================================================== */
#define MEDIUM_ARBITRARY            0
#define MEDIUM_PETROLEUM            1
#define MEDIUM_LPG                  2


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
float ModBus_GetSoundSpeed(void);     /* Получение скорости звука из регистра 2094 */
float ModBus_GetParameter_Float(uint16_t addr);
void ModBus_SetParameter_Float(uint16_t addr, float value);
uint16_t ModBus_GetParameter_Int(uint16_t addr);
void ModBus_SetParameter_Int(uint16_t addr, uint16_t value);
uint32_t ModBus_GetPulseWidthIterations(void);
extern void ModBus_TransmitFrame(uint8_t *frame, uint16_t len);

/* === ЗАГОТОВКИ: РЕЖИМЫ === */
void Emulation_Enter(void);
void Emulation_Exit(void);
bool Emulation_IsActive(void);

/* === ЗАГОТОВКИ: ДОСТУП === */
bool Access_IsAdmin(void);
bool Access_IsBlocked(void);
void Access_SetAdmin(bool state);

/* === ЗАГОТОВКИ: ОШИБКИ === */
void Error_SetCode(uint16_t code);
uint16_t Error_GetCode(void);

/* === ЗАГОТОВКИ: НАСТРОЙКИ СВЯЗИ === */
void Comm_ApplyBaudRate(uint16_t baud_code);
void Comm_ApplyParity(uint16_t parity_code);
void Comm_ApplyAddress(uint16_t addr);

/* === ПУБЛИЧНЫЙ API ЗАПИСИ В INPUT-РЕГИСТРЫ (для calc.c) === */
void ModBus_WriteInputFloat(uint16_t addr, float value);
void ModBus_WriteInputInt16(uint16_t addr, int16_t value);

#ifdef __cplusplus
}
#endif

#endif /* __MODBUS_H */
