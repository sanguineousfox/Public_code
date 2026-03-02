/* USER CODE BEGIN Header */
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : modbus.h
  * @brief          : Заголовочный файл Modbus RTU (ПМП-201Е)
  ******************************************************************************
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
 * НАСТРОЙКИ MODBUS
 * ========================================================================== */
#define MODBUS_DEFAULT_ADDRESS      1
#define MODBUS_BUFFER_SIZE          256
#define HOLD_HOLDING_REG_COUNT      64
#define REG_INPUT_REG_COUNT         64

/* ==========================================================================
 * КАРТА РЕГИСТРОВ
 * ========================================================================== */

/* --- ГРУППА 1: ВХОДНЫЕ РЕГИСТРЫ (INT16) (Адреса 1-37) --- */
#define MB_ADDR_LEVEL_INT       1
#define MB_ADDR_TEMP_INT        2
#define MB_ADDR_LEVEL_PCT_INT   3
#define MB_ADDR_VOLUME_INT      4
#define MB_ADDR_MASS_INT        5
#define MB_ADDR_DENSITY_INT     6
#define MB_ADDR_VOL_MAIN_INT    7
#define MB_ADDR_LEVEL_SEP_INT   8
#define MB_ADDR_TEMP_VAPOR_INT  10
#define MB_ADDR_MASS_VAPOR_INT  11
#define MB_ADDR_MASS_LIQ_INT    12
#define MB_ADDR_VOL_STD_INT     17
#define MB_ADDR_DENS_STD_INT    18
#define MB_ADDR_DENS_MEAS_INT   19
#define MB_ADDR_TEMP_DENS_INT   20
#define MB_ADDR_VOL_SEP_INT     21
#define MB_ADDR_ERROR_REL_INT   31
#define MB_ADDR_MB_ADDR_SET     35
#define MB_ADDR_MB_BAUD_SET     36
#define MB_ADDR_MB_PARITY_SET   37

/* --- ГРУППА 2: ИЗМЕРЯЕМЫЕ ПАРАМЕТРЫ (FLOAT32) (Адреса 1000-1040) --- */
#define MB_ADDR_LEVEL_FLT       1000
#define MB_ADDR_TEMP_FLT        1002
#define MB_ADDR_LEVEL_PCT_FLT   1004
#define MB_ADDR_VOLUME_FLT      1006
#define MB_ADDR_MASS_FLT        1008
#define MB_ADDR_DENSITY_FLT     1010
#define MB_ADDR_VOL_MAIN_FLT    1012
#define MB_ADDR_LEVEL_SEP_FLT   1014
#define MB_ADDR_TEMP_VAPOR_FLT  1018
#define MB_ADDR_MASS_VAPOR_FLT  1020
#define MB_ADDR_MASS_LIQ_FLT    1022
#define MB_ADDR_VOL_STD_FLT     1032
#define MB_ADDR_DENS_STD_FLT    1034
#define MB_ADDR_DENS_MEAS_FLT   1036
#define MB_ADDR_TEMP_DENS_FLT   1038
#define MB_ADDR_VOL_SEP_FLT     1040

/* --- ГРУППА 3: НАСТРОЙКИ И КАЛИБРОВКА (FLOAT32) (Адреса 2000-2096) --- */
#define MB_ADDR_CAL_LOW_LVL     2000
#define MB_ADDR_CAL_HIGH_LVL    2002
#define MB_ADDR_PROBE_DEPTH     2004
#define MB_ADDR_TANK_BOTTOM     2006
#define MB_ADDR_TANK_SHAPE      2008
#define MB_ADDR_TANK_HEIGHT     2010
#define MB_ADDR_TANK_VOL        2012
#define MB_ADDR_TABLE_POINTS    2014
#define MB_ADDR_CAL_LOW_DENS    2018
#define MB_ADDR_CAL_HIGH_DENS   2020
#define MB_ADDR_EXPANSION_COEF  2022
#define MB_ADDR_DENS_INIT       2024
#define MB_ADDR_TEMP_INIT_DENS  2026
#define MB_ADDR_PROBE_DEPTH_DENS 2028
#define MB_ADDR_MASS_FRACTION   2030
#define MB_ADDR_THRESH_LOW_DENS 2036
#define MB_ADDR_THRESH_SEP_LVL  2038
#define MB_ADDR_THRESH_LVL      2040
#define MB_ADDR_MAGNET_DIFF     2042
#define MB_ADDR_MASS_PROPANE    2050
#define MB_ADDR_THRESH_DENS_UP  2052
#define MB_ADDR_MASS_ISOBUTANE  2054
#define MB_ADDR_ERR_REL_MASS    2056
#define MB_ADDR_ERR_REL_TANK    2060
#define MB_ADDR_TEMP_STD        2062
#define MB_ADDR_BAUDRATE_FLT    2064
#define MB_ADDR_PARITY_FLT      2066
#define MB_ADDR_MB_ADDR_FLT     2068
#define MB_ADDR_DAMPING_TIME    2086
#define MB_ADDR_VOL_STD_15      2090
#define MB_ADDR_DENS_STD_15     2092
#define MB_ADDR_WAVEGUIDE_LEN   2096

/* --- ГРУППА 4: ДОП. НАСТРОЙКИ И ДАННЫЕ (Адреса 2098-2418) --- */
#define MB_ADDR_WAVEGUIDE_ERR   2098
#define MB_ADDR_LEVEL_OFFSET    2120
#define MB_ADDR_DENS_OFFSET     2148
#define MB_ADDR_MEDIUM_TYPE     2154
#define MB_ADDR_EXPANSION_COEF2 2156
#define MB_ADDR_UNIT_LEVEL      2160
#define MB_ADDR_UNIT_TEMP       2162
#define MB_ADDR_UNIT_VOL        2164
#define MB_ADDR_UNIT_MASS       2166
#define MB_ADDR_UNIT_DENS       2168
#define MB_ADDR_SERIAL_HIGH     2298
#define MB_ADDR_SERIAL_LOW      2300
#define MB_ADDR_DAMPING_DENS    2312
#define MB_ADDR_CAL_PARAM_C1    2386
#define MB_ADDR_CAL_PARAM_C2    2388
#define MB_ADDR_DIST_D1         2392
#define MB_ADDR_DIST_D5         2394
#define MB_ADDR_TEMP_SENS_COUNT 2414
#define MB_ADDR_ERROR_CODE      2416
#define MB_ADDR_MB_ADDR_LINE    2418

/* --- ГРУППА 5: ВЕРСИИ И ДАТЧИКИ (Адреса 2420-3000) --- */
#define MB_ADDR_FW_VERSION      2420
#define MB_ADDR_ADMIN_PASS      2426
#define MB_ADDR_ERR_DELAY       2438
#define MB_ADDR_TEMP_H1         2500
#define MB_ADDR_TEMP_H2         2502
#define MB_ADDR_TEMP_H3         2504
#define MB_ADDR_TEMP_H4         2506
#define MB_ADDR_TEMP_H5         2508
#define MB_ADDR_TEMP_H6         2510
#define MB_ADDR_TEMP_H7         2512
#define MB_ADDR_TEMP_H8         2514
#define MB_ADDR_TEMP_VAL_1      2600
#define MB_ADDR_TEMP_VAL_2      2602
#define MB_ADDR_TEMP_VAL_3      2604
#define MB_ADDR_TEMP_VAL_4      2606
#define MB_ADDR_TEMP_VAL_5      2608
#define MB_ADDR_TEMP_VAL_6      2610
#define MB_ADDR_TEMP_VAL_7      2612
#define MB_ADDR_TEMP_VAL_8      2614
#define MB_ADDR_DENS_VAL_1      2700
#define MB_ADDR_DENS_VAL_2      2702
#define MB_ADDR_DENS_VAL_3      2704
#define MB_ADDR_DENS_VAL_4      2706
#define MB_ADDR_DENS_VAL_5      2708
#define MB_ADDR_DENS_VAL_6      2710
#define MB_ADDR_DENS_VAL_7      2712
#define MB_ADDR_DENS_VAL_8      2714
#define MB_ADDR_COMMAND_REG     3000

/* --- ГРУППА 6: КОМАНДЫ И ТАБЛИЦЫ (Адреса 3002-38778) --- */
#define MB_ADDR_COMMAND_PARAM   3002
#define MB_ADDR_DISPLAY_REG_1   4000
#define MB_ADDR_DISPLAY_REG_125 4124
#define MB_ADDR_LINK_REG_1      5000
#define MB_ADDR_LINK_REG_125    5124
#define MB_ADDR_GRAD_POINTS     32768
#define MB_ADDR_GRAD_H_START    32770
#define MB_ADDR_GRAD_STEP_H     32772
#define MB_ADDR_GRAD_TANK_H     32774
#define MB_ADDR_GRAD_TANK_VOL   32776
#define MB_ADDR_GRAD_VOL_1      32778
#define MB_ADDR_GRAD_VOL_2      32780
#define MB_ADDR_GRAD_VOL_3001   38778



/* ==========================================================================
 * ПРОТОТИПЫ ФУНКЦИЙ
 * ========================================================================== */
void ModBus_Init(void);
void ModBus_Process(void);
void ModBus_RxCallback(UART_HandleTypeDef *huart);
void ModBus_UpdateVoltages(float vdda, float v24, float v12, float v5);
void ModBus_UpdateMeasurements(float level, float temp, float waveguide);
void ModBus_UpdateFirmwareVersion(uint16_t version);

extern void ModBus_TransmitFrame(uint8_t *frame, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __MODBUS_H */
