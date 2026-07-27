/**
 * @file    modbus.h
 * @brief   Modbus RTU и неизменяемая карта регистров уровнемера.
 *
 * Порядок float32: ABCD (старшее 16-битное слово первым).
 * Для MB_ADDR_TEMP = 1002:
 *   1002 = bits 31..16;
 *   1003 = bits 15..0.
 */
#ifndef MODBUS_H
#define MODBUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define MODBUS_DEFAULT_ADDRESS          1U
#define MODBUS_DEFAULT_BAUDRATE         19200U
#define MODBUS_BUFFER_SIZE              256U
#define MODBUS_FLOAT_WORD_COUNT         2U
#define MODBUS_INVALID_INDEX            0xFFFFU

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
/* Скорость распространения механической волны в материале волновода, м/с. */
#define MB_ADDR_MATERIAL_WAVE_SPEED 2094
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

/* === Регистры информации (адреса сохранены) === */
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

/* === Датчики температуры (адреса сохранены) === */
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

/* === Управление === */
#define MB_ADDR_COMMAND             3000
#define MB_ADDR_COMMAND_PARAM       3002

typedef enum {
    MODBUS_REGISTER_UNDEFINED = 0,
    MODBUS_REGISTER_UINT16,
    MODBUS_REGISTER_FLOAT32
} ModBus_RegisterType_t;

void ModBus_Init(void);
void ModBus_Process(void);
/* Вызывать в фоне после измерения и вывода; allow_write=false ставит EEPROM на паузу. */
void ModBus_StorageProcess(bool allow_write);
bool ModBus_StorageIsBusy(void);
void ModBus_RxCallback(UART_HandleTypeDef *huart);
void ModBus_RestartRx(void);

uint16_t ModBus_CRC16(const uint8_t *data, uint16_t length);
uint8_t ModBus_GetDeviceAddress(void);
/* true, если принимается/обрабатывается кадр или идет ответ RS-485. */
bool ModBus_CommunicationIsBusy(void);

float ModBus_GetParameter_Float(uint16_t address);
void ModBus_SetParameter_Float(uint16_t address, float value);
uint16_t ModBus_GetParameter_Int(uint16_t address);
void ModBus_SetParameter_Int(uint16_t address, uint16_t value);

bool ModBus_ReadRawRegister(uint16_t address, uint16_t *value);
ModBus_RegisterType_t ModBus_GetRegisterType(uint16_t address);
uint16_t ModBus_AddressToIndex_External(uint16_t address);

float ModBus_GetWaveguideLength(void);
void ModBus_SetWaveguideLength(float length_m);
float ModBus_GetMaterialWaveSpeed(void);
void ModBus_SetMaterialWaveSpeed(float speed_mps);
void ModBus_SetTemperature(float temperature);
float ModBus_GetTemperature(void);
uint32_t ModBus_GetPulseWidthIterations(void);

/**
 * Публикация готового измерительного снимка в RAM.
 * Вызывать сразу после расчёта уровня, до USART2/I2C/EEPROM.
 */
void ModBus_PublishLiveMeasurements(float level_mm,
                                    float temperature_c,
                                    float percent,
                                    float volume_m3,
                                    uint16_t status);

void ModBus_UpdateMeasurements(float level, float temperature, float waveguide);
void ModBus_UpdateVoltages(float vdda, float v24, float v12, float v5);
void ModBus_UpdateFirmwareVersion(uint16_t version);
void ModBus_ForceSaveToEEPROM(void);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_H */
