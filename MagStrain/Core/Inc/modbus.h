/* USER CODE BEGIN Header */
/*
 * @file           : modbus.h
 * @brief          : Заголовочный файл Modbus RTU
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
КАРТА РЕГИСТРОВ
========================================================================== */
#define MB_ADDR_WAVEGUIDE_LEN       2096
#define MB_ADDR_CAL_LOW_LVL         2000
#define MB_ADDR_CAL_HIGH_LVL        2002
#define MB_ADDR_PROBE_DEPTH         2004
#define MB_ADDR_TANK_HEIGHT         2010
#define MB_ADDR_DAMPING_TIME        2086
#define MB_ADDR_POLL_PERIOD         2088
#define MB_ADDR_LEVEL_OFFSET        2120
#define MB_ADDR_THRESH_LVL          2040
#define MB_ADDR_MB_ADDR_SET         35
#define MB_ADDR_MB_BAUD_SET         36
#define MB_ADDR_MEDIUM_TYPE         2154
#define MB_ADDR_UNIT_LEVEL          2160
#define MB_ADDR_UNIT_TEMP           2162
#define MB_ADDR_FW_VERSION          2420

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
void ModBus_RestartRx(void);  /* Для обработки ошибок UART */

/* === ФУНКЦИИ ДЛЯ РАБОТЫ С ПАРАМЕТРАМИ === */
float ModBus_GetWaveguideLength(void);
void ModBus_SetWaveguideLength(float length_mm);
float ModBus_GetParameter_Float(uint16_t addr);
void ModBus_SetParameter_Float(uint16_t addr, float value);
uint16_t ModBus_GetParameter_Int(uint16_t addr);
void ModBus_SetParameter_Int(uint16_t addr, uint16_t value);
uint32_t ModBus_GetPulseWidthIterations(void);

extern void ModBus_TransmitFrame(uint8_t *frame, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __MODBUS_H */
