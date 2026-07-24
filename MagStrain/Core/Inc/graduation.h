/**
@file    graduation.h
@brief   Модуль работы с градуировочными таблицами резервуаров
*/
#ifndef GRADUATION_H
#define GRADUATION_H
#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define GRAD_MAX_POINTS             1500
#define GRAD_EEPROM_BASE            0x0400
#define GRAD_HEADER_SIZE            24
#define GRAD_DATA_START             (GRAD_EEPROM_BASE + GRAD_HEADER_SIZE)
#define GRAD_MAGIC                  0x47524144

typedef enum {
    GRAD_TYPE_VERTICAL      = 0,
    GRAD_TYPE_HORIZ_FLAT    = 1,
    GRAD_TYPE_BY_TABLE      = 2,
    GRAD_TYPE_HORIZ_ELLIPT  = 3
} GradTankType_t;

typedef struct {
    uint32_t magic;
    uint16_t points_count;
    uint16_t reserved;
    float    start_height_m;
    float    step_height_m;
    float    tank_height_m;
    float    tank_volume_m3;
} __attribute__((packed)) GradHeader_t;

typedef struct {
    GradHeader_t header;
    float        volumes[GRAD_MAX_POINTS];
    bool         loaded;
    bool         valid;
    uint16_t     actual_points;
    uint32_t     crc32;
} GradState_t;

void Grad_Init(void);
bool Grad_IsValid(void);
HAL_StatusTypeDef Grad_LoadFromEEPROM(void);
HAL_StatusTypeDef Grad_SaveToEEPROM(void);
HAL_StatusTypeDef Grad_Clear(void);
HAL_StatusTypeDef Grad_WriteHeader(const GradHeader_t *header);
HAL_StatusTypeDef Grad_ReadHeader(GradHeader_t *header);
HAL_StatusTypeDef Grad_WriteVolumes(uint16_t start_index,
    const float *volumes,
    uint16_t count);
HAL_StatusTypeDef Grad_ReadVolumes(uint16_t start_index,
    float *volumes,
    uint16_t count);
float Grad_InterpolateVolume(float level_m);
float Grad_CalculateVolume(GradTankType_t tank_type,
    float level_m,
    float height_m,
    float volume_m3);
GradState_t* Grad_GetState(void);
void Grad_UpdateModbusRegisters(void);
void Grad_ProcessCommand(uint16_t cmd, float param);

#ifdef __cplusplus
}
#endif
#endif
