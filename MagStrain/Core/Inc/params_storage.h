/**
 * @file    params_storage.h
 * @brief   Хранилище параметров в AT24C64 с CRC32 и фоновой записью.
 */
#ifndef PARAMS_STORAGE_H
#define PARAMS_STORAGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* 0x0200..0x03FF зарезервировано для параметров Modbus. */
#define PARAMS_STORAGE_BASE_ADDRESS      0x0200U
#define PARAMS_STORAGE_REGION_SIZE       0x0200U
/* Версия 4: адресно-ориентированный формат + параметр скорости волны. */
#define PARAMS_STORAGE_FORMAT_VERSION    4U
#define PARAMS_STORAGE_MAX_PAYLOAD       (PARAMS_STORAGE_REGION_SIZE - 16U)

typedef enum {
    PARAMS_STORAGE_NOT_AVAILABLE = 0,
    PARAMS_STORAGE_EMPTY,
    PARAMS_STORAGE_VALID,
    PARAMS_STORAGE_CORRUPTED
} ParamsStorageState_t;

typedef enum {
    PARAMS_SAVE_IDLE = 0,
    PARAMS_SAVE_BUSY,
    PARAMS_SAVE_COMPLETE,
    PARAMS_SAVE_ERROR
} ParamsStorageSaveState_t;

ParamsStorageState_t ParamsStorage_Init(void);
bool ParamsStorage_IsAvailable(void);

HAL_StatusTypeDef ParamsStorage_Load(uint8_t *payload,
                                     uint16_t capacity,
                                     uint16_t *payload_size);

/* Фоновая запись: Begin копирует снимок, Process выполняет максимум один шаг. */
HAL_StatusTypeDef ParamsStorage_BeginSave(const uint8_t *payload,
                                          uint16_t payload_size);
ParamsStorageSaveState_t ParamsStorage_ProcessSave(void);
ParamsStorageSaveState_t ParamsStorage_GetSaveState(void);
void ParamsStorage_ClearSaveResult(void);

/* Совместимый синхронный интерфейс; в основном цикле не использовать. */
HAL_StatusTypeDef ParamsStorage_Save(const uint8_t *payload,
                                     uint16_t payload_size);
HAL_StatusTypeDef ParamsStorage_Invalidate(void);

#ifdef __cplusplus
}
#endif

#endif /* PARAMS_STORAGE_H */
