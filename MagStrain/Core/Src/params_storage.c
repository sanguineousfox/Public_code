/**
 * @file    params_storage.c
 * @brief   Хранилище параметров AT24C64 с CRC32 и фоновой постраничной записью.
 */
#include "params_storage.h"
#include "at24c64.h"
#include <string.h>

#define PARAMS_STORAGE_MAGIC        0x4D425053UL /* "MBPS" */
#define PARAMS_HEADER_SIZE          16U
#define PARAMS_READY_TIMEOUT_MS     30U
#define PARAMS_POWERUP_DELAY_MS     150U
#define PARAMS_INIT_RETRY_COUNT     20U
#define PARAMS_INIT_RETRY_DELAY_MS  20U

typedef enum {
    SAVE_PHASE_IDLE = 0,
    SAVE_PHASE_START_PAGE,
    SAVE_PHASE_WAIT_READY,
    SAVE_PHASE_VERIFY_IMAGE,
    SAVE_PHASE_COMPLETE,
    SAVE_PHASE_ERROR
} SavePhase_t;

static bool storage_available = false;
static uint8_t storage_buffer[PARAMS_STORAGE_REGION_SIZE];
static uint8_t verify_buffer[AT24C64_PAGE_SIZE_BYTES];
static SavePhase_t save_phase = SAVE_PHASE_IDLE;
static uint16_t save_total_size = 0U;
static uint16_t save_payload_size = 0U;
static uint16_t save_offset = 0U;
static uint16_t save_chunk_size = 0U;
static uint32_t save_wait_started = 0U;
static ParamsStorageError_t last_error = PARAMS_STORAGE_ERROR_NONE;

static uint32_t CRC32_Update(uint32_t crc,
                             const uint8_t *data,
                             uint16_t length)
{
    uint16_t i;

    for (i = 0U; i < length; ++i) {
        uint8_t bit;
        crc ^= data[i];
        for (bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 1UL) ?
                ((crc >> 1) ^ 0xEDB88320UL) :
                (crc >> 1);
        }
    }

    return crc;
}

static uint32_t CRC32_Calculate(const uint8_t *data, uint16_t length)
{
    return ~CRC32_Update(0xFFFFFFFFUL, data, length);
}

static void WriteU16(uint8_t *buffer, uint16_t offset, uint16_t value)
{
    buffer[offset] = (uint8_t)(value >> 8);
    buffer[offset + 1U] = (uint8_t)value;
}

static uint16_t ReadU16(const uint8_t *buffer, uint16_t offset)
{
    return (uint16_t)(((uint16_t)buffer[offset] << 8) |
                      (uint16_t)buffer[offset + 1U]);
}

static void WriteU32(uint8_t *buffer, uint16_t offset, uint32_t value)
{
    buffer[offset] = (uint8_t)(value >> 24);
    buffer[offset + 1U] = (uint8_t)(value >> 16);
    buffer[offset + 2U] = (uint8_t)(value >> 8);
    buffer[offset + 3U] = (uint8_t)value;
}

static uint32_t ReadU32(const uint8_t *buffer, uint16_t offset)
{
    return ((uint32_t)buffer[offset] << 24) |
           ((uint32_t)buffer[offset + 1U] << 16) |
           ((uint32_t)buffer[offset + 2U] << 8) |
           (uint32_t)buffer[offset + 3U];
}

static void SetError(ParamsStorageError_t error)
{
    last_error = error;
}

ParamsStorageError_t ParamsStorage_GetLastError(void)
{
    return last_error;
}

const char *ParamsStorage_ErrorToString(ParamsStorageError_t error)
{
    switch (error) {
        case PARAMS_STORAGE_ERROR_NONE: return "NONE";
        case PARAMS_STORAGE_ERROR_NOT_AVAILABLE: return "NOT_AVAILABLE";
        case PARAMS_STORAGE_ERROR_READ: return "READ_ERROR";
        case PARAMS_STORAGE_ERROR_BAD_MAGIC: return "BAD_MAGIC";
        case PARAMS_STORAGE_ERROR_BAD_VERSION: return "BAD_VERSION";
        case PARAMS_STORAGE_ERROR_BAD_SIZE: return "BAD_SIZE";
        case PARAMS_STORAGE_ERROR_BAD_CRC: return "BAD_CRC";
        case PARAMS_STORAGE_ERROR_WRITE: return "WRITE_ERROR";
        case PARAMS_STORAGE_ERROR_WRITE_TIMEOUT: return "WRITE_TIMEOUT";
        case PARAMS_STORAGE_ERROR_VERIFY_MISMATCH: return "VERIFY_MISMATCH";
        default: return "UNKNOWN";
    }
}

static HAL_StatusTypeDef ReadHeader(uint16_t *payload_size)
{
    uint32_t magic;
    uint16_t version;
    uint16_t size;

    if (!storage_available || payload_size == NULL) {
        SetError(PARAMS_STORAGE_ERROR_NOT_AVAILABLE);
        return HAL_ERROR;
    }

    if (AT24C64_ReadBytes(AT24C64_DEFAULT_ADDRESS,
                          PARAMS_STORAGE_BASE_ADDRESS,
                          storage_buffer,
                          PARAMS_HEADER_SIZE) != HAL_OK) {
        SetError(PARAMS_STORAGE_ERROR_READ);
        return HAL_ERROR;
    }

    magic = ReadU32(storage_buffer, 0U);
    version = ReadU16(storage_buffer, 4U);
    size = ReadU16(storage_buffer, 6U);

    if (magic != PARAMS_STORAGE_MAGIC) {
        SetError(PARAMS_STORAGE_ERROR_BAD_MAGIC);
        return HAL_ERROR;
    }
    if (version != PARAMS_STORAGE_FORMAT_VERSION) {
        SetError(PARAMS_STORAGE_ERROR_BAD_VERSION);
        return HAL_ERROR;
    }
    if (size == 0U || size > PARAMS_STORAGE_MAX_PAYLOAD) {
        SetError(PARAMS_STORAGE_ERROR_BAD_SIZE);
        return HAL_ERROR;
    }

    *payload_size = size;
    return HAL_OK;
}

/*
 * После записи всех страниц перечитывается и проверяется весь снимок.
 * Постраничного совпадения недостаточно: статус 90 должен означать, что
 * заголовок, размер и CRC реально читаются после завершения всей операции.
 */
static HAL_StatusTypeDef VerifyStoredImage(void)
{
    uint16_t offset = 0U;
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t stored_crc;
    uint16_t stored_version;
    uint16_t stored_size;

    while (offset < save_total_size) {
        uint16_t remaining = (uint16_t)(save_total_size - offset);
        uint16_t chunk =
            (remaining < sizeof(verify_buffer)) ?
                remaining : (uint16_t)sizeof(verify_buffer);

        if (AT24C64_ReadBytes(AT24C64_DEFAULT_ADDRESS,
                              (uint16_t)(PARAMS_STORAGE_BASE_ADDRESS + offset),
                              verify_buffer,
                              chunk) != HAL_OK) {
            SetError(PARAMS_STORAGE_ERROR_READ);
            return HAL_ERROR;
        }

        if (memcmp(verify_buffer,
                   &storage_buffer[offset],
                   chunk) != 0) {
            SetError(PARAMS_STORAGE_ERROR_VERIFY_MISMATCH);
            return HAL_ERROR;
        }

        if ((uint16_t)(offset + chunk) > PARAMS_HEADER_SIZE) {
            uint16_t payload_start =
                (offset < PARAMS_HEADER_SIZE) ?
                    (uint16_t)(PARAMS_HEADER_SIZE - offset) : 0U;
            crc = CRC32_Update(crc,
                               &verify_buffer[payload_start],
                               (uint16_t)(chunk - payload_start));
        }

        offset = (uint16_t)(offset + chunk);
    }

    if (ReadU32(storage_buffer, 0U) != PARAMS_STORAGE_MAGIC) {
        SetError(PARAMS_STORAGE_ERROR_BAD_MAGIC);
        return HAL_ERROR;
    }

    stored_version = ReadU16(storage_buffer, 4U);
    stored_size = ReadU16(storage_buffer, 6U);
    stored_crc = ReadU32(storage_buffer, 8U);

    if (stored_version != PARAMS_STORAGE_FORMAT_VERSION) {
        SetError(PARAMS_STORAGE_ERROR_BAD_VERSION);
        return HAL_ERROR;
    }
    if (stored_size != save_payload_size ||
        stored_size > PARAMS_STORAGE_MAX_PAYLOAD) {
        SetError(PARAMS_STORAGE_ERROR_BAD_SIZE);
        return HAL_ERROR;
    }
    if ((~crc) != stored_crc) {
        SetError(PARAMS_STORAGE_ERROR_BAD_CRC);
        return HAL_ERROR;
    }

    SetError(PARAMS_STORAGE_ERROR_NONE);
    return HAL_OK;
}

bool ParamsStorage_IsAvailable(void)
{
    return storage_available;
}

ParamsStorageState_t ParamsStorage_Init(void)
{
    uint16_t payload_size = 0U;
    uint32_t magic;
    uint8_t attempt;

    save_phase = SAVE_PHASE_IDLE;
    storage_available = false;
    SetError(PARAMS_STORAGE_ERROR_NONE);

    HAL_Delay(PARAMS_POWERUP_DELAY_MS);

    for (attempt = 0U; attempt < PARAMS_INIT_RETRY_COUNT; ++attempt) {
        if (AT24C64_Init(AT24C64_DEFAULT_ADDRESS) == HAL_OK &&
            AT24C64_ReadBytes(AT24C64_DEFAULT_ADDRESS,
                              PARAMS_STORAGE_BASE_ADDRESS,
                              storage_buffer,
                              PARAMS_HEADER_SIZE) == HAL_OK) {
            storage_available = true;
            break;
        }

        HAL_Delay(PARAMS_INIT_RETRY_DELAY_MS);
    }

    if (!storage_available) {
        SetError(PARAMS_STORAGE_ERROR_NOT_AVAILABLE);
        return PARAMS_STORAGE_NOT_AVAILABLE;
    }

    magic = ReadU32(storage_buffer, 0U);
    if (magic == 0xFFFFFFFFUL || magic == 0UL) {
        SetError(PARAMS_STORAGE_ERROR_NONE);
        return PARAMS_STORAGE_EMPTY;
    }

    for (attempt = 0U; attempt < PARAMS_INIT_RETRY_COUNT; ++attempt) {
        if (ReadHeader(&payload_size) == HAL_OK &&
            ParamsStorage_Load(NULL, 0U, &payload_size) == HAL_OK) {
            SetError(PARAMS_STORAGE_ERROR_NONE);
            return PARAMS_STORAGE_VALID;
        }

        HAL_Delay(PARAMS_INIT_RETRY_DELAY_MS);
    }

    return PARAMS_STORAGE_CORRUPTED;
}

HAL_StatusTypeDef ParamsStorage_Load(uint8_t *payload,
                                     uint16_t capacity,
                                     uint16_t *payload_size)
{
    uint16_t size;
    uint32_t stored_crc;
    uint32_t calculated_crc;

    if (!storage_available || payload_size == NULL) {
        SetError(PARAMS_STORAGE_ERROR_NOT_AVAILABLE);
        return HAL_ERROR;
    }
    if (ReadHeader(&size) != HAL_OK) {
        return HAL_ERROR;
    }

    if (AT24C64_ReadBytes(AT24C64_DEFAULT_ADDRESS,
                          PARAMS_STORAGE_BASE_ADDRESS,
                          storage_buffer,
                          (uint16_t)(PARAMS_HEADER_SIZE + size)) != HAL_OK) {
        SetError(PARAMS_STORAGE_ERROR_READ);
        return HAL_ERROR;
    }

    stored_crc = ReadU32(storage_buffer, 8U);
    calculated_crc = CRC32_Calculate(&storage_buffer[PARAMS_HEADER_SIZE], size);
    if (stored_crc != calculated_crc) {
        SetError(PARAMS_STORAGE_ERROR_BAD_CRC);
        return HAL_ERROR;
    }

    *payload_size = size;
    if (payload == NULL) {
        SetError(PARAMS_STORAGE_ERROR_NONE);
        return HAL_OK;
    }
    if (capacity < size) {
        SetError(PARAMS_STORAGE_ERROR_BAD_SIZE);
        return HAL_ERROR;
    }

    memcpy(payload, &storage_buffer[PARAMS_HEADER_SIZE], size);
    SetError(PARAMS_STORAGE_ERROR_NONE);
    return HAL_OK;
}

HAL_StatusTypeDef ParamsStorage_BeginSave(const uint8_t *payload,
                                          uint16_t payload_size)
{
    uint32_t crc;

    if (!storage_available || payload == NULL ||
        payload_size == 0U ||
        payload_size > PARAMS_STORAGE_MAX_PAYLOAD ||
        save_phase == SAVE_PHASE_START_PAGE ||
        save_phase == SAVE_PHASE_WAIT_READY ||
        save_phase == SAVE_PHASE_VERIFY_IMAGE) {
        SetError(PARAMS_STORAGE_ERROR_WRITE);
        return HAL_ERROR;
    }

    AT24C64_AbortWriteCycle();

    memset(storage_buffer, 0xFF, sizeof(storage_buffer));
    memcpy(&storage_buffer[PARAMS_HEADER_SIZE], payload, payload_size);

    crc = CRC32_Calculate(payload, payload_size);
    WriteU32(storage_buffer, 0U, PARAMS_STORAGE_MAGIC);
    WriteU16(storage_buffer, 4U, PARAMS_STORAGE_FORMAT_VERSION);
    WriteU16(storage_buffer, 6U, payload_size);
    WriteU32(storage_buffer, 8U, crc);
    WriteU32(storage_buffer, 12U, 0UL);

    save_total_size = (uint16_t)(PARAMS_HEADER_SIZE + payload_size);
    save_payload_size = payload_size;
    save_offset = 0U;
    save_chunk_size = 0U;
    SetError(PARAMS_STORAGE_ERROR_NONE);
    save_phase = SAVE_PHASE_START_PAGE;
    return HAL_OK;
}

ParamsStorageSaveState_t ParamsStorage_ProcessSave(void)
{
    if (save_phase == SAVE_PHASE_IDLE) return PARAMS_SAVE_IDLE;
    if (save_phase == SAVE_PHASE_COMPLETE) return PARAMS_SAVE_COMPLETE;
    if (save_phase == SAVE_PHASE_ERROR) return PARAMS_SAVE_ERROR;

    if (save_phase == SAVE_PHASE_START_PAGE) {
        uint16_t current_address =
            (uint16_t)(PARAMS_STORAGE_BASE_ADDRESS + save_offset);
        uint16_t page_offset =
            (uint16_t)(current_address % AT24C64_PAGE_SIZE_BYTES);
        uint16_t page_space =
            (uint16_t)(AT24C64_PAGE_SIZE_BYTES - page_offset);
        uint16_t remaining = (uint16_t)(save_total_size - save_offset);

        save_chunk_size = (remaining < page_space) ? remaining : page_space;

        if (AT24C64_WritePageBegin(AT24C64_DEFAULT_ADDRESS,
                                   current_address,
                                   &storage_buffer[save_offset],
                                   save_chunk_size) != HAL_OK) {
            AT24C64_AbortWriteCycle();
            SetError(PARAMS_STORAGE_ERROR_WRITE);
            save_phase = SAVE_PHASE_ERROR;
            return PARAMS_SAVE_ERROR;
        }

        save_wait_started = HAL_GetTick();
        save_phase = SAVE_PHASE_WAIT_READY;
        return PARAMS_SAVE_BUSY;
    }

    if (save_phase == SAVE_PHASE_WAIT_READY) {
        if (AT24C64_PollReady(AT24C64_DEFAULT_ADDRESS) == HAL_OK) {
            uint16_t current_address =
                (uint16_t)(PARAMS_STORAGE_BASE_ADDRESS + save_offset);

            if (save_chunk_size == 0U ||
                save_chunk_size > sizeof(verify_buffer) ||
                AT24C64_ReadBytes(AT24C64_DEFAULT_ADDRESS,
                                  current_address,
                                  verify_buffer,
                                  save_chunk_size) != HAL_OK ||
                memcmp(verify_buffer,
                       &storage_buffer[save_offset],
                       save_chunk_size) != 0) {
                AT24C64_AbortWriteCycle();
                SetError(PARAMS_STORAGE_ERROR_VERIFY_MISMATCH);
                save_phase = SAVE_PHASE_ERROR;
                return PARAMS_SAVE_ERROR;
            }

            save_offset = (uint16_t)(save_offset + save_chunk_size);
            save_chunk_size = 0U;
            if (save_offset >= save_total_size) {
                save_phase = SAVE_PHASE_VERIFY_IMAGE;
                return PARAMS_SAVE_BUSY;
            }

            save_phase = SAVE_PHASE_START_PAGE;
            return PARAMS_SAVE_BUSY;
        }

        if ((uint32_t)(HAL_GetTick() - save_wait_started) >=
            PARAMS_READY_TIMEOUT_MS) {
            AT24C64_AbortWriteCycle();
            SetError(PARAMS_STORAGE_ERROR_WRITE_TIMEOUT);
            save_phase = SAVE_PHASE_ERROR;
            return PARAMS_SAVE_ERROR;
        }

        return PARAMS_SAVE_BUSY;
    }

    if (save_phase == SAVE_PHASE_VERIFY_IMAGE) {
        if (VerifyStoredImage() == HAL_OK) {
            save_phase = SAVE_PHASE_COMPLETE;
            return PARAMS_SAVE_COMPLETE;
        }

        save_phase = SAVE_PHASE_ERROR;
        return PARAMS_SAVE_ERROR;
    }

    SetError(PARAMS_STORAGE_ERROR_WRITE);
    return PARAMS_SAVE_ERROR;
}

ParamsStorageSaveState_t ParamsStorage_GetSaveState(void)
{
    switch (save_phase) {
        case SAVE_PHASE_IDLE: return PARAMS_SAVE_IDLE;
        case SAVE_PHASE_COMPLETE: return PARAMS_SAVE_COMPLETE;
        case SAVE_PHASE_ERROR: return PARAMS_SAVE_ERROR;
        default: return PARAMS_SAVE_BUSY;
    }
}

void ParamsStorage_ClearSaveResult(void)
{
    if (save_phase == SAVE_PHASE_COMPLETE || save_phase == SAVE_PHASE_ERROR) {
        AT24C64_AbortWriteCycle();
        save_phase = SAVE_PHASE_IDLE;
        save_total_size = 0U;
        save_payload_size = 0U;
        save_offset = 0U;
        save_chunk_size = 0U;
    }
}

HAL_StatusTypeDef ParamsStorage_Save(const uint8_t *payload,
                                     uint16_t payload_size)
{
    ParamsStorageSaveState_t state;

    if (ParamsStorage_BeginSave(payload, payload_size) != HAL_OK) {
        return HAL_ERROR;
    }

    do {
        state = ParamsStorage_ProcessSave();
        if (state == PARAMS_SAVE_BUSY) HAL_Delay(1U);
    } while (state == PARAMS_SAVE_BUSY);

    ParamsStorage_ClearSaveResult();
    return (state == PARAMS_SAVE_COMPLETE) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef ParamsStorage_Invalidate(void)
{
    static const uint8_t zero_magic[4] = {0U, 0U, 0U, 0U};

    if (!storage_available) return HAL_ERROR;
    if (ParamsStorage_GetSaveState() == PARAMS_SAVE_BUSY) return HAL_BUSY;

    return AT24C64_WriteBytes(AT24C64_DEFAULT_ADDRESS,
                              PARAMS_STORAGE_BASE_ADDRESS,
                              zero_magic,
                              sizeof(zero_magic));
}
