/**
 * @file    params_storage.c
 * @brief   Хранилище параметров AT24C64 с CRC32 и фоновой постраничной записью.
 */
#include "params_storage.h"
#include "at24c64.h"
#include <string.h>

#define PARAMS_STORAGE_MAGIC       0x4D425053UL /* "MBPS" */
#define PARAMS_HEADER_SIZE         16U
#define PARAMS_READY_TIMEOUT_MS    30U

typedef enum {
    SAVE_PHASE_IDLE = 0,
    SAVE_PHASE_START_PAGE,
    SAVE_PHASE_WAIT_READY,
    SAVE_PHASE_COMPLETE,
    SAVE_PHASE_ERROR
} SavePhase_t;

static bool storage_available = false;
static uint8_t storage_buffer[PARAMS_STORAGE_REGION_SIZE];
static SavePhase_t save_phase = SAVE_PHASE_IDLE;
static uint16_t save_total_size = 0U;
static uint16_t save_offset = 0U;
static uint16_t save_chunk_size = 0U;
static uint32_t save_wait_started = 0U;

static uint32_t CRC32_Calculate(const uint8_t *data, uint16_t length)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint16_t i;

    for (i = 0U; i < length; ++i) {
        uint8_t bit;
        crc ^= data[i];
        for (bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 1UL) ? ((crc >> 1) ^ 0xEDB88320UL) : (crc >> 1);
        }
    }
    return ~crc;
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

static HAL_StatusTypeDef ReadHeader(uint16_t *payload_size)
{
    uint32_t magic;
    uint16_t version;
    uint16_t size;

    if (!storage_available || payload_size == NULL) return HAL_ERROR;

    if (AT24C64_ReadBytes(AT24C64_DEFAULT_ADDRESS,
                          PARAMS_STORAGE_BASE_ADDRESS,
                          storage_buffer,
                          PARAMS_HEADER_SIZE) != HAL_OK) {
        return HAL_ERROR;
    }

    magic = ReadU32(storage_buffer, 0U);
    version = ReadU16(storage_buffer, 4U);
    size = ReadU16(storage_buffer, 6U);

    if (magic != PARAMS_STORAGE_MAGIC ||
        version != PARAMS_STORAGE_FORMAT_VERSION ||
        size > PARAMS_STORAGE_MAX_PAYLOAD) {
        return HAL_ERROR;
    }

    *payload_size = size;
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

    storage_available =
        (AT24C64_Init(AT24C64_DEFAULT_ADDRESS) == HAL_OK);
    save_phase = SAVE_PHASE_IDLE;

    if (!storage_available) return PARAMS_STORAGE_NOT_AVAILABLE;

    if (AT24C64_ReadBytes(AT24C64_DEFAULT_ADDRESS,
                          PARAMS_STORAGE_BASE_ADDRESS,
                          storage_buffer,
                          PARAMS_HEADER_SIZE) != HAL_OK) {
        return PARAMS_STORAGE_NOT_AVAILABLE;
    }

    magic = ReadU32(storage_buffer, 0U);
    if (magic == 0xFFFFFFFFUL || magic == 0UL) {
        return PARAMS_STORAGE_EMPTY;
    }

    if (ReadHeader(&payload_size) != HAL_OK) {
        return PARAMS_STORAGE_CORRUPTED;
    }

    if (ParamsStorage_Load(NULL, 0U, &payload_size) == HAL_OK) {
        return PARAMS_STORAGE_VALID;
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

    if (!storage_available || payload_size == NULL) return HAL_ERROR;
    if (ReadHeader(&size) != HAL_OK) return HAL_ERROR;

    if (AT24C64_ReadBytes(AT24C64_DEFAULT_ADDRESS,
                          PARAMS_STORAGE_BASE_ADDRESS,
                          storage_buffer,
                          (uint16_t)(PARAMS_HEADER_SIZE + size)) != HAL_OK) {
        return HAL_ERROR;
    }

    stored_crc = ReadU32(storage_buffer, 8U);
    calculated_crc = CRC32_Calculate(&storage_buffer[PARAMS_HEADER_SIZE], size);
    if (stored_crc != calculated_crc) return HAL_ERROR;

    *payload_size = size;
    if (payload == NULL) return HAL_OK;
    if (capacity < size) return HAL_ERROR;

    memcpy(payload, &storage_buffer[PARAMS_HEADER_SIZE], size);
    return HAL_OK;
}

HAL_StatusTypeDef ParamsStorage_BeginSave(const uint8_t *payload,
                                          uint16_t payload_size)
{
    uint32_t crc;

    if (!storage_available || payload == NULL ||
        payload_size > PARAMS_STORAGE_MAX_PAYLOAD ||
        save_phase == SAVE_PHASE_START_PAGE ||
        save_phase == SAVE_PHASE_WAIT_READY) {
        return HAL_ERROR;
    }

    memset(storage_buffer, 0xFF, sizeof(storage_buffer));
    memcpy(&storage_buffer[PARAMS_HEADER_SIZE], payload, payload_size);

    crc = CRC32_Calculate(payload, payload_size);
    WriteU32(storage_buffer, 0U, PARAMS_STORAGE_MAGIC);
    WriteU16(storage_buffer, 4U, PARAMS_STORAGE_FORMAT_VERSION);
    WriteU16(storage_buffer, 6U, payload_size);
    WriteU32(storage_buffer, 8U, crc);
    WriteU32(storage_buffer, 12U, 0UL);

    save_total_size = (uint16_t)(PARAMS_HEADER_SIZE + payload_size);
    save_offset = 0U;
    save_chunk_size = 0U;
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
            save_phase = SAVE_PHASE_ERROR;
            return PARAMS_SAVE_ERROR;
        }

        save_wait_started = HAL_GetTick();
        save_phase = SAVE_PHASE_WAIT_READY;
        return PARAMS_SAVE_BUSY;
    }

    if (save_phase == SAVE_PHASE_WAIT_READY) {
        if (AT24C64_PollReady(AT24C64_DEFAULT_ADDRESS) == HAL_OK) {
            save_offset = (uint16_t)(save_offset + save_chunk_size);
            save_chunk_size = 0U;
            if (save_offset >= save_total_size) {
                save_phase = SAVE_PHASE_COMPLETE;
                return PARAMS_SAVE_COMPLETE;
            }
            save_phase = SAVE_PHASE_START_PAGE;
            return PARAMS_SAVE_BUSY;
        }

        if ((uint32_t)(HAL_GetTick() - save_wait_started) >=
            PARAMS_READY_TIMEOUT_MS) {
            save_phase = SAVE_PHASE_ERROR;
            return PARAMS_SAVE_ERROR;
        }
        return PARAMS_SAVE_BUSY;
    }

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
        save_phase = SAVE_PHASE_IDLE;
        save_total_size = 0U;
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
