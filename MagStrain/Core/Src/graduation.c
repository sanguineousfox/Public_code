/** @file graduation.c @brief Градуировочные таблицы резервуара. */
#include "graduation.h"
#include "at24c64.h"
#include "modbus.h"
#include "utils.h"

#include <math.h>
#include <string.h>

#define STORAGE_HEADER_CRC_OFFSET 28U

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t points_count;
    float start_height_m;
    float step_height_m;
    float tank_height_m;
    float tank_volume_m3;
    uint32_t data_crc32;
    uint32_t header_crc32;
} __attribute__((packed)) GradStorageHeader_t;

static GradState_t state;

static uint32_t CRC32_Calculate(const uint8_t *data, uint32_t length)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t i;

    for (i = 0U; i < length; ++i) {
        uint8_t bit;
        crc ^= data[i];
        for (bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 1UL) ? ((crc >> 1) ^ 0xEDB88320UL) : (crc >> 1);
        }
    }
    return ~crc;
}

static bool HeaderValuesAreValid(const GradStorageHeader_t *header)
{
    uint32_t header_crc;

    if (header == NULL ||
        header->magic != GRAD_MAGIC ||
        header->version != GRAD_FORMAT_VERSION ||
        header->points_count > GRAD_MAX_POINTS ||
        !isfinite(header->start_height_m) ||
        !isfinite(header->step_height_m) ||
        !isfinite(header->tank_height_m) ||
        !isfinite(header->tank_volume_m3) ||
        header->step_height_m <= 0.0f) {
        return false;
    }

    header_crc = CRC32_Calculate((const uint8_t *)header,
                                 STORAGE_HEADER_CRC_OFFSET);
    return header_crc == header->header_crc32;
}

static void PublicToStorage(const GradHeader_t *source,
                            GradStorageHeader_t *target,
                            uint16_t points,
                            uint32_t data_crc)
{
    memset(target, 0, sizeof(*target));
    target->magic = GRAD_MAGIC;
    target->version = GRAD_FORMAT_VERSION;
    target->points_count = points;
    target->start_height_m = source->start_height_m;
    target->step_height_m = source->step_height_m;
    target->tank_height_m = source->tank_height_m;
    target->tank_volume_m3 = source->tank_volume_m3;
    target->data_crc32 = data_crc;
    target->header_crc32 = CRC32_Calculate((const uint8_t *)target,
                                           STORAGE_HEADER_CRC_OFFSET);
}

static void StorageToPublic(const GradStorageHeader_t *source,
                            GradHeader_t *target)
{
    memset(target, 0, sizeof(*target));
    target->magic = source->magic;
    target->points_count = source->points_count;
    target->reserved = source->version;
    target->start_height_m = source->start_height_m;
    target->step_height_m = source->step_height_m;
    target->tank_height_m = source->tank_height_m;
    target->tank_volume_m3 = source->tank_volume_m3;
}

static float CalculateVertical(float level, float height, float volume)
{
    if (height <= 0.0f || volume <= 0.0f || level <= 0.0f) return 0.0f;
    if (level >= height) return volume;
    return volume * level / height;
}

static float CalculateHorizontal(float level, float diameter, float volume)
{
    float radius;
    float area;
    float length;

    if (diameter <= 0.0f || volume <= 0.0f || level <= 0.0f) return 0.0f;
    if (level >= diameter) return volume;

    radius = diameter * 0.5f;
    area = radius * radius * acosf((radius - level) / radius) -
           (radius - level) * sqrtf(2.0f * radius * level - level * level);
    length = volume / (3.14159265358979323846f * radius * radius);
    return area * length;
}

static float CalculateHorizontalElliptic(float level,
                                         float diameter,
                                         float volume)
{
    float cylindrical = CalculateHorizontal(level, diameter, volume);
    float radius;
    float fraction;

    if (diameter <= 0.0f || volume <= 0.0f || level <= 0.0f) return 0.0f;
    if (level >= diameter) return volume;

    radius = diameter * 0.5f;
    if (level <= radius) {
        fraction = (level / radius) * (level / radius);
    } else {
        float remaining = (diameter - level) / radius;
        fraction = 1.0f - remaining * remaining;
    }

    return cylindrical + volume * 0.1f * fraction;
}

void Grad_Init(void)
{
    memset(&state, 0, sizeof(state));
    if (Grad_LoadFromEEPROM() == HAL_OK) {
        USART2_BufInit();
        USART2_BufPrint("[GRAD] Таблица загружена: ");
        USART2_BufPrintInt(state.actual_points);
        USART2_BufPrint(" точек\r\n");
        USART2_BufFlush();
    } else {
        USART2_Print("[GRAD] Валидная таблица не найдена\r\n");
    }
}

bool Grad_IsValid(void)
{
    return state.loaded && state.valid;
}

HAL_StatusTypeDef Grad_LoadFromEEPROM(void)
{
    GradStorageHeader_t storage_header;
    uint32_t data_size;
    uint32_t calculated_crc;

    if (AT24C64_ReadBytes(AT24C64_DEFAULT_ADDRESS,
                          GRAD_EEPROM_BASE,
                          (uint8_t *)&storage_header,
                          sizeof(storage_header)) != HAL_OK ||
        !HeaderValuesAreValid(&storage_header)) {
        state.loaded = false;
        state.valid = false;
        return HAL_ERROR;
    }

    data_size = (uint32_t)storage_header.points_count * sizeof(float);
    if (data_size > 0U &&
        AT24C64_ReadBytes(AT24C64_DEFAULT_ADDRESS,
                          GRAD_DATA_START,
                          (uint8_t *)state.volumes,
                          (uint16_t)data_size) != HAL_OK) {
        state.loaded = false;
        state.valid = false;
        return HAL_ERROR;
    }

    calculated_crc = CRC32_Calculate((const uint8_t *)state.volumes, data_size);
    if (calculated_crc != storage_header.data_crc32) {
        state.loaded = false;
        state.valid = false;
        return HAL_ERROR;
    }

    StorageToPublic(&storage_header, &state.header);
    state.actual_points = storage_header.points_count;
    state.crc32 = calculated_crc;
    state.loaded = true;
    state.valid = true;
    Grad_UpdateModbusRegisters();
    return HAL_OK;
}

HAL_StatusTypeDef Grad_SaveToEEPROM(void)
{
    GradStorageHeader_t storage_header;
    uint32_t data_size;
    uint32_t data_crc;

    if (!state.loaded || state.actual_points > GRAD_MAX_POINTS) {
        return HAL_ERROR;
    }

    data_size = (uint32_t)state.actual_points * sizeof(float);
    data_crc = CRC32_Calculate((const uint8_t *)state.volumes, data_size);
    PublicToStorage(&state.header,
                    &storage_header,
                    state.actual_points,
                    data_crc);

    if (data_size > 0U &&
        AT24C64_WriteBytes(AT24C64_DEFAULT_ADDRESS,
                           GRAD_DATA_START,
                           (const uint8_t *)state.volumes,
                           (uint16_t)data_size) != HAL_OK) {
        return HAL_ERROR;
    }

    if (AT24C64_WriteBytes(AT24C64_DEFAULT_ADDRESS,
                           GRAD_EEPROM_BASE,
                           (const uint8_t *)&storage_header,
                           sizeof(storage_header)) != HAL_OK) {
        return HAL_ERROR;
    }

    state.header.magic = GRAD_MAGIC;
    state.header.points_count = state.actual_points;
    state.crc32 = data_crc;
    state.valid = true;
    Grad_UpdateModbusRegisters();
    return HAL_OK;
}

HAL_StatusTypeDef Grad_Clear(void)
{
    static const uint8_t invalid_magic[4] = {0U, 0U, 0U, 0U};

    memset(&state, 0, sizeof(state));
    return AT24C64_WriteBytes(AT24C64_DEFAULT_ADDRESS,
                              GRAD_EEPROM_BASE,
                              invalid_magic,
                              sizeof(invalid_magic));
}

HAL_StatusTypeDef Grad_WriteHeader(const GradHeader_t *header)
{
    if (header == NULL || header->points_count > GRAD_MAX_POINTS) {
        return HAL_ERROR;
    }

    state.header = *header;
    state.header.magic = GRAD_MAGIC;
    state.actual_points = header->points_count;
    state.loaded = true;
    state.valid = false;
    return HAL_OK;
}

HAL_StatusTypeDef Grad_ReadHeader(GradHeader_t *header)
{
    if (header == NULL || !state.loaded) return HAL_ERROR;
    *header = state.header;
    return HAL_OK;
}

HAL_StatusTypeDef Grad_WriteVolumes(uint16_t start_index,
                                    const float *volumes,
                                    uint16_t count)
{
    uint32_t end = (uint32_t)start_index + count;

    if (volumes == NULL || count == 0U || end > GRAD_MAX_POINTS) {
        return HAL_ERROR;
    }

    memcpy(&state.volumes[start_index], volumes, count * sizeof(float));
    if (end > state.actual_points) {
        state.actual_points = (uint16_t)end;
        state.header.points_count = state.actual_points;
    }
    state.loaded = true;
    state.valid = false;

    return AT24C64_WriteBytes(AT24C64_DEFAULT_ADDRESS,
                              (uint16_t)(GRAD_DATA_START +
                                         start_index * sizeof(float)),
                              (const uint8_t *)volumes,
                              (uint16_t)(count * sizeof(float)));
}

HAL_StatusTypeDef Grad_ReadVolumes(uint16_t start_index,
                                   float *volumes,
                                   uint16_t count)
{
    uint32_t end = (uint32_t)start_index + count;

    if (volumes == NULL || end > state.actual_points) return HAL_ERROR;
    memcpy(volumes, &state.volumes[start_index], count * sizeof(float));
    return HAL_OK;
}

float Grad_InterpolateVolume(float level_m)
{
    float relative;
    uint16_t index;
    float fraction;

    if (!Grad_IsValid() || state.actual_points < 2U ||
        state.header.step_height_m <= 0.0f) return NAN;

    if (level_m <= state.header.start_height_m) return state.volumes[0];

    relative = (level_m - state.header.start_height_m) /
               state.header.step_height_m;
    index = (uint16_t)relative;
    if (index >= state.actual_points - 1U) {
        return state.volumes[state.actual_points - 1U];
    }

    fraction = relative - (float)index;
    return state.volumes[index] +
           (state.volumes[index + 1U] - state.volumes[index]) * fraction;
}

float Grad_CalculateVolume(GradTankType_t tank_type,
                           float level_m,
                           float height_m,
                           float volume_m3)
{
    switch (tank_type) {
        case GRAD_TYPE_VERTICAL:
            return CalculateVertical(level_m, height_m, volume_m3);
        case GRAD_TYPE_HORIZ_FLAT:
            return CalculateHorizontal(level_m, height_m, volume_m3);
        case GRAD_TYPE_HORIZ_ELLIPT:
            return CalculateHorizontalElliptic(level_m, height_m, volume_m3);
        case GRAD_TYPE_BY_TABLE:
            return Grad_InterpolateVolume(level_m);
        default:
            return NAN;
    }
}

GradState_t *Grad_GetState(void)
{
    return &state;
}

void Grad_UpdateModbusRegisters(void)
{
    ModBus_SetParameter_Float(MB_ADDR_CALIB_POINTS,
                              (float)state.actual_points);
}

void Grad_ProcessCommand(uint16_t command, float parameter)
{
    switch (command) {
        case 300U: {
            GradHeader_t header = {0};
            (void)Grad_Clear();
            if (isfinite(parameter) && parameter > 0.0f &&
                parameter <= (float)GRAD_MAX_POINTS) {
                header.magic = GRAD_MAGIC;
                header.points_count = (uint16_t)parameter;
                header.start_height_m = 0.0f;
                header.step_height_m = 0.01f;
                (void)Grad_WriteHeader(&header);
                ModBus_SetParameter_Int(MB_ADDR_COMMAND, 90U);
            } else {
                ModBus_SetParameter_Int(MB_ADDR_COMMAND, 0U);
            }
            break;
        }

        case 301U:
            ModBus_SetParameter_Int(MB_ADDR_COMMAND,
                (Grad_SaveToEEPROM() == HAL_OK) ? 90U : 0U);
            break;

        case 302U:
            ModBus_SetParameter_Int(MB_ADDR_COMMAND,
                (Grad_Clear() == HAL_OK) ? 90U : 0U);
            break;

        default:
            break;
    }
}
