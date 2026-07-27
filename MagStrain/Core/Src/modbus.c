/**
 * @file    modbus.c
 * @brief   Modbus RTU с типизированной картой и атомарной записью float32.
 */
#include "modbus.h"
#include "measurement_snapshot.h"
#include "params_storage.h"
#include "rs485.h"
#include "utils.h"

#include <math.h>
#include <string.h>

#define MODBUS_MAX_READ_REGISTERS       125U
#define MODBUS_MAX_WRITE_REGISTERS      123U
#define MODBUS_FRAME_END_TIMEOUT_MS     2U
#define MODBUS_INTERBYTE_RESET_MS       4U
#define MODBUS_TX_TIMEOUT_MS            100U
#define MODBUS_EEPROM_WRITE_DELAY_MS    5000U

#define PARAM_FLAG_PERSISTENT           0x01U

#define DESC_U16(addr, flags)   { (addr), MODBUS_REGISTER_UINT16, (flags) }
#define DESC_F32(addr, flags)   { (addr), MODBUS_REGISTER_FLOAT32, (flags) }

#define PERSISTENT PARAM_FLAG_PERSISTENT
#define VOLATILE   0U

typedef struct {
    uint16_t address;
    ModBus_RegisterType_t type;
    uint8_t flags;
} ModBus_Descriptor_t;

/*
 * Только явно зарезервированные адреса. Пробелы в карте не перехватываются.
 * Каждый descriptor получает две внутренние ячейки; UINT16 использует первую.
 */
static const ModBus_Descriptor_t descriptors[] = {
    DESC_U16(MB_ADDR_LEVEL_INT, VOLATILE),
    DESC_U16(MB_ADDR_TEMP_INT, VOLATILE),
    DESC_U16(MB_ADDR_PERCENT_INT, VOLATILE),
    DESC_U16(MB_ADDR_VOLUME_INT, VOLATILE),
    DESC_U16(MB_ADDR_MASS_INT, VOLATILE),
    DESC_U16(MB_ADDR_DENSITY_INT, VOLATILE),
    DESC_U16(MB_ADDR_VOLUME_MAIN_INT, VOLATILE),
    DESC_U16(MB_ADDR_LEVEL_INT_SEP, VOLATILE),
    DESC_U16(MB_ADDR_TEMP_VAPOR_INT, VOLATILE),
    DESC_U16(MB_ADDR_MASS_VAPOR_INT, VOLATILE),
    DESC_U16(MB_ADDR_MASS_LIQ_INT, VOLATILE),
    DESC_U16(MB_ADDR_VOLUME_STD_INT, VOLATILE),
    DESC_U16(MB_ADDR_DENSITY_STD_INT, VOLATILE),
    DESC_U16(MB_ADDR_DENSITY_MEAS_INT, VOLATILE),
    DESC_U16(MB_ADDR_TEMP_DENS_INT, VOLATILE),
    DESC_U16(MB_ADDR_VOLUME_SEP_INT, VOLATILE),
    DESC_U16(MB_ADDR_MASS_ERROR_INT, VOLATILE),
    DESC_U16(MB_ADDR_MB_ADDR_SET, PERSISTENT),
    DESC_U16(MB_ADDR_MB_BAUD_SET, PERSISTENT),
    DESC_U16(MB_ADDR_MB_PARITY_SET, PERSISTENT),

    DESC_F32(MB_ADDR_LEVEL, VOLATILE),
    DESC_F32(MB_ADDR_TEMP, VOLATILE),
    DESC_F32(MB_ADDR_PERCENT, VOLATILE),
    DESC_F32(MB_ADDR_VOLUME, VOLATILE),
    DESC_F32(MB_ADDR_MASS, VOLATILE),
    DESC_F32(MB_ADDR_DENSITY, VOLATILE),
    DESC_F32(MB_ADDR_VOLUME_MAIN, VOLATILE),
    DESC_F32(MB_ADDR_LEVEL_SEP, VOLATILE),
    DESC_F32(MB_ADDR_TEMP_VAPOR, VOLATILE),
    DESC_F32(MB_ADDR_MASS_VAPOR, VOLATILE),
    DESC_F32(MB_ADDR_MASS_LIQ, VOLATILE),
    DESC_F32(MB_ADDR_VOLUME_STD, VOLATILE),
    DESC_F32(MB_ADDR_DENSITY_STD, VOLATILE),
    DESC_F32(MB_ADDR_DENSITY_MEAS, VOLATILE),
    DESC_F32(MB_ADDR_TEMP_DENS, VOLATILE),
    DESC_F32(MB_ADDR_VOLUME_SEP, VOLATILE),

    DESC_F32(MB_ADDR_CAL_LOW_LVL, PERSISTENT),
    DESC_F32(MB_ADDR_CAL_HIGH_LVL, PERSISTENT),
    DESC_F32(MB_ADDR_PROBE_DEPTH, PERSISTENT),
    DESC_F32(MB_ADDR_LEVEL_OFFSET, PERSISTENT),
    DESC_F32(MB_ADDR_TANK_GEOM, PERSISTENT),
    DESC_F32(MB_ADDR_TANK_HEIGHT, PERSISTENT),
    DESC_F32(MB_ADDR_TANK_VOLUME, PERSISTENT),
    DESC_F32(MB_ADDR_CALIB_POINTS, PERSISTENT),
    DESC_F32(MB_ADDR_DENSITY_LOW, PERSISTENT),
    DESC_F32(MB_ADDR_DENSITY_HIGH, PERSISTENT),
    DESC_F32(MB_ADDR_EXPANSION_COEF, PERSISTENT),
    DESC_F32(MB_ADDR_DENSITY_ORIG, PERSISTENT),
    DESC_F32(MB_ADDR_TEMP_ORIG, PERSISTENT),
    DESC_F32(MB_ADDR_SEP_DEPTH, PERSISTENT),
    DESC_F32(MB_ADDR_PROPANE_RATIO, PERSISTENT),
    DESC_F32(MB_ADDR_DENSITY_MIN, PERSISTENT),
    DESC_F32(MB_ADDR_LEVEL_ZERO_SEP, PERSISTENT),
    DESC_F32(MB_ADDR_THRESH_LVL, PERSISTENT),
    DESC_F32(MB_ADDR_MAGNET_DIFF, PERSISTENT),
    DESC_F32(MB_ADDR_BUTANE_RATIO, PERSISTENT),
    DESC_F32(MB_ADDR_DENSITY_MAX, PERSISTENT),
    DESC_F32(MB_ADDR_ISOBUTANE_RATIO, PERSISTENT),
    DESC_F32(MB_ADDR_MASS_REL_ERROR, PERSISTENT),
    DESC_F32(MB_ADDR_TANK_ERROR, PERSISTENT),
    DESC_F32(MB_ADDR_TEMP_STD, PERSISTENT),
    DESC_F32(MB_ADDR_BAUD_RATE, PERSISTENT),
    DESC_F32(MB_ADDR_PARITY, PERSISTENT),
    DESC_F32(MB_ADDR_DEVICE_ADDR, PERSISTENT),
    DESC_F32(MB_ADDR_DAMPING_TIME, PERSISTENT),
    DESC_F32(MB_ADDR_POLL_PERIOD, PERSISTENT),
    DESC_F32(MB_ADDR_VOLUME_15C, PERSISTENT),
    DESC_F32(MB_ADDR_DENSITY_15C, PERSISTENT),
    DESC_F32(MB_ADDR_MATERIAL_WAVE_SPEED, PERSISTENT),
    DESC_F32(MB_ADDR_WAVEGUIDE_LEN, PERSISTENT),
    DESC_F32(MB_ADDR_WAVEGUIDE_DEV, PERSISTENT),
    DESC_F32(MB_ADDR_LEVEL_CORR, PERSISTENT),
    DESC_F32(MB_ADDR_DENSITY_CORR, PERSISTENT),
    DESC_F32(MB_ADDR_MEDIUM_TYPE, PERSISTENT),
    DESC_F32(MB_ADDR_TANK_EXPANSION, PERSISTENT),
    DESC_F32(MB_ADDR_UNIT_LEVEL, PERSISTENT),
    DESC_F32(MB_ADDR_UNIT_TEMP, PERSISTENT),
    DESC_F32(MB_ADDR_UNIT_VOLUME, PERSISTENT),
    DESC_F32(MB_ADDR_UNIT_MASS, PERSISTENT),
    DESC_F32(MB_ADDR_UNIT_DENSITY, PERSISTENT),

    DESC_U16(MB_ADDR_SERIAL_HI, PERSISTENT),
    DESC_U16(MB_ADDR_SERIAL_LO, PERSISTENT),
    DESC_F32(MB_ADDR_DAMPING_DENS, PERSISTENT),
    DESC_F32(MB_ADDR_CAL_C1, PERSISTENT),
    DESC_F32(MB_ADDR_CAL_C2, PERSISTENT),
    DESC_F32(MB_ADDR_CAL_D4, PERSISTENT),
    DESC_F32(MB_ADDR_CAL_D5, PERSISTENT),
    DESC_U16(MB_ADDR_TEMP_SENS_COUNT, VOLATILE),
    DESC_U16(MB_ADDR_ERROR_CODE, VOLATILE),
    DESC_U16(MB_ADDR_SENS_ADDR, PERSISTENT),
    DESC_U16(MB_ADDR_FW_VERSION, VOLATILE),
    DESC_U16(MB_ADDR_ADMIN_PASS, PERSISTENT),
    DESC_F32(MB_ADDR_ERROR_DELAY, PERSISTENT),

    DESC_F32(MB_ADDR_TEMP_SENS_1_H, PERSISTENT),
    DESC_F32(MB_ADDR_TEMP_SENS_2_H, PERSISTENT),
    DESC_F32(MB_ADDR_TEMP_SENS_3_H, PERSISTENT),
    DESC_F32(MB_ADDR_TEMP_SENS_4_H, PERSISTENT),
    DESC_F32(MB_ADDR_TEMP_SENS_5_H, PERSISTENT),
    DESC_F32(MB_ADDR_TEMP_SENS_6_H, PERSISTENT),
    DESC_F32(MB_ADDR_TEMP_SENS_7_H, PERSISTENT),
    DESC_F32(MB_ADDR_TEMP_SENS_8_H, PERSISTENT),
    DESC_F32(MB_ADDR_TEMP_SENS_1_V, VOLATILE),
    DESC_F32(MB_ADDR_TEMP_SENS_2_V, VOLATILE),
    DESC_F32(MB_ADDR_TEMP_SENS_3_V, VOLATILE),
    DESC_F32(MB_ADDR_TEMP_SENS_4_V, VOLATILE),
    DESC_F32(MB_ADDR_TEMP_SENS_5_V, VOLATILE),
    DESC_F32(MB_ADDR_TEMP_SENS_6_V, VOLATILE),
    DESC_F32(MB_ADDR_TEMP_SENS_7_V, VOLATILE),
    DESC_F32(MB_ADDR_TEMP_SENS_8_V, VOLATILE),
    DESC_F32(MB_ADDR_DENS_SENS_1, VOLATILE),
    DESC_F32(MB_ADDR_DENS_SENS_2, VOLATILE),
    DESC_F32(MB_ADDR_DENS_SENS_3, VOLATILE),
    DESC_F32(MB_ADDR_DENS_SENS_4, VOLATILE),
    DESC_F32(MB_ADDR_DENS_SENS_5, VOLATILE),
    DESC_F32(MB_ADDR_DENS_SENS_6, VOLATILE),
    DESC_F32(MB_ADDR_DENS_SENS_7, VOLATILE),
    DESC_F32(MB_ADDR_DENS_SENS_8, VOLATILE),

    DESC_U16(MB_ADDR_COMMAND, VOLATILE),
    DESC_F32(MB_ADDR_COMMAND_PARAM, VOLATILE)
};

#define DESCRIPTOR_COUNT ((uint16_t)(sizeof(descriptors) / sizeof(descriptors[0])))
#define REGISTER_STORAGE_WORDS ((uint16_t)(DESCRIPTOR_COUNT * 2U))

/* Максимум: count + все persistent слова. */
#define PERSISTENCE_BUFFER_SIZE  400U

#if PERSISTENCE_BUFFER_SIZE > PARAMS_STORAGE_MAX_PAYLOAD
#error "Persistence buffer is larger than EEPROM storage payload"
#endif

typedef struct {
    uint16_t regs[REGISTER_STORAGE_WORDS];
    uint8_t device_address;
    volatile uint16_t rx_length;
    volatile uint8_t rx_byte;
    volatile uint32_t last_byte_tick;
    volatile uint8_t processing;
    volatile uint8_t tx_busy;
    uint8_t rx_buffer[MODBUS_BUFFER_SIZE];
    uint8_t process_buffer[MODBUS_BUFFER_SIZE];
} ModBus_State_t;

typedef struct {
    uint16_t address;
    float value;
} FloatDefault_t;

typedef struct {
    uint16_t address;
    uint16_t value;
} U16Default_t;

static ModBus_State_t modbus;
static uint8_t persistence_buffer[PERSISTENCE_BUFFER_SIZE];
static bool storage_available = false;
static bool storage_dirty = false;
static bool storage_force_requested = false;
static bool request_is_broadcast = false;
static uint32_t storage_dirty_since = 0U;
static uint32_t storage_revision = 0U;
static uint32_t storage_save_revision = 0U;

static float diagnostic_vdda;
static float diagnostic_v24;
static float diagnostic_v12;
static float diagnostic_v5;

extern UART_HandleTypeDef huart1;

static void SynchronizeCommunicationSettings(uint16_t touched_address);

static const FloatDefault_t float_defaults[] = {
    {MB_ADDR_CAL_LOW_LVL, 0.1f},
    {MB_ADDR_CAL_HIGH_LVL, 1.0f},
    {MB_ADDR_PROBE_DEPTH, 0.0f},
    {MB_ADDR_LEVEL_OFFSET, 0.0f},
    {MB_ADDR_TANK_GEOM, 0.0f},
    {MB_ADDR_TANK_HEIGHT, 0.95f},
    {MB_ADDR_TANK_VOLUME, 0.0f},
    {MB_ADDR_DAMPING_TIME, 10.0f},
    {MB_ADDR_POLL_PERIOD, 100.0f},
    {MB_ADDR_MATERIAL_WAVE_SPEED, 3370.0f},
    {MB_ADDR_WAVEGUIDE_LEN, 1.2f},
    {MB_ADDR_THRESH_LVL, 0.9f},
    {MB_ADDR_MEDIUM_TYPE, 1.0f},
    {MB_ADDR_UNIT_LEVEL, 0.0f},
    {MB_ADDR_UNIT_TEMP, 0.0f},
    {MB_ADDR_BAUD_RATE, (float)MODBUS_DEFAULT_BAUDRATE},
    {MB_ADDR_PARITY, 0.0f},
    {MB_ADDR_DEVICE_ADDR, (float)MODBUS_DEFAULT_ADDRESS}
};

static const U16Default_t u16_defaults[] = {
    {MB_ADDR_MB_ADDR_SET, MODBUS_DEFAULT_ADDRESS},
    {MB_ADDR_MB_BAUD_SET, MODBUS_DEFAULT_BAUDRATE},
    {MB_ADDR_MB_PARITY_SET, 0U},
    {MB_ADDR_ADMIN_PASS, 0U}
};

static const ModBus_Descriptor_t *FindDescriptorByStart(uint16_t address,
                                                        uint16_t *descriptor_index)
{
    uint16_t i;

    for (i = 0U; i < DESCRIPTOR_COUNT; ++i) {
        if (descriptors[i].address == address) {
            if (descriptor_index != NULL) {
                *descriptor_index = i;
            }
            return &descriptors[i];
        }
    }

    return NULL;
}

static const ModBus_Descriptor_t *FindDescriptorByWord(uint16_t address,
                                                       uint16_t *descriptor_index,
                                                       uint8_t *word_offset)
{
    uint16_t i;

    for (i = 0U; i < DESCRIPTOR_COUNT; ++i) {
        const ModBus_Descriptor_t *descriptor = &descriptors[i];

        if (address == descriptor->address) {
            if (descriptor_index != NULL) {
                *descriptor_index = i;
            }
            if (word_offset != NULL) {
                *word_offset = 0U;
            }
            return descriptor;
        }

        if (descriptor->type == MODBUS_REGISTER_FLOAT32 &&
            address == (uint16_t)(descriptor->address + 1U)) {
            if (descriptor_index != NULL) {
                *descriptor_index = i;
            }
            if (word_offset != NULL) {
                *word_offset = 1U;
            }
            return descriptor;
        }
    }

    return NULL;
}

static uint16_t StorageIndex(uint16_t descriptor_index, uint8_t word_offset)
{
    return (uint16_t)(descriptor_index * 2U + word_offset);
}

uint16_t ModBus_AddressToIndex_External(uint16_t address)
{
    uint16_t descriptor_index;
    uint8_t word_offset;

    if (FindDescriptorByWord(address, &descriptor_index, &word_offset) == NULL) {
        return MODBUS_INVALID_INDEX;
    }

    return StorageIndex(descriptor_index, word_offset);
}

ModBus_RegisterType_t ModBus_GetRegisterType(uint16_t address)
{
    const ModBus_Descriptor_t *descriptor =
        FindDescriptorByWord(address, NULL, NULL);

    return (descriptor == NULL) ? MODBUS_REGISTER_UNDEFINED : descriptor->type;
}

bool ModBus_ReadRawRegister(uint16_t address, uint16_t *value)
{
    uint16_t index;

    if (value == NULL) {
        return false;
    }

    /* 1000..1007 читаются из уже подготовленного снимка RAM. */
    if (MeasurementSnapshot_ReadWord(address, value)) {
        return true;
    }

    index = ModBus_AddressToIndex_External(address);
    if (index == MODBUS_INVALID_INDEX || index >= REGISTER_STORAGE_WORDS) {
        return false;
    }

    *value = modbus.regs[index];
    return true;
}

static bool WriteRawRegister(uint16_t address, uint16_t value)
{
    uint16_t index = ModBus_AddressToIndex_External(address);

    if (index == MODBUS_INVALID_INDEX || index >= REGISTER_STORAGE_WORDS) {
        return false;
    }

    modbus.regs[index] = value;
    return true;
}

static void FloatToWords(float value, uint16_t *high_word, uint16_t *low_word)
{
    uint32_t raw;

    memcpy(&raw, &value, sizeof(raw));
    *high_word = (uint16_t)(raw >> 16);
    *low_word = (uint16_t)raw;
}

static float WordsToFloat(uint16_t high_word, uint16_t low_word)
{
    uint32_t raw = ((uint32_t)high_word << 16) | low_word;
    float value;

    memcpy(&value, &raw, sizeof(value));
    return value;
}

static bool FloatValueIsValid(uint16_t address, float value)
{
    if (!isfinite(value)) {
        return false;
    }

    switch (address) {
        case MB_ADDR_POLL_PERIOD:
            return value >= 100.0f && value <= 60000.0f;
        case MB_ADDR_MATERIAL_WAVE_SPEED:
            return value >= 1000.0f && value <= 10000.0f;
        case MB_ADDR_WAVEGUIDE_LEN:
            return value >= 0.1f && value <= 50.0f;
        case MB_ADDR_DEVICE_ADDR:
            return value >= 1.0f && value <= 247.0f;
        case MB_ADDR_BAUD_RATE:
            return value >= 1200.0f && value <= 65535.0f;
        case MB_ADDR_PARITY:
            return value >= 0.0f && value <= 2.0f;
        case MB_ADDR_TANK_GEOM:
            return value >= 0.0f && value <= 3.0f;
        case MB_ADDR_DAMPING_TIME:
            return value >= 0.0f && value <= 60000.0f;
        default:
            return value >= -1000000000.0f && value <= 1000000000.0f;
    }
}

static bool U16ValueIsValid(uint16_t address, uint16_t value)
{
    switch (address) {
        case MB_ADDR_MB_ADDR_SET:
            return value >= 1U && value <= 247U;
        case MB_ADDR_MB_BAUD_SET:
            return value >= 1200U;
        case MB_ADDR_MB_PARITY_SET:
            return value <= 2U;
        default:
            return true;
    }
}

static bool DescriptorIsPersistent(const ModBus_Descriptor_t *descriptor)
{
    return descriptor != NULL &&
           (descriptor->flags & PARAM_FLAG_PERSISTENT) != 0U;
}

static void MarkPersistentDirty(const ModBus_Descriptor_t *descriptor)
{
    if (DescriptorIsPersistent(descriptor)) {
        storage_dirty = true;
        storage_dirty_since = HAL_GetTick();
        storage_revision++;
    }
}

float ModBus_GetParameter_Float(uint16_t address)
{
    uint16_t descriptor_index;
    uint16_t live_words[2];
    const ModBus_Descriptor_t *descriptor;

    if (address >= MEASUREMENT_SNAPSHOT_FIRST_ADDRESS &&
        address < MEASUREMENT_SNAPSHOT_LAST_ADDRESS &&
        ((address - MEASUREMENT_SNAPSHOT_FIRST_ADDRESS) & 1U) == 0U &&
        MeasurementSnapshot_ReadRange(address, 2U, live_words)) {
        return WordsToFloat(live_words[0], live_words[1]);
    }

    descriptor = FindDescriptorByStart(address, &descriptor_index);
    if (descriptor == NULL || descriptor->type != MODBUS_REGISTER_FLOAT32) {
        return NAN;
    }

    return WordsToFloat(modbus.regs[StorageIndex(descriptor_index, 0U)],
                        modbus.regs[StorageIndex(descriptor_index, 1U)]);
}

void ModBus_SetParameter_Float(uint16_t address, float value)
{
    uint16_t descriptor_index;
    uint16_t high_word;
    uint16_t low_word;
    const ModBus_Descriptor_t *descriptor =
        FindDescriptorByStart(address, &descriptor_index);

    if (descriptor == NULL || descriptor->type != MODBUS_REGISTER_FLOAT32) {
        return;
    }

    FloatToWords(value, &high_word, &low_word);
    if (modbus.regs[StorageIndex(descriptor_index, 0U)] == high_word &&
        modbus.regs[StorageIndex(descriptor_index, 1U)] == low_word) {
        return;
    }

    modbus.regs[StorageIndex(descriptor_index, 0U)] = high_word;
    modbus.regs[StorageIndex(descriptor_index, 1U)] = low_word;
    MarkPersistentDirty(descriptor);
    SynchronizeCommunicationSettings(address);
}

uint16_t ModBus_GetParameter_Int(uint16_t address)
{
    uint16_t descriptor_index;
    const ModBus_Descriptor_t *descriptor =
        FindDescriptorByStart(address, &descriptor_index);

    if (descriptor == NULL || descriptor->type != MODBUS_REGISTER_UINT16) {
        return 0U;
    }

    return modbus.regs[StorageIndex(descriptor_index, 0U)];
}

void ModBus_SetParameter_Int(uint16_t address, uint16_t value)
{
    uint16_t descriptor_index;
    const ModBus_Descriptor_t *descriptor =
        FindDescriptorByStart(address, &descriptor_index);

    if (descriptor == NULL || descriptor->type != MODBUS_REGISTER_UINT16) {
        return;
    }

    if (modbus.regs[StorageIndex(descriptor_index, 0U)] == value) {
        return;
    }

    modbus.regs[StorageIndex(descriptor_index, 0U)] = value;
    MarkPersistentDirty(descriptor);
    SynchronizeCommunicationSettings(address);
}

/*
 * Формат payload v4:
 *   uint16_t record_count;
 *   records[]:
 *     uint16_t key; bit15=1 для uint16, bit15=0 для float32;
 *     uint16_t value[1 или 2].
 * Адрес хранится в key[14:0], поэтому добавление нового параметра не сдвигает
 * значения остальных параметров в EEPROM.
 */
static uint16_t SerializePersistent(uint8_t *buffer, uint16_t capacity)
{
    uint16_t i;
    uint16_t offset = 2U;
    uint16_t record_count = 0U;

    if (buffer == NULL || capacity < 2U) return 0U;

    for (i = 0U; i < DESCRIPTOR_COUNT; ++i) {
        const ModBus_Descriptor_t *descriptor = &descriptors[i];
        uint16_t key;
        uint8_t words;
        uint8_t word;

        if (!DescriptorIsPersistent(descriptor)) continue;

        words = (descriptor->type == MODBUS_REGISTER_FLOAT32) ? 2U : 1U;
        if ((uint32_t)offset + 2U + (uint32_t)words * 2U > capacity) {
            return 0U;
        }

        key = descriptor->address;
        if (descriptor->type == MODBUS_REGISTER_UINT16) key |= 0x8000U;

        buffer[offset++] = (uint8_t)(key >> 8);
        buffer[offset++] = (uint8_t)key;
        for (word = 0U; word < words; ++word) {
            uint16_t value = modbus.regs[StorageIndex(i, word)];
            buffer[offset++] = (uint8_t)(value >> 8);
            buffer[offset++] = (uint8_t)value;
        }
        record_count++;
    }

    buffer[0] = (uint8_t)(record_count >> 8);
    buffer[1] = (uint8_t)record_count;
    return offset;
}

static bool DeserializePersistent(const uint8_t *buffer, uint16_t size)
{
    uint16_t record_count;
    uint16_t offset = 2U;
    uint16_t record;

    if (buffer == NULL || size < 2U) return false;

    record_count = (uint16_t)(((uint16_t)buffer[0] << 8) | buffer[1]);

    for (record = 0U; record < record_count; ++record) {
        uint16_t key;
        uint16_t address;
        bool stored_u16;
        uint8_t words;
        uint16_t descriptor_index;
        const ModBus_Descriptor_t *descriptor;
        uint16_t word0;
        uint16_t word1 = 0U;

        if ((uint16_t)(size - offset) < 4U) return false;

        key = (uint16_t)(((uint16_t)buffer[offset] << 8) |
                         buffer[offset + 1U]);
        offset = (uint16_t)(offset + 2U);
        stored_u16 = (key & 0x8000U) != 0U;
        address = (uint16_t)(key & 0x7FFFU);
        words = stored_u16 ? 1U : 2U;

        if ((uint32_t)offset + (uint32_t)words * 2U > size) return false;

        word0 = (uint16_t)(((uint16_t)buffer[offset] << 8) |
                           buffer[offset + 1U]);
        offset = (uint16_t)(offset + 2U);
        if (words == 2U) {
            word1 = (uint16_t)(((uint16_t)buffer[offset] << 8) |
                               buffer[offset + 1U]);
            offset = (uint16_t)(offset + 2U);
        }

        descriptor = FindDescriptorByStart(address, &descriptor_index);
        if (descriptor == NULL || !DescriptorIsPersistent(descriptor)) continue;

        if (stored_u16 && descriptor->type == MODBUS_REGISTER_UINT16) {
            if (U16ValueIsValid(address, word0)) {
                modbus.regs[StorageIndex(descriptor_index, 0U)] = word0;
            }
        } else if (!stored_u16 &&
                   descriptor->type == MODBUS_REGISTER_FLOAT32) {
            float value = WordsToFloat(word0, word1);
            if (FloatValueIsValid(address, value)) {
                modbus.regs[StorageIndex(descriptor_index, 0U)] = word0;
                modbus.regs[StorageIndex(descriptor_index, 1U)] = word1;
            }
        }
    }

    return offset == size;
}

static void StorageStartSave(void)
{
    uint16_t size;

    if (!storage_available || !storage_dirty ||
        ParamsStorage_GetSaveState() == PARAMS_SAVE_BUSY) {
        return;
    }

    size = SerializePersistent(persistence_buffer, sizeof(persistence_buffer));
    if (size == 0U) return;

    if (ParamsStorage_BeginSave(persistence_buffer, size) == HAL_OK) {
        storage_save_revision = storage_revision;
        storage_force_requested = false;
    }
}

void ModBus_StorageProcess(bool allow_write)
{
    ParamsStorageSaveState_t state = ParamsStorage_GetSaveState();

    if (state == PARAMS_SAVE_COMPLETE) {
        if (storage_save_revision == storage_revision) {
            storage_dirty = false;
        }
        ParamsStorage_ClearSaveResult();
        state = PARAMS_SAVE_IDLE;
    } else if (state == PARAMS_SAVE_ERROR) {
        ParamsStorage_ClearSaveResult();
        storage_dirty = true;
        storage_dirty_since = HAL_GetTick();
        state = PARAMS_SAVE_IDLE;
    }

    if (!allow_write || !storage_available) return;

    if (state == PARAMS_SAVE_BUSY) {
        (void)ParamsStorage_ProcessSave();
        return;
    }

    if (storage_dirty &&
        (storage_force_requested ||
         (uint32_t)(HAL_GetTick() - storage_dirty_since) >=
            MODBUS_EEPROM_WRITE_DELAY_MS)) {
        StorageStartSave();
        if (ParamsStorage_GetSaveState() == PARAMS_SAVE_BUSY) {
            (void)ParamsStorage_ProcessSave();
        }
    }
}

bool ModBus_StorageIsBusy(void)
{
    return ParamsStorage_GetSaveState() == PARAMS_SAVE_BUSY;
}

static void ApplyDefaults(void)
{
    uint16_t i;

    for (i = 0U;
         i < (uint16_t)(sizeof(float_defaults) / sizeof(float_defaults[0]));
         ++i) {
        ModBus_SetParameter_Float(float_defaults[i].address,
                                  float_defaults[i].value);
    }

    for (i = 0U;
         i < (uint16_t)(sizeof(u16_defaults) / sizeof(u16_defaults[0]));
         ++i) {
        ModBus_SetParameter_Int(u16_defaults[i].address,
                                u16_defaults[i].value);
    }
}

static void WriteFloatDirect(uint16_t address, float value)
{
    uint16_t descriptor_index;
    uint16_t high_word;
    uint16_t low_word;
    const ModBus_Descriptor_t *descriptor =
        FindDescriptorByStart(address, &descriptor_index);

    if (descriptor == NULL || descriptor->type != MODBUS_REGISTER_FLOAT32) {
        return;
    }

    FloatToWords(value, &high_word, &low_word);
    modbus.regs[StorageIndex(descriptor_index, 0U)] = high_word;
    modbus.regs[StorageIndex(descriptor_index, 1U)] = low_word;
}

static void WriteU16Direct(uint16_t address, uint16_t value)
{
    uint16_t descriptor_index;
    const ModBus_Descriptor_t *descriptor =
        FindDescriptorByStart(address, &descriptor_index);

    if (descriptor != NULL && descriptor->type == MODBUS_REGISTER_UINT16) {
        modbus.regs[StorageIndex(descriptor_index, 0U)] = value;
    }
}

static uint8_t ClampDeviceAddress(float value)
{
    uint16_t address;

    if (!isfinite(value)) {
        return MODBUS_DEFAULT_ADDRESS;
    }

    address = (uint16_t)(value + 0.5f);
    if (address < 1U || address > 247U) {
        return MODBUS_DEFAULT_ADDRESS;
    }

    return (uint8_t)address;
}

static void SynchronizeCommunicationSettings(uint16_t touched_address)
{
    if (touched_address == MB_ADDR_MB_ADDR_SET) {
        uint16_t value = ModBus_GetParameter_Int(MB_ADDR_MB_ADDR_SET);
        if (value < 1U || value > 247U) {
            value = MODBUS_DEFAULT_ADDRESS;
            WriteU16Direct(MB_ADDR_MB_ADDR_SET, value);
        }
        WriteFloatDirect(MB_ADDR_DEVICE_ADDR, (float)value);
        modbus.device_address = (uint8_t)value;
    } else if (touched_address == MB_ADDR_DEVICE_ADDR) {
        uint8_t value = ClampDeviceAddress(
            ModBus_GetParameter_Float(MB_ADDR_DEVICE_ADDR));
        WriteU16Direct(MB_ADDR_MB_ADDR_SET, value);
        WriteFloatDirect(MB_ADDR_DEVICE_ADDR, (float)value);
        modbus.device_address = value;
    } else if (touched_address == MB_ADDR_MB_BAUD_SET) {
        WriteFloatDirect(MB_ADDR_BAUD_RATE,
                         (float)ModBus_GetParameter_Int(MB_ADDR_MB_BAUD_SET));
    } else if (touched_address == MB_ADDR_BAUD_RATE) {
        float baud = ModBus_GetParameter_Float(MB_ADDR_BAUD_RATE);
        if (isfinite(baud) && baud >= 1200.0f && baud <= 65535.0f) {
            WriteU16Direct(MB_ADDR_MB_BAUD_SET, (uint16_t)(baud + 0.5f));
        }
    } else if (touched_address == MB_ADDR_MB_PARITY_SET) {
        WriteFloatDirect(MB_ADDR_PARITY,
                         (float)ModBus_GetParameter_Int(MB_ADDR_MB_PARITY_SET));
    } else if (touched_address == MB_ADDR_PARITY) {
        float parity = ModBus_GetParameter_Float(MB_ADDR_PARITY);
        if (isfinite(parity) && parity >= 0.0f && parity <= 2.0f) {
            WriteU16Direct(MB_ADDR_MB_PARITY_SET,
                           (uint16_t)(parity + 0.5f));
        }
    }
}

static void AutoWriteSerialNumber(void)
{
    uint16_t serial_hi = ModBus_GetParameter_Int(MB_ADDR_SERIAL_HI);
    uint16_t serial_lo = ModBus_GetParameter_Int(MB_ADDR_SERIAL_LO);

    if (serial_hi == 0U && serial_lo == 0U) {
        volatile const uint32_t *uid =
            (volatile const uint32_t *)0x1FFFF7E8UL;
        WriteU16Direct(MB_ADDR_SERIAL_HI, (uint16_t)(uid[0] >> 16));
        WriteU16Direct(MB_ADDR_SERIAL_LO, (uint16_t)uid[0]);
        storage_dirty = true;
        storage_dirty_since = HAL_GetTick();
        storage_revision++;
    }
}

uint16_t ModBus_CRC16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;

    if (data == NULL) {
        return crc;
    }

    while (length-- > 0U) {
        uint8_t bit;
        crc ^= *data++;
        for (bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 1U) ?
                (uint16_t)((crc >> 1) ^ 0xA001U) :
                (uint16_t)(crc >> 1);
        }
    }

    return crc;
}

static void SendFrame(uint8_t *frame, uint16_t payload_length)
{
    uint16_t crc;

    if (request_is_broadcast || frame == NULL ||
        payload_length > (MODBUS_BUFFER_SIZE - 2U)) {
        return;
    }

    crc = ModBus_CRC16(frame, payload_length);
    frame[payload_length] = (uint8_t)crc;
    frame[payload_length + 1U] = (uint8_t)(crc >> 8);

    /* Ответ Modbus имеет высший приоритет. Пока DE активен, возможное
     * аппаратное эхо USART1 игнорируется и не попадает в следующий запрос. */
    __disable_irq();
    modbus.tx_busy = 1U;
    modbus.rx_length = 0U;
    __enable_irq();

    (void)RS485_Transmit(&huart1,
                         frame,
                         (uint16_t)(payload_length + 2U),
                         MODBUS_TX_TIMEOUT_MS);

    __disable_irq();
    modbus.rx_length = 0U;
    modbus.last_byte_tick = HAL_GetTick();
    modbus.tx_busy = 0U;
    __enable_irq();
}

static void SendException(uint8_t function, uint8_t exception_code)
{
    uint8_t response[5];

    response[0] = modbus.device_address;
    response[1] = (uint8_t)(function | 0x80U);
    response[2] = exception_code;
    SendFrame(response, 3U);
}

static bool ReadRangeIsValid(uint16_t start_address, uint16_t count)
{
    uint16_t i;

    if (count == 0U) {
        return false;
    }

    for (i = 0U; i < count; ++i) {
        uint32_t address = (uint32_t)start_address + i;
        if (address > 0xFFFFUL ||
            FindDescriptorByWord((uint16_t)address, NULL, NULL) == NULL) {
            return false;
        }
    }

    return true;
}

static bool WriteRangeIsAtomic(uint16_t start_address, uint16_t count)
{
    uint32_t end_address = (uint32_t)start_address + count - 1U;
    uint16_t i;

    if (!ReadRangeIsValid(start_address, count)) {
        return false;
    }

    for (i = 0U; i < DESCRIPTOR_COUNT; ++i) {
        const ModBus_Descriptor_t *descriptor = &descriptors[i];

        if (descriptor->type == MODBUS_REGISTER_FLOAT32) {
            uint32_t float_start = descriptor->address;
            uint32_t float_end = float_start + 1U;
            bool touched = !(end_address < float_start ||
                             start_address > float_end);
            bool fully_covered = start_address <= float_start &&
                                 end_address >= float_end;
            if (touched && !fully_covered) {
                return false;
            }
        }
    }

    return true;
}

static bool ValidateWriteValues(uint16_t start_address,
                                uint16_t count,
                                const uint8_t *data)
{
    uint32_t request_end = (uint32_t)start_address + count - 1U;
    uint16_t i;

    if (data == NULL) return false;

    for (i = 0U; i < DESCRIPTOR_COUNT; ++i) {
        const ModBus_Descriptor_t *descriptor = &descriptors[i];
        uint32_t descriptor_end = descriptor->address +
            ((descriptor->type == MODBUS_REGISTER_FLOAT32) ? 1U : 0U);

        if (descriptor->address > request_end || descriptor_end < start_address) {
            continue;
        }

        if (descriptor->type == MODBUS_REGISTER_UINT16) {
            uint16_t offset = (uint16_t)(descriptor->address - start_address);
            uint16_t value = (uint16_t)(((uint16_t)data[offset * 2U] << 8) |
                                        data[offset * 2U + 1U]);
            if (!U16ValueIsValid(descriptor->address, value)) return false;
        } else {
            uint16_t offset = (uint16_t)(descriptor->address - start_address);
            uint16_t high_word =
                (uint16_t)(((uint16_t)data[offset * 2U] << 8) |
                           data[offset * 2U + 1U]);
            uint16_t low_word =
                (uint16_t)(((uint16_t)data[(offset + 1U) * 2U] << 8) |
                           data[(offset + 1U) * 2U + 1U]);
            if (!FloatValueIsValid(descriptor->address,
                                   WordsToFloat(high_word, low_word))) {
                return false;
            }
        }
    }

    return true;
}

static void ProcessRead(uint8_t function,
                        uint16_t start_address,
                        uint16_t register_count)
{
    uint8_t response[MODBUS_BUFFER_SIZE];
    uint16_t live_words[MEASUREMENT_SNAPSHOT_WORD_COUNT];
    uint16_t i;
    uint16_t index = 0U;
    bool fast_snapshot;

    if (register_count == 0U ||
        register_count > MODBUS_MAX_READ_REGISTERS) {
        SendException(function, 0x03U);
        return;
    }

    /* Быстрый путь проверяется раньше общей карты, поэтому чтение уровня
     * не выполняет даже линейный поиск descriptor. */
    fast_snapshot = MeasurementSnapshot_ReadRange(start_address,
                                                   register_count,
                                                   live_words);

    if (!fast_snapshot &&
        !ReadRangeIsValid(start_address, register_count)) {
        SendException(function, 0x02U);
        return;
    }

    response[index++] = modbus.device_address;
    response[index++] = function;
    response[index++] = (uint8_t)(register_count * 2U);

    for (i = 0U; i < register_count; ++i) {
        uint16_t value = 0U;
        if (fast_snapshot) {
            value = live_words[i];
        } else {
            (void)ModBus_ReadRawRegister((uint16_t)(start_address + i),
                                         &value);
        }
        response[index++] = (uint8_t)(value >> 8);
        response[index++] = (uint8_t)value;
    }

    SendFrame(response, index);
}

static void MarkDescriptorDirtyByWord(uint16_t address)
{
    const ModBus_Descriptor_t *descriptor =
        FindDescriptorByWord(address, NULL, NULL);
    MarkPersistentDirty(descriptor);
}

static void ProcessWriteSingle(uint16_t address, uint16_t value)
{
    const ModBus_Descriptor_t *descriptor =
        FindDescriptorByWord(address, NULL, NULL);
    uint8_t response[8];
    uint8_t old_address = modbus.device_address;

    if (descriptor == NULL) {
        SendException(0x06U, 0x02U);
        return;
    }

    /* Float32 нельзя менять половиной через функцию 0x06. */
    if (descriptor->type == MODBUS_REGISTER_FLOAT32) {
        SendException(0x06U, 0x03U);
        return;
    }

    if (!U16ValueIsValid(descriptor->address, value)) {
        SendException(0x06U, 0x03U);
        return;
    }

    if (!WriteRawRegister(address, value)) {
        SendException(0x06U, 0x04U);
        return;
    }

    MarkDescriptorDirtyByWord(address);
    SynchronizeCommunicationSettings(descriptor->address);

    response[0] = old_address;
    response[1] = 0x06U;
    response[2] = (uint8_t)(address >> 8);
    response[3] = (uint8_t)address;
    response[4] = (uint8_t)(value >> 8);
    response[5] = (uint8_t)value;
    SendFrame(response, 6U);
}

static void ProcessWriteMultiple(uint16_t start_address,
                                 uint16_t register_count,
                                 const uint8_t *data)
{
    uint8_t response[8];
    uint16_t i;
    uint8_t old_address = modbus.device_address;

    if (register_count == 0U ||
        register_count > MODBUS_MAX_WRITE_REGISTERS) {
        SendException(0x10U, 0x03U);
        return;
    }

    if (data == NULL) {
        SendException(0x10U, 0x03U);
        return;
    }

    if (!ReadRangeIsValid(start_address, register_count)) {
        SendException(0x10U, 0x02U);
        return;
    }

    if (!WriteRangeIsAtomic(start_address, register_count) ||
        !ValidateWriteValues(start_address, register_count, data)) {
        SendException(0x10U, 0x03U);
        return;
    }

    for (i = 0U; i < register_count; ++i) {
        uint16_t address = (uint16_t)(start_address + i);
        uint16_t value = (uint16_t)(((uint16_t)data[i * 2U] << 8) |
                                    data[i * 2U + 1U]);
        if (!WriteRawRegister(address, value)) {
            SendException(0x10U, 0x04U);
            return;
        }
    }

    for (i = 0U; i < DESCRIPTOR_COUNT; ++i) {
        uint32_t descriptor_end = descriptors[i].address +
            ((descriptors[i].type == MODBUS_REGISTER_FLOAT32) ? 1U : 0U);
        uint32_t request_end = (uint32_t)start_address + register_count - 1U;
        if (descriptors[i].address <= request_end &&
            descriptor_end >= start_address) {
            MarkPersistentDirty(&descriptors[i]);
            SynchronizeCommunicationSettings(descriptors[i].address);
        }
    }

    response[0] = old_address;
    response[1] = 0x10U;
    response[2] = (uint8_t)(start_address >> 8);
    response[3] = (uint8_t)start_address;
    response[4] = (uint8_t)(register_count >> 8);
    response[5] = (uint8_t)register_count;
    SendFrame(response, 6U);
}

static void ProcessFrame(const uint8_t *frame, uint16_t length)
{
    uint16_t received_crc;
    uint16_t calculated_crc;
    uint8_t destination;
    uint8_t function;

    if (frame == NULL || length < 4U) {
        return;
    }

    received_crc = (uint16_t)(((uint16_t)frame[length - 1U] << 8) |
                              frame[length - 2U]);
    calculated_crc = ModBus_CRC16(frame, (uint16_t)(length - 2U));
    if (received_crc != calculated_crc) {
        return;
    }

    destination = frame[0];
    if (destination != modbus.device_address && destination != 0U) {
        return;
    }

    request_is_broadcast = (destination == 0U);
    function = frame[1];

    if (request_is_broadcast && function != 0x06U && function != 0x10U) {
        request_is_broadcast = false;
        return;
    }

    switch (function) {
        case 0x03U:
        case 0x04U:
            if (length != 8U) {
                SendException(function, 0x03U);
            } else {
                uint16_t start = (uint16_t)(((uint16_t)frame[2] << 8) | frame[3]);
                uint16_t count = (uint16_t)(((uint16_t)frame[4] << 8) | frame[5]);
                ProcessRead(function, start, count);
            }
            break;

        case 0x06U:
            if (length != 8U) {
                SendException(function, 0x03U);
            } else {
                uint16_t address = (uint16_t)(((uint16_t)frame[2] << 8) | frame[3]);
                uint16_t value = (uint16_t)(((uint16_t)frame[4] << 8) | frame[5]);
                ProcessWriteSingle(address, value);
            }
            break;

        case 0x10U:
            if (length < 9U) {
                SendException(function, 0x03U);
            } else {
                uint16_t start = (uint16_t)(((uint16_t)frame[2] << 8) | frame[3]);
                uint16_t count = (uint16_t)(((uint16_t)frame[4] << 8) | frame[5]);
                uint8_t byte_count = frame[6];
                if (byte_count != (uint8_t)(count * 2U) ||
                    length != (uint16_t)(9U + byte_count)) {
                    SendException(function, 0x03U);
                } else {
                    ProcessWriteMultiple(start, count, &frame[7]);
                }
            }
            break;

        default:
            SendException(function, 0x01U);
            break;
    }

    request_is_broadcast = false;
}

void ModBus_Init(void)
{
    ParamsStorageState_t storage_state;
    uint16_t payload_size = 0U;
    bool loaded = false;

    memset(&modbus, 0, sizeof(modbus));
    MeasurementSnapshot_Init();
    modbus.device_address = MODBUS_DEFAULT_ADDRESS;
    modbus.last_byte_tick = HAL_GetTick();

    RS485_Init();
    storage_state = ParamsStorage_Init();
    storage_available = (storage_state != PARAMS_STORAGE_NOT_AVAILABLE);

    /* Сначала безопасные значения: отсутствующие записи старой схемы не дадут NaN. */
    ApplyDefaults();
    storage_dirty = false;
    storage_force_requested = false;
    storage_revision = 0U;

    if (storage_state == PARAMS_STORAGE_VALID &&
        ParamsStorage_Load(persistence_buffer,
                           sizeof(persistence_buffer),
                           &payload_size) == HAL_OK) {
        loaded = DeserializePersistent(persistence_buffer, payload_size);
    }

    if (!loaded && storage_available) {
        /* Старый/поврежденный формат не используется; перезапись только в фоне. */
        storage_dirty = true;
        storage_dirty_since = HAL_GetTick();
        storage_revision++;
    }

    AutoWriteSerialNumber();
    SynchronizeCommunicationSettings(MB_ADDR_MB_ADDR_SET);
    SynchronizeCommunicationSettings(MB_ADDR_MB_BAUD_SET);
    SynchronizeCommunicationSettings(MB_ADDR_MB_PARITY_SET);

    (void)HAL_UART_Receive_IT(&huart1,
                              (uint8_t *)&modbus.rx_byte,
                              1U);
}

uint8_t ModBus_GetDeviceAddress(void)
{
    return modbus.device_address;
}

bool ModBus_CommunicationIsBusy(void)
{
    /* Все поля aligned и атомарно читаются на Cortex-M3. Здесь нельзя
     * запрещать IRQ: функция вызывается также из обработчика USART2. */
    return modbus.rx_length > 0U ||
           modbus.processing != 0U ||
           modbus.tx_busy != 0U;
}

void ModBus_RxCallback(UART_HandleTypeDef *huart)
{
    uint32_t now;

    if (huart == NULL || huart->Instance != USART1) {
        return;
    }

    /* При передаче собственный ответ может возвращаться на RX. */
    if (modbus.tx_busy != 0U) {
        modbus.rx_length = 0U;
        (void)HAL_UART_Receive_IT(&huart1,
                                  (uint8_t *)&modbus.rx_byte,
                                  1U);
        return;
    }

    now = HAL_GetTick();
    if ((uint32_t)(now - modbus.last_byte_tick) >
        MODBUS_INTERBYTE_RESET_MS) {
        modbus.rx_length = 0U;
    }

    modbus.last_byte_tick = now;

    if (modbus.rx_length < MODBUS_BUFFER_SIZE) {
        modbus.rx_buffer[modbus.rx_length++] = modbus.rx_byte;
    } else {
        modbus.rx_length = 0U;
    }

    (void)HAL_UART_Receive_IT(&huart1,
                              (uint8_t *)&modbus.rx_byte,
                              1U);
}

void ModBus_RestartRx(void)
{
    modbus.rx_length = 0U;
    (void)HAL_UART_Receive_IT(&huart1,
                              (uint8_t *)&modbus.rx_byte,
                              1U);
}

static uint16_t ExpectedRequestLength(const uint8_t *buffer,
                                      uint16_t received_length)
{
    if (buffer == NULL || received_length < 2U) {
        return 0U;
    }

    switch (buffer[1]) {
        case 0x03U:
        case 0x04U:
        case 0x06U:
            return 8U;

        case 0x10U:
            if (received_length >= 7U) {
                return (uint16_t)(9U + buffer[6]);
            }
            return 0U;

        default:
            /* Для неподдерживаемой стандартной команды достаточно 8 байт,
             * чтобы сразу вернуть Illegal Function. */
            return 8U;
    }
}

void ModBus_Process(void)
{
    uint16_t frame_length = 0U;
    uint16_t expected_length;
    uint16_t current_length;
    bool complete_by_length;
    bool complete_by_silence;

    current_length = modbus.rx_length;
    expected_length = ExpectedRequestLength(modbus.rx_buffer, current_length);
    complete_by_length = expected_length > 0U &&
                         current_length >= expected_length;
    complete_by_silence = current_length > 0U &&
        (uint32_t)(HAL_GetTick() - modbus.last_byte_tick) >
            MODBUS_FRAME_END_TIMEOUT_MS;

    if (!complete_by_length && !complete_by_silence) {
        return;
    }

    __disable_irq();
    current_length = modbus.rx_length;
    expected_length = ExpectedRequestLength(modbus.rx_buffer, current_length);

    if (expected_length > 0U && current_length >= expected_length) {
        frame_length = expected_length;
    } else {
        frame_length = current_length;
    }

    if (frame_length == 0U || frame_length > MODBUS_BUFFER_SIZE) {
        frame_length = 0U;
    } else {
        memcpy(modbus.process_buffer, modbus.rx_buffer, frame_length);
    }

    modbus.processing = (frame_length > 0U) ? 1U : 0U;
    modbus.rx_length = 0U;
    __enable_irq();

    if (frame_length > 0U) {
        ProcessFrame(modbus.process_buffer, frame_length);
        modbus.processing = 0U;
    }
}

float ModBus_GetWaveguideLength(void)
{
    float value = ModBus_GetParameter_Float(MB_ADDR_WAVEGUIDE_LEN);
    return (isfinite(value) && value >= 0.1f && value <= 50.0f) ?
        value : 1.2f;
}

void ModBus_SetWaveguideLength(float length_m)
{
    if (isfinite(length_m) && length_m >= 0.1f && length_m <= 50.0f) {
        ModBus_SetParameter_Float(MB_ADDR_WAVEGUIDE_LEN, length_m);
    }
}

float ModBus_GetMaterialWaveSpeed(void)
{
    float value = ModBus_GetParameter_Float(MB_ADDR_MATERIAL_WAVE_SPEED);
    return (isfinite(value) && value >= 1000.0f && value <= 10000.0f) ?
        value : 3370.0f;
}

void ModBus_SetMaterialWaveSpeed(float speed_mps)
{
    if (isfinite(speed_mps) && speed_mps >= 1000.0f && speed_mps <= 10000.0f) {
        ModBus_SetParameter_Float(MB_ADDR_MATERIAL_WAVE_SPEED, speed_mps);
    }
}

void ModBus_SetTemperature(float temperature)
{
    ModBus_SetParameter_Float(MB_ADDR_TEMP, temperature);
}

float ModBus_GetTemperature(void)
{
    return ModBus_GetParameter_Float(MB_ADDR_TEMP);
}

uint32_t ModBus_GetPulseWidthIterations(void)
{
    return 10U;
}

void ModBus_PublishLiveMeasurements(float level_mm,
                                    float temperature_c,
                                    float percent,
                                    float volume_m3,
                                    uint16_t status)
{
    /* Публикация снимка выполняется первой: Modbus уже может вернуть новые
     * данные, даже если дальнейшее обновление общей карты ещё не закончено. */
    MeasurementSnapshot_Publish(level_mm,
                                temperature_c,
                                percent,
                                volume_m3,
                                status,
                                HAL_GetTick());

    WriteFloatDirect(MB_ADDR_LEVEL, level_mm);
    WriteFloatDirect(MB_ADDR_TEMP, temperature_c);
    WriteFloatDirect(MB_ADDR_PERCENT, percent);
    WriteFloatDirect(MB_ADDR_VOLUME, volume_m3);
}

void ModBus_UpdateMeasurements(float level,
                               float temperature,
                               float waveguide)
{
    float percent = 0.0f;
    float volume = ModBus_GetParameter_Float(MB_ADDR_VOLUME);

    if (isfinite(waveguide) && waveguide > 0.0f) {
        percent = level / waveguide * 100.0f;
    }
    if (!isfinite(volume)) {
        volume = 0.0f;
    }

    ModBus_PublishLiveMeasurements(level,
                                   temperature,
                                   percent,
                                   volume,
                                   1U);
}

void ModBus_UpdateVoltages(float vdda, float v24, float v12, float v5)
{
    /* В текущей зарезервированной карте для напряжений адресов нет. */
    diagnostic_vdda = vdda;
    diagnostic_v24 = v24;
    diagnostic_v12 = v12;
    diagnostic_v5 = v5;
    (void)diagnostic_vdda;
    (void)diagnostic_v24;
    (void)diagnostic_v12;
    (void)diagnostic_v5;
}

void ModBus_UpdateFirmwareVersion(uint16_t version)
{
    WriteU16Direct(MB_ADDR_FW_VERSION, version);
}

void ModBus_ForceSaveToEEPROM(void)
{
    if (storage_dirty) {
        storage_force_requested = true;
    }
}
