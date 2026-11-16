#include "graduation.h"
#include "AT24C64.h"
#include "modbus.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static uint8_t eeprom[8192];
static uint16_t command_status;
static unsigned writes;
static bool oil_drum_selected;

HAL_StatusTypeDef AT24C64_ReadBytes(uint8_t device, uint16_t address,
                                   uint8_t *data, uint16_t size)
{
    assert(device == AT24C64_DEFAULT_ADDRESS);
    assert((size_t)address + size <= sizeof(eeprom));
    if (size > 256U) return HAL_TIMEOUT;
    memcpy(data, eeprom + address, size);
    return HAL_OK;
}

HAL_StatusTypeDef AT24C64_WriteBytes(uint8_t device, uint16_t address,
                                    const uint8_t *data, uint16_t size)
{
    assert(device == AT24C64_DEFAULT_ADDRESS);
    assert(address >= GRAD_EEPROM_BASE && (size_t)address + size <= sizeof(eeprom));
    memcpy(eeprom + address, data, size);
    ++writes;
    return HAL_OK;
}

void ModBus_SetParameter_Float(uint16_t address, float value)
{
    assert(address == MB_ADDR_CALIB_POINTS);
    assert(value == Grad_GetState()->actual_points);
}

float ModBus_GetParameter_Float(uint16_t address)
{
    switch (address) {
        case MB_ADDR_TANK_GEOM:
            return oil_drum_selected ? (float)GRAD_TYPE_BY_TABLE : (float)GRAD_TYPE_VERTICAL;
        case MB_ADDR_TANK_HEIGHT:
            return GRAD_OIL_DRUM_HEIGHT_M;
        case MB_ADDR_TANK_VOLUME:
            return GRAD_OIL_DRUM_VOLUME_M3;
        default:
            assert(false);
            return NAN;
    }
}

void ModBus_SetParameter_Int(uint16_t address, uint16_t value)
{
    assert(address == MB_ADDR_COMMAND);
    command_status = value;
}

int main(void)
{
    memset(eeprom, 0xFF, sizeof(eeprom));
    Grad_Init();
    assert(!Grad_IsValid());

    oil_drum_selected = true;
    Grad_Init();
    assert(Grad_IsValid() && writes == 0U);
    assert(Grad_GetState()->actual_points == 83U);
    assert(Grad_InterpolateVolume(0.0f) == 0.0f);
    assert(fabsf(Grad_InterpolateVolume(0.70f) - 0.1798788f) < 0.000001f);
    assert(fabsf(Grad_InterpolateVolume(GRAD_OIL_DRUM_HEIGHT_M) -
                 GRAD_OIL_DRUM_VOLUME_M3) < 0.000001f);

    GradHeader_t header = { .points_count = 3U, .start_height_m = 0.25f };
    const float points[] = { 1.0f, 3.0f, 9.0f };
    assert(Grad_WriteHeader(&header) == HAL_OK);
    Grad_ProcessCommand(301U, 0.0f);
    assert(command_status == 0U && writes == 0U);
    header.step_height_m = 0.125f;
    assert(Grad_WriteHeader(&header) == HAL_OK);
    assert(Grad_WriteVolumes(0U, points, 3U) == HAL_OK);
    Grad_ProcessCommand(301U, 0.0f);
    assert(command_status == 90U);
    Grad_Init();
    assert(Grad_IsValid());
    assert(Grad_CalculateVolume(GRAD_TYPE_BY_TABLE, 0.4375f, 0.0f, 0.0f) == 6.0f);
    assert(Grad_InterpolateVolume(0.0f) == 1.0f);
    assert(Grad_InterpolateVolume(0.375f) == 3.0f);
    assert(Grad_InterpolateVolume(0.5f) == 9.0f);
    assert(Grad_InterpolateVolume(8192.0f) == 9.0f);
    assert(isnan(Grad_InterpolateVolume(NAN)));

    header.points_count = GRAD_MAX_POINTS;
    header.start_height_m = 0.0f;
    assert(Grad_WriteHeader(&header) == HAL_OK);
    for (unsigned i = 0; i < GRAD_MAX_POINTS; ++i) {
        Grad_GetState()->volumes[i] = (float)i;
    }
    assert(Grad_SaveToEEPROM() == HAL_OK);
    Grad_Init();
    assert(Grad_IsValid());
    assert(Grad_GetState()->actual_points == GRAD_MAX_POINTS);
    assert(Grad_InterpolateVolume(187.375f) == 1499.0f);

    oil_drum_selected = false;
    eeprom[GRAD_DATA_START + 300U] ^= 1U;
    Grad_Init();
    assert(!Grad_IsValid());
    assert(isnan(Grad_InterpolateVolume(1.0f)));
    puts("PASS: table interpolation, bounds, invalid header, 1500-point EEPROM reload and CRC");
}
