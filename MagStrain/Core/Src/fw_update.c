#include "fw_update.h"
#include "AT24C64.h"
#include <stddef.h>
#include <string.h>
#if defined(FW_EEPROM_DIAGNOSTICS) && !defined(FW_BOOTLOADER)
#include "utils.h"
#endif
_Static_assert(sizeof(FW_Header) == 28U, "EEPROM header layout changed");
_Static_assert(FW_IMAGE_BASE + FW_APP_SIZE <= 65536UL, "Image exceeds EEPROM");

uint32_t FW_Crc(uint32_t crc, const uint8_t *data, uint32_t size)
{
    while (size--) {
        crc ^= *data++;
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ ((crc & 1U) ? 0xEDB88320UL : 0U);
    }
    return crc;
}
bool FW_VectorsValid(uint32_t sp, uint32_t pc, uint32_t size)
{
    return sp > 0x20000000UL && sp <= 0x20005000UL && (sp & 7U) == 0U &&
           (pc & 1U) && (pc & ~1UL) >= FW_APP_BASE &&
           (pc & ~1UL) < FW_APP_BASE + size;
}
bool FW_HeaderValid(const FW_Header *h)
{
    return h->magic == FW_MAGIC && h->size >= 8U && h->size <= FW_APP_SIZE &&
           h->app_base == FW_APP_BASE && h->target == FW_TARGET &&
           h->header_crc == (FW_Crc(0xFFFFFFFFUL, (const uint8_t *)&h->size, 20U) ^ 0xFFFFFFFFUL);
}
#ifndef FW_BOOTLOADER
static FW_State state;
static FW_Header header;
static uint32_t received, checked, crc;
static uint16_t error;
static uint16_t Word(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }
static uint32_t DWord(const uint8_t *p) { return Word(p) | ((uint32_t)Word(p + 2) << 16); }
#ifdef FW_EEPROM_DIAGNOSTICS
static void LogEepromFailure(const char *operation, uint16_t address,
                             uint32_t observed, uint32_t expected)
{
    USART2_Print("[FWUP EEPROM] ");
    USART2_Print(operation);
    USART2_Print(" addr=0x");
    USART2_PrintHexByte((uint8_t)(address >> 8));
    USART2_PrintHexByte((uint8_t)address);
    USART2_Print(" observed=");
    USART2_PrintNum(observed);
    USART2_Print(" expected=");
    USART2_PrintNum(expected);
    USART2_Print("\r\n");
}
#else
#define LogEepromFailure(...) ((void)0)
#endif
static bool Read(uint16_t a, void *p, uint16_t n)
{
    HAL_StatusTypeDef status = AT24C64_ReadBytes(AT24C64_DEFAULT_ADDRESS, a, p, n);
    if (status != HAL_OK) LogEepromFailure("read HAL", a, status, HAL_OK);
    return status == HAL_OK;
}
static bool Write(uint16_t a, const void *p, uint16_t n)
{
    HAL_StatusTypeDef status = AT24C64_WriteBytes(AT24C64_DEFAULT_ADDRESS, a, p, n);
    if (status != HAL_OK) LogEepromFailure("write HAL", a, status, HAL_OK);
    return status == HAL_OK;
}
/* Probe one reserved byte, restoring it before accepting image data.
 * Its aliases on 8/16/32 KiB EEPROMs lie outside the legacy parameters/table. */
static uint8_t ProbeStorage(void)
{
    static const uint16_t aliases[] = {0x1F80U, 0x3F80U, 0x7F80U};
    const uint16_t probe_address = 0xFF80U;
    uint8_t saved, previous[sizeof(aliases) / sizeof(aliases[0])], value;
    if (!Read(probe_address, &saved, 1U)) return 1U;
    for (unsigned i = 0; i < sizeof(previous); ++i) {
        if (!Read(aliases[i], &previous[i], 1U)) return 1U;
    }
    uint8_t pattern = (uint8_t)~saved;
    uint8_t result = 0U;
    if (!Write(probe_address, &pattern, 1U) || !Read(probe_address, &value, 1U)) {
        result = 1U;
    } else if (value != pattern) {
        LogEepromFailure("probe mismatch", probe_address, value, pattern);
        result = 1U;
    } else {
        for (unsigned i = 0; i < sizeof(previous); ++i) {
            if (!Read(aliases[i], &value, 1U)) { result = 1U; break; }
            if (value != previous[i]) {
                LogEepromFailure("alias changed", aliases[i], value, previous[i]);
                result = 4U;
                break;
            }
        }
    }
    if (!Write(probe_address, &saved, 1U) || !Read(probe_address, &value, 1U)) return 1U;
    if (value != saved) {
        LogEepromFailure("restore mismatch", probe_address, value, saved);
        return 1U;
    }
    return result;
}

static uint8_t Fail(uint16_t why)
{ error = why; state = FW_ERROR; return 4U; }
bool FW_Active(void) { return state >= FW_RECEIVING && state <= FW_READY; }
uint16_t FW_Read(uint16_t address)
{
    switch (address - FW_MB_FIRST) {
    case 0: return (uint16_t)state;
    case 1: return error;
    case 2: return (uint16_t)received;
    case 3: return (uint16_t)(received >> 16);
    case 4: return (uint16_t)header.size;
    case 5: return (uint16_t)(header.size >> 16);
    case 6: return (uint16_t)checked;
    case 7: return (uint16_t)(checked >> 16);
    case 8: return (uint16_t)FW_APP_SIZE;
    case 9: return 1U; /* Protocol version. */
    default: return 0U;
    }
}
void FW_Process(void)
{
    uint8_t data[FW_CHUNK];
    if (state != FW_VERIFYING) return;
    uint16_t n = (header.size - checked > FW_CHUNK) ? FW_CHUNK : (uint16_t)(header.size - checked);
    if (!Read((uint16_t)(FW_IMAGE_BASE + checked), data, n)) { (void)Fail(1U); return; }
    if (checked == 0U) {
        uint32_t vectors[2];
        memcpy(vectors, data, sizeof(vectors));
        if (!FW_VectorsValid(vectors[0], vectors[1], header.size)) { (void)Fail(3U); return; }
    }
    crc = FW_Crc(crc, data, n);
    checked += n;
    if (checked == header.size) {
        if ((crc ^ 0xFFFFFFFFUL) != header.crc) { (void)Fail(2U); return; }
        state = FW_VERIFIED;
    }
}
uint8_t FW_Write(uint16_t address, uint16_t count, const uint8_t *data)
{
    uint8_t bytes[FW_CHUNK], verify[FW_CHUNK];
    uint32_t zero = 0U;
    if (data == NULL) return 3U;
    if (address == FW_MB_FIRST) {
        uint16_t command = Word(data);
        if (command == 1U) {
            FW_Header old;
            if (count != 7U) return 3U;
            uint32_t size = DWord(data + 2);
            if (size < 8U || size > FW_APP_SIZE || Word(data + 12) != FW_TARGET) return 3U;
#ifdef FW_EEPROM_DIAGNOSTICS
            USART2_Print("[FWUP EEPROM] begin\r\n");
#endif
            /* Never overwrite an image already committed for the loader. */
            if (!Read(FW_META_BASE, &old, sizeof(old))) return Fail(1U);
            if (FW_HeaderValid(&old)) return 6U;
            uint8_t probe_error = ProbeStorage();
            if (probe_error != 0U) return Fail(probe_error);
            if (!Write(FW_META_BASE, &zero, sizeof(zero)) ||
                !Read(FW_META_BASE, &old.magic, sizeof(old.magic))) return Fail(1U);
            if (old.magic != 0U) {
                LogEepromFailure("header clear mismatch", FW_META_BASE, old.magic, 0U);
                return Fail(1U);
            }
            memset(&header, 0, sizeof(header));
            header.size = size; header.crc = DWord(data + 6);
            header.version = Word(data + 10); header.target = FW_TARGET;
            header.app_base = FW_APP_BASE;
            received = checked = error = 0U;
            state = FW_RECEIVING;
            return 0U;
        }
        if (count != 1U) return 3U;
        if (command == 2U) {
            if (state == FW_VERIFYING || state == FW_VERIFIED) return 0U;
            if (state != FW_RECEIVING || received != header.size) return 3U;
            checked = 0U; crc = 0xFFFFFFFFUL; state = FW_VERIFYING; return 0U;
        }
        if (command == 3U) {
            if (state == FW_READY) return 0U;
            if (state != FW_VERIFIED) return 3U;
            header.magic = FW_MAGIC;
            header.header_crc = FW_Crc(0xFFFFFFFFUL, (const uint8_t *)&header.size, 20U) ^ 0xFFFFFFFFUL;
            if (!Write(FW_META_BASE + 4U, &header.size, sizeof(header) - 4U) ||
                !Read(FW_META_BASE + 4U, verify, sizeof(header) - 4U) ||
                memcmp(verify, &header.size, sizeof(header) - 4U) ||
                !Write(FW_META_BASE, &header.magic, 4U) ||
                !Read(FW_META_BASE, verify, sizeof(header)) || memcmp(verify, &header, sizeof(header))) return Fail(1U);
            state = FW_READY; return 0U;
        }
        if (command == 4U) {
            if (state == FW_READY) return 6U;
            if (!Write(FW_META_BASE, &zero, 4U)) return Fail(1U);
            state = FW_IDLE; received = checked = error = 0U; return 0U;
        }
        return 3U;
    }
    if (address != FW_MB_DATA) return 2U;
    if (count < 3U || count > 66U || state != FW_RECEIVING) return 3U;
    uint16_t offset = Word(data), n = Word(data + 2);
    if (n == 0U || n > FW_CHUNK || count != 2U + (n + 1U) / 2U ||
        (uint32_t)offset + n > header.size || offset > received) return 3U;
    memcpy(bytes, data + 4, n);
    if (offset < received) {
        /* Retry of an acknowledged/lost-response block is idempotent. */
        if ((uint32_t)offset + n > received) return 3U;
        if (!Read((uint16_t)(FW_IMAGE_BASE + offset), verify, n)) return Fail(1U);
        return memcmp(bytes, verify, n) == 0 ? 0U : 3U;
    }
    if (!Write((uint16_t)(FW_IMAGE_BASE + offset), bytes, n) ||
        !Read((uint16_t)(FW_IMAGE_BASE + offset), verify, n) || memcmp(bytes, verify, n)) return Fail(1U);
    received += n;
    return 0U;
}
#endif
