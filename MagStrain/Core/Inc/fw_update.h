#ifndef FW_UPDATE_H
#define FW_UPDATE_H
#include <stdbool.h>
#include <stdint.h>
/* Existing parameter addresses and serialized data are deliberately unchanged. */
#define FW_META_BASE 0x2000U
#define FW_IMAGE_BASE 0x2080U
#define FW_APP_BASE 0x08003000UL
#define FW_APP_SIZE (52U * 1024U)
#define FW_MAGIC 0x46573149UL
#define FW_TARGET 0x103BU
#define FW_MB_FIRST 6200U
#define FW_MB_LAST 6275U
#define FW_MB_DATA 6210U
#define FW_CHUNK 128U
/* Header words are little endian in EEPROM; magic is written LAST. */
typedef struct {
    uint32_t magic, size, crc, app_base, target, version, header_crc;
} FW_Header;
typedef enum { FW_IDLE, FW_RECEIVING, FW_VERIFYING, FW_VERIFIED, FW_READY, FW_ERROR } FW_State;
uint32_t FW_Crc(uint32_t crc, const uint8_t *data, uint32_t size);
bool FW_HeaderValid(const FW_Header *h);
bool FW_VectorsValid(uint32_t sp, uint32_t pc, uint32_t size);
bool FW_Active(void);
void FW_Process(void);
uint16_t FW_Read(uint16_t address);
/* Return a Modbus exception (zero on success). Wire bytes MSB first. */
uint8_t FW_Write(uint16_t address, uint16_t count, const uint8_t *data);
#endif
