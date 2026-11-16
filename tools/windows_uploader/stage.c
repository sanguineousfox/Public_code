#include <stddef.h>
#include <stdint.h>
#include "update_config.h"

int PortWrite(const uint8_t *bytes, size_t length);
int PortReadByte(void);
void PortDiscardInput(void);
void DelayMs(uint32_t ms);
uint32_t NowMs(void);
void Print(const char *text);
void PrintNumber(uint32_t value, unsigned base, unsigned width);

static uint16_t Crc16(const uint8_t *data, size_t size)
{
    uint16_t crc = 0xFFFFU;
    while (size--) {
        crc ^= *data++;
        for (unsigned bit = 0; bit < 8U; ++bit)
            crc = (uint16_t)((crc >> 1) ^ ((crc & 1U) ? 0xA001U : 0U));
    }
    return crc;
}

static uint32_t Crc32(const uint8_t *data, size_t size)
{
    uint32_t crc = 0xFFFFFFFFU;
    while (size--) {
        crc ^= *data++;
        for (unsigned bit = 0; bit < 8U; ++bit)
            crc = (crc >> 1) ^ ((crc & 1U) ? 0xEDB88320U : 0U);
    }
    return ~crc;
}

static uint16_t Word(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] << 8 | data[1]);
}

static void SetWord(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value >> 8);
    data[1] = (uint8_t)value;
}

static uint32_t LittleDword(const uint8_t *data)
{
    return data[0] | (uint32_t)data[1] << 8 | (uint32_t)data[2] << 16 | (uint32_t)data[3] << 24;
}

/* Return zero on success, a positive Modbus exception, or a negative transport error. */
static int Request(uint8_t fc, const uint8_t *body, size_t length, uint8_t *reply)
{
    uint8_t tx[141];
    tx[0] = 1U;
    tx[1] = fc;
    for (size_t i = 0; i < length; ++i) tx[2U + i] = body[i];
    uint16_t crc = Crc16(tx, length + 2U);
    tx[length + 2U] = (uint8_t)crc;
    tx[length + 3U] = (uint8_t)(crc >> 8);

    int last = -1;
    for (unsigned attempt = 0; attempt < 5U; ++attempt) {
        DelayMs(20U);
        PortDiscardInput();
        if (!PortWrite(tx, length + 4U)) return -2;
        last = -1;
        size_t expected = 3U;
        size_t i;
        for (i = 0; i < expected; ++i) {
            int value = PortReadByte();
            if (value < 0) break;
            reply[i] = (uint8_t)value;
            if (i != 2U) continue;
            if (reply[0] != 1U) break;
            if (reply[1] == (fc | 0x80U)) expected = 5U;
            else if (reply[1] != fc) break;
            else if (fc == 3U) {
                if (reply[2] != 2U * Word(body + 2)) break;
                expected = 5U + reply[2];
            } else expected = 8U;
        }
        if (i != expected) continue;
        crc = Crc16(reply, expected - 2U);
        if (reply[expected - 2U] != (uint8_t)crc ||
            reply[expected - 1U] != (uint8_t)(crc >> 8)) {
            last = -3;
            continue;
        }
        if (reply[1] == (fc | 0x80U)) {
            if (reply[2] != 6U) return reply[2];
            last = 6;
            DelayMs(200U);
            continue;
        }
        if (fc != 3U && (Word(reply + 2) != Word(body) ||
                         Word(reply + 4) != Word(body + 2))) {
            last = -4;
            continue;
        }
        return 0;
    }
    return last;
}

static int ReadStatus(uint8_t *reply)
{
    const uint8_t body[] = {0x18, 0x38, 0, 10}; /* 6200, ten registers */
    return Request(3U, body, sizeof(body), reply);
}

static int WriteBlock(uint16_t reg, const uint8_t *data, uint8_t length)
{
    uint8_t body[137], reply[8];
    SetWord(body, reg);
    SetWord(body + 2, (uint16_t)(length / 2U));
    body[4] = length;
    for (unsigned i = 0; i < length; ++i) body[5U + i] = data[i];
    return Request(16U, body, (size_t)length + 5U, reply);
}

static int ReportError(const char *phase, int code)
{
    Print(phase);
    if (code > 0) {
        Print(": Modbus exception ");
        PrintNumber((uint32_t)code, 10U, 1U);
    } else if (code == -2) Print(": serial write failed");
    else if (code == -3) Print(": CRC16 error in reply");
    else if (code == -4) Print(": unexpected write acknowledgement");
    else Print(": no valid reply");
    Print("\r\n");
    return 1;
}

int StageImage(const uint8_t *image, size_t size)
{
    if (size < 8U || size > 53248U) {
        Print("Invalid image length.\r\n");
        return 1;
    }
    uint32_t sp = LittleDword(image), pc = LittleDword(image + 4);
    if (sp <= 0x20000000U || sp > 0x20005000U || (sp & 7U) ||
        (pc & 1U) == 0U || (pc & ~1U) < 0x08003000U ||
        (pc & ~1U) >= 0x08003000U + size) {
        Print("Invalid application vectors.\r\n");
        return 1;
    }
    uint32_t crc = Crc32(image, size);
    uint8_t reply[25];
    int result = ReadStatus(reply);
    if (result != 0) return ReportError("Read 6200-6209", result);
    if (Word(reply + 21) != 1U) {
        Print("Unsupported update protocol. Check register 6209.\r\n");
        return 1;
    }

    uint8_t header[14];
    SetWord(header, 1U);
    SetWord(header + 2, (uint16_t)size);
    SetWord(header + 4, 0U);
    SetWord(header + 6, (uint16_t)crc);
    SetWord(header + 8, (uint16_t)(crc >> 16));
    SetWord(header + 10, 10U * FW_WINDOWS_MAJOR + FW_WINDOWS_MINOR);
    SetWord(header + 12, 0x103BU);
    result = WriteBlock(6200U, header, sizeof(header));
    if (result != 0) {
        if (ReadStatus(reply) == 0) {
            Print("Device error register 6201 = ");
            PrintNumber(Word(reply + 5), 10U, 1U);
            Print("\r\n");
        }
        return ReportError("Begin transfer", result);
    }
    Print("Uploading image to EEPROM...\r\n");
    for (size_t offset = 0; offset < size; offset += 128U) {
        size_t length = size - offset;
        if (length > 128U) length = 128U;
        uint8_t data[132];
        SetWord(data, (uint16_t)offset);
        SetWord(data + 2, (uint16_t)length);
        for (size_t i = 0; i < length; ++i) data[4U + i] = image[offset + i];
        if (length & 1U) data[4U + length] = 0U;
        result = WriteBlock(6210U, data, (uint8_t)(4U + length + (length & 1U)));
        if (result != 0) {
            Print("Offset ");
            PrintNumber((uint32_t)offset, 10U, 1U);
            Print("\r\n");
            return ReportError("Write image", result);
        }
        if (((offset + length) % 4096U) == 0U || offset + length == size) {
            PrintNumber((uint32_t)(offset + length), 10U, 1U);
            Print(" bytes accepted\r\n");
        }
    }

    const uint8_t verify[] = {0x18, 0x38, 0, 2}; /* FC06, 6200 = verify */
    result = Request(6U, verify, sizeof(verify), reply);
    if (result != 0) return ReportError("Start verification", result);
    uint32_t start = NowMs();
    for (;;) {
        result = ReadStatus(reply);
        if (result != 0) return ReportError("Read verification status", result);
        uint16_t state = Word(reply + 3);
        if (state == 3U) break;
        if (state != 2U || NowMs() - start >= 120000U) {
            Print("Verification failed. State=");
            PrintNumber(state, 10U, 1U);
            Print(" error=");
            PrintNumber(Word(reply + 5), 10U, 1U);
            Print("\r\n");
            return 1;
        }
        DelayMs(50U);
    }
    Print("Verified ");
    PrintNumber((uint32_t)size, 10U, 1U);
    Print(" bytes, CRC32=");
    PrintNumber(crc, 16U, 8U);
    Print("\r\nInstallation is separate: FC06, register 6200 = 3.\r\n");
    return 0;
}

#ifdef FW_WINDOWS_INSTALL
int InstallImage(size_t size)
{
    uint8_t reply[25];
    int result = ReadStatus(reply);
    if (result != 0) return ReportError("Read 6200-6209", result);
    uint16_t state = Word(reply + 3);
    uint16_t error = Word(reply + 5);
    Print("Before install: 6200=");
    PrintNumber(state, 10U, 1U);
    Print(", 6201=");
    PrintNumber(error, 10U, 1U);
    Print("\r\n");
    if ((state != 3U && state != 4U) || error != 0U || Word(reply + 21) != 1U ||
        Word(reply + 7) != size || Word(reply + 9) != 0U ||
        Word(reply + 11) != size || Word(reply + 13) != 0U ||
        Word(reply + 15) != size || Word(reply + 17) != 0U) {
        Print("No matching verified image. Installation command was not sent.\r\n");
        return 1;
    }

    const uint8_t install[] = {0x18, 0x38, 0, 3};
    result = Request(6U, install, sizeof(install), reply);
    if (result != 0) {
        ReportError("Write 6200 = 3", result);
    } else {
        Print("Device acknowledged FC06, 6200 = 3.\r\n");
    }

    const uint8_t version[] = {0x09, 0x74, 0, 2}; /* 2420 and 2421 */
    uint32_t start = NowMs();
    int got_reply = 0;
    do {
        DelayMs(500U);
        if (Request(3U, version, sizeof(version), reply) == 0) {
            got_reply = 1;
            if (Word(reply + 3) == FW_WINDOWS_MAJOR && Word(reply + 5) == FW_WINDOWS_MINOR) {
                Print("Installed version: " FW_VERSION_TEXT "\r\n");
                return 0;
            }
        }
    } while (NowMs() - start < 30000U);
    Print(got_reply ? "Device responded, but version " FW_VERSION_TEXT " was not confirmed.\r\n"
                    : "No version reply after installation command.\r\n");
    if (ReadStatus(reply) == 0) {
        Print("After install: 6200=");
        PrintNumber(Word(reply + 3), 10U, 1U);
        Print(", 6201=");
        PrintNumber(Word(reply + 5), 10U, 1U);
        Print("\r\n");
    }
    return 1;
}
#endif
