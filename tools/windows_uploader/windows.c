#include <stddef.h>
#include <stdint.h>
#include "update_config.h"

typedef void *Handle;
typedef struct {
    uint32_t length, baud, flags;
    uint16_t reserved, xon_limit, xoff_limit;
    uint8_t byte_size, parity, stop_bits, xon, xoff, error, eof, event;
    uint16_t reserved1;
} CommState;

typedef struct {
    uint32_t read_interval, read_multiplier, read_constant, write_multiplier, write_constant;
} CommTimeouts;

_Static_assert(sizeof(CommState) == 28U, "Win32 DCB size");
extern Handle (*CreateFileA)(const char *, uint32_t, uint32_t, void *, uint32_t, uint32_t, Handle);
extern Handle (*GetStdHandle)(int32_t);
extern int (*GetCommState)(Handle, CommState *);
extern int (*SetCommState)(Handle, CommState *);
extern int (*SetCommTimeouts)(Handle, const CommTimeouts *);
extern int (*ReadFile)(Handle, void *, uint32_t, uint32_t *, void *);
extern int (*WriteFile)(Handle, const void *, uint32_t, uint32_t *, void *);
extern int (*PurgeComm)(Handle, uint32_t);
extern int (*CloseHandle)(Handle);
extern void (*Sleep)(uint32_t);
extern uint32_t (*GetTickCount)(void);
extern uint32_t (*GetLastError)(void);
extern void (*ExitProcess)(uint32_t);
extern const uint8_t _binary_application_v149_1_bin_start[];
extern const uint8_t _binary_application_v149_1_bin_end[];
extern int StageImage(const uint8_t *image, size_t size);
#ifdef FW_WINDOWS_INSTALL
extern int InstallImage(size_t size);
#endif

static Handle port, output;

void Print(const char *text)
{
    uint32_t length = 0U, written;
    while (text[length]) ++length;
    WriteFile(output, text, length, &written, 0);
}

void PrintNumber(uint32_t value, unsigned base, unsigned width)
{
    static const char digits[] = "0123456789ABCDEF";
    char buffer[33];
    unsigned pos = sizeof(buffer) - 1U;
    buffer[pos] = '\0';
    do {
        buffer[--pos] = digits[value % base];
        value /= base;
        if (width != 0U) --width;
    } while (value != 0U || width != 0U);
    Print(buffer + pos);
}

int PortWrite(const uint8_t *bytes, size_t length)
{
    uint32_t count = 0U;
    return WriteFile(port, bytes, (uint32_t)length, &count, 0) && count == length;
}

int PortReadByte(void)
{
    uint8_t byte;
    uint32_t count = 0U;
    return ReadFile(port, &byte, 1U, &count, 0) && count == 1U ? byte : -1;
}

void PortDiscardInput(void) { PurgeComm(port, 8U); }
void DelayMs(uint32_t ms) { Sleep(ms); }
uint32_t NowMs(void) { return GetTickCount(); }

static void WindowsError(const char *operation)
{
    uint32_t error = GetLastError();
    Print(operation);
    Print(" failed. Windows error ");
    PrintNumber(error, 10U, 1U);
    Print(". Close the Modbus master and verify " FW_PORT_NAME ".\r\n");
}

static int ConfigurePort(void)
{
    CommState state;
    state.length = sizeof(state);
    if (!GetCommState(port, &state)) { WindowsError("GetCommState"); return 0; }
    state.baud = 19200U;
    state.flags = 1U; /* fBinary, no flow control, no parity checking. */
    state.byte_size = 8U;
    state.parity = 0U;
    state.stop_bits = 0U;
    if (!SetCommState(port, &state)) { WindowsError("SetCommState"); return 0; }
    const CommTimeouts timeouts = {0U, 0U, 1000U, 0U, 2000U};
    if (!SetCommTimeouts(port, &timeouts)) { WindowsError("SetCommTimeouts"); return 0; }
    return 1;
}

void entry(void)
{
    output = GetStdHandle(-11);
#ifdef FW_WINDOWS_INSTALL
    Print("MagStrain v" FW_VERSION_TEXT " - install verified image on " FW_PORT_NAME ", 19200 8N1, address 1\r\n");
    Print("Type I then Enter to send the installation command.\r\n");
    char confirmation[32];
    uint32_t length = 0U;
    if (!ReadFile(GetStdHandle(-10), confirmation, sizeof(confirmation), &length, 0) ||
        length < 2U || confirmation[0] != 'I' ||
        (confirmation[1] != '\r' && confirmation[1] != '\n')) {
        Print("Cancelled.\r\n");
        ExitProcess(1U);
        for (;;) {}
    }
#else
    Print("MagStrain v" FW_VERSION_TEXT " - EEPROM staging on " FW_PORT_NAME ", 19200 8N1, address 1\r\n");
#endif
    port = CreateFileA("\\\\.\\" FW_PORT_NAME, 0xC0000000U, 0U, 0, 3U, 0U, 0);
    int status = 1;
    if (port == (Handle)(intptr_t)-1) WindowsError("Open " FW_PORT_NAME);
    else {
        if (ConfigurePort()) {
            Print(FW_PORT_NAME " opened. Checking Modbus...\r\n");
            size_t size = (size_t)((uintptr_t)_binary_application_v149_1_bin_end -
                                  (uintptr_t)_binary_application_v149_1_bin_start);
#ifdef FW_WINDOWS_INSTALL
            status = InstallImage(size);
#else
            status = StageImage(_binary_application_v149_1_bin_start, size);
#endif
        }
        CloseHandle(port);
    }
    Print("Press Enter to close.\r\n");
    char line[32];
    uint32_t count;
    ReadFile(GetStdHandle(-10), line, sizeof(line), &count, 0);
    ExitProcess((uint32_t)status);
    for (;;) {}
}
