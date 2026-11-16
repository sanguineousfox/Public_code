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
extern int (*SetConsoleCtrlHandler)(int (*)(uint32_t), int);
typedef struct {
    uint16_t year, month, weekday, day, hour, minute, second, millisecond;
} SystemTime;
extern uint32_t (*GetModuleFileNameW)(Handle, uint16_t *, uint32_t);
extern Handle (*CreateFileW)(const uint16_t *, uint32_t, uint32_t, void *, uint32_t, uint32_t, Handle);
extern void (*GetLocalTime)(SystemTime *);
extern int (*FlushFileBuffers)(Handle);
#if FW_SECONDS_LOGGER
extern void RecordLevelHistory(void);
#define RECORD_MEASUREMENTS RecordLevelHistory
#define SESSION_TITLE "MagStrain 1 Hz history"
#define REPORT_PREFIX "MagStrain_seconds_"
#else
extern void RecordPulseTrace(void);
#define RECORD_MEASUREMENTS RecordPulseTrace
#define SESSION_TITLE "MagStrain pulse trace"
#define REPORT_PREFIX "MagStrain_pulse_trace_"
#endif

static Handle report;
static int report_error;
static Handle port, output;
static uint32_t stop_requested;

static int StopOnConsoleSignal(uint32_t event)
{
    /* CTRL_C_EVENT and CTRL_BREAK_EVENT. Other events use the normal handler. */
    if (event > 1U) return 0;
    __atomic_store_n(&stop_requested, 1U, __ATOMIC_RELAXED);
    return 1;
}

int StopRequested(void)
{
    return __atomic_load_n(&stop_requested, __ATOMIC_RELAXED) != 0U;
}

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
    const CommTimeouts timeouts = {0U, 0U, 120U, 0U, 500U};
    if (!SetCommTimeouts(port, &timeouts)) { WindowsError("SetCommTimeouts"); return 0; }
    return 1;
}

static uint16_t report_path[1024];
static char report_name[64];

void Report(const char *text)
{
    uint32_t length = 0U, written = 0U;
    if (report_error) return;
    while (text[length]) ++length;
    if (!WriteFile(report, text, length, &written, 0) || written != length)
        report_error = 1;
}

static char *Append(char *dest, const char *text)
{
    while (*text != '\0') *dest++ = *text++;
    *dest = '\0';
    return dest;
}

static char *AppendDigits(char *dest, unsigned value, unsigned digits)
{
    for (unsigned i = digits; i > 0U; --i) {
        dest[i - 1U] = (char)('0' + value % 10U);
        value /= 10U;
    }
    dest[digits] = '\0';
    return dest + digits;
}

static int OpenReport(void)
{
    uint32_t size = GetModuleFileNameW(0, report_path, sizeof(report_path) / sizeof(report_path[0]));
    if (size == 0U || size >= sizeof(report_path) / sizeof(report_path[0])) {
        WindowsError("Locate utility folder");
        return 0;
    }
    uint32_t folder = size;
    while (folder > 0U && report_path[folder - 1U] != '\\') --folder;
    if (folder + sizeof(report_name) >= sizeof(report_path) / sizeof(report_path[0])) return 0;

    SystemTime time;
    GetLocalTime(&time);
    char *name = Append(report_name, REPORT_PREFIX);
    name = AppendDigits(name, time.year, 4U);
    name = AppendDigits(name, time.month, 2U);
    name = AppendDigits(name, time.day, 2U);
    name = Append(name, "_");
    name = AppendDigits(name, time.hour, 2U);
    name = AppendDigits(name, time.minute, 2U);
    name = AppendDigits(name, time.second, 2U);
    name = AppendDigits(name, time.millisecond, 3U);
    name = Append(name, ".csv");
    for (size_t i = 0; i <= (size_t)(name - report_name); ++i)
        report_path[folder + i] = (uint8_t)report_name[i];
    report = CreateFileW(report_path, 0x40000000U, 1U, 0, 1U, 0U, 0);
    if (report == (Handle)(intptr_t)-1) {
        WindowsError("Create CSV in utility folder");
        return 0;
    }
    return 1;
}

void entry(void)
{
    output = GetStdHandle(-11);
    Print(SESSION_TITLE " - " FW_PORT_NAME ", 19200 8N1, address 1\r\n");
    port = CreateFileA("\\\\.\\" FW_PORT_NAME, 0xC0000000U, 0U, 0, 3U, 0U, 0);
    int status = 1;
    if (port == (Handle)(intptr_t)-1) {
        WindowsError("Open " FW_PORT_NAME);
    } else {
        if (ConfigurePort() && OpenReport()) {
            Print("Writing: ");
            Print(report_name);
            Print("\r\n");
            (void)SetConsoleCtrlHandler(StopOnConsoleSignal, 1);
            RECORD_MEASUREMENTS();
            (void)SetConsoleCtrlHandler(StopOnConsoleSignal, 0);
            status = 0;
            if (!FlushFileBuffers(report)) report_error = 1;
            if (!CloseHandle(report)) report_error = 1;
            Print(report_error ? "File write failed: " : "Saved: ");
            Print(report_name);
            Print("\r\n");
            if (report_error) status = 1;
        }
        CloseHandle(port);
    }
    Print("Press Enter to close.\r\n");
    char line[16];
    uint32_t count;
    ReadFile(GetStdHandle(-10), line, sizeof(line), &count, 0);
    ExitProcess((uint32_t)status);
    for (;;) {}
}
