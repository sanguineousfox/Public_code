#include <stddef.h>
#include <stdint.h>
#include "update_config.h"

int PortWrite(const uint8_t *bytes, size_t length);
int PortReadByte(void);
void PortDiscardInput(void);
void DelayMs(uint32_t ms);
void Print(const char *text);
void Report(const char *text);
uint32_t NowMs(void);
int StopRequested(void);

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

static uint16_t Word(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] << 8 | data[1]);
}

static void SetWord(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value >> 8);
    data[1] = (uint8_t)value;
}

static int ReadWords(uint16_t address, unsigned count, uint8_t *reply)
{
    uint8_t request[8] = {1, 3, 0, 0, 0, 0, 0, 0};
    SetWord(request + 2, address);
    SetWord(request + 4, (uint16_t)count);
    uint16_t crc = Crc16(request, 6U);
    request[6] = (uint8_t)crc;
    request[7] = (uint8_t)(crc >> 8);

    DelayMs(5U);
    PortDiscardInput();
    if (!PortWrite(request, sizeof(request))) return -2;

    unsigned expected = 3U;
    for (unsigned i = 0U; i < expected; ++i) {
        int byte = PortReadByte();
        if (byte < 0) return -1;
        reply[i] = (uint8_t)byte;
        if (i == 0U && reply[0] != 1U) return -4;
        if (i != 2U) continue;
        if (reply[1] == 0x83U) expected = 5U;
        else if (reply[1] == 3U && reply[2] == count * 2U) expected = 5U + reply[2];
        else return -4;
    }
    crc = Crc16(reply, expected - 2U);
    if (reply[expected - 2U] != (uint8_t)crc ||
        reply[expected - 1U] != (uint8_t)(crc >> 8)) return -3;
    return reply[1] == 3U ? 0 : reply[2];
}

enum {
    TRACE_WORDS = 24,
    DIAGNOSTIC_INTERVAL_MS = 200,
    SESSION_LENGTH_MS = FW_TRACE_MINUTES * 60000
};

static void CsvInt(int32_t value)
{
    char digits[12];
    unsigned offset = sizeof(digits) - 1U;
    uint32_t magnitude = value < 0 ? 0U - (uint32_t)value : (uint32_t)value;
    digits[offset] = '\0';
    do {
        digits[--offset] = (char)('0' + magnitude % 10U);
        magnitude /= 10U;
    } while (magnitude != 0U);
    if (value < 0) digits[--offset] = '-';
    Report(digits + offset);
}

static void CsvField(int32_t value)
{
    Report(";");
    CsvInt(value);
}

static uint32_t FloatBits(const uint8_t *words)
{
    return ((uint32_t)Word(words + 2) << 16) | Word(words);
}

/* Register 1000 is returned in mm by this firmware. A blank means invalid. */
static void CsvLevel(const uint8_t *words)
{
    union { uint32_t bits; float value; } level;
    level.bits = FloatBits(words);
    if ((level.bits & 0x7F800000U) == 0x7F800000U ||
        level.value < -100000.0f || level.value > 100000.0f) return;
    if (level.value < 0.0f) Report("-");
    float magnitude = level.value < 0.0f ? -level.value : level.value;
    unsigned thousandths = (unsigned)(magnitude * 1000.0f + 0.5f);
    CsvInt((int32_t)(thousandths / 1000U));
    Report(".");
    unsigned fraction = thousandths % 1000U;
    if (fraction < 100U) Report("0");
    if (fraction < 10U) Report("0");
    CsvInt((int32_t)fraction);
}

#if !FW_SECONDS_LOGGER
void RecordPulseTrace(void)
{
    uint8_t version[9], period[9];
    uint8_t pulse[5 + TRACE_WORDS * 2], level[9], error[7];
    int version_status = ReadWords(2420U, 2U, version);
    int period_status = ReadWords(2088U, 2U, period);
    /* Even if the first request times out, record the rest of the session. */
    if (version_status != 0)
        Print("No reply to version query. Continuing to log requests.\r\n");

    union { uint32_t bits; float value; } configured_period;
    configured_period.bits = period_status == 0 ? FloatBits(period + 3) : 0U;
    int32_t period_ms = configured_period.value >= 50.0f &&
                        configured_period.value <= 60000.0f ?
                        (int32_t)(configured_period.value + 0.5f) : -1;

    Print("Recording up to " FW_STRING(FW_TRACE_MINUTES) " minutes. Ctrl+C saves and stops.\r\n");
    Report("fw_major;fw_minor;period_ms;cycle_ms;trace_ms;level_ms;error_ms;trace_status;level_status;error_status;level_mm;error_2416;mode;state;valid_pairs;sequence;new_launches;missed_launches");
    for (unsigned i = 1U; i <= 10U; ++i) {
        Report(";t1_"); CsvInt((int32_t)i);
        Report(";t2_"); CsvInt((int32_t)i);
    }
    Report("\r\n");

    uint32_t start = NowMs();
    uint16_t last_sequence = 0U;
    int have_sequence = 0;
    do {
        uint32_t cycle_start = NowMs();
        const int trace_status = ReadWords(6044U, TRACE_WORDS, pulse);
        const uint32_t trace_ms = NowMs() - start;
        const int level_status = ReadWords(1000U, 2U, level);
        const uint32_t level_ms = NowMs() - start;
        const int error_status = ReadWords(2416U, 1U, error);

        const uint32_t error_ms = NowMs() - start;

        CsvInt(version_status == 0 ? Word(version + 3) : -1);
        CsvField(version_status == 0 ? Word(version + 5) : -1);
        CsvField(period_ms);
        CsvField((int32_t)(cycle_start - start));
        CsvField((int32_t)trace_ms);
        CsvField((int32_t)level_ms);
        CsvField((int32_t)error_ms);
        CsvField(trace_status);
        CsvField(level_status);
        CsvField(error_status);
        Report(";");
        if (level_status == 0) CsvLevel(level + 3);
        Report(";");
        if (error_status == 0) CsvInt(Word(error + 3));

        if (trace_status == 0) {
            for (unsigned i = 0; i < 4; ++i) CsvField(Word(pulse + 3U + 2U * i));
            uint16_t sequence = Word(pulse + 9);
            uint16_t new_launches = (uint16_t)(sequence - last_sequence);
            CsvField(have_sequence ? new_launches : -1);
            CsvField(!have_sequence ? -1 : new_launches > 10U ? new_launches - 10 : 0);
            last_sequence = sequence;
            have_sequence = 1;
            for (unsigned i = 4; i < TRACE_WORDS; ++i) CsvField(Word(pulse + 3U + 2U * i));
        } else {
            for (unsigned i = 0; i < TRACE_WORDS + 2U; ++i) Report(";");
        }
        Report("\r\n");

        if (StopRequested()) break;
        uint32_t elapsed = NowMs() - cycle_start;
        if (elapsed < DIAGNOSTIC_INTERVAL_MS) DelayMs(DIAGNOSTIC_INTERVAL_MS - elapsed);
    } while (!StopRequested() && NowMs() - start < SESSION_LENGTH_MS);
}
#else
enum {
    HISTORY_ADDRESS = 6070,
    HISTORY_CAPACITY = 30,
    HISTORY_HEADER_WORDS = 3,
    HISTORY_POINT_WORDS = 3,
    HISTORY_WORDS = HISTORY_HEADER_WORDS + HISTORY_CAPACITY * HISTORY_POINT_WORDS,
    HISTORY_STATUS_VALID = 1,
    HISTORY_POLL_MS = 500
};

static void ReportEvent(const char *event, uint32_t received_ms, int32_t detail)
{
    CsvInt((int32_t)received_ms);
    Report(";;;;");
    Report(event);
    CsvField(detail);
    Report("\r\n");
}

void RecordLevelHistory(void)
{
    uint8_t reply[5U + 2U * HISTORY_WORDS];
    const uint32_t start = NowMs();
    uint32_t next_poll = 0U;
    uint32_t previous = 0U;
    int started = 0;

    Print("Recording 1 Hz history for " FW_STRING(FW_TRACE_MINUTES) " minutes (firmware 151.4+). Ctrl+C saves and stops.\r\n");
    Report("received_ms;device_second;level_mm;status;event;detail\r\n");
    do {
        int status = ReadWords(HISTORY_ADDRESS, HISTORY_HEADER_WORDS + HISTORY_POINT_WORDS, reply);
        if (status < 0) {
            /* Retry in the same poll window. A timeout never discards device points. */
            status = ReadWords(HISTORY_ADDRESS, HISTORY_HEADER_WORDS + HISTORY_POINT_WORDS, reply);
        }
        if (status == 0 && started) {
            uint32_t latest = FloatBits(reply + 3U);
            if (latest > previous && latest - previous > 1U) {
                status = ReadWords(HISTORY_ADDRESS, HISTORY_WORDS, reply);
            }
        }

        const uint32_t received_ms = NowMs() - start;
        if (status != 0) {
            ReportEvent("modbus_error", received_ms, status);
        } else {
            uint32_t latest = FloatBits(reply + 3U);
            uint16_t count = Word(reply + 7U);
            if (started && latest < previous) {
                ReportEvent("device_restarted", received_ms, (int32_t)latest);
                started = 0;
            }
            if (count > HISTORY_CAPACITY || count > latest) {
                ReportEvent("invalid_history", received_ms, count);
            } else if (count != 0U) {
                uint32_t records = started ? latest - previous : 1U;
                if (records > count) {
                    ReportEvent("history_overrun", received_ms, (int32_t)(records - count));
                    records = count;
                }
                while (records > 0U) {
                    --records;
                    const uint8_t *point = reply + 3U + 2U *
                        (HISTORY_HEADER_WORDS + records * HISTORY_POINT_WORDS);
                    uint16_t quality = Word(point + 4U);
                    CsvInt((int32_t)received_ms);
                    CsvField((int32_t)(latest - records));
                    Report(";");
                    if (quality & HISTORY_STATUS_VALID) CsvLevel(point);
                    CsvField(quality);
                    Report(";;\r\n");
                }
                started = 1;
                previous = latest;
            }
        }

        if (StopRequested()) break;
        uint32_t elapsed = NowMs() - start;
        do {
            next_poll += HISTORY_POLL_MS;
        } while (next_poll <= elapsed);
        if (next_poll >= SESSION_LENGTH_MS) break;
        DelayMs(next_poll - elapsed);
    } while (!StopRequested() && NowMs() - start < SESSION_LENGTH_MS);
}
#endif
