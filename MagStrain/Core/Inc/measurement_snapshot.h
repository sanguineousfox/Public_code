/**
 * @file    measurement_snapshot.h
 * @brief   Атомарный быстрый снимок текущих измерений в RAM.
 *
 * Снимок содержит уже подготовленные 16-битные слова Modbus для диапазона
 * 1000..1013. Объёмы хранятся в м³. Порядок слов по таблице Е.3 ПМП-201Е:
 * младшее слово float32 по базовому адресу, старшее - по следующему.
 * EEPROM, I2C, фильтрация и преобразование float при чтении не выполняются.
 */
#ifndef MEASUREMENT_SNAPSHOT_H
#define MEASUREMENT_SNAPSHOT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define MEASUREMENT_SNAPSHOT_FIRST_ADDRESS  1000U
#define MEASUREMENT_SNAPSHOT_LAST_ADDRESS   1013U
#define MEASUREMENT_SNAPSHOT_WORD_COUNT     14U
#define MEASUREMENT_STATUS_VALID            0x0001U
#define MEASUREMENT_STATUS_HELD             0x0010U

#define MEASUREMENT_HISTORY_PERIOD_MS       1000U
#define MEASUREMENT_LEVEL_MAX_AGE_MS        1000U
#define MEASUREMENT_HISTORY_CAPACITY        30U
#define MEASUREMENT_HISTORY_HEADER_WORDS    3U
#define MEASUREMENT_HISTORY_POINT_WORDS     3U
#define MEASUREMENT_HISTORY_WORDS \
    (MEASUREMENT_HISTORY_HEADER_WORDS + MEASUREMENT_HISTORY_CAPACITY * MEASUREMENT_HISTORY_POINT_WORDS)

void MeasurementSnapshot_Init(void);

/* These functions run in the main loop, never in a UART interrupt. */
void MeasurementSnapshot_StartHistory(uint32_t now_ms);
void MeasurementSnapshot_RecordDue(uint32_t now_ms);
uint16_t MeasurementSnapshot_ReadHistoryWord(uint16_t offset);

/* Preserve the full recent snapshot and mark its level as held. */
bool MeasurementSnapshot_HoldLevel(uint32_t now_ms);

void MeasurementSnapshot_Publish(float level_mm,
                                 float temperature_c,
                                 float percent,
                                 float volume_m3,
                                 float mass_kg,
                                 float density_kg_m3,
                                 uint16_t status,
                                 uint32_t timestamp_ms);

bool MeasurementSnapshot_ReadWord(uint16_t address, uint16_t *value);

bool MeasurementSnapshot_ReadRange(uint16_t start_address,
                                   uint16_t register_count,
                                   uint16_t *destination);

bool MeasurementSnapshot_IsValid(void);
uint32_t MeasurementSnapshot_GetTimestamp(void);
uint16_t MeasurementSnapshot_GetStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* MEASUREMENT_SNAPSHOT_H */
