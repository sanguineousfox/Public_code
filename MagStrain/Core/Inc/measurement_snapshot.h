/**
 * @file    measurement_snapshot.h
 * @brief   Атомарный быстрый снимок текущих измерений в RAM.
 *
 * Снимок содержит уже подготовленные 16-битные слова Modbus для диапазона
 * 1000..1007. Порядок слов соответствует таблице Е.3 ПМП-201Е:
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
#define MEASUREMENT_SNAPSHOT_LAST_ADDRESS   1007U
#define MEASUREMENT_SNAPSHOT_WORD_COUNT     8U

void MeasurementSnapshot_Init(void);

void MeasurementSnapshot_Publish(float level_mm,
                                 float temperature_c,
                                 float percent,
                                 float volume_m3,
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
