/**
 * @file    temp_sensors.h
 * @brief   Модуль опроса до 8 датчиков температуры LM75B по шине I2C
 *          Интеграция с ПМП-201Е (регистры 2500-2714)
 */
#ifndef TEMP_SENSORS_H
#define TEMP_SENSORS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ==========================================================================
   КОНСТАНТЫ
   ========================================================================== */
#define TEMP_SENSORS_MAX_COUNT      8       // Максимум датчиков по ТУ
#define TEMP_SENSOR_SCAN_TIMEOUT_MS 50      // Таймаут при сканировании I2C
#define TEMP_SENSOR_READ_TIMEOUT_MS 100     // Таймаут чтения температуры

/* Базовый адрес LM75B (A0=A1=A2=GND) */
#define LM75B_BASE_ADDRESS          0x48

/* Диапазон корректных температур (для валидации) */
#define TEMP_MIN_VALID              (-55.0f)
#define TEMP_MAX_VALID              (130.0f)

/* Код "ошибочного" значения при отсутствии датчика (0xFFFFFFFF как float) */
#define TEMP_ERROR_VALUE            0xFFFFFFFF

/* ==========================================================================
   ТИПЫ ДАННЫХ
   ========================================================================== */

/**
 * @brief Структура состояния одного датчика температуры
 */
typedef struct {
    uint8_t  address;           // 8-bit адрес I2C (сдвинутый на 1)
    uint8_t  index;             // Порядковый номер (1-8)
    float    height_m;          // Высота установки, м (из EEPROM)
    float    temperature_c;     // Текущая температура, °C
    float    density_kg_m3;     // Плотность, приведенная к этой температуре
    bool     present;           // Флаг: датчик обнаружен на шине
    bool     valid;             // Флаг: последнее чтение корректно
    uint32_t error_count;       // Счетчик ошибок чтения
} TempSensor_t;

/**
 * @brief Структура состояния всей подсистемы температур
 */
typedef struct {
    TempSensor_t sensors[TEMP_SENSORS_MAX_COUNT];
    uint8_t      detected_count;      // Сколько датчиков реально найдено
    float        avg_liquid_temp;     // Средняя температура жидкости (ниже уровня)
    float        avg_vapor_temp;      // Средняя температура паровой фазы (выше уровня)
    float        temp_at_density;     // Температура в точке измерения плотности
    bool         scan_done;           // Сканирование шины выполнено
    uint32_t     last_scan_time_ms;   // Время последнего сканирования
    uint32_t     last_read_time_ms;   // Время последнего опроса
} TempSensorsState_t;

/* ==========================================================================
   ПРОТОТИПЫ ФУНКЦИЙ
   ========================================================================== */

/**
 * @brief Инициализация модуля температурных датчиков
 *        Выполняет сканирование шины I2C и поиск LM75B
 */
void TempSensors_Init(void);

/**
 * @brief Сканирование шины I2C для поиска всех LM75B
 *        Заполняет массив sensors[] найденными устройствами
 * @return Количество найденных датчиков
 */
uint8_t TempSensors_ScanBus(void);

/**
 * @brief Опрос всех найденных датчиков
 *        Обновляет температуры и записывает в регистры Modbus
 */
void TempSensors_ReadAll(void);

/**
 * @brief Расчет средних температур и плотности по датчикам
 * @param level_m        Текущий уровень жидкости, м
 * @param density_level  Уровень раздела сред, м (0 если нет)
 */
void TempSensors_CalculateAverages(float level_m, float density_level);

/**
 * @brief Загрузка высот установки датчиков из EEPROM (регистры 2500-2514)
 */
void TempSensors_LoadHeightsFromEEPROM(void);

/**
 * @brief Сохранение высот установки датчиков в EEPROM
 */
void TempSensors_SaveHeightsToEEPROM(void);

/**
 * @brief Обновление регистров Modbus (2600-2714) текущими значениями
 */
void TempSensors_UpdateModbusRegisters(void);

/**
 * @brief Получение указателя на состояние подсистемы
 */
TempSensorsState_t* TempSensors_GetState(void);

/**
 * @brief Проверка наличия датчика по индексу (0-7)
 */
bool TempSensors_IsPresent(uint8_t index);

/**
 * @brief Получение температуры датчика по индексу (0-7)
 * @return Температура в °C или NaN если датчик отсутствует
 */
float TempSensors_GetTemperature(uint8_t index);

/**
 * @brief Получение количества обнаруженных датчиков
 */
uint8_t TempSensors_GetDetectedCount(void);

#ifdef __cplusplus
}
#endif

#endif /* TEMP_SENSORS_H */
