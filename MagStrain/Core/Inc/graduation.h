/**
 * @file    graduation.h
 * @brief   Модуль работы с градуировочными таблицами резервуаров
 *          Хранение в EEPROM AT24C64 (адреса 0x0400+)
 *          Поддержка ПМП-201Е (регистр 32768+)
 */
#ifndef GRADUATION_H
#define GRADUATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ==========================================================================
   КОНСТАНТЫ
   ========================================================================== */

/* Максимальное количество точек в градуировочной таблице
 * EEPROM AT24C64 = 8192 байт
 * Занято: ~1024 байт (регистры, параметры)
 * Осталось: ~7168 байт
 * Заголовок: 24 байта
 * На точки: (7168 - 24) / 4 ≈ 1786 точек
 * Округляем до 1500 для запаса под служебные данные
 */
#define GRAD_MAX_POINTS             1500

/* Адреса в EEPROM для хранения градуировочной таблицы */
#define GRAD_EEPROM_BASE            0x0400
#define GRAD_HEADER_SIZE            24      /* Байт в заголовке */
#define GRAD_DATA_START             (GRAD_EEPROM_BASE + GRAD_HEADER_SIZE)

/* Magic для проверки валидности таблицы */
#define GRAD_MAGIC                  0x47524144  /* "GRAD" */

/* ==========================================================================
   ТИПЫ РЕЗЕРВУАРОВ (согласно ПМП-201Е, параметр Gr)
   ========================================================================== */
typedef enum {
    GRAD_TYPE_VERTICAL      = 0,    /* Вертикальный (линейная зависимость) */
    GRAD_TYPE_HORIZ_FLAT    = 1,    /* Горизонтальный цилиндрический с плоскими днищами */
    GRAD_TYPE_BY_TABLE      = 2,    /* По градуировочной таблице */
    GRAD_TYPE_HORIZ_ELLIPT  = 3     /* Горизонтальный цилиндрический с эллиптическими днищами */
} GradTankType_t;

/* ==========================================================================
   СТРУКТУРЫ ДАННЫХ
   ========================================================================== */

/**
 * @brief Заголовок градуировочной таблицы (хранится в EEPROM)
 */
typedef struct {
    uint32_t magic;                 /* Magic number (GRAD_MAGIC) */
    uint16_t points_count;          /* Количество точек */
    uint16_t reserved;              /* Зарезервировано */
    float    start_height_m;        /* Начальная высота градуировки, м */
    float    step_height_m;         /* Шаг по высоте, м */
    float    tank_height_m;         /* Высота/диаметр резервуара, м */
    float    tank_volume_m3;        /* Полный объем резервуара, м³ */
} __attribute__((packed)) GradHeader_t;

/**
 * @brief Структура состояния модуля градуировки
 */
typedef struct {
    GradHeader_t header;            /* Заголовок таблицы */
    float        volumes[GRAD_MAX_POINTS];  /* Массив объемов, м³ */
    bool         loaded;            /* Таблица загружена в RAM */
    bool         valid;             /* Таблица валидна (magic совпадает) */
    uint16_t     actual_points;     /* Реальное количество точек */
    uint32_t     crc32;             /* Контрольная сумма данных */
} GradState_t;

/* ==========================================================================
   ПРОТОТИПЫ ФУНКЦИИ
   ========================================================================== */

/**
 * @brief Инициализация модуля градуировки
 *        Загружает таблицу из EEPROM в RAM
 */
void Grad_Init(void);

/**
 * @brief Проверка валидности таблицы в EEPROM
 * @return true если таблица валидна
 */
bool Grad_IsValid(void);

/**
 * @brief Загрузка таблицы из EEPROM в RAM
 * @return HAL_OK при успехе
 */
HAL_StatusTypeDef Grad_LoadFromEEPROM(void);

/**
 * @brief Сохранение таблицы из RAM в EEPROM
 * @return HAL_OK при успехе
 */
HAL_StatusTypeDef Grad_SaveToEEPROM(void);

/**
 * @brief Очистка таблицы (форматирование)
 */
HAL_StatusTypeDef Grad_Clear(void);

/**
 * @brief Запись заголовка таблицы
 */
HAL_StatusTypeDef Grad_WriteHeader(const GradHeader_t *header);

/**
 * @brief Чтение заголовка таблицы
 */
HAL_StatusTypeDef Grad_ReadHeader(GradHeader_t *header);

/**
 * @brief Запись блока объемов (пакетная запись)
 * @param start_index Начальный индекс точки
 * @param volumes     Массив объемов
 * @param count       Количество точек
 */
HAL_StatusTypeDef Grad_WriteVolumes(uint16_t start_index,
                                     const float *volumes,
                                     uint16_t count);

/**
 * @brief Чтение блока объемов
 */
HAL_StatusTypeDef Grad_ReadVolumes(uint16_t start_index,
                                    float *volumes,
                                    uint16_t count);

/**
 * @brief Интерполяция объема по уровню жидкости
 * @param level_m Уровень жидкости, м
 * @return Объем в м³ или NaN если таблица не валидна
 */
float Grad_InterpolateVolume(float level_m);

/**
 * @brief Расчет объема для резервуара без градуировочной таблицы
 * @param tank_type Тип резервуара
 * @param level_m   Уровень жидкости, м
 * @param height_m  Высота/диаметр резервуара, м
 * @param volume_m3 Полный объем резервуара, м³
 * @return Объем в м³
 */
float Grad_CalculateVolume(GradTankType_t tank_type,
                            float level_m,
                            float height_m,
                            float volume_m3);

/**
 * @brief Получение указателя на состояние модуля
 */
GradState_t* Grad_GetState(void);

/**
 * @brief Обновление регистров Modbus (2014 - количество точек)
 */
void Grad_UpdateModbusRegisters(void);

/**
 * @brief Обработка команды загрузки таблицы через Modbus
 *        Вызывается при записи в регистр 3000 (команда)
 * @param cmd Код команды
 * @param param Параметр команды
 */
void Grad_ProcessCommand(uint16_t cmd, float param);

#ifdef __cplusplus
}
#endif

#endif /* GRADUATION_H */
