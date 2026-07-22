/**
 * @file    graduation.c
 * @brief   Реализация модуля работы с градуировочными таблицами
 */
#include "graduation.h"
#include "at24c64.h"
#include "modbus.h"
#include "main.h"
#include <string.h>
#include <math.h>

/* ==========================================================================
   ЛОКАЛЬНЫЕ ПЕРЕМЕННЫЕ
   ========================================================================== */
static GradState_t grad_state;

/* Внешний доступ к I2C */
extern I2C_HandleTypeDef hi2c2;

/* ==========================================================================
   ВНУТРЕННИЕ ФУНКЦИИ
   ========================================================================== */

/**
 * @brief Расчет CRC32 для блока данных
 */
static uint32_t calculate_crc32(const uint8_t *data, uint32_t length)
{
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
        }
    }
    return ~crc;
}

/**
 * @brief Расчет объема для вертикального резервуара
 *        V = π * R² * h = (π * D² / 4) * h
 *        Но проще: V = V_full * (h / H)
 */
static float calc_volume_vertical(float level_m, float height_m, float volume_m3)
{
    if (height_m <= 0.0f || volume_m3 <= 0.0f) return 0.0f;
    if (level_m <= 0.0f) return 0.0f;
    if (level_m >= height_m) return volume_m3;

    return volume_m3 * (level_m / height_m);
}

/**
 * @brief Расчет объема для горизонтального цилиндра с плоскими днищами
 *        V = L * [R² * arccos((R-h)/R) - (R-h) * sqrt(2*R*h - h²)]
 *        где R = D/2, L - длина цилиндра
 */
static float calc_volume_horiz_flat(float level_m, float diameter_m, float volume_m3)
{
    if (diameter_m <= 0.0f || volume_m3 <= 0.0f) return 0.0f;
    if (level_m <= 0.0f) return 0.0f;
    if (level_m >= diameter_m) return volume_m3;

    float R = diameter_m / 2.0f;
    float h = level_m;

    /* Площадь сегмента круга */
    float segment_area = R * R * acosf((R - h) / R) - (R - h) * sqrtf(2.0f * R * h - h * h);

    /* Полный объем = π * R² * L, значит L = V_full / (π * R²) */
    float L = volume_m3 / (3.14159265f * R * R);

    return segment_area * L;
}

/**
 * @brief Расчет объема для горизонтального цилиндра с эллиптическими днищами
 *        Высота днищ = D/4
 *        Добавляем объем эллиптических днищ
 */
static float calc_volume_horiz_ellipt(float level_m, float diameter_m, float volume_m3)
{
    if (diameter_m <= 0.0f || volume_m3 <= 0.0f) return 0.0f;
    if (level_m <= 0.0f) return 0.0f;
    if (level_m >= diameter_m) return volume_m3;

    float R = diameter_m / 2.0f;
    float h = level_m;
    float head_height = diameter_m / 4.0f;  /* Высота днища = D/4 */

    /* Объем цилиндрической части (как для плоских днищ) */
    float cyl_part = calc_volume_horiz_flat(level_m, diameter_m, volume_m3);

    /* Добавка на эллиптические днища (приближенно) */
    /* Объем эллиптического днища: V_head = (π/4) * R² * head_height * (h/R)² для h < R */
    float head_fraction = 0.0f;
    if (h <= R) {
        head_fraction = (h / R) * (h / R);
    } else {
        head_fraction = 1.0f - ((diameter_m - h) / R) * ((diameter_m - h) / R);
    }

    /* Общий объем двух днищ при полном заполнении */
    float total_heads_volume = volume_m3 * 0.1f;  /* Примерно 10% на днища */
    float heads_contribution = total_heads_volume * head_fraction;

    return cyl_part + heads_contribution;
}

/**
 * @brief Обновление регистра Modbus с количеством точек
 */
static void update_points_count_register(void)
{
    ModBus_SetParameter_Float(MB_ADDR_CALIB_POINTS, (float)grad_state.actual_points);
}

/* ==========================================================================
   ПУБЛИЧНЫЕ ФУНКЦИИ
   ========================================================================== */

void Grad_Init(void)
{
    memset(&grad_state, 0, sizeof(grad_state));
    grad_state.loaded = false;
    grad_state.valid = false;

    USART2_Print("[GRAD] Инициализация модуля градуировки...\r\n");

    /* Пытаемся загрузить таблицу из EEPROM */
    if (Grad_LoadFromEEPROM() == HAL_OK && grad_state.valid) {
        USART2_BufInit();
        USART2_BufPrint("[GRAD] Таблица загружена: ");
        USART2_BufPrintInt(grad_state.actual_points);
        USART2_BufPrint(" точек, H=");
        USART2_BufPrintFloat(grad_state.header.tank_height_m);
        USART2_BufPrint(" м, V=");
        USART2_BufPrintFloat(grad_state.header.tank_volume_m3);
        USART2_BufPrint(" м³\r\n");
        USART2_BufFlush();

        /* Обновляем регистры Modbus */
        Grad_UpdateModbusRegisters();
    } else {
        USART2_Print("[GRAD] Таблица не найдена или невалидна\r\n");
        USART2_Print("[GRAD] Будет использоваться расчет по формулам\r\n");
    }
}

bool Grad_IsValid(void)
{
    return grad_state.valid && grad_state.loaded;
}

HAL_StatusTypeDef Grad_LoadFromEEPROM(void)
{
    /* Читаем заголовок */
    uint8_t header_buf[sizeof(GradHeader_t)];
    HAL_StatusTypeDef status = AT24C64_ReadBytes(AT24C64_DEFAULT_ADDRESS,
                                                   GRAD_EEPROM_BASE,
                                                   header_buf,
                                                   sizeof(GradHeader_t));
    if (status != HAL_OK) {
        grad_state.valid = false;
        return status;
    }

    memcpy(&grad_state.header, header_buf, sizeof(GradHeader_t));

    /* Проверяем magic */
    if (grad_state.header.magic != GRAD_MAGIC) {
        grad_state.valid = false;
        return HAL_ERROR;
    }

    /* Проверяем количество точек */
    if (grad_state.header.points_count > GRAD_MAX_POINTS) {
        grad_state.header.points_count = GRAD_MAX_POINTS;
    }

    grad_state.actual_points = grad_state.header.points_count;

    /* Читаем массив объемов */
    if (grad_state.actual_points > 0) {
        uint32_t data_size = grad_state.actual_points * sizeof(float);
        uint8_t *vol_buf = (uint8_t *)grad_state.volumes;

        status = AT24C64_ReadBytes(AT24C64_DEFAULT_ADDRESS,
                                    GRAD_DATA_START,
                                    vol_buf,
                                    data_size);
        if (status != HAL_OK) {
            grad_state.valid = false;
            return status;
        }

        /* Проверяем CRC (если есть) */
        grad_state.crc32 = calculate_crc32(vol_buf, data_size);
    }

    grad_state.loaded = true;
    grad_state.valid = true;

    return HAL_OK;
}

HAL_StatusTypeDef Grad_SaveToEEPROM(void)
{
    if (!grad_state.loaded) return HAL_ERROR;

    /* Записываем заголовок */
    grad_state.header.magic = GRAD_MAGIC;
    grad_state.header.points_count = grad_state.actual_points;

    HAL_StatusTypeDef status = AT24C64_WriteBytes(AT24C64_DEFAULT_ADDRESS,
                                                    GRAD_EEPROM_BASE,
                                                    (uint8_t *)&grad_state.header,
                                                    sizeof(GradHeader_t));
    if (status != HAL_OK) return status;

    /* Записываем массив объемов */
    if (grad_state.actual_points > 0) {
        uint32_t data_size = grad_state.actual_points * sizeof(float);
        status = AT24C64_WriteBytes(AT24C64_DEFAULT_ADDRESS,
                                     GRAD_DATA_START,
                                     (uint8_t *)grad_state.volumes,
                                     data_size);
        if (status != HAL_OK) return status;
    }

    USART2_BufInit();
    USART2_BufPrint("[GRAD] Таблица сохранена: ");
    USART2_BufPrintInt(grad_state.actual_points);
    USART2_BufPrint(" точек\r\n");
    USART2_BufFlush();

    return HAL_OK;
}

HAL_StatusTypeDef Grad_Clear(void)
{
    memset(&grad_state.header, 0, sizeof(GradHeader_t));
    memset(grad_state.volumes, 0, sizeof(grad_state.volumes));
    grad_state.actual_points = 0;
    grad_state.loaded = false;
    grad_state.valid = false;

    /* Записываем нулевой magic */
    uint8_t zero_header[sizeof(GradHeader_t)] = {0};
    return AT24C64_WriteBytes(AT24C64_DEFAULT_ADDRESS,
                               GRAD_EEPROM_BASE,
                               zero_header,
                               sizeof(GradHeader_t));
}

HAL_StatusTypeDef Grad_WriteHeader(const GradHeader_t *header)
{
    if (header == NULL) return HAL_ERROR;

    memcpy(&grad_state.header, header, sizeof(GradHeader_t));
    grad_state.header.magic = GRAD_MAGIC;
    grad_state.actual_points = header->points_count;
    grad_state.loaded = true;

    /* Сразу сохраняем заголовок в EEPROM */
    return AT24C64_WriteBytes(AT24C64_DEFAULT_ADDRESS,
                               GRAD_EEPROM_BASE,
                               (uint8_t *)&grad_state.header,
                               sizeof(GradHeader_t));
}

HAL_StatusTypeDef Grad_ReadHeader(GradHeader_t *header)
{
    if (header == NULL) return HAL_ERROR;
    memcpy(header, &grad_state.header, sizeof(GradHeader_t));
    return HAL_OK;
}

HAL_StatusTypeDef Grad_WriteVolumes(uint16_t start_index,
                                     const float *volumes,
                                     uint16_t count)
{
    if (volumes == NULL) return HAL_ERROR;
    if (start_index + count > GRAD_MAX_POINTS) return HAL_ERROR;

    /* Копируем в RAM */
    memcpy(&grad_state.volumes[start_index], volumes, count * sizeof(float));

    /* Обновляем количество точек если нужно */
    if (start_index + count > grad_state.actual_points) {
        grad_state.actual_points = start_index + count;
        grad_state.header.points_count = grad_state.actual_points;
    }

    /* Записываем в EEPROM */
    uint32_t eeprom_addr = GRAD_DATA_START + start_index * sizeof(float);
    return AT24C64_WriteBytes(AT24C64_DEFAULT_ADDRESS,
                               eeprom_addr,
                               (uint8_t *)volumes,
                               count * sizeof(float));
}

HAL_StatusTypeDef Grad_ReadVolumes(uint16_t start_index,
                                    float *volumes,
                                    uint16_t count)
{
    if (volumes == NULL) return HAL_ERROR;
    if (start_index + count > GRAD_MAX_POINTS) return HAL_ERROR;

    /* Читаем из EEPROM */
    uint32_t eeprom_addr = GRAD_DATA_START + start_index * sizeof(float);
    HAL_StatusTypeDef status = AT24C64_ReadBytes(AT24C64_DEFAULT_ADDRESS,
                                                   eeprom_addr,
                                                   (uint8_t *)volumes,
                                                   count * sizeof(float));
    if (status == HAL_OK) {
        /* Обновляем RAM */
        memcpy(&grad_state.volumes[start_index], volumes, count * sizeof(float));
    }

    return status;
}

float Grad_InterpolateVolume(float level_m)
{
    if (!grad_state.valid || !grad_state.loaded || grad_state.actual_points < 2) {
        return NAN;
    }

    float h_start = grad_state.header.start_height_m;
    float h_step = grad_state.header.step_height_m;

    if (h_step <= 0.0f) return NAN;

    /* Проверяем границы */
    if (level_m <= h_start) return 0.0f;

    float h_end = h_start + h_step * (float)(grad_state.actual_points - 1);
    if (level_m >= h_end) return grad_state.header.tank_volume_m3;

    /* Находим индекс точки */
    float rel_h = (level_m - h_start) / h_step;
    uint16_t idx = (uint16_t)rel_h;

    if (idx >= grad_state.actual_points - 1) {
        return grad_state.volumes[grad_state.actual_points - 1];
    }

    /* Линейная интерполяция */
    float frac = rel_h - (float)idx;
    float v0 = grad_state.volumes[idx];
    float v1 = grad_state.volumes[idx + 1];

    return v0 + (v1 - v0) * frac;
}

float Grad_CalculateVolume(GradTankType_t tank_type,
                            float level_m,
                            float height_m,
                            float volume_m3)
{
    switch (tank_type) {
        case GRAD_TYPE_VERTICAL:
            return calc_volume_vertical(level_m, height_m, volume_m3);

        case GRAD_TYPE_HORIZ_FLAT:
            return calc_volume_horiz_flat(level_m, height_m, volume_m3);

        case GRAD_TYPE_HORIZ_ELLIPT:
            return calc_volume_horiz_ellipt(level_m, height_m, volume_m3);

        case GRAD_TYPE_BY_TABLE:
            return Grad_InterpolateVolume(level_m);

        default:
            return NAN;
    }
}

GradState_t* Grad_GetState(void)
{
    return &grad_state;
}

void Grad_UpdateModbusRegisters(void)
{
    if (grad_state.valid && grad_state.loaded) {
        /* Регистр 2014 - количество точек */
        ModBus_SetParameter_Float(MB_ADDR_CALIB_POINTS, (float)grad_state.actual_points);

        /* Регистр 2010 - высота резервуара */
        ModBus_SetParameter_Float(MB_ADDR_TANK_HEIGHT, grad_state.header.tank_height_m);

        /* Регистр 2012 - объем резервуара */
        ModBus_SetParameter_Float(MB_ADDR_TANK_VOLUME, grad_state.header.tank_volume_m3);

        /* Регистр 2008 - способ расчета (2 = по таблице) */
        ModBus_SetParameter_Float(MB_ADDR_TANK_GEOM, (float)GRAD_TYPE_BY_TABLE);
    }
}

void Grad_ProcessCommand(uint16_t cmd, float param)
{
    switch (cmd) {
        case 300:  /* Команда: начать загрузку таблицы */
        {
            USART2_Print("[GRAD] Начало загрузки таблицы\r\n");
            Grad_Clear();

            /* param содержит количество точек */
            if (param > 0 && param <= GRAD_MAX_POINTS) {
                GradHeader_t header = {0};
                header.magic = GRAD_MAGIC;
                header.points_count = (uint16_t)param;
                header.start_height_m = 0.0f;
                header.step_height_m = 0.01f;  /* По умолчанию 10 мм */
                Grad_WriteHeader(&header);

                ModBus_SetParameter_Int(MB_ADDR_COMMAND, 90);  /* Выполнено */
            } else {
                ModBus_SetParameter_Int(MB_ADDR_COMMAND, 0);   /* Отказ */
            }
            break;
        }

        case 301:  /* Команда: завершить загрузку таблицы */
        {
            USART2_Print("[GRAD] Завершение загрузки таблицы\r\n");
            if (grad_state.loaded) {
                Grad_SaveToEEPROM();
                Grad_UpdateModbusRegisters();
                ModBus_SetParameter_Int(MB_ADDR_COMMAND, 90);
            } else {
                ModBus_SetParameter_Int(MB_ADDR_COMMAND, 0);
            }
            break;
        }

        case 302:  /* Команда: очистить таблицу */
        {
            USART2_Print("[GRAD] Очистка таблицы\r\n");
            Grad_Clear();
            ModBus_SetParameter_Int(MB_ADDR_COMMAND, 90);
            break;
        }

        default:
            break;
    }
}
