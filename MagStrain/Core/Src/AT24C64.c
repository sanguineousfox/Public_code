/**
  @file    at24c64.c
  @brief   Драйвер EEPROM AT24C64 через I2C2 с диагностикой
*/

#include "at24c64.h"
#include "i2c_config.h"
#include "main.h"
#include <string.h>

extern I2C_HandleTypeDef hi2c2;

/**
  @brief Инициализация AT24C64 + WP пин с диагностикой
*/
HAL_StatusTypeDef AT24C64_Init(uint8_t dev_address)
{
    HAL_StatusTypeDef status;

    /* Настройка WP пина (PB8) как выход */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = AT24C64_WP_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(AT24C64_WP_PORT, &GPIO_InitStruct);

    /* Разрешаем запись (WP = 0) */
    AT24C64_WP_ENABLE_WRITE();

    /* ★ ДИАГНОСТИКА: Проверяем состояние I2C2 ★ */
    USART2_Print("[EEPROM] Проверка I2C2...\r\n");
    if (hi2c2.Instance == NULL) {
        USART2_Print("[EEPROM] ОШИБКА: I2C2 не инициализирован!\r\n");
        return HAL_ERROR;
    }

    /* ★ ДИАГНОСТИКА: Проверяем состояние шины I2C ★ */
    if (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) {
        USART2_Print("[EEPROM] ОШИБКА: I2C2 не в состоянии READY!\r\n");
        USART2_Print("[EEPROM] Состояние I2C2: ");
        USART2_BufInit();
        USART2_BufPrintInt(HAL_I2C_GetState(&hi2c2));
        USART2_BufPrint("\r\n");
        USART2_BufFlush();
        return HAL_ERROR;
    }

    USART2_Print("[EEPROM] I2C2 готов. Адрес устройства: 0x");
    USART2_BufInit();
    USART2_BufPrintHexByte(dev_address);  /* ★ ТЕПЕРЬ КОМПИЛИРУЕТСЯ ★ */
    USART2_BufPrint("\r\n");
    USART2_BufFlush();

    /* Проверяем доступность EEPROM */
    uint8_t dummy = 0;
    status = HAL_I2C_Mem_Read(&hi2c2, dev_address, 0x0000,
                              I2C_MEMADD_SIZE_16BIT, &dummy, 1, 100);

    if (status != HAL_OK) {
        USART2_Print("[EEPROM] ОШИБКА чтения: ");
        USART2_BufInit();
        USART2_BufPrintInt(status);
        USART2_BufPrint("\r\n");
        USART2_BufFlush();

        /* ★ ДИАГНОСТИКА: Проверяем ошибку I2C ★ */
        uint32_t error = HAL_I2C_GetError(&hi2c2);
        USART2_Print("[EEPROM] Код ошибки I2C: ");
        USART2_BufInit();
        USART2_BufPrintInt(error);
        USART2_BufPrint("\r\n");
        USART2_BufFlush();

        if (error == HAL_I2C_ERROR_AF) {
            USART2_Print("[EEPROM] NACK - устройство не отвечает!\r\n");
            USART2_Print("[EEPROM] Проверьте:\r\n");
            USART2_Print("  1. Подключена ли EEPROM\r\n");
            USART2_Print("  2. Правильность адреса (A0, A1, A2)\r\n");
            USART2_Print("  3. Pull-up резисторы на SDA/SCL\r\n");
        }
    } else {
        USART2_Print("[EEPROM] EEPROM найдена!\r\n");
    }

    return status;
}

/**
  @brief Чтение одного байта
*/
HAL_StatusTypeDef AT24C64_ReadByte(uint8_t dev_address, uint16_t mem_address, uint8_t *data)
{
    if (data == NULL || mem_address >= AT24C64_SIZE)
        return HAL_ERROR;

    return HAL_I2C_Mem_Read(&hi2c2, dev_address, mem_address,
                            I2C_MEMADD_SIZE_16BIT, data, 1, 100);
}

/**
  @brief Чтение нескольких байт
*/
HAL_StatusTypeDef AT24C64_ReadBytes(uint8_t dev_address, uint16_t mem_address,
                                    uint8_t *data, uint16_t size)
{
    if (data == NULL || mem_address >= AT24C64_SIZE ||
        (mem_address + size) > AT24C64_SIZE)
        return HAL_ERROR;

    return HAL_I2C_Mem_Read(&hi2c2, dev_address, mem_address,
                            I2C_MEMADD_SIZE_16BIT, data, size, 100);
}

/**
  @brief Запись одного байта
*/
HAL_StatusTypeDef AT24C64_WriteByte(uint8_t dev_address, uint16_t mem_address, uint8_t data)
{
    if (mem_address >= AT24C64_SIZE)
        return HAL_ERROR;

    AT24C64_WP_ENABLE_WRITE();
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c2, dev_address, mem_address,
                                                 I2C_MEMADD_SIZE_16BIT, &data, 1, 100);
    if (status == HAL_OK) {
        HAL_Delay(AT24C64_WRITE_DELAY_MS);
    }
    return status;
}

/**
  @brief Запись нескольких байт (с обработкой границ страниц 32 байта)
*/
HAL_StatusTypeDef AT24C64_WriteBytes(uint8_t dev_address, uint16_t mem_address,
                                     uint8_t *data, uint16_t size)
{
    if (data == NULL || mem_address >= AT24C64_SIZE ||
        (mem_address + size) > AT24C64_SIZE || size == 0)
        return HAL_ERROR;

    AT24C64_WP_ENABLE_WRITE();
    uint16_t written = 0;
    uint16_t addr = mem_address;

    while (written < size) {
        uint16_t page_offset = addr % AT24C64_PAGE_SIZE;
        uint16_t space_in_page = AT24C64_PAGE_SIZE - page_offset;
        uint16_t chunk_size = (size - written) < space_in_page ?
                             (size - written) : space_in_page;

        HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c2, dev_address, addr,
                                                     I2C_MEMADD_SIZE_16BIT,
                                                     &data[written], chunk_size, 100);
        if (status != HAL_OK)
            return status;
        HAL_Delay(AT24C64_WRITE_DELAY_MS);

        written += chunk_size;
        addr += chunk_size;
    }
    return HAL_OK;
}

/**
  @brief Ожидание готовности EEPROM
*/
HAL_StatusTypeDef AT24C64_WaitReady(uint8_t dev_address)
{
    uint32_t timeout = 20;
    while (timeout--) {
        uint8_t dummy = 0;
        if (HAL_I2C_Mem_Read(&hi2c2, dev_address, 0x0000,
                             I2C_MEMADD_SIZE_16BIT, &dummy, 1, 50) == HAL_OK) {
            return HAL_OK;
        }
        HAL_Delay(1);
    }
    return HAL_ERROR;
}

/* =========================================================================
ФУНКЦИИ ДЛЯ МОДБУС-ПАРАМЕТРОВ
========================================================================= */
static uint16_t AT24C64_FloatAddr(uint16_t mb_addr)
{
    return EEPROM_FLOAT_BASE + (mb_addr * 4);
}

static uint16_t AT24C64_IntAddr(uint16_t mb_addr)
{
    return EEPROM_INT_BASE + (mb_addr * 2);
}

HAL_StatusTypeDef AT24C64_SaveFloatParam(uint16_t mb_addr, float value)
{
    uint16_t eeprom_addr = AT24C64_FloatAddr(mb_addr);
    uint8_t buf[4];
    memcpy(buf, &value, 4);
    return AT24C64_WriteBytes(AT24C64_DEFAULT_ADDRESS, eeprom_addr, buf, 4);
}

HAL_StatusTypeDef AT24C64_LoadFloatParam(uint16_t mb_addr, float *value)
{
    uint16_t eeprom_addr = AT24C64_FloatAddr(mb_addr);
    uint8_t buf[4];
    HAL_StatusTypeDef status = AT24C64_ReadBytes(AT24C64_DEFAULT_ADDRESS,
                                                 eeprom_addr, buf, 4);
    if (status == HAL_OK) {
        memcpy(value, buf, 4);
    }
    return status;
}

HAL_StatusTypeDef AT24C64_SaveIntParam(uint16_t mb_addr, uint16_t value)
{
    uint16_t eeprom_addr = AT24C64_IntAddr(mb_addr);
    uint8_t buf[2];
    buf[0] = (value >> 8) & 0xFF;
    buf[1] = value & 0xFF;
    return AT24C64_WriteBytes(AT24C64_DEFAULT_ADDRESS, eeprom_addr, buf, 2);
}

HAL_StatusTypeDef AT24C64_LoadIntParam(uint16_t mb_addr, uint16_t *value)
{
    uint16_t eeprom_addr = AT24C64_IntAddr(mb_addr);
    uint8_t buf[2];
    HAL_StatusTypeDef status = AT24C64_ReadBytes(AT24C64_DEFAULT_ADDRESS,
                                                 eeprom_addr, buf, 2);
    if (status == HAL_OK) {
        *value = ((uint16_t)buf[0] << 8) | buf[1];
    }
    return status;
}

/* ★ ИСПРАВЛЕНО: Увеличен лимит с 256 до 260 регистров ★ */
HAL_StatusTypeDef AT24C64_SaveAllRegisters(uint16_t *regs, uint16_t count)
{
    if (regs == NULL || count > 400)  /* ★ Было 256, стало 400 ★ */
        return HAL_ERROR;

    static uint8_t buf[800];  /* ★ Static для экономии стека ★ */
    for (uint16_t i = 0; i < count; i++) {
        buf[i * 2] = (regs[i] >> 8) & 0xFF;
        buf[i * 2 + 1] = regs[i] & 0xFF;
    }
    return AT24C64_WriteBytes(AT24C64_DEFAULT_ADDRESS, EEPROM_REGS_BASE, buf, count * 2);
}


/* ★ ИСПРАВЛЕНО: Увеличен лимит с 256 до 260 регистров ★ */
HAL_StatusTypeDef AT24C64_LoadAllRegisters(uint16_t *regs, uint16_t count)
{
    if (regs == NULL || count > 400)  /* ★ Было 256, стало 400 ★ */
        return HAL_ERROR;

    static uint8_t buf[800];  /* ★ Static для экономии стека ★ */
    HAL_StatusTypeDef status = AT24C64_ReadBytes(AT24C64_DEFAULT_ADDRESS,
                                                 EEPROM_REGS_BASE, buf, count * 2);
    if (status == HAL_OK) {
        for (uint16_t i = 0; i < count; i++) {
            regs[i] = ((uint16_t)buf[i * 2] << 8) | buf[i * 2 + 1];
        }
    }
    return status;
}

HAL_StatusTypeDef AT24C64_Format(void)
{
    uint8_t magic[4];
    uint32_t m = EEPROM_MAGIC_VALUE;
    memcpy(magic, &m, 4);
    return AT24C64_WriteBytes(AT24C64_DEFAULT_ADDRESS, EEPROM_MAGIC_ADDR, magic, 4);
}

uint8_t AT24C64_IsFormatted(void)
{
    uint8_t buf[4];
    if (AT24C64_ReadBytes(AT24C64_DEFAULT_ADDRESS, EEPROM_MAGIC_ADDR, buf, 4) != HAL_OK)
        return 0;

    uint32_t magic;
    memcpy(&magic, buf, 4);
    return (magic == EEPROM_MAGIC_VALUE) ? 1 : 0;
}
