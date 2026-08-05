/**
 * @file    at24c64.c
 * @brief   Низкоуровневый драйвер EEPROM AT24C64 через I2C2.
 */
#include "at24c64.h"
#include "i2c_config.h"
#include <stddef.h>

/*
 * Признак фоновой записи страницы.
 *
 * В прежней реализации WP поднимался сразу после HAL_I2C_Mem_Write().
 * Эта функция завершает только передачу адреса и данных по I2C; внутренний
 * цикл программирования EEPROM продолжается ещё несколько миллисекунд.
 * На части микросхем раннее поднятие WP приводит к неполной/повреждённой
 * записи, хотя сама передача по I2C завершилась без ошибки.
 */
static uint8_t write_cycle_active = 0U;

static void AT24C64_SetWriteProtect(GPIO_PinState state)
{
    HAL_GPIO_WritePin(AT24C64_WP_PORT, AT24C64_WP_PIN, state);
}

static uint8_t AT24C64_RangeIsValid(uint16_t address, uint16_t size)
{
    uint32_t end_address;

    if (size == 0U) {
        return 0U;
    }

    end_address = (uint32_t)address + (uint32_t)size;
    return (end_address <= AT24C64_SIZE_BYTES) ? 1U : 0U;
}

HAL_StatusTypeDef AT24C64_IsReady(uint8_t device_address)
{
    return HAL_I2C_IsDeviceReady(&hi2c2,
                                 device_address,
                                 3U,
                                 AT24C64_READY_TIMEOUT_MS);
}

HAL_StatusTypeDef AT24C64_PollReady(uint8_t device_address)
{
    HAL_StatusTypeDef status =
        HAL_I2C_IsDeviceReady(&hi2c2, device_address, 1U, 1U);

    /*
     * ACK означает завершение внутреннего цикла программирования.
     * Только теперь разрешается вернуть аппаратную защиту записи.
     */
    if (status == HAL_OK && write_cycle_active != 0U) {
        AT24C64_SetWriteProtect(GPIO_PIN_SET);
        write_cycle_active = 0U;
    }

    return status;
}

HAL_StatusTypeDef AT24C64_WritePageBegin(uint8_t device_address,
                                         uint16_t memory_address,
                                         const uint8_t *data,
                                         uint16_t size)
{
    uint16_t page_offset;
    HAL_StatusTypeDef status;

    if (data == NULL || size == 0U ||
        !AT24C64_RangeIsValid(memory_address, size)) {
        return HAL_ERROR;
    }

    if (write_cycle_active != 0U) {
        return HAL_BUSY;
    }

    page_offset = (uint16_t)(memory_address % AT24C64_PAGE_SIZE_BYTES);
    if ((uint16_t)(page_offset + size) > AT24C64_PAGE_SIZE_BYTES) {
        return HAL_ERROR;
    }

    AT24C64_SetWriteProtect(GPIO_PIN_RESET);
    status = HAL_I2C_Mem_Write(&hi2c2,
                               device_address,
                               memory_address,
                               I2C_MEMADD_SIZE_16BIT,
                               (uint8_t *)data,
                               size,
                               10U);

    if (status == HAL_OK) {
        /* WP остаётся выключенной до успешного AT24C64_PollReady(). */
        write_cycle_active = 1U;
    } else {
        AT24C64_SetWriteProtect(GPIO_PIN_SET);
        write_cycle_active = 0U;
    }

    return status;
}

void AT24C64_AbortWriteCycle(void)
{
    AT24C64_SetWriteProtect(GPIO_PIN_SET);
    write_cycle_active = 0U;
}

HAL_StatusTypeDef AT24C64_Init(uint8_t device_address)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio.Pin = AT24C64_WP_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(AT24C64_WP_PORT, &gpio);

    write_cycle_active = 0U;
    AT24C64_SetWriteProtect(GPIO_PIN_SET);
    return AT24C64_IsReady(device_address);
}

HAL_StatusTypeDef AT24C64_ReadByte(uint8_t device_address,
                                   uint16_t memory_address,
                                   uint8_t *value)
{
    return AT24C64_ReadBytes(device_address, memory_address, value, 1U);
}

HAL_StatusTypeDef AT24C64_ReadBytes(uint8_t device_address,
                                    uint16_t memory_address,
                                    uint8_t *data,
                                    uint16_t size)
{
    if (data == NULL || !AT24C64_RangeIsValid(memory_address, size)) {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Read(&hi2c2,
                            device_address,
                            memory_address,
                            I2C_MEMADD_SIZE_16BIT,
                            data,
                            size,
                            AT24C64_IO_TIMEOUT_MS);
}

HAL_StatusTypeDef AT24C64_WriteByte(uint8_t device_address,
                                    uint16_t memory_address,
                                    uint8_t value)
{
    return AT24C64_WriteBytes(device_address, memory_address, &value, 1U);
}

HAL_StatusTypeDef AT24C64_WriteBytes(uint8_t device_address,
                                     uint16_t memory_address,
                                     const uint8_t *data,
                                     uint16_t size)
{
    uint16_t written = 0U;
    HAL_StatusTypeDef status = HAL_OK;

    if (data == NULL || !AT24C64_RangeIsValid(memory_address, size)) {
        return HAL_ERROR;
    }

    if (write_cycle_active != 0U) {
        return HAL_BUSY;
    }

    AT24C64_SetWriteProtect(GPIO_PIN_RESET);

    while (written < size) {
        uint16_t current_address = (uint16_t)(memory_address + written);
        uint16_t page_offset =
            (uint16_t)(current_address % AT24C64_PAGE_SIZE_BYTES);
        uint16_t page_space =
            (uint16_t)(AT24C64_PAGE_SIZE_BYTES - page_offset);
        uint16_t remaining = (uint16_t)(size - written);
        uint16_t chunk = (remaining < page_space) ? remaining : page_space;

        status = HAL_I2C_Mem_Write(&hi2c2,
                                   device_address,
                                   current_address,
                                   I2C_MEMADD_SIZE_16BIT,
                                   (uint8_t *)&data[written],
                                   chunk,
                                   AT24C64_IO_TIMEOUT_MS);
        if (status != HAL_OK) {
            break;
        }

        /* WP остаётся в нуле на всём внутреннем цикле записи. */
        status = AT24C64_IsReady(device_address);
        if (status != HAL_OK) {
            break;
        }

        written = (uint16_t)(written + chunk);
    }

    AT24C64_SetWriteProtect(GPIO_PIN_SET);
    return status;
}
