/* Minimal installer. On an interrupted install, retry the committed EEPROM
 * image before any jump. The last bootloader page is a persistent busy flag. */
#include "stm32f1xx_hal.h"
#include "fw_update.h"
#include "AT24C64.h"
#include "i2c_config.h"
#include <string.h>
#define INSTALL_FLAG 0x08002C00UL
static uint8_t buffer[128];
void SysTick_Handler(void) { HAL_IncTick(); }
void Error_Handler(void) { for (;;) {} }
static bool read_eeprom(uint16_t addr, void *p, uint16_t n)
{ return AT24C64_ReadBytes(AT24C64_DEFAULT_ADDRESS, addr, p, n) == HAL_OK; }
static bool erase_page(uint32_t address)
{
    FLASH_EraseInitTypeDef erase = {0}; uint32_t error;
    erase.TypeErase = FLASH_TYPEERASE_PAGES; erase.PageAddress = address; erase.NbPages = 1;
    return HAL_FLASHEx_Erase(&erase, &error) == HAL_OK;
}
static bool image_valid(const FW_Header *h)
{
    uint32_t crc = 0xFFFFFFFFUL, vectors[2];
    if (!read_eeprom(FW_IMAGE_BASE, vectors, 8U) ||
        !FW_VectorsValid(vectors[0], vectors[1], h->size)) return false;
    for (uint32_t i = 0; i < h->size; i += sizeof(buffer)) {
        uint16_t n = h->size - i > sizeof(buffer) ? sizeof(buffer) : h->size - i;
        if (!read_eeprom(FW_IMAGE_BASE + i, buffer, n)) return false;
        crc = FW_Crc(crc, buffer, n);
    }
    return (crc ^ 0xFFFFFFFFUL) == h->crc;
}
static bool install(const FW_Header *h)
{
    bool ok = false;
    HAL_FLASH_Unlock();
    /* This flag is in a dedicated page outside both code and application. */
    if (*(volatile uint16_t *)INSTALL_FLAG == 0xFFFFU &&
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, INSTALL_FLAG, 0U) != HAL_OK) goto done;
    for (uint32_t a = FW_APP_BASE; a < FW_APP_BASE + FW_APP_SIZE; a += 1024U)
        if (!erase_page(a)) goto done;
    for (uint32_t i = 0; i < h->size; i += sizeof(buffer)) {
        uint16_t n = h->size - i > sizeof(buffer) ? sizeof(buffer) : h->size - i;
        memset(buffer, 0xFF, sizeof(buffer));
        if (!read_eeprom(FW_IMAGE_BASE + i, buffer, n)) goto done;
        for (uint16_t j = 0; j < n; j += 2U) {
            uint16_t half = buffer[j] | ((uint16_t)buffer[j + 1U] << 8);
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, FW_APP_BASE + i + j, half) != HAL_OK) goto done;
        }
    }
    if ((FW_Crc(0xFFFFFFFFUL, (const uint8_t *)FW_APP_BASE, h->size) ^ 0xFFFFFFFFUL) != h->crc) goto done;
    if (!erase_page(INSTALL_FLAG)) goto done;
    ok = true;
done:
    HAL_FLASH_Lock(); return ok;
}
#ifndef FW_INSTALL_TEST
__attribute__((naked, noreturn)) static void jump(uint32_t sp __attribute__((unused)), uint32_t pc __attribute__((unused)))
{
    __asm volatile("msr msp, r0\n bx r1");
}
int main(void)
{
    SystemCoreClockUpdate();
    HAL_Init(); /* Reset HSI 8 MHz; no PLL; I2C timing derives from PCLK1. */
    if (MX_I2C2_Init() != HAL_OK) Error_Handler();
    (void)AT24C64_Init(AT24C64_DEFAULT_ADDRESS);
    for (;;) {
        FW_Header h;
        bool read_ok = read_eeprom(FW_META_BASE, &h, sizeof(h));
        bool install_started = *(volatile uint16_t *)INSTALL_FLAG != 0xFFFFU;
        if (read_ok && FW_HeaderValid(&h) && image_valid(&h)) {
            bool already_installed = !install_started &&
                (FW_Crc(0xFFFFFFFFUL, (const uint8_t *)FW_APP_BASE, h.size) ^ 0xFFFFFFFFUL) == h.crc;
            if (!already_installed && !install(&h)) { HAL_Delay(100U); continue; }
            /* Flash has passed CRC. An EEPROM cleanup failure must not
             * keep the verified application from starting. */
            uint32_t zero = 0U;
            (void)AT24C64_WriteBytes(AT24C64_DEFAULT_ADDRESS, FW_META_BASE,
                                     (uint8_t *)&zero, sizeof(zero));
        } else if (install_started) {
            /* Do not start a potentially partial image after an interrupted install. */
            HAL_Delay(100U); continue;
        }
        uint32_t sp = *(volatile uint32_t *)FW_APP_BASE;
        uint32_t pc = *(volatile uint32_t *)(FW_APP_BASE + 4U);
        if (!FW_VectorsValid(sp, pc, FW_APP_SIZE)) { HAL_Delay(100U); continue; }
        HAL_I2C_DeInit(&hi2c2);
        SysTick->CTRL = 0U; SysTick->LOAD = 0U; SysTick->VAL = 0U;
        __disable_irq();
        for (unsigned i = 0; i < 8; ++i) { NVIC->ICER[i] = 0xFFFFFFFFUL; NVIC->ICPR[i] = 0xFFFFFFFFUL; }
        SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;
        SCB->VTOR = FW_APP_BASE; __DSB(); __ISB();
        /* Application Reset_Handler starts with the reset interrupt mask. */
        __enable_irq(); jump(sp, pc);
    }
}

#endif /* FW_INSTALL_TEST */
