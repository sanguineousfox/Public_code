/* USER CODE BEGIN Header */
/*
 * @file           : stm32f1xx_it.c
 * @brief          : Interrupt handlers
 */
/* USER CODE END Header */

#include "main.h"
#include "stm32f1xx_it.h"

/* ==========================================================================
ВНЕШНИЕ ПЕРЕМЕННЫЕ ИЗ main.c
========================================================================== */
extern volatile uint32_t captured_pulses[MAX_CAPTURED_PULSES];
extern volatile uint8_t capture_count;
extern volatile uint8_t tof_measurement_done;
extern volatile uint8_t tof_timeout;

/* ==========================================================================
ОБРАБОТЧИК ПРЕРЫВАНИЯ TIM3 (Input Capture Channel 4)
★ ИСПРАВЛЕНО: Мёртвое время 60 мкс для игнорирования паразитного импульса
========================================================================== */
void TIM3_IRQHandler(void)
{
    /* Проверяем, что прерывание именно от захвата канала 4 */
    if ((TIM3->SR & TIM_SR_CC4IF) != 0)
    {
        /* Сбрасываем флаг прерывания */
        TIM3->SR &= ~TIM_SR_CC4IF;

        /* Читаем захваченное значение счётчика */
        uint32_t current_cnt = TIM3->CCR4;

        /* ★ МЁРТВОЕ ВРЕМЯ (BLANKING WINDOW) ★
           Если импульс пришёл раньше, чем через 60 мкс после старта таймера,
           это паразитный выброс (звон) от излучателя. Игнорируем его. */
        if (current_cnt < BLANKING_WINDOW_TICKS) {
            return;
        }

        /* Если импульс валидный (после 60 мкс), сохраняем его */
        if (capture_count < MAX_CAPTURED_PULSES) {
            captured_pulses[capture_count] = current_cnt;
            capture_count++;
        }

        /* Если поймали минимально необходимую пару импульсов, завершаем измерение */
        if (capture_count >= 2) {
            tof_measurement_done = 1;

            /* Отключаем прерывание захвата до следующего измерения */
            TIM3->DIER &= ~TIM_DIER_CC4IE;
        }
    }
}
/* ==========================================================================
ДРУГИЕ ОБРАБОТЧИКИ ПРЕРЫВАНИЙ (оставь как есть)
========================================================================== */
void NMI_Handler(void) { while (1) {} }
void HardFault_Handler(void) { while (1) {} }
void MemManage_Handler(void) { while (1) {} }
void BusFault_Handler(void) { while (1) {} }
void UsageFault_Handler(void) { while (1) {} }
void SVC_Handler(void) {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void) {}
void SysTick_Handler(void) { HAL_IncTick(); }
