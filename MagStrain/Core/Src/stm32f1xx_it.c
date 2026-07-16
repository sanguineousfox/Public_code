/* USER CODE BEGIN Header */
/**
  @file    stm32f1xx_it.c
  @brief   Interrupt Service Routines
  ★ ИСПРАВЛЕНО: убран dead_time который блокировал первый значащий импульс
  ★ ИСПРАВЛЕНО: остановка захвата после 2 импульсов (эхо не фиксируется)
*/
/* USER CODE END Header */
#include "main.h"
#include "stm32f1xx_it.h"

/* USER CODE BEGIN TD */
extern volatile uint32_t tof_capture_value;
extern volatile uint8_t tof_measurement_done;
extern volatile uint32_t captured_pulses[];
extern volatile uint8_t capture_count;
extern volatile uint8_t expected_pulse_pairs;
extern volatile uint32_t last_capture_cnt;

/* ★ Флаг игнорирования первого паразитного импульса ★ */
extern volatile uint8_t first_pulse_ignored;
/* USER CODE END TD */

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

void NMI_Handler(void) { while (1) { } }
void HardFault_Handler(void) { while (1) { } }
void MemManage_Handler(void) { while (1) { } }
void BusFault_Handler(void) { while (1) { } }
void UsageFault_Handler(void) { while (1) { } }
void SVC_Handler(void) { }
void DebugMon_Handler(void) { }
void PendSV_Handler(void) { }

void SysTick_Handler(void)
{
    HAL_IncTick();
}

/* ==========================================================================
ОБРАБОТЧИК ПРЕРЫВАНИЯ TIM3
★ ИСПРАВЛЕНО: убран dead_time - теперь захватываются все импульсы
★ ИСПРАВЛЕНО: остановка захвата после получения 2 импульсов
========================================================================== */
void TIM3_IRQHandler(void)
{
    if (TIM3->SR & TIM_SR_CC4IF) {
        uint32_t cnt_now = TIM3->CNT;

        /* ★ 1. ИГНОРИРУЕМ ПЕРВЫЙ ПАРАЗИТНЫЙ ИМПУЛЬС (~28 мкс) ★ */
        if (!first_pulse_ignored) {
            first_pulse_ignored = 1;
            TIM3->SR = 0;
            return;
        }

        /* ★ 2. ЕСЛИ УЖЕ ЗАХВАТИЛИ 2 ИМПУЛЬСА - СТОП (эхо не нужно) ★ */
        if (capture_count >= 2) {
            TIM3->DIER &= ~TIM_DIER_CC4IE;
            TIM3->SR = 0;
            return;
        }

        /* ★ 3. ЗАХВАТЫВАЕМ ИМПУЛЬС ★ */
        captured_pulses[capture_count] = cnt_now;
        capture_count++;

        /* 4. ToF = первый захваченный импульс */
        if (capture_count == 1) {
            tof_capture_value = captured_pulses[0];
        }

        /* 5. Если захватили 2 импульса - готово */
        if (capture_count >= 2) {
            tof_measurement_done = 1;
            TIM3->DIER &= ~TIM_DIER_CC4IE;
        }

        TIM3->SR = 0;
    }
}

void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}

void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}

void ADC1_2_IRQHandler(void)
{
    HAL_ADC_IRQHandler(&hadc1);
    HAL_ADC_IRQHandler(&hadc2);
}
