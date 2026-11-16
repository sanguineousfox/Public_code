/**
 * @file    stm32f1xx_it.c
 * @brief   Обработчики прерываний STM32F103.
 */
#include "main.h"
#include "stm32f1xx_it.h"
#include "modbus.h"
#include "utils.h"

/**
 * @brief Фиксирует два отдельных импульса, сформированных внешней схемой.
 *
 * Захват всегда выполняется по одному и тому же нарастающему фронту.
 * Никакого переключения полярности TIM3 в процессе измерения нет.
 *
 * Алгоритм:
 *  1. игнорировать события до окончания защитного окна;
 *  2. сохранить первый сформированный импульс как t1;
 *  3. если второй импульс появился, сохранить его как t2 и завершить запуск;
 *  4. проверку интервала и политику принятия замера применить в main.c.
 */
void TIM3_IRQHandler(void)
{
    uint32_t current_ticks;

    /* TIM3 CH4 постоянно настроен на нарастающий фронт. Внешняя схема уже
     * сформировала два отдельных цифровых импульса, поэтому полярность
     * захвата в обработчике никогда не переключается. */
    uint32_t status = TIM3->SR;
    if ((status & TIM_SR_CC4IF) == 0U) {
        return;
    }

    /* CCR4 содержит момент фронта, зафиксированный аппаратно независимо от
     * задержки входа в прерывание. */
    current_ticks = TIM3->CCR4;
    if (((status | TIM3->SR) & TIM_SR_UIF) != 0U) {
        TIM3->DIER &= ~TIM_DIER_CC4IE;
        return;
    }

    /* Чтение CCR4 уже сняло CC4IF. Новое событие может прийти после чтения. */
    if (((status | TIM3->SR) & TIM_SR_CC4OF) != 0U) {
        TIM3->SR = ~TIM_SR_CC4OF;
        if (current_ticks >= BLANKING_WINDOW_TICKS) {
            capture_overrun = true;
        }
    }

    /* Защитное окно отсекает наводку от задающего импульса. */
    if (current_ticks < BLANKING_WINDOW_TICKS) {
        return;
    }

    if (capture_count == 0U) {
        /* Фиксируем ПЕРВЫЙ сформированный импульс после blanking-окна.
         * Это время t1 будет использовано как ToF, если следующий импульс
         * подтвердит валидность пары. Поздние события за физическим концом
         * волновода не принимаются. */
        if (current_ticks > capture_max_tof_ticks) {
            tof_timeout = 1U;
            TIM3->DIER &= ~TIM_DIER_CC4IE;
            return;
        }

        captured_pulses[0] = current_ticks;
        capture_count = 1U;
        return;
    }

    /* Второй фронт за пределами окна не участвует даже при задержке main. */
    if (current_ticks - captured_pulses[0] <= CAPTURE_SECOND_PULSE_TIMEOUT_TICKS) {
        captured_pulses[1] = current_ticks;
        capture_count = 2U;
    }
    tof_measurement_done = 1U;
    TIM3->DIER &= ~TIM_DIER_CC4IE;
}

void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}

void USART2_IRQHandler(void)
{
#if (USART2_DEBUG_ENABLED != 0U)
    HAL_UART_IRQHandler(&huart2);
#endif
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == NULL) {
        return;
    }

    if (huart->Instance == USART1) {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
        __HAL_UART_CLEAR_FEFLAG(&huart1);
        __HAL_UART_CLEAR_NEFLAG(&huart1);
        __HAL_UART_CLEAR_OREFLAG(&huart1);
        ModBus_RestartRx();
    } else if (huart->Instance == USART2) {
        USART2_TxErrorCallback(huart);
    }
}

void NMI_Handler(void) { while (1) {} }
void HardFault_Handler(void) { while (1) {} }
void MemManage_Handler(void) { while (1) {} }
void BusFault_Handler(void) { while (1) {} }
void UsageFault_Handler(void) { while (1) {} }
void SVC_Handler(void) {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void) {}
void SysTick_Handler(void) { HAL_IncTick(); }
