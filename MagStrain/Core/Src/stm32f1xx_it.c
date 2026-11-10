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
 *  1. игнорировать все события первые 80 мкс;
 *  2. сохранить первый сформированный импульс как t1;
 *  3. если второй импульс появился, сохранить его как t2 и завершить запуск;
 *  4. проверку интервала t2-t1 = 14...26 мкс выполнить в main.c.
 *
 * ВАЖНО: t1 является самостоятельным измерением ToF. Если t2 отсутствует
 * или его интервал неверен, main.c включает аварийный режим катушки, но
 * продолжает рассчитывать уровень по t1. ISR не ищет последующие отражения.
 */
void TIM3_IRQHandler(void)
{
    uint32_t current_ticks;

    /* TIM3 CH4 постоянно настроен на нарастающий фронт. Внешняя схема уже
     * сформировала два отдельных цифровых импульса, поэтому полярность
     * захвата в обработчике никогда не переключается. */
    if ((TIM3->SR & TIM_SR_CC4IF) == 0U) {
        return;
    }

    /* CCR4 содержит момент фронта, зафиксированный аппаратно независимо от
     * задержки входа в прерывание. */
    current_ticks = TIM3->CCR4;

    /* Сбрасываем флаг захвата и overcapture. Если overcapture возник, текущий
     * физический запуск все равно завершится проверкой пары в main.c. */
    TIM3->SR &= ~(TIM_SR_CC4IF | TIM_SR_CC4OF);

    /* Первые 80 мкс после отправки задающего импульса полностью закрыты.
     * Окно уменьшено со 100 мкс, потому что около верхней границы рабочего
     * диапазона истинный t1 приходит раньше 100 мкс. Наводка после 80 мкс
     * дополнительно отбраковывается проверкой интервала пары в main.c. */
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
#if (EMERGENCY_SINGLE_PULSE_DEBUG_MODE != 0U)
        /*
         * Аварийная отладка при отсутствующем втором канале: t1 достаточно.
         * Завершаем захват прямо в ISR и вообще не открываем окно ожидания t2.
         */
        tof_measurement_done = 1U;
        TIM3->DIER &= ~TIM_DIER_CC4IE;
#endif
        return;
    }

    /* Фиксируем ВТОРОЙ сформированный импульс и сразу завершаем запуск.
     * Интервал проверяется в main.c. Даже при неверном интервале t1 остается
     * измерением уровня, но выставляется авария катушки фиксации. Мы не
     * сдвигаем t1 на последующие отражения. */
    captured_pulses[1] = current_ticks;
    capture_count = 2U;
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
