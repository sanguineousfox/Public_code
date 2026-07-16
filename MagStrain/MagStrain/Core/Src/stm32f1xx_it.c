/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "stm32f1xx_it.h"

/* Прототипы внешних переменных, которые используются в обработчиках прерываний.
   Они должны быть объявлены как extern или определены глобально (обычно в main.c). */
extern volatile uint8_t tof_measurement_done;
extern volatile uint32_t tof_capture_value;
extern volatile uint8_t dead_time_active;
extern volatile uint32_t last_capture_cnt;
extern volatile uint8_t capture_count;
extern volatile uint8_t expected_pulse_pairs;

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

/**
 * @brief TIM3 Interrupt Handler (для захвата импульсов)
 */
void TIM3_IRQHandler(void)
{
  if (TIM3->SR & TIM_SR_CC4IF) {
      uint32_t cnt_now = TIM3->CNT;

      /* 1. Проверка мёртвого времени 65 мкс ТОЛЬКО перед нечётными */
      if (dead_time_active) {
          uint32_t elapsed;
          if (cnt_now >= last_capture_cnt) {
              elapsed = cnt_now - last_capture_cnt;
          } else {
              elapsed = (0xFFFF - last_capture_cnt) + cnt_now;
          }

          if (elapsed < DEAD_TIME_TICKS) { // Убедитесь, что DEAD_TIME_TICKS объявлен где-то (например в main.h или constants)
              TIM3->SR = 0;
              return;
          }

          dead_time_active = 0;
      }

      /* 2. Сохраняем импульс в массив */
      if (capture_count < MAX_CAPTURED_PULSES) { // Убедитесь, что MAX_CAPTURED_PULSES объявлен
          captured_pulses[capture_count] = cnt_now;
          capture_count++;
      }

      /* 3. Запоминаем время */
      last_capture_cnt = cnt_now;

      /* 4. Включаем мёртвое время ТОЛЬКО после чётных (2, 4) */
      if ((capture_count % 2) == 0) {
          dead_time_active = 1;
      }

      /* 5. ToF = первый импульс */
      if (capture_count >= 1) {
          tof_capture_value = captured_pulses[0];
      }

      /* 6. Проверка количества импульсов (4 импульса) */
      uint8_t expected_pulses = expected_pulse_pairs * 2;
      if (capture_count >= expected_pulses) {
          tof_measurement_done = 1;
      }

      /* 7. Сброс флага */
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
