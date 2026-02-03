/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
#include "modbus.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
extern volatile uint32_t capture_times[3];
extern volatile uint8_t capture_index;
extern volatile uint8_t measurement_done;
extern volatile uint16_t last_capture;

/* Внешние объявления из main.c */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
void NMI_Handler(void)
{
  while (1)
  {
  }
}

void HardFault_Handler(void)
{
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/******************************************************************************/

void EXTI1_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}

void TIM3_IRQHandler(void)
{
    /* Проверка флага захвата канала 4 */
    if (TIM3->SR & TIM_SR_CC4IF) {
        /* Чтение значения захвата */
        uint16_t capture = TIM3->CCR4;

        /* Сохранение времени захвата */
        if (capture_index < 3) {
            capture_times[capture_index] = capture;
            capture_index++;

            /* Проверка завершения измерения (3 импульса) */
            if (capture_index >= 3) {
                measurement_done = 1;
                TIM3->CR1 &= ~TIM_CR1_CEN;  // Остановка таймера
                TIM3->DIER &= ~TIM_DIER_CC4IE;  // Отключение прерывания
            }
        }

        /* Сброс флага прерывания */
        TIM3->SR &= ~TIM_SR_CC4IF;
    }

    /* Обработка переполнения (если нужно для длинных периодов) */
    if (TIM3->SR & TIM_SR_UIF) {
        last_capture += 0x10000;  // Учёт переполнения 16-битного счётчика
        TIM3->SR &= ~TIM_SR_UIF;
    }
}

void USART1_IRQHandler(void)
{
    // Используем стандартный обработчик HAL
    HAL_UART_IRQHandler(&huart1);
}

void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}

// Обработчики прерываний ADC (добавлены)
void ADC1_2_IRQHandler(void)
{
    HAL_ADC_IRQHandler(&hadc1);
    HAL_ADC_IRQHandler(&hadc2);
}
/**
  * @brief This function handles TIM3 global interrupt.
  */

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim4);
}
