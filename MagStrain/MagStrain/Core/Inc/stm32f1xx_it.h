#ifndef __STM32F1xx_IT_H
#define __STM32F1xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported functions prototypes ---------------------------------------------*/
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void EXTI1_IRQHandler(void);
void TIM3_IRQHandler(void);
void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
void ADC1_2_IRQHandler(void);  // Добавлено

#ifdef __cplusplus
}
#endif

#endif /* __STM32F1xx_IT_H */
