#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

void Error_Handler(void);
void USART2_Print(const char* str);
void uint32_to_dec_str(uint32_t value, char* buffer);
void USART2_PrintHexByte(uint8_t byte);
void USART2_PrintHexBuffer(const uint8_t* buffer, uint16_t length);

#define Read_24V_Pin GPIO_PIN_0
#define Read_24V_GPIO_Port GPIOA
#define Read_12V_Pin GPIO_PIN_1
#define Read_12V_GPIO_Port GPIOA
#define Read_5V_Pin GPIO_PIN_5
#define Read_5V_GPIO_Port GPIOA
#define CLIK_Pin GPIO_PIN_1
#define CLIK_GPIO_Port GPIOB
#define LED_BLUE_Pin GPIO_PIN_12
#define LED_BLUE_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_13
#define LED_RED_GPIO_Port GPIOB
#define Check_OPA552_Pin GPIO_PIN_8
#define Check_OPA552_GPIO_Port GPIOA
#define Gen_Impuls_Pin GPIO_PIN_5
#define Gen_Impuls_GPIO_Port GPIOB
#define ON_VCC_5_Pin GPIO_PIN_6
#define ON_VCC_5_GPIO_Port GPIOB
#define Switch_In_impuls_Pin GPIO_PIN_7
#define Switch_In_impuls_GPIO_Port GPIOB

/* === КОНСТАНТЫ ЗАХВАТА === */
#define MAX_PULSE_PAIRS             4
#define MAX_CAPTURED_PULSES         (MAX_PULSE_PAIRS * 2)
#define BLANKING_WINDOW_TICKS       65     /* 50 мкс — пропускаем триггер */
#define DEAD_TIME_TICKS             670     /* 65 мкс — между КАЖДЫМИ двумя импульсами */

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim4;

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
