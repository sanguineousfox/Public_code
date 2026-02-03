#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "modbus.h"

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void USART2_Print(const char* str);
void uint32_to_dec_str(uint32_t value, char* buffer);
void float_to_str(float value, char* buffer, int decimals);
void USART2_PrintHexByte(uint8_t byte);
void USART2_PrintHexBuffer(const uint8_t* buffer, uint16_t length);
void USART2_PrintModBusCommand(const uint8_t* data, uint16_t length);
void USART2_PrintModBusResponse(const uint8_t* data, uint16_t length);
void ModBus_DebugFrame(const uint8_t* frame, uint16_t length, const char* prefix);

/* Private defines -----------------------------------------------------------*/
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

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
extern TIM_HandleTypeDef htim4;
