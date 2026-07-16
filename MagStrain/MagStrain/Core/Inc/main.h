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
#define Read_24V_Pin      GPIO_PIN_0
#define Read_24V_GPIO_Port  GPIOA
#define Read_12V_Pin      GPIO_PIN_1
#define Read_12V_GPIO_Port GPIOA
#define Read_5V_Pin       GPIO_PIN_5
#define Read_5V_GPIO_Port GPIOA
#define CLIK_Pin          GPIO_PIN_1
#define CLIK_GPIO_Port    GPIOB
#define LED_BLUE_Pin      GPIO_PIN_12
#define LED_BLUE_GPIO_Port GPIOB
#define LED_RED_Pin       GPIO_PIN_13
#define LED_RED_GPIO_Port GPIOB
#define Gen_Impuls_Pin    GPIO_PIN_5
#define Gen_Impuls_GPIO_Port GPIOB
#define ON_VCC_5_Pin      GPIO_PIN_6
#define ON_VCC_5_GPIO_Port GPIOB
#define Switch_In_impuls_Pin GPIO_PIN_7
#define Switch_In_impuls_GPIO_Port GPIOB

/* Глобальные константы для таймеров, DMA и прерываний (для MSP) */
#define DEAD_TIME_TICKS      64       // Мёртвое время в тиках (~65 мкс при 10.28MHz)
#define MAX_CAPTURED_PULSES  4        // Максимальное количество импульсов для измерения
#define CAPTURE_COUNT        0        // Счетчик захвата (инициализируется в MSP или main.c)

/* Объявление внешних переменных для прерываний */
extern volatile uint32_t tof_capture_value;     /* Значение захваченного таймера TIM3 */
extern volatile uint8_t  tof_measurement_done;  /* Флаг завершения измерения TOF */
extern volatile uint32_t captured_pulses[];     /* Массив для хранения временных меток импульсов */
extern volatile uint8_t  capture_count;         /* Счетчик захваченных импульсов */
extern volatile uint8_t  expected_pulse_pairs;  /* Ожидаемое количество пар импульсов */
extern volatile uint32_t last_capture_cnt;      /* Последняя запись таймера (для мёртвого времени) */
extern volatile uint8_t  dead_time_active;      /* Флаг активного "мёртвого" времени */

/* Внешние объекты для TIM4 (если используется, но в коде мы используем TIM3) */
extern TIM_HandleTypeDef htim4;

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
