/**
 * @file           : main.h
 */
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

/* ==========================================================================
   ПРОТОТИПЫ ФУНКЦИЙ (пользовательские)
   ========================================================================== */
void Error_Handler(void);
void USART2_Print(const char *str);
void uint32_to_dec_str(uint32_t value, char *buffer);
void USART2_PrintHexByte(uint8_t byte);
void USART2_PrintHexBuffer(const uint8_t *buffer, uint16_t length);

/* === ФУНКЦИИ ИЗМЕРЕНИЙ === */
void TIM3_InputCapture_Init(void);
void generate_pulse_and_measure(void);
uint32_t measure_time_of_flight(void);
uint32_t measure_time_of_flight_test(void);
void Read_All_Voltages(void);
void Read_Temperature(void);
void Process_Measurement_Results(float tof_us, float position_mm, uint8_t signal_captured);

/* === ФУНКЦИИ АЦП === */
uint32_t Read_ADC_Single(ADC_HandleTypeDef *hadc, uint32_t channel, uint32_t sampling_time);
uint32_t Read_ADC_Average(ADC_HandleTypeDef *hadc, uint32_t channel, uint32_t sampling_time, uint8_t samples);

/* === БУФЕРНЫЙ ВЫВОД В USART2 (оптимизация) === */
void USART2_BufInit(void);
void USART2_BufPrint(const char *str);
void USART2_BufPrintInt(int32_t val);
void USART2_BufPrintFloat(float val);
void USART2_BufFlush(void);

/* === ФУНКЦИИ КАЛИБРОВКИ === */
void Process_Calibration_Command(uint16_t cmd);

/* ==========================================================================
   КОНФИГУРАЦИЯ GPIO (Pin Mapping)
   ========================================================================== */

/* === АЦП: Напряжения питания === */
#define Read_24V_Pin            GPIO_PIN_0
#define Read_24V_GPIO_Port      GPIOA
#define Read_12V_Pin            GPIO_PIN_1
#define Read_12V_GPIO_Port      GPIOA
#define Read_5V_Pin             GPIO_PIN_5
#define Read_5V_GPIO_Port       GPIOA

/* === Ультразвуковой датчик === */
#define CLIK_Pin                GPIO_PIN_1
#define CLIK_GPIO_Port          GPIOB
#define Gen_Impuls_Pin          GPIO_PIN_5
#define Gen_Impuls_GPIO_Port    GPIOB
#define Switch_In_impuls_Pin    GPIO_PIN_7
#define Switch_In_impuls_GPIO_Port GPIOB

/* === Индикация === */
#define LED_BLUE_Pin            GPIO_PIN_12
#define LED_BLUE_GPIO_Port      GPIOB
#define LED_RED_Pin             GPIO_PIN_13
#define LED_RED_GPIO_Port       GPIOB

/* === Доп. функции === */
#define Check_OPA552_Pin        GPIO_PIN_8
#define Check_OPA552_GPIO_Port  GPIOA
#define ON_VCC_5_Pin            GPIO_PIN_6
#define ON_VCC_5_GPIO_Port      GPIOB

/* ==========================================================================
   КОНСТАНТЫ ЗАХВАТА И ИЗМЕРЕНИЙ
   ========================================================================== */
#define MAX_PULSE_PAIRS             1
#define MAX_CAPTURED_PULSES         (MAX_PULSE_PAIRS * 2)
#define BLANKING_WINDOW_TICKS       650
#define DEAD_TIME_TICKS             670

/* ==========================================================================
   ВНЕШНИЕ ПЕРЕМЕННЫЕ
   ========================================================================== */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim4;

/* ==========================================================================
   ПРОТОТИПЫ ФУНКЦИЙ ИНИЦИАЛИЗАЦИИ
   ========================================================================== */
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_ADC1_Init(void);
void MX_ADC2_Init(void);
HAL_StatusTypeDef MX_I2C2_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
