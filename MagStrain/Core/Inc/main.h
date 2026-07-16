/* USER CODE BEGIN Header */
/*
@file           : main.h
@brief          : Заголовочный файл основного модуля ПМП-201Е
*/
/* USER CODE END Header */
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

#define TIMER_CLOCK_HZ          72000000.0f
#define TOF_TICK_US             0.0972f

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

/* === ФУНКЦИИ АЦП (нужны прототипы до использования) === */
uint32_t Read_ADC_Single(ADC_HandleTypeDef *hadc, uint32_t channel, uint32_t sampling_time);
uint32_t Read_ADC_Average(ADC_HandleTypeDef *hadc, uint32_t channel, uint32_t sampling_time, uint8_t samples);

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
#define CLIK_Pin                GPIO_PIN_1      /* PB1: Вход захвата TIM3_CH4 */
#define CLIK_GPIO_Port          GPIOB
#define Gen_Impuls_Pin          GPIO_PIN_5      /* PB5: Генерация импульса */
#define Gen_Impuls_GPIO_Port    GPIOB
#define Switch_In_impuls_Pin    GPIO_PIN_7      /* PB7: Переключение Rx/Tx */
#define Switch_In_impuls_GPIO_Port GPIOB

/* === Индикация === */
#define LED_BLUE_Pin            GPIO_PIN_12     /* PB12: Синий LED (heartbeat) */
#define LED_BLUE_GPIO_Port      GPIOB
#define LED_RED_Pin             GPIO_PIN_13     /* PB13: Красный LED (измерение) */
#define LED_RED_GPIO_Port       GPIOB

/* === Доп. функции === */
#define Check_OPA552_Pin        GPIO_PIN_8
#define Check_OPA552_GPIO_Port  GPIOA
#define ON_VCC_5_Pin            GPIO_PIN_6
#define ON_VCC_5_GPIO_Port      GPIOB

/* ==========================================================================
КОНСТАНТЫ ЗАХВАТА И ИЗМЕРЕНИЙ

========================================================================== */
#define MAX_PULSE_PAIRS             1           /* Макс. пар импульсов для захвата */
#define MAX_CAPTURED_PULSES         (MAX_PULSE_PAIRS * 2)  /* Фронтов = пар * 2 */
#define BLANKING_WINDOW_TICKS       65          /* ~6 мкс: игнорирование триггера */
#define DEAD_TIME_TICKS             140         /* ~17.5 мкс: задержка между импульсами (ИСПРАВЛЕНО!) */

/* ==========================================================================
ВНЕШНИЕ ПЕРЕМЕННЫЕ (экземпляры драйверов)
========================================================================== */
extern UART_HandleTypeDef huart1; /* USART1: Modbus RTU (RS-485) */
extern UART_HandleTypeDef huart2; /* USART2: Отладочный UART (115200) */
extern ADC_HandleTypeDef hadc1; /* ADC1: VREFINT, +24V */
extern ADC_HandleTypeDef hadc2; /* ADC2: +12V, +5V */
extern TIM_HandleTypeDef htim4; /* TIM4: (резерв) */

/* ★ Флаг игнорирования первого паразитного импульса ★ */
extern volatile uint8_t first_pulse_ignored;

/* ==========================================================================
ПРОТОТИПЫ ФУНКЦИЙ ИНИЦИАЛИЗАЦИИ (CubeMX style)
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
