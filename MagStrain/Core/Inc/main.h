/**
 * @file    main.h
 * @brief   Общие определения аппаратной платформы и измерительного тракта.
 *
 * ВАЖНО ДЛЯ НАСТРОЙКИ ФИКСАЦИИ:
 * Все физические временные границы захвата собраны в одном месте ниже.
 * Если материал волновода изменится, в первую очередь корректируются
 * CAPTURE_PAIR_INTERVAL_MIN_US и CAPTURE_PAIR_INTERVAL_MAX_US.
 */
#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* ADC */
#define Read_24V_Pin                GPIO_PIN_0
#define Read_24V_GPIO_Port          GPIOA
#define Read_12V_Pin                GPIO_PIN_1
#define Read_12V_GPIO_Port          GPIOA
#define Read_5V_Pin                 GPIO_PIN_5
#define Read_5V_GPIO_Port           GPIOA

/* Измерительный тракт */
#define CLIK_Pin                    GPIO_PIN_1
#define CLIK_GPIO_Port              GPIOB
#define Gen_Impuls_Pin              GPIO_PIN_5
#define Gen_Impuls_GPIO_Port        GPIOB
#define Switch_In_impuls_Pin        GPIO_PIN_7
#define Switch_In_impuls_GPIO_Port  GPIOB

/* Индикация */
#define LED_BLUE_Pin                GPIO_PIN_12
#define LED_BLUE_GPIO_Port          GPIOB
#define LED_RED_Pin                 GPIO_PIN_13
#define LED_RED_GPIO_Port           GPIOB

/* RS-485 */
#define RS485_CTRL_Pin              GPIO_PIN_0
#define RS485_CTRL_GPIO_Port        GPIOB

/* Дополнительные цепи */
#define Check_OPA552_Pin            GPIO_PIN_8
#define Check_OPA552_GPIO_Port      GPIOA
#define ON_VCC_5_Pin                GPIO_PIN_6
#define ON_VCC_5_GPIO_Port          GPIOB

/* ==========================================================================
 * ПАРАМЕТРЫ АППАРАТНОЙ ФИКСАЦИИ TIM3 CH4
 * ========================================================================== */

/* TIM3 тактируется от 72 МГц и имеет PSC=6, поэтому частота счетчика:
 * 72 000 000 / (6 + 1) = 10 285 714 Гц, один тик примерно 0,09722 мкс. */
#define TIM3_CAPTURE_FREQUENCY_HZ   10285714UL
#define TIM3_CAPTURE_TICK_US        0.097222222f

/* Постоянная задержка электронного тракта от момента запуска PB5 до
 * срабатывания цифрового входа TIM3, не относящаяся к распространению
 * крутильной волны по звукопроводу. В нее входят приемная катушка,
 * усилительные каскады, фильтры и достижение порога входа STM32.
 *
 * Значение 18,00 мкс определено совместно со скоростью 2841,37 м/с по двум
 * контрольным положениям магнита. Оно вычитается только при переводе сырого
 * ToF в расстояние. Из интервала t2-t1 это значение НЕ вычитается, поэтому
 * проверка исправности пары импульсов 14...26 мкс остается без изменений. */
#define ELECTRONICS_DELAY_US        18.00f

/* Верхняя невалидная зона звукопровода около электронной головки.
 * Для текущей конструкции с никелевым участком магнит нельзя считать
 * корректно ближе 240 мм к приёмной части. Поэтому при общей длине
 * звукопровода 1040 мм максимальный измеряемый уровень от дна равен
 * 1040 - 240 = 800 мм. В дальнейшем значение можно заменить на 200 мм
 * для окончательного звукопровода из Nispan. */
#define UPPER_INVALID_ZONE_MM       220.00f

/* Калибровка измерительного канала по пяти опорным положениям.
 *
 * Команды 01 и 02 задают реальные пределы измерения 0 % и 100 %.
 * Команды 11/12/13 задают три промежуточные риски 260/520/780 мм.
 * Во всех пяти точках в EEPROM сохраняется СЫРОЕ время ToFraw. Поэтому
 * рабочий расчёт уровня после полной калибровки не зависит от принятой
 * скорости Nispan и от постоянной задержки электронного тракта.
 *
 * Полная последовательность:
 *   01 -> 11 -> 12 -> 13 -> 02.
 *
 * Команда 01 начинает новый цикл и очищает валидность старой таблицы.
 * Таблица включается только при маске 0x001F и строго убывающем ToFraw:
 * t0% > t260 > t520 > t780 > t100%.
 * Это одновременно обнаруживает переключение на ложный импульс в верхней
 * зоне: при нарушении монотонности команда завершается ошибкой. */
#define SENSOR_CAL_LEVEL_260_MM       260.00f
#define SENSOR_CAL_LEVEL_520_MM       520.00f
#define SENSOR_CAL_LEVEL_780_MM       780.00f
#define SENSOR_CAL_POINT_LOW_BIT      0x0001U
#define SENSOR_CAL_POINT_260_BIT      0x0002U
#define SENSOR_CAL_POINT_520_BIT      0x0004U
#define SENSOR_CAL_POINT_780_BIT      0x0008U
#define SENSOR_CAL_POINT_HIGH_BIT     0x0010U
#define SENSOR_CAL_FULL_MASK          0x001FU
#define SENSOR_CAL_STORAGE_TAG        0xA500U
#define SENSOR_CAL_STORAGE_TAG_MASK   0xFF00U

/* Перевод целого количества микросекунд в тики таймера.
 * Для нижних границ применяем округление вверх, для верхних — вниз. */
#define CAPTURE_US_TO_TICKS_CEIL(us) \
    ((((uint32_t)(us) * TIM3_CAPTURE_FREQUENCY_HZ) + 999999UL) / 1000000UL)
#define CAPTURE_US_TO_TICKS_FLOOR(us) \
    (((uint32_t)(us) * TIM3_CAPTURE_FREQUENCY_HZ) / 1000000UL)

/* Первые 100 мкс после задающего импульса полностью закрыты для фиксации.
 * В этом интервале находится сильная электромагнитная наводка с волновода. */
#define CAPTURE_BLANKING_TIME_US       100U
#define BLANKING_WINDOW_TICKS          CAPTURE_US_TO_TICKS_CEIL(CAPTURE_BLANKING_TIME_US)

/*
 * Внешняя аппаратная схема уже преобразует значащий аналоговый отклик
 * в ДВА отдельных цифровых импульса. TIM3 фиксирует только нарастающий
 * фронт каждого сформированного импульса. Полярность захвата не меняется.
 *
 * Первый захват — время первого сформированного импульса t1.
 * Второй захват — время второго сформированного импульса t2.
 * Валидность пары определяется интервалом t2 - t1.
 */
#define CAPTURE_PAIR_INTERVAL_MIN_US   12U
#define CAPTURE_PAIR_INTERVAL_MAX_US   26U
#define MIN_CLICK_WIDTH_TICKS          CAPTURE_US_TO_TICKS_CEIL(CAPTURE_PAIR_INTERVAL_MIN_US)
#define MAX_CLICK_WIDTH_TICKS          CAPTURE_US_TO_TICKS_FLOOR(CAPTURE_PAIR_INTERVAL_MAX_US)

/* К расчетному времени прохождения по всей длине волновода добавляется запас.
 * Он допускает задержку аналогового тракта, но отсекает ложные пары на 850–970 мкс
 * при длине волновода 1,0 м, которые наблюдались в предыдущей версии. */
#define CAPTURE_MAX_TOF_MARGIN_US      60U

/* Частота возбуждения волновода строго ограничена 10 Гц.
 * Период между двумя фронтами PB5 не может быть меньше 100 мс даже при
 * повторном вызове измерительной функции из команды калибровки. */
#define EXCITATION_FREQUENCY_HZ         10U
#define EXCITATION_PERIOD_MS            (1000U / EXCITATION_FREQUENCY_HZ)

/* После первого импульса пары второй должен появиться не позднее верхней
 * границы 26 мкс. Добавочный запас 5 мкс учитывает задержку выполнения
 * фонового кода, но не допускает ожидания поздних отражений. */
#define CAPTURE_SECOND_PULSE_GRACE_US   5U
#define CAPTURE_SECOND_PULSE_TIMEOUT_TICKS \
    CAPTURE_US_TO_TICKS_CEIL(CAPTURE_PAIR_INTERVAL_MAX_US + \
                             CAPTURE_SECOND_PULSE_GRACE_US)

/* Фильтр уровня использует скользящее окно из 11 последовательных запусков.
 * После заполнения окна новый результат формируется при каждом следующем
 * запуске, то есть с частотой до 10 Гц, а не один раз за отдельную пачку. */
#define MEASUREMENT_REQUIRED_SAMPLES    11U

/* Статус быстрого снимка измерений. */
#define MEASUREMENT_STATUS_VALID         0x0001U
#define MEASUREMENT_STATUS_SINGLE_PULSE  0x0002U
#define MEASUREMENT_STATUS_COIL_FAULT    0x0004U

/* Код аварии в Modbus-регистре MB_ADDR_ERROR_CODE (2416).
 * Измерение при этом продолжается по первому входному импульсу. */
#define MEASUREMENT_ERROR_NONE           0x0000U
#define MEASUREMENT_ERROR_CAPTURE_COIL   0x0101U

/* После сортировки удаляются ровно один минимум и один максимум. Разброс
 * оставшихся девяти значений обязан быть небольшим. При превышении порога
 * серия отвергается полностью, а последнее корректное значение остается в RAM.
 * 4 мкс при c=2841,37 м/с соответствуют примерно 11,4 мм полного диапазона. */
#define MEASUREMENT_MAX_SPREAD_US      4U
#define MEASUREMENT_MAX_SPREAD_TICKS   CAPTURE_US_TO_TICKS_CEIL(MEASUREMENT_MAX_SPREAD_US)

#define MAX_PULSE_PAIRS                1U
#define MAX_CAPTURED_PULSES            (MAX_PULSE_PAIRS * 2U)

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

extern volatile uint32_t captured_pulses[MAX_CAPTURED_PULSES];
extern volatile uint8_t capture_count;
extern volatile uint8_t tof_measurement_done;
extern volatile uint8_t tof_timeout;

/* Верхняя допустимая граница времени первого сформированного импульса в тиках TIM3.
 * Она рассчитывается перед серией по длине волновода и скорости волны. */
extern volatile uint32_t capture_max_tof_ticks;

void Error_Handler(void);
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_ADC1_Init(void);
void MX_ADC2_Init(void);
HAL_StatusTypeDef MX_I2C2_Init(void);

void TIM3_InputCapture_Init(void);
void generate_pulse_and_measure(void);
uint32_t measure_time_of_flight(void);
uint32_t measure_time_of_flight_test(void);
void Read_All_Voltages(void);
void Read_Temperature(void);
void Process_Measurement_Results(float tof_us,
                                 float position_mm,
                                 uint8_t signal_was_captured);
/**
 * @brief Выполняет команду управления. Для 01/02 и 223 итоговый код 90
 *        устанавливается только после подтвержденной записи EEPROM.
 */
void Process_Calibration_Command(uint16_t command);

uint32_t Read_ADC_Single(ADC_HandleTypeDef *hadc,
                         uint32_t channel,
                         uint32_t sampling_time);
uint32_t Read_ADC_Average(ADC_HandleTypeDef *hadc,
                          uint32_t channel,
                          uint32_t sampling_time,
                          uint8_t samples);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */
