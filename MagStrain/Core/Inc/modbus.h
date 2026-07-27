/**
 * @file    modbus.h
 * @brief   Карта Modbus RTU ПМП-201Е по таблице Е.4 руководства
 *          СЕНС.421411.028РЭ.
 *
 * ВАЖНО: порядок слов float32 соответствует таблице Е.3 руководства:
 *   - по базовому адресу передается МЛАДШЕЕ 16-битное слово;
 *   - по следующему адресу передается СТАРШЕЕ 16-битное слово;
 *   - внутри каждого регистра байты передаются стандартно: MSB, затем LSB.
 *
 * Пример IEEE754 0x4634D480:
 *   address     = 0xD480;   // младшее слово
 *   address + 1 = 0x4634;   // старшее слово
 *
 * Дополнительные адреса 2088 и 2094 оставлены как расширение данной прошивки.
 * Они не входят в таблицу Е.4, но нужны для периода измерения и скорости волны.
 */
#ifndef MODBUS_H
#define MODBUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define MODBUS_DEFAULT_ADDRESS          1U
#define MODBUS_DEFAULT_BAUDRATE         19200U
#define MODBUS_DEFAULT_BAUD_CODE        5U   /* Код 5 по таблице Е.4 = 19200 бит/с. */
#define MODBUS_DEFAULT_PARITY_CODE      0U   /* Код 0 по таблице Е.4 = 8N1. */
#define MODBUS_BUFFER_SIZE              256U
#define MODBUS_FLOAT_WORD_COUNT         2U
#define MODBUS_INVALID_INDEX            0xFFFFU

/* ========================================================================== */
/* Таблица Е.4: измеряемые параметры int16                                    */
/* ========================================================================== */
#define MB_ADDR_LEVEL_INT           1U    /* h: уровень жидкости, мм, int16. */
#define MB_ADDR_TEMP_INT            2U    /* t°: температура жидкости, 0.01 °C, int16. */
#define MB_ADDR_PERCENT_INT         3U    /* %: заполнение резервуара, 0.01 %, int16. */
#define MB_ADDR_VOLUME_INT          4U    /* U: объем жидкости, 10 дм3, int16. */
#define MB_ADDR_MASS_INT            5U    /* G: масса продукта, 10 кг, int16. */
#define MB_ADDR_DENSITY_INT         6U    /* r: плотность, 0.1 кг/м3, int16. */
#define MB_ADDR_VOLUME_MAIN_INT     7U    /* U1: объем основного продукта, 10 дм3, int16. */
#define MB_ADDR_LEVEL_INT_SEP       8U    /* h2: уровень раздела сред, мм, int16. */
#define MB_ADDR_TEMP_VAPOR_INT      10U   /* t-: температура паровой фазы, 0.01 °C, int16. */
#define MB_ADDR_MASS_VAPOR_INT      11U   /* G-: масса паровой фазы СУГ, 10 кг, int16. */
#define MB_ADDR_MASS_LIQ_INT        12U   /* G_: масса жидкой фазы СУГ, 10 кг, int16. */
#define MB_ADDR_VOLUME_STD_INT      17U   /* Ut: объем при стандартных условиях, 10 дм3. */
#define MB_ADDR_DENSITY_STD_INT     18U   /* rt: плотность при стандартных условиях, 0.1 кг/м3. */
#define MB_ADDR_DENSITY_MEAS_INT    19U   /* ri: измеренная плотность, 0.1 кг/м3. */
#define MB_ADDR_TEMP_DENS_INT       20U   /* tr: температура измерения плотности, 0.01 °C. */
#define MB_ADDR_VOLUME_SEP_INT      21U   /* U2: объем жидкости под разделом, 10 дм3. */
#define MB_ADDR_MASS_ERROR_INT      31U   /* G: относительная погрешность массы, 0.01 %. */

/* ========================================================================== */
/* Таблица Е.4: параметры связи int16                                         */
/* ========================================================================== */
#define MB_ADDR_MB_ADDR_SET         35U   /* AA: адрес устройства Modbus, 1..247. */
#define MB_ADDR_MB_BAUD_SET         36U   /* rS: код скорости 0..9, см. MB_BAUD_CODE_*. */
#define MB_ADDR_MB_PARITY_SET       37U   /* rP: код формата кадра 0=8N1,1=8N2,2=8O1,3=8E1. */

#define MB_BAUD_CODE_1200           0U
#define MB_BAUD_CODE_2400           1U
#define MB_BAUD_CODE_4800           2U
#define MB_BAUD_CODE_9600           3U
#define MB_BAUD_CODE_14400          4U
#define MB_BAUD_CODE_19200          5U
#define MB_BAUD_CODE_38400          6U
#define MB_BAUD_CODE_56000          7U
#define MB_BAUD_CODE_57600          8U
#define MB_BAUD_CODE_115200         9U

#define MB_PARITY_CODE_8N1          0U
#define MB_PARITY_CODE_8N2          1U
#define MB_PARITY_CODE_8O1          2U
#define MB_PARITY_CODE_8E1          3U

/* ========================================================================== */
/* Таблица Е.4: измеряемые параметры float32                                  */
/* Базовый адрес = младшее слово, следующий адрес = старшее слово.            */
/* ========================================================================== */
#define MB_ADDR_LEVEL               1000U /* h: уровень жидкости; единицы задает 2160. */
#define MB_ADDR_TEMP                1002U /* t°: температура жидкости; единицы задает 2162. */
#define MB_ADDR_PERCENT             1004U /* %: процентное заполнение резервуара. */
#define MB_ADDR_VOLUME              1006U /* U: объем жидкости; единицы задает 2164. */
#define MB_ADDR_MASS                1008U /* G: масса продукта; единицы задает 2166. */
#define MB_ADDR_DENSITY             1010U /* r: плотность; единицы задает 2168. */
#define MB_ADDR_VOLUME_MAIN         1012U /* U1: объем основного продукта. */
#define MB_ADDR_LEVEL_SEP           1014U /* h2: уровень раздела сред. */
#define MB_ADDR_TEMP_VAPOR          1018U /* t-: температура паровой фазы СУГ. */
#define MB_ADDR_MASS_VAPOR          1020U /* G-: масса паровой фазы СУГ. */
#define MB_ADDR_MASS_LIQ            1022U /* G_: масса жидкой фазы СУГ. */
#define MB_ADDR_VOLUME_STD          1032U /* Ut: объем при стандартных условиях. */
#define MB_ADDR_DENSITY_STD         1034U /* rt: плотность при стандартных условиях. */
#define MB_ADDR_DENSITY_MEAS        1036U /* ri: измеренная плотность. */
#define MB_ADDR_TEMP_DENS           1038U /* tr: температура при измерении плотности. */
#define MB_ADDR_VOLUME_SEP          1040U /* U2: объем жидкости под разделом сред. */

/* ========================================================================== */
/* Таблица Е.4: основные настроечные параметры float32                        */
/* ========================================================================== */
#define MB_ADDR_CAL_LOW_LVL         2000U /* h_: нижняя контрольная точка уровня, м. */
#define MB_ADDR_CAL_HIGH_LVL        2002U /* h-: верхняя контрольная точка уровня, м. */
#define MB_ADDR_PROBE_DEPTH         2004U /* d1: глубина погружения поплавка уровня, м. */
#define MB_ADDR_LEVEL_OFFSET        2006U /* d0: отступ от дна резервуара, м. */
#define MB_ADDR_TANK_GEOM           2008U /* Gr: способ расчета объема, код 0..3. */
#define MB_ADDR_TANK_HEIGHT         2010U /* H: высота/диаметр резервуара, м. */
#define MB_ADDR_TANK_VOLUME         2012U /* U: полный объем резервуара, м3. */
#define MB_ADDR_CALIB_POINTS        2014U /* Количество точек градуировочной таблицы. */
#define MB_ADDR_DENSITY_LOW         2018U /* r_: нижняя контрольная точка плотности, кг/м3. */
#define MB_ADDR_DENSITY_HIGH        2020U /* r-: верхняя контрольная точка плотности, кг/м3. */
#define MB_ADDR_EXPANSION_COEF      2022U /* Lo: коэффициент объемного расширения, 10^-3/°C. */
#define MB_ADDR_DENSITY_ORIG        2024U /* ro: исходная плотность, кг/м3. */
#define MB_ADDR_TEMP_ORIG           2026U /* to: температура исходной плотности, °C. */
#define MB_ADDR_SEP_DEPTH           2028U /* d2: погружение поплавка раздела сред, м. */
#define MB_ADDR_PROPANE_RATIO       2030U /* Pr: массовая доля пропана, %. */
#define MB_ADDR_DENSITY_MIN         2036U /* d3: нижний порог корректной плотности. */
#define MB_ADDR_LEVEL_ZERO_SEP      2038U /* d6: порог обнуления уровня раздела, м. */
#define MB_ADDR_THRESH_LVL          2040U /* d7: порог обнуления уровня, м. */
#define MB_ADDR_MAGNET_DIFF         2042U /* d8: разность высот магнитов поплавков, м. */
#define MB_ADDR_BUTANE_RATIO        2050U /* Pb: массовая доля бутана, %. */
#define MB_ADDR_DENSITY_MAX         2052U /* d9: верхний порог корректной плотности. */
#define MB_ADDR_ISOBUTANE_RATIO     2054U /* Pi: массовая доля изобутана, %. */
#define MB_ADDR_MASS_REL_ERROR      2056U /* G: относительная погрешность массы, %. */
#define MB_ADDR_TANK_ERROR          2060U /* delta-t: погрешность вместимости резервуара, %. */
#define MB_ADDR_TEMP_STD            2062U /* tS: температура стандартных условий, °C. */
#define MB_ADDR_BAUD_RATE           2064U /* rS: код скорости 0..9 в формате float32. */
#define MB_ADDR_PARITY              2066U /* rP: код формата кадра 0..3 в формате float32. */
#define MB_ADDR_DEVICE_ADDR         2068U /* AA: адрес Modbus 1..247 в формате float32. */
#define MB_ADDR_DAMPING_TIME        2086U /* dt: демпфирование уровня, 0 или 5..120 с. */

/* Расширение прошивки, отсутствует в таблице Е.4. */
#define MB_ADDR_POLL_PERIOD         2088U /* Период запуска измерения, мс; минимум 100 мс. */

#define MB_ADDR_VOLUME_15C          2090U /* UF: объем, приведенный к 15 °C. */
#define MB_ADDR_DENSITY_15C         2092U /* rF: плотность, приведенная к 15 °C. */

/* Расширение прошивки, отсутствует в таблице Е.4. */
#define MB_ADDR_MATERIAL_WAVE_SPEED 2094U /* Скорость волны в звукопроводе, м/с. */

#define MB_ADDR_WAVEGUIDE_LEN       2096U /* Lc: текущая длина звукопровода, м. */
#define MB_ADDR_WAVEGUIDE_DEV       2098U /* delta: отклонение длины звукопровода, %. */
#define MB_ADDR_LEVEL_CORR          2120U /* dh: поправка измерений уровня, м. */
#define MB_ADDR_DENSITY_CORR        2148U /* dr: поправка измерений плотности, кг/м3. */
#define MB_ADDR_MEDIUM_TYPE         2154U /* cE: тип среды: 0 произвольная, 1 нефтепродукт, 2 СУГ. */
#define MB_ADDR_TANK_EXPANSION      2156U /* ct: линейное расширение стенки, 10^-6/°C. */

/* ========================================================================== */
/* Таблица Е.4: единицы отображения int16                                     */
/* ========================================================================== */
#define MB_ADDR_UNIT_LEVEL          2160U /* Eh: 9=метры, 8=миллиметры. */
#define MB_ADDR_UNIT_TEMP           2162U /* Et: 24=градусы Цельсия. */
#define MB_ADDR_UNIT_VOLUME         2164U /* EU: 41=м3, 40=литры, 39=декалитры. */
#define MB_ADDR_UNIT_MASS           2166U /* EG: 55=тонны, 56=килограммы. */
#define MB_ADDR_UNIT_DENSITY        2168U /* Er: 72=г/см3, 73=кг/м3, 74=т/м3. */

/* ========================================================================== */
/* Таблица Е.4: информационные и калибровочные параметры                      */
/* ========================================================================== */
#define MB_ADDR_SERIAL_HI           2298U /* S1: старшие разряды заводского номера, int16. */
#define MB_ADDR_SERIAL_LO           2300U /* S2: младшие разряды заводского номера, int16. */
#define MB_ADDR_DAMPING_DENS        2312U /* dd: демпфирование плотности, 0 или 10..720 с. */
#define MB_ADDR_CAL_C1              2386U /* C1: калибровка уровня для h_, float32. */
#define MB_ADDR_CAL_C2              2388U /* C2: калибровка уровня для h-, float32. */
#define MB_ADDR_CAL_D4              2392U /* d4: расстояние для нижней точки плотности. */
#define MB_ADDR_CAL_D5              2394U /* d5: расстояние для верхней точки плотности. */
#define MB_ADDR_TEMP_SENS_COUNT     2414U /* Количество датчиков температуры, float32. */
#define MB_ADDR_ERROR_CODE          2416U /* Er: текущий код ошибки преобразователя, int16. */
#define MB_ADDR_SENS_ADDR           2418U /* Ad: адрес преобразователя в линии СЕНС, int16. */
#define MB_ADDR_FW_VERSION          2420U /* Pn: номер версии программы, int16. */
#define MB_ADDR_ADMIN_PASS          2426U /* P1: пароль администратора, float32. */
#define MB_ADDR_ERROR_DELAY         2438U /* F: задержка реакции на ошибку, с, float32. */

/* ========================================================================== */
/* Таблица Е.4: датчики температуры                                           */
/* ========================================================================== */
#define MB_ADDR_TEMP_SENS_1_H       2500U /* 1.ht: высота датчика температуры №1, м. */
#define MB_ADDR_TEMP_SENS_2_H       2502U /* 2.ht: высота датчика температуры №2, м. */
#define MB_ADDR_TEMP_SENS_3_H       2504U /* 3.ht: высота датчика температуры №3, м. */
#define MB_ADDR_TEMP_SENS_4_H       2506U /* 4.ht: высота датчика температуры №4, м. */
#define MB_ADDR_TEMP_SENS_5_H       2508U /* 5.ht: высота датчика температуры №5, м. */
#define MB_ADDR_TEMP_SENS_6_H       2510U /* 6.ht: высота датчика температуры №6, м. */
#define MB_ADDR_TEMP_SENS_7_H       2512U /* 7.ht: высота датчика температуры №7, м. */
#define MB_ADDR_TEMP_SENS_8_H       2514U /* 8.ht: высота датчика температуры №8, м. */
#define MB_ADDR_TEMP_SENS_1_V       2600U /* 1.°C: температура датчика №1, °C. */
#define MB_ADDR_TEMP_SENS_2_V       2602U /* 2.°C: температура датчика №2, °C. */
#define MB_ADDR_TEMP_SENS_3_V       2604U /* 3.°C: температура датчика №3, °C. */
#define MB_ADDR_TEMP_SENS_4_V       2606U /* 4.°C: температура датчика №4, °C. */
#define MB_ADDR_TEMP_SENS_5_V       2608U /* 5.°C: температура датчика №5, °C. */
#define MB_ADDR_TEMP_SENS_6_V       2610U /* 6.°C: температура датчика №6, °C. */
#define MB_ADDR_TEMP_SENS_7_V       2612U /* 7.°C: температура датчика №7, °C. */
#define MB_ADDR_TEMP_SENS_8_V       2614U /* 8.°C: температура датчика №8, °C. */
#define MB_ADDR_DENS_SENS_1         2700U /* 1.r: плотность, приведенная к датчику №1. */
#define MB_ADDR_DENS_SENS_2         2702U /* 2.r: плотность, приведенная к датчику №2. */
#define MB_ADDR_DENS_SENS_3         2704U /* 3.r: плотность, приведенная к датчику №3. */
#define MB_ADDR_DENS_SENS_4         2706U /* 4.r: плотность, приведенная к датчику №4. */
#define MB_ADDR_DENS_SENS_5         2708U /* 5.r: плотность, приведенная к датчику №5. */
#define MB_ADDR_DENS_SENS_6         2710U /* 6.r: плотность, приведенная к датчику №6. */
#define MB_ADDR_DENS_SENS_7         2712U /* 7.r: плотность, приведенная к датчику №7. */
#define MB_ADDR_DENS_SENS_8         2714U /* 8.r: плотность, приведенная к датчику №8. */

/* ========================================================================== */
/* Таблица Е.4: команды управления                                            */
/* ========================================================================== */
#define MB_ADDR_COMMAND             3000U /* Запись номера команды; чтение результата 0/85/90/99. */
#define MB_ADDR_COMMAND_PARAM       3002U /* Параметр команды управления, float32. */

/* ========================================================================== */
/* Таблица Е.4: регистры отображения                                          */
/* ========================================================================== */
#define MB_ADDR_DISPLAY_FIRST       4000U /* Регистр отображения №1. */
#define MB_ADDR_DISPLAY_LAST        4124U /* Регистр отображения №125. */
#define MB_ADDR_DISPLAY_MAP_FIRST   5000U /* Адрес источника для регистра отображения №1. */
#define MB_ADDR_DISPLAY_MAP_LAST    5124U /* Адрес источника для регистра отображения №125. */
#define MB_DISPLAY_REGISTER_COUNT   125U

/* ========================================================================== */
/* Таблица Е.4: градуировочная таблица                                        */
/* ========================================================================== */
#define MB_ADDR_GRAD_POINT_COUNT    32768U /* Количество точек, float32. */
#define MB_ADDR_GRAD_START_HEIGHT   32770U /* Начальная высота градуировки, м, float32. */
#define MB_ADDR_GRAD_LEVEL_STEP     32772U /* Шаг градуировки по уровню, м, float32. */
#define MB_ADDR_GRAD_TANK_HEIGHT    32774U /* Высота/диаметр резервуара, м, float32. */
#define MB_ADDR_GRAD_TANK_VOLUME    32776U /* Объем резервуара, м3, float32. */
#define MB_ADDR_GRAD_VOLUME_FIRST   32778U /* Объем в точке №1, м3, float32. */
#define MB_ADDR_GRAD_VOLUME_LAST    38778U /* Объем в точке №3001, м3, float32. */
#define MB_GRAD_DOCUMENT_POINT_COUNT 3001U

/* Базовый адрес объема точки с индексом 0..3000. */
#define MB_ADDR_GRAD_VOLUME(index)  ((uint16_t)(MB_ADDR_GRAD_VOLUME_FIRST + ((uint16_t)(index) * 2U)))

/* Полный документированный диапазон слов градуировки, включая старшее слово
 * последнего float32. */
#define MB_ADDR_GRAD_FIRST_WORD     MB_ADDR_GRAD_POINT_COUNT
#define MB_ADDR_GRAD_LAST_WORD      ((uint16_t)(MB_ADDR_GRAD_VOLUME_LAST + 1U))

/* Окна, в которых неопределенные промежуточные адреса читаются как 0. Это
 * позволяет фирменным утилитам читать карту крупными блоками без исключения
 * Illegal Data Address на зарезервированных местах. */
#define MB_DOC_WINDOW_INT_FIRST     1U
#define MB_DOC_WINDOW_INT_LAST      37U
#define MB_DOC_WINDOW_MEAS_FIRST    1000U
#define MB_DOC_WINDOW_MEAS_LAST     1041U
#define MB_DOC_WINDOW_CFG_FIRST     2000U
#define MB_DOC_WINDOW_CFG_LAST      2168U
#define MB_DOC_WINDOW_INFO_FIRST    2298U
#define MB_DOC_WINDOW_INFO_LAST     2439U
#define MB_DOC_WINDOW_TEMP_FIRST    2500U
#define MB_DOC_WINDOW_TEMP_LAST     2715U
#define MB_DOC_WINDOW_CMD_FIRST     3000U
#define MB_DOC_WINDOW_CMD_LAST      3003U

typedef enum {
    MODBUS_REGISTER_UNDEFINED = 0,
    MODBUS_REGISTER_UINT16,
    MODBUS_REGISTER_FLOAT32
} ModBus_RegisterType_t;

void ModBus_Init(void);
void ModBus_Process(void);
void ModBus_StorageProcess(bool allow_write);
bool ModBus_StorageIsBusy(void);
void ModBus_RxCallback(UART_HandleTypeDef *huart);
void ModBus_RestartRx(void);

uint16_t ModBus_CRC16(const uint8_t *data, uint16_t length);
uint8_t ModBus_GetDeviceAddress(void);
bool ModBus_CommunicationIsBusy(void);

float ModBus_GetParameter_Float(uint16_t address);
void ModBus_SetParameter_Float(uint16_t address, float value);
uint16_t ModBus_GetParameter_Int(uint16_t address);
void ModBus_SetParameter_Int(uint16_t address, uint16_t value);

bool ModBus_ReadRawRegister(uint16_t address, uint16_t *value);
ModBus_RegisterType_t ModBus_GetRegisterType(uint16_t address);
uint16_t ModBus_AddressToIndex_External(uint16_t address);

float ModBus_GetWaveguideLength(void);
void ModBus_SetWaveguideLength(float length_m);
float ModBus_GetMaterialWaveSpeed(void);
void ModBus_SetMaterialWaveSpeed(float speed_mps);
void ModBus_SetTemperature(float temperature);
float ModBus_GetTemperature(void);
uint32_t ModBus_GetPulseWidthIterations(void);

void ModBus_PublishLiveMeasurements(float level_mm,
                                    float temperature_c,
                                    float percent,
                                    float volume_m3,
                                    uint16_t status);

void ModBus_UpdateMeasurements(float level, float temperature, float waveguide);
void ModBus_UpdateVoltages(float vdda, float v24, float v12, float v5);
void ModBus_UpdateFirmwareVersion(uint16_t version);
void ModBus_ForceSaveToEEPROM(void);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_H */
