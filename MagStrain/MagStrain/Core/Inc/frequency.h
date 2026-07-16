#ifndef FREQUENCY_H
#define FREQUENCY_H

#include <stdint.h>

// Глобальные переменные для измерений
extern volatile uint32_t first_capture;
extern volatile uint32_t last_capture;
extern volatile uint32_t period_ticks;
extern volatile uint32_t overflow_count;
extern volatile uint8_t capture_count;
extern volatile uint8_t measurement_ready;
extern volatile uint32_t errors;

#endif /* FREQUENCY_H */
