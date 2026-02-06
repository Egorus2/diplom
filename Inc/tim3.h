#ifndef TIM3_H
#define TIM3_H

#include <stdint.h>

extern volatile uint8_t sensor_ready;

void TIM3_Init_1kHz(void);

#endif