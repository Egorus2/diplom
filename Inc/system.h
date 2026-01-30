#ifndef SYSTEM_H
#define SYSTEM_H

#include <stdint.h>

void RCC_Init(void);

void sysTickInit(void);

void SysTick_Handler(void);

void Delay_ms(uint16_t delay);


extern volatile uint64_t msCounter;

#endif
