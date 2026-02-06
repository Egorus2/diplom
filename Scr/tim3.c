#include "stm32f4xx.h"                  // Device header
#include "system_stm32f4xx.h"

#include "tim3.h"

void TIM3_IRQHandler(void);

volatile uint8_t sensor_ready = 0;

void TIM3_Init_1kHz(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    TIM3->PSC = 83;
    TIM3->ARR = 999;
    
    TIM3->DIER |= TIM_DIER_UIE;  
    TIM3->CR1 |= TIM_CR1_CEN;   
    
    NVIC_EnableIRQ(TIM3_IRQn);  
}


void TIM3_IRQHandler(void)
{
    if (TIM3->SR & TIM_SR_UIF) {
        TIM3->SR &= ~TIM_SR_UIF; 
        sensor_ready = 1;         
    }
}