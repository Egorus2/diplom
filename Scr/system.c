#include "stm32f4xx.h"                  // Device header
#include "system_stm32f4xx.h"

#include "system.h"

volatile uint64_t msCounter;

void RCC_Init(void)
/*
 * @brief  RCC initialization 
 * @param  None
 * @retval None
 */
{
	RCC->CR |= RCC_CR_HSEON; //external oscillator
	while(!(RCC->CR & RCC_CR_HSERDY));
	
	//disable PLL for setup
	RCC->CR &= ~RCC_CR_PLLON;
	
	//PLL config
	RCC->PLLCFGR = (25UL << RCC_PLLCFGR_PLLM_Pos)|(336UL << RCC_PLLCFGR_PLLN_Pos)|
									RCC_PLLCFGR_PLLP_0 | RCC_PLLCFGR_PLLSRC_HSE;
	
	//PLL ON
	RCC->CR |= RCC_CR_PLLON;
	while(!(RCC->CR & RCC_CR_PLLRDY));
	
	//wait state for flash memory
	do{
		FLASH->ACR |= FLASH_ACR_LATENCY_2WS;
	}while(!(FLASH->ACR & FLASH_ACR_LATENCY_2WS));
	
	//APB1 prescaler
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV1 | RCC_CFGR_SW_PLL;
	//update SystemCoreClock value
	SystemCoreClockUpdate();        
	
}

void sysTickInit(void)
/*
 * @brief  Initializing the system clock to operate at a frequency of 1kHz(T = 1 ms)
 * @param  None
 * @retval None
 */
{
	msCounter = 0;
	SysTick->LOAD = (SystemCoreClock/1000) - 1;
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}


void SysTick_Handler(void)
/*
 * @brief  System timer interrupt handler.
 * @param  None
 * @retval None
 */
{

	msCounter++;
	
}

void Delay_ms(uint16_t delay)
/*
 * @brief  standart delay(blocking)
 * @param  dalay value
 * @retval None
 */
{
	uint64_t start = msCounter;
	while((msCounter - start) <= delay);
}

