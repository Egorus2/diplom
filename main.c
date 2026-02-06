#include "stm32f4xx.h"                  // Device header
#include "system_stm32f4xx.h"

#include "system.h" 
#include "usart.h"
#include "imu_util.h"

#define TIMEOUT_USART 3UL
#define MAX_LEN_RX 32UL


int main(void)
{
	RCC_Init();
	sysTickInit();
	//usart+dma
	GPIO_USART1_Init();
	USART1_Init();
	DMA2_USART1_RX_TX_Init();
	usart1_Transm_str("\x1B[2J\x1B[H", TIMEOUT_USART);    // clear the terminal
	
	//imu
	TIM3_Init_1kHz();
	GPIO_I2C_Init();
	I2C_init();
	
	ReadWhoAmI();
	
  while(1)
	{
		
	}
}
