#include "stm32f4xx.h"                  // Device header
#include "system_stm32f4xx.h"

#include "system.h" 


int main(void)
{
	RCC_Init();
	sysTickInit();
	
  while(1)
	{

	}
}
