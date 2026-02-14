#include "stm32f4xx.h"                  // Device header
#include "system_stm32f4xx.h"
#include <stdio.h>

#include "system.h" 
#include "usart.h"
#include "imu_util.h"




int main(void)
{
	//local variables 
	Gyro_t gyro;
	uint8_t i = 0;
	
	//general init func's
	RCC_Init();
	sysTickInit();
	
	usart1_init();
	
	imu_util_init(&gyro);

  while(1)
	{
		if(gyro_ready)
		{
			gyro_ready = 0;
			gyro_processed_values(&gyro, gyro_buffer);

			i++;
			if(i == 5)
			{
				i = 0;
				char buf[32];
				snprintf(buf, sizeof(buf), "%.3f %.3f %.3f\r\n", gyro.x_fil, gyro.y_fil, gyro.z_fil);
				usart1_Transm_str(buf, TIMEOUT_USART);
			}
		}
	}
}

