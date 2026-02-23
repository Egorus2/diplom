#include "stm32f4xx.h"                  // Device header
#include "system_stm32f4xx.h"
#include <stdio.h>

#include "system.h" 
#include "usart.h"
#include "imu_util.h"




int main(void)
{
	//local variables 
    Sensor_data_t gyro;
	Sensor_data_t accel;
    compl_filter_t compl_filter;
	uint8_t u = 0;
    
	//general init func's
	RCC_Init();
	sysTickInit();
	
	usart1_init();
	
	imu_util_init(&gyro, &accel, &compl_filter);

  while(1)
	{
		if(gyro_ready)
		{
			gyro_ready = 0;
			sensor_processed_values(&gyro, gyro_buffer, GYRO);
		}
		
		if(accel_ready)
		{
			accel_ready = 0;
			sensor_processed_values(&accel, accel_buffer, ACCELEROM);
			u++;
			if(u == SAMPLES_PER_UPDATE)
			{
				u = 0;
                complementary_filter(&gyro, &accel, &compl_filter);
                usart1_Transm_str("\x1B[2J\x1B[H", TIMEOUT_USART);    // clear the terminal
				char buf1[32];
				snprintf(buf1, sizeof(buf1), "roll %.3f pitch %.3f\r\n", compl_filter.roll, compl_filter.pitch);
				usart1_Transm_str(buf1, TIMEOUT_USART);
			}	
		}
	}
	
}


