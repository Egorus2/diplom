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
	uint8_t i = 0;
	uint8_t u = 0;
	//general init func's
	RCC_Init();
	sysTickInit();
	
	usart1_init();
	
	imu_util_init(&gyro, &accel);

  while(1)
	{
		if(gyro_ready)
		{
			gyro_ready = 0;
			sensor_processed_values(&gyro, gyro_buffer, GYRO);

			i++;
			if(i == 5)
			{
				i = 0;
				char buf[32];
				snprintf(buf, sizeof(buf), "%.3f %.3f %.5f\r\n", gyro.x_fil, gyro.y_fil, gyro.z_fil);
				usart1_Transm_str(buf, TIMEOUT_USART);
			}
		}
		
		if(accel_ready)
		{
			accel_ready = 0;
			sensor_processed_values(&accel, accel_buffer, ACCELEROM);
//			u++;
//			if(u == 5)
//			{
//				u = 0;
//				char buf1[32];
//				snprintf(buf1, sizeof(buf1), "%.3f %.3f %.3f\r\n", accel.ax, accel.ay, accel.az);
//				usart1_Transm_str(buf1, TIMEOUT_USART);
//			}	
		}
	}
	
}


