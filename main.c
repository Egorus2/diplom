#include "stm32f4xx.h"                  // Device header
#include "system_stm32f4xx.h"
#include <stdio.h>
#include <math.h>

#include "system.h" 
#include "usart.h"
#include "imu_util.h"
#include "MahonyAHRS.h"


int main(void)
{
	//local variables 
    Sensor_data_t gyro;
	Sensor_data_t accel;
    Sensor_data_t magnet;
    compl_filter_t compl_filter;
	uint8_t u = 0;
    
	//general init func's
	RCC_Init();
	sysTickInit();
	
	usart1_init();
	
	imu_util_init(&gyro, &accel, &magnet, &compl_filter);
    
    float half_yaw = (compl_filter.yaw_bias * DEG_TO_RAD_CONST) / 2.0f;
    q0 = cosf(half_yaw);
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = sinf(half_yaw);

  while(1)
	{
		if(gyro_ready)
		{
			gyro_ready = 0;
			sensor_processed_values(&gyro, gyro_buffer, GYRO);
		}
		
		else if(accel_ready)
		{
			accel_ready = 0;
			sensor_processed_values(&accel, accel_buffer, ACCELEROM);
//            MahonyAHRSupdate(FLOAT_FROM_Q31(gyro.x_fil_q31) * 250.0f * 0.0174532925f,
//                               FLOAT_FROM_Q31(gyro.y_fil_q31) * 250.0f * 0.0174532925f,
//                               FLOAT_FROM_Q31(gyro.z_fil_q31) * 250.0f * 0.0174532925f,
//                               FLOAT_FROM_Q31(accel.x_fil_q31), 
//                               FLOAT_FROM_Q31(accel.y_fil_q31), 
//                               FLOAT_FROM_Q31(accel.z_fil_q31), 
//                               FLOAT_FROM_Q31(magnet.x_fil_q31), 
//                               FLOAT_FROM_Q31(magnet.y_fil_q31), 
//                               FLOAT_FROM_Q31(magnet.z_fil_q31)
//                               );
            MahonyAHRSupdateIMU(FLOAT_FROM_Q31(gyro.x_fil_q31) * 250.0f * 0.0174532925f,
                               FLOAT_FROM_Q31(gyro.y_fil_q31) * 250.0f * 0.0174532925f,
                               FLOAT_FROM_Q31(gyro.z_fil_q31) * 250.0f * 0.0174532925f,
                               FLOAT_FROM_Q31(accel.x_fil_q31), 
                               FLOAT_FROM_Q31(accel.y_fil_q31), 
                               FLOAT_FROM_Q31(accel.z_fil_q31));                
		}
        else if(magnet_ready)
        {
            magnet_ready = 0;
            sensor_processed_values(&magnet, magnet_buffer, MAGNET);
			u++;
            

            
			if(u == SAMPLES_PER_UPDATE)
			{
				u = 0;
                float roll  = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2) * 57.2957795f;
                float pitch = asinf(-2.0f * (q1*q3 - q0*q2)) * 57.2957795f;
                float yaw   = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3) * 57.2957795f;
                if (yaw < 0) yaw += 360.0f;
                //usart1_Transm_str("\x1B[2J\x1B[H", TIMEOUT_USART);    // clear the terminal
				char buf1[32];
                snprintf(buf1, sizeof(buf1), "%.4f, %.4f, %.4f\r\n", roll, pitch, yaw);
				usart1_Transm_str(buf1, TIMEOUT_USART);
                
			}            
        }
	}
	
}


