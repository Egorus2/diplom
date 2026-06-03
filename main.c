#include "stm32f4xx.h"                  // Device header
#include "system_stm32f4xx.h"
#include <stdio.h>
#include <math.h>

#include "system.h" 
#include "usart.h"
#include "imu_util.h"
#include "kalman.h"


int main(void)
{
	//local variables 
    Kalman_t KalmanRoll;
    Kalman_t KalmanPitch;
    
    Sensor_data_t gyro;
	Sensor_data_t accel;
    Sensor_data_t magnet;
    compl_filter_t compl_filter;
	uint8_t u = 0;
    float gyro_std_rad[3] = {3.3e-5f, 3.7e-5f, 3.4e-5f};  // 0.002 °/s ? ???/?
    float acc_std[3] = {0.00045f, 0.00031f, 0.00055f};
    float mag_std[3] = {0.001f, 0.001f, 0.001f};  
    
	//general init func's
	RCC_Init();
	sysTickInit();
    
    Kalman_Init(&KalmanRoll);
    Kalman_Init(&KalmanPitch);
	
	usart1_init();
	
	imu_util_init(&gyro, &accel, &magnet, &compl_filter);
    

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
            
            float roll_acc = atan2f(FLOAT_FROM_Q31(accel.y_fil_q31), FLOAT_FROM_Q31(accel.z_fil_q31)) * 57.2958f;

            float pitch_acc = atan2f(-FLOAT_FROM_Q31(accel.x_fil_q31),sqrtf(FLOAT_FROM_Q31(accel.y_fil_q31)*FLOAT_FROM_Q31(accel.y_fil_q31) + FLOAT_FROM_Q31(accel.z_fil_q31)*FLOAT_FROM_Q31(accel.z_fil_q31)))* 57.2958f;
            
            float roll = Kalman_GetAngle(&KalmanRoll,
                    roll_acc,
                    FLOAT_FROM_Q31(gyro.x_fil_q31) * 250.0f,
                    0.00125f);

            float pitch = Kalman_GetAngle(&KalmanPitch,
                    pitch_acc,
                    FLOAT_FROM_Q31(gyro.y_fil_q31) * 250.0f,
                    0.00125f);
            
                char buf1[32];
                snprintf(buf1, sizeof(buf1), "%.4f, %.4f\r\n", roll, pitch);
				usart1_Transm_str(buf1, TIMEOUT_USART);
            
//            MadgwickAHRSupdate(FLOAT_FROM_Q31(gyro.x_fil_q31) * 250.0f * 0.0174532925f,
//                               FLOAT_FROM_Q31(gyro.y_fil_q31) * 250.0f * 0.0174532925f,
//                               FLOAT_FROM_Q31(gyro.z_fil_q31) * 250.0f * 0.0174532925f,
//                               FLOAT_FROM_Q31(accel.x_fil_q31), 
//                               FLOAT_FROM_Q31(accel.y_fil_q31), 
//                               FLOAT_FROM_Q31(accel.z_fil_q31), 
//                               FLOAT_FROM_Q31(magnet.x_fil_q31), 
//                               FLOAT_FROM_Q31(magnet.y_fil_q31), 
//                               FLOAT_FROM_Q31(magnet.z_fil_q31)
//                               );
		}
        else if(magnet_ready)
        {
            magnet_ready = 0;
            sensor_processed_values(&magnet, magnet_buffer, MAGNET);
			u++;
            

            
			if(u == SAMPLES_PER_UPDATE)
			{
				u = 0;
                float roll;
                float pitch;
                float yaw;
                //usart1_Transm_str("\x1B[2J\x1B[H", TIMEOUT_USART);    // clear the terminal
//				char buf1[32];
//                snprintf(buf1, sizeof(buf1), "%.4f, %.4f, %.4f\r\n", roll, pitch, yaw);
//				usart1_Transm_str(buf1, TIMEOUT_USART);
                
			}            
        }
	}
	
}


