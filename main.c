#include "stm32f4xx.h"                  // Device header
#include "system_stm32f4xx.h"
#include <stdio.h>
#include <math.h>

#include "system.h" 
#include "usart.h"
#include "imu_util.h"
#include "MadgwickAHRS.h"
#include "MahonyAHRS.h"
#include "kal.h"


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
    
//    float half_yaw = (compl_filter.yaw_bias * DEG_TO_RAD_CONST) / 2.0f;
//    pq0 = cosf(half_yaw);
//    pq1 = 0.0f;
//    pq2 = 0.0f;
//    pq3 = sinf(half_yaw);

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
            float gx = FLOAT_FROM_Q31(gyro.x_fil_q31) * 250.0f * 0.0174532925f;
            float gy = FLOAT_FROM_Q31(gyro.y_fil_q31) * 250.0f * 0.0174532925f;
            float gz = FLOAT_FROM_Q31(gyro.z_fil_q31) * 250.0f * 0.0174532925f;           
            float ax = FLOAT_FROM_Q31(accel.x_fil_q31);
            float ay = FLOAT_FROM_Q31(accel.y_fil_q31);
            float az = FLOAT_FROM_Q31(accel.z_fil_q31);
            float mx = FLOAT_FROM_Q31(magnet.x_fil_q31);
            float my = FLOAT_FROM_Q31(magnet.y_fil_q31);
            float mz = FLOAT_FROM_Q31(magnet.z_fil_q31);
            MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
            MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);
            AHRSupdateIMU(gx, gy, gz, ax, ay, az);
		}
        else if(magnet_ready)
        {
            magnet_ready = 0;
            sensor_processed_values(&magnet, magnet_buffer, MAGNET);
			u++;
            

            
			if(u == SAMPLES_PER_UPDATE)
			{
				u = 0;
                float proll  = atan2f(pq0*pq1 + pq2*pq3, 0.5f - pq1*pq1 - pq2*pq2) * 57.2957795f;
                float ppitch = asinf(-2.0f * (pq1*pq3 - pq0*pq2)) * 57.2957795f;
                float pyaw   = atan2f(pq1*pq2 + pq0*pq3, 0.5f - pq2*pq2 - pq3*pq3) * 57.2957795f;
                if (pyaw < 0) pyaw += 360.0f;
                
                float rroll  = atan2f(rq0*rq1 + rq2*rq3, 0.5f - rq1*rq1 - rq2*rq2) * 57.2957795f;
                float rpitch = asinf(-2.0f * (rq1*rq3 - rq0*rq2)) * 57.2957795f;
                float ryaw   = atan2f(rq1*rq2 + rq0*rq3, 0.5f - rq2*rq2 - rq3*rq3) * 57.2957795f;
                if (ryaw < 0) ryaw += 360.0f;
                
                float kroll  = atan2f(kq0*kq1 + kq2*kq3, 0.5f - kq1*kq1 - kq2*kq2) * 57.2957795f;
                float kpitch = asinf(-2.0f * (kq1*kq3 - kq0*kq2)) * 57.2957795f;
                float kyaw   = atan2f(kq1*kq2 + kq0*kq3, 0.5f - kq2*kq2 - kq3*kq3) * 57.2957795f;
                if (kyaw < 0) kyaw += 360.0f;
                //usart1_Transm_str("\x1B[2J\x1B[H", TIMEOUT_USART);    // clear the terminal
				char buf1[80];
                snprintf(buf1, sizeof(buf1), "%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\r\n", proll, ppitch, pyaw, rroll, rpitch, ryaw, kroll, kpitch, kyaw);
				usart1_Transm_str(buf1, TIMEOUT_USART);
                
			}            
        }
	}
	
}


