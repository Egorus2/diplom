#include "stm32f4xx.h"                  // Device header
#include "system_stm32f4xx.h"
#include <stdio.h>

#include "system.h" 
#include "usart.h"
#include "imu_util.h"




int main(void)
{
	RCC_Init();
	sysTickInit();
	//usart+dma
	GPIO_USART1_Init();
	USART1_Init();
	DMA2_USART1_RX_TX_Init();
	usart1_Transm_str("\x1B[2J\x1B[H", TIMEOUT_USART);    // clear the terminal
	
//	float gx, gy, gz;
//	uint8_t i = 0;
	int16_t bias_x, bias_y, bias_z;
	char buf[32];
	
	
	//imu
	GPIO_I2C_Init();
	I2C_init();
	I2C1_ctrl_reg_gyro();
	TIM3_Init_800Hz();
	
	calibration_gyro(&bias_x, &bias_y, &bias_z);
	Delay_ms(2);
	usart1_Transm_str("\x1B[2J\x1B[H", TIMEOUT_USART);
	snprintf(buf, sizeof(buf), "%.3d %.3d %.3d\r\n", bias_x, bias_y, bias_z);
	usart1_Transm_str(buf, TIMEOUT_USART);
  while(1)
	{
//		if(sensor_ready)
//		{
//			sensor_ready = 0;
//			I3G4250D_ReadGyro(&gx, &gy, &gz);
//			i++;
//			if(i == 10)
//			{
//				i = 0;
//				char buf[32];
////				usart1_Transm_str("\x1B[2J\x1B[H", TIMEOUT_USART);    // clear the terminal
////				snprintf(buf, sizeof(buf), "Gyro: X=%.3f Y=%.3f Z=%.3f\r\n", gx, gy, gz);
//				snprintf(buf, sizeof(buf), "%.3f %.3f %.3f\r\n", gx, gy, gz);
//				usart1_Transm_str(buf, TIMEOUT_USART);
//			}

	}
}

