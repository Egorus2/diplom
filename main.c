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
	//usart+dma init func's
	GPIO_USART1_Init();
	USART1_Init();
	DMA2_USART1_RX_TX_Init();
	usart1_Transm_str("\x1B[2J\x1B[H", TIMEOUT_USART);    // clear the terminal
	
	//imu init func's
	GPIO_I2C_Init();
	I2C_init();
	I2C1_ctrl_reg_gyro();
	TIM3_Init_800Hz();
	gyro_struct_init(&gyro);
	
	//calibration
	calibration_gyro(&gyro.bias_x, &gyro.bias_y, &gyro.bias_z);
	
	//setup for i2c+dma
	I2C_DMA_init_forRead();

  while(1)
	{
		if(gyro_ready)
		{
			gyro_ready = 0;
			gyro_processed_values(&gyro);
			i++;
			if(i == 10)
			{
				i = 0;
				char buf[32];
				snprintf(buf, sizeof(buf), "%.3f %.3f %.3f\r\n", gyro.x_fil, gyro.y_fil, gyro.z_fil);
				usart1_Transm_str(buf, TIMEOUT_USART);
			}
		}
	}
}

