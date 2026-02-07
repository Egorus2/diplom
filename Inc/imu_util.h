#ifndef IMU_UTIL_
#define IMU_UTIL_
	
	//privat macros
	#define WRITE 0U
	#define READ 1U
	#define ADDR_WRITE(addres) (((uint32_t)addres << 1U) | 0U)
	#define ADDR_READ(addres) (((uint32_t)addres << 1U) | 1U)
	
	//privat defines
	#define GYRO_ADDRES 0x68
	#define ACCELER_ADDRES 0x18
	#define MAGNET_ADDRES 0x1C
	#define GYRO_250DPS_SCALE  0.00875f

	//privat includes
	#include <stdint.h>
	//ext variables 
	extern volatile uint8_t sensor_ready;

	//init func's
	void TIM3_Init_800Hz(void);
	void GPIO_I2C_Init(void);
	void I2C_init(void);

	uint8_t ReadWhoAmI(void);
	void I2C1_ctrl_reg_gyro(void);
	void I2C1_ReadXYZ_Raw(uint8_t Address, int16_t *gx, int16_t *gy, int16_t *gz);
	void calibration_gyro(int16_t *bias_x, int16_t *bias_y, int16_t *bias_z);

#endif

	
//	*gx *= GYRO_250DPS_SCALE;
//	*gy *= GYRO_250DPS_SCALE;
//	*gz *= GYRO_250DPS_SCALE;
