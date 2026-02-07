#ifndef IMU_UTIL_
#define IMU_UTIL_
	
	//privat macros
	#define ADDR_WRITE(addres) ((addres << 1) | 0)
	#define ADDR_READ(addres) ((addres << 1) | 1)
	
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
	void TIM3_Init_1kHz(void);
	void GPIO_I2C_Init(void);
	void I2C_init(void);

	uint8_t ReadWhoAmI(void);
	void I2C1_ctrl_reg_gyro(void);
	void I3G4250D_ReadGyro(float *gx, float *gy, float *gz);

#endif
