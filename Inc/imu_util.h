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
	
	//struct
	typedef struct{
		float x_fil, y_fil, z_fil;
		float alpha;
		int16_t bias_x, bias_y, bias_z;
		uint16_t plug;
  }Gyro_t;
  
	
	//ext variables 
	extern volatile uint8_t sensor_ready;

	//init func's
	void TIM3_Init_800Hz(void);
	void GPIO_I2C_Init(void);
	void I2C_init(void);
	void gyro_struct_init(Gyro_t *gyro);
	//operational functions
	uint8_t ReadWhoAmI(void);
	void I2C1_ctrl_reg_gyro(void);
	void calibration_gyro(int16_t *bias_x, int16_t *bias_y, int16_t *bias_z);
	void gyro_processed_values(Gyro_t* g);

#endif

	
//	*gx *= GYRO_250DPS_SCALE;
//	*gy *= GYRO_250DPS_SCALE;
//	*gz *= GYRO_250DPS_SCALE;
