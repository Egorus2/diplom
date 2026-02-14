#ifndef IMU_UTIL_
#define IMU_UTIL_
	
	//privat includes
	#include <stdint.h>
	
	//privat macros
	#define WRITE 0U
	#define READ 1U
	#define ADDR_WRITE(addres) (((uint32_t)addres << 1U) | 0U)
	#define ADDR_READ(addres) (((uint32_t)addres << 1U) | 1U)
	
	//privat defines
	#define GYRO_ADDRES 0x68
	#define ACCELER_ADDRES 0x18
	#define MAGNET_ADDRES 0x1C
	#define SENSOR_SUBADDR_REG 0x28
	#define GYRO_250DPS_SCALE  0.00875f
	#define MAX_LEN_I2C 6U
	#define SENSOR_COUNT 2U

	//enum
	typedef enum{
		I2C_STATE_FREE,
		I2C_STATE_START,
		I2C_STATE_ADDR_W,
		I2C_STATE_RESTART,
		I2C_STATE_ADDR_R,
		I2C_STATE_ADDR_CLEAR,
		I2C_STATE_DMA_RUN
	} i2c_state_t;
	
	typedef enum{
		Gyro,
		Accelerometer
	} sensor_t;
		
	
	//struct
	typedef struct{
		i2c_state_t state;
		sensor_t curr_sensor;
  }state_machine_t;
	
	typedef struct{
		float x_fil, y_fil, z_fil;
		float alpha;
		int16_t bias_x, bias_y, bias_z;
		uint16_t plug;
  }Gyro_t;
  
	
	//ext variables 
	extern volatile uint8_t gyro_ready;
	extern volatile uint8_t accel_ready;
	extern uint8_t gyro_buffer[MAX_LEN_I2C];
	extern uint8_t accel_buffer[MAX_LEN_I2C];
	
	//init func's
	void TIM3_Init_800Hz(void);
	void GPIO_I2C_Init(void);
	void I2C_init(void);
	void gyro_struct_init(Gyro_t *gyro);
	void I2C_DMA_init_forRead(void);
	
	//operational functions
	uint8_t ReadWhoAmI(void);
	void I2C1_ctrl_reg_gyro(void);
	void calibration_gyro(int16_t *bias_x, int16_t *bias_y, int16_t *bias_z);
	void gyro_processed_values(Gyro_t* g, uint8_t* gyro_buf);

#endif


