#ifndef IMU_UTIL_
#define IMU_UTIL_
	
	//includes
	#include <stdint.h>
	
	//macros
	#define WRITE 0U
	#define READ 1U
	#define ADDR_WRITE(addres) (((uint32_t)addres << 1U) | 0U)
	#define ADDR_READ(addres) (((uint32_t)addres << 1U) | 1U)
	
	//typedef
	typedef int32_t q31_t;
	
	//defines
	#define Q31_FROM_FLOAT(f) ((q31_t)(f * 2147483648.0f))
	#define FLOAT_FROM_Q31(q) ((float)q / 2147483648.0f)
	#define Q31_FROM_INT16(i) ((q31_t)(i << 16))
	#define GYRO_ADDRES 0x68
	#define ACCELER_ADDRES 0x18
	#define MAGNET_ADDRES 0x1C
	#define SENSOR_SUBADDR_REG 0x28
	#define GYRO_250DPS_SCALE 0.00875f
	#define GYRO_250DPS_SCALE_Q31 Q31_FROM_FLOAT(GYRO_250DPS_SCALE)
	#define ACCEL_2G_SCALE (1.0f / 16384.0f)
	#define MAX_LEN_I2C 6U
	#define SENSOR_COUNT 2U
    #define RAD_TO_DEG_CONST (180.0f/3.141592f)
    #define SAMPLES_PER_UPDATE 5U

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
		GYRO,
		ACCELEROM
	} sensor_t;
		
	//struct
	typedef struct{
		i2c_state_t state;
		sensor_t curr_sensor;
    } state_machine_t;
	
    
    typedef struct{
		float x_fil, y_fil, z_fil;
		int16_t bias_x, bias_y, bias_z;
        q31_t x_fil_q31, y_fil_q31, z_fil_q31;
        float alpha;
        q31_t alpha_q31, beta_q31;
    } Sensor_data_t;
    
    typedef struct{
        float roll, pitch;
        float alpha, beta;
        float dt;
    } compl_filter_t;
        

	//ext variables 
	extern volatile uint8_t gyro_ready;
	extern volatile uint8_t accel_ready;
	extern uint8_t gyro_buffer[MAX_LEN_I2C];
	extern uint8_t accel_buffer[MAX_LEN_I2C];
	
	//init func's
	void TIM3_Init_800Hz(void);
	void GPIO_I2C_Init(void);
	void I2C_init(void);
	void gyro_struct_init(Sensor_data_t *gyro);
    void accel_struct_init(Sensor_data_t *accel);
    void compl_filter_struct_init(compl_filter_t *C, uint8_t samples_per_update);
	void I2C_DMA_init_forRead(void);
	void I2C1_ctrl_reg_gyro(void);
	void I2C1_ctrl_reg_accel(void);
	void imu_util_init(Sensor_data_t *G, Sensor_data_t *A, compl_filter_t *C);
    void complementary_filter(Sensor_data_t *G, Sensor_data_t *A, compl_filter_t *Comp);
	
	//operational functions
	uint8_t ReadWhoAmI(void);
	void calibration_gyro(int16_t *bias_x, int16_t *bias_y, int16_t *bias_z);
	void sensor_processed_values(Sensor_data_t *st, uint8_t *buf, uint8_t curr_sens);

#endif


