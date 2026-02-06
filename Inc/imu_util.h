#ifndef IMU_UTIL_
#define IMU_UTIL_

#include <stdint.h>

extern volatile uint8_t sensor_ready;

void TIM3_Init_1kHz(void);


void GPIO_I2C_Init(void);
void I2C_init(void);
void I2C1_Start(void);
void I2C1_Stop(void);
uint8_t I2C1_WriteByte(uint8_t data);
uint8_t I2C1_ReadByte(uint8_t nack);
uint8_t ReadWhoAmI(void);
void I2C1_ctrl_reg_gyro(void);
void I3G4250D_ReadGyro(int16_t *gx, int16_t *gy, int16_t *gz);

#endif
