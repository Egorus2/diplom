#include "stm32f4xx.h"                  // Device header
#include "system_stm32f4xx.h"

#include "imu_util.h"
#include "system.h"
//local func's
void TIM3_IRQHandler(void);
void I2C1_Start(void);
void I2C1_Stop(void);
uint8_t I2C1_WriteByte(uint8_t data);
uint8_t I2C1_ReadByte(uint8_t nack);
uint8_t I2C_ADDR(uint8_t Address, uint8_t write_read);
void I2C1_ReadXYZ_Raw(uint8_t Address, int16_t *gx, int16_t *gy, int16_t *gz);

//variables
volatile uint8_t sensor_ready = 0;



void GPIO_I2C_Init(void)
/*
 * @brief  GPIOB I2C Initialization (PB6 - clk, PB7 - data)
 * @param  None
 * @retval None
 */
{
	//RCC port B
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	while(!(RCC->AHB1ENR & RCC_AHB1ENR_GPIOBEN));
	
	//PB6 - clk//
	//MODER - AF
	GPIOB->MODER &= ~(3UL << GPIO_MODER_MODE6_Pos);
	GPIOB->MODER |= (2UL << GPIO_MODER_MODE6_Pos);
	//open drain
	GPIOB->OTYPER |= GPIO_OTYPER_OT6;
	//speed  - High speed
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED6_Msk);
	GPIOB->OSPEEDR |= (2UL << GPIO_OSPEEDR_OSPEED6_Pos);
	//pull up - no need, there are external pull up resistor
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD6_Msk);
	//AF - AF04
	GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL6_Msk);
	GPIOB->AFR[0] |= (4UL << GPIO_AFRL_AFSEL6_Pos);
	
	//PB7 - data//
	//MODER - AF
	GPIOB->MODER &= ~(3UL << GPIO_MODER_MODE7_Pos);
	GPIOB->MODER |= (2UL << GPIO_MODER_MODE7_Pos);
	//open drain
	GPIOB->OTYPER |= GPIO_OTYPER_OT7;
	//speed  - High speed
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED7_Msk);
	GPIOB->OSPEEDR |= (2UL << GPIO_OSPEEDR_OSPEED7_Pos);
	//pull up - no need, there are external pull up resistor
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD7_Msk);
	//AF - AF04
	GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL7_Msk);
	GPIOB->AFR[0] |= (4UL << GPIO_AFRL_AFSEL7_Pos);
}
	
void I2C_init(void)
{
	//RCC
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	//stop i2c to srt up
	I2C1->CR1 &= ~I2C_CR1_PE;
	//freq of APB1
	I2C1->CR2 &= ~(I2C_CR2_FREQ_Msk);
	I2C1->CR2 |= 42UL;
	//trise
	I2C1->TRISE = 14UL;
	//fast mode, duty = 0
	I2C1->CCR |= I2C_CCR_FS;
	I2C1->CCR &= ~I2C_CCR_DUTY;
	//ccr
	I2C1->CCR &= ~I2C_CCR_CCR_Msk;
	I2C1->CCR |= (35UL & 0xFFF);
	//en i2c
	I2C1->CR1 |= I2C_CR1_PE;
	
}

void I2C1_Start(void)
/*
 * @brief  generate start condition
 * @param  None
 * @retval None
 */
{
	I2C1->CR1 |= I2C_CR1_START;
	while(!(I2C1->SR1 & I2C_SR1_SB));
}

void I2C1_Stop(void)
/*
 * @brief  generate stop condition
 * @param  None
 * @retval None
 */
{
	I2C1->CR1 |= I2C_CR1_STOP;
	while(I2C1->CR1 & I2C_CR1_STOP);
}
	
uint8_t I2C1_WriteByte(uint8_t data)
/*
 * @brief  i2c1 func to write a byte
 * @param  result data
 * @retval 1 - error, 0 - success
 */
{
	I2C1->DR = data;         // data transmit
	//wait until Byte transfer finished or Acknowledge failure
	while(!(I2C1->SR1 & (I2C_SR1_BTF | I2C_SR1_AF)));  
	//check if error
	if(I2C1->SR1 & I2C_SR1_AF)
	{
		I2C1->SR1 &= ~I2C_SR1_AF;
		I2C1_Stop();
		return 1;      //error
	}
	return 0;  //success
}
	
uint8_t I2C1_ReadByte(uint8_t nack)
/*
 * @brief  I2C Read Byte func
 * @param  this is the last pack? 1 - yes, 0 - no
 * @retval data
 */
{
    if (nack) {
			I2C1->CR1 |= I2C_CR1_STOP;
      I2C1->CR1 &= ~I2C_CR1_ACK;
    } else {
        I2C1->CR1 |= I2C_CR1_ACK;
    }
    
    while (!(I2C1->SR1 & I2C_SR1_RXNE));
    uint8_t data = (uint8_t)(I2C1->DR);
    
    if (nack) {
			I2C1->CR1 |= I2C_CR1_STOP;  
       while (I2C1->CR1 & I2C_CR1_STOP);
    }
    
    return data;
}

uint8_t I2C_ADDR(uint8_t Address, uint8_t write_read)
/*
 * @brief  i2c send device address
 * @param  address of device, read - 1 write - 0
 * @retval status 0 - ok, 1 - error
 */
{
	if(write_read)
	{
		I2C1->DR = ADDR_READ(Address); 
	}
	else
	{
		I2C1->DR = ADDR_WRITE(Address); 
	}

	while (!(I2C1->SR1 & I2C_SR1_ADDR)) {
			if (I2C1->SR1 & I2C_SR1_AF) {
					I2C1->SR1 &= ~I2C_SR1_AF;
					I2C1_Stop();
					return 1;
			}
	}
	
	(void)I2C1->SR1; 
	(void)I2C1->SR2;
	return 0;
}

uint8_t ReadWhoAmI(void)
/*
 * @brief  a function for reading the device address
 * @param  None
 * @retval device address
 */
{
	uint8_t data = 0;
	//start
	I2C1_Start();
	//addres + W
	if(I2C_ADDR(GYRO_ADDRES, WRITE)) 
		return 0xFF;
	//subaddres
	I2C1->DR = 0x0F;
	while (!(I2C1->SR1 & I2C_SR1_BTF));  
	
	//SR
	I2C1_Start();
	//addres + R
	if(I2C_ADDR(GYRO_ADDRES, READ)) 
		return 0xFF;

	data = I2C1_ReadByte(1);
	
	return data; 
}

void I2C1_ctrl_reg_gyro(void)
/*
 * @brief  setting the gyroscope parameters
 * @param  None
 * @retval None
 */
{
	//start
	I2C1_Start();
	//addres + W
	I2C_ADDR(GYRO_ADDRES, WRITE);
	//subaddres 
	I2C1_WriteByte(0x20); 
	//settings 
	I2C1_WriteByte(0xCF);   // DR - 11(800Hz) , BW - 00, PD - 1(ON), ZEN - 1, YEN - 1, XEN - 1
	//stop
	I2C1_Stop();
    
}

void I2C1_ReadXYZ_Raw(uint8_t Address, int16_t *gx, int16_t *gy, int16_t *gz)
/*
 * @brief  i2c1 read raw axis values
 * @param  address of each axis variable
 * @retval None
 */
{
	//temp buff
	uint8_t data[6];
	
	//start
	I2C1_Start();
	//addres + W
	I2C_ADDR(Address, WRITE);
	//subaddres + autoincrement
	I2C1_WriteByte(0x28 | 0x80);
	//SR
	I2C1_Start();
	//addres + R
	I2C_ADDR(Address, READ);
	//read x, y, z (high and low reg)
	for (int i = 0; i < 5; i++) {
			data[i] = I2C1_ReadByte(0); 
	}
	data[5] = I2C1_ReadByte(1);     
	
	//generating values
	*gx = (int16_t)(data[1] << 8 | data[0]);  // X: OUT_X_H << 8 | OUT_X_L
	*gy = (int16_t)(data[3] << 8 | data[2]);  // Y: OUT_Y_H << 8 | OUT_Y_L
	*gz = (int16_t)(data[5] << 8 | data[4]);  // Z: OUT_Z_H << 8 | OUT_Z_L

}

void gyro_struct_init(Gyro_t* gyro)
/*
 * @brief  initialization for struct Gyro_t
 * @param  address to struct Gyro_t value
 * @retval None
 */
{
	gyro->bias_x = 0; gyro->bias_y = 0; gyro->bias_z = 0;
	gyro->x_fil = 0.0f; gyro->y_fil = 0.0f; gyro->z_fil = 0.0f;
	gyro->alpha = 0.8f;   // The alpha coefficient
}

void calibration_gyro(int16_t *bias_x, int16_t *bias_y, int16_t *bias_z)
/*
 * @brief  calibration to calculate bias value for each axis
 * @param  bias
 * @retval None
 */
{
	int16_t gx, gy, gz;
	int64_t sum_x, sum_y, sum_z;
	sum_x = 0; sum_y = 0; sum_z = 0;
	
	for(uint16_t i = 0; i < 400; i++)
	{
		I2C1_ReadXYZ_Raw(GYRO_ADDRES, &gx, &gy, &gz);
		sum_x += gx;
		sum_y += gy;
		sum_z += gz;
		Delay_ms(1);
	}
	*bias_x = (int16_t)(sum_x / 400); 
	*bias_y = (int16_t)(sum_y / 400); 
	*bias_z = (int16_t)(sum_z / 400);
}

void gyro_processed_values(Gyro_t* g)
/*
 * @brief  calculation of filtered values on each axis
 * @param  address to struct Gyro_t value
 * @retval None
 */
{
	int16_t gx, gy, gz;
	float x_dps, y_dps, z_dps;
	I2C1_ReadXYZ_Raw(GYRO_ADDRES, &gx, &gy, &gz);
	
	gx -= g->bias_x; gy -= g->bias_y; gz -= g->bias_z;
	
	x_dps = gx * GYRO_250DPS_SCALE; y_dps = gy * GYRO_250DPS_SCALE; z_dps = gz * GYRO_250DPS_SCALE;
	
	g->x_fil = g->x_fil * g->alpha + (1.0f - g->alpha) * x_dps;
	g->y_fil = g->y_fil * g->alpha + (1.0f - g->alpha) * y_dps;
	g->z_fil = g->z_fil * g->alpha + (1.0f - g->alpha) * z_dps;
}

void TIM3_Init_800Hz(void)
/*
 * @brief  initialization for TIM3, to read sensor data with freq = 800Hz
 * @param  None
 * @retval None
 */
{
	
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	TIM3->PSC = 84 - 1;
	TIM3->ARR = 1250 - 1;  
	
	TIM3->DIER |= TIM_DIER_UIE;  
	TIM3->CR1 |= TIM_CR1_CEN;   
	
	NVIC_EnableIRQ(TIM3_IRQn);  
}


void TIM3_IRQHandler(void)
{
	if (TIM3->SR & TIM_SR_UIF) {
			TIM3->SR &= ~TIM_SR_UIF; 
			sensor_ready = 1;         
	}
}
	
	
	
	
	
	
