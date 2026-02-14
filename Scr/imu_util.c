#include "stm32f4xx.h"                  // Device header
#include "system_stm32f4xx.h"

#include "imu_util.h"
#include "system.h"

//IRQ
void TIM3_IRQHandler(void);
void I2C1_EV_IRQHandler(void);
void DMA1_Stream0_IRQHandler(void);

//local func's
void I2C1_Start(void);
void I2C1_Stop(void);
uint8_t I2C1_WriteByte(uint8_t data);
uint8_t I2C1_ReadByte(uint8_t nack);
uint8_t I2C_ADDR(uint8_t Address, uint8_t write_read);
void I2C1_ReadXYZ_Raw(uint8_t Address, int16_t *gx, int16_t *gy, int16_t *gz);

//flags
volatile uint8_t gyro_ready = 0;
volatile uint8_t accel_ready = 0;

//buffer
static volatile uint8_t i2c_buffer[MAX_LEN_I2C] = {0};
uint8_t gyro_buffer[MAX_LEN_I2C] = {0};
uint8_t accel_buffer[MAX_LEN_I2C] = {0};

//variables
static volatile state_machine_t i2c_sm = {
													.state = I2C_STATE_FREE,
													.curr_sensor = Gyro};
static const volatile uint8_t sensor_addr[SENSOR_COUNT] = {
															[Gyro] = GYRO_ADDRES,
															[Accelerometer] = ACCELER_ADDRES};



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

void I2C_DMA_init_forRead(void)
/*
 * @brief  function to setup i2c+dma irq's
 * @param  None
 * @retval None
 */
{
	//event irq
	I2C1->CR2 |= I2C_CR2_ITEVTEN;
	//dma request
	I2C1->CR2 |= I2C_CR2_DMAEN;
	
	//DMA
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
		//RX//
	// turn off dma stream to set up
	DMA1_Stream0->CR &= ~DMA_SxCR_EN;
	while(DMA1_Stream0->CR & DMA_SxCR_EN);
	//source of data
	DMA1_Stream0->PAR = (uint32_t)&(I2C1->DR);
	//receiver of data
	DMA1_Stream0->M0AR = (uint32_t)i2c_buffer;
	//size of buffer
	DMA1_Stream0->NDTR = (uint16_t)(MAX_LEN_I2C - 1);
	//config for DMA // channel 4, memory increment, circular mode, per->mem
	DMA1_Stream0->CR = (1UL << DMA_SxCR_CHSEL_Pos) | 
											DMA_SxCR_MINC |
											DMA_SxCR_TCIE;
	
	//NVIC enable
	NVIC_EnableIRQ(I2C1_EV_IRQn);
	NVIC_EnableIRQ(DMA1_Stream0_IRQn);
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

void gyro_processed_values(Gyro_t* g, uint8_t* gyro_buf)
/*
 * @brief  calculation of filtered values on each axis
 * @param  address to struct Gyro_t value
 * @retval None
 */
{
	int16_t gx, gy, gz;
	float x_dps, y_dps, z_dps;
	gx = (int16_t)(gyro_buf[1] << 8 | gyro_buf[0]);
	gy = (int16_t)(gyro_buf[3] << 8 | gyro_buf[2]);
	gz = (int16_t)(gyro_buf[5] << 8 | gyro_buf[4]);
	
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

void imu_util_init(Gyro_t* gyro)
/*
 * @brief  all init func's
 * @param  struct gyro
 * @retval None
 */
{
	//imu init func's
	GPIO_I2C_Init();
	I2C_init();
	I2C1_ctrl_reg_gyro();
	
	gyro_struct_init(gyro);
	
	//calibration
	calibration_gyro(&gyro->bias_x, &gyro->bias_y, &gyro->bias_z);
	
	//setup for i2c+dma
	I2C_DMA_init_forRead();
	TIM3_Init_800Hz();
}


void TIM3_IRQHandler(void)
{
	if (TIM3->SR & TIM_SR_UIF) {
		TIM3->SR &= ~TIM_SR_UIF; 
		
		if(i2c_sm.state != I2C_STATE_FREE) return;
		
		//start
		I2C1->CR1 |= I2C_CR1_START;
		//state - Start
		i2c_sm.state++;
	}

}

void I2C1_EV_IRQHandler(void)
{
	switch (i2c_sm.state)
  {
		//state 1 - waiting for SB and send sensor address(W)
  	case I2C_STATE_START:
			if(I2C1->SR1 & I2C_SR1_SB){
				//get the desired address
				uint8_t addr = sensor_addr[i2c_sm.curr_sensor];//address + W
				I2C1->DR = ADDR_WRITE(addr); 
				i2c_sm.state++;
			}
  		break;
		//state 2 - clear addr flag and send subaddress
		case I2C_STATE_ADDR_W:
			if(I2C1->SR1 & I2C_SR1_ADDR){
				(void)I2C1->SR1; 
				(void)I2C1->SR2;
				I2C1->DR = SENSOR_SUBADDR_REG | 0x80; // autoinc
				i2c_sm.state++;
			}
  		break;
		//state 3 - restart
		case I2C_STATE_RESTART:
			if(I2C1->SR1 & I2C_SR1_BTF){
				 I2C1->CR1 |= I2C_CR1_START;
			   i2c_sm.state++;
			}
  		break;
		//state 4 - waiting for SB and send sensor address(R)
		case I2C_STATE_ADDR_R:
			if(I2C1->SR1 & I2C_SR1_SB){
				uint8_t addr = sensor_addr[i2c_sm.curr_sensor];//address + R
				I2C1->DR = ADDR_READ(addr); 
				i2c_sm.state++;
			}
  		break;
		//state 5 - addr clear and dam en
		case I2C_STATE_ADDR_CLEAR:
			if(I2C1->SR1 & I2C_SR1_ADDR){
				I2C1->CR1 |= I2C_CR1_ACK;
				(void)I2C1->SR1; 
				(void)I2C1->SR2;
				
				DMA1->LIFCR = DMA_LIFCR_CTCIF0;
				i2c_sm.state++;
				DMA1_Stream0->CR |= DMA_SxCR_EN;
				
			}
  		break;
		//unused state's
		case I2C_STATE_FREE:
		case I2C_STATE_DMA_RUN:	
			break;
  }
}

void DMA1_Stream0_IRQHandler(void)
{
	if(DMA1->LISR & DMA_LISR_TCIF0)
	{
		DMA1->LIFCR = DMA_LIFCR_CTCIF0;
		
		if(i2c_sm.state == I2C_STATE_DMA_RUN)
		{
			//6 byte recive
      I2C1->CR1 &= ~I2C_CR1_ACK;
			I2C1->CR1 |= I2C_CR1_STOP;
			//blocking :(
			while (!(I2C1->SR1 & I2C_SR1_RXNE));
			
			i2c_buffer[5] = (uint8_t)(I2C1->DR);
			
			if (i2c_sm.curr_sensor == Gyro) 
			{
				//copy i2c_buffer to gyro_buffer(global) 
				gyro_buffer[0] = i2c_buffer[0]; 
				gyro_buffer[1] = i2c_buffer[1];
				gyro_buffer[2] = i2c_buffer[2]; 
				gyro_buffer[3] = i2c_buffer[3];
				gyro_buffer[4] = i2c_buffer[4]; 
				gyro_buffer[5] = i2c_buffer[5];
				
				gyro_ready = 1;
			} 
			else if (i2c_sm.curr_sensor == Accelerometer) 
			{
				//copy i2c_buffer to gyro_buffer(global) 
				accel_buffer[0] = i2c_buffer[0]; 
				accel_buffer[1] = i2c_buffer[1];
				accel_buffer[2] = i2c_buffer[2]; 
				accel_buffer[3] = i2c_buffer[3];
				accel_buffer[4] = i2c_buffer[4]; 
				accel_buffer[5] = i2c_buffer[5];
				
				accel_ready = 1;
			}

			//i2c_sm.curr_sensor = (i2c_sm.curr_sensor + 1) % SENSOR_COUNT;
			
			if(i2c_sm.curr_sensor != Gyro)
			{
				//start
				I2C1->CR1 |= I2C_CR1_START;
				//state - Start
				i2c_sm.state = I2C_STATE_START;
			}
			else
			{
				i2c_sm.state = I2C_STATE_FREE;
			}
            
		}
		
	}
}




	
	
	
	
	
	
