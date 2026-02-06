#include "stm32f4xx.h"                  // Device header
#include "system_stm32f4xx.h"

#include "imu_util.h"
//local func's
void TIM3_IRQHandler(void);
void I2C1_Start(void);
void I2C1_Stop(void);
uint8_t I2C1_WriteByte(uint8_t data);
uint8_t I2C1_ReadByte(uint8_t nack);

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


uint8_t ReadWhoAmI(void)
{
    uint8_t data = 0;
    //start
    I2C1_Start();
    //addres + W
    I2C1->DR = ADDR_WRITE(ACCELER_ADDRES); 
    

    while (!(I2C1->SR1 & I2C_SR1_ADDR)) {
        if (I2C1->SR1 & I2C_SR1_AF) {
            I2C1->SR1 &= ~I2C_SR1_AF;
            I2C1_Stop();
            return 0xFF;
        }
    }
    
    (void)I2C1->SR1; 
    (void)I2C1->SR2;  
		//subaddres
    I2C1->DR = 0x0F;
    while (!(I2C1->SR1 & I2C_SR1_BTF));  
    
		//SR
    I2C1_Start();
		//addres + R
    I2C1->DR = ADDR_READ(ACCELER_ADDRES); 
    
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) {
        if (I2C1->SR1 & I2C_SR1_AF) {
            I2C1->SR1 &= ~I2C_SR1_AF;
            I2C1_Stop();
            return 0xFF;
        }
    }
    (void)I2C1->SR1;
    (void)I2C1->SR2;

    data = I2C1_ReadByte(1);
    
    return data; 
}

void I2C1_ctrl_reg_gyro(void)
{
    //start
    I2C1_Start();
    //addres + W
    I2C1->DR = ADDR_WRITE(GYRO_ADDRES); 
    

    while (!(I2C1->SR1 & I2C_SR1_ADDR)) {
        if (I2C1->SR1 & I2C_SR1_AF) {
            I2C1->SR1 &= ~I2C_SR1_AF;
            I2C1_Stop();
        }
    }
    
    (void)I2C1->SR1; 
    (void)I2C1->SR2;  
		//subaddres
    I2C1->DR = 0x20;
    while (!(I2C1->SR1 & I2C_SR1_BTF));  

    I2C1_WriteByte(0xCF);
		
		I2C1_Stop();
    
}

void I3G4250D_ReadGyro(int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t data[6];
    
    I2C1_Start();
    I2C1->DR = 0xD0;  
    while (!(I2C1->SR1 & I2C_SR1_ADDR)); 
		(void)I2C1->SR1; (void)I2C1->SR2;
    
    I2C1->DR = (0x28 | 0x80);  
    while (!(I2C1->SR1 & I2C_SR1_BTF));
    
   
    I2C1_Start();
    I2C1->DR = 0xD1;  
    while (!(I2C1->SR1 & I2C_SR1_ADDR)); 
		(void)I2C1->SR1; (void)I2C1->SR2;
    
   
    for (int i = 0; i < 5; i++) {
        data[i] = I2C1_ReadByte(0); 
    }
    data[5] = I2C1_ReadByte(1);     
    
    
    *gx = (int16_t)(data[1] << 8 | data[0]);  // X: OUT_X_H << 8 | OUT_X_L
    *gy = (int16_t)(data[3] << 8 | data[2]);  // Y: OUT_Y_H << 8 | OUT_Y_L
    *gz = (int16_t)(data[5] << 8 | data[4]);  // Z: OUT_Z_H << 8 | OUT_Z_L
}

void TIM3_Init_1kHz(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    TIM3->PSC = 83;
    TIM3->ARR = 999;
    
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
	
	
	
	
	
	
