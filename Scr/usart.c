 #include "stm32f4xx.h"   

#include "usart.h"

//interrup
void USART1_IRQHandler(void);
//unused func
uint8_t usart1_Recive_str_blocking(char* buf, uint8_t max_len);
int16_t usart1_Recive_byte_blocking(void);
int16_t usart1_Recive_byte(uint8_t timout);


static volatile uint8_t rx_buffer[MAX_LEN_RX] = {0};
extern volatile uint64_t msCounter;

void GPIO_USART1_Init(void)
/*
 * @brief  GPIO USART1 initialization 
 * @param  None
 * @retval None
 */
{
	//clock enable
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	
	//PA9 - TX//
	//MODER AF
	GPIOA->MODER &= ~(3UL << GPIO_MODER_MODER9_Pos);
	GPIOA->MODER |= (2UL << GPIO_MODER_MODER9_Pos);
	// OTYPER push-pull
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT9;
	// Port speed - hight
	GPIOA->OSPEEDR &= ~(3UL << GPIO_OSPEEDR_OSPEED9_Pos);
	GPIOA->OSPEEDR |= (2UL << GPIO_OSPEEDR_OSPEED9_Pos);
	//AF set
	GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL9_Msk;
	GPIOA->AFR[1] |= (7UL << GPIO_AFRH_AFSEL9_Pos);
	
	//PA10 - RX//
	//MODER AF
	GPIOA->MODER &= ~(3UL << GPIO_MODER_MODER10_Pos);
	GPIOA->MODER |= (2UL << GPIO_MODER_MODER10_Pos);
	// OTYPER push-pull
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT10;
	// Port speed - hight
	GPIOA->OSPEEDR &= ~(3UL << GPIO_OSPEEDR_OSPEED10_Pos);
	GPIOA->OSPEEDR |= (2UL << GPIO_OSPEEDR_OSPEED10_Pos);
	// AF set
	GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL10_Msk;
	GPIOA->AFR[1] |= (7UL << GPIO_AFRH_AFSEL10_Pos);

}

void USART1_Init(void)
/*
 * @brief  USART1 initialization 
 * @param  None
 * @retval None
 */
{

	//line speed 115200 bod
	USART1->BRR = (45 << 4) | 9;
	// word lenght - 8 bit, stop bit - 1
	USART1->CR1 &= ~USART_CR1_M;
	USART1->CR2 &= ~USART_CR2_STOP;
	//TE RE
	USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;
	//USART EN
	USART1->CR1 |= USART_CR1_UE;
	
	//dma enable receiver
	USART1->CR3 |= USART_CR3_DMAR | USART_CR3_DMAT;
	
	// interrupt settings
	
	// enable IRQ IDLE
	USART1->CR1 |= USART_CR1_IDLEIE;
	//NVIC EN
	NVIC_EnableIRQ(USART1_IRQn);
	
}

void DMA2_USART1_RX_TX_Init(void)
/*
 * @brief  DMA2 USART1 initialization RX
 * @param  None
 * @retval None
 */
{
	//RCC
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
	
	//RX//
	// turn off dma stream to set up
	DMA2_Stream5->CR &= ~DMA_SxCR_EN;
	while(DMA2_Stream5->CR & DMA_SxCR_EN);
	//source of data
	DMA2_Stream5->PAR = (uint32_t)&(USART1->DR);
	//receiver of data
	DMA2_Stream5->M0AR = (uint32_t)rx_buffer;
	//size of buffer
	DMA2_Stream5->NDTR = (uint16_t)MAX_LEN_RX;
	//config for DMA // channel 4, memory increment, circular mode, per->mem
	DMA2_Stream5->CR = (4UL << DMA_SxCR_CHSEL_Pos) | 
											DMA_SxCR_MINC | 
											DMA_SxCR_CIRC;
	//enable DMA
	DMA2_Stream5->CR |= DMA_SxCR_EN;
	
	//TX//
	// turn off dma stream to set up
	DMA2_Stream7->CR &= ~DMA_SxCR_EN;
	while(DMA2_Stream7->CR & DMA_SxCR_EN);
	//Peripheral address
	DMA2_Stream7->PAR = (uint32_t)&(USART1->DR);
	//Memory address
	DMA2_Stream7->M0AR = (uint32_t)rx_buffer;
	//size of buffer
	DMA2_Stream7->NDTR = 0;
	//config for DMA // channel 4, memory increment, mem->per
	DMA2_Stream7->CR = (4UL << DMA_SxCR_CHSEL_Pos) | 
											DMA_SxCR_MINC | 
											DMA_SxCR_DIR_0;

}


uint8_t usart1_Transm_byte(uint8_t byte, uint8_t Timout_ms)
/*
 * @brief  usart transmit 1 byte
 * @param  message, timeout value
 * @retval 0 - error, 1 - successful execution
 */
{
	uint64_t Start_ms = msCounter;
	while(!(USART1->SR & USART_SR_TXE)){
		if((msCounter - Start_ms) >= Timout_ms){
			return 0;
		}
	}

	USART1->DR = byte;
	return 1;
}


void usart1_Transm_str(const char* str, uint8_t Timout_ms)
/*
 * @brief  usart1 transmit str with timeout
 * @param  str, error delay
 * @retval None
 */
{
	while(*str)
	{
		if(!usart1_Transm_byte((uint8_t)*str, Timout_ms))
			return;
		str++;
	}
}

int16_t usart1_Recive_byte(uint8_t timout)
/*
 * @brief  usart1 Recive byte with timeout
 * @param  error delay
 * @retval recived byte
 */
{
	uint64_t start = msCounter;
	while(!(USART1->SR & USART_SR_RXNE))
	{
		if((msCounter - start) < timout)
		{
			return -1;
		}
	}
	return (int16_t)(USART1->DR);
}

int16_t usart1_Recive_byte_blocking(void)
/*
 * @brief  usart1 Recive byte with no timeout, blocking function
 * @param  None
 * @retval recived byte
 */
{
	while(!(USART1->SR & USART_SR_RXNE));
	return (int16_t)(USART1->DR);
}

uint8_t usart1_Recive_str_blocking(char* buf, uint8_t max_len)
/*
 * @brief  usart1 Recive string with no timeout, blocking function
 * @param  buffer, max length of the string
 * @retval length of the string
 */
{
	uint8_t i = 0;   // current lenth
	int16_t c;       // recived symbol
	while(i < (max_len - 1))
	{
		c = usart1_Recive_byte_blocking();
		
		if(c == 0x08 && i > 0)     // backspace
			i --;
		else{
			if(c == '\r' || c == '\n') // CR or enter
			{
				buf[i] = '\0';						// end of the string
				return i;
			}
			
			buf[i++] = (char)c;	
		}	
	}
		buf[i] = '\0';       // end of the string
		return i;
}

void USART1_IRQHandler(void)
/*
 * @brief  usart1 inerrupt handler
 * @param  None
 * @retval None
 */
{
	if(USART1->SR & USART_SR_IDLE)
	{
		//sequence to clear IDLE flag
		volatile uint32_t tmp = USART1->SR;
		tmp = USART1->DR;
		//stop DMA RX
		DMA2_Stream5->CR &= ~DMA_SxCR_EN;
		while(DMA2_Stream5->CR & DMA_SxCR_EN);
		// length of current package
		uint32_t ndtr = DMA2_Stream5->NDTR;
		uint32_t len = MAX_LEN_RX - ndtr;	
		
		//clear end of tx flag
		DMA2->HIFCR |= DMA_HIFCR_CTCIF7;
		//stop DMA TX
		DMA2_Stream7->CR &= ~DMA_SxCR_EN;
		while(DMA2_Stream7->CR & DMA_SxCR_EN);
		DMA2_Stream7->NDTR = len;   //len of transmit 
		
		//reenable DMA
		DMA2_Stream7->CR |= DMA_SxCR_EN;
		DMA2_Stream5->CR |= DMA_SxCR_EN;
	}
	
}
