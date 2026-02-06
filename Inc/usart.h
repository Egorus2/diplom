#ifndef USART_H
#define USART_H

#include <stdint.h>

#define TIMEOUT_USART 3UL
#define MAX_LEN_RX 32UL

//init
void GPIO_USART1_Init(void);
void USART1_Init(void);
void DMA2_USART1_RX_TX_Init(void);

//Trancmit
uint8_t usart1_Transm_byte(uint8_t mess, uint8_t Timout_ms);
void usart1_Transm_str(const char* str, uint8_t Timout_ms);


#endif
