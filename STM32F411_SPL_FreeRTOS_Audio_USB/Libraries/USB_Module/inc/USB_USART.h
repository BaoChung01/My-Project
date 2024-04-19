#ifndef 	__USB_USART_H__
#define 	__USB_USART_H__

#include "stm32f4xx_conf.h"

void USART1_init(uint32_t bound);
void USART1_WriteData(uint16_t data);
uint16_t USART1_ReceiveData(void);

#endif
