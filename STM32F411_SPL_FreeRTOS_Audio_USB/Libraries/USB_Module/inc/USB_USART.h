#ifndef 	__USB_USART_H__
#define 	__USB_USART_H__

#include "stm32f4xx_conf.h"

void USART6_init(uint32_t bound);
void USART6_WriteData(uint16_t *pChar);
uint16_t USART6_ReceiveData(void);

#endif
