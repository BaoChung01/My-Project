#ifndef USART_HERCULES
#define USART_HERCULES

#include "stm32f4xx.h"

extern volatile uint8_t USART_StringDataSend_aa[30];

void Init_USART(void);
void USART_sendDataString(volatile uint8_t* p_Data);

#endif
