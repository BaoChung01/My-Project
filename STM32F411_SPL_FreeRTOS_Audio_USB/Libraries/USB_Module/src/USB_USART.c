#include "USB_USART.h"
/*
  USART1_RX: C7
  USART1_TX: C6 
*/
void USART6_init(uint32_t bound)
{
		/*-------------------- Init USART ---------------------------*/
	USART_DeInit(USART6);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);	//Enable RCC for USART_1

	/* USART */
	USART_InitTypeDef usart6;
	usart6.USART_BaudRate = bound;
	usart6.USART_WordLength = USART_WordLength_8b;
	usart6.USART_StopBits = USART_StopBits_1;
	usart6.USART_Parity = USART_Parity_No;
	usart6.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart6.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	
	USART_Init(USART6, &usart6);
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
	
	/* GPIO */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_InitTypeDef 		GPIO_InitStructure;
	NVIC_InitTypeDef 		NVIC_InitStructure;
	
	/* USART1_TX: GPIOC.6 */
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
		
	/* USART1_RX: GPIOC.7 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	/* USART1 NVIC */
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_Cmd(USART6, ENABLE);
	SET_BIT(USART6->CR1, USART_CR1_UE);
	SET_BIT(USART6->BRR, 0x683);
	SET_BIT(USART6->CR1, USART_CR1_TE);
}

void USART6_WriteData(uint16_t *pChar)
{
	int i = 0;
	while(*(pChar + i) != '\0')
	{
		USART_SendData(USART6, *(pChar+i));	
		i++;
	}
}

uint16_t USART6_ReceiveData(void)
{
	return USART_ReceiveData(USART6);
}
