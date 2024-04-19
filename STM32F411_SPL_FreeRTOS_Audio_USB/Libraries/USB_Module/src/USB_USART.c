#include "USB_USART.h"

/*USART1_RX: A10
	USART1_TX: A9 
*/

void USART1_init(uint32_t bound)
{
		/*-------------------- Init UsART ---------------------------*/
	//USART_DeInit(USART1);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);	//Enable RCC for USART_1

	//USART
	USART_InitTypeDef usart1;
	usart1.USART_BaudRate = bound;
	usart1.USART_WordLength = USART_WordLength_8b;
	usart1.USART_StopBits = USART_StopBits_1;
	usart1.USART_Parity = USART_Parity_No;
	//usart1.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart1.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	
	USART_Init(USART1, &usart1);
	//USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	
	//GPIO
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitTypeDef 		GPIO_InitStructure;
//	NVIC_InitTypeDef 		NVIC_InitStructure;
	
	//USART1_TX: GPIOA.9
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
	//USART1_RX: GPIOA.10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
//	//USART1 NVIC
//	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);


	USART_Cmd(USART1, ENABLE);
}

void USART1_WriteData(uint16_t data)
{
	
	USART_SendData(USART1, data);
}

uint16_t USART1_ReceiveData(void)
{
	return USART_ReceiveData(USART1);
}
