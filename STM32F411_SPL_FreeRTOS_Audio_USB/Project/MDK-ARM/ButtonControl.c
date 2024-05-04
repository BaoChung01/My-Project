/*
* Filename: ButtonControl.c
* Content: handle the source code of the button to control the audio
*/
#include "ButtonControl.h"

#define LED_ON 					1
#define LED_OFF 				0

/* Using show Led state */
volatile int ledState = LED_OFF;

/** 
* This function using EXTI4 of PORTA (PA4) 
* @brief  Init EXTI4 to use the button control 
*/
void Button_init(void)
{
	/* Enable RCC for Port A */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	/* Enable RCC for EXTI */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
	/* Init GPIO input for PA0 */
	GPIO_InitTypeDef initGPIOx;
	initGPIOx.GPIO_Mode = GPIO_Mode_IN;  
	initGPIOx.GPIO_Pin = GPIO_Pin_0;
	initGPIOx.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &initGPIOx);									
	
	/* Connect pin PA4 to EXTI4 */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);	
	
	/* Init EXTI for PA0 */
	EXTI_InitTypeDef initEXTIx;
	initEXTIx.EXTI_Line = EXTI_Line0;
	initEXTIx.EXTI_Mode = EXTI_Mode_Interrupt;
	initEXTIx.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	initEXTIx.EXTI_LineCmd = ENABLE;
	EXTI_Init(&initEXTIx);
	
	/* Init NVIC for EXTI of PA0*/
	NVIC_InitTypeDef initNVICx;
	initNVICx.NVIC_IRQChannel = EXTI0_IRQn;
	initNVICx.NVIC_IRQChannelPreemptionPriority = 0;
	initNVICx.NVIC_IRQChannelSubPriority = 0;
	initNVICx.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&initNVICx);
}


