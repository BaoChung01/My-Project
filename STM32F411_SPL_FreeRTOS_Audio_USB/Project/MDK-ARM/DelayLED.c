/*
** ###################################################################
**     Processor:           STM32F411E DISCOVERY
**     Compiler:            Keil ARM C/C++ Compiler
**     Version:             rev. 1.0, 06/03/2024 - 19:58:27
**
**     Abstract:
**         Build DelayLED.c for Stm32f411e Discovery
**
** ###################################################################
*/

#include "DelayLED.h"



#define SYS_CLOCK						100000000
#define DELAY_1SECOND 				1000
#define TRUE 		 		  				1
#define VALUE_INIT_0  				0
#define TIM_PRESCALER_84MHZ (8400 - 1)
#define TIM_PERIOD_MS 			(1000 - 1)


/* 
* Init Timer for Delay_ms function 
* This function just work coreclly at 100Mhz (STM32F411E) 
* This function using TIM2 and using interrupt 
*/
void Init_timerDelay(void)
{
	/* Enable RCC for TIM2 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	/* Init TIM2 */
	TIM_TimeBaseInitTypeDef initTimer;
	initTimer.TIM_CounterMode = TIM_CounterMode_Up;
	initTimer.TIM_Prescaler = ((SYS_CLOCK / 1000000) - 1);
	initTimer.TIM_Period = 1000 - 1;
	TIM_TimeBaseInit(TIM2, &initTimer); 
	
	/* Init NVIC for TIM2 */
	NVIC_InitTypeDef initNVIC_Timer;
	initNVIC_Timer.NVIC_IRQChannel = TIM2_IRQn;
	initNVIC_Timer.NVIC_IRQChannelCmd = ENABLE;
	initNVIC_Timer.NVIC_IRQChannelPreemptionPriority = 7;
	initNVIC_Timer.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&initNVIC_Timer);
	
	TIM_ITConfig(TIM2,TIM_IT_Update, ENABLE);
	
	/* Enable TIM2 */
	TIM_Cmd(TIM2, ENABLE);
}




