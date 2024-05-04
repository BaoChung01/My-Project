/*
* Filename: DelayLED.c
* Content: handle DelayLED source code of the program
*/
#include "DelayLED.h"

#define SYS_CLOCK                   100000000
#define SYS_CLOCK_DIV_1MHZ          1000000
#define TIM_PERIOD_MS               (1000 - 1)
#define PULSE_REDUNDANT             1

/** 
* This function use the TIM2 with interrupt  
* @brief  This function use to initialize the timer for Delay_ms function 
*/
void Init_timerDelay(void)
{
	/* Enable RCC for TIM2 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	/* Init TIM2 */
	TIM_TimeBaseInitTypeDef initTimer;
	initTimer.TIM_CounterMode = TIM_CounterMode_Up;
	initTimer.TIM_Prescaler = ((SYS_CLOCK / SYS_CLOCK_DIV_1MHZ) - PULSE_REDUNDANT);
	initTimer.TIM_Period = TIM_PERIOD_MS;
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




