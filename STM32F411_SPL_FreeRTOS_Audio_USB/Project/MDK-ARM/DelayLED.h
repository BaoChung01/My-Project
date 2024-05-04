/*
* Filename: DelayLED.h
* Content: DelayLED library of the program
*/
#ifndef _DELAYLED_H_
#define _DELAYLED_H_

#include "stm32f4xx.h"                  // Device header

/** 
* This function use the TIM2 with interrupt  
* @brief  This function use to initialize the timer for Delay_ms function 
*/
void Init_timerDelay(void);

#endif


