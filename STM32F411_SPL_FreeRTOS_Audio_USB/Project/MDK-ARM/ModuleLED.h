/*
** ###################################################################
**     Processor:           STM32F411E DISCOVERY
**     Compiler:            Keil ARM C/C++ Compiler
**     Version:             rev. 1.0, 06/03/2024 - 19:58:27
**
**     Abstract:
**         Build LED.h for Stm32f411e Discovery
**
** ###################################################################
*/
#ifndef _MODULELED_H_
#define _MODULELED_H_

#include "stm32f4xx.h"                  // Device header
#include "DelayLED.h"
#include "user_function.h"
#include "user_interrupt.h"

/* LED output Result enumerations */
typedef enum {
	LED_Output_OK,   // Everything OK
	LED_Output_Error // Error occurred
} LED_Output;

/* LED state enumerations */
typedef enum {
	LED_State_Startup,          // At the startup 2s					
	LED_State_Running_Playing,  // At the Running State when the song is playing
	LED_State_Running_NotPlay		// At the Running state when the song doesn't play
} LED_State;

/** 
* This function just work coreclly at 72Mhz (stm32f103c8t6).
* This function using lED PC13  
* @brief  Init Timer for Delay_ms function 
*/
void Init_LED(void);

/**
 * @brief  Operation of LED at startup state (2s)
 * @retval Startup result: 
 *								- LED_Output_OK: Startup completed success
 * 								- LED_Output_Error: Startup completed failure			
 */
LED_Output LED_startup(void);

/**
 * @brief  Operation of LED at Running state 								
 */
void LED_running(void);



#endif


