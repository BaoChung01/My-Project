/*
* Filename: ModuleLED.h
* Content: ModuleLED library of the program
*/
#ifndef _MODULELED_H_
#define _MODULELED_H_

#include "stm32f4xx.h"                  // Device header
#include "DelayLED.h"
#include "user_function.h"
#include "user_interrupt.h"

/* LED output Result enumerations */
typedef enum {
	LED_Output_OK,   				// Everything OK
	LED_Output_Error 				// Error occurred
} LED_Output;

/* LED state enumerations */
typedef enum {
	LED_State_Startup,         		// At the startup 2s					
	LED_State_Running_Playing,  	// At the Running State when the song is playing
	LED_State_Running_NotPlay		// At the Running state when the song doesn't play
} LED_State;

/* Public function */
/** 
* This function use PD15, PD14, PD13, PD12 pins
* @brief Init LED module
*/
void Init_LED(void);

/**
 * @brief  Operation of LED at startup state (2s)
 * @retval LED_Output_OK: Startup completed success
 * @retval LED_Output_Error: Startup completed failure			
 */
LED_Output LED_startup(void);

/**
 * @brief  Operation of LED at Running state 								
 */
void LED_running(void);



#endif


