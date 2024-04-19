/*
** ###################################################################
**     Processor:           STM32F411E DISCOVERY
**     Compiler:            Keil ARM C/C++ Compiler
**     Version:             rev. 1.0, 06/03/2024 - 19:58:27
**
**     Abstract:
**         Build LED.c for Stm32f411e Discovery
**
** ###################################################################
*/

#include "ModuleLED.h"


#define DELAY_200MS 200
#define DELAY_1000MS 1000
#define DELAY_500MS 500


#define ALL_LED (GPIO_Pin_15 | GPIO_Pin_14 | GPIO_Pin_13 | GPIO_Pin_12)
#define TURN_ON_LED GPIO_SetBits(GPIOD, ALL_LED)
#define TURN_OFF_LED GPIO_ResetBits(GPIOD, ALL_LED)

/* Private functions */
extern void LED_blink_200ms(void);
extern void LED_blink_500ms(void);
extern void LED_blink_1000ms(void);

/* 
* Init Timer for Delay_ms function 
* This function just work coreclly at 72Mhz (stm32f103c8t6).
* This function using lED PC13  
*/
void Init_LED(void)
{
	/* Enable RCC for GPIO PC13 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	
	/* GPIO Output PD15, PD14, PD13, PD12 */
	GPIO_InitTypeDef init_GPIO;
	init_GPIO.GPIO_Mode = GPIO_Mode_OUT;
	init_GPIO.GPIO_PuPd = GPIO_PuPd_NOPULL;
	init_GPIO.GPIO_OType = GPIO_OType_PP;
	init_GPIO.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_14 | GPIO_Pin_13 | GPIO_Pin_12;
	init_GPIO.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &init_GPIO);
}

/**
 * @brief  Operation of LED at startup state (2s)
 * @retval Startup result: 
 *								- LED_Output_OK: Startup completed success
 * 								- LED_Output_Error: Startup completed failure			
 */
LED_Output LED_startup(void)
{
	/* Blink all LED every 0.2s */
	TURN_ON_LED;
	USER_DelayMs(DELAY_200MS);
	TURN_OFF_LED;
	USER_DelayMs(DELAY_200MS);
	
	/* ---------------------------------------------------------------------------------------------------------------- */
	// Add soucre code in startup state of another module here 
	
	/* ---------------------------------------------------------------------------------------------------------------- */
	
	return LED_Output_OK;
}

/**
 * @brief  Operation of LED at Running state 
 */






