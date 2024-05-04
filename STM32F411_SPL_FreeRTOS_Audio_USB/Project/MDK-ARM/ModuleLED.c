/*
* Filename: ModuleLED.c
* Content: handle ModuleLED source code of the program
*/
#include "ModuleLED.h"

#define DELAY_200MS 				200
#define DELAY_1000MS 				1000
#define DELAY_500MS 				500
#define ALL_LED 					  (GPIO_Pin_15 | GPIO_Pin_14 | GPIO_Pin_13 | GPIO_Pin_12)
#define TURN_ON_LED 				GPIO_SetBits(GPIOD, ALL_LED)
#define TURN_OFF_LED 				GPIO_ResetBits(GPIOD, ALL_LED)

/* Private functions */
extern void LED_blink_200ms(void);
extern void LED_blink_500ms(void);
extern void LED_blink_1000ms(void);

/** 
* This function use PD15, PD14, PD13, PD12 pins
* @brief Init LED module
*/
void Init_LED(void)
{
	/* Enable RCC for GPIO port D */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	
	/* GPIO Output PD15, PD14, PD13, PD12 */
	GPIO_InitTypeDef init_GPIO;
	init_GPIO.GPIO_Mode = GPIO_Mode_OUT;
	init_GPIO.GPIO_PuPd = GPIO_PuPd_NOPULL;
	init_GPIO.GPIO_OType = GPIO_OType_PP;
	init_GPIO.GPIO_Pin = ALL_LED;
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








