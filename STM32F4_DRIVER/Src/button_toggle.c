/*
 * button_toggle.c
 *
 *  Created on: Apr 8, 2024
 *      Author: Bao Chung
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio.h"

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000 ; i ++);
}

int main(void)
{
	GPIO_Handle_t gpioinit, buttoninit;

	//this is led gpio configuration
	gpioinit.pGPIOx = GPIOD;
	gpioinit.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioinit.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	gpioinit.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioinit.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	gpioinit.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&gpioinit);

	//this is button gpio configuration
	buttoninit.pGPIOx = GPIOA;
	buttoninit.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	buttoninit.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	buttoninit.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	buttoninit.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&buttoninit);
	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0))
		{
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
			delay();
		}

	}
	return 0;
}

