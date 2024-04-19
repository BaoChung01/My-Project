/*
 * i2c_master_tx.c
 *
 *  Created on: Apr 13, 2024
 *      Author: Bao Chung
 */


#include <stdio.h>
#include <string.h>
#include "stm32f407xx.h"

#define MY_ADDR 0x61

#define SLAVE_ADDR MY_ADDR

void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}

I2C_Handle_t I2C1Handle;

//rcv buffer
uint8_t rvc_buff[32];
/*
 * PB6 ->  SCL
 * PB7 -> SDA
 */
void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;

	// note: internal pull up resistors are used

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);

	//SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&I2CPins);
}

void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle  = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);
}

void GPIO_ButtonInit()
{
	GPIO_Handle_t GPIOBtn;
	GPIOBtn.pGPIOx = GPIOA;

	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_Init(&GPIOBtn);
}

int main(void)
{
	uint8_t commmandcode;
	uint8_t len;
	GPIO_ButtonInit();

	//i2c pin inits
	I2C1_GPIOInits();

	//i2c peripheral configuration
	I2C1_Inits();

	while(1)
	{
		//wait till button is pressed
		while(!(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0)));

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commmandcode = 0x51;

		I2C_MasterSendData(&I2C1Handle,&commmandcode,1,SLAVE_ADDR,I2C_ENABLE_SR);

		I2C_MasterSendData(&I2C1Handle, &len, 1, SLAVE_ADDR, SLAVE_ADDR);

		commmandcode = 0x52;
		I2C_MasterSendData(&I2C1Handle,&commmandcode,1,SLAVE_ADDR,I2C_ENABLE_SR);


		I2C_MasterReceiveData(&I2C1Handle,rvc_buff,len,SLAVE_ADDR,I2C_DISABLE_SR);

		rvc_buff[len+1] = '\0';
	}
	return 0;
}
