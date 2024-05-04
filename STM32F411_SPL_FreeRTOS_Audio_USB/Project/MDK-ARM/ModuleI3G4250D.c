/*
* Filename: ModuleI3G4250D.c
* Content: handle ModuleI3G4250D source code of the program
*/
#include "ModuleI3G4250D.h"

#define READ_MODE 				0x80

/* Output Sensor */
I3G4250D_Output_uint8_t I3G4250D_Data;

static GPIO_InitTypeDef GPIOInit;
static SPI_InitTypeDef  SPIinit;
/**************************************************************************************************/
/* Private function */
void GPIOConfig(void);
void SPIConfig(void);
void SPI_Tx(uint8_t adress, uint8_t data);
uint8_t SPI_Rx(uint8_t adress);

/**************************************************************************************************/
/* Public function */
I3G4250D_Result_t I3G4250D_Init(void)
{
	int result;
	GPIOConfig();
	SPIConfig();
	I3G4250D_HIGH_CS;
	
	I3G4250D_LOW_CS;
	SPI_Tx(I3G4250D_CTRL_REG1_ADDR, 0xFF);
	
	/* Read data WHO AM I */
	result = SPI_Rx(I3G4250D_WHO_AM_I_ADDR | READ_MODE);			
	if(result != I3G4250D_WHO_AM_I)
	{
		I3G4250D_HIGH_CS;
		return I3G4250D_Result_Error;
	}
	I3G4250D_HIGH_CS;
	return I3G4250D_Result_Ok;
}

I3G4250D_Result_t I3G4250D_Read(I3G4250D_Output_uint8_t* p_I3G4250D_Data)
{
	p_I3G4250D_Data->X = SPI_Rx(I3G4250D_OUT_X_H_ADDR);
	p_I3G4250D_Data->Y = SPI_Rx(I3G4250D_OUT_Y_H_ADDR);
	p_I3G4250D_Data->Z = SPI_Rx(I3G4250D_OUT_Z_H_ADDR);
	
	return I3G4250D_Result_Ok;
}
/**************************************************************************************************/
/* Private function */
void SPIConfig(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	SPIinit.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64; // Clock speed of SPI
	SPIinit.SPI_CPHA = SPI_CPHA_2Edge; // Defines edge for bit capture 2nd edge used
	SPIinit.SPI_CPOL = SPI_CPOL_High; // Clock polarity
	SPIinit.SPI_CRCPolynomial = 7;
	SPIinit.SPI_DataSize = SPI_DataSize_8b; // 8 bit or 16bit
	SPIinit.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // Unidirectional or bidirectional. We use bidirect.
	SPIinit.SPI_FirstBit = SPI_FirstBit_MSB; // Start reading with MSB
	SPIinit.SPI_Mode = SPI_Mode_Master; // stm32f4 will be master
	SPIinit.SPI_NSS = SPI_NSS_Soft; // Hardware or Software

	SPI_Init(SPI1, &SPIinit);
	SPI_Cmd(SPI1, ENABLE);

}

void GPIOConfig(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIOInit.GPIO_Mode = GPIO_Mode_AF;
	GPIOInit.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIOInit.GPIO_OType = GPIO_OType_PP;
	GPIOInit.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIOInit.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOA, &GPIOInit);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIOInit.GPIO_Mode = GPIO_Mode_OUT;
	GPIOInit.GPIO_Pin = GPIO_Pin_3;
	GPIOInit.GPIO_OType = GPIO_OType_PP;
	GPIOInit.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIOInit.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOE, &GPIOInit);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIOInit.GPIO_Mode = GPIO_Mode_OUT;
	GPIOInit.GPIO_OType = GPIO_OType_PP;
	GPIOInit.GPIO_Pin = GPIO_Pin_12;
	GPIOInit.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIOInit.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOD, &GPIOInit);
}

void SPI_Tx(uint8_t adress, uint8_t data)
{
	// adress:  Open the LIS302DL datasheet to find out the adress of a certain register.
  GPIO_ResetBits(GPIOE,GPIO_Pin_3);
  while(!SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE));
  SPI_I2S_SendData(SPI1,adress);
  while(!SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE));
  SPI_I2S_ReceiveData(SPI1);

  while(!SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE));
  SPI_I2S_SendData(SPI1,data);
  while(!SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE));
  SPI_I2S_ReceiveData(SPI1);

  GPIO_SetBits(GPIOE,GPIO_Pin_3);

}

uint8_t SPI_Rx(uint8_t adress)
{
  GPIO_ResetBits(GPIOE,GPIO_Pin_3); //CS pin logic 0
  adress=(READ_MODE) | (adress); //  this tells the sensor to read and not to write, that's where the (0x80 | adress) comes from.

  while(!SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE));
  SPI_I2S_SendData(SPI1,adress);
  while(!SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE));
  SPI_I2S_ReceiveData(SPI1);

  while(!SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE));
  SPI_I2S_SendData(SPI1,0x00);
  while(!SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE));
  SPI_I2S_ReceiveData(SPI1);
  GPIO_SetBits(GPIOE,GPIO_Pin_3); //CS pin logic 1
  return SPI_I2S_ReceiveData(SPI1);
}

