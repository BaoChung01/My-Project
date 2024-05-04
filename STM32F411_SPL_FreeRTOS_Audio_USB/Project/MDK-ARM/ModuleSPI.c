/*
* Filename: ModuleSPI.c
* Content: handle ModuleSPI source code of the program
*/
#include "stm32f4xx.h"                  // Device header
#include "ModuleSPI.h"

#define CS_PIN 				GPIO_Pin_4
#define MOSI_PIN			GPIO_Pin_7
#define MISO_PIN 			GPIO_Pin_6
#define SCK_PIN 			GPIO_Pin_5

/** 
* This function using EXTI4 of PORTA (PA4) 
* @brief  Init EXTI4 to use the button control 
*/
void SPI_initModule(void)
{
	/* RCC for SPI1 and Port A */ 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	/* Init MOSI = PA7, MISO = PA6 , SCK = PA5 */
	GPIO_InitTypeDef init_pinSPI;
	init_pinSPI.GPIO_Mode = GPIO_Mode_AF;
	init_pinSPI.GPIO_OType = GPIO_OType_PP;
	init_pinSPI.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	init_pinSPI.GPIO_Speed = GPIO_Speed_50MHz;
	init_pinSPI.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &init_pinSPI);
	
	/* Config chip select */
	init_pinSPI.GPIO_Mode = GPIO_Mode_OUT;
	init_pinSPI.GPIO_OType = GPIO_OType_PP;
	init_pinSPI.GPIO_PuPd = GPIO_PuPd_NOPULL;
	init_pinSPI.GPIO_Speed = GPIO_Speed_50MHz;
	init_pinSPI.GPIO_Pin = GPIO_Pin_4;
	GPIO_Init(GPIOA, &init_pinSPI);
	
	/* When do nothing */
	GPIO_SetBits(GPIOA, GPIO_Pin_4);
	
	/* Connect GPIO to SPI */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);  // SCK
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);  // MISO
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);  // MOSI
	
	/* Config SPI */
	SPI_InitTypeDef init_SPI;
	init_SPI.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	init_SPI.SPI_Mode = SPI_Mode_Master;
	init_SPI.SPI_DataSize = SPI_DataSize_8b;
	init_SPI.SPI_NSS = SPI_NSS_Soft;
	init_SPI.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	init_SPI.SPI_FirstBit = SPI_FirstBit_MSB;
	init_SPI.SPI_CPHA = SPI_CPHA_1Edge;
	init_SPI.SPI_CPOL = SPI_CPOL_Low;
	SPI_Init(SPI1, &init_SPI);
	
	/* Enable SPI1 */
	SPI_Cmd(SPI1, ENABLE);
}

/** 
* This function using SPI1 (PA5 = SCK, PA6 = MISO, PA7 = MOSI, PA4 = CS) 
* @brief Transmit a char datatype using SPI1 
* @param p_data: A char datatype data
*/
void SPI_Transmit_Char(unsigned char p_data)
{
	/* Enable CS pin */
	GPIO_ResetBits(GPIOA, CS_PIN); 
	
	/* Send data */
	SPI_SendData(SPI1, p_data);
	
	/* Disable CS pin */
	GPIO_SetBits(GPIOA, CS_PIN); 
}



