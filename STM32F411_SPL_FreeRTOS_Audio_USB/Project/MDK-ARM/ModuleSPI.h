/*
** ###################################################################
**     Processor:           STM32F411E DISCOVERY
**     Compiler:            Keil ARM C/C++ Compiler
**     Version:             rev. 1.0, 07/03/2024 - 19:58:27
**
**     Abstract:
**         Build ModuleSPI.h for Stm32f411e Discovery
**
** ###################################################################
*/
#ifndef _MODULESPI_H_
#define _MODULESPI_H_

/** 
* This function using SPI1 (PA5 = SCK, PA6 = MISO, PA7 = MOSI, PA4 = CS) 
* @brief Init SPI1 to use connect with motion sensor 
*/
void SPI_initModule(void);

/** 
* This function using SPI1 (PA5 = SCK, PA6 = MISO, PA7 = MOSI, PA4 = CS) 
* @brief Transmit a char datatype using SPI1 
* @param p_data: A char datatype data
*/
void SPI_Transmit_Char(unsigned char p_data);

/** 
* This function using SPI1 (PA5 = SCK, PA6 = MISO, PA7 = MOSI, PA4 = CS) 
* @brief Receive a char datatype using SPI1 
* @param p_data: A char datatype data
*/
void SPI_Receive_Char(unsigned char p_data);


#endif
