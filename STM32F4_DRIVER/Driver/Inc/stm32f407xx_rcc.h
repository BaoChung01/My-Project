/*
 * stm32f407xx_rcc.h
 *
 *  Created on: Apr 12, 2024
 *      Author: Bao Chung
 */

#ifndef INC_STM32F407XX_RCC_H_
#define INC_STM32F407XX_RCC_H_

#include "stm32f407xx.h"
//This returns the APB1 clock value
uint32_t RCC_GetPCLK1Value(void);

//This returns the APB2 clock value
uint32_t RCC_GetPCLK2Value(void);

#endif /* INC_STM32F407XX_RCC_H_ */
