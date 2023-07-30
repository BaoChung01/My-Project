#ifndef _STM32_RCC_H_
#define _STM32_RCC_H_

#include "stm32f1.h"

//this returns the APB1 clock value
uint32_t RCC_GetPCLK1Value(void);

//this returns the APB2 clock value
uint32_t RCC_GetPCLK2Value(void);

#endif 