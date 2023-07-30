#ifndef _STM32_GPIO_H_
#define _STM32_GPIO_H_

#include "stm32f1.h"
typedef struct
{
  uint8_t GPIO_PinNumber;
  uint8_t GPIO_PinMode;
  uint8_t GPIO_PinSpeed;
  uint8_t GPIO_PinPuPdControl;
  uint8_t GPIO_PinOPType;
  uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

typedef struct
{
  GPIO_RegDef_t *GPIOx;
  GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;

#define GPIO_PIN_NO_0           0
#define GPIO_PIN_NO_1           1
#define GPIO_PIN_NO_2           2
#define GPIO_PIN_NO_3           3
#define GPIO_PIN_NO_4           4
#define GPIO_PIN_NO_5           5
#define GPIO_PIN_NO_6           6
#define GPIO_PIN_NO_7           7
#define GPIO_PIN_NO_8           8
#define GPIO_PIN_NO_9           9
#define GPIO_PIN_NO_10          10
#define GPIO_PIN_NO_11          11
#define GPIO_PIN_NO_12          12
#define GPIO_PIN_NO_13          13
#define GPIO_PIN_NO_14          14
#define GPIO_PIN_NO_15          15

#define GPIO_MODE_IN            0
#define GPIO_MODE_OUT           1
#define GPIO_MODE_ALTFN         2
#define GPIO_MODE_ANALOG        3
#define GPIO_MODE_IT_FL         4
#define GPIO_MODE_IT_RT         5
#define GPIO_MODE_IT_RFT        6

      
#define GPIO_Speed_10MHz 1
#define GPIO_Speed_2MHz  2
#define GPIO_Speed_50MHz 3

#define GPIO_NO_PUPD    0
#define GPIO_PIN_PU     1
#define GPIO_PIN_PD     2

void GPIO_PeriClockControl(GPIO_RegDef_t *GPIOx, uint8_t EnorDi);
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *GPIOx);

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *GPIOx,uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *GPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *GPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *GPIOx,uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *GPIOx,uint8_t PinNumber);

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif