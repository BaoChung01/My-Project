#ifndef _USER_INTERRUPT_
#define _USER_INTERRUPT_

#include "stm32f4xx.h"                  // Device header
#include "common.h"

typedef struct{
	volatile uint32_t i;
	volatile uint32_t j;
	volatile uint32_t msTick_u32;
	volatile uint16_t timeMotionSensor_u16;
	volatile uint16_t timeButton_u16;
	volatile uint8_t pressButton_u8;
	volatile uint8_t stateButton_bit:1;
}Interrupt_t;
extern Interrupt_t USER_Interrupt;

typedef struct{
	/*  */
	uint8_t IntialCount_u8;
	
	/*bit field*/
	EnableDisable_t	StartStop_bit:1;
	EnableDisable_t Skip_bit:1;
	EnableDisable_t Pause_bit:1;
	EnableDisable_t RejectUSB_bit:1;
	EnableDisable_t StateUSB_bit:1;
	EnableDisable_t :1;
	EnableDisable_t :1;
	EnableDisable_t :1;
}ModeMain_t;
extern ModeMain_t VariableMode;

/* Interrupt Fucntion: Blink led by using button PA0 */
void EXTI0_IRQHandler(void);

/* Interrupt for TIM2 Fucntion */
void TIM2_IRQHandler(void);


#endif /* _USER_INTERRUPT_ */
