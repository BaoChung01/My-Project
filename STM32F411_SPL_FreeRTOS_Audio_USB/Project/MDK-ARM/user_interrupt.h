#ifndef USER_INTERRUPT
#define USER_INTERRUPT

#include "stm32f4xx.h"                  // Device header

#define INITIAL_COUNT 5

extern volatile uint32_t i;
extern volatile uint32_t j;
extern volatile uint32_t msTick;
extern volatile uint8_t pressButton;
extern volatile uint16_t timeButton;
extern volatile uint8_t stateButton;
extern volatile uint16_t timeMotionSensor_u16;


typedef enum{
	Disable,
	Enable
}EnableDisable_t;

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


#endif
