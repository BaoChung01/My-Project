#include "user_interrupt.h"

#define BUTTON_RELEASE 0
#define BUTTON_PRESS 1

Interrupt_t USER_Interrupt;

//USER_Interrupt.timeButton_u16 = (uint16_t)3000;

ModeMain_t VariableMode;


/* Interrupt for TIM2 Fucntion */
void TIM2_IRQHandler(void)
{
	/* Check interrupt flag TIM2, if it's RESET value -> Clear it */
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != 0)
	{
		/* Clear Flag Interrupt */
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		
		/* Active */
		/* Timer */
		USER_Interrupt.msTick_u32++;
      
		/* 
		Time of Motion Sensor update data 
		After receiving a command, it takes at least 2s to receive the next command
		*/
		if (USER_Interrupt.timeMotionSensor_u16 > 0){
			USER_Interrupt.timeMotionSensor_u16--;
		}
      
		/* 
		Check button
		timeButton to distinguish press and hold button
		If stateButton != 0 <=> Press button
		timeButton-- for check to see if the button is pressed or hold
			if stateButton != 0 during 3000ms => hold button
		If stateButton == 0 <=> Release button => If timeButton < 3000 => Press -> Release
		pressButton to determine number of consecutive button presses
		Reset timeButton as initial
		*/
		if (USER_Interrupt.stateButton_bit == BUTTON_PRESS){                             //press button
			if (USER_Interrupt.timeButton_u16 > 0){
				USER_Interrupt.timeButton_u16--;
			}
			else if (USER_Interrupt.timeButton_u16 == 0){
				ON_BIT(VariableMode.RejectUSB_bit, BIT_0);
				USER_Interrupt.timeButton_u16 = TIME_BUTTON_3000MS;
				//while (stateButton != 0){}                                   //wait release button
				} 
			}
		else{                                                            //release button
		/*  */
			if (USER_Interrupt.timeButton_u16 < (TIME_BUTTON_3000MS-20)){      //20ms to anti-jamming
				USER_Interrupt.pressButton_u8++;
				if (USER_Interrupt.pressButton_u8 >= STATE_BUTTON(MAX)){
					USER_Interrupt.pressButton_u8 = STATE_BUTTON(SKIP);
				}
				/* press button will be reset on function get button */
			}
			/* Reset time Button */
			USER_Interrupt.timeButton_u16 = TIME_BUTTON_3000MS;
		}
	}
}

/* Interrupt for Button Fucntion */
void EXTI0_IRQHandler(void)
{
	/* Check and clear flag interrupt EXTI4 */
	if (EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		/* Clear Flag */
		EXTI_ClearITPendingBit(EXTI_Line0);
      
		/* Active */
		TOGGLE_BIT(USER_Interrupt.stateButton_bit, BIT_0);
	}
}
