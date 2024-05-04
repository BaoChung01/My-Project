#include "user_function.h"

#define INTERVAL_PRESS_BUTTON 700
#define INTERVAL_GET_COMMAND_SENSOR 2000
#define POSITIVE 1
#define NEGATIVE -1

MotionSensor_t USER_MotionSensor;

/* Absolutely value*/
uint16_t USER_ABS16b(signed short p_Input_s16){
	/* If input is +, return itself */
	if (p_Input_s16 >= POSITIVE){
		return p_Input_s16;
	}
	/* If input is -, return -input */
	else if (p_Input_s16 <= NEGATIVE){
		return (p_Input_s16*NEGATIVE);
	}
	return 0;
}

/* Delay_ms Function */
void USER_DelayMs(int p_Time_ms){
	USER_Interrupt.msTick_u32 = 0; 
	while (USER_Interrupt.msTick_u32 < p_Time_ms){}
}

/* Get state button */
void USER_GetStateButton(void){
	if (USER_Interrupt.timeButton_u16 == TIME_BUTTON_3000MS){
		if (USER_Interrupt.pressButton_u8 != 0){
			USER_DelayMs(INTERVAL_PRESS_BUTTON);	//max interval of 2 times press button
			if (USER_Interrupt.pressButton_u8 == STATE_BUTTON(START_STOP)){
				TOGGLE_BIT(VariableMode.StartStop_bit, BIT_0);	// On/Off bit
			}
			else if (USER_Interrupt.pressButton_u8 == STATE_BUTTON(SKIP)){
				ON_BIT(VariableMode.Skip_bit, BIT_0);				// On bit
			}
			else if (USER_Interrupt.pressButton_u8 == STATE_BUTTON(PAUSE)){
				TOGGLE_BIT(VariableMode.Pause_bit, BIT_0);			// On/Off bit
			}
			USER_Interrupt.pressButton_u8 = 0;
			USER_Interrupt.stateButton_bit = 0;	//Ensure not Error Signal
		}
	}
}


/* process motion sensor */
void USER_GetMotionSensor(void){
	signed short f_DeltaX_s16 = 0;
	signed short f_DeltaY_s16 = 0;
	signed short f_DeltaZ_s16 = 0;
	
	if (USER_Interrupt.timeMotionSensor_u16 == 0){
		/* Read data from Motion sensor */
		I3G4250D_Read(&I3G4250D_Data);

		/* sensitivity of per Axis is not the same -> normalized to the same sensitivity base */
		f_DeltaX_s16 = (I3G4250D_Data.X)*HS_AXIS_X;	
		f_DeltaY_s16 = (I3G4250D_Data.Y)*HS_AXIS_Y;	
		f_DeltaZ_s16 = (I3G4250D_Data.Z + 15)*HS_AXIS_Z;	
	
		/* 
		the minimum change of one of 3 AXIS must pass HS_AXIS_COMPARE -> anti-jamming
		Get only the value of the largest change in the 3 AXIS 													
		*/
		if ((USER_ABS16b(f_DeltaX_s16) > HS_AXIS_COMPARE) ||
				(USER_ABS16b(f_DeltaY_s16) > HS_AXIS_COMPARE) || 
				(USER_ABS16b(f_DeltaZ_s16) > HS_AXIS_COMPARE)){
			/* if X-AXIS is largest change */
			if ((USER_ABS16b(f_DeltaX_s16) >= USER_ABS16b(f_DeltaY_s16)) && 
					(USER_ABS16b(f_DeltaX_s16) >= USER_ABS16b(f_DeltaZ_s16))){
				if (f_DeltaX_s16 >= POSITIVE){
					ON_BIT(USER_MotionSensor.ChangeValueX_2bit, BIT_0); // = 01 (+)
					OFF_BIT(USER_MotionSensor.ChangeValueX_2bit, BIT_1);
				}
				else{
					OFF_BIT(USER_MotionSensor.ChangeValueX_2bit, BIT_0); // = 10 (-)
					ON_BIT(USER_MotionSensor.ChangeValueX_2bit, BIT_1);
				}
			}
			/* if Y-AXIS is largest change */
			else if ((USER_ABS16b(f_DeltaY_s16) >= USER_ABS16b(f_DeltaX_s16)) && 
							(USER_ABS16b(f_DeltaY_s16) >= USER_ABS16b(f_DeltaZ_s16))){
				if (f_DeltaY_s16 >= POSITIVE){
					ON_BIT(USER_MotionSensor.ChangeValueY_2bit, BIT_0); // = 01 (+)
					OFF_BIT(USER_MotionSensor.ChangeValueY_2bit, BIT_1);
				}
				else{
					OFF_BIT(USER_MotionSensor.ChangeValueY_2bit, BIT_0); // = 10 (-)
					ON_BIT(USER_MotionSensor.ChangeValueY_2bit, BIT_1);
				}
			}
			/* if Z-AXIS is largest change */
			else if ((USER_ABS16b(f_DeltaZ_s16) >= USER_ABS16b(f_DeltaX_s16)) && 
							(USER_ABS16b(f_DeltaZ_s16) >= USER_ABS16b(f_DeltaY_s16))){
				if (f_DeltaZ_s16 >= POSITIVE){
					ON_BIT(USER_MotionSensor.ChangeValueZ_2bit, BIT_0); // = 01 (+)
					OFF_BIT(USER_MotionSensor.ChangeValueZ_2bit, BIT_1);
				}
				else{
					OFF_BIT(USER_MotionSensor.ChangeValueZ_2bit, BIT_0); // = 10 (-)
					ON_BIT(USER_MotionSensor.ChangeValueZ_2bit, BIT_1);
				}
			}
			USER_Interrupt.timeMotionSensor_u16 = INTERVAL_GET_COMMAND_SENSOR;	// the interval of 2 times get command is 2s
		}
	}
}

/* Sound of button */
void USER_SoundButton(void){
	/*  Speak “Peak” when button is pressed */
	if (USER_Interrupt.timeButton_u16 < TIME_BUTTON_3000MS){	//press button
		/* "Peak" */
		for(int i = 0; i < sizeof(beep_sound); i++){
			OutputAudioSample(beep_sound[i]);
		}
	}
}

/* Error sound */
void USER_SoundError(void){
	/* (USB is not plug) && (button is pressed)  ->  the audio speak “the USB doesn’t plug” */
	if ((VariableMode.RejectUSB_bit != 0) && (USER_Interrupt.timeButton_u16 < TIME_BUTTON_3000MS)){
		for(int i = 0; i < sizeof(error_sound); i++){
			OutputAudioSample(error_sound[i]);
		}
	}
}
