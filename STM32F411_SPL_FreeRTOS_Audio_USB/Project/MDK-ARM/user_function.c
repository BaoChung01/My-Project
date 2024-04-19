#include "user_function.h"

MotionSensor_t USER_MotionSensor;

/* Absolutely value*/
uint16_t USER_ABS16b(signed short p_Input_s16){
	/* If input is +, return itself */
	if (p_Input_s16 >= 0){
		return p_Input_s16;
	}
	/* If input is -, return -input */
	else {
		return (p_Input_s16*(-1));
	}
}

/* Delay_ms Function */
void USER_DelayMs(int p_Time_ms){
	msTick = 0; 
	while (msTick < p_Time_ms){}
}

/* Get state button */
void USER_GetStateButton(void){
	if (timeButton == 3000){
		if (pressButton != 0){
			USER_DelayMs(700);	//max interval of 2 times press button
			if (pressButton == 1){
				VariableMode.StartStop_bit ^= 1;	// On/Off bit
			}
			else if (pressButton == 2){
				VariableMode.Skip_bit |= 1;				// On bit
			}
			else if (pressButton == 3){
				VariableMode.Pause_bit ^= 1;			// On/Off bit
			}
			pressButton = 0;
			stateButton = 0;	//Ensure not Error Signal
		}
	}
}


/* process motion sensor */
void USER_GetMotionSensor(void){
	signed short f_DeltaX_s16 = 0;
	signed short f_DeltaY_s16 = 0;
	signed short f_DeltaZ_s16 = 0;
	
	if (timeMotionSensor_u16 == 0){
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
				if (f_DeltaX_s16 >= 0){
					USER_MotionSensor.ChangeValueX_2bit |= 1u;		// = 01 (+)
				}
				else{
					USER_MotionSensor.ChangeValueX_2bit |= 2u;		// = 10 (-)
				}
			}
			/* if Y-AXIS is largest change */
			else if ((USER_ABS16b(f_DeltaY_s16) >= USER_ABS16b(f_DeltaX_s16)) && 
							(USER_ABS16b(f_DeltaY_s16) >= USER_ABS16b(f_DeltaZ_s16))){
				if (f_DeltaY_s16 >= 0){
					USER_MotionSensor.ChangeValueY_2bit |= 1u;		// = 01 (+)
				}
				else{
					USER_MotionSensor.ChangeValueY_2bit |= 2u;		// = 10 (-)
				}
			}
			/* if Z-AXIS is largest change */
			else if ((USER_ABS16b(f_DeltaZ_s16) >= USER_ABS16b(f_DeltaX_s16)) && 
							(USER_ABS16b(f_DeltaZ_s16) >= USER_ABS16b(f_DeltaY_s16))){
				if (f_DeltaZ_s16 >= 0){
					USER_MotionSensor.ChangeValueZ_2bit |= 1u;		// = 01 (+)
				}
				else{
					USER_MotionSensor.ChangeValueZ_2bit |= 2u;		// = 10 (-)
				}
			}
			timeMotionSensor_u16 = 2000;	// the interval of 2 times get command is 2s
		}
	}
}

/* Sound of button */
void USER_SoundButton(void){
	/*  Speak “Peak” when button is pressed */
	if (timeButton < 3000){	//press button
		/* "Peak" */
		for(i = 0; i < sizeof(beep_sound); i++){	//Length of "Beep" sound is 512 byte 
			OutputAudioSample(beep_sound[i]);
			if ((timeButton == 3000) && (i > 200)){	//i>200 ensure to generate "Beep" sound
				break;
			}
		}
	}
}

/* Error sound */
void USER_SoundError(void){
	/* (USB is not plug) && (button is pressed)  ->  the audio speak “the USB doesn’t plug” */
	if ((VariableMode.RejectUSB_bit != 0) && (timeButton < 3000)){
		for(i = 0; i < sizeof(error_sound); i++){
			OutputAudioSample(error_sound[i]);
		}
	}
}
