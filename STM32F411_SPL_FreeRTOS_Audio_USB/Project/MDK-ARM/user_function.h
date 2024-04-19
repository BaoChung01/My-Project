#ifndef USER_FUNCTION
#define USER_FUNCTION

#include "user_interrupt.h"
#include "DelayLED.h"
#include "ModuleI3G4250D.h"
#include "Audio.h"

/* sensitivity of per Axis is not the same */
#define HS_AXIS_X 1
#define HS_AXIS_Y 3
#define HS_AXIS_Z 1
#define HS_AXIS_COMPARE 15
typedef struct{
	uint8_t ChangeValueX_2bit:2;		// 0: No Change  --  1: Change +  -- 2: Change -
	uint8_t ChangeValueY_2bit:2;		// 0: No Change  --  1: Change +  -- 2: Change -
	uint8_t ChangeValueZ_2bit:2;		// 0: No Change  --  1: Change +  -- 2: Change -
}MotionSensor_t;
extern MotionSensor_t USER_MotionSensor;

/* Function */
void USER_DelayMs(int p_Time_ms);	
void USER_GetStateButton(void);
void USER_GetMotionSensor(void);
void USER_SoundButton(void);
void USER_SoundError(void);

#endif
