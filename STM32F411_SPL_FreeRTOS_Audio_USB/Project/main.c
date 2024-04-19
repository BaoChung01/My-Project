#include "main.h"



USB_OTG_CORE_HANDLE		USB_OTG_Core;
USBH_HOST				      USB_Host;
RCC_ClocksTypeDef		  RCC_Clocks;
volatile int			    enum_done = 0;

TaskHandle_t TASK_0 = NULL;
TaskHandle_t TASK_1 = NULL;
TaskHandle_t TASK_2 = NULL;
TaskHandle_t TASK_3 = NULL;
TaskHandle_t TASK_4 = NULL;
TaskHandle_t TASK_5 = NULL;
TaskHandle_t TASK_6 = NULL;

//--------------------------------------------------------------------------------------------------------

int main(){
	/* Init LED */
	Init_LED();
	
	/* Init USART */
	Init_USART();
	
	/* Init TIM2 to use Delay function */
	Init_timerDelay();
	
	/* Button Init */
	Button_init();
	
	/* Init I3G42450D (Motion sensor)*/
	I3G4250D_Init();
	
	/*  USB Init  */
	USBH_Init(&USB_OTG_Core, USB_OTG_FS_CORE_ID, &USB_Host, &USBH_MSC_cb, &USR_Callbacks);
	
	/* Task of Audio Project */
	xTaskCreate(Task0_Init, "TASK_0", configMINIMAL_STACK_SIZE, NULL, 4, NULL);
	xTaskCreate(Task1_Led, "TASK_1", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
  xTaskCreate(Task2_SenSor, "TASK_2", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
	xTaskCreate(Task3_USB, "TASK_3", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	xTaskCreate(Task4_Button, "TASK_4", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
	xTaskCreate(Task5_Audio, "TASK_5", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	xTaskCreate(Task6_Alarm, "TASK_6", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
	
	/*start FREERTOS scheduler*/
  vTaskStartScheduler();
	
	while(1){}	
}

static void Task0_Init(void *pvParameters){
	/* Initial Audio */
	InitializeAudio(Audio44100HzSettings);
	SetAudioVolume(soundVolume);
	
	/* Link led during 2s */
	while (VariableMode.IntialCount_u8 < INITIAL_COUNT){
		GPIO_SetBits(GPIOD, GPIO_Pin_15);
		vTaskDelay(DELAY_200MS);
		GPIO_ResetBits(GPIOD, GPIO_Pin_15);
		vTaskDelay(DELAY_200MS);
		VariableMode.IntialCount_u8++;
	}
	
	/* Send ""send massenge to PC “Start state” (UART) */
	USART_sendDataString(USART_StringDataSend_aa);
	
	/* End Task 0 */
	vTaskDelete(TASK_0);
}

/* Task1_Led */
static void Task1_Led(void *pvParameters){
	while(1){
		if ((VariableMode.StartStop_bit == 1) && (VariableMode.Pause_bit == 0)){	//Playing Song
			GPIO_SetBits(GPIOD, GPIO_Pin_15);
			vTaskDelay(DELAY_500MS);
			GPIO_ResetBits(GPIOD, GPIO_Pin_15);
			vTaskDelay(DELAY_500MS);
		}
		else if ((VariableMode.StartStop_bit == 1) && (VariableMode.Pause_bit == 1)){	//No Play any Song
			GPIO_SetBits(GPIOD, GPIO_Pin_15);
			vTaskDelay(DELAY_1000MS);
			GPIO_ResetBits(GPIOD, GPIO_Pin_15);
			vTaskDelay(DELAY_1000MS);
		}
	}
}

/* Task Sensor */
static void Task2_SenSor(void *pvParameters){
	while(1){
		USER_GetMotionSensor();
		vTaskDelay(50);
	}
}

/* Task USB */
void Task3_USB(void *pvParameters){
	while(1){
		/* Get button state and process data from usb */
		if (VariableMode.StartStop_bit != 0){
			/*  */
			if (VariableMode.Pause_bit == 0){
				/* Read data from USB */
			}
		}
		if (VariableMode.Skip_bit != 0){
			/* Read data from USB a interval skip */
			
			/* Performed requirement and then Reset state of the skip command bit */
			VariableMode.Skip_bit &= 0; 
		}
		
		/* Get value from motion sensor to process requirements */
		if ((USER_MotionSensor.ChangeValueX_2bit != 0) ||
				(USER_MotionSensor.ChangeValueY_2bit != 0) ||
				(USER_MotionSensor.ChangeValueZ_2bit != 0)){
					
			/* process requirement */
					
			/* Reset requirement */
			USER_MotionSensor.ChangeValueX_2bit &= ~3;
			USER_MotionSensor.ChangeValueY_2bit &= ~3;
			USER_MotionSensor.ChangeValueZ_2bit &= ~3;
		}
			
		vTaskDelay(10);
	}
	USBH_Process(&USB_OTG_Core, &USB_Host);
}

/* Task Button */
void Task4_Button(void *pvParameters){
	while(1){
		USER_GetStateButton();
		vTaskDelay(10);
	}
}

/* Task Audio */
void Task5_Audio(void *pvParameters){
	while(1){
		if (VariableMode.StartStop_bit != 0){
			if (VariableMode.Pause_bit == 0){
				/* Run Music */
				
			}
		}
		vTaskDelay(1);
	}
}

/* Task Alarm */
void Task6_Alarm(void *pvParameters){
	while(1){
		/*  Speak “Peak” when button is pressed */
		USER_SoundButton();
		
		/* (USB is not plug) && (button is pressed)  ->  the audio speak “the USB doesn’t plug” */
		USER_SoundError();
	
		vTaskDelay(1);
	}
}


