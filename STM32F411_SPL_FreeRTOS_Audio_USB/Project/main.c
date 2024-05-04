#include "main.h"

#define USART_BAUD_115200 115200

#define PRIORITY_MIN    0
#define PRIORITY_LOW    1
#define PRIORITY_MEDIUM 2
#define PRIORITY_HIGH   3
#define PRIORITY_MAX    4

USB_OTG_CORE_HANDLE		USB_OTG_Core;
USBH_HOST				      USB_Host;
RCC_ClocksTypeDef		  RCC_Clocks;
volatile int			    enum_done = 0;
uint8_t dataSend[30] = "Start state";

TaskHandle_t TASK_Init = NULL;
TaskHandle_t TASK_Led = NULL;
TaskHandle_t TASK_Sensor = NULL;
TaskHandle_t TASK_USB = NULL;
TaskHandle_t TASK_Button = NULL;
TaskHandle_t TASK_Audio = NULL;
TaskHandle_t TASK_Alarm = NULL;

//--------------------------------------------------------------------------------------------------------

int main(){
	/* Init LED */
	Init_LED();
	
	/* Init USART */
	//USART6_init(USART_BAUD_115200);
	
	/* Init TIM2 to use Delay function */
	Init_timerDelay();
	
	/* Button Init */
	Button_init();
	USER_Interrupt.timeButton_u16 = 3000;
	
	/* Init I3G42450D (Motion sensor)*/
	I3G4250D_Init();
	
	/*  USB Init  */
	//USBH_Init(&USB_OTG_Core, USB_OTG_FS_CORE_ID, &USB_Host, &USBH_MSC_cb, &USR_Callbacks);
	
	/* Task of Audio Project */
	xTaskCreate(Task0_Init, "TASK_Init", configMINIMAL_STACK_SIZE, NULL, 4, &TASK_Init);
	xTaskCreate(Task1_Led, "TASK_Led", configMINIMAL_STACK_SIZE, NULL, 2, &TASK_Led);
  xTaskCreate(Task2_SenSor, "TASK_Sensor", configMINIMAL_STACK_SIZE, NULL, 3, &TASK_Sensor);
	xTaskCreate(Task3_USB, "TASK_USB", configMINIMAL_STACK_SIZE, NULL, 2, &TASK_USB);
	xTaskCreate(Task4_Button, "TASK_Button", configMINIMAL_STACK_SIZE, NULL, 3, &TASK_Button);
	xTaskCreate(Task5_Audio, "TASK_Audio", configMINIMAL_STACK_SIZE, NULL, 2, &TASK_Audio);
	xTaskCreate(Task6_Alarm, "TASK_Alarm", configMINIMAL_STACK_SIZE, NULL, 3, &TASK_Alarm);
	
	/*start FREERTOS scheduler*/
  vTaskStartScheduler();
	
	while(1){}	
}

static void Task0_Init(void *pvParameters){
	/* Initial Audio */
	InitializeAudio(Audio44100HzSettings);
	SetAudioVolume(soundVolume);
	
	/* Link led during 2s */
	while (VariableMode.IntialCount_u8 < INITIAL_2000MS){
		GPIO_SetBits(GPIOD, GPIO_Pin_15);
		vTaskDelay(DELAY_200MS);
		GPIO_ResetBits(GPIOD, GPIO_Pin_15);
		vTaskDelay(DELAY_200MS);
		VariableMode.IntialCount_u8++;
	}
	
	/* Send ""send massenge to PC “Start state” (UART) */
	//USART6_WriteData((uint16_t*)dataSend);
	
	/* End Task 0 */
	vTaskDelete(TASK_Init);
}

/* 
Task1_Led: Blink Led to display status system
- Don't Blink: System is turn off
- Blink every 500ms: System is being turn on and run
- Blink every 1000ms: System is being turn on, but it is not running 
*/
static void Task1_Led(void *pvParameters){
	while(1){
		if ((VariableMode.StartStop_bit == HIGH) && (VariableMode.Pause_bit == LOW)){        //Playing Song
			GPIO_SetBits(GPIOD, GPIO_Pin_15);
			vTaskDelay(DELAY_500MS);
			GPIO_ResetBits(GPIOD, GPIO_Pin_15);
			vTaskDelay(DELAY_500MS);
		}
		else if ((VariableMode.StartStop_bit == HIGH) && (VariableMode.Pause_bit == HIGH)){   //No Play any Song
			GPIO_SetBits(GPIOD, GPIO_Pin_15);
			vTaskDelay(DELAY_1000MS);
			GPIO_ResetBits(GPIOD, GPIO_Pin_15);
			vTaskDelay(DELAY_1000MS);
		}
		vTaskDelay(1);
	}
}

/* 
Task Sensor: Get and process data from Motion Sensor 
*/
static void Task2_SenSor(void *pvParameters){
	while(1){
		USER_GetMotionSensor();
		vTaskDelay(50);
	}
}

/* 
Task USB

*/
void Task3_USB(void *pvParameters){
	while(1){
		/* Get button state and process data from usb */
		if (VariableMode.StartStop_bit != LOW){
			/*  */
			if (VariableMode.Pause_bit == LOW){
				/* Read data from USB */
			}
		}
		if (VariableMode.Skip_bit != LOW){
			/* Read data from USB a interval skip */
			
			/* Performed requirement and then Reset state of the skip command bit */
			OFF_BIT(VariableMode.Skip_bit, BIT_0);
		}
		
		/* Get value from motion sensor to process requirements */
		if ((USER_MotionSensor.ChangeValueX_2bit != 0) ||
				(USER_MotionSensor.ChangeValueY_2bit != 0) ||
				(USER_MotionSensor.ChangeValueZ_2bit != 0)){
					
			/* process requirement */
					
			/* Reset requirement */
			CLEAR_2BIT(USER_MotionSensor.ChangeValueX_2bit);
			CLEAR_2BIT(USER_MotionSensor.ChangeValueY_2bit);
			CLEAR_2BIT(USER_MotionSensor.ChangeValueZ_2bit);
		}
			
		vTaskDelay(10);
		//USBH_Process(&USB_OTG_Core, &USB_Host);
	}
}

/* 
Task Button: Get State of Button
*/
void Task4_Button(void *pvParameters){
	while(1){
		USER_GetStateButton();
		vTaskDelay(10);
	}
}

/* Task Audio */
void Task5_Audio(void *pvParameters){
	while(1){
		if (VariableMode.StartStop_bit != LOW){
			if (VariableMode.Pause_bit == LOW){
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
		//USER_SoundError();
	
		vTaskDelay(1);
	}
}


