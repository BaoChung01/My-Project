/*
** ###################################################################
**     Processor:           STM32F411E DISCOVERY
**     Compiler:            Keil ARM C/C++ Compiler
**     Version:             rev. 1.0, 06/03/2024 - 19:58:27
**
**     Abstract:
**         Build main.h for Stm32f411e Discovery
**
** ###################################################################
*/
#ifndef _MAIN_H_
#define _MAIN_H_

#include "ModuleLED.h"
#include "DelayLED.h"
#include "ButtonControl.h"
#include "ModuleI3G4250D.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "stm32f4xx_it.h"
#include "usb_hcd_int.h"
#include "usbh_usr.h"
#include "usbh_core.h"
#include "usbh_msc_core.h"
#include "mp3.h"
#include "core_cm4.h"
#include "stm32f4xx_conf.h"
#include "USB_USART.h"
#include "user_function.h"
#include "user_interrupt.h"
#include "USB_Requirement.h"
#include "usb_hcd_int.h"
#include "usbh_usr.h"
#include "usbh_core.h"
#include "usbh_msc_core.h"
#include <string.h>
#include "common.h"


#define DELAY_200MS 200
#define DELAY_1000MS 1000
#define DELAY_500MS 500
#define HAVE_A_SONG 1
#define NO_ANY_SONG 0

void LED_blink_200ms(void);
void LED_blink_500ms(void);
void LED_blink_1000ms(void);

/* Task */
static void Task0_Init(void *pvParameters);	/* Task Init */
static void Task1_Led(void *pvParameters);	/* Task Led */
static void Task2_SenSor(void *pvParameters);	/* Task SenSor */
static void Task3_USB(void *pvParameters);	/* Task USB */
static void Task4_Button(void *pvParameters); /* Task Button */
static void Task5_Audio(void *pvParameters);	/* Task Audio */
static void Task6_Alarm(void *pvParameters);	/* Task Alarm */

#endif
