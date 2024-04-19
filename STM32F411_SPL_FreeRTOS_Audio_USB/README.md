# Audio Project on STM32F411E discovery board

Use STM32F411E discovery board to play music from USB

## Table of Contents

1. [About the Project](#about-the-project)
1. [User story](#User-story)
1. [Role](#Role)
1. [Getting Started](#getting-started)
    1. [Building](#building)
1. [Versioning](#versioning)
1. [Further Reading](#further-reading)

## About the Project

1. Module hardware use in the project				
	- UART			-> print the log while running			
	- BUTTON		-> play/stop the music, skip/pause the song			
	- LED			-> show the status of playing song. status program			
	- AUDIO			-> output the audio			
	- USB OTG		-> read the song			
	- Motion sensor	-> change the volume			
2. Use the IDE KEIL C and the periperal standard library				
3. The project use the RTOS to implement

**[Back to top](#table-of-contents)**

## User story
1. At the startup 2s:				
	- Blink all the LED every 0.2s				
	- Send the message to PC "Startup state"				
	- Read the data from motion sensor				
	- Read the data from USB				
	- Send the initial sound to audio				
2. Running state:				
	- Press the Button one time for start/stop				
	- Press the button twice to skip the song				
	- Press the button 3 times to pause the song				
	- Hold the button for 3s to reject the USB				
3. The motion sensor for increase/decrease the volume				
	- the Y axis is for incease/decrease the volume				
	- The X axis (shaking) for next/previous song				
	- The Z axis is for running the song faster/normal				
4. The log file will be written into the USB as file log				
5. The LED is blink while running				
	- 1s if doesnt play any song				
	- 0.5s if playing the song				
6. Audio				
	- if the USB doesnt plug in, and button press, the audio "the USB doesnt plug" is speak				
	- Output the audio from USB				
	- Speak the sound Beep when button press

**[Back to top](#table-of-contents)**

## User story

LyNC: 

	- Setup GPIO Led, Button

	- Setup SPI Motion Sensor

ChungVB:

	- Create STM32F411 Project on Keil C

	- Setup Standard Library for STM32F411E

	- Setup Audio Module

	- Setup FreeRTOS

TaiNT:

	- Setup USB (Not finish)

TuanLDM:

	- Setup USART
	- Responsible for coding following requirement based on module which was Setup by colleagues

**[Back to top](#table-of-contents)**

## Getting Started

1. ST-Link v2 Programmer
2. Keil C v5
3. KIT STM32F411E DISCOVERY
4. USB-host cable
5. Loudspeaker

Clone the repo as follows:
git clone https://github.com/BaoChung/My-Project.git

Use Keil C v5 -> build-> flash to board

### Getting the Source


This project is [hosted on GitHub](https://github.com/BaoChung/My-Project). You can clone this project directly using this command:
```
git clone https://github.com/BaoChung/My-Project.git
```

### Building

Use Keil C v5 -> build-> flash to board

**[Back to top](#table-of-contents)**


## Versioning

This project uses [Original Versioning](https://vedder.se/2012/12/stm32f4-discovery-usb-host-and-mp3-player/). For a list of available versions, see the [repository tag list](https://github.com/vanbwodonk/STM32F4_USB_MP3).


## Further Reading

https://www.st.com/resource/en/application_note/an3997-audio-playback-and-recording-using-the-stm32f4discovery-stmicroelectronics.pdf

**[Back to top](#table-of-contents)**




