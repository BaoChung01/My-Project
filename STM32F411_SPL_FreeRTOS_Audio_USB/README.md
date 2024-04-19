# Audio Project on STM32F411E discovery board

Use the STM32F411E discovery board to play music from the USB

## Table of Contents

1. [About the Project](#about-the-project)
1. [User story](#User-story)
1. [Role](#Role)
1. [Getting Started](#getting-started)
    1. [Building](#building)
1. [Versioning](#versioning)
1. [Further Reading](#further-reading)

## About the Project

1. Module hardware used in the project				
	- UART			-> print the log while running			
	- BUTTON		-> play/stop the music, skip/pause the song			
	- LED			-> shows the status of the playing the song. status program			
	- AUDIO			-> Output the audio			
	- USB OTG		-> Read the song			
	- Motion sensor	-> Change the volume			
2. Use the IDE KEIL C and the peripheral standard library				
3. The project uses the RTOS to implement

**[Back to top](#table-of-contents)**

## User story
1. At the startup 2s:				
	- Blink all the LEDs every 0.2s				
	- Send the message to PC "Startup state"				
	- Read the data from the motion sensor				
	- Read the data from USB				
	- Send the initial sound to the audio				
2. Running state:				
	- Press the Button one time for start/stop				
	- Press the button twice to skip the song				
	- Press the button 3 times to pause the song				
	- Hold the button for 3s to reject the USB				
3. The motion sensor for increase/decrease the volume				
	- the Y axis is for inceasing/decreasing the volume				
	- The X axis (shaking) for next/previous song				
	- The Z axis is for running the song faster/normal				
4. The log file will be written into the USB as a file log				
5. The LED is blink while running				
	- 1s if doesn't play any song				
	- 0.5s if playing the song				
6. Audio				
	- if the USB doesn't plugin, and the button pressed, the audio "the USB doesn't plug" is speak				
	- Output the audio from USB				
	- Speak the sound Beep when the button press

**[Back to top](#table-of-contents)**

## User story

LyNC: 

	- Setup GPIO LED, Button

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
	- Responsible for coding following requirements based on the module which was Setup by colleagues

**[Back to top](#table-of-contents)**

## Getting Started

1. ST-Link v2 Programmer
2. Keil C v5
3. KIT STM32F411E DISCOVERY
4. USB-host cable
5. Loudspeaker

Clone the repo as follows:
git clone https://github.com/BaoChung01/My-Project.git

Use Keil C v5 -> build-> flash to board

### Getting the Source


This project is [hosted on GitHub](https://github.com/BaoChung01/My-Project.git). You can clone this project directly using this command:
```
git clone https://github.com/BaoChung01/My-Project.git
```

### Building

Use Keil C v5 -> build-> flash to board

**[Back to top](#table-of-contents)**


## Versioning

This project uses [Original Versioning](https://vedder.se/2012/12/stm32f4-discovery-usb-host-and-mp3-player/). For a list of available versions, see the [repository tag list](https://github.com/vanbwodonk/STM32F4_USB_MP3).


## Further Reading

https://www.st.com/resource/en/application_note/an3997-audio-playback-and-recording-using-the-stm32f4discovery-stmicroelectronics.pdf

**[Back to top](#table-of-contents)**




