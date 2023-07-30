#include <stm32f1.h>
#include <stm32_gpio.h>
#include <stdint.h>
void delay()
{
  for(uint32_t i=0;i<500000/2;i++);
}
int a;
void main()
{ 
  
  GPIO_Handle_t GpioLed, GpioBtn;
  
  GpioLed.GPIOx=GPIOC;
  GpioLed.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_13;
  GpioLed.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_OUT;
  GpioLed.GPIO_PinConfig.GPIO_PinSpeed= GPIO_Speed_2MHz;
  GpioLed.GPIO_PinConfig.GPIO_PinOPType=  GPIO_PIN_PU;
  GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU; 
  GPIO_Init(&GpioLed);
  GPIO_WriteToOutputPin(GPIOC,GPIO_PIN_NO_13,GPIO_PIN_RESET);
  
  GpioBtn.GPIOx=GPIOB;
  GpioBtn.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_11;
  GpioBtn.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_IT_FL;
  GpioBtn.GPIO_PinConfig.GPIO_PinSpeed= GPIO_Speed_50MHz;
  GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU; 
  GPIO_Init(&GpioBtn);
  
  GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10,3);
  GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10,ENABLE);
  while(1)
  {
GPIO_ToggleOutputPin(GPIOC,GPIO_PIN_NO_13);
 delay();
  }
 
}

void EXTI15_10_IRQHander()
{
    delay();
    GPIO_IRQHandling(GPIO_PIN_NO_11);
    GPIO_ToggleOutputPin(GPIOC,GPIO_PIN_NO_13);
    a++;
}
