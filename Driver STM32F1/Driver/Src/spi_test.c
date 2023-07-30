#include "stm32f1.h"
#include "stm32_spi.h"
#include "stm32_gpio.h"
#include "string.h"
/*
 *PB14 -> SPI2_MISO
 *PB15 -> SPI2_MOSI
 *PB13 -> SPI2_SCLK
 *PB12 -> SPI2_NSS
*ALT function mode :5 
*/
void SPI2_GPIOInits(void)
{
  GPIO_Handle_t SPIPins;
  SPIPins.GPIOx=GPIOB;
  SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode=5;
  SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_NO_PUPD;
  SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl= GPIO_NO_PUPD;
  SPIPins.GPIO_PinConfig.GPIO_PinSpeed= GPIO_Speed_50MHz;
  
  //SCLK
  SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
  GPIO_Init(&SPIPins);
  
  //MOSI
  SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
  GPIO_Init(&SPIPins);
  
  //MISO
  SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
  GPIO_Init(&SPIPins);
  
  //NSS
  SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
  GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{
  SPI_Handle_t SPI2handle;
  SPI2handle.pSPIx =SPI2;
  SPI2handle.SPIConfig.SPI_BusConfig=SPI_BUS_CONFIG_FD;
  SPI2handle.SPIConfig.SPI_DeviceMode=SPI_DEVICE_MODE_MASTER;
  SPI2handle.SPIConfig.SPI_SclkSpeed= SPI_SCLK_SPEED_DIV2;
  SPI2handle.SPIConfig.SPI_DFF=SPI_DFF_8BITS;
  SPI2handle.SPIConfig.SPI_CPOL=SPI_CPOL_HIGH;
  SPI2handle.SPIConfig.SPI_CPHA=SPI_CPHA_LOW;
  SPI2handle.SPIConfig.SPU_SSM=SPI_SSM_EN;
  // sortware slave management enable for NSS pin
  SPI_Init(&SPI2handle);
}
int main()
{
  char user_data[]= "Hello World";
  SPI2_GPIOInits();
  SPI2_Inits();
  SPI_SSIConfig(SPI2, ENABLE);
  SPI_PeriClockControl(SPI2, ENABLE);
  
  SPI_SendData(SPI2,(uint8_t *)user_data, strlen(user_data));
  while (SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));
         SPI_PeriClockControl(SPI2, DISABLE);
  while(1);
  
}

