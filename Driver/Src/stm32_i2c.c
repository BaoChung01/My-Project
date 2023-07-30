#include "stm32_i2c.h"
#include "stm32_rcc.h"
#include "stm32f1.h"


static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
  pI2Cx->CR1 |= (1<<I2C_CR1_START);
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr)
{  
  SlaveAddr = SlaveAddr <<1;
  SlaveAddr &= ~(1); // slave addr is slave address + r/nw bit =0;
  pI2Cx->DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
  uint32_t dummy_read;
  //check for device mode
  if (pI2CHandle->pI2Cx->SR2 & (1<<I2C_SR2_MSL))
  {
    //device is in master mode
   if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
   {
    if (pI2CHandle->RxSize ==1 )
    {
      //first disable the ack
      I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
      //clear the ADDR Flag (read SR1, read SR2)
      dummy_read = pI2CHandle->pI2Cx->SR1;
      dummy_read = pI2CHandle->pI2Cx->SR2;
      (void) dummy_read;
    }
   }
    else
    {
    //clear the ADDR Flag (read SR1, read SR2)
    dummy_read = pI2CHandle->pI2Cx->SR1;
    dummy_read = pI2CHandle->pI2Cx->SR2;
    (void)dummy_read;
    }
  }
  
  else
  {
    //device in slave mode
    //clear the ADDR Flag (read SR1, read SR2)
    dummy_read = pI2CHandle->pI2Cx->SR1;
    dummy_read = pI2CHandle->pI2Cx->SR2;
    (void)dummy_read;
  }
}
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
  //implement the code to disable ITBUFEN control bit
  pI2CHandle->pI2Cx->CR2 &= ~(1<<I2C_CR2_ITBUFEN);
  //IMPLEMENT the code to disable ITEVFEN control bit
  pI2CHandle->pI2Cx->CR2 &= ~(1<<I2C_CR2_ITEVTEN);
  
  pI2CHandle->TxRxState = I2C_READY;
  pI2CHandle->pRxBuffer=NULL;
  pI2CHandle->RxLen = 0;
  pI2CHandle->RxSize = 0;
  
  if (pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
  {
    I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
  }
}
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
  //WE have to do the data reception
  if (pI2CHandle->RxSize == 1)
  {
    *pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
    pI2CHandle->RxLen--;
  }
  
  if(pI2CHandle->RxSize >1)
  {
    if(pI2CHandle->RxLen ==2)
    {
      //clear the ack bit
      I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
    }
    //read DR
    *pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
    pI2CHandle->pRxBuffer++;
    pI2CHandle->TxLen--;
  }
  if(pI2CHandle->RxLen == 0)
  {
    //close the I2C data receptiom and notify the application
    //1 generation the stop condition
    if(pI2CHandle->Sr ==  I2C_DISABLE_SR)
    {
      I2C_GeneratesStopCondition(pI2CHandle->pI2Cx);
      //close the I2C rx
      I2C_CloseReceiveData(pI2CHandle);
      //3 notify the application
      I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
    }
  }
}
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
  if(pI2CHandle->TxLen>0)
  {
    //1. load the data in the SR
    pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
    //2. decrement the TxLen
    pI2CHandle->TxLen--;
    //3. increment the buffer address
    pI2CHandle->pTxBuffer++;
  }
}
void I2C_PeripheralControl (I2C_RegDef_t *pI2Cx, uint8_t EnorDI)
{
  if (EnorDI == ENABLE)
  {
    pI2Cx->CR1 |= (1<<I2C_CR1_FE);
  }
  else
  {
    pI2Cx->CR1 &= ~(1<<I2C_CR1_FE);
  }
}
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
  SlaveAddr = SlaveAddr<<1;
  SlaveAddr|= 1;
  pI2Cx->DR = SlaveAddr;
}

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
  if(EnorDi== ENABLE)
  {
      if(pI2Cx == I2C1)
      {
        I2C1_PCLK_EN();
      }
      else if (pI2Cx == I2C2)
      {
        I2C2_PCLK_EN();
      }
  }
  else
  {
    if(pI2Cx == I2C1)
      {
        I2C1_PCLK_DI();
      }
      else if (pI2Cx == I2C2)
      {
        I2C2_PCLK_DI();
      }
  }
}

void I2C_Init(I2C_Handle_t *pI2CHandle)
{
    uint32_t tempreg=0;
    
    I2C_PeripheralControl(pI2CHandle->pI2Cx, ENABLE);
    
    //Ack
    tempreg |= pI2CHandle->I2C_Config.I2C_AckControl <<10;
    pI2CHandle->pI2Cx->CR1 = tempreg; 
    
    //FREQ CR2
    tempreg= 0;
    tempreg|=RCC_GetPCLK1Value()/1000000u;
    pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);
    //OAR1 7 bit or 10 bit
    tempreg=0;
    tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress <<1;
    tempreg |= (1 <<14); // should always be kept at 1 by software
    pI2CHandle->pI2Cx->OAR1 = tempreg;
        //CCR
  tempreg = 0;
  uint16_t ccr_value = 0;

    if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
    {
      //standard mode
      ccr_value = (RCC_GetPCLK1Value()/ (2*pI2CHandle->I2C_Config.I2C_SCLSpeed));
      tempreg |= (ccr_value & 0xFFF);
    }
    else //fastmode
    {
      tempreg |= (1<<15);   
      tempreg |=(pI2CHandle->I2C_Config.I2C_PMDutyCycle << 14);
      if(pI2CHandle->I2C_Config.I2C_PMDutyCycle== I2C_FM_DUTY_2)
      {
        ccr_value = (RCC_GetPCLK1Value() / (3*pI2CHandle->I2C_Config.I2C_SCLSpeed));
      }
      else
      {
        ccr_value = (RCC_GetPCLK1Value() / (25*pI2CHandle->I2C_Config.I2C_SCLSpeed));
      }
      tempreg |= (ccr_value & 0xFFF);
    }
    pI2CHandle->pI2Cx->CCR = tempreg;
    
    //TRISE
    if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
    {
      //standard mode
      tempreg = (RCC_GetPCLK1Value()/1000000U)+1;
    }
    else
    {
      //fast mode
      tempreg = ((RCC_GetPCLK1Value()*300)/1000000000U)+1;
    }
    pI2CHandle->pI2Cx->TRISE = (tempreg&0x3F);
    
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
  if(pI2Cx->SR1 & FlagName)
  {
    return FLAG_SET;
  }
  return FLAG_RESET;
}
void I2C_GeneratesStopCondition(I2C_RegDef_t *pI2Cx)
{
  pI2Cx->CR1 |= (1<<I2C_CR1_STOP);
}
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
  //SRART condition
  I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
  //checking the SB flag in the SR1 SCL=LOW
  while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));
  //send address data and bit readwite total 8 bit
   I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);
  //checking the ADDR flag in the SR1
   while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));
   //clear the add flag SCL=LOW;
   I2C_ClearADDRFlag(pI2CHandle);
   //send the data until the len become 0
   while(Len>0)
   {
    while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));// wait until TXE set
          pI2CHandle->pI2Cx->DR = *pTxBuffer;
          pTxBuffer++;
          Len--;
   }
   
   //when Len become zero wait for TXE-1 and BTF=1 before gene the stop conditon
     while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
     while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));
  
  // generation stop conditon
  // note generation stop auto clear the BTF
     if(Sr == I2C_DISABLE_SR)
     {
       I2C_GeneratesStopCondition(pI2CHandle->pI2Cx);
     }
}
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
  if (EnOrDi==ENABLE)
  {
    pI2Cx->CR1 |= (1<< I2C_CR1_ACK);
  }
  else
  {
    pI2Cx->CR1 &= ~(1<< I2C_CR1_ACK);
  }
}
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
  //1. Start condition
  I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
  
  //2. confirm that start generation
  while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));
  //3. send the address of the slave with r/w bit set to R(1) total 8 bits
  I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);
  //check the ADDR flag in SR1
  while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));
  //read only 1 byte from slave
  if (Len==1)
  {
    //disable acking
    I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
    //clear the ADDR flag
    I2C_ClearADDRFlag(pI2CHandle);
    //wait until RXNE  1
    while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));
    
    //genenation stop condition
    if(Sr== I2C_DISABLE_SR)
      I2C_GeneratesStopCondition(pI2CHandle->pI2Cx);
    //read data in the buffer
    *pRxBuffer = pI2CHandle->pI2Cx->DR;
  }
  //when len >1
  if (Len>1)
  {
    //clear the ADDR flag
    I2C_ClearADDRFlag(pI2CHandle);
    //read data until Len become 0
    for(uint32_t i=Len;i>0;i--)
    {
      //wait until RXNE becomes 1
      while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));
      if (i==2)//if last 2 bytes
      {
        //disable acking
        I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
        //generation stop condition
        if (Sr == I2C_DISABLE_SR)
          I2C_GeneratesStopCondition(pI2CHandle->pI2Cx);
      }
       //read tha data from data register in to buffer
      *pRxBuffer =  pI2CHandle->pI2Cx->DR;
      pRxBuffer++;
    }
  }
  if (pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
  {
    I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
  }
}
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
  if(EnorDi==ENABLE)
  {
    if(IRQNumber<=31)
    {
      //program ISER0 Register
      *NVIC_ISER0 |= (1<<IRQNumber);
    }
    else if (IRQNumber>31 && IRQNumber<64)
    {   
      //program ISER1 Register
      *NVIC_ISER1 |= (1<<(IRQNumber%32));
    }
      else if (IRQNumber>=64 && IRQNumber<96)
    {
      //program ISER2 Register
      *NVIC_ISER2 |= (1<<(IRQNumber%64));
    }
    
  }
  else
  {
    if(IRQNumber<=31)
    {
      //program ICER0 Register
      *NVIC_ICER0 |= (1<<IRQNumber);
    }
    else if (IRQNumber>31 && IRQNumber<64)
    {   
      //program ICER1 Register
      *NVIC_ICER1 |= (1<<(IRQNumber%32));
    }
      else if (IRQNumber>=64 && IRQNumber<96)
    {
      //program ICER2 Register
      *NVIC_ICER2 |= (1<<(IRQNumber%64));
    }
  }
}
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    uint8_t ipr = IRQNumber/4;
    uint8_t irq = IRQNumber%4;
   *(NVIC_PR_BASE_ADDR + ipr) |= (IRQPriority<<(8*irq +4));
}

  uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
    uint8_t busystate = pI2CHandle->TxRxState;
    if ((busystate != I2C_BUSY_IN_TX) &&(busystate != I2C_BUSY_IN_RX))
    {
      pI2CHandle->pTxBuffer=pTxBuffer;
      pI2CHandle->TxLen =  Len;
      pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
      pI2CHandle->DevAddr = SlaveAddr;
      pI2CHandle->Sr = Sr;
      //start condition
      I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
      //implement the code to enable ITBUFEN control bit
      pI2CHandle->pI2Cx->CR2 |= (1<<I2C_CR2_ITBUFEN);
      //implement the code to enable ITEVTEN control bit
      pI2CHandle->pI2Cx->CR2 |= (1<<I2C_CR2_ITEVTEN);
      //implement the code to enable ITERREN control bit
      pI2CHandle->pI2Cx->CR2 |= (1<<I2C_CR2_ITERREN);
    }
    return busystate;
}
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
  uint8_t busystate = pI2CHandle->TxRxState;
    if ((busystate != I2C_BUSY_IN_TX) &&(busystate != I2C_BUSY_IN_RX))
    {
      pI2CHandle->pRxBuffer=pRxBuffer;
      pI2CHandle->RxLen =  Len;
      pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
      pI2CHandle->DevAddr = SlaveAddr;
      pI2CHandle->Sr = Sr;
      //start condition
      I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
      //implement the code to enable ITBUFEN control bit
      pI2CHandle->pI2Cx->CR2 |= (1<<I2C_CR2_ITBUFEN);
      //implement the code to enable ITEVTEN control bit
      pI2CHandle->pI2Cx->CR2 |= (1<<I2C_CR2_ITEVTEN);
      //implement the code to enable ITERREN control bit
      pI2CHandle->pI2Cx->CR2 |= (1<<I2C_CR2_ITERREN);
    }
    return busystate;
}  
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
  //implement the code to disable ITBUFEN Control bit
  pI2CHandle->pI2Cx->CR2 &= ~(1<<I2C_CR2_ITBUFEN);
  //implement the code to disable ITEVTEN Control bit
  pI2CHandle->pI2Cx->CR2 &= ~(1<<I2C_CR2_ITEVTEN);
  
  pI2CHandle->TxRxState = I2C_READY;
  pI2CHandle->pTxBuffer= NULL;
  pI2CHandle->TxLen = 0;
}
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
  //interrupt handling for both master and slave mode of a device
  uint32_t temp1, temp2, temp3;
  temp1 = pI2CHandle->pI2Cx->CR2 &(1<<I2C_CR2_ITEVTEN);
  temp2 = pI2CHandle->pI2Cx->CR2 &(1<<I2C_CR2_ITBUFEN);
  
  temp3 = pI2CHandle->pI2Cx->SR1 &(1<<I2C_SR1_SB);
  
  //SB flag
  if(temp1 && temp3)
  {
    if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
    {
      I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
    }
    else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
    {
      I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
    }
  }
  temp3 = pI2CHandle->pI2Cx->SR1 &(1<<I2C_SR1_ADDR);
  //ADDR event
  //when master mode address is sent
  // when slave mode dia chi khop voi dia chi rieng
  if (temp1 && temp3)
  {
    //interrupt is generation because of ADDR event
    I2C_ClearADDRFlag(pI2CHandle);
  }
  temp3 = pI2CHandle->pI2Cx->SR1 &(1<<I2C_SR1_BTF);
  //BTF(Byte Transfer Finished
  if (temp1 && temp3)
  {
    //BTF flag set
    if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
    {
      //BTF TXE =1
      if(pI2CHandle->TxLen ==0)
      {
        // 1 generation the stop condition
        if(pI2CHandle->Sr == I2C_DISABLE_SR)
          I2C_GeneratesStopCondition(pI2CHandle->pI2Cx);
        
        //2. reset all the member element of the handle structure
        I2C_CloseSendData(pI2CHandle);
        //3 notify the application about trasmission complete
        I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
      }
    }
    else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
    {
      
    }
  }
  temp3 = pI2CHandle->pI2Cx->SR1 &(1<<I2C_SR1_STOPF); 
  if(temp1 && temp3)
  {
    //stop flag is set
    //clear the stop
    pI2CHandle->pI2Cx->CR1 |=0x0000;
    //notify the application that stop is detected
    I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
  }
  temp3 = pI2CHandle->pI2Cx->SR1&(1<<I2C_SR1_TXE);
  if(temp1 && temp2 && temp3)
  {
    //check for device mode
    if (pI2CHandle->pI2Cx->SR2 &(1<<I2C_SR2_MSL))
    {
      //TXE flag is set
      // we have to do the data transmission
      if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
      {
        I2C_MasterHandleTXEInterrupt(pI2CHandle);
      }
    }
    else
    {
      //slave
      //make sure that the slave is really in transmitter mode
      if(pI2CHandle->pI2Cx->SR2 &(1<<I2C_SR2_TRA))
      {
        I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
      }
    }
  }
  temp3 = pI2CHandle->pI2Cx->SR1 &(1<<I2C_SR1_RXNE);
  if (temp1 && temp2 && temp3)
  {
    //check device mode
    if (pI2CHandle->pI2Cx->SR2 &(1<<I2C_SR2_MSL))
    {  
      //the devide is master
      //TXE flag is set
      if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
      {
        I2C_MasterHandleRXNEInterrupt(pI2CHandle);
      }
    }
    else
    {
      //slave
      //make sure that the slave is really in transmitter mode
      if(pI2CHandle->pI2Cx->SR2 &(1<<I2C_SR2_TRA))
      {
        I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
      }
    }
  }
}
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
  uint32_t temp1, temp2;
  //ITERREN control bit
  temp2= (pI2CHandle->pI2Cx->CR2) &(1<<I2C_CR2_ITERREN);
  //check for bus error
  temp1= (pI2CHandle->pI2Cx->SR1) &(1<<I2C_SR1_BERR);
  if (temp1 && temp2)
  {  
    //this is bus error
    //implement code to clear the bus error flag
    pI2CHandle->pI2Cx->SR1 &= ~(1<<I2C_SR1_BERR);
    //implement code to notify the application about the error
    I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
  }
  //check for arbitration lost error
  temp1= (pI2CHandle->pI2Cx->SR1) &(1<<I2C_SR1_ARLO);
  if (temp1 && temp2)
  {  
    //this is arbitration lost error
    //implement code to clear the arbitration lost error flag
    pI2CHandle->pI2Cx->SR1 &= ~(1<<I2C_SR1_ARLO);
    //implement code to notify the application about the error
    I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
  }
  // check for ACK failure error
  temp1 = (pI2CHandle->pI2Cx->SR1) &(1<<I2C_SR1_AF);
  if (temp1 && temp2)
  {  
    //this is ACK failure error
    //implement code to clear the ACK failureerror flag
    pI2CHandle->pI2Cx->SR1 &= ~(1<<I2C_SR1_AF);
    //implement code to notify the application about the error
    I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
  }
  //check for overrun underrun error
  temp1= (pI2CHandle->pI2Cx->SR1) &(1<<I2C_SR1_OVR);
  if (temp1 && temp2)
  {  
    //this is overrun underrun error
    //implement code to clear the overrun underrun error flag
    pI2CHandle->pI2Cx->SR1 &= ~(1<<I2C_SR1_OVR);
    //implement code to notify the application about the error
    I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
  }
  //check for timeout error
  temp1= (pI2CHandle->pI2Cx->SR1) &(1<<I2C_SR1_TIMEOUT);
  if (temp1 && temp2)
  {  
    //this is overrun underrun error
    //implement code to clear the overrun underrun error flag
    pI2CHandle->pI2Cx->SR1 &= ~(1<<I2C_SR1_TIMEOUT);
    //implement code to notify the application about the error
    I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
  }
}
__attribute__((weak)) void I2C_ApplicationEventCallback(I2C_Handle_t *pUSARTHandle, uint8_t ApEv)
{

}