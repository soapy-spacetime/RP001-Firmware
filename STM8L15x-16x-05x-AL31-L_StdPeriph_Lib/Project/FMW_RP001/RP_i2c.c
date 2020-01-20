
/* Includes ------------------------------------------------------------------*/
#include "RP_i2c.h"


/* Private types -------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* 2MBit EEPROM has 18-bit addresses - top two bits are in slave address.
 * This macro converts the 18-bit address to a slave address
 */
#define EE_ADDRESS(wordAddr) ((uint8_t)(EE_SLAVE_ADDRESS | (0x06 &(wordAddr>>15))))
   
#define RP_EE_WritePage(pBuffer, WriteAddr, pLength) RP_I2C_GenericWrite(eeAddress, WriteAddr, TRUE, pBuffer, pLength)

/* Private variables -------------------------------------------------------------*/
__IO uint8_t eeAddress = 0;
__IO uint8_t accGyroAddress = 0;
__IO uint8_t magAddress = 0;
__IO uint32_t I2C_Timeout = I2C_TIMEOUT_MAX;
__IO uint8_t* I2C_DataRemainingPointer;
__IO uint8_t I2C_DataNum;

/* Low Level Functions (private) -----------------------------------------------*/
void RP_LowLevel_DeInit(void);
void RP_LowLevel_Init(void);
void RP_LowLevel_DMAConfig(uint16_t pBuffer, uint8_t BufferSize, RP_DIRECTION Direction);

/* EE Functions (private) -----------------------------------------------*/


/**
  * @brief  DeInitializes peripherals used by the I2C EEPROM driver.
  * @param  None
  * @retval None
  */
void RP_LowLevel_DeInit(void)
{
  /* RP_I2C Peripheral Disable */
  I2C_Cmd(RP_I2C, DISABLE);

  /* RP_I2C DeInit */
  I2C_DeInit(RP_I2C);

  /*!< RP_I2C Periph clock disable */
  CLK_PeripheralClockConfig(RP_I2C_CLK, DISABLE);

  /*!< GPIO configuration */
  /*!< Configure RP_I2C pins: SCL */
  GPIO_Init(RP_I2C_SCL_GPIO_PORT, RP_I2C_SCL_PIN, GPIO_Mode_In_PU_No_IT);

  /*!< Configure RP_I2C pins: SDA */
  GPIO_Init(RP_I2C_SDA_GPIO_PORT, RP_I2C_SDA_PIN, GPIO_Mode_In_PU_No_IT);

  /* Disable and Deinitialize the DMA channels */
  DMA_Cmd(RP_I2C_DMA_CHANNEL_TX, DISABLE);
  DMA_Cmd(RP_I2C_DMA_CHANNEL_RX, DISABLE);
  DMA_DeInit(RP_I2C_DMA_CHANNEL_TX);
  DMA_DeInit(RP_I2C_DMA_CHANNEL_RX);
}

/**
  * @brief  Initializes peripherals used by the I2C EEPROM driver.
  * @param  None
  * @retval None
  */
void RP_LowLevel_Init(void)
{
  /*!< RP_I2C Periph clock enable */
  CLK_PeripheralClockConfig(RP_I2C_CLK, ENABLE);

  /*!< Enable the DMA clock */
  CLK_PeripheralClockConfig(CLK_Peripheral_DMA1, ENABLE);

  /* I2C TX DMA Channel configuration */
  DMA_DeInit(RP_I2C_DMA_CHANNEL_TX);
  DMA_Init(RP_I2C_DMA_CHANNEL_TX,
           0, /* This parameter will be configured durig communication */
           RP_I2C_DR_Address,
           0xFF, /* This parameter will be configured durig communication */
           DMA_DIR_PeripheralToMemory,/* This parameter will be configured durig communication */
           DMA_Mode_Normal,
           DMA_MemoryIncMode_Inc,
           DMA_Priority_VeryHigh,
           DMA_MemoryDataSize_Byte);

  /* I2C RX DMA Channel configuration */
  DMA_DeInit(RP_I2C_DMA_CHANNEL_RX);
  DMA_Init(RP_I2C_DMA_CHANNEL_RX, 0, /* This parameter will be configured durig communication */
           RP_I2C_DR_Address,
           0xFF, /* This parameter will be configured durig communication */
           DMA_DIR_PeripheralToMemory,/* This parameter will be configured durig communication */
           DMA_Mode_Normal,
           DMA_MemoryIncMode_Inc,
           DMA_Priority_VeryHigh,
           DMA_MemoryDataSize_Byte);


}

/**
  * @brief  Initializes DMA channel used by the I2C EEPROM driver.
  * @param  None
  * @retval None
  */
void RP_LowLevel_DMAConfig(uint16_t pBuffer, uint8_t BufferSize, RP_DIRECTION Direction)
{
  /* Initialize the DMA with the new parameters */
  if (Direction == RP_DIRECTION_TX)
  {
    /* Configure the DMA Tx Channel with the buffer address and the buffer size */
    DMA_Init(RP_I2C_DMA_CHANNEL_TX, pBuffer, RP_I2C_DR_Address, BufferSize,
             DMA_DIR_MemoryToPeripheral, DMA_Mode_Normal, DMA_MemoryIncMode_Inc,
             DMA_Priority_VeryHigh, DMA_MemoryDataSize_Byte);
  }
  else
  {
    /* Configure the DMA Rx Channel with the buffer address and the buffer size */
    DMA_Init(RP_I2C_DMA_CHANNEL_RX, pBuffer, RP_I2C_DR_Address, BufferSize,
             DMA_DIR_PeripheralToMemory, DMA_Mode_Normal, DMA_MemoryIncMode_Inc,
             DMA_Priority_VeryHigh, DMA_MemoryDataSize_Byte);
  }

  /* Enable the DMA Channels Interrupts */
  DMA_ITConfig(RP_I2C_DMA_CHANNEL_TX, DMA_ITx_TC, ENABLE);
  DMA_ITConfig(RP_I2C_DMA_CHANNEL_RX, DMA_ITx_TC, ENABLE);
}



/**
  * @brief  DeInitializes peripherals used by the I2C EEPROM driver.
  * @param  None
  * @retval None
  */
void RP_I2C_DeInit(void)
{
  RP_LowLevel_DeInit();
}

/**
  * @brief  Initializes peripherals used by the I2C EEPROM driver.
  * @param  None
  * @retval None
  */
void RP_I2C_Init(void)
{
  RP_LowLevel_Init();

  /*!< I2C configuration */
  /* RP_I2C Peripheral Enable */
  I2C_Cmd(RP_I2C, ENABLE);
  /* RP_I2C configuration after enabling it */
  I2C_Init(RP_I2C, I2C_SPEED, OWN_SLAVE_ADDRESS, I2C_Mode_I2C, I2C_DutyCycle_2,
           I2C_Ack_Enable, I2C_AcknowledgedAddress_7bit);

  /* Enable the RP_I2C peripheral DMA requests */
  I2C_DMACmd(RP_I2C, ENABLE);

  eeAddress = EE_SLAVE_ADDRESS;
  accGyroAddress = ACC_GYRO_SLAVE_ADDRESS;
  magAddress = MAG_SLAVE_ADDRESS;
}

/**
  * @brief  Reads a block of data from the slave.
  * @param  pBuffer : pointer to the buffer that receives the data read from
  *         the slave.
  * @param  ReadAddr : slave's internal address to read from.
  * @param  pLength : pointer to the variable holding number of bytes to
  *         read from the slave.
  *
  *        @note The variable pointed by pLength is reset to 0 when all the
  *              data are read from the slave. Application should monitor this
  *              variable in order know when the transfer is complete.
  *
  * @note When number of data to be read is higher than 1, this function just
  *       configure the communication and enable the DMA channel to transfer data.
  *       Meanwhile, the user application may perform other tasks.
  *       When number of data to be read is 1, then the DMA is not used.
  *
  * @retval None
  */
void RP_I2C_GenericRead(uint8_t SlaveAddr, uint16_t ReadAddr, bool ReadAddrIs2Bytes, uint8_t* pBuffer,  volatile uint8_t* pLength)
{
  __IO uint32_t timeout = 0xFFFF;

  /*!< Wait the end of last communication */
  for (;timeout > 0; timeout--);

  /* Set the pointer to the Number of data to be read. This pointer will be used
      by the DMA Transfer Complete interrupt Handler in order to reset the 
      variable to 0. User should check on this variable in order to know if the 
      DMA transfer has been completed or not. */
  I2C_DataRemainingPointer = pLength;

  /*!< While the bus is busy */
  while (I2C_GetFlagStatus(RP_I2C, I2C_FLAG_BUSY))
  {}

  /*!< Send START condition */
  I2C_GenerateSTART(RP_I2C, ENABLE);

  /*!< Test on EV5 and clear it */
  while (!I2C_CheckEvent(RP_I2C, I2C_EVENT_MASTER_MODE_SELECT))
  {}

  /*!< Send EEPROM address for write */
  I2C_Send7bitAddress(RP_I2C, SlaveAddr, I2C_Direction_Transmitter);

  /*!< Test on EV6 and clear it */
  while (!I2C_CheckEvent(RP_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
  {}
  
  if(ReadAddrIs2Bytes){
    /*!< Send the EEPROM's internal address to read from: MSB of the address first */
    I2C_SendData(RP_I2C, (uint8_t)((ReadAddr & 0xFF00) >> 8));

    /*!< Test on EV8 and clear it */
    while (!I2C_CheckEvent(RP_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {}
  }
  
  /*!< Send the EEPROM's internal address to read from: LSB of the address */
  I2C_SendData(RP_I2C, (uint8_t)(ReadAddr & 0x00FF));


  /*!< Test on EV8 and clear it */
  while (!I2C_CheckEvent(RP_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
  {}

  /*!< Send START condition a second time */
  I2C_GenerateSTART(RP_I2C, ENABLE);

  /*!< Test on EV5 and clear it */
  while (!I2C_CheckEvent(RP_I2C, I2C_EVENT_MASTER_MODE_SELECT))
  {}

  /*!< Send EEPROM address for read */
  I2C_Send7bitAddress(RP_I2C, (uint8_t)eeAddress, I2C_Direction_Receiver);

  /*!< Test on EV6 and clear it */
  while (!I2C_CheckEvent(RP_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
  {}

  /* If number of data to be read is 1, then DMA couldn't be used */
  if ((*pLength) < 2)
  {
    /*!< Disable Acknowledgement */
    I2C_AcknowledgeConfig(RP_I2C, DISABLE);

    /*!< Send STOP Condition */
    I2C_GenerateSTOP(RP_I2C, ENABLE);

    /*!< Test on EV7 and clear it */
    while (!I2C_CheckEvent(RP_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
    {}

    /*!< Read a byte from the EEPROM */
    *pBuffer = I2C_ReceiveData(RP_I2C);

    /*!< Decrement the read bytes counter */
    (uint8_t)(*pLength)--;

    /*!< Enable Acknowledgement to be ready for another reception */
    I2C_AcknowledgeConfig(RP_I2C, ENABLE);
  }
  /* DMA could be used for number of data higher than 1 */
  else
  {
    /* Configure the DMA Rx Channel with the buffer address and the buffer size */
    RP_LowLevel_DMAConfig((uint16_t)pBuffer, (uint8_t)(*pLength), RP_DIRECTION_RX);

    /* Inform the DMA that the next End Of Transfer Signal will be the last one */
    I2C_DMALastTransferCmd(RP_I2C, ENABLE);

    /* Enable the DMA Rx Channel */
    DMA_Cmd(RP_I2C_DMA_CHANNEL_RX, ENABLE);

    /* Global DMA Enable */
    DMA_GlobalCmd(ENABLE);
  }
}

/**
  * @brief  Writes one byte to the I2C slave.
  * @param  pBuffer : pointer to the buffer  containing the data to be written
  *         to the slave.
  * @param  WriteAddr : slave's internal address to write to.
  * @retval None
  */
void RP_I2C_GenericWriteByte(uint8_t SlaveAddr, uint16_t WriteAddr, bool WriteAddrIs2Bytes, uint8_t Buffer)
{
  /*!< Send STRAT condition */
  I2C_GenerateSTART(RP_I2C, ENABLE);

  /*!< Test on EV5 and clear it */
  while (!I2C_CheckEvent(RP_I2C, I2C_EVENT_MASTER_MODE_SELECT))
  {}

  /*!< Send EEPROM address for write */
  I2C_Send7bitAddress(RP_I2C, SlaveAddr, I2C_Direction_Transmitter);

  /*!< Test on EV6 and clear it */
  while (!I2C_CheckEvent(RP_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
  {}
  
  if(WriteAddrIs2Bytes){
    /*!< Send the EEPROM's internal address to write to : MSB of the address first */
    I2C_SendData(RP_I2C, (uint8_t)((WriteAddr & 0xFF00) >> 8));

    /*!< Test on EV8 and clear it */
    while (!I2C_CheckEvent(RP_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {}
  }
  /*!< Send the EEPROM's internal address to write to : LSB of the address */
  I2C_SendData(RP_I2C, (uint8_t)(WriteAddr & 0x00FF));

  /*!< Test on EV8 and clear it */
  while (!I2C_CheckEvent(RP_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
  {}

  /*!< Send the byte to be written */
  I2C_SendData(RP_I2C, Buffer);

  /*!< Test on EV8 and clear it */
  while (!I2C_CheckEvent(RP_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
  {}

  /*!< Send STOP condition */
  I2C_GenerateSTOP(RP_I2C, ENABLE);
}

/**
  * @brief  Writes more than one byte to the slave with a single WRITE cycle.
  * @param  pBuffer : pointer to the buffer containing the data to be written.
  * @param  WriteAddr : slave's internal address to write to.
  * @param  NumByteToWrite : pointer to the variable holding number of bytes to
  *         be written.
  *
  *        @note The variable pointed by NumByteToWrite is reset to 0 when all the
  *              data is writted. Application should monitor this variable in order
  *               know when the transfer is complete.
  *
  * @note When number of data to be written is higher than 1, this function just
  *       configure the communication and enable the DMA channel to transfer data.
  *       Meanwhile, the user application may perform other tasks.
  *       When number of data to be written is 1, then the DMA is not used.
  *
  * @retval None
  */
void RP_I2C_GenericWrite(uint8_t SlaveAddr, uint16_t WriteAddr, bool WriteAddrIs2Bytes, uint8_t* pBuffer, volatile uint8_t* pLength)
{
  __IO uint32_t timeout = 0xFFFF;

  /*!< Wait the end of last communication */
  for (;timeout > 0; timeout--);

  /* Set the pointer to the Number of data to be written. This pointer will be used
      by the DMA Transfer Completer interrupt Handler in order to reset the 
      variable to 0. User should check on this variable in order to know if the 
      DMA transfer has been complete or not. */
  I2C_DataRemainingPointer = pLength;

  /*!< While the bus is busy */
  while (I2C_GetFlagStatus(RP_I2C, I2C_FLAG_BUSY))
  {}

  /*!< Send START condition */
  I2C_GenerateSTART(RP_I2C, ENABLE);

  /*!< Test on EV5 and clear it */
  while (!I2C_CheckEvent(RP_I2C, I2C_EVENT_MASTER_MODE_SELECT))
  {}

  /*!< Send EEPROM address for write */
  I2C_Send7bitAddress(RP_I2C, SlaveAddr, I2C_Direction_Transmitter);

  /*!< Test on EV6 and clear it */
  while (!I2C_CheckEvent(RP_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
  {}
  
  if(WriteAddrIs2Bytes){
    /*!< Send the EEPROM's internal address to write to : MSB of the address first */
    I2C_SendData(RP_I2C, (uint8_t)((WriteAddr & 0xFF00) >> 8));

    /*!< Test on EV8 and clear it */
    while (!I2C_CheckEvent(RP_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {}
  }

  /*!< Send the EEPROM's internal address to write to : LSB of the address */
  I2C_SendData(RP_I2C, (uint8_t)(WriteAddr & 0x00FF));

  /*!< Test on EV8 and clear it */
  while (! I2C_CheckEvent(RP_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
  {}
  
  /* If number of data to be written is 1, then DMA couldn't be used */
  if ((*pLength) < 2)
  {
    /*!< Send the byte to be written */
    I2C_SendData(RP_I2C, *pBuffer);

    /*!< Test on EV8 and clear it */
    while (!I2C_CheckEvent(RP_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {}

    /*!< Send STOP condition */
    I2C_GenerateSTOP(RP_I2C, ENABLE);

    (*pLength)--;
  }
  /* DMA could be used for number of data higher than 1 */
  else
  {
    /* Configure the DMA Tx Channel with the buffer address and the buffer size */
    RP_LowLevel_DMAConfig((uint16_t)pBuffer, (*pLength), RP_DIRECTION_TX);

    /* Enable the DMA Tx Channel */
    DMA_Cmd(RP_I2C_DMA_CHANNEL_TX, ENABLE);

    /* Global DMA Enable */
    DMA_GlobalCmd(ENABLE);
  }
}

bool RP_I2C_WaitForOperationComplete(uint32_t timout)
{
  I2C_Timeout = timout;
  while ((*I2C_DataRemainingPointer > 0) && (--I2C_Timeout > 0)){
    // wait
  }
  return (bool)(I2C_Timeout != 0);
}

/**
  * @brief  Writes one byte to the I2C EEPROM.
  * @param  pBuffer : pointer to the buffer  containing the data to be written
  *         to the EEPROM.
  * @param  WriteAddr : EEPROM's internal address to write to.
  * @retval None
  */
void RP_EE_WriteByte(uint32_t WriteAddr, uint8_t Buffer)
{
  eeAddress = EE_ADDRESS(WriteAddr);
  RP_I2C_GenericWriteByte(eeAddress, (uint16_t)WriteAddr, TRUE, Buffer);
}

/**
  * @brief  Reads a block of data from the EEPROM.
  * @param  pBuffer : pointer to the buffer that receives the data read from
  *         the EEPROM.
  * @param  ReadAddr : EEPROM's internal address to read from.
  * @param  pLength : pointer to the variable holding number of bytes to
  *         read from the EEPROM.
  *
  *        @note The variable pointed by pLength is reset to 0 when all the
  *              data are read from the EEPROM. Application should monitor this
  *              variable in order know when the transfer is complete.
  *
  * @note When number of data to be read is higher than 1, this function just
  *       configure the communication and enable the DMA channel to transfer data.
  *       Meanwhile, the user application may perform other tasks.
  *       When number of data to be read is 1, then the DMA is not used.
  *       Note that reads do not need to be page aligned.
  *
  * @retval None
  */
inline void RP_EE_ReadBuffer(uint32_t ReadAddr, uint8_t* pBuffer, volatile uint8_t* pLength)
{
  eeAddress = EE_ADDRESS(ReadAddr);
  RP_I2C_GenericRead(eeAddress, (uint16_t)ReadAddr, TRUE, pBuffer, pLength);
}


/**
  * @brief  Writes more than one byte to the EEPROM with a single WRITE cycle.
  * @note   The number of byte can't exceed the EEPROM page size.
  * @param  pBuffer : pointer to the buffer containing the data to be written to
  *         the EEPROM.
  * @param  WriteAddr : EEPROM's internal address to write to.
  * @param  pLength : pointer to the variable holding number of bytes to
  *         written to the EEPROM.
  *
  *        @note The variable pointed by pLength is reset to 0 when all the
  *              data are written to the EEPROM. Application should monitor this
  *              variable in order know when the transfer is complete.
  *
  * @note When number of data to be written is higher than 1, this function just
  *       configure the communication and enable the DMA channel to transfer data.
  *       Meanwhile, the user application may perform other tasks.
  *       When number of data to be written is 1, then the DMA is not used.
  *
  * @retval None
  
inline void RP_EE_WritePage(uint8_t* pBuffer, uint16_t WriteAddr, volatile uint8_t* pLength)
{
  RP_I2C_GenericWrite(eeAddress, WriteAddr, TRUE, pBuffer, pLength);
}
*/
/**
  * @brief  Writes buffer of data to the I2C EEPROM.
  * @param  pBuffer : pointer to the buffer  containing the data to be written
  *         to the EEPROM.
  * @param  WriteAddr : EEPROM's internal address to write to.
  * @param  Length : number of bytes to write to the EEPROM.
  * @retval None
  */
void RP_EE_WriteBuffer(uint32_t WriteAddr, uint8_t* pBuffer, uint8_t Length)
{
  eeAddress = EE_ADDRESS(WriteAddr);
  
  uint8_t NumOfPage = 0; //the number of whole pages covered by this write
  uint8_t NumOfSingle = 0; //the number of bytes that will not fit into a whole page
  uint8_t PageRemaining = 0; //the number of bytes remaining in the first page 

  // Note: PageRemaining will never be 0 (range from 1 to EE_PAGESIZE) 
  PageRemaining = (uint8_t)(EE_PAGESIZE - (WriteAddr % EE_PAGESIZE)); 

  if(Length < PageRemaining){
    /* Store the number of data to be written */
        I2C_DataNum = NumOfSingle;
        RP_EE_WritePage(pBuffer, (uint16_t)WriteAddr, &I2C_DataNum);
        /* Wait transfer through DMA to be complete */
        I2C_Timeout = I2C_TIMEOUT_MAX;
        while ((I2C_DataNum > 0) && (--I2C_Timeout > 0))
        {}
        RP_EE_WaitEepromStandbyState();
  }
  /*!< If Length > PageRemaining */
  else{
      /* Store the number of data to be written */
      I2C_DataNum = PageRemaining;
      RP_EE_WritePage(pBuffer, (uint16_t)WriteAddr, &I2C_DataNum);
      /* Wait transfer through DMA to be complete */
      I2C_Timeout = I2C_TIMEOUT_MAX;
      while ((I2C_DataNum > 0) && (--I2C_Timeout > 0))
      {}
      RP_EE_WaitEepromStandbyState();
      
      // Update pointers, now page aligned
      WriteAddr += PageRemaining;
      eeAddress = EE_ADDRESS(WriteAddr);
      pBuffer += PageRemaining;
      
      // Update length and calculate number of pages and single bytes
      Length -= PageRemaining;
      NumOfPage = (uint8_t)(Length / EE_PAGESIZE);
      NumOfSingle = (uint8_t)(Length % EE_PAGESIZE);

      // Transfer remaining pages
       while (NumOfPage--)
      {
        /* Store the number of data to be written */
        I2C_DataNum = EE_PAGESIZE;
        RP_EE_WritePage(pBuffer, (uint16_t)WriteAddr, &I2C_DataNum);
        /* Wait transfer through DMA to be complete */
        I2C_Timeout = I2C_TIMEOUT_MAX;
        while ((I2C_DataNum > 0) && (--I2C_Timeout > 0))
        {}
        RP_EE_WaitEepromStandbyState();
        
        /* Move the pointers forward */
        WriteAddr +=  EE_PAGESIZE;
        eeAddress = EE_ADDRESS(WriteAddr);
        pBuffer += EE_PAGESIZE;
      }

      //Transfer remaining single bytes
      if (NumOfSingle != 0)
      {
        /* Store the number of data to be written */
        I2C_DataNum = NumOfSingle;
        RP_EE_WritePage(pBuffer, (uint16_t)WriteAddr, &I2C_DataNum);
        /* Wait transfer through DMA to be complete */
        I2C_Timeout = I2C_TIMEOUT_MAX;
        while ((I2C_DataNum > 0) && (--I2C_Timeout > 0))
        {}
        RP_EE_WaitEepromStandbyState();
      }
  }
}

/**
  * @brief  Wait for EEPROM Standby state
  * @param  None
  * @retval None
  */
void RP_EE_WaitEepromStandbyState(void)
{
  __IO uint8_t tempreg = 0;
  __IO uint32_t timeout = 0xFFFF;

  do
  {
    /*!< Send START condition */
    I2C_GenerateSTART(RP_I2C, ENABLE);

    /* Test on EEPROM_I2C EV5 and clear it */
    while (!I2C_GetFlagStatus(RP_I2C, I2C_FLAG_SB))  /* EV5 */
    {
    }

    /*!< Send EEPROM address for write */
    I2C_Send7bitAddress(RP_I2C, (uint8_t)eeAddress, I2C_Direction_Transmitter);

    /*!< Wait for address aknowledgement */
    for (;timeout > 0; timeout--);

    /*!< Read sEE SR1 register to clear pending flags */
    tempreg = I2C_ReadRegister(RP_I2C, I2C_Register_SR1);

  }
  while (!(tempreg & 0x02));

  /*!< Clear AF flag */
  I2C_ClearFlag(RP_I2C, I2C_FLAG_AF);

  /*!< STOP condition */
  I2C_GenerateSTOP(RP_I2C, ENABLE);
}



/**
  * @brief  This function handles the DMA Tx Channel interrupt Handler.
  *     @note This function should be called in the
  *       DMA1_CHANNEL2_3_IRQHandler in the stm8l15x_it.c file.
  *
  *       // INTERRUPT_HANDLER(DMA1_CHANNEL2_3_IRQHandler, 3)
  *       // {
  *           // RP_I2C_DMA_TX_IRQHandler();
  *       // }
  * @param  None
  * @retval None
  */
void RP_I2C_DMA_TX_IRQHandler(void)
{
  /* Check if the DMA transfer is complete */
  if (DMA_GetFlagStatus(RP_I2C_DMA_FLAG_TX_TC) != RESET)
  {
    /* Disable the DMA Tx Channel and Clear all its Flags */
    DMA_Cmd(RP_I2C_DMA_CHANNEL_TX, DISABLE);
    DMA_ClearFlag(RP_I2C_DMA_FLAG_TX_TC);

    /*!< Wait till all data have been physically transferred on the bus */
    while (!I2C_CheckEvent(RP_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {}

    /*!< Send STOP condition */
    I2C_GenerateSTOP(RP_I2C, ENABLE);

    /* Reset the variable holding the number of data to be written */
    *I2C_DataRemainingPointer = 0;
  }
}

/**
  * @brief  This function handles the DMA Rx Channel interrupt Handler.
  *     @note This function should be called in the
  *       DMA1_CHANNEL0_1_IRQHandler in the stm8l15x_it.c file
  *       just as follow.
  *
  *       // INTERRUPT_HANDLER(DMA1_CHANNEL0_1_IRQHandler, 2)
  *       // {
  *           // RP_I2C_DMA_RX_IRQHandler();
  *       // }
  * @param  None
  * @retval None
  */
void RP_I2C_DMA_RX_IRQHandler(void)
{
  /* Check if the DMA transfer is complete */
  if (DMA_GetFlagStatus(RP_I2C_DMA_FLAG_RX_TC) != RESET)
  {
    /*!< Send STOP Condition */
    I2C_GenerateSTOP(RP_I2C, ENABLE);

    /* Disable the DMA Rx Channel and Clear all its Flags */
    DMA_Cmd(RP_I2C_DMA_CHANNEL_RX, DISABLE);
    DMA_ClearFlag(RP_I2C_DMA_FLAG_RX_TC);

    /* Reset the variable holding the number of data to be read */
    *I2C_DataRemainingPointer = 0;
  }
}

/**
  * @}
  */


/**
  * @}
  */


