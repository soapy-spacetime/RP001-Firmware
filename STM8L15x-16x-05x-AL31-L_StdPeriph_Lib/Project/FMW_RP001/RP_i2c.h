/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RP_I2C
#define __RP_I2C

/* Includes ------------------------------------------------------------------*/
#include "stm8_eval.h"

/* Private types -------------------------------------------------------------*/
typedef enum {RP_DIRECTION_TX, RP_DIRECTION_RX} RP_DIRECTION;

/* Private define ------------------------------------------------------------*/
  
#define EE_SLAVE_ADDRESS     0xA0
#define ACC_GYRO_SLAVE_ADDRESS 0xD4
#define MAG_SLAVE_ADDRESS 0x38
#define OWN_SLAVE_ADDRESS    0xF0

#define I2C_SPEED          200000

#define EE_PAGESIZE    128 //actually 256, but the DMA can't handle that
 
/* Defintions for the state of the DMA transfer */   
#define sEE_STATE_READY         0
#define sEE_STATE_BUSY          1
   
/* Maximum timeout value for counting before exiting waiting loop on DMA 
   Trasnfer Complete. This value depends directly on the maximum page size and
   the sytem clock frequency. */
#define I2C_TIMEOUT_MAX         0x10000


/**
  * @brief  I2C EEPROM Interface pins
  */
#define RP_I2C                          I2C1
#define RP_I2C_CLK                      CLK_Peripheral_I2C1
#define RP_I2C_SCL_PIN                  GPIO_Pin_1                  /* PC.01 */
#define RP_I2C_SCL_GPIO_PORT            GPIOC                       /* GPIOC */
#define RP_I2C_SDA_PIN                  GPIO_Pin_0                  /* PC.00 */
#define RP_I2C_SDA_GPIO_PORT            GPIOC                       /* GPIOC */

#define RP_I2C_DMA                      DMA1
#define RP_I2C_DMA_CHANNEL_TX           DMA1_Channel3
#define RP_I2C_DMA_CHANNEL_RX           DMA1_Channel0
#define RP_I2C_DMA_FLAG_TX_TC           DMA1_FLAG_TC3
#define RP_I2C_DMA_FLAG_RX_TC           DMA1_FLAG_TC0
#define RP_I2C_DR_Address               ((uint16_t)0x005216)
#define RP_USE_DMA

  
/* Public Functions -------------------------------------------------------------*/
void RP_I2C_DeInit(void);
void RP_I2C_Init(void);
void RP_I2C_GenericRead(uint8_t SlaveAddr, uint16_t ReadAddr, bool ReadAddrIs2Bytes, uint8_t* pBuffer,  volatile uint8_t* pLength);
void RP_I2C_GenericWrite(uint8_t SlaveAddr, uint16_t WriteAddr, bool WriteAddrIs2Bytes, uint8_t* pBuffer, volatile uint8_t* pLength);
void RP_I2C_GenericWriteByte(uint8_t SlaveAddr, uint16_t WriteAddr, bool WriteAddrIs2Bytes, uint8_t Buffer);
bool RP_I2C_WaitForOperationComplete(uint32_t timout);
void RP_I2C_DMA_RX_IRQHandler(void);
void RP_I2C_DMA_TX_IRQHandler(void);

void RP_EE_WriteByte(uint32_t WriteAddr, uint8_t Buffer);
void RP_EE_WriteBuffer(uint32_t WriteAddr, uint8_t* pBuffer, uint8_t Length);
inline void RP_EE_ReadBuffer(uint32_t ReadAddr, uint8_t* pBuffer, volatile uint8_t* pLength);
void RP_EE_WaitEepromStandbyState(void);
#endif /* __RP_I2C */
