/**
  ******************************************************************************
  * @file    Project/STM8L15x_StdPeriph_Template/main.c
  * @author  MCD Application Team
  * @version V1.6.1
  * @date    30-September-2014
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm8l15x.h"
#include "string.h"
#include "RP_i2c.h"

/** @addtogroup STM8L15x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define READ_BUFFER_SIZE 64
#define WRITE_BUFFER_SIZE 256
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t EEAddress = 0;
volatile uint16_t ReadLength = 0;
volatile uint16_t WriteLength = 0;
uint8_t ReadBuffer[READ_BUFFER_SIZE];
uint8_t WriteBuffer[WRITE_BUFFER_SIZE];

/* Private function prototypes -----------------------------------------------*/
void initGPIO(void);
void initADC(void);

void callback_INT1(void);
void callback_INT2(void);
void callback_INTM(void);
void callback_DRDY_M(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
void main(void)
{
  memcpy("Hello World", &WriteBuffer[0], 12);
  WriteLength = strlen("Hello World");
  
  //initGPIO();
  RP_I2C_Init();
  //initADC();
  
  //Enable global interrupts
  enableInterrupts();
  
  RP_EE_WriteBuffer(EEAddress, WriteBuffer, WriteLength);
  EEAddress += WriteLength;
  
  /* Infinite loop */
  while (1)
  {
    
  }
}


void initGPIO(void){
  GPIO_Init(GPIOD, GPIO_Pin_0, GPIO_Mode_In_FL_No_IT); //TP1
  GPIO_Init(GPIOA, GPIO_Pin_3, GPIO_Mode_In_FL_No_IT); //TP2
  GPIO_Init(GPIOA, GPIO_Pin_4, GPIO_Mode_In_FL_No_IT); //TP3
  GPIO_Init(GPIOC, GPIO_Pin_6, GPIO_Mode_In_FL_No_IT); //TP4
  
  GPIO_Init(GPIOC, GPIO_Pin_3, GPIO_Mode_In_FL_IT); //INT1
  EXTI_SetPinSensitivity(EXTI_Pin_2, EXTI_Trigger_Falling);
  
  GPIO_Init(GPIOC, GPIO_Pin_2, GPIO_Mode_In_FL_IT); //INT2
  EXTI_SetPinSensitivity(EXTI_Pin_2, EXTI_Trigger_Falling);
  
  GPIO_Init(GPIOC, GPIO_Pin_4, GPIO_Mode_In_FL_IT); //INT_M
  EXTI_SetPinSensitivity(EXTI_Pin_2, EXTI_Trigger_Falling);
  
  GPIO_Init(GPIOC, GPIO_Pin_5, GPIO_Mode_In_FL_IT); //DRDY_M
  EXTI_SetPinSensitivity(EXTI_Pin_2, EXTI_Trigger_Falling);
  
}

void callback_INT1(void)
{
  
}

void callback_INT2(void)
{
  
}

void callback_INTM(void)
{
  
}

void callback_DRDY_M(void)
{
  
}

void initADC(void){
  ADC_Init(ADC1, ADC_ConversionMode_Single, ADC_Resolution_12Bit, ADC_Prescaler_1);
}



#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
    
  }
}
#endif

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
