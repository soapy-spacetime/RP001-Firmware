/* Includes ------------------------------------------------------------------*/
#include "stm8l15x.h"
#include "string.h"
#include "RP_i2c.h"

#include "stm8_eval_i2c_ee.h"


/* Private define ------------------------------------------------------------*/
//#define ENABLE_GPIO
//#define ENABLE_ADC
#define ENABLE_TIM2
#define EEPROM_TEST
//#define TIME_TEST

#define DATA_LIMIT 10

#define DATA_FREQUENCY_HZ 10
#define DATA_PERIOD_MS (1000 / DATA_FREQUENCY_HZ)
#define DATA_TIME_FMT(ms) ((ms) / 10) //Store the data in units of 10s of ms (increments of 1 @ 100Hz)
#define TIM2_PERIOD ((16000000 / 128 / DATA_FREQUENCY_HZ) - 1)

#define DATAPOINT_SIZE 20
#define EEPROM_SIZE (1<<18)
#define END_OF_DATA_BYTE 0xAA
/* Private macro -------------------------------------------------------------*/
#define DATAPOINT_HANDLE(dp, field) ((uint8_t *)(dp.fmt.field))
#define max(a,b) ((a) > (b) ? (a) : (b))
#define min(a,b) ((a) < (b) ? (a) : (b))
     
/* Private typedef -----------------------------------------------------------*/
typedef union{
  uint8_t bytes[DATAPOINT_SIZE];
  struct {
    uint16_t time;
    int16_t acc[3];
    int16_t ang[3];
    int16_t mag[3];
  } fmt;
}datapoint_t;
/* Private variables ---------------------------------------------------------*/
static uint32_t EEAddress = 0;
#ifdef EEPROM_TEST
#define READ_BUFFER_SIZE 64
#define WRITE_BUFFER_SIZE 64
volatile uint16_t WriteLength = 0;
static uint8_t WriteBuffer[WRITE_BUFFER_SIZE];
volatile uint16_t ReadLength = 0;
static uint8_t ReadBuffer[READ_BUFFER_SIZE];
#endif

static lsm9ds1_status_t status;
datapoint_t datapoint;

stmdev_ctx_t dev_ctx_mag;
stmdev_ctx_t dev_ctx_imu;

/* Private function prototypes -----------------------------------------------*/
#ifdef ENABLE_TIM2
void TIM2_Config(void);
#endif

#ifdef ENABLE_ADC
void initADC(void);
#endif

#ifdef ENABLE_GPIO
void initGPIO(void);
void callback_INT1(void);
void callback_INT2(void);
void callback_INTM(void);
void callback_DRDY_M(void);
#endif

/* Private functions ---------------------------------------------------------*/

/**
* @brief  Main program.
* @param  None
* @retval None
*/
void main(void)
{
  /* ----- Initialize System Clocks ----- */
  /* Select HSI (16MHz) as system clock source */
  CLK_SYSCLKSourceSwitchCmd(ENABLE);
  CLK_SYSCLKSourceConfig(CLK_SYSCLKSource_HSI);
  //  /* High speed internal clock prescaler: 1 */
  CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_1);
  
  while (CLK_GetSYSCLKSource() != CLK_SYSCLKSource_HSI)
  {}
  
#ifdef TIME_TEST
  TIM2_Config();
  bool state = FALSE;
  
  // Toggle SCL to free bus
  GPIO_Init(GPIOC, GPIO_Pin_1, GPIO_Mode_Out_OD_HiZ_Slow); //PC1 - SCL
  while(1){
    if(TIM2_GetFlagStatus(TIM2_FLAG_Update)){
      TIM2_ClearFlag(TIM2_FLAG_Update);
      
      GPIO_WriteBit(GPIOC, GPIO_Pin_1, state);
      state = !state;
    }
  }
#endif
  
  // Toggle SCL to free bus
  GPIO_Init(GPIOC, GPIO_Pin_1, GPIO_Mode_Out_OD_HiZ_Slow); //PC1 - SCL
  
  for(int j = 0; j < 10; j++){
    GPIO_WriteBit(GPIOC, GPIO_Pin_1, 0);
    for(int i = 0; i < 100; i++){
    }
    GPIO_WriteBit(GPIOC, GPIO_Pin_1, 1);
    for(int i = 0; i < 100; i++){
    }
  }
  GPIO_DeInit(GPIOC);
  
  
  if(RP_I2C_Init() == RP_I2C_FAILURE){
    while(1);    
  }
  
#ifdef ENABLE_TIM2
  TIM2_Config();
#endif
#ifdef ENABLE_GPIO
  initGPIO();
#endif
#ifdef ENABLE_ADC
  initADC();
#endif
  
  //Enable global interrupts
  enableInterrupts();
  
  
  
#ifdef EEPROM_TEST
/* TODO! Figure out:
 *      - Which slave is sending the Nack
 *      - How to handle the Nack appropriately
 */
  memcpy(&WriteBuffer[0], "Hello World", 12);
  WriteLength = strlen("Hello World");
  
  if(RP_EE_WriteBuffer(EEAddress, WriteBuffer, WriteLength) == RP_I2C_SUCCESS){
    EEAddress += WriteLength;
    while(1){
    if(RP_EE_ReadBuffer(0, ReadBuffer, 0) == RP_I2C_FAILURE)
      if(RP_EE_ReadBuffer(0, ReadBuffer, 1) == RP_I2C_SUCCESS)
        if(RP_EE_ReadBuffer(0, ReadBuffer, 2) == RP_I2C_SUCCESS)
          if(RP_EE_ReadBuffer(0, ReadBuffer, 3) == RP_I2C_SUCCESS)
            RP_EE_ReadBuffer(0, ReadBuffer, WriteLength);
    }
  }
  while(1);
#endif
  
#ifdef DATA_LIMIT
  uint32_t count = 0;
#endif
  
  /* Infinite loop */
  while (1)
  {
#ifdef ENABLE_TIM2
    // Wait for correct period
    if(TIM2_GetFlagStatus(TIM2_FLAG_Update)){
      TIM2_ClearFlag(TIM2_FLAG_Update);
#else
    /* Read IMU device status register */
    lsm9ds1_dev_status_get(&dev_ctx_mag, &dev_ctx_imu, &status);
    if ( status.status_imu.xlda && status.status_imu.gda && status.status_mag.yda && status.status_mag.zda){
#endif
  
      /* Read data */      
      lsm9ds1_acceleration_raw_get(&dev_ctx_imu, DATAPOINT_HANDLE(datapoint,acc));
      lsm9ds1_angular_rate_raw_get(&dev_ctx_imu, DATAPOINT_HANDLE(datapoint,ang));
      lsm9ds1_magnetic_raw_get(&dev_ctx_mag, DATAPOINT_HANDLE(datapoint,mag));
      
      /* Store timestamp */
      datapoint.fmt.time += DATA_TIME_FMT(DATA_PERIOD_MS);
      /* Store data */
      RP_EE_WriteBuffer(EEAddress, datapoint.bytes, DATAPOINT_SIZE);
      EEAddress += DATAPOINT_SIZE;
      
      if(EEAddress >= (EEPROM_SIZE - DATAPOINT_SIZE))
        break;
      
#ifdef DATA_LIMIT
      if(++count >= DATA_LIMIT)
        break;
#endif
    }
  } 
  /*End Infinite loop */
  
  // Write remaining EEPROM with filler to denote end of sampling
  memset(datapoint.bytes, END_OF_DATA_BYTE, DATAPOINT_SIZE);
  RP_EE_WriteBuffer(EEAddress, datapoint.bytes, min((EEPROM_SIZE - EEAddress), DATAPOINT_SIZE));
  
  //Stop
  while(1);
}

#ifdef ENABLE_TIM2
/**
  * @brief  Configure TIM2 peripheral   
  * @param  None
  * @retval None
  */
static void TIM2_Config(void)
{
  /* TIM2 configuration:
     - TIM2 counter is clocked by HSI div 128 = 125 KHz
    TIM2 Channel1 output frequency = TIM2CLK / (TIM2 Prescaler * (TIM2_PERIOD + 1))
                                   = 16MHz / (128 * 625) = 200 Hz */
  
  /* Enable TIM2 clock */
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM2, ENABLE);
  
  /* Time Base configuration */
  TIM2_TimeBaseInit(TIM2_Prescaler_128, TIM2_CounterMode_Up, TIM2_PERIOD);

  /* Channel 1 configuration in Active mode */
  TIM2_OC1Init(TIM2_OCMode_Active, TIM2_OutputState_Enable, 0, TIM2_OCPolarity_High, TIM2_OCIdleState_Reset);

  
//  TIM2_ITConfig(TIM2_IT_Update, ENABLE);
  
  /* TIM2 counter enable */
  TIM2_Cmd(ENABLE);
}
#endif

#ifdef ENABLE_GPIO
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
#endif

#ifdef ENABLE_ADC
void initADC(void){
  ADC_Init(ADC1, ADC_ConversionMode_Single, ADC_Resolution_12Bit, ADC_Prescaler_1);
}
#endif


#ifdef  USE_FULL_ASSERT

/**
* @brief  Reports the name of the source file and the source line number
*   where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
* @retval None
*/
void assert_failed(void)
{ 
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {
    
  }
}

void log(char* msg){
  //strcpy(WriteBuffer, msg);
}
#endif

