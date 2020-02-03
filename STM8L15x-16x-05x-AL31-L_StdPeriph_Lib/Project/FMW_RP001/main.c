/* Includes ------------------------------------------------------------------*/
#include "stm8l15x.h"
#include "string.h"
#include "RP_i2c.h"

#include "stm8_eval_i2c_ee.h"


/* Private define ------------------------------------------------------------*/
//#define ENABLE_GPIO
//#define ENABLE_ADC
#define READ_BUFFER_SIZE 64
#define WRITE_BUFFER_SIZE 64
#define DATAPOINT_SIZE 20
/* Private macro -------------------------------------------------------------*/
#define DATAPOINT_HANDLE(dp, field) ((uint8_t *)(dp.fmt.field))

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
volatile uint16_t ReadLength = 0;
volatile uint16_t WriteLength = 0;
static uint8_t ReadBuffer[READ_BUFFER_SIZE];
static uint8_t WriteBuffer[WRITE_BUFFER_SIZE];

static lsm9ds1_status_t status;
datapoint_t datapoint;

stmdev_ctx_t dev_ctx_mag;
stmdev_ctx_t dev_ctx_imu;

/* Private function prototypes -----------------------------------------------*/
#ifdef ENABLE_GPIO
void initGPIO(void);
#endif

#ifdef ENABLE_ADC
void initADC(void);
#endif

#ifdef ENABLE_GPIO
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
  /* High speed internal clock prescaler: 1 */
  CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_1);

  while (CLK_GetSYSCLKSource() != CLK_SYSCLKSource_HSI)
  {}
  
  memcpy(&WriteBuffer[0], "Hello World", 12);
  WriteLength = strlen("Hello World");
  
#ifdef ENABLE_GPIO
  initGPIO();
#endif
#ifdef ENABLE_ADC
  initADC();
#endif
  if(RP_I2C_Init() == RP_I2C_FAILURE){
    while(1); // Bus is hanging
  }
  
  //Enable global interrupts
  enableInterrupts();
  
  uint8_t len = 11;
  
  if(RP_EE_WaitEepromStandbyState() == RP_I2C_SUCCESS){
    if(RP_EE_WriteBuffer(EEAddress, WriteBuffer, WriteLength) == RP_I2C_SUCCESS){
      EEAddress += WriteLength;
      
      if(RP_EE_ReadBuffer(0, ReadBuffer, &len) == RP_I2C_SUCCESS){
        RP_I2C_WaitForOperationComplete(0);
        while(1); //Double check that write & read were successful
      } else{
        //the read failed
        while(1);
      }
    } else{
      //the write failed
      while(1);
    }
  }else{
    //the status check failed
    while(1);
  }
  
  /* Infinite loop */
  while (1)
  {
    /* Read IMU device status register */
    lsm9ds1_dev_status_get(&dev_ctx_mag, &dev_ctx_imu, &status);

    if ( status.status_imu.xlda && status.status_imu.gda && status.status_mag.zyxda)
    {
      /* Read data */
      memset(datapoint.bytes, 0x00, DATAPOINT_SIZE);

      lsm9ds1_acceleration_raw_get(&dev_ctx_imu, DATAPOINT_HANDLE(datapoint,acc));
      lsm9ds1_angular_rate_raw_get(&dev_ctx_imu, DATAPOINT_HANDLE(datapoint,ang));
      lsm9ds1_magnetic_raw_get(&dev_ctx_mag, DATAPOINT_HANDLE(datapoint,mag));

      /* Store timestamp */
      datapoint.fmt.time += 1;
      /* Store data */
      RP_EE_WriteBuffer(EEAddress, datapoint.bytes, DATAPOINT_SIZE);
      RP_I2C_WaitForOperationComplete(RP_I2C_TIMEOUT_MAX);
      EEAddress += DATAPOINT_SIZE;
    }
  }
}

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

