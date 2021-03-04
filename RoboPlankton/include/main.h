#ifndef __MAIN_H
#define __MAIN_H


/* Public include -----------------------------------------------------------*/
#ifndef __SDCC
#define __SDCC // to help IntelliSense navigate driver
#endif
#include "stm8l15x_conf.h"
#include "string.h"

/* Public define ------------------------------------------------------------*/
//#define ENABLE_GPIO
//#define ENABLE_ADC
#define ENABLE_TIM2

//#define EEPROM_TEST
//#define TIME_TEST
//#define DATA_LIMIT 10

#define DATA_FREQUENCY_HZ 100 // up to 150
#define DATA_PERIOD_MS (1000 / DATA_FREQUENCY_HZ)
#define DATA_TIME_FMT(ms) 1 //Just increment the time field for each data point - use the metadata to know how much time has passed
#define TIM2_PERIOD ((16000000 / 128 / DATA_FREQUENCY_HZ) - 1)

#define DATAPOINT_SIZE  18
#define METADATA_SIZE   4
#define EEPROM_SIZE ((uint32_t)(1)<<18)

/* Public macro -------------------------------------------------------------*/
#define DATAPOINT_HANDLE(dp, field) ((uint8_t *)(dp.fmt.field))
#define max(a,b) ((a) > (b) ? (a) : (b))
#define min(a,b) ((a) < (b) ? (a) : (b))
     
/* Public typedef -----------------------------------------------------------*/
typedef union{
  uint8_t bytes[DATAPOINT_SIZE];
  struct {
    int16_t acc[3];
    int16_t ang[3];
    int16_t mag[3];
  } fmt;
}datapoint_t;

typedef union{
  uint8_t bytes[METADATA_SIZE];
  struct{
    uint8_t sampleFrequency;
    // lsm9ds1_xl_fs_t accFullScale;
    // lsm9ds1_gy_fs_t gyrFullScale;
    // lsm9ds1_mag_fs_t magFullScale;
  }fmt;
}metadata_t;

/* Public variables ---------------------------------------------------------*/

/* Public function prototypes -----------------------------------------------*/

#endif // __MAIN_H