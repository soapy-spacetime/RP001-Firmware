
/* Includes ------------------------------------------------------------------*/
#include "RP_i2c.h"


/* Private types -------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
//#define SIMPLE_INIT
/* Private macro -------------------------------------------------------------*/

/* 2MBit EEPROM has 18-bit addresses - top two bits are in slave address.
* This macro converts the 18-bit address to a slave address
*/
#define EE_ADDRESS(wordAddr) ((uint8_t)(EE_SLAVE_ADDRESS | (0x06 &(wordAddr>>15))))

#define RP_EE_WritePage(pBuffer, WriteAddr, pLength) RP_I2C_GenericWrite(eeAddress, WriteAddr, TRUE, pBuffer, pLength)

/* Private variables -------------------------------------------------------------*/
uint8_t eeAddress = 0;
uint8_t imuAddress = 0;
uint8_t magAddress = 0;
__IO uint32_t I2C_Timeout = RP_I2C_TIMEOUT_MAX;

lsm9ds1_id_t whoamI = {.imu = 0, .mag = 0};

/* Low Level Functions (private) -----------------------------------------------*/
void RP_LowLevel_DeInit(void);
void RP_LowLevel_Init(void);
#ifdef RP_USE_DMA
void RP_LowLevel_DMAConfig(uint16_t pBuffer, uint8_t BufferSize, RP_DIRECTION Direction);
#endif

/* EE Functions (private) -----------------------------------------------*/
bool RP_EE_WaitForStandby(void);

uint8_t EE_PageBuffer[EE_PAGESIZE];
uint16_t EE_PageBuffer_ind = 0;
uint32_t EE_PageBuffer_Address = 0;

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
  
#ifdef RP_USE_DMA
  /* Disable and Deinitialize the DMA channels */
  DMA_Cmd(RP_I2C_DMA_CHANNEL_TX, DISABLE);
  DMA_Cmd(RP_I2C_DMA_CHANNEL_RX, DISABLE);
  DMA_DeInit(RP_I2C_DMA_CHANNEL_TX);
  DMA_DeInit(RP_I2C_DMA_CHANNEL_RX);
#endif
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
  
#ifdef RP_USE_DMA
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
#endif
}

#ifdef RP_USE_DMA
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
#endif


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
bool RP_I2C_Init(void)
{
  RP_LowLevel_Init();
  
  /*!< I2C configuration */
  /* RP_I2C Peripheral Enable */
  I2C_Cmd(RP_I2C, ENABLE);
  /* RP_I2C configuration after enabling it */
  I2C_Init(RP_I2C, RP_I2C_SPEED, OWN_SLAVE_ADDRESS, I2C_Mode_I2C, I2C_DutyCycle_16_9,
           I2C_Ack_Enable, I2C_AcknowledgedAddress_7bit);
  
#ifdef RP_USE_DMA
  /* Enable the RP_I2C peripheral DMA requests */
  I2C_DMACmd(RP_I2C, ENABLE);
#endif
  
  eeAddress = EE_SLAVE_ADDRESS;
  imuAddress = IMU_SLAVE_ADDRESS;
  magAddress = MAG_SLAVE_ADDRESS;
  
  if(I2C_GetFlagStatus(RP_I2C,I2C_FLAG_BUSY)){
    return RP_I2C_FAILURE;
  }
#ifdef SIMPLE_INIT
  return RP_I2C_SUCCESS;
#endif
  /* Initialize magnetic sensors driver interface */
  dev_ctx_mag.write_reg = platform_write;
  dev_ctx_mag.read_reg = platform_read;
  dev_ctx_mag.handle = (void*)&magAddress;
  
  /* Initialize inertial sensors (IMU) driver interface */
  dev_ctx_imu.write_reg = platform_write;
  dev_ctx_imu.read_reg = platform_read;
  dev_ctx_imu.handle = (void*)&imuAddress;
    
  /* Restore default configuration - set reset bit, wait for it to be cleared*/
  uint8_t rst;
  if(lsm9ds1_dev_reset_set(&dev_ctx_mag, &dev_ctx_imu, PROPERTY_ENABLE) != RP_I2C_SUCCESS)
    return RP_I2C_FAILURE;
  do {
    lsm9ds1_dev_reset_get(&dev_ctx_mag, &dev_ctx_imu, &rst);
  } while (rst);
  
  /* Check device ID */
  if(lsm9ds1_dev_id_get(&dev_ctx_mag, &dev_ctx_imu, &whoamI) != RP_I2C_SUCCESS){
    return RP_I2C_FAILURE;
  } else if (
             (whoamI.imu != LSM9DS1_IMU_ID) || 
               (whoamI.mag != LSM9DS1_MAG_ID)
                 ){
                   return RP_I2C_FAILURE;
                 }
      
  /* Set full scale */
  lsm9ds1_xl_full_scale_set(&dev_ctx_imu, RP_ACC_FULL_SCALE);
  lsm9ds1_gy_full_scale_set(&dev_ctx_imu, RP_GYR_FULL_SCALE);
  lsm9ds1_mag_full_scale_set(&dev_ctx_mag, RP_MAG_FULL_SCALE);
  
  /* Configure filtering chain - See datasheet for filtering chain details */
  /* Accelerometer filtering chain */
  lsm9ds1_xl_filter_aalias_bandwidth_set(&dev_ctx_imu, LSM9DS1_50Hz);   // Assuming this is LPF1
  lsm9ds1_xl_filter_lp_bandwidth_set(&dev_ctx_imu, LSM9DS1_LP_DISABLE); // Disable LPF2
  lsm9ds1_xl_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LP_OUT);         // LPF2 disabled, so this is actually unfiltered output
  
  /* Gyroscope filtering chain */
  //lsm9ds1_gy_filter_lp_bandwidth_set(&dev_ctx_imu, LSM9DS1_LP_ULTRA_LIGHT);   //LPF2 bypassed (see datasheet Table 47)
  //lsm9ds1_gy_filter_hp_bandwidth_set(&dev_ctx_imu, LSM9DS1_HP_ULTRA_LOW);     //HPF bypassed
  lsm9ds1_gy_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LPF1_OUT); // Bypass LPF2 & HPF; LPF1 BW set by ODR
  
  /* Set Output Data Rate / Power mode */
  lsm9ds1_imu_data_rate_set(&dev_ctx_imu, LSM9DS1_IMU_238Hz);           // sets Gyro LPF1 BW to 76Hz
  lsm9ds1_mag_data_rate_set(&dev_ctx_mag, LSM9DS1_MAG_UHP_155Hz);       // no filtering on Magnetometer
  lsm9ds1_filter_settling_mask_set(&dev_ctx_imu, TRUE);                 // make data ready flag wait for filters to settle 
  
  /* Enable Block Data Update */
  lsm9ds1_block_data_update_set(&dev_ctx_mag, &dev_ctx_imu, PROPERTY_ENABLE);
  
  return RP_I2C_SUCCESS;
}

/**
* @brief  Device status register.[get]
*
* @param  ctx_mag   Read / write magnetometer interface definitions.(ptr)
* @param  ctx_imu   Read / write imu interface definitions.(ptr)
* @param  val       Device status registers.(ptr)
* @retval           Interface status (MANDATORY: return 0 -> no Error).
*
*/
int32_t lsm9ds1_dev_status_get(stmdev_ctx_t *ctx_mag, stmdev_ctx_t *ctx_imu,
                               lsm9ds1_status_t *val)
{
  int32_t ret;
  
  ret = lsm9ds1_read_reg(ctx_imu, LSM9DS1_STATUS_REG,
                         (uint8_t*)&(val->status_imu), 1);
  if(ret == 0){
    ret = lsm9ds1_read_reg(ctx_mag, LSM9DS1_STATUS_REG_M,
                           (uint8_t*)&(val->status_mag), 1);
  }
  
  return ret;
}

/**
* @brief  Mask DRDY on pin (both XL & Gyro) until filter settling ends.[set]
*
* @param  ctx    Read / write interface definitions.(ptr)
* @param  val    Change the values of drdy_mask_bit in reg CTRL_REG9.
* @retval        Interface status (MANDATORY: return 0 -> no Error).
*
*/
int32_t lsm9ds1_filter_settling_mask_set(stmdev_ctx_t *ctx, uint8_t val)
{
  lsm9ds1_ctrl_reg9_t ctrl_reg9;
  int32_t ret;
  
  ret = lsm9ds1_read_reg(ctx, LSM9DS1_CTRL_REG9, (uint8_t*)&ctrl_reg9, 1);
  if(ret == 0){
    ctrl_reg9.drdy_mask_bit = (uint8_t)val;
    ret = lsm9ds1_write_reg(ctx, LSM9DS1_CTRL_REG9, (uint8_t*)&ctrl_reg9, 1);
  }
  return ret;
}

/**
* @brief  Magnetometer data rate selection.[set]
*
* @param  ctx    Read / write interface definitions.(ptr)
* @param  val    Change the values of "fast_odr" in reg LSM9DS1.
* @retval        Interface status (MANDATORY: return 0 -> no Error).
*
*/
int32_t lsm9ds1_mag_data_rate_set(stmdev_ctx_t *ctx,
                                  lsm9ds1_mag_data_rate_t val)
{
  lsm9ds1_ctrl_reg1_m_t ctrl_reg1_m;
  lsm9ds1_ctrl_reg3_m_t ctrl_reg3_m;
  lsm9ds1_ctrl_reg4_m_t ctrl_reg4_m;
  int32_t ret;
  
  ret = lsm9ds1_read_reg(ctx, LSM9DS1_CTRL_REG1_M, (uint8_t*)&ctrl_reg1_m, 1);
  if(ret == 0){
    ctrl_reg1_m.fast_odr = (((uint8_t)val & 0x08U) >> 3);
    ctrl_reg1_m._do = ((uint8_t)val & 0x07U);
    ctrl_reg1_m.om = (((uint8_t)val & 0x30U) >> 4);
    ctrl_reg1_m.temp_comp = PROPERTY_ENABLE;
    ret = lsm9ds1_write_reg(ctx, LSM9DS1_CTRL_REG1_M,
                            (uint8_t*)&ctrl_reg1_m, 1);
  }
  if(ret == 0){
    ret = lsm9ds1_read_reg(ctx, LSM9DS1_CTRL_REG3_M,
                           (uint8_t*)&ctrl_reg3_m, 1);
  }
  if(ret == 0){
    ctrl_reg3_m.md = (((uint8_t)val & 0xC0U) >> 6);
    ret = lsm9ds1_write_reg(ctx, LSM9DS1_CTRL_REG3_M,
                            (uint8_t*)&ctrl_reg3_m, 1);
  }
  if(ret == 0){
    ret = lsm9ds1_read_reg(ctx, LSM9DS1_CTRL_REG4_M, (uint8_t*)&ctrl_reg4_m, 1);
  }
  if(ret == 0){
    ctrl_reg4_m.omz = (((uint8_t)val & 0x30U) >> 4);;
    ret = lsm9ds1_write_reg(ctx, LSM9DS1_CTRL_REG4_M,
                            (uint8_t*)&ctrl_reg4_m, 1);
  }
  return ret;
}

/**
* @brief  Data rate selection when both the accelerometer and gyroscope
*         are activated.[set]
*
* @param  ctx    Read / write interface definitions.(ptr)
* @param  val    Change the values of "odr_g" in reg LSM9DS1.
* @retval        Interface status (MANDATORY: return 0 -> no Error).
*
*/
int32_t lsm9ds1_imu_data_rate_set(stmdev_ctx_t *ctx, lsm9ds1_imu_odr_t val)
{
  lsm9ds1_ctrl_reg1_g_t ctrl_reg1_g;
  lsm9ds1_ctrl_reg6_xl_t ctrl_reg6_xl;
  lsm9ds1_ctrl_reg3_g_t ctrl_reg3_g;
  int32_t ret;
  
  ret = lsm9ds1_read_reg(ctx, LSM9DS1_CTRL_REG1_G, (uint8_t*)&ctrl_reg1_g, 1);
  if(ret == 0){
    ctrl_reg1_g.odr_g = (uint8_t)val & 0x07U;
    ret = lsm9ds1_write_reg(ctx, LSM9DS1_CTRL_REG1_G,
                            (uint8_t*)&ctrl_reg1_g, 1);
  }
  if(ret == 0){
    ret = lsm9ds1_read_reg(ctx, LSM9DS1_CTRL_REG6_XL,
                           (uint8_t*)&ctrl_reg6_xl, 1);
  }
  if(ret == 0){
    ctrl_reg6_xl.odr_xl = (((uint8_t)val & 0x70U) >> 4);
    ret = lsm9ds1_write_reg(ctx, LSM9DS1_CTRL_REG6_XL,
                            (uint8_t*)&ctrl_reg6_xl, 1);
  }
  if(ret == 0){
    ret = lsm9ds1_read_reg(ctx, LSM9DS1_CTRL_REG3_G,
                           (uint8_t*)&ctrl_reg3_g, 1);
  }
  if(ret == 0){
    ctrl_reg3_g.lp_mode = (((uint8_t)val & 0x80U) >> 7);
    ret = lsm9ds1_write_reg(ctx, LSM9DS1_CTRL_REG3_G,
                            (uint8_t*)&ctrl_reg3_g, 1);
  }
  
  return ret;
}

/**
* @brief  Gyro output filter path configuration.[set]
*
* @param  ctx    Read / write interface definitions.(ptr)
* @param  val    Change the values of "out_sel" in reg LSM9DS1.
* @retval        Interface status (MANDATORY: return 0 -> no Error).
*
*/
int32_t lsm9ds1_gy_filter_out_path_set(stmdev_ctx_t *ctx,
                                       lsm9ds1_gy_out_path_t val)
{
  lsm9ds1_ctrl_reg2_g_t ctrl_reg2_g;
  lsm9ds1_ctrl_reg3_g_t ctrl_reg3_g;
  int32_t ret;
  
  ret = lsm9ds1_read_reg(ctx, LSM9DS1_CTRL_REG2_G,
                         (uint8_t*)&ctrl_reg2_g, 1);
  if(ret == 0){
    ctrl_reg2_g.out_sel = ((uint8_t)val & 0x03U);
    ret = lsm9ds1_write_reg(ctx, LSM9DS1_CTRL_REG2_G,
                            (uint8_t*)&ctrl_reg2_g, 1);
  }
  if(ret == 0){
    ret = lsm9ds1_read_reg(ctx, LSM9DS1_CTRL_REG3_G,
                           (uint8_t*)&ctrl_reg3_g, 1);
  }
  if(ret == 0){
    ctrl_reg3_g.hp_en = (((uint8_t)val & 0x10U) >> 4 );
    ret = lsm9ds1_write_reg(ctx, LSM9DS1_CTRL_REG3_G,
                            (uint8_t*)&ctrl_reg3_g, 1);
  }
  
  return ret;
}

/**
* @brief  Gyroscope high-pass filter bandwidth selection.[set]
*
* @param  ctx    Read / write interface definitions.(ptr)
* @param  val    Change the values of "hpcf_g" in reg LSM9DS1.
* @retval        Interface status (MANDATORY: return 0 -> no Error).
*
*/
int32_t lsm9ds1_gy_filter_hp_bandwidth_set(stmdev_ctx_t *ctx,
                                           lsm9ds1_gy_hp_bw_t val)
{
  lsm9ds1_ctrl_reg3_g_t ctrl_reg3_g;
  int32_t ret;
  
  ret = lsm9ds1_read_reg(ctx, LSM9DS1_CTRL_REG3_G, (uint8_t*)&ctrl_reg3_g, 1);
  if(ret == 0){
    ctrl_reg3_g.hpcf_g = (uint8_t)val;
    ret = lsm9ds1_write_reg(ctx, LSM9DS1_CTRL_REG3_G,
                            (uint8_t*)&ctrl_reg3_g, 1);
  }
  return ret;
}

/**
* @brief  Gyroscope lowpass filter bandwidth selection.[set]
*
* @param  ctx    Read / write interface definitions.(ptr)
* @param  val    Change the values of "bw_g" in reg LSM9DS1.
* @retval        Interface status (MANDATORY: return 0 -> no Error).
*
*/
int32_t lsm9ds1_gy_filter_lp_bandwidth_set(stmdev_ctx_t *ctx,
                                           lsm9ds1_gy_lp_bw_t val)
{
  lsm9ds1_ctrl_reg1_g_t ctrl_reg1_g;
  int32_t ret;
  
  ret = lsm9ds1_read_reg(ctx, LSM9DS1_CTRL_REG1_G, (uint8_t*)&ctrl_reg1_g, 1);
  if(ret == 0){
    ctrl_reg1_g.bw_g = (uint8_t)val;
    ret = lsm9ds1_write_reg(ctx, LSM9DS1_CTRL_REG1_G,
                            (uint8_t*)&ctrl_reg1_g, 1);
  }
  return ret;
}

/**
* @brief  Accelerometer output filter path configuration.[set]
*
* @param  ctx    Read / write interface definitions.(ptr)
* @param  val    Change the values of "fds" in reg LSM9DS1.
* @retval        Interface status (MANDATORY: return 0 -> no Error).
*
*/
int32_t lsm9ds1_xl_filter_out_path_set(stmdev_ctx_t *ctx,
                                       lsm9ds1_xl_out_path_t val)
{
  lsm9ds1_ctrl_reg7_xl_t ctrl_reg7_xl;
  int32_t ret;
  
  ret = lsm9ds1_read_reg(ctx, LSM9DS1_CTRL_REG7_XL,
                         (uint8_t*)&ctrl_reg7_xl, 1);
  if(ret == 0){
    ctrl_reg7_xl.fds = (uint8_t)val;
    ret = lsm9ds1_write_reg(ctx, LSM9DS1_CTRL_REG7_XL,
                            (uint8_t*)&ctrl_reg7_xl, 1);
  }
  return ret;
}

/**
* @brief  Accelerometer digital filter low pass cutoff frequency
*         selection.[set]
*
* @param  ctx    Read / write interface definitions.(ptr)
* @param  val    Change the values of "dcf" in reg LSM9DS1.
* @retval        Interface status (MANDATORY: return 0 -> no Error).
*
*/
int32_t lsm9ds1_xl_filter_lp_bandwidth_set(stmdev_ctx_t *ctx,
                                           lsm9ds1_xl_lp_bw_t val)
{
  lsm9ds1_ctrl_reg7_xl_t ctrl_reg7_xl;
  int32_t ret;
  
  ret = lsm9ds1_read_reg(ctx, LSM9DS1_CTRL_REG7_XL,
                         (uint8_t*)&ctrl_reg7_xl, 1);
  if(ret == 0){
    ctrl_reg7_xl.hr = ((uint8_t)val & 0x10U) >> 4;
    ctrl_reg7_xl.dcf = ((uint8_t)val & 0x03U);
    ret = lsm9ds1_write_reg(ctx, LSM9DS1_CTRL_REG7_XL,
                            (uint8_t*)&ctrl_reg7_xl, 1);
  }
  return ret;
}

/**
* @brief  Configure accelerometer anti aliasing filter.[set]
*
* @param  ctx    Read / write interface definitions.(ptr)
* @param  val    Change the values of "bw_xl" in reg LSM9DS1.
* @retval        Interface status (MANDATORY: return 0 -> no Error).
*
*/
int32_t lsm9ds1_xl_filter_aalias_bandwidth_set(stmdev_ctx_t *ctx,
                                               lsm9ds1_xl_aa_bw_t val)
{
  lsm9ds1_ctrl_reg6_xl_t ctrl_reg6_xl;
  int32_t ret;
  
  ret = lsm9ds1_read_reg(ctx, LSM9DS1_CTRL_REG6_XL,
                         (uint8_t*)&ctrl_reg6_xl, 1);
  if(ret == 0){
    ctrl_reg6_xl.bw_xl = ((uint8_t)val & 0x03U);
    ctrl_reg6_xl.bw_scal_odr = (((uint8_t)val & 0x10U) >> 4 );
    ret = lsm9ds1_write_reg(ctx, LSM9DS1_CTRL_REG6_XL,
                            (uint8_t*)&ctrl_reg6_xl, 1);
  }
  return ret;
}

/**
* @brief  Magnetometer full Scale Selection.[set]
*
* @param  ctx    Read / write interface definitions.(ptr)
* @param  val    Change the values of "fs" in reg LSM9DS1.
* @retval        Interface status (MANDATORY: return 0 -> no Error).
*
*/
int32_t lsm9ds1_mag_full_scale_set(stmdev_ctx_t *ctx, lsm9ds1_mag_fs_t val)
{
  lsm9ds1_ctrl_reg2_m_t ctrl_reg2_m;
  int32_t ret;
  
  ret = lsm9ds1_read_reg(ctx, LSM9DS1_CTRL_REG2_M, (uint8_t*)&ctrl_reg2_m, 1);
  if(ret == 0){
    ctrl_reg2_m.fs = (uint8_t)val;
    ret = lsm9ds1_write_reg(ctx, LSM9DS1_CTRL_REG2_M,
                            (uint8_t*)&ctrl_reg2_m, 1);
  }
  return ret;
}

/**
* @brief  Gyroscope full-scale selection.[set]
*
* @param  ctx    Read / write interface definitions.(ptr)
* @param  val    Change the values of "fs_g" in reg LSM9DS1.
* @retval        Interface status (MANDATORY: return 0 -> no Error).
*
*/
int32_t lsm9ds1_gy_full_scale_set(stmdev_ctx_t *ctx, lsm9ds1_gy_fs_t val)
{
  lsm9ds1_ctrl_reg1_g_t ctrl_reg1_g;
  int32_t ret;
  
  ret = lsm9ds1_read_reg(ctx, LSM9DS1_CTRL_REG1_G, (uint8_t*)&ctrl_reg1_g, 1);
  if(ret == 0){
    ctrl_reg1_g.fs_g = (uint8_t)val;
    ret = lsm9ds1_write_reg(ctx, LSM9DS1_CTRL_REG1_G,
                            (uint8_t*)&ctrl_reg1_g, 1);
  }
  return ret;
}

/**
* @brief  Accelerometer full-scale selection.[set]
*
* @param  ctx    Read / write interface definitions.(ptr)
* @param  val    Change the values of "fs_xl" in reg LSM9DS1.
* @retval        Interface status (MANDATORY: return 0 -> no Error).
*
*/
int32_t lsm9ds1_xl_full_scale_set(stmdev_ctx_t *ctx, lsm9ds1_xl_fs_t val)
{
  lsm9ds1_ctrl_reg6_xl_t ctrl_reg6_xl;
  int32_t ret;
  
  ret = lsm9ds1_read_reg(ctx, LSM9DS1_CTRL_REG6_XL,
                         (uint8_t*)&ctrl_reg6_xl, 1);
  if(ret == 0){
    ctrl_reg6_xl.fs_xl = (uint8_t)val;
    ret = lsm9ds1_write_reg(ctx, LSM9DS1_CTRL_REG6_XL,
                            (uint8_t*)&ctrl_reg6_xl, 1);
  }
  return ret;
}

/**
* @brief  Blockdataupdate.[set]
*
* @param  ctx_mag   Read / write magnetometer interface definitions.(ptr)
* @param  ctx_imu   Read / write imu interface definitions.(ptr)
* @param  val       Change the values of bdu in reg CTRL_REG8.
* @retval           Interface status (MANDATORY: return 0 -> no Error).
*
*/
int32_t lsm9ds1_block_data_update_set(stmdev_ctx_t *ctx_mag,
                                      stmdev_ctx_t *ctx_imu, uint8_t val)
{
  lsm9ds1_ctrl_reg8_t ctrl_reg8;
  lsm9ds1_ctrl_reg5_m_t ctrl_reg5_m;
  int32_t ret;
  
  ret = lsm9ds1_read_reg(ctx_imu, LSM9DS1_CTRL_REG8, (uint8_t*)&ctrl_reg8, 1);
  if(ret == 0){
    ctrl_reg8.bdu = (uint8_t)val;
    ret = lsm9ds1_write_reg(ctx_imu, LSM9DS1_CTRL_REG8, (uint8_t*)&ctrl_reg8, 1);
  }
  if(ret == 0){
    ret = lsm9ds1_read_reg(ctx_mag, LSM9DS1_CTRL_REG5_M,
                           (uint8_t*)&ctrl_reg5_m, 1);
  }
  if(ret == 0){
    ctrl_reg5_m.fast_read = (uint8_t)(~val);
    ctrl_reg5_m.bdu = (uint8_t)val;
    ret = lsm9ds1_write_reg(ctx_mag, LSM9DS1_CTRL_REG5_M,
                            (uint8_t*)&ctrl_reg5_m, 1);
  }
  
  return ret;
}

/**
* @brief  Software reset. Restore the default values in user registers.[set]
*
* @param  ctx_mag   Read / write magnetometer interface definitions.(ptr)
* @param  ctx_imu   Read / write imu interface definitions.(ptr)
* @param  val    Change the values of sw_reset in reg CTRL_REG8.
* @retval        Interface status (MANDATORY: return 0 -> no Error).
*
*/
int32_t lsm9ds1_dev_reset_set(stmdev_ctx_t *ctx_mag, stmdev_ctx_t *ctx_imu,
                              uint8_t val)
{
  lsm9ds1_ctrl_reg2_m_t ctrl_reg2_m;
  lsm9ds1_ctrl_reg8_t ctrl_reg8;
  int32_t ret;
  
  ret = lsm9ds1_read_reg(ctx_imu, LSM9DS1_CTRL_REG8, (uint8_t*)&ctrl_reg8, 1);
  if(ret == 0){
    ctrl_reg8.sw_reset = (uint8_t)val;
    ret = lsm9ds1_write_reg(ctx_imu, LSM9DS1_CTRL_REG8,
                            (uint8_t*)&ctrl_reg8, 1);
  }
  if(ret == 0){
    ret = lsm9ds1_read_reg(ctx_mag, LSM9DS1_CTRL_REG2_M,
                           (uint8_t*)&ctrl_reg2_m, 1);
  }
  if(ret == 0){
    ctrl_reg2_m.soft_rst = (uint8_t)val;
    ret = lsm9ds1_write_reg(ctx_mag, LSM9DS1_CTRL_REG2_M,
                            (uint8_t*)&ctrl_reg2_m, 1);
  }
  
  return ret;
}

/**
* @brief  Software reset. Restore the default values in user registers.[get]
*
* @param  ctx_mag   Read / write magnetometer interface definitions.(ptr)
* @param  ctx_imu   Read / write imu interface definitions.(ptr)
* @param  val       Get the values of sw_reset in reg CTRL_REG8.(ptr)
* @retval           Interface status (MANDATORY: return 0 -> no Error).
*
*/
int32_t lsm9ds1_dev_reset_get(stmdev_ctx_t *ctx_mag, stmdev_ctx_t *ctx_imu,
                              uint8_t *val)
{
  lsm9ds1_ctrl_reg2_m_t ctrl_reg2_m;
  lsm9ds1_ctrl_reg8_t ctrl_reg8;
  int32_t ret;
  
  ret = lsm9ds1_read_reg(ctx_imu, LSM9DS1_CTRL_REG8, (uint8_t*)&ctrl_reg8, 1);
  if(ret == 0){
    ret = lsm9ds1_read_reg(ctx_mag, LSM9DS1_CTRL_REG2_M,
                           (uint8_t*)&ctrl_reg2_m, 1);
    *val = (uint8_t)(ctrl_reg2_m.soft_rst & ctrl_reg8.sw_reset);
  }
  return ret;
}

/**
* @brief  DeviceWhoamI.[get]
*
* @param  ctx_mag   Read / write magnetometer interface definitions.(ptr)
* @param  ctx_imu   Read / write imu interface definitions.(ptr)
* @param  buff      Buffer that stores the data read.(ptr)
* @retval           Interface status (MANDATORY: return 0 -> no Error).
*
*/
int32_t lsm9ds1_dev_id_get(stmdev_ctx_t *ctx_mag, stmdev_ctx_t *ctx_imu,
                           lsm9ds1_id_t *buff)
{
  int32_t ret;  
  ret = lsm9ds1_read_reg(ctx_imu, LSM9DS1_WHO_AM_I,
                         (uint8_t*)&(buff->imu), 1);
  
  if(ret == RP_I2C_SUCCESS){
    ret = lsm9ds1_read_reg(ctx_mag, LSM9DS1_WHO_AM_I_M,
                           (uint8_t*)&(buff->mag), 1);
  }
  return ret;
}

/**
* @brief  Angular rate sensor. The value is expressed as a 16-bit word in
*         two’s complement.[get]
*
* @param  ctx    Read / write interface definitions.(ptr)
* @param  buff   Buffer that stores the data read.(ptr)
* @retval        Interface status (MANDATORY: return 0 -> no Error).
*
*/
int32_t lsm9ds1_angular_rate_raw_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lsm9ds1_read_reg(ctx, LSM9DS1_OUT_X_L_G, buff, 6);
  return ret;
}

/**
* @brief  Linear acceleration output register. The value is expressed as
*         a 16-bit word in two’s complement.[get]
*
* @param  ctx    Read / write interface definitions.(ptr)
* @param  buff   Buffer that stores the data read.(ptr)
* @retval        Interface status (MANDATORY: return 0 -> no Error).
*
*/
int32_t lsm9ds1_acceleration_raw_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lsm9ds1_read_reg(ctx, LSM9DS1_OUT_X_L_XL, buff, 6);
  return ret;
}

/**
* @brief  Magnetic sensor. The value is expressed as a 16-bit word in
*         two’s complement.[get]
*
* @param  ctx    Read / write interface definitions.(ptr)
* @param  buff   Buffer that stores the data read.(ptr)
* @retval        Interface status (MANDATORY: return 0 -> no Error).
*
*/
int32_t lsm9ds1_magnetic_raw_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lsm9ds1_read_reg(ctx, LSM9DS1_OUT_X_L_M, buff, 6);
  return ret;
}

/**
* @brief  Write generic device register (platform dependent)
*
* @param  handle    customizable argument. In this examples is used in
*                   order to select the correct sensor I2C address.
* @param  reg       register to write
* @param  bufp      pointer to data to write in register reg
* @param  len       number of consecutive register to write
*
*/
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len)
{
  uint8_t *i2c_address = handle;
  return RP_I2C_GenericWrite(*i2c_address, (uint16_t)reg, FALSE, bufp, len);
}

/**
* @brief  Read generic device register (platform dependent)
*
* @param  handle    customizable argument. In this examples is used in
*                   order to select the correct sensor I2C address.
* @param  reg       register to read
* @param  bufp      pointer to buffer that store the data read
* @param  len       number of consecutive register to read
*
*/
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  uint8_t *i2c_address = handle;
  return RP_I2C_GenericRead(*i2c_address, (uint16_t)reg, FALSE, bufp, len);
}

bool RP_I2C_WaitForEvent(I2C_Event_TypeDef I2C_Event){
  I2C_Timeout= RP_I2C_TIMEOUT_MAX;
  while(!I2C_CheckEvent(RP_I2C, I2C_Event) && --I2C_Timeout)
  {}
  return (bool)(I2C_Timeout == 0);
}

bool RP_I2C_WaitWhileFlag(I2C_FLAG_TypeDef I2C_Flag, bool flagState){
  I2C_Timeout = RP_I2C_TIMEOUT_MAX;
  while(((bool)I2C_GetFlagStatus(RP_I2C, I2C_Flag) == flagState) && --I2C_Timeout)
  {}
  return (bool)(I2C_Timeout == 0);
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
* @retval error
*/
bool RP_I2C_GenericRead(uint8_t SlaveAddr, uint16_t ReadAddr, bool ReadAddrIs2Bytes, uint8_t* pBuffer,  uint16_t Length)
{    
  if(Length == 0) return RP_I2C_FAILURE;
  
  /*!< While the bus is busy */
  if(RP_I2C_WaitWhileFlag(I2C_FLAG_BUSY, TRUE) != RP_I2C_SUCCESS){
    return RP_I2C_FAILURE;
  }
  
  /*!< Send START condition */
  I2C_GenerateSTART(RP_I2C, ENABLE);
  
  /*!< Test on EV5 and clear it */
  if(RP_I2C_WaitForEvent(I2C_EVENT_MASTER_MODE_SELECT) != RP_I2C_SUCCESS){
    return RP_I2C_FAILURE;
  }
  
  /*!< Send EEPROM address for write */
  I2C_Send7bitAddress(RP_I2C, SlaveAddr, I2C_Direction_Transmitter);
  
  /*!< Test on EV6 and clear it */
  if(RP_I2C_WaitForEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != RP_I2C_SUCCESS){
    return RP_I2C_FAILURE;
  }
  
  if(ReadAddrIs2Bytes){
    /*!< Send the slave's internal address to read from: MSB of the address first */
    I2C_SendData(RP_I2C, (uint8_t)((ReadAddr & 0xFF00) >> 8));
    
    /*!< Test on EV8 and clear it */
    if(RP_I2C_WaitForEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED) != RP_I2C_SUCCESS){
      return RP_I2C_FAILURE;
    }
  }
  
  /*!< Send the slave's internal address to read from: LSB of the address */
  I2C_SendData(RP_I2C, (uint8_t)(ReadAddr & 0x00FF));
  
  
  /*!< Test on EV8 and clear it */
  if(RP_I2C_WaitForEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED) != RP_I2C_SUCCESS){
    return RP_I2C_FAILURE;
  }
  
  /*!< Send START condition a second time */
  I2C_GenerateSTART(RP_I2C, ENABLE);
  
  /*!< Test on EV5 and clear it */
  if(RP_I2C_WaitForEvent(I2C_EVENT_MASTER_MODE_SELECT) != RP_I2C_SUCCESS){
    return RP_I2C_FAILURE;
  }
  
  /*!< Send slave address for read */
  I2C_Send7bitAddress(RP_I2C, SlaveAddr, I2C_Direction_Receiver);
  
  /*!<
  * The peripheral is shit, so always read at least three bytes.
  * There are supposedlyways that it works to read just 1 or 2, 
  * but they don't seem to work.
  * If fewer are needed, discard as necessary...
  */
  
  /*!< Test on EV6 and clear it */
  if(RP_I2C_WaitForEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) != RP_I2C_SUCCESS){
    return RP_I2C_FAILURE;
  }
  /*!< Receive bytes until 3 are left */
  while(Length > 3)
  {
    /*!< Test on EV7 and clear it */
    if(RP_I2C_WaitForEvent(I2C_EVENT_MASTER_BYTE_RECEIVED) != RP_I2C_SUCCESS){
      return RP_I2C_FAILURE;
    }
    
    /*!< Read a byte from the slave */
    *(pBuffer++) = I2C_ReceiveData(RP_I2C);
    Length--;
  }
  
  //Receive Data N-2
  /*!< Test on EV7 and clear it */
  if(RP_I2C_WaitForEvent(I2C_EVENT_MASTER_BYTE_RECEIVED) != RP_I2C_SUCCESS){
    return RP_I2C_FAILURE;
  }
  
  //Do nothing - leave shift register full
  
  
  //Receive Data N-1
  /*!< Wait for both data and shift registers to fill */
  if(RP_I2C_WaitWhileFlag(I2C_FLAG_BTF, FALSE) != RP_I2C_SUCCESS){
    return RP_I2C_FAILURE;
  }
  
  /*!< Disable Acknowledgement */
  I2C_AcknowledgeConfig(RP_I2C, DISABLE);
  
  //Read Data N-2
  /*!< Read a byte from the slave */
  *(pBuffer++) = I2C_ReceiveData(RP_I2C);
  Length--;
  
  //Receive Data N
  /*!< Test on EV7 and clear it */
  if(RP_I2C_WaitForEvent(I2C_EVENT_MASTER_BYTE_RECEIVED) != RP_I2C_SUCCESS){
    return RP_I2C_FAILURE;
  }
  
  /*!< Send STOP Condition */
  I2C_GenerateSTOP(RP_I2C, ENABLE);
  
  //Read Data N-1
  /*!< Read a byte from the slave */
  if(Length){
    *(pBuffer++) = I2C_ReceiveData(RP_I2C);
    Length--;
  } else I2C_ReceiveData(RP_I2C);
  
  /*!< Wait for shift register to populate data register */
  if(RP_I2C_WaitWhileFlag(I2C_FLAG_RXNE, FALSE) != RP_I2C_SUCCESS){
    return RP_I2C_FAILURE;
  }
  
  //Read Data N
  /*!< Read a byte from the slave */
  if(Length){
    *(pBuffer) = I2C_ReceiveData(RP_I2C);
    Length--;
  } else I2C_ReceiveData(RP_I2C);
  
  
  /*!< Enable Acknowledgement to be ready for another reception */
  I2C_AcknowledgeConfig(RP_I2C, ENABLE);
  
  return RP_I2C_SUCCESS;
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
  /*!< Send START condition */
  I2C_GenerateSTART(RP_I2C, ENABLE);
  
  /*!< Test on EV5 and clear it */
  while (!I2C_CheckEvent(RP_I2C, I2C_EVENT_MASTER_MODE_SELECT))
  {}
  
  /*!< Send slave address for write */
  I2C_Send7bitAddress(RP_I2C, SlaveAddr, I2C_Direction_Transmitter);
  
  /*!< Test on EV6 and clear it */
  while (!I2C_CheckEvent(RP_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
  {}
  
  if(WriteAddrIs2Bytes){
    /*!< Send the slave's internal address to write to : MSB of the address first */
    I2C_SendData(RP_I2C, (uint8_t)((WriteAddr & 0xFF00) >> 8));
    
    /*!< Test on EV8 and clear it */
    while (!I2C_CheckEvent(RP_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {}
  }
  /*!< Send the slave's internal address to write to : LSB of the address */
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
bool RP_I2C_GenericWrite(uint8_t SlaveAddr, uint16_t WriteAddr, bool WriteAddrIs2Bytes, uint8_t* pBuffer, uint16_t Length)
{    
  /*!< While the bus is busy */
  if(RP_I2C_WaitWhileFlag(I2C_FLAG_BUSY, TRUE) != RP_I2C_SUCCESS){
    return RP_I2C_FAILURE;
  }
  
  /*!< Send START condition */
  I2C_GenerateSTART(RP_I2C, ENABLE);
  
  /*!< Test on EV5 and clear it */
  if(RP_I2C_WaitForEvent(I2C_EVENT_MASTER_MODE_SELECT) != RP_I2C_SUCCESS)
    return RP_I2C_FAILURE;
  
  /*!< Send slave address for write */
  I2C_Send7bitAddress(RP_I2C, SlaveAddr, I2C_Direction_Transmitter);
  
  /*!< Test on EV6 and clear it */
  if(RP_I2C_WaitForEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != RP_I2C_SUCCESS)
    return RP_I2C_FAILURE;
  
  if(WriteAddrIs2Bytes){
    /*!< Send the slave's internal address to write to : MSB of the address first */
    I2C_SendData(RP_I2C, (uint8_t)((WriteAddr & 0xFF00) >> 8));
    
    /*!< Test on EV8 and clear it */
    if(RP_I2C_WaitForEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED) != RP_I2C_SUCCESS)
      return RP_I2C_FAILURE;
  }
  
  /*!< Send the slave's internal address to write to : LSB of the address */
  I2C_SendData(RP_I2C, (uint8_t)(WriteAddr & 0x00FF));
  
  while(Length)
  {
    /*!< Test on EV8 and clear it */
    if(RP_I2C_WaitForEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED) != RP_I2C_SUCCESS)
      return RP_I2C_FAILURE;
    
    /*!< Send the byte to be written */
    I2C_SendData(RP_I2C, *(pBuffer++));        
    Length--;
  }
  
  /*!< Send STOP condition */
  I2C_GenerateSTOP(RP_I2C, ENABLE);
  
  return RP_I2C_SUCCESS;
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
bool RP_EE_ReadBuffer(uint32_t ReadAddr, uint8_t* pBuffer, uint16_t Length)
{
  eeAddress = EE_ADDRESS(ReadAddr);
  if(Length == 0 || RP_EE_WaitForStandby() != RP_I2C_SUCCESS) return RP_I2C_FAILURE;
  return RP_I2C_GenericRead(eeAddress, (uint16_t)ReadAddr, TRUE, pBuffer, Length);
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

bool RP_EE_WriteBuffer_Delayed(uint8_t* pBuffer, uint16_t Length){
  if(Length == 0) return RP_I2C_FAILURE;
  else if(Length <= EE_PAGESIZE - EE_PageBuffer_ind){
    /* Add to page buffer*/
    memcpy(&EE_PageBuffer[EE_PageBuffer_ind], pBuffer, Length);
    EE_PageBuffer_ind += Length;
  } else {
    uint16_t remainder = Length - (EE_PAGESIZE - EE_PageBuffer_ind); //what won't fit on this page
    
    /* Add what fits to page buffer*/
    memcpy(&EE_PageBuffer[EE_PageBuffer_ind], pBuffer, Length - remainder);
    EE_PageBuffer_ind   += Length - remainder;
    pBuffer             += Length - remainder;
    
    /* Flush page buffer to EE*/
    if(RP_EE_Flush()!= RP_I2C_SUCCESS) return RP_I2C_FAILURE;
    
    /* Add rest to page buffer*/
    memcpy(&EE_PageBuffer[EE_PageBuffer_ind], pBuffer, remainder);
    EE_PageBuffer_ind += remainder;
  }
  return RP_I2C_SUCCESS;
}

bool RP_EE_Flush(void){
  if(EE_PageBuffer_ind != EE_PAGESIZE){
    memset(&EE_PageBuffer[EE_PageBuffer_ind], EE_FILLER_DATA_BYTE, EE_PAGESIZE - EE_PageBuffer_ind);
  }
  
  if(RP_EE_WriteBuffer(EE_PageBuffer_Address, EE_PageBuffer, EE_PAGESIZE) != RP_I2C_SUCCESS) return RP_I2C_FAILURE;
  EE_PageBuffer_ind = 0;
  EE_PageBuffer_Address += EE_PAGESIZE;
  return RP_I2C_SUCCESS;
}

/**
* @brief  Writes buffer of data to the I2C EEPROM.
* @param  pBuffer : pointer to the buffer  containing the data to be written
*         to the EEPROM.
* @param  WriteAddr : EEPROM's internal address to write to.
* @param  Length : number of bytes to write to the EEPROM.
* @retval err
*/
bool RP_EE_WriteBuffer(uint32_t WriteAddr, uint8_t* pBuffer, uint16_t Length)
{
  if(Length == 0) return RP_I2C_FAILURE;
  
  eeAddress = EE_ADDRESS(WriteAddr);
  
  uint8_t NumOfPage = 0; //the number of whole pages covered by this write
  uint8_t NumOfSingle = 0; //the number of bytes that will not fit into a whole page
  uint8_t PageRemaining = 0; //the number of bytes remaining in the first page 
  
  // Note: PageRemaining will never be 0 (range from 1 to EE_PAGESIZE) 
  PageRemaining = (uint8_t)(EE_PAGESIZE - (WriteAddr % EE_PAGESIZE)); 
  
  if(Length < PageRemaining){
    // All the data will fit on this page - just write it
    RP_EE_WritePage(pBuffer, (uint16_t)WriteAddr, Length);
  }
  /*!< If Length >= PageRemaining */
  else{
    // Write until the end of this page
  if(RP_EE_WaitForStandby() != RP_I2C_SUCCESS) return RP_I2C_FAILURE;
    RP_EE_WritePage(pBuffer, (uint16_t)WriteAddr, PageRemaining);
    
    // Update pointers, now page aligned
    WriteAddr += PageRemaining;
    eeAddress = EE_ADDRESS(WriteAddr);
    pBuffer += PageRemaining;
    
    // Update length and calculate number of pages and single bytes
    Length -= PageRemaining;
    NumOfPage = (uint8_t)(Length / EE_PAGESIZE);
    NumOfSingle = (uint8_t)(Length % EE_PAGESIZE);
    
    // Transfer any remaining pages (could be zero, if Length == PageRemaining)
    while (NumOfPage--)
    {
  if(RP_EE_WaitForStandby() != RP_I2C_SUCCESS) return RP_I2C_FAILURE;
      RP_EE_WritePage(pBuffer, (uint16_t)WriteAddr, EE_PAGESIZE);
      
      /* Move the pointers forward */
      WriteAddr +=  EE_PAGESIZE;
      eeAddress = EE_ADDRESS(WriteAddr);
      pBuffer += EE_PAGESIZE;
    }
    
    //Transfer any remaining single bytes
    if (NumOfSingle != 0)
    {
  if(RP_EE_WaitForStandby() != RP_I2C_SUCCESS) return RP_I2C_FAILURE;
      RP_EE_WritePage(pBuffer, (uint16_t)WriteAddr, NumOfSingle);
    }
  }
  return RP_I2C_SUCCESS;
}

bool RP_EE_WaitForStandby(void){
  uint8_t retries = EE_STANDBY_TIMEOUT;
  do
  {
    /*!< Send START condition */
    I2C_GenerateSTART(RP_I2C, ENABLE);

    /* Test on EEPROM_I2C EV5 and clear it */
    if(RP_I2C_WaitWhileFlag(I2C_FLAG_SB, FALSE) != RP_I2C_SUCCESS)
      return RP_I2C_FAILURE;

    /*!< Send EEPROM address for write */
    I2C_Send7bitAddress(RP_I2C, eeAddress, I2C_Direction_Transmitter);

  }
  while(RP_I2C_WaitWhileFlag(I2C_FLAG_ADDR,FALSE) == RP_I2C_FAILURE && retries--); // Retry if timeout while waiting for address ack

  /*!< Clear AF flag */
  I2C_ClearFlag(RP_I2C, I2C_FLAG_AF);

  /*!< STOP condition */
  I2C_GenerateSTOP(RP_I2C, ENABLE);  
  
  return RP_I2C_SUCCESS;
}

#ifdef RP_USE_DMA
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
    /*!< Disable Acknowledgement */
    I2C_AcknowledgeConfig(RP_I2C, DISABLE);
    
    /*!< Send STOP Condition */
    I2C_GenerateSTOP(RP_I2C, ENABLE);
    
    /*!< Re-Enable Acknowledgement */
    I2C_AcknowledgeConfig(RP_I2C, ENABLE);
    
    /* Disable the DMA Rx Channel and Clear all its Flags */
    DMA_Cmd(RP_I2C_DMA_CHANNEL_RX, DISABLE);
    DMA_ClearFlag(RP_I2C_DMA_FLAG_RX_TC);
    
    /* Reset the variable holding the number of data to be read */
    *I2C_DataRemainingPointer = 0;
  }
}
#endif


