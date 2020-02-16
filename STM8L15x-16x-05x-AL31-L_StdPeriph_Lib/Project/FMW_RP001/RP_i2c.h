/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RP_I2C
#define __RP_I2C

/* Includes ------------------------------------------------------------------*/
#include "stm8_eval.h"

/* Private types -------------------------------------------------------------*/
typedef enum {RP_DIRECTION_TX, RP_DIRECTION_RX} RP_DIRECTION;
typedef enum {EE_STATE_READY, EE_STATE_BUSY} EE_STATE;

/* Private define ------------------------------------------------------------*/
  
#define EE_SLAVE_ADDRESS     0xA0
#define IMU_SLAVE_ADDRESS    0xD4
#define MAG_SLAVE_ADDRESS    0x38
#define OWN_SLAVE_ADDRESS    0xF0

#define RP_I2C_SPEED          100000

#define EE_PAGESIZE    128 //actually 256, but the DMA can't handle that
 
/* Defintions for the state of the DMA transfer */   
#define sEE_STATE_READY         0
#define sEE_STATE_BUSY          1

   
/* Maximum timeout value for counting before exiting waiting loop on DMA 
   Trasnfer Complete. This value depends directly on the maximum page size and
   the sytem clock frequency. */
#define RP_I2C_TIMEOUT_MAX         0x8000 
#define RP_I2C_SUCCESS             FALSE // i.e. no error
#define RP_I2C_FAILURE             TRUE  // i.e. error

/**
  * @brief  I2C Interface pins
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
//#define RP_USE_DMA

  
/* Public Functions -------------------------------------------------------------*/
void RP_I2C_DeInit(void);
bool RP_I2C_Init(void);
bool RP_I2C_WaitForEvent(I2C_TypeDef* I2Cx, I2C_Event_TypeDef I2C_Event);
bool RP_I2C_WaitWhileFlag(I2C_TypeDef* I2Cx, I2C_FLAG_TypeDef I2C_Flag, bool flagState);
bool RP_I2C_GenericRead(uint8_t SlaveAddr, uint16_t ReadAddr, bool ReadAddrIs2Bytes, uint8_t* pBuffer,  uint16_t Length);
bool RP_I2C_GenericWrite(uint8_t SlaveAddr, uint16_t WriteAddr, bool WriteAddrIs2Bytes, uint8_t* pBuffer, uint16_t Length);
void RP_I2C_GenericWriteByte(uint8_t SlaveAddr, uint16_t WriteAddr, bool WriteAddrIs2Bytes, uint8_t Buffer);

#ifdef RP_USE_DMA
void RP_I2C_DMA_RX_IRQHandler(void);
void RP_I2C_DMA_TX_IRQHandler(void);
#endif

void RP_EE_WriteByte(uint32_t WriteAddr, uint8_t Buffer);
bool RP_EE_WriteBuffer(uint32_t WriteAddr, uint8_t* pBuffer, uint16_t Length);
bool RP_EE_ReadBuffer(uint32_t ReadAddr, uint8_t* pBuffer, uint16_t Length);


/* IMU/MAG Typedefs -------------------------------------------------------------*/
#define PROPERTY_DISABLE                (0U)
#define PROPERTY_ENABLE                 (1U)

typedef union{
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;

typedef int32_t (*stmdev_write_ptr)(void *, uint8_t, uint8_t*, uint16_t);
typedef int32_t (*stmdev_read_ptr) (void *, uint8_t, uint8_t*, uint16_t);
typedef struct {
  uint8_t imu;
  uint8_t mag;
} lsm9ds1_id_t;

typedef struct {
  /** Component mandatory fields **/
  stmdev_write_ptr  write_reg;
  stmdev_read_ptr   read_reg;
  /** Customizable optional pointer **/
  void *handle;
} stmdev_ctx_t;

/** Device Identification (Who am I) **/
#define LSM9DS1_IMU_ID             0x68U

/** Device Identification (Who am I) **/
#define LSM9DS1_MAG_ID             0x3DU

#define LSM9DS1_ACT_THS            0x04U
typedef struct {
  uint8_t act_ths                  : 7;
  uint8_t sleep_on_inact_en        : 1;
} lsm9ds1_act_ths_t;

#define LSM9DS1_ACT_DUR            0x05U
#define LSM9DS1_INT_GEN_CFG_XL     0x06U
typedef struct {
  uint8_t xlie_xl                  : 1;
  uint8_t xhie_xl                  : 1;
  uint8_t ylie_xl                  : 1;
  uint8_t yhie_xl                  : 1;
  uint8_t zlie_xl                  : 1;
  uint8_t zhie_xl                  : 1;
  uint8_t _6d                      : 1;
  uint8_t aoi_xl                   : 1;
} lsm9ds1_int_gen_cfg_xl_t;

#define LSM9DS1_INT_GEN_THS_X_XL   0x07U
#define LSM9DS1_INT_GEN_THS_Y_XL   0x08U
#define LSM9DS1_INT_GEN_THS_Z_XL   0x09U
#define LSM9DS1_INT_GEN_DUR_XL     0x0AU
typedef struct {
  uint8_t dur_xl                   : 7;
  uint8_t wait_xl                  : 1;
} lsm9ds1_int_gen_dur_xl_t;

#define LSM9DS1_REFERENCE_G        0x0BU
#define LSM9DS1_INT1_CTRL          0x0CU
typedef struct {
  uint8_t int1_drdy_xl             : 1;
  uint8_t int1_drdy_g              : 1;
  uint8_t int1_boot                : 1;
  uint8_t int1_fth                 : 1;
  uint8_t int1_ovr                 : 1;
  uint8_t int1_fss5                : 1;
  uint8_t int1_ig_xl               : 1;
  uint8_t int1_ig_g                : 1;
} lsm9ds1_int1_ctrl_t;

#define LSM9DS1_INT2_CTRL          0x0DU
typedef struct {
  uint8_t int2_drdy_xl             : 1;
  uint8_t int2_drdy_g              : 1;
  uint8_t int2_drdy_temp           : 1;
  uint8_t int2_fth                 : 1;
  uint8_t int2_ovr                 : 1;
  uint8_t int2_fss5                : 1;
  uint8_t not_used_01              : 1;
  uint8_t int2_inact               : 1;
} lsm9ds1_int2_ctrl_t;

#define LSM9DS1_WHO_AM_I           0x0FU
#define LSM9DS1_CTRL_REG1_G        0x10U
typedef struct {
  uint8_t bw_g                     : 2;
  uint8_t not_used_01              : 1;
  uint8_t fs_g                     : 2;
  uint8_t odr_g                    : 3;
} lsm9ds1_ctrl_reg1_g_t;

#define LSM9DS1_CTRL_REG2_G        0x11U
typedef struct {
  uint8_t out_sel                  : 2;
  uint8_t int_sel                  : 2;
  uint8_t not_used_01              : 4;
} lsm9ds1_ctrl_reg2_g_t;

#define LSM9DS1_CTRL_REG3_G        0x12U
typedef struct {
  uint8_t hpcf_g                   : 4;
  uint8_t not_used_01              : 2;
  uint8_t hp_en                    : 1;
  uint8_t lp_mode                  : 1;
} lsm9ds1_ctrl_reg3_g_t;

#define LSM9DS1_ORIENT_CFG_G       0x13U
typedef struct {
  uint8_t orient                   : 3;
  uint8_t signz_g                  : 1;
  uint8_t signy_g                  : 1;
  uint8_t signx_g                  : 1;
  uint8_t not_used_01              : 2;
} lsm9ds1_orient_cfg_g_t;

#define LSM9DS1_INT_GEN_SRC_G      0x14U
typedef struct {
  uint8_t xl_g                     : 1;
  uint8_t xh_g                     : 1;
  uint8_t yl_g                     : 1;
  uint8_t yh_g                     : 1;
  uint8_t zl_g                     : 1;
  uint8_t zh_g                     : 1;
  uint8_t ia_g                     : 1;
  uint8_t not_used_01              : 1;
} lsm9ds1_int_gen_src_g_t;

#define LSM9DS1_OUT_TEMP_L         0x15U
#define LSM9DS1_OUT_TEMP_H         0x16U
#define LSM9DS1_STATUS_REG         0x17U
typedef struct {
  uint8_t xlda                     : 1;
  uint8_t gda                      : 1;
  uint8_t tda                      : 1;
  uint8_t boot_status              : 1;
  uint8_t inact                    : 1;
  uint8_t ig_g                     : 1;
  uint8_t ig_xl                    : 1;
  uint8_t not_used_01              : 1;
} lsm9ds1_status_reg_t;

#define LSM9DS1_OUT_X_L_G          0x18U
#define LSM9DS1_OUT_X_H_G          0x19U
#define LSM9DS1_OUT_Y_L_G          0x1AU
#define LSM9DS1_OUT_Y_H_G          0x1BU
#define LSM9DS1_OUT_Z_L_G          0x1CU
#define LSM9DS1_OUT_Z_H_G          0x1DU
#define LSM9DS1_CTRL_REG4          0x1EU
typedef struct {
  uint8_t _4d_xl1                   : 1;
  uint8_t lir_xl1                  : 1;
  uint8_t not_used_01              : 1;
  uint8_t xen_g                    : 1;
  uint8_t yen_g                    : 1;
  uint8_t zen_g                    : 1;
  uint8_t not_used_02              : 2;
} lsm9ds1_ctrl_reg4_t;

#define LSM9DS1_CTRL_REG5_XL       0x1FU
typedef struct {
  uint8_t not_used_01              : 3;
  uint8_t xen_xl                   : 1;
  uint8_t yen_xl                   : 1;
  uint8_t zen_xl                   : 1;
  uint8_t dec                      : 2;
} lsm9ds1_ctrl_reg5_xl_t;

#define LSM9DS1_CTRL_REG6_XL       0x20U
typedef struct {
  uint8_t bw_xl                    : 2;
  uint8_t bw_scal_odr              : 1;
  uint8_t fs_xl                    : 2;
  uint8_t odr_xl                   : 3;
} lsm9ds1_ctrl_reg6_xl_t;

#define LSM9DS1_CTRL_REG7_XL       0x21U
typedef struct {
  uint8_t hpis1                    : 1;
  uint8_t not_used_01              : 1;
  uint8_t fds                      : 1;
  uint8_t not_used_02              : 2;
  uint8_t dcf                      : 2;
  uint8_t hr                       : 1;
} lsm9ds1_ctrl_reg7_xl_t;

#define LSM9DS1_CTRL_REG8          0x22U
typedef struct {
  uint8_t sw_reset                 : 1;
  uint8_t ble                      : 1;
  uint8_t if_add_inc               : 1;
  uint8_t sim                      : 1;
  uint8_t pp_od                    : 1;
  uint8_t h_lactive                : 1;
  uint8_t bdu                      : 1;
  uint8_t boot                     : 1;
} lsm9ds1_ctrl_reg8_t;

#define LSM9DS1_CTRL_REG9          0x23U
typedef struct {
  uint8_t stop_on_fth              : 1;
  uint8_t fifo_en                  : 1;
  uint8_t i2c_disable              : 1;
  uint8_t drdy_mask_bit            : 1;
  uint8_t fifo_temp_en             : 1;
  uint8_t not_used_01              : 1;
  uint8_t sleep_g                  : 1;
  uint8_t not_used_02              : 1;
} lsm9ds1_ctrl_reg9_t;

#define LSM9DS1_CTRL_REG10         0x24U
typedef struct {
  uint8_t st_xl                    : 1;
  uint8_t not_used_01              : 1;
  uint8_t st_g                     : 1;
  uint8_t not_used_02              : 5;
} lsm9ds1_ctrl_reg10_t;

#define LSM9DS1_INT_GEN_SRC_XL     0x26U
typedef struct {
  uint8_t xl_xl                    : 1;
  uint8_t xh_xl                    : 1;
  uint8_t yl_xl                    : 1;
  uint8_t yh_xl                    : 1;
  uint8_t zl_xl                    : 1;
  uint8_t zh_xl                    : 1;
  uint8_t ia_xl                    : 1;
  uint8_t not_used_01              : 1;
} lsm9ds1_int_gen_src_xl_t;

#define LSM9DS1_OUT_X_L_XL         0x28U
#define LSM9DS1_OUT_X_H_XL         0x29U
#define LSM9DS1_OUT_Y_L_XL         0x2AU
#define LSM9DS1_OUT_Y_H_XL         0x2BU
#define LSM9DS1_OUT_Z_L_XL         0x2CU
#define LSM9DS1_OUT_Z_H_XL         0x2DU
#define LSM9DS1_FIFO_CTRL          0x2EU
typedef struct {
  uint8_t fth                      : 5;
  uint8_t fmode                    : 3;
} lsm9ds1_fifo_ctrl_t;

#define LSM9DS1_FIFO_SRC           0x2FU
typedef struct {
  uint8_t fss                      : 6;
  uint8_t ovrn                     : 1;
  uint8_t fth                      : 1;
} lsm9ds1_fifo_src_t;

#define LSM9DS1_INT_GEN_CFG_G      0x30U
typedef struct {
  uint8_t xlie_g                   : 1;
  uint8_t xhie_g                   : 1;
  uint8_t ylie_g                   : 1;
  uint8_t yhie_g                   : 1;
  uint8_t zlie_g                   : 1;
  uint8_t zhie_g                   : 1;
  uint8_t lir_g                    : 1;
  uint8_t aoi_g                    : 1;
} lsm9ds1_int_gen_cfg_g_t;

#define LSM9DS1_INT_GEN_THS_XH_G   0x31U
typedef struct {
  uint8_t ths_g_x                  : 7;
  uint8_t dcrm_g                   : 1;
} lsm9ds1_int_gen_ths_xh_g_t;

#define LSM9DS1_INT_GEN_THS_XL_G   0x32U
typedef struct {
  uint8_t ths_g_x                  : 8;
} lsm9ds1_int_gen_ths_xl_g_t;
#define LSM9DS1_INT_GEN_THS_YH_G   0x33U
typedef struct {
  uint8_t ths_g_y                  : 7;
  uint8_t not_used_01              : 1;
} lsm9ds1_int_gen_ths_yh_g_t;

#define LSM9DS1_INT_GEN_THS_YL_G   0x34U
typedef struct {
  uint8_t ths_g_y                  : 8;
} lsm9ds1_int_gen_ths_yl_g_t;
#define LSM9DS1_INT_GEN_THS_ZH_G   0x35U
typedef struct {
  uint8_t ths_g_z                  : 7;
  uint8_t not_used_01              : 1;
} lsm9ds1_int_gen_ths_zh_g_t;

#define LSM9DS1_INT_GEN_THS_ZL_G   0x36U
typedef struct {
  uint8_t ths_g_z                  : 8;
} lsm9ds1_int_gen_ths_zl_g_t;
#define LSM9DS1_INT_GEN_DUR_G      0x37U
typedef struct {
  uint8_t dur_g                    : 7;
  uint8_t wait_g                   : 1;
} lsm9ds1_int_gen_dur_g_t;

#define LSM9DS1_OFFSET_X_REG_L_M   0x05U
#define LSM9DS1_OFFSET_X_REG_H_M   0x06U
#define LSM9DS1_OFFSET_Y_REG_L_M   0x07U
#define LSM9DS1_OFFSET_Y_REG_H_M   0x08U
#define LSM9DS1_OFFSET_Z_REG_L_M   0x09U
#define LSM9DS1_OFFSET_Z_REG_H_M   0x0AU

#define LSM9DS1_WHO_AM_I_M         0x0FU
#define LSM9DS1_CTRL_REG1_M        0x20U
typedef struct {
  uint8_t st                       : 1;
  uint8_t fast_odr                 : 1;
  uint8_t _do                       : 3;
  uint8_t om                       : 2;
  uint8_t temp_comp                : 1;
} lsm9ds1_ctrl_reg1_m_t;

#define LSM9DS1_CTRL_REG2_M        0x21U
typedef struct {
  uint8_t not_used_01              : 2;
  uint8_t soft_rst                 : 1;
  uint8_t reboot                   : 1;
  uint8_t not_used_02              : 1;
  uint8_t fs                       : 2;
  uint8_t not_used_03              : 1;
} lsm9ds1_ctrl_reg2_m_t;

#define LSM9DS1_CTRL_REG3_M        0x22U
typedef struct {
  uint8_t md                       : 2;
  uint8_t sim                      : 1;
  uint8_t not_used_01              : 2;
  uint8_t lp                       : 1;
  uint8_t not_used_02              : 1;
  uint8_t i2c_disable              : 1;
} lsm9ds1_ctrl_reg3_m_t;

#define LSM9DS1_CTRL_REG4_M        0x23U
typedef struct {
  uint8_t not_used_01              : 1;
  uint8_t ble                      : 1;
  uint8_t omz                      : 2;
  uint8_t not_used_02              : 4;
} lsm9ds1_ctrl_reg4_m_t;

#define LSM9DS1_CTRL_REG5_M        0x24U
typedef struct {
  uint8_t not_used_01              : 6;
  uint8_t bdu                      : 1;
  uint8_t fast_read                : 1;
} lsm9ds1_ctrl_reg5_m_t;

#define LSM9DS1_STATUS_REG_M       0x27U
typedef struct {
  uint8_t xda                      : 1;
  uint8_t yda                      : 1;
  uint8_t zda                      : 1;
  uint8_t zyxda                    : 1;
  uint8_t _xor                     : 1;
  uint8_t yor                      : 1;
  uint8_t zor                      : 1;
  uint8_t zyxor                    : 1;
} lsm9ds1_status_reg_m_t;

#define LSM9DS1_OUT_X_L_M          0x28U
#define LSM9DS1_OUT_X_H_M          0x29U
#define LSM9DS1_OUT_Y_L_M          0x2AU
#define LSM9DS1_OUT_Y_H_M          0x2BU
#define LSM9DS1_OUT_Z_L_M          0x2CU
#define LSM9DS1_OUT_Z_H_M          0x2DU
#define LSM9DS1_INT_CFG_M          0x30U
typedef struct {
  uint8_t ien                      : 1;
  uint8_t iel                      : 1;
  uint8_t iea                      : 1;
  uint8_t not_used_01              : 2;
  uint8_t zien                     : 1;
  uint8_t yien                     : 1;
  uint8_t xien                     : 1;
} lsm9ds1_int_cfg_m_t;

#define LSM9DS1_INT_SRC_M          0x31U
typedef struct {
  uint8_t _int                      : 1;
  uint8_t mroi                     : 1;
  uint8_t nth_z                    : 1;
  uint8_t nth_y                    : 1;
  uint8_t nth_x                    : 1;
  uint8_t pth_z                    : 1;
  uint8_t pth_y                    : 1;
  uint8_t pth_x                    : 1;
} lsm9ds1_int_src_m_t;

#define LSM9DS1_INT_THS_L_M        0x32U
typedef struct {
  uint8_t ths                      : 8;
} lsm9ds1_int_ths_l_m_t;

#define LSM9DS1_INT_THS_H_M        0x33U
typedef struct {
  uint8_t ths                      : 7;
  uint8_t not_used_01              : 1;
} lsm9ds1_int_ths_h_m_t;

typedef enum {
  LSM9DS1_2g     = 0,
  LSM9DS1_16g    = 1,
  LSM9DS1_4g     = 2,
  LSM9DS1_8g     = 3,
} lsm9ds1_xl_fs_t;

typedef enum {
  LSM9DS1_245dps = 0,
  LSM9DS1_500dps = 1,
  LSM9DS1_2000dps = 3,
} lsm9ds1_gy_fs_t;

typedef enum {
  LSM9DS1_4Ga    = 0,
  LSM9DS1_8Ga    = 1,
  LSM9DS1_12Ga   = 2,
  LSM9DS1_16Ga   = 3,
} lsm9ds1_mag_fs_t;

typedef enum {
  LSM9DS1_AUTO      = 0x00,
  LSM9DS1_408Hz     = 0x10,
  LSM9DS1_211Hz     = 0x11,
  LSM9DS1_105Hz     = 0x12,
  LSM9DS1_50Hz      = 0x13,
} lsm9ds1_xl_aa_bw_t;

typedef enum {
  LSM9DS1_LP_DISABLE     = 0x00,
  LSM9DS1_LP_ODR_DIV_50     = 0x10,
  LSM9DS1_LP_ODR_DIV_100    = 0x11,
  LSM9DS1_LP_ODR_DIV_9      = 0x12,
  LSM9DS1_LP_ODR_DIV_400    = 0x13,
} lsm9ds1_xl_lp_bw_t;

typedef enum {
  LSM9DS1_LP_OUT     = 0,
  LSM9DS1_HP_OUT     = 1,
} lsm9ds1_xl_out_path_t;

typedef enum {
  LSM9DS1_LP_STRONG        = 0,
  LSM9DS1_LP_MEDIUM        = 1,
  LSM9DS1_LP_LIGHT         = 2,
  LSM9DS1_LP_ULTRA_LIGHT   = 3,
} lsm9ds1_gy_lp_bw_t;

typedef enum {
  LSM9DS1_HP_EXTREME       = 0,
  LSM9DS1_HP_ULTRA_STRONG  = 1,
  LSM9DS1_HP_STRONG        = 2,
  LSM9DS1_HP_ULTRA_HIGH    = 3,
  LSM9DS1_HP_HIGH          = 4,
  LSM9DS1_HP_MEDIUM        = 5,
  LSM9DS1_HP_LOW           = 6,
  LSM9DS1_HP_ULTRA_LOW     = 7,
  LSM9DS1_HP_LIGHT         = 8,
  LSM9DS1_HP_ULTRA_LIGHT   = 9,
} lsm9ds1_gy_hp_bw_t;
/*
  High-pass filter cutoff frequency settings
		-------------------- ODR --------------------
HPCF_G [3:0]	14.9Hz	59.5Hz	119Hz	238Hz	476Hz	952Hz
-------------------------------------------------------------
0000		1 	4 	8	15	30 	57
0001 		0.5 	2 	4 	8 	15 	30
0010		0.2	1	2	4	8	15
0011		0.1	0.5	1	2	4	8
0100		0.05	0.2	0.5	1	2	4
0101		0.02	0.1	0.2	0.5	1	2
0110		0.01	0.05	0.1	0.2	0.5	1
0111		0.005	0.02	0.05	0.1	0.2	0.5
1000		0.002	0.01	0.02	0.05	0.1	0.2
1001		0.001	0.005	0.01	0.02	0.05	0.1
*/

typedef enum {
  LSM9DS1_LPF1_OUT              = 0x00,
  LSM9DS1_LPF1_HPF_OUT          = 0x01,
  LSM9DS1_LPF1_LPF2_OUT         = 0x02,
  LSM9DS1_LPF1_HPF_LPF2_OUT     = 0x12,
} lsm9ds1_gy_out_path_t;

typedef enum {
  LSM9DS1_IMU_OFF              = 0x00,
  LSM9DS1_GY_OFF_XL_10Hz       = 0x10,
  LSM9DS1_GY_OFF_XL_50Hz       = 0x20,
  LSM9DS1_GY_OFF_XL_119Hz      = 0x30,
  LSM9DS1_GY_OFF_XL_238Hz      = 0x40,
  LSM9DS1_GY_OFF_XL_476Hz      = 0x50,
  LSM9DS1_GY_OFF_XL_952Hz      = 0x60,
  LSM9DS1_XL_OFF_GY_14Hz9      = 0x01,
  LSM9DS1_XL_OFF_GY_59Hz5      = 0x02,
  LSM9DS1_XL_OFF_GY_119Hz      = 0x03,
  LSM9DS1_XL_OFF_GY_238Hz      = 0x04,
  LSM9DS1_XL_OFF_GY_476Hz      = 0x05,
  LSM9DS1_XL_OFF_GY_952Hz      = 0x06,
  LSM9DS1_IMU_14Hz9            = 0x11,
  LSM9DS1_IMU_59Hz5            = 0x22,
  LSM9DS1_IMU_119Hz            = 0x33,
  LSM9DS1_IMU_238Hz            = 0x44,
  LSM9DS1_IMU_476Hz            = 0x55,
  LSM9DS1_IMU_952Hz            = 0x66,
  LSM9DS1_XL_OFF_GY_14Hz9_LP   = 0x81,
  LSM9DS1_XL_OFF_GY_59Hz5_LP   = 0x82,
  LSM9DS1_XL_OFF_GY_119Hz_LP   = 0x83,
  LSM9DS1_IMU_14Hz9_LP         = 0x91,
  LSM9DS1_IMU_59Hz5_LP         = 0xA2,
  LSM9DS1_IMU_119Hz_LP         = 0xB3,
} lsm9ds1_imu_odr_t;

typedef enum {
  LSM9DS1_MAG_POWER_DOWN    = 0xC0,
  LSM9DS1_MAG_LP_0Hz625     = 0x00,
  LSM9DS1_MAG_LP_1Hz25      = 0x01,
  LSM9DS1_MAG_LP_2Hz5       = 0x02,
  LSM9DS1_MAG_LP_5Hz        = 0x03,
  LSM9DS1_MAG_LP_10Hz       = 0x04,
  LSM9DS1_MAG_LP_20Hz       = 0x05,
  LSM9DS1_MAG_LP_40Hz       = 0x06,
  LSM9DS1_MAG_LP_80Hz       = 0x07,
  LSM9DS1_MAG_MP_0Hz625     = 0x10,
  LSM9DS1_MAG_MP_1Hz25      = 0x11,
  LSM9DS1_MAG_MP_2Hz5       = 0x12,
  LSM9DS1_MAG_MP_5Hz        = 0x13,
  LSM9DS1_MAG_MP_10Hz       = 0x14,
  LSM9DS1_MAG_MP_20Hz       = 0x15,
  LSM9DS1_MAG_MP_40Hz       = 0x16,
  LSM9DS1_MAG_MP_80Hz       = 0x17,
  LSM9DS1_MAG_HP_0Hz625     = 0x20,
  LSM9DS1_MAG_HP_1Hz25      = 0x21,
  LSM9DS1_MAG_HP_2Hz5       = 0x22,
  LSM9DS1_MAG_HP_5Hz        = 0x23,
  LSM9DS1_MAG_HP_10Hz       = 0x24,
  LSM9DS1_MAG_HP_20Hz       = 0x25,
  LSM9DS1_MAG_HP_40Hz       = 0x26,
  LSM9DS1_MAG_HP_80Hz       = 0x27,
  LSM9DS1_MAG_UHP_0Hz625    = 0x30,
  LSM9DS1_MAG_UHP_1Hz25     = 0x31,
  LSM9DS1_MAG_UHP_2Hz5      = 0x32,
  LSM9DS1_MAG_UHP_5Hz       = 0x33,
  LSM9DS1_MAG_UHP_10Hz      = 0x34,
  LSM9DS1_MAG_UHP_20Hz      = 0x35,
  LSM9DS1_MAG_UHP_40Hz      = 0x36,
  LSM9DS1_MAG_UHP_80Hz      = 0x37,
  LSM9DS1_MAG_UHP_155Hz     = 0x38,
  LSM9DS1_MAG_HP_300Hz      = 0x28,
  LSM9DS1_MAG_MP_560Hz      = 0x18,
  LSM9DS1_MAG_LP_1000Hz     = 0x08,
  LSM9DS1_MAG_ONE_SHOT      = 0x70,
} lsm9ds1_mag_data_rate_t;

typedef struct {
  lsm9ds1_status_reg_m_t status_mag;
  lsm9ds1_status_reg_t   status_imu;
} lsm9ds1_status_t;


/* IMU/MAG Functions -------------------------------------------------------------*/
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);

int32_t lsm9ds1_dev_id_get(stmdev_ctx_t *ctx_mag, stmdev_ctx_t *ctx_imu, lsm9ds1_id_t *buff);

#define lsm9ds1_read_reg(ctx, reg, data, len) ctx->read_reg(ctx->handle, reg, data, len)
#define lsm9ds1_write_reg(ctx, reg, data, len) ctx->write_reg(ctx->handle, reg, data, len)

int32_t lsm9ds1_dev_reset_set(stmdev_ctx_t *ctx_mag, stmdev_ctx_t *ctx_imu, uint8_t val);
int32_t lsm9ds1_dev_reset_get(stmdev_ctx_t *ctx_mag, stmdev_ctx_t *ctx_imu, uint8_t *val);

int32_t lsm9ds1_block_data_update_set(stmdev_ctx_t *ctx_mag, stmdev_ctx_t *ctx_imu, uint8_t val);
int32_t lsm9ds1_xl_full_scale_set(stmdev_ctx_t *ctx, lsm9ds1_xl_fs_t val);
int32_t lsm9ds1_gy_full_scale_set(stmdev_ctx_t *ctx, lsm9ds1_gy_fs_t val);
int32_t lsm9ds1_mag_full_scale_set(stmdev_ctx_t *ctx, lsm9ds1_mag_fs_t val);

int32_t lsm9ds1_xl_filter_aalias_bandwidth_set(stmdev_ctx_t *ctx, lsm9ds1_xl_aa_bw_t val);
int32_t lsm9ds1_xl_filter_lp_bandwidth_set(stmdev_ctx_t *ctx, lsm9ds1_xl_lp_bw_t val);
int32_t lsm9ds1_xl_filter_out_path_set(stmdev_ctx_t *ctx, lsm9ds1_xl_out_path_t val);

int32_t lsm9ds1_gy_filter_lp_bandwidth_set(stmdev_ctx_t *ctx, lsm9ds1_gy_lp_bw_t val);
int32_t lsm9ds1_gy_filter_hp_bandwidth_set(stmdev_ctx_t *ctx, lsm9ds1_gy_hp_bw_t val);
int32_t lsm9ds1_gy_filter_out_path_set(stmdev_ctx_t *ctx, lsm9ds1_gy_out_path_t val);

int32_t lsm9ds1_imu_data_rate_set(stmdev_ctx_t *ctx, lsm9ds1_imu_odr_t val);
int32_t lsm9ds1_mag_data_rate_set(stmdev_ctx_t *ctx, lsm9ds1_mag_data_rate_t val);
int32_t lsm9ds1_filter_settling_mask_set(stmdev_ctx_t *ctx, uint8_t val);

int32_t lsm9ds1_dev_status_get(stmdev_ctx_t *ctx_mag, stmdev_ctx_t *ctx_imu, lsm9ds1_status_t *val);
int32_t lsm9ds1_angular_rate_raw_get(stmdev_ctx_t *ctx, uint8_t *buff);
int32_t lsm9ds1_acceleration_raw_get(stmdev_ctx_t *ctx, uint8_t *buff);
int32_t lsm9ds1_magnetic_raw_get(stmdev_ctx_t *ctx, uint8_t *buff);

/* Public IMU/MAG Variables-------------------------------------------------------------*/
extern stmdev_ctx_t dev_ctx_mag;
extern stmdev_ctx_t dev_ctx_imu;

#endif /* __RP_I2C */
