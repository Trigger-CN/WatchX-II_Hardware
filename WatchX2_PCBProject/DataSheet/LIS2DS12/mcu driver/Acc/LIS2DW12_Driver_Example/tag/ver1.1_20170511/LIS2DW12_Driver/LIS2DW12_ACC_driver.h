/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
* File Name          : LIS2DW12_ACC_driver.h
* Author             : MEMS Application Team
* Version            : v1.1
* Date               : 11 May 2017
* Description        : EKSTM32 main file
*
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LIS2DW12_ACC_DRIVER__H
#define __LIS2DW12_ACC_DRIVER__H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

//these could change accordingly with the architecture

#ifdef __cplusplus
  extern "C" {
#endif

#ifndef __ARCHDEP__TYPES
#define __ARCHDEP__TYPES

typedef unsigned char u8_t;
typedef unsigned short int u16_t;
typedef unsigned int u32_t;
typedef int i32_t;
typedef short int i16_t;
typedef signed char i8_t;

#endif /*__ARCHDEP__TYPES*/

/* Exported common structure --------------------------------------------------------*/

#ifndef __SHARED__TYPES
#define __SHARED__TYPES

typedef union{
	i16_t i16bit[3];
	u8_t u8bit[6];
} Type3Axis16bit_U;	

typedef union{
	i16_t i16bit;
	u8_t u8bit[2];
} Type1Axis16bit_U;

typedef union{
	i32_t i32bit;
	u8_t u8bit[4];
} Type1Axis32bit_U;

typedef enum {
  MEMS_SUCCESS				=		0x01,
  MEMS_ERROR				=		0x00	
} status_t;

#endif /*__SHARED__TYPES*/

/* Exported macro ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/************** I2C Address *****************/

#define LIS2DW12_I2C_ADDRESS_LOW     0x30  // SAD[0] = 0
#define LIS2DW12_I2C_ADDRESS_HIGH    0x32  // SAD[0] = 1

/************** Who am I  *******************/

#define LIS2DW12_ACC_WHO_AM_I         0x44

/************** Device Register  *******************/
#define LIS2DW12_ACC_OUT_T_L  	0X0D
#define LIS2DW12_ACC_OUT_T_H  	0X0E
#define LIS2DW12_ACC_WHO_AM_I_REG  	0X0F
#define LIS2DW12_ACC_CTRL1  	0X20
#define LIS2DW12_ACC_CTRL2  	0X21
#define LIS2DW12_ACC_CTRL3  	0X22
#define LIS2DW12_ACC_CTRL4_INT1_PAD_CTRL  	0X23
#define LIS2DW12_ACC_CTRL5_INT2_PAD_CTRL  	0X24
#define LIS2DW12_ACC_CTRL6  	0X25
#define LIS2DW12_ACC_OUT_T  	0X26
#define LIS2DW12_ACC_STATUS  	0X27
#define LIS2DW12_ACC_OUT_X_L  	0X28
#define LIS2DW12_ACC_OUT_X_H  	0X29
#define LIS2DW12_ACC_OUT_Y_L  	0X2A
#define LIS2DW12_ACC_OUT_Y_H  	0X2B
#define LIS2DW12_ACC_OUT_Z_L  	0X2C
#define LIS2DW12_ACC_OUT_Z_H  	0X2D
#define LIS2DW12_ACC_FIFO_CTRL  	0X2E
#define LIS2DW12_ACC_FIFO_SAMPLES  	0X2F
#define LIS2DW12_ACC_TAP_THS_X  	0X30
#define LIS2DW12_ACC_TAP_THS_Y  	0X31
#define LIS2DW12_ACC_TAP_THS_Z  	0X32
#define LIS2DW12_ACC_INT_DUR  	0X33
#define LIS2DW12_ACC_WAKE_UP_THS  	0X34
#define LIS2DW12_ACC_WAKE_UP_DUR  	0X35
#define LIS2DW12_ACC_FREE_FALL  	0X36
#define LIS2DW12_ACC_STATUS_DUP  	0X37
#define LIS2DW12_ACC_WAKE_UP_SRC  	0X38
#define LIS2DW12_ACC_TAP_SRC  	0X39
#define LIS2DW12_ACC_SIXD_SRC  	0X3A
#define LIS2DW12_ACC_ALL_INT_SRC  	0X3B
#define LIS2DW12_ACC_ABS_INT_X  	0X3C
#define LIS2DW12_ACC_ABS_INT_Y  	0X3D
#define LIS2DW12_ACC_ABS_INT_Z  	0X3E
#define LIS2DW12_ACC_ABS_INT_CFG  	0X3F


/************** Generic Function  *******************/

/*******************************************************************************
* Register      : Generic - All
* Address       : Generic - All
* Bit Group Name: None
* Permission    : W
*******************************************************************************/
status_t LIS2DW12_ACC_WriteReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len);

/*******************************************************************************
* Register      : Generic - All
* Address       : Generic - All
* Bit Group Name: None
* Permission    : R
*******************************************************************************/
status_t LIS2DW12_ACC_ReadReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len);

/**************** Base Function  *******************/

/*******************************************************************************
* Register      : WHO_AM_I
* Address       : 0X0F
* Bit Group Name: WHO_AM_I_BIT
* Permission    : RO
*******************************************************************************/
#define  	LIS2DW12_ACC_WHO_AM_I_BIT_MASK  	0xFF
#define  	LIS2DW12_ACC_WHO_AM_I_BIT_POSITION  	0
status_t LIS2DW12_ACC_R_WhoAmI(void *handle, u8_t *value);

/*******************************************************************************
* Register      : CTRL2
* Address       : 0X21
* Bit Group Name: BDU
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_BDU_DISABLE 		 =0x00,
  	LIS2DW12_ACC_BDU_ENABLE 		 =0x08,
} LIS2DW12_ACC_BDU_t;

#define  	LIS2DW12_ACC_BDU_MASK  	0x08
status_t  LIS2DW12_ACC_W_BlockDataUpdate(void *handle, LIS2DW12_ACC_BDU_t newValue);
status_t LIS2DW12_ACC_R_BlockDataUpdate(void *handle, LIS2DW12_ACC_BDU_t *value);

/*******************************************************************************
* Register      : CTRL6
* Address       : 0X25
* Bit Group Name: FS
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_FS_2g 		 =0x00,
  	LIS2DW12_ACC_FS_4g 		 =0x10,
  	LIS2DW12_ACC_FS_8g 		 =0x20,
  	LIS2DW12_ACC_FS_16g 		 =0x30,
} LIS2DW12_ACC_FS_t;

#define  	LIS2DW12_ACC_FS_MASK  	0x30
status_t  LIS2DW12_ACC_W_FullScaleSelection(void *handle, LIS2DW12_ACC_FS_t newValue);
status_t LIS2DW12_ACC_R_FullScaleSelection(void *handle, LIS2DW12_ACC_FS_t *value);

/*******************************************************************************
* Register      : CTRL1
* Address       : 0X20
* Bit Group Name: ODR
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_ODR_POWER_DOWN 		 =0x00,
  	LIS2DW12_ACC_ODR_LP_1Hz6_HP_12Hz5 		 =0x10,
  	LIS2DW12_ACC_ODR_LP_12Hz5_HP_12Hz5 		 =0x20,
  	LIS2DW12_ACC_ODR_LP_25Hz_HP_25Hz 		 =0x30,
  	LIS2DW12_ACC_ODR_LP_50Hz_HP_50Hz 		 =0x40,
  	LIS2DW12_ACC_ODR_LP_100Hz_HP_100Hz 		 =0x50,
  	LIS2DW12_ACC_ODR_LP_200Hz_HP_200Hz 		 =0x60,
  	LIS2DW12_ACC_ODR_LP_200Hz_HP_400Hz 		 =0x70,
  	LIS2DW12_ACC_ODR_LP_200Hz_HP_800Hz 		 =0x80,
  	LIS2DW12_ACC_ODR_LP_200Hz_HP_1600Hz 		 =0x90,
} LIS2DW12_ACC_ODR_t;

#define  	LIS2DW12_ACC_ODR_MASK  	0xF0
status_t  LIS2DW12_ACC_W_OutputDataRate(void *handle, LIS2DW12_ACC_ODR_t newValue);
status_t LIS2DW12_ACC_R_OutputDataRate(void *handle, LIS2DW12_ACC_ODR_t *value);

/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : Acceleration
* Permission    : RO 
*******************************************************************************/
status_t LIS2DW12_ACC_Get_Acceleration(void *handle, u8_t *buff);

/*******************************************************************************
  * Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : Temperature
* Permission    : RO 
*******************************************************************************/
status_t LIS2DW12_ACC_Get_Temperature(void *handle, u8_t *buff); 
 
/**************** Advanced Function  *******************/

/*******************************************************************************
* Register      : CTRL1
* Address       : 0X20
* Bit Group Name: LP_MODE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_LP_MODE1_12bit 		 =0x00,
  	LIS2DW12_ACC_LP_MODE2_14bit 		 =0x01,
  	LIS2DW12_ACC_LP_MODE3_14bit 		 =0x02,
  	LIS2DW12_ACC_LP_MODE4_14bit 		 =0x03,
} LIS2DW12_ACC_LP_MODE_t;

#define  	LIS2DW12_ACC_LP_MODE_MASK  	0x03
status_t  LIS2DW12_ACC_W_LowPowerModeSelection(void *handle, LIS2DW12_ACC_LP_MODE_t newValue);
status_t LIS2DW12_ACC_R_LowPowerModeSelection(void *handle, LIS2DW12_ACC_LP_MODE_t *value);

/*******************************************************************************
* Register      : CTRL1
* Address       : 0X20
* Bit Group Name: MODE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_MODE_LOW_POWER_STD 		 =0x00,
  	LIS2DW12_ACC_MODE_HIGH_PERFORMANCE 		 =0x04,
  	LIS2DW12_ACC_MODE_LOW_POWER_SINGLE 		 =0x08,
} LIS2DW12_ACC_MODE_t;

#define  	LIS2DW12_ACC_MODE_MASK  	0x0C
status_t  LIS2DW12_ACC_W_ModeSelection(void *handle, LIS2DW12_ACC_MODE_t newValue);
status_t LIS2DW12_ACC_R_ModeSelection(void *handle, LIS2DW12_ACC_MODE_t *value);

/*******************************************************************************
* Register      : CTRL2
* Address       : 0X21
* Bit Group Name: SIM
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_SIM_4_WIRE 		 =0x00,
  	LIS2DW12_ACC_SIM_3_WIRE 		 =0x01,
} LIS2DW12_ACC_SIM_t;

#define  	LIS2DW12_ACC_SIM_MASK  	0x01
status_t  LIS2DW12_ACC_W_SPI_mode_selection(void *handle, LIS2DW12_ACC_SIM_t newValue);
status_t LIS2DW12_ACC_R_SPI_mode_selection(void *handle, LIS2DW12_ACC_SIM_t *value);

/*******************************************************************************
* Register      : CTRL2
* Address       : 0X21
* Bit Group Name: I2C_DISABLE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_I2C_DISABLE_ENABLE 		 =0x00,
  	LIS2DW12_ACC_I2C_DISABLE_DISABLE 		 =0x02,
} LIS2DW12_ACC_I2C_DISABLE_t;

#define  	LIS2DW12_ACC_I2C_DISABLE_MASK  	0x02
status_t  LIS2DW12_ACC_W_I2C_status(void *handle, LIS2DW12_ACC_I2C_DISABLE_t newValue);
status_t LIS2DW12_ACC_R_I2C_status(void *handle, LIS2DW12_ACC_I2C_DISABLE_t *value);

/*******************************************************************************
* Register      : CTRL2
* Address       : 0X21
* Bit Group Name: IF_ADD_INC
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_IF_ADD_INC_DISABLE 		 =0x00,
  	LIS2DW12_ACC_IF_ADD_INC_ENABLE 		 =0x04,
} LIS2DW12_ACC_IF_ADD_INC_t;

#define  	LIS2DW12_ACC_IF_ADD_INC_MASK  	0x04
status_t  LIS2DW12_ACC_W_AutoIncrementedWithMultipleAccess(void *handle, LIS2DW12_ACC_IF_ADD_INC_t newValue);
status_t LIS2DW12_ACC_R_AutoIncrementedWithMultipleAccess(void *handle, LIS2DW12_ACC_IF_ADD_INC_t *value);

/*******************************************************************************
* Register      : CTRL2
* Address       : 0X21
* Bit Group Name: SOFT_RESET
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_SOFT_RESET_DISABLE 		 =0x00,
  	LIS2DW12_ACC_SOFT_RESET_ENABLE 		 =0x40,
} LIS2DW12_ACC_SOFT_RESET_t;

#define  	LIS2DW12_ACC_SOFT_RESET_MASK  	0x40
status_t  LIS2DW12_ACC_W_SoftReset(void *handle, LIS2DW12_ACC_SOFT_RESET_t newValue);
status_t LIS2DW12_ACC_R_SoftReset(void *handle, LIS2DW12_ACC_SOFT_RESET_t *value);

/*******************************************************************************
* Register      : CTRL2
* Address       : 0X21
* Bit Group Name: BOOT
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_BOOT_DISABLE 		 =0x00,
  	LIS2DW12_ACC_BOOT_FORCE 		 =0x80,
} LIS2DW12_ACC_BOOT_t;

#define  	LIS2DW12_ACC_BOOT_MASK  	0x80
status_t  LIS2DW12_ACC_W_Reboot(void *handle, LIS2DW12_ACC_BOOT_t newValue);
status_t LIS2DW12_ACC_R_Reboot(void *handle, LIS2DW12_ACC_BOOT_t *value);

/*******************************************************************************
* Register      : CTRL3
* Address       : 0X22
* Bit Group Name: SLP_MODE_1
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_SLP_MODE_1_OFF 		 =0x00,
  	LIS2DW12_ACC_SLP_MODE_1_ARMED 		 =0x01,
} LIS2DW12_ACC_SLP_MODE_1_t;

#define  	LIS2DW12_ACC_SLP_MODE_1_MASK  	0x01
status_t  LIS2DW12_ACC_W_SingleLowPowerMode(void *handle, LIS2DW12_ACC_SLP_MODE_1_t newValue);
status_t LIS2DW12_ACC_R_SingleLowPowerMode(void *handle, LIS2DW12_ACC_SLP_MODE_1_t *value);

/*******************************************************************************
* Register      : CTRL3
* Address       : 0X22
* Bit Group Name: SLP_MODE_SEL
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_SLP_MODE_SEL_ON_INT2 		 =0x00,
  	LIS2DW12_ACC_SLP_MODE_SEL_ON_REGISTER 		 =0x02,
} LIS2DW12_ACC_SLP_MODE_SEL_t;

#define  	LIS2DW12_ACC_SLP_MODE_SEL_MASK  	0x02
status_t  LIS2DW12_ACC_W_SingleLowPowerModeSource(void *handle, LIS2DW12_ACC_SLP_MODE_SEL_t newValue);
status_t LIS2DW12_ACC_R_SingleLowPowerModeSource(void *handle, LIS2DW12_ACC_SLP_MODE_SEL_t *value);

/*******************************************************************************
* Register      : CTRL3
* Address       : 0X22
* Bit Group Name: H_LACTIVE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_H_LACTIVE_HIGH 		 =0x00,
  	LIS2DW12_ACC_H_LACTIVE_LOW 		 =0x08,
} LIS2DW12_ACC_H_LACTIVE_t;

#define  	LIS2DW12_ACC_H_LACTIVE_MASK  	0x08
status_t  LIS2DW12_ACC_W_InterruptPolarity(void *handle, LIS2DW12_ACC_H_LACTIVE_t newValue);
status_t LIS2DW12_ACC_R_InterruptPolarity(void *handle, LIS2DW12_ACC_H_LACTIVE_t *value);

/*******************************************************************************
* Register      : CTRL3
* Address       : 0X22
* Bit Group Name: LIR
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_LIR_DISABLE 		 =0x00,
  	LIS2DW12_ACC_LIR_ENABLE 		 =0x10,
} LIS2DW12_ACC_LIR_t;

#define  	LIS2DW12_ACC_LIR_MASK  	0x10
status_t  LIS2DW12_ACC_W_LatchIntteruptRq(void *handle, LIS2DW12_ACC_LIR_t newValue);
status_t LIS2DW12_ACC_R_LatchIntteruptRq(void *handle, LIS2DW12_ACC_LIR_t *value);

/*******************************************************************************
* Register      : CTRL3
* Address       : 0X22
* Bit Group Name: PP_OD
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_PP_OD_PUSH_PULL 		 =0x00,
  	LIS2DW12_ACC_PP_OD_OPEN_DRAIN 		 =0x20,
} LIS2DW12_ACC_PP_OD_t;

#define  	LIS2DW12_ACC_PP_OD_MASK  	0x20
status_t  LIS2DW12_ACC_W_InterruptPadMode(void *handle, LIS2DW12_ACC_PP_OD_t newValue);
status_t LIS2DW12_ACC_R_InterruptPadMode(void *handle, LIS2DW12_ACC_PP_OD_t *value);

/*******************************************************************************
* Register      : CTRL3
* Address       : 0X22
* Bit Group Name: ST1
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_ST_DISABLE 		 =0x00,
  	LIS2DW12_ACC_ST_POSITIVE 		 =0x40,
  	LIS2DW12_ACC_ST_NEGATIVE 		 =0x80,
} LIS2DW12_ACC_ST_t;

#define  	LIS2DW12_ACC_ST_MASK  	0xC0
status_t  LIS2DW12_ACC_W_SelfTest(void *handle, LIS2DW12_ACC_ST_t newValue);
status_t LIS2DW12_ACC_R_SelfTest(void *handle, LIS2DW12_ACC_ST_t *value);

/*******************************************************************************
* Register      : CTRL4_INT1_PAD_CTRL
* Address       : 0X23
* Bit Group Name: INT1_MODE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_INT1_MODE_UNROUTED 		 =0x00,
  	LIS2DW12_ACC_INT1_MODE_DATA_READY 		 =0x01,
  	LIS2DW12_ACC_INT1_MODE_FIFO_TRESHOLD		 =0x02,
  	LIS2DW12_ACC_INT1_MODE_FIFO_FULL =0x04,
  	LIS2DW12_ACC_INT1_MODE_DOUBLE_TAP 		 =0x08,
  	LIS2DW12_ACC_INT1_MODE_FREE_FALL 		 =0x10,
  	LIS2DW12_ACC_INT1_MODE_WAKE_UP 		 =0x20,
  	LIS2DW12_ACC_INT1_MODE_SINGLE_TAP 		 =0x40,
  	LIS2DW12_ACC_INT1_MODE_6D 		 =0x80,
} LIS2DW12_ACC_INT1_MODE_t;

#define  	LIS2DW12_ACC_INT1_MODE_MASK  	0xFF
status_t  LIS2DW12_ACC_W_PinFunction_INT1(void *handle, LIS2DW12_ACC_INT1_MODE_t newValue);
status_t LIS2DW12_ACC_R_PinFunction_INT1(void *handle, LIS2DW12_ACC_INT1_MODE_t *value);


/*******************************************************************************
* Register      : CTRL5_INT2_PAD_CTRL
* Address       : 0X24
* Bit Group Name: INT2_DRDY
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_INT2_MODE_UNROUTED 		 =0x00,
  	LIS2DW12_ACC_INT2_MODE_DATA_READY 		 =0x01,
  	LIS2DW12_ACC_INT2_MODE_FIFO_TRESHOLD		 =0x02,
  	LIS2DW12_ACC_INT2_MODE_FIFO_FULL =0x04,
  	LIS2DW12_ACC_INT2_MODE_OVERRUN 		 =0x08,
  	LIS2DW12_ACC_INT2_MODE_TEMPERATURE_DATA_READY 		 =0x10,
  	LIS2DW12_ACC_INT2_MODE_BOOT 		 =0x20,
  	LIS2DW12_ACC_INT2_MODE_SLEEP_CHANGE 		 =0x40,
} LIS2DW12_ACC_INT2_MODE_t;

#define  	LIS2DW12_ACC_INT2_MODE_MASK  	0xFF
status_t  LIS2DW12_ACC_W_PinFunction_INT2(void *handle, LIS2DW12_ACC_INT2_MODE_t newValue);
status_t  LIS2DW12_ACC_R_PinFunction_INT2(void *handle, LIS2DW12_ACC_INT2_MODE_t *value);


/*******************************************************************************
* Register      : CTRL6
* Address       : 0X25
* Bit Group Name: LOW_NOISE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_LOW_NOISE_DISABLE 		 =0x00,
  	LIS2DW12_ACC_LOW_NOISE_ENABLE 		 =0x04,
} LIS2DW12_ACC_LOW_NOISE_t;

#define  	LIS2DW12_ACC_LOW_NOISE_MASK  	0x04
status_t  LIS2DW12_ACC_W_LowNoiseConfiguration(void *handle, LIS2DW12_ACC_LOW_NOISE_t newValue);
status_t LIS2DW12_ACC_R_LowNoiseConfiguration(void *handle, LIS2DW12_ACC_LOW_NOISE_t *value);

/*******************************************************************************
* Register      : CTRL6
* Address       : 0X25
* Bit Group Name: FDS
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_FDS_LOW_PASS 		 =0x00,
  	LIS2DW12_ACC_FDS_HIGH_PASS 		 =0x08,
} LIS2DW12_ACC_FDS_t;

#define  	LIS2DW12_ACC_FDS_MASK  	0x08
status_t  LIS2DW12_ACC_W_FliterConfiguration(void *handle, LIS2DW12_ACC_FDS_t newValue);
status_t LIS2DW12_ACC_R_FliterConfiguration(void *handle, LIS2DW12_ACC_FDS_t *value);

/*******************************************************************************
* Register      : CTRL6
* Address       : 0X25
* Bit Group Name: BW_FILT
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_BW_FILT_MAX 		 =0x00,
  	LIS2DW12_ACC_BW_FILT_ODR_DIV_4 		 =0x40,
  	LIS2DW12_ACC_BW_FILT_ODR_DIV_10 		 =0x80,
  	LIS2DW12_ACC_BW_FILT_ODR_DIV_20 		 =0xC0,
} LIS2DW12_ACC_BW_FILT_t;

#define  	LIS2DW12_ACC_BW_FILT_MASK  	0xC0
status_t  LIS2DW12_ACC_W_FilterCutOff(void *handle, LIS2DW12_ACC_BW_FILT_t newValue);
status_t LIS2DW12_ACC_R_FilterCutOff(void *handle, LIS2DW12_ACC_BW_FILT_t *value);

/*******************************************************************************
* Register      : OUT_T
* Address       : 0X26
* Bit Group Name: TEMP
* Permission    : RO
*******************************************************************************/
#define  	LIS2DW12_ACC_TEMP_MASK  	0xFF
#define  	LIS2DW12_ACC_TEMP_POSITION  	0
status_t LIS2DW12_ACC_R_TEMP_bits(void *handle, u8_t *value);

/*******************************************************************************
* Register      : STATUS
* Address       : 0X27
* Bit Group Name: DRDY, FF_IA, 6D_IA, SINGLE_TAP, 
*                 DOUBLE_TAP, SLEEP_STATE, WU_IA, FIFO_THS
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_IDLE 		 =0x0000,
  	LIS2DW12_ACC_DRDY_NEW_AVAILABLE 		 =0x0001,
  	LIS2DW12_ACC_FF_IA_DETECTED 		 =0x0002,
  	LIS2DW12_ACC_6D_IA_DETECTED 		 =0x0004,
  	LIS2DW12_ACC_SINGLE_TAP_DETECTED 		 =0x0008,
  	LIS2DW12_ACC_DOUBLE_TAP_DETECTED 		 =0x0010,
  	LIS2DW12_ACC_SLEEP_STATE_DETECTED 		 =0x0020,
  	LIS2DW12_ACC_WU_IA_DETECTED 		 =0x0040,
  	LIS2DW12_ACC_FIFO_THS_LEVEL_REACHED 		 =0x0080,
  	LIS2DW12_ACC_SLEEP_STATE_IA_OCCURRED 		 =0x2000,
  	LIS2DW12_ACC_DRDY_T_AVAILABLE 		 =0x4000,
  	LIS2DW12_ACC_OVR_OCCURRED 		 =0x8000,
} LIS2DW12_ACC_STATUS_t;

status_t LIS2DW12_ACC_R_GetStatus(void *handle, LIS2DW12_ACC_STATUS_t *value);

/*******************************************************************************
* Register      : FIFO_CTRL
* Address       : 0X2E
* Bit Group Name: FTH
* Permission    : RW
*******************************************************************************/
#define  	LIS2DW12_ACC_FTH_MASK  	0x1F
#define  	LIS2DW12_ACC_FTH_POSITION  	0
status_t  LIS2DW12_ACC_W_FIFO_Threshold(void *handle, u8_t newValue);
status_t LIS2DW12_ACC_R_FIFO_Threshold(void *handle, u8_t *value);

/*******************************************************************************
* Register      : FIFO_CTRL
* Address       : 0X2E
* Bit Group Name: FMODE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_FMODE_BYPASS 		 =0x00,
  	LIS2DW12_ACC_FMODE_STOP_WHEN_FULL 		 =0x20,
  	LIS2DW12_ACC_FMODE_STREAM_TO_FIFO 		 =0x60,
  	LIS2DW12_ACC_FMODE_BYPASS_TO_STREAM 		 =0x80,
  	LIS2DW12_ACC_FMODE_STREAM		 =0xC0,
} LIS2DW12_ACC_FMODE_t;

#define  	LIS2DW12_ACC_FMODE_MASK  	0xE0
status_t  LIS2DW12_ACC_W_FIFO_mode(void *handle, LIS2DW12_ACC_FMODE_t newValue);
status_t LIS2DW12_ACC_R_FIFO_mode(void *handle, LIS2DW12_ACC_FMODE_t *value);

/*******************************************************************************
* Register      : FIFO_SAMPLES
* Address       : 0X2F
* Bit Group Name: DIFF
* Permission    : RO
*******************************************************************************/
#define  	LIS2DW12_ACC_DIFF_MASK  	0x3F
#define  	LIS2DW12_ACC_DIFF_POSITION  0
status_t LIS2DW12_ACC_R_FIFO_level(void *handle, u8_t *value);

/*******************************************************************************
* Register      : FIFO_SAMPLES
* Address       : 0X2F
* Bit Group Name: FIFO_OVR
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_FIFO_OVR_NOT_OCCURRED 		 =0x00,
  	LIS2DW12_ACC_FIFO_OVR_OCCURRED 		 =0x40,
} LIS2DW12_ACC_FIFO_OVR_t;

#define  	LIS2DW12_ACC_FIFO_OVR_MASK  	0x40
status_t LIS2DW12_ACC_R_FIFO_OverrunStatus(void *handle, LIS2DW12_ACC_FIFO_OVR_t *value);

/*******************************************************************************
* Register      : FIFO_SAMPLES
* Address       : 0X2F
* Bit Group Name: FIFO_FTH
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_FIFO_FTH_BELOW_LEVEL 		 =0x00,
  	LIS2DW12_ACC_FIFO_FTH_LEVEL_REACHED 		 =0x80,
} LIS2DW12_ACC_FIFO_FTH_t;

#define  	LIS2DW12_ACC_FIFO_FTH_MASK  	0x80
status_t LIS2DW12_ACC_R_FIFO_Threshold_Status(void *handle, LIS2DW12_ACC_FIFO_FTH_t *value);

/*******************************************************************************
* Register      : TAP_THS_X
* Address       : 0X30
* Bit Group Name: TAP_THSX
* Permission    : RW
*******************************************************************************/
#define  	LIS2DW12_ACC_TAP_THSX_MASK  	0x1F
#define  	LIS2DW12_ACC_TAP_THSX_POSITION  	0
status_t  LIS2DW12_ACC_W_TAP_X_Threshold(void *handle, u8_t newValue);
status_t LIS2DW12_ACC_R_TAP_X_Threshold(void *handle, u8_t *value);

/*******************************************************************************
* Register      : TAP_THS_X
* Address       : 0X30
* Bit Group Name: 6D_THS
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_6D_THS_80deg 		 =0x00,
  	LIS2DW12_ACC_6D_THS_70deg 		 =0x20,
  	LIS2DW12_ACC_6D_THS_60deg 		 =0x40,
  	LIS2DW12_ACC_6D_THS_50deg 		 =0x60,
} LIS2DW12_ACC_6D_THS_t;

#define  	LIS2DW12_ACC_6D_THS_MASK  	0x60
status_t  LIS2DW12_ACC_W_6D_Threshold(void *handle, LIS2DW12_ACC_6D_THS_t newValue);
status_t LIS2DW12_ACC_R_6D_Threshold(void *handle, LIS2DW12_ACC_6D_THS_t *value);

/*******************************************************************************
* Register      : TAP_THS_X
* Address       : 0X30
* Bit Group Name: 4D_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_4D_EN_DISABLE 		 =0x00,
  	LIS2DW12_ACC_4D_EN_ENABLE 		 =0x80,
} LIS2DW12_ACC_4D_EN_t;

#define  	LIS2DW12_ACC_4D_EN_MASK  	0x80
status_t  LIS2DW12_ACC_W_4D_Function(void *handle, LIS2DW12_ACC_4D_EN_t newValue);
status_t LIS2DW12_ACC_R_4D_Function(void *handle, LIS2DW12_ACC_4D_EN_t *value);

/*******************************************************************************
* Register      : TAP_THS_Y
* Address       : 0X31
* Bit Group Name: TAP_THSY
* Permission    : RW
*******************************************************************************/
#define  	LIS2DW12_ACC_TAP_THSY_MASK  	0x1F
#define  	LIS2DW12_ACC_TAP_THSY_POSITION  	0
status_t  LIS2DW12_ACC_W_TAP_Y_Threshold(void *handle, u8_t newValue);
status_t LIS2DW12_ACC_R_TAP_Y_Threshold(void *handle, u8_t *value);

/*******************************************************************************
* Register      : TAP_THS_Y
* Address       : 0X31
* Bit Group Name: TAP_PRIOR
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_TAP_PRIOR_XYZ 		 =0x00,
  	LIS2DW12_ACC_TAP_PRIOR_YXZ 		 =0x20,
  	LIS2DW12_ACC_TAP_PRIOR_XZY 		 =0x40,
  	LIS2DW12_ACC_TAP_PRIOR_ZYX 		 =0x60,
  	/*LIS2DW12_ACC_TAP_PRIOR_XYZ 		 =0x80,*/
  	LIS2DW12_ACC_TAP_PRIOR_YZX 		 =0xA0,
  	LIS2DW12_ACC_TAP_PRIOR_ZXY 		 =0xC0,
  	/*LIS2DW12_ACC_TAP_PRIOR_ZYX 		 =0xE0,*/


} LIS2DW12_ACC_TAP_PRIOR_t;

#define  	LIS2DW12_ACC_TAP_PRIOR_MASK 0xE0
status_t  LIS2DW12_ACC_W_TAP_PriorityAxisTreshold(void *handle, LIS2DW12_ACC_TAP_PRIOR_t newValue);
status_t LIS2DW12_ACC_R_TAP_PriorityAxisTreshold(void *handle, LIS2DW12_ACC_TAP_PRIOR_t *value);

/*******************************************************************************
* Register      : TAP_THS_Z
* Address       : 0X32
* Bit Group Name: TAP_THSZ
* Permission    : RW
*******************************************************************************/
#define  	LIS2DW12_ACC_TAP_THSZ_MASK  	0x1F
#define  	LIS2DW12_ACC_TAP_THSZ_POSITION  	0
status_t  LIS2DW12_ACC_W_TAP_Z_Threshold(void *handle, u8_t newValue);
status_t LIS2DW12_ACC_R_TAP_Z_Threshold(void *handle, u8_t *value);

/*******************************************************************************
* Register      : TAP_THS_Z
* Address       : 0X32
* Bit Group Name: TAP_Z_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_TAP_Z_EN_DISABLE 		 =0x00,
  	LIS2DW12_ACC_TAP_Z_EN_ENABLE 		 =0x20,
} LIS2DW12_ACC_TAP_Z_EN_t;

#define  	LIS2DW12_ACC_TAP_Z_EN_MASK  	0x20
status_t  LIS2DW12_ACC_W_DetectTAP_on_Z_Axis(void *handle, LIS2DW12_ACC_TAP_Z_EN_t newValue);
status_t LIS2DW12_ACC_R_DetectTAP_on_Z_Axis(void *handle, LIS2DW12_ACC_TAP_Z_EN_t *value);

/*******************************************************************************
* Register      : TAP_THS_Z
* Address       : 0X32
* Bit Group Name: TAP_Y_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_TAP_Y_EN_DISABLE 		 =0x00,
  	LIS2DW12_ACC_TAP_Y_EN_ENABLE 		 =0x40,
} LIS2DW12_ACC_TAP_Y_EN_t;

#define  	LIS2DW12_ACC_TAP_Y_EN_MASK  	0x40
status_t  LIS2DW12_ACC_W_DetectTAP_on_Y_Axis(void *handle, LIS2DW12_ACC_TAP_Y_EN_t newValue);
status_t LIS2DW12_ACC_R_DetectTAP_on_Y_Axis(void *handle, LIS2DW12_ACC_TAP_Y_EN_t *value);

/*******************************************************************************
* Register      : TAP_THS_Z
* Address       : 0X32
* Bit Group Name: TAP_X_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_TAP_X_EN_DISABLE 		 =0x00,
  	LIS2DW12_ACC_TAP_X_EN_ENABLE 		 =0x80,
} LIS2DW12_ACC_TAP_X_EN_t;

#define  	LIS2DW12_ACC_TAP_X_EN_MASK  	0x80
status_t  LIS2DW12_ACC_W_DetectTAP_on_X_Axis(void *handle, LIS2DW12_ACC_TAP_X_EN_t newValue);
status_t LIS2DW12_ACC_R_DetectTAP_on_X_Axis(void *handle, LIS2DW12_ACC_TAP_X_EN_t *value);

/*******************************************************************************
* Register      : INT_DUR
* Address       : 0X33
* Bit Group Name: SHOCK
* Permission    : RW
*******************************************************************************/
#define  	LIS2DW12_ACC_SHOCK_MASK  	0x03
#define  	LIS2DW12_ACC_SHOCK_POSITION  	0
status_t  LIS2DW12_ACC_W_MaxDurationOverThreshold(void *handle, u8_t newValue);
status_t LIS2DW12_ACC_R_MaxDurationOverThreshold(void *handle, u8_t *value);

/*******************************************************************************
* Register      : INT_DUR
* Address       : 0X33
* Bit Group Name: QUIET
* Permission    : RW
*******************************************************************************/
#define  	LIS2DW12_ACC_QUIET_MASK  	0x0C
#define  	LIS2DW12_ACC_QUIET_POSITION  	2
status_t  LIS2DW12_ACC_W_QuietTimeAfterTAP(void *handle, u8_t newValue);
status_t LIS2DW12_ACC_R_QuietTimeAfterTAP(void *handle, u8_t *value);

/*******************************************************************************
* Register      : INT_DUR
* Address       : 0X33
* Bit Group Name: LATENCY
* Permission    : RW
*******************************************************************************/
#define  	LIS2DW12_ACC_LATENCY_MASK  	0xF0
#define  	LIS2DW12_ACC_LATENCY_POSITION  	4
status_t  LIS2DW12_ACC_W_DoubleTAP_Latency(void *handle, u8_t newValue);
status_t LIS2DW12_ACC_R_DoubleTAP_Latency(void *handle, u8_t *value);

/*******************************************************************************
* Register      : WAKE_UP_THS
* Address       : 0X34
* Bit Group Name: WK_THS
* Permission    : RW
*******************************************************************************/
#define  	LIS2DW12_ACC_WK_THS_MASK  	0x3F
status_t  LIS2DW12_ACC_W_WakeUpThreshold(void *handle, u8_t newValue);
status_t LIS2DW12_ACC_R_WakeUpThreshold(void *handle, u8_t *value);

/*******************************************************************************
* Register      : WAKE_UP_THS
* Address       : 0X34
* Bit Group Name: SLEEP_ON
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_SLEEP_ON_DISABLE 		 =0x00,
  	LIS2DW12_ACC_SLEEP_ON_ENABLE 		 =0x40,
} LIS2DW12_ACC_SLEEP_ON_t;

#define  	LIS2DW12_ACC_SLEEP_ON_MASK  	0x40
status_t  LIS2DW12_ACC_W_SleepFeature(void *handle, LIS2DW12_ACC_SLEEP_ON_t newValue);
status_t LIS2DW12_ACC_R_SleepFeature(void *handle, LIS2DW12_ACC_SLEEP_ON_t *value);

/*******************************************************************************
* Register      : WAKE_UP_THS
* Address       : 0X34
* Bit Group Name: SINGLE_DOUBLE_TAP
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_SINGLE_DOUBLE_TAP_SINGLE 		 =0x00,
  	LIS2DW12_ACC_SINGLE_DOUBLE_TAP_DOUBLE 		 =0x80,
} LIS2DW12_ACC_SINGLE_DOUBLE_TAP_t;

#define  	LIS2DW12_ACC_SINGLE_DOUBLE_TAP_MASK  	0x80
status_t  LIS2DW12_ACC_W_TAP_mode(void *handle, LIS2DW12_ACC_SINGLE_DOUBLE_TAP_t newValue);
status_t LIS2DW12_ACC_R_TAP_mode(void *handle, LIS2DW12_ACC_SINGLE_DOUBLE_TAP_t *value);

/*******************************************************************************
* Register      : WAKE_UP_DUR
* Address       : 0X35
* Bit Group Name: SLEEP_DUR
* Permission    : RW
*******************************************************************************/
#define  	LIS2DW12_ACC_SLEEP_DUR_MASK  	0x0F
#define  	LIS2DW12_ACC_SLEEP_DUR_POSITION  	0
status_t  LIS2DW12_ACC_W_InactivityBeforeSleep(void *handle, u8_t newValue);
status_t LIS2DW12_ACC_R_InactivityBeforeSleep(void *handle, u8_t *value);

/*******************************************************************************
* Register      : WAKE_UP_DUR
* Address       : 0X35
* Bit Group Name: WAKE_DUR
* Permission    : RW
*******************************************************************************/
#define  	LIS2DW12_ACC_WAKE_DUR_MASK  	0x60
#define  	LIS2DW12_ACC_WAKE_DUR_POSITION  	5
status_t  LIS2DW12_ACC_W_WakeUpDuration(void *handle, u8_t newValue);
status_t LIS2DW12_ACC_R_WakeUpDuration(void *handle, u8_t *value);

/*******************************************************************************
* Register      : WAKE_UP_DUR
* Address       : 0X35
* Bit Group Name: FF_DUR5
* Permission    : RW
*******************************************************************************/
#define  	LIS2DW12_ACC_FF_DUR5_MASK  	0x80
#define  	LIS2DW12_ACC_FF_DUR5_POSITION  	7
status_t  LIS2DW12_ACC_W_FreeFallDuration(void *handle, u8_t newValue);
status_t LIS2DW12_ACC_R_FreeFallDuration(void *handle, u8_t *value);

/*******************************************************************************
* Register      : FREE_FALL
* Address       : 0X36
* Bit Group Name: FF_THS
* Permission    : RW
*******************************************************************************/
#define  	LIS2DW12_ACC_FF_THS_MASK  	0x07
#define  	LIS2DW12_ACC_FF_THS_POSITION  	0
status_t  LIS2DW12_ACC_W_FreeFallThreshold(void *handle, u8_t newValue);
status_t LIS2DW12_ACC_R_FreeFallThreshold(void *handle, u8_t *value);

/*******************************************************************************
* Register      : FREE_FALL
* Address       : 0X36
* Bit Group Name: FF_DUR
* Permission    : RW
*******************************************************************************/
#define  	LIS2DW12_ACC_FF_DUR_MASK  	0x78
#define  	LIS2DW12_ACC_FF_DUR_POSITION  	3
status_t  LIS2DW12_ACC_W_LIS2DW12_ACC_R_FreeFallDuration(void *handle, u8_t newValue);
status_t LIS2DW12_ACC_R_LIS2DW12_ACC_R_FreeFallDuration(void *handle, u8_t *value);

/*******************************************************************************
* Register      : WAKE_UP_SRC
* Address       : 0X38
* Bit Group Name: Z_WU
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_Z_WU_NOT_OCCURRED 		 =0x00,
  	LIS2DW12_ACC_Z_WU_OCCURRED 		 =0x01,
} LIS2DW12_ACC_Z_WU_t;

#define  	LIS2DW12_ACC_Z_WU_MASK  	0x01
status_t LIS2DW12_ACC_R_WakeUpEvent_On_Z(void *handle, LIS2DW12_ACC_Z_WU_t *value);

/*******************************************************************************
* Register      : WAKE_UP_SRC
* Address       : 0X38
* Bit Group Name: Y_WU
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_Y_WU_NOT_OCCURRED 		 =0x00,
  	LIS2DW12_ACC_Y_WU_OCCURRED 		 =0x02,
} LIS2DW12_ACC_Y_WU_t;

#define  	LIS2DW12_ACC_Y_WU_MASK  	0x02
status_t LIS2DW12_ACC_R_WakeUpEvent_On_Y(void *handle, LIS2DW12_ACC_Y_WU_t *value);

/*******************************************************************************
* Register      : WAKE_UP_SRC
* Address       : 0X38
* Bit Group Name: X_WU
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_X_WU_NOT_OCCURRED 		 =0x00,
  	LIS2DW12_ACC_X_WU_OCCURRED 		 =0x04,
} LIS2DW12_ACC_X_WU_t;

#define  	LIS2DW12_ACC_X_WU_MASK  	0x04
status_t LIS2DW12_ACC_R_LIS2DW12_ACC_R_WakeUpEvent_On_X(void *handle, LIS2DW12_ACC_X_WU_t *value);

/*******************************************************************************
* Register      : TAP_SRC
* Address       : 0X39
* Bit Group Name: Z_TAP
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_Z_TAP_NOT_OCCURRED 		 =0x00,
  	LIS2DW12_ACC_Z_TAP_OCCURRED 		 =0x01,
} LIS2DW12_ACC_Z_TAP_t;

#define  	LIS2DW12_ACC_Z_TAP_MASK  	0x01
status_t LIS2DW12_ACC_R_TapOn_Z_event(void *handle, LIS2DW12_ACC_Z_TAP_t *value);

/*******************************************************************************
* Register      : TAP_SRC
* Address       : 0X39
* Bit Group Name: Y_TAP
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_Y_TAP_NOT_OCCURRED 		 =0x00,
  	LIS2DW12_ACC_Y_TAP_OCCURRED 		 =0x02,
} LIS2DW12_ACC_Y_TAP_t;

#define  	LIS2DW12_ACC_Y_TAP_MASK  	0x02
status_t LIS2DW12_ACC_R_TapOn_Y_event(void *handle, LIS2DW12_ACC_Y_TAP_t *value);

/*******************************************************************************
* Register      : TAP_SRC
* Address       : 0X39
* Bit Group Name: X_TAP
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_X_TAP_NOT_OCCURRED 		 =0x00,
  	LIS2DW12_ACC_X_TAP_OCCURRED 		 =0x04,
} LIS2DW12_ACC_X_TAP_t;

#define  	LIS2DW12_ACC_X_TAP_MASK  	0x04
status_t LIS2DW12_ACC_R_TapOn_X_event(void *handle, LIS2DW12_ACC_X_TAP_t *value);

/*******************************************************************************
* Register      : TAP_SRC
* Address       : 0X39
* Bit Group Name: TAP_SIGN
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_TAP_SIGN_POSITIVE 		 =0x00,
  	LIS2DW12_ACC_TAP_SIGN_NEGATIVE 		 =0x08,
} LIS2DW12_ACC_TAP_SIGN_t;

#define  	LIS2DW12_ACC_TAP_SIGN_MASK  	0x08
status_t LIS2DW12_ACC_R_TAP_sign(void *handle, LIS2DW12_ACC_TAP_SIGN_t *value);

/*******************************************************************************
* Register      : TAP_SRC
* Address       : 0X39
* Bit Group Name: TAP_IA
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_TAP_IA_NOT_OCCURRED 		 =0x00,
  	LIS2DW12_ACC_TAP_IA_OCCURRED 		 =0x40,
} LIS2DW12_ACC_TAP_IA_t;

#define  	LIS2DW12_ACC_TAP_IA_MASK  	0x40
status_t LIS2DW12_ACC_R_TAP_event(void *handle, LIS2DW12_ACC_TAP_IA_t *value);

/*******************************************************************************
* Register      : SIXD_SRC
* Address       : 0X3A
* Bit Group Name: XL
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_XL_UNDER_THRESHOLD 		 =0x00,
  	LIS2DW12_ACC_XL_OVER_THRESHOLD 		 =0x01,
} LIS2DW12_ACC_XL_t;

#define  	LIS2DW12_ACC_XL_MASK  	0x01
status_t LIS2DW12_ACC_R_6D_X_Low(void *handle, LIS2DW12_ACC_XL_t *value);

/*******************************************************************************
* Register      : SIXD_SRC
* Address       : 0X3A
* Bit Group Name: XH
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_XH_UNDER_THRESHOLD 		 =0x00,
  	LIS2DW12_ACC_XH_OVER_THRESHOLD 		 =0x02,
} LIS2DW12_ACC_XH_t;

#define  	LIS2DW12_ACC_XH_MASK  	0x02
status_t LIS2DW12_ACC_R_6D_X_High(void *handle, LIS2DW12_ACC_XH_t *value);

/*******************************************************************************
* Register      : SIXD_SRC
* Address       : 0X3A
* Bit Group Name: YL
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_YL_UNDER_THRESHOLD 		 =0x00,
  	LIS2DW12_ACC_YL_OVER_THRESHOLD 		 =0x04,
} LIS2DW12_ACC_YL_t;

#define  	LIS2DW12_ACC_YL_MASK  	0x04
status_t LIS2DW12_ACC_R_6D_Y_Low(void *handle, LIS2DW12_ACC_YL_t *value);

/*******************************************************************************
* Register      : SIXD_SRC
* Address       : 0X3A
* Bit Group Name: YH
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_YH_UNDER_THRESHOLD 		 =0x00,
  	LIS2DW12_ACC_YH_OVER_THRESHOLD 		 =0x08,
} LIS2DW12_ACC_YH_t;

#define  	LIS2DW12_ACC_YH_MASK  	0x08
status_t LIS2DW12_ACC_R_6D_Y_High(void *handle, LIS2DW12_ACC_YH_t *value);

/*******************************************************************************
* Register      : SIXD_SRC
* Address       : 0X3A
* Bit Group Name: ZL
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_ZL_UNDER_THRESHOLD 		 =0x00,
  	LIS2DW12_ACC_ZL_OVER_THRESHOLD 		 =0x10,
} LIS2DW12_ACC_ZL_t;

#define  	LIS2DW12_ACC_ZL_MASK  	0x10
status_t LIS2DW12_ACC_R_6D_Z_Low(void *handle, LIS2DW12_ACC_ZL_t *value);

/*******************************************************************************
* Register      : SIXD_SRC
* Address       : 0X3A
* Bit Group Name: ZH
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_ZH_UNDER_THRESHOLD 		 =0x00,
  	LIS2DW12_ACC_ZH_OVER_THRESHOLD 		 =0x20,
} LIS2DW12_ACC_ZH_t;

#define  	LIS2DW12_ACC_ZH_MASK  	0x20
status_t LIS2DW12_ACC_R_6D_Z_High(void *handle, LIS2DW12_ACC_ZH_t *value);

/*******************************************************************************
* Register      : ALL_INT_SRC
* Address       : 0X3B
* Bit Group Name: SLEEP_CHANGE_IA
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_SLEEP_CHANGE_IA_NOT_OCCURRED 		 =0x00,
  	LIS2DW12_ACC_SLEEP_CHANGE_IA_OCCURRED 		 =0x20,
} LIS2DW12_ACC_SLEEP_CHANGE_IA_t;

#define  	LIS2DW12_ACC_SLEEP_CHANGE_IA_MASK  	0x20
status_t LIS2DW12_ACC_R_SleepChange_event(void *handle, LIS2DW12_ACC_SLEEP_CHANGE_IA_t *value);

/*******************************************************************************
* Register      : ABS_INT_X
* Address       : 0X3C
* Bit Group Name: ABS_INT_X
* Permission    : RW
*******************************************************************************/
#define  	LIS2DW12_ACC_ABS_INT_X_MASK  	0xFF
#define  	LIS2DW12_ACC_ABS_INT_X_POSITION  	0
status_t  LIS2DW12_ACC_W_WakeUpOffset_X(void *handle, u8_t newValue);
status_t LIS2DW12_ACC_R_WakeUpOffset_X(void *handle, u8_t *value);

/*******************************************************************************
* Register      : ABS_INT_Y
* Address       : 0X3D
* Bit Group Name: ABS_INT_Y
* Permission    : RW
*******************************************************************************/
#define  	LIS2DW12_ACC_ABS_INT_Y_MASK  	0xFF
#define  	LIS2DW12_ACC_ABS_INT_Y_POSITION  	0
status_t  LIS2DW12_ACC_W_WakeUpOffset_Y(void *handle, u8_t newValue);
status_t LIS2DW12_ACC_R_WakeUpOffset_Y(void *handle, u8_t *value);

/*******************************************************************************
* Register      : ABS_INT_Z
* Address       : 0X3E
* Bit Group Name: ABS_INT_Z
* Permission    : RW
*******************************************************************************/
#define  	LIS2DW12_ACC_ABS_INT_Z_MASK  	0x7F
#define  	LIS2DW12_ACC_ABS_INT_Z_POSITION  	0
status_t  LIS2DW12_ACC_W_WakeUpOffset_Z(void *handle, u8_t newValue);
status_t LIS2DW12_ACC_R_WakeUpOffset_Z(void *handle, u8_t *value);

/*******************************************************************************
* Register      : ABS_INT_CFG
* Address       : 0X3F
* Bit Group Name: LPASS_ON6D
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_LPASS_ON6D_ODR_DIV_2 		 =0x00,
  	LIS2DW12_ACC_LPASS_ON6D_USE_LPF2 		 =0x01,
} LIS2DW12_ACC_LPASS_ON6D_t;

#define  	LIS2DW12_ACC_LPASS_ON6D_MASK  	0x01
status_t  LIS2DW12_ACC_W_6D_FilterInput(void *handle, LIS2DW12_ACC_LPASS_ON6D_t newValue);
status_t LIS2DW12_ACC_R_6D_FilterInput(void *handle, LIS2DW12_ACC_LPASS_ON6D_t *value);

/*******************************************************************************
* Register      : ABS_INT_CFG
* Address       : 0X3F
* Bit Group Name: HP_REF_MODE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_HP_REF_MODE_NONE 		 =0x00,
  	LIS2DW12_ACC_HP_REF_MODE_ENABLE 		 =0x02,
} LIS2DW12_ACC_HP_REF_MODE_t;

#define  	LIS2DW12_ACC_HP_REF_MODE_MASK  	0x02
status_t  LIS2DW12_ACC_W_HP_FilterReference(void *handle, LIS2DW12_ACC_HP_REF_MODE_t newValue);
status_t LIS2DW12_ACC_R_HP_FilterReference(void *handle, LIS2DW12_ACC_HP_REF_MODE_t *value);

/*******************************************************************************
* Register      : ABS_INT_CFG
* Address       : 0X3F
* Bit Group Name: USR_OFF_W
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_USR_OFF_W_977_ug 		 =0x00,
  	LIS2DW12_ACC_USR_OFF_W_15mg 		 =0x04,
} LIS2DW12_ACC_USR_OFF_W_t;

#define  	LIS2DW12_ACC_USR_OFF_W_MASK  	0x04
status_t  LIS2DW12_ACC_W_UserOffsetWeight(void *handle, LIS2DW12_ACC_USR_OFF_W_t newValue);
status_t LIS2DW12_ACC_R_UserOffsetWeight(void *handle, LIS2DW12_ACC_USR_OFF_W_t *value);

/*******************************************************************************
* Register      : ABS_INT_CFG
* Address       : 0X3F
* Bit Group Name: USR_OFF_ON_WU
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_USR_OFF_ON_WU_DISABLE 		 =0x00,
  	LIS2DW12_ACC_USR_OFF_ON_WU_ENABLE 		 =0x08,
} LIS2DW12_ACC_USR_OFF_ON_WU_t;

#define  	LIS2DW12_ACC_USR_OFF_ON_WU_MASK  	0x08
status_t  LIS2DW12_ACC_W_UserOffsetOnWakeUp(void *handle, LIS2DW12_ACC_USR_OFF_ON_WU_t newValue);
status_t LIS2DW12_ACC_R_UserOffsetOnWakeUp(void *handle, LIS2DW12_ACC_USR_OFF_ON_WU_t *value);

/*******************************************************************************
* Register      : ABS_INT_CFG
* Address       : 0X3F
* Bit Group Name: USR_OFF_ON_OUT
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_USR_OFF_ON_OUT_DISABLE 		 =0x00,
  	LIS2DW12_ACC_USR_OFF_ON_OUT_ENABLE 		 =0x10,
} LIS2DW12_ACC_USR_OFF_ON_OUT_t;

#define  	LIS2DW12_ACC_USR_OFF_ON_OUT_MASK  	0x10
status_t  LIS2DW12_ACC_W_UserOffsetOnOutputs(void *handle, LIS2DW12_ACC_USR_OFF_ON_OUT_t newValue);
status_t LIS2DW12_ACC_R_UserOffsetOnOutputs(void *handle, LIS2DW12_ACC_USR_OFF_ON_OUT_t *value);

/*******************************************************************************
* Register      : ABS_INT_CFG
* Address       : 0X3F
* Bit Group Name: INTERRUPTS_ENABLE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_INTERRUPTS_ENABLE_DISABLE 		 =0x00,
  	LIS2DW12_ACC_INTERRUPTS_ENABLE_ENABLE 		 =0x20,
} LIS2DW12_ACC_INTERRUPTS_ENABLE_t;

#define  	LIS2DW12_ACC_INTERRUPTS_ENABLE_MASK  	0x20
status_t  LIS2DW12_ACC_W_HardwarePin(void *handle, LIS2DW12_ACC_INTERRUPTS_ENABLE_t newValue);
status_t LIS2DW12_ACC_R_HardwarePin(void *handle, LIS2DW12_ACC_INTERRUPTS_ENABLE_t *value);

/*******************************************************************************
* Register      : ABS_INT_CFG
* Address       : 0X3F
* Bit Group Name: INT2_ON_INT1
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_INT2_ON_INT1_DISABLE 		 =0x00,
  	LIS2DW12_ACC_INT2_ON_INT1_ENABLE 		 =0x40,
} LIS2DW12_ACC_INT2_ON_INT1_t;

#define  	LIS2DW12_ACC_INT2_ON_INT1_MASK  	0x40
status_t  LIS2DW12_ACC_W_UseInt1_PinOnly(void *handle, LIS2DW12_ACC_INT2_ON_INT1_t newValue);
status_t LIS2DW12_ACC_R_UseInt1_PinOnly(void *handle, LIS2DW12_ACC_INT2_ON_INT1_t *value);

/*******************************************************************************
* Register      : ABS_INT_CFG
* Address       : 0X3F
* Bit Group Name: DRDY_PULSED
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LIS2DW12_ACC_DRDY_PULSED_LATCHED 		 =0x00,
  	LIS2DW12_ACC_DRDY_PULSED_PULSED 		 =0x80,
} LIS2DW12_ACC_DRDY_PULSED_t;

#define  	LIS2DW12_ACC_DRDY_PULSED_MASK  	0x80
status_t  LIS2DW12_ACC_W_DataReady(void *handle, LIS2DW12_ACC_DRDY_PULSED_t newValue);
status_t LIS2DW12_ACC_R_DataReady(void *handle, LIS2DW12_ACC_DRDY_PULSED_t *value);


#ifdef __cplusplus
}
#endif

#endif /* __LIS2DW12_ACC_DRIVER__H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
