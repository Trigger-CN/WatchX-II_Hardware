/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
* File Name          : LSM330_ACC_driver.h
* Author             : MEMS Application Team
* Version            : v1.1
* Date               : 20 December 2016   
* Description        : LSM330 Platform Independent Driver
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
#ifndef __LSM330_ACC_DRIVER__H
#define __LSM330_ACC_DRIVER__H

#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

//these could change accordingly with the architecture

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

#define LSM330_ACC_I2C_ADDRESS         0X3A

/************** Who am I  *******************/

#define LSM330_ACC_WHO_AM_I         0X40

/************** Device Register  *******************/
#define LSM330_ACC_OUT_T  	0X0C
#define LSM330_ACC_INFO1  	0X0D
#define LSM330_ACC_INFO2  	0X0E
#define LSM330_ACC_WHO_AM_I_REG  	0X0F
#define LSM330_ACC_OFF_X  	0X10
#define LSM330_ACC_OFF_Y  	0X11
#define LSM330_ACC_OFF_Z  	0X12
#define LSM330_ACC_CS_X  	0X13
#define LSM330_ACC_CS_Y  	0X14
#define LSM330_ACC_CS_Z  	0X15
#define LSM330_ACC_LC_L  	0X16
#define LSM330_ACC_LC_H  	0X17
#define LSM330_ACC_STAT  	0X18
#define LSM330_ACC_PEAK1  	0X19
#define LSM330_ACC_PEAK2  	0X1A
#define LSM330_ACC_VFC_1  	0X1B
#define LSM330_ACC_VFC_2  	0X1C
#define LSM330_ACC_VFC_3  	0X1D
#define LSM330_ACC_VFC_4  	0X1E
#define LSM330_ACC_THRS3  	0X1F
#define LSM330_ACC_CTRL4  	0X20
#define LSM330_ACC_CTRL1  	0X21
#define LSM330_ACC_CTRL2  	0X22
#define LSM330_ACC_CTRL3  	0X23
#define LSM330_ACC_CTRL5  	0X24
#define LSM330_ACC_CTRL6  	0X25
#define LSM330_ACC_STATUS  	0X27
#define LSM330_ACC_OUT_X_L  	0X28
#define LSM330_ACC_OUT_X_H  	0X29
#define LSM330_ACC_OUT_Y_L  	0X2A
#define LSM330_ACC_OUT_Y_H  	0X2B
#define LSM330_ACC_OUT_Z_L  	0X2C
#define LSM330_ACC_OUT_Z_H  	0X2D
#define LSM330_ACC_FIFO_CTRL  	0X2E
#define LSM330_ACC_FIFO_SRC  	0X2F
#define LSM330_ACC_ST1_PR0  	0X40
#define LSM330_ACC_ST1_PR1  	0X41
#define LSM330_ACC_ST1_PR2  	0X42
#define LSM330_ACC_ST1_PR3  	0X43
#define LSM330_ACC_ST1_PR4  	0X44
#define LSM330_ACC_ST1_PR5  	0X45
#define LSM330_ACC_ST1_PR6  	0X46
#define LSM330_ACC_ST1_PR7  	0X47
#define LSM330_ACC_ST1_PR8  	0X48
#define LSM330_ACC_ST1_PR9  	0X49
#define LSM330_ACC_ST1_PR10  	0X4A
#define LSM330_ACC_ST1_PR11  	0X4B
#define LSM330_ACC_ST1_PR12  	0X4C
#define LSM330_ACC_ST1_PR13  	0X4D
#define LSM330_ACC_ST1_PR14  	0X4E
#define LSM330_ACC_ST1_PR15  	0X4F
#define LSM330_ACC_TIM4_SM1  	0X50
#define LSM330_ACC_TIM3_SM1  	0X51
#define LSM330_ACC_TIM2_L_SM1  	0X52
#define LSM330_ACC_TIM2_H_SM1  	0X53
#define LSM330_ACC_TIM1_L_SM1  	0X54
#define LSM330_ACC_TIM1_H_SM1  	0X55
#define LSM330_ACC_THRS2_SM1  	0X56
#define LSM330_ACC_THRS1_SM1  	0X57
#define LSM330_ACC_MASKB_SM1  	0X59
#define LSM330_ACC_MASKA_SM1  	0X5A
#define LSM330_ACC_SETT_SM1  	0X5B
#define LSM330_ACC_PR_SM1  	0X5C
#define LSM330_ACC_TC1_L_SM1  	0X5D
#define LSM330_ACC_TC1_H_SM1  	0X5E
#define LSM330_ACC_OUTS_SM1  	0X5F
#define LSM330_ACC_ST2_PR0  	0X60
#define LSM330_ACC_ST2_PR1  	0X61
#define LSM330_ACC_ST2_PR2  	0X62
#define LSM330_ACC_ST2_PR3  	0X63
#define LSM330_ACC_ST2_PR4  	0X64
#define LSM330_ACC_ST2_PR5  	0X65
#define LSM330_ACC_ST2_PR6  	0X66
#define LSM330_ACC_ST2_PR7  	0X67
#define LSM330_ACC_ST2_PR8  	0X68
#define LSM330_ACC_ST2_PR9  	0X69
#define LSM330_ACC_ST2_PR10  	0X6A
#define LSM330_ACC_ST2_PR11  	0X6B
#define LSM330_ACC_ST2_PR12  	0X6C
#define LSM330_ACC_ST2_PR13  	0X6D
#define LSM330_ACC_ST2_PR14  	0X6E
#define LSM330_ACC_ST2_PR15  	0X6F
#define LSM330_ACC_TIM4_SM2  	0X70
#define LSM330_ACC_TIM3_SM2  	0X71
#define LSM330_ACC_TIM2_L_SM2  	0X72
#define LSM330_ACC_TIM2_H_SM2  	0X73
#define LSM330_ACC_TIM1_L_SM2  	0X74
#define LSM330_ACC_TIM1_H_SM2  	0X75
#define LSM330_ACC_THRS2_SM2  	0X76
#define LSM330_ACC_THRS1_SM2  	0X77
#define LSM330_ACC_DES_SM2  	0X78
#define LSM330_ACC_MASKB_SM2  	0X79
#define LSM330_ACC_MASKA_SM2  	0X7A
#define LSM330_ACC_SETT_SM2  	0X7B
#define LSM330_ACC_PR_SM2  	0X7C
#define LSM330_ACC_TC_L_SM2  	0X7D
#define LSM330_ACC_TC_H_SM2  	0X7E
#define LSM330_ACC_OUTS_SM2  	0X7F

/************** Generic Function  *******************/

/*******************************************************************************
* Register      : Generic - All
* Address       : Generic - All
* Bit Group Name: None
* Permission    : W
*******************************************************************************/
status_t LSM330_ACC_WriteReg( void *handle, u8_t Reg, u8_t *Bufp, u16_t len );

/*******************************************************************************
* Register      : Generic - All
* Address       : Generic - All
* Bit Group Name: None
* Permission    : R
*******************************************************************************/
status_t LSM330_ACC_ReadReg( void *handle, u8_t Reg, u8_t *Bufp, u16_t len );

/**************** Base Function  *******************/

/*******************************************************************************
* Register      : WHO_AM_I_REG
* Address       : 0X0F
* Bit Group Name: WHO_AM_I_BYTE
* Permission    : RO
*******************************************************************************/
#define  	LSM330_ACC_WHO_AM_I_BYTE_MASK  	0xFF
#define  	LSM330_ACC_WHO_AM_I_BYTE_POSITION  	0
status_t LSM330_ACC_R_WHO_AM_I_(void *handle, u8_t *value);

/*******************************************************************************
* Register      : CTRL4
* Address       : 0X20
* Bit Group Name: ODR
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_ODR_POWER_DOWN 		 =0x00,
  	LSM330_ACC_ODR_3Hz 		 =0x10,
  	LSM330_ACC_ODR_6Hz 		 =0x20,
  	LSM330_ACC_ODR_12Hz 		 =0x30,
  	LSM330_ACC_ODR_25Hz 		 =0x40,
  	LSM330_ACC_ODR_50Hz 		 =0x50,
  	LSM330_ACC_ODR_100Hz 		 =0x60,
  	LSM330_ACC_ODR_400Hz 		 =0x70,
  	LSM330_ACC_ODR_800Hz 		 =0x80,
  	LSM330_ACC_ODR_1600Hz 		 =0x90,
} LSM330_ACC_ODR_t;

#define  	LSM330_ACC_ODR_MASK  	0xF0
status_t  LSM330_ACC_W_OutputDataRate(void *handle, LSM330_ACC_ODR_t newValue);
status_t LSM330_ACC_R_OutputDataRate(void *handle, LSM330_ACC_ODR_t *value);

/*******************************************************************************
* Register      : CTRL5
* Address       : 0X24
* Bit Group Name: FSCALE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_FSCALE_2g 		 =0x00,
  	LSM330_ACC_FSCALE_4g 		 =0x08,
  	LSM330_ACC_FSCALE_6g 		 =0x10,
  	LSM330_ACC_FSCALE_8g 		 =0x18,
  	LSM330_ACC_FSCALE_16g 		 =0x20,
} LSM330_ACC_FSCALE_t;

#define  	LSM330_ACC_FSCALE_MASK  	0x38
status_t  LSM330_ACC_W_FullScale(void *handle, LSM330_ACC_FSCALE_t newValue);
status_t LSM330_ACC_R_FullScale(void *handle, LSM330_ACC_FSCALE_t *value);

/*******************************************************************************
* Register      : CTRL4
* Address       : 0X20
* Bit Group Name: BDU
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_BDU_DISABLE 		 =0x00,
  	LSM330_ACC_BDU_ENABLE 		 =0x08,
} LSM330_ACC_BDU_t;

#define  	LSM330_ACC_BDU_MASK  	0x08
status_t  LSM330_ACC_W_BlockDataUpdate(void *handle, LSM330_ACC_BDU_t newValue);
status_t LSM330_ACC_R_BlockDataUpdate(void *handle, LSM330_ACC_BDU_t *value);

/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : Acceleration
* Permission    : RO 
*******************************************************************************/
status_t LSM330_ACC_Get_Acceleration(void *handle, u8_t *buff); 

/**************** Advanced Function  *******************/

/*******************************************************************************
* Register      : INFO1
* Address       : 0X0D
* Bit Group Name: INFO1_BYTE
* Permission    : RO
*******************************************************************************/
#define  	LSM330_ACC_INFO1_BYTE_MASK  	0xFF
#define  	LSM330_ACC_INFO1_BYTE_POSITION  	0
status_t LSM330_ACC_R_RegisterInfo1(void *handle, u8_t *value);

/*******************************************************************************
* Register      : INFO2
* Address       : 0X0E
* Bit Group Name: INFO2_BYTE
* Permission    : RO
*******************************************************************************/
#define  	LSM330_ACC_INFO2_BYTE_MASK  	0xFF
#define  	LSM330_ACC_INFO2_BYTE_POSITION  	0
status_t LSM330_ACC_R_RegisterInfo2(void *handle, u8_t *value);

/*******************************************************************************
* Register      : OFF_X
* Address       : 0X10
* Bit Group Name: OFF_X_BITS
* Permission    : RW
*******************************************************************************/
#define  	LSM330_ACC_OFF_X_BITS_MASK  	0xFF
#define  	LSM330_ACC_OFF_X_BITS_POSITION  	0
status_t  LSM330_ACC_W_OffsetCorrectionX(void *handle, u8_t newValue);
status_t LSM330_ACC_R_OffsetCorrectionX(void *handle, u8_t *value);

/*******************************************************************************
* Register      : OFF_Y
* Address       : 0X11
* Bit Group Name: OFF_Y_BITS
* Permission    : RW
*******************************************************************************/
#define  	LSM330_ACC_OFF_Y_BITS_MASK  	0xFF
#define  	LSM330_ACC_OFF_Y_BITS_POSITION  	0
status_t  LSM330_ACC_W_OffsetCorrectionY(void *handle, u8_t newValue);
status_t LSM330_ACC_R_OffsetCorrectionY(void *handle, u8_t *value);

/*******************************************************************************
* Register      : OFF_Z
* Address       : 0X12
* Bit Group Name: OFF_Z_BITS
* Permission    : RW
*******************************************************************************/
#define  	LSM330_ACC_OFF_Z_BITS_MASK  	0xFF
#define  	LSM330_ACC_OFF_Z_BITS_POSITION  	0
status_t  LSM330_ACC_W_OffsetCorrectionZ(void *handle, u8_t newValue);
status_t LSM330_ACC_R_OffsetCorrectionZ(void *handle, u8_t *value);

/*******************************************************************************
* Register      : CS_X
* Address       : 0X13
* Bit Group Name: CS_X_BITS
* Permission    : RW
*******************************************************************************/
#define  	LSM330_ACC_CS_X_BITS_MASK  	0xFF
#define  	LSM330_ACC_CS_X_BITS_POSITION  	0
status_t  LSM330_ACC_W_ConstantShiftX(void *handle, u8_t newValue);
status_t LSM330_ACC_R_ConstantShiftX(void *handle, u8_t *value);

/*******************************************************************************
* Register      : CS_Y
* Address       : 0X14
* Bit Group Name: CS_Y_BITS
* Permission    : RW
*******************************************************************************/
#define  	LSM330_ACC_CS_Y_BITS_MASK  	0xFF
#define  	LSM330_ACC_CS_Y_BITS_POSITION  	0
status_t  LSM330_ACC_W_ConstantShiftY(void *handle, u8_t newValue);
status_t LSM330_ACC_R_ConstantShiftY(void *handle, u8_t *value);

/*******************************************************************************
* Register      : CS_Z
* Address       : 0X15
* Bit Group Name: CS_Z_BITS
* Permission    : RW
*******************************************************************************/
#define  	LSM330_ACC_CS_Z_BITS_MASK  	0xFF
#define  	LSM330_ACC_CS_Z_BITS_POSITION  	0
status_t  LSM330_ACC_W_ConstantShiftZ(void *handle, u8_t newValue);
status_t LSM330_ACC_R_ConstantShiftZ(void *handle, u8_t *value);

/*******************************************************************************
* Register      : STAT
* Address       : 0X18
* Bit Group Name: DRDY
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_ACC_DRDY_DOWN 		 =0x00,
  	LSM330_ACC_DRDY_UP 		 =0x01,
} LSM330_ACC_DRDY_t;

#define  	LSM330_ACC_DRDY_MASK  	0x01
status_t LSM330_ACC_R_DataReadyFlag(void *handle, LSM330_ACC_DRDY_t *value);

/*******************************************************************************
* Register      : STAT
* Address       : 0X18
* Bit Group Name: DOR
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_ACC_DOR_DOWN 		 =0x00,
  	LSM330_ACC_DOR_UP 		 =0x02,
} LSM330_ACC_DOR_t;

#define  	LSM330_ACC_DOR_MASK  	0x02
status_t LSM330_ACC_R_DataOverrunFlag(void *handle, LSM330_ACC_DOR_t *value);

/*******************************************************************************
* Register      : STAT
* Address       : 0X18
* Bit Group Name: INT_SM2
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_ACC_INT_SM2_DOWN 		 =0x00,
  	LSM330_ACC_INT_SM2_UP 		 =0x04,
} LSM330_ACC_INT_SM2_t;

#define  	LSM330_ACC_INT_SM2_MASK  	0x04
status_t LSM330_ACC_R_SM2_InterruptFlag(void *handle, LSM330_ACC_INT_SM2_t *value);

/*******************************************************************************
* Register      : STAT
* Address       : 0X18
* Bit Group Name: INT_SM1
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_ACC_INT_SM1_DOWN 		 =0x00,
  	LSM330_ACC_INT_SM1_UP 		 =0x08,
} LSM330_ACC_INT_SM1_t;

#define  	LSM330_ACC_INT_SM1_MASK  	0x08
status_t LSM330_ACC_R_SM1_InterruptFlag(void *handle, LSM330_ACC_INT_SM1_t *value);

/*******************************************************************************
* Register      : STAT
* Address       : 0X18
* Bit Group Name: SYNC2
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SYNC2_DOWN 		 =0x00,
  	LSM330_ACC_SYNC2_UP 		 =0x10,
} LSM330_ACC_SYNC2_t;

#define  	LSM330_ACC_SYNC2_MASK  	0x10
status_t LSM330_ACC_R_SM2_WaitFlag(void *handle, LSM330_ACC_SYNC2_t *value);

/*******************************************************************************
* Register      : STAT
* Address       : 0X18
* Bit Group Name: SYNC1
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SYNC1_DOWN 		 =0x00,
  	LSM330_ACC_SYNC1_UP 		 =0x20,
} LSM330_ACC_SYNC1_t;

#define  	LSM330_ACC_SYNC1_MASK  	0x20
status_t LSM330_ACC_R_SM1_WaitFlag(void *handle, LSM330_ACC_SYNC1_t *value);

/*******************************************************************************
* Register      : STAT
* Address       : 0X18
* Bit Group Name: SYNCW
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SYNCW_DOWN 		 =0x00,
  	LSM330_ACC_SYNCW_UP 		 =0x40,
} LSM330_ACC_SYNCW_t;

#define  	LSM330_ACC_SYNCW_MASK  	0x40
status_t LSM330_ACC_R_ExternalEventFlag(void *handle, LSM330_ACC_SYNCW_t *value);

/*******************************************************************************
* Register      : STAT
* Address       : 0X18
* Bit Group Name: LONG
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_ACC_LONG_DOWN 		 =0x00,
  	LSM330_ACC_LONG_UP 		 =0x80,
} LSM330_ACC_LONG_t;

#define  	LSM330_ACC_LONG_MASK  	0x80
status_t LSM330_ACC_R_LongCounterInterruptFlag(void *handle, LSM330_ACC_LONG_t *value);

/*******************************************************************************
* Register      : PEAK1
* Address       : 0X19
* Bit Group Name: PEAK1_BIT
* Permission    : RO
*******************************************************************************/
#define  	LSM330_ACC_PEAK1_BIT_MASK  	0xFF
#define  	LSM330_ACC_PEAK1_BIT_POSITION  	0
status_t LSM330_ACC_R_PeakSM1(void *handle, u8_t *value);

/*******************************************************************************
* Register      : PEAK2
* Address       : 0X1A
* Bit Group Name: PEAK2_BIT
* Permission    : RO
*******************************************************************************/
#define  	LSM330_ACC_PEAK2_BIT_MASK  	0xFF
#define  	LSM330_ACC_PEAK2_BIT_POSITION  	0
status_t LSM330_ACC_R_PeakSM2(void *handle, u8_t *value);

/*******************************************************************************
* Register      : THRS3
* Address       : 0X1F
* Bit Group Name: THRS3_BIT
* Permission    : RW
*******************************************************************************/
#define  	LSM330_ACC_THRS3_BIT_MASK  	0xFF
#define  	LSM330_ACC_THRS3_BIT_POSITION  	0
status_t  LSM330_ACC_W_Threshold3(void *handle, u8_t newValue);
status_t LSM330_ACC_R_Threshold3(void *handle, u8_t *value);

/*******************************************************************************
* Register      : CTRL4
* Address       : 0X20
* Bit Group Name: XEN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_XEN_DISABLE 		 =0x00,
  	LSM330_ACC_XEN_ENABLE 		 =0x01,
} LSM330_ACC_XEN_t;

#define  	LSM330_ACC_XEN_MASK  	0x01
status_t  LSM330_ACC_W_AxisX(void *handle, LSM330_ACC_XEN_t newValue);
status_t LSM330_ACC_R_AxisX(void *handle, LSM330_ACC_XEN_t *value);

/*******************************************************************************
* Register      : CTRL4
* Address       : 0X20
* Bit Group Name: YEN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_YEN_DISABLE 		 =0x00,
  	LSM330_ACC_YEN_ENABLE 		 =0x02,
} LSM330_ACC_YEN_t;

#define  	LSM330_ACC_YEN_MASK  	0x02
status_t  LSM330_ACC_W_AxisY(void *handle, LSM330_ACC_YEN_t newValue);
status_t LSM330_ACC_R_AxisY(void *handle, LSM330_ACC_YEN_t *value);

/*******************************************************************************
* Register      : CTRL4
* Address       : 0X20
* Bit Group Name: ZEN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_ZEN_DISABLE 		 =0x00,
  	LSM330_ACC_ZEN_ENABLE 		 =0x04,
} LSM330_ACC_ZEN_t;

#define  	LSM330_ACC_ZEN_MASK  	0x04
status_t  LSM330_ACC_W_AxisZ(void *handle, LSM330_ACC_ZEN_t newValue);
status_t LSM330_ACC_R_AxisZ(void *handle, LSM330_ACC_ZEN_t *value);

/*******************************************************************************
* Register      : CTRL1
* Address       : 0X21
* Bit Group Name: SM1_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM1_EN_DISABLE 		 =0x00,
  	LSM330_ACC_SM1_EN_ENABLE 		 =0x01,
} LSM330_ACC_SM1_EN_t;

#define  	LSM330_ACC_SM1_EN_MASK  	0x01
status_t  LSM330_ACC_W_StateMachine1(void *handle, LSM330_ACC_SM1_EN_t newValue);
status_t LSM330_ACC_R_StateMachine1(void *handle, LSM330_ACC_SM1_EN_t *value);

/*******************************************************************************
* Register      : CTRL1
* Address       : 0X21
* Bit Group Name: SM1_PIN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM1_PIN_INT1 		 =0x00,
  	LSM330_ACC_SM1_PIN_INT2 		 =0x08,
} LSM330_ACC_SM1_PIN_t;

#define  	LSM330_ACC_SM1_PIN_MASK  	0x08
status_t  LSM330_ACC_W_SM1_PinSelector(void *handle, LSM330_ACC_SM1_PIN_t newValue);
status_t LSM330_ACC_R_SM1_PinSelector(void *handle, LSM330_ACC_SM1_PIN_t *value);

/*******************************************************************************
* Register      : CTRL1
* Address       : 0X21
* Bit Group Name: HYST_SM1
* Permission    : RW
*******************************************************************************/
#define  	LSM330_ACC_HYST_SM1_MASK  	0xE0
#define  	LSM330_ACC_HYST_SM1_POSITION  	5
status_t  LSM330_ACC_W_SM1_Hysteresis(void *handle, u8_t newValue);
status_t LSM330_ACC_R_SM1_Hysteresis(void *handle, u8_t *value);

/*******************************************************************************
* Register      : CTRL2
* Address       : 0X22
* Bit Group Name: SM2_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM2_EN_DISABLE 		 =0x00,
  	LSM330_ACC_SM2_EN_ENABLE 		 =0x01,
} LSM330_ACC_SM2_EN_t;

#define  	LSM330_ACC_SM2_EN_MASK  	0x01
status_t  LSM330_ACC_W_StateMachine2(void *handle, LSM330_ACC_SM2_EN_t newValue);
status_t LSM330_ACC_R_StateMachine2(void *handle, LSM330_ACC_SM2_EN_t *value);

/*******************************************************************************
* Register      : CTRL2
* Address       : 0X22
* Bit Group Name: SM2_PIN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM2_PIN_INT1 		 =0x00,
  	LSM330_ACC_SM2_PIN_INT2 		 =0x08,
} LSM330_ACC_SM2_PIN_t;

#define  	LSM330_ACC_SM2_PIN_MASK  	0x08
status_t  LSM330_ACC_W_SM2_PinSelector(void *handle, LSM330_ACC_SM2_PIN_t newValue);
status_t LSM330_ACC_R_SM2_PinSelector(void *handle, LSM330_ACC_SM2_PIN_t *value);

/*******************************************************************************
* Register      : CTRL2
* Address       : 0X22
* Bit Group Name: HYST_SM2
* Permission    : RW
*******************************************************************************/
#define  	LSM330_ACC_HYST_SM2_MASK  	0xE0
#define  	LSM330_ACC_HYST_SM2_POSITION  	5
status_t  LSM330_ACC_W_SM2_Hysteresis(void *handle, u8_t newValue);
status_t LSM330_ACC_R_SM2_Hysteresis(void *handle, u8_t *value);

/*******************************************************************************
* Register      : CTRL3
* Address       : 0X23
* Bit Group Name: STRT
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_STRT_NO 		 =0x00,
  	LSM330_ACC_STRT_RESET 		 =0x01,
} LSM330_ACC_STRT_t;

#define  	LSM330_ACC_STRT_MASK  	0x01
status_t  LSM330_ACC_W_SoftReset(void *handle, LSM330_ACC_STRT_t newValue);
status_t LSM330_ACC_R_SoftReset(void *handle, LSM330_ACC_STRT_t *value);

/*******************************************************************************
* Register      : CTRL3
* Address       : 0X23
* Bit Group Name: VFILT
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_VFILT_DISABLE 		 =0x00,
  	LSM330_ACC_VFILT_ENABLE 		 =0x04,
} LSM330_ACC_VFILT_t;

#define  	LSM330_ACC_VFILT_MASK  	0x04
status_t  LSM330_ACC_W_VectorFilter(void *handle, LSM330_ACC_VFILT_t newValue);
status_t LSM330_ACC_R_VectorFilter(void *handle, LSM330_ACC_VFILT_t *value);

/*******************************************************************************
* Register      : CTRL3
* Address       : 0X23
* Bit Group Name: INT1_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_INT1_EN_DISABLE 		 =0x00,
  	LSM330_ACC_INT1_EN_ENABLE 		 =0x08,
} LSM330_ACC_INT1_EN_t;

#define  	LSM330_ACC_INT1_EN_MASK  	0x08
status_t  LSM330_ACC_W_INT1_Pin(void *handle, LSM330_ACC_INT1_EN_t newValue);
status_t LSM330_ACC_R_INT1_Pin(void *handle, LSM330_ACC_INT1_EN_t *value);

/*******************************************************************************
* Register      : CTRL3
* Address       : 0X23
* Bit Group Name: INT2_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_INT2_EN_DISABLE 		 =0x00,
  	LSM330_ACC_INT2_EN_ENABLE 		 =0x10,
} LSM330_ACC_INT2_EN_t;

#define  	LSM330_ACC_INT2_EN_MASK  	0x10
status_t  LSM330_ACC_W_INT2_Pin(void *handle, LSM330_ACC_INT2_EN_t newValue);
status_t LSM330_ACC_R_INT2_Pin(void *handle, LSM330_ACC_INT2_EN_t *value);

/*******************************************************************************
* Register      : CTRL3
* Address       : 0X23
* Bit Group Name: IEL
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_IEL_LATCHED 		 =0x00,
  	LSM330_ACC_IEL_PULSED 		 =0x20,
} LSM330_ACC_IEL_t;

#define  	LSM330_ACC_IEL_MASK  	0x20
status_t  LSM330_ACC_W_InterruptSignalType(void *handle, LSM330_ACC_IEL_t newValue);
status_t LSM330_ACC_R_InterruptSignalType(void *handle, LSM330_ACC_IEL_t *value);

/*******************************************************************************
* Register      : CTRL3
* Address       : 0X23
* Bit Group Name: IEA
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_IEA_LOW 		 =0x00,
  	LSM330_ACC_IEA_HIGH 		 =0x40,
} LSM330_ACC_IEA_t;

#define  	LSM330_ACC_IEA_MASK  	0x40
status_t  LSM330_ACC_W_InterruptSignalPolarity(void *handle, LSM330_ACC_IEA_t newValue);
status_t LSM330_ACC_R_InterruptSignalPolarity(void *handle, LSM330_ACC_IEA_t *value);

/*******************************************************************************
* Register      : CTRL3
* Address       : 0X23
* Bit Group Name: DR_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_DR_EN_DISABLE 		 =0x00,
  	LSM330_ACC_DR_EN_ENABLE 		 =0x80,
} LSM330_ACC_DR_EN_t;

#define  	LSM330_ACC_DR_EN_MASK  	0x80
status_t  LSM330_ACC_W_DataReadyOnINT1(void *handle, LSM330_ACC_DR_EN_t newValue);
status_t LSM330_ACC_R_DataReadyOnINT1(void *handle, LSM330_ACC_DR_EN_t *value);

/*******************************************************************************
* Register      : CTRL5
* Address       : 0X24
* Bit Group Name: SIM
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SIM_4WIRE 		 =0x00,
  	LSM330_ACC_SIM_3WIRE 		 =0x01,
} LSM330_ACC_SIM_t;

#define  	LSM330_ACC_SIM_MASK  	0x01
status_t  LSM330_ACC_W_SerialInterfaceSPI(void *handle, LSM330_ACC_SIM_t newValue);
status_t LSM330_ACC_R_SerialInterfaceSPI(void *handle, LSM330_ACC_SIM_t *value);

/*******************************************************************************
* Register      : CTRL5
* Address       : 0X24
* Bit Group Name: ST
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_ST_DISABLE 		 =0x00,
  	LSM330_ACC_ST_POSITIVE 		 =0x02,
  	LSM330_ACC_ST_NEGATIVE 		 =0x04,
} LSM330_ACC_ST_t;

#define  	LSM330_ACC_ST_MASK  	0x06
status_t  LSM330_ACC_W_SelfTest(void *handle, LSM330_ACC_ST_t newValue);
status_t LSM330_ACC_R_SelfTest(void *handle, LSM330_ACC_ST_t *value);

/*******************************************************************************
* Register      : CTRL5
* Address       : 0X24
* Bit Group Name: BW
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_BW_800Hz 		 =0x00,
  	LSM330_ACC_BW_200Hz 		 =0x40,
  	LSM330_ACC_BW_400Hz 		 =0x80,
  	LSM330_ACC_BW_50Hz 		 =0xC0,
} LSM330_ACC_BW_t;

#define  	LSM330_ACC_BW_MASK  	0xC0
status_t  LSM330_ACC_W_AntiAliasingFilter(void *handle, LSM330_ACC_BW_t newValue);
status_t LSM330_ACC_R_AntiAliasingFilter(void *handle, LSM330_ACC_BW_t *value);

/*******************************************************************************
* Register      : CTRL6
* Address       : 0X25
* Bit Group Name: I2_BOOT
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_I2_BOOT_DISABLE 		 =0x00,
  	LSM330_ACC_I2_BOOT_ENABLE 		 =0x01,
} LSM330_ACC_I2_BOOT_t;

#define  	LSM330_ACC_I2_BOOT_MASK  	0x01
status_t  LSM330_ACC_W_BootOnINT2(void *handle, LSM330_ACC_I2_BOOT_t newValue);
status_t LSM330_ACC_R_BootOnINT2(void *handle, LSM330_ACC_I2_BOOT_t *value);

/*******************************************************************************
* Register      : CTRL6
* Address       : 0X25
* Bit Group Name: I1_OVERRUN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_I1_OVERRUN_DISABLE 		 =0x00,
  	LSM330_ACC_I1_OVERRUN_ENABLE 		 =0x02,
} LSM330_ACC_I1_OVERRUN_t;

#define  	LSM330_ACC_I1_OVERRUN_MASK  	0x02
status_t  LSM330_ACC_W_FIFO_OverrunOnINT1(void *handle, LSM330_ACC_I1_OVERRUN_t newValue);
status_t LSM330_ACC_R_FIFO_OverrunOnINT1(void *handle, LSM330_ACC_I1_OVERRUN_t *value);

/*******************************************************************************
* Register      : CTRL6
* Address       : 0X25
* Bit Group Name: I1_WTM
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_I1_WTM_DISABLE 		 =0x00,
  	LSM330_ACC_I1_WTM_ENABLE 		 =0x04,
} LSM330_ACC_I1_WTM_t;

#define  	LSM330_ACC_I1_WTM_MASK  	0x04
status_t  LSM330_ACC_W_FIFO_WatermarkOnINT1(void *handle, LSM330_ACC_I1_WTM_t newValue);
status_t LSM330_ACC_R_FIFO_WatermarkOnINT1(void *handle, LSM330_ACC_I1_WTM_t *value);

/*******************************************************************************
* Register      : CTRL6
* Address       : 0X25
* Bit Group Name: I1_EMPTY
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_I1_EMPTY_DISABLE 		 =0x00,
  	LSM330_ACC_I1_EMPTY_ENABLE 		 =0x08,
} LSM330_ACC_I1_EMPTY_t;

#define  	LSM330_ACC_I1_EMPTY_MASK  	0x08
status_t  LSM330_ACC_W_FIFO_EmptyOnINT1(void *handle, LSM330_ACC_I1_EMPTY_t newValue);
status_t LSM330_ACC_R_FIFO_EmptyOnINT1(void *handle, LSM330_ACC_I1_EMPTY_t *value);

/*******************************************************************************
* Register      : CTRL6
* Address       : 0X25
* Bit Group Name: ADD_INC
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_ADD_INC_DISABLE 		 =0x00,
  	LSM330_ACC_ADD_INC_ENABLE 		 =0x10,
} LSM330_ACC_ADD_INC_t;

#define  	LSM330_ACC_ADD_INC_MASK  	0x10
status_t  LSM330_ACC_W_AutoInc(void *handle, LSM330_ACC_ADD_INC_t newValue);
status_t LSM330_ACC_R_AutoInc(void *handle, LSM330_ACC_ADD_INC_t *value);

/*******************************************************************************
* Register      : CTRL6
* Address       : 0X25
* Bit Group Name: STP_WTM
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_STP_WTM_DISABLE 		 =0x00,
  	LSM330_ACC_STP_WTM_ENABLE 		 =0x20,
} LSM330_ACC_STP_WTM_t;

#define  	LSM330_ACC_STP_WTM_MASK  	0x20
status_t  LSM330_ACC_W_StopOnWatermark(void *handle, LSM330_ACC_STP_WTM_t newValue);
status_t LSM330_ACC_R_StopOnWatermark(void *handle, LSM330_ACC_STP_WTM_t *value);

/*******************************************************************************
* Register      : CTRL6
* Address       : 0X25
* Bit Group Name: FIFO_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_FIFO_EN_DISABLE 		 =0x00,
  	LSM330_ACC_FIFO_EN_ENABLE 		 =0x40,
} LSM330_ACC_FIFO_EN_t;

#define  	LSM330_ACC_FIFO_EN_MASK  	0x40
status_t  LSM330_ACC_W_FIFO_(void *handle, LSM330_ACC_FIFO_EN_t newValue);
status_t LSM330_ACC_R_FIFO_(void *handle, LSM330_ACC_FIFO_EN_t *value);

/*******************************************************************************
* Register      : CTRL6
* Address       : 0X25
* Bit Group Name: BOOT
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_BOOT_NO 		 =0x00,
  	LSM330_ACC_BOOT_YES 		 =0x80,
} LSM330_ACC_BOOT_t;

#define  	LSM330_ACC_BOOT_MASK  	0x80
status_t  LSM330_ACC_W_ForceReboot(void *handle, LSM330_ACC_BOOT_t newValue);
status_t LSM330_ACC_R_ForceReboot(void *handle, LSM330_ACC_BOOT_t *value);

/*******************************************************************************
* Register      : STATUS
* Address       : 0X27
* Bit Group Name: XDA
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_ACC_XDA_NOT_AVAILABLE 		 =0x00,
  	LSM330_ACC_XDA_AVAILABLE 		 =0x01,
} LSM330_ACC_XDA_t;

#define  	LSM330_ACC_XDA_MASK  	0x01
status_t LSM330_ACC_R_NewDataX(void *handle, LSM330_ACC_XDA_t *value);

/*******************************************************************************
* Register      : STATUS
* Address       : 0X27
* Bit Group Name: YDA
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_ACC_YDA_NOT_AVAILABLE 		 =0x00,
  	LSM330_ACC_YDA_AVAILABLE 		 =0x02,
} LSM330_ACC_YDA_t;

#define  	LSM330_ACC_YDA_MASK  	0x02
status_t LSM330_ACC_R_NewDataY(void *handle, LSM330_ACC_YDA_t *value);

/*******************************************************************************
* Register      : STATUS
* Address       : 0X27
* Bit Group Name: ZDA
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_ACC_ZDA_NOT_AVAILABLE 		 =0x00,
  	LSM330_ACC_ZDA_AVAILABLE 		 =0x04,
} LSM330_ACC_ZDA_t;

#define  	LSM330_ACC_ZDA_MASK  	0x04
status_t LSM330_ACC_R_NewDataZ(void *handle, LSM330_ACC_ZDA_t *value);

/*******************************************************************************
* Register      : STATUS
* Address       : 0X27
* Bit Group Name: ZYXDA
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_ACC_ZYXDA_NOT_AVAILABLE 		 =0x00,
  	LSM330_ACC_ZYXDA_AVAILABLE 		 =0x08,
} LSM330_ACC_ZYXDA_t;

#define  	LSM330_ACC_ZYXDA_MASK  	0x08
status_t LSM330_ACC_R_NewDataXYZ(void *handle, LSM330_ACC_ZYXDA_t *value);

/*******************************************************************************
* Register      : STATUS
* Address       : 0X27
* Bit Group Name: XOR
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_ACC_XOR_NO 		 =0x00,
  	LSM330_ACC_XOR_YES 		 =0x10,
} LSM330_ACC_XOR_t;

#define  	LSM330_ACC_XOR_MASK  	0x10
status_t LSM330_ACC_R_DataOverrunX(void *handle, LSM330_ACC_XOR_t *value);

/*******************************************************************************
* Register      : STATUS
* Address       : 0X27
* Bit Group Name: YOR
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_ACC_YOR_NO 		 =0x00,
  	LSM330_ACC_YOR_YES 		 =0x20,
} LSM330_ACC_YOR_t;

#define  	LSM330_ACC_YOR_MASK  	0x20
status_t LSM330_ACC_R_DataOverrunY(void *handle, LSM330_ACC_YOR_t *value);

/*******************************************************************************
* Register      : STATUS
* Address       : 0X27
* Bit Group Name: ZOR
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_ACC_ZOR_NO 		 =0x00,
  	LSM330_ACC_ZOR_YES 		 =0x40,
} LSM330_ACC_ZOR_t;

#define  	LSM330_ACC_ZOR_MASK  	0x40
status_t LSM330_ACC_R_DataOverrunZ(void *handle, LSM330_ACC_ZOR_t *value);

/*******************************************************************************
* Register      : STATUS
* Address       : 0X27
* Bit Group Name: ZYXOR
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_ACC_ZYXOR_NO 		 =0x00,
  	LSM330_ACC_ZYXOR_YES 		 =0x80,
} LSM330_ACC_ZYXOR_t;

#define  	LSM330_ACC_ZYXOR_MASK  	0x80
status_t LSM330_ACC_R_DataOverrunXYZ(void *handle, LSM330_ACC_ZYXOR_t *value);

/*******************************************************************************
* Register      : FIFO_CTRL
* Address       : 0X2E
* Bit Group Name: WTMP
* Permission    : RW
*******************************************************************************/
#define  	LSM330_ACC_WTMP_MASK  	0x1F
#define  	LSM330_ACC_WTMP_POSITION  	0
status_t  LSM330_ACC_W_FIFO_WatermarkPointer(void *handle, u8_t newValue);
status_t LSM330_ACC_R_FIFO_WatermarkPointer(void *handle, u8_t *value);

/*******************************************************************************
* Register      : FIFO_CTRL
* Address       : 0X2E
* Bit Group Name: FMODE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_FMODE_BYPASS 		 =0x00,
  	LSM330_ACC_FMODE_FIFO 		 =0x20,
  	LSM330_ACC_FMODE_STREAM 		 =0x40,
  	LSM330_ACC_FMODE_STREAM_TO_FIFO 		 =0x60,
  	LSM330_ACC_FMODE_BYPASS_TO_STREAM 		 =0x80,
  	LSM330_ACC_FMODE_BYPASS_TO_FIFO 		 =0xE0,
} LSM330_ACC_FMODE_t;

#define  	LSM330_ACC_FMODE_MASK  	0xE0
status_t  LSM330_ACC_W_FIFO_Mode(void *handle, LSM330_ACC_FMODE_t newValue);
status_t LSM330_ACC_R_FIFO_Mode(void *handle, LSM330_ACC_FMODE_t *value);

/*******************************************************************************
* Register      : FIFO_SRC
* Address       : 0X2F
* Bit Group Name: FSS
* Permission    : RO
*******************************************************************************/
#define  	LSM330_ACC_FSS_MASK  	0x1F
#define  	LSM330_ACC_FSS_POSITION  	0
status_t LSM330_ACC_R_FIFO_StoredData(void *handle, u8_t *value);

/*******************************************************************************
* Register      : FIFO_SRC
* Address       : 0X2F
* Bit Group Name: EMPTY
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_ACC_EMPTY_DOWN 		 =0x00,
  	LSM330_ACC_EMPTY_UP 		 =0x20,
} LSM330_ACC_EMPTY_t;

#define  	LSM330_ACC_EMPTY_MASK  	0x20
status_t LSM330_ACC_R_FIFO_EmptyFlag(void *handle, LSM330_ACC_EMPTY_t *value);

/*******************************************************************************
* Register      : FIFO_SRC
* Address       : 0X2F
* Bit Group Name: OVR
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_ACC_OVR_DOWN 		 =0x00,
  	LSM330_ACC_OVR_UP 		 =0x40,
} LSM330_ACC_OVR_t;

#define  	LSM330_ACC_OVR_MASK  	0x40
status_t LSM330_ACC_R_FIFO_OverrunFlag(void *handle, LSM330_ACC_OVR_t *value);

/*******************************************************************************
* Register      : FIFO_SRC
* Address       : 0X2F
* Bit Group Name: WTM
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_ACC_WTM_DOWN 		 =0x00,
  	LSM330_ACC_WTM_UP 		 =0x80,
} LSM330_ACC_WTM_t;

#define  	LSM330_ACC_WTM_MASK  	0x80
status_t LSM330_ACC_R_FIFO_WatermarkFlag(void *handle, LSM330_ACC_WTM_t *value);

/*******************************************************************************
* Register      : TIM4_SM1
* Address       : 0X50
* Bit Group Name: TIM4_SM1_BIT
* Permission    : RW
*******************************************************************************/
#define  	LSM330_ACC_TIM4_SM1_BIT_MASK  	0xFF
#define  	LSM330_ACC_TIM4_SM1_BIT_POSITION  	0
status_t  LSM330_ACC_W_Timer4_SM1(void *handle, u8_t newValue);
status_t LSM330_ACC_R_Timer4_SM1(void *handle, u8_t *value);

/*******************************************************************************
* Register      : TIM3_SM1
* Address       : 0X51
* Bit Group Name: TIM3_SM1_BIT
* Permission    : RW
*******************************************************************************/
#define  	LSM330_ACC_TIM3_SM1_BIT_MASK  	0xFF
#define  	LSM330_ACC_TIM3_SM1_BIT_POSITION  	0
status_t  LSM330_ACC_W_Timer3_SM1(void *handle, u8_t newValue);
status_t LSM330_ACC_R_Timer3_SM1(void *handle, u8_t *value);

/*******************************************************************************
* Register      : THRS2_SM1
* Address       : 0X56
* Bit Group Name: THRS2_SM1_BIT
* Permission    : RW
*******************************************************************************/
#define  	LSM330_ACC_THRS2_SM1_BIT_MASK  	0xFF
#define  	LSM330_ACC_THRS2_SM1_BIT_POSITION  	0
status_t  LSM330_ACC_W_Threshold2_SM1(void *handle, u8_t newValue);
status_t LSM330_ACC_R_Threshold2_SM1(void *handle, u8_t *value);

/*******************************************************************************
* Register      : THRS1_SM1
* Address       : 0X57
* Bit Group Name: THRS1_SM1_BIT
* Permission    : RW
*******************************************************************************/
#define  	LSM330_ACC_THRS1_SM1_BIT_MASK  	0xFF
#define  	LSM330_ACC_THRS1_SM1_BIT_POSITION  	0
status_t  LSM330_ACC_W_Threshold1_SM1(void *handle, u8_t newValue);
status_t LSM330_ACC_R_Threshold1_SM1(void *handle, u8_t *value);

/*******************************************************************************
* Register      : MASKB_SM1
* Address       : 0X59
* Bit Group Name: SM1_MB_N_V
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM1_MB_N_V_DISABLE 		 =0x00,
  	LSM330_ACC_SM1_MB_N_V_ENABLE 		 =0x01,
} LSM330_ACC_SM1_MB_N_V_t;

#define  	LSM330_ACC_SM1_MB_N_V_MASK  	0x01
status_t  LSM330_ACC_W_SM1_MaskB_NegativeVector(void *handle, LSM330_ACC_SM1_MB_N_V_t newValue);
status_t LSM330_ACC_R_SM1_MaskB_NegativeVector(void *handle, LSM330_ACC_SM1_MB_N_V_t *value);

/*******************************************************************************
* Register      : MASKB_SM1
* Address       : 0X59
* Bit Group Name: SM1_MB_P_V
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM1_MB_P_V_DISABLE 		 =0x00,
  	LSM330_ACC_SM1_MB_P_V_ENABLE 		 =0x02,
} LSM330_ACC_SM1_MB_P_V_t;

#define  	LSM330_ACC_SM1_MB_P_V_MASK  	0x02
status_t  LSM330_ACC_W_SM1_MaskB_PositiveVector(void *handle, LSM330_ACC_SM1_MB_P_V_t newValue);
status_t LSM330_ACC_R_SM1_MaskB_PositiveVector(void *handle, LSM330_ACC_SM1_MB_P_V_t *value);

/*******************************************************************************
* Register      : MASKB_SM1
* Address       : 0X59
* Bit Group Name: SM1_MB_N_Z
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM1_MB_N_Z_DISABLE 		 =0x00,
  	LSM330_ACC_SM1_MB_N_Z_ENABLE 		 =0x04,
} LSM330_ACC_SM1_MB_N_Z_t;

#define  	LSM330_ACC_SM1_MB_N_Z_MASK  	0x04
status_t  LSM330_ACC_W_SM1_MaskB_NegativeZ(void *handle, LSM330_ACC_SM1_MB_N_Z_t newValue);
status_t LSM330_ACC_R_SM1_MaskB_NegativeZ(void *handle, LSM330_ACC_SM1_MB_N_Z_t *value);

/*******************************************************************************
* Register      : MASKB_SM1
* Address       : 0X59
* Bit Group Name: SM1_MB_P_Z
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM1_MB_P_Z_DISABLE 		 =0x00,
  	LSM330_ACC_SM1_MB_P_Z_ENABLE 		 =0x08,
} LSM330_ACC_SM1_MB_P_Z_t;

#define  	LSM330_ACC_SM1_MB_P_Z_MASK  	0x08
status_t  LSM330_ACC_W_SM1_MaskB_PositiveZ(void *handle, LSM330_ACC_SM1_MB_P_Z_t newValue);
status_t LSM330_ACC_R_SM1_MaskB_PositiveZ(void *handle, LSM330_ACC_SM1_MB_P_Z_t *value);

/*******************************************************************************
* Register      : MASKB_SM1
* Address       : 0X59
* Bit Group Name: SM1_MB_N_Y
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM1_MB_N_Y_DISABLE 		 =0x00,
  	LSM330_ACC_SM1_MB_N_Y_ENABLE 		 =0x10,
} LSM330_ACC_SM1_MB_N_Y_t;

#define  	LSM330_ACC_SM1_MB_N_Y_MASK  	0x10
status_t  LSM330_ACC_W_SM1_MaskB_NegativeY(void *handle, LSM330_ACC_SM1_MB_N_Y_t newValue);
status_t LSM330_ACC_R_SM1_MaskB_NegativeY(void *handle, LSM330_ACC_SM1_MB_N_Y_t *value);

/*******************************************************************************
* Register      : MASKB_SM1
* Address       : 0X59
* Bit Group Name: SM1_MB_P_Y
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM1_MB_P_Y_DISABLE 		 =0x00,
  	LSM330_ACC_SM1_MB_P_Y_ENABLE 		 =0x20,
} LSM330_ACC_SM1_MB_P_Y_t;

#define  	LSM330_ACC_SM1_MB_P_Y_MASK  	0x20
status_t  LSM330_ACC_W_SM1_MaskB_PositiveY(void *handle, LSM330_ACC_SM1_MB_P_Y_t newValue);
status_t LSM330_ACC_R_SM1_MaskB_PositiveY(void *handle, LSM330_ACC_SM1_MB_P_Y_t *value);

/*******************************************************************************
* Register      : MASKB_SM1
* Address       : 0X59
* Bit Group Name: SM1_MB_N_X
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM1_MB_N_X_DISABLE 		 =0x00,
  	LSM330_ACC_SM1_MB_N_X_ENABLE 		 =0x40,
} LSM330_ACC_SM1_MB_N_X_t;

#define  	LSM330_ACC_SM1_MB_N_X_MASK  	0x40
status_t  LSM330_ACC_W_SM1_MaskB_NegativeX(void *handle, LSM330_ACC_SM1_MB_N_X_t newValue);
status_t LSM330_ACC_R_SM1_MaskB_NegativeX(void *handle, LSM330_ACC_SM1_MB_N_X_t *value);

/*******************************************************************************
* Register      : MASKB_SM1
* Address       : 0X59
* Bit Group Name: SM1_MB_P_X
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM1_MB_P_X_DISABLE 		 =0x00,
  	LSM330_ACC_SM1_MB_P_X_ENABLE 		 =0x80,
} LSM330_ACC_SM1_MB_P_X_t;

#define  	LSM330_ACC_SM1_MB_P_X_MASK  	0x80
status_t  LSM330_ACC_W_SM1_MaskB_PositiveX(void *handle, LSM330_ACC_SM1_MB_P_X_t newValue);
status_t LSM330_ACC_R_SM1_MaskB_PositiveX(void *handle, LSM330_ACC_SM1_MB_P_X_t *value);

/*******************************************************************************
* Register      : MASKA_SM1
* Address       : 0X5A
* Bit Group Name: SM1_MA_N_V
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM1_MA_N_V_DISABLE 		 =0x00,
  	LSM330_ACC_SM1_MA_N_V_ENABLE 		 =0x01,
} LSM330_ACC_SM1_MA_N_V_t;

#define  	LSM330_ACC_SM1_MA_N_V_MASK  	0x01
status_t  LSM330_ACC_W_SM1_MaskA_NegativeVector(void *handle, LSM330_ACC_SM1_MA_N_V_t newValue);
status_t LSM330_ACC_R_SM1_MaskA_NegativeVector(void *handle, LSM330_ACC_SM1_MA_N_V_t *value);

/*******************************************************************************
* Register      : MASKA_SM1
* Address       : 0X5A
* Bit Group Name: SM1_MA_P_V
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM1_MA_P_V_DISABLE 		 =0x00,
  	LSM330_ACC_SM1_MA_P_V_ENABLE 		 =0x02,
} LSM330_ACC_SM1_MA_P_V_t;

#define  	LSM330_ACC_SM1_MA_P_V_MASK  	0x02
status_t  LSM330_ACC_W_SM1_MaskA_PositiveVector(void *handle, LSM330_ACC_SM1_MA_P_V_t newValue);
status_t LSM330_ACC_R_SM1_MaskA_PositiveVector(void *handle, LSM330_ACC_SM1_MA_P_V_t *value);

/*******************************************************************************
* Register      : MASKA_SM1
* Address       : 0X5A
* Bit Group Name: SM1_MA_N_Z
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM1_MA_N_Z_DISABLE 		 =0x00,
  	LSM330_ACC_SM1_MA_N_Z_ENABLE 		 =0x04,
} LSM330_ACC_SM1_MA_N_Z_t;

#define  	LSM330_ACC_SM1_MA_N_Z_MASK  	0x04
status_t  LSM330_ACC_W_SM1_MaskA_NegativeZ(void *handle, LSM330_ACC_SM1_MA_N_Z_t newValue);
status_t LSM330_ACC_R_SM1_MaskA_NegativeZ(void *handle, LSM330_ACC_SM1_MA_N_Z_t *value);

/*******************************************************************************
* Register      : MASKA_SM1
* Address       : 0X5A
* Bit Group Name: SM1_MA_P_Z
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM1_MA_P_Z_DISABLE 		 =0x00,
  	LSM330_ACC_SM1_MA_P_Z_ENABLE 		 =0x08,
} LSM330_ACC_SM1_MA_P_Z_t;

#define  	LSM330_ACC_SM1_MA_P_Z_MASK  	0x08
status_t  LSM330_ACC_W_SM1_MaskA_PositiveZ(void *handle, LSM330_ACC_SM1_MA_P_Z_t newValue);
status_t LSM330_ACC_R_SM1_MaskA_PositiveZ(void *handle, LSM330_ACC_SM1_MA_P_Z_t *value);

/*******************************************************************************
* Register      : MASKA_SM1
* Address       : 0X5A
* Bit Group Name: SM1_MA_N_Y
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM1_MA_N_Y_DISABLE 		 =0x00,
  	LSM330_ACC_SM1_MA_N_Y_ENABLE 		 =0x10,
} LSM330_ACC_SM1_MA_N_Y_t;

#define  	LSM330_ACC_SM1_MA_N_Y_MASK  	0x10
status_t  LSM330_ACC_W_SM1_MaskA_NegativeY(void *handle, LSM330_ACC_SM1_MA_N_Y_t newValue);
status_t LSM330_ACC_R_SM1_MaskA_NegativeY(void *handle, LSM330_ACC_SM1_MA_N_Y_t *value);

/*******************************************************************************
* Register      : MASKA_SM1
* Address       : 0X5A
* Bit Group Name: SM1_MA_P_Y
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM1_MA_P_Y_DISABLE 		 =0x00,
  	LSM330_ACC_SM1_MA_P_Y_ENABLE 		 =0x20,
} LSM330_ACC_SM1_MA_P_Y_t;

#define  	LSM330_ACC_SM1_MA_P_Y_MASK  	0x20
status_t  LSM330_ACC_W_SM1_MaskA_PositiveY(void *handle, LSM330_ACC_SM1_MA_P_Y_t newValue);
status_t LSM330_ACC_R_SM1_MaskA_PositiveY(void *handle, LSM330_ACC_SM1_MA_P_Y_t *value);

/*******************************************************************************
* Register      : MASKA_SM1
* Address       : 0X5A
* Bit Group Name: SM1_MA_N_X
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM1_MA_N_X_DISABLE 		 =0x00,
  	LSM330_ACC_SM1_MA_N_X_ENABLE 		 =0x40,
} LSM330_ACC_SM1_MA_N_X_t;

#define  	LSM330_ACC_SM1_MA_N_X_MASK  	0x40
status_t  LSM330_ACC_W_SM1_MaskA_NegativeX(void *handle, LSM330_ACC_SM1_MA_N_X_t newValue);
status_t LSM330_ACC_R_SM1_MaskA_NegativeX(void *handle, LSM330_ACC_SM1_MA_N_X_t *value);

/*******************************************************************************
* Register      : MASKA_SM1
* Address       : 0X5A
* Bit Group Name: SM1_MA_P_X
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM1_MA_P_X_DISABLE 		 =0x00,
  	LSM330_ACC_SM1_MA_P_X_ENABLE 		 =0x80,
} LSM330_ACC_SM1_MA_P_X_t;

#define  	LSM330_ACC_SM1_MA_P_X_MASK  	0x80
status_t  LSM330_ACC_W_SM1_MaskA_PositiveX(void *handle, LSM330_ACC_SM1_MA_P_X_t newValue);
status_t LSM330_ACC_R_SM1_MaskA_PositiveX(void *handle, LSM330_ACC_SM1_MA_P_X_t *value);

/*******************************************************************************
* Register      : SETT_SM1
* Address       : 0X5B
* Bit Group Name: SM1_SITR
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM1_SITR_DISABLE 		 =0x00,
  	LSM330_ACC_SM1_SITR_ENABLE 		 =0x01,
} LSM330_ACC_SM1_SITR_t;

#define  	LSM330_ACC_SM1_SITR_MASK  	0x01
status_t  LSM330_ACC_W_SM1_ProgramFlowModification(void *handle, LSM330_ACC_SM1_SITR_t newValue);
status_t LSM330_ACC_R_SM1_ProgramFlowModification(void *handle, LSM330_ACC_SM1_SITR_t *value);

/*******************************************************************************
* Register      : SETT_SM1
* Address       : 0X5B
* Bit Group Name: SM1_R_TAM
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM1_R_TAM_DISABLE 		 =0x00,
  	LSM330_ACC_SM1_R_TAM_ENABLE 		 =0x02,
} LSM330_ACC_SM1_R_TAM_t;

#define  	LSM330_ACC_SM1_R_TAM_MASK  	0x02
status_t  LSM330_ACC_W_SM1_NextConditionValidation(void *handle, LSM330_ACC_SM1_R_TAM_t newValue);
status_t LSM330_ACC_R_SM1_NextConditionValidation(void *handle, LSM330_ACC_SM1_R_TAM_t *value);

/*******************************************************************************
* Register      : SETT_SM1
* Address       : 0X5B
* Bit Group Name: SM1_THR3_MA
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM1_THR3_MA_DISABLE 		 =0x00,
  	LSM330_ACC_SM1_THR3_MA_ENABLE 		 =0x04,
} LSM330_ACC_SM1_THR3_MA_t;

#define  	LSM330_ACC_SM1_THR3_MA_MASK  	0x04
status_t  LSM330_ACC_W_SM1_Threshold3_OnMaskA(void *handle, LSM330_ACC_SM1_THR3_MA_t newValue);
status_t LSM330_ACC_R_SM1_Threshold3_OnMaskA(void *handle, LSM330_ACC_SM1_THR3_MA_t *value);

/*******************************************************************************
* Register      : SETT_SM1
* Address       : 0X5B
* Bit Group Name: SM1_ABS
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM1_ABS_DISABLE 		 =0x00,
  	LSM330_ACC_SM1_ABS_ENABLE 		 =0x20,
} LSM330_ACC_SM1_ABS_t;

#define  	LSM330_ACC_SM1_ABS_MASK  	0x20
status_t  LSM330_ACC_W_SM1_AbsoluteValue(void *handle, LSM330_ACC_SM1_ABS_t newValue);
status_t LSM330_ACC_R_SM1_AbsoluteValue(void *handle, LSM330_ACC_SM1_ABS_t *value);

/*******************************************************************************
* Register      : SETT_SM1
* Address       : 0X5B
* Bit Group Name: SM1_THR3_SA
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM1_THR3_SA_DISABLE 		 =0x00,
  	LSM330_ACC_SM1_THR3_SA_ENABLE 		 =0x40,
} LSM330_ACC_SM1_THR3_SA_t;

#define  	LSM330_ACC_SM1_THR3_SA_MASK  	0x40
status_t  LSM330_ACC_W_SM1_Threshold3_OnMaskB(void *handle, LSM330_ACC_SM1_THR3_SA_t newValue);
status_t LSM330_ACC_R_SM1_Threshold3_OnMaskB(void *handle, LSM330_ACC_SM1_THR3_SA_t *value);

/*******************************************************************************
* Register      : SETT_SM1
* Address       : 0X5B
* Bit Group Name: SM1_P_DET
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM1_P_DET_DISABLE 		 =0x00,
  	LSM330_ACC_SM1_P_DET_ENABLE 		 =0x80,
} LSM330_ACC_SM1_P_DET_t;

#define  	LSM330_ACC_SM1_P_DET_MASK  	0x80
status_t  LSM330_ACC_W_SM1_PeakDetection(void *handle, LSM330_ACC_SM1_P_DET_t newValue);
status_t LSM330_ACC_R_SM1_PeakDetection(void *handle, LSM330_ACC_SM1_P_DET_t *value);

/*******************************************************************************
* Register      : PR_SM1
* Address       : 0X5C
* Bit Group Name: SM1_RP
* Permission    : RW
*******************************************************************************/
#define  	LSM330_ACC_SM1_RP_MASK  	0x0F
#define  	LSM330_ACC_SM1_RP_POSITION  	0
status_t  LSM330_ACC_W_SM1_ResetPointer(void *handle, u8_t newValue);
status_t LSM330_ACC_R_SM1_ResetPointer(void *handle, u8_t *value);

/*******************************************************************************
* Register      : PR_SM1
* Address       : 0X5C
* Bit Group Name: SM1_PP
* Permission    : RW
*******************************************************************************/
#define  	LSM330_ACC_SM1_PP_MASK  	0xF0
#define  	LSM330_ACC_SM1_PP_POSITION  	4
status_t  LSM330_ACC_W_SM1_ProgramPointer(void *handle, u8_t newValue);
status_t LSM330_ACC_R_SM1_ProgramPointer(void *handle, u8_t *value);

/*******************************************************************************
* Register      : OUTS_SM1
* Address       : 0X5F
* Bit Group Name: SM1_FLAG_N_V
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM1_FLAG_N_V_DOWN 		 =0x00,
  	LSM330_ACC_SM1_FLAG_N_V_UP 		 =0x01,
} LSM330_ACC_SM1_FLAG_N_V_t;

#define  	LSM330_ACC_SM1_FLAG_N_V_MASK  	0x01
status_t LSM330_ACC_R_SM1_NegativeVectorFlag(void *handle, LSM330_ACC_SM1_FLAG_N_V_t *value);

/*******************************************************************************
* Register      : OUTS_SM1
* Address       : 0X5F
* Bit Group Name: SM1_FLAG_P_V
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM1_FLAG_P_V_DOWN 		 =0x00,
  	LSM330_ACC_SM1_FLAG_P_V_UP 		 =0x02,
} LSM330_ACC_SM1_FLAG_P_V_t;

#define  	LSM330_ACC_SM1_FLAG_P_V_MASK  	0x02
status_t LSM330_ACC_R_SM1_PositiveVectorFlag(void *handle, LSM330_ACC_SM1_FLAG_P_V_t *value);

/*******************************************************************************
* Register      : OUTS_SM1
* Address       : 0X5F
* Bit Group Name: SM1_FLAG_N_Z
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM1_FLAG_N_Z_DOWN 		 =0x00,
  	LSM330_ACC_SM1_FLAG_N_Z_UP 		 =0x04,
} LSM330_ACC_SM1_FLAG_N_Z_t;

#define  	LSM330_ACC_SM1_FLAG_N_Z_MASK  	0x04
status_t LSM330_ACC_R_SM1_NegativeZFlag(void *handle, LSM330_ACC_SM1_FLAG_N_Z_t *value);

/*******************************************************************************
* Register      : OUTS_SM1
* Address       : 0X5F
* Bit Group Name: SM1_FLAG_P_Z
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM1_FLAG_P_Z_DOWN 		 =0x00,
  	LSM330_ACC_SM1_FLAG_P_Z_UP 		 =0x08,
} LSM330_ACC_SM1_FLAG_P_Z_t;

#define  	LSM330_ACC_SM1_FLAG_P_Z_MASK  	0x08
status_t LSM330_ACC_R_SM1_PositiveZFlag(void *handle, LSM330_ACC_SM1_FLAG_P_Z_t *value);

/*******************************************************************************
* Register      : OUTS_SM1
* Address       : 0X5F
* Bit Group Name: SM1_FLAG_N_Y
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM1_FLAG_N_Y_DOWN 		 =0x00,
  	LSM330_ACC_SM1_FLAG_N_Y_UP 		 =0x10,
} LSM330_ACC_SM1_FLAG_N_Y_t;

#define  	LSM330_ACC_SM1_FLAG_N_Y_MASK  	0x10
status_t LSM330_ACC_R_SM1_NegativeYFlag(void *handle, LSM330_ACC_SM1_FLAG_N_Y_t *value);

/*******************************************************************************
* Register      : OUTS_SM1
* Address       : 0X5F
* Bit Group Name: SM1_FLAG_P_Y
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM1_FLAG_P_Y_DOWN 		 =0x00,
  	LSM330_ACC_SM1_FLAG_P_Y_UP 		 =0x20,
} LSM330_ACC_SM1_FLAG_P_Y_t;

#define  	LSM330_ACC_SM1_FLAG_P_Y_MASK  	0x20
status_t LSM330_ACC_R_SM1_PositiveYFlag(void *handle, LSM330_ACC_SM1_FLAG_P_Y_t *value);

/*******************************************************************************
* Register      : OUTS_SM1
* Address       : 0X5F
* Bit Group Name: SM1_FLAG_N_X
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM1_FLAG_N_X_DOWN 		 =0x00,
  	LSM330_ACC_SM1_FLAG_N_X_UP 		 =0x40,
} LSM330_ACC_SM1_FLAG_N_X_t;

#define  	LSM330_ACC_SM1_FLAG_N_X_MASK  	0x40
status_t LSM330_ACC_R_SM1_NegativeXFlag(void *handle, LSM330_ACC_SM1_FLAG_N_X_t *value);

/*******************************************************************************
* Register      : OUTS_SM1
* Address       : 0X5F
* Bit Group Name: SM1_FLAG_P_X
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM1_FLAG_P_X_DOWN 		 =0x00,
  	LSM330_ACC_SM1_FLAG_P_X_UP 		 =0x80,
} LSM330_ACC_SM1_FLAG_P_X_t;

#define  	LSM330_ACC_SM1_FLAG_P_X_MASK  	0x80
status_t LSM330_ACC_R_SM1_PositiveXFlag(void *handle, LSM330_ACC_SM1_FLAG_P_X_t *value);

/*******************************************************************************
* Register      : TIM4_SM2
* Address       : 0X70
* Bit Group Name: TIM4_SM2_BIT
* Permission    : RW
*******************************************************************************/
#define  	LSM330_ACC_TIM4_SM2_BIT_MASK  	0xFF
#define  	LSM330_ACC_TIM4_SM2_BIT_POSITION  	0
status_t  LSM330_ACC_W_Timer4_SM2(void *handle, u8_t newValue);
status_t LSM330_ACC_R_Timer4_SM2(void *handle, u8_t *value);

/*******************************************************************************
* Register      : TIM3_SM2
* Address       : 0X71
* Bit Group Name: TIM3_SM2_BIT
* Permission    : RW
*******************************************************************************/
#define  	LSM330_ACC_TIM3_SM2_BIT_MASK  	0xFF
#define  	LSM330_ACC_TIM3_SM2_BIT_POSITION  	0
status_t  LSM330_ACC_W_Timer3_SM2(void *handle, u8_t newValue);
status_t LSM330_ACC_R_Timer3_SM2(void *handle, u8_t *value);

/*******************************************************************************
* Register      : THRS2_SM2
* Address       : 0X76
* Bit Group Name: THRS2_SM2_BIT
* Permission    : RW
*******************************************************************************/
#define  	LSM330_ACC_THRS2_SM2_BIT_MASK  	0xFF
#define  	LSM330_ACC_THRS2_SM2_BIT_POSITION  	0
status_t  LSM330_ACC_W_Threshold2_SM2(void *handle, u8_t newValue);
status_t LSM330_ACC_R_Threshold2_SM2(void *handle, u8_t *value);

/*******************************************************************************
* Register      : THRS1_SM2
* Address       : 0X77
* Bit Group Name: THRS1_SM2_BIT
* Permission    : RW
*******************************************************************************/
#define  	LSM330_ACC_THRS1_SM2_BIT_MASK  	0xFF
#define  	LSM330_ACC_THRS1_SM2_BIT_POSITION  	0
status_t  LSM330_ACC_W_Threshold1_SM2(void *handle, u8_t newValue);
status_t LSM330_ACC_R_Threshold1_SM2(void *handle, u8_t *value);

/*******************************************************************************
* Register      : DES_SM2
* Address       : 0X78
* Bit Group Name: DES_SM2_BIT
* Permission    : RW
*******************************************************************************/
#define  	LSM330_ACC_DES_SM2_BIT_MASK  	0xFF
#define  	LSM330_ACC_DES_SM2_BIT_POSITION  	0
status_t  LSM330_ACC_W_DecimationCounterSM2(void *handle, u8_t newValue);
status_t LSM330_ACC_R_DecimationCounterSM2(void *handle, u8_t *value);

/*******************************************************************************
* Register      : MASKB_SM2
* Address       : 0X79
* Bit Group Name: SM2_MB_N_V
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM2_MB_N_V_DISABLE 		 =0x00,
  	LSM330_ACC_SM2_MB_N_V_ENABLE 		 =0x01,
} LSM330_ACC_SM2_MB_N_V_t;

#define  	LSM330_ACC_SM2_MB_N_V_MASK  	0x01
status_t  LSM330_ACC_W_SM2_MaskB_NegativeVector(void *handle, LSM330_ACC_SM2_MB_N_V_t newValue);
status_t LSM330_ACC_R_SM2_MaskB_NegativeVector(void *handle, LSM330_ACC_SM2_MB_N_V_t *value);

/*******************************************************************************
* Register      : MASKB_SM2
* Address       : 0X79
* Bit Group Name: SM2_MB_P_V
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM2_MB_P_V_DISABLE 		 =0x00,
  	LSM330_ACC_SM2_MB_P_V_ENABLE 		 =0x02,
} LSM330_ACC_SM2_MB_P_V_t;

#define  	LSM330_ACC_SM2_MB_P_V_MASK  	0x02
status_t  LSM330_ACC_W_SM2_MaskB_PositiveVector(void *handle, LSM330_ACC_SM2_MB_P_V_t newValue);
status_t LSM330_ACC_R_SM2_MaskB_PositiveVector(void *handle, LSM330_ACC_SM2_MB_P_V_t *value);

/*******************************************************************************
* Register      : MASKB_SM2
* Address       : 0X79
* Bit Group Name: SM2_MB_N_Z
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM2_MB_N_Z_DISABLE 		 =0x00,
  	LSM330_ACC_SM2_MB_N_Z_ENABLE 		 =0x04,
} LSM330_ACC_SM2_MB_N_Z_t;

#define  	LSM330_ACC_SM2_MB_N_Z_MASK  	0x04
status_t  LSM330_ACC_W_SM2_MaskB_NegativeZ(void *handle, LSM330_ACC_SM2_MB_N_Z_t newValue);
status_t LSM330_ACC_R_SM2_MaskB_NegativeZ(void *handle, LSM330_ACC_SM2_MB_N_Z_t *value);

/*******************************************************************************
* Register      : MASKB_SM2
* Address       : 0X79
* Bit Group Name: SM2_MB_P_Z
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM2_MB_P_Z_DISABLE 		 =0x00,
  	LSM330_ACC_SM2_MB_P_Z_ENABLE 		 =0x08,
} LSM330_ACC_SM2_MB_P_Z_t;

#define  	LSM330_ACC_SM2_MB_P_Z_MASK  	0x08
status_t  LSM330_ACC_W_SM2_MaskB_PositiveZ(void *handle, LSM330_ACC_SM2_MB_P_Z_t newValue);
status_t LSM330_ACC_R_SM2_MaskB_PositiveZ(void *handle, LSM330_ACC_SM2_MB_P_Z_t *value);

/*******************************************************************************
* Register      : MASKB_SM2
* Address       : 0X79
* Bit Group Name: SM2_MB_N_Y
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM2_MB_N_Y_DISABLE 		 =0x00,
  	LSM330_ACC_SM2_MB_N_Y_ENABLE 		 =0x10,
} LSM330_ACC_SM2_MB_N_Y_t;

#define  	LSM330_ACC_SM2_MB_N_Y_MASK  	0x10
status_t  LSM330_ACC_W_SM2_MaskB_NegativeY(void *handle, LSM330_ACC_SM2_MB_N_Y_t newValue);
status_t LSM330_ACC_R_SM2_MaskB_NegativeY(void *handle, LSM330_ACC_SM2_MB_N_Y_t *value);

/*******************************************************************************
* Register      : MASKB_SM2
* Address       : 0X79
* Bit Group Name: SM2_MB_P_Y
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM2_MB_P_Y_DISABLE 		 =0x00,
  	LSM330_ACC_SM2_MB_P_Y_ENABLE 		 =0x20,
} LSM330_ACC_SM2_MB_P_Y_t;

#define  	LSM330_ACC_SM2_MB_P_Y_MASK  	0x20
status_t  LSM330_ACC_W_SM2_MaskB_PositiveY(void *handle, LSM330_ACC_SM2_MB_P_Y_t newValue);
status_t LSM330_ACC_R_SM2_MaskB_PositiveY(void *handle, LSM330_ACC_SM2_MB_P_Y_t *value);

/*******************************************************************************
* Register      : MASKB_SM2
* Address       : 0X79
* Bit Group Name: SM2_MB_N_X
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM2_MB_N_X_DISABLE 		 =0x00,
  	LSM330_ACC_SM2_MB_N_X_ENABLE 		 =0x40,
} LSM330_ACC_SM2_MB_N_X_t;

#define  	LSM330_ACC_SM2_MB_N_X_MASK  	0x40
status_t  LSM330_ACC_W_SM2_MaskB_NegativeX(void *handle, LSM330_ACC_SM2_MB_N_X_t newValue);
status_t LSM330_ACC_R_SM2_MaskB_NegativeX(void *handle, LSM330_ACC_SM2_MB_N_X_t *value);

/*******************************************************************************
* Register      : MASKB_SM2
* Address       : 0X79
* Bit Group Name: SM2_MB_P_X
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM2_MB_P_X_DISABLE 		 =0x00,
  	LSM330_ACC_SM2_MB_P_X_ENABLE 		 =0x80,
} LSM330_ACC_SM2_MB_P_X_t;

#define  	LSM330_ACC_SM2_MB_P_X_MASK  	0x80
status_t  LSM330_ACC_W_SM2_MaskB_PositiveX(void *handle, LSM330_ACC_SM2_MB_P_X_t newValue);
status_t LSM330_ACC_R_SM2_MaskB_PositiveX(void *handle, LSM330_ACC_SM2_MB_P_X_t *value);

/*******************************************************************************
* Register      : MASKA_SM2
* Address       : 0X7A
* Bit Group Name: SM2_MA_N_V
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM2_MA_N_V_DISABLE 		 =0x00,
  	LSM330_ACC_SM2_MA_N_V_ENABLE 		 =0x01,
} LSM330_ACC_SM2_MA_N_V_t;

#define  	LSM330_ACC_SM2_MA_N_V_MASK  	0x01
status_t  LSM330_ACC_W_SM2_MaskA_NegativeVector(void *handle, LSM330_ACC_SM2_MA_N_V_t newValue);
status_t LSM330_ACC_R_SM2_MaskA_NegativeVector(void *handle, LSM330_ACC_SM2_MA_N_V_t *value);

/*******************************************************************************
* Register      : MASKA_SM2
* Address       : 0X7A
* Bit Group Name: SM2_MA_P_V
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM2_MA_P_V_DISABLE 		 =0x00,
  	LSM330_ACC_SM2_MA_P_V_ENABLE 		 =0x02,
} LSM330_ACC_SM2_MA_P_V_t;

#define  	LSM330_ACC_SM2_MA_P_V_MASK  	0x02
status_t  LSM330_ACC_W_SM2_MaskA_PositiveVector(void *handle, LSM330_ACC_SM2_MA_P_V_t newValue);
status_t LSM330_ACC_R_SM2_MaskA_PositiveVector(void *handle, LSM330_ACC_SM2_MA_P_V_t *value);

/*******************************************************************************
* Register      : MASKA_SM2
* Address       : 0X7A
* Bit Group Name: SM2_MA_N_Z
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM2_MA_N_Z_DISABLE 		 =0x00,
  	LSM330_ACC_SM2_MA_N_Z_ENABLE 		 =0x04,
} LSM330_ACC_SM2_MA_N_Z_t;

#define  	LSM330_ACC_SM2_MA_N_Z_MASK  	0x04
status_t  LSM330_ACC_W_SM2_MaskA_NegativeZ(void *handle, LSM330_ACC_SM2_MA_N_Z_t newValue);
status_t LSM330_ACC_R_SM2_MaskA_NegativeZ(void *handle, LSM330_ACC_SM2_MA_N_Z_t *value);

/*******************************************************************************
* Register      : MASKA_SM2
* Address       : 0X7A
* Bit Group Name: SM2_MA_P_Z
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM2_MA_P_Z_DISABLE 		 =0x00,
  	LSM330_ACC_SM2_MA_P_Z_ENABLE 		 =0x08,
} LSM330_ACC_SM2_MA_P_Z_t;

#define  	LSM330_ACC_SM2_MA_P_Z_MASK  	0x08
status_t  LSM330_ACC_W_SM2_MaskA_PositiveZ(void *handle, LSM330_ACC_SM2_MA_P_Z_t newValue);
status_t LSM330_ACC_R_SM2_MaskA_PositiveZ(void *handle, LSM330_ACC_SM2_MA_P_Z_t *value);

/*******************************************************************************
* Register      : MASKA_SM2
* Address       : 0X7A
* Bit Group Name: SM2_MA_N_Y
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM2_MA_N_Y_DISABLE 		 =0x00,
  	LSM330_ACC_SM2_MA_N_Y_ENABLE 		 =0x10,
} LSM330_ACC_SM2_MA_N_Y_t;

#define  	LSM330_ACC_SM2_MA_N_Y_MASK  	0x10
status_t  LSM330_ACC_W_SM2_MaskA_NegativeY(void *handle, LSM330_ACC_SM2_MA_N_Y_t newValue);
status_t LSM330_ACC_R_SM2_MaskA_NegativeY(void *handle, LSM330_ACC_SM2_MA_N_Y_t *value);

/*******************************************************************************
* Register      : MASKA_SM2
* Address       : 0X7A
* Bit Group Name: SM2_MA_P_Y
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM2_MA_P_Y_DISABLE 		 =0x00,
  	LSM330_ACC_SM2_MA_P_Y_ENABLE 		 =0x20,
} LSM330_ACC_SM2_MA_P_Y_t;

#define  	LSM330_ACC_SM2_MA_P_Y_MASK  	0x20
status_t  LSM330_ACC_W_SM2_MaskA_PositiveY(void *handle, LSM330_ACC_SM2_MA_P_Y_t newValue);
status_t LSM330_ACC_R_SM2_MaskA_PositiveY(void *handle, LSM330_ACC_SM2_MA_P_Y_t *value);

/*******************************************************************************
* Register      : MASKA_SM2
* Address       : 0X7A
* Bit Group Name: SM2_MA_N_X
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM2_MA_N_X_DISABLE 		 =0x00,
  	LSM330_ACC_SM2_MA_N_X_ENABLE 		 =0x40,
} LSM330_ACC_SM2_MA_N_X_t;

#define  	LSM330_ACC_SM2_MA_N_X_MASK  	0x40
status_t  LSM330_ACC_W_SM2_MaskA_NegativeX(void *handle, LSM330_ACC_SM2_MA_N_X_t newValue);
status_t LSM330_ACC_R_SM2_MaskA_NegativeX(void *handle, LSM330_ACC_SM2_MA_N_X_t *value);

/*******************************************************************************
* Register      : MASKA_SM2
* Address       : 0X7A
* Bit Group Name: SM2_MA_P_X
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM2_MA_P_X_DISABLE 		 =0x00,
  	LSM330_ACC_SM2_MA_P_X_ENABLE 		 =0x80,
} LSM330_ACC_SM2_MA_P_X_t;

#define  	LSM330_ACC_SM2_MA_P_X_MASK  	0x80
status_t  LSM330_ACC_W_SM2_MaskA_PositiveX(void *handle, LSM330_ACC_SM2_MA_P_X_t newValue);
status_t LSM330_ACC_R_SM2_MaskA_PositiveX(void *handle, LSM330_ACC_SM2_MA_P_X_t *value);

/*******************************************************************************
* Register      : SETT_SM2
* Address       : 0X7B
* Bit Group Name: SM2_SITR
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM2_SITR_DISABLE 		 =0x00,
  	LSM330_ACC_SM2_SITR_ENABLE 		 =0x01,
} LSM330_ACC_SM2_SITR_t;

#define  	LSM330_ACC_SM2_SITR_MASK  	0x01
status_t  LSM330_ACC_W_SM2_ProgramFlowModification(void *handle, LSM330_ACC_SM2_SITR_t newValue);
status_t LSM330_ACC_R_SM2_ProgramFlowModification(void *handle, LSM330_ACC_SM2_SITR_t *value);

/*******************************************************************************
* Register      : SETT_SM2
* Address       : 0X7B
* Bit Group Name: SM2_R_TAM
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM2_R_TAM_DISABLE 		 =0x00,
  	LSM330_ACC_SM2_R_TAM_ENABLE 		 =0x02,
} LSM330_ACC_SM2_R_TAM_t;

#define  	LSM330_ACC_SM2_R_TAM_MASK  	0x02
status_t  LSM330_ACC_W_SM2_NextConditionValidation(void *handle, LSM330_ACC_SM2_R_TAM_t newValue);
status_t LSM330_ACC_R_SM2_NextConditionValidation(void *handle, LSM330_ACC_SM2_R_TAM_t *value);

/*******************************************************************************
* Register      : SETT_SM2
* Address       : 0X7B
* Bit Group Name: SM2_THR3_MA
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM2_THR3_MA_DISABLE 		 =0x00,
  	LSM330_ACC_SM2_THR3_MA_ENABLE 		 =0x04,
} LSM330_ACC_SM2_THR3_MA_t;

#define  	LSM330_ACC_SM2_THR3_MA_MASK  	0x04
status_t  LSM330_ACC_W_SM2_Threshold3_OnMaskA(void *handle, LSM330_ACC_SM2_THR3_MA_t newValue);
status_t LSM330_ACC_R_SM2_Threshold3_OnMaskA(void *handle, LSM330_ACC_SM2_THR3_MA_t *value);

/*******************************************************************************
* Register      : SETT_SM2
* Address       : 0X7B
* Bit Group Name: SM2_ABS
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM2_ABS_DISABLE 		 =0x00,
  	LSM330_ACC_SM2_ABS_ENABLE 		 =0x20,
} LSM330_ACC_SM2_ABS_t;

#define  	LSM330_ACC_SM2_ABS_MASK  	0x20
status_t  LSM330_ACC_W_SM2_AbsoluteValue(void *handle, LSM330_ACC_SM2_ABS_t newValue);
status_t LSM330_ACC_R_SM2_AbsoluteValue(void *handle, LSM330_ACC_SM2_ABS_t *value);

/*******************************************************************************
* Register      : SETT_SM2
* Address       : 0X7B
* Bit Group Name: SM2_THR3_SA
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM2_THR3_SA_DISABLE 		 =0x00,
  	LSM330_ACC_SM2_THR3_SA_ENABLE 		 =0x40,
} LSM330_ACC_SM2_THR3_SA_t;

#define  	LSM330_ACC_SM2_THR3_SA_MASK  	0x40
status_t  LSM330_ACC_W_SM2_Threshold3_OnMaskB(void *handle, LSM330_ACC_SM2_THR3_SA_t newValue);
status_t LSM330_ACC_R_SM2_Threshold3_OnMaskB(void *handle, LSM330_ACC_SM2_THR3_SA_t *value);

/*******************************************************************************
* Register      : SETT_SM2
* Address       : 0X7B
* Bit Group Name: SM2_P_DET
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM2_P_DET_DISABLE 		 =0x00,
  	LSM330_ACC_SM2_P_DET_ENABLE 		 =0x80,
} LSM330_ACC_SM2_P_DET_t;

#define  	LSM330_ACC_SM2_P_DET_MASK  	0x80
status_t  LSM330_ACC_W_SM2_PeakDetection(void *handle, LSM330_ACC_SM2_P_DET_t newValue);
status_t LSM330_ACC_R_SM2_PeakDetection(void *handle, LSM330_ACC_SM2_P_DET_t *value);

/*******************************************************************************
* Register      : PR_SM2
* Address       : 0X7C
* Bit Group Name: SM2_RP
* Permission    : RW
*******************************************************************************/
#define  	LSM330_ACC_SM2_RP_MASK  	0x0F
#define  	LSM330_ACC_SM2_RP_POSITION  	0
status_t  LSM330_ACC_W_SM2_ResetPointer(void *handle, u8_t newValue);
status_t LSM330_ACC_R_SM2_ResetPointer(void *handle, u8_t *value);

/*******************************************************************************
* Register      : PR_SM2
* Address       : 0X7C
* Bit Group Name: SM2_PP
* Permission    : RW
*******************************************************************************/
#define  	LSM330_ACC_SM2_PP_MASK  	0xF0
#define  	LSM330_ACC_SM2_PP_POSITION  	4
status_t  LSM330_ACC_W_SM2_ProgramPointer(void *handle, u8_t newValue);
status_t LSM330_ACC_R_SM2_ProgramPointer(void *handle, u8_t *value);

/*******************************************************************************
* Register      : OUTS_SM2
* Address       : 0X7F
* Bit Group Name: SM2_FLAG_N_V
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM2_FLAG_N_V_DOWN 		 =0x00,
  	LSM330_ACC_SM2_FLAG_N_V_UP 		 =0x01,
} LSM330_ACC_SM2_FLAG_N_V_t;

#define  	LSM330_ACC_SM2_FLAG_N_V_MASK  	0x01
status_t LSM330_ACC_R_SM2_NegativeVectorFlag(void *handle, LSM330_ACC_SM2_FLAG_N_V_t *value);

/*******************************************************************************
* Register      : OUTS_SM2
* Address       : 0X7F
* Bit Group Name: SM2_FLAG_P_V
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM2_FLAG_P_V_DOWN 		 =0x00,
  	LSM330_ACC_SM2_FLAG_P_V_UP 		 =0x02,
} LSM330_ACC_SM2_FLAG_P_V_t;

#define  	LSM330_ACC_SM2_FLAG_P_V_MASK  	0x02
status_t LSM330_ACC_R_SM2_PositiveVectorFlag(void *handle, LSM330_ACC_SM2_FLAG_P_V_t *value);

/*******************************************************************************
* Register      : OUTS_SM2
* Address       : 0X7F
* Bit Group Name: SM2_FLAG_N_Z
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM2_FLAG_N_Z_DOWN 		 =0x00,
  	LSM330_ACC_SM2_FLAG_N_Z_UP 		 =0x04,
} LSM330_ACC_SM2_FLAG_N_Z_t;

#define  	LSM330_ACC_SM2_FLAG_N_Z_MASK  	0x04
status_t LSM330_ACC_R_SM2_NegativeZFlag(void *handle, LSM330_ACC_SM2_FLAG_N_Z_t *value);

/*******************************************************************************
* Register      : OUTS_SM2
* Address       : 0X7F
* Bit Group Name: SM2_FLAG_P_Z
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM2_FLAG_P_Z_DOWN 		 =0x00,
  	LSM330_ACC_SM2_FLAG_P_Z_UP 		 =0x08,
} LSM330_ACC_SM2_FLAG_P_Z_t;

#define  	LSM330_ACC_SM2_FLAG_P_Z_MASK  	0x08
status_t LSM330_ACC_R_SM2_PositiveZFlag(void *handle, LSM330_ACC_SM2_FLAG_P_Z_t *value);

/*******************************************************************************
* Register      : OUTS_SM2
* Address       : 0X7F
* Bit Group Name: SM2_FLAG_N_Y
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM2_FLAG_N_Y_DOWN 		 =0x00,
  	LSM330_ACC_SM2_FLAG_N_Y_UP 		 =0x10,
} LSM330_ACC_SM2_FLAG_N_Y_t;

#define  	LSM330_ACC_SM2_FLAG_N_Y_MASK  	0x10
status_t LSM330_ACC_R_SM2_NegativeYFlag(void *handle, LSM330_ACC_SM2_FLAG_N_Y_t *value);

/*******************************************************************************
* Register      : OUTS_SM2
* Address       : 0X7F
* Bit Group Name: SM2_FLAG_P_Y
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM2_FLAG_P_Y_DOWN 		 =0x00,
  	LSM330_ACC_SM2_FLAG_P_Y_UP 		 =0x20,
} LSM330_ACC_SM2_FLAG_P_Y_t;

#define  	LSM330_ACC_SM2_FLAG_P_Y_MASK  	0x20
status_t LSM330_ACC_R_SM2_PositiveYFlag(void *handle, LSM330_ACC_SM2_FLAG_P_Y_t *value);

/*******************************************************************************
* Register      : OUTS_SM2
* Address       : 0X7F
* Bit Group Name: SM2_FLAG_N_X
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM2_FLAG_N_X_DOWN 		 =0x00,
  	LSM330_ACC_SM2_FLAG_N_X_UP 		 =0x40,
} LSM330_ACC_SM2_FLAG_N_X_t;

#define  	LSM330_ACC_SM2_FLAG_N_X_MASK  	0x40
status_t LSM330_ACC_R_SM2_NegativeXFlag(void *handle, LSM330_ACC_SM2_FLAG_N_X_t *value);

/*******************************************************************************
* Register      : OUTS_SM2
* Address       : 0X7F
* Bit Group Name: SM2_FLAG_P_X
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_ACC_SM2_FLAG_P_X_DOWN 		 =0x00,
  	LSM330_ACC_SM2_FLAG_P_X_UP 		 =0x80,
} LSM330_ACC_SM2_FLAG_P_X_t;

#define  	LSM330_ACC_SM2_FLAG_P_X_MASK  	0x80
status_t LSM330_ACC_R_SM2_PositiveXFlag(void *handle, LSM330_ACC_SM2_FLAG_P_X_t *value);

/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : LongCounter
* Permission    : RW 
*******************************************************************************/
status_t LSM330_ACC_Set_LongCounter(void *handle, u8_t *buff);
status_t LSM330_ACC_Get_LongCounter(void *handle, u8_t *buff); 

/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : VectorFilterCoefficient
* Permission    : RW 
*******************************************************************************/
status_t LSM330_ACC_Set_VectorFilterCoefficient(void *handle, u8_t *buff);
status_t LSM330_ACC_Get_VectorFilterCoefficient(void *handle, u8_t *buff); 
/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : ProgramSM1
* Permission    : RW 
*******************************************************************************/
status_t LSM330_ACC_Set_ProgramSM1(void *handle, u8_t *buff);
status_t LSM330_ACC_Get_ProgramSM1(void *handle, u8_t *buff); 

/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : Timer2_SM1
* Permission    : RW 
*******************************************************************************/
status_t LSM330_ACC_Set_Timer2_SM1(void *handle, u8_t *buff);
status_t LSM330_ACC_Get_Timer2_SM1(void *handle, u8_t *buff); 
/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : Timer1_SM1
* Permission    : RW 
*******************************************************************************/
status_t LSM330_ACC_Set_Timer1_SM1(void *handle, u8_t *buff);
status_t LSM330_ACC_Get_Timer1_SM1(void *handle, u8_t *buff); 

/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : CounterStatus_SM1
* Permission    : RO 
*******************************************************************************/
status_t LSM330_ACC_Get_CounterStatus_SM1(void *handle, u8_t *buff); 
/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : ProgramSM2
* Permission    : RW 
*******************************************************************************/
status_t LSM330_ACC_Set_ProgramSM2(void *handle, u8_t *buff);
status_t LSM330_ACC_Get_ProgramSM2(void *handle, u8_t *buff); 

/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   :  Timer2_SM2
* Permission    : RW 
*******************************************************************************/
status_t LSM330_ACC_Set_Timer2_SM2(void *handle, u8_t *buff);
status_t LSM330_ACC_Get_Timer2_SM2(void *handle, u8_t *buff); 

/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   :  Timer1_SM2
* Permission    : RW 
*******************************************************************************/
status_t LSM330_ACC_Set_Timer1_SM2(void *handle, u8_t *buff);
status_t LSM330_ACC_Get_Timer1_SM2(void *handle, u8_t *buff); 

/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : CounterStatus_SM2
* Permission    : RO 
*******************************************************************************/
status_t LSM330_ACC_Get_CounterStatus_SM2(void *handle, u8_t *buff); 

/*******************************************************************************
* Register      : OUT_T
* Address       : 0X0C
* Bit Group Name: OUT_T_BYTE
* Permission    : RO
*******************************************************************************/
#define  	LSM330_ACC_OUT_T_BYTE_MASK  	0xFF
#define  	LSM330_ACC_OUT_T_BYTE_POSITION  	0
status_t LSM330_ACC_R_Temperature(void *handle, u8_t *value);

/************** Utility  *******************/

/*******************************************************************************
  * Register      : Generic - All
* Address       : Generic - All
* Bit Group Name: None
* Permission    : W
*******************************************************************************/
void LIS2MDL_MAG_SwapHighLowByte(u8_t *bufferToSwap, u8_t numberOfByte, u8_t dimension); 

#ifdef __cplusplus
  }
#endif

#endif

