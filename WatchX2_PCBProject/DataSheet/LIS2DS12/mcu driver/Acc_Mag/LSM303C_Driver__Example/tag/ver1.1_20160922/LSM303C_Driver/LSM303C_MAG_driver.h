/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
* File Name          : LSM303C_MAG_driver.h
* Author             : MEMS Application Team
* Version            : v1.1
* Date               : 22 Nov 2016  
* Description        : LSM303C header driver file
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
#ifndef __LSM303C_MAG_DRIVER__H
#define __LSM303C_MAG_DRIVER__H


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

#define LSM303C_MAG_I2C_ADDRESS         0x3C

/************** Who am I  *******************/

#define LSM303C_MAG_WHO_AM_I         0x3D

/************** Device Register  *******************/
#define LSM303C_MAG_WHO_AM_I_REG  	0X0F
#define LSM303C_MAG_CTRL_REG1  	0X20
#define LSM303C_MAG_CTRL_REG2  	0X21
#define LSM303C_MAG_CTRL_REG3  	0X22
#define LSM303C_MAG_CTRL_REG4  	0X23
#define LSM303C_MAG_CTRL_REG5  	0X24
#define LSM303C_MAG_STATUS_REG  	0X27
#define LSM303C_MAG_OUTX_L  	0X28
#define LSM303C_MAG_OUTX_H  	0X29
#define LSM303C_MAG_OUTY_L  	0X2A
#define LSM303C_MAG_OUTY_H  	0X2B
#define LSM303C_MAG_OUTZ_L  	0X2C
#define LSM303C_MAG_OUTZ_H  	0X2D
#define LSM303C_MAG_TEMP_OUT_L  	0X2E
#define LSM303C_MAG_TEMP_OUT_H  	0X2F
#define LSM303C_MAG_INT_CFG  	0X30
#define LSM303C_MAG_INT_SRC  	0X31
#define LSM303C_MAG_INT_THS_L  	0X32
#define LSM303C_MAG_INT_THS_H  	0X33

/************** Generic Function  *******************/

/*******************************************************************************
* Register      : Generic - All
* Address       : Generic - All
* Bit Group Name: None
* Permission    : W
*******************************************************************************/
status_t LSM303C_ACC_GYRO_WriteReg( void *handle, u8_t Reg, u8_t *Bufp, u16_t len );

/*******************************************************************************
* Register      : Generic - All
* Address       : Generic - All
* Bit Group Name: None
* Permission    : R
*******************************************************************************/
status_t LSM303C_ACC_GYRO_ReadReg( void *handle, u8_t Reg, u8_t *Bufp, u16_t len );

/**************** Base Function  *******************/

/*******************************************************************************
* Register      : WHO_AM_I_REG
* Address       : 0X0F
* Bit Group Name: WHO_AM_I_BIT
* Permission    : RO
*******************************************************************************/
#define  	LSM303C_MAG_WHO_AM_I_BIT_MASK  	0xFF
#define  	LSM303C_MAG_WHO_AM_I_BIT_POSITION  	0
status_t LSM303C_MAG_R_WHO_AM_I_(void *handle, u8_t *value);

/*******************************************************************************
* Register      : CTRL_REG3
* Address       : 0X22
* Bit Group Name: MD
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM303C_MAG_MD_CONTINUOUS 		 =0x00,
  	LSM303C_MAG_MD_SINGLE 		 =0x01,
  	LSM303C_MAG_MD_POWER_DOWN 		 =0x02,
  	LSM303C_MAG_MD_POWER_DOWN_AUTO 		 =0x03,
} LSM303C_MAG_MD_t;

#define  	LSM303C_MAG_MD_MASK  	0x03
status_t  LSM303C_MAG_W_SystemOperatingMode(void *handle, LSM303C_MAG_MD_t newValue);
status_t LSM303C_MAG_R_SystemOperatingMode(void *handle, LSM303C_MAG_MD_t *value);

/*******************************************************************************
* Register      : CTRL_REG5
* Address       : 0X24
* Bit Group Name: BDU
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM303C_MAG_BDU_DISABLE 		 =0x00,
  	LSM303C_MAG_BDU_ENABLE 		 =0x40,
} LSM303C_MAG_BDU_t;

#define  	LSM303C_MAG_BDU_MASK  	0x40
status_t  LSM303C_MAG_W_BlockDataUpdate(void *handle, LSM303C_MAG_BDU_t newValue);
status_t LSM303C_MAG_R_BlockDataUpdate(void *handle, LSM303C_MAG_BDU_t *value);

/*******************************************************************************
* Register      : CTRL_REG2
* Address       : 0X21
* Bit Group Name: FS
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM303C_MAG_FS_4Ga 		 =0x00,
  	LSM303C_MAG_FS_8Ga 		 =0x20,
  	LSM303C_MAG_FS_12Ga 		 =0x40,
  	LSM303C_MAG_FS_16Ga 		 =0x60,
} LSM303C_MAG_FS_t;

#define  	LSM303C_MAG_FS_MASK  	0x60
status_t  LSM303C_MAG_W_FullScale(void *handle, LSM303C_MAG_FS_t newValue);
status_t LSM303C_MAG_R_FullScale(void *handle, LSM303C_MAG_FS_t *value);

/*******************************************************************************
* Register      : CTRL_REG1
* Address       : 0X20
* Bit Group Name: DO
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM303C_MAG_DO_0_625Hz 		 =0x00,
  	LSM303C_MAG_DO_1_25Hz 		 =0x04,
  	LSM303C_MAG_DO_2_5Hz 		 =0x08,
  	LSM303C_MAG_DO_5Hz 		 =0x0C,
  	LSM303C_MAG_DO_10Hz 		 =0x10,
  	LSM303C_MAG_DO_20Hz 		 =0x14,
  	LSM303C_MAG_DO_40Hz 		 =0x18,
  	LSM303C_MAG_DO_80Hz 		 =0x1C,
} LSM303C_MAG_DO_t;

#define  	LSM303C_MAG_DO_MASK  	0x1C
status_t  LSM303C_MAG_W_OutputDataRate(void *handle, LSM303C_MAG_DO_t newValue);
status_t LSM303C_MAG_R_OutputDataRate(void *handle, LSM303C_MAG_DO_t *value);

/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : Magnetic
* Permission    : RO 
*******************************************************************************/
status_t LSM303C_MAG_Get_Magnetic(void *handle,u8_t *buff); 

/**************** Advanced Function  *******************/

/*******************************************************************************
* Register      : CTRL_REG1
* Address       : 0X20
* Bit Group Name: ST
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM303C_MAG_ST_DISABLE 		 =0x00,
  	LSM303C_MAG_ST_ENABLE 		 =0x01,
} LSM303C_MAG_ST_t;

#define  	LSM303C_MAG_ST_MASK  	0x01
status_t  LSM303C_MAG_W_SelfTest(void *handle, LSM303C_MAG_ST_t newValue);
status_t LSM303C_MAG_R_SelfTest(void *handle, LSM303C_MAG_ST_t *value);

/*******************************************************************************
* Register      : CTRL_REG1
* Address       : 0X20
* Bit Group Name: OM
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM303C_MAG_OM_LOW_POWER 		 =0x00,
  	LSM303C_MAG_OM_MEDIUM 		 =0x20,
  	LSM303C_MAG_OM_HIGH 		 =0x40,
  	LSM303C_MAG_OM_ULTRA_HIGH 		 =0x60,
} LSM303C_MAG_OM_t;

#define  	LSM303C_MAG_OM_MASK  	0x60
status_t  LSM303C_MAG_W_OperatingModeXY(void *handle, LSM303C_MAG_OM_t newValue);
status_t LSM303C_MAG_R_OperatingModeXY(void *handle, LSM303C_MAG_OM_t *value);

/*******************************************************************************
* Register      : CTRL_REG1
* Address       : 0X20
* Bit Group Name: TEMP_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM303C_MAG_TEMP_EN_DISABLE 		 =0x00,
  	LSM303C_MAG_TEMP_EN_ENABLE 		 =0x80,
} LSM303C_MAG_TEMP_EN_t;

#define  	LSM303C_MAG_TEMP_EN_MASK  	0x80
status_t  LSM303C_MAG_W_TemperatureSensor(void *handle, LSM303C_MAG_TEMP_EN_t newValue);
status_t LSM303C_MAG_R_TemperatureSensor(void *handle, LSM303C_MAG_TEMP_EN_t *value);

/*******************************************************************************
* Register      : CTRL_REG2
* Address       : 0X21
* Bit Group Name: SOFT_RST
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM303C_MAG_SOFT_RST_NO 		 =0x00,
  	LSM303C_MAG_SOFT_RST_YES 		 =0x04,
} LSM303C_MAG_SOFT_RST_t;

#define  	LSM303C_MAG_SOFT_RST_MASK  	0x04
status_t  LSM303C_MAG_W_SoftRST(void *handle, LSM303C_MAG_SOFT_RST_t newValue);
status_t LSM303C_MAG_R_SoftRST(void *handle, LSM303C_MAG_SOFT_RST_t *value);

/*******************************************************************************
* Register      : CTRL_REG2
* Address       : 0X21
* Bit Group Name: REBOOT
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM303C_MAG_REBOOT_NO 		 =0x00,
  	LSM303C_MAG_REBOOT_YES 		 =0x08,
} LSM303C_MAG_REBOOT_t;

#define  	LSM303C_MAG_REBOOT_MASK  	0x08
status_t  LSM303C_MAG_W_Reboot(void *handle, LSM303C_MAG_REBOOT_t newValue);
status_t LSM303C_MAG_R_Reboot(void *handle, LSM303C_MAG_REBOOT_t *value);

/*******************************************************************************
* Register      : CTRL_REG3
* Address       : 0X22
* Bit Group Name: SIM
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM303C_MAG_SIM_4_WIRE 		 =0x00,
  	LSM303C_MAG_SIM_3_WIRE 		 =0x04,
} LSM303C_MAG_SIM_t;

#define  	LSM303C_MAG_SIM_MASK  	0x04
status_t  LSM303C_MAG_W_SerialInterfaceMode(void *handle, LSM303C_MAG_SIM_t newValue);
status_t LSM303C_MAG_R_SerialInterfaceMode(void *handle, LSM303C_MAG_SIM_t *value);

/*******************************************************************************
* Register      : CTRL_REG3
* Address       : 0X22
* Bit Group Name: LP
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM303C_MAG_LP_DISABLE 		 =0x00,
  	LSM303C_MAG_LP_ENABLE 		 =0x20,
} LSM303C_MAG_LP_t;

#define  	LSM303C_MAG_LP_MASK  	0x20
status_t  LSM303C_MAG_W_FastLowPowerXYZ(void *handle, LSM303C_MAG_LP_t newValue);
status_t LSM303C_MAG_R_FastLowPowerXYZ(void *handle, LSM303C_MAG_LP_t *value);

/*******************************************************************************
* Register      : CTRL_REG4
* Address       : 0X23
* Bit Group Name: BLE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM303C_MAG_BLE_INVERT 		 =0x00,
  	LSM303C_MAG_BLE_DEFAULT 		 =0x02,
} LSM303C_MAG_BLE_t;

#define  	LSM303C_MAG_BLE_MASK  	0x02
status_t  LSM303C_MAG_W_LittleBigEndianInversion(void *handle, LSM303C_MAG_BLE_t newValue);
status_t LSM303C_MAG_R_LittleBigEndianInversion(void *handle, LSM303C_MAG_BLE_t *value);

/*******************************************************************************
* Register      : CTRL_REG4
* Address       : 0X23
* Bit Group Name: OMZ
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM303C_MAG_OMZ_LOW_POWER 		 =0x00,
  	LSM303C_MAG_OMZ_MEDIUM 		 =0x04,
  	LSM303C_MAG_OMZ_HIGH 		 =0x08,
  	LSM303C_MAG_OMZ_ULTRA_HIGH 		 =0x0C,
} LSM303C_MAG_OMZ_t;

#define  	LSM303C_MAG_OMZ_MASK  	0x0C
status_t  LSM303C_MAG_W_OperatingModeZ(void *handle, LSM303C_MAG_OMZ_t newValue);
status_t LSM303C_MAG_R_OperatingModeZ(void *handle, LSM303C_MAG_OMZ_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X27
* Bit Group Name: XDA
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM303C_MAG_XDA_NOT_AVAILABLE 		 =0x00,
  	LSM303C_MAG_XDA_AVAILABLE 		 =0x01,
} LSM303C_MAG_XDA_t;

#define  	LSM303C_MAG_XDA_MASK  	0x01
status_t LSM303C_MAG_R_NewXData(void *handle, LSM303C_MAG_XDA_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X27
* Bit Group Name: YDA
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM303C_MAG_YDA_NOT_AVAILABLE 		 =0x00,
  	LSM303C_MAG_YDA_AVAILABLE 		 =0x02,
} LSM303C_MAG_YDA_t;

#define  	LSM303C_MAG_YDA_MASK  	0x02
status_t LSM303C_MAG_R_NewYData(void *handle, LSM303C_MAG_YDA_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X27
* Bit Group Name: ZDA
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM303C_MAG_ZDA_NOT_AVAILABLE 		 =0x00,
  	LSM303C_MAG_ZDA_AVAILABLE 		 =0x04,
} LSM303C_MAG_ZDA_t;

#define  	LSM303C_MAG_ZDA_MASK  	0x04
status_t LSM303C_MAG_R_NewZData(void *handle, LSM303C_MAG_ZDA_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X27
* Bit Group Name: ZYXDA
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM303C_MAG_ZYXDA_NOT_AVAILABLE 		 =0x00,
  	LSM303C_MAG_ZYXDA_AVAILABLE 		 =0x08,
} LSM303C_MAG_ZYXDA_t;

#define  	LSM303C_MAG_ZYXDA_MASK  	0x08
status_t LSM303C_MAG_R_NewXYZData(void *handle, LSM303C_MAG_ZYXDA_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X27
* Bit Group Name: XOR
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM303C_MAG_XOR_NOT_OVERRUN 		 =0x00,
  	LSM303C_MAG_XOR_OVERRUN 		 =0x10,
} LSM303C_MAG_XOR_t;

#define  	LSM303C_MAG_XOR_MASK  	0x10
status_t LSM303C_MAG_R_DataXOverrun(void *handle, LSM303C_MAG_XOR_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X27
* Bit Group Name: YOR
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM303C_MAG_YOR_NOT_OVERRUN 		 =0x00,
  	LSM303C_MAG_YOR_OVERRUN 		 =0x20,
} LSM303C_MAG_YOR_t;

#define  	LSM303C_MAG_YOR_MASK  	0x20
status_t LSM303C_MAG_R_DataYOverrun(void *handle, LSM303C_MAG_YOR_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X27
* Bit Group Name: ZOR
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM303C_MAG_ZOR_NOT_OVERRUN 		 =0x00,
  	LSM303C_MAG_ZOR_OVERRUN 		 =0x40,
} LSM303C_MAG_ZOR_t;

#define  	LSM303C_MAG_ZOR_MASK  	0x40
status_t LSM303C_MAG_R_DataZOverrun(void *handle, LSM303C_MAG_ZOR_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X27
* Bit Group Name: ZYXOR
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM303C_MAG_ZYXOR_NOT_OVERRUN 		 =0x00,
  	LSM303C_MAG_ZYXOR_OVERRUN 		 =0x80,
} LSM303C_MAG_ZYXOR_t;

#define  	LSM303C_MAG_ZYXOR_MASK  	0x80
status_t LSM303C_MAG_R_DataXYZOverrun(void *handle, LSM303C_MAG_ZYXOR_t *value);

/*******************************************************************************
* Register      : INT_CFG
* Address       : 0X30
* Bit Group Name: IEN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM303C_MAG_IEN_DISABLE 		 =0x00,
  	LSM303C_MAG_IEN_ENABLE 		 =0x01,
} LSM303C_MAG_IEN_t;

#define  	LSM303C_MAG_IEN_MASK  	0x01
status_t  LSM303C_MAG_W_InterruptEnable(void *handle, LSM303C_MAG_IEN_t newValue);
status_t LSM303C_MAG_R_InterruptEnable(void *handle, LSM303C_MAG_IEN_t *value);

/*******************************************************************************
* Register      : INT_CFG
* Address       : 0X30
* Bit Group Name: LIR
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM303C_MAG_LIR_LATCHED 		 =0x00,
  	LSM303C_MAG_LIR_NOT_LATCHED 		 =0x02,
} LSM303C_MAG_LIR_t;

#define  	LSM303C_MAG_LIR_MASK  	0x02
status_t  LSM303C_MAG_W_LatchInterruptRq(void *handle, LSM303C_MAG_LIR_t newValue);
status_t LSM303C_MAG_R_LatchInterruptRq(void *handle, LSM303C_MAG_LIR_t *value);

/*******************************************************************************
* Register      : INT_CFG
* Address       : 0X30
* Bit Group Name: IEA
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM303C_MAG_IEA_LOW 		 =0x00,
  	LSM303C_MAG_IEA_HIGH 		 =0x04,
} LSM303C_MAG_IEA_t;

#define  	LSM303C_MAG_IEA_MASK  	0x04
status_t  LSM303C_MAG_W_InterruptActive(void *handle, LSM303C_MAG_IEA_t newValue);
status_t LSM303C_MAG_R_InterruptActive(void *handle, LSM303C_MAG_IEA_t *value);

/*******************************************************************************
* Register      : INT_CFG
* Address       : 0X30
* Bit Group Name: ZIEN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM303C_MAG_ZIEN_DISABLE 		 =0x00,
  	LSM303C_MAG_ZIEN_ENABLE 		 =0x20,
} LSM303C_MAG_ZIEN_t;

#define  	LSM303C_MAG_ZIEN_MASK  	0x20
status_t  LSM303C_MAG_W_InterruptOnZ(void *handle, LSM303C_MAG_ZIEN_t newValue);
status_t LSM303C_MAG_R_InterruptOnZ(void *handle, LSM303C_MAG_ZIEN_t *value);

/*******************************************************************************
* Register      : INT_CFG
* Address       : 0X30
* Bit Group Name: YIEN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM303C_MAG_YIEN_DISABLE 		 =0x00,
  	LSM303C_MAG_YIEN_ENABLE 		 =0x40,
} LSM303C_MAG_YIEN_t;

#define  	LSM303C_MAG_YIEN_MASK  	0x40
status_t  LSM303C_MAG_W_InterruptOnY(void *handle, LSM303C_MAG_YIEN_t newValue);
status_t LSM303C_MAG_R_InterruptOnY(void *handle, LSM303C_MAG_YIEN_t *value);

/*******************************************************************************
* Register      : INT_CFG
* Address       : 0X30
* Bit Group Name: XIEN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM303C_MAG_XIEN_DISABLE 		 =0x00,
  	LSM303C_MAG_XIEN_ENABLE 		 =0x80,
} LSM303C_MAG_XIEN_t;

#define  	LSM303C_MAG_XIEN_MASK  	0x80
status_t  LSM303C_MAG_W_InterruptOnX(void *handle, LSM303C_MAG_XIEN_t newValue);
status_t LSM303C_MAG_R_InterruptOnX(void *handle, LSM303C_MAG_XIEN_t *value);

/*******************************************************************************
* Register      : INT_SRC
* Address       : 0X31
* Bit Group Name: INT
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM303C_MAG_INT_DOWN 		 =0x00,
  	LSM303C_MAG_INT_UP 		 =0x01,
} LSM303C_MAG_INT_t;

#define  	LSM303C_MAG_INT_MASK  	0x01
status_t  LSM303C_MAG_W_InterruptFlag(void *handle, LSM303C_MAG_INT_t newValue);
status_t LSM303C_MAG_R_InterruptFlag(void *handle, LSM303C_MAG_INT_t *value);

/*******************************************************************************
* Register      : INT_SRC
* Address       : 0X31
* Bit Group Name: MROI
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM303C_MAG_MROI_IN_RANGE 		 =0x00,
  	LSM303C_MAG_MROI_OVERFLOW 		 =0x02,
} LSM303C_MAG_MROI_t;

#define  	LSM303C_MAG_MROI_MASK  	0x02
status_t  LSM303C_MAG_W_MagneticFieldOverflow(void *handle, LSM303C_MAG_MROI_t newValue);
status_t LSM303C_MAG_R_MagneticFieldOverflow(void *handle, LSM303C_MAG_MROI_t *value);

/*******************************************************************************
* Register      : INT_SRC
* Address       : 0X31
* Bit Group Name: NTH_Z
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM303C_MAG_NTH_Z_DOWN 		 =0x00,
  	LSM303C_MAG_NTH_Z_UP 		 =0x04,
} LSM303C_MAG_NTH_Z_t;

#define  	LSM303C_MAG_NTH_Z_MASK  	0x04
status_t  LSM303C_MAG_W_NegativeThresholdFlagZ(void *handle, LSM303C_MAG_NTH_Z_t newValue);
status_t LSM303C_MAG_R_NegativeThresholdFlagZ(void *handle, LSM303C_MAG_NTH_Z_t *value);

/*******************************************************************************
* Register      : INT_SRC
* Address       : 0X31
* Bit Group Name: NTH_Y
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM303C_MAG_NTH_Y_DOWN 		 =0x00,
  	LSM303C_MAG_NTH_Y_UP 		 =0x08,
} LSM303C_MAG_NTH_Y_t;

#define  	LSM303C_MAG_NTH_Y_MASK  	0x08
status_t  LSM303C_MAG_W_NegativeThresholdFlagY(void *handle, LSM303C_MAG_NTH_Y_t newValue);
status_t LSM303C_MAG_R_NegativeThresholdFlagY(void *handle, LSM303C_MAG_NTH_Y_t *value);

/*******************************************************************************
* Register      : INT_SRC
* Address       : 0X31
* Bit Group Name: NTH_X
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM303C_MAG_NTH_X_DOWN 		 =0x00,
  	LSM303C_MAG_NTH_X_UP 		 =0x10,
} LSM303C_MAG_NTH_X_t;

#define  	LSM303C_MAG_NTH_X_MASK  	0x10
status_t  LSM303C_MAG_W_NegativeThresholdFlagX(void *handle, LSM303C_MAG_NTH_X_t newValue);
status_t LSM303C_MAG_R_NegativeThresholdFlagX(void *handle, LSM303C_MAG_NTH_X_t *value);

/*******************************************************************************
* Register      : INT_SRC
* Address       : 0X31
* Bit Group Name: PTH_Z
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM303C_MAG_PTH_Z_DOWN 		 =0x00,
  	LSM303C_MAG_PTH_Z_UP 		 =0x20,
} LSM303C_MAG_PTH_Z_t;

#define  	LSM303C_MAG_PTH_Z_MASK  	0x20
status_t  LSM303C_MAG_W_PositiveThresholdFlagZ(void *handle, LSM303C_MAG_PTH_Z_t newValue);
status_t LSM303C_MAG_R_PositiveThresholdFlagZ(void *handle, LSM303C_MAG_PTH_Z_t *value);

/*******************************************************************************
* Register      : INT_SRC
* Address       : 0X31
* Bit Group Name: PTH_Y
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM303C_MAG_PTH_Y_DOWN 		 =0x00,
  	LSM303C_MAG_PTH_Y_UP 		 =0x40,
} LSM303C_MAG_PTH_Y_t;

#define  	LSM303C_MAG_PTH_Y_MASK  	0x40
status_t  LSM303C_MAG_W_PositiveThresholdFlagY(void *handle, LSM303C_MAG_PTH_Y_t newValue);
status_t LSM303C_MAG_R_PositiveThresholdFlagY(void *handle, LSM303C_MAG_PTH_Y_t *value);

/*******************************************************************************
* Register      : INT_SRC
* Address       : 0X31
* Bit Group Name: PTH_X
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM303C_MAG_PTH_X_DOWN 		 =0x00,
  	LSM303C_MAG_PTH_X_UP 		 =0x80,
} LSM303C_MAG_PTH_X_t;

#define  	LSM303C_MAG_PTH_X_MASK  	0x80
status_t  LSM303C_MAG_W_PositiveThresholdFlagX(void *handle, LSM303C_MAG_PTH_X_t newValue);
status_t LSM303C_MAG_R_PositiveThresholdFlagX(void *handle, LSM303C_MAG_PTH_X_t *value);

/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : Temperature
* Permission    : RO 
*******************************************************************************/
status_t LSM303C_MAG_Get_Temperature(void *handle,u8_t *buff); 

/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : MagneticThreshold
* Permission    : RW 
*******************************************************************************/
status_t LSM303C_MAG_Set_MagneticThreshold(void *handle,u8_t *buff);
status_t LSM303C_MAG_Get_MagneticThreshold(void *handle,u8_t *buff); 

/*******************************************************************************
* Register      : Generic - All
* Address       : Generic - All
* Bit Group Name: None
* Permission    : W
*******************************************************************************/
void LSM303C_MAG_SwapHighLowByte(u8_t *bufferToSwap, u8_t numberOfByte, u8_t dimension); 

#ifdef __cplusplus
}
#endif

#endif

