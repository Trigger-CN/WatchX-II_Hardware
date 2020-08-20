/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
* File Name          : LSM330_GYRO_driver.h
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
#ifndef __LSM330_GYRO_DRIVER__H
#define __LSM330_GYRO_DRIVER__H

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

#define LSM330_GYRO_I2C_ADDRESS         0xD6

/************** Who am I  *******************/

#define LSM330_GYRO_WHO_AM_I         0xD4

/************** Device Register  *******************/
#define LSM330_GYRO_WHO_AM_I_REG  	0X0F
#define LSM330_GYRO_CTRL_REG1  	0X20
#define LSM330_GYRO_CTRL_REG2  	0X21
#define LSM330_GYRO_CTRL_REG3  	0X22
#define LSM330_GYRO_CTRL_REG4  	0X23
#define LSM330_GYRO_CTRL_REG5  	0X24
#define LSM330_GYRO_REFERENCE  	0X25
#define LSM330_GYRO_OUT_TEMP  	0X26
#define LSM330_GYRO_STATUS_REG  	0X27
#define LSM330_GYRO_OUT_X_L  	0X28
#define LSM330_GYRO_OUT_X_H  	0X29
#define LSM330_GYRO_OUT_Y_L  	0X2A
#define LSM330_GYRO_OUT_Y_H  	0X2B
#define LSM330_GYRO_OUT_Z_L  	0X2C
#define LSM330_GYRO_OUT_Z_H  	0X2D
#define LSM330_GYRO_FIFO_CTRL_REG  	0X2E
#define LSM330_GYRO_FIFO_SRC_REG  	0X2F
#define LSM330_GYRO_INT1_CFG  	0X30
#define LSM330_GYRO_INT1_SRC  	0X31
#define LSM330_GYRO_INT1_TSH_XH  	0X32
#define LSM330_GYRO_INT1_TSH_XL  	0X33
#define LSM330_GYRO_INT1_TSH_YH  	0X34
#define LSM330_GYRO_INT1_TSH_YL  	0X35
#define LSM330_GYRO_INT1_TSH_ZH  	0X36
#define LSM330_GYRO_INT1_TSH_ZL  	0X37
#define LSM330_GYRO_INT1_DURATION  	0X38

/************** Generic Function  *******************/

/*******************************************************************************
* Register      : Generic - All
* Address       : Generic - All
* Bit Group Name: None
* Permission    : W
*******************************************************************************/
status_t LSM330_GYRO_WriteReg( void *handle, u8_t Reg, u8_t *Bufp, u16_t len );

/*******************************************************************************
* Register      : Generic - All
* Address       : Generic - All
* Bit Group Name: None
* Permission    : R
*******************************************************************************/
status_t LSM330_GYRO_ReadReg( void *handle, u8_t Reg, u8_t *Bufp, u16_t len );

/**************** Base Function  *******************/

/*******************************************************************************
* Register      : WHO_AM_I_REG
* Address       : 0X0F
* Bit Group Name: WHO_AM_I_BIT
* Permission    : RO
*******************************************************************************/
#define  	LSM330_GYRO_WHO_AM_I_BIT_MASK  	0xFF
#define  	LSM330_GYRO_WHO_AM_I_BIT_POSITION  	0
status_t LSM330_GYRO_R_WHO_AM_I_(void *handle, u8_t *value);

/*******************************************************************************
* Register      : CTRL_REG1
* Address       : 0X20
* Bit Group Name: DR
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_DR_95Hz 		 =0x00,
  	LSM330_GYRO_DR_190Hz 		 =0x40,
  	LSM330_GYRO_DR_380Hz 		 =0x80,
  	LSM330_GYRO_DR_760Hz 		 =0xC0,
} LSM330_GYRO_DR_t;

#define  	LSM330_GYRO_DR_MASK  	0xC0
status_t  LSM330_GYRO_W_DataRate(void *handle, LSM330_GYRO_DR_t newValue);
status_t LSM330_GYRO_R_DataRate(void *handle, LSM330_GYRO_DR_t *value);

/*******************************************************************************
* Register      : CTRL_REG4
* Address       : 0X23
* Bit Group Name: FS
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_FS_250dps 		 =0x00,
  	LSM330_GYRO_FS_500dps 		 =0x10,
  	LSM330_GYRO_FS_2000dps 		 =0x20,
} LSM330_GYRO_FS_t;

#define  	LSM330_GYRO_FS_MASK  	0x30
status_t  LSM330_GYRO_W_FullScale(void *handle, LSM330_GYRO_FS_t newValue);
status_t LSM330_GYRO_R_FullScale(void *handle, LSM330_GYRO_FS_t *value);

/*******************************************************************************
* Register      : CTRL_REG4
* Address       : 0X23
* Bit Group Name: BDU
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_BDU_DISABLE 		 =0x00,
  	LSM330_GYRO_BDU_ENABLE 		 =0x80,
} LSM330_GYRO_BDU_t;

#define  	LSM330_GYRO_BDU_MASK  	0x80
status_t  LSM330_GYRO_W_BlockDataUpdate(void *handle, LSM330_GYRO_BDU_t newValue);
status_t LSM330_GYRO_R_BlockDataUpdate(void *handle, LSM330_GYRO_BDU_t *value);

/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : AngularRate
* Permission    : RO 
*******************************************************************************/
status_t LSM330_GYRO_Get_AngularRate(void *handle, u8_t *buff); 

/*******************************************************************************
* Register      : CTRL_REG1
* Address       : 0X20
* Bit Group Name: YEN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_YEN_DISABLE 		 =0x00,
  	LSM330_GYRO_YEN_ENABLE 		 =0x01,
} LSM330_GYRO_YEN_t;

#define  	LSM330_GYRO_YEN_MASK  	0x01
status_t  LSM330_GYRO_W_AxisY(void *handle, LSM330_GYRO_YEN_t newValue);
status_t LSM330_GYRO_R_AxisY(void *handle, LSM330_GYRO_YEN_t *value);

/*******************************************************************************
* Register      : CTRL_REG1
* Address       : 0X20
* Bit Group Name: XEN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_XEN_DISABLE 		 =0x00,
  	LSM330_GYRO_XEN_ENABLE 		 =0x02,
} LSM330_GYRO_XEN_t;

#define  	LSM330_GYRO_XEN_MASK  	0x02
status_t  LSM330_GYRO_W_AxisX(void *handle, LSM330_GYRO_XEN_t newValue);
status_t LSM330_GYRO_R_AxisX(void *handle, LSM330_GYRO_XEN_t *value);

/*******************************************************************************
* Register      : CTRL_REG1
* Address       : 0X20
* Bit Group Name: ZEN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_ZEN_DISABLE 		 =0x00,
  	LSM330_GYRO_ZEN_ENABLE 		 =0x04,
} LSM330_GYRO_ZEN_t;

#define  	LSM330_GYRO_ZEN_MASK  	0x04
status_t  LSM330_GYRO_W_AxisZ(void *handle, LSM330_GYRO_ZEN_t newValue);
status_t LSM330_GYRO_R_AxisZ(void *handle, LSM330_GYRO_ZEN_t *value);

/*******************************************************************************
* Register      : CTRL_REG1
* Address       : 0X20
* Bit Group Name: PD
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_PD_POWER_DOWN 		 =0x00,
  	LSM330_GYRO_PD_NORMAL_OR_SLEEP 		 =0x08,
} LSM330_GYRO_PD_t;

#define  	LSM330_GYRO_PD_MASK  	0x08
status_t  LSM330_GYRO_W_SystemStatus(void *handle, LSM330_GYRO_PD_t newValue);
status_t LSM330_GYRO_R_SystemStatus(void *handle, LSM330_GYRO_PD_t *value);

/*******************************************************************************
* Register      : CTRL_REG1
* Address       : 0X20
* Bit Group Name: BW
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_BW_LOW 		 =0x00,
  	LSM330_GYRO_BW_NORMAL 		 =0x10,
  	LSM330_GYRO_BW_HIGH 		 =0x20,
  	LSM330_GYRO_BW_ULTRA_HIGH 		 =0x30,
} LSM330_GYRO_BW_t;

#define  	LSM330_GYRO_BW_MASK  	0x30
status_t  LSM330_GYRO_W_Bandwidth(void *handle, LSM330_GYRO_BW_t newValue);
status_t LSM330_GYRO_R_Bandwidth(void *handle, LSM330_GYRO_BW_t *value);

/*******************************************************************************
* Register      : CTRL_REG2
* Address       : 0X21
* Bit Group Name: HPCF
* Permission    : RW
*******************************************************************************/
#define  	LSM330_GYRO_HPCF_MASK  	0x0F
#define  	LSM330_GYRO_HPCF_POSITION  	0
status_t  LSM330_GYRO_W_HighPassFilterCutoffFreq(void *handle, u8_t newValue);
status_t LSM330_GYRO_R_HighPassFilterCutoffFreq(void *handle, u8_t *value);

/*******************************************************************************
* Register      : CTRL_REG2
* Address       : 0X21
* Bit Group Name: HPM
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_HPM_RST_READ_HP_RESET_FILTER 		 =0x00,
  	LSM330_GYRO_HPM_USE_REFERENCE 		 =0x10,
  	LSM330_GYRO_HPM_NORMAL_MODE 		 =0x20,
  	LSM330_GYRO_HPM_AUTORESET 		 =0x30,
} LSM330_GYRO_HPM_t;

#define  	LSM330_GYRO_HPM_MASK  	0x30
status_t  LSM330_GYRO_W_HighPassFilterMode(void *handle, LSM330_GYRO_HPM_t newValue);
status_t LSM330_GYRO_R_HighPassFilterMode(void *handle, LSM330_GYRO_HPM_t *value);

/*******************************************************************************
* Register      : CTRL_REG3
* Address       : 0X22
* Bit Group Name: I2_EMPTY
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_I2_EMPTY_DISABLE 		 =0x00,
  	LSM330_GYRO_I2_EMPTY_ENABLE 		 =0x01,
} LSM330_GYRO_I2_EMPTY_t;

#define  	LSM330_GYRO_I2_EMPTY_MASK  	0x01
status_t  LSM330_GYRO_W_FIFO_empty_INT2(void *handle, LSM330_GYRO_I2_EMPTY_t newValue);
status_t LSM330_GYRO_R_FIFO_empty_INT2(void *handle, LSM330_GYRO_I2_EMPTY_t *value);

/*******************************************************************************
* Register      : CTRL_REG3
* Address       : 0X22
* Bit Group Name: I2_ORUN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_I2_ORUN_DISABLE 		 =0x00,
  	LSM330_GYRO_I2_ORUN_ENABLE 		 =0x02,
} LSM330_GYRO_I2_ORUN_t;

#define  	LSM330_GYRO_I2_ORUN_MASK  	0x02
status_t  LSM330_GYRO_W_FIFO_overrun_INT2(void *handle, LSM330_GYRO_I2_ORUN_t newValue);
status_t LSM330_GYRO_R_FIFO_overrun_INT2(void *handle, LSM330_GYRO_I2_ORUN_t *value);

/*******************************************************************************
* Register      : CTRL_REG3
* Address       : 0X22
* Bit Group Name: I2_WTM
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_I2_WTM_DISABLE 		 =0x00,
  	LSM330_GYRO_I2_WTM_ENABLE 		 =0x04,
} LSM330_GYRO_I2_WTM_t;

#define  	LSM330_GYRO_I2_WTM_MASK  	0x04
status_t  LSM330_GYRO_W_FIFO_watermark_INT2(void *handle, LSM330_GYRO_I2_WTM_t newValue);
status_t LSM330_GYRO_R_FIFO_watermark_INT2(void *handle, LSM330_GYRO_I2_WTM_t *value);

/*******************************************************************************
* Register      : CTRL_REG3
* Address       : 0X22
* Bit Group Name: I2_DRDY
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_I2_DRDY_DISABLE 		 =0x00,
  	LSM330_GYRO_I2_DRDY_ENABLE 		 =0x08,
} LSM330_GYRO_I2_DRDY_t;

#define  	LSM330_GYRO_I2_DRDY_MASK  	0x08
status_t  LSM330_GYRO_W_DataReady_INT2(void *handle, LSM330_GYRO_I2_DRDY_t newValue);
status_t LSM330_GYRO_R_DataReady_INT2(void *handle, LSM330_GYRO_I2_DRDY_t *value);

/*******************************************************************************
* Register      : CTRL_REG3
* Address       : 0X22
* Bit Group Name: PP_OD
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_PP_OD_PUSH_PULL 		 =0x00,
  	LSM330_GYRO_PP_OD_OPEN_DRAIN 		 =0x10,
} LSM330_GYRO_PP_OD_t;

#define  	LSM330_GYRO_PP_OD_MASK  	0x10
status_t  LSM330_GYRO_W_PinConfiguration(void *handle, LSM330_GYRO_PP_OD_t newValue);
status_t LSM330_GYRO_R_PinConfiguration(void *handle, LSM330_GYRO_PP_OD_t *value);

/*******************************************************************************
* Register      : CTRL_REG3
* Address       : 0X22
* Bit Group Name: H_LACTIVE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_H_LACTIVE_HIGH 		 =0x00,
  	LSM330_GYRO_H_LACTIVE_LOW 		 =0x20,
} LSM330_GYRO_H_LACTIVE_t;

#define  	LSM330_GYRO_H_LACTIVE_MASK  	0x20
status_t  LSM330_GYRO_W_InterruptActive(void *handle, LSM330_GYRO_H_LACTIVE_t newValue);
status_t LSM330_GYRO_R_InterruptActive(void *handle, LSM330_GYRO_H_LACTIVE_t *value);

/*******************************************************************************
* Register      : CTRL_REG3
* Address       : 0X22
* Bit Group Name: I1_BOOT
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_I1_BOOT_DISABLE 		 =0x00,
  	LSM330_GYRO_I1_BOOT_ENABLE 		 =0x40,
} LSM330_GYRO_I1_BOOT_t;

#define  	LSM330_GYRO_I1_BOOT_MASK  	0x40
status_t  LSM330_GYRO_W_BootStatusOnINT1(void *handle, LSM330_GYRO_I1_BOOT_t newValue);
status_t LSM330_GYRO_R_BootStatusOnINT1(void *handle, LSM330_GYRO_I1_BOOT_t *value);

/*******************************************************************************
* Register      : CTRL_REG3
* Address       : 0X22
* Bit Group Name: I1_INT1
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_I1_INT1_DISABLE 		 =0x00,
  	LSM330_GYRO_I1_INT1_ENABLE 		 =0x80,
} LSM330_GYRO_I1_INT1_t;

#define  	LSM330_GYRO_I1_INT1_MASK  	0x80
status_t  LSM330_GYRO_W_EnableIntOnINT1(void *handle, LSM330_GYRO_I1_INT1_t newValue);
status_t LSM330_GYRO_R_EnableIntOnINT1(void *handle, LSM330_GYRO_I1_INT1_t *value);

/*******************************************************************************
* Register      : CTRL_REG4
* Address       : 0X23
* Bit Group Name: SIM
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_SIM_4WIRE 		 =0x00,
  	LSM330_GYRO_SIM_3WIRE 		 =0x01,
} LSM330_GYRO_SIM_t;

#define  	LSM330_GYRO_SIM_MASK  	0x01
status_t  LSM330_GYRO_W_SPI_Configuration(void *handle, LSM330_GYRO_SIM_t newValue);
status_t LSM330_GYRO_R_SPI_Configuration(void *handle, LSM330_GYRO_SIM_t *value);

/*******************************************************************************
* Register      : CTRL_REG4
* Address       : 0X23
* Bit Group Name: BLE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_BLE_LSB_AT_LOW 		 =0x00,
  	LSM330_GYRO_BLE_MSB_AT_LOW 		 =0x40,
} LSM330_GYRO_BLE_t;

#define  	LSM330_GYRO_BLE_MASK  	0x40
status_t  LSM330_GYRO_W_BigLittleEndianSelection(void *handle, LSM330_GYRO_BLE_t newValue);
status_t LSM330_GYRO_R_BigLittleEndianSelection(void *handle, LSM330_GYRO_BLE_t *value);

/*******************************************************************************
* Register      : CTRL_REG5
* Address       : 0X24
* Bit Group Name: OUT_SEL
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_OUT_SEL_BYPASS_HPF_AND_LPF2 		 =0x00,
  	LSM330_GYRO_OUT_SEL_BYPASS_LPF2 		 =0x01,
  	LSM330_GYRO_OUT_SEL_USE_HPF_AND_LPF2 		 =0x02,
} LSM330_GYRO_OUT_SEL_t;

#define  	LSM330_GYRO_OUT_SEL_MASK  	0x03
status_t  LSM330_GYRO_W_OutputDataSelection(void *handle, LSM330_GYRO_OUT_SEL_t newValue);
status_t LSM330_GYRO_R_OutputDataSelection(void *handle, LSM330_GYRO_OUT_SEL_t *value);

/*******************************************************************************
* Register      : CTRL_REG5
* Address       : 0X24
* Bit Group Name: INT1_SEL
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_INT1_SEL_BYPASS_HFP_AND_LPF2 		 =0x00,
  	LSM330_GYRO_INT1_SEL_BYPASS_LPF2 		 =0x04,
  	LSM330_GYRO_INT1_SEL_USE_HFP_AND_LPF2 		 =0x08,
} LSM330_GYRO_INT1_SEL_t;

#define  	LSM330_GYRO_INT1_SEL_MASK  	0x0C
status_t  LSM330_GYRO_W_InterruptDataSelection(void *handle, LSM330_GYRO_INT1_SEL_t newValue);
status_t LSM330_GYRO_R_InterruptDataSelection(void *handle, LSM330_GYRO_INT1_SEL_t *value);

/*******************************************************************************
* Register      : CTRL_REG5
* Address       : 0X24
* Bit Group Name: HPEN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_HPEN_DISABLE 		 =0x00,
  	LSM330_GYRO_HPEN_ENABLE 		 =0x10,
} LSM330_GYRO_HPEN_t;

#define  	LSM330_GYRO_HPEN_MASK  	0x10
status_t  LSM330_GYRO_W_HighPassFilter(void *handle, LSM330_GYRO_HPEN_t newValue);
status_t LSM330_GYRO_R_HighPassFilter(void *handle, LSM330_GYRO_HPEN_t *value);

/*******************************************************************************
* Register      : CTRL_REG5
* Address       : 0X24
* Bit Group Name: FIFO_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_FIFO_EN_DISABLE 		 =0x00,
  	LSM330_GYRO_FIFO_EN_ENABLE 		 =0x40,
} LSM330_GYRO_FIFO_EN_t;

#define  	LSM330_GYRO_FIFO_EN_MASK  	0x40
status_t  LSM330_GYRO_W_OpenFIFO(void *handle, LSM330_GYRO_FIFO_EN_t newValue);
status_t LSM330_GYRO_R_OpenFIFO(void *handle, LSM330_GYRO_FIFO_EN_t *value);

/*******************************************************************************
* Register      : CTRL_REG5
* Address       : 0X24
* Bit Group Name: BOOT
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_BOOT_NO 		 =0x00,
  	LSM330_GYRO_BOOT_YES 		 =0x80,
} LSM330_GYRO_BOOT_t;

#define  	LSM330_GYRO_BOOT_MASK  	0x80
status_t  LSM330_GYRO_W_RebootMemory(void *handle, LSM330_GYRO_BOOT_t newValue);
status_t LSM330_GYRO_R_RebootMemory(void *handle, LSM330_GYRO_BOOT_t *value);

/*******************************************************************************
* Register      : OUT_TEMP
* Address       : 0X26
* Bit Group Name: OUT_TEMP_BIT
* Permission    : RO
*******************************************************************************/
#define  	LSM330_GYRO_OUT_TEMP_BIT_MASK  	0xFF
#define  	LSM330_GYRO_OUT_TEMP_BIT_POSITION  	0
status_t LSM330_GYRO_R_Temperature(void *handle, u8_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X27
* Bit Group Name: XDA
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_XDA_NOT_AVAILABLE 		 =0x00,
  	LSM330_GYRO_XDA_AVAILABLE 		 =0x01,
} LSM330_GYRO_XDA_t;

#define  	LSM330_GYRO_XDA_MASK  	0x01
status_t LSM330_GYRO_R_NewDataX(void *handle, LSM330_GYRO_XDA_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X27
* Bit Group Name: YDA
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_YDA_NOT_AVAILABLE 		 =0x00,
  	LSM330_GYRO_YDA_AVAILABLE 		 =0x02,
} LSM330_GYRO_YDA_t;

#define  	LSM330_GYRO_YDA_MASK  	0x02
status_t LSM330_GYRO_R_NewDataY(void *handle, LSM330_GYRO_YDA_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X27
* Bit Group Name: ZDA
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_ZDA_NOT_AVAILABLE 		 =0x00,
  	LSM330_GYRO_ZDA_AVAILABLE 		 =0x04,
} LSM330_GYRO_ZDA_t;

#define  	LSM330_GYRO_ZDA_MASK  	0x04
status_t LSM330_GYRO_R_NewDataZ(void *handle, LSM330_GYRO_ZDA_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X27
* Bit Group Name: ZYXDA
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_ZYXDA_NOT_AVAILABLE 		 =0x00,
  	LSM330_GYRO_ZYXDA_AVAILABLE 		 =0x08,
} LSM330_GYRO_ZYXDA_t;

#define  	LSM330_GYRO_ZYXDA_MASK  	0x08
status_t LSM330_GYRO_R_NewDataXYZ(void *handle, LSM330_GYRO_ZYXDA_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X27
* Bit Group Name: XOR
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_XOR_NO_OVERRUN 		 =0x00,
  	LSM330_GYRO_XOR_OVERRUN 		 =0x10,
} LSM330_GYRO_XOR_t;

#define  	LSM330_GYRO_XOR_MASK  	0x10
status_t LSM330_GYRO_R_DataOverrunX(void *handle, LSM330_GYRO_XOR_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X27
* Bit Group Name: YOR
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_YOR_NO_OVERRUN 		 =0x00,
  	LSM330_GYRO_YOR_OVERRUN 		 =0x20,
} LSM330_GYRO_YOR_t;

#define  	LSM330_GYRO_YOR_MASK  	0x20
status_t LSM330_GYRO_R_DataOverrunY(void *handle, LSM330_GYRO_YOR_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X27
* Bit Group Name: ZOR
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_ZOR_NO_OVERRUN 		 =0x00,
  	LSM330_GYRO_ZOR_OVERRUN 		 =0x40,
} LSM330_GYRO_ZOR_t;

#define  	LSM330_GYRO_ZOR_MASK  	0x40
status_t LSM330_GYRO_R_DataOverrunZ(void *handle, LSM330_GYRO_ZOR_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X27
* Bit Group Name: ZYXOR
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_ZYXOR_NO_OVERRUN 		 =0x00,
  	LSM330_GYRO_ZYXOR_OVERRUN 		 =0x80,
} LSM330_GYRO_ZYXOR_t;

#define  	LSM330_GYRO_ZYXOR_MASK  	0x80
status_t LSM330_GYRO_R_DataOverrunXYZ(void *handle, LSM330_GYRO_ZYXOR_t *value);

/*******************************************************************************
* Register      : FIFO_CTRL_REG
* Address       : 0X2E
* Bit Group Name: WTM
* Permission    : RW
*******************************************************************************/
#define  	LSM330_GYRO_WTM_MASK  	0x1F
#define  	LSM330_GYRO_WTM_POSITION  	0
status_t  LSM330_GYRO_W_FIFO_Watermark(void *handle, u8_t newValue);
status_t LSM330_GYRO_R_FIFO_Watermark(void *handle, u8_t *value);

/*******************************************************************************
* Register      : FIFO_CTRL_REG
* Address       : 0X2E
* Bit Group Name: FM
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_FM_BYPASS 		 =0x00,
  	LSM330_GYRO_FM_FIFO 		 =0x20,
  	LSM330_GYRO_FM_STREAM 		 =0x40,
  	LSM330_GYRO_FM_STREAM_TO_FIFO 		 =0x60,
  	LSM330_GYRO_FM_BYPASS_TO_STREAM 		 =0x80,
} LSM330_GYRO_FM_t;

#define  	LSM330_GYRO_FM_MASK  	0xE0
status_t  LSM330_GYRO_W_FIFO_Mode(void *handle, LSM330_GYRO_FM_t newValue);
status_t LSM330_GYRO_R_FIFO_Mode(void *handle, LSM330_GYRO_FM_t *value);

/*******************************************************************************
* Register      : FIFO_SRC_REG
* Address       : 0X2F
* Bit Group Name: EMPTY
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_EMPTY_DOWN 		 =0x00,
  	LSM330_GYRO_EMPTY_UP 		 =0x20,
} LSM330_GYRO_EMPTY_t;

#define  	LSM330_GYRO_EMPTY_MASK  	0x20
status_t LSM330_GYRO_R_FIFO_EmptyFlag(void *handle, LSM330_GYRO_EMPTY_t *value);

/*******************************************************************************
* Register      : FIFO_SRC_REG
* Address       : 0X2F
* Bit Group Name: OVRN
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_OVRN_DOWN 		 =0x00,
  	LSM330_GYRO_OVRN_UP 		 =0x40,
} LSM330_GYRO_OVRN_t;

#define  	LSM330_GYRO_OVRN_MASK  	0x40
status_t LSM330_GYRO_R_FIFO_OverrunFlag(void *handle, LSM330_GYRO_OVRN_t *value);

/*******************************************************************************
* Register      : FIFO_SRC_REG
* Address       : 0X2F
* Bit Group Name: WTM
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_WTM_DOWN 		 =0x00,
  	LSM330_GYRO_WTM_UP 		 =0x80,
} LSM330_GYRO_WTM_t;

#define  	LSM330_GYRO_WTM_FLAG_MASK  	0x80
status_t LSM330_GYRO_R_FIFO_WatermarkFlag(void *handle, LSM330_GYRO_WTM_t *value);

/*******************************************************************************
* Register      : INT1_CFG
* Address       : 0X30
* Bit Group Name: XLIE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_XLIE_DISABLE 		 =0x00,
  	LSM330_GYRO_XLIE_ENABLE 		 =0x01,
} LSM330_GYRO_XLIE_t;

#define  	LSM330_GYRO_XLIE_MASK  	0x01
status_t  LSM330_GYRO_W_LowLevelInterruptOnX(void *handle, LSM330_GYRO_XLIE_t newValue);
status_t LSM330_GYRO_R_LowLevelInterruptOnX(void *handle, LSM330_GYRO_XLIE_t *value);

/*******************************************************************************
* Register      : INT1_CFG
* Address       : 0X30
* Bit Group Name: XHIE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_XHIE_DISABLE 		 =0x00,
  	LSM330_GYRO_XHIE_ENABLE 		 =0x02,
} LSM330_GYRO_XHIE_t;

#define  	LSM330_GYRO_XHIE_MASK  	0x02
status_t  LSM330_GYRO_W_HighLevelInterruptOnX(void *handle, LSM330_GYRO_XHIE_t newValue);
status_t LSM330_GYRO_R_HighLevelInterruptOnX(void *handle, LSM330_GYRO_XHIE_t *value);

/*******************************************************************************
* Register      : INT1_CFG
* Address       : 0X30
* Bit Group Name: YLIE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_YLIE_DISABLE 		 =0x00,
  	LSM330_GYRO_YLIE_ENABLE 		 =0x04,
} LSM330_GYRO_YLIE_t;

#define  	LSM330_GYRO_YLIE_MASK  	0x04
status_t  LSM330_GYRO_W_LowLevelInterruptOnY(void *handle, LSM330_GYRO_YLIE_t newValue);
status_t LSM330_GYRO_R_LowLevelInterruptOnY(void *handle, LSM330_GYRO_YLIE_t *value);

/*******************************************************************************
* Register      : INT1_CFG
* Address       : 0X30
* Bit Group Name: YHIE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_YHIE_DISABLE 		 =0x00,
  	LSM330_GYRO_YHIE_ENABLE 		 =0x08,
} LSM330_GYRO_YHIE_t;

#define  	LSM330_GYRO_YHIE_MASK  	0x08
status_t  LSM330_GYRO_W_HighLevelInterruptOnY(void *handle, LSM330_GYRO_YHIE_t newValue);
status_t LSM330_GYRO_R_HighLevelInterruptOnY(void *handle, LSM330_GYRO_YHIE_t *value);

/*******************************************************************************
* Register      : INT1_CFG
* Address       : 0X30
* Bit Group Name: ZLIE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_ZLIE_DISABLE 		 =0x00,
  	LSM330_GYRO_ZLIE_ENABLE 		 =0x10,
} LSM330_GYRO_ZLIE_t;

#define  	LSM330_GYRO_ZLIE_MASK  	0x10
status_t  LSM330_GYRO_W_LowLevelInterruptOnZ(void *handle, LSM330_GYRO_ZLIE_t newValue);
status_t LSM330_GYRO_R_LowLevelInterruptOnZ(void *handle, LSM330_GYRO_ZLIE_t *value);

/*******************************************************************************
* Register      : INT1_CFG
* Address       : 0X30
* Bit Group Name: ZHIE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_ZHIE_DISABLE 		 =0x00,
  	LSM330_GYRO_ZHIE_ENABLE 		 =0x20,
} LSM330_GYRO_ZHIE_t;

#define  	LSM330_GYRO_ZHIE_MASK  	0x20
status_t  LSM330_GYRO_W_HighLevelInterruptOnZ(void *handle, LSM330_GYRO_ZHIE_t newValue);
status_t LSM330_GYRO_R_HighLevelInterruptOnZ(void *handle, LSM330_GYRO_ZHIE_t *value);

/*******************************************************************************
* Register      : INT1_CFG
* Address       : 0X30
* Bit Group Name: LIR
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_LIR_NOT_LATCHED 		 =0x00,
  	LSM330_GYRO_LIR_LATCHED 		 =0x40,
} LSM330_GYRO_LIR_t;

#define  	LSM330_GYRO_LIR_MASK  	0x40
status_t  LSM330_GYRO_W_InterruptRequestMode(void *handle, LSM330_GYRO_LIR_t newValue);
status_t LSM330_GYRO_R_InterruptRequestMode(void *handle, LSM330_GYRO_LIR_t *value);

/*******************************************************************************
* Register      : INT1_CFG
* Address       : 0X30
* Bit Group Name: AND_OR
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_AND_OR_OR 		 =0x00,
  	LSM330_GYRO_AND_OR_AND 		 =0x80,
} LSM330_GYRO_AND_OR_t;

#define  	LSM330_GYRO_AND_OR_MASK  	0x80
status_t  LSM330_GYRO_W_InterruptCombination(void *handle, LSM330_GYRO_AND_OR_t newValue);
status_t LSM330_GYRO_R_InterruptCombination(void *handle, LSM330_GYRO_AND_OR_t *value);

/*******************************************************************************
* Register      : INT1_SRC
* Address       : 0X31
* Bit Group Name: XL
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_XL_DOWN 		 =0x00,
  	LSM330_GYRO_XL_UP 		 =0x01,
} LSM330_GYRO_XL_t;

#define  	LSM330_GYRO_XL_MASK  	0x01
status_t LSM330_GYRO_R_InterruptLowFlagX(void *handle, LSM330_GYRO_XL_t *value);

/*******************************************************************************
* Register      : INT1_SRC
* Address       : 0X31
* Bit Group Name: XH
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_XH_DOWN 		 =0x00,
  	LSM330_GYRO_XH_UP 		 =0x02,
} LSM330_GYRO_XH_t;

#define  	LSM330_GYRO_XH_MASK  	0x02
status_t LSM330_GYRO_R_InterruptHighFlagX(void *handle, LSM330_GYRO_XH_t *value);

/*******************************************************************************
* Register      : INT1_SRC
* Address       : 0X31
* Bit Group Name: YL
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_YL_DOWN 		 =0x00,
  	LSM330_GYRO_YL_UP 		 =0x04,
} LSM330_GYRO_YL_t;

#define  	LSM330_GYRO_YL_MASK  	0x04
status_t LSM330_GYRO_R_InterruptLowFlagY(void *handle, LSM330_GYRO_YL_t *value);

/*******************************************************************************
* Register      : INT1_SRC
* Address       : 0X31
* Bit Group Name: YH
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_YH_DOWN 		 =0x00,
  	LSM330_GYRO_YH_UP 		 =0x08,
} LSM330_GYRO_YH_t;

#define  	LSM330_GYRO_YH_MASK  	0x08
status_t LSM330_GYRO_R_InterruptHighFlagY(void *handle, LSM330_GYRO_YH_t *value);

/*******************************************************************************
* Register      : INT1_SRC
* Address       : 0X31
* Bit Group Name: ZL
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_ZL_DOWN 		 =0x00,
  	LSM330_GYRO_ZL_UP 		 =0x10,
} LSM330_GYRO_ZL_t;

#define  	LSM330_GYRO_ZL_MASK  	0x10
status_t LSM330_GYRO_R_InterruptLowFlagZ(void *handle, LSM330_GYRO_ZL_t *value);

/*******************************************************************************
* Register      : INT1_SRC
* Address       : 0X31
* Bit Group Name: ZH
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_ZH_DOWN 		 =0x00,
  	LSM330_GYRO_ZH_UP 		 =0x20,
} LSM330_GYRO_ZH_t;

#define  	LSM330_GYRO_ZH_MASK  	0x20
status_t LSM330_GYRO_R_InterruptHighFlagZ(void *handle, LSM330_GYRO_ZH_t *value);

/*******************************************************************************
* Register      : INT1_SRC
* Address       : 0X31
* Bit Group Name: IA
* Permission    : RO
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_IA_DOWN 		 =0x00,
  	LSM330_GYRO_IA_UP 		 =0x40,
} LSM330_GYRO_IA_t;

#define  	LSM330_GYRO_IA_MASK  	0x40
status_t LSM330_GYRO_R_InterruptActiveFlag(void *handle, LSM330_GYRO_IA_t *value);

/*******************************************************************************
* Register      : INT1_DURATION
* Address       : 0X38
* Bit Group Name: D
* Permission    : RW
*******************************************************************************/
#define  	LSM330_GYRO_D_MASK  	0x7F
#define  	LSM330_GYRO_D_POSITION  	0
status_t  LSM330_GYRO_W_Duration(void *handle, u8_t newValue);
status_t LSM330_GYRO_R_Duration(void *handle, u8_t *value);

/*******************************************************************************
* Register      : INT1_DURATION
* Address       : 0X38
* Bit Group Name: WAIT
* Permission    : RW
*******************************************************************************/
typedef enum {
  	LSM330_GYRO_WAIT_DISABLE 		 =0x00,
  	LSM330_GYRO_WAIT_ENABLE 		 =0x80,
} LSM330_GYRO_WAIT_t;

#define  	LSM330_GYRO_WAIT_MASK  	0x80
status_t  LSM330_GYRO_W_Wait(void *handle, LSM330_GYRO_WAIT_t newValue);
status_t LSM330_GYRO_R_Wait(void *handle, LSM330_GYRO_WAIT_t *value);

/*******************************************************************************
* Register      : FIFO_SRC
* Address       : 0X2F
* Bit Group Name: FSS
* Permission    : RO
*******************************************************************************/
#define  	LSM330_GYRO_FSS_MASK  	0x3F
#define  	LSM330_GYRO_FSS_POSITION  	0
status_t LSM330_GYRO_R_DataStoredFIFO(void *handle, u8_t *value);

/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : InterruptThreshold
* Permission    : RW 
*******************************************************************************/
status_t LSM330_GYRO_Set_InterruptThreshold(void *handle, u8_t *buff);
status_t LSM330_GYRO_Get_InterruptThreshold(void *handle, u8_t *buff); 

/************** Utility  *******************/

/*******************************************************************************
  * Register      : Generic - All
* Address       : Generic - All
* Bit Group Name: None
* Permission    : W
*******************************************************************************/
void LSM330_GYRO_SwapHighLowByte(u8_t *bufferToSwap, u8_t numberOfByte, u8_t dimension); 

#ifdef __cplusplus
  }
#endif

#endif

