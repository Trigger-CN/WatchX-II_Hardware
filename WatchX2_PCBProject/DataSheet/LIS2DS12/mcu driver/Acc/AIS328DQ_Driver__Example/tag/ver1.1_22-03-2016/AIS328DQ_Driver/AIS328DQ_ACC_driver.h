/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
* File Name          : AIS328DQ_ACC_driver.h
* Author             : MEMS Application Team
* Version            : v1.1
* Date               : 22 Mar 2016  
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
#ifndef __AIS328DQ_ACC_DRIVER__H
#define __AIS328DQ_ACC_DRIVER__H

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

#define AIS328DQ_ACC_I2C_ADDRESS         0x32

/************** Who am I  *******************/

#define AIS328DQ_ACC_WHO_AM_I_VAL         0x32

/************** Device Register  *******************/
#define AIS328DQ_ACC_WHO_AM_I  	0x0F
#define AIS328DQ_ACC_CTRL_REG1  	0x20
#define AIS328DQ_ACC_CTRL_REG2  	0x21
#define AIS328DQ_ACC_CTRL_REG3  	0x22
#define AIS328DQ_ACC_CTRL_REG4  	0x23
#define AIS328DQ_ACC_CTRL_REG5  	0x24
#define AIS328DQ_ACC_HP_FILTER_RESET  	0x25
#define AIS328DQ_ACC_REFERENCE  	0x26
#define AIS328DQ_ACC_STATUS_REG  	0x27
#define AIS328DQ_ACC_OUT_X_L  	0x28
#define AIS328DQ_ACC_OUT_X_H  	0x29
#define AIS328DQ_ACC_OUT_Y_L  	0x2A
#define AIS328DQ_ACC_OUT_Y_H  	0x2B
#define AIS328DQ_ACC_OUT_Z_L  	0x2C
#define AIS328DQ_ACC_OUT_Z_H  	0x2D
#define AIS328DQ_ACC_INT1_CFG  	0x30
#define AIS328DQ_ACC_INT1_SOURCE  	0x31
#define AIS328DQ_ACC_INT1_THS  	0x32
#define AIS328DQ_ACC_INT1_DURATION  	0x33
#define AIS328DQ_ACC_INT2_CFG  	0x30
#define AIS328DQ_ACC_INT2_SOURCE  	0x31
#define AIS328DQ_ACC_INT2_THS  	0x32
#define AIS328DQ_ACC_INT2_DURATION  	0x33

/************** Generic Function  *******************/

/*******************************************************************************
* Register      : Generic - All
* Address       : Generic - All
* Bit Group Name: None
* Permission    : W
*******************************************************************************/
status_t AIS328DQ_ACC_WriteReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len);

/*******************************************************************************
* Register      : Generic - All
* Address       : Generic - All
* Bit Group Name: None
* Permission    : R
*******************************************************************************/
status_t AIS328DQ_ACC_ReadReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len);

/**************** Base Function  *******************/

/*******************************************************************************
* Register      : WHO_AM_I
* Address       : 0F
* Bit Group Name: WHO_AM_I_BIT
* Permission    : This value can be only OR or RW 
*******************************************************************************/
#define  	AIS328DQ_ACC_WHO_AM_I_BIT_MASK  	0xFF
#define  	AIS328DQ_ACC_WHO_AM_I_BIT_POSITION  	0
status_t AIS328DQ_ACC_R_WHO_AM_I_BIT_bits(void *handle, u8_t *value);

/*******************************************************************************
* Register      : CTRL_REG4
* Address       : 23
* Bit Group Name: BDU
* Permission    : RW
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_BDU_ENABLE 		 =0x00,
  	AIS328DQ_ACC_BDU_DISABLE 		 =0x80,
} AIS328DQ_ACC_BDU_t;

#define  	AIS328DQ_ACC_BDU_MASK  	0x80
status_t  AIS328DQ_ACC_W_CTRL_REG4_BDU(void *handle, AIS328DQ_ACC_BDU_t newValue);
status_t AIS328DQ_ACC_R_CTRL_REG4_BDU(void *handle, AIS328DQ_ACC_BDU_t *value);

/*******************************************************************************
* Register      : CTRL_REG4
* Address       : 23
* Bit Group Name: FS
* Permission    : RW
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_FS_2G 		 =0x00,
  	AIS328DQ_ACC_FS_4G 		 =0x10,
  	AIS328DQ_ACC_FS_8G 		 =0x20,
} AIS328DQ_ACC_FS_t;

#define  	AIS328DQ_ACC_FS_MASK  	0x30
status_t  AIS328DQ_ACC_W_CTRL_REG4_FS(void *handle, AIS328DQ_ACC_FS_t newValue);
status_t AIS328DQ_ACC_R_CTRL_REG4_FS(void *handle, AIS328DQ_ACC_FS_t *value);

/*******************************************************************************
* Register      : CTRL_REG1
* Address       : 20
* Bit Group Name: DR
* Permission    : RW
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_DR_50HZ 		 =0x00,
  	AIS328DQ_ACC_DR_100HZ 		 =0x08,
  	AIS328DQ_ACC_DR_400HZ 		 =0x10,
  	AIS328DQ_ACC_DR_1000HZ 		 =0x18,
} AIS328DQ_ACC_DR_t;

#define  	AIS328DQ_ACC_DR_MASK  	0x18
status_t  AIS328DQ_ACC_W_CTRL_REG1_ODR(void *handle, AIS328DQ_ACC_DR_t newValue);
status_t AIS328DQ_ACC_R_CTRL_REG1_ODR(void *handle, AIS328DQ_ACC_DR_t *value);

/**************** Advanced Function  *******************/

/*******************************************************************************
* Register      : CTRL_REG1
* Address       : 20
* Bit Group Name: XEN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_XEN_DISABLE 		 =0x00,
  	AIS328DQ_ACC_XEN_ENABLE 		 =0x01,
} AIS328DQ_ACC_XEN_t;

#define  	AIS328DQ_ACC_XEN_MASK  	0x01
status_t  AIS328DQ_ACC_W_CTRL_REG1_XEN(void *handle, AIS328DQ_ACC_XEN_t newValue);
status_t AIS328DQ_ACC_R_CTRL_REG1_XEN(void *handle, AIS328DQ_ACC_XEN_t *value);

/*******************************************************************************
* Register      : CTRL_REG1
* Address       : 20
* Bit Group Name: YEN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_YEN_DISABLE 		 =0x00,
  	AIS328DQ_ACC_YEN_ENABLE 		 =0x02,
} AIS328DQ_ACC_YEN_t;

#define  	AIS328DQ_ACC_YEN_MASK  	0x02
status_t  AIS328DQ_ACC_W_CTRL_REG1_YEN(void *handle, AIS328DQ_ACC_YEN_t newValue);
status_t AIS328DQ_ACC_R_CTRL_REG1_YEN(void *handle, AIS328DQ_ACC_YEN_t *value);

/*******************************************************************************
* Register      : CTRL_REG1
* Address       : 20
* Bit Group Name: ZEN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_ZEN_DISABLE 		 =0x00,
  	AIS328DQ_ACC_ZEN_ENABLE 		 =0x04,
} AIS328DQ_ACC_ZEN_t;

#define  	AIS328DQ_ACC_ZEN_MASK  	0x04
status_t  AIS328DQ_ACC_W_CTRL_REG1_ZEN(void *handle, AIS328DQ_ACC_ZEN_t newValue);
status_t AIS328DQ_ACC_R_CTRL_REG1_ZEN(void *handle, AIS328DQ_ACC_ZEN_t *value);

/*******************************************************************************
* Register      : CTRL_REG1
* Address       : 20
* Bit Group Name: PM
* Permission    : RW
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_PM_POWER_DOWN 		 =0x00,
  	AIS328DQ_ACC_PM_NORMAL_MODE 		 =0x20,
  	AIS328DQ_ACC_PM_LOW_POWER_0_5Hz          =0x40,
  	AIS328DQ_ACC_PM_LOW_POWER_1Hz 		 =0x60,
  	AIS328DQ_ACC_PM_LOW_POWER_2Hz 		 =0x80,
  	AIS328DQ_ACC_PM_LOW_POWER_5Hz 		 =0xA0,
  	AIS328DQ_ACC_PM_LOW_POWER_10Hz 		 =0xC0,
} AIS328DQ_ACC_PM_t;

#define  	AIS328DQ_ACC_PM_MASK  	0xE0
status_t  AIS328DQ_ACC_W_CTRL_REG1_PM(void *handle, AIS328DQ_ACC_PM_t newValue);
status_t AIS328DQ_ACC_R_CTRL_REG1_PM(void *handle, AIS328DQ_ACC_PM_t *value);

/*******************************************************************************
* Register      : CTRL_REG2
* Address       : 21
* Bit Group Name: HPCF
* Permission    : RW
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_HPCF_HPC_8 		 =0x00,
  	AIS328DQ_ACC_HPCF_HPC_16 		 =0x01,
  	AIS328DQ_ACC_HPCF_HPC_32 		 =0x02,
  	AIS328DQ_ACC_HPCF_HPC_64 		 =0x03,
} AIS328DQ_ACC_HPCF_t;

#define  	AIS328DQ_ACC_HPCF_MASK  	0x03
status_t  AIS328DQ_ACC_W_CTRL_REG2_HPCF(void *handle, AIS328DQ_ACC_HPCF_t newValue);
status_t AIS328DQ_ACC_R_CTRL_REG2_HPCF(void *handle, AIS328DQ_ACC_HPCF_t *value);

/*******************************************************************************
* Register      : CTRL_REG2
* Address       : 21
* Bit Group Name: HPEN1
* Permission    : RW
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_HPEN1_DISABLE 		 =0x00,
  	AIS328DQ_ACC_HPEN1_ENABLE 		 =0x04,
} AIS328DQ_ACC_HPEN1_t;

#define  	AIS328DQ_ACC_HPEN1_MASK  	0x04
status_t  AIS328DQ_ACC_W_CTRL_REG2_HPEN1(void *handle, AIS328DQ_ACC_HPEN1_t newValue);
status_t AIS328DQ_ACC_R_CTRL_REG2_HPEN1(void *handle, AIS328DQ_ACC_HPEN1_t *value);

/*******************************************************************************
* Register      : CTRL_REG2
* Address       : 21
* Bit Group Name: HPEN2
* Permission    : RW
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_HPEN2_DISABLE 		 =0x00,
  	AIS328DQ_ACC_HPEN2_ENABLE 		 =0x08,
} AIS328DQ_ACC_HPEN2_t;

#define  	AIS328DQ_ACC_HPEN2_MASK  	0x08
status_t  AIS328DQ_ACC_W_CTRL_REG2_HPEN2(void *handle, AIS328DQ_ACC_HPEN2_t newValue);
status_t AIS328DQ_ACC_R_CTRL_REG2_HPEN2(void *handle, AIS328DQ_ACC_HPEN2_t *value);

/*******************************************************************************
* Register      : CTRL_REG2
* Address       : 21
* Bit Group Name: FDS
* Permission    : RW
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_FDS_DISABLE 		 =0x00,
  	AIS328DQ_ACC_FDS_ENABLE 		 =0x10,
} AIS328DQ_ACC_FDS_t;

#define  	AIS328DQ_ACC_FDS_MASK  	0x10
status_t  AIS328DQ_ACC_W_CTRL_REG2_FDS(void *handle, AIS328DQ_ACC_FDS_t newValue);
status_t AIS328DQ_ACC_R_CTRL_REG2_FDS(void *handle, AIS328DQ_ACC_FDS_t *value);

/*******************************************************************************
* Register      : CTRL_REG2
* Address       : 21
* Bit Group Name: HPM
* Permission    : RW
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_HPM_NORMAL0 		 =0x00,
  	AIS328DQ_ACC_HPM_REF_SIGNAL 		 =0x20,
  	AIS328DQ_ACC_HPM_NORMAL1 		 =0x40,
} AIS328DQ_ACC_HPM_t;

#define  	AIS328DQ_ACC_HPM_MASK  	0x60
status_t  AIS328DQ_ACC_W_CTRL_REG2_HPM(void *handle, AIS328DQ_ACC_HPM_t newValue);
status_t AIS328DQ_ACC_R_CTRL_REG2_HPM(void *handle, AIS328DQ_ACC_HPM_t *value);

/*******************************************************************************
* Register      : CTRL_REG2
* Address       : 21
* Bit Group Name: BOOT
* Permission    : RW
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_BOOT_NORMAL_MODE 		 =0x00,
  	AIS328DQ_ACC_BOOT_REBOOT_MEMORY 		 =0x80,
} AIS328DQ_ACC_BOOT_t;

#define  	AIS328DQ_ACC_BOOT_MASK  	0x80
status_t  AIS328DQ_ACC_W_CTRL_REG2_BOOT(void *handle, AIS328DQ_ACC_BOOT_t newValue);
status_t AIS328DQ_ACC_R_CTRL_REG2_BOOT(void *handle, AIS328DQ_ACC_BOOT_t *value);

/*******************************************************************************
* Register      : CTRL_REG3
* Address       : 22
* Bit Group Name: I1_CFG
* Permission    : RW
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_I1_CFG_INT1_SRC 		 =0x00,
  	AIS328DQ_ACC_I1_CFG_INT1_OR_INT2 		 =0x01,
  	AIS328DQ_ACC_I1_CFG_DATA_READY 		 =0x02,
  	AIS328DQ_ACC_I1_CFG_BOOT_RUN 		 =0x03,
} AIS328DQ_ACC_I1_CFG_t;

#define  	AIS328DQ_ACC_I1_CFG_MASK  	0x03
status_t  AIS328DQ_ACC_W_CTRL_REG3_I1_CFG(void *handle, AIS328DQ_ACC_I1_CFG_t newValue);
status_t AIS328DQ_ACC_R_CTRL_REG3_I1_CFG(void *handle, AIS328DQ_ACC_I1_CFG_t *value);

/*******************************************************************************
* Register      : CTRL_REG3
* Address       : 22
* Bit Group Name: LIR1
* Permission    : RW
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_LIR1_NOT_LATCHED 		 =0x00,
  	AIS328DQ_ACC_LIR1_LATCHED 		 =0x04,
} AIS328DQ_ACC_LIR1_t;

#define  	AIS328DQ_ACC_LIR1_MASK  	0x04
status_t  AIS328DQ_ACC_W_CTRL_REG3_LIR1(void *handle, AIS328DQ_ACC_LIR1_t newValue);
status_t AIS328DQ_ACC_R_CTRL_REG3_LIR1(void *handle, AIS328DQ_ACC_LIR1_t *value);

/*******************************************************************************
* Register      : CTRL_REG3
* Address       : 22
* Bit Group Name: I2_CFG
* Permission    : RW
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_I2_CFG_INT1_SRC 		 =0x00,
  	AIS328DQ_ACC_I2_CFG_INT1_OR_INT2 		 =0x08,
  	AIS328DQ_ACC_I2_CFG_DATA_READY 		 =0x10,
  	AIS328DQ_ACC_I2_CFG_BOOT_RUN 		 =0x18,
} AIS328DQ_ACC_I2_CFG_t;

#define  	AIS328DQ_ACC_I2_CFG_MASK  	0x18
status_t  AIS328DQ_ACC_W_CTRL_REG3_I2_CFG(void *handle, AIS328DQ_ACC_I2_CFG_t newValue);
status_t AIS328DQ_ACC_R_CTRL_REG3_I2_CFG(void *handle, AIS328DQ_ACC_I2_CFG_t *value);

/*******************************************************************************
* Register      : CTRL_REG3
* Address       : 22
* Bit Group Name: LIR2
* Permission    : RW
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_LIR2_NOT_LATCHED 		 =0x00,
  	AIS328DQ_ACC_LIR2_LATCHED 		 =0x20,
} AIS328DQ_ACC_LIR2_t;

#define  	AIS328DQ_ACC_LIR2_MASK  	0x20
status_t  AIS328DQ_ACC_W_CTRL_REG3_LIR2(void *handle, AIS328DQ_ACC_LIR2_t newValue);
status_t AIS328DQ_ACC_R_CTRL_REG3_LIR2(void *handle, AIS328DQ_ACC_LIR2_t *value);

/*******************************************************************************
* Register      : CTRL_REG3
* Address       : 22
* Bit Group Name: PP_OD
* Permission    : RW
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_PP_OD_PUSH_PULL 		 =0x00,
  	AIS328DQ_ACC_PP_OD_OPEN_DRAIN 		 =0x40,
} AIS328DQ_ACC_PP_OD_t;

#define  	AIS328DQ_ACC_PP_OD_MASK  	0x40
status_t  AIS328DQ_ACC_W_CTRL_REG3_PPOD(void *handle, AIS328DQ_ACC_PP_OD_t newValue);
status_t AIS328DQ_ACC_R_CTRL_REG3_PPOD(void *handle, AIS328DQ_ACC_PP_OD_t *value);

/*******************************************************************************
* Register      : CTRL_REG3
* Address       : 22
* Bit Group Name: IHL
* Permission    : RW
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_IHL_ACTIVE_HIGH 		 =0x00,
  	AIS328DQ_ACC_IHL_ACTIVE_LOW 		 =0x80,
} AIS328DQ_ACC_IHL_t;

#define  	AIS328DQ_ACC_IHL_MASK  	0x80
status_t  AIS328DQ_ACC_W_CTRL_REG3_IHL(void *handle, AIS328DQ_ACC_IHL_t newValue);
status_t AIS328DQ_ACC_R_CTRL_REG3_IHL(void *handle, AIS328DQ_ACC_IHL_t *value);

/*******************************************************************************
* Register      : CTRL_REG4
* Address       : 23
* Bit Group Name: SIM
* Permission    : RW
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_SIM_4_WIRE 		 =0x00,
  	AIS328DQ_ACC_SIM_3_WIRE 		 =0x01,
} AIS328DQ_ACC_SIM_t;

#define  	AIS328DQ_ACC_SIM_MASK  	0x01
status_t  AIS328DQ_ACC_W_CTRL_REG4_SIM(void *handle, AIS328DQ_ACC_SIM_t newValue);
status_t AIS328DQ_ACC_R_CTRL_REG4_SIM(void *handle, AIS328DQ_ACC_SIM_t *value);

/*******************************************************************************
* Register      : CTRL_REG4
* Address       : 23
* Bit Group Name: ST
* Permission    : RW
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_ST_DISABLE 		 =0x00,
  	AIS328DQ_ACC_ST_ENABLE 		 =0x02,
} AIS328DQ_ACC_ST_t;

#define  	AIS328DQ_ACC_ST_MASK  	0x02
status_t  AIS328DQ_ACC_W_CTRL_REG4_ST(void *handle, AIS328DQ_ACC_ST_t newValue);
status_t AIS328DQ_ACC_R_CTRL_REG4_ST(void *handle, AIS328DQ_ACC_ST_t *value);

/*******************************************************************************
* Register      : CTRL_REG4
* Address       : 23
* Bit Group Name: STSIGN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_STSIGN_PLUS 		 =0x00,
  	AIS328DQ_ACC_STSIGN_MINUS 		 =0x08,
} AIS328DQ_ACC_STSIGN_t;

#define  	AIS328DQ_ACC_STSIGN_MASK  	0x08
status_t  AIS328DQ_ACC_W_CTRL_REG4_STSIGN(void *handle, AIS328DQ_ACC_STSIGN_t newValue);
status_t AIS328DQ_ACC_R_CTRL_REG4_STSIGN(void *handle, AIS328DQ_ACC_STSIGN_t *value);

/*******************************************************************************
* Register      : CTRL_REG4
* Address       : 23
* Bit Group Name: BLE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_BLE_LITTLE_ENDIAN 		 =0x00,
  	AIS328DQ_ACC_BLE_BIG_ENDIAN 		 =0x40,
} AIS328DQ_ACC_BLE_t;

#define  	AIS328DQ_ACC_BLE_MASK  	0x40
status_t  AIS328DQ_ACC_W_CTRL_REG4_BLE(void *handle, AIS328DQ_ACC_BLE_t newValue);
status_t AIS328DQ_ACC_R_CTRL_REG4_BLE(void *handle, AIS328DQ_ACC_BLE_t *value);

/*******************************************************************************
* Register      : CTRL_REG5
* Address       : 24
* Bit Group Name: TURNON
* Permission    : RW
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_TURNON_SLEEP_TO_WAKE_DIS 		 =0x00,
  	AIS328DQ_ACC_TURNON_SLEEP_TO_WAKE_EN 		 =0x03,
} AIS328DQ_ACC_TURNON_t;

#define  	AIS328DQ_ACC_TURNON_MASK  	0x03
status_t  AIS328DQ_ACC_W_CTRL_REG5_TURNON(void *handle, AIS328DQ_ACC_TURNON_t newValue);
status_t AIS328DQ_ACC_R_CTRL_REG5_TURNON(void *handle, AIS328DQ_ACC_TURNON_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 27
* Bit Group Name: XDA
* Permission    : This value can be only OR or RW 
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_XDA_NOT_AVAIL 		 =0x00,
  	AIS328DQ_ACC_XDA_AVAIL 		 =0x01,
} AIS328DQ_ACC_XDA_t;

#define  	AIS328DQ_ACC_XDA_MASK  	0x01
status_t AIS328DQ_ACC_R_STATUS_XDA(void *handle, AIS328DQ_ACC_XDA_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 27
* Bit Group Name: YDA
* Permission    : This value can be only OR or RW 
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_YDA_NOT_AVAIL 		 =0x00,
  	AIS328DQ_ACC_YDA_AVAIL 		 =0x02,
} AIS328DQ_ACC_YDA_t;

#define  	AIS328DQ_ACC_YDA_MASK  	0x02
status_t AIS328DQ_ACC_R_STATUS_YDA(void *handle, AIS328DQ_ACC_YDA_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 27
* Bit Group Name: ZDA
* Permission    : This value can be only OR or RW 
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_ZDA_NOT_AVAIL 		 =0x00,
  	AIS328DQ_ACC_ZDA_AVAIL 		 =0x04,
} AIS328DQ_ACC_ZDA_t;

#define  	AIS328DQ_ACC_ZDA_MASK  	0x04
status_t AIS328DQ_ACC_R_STATUS_ZDA(void *handle, AIS328DQ_ACC_ZDA_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 27
* Bit Group Name: ZYXDA
* Permission    : This value can be only OR or RW 
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_ZYXDA_NOT_AVAIL 		 =0x00,
  	AIS328DQ_ACC_ZYXDA_AVAIL 		 =0x08,
} AIS328DQ_ACC_ZYXDA_t;

#define  	AIS328DQ_ACC_ZYXDA_MASK  	0x08
status_t AIS328DQ_ACC_R_STATUS_ZYXDA(void *handle, AIS328DQ_ACC_ZYXDA_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 27
* Bit Group Name: XOR
* Permission    : This value can be only OR or RW 
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_XOR_NO_OVERRUN 		 =0x00,
  	AIS328DQ_ACC_XOR_OVERRUN 		 =0x10,
} AIS328DQ_ACC_XOR_t;

#define  	AIS328DQ_ACC_XOR_MASK  	0x10
status_t AIS328DQ_ACC_R_STATUS_XOR(void *handle, AIS328DQ_ACC_XOR_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 27
* Bit Group Name: YOR
* Permission    : This value can be only OR or RW 
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_YOR_NO_OVERRUN 		 =0x00,
  	AIS328DQ_ACC_YOR_OVERRUN 		 =0x20,
} AIS328DQ_ACC_YOR_t;

#define  	AIS328DQ_ACC_YOR_MASK  	0x20
status_t AIS328DQ_ACC_R_STATUS_YOR(void *handle, AIS328DQ_ACC_YOR_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 27
* Bit Group Name: ZOR
* Permission    : This value can be only OR or RW 
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_ZOR_NO_OVERRUN 		 =0x00,
  	AIS328DQ_ACC_ZOR_OVERRUN 		 =0x40,
} AIS328DQ_ACC_ZOR_t;

#define  	AIS328DQ_ACC_ZOR_MASK  	0x40
status_t AIS328DQ_ACC_R_STATUS_ZOR(void *handle, AIS328DQ_ACC_ZOR_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 27
* Bit Group Name: ZYXOR
* Permission    : This value can be only OR or RW 
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_ZYXOR_NO_OVERRUN 		 =0x00,
  	AIS328DQ_ACC_ZYXOR_OVERRUN 		 =0x80,
} AIS328DQ_ACC_ZYXOR_t;

#define  	AIS328DQ_ACC_ZYXOR_MASK  	0x80
status_t AIS328DQ_ACC_R_STATUS_ZYXOR(void *handle, AIS328DQ_ACC_ZYXOR_t *value);

/*******************************************************************************
* Register      : INT1_CFG
* Address       : 30
* Bit Group Name: XLIE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_INT1_XLIE_DISABLE 		 =0x00,
  	AIS328DQ_ACC_INT1_XLIE_ENABLE 		 =0x01,
} AIS328DQ_ACC_INT1_XLIE_t;

#define  	AIS328DQ_ACC_INT1_XLIE_MASK  	0x01
status_t  AIS328DQ_ACC_W_INT1_CFG_XLIE(void *handle, AIS328DQ_ACC_INT1_XLIE_t newValue);
status_t AIS328DQ_ACC_R_INT1_CFG_XLIE(void *handle, AIS328DQ_ACC_INT1_XLIE_t *value);

/*******************************************************************************
* Register      : INT1_CFG
* Address       : 30
* Bit Group Name: XHIE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_INT1_XHIE_DISABLE 		 =0x00,
  	AIS328DQ_ACC_INT1_XHIE_ENABLE 		 =0x02,
} AIS328DQ_ACC_INT1_XHIE_t;

#define  	AIS328DQ_ACC_INT1_XHIE_MASK  	0x02
status_t  AIS328DQ_ACC_W_INT1_CFG_XHIE(void *handle, AIS328DQ_ACC_INT1_XHIE_t newValue);
status_t AIS328DQ_ACC_R_INT1_CFG_XHIE(void *handle, AIS328DQ_ACC_INT1_XHIE_t *value);

/*******************************************************************************
* Register      : INT1_CFG
* Address       : 30
* Bit Group Name: YLIE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_INT1_YLIE_DISABLE 		 =0x00,
  	AIS328DQ_ACC_INT1_YLIE_ENABLE 		 =0x04,
} AIS328DQ_ACC_INT1_YLIE_t;

#define  	AIS328DQ_ACC_INT1_YLIE_MASK  	0x04
status_t  AIS328DQ_ACC_W_INT1_CFG_YLIE(void *handle, AIS328DQ_ACC_INT1_YLIE_t newValue);
status_t AIS328DQ_ACC_R_INT1_CFG_YLIE(void *handle, AIS328DQ_ACC_INT1_YLIE_t *value);

/*******************************************************************************
* Register      : INT1_CFG
* Address       : 30
* Bit Group Name: YHIE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_INT1_YHIE_DISABLE 		 =0x00,
  	AIS328DQ_ACC_INT1_YHIE_ENABLE 		 =0x08,
} AIS328DQ_ACC_INT1_YHIE_t;

#define  	AIS328DQ_ACC_INT1_YHIE_MASK  	0x08
status_t  AIS328DQ_ACC_W_INT1_CFG_YHIE(void *handle, AIS328DQ_ACC_INT1_YHIE_t newValue);
status_t AIS328DQ_ACC_R_INT1_CFG_YHIE(void *handle, AIS328DQ_ACC_INT1_YHIE_t *value);

/*******************************************************************************
* Register      : INT1_CFG
* Address       : 30
* Bit Group Name: ZLIE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_INT1_ZLIE_DISABLE 		 =0x00,
  	AIS328DQ_ACC_INT1_ZLIE_ENABLE 		 =0x10,
} AIS328DQ_ACC_INT1_ZLIE_t;

#define  	AIS328DQ_ACC_INT1_ZLIE_MASK  	0x10
status_t  AIS328DQ_ACC_W_INT1_CFG_ZLIE(void *handle, AIS328DQ_ACC_INT1_ZLIE_t newValue);
status_t AIS328DQ_ACC_R_INT1_CFG_ZLIE(void *handle, AIS328DQ_ACC_INT1_ZLIE_t *value);

/*******************************************************************************
* Register      : INT1_CFG
* Address       : 30
* Bit Group Name: ZHIE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_INT1_ZHIE_DISABLE 		 =0x00,
  	AIS328DQ_ACC_INT1_ZHIE_ENABLE 		 =0x20,
} AIS328DQ_ACC_INT1_ZHIE_t;

#define  	AIS328DQ_ACC_INT1_ZHIE_MASK  	0x20
status_t  AIS328DQ_ACC_W_INT1_CFG_ZHIE(void *handle, AIS328DQ_ACC_INT1_ZHIE_t newValue);
status_t AIS328DQ_ACC_R_INT1_CFG_ZHIE(void *handle, AIS328DQ_ACC_INT1_ZHIE_t *value);

/*******************************************************************************
* Register      : INT1_CFG
* Address       : 30
* Bit Group Name: 6D
* Permission    : RW
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_INT1_6D_DISABLE 		 =0x00,
  	AIS328DQ_ACC_INT1_6D_ENABLE 		 =0x40,
} AIS328DQ_ACC_INT1_6D_t;

#define  	AIS328DQ_ACC_INT1_6D_MASK  	0x40
status_t  AIS328DQ_ACC_W_INT1_6D(void *handle, AIS328DQ_ACC_INT1_6D_t newValue);
status_t AIS328DQ_ACC_R_INT1_6D(void *handle, AIS328DQ_ACC_INT1_6D_t *value);

/*******************************************************************************
* Register      : INT1_CFG
* Address       : 30
* Bit Group Name: AOI
* Permission    : RW
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_INT1_AOI_DISABLE 		 =0x00,
  	AIS328DQ_ACC_INT1_AOI_ENABLE 		 =0x80,
} AIS328DQ_ACC_INT1_AOI_t;

#define  	AIS328DQ_ACC_INT1_AOI_MASK  	0x80
status_t  AIS328DQ_ACC_W_INT1_CFG_AOI(void *handle, AIS328DQ_ACC_INT1_AOI_t newValue);
status_t AIS328DQ_ACC_R_INT1_CFG_AOI(void *handle, AIS328DQ_ACC_INT1_AOI_t *value);

/*******************************************************************************
* Register      : INT1_SOURCE
* Address       : 31
* Bit Group Name: XL
* Permission    : This value can be only OR or RW 
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_INT1_XL_NO_EVENT 		 =0x00,
  	AIS328DQ_ACC_INT1_XL_EVENT 		 =0x01,
} AIS328DQ_ACC_INT1_XL_t;

#define  	AIS328DQ_ACC_INT1_XL_MASK  	0x01
status_t AIS328DQ_ACC_R_INT1_SOURCE_XL(void *handle, AIS328DQ_ACC_INT1_XL_t *value);

/*******************************************************************************
* Register      : INT1_SOURCE
* Address       : 31
* Bit Group Name: XH
* Permission    : This value can be only OR or RW 
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_INT1_XH_NO_EVENT 		 =0x00,
  	AIS328DQ_ACC_INT1_XH_EVENT 		 =0x02,
} AIS328DQ_ACC_INT1_XH_t;

#define  	AIS328DQ_ACC_INT1_XH_MASK  	0x02
status_t AIS328DQ_ACC_R_INT1_SOURCE_XH(void *handle, AIS328DQ_ACC_INT1_XH_t *value);

/*******************************************************************************
* Register      : INT1_SOURCE
* Address       : 31
* Bit Group Name: YL
* Permission    : This value can be only OR or RW 
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_INT1_YL_NO_EVENT 		 =0x00,
  	AIS328DQ_ACC_INT1_YL_EVENT 		 =0x04,
} AIS328DQ_ACC_INT1_YL_t;

#define  	AIS328DQ_ACC_INT1_YL_MASK  	0x04
status_t AIS328DQ_ACC_R_INT1_SOURCE_YL(void *handle, AIS328DQ_ACC_INT1_YL_t *value);

/*******************************************************************************
* Register      : INT1_SOURCE
* Address       : 31
* Bit Group Name: YH
* Permission    : This value can be only OR or RW 
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_INT1_YH_NO_EVENT 		 =0x00,
  	AIS328DQ_ACC_INT1_YH_EVENT 		 =0x08,
} AIS328DQ_ACC_INT1_YH_t;

#define  	AIS328DQ_ACC_INT1_YH_MASK  	0x08
status_t AIS328DQ_ACC_R_INT1_SOURCE_YH(void *handle, AIS328DQ_ACC_INT1_YH_t *value);

/*******************************************************************************
* Register      : INT1_SOURCE
* Address       : 31
* Bit Group Name: ZL
* Permission    : This value can be only OR or RW 
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_INT1_ZL_NO_EVENT 		 =0x00,
  	AIS328DQ_ACC_INT1_ZL_EVENT 		 =0x10,
} AIS328DQ_ACC_INT1_ZL_t;

#define  	AIS328DQ_ACC_INT1_ZL_MASK  	0x10
status_t AIS328DQ_ACC_R_INT1_SOURCE_ZL(void *handle, AIS328DQ_ACC_INT1_ZL_t *value);

/*******************************************************************************
* Register      : INT1_SOURCE
* Address       : 31
* Bit Group Name: ZH
* Permission    : This value can be only OR or RW 
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_INT1_ZH_NO_EVENT 		 =0x00,
  	AIS328DQ_ACC_INT1_ZH_EVENT 		 =0x20,
} AIS328DQ_ACC_INT1_ZH_t;

#define  	AIS328DQ_ACC_INT1_ZH_MASK  	0x20
status_t AIS328DQ_ACC_R_INT1_SOURCE_ZH(void *handle, AIS328DQ_ACC_INT1_ZH_t *value);

/*******************************************************************************
* Register      : INT1_SOURCE
* Address       : 31
* Bit Group Name: IA
* Permission    : This value can be only OR or RW 
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_INT1_IA_NO_EVENT 		 =0x00,
  	AIS328DQ_ACC_INT1_IA_EVENT 		 =0x40,
} AIS328DQ_ACC_INT1_IA_t;

#define  	AIS328DQ_ACC_INT1_IA_MASK  	0x40
status_t AIS328DQ_ACC_R_INT1_SOURCE_IA(void *handle, AIS328DQ_ACC_INT1_IA_t *value);

/*******************************************************************************
* Register      : INT1_THS
* Address       : 32
* Bit Group Name: THS
* Permission    : RW
*******************************************************************************/
#define  	AIS328DQ_ACC_INT1_THS_MASK  	0x7F
#define  	AIS328DQ_ACC_INT1_THS_POSITION  	0
status_t  AIS328DQ_ACC_W_INT1_THS(void *handle, u8_t newValue);
status_t AIS328DQ_ACC_R_INT1_THS(void *handle, u8_t *value);

/*******************************************************************************
* Register      : INT1_DURATION
* Address       : 33
* Bit Group Name: D
* Permission    : RW
*******************************************************************************/
#define  	AIS328DQ_ACC_INT1_D_MASK  	0x7F
#define  	AIS328DQ_ACC_INT1_D_POSITION  	0
status_t  AIS328DQ_ACC_W_INT1_DURATION(void *handle, u8_t newValue);
status_t AIS328DQ_ACC_R_INT1_DURATION(void *handle, u8_t *value);

/*******************************************************************************
* Register      : INT2_CFG
* Address       : 30
* Bit Group Name: XLIE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_INT2_XLIE_DISABLE 		 =0x00,
  	AIS328DQ_ACC_INT2_XLIE_ENABLE 		 =0x01,
} AIS328DQ_ACC_INT2_XLIE_t;

#define  	AIS328DQ_ACC_INT2_XLIE_MASK  	0x01
status_t  AIS328DQ_ACC_W_INT2_CFG_XLIE(void *handle, AIS328DQ_ACC_INT2_XLIE_t newValue);
status_t AIS328DQ_ACC_R_INT2_CFG_XLIE(void *handle, AIS328DQ_ACC_INT2_XLIE_t *value);

/*******************************************************************************
* Register      : INT2_CFG
* Address       : 30
* Bit Group Name: XHIE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_INT2_XHIE_DISABLE 		 =0x00,
  	AIS328DQ_ACC_INT2_XHIE_ENABLE 		 =0x02,
} AIS328DQ_ACC_INT2_XHIE_t;

#define  	AIS328DQ_ACC_INT2_XHIE_MASK  	0x02
status_t  AIS328DQ_ACC_W_INT2_CFG_XHIE(void *handle, AIS328DQ_ACC_INT2_XHIE_t newValue);
status_t AIS328DQ_ACC_R_INT2_CFG_XHIE(void *handle, AIS328DQ_ACC_INT2_XHIE_t *value);

/*******************************************************************************
* Register      : INT2_CFG
* Address       : 30
* Bit Group Name: YLIE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_INT2_YLIE_DISABLE 		 =0x00,
  	AIS328DQ_ACC_INT2_YLIE_ENABLE 		 =0x04,
} AIS328DQ_ACC_INT2_YLIE_t;

#define  	AIS328DQ_ACC_INT2_YLIE_MASK  	0x04
status_t  AIS328DQ_ACC_W_INT2_CFG_YLIE(void *handle, AIS328DQ_ACC_INT2_YLIE_t newValue);
status_t AIS328DQ_ACC_R_INT2_CFG_YLIE(void *handle, AIS328DQ_ACC_INT2_YLIE_t *value);

/*******************************************************************************
* Register      : INT2_CFG
* Address       : 30
* Bit Group Name: YHIE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_INT2_YHIE_DISABLE 		 =0x00,
  	AIS328DQ_ACC_INT2_YHIE_ENABLE 		 =0x08,
} AIS328DQ_ACC_INT2_YHIE_t;

#define  	AIS328DQ_ACC_INT2_YHIE_MASK  	0x08
status_t  AIS328DQ_ACC_W_INT2_CFG_YHIE(void *handle, AIS328DQ_ACC_INT2_YHIE_t newValue);
status_t AIS328DQ_ACC_R_INT2_CFG_YHIE(void *handle, AIS328DQ_ACC_INT2_YHIE_t *value);

/*******************************************************************************
* Register      : INT2_CFG
* Address       : 30
* Bit Group Name: ZLIE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_INT2_ZLIE_DISABLE 		 =0x00,
  	AIS328DQ_ACC_INT2_ZLIE_ENABLE 		 =0x10,
} AIS328DQ_ACC_INT2_ZLIE_t;

#define  	AIS328DQ_ACC_INT2_ZLIE_MASK  	0x10
status_t  AIS328DQ_ACC_W_INT2_CFG_ZLIE(void *handle, AIS328DQ_ACC_INT2_ZLIE_t newValue);
status_t AIS328DQ_ACC_R_INT2_CFG_ZLIE(void *handle, AIS328DQ_ACC_INT2_ZLIE_t *value);

/*******************************************************************************
* Register      : INT2_CFG
* Address       : 30
* Bit Group Name: ZHIE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_INT2_ZHIE_DISABLE 		 =0x00,
  	AIS328DQ_ACC_INT2_ZHIE_ENABLE 		 =0x20,
} AIS328DQ_ACC_INT2_ZHIE_t;

#define  	AIS328DQ_ACC_INT2_ZHIE_MASK  	0x20
status_t  AIS328DQ_ACC_W_INT2_CFG_ZHIE(void *handle, AIS328DQ_ACC_INT2_ZHIE_t newValue);
status_t AIS328DQ_ACC_R_INT2_CFG_ZHIE(void *handle, AIS328DQ_ACC_INT2_ZHIE_t *value);

/*******************************************************************************
* Register      : INT2_CFG
* Address       : 30
* Bit Group Name: 6D
* Permission    : RW
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_INT2_6D_DISABLE 		 =0x00,
  	AIS328DQ_ACC_INT2_6D_ENABLE 		 =0x40,
} AIS328DQ_ACC_INT2_6D_t;

#define  	AIS328DQ_ACC_INT2_6D_MASK  	0x40
status_t  AIS328DQ_ACC_W_INT2_6D(void *handle, AIS328DQ_ACC_INT2_6D_t newValue);
status_t AIS328DQ_ACC_R_INT2_6D(void *handle, AIS328DQ_ACC_INT2_6D_t *value);

/*******************************************************************************
* Register      : INT2_CFG
* Address       : 30
* Bit Group Name: AOI
* Permission    : RW
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_INT2_AOI_DISABLE 		 =0x00,
  	AIS328DQ_ACC_INT2_AOI_ENABLE 		 =0x80,
} AIS328DQ_ACC_INT2_AOI_t;

#define  	AIS328DQ_ACC_INT2_AOI_MASK  	0x80
status_t  AIS328DQ_ACC_W_INT2_CFG_AOI(void *handle, AIS328DQ_ACC_INT2_AOI_t newValue);
status_t AIS328DQ_ACC_R_INT2_CFG_AOI(void *handle, AIS328DQ_ACC_INT2_AOI_t *value);

/*******************************************************************************
* Register      : INT2_SOURCE
* Address       : 31
* Bit Group Name: XL
* Permission    : This value can be only OR or RW 
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_INT2_XL_NO_EVENT 		 =0x00,
  	AIS328DQ_ACC_INT2_XL_EVENT 		 =0x01,
} AIS328DQ_ACC_INT2_XL_t;

#define  	AIS328DQ_ACC_INT2_XL_MASK  	0x01
status_t AIS328DQ_ACC_R_INT2_SOURCE_XL(void *handle, AIS328DQ_ACC_INT2_XL_t *value);

/*******************************************************************************
* Register      : INT2_SOURCE
* Address       : 31
* Bit Group Name: XH
* Permission    : This value can be only OR or RW 
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_INT2_XH_NO_EVENT 		 =0x00,
  	AIS328DQ_ACC_INT2_XH_EVENT 		 =0x02,
} AIS328DQ_ACC_INT2_XH_t;

#define  	AIS328DQ_ACC_INT2_XH_MASK  	0x02
status_t AIS328DQ_ACC_R_INT2_SOURCE_XH(void *handle, AIS328DQ_ACC_INT2_XH_t *value);

/*******************************************************************************
* Register      : INT2_SOURCE
* Address       : 31
* Bit Group Name: YL
* Permission    : This value can be only OR or RW 
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_INT2_YL_NO_EVENT 		 =0x00,
  	AIS328DQ_ACC_INT2_YL_EVENT 		 =0x04,
} AIS328DQ_ACC_INT2_YL_t;

#define  	AIS328DQ_ACC_INT2_YL_MASK  	0x04
status_t AIS328DQ_ACC_R_INT2_SOURCE_YL(void *handle, AIS328DQ_ACC_INT2_YL_t *value);

/*******************************************************************************
* Register      : INT2_SOURCE
* Address       : 31
* Bit Group Name: YH
* Permission    : This value can be only OR or RW 
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_INT2_YH_NO_EVENT 		 =0x00,
  	AIS328DQ_ACC_INT2_YH_EVENT 		 =0x08,
} AIS328DQ_ACC_INT2_YH_t;

#define  	AIS328DQ_ACC_INT2_YH_MASK  	0x08
status_t AIS328DQ_ACC_R_INT2_SOURCE_YH(void *handle, AIS328DQ_ACC_INT2_YH_t *value);

/*******************************************************************************
* Register      : INT2_SOURCE
* Address       : 31
* Bit Group Name: ZL
* Permission    : This value can be only OR or RW 
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_INT2_ZL_NO_EVENT 		 =0x00,
  	AIS328DQ_ACC_INT2_ZL_EVENT 		 =0x10,
} AIS328DQ_ACC_INT2_ZL_t;

#define  	AIS328DQ_ACC_INT2_ZL_MASK  	0x10
status_t AIS328DQ_ACC_R_INT2_SOURCE_ZL(void *handle, AIS328DQ_ACC_INT2_ZL_t *value);

/*******************************************************************************
* Register      : INT2_SOURCE
* Address       : 31
* Bit Group Name: ZH
* Permission    : This value can be only OR or RW 
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_INT2_ZH_NO_EVENT 		 =0x00,
  	AIS328DQ_ACC_INT2_ZH_EVENT 		 =0x20,
} AIS328DQ_ACC_INT2_ZH_t;

#define  	AIS328DQ_ACC_INT2_ZH_MASK  	0x20
status_t AIS328DQ_ACC_R_INT2_SOURCE_ZH(void *handle, AIS328DQ_ACC_INT2_ZH_t *value);

/*******************************************************************************
* Register      : INT2_SOURCE
* Address       : 31
* Bit Group Name: IA
* Permission    : This value can be only OR or RW 
*******************************************************************************/
typedef enum {
  	AIS328DQ_ACC_INT2_IA_NO_EVENT 		 =0x00,
  	AIS328DQ_ACC_INT2_IA_EVENT 		 =0x40,
} AIS328DQ_ACC_INT2_IA_t;

#define  	AIS328DQ_ACC_INT2_IA_MASK  	0x40
status_t AIS328DQ_ACC_R_INT2_SOURCE_IA(void *handle, AIS328DQ_ACC_INT2_IA_t *value);

/*******************************************************************************
* Register      : INT2_THS
* Address       : 32
* Bit Group Name: THS
* Permission    : RW
*******************************************************************************/
#define  	AIS328DQ_ACC_INT2_THS_MASK  	0x7F
#define  	AIS328DQ_ACC_INT2_THS_POSITION  	0
status_t  AIS328DQ_ACC_W_INT2_THS(void *handle, u8_t newValue);
status_t AIS328DQ_ACC_R_INT2_THS(void *handle, u8_t *value);

/*******************************************************************************
* Register      : INT2_DURATION
* Address       : 33
* Bit Group Name: D
* Permission    : RW
*******************************************************************************/
#define  	AIS328DQ_ACC_INT2_D_MASK  	0x7F
#define  	AIS328DQ_ACC_INT2_D_POSITION  	0
status_t  AIS328DQ_ACC_W_INT2_DURATION(void *handle, u8_t newValue);
status_t AIS328DQ_ACC_R_INT2_DURATION(void *handle, u8_t *value);
/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : Reference
* Permission    : RW 
*******************************************************************************/
status_t AIS328DQ_ACC_Set_Reference(void *handle, u8_t *buff);
status_t AIS328DQ_ACC_Get_Reference(void *handle, u8_t *buff); 
/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : Acceleration
* Permission    : RO 
*******************************************************************************/
status_t AIS328DQ_ACC_Get_Acceleration(void *handle, u8_t *buff); 
#endif
