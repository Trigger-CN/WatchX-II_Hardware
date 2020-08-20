/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
* File Name          : A3G4250D_GYRO_driver.h
* Author             : MEMS Application Team
* Version            : v1.1
* Date               : 31 Mar 2016  
* Description        : A3G4250D source driver file
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
#ifndef __A3G4250D_GYRO_DRIVER__H
#define __A3G4250D_GYRO_DRIVER__H

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

#define A3G4250D_GYRO_I2C_ADDRESS         0xD2

/************** Who am I  *******************/

#define A3G4250D_GYRO_WHO_AM_I_VAL         0xD3

/* Private Function Prototype -------------------------------------------------------*/

/************** Device Register  *******************/
#define A3G4250D_GYRO_WHO_AM_I  	0X0F
#define A3G4250D_GYRO_CTRL_REG1  	0X20
#define A3G4250D_GYRO_CTRL_REG2  	0X21
#define A3G4250D_GYRO_CTRL_REG3  	0X22
#define A3G4250D_GYRO_CTRL_REG4  	0X23
#define A3G4250D_GYRO_CTRL_REG5  	0X24
#define A3G4250D_GYRO_REFERENCE  	0X25
#define A3G4250D_GYRO_OUT_TEMP  	0X26
#define A3G4250D_GYRO_STATUS_REG  	0X27
#define A3G4250D_GYRO_OUT_X_L  	0X28
#define A3G4250D_GYRO_OUT_X_H  	0X29
#define A3G4250D_GYRO_OUT_Y_L  	0X2A
#define A3G4250D_GYRO_OUT_Y_H  	0X2B
#define A3G4250D_GYRO_OUT_Z_L  	0X2C
#define A3G4250D_GYRO_OUT_Z_H  	0X2D
#define A3G4250D_GYRO_FIFO_CTRL_REG  	0X2E
#define A3G4250D_GYRO_FIFO_SRC_REG  	0X2F
#define A3G4250D_GYRO_INT1_CFG  	0X30
#define A3G4250D_GYRO_INT1_SRC  	0X31
#define A3G4250D_GYRO_INT1_TSH_XH  	0X32
#define A3G4250D_GYRO_INT1_TSH_XL  	0X33
#define A3G4250D_GYRO_INT1_TSH_YH  	0X34
#define A3G4250D_GYRO_INT1_TSH_YL  	0X35
#define A3G4250D_GYRO_INT1_TSH_ZH  	0X36
#define A3G4250D_GYRO_INT1_TSH_ZL  	0X37
#define A3G4250D_GYRO_INT1_DURATION  	0X38

/************** Generic Function  *******************/

/*******************************************************************************
* Register      : Generic - All
* Address       : Generic - All
* Bit Group Name: None
* Permission    : W
*******************************************************************************/
status_t A3G4250D_GYRO_WriteReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len);

/*******************************************************************************
* Register      : Generic - All
* Address       : Generic - All
* Bit Group Name: None
* Permission    : R
*******************************************************************************/
status_t A3G4250D_GYRO_ReadReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len);

/**************** Base Function  *******************/

/*******************************************************************************
* Register      : WHO_AM_I
* Address       : 0X0F
* Bit Group Name: WHO_AM_I_BIT
* Permission    : RO
*******************************************************************************/
#define  	A3G4250D_GYRO_WHO_AM_I_BIT_MASK  	0xFF
#define  	A3G4250D_GYRO_WHO_AM_I_BIT_POSITION  	0
status_t A3G4250D_GYRO_R_WHO_AM_I_BIT_bits(void *handle, u8_t *value);

/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : AngularRate
* Permission    : RO 
*******************************************************************************/
status_t A3G4250D_GYRO_Get_AngularRate(void *handle, u8_t *buff); 

/*******************************************************************************
* Register      : CTRL_REG1
* Address       : 0X20
* Bit Group Name: DR
* Permission    : RW
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_DR_100HZ 		 =0x00,
  	A3G4250D_GYRO_DR_200HZ 		 =0x40,
  	A3G4250D_GYRO_DR_400HZ 		 =0x80,
  	A3G4250D_GYRO_DR_800HZ 		 =0xC0,
} A3G4250D_GYRO_DR_t;

#define  	A3G4250D_GYRO_DR_MASK  	0xC0
status_t  A3G4250D_GYRO_W_DR_bits(void *handle, A3G4250D_GYRO_DR_t newValue);
status_t A3G4250D_GYRO_R_DR_bits(void *handle, A3G4250D_GYRO_DR_t *value);

/**************** Advanced Function  *******************/

/*******************************************************************************
* Register      : CTRL_REG1
* Address       : 0X20
* Bit Group Name: XEN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_XEN_DISABLE 		 =0x00,
  	A3G4250D_GYRO_XEN_ENABLE 		 =0x01,
} A3G4250D_GYRO_XEN_t;

#define  	A3G4250D_GYRO_XEN_MASK  	0x01
status_t  A3G4250D_GYRO_W_XEN_bits(void *handle, A3G4250D_GYRO_XEN_t newValue);
status_t A3G4250D_GYRO_R_XEN_bits(void *handle, A3G4250D_GYRO_XEN_t *value);

/*******************************************************************************
* Register      : CTRL_REG1
* Address       : 0X20
* Bit Group Name: YEN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_YEN_DISABLE 		 =0x00,
  	A3G4250D_GYRO_YEN_ENABLE 		 =0x02,
} A3G4250D_GYRO_YEN_t;

#define  	A3G4250D_GYRO_YEN_MASK  	0x02
status_t  A3G4250D_GYRO_W_YEN_bits(void *handle, A3G4250D_GYRO_YEN_t newValue);
status_t A3G4250D_GYRO_R_YEN_bits(void *handle, A3G4250D_GYRO_YEN_t *value);

/*******************************************************************************
* Register      : CTRL_REG1
* Address       : 0X20
* Bit Group Name: ZEN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_ZEN_DISABLE 		 =0x00,
  	A3G4250D_GYRO_ZEN_ENABLE 		 =0x04,
} A3G4250D_GYRO_ZEN_t;

#define  	A3G4250D_GYRO_ZEN_MASK  	0x04
status_t  A3G4250D_GYRO_W_ZEN_bits(void *handle, A3G4250D_GYRO_ZEN_t newValue);
status_t A3G4250D_GYRO_R_ZEN_bits(void *handle, A3G4250D_GYRO_ZEN_t *value);

/*******************************************************************************
* Register      : CTRL_REG1
* Address       : 0X20
* Bit Group Name: PD
* Permission    : RW
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_PD_PWRDWN_MODE 		 =0x00,
  	A3G4250D_GYRO_PD_NORMAL_MODE 		 =0x08,
} A3G4250D_GYRO_PD_t;

#define  	A3G4250D_GYRO_PD_MASK  	0x08
status_t  A3G4250D_GYRO_W_PD_bits(void *handle, A3G4250D_GYRO_PD_t newValue);
status_t A3G4250D_GYRO_R_PD_bits(void *handle, A3G4250D_GYRO_PD_t *value);

/*******************************************************************************
* Register      : CTRL_REG1
* Address       : 0X20
* Bit Group Name: BW
* Permission    : RW
*******************************************************************************/
#define  	A3G4250D_GYRO_BW_MASK  	0x30
#define  	A3G4250D_GYRO_BW_POSITION  	4
status_t  A3G4250D_GYRO_W_BW_bits(void *handle, u8_t newValue);
status_t A3G4250D_GYRO_R_BW_bits(void *handle, u8_t *value);

/*******************************************************************************
* Register      : CTRL_REG2
* Address       : 0X21
* Bit Group Name: HPCF
* Permission    : RW
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_HPCF_0 		 =0x00,
  	A3G4250D_GYRO_HPCF_1 		 =0x01,
  	A3G4250D_GYRO_HPCF_2 		 =0x02,
  	A3G4250D_GYRO_HPCF_3 		 =0x03,
  	A3G4250D_GYRO_HPCF_4 		 =0x04,
  	A3G4250D_GYRO_HPCF_5 		 =0x05,
  	A3G4250D_GYRO_HPCF_6 		 =0x06,
  	A3G4250D_GYRO_HPCF_7 		 =0x07,
  	A3G4250D_GYRO_HPCF_8 		 =0x08,
  	A3G4250D_GYRO_HPCF_9 		 =0x09,
} A3G4250D_GYRO_HPCF_t;

#define  	A3G4250D_GYRO_HPCF_MASK  	0x0F
status_t  A3G4250D_GYRO_W_HPCF_bits(void *handle, A3G4250D_GYRO_HPCF_t newValue);
status_t A3G4250D_GYRO_R_HPCF_bits(void *handle, A3G4250D_GYRO_HPCF_t *value);

/*******************************************************************************
* Register      : CTRL_REG2
* Address       : 0X21
* Bit Group Name: HPM
* Permission    : RW
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_HPM_NORMAL0 		 =0x00,
  	A3G4250D_GYRO_HPM_REF_SIG 		 =0x10,
  	A3G4250D_GYRO_HPM_NORMAL1 		 =0x20,
  	A3G4250D_GYRO_HPM_AUTO_RESET 		 =0x30,
} A3G4250D_GYRO_HPM_t;

#define  	A3G4250D_GYRO_HPM_MASK  	0x30
status_t  A3G4250D_GYRO_W_HPM_bits(void *handle, A3G4250D_GYRO_HPM_t newValue);
status_t A3G4250D_GYRO_R_HPM_bits(void *handle, A3G4250D_GYRO_HPM_t *value);

/*******************************************************************************
* Register      : CTRL_REG3
* Address       : 0X22
* Bit Group Name: I2_EMPTY
* Permission    : RW
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_I2_EMPTY_DISABLE 		 =0x00,
  	A3G4250D_GYRO_I2_EMPTY_ENABLE 		 =0x01,
} A3G4250D_GYRO_I2_EMPTY_t;

#define  	A3G4250D_GYRO_I2_EMPTY_MASK  	0x01
status_t  A3G4250D_GYRO_W_I2_EMPTY_bits(void *handle, A3G4250D_GYRO_I2_EMPTY_t newValue);
status_t A3G4250D_GYRO_R_I2_EMPTY_bits(void *handle, A3G4250D_GYRO_I2_EMPTY_t *value);

/*******************************************************************************
* Register      : CTRL_REG3
* Address       : 0X22
* Bit Group Name: I2_ORUN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_I2_ORUN_DISABLE 		 =0x00,
  	A3G4250D_GYRO_I2_ORUN_ENABLE 		 =0x02,
} A3G4250D_GYRO_I2_ORUN_t;

#define  	A3G4250D_GYRO_I2_ORUN_MASK  	0x02
status_t  A3G4250D_GYRO_W_I2_ORUN_bits(void *handle, A3G4250D_GYRO_I2_ORUN_t newValue);
status_t A3G4250D_GYRO_R_I2_ORUN_bits(void *handle, A3G4250D_GYRO_I2_ORUN_t *value);

/*******************************************************************************
* Register      : CTRL_REG3
* Address       : 0X22
* Bit Group Name: I2_WTM
* Permission    : RW
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_I2_WTM_DISABLE 		 =0x00,
  	A3G4250D_GYRO_I2_WTM_ENABLE 		 =0x04,
} A3G4250D_GYRO_I2_WTM_t;

#define  	A3G4250D_GYRO_I2_WTM_MASK  	0x04
status_t  A3G4250D_GYRO_W_I2_WTM_bits(void *handle, A3G4250D_GYRO_I2_WTM_t newValue);
status_t A3G4250D_GYRO_R_I2_WTM_bits(void *handle, A3G4250D_GYRO_I2_WTM_t *value);

/*******************************************************************************
* Register      : CTRL_REG3
* Address       : 0X22
* Bit Group Name: I2_DRDY
* Permission    : RW
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_I2_DRDY_DISABLE 		 =0x00,
  	A3G4250D_GYRO_I2_DRDY_ENABLE 		 =0x08,
} A3G4250D_GYRO_I2_DRDY_t;

#define  	A3G4250D_GYRO_I2_DRDY_MASK  	0x08
status_t  A3G4250D_GYRO_W_I2_DRDY_bits(void *handle, A3G4250D_GYRO_I2_DRDY_t newValue);
status_t A3G4250D_GYRO_R_I2_DRDY_bits(void *handle, A3G4250D_GYRO_I2_DRDY_t *value);

/*******************************************************************************
* Register      : CTRL_REG3
* Address       : 0X22
* Bit Group Name: PP_OD
* Permission    : RW
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_PP_OD_PUSH_PULL 		 =0x00,
  	A3G4250D_GYRO_PP_OD_OPEN_DRAIN 		 =0x10,
} A3G4250D_GYRO_PP_OD_t;

#define  	A3G4250D_GYRO_PP_OD_MASK  	0x10
status_t  A3G4250D_GYRO_W_PP_OD_bits(void *handle, A3G4250D_GYRO_PP_OD_t newValue);
status_t A3G4250D_GYRO_R_PP_OD_bits(void *handle, A3G4250D_GYRO_PP_OD_t *value);

/*******************************************************************************
* Register      : CTRL_REG3
* Address       : 0X22
* Bit Group Name: H_LACTIVE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_H_LACTIVE_HIGH 		 =0x00,
  	A3G4250D_GYRO_H_LACTIVE_LOW 		 =0x20,
} A3G4250D_GYRO_H_LACTIVE_t;

#define  	A3G4250D_GYRO_H_LACTIVE_MASK  	0x20
status_t  A3G4250D_GYRO_W_H_LACTIVE_bits(void *handle, A3G4250D_GYRO_H_LACTIVE_t newValue);
status_t A3G4250D_GYRO_R_H_LACTIVE_bits(void *handle, A3G4250D_GYRO_H_LACTIVE_t *value);

/*******************************************************************************
* Register      : CTRL_REG3
* Address       : 0X22
* Bit Group Name: I1_BOOT
* Permission    : RW
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_I1_BOOT_DISABLE 		 =0x00,
  	A3G4250D_GYRO_I1_BOOT_ENABLE 		 =0x40,
} A3G4250D_GYRO_I1_BOOT_t;

#define  	A3G4250D_GYRO_I1_BOOT_MASK  	0x40
status_t  A3G4250D_GYRO_W_I1_BOOT_bits(void *handle, A3G4250D_GYRO_I1_BOOT_t newValue);
status_t A3G4250D_GYRO_R_I1_BOOT_bits(void *handle, A3G4250D_GYRO_I1_BOOT_t *value);

/*******************************************************************************
* Register      : CTRL_REG3
* Address       : 0X22
* Bit Group Name: I1_INT1
* Permission    : RW
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_I1_INT1_DISABLE 		 =0x00,
  	A3G4250D_GYRO_I1_INT1_ENABLE 		 =0x80,
} A3G4250D_GYRO_I1_INT1_t;

#define  	A3G4250D_GYRO_I1_INT1_MASK  	0x80
status_t  A3G4250D_GYRO_W_I1_INT1_bits(void *handle, A3G4250D_GYRO_I1_INT1_t newValue);
status_t A3G4250D_GYRO_R_I1_INT1_bits(void *handle, A3G4250D_GYRO_I1_INT1_t *value);

/*******************************************************************************
* Register      : CTRL_REG4
* Address       : 0X23
* Bit Group Name: SIM
* Permission    : RW
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_SIM_4_WIRE 		 =0x00,
  	A3G4250D_GYRO_SIM_3_WIRE 		 =0x01,
} A3G4250D_GYRO_SIM_t;

#define  	A3G4250D_GYRO_SIM_MASK  	0x01
status_t  A3G4250D_GYRO_W_SIM_bits(void *handle, A3G4250D_GYRO_SIM_t newValue);
status_t A3G4250D_GYRO_R_SIM_bits(void *handle, A3G4250D_GYRO_SIM_t *value);

/*******************************************************************************
* Register      : CTRL_REG4
* Address       : 0X23
* Bit Group Name: ST
* Permission    : RW
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_ST_SELF_TEST_DISABLED 		 =0x00,
  	A3G4250D_GYRO_ST_SELF_TEST_0 		 =0x02,
  	A3G4250D_GYRO_ST_SELF_TEST_1 		 =0x06,
} A3G4250D_GYRO_ST_t;

#define  	A3G4250D_GYRO_ST_MASK  	0x06
status_t  A3G4250D_GYRO_W_ST_bits(void *handle, A3G4250D_GYRO_ST_t newValue);
status_t A3G4250D_GYRO_R_ST_bits(void *handle, A3G4250D_GYRO_ST_t *value);

/*******************************************************************************
* Register      : CTRL_REG4
* Address       : 0X23
* Bit Group Name: BLE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_BLE_LITTLE_ENDIAN 		 =0x00,
  	A3G4250D_GYRO_BLE_BIG_ENDIAN 		 =0x40,
} A3G4250D_GYRO_BLE_t;

#define  	A3G4250D_GYRO_BLE_MASK  	0x40
status_t  A3G4250D_GYRO_W_BLE_bits(void *handle, A3G4250D_GYRO_BLE_t newValue);
status_t A3G4250D_GYRO_R_BLE_bits(void *handle, A3G4250D_GYRO_BLE_t *value);

/*******************************************************************************
* Register      : CTRL_REG5
* Address       : 0X24
* Bit Group Name: OUT_SEL
* Permission    : RW
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_OUT_SEL_FILTER_0 		 =0x00,
  	A3G4250D_GYRO_OUT_SEL_FILTER_1 		 =0x01,
  	A3G4250D_GYRO_OUT_SEL_FILTER_2 		 =0x02,
  	A3G4250D_GYRO_OUT_SEL_FILTER_3 		 =0x03,
} A3G4250D_GYRO_OUT_SEL_t;

#define  	A3G4250D_GYRO_OUT_SEL_MASK  	0x03
status_t  A3G4250D_GYRO_W_OUT_SEL_bits(void *handle, A3G4250D_GYRO_OUT_SEL_t newValue);
status_t A3G4250D_GYRO_R_OUT_SEL_bits(void *handle, A3G4250D_GYRO_OUT_SEL_t *value);

/*******************************************************************************
* Register      : CTRL_REG5
* Address       : 0X24
* Bit Group Name: INT1_SEL
* Permission    : RW
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_INT1_SEL_FILTER_0 		 =0x00,
  	A3G4250D_GYRO_INT1_SEL_FILTER_1 		 =0x04,
  	A3G4250D_GYRO_INT1_SEL_FILTER_2 		 =0x08,
  	A3G4250D_GYRO_INT1_SEL_FILTER_3 		 =0x0C,
} A3G4250D_GYRO_INT1_SEL_t;

#define  	A3G4250D_GYRO_INT1_SEL_MASK  	0x0C
status_t  A3G4250D_GYRO_W_INT1_SEL_bits(void *handle, A3G4250D_GYRO_INT1_SEL_t newValue);
status_t A3G4250D_GYRO_R_INT1_SEL_bits(void *handle, A3G4250D_GYRO_INT1_SEL_t *value);

/*******************************************************************************
* Register      : CTRL_REG5
* Address       : 0X24
* Bit Group Name: HPEN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_HPEN_DISABLE 		 =0x00,
  	A3G4250D_GYRO_HPEN_ENABLE 		 =0x10,
} A3G4250D_GYRO_HPEN_t;

#define  	A3G4250D_GYRO_HPEN_MASK  	0x10
status_t  A3G4250D_GYRO_W_HPEN_bits(void *handle, A3G4250D_GYRO_HPEN_t newValue);
status_t A3G4250D_GYRO_R_HPEN_bits(void *handle, A3G4250D_GYRO_HPEN_t *value);

/*******************************************************************************
* Register      : CTRL_REG5
* Address       : 0X24
* Bit Group Name: FIFO_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_FIFO_EN_DISABLE 		 =0x00,
  	A3G4250D_GYRO_FIFO_EN_ENABLE 		 =0x40,
} A3G4250D_GYRO_FIFO_EN_t;

#define  	A3G4250D_GYRO_FIFO_EN_MASK  	0x40
status_t  A3G4250D_GYRO_W_FIFO_EN_bits(void *handle, A3G4250D_GYRO_FIFO_EN_t newValue);
status_t A3G4250D_GYRO_R_FIFO_EN_bits(void *handle, A3G4250D_GYRO_FIFO_EN_t *value);

/*******************************************************************************
* Register      : CTRL_REG5
* Address       : 0X24
* Bit Group Name: BOOT
* Permission    : RW
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_BOOT_NORMAL_MODE 		 =0x00,
  	A3G4250D_GYRO_BOOT_REBOOT_MEMORY 		 =0x80,
} A3G4250D_GYRO_BOOT_t;

#define  	A3G4250D_GYRO_BOOT_MASK  	0x80
status_t  A3G4250D_GYRO_W_BOOT_bits(void *handle, A3G4250D_GYRO_BOOT_t newValue);
status_t A3G4250D_GYRO_R_BOOT_bits(void *handle, A3G4250D_GYRO_BOOT_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X27
* Bit Group Name: XDA
* Permission    : RO
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_XDA_NOT_AVAIL 		 =0x00,
  	A3G4250D_GYRO_XDA_AVAIL 		 =0x01,
} A3G4250D_GYRO_XDA_t;

#define  	A3G4250D_GYRO_XDA_MASK  	0x01
status_t A3G4250D_GYRO_R_XDA_bits(void *handle, A3G4250D_GYRO_XDA_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X27
* Bit Group Name: YDA
* Permission    : RO
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_YDA_NOT_AVAIL 		 =0x00,
  	A3G4250D_GYRO_YDA_AVAIL 		 =0x02,
} A3G4250D_GYRO_YDA_t;

#define  	A3G4250D_GYRO_YDA_MASK  	0x02
status_t A3G4250D_GYRO_R_YDA_bits(void *handle, A3G4250D_GYRO_YDA_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X27
* Bit Group Name: ZDA
* Permission    : RO
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_ZDA_NOT_AVAIL 		 =0x00,
  	A3G4250D_GYRO_ZDA_AVAIL 		 =0x04,
} A3G4250D_GYRO_ZDA_t;

#define  	A3G4250D_GYRO_ZDA_MASK  	0x04
status_t A3G4250D_GYRO_R_ZDA_bits(void *handle, A3G4250D_GYRO_ZDA_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X27
* Bit Group Name: ZYXDA
* Permission    : RO
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_ZYXDA_NOT_AVAIL 		 =0x00,
  	A3G4250D_GYRO_ZYXDA_AVAIL 		 =0x08,
} A3G4250D_GYRO_ZYXDA_t;

#define  	A3G4250D_GYRO_ZYXDA_MASK  	0x08
status_t A3G4250D_GYRO_R_ZYXDA_bits(void *handle, A3G4250D_GYRO_ZYXDA_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X27
* Bit Group Name: XOR
* Permission    : RO
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_XOR_NO_OVERRUN 		 =0x00,
  	A3G4250D_GYRO_XOR_OVERRUN 		 =0x10,
} A3G4250D_GYRO_XOR_t;

#define  	A3G4250D_GYRO_XOR_MASK  	0x10
status_t A3G4250D_GYRO_R_XOR_bits(void *handle, A3G4250D_GYRO_XOR_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X27
* Bit Group Name: YOR
* Permission    : RO
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_YOR_NO_OVERRUN 		 =0x00,
  	A3G4250D_GYRO_YOR_OVERRUN 		 =0x20,
} A3G4250D_GYRO_YOR_t;

#define  	A3G4250D_GYRO_YOR_MASK  	0x20
status_t A3G4250D_GYRO_R_YOR_bits(void *handle, A3G4250D_GYRO_YOR_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X27
* Bit Group Name: ZOR
* Permission    : RO
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_ZOR_NO_OVERRUN 		 =0x00,
  	A3G4250D_GYRO_ZOR_OVERRUN 		 =0x40,
} A3G4250D_GYRO_ZOR_t;

#define  	A3G4250D_GYRO_ZOR_MASK  	0x40
status_t A3G4250D_GYRO_R_ZOR_bits(void *handle, A3G4250D_GYRO_ZOR_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X27
* Bit Group Name: ZYXOR
* Permission    : RO
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_ZYXOR_NO_OVERRUN 		 =0x00,
  	A3G4250D_GYRO_ZYXOR_OVERRUN 		 =0x80,
} A3G4250D_GYRO_ZYXOR_t;

#define  	A3G4250D_GYRO_ZYXOR_MASK  	0x80
status_t A3G4250D_GYRO_R_ZYXOR_bits(void *handle, A3G4250D_GYRO_ZYXOR_t *value);

/*******************************************************************************
* Register      : FIFO_CTRL_REG
* Address       : 0X2E
* Bit Group Name: WTM
* Permission    : RW
*******************************************************************************/
#define  	A3G4250D_GYRO_THSD_WTM_MASK  	0x1F
#define  	A3G4250D_GYRO_THSD_WTM_POSITION  	0
status_t  A3G4250D_GYRO_W_THSD_WTM_bits(void *handle, u8_t newValue);
status_t A3G4250D_GYRO_R_THSD_WTM_bits(void *handle, u8_t *value);

/*******************************************************************************
* Register      : FIFO_CTRL_REG
* Address       : 0X2E
* Bit Group Name: FM
* Permission    : RW
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_FM_BYPASS_MODE 		 =0x00,
  	A3G4250D_GYRO_FM_FIFO_MODE 		 =0x20,
  	A3G4250D_GYRO_FM_STREAM_MODE 		 =0x40,
} A3G4250D_GYRO_FM_t;

#define  	A3G4250D_GYRO_FM_MASK  	0xE0
status_t  A3G4250D_GYRO_W_FM_bits(void *handle, A3G4250D_GYRO_FM_t newValue);
status_t A3G4250D_GYRO_R_FM_bits(void *handle, A3G4250D_GYRO_FM_t *value);

/*******************************************************************************
* Register      : FIFO_SRC_REG
* Address       : 0X2F
* Bit Group Name: FSS
* Permission    : RO
*******************************************************************************/
#define  	A3G4250D_GYRO_FSS_MASK  	0x1F
#define  	A3G4250D_GYRO_FSS_POSITION  	0
status_t A3G4250D_GYRO_R_FSS_bits(void *handle, u8_t *value);

/*******************************************************************************
* Register      : FIFO_SRC_REG
* Address       : 0X2F
* Bit Group Name: EMPTY
* Permission    : RO
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_EMPTY_NOT_EMPTY 		 =0x00,
  	A3G4250D_GYRO_EMPTY_EMPTY 		 =0x20,
} A3G4250D_GYRO_EMPTY_t;

#define  	A3G4250D_GYRO_EMPTY_MASK  	0x20
status_t A3G4250D_GYRO_R_EMPTY_bits(void *handle, A3G4250D_GYRO_EMPTY_t *value);

/*******************************************************************************
* Register      : FIFO_SRC_REG
* Address       : 0X2F
* Bit Group Name: OVRN
* Permission    : RO
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_OVRN_FIFO_NOT_FULL 		 =0x00,
  	A3G4250D_GYRO_OVRN_FIFO_FULL 		 =0x40,
} A3G4250D_GYRO_OVRN_t;

#define  	A3G4250D_GYRO_OVRN_MASK  	0x40
status_t A3G4250D_GYRO_R_OVRN_bits(void *handle, A3G4250D_GYRO_OVRN_t *value);

/*******************************************************************************
* Register      : FIFO_SRC_REG
* Address       : 0X2F
* Bit Group Name: WTM
* Permission    : RO
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_WTM_LOWER 		 =0x00,
  	A3G4250D_GYRO_WTM_HIGHER 		 =0x80,
} A3G4250D_GYRO_WTM_t;

#define  	A3G4250D_GYRO_WTM_MASK  	0x80
status_t A3G4250D_GYRO_R_WTM_bits(void *handle, A3G4250D_GYRO_WTM_t *value);

/*******************************************************************************
* Register      : INT1_CFG
* Address       : 0X30
* Bit Group Name: XLIE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_XLIE_DISABLE 		 =0x00,
  	A3G4250D_GYRO_XLIE_ENABLE 		 =0x01,
} A3G4250D_GYRO_XLIE_t;

#define  	A3G4250D_GYRO_XLIE_MASK  	0x01
status_t  A3G4250D_GYRO_W_XLIE_bits(void *handle, A3G4250D_GYRO_XLIE_t newValue);
status_t A3G4250D_GYRO_R_XLIE_bits(void *handle, A3G4250D_GYRO_XLIE_t *value);

/*******************************************************************************
* Register      : INT1_CFG
* Address       : 0X30
* Bit Group Name: XHIE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_XHIE_DISABLE 		 =0x00,
  	A3G4250D_GYRO_XHIE_ENABLE 		 =0x02,
} A3G4250D_GYRO_XHIE_t;

#define  	A3G4250D_GYRO_XHIE_MASK  	0x02
status_t  A3G4250D_GYRO_W_XHIE_bits(void *handle, A3G4250D_GYRO_XHIE_t newValue);
status_t A3G4250D_GYRO_R_XHIE_bits(void *handle, A3G4250D_GYRO_XHIE_t *value);

/*******************************************************************************
* Register      : INT1_CFG
* Address       : 0X30
* Bit Group Name: YLIE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_YLIE_DISABLE 		 =0x00,
  	A3G4250D_GYRO_YLIE_ENABLE 		 =0x04,
} A3G4250D_GYRO_YLIE_t;

#define  	A3G4250D_GYRO_YLIE_MASK  	0x04
status_t  A3G4250D_GYRO_W_YLIE_bits(void *handle, A3G4250D_GYRO_YLIE_t newValue);
status_t A3G4250D_GYRO_R_YLIE_bits(void *handle, A3G4250D_GYRO_YLIE_t *value);

/*******************************************************************************
* Register      : INT1_CFG
* Address       : 0X30
* Bit Group Name: YHIE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_YHIE_DISABLE 		 =0x00,
  	A3G4250D_GYRO_YHIE_ENABLE 		 =0x08,
} A3G4250D_GYRO_YHIE_t;

#define  	A3G4250D_GYRO_YHIE_MASK  	0x08
status_t  A3G4250D_GYRO_W_YHIE_bits(void *handle, A3G4250D_GYRO_YHIE_t newValue);
status_t A3G4250D_GYRO_R_YHIE_bits(void *handle, A3G4250D_GYRO_YHIE_t *value);

/*******************************************************************************
* Register      : INT1_CFG
* Address       : 0X30
* Bit Group Name: ZLIE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_ZLIE_DISABLE 		 =0x00,
  	A3G4250D_GYRO_ZLIE_ENABLE 		 =0x10,
} A3G4250D_GYRO_ZLIE_t;

#define  	A3G4250D_GYRO_ZLIE_MASK  	0x10
status_t  A3G4250D_GYRO_W_ZLIE_bits(void *handle, A3G4250D_GYRO_ZLIE_t newValue);
status_t A3G4250D_GYRO_R_ZLIE_bits(void *handle, A3G4250D_GYRO_ZLIE_t *value);

/*******************************************************************************
* Register      : INT1_CFG
* Address       : 0X30
* Bit Group Name: ZHIE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_ZHIE_DISABLE 		 =0x00,
  	A3G4250D_GYRO_ZHIE_ENABLE 		 =0x20,
} A3G4250D_GYRO_ZHIE_t;

#define  	A3G4250D_GYRO_ZHIE_MASK  	0x20
status_t  A3G4250D_GYRO_W_ZHIE_bits(void *handle, A3G4250D_GYRO_ZHIE_t newValue);
status_t A3G4250D_GYRO_R_ZHIE_bits(void *handle, A3G4250D_GYRO_ZHIE_t *value);

/*******************************************************************************
* Register      : INT1_CFG
* Address       : 0X30
* Bit Group Name: LIR
* Permission    : RW
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_LIR_NOT_LATCHED 		 =0x00,
  	A3G4250D_GYRO_LIR_LATCHED 		 =0x40,
} A3G4250D_GYRO_LIR_t;

#define  	A3G4250D_GYRO_LIR_MASK  	0x40
status_t  A3G4250D_GYRO_W_LIR_bits(void *handle, A3G4250D_GYRO_LIR_t newValue);
status_t A3G4250D_GYRO_R_LIR_bits(void *handle, A3G4250D_GYRO_LIR_t *value);

/*******************************************************************************
* Register      : INT1_CFG
* Address       : 0X30
* Bit Group Name: AND_OR
* Permission    : RW
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_AND_OR_OR_COMBINATION 		 =0x00,
  	A3G4250D_GYRO_AND_OR_AND_COMBINATION 		 =0x80,
} A3G4250D_GYRO_AND_OR_t;

#define  	A3G4250D_GYRO_AND_OR_MASK  	0x80
status_t  A3G4250D_GYRO_W_AND_OR_bits(void *handle, A3G4250D_GYRO_AND_OR_t newValue);
status_t A3G4250D_GYRO_R_AND_OR_bits(void *handle, A3G4250D_GYRO_AND_OR_t *value);

/*******************************************************************************
* Register      : INT1_SRC
* Address       : 0X31
* Bit Group Name: XL
* Permission    : RO
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_XL_NO_EVENT 		 =0x00,
  	A3G4250D_GYRO_XL_EVENT 		 =0x01,
} A3G4250D_GYRO_XL_t;

#define  	A3G4250D_GYRO_XL_MASK  	0x01
status_t A3G4250D_GYRO_R_XL_bits(void *handle, A3G4250D_GYRO_XL_t *value);

/*******************************************************************************
* Register      : INT1_SRC
* Address       : 0X31
* Bit Group Name: XH
* Permission    : RO
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_XH_NO_EVENT 		 =0x00,
  	A3G4250D_GYRO_XH_EVENT 		 =0x02,
} A3G4250D_GYRO_XH_t;

#define  	A3G4250D_GYRO_XH_MASK  	0x02
status_t A3G4250D_GYRO_R_XH_bits(void *handle, A3G4250D_GYRO_XH_t *value);

/*******************************************************************************
* Register      : INT1_SRC
* Address       : 0X31
* Bit Group Name: YL
* Permission    : RO
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_YL_NO_EVENT 		 =0x00,
  	A3G4250D_GYRO_YL_EVENT 		 =0x04,
} A3G4250D_GYRO_YL_t;

#define  	A3G4250D_GYRO_YL_MASK  	0x04
status_t A3G4250D_GYRO_R_YL_bits(void *handle, A3G4250D_GYRO_YL_t *value);

/*******************************************************************************
* Register      : INT1_SRC
* Address       : 0X31
* Bit Group Name: YH
* Permission    : RO
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_YH_NO_EVENT 		 =0x00,
  	A3G4250D_GYRO_YH_EVENT 		 =0x08,
} A3G4250D_GYRO_YH_t;

#define  	A3G4250D_GYRO_YH_MASK  	0x08
status_t A3G4250D_GYRO_R_YH_bits(void *handle, A3G4250D_GYRO_YH_t *value);

/*******************************************************************************
* Register      : INT1_SRC
* Address       : 0X31
* Bit Group Name: ZL
* Permission    : RO
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_ZL_NO_EVENT 		 =0x00,
  	A3G4250D_GYRO_ZL_EVENT 		 =0x10,
} A3G4250D_GYRO_ZL_t;

#define  	A3G4250D_GYRO_ZL_MASK  	0x10
status_t A3G4250D_GYRO_R_ZL_bits(void *handle, A3G4250D_GYRO_ZL_t *value);

/*******************************************************************************
* Register      : INT1_SRC
* Address       : 0X31
* Bit Group Name: ZH
* Permission    : RO
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_ZH_NO_EVENT 		 =0x00,
  	A3G4250D_GYRO_ZH_EVENT 		 =0x20,
} A3G4250D_GYRO_ZH_t;

#define  	A3G4250D_GYRO_ZH_MASK  	0x20
status_t A3G4250D_GYRO_R_ZH_bits(void *handle, A3G4250D_GYRO_ZH_t *value);

/*******************************************************************************
* Register      : INT1_SRC
* Address       : 0X31
* Bit Group Name: IA
* Permission    : RO
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_IA_NO_EVENT 		 =0x00,
  	A3G4250D_GYRO_IA_EVENT 		 =0x40,
} A3G4250D_GYRO_IA_t;

#define  	A3G4250D_GYRO_IA_MASK  	0x40
status_t A3G4250D_GYRO_R_IA_bits(void *handle, A3G4250D_GYRO_IA_t *value);

/*******************************************************************************
* Register      : INT1_DURATION
* Address       : 0X38
* Bit Group Name: D
* Permission    : RW
*******************************************************************************/
#define  	A3G4250D_GYRO_D_MASK  	0x7F
#define  	A3G4250D_GYRO_D_POSITION  	0
status_t  A3G4250D_GYRO_W_D_bits(void *handle, u8_t newValue);
status_t A3G4250D_GYRO_R_D_bits(void *handle, u8_t *value);

/*******************************************************************************
* Register      : INT1_DURATION
* Address       : 0X38
* Bit Group Name: WAIT
* Permission    : RW
*******************************************************************************/
typedef enum {
  	A3G4250D_GYRO_WAIT_DISABLE 		 =0x00,
  	A3G4250D_GYRO_WAIT_ENABLE 		 =0x80,
} A3G4250D_GYRO_WAIT_t;

#define  	A3G4250D_GYRO_WAIT_MASK  	0x80
status_t  A3G4250D_GYRO_W_WAIT_bits(void *handle, A3G4250D_GYRO_WAIT_t newValue);
status_t A3G4250D_GYRO_R_WAIT_bits(void *handle, A3G4250D_GYRO_WAIT_t *value);

/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : Reference
* Permission    : RW 
*******************************************************************************/
status_t A3G4250D_GYRO_Set_Reference(void *handle, u8_t *buff);
status_t A3G4250D_GYRO_Get_Reference(void *handle, u8_t *buff); 

/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : Temperature
* Permission    : RO 
*******************************************************************************/
status_t A3G4250D_GYRO_Get_Temperature(void *handle, u8_t *buff); 

/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : Threshold
* Permission    : RW 
*******************************************************************************/
status_t A3G4250D_GYRO_Set_Threshold(void *handle, u8_t *buff);
status_t A3G4250D_GYRO_Get_Threshold(void *handle, u8_t *buff); 

#endif
