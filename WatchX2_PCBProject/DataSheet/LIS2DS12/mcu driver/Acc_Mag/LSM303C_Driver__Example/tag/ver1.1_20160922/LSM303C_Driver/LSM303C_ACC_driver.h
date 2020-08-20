/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
* File Name          : LSM303C_ACC_driver.h
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
#ifndef __LSM303C_ACC_DRIVER__H
#define __LSM303C_ACC_DRIVER__H

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

//#ifndef __SHARED__TYPES
//#define __SHARED__TYPES
//
//typedef struct {
//  i16_t AXIS_X;
//  i16_t AXIS_Y;
//  i16_t AXIS_Z;
//} AxesRaw_t;
//
//typedef u8_t IntPinConf_t;
//typedef u8_t Axis_t;
//typedef u8_t IntConf_t;
//
//#endif /*__SHARED__TYPES*/
/* Exported device structure --------------------------------------------------------*/
//You can use LSM303C_ACC_AXIS_EN_t with or condition
typedef enum { 
	LSM303C_ACC_DISABLE_ALL 		 = 0x00,
	LSM303C_ACC_X_ENABLE 		 = 0x01,
	LSM303C_ACC_Y_ENABLE 		 = 0x02,
	LSM303C_ACC_Z_ENABLE 		 = 0x04
} LSM303C_ACC_AXIS_EN_t;

typedef enum { 
	LSM303C_ACC_BDU_DISABLE 		 = 0x00,
	LSM303C_ACC_BDU_ENABLE 		 = 0x08
} LSM303C_ACC_BDU_t;

typedef enum { 
	LSM303C_ACC_ODR_POWER_DOWN 	 = 0x00,
	LSM303C_ACC_ODR_10_Hz 		 = 0x10,
	LSM303C_ACC_ODR_50_Hz 		 = 0x20,
	LSM303C_ACC_ODR_100_Hz 		 = 0x30,
	LSM303C_ACC_ODR_200_Hz 		 = 0x40,
	LSM303C_ACC_ODR_400_Hz 		 = 0x50,
	LSM303C_ACC_ODR_800_Hz 		 = 0x60,
	LSM303C_ACC_ODR_MASK 		 = 0x60
} LSM303C_ACC_ODR_t;

typedef enum { 
	LSM303C_ACC_HIGH_RES_OFF 		 = 0x00,
	LSM303C_ACC_HIGH_RES_ON	 		 = 0x80
} LSM303C_ACC_HR_t;

typedef enum { 
	LSM303C_ACC_HP_INT1_DIS	 		= 0x00,
	LSM303C_ACC_HP_INT1_EN		 	= 0x01
} LSM303C_ACC_HPIS_INT1_t;

typedef enum { 
	LSM303C_ACC_HP_INT2_DIS	 		= 0x00,
	LSM303C_ACC_HP_INT2_EN		 	= 0x02
} LSM303C_ACC_HPIS_INT2_t;

typedef enum { 
	LSM303C_ACC_FDS_BYPASS 		 = 0x00,
	LSM303C_ACC_FDS_FILTER 		 = 0x04
} LSM303C_ACC_FDS_t;
//Referring DS to HP filter Usage
typedef enum { 
	LSM303C_ACC_HPM_NORMAL_MODE 		 = 0x00,
	LSM303C_ACC_HPM_REF_FOR_FILT 		 = 0x08
} LSM303C_ACC_HPM_t;
//Referring DS to HP filter Usage
typedef enum { 
	LSM303C_ACC_DFC_ODR_DIV_50 		 = 0x00,
	LSM303C_ACC_DFC_ODR_DIV_100 		 = 0x20,
	LSM303C_ACC_DFC_ODR_DIV_9 		 = 0x40,
	LSM303C_ACC_DFC_ODR_DIV_400		 = 0x60
} LSM303C_ACC_DFC_t;

typedef enum { 
	LSM303C_ACC_INT1_DISABLE 		= 0x00,
	LSM303C_ACC_INT1_DRDY 		 	= 0x01,
	LSM303C_ACC_INT1_FIFO_TH 		= 0x02,
	LSM303C_ACC_INT1_FIFO_OVR 	 	= 0x00,
	LSM303C_ACC_INT1_INTGEN1 		= 0x08,
	LSM303C_ACC_INT1_INTGEN2 		= 0x10,
	LSM303C_ACC_INT1_INACTIVITY 		= 0x20
} LSM303C_ACC_INT1_DRDY_t;

typedef enum { 
	LSM303C_ACC_INT2_DISABLE 		= 0x00,
	LSM303C_ACC_INT2_DRDY 		 	= 0x01,
	LSM303C_ACC_INT2_FIFO_TH 		= 0x02,
	LSM303C_ACC_INT2_FIFO_EMPTY 	 	= 0x00,
	LSM303C_ACC_INT2_INTGEN1 		= 0x08,
	LSM303C_ACC_INT2_INTGEN2 		= 0x10,
	LSM303C_ACC_INT2_BOOT 			= 0x20
} LSM303C_ACC_INT2_DRDY_t;

typedef enum { 
	LSM303C_ACC_REBOOT				= 0x80
} LSM303C_ACC_INT2_INTGEN2_t;

typedef enum { 
	LSM303C_ACC_SIM_4WIRE_INTERFACE 		 = 0x00,
	LSM303C_ACC_SIM_3WIRE_INTERFACE 		 = 0x01
} LSM303C_ACC_SIM_t;

typedef enum { 
	LSM303C_ACC_I2C_ENABLE 		 = 0x00,
	LSM303C_ACC_I2C_DISABLE 		 = 0x02
} LSM303C_ACC_I2C_t;

typedef enum { 
	LSM303C_ACC_IF_ADD_INC_DISABLE 		 = 0x00,
	LSM303C_ACC_IF_ADD_INC_ENABLE 		 = 0x04
} LSM303C_ACC_IF_ADD_INC_t;

typedef enum { 
	LSM303C_ACC_FS_2g 		 = 0x00,
	LSM303C_ACC_FS_4g 		 = 0x20,
	LSM303C_ACC_FS_8g 		 = 0x30
} LSM303C_ACC_FS_t;


typedef enum { 
	LSM303C_ACC_BW_400_Hz 		 = 0x08,
	LSM303C_ACC_BW_200_Hz 		 = 0x48,
	LSM303C_ACC_BW_100_Hz 		 = 0x88,
	LSM303C_ACC_BW_50_Hz 		 = 0xC8
} LSM303C_ACC_BW_t;

typedef enum { 
	LSM303C_ACC_PUSH_PULL_ACTIVE_HIGH		 = 0x00,
	LSM303C_ACC_OPEN_DRAIN_ACTIVE_HIGH 	 	 = 0x01,
	LSM303C_ACC_PUSH_PULL_ACTIVE_LOW	 	 	 = 0x02,
	LSM303C_ACC_OPEN_DRAIN_ACTIVE_LOW 	 	 = 0x03
} LSM303C_ACC_INT_PIN_CFG_t;

typedef enum { 
	LSM303C_ACC_ST_DISABLE 			= 0x00,
	LSM303C_ACC_ST_POSITIVE 			= 0x04,
	LSM303C_ACC_ST_NEGATIVE 		 	= 0x08,
	LSM303C_ACC_ST_NA 		 		= 0x0C
} LSM303C_ACC_ST_t;

typedef enum { 
	LSM303C_ACC_DEC_DISABLE 		 = 0x00,
	LSM303C_ACC_DEC_2_SAMPLES 		 = 0x10,
	LSM303C_ACC_DEC_4_SAMPLES 		 = 0x20,
	LSM303C_ACC_DEC_8_SAMPLES 		 = 0x30
} LSM303C_ACC_DEC_t;

typedef enum { 
	LSM303C_ACC_SOFT_RESET_DISABLE 		 = 0x00,
	LSM303C_ACC_SOFT_RESET_ENABLE 		 = 0x40
} LSM303C_ACC_SOFT_RESET_t;

typedef enum { 
	LSM303C_ACC_DEBUG_DISABLE 		 = 0x00,
	LSM303C_ACC_DEBUG_ENABLE 		 = 0x80
} LSM303C_ACC_DEBUG_t;

typedef enum { 
	LSM303C_ACC_INT1_PULS_INT2_PULS	 		= 0x00,
	LSM303C_ACC_INT1_LAT_INT2_PULS 		 	= 0x04,
	LSM303C_ACC_INT1_PULS_INT2_LAT 		 	= 0x08,
	LSM303C_ACC_INT1_LAT_INT2_LAT 		 	= 0x0C
} LSM303C_ACC_LAT_SIG_t;

typedef enum { 
	LSM303C_ACC_DUR1_NORST_DUR2_NORST	 = 0x00,
	LSM303C_ACC_DUR1_RST_DUR2_NORST		 = 0x10,
	LSM303C_ACC_DUR1_NORST_DUR2_RST	 	 = 0x20,
	LSM303C_ACC_DUR1_RST_DUR2_RST		 = 0x30
} LSM303C_ACC_RESET_DUR_t;

//You can use this typedef with or condition 
typedef enum {
	LSM303C_ACC_IG1_ON_LOW_X_FLAG_UP		 	= 0x01,	
	LSM303C_ACC_IG1_ON_HIGH_X_FLAG_UP		= 0x02,
	LSM303C_ACC_IG1_ON_LOW_Y_FLAG_UP		 	= 0x04,
	LSM303C_ACC_IG1_ON_HIGH_Y_FLAG_UP		= 0x08,
	LSM303C_ACC_IG1_ON_LOW_Z_FLAG_UP		 	= 0x10,
	LSM303C_ACC_IG1_ON_HIGH_Z_FLAG_UP		= 0x20,
	LSM303C_ACC_IA_IG1_FLAG_UP 		 		= 0x40
} LSM303C_ACC_IG_FLAGS_t;

typedef enum { 
	LSM303C_ACC_X_NEW_DATA_AVAILABLE 		 = 0x01,
	LSM303C_ACC_Y_NEW_DATA_AVAILABLE 		 = 0x02,
	LSM303C_ACC_Z_NEW_DATA_AVAILABLE 		 = 0x04,
	LSM303C_ACC_ZYX_NEW_DATA_AVAILABLE 		 = 0x08,
	LSM303C_ACC_X_OVERRUN 		 = 0x10,
	LSM303C_ACC_Y_OVERRUN 		 = 0x20,
	LSM303C_ACC_Z_OVERRUN 		 = 0x40,
	LSM303C_ACC_ZYX_OVERRUN 		 = 0x80
} LSM303C_ACC_STATUS_FLAGS_t;

//You can use this typedef with or condition see DS 4D and 6D interrupt
typedef enum { 
	LSM303C_ACC_IG_DIS		 		 	= 0x00,
	LSM303C_ACC_IG_LOW_X	 			 	= 0x01,
	LSM303C_ACC_IG_HIGH_X	 		 	= 0x02,
	LSM303C_ACC_IG_LOW_Y_	 		 	= 0x04,
	LSM303C_ACC_IG_HIGH_Y	 			= 0x08,
	LSM303C_ACC_IG_LOW_Z		 			= 0x10,
	LSM303C_ACC_IG_HIGH_Z	 			= 0x20,
	LSM303C_ACC_IG_6D		 		 	= 0x40,	
	LSM303C_ACC_IG_4D		 		 	= 0x80	
} LSM303C_ACC_IG_CONFIG_t;

typedef enum { 
	LSM303C_ACC_4D_INTGEN1_DIS 		 = 0x00,
	LSM303C_ACC_4D_INTGEN1_EN 		 = 0x01
} LSM303C_ACC_4D_INTGEN1_t;

typedef enum {	
	LSM303C_ACC_4D_INTGEN2_DIS 		 = 0x00,
	LSM303C_ACC_4D_INTGEN2_EN 		 = 0x02
} LSM303C_ACC_4D_INTGEN2_t;

typedef enum { 
	LSM303C_ACC_AOI_IG_OR 		 		= 0x00,
	LSM303C_ACC_AOI_IG_AND 		 		= 0x80
} LSM303C_ACC_AOI_IG_t;

typedef enum { 
	LSM303C_ACC_WAIT_IG_OFF 		 			= 0x00,
	LSM303C_ACC_WAIT_IG_ON 		 			= 0x80
} LSM303C_ACC_WAIT_IG_t;

typedef enum { 
	LSM303C_ACC_FMODE_BYPASS 				 = 0x00,
	LSM303C_ACC_FMODE_STOP_WHEN_FULL 		 = 0x20,
	LSM303C_ACC_FMODE_STREAM 		 		 = 0x40,
	LSM303C_ACC_FMODE_STREAM_TO_FIFO 		 = 0x60,
	LSM303C_ACC_FMODE_BYPASS_TO_STREAM 		 = 0x80,
	LSM303C_ACC_FMODE_BYPASS_TO_FIFO 		 = 0xE0
} LSM303C_ACC_FMODE_t;

typedef enum { 
	LSM303C_ACC_FIFO_EMPTY_TRUE 		 = 0x20,
	LSM303C_ACC_OVR_FIFO_TRUE 		 = 0x40,
	LSM303C_ACC_FTH_PASS 			 = 0x80
} LSM303C_ACC_FIFO_FLAGS_t;

typedef enum { 
	LSM303C_ACC_STOP_FTH_DIS_FIFO_TH 	 = 0x00,
	LSM303C_ACC_STOP_FTH_EN_FIFO_TH 		 = 0x40
} LSM303C_ACC_STOP_FTH_t;

typedef enum { 
	LSM303C_ACC_FIFO_DISABLE 	 = 0x00,
	LSM303C_ACC_FIFO_ENABLE 		 = 0x80
} LSM303C_ACC_FIFO_EN_t;

#define 	LSM303C_ACC_WHO_AM_I		 0x41
#define 	LSM303C_ACC_MEMS_I2C_ADDRESS  0x3A

/* Registers Name ------------------------------------------------------------------------*/

#define 	LSM303C_ACC_TEMP_L		 0x0B
#define 	LSM303C_ACC_TEMP_H		 0x0C
#define 	LSM303C_ACC_ACT_TSH		 0x1E
#define 	LSM303C_ACC_ACT_DUR		 0x1F
#define 	LSM303C_ACC_WHO_AM_I_REG	 0x0F
#define 	LSM303C_ACC_CTRL1		 0x20
#define 	LSM303C_ACC_CTRL2		 0x21
#define 	LSM303C_ACC_CTRL3		 0x22
#define 	LSM303C_ACC_CTRL4		 0x23
#define 	LSM303C_ACC_CTRL5		 0x24
#define 	LSM303C_ACC_CTRL6		 0x25
#define 	LSM303C_ACC_CTRL7		 0x26
#define 	LSM303C_ACC_STATUS		 0x27
#define 	LSM303C_ACC_OUT_X_L		 0x28
#define 	LSM303C_ACC_OUT_X_H		 0x29
#define 	LSM303C_ACC_OUT_Y_L		 0x2A
#define 	LSM303C_ACC_OUT_Y_H		 0x2B
#define 	LSM303C_ACC_OUT_Z_L		 0x2C
#define 	LSM303C_ACC_OUT_Z_H		 0x2D
#define 	LSM303C_ACC_FIFO_CTRL	 0x2E
#define 	LSM303C_ACC_FIFO_SRC		 0x2F
#define 	LSM303C_ACC_IG_CFG1		 0x30
#define 	LSM303C_ACC_IG_SRC1		 0x31
#define 	LSM303C_ACC_IG_THS_X1	 0x32
#define 	LSM303C_ACC_IG_THS_Y1	 0x33
#define 	LSM303C_ACC_IG_THS_Z1	 0x34
#define 	LSM303C_ACC_IG_DUR1		 0x35
#define 	LSM303C_ACC_IG_CFG2		 0x36
#define 	LSM303C_ACC_IG_SRC2		 0x37
#define 	LSM303C_ACC_IG_THS2		 0x38
#define 	LSM303C_ACC_IG_DUR2		 0x39
#define 	LSM303C_ACC_XL_REFERENCE		 0x3A
#define 	LSM303C_ACC_XH_REFERENCE		 0x3B
#define 	LSM303C_ACC_YL_REFERENCE		 0x3C
#define 	LSM303C_ACC_YH_REFERENCE		 0x3D
#define 	LSM303C_ACC_ZL_REFERENCE		 0x3E
#define 	LSM303C_ACC_ZH_REFERENCE		 0x3F
/* Exported macro ------------------------------------------------------------*/

#ifndef __SHARED__MACROS

#define __SHARED__MACROS
#define ValBit(VAR,Place)         (VAR & (1<<Place))
#define BIT(x) ( (x) )

#endif /*__SHARED__MACROS*/

/* Exported constants --------------------------------------------------------*/

#ifndef __SHARED__CONSTANTS
#define __SHARED__CONSTANTS

#define MEMS_SET                                        0x01
#define MEMS_RESET                                      0x00

#endif /*__SHARED__CONSTANTS*/

/* Exported functions --------------------------------------------------------*/
/**********Sensor Configuration Functions***********/
status_t LSM303C_ACC_SetODR(void *handle, LSM303C_ACC_ODR_t ov);
status_t LSM303C_ACC_SetFullScale(void *handle, LSM303C_ACC_FS_t ov);
status_t LSM303C_ACC_EnableAxis(void *handle, u8_t ov);

/***************Filtering Functions****************/
status_t LSM303C_ACC_HighPassFilterMode(void *handle, LSM303C_ACC_HPM_t ov);
status_t LSM303C_ACC_Select_Bandwidth(void *handle, LSM303C_ACC_BW_t ov);

/***************Interrupt Functions****************/
status_t LSM303C_ACC_EnableInterruptGeneratorOne(void *handle, LSM303C_ACC_IG_CONFIG_t ov);
status_t LSM303C_ACC_EnableInterruptGeneratorTwo(void *handle, LSM303C_ACC_IG_CONFIG_t ov);
status_t LSM303C_ACC_InterruptGeneratorOne_Wait(void *handle, LSM303C_ACC_WAIT_IG_t ov);
status_t LSM303C_ACC_InterruptGeneratorTwo_Wait(void *handle, LSM303C_ACC_WAIT_IG_t ov);
status_t LSM303C_ACC_InterruptGeneratorOne_LogicCondition(void *handle, LSM303C_ACC_AOI_IG_t ov);
status_t LSM303C_ACC_InterruptGeneratorTwo_LogicCondition(void *handle, LSM303C_ACC_AOI_IG_t ov);
status_t LSM303C_ACC_InterruptGeneratorOne_Flag(void *handle, LSM303C_ACC_IG_FLAGS_t *value);
status_t LSM303C_ACC_InterruptGeneratorTwo_Flag(void *handle, LSM303C_ACC_IG_FLAGS_t *value);

status_t LSM303C_ACC_IntPin_Mode(void *handle, LSM303C_ACC_INT_PIN_CFG_t ov);
status_t LSM303C_ACC_EnableInterruptPinOne(void *handle, LSM303C_ACC_INT1_DRDY_t ov);
status_t LSM303C_ACC_EnableInterruptPinTwo(void *handle, LSM303C_ACC_INT2_DRDY_t ov);
status_t LSM303C_ACC_InterruptSignalsMode(void *handle, LSM303C_ACC_LAT_SIG_t ov);
status_t LSM303C_ACC_ResetInterruptDuration(void *handle, LSM303C_ACC_RESET_DUR_t ov);
status_t LSM303C_ACC_Status_Flags(void *handle, u8_t *value);
status_t LSM303C_ACC_SetInterrupt1_Threshold_X(void *handle, u8_t buff);
status_t LSM303C_ACC_SetInterrupt1_Threshold_Y(void *handle, u8_t buff);
status_t LSM303C_ACC_SetInterrupt1_Threshold_Z(void *handle, u8_t buff);
status_t LSM303C_ACC_SetInterrupt2_Threshold_ZYX(void *handle, u8_t buff);
status_t LSM303C_ACC_SetInterrupt1_Duration(void *handle, u8_t buff);
status_t LSM303C_ACC_SetInterrupt2_Duration(void *handle, u8_t buff);
status_t LSM303C_ACC_SetReferenceValue(void *handle, u8_t* buff);
status_t LSM303C_ACC_GetReferenceValue(void *handle, u8_t* buff);

/*****************FIFO Functions******************/
status_t LSM303C_ACC_FIFO_Mode(void *handle, LSM303C_ACC_FMODE_t ov);
status_t LSM303C_ACC_FIFO_Flags(void *handle, LSM303C_ACC_FIFO_FLAGS_t *value);
status_t LSM303C_ACC_FIFO_StopAtTh(void *handle, LSM303C_ACC_STOP_FTH_t ov);
status_t LSM303C_ACC_FIFO(void *handle, LSM303C_ACC_FIFO_EN_t ov);
status_t LSM303C_ACC_SetFIFO_threshold(void *handle, u8_t buff);
status_t LSM303C_ACC_GetFIFO_StoredData(void *handle, u8_t buff);

/***************Utility Functions****************/
status_t LSM303C_ACC_BlockDataUpdate(void *handle, LSM303C_ACC_BDU_t ov);
status_t LSM303C_ACC_SetActivity_Duration(void *handle, u8_t buff);
status_t LSM303C_ACC_SetActivity_Threshold(void *handle, u8_t buff);
status_t LSM303C_ACC_SelfTest(void *handle, LSM303C_ACC_ST_t ov);
status_t LSM303C_ACC_AxOperativeMode(void *handle, LSM303C_ACC_HR_t ov);
status_t LSM303C_ACC_I2C_Mode(void *handle, LSM303C_ACC_I2C_t ov);
status_t LSM303C_ACC_AuotoInc(void *handle, LSM303C_ACC_IF_ADD_INC_t ov);
status_t LSM303C_ACC_SerialInterfaceMode(void *handle, LSM303C_ACC_SIM_t ov);
status_t LSM303C_ACC_DebugMode(void *handle, LSM303C_ACC_DEBUG_t ov);
status_t LSM303C_ACC_SoftReset(void *handle, LSM303C_ACC_SOFT_RESET_t ov);
status_t LSM303C_ACC_Reboot(void *handle);

/****************Reading Functions*****************/
status_t LSM303C_ACC_GetAccRaw(void *handle, u8_t* buff);
status_t LSM303C_ACC_GetTemperatureRaw(void *handle, u16_t* buff);


/*********************Generic*********************/
status_t LSM303C_ACC_ReadReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len);
status_t LSM303C_ACC_WriteReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __LSM303C_ACC_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/



