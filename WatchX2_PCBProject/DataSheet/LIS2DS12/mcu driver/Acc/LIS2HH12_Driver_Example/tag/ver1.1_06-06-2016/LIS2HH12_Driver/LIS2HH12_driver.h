/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
* File Name          : LIS2HH12_driver.h
* Author             : MEMS Application Team
* Version            : v1.1
* Date               : 06 June 2016  
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
#ifndef __LIS2HH12_DRIVER__H
#define __LIS2HH12_DRIVER__H

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

typedef enum {
  MEMS_ENABLE                            =             0x01,
  MEMS_DISABLE                           =             0x00
} LIS2HH12_ACC_State_t;

typedef enum { 
	LIS2HH12_DISABLE_ALL 		 = 0x00,
	LIS2HH12_X_ENABLE 		 = 0x01,
	LIS2HH12_Y_ENABLE 		 = 0x02,
	LIS2HH12_Z_ENABLE 		 = 0x04
} LIS2HH12_AXIS_EN_t;

typedef enum { 
	LIS2HH12_BDU_DISABLE 		 = 0x00,
	LIS2HH12_BDU_ENABLE 		 = 0x08
} LIS2HH12_BDU_t;

typedef enum { 
	LIS2HH12_ODR_POWER_DOWN 	 = 0x00,
	LIS2HH12_ODR_10_Hz 		 = 0x10,
	LIS2HH12_ODR_50_Hz 		 = 0x20,
	LIS2HH12_ODR_100_Hz 		 = 0x30,
	LIS2HH12_ODR_200_Hz 		 = 0x40,
	LIS2HH12_ODR_400_Hz 		 = 0x50,
	LIS2HH12_ODR_800_Hz 		 = 0x60,
	LIS2HH12_ODR_MASK 		 = 0x60
} LIS2HH12_ODR_t;

typedef enum { 
	LIS2HH12_HIGH_RES_OFF 		 = 0x00,
	LIS2HH12_HIGH_RES_ON	 		 = 0x80
} LIS2HH12_HR_t;

typedef enum { 
	LIS2HH12_HP_INT1_DIS	 		= 0x00,
	LIS2HH12_HP_INT1_EN		 	= 0x01
} LIS2HH12_HPIS_INT1_t;

typedef enum { 
	LIS2HH12_HP_INT2_DIS	 		= 0x00,
	LIS2HH12_HP_INT2_EN		 	= 0x02
} LIS2HH12_HPIS_INT2_t;

typedef enum { 
	LIS2HH12_FDS_BYPASS 		 = 0x00,
	LIS2HH12_FDS_FILTER 		 = 0x04
} LIS2HH12_FDS_t;
//Referring DS to HP filter Usage
typedef enum { 
	LIS2HH12_HPM_NORMAL_MODE 		 = 0x00,
	LIS2HH12_HPM_REF_FOR_FILT 		 = 0x08
} LIS2HH12_HPM_t;
//Referring DS to HP filter Usage
typedef enum { 
	LIS2HH12_DFC_ODR_DIV_50 		 = 0x00,
	LIS2HH12_DFC_ODR_DIV_100 		 = 0x20,
	LIS2HH12_DFC_ODR_DIV_9 		 = 0x40,
	LIS2HH12_DFC_ODR_DIV_400		 = 0x60
} LIS2HH12_DFC_t;

typedef enum { 
	LIS2HH12_INT1_DISABLE 		= 0x00,
	LIS2HH12_INT1_DRDY 		 	= 0x01,
	LIS2HH12_INT1_FIFO_TH 		= 0x02,
	LIS2HH12_INT1_FIFO_OVR 	 	= 0x00,
	LIS2HH12_INT1_INTGEN1 		= 0x08,
	LIS2HH12_INT1_INTGEN2 		= 0x10,
	LIS2HH12_INT1_INACTIVITY 		= 0x20
} LIS2HH12_INT1_DRDY_t;

typedef enum { 
	LIS2HH12_INT2_DISABLE 		= 0x00,
	LIS2HH12_INT2_DRDY 		 	= 0x01,
	LIS2HH12_INT2_FIFO_TH 		= 0x02,
	LIS2HH12_INT2_FIFO_EMPTY 	 	= 0x00,
	LIS2HH12_INT2_INTGEN1 		= 0x08,
	LIS2HH12_INT2_INTGEN2 		= 0x10,
	LIS2HH12_INT2_BOOT 			= 0x20
} LIS2HH12_INT2_DRDY_t;

typedef enum { 
	LIS2HH12_REBOOT				= 0x80
} LIS2HH12_INT2_INTGEN2_t;

typedef enum { 
	LIS2HH12_SIM_4WIRE_INTERFACE 		 = 0x00,
	LIS2HH12_SIM_3WIRE_INTERFACE 		 = 0x01
} LIS2HH12_SIM_t;

typedef enum { 
	LIS2HH12_I2C_ENABLE 		 = 0x00,
	LIS2HH12_I2C_DISABLE 		 = 0x02
} LIS2HH12_I2C_t;

typedef enum { 
	LIS2HH12_IF_ADD_INC_DISABLE 		 = 0x00,
	LIS2HH12_IF_ADD_INC_ENABLE 		 = 0x04
} LIS2HH12_IF_ADD_INC_t;

typedef enum { 
	LIS2HH12_FS_2g 		 = 0x00,
	LIS2HH12_FS_4g 		 = 0x20,
	LIS2HH12_FS_8g 		 = 0x30
} LIS2HH12_FS_t;


typedef enum { 
	LIS2HH12_BW_400_Hz 		 = 0x08,
	LIS2HH12_BW_200_Hz 		 = 0x48,
	LIS2HH12_BW_100_Hz 		 = 0x88,
	LIS2HH12_BW_50_Hz 		 = 0xC8
} LIS2HH12_BW_t;

typedef enum { 
	LIS2HH12_PUSH_PULL_ACTIVE_HIGH		 = 0x00,
	LIS2HH12_OPEN_DRAIN_ACTIVE_HIGH 	 	 = 0x01,
	LIS2HH12_PUSH_PULL_ACTIVE_LOW	 	 	 = 0x02,
	LIS2HH12_OPEN_DRAIN_ACTIVE_LOW 	 	 = 0x03
} LIS2HH12_INT_PIN_CFG_t;

typedef enum { 
	LIS2HH12_ST_DISABLE 			= 0x00,
	LIS2HH12_ST_POSITIVE 			= 0x04,
	LIS2HH12_ST_NEGATIVE 		 	= 0x08,
	LIS2HH12_ST_NA 		 		= 0x0C
} LIS2HH12_ST_t;

typedef enum { 
	LIS2HH12_DEC_DISABLE 		 = 0x00,
	LIS2HH12_DEC_2_SAMPLES 		 = 0x10,
	LIS2HH12_DEC_4_SAMPLES 		 = 0x20,
	LIS2HH12_DEC_8_SAMPLES 		 = 0x30
} LIS2HH12_DEC_t;

typedef enum { 
	LIS2HH12_SOFT_RESET_DISABLE 		 = 0x00,
	LIS2HH12_SOFT_RESET_ENABLE 		 = 0x40
} LIS2HH12_SOFT_RESET_t;

typedef enum { 
	LIS2HH12_DEBUG_DISABLE 		 = 0x00,
	LIS2HH12_DEBUG_ENABLE 		 = 0x80
} LIS2HH12_DEBUG_t;

typedef enum { 
	LIS2HH12_INT1_PULS_INT2_PULS	 		= 0x00,
	LIS2HH12_INT1_LAT_INT2_PULS 		 	= 0x04,
	LIS2HH12_INT1_PULS_INT2_LAT 		 	= 0x08,
	LIS2HH12_INT1_LAT_INT2_LAT 		 	= 0x0C
} LIS2HH12_LAT_SIG_t;

typedef enum { 
	LIS2HH12_DUR1_NORST_DUR2_NORST	 = 0x00,
	LIS2HH12_DUR1_RST_DUR2_NORST		 = 0x10,
	LIS2HH12_DUR1_NORST_DUR2_RST	 	 = 0x20,
	LIS2HH12_DUR1_RST_DUR2_RST		 = 0x30
} LIS2HH12_RESET_DUR_t;

//You can use this typedef with or condition 
typedef enum {
	LIS2HH12_IG1_ON_LOW_X_FLAG_UP		 	= 0x01,	
	LIS2HH12_IG1_ON_HIGH_X_FLAG_UP		= 0x02,
	LIS2HH12_IG1_ON_LOW_Y_FLAG_UP		 	= 0x04,
	LIS2HH12_IG1_ON_HIGH_Y_FLAG_UP		= 0x08,
	LIS2HH12_IG1_ON_LOW_Z_FLAG_UP		 	= 0x10,
	LIS2HH12_IG1_ON_HIGH_Z_FLAG_UP		= 0x20,
	LIS2HH12_IA_IG1_FLAG_UP 		 		= 0x40
} LIS2HH12_IG_FLAGS_t;

typedef enum { 
	LIS2HH12_X_NEW_DATA_AVAILABLE 		 = 0x01,
	LIS2HH12_Y_NEW_DATA_AVAILABLE 		 = 0x02,
	LIS2HH12_Z_NEW_DATA_AVAILABLE 		 = 0x04,
	LIS2HH12_ZYX_NEW_DATA_AVAILABLE 		 = 0x08,
	LIS2HH12_X_OVERRUN 		 = 0x10,
	LIS2HH12_Y_OVERRUN 		 = 0x20,
	LIS2HH12_Z_OVERRUN 		 = 0x40,
	LIS2HH12_ZYX_OVERRUN 		 = 0x80
} LIS2HH12_STATUS_FLAGS_t;

//You can use this typedef with or condition see DS 4D and 6D interrupt
typedef enum { 
	LIS2HH12_IG_DIS		 		 	= 0x00,
	LIS2HH12_IG_LOW_X	 			 	= 0x01,
	LIS2HH12_IG_HIGH_X	 		 	= 0x02,
	LIS2HH12_IG_LOW_Y_	 		 	= 0x04,
	LIS2HH12_IG_HIGH_Y	 			= 0x08,
	LIS2HH12_IG_LOW_Z		 			= 0x10,
	LIS2HH12_IG_HIGH_Z	 			= 0x20,
	LIS2HH12_IG_6D		 		 	= 0x40,	
	LIS2HH12_IG_4D		 		 	= 0x80	
} LIS2HH12_IG_CONFIG_t;

typedef enum { 
	LIS2HH12_4D_INTGEN1_DIS 		 = 0x00,
	LIS2HH12_4D_INTGEN1_EN 		 = 0x01
} LIS2HH12_4D_INTGEN1_t;

typedef enum {	
	LIS2HH12_4D_INTGEN2_DIS 		 = 0x00,
	LIS2HH12_4D_INTGEN2_EN 		 = 0x02
} LIS2HH12_4D_INTGEN2_t;

typedef enum { 
	LIS2HH12_AOI_IG_OR 		 		= 0x00,
	LIS2HH12_AOI_IG_AND 		 		= 0x80
} LIS2HH12_AOI_IG_t;

typedef enum { 
	LIS2HH12_WAIT_IG_OFF 		 			= 0x00,
	LIS2HH12_WAIT_IG_ON 		 			= 0x80
} LIS2HH12_WAIT_IG_t;

typedef enum { 
	LIS2HH12_FMODE_BYPASS 				 = 0x00,
	LIS2HH12_FMODE_STOP_WHEN_FULL 		 = 0x20,
	LIS2HH12_FMODE_STREAM 		 		 = 0x40,
	LIS2HH12_FMODE_STREAM_TO_FIFO 		 = 0x60,
	LIS2HH12_FMODE_BYPASS_TO_STREAM 		 = 0x80,
	LIS2HH12_FMODE_BYPASS_TO_FIFO 		 = 0xE0
} LIS2HH12_FMODE_t;

typedef enum { 
	LIS2HH12_FIFO_EMPTY_TRUE 		 = 0x20,
	LIS2HH12_OVR_FIFO_TRUE 		 = 0x40,
	LIS2HH12_FTH_PASS 			 = 0x80
} LIS2HH12_FIFO_FLAGS_t;

typedef enum { 
	LIS2HH12_STOP_FTH_DIS_FIFO_TH 	 = 0x00,
	LIS2HH12_STOP_FTH_EN_FIFO_TH 		 = 0x40
} LIS2HH12_STOP_FTH_t;

typedef enum { 
	LIS2HH12_FIFO_DISABLE 	 = 0x00,
	LIS2HH12_FIFO_ENABLE 		 = 0x80
} LIS2HH12_FIFO_EN_t;

#define 	LIS2HH12_WHO_AM_I		 0x41
#define 	LIS2HH12_MEMS_I2C_ADDRESS  0x3A

/* Registers Name ------------------------------------------------------------------------*/

#define 	LIS2HH12_TEMP_L		 0x0B
#define 	LIS2HH12_TEMP_H		 0x0C
#define 	LIS2HH12_ACT_TSH		 0x1E
#define 	LIS2HH12_ACT_DUR		 0x1F
#define 	LIS2HH12_WHO_AM_I_REG	 0x0F
#define 	LIS2HH12_CTRL1		 0x20
#define 	LIS2HH12_CTRL2		 0x21
#define 	LIS2HH12_CTRL3		 0x22
#define 	LIS2HH12_CTRL4		 0x23
#define 	LIS2HH12_CTRL5		 0x24
#define 	LIS2HH12_CTRL6		 0x25
#define 	LIS2HH12_CTRL7		 0x26
#define 	LIS2HH12_STATUS		 0x27
#define 	LIS2HH12_OUT_X_L		 0x28
#define 	LIS2HH12_OUT_X_H		 0x29
#define 	LIS2HH12_OUT_Y_L		 0x2A
#define 	LIS2HH12_OUT_Y_H		 0x2B
#define 	LIS2HH12_OUT_Z_L		 0x2C
#define 	LIS2HH12_OUT_Z_H		 0x2D
#define 	LIS2HH12_FIFO_CTRL	 0x2E
#define 	LIS2HH12_FIFO_SRC		 0x2F
#define 	LIS2HH12_IG_CFG1		 0x30
#define 	LIS2HH12_IG_SRC1		 0x31
#define 	LIS2HH12_IG_THS_X1	 0x32
#define 	LIS2HH12_IG_THS_Y1	 0x33
#define 	LIS2HH12_IG_THS_Z1	 0x34
#define 	LIS2HH12_IG_DUR1		 0x35
#define 	LIS2HH12_IG_CFG2		 0x36
#define 	LIS2HH12_IG_SRC2		 0x37
#define 	LIS2HH12_IG_THS2		 0x38
#define 	LIS2HH12_IG_DUR2		 0x39
#define 	LIS2HH12_XL_REFERENCE		 0x3A
#define 	LIS2HH12_XH_REFERENCE		 0x3B
#define 	LIS2HH12_YL_REFERENCE		 0x3C
#define 	LIS2HH12_YH_REFERENCE		 0x3D
#define 	LIS2HH12_ZL_REFERENCE		 0x3E
#define 	LIS2HH12_ZH_REFERENCE		 0x3F
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
status_t LIS2HH12_SetODR(void *handle, LIS2HH12_ODR_t ov);
status_t LIS2HH12_SetFullScale(void *handle, LIS2HH12_FS_t ov);
status_t LIS2HH12_EnableAxis(void *handle, u8_t ov);

/***************Filtering Functions****************/
status_t LIS2HH12_HighPassFilterMode(void *handle, LIS2HH12_HPM_t ov);
status_t LIS2HH12_Select_Bandwidth(void *handle, LIS2HH12_BW_t ov);

/***************Interrupt Functions****************/
status_t LIS2HH12_EnableInterruptGeneratorOne(void *handle, LIS2HH12_IG_CONFIG_t ov);
status_t LIS2HH12_EnableInterruptGeneratorTwo(void *handle, LIS2HH12_IG_CONFIG_t ov);
status_t LIS2HH12_InterruptGeneratorOne_Wait(void *handle, LIS2HH12_WAIT_IG_t ov);
status_t LIS2HH12_InterruptGeneratorTwo_Wait(void *handle, LIS2HH12_WAIT_IG_t ov);
status_t LIS2HH12_InterruptGeneratorOne_LogicCondition(void *handle, LIS2HH12_AOI_IG_t ov);
status_t LIS2HH12_InterruptGeneratorTwo_LogicCondition(void *handle, LIS2HH12_AOI_IG_t ov);
status_t LIS2HH12_InterruptGeneratorOne_Flag(void *handle, LIS2HH12_IG_FLAGS_t *value);
status_t LIS2HH12_InterruptGeneratorTwo_Flag(void *handle, LIS2HH12_IG_FLAGS_t *value);

status_t LIS2HH12_IntPin_Mode(void *handle, LIS2HH12_INT_PIN_CFG_t ov);
status_t LIS2HH12_EnableInterruptPinOne(void *handle, LIS2HH12_INT1_DRDY_t ov);
status_t LIS2HH12_EnableInterruptPinTwo(void *handle, LIS2HH12_INT2_DRDY_t ov);
status_t LIS2HH12_InterruptSignalsMode(void *handle, LIS2HH12_LAT_SIG_t ov);
status_t LIS2HH12_ResetInterruptDuration(void *handle, LIS2HH12_RESET_DUR_t ov);
status_t LIS2HH12_Status_Flags(void *handle, u8_t *value);
status_t LIS2HH12_SetInterrupt1_Threshold_X(void *handle, u8_t buff);
status_t LIS2HH12_SetInterrupt1_Threshold_Y(void *handle, u8_t buff);
status_t LIS2HH12_SetInterrupt1_Threshold_Z(void *handle, u8_t buff);
status_t LIS2HH12_SetInterrupt2_Threshold_ZYX(void *handle, u8_t buff);
status_t LIS2HH12_SetInterrupt1_Duration(void *handle, u8_t buff);
status_t LIS2HH12_SetInterrupt2_Duration(void *handle, u8_t buff);
status_t LIS2HH12_SetReferenceValue(void *handle, u8_t *buff);
status_t LIS2HH12_GetReferenceValue(void *handle, u8_t *buff);

/*****************FIFO Functions******************/
status_t LIS2HH12_FIFO_Mode(void *handle, LIS2HH12_FMODE_t ov);
status_t LIS2HH12_FIFO_Flags(void *handle, LIS2HH12_FIFO_FLAGS_t *value);
status_t LIS2HH12_FIFO_StopAtTh(void *handle, LIS2HH12_STOP_FTH_t ov);
status_t LIS2HH12_FIFO(void *handle, LIS2HH12_FIFO_EN_t ov);
status_t LIS2HH12_SetFIFO_threshold(void *handle, u8_t buff);
status_t LIS2HH12_GetFIFO_StoredData(void *handle, u8_t buff);

/***************Utility Functions****************/
status_t LIS2HH12_BlockDataUpdate(void *handle, LIS2HH12_BDU_t ov);
status_t LIS2HH12_SetActivity_Duration(void *handle, u8_t buff);
status_t LIS2HH12_SetActivity_Threshold(void *handle, u8_t buff);
status_t LIS2HH12_SelfTest(void *handle, LIS2HH12_ST_t ov);
status_t LIS2HH12_AxOperativeMode(void *handle, LIS2HH12_HR_t ov);
status_t LIS2HH12_I2C_Mode(void *handle, LIS2HH12_I2C_t ov);
status_t LIS2HH12_AuotoInc(void *handle, LIS2HH12_IF_ADD_INC_t ov);
status_t LIS2HH12_SerialInterfaceMode(void *handle, LIS2HH12_SIM_t ov);
status_t LIS2HH12_DebugMode(void *handle, LIS2HH12_DEBUG_t ov);
status_t LIS2HH12_SoftReset(void *handle, LIS2HH12_SOFT_RESET_t ov);
status_t LIS2HH12_Reboot(void *handle);

/****************Reading Functions*****************/
status_t LIS2HH12_GetAccRaw(void *handle, u8_t* buff);
status_t LIS2HH12_GetTemperatureRaw(void *handle, u16_t* buff);


/*********************Generic*********************/
u8_t LIS2HH12_ReadReg(void *handle, u8_t Reg, u8_t* Data);
u8_t LIS2HH12_WriteReg(void *handle, u8_t Reg, u8_t Data);


#endif /* __LIS2HH12_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/



