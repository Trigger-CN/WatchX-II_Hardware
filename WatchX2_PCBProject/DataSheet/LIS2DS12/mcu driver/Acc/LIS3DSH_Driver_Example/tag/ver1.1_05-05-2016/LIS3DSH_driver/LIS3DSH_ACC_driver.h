/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
* File Name          : LIS3DSH_ACC_driver.h
* Author             : MEMS Application Team
* Version            : v1.1
* Date               : 05 Maggio 2016   
* Description        : LIS3DSH Platform Independent Driver
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
#ifndef __LIS3DSH_DRIVER__H
#define __LIS3DSH_DRIVER__H

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

//define structure
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
  MEMS_SUCCESS                  =		0x01,
  MEMS_ERROR			=		0x00	
} status_t;

#endif /*__SHARED__TYPES*/

typedef u8_t LIS3DSH_ACC_Axis_t;
typedef u8_t  LIS3DSH_ACC_Int1Conf_t;

typedef enum {
  LIS3DSH_ACC_MEMS_ENABLE                            =             0x01,
  LIS3DSH_ACC_MEMS_DISABLE                           =             0x00
} LIS3DSH_ACC_State_t;

typedef enum {
  LIS3DSH_ACC_POL_HIGH			=		0x01,
  LIS3DSH_ACC_POL_LOW			=		0x00	
} LIS3DSH_ACC_Polarity_t;

typedef enum { 
  LIS3DSH_ACC_POWER_DOWN			=		0x00,
  LIS3DSH_ACC_ODR_3_125			=		0x01,		
  LIS3DSH_ACC_ODR_6_25Hz			=		0x02,
  LIS3DSH_ACC_ODR_12_5Hz		        =		0x03,
  LIS3DSH_ACC_ODR_25Hz		        =		0x04,
  LIS3DSH_ACC_ODR_50Hz		        =		0x05,	
  LIS3DSH_ACC_ODR_100Hz		        =		0x06,
  LIS3DSH_ACC_ODR_400Hz		        =		0x07,
  LIS3DSH_ACC_ODR_800Hz		        =		0x08,
  LIS3DSH_ACC_ODR_1600Hz			=		0x09	
} LIS3DSH_ACC_ODR_t;

typedef enum { 
  LIS3DSH_ACC_BANDWIDTH_800Hz		=		0x00,
  LIS3DSH_ACC_BANDWIDTH_400Hz		=		0x01,		
  LIS3DSH_ACC_BANDWIDTH_200Hz		=		0x02,
  LIS3DSH_ACC_BANDWIDTH_50Hz	        =		0x03
} LIS3DSH_ACC_BandWidth_t;

typedef enum {
  LIS3DSH_ACC_SET_AXIS_X			=		0x01,
  LIS3DSH_ACC_SET_AXIS_Y			=		0x02,
  LIS3DSH_ACC_SET_AXIS_Z			=		0x03
} LIS3DSH_ACC_SET_AXIS_t;

typedef enum {
  LIS3DSH_ACC_SET_VFC_1			=		0x01,
  LIS3DSH_ACC_SET_VFC_2			=		0x02,
  LIS3DSH_ACC_SET_VFC_3			=		0x03,
  LIS3DSH_ACC_SET_VFC_4			=		0x04
} LIS3DSH_ACC_SET_VFC_t;  

typedef enum {
  LIS3DSH_ACC_THRS_1			=		0x01,
  LIS3DSH_ACC_THRS_2			=		0x02
} LIS3DSH_ACC_THRS_t;  

typedef enum {
  LIS3DSH_ACC_SM1				=		0x01,
  LIS3DSH_ACC_SM2				=		0x02
} LIS3DSH_ACC_SM_t; 

typedef enum {
  LIS3DSH_ACC_TIM_1				=		0x01,
  LIS3DSH_ACC_TIM_2				=		0x02,
  LIS3DSH_ACC_TIM_3				=		0x03,
  LIS3DSH_ACC_TIM_4				=		0x04   
} LIS3DSH_ACC_TIM_t; 

typedef enum {
  LIS3DSH_ACC_MASK_A			=		0x01,
  LIS3DSH_ACC_MASK_B			=		0x02 
} LIS3DSH_ACC_MASK_t;

typedef enum {
  LIS3DSH_ACC_FULLSCALE_2                   =               0x00,
  LIS3DSH_ACC_FULLSCALE_4                   =               0x01,
  LIS3DSH_ACC_FULLSCALE_6                   =               0x02,
  LIS3DSH_ACC_FULLSCALE_8                   =               0x03,
  LIS3DSH_ACC_FULLSCALE_16                  =               0x04
} LIS3DSH_ACC_Fullscale_t;

typedef enum {
  LIS3DSH_ACC_BLE_LSB			=		0x00,
  LIS3DSH_ACC_BLE_MSB			=		0x01
} LIS3DSH_ACC_Endianess_t;


typedef enum {
  LIS3DSH_ACC_SELF_TEST_NORMAL		=               0x00,
  LIS3DSH_ACC_SELF_TEST_POSITIVE		=               0x01,
  LIS3DSH_ACC_SELF_TEST_NEGATIVE		=               0x02
} LIS3DSH_ACC_SelfTest_t;


typedef enum {
  LIS3DSH_ACC_FIFO_BYPASS_MODE              =               0x00,
  LIS3DSH_ACC_FIFO_MODE                     =               0x01,
  LIS3DSH_ACC_FIFO_STREAM_MODE              =               0x02,
  LIS3DSH_ACC_FIFO_STREAM_TRIGGER_MODE	=               0x03,
  LIS3DSH_ACC_FIFO_BYPASS_THAN_STREAM	=		0x04,
  LIS3DSH_ACC_FIFO_BYPASS_THAN_FIFO		=		0x07
} LIS3DSH_ACC_FifoMode_t;

typedef enum {
  LIS3DSH_ACC_TRIG_INT1                     =		0x00,
  LIS3DSH_ACC_TRIG_INT2 			=		0x01
} LIS3DSH_ACC_TrigInt_t;

typedef enum {
  LIS3DSH_ACC_SPI_4_WIRE                    =               0x00,
  LIS3DSH_ACC_SPI_3_WIRE                    =               0x01
} LIS3DSH_ACC_SPIMode_t;

typedef enum {
  LIS3DSH_ACC_X_ENABLE                      =               0x01,
  LIS3DSH_ACC_X_DISABLE                     =               0x00,
  LIS3DSH_ACC_Y_ENABLE                      =               0x02,
  LIS3DSH_ACC_Y_DISABLE                     =               0x00,
  LIS3DSH_ACC_Z_ENABLE                      =               0x04,
  LIS3DSH_ACC_Z_DISABLE                     =               0x00    
} LIS3DSH_ACC_AXISenable_t;


/* Exported constants --------------------------------------------------------*/

#ifndef __SHARED__CONSTANTS
#define __SHARED__CONSTANTS

#define MEMS_SET                                0x01
#define MEMS_RESET                              0x00

#endif /*__SHARED__CONSTANTS*/

// I2C Address
#define LIS3DSH_ACC_I2C_ADDRESS_LOW                0x3C  	//SEL=0
#define LIS3DSH_ACC_I2C_ADDRESS_HIGH               0x3A  	//SEL=1

//Register info
#define LIS3DSH_ACC_OUT_T					0x0C
#define LIS3DSH_ACC_INFO_1					0x0D
#define LIS3DSH_ACC_INFO_2					0x0E
#define LIS3DSH_ACC_WHO_AM_I				0x0F	// device identification register

#define LIS3DSH_ACC_I_AM_LIS3DSH			        0x3F

//offset corrction register
#define LIS3DSH_ACC_OFF_X					0x10
#define LIS3DSH_ACC_OFF_Y					0x11
#define LIS3DSH_ACC_OFF_Z					0x12

//Constant shift register
#define LIS3DSH_ACC_CS_X					0x13
#define LIS3DSH_ACC_CS_Y					0x14
#define LIS3DSH_ACC_CS_Z					0x15

//Long counter register
#define LIS3DSH_ACC_LC_L					0x16
#define LIS3DSH_ACC_LC_H					0x17

//STAT_REG
#define LIS3DSH_ACC_STAT					0x18
#define LIS3DSH_ACC_LONG					BIT(7)
#define LIS3DSH_ACC_SYNCW					BIT(6)
#define LIS3DSH_ACC_SYNC1					BIT(5)
#define LIS3DSH_ACC_SYNC2					BIT(4)
#define LIS3DSH_ACC_INT_SM1					BIT(3)
#define LIS3DSH_ACC_INT_SM2					BIT(2)
#define LIS3DSH_ACC_DOR					BIT(1)
#define LIS3DSH_ACC_DRDY					BIT(0)

//Peack detection value for SM1/SM2
#define LIS3DSH_ACC_PEAK1					0x19
#define LIS3DSH_ACC_PEAK2					0x1A

//Vector coefficient Register
#define LIS3DSH_ACC_VFC_1					0x1B
#define LIS3DSH_ACC_VFC_2					0x1C
#define LIS3DSH_ACC_VFC_3					0x1D
#define LIS3DSH_ACC_VFC_4					0x1E

//threshold register
#define LIS3DSH_ACC_THRS3					0X1F

//CONTROL REGISTER 1
#define LIS3DSH_ACC_CNTL4					0x20
#define LIS3DSH_ACC_ODR_BIT				        BIT(4)
#define LIS3DSH_ACC_BDU					BIT(3)
#define LIS3DSH_ACC_ZEN					BIT(2)
#define LIS3DSH_ACC_YEN					BIT(1)
#define LIS3DSH_ACC_XEN					BIT(0)


//CONTROL REGISTER for SM1 / SM2
#define LIS3DSH_ACC_CNTL1					0x21
#define LIS3DSH_ACC_CNTL2					0x22
#define LIS3DSH_ACC_HYST     				BIT(7)
#define LIS3DSH_ACC_SM_PIN					BIT(3)
#define LIS3DSH_ACC_SM_EN					BIT(0)

//CONTROL REGISTER 3
#define LIS3DSH_ACC_CNTL3					0x23
#define LIS3DSH_ACC_DR_EN					BIT(7)
#define LIS3DSH_ACC_IEA					BIT(6)
#define LIS3DSH_ACC_IEL				        BIT(5)
#define LIS3DSH_ACC_INT2_EN					BIT(4)
#define LIS3DSH_ACC_INT1_EN					BIT(3)
#define LIS3DSH_ACC_VFILT					BIT(2)
#define LIS3DSH_ACC_STRT					BIT(0)

//CONTROL REGISTER 5
#define LIS3DSH_ACC_CNTL5					0x24
#define LIS3DSH_ACC_BW					BIT(7)
#define LIS3DSH_ACC_FSCALE					BIT(5)
#define LIS3DSH_ACC_ST					BIT(2)
#define LIS3DSH_ACC_SIM					BIT(0)

//CONTROL REGISTER 6
#define LIS3DSH_ACC_CNTL6					0x25
#define LIS3DSH_ACC_BOOT					BIT(7)
#define LIS3DSH_ACC_FIFO_EN					BIT(6)
#define LIS3DSH_ACC_WTM_EN					BIT(5)
#define LIS3DSH_ACC_ADD_INC					BIT(4)
#define LIS3DSH_ACC_I1_EMPTY				BIT(3)
#define LIS3DSH_ACC_I1_WTM					BIT(2)
#define LIS3DSH_ACC_I1_OVERRUN				BIT(1)
#define LIS3DSH_ACC_I2_BOOT					BIT(0)

//STATUS_REG_AXIES
#define LIS3DSH_ACC_STATUS					0x27
#define LIS3DSH_ACC_ZYXOR                                   BIT(7)
#define LIS3DSH_ACC_ZOR                                     BIT(6)
#define LIS3DSH_ACC_YOR                                     BIT(5)
#define LIS3DSH_ACC_XOR                                     BIT(4)
#define LIS3DSH_ACC_ZYXDA                                   BIT(3)
#define LIS3DSH_ACC_ZDA                                     BIT(2)
#define LIS3DSH_ACC_YDA                                     BIT(1)
#define LIS3DSH_ACC_XDA                                     BIT(0)

//OUTPUT ACCELERATION REGISTER
#define LIS3DSH_ACC_OUT_X_L					0x28
#define LIS3DSH_ACC_OUT_X_H					0x29
#define LIS3DSH_ACC_OUT_Y_L					0x2A
#define LIS3DSH_ACC_OUT_Y_H					0x2B
#define LIS3DSH_ACC_OUT_Z_L					0x2C
#define LIS3DSH_ACC_OUT_Z_H					0x2D

//FIFO CONTROL REGISTER
#define LIS3DSH_ACC_FIFO_CTRL				0x2E
#define LIS3DSH_ACC_FMODE					BIT(5)
#define LIS3DSH_ACC_WTMP					BIT(0)

//FIFO REGISTERS
#define LIS3DSH_ACC_FIFO_SRC			        0x2F

// State machine code register value for SM1
#define LIS3DSH_ACC_ST1_1					0x40
#define LIS3DSH_ACC_ST2_1					0x41
#define LIS3DSH_ACC_ST3_1					0x42
#define LIS3DSH_ACC_ST4_1					0x43
#define LIS3DSH_ACC_ST5_1					0x44
#define LIS3DSH_ACC_ST6_1					0x45
#define LIS3DSH_ACC_ST7_1					0x46
#define LIS3DSH_ACC_ST8_1					0x47
#define LIS3DSH_ACC_ST9_1					0x48
#define LIS3DSH_ACC_ST10_1					0x49
#define LIS3DSH_ACC_ST11_1					0x4A
#define LIS3DSH_ACC_ST12_1					0x4B
#define LIS3DSH_ACC_ST13_1					0x4C
#define LIS3DSH_ACC_ST14_1					0x4D
#define LIS3DSH_ACC_ST15_1					0x4E
#define LIS3DSH_ACC_ST16_1					0x4F
  
// State machine code register value for SM2
#define LIS3DSH_ACC_ST1_2					0x60
#define LIS3DSH_ACC_ST2_2					0x61
#define LIS3DSH_ACC_ST3_2					0x62
#define LIS3DSH_ACC_ST4_2					0x63
#define LIS3DSH_ACC_ST5_2					0x64
#define LIS3DSH_ACC_ST6_2					0x65
#define LIS3DSH_ACC_ST7_2					0x66
#define LIS3DSH_ACC_ST8_2					0x67
#define LIS3DSH_ACC_ST9_2					0x68
#define LIS3DSH_ACC_ST10_2					0x69
#define LIS3DSH_ACC_ST11_2					0x6A
#define LIS3DSH_ACC_ST12_2					0x6B
#define LIS3DSH_ACC_ST13_2					0x6C
#define LIS3DSH_ACC_ST14_2					0x6D
#define LIS3DSH_ACC_ST15_2					0x6E
#define LIS3DSH_ACC_ST16_2					0x6F

//General timer for SM1
#define LIS3DSH_ACC_TIM4_1					0x50
#define LIS3DSH_ACC_TIM3_1					0x51

//16bit unsigned value timer for SM1
#define LIS3DSH_ACC_TIM2_1_L				0x52
#define LIS3DSH_ACC_TIM2_1_H				0x53
#define LIS3DSH_ACC_TIM1_1_L				0x54
#define LIS3DSH_ACC_TIM1_1_H				0x55

//threshold signed value for SM1
#define LIS3DSH_ACC_THRS2_1					0X56
#define LIS3DSH_ACC_THRS1_1					0X57

//axis and sign mask for SM1 motion direction operation
#define LIS3DSH_ACC_MASKB_1					0x59
//axis and sign mask for SM2 motion direction operation
#define LIS3DSH_ACC_MASKA_1					0x5A
#define LIS3DSH_ACC_P_X					BIT(7)
#define LIS3DSH_ACC_N_X					BIT(6)
#define LIS3DSH_ACC_P_Y					BIT(5)
#define LIS3DSH_ACC_N_Y					BIT(4)
#define LIS3DSH_ACC_P_Z					BIT(3)
#define LIS3DSH_ACC_N_Z					BIT(2)
#define LIS3DSH_ACC_P_V					BIT(1)
#define LIS3DSH_ACC_N_V					BIT(0)

//threshold, peak settings for SM1 / SM2
#define LIS3DSH_ACC_SETT1					0x5B
#define LIS3DSH_ACC_SETT2					0x7B
#define LIS3DSH_ACC_P_DET					BIT(7)
#define LIS3DSH_ACC_THR3_SA					BIT(6)
#define	LIS3DSH_ACC_ABS					BIT(5)
#define LIS3DSH_ACC_RADI					BIT(4) //only SETT2
#define LIS3DSH_ACC_D_CS					BIT(3) //only SETT2
#define LIS3DSH_ACC_THR3_MA					BIT(2)
#define LIS3DSH_ACC_R_TAM					BIT(1)
#define LIS3DSH_ACC_SITR					BIT(0)

//program and reset pointer for SM1/SM2
#define LIS3DSH_ACC_PR1					0x5C
#define LIS3DSH_ACC_PR2					0x7C
#define LIS3DSH_ACC_PP					BIT(4)      
#define LIS3DSH_ACC_RP					BIT(0)     

//General timer for SM2
#define LIS3DSH_ACC_TIM4_2					0x70
#define LIS3DSH_ACC_TIM3_2					0x71
//16bit unsigned value timer for SM2
#define LIS3DSH_ACC_TIM2_2_L				0x72
#define LIS3DSH_ACC_TIM2_2_H				0x73
#define LIS3DSH_ACC_TIM1_2_L				0x74
#define LIS3DSH_ACC_TIM1_2_H				0x75

//threshold signed value for SM1
#define LIS3DSH_ACC_THRS2_2					0x76
#define LIS3DSH_ACC_THRS1_2					0x77

//decimation counter value for SM2
#define LIS3DSH_ACC_DES2					0x78

//axis and sign mask for SM1 motion direction operation
#define LIS3DSH_ACC_MASKB_2					0x79
//axis and sign mask for SM1 motion direction operation
#define LIS3DSH_ACC_MASKA_2					0x7A
#define LIS3DSH_ACC_P_X					BIT(7)
#define LIS3DSH_ACC_N_X					BIT(6)
#define LIS3DSH_ACC_P_Y					BIT(5)
#define LIS3DSH_ACC_N_Y					BIT(4)
#define LIS3DSH_ACC_P_Z					BIT(3)
#define LIS3DSH_ACC_N_Z					BIT(2)
#define LIS3DSH_ACC_P_V					BIT(1)
#define LIS3DSH_ACC_N_V					BIT(0)

//General Timer 16 bit for SM1/SM2
#define LIS3DSH_ACC_TC1_L					0x5D
#define LIS3DSH_ACC_TC1_H					0x5E
#define LIS3DSH_ACC_TC2_L					0x7D
#define LIS3DSH_ACC_TC2_H					0x7E

// output flags on axis for interrupt SM1/SM2
#define LIS3DSH_ACC_OUTS1					0x5F
#define LIS3DSH_ACC_OUTS2					0x7F

//mask Stat register flag
#define LIS3DSH_ACC_F_LONG					0x80
#define LIS3DSH_ACC_F_SYNCW					0x40
#define LIS3DSH_ACC_F_SYNC1					0x20
#define LIS3DSH_ACC_F_SYNC2					0x10
#define LIS3DSH_ACC_F_INT_SM1				0x08
#define LIS3DSH_ACC_F_INT_SM2				0x04
#define LIS3DSH_ACC_F_DOR					0x02
#define LIS3DSH_ACC_F_DRDY					0x01

//STATUS REGISTER bit mask
#define LIS3DSH_ACC_STATUS_REG_ZYXOR                        0x80    // 1	:	new data set has over written the previous one
							// 0	:	no overrun has occurred (default)	
#define LIS3DSH_ACC_STATUS_REG_ZOR                          0x40    // 0	:	no overrun has occurred (default)
							// 1	:	new Z-axis data has over written the previous one
#define LIS3DSH_ACC_STATUS_REG_YOR                          0x20    // 0	:	no overrun has occurred (default)
							// 1	:	new Y-axis data has over written the previous one
#define LIS3DSH_ACC_STATUS_REG_XOR                          0x10    // 0	:	no overrun has occurred (default)
							// 1	:	new X-axis data has over written the previous one
#define LIS3DSH_ACC_STATUS_REG_ZYXDA                        0x08    // 0	:	a new set of data is not yet avvious one
                                                        // 1	:	a new set of data is available 
#define LIS3DSH_ACC_STATUS_REG_ZDA                          0x04    // 0	:	a new data for the Z-Axis is not availvious one
                                                        // 1	:	a new data for the Z-Axis is available
#define LIS3DSH_ACC_STATUS_REG_YDA                          0x02    // 0	:	a new data for the Y-Axis is not available
                                                        // 1	:	a new data for the Y-Axis is available
#define LIS3DSH_ACC_STATUS_REG_XDA                          0x01    // 0	:	a new data for the X-Axis is not available

#define LIS3DSH_ACC_DATAREADY_BIT                           STATUS_REG_ZYXDA

//bit mask for out flag for interrupt
#define LIS3DSH_ACC_F_P_X					0x80
#define LIS3DSH_ACC_F_N_X					0x40
#define LIS3DSH_ACC_F_P_Y					0x20
#define LIS3DSH_ACC_F_N_Y					0x10
#define LIS3DSH_ACC_F_P_Z					0x08
#define LIS3DSH_ACC_F_N_Z					0x04
#define LIS3DSH_ACC_F_P_V					0x02
#define LIS3DSH_ACC_F_N_V					0x01

//bit mask of FIFO SOURCE
#define LIS3DSH_ACC_FIFO_WTM_S				0x80
#define LIS3DSH_ACC_FIFO_OVRN_S				0x40
#define LIS3DSH_ACC_FIFO_EMPTY_S				0x20
#define	LIS3DSH_ACC_FIFO_STORED_S				0x1F



/* Exported macro ------------------------------------------------------------*/
#define ValBit(VAR,Place)         (VAR & (1<<Place))
#define BIT(x) ( (x) )

/* Exported functions --------------------------------------------------------*/

//Generic
status_t LIS3DSH_ACC_WriteReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len);
status_t LIS3DSH_ACC_ReadReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len);

//Sensor Configuration Functions
status_t LIS3DSH_ACC_SetODR(void *handle, LIS3DSH_ACC_ODR_t ov);
status_t LIS3DSH_ACC_SetAxis(void *handle, LIS3DSH_ACC_Axis_t axis);
status_t LIS3DSH_ACC_SetFullScale(void *handle, LIS3DSH_ACC_Fullscale_t fs);
status_t LIS3DSH_ACC_ReBootEnable(void *handle, LIS3DSH_ACC_State_t boot);
status_t LIS3DSH_ACC_AddIncEnable(void *handle, LIS3DSH_ACC_State_t addinc);
status_t LIS3DSH_ACC_BootInt2(void *handle, LIS3DSH_ACC_State_t booti2);
status_t LIS3DSH_ACC_SetBDU(void *handle, LIS3DSH_ACC_State_t bdu);
status_t LIS3DSH_ACC_SetSelfTest(void *handle, LIS3DSH_ACC_SelfTest_t st);
status_t LIS3DSH_ACC_SoftReset(void *handle, LIS3DSH_ACC_State_t strt);
status_t LIS3DSH_ACC_SetOFFSET(void *handle, LIS3DSH_ACC_SET_AXIS_t axis, u8_t val); 
status_t LIS3DSH_ACC_SetCS(void *handle, LIS3DSH_ACC_SET_AXIS_t axis, u8_t val);
status_t LIS3DSH_ACC_GetStatBIT(void *handle, u8_t StatBITMask);

//threshold
status_t LIS3DSH_ACC_GetThrs3(void *handle, u8_t* val);
status_t LIS3DSH_ACC_SetThrs3(void *handle, u8_t val);
status_t LIS3DSH_ACC_SetThrsSM(void *handle, LIS3DSH_ACC_SM_t sm, LIS3DSH_ACC_THRS_t thrs, u8_t val);
status_t LIS3DSH_ACC_GetThrsSM(void *handle, LIS3DSH_ACC_SM_t sm, LIS3DSH_ACC_THRS_t thrs, u8_t* val); 

//state machine
status_t LIS3DSH_ACC_SetSMCodeReg(void *handle, u8_t CodeADD, u8_t CodeByte);
status_t LIS3DSH_ACC_SetSMBufferCodeReg(void *handle, LIS3DSH_ACC_SM_t sm, u8_t* CodeBuff);
status_t LIS3DSH_ACC_GetSMCodeRegister(void *handle, LIS3DSH_ACC_SM_t sm, u8_t RegNumber, u8_t* val);
status_t LIS3DSH_ACC_SetLC(void *handle, u16_t val); 
status_t LIS3DSH_ACC_GetLC(void *handle, i16_t* val);
status_t LIS3DSH_ACC_GetResetPointSM(void *handle, LIS3DSH_ACC_SM_t sm, u8_t* val);
status_t LIS3DSH_ACC_GetProgramPointSM(void *handle, LIS3DSH_ACC_SM_t sm, u8_t* val);
status_t LIS3DSH_ACC_GetDecimSM2(void *handle, u8_t* val);
status_t LIS3DSH_ACC_SetTimerSM(void *handle, LIS3DSH_ACC_SM_t sm, LIS3DSH_ACC_TIM_t timer, u16_t val);
status_t LIS3DSH_ACC_SetMaskSM(void *handle, LIS3DSH_ACC_SM_t sm, LIS3DSH_ACC_MASK_t mask, u8_t val);
status_t LIS3DSH_ACC_GetTCSM(void *handle, LIS3DSH_ACC_SM_t sm, u16_t* val);
status_t LIS3DSH_ACC_GetPeakSM(void *handle, LIS3DSH_ACC_SM_t sm, u8_t* val);
status_t LIS3DSH_ACC_SetHystSM(void *handle, LIS3DSH_ACC_SM_t  sm, u8_t val);
status_t LIS3DSH_ACC_SetIntPinSM(void *handle, LIS3DSH_ACC_SM_t sm, LIS3DSH_ACC_State_t state);
status_t LIS3DSH_ACC_SetSitrSM(void *handle, LIS3DSH_ACC_SM_t sm, LIS3DSH_ACC_State_t state);
status_t LIS3DSH_ACC_SetIntEnaSM(void *handle, LIS3DSH_ACC_SM_t sm, LIS3DSH_ACC_State_t state);

//Filtering Functions
status_t LIS3DSH_ACC_BandWidth(void *handle, LIS3DSH_ACC_BandWidth_t bw);
status_t LIS3DSH_ACC_VectFiltEnable(void *handle, LIS3DSH_ACC_State_t vfe);
status_t LIS3DSH_ACC_SetVectorCoeff(void *handle, LIS3DSH_ACC_SET_VFC_t vfc, u8_t val);
status_t LIS3DSH_ACC_GetVectorCoeff(void *handle, LIS3DSH_ACC_SET_VFC_t vfc, u8_t* val);

//Interrupt Functions
status_t LIS3DSH_ACC_DataReadyInt(void *handle, LIS3DSH_ACC_State_t drdy);
status_t LIS3DSH_ACC_Int1Enable(void *handle, LIS3DSH_ACC_State_t conf);
status_t LIS3DSH_ACC_Int2Enable(void *handle, LIS3DSH_ACC_State_t conf);
status_t LIS3DSH_ACC_IntLatchEnable(void *handle, LIS3DSH_ACC_State_t latch);
status_t LIS3DSH_ACC_IntSignPol(void *handle, LIS3DSH_ACC_Polarity_t pol);
status_t LIS3DSH_ACC_ResetInt1Latch(void *handle);
status_t LIS3DSH_ACC_SetIntConfiguration(void *handle, LIS3DSH_ACC_Int1Conf_t ic);
status_t LIS3DSH_ACC_SetInt1Threshold(void *handle, u8_t ths);
status_t LIS3DSH_ACC_SetInt1Duration(void *handle, LIS3DSH_ACC_Int1Conf_t id);
status_t LIS3DSH_ACC_GetInt1Src(void *handle, u8_t* val);
status_t LIS3DSH_ACC_GetInt1SrcBit(void *handle, u8_t statusBIT);
status_t LIS3DSH_ACC_GetOutSBitSM(void *handle, LIS3DSH_ACC_SM_t sm, u8_t FLAG_INT_OUT);
status_t LIS3DSH_ACC_SetPeakDetSM(void *handle, LIS3DSH_ACC_SM_t sm, LIS3DSH_ACC_State_t state);
status_t LIS3DSH_ACC_SetThr3SaSM(void *handle, LIS3DSH_ACC_SM_t sm, LIS3DSH_ACC_State_t state);
status_t LIS3DSH_ACC_SetAbsSM(void *handle, LIS3DSH_ACC_SM_t sm, LIS3DSH_ACC_State_t state);
status_t LIS3DSH_ACC_SetRTamSM(void *handle, LIS3DSH_ACC_SM_t sm, LIS3DSH_ACC_State_t state);
status_t LIS3DSH_ACC_SetThr3MaSM(void *handle, LIS3DSH_ACC_SM_t sm, LIS3DSH_ACC_State_t state);
       
//FIFO Functions
status_t LIS3DSH_ACC_FIFOMode(void *handle, LIS3DSH_ACC_FifoMode_t fm);
status_t LIS3DSH_ACC_SetWaterMark(void *handle, u8_t wtm);
status_t LIS3DSH_ACC_SetTriggerInt(void *handle, LIS3DSH_ACC_TrigInt_t tr);
status_t LIS3DSH_ACC_GetFifoSourceReg(void *handle, u8_t* val);
status_t LIS3DSH_ACC_GetFifoSourceBit(void *handle, u8_t statusBIT);
status_t LIS3DSH_ACC_GetFifoSourceFSS(void *handle, u8_t* val);
status_t LIS3DSH_ACC_FIFOEnable(void *handle, LIS3DSH_ACC_State_t fifo, u8_t nMax);
status_t LIS3DSH_ACC_FifoEmptyInt1(void *handle, LIS3DSH_ACC_State_t empty);
status_t LIS3DSH_ACC_FifoOvrInt1(void *handle, LIS3DSH_ACC_State_t overrun);
status_t LIS3DSH_ACC_ReadFifoData(void *handle, Type3Axis16bit_U* FifoBuff, u8_t* depth);

//Other Reading Functions
status_t LIS3DSH_ACC_GetSatusReg(void *handle, u8_t* val);
status_t LIS3DSH_ACC_GetSatusBit(void *handle, u8_t statusBIT);
status_t LIS3DSH_ACC_GetAccAxesRaw(void *handle, Type3Axis16bit_U* buff);
status_t LIS3DSH_ACC_GetWHO_AM_I(void *handle, u8_t* val);
status_t LIS3DSH_ACC_GetOUT_T(void *handle, u8_t* val);

#endif /* __LIS3DSH_DRIVER__H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/



