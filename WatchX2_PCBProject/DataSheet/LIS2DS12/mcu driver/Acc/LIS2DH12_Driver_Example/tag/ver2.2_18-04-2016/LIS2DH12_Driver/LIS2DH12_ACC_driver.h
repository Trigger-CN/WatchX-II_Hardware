/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
* File Name          : LIS2DH12_ACC_driver.h
* Author             : MEMS Application Team
* Version            : v2.2
* Date               : 18 April 2016    
* Description        : LIS2DH12 Platform Independent Driver - header file
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
#ifndef __LIS2DH12_DRIVER__H
#define __LIS2DH12_DRIVER__H

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
  MEMS_SUCCESS				=		0x01,
  MEMS_ERROR				=		0x00
} status_t;

#endif /*__SHARED__TYPES*/

typedef u8_t LIS2DH12_IntPinConf_t;
typedef u8_t LIS2DH12_AxesEnabled_t;
typedef u8_t LIS2DH12_Int1Conf_t;
typedef u8_t LIS2DH12_Int2Conf_t;

typedef enum {
  MEMS_ENABLE                            =             0x01,
  MEMS_DISABLE                           =             0x00
} LIS2DH12_ACC_State_t;

typedef enum {  
  LIS2DH12_ODR_POWER_DOWN                =               0x00,
  LIS2DH12_ODR_1Hz                       =               0x01,
  LIS2DH12_ODR_10Hz                      =               0x02,
  LIS2DH12_ODR_25Hz                      =               0x03,
  LIS2DH12_ODR_50Hz                      =               0x04,
  LIS2DH12_ODR_100Hz                     =               0x05,
  LIS2DH12_ODR_200Hz                     =               0x06,
  LIS2DH12_ODR_400Hz                     =               0x07,
  LIS2DH12_ODR_1620Hz_LP                 =               0x08,
  LIS2DH12_ODR_1344Hz_NP_5367HZ_LP       =               0x09
} LIS2DH12_ODR_t;

typedef enum {
  LIS2DH12_HIGH_RES                      =               0x00,
  LIS2DH12_LOW_POWER                     =               0x01,
  LIS2DH12_NORMAL                        =               0x02
} LIS2DH12_Mode_t;

typedef enum {
  LIS2DH12_HPM_NORMAL_MODE_RES           =               0x00,
  LIS2DH12_HPM_REF_SIGNAL                =               0x01,
  LIS2DH12_HPM_NORMAL_MODE               =               0x02,
  LIS2DH12_HPM_AUTORESET_INT             =               0x03
} LIS2DH12_HPFMode_t;

typedef enum {
  LIS2DH12_HPFCF_0                       =               0x00,
  LIS2DH12_HPFCF_1                       =               0x01,
  LIS2DH12_HPFCF_2                       =               0x02,
  LIS2DH12_HPFCF_3                       =               0x03
} LIS2DH12_HPFCutOffFreq_t;

typedef enum {
  LIS2DH12_FULLSCALE_2                   =               0x00,
  LIS2DH12_FULLSCALE_4                   =               0x01,
  LIS2DH12_FULLSCALE_8                   =               0x02,
  LIS2DH12_FULLSCALE_16                  =               0x03
} LIS2DH12_Fullscale_t;

typedef enum {
  LIS2DH12_BLE_LSB                       =               0x00,
  LIS2DH12_BLE_MSB                       =               0x01
} LIS2DH12_Endianess_t;

typedef enum {
  LIS2DH12_TEMP_ENABLE                   =               0x00,
  LIS2DH12_TEMP_DISABLE                  =               0x03
} LIS2DH12_TempMode_t;

typedef enum {
  LIS2DH12_SELF_TEST_DISABLE             =               0x00,
  LIS2DH12_SELF_TEST_0                   =               0x01,
  LIS2DH12_SELF_TEST_1                   =               0x02
} LIS2DH12_SelfTest_t;

typedef enum {
  LIS2DH12_FIFO_BYPASS_MODE              =               0x00,
  LIS2DH12_FIFO_MODE                     =               0x01,
  LIS2DH12_FIFO_STREAM_MODE              =               0x02,
  LIS2DH12_FIFO_TRIGGER_MODE             =               0x03,
  LIS2DH12_FIFO_DISABLE                  =               0x04
} LIS2DH12_FifoMode_t;

typedef enum {
  LIS2DH12_TRIG_INT1                     =               0x00,
  LIS2DH12_TRIG_INT2                     =               0x01
} LIS2DH12_TrigInt_t;

typedef enum {
  LIS2DH12_SPI_4_WIRE                    =               0x00,
  LIS2DH12_SPI_3_WIRE                    =               0x01
} LIS2DH12_SPIMode_t;

typedef enum {
  LIS2DH12_X_ENABLE                      =               0x01,
  LIS2DH12_X_DISABLE                     =               0x00,
  LIS2DH12_Y_ENABLE                      =               0x02,
  LIS2DH12_Y_DISABLE                     =               0x00,
  LIS2DH12_Z_ENABLE                      =               0x04,
  LIS2DH12_Z_DISABLE                     =               0x00    
} LIS2DH12_AXISenable_t;

typedef enum {
  LIS2DH12_INT1_6D_4D_DISABLE            =               0x00,
  LIS2DH12_INT1_6D_ENABLE                =               0x01,
  LIS2DH12_INT1_4D_ENABLE                =               0x02 
} LIS2DH12_INT1_6D_4D_t;

typedef enum {
  LIS2DH12_INT2_6D_4D_DISABLE            =               0x00,
  LIS2DH12_INT2_6D_ENABLE                =               0x01,
  LIS2DH12_INT2_4D_ENABLE                =               0x02 
} LIS2DH12_INT2_6D_4D_t;

typedef enum {
  LIS2DH12_UP_SX                         =               0x44,
  LIS2DH12_UP_DX                         =               0x42,
  LIS2DH12_DW_SX                         =               0x41,
  LIS2DH12_DW_DX                         =               0x48,
  LIS2DH12_TOP                           =               0x60,
  LIS2DH12_BOTTOM                        =               0x50
} LIS2DH12_POSITION_6D_t;

typedef enum {
  LIS2DH12_INT_MODE_OR                   =               0x00,
  LIS2DH12_INT_MODE_6D_MOVEMENT          =               0x01,
  LIS2DH12_INT_MODE_AND                  =               0x02,
  LIS2DH12_INT_MODE_6D_POSITION          =               0x03  
} LIS2DH12_IntMode_t;

//interrupt click response
//  b7 = don't care   b6 = IA  b5 = DClick  b4 = Sclick  b3 = Sign  
//  b2 = z      b1 = y     b0 = x
typedef enum {
LIS2DH12_DCLICK_Z_P                      =               0x24,
LIS2DH12_DCLICK_Z_N                      =               0x2C,
LIS2DH12_SCLICK_Z_P                      =               0x14,
LIS2DH12_SCLICK_Z_N                      =               0x1C,
LIS2DH12_DCLICK_Y_P                      =               0x22,
LIS2DH12_DCLICK_Y_N                      =               0x2A,
LIS2DH12_SCLICK_Y_P                      =               0x12,
LIS2DH12_SCLICK_Y_N                      =               0x1A,
LIS2DH12_DCLICK_X_P                      =               0x21,
LIS2DH12_DCLICK_X_N                      =               0x29,
LIS2DH12_SCLICK_X_P                      =               0x11,
LIS2DH12_SCLICK_X_N                      =               0x19,
LIS2DH12_NO_CLICK                        =               0x00
} LIS2DH12_Click_Response; 


/* Exported constants --------------------------------------------------------*/

#ifndef __SHARED__CONSTANTS
#define __SHARED__CONSTANTS

#define MEMS_SET                                        0x01
#define MEMS_RESET                                      0x00

#endif /*__SHARED__CONSTANTS*/

// WHO_AM_I value
#define LIS2DH12_WHO_AM_I                              0x33

//Register Definition
// WHO_AM_I register
#define LIS2DH12_WHO_AM_I_REG                          0x0F  // device identification register

// CONTROL REGISTER 1
#define LIS2DH12_CTRL_REG1                             0x20
#define LIS2DH12_ODR_BIT                               BIT(4)
#define LIS2DH12_LPEN                                  BIT(3)
#define LIS2DH12_ZEN                                   BIT(2)
#define LIS2DH12_YEN                                   BIT(1)
#define LIS2DH12_XEN                                   BIT(0)

//CONTROL REGISTER 2
#define LIS2DH12_CTRL_REG2                              0x21
#define LIS2DH12_HPM                                    BIT(6)
#define LIS2DH12_HPCF                                   BIT(4)
#define LIS2DH12_FDS                                    BIT(3)
#define LIS2DH12_HPCLICK                                BIT(2)
#define LIS2DH12_HPIS2                                  BIT(1)
#define LIS2DH12_HPIS1                                  BIT(0)

//CONTROL REGISTER 3
#define LIS2DH12_CTRL_REG3                              0x22
#define LIS2DH12_I1_CLICK                               BIT(7)
#define LIS2DH12_I1_AOI1                                BIT(6)
#define LIS2DH12_I1_AOI2                                BIT(5)
#define LIS2DH12_I1_DRDY1                               BIT(4)
#define LIS2DH12_I1_DRDY2                               BIT(3)
#define LIS2DH12_I1_WTM                                 BIT(2)
#define LIS2DH12_I1_ORUN                                BIT(1)

//CONTROL REGISTER 6
#define LIS2DH12_CTRL_REG6                              0x25
#define LIS2DH12_I2_CLICK                               BIT(7)
#define LIS2DH12_I2_INT1                                BIT(6)
#define LIS2DH12_I2_INT2                                BIT(5)
#define LIS2DH12_I2_BOOT                                BIT(4)
#define LIS2DH12_P2_ACT                                 BIT(3)
#define LIS2DH12_H_LACTIVE                              BIT(1)

//TEMPERATURE CONFIG REGISTER
#define LIS2DH12_TEMP_CFG_REG                           0x1F
#define LIS2DH12_TEMP_EN                                BIT(6)

//CONTROL REGISTER 4
#define LIS2DH12_CTRL_REG4                              0x23
#define LIS2DH12_BDU                                    BIT(7)
#define LIS2DH12_BLE                                    BIT(6)
#define LIS2DH12_FS                                     BIT(4)
#define LIS2DH12_HR                                     BIT(3)
#define LIS2DH12_ST                                     BIT(1)
#define LIS2DH12_SIM                                    BIT(0)

//CONTROL REGISTER 5
#define LIS2DH12_CTRL_REG5                               0x24
#define LIS2DH12_BOOT                                    BIT(7)
#define LIS2DH12_FIFO_EN                                 BIT(6)
#define LIS2DH12_LIR_INT1                                BIT(3)
#define LIS2DH12_D4D_INT1                                BIT(2)
#define LIS2DH12_LIR_INT2                                BIT(1)
#define LIS2DH12_D4D_INT2                                BIT(0)

//REFERENCE/DATA_CAPTURE
#define LIS2DH12_REFERENCE_REG                           0x26
#define LIS2DH12_REF                                     BIT(0)

//STATUS_REG_AXIES
#define LIS2DH12_STATUS_REG                              0x27
#define LIS2DH12_ZYXOR                                   BIT(7)
#define LIS2DH12_ZOR                                     BIT(6)
#define LIS2DH12_YOR                                     BIT(5)
#define LIS2DH12_XOR                                     BIT(4)
#define LIS2DH12_ZYXDA                                   BIT(3)
#define LIS2DH12_ZDA                                     BIT(2)
#define LIS2DH12_YDA                                     BIT(1)
#define LIS2DH12_XDA                                     BIT(0)

//STATUS_REG_AUX
#define LIS2DH12_STATUS_AUX                              0x07

//INTERRUPT 1 CONFIGURATION
#define LIS2DH12_INT1_CFG                                0x30
#define LIS2DH12_ANDOR                                   BIT(7)
#define LIS2DH12_INT_6D                                  BIT(6)
#define LIS2DH12_ZHIE                                    BIT(5)
#define LIS2DH12_ZLIE                                    BIT(4)
#define LIS2DH12_YHIE                                    BIT(3)
#define LIS2DH12_YLIE                                    BIT(2)
#define LIS2DH12_XHIE                                    BIT(1)
#define LIS2DH12_XLIE                                    BIT(0)

//INTERRUPT 2 CONFIGURATION
#define LIS2DH12_INT2_CFG                                0x34

//FIFO CONTROL REGISTER
#define LIS2DH12_FIFO_CTRL_REG                           0x2E
#define LIS2DH12_FM                                      BIT(6)
#define LIS2DH12_TR                                      BIT(5)
#define LIS2DH12_FTH                                     BIT(0)

//CONTROL REG3 bit mask
#define LIS2DH12_CLICK_ON_PIN_INT1_ENABLE                0x80
#define LIS2DH12_CLICK_ON_PIN_INT1_DISABLE               0x00
#define LIS2DH12_I1_INT1_ON_PIN_INT1_ENABLE              0x40
#define LIS2DH12_I1_INT1_ON_PIN_INT1_DISABLE             0x00
#define LIS2DH12_I1_INT2_ON_PIN_INT1_ENABLE              0x20
#define LIS2DH12_I1_INT2_ON_PIN_INT1_DISABLE             0x00
#define LIS2DH12_I1_DRDY1_ON_INT1_ENABLE                 0x10
#define LIS2DH12_I1_DRDY1_ON_INT1_DISABLE                0x00
#define LIS2DH12_I1_DRDY2_ON_INT1_ENABLE                 0x08
#define LIS2DH12_I1_DRDY2_ON_INT1_DISABLE                0x00
#define LIS2DH12_WTM_ON_INT1_ENABLE                      0x04
#define LIS2DH12_WTM_ON_INT1_DISABLE                     0x00
#define LIS2DH12_INT1_OVERRUN_ENABLE                     0x02
#define LIS2DH12_INT1_OVERRUN_DISABLE                    0x00

//CONTROL REG6 bit mask
#define LIS2DH12_CLICK_ON_PIN_INT2_ENABLE                0x80
#define LIS2DH12_CLICK_ON_PIN_INT2_DISABLE               0x00
#define LIS2DH12_I2_INT1_ON_PIN_INT2_ENABLE              0x40
#define LIS2DH12_I2_INT1_ON_PIN_INT2_DISABLE             0x00
#define LIS2DH12_I2_INT2_ON_PIN_INT2_ENABLE              0x20
#define LIS2DH12_I2_INT2_ON_PIN_INT2_DISABLE             0x00
#define LIS2DH12_I2_BOOT_ON_INT2_ENABLE                  0x10
#define LIS2DH12_I2_BOOT_ON_INT2_DISABLE                 0x00
#define LIS2DH12_I2_ACTIVITY_ON_INT2_ENABLE              0x08
#define LIS2DH12_I2_ACTIVITY_ON_INT2_DISABLE             0x00
#define LIS2DH12_INT_ACTIVE_HIGH                         0x00
#define LIS2DH12_INT_ACTIVE_LOW                          0x02

//INT1_CFG bit mask
#define LIS2DH12_INT1_AND                                0x80
#define LIS2DH12_INT1_OR                                 0x00
#define LIS2DH12_INT1_ZHIE_ENABLE                        0x20
#define LIS2DH12_INT1_ZHIE_DISABLE                       0x00
#define LIS2DH12_INT1_ZLIE_ENABLE                        0x10
#define LIS2DH12_INT1_ZLIE_DISABLE                       0x00
#define LIS2DH12_INT1_YHIE_ENABLE                        0x08
#define LIS2DH12_INT1_YHIE_DISABLE                       0x00
#define LIS2DH12_INT1_YLIE_ENABLE                        0x04
#define LIS2DH12_INT1_YLIE_DISABLE                       0x00
#define LIS2DH12_INT1_XHIE_ENABLE                        0x02
#define LIS2DH12_INT1_XHIE_DISABLE                       0x00
#define LIS2DH12_INT1_XLIE_ENABLE                        0x01
#define LIS2DH12_INT1_XLIE_DISABLE                       0x00

//INT2_CFG bit mask
#define LIS2DH12_INT2_AND                                0x80
#define LIS2DH12_INT2_OR                                 0x00
#define LIS2DH12_INT2_ZHIE_ENABLE                        0x20
#define LIS2DH12_INT2_ZHIE_DISABLE                       0x00
#define LIS2DH12_INT2_ZLIE_ENABLE                        0x10
#define LIS2DH12_INT2_ZLIE_DISABLE                       0x00
#define LIS2DH12_INT2_YHIE_ENABLE                        0x08
#define LIS2DH12_INT2_YHIE_DISABLE                       0x00
#define LIS2DH12_INT2_YLIE_ENABLE                        0x04
#define LIS2DH12_INT2_YLIE_DISABLE                       0x00
#define LIS2DH12_INT2_XHIE_ENABLE                        0x02
#define LIS2DH12_INT2_XHIE_DISABLE                       0x00
#define LIS2DH12_INT2_XLIE_ENABLE                        0x01
#define LIS2DH12_INT2_XLIE_DISABLE                       0x00

//INT1_SRC bit mask
#define LIS2DH12_INT1_SRC_IA                             0x40
#define LIS2DH12_INT1_SRC_ZH                             0x20
#define LIS2DH12_INT1_SRC_ZL                             0x10
#define LIS2DH12_INT1_SRC_YH                             0x08
#define LIS2DH12_INT1_SRC_YL                             0x04
#define LIS2DH12_INT1_SRC_XH                             0x02
#define LIS2DH12_INT1_SRC_XL                             0x01

//INT2_SRC bit mask
#define LIS2DH12_INT2_SRC_IA                             0x40
#define LIS2DH12_INT2_SRC_ZH                             0x20
#define LIS2DH12_INT2_SRC_ZL                             0x10
#define LIS2DH12_INT2_SRC_YH                             0x08
#define LIS2DH12_INT2_SRC_YL                             0x04
#define LIS2DH12_INT2_SRC_XH                             0x02
#define LIS2DH12_INT2_SRC_XL                             0x01

//INT1 REGISTERS
#define LIS2DH12_INT1_THS                                0x32
#define LIS2DH12_INT1_DURATION                           0x33

//INT2 REGISTERS
#define LIS2DH12_INT2_THS                                0x36
#define LIS2DH12_INT2_DURATION                           0x37

//INTERRUPT 1 SOURCE REGISTER
#define LIS2DH12_INT1_SRC                                0x31
#define INT1                                             0x31

//INTERRUPT 2 SOURCE REGISTER
#define LIS2DH12_INT2_SRC                                0x35
#define INT2                                             0x35

//FIFO Source Register bit Mask
#define LIS2DH12_FIFO_SRC_WTM                            0x80
#define LIS2DH12_FIFO_SRC_OVRUN                          0x40
#define LIS2DH12_FIFO_SRC_EMPTY                          0x20
  
//INTERRUPT CLICK REGISTER
#define LIS2DH12_CLICK_CFG				0x38
//INTERRUPT CLICK CONFIGURATION bit mask
#define LIS2DH12_ZD_ENABLE                               0x20
#define LIS2DH12_ZD_DISABLE                              0x00
#define LIS2DH12_ZS_ENABLE                               0x10
#define LIS2DH12_ZS_DISABLE                              0x00
#define LIS2DH12_YD_ENABLE                               0x08
#define LIS2DH12_YD_DISABLE                              0x00
#define LIS2DH12_YS_ENABLE                               0x04
#define LIS2DH12_YS_DISABLE                              0x00
#define LIS2DH12_XD_ENABLE                               0x02
#define LIS2DH12_XD_DISABLE                              0x00
#define LIS2DH12_XS_ENABLE                               0x01
#define LIS2DH12_XS_DISABLE                              0x00

//INTERRUPT CLICK SOURCE REGISTER
#define LIS2DH12_CLICK_SRC                               0x39
//INTERRUPT CLICK SOURCE REGISTER bit mask
#define LIS2DH12_IA                                      0x40
#define LIS2DH12_DCLICK                                  0x20
#define LIS2DH12_SCLICK                                  0x10
#define LIS2DH12_CLICK_SIGN                              0x08
#define LIS2DH12_CLICK_Z                                 0x04
#define LIS2DH12_CLICK_Y                                 0x02
#define LIS2DH12_CLICK_X                                 0x01

//Click-click Register
#define LIS2DH12_CLICK_THS                               0x3A
#define LIS2DH12_TIME_LIMIT                              0x3B
#define LIS2DH12_TIME_LATENCY                            0x3C
#define LIS2DH12_TIME_WINDOW                             0x3D

//OUTPUT REGISTER
#define LIS2DH12_OUT_X_L					0x28
#define LIS2DH12_OUT_X_H					0x29
#define LIS2DH12_OUT_Y_L					0x2A
#define LIS2DH12_OUT_Y_H					0x2B
#define LIS2DH12_OUT_Z_L					0x2C
#define LIS2DH12_OUT_Z_H					0x2D

//TEMP REGISTERS
#define LIS2DH12_OUT_TEMP_L					0x0C
#define LIS2DH12_OUT_TEMP_H					0x0D

//STATUS REGISTER bit mask
#define LIS2DH12_STATUS_REG_ZYXOR                        0x80    // 1	:	new data set has over written the previous one
							// 0	:	no overrun has occurred (default)	
#define LIS2DH12_STATUS_REG_ZOR                          0x40    // 0	:	no overrun has occurred (default)
							// 1	:	new Z-axis data has over written the previous one
#define LIS2DH12_STATUS_REG_YOR                          0x20    // 0	:	no overrun has occurred (default)
							// 1	:	new Y-axis data has over written the previous one
#define LIS2DH12_STATUS_REG_XOR                          0x10    // 0	:	no overrun has occurred (default)
							// 1	:	new X-axis data has over written the previous one
#define LIS2DH12_STATUS_REG_ZYXDA                        0x08    // 0	:	a new set of data is not yet avvious one
                                                        // 1	:	a new set of data is available 
#define LIS2DH12_STATUS_REG_ZDA                          0x04    // 0	:	a new data for the Z-Axis is not availvious one
                                                        // 1	:	a new data for the Z-Axis is available
#define LIS2DH12_STATUS_REG_YDA                          0x02    // 0	:	a new data for the Y-Axis is not available
                                                        // 1	:	a new data for the Y-Axis is available
#define LIS2DH12_STATUS_REG_XDA                          0x01    // 0	:	a new data for the X-Axis is not available

#define LIS2DH12_DATAREADY_BIT                           LIS2DH12_STATUS_REG_ZYXDA


//STATUS AUX REGISTER bit mask
#define LIS2DH12_STATUS_AUX_TOR                           0x40
#define LIS2DH12_STATUS_AUX_TDA                           0x04

// I2C Address
#define LIS2DH12_I2C_ADDRESS_LOW                0x30  // SAD[0] = 0
#define LIS2DH12_I2C_ADDRESS_HIGH               0x32  // SAD[0] = 1

//FIFO REGISTERS
#define LIS2DH12_FIFO_CTRL_REG			        0x2E
#define LIS2DH12_FIFO_SRC_REG			        0x2F

//Sleep to Wake Registers
#define LIS2DH12_ACT_THS					0x3E
#define LIS2DH12_ACT_DUR					0x3F

/* Exported macro ------------------------------------------------------------*/

#ifndef __SHARED__MACROS

#define __SHARED__MACROS
#define ValBit(VAR,Place)         (VAR & (1<<Place))
#define BIT(x) ( (x) )

#endif /*__SHARED__MACROS*/

/* Exported functions --------------------------------------------------------*/
//Generic
status_t LIS2DH12_ACC_WriteReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len);
status_t LIS2DH12_ACC_ReadReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len);

//Sensor Configuration Functions
status_t LIS2DH12_GetODR(void *handle, LIS2DH12_ODR_t *ov);
status_t LIS2DH12_SetODR(void *handle, LIS2DH12_ODR_t ov);
status_t LIS2DH12_GetMode(void *handle, LIS2DH12_Mode_t *md);
status_t LIS2DH12_SetMode(void *handle, LIS2DH12_Mode_t md);
status_t LIS2DH12_GetAxesEnabled(void *handle, LIS2DH12_AxesEnabled_t *axes);
status_t LIS2DH12_SetAxesEnabled(void *handle, LIS2DH12_AxesEnabled_t axes);
status_t LIS2DH12_GetFullScale(void *handle, LIS2DH12_Fullscale_t *fs);
status_t LIS2DH12_SetFullScale(void *handle, LIS2DH12_Fullscale_t fs);
status_t LIS2DH12_SetBDU(void *handle, LIS2DH12_ACC_State_t bdu);
status_t LIS2DH12_SetBLE(void *handle, LIS2DH12_Endianess_t ble);
status_t LIS2DH12_SetSelfTest(void *handle, LIS2DH12_SelfTest_t st);
status_t LIS2DH12_SetTemperature(void *handle, LIS2DH12_TempMode_t tempmode);

//Filtering Functions
status_t LIS2DH12_HPFClickEnable(void *handle, LIS2DH12_ACC_State_t hpfe);
status_t LIS2DH12_HPFAOI1Enable(void *handle, LIS2DH12_ACC_State_t hpfe);
status_t LIS2DH12_HPFAOI2Enable(void *handle, LIS2DH12_ACC_State_t hpfe);
status_t LIS2DH12_SetHPFMode(void *handle, LIS2DH12_HPFMode_t hpf);
status_t LIS2DH12_SetHPFCutOFF(void *handle, LIS2DH12_HPFCutOffFreq_t hpf);
status_t LIS2DH12_SetFilterDataSel(void *handle, LIS2DH12_ACC_State_t state);

//Interrupt Functions
status_t LIS2DH12_SetInt1Pin(void *handle, LIS2DH12_IntPinConf_t pinConf);
status_t LIS2DH12_SetInt2Pin(void *handle, LIS2DH12_IntPinConf_t pinConf);
status_t LIS2DH12_Int1LatchEnable(void *handle, LIS2DH12_ACC_State_t latch);
status_t LIS2DH12_ResetInt1Latch(void *handle);
status_t LIS2DH12_Int2LatchEnable(void *handle, LIS2DH12_ACC_State_t latch);
status_t LIS2DH12_ResetInt2Latch(void *handle);
status_t LIS2DH12_SetInt1Configuration(void *handle, LIS2DH12_Int1Conf_t ic);
status_t LIS2DH12_SetInt2Configuration(void *handle, LIS2DH12_Int2Conf_t ic);
status_t LIS2DH12_SetInt1Threshold(void *handle, u8_t ths);
status_t LIS2DH12_SetInt1Duration(void *handle, LIS2DH12_Int1Conf_t id);
status_t LIS2DH12_SetInt2Threshold(void *handle, u8_t ths); 
status_t LIS2DH12_SetInt2Duration(void *handle, LIS2DH12_Int2Conf_t id); 
status_t LIS2DH12_SetInt1Mode(void *handle, LIS2DH12_IntMode_t ic);
status_t LIS2DH12_SetInt2Mode(void *handle, LIS2DH12_IntMode_t ic);
status_t LIS2DH12_SetClickCFG(void *handle, u8_t status);
status_t LIS2DH12_SetClickTHS(void *handle, u8_t ths);
status_t LIS2DH12_SetClickLIMIT(void *handle, u8_t t_limit);
status_t LIS2DH12_SetClickLATENCY(void *handle, u8_t t_latency);
status_t LIS2DH12_SetClickWINDOW(void *handle, u8_t t_window);
status_t LIS2DH12_SetInt16D4DConfiguration(void *handle, LIS2DH12_INT1_6D_4D_t ic);
status_t LIS2DH12_SetInt26D4DConfiguration(void *handle, LIS2DH12_INT2_6D_4D_t ic);
status_t LIS2DH12_GetInt1Src(void *handle, u8_t* val);
status_t LIS2DH12_GetInt1SrcBit(void *handle, u8_t statusBIT, u8_t* val);
status_t LIS2DH12_GetInt2Src(void *handle, u8_t* val);
status_t LIS2DH12_GetInt2SrcBit(void *handle, u8_t statusBIT, u8_t* val);

//FIFO Functions
status_t LIS2DH12_FIFOModeEnable(void *handle, LIS2DH12_FifoMode_t fm);
status_t LIS2DH12_SetWaterMark(void *handle, u8_t wtm);
status_t LIS2DH12_SetTriggerInt(void *handle, LIS2DH12_TrigInt_t tr);
status_t LIS2DH12_GetFifoSourceReg(void *handle, u8_t* val);
status_t LIS2DH12_GetFifoSourceBit(void *handle, u8_t statusBIT, u8_t* val);
status_t LIS2DH12_GetFifoSourceFSS(void *handle, u8_t* val);
status_t LIS2DH12_SetSPIInterface(void *handle, LIS2DH12_SPIMode_t spi);

//Sleep to Wake Functions
status_t LIS2DH12_SetActTHS(void *handle, u8_t val);
status_t LIS2DH12_SetActDUR(void *handle, u8_t val);

//Other Reading Functions
status_t LIS2DH12_GetStatusReg(void *handle, u8_t* val);
status_t LIS2DH12_GetStatusBit(void *handle, u8_t statusBIT, u8_t* val);
status_t LIS2DH12_GetStatusAUXBit(void *handle, u8_t statusBIT, u8_t* val);
status_t LIS2DH12_GetStatusAUX(void *handle, u8_t* val);
status_t LIS2DH12_GetAccAxesRaw(void *handle, u8_t *buff);
status_t LIS2DH12_GetClickResponse(void *handle, u8_t* val);
status_t LIS2DH12_GetTempRaw(void *handle, i8_t* val);
status_t LIS2DH12_GetWHO_AM_I(void *handle, u8_t* val);
status_t LIS2DH12_Get6DPosition(void *handle, u8_t* val, u8_t INT);

#endif /* __LIS2DH12_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/



