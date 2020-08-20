/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
* File Name          : H3LIS331DL_ACC_driver.h
* Author             : MEMS Application Team
* Version            : v1.1
* Date               : 26 Feb 2016  
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
#ifndef __H3LIS331DL_DRIVER__H
#define __H3LIS331DL_DRIVER__H

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

//these could change accordingly with the architecture

#ifndef __ARCHDEP__TYPES
#define __ARCHDEP__TYPES
typedef unsigned char u8_t;
typedef unsigned short int u16_t;
typedef short int i16_t;
typedef short int i8_t;

#endif /*__ARCHDEP__TYPES*/

typedef u8_t H3LIS331DL_Axis_t;
typedef u8_t H3LIS331DL_IntConf_t;

//define structure
#ifndef __SHARED__TYPES
#define __SHARED__TYPES

typedef enum {
  MEMS_SUCCESS                            =		0x01,
  MEMS_ERROR			          =		0x00	
} status_t;

typedef enum {
  MEMS_ENABLE			          =		0x01,
  MEMS_DISABLE			          =		0x00	
} State_t;

typedef struct {
  i16_t AXIS_X;
  i16_t AXIS_Y;
  i16_t AXIS_Z;
} AxesRaw_t;

#endif /*__SHARED__TYPES*/

typedef enum {  
  H3LIS331DL_ODR_50Hz                       =		0x00,
  H3LIS331DL_ODR_100Hz		          =		0x01,	
  H3LIS331DL_ODR_400Hz		          =		0x02,
  H3LIS331DL_ODR_1000Hz		          =		0x03
} H3LIS331DL_ODR_t;

typedef enum {
  H3LIS331DL_CONTINUOUS_MODE                =		0x00,
  H3LIS331DL_SINGLE_MODE 		          =		0x01,
  H3LIS331DL_SLEEP_MODE			  =		0x02
} H3LIS331DL_Mode_M_t;

typedef enum {
  H3LIS331DL_POWER_DOWN                     =		0x00,
  H3LIS331DL_NORMAL 			  =		0x01,
  H3LIS331DL_LOW_POWER_05		          =		0x02,
  H3LIS331DL_LOW_POWER_1 		          =		0x03,
  H3LIS331DL_LOW_POWER_2			  =		0x04,
  H3LIS331DL_LOW_POWER_5			  =		0x05,
  H3LIS331DL_LOW_POWER_10		          =		0x06,
} H3LIS331DL_Mode_t;

typedef enum {
  H3LIS331DL_HPM_NORMAL_MODE_RES            =             0x00,
  H3LIS331DL_HPM_REF_SIGNAL                 =             0x01,
  H3LIS331DL_HPM_NORMAL_MODE                =             0x02,
} H3LIS331DL_HPFMode_t;

typedef enum {
  H3LIS331DL_HPFCF_0                        =             0x00,
  H3LIS331DL_HPFCF_1                        =             0x01,
  H3LIS331DL_HPFCF_2                        =             0x02,
  H3LIS331DL_HPFCF_3                        =             0x03
} H3LIS331DL_HPFCutOffFreq_t;

typedef enum {
  H3LIS331DL_INT_SOURCE                     =             0x00,
  H3LIS331DL_INT_1OR2_SOURCE                =             0x01,
  H3LIS331DL_DATA_READY                     =             0x02,
  H3LIS331DL_BOOT_RUNNING                   =             0x03
} H3LIS331DL_INT_Conf_t;

typedef enum {
  H3LIS331DL_SLEEP_TO_WAKE_DIS              =             0x00,
  H3LIS331DL_SLEEP_TO_WAKE_ENA              =             0x03,
} H3LIS331DL_Sleep_To_Wake_Conf_t;

typedef enum {
  H3LIS331DL_FULLSCALE_2                    =             0x00,
  H3LIS331DL_FULLSCALE_4                    =             0x01,
  H3LIS331DL_FULLSCALE_8                    =             0x03,
} H3LIS331DL_Fullscale_t;

typedef enum {
  H3LIS331DL_BLE_LSB                        =		0x00,
  H3LIS331DL_BLE_MSB                        =		0x01
} H3LIS331DL_Endianess_t;

typedef enum {
  H3LIS331DL_SPI_4_WIRE                     =             0x00,
  H3LIS331DL_SPI_3_WIRE                     =             0x01
} H3LIS331DL_SPIMode_t;

typedef enum {
  H3LIS331DL_X_ENABLE                       =             0x01,
  H3LIS331DL_X_DISABLE                      =             0x00,
  H3LIS331DL_Y_ENABLE                       =             0x02,
  H3LIS331DL_Y_DISABLE                      =             0x00,
  H3LIS331DL_Z_ENABLE                       =             0x04,
  H3LIS331DL_Z_DISABLE                      =             0x00    
} H3LIS331DL_AXISenable_t;

typedef enum {
  H3LIS331DL_UP_SX                          =             0x44,
  H3LIS331DL_UP_DX                          =             0x42,
  H3LIS331DL_DW_SX                          =             0x41,
  H3LIS331DL_DW_DX                          =             0x48,
  H3LIS331DL_TOP                            =             0x60,
  H3LIS331DL_BOTTOM                         =             0x50
} H3LIS331DL_POSITION_6D_t;

typedef enum {
  H3LIS331DL_INT_MODE_OR                    =             0x00,
  H3LIS331DL_INT_MODE_6D_MOVEMENT           =             0x01,
  H3LIS331DL_INT_MODE_AND                   =             0x02,
  H3LIS331DL_INT_MODE_6D_POSITION           =             0x03  
} H3LIS331DL_IntMode_t;


/* Exported constants --------------------------------------------------------*/

#ifndef __SHARED__CONSTANTS
#define __SHARED__CONSTANTS

#define MEMS_SET                                        0x01
#define MEMS_RESET                                      0x00

#endif /*__SHARED__CONSTANTS*/

#define H3LIS331DL_MEMS_I2C_ADDRESS                       0x32

//Register and define
#define H3LIS331DL_WHO_AM_I				0x0F  // device identification register

// CONTROL REGISTER 1 
#define H3LIS331DL_CTRL_REG1       			0x20
#define H3LIS331DL_PM				        BIT(5)
#define H3LIS331DL_DR				        BIT(3)
#define H3LIS331DL_ZEN					BIT(2)
#define H3LIS331DL_YEN					BIT(1)
#define H3LIS331DL_XEN					BIT(0)

//CONTROL REGISTER 2 
#define H3LIS331DL_CTRL_REG2				0x21
#define H3LIS331DL_BOOT                                   BIT(7)
#define H3LIS331DL_HPM     				BIT(5)
#define H3LIS331DL_FDS     				BIT(4)
#define H3LIS331DL_HPEN2					BIT(3)
#define H3LIS331DL_HPEN1					BIT(2)
#define H3LIS331DL_HPCF					BIT(0)

//CONTROL REGISTER 3 
#define H3LIS331DL_CTRL_REG3				0x22
#define H3LIS331DL_IHL                                    BIT(7)
#define H3LIS331DL_PP_OD					BIT(6)
#define H3LIS331DL_LIR2				        BIT(5)
#define H3LIS331DL_I2_CFG  				BIT(3)
#define H3LIS331DL_LIR1    				BIT(2)
#define H3LIS331DL_I1_CFG  				BIT(0)

//CONTROL REGISTER 4
#define H3LIS331DL_CTRL_REG4				0x23
#define H3LIS331DL_BDU					BIT(7)
#define H3LIS331DL_BLE					BIT(6)
#define H3LIS331DL_FS					BIT(4)
#define H3LIS331DL_ST_SIGN				BIT(3)
#define H3LIS331DL_ST       				BIT(1)
#define H3LIS331DL_SIM					BIT(0)

//CONTROL REGISTER 5
#define H3LIS331DL_CTRL_REG5       			0x24
#define H3LIS331DL_TURN_ON                                BIT(0)

#define H3LIS331DL_HP_FILTER_RESET			0x25

//REFERENCE/DATA_CAPTURE
#define H3LIS331DL_REFERENCE_REG		                0x26
#define H3LIS331DL_REF		                	BIT(0)

//STATUS_REG_AXIES 
#define H3LIS331DL_STATUS_REG				0x27

//INTERRUPT 1 CONFIGURATION 
#define H3LIS331DL_INT1_CFG				0x30

//INTERRUPT 2 CONFIGURATION 
#define H3LIS331DL_INT2_CFG				0x34
#define H3LIS331DL_ANDOR                                  BIT(7)
#define H3LIS331DL_INT_6D                                 BIT(6)

//INT REGISTERS 
#define H3LIS331DL_INT1_THS                               0x32
#define H3LIS331DL_INT1_DURATION                          0x33
#define H3LIS331DL_INT2_THS                               0x36
#define H3LIS331DL_INT2_DURATION                          0x37

//INTERRUPT 1 SOURCE REGISTER 
#define H3LIS331DL_INT1_SRC                               0x31
#define H3LIS331DL_INT2_SRC			        0x35

//INT_CFG  bit mask
#define H3LIS331DL_INT_AND                                0x80
#define H3LIS331DL_INT_OR                                 0x00
#define H3LIS331DL_INT_ZHIE_ENABLE                        0x20
#define H3LIS331DL_INT_ZHIE_DISABLE                       0x00
#define H3LIS331DL_INT_ZLIE_ENABLE                        0x10
#define H3LIS331DL_INT_ZLIE_DISABLE                       0x00
#define H3LIS331DL_INT_YHIE_ENABLE                        0x08
#define H3LIS331DL_INT_YHIE_DISABLE                       0x00
#define H3LIS331DL_INT_YLIE_ENABLE                        0x04
#define H3LIS331DL_INT_YLIE_DISABLE                       0x00
#define H3LIS331DL_INT_XHIE_ENABLE                        0x02
#define H3LIS331DL_INT_XHIE_DISABLE                       0x00
#define H3LIS331DL_INT_XLIE_ENABLE                        0x01
#define H3LIS331DL_INT_XLIE_DISABLE                       0x00

//INT_SRC  bit mask
#define H3LIS331DL_INT_SRC_IA                             0x40
#define H3LIS331DL_INT_SRC_ZH                             0x20
#define H3LIS331DL_INT_SRC_ZL                             0x10
#define H3LIS331DL_INT_SRC_YH                             0x08
#define H3LIS331DL_INT_SRC_YL                             0x04
#define H3LIS331DL_INT_SRC_XH                             0x02
#define H3LIS331DL_INT_SRC_XL                             0x01

//OUTPUT REGISTER
#define H3LIS331DL_OUT_X_L                                0x28
#define H3LIS331DL_OUT_X_H                                0x29
#define H3LIS331DL_OUT_Y_L			        0x2A
#define H3LIS331DL_OUT_Y_H		                0x2B
#define H3LIS331DL_OUT_Z_L			        0x2C
#define H3LIS331DL_OUT_Z_H		                0x2D

//STATUS REGISTER bit mask
#define H3LIS331DL_STATUS_REG_ZYXOR                       0x80    // 1	:	new data set has over written the previous one
						                // 0	:	no overrun has occurred (default)	
#define H3LIS331DL_STATUS_REG_ZOR                         0x40    // 0	:	no overrun has occurred (default)
							        // 1	:	new Z-axis data has over written the previous one
#define H3LIS331DL_STATUS_REG_YOR                         0x20    // 0	:	no overrun has occurred (default)
						        	// 1	:	new Y-axis data has over written the previous one
#define H3LIS331DL_STATUS_REG_XOR                         0x10    // 0	:	no overrun has occurred (default)
							        // 1	:	new X-axis data has over written the previous one
#define H3LIS331DL_STATUS_REG_ZYXDA                       0x08    // 0	:	a new set of data is not yet avvious one
                                                                // 1	:	a new set of data is available 
#define H3LIS331DL_STATUS_REG_ZDA                         0x04    // 0	:	a new data for the Z-Axis is not availvious one
                                                                // 1	:	a new data for the Z-Axis is available
#define H3LIS331DL_STATUS_REG_YDA                         0x02    // 0	:	a new data for the Y-Axis is not available
                                                                // 1	:	a new data for the Y-Axis is available
#define H3LIS331DL_STATUS_REG_XDA                         0x01    // 0	:	a new data for the X-Axis is not available
                                                                // 1	:	a new data for the X-Axis is available
#define H3LIS331DL_DATAREADY_BIT                          H3LIS331DL_STATUS_REG_ZYXDA



/* Exported macro ------------------------------------------------------------*/

#ifndef __SHARED__MACROS

#define __SHARED__MACROS
#define ValBit(VAR,Place)         (VAR & (1<<Place))
#define BIT(x) ( (x) )

#endif /*__SHARED__MACROS*/

/* Exported functions --------------------------------------------------------*/

//Sensor Configuration Functions
status_t H3LIS331DL_GetWHO_AM_I(void *handle, u8_t* val);
status_t H3LIS331DL_SetODR(void *handle, H3LIS331DL_ODR_t dr);
status_t H3LIS331DL_SetMode(void *handle, H3LIS331DL_Mode_t pm);
status_t H3LIS331DL_SetAxis(void *handle, H3LIS331DL_Axis_t axis);
status_t H3LIS331DL_SetFullScale(void *handle, H3LIS331DL_Fullscale_t fs);
status_t H3LIS331DL_SetBDU(void *handle, State_t bdu);
status_t H3LIS331DL_SetBLE(void *handle, H3LIS331DL_Endianess_t ble);
status_t H3LIS331DL_SetSelfTest(void *handle, State_t st);
status_t H3LIS331DL_SetSelfTestSign(void *handle, State_t st_sign);
status_t H3LIS331DL_TurnONEnable(void *handle, H3LIS331DL_Sleep_To_Wake_Conf_t stw);
status_t H3LIS331DL_SetBOOT(void *handle, State_t boot);
status_t H3LIS331DL_SetFDS(void *handle, State_t fds);
status_t H3LIS331DL_SetSPI34Wire(void *handle, H3LIS331DL_SPIMode_t sim);

//Filtering Functions
status_t H3LIS331DL_SetHPFMode(void *handle, H3LIS331DL_HPFMode_t hpm);
status_t H3LIS331DL_SetHPFCutOFF(void *handle, H3LIS331DL_HPFCutOffFreq_t hpf);
status_t H3LIS331DL_SetFilterDataSel(void *handle, State_t state);
status_t H3LIS331DL_SetReference(void *handle, i8_t ref);

//Interrupt Functions
status_t H3LIS331DL_SetIntHighLow(void *handle, State_t hil);
status_t H3LIS331DL_SetIntPPOD(void *handle, State_t pp_od);
status_t H3LIS331DL_SetInt1DataSign(void *handle, H3LIS331DL_INT_Conf_t i_cfg);
status_t H3LIS331DL_SetInt2DataSign(void *handle, H3LIS331DL_INT_Conf_t i_cfg);
status_t H3LIS331DL_SetInt1HPEnable(void *handle, State_t stat);
status_t H3LIS331DL_SetInt2HPEnable(void *handle, State_t stat);
status_t H3LIS331DL_Int1LatchEnable(void *handle, State_t latch);
status_t H3LIS331DL_Int2LatchEnable(void *handle, State_t latch);
status_t H3LIS331DL_ResetInt1Latch(void *handle);
status_t H3LIS331DL_ResetInt2Latch(void *handle);
status_t H3LIS331DL_SetInt1Configuration(void *handle, H3LIS331DL_IntConf_t ic);
status_t H3LIS331DL_SetInt2Configuration(void *handle, H3LIS331DL_IntConf_t ic);
status_t H3LIS331DL_SetInt1Threshold(void *handle, u8_t ths);
status_t H3LIS331DL_SetInt2Threshold(void *handle, u8_t ths);
status_t H3LIS331DL_SetInt1Duration(void *handle, u8_t id);
status_t H3LIS331DL_SetInt2Duration(void *handle, u8_t id);
status_t H3LIS331DL_SetInt1Mode(void *handle, H3LIS331DL_IntMode_t int_mode);
status_t H3LIS331DL_SetInt2Mode(void *handle, H3LIS331DL_IntMode_t int_mode);
status_t H3LIS331DL_GetInt1Src(void *handle, u8_t* val);
status_t H3LIS331DL_GetInt2Src(void *handle, u8_t* val);
status_t H3LIS331DL_GetInt1SrcBit(void *handle, u8_t statusBIT, u8_t* val);
status_t H3LIS331DL_GetInt2SrcBit(void *handle, u8_t statusBIT, u8_t* val); 

//Other Reading Functions
status_t H3LIS331DL_GetStatusReg(void *handle, u8_t* val);
status_t H3LIS331DL_GetStatusBit(void *handle, u8_t statusBIT, u8_t* val);
status_t H3LIS331DL_GetAccAxesRaw(void *handle, AxesRaw_t* buff);
status_t H3LIS331DL_Get6DPositionInt1(void *handle, u8_t* val);
status_t H3LIS331DL_Get6DPositionInt2(void *handle, u8_t* val);

//Generic
status_t H3LIS331DL_ACC_WriteReg( void *handle, u8_t Reg, u8_t *Bufp, u16_t len );
status_t H3LIS331DL_ACC_ReadReg( void *handle, u8_t Reg, u8_t *Bufp, u16_t len );

#endif /*__H3LIS331DL_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/



