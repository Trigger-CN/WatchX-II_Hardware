/******************** (C) COPYRIGHT 2013 STMicroelectronics ********************
* File Name          : L3GD20H_driver.h
* Author             : MEMS Application Team
* Version            : ver 1.0
* Date               : October 2013  
* Description        : L3GD20H header driver file
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
#ifndef __L3GD20HH_DRIVER__H
#define __L3GD20H_DRIVER__H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

//these could change accordingly with the architecture

#ifndef __ARCHDEP__TYPES
#define __ARCHDEP__TYPES
typedef unsigned char u8_t;
typedef unsigned short int u16_t;
typedef short int i16_t;
typedef signed char i8_t;
#endif /*__ARCHDEP__TYPES*/

typedef u8_t L3GD20H_Int1PinConf_t;
typedef u8_t L3GD20H_Int2PinConf_t;
typedef u8_t L3GD20H_Int1Conf_t;
typedef u8_t L3GD20H_Axis_t;

/* Exported common structure --------------------------------------------------------*/

#ifndef __SHARED__TYPES
#define __SHARED__TYPES

typedef enum {
  MEMS_SUCCESS				=		0x01,
  MEMS_ERROR				=		0x00	
} status_t;

typedef enum {
  MEMS_ENABLE				=		0x01,
  MEMS_DISABLE				=		0x00	
} State_t;

typedef struct {
  i16_t AXIS_X;
  i16_t AXIS_Y;
  i16_t AXIS_Z;
} AxesRaw_t;

#endif /*__SHARED__TYPES*/

typedef enum {
  L3GD20H_DISABLE                   =		0x00,
  L3GD20H_LEVEL_SENSITIVE  			=		0x01,
  L3GD20H_EDGE_SENSITIVE			=		0x02,
  L3GD20H_PULSE_SENSITIVE			=		0x03
} L3GD20H_TRG_Mode_t;

typedef enum {
  L3GD20H_RST                   =		0x00,
  L3GD20H_DEC					=		0x80
} L3GD20H_DCRM_t;

typedef enum {
  L3GD20H_ACTIVE_HIGH           =		0x00,
  L3GD20H_ACTIVE_LOW			=		0x20
} L3GD20H_DRDY_HL_t;

typedef enum {
  L3GD20H_I2C_DISABLE   	            =		0x00,
  L3GD20H_I2C_ENABLE					=		0x80
} L3GD20H_I2C_Mode_t;

typedef enum {  
  L3GD20H_ODR_12_2Hz		            =		0x80,
  L3GD20H_ODR_25Hz_			            =		0x84,
  L3GD20H_ODR_50Hz_BW_16_6				=		0x88,
  L3GD20H_ODR_100Hz_BW_12_5             =		0x00,
  L3GD20H_ODR_100Hz_BW_25				=		0x01,	
  L3GD20H_ODR_200Hz_BW_12_5             =		0x04,
  L3GD20H_ODR_200Hz_BW_25				=		0x05,
  L3GD20H_ODR_200Hz_BW_50				=		0x06,
  L3GD20H_ODR_190Hz_BW_70				=		0x07,	
  L3GD20H_ODR_400Hz_BW_20				=		0x08,
  L3GD20H_ODR_400Hz_BW_25				=		0x09,
  L3GD20H_ODR_400Hz_BW_50				=		0x0A,
  L3GD20H_ODR_400Hz_BW_110              =		0x0B,	
  L3GD20H_ODR_800Hz_BW_30				=		0x0C,
  L3GD20H_ODR_800Hz_BW_35				=		0x0D,
  L3GD20H_ODR_800Hz_BW_50				=		0x0E,
  L3GD20H_ODR_800Hz_BW_110     	        =		0x0F
} L3GD20H_ODR_t;

typedef enum {
  L3GD20H_POWER_DOWN                    =		0x00,
  L3GD20H_SLEEP 						=		0x01,
  L3GD20H_NORMAL						=		0x02
} L3GD20H_Mode_t;

typedef enum {
  L3GD20H_HPM_NORMAL_MODE_RES           =               0x00,
  L3GD20H_HPM_REF_SIGNAL                =               0x01,
  L3GD20H_HPM_NORMAL_MODE               =               0x02,
  L3GD20H_HPM_AUTORESET_INT             =               0x03
} L3GD20H_HPFMode_t;

typedef enum {
  L3GD20H_HPFCF_0                       =               0x00,
  L3GD20H_HPFCF_1                       =               0x01,
  L3GD20H_HPFCF_2                       = 	       0x02,
  L3GD20H_HPFCF_3                       =               0x03,
  L3GD20H_HPFCF_4                       =               0x04,
  L3GD20H_HPFCF_5                       =               0x05,
  L3GD20H_HPFCF_6                       =               0x06,
  L3GD20H_HPFCF_7                       =               0x07,
  L3GD20H_HPFCF_8                       =               0x08,
  L3GD20H_HPFCF_9                       =               0x09
} L3GD20H_HPFCutOffFreq_t;

typedef enum {
  L3GD20H_PUSH_PULL                     =		0x00,
  L3GD20H_OPEN_DRAIN                    =                0x01  
} L3GD20H_IntPinMode_t;

typedef enum {
  L3GD20H_FULLSCALE_250                 	=               0x00,
  L3GD20H_FULLSCALE_500                 	=               0x01,
  L3GD20H_FULLSCALE_2000               	=               0x02	
} L3GD20H_Fullscale_t;

typedef enum {
  L3GD20H_BLE_LSB			=		0x00,
  L3GD20H_BLE_MSB			=		0x01
} L3GD20H_Endianess_t;

typedef enum {
  L3GD20H_SPI_4_WIRE                     =               0x00,
  L3GD20H_SPI_3_WIRE                     =               0x01
} L3GD20H_SPIMode_t;

typedef enum {
  L3GD20H_FIFO_DISABLE                  =               0x05,
  L3GD20H_FIFO_BYPASS_MODE              =               0x00,
  L3GD20H_FIFO_MODE                     =               0x01,
  L3GD20H_FIFO_STREAM_MODE              =               0x02,
  L3GD20H_FIFO_STREAM_TO_FIFO_MODE      =               0x03,
  L3GD20H_FIFO_BYPASS_TO_STREAM_MODE    =               0x04    
} L3GD20H_FifoMode_t;

typedef enum {
  L3GD20H_NONE                          =               0x00,
  L3GD20H_HPF                           =               0x01,
  L3GD20H_LPF2                          =               0x02,
  L3GD20H_HPFLPF2                       =               0x03
} L3GD20H_HPF_LPF2_Enable;

typedef enum {
  L3GD20H_THS_X                         =                0x00,
  L3GD20H_THS_Y                         =                0x01,  
  L3GD20H_THS_Z                         =                0x02
} L3GD20H_IntThsAxis;

typedef enum {
  L3GD20H_FTH_ENABLE                        =                0x20,
  L3GD20H_FTH_DISABLE                       =                0x00
} L3GD20H_StopOn_FTH_t;

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

/**************CONTROL REGISTERS*****************/

#define L3GD20H_MEMS_I2C_ADDRESS         0xD6

/***************CTRL1***************/
#define L3GD20H_CTRL_REG1				0x20
#define L3GD20H_ODR_BIT                                  BIT(4)
#define L3GD20H_PD					BIT(3)
#define L3GD20H_ZEN					BIT(2)
#define L3GD20H_YEN					BIT(1)
#define L3GD20H_XEN					BIT(0)
#define L3GD20H_X_ENABLE                                 0x02
#define L3GD20H_Y_ENABLE                                 0x01
#define L3GD20H_Z_ENABLE                                 0x04

/***************CTRL2***************/
#define L3GD20H_CTRL_REG2				0x21
#define L3GD20H_HPM					BIT(4)
#define L3GD20H_HPFC3					BIT(3)
#define L3GD20H_HPFC2					BIT(2)
#define L3GD20H_HPFC1					BIT(1)
#define L3GD20H_HPFC0					BIT(0)

/***************CTRL3***************/
#define L3GD20H_CTRL_REG3				0x22
#define L3GD20H_I1_INT					BIT(7)
#define L3GD20H_I1_BOOT					BIT(6)
#define L3GD20H_H_LACTIVE				BIT(5)
#define L3GD20H_PP_OD					BIT(4)
#define L3GD20H_I2_DRDY					BIT(3)
#define L3GD20H_I2_WTM					BIT(2)
#define L3GD20H_I2_ORUN					BIT(1)
#define L3GD20H_I2_EMPTY					BIT(0)
#define L3GD20H_I1_ON_PIN_INT1_ENABLE                   0x80
#define L3GD20H_I1_ON_PIN_INT1_DISABLE                  0x00
#define L3GD20H_I1_BOOT_ON_INT1_ENABLE                  0x40
#define L3GD20H_I1_BOOT_ON_INT1_DISABLE                 0x00
#define L3GD20H_INT1_ACTIVE_HIGH                        0x00
#define L3GD20H_INT1_ACTIVE_LOW                         0x20
#define L3GD20H_I2_DRDY_ON_INT2_ENABLE                  0x08
#define L3GD20H_I2_DRDY_ON_INT2_DISABLE                 0x00
#define L3GD20H_WTM_ON_INT2_ENABLE                      0x04
#define L3GD20H_WTM_ON_INT2_DISABLE                     0x00
#define L3GD20H_OVERRUN_ON_INT2_ENABLE                  0x02
#define L3GD20H_OVERRUN_ON_INT2_DISABLE                 0x00
#define L3GD20H_EMPTY_ON_INT2_ENABLE                    0x01
#define L3GD20H_EMPTY_ON_INT2_DISABLE                   0x00

/***************CTRL4***************/
#define L3GD20H_CTRL_REG4				0x23
#define L3GD20H_BDU					BIT(7)
#define L3GD20H_BLE					BIT(6)
#define L3GD20H_FS					BIT(4)
#define L3GD20H_SIM					BIT(0)

/***************CTRL5***************/
#define L3GD20H_CTRL_REG5			        0x24
#define L3GD20H_FIFO_EN                                 BIT(6)
#define L3GD20H_HPEN                                    BIT(4)
#define L3GD20H_INT1_SEL1                               BIT(3)
#define L3GD20H_INT1_SEL0                               BIT(2)
#define L3GD20H_OUT_SEL1                                BIT(1)
#define L3GD20H_OUT_SEL0                                BIT(0)

/**************GYROSCOPE INTERRUPT REGISTERS***************/
#define L3GD20H_INT1_CFG					0x30
#define L3GD20H_INT1_SRC					0x31
#define L3GD20H_INT1_THS_XH				0x32
#define L3GD20H_INT1_THS_XL				0x33
#define L3GD20H_INT1_THS_YH				0x34
#define L3GD20H_INT1_THS_YL				0x35
#define L3GD20H_INT1_THS_ZH				0x36
#define L3GD20H_INT1_THS_ZL				0x37
#define L3GD20H_INT1_DURATION				0x38
#define L3GD20H_LIR                                     BIT(6)
#define L3GD20H_ANDOR                                   BIT(7)
#define L3GD20H_ZHIE                                    BIT(5)
#define L3GD20H_ZLIE                                    BIT(4)
#define L3GD20H_YHIE                                    BIT(3)
#define L3GD20H_YLIE                                    BIT(2)
#define L3GD20H_XHIE                                    BIT(1)
#define L3GD20H_XLIE                                    BIT(0)
#define L3GD20H_INT1_AND                                0x80
#define L3GD20H_INT1_OR                                 0x00
#define L3GD20H_INT1_LIR_ENABLE                         0x40
#define L3GD20H_INT1_LIR_DISABLE                        0x00
#define L3GD20H_INT1_ZHIE_ENABLE                        0x20
#define L3GD20H_INT1_ZHIE_DISABLE                       0x00
#define L3GD20H_INT1_ZLIE_ENABLE                        0x10
#define L3GD20H_INT1_ZLIE_DISABLE                       0x00
#define L3GD20H_INT1_YHIE_ENABLE                        0x08
#define L3GD20H_INT1_YHIE_DISABLE                       0x00
#define L3GD20H_INT1_YLIE_ENABLE                        0x04
#define L3GD20H_INT1_YLIE_DISABLE                       0x00
#define L3GD20H_INT1_XHIE_ENABLE                        0x02
#define L3GD20H_INT1_XHIE_DISABLE                       0x00
#define L3GD20H_INT1_XLIE_ENABLE                        0x01
#define L3GD20H_INT1_XLIE_DISABLE                       0x00
/*************** LOW ODR ***************/
#define L3GD20H_LOW_ODR                       0x39

/**********GYROSCOPE: STATUS AND OUTPUT REGISTERS***********/
//OUTPUT REGISTER
#define L3GD20H_OUT_X_L					0x28
#define L3GD20H_OUT_X_H					0x29
#define L3GD20H_OUT_Y_L					0x2A
#define L3GD20H_OUT_Y_H					0x2B
#define L3GD20H_OUT_Z_L					0x2C
#define L3GD20H_OUT_Z_H					0x2D
#define L3GD20H_STATUS_REG                              0x27
#define L3GD20H_STATUS_REG_ZYXOR                        0x07    // 1	:	new data set has over written the previous one
							// 0	:	no overrun has occurred (default)	
#define L3GD20H_STATUS_REG_ZOR                          0x06    // 0	:	no overrun has occurred (default)
							// 1	:	new Z-axis data has over written the previous one
#define L3GD20H_STATUS_REG_YOR                          0x05    // 0	:	no overrun has occurred (default)
							// 1	:	new Y-axis data has over written the previous one
#define L3GD20H_STATUS_REG_XOR                          0x04    // 0	:	no overrun has occurred (default)
							// 1	:	new X-axis data has over written the previous one
#define L3GD20H_STATUS_REG_ZYXDA                        0x03    // 0	:	a new set of data is not yet avvious one
                                                        // 1	:	a new set of data is available 
#define L3GD20H_STATUS_REG_ZDA                          0x02    // 0	:	a new data for the Z-Axis is not availvious one
                                                        // 1	:	a new data for the Z-Axis is available
#define L3GD20H_STATUS_REG_YDA                          0x01    // 0	:	a new data for the Y-Axis is not available
                                                        // 1	:	a new data for the Y-Axis is available
#define STATUS_REG_XDA                         			0x00    // 0	:	a new data for the X-Axis is not available

#define L3GD20H_DATAREADY_BIT                           L3GD20H_STATUS_REG_ZYXDA

#define L3GD20H_I_AM_L3GD20H			        0xD4

/*************GYROSCOPE FIFO CONTROL REGISTER**************/
#define L3GD20H_FM0                                      BIT(5)
#define L3GD20H_FIFO_CTRL_REG                            0x2E
#define L3GD20H_FIFO_SRC_REG			        0x2F

/* Exported functions --------------------------------------------------------*/
/**********Sensor Configuration Functions***********/
status_t L3GD20H_SetODR(L3GD20H_ODR_t ov);
status_t L3GD20H_SetMode(L3GD20H_Mode_t md);
status_t L3GD20H_SetAxis(L3GD20H_Axis_t axis);
status_t L3GD20H_SetFullScale(L3GD20H_Fullscale_t fs);
status_t L3GD20H_SetBDU(State_t bdu);
status_t L3GD20H_SetBLE(L3GD20H_Endianess_t ble);
status_t L3GD20H_SetSPIInterface(L3GD20H_SPIMode_t spi);
status_t L3GD20H_SoftwareRST(L3GD20H_I2C_Mode_t ov);
status_t L3GD20H_SetDRDY(L3GD20H_DRDY_HL_t ov);

/***************Filtering Functions****************/
status_t L3GD20H_SetHPFMode(L3GD20H_HPFMode_t hpf);
status_t L3GD20H_SetHPFCutOFF(L3GD20H_HPFCutOffFreq_t hpf);status_t L3GD20H_HPFEnable(State_t hpf);
status_t L3GD20H_SetOutputDataAndFifoFilters(L3GD20H_HPF_LPF2_Enable hpf);
status_t L3GD20H_SetInt1Filters(L3GD20H_HPF_LPF2_Enable hpf);

/***************Interrupt Functions****************/
status_t L3GD20H_SetIntPinMode(L3GD20H_IntPinMode_t pm);
status_t L3GD20H_SetInt1Pin(L3GD20H_Int1PinConf_t pinConf);
status_t L3GD20H_SetInt2Pin(L3GD20H_Int2PinConf_t pinConf);
status_t L3GD20H_Int1LatchEnable(State_t latch);
status_t L3GD20H_ResetInt1Latch(void);
status_t L3GD20H_SetIntConfiguration(L3GD20H_Int1Conf_t ic);
status_t L3GD20H_SetInt1Threshold(L3GD20H_IntThsAxis axis, u16_t ths);
status_t L3GD20H_SetInt1Duration(L3GD20H_Int1Conf_t id);
status_t L3GD20H_Set_TRG_mode(L3GD20H_TRG_Mode_t ov);
status_t L3GD20H_IntGen_CounterMode(L3GD20H_DCRM_t ov);

/*****************FIFO Functions******************/
status_t L3GD20H_FIFOModeEnable(L3GD20H_FifoMode_t fm);
status_t L3GD20H_SetWaterMark(u8_t wtm);
status_t L3GD20H_StopOnFTH(L3GD20H_StopOn_FTH_t ov);

/****************Reading Functions*****************/
status_t L3GD20H_GetSatusReg(u8_t* buff);
status_t L3GD20H_GetAngRateRaw(AxesRaw_t* buff);
status_t L3GD20H_GetFifoSourceReg(u8_t* buff);
status_t L3GD20H_GetInt1Src(u8_t* buff);


/*********************Generic*********************/
u8_t L3GD20H_ReadReg(u8_t Reg, u8_t* Data);
u8_t L3GD20H_WriteReg(u8_t WriteAddr, u8_t Data);


#endif /* __L3GD20H_H */

/******************* (C) COPYRIGHT 2013 STMicroelectronics *****END OF FILE****/



