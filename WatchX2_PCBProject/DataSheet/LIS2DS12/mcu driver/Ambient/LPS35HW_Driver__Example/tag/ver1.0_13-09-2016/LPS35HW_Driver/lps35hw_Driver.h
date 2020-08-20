/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
* @file    : LPS35HW_driver.h
* @author  : MEMS Application Team
* @version : v1.0
* @date    : 13 September 2016  
* @brief   : LPS35HW source driver file
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
#ifndef __LPS35HW_DRIVER__H
#define __LPS35HW_DRIVER__H

#include <stdint.h>

/* Uncomment the line below to expanse the "assert_param" macro in the  drivers code */
//#define USE_FULL_ASSERT_LPS35HW

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT_LPS35HW

/**
* @brief  The assert_param macro is used for function's parameters check.
* @param  expr: If expr is false, it calls assert_failed function which reports
*         the name of the source file and the source line number of the call
*         that failed. If expr is true, it returns no value.
* @retval None
*/
#define LPS35HW_assert_param(expr) ((expr) ? (void)0 : LPS35HW_assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
void LPS35HW_assert_failed(uint8_t* file, uint32_t line);
#else
#define LPS35HW_assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT_LPS35HW */


#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup Environmental_Sensor
* @{
*/

/** @addtogroup LPS35HW_DRIVER
* @{
*/

/* Exported Types -------------------------------------------------------------*/
/** @defgroup LPS35HW_Exported_Types
* @{
*/

/**
* @brief  Error type.
*/
typedef enum {LPS35HW_OK = (uint8_t)0, LPS35HW_ERROR = !LPS35HW_OK} LPS35HW_Error_et;

/**
* @brief  Enable/Disable type.
*/
typedef enum {LPS35HW_DISABLE = (uint8_t)0, LPS35HW_ENABLE = !LPS35HW_DISABLE} LPS35HW_State_et;
#define IS_LPS35HW_State(MODE) ((MODE == LPS35HW_ENABLE) || (MODE == LPS35HW_DISABLE) )

/**
* @brief  Bit status type.
*/
typedef enum {LPS35HW_RESET = (uint8_t)0, LPS35HW_SET = !LPS35HW_RESET} LPS35HW_BitStatus_et;
#define IS_LPS35HW_BitStatus(MODE) ((MODE == LPS35HW_RESET) || (MODE == LPS35HW_SET))

/*RES_CONF see LC_EN bit*/
 /**
* @brief  LPS35HW Power/Noise Mode configuration.
*/
typedef enum {
  LPS35HW_LowNoise   =  (uint8_t)0x00,       /*!< Low Noise mode */
  LPS35HW_LowPower   =  (uint8_t)0x01        /*!< Low Current mode */
} LPS35HW_PowerMode_et;

#define IS_LPS35HW_PowerMode(MODE) ((MODE == LPS35HW_LowNoise) || (MODE == LPS35HW_LowPower))

/**
* @brief  Output data rate configuration.
*/
typedef enum {
  LPS35HW_ODR_ONE_SHOT  = (uint8_t)0x00,         /*!< Output Data Rate: one shot */
  LPS35HW_ODR_1HZ       = (uint8_t)0x10,         /*!< Output Data Rate: 1Hz */
  LPS35HW_ODR_10HZ      = (uint8_t)0x20,         /*!< Output Data Rate: 10Hz */
  LPS35HW_ODR_25HZ      = (uint8_t)0x30,         /*!< Output Data Rate: 25Hz */
  LPS35HW_ODR_50HZ      = (uint8_t)0x40,         /*!< Output Data Rate: 50Hz */
  LPS35HW_ODR_75HZ      = (uint8_t)0x50          /*!< Output Data Rate: 75Hz */
} LPS35HW_Odr_et;

#define IS_LPS35HW_ODR(ODR) ((ODR == LPS35HW_ODR_ONE_SHOT) || (ODR == LPS35HW_ODR_1HZ) || \
(ODR == LPS35HW_ODR_10HZ) || (ODR == LPS35HW_ODR_25HZ)|| (ODR == LPS35HW_ODR_50HZ) || (ODR == LPS35HW_ODR_75HZ))

/**
* @brief  Low Pass Filter Cutoff Configuration.
*/
typedef enum {

  LPS35HW_ODR_9  = (uint8_t)0x00,         /*!< Filter Cutoff ODR/9 */
  LPS35HW_ODR_20 = (uint8_t)0x04          /*!< Filter Cutoff ODR/20 */
} LPS35HW_LPF_Cutoff_et;

#define IS_LPS35HW_LPF_Cutoff(CUTOFF) ((CUTOFF == LPS35HW_ODR_9) || (CUTOFF == LPS35HW_ODR_20) )

/**
* @brief  Block data update.
*/

typedef enum {
  LPS35HW_BDU_CONTINUOUS_UPDATE     =  (uint8_t)0x00,  /*!< Data updated continuously */
  LPS35HW_BDU_NO_UPDATE             =  (uint8_t)0x02   /*!< Data updated after a read operation */
} LPS35HW_Bdu_et;
#define IS_LPS35HW_BDUMode(MODE) ((MODE == LPS35HW_BDU_CONTINUOUS_UPDATE) || (MODE == LPS35HW_BDU_NO_UPDATE))

/**
* @brief  LPS35HW Spi Mode configuration.
*/
typedef enum {
  LPS35HW_SPI_4_WIRE   =  (uint8_t)0x00,
  LPS35HW_SPI_3_WIRE   =  (uint8_t)0x01
} LPS35HW_SPIMode_et;

#define IS_LPS35HW_SPIMode(MODE) ((MODE == LPS35HW_SPI_4_WIRE) || (MODE == LPS35HW_SPI_3_WIRE))


/**
* @brief  LPS35HW Interrupt Active Level Configuration (on High or Low)
*/
typedef enum
{
  LPS35HW_ActiveHigh = (uint8_t)0x00,
  LPS35HW_ActiveLow  = (uint8_t)0x80
}LPS35HW_InterruptActiveLevel_et;
#define IS_LPS35HW_InterruptActiveLevel(MODE) ((MODE == LPS35HW_ActiveHigh) || (MODE == LPS35HW_ActiveLow))

/**
* @brief  LPS35HW Push-pull/Open Drain selection on Interrupt pads.
*/
typedef enum
{
  LPS35HW_PushPull = (uint8_t)0x00,
  LPS35HW_OpenDrain  = (uint8_t)0x40
}LPS35HW_OutputType_et;
#define IS_LPS35HW_OutputType(MODE) ((MODE == LPS35HW_PushPull) || (MODE == LPS35HW_OpenDrain))


/**
* @brief  Data Signal on INT pad control bits.
*/
typedef enum
{
  LPS35HW_DATA = (uint8_t)0x00,
  LPS35HW_P_HIGH = (uint8_t)0x01,
  LPS35HW_P_LOW = (uint8_t)0x02,
  LPS35HW_P_LOW_HIGH = (uint8_t)0x03
}LPS35HW_OutputSignalConfig_et;
#define IS_LPS35HW_OutputSignal(MODE) ((MODE == LPS35HW_DATA) || (MODE == LPS35HW_P_HIGH)||\
(MODE == LPS35HW_P_LOW) || (MODE == LPS35HW_P_LOW_HIGH))



/**
* @brief  LPS35HW Interrupt Differential Status.
*/

typedef struct
{
  uint8_t PH;          /*!< High Differential Pressure event occured */
  uint8_t PL;          /*!< Low Differential Pressure event occured */
  uint8_t IA;          /*!< One or more interrupt events have been  generated.Interrupt Active */
  uint8_t BOOT;        /*!< i '1' indicates that the Boot (Reboot) phase is running */
}LPS35HW_InterruptDiffStatus_st;


/**
* @brief  LPS35HW Pressure and Temperature data status.
*/
typedef struct
{
  uint8_t TempDataAvailable;    /*!< Temperature data available bit */
  uint8_t PressDataAvailable;   /*!< Pressure data available bit */
  uint8_t TempDataOverrun;      /*!< Temperature data over-run bit */
  uint8_t PressDataOverrun;     /*!< Pressure data over-run bit */
}LPS35HW_DataStatus_st;


/**
* @brief  LPS35HW Clock Tree  configuration.
*/
typedef enum {
  LPS35HW_CTE_NotBalanced   =  (uint8_t)0x00,
  LPS35HW_CTE_Balanced      =  (uint8_t)0x20
} LPS35HW_CTE_et;

#define IS_LPS35HW_CTE(MODE) ((MODE == LPS35HW_CTE_NotBalanced) || (MODE == LPS35HW_CTE_Balanced))

/**
* @brief  LPS35HW Fifo Mode.
*/

typedef enum {
  LPS35HW_FIFO_BYPASS_MODE                    = (uint8_t)0x00,    /*!< The FIFO is disabled and empty. The pressure is read directly*/
  LPS35HW_FIFO_MODE                           = (uint8_t)0x20,    /*!< Stops collecting data when full */
  LPS35HW_FIFO_STREAM_MODE                    = (uint8_t)0x40,    /*!< Keep the newest measurements in the FIFO*/
  LPS35HW_FIFO_TRIGGER_STREAMTOFIFO_MODE      = (uint8_t)0x60,    /*!< STREAM MODE until trigger deasserted, then change to FIFO MODE*/
  LPS35HW_FIFO_TRIGGER_BYPASSTOSTREAM_MODE    = (uint8_t)0x80,    /*!< BYPASS MODE until trigger deasserted, then STREAM MODE*/
  LPS35HW_FIFO_TRIGGER_BYPASSTOFIFO_MODE      = (uint8_t)0xE0     /*!< BYPASS mode until trigger deasserted, then FIFO MODE*/
} LPS35HW_FifoMode_et;

#define IS_LPS35HW_FifoMode(MODE) ((MODE == LPS35HW_FIFO_BYPASS_MODE) || (MODE ==LPS35HW_FIFO_MODE)||\
(MODE == LPS35HW_FIFO_STREAM_MODE) || (MODE == LPS35HW_FIFO_TRIGGER_STREAMTOFIFO_MODE)||\
(MODE == LPS35HW_FIFO_TRIGGER_BYPASSTOSTREAM_MODE) ||  (MODE == LPS35HW_FIFO_TRIGGER_BYPASSTOFIFO_MODE))


/**
* @brief  LPS35HW Fifo Satus.
*/
typedef struct {
  uint8_t FIFO_LEVEL;          /*!< FIFO Stored data level: 00000: FIFO empty; 10000: FIFO is FULL and ha 32 unread samples  */
  uint8_t FIFO_EMPTY;          /*!< Empty FIFO Flag .1 FIFO is empty (see FIFO_level)	*/
  uint8_t FIFO_FULL;           /*!< Full FIFO flag.1 FIFO is Full (see FIFO_level)	*/
  uint8_t FIFO_OVR;            /*!< Overrun bit status. 1 FIFO is full and at least one sample in the FIFO has been overwritten */
  uint8_t FIFO_FTH;            /*!< FIFO Threshold (Watermark) Status. 1 FIFO filling is equal or higher then FTH (wtm) level.*/
}LPS35HW_FifoStatus_st;

/**
* @brief  LPS35HW Configuration structure definition.
*/
typedef struct
{
  LPS35HW_PowerMode_et   PowerMode;       /*!< Enable Low Current Mode (low Power) or Low Noise Mode*/
  LPS35HW_Odr_et         OutputDataRate;  /*!< Output Data Rate */
  LPS35HW_Bdu_et         BDU;             /*!< Enable to inhibit the output registers update between the reading of upper and lower register parts.*/
  LPS35HW_State_et       LowPassFilter;   /*!< Enable/ Disable Low Pass Filter */
  LPS35HW_LPF_Cutoff_et  LPF_Cutoff;      /*!< Low Pass Filter Configuration */
  LPS35HW_SPIMode_et     Sim;             /*!< SPI Serial Interface Mode selection */
  LPS35HW_State_et       IfAddInc;        /*!< Enable/Disable Register address automatically inceremented during a multiple byte access */
}LPS35HW_ConfigTypeDef_st;

/**
* @brief  LPS35HW Interrupt structure definition .
*/
typedef struct {
  LPS35HW_InterruptActiveLevel_et  INT_H_L;         /*!< Interrupt active high, low. Default value: 0 */
  LPS35HW_OutputType_et            PP_OD;           /*!< Push-pull/open drain selection on interrupt pads. Default value: 0 */
  LPS35HW_OutputSignalConfig_et    OutputSignal_INT;/*!< Data signal on INT Pad: Data,Pressure High, Preessure Low,P High or Low*/
  LPS35HW_State_et                 DRDY;            /*!< Enable/Disable Data Ready Interrupt on INT_DRDY Pin*/
  LPS35HW_State_et                 FIFO_OVR;        /*!< Enable/Disable FIFO Overrun Interrupt on INT_DRDY Pin*/
  LPS35HW_State_et                 FIFO_FTH;        /*!< Enable/Disable FIFO threshold (Watermark) interrupt on INT_DRDY pin.*/
  LPS35HW_State_et                 FIFO_FULL;       /*!< Enable/Disable FIFO FULL interrupt on INT_DRDY pin.*/
  LPS35HW_State_et                 LatchIRQ;        /*!< Latch Interrupt request in to INT_SOURCE reg*/
  int16_t                          THS_threshold;   /*!< Threshold value for pressure interrupt generation*/
  LPS35HW_State_et                 AutoRifP;        /*!< Enable/Disable  AutoRifP function */
  LPS35HW_State_et                 AutoZero;        /*!< Enable/Disable  AutoZero function */
}LPS35HW_InterruptTypeDef_st;

/**
* @brief  LPS35HW FIFO structure definition.
*/
typedef struct {
  LPS35HW_FifoMode_et     FIFO_MODE;  /*!< Fifo Mode Selection */
  LPS35HW_State_et        WTM_INT;    /*!< Enable/Disable the watermark interrupt*/
  uint8_t                 WTM_LEVEL;  /*!< FIFO threshold/Watermark level selection*/
}LPS35HW_FIFOTypeDef_st;

#define IS_LPS35HW_WtmLevel(LEVEL) ((LEVEL > 0) && (LEVEL <=31))
/**
* @brief  LPS35HW Measure Type definition.
*/
typedef struct {
  int16_t Tout;
  int32_t Pout;
}LPS35HW_MeasureTypeDef_st;


/**
* @brief  LPS35HW Driver Version Info structure definition.
*/
typedef struct {
  uint8_t   Major;
  uint8_t   Minor;
  uint8_t Point;
}LPS35HW_DriverVersion_st;


/**
* @brief  Bitfield positioning.
*/
#define LPS35HW_BIT(x) ((uint8_t)x)

/**
* @brief  I2C address.
*/
/* SD0/SA0(pin 5) is connected to the voltage supply*/
//#define LPS35HW_ADDRESS  (uint8_t)0xBA
/*SDO/SA0 (pin5) is connected to the GND*/
#define LPS35HW_ADDRESS  (uint8_t)0xB8

/**
* @brief  Set the LPS35HW driver version.
*/

#define LPS35HW_DriverVersion_Major (uint8_t)1
#define LPS35HW_DriverVersion_Minor (uint8_t)0
#define LPS35HW_DriverVersion_Point (uint8_t)0

/**
* @}
*/


/* Exported Constants ---------------------------------------------------------*/
/** @defgroup LPS35HW_Exported_Constants
* @{
*/


/**
* @addtogroup LPS35HW_Register
* @{
*/



/**
* @brief Device Identification register.
* \code
* Read
* Default value: 0xB1
* 7:0 This read-only register contains the device identifier that, for LPS35HW, is set to B1h.
* \endcode
*/

#define LPS35HW_WHO_AM_I_REG         (uint8_t)0x0F

/**
* @brief Device Identification value.
*/
#define LPS35HW_WHO_AM_I_VAL         (uint8_t)0xB1


/**
* @brief Reference Pressure  Register(LSB data)
* \code
* Read/write
* Default value: 0x00
* 7:0 REFL7-0: Lower part of the reference pressure value that
*      is sum to the sensor output pressure.
* \endcode
*/
#define LPS35HW_REF_P_XL_REG         (uint8_t)0x15


/**
* @brief Reference Pressure Register (Middle data)
* \code
* Read/write
* Default value: 0x00
* 7:0 REFL15-8: Middle part of the reference pressure value that
*      is sum to the sensor output pressure.
* \endcode
*/
#define LPS35HW_REF_P_L_REG          (uint8_t)0x16

/**
* @brief Reference Pressure Register (MSB data)
* \code
* Read/write
* Default value: 0x00
* 7:0 REFL23-16 Higest part of the reference pressure value that
*      is sum to the sensor output pressure.
* \endcode
*/
#define LPS35HW_REF_P_H_REG          (uint8_t)0x17


/**
* @brief Pressure and temperature resolution mode Register
* \code
* Read/write
* Default value: 0x05
* 7:2 These bits must be set to 0 for proper operation of the device
* 1: Reserved
* 0 LC_EN: Low Current Mode Enable. Default 0
* \endcode
*/
#define LPS35HW_RES_CONF_REG     (uint8_t)0x1A

#define LPS35HW_LCEN_MASK        (uint8_t)0x01

/**
* @brief Control Register 1
* \code
* Read/write
* Default value: 0x00
* 7: This bit must be set to 0 for proper operation of the device
* 6:4 ODR2, ODR1, ODR0: output data rate selection.Default 000
*     ODR2  | ODR1  | ODR0  | Pressure output data-rate(Hz)  | Pressure output data-rate(Hz)
*   ----------------------------------------------------------------------------------
*      0    |  0    |  0    |         one shot               |         one shot
*      0    |  0    |  1    |            1                   |            1
*      0    |  1    |  0    |            10                  |           10
*      0    |  1    |  1    |            25                  |           25
*      1    |  0    |  0    |            50                  |           50
*      1    |  0    |  1    |            75                  |         75
*      1    |  1    |  0    |         Reserved               |         Reserved
*      1    |  1    |  1    |         Reserved               |         Reserved
*
* 3 EN_LPFP: Enable Low Pass filter on Pressure data. Default value:0
* 2:LPF_CFG Low-pass configuration register. (0: Filter cutoff is ODR/9; 1: filter cutoff is ODR/20)
* 1 BDU: block data update. 0 - continuous update; 1 - output registers not updated until MSB and LSB reading.
* 0 SIM: SPI Serial Interface Mode selection. 0 - SPI 4-wire; 1 - SPI 3-wire
* \endcode
*/
#define LPS35HW_CTRL_REG1      (uint8_t)0x10

#define LPS35HW_ODR_MASK                (uint8_t)0x70
#define LPS35HW_LPFP_MASK               (uint8_t)0x08
#define LPS35HW_LPFP_CUTOFF_MASK        (uint8_t)0x04
#define LPS35HW_BDU_MASK                (uint8_t)0x02
#define LPS35HW_SIM_MASK                (uint8_t)0x01

#define LPS35HW_LPFP_BIT    LPS35HW_BIT(3)


/**
* @brief Control  Register 2
* \code
* Read/write
* Default value: 0x10
* 7 BOOT:  Reboot memory content. 0: normal mode; 1: reboot memory content. Self-clearing upon completation
* 6 FIFO_EN: FIFO Enable. 0: disable; 1:  enable
* 5 STOP_ON_FTH: Stop on FIFO Threshold  FIFO Watermark level use. 0: disable; 1: enable
* 4 IF_ADD_INC: Register address automatically incrementeed during a multiple byte access with a serial interface (I2C or SPI). Default value 1.( 0: disable; 1: enable)
* 3 I2C DIS:  Disable I2C interface 0: I2C Enabled; 1: I2C disabled
* 2 SWRESET: Software reset. 0: normal mode; 1: SW reset. Self-clearing upon completation
* 1 AUTO_ZERO: Autozero enable. 0: normal mode; 1: autozero enable.
* 0 ONE_SHOT: One shot enable. 0: waiting for start of conversion; 1: start for a new dataset
* \endcode
*/
#define LPS35HW_CTRL_REG2      (uint8_t)0x11

#define LPS35HW_BOOT_BIT       LPS35HW_BIT(7)
#define LPS35HW_FIFO_EN_BIT    LPS35HW_BIT(6)
#define LPS35HW_WTM_EN_BIT     LPS35HW_BIT(5)
#define LPS35HW_ADD_INC_BIT    LPS35HW_BIT(4)
#define LPS35HW_I2C_BIT        LPS35HW_BIT(3)
#define LPS35HW_SW_RESET_BIT   LPS35HW_BIT(2)

#define LPS35HW_FIFO_EN_MASK   (uint8_t)0x40
#define LPS35HW_WTM_EN_MASK    (uint8_t)0x20
#define LPS35HW_ADD_INC_MASK   (uint8_t)0x10
#define LPS35HW_I2C_MASK       (uint8_t)0x08
#define LPS35HW_ONE_SHOT_MASK  (uint8_t)0x01


/**
* @brief CTRL Reg3 Interrupt Control Register
* \code
* Read/write
* Default value: 0x00
* 7 INT_H_L: Interrupt active high, low. 0:active high; 1: active low.
* 6 PP_OD: Push-Pull/OpenDrain selection on interrupt pads. 0: Push-pull; 1: open drain.
* 5 F_FSS5: FIFO full flag on INT_DRDY pin. Defaul value: 0. (0: Diasable; 1 : Enable).
* 4 F_FTH: FIFO threshold (watermark) status on INT_DRDY pin. Defaul value: 0. (0: Diasable; 1 : Enable).
* 3 F_OVR: FIFO overrun interrupt on INT_DRDY pin. Defaul value: 0. (0: Diasable; 1 : Enable).
* 2 DRDY: Data-ready signal on INT_DRDY pin. Defaul value: 0. (0: Diasable; 1 : Enable).
* 1:0 INT_S2, INT_S1: data signal on INT pad control bits.
*    INT_S2  | INT_S1  | INT pin
*   ------------------------------------------------------
*        0       |      0      |     Data signal( in order of priority:PTH_DRDY or F_FTH or F_OVR_or F_FSS5
*        0       |      1      |     Pressure high (P_high)
*        1       |      0      |     Pressure low (P_low)
*        1       |      1      |     P_low OR P_high
* \endcode
*/
#define LPS35HW_CTRL_REG3      (uint8_t)0x12

#define LPS35HW_PP_OD_BIT       LPS35HW_BIT(6)
#define LPS35HW_FIFO_FULL_BIT   LPS35HW_BIT(5)
#define LPS35HW_FIFO_FTH_BIT    LPS35HW_BIT(4)
#define LPS35HW_FIFO_OVR_BIT    LPS35HW_BIT(3)
#define LPS35HW_DRDY_BIT        LPS35HW_BIT(2)


#define LPS35HW_INT_H_L_MASK            (uint8_t)0x80
#define LPS35HW_PP_OD_MASK              (uint8_t)0x40
#define LPS35HW_FIFO_FULL_MASK          (uint8_t)0x20
#define LPS35HW_FIFO_FTH_MASK           (uint8_t)0x10
#define LPS35HW_FIFO_OVR_MASK           (uint8_t)0x08
#define LPS35HW_DRDY_MASK               (uint8_t)0x04
#define LPS35HW_INT_S12_MASK            (uint8_t)0x03


/**
* @brief Interrupt Differential configuration Register
* \code
* Read/write
* Default value: 0x00.
* 7 AUTORIFP: AutoRifP Enable ??
* 6 RESET_ARP: Reset AutoRifP function
* 4 AUTOZERO: Autozero enabled
* 5 RESET_AZ: Reset Autozero Function
* 3 DIFF_EN: Interrupt generation enable
* 2 LIR: Latch Interrupt request into INT_SOURCE register. 0 - interrupt request not latched; 1 - interrupt request latched
* 1 PL_E: Enable interrupt generation on differential pressure low event. 0 - disable; 1 - enable
* 0 PH_E: Enable interrupt generation on differential pressure high event. 0 - disable; 1 - enable
* \endcode
*/
#define LPS35HW_INTERRUPT_CFG_REG  (uint8_t)0x0B

#define LPS35HW_DIFF_EN_BIT       LPS35HW_BIT(3)
#define LPS35HW_LIR_BIT           LPS35HW_BIT(2)
#define LPS35HW_PLE_BIT           LPS35HW_BIT(1)
#define LPS35HW_PHE_BIT           LPS35HW_BIT(0)

#define LPS35HW_AUTORIFP_MASK     (uint8_t)0x80
#define LPS35HW_RESET_ARP_MASK    (uint8_t)0x40
#define LPS35HW_AUTOZERO_MASK     (uint8_t)0x20
#define LPS35HW_RESET_AZ_MASK     (uint8_t)0x10
#define LPS35HW_DIFF_EN_MASK      (uint8_t)0x08
#define LPS35HW_LIR_MASK          (uint8_t)0x04
#define LPS35HW_PLE_MASK          (uint8_t)0x02
#define LPS35HW_PHE_MASK          (uint8_t)0x01



/**
* @brief Interrupt source Register (It is cleared by reading it)
* \code
* Read
* Default value: ----.
* 7 BOOT_STATUS:  If 1 indicates that the Boot (Reboot) phase is running.
* 6:3 Reserved: Keep these bits at 0
* 2 IA: Interrupt Active.0: no interrupt has been generated; 1: one or more interrupt events have been generated.
* 1 PL: Differential pressure Low. 0: no interrupt has been generated; 1: Low differential pressure event has occurred.
* 0 PH: Differential pressure High. 0: no interrupt has been generated; 1: High differential pressure event has occurred.
* \endcode
*/
#define LPS35HW_INTERRUPT_SOURCE_REG   (uint8_t)0x25

#define LPS35HW_BOOT_STATUS_BIT        LPS35HW_BIT(7)
#define LPS35HW_IA_BIT                 LPS35HW_BIT(2)
#define LPS35HW_PL_BIT                 LPS35HW_BIT(1)
#define LPS35HW_PH_BIT                 LPS35HW_BIT(0)

#define LPS35HW_BOOT_STATUS_MASK      (uint8_t)0x80
#define LPS35HW_IA_MASK               (uint8_t)0x04
#define LPS35HW_PL_MASK               (uint8_t)0x02
#define LPS35HW_PH_MASK               (uint8_t)0x01


/**
* @brief  Status Register
* \code
* Read
* Default value: ---
* 7:6 Reserved: 0
* 5 T_OR: Temperature data overrun. 0: no overrun has occurred; 1: a new data for temperature has overwritten the previous one.
* 4 P_OR: Pressure data overrun. 0: no overrun has occurred; 1: new data for pressure has overwritten the previous one.
* 3:2 Reserved: 0
* 1 T_DA: Temperature data available. 0: new data for temperature is not yet available; 1: new data for temperature is available.
* 0 P_DA: Pressure data available. 0: new data for pressure is not yet available; 1: new data for pressure is available.
* \endcode
*/
#define LPS35HW_STATUS_REG         (uint8_t)0x27

#define LPS35HW_TOR_BIT            LPS35HW_BIT(5)
#define LPS35HW_POR_BIT            LPS35HW_BIT(4)
#define LPS35HW_TDA_BIT            LPS35HW_BIT(1)
#define LPS35HW_PDA_BIT            LPS35HW_BIT(0)

#define LPS35HW_TOR_MASK           (uint8_t)0x20
#define LPS35HW_POR_MASK           (uint8_t)0x10
#define LPS35HW_TDA_MASK           (uint8_t)0x02
#define LPS35HW_PDA_MASK           (uint8_t)0x01



/**
* @brief  Pressure data (LSB) register.
* \code
* Read
* Default value: 0x00.(To be verified)
* POUT7 - POUT0: Pressure data LSB (2's complement).
* Pressure output data: Pout(hPA)=(PRESS_OUT_H & PRESS_OUT_L &
* PRESS_OUT_XL)[dec]/4096.
* \endcode
*/

#define LPS35HW_PRESS_OUT_XL_REG        (uint8_t)0x28
/**
* @brief  Pressure data (Middle part) register.
* \code
* Read
* Default value: 0x80.
* POUT15 - POUT8: Pressure data middle part (2's complement).
* Pressure output data: Pout(hPA)=(PRESS_OUT_H & PRESS_OUT_L &
* PRESS_OUT_XL)[dec]/4096.
* \endcode
*/
#define LPS35HW_PRESS_OUT_L_REG        (uint8_t)0x29

/**
* @brief  Pressure data (MSB) register.
* \code
* Read
* Default value: 0x2F.
* POUT23 - POUT16: Pressure data MSB (2's complement).
* Pressure output data: Pout(hPA)=(PRESS_OUT_H & PRESS_OUT_L &
* PRESS_OUT_XL)[dec]/4096.
* \endcode
*/
#define LPS35HW_PRESS_OUT_H_REG        (uint8_t)0x2A

/**
* @brief  Temperature data (LSB) register.
* \code
* Read
* Default value: 0x00.
* TOUT7 - TOUT0: temperature data LSB.
* Tout(degC)=TEMP_OUT/100
* \endcode
*/
#define LPS35HW_TEMP_OUT_L_REG         (uint8_t)0x2B

/**
* @brief  Temperature data (MSB) register.
* \code
* Read
* Default value: 0x00.
* TOUT15 - TOUT8: temperature data MSB.
* Tout(degC)=TEMP_OUT/100
* \endcode
*/
#define LPS35HWH_TEMP_OUT_H_REG         (uint8_t)0x2C

/**
* @brief Threshold pressure (LSB) register.
* \code
* Read/write
* Default value: 0x00.
* 7:0 THS7-THS0: LSB Threshold pressure Low part of threshold value for pressure interrupt
* generation. The complete threshold value is given by THS_P_H & THS_P_L and is
* expressed as unsigned number. P_ths(hPA)=(THS_P_H & THS_P_L)[dec]/16.
* \endcode
*/
#define LPS35HW_THS_P_LOW_REG           (uint8_t)0x0C

/**
* @brief Threshold pressure (MSB)
* \code
* Read/write
* Default value: 0x00.
* 7:0 THS15-THS8: MSB Threshold pressure. High part of threshold value for pressure interrupt
* generation. The complete threshold value is given by THS_P_H & THS_P_L and is
* expressed as unsigned number. P_ths(mbar)=(THS_P_H & THS_P_L)[dec]/16.
* \endcode
*/
#define LPS35HW_THS_P_HIGH_REG         (uint8_t)0x0D

/**
* @brief FIFO control register
* \code
* Read/write
* Default value: 0x00
* 7:5 F_MODE2, F_MODE1, F_MODE0: FIFO mode selection.
*     FM2   | FM1   | FM0   |    FIFO MODE
*   ---------------------------------------------------
*      0    |  0    |  0    | BYPASS MODE
*      0    |  0    |  1    | FIFO MODE. Stops collecting data when full
*      0    |  1    |  0    | STREAM MODE: Keep the newest measurements in the FIFO
*      0    |  1    |  1    | STREAM MODE until trigger deasserted, then change to FIFO MODE
*      1    |  0    |  0    | BYPASS MODE until trigger deasserted, then STREAM MODE
*      1    |  0    |  1    | Reserved for future use
*      1    |  1    |  0    | Reserved
*      1    |  1    |  1    | BYPASS mode until trigger deasserted, then FIFO MODE
*
* 4:0 WTM_POINT4-0 : FIFO Watermark level selection (0-31)
*/
#define LPS35HW_CTRL_FIFO_REG          (uint8_t)0x14

#define LPS35HW_FIFO_MODE_MASK        (uint8_t)0xE0
#define LPS35HW_WTM_POINT_MASK        (uint8_t)0x1F


/**
* @brief FIFO Status register
* \code
* Read
* Default value: ----
* 7 FTH_FIFO: FIFO threshold status. 0:FIFO filling is lower than FTH level; 1: FIFO is equal or higher than FTH level.
* 6 OVR: Overrun bit status. 0 - FIFO not full; 1 -FIFO is full and at least one sample in the FIFO has been overwritten.
* 5:0 FSS: FIFO Stored data level. 000000: FIFO empty, 100000: FIFO is full and has 32 unread samples.
* \endcode
*/
#define LPS35HW_STATUS_FIFO_REG        (uint8_t)0x26

#define LPS35HW_FTH_FIFO_BIT          LPS35HW_BIT(7)
#define LPS35HW_OVR_FIFO_BIT          LPS35HW_BIT(6)

#define LPS35HW_FTH_FIFO_MASK         (uint8_t)0x80
#define LPS35HW_OVR_FIFO_MASK         (uint8_t)0x40
#define LPS35HW_LEVEL_FIFO_MASK       (uint8_t)0x3F
#define LPS35HW_FIFO_EMPTY            (uint8_t)0x00
#define LPS35HW_FIFO_FULL             (uint8_t)0x20



/**
* @brief Pressure offset register  (LSB)
* \code
* Read/write
* Default value: 0x00
* 7:0 RPDS7-0:Pressure Offset for 1 point calibration (OPC) after soldering.
* This register contains the low part of the pressure offset value after soldering,for
* differential pressure computing. The complete value is given by RPDS_L & RPDS_H
* and is expressed as signed 2 complement value.
* \endcode
*/
#define LPS35HW_RPDS_L_REG        (uint8_t)0x18

/**
* @brief Pressure offset register (MSB)
* \code
* Read/write
* Default value: 0x00
* 7:0 RPDS15-8:Pressure Offset for 1 point calibration  (OPC) after soldering.
* This register contains the high part of the pressure offset value after soldering (see description RPDS_L)
* \endcode
*/
#define LPS35HW_RPDS_H_REG        (uint8_t)0x19


/**
* @brief Clock Tree Configuration register
* \code
* Read/write
* Default value: 0x00
* 7:6 Reserved.
* 5: CTE: Clock Tree Enhancement
* \endcode
*/

#define LPS35HW_CLOCK_TREE_CONFIGURATION        (uint8_t)0x43

#define LPS35HW_CTE_MASK           (uint8_t)0x20

/**
* @}
*/


/**
* @}
*/


/* Exported Functions -------------------------------------------------------------*/
/** @defgroup LPS35HW_Exported_Functions
* @{
*/

LPS35HW_Error_et LPS35HW_ReadReg( void *handle, uint8_t RegAddr, uint16_t NumByteToRead, uint8_t *Data );
LPS35HW_Error_et LPS35HW_WriteReg( void *handle, uint8_t RegAddr, uint16_t NumByteToWrite, uint8_t *Data );

/**
* @brief  Init the HAL layer.
* @param  None
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
#define LPS35HW_HalInit  (LPS35HW_Error_et)HAL_Init_I2C

/**
* @brief  DeInit the HAL layer.
* @param  None
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
#define LPS35HW_HalDeInit  (LPS35HW_Error_et)HAL_DeInit_I2C


/**
* @brief  Get the LPS35HW driver version.
* @param  None
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Get_DriverVersion(LPS35HW_DriverVersion_st *Version);

/**
* @brief  Initialization function for LPS35HW.
*         This function make a memory boot.
*         Init the sensor with a standard basic confifuration.
*         Low Power, ODR 25 Hz, Low Pass Filter disabled; BDU enabled; I2C enabled;
*         NO FIFO; NO Interrupt Enabled.
* @param  None.
* @retval Error code[LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Init(void *handle);

/**
* @brief  DeInit the LPS2Hb driver.
* @param  None
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/

LPS35HW_Error_et LPS35HW_DeInit(void *handle);


/**
* @brief  Read identification code by WHO_AM_I register
* @param  Buffer to empty by Device identification Value.
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Get_DeviceID(void *handle, uint8_t* deviceid);


/**
* @brief  Set LPS35HW Low Power or Low Noise Mode Configuration
* @param  LPS35HW_LowNoise or LPS35HW_LowPower mode
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Set_PowerMode(void *handle, LPS35HW_PowerMode_et mode);

/**
* @brief  Get LPS35HW Power Mode
* @param   Buffer to empty with Mode: Low Noise or Low Current
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Get_PowerMode(void *handle, LPS35HW_PowerMode_et* mode);


/**
* @brief  Set LPS35HW Output Data Rate
* @param  Output Data Rate
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Set_Odr(void *handle, LPS35HW_Odr_et odr);


/**
* @brief  Get LPS35HW Output Data Rate
* @param  Buffer to empty with Output Data Rate
* @retval  Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Get_Odr(void *handle, LPS35HW_Odr_et* odr);

/**
* @brief  Enable/Disale low-pass filter on LPS35HW pressure data
* @param  state: enable or disable
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Set_LowPassFilter(void *handle, LPS35HW_State_et state);


/**
* @brief  Set low-pass filter cutoff configuration on LPS35HW pressure data
* @param Filter Cutoff ODR/9 or ODR/20
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Set_LowPassFilterCutoff(void *handle, LPS35HW_LPF_Cutoff_et cutoff);

/**
* @brief  Set Block Data Update mode
* @param  LPS35HW_BDU_CONTINUOS_UPDATE/ LPS35HW_BDU_NO_UPDATE
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Set_Bdu(void *handle, LPS35HW_Bdu_et bdu);


/**
* @brief  Get Block Data Update mode
* @param  Buffer to empty whit the bdu mode read from sensor
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Get_Bdu(void *handle, LPS35HW_Bdu_et* bdu);

/**
* @brief  Set SPI mode: 3 Wire Interface OR 4 Wire Interface
* @param  LPS35HW_SPI_4_WIRE/LPS35HW_SPI_3_WIRE
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Set_SpiInterface(void *handle, LPS35HW_SPIMode_et spimode);

/**
* @brief  Get SPI mode: 3 Wire Interface OR 4 Wire Interface
* @param  buffer to empty with SPI mode
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Get_SpiInterface(void *handle, LPS35HW_SPIMode_et* spimode);

/**
* @brief Software Reset
* @param  void
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_SwReset(void *handle);

/**
* @brief Reboot Memory Content.
* @param  void
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_MemoryBoot(void *handle);

/**
* @brief Software Reset ann BOOT
* @param  void
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_SwResetAndMemoryBoot(void *handle);


/**
* @brief  Enable or Disable FIFO
* @param  LPS35HW_ENABLE/LPS35HW_DISABLE
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Set_FifoModeUse(void *handle, LPS35HW_State_et status);

/**
* @brief  Enable or Disable FIFO Watermark level use. Stop on FIFO Threshold
* @param  LPS35HW_ENABLE/LPS35HW_DISABLE
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Set_FifoWatermarkLevelUse(void *handle, LPS35HW_State_et status);

/**
* @brief  Enable or Disable the Automatic increment register address during a multiple byte access with a serial interface (I2C or SPI)
* @param  LPS35HW_ENABLE/LPS35HW_DISABLE. Default is LPS35HW_ENABLE
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Set_AutomaticIncrementRegAddress(void *handle, LPS35HW_State_et status);


/**
* @brief  Set One Shot bit to start a new conversion (ODR mode has to be 000)
* @param  void
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_StartOneShotMeasurement(void *handle);

/**
* @brief  Enable/Disable I2C
* @param  State. Enable (reset bit)/ Disable (set bit)
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Set_I2C(void *handle, LPS35HW_State_et i2cstate);


/*CTRL_REG3 Interrupt Control*/
/**
* @brief  Set Interrupt Active on High or Low Level
* @param  LPS35HW_ActiveHigh/LPS35HW_ActiveLow
* @retval  Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Set_InterruptActiveLevel(void *handle, LPS35HW_InterruptActiveLevel_et mode);

/**
* @brief  Set Push-pull/open drain selection on interrupt pads.
* @param  LPS35HW_PushPull/LPS35HW_OpenDrain
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Set_InterruptOutputType(void *handle, LPS35HW_OutputType_et output);

/**
* @brief  Set Data signal on INT1 pad control bits.
* @param  LPS35HW_DATA,LPS35HW_P_HIGH_LPS35HW_P_LOW,LPS35HW_P_LOW_HIGH
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Set_InterruptControlConfig(void *handle, LPS35HW_OutputSignalConfig_et config);


/**
* @brief   Enable/Disable Data-ready signal on INT_DRDY pin.
* @param  LPS35HW_ENABLE/LPS35HW_DISABLE
* @retval  Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Set_DRDYInterrupt(void *handle, LPS35HW_State_et status);

 /**
* @brief   Enable/Disable FIFO overrun interrupt on INT_DRDY pin.
* @param  LPS35HW_ENABLE/LPS35HW_DISABLE
* @retval  Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Set_FIFO_OVR_Interrupt(void *handle, LPS35HW_State_et status);

 /**
* @brief   Enable/Disable FIFO threshold (Watermark) interrupt on INT_DRDY pin.
* @param  LPS35HW_ENABLE/LPS35HW_DISABLE
* @retval  Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Set_FIFO_FTH_Interrupt(void *handle, LPS35HW_State_et status);

/**
* @brief   Enable/Disable FIFO FULL interrupt on INT_DRDY pin.
* @param  LPS35HW_ENABLE/LPS35HW_DISABLE
* @retval  Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Set_FIFO_FULL_Interrupt(void *handle, LPS35HW_State_et status);

/**
* @brief   Enable AutoRifP function
* @param   none
* @detail When this function is enabled, an internal register is set with the current pressure values
*         and the content is subtracted from the pressure output value and result is used for the interrupt generation.
*        the AutoRifP is slf creared.
* @retval  Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Set_AutoRifP(void *handle);

/**
* @brief   Disable AutoRifP
* @param   none
* @detail  the RESET_ARP bit is used to disable the AUTORIFP function. This bis i is selfdleared
* @retval  Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_ResetAutoRifP(void *handle);

/**?????
* @brief  Set AutoZero Function bit
* @detail When set to ‘1’, the actual pressure output is copied in the REF_P reg (@0x15..0x17)
* @param  None
* @retval  Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Set_AutoZeroFunction(void *handle);

/**???
* @brief  Set ResetAutoZero Function bit
* @details REF_P reg (@0x015..17) set pressure reference to default value RPDS reg (0x18/19).
* @param  None
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_ResetAutoZeroFunction(void *handle);


/**
* @brief  Enable/ Disable the computing of differential pressure output (Interrupt Generation)
* @param  LPS35HW_ENABLE,LPS35HW_DISABLE
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Set_InterruptDifferentialGeneration(void *handle, LPS35HW_State_et diff_en) ;



/**
* @brief  Get the DIFF_EN bit value
* @param  buffer to empty with the read value of DIFF_EN bit
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Get_InterruptDifferentialGeneration(void *handle, LPS35HW_State_et* diff_en);


/**
* @brief  Latch Interrupt request to the INT_SOURCE register.
* @param  LPS35HW_ENABLE/LPS35HW_DISABLE
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_LatchInterruptRequest(void *handle, LPS35HW_State_et status);

/**
* @brief  Enable\Disable Interrupt Generation on differential pressure Low event
* @param  LPS35HW_ENABLE/LPS35HW_DISABLE
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Set_PLE(void *handle, LPS35HW_State_et status);

/**
* @brief  Enable\Disable Interrupt Generation on differential pressure High event
* @param  LPS35HW_ENABLE/LPS35HW_DISABLE
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Set_PHE(void *handle, LPS35HW_State_et status);

/**
* @brief   Get the Interrupt Generation on differential pressure status event and the Boot Status.
* @detail  The INT_SOURCE register is cleared by reading it.
* @param   Status Event Flag: BOOT, PH,PL,IA
* @retval  Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Get_InterruptDifferentialEventStatus(void *handle, LPS35HW_InterruptDiffStatus_st* interruptsource);


/**
* @brief  Get the status of Pressure and Temperature data
* @param  Data Status Flag:  TempDataAvailable, TempDataOverrun, PressDataAvailable, PressDataOverrun
* @retval  Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Get_DataStatus(void *handle, LPS35HW_DataStatus_st* datastatus);


/**
* @brief  Get the LPS35HW raw presure value
* @param  The buffer to empty with the pressure raw value
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Get_RawPressure(void *handle, int32_t *raw_press);

/**
* @brief  Get the LPS35HW Pressure value in hPA.
* @param  The buffer to empty with the pressure value that must be divided by 100 to get the value in hPA
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Get_Pressure(void *handle, int32_t* Pout);

/**
* @brief  Read LPS35HW output register, and calculate the raw temperature.
* @param  The buffer to empty with the temperature raw value
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Get_RawTemperature(void *handle, int16_t *raw_data);

/**
* @brief  Read the Temperature value in °C.
* @param  The buffer to empty with the temperature value that must be divided by 10 to get the value in ['C]
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Get_Temperature(void *handle, int16_t* Tout);

/**
* @brief  Get the threshold value used for pressure interrupt generation.
* @param  The buffer to empty with the temperature value
* @retval  Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Get_PressureThreshold(void *handle, int16_t *P_ths);

/**
* @brief  Set the threshold value used for pressure interrupt generation.
* @param  The buffer to empty with the temperature value
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Set_PressureThreshold(void *handle, int16_t P_ths);

/**
* @brief  Set Fifo Mode.
* @param  Fifo Mode struct
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Set_FifoMode(void *handle, LPS35HW_FifoMode_et fifomode);
/**
* @brief  Get Fifo Mode.
* @param  Buffer to empty with fifo mode value
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Get_FifoMode(void *handle, LPS35HW_FifoMode_et* fifomode);

/**
* @brief  Set Fifo Watermark Level.
* @param  Watermark level value [0 31]
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Set_FifoWatermarkLevel(void *handle, uint8_t wtmlevel);

/**
* @brief   Get FIFO Watermark Level
* @param   buffer to empty with watermak level[0,31] value read from sensor
* @retval  Status [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Get_FifoWatermarkLevel(void *handle, uint8_t *wtmlevel);


/**
* @brief  Get Fifo Status.
* @param  Buffer to empty with fifo status
* @retval Status [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Get_FifoStatus(void *handle, LPS35HW_FifoStatus_st* status);


/**
* @brief  Get the reference pressure after soldering for computing differential pressure (hPA)
* @param buffer to empty with the he pressure value (hPA)
* @retval  Status [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Get_PressureOffsetValue(void *handle, int16_t *pressoffset);

/**
* @brief  Get the Reference Pressure value
* @detail  It is a 24-bit data added to the sensor output measurement to detect a measured pressure beyond programmed limits.
* @param  Buffer to empty with reference pressure value
* @retval  Status [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Get_ReferencePressure(void *handle, int32_t* RefP);


/**
* @brief  Check if the single measurement has completed.
* @param  the returned value is set to 1, when the measurement is completed
* @retval Status [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_IsMeasurementCompleted(void *handle, uint8_t* Is_Measurement_Completed);


/**
* @brief  Get the values of the last single measurement.
* @param  Pressure and temperature value
* @retvalStatus [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Get_Measurement(void *handle, LPS35HW_MeasureTypeDef_st *Measurement_Value);


/**
* @brief   Set Generic Configuration
* @param   Struct to empty with the chosen values
* @retval  Error code[LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Set_GenericConfig(void *handle, LPS35HW_ConfigTypeDef_st* pxLPS35HWInit);

/**
* @brief  Get Generic configuration
* @param  Struct to empty with configuration values
* @retval Error code[LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Get_GenericConfig(void *handle, LPS35HW_ConfigTypeDef_st* pxLPS35HWInit);

/**
* @brief  Set Interrupt configuration
* @param  Struct holding the configuration values
* @retval  Error code[LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Set_InterruptConfig(void *handle, LPS35HW_InterruptTypeDef_st* pLPS35HWInt);

/**
* @brief  LPS35HWGet_InterruptConfig
* @param  Struct to empty with configuration values
* @retval S Error code[LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Get_InterruptConfig(void *handle, LPS35HW_InterruptTypeDef_st* pLPS35HWInt);

/**
* @brief  Set Fifo configuration
* @param  Struct holding the configuration values
* @retval  Error code[LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Set_FifoConfig(void *handle, LPS35HW_FIFOTypeDef_st* pLPS35HWFIFO);

/**
* @brief  Get Fifo configuration
* @param  Struct to empty with the configuration values
* @retval Error code[LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Get_FifoConfig(void *handle, LPS35HW_FIFOTypeDef_st* pLPS35HWFIFO);

/**
* @brief  Clock Tree Confoguration
* @param  LPS35HW_CTE_NotBalanced, LPS35HW_CTE_ABalanced
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*/
LPS35HW_Error_et LPS35HW_Set_ClockTreeConfifuration(void *handle, LPS35HW_CTE_et mode);


#ifdef __cplusplus
}
#endif


/**
* @}
*/

/**
* @}
*/

/**
* @}
*/

#endif /* __LPS35HW_DRIVER__H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
