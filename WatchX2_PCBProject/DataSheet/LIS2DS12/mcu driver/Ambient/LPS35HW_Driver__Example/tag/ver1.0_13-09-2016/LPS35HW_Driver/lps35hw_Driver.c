/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
* @file    : LPS35HW_driver.c
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

/* Includes ------------------------------------------------------------------*/
#include "LPS35HW_Driver.h"
#ifdef  USE_FULL_ASSERT_LPS35HW
#include <stdio.h>
#endif

/** @addtogroup Environmental_Sensor
* @{
*/

/** @defgroup LPS35HW_DRIVER
* @brief LPS35HW DRIVER
* @{
*/

/** @defgroup LPS35HW_Imported_Function_Prototypes
* @{
*/
/* Example: */
/* extern uint8_t Sensor_IO_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite ); */
/* extern uint8_t Sensor_IO_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead ); */

/**
* @}
*/

/** @defgroup LPS35HW_Private_Function_Prototypes
* @{
*/

/**
* @}
*/

/** @defgroup LPS35HW_Private_Functions
* @{
*/

/**
* @}
*/

/** @defgroup LPS35HW_Public_Functions
* @{
*/

/*******************************************************************************
* Function Name   : LPS35HW_ReadReg
* Description   : Generic Reading function. It must be fullfilled with either
*         : I2C or SPI reading functions
* Input       : Register Address
* Output      : Data Read
* Return      : None
*******************************************************************************/
LPS35HW_Error_et LPS35HW_ReadReg( void *handle, uint8_t RegAddr, uint16_t NumByteToRead, uint8_t *Data )
{
  /* Example: */
  /*
  if ( Sensor_IO_Read( handle, RegAddr, Data, NumByteToRead ) )
    return LPS35HW_ERROR;
  else
    return LPS35HW_OK;
  */
}

/*******************************************************************************
* Function Name   : LPS35HW_WriteReg
* Description   : Generic Writing function. It must be fullfilled with either
*         : I2C or SPI writing function
* Input       : Register Address, Data to be written
* Output      : None
* Return      : None
*******************************************************************************/
LPS35HW_Error_et LPS35HW_WriteReg( void *handle, uint8_t RegAddr, uint16_t NumByteToWrite, uint8_t *Data )
{
  /* Example: */
  /*
  if ( Sensor_IO_Write( handle, RegAddr, Data, NumByteToWrite ) )
    return LPS35HW_ERROR;
  else
    return LPS35HW_OK;
  */
}

/*******************************************************************************
* @brief  Read identification code by WHO_AM_I register
* @param *handle Device handle.
* @param  Buffer to empty by Device identification Value.
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Get_DeviceID(void *handle, uint8_t* deviceid)
{
  if(LPS35HW_ReadReg(handle, LPS35HW_WHO_AM_I_REG, 1, deviceid))
    return LPS35HW_ERROR;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief  Get the LPS35HW driver version.
* @param  None
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Get_DriverVersion(LPS35HW_DriverVersion_st *Version)
{
  Version->Major = LPS35HW_DriverVersion_Major;
  Version->Minor = LPS35HW_DriverVersion_Minor;
  Version->Point = LPS35HW_DriverVersion_Point;
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief  Set LPS35HW Low Power or Low Noise Mode Configuration
* @param *handle Device handle.
* @param  LPS35HW_LowNoise or LPS35HW_LowPower mode
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Set_PowerMode(void *handle, LPS35HW_PowerMode_et mode)
{
  uint8_t tmp;
  
  LPS35HW_assert_param(IS_LPS35HW_PowerMode(mode));
  
  if(LPS35HW_ReadReg(handle, LPS35HW_RES_CONF_REG, 1, &tmp))
    return LPS35HW_ERROR;
  
  tmp &= ~LPS35HW_LCEN_MASK;
  tmp |= (uint8_t)mode;
  
  if(LPS35HW_WriteReg(handle, LPS35HW_RES_CONF_REG, 1, &tmp))
    return LPS35HW_ERROR;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief  Get LPS35HW Power Mode
* @param *handle Device handle.
* @param   Buffer to empty with Mode: Low Noise or Low Current
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Get_PowerMode(void *handle, LPS35HW_PowerMode_et* mode)
{
  uint8_t tmp;
  
  if(LPS35HW_ReadReg(handle, LPS35HW_RES_CONF_REG, 1, &tmp))
    return LPS35HW_ERROR;
  
  *mode = (LPS35HW_PowerMode_et)(tmp & LPS35HW_LCEN_MASK);
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief  Set LPS35HW Output Data Rate
* @param *handle Device handle.
* @param  Output Data Rate
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Set_Odr(void *handle, LPS35HW_Odr_et odr)
{
  uint8_t tmp;
  
  LPS35HW_assert_param(IS_LPS35HW_ODR(odr));
  
  if(LPS35HW_ReadReg(handle, LPS35HW_CTRL_REG1, 1, &tmp))
    return LPS35HW_ERROR;
  
  tmp &= ~LPS35HW_ODR_MASK;
  tmp |= (uint8_t)odr;
  
  if(LPS35HW_WriteReg(handle, LPS35HW_CTRL_REG1, 1, &tmp))
    return LPS35HW_ERROR;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief  Get LPS35HW Output Data Rate
* @param *handle Device handle.
* @param  Buffer to empty with Output Data Rate
* @retval  Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Get_Odr(void *handle, LPS35HW_Odr_et* odr)
{
  uint8_t tmp;
  
  if(LPS35HW_ReadReg(handle, LPS35HW_CTRL_REG1, 1, &tmp))
    return LPS35HW_ERROR;
  
  *odr = (LPS35HW_Odr_et)(tmp & LPS35HW_ODR_MASK);
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief  Enable/Disale low-pass filter on LPS35HW pressure data
* @param *handle Device handle.
* @param  state: enable or disable
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Set_LowPassFilter(void *handle, LPS35HW_State_et state)
{
  uint8_t tmp;
  
  LPS35HW_assert_param(IS_LPS35HW_State(state));
  
  if(LPS35HW_ReadReg(handle, LPS35HW_CTRL_REG1, 1, &tmp))
    return LPS35HW_ERROR;
  
  tmp &= ~LPS35HW_LPFP_MASK;
  tmp |= ((uint8_t)state)<<LPS35HW_LPFP_BIT;
  
  if(LPS35HW_WriteReg(handle, LPS35HW_CTRL_REG1, 1, &tmp))
    return LPS35HW_ERROR;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief  Set low-pass filter cutoff configuration on LPS35HW pressure data
* @param *handle Device handle.
* @param  Filter Cutoff ODR/9 or ODR/20
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Set_LowPassFilterCutoff(void *handle, LPS35HW_LPF_Cutoff_et cutoff)
{
  uint8_t tmp;
  
  LPS35HW_assert_param(IS_LPS35HW_LPF_Cutoff(cutoff));
  
  if(LPS35HW_ReadReg(handle, LPS35HW_CTRL_REG1, 1, &tmp))
    return LPS35HW_ERROR;
  
  tmp &= ~LPS35HW_LPFP_CUTOFF_MASK;
  tmp |= (uint8_t)cutoff;
  
  if(LPS35HW_WriteReg(handle, LPS35HW_CTRL_REG1, 1, &tmp))
    return LPS35HW_ERROR;
  
  return LPS35HW_OK;
  
}

/*******************************************************************************
* @brief  Set Block Data Mode
* @detail It is recommended to set BDU bit to ‘1’.
* @detail This feature avoids reading LSB and MSB related to different samples.
* @param *handle Device handle.
* @param  LPS35HW_BDU_CONTINUOUS_UPDATE, LPS35HW_BDU_NO_UPDATE
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Set_Bdu(void *handle, LPS35HW_Bdu_et bdu)
{
  uint8_t tmp;
  
  LPS35HW_assert_param(IS_LPS35HW_BDUMode(bdu));
  
  if(LPS35HW_ReadReg(handle, LPS35HW_CTRL_REG1, 1, &tmp))
    return LPS35HW_ERROR;
  
  tmp &= ~LPS35HW_BDU_MASK;
  tmp |= ((uint8_t)bdu);
  
  if(LPS35HW_WriteReg(handle, LPS35HW_CTRL_REG1, 1, &tmp))
  return LPS35HW_OK;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief  Get Block Data Mode
* @param *handle Device handle.
* @param Buffer to empty whit the bdu mode read from sensor
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Get_Bdu(void *handle, LPS35HW_Bdu_et* bdu)
{
  uint8_t tmp;
  
  if(LPS35HW_ReadReg(handle, LPS35HW_CTRL_REG1, 1, &tmp))
    return LPS35HW_ERROR;
  
  *bdu = (LPS35HW_Bdu_et)(tmp & LPS35HW_BDU_MASK);
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief  Set SPI mode: 3 Wire Interface or 4 Wire Interface
* @param *handle Device handle.
* @param LPS35HW_SPI_3_WIRE, LPS35HW_SPI_4_WIRE
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Set_SpiInterface(void *handle, LPS35HW_SPIMode_et spimode)
{
  uint8_t tmp;
  
  LPS35HW_assert_param(IS_LPS35HW_SPIMode(spimode));
  
  if(LPS35HW_ReadReg(handle, LPS35HW_CTRL_REG1, 1, &tmp))
    return LPS35HW_ERROR;
  
  tmp &= ~LPS35HW_SIM_MASK;
  tmp |= (uint8_t)spimode;
  
  if(LPS35HW_WriteReg(handle, LPS35HW_CTRL_REG1, 1, &tmp))
    return LPS35HW_ERROR;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief  Clock Tree Configuration
* @param *handle Device handle.
* @param  LPS35HW_CTE_NotBalanced, LPS35HW_CTE_ABalanced
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Set_ClockTreeConfifuration(void *handle, LPS35HW_CTE_et mode)
{
  uint8_t tmp;
  
  LPS35HW_assert_param(IS_LPS35HW_CTE(mode));
  
  if(LPS35HW_ReadReg(handle, LPS35HW_CLOCK_TREE_CONFIGURATION, 1, &tmp))
    return LPS35HW_ERROR;
  
  tmp &= ~LPS35HW_CTE_MASK;
  tmp |= (uint8_t)mode;
  
  if(LPS35HW_WriteReg(handle, LPS35HW_CLOCK_TREE_CONFIGURATION, 1, &tmp))
    return LPS35HW_ERROR;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief  Get SPI mode: 3 Wire Interface or 4 Wire Interface
* @param *handle Device handle.
* @param Buffet to empty with spi mode read from Sensor
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Get_SpiInterface(void *handle, LPS35HW_SPIMode_et* spimode)
{
  uint8_t tmp;
  
  if(LPS35HW_ReadReg(handle, LPS35HW_CTRL_REG1, 1, &tmp))
    return LPS35HW_ERROR;
  
  *spimode = (LPS35HW_SPIMode_et)(tmp & LPS35HW_SIM_MASK);
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief   Software Reset. Self-clearing upon completion
* @param *handle Device handle.
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_SwReset(void *handle)
{
  uint8_t tmp;
  
  if(LPS35HW_ReadReg(handle, LPS35HW_CTRL_REG2, 1, &tmp))
    return LPS35HW_ERROR;
  
  tmp |= (0x01<<LPS35HW_SW_RESET_BIT);
  
  if(LPS35HW_WriteReg(handle, LPS35HW_CTRL_REG2, 1, &tmp))
    return LPS35HW_ERROR;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief  Reboot Memory Content
* @param *handle Device handle.
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_MemoryBoot(void *handle)
{
  uint8_t tmp;
  
  if(LPS35HW_ReadReg(handle, LPS35HW_CTRL_REG2, 1, &tmp))
    return LPS35HW_ERROR;
  
  tmp |= (0x01<<LPS35HW_BOOT_BIT);
  
  if(LPS35HW_WriteReg(handle, LPS35HW_CTRL_REG2, 1, &tmp))
    return LPS35HW_ERROR;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief   Software Reset ann Reboot Memory Content.
* @detail  The device is reset to the power on configuration if the SWRESET bit is set to ‘1’
  + and BOOT is set to ‘1’; Self-clearing upon completion.
* @param *handle Device handle.
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_SwResetAndMemoryBoot(void *handle)
{
  uint8_t tmp;
  
  if(LPS35HW_ReadReg(handle, LPS35HW_CTRL_REG2, 1, &tmp))
    return LPS35HW_ERROR;
  
  tmp |= ((0x01<<LPS35HW_SW_RESET_BIT) | (0x01<<LPS35HW_BOOT_BIT));
  
  if(LPS35HW_WriteReg(handle, LPS35HW_CTRL_REG2, 1, &tmp))
    return LPS35HW_ERROR;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief   Enable/Disable FIFO Mode
* @param *handle Device handle.
* @param LPS35HW_ENABLE/LPS35HW_DISABLE
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Set_FifoModeUse(void *handle, LPS35HW_State_et status)
{
  uint8_t tmp;
  
  LPS35HW_assert_param(IS_LPS35HW_State(status));
  
  if(LPS35HW_ReadReg(handle, LPS35HW_CTRL_REG2, 1, &tmp))
    return LPS35HW_ERROR;
  
  tmp &= ~LPS35HW_FIFO_EN_MASK;
  tmp |= ((uint8_t)status)<<LPS35HW_FIFO_EN_BIT;
  
  if(LPS35HW_WriteReg(handle, LPS35HW_CTRL_REG2, 1, &tmp))
    return LPS35HW_ERROR;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief   Enable/Disable FIFO Watermark Level Use
* @param *handle Device handle.
* @param   LPS35HW_ENABLE/LPS35HW_DISABLE
* @retval  Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Set_FifoWatermarkLevelUse(void *handle, LPS35HW_State_et status)
{
  uint8_t tmp;
  
  LPS35HW_assert_param(IS_LPS35HW_State(status));
  
  if(LPS35HW_ReadReg(handle, LPS35HW_CTRL_REG2, 1, &tmp))
    return LPS35HW_ERROR;
  
  tmp &= ~LPS35HW_WTM_EN_MASK;
  tmp |= ((uint8_t)status)<<LPS35HW_WTM_EN_BIT;
  
  if(LPS35HW_WriteReg(handle, LPS35HW_CTRL_REG2, 1, &tmp))
    return LPS35HW_ERROR;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief  Enable or Disable the Automatic increment register address during a 
*         multiple byte access with a serial interface (I2C or SPI)
* @param *handle Device handle.
* @param  LPS35HW_ENABLE/LPS35HW_DISABLE. Default is LPS35HW_ENABLE
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Set_AutomaticIncrementRegAddress(void *handle, LPS35HW_State_et status)
{
  uint8_t tmp;
  
  LPS35HW_assert_param(IS_LPS35HW_State(status));
  
  if(LPS35HW_ReadReg(handle, LPS35HW_CTRL_REG2, 1, &tmp))
    return LPS35HW_ERROR;
  
  tmp &= ~LPS35HW_ADD_INC_MASK;
  tmp |= (((uint8_t)status)<<LPS35HW_ADD_INC_BIT);
  
  if(LPS35HW_WriteReg(handle, LPS35HW_CTRL_REG2, 1, &tmp))
    return LPS35HW_ERROR;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief  Enable/Disable I2C Interface
* @param *handle Device handle.
* @param State: LPS35HW_ENABLE (reset bit)/ LPS35HW_DISABLE (set bit)
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Set_I2C(void *handle, LPS35HW_State_et statei2c)
{
  uint8_t tmp;
  
  LPS35HW_assert_param(IS_LPS35HW_State(statei2c));
  
  if(LPS35HW_ReadReg(handle, LPS35HW_CTRL_REG2, 1, &tmp))
    return LPS35HW_ERROR;
  
  /*Reset Bit->I2C Enabled*/
  tmp &= ~LPS35HW_I2C_MASK;
  tmp|=((uint8_t)~statei2c)<<LPS35HW_I2C_BIT;
  
  if(LPS35HW_WriteReg(handle, LPS35HW_CTRL_REG2, 1, &tmp))
    return LPS35HW_ERROR;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief   Set the one-shot bit in order to start acquisition when the ONE SHOT mode
*          has been selected by the ODR configuration.
* @detail  Once the measurement is done, ONE_SHOT bit will self-clear.
* @param *handle Device handle.
* @retval  Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_StartOneShotMeasurement(void *handle)
{
  uint8_t tmp;
  
  if(LPS35HW_ReadReg(handle, LPS35HW_CTRL_REG2, 1, &tmp))
    return LPS35HW_ERROR;
  
  /* Set the one shot bit */
  /* Once the measurement is done, one shot bit will self-clear*/
  tmp |= LPS35HW_ONE_SHOT_MASK;
  
  if(LPS35HW_WriteReg(handle, LPS35HW_CTRL_REG2, 1, &tmp))
    return LPS35HW_ERROR;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief  Set Interrupt Active on High or Low Level
* @param *handle Device handle.
* @param  LPS35HW_ActiveHigh/LPS35HW_ActiveLow
* @retval  Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Set_InterruptActiveLevel(void *handle, LPS35HW_InterruptActiveLevel_et mode)
{
  uint8_t tmp;
  
  LPS35HW_assert_param(IS_LPS35HW_InterruptActiveLevel(mode));
  
  if(LPS35HW_ReadReg(handle, LPS35HW_CTRL_REG3, 1, &tmp))
    return LPS35HW_ERROR;
  
  tmp &= ~LPS35HW_INT_H_L_MASK;
  tmp |= ((uint8_t)mode);
  
  if(LPS35HW_WriteReg(handle, LPS35HW_CTRL_REG3, 1, &tmp))
    return LPS35HW_ERROR;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief   Push-pull/open drain selection on interrupt pads. Default tmp: 0
* @param *handle Device handle.
* @param   LPS35HW_PushPull/LPS35HW_OpenDrain
* @retval  Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Set_InterruptOutputType(void *handle, LPS35HW_OutputType_et output)
{
  uint8_t tmp;
  
  LPS35HW_assert_param(IS_LPS35HW_OutputType(output));
  
  if(LPS35HW_ReadReg(handle, LPS35HW_CTRL_REG3, 1, &tmp))
    return LPS35HW_ERROR;
  
  tmp &= ~LPS35HW_PP_OD_MASK;
  tmp |= (uint8_t)output;
  
  if(LPS35HW_WriteReg(handle, LPS35HW_CTRL_REG3, 1, &tmp))
    return LPS35HW_ERROR;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief  Set Data signal on INT pad control bits.
* @param *handle Device handle.
* @param  LPS35HW_DATA,LPS35HW_P_HIGH_LPS35HW_P_LOW,LPS35HW_P_LOW_HIGH
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Set_InterruptControlConfig(void *handle, LPS35HW_OutputSignalConfig_et config)
{
  uint8_t tmp;
  
  LPS35HW_assert_param(IS_LPS35HW_OutputSignal(config));
  
  if(LPS35HW_ReadReg(handle, LPS35HW_CTRL_REG3, 1, &tmp))
    return LPS35HW_ERROR;
  
  tmp &= ~(LPS35HW_INT_S12_MASK);
  tmp |= (uint8_t)config;
  
  if(LPS35HW_WriteReg(handle, LPS35HW_CTRL_REG3, 1, &tmp))
    return LPS35HW_ERROR;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief   Enable/Disable Data-ready signal on INT_DRDY pin.
* @param *handle Device handle.
* @param  LPS35HW_ENABLE/LPS35HW_DISABLE
* @retval  Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Set_DRDYInterrupt(void *handle, LPS35HW_State_et status)
{
  uint8_t tmp;
  
  LPS35HW_assert_param(IS_LPS35HW_State(status));
  
  if(LPS35HW_ReadReg(handle, LPS35HW_CTRL_REG3, 1, &tmp))
    return LPS35HW_ERROR;
  
  tmp &= ~LPS35HW_DRDY_MASK;
  tmp |= ((uint8_t)status)<<LPS35HW_DRDY_BIT;
  
  if(LPS35HW_WriteReg(handle, LPS35HW_CTRL_REG3, 1, &tmp))
    return LPS35HW_ERROR;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief   Enable/Disable FIFO overrun interrupt on INT_DRDY pin.
* @param *handle Device handle.
* @param  LPS35HW_ENABLE/LPS35HW_DISABLE
* @retval  Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Set_FIFO_OVR_Interrupt(void *handle, LPS35HW_State_et status)
{
  uint8_t tmp;
  
  LPS35HW_assert_param(IS_LPS35HW_State(status));
  
  if(LPS35HW_ReadReg(handle, LPS35HW_CTRL_REG3, 1, &tmp))
    return LPS35HW_ERROR;
  
  tmp &= ~LPS35HW_FIFO_OVR_MASK;
  tmp |= ((uint8_t)status)<<LPS35HW_FIFO_OVR_BIT;
  
  if(LPS35HW_WriteReg(handle, LPS35HW_CTRL_REG3, 1, &tmp))
    return LPS35HW_ERROR;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief   Enable/Disable FIFO threshold (Watermark) interrupt on INT_DRDY pin.
* @param *handle Device handle.
* @param  LPS35HW_ENABLE/LPS35HW_DISABLE
* @retval  Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Set_FIFO_FTH_Interrupt(void *handle, LPS35HW_State_et status)
{
  uint8_t tmp;
  
  LPS35HW_assert_param(IS_LPS35HW_State(status));
  
  if(LPS35HW_ReadReg(handle, LPS35HW_CTRL_REG3, 1, &tmp))
    return LPS35HW_ERROR;
  
  tmp &= ~LPS35HW_FIFO_FTH_MASK;
  tmp |= ((uint8_t)status)<<LPS35HW_FIFO_FTH_BIT;
  
  if(LPS35HW_WriteReg(handle, LPS35HW_CTRL_REG3, 1, &tmp))
    return LPS35HW_ERROR;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief   Enable/Disable FIFO FULL interrupt on INT_DRDY pin.
* @param *handle Device handle.
* @param  LPS35HW_ENABLE/LPS35HW_DISABLE
* @retval  Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Set_FIFO_FULL_Interrupt(void *handle, LPS35HW_State_et status)
{
  uint8_t tmp;
  
  LPS35HW_assert_param(IS_LPS35HW_State(status));
  
  if(LPS35HW_ReadReg(handle, LPS35HW_CTRL_REG3, 1, &tmp))
    return LPS35HW_ERROR;
  
  tmp &= ~LPS35HW_FIFO_FULL_MASK;
  tmp |= ((uint8_t)status)<<LPS35HW_FIFO_FULL_BIT;
  
  if(LPS35HW_WriteReg(handle, LPS35HW_CTRL_REG3, 1, &tmp))
    return LPS35HW_ERROR;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief   Enable AutoRifP function
* @detail When this function is enabled, an internal register is set with the current pressure values
*         and the content is subtracted from the pressure output value and result is used for the interrupt generation.
*               the AutoRifP is slf creared.
* @param *handle Device handle.
* @retval  Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Set_AutoRifP(void *handle)
{
  uint8_t tmp;
  
  if(LPS35HW_ReadReg(handle, LPS35HW_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS35HW_ERROR;
  
  tmp |= ((uint8_t)LPS35HW_AUTORIFP_MASK);
  
  if(LPS35HW_WriteReg(handle, LPS35HW_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS35HW_ERROR;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief   Disable AutoRifP function
* @detail  the RESET_ARP bit is used to disable the AUTORIFP function. This bis i is selfdleared
* @param *handle Device handle.
* @retval  Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_ResetAutoRifP(void *handle)
{
  uint8_t tmp;
  
  if(LPS35HW_ReadReg(handle, LPS35HW_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS35HW_ERROR;
  
  tmp |= ((uint8_t)LPS35HW_RESET_ARP_MASK);
  
  if(LPS35HW_WriteReg(handle, LPS35HW_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS35HW_ERROR;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief  Set AutoZero Function bit
* @detail When set to ‘1’, the actual pressure output is copied in the REF_P reg (@0x15..0x17)
* @param *handle Device handle.
* @retval  Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Set_AutoZeroFunction(void *handle)
{
  uint8_t tmp;
  
  if(LPS35HW_ReadReg(handle, LPS35HW_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS35HW_ERROR;
  
  tmp |= LPS35HW_AUTOZERO_MASK;
  
  if(LPS35HW_WriteReg(handle, LPS35HW_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS35HW_ERROR;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief  Set ResetAutoZero Function bit
* @details REF_P reg (@0x015..17) set pressure reference to default value RPDS reg (0x18/19).
* @param *handle Device handle.
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_ResetAutoZeroFunction(void *handle)
{
  uint8_t tmp;
  
  if(LPS35HW_ReadReg(handle, LPS35HW_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS35HW_ERROR;
  
  /* Set the RESET_AZ bit*/
  /* RESET_AZ is self cleared*/
  tmp |= LPS35HW_RESET_AZ_MASK;
  
  if(LPS35HW_WriteReg(handle, LPS35HW_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS35HW_ERROR;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief  Enable/ Disable the computing of differential pressure output (Interrupt Generation)
* @param *handle Device handle.
* @param  LPS35HW_ENABLE,LPS35HW_DISABLE
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Set_InterruptDifferentialGeneration(void *handle, LPS35HW_State_et diff_en)
{
  uint8_t tmp;
  
  LPS35HW_assert_param(IS_LPS35HW_State(diff_en));
  
  if(LPS35HW_ReadReg(handle, LPS35HW_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS35HW_ERROR;
  
  tmp &= ~LPS35HW_DIFF_EN_MASK;
  tmp |= ((uint8_t)diff_en)<<LPS35HW_DIFF_EN_BIT;
  
  if(LPS35HW_WriteReg(handle, LPS35HW_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS35HW_ERROR;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief  Get the DIFF_EN bit value
* @param *handle Device handle.
* @param  buffer to empty with the read value of DIFF_EN bit
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Get_InterruptDifferentialGeneration(void *handle, LPS35HW_State_et* diff_en)
{
  uint8_t tmp;
  
  if(LPS35HW_ReadReg(handle, LPS35HW_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS35HW_ERROR;
  
*diff_en= (LPS35HW_State_et)((tmp & LPS35HW_DIFF_EN_MASK)>>LPS35HW_DIFF_EN_BIT);
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief  Latch Interrupt request to the INT_SOURCE register.
* @param *handle Device handle.
* @param  LPS35HW_ENABLE/LPS35HW_DISABLE
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_LatchInterruptRequest(void *handle, LPS35HW_State_et status)
{
  uint8_t tmp;
  
  LPS35HW_assert_param(IS_LPS35HW_State(status));
  
  if(LPS35HW_ReadReg(handle, LPS35HW_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS35HW_ERROR;
  
  tmp &= ~LPS35HW_LIR_MASK;
  tmp |= (((uint8_t)status)<<LPS35HW_LIR_BIT);
  
  if(LPS35HW_WriteReg(handle, LPS35HW_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS35HW_ERROR;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief  Enable\Disable Interrupt Generation on differential pressure Low event
* @param *handle Device handle.
* @param  LPS35HW_ENABLE/LPS35HW_DISABLE
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Set_PLE(void *handle, LPS35HW_State_et status)
{
  uint8_t tmp;
  
  LPS35HW_assert_param(IS_LPS35HW_State(status));
  
  if(LPS35HW_ReadReg(handle, LPS35HW_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS35HW_ERROR;
  
  tmp &= ~LPS35HW_PLE_MASK;
  tmp |= (((uint8_t)status)<<LPS35HW_PLE_BIT);
  
  if(LPS35HW_WriteReg(handle, LPS35HW_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS35HW_ERROR;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief  Enable\Disable Interrupt Generation on differential pressure High event
* @param *handle Device handle.
* @param  LPS35HW_ENABLE/LPS35HW_DISABLE
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Set_PHE(void *handle, LPS35HW_State_et status)
{
  uint8_t tmp;
  
  LPS35HW_assert_param(IS_LPS35HW_State(status));
  
  if(LPS35HW_ReadReg(handle, LPS35HW_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS35HW_ERROR;
  
  tmp &= ~LPS35HW_PHE_MASK;
  tmp |= (((uint8_t)status)<<LPS35HW_PHE_BIT);
  
  if(LPS35HW_WriteReg(handle, LPS35HW_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS35HW_ERROR;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief   Get the Interrupt Generation on differential pressure status event and the Boot Status.
* @detail  The INT_SOURCE register is cleared by reading it.
* @param *handle Device handle.
* @param   Status Event Flag: BOOT, PH,PL,IA
* @retval  Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Get_InterruptDifferentialEventStatus(void *handle, LPS35HW_InterruptDiffStatus_st* interruptsource)
{
  uint8_t tmp;
  
  if(LPS35HW_ReadReg(handle, LPS35HW_INTERRUPT_SOURCE_REG, 1, &tmp))
    return LPS35HW_ERROR;
  
  interruptsource->PH = (uint8_t)(tmp & LPS35HW_PH_MASK);
  interruptsource->PL = (uint8_t)((tmp & LPS35HW_PL_MASK)>>LPS35HW_PL_BIT);
  interruptsource->IA = (uint8_t)((tmp & LPS35HW_IA_MASK)>>LPS35HW_IA_BIT);
  interruptsource->BOOT= (uint8_t)((tmp & LPS35HW_BOOT_STATUS_MASK)>>LPS35HW_BOOT_STATUS_BIT);
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief  Get the status of Pressure and Temperature data
* @param *handle Device handle.
* @param  Data Status Flag:  TempDataAvailable, TempDataOverrun, PressDataAvailable, PressDataOverrun
* @retval  Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Get_DataStatus(void *handle, LPS35HW_DataStatus_st* datastatus)
{
  uint8_t tmp;
  
  if(LPS35HW_ReadReg(handle, LPS35HW_STATUS_REG, 1, &tmp))
    return LPS35HW_ERROR;
  
  datastatus->PressDataAvailable = (uint8_t)(tmp & LPS35HW_PDA_MASK);
  datastatus->TempDataAvailable = (uint8_t)((tmp & LPS35HW_TDA_MASK)>>LPS35HW_PDA_BIT);
  datastatus->TempDataOverrun = (uint8_t)((tmp & LPS35HW_TOR_MASK)>>LPS35HW_TOR_BIT);
  datastatus->PressDataOverrun = (uint8_t)((tmp & LPS35HW_POR_MASK)>>LPS35HW_POR_BIT);
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief  Get the LPS35HW raw presure value
* @detail   The data are expressed as PRESS_OUT_H/_L/_XL in 2’s complement.
  Pout(hPA)=PRESS_OUT / 4096
* @param *handle Device handle.
* @param  The buffer to empty with the pressure raw value
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Get_RawPressure(void *handle, int32_t *raw_press)
{
  uint8_t buffer[3];
  uint32_t tmp = 0;
  uint8_t i;
  
  if(LPS35HW_ReadReg(handle, LPS35HW_PRESS_OUT_XL_REG, 3, buffer))
    return LPS35HW_ERROR;
  
  /* Build the raw data */
  for(i=0; i<3; i++)
  tmp |= (((uint32_t)buffer[i]) << (8*i));
  
  /* convert the 2's complement 24 bit to 2's complement 32 bit */
  if(tmp & 0x00800000)
  tmp |= 0xFF000000;
  
*raw_press = ((int32_t)tmp);
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief    Get the LPS35HW Pressure value in hPA.
* @detail   The data are expressed as PRESS_OUT_H/_L/_XL in 2’s complement.
  Pout(hPA)=PRESS_OUT / 4096
* @param *handle Device handle.
* @param      The buffer to empty with the pressure value that must be divided by 100 to get the value in hPA
* @retval   Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Get_Pressure(void *handle, int32_t* Pout)
{
  int32_t raw_press;
  
  if(LPS35HW_Get_RawPressure(handle, &raw_press))
    return LPS35HW_ERROR;
  
*Pout = (raw_press*100)/4096;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief    Get the Raw Temperature value.
* @detail   Temperature data are expressed as TEMP_OUT_H&TEMP_OUT_L as 2’s complement number.
*            Tout(degC)=TEMP_OUT/100
* @param *handle Device handle.
* @param     Buffer to empty with the temperature raw tmp.
* @retval   Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Get_RawTemperature(void *handle, int16_t* raw_data)
{
  uint8_t buffer[2];
  uint16_t tmp;
  
  if(LPS35HW_ReadReg(handle, LPS35HW_TEMP_OUT_L_REG, 2, buffer))
    return LPS35HW_ERROR;
  
  /* Build the raw tmp */
  tmp = (((uint16_t)buffer[1]) << 8) + (uint16_t)buffer[0];
  
*raw_data = ((int16_t)tmp);
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief    Get the Temperature value in °C.
* @detail   Temperature data are expressed as TEMP_OUT_H&TEMP_OUT_L as 2’s complement number.
*           Tout(degC)=TEMP_OUT/100
* @param *handle Device handle.
* @param Buffer to empty with the temperature value that must be divided by 10 to get the value in °C
* @retval  Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Get_Temperature(void *handle, int16_t* Tout)
{
  int16_t raw_data;
  
  if(LPS35HW_Get_RawTemperature(handle, &raw_data))
    return LPS35HW_ERROR;
  
*Tout = (raw_data*10)/100;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief    Get the threshold value used for pressure interrupt generation (hPA).
* @detail   THS_P=THS_P_H&THS_P_L and is expressed as unsigned number. P_ths(hPA)=(THS_P)/16.
* @param *handle Device handle.
* @param    Buffer to empty with the pressure threshold in hPA
* @retval   Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Get_PressureThreshold(void *handle, int16_t* P_ths)
{
  uint8_t tempReg[2];
  
  if(LPS35HW_ReadReg(handle, LPS35HW_THS_P_LOW_REG, 2, tempReg))
    return LPS35HW_ERROR;
  
*P_ths= (((((uint16_t)tempReg[1])<<8) + tempReg[0])/16);
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief    Set the threshold value  used for pressure interrupt generation (hPA).
* @detail   THS_P=THS_P_H&THS_P_L and is expressed as unsigned number. P_ths(hPA)=(THS_P)/16.
* @param *handle Device handle.
* @param 	  Pressure threshold in hPA
* @retval   Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Set_PressureThreshold(void *handle, int16_t P_ths)
{
  uint8_t buffer[2];
  
  buffer[0] = (uint8_t)(16 * P_ths);
  buffer[1] = (uint8_t)(((uint16_t)(16 * P_ths))>>8);
  
  if(LPS35HW_WriteReg(handle, LPS35HW_THS_P_LOW_REG, 2, buffer))
    return LPS35HW_ERROR;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief  Set Fifo Mode.
* @param *handle Device handle.
* @param  Fifo Mode struct
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Set_FifoMode(void *handle, LPS35HW_FifoMode_et fifomode)
{
  uint8_t tmp;
  
  LPS35HW_assert_param(IS_LPS35HW_FifoMode(fifomode));
  
  if(LPS35HW_ReadReg(handle, LPS35HW_CTRL_FIFO_REG, 1, &tmp))
    return LPS35HW_ERROR;
  
  tmp &= ~LPS35HW_FIFO_MODE_MASK;
  tmp |= (uint8_t)fifomode;
  
  if(LPS35HW_WriteReg(handle, LPS35HW_CTRL_FIFO_REG, 1, &tmp))
    return LPS35HW_ERROR;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief    Get Fifo Mode
* @param *handle Device handle.
* @param   buffer to empty with fifo mode tmp
* @retval   Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Get_FifoMode(void *handle, LPS35HW_FifoMode_et* fifomode)
{
  uint8_t tmp;
  
  if(LPS35HW_ReadReg(handle, LPS35HW_CTRL_FIFO_REG, 1, &tmp))
    return LPS35HW_ERROR;
  
  tmp &= LPS35HW_FIFO_MODE_MASK;
*fifomode = (LPS35HW_FifoMode_et)tmp;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief    Set Fifo Watermark Level.
* @param *handle Device handle.
* @param    Watermark level value [0 31]
* @retval   Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Set_FifoWatermarkLevel(void *handle, uint8_t wtmlevel)
{
  uint8_t tmp;
  
  LPS35HW_assert_param(IS_LPS35HW_WtmLevel(wtmlevel));
  
  if(LPS35HW_ReadReg(handle, LPS35HW_CTRL_FIFO_REG, 1, &tmp))
    return LPS35HW_ERROR;
  
  tmp &= ~LPS35HW_WTM_POINT_MASK;
  tmp |= wtmlevel;
  
  if(LPS35HW_WriteReg(handle, LPS35HW_CTRL_FIFO_REG, 1, &tmp))
    return LPS35HW_ERROR;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief   Get FIFO Watermark Level
* @param *handle Device handle.
* @param   buffer to empty with watermak level[0,31] value read from sensor
* @retval  Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Get_FifoWatermarkLevel(void *handle, uint8_t *wtmlevel)
{
  if(LPS35HW_ReadReg(handle, LPS35HW_CTRL_FIFO_REG, 1, wtmlevel))
    return LPS35HW_ERROR;
  
  *wtmlevel &= LPS35HW_WTM_POINT_MASK;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief    Get the Fifo Status
* @param *handle Device handle.
* @param    Status Flag: FIFO_FTH,FIFO_EMPTY,FIFO_FULL,FIFO_OVR and level of the FIFO->FIFO_LEVEL
* @retval   Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Get_FifoStatus(void *handle, LPS35HW_FifoStatus_st* status)
{
  uint8_t tmp;
  
  if(LPS35HW_ReadReg(handle, LPS35HW_STATUS_FIFO_REG, 1, &tmp))
    return LPS35HW_ERROR;
  
  status->FIFO_FTH = (uint8_t)((tmp & LPS35HW_FTH_FIFO_MASK)>>LPS35HW_FTH_FIFO_BIT);
  status->FIFO_OVR=(uint8_t)((tmp & LPS35HW_OVR_FIFO_MASK)>>LPS35HW_OVR_FIFO_BIT);
  status->FIFO_LEVEL = (uint8_t)(tmp & LPS35HW_LEVEL_FIFO_MASK);
  
  if(status->FIFO_LEVEL ==LPS35HW_FIFO_EMPTY)
  status->FIFO_EMPTY=0x01;
  else
  status->FIFO_EMPTY=0x00;
  
  if (status->FIFO_LEVEL ==LPS35HW_FIFO_FULL)
  status->FIFO_FULL=0x01;
  else
  status->FIFO_FULL=0x00;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief  Get the reference pressure after soldering for computing differential pressure (hPA)
* @param *handle Device handle.
* @param buffer to empty with the he pressure value (hPA)
* @retval  Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Get_PressureOffsetValue(void *handle, int16_t *pressoffset)
{
  uint8_t buffer[2];
  int16_t raw_press;
  
  if(LPS35HW_ReadReg(handle, LPS35HW_RPDS_L_REG, 2, buffer))
    return LPS35HW_ERROR;
  
  raw_press = (int16_t)((((uint16_t)buffer[1]) << 8) + (uint16_t)buffer[0]);
  
  *pressoffset = (raw_press*100)/4096;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief  Get the Reference Pressure value
* @detail  It is a 24-bit data added to the sensor output measurement to detect a measured pressure beyond programmed limits.
* @param *handle Device handle.
* @param  Buffer to empty with reference pressure value
* @retval  Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Get_ReferencePressure(void *handle, int32_t* RefP)
{
  uint8_t buffer[3];
  uint32_t tempVal=0;
  int32_t raw_press;
  uint8_t i;
  
  if(LPS35HW_ReadReg(handle, LPS35HW_REF_P_XL_REG, 3, buffer))
    return LPS35HW_ERROR;
  
  /* Build the raw data */
  for(i=0; i<3; i++)
  tempVal |= (((uint32_t)buffer[i]) << (8*i));
  
  /* convert the 2's complement 24 bit to 2's complement 32 bit */
  if(tempVal & 0x00800000)
  tempVal |= 0xFF000000;
  
  raw_press =((int32_t)tempVal);
  *RefP = (raw_press*100)/4096;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief  Check if the single measurement has completed.
* @param *handle Device handle.
* @param  the returned value is set to 1, when the measurement is completed
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_IsMeasurementCompleted(void *handle, uint8_t* Is_Measurement_Completed)
{
  uint8_t tmp;
  LPS35HW_DataStatus_st datastatus;
  
  if(LPS35HW_ReadReg(handle, LPS35HW_STATUS_REG, 1, &tmp))
    return LPS35HW_ERROR;
  
  datastatus.TempDataAvailable=(uint8_t)((tmp&LPS35HW_TDA_MASK)>>LPS35HW_TDA_BIT);
  datastatus.PressDataAvailable= (uint8_t)(tmp&LPS35HW_PDA_MASK);
  
  *Is_Measurement_Completed=(uint8_t)((datastatus.PressDataAvailable) & (datastatus.TempDataAvailable));
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief  Get the values of the last single measurement.
* @param *handle Device handle.
* @param  Pressure and temperature tmp
* @retval Error Code [LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Get_Measurement(void *handle, LPS35HW_MeasureTypeDef_st *Measurement_Value)
{
  int16_t Tout;
  int32_t Pout;
  
  if(LPS35HW_Get_Temperature(handle, &Tout))
    return LPS35HW_ERROR;
  
  Measurement_Value->Tout=Tout;
  
  if(LPS35HW_Get_Pressure(handle, &Pout))
    return LPS35HW_ERROR;
  
  Measurement_Value->Pout=Pout;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief  Initialization function for LPS35HW.
*         This function make a memory boot.
*         Init the sensor with a standard basic confifuration.
*         Low Power, ODR 25 Hz, Low Pass Filter disabled; BDU enabled; I2C enabled;
*        NO FIFO; NO Interrupt Enabled.
* @param *handle Device handle.
* @retval Error code[LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Init(void *handle)
{
  LPS35HW_ConfigTypeDef_st pLPS35HWInit;
  
  /* Make LPS35HW Reset and Reboot */
  if(LPS35HW_SwResetAndMemoryBoot(handle))
    return LPS35HW_ERROR;
  
  pLPS35HWInit.PowerMode=LPS35HW_LowPower;
  pLPS35HWInit.OutputDataRate=LPS35HW_ODR_25HZ;
  pLPS35HWInit.LowPassFilter=LPS35HW_DISABLE;
  pLPS35HWInit.LPF_Cutoff=LPS35HW_ODR_9;
  pLPS35HWInit.BDU=LPS35HW_BDU_NO_UPDATE;
  pLPS35HWInit.IfAddInc=LPS35HW_ENABLE; //default
  pLPS35HWInit.Sim= LPS35HW_SPI_4_WIRE;
  
  /* Set Generic Configuration*/
  if(LPS35HW_Set_GenericConfig(handle, &pLPS35HWInit))
    return LPS35HW_ERROR;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief  De initialization function for LPS35HW.
*         This function make a memory boot and clear the data output flags.
* @param *handle Device handle.
* @retval Error code[LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_DeInit(void *handle)
{
  LPS35HW_MeasureTypeDef_st Measurement_Value;
  
  /* Make LPS35HW Reset and Reboot */
  if(LPS35HW_SwResetAndMemoryBoot(handle))
    return LPS35HW_ERROR;
  
  /* Dump of data output */
  if(LPS35HW_Get_Measurement(handle, &Measurement_Value))
    return LPS35HW_ERROR;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief   Set Generic Configuration
* @param *handle Device handle.
* @param   Struct to empty with the chosen values
* @retval  Error code[LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Set_GenericConfig(void *handle, LPS35HW_ConfigTypeDef_st* pxLPS35HWInit)
{
  
  /* Enable Low Current Mode (low Power) or Low Noise Mode*/
  if(LPS35HW_Set_PowerMode(handle, pxLPS35HWInit->PowerMode))
    return LPS35HW_ERROR;
  
  /* Init the Output Data Rate*/
  if(LPS35HW_Set_Odr(handle, pxLPS35HWInit->OutputDataRate))
    return LPS35HW_ERROR;
  
  /* BDU bit is used to inhibit the output registers update between the reading of upper and
    lower register parts. In default mode (BDU = ‘0’), the lower and upper register parts are
    updated continuously. If it is not sure to read faster than output data rate, it is recommended
    to set BDU bit to ‘1’. In this way, after the reading of the lower (upper) register part, the
    content of that output registers is not updated until the upper (lower) part is read too.
  This feature avoids reading LSB and MSB related to different samples.*/
  
  if(LPS35HW_Set_Bdu(handle, pxLPS35HWInit->BDU))
    return LPS35HW_ERROR;
  
  /*Enable/Disale low-pass filter on LPS35HW pressure data*/
  if(LPS35HW_Set_LowPassFilter(handle, pxLPS35HWInit->LowPassFilter))
    return LPS35HW_ERROR;
  
  /* Set low-pass filter cutoff configuration*/
  if(LPS35HW_Set_LowPassFilterCutoff(handle, pxLPS35HWInit->LPF_Cutoff))
    return LPS35HW_ERROR;
  
  /* SIM bit selects the SPI serial interface mode.*/
  /* This feature has effect only if SPI interface is used*/
  
  if(LPS35HW_Set_SpiInterface(handle, pxLPS35HWInit->Sim))
    return LPS35HW_ERROR;
  
  /*Enable or Disable the Automatic increment register address during a multiple byte access with a serial interface (I2C or SPI)*/
  if(LPS35HW_Set_AutomaticIncrementRegAddress(handle, pxLPS35HWInit->IfAddInc))
    return LPS35HW_ERROR;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief  Get Generic configuration
* @param *handle Device handle.
* @param  Struct to empty with configuration values
* @retval Error code[LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Get_GenericConfig(void *handle, LPS35HW_ConfigTypeDef_st* pxLPS35HWInit)
{
  uint8_t tmp;
  
  /*Read LPS35HW_RES_CONF_REG*/
  if(LPS35HW_Get_PowerMode(handle, &pxLPS35HWInit->PowerMode))
    return LPS35HW_ERROR;
  
  /*Read LPS35HW_CTRL_REG1*/
  if(LPS35HW_ReadReg(handle, LPS35HW_CTRL_REG1, 1, &tmp))
    return LPS35HW_ERROR;
  
  pxLPS35HWInit->OutputDataRate= (LPS35HW_Odr_et)(tmp & LPS35HW_ODR_MASK);
  pxLPS35HWInit->BDU=(LPS35HW_Bdu_et)(tmp & LPS35HW_BDU_MASK);
  pxLPS35HWInit->Sim=(LPS35HW_SPIMode_et)(tmp& LPS35HW_SIM_MASK);
  pxLPS35HWInit->LowPassFilter=(LPS35HW_State_et)((tmp& LPS35HW_LPFP_MASK)>>LPS35HW_LPFP_BIT);
  pxLPS35HWInit->LPF_Cutoff=(LPS35HW_LPF_Cutoff_et)(tmp& LPS35HW_LPFP_CUTOFF_MASK);
  
  /*Read LPS35HW_CTRL_REG2*/
  if(LPS35HW_ReadReg(handle, LPS35HW_CTRL_REG2, 1, &tmp))
    return LPS35HW_ERROR;
  
  pxLPS35HWInit->IfAddInc=(LPS35HW_State_et)((tmp& LPS35HW_ADD_INC_MASK)>>LPS35HW_ADD_INC_BIT);
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief  Set Interrupt configuration
* @param *handle Device handle.
* @param  Struct holding the configuration values
* @retval  Error code[LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Set_InterruptConfig(void *handle, LPS35HW_InterruptTypeDef_st* pLPS35HWInt)
{
  /* Enable Low Current Mode (low Power) or Low Noise Mode*/
  if(LPS35HW_Set_InterruptActiveLevel(handle, pLPS35HWInt->INT_H_L))
    return LPS35HW_ERROR;
  
  /* Push-pull/open drain selection on interrupt pads.*/
  if(LPS35HW_Set_InterruptOutputType(handle, pLPS35HWInt->PP_OD))
    return LPS35HW_ERROR;
  
  /* Set Data signal on INT pad control bits.*/
  if(LPS35HW_Set_InterruptControlConfig(handle, pLPS35HWInt->OutputSignal_INT))
    return LPS35HW_ERROR;
  
  /* Enable/Disable Data-ready signal on INT_DRDY pin. */
  if(LPS35HW_Set_DRDYInterrupt(handle, pLPS35HWInt->DRDY))
    return LPS35HW_ERROR;
  
  /* Enable/Disable FIFO overrun interrupt on INT_DRDY pin. */
  if(LPS35HW_Set_FIFO_OVR_Interrupt(handle, pLPS35HWInt->FIFO_OVR))
    return LPS35HW_ERROR;
  
  /* Enable/Disable FIFO Treshold interrupt on INT_DRDY pin. */
  if(LPS35HW_Set_FIFO_FTH_Interrupt(handle, pLPS35HWInt->FIFO_FTH))
    return LPS35HW_ERROR;
  
  /* Enable/Disable FIFO FULL interrupt on INT_DRDY pin. */
  if(LPS35HW_Set_FIFO_FULL_Interrupt(handle, pLPS35HWInt->FIFO_FULL))
    return LPS35HW_ERROR;
  
  /* Latch Interrupt request to the INT_SOURCE register. */
  if(LPS35HW_LatchInterruptRequest(handle, pLPS35HWInt->LatchIRQ))
    return LPS35HW_ERROR;
  
  /* Set the threshold value  used for pressure interrupt generation (hPA). */
  if(LPS35HW_Set_PressureThreshold(handle, pLPS35HWInt->THS_threshold))
    return LPS35HW_ERROR;
  
  /*Enable/Disable  AutoRifP function */
  if(pLPS35HWInt->AutoRifP==LPS35HW_ENABLE)
  {
    if(LPS35HW_Set_AutoRifP(handle))
      return LPS35HW_ERROR;
  }
  else
  {
    if(LPS35HW_ResetAutoRifP(handle))
      return LPS35HW_ERROR;
  }
  
  /*Enable/Disable AutoZero function*/
  if(pLPS35HWInt->AutoZero==LPS35HW_ENABLE)
  {
    if(LPS35HW_Set_AutoZeroFunction(handle))
      return LPS35HW_ERROR;
  }
  else{
    if(LPS35HW_ResetAutoZeroFunction(handle))
      return LPS35HW_ERROR;
  }
  
  if(pLPS35HWInt->OutputSignal_INT==LPS35HW_P_HIGH)
  {
    /* Enable\Disable Interrupt Generation on differential pressure high event*/
    if(LPS35HW_Set_PHE(handle, LPS35HW_ENABLE))
      return LPS35HW_ERROR;
    if(LPS35HW_Set_InterruptDifferentialGeneration(handle, LPS35HW_ENABLE))
      return LPS35HW_ERROR;
  }
  else  if(pLPS35HWInt->OutputSignal_INT==LPS35HW_P_LOW)
  {
    /* Enable Interrupt Generation on differential pressure Loe event*/
    if(LPS35HW_Set_PLE(handle, LPS35HW_ENABLE))
    return LPS35HW_ERROR;
    if(LPS35HW_Set_InterruptDifferentialGeneration(handle, LPS35HW_ENABLE))
    return LPS35HW_ERROR;
  }
  else  if(pLPS35HWInt->OutputSignal_INT==LPS35HW_P_LOW_HIGH)
  {
    /* Enable Interrupt Generation on differential pressure high event*/
    if(LPS35HW_Set_PHE(handle, LPS35HW_ENABLE))
      return LPS35HW_ERROR;
    /* Enable\Disable Interrupt Generation on differential pressure Loe event*/
    if(LPS35HW_Set_PLE(handle, LPS35HW_ENABLE))
      return LPS35HW_ERROR;
    if(LPS35HW_Set_InterruptDifferentialGeneration(handle, LPS35HW_ENABLE))
      return LPS35HW_ERROR;
  }
  else
  {
    if(LPS35HW_Set_InterruptDifferentialGeneration(handle, LPS35HW_DISABLE))
      return LPS35HW_ERROR;
    /* Disable Interrupt Generation on differential pressure High event*/
    if(LPS35HW_Set_PHE(handle, LPS35HW_DISABLE))
      return LPS35HW_ERROR;
    /* Disable Interrupt Generation on differential pressure Low event*/
    if(LPS35HW_Set_PLE(handle, LPS35HW_DISABLE))
      return LPS35HW_ERROR;
  }
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief  LPS35HWGet_InterruptConfig
* @param *handle Device handle.
* @param  Struct to empty with configuration values
* @retval S Error code[LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Get_InterruptConfig(void *handle, LPS35HW_InterruptTypeDef_st* pLPS35HWInt)
{
  uint8_t tmp;
  
  /*Read LPS35HW_CTRL_REG3*/
  if(LPS35HW_ReadReg(handle, LPS35HW_CTRL_REG3, 1, &tmp))
    return LPS35HW_ERROR;
  
  pLPS35HWInt->INT_H_L= (LPS35HW_InterruptActiveLevel_et)(tmp & LPS35HW_INT_H_L_MASK);
  pLPS35HWInt->PP_OD=(LPS35HW_OutputType_et)(tmp & LPS35HW_PP_OD_MASK);
  pLPS35HWInt->OutputSignal_INT=(LPS35HW_OutputSignalConfig_et)(tmp& LPS35HW_INT_S12_MASK);
  pLPS35HWInt->DRDY=(LPS35HW_State_et)((tmp& LPS35HW_DRDY_MASK)>>LPS35HW_DRDY_BIT);
  pLPS35HWInt->FIFO_OVR=(LPS35HW_State_et)((tmp& LPS35HW_FIFO_OVR_MASK)>>LPS35HW_FIFO_OVR_BIT);
  pLPS35HWInt->FIFO_FTH=(LPS35HW_State_et)((tmp& LPS35HW_FIFO_FTH_MASK)>>LPS35HW_FIFO_FTH_BIT);
  pLPS35HWInt->FIFO_FULL=(LPS35HW_State_et)((tmp& LPS35HW_FIFO_FULL_MASK)>>LPS35HW_FIFO_FULL_BIT);
  
  /*Read LPS35HW_INTERRUPT_CFG_REG*/
  if(LPS35HW_ReadReg(handle, LPS35HW_INTERRUPT_CFG_REG, 1, &tmp))
    return LPS35HW_ERROR;
  
  pLPS35HWInt->LatchIRQ= (LPS35HW_State_et)((tmp& LPS35HW_LIR_MASK)>>LPS35HW_LIR_BIT);
  
  if(LPS35HW_Get_PressureThreshold(handle, &pLPS35HWInt->THS_threshold))
    return LPS35HW_ERROR;
  
  //AutoRifP and Autozero are self clear //
  pLPS35HWInt->AutoRifP=LPS35HW_DISABLE;
  pLPS35HWInt->AutoZero=LPS35HW_DISABLE;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief  Set Fifo configuration
* @param *handle Device handle.
* @param  Struct holding the configuration values
* @retval  Error code[LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Set_FifoConfig(void *handle, LPS35HW_FIFOTypeDef_st* pLPS35HWFIFO)
{
  
  if(pLPS35HWFIFO->FIFO_MODE == LPS35HW_FIFO_BYPASS_MODE) 
  {
    /* FIFO Disable-> FIFO_EN bit=0 in CTRL_REG2*/
    if(LPS35HW_Set_FifoModeUse(handle, LPS35HW_DISABLE))
      return LPS35HW_ERROR;
    
    /* Force->Disable FIFO Watermark Level Use*/
    if(LPS35HW_Set_FifoWatermarkLevelUse(handle, LPS35HW_DISABLE))
      return LPS35HW_ERROR;
    
    /* Force->Disable FIFO Treshold interrupt on INT_DRDY pin. */
    if(LPS35HW_Set_FIFO_FTH_Interrupt(handle, LPS35HW_DISABLE))
      return LPS35HW_ERROR;
  }
  else 
  {
    /* FIFO Enable-> FIFO_EN bit=1 in CTRL_REG2*/
    if(LPS35HW_Set_FifoModeUse(handle, LPS35HW_ENABLE))
      return LPS35HW_ERROR;
    
    if (pLPS35HWFIFO->WTM_INT)
    {
      /* Enable FIFO Watermark Level Use*/
      if(LPS35HW_Set_FifoWatermarkLevelUse(handle, LPS35HW_ENABLE))
        return LPS35HW_ERROR;
      /*Set Fifo Watermark Level*/
      if(LPS35HW_Set_FifoWatermarkLevel(handle, pLPS35HWFIFO->WTM_LEVEL))
        return LPS35HW_ERROR;
      /* Force->Enable FIFO Treshold interrupt on INT_DRDY pin. */
      if(LPS35HW_Set_FIFO_FTH_Interrupt(handle, LPS35HW_ENABLE))
        return LPS35HW_ERROR;
    }
  }
  
  if(LPS35HW_Set_FifoMode(handle, pLPS35HWFIFO->FIFO_MODE))
    return LPS35HW_ERROR;
  
  return LPS35HW_OK;
}

/*******************************************************************************
* @brief  Get Fifo configuration
* @param *handle Device handle.
* @param  Struct to empty with the configuration values
* @retval Error code[LPS35HW_ERROR, LPS35HW_OK]
*******************************************************************************/
LPS35HW_Error_et LPS35HW_Get_FifoConfig(void *handle, LPS35HW_FIFOTypeDef_st* pLPS35HWFIFO)
{
  uint8_t tmp;
  
  if(LPS35HW_ReadReg(handle, LPS35HW_CTRL_FIFO_REG, 1, &tmp))
    return LPS35HW_ERROR;
  
  /*!< Fifo Mode Selection */
  pLPS35HWFIFO->FIFO_MODE= (LPS35HW_FifoMode_et)(tmp& LPS35HW_FIFO_MODE_MASK);
  
  /*!< FIFO threshold/Watermark level selection*/
  pLPS35HWFIFO->WTM_LEVEL= (uint8_t)(tmp& LPS35HW_WTM_POINT_MASK);
  
  if(LPS35HW_ReadReg(handle, LPS35HW_CTRL_REG2, 1, &tmp))
    return LPS35HW_ERROR;
  
  /*!< Enable/Disable the watermark interrupt*/
  pLPS35HWFIFO->WTM_INT= (LPS35HW_State_et)((tmp& LPS35HW_WTM_EN_MASK)>>LPS35HW_WTM_EN_BIT);
  
  return LPS35HW_OK;
}

#ifdef  USE_FULL_ASSERT_LPS35HW
/*******************************************************************************
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param file: pointer to the source file name
* @param line: assert_param error line source number
* @retval : None
*******************************************************************************/
void LPS35HW_assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number */
  printf("Wrong parameters tmp: file %s on line %d\r\n", file, (int)line);
  
  /* Infinite loop */
  while (1)
  {
  }
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

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
