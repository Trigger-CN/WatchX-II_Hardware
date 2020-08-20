/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
* File Name          : LIS2DW12_ACC_driver.c
* Author             : MEMS Application Team
* Version            : v1.1
* Date               : 11 May 2017
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

/* Includes ------------------------------------------------------------------*/
#include "LIS2DW12_ACC_driver.h"
#include "spi.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/************** Generic Function  *******************/

/*******************************************************************************
  * Function Name    : LIS2DW12_ACC_WriteReg
  * Description    : Generic Writing function. It must be fullfilled with either
  *          : I2C or SPI writing function
  * Input        : Register Address, ptr to buffer to be written,
  *                                 length of buffer
  * Output      : None
  * Return      : None
*******************************************************************************/
status_t LIS2DW12_ACC_WriteReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len)
{
  /* Example 
    HAL_GPIO_WritePin(CS_DEV_GPIO_Port, CS_DEV_Pin, GPIO_PIN_RESET);  
    HAL_SPI_Transmit(&hspi2, &Reg, 1, 1000);
    HAL_SPI_Transmit(&hspi2, Bufp, len, 1000);
    HAL_GPIO_WritePin(CS_DEV_GPIO_Port, CS_DEV_Pin, GPIO_PIN_SET);    
  */
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name    : LIS2DW12_ACC_ReadReg
  * Description    : Generic Reading function. It must be fullfilled with either
  *          : I2C or SPI writing function
  * Input        : Register Address, ptr to buffer to be read,
  *                                 length of buffer
  * Output      : None
  * Return      : None
*******************************************************************************/
status_t LIS2DW12_ACC_ReadReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len)
{
  /* Example
    HAL_GPIO_WritePin(CS_DEV_GPIO_Port, CS_DEV_Pin, GPIO_PIN_RESET); 
    uint8_t dummy = Reg|0x80;
    HAL_SPI_Transmit(&hspi2, &dummy, 1, 1000);    
    HAL_SPI_Receive(&hspi2, Bufp, len, 1000);
    HAL_GPIO_WritePin(CS_DEV_GPIO_Port, CS_DEV_Pin, GPIO_PIN_SET); 
  */
  return MEMS_SUCCESS; 
}

/**************** Base Function  *******************/

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_WhoAmI
  * Description    : Read WHO_AM_I_BIT
  * Input          : Pointer to u8_t
  * Output         : Status of WHO_AM_I_BIT 
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_WhoAmI(void *handle, u8_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_WHO_AM_I_REG, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_WHO_AM_I_BIT_MASK; //coerce  
  *value = *value >> LIS2DW12_ACC_WHO_AM_I_BIT_POSITION; //mask  
  
    return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_OutputDataRate
  * Description    : Write ODR
  * Input          : LIS2DW12_ACC_ODR_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_OutputDataRate(void *handle, LIS2DW12_ACC_ODR_t newValue)
{
  u8_t value;
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_CTRL1, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_ODR_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_CTRL1, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_OutputDataRate
  * Description    : Read ODR
  * Input          : Pointer to LIS2DW12_ACC_ODR_t
  * Output         : Status of ODR see LIS2DW12_ACC_ODR_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_OutputDataRate(void *handle, LIS2DW12_ACC_ODR_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_CTRL1, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_ODR_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_FullScaleSelection
  * Description    : Write FS
  * Input          : LIS2DW12_ACC_FS_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_FullScaleSelection(void *handle, LIS2DW12_ACC_FS_t newValue)
{
  u8_t value;
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_CTRL6, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_FS_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_CTRL6, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_FullScaleSelection
  * Description    : Read FS
  * Input          : Pointer to LIS2DW12_ACC_FS_t
  * Output         : Status of FS see LIS2DW12_ACC_FS_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_FullScaleSelection(void *handle, LIS2DW12_ACC_FS_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_CTRL6, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_FS_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_BlockDataUpdate
  * Description    : Write BDU
  * Input          : LIS2DW12_ACC_BDU_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_BlockDataUpdate(void *handle, LIS2DW12_ACC_BDU_t newValue)
{
  u8_t value;
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_CTRL2, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_BDU_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_CTRL2, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_BlockDataUpdate
  * Description    : Read BDU
  * Input          : Pointer to LIS2DW12_ACC_BDU_t
  * Output         : Status of BDU see LIS2DW12_ACC_BDU_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_BlockDataUpdate(void *handle, LIS2DW12_ACC_BDU_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_CTRL2, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_BDU_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : status_t LIS2DW12_ACC_Get_Acceleration(u8_t *buff)
  * Description    : Read Acceleration output register
  * Input          : pointer to [u8_t]
  * Output         : Acceleration buffer u8_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_Get_Acceleration(void *handle, u8_t *buff) 
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;
  
  numberOfByteForDimension=6/3;
  
  k=0;
  for (i=0; i<3;i++ ) 
  {
    for (j=0; j<numberOfByteForDimension;j++ )
    {  
      if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_OUT_X_L+k, &buff[k], 1))
        return MEMS_ERROR;
      k++;  
    }
  }
  
  return MEMS_SUCCESS; 
}

/*******************************************************************************
  * Function Name  : status_t LIS2DW12_ACC_Get_Temperature(u8_t *buff)
  * Description    : Read Temperature output register
  * Input          : pointer to [u8_t]
  * Output         : Temperature buffer u8_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_Get_Temperature(void *handle, u8_t *buff) 
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;
  
  numberOfByteForDimension=2/1;
  
  k=0;
  for (i=0; i<1;i++ ) 
  {
    for (j=0; j<numberOfByteForDimension;j++ )
    {  
      if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_OUT_T_L+k, &buff[k], 1))
        return MEMS_ERROR;
      k++;  
    }
  }
  
  return MEMS_SUCCESS; 
}

/**************** Advanced Function  *******************/

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_LowPowerModeSelection
  * Description    : Write LP_MODE
  * Input          : LIS2DW12_ACC_LP_MODE_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_LowPowerModeSelection(void *handle, LIS2DW12_ACC_LP_MODE_t newValue)
{
  u8_t value;
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_CTRL1, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_LP_MODE_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_CTRL1, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_LowPowerModeSelection
  * Description    : Read LP_MODE
  * Input          : Pointer to LIS2DW12_ACC_LP_MODE_t
  * Output         : Status of LP_MODE see LIS2DW12_ACC_LP_MODE_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_LowPowerModeSelection(void *handle, LIS2DW12_ACC_LP_MODE_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_CTRL1, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_LP_MODE_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_ModeSelection
  * Description    : Write MODE
  * Input          : LIS2DW12_ACC_MODE_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_ModeSelection(void *handle, LIS2DW12_ACC_MODE_t newValue)
{
  u8_t value;
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_CTRL1, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_MODE_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_CTRL1, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_ModeSelection
  * Description    : Read MODE
  * Input          : Pointer to LIS2DW12_ACC_MODE_t
  * Output         : Status of MODE see LIS2DW12_ACC_MODE_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_ModeSelection(void *handle, LIS2DW12_ACC_MODE_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_CTRL1, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_MODE_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_SPI_mode_selection
  * Description    : Write SIM
  * Input          : LIS2DW12_ACC_SIM_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_SPI_mode_selection(void *handle, LIS2DW12_ACC_SIM_t newValue)
{
  u8_t value;
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_CTRL2, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_SIM_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_CTRL2, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_SPI_mode_selection
  * Description    : Read SIM
  * Input          : Pointer to LIS2DW12_ACC_SIM_t
  * Output         : Status of SIM see LIS2DW12_ACC_SIM_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_SPI_mode_selection(void *handle, LIS2DW12_ACC_SIM_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_CTRL2, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_SIM_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_I2C_status
  * Description    : Write I2C_DISABLE
  * Input          : LIS2DW12_ACC_I2C_DISABLE_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_I2C_status(void *handle, LIS2DW12_ACC_I2C_DISABLE_t newValue)
{
  u8_t value;
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_CTRL2, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_I2C_DISABLE_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_CTRL2, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_I2C_status
  * Description    : Read I2C_DISABLE
  * Input          : Pointer to LIS2DW12_ACC_I2C_DISABLE_t
  * Output         : Status of I2C_DISABLE see LIS2DW12_ACC_I2C_DISABLE_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_I2C_status(void *handle, LIS2DW12_ACC_I2C_DISABLE_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_CTRL2, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_I2C_DISABLE_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_AutoIncrementedWithMultipleAccess
  * Description    : Write IF_ADD_INC
  * Input          : LIS2DW12_ACC_IF_ADD_INC_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_AutoIncrementedWithMultipleAccess(void *handle, LIS2DW12_ACC_IF_ADD_INC_t newValue)
{
  u8_t value;
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_CTRL2, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_IF_ADD_INC_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_CTRL2, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_AutoIncrementedWithMultipleAccess
  * Description    : Read IF_ADD_INC
  * Input          : Pointer to LIS2DW12_ACC_IF_ADD_INC_t
  * Output         : Status of IF_ADD_INC see LIS2DW12_ACC_IF_ADD_INC_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_AutoIncrementedWithMultipleAccess(void *handle, LIS2DW12_ACC_IF_ADD_INC_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_CTRL2, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_IF_ADD_INC_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_SoftReset
  * Description    : Write SOFT_RESET
  * Input          : LIS2DW12_ACC_SOFT_RESET_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_SoftReset(void *handle, LIS2DW12_ACC_SOFT_RESET_t newValue)
{
  u8_t value;
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_CTRL2, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_SOFT_RESET_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_CTRL2, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_SoftReset
  * Description    : Read SOFT_RESET
  * Input          : Pointer to LIS2DW12_ACC_SOFT_RESET_t
  * Output         : Status of SOFT_RESET see LIS2DW12_ACC_SOFT_RESET_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_SoftReset(void *handle, LIS2DW12_ACC_SOFT_RESET_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_CTRL2, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_SOFT_RESET_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_Reboot
  * Description    : Write BOOT
  * Input          : LIS2DW12_ACC_BOOT_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_Reboot(void *handle, LIS2DW12_ACC_BOOT_t newValue)
{
  u8_t value;
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_CTRL2, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_BOOT_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_CTRL2, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_Reboot
  * Description    : Read BOOT
  * Input          : Pointer to LIS2DW12_ACC_BOOT_t
  * Output         : Status of BOOT see LIS2DW12_ACC_BOOT_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_Reboot(void *handle, LIS2DW12_ACC_BOOT_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_CTRL2, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_BOOT_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_SingleLowPowerMode
  * Description    : Write SLP_MODE_1
  * Input          : LIS2DW12_ACC_SLP_MODE_1_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_SingleLowPowerMode(void *handle, LIS2DW12_ACC_SLP_MODE_1_t newValue)
{
  u8_t value;
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_CTRL3, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_SLP_MODE_1_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_CTRL3, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_SingleLowPowerMode
  * Description    : Read SLP_MODE_1
  * Input          : Pointer to LIS2DW12_ACC_SLP_MODE_1_t
  * Output         : Status of SLP_MODE_1 see LIS2DW12_ACC_SLP_MODE_1_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_SingleLowPowerMode(void *handle, LIS2DW12_ACC_SLP_MODE_1_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_CTRL3, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_SLP_MODE_1_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_SingleLowPowerModeSource
  * Description    : Write SLP_MODE_SEL
  * Input          : LIS2DW12_ACC_SLP_MODE_SEL_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_SingleLowPowerModeSource(void *handle, LIS2DW12_ACC_SLP_MODE_SEL_t newValue)
{
  u8_t value;
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_CTRL3, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_SLP_MODE_SEL_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_CTRL3, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_SingleLowPowerModeSource
  * Description    : Read SLP_MODE_SEL
  * Input          : Pointer to LIS2DW12_ACC_SLP_MODE_SEL_t
  * Output         : Status of SLP_MODE_SEL see LIS2DW12_ACC_SLP_MODE_SEL_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_SingleLowPowerModeSource(void *handle, LIS2DW12_ACC_SLP_MODE_SEL_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_CTRL3, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_SLP_MODE_SEL_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_InterruptPolarity
  * Description    : Write H_LACTIVE
  * Input          : LIS2DW12_ACC_H_LACTIVE_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_InterruptPolarity(void *handle, LIS2DW12_ACC_H_LACTIVE_t newValue)
{
  u8_t value;
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_CTRL3, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_H_LACTIVE_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_CTRL3, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_InterruptPolarity
  * Description    : Read H_LACTIVE
  * Input          : Pointer to LIS2DW12_ACC_H_LACTIVE_t
  * Output         : Status of H_LACTIVE see LIS2DW12_ACC_H_LACTIVE_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_InterruptPolarity(void *handle, LIS2DW12_ACC_H_LACTIVE_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_CTRL3, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_H_LACTIVE_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_LatchIntteruptRq
  * Description    : Write LIR
  * Input          : LIS2DW12_ACC_LIR_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_LatchIntteruptRq(void *handle, LIS2DW12_ACC_LIR_t newValue)
{
  u8_t value;
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_CTRL3, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_LIR_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_CTRL3, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_LatchIntteruptRq
  * Description    : Read LIR
  * Input          : Pointer to LIS2DW12_ACC_LIR_t
  * Output         : Status of LIR see LIS2DW12_ACC_LIR_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_LatchIntteruptRq(void *handle, LIS2DW12_ACC_LIR_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_CTRL3, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_LIR_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_InterruptPadMode
  * Description    : Write PP_OD
  * Input          : LIS2DW12_ACC_PP_OD_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_InterruptPadMode(void *handle, LIS2DW12_ACC_PP_OD_t newValue)
{
  u8_t value;
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_CTRL3, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_PP_OD_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_CTRL3, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_InterruptPadMode
  * Description    : Read PP_OD
  * Input          : Pointer to LIS2DW12_ACC_PP_OD_t
  * Output         : Status of PP_OD see LIS2DW12_ACC_PP_OD_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_InterruptPadMode(void *handle, LIS2DW12_ACC_PP_OD_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_CTRL3, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_PP_OD_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_SelfTest
  * Description    : Write ST
  * Input          : LIS2DW12_ACC_ST_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_SelfTest(void *handle, LIS2DW12_ACC_ST_t newValue)
{
  u8_t value;
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_CTRL3, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_ST_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_CTRL3, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_SelfTest
  * Description    : Read ST
  * Input          : Pointer to LIS2DW12_ACC_ST_t
  * Output         : Status of ST1 see LIS2DW12_ACC_ST_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_SelfTest(void *handle, LIS2DW12_ACC_ST_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_CTRL3, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_ST_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_PinFunction_INT1
  * Description    : Write INT1_MODE
  * Input          : LIS2DW12_ACC_INT1_MODE_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_PinFunction_INT1(void *handle, LIS2DW12_ACC_INT1_MODE_t newValue)
{
  u8_t value;
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_CTRL4_INT1_PAD_CTRL, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_INT1_MODE_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_CTRL4_INT1_PAD_CTRL, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_PinFunction_INT1
  * Description    : Read INT1_MODE
  * Input          : Pointer to LIS2DW12_ACC_INT1_MODE_t
  * Output         : Status of INT1_MODE see LIS2DW12_ACC_INT1_MODE_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_PinFunction_INT1(void *handle, LIS2DW12_ACC_INT1_MODE_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_CTRL4_INT1_PAD_CTRL, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_INT1_MODE_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_PinFunction_INT2
  * Description    : Write INT2_DRDY
  * Input          : LIS2DW12_ACC_INT2_MODE_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_PinFunction_INT2(void *handle, LIS2DW12_ACC_INT2_MODE_t newValue)
{
  u8_t value;
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_CTRL5_INT2_PAD_CTRL, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_INT2_MODE_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_CTRL5_INT2_PAD_CTRL, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_PinFunction_INT2
  * Description    : Read INT2_DRDY
  * Input          : Pointer to LIS2DW12_ACC_INT2_DRDY_t
  * Output         : Status of INT2_DRDY see LIS2DW12_ACC_INT2_DRDY_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_PinFunction_INT2(void *handle, LIS2DW12_ACC_INT2_MODE_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_CTRL5_INT2_PAD_CTRL, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_INT2_MODE_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_LowNoiseConfiguration
  * Description    : Write LOW_NOISE
  * Input          : LIS2DW12_ACC_LOW_NOISE_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_LowNoiseConfiguration(void *handle, LIS2DW12_ACC_LOW_NOISE_t newValue)
{
  u8_t value;
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_CTRL6, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_LOW_NOISE_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_CTRL6, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_LowNoiseConfiguration
  * Description    : Read LOW_NOISE
  * Input          : Pointer to LIS2DW12_ACC_LOW_NOISE_t
  * Output         : Status of LOW_NOISE see LIS2DW12_ACC_LOW_NOISE_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_LowNoiseConfiguration(void *handle, LIS2DW12_ACC_LOW_NOISE_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_CTRL6, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_LOW_NOISE_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_FliterConfiguration
  * Description    : Write FDS
  * Input          : LIS2DW12_ACC_FDS_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_FliterConfiguration(void *handle, LIS2DW12_ACC_FDS_t newValue)
{
  u8_t value;
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_CTRL6, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_FDS_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_CTRL6, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_FliterConfiguration
  * Description    : Read FDS
  * Input          : Pointer to LIS2DW12_ACC_FDS_t
  * Output         : Status of FDS see LIS2DW12_ACC_FDS_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_FliterConfiguration(void *handle, LIS2DW12_ACC_FDS_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_CTRL6, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_FDS_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_FilterCutOff
  * Description    : Write BW_FILT
  * Input          : LIS2DW12_ACC_BW_FILT_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_FilterCutOff(void *handle, LIS2DW12_ACC_BW_FILT_t newValue)
{
  u8_t value;
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_CTRL6, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_BW_FILT_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_CTRL6, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_FilterCutOff
  * Description    : Read BW_FILT
  * Input          : Pointer to LIS2DW12_ACC_BW_FILT_t
  * Output         : Status of BW_FILT see LIS2DW12_ACC_BW_FILT_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_FilterCutOff(void *handle, LIS2DW12_ACC_BW_FILT_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_CTRL6, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_BW_FILT_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_TEMP_bits
  * Description    : Read TEMP
  * Input          : Pointer to u8_t
  * Output         : Status of TEMP 
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_TEMP_bits(void *handle, u8_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_OUT_T, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_TEMP_MASK; //coerce  
  *value = *value >> LIS2DW12_ACC_TEMP_POSITION; //mask  
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_GetStatus
  * Description    : Read DRDY
  * Input          : Pointer to LIS2DW12_ACC_STATUS_t
  * Output         : Status of DRDY see LIS2DW12_ACC_STATUS_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_GetStatus(void *handle, LIS2DW12_ACC_STATUS_t *value)
{
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_STATUS, (u8_t *)value , 1) )
    return MEMS_ERROR;
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_STATUS_DUP, ((u8_t *)value+1) , 1) )
    return MEMS_ERROR;
    
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_SetFIFO_Threshold
  * Description    : Write FTH
  * Input          : u8_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_FIFO_Threshold(void *handle, u8_t newValue)
{
  u8_t value;
  
  newValue = newValue << LIS2DW12_ACC_FTH_POSITION; //mask  
  newValue &= LIS2DW12_ACC_FTH_MASK; //coerce
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_FIFO_CTRL, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_FTH_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_FIFO_CTRL, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_SetFIFO_Threshold
  * Description    : Read FTH
  * Input          : Pointer to u8_t
  * Output         : Status of FTH 
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_FIFO_Threshold(void *handle, u8_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_FIFO_CTRL, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_FTH_MASK; //coerce  
  *value = *value >> LIS2DW12_ACC_FTH_POSITION; //mask  
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_FIFO_mode
  * Description    : Write FMODE
  * Input          : LIS2DW12_ACC_FMODE_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_FIFO_mode(void *handle, LIS2DW12_ACC_FMODE_t newValue)
{
  u8_t value;
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_FIFO_CTRL, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_FMODE_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_FIFO_CTRL, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_FIFO_mode
  * Description    : Read FMODE
  * Input          : Pointer to LIS2DW12_ACC_FMODE_t
  * Output         : Status of FMODE see LIS2DW12_ACC_FMODE_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_FIFO_mode(void *handle, LIS2DW12_ACC_FMODE_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_FIFO_CTRL, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_FMODE_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_FIFO_level
  * Description    : Read DIFF
  * Input          : Pointer to u8_t
  * Output         : Status of DIFF 
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_FIFO_level(void *handle, u8_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_FIFO_SAMPLES, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_DIFF_MASK; //coerce  
  *value = *value >> LIS2DW12_ACC_DIFF_POSITION; //mask  
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_FIFO_OverrunStatus
  * Description    : Read FIFO_OVR
  * Input          : Pointer to LIS2DW12_ACC_FIFO_OVR_t
  * Output         : Status of FIFO_OVR see LIS2DW12_ACC_FIFO_OVR_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_FIFO_OverrunStatus(void *handle, LIS2DW12_ACC_FIFO_OVR_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_FIFO_SAMPLES, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_FIFO_OVR_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_FIFO_Threshold_Status
  * Description    : Read FIFO_FTH
  * Input          : Pointer to LIS2DW12_ACC_FIFO_FTH_t
  * Output         : Status of FIFO_FTH see LIS2DW12_ACC_FIFO_FTH_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_FIFO_Threshold_Status(void *handle, LIS2DW12_ACC_FIFO_FTH_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_FIFO_SAMPLES, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_FIFO_FTH_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_TAP_X_Threshold
  * Description    : Write TAP_THSX
  * Input          : u8_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_TAP_X_Threshold(void *handle, u8_t newValue)
{
  u8_t value;
  
  newValue = newValue << LIS2DW12_ACC_TAP_THSX_POSITION; //mask  
  newValue &= LIS2DW12_ACC_TAP_THSX_MASK; //coerce
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_TAP_THS_X, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_TAP_THSX_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_TAP_THS_X, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_TAP_X_Threshold
  * Description    : Read TAP_THSX
  * Input          : Pointer to u8_t
  * Output         : Status of TAP_THSX 
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_TAP_X_Threshold(void *handle, u8_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_TAP_THS_X, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_TAP_THSX_MASK; //coerce  
  *value = *value >> LIS2DW12_ACC_TAP_THSX_POSITION; //mask  
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_6D_Threshold
  * Description    : Write 6D_THS
  * Input          : LIS2DW12_ACC_6D_THS_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_6D_Threshold(void *handle, LIS2DW12_ACC_6D_THS_t newValue)
{
  u8_t value;
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_TAP_THS_X, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_6D_THS_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_TAP_THS_X, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_6D_Threshold
  * Description    : Read 6D_THS
  * Input          : Pointer to LIS2DW12_ACC_6D_THS_t
  * Output         : Status of 6D_THS see LIS2DW12_ACC_6D_THS_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_6D_Threshold(void *handle, LIS2DW12_ACC_6D_THS_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_TAP_THS_X, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_6D_THS_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_4D_Function
  * Description    : Write 4D_EN
  * Input          : LIS2DW12_ACC_4D_EN_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_4D_Function(void *handle, LIS2DW12_ACC_4D_EN_t newValue)
{
  u8_t value;
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_TAP_THS_X, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_4D_EN_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_TAP_THS_X, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_4D_Function
  * Description    : Read 4D_EN
  * Input          : Pointer to LIS2DW12_ACC_4D_EN_t
  * Output         : Status of 4D_EN see LIS2DW12_ACC_4D_EN_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_4D_Function(void *handle, LIS2DW12_ACC_4D_EN_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_TAP_THS_X, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_4D_EN_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_TAP_Y_Threshold
  * Description    : Write TAP_THSY
  * Input          : u8_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_TAP_Y_Threshold(void *handle, u8_t newValue)
{
  u8_t value;
  
  newValue = newValue << LIS2DW12_ACC_TAP_THSY_POSITION; //mask  
  newValue &= LIS2DW12_ACC_TAP_THSY_MASK; //coerce
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_TAP_THS_Y, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_TAP_THSY_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_TAP_THS_Y, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_TAP_Y_Threshold
  * Description    : Read TAP_THSY
  * Input          : Pointer to u8_t
  * Output         : Status of TAP_THSY 
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_TAP_Y_Threshold(void *handle, u8_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_TAP_THS_Y, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_TAP_THSY_MASK; //coerce  
  *value = *value >> LIS2DW12_ACC_TAP_THSY_POSITION; //mask  
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_TAP_PriorityAxisTreshold
  * Description    : Write TAP_PRIOR
  * Input          : LIS2DW12_ACC_TAP_PRIOR_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_TAP_PriorityAxisTreshold(void *handle, LIS2DW12_ACC_TAP_PRIOR_t newValue)
{
  u8_t value;
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_TAP_THS_Y, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_TAP_PRIOR_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_TAP_THS_Y, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_TAP_PriorityAxisTreshold
  * Description    : Read TAP_PRIOR
  * Input          : Pointer to LIS2DW12_ACC_TAP_PRIOR_t
  * Output         : Status of TAP_PRIOR see LIS2DW12_ACC_TAP_PRIOR_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_TAP_PriorityAxisTreshold(void *handle, LIS2DW12_ACC_TAP_PRIOR_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_TAP_THS_Y, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_TAP_PRIOR_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_TAP_Z_Threshold
  * Description    : Write TAP_THSZ
  * Input          : u8_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_TAP_Z_Threshold(void *handle, u8_t newValue)
{
  u8_t value;
  
  newValue = newValue << LIS2DW12_ACC_TAP_THSZ_POSITION; //mask  
  newValue &= LIS2DW12_ACC_TAP_THSZ_MASK; //coerce
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_TAP_THS_Z, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_TAP_THSZ_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_TAP_THS_Z, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_TAP_Z_Threshold
  * Description    : Read TAP_THSZ
  * Input          : Pointer to u8_t
  * Output         : Status of TAP_THSZ 
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_TAP_Z_Threshold(void *handle, u8_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_TAP_THS_Z, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_TAP_THSZ_MASK; //coerce  
  *value = *value >> LIS2DW12_ACC_TAP_THSZ_POSITION; //mask  
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_DetectTAP_on_Z_Axis
  * Description    : Write TAP_Z_EN
  * Input          : LIS2DW12_ACC_TAP_Z_EN_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_DetectTAP_on_Z_Axis(void *handle, LIS2DW12_ACC_TAP_Z_EN_t newValue)
{
  u8_t value;
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_TAP_THS_Z, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_TAP_Z_EN_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_TAP_THS_Z, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_DetectTAP_on_Z_Axis
  * Description    : Read TAP_Z_EN
  * Input          : Pointer to LIS2DW12_ACC_TAP_Z_EN_t
  * Output         : Status of TAP_Z_EN see LIS2DW12_ACC_TAP_Z_EN_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_DetectTAP_on_Z_Axis(void *handle, LIS2DW12_ACC_TAP_Z_EN_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_TAP_THS_Z, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_TAP_Z_EN_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_DetectTAP_on_Y_Axis
  * Description    : Write TAP_Y_EN
  * Input          : LIS2DW12_ACC_TAP_Y_EN_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_DetectTAP_on_Y_Axis(void *handle, LIS2DW12_ACC_TAP_Y_EN_t newValue)
{
  u8_t value;
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_TAP_THS_Z, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_TAP_Y_EN_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_TAP_THS_Z, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_DetectTAP_on_Y_Axis
  * Description    : Read TAP_Y_EN
  * Input          : Pointer to LIS2DW12_ACC_TAP_Y_EN_t
  * Output         : Status of TAP_Y_EN see LIS2DW12_ACC_TAP_Y_EN_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_DetectTAP_on_Y_Axis(void *handle, LIS2DW12_ACC_TAP_Y_EN_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_TAP_THS_Z, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_TAP_Y_EN_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_DetectTAP_on_X_Axis
  * Description    : Write TAP_X_EN
  * Input          : LIS2DW12_ACC_TAP_X_EN_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_DetectTAP_on_X_Axis(void *handle, LIS2DW12_ACC_TAP_X_EN_t newValue)
{
  u8_t value;
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_TAP_THS_Z, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_TAP_X_EN_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_TAP_THS_Z, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_DetectTAP_on_X_Axis
  * Description    : Read TAP_X_EN
  * Input          : Pointer to LIS2DW12_ACC_TAP_X_EN_t
  * Output         : Status of TAP_X_EN see LIS2DW12_ACC_TAP_X_EN_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_DetectTAP_on_X_Axis(void *handle, LIS2DW12_ACC_TAP_X_EN_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_TAP_THS_Z, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_TAP_X_EN_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_MaxDurationOverThreshold
  * Description    : Write SHOCK
  * Input          : u8_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_MaxDurationOverThreshold(void *handle, u8_t newValue)
{
  u8_t value;
  
  newValue = newValue << LIS2DW12_ACC_SHOCK_POSITION; //mask  
  newValue &= LIS2DW12_ACC_SHOCK_MASK; //coerce
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_INT_DUR, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_SHOCK_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_INT_DUR, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_MaxDurationOverThreshold
  * Description    : Read SHOCK
  * Input          : Pointer to u8_t
  * Output         : Status of SHOCK 
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_MaxDurationOverThreshold(void *handle, u8_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_INT_DUR, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_SHOCK_MASK; //coerce  
  *value = *value >> LIS2DW12_ACC_SHOCK_POSITION; //mask  
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_QuietTimeAfterTAP
  * Description    : Write QUIET
  * Input          : u8_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_QuietTimeAfterTAP(void *handle, u8_t newValue)
{
  u8_t value;
  
  newValue = newValue << LIS2DW12_ACC_QUIET_POSITION; //mask  
  newValue &= LIS2DW12_ACC_QUIET_MASK; //coerce
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_INT_DUR, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_QUIET_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_INT_DUR, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_QuietTimeAfterTAP
  * Description    : Read QUIET
  * Input          : Pointer to u8_t
  * Output         : Status of QUIET 
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_QuietTimeAfterTAP(void *handle, u8_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_INT_DUR, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_QUIET_MASK; //coerce  
  *value = *value >> LIS2DW12_ACC_QUIET_POSITION; //mask  
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_DoubleTAP_Latency
  * Description    : Write LATENCY
  * Input          : u8_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_DoubleTAP_Latency(void *handle, u8_t newValue)
{
  u8_t value;
  
  newValue = newValue << LIS2DW12_ACC_LATENCY_POSITION; //mask  
  newValue &= LIS2DW12_ACC_LATENCY_MASK; //coerce
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_INT_DUR, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_LATENCY_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_INT_DUR, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_DoubleTAP_Latency
  * Description    : Read LATENCY
  * Input          : Pointer to u8_t
  * Output         : Status of LATENCY 
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_DoubleTAP_Latency(void *handle, u8_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_INT_DUR, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_LATENCY_MASK; //coerce  
  *value = *value >> LIS2DW12_ACC_LATENCY_POSITION; //mask  
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_WakeUpThreshold
  * Description    : Write WK_THS
  * Input          : u8_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_WakeUpThreshold(void *handle, u8_t newValue)
{
  u8_t value;
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_WAKE_UP_THS, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_WK_THS_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_WAKE_UP_THS, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_WakeUpThreshold
  * Description    : Read WK_THS
  * Input          : Pointer to u8_t
  * Output         : Status of WK_THS see LIS2DW12_ACC_WK_THS_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_WakeUpThreshold(void *handle, u8_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_WAKE_UP_THS, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_WK_THS_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_SleepFeature
  * Description    : Write SLEEP_ON
  * Input          : LIS2DW12_ACC_SLEEP_ON_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_SleepFeature(void *handle, LIS2DW12_ACC_SLEEP_ON_t newValue)
{
  u8_t value;
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_WAKE_UP_THS, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_SLEEP_ON_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_WAKE_UP_THS, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_SleepFeature
  * Description    : Read SLEEP_ON
  * Input          : Pointer to LIS2DW12_ACC_SLEEP_ON_t
  * Output         : Status of SLEEP_ON see LIS2DW12_ACC_SLEEP_ON_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_SleepFeature(void *handle, LIS2DW12_ACC_SLEEP_ON_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_WAKE_UP_THS, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_SLEEP_ON_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_TAP_mode
  * Description    : Write SINGLE_DOUBLE_TAP
  * Input          : LIS2DW12_ACC_SINGLE_DOUBLE_TAP_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_TAP_mode(void *handle, LIS2DW12_ACC_SINGLE_DOUBLE_TAP_t newValue)
{
  u8_t value;
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_WAKE_UP_THS, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_SINGLE_DOUBLE_TAP_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_WAKE_UP_THS, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_TAP_mode
  * Description    : Read SINGLE_DOUBLE_TAP
  * Input          : Pointer to LIS2DW12_ACC_SINGLE_DOUBLE_TAP_t
  * Output         : Status of SINGLE_DOUBLE_TAP see LIS2DW12_ACC_SINGLE_DOUBLE_TAP_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_TAP_mode(void *handle, LIS2DW12_ACC_SINGLE_DOUBLE_TAP_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_WAKE_UP_THS, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_SINGLE_DOUBLE_TAP_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_InactivityBeforeSleep
  * Description    : Write SLEEP_DUR
  * Input          : u8_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_InactivityBeforeSleep(void *handle, u8_t newValue)
{
  u8_t value;
  
  newValue = newValue << LIS2DW12_ACC_SLEEP_DUR_POSITION; //mask  
  newValue &= LIS2DW12_ACC_SLEEP_DUR_MASK; //coerce
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_WAKE_UP_DUR, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_SLEEP_DUR_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_WAKE_UP_DUR, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_InactivityBeforeSleep
  * Description    : Read SLEEP_DUR
  * Input          : Pointer to u8_t
  * Output         : Status of SLEEP_DUR 
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_InactivityBeforeSleep(void *handle, u8_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_WAKE_UP_DUR, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_SLEEP_DUR_MASK; //coerce  
  *value = *value >> LIS2DW12_ACC_SLEEP_DUR_POSITION; //mask  
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_WakeUpDuration
  * Description    : Write WAKE_DUR
  * Input          : u8_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_WakeUpDuration(void *handle, u8_t newValue)
{
  u8_t value;
  
  newValue = newValue << LIS2DW12_ACC_WAKE_DUR_POSITION; //mask  
  newValue &= LIS2DW12_ACC_WAKE_DUR_MASK; //coerce
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_WAKE_UP_DUR, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_WAKE_DUR_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_WAKE_UP_DUR, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_WakeUpDuration
  * Description    : Read WAKE_DUR
  * Input          : Pointer to u8_t
  * Output         : Status of WAKE_DUR 
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_WakeUpDuration(void *handle, u8_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_WAKE_UP_DUR, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_WAKE_DUR_MASK; //coerce  
  *value = *value >> LIS2DW12_ACC_WAKE_DUR_POSITION; //mask  
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_FreeFallDuration
  * Description    : Write FF_DUR5
  * Input          : u8_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_FreeFallDuration(void *handle, u8_t newValue)
{
  u8_t value;
  
  newValue = newValue << LIS2DW12_ACC_FF_DUR5_POSITION; //mask  
  newValue &= LIS2DW12_ACC_FF_DUR5_MASK; //coerce
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_WAKE_UP_DUR, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_FF_DUR5_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_WAKE_UP_DUR, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_FreeFallDuration
  * Description    : Read FF_DUR5
  * Input          : Pointer to u8_t
  * Output         : Status of FF_DUR5 
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_FreeFallDuration(void *handle, u8_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_WAKE_UP_DUR, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_FF_DUR5_MASK; //coerce  
  *value = *value >> LIS2DW12_ACC_FF_DUR5_POSITION; //mask  
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_FreeFallThreshold
  * Description    : Write FF_THS
  * Input          : u8_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_FreeFallThreshold(void *handle, u8_t newValue)
{
  u8_t value;
  
  newValue = newValue << LIS2DW12_ACC_FF_THS_POSITION; //mask  
  newValue &= LIS2DW12_ACC_FF_THS_MASK; //coerce
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_FREE_FALL, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_FF_THS_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_FREE_FALL, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_FreeFallThreshold
  * Description    : Read FF_THS
  * Input          : Pointer to u8_t
  * Output         : Status of FF_THS 
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_FreeFallThreshold(void *handle, u8_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_FREE_FALL, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_FF_THS_MASK; //coerce  
  *value = *value >> LIS2DW12_ACC_FF_THS_POSITION; //mask  
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_LIS2DW12_ACC_R_FreeFallDuration
  * Description    : Write FF_DUR
  * Input          : u8_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_LIS2DW12_ACC_R_FreeFallDuration(void *handle, u8_t newValue)
{
  u8_t value;
  
  newValue = newValue << LIS2DW12_ACC_FF_DUR_POSITION; //mask  
  newValue &= LIS2DW12_ACC_FF_DUR_MASK; //coerce
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_FREE_FALL, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_FF_DUR_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_FREE_FALL, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_LIS2DW12_ACC_R_FreeFallDuration
  * Description    : Read FF_DUR
  * Input          : Pointer to u8_t
  * Output         : Status of FF_DUR 
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_LIS2DW12_ACC_R_FreeFallDuration(void *handle, u8_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_FREE_FALL, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_FF_DUR_MASK; //coerce  
  *value = *value >> LIS2DW12_ACC_FF_DUR_POSITION; //mask  
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_WakeUpEvent_On_Z
  * Description    : Read Z_WU
  * Input          : Pointer to LIS2DW12_ACC_Z_WU_t
  * Output         : Status of Z_WU see LIS2DW12_ACC_Z_WU_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_WakeUpEvent_On_Z(void *handle, LIS2DW12_ACC_Z_WU_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_WAKE_UP_SRC, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_Z_WU_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_WakeUpEvent_On_Y
  * Description    : Read Y_WU
  * Input          : Pointer to LIS2DW12_ACC_Y_WU_t
  * Output         : Status of Y_WU see LIS2DW12_ACC_Y_WU_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_WakeUpEvent_On_Y(void *handle, LIS2DW12_ACC_Y_WU_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_WAKE_UP_SRC, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_Y_WU_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_LIS2DW12_ACC_R_WakeUpEvent_On_X
  * Description    : Read X_WU
  * Input          : Pointer to LIS2DW12_ACC_X_WU_t
  * Output         : Status of X_WU see LIS2DW12_ACC_X_WU_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_LIS2DW12_ACC_R_WakeUpEvent_On_X(void *handle, LIS2DW12_ACC_X_WU_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_WAKE_UP_SRC, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_X_WU_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_TapOn_Z_event
  * Description    : Read Z_TAP
  * Input          : Pointer to LIS2DW12_ACC_Z_TAP_t
  * Output         : Status of Z_TAP see LIS2DW12_ACC_Z_TAP_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_TapOn_Z_event(void *handle, LIS2DW12_ACC_Z_TAP_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_TAP_SRC, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_Z_TAP_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_TapOn_Y_event
  * Description    : Read Y_TAP
  * Input          : Pointer to LIS2DW12_ACC_Y_TAP_t
  * Output         : Status of Y_TAP see LIS2DW12_ACC_Y_TAP_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_TapOn_Y_event(void *handle, LIS2DW12_ACC_Y_TAP_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_TAP_SRC, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_Y_TAP_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_TapOn_X_event
  * Description    : Read X_TAP
  * Input          : Pointer to LIS2DW12_ACC_X_TAP_t
  * Output         : Status of X_TAP see LIS2DW12_ACC_X_TAP_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_TapOn_X_event(void *handle, LIS2DW12_ACC_X_TAP_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_TAP_SRC, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_X_TAP_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_TAP_sign
  * Description    : Read TAP_SIGN
  * Input          : Pointer to LIS2DW12_ACC_TAP_SIGN_t
  * Output         : Status of TAP_SIGN see LIS2DW12_ACC_TAP_SIGN_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_TAP_sign(void *handle, LIS2DW12_ACC_TAP_SIGN_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_TAP_SRC, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_TAP_SIGN_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_TAP_event
  * Description    : Read TAP_IA
  * Input          : Pointer to LIS2DW12_ACC_TAP_IA_t
  * Output         : Status of TAP_IA see LIS2DW12_ACC_TAP_IA_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_TAP_event(void *handle, LIS2DW12_ACC_TAP_IA_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_TAP_SRC, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_TAP_IA_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_6D_X_Low
  * Description    : Read XL
  * Input          : Pointer to LIS2DW12_ACC_XL_t
  * Output         : Status of XL see LIS2DW12_ACC_XL_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_6D_X_Low(void *handle, LIS2DW12_ACC_XL_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_SIXD_SRC, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_XL_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_6D_X_High
  * Description    : Read XH
  * Input          : Pointer to LIS2DW12_ACC_XH_t
  * Output         : Status of XH see LIS2DW12_ACC_XH_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_6D_X_High(void *handle, LIS2DW12_ACC_XH_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_SIXD_SRC, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_XH_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_6D_Y_Low
  * Description    : Read YL
  * Input          : Pointer to LIS2DW12_ACC_YL_t
  * Output         : Status of YL see LIS2DW12_ACC_YL_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_6D_Y_Low(void *handle, LIS2DW12_ACC_YL_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_SIXD_SRC, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_YL_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_6D_Y_High
  * Description    : Read YH
  * Input          : Pointer to LIS2DW12_ACC_YH_t
  * Output         : Status of YH see LIS2DW12_ACC_YH_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_6D_Y_High(void *handle, LIS2DW12_ACC_YH_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_SIXD_SRC, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_YH_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_6D_Z_Low
  * Description    : Read ZL
  * Input          : Pointer to LIS2DW12_ACC_ZL_t
  * Output         : Status of ZL see LIS2DW12_ACC_ZL_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_6D_Z_Low(void *handle, LIS2DW12_ACC_ZL_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_SIXD_SRC, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_ZL_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_6D_Z_High
  * Description    : Read ZH
  * Input          : Pointer to LIS2DW12_ACC_ZH_t
  * Output         : Status of ZH see LIS2DW12_ACC_ZH_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_6D_Z_High(void *handle, LIS2DW12_ACC_ZH_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_SIXD_SRC, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_ZH_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_SleepChange_event
  * Description    : Read SLEEP_CHANGE_IA
  * Input          : Pointer to LIS2DW12_ACC_SLEEP_CHANGE_IA_t
  * Output         : Status of SLEEP_CHANGE_IA see LIS2DW12_ACC_SLEEP_CHANGE_IA_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_SleepChange_event(void *handle, LIS2DW12_ACC_SLEEP_CHANGE_IA_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_ALL_INT_SRC, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_SLEEP_CHANGE_IA_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_WakeUpOffset_X
  * Description    : Write ABS_INT_X
  * Input          : u8_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_WakeUpOffset_X(void *handle, u8_t newValue)
{
  u8_t value;
  
  newValue = newValue << LIS2DW12_ACC_ABS_INT_X_POSITION; //mask  
  newValue &= LIS2DW12_ACC_ABS_INT_X_MASK; //coerce
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_ABS_INT_X, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_ABS_INT_X_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_ABS_INT_X, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_WakeUpOffset_X
  * Description    : Read ABS_INT_X
  * Input          : Pointer to u8_t
  * Output         : Status of ABS_INT_X 
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_WakeUpOffset_X(void *handle, u8_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_ABS_INT_X, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_ABS_INT_X_MASK; //coerce  
  *value = *value >> LIS2DW12_ACC_ABS_INT_X_POSITION; //mask  
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_WakeUpOffset_Y
  * Description    : Write ABS_INT_Y
  * Input          : u8_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_WakeUpOffset_Y(void *handle, u8_t newValue)
{
  u8_t value;
  
  newValue = newValue << LIS2DW12_ACC_ABS_INT_Y_POSITION; //mask  
  newValue &= LIS2DW12_ACC_ABS_INT_Y_MASK; //coerce
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_ABS_INT_Y, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_ABS_INT_Y_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_ABS_INT_Y, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_WakeUpOffset_Y
  * Description    : Read ABS_INT_Y
  * Input          : Pointer to u8_t
  * Output         : Status of ABS_INT_Y 
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_WakeUpOffset_Y(void *handle, u8_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_ABS_INT_Y, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_ABS_INT_Y_MASK; //coerce  
  *value = *value >> LIS2DW12_ACC_ABS_INT_Y_POSITION; //mask  
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_WakeUpOffset_Z
  * Description    : Write ABS_INT_Z
  * Input          : u8_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_WakeUpOffset_Z(void *handle, u8_t newValue)
{
  u8_t value;
  
  newValue = newValue << LIS2DW12_ACC_ABS_INT_Z_POSITION; //mask  
  newValue &= LIS2DW12_ACC_ABS_INT_Z_MASK; //coerce
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_ABS_INT_Z, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_ABS_INT_Z_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_ABS_INT_Z, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_WakeUpOffset_Z
  * Description    : Read ABS_INT_Z
  * Input          : Pointer to u8_t
  * Output         : Status of ABS_INT_Z 
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_WakeUpOffset_Z(void *handle, u8_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_ABS_INT_Z, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_ABS_INT_Z_MASK; //coerce  
  *value = *value >> LIS2DW12_ACC_ABS_INT_Z_POSITION; //mask  
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_6D_FilterInput
  * Description    : Write LPASS_ON6D
  * Input          : LIS2DW12_ACC_LPASS_ON6D_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_6D_FilterInput(void *handle,LIS2DW12_ACC_LPASS_ON6D_t newValue)
{
  u8_t value;
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_ABS_INT_CFG, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_LPASS_ON6D_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_ABS_INT_CFG, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_6D_FilterInput
  * Description    : Read LPASS_ON6D
  * Input          : Pointer to LIS2DW12_ACC_LPASS_ON6D_t
  * Output         : Status of LPASS_ON6D see LIS2DW12_ACC_LPASS_ON6D_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_6D_FilterInput(void *handle,LIS2DW12_ACC_LPASS_ON6D_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_ABS_INT_CFG, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_LPASS_ON6D_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_HP_FilterReference
  * Description    : Write HP_REF_MODE
  * Input          : LIS2DW12_ACC_HP_REF_MODE_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_HP_FilterReference(void *handle,LIS2DW12_ACC_HP_REF_MODE_t newValue)
{
  u8_t value;
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_ABS_INT_CFG, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_HP_REF_MODE_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_ABS_INT_CFG, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_HP_FilterReference
  * Description    : Read HP_REF_MODE
  * Input          : Pointer to LIS2DW12_ACC_HP_REF_MODE_t
  * Output         : Status of HP_REF_MODE see LIS2DW12_ACC_HP_REF_MODE_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_HP_FilterReference(void *handle,LIS2DW12_ACC_HP_REF_MODE_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_ABS_INT_CFG, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_HP_REF_MODE_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_UserOffsetWeight
  * Description    : Write USR_OFF_W
  * Input          : LIS2DW12_ACC_USR_OFF_W_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_UserOffsetWeight(void *handle,LIS2DW12_ACC_USR_OFF_W_t newValue)
{
  u8_t value;
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_ABS_INT_CFG, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_USR_OFF_W_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_ABS_INT_CFG, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_UserOffsetWeight
  * Description    : Read USR_OFF_W
  * Input          : Pointer to LIS2DW12_ACC_USR_OFF_W_t
  * Output         : Status of USR_OFF_W see LIS2DW12_ACC_USR_OFF_W_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_UserOffsetWeight(void *handle,LIS2DW12_ACC_USR_OFF_W_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_ABS_INT_CFG, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_USR_OFF_W_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_UserOffsetOnWakeUp
  * Description    : Write USR_OFF_ON_WU
  * Input          : LIS2DW12_ACC_USR_OFF_ON_WU_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_UserOffsetOnWakeUp(void *handle,LIS2DW12_ACC_USR_OFF_ON_WU_t newValue)
{
  u8_t value;
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_ABS_INT_CFG, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_USR_OFF_ON_WU_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_ABS_INT_CFG, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_UserOffsetOnWakeUp
  * Description    : Read USR_OFF_ON_WU
  * Input          : Pointer to LIS2DW12_ACC_USR_OFF_ON_WU_t
  * Output         : Status of USR_OFF_ON_WU see LIS2DW12_ACC_USR_OFF_ON_WU_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_UserOffsetOnWakeUp(void *handle,LIS2DW12_ACC_USR_OFF_ON_WU_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_ABS_INT_CFG, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_USR_OFF_ON_WU_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_UserOffsetOnOutputs
  * Description    : Write USR_OFF_ON_OUT
  * Input          : LIS2DW12_ACC_USR_OFF_ON_OUT_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_UserOffsetOnOutputs(void *handle,LIS2DW12_ACC_USR_OFF_ON_OUT_t newValue)
{
  u8_t value;
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_ABS_INT_CFG, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_USR_OFF_ON_OUT_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_ABS_INT_CFG, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_UserOffsetOnOutputs
  * Description    : Read USR_OFF_ON_OUT
  * Input          : Pointer to LIS2DW12_ACC_USR_OFF_ON_OUT_t
  * Output         : Status of USR_OFF_ON_OUT see LIS2DW12_ACC_USR_OFF_ON_OUT_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_UserOffsetOnOutputs(void *handle,LIS2DW12_ACC_USR_OFF_ON_OUT_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_ABS_INT_CFG, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_USR_OFF_ON_OUT_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_HardwarePin
  * Description    : Write INTERRUPTS_ENABLE
  * Input          : LIS2DW12_ACC_INTERRUPTS_ENABLE_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_HardwarePin(void *handle,LIS2DW12_ACC_INTERRUPTS_ENABLE_t newValue)
{
  u8_t value;
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_ABS_INT_CFG, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_INTERRUPTS_ENABLE_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_ABS_INT_CFG, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_HardwarePin
  * Description    : Read INTERRUPTS_ENABLE
  * Input          : Pointer to LIS2DW12_ACC_INTERRUPTS_ENABLE_t
  * Output         : Status of INTERRUPTS_ENABLE see LIS2DW12_ACC_INTERRUPTS_ENABLE_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_HardwarePin(void *handle,LIS2DW12_ACC_INTERRUPTS_ENABLE_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_ABS_INT_CFG, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_INTERRUPTS_ENABLE_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_UseInt1_PinOnly
  * Description    : Write INT2_ON_INT1
  * Input          : LIS2DW12_ACC_INT2_ON_INT1_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_UseInt1_PinOnly(void *handle,LIS2DW12_ACC_INT2_ON_INT1_t newValue)
{
  u8_t value;
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_ABS_INT_CFG, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_INT2_ON_INT1_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_ABS_INT_CFG, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_UseInt1_PinOnly
  * Description    : Read INT2_ON_INT1
  * Input          : Pointer to LIS2DW12_ACC_INT2_ON_INT1_t
  * Output         : Status of INT2_ON_INT1 see LIS2DW12_ACC_INT2_ON_INT1_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_UseInt1_PinOnly(void *handle,LIS2DW12_ACC_INT2_ON_INT1_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_ABS_INT_CFG, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_INT2_ON_INT1_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_W_DataReady
  * Description    : Write DRDY_PULSED
  * Input          : LIS2DW12_ACC_DRDY_PULSED_t
  * Output         : None
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LIS2DW12_ACC_W_DataReady(void *handle,LIS2DW12_ACC_DRDY_PULSED_t newValue)
{
  u8_t value;
  
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_ABS_INT_CFG, &value, 1) )
    return MEMS_ERROR;
  
  value &= ~LIS2DW12_ACC_DRDY_PULSED_MASK; 
  value |= newValue;
  
  if( !LIS2DW12_ACC_WriteReg(handle, LIS2DW12_ACC_ABS_INT_CFG, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name  : LIS2DW12_ACC_R_DataReady
  * Description    : Read DRDY_PULSED
  * Input          : Pointer to LIS2DW12_ACC_DRDY_PULSED_t
  * Output         : Status of DRDY_PULSED see LIS2DW12_ACC_DRDY_PULSED_t
  * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DW12_ACC_R_DataReady(void *handle,LIS2DW12_ACC_DRDY_PULSED_t *value)
{
  if( !LIS2DW12_ACC_ReadReg(handle, LIS2DW12_ACC_ABS_INT_CFG, (u8_t *)value , 1) )
    return MEMS_ERROR;
  
  *value &= LIS2DW12_ACC_DRDY_PULSED_MASK; //mask
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
  * Function Name    : SwapHighLowByte
  * Description    : Swap High/low byte in multiple byte values 
  *                     It works with minimum 2 byte for every dimension.
  *                     Example x,y,z with 2 byte for every dimension
  *
  * Input        : bufferToSwap -> buffer to swap 
  *                     numberOfByte -> the buffer length in byte
  *                     dimension -> number of dimension 
  *
  * Output      : bufferToSwap -> buffer swapped 
  * Return      : None
*******************************************************************************/
void LIS2DW12_ACC_SwapHighLowByte(void *handle,u8_t *bufferToSwap, u8_t numberOfByte, u8_t dimension)
{
  
  u8_t numberOfByteForDimension, i, j;
  u8_t tempValue[10];
  
  numberOfByteForDimension=numberOfByte/dimension;
  
  for (i=0; i<dimension;i++ )
  {
    for (j=0; j<numberOfByteForDimension;j++ )
      tempValue[j]=bufferToSwap[j+i*numberOfByteForDimension];
    for (j=0; j<numberOfByteForDimension;j++ )
      *(bufferToSwap+i*(numberOfByteForDimension)+j)=*(tempValue+(numberOfByteForDimension-1)-j);
  } 
}

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
