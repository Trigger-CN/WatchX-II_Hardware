/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
* File Name          : LSM330_GYRO_driver.c
* Author             : MEMS Application Team
* Version            : v1.1
* Date               : 20 December 2016   
* Description        : LSM330 Platform Independent Driver
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
#include "LSM330_GYRO_driver.h"
//#include "i2C_mems.h"                                           //[Example]

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/************** Generic Function  *******************/

/*******************************************************************************
* Function Name : LSM330_GYRO_ReadReg
* Description   : Generic Reading function. It must be fullfilled with either
*               : I2C or SPI reading functions
* Input         : Register Address
* Output        : Data Read
* Return			: None
*******************************************************************************/
status_t LSM330_GYRO_ReadReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len)
{
   
  //To be completed with either I2c or SPI writing function
  //i.e.: SPI_Mems_Write_Reg(Reg, Data);
  //I2Cx_Write(Bufp, LIS2DH_ACC_GYRO_I2C_ADDRESS, Reg, len);    //[Example]
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name : LSM330_GYRO_WriteReg
* Description   : Generic Writing function. It must be fullfilled with either
*					: I2C or SPI writing function
* Input         : Register Address, Data to be written
* Output			: None
* Return			: None
*******************************************************************************/
status_t LSM330_GYRO_WriteReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len)
{
    
  //To be completed with either I2c or SPI writing function
  //i.e.: SPI_Mems_Write_Reg(Reg, Data);
  //I2Cx_Read(Bufp, LIS2DH_ACC_GYRO_I2C_ADDRESS, Reg, len); //[Example]
  return MEMS_SUCCESS; 
}

/**************** Base Function  *******************/

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_WHO_AM_I_
* Description    : Read WHO_AM_I_BIT
* Input          : Pointer to u8_t
* Output         : Status of WHO_AM_I_BIT 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_WHO_AM_I_(void *handle, u8_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_WHO_AM_I_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_WHO_AM_I_BIT_MASK; //coerce	
  *value = *value >> LSM330_GYRO_WHO_AM_I_BIT_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_W_DataRate
* Description    : Write DR
* Input          : LSM330_GYRO_DR_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_GYRO_W_DataRate(void *handle, LSM330_GYRO_DR_t newValue)
{
  u8_t value;

  if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_GYRO_DR_MASK; 
  value |= newValue;
  
  if( !LSM330_GYRO_WriteReg(handle, LSM330_GYRO_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_DataRate
* Description    : Read DR
* Input          : Pointer to LSM330_GYRO_DR_t
* Output         : Status of DR see LSM330_GYRO_DR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_DataRate(void *handle, LSM330_GYRO_DR_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_DR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_W_FullScale
* Description    : Write FS
* Input          : LSM330_GYRO_FS_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_GYRO_W_FullScale(void *handle, LSM330_GYRO_FS_t newValue)
{
  u8_t value;

  if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_GYRO_FS_MASK; 
  value |= newValue;
  
  if( !LSM330_GYRO_WriteReg(handle, LSM330_GYRO_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_FullScale
* Description    : Read FS
* Input          : Pointer to LSM330_GYRO_FS_t
* Output         : Status of FS see LSM330_GYRO_FS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_FullScale(void *handle, LSM330_GYRO_FS_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG4, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_FS_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_W_BlockDataUpdate
* Description    : Write BDU
* Input          : LSM330_GYRO_BDU_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_GYRO_W_BlockDataUpdate(void *handle, LSM330_GYRO_BDU_t newValue)
{
  u8_t value;

  if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_GYRO_BDU_MASK; 
  value |= newValue;
  
  if( !LSM330_GYRO_WriteReg(handle, LSM330_GYRO_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_BlockDataUpdate
* Description    : Read BDU
* Input          : Pointer to LSM330_GYRO_BDU_t
* Output         : Status of BDU see LSM330_GYRO_BDU_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_BlockDataUpdate(void *handle, LSM330_GYRO_BDU_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG4, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_BDU_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : status_t LSM330_GYRO_Get_AngularRate(void *handle,  u8_t *buff)
* Description    : Read AngularRate output register
* Input          : pointer to [u8_t]
* Output         : AngularRate buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_Get_AngularRate(void *handle, u8_t *buff) 
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;
  
  numberOfByteForDimension=6/3;

  k=0;
  for (i=0; i<3;i++ ) 
  {
    for (j=0; j<numberOfByteForDimension;j++ )
    {	
      if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_OUT_X_L+k, &buff[k], 1))
        return MEMS_ERROR;
      k++;	
    }
  }

  return MEMS_SUCCESS; 
}

/**************** Advanced Function  *******************/

/*******************************************************************************
* Function Name  : LSM330_GYRO_W_AxisY
* Description    : Write YEN
* Input          : LSM330_GYRO_YEN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_GYRO_W_AxisY(void *handle, LSM330_GYRO_YEN_t newValue)
{
  u8_t value;

  if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_GYRO_YEN_MASK; 
  value |= newValue;
  
  if( !LSM330_GYRO_WriteReg(handle, LSM330_GYRO_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_AxisY
* Description    : Read YEN
* Input          : Pointer to LSM330_GYRO_YEN_t
* Output         : Status of YEN see LSM330_GYRO_YEN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_AxisY(void *handle, LSM330_GYRO_YEN_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_YEN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_W_AxisX
* Description    : Write XEN
* Input          : LSM330_GYRO_XEN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_GYRO_W_AxisX(void *handle, LSM330_GYRO_XEN_t newValue)
{
  u8_t value;

  if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_GYRO_XEN_MASK; 
  value |= newValue;
  
  if( !LSM330_GYRO_WriteReg(handle, LSM330_GYRO_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_AxisX
* Description    : Read XEN
* Input          : Pointer to LSM330_GYRO_XEN_t
* Output         : Status of XEN see LSM330_GYRO_XEN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_AxisX(void *handle, LSM330_GYRO_XEN_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_XEN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_W_AxisZ
* Description    : Write ZEN
* Input          : LSM330_GYRO_ZEN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_GYRO_W_AxisZ(void *handle, LSM330_GYRO_ZEN_t newValue)
{
  u8_t value;

  if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_GYRO_ZEN_MASK; 
  value |= newValue;
  
  if( !LSM330_GYRO_WriteReg(handle, LSM330_GYRO_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_AxisZ
* Description    : Read ZEN
* Input          : Pointer to LSM330_GYRO_ZEN_t
* Output         : Status of ZEN see LSM330_GYRO_ZEN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_AxisZ(void *handle, LSM330_GYRO_ZEN_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_ZEN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_W_SystemStatus
* Description    : Write PD
* Input          : LSM330_GYRO_PD_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_GYRO_W_SystemStatus(void *handle, LSM330_GYRO_PD_t newValue)
{
  u8_t value;

  if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_GYRO_PD_MASK; 
  value |= newValue;
  
  if( !LSM330_GYRO_WriteReg(handle, LSM330_GYRO_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_SystemStatus
* Description    : Read PD
* Input          : Pointer to LSM330_GYRO_PD_t
* Output         : Status of PD see LSM330_GYRO_PD_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_SystemStatus(void *handle, LSM330_GYRO_PD_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_PD_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_W_Bandwidth
* Description    : Write BW
* Input          : LSM330_GYRO_BW_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_GYRO_W_Bandwidth(void *handle, LSM330_GYRO_BW_t newValue)
{
  u8_t value;

  if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_GYRO_BW_MASK; 
  value |= newValue;
  
  if( !LSM330_GYRO_WriteReg(handle, LSM330_GYRO_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_Bandwidth
* Description    : Read BW
* Input          : Pointer to LSM330_GYRO_BW_t
* Output         : Status of BW see LSM330_GYRO_BW_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_Bandwidth(void *handle, LSM330_GYRO_BW_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_BW_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_W_HighPassFilterCutoffFreq
* Description    : Write HPCF
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_GYRO_W_HighPassFilterCutoffFreq(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM330_GYRO_HPCF_POSITION; //mask	
  newValue &= LSM330_GYRO_HPCF_MASK; //coerce
  
  if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_GYRO_HPCF_MASK; 
  value |= newValue;
  
  if( !LSM330_GYRO_WriteReg(handle, LSM330_GYRO_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_HighPassFilterCutoffFreq
* Description    : Read HPCF
* Input          : Pointer to u8_t
* Output         : Status of HPCF 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_HighPassFilterCutoffFreq(void *handle, u8_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_HPCF_MASK; //coerce	
  *value = *value >> LSM330_GYRO_HPCF_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_W_HighPassFilterMode
* Description    : Write HPM
* Input          : LSM330_GYRO_HPM_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_GYRO_W_HighPassFilterMode(void *handle, LSM330_GYRO_HPM_t newValue)
{
  u8_t value;

  if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_GYRO_HPM_MASK; 
  value |= newValue;
  
  if( !LSM330_GYRO_WriteReg(handle, LSM330_GYRO_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_HighPassFilterMode
* Description    : Read HPM
* Input          : Pointer to LSM330_GYRO_HPM_t
* Output         : Status of HPM see LSM330_GYRO_HPM_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_HighPassFilterMode(void *handle, LSM330_GYRO_HPM_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_HPM_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_W_FIFO_empty_INT2
* Description    : Write I2_EMPTY
* Input          : LSM330_GYRO_I2_EMPTY_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_GYRO_W_FIFO_empty_INT2(void *handle, LSM330_GYRO_I2_EMPTY_t newValue)
{
  u8_t value;

  if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_GYRO_I2_EMPTY_MASK; 
  value |= newValue;
  
  if( !LSM330_GYRO_WriteReg(handle, LSM330_GYRO_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_FIFO_empty_INT2
* Description    : Read I2_EMPTY
* Input          : Pointer to LSM330_GYRO_I2_EMPTY_t
* Output         : Status of I2_EMPTY see LSM330_GYRO_I2_EMPTY_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_FIFO_empty_INT2(void *handle, LSM330_GYRO_I2_EMPTY_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG3, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_I2_EMPTY_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_W_FIFO_overrun_INT2
* Description    : Write I2_ORUN
* Input          : LSM330_GYRO_I2_ORUN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_GYRO_W_FIFO_overrun_INT2(void *handle, LSM330_GYRO_I2_ORUN_t newValue)
{
  u8_t value;

  if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_GYRO_I2_ORUN_MASK; 
  value |= newValue;
  
  if( !LSM330_GYRO_WriteReg(handle, LSM330_GYRO_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_FIFO_overrun_INT2
* Description    : Read I2_ORUN
* Input          : Pointer to LSM330_GYRO_I2_ORUN_t
* Output         : Status of I2_ORUN see LSM330_GYRO_I2_ORUN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_FIFO_overrun_INT2(void *handle, LSM330_GYRO_I2_ORUN_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG3, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_I2_ORUN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_W_FIFO_watermark_INT2
* Description    : Write I2_WTM
* Input          : LSM330_GYRO_I2_WTM_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_GYRO_W_FIFO_watermark_INT2(void *handle, LSM330_GYRO_I2_WTM_t newValue)
{
  u8_t value;

  if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_GYRO_I2_WTM_MASK; 
  value |= newValue;
  
  if( !LSM330_GYRO_WriteReg(handle, LSM330_GYRO_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_FIFO_watermark_INT2
* Description    : Read I2_WTM
* Input          : Pointer to LSM330_GYRO_I2_WTM_t
* Output         : Status of I2_WTM see LSM330_GYRO_I2_WTM_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_FIFO_watermark_INT2(void *handle, LSM330_GYRO_I2_WTM_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG3, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_I2_WTM_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_W_DataReady_INT2
* Description    : Write I2_DRDY
* Input          : LSM330_GYRO_I2_DRDY_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_GYRO_W_DataReady_INT2(void *handle, LSM330_GYRO_I2_DRDY_t newValue)
{
  u8_t value;

  if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_GYRO_I2_DRDY_MASK; 
  value |= newValue;
  
  if( !LSM330_GYRO_WriteReg(handle, LSM330_GYRO_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_DataReady_INT2
* Description    : Read I2_DRDY
* Input          : Pointer to LSM330_GYRO_I2_DRDY_t
* Output         : Status of I2_DRDY see LSM330_GYRO_I2_DRDY_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_DataReady_INT2(void *handle, LSM330_GYRO_I2_DRDY_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG3, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_I2_DRDY_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_W_PinConfiguration
* Description    : Write PP_OD
* Input          : LSM330_GYRO_PP_OD_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_GYRO_W_PinConfiguration(void *handle, LSM330_GYRO_PP_OD_t newValue)
{
  u8_t value;

  if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_GYRO_PP_OD_MASK; 
  value |= newValue;
  
  if( !LSM330_GYRO_WriteReg(handle, LSM330_GYRO_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_PinConfiguration
* Description    : Read PP_OD
* Input          : Pointer to LSM330_GYRO_PP_OD_t
* Output         : Status of PP_OD see LSM330_GYRO_PP_OD_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_PinConfiguration(void *handle, LSM330_GYRO_PP_OD_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG3, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_PP_OD_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_W_InterruptActive
* Description    : Write H_LACTIVE
* Input          : LSM330_GYRO_H_LACTIVE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_GYRO_W_InterruptActive(void *handle, LSM330_GYRO_H_LACTIVE_t newValue)
{
  u8_t value;

  if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_GYRO_H_LACTIVE_MASK; 
  value |= newValue;
  
  if( !LSM330_GYRO_WriteReg(handle, LSM330_GYRO_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_InterruptActive
* Description    : Read H_LACTIVE
* Input          : Pointer to LSM330_GYRO_H_LACTIVE_t
* Output         : Status of H_LACTIVE see LSM330_GYRO_H_LACTIVE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_InterruptActive(void *handle, LSM330_GYRO_H_LACTIVE_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG3, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_H_LACTIVE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_W_BootStatusOnINT1
* Description    : Write I1_BOOT
* Input          : LSM330_GYRO_I1_BOOT_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_GYRO_W_BootStatusOnINT1(void *handle, LSM330_GYRO_I1_BOOT_t newValue)
{
  u8_t value;

  if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_GYRO_I1_BOOT_MASK; 
  value |= newValue;
  
  if( !LSM330_GYRO_WriteReg(handle, LSM330_GYRO_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_BootStatusOnINT1
* Description    : Read I1_BOOT
* Input          : Pointer to LSM330_GYRO_I1_BOOT_t
* Output         : Status of I1_BOOT see LSM330_GYRO_I1_BOOT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_BootStatusOnINT1(void *handle, LSM330_GYRO_I1_BOOT_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG3, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_I1_BOOT_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_W_EnableIntOnINT1
* Description    : Write I1_INT1
* Input          : LSM330_GYRO_I1_INT1_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_GYRO_W_EnableIntOnINT1(void *handle, LSM330_GYRO_I1_INT1_t newValue)
{
  u8_t value;

  if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_GYRO_I1_INT1_MASK; 
  value |= newValue;
  
  if( !LSM330_GYRO_WriteReg(handle, LSM330_GYRO_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_EnableIntOnINT1
* Description    : Read I1_INT1
* Input          : Pointer to LSM330_GYRO_I1_INT1_t
* Output         : Status of I1_INT1 see LSM330_GYRO_I1_INT1_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_EnableIntOnINT1(void *handle, LSM330_GYRO_I1_INT1_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG3, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_I1_INT1_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_W_SPI_Configuration
* Description    : Write SIM
* Input          : LSM330_GYRO_SIM_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_GYRO_W_SPI_Configuration(void *handle, LSM330_GYRO_SIM_t newValue)
{
  u8_t value;

  if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_GYRO_SIM_MASK; 
  value |= newValue;
  
  if( !LSM330_GYRO_WriteReg(handle, LSM330_GYRO_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_SPI_Configuration
* Description    : Read SIM
* Input          : Pointer to LSM330_GYRO_SIM_t
* Output         : Status of SIM see LSM330_GYRO_SIM_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_SPI_Configuration(void *handle, LSM330_GYRO_SIM_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG4, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_SIM_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_W_BigLittleEndianSelection
* Description    : Write BLE
* Input          : LSM330_GYRO_BLE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_GYRO_W_BigLittleEndianSelection(void *handle, LSM330_GYRO_BLE_t newValue)
{
  u8_t value;

  if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_GYRO_BLE_MASK; 
  value |= newValue;
  
  if( !LSM330_GYRO_WriteReg(handle, LSM330_GYRO_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_BigLittleEndianSelection
* Description    : Read BLE
* Input          : Pointer to LSM330_GYRO_BLE_t
* Output         : Status of BLE see LSM330_GYRO_BLE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_BigLittleEndianSelection(void *handle, LSM330_GYRO_BLE_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG4, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_BLE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_W_OutputDataSelection
* Description    : Write OUT_SEL
* Input          : LSM330_GYRO_OUT_SEL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_GYRO_W_OutputDataSelection(void *handle, LSM330_GYRO_OUT_SEL_t newValue)
{
  u8_t value;

  if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG5, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_GYRO_OUT_SEL_MASK; 
  value |= newValue;
  
  if( !LSM330_GYRO_WriteReg(handle, LSM330_GYRO_CTRL_REG5, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_OutputDataSelection
* Description    : Read OUT_SEL
* Input          : Pointer to LSM330_GYRO_OUT_SEL_t
* Output         : Status of OUT_SEL see LSM330_GYRO_OUT_SEL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_OutputDataSelection(void *handle, LSM330_GYRO_OUT_SEL_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG5, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_OUT_SEL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_W_InterruptDataSelection
* Description    : Write INT1_SEL
* Input          : LSM330_GYRO_INT1_SEL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_GYRO_W_InterruptDataSelection(void *handle, LSM330_GYRO_INT1_SEL_t newValue)
{
  u8_t value;

  if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG5, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_GYRO_INT1_SEL_MASK; 
  value |= newValue;
  
  if( !LSM330_GYRO_WriteReg(handle, LSM330_GYRO_CTRL_REG5, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_InterruptDataSelection
* Description    : Read INT1_SEL
* Input          : Pointer to LSM330_GYRO_INT1_SEL_t
* Output         : Status of INT1_SEL see LSM330_GYRO_INT1_SEL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_InterruptDataSelection(void *handle, LSM330_GYRO_INT1_SEL_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG5, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_INT1_SEL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_W_HighPassFilter
* Description    : Write HPEN
* Input          : LSM330_GYRO_HPEN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_GYRO_W_HighPassFilter(void *handle, LSM330_GYRO_HPEN_t newValue)
{
  u8_t value;

  if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG5, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_GYRO_HPEN_MASK; 
  value |= newValue;
  
  if( !LSM330_GYRO_WriteReg(handle, LSM330_GYRO_CTRL_REG5, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_HighPassFilter
* Description    : Read HPEN
* Input          : Pointer to LSM330_GYRO_HPEN_t
* Output         : Status of HPEN see LSM330_GYRO_HPEN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_HighPassFilter(void *handle, LSM330_GYRO_HPEN_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG5, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_HPEN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_W_OpenFIFO
* Description    : Write FIFO_EN
* Input          : LSM330_GYRO_FIFO_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_GYRO_W_OpenFIFO(void *handle, LSM330_GYRO_FIFO_EN_t newValue)
{
  u8_t value;

  if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG5, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_GYRO_FIFO_EN_MASK; 
  value |= newValue;
  
  if( !LSM330_GYRO_WriteReg(handle, LSM330_GYRO_CTRL_REG5, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_OpenFIFO
* Description    : Read FIFO_EN
* Input          : Pointer to LSM330_GYRO_FIFO_EN_t
* Output         : Status of FIFO_EN see LSM330_GYRO_FIFO_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_OpenFIFO(void *handle, LSM330_GYRO_FIFO_EN_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG5, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_FIFO_EN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_W_RebootMemory
* Description    : Write BOOT
* Input          : LSM330_GYRO_BOOT_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_GYRO_W_RebootMemory(void *handle, LSM330_GYRO_BOOT_t newValue)
{
  u8_t value;

  if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG5, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_GYRO_BOOT_MASK; 
  value |= newValue;
  
  if( !LSM330_GYRO_WriteReg(handle, LSM330_GYRO_CTRL_REG5, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_RebootMemory
* Description    : Read BOOT
* Input          : Pointer to LSM330_GYRO_BOOT_t
* Output         : Status of BOOT see LSM330_GYRO_BOOT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_RebootMemory(void *handle, LSM330_GYRO_BOOT_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_CTRL_REG5, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_BOOT_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_Temperature
* Description    : Read OUT_TEMP_BIT
* Input          : Pointer to u8_t
* Output         : Status of OUT_TEMP_BIT 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_Temperature(void *handle, u8_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_OUT_TEMP, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_OUT_TEMP_BIT_MASK; //coerce	
  *value = *value >> LSM330_GYRO_OUT_TEMP_BIT_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_NewDataX
* Description    : Read XDA
* Input          : Pointer to LSM330_GYRO_XDA_t
* Output         : Status of XDA see LSM330_GYRO_XDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_NewDataX(void *handle, LSM330_GYRO_XDA_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_XDA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_NewDataY
* Description    : Read YDA
* Input          : Pointer to LSM330_GYRO_YDA_t
* Output         : Status of YDA see LSM330_GYRO_YDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_NewDataY(void *handle, LSM330_GYRO_YDA_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_YDA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_NewDataZ
* Description    : Read ZDA
* Input          : Pointer to LSM330_GYRO_ZDA_t
* Output         : Status of ZDA see LSM330_GYRO_ZDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_NewDataZ(void *handle, LSM330_GYRO_ZDA_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_ZDA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_NewDataXYZ
* Description    : Read ZYXDA
* Input          : Pointer to LSM330_GYRO_ZYXDA_t
* Output         : Status of ZYXDA see LSM330_GYRO_ZYXDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_NewDataXYZ(void *handle, LSM330_GYRO_ZYXDA_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_ZYXDA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_DataOverrunX
* Description    : Read XOR
* Input          : Pointer to LSM330_GYRO_XOR_t
* Output         : Status of XOR see LSM330_GYRO_XOR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_DataOverrunX(void *handle, LSM330_GYRO_XOR_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_XOR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_DataOverrunY
* Description    : Read YOR
* Input          : Pointer to LSM330_GYRO_YOR_t
* Output         : Status of YOR see LSM330_GYRO_YOR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_DataOverrunY(void *handle, LSM330_GYRO_YOR_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_YOR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_DataOverrunZ
* Description    : Read ZOR
* Input          : Pointer to LSM330_GYRO_ZOR_t
* Output         : Status of ZOR see LSM330_GYRO_ZOR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_DataOverrunZ(void *handle, LSM330_GYRO_ZOR_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_ZOR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_DataOverrunXYZ
* Description    : Read ZYXOR
* Input          : Pointer to LSM330_GYRO_ZYXOR_t
* Output         : Status of ZYXOR see LSM330_GYRO_ZYXOR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_DataOverrunXYZ(void *handle, LSM330_GYRO_ZYXOR_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_ZYXOR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_W_FIFO_Watermark
* Description    : Write WTM
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_GYRO_W_FIFO_Watermark(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM330_GYRO_WTM_POSITION; //mask	
  newValue &= LSM330_GYRO_WTM_MASK; //coerce
  
  if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_FIFO_CTRL_REG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_GYRO_WTM_MASK; 
  value |= newValue;
  
  if( !LSM330_GYRO_WriteReg(handle, LSM330_GYRO_FIFO_CTRL_REG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_FIFO_Watermark
* Description    : Read WTM
* Input          : Pointer to u8_t
* Output         : Status of WTM 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_FIFO_Watermark(void *handle, u8_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_FIFO_CTRL_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_WTM_MASK; //coerce	
  *value = *value >> LSM330_GYRO_WTM_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_W_FIFO_Mode
* Description    : Write FM
* Input          : LSM330_GYRO_FM_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_GYRO_W_FIFO_Mode(void *handle, LSM330_GYRO_FM_t newValue)
{
  u8_t value;

  if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_FIFO_CTRL_REG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_GYRO_FM_MASK; 
  value |= newValue;
  
  if( !LSM330_GYRO_WriteReg(handle, LSM330_GYRO_FIFO_CTRL_REG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_FIFO_Mode
* Description    : Read FM
* Input          : Pointer to LSM330_GYRO_FM_t
* Output         : Status of FM see LSM330_GYRO_FM_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_FIFO_Mode(void *handle, LSM330_GYRO_FM_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_FIFO_CTRL_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_FM_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_FIFO_EmptyFlag
* Description    : Read EMPTY
* Input          : Pointer to LSM330_GYRO_EMPTY_t
* Output         : Status of EMPTY see LSM330_GYRO_EMPTY_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_FIFO_EmptyFlag(void *handle, LSM330_GYRO_EMPTY_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_FIFO_SRC_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_EMPTY_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_FIFO_OverrunFlag
* Description    : Read OVRN
* Input          : Pointer to LSM330_GYRO_OVRN_t
* Output         : Status of OVRN see LSM330_GYRO_OVRN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_FIFO_OverrunFlag(void *handle, LSM330_GYRO_OVRN_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_FIFO_SRC_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_OVRN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_FIFO_WatermarkFlag
* Description    : Read WTM
* Input          : Pointer to LSM330_GYRO_WTM_t
* Output         : Status of WTM see LSM330_GYRO_WTM_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_FIFO_WatermarkFlag(void *handle, LSM330_GYRO_WTM_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_FIFO_SRC_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_WTM_FLAG_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_W_LowLevelInterruptOnX
* Description    : Write XLIE
* Input          : LSM330_GYRO_XLIE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_GYRO_W_LowLevelInterruptOnX(void *handle, LSM330_GYRO_XLIE_t newValue)
{
  u8_t value;

  if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_GYRO_XLIE_MASK; 
  value |= newValue;
  
  if( !LSM330_GYRO_WriteReg(handle, LSM330_GYRO_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_LowLevelInterruptOnX
* Description    : Read XLIE
* Input          : Pointer to LSM330_GYRO_XLIE_t
* Output         : Status of XLIE see LSM330_GYRO_XLIE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_LowLevelInterruptOnX(void *handle, LSM330_GYRO_XLIE_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_INT1_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_XLIE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_W_HighLevelInterruptOnX
* Description    : Write XHIE
* Input          : LSM330_GYRO_XHIE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_GYRO_W_HighLevelInterruptOnX(void *handle, LSM330_GYRO_XHIE_t newValue)
{
  u8_t value;

  if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_GYRO_XHIE_MASK; 
  value |= newValue;
  
  if( !LSM330_GYRO_WriteReg(handle, LSM330_GYRO_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_HighLevelInterruptOnX
* Description    : Read XHIE
* Input          : Pointer to LSM330_GYRO_XHIE_t
* Output         : Status of XHIE see LSM330_GYRO_XHIE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_HighLevelInterruptOnX(void *handle, LSM330_GYRO_XHIE_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_INT1_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_XHIE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_W_LowLevelInterruptOnY
* Description    : Write YLIE
* Input          : LSM330_GYRO_YLIE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_GYRO_W_LowLevelInterruptOnY(void *handle, LSM330_GYRO_YLIE_t newValue)
{
  u8_t value;

  if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_GYRO_YLIE_MASK; 
  value |= newValue;
  
  if( !LSM330_GYRO_WriteReg(handle, LSM330_GYRO_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_LowLevelInterruptOnY
* Description    : Read YLIE
* Input          : Pointer to LSM330_GYRO_YLIE_t
* Output         : Status of YLIE see LSM330_GYRO_YLIE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_LowLevelInterruptOnY(void *handle, LSM330_GYRO_YLIE_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_INT1_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_YLIE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_W_HighLevelInterruptOnY
* Description    : Write YHIE
* Input          : LSM330_GYRO_YHIE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_GYRO_W_HighLevelInterruptOnY(void *handle, LSM330_GYRO_YHIE_t newValue)
{
  u8_t value;

  if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_GYRO_YHIE_MASK; 
  value |= newValue;
  
  if( !LSM330_GYRO_WriteReg(handle, LSM330_GYRO_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_HighLevelInterruptOnY
* Description    : Read YHIE
* Input          : Pointer to LSM330_GYRO_YHIE_t
* Output         : Status of YHIE see LSM330_GYRO_YHIE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_HighLevelInterruptOnY(void *handle, LSM330_GYRO_YHIE_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_INT1_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_YHIE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_W_LowLevelInterruptOnZ
* Description    : Write ZLIE
* Input          : LSM330_GYRO_ZLIE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_GYRO_W_LowLevelInterruptOnZ(void *handle, LSM330_GYRO_ZLIE_t newValue)
{
  u8_t value;

  if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_GYRO_ZLIE_MASK; 
  value |= newValue;
  
  if( !LSM330_GYRO_WriteReg(handle, LSM330_GYRO_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_LowLevelInterruptOnZ
* Description    : Read ZLIE
* Input          : Pointer to LSM330_GYRO_ZLIE_t
* Output         : Status of ZLIE see LSM330_GYRO_ZLIE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_LowLevelInterruptOnZ(void *handle, LSM330_GYRO_ZLIE_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_INT1_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_ZLIE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_W_HighLevelInterruptOnZ
* Description    : Write ZHIE
* Input          : LSM330_GYRO_ZHIE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_GYRO_W_HighLevelInterruptOnZ(void *handle, LSM330_GYRO_ZHIE_t newValue)
{
  u8_t value;

  if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_GYRO_ZHIE_MASK; 
  value |= newValue;
  
  if( !LSM330_GYRO_WriteReg(handle, LSM330_GYRO_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_HighLevelInterruptOnZ
* Description    : Read ZHIE
* Input          : Pointer to LSM330_GYRO_ZHIE_t
* Output         : Status of ZHIE see LSM330_GYRO_ZHIE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_HighLevelInterruptOnZ(void *handle, LSM330_GYRO_ZHIE_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_INT1_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_ZHIE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_W_InterruptRequestMode
* Description    : Write LIR
* Input          : LSM330_GYRO_LIR_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_GYRO_W_InterruptRequestMode(void *handle, LSM330_GYRO_LIR_t newValue)
{
  u8_t value;

  if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_GYRO_LIR_MASK; 
  value |= newValue;
  
  if( !LSM330_GYRO_WriteReg(handle, LSM330_GYRO_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_InterruptRequestMode
* Description    : Read LIR
* Input          : Pointer to LSM330_GYRO_LIR_t
* Output         : Status of LIR see LSM330_GYRO_LIR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_InterruptRequestMode(void *handle, LSM330_GYRO_LIR_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_INT1_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_LIR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_W_InterruptCombination
* Description    : Write AND_OR
* Input          : LSM330_GYRO_AND_OR_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_GYRO_W_InterruptCombination(void *handle, LSM330_GYRO_AND_OR_t newValue)
{
  u8_t value;

  if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_GYRO_AND_OR_MASK; 
  value |= newValue;
  
  if( !LSM330_GYRO_WriteReg(handle, LSM330_GYRO_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_InterruptCombination
* Description    : Read AND_OR
* Input          : Pointer to LSM330_GYRO_AND_OR_t
* Output         : Status of AND_OR see LSM330_GYRO_AND_OR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_InterruptCombination(void *handle, LSM330_GYRO_AND_OR_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_INT1_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_AND_OR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_InterruptLowFlagX
* Description    : Read XL
* Input          : Pointer to LSM330_GYRO_XL_t
* Output         : Status of XL see LSM330_GYRO_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_InterruptLowFlagX(void *handle, LSM330_GYRO_XL_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_INT1_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_InterruptHighFlagX
* Description    : Read XH
* Input          : Pointer to LSM330_GYRO_XH_t
* Output         : Status of XH see LSM330_GYRO_XH_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_InterruptHighFlagX(void *handle, LSM330_GYRO_XH_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_INT1_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_XH_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_InterruptLowFlagY
* Description    : Read YL
* Input          : Pointer to LSM330_GYRO_YL_t
* Output         : Status of YL see LSM330_GYRO_YL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_InterruptLowFlagY(void *handle, LSM330_GYRO_YL_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_INT1_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_YL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_InterruptHighFlagY
* Description    : Read YH
* Input          : Pointer to LSM330_GYRO_YH_t
* Output         : Status of YH see LSM330_GYRO_YH_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_InterruptHighFlagY(void *handle, LSM330_GYRO_YH_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_INT1_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_YH_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_InterruptLowFlagZ
* Description    : Read ZL
* Input          : Pointer to LSM330_GYRO_ZL_t
* Output         : Status of ZL see LSM330_GYRO_ZL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_InterruptLowFlagZ(void *handle, LSM330_GYRO_ZL_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_INT1_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_ZL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_InterruptHighFlagZ
* Description    : Read ZH
* Input          : Pointer to LSM330_GYRO_ZH_t
* Output         : Status of ZH see LSM330_GYRO_ZH_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_InterruptHighFlagZ(void *handle, LSM330_GYRO_ZH_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_INT1_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_ZH_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_InterruptActiveFlag
* Description    : Read IA
* Input          : Pointer to LSM330_GYRO_IA_t
* Output         : Status of IA see LSM330_GYRO_IA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_InterruptActiveFlag(void *handle, LSM330_GYRO_IA_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_INT1_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_IA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_W_D_bits
* Description    : Write D
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_GYRO_W_Duration(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM330_GYRO_D_POSITION; //mask	
  newValue &= LSM330_GYRO_D_MASK; //coerce
  
  if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_INT1_DURATION, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_GYRO_D_MASK; 
  value |= newValue;
  
  if( !LSM330_GYRO_WriteReg(handle, LSM330_GYRO_INT1_DURATION, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_D_bits
* Description    : Read D
* Input          : Pointer to u8_t
* Output         : Status of D 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_Duration(void *handle, u8_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_INT1_DURATION, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_D_MASK; //coerce	
  *value = *value >> LSM330_GYRO_D_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_DataStoredFIFO
* Description    : Read FSS
* Input          : Pointer to u8_t
* Output         : Status of FSS 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_DataStoredFIFO(void *handle, u8_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_FIFO_SRC_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_FSS_MASK; //coerce	
  *value = *value >> LSM330_GYRO_FSS_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_W_Wait
* Description    : Write WAIT
* Input          : LSM330_GYRO_WAIT_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_GYRO_W_Wait(void *handle, LSM330_GYRO_WAIT_t newValue)
{
  u8_t value;

  if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_INT1_DURATION, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_GYRO_WAIT_MASK; 
  value |= newValue;
  
  if( !LSM330_GYRO_WriteReg(handle, LSM330_GYRO_INT1_DURATION, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_GYRO_R_Wait
* Description    : Read WAIT
* Input          : Pointer to LSM330_GYRO_WAIT_t
* Output         : Status of WAIT see LSM330_GYRO_WAIT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_R_Wait(void *handle, LSM330_GYRO_WAIT_t *value)
{
 if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_INT1_DURATION, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_GYRO_WAIT_MASK; //mask

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : status_t LSM330_GYRO_Set_InterruptThreshold(void *handle,  u8_t *buff) 
* Description    : Set InterruptThreshold data row
* Input          : pointer to [u8_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_Set_InterruptThreshold(void *handle,  u8_t *buff) 
{
  u8_t  i;
  u8_t numberOfByteForDimension;
  
  numberOfByteForDimension=6/3;
  LSM330_GYRO_SwapHighLowByte(buff, 6, numberOfByteForDimension);

  for (i=0; i<6;i++ ) 
  {
    if( !LSM330_GYRO_WriteReg(handle, LSM330_GYRO_INT1_TSH_XH+i,  &buff[i], 1) )
      return MEMS_ERROR;
  }
  return MEMS_SUCCESS;  
}

/*******************************************************************************
* Function Name  : status_t LSM330_GYRO_Get_InterruptThreshold(void *handle,  u8_t *buff)
* Description    : Read InterruptThreshold output register
* Input          : pointer to [u8_t]
* Output         : InterruptThreshold buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_GYRO_Get_InterruptThreshold(void *handle,  u8_t *buff) 
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;
  
  numberOfByteForDimension=6/3;

  k=0;
  for (i=0; i<3;i++ ) 
  {
    for (j=0; j<numberOfByteForDimension;j++ )
    {	
      if( !LSM330_GYRO_ReadReg(handle, LSM330_GYRO_INT1_TSH_XH+k, &buff[k], 1))
        return MEMS_ERROR;
      k++;	
    }
  }
  
  LSM330_GYRO_SwapHighLowByte(buff, 6, numberOfByteForDimension);

  return MEMS_SUCCESS; 
}

/************** Utility  *******************/

/*******************************************************************************
* Function Name		: SwapHighLowByte
* Description		: Swap High/low byte in multiple byte values 
*                     It works with minimum 2 byte for every dimension.
*                     Example x,y,z with 2 byte for every dimension
*
* Input				: bufferToSwap -> buffer to swap 
*                     numberOfByte -> the buffer length in byte
*                     dimension -> number of dimension 
*
* Output			: bufferToSwap -> buffer swapped 
* Return			: None
*******************************************************************************/
void LSM330_GYRO_SwapHighLowByte(u8_t *bufferToSwap, u8_t numberOfByte, u8_t dimension)
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

