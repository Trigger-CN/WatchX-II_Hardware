/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
* File Name          : LSM330_ACC_driver.c
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
#include "LSM330_ACC_driver.h"
//#include "i2C_mems.h"                                 //[Example]

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/************** Generic Function  *******************/

/*******************************************************************************
* Function Name : LSM330_ACC_ReadReg
* Description   : Generic Reading function. It must be fullfilled with either
*               : I2C or SPI reading functions
* Input         : Register Address
* Output        : Data Read
* Return			: None
*******************************************************************************/
status_t LSM330_ACC_ReadReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len)
{
   
  //To be completed with either I2c or SPI writing function
  //i.e.: SPI_Mems_Write_Reg(Reg, Data);
  //I2Cx_Write(Bufp, LIS2DH_ACC_GYRO_I2C_ADDRESS, Reg, len);    //[Example]
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name : LSM330_ACC_WriteReg
* Description   : Generic Writing function. It must be fullfilled with either
*					: I2C or SPI writing function
* Input         : Register Address, Data to be written
* Output			: None
* Return			: None
*******************************************************************************/
status_t LSM330_ACC_WriteReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len)
{
    
  //To be completed with either I2c or SPI writing function
  //i.e.: SPI_Mems_Write_Reg(Reg, Data);
  //I2Cx_Read(Bufp, LIS2DH_ACC_GYRO_I2C_ADDRESS, Reg, len); //[Example]
  return MEMS_SUCCESS; 
}

/**************** Base Function  *******************/

/*******************************************************************************
* Function Name  : LSM330_ACC_R_WHO_AM_I_
* Description    : Read WHO_AM_I_BYTE
* Input          : Pointer to u8_t
* Output         : Status of WHO_AM_I_BYTE 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_WHO_AM_I_(void *handle, u8_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_WHO_AM_I_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_WHO_AM_I_BYTE_MASK; //coerce	
  *value = *value >> LSM330_ACC_WHO_AM_I_BYTE_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_OutputDataRate
* Description    : Write ODR
* Input          : LSM330_ACC_ODR_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_OutputDataRate(void *handle, LSM330_ACC_ODR_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL4, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_ODR_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_CTRL4, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_OutputDataRate
* Description    : Read ODR
* Input          : Pointer to LSM330_ACC_ODR_t
* Output         : Status of ODR see LSM330_ACC_ODR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_OutputDataRate(void *handle, LSM330_ACC_ODR_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL4, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_ODR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_FullScale
* Description    : Write FSCALE
* Input          : LSM330_ACC_FSCALE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_FullScale(void *handle, LSM330_ACC_FSCALE_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL5, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_FSCALE_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_CTRL5, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_FullScale
* Description    : Read FSCALE
* Input          : Pointer to LSM330_ACC_FSCALE_t
* Output         : Status of FSCALE see LSM330_ACC_FSCALE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_FullScale(void *handle, LSM330_ACC_FSCALE_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL5, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_FSCALE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_BlockDataUpdate
* Description    : Write BDU
* Input          : LSM330_ACC_BDU_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_BlockDataUpdate(void *handle, LSM330_ACC_BDU_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL4, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_BDU_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_CTRL4, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_BlockDataUpdate
* Description    : Read BDU
* Input          : Pointer to LSM330_ACC_BDU_t
* Output         : Status of BDU see LSM330_ACC_BDU_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_BlockDataUpdate(void *handle, LSM330_ACC_BDU_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL4, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_BDU_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : status_t LSM330_ACC_Get_Acceleration(void *handle, u8_t *buff)
* Description    : Read Acceleration output register
* Input          : pointer to [u8_t]
* Output         : Acceleration buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_Get_Acceleration(void *handle, u8_t *buff) 
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;
  
  numberOfByteForDimension=6/3;

  k=0;
  for (i=0; i<3;i++ ) 
  {
    for (j=0; j<numberOfByteForDimension;j++ )
    {	
      if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_OUT_X_L+k, &buff[k], 1))
        return MEMS_ERROR;
      k++;	
    }
  }

  return MEMS_SUCCESS; 
}

/**************** Advanced Function  *******************/

/*******************************************************************************
* Function Name  : LSM330_ACC_R_RegisterInfo1
* Description    : Read INFO1_BYTE
* Input          : Pointer to u8_t
* Output         : Status of INFO1_BYTE 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_RegisterInfo1(void *handle, u8_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_INFO1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_INFO1_BYTE_MASK; //coerce	
  *value = *value >> LSM330_ACC_INFO1_BYTE_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_RegisterInfo2
* Description    : Read INFO2_BYTE
* Input          : Pointer to u8_t
* Output         : Status of INFO2_BYTE 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_RegisterInfo2(void *handle, u8_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_INFO2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_INFO2_BYTE_MASK; //coerce	
  *value = *value >> LSM330_ACC_INFO2_BYTE_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_OffsetCorrectionX
* Description    : Write OFF_X_BITS
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_OffsetCorrectionX(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM330_ACC_OFF_X_BITS_POSITION; //mask	
  newValue &= LSM330_ACC_OFF_X_BITS_MASK; //coerce
  
  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_OFF_X, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_OFF_X_BITS_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_OFF_X, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_OffsetCorrectionX
* Description    : Read OFF_X_BITS
* Input          : Pointer to u8_t
* Output         : Status of OFF_X_BITS 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_OffsetCorrectionX(void *handle, u8_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_OFF_X, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_OFF_X_BITS_MASK; //coerce	
  *value = *value >> LSM330_ACC_OFF_X_BITS_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_OffsetCorrectionY
* Description    : Write OFF_Y_BITS
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_OffsetCorrectionY(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM330_ACC_OFF_Y_BITS_POSITION; //mask	
  newValue &= LSM330_ACC_OFF_Y_BITS_MASK; //coerce
  
  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_OFF_Y, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_OFF_Y_BITS_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_OFF_Y, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_OffsetCorrectionY
* Description    : Read OFF_Y_BITS
* Input          : Pointer to u8_t
* Output         : Status of OFF_Y_BITS 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_OffsetCorrectionY(void *handle, u8_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_OFF_Y, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_OFF_Y_BITS_MASK; //coerce	
  *value = *value >> LSM330_ACC_OFF_Y_BITS_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_OffsetCorrectionZ
* Description    : Write OFF_Z_BITS
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_OffsetCorrectionZ(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM330_ACC_OFF_Z_BITS_POSITION; //mask	
  newValue &= LSM330_ACC_OFF_Z_BITS_MASK; //coerce
  
  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_OFF_Z, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_OFF_Z_BITS_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_OFF_Z, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_OffsetCorrectionZ
* Description    : Read OFF_Z_BITS
* Input          : Pointer to u8_t
* Output         : Status of OFF_Z_BITS 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_OffsetCorrectionZ(void *handle, u8_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_OFF_Z, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_OFF_Z_BITS_MASK; //coerce	
  *value = *value >> LSM330_ACC_OFF_Z_BITS_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_ConstantShiftX
* Description    : Write CS_X_BITS
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_ConstantShiftX(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM330_ACC_CS_X_BITS_POSITION; //mask	
  newValue &= LSM330_ACC_CS_X_BITS_MASK; //coerce
  
  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CS_X, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_CS_X_BITS_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_CS_X, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_ConstantShiftX
* Description    : Read CS_X_BITS
* Input          : Pointer to u8_t
* Output         : Status of CS_X_BITS 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_ConstantShiftX(void *handle, u8_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CS_X, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_CS_X_BITS_MASK; //coerce	
  *value = *value >> LSM330_ACC_CS_X_BITS_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_ConstantShiftY
* Description    : Write CS_Y_BITS
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_ConstantShiftY(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM330_ACC_CS_Y_BITS_POSITION; //mask	
  newValue &= LSM330_ACC_CS_Y_BITS_MASK; //coerce
  
  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CS_Y, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_CS_Y_BITS_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_CS_Y, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_ConstantShiftY
* Description    : Read CS_Y_BITS
* Input          : Pointer to u8_t
* Output         : Status of CS_Y_BITS 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_ConstantShiftY(void *handle, u8_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CS_Y, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_CS_Y_BITS_MASK; //coerce	
  *value = *value >> LSM330_ACC_CS_Y_BITS_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_ConstantShiftZ
* Description    : Write CS_Z_BITS
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_ConstantShiftZ(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM330_ACC_CS_Z_BITS_POSITION; //mask	
  newValue &= LSM330_ACC_CS_Z_BITS_MASK; //coerce
  
  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CS_Z, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_CS_Z_BITS_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_CS_Z, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_ConstantShiftZ
* Description    : Read CS_Z_BITS
* Input          : Pointer to u8_t
* Output         : Status of CS_Z_BITS 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_ConstantShiftZ(void *handle, u8_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CS_Z, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_CS_Z_BITS_MASK; //coerce	
  *value = *value >> LSM330_ACC_CS_Z_BITS_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_DataReadyFlag
* Description    : Read DRDY
* Input          : Pointer to LSM330_ACC_DRDY_t
* Output         : Status of DRDY see LSM330_ACC_DRDY_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_DataReadyFlag(void *handle, LSM330_ACC_DRDY_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_STAT, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_DRDY_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_DataOverrunFlag
* Description    : Read DOR
* Input          : Pointer to LSM330_ACC_DOR_t
* Output         : Status of DOR see LSM330_ACC_DOR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_DataOverrunFlag(void *handle, LSM330_ACC_DOR_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_STAT, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_DOR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM2_InterruptFlag
* Description    : Read INT_SM2
* Input          : Pointer to LSM330_ACC_INT_SM2_t
* Output         : Status of INT_SM2 see LSM330_ACC_INT_SM2_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM2_InterruptFlag(void *handle, LSM330_ACC_INT_SM2_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_STAT, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_INT_SM2_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM1_InterruptFlag
* Description    : Read INT_SM1
* Input          : Pointer to LSM330_ACC_INT_SM1_t
* Output         : Status of INT_SM1 see LSM330_ACC_INT_SM1_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM1_InterruptFlag(void *handle, LSM330_ACC_INT_SM1_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_STAT, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_INT_SM1_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM2_WaitFlag
* Description    : Read SYNC2
* Input          : Pointer to LSM330_ACC_SYNC2_t
* Output         : Status of SYNC2 see LSM330_ACC_SYNC2_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM2_WaitFlag(void *handle, LSM330_ACC_SYNC2_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_STAT, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SYNC2_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM1_WaitFlag
* Description    : Read SYNC1
* Input          : Pointer to LSM330_ACC_SYNC1_t
* Output         : Status of SYNC1 see LSM330_ACC_SYNC1_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM1_WaitFlag(void *handle, LSM330_ACC_SYNC1_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_STAT, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SYNC1_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_ExternalEventFlag
* Description    : Read SYNCW
* Input          : Pointer to LSM330_ACC_SYNCW_t
* Output         : Status of SYNCW see LSM330_ACC_SYNCW_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_ExternalEventFlag(void *handle, LSM330_ACC_SYNCW_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_STAT, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SYNCW_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_LongCounterInterruptFlag
* Description    : Read LONG
* Input          : Pointer to LSM330_ACC_LONG_t
* Output         : Status of LONG see LSM330_ACC_LONG_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_LongCounterInterruptFlag(void *handle, LSM330_ACC_LONG_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_STAT, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_LONG_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_PeakSM1
* Description    : Read PEAK1_BIT
* Input          : Pointer to u8_t
* Output         : Status of PEAK1_BIT 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_PeakSM1(void *handle, u8_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_PEAK1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_PEAK1_BIT_MASK; //coerce	
  *value = *value >> LSM330_ACC_PEAK1_BIT_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_PeakSM2
* Description    : Read PEAK2_BIT
* Input          : Pointer to u8_t
* Output         : Status of PEAK2_BIT 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_PeakSM2(void *handle, u8_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_PEAK2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_PEAK2_BIT_MASK; //coerce	
  *value = *value >> LSM330_ACC_PEAK2_BIT_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_Threshold3
* Description    : Write THRS3_BIT
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_Threshold3(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM330_ACC_THRS3_BIT_POSITION; //mask	
  newValue &= LSM330_ACC_THRS3_BIT_MASK; //coerce
  
  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_THRS3, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_THRS3_BIT_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_THRS3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_Threshold3
* Description    : Read THRS3_BIT
* Input          : Pointer to u8_t
* Output         : Status of THRS3_BIT 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_Threshold3(void *handle, u8_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_THRS3, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_THRS3_BIT_MASK; //coerce	
  *value = *value >> LSM330_ACC_THRS3_BIT_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_AxisX
* Description    : Write XEN
* Input          : LSM330_ACC_XEN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_AxisX(void *handle, LSM330_ACC_XEN_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL4, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_XEN_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_CTRL4, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_AxisX
* Description    : Read XEN
* Input          : Pointer to LSM330_ACC_XEN_t
* Output         : Status of XEN see LSM330_ACC_XEN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_AxisX(void *handle, LSM330_ACC_XEN_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL4, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_XEN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_AxisY
* Description    : Write YEN
* Input          : LSM330_ACC_YEN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_AxisY(void *handle, LSM330_ACC_YEN_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL4, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_YEN_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_CTRL4, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_AxisY
* Description    : Read YEN
* Input          : Pointer to LSM330_ACC_YEN_t
* Output         : Status of YEN see LSM330_ACC_YEN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_AxisY (void *handle, LSM330_ACC_YEN_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL4, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_YEN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_AxisZ
* Description    : Write ZEN
* Input          : LSM330_ACC_ZEN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_AxisZ (void *handle, LSM330_ACC_ZEN_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL4, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_ZEN_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_CTRL4, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_AxisZ
* Description    : Read ZEN
* Input          : Pointer to LSM330_ACC_ZEN_t
* Output         : Status of ZEN see LSM330_ACC_ZEN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_AxisZ (void *handle, LSM330_ACC_ZEN_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL4, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_ZEN_MASK; //mask

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM330_ACC_W_StateMachine1
* Description    : Write SM1_EN
* Input          : LSM330_ACC_SM1_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_StateMachine1 (void *handle, LSM330_ACC_SM1_EN_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM1_EN_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_CTRL1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_StateMachine1
* Description    : Read SM1_EN
* Input          : Pointer to LSM330_ACC_SM1_EN_t
* Output         : Status of SM1_EN see LSM330_ACC_SM1_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_StateMachine1 (void *handle, LSM330_ACC_SM1_EN_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM1_EN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM1_PinSelector
* Description    : Write SM1_PIN
* Input          : LSM330_ACC_SM1_PIN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM1_PinSelector (void *handle, LSM330_ACC_SM1_PIN_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM1_PIN_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_CTRL1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM1_PinSelector
* Description    : Read SM1_PIN
* Input          : Pointer to LSM330_ACC_SM1_PIN_t
* Output         : Status of SM1_PIN see LSM330_ACC_SM1_PIN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM1_PinSelector (void *handle, LSM330_ACC_SM1_PIN_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM1_PIN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM1_Hysteresis
* Description    : Write HYST_SM1
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM1_Hysteresis(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM330_ACC_HYST_SM1_POSITION; //mask	
  newValue &= LSM330_ACC_HYST_SM1_MASK; //coerce
  
  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_HYST_SM1_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_CTRL1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM1_Hysteresis
* Description    : Read HYST_SM1
* Input          : Pointer to u8_t
* Output         : Status of HYST_SM1 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM1_Hysteresis(void *handle, u8_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_HYST_SM1_MASK; //coerce	
  *value = *value >> LSM330_ACC_HYST_SM1_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_StateMachine2
* Description    : Write SM2_EN
* Input          : LSM330_ACC_SM2_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_StateMachine2 (void *handle, LSM330_ACC_SM2_EN_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM2_EN_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_CTRL2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_StateMachine2
* Description    : Read SM2_EN
* Input          : Pointer to LSM330_ACC_SM2_EN_t
* Output         : Status of SM2_EN see LSM330_ACC_SM2_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_StateMachine2 (void *handle, LSM330_ACC_SM2_EN_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM2_EN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM2_PinSelector
* Description    : Write SM2_PIN
* Input          : LSM330_ACC_SM2_PIN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM2_PinSelector (void *handle, LSM330_ACC_SM2_PIN_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM2_PIN_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_CTRL2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM2_PinSelector
* Description    : Read SM2_PIN
* Input          : Pointer to LSM330_ACC_SM2_PIN_t
* Output         : Status of SM2_PIN see LSM330_ACC_SM2_PIN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM2_PinSelector (void *handle, LSM330_ACC_SM2_PIN_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM2_PIN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM2_Hysteresis
* Description    : Write HYST_SM2
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM2_Hysteresis(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM330_ACC_HYST_SM2_POSITION; //mask	
  newValue &= LSM330_ACC_HYST_SM2_MASK; //coerce
  
  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_HYST_SM2_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_CTRL2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM2_Hysteresis
* Description    : Read HYST_SM2
* Input          : Pointer to u8_t
* Output         : Status of HYST_SM2 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM2_Hysteresis(void *handle, u8_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_HYST_SM2_MASK; //coerce	
  *value = *value >> LSM330_ACC_HYST_SM2_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SoftReset
* Description    : Write STRT
* Input          : LSM330_ACC_STRT_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SoftReset (void *handle, LSM330_ACC_STRT_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL3, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_STRT_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_CTRL3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SoftReset
* Description    : Read STRT
* Input          : Pointer to LSM330_ACC_STRT_t
* Output         : Status of STRT see LSM330_ACC_STRT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SoftReset (void *handle, LSM330_ACC_STRT_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL3, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_STRT_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_VectorFilter
* Description    : Write VFILT
* Input          : LSM330_ACC_VFILT_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_VectorFilter (void *handle, LSM330_ACC_VFILT_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL3, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_VFILT_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_CTRL3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_VectorFilter
* Description    : Read VFILT
* Input          : Pointer to LSM330_ACC_VFILT_t
* Output         : Status of VFILT see LSM330_ACC_VFILT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_VectorFilter (void *handle, LSM330_ACC_VFILT_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL3, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_VFILT_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_INT1_Pin
* Description    : Write INT1_EN
* Input          : LSM330_ACC_INT1_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_INT1_Pin (void *handle, LSM330_ACC_INT1_EN_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL3, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_INT1_EN_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_CTRL3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_INT1_Pin
* Description    : Read INT1_EN
* Input          : Pointer to LSM330_ACC_INT1_EN_t
* Output         : Status of INT1_EN see LSM330_ACC_INT1_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_INT1_Pin (void *handle, LSM330_ACC_INT1_EN_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL3, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_INT1_EN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_INT2_Pin
* Description    : Write INT2_EN
* Input          : LSM330_ACC_INT2_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_INT2_Pin (void *handle, LSM330_ACC_INT2_EN_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL3, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_INT2_EN_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_CTRL3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_INT2_Pin
* Description    : Read INT2_EN
* Input          : Pointer to LSM330_ACC_INT2_EN_t
* Output         : Status of INT2_EN see LSM330_ACC_INT2_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_INT2_Pin (void *handle, LSM330_ACC_INT2_EN_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL3, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_INT2_EN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_InterruptSignalType
* Description    : Write IEL
* Input          : LSM330_ACC_IEL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_InterruptSignalType (void *handle, LSM330_ACC_IEL_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL3, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_IEL_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_CTRL3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_InterruptSignalType
* Description    : Read IEL
* Input          : Pointer to LSM330_ACC_IEL_t
* Output         : Status of IEL see LSM330_ACC_IEL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_InterruptSignalType (void *handle, LSM330_ACC_IEL_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL3, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_IEL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_InterruptSignalPolarity
* Description    : Write IEA
* Input          : LSM330_ACC_IEA_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_InterruptSignalPolarity (void *handle, LSM330_ACC_IEA_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL3, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_IEA_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_CTRL3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_InterruptSignalPolarity
* Description    : Read IEA
* Input          : Pointer to LSM330_ACC_IEA_t
* Output         : Status of IEA see LSM330_ACC_IEA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_InterruptSignalPolarity (void *handle, LSM330_ACC_IEA_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL3, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_IEA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_DataReadyOnINT1
* Description    : Write DR_EN
* Input          : LSM330_ACC_DR_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_DataReadyOnINT1 (void *handle, LSM330_ACC_DR_EN_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL3, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_DR_EN_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_CTRL3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_DataReadyOnINT1
* Description    : Read DR_EN
* Input          : Pointer to LSM330_ACC_DR_EN_t
* Output         : Status of DR_EN see LSM330_ACC_DR_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_DataReadyOnINT1 (void *handle, LSM330_ACC_DR_EN_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL3, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_DR_EN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SerialInterfaceSPI
* Description    : Write SIM
* Input          : LSM330_ACC_SIM_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SerialInterfaceSPI (void *handle, LSM330_ACC_SIM_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL5, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SIM_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_CTRL5, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SerialInterfaceSPI
* Description    : Read SIM
* Input          : Pointer to LSM330_ACC_SIM_t
* Output         : Status of SIM see LSM330_ACC_SIM_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SerialInterfaceSPI (void *handle, LSM330_ACC_SIM_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL5, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SIM_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SelfTest
* Description    : Write ST
* Input          : LSM330_ACC_ST_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SelfTest (void *handle, LSM330_ACC_ST_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL5, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_ST_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_CTRL5, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SelfTest
* Description    : Read ST
* Input          : Pointer to LSM330_ACC_ST_t
* Output         : Status of ST see LSM330_ACC_ST_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SelfTest (void *handle, LSM330_ACC_ST_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL5, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_ST_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_AntiAliasingFilter
* Description    : Write BW
* Input          : LSM330_ACC_BW_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_AntiAliasingFilter (void *handle, LSM330_ACC_BW_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL5, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_BW_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_CTRL5, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_AntiAliasingFilter
* Description    : Read BW
* Input          : Pointer to LSM330_ACC_BW_t
* Output         : Status of BW see LSM330_ACC_BW_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_AntiAliasingFilter (void *handle, LSM330_ACC_BW_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL5, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_BW_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_BootOnINT2
* Description    : Write I2_BOOT
* Input          : LSM330_ACC_I2_BOOT_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_BootOnINT2 (void *handle, LSM330_ACC_I2_BOOT_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL6, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_I2_BOOT_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_CTRL6, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_BootOnINT2
* Description    : Read I2_BOOT
* Input          : Pointer to LSM330_ACC_I2_BOOT_t
* Output         : Status of I2_BOOT see LSM330_ACC_I2_BOOT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_BootOnINT2 (void *handle, LSM330_ACC_I2_BOOT_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL6, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_I2_BOOT_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_FIFO_OverrunOnINT1
* Description    : Write I1_OVERRUN
* Input          : LSM330_ACC_I1_OVERRUN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_FIFO_OverrunOnINT1 (void *handle, LSM330_ACC_I1_OVERRUN_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL6, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_I1_OVERRUN_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_CTRL6, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_FIFO_OverrunOnINT1
* Description    : Read I1_OVERRUN
* Input          : Pointer to LSM330_ACC_I1_OVERRUN_t
* Output         : Status of I1_OVERRUN see LSM330_ACC_I1_OVERRUN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_FIFO_OverrunOnINT1 (void *handle, LSM330_ACC_I1_OVERRUN_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL6, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_I1_OVERRUN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_FIFO_WatermarkOnINT1
* Description    : Write I1_WTM
* Input          : LSM330_ACC_I1_WTM_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_FIFO_WatermarkOnINT1 (void *handle, LSM330_ACC_I1_WTM_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL6, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_I1_WTM_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_CTRL6, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_FIFO_WatermarkOnINT1
* Description    : Read I1_WTM
* Input          : Pointer to LSM330_ACC_I1_WTM_t
* Output         : Status of I1_WTM see LSM330_ACC_I1_WTM_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_FIFO_WatermarkOnINT1 (void *handle, LSM330_ACC_I1_WTM_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL6, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_I1_WTM_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_FIFO_EmptyOnINT1
* Description    : Write I1_EMPTY
* Input          : LSM330_ACC_I1_EMPTY_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_FIFO_EmptyOnINT1 (void *handle, LSM330_ACC_I1_EMPTY_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL6, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_I1_EMPTY_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_CTRL6, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_FIFO_EmptyOnINT1
* Description    : Read I1_EMPTY
* Input          : Pointer to LSM330_ACC_I1_EMPTY_t
* Output         : Status of I1_EMPTY see LSM330_ACC_I1_EMPTY_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_FIFO_EmptyOnINT1 (void *handle, LSM330_ACC_I1_EMPTY_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL6, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_I1_EMPTY_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_AutoInc
* Description    : Write ADD_INC
* Input          : LSM330_ACC_ADD_INC_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_AutoInc (void *handle, LSM330_ACC_ADD_INC_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL6, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_ADD_INC_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_CTRL6, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_AutoInc
* Description    : Read ADD_INC
* Input          : Pointer to LSM330_ACC_ADD_INC_t
* Output         : Status of ADD_INC see LSM330_ACC_ADD_INC_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_AutoInc (void *handle, LSM330_ACC_ADD_INC_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL6, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_ADD_INC_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_StopOnWatermark
* Description    : Write STP_WTM
* Input          : LSM330_ACC_STP_WTM_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_StopOnWatermark (void *handle, LSM330_ACC_STP_WTM_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL6, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_STP_WTM_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_CTRL6, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_StopOnWatermark
* Description    : Read STP_WTM
* Input          : Pointer to LSM330_ACC_STP_WTM_t
* Output         : Status of STP_WTM see LSM330_ACC_STP_WTM_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_StopOnWatermark (void *handle, LSM330_ACC_STP_WTM_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL6, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_STP_WTM_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_FIFO_
* Description    : Write FIFO_EN
* Input          : LSM330_ACC_FIFO_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_FIFO_ (void *handle, LSM330_ACC_FIFO_EN_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL6, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_FIFO_EN_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_CTRL6, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_FIFO_
* Description    : Read FIFO_EN
* Input          : Pointer to LSM330_ACC_FIFO_EN_t
* Output         : Status of FIFO_EN see LSM330_ACC_FIFO_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_FIFO_ (void *handle, LSM330_ACC_FIFO_EN_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL6, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_FIFO_EN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_ForceReboot
* Description    : Write BOOT
* Input          : LSM330_ACC_BOOT_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_ForceReboot (void *handle, LSM330_ACC_BOOT_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL6, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_BOOT_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_CTRL6, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_ForceReboot
* Description    : Read BOOT
* Input          : Pointer to LSM330_ACC_BOOT_t
* Output         : Status of BOOT see LSM330_ACC_BOOT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_ForceReboot (void *handle, LSM330_ACC_BOOT_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_CTRL6, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_BOOT_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_NewDataX
* Description    : Read XDA
* Input          : Pointer to LSM330_ACC_XDA_t
* Output         : Status of XDA see LSM330_ACC_XDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_NewDataX (void *handle, LSM330_ACC_XDA_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_STATUS, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_XDA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_NewDataY
* Description    : Read YDA
* Input          : Pointer to LSM330_ACC_YDA_t
* Output         : Status of YDA see LSM330_ACC_YDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_NewDataY (void *handle, LSM330_ACC_YDA_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_STATUS, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_YDA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_NewDataZ
* Description    : Read ZDA
* Input          : Pointer to LSM330_ACC_ZDA_t
* Output         : Status of ZDA see LSM330_ACC_ZDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_NewDataZ (void *handle, LSM330_ACC_ZDA_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_STATUS, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_ZDA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_NewDataXYZ
* Description    : Read ZYXDA
* Input          : Pointer to LSM330_ACC_ZYXDA_t
* Output         : Status of ZYXDA see LSM330_ACC_ZYXDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_NewDataXYZ (void *handle, LSM330_ACC_ZYXDA_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_STATUS, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_ZYXDA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_DataOverrunX
* Description    : Read XOR
* Input          : Pointer to LSM330_ACC_XOR_t
* Output         : Status of XOR see LSM330_ACC_XOR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_DataOverrunX (void *handle, LSM330_ACC_XOR_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_STATUS, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_XOR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_DataOverrunY
* Description    : Read YOR
* Input          : Pointer to LSM330_ACC_YOR_t
* Output         : Status of YOR see LSM330_ACC_YOR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_DataOverrunY (void *handle, LSM330_ACC_YOR_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_STATUS, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_YOR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_DataOverrunZ
* Description    : Read ZOR
* Input          : Pointer to LSM330_ACC_ZOR_t
* Output         : Status of ZOR see LSM330_ACC_ZOR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_DataOverrunZ (void *handle, LSM330_ACC_ZOR_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_STATUS, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_ZOR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_DataOverrunXYZ
* Description    : Read ZYXOR
* Input          : Pointer to LSM330_ACC_ZYXOR_t
* Output         : Status of ZYXOR see LSM330_ACC_ZYXOR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_DataOverrunXYZ (void *handle, LSM330_ACC_ZYXOR_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_STATUS, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_ZYXOR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_FIFO_WatermarkPointer
* Description    : Write WTMP
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_FIFO_WatermarkPointer(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM330_ACC_WTMP_POSITION; //mask	
  newValue &= LSM330_ACC_WTMP_MASK; //coerce
  
  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_FIFO_CTRL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_WTMP_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_FIFO_CTRL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_FIFO_WatermarkPointer
* Description    : Read WTMP
* Input          : Pointer to u8_t
* Output         : Status of WTMP 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_FIFO_WatermarkPointer(void *handle, u8_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_FIFO_CTRL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_WTMP_MASK; //coerce	
  *value = *value >> LSM330_ACC_WTMP_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_FIFO_Mode
* Description    : Write FMODE
* Input          : LSM330_ACC_FMODE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_FIFO_Mode (void *handle, LSM330_ACC_FMODE_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_FIFO_CTRL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_FMODE_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_FIFO_CTRL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_FIFO_Mode
* Description    : Read FMODE
* Input          : Pointer to LSM330_ACC_FMODE_t
* Output         : Status of FMODE see LSM330_ACC_FMODE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_FIFO_Mode (void *handle, LSM330_ACC_FMODE_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_FIFO_CTRL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_FMODE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_FIFO_StoredData
* Description    : Read FSS
* Input          : Pointer to u8_t
* Output         : Status of FSS 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_FIFO_StoredData(void *handle, u8_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_FIFO_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_FSS_MASK; //coerce	
  *value = *value >> LSM330_ACC_FSS_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_FIFO_EmptyFlag
* Description    : Read EMPTY
* Input          : Pointer to LSM330_ACC_EMPTY_t
* Output         : Status of EMPTY see LSM330_ACC_EMPTY_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_FIFO_EmptyFlag (void *handle, LSM330_ACC_EMPTY_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_FIFO_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_EMPTY_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_FIFO_OverrunFlag
* Description    : Read OVR
* Input          : Pointer to LSM330_ACC_OVR_t
* Output         : Status of OVR see LSM330_ACC_OVR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_FIFO_OverrunFlag (void *handle, LSM330_ACC_OVR_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_FIFO_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_OVR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_FIFO_WatermarkFlag
* Description    : Read WTM
* Input          : Pointer to LSM330_ACC_WTM_t
* Output         : Status of WTM see LSM330_ACC_WTM_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_FIFO_WatermarkFlag (void *handle, LSM330_ACC_WTM_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_FIFO_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_WTM_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_Timer4_SM1
* Description    : Write TIM4_SM1_BIT
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_Timer4_SM1(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM330_ACC_TIM4_SM1_BIT_POSITION; //mask	
  newValue &= LSM330_ACC_TIM4_SM1_BIT_MASK; //coerce
  
  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_TIM4_SM1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_TIM4_SM1_BIT_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_TIM4_SM1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_Timer4_SM1
* Description    : Read TIM4_SM1_BIT
* Input          : Pointer to u8_t
* Output         : Status of TIM4_SM1_BIT 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_Timer4_SM1(void *handle, u8_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_TIM4_SM1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_TIM4_SM1_BIT_MASK; //coerce	
  *value = *value >> LSM330_ACC_TIM4_SM1_BIT_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_Timer3_SM1
* Description    : Write TIM3_SM1_BIT
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_Timer3_SM1(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM330_ACC_TIM3_SM1_BIT_POSITION; //mask	
  newValue &= LSM330_ACC_TIM3_SM1_BIT_MASK; //coerce
  
  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_TIM3_SM1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_TIM3_SM1_BIT_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_TIM3_SM1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_Timer3_SM1
* Description    : Read TIM3_SM1_BIT
* Input          : Pointer to u8_t
* Output         : Status of TIM3_SM1_BIT 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_Timer3_SM1(void *handle, u8_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_TIM3_SM1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_TIM3_SM1_BIT_MASK; //coerce	
  *value = *value >> LSM330_ACC_TIM3_SM1_BIT_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_Threshold2_SM1
* Description    : Write THRS2_SM1_BIT
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_Threshold2_SM1(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM330_ACC_THRS2_SM1_BIT_POSITION; //mask	
  newValue &= LSM330_ACC_THRS2_SM1_BIT_MASK; //coerce
  
  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_THRS2_SM1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_THRS2_SM1_BIT_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_THRS2_SM1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_Threshold2_SM1
* Description    : Read THRS2_SM1_BIT
* Input          : Pointer to u8_t
* Output         : Status of THRS2_SM1_BIT 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_Threshold2_SM1(void *handle, u8_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_THRS2_SM1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_THRS2_SM1_BIT_MASK; //coerce	
  *value = *value >> LSM330_ACC_THRS2_SM1_BIT_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_Threshold1_SM1
* Description    : Write THRS1_SM1_BIT
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_Threshold1_SM1(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM330_ACC_THRS1_SM1_BIT_POSITION; //mask	
  newValue &= LSM330_ACC_THRS1_SM1_BIT_MASK; //coerce
  
  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_THRS1_SM1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_THRS1_SM1_BIT_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_THRS1_SM1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_Threshold1_SM1
* Description    : Read THRS1_SM1_BIT
* Input          : Pointer to u8_t
* Output         : Status of THRS1_SM1_BIT 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_Threshold1_SM1(void *handle, u8_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_THRS1_SM1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_THRS1_SM1_BIT_MASK; //coerce	
  *value = *value >> LSM330_ACC_THRS1_SM1_BIT_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM1_MaskB_NegativeVector
* Description    : Write SM1_MB_N_V
* Input          : LSM330_ACC_SM1_MB_N_V_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM1_MaskB_NegativeVector (void *handle, LSM330_ACC_SM1_MB_N_V_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKB_SM1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM1_MB_N_V_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_MASKB_SM1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM1_MaskB_NegativeVector
* Description    : Read SM1_MB_N_V
* Input          : Pointer to LSM330_ACC_SM1_MB_N_V_t
* Output         : Status of SM1_MB_N_V see LSM330_ACC_SM1_MB_N_V_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM1_MaskB_NegativeVector (void *handle, LSM330_ACC_SM1_MB_N_V_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKB_SM1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM1_MB_N_V_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM1_MaskB_PositiveVector
* Description    : Write SM1_MB_P_V
* Input          : LSM330_ACC_SM1_MB_P_V_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM1_MaskB_PositiveVector (void *handle, LSM330_ACC_SM1_MB_P_V_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKB_SM1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM1_MB_P_V_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_MASKB_SM1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM1_MaskB_PositiveVector
* Description    : Read SM1_MB_P_V
* Input          : Pointer to LSM330_ACC_SM1_MB_P_V_t
* Output         : Status of SM1_MB_P_V see LSM330_ACC_SM1_MB_P_V_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM1_MaskB_PositiveVector (void *handle, LSM330_ACC_SM1_MB_P_V_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKB_SM1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM1_MB_P_V_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM1_MaskB_NegativeZ
* Description    : Write SM1_MB_N_Z
* Input          : LSM330_ACC_SM1_MB_N_Z_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM1_MaskB_NegativeZ (void *handle, LSM330_ACC_SM1_MB_N_Z_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKB_SM1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM1_MB_N_Z_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_MASKB_SM1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM1_MaskB_NegativeZ
* Description    : Read SM1_MB_N_Z
* Input          : Pointer to LSM330_ACC_SM1_MB_N_Z_t
* Output         : Status of SM1_MB_N_Z see LSM330_ACC_SM1_MB_N_Z_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM1_MaskB_NegativeZ (void *handle, LSM330_ACC_SM1_MB_N_Z_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKB_SM1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM1_MB_N_Z_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM1_MaskB_PositiveZ
* Description    : Write SM1_MB_P_Z
* Input          : LSM330_ACC_SM1_MB_P_Z_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM1_MaskB_PositiveZ (void *handle, LSM330_ACC_SM1_MB_P_Z_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKB_SM1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM1_MB_P_Z_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_MASKB_SM1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM1_MaskB_PositiveZ
* Description    : Read SM1_MB_P_Z
* Input          : Pointer to LSM330_ACC_SM1_MB_P_Z_t
* Output         : Status of SM1_MB_P_Z see LSM330_ACC_SM1_MB_P_Z_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM1_MaskB_PositiveZ (void *handle, LSM330_ACC_SM1_MB_P_Z_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKB_SM1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM1_MB_P_Z_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM1_MaskB_NegativeY
* Description    : Write SM1_MB_N_Y
* Input          : LSM330_ACC_SM1_MB_N_Y_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM1_MaskB_NegativeY (void *handle, LSM330_ACC_SM1_MB_N_Y_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKB_SM1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM1_MB_N_Y_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_MASKB_SM1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM1_MaskB_NegativeY
* Description    : Read SM1_MB_N_Y
* Input          : Pointer to LSM330_ACC_SM1_MB_N_Y_t
* Output         : Status of SM1_MB_N_Y see LSM330_ACC_SM1_MB_N_Y_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM1_MaskB_NegativeY (void *handle, LSM330_ACC_SM1_MB_N_Y_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKB_SM1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM1_MB_N_Y_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM1_MaskB_PositiveY
* Description    : Write SM1_MB_P_Y
* Input          : LSM330_ACC_SM1_MB_P_Y_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM1_MaskB_PositiveY (void *handle, LSM330_ACC_SM1_MB_P_Y_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKB_SM1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM1_MB_P_Y_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_MASKB_SM1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM1_MaskB_PositiveY
* Description    : Read SM1_MB_P_Y
* Input          : Pointer to LSM330_ACC_SM1_MB_P_Y_t
* Output         : Status of SM1_MB_P_Y see LSM330_ACC_SM1_MB_P_Y_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM1_MaskB_PositiveY (void *handle, LSM330_ACC_SM1_MB_P_Y_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKB_SM1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM1_MB_P_Y_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM1_MaskB_NegativeX
* Description    : Write SM1_MB_N_X
* Input          : LSM330_ACC_SM1_MB_N_X_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM1_MaskB_NegativeX (void *handle, LSM330_ACC_SM1_MB_N_X_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKB_SM1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM1_MB_N_X_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_MASKB_SM1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM1_MaskB_NegativeX
* Description    : Read SM1_MB_N_X
* Input          : Pointer to LSM330_ACC_SM1_MB_N_X_t
* Output         : Status of SM1_MB_N_X see LSM330_ACC_SM1_MB_N_X_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM1_MaskB_NegativeX (void *handle, LSM330_ACC_SM1_MB_N_X_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKB_SM1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM1_MB_N_X_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM1_MaskB_PositiveX
* Description    : Write SM1_MB_P_X
* Input          : LSM330_ACC_SM1_MB_P_X_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM1_MaskB_PositiveX (void *handle, LSM330_ACC_SM1_MB_P_X_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKB_SM1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM1_MB_P_X_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_MASKB_SM1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM1_MaskB_PositiveX
* Description    : Read SM1_MB_P_X
* Input          : Pointer to LSM330_ACC_SM1_MB_P_X_t
* Output         : Status of SM1_MB_P_X see LSM330_ACC_SM1_MB_P_X_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM1_MaskB_PositiveX (void *handle, LSM330_ACC_SM1_MB_P_X_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKB_SM1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM1_MB_P_X_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM1_MaskA_NegativeVector
* Description    : Write SM1_MA_N_V
* Input          : LSM330_ACC_SM1_MA_N_V_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM1_MaskA_NegativeVector (void *handle, LSM330_ACC_SM1_MA_N_V_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKA_SM1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM1_MA_N_V_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_MASKA_SM1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM1_MaskA_NegativeVector
* Description    : Read SM1_MA_N_V
* Input          : Pointer to LSM330_ACC_SM1_MA_N_V_t
* Output         : Status of SM1_MA_N_V see LSM330_ACC_SM1_MA_N_V_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM1_MaskA_NegativeVector (void *handle, LSM330_ACC_SM1_MA_N_V_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKA_SM1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM1_MA_N_V_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM1_MaskA_PositiveVector
* Description    : Write SM1_MA_P_V
* Input          : LSM330_ACC_SM1_MA_P_V_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM1_MaskA_PositiveVector (void *handle, LSM330_ACC_SM1_MA_P_V_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKA_SM1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM1_MA_P_V_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_MASKA_SM1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM1_MaskA_PositiveVector
* Description    : Read SM1_MA_P_V
* Input          : Pointer to LSM330_ACC_SM1_MA_P_V_t
* Output         : Status of SM1_MA_P_V see LSM330_ACC_SM1_MA_P_V_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM1_MaskA_PositiveVector (void *handle, LSM330_ACC_SM1_MA_P_V_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKA_SM1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM1_MA_P_V_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM1_MaskA_NegativeZ
* Description    : Write SM1_MA_N_Z
* Input          : LSM330_ACC_SM1_MA_N_Z_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM1_MaskA_NegativeZ (void *handle, LSM330_ACC_SM1_MA_N_Z_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKA_SM1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM1_MA_N_Z_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_MASKA_SM1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM1_MaskA_NegativeZ
* Description    : Read SM1_MA_N_Z
* Input          : Pointer to LSM330_ACC_SM1_MA_N_Z_t
* Output         : Status of SM1_MA_N_Z see LSM330_ACC_SM1_MA_N_Z_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM1_MaskA_NegativeZ (void *handle, LSM330_ACC_SM1_MA_N_Z_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKA_SM1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM1_MA_N_Z_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM1_MaskA_PositiveZ
* Description    : Write SM1_MA_P_Z
* Input          : LSM330_ACC_SM1_MA_P_Z_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM1_MaskA_PositiveZ (void *handle, LSM330_ACC_SM1_MA_P_Z_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKA_SM1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM1_MA_P_Z_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_MASKA_SM1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM1_MaskA_PositiveZ
* Description    : Read SM1_MA_P_Z
* Input          : Pointer to LSM330_ACC_SM1_MA_P_Z_t
* Output         : Status of SM1_MA_P_Z see LSM330_ACC_SM1_MA_P_Z_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM1_MaskA_PositiveZ (void *handle, LSM330_ACC_SM1_MA_P_Z_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKA_SM1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM1_MA_P_Z_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM1_MaskA_NegativeY
* Description    : Write SM1_MA_N_Y
* Input          : LSM330_ACC_SM1_MA_N_Y_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM1_MaskA_NegativeY (void *handle, LSM330_ACC_SM1_MA_N_Y_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKA_SM1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM1_MA_N_Y_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_MASKA_SM1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM1_MaskA_NegativeY
* Description    : Read SM1_MA_N_Y
* Input          : Pointer to LSM330_ACC_SM1_MA_N_Y_t
* Output         : Status of SM1_MA_N_Y see LSM330_ACC_SM1_MA_N_Y_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM1_MaskA_NegativeY (void *handle, LSM330_ACC_SM1_MA_N_Y_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKA_SM1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM1_MA_N_Y_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM1_MaskA_PositiveY
* Description    : Write SM1_MA_P_Y
* Input          : LSM330_ACC_SM1_MA_P_Y_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM1_MaskA_PositiveY (void *handle, LSM330_ACC_SM1_MA_P_Y_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKA_SM1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM1_MA_P_Y_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_MASKA_SM1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM1_MaskA_PositiveY
* Description    : Read SM1_MA_P_Y
* Input          : Pointer to LSM330_ACC_SM1_MA_P_Y_t
* Output         : Status of SM1_MA_P_Y see LSM330_ACC_SM1_MA_P_Y_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM1_MaskA_PositiveY (void *handle, LSM330_ACC_SM1_MA_P_Y_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKA_SM1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM1_MA_P_Y_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM1_MaskA_NegativeX
* Description    : Write SM1_MA_N_X
* Input          : LSM330_ACC_SM1_MA_N_X_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM1_MaskA_NegativeX (void *handle, LSM330_ACC_SM1_MA_N_X_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKA_SM1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM1_MA_N_X_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_MASKA_SM1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM1_MaskA_NegativeX
* Description    : Read SM1_MA_N_X
* Input          : Pointer to LSM330_ACC_SM1_MA_N_X_t
* Output         : Status of SM1_MA_N_X see LSM330_ACC_SM1_MA_N_X_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM1_MaskA_NegativeX (void *handle, LSM330_ACC_SM1_MA_N_X_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKA_SM1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM1_MA_N_X_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM1_MaskA_PositiveX
* Description    : Write SM1_MA_P_X
* Input          : LSM330_ACC_SM1_MA_P_X_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM1_MaskA_PositiveX (void *handle, LSM330_ACC_SM1_MA_P_X_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKA_SM1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM1_MA_P_X_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_MASKA_SM1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM1_MaskA_PositiveX
* Description    : Read SM1_MA_P_X
* Input          : Pointer to LSM330_ACC_SM1_MA_P_X_t
* Output         : Status of SM1_MA_P_X see LSM330_ACC_SM1_MA_P_X_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM1_MaskA_PositiveX (void *handle, LSM330_ACC_SM1_MA_P_X_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKA_SM1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM1_MA_P_X_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM1_ProgramFlowModification
* Description    : Write SM1_SITR
* Input          : LSM330_ACC_SM1_SITR_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM1_ProgramFlowModification (void *handle, LSM330_ACC_SM1_SITR_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_SETT_SM1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM1_SITR_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_SETT_SM1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM1_ProgramFlowModification
* Description    : Read SM1_SITR
* Input          : Pointer to LSM330_ACC_SM1_SITR_t
* Output         : Status of SM1_SITR see LSM330_ACC_SM1_SITR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM1_ProgramFlowModification (void *handle, LSM330_ACC_SM1_SITR_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_SETT_SM1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM1_SITR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM1_NextConditionValidation
* Description    : Write SM1_R_TAM
* Input          : LSM330_ACC_SM1_R_TAM_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM1_NextConditionValidation (void *handle, LSM330_ACC_SM1_R_TAM_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_SETT_SM1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM1_R_TAM_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_SETT_SM1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM1_NextConditionValidation
* Description    : Read SM1_R_TAM
* Input          : Pointer to LSM330_ACC_SM1_R_TAM_t
* Output         : Status of SM1_R_TAM see LSM330_ACC_SM1_R_TAM_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM1_NextConditionValidation (void *handle, LSM330_ACC_SM1_R_TAM_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_SETT_SM1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM1_R_TAM_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM1_Threshold3
* Description    : Write SM1_THR3_MA
* Input          : LSM330_ACC_SM1_THR3_MA_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM1_Threshold3_OnMaskA (void *handle, LSM330_ACC_SM1_THR3_MA_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_SETT_SM1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM1_THR3_MA_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_SETT_SM1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM1_Threshold3
* Description    : Read SM1_THR3_MA
* Input          : Pointer to LSM330_ACC_SM1_THR3_MA_t
* Output         : Status of SM1_THR3_MA see LSM330_ACC_SM1_THR3_MA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM1_Threshold3_OnMaskA (void *handle, LSM330_ACC_SM1_THR3_MA_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_SETT_SM1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM1_THR3_MA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM1_AbsoluteValue
* Description    : Write SM1_ABS
* Input          : LSM330_ACC_SM1_ABS_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM1_AbsoluteValue (void *handle, LSM330_ACC_SM1_ABS_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_SETT_SM1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM1_ABS_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_SETT_SM1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM1_AbsoluteValue
* Description    : Read SM1_ABS
* Input          : Pointer to LSM330_ACC_SM1_ABS_t
* Output         : Status of SM1_ABS see LSM330_ACC_SM1_ABS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM1_AbsoluteValue (void *handle, LSM330_ACC_SM1_ABS_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_SETT_SM1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM1_ABS_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM1_Threshold3_OnMaskB
* Description    : Write SM1_THR3_SA
* Input          : LSM330_ACC_SM1_THR3_SA_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM1_Threshold3_OnMaskB (void *handle, LSM330_ACC_SM1_THR3_SA_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_SETT_SM1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM1_THR3_SA_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_SETT_SM1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM1_Threshold3_OnMaskB
* Description    : Read SM1_THR3_SA
* Input          : Pointer to LSM330_ACC_SM1_THR3_SA_t
* Output         : Status of SM1_THR3_SA see LSM330_ACC_SM1_THR3_SA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM1_Threshold3_OnMaskB (void *handle, LSM330_ACC_SM1_THR3_SA_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_SETT_SM1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM1_THR3_SA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM1_PeakDetection
* Description    : Write SM1_P_DET
* Input          : LSM330_ACC_SM1_P_DET_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM1_PeakDetection (void *handle, LSM330_ACC_SM1_P_DET_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_SETT_SM1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM1_P_DET_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_SETT_SM1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM1_PeakDetection
* Description    : Read SM1_P_DET
* Input          : Pointer to LSM330_ACC_SM1_P_DET_t
* Output         : Status of SM1_P_DET see LSM330_ACC_SM1_P_DET_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM1_PeakDetection (void *handle, LSM330_ACC_SM1_P_DET_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_SETT_SM1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM1_P_DET_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM1_ResetPointer
* Description    : Write SM1_RP
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM1_ResetPointer(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM330_ACC_SM1_RP_POSITION; //mask	
  newValue &= LSM330_ACC_SM1_RP_MASK; //coerce
  
  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_PR_SM1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM1_RP_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_PR_SM1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM1_ResetPointer
* Description    : Read SM1_RP
* Input          : Pointer to u8_t
* Output         : Status of SM1_RP 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM1_ResetPointer(void *handle, u8_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_PR_SM1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM1_RP_MASK; //coerce	
  *value = *value >> LSM330_ACC_SM1_RP_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM1_ProgramPointer
* Description    : Write SM1_PP
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM1_ProgramPointer(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM330_ACC_SM1_PP_POSITION; //mask	
  newValue &= LSM330_ACC_SM1_PP_MASK; //coerce
  
  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_PR_SM1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM1_PP_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_PR_SM1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM1_ProgramPointer
* Description    : Read SM1_PP
* Input          : Pointer to u8_t
* Output         : Status of SM1_PP 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM1_ProgramPointer(void *handle, u8_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_PR_SM1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM1_PP_MASK; //coerce	
  *value = *value >> LSM330_ACC_SM1_PP_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM1_NegativeVectorFlag
* Description    : Read SM1_FLAG_N_V
* Input          : Pointer to LSM330_ACC_SM1_FLAG_N_V_t
* Output         : Status of SM1_FLAG_N_V see LSM330_ACC_SM1_FLAG_N_V_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM1_NegativeVectorFlag (void *handle, LSM330_ACC_SM1_FLAG_N_V_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_OUTS_SM1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM1_FLAG_N_V_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM1_PositiveVectorFlag
* Description    : Read SM1_FLAG_P_V
* Input          : Pointer to LSM330_ACC_SM1_FLAG_P_V_t
* Output         : Status of SM1_FLAG_P_V see LSM330_ACC_SM1_FLAG_P_V_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM1_PositiveVectorFlag (void *handle, LSM330_ACC_SM1_FLAG_P_V_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_OUTS_SM1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM1_FLAG_P_V_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM1_NegativeZFlag
* Description    : Read SM1_FLAG_N_Z
* Input          : Pointer to LSM330_ACC_SM1_FLAG_N_Z_t
* Output         : Status of SM1_FLAG_N_Z see LSM330_ACC_SM1_FLAG_N_Z_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM1_NegativeZFlag (void *handle, LSM330_ACC_SM1_FLAG_N_Z_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_OUTS_SM1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM1_FLAG_N_Z_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM1_PositiveZFlag
* Description    : Read SM1_FLAG_P_Z
* Input          : Pointer to LSM330_ACC_SM1_FLAG_P_Z_t
* Output         : Status of SM1_FLAG_P_Z see LSM330_ACC_SM1_FLAG_P_Z_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM1_PositiveZFlag (void *handle, LSM330_ACC_SM1_FLAG_P_Z_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_OUTS_SM1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM1_FLAG_P_Z_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM1_NegativeYFlag
* Description    : Read SM1_FLAG_N_Y
* Input          : Pointer to LSM330_ACC_SM1_FLAG_N_Y_t
* Output         : Status of SM1_FLAG_N_Y see LSM330_ACC_SM1_FLAG_N_Y_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM1_NegativeYFlag (void *handle, LSM330_ACC_SM1_FLAG_N_Y_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_OUTS_SM1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM1_FLAG_N_Y_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM1_PositiveYFlag
* Description    : Read SM1_FLAG_P_Y
* Input          : Pointer to LSM330_ACC_SM1_FLAG_P_Y_t
* Output         : Status of SM1_FLAG_P_Y see LSM330_ACC_SM1_FLAG_P_Y_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM1_PositiveYFlag (void *handle, LSM330_ACC_SM1_FLAG_P_Y_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_OUTS_SM1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM1_FLAG_P_Y_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM1_NegativeXFlag
* Description    : Read SM1_FLAG_N_X
* Input          : Pointer to LSM330_ACC_SM1_FLAG_N_X_t
* Output         : Status of SM1_FLAG_N_X see LSM330_ACC_SM1_FLAG_N_X_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM1_NegativeXFlag (void *handle, LSM330_ACC_SM1_FLAG_N_X_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_OUTS_SM1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM1_FLAG_N_X_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM1_PositiveXFlag
* Description    : Read SM1_FLAG_P_X
* Input          : Pointer to LSM330_ACC_SM1_FLAG_P_X_t
* Output         : Status of SM1_FLAG_P_X see LSM330_ACC_SM1_FLAG_P_X_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM1_PositiveXFlag (void *handle, LSM330_ACC_SM1_FLAG_P_X_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_OUTS_SM1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM1_FLAG_P_X_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_Timer4_SM2
* Description    : Write TIM4_SM2_BIT
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_Timer4_SM2(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM330_ACC_TIM4_SM2_BIT_POSITION; //mask	
  newValue &= LSM330_ACC_TIM4_SM2_BIT_MASK; //coerce
  
  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_TIM4_SM2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_TIM4_SM2_BIT_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_TIM4_SM2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_Timer4_SM2
* Description    : Read TIM4_SM2_BIT
* Input          : Pointer to u8_t
* Output         : Status of TIM4_SM2_BIT 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_Timer4_SM2(void *handle, u8_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_TIM4_SM2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_TIM4_SM2_BIT_MASK; //coerce	
  *value = *value >> LSM330_ACC_TIM4_SM2_BIT_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_Timer3_SM2
* Description    : Write TIM3_SM2_BIT
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_Timer3_SM2(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM330_ACC_TIM3_SM2_BIT_POSITION; //mask	
  newValue &= LSM330_ACC_TIM3_SM2_BIT_MASK; //coerce
  
  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_TIM3_SM2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_TIM3_SM2_BIT_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_TIM3_SM2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_Timer3_SM2
* Description    : Read TIM3_SM2_BIT
* Input          : Pointer to u8_t
* Output         : Status of TIM3_SM2_BIT 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_Timer3_SM2(void *handle, u8_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_TIM3_SM2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_TIM3_SM2_BIT_MASK; //coerce	
  *value = *value >> LSM330_ACC_TIM3_SM2_BIT_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_Threshold2_SM2
* Description    : Write THRS2_SM2_BIT
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_Threshold2_SM2(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM330_ACC_THRS2_SM2_BIT_POSITION; //mask	
  newValue &= LSM330_ACC_THRS2_SM2_BIT_MASK; //coerce
  
  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_THRS2_SM2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_THRS2_SM2_BIT_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_THRS2_SM2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_Threshold2_SM2
* Description    : Read THRS2_SM2_BIT
* Input          : Pointer to u8_t
* Output         : Status of THRS2_SM2_BIT 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_Threshold2_SM2(void *handle, u8_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_THRS2_SM2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_THRS2_SM2_BIT_MASK; //coerce	
  *value = *value >> LSM330_ACC_THRS2_SM2_BIT_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_Threshold1_SM2
* Description    : Write THRS1_SM2_BIT
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_Threshold1_SM2(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM330_ACC_THRS1_SM2_BIT_POSITION; //mask	
  newValue &= LSM330_ACC_THRS1_SM2_BIT_MASK; //coerce
  
  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_THRS1_SM2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_THRS1_SM2_BIT_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_THRS1_SM2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_Threshold1_SM2
* Description    : Read THRS1_SM2_BIT
* Input          : Pointer to u8_t
* Output         : Status of THRS1_SM2_BIT 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_Threshold1_SM2(void *handle, u8_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_THRS1_SM2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_THRS1_SM2_BIT_MASK; //coerce	
  *value = *value >> LSM330_ACC_THRS1_SM2_BIT_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_DecimationCounterSM2
* Description    : Write DES_SM2_BIT
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_DecimationCounterSM2(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM330_ACC_DES_SM2_BIT_POSITION; //mask	
  newValue &= LSM330_ACC_DES_SM2_BIT_MASK; //coerce
  
  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_DES_SM2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_DES_SM2_BIT_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_DES_SM2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_DecimationCounterSM2
* Description    : Read DES_SM2_BIT
* Input          : Pointer to u8_t
* Output         : Status of DES_SM2_BIT 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_DecimationCounterSM2(void *handle, u8_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_DES_SM2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_DES_SM2_BIT_MASK; //coerce	
  *value = *value >> LSM330_ACC_DES_SM2_BIT_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM2_MaskB_NegativeVector
* Description    : Write SM2_MB_N_V
* Input          : LSM330_ACC_SM2_MB_N_V_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM2_MaskB_NegativeVector (void *handle, LSM330_ACC_SM2_MB_N_V_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKB_SM2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM2_MB_N_V_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_MASKB_SM2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM2_MaskB_NegativeVector
* Description    : Read SM2_MB_N_V
* Input          : Pointer to LSM330_ACC_SM2_MB_N_V_t
* Output         : Status of SM2_MB_N_V see LSM330_ACC_SM2_MB_N_V_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM2_MaskB_NegativeVector (void *handle, LSM330_ACC_SM2_MB_N_V_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKB_SM2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM2_MB_N_V_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM2_MaskB_PositiveVector
* Description    : Write SM2_MB_P_V
* Input          : LSM330_ACC_SM2_MB_P_V_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM2_MaskB_PositiveVector (void *handle, LSM330_ACC_SM2_MB_P_V_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKB_SM2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM2_MB_P_V_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_MASKB_SM2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM2_MaskB_PositiveVector
* Description    : Read SM2_MB_P_V
* Input          : Pointer to LSM330_ACC_SM2_MB_P_V_t
* Output         : Status of SM2_MB_P_V see LSM330_ACC_SM2_MB_P_V_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM2_MaskB_PositiveVector (void *handle, LSM330_ACC_SM2_MB_P_V_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKB_SM2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM2_MB_P_V_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM2_MaskB_NegativeZ
* Description    : Write SM2_MB_N_Z
* Input          : LSM330_ACC_SM2_MB_N_Z_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM2_MaskB_NegativeZ (void *handle, LSM330_ACC_SM2_MB_N_Z_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKB_SM2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM2_MB_N_Z_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_MASKB_SM2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM2_MaskB_NegativeZ
* Description    : Read SM2_MB_N_Z
* Input          : Pointer to LSM330_ACC_SM2_MB_N_Z_t
* Output         : Status of SM2_MB_N_Z see LSM330_ACC_SM2_MB_N_Z_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM2_MaskB_NegativeZ (void *handle, LSM330_ACC_SM2_MB_N_Z_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKB_SM2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM2_MB_N_Z_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM2_MaskB_PositiveZ
* Description    : Write SM2_MB_P_Z
* Input          : LSM330_ACC_SM2_MB_P_Z_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM2_MaskB_PositiveZ (void *handle, LSM330_ACC_SM2_MB_P_Z_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKB_SM2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM2_MB_P_Z_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_MASKB_SM2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM2_MaskB_PositiveZ
* Description    : Read SM2_MB_P_Z
* Input          : Pointer to LSM330_ACC_SM2_MB_P_Z_t
* Output         : Status of SM2_MB_P_Z see LSM330_ACC_SM2_MB_P_Z_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM2_MaskB_PositiveZ (void *handle, LSM330_ACC_SM2_MB_P_Z_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKB_SM2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM2_MB_P_Z_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM2_MaskB_NegativeY
* Description    : Write SM2_MB_N_Y
* Input          : LSM330_ACC_SM2_MB_N_Y_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM2_MaskB_NegativeY (void *handle, LSM330_ACC_SM2_MB_N_Y_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKB_SM2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM2_MB_N_Y_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_MASKB_SM2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM2_MaskB_NegativeY
* Description    : Read SM2_MB_N_Y
* Input          : Pointer to LSM330_ACC_SM2_MB_N_Y_t
* Output         : Status of SM2_MB_N_Y see LSM330_ACC_SM2_MB_N_Y_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM2_MaskB_NegativeY (void *handle, LSM330_ACC_SM2_MB_N_Y_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKB_SM2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM2_MB_N_Y_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM2_MaskB_PositiveY
* Description    : Write SM2_MB_P_Y
* Input          : LSM330_ACC_SM2_MB_P_Y_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM2_MaskB_PositiveY (void *handle, LSM330_ACC_SM2_MB_P_Y_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKB_SM2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM2_MB_P_Y_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_MASKB_SM2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM2_MaskB_PositiveY
* Description    : Read SM2_MB_P_Y
* Input          : Pointer to LSM330_ACC_SM2_MB_P_Y_t
* Output         : Status of SM2_MB_P_Y see LSM330_ACC_SM2_MB_P_Y_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM2_MaskB_PositiveY (void *handle, LSM330_ACC_SM2_MB_P_Y_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKB_SM2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM2_MB_P_Y_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM2_MaskB_NegativeX
* Description    : Write SM2_MB_N_X
* Input          : LSM330_ACC_SM2_MB_N_X_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM2_MaskB_NegativeX (void *handle, LSM330_ACC_SM2_MB_N_X_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKB_SM2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM2_MB_N_X_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_MASKB_SM2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM2_MaskB_NegativeX
* Description    : Read SM2_MB_N_X
* Input          : Pointer to LSM330_ACC_SM2_MB_N_X_t
* Output         : Status of SM2_MB_N_X see LSM330_ACC_SM2_MB_N_X_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM2_MaskB_NegativeX (void *handle, LSM330_ACC_SM2_MB_N_X_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKB_SM2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM2_MB_N_X_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM2_MaskB_PositiveX
* Description    : Write SM2_MB_P_X
* Input          : LSM330_ACC_SM2_MB_P_X_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM2_MaskB_PositiveX (void *handle, LSM330_ACC_SM2_MB_P_X_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKB_SM2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM2_MB_P_X_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_MASKB_SM2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM2_MaskB_PositiveX
* Description    : Read SM2_MB_P_X
* Input          : Pointer to LSM330_ACC_SM2_MB_P_X_t
* Output         : Status of SM2_MB_P_X see LSM330_ACC_SM2_MB_P_X_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM2_MaskB_PositiveX (void *handle, LSM330_ACC_SM2_MB_P_X_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKB_SM2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM2_MB_P_X_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM2_MaskA_NegativeVector
* Description    : Write SM2_MA_N_V
* Input          : LSM330_ACC_SM2_MA_N_V_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM2_MaskA_NegativeVector (void *handle, LSM330_ACC_SM2_MA_N_V_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKA_SM2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM2_MA_N_V_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_MASKA_SM2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM2_MaskA_NegativeVector
* Description    : Read SM2_MA_N_V
* Input          : Pointer to LSM330_ACC_SM2_MA_N_V_t
* Output         : Status of SM2_MA_N_V see LSM330_ACC_SM2_MA_N_V_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM2_MaskA_NegativeVector (void *handle, LSM330_ACC_SM2_MA_N_V_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKA_SM2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM2_MA_N_V_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM2_MaskA_PositiveVector
* Description    : Write SM2_MA_P_V
* Input          : LSM330_ACC_SM2_MA_P_V_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM2_MaskA_PositiveVector (void *handle, LSM330_ACC_SM2_MA_P_V_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKA_SM2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM2_MA_P_V_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_MASKA_SM2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM2_MaskA_PositiveVector
* Description    : Read SM2_MA_P_V
* Input          : Pointer to LSM330_ACC_SM2_MA_P_V_t
* Output         : Status of SM2_MA_P_V see LSM330_ACC_SM2_MA_P_V_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM2_MaskA_PositiveVector (void *handle, LSM330_ACC_SM2_MA_P_V_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKA_SM2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM2_MA_P_V_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM2_MaskA_NegativeZ
* Description    : Write SM2_MA_N_Z
* Input          : LSM330_ACC_SM2_MA_N_Z_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM2_MaskA_NegativeZ (void *handle, LSM330_ACC_SM2_MA_N_Z_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKA_SM2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM2_MA_N_Z_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_MASKA_SM2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM2_MaskA_NegativeZ
* Description    : Read SM2_MA_N_Z
* Input          : Pointer to LSM330_ACC_SM2_MA_N_Z_t
* Output         : Status of SM2_MA_N_Z see LSM330_ACC_SM2_MA_N_Z_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM2_MaskA_NegativeZ (void *handle, LSM330_ACC_SM2_MA_N_Z_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKA_SM2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM2_MA_N_Z_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM2_MaskA_PositiveZ
* Description    : Write SM2_MA_P_Z
* Input          : LSM330_ACC_SM2_MA_P_Z_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM2_MaskA_PositiveZ (void *handle, LSM330_ACC_SM2_MA_P_Z_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKA_SM2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM2_MA_P_Z_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_MASKA_SM2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM2_MaskA_PositiveZ
* Description    : Read SM2_MA_P_Z
* Input          : Pointer to LSM330_ACC_SM2_MA_P_Z_t
* Output         : Status of SM2_MA_P_Z see LSM330_ACC_SM2_MA_P_Z_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM2_MaskA_PositiveZ (void *handle, LSM330_ACC_SM2_MA_P_Z_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKA_SM2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM2_MA_P_Z_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM2_MaskA_NegativeY
* Description    : Write SM2_MA_N_Y
* Input          : LSM330_ACC_SM2_MA_N_Y_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM2_MaskA_NegativeY (void *handle, LSM330_ACC_SM2_MA_N_Y_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKA_SM2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM2_MA_N_Y_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_MASKA_SM2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM2_MaskA_NegativeY
* Description    : Read SM2_MA_N_Y
* Input          : Pointer to LSM330_ACC_SM2_MA_N_Y_t
* Output         : Status of SM2_MA_N_Y see LSM330_ACC_SM2_MA_N_Y_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM2_MaskA_NegativeY (void *handle, LSM330_ACC_SM2_MA_N_Y_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKA_SM2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM2_MA_N_Y_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM2_MaskA_PositiveY
* Description    : Write SM2_MA_P_Y
* Input          : LSM330_ACC_SM2_MA_P_Y_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM2_MaskA_PositiveY (void *handle, LSM330_ACC_SM2_MA_P_Y_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKA_SM2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM2_MA_P_Y_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_MASKA_SM2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM2_MaskA_PositiveY
* Description    : Read SM2_MA_P_Y
* Input          : Pointer to LSM330_ACC_SM2_MA_P_Y_t
* Output         : Status of SM2_MA_P_Y see LSM330_ACC_SM2_MA_P_Y_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM2_MaskA_PositiveY (void *handle, LSM330_ACC_SM2_MA_P_Y_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKA_SM2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM2_MA_P_Y_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM2_MaskA_NegativeX
* Description    : Write SM2_MA_N_X
* Input          : LSM330_ACC_SM2_MA_N_X_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM2_MaskA_NegativeX (void *handle, LSM330_ACC_SM2_MA_N_X_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKA_SM2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM2_MA_N_X_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_MASKA_SM2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM2_MaskA_NegativeX
* Description    : Read SM2_MA_N_X
* Input          : Pointer to LSM330_ACC_SM2_MA_N_X_t
* Output         : Status of SM2_MA_N_X see LSM330_ACC_SM2_MA_N_X_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM2_MaskA_NegativeX (void *handle, LSM330_ACC_SM2_MA_N_X_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKA_SM2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM2_MA_N_X_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM2_MaskA_PositiveX
* Description    : Write SM2_MA_P_X
* Input          : LSM330_ACC_SM2_MA_P_X_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM2_MaskA_PositiveX (void *handle, LSM330_ACC_SM2_MA_P_X_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKA_SM2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM2_MA_P_X_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_MASKA_SM2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM2_MaskA_PositiveX
* Description    : Read SM2_MA_P_X
* Input          : Pointer to LSM330_ACC_SM2_MA_P_X_t
* Output         : Status of SM2_MA_P_X see LSM330_ACC_SM2_MA_P_X_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM2_MaskA_PositiveX (void *handle, LSM330_ACC_SM2_MA_P_X_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_MASKA_SM2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM2_MA_P_X_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM2_ProgramFlowModification
* Description    : Write SM2_SITR
* Input          : LSM330_ACC_SM2_SITR_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM2_ProgramFlowModification (void *handle, LSM330_ACC_SM2_SITR_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_SETT_SM2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM2_SITR_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_SETT_SM2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM2_ProgramFlowModification
* Description    : Read SM2_SITR
* Input          : Pointer to LSM330_ACC_SM2_SITR_t
* Output         : Status of SM2_SITR see LSM330_ACC_SM2_SITR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM2_ProgramFlowModification (void *handle, LSM330_ACC_SM2_SITR_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_SETT_SM2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM2_SITR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM2_NextConditionValidation
* Description    : Write SM2_R_TAM
* Input          : LSM330_ACC_SM2_R_TAM_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM2_NextConditionValidation (void *handle, LSM330_ACC_SM2_R_TAM_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_SETT_SM2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM2_R_TAM_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_SETT_SM2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM2_NextConditionValidation
* Description    : Read SM2_R_TAM
* Input          : Pointer to LSM330_ACC_SM2_R_TAM_t
* Output         : Status of SM2_R_TAM see LSM330_ACC_SM2_R_TAM_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM2_NextConditionValidation (void *handle, LSM330_ACC_SM2_R_TAM_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_SETT_SM2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM2_R_TAM_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM2_Threshold3_OnMaskA
* Description    : Write SM2_THR3_MA
* Input          : LSM330_ACC_SM2_THR3_MA_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM2_Threshold3_OnMaskA (void *handle, LSM330_ACC_SM2_THR3_MA_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_SETT_SM2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM2_THR3_MA_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_SETT_SM2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM2_Threshold3_OnMaskA
* Description    : Read SM2_THR3_MA
* Input          : Pointer to LSM330_ACC_SM2_THR3_MA_t
* Output         : Status of SM2_THR3_MA see LSM330_ACC_SM2_THR3_MA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM2_Threshold3_OnMaskA (void *handle, LSM330_ACC_SM2_THR3_MA_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_SETT_SM2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM2_THR3_MA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM2_AbsoluteValue
* Description    : Write SM2_ABS
* Input          : LSM330_ACC_SM2_ABS_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM2_AbsoluteValue (void *handle, LSM330_ACC_SM2_ABS_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_SETT_SM2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM2_ABS_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_SETT_SM2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM2_AbsoluteValue
* Description    : Read SM2_ABS
* Input          : Pointer to LSM330_ACC_SM2_ABS_t
* Output         : Status of SM2_ABS see LSM330_ACC_SM2_ABS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM2_AbsoluteValue (void *handle, LSM330_ACC_SM2_ABS_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_SETT_SM2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM2_ABS_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM2_Threshold3_OnMaskB
* Description    : Write SM2_THR3_SA
* Input          : LSM330_ACC_SM2_THR3_SA_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM2_Threshold3_OnMaskB (void *handle, LSM330_ACC_SM2_THR3_SA_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_SETT_SM2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM2_THR3_SA_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_SETT_SM2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM2_Threshold3_OnMaskB
* Description    : Read SM2_THR3_SA
* Input          : Pointer to LSM330_ACC_SM2_THR3_SA_t
* Output         : Status of SM2_THR3_SA see LSM330_ACC_SM2_THR3_SA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM2_Threshold3_OnMaskB (void *handle, LSM330_ACC_SM2_THR3_SA_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_SETT_SM2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM2_THR3_SA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM2_PeakDetection
* Description    : Write SM2_P_DET
* Input          : LSM330_ACC_SM2_P_DET_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM2_PeakDetection (void *handle, LSM330_ACC_SM2_P_DET_t newValue)
{
  u8_t value;

  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_SETT_SM2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM2_P_DET_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_SETT_SM2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM2_PeakDetection
* Description    : Read SM2_P_DET
* Input          : Pointer to LSM330_ACC_SM2_P_DET_t
* Output         : Status of SM2_P_DET see LSM330_ACC_SM2_P_DET_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM2_PeakDetection (void *handle, LSM330_ACC_SM2_P_DET_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_SETT_SM2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM2_P_DET_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM2_ResetPointer
* Description    : Write SM2_RP
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM2_ResetPointer(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM330_ACC_SM2_RP_POSITION; //mask	
  newValue &= LSM330_ACC_SM2_RP_MASK; //coerce
  
  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_PR_SM2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM2_RP_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_PR_SM2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM2_ResetPointer
* Description    : Read SM2_RP
* Input          : Pointer to u8_t
* Output         : Status of SM2_RP 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM2_ResetPointer(void *handle, u8_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_PR_SM2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM2_RP_MASK; //coerce	
  *value = *value >> LSM330_ACC_SM2_RP_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_W_SM2_ProgramPointer
* Description    : Write SM2_PP
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM330_ACC_W_SM2_ProgramPointer(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM330_ACC_SM2_PP_POSITION; //mask	
  newValue &= LSM330_ACC_SM2_PP_MASK; //coerce
  
  if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_PR_SM2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM330_ACC_SM2_PP_MASK; 
  value |= newValue;
  
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_PR_SM2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM2_ProgramPointer
* Description    : Read SM2_PP
* Input          : Pointer to u8_t
* Output         : Status of SM2_PP 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM2_ProgramPointer(void *handle, u8_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_PR_SM2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM2_PP_MASK; //coerce	
  *value = *value >> LSM330_ACC_SM2_PP_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM2_NegativeVectorFlag
* Description    : Read SM2_FLAG_N_V
* Input          : Pointer to LSM330_ACC_SM2_FLAG_N_V_t
* Output         : Status of SM2_FLAG_N_V see LSM330_ACC_SM2_FLAG_N_V_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM2_NegativeVectorFlag (void *handle, LSM330_ACC_SM2_FLAG_N_V_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_OUTS_SM2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM2_FLAG_N_V_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM1_PositiveVectorFlag
* Description    : Read SM2_FLAG_P_V
* Input          : Pointer to LSM330_ACC_SM2_FLAG_P_V_t
* Output         : Status of SM2_FLAG_P_V see LSM330_ACC_SM2_FLAG_P_V_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM2_PositiveVectorFlag (void *handle, LSM330_ACC_SM2_FLAG_P_V_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_OUTS_SM2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM2_FLAG_P_V_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM2_NegativeZFlag
* Description    : Read SM2_FLAG_N_Z
* Input          : Pointer to LSM330_ACC_SM2_FLAG_N_Z_t
* Output         : Status of SM2_FLAG_N_Z see LSM330_ACC_SM2_FLAG_N_Z_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM2_NegativeZFlag (void *handle, LSM330_ACC_SM2_FLAG_N_Z_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_OUTS_SM2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM2_FLAG_N_Z_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM2_PositiveZFlag
* Description    : Read SM2_FLAG_P_Z
* Input          : Pointer to LSM330_ACC_SM2_FLAG_P_Z_t
* Output         : Status of SM2_FLAG_P_Z see LSM330_ACC_SM2_FLAG_P_Z_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM2_PositiveZFlag (void *handle, LSM330_ACC_SM2_FLAG_P_Z_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_OUTS_SM2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM2_FLAG_P_Z_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM2_NegativeYFlag
* Description    : Read SM2_FLAG_N_Y
* Input          : Pointer to LSM330_ACC_SM2_FLAG_N_Y_t
* Output         : Status of SM2_FLAG_N_Y see LSM330_ACC_SM2_FLAG_N_Y_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM2_NegativeYFlag (void *handle, LSM330_ACC_SM2_FLAG_N_Y_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_OUTS_SM2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM2_FLAG_N_Y_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM2_PositiveYFlag
* Description    : Read SM2_FLAG_P_Y
* Input          : Pointer to LSM330_ACC_SM2_FLAG_P_Y_t
* Output         : Status of SM2_FLAG_P_Y see LSM330_ACC_SM2_FLAG_P_Y_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM2_PositiveYFlag (void *handle, LSM330_ACC_SM2_FLAG_P_Y_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_OUTS_SM2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM2_FLAG_P_Y_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM2_NegativeXFlag
* Description    : Read SM2_FLAG_N_X
* Input          : Pointer to LSM330_ACC_SM2_FLAG_N_X_t
* Output         : Status of SM2_FLAG_N_X see LSM330_ACC_SM2_FLAG_N_X_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM2_NegativeXFlag (void *handle, LSM330_ACC_SM2_FLAG_N_X_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_OUTS_SM2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM2_FLAG_N_X_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_SM2_PositiveXFlag
* Description    : Read SM2_FLAG_P_X
* Input          : Pointer to LSM330_ACC_SM2_FLAG_P_X_t
* Output         : Status of SM2_FLAG_P_X see LSM330_ACC_SM2_FLAG_P_X_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_SM2_PositiveXFlag (void *handle, LSM330_ACC_SM2_FLAG_P_X_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_OUTS_SM2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_SM2_FLAG_P_X_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : status_t LSM330_ACC_Set_LongCounter(void *handle, u8_t *buff) 
* Description    : Set LongCounter data row
* Input          : pointer to [u8_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_Set_LongCounter(void *handle, u8_t *buff) 
{
  u8_t  i;

  for (i=0; i<2;i++ ) 
  {
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_LC_L+i,  &buff[i], 1) )
    return MEMS_ERROR;
  }
  return MEMS_SUCCESS;  
}

/*******************************************************************************
* Function Name  : status_t LSM330_ACC_Get_LongCounter(void *handle, u8_t *buff)
* Description    : Read LongCounter output register
* Input          : pointer to [u8_t]
* Output         : LongCounter buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_Get_LongCounter(void *handle, u8_t *buff) 
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;
  
  numberOfByteForDimension=2/1;

  k=0;
  for (i=0; i<1;i++ ) 
  {
    for (j=0; j<numberOfByteForDimension;j++ )
    {	
      if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_LC_L+k, &buff[k], 1))
        return MEMS_ERROR;
      k++;	
    }
  }

  return MEMS_SUCCESS; 
}

/*******************************************************************************
* Function Name  : status_t LSM330_ACC_Set_VectorFilterCoefficient(void *handle, u8_t *buff) 
* Description    : Set VectorFilterCoefficient data row
* Input          : pointer to [u8_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_Set_VectorFilterCoefficient(void *handle, u8_t *buff) 
{
  u8_t  i;

  for (i=0; i<4;i++ ) 
  {
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_VFC_1+i,  &buff[i], 1) )
    return MEMS_ERROR;
  }
  return MEMS_SUCCESS;  
}

/*******************************************************************************
* Function Name  : status_t LSM330_ACC_Get_VectorFilterCoefficient(void *handle, u8_t *buff)
* Description    : Read VectorFilterCoefficient output register
* Input          : pointer to [u8_t]
* Output         : VectorFilterCoefficient buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_Get_VectorFilterCoefficient(void *handle, u8_t *buff) 
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;
  
  numberOfByteForDimension=4/4;

  k=0;
  for (i=0; i<4;i++ ) 
  {
    for (j=0; j<numberOfByteForDimension;j++ )
    {	
      if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_VFC_1+k, &buff[k], 1))
        return MEMS_ERROR;
      k++;	
    }
  }

  return MEMS_SUCCESS; 
}

/*******************************************************************************
* Function Name  : status_t LSM330_ACC_Set_ProgramSM1(void *handle, u8_t *buff) 
* Description    : Set ProgramSM1 data row
* Input          : pointer to [u8_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_Set_ProgramSM1(void *handle, u8_t *buff) 
{
  u8_t  i;

  for (i=0; i<16;i++ ) 
  {
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_ST1_PR0+i,  &buff[i], 1) )
    return MEMS_ERROR;
  }
  return MEMS_SUCCESS;  
}

/*******************************************************************************
* Function Name  : status_t LSM330_ACC_Get_ProgramSM1(void *handle, u8_t *buff)
* Description    : Read ProgramSM1 output register
* Input          : pointer to [u8_t]
* Output         : ProgramSM1 buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_Get_ProgramSM1(void *handle, u8_t *buff) 
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;
  
  numberOfByteForDimension=16/16;

  k=0;
  for (i=0; i<16;i++ ) 
  {
    for (j=0; j<numberOfByteForDimension;j++ )
    {	
      if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_ST1_PR0+k, &buff[k], 1))
        return MEMS_ERROR;
      k++;	
    }
  }

  return MEMS_SUCCESS; 
}


/*******************************************************************************
* Function Name  : status_t LSM330_ACC_Set_Timer2_SM1(void *handle, u8_t *buff) 
* Description    : Set Timer2_SM1 data row
* Input          : pointer to [u8_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_Set_Timer2_SM1(void *handle, u8_t *buff) 
{
  u8_t  i;

  for (i=0; i<2;i++ ) 
  {
    if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_TIM2_L_SM1+i,  &buff[i], 1) )
      return MEMS_ERROR;
  }
  return MEMS_SUCCESS;  
}

/*******************************************************************************
* Function Name  : status_t LSM330_ACC_Get_Timer2_SM1(void *handle, u8_t *buff)
* Description    : Read Timer2_SM1 output register
* Input          : pointer to [u8_t]
* Output         : Timer2_SM1 buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_Get_Timer2_SM1(void *handle, u8_t *buff) 
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;
  
  numberOfByteForDimension=2/1;

  k=0;
  for (i=0; i<1;i++ ) 
  {
    for (j=0; j<numberOfByteForDimension;j++ )
    {	
      if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_TIM2_L_SM1+k, &buff[k], 1))
        return MEMS_ERROR;
      k++;	
    }
  }

  return MEMS_SUCCESS; 
}


/*******************************************************************************
* Function Name  : status_t LSM330_ACC_Set_Timer1_SM1(void *handle, u8_t *buff) 
* Description    : Set Timer1_SM1 data row
* Input          : pointer to [u8_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_Set_Timer1_SM1(void *handle, u8_t *buff) 
{
  u8_t  i;

  for (i=0; i<2;i++ ) 
  {
    if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_TIM1_L_SM1+i,  &buff[i], 1) )
      return MEMS_ERROR;
  }
  return MEMS_SUCCESS;  
}

/*******************************************************************************
* Function Name  : status_t LSM330_ACC_Get_Timer1_SM1(void *handle, u8_t *buff)
* Description    : Read Timer1_SM1 output register
* Input          : pointer to [u8_t]
* Output         : Timer1_SM1 buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_Get_Timer1_SM1(void *handle, u8_t *buff) 
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;
  
  numberOfByteForDimension=2/1;

  k=0;
  for (i=0; i<1;i++ ) 
  {
    for (j=0; j<numberOfByteForDimension;j++ )
    {	
      if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_TIM1_L_SM1+k, &buff[k], 1))
        return MEMS_ERROR;
      k++;	
    }
  }

  return MEMS_SUCCESS; 
}

/*******************************************************************************
* Function Name  : status_t LSM330_ACC_Get_CounterStatus_SM1(void *handle, u8_t *buff)
* Description    : Read CounterStatus_SM1 output register
* Input          : pointer to [u8_t]
* Output         : CounterStatus_SM1 buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_Get_CounterStatus_SM1(void *handle, u8_t *buff) 
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;
  
  numberOfByteForDimension=2/1;

  k=0;
  for (i=0; i<1;i++ ) 
  {
    for (j=0; j<numberOfByteForDimension;j++ )
    {	
      if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_TC1_L_SM1+k, &buff[k], 1))
        return MEMS_ERROR;
      k++;	
    }
  }

  return MEMS_SUCCESS; 
}


/*******************************************************************************
* Function Name  : status_t LSM330_ACC_Set_ProgramSM2(void *handle, u8_t *buff) 
* Description    : Set ProgramSM2 data row
* Input          : pointer to [u8_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_Set_ProgramSM2(void *handle, u8_t *buff) 
{
  u8_t  i;

  for (i=0; i<16;i++ ) 
  {
    if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_ST2_PR0+i,  &buff[i], 1) )
      return MEMS_ERROR;
  }
  return MEMS_SUCCESS;  
}

/*******************************************************************************
* Function Name  : status_t LSM330_ACC_Get_ProgramSM2(void *handle, u8_t *buff)
* Description    : Read ProgramSM2 output register
* Input          : pointer to [u8_t]
* Output         : ProgramSM2 buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_Get_ProgramSM2(void *handle, u8_t *buff) 
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;
  
  numberOfByteForDimension=16/16;

  k=0;
  for (i=0; i<16;i++ ) 
  {
    for (j=0; j<numberOfByteForDimension;j++ )
    {	
      if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_ST2_PR0+k, &buff[k], 1))
        return MEMS_ERROR;
      k++;	
    }
  }

  return MEMS_SUCCESS; 
}

/*******************************************************************************
* Function Name  : status_t LSM330_ACC_Set_ Timer2_SM2(void *handle, u8_t *buff) 
* Description    : Set  Timer2_SM2 data row
* Input          : pointer to [u8_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_Set_Timer2_SM2(void *handle, u8_t *buff) 
{
  u8_t  i;

  for (i=0; i<2;i++ ) 
  {
    if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_TIM2_L_SM2+i,  &buff[i], 1) )
      return MEMS_ERROR;
  }
  return MEMS_SUCCESS;  
}

/*******************************************************************************
* Function Name  : status_t LSM330_ACC_Get_ Timer2_SM2(void *handle, u8_t *buff)
* Description    : Read  Timer2_SM2 output register
* Input          : pointer to [u8_t]
* Output         :  Timer2_SM2 buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_Get_Timer2_SM2(void *handle, u8_t *buff) 
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;
  
  numberOfByteForDimension=2/1;

  k=0;
  for (i=0; i<1;i++ ) 
  {
    for (j=0; j<numberOfByteForDimension;j++ )
    {	
      if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_TIM2_L_SM2+k, &buff[k], 1))
        return MEMS_ERROR;
      k++;	
    }
  }

  return MEMS_SUCCESS; 
}

/*******************************************************************************
* Function Name  : status_t LSM330_ACC_Set_ Timer1_SM2(void *handle, u8_t *buff) 
* Description    : Set  Timer1_SM2 data row
* Input          : pointer to [u8_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_Set_Timer1_SM2(void *handle, u8_t *buff) 
{
  u8_t  i;

  for (i=0; i<2;i++ ) 
  {
  if( !LSM330_ACC_WriteReg(handle, LSM330_ACC_TIM1_L_SM2+i,  &buff[i], 1) )
    return MEMS_ERROR;
  }
  return MEMS_SUCCESS;  
}

/*******************************************************************************
* Function Name  : status_t LSM330_ACC_Get_ Timer1_SM2(void *handle, u8_t *buff)
* Description    : Read  Timer1_SM2 output register
* Input          : pointer to [u8_t]
* Output         :  Timer1_SM2 buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_Get_Timer1_SM2(void *handle, u8_t *buff) 
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;
  
  numberOfByteForDimension=2/1;

  k=0;
  for (i=0; i<1;i++ ) 
  {
    for (j=0; j<numberOfByteForDimension;j++ )
    {	
      if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_TIM1_L_SM2+k, &buff[k], 1))
        return MEMS_ERROR;
      k++;	
    }
  }
  return MEMS_SUCCESS; 
}

/*******************************************************************************
* Function Name  : status_t LSM330_ACC_Get_CounterStatus_SM2(void *handle, u8_t *buff)
* Description    : Read CounterStatus_SM2 output register
* Input          : pointer to [u8_t]
* Output         : CounterStatus_SM2 buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_Get_CounterStatus_SM2(void *handle, u8_t *buff) 
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;
  
  numberOfByteForDimension=2/1;

  k=0;
  for (i=0; i<1;i++ ) 
  {
    for (j=0; j<numberOfByteForDimension;j++ )
    {	
      if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_TC_L_SM2+k, &buff[k], 1))
        return MEMS_ERROR;
      k++;	
    }
  }

  return MEMS_SUCCESS; 
}

/*******************************************************************************
* Function Name  : LSM330_ACC_R_Temperature
* Description    : Read OUT_T_BYTE
* Input          : Pointer to u8_t
* Output         : Status of OUT_T_BYTE 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM330_ACC_R_Temperature(void *handle, u8_t *value)
{
 if( !LSM330_ACC_ReadReg( handle, LSM330_ACC_OUT_T, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM330_ACC_OUT_T_BYTE_MASK; //coerce	
  *value = *value >> LSM330_ACC_OUT_T_BYTE_POSITION; //mask	

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
void LSM330_ACC_SwapHighLowByte(u8_t *bufferToSwap, u8_t numberOfByte, u8_t dimension)
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
