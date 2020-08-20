/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
* File Name          : A3G4250D_GYRO_driver.c
* Author             : MEMS Application Team
* Version            : v1.1
* Date               : 31 Mar 2016  
* Description        : A3G4250D source driver file
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
#include "A3G4250D_GYRO_driver.h"
//#include "i2C_mems.h"												//[Example]

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/************** Generic Function  *******************/

/*******************************************************************************
* Function Name		: A3G4250D_GYRO_WriteReg
* Description		: Generic Writing function. It must be fullfilled with either
*					: I2C or SPI writing function
* Input				: Register Address, ptr to buffer to be written,
*                                 length of buffer
* Output			: None
* Return			: None
*******************************************************************************/
status_t A3G4250D_GYRO_WriteReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len)
{
  //To be completed with either I2c or SPI writing function
  //i.e.: SPI_Mems_Write_Reg(Reg, Data);
  //I2Cx_Write(Bufp, A3G4250D_GYRO_I2C_ADDRESS, Reg, len);
  return MEMS_SUCCESS;                                             //[Example]
}

/*******************************************************************************
* Function Name		: A3G4250D_GYRO_ReadReg
* Description		: Generic Reading function. It must be fullfilled with either
*					: I2C or SPI writing function
* Input				: Register Address, ptr to buffer to be read,
*                                 length of buffer
* Output			: None
* Return			: None
*******************************************************************************/
status_t A3G4250D_GYRO_ReadReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len)
{
  //To be completed with either I2c or SPI writing function
  //i.e.: SPI_Mems_Write_Reg(Reg, Data);
  //I2Cx_Read(Bufp, A3G4250D_GYRO_I2C_ADDRESS, Reg, len);
  return MEMS_SUCCESS;                                             //[Example]
}

/**************** Base Function  *******************/

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_WHO_AM_I_BIT_bits
* Description    : Read WHO_AM_I_BIT
* Input          : Pointer to u8_t
* Output         : Status of WHO_AM_I_BIT 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_WHO_AM_I_BIT_bits(void *handle, u8_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_WHO_AM_I, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_WHO_AM_I_BIT_MASK; //coerce	
  *value = *value >> A3G4250D_GYRO_WHO_AM_I_BIT_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : status_t A3G4250D_GYRO_Get_AngularRate(u8_t *buff)
* Description    : Read AngularRate output register
* Input          : pointer to [u8_t]
* Output         : AngularRate buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_Get_AngularRate(void *handle, u8_t *buff) 
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;
  
  numberOfByteForDimension=6/3;

  k=0;
  for (i=0; i<3;i++ ) 
  {
	for (j=0; j<numberOfByteForDimension;j++ )
	{	
		if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_OUT_X_L+k, &buff[k], 1))
		  return MEMS_ERROR;
		k++;	
	}
  }

  return MEMS_SUCCESS; 
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_W_DR_bits
* Description    : Write DR
* Input          : A3G4250D_GYRO_DR_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  A3G4250D_GYRO_W_DR_bits(void *handle, A3G4250D_GYRO_DR_t newValue)
{
  u8_t value;

  if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  value &= ~A3G4250D_GYRO_DR_MASK; 
  value |= newValue;
  
  if( !A3G4250D_GYRO_WriteReg(handle, A3G4250D_GYRO_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_DR_bits
* Description    : Read DR
* Input          : Pointer to A3G4250D_GYRO_DR_t
* Output         : Status of DR see A3G4250D_GYRO_DR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_DR_bits(void *handle, A3G4250D_GYRO_DR_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_DR_MASK; //mask

  return MEMS_SUCCESS;
}

/**************** Advanced Function  *******************/

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
void A3G4250D_GYRO_SwapHighLowByte(u8_t *bufferToSwap, u8_t numberOfByte, u8_t dimension)
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

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_W_XEN_bits
* Description    : Write XEN
* Input          : A3G4250D_GYRO_XEN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  A3G4250D_GYRO_W_XEN_bits(void *handle, A3G4250D_GYRO_XEN_t newValue)
{
  u8_t value;

  if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  value &= ~A3G4250D_GYRO_XEN_MASK; 
  value |= newValue;
  
  if( !A3G4250D_GYRO_WriteReg(handle, A3G4250D_GYRO_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_XEN_bits
* Description    : Read XEN
* Input          : Pointer to A3G4250D_GYRO_XEN_t
* Output         : Status of XEN see A3G4250D_GYRO_XEN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_XEN_bits(void *handle, A3G4250D_GYRO_XEN_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_XEN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_W_YEN_bits
* Description    : Write YEN
* Input          : A3G4250D_GYRO_YEN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  A3G4250D_GYRO_W_YEN_bits(void *handle, A3G4250D_GYRO_YEN_t newValue)
{
  u8_t value;

  if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  value &= ~A3G4250D_GYRO_YEN_MASK; 
  value |= newValue;
  
  if( !A3G4250D_GYRO_WriteReg(handle, A3G4250D_GYRO_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_YEN_bits
* Description    : Read YEN
* Input          : Pointer to A3G4250D_GYRO_YEN_t
* Output         : Status of YEN see A3G4250D_GYRO_YEN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_YEN_bits(void *handle, A3G4250D_GYRO_YEN_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_YEN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_W_ZEN_bits
* Description    : Write ZEN
* Input          : A3G4250D_GYRO_ZEN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  A3G4250D_GYRO_W_ZEN_bits(void *handle, A3G4250D_GYRO_ZEN_t newValue)
{
  u8_t value;

  if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  value &= ~A3G4250D_GYRO_ZEN_MASK; 
  value |= newValue;
  
  if( !A3G4250D_GYRO_WriteReg(handle, A3G4250D_GYRO_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_ZEN_bits
* Description    : Read ZEN
* Input          : Pointer to A3G4250D_GYRO_ZEN_t
* Output         : Status of ZEN see A3G4250D_GYRO_ZEN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_ZEN_bits(void *handle, A3G4250D_GYRO_ZEN_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_ZEN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_W_PD_bits
* Description    : Write PD
* Input          : A3G4250D_GYRO_PD_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  A3G4250D_GYRO_W_PD_bits(void *handle, A3G4250D_GYRO_PD_t newValue)
{
  u8_t value;

  if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  value &= ~A3G4250D_GYRO_PD_MASK; 
  value |= newValue;
  
  if( !A3G4250D_GYRO_WriteReg(handle, A3G4250D_GYRO_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_PD_bits
* Description    : Read PD
* Input          : Pointer to A3G4250D_GYRO_PD_t
* Output         : Status of PD see A3G4250D_GYRO_PD_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_PD_bits(void *handle, A3G4250D_GYRO_PD_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_PD_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_W_BW_bits
* Description    : Write BW
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  A3G4250D_GYRO_W_BW_bits(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << A3G4250D_GYRO_BW_POSITION; //mask	
  newValue &= A3G4250D_GYRO_BW_MASK; //coerce
  
  if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  value &= ~A3G4250D_GYRO_BW_MASK; 
  value |= newValue;
  
  if( !A3G4250D_GYRO_WriteReg(handle, A3G4250D_GYRO_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_BW_bits
* Description    : Read BW
* Input          : Pointer to u8_t
* Output         : Status of BW 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_BW_bits(void *handle, u8_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_BW_MASK; //coerce	
  *value = *value >> A3G4250D_GYRO_BW_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_W_HPCF_bits
* Description    : Write HPCF
* Input          : A3G4250D_GYRO_HPCF_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  A3G4250D_GYRO_W_HPCF_bits(void *handle, A3G4250D_GYRO_HPCF_t newValue)
{
  u8_t value;

  if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;

  value &= ~A3G4250D_GYRO_HPCF_MASK; 
  value |= newValue;
  
  if( !A3G4250D_GYRO_WriteReg(handle, A3G4250D_GYRO_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_HPCF_bits
* Description    : Read HPCF
* Input          : Pointer to A3G4250D_GYRO_HPCF_t
* Output         : Status of HPCF see A3G4250D_GYRO_HPCF_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_HPCF_bits(void *handle, A3G4250D_GYRO_HPCF_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_HPCF_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_W_HPM_bits
* Description    : Write HPM
* Input          : A3G4250D_GYRO_HPM_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  A3G4250D_GYRO_W_HPM_bits(void *handle, A3G4250D_GYRO_HPM_t newValue)
{
  u8_t value;

  if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;

  value &= ~A3G4250D_GYRO_HPM_MASK; 
  value |= newValue;
  
  if( !A3G4250D_GYRO_WriteReg(handle, A3G4250D_GYRO_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_HPM_bits
* Description    : Read HPM
* Input          : Pointer to A3G4250D_GYRO_HPM_t
* Output         : Status of HPM see A3G4250D_GYRO_HPM_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_HPM_bits(void *handle, A3G4250D_GYRO_HPM_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_HPM_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_W_I2_EMPTY_bits
* Description    : Write I2_EMPTY
* Input          : A3G4250D_GYRO_I2_EMPTY_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  A3G4250D_GYRO_W_I2_EMPTY_bits(void *handle, A3G4250D_GYRO_I2_EMPTY_t newValue)
{
  u8_t value;

  if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  value &= ~A3G4250D_GYRO_I2_EMPTY_MASK; 
  value |= newValue;
  
  if( !A3G4250D_GYRO_WriteReg(handle, A3G4250D_GYRO_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_I2_EMPTY_bits
* Description    : Read I2_EMPTY
* Input          : Pointer to A3G4250D_GYRO_I2_EMPTY_t
* Output         : Status of I2_EMPTY see A3G4250D_GYRO_I2_EMPTY_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_I2_EMPTY_bits(void *handle, A3G4250D_GYRO_I2_EMPTY_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG3, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_I2_EMPTY_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_W_I2_ORUN_bits
* Description    : Write I2_ORUN
* Input          : A3G4250D_GYRO_I2_ORUN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  A3G4250D_GYRO_W_I2_ORUN_bits(void *handle, A3G4250D_GYRO_I2_ORUN_t newValue)
{
  u8_t value;

  if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  value &= ~A3G4250D_GYRO_I2_ORUN_MASK; 
  value |= newValue;
  
  if( !A3G4250D_GYRO_WriteReg(handle, A3G4250D_GYRO_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_I2_ORUN_bits
* Description    : Read I2_ORUN
* Input          : Pointer to A3G4250D_GYRO_I2_ORUN_t
* Output         : Status of I2_ORUN see A3G4250D_GYRO_I2_ORUN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_I2_ORUN_bits(void *handle, A3G4250D_GYRO_I2_ORUN_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG3, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_I2_ORUN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_W_I2_WTM_bits
* Description    : Write I2_WTM
* Input          : A3G4250D_GYRO_I2_WTM_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  A3G4250D_GYRO_W_I2_WTM_bits(void *handle, A3G4250D_GYRO_I2_WTM_t newValue)
{
  u8_t value;

  if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  value &= ~A3G4250D_GYRO_I2_WTM_MASK; 
  value |= newValue;
  
  if( !A3G4250D_GYRO_WriteReg(handle, A3G4250D_GYRO_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_I2_WTM_bits
* Description    : Read I2_WTM
* Input          : Pointer to A3G4250D_GYRO_I2_WTM_t
* Output         : Status of I2_WTM see A3G4250D_GYRO_I2_WTM_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_I2_WTM_bits(void *handle, A3G4250D_GYRO_I2_WTM_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG3, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_I2_WTM_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_W_I2_DRDY_bits
* Description    : Write I2_DRDY
* Input          : A3G4250D_GYRO_I2_DRDY_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  A3G4250D_GYRO_W_I2_DRDY_bits(void *handle, A3G4250D_GYRO_I2_DRDY_t newValue)
{
  u8_t value;

  if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  value &= ~A3G4250D_GYRO_I2_DRDY_MASK; 
  value |= newValue;
  
  if( !A3G4250D_GYRO_WriteReg(handle, A3G4250D_GYRO_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_I2_DRDY_bits
* Description    : Read I2_DRDY
* Input          : Pointer to A3G4250D_GYRO_I2_DRDY_t
* Output         : Status of I2_DRDY see A3G4250D_GYRO_I2_DRDY_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_I2_DRDY_bits(void *handle, A3G4250D_GYRO_I2_DRDY_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG3, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_I2_DRDY_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_W_PP_OD_bits
* Description    : Write PP_OD
* Input          : A3G4250D_GYRO_PP_OD_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  A3G4250D_GYRO_W_PP_OD_bits(void *handle, A3G4250D_GYRO_PP_OD_t newValue)
{
  u8_t value;

  if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  value &= ~A3G4250D_GYRO_PP_OD_MASK; 
  value |= newValue;
  
  if( !A3G4250D_GYRO_WriteReg(handle, A3G4250D_GYRO_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_PP_OD_bits
* Description    : Read PP_OD
* Input          : Pointer to A3G4250D_GYRO_PP_OD_t
* Output         : Status of PP_OD see A3G4250D_GYRO_PP_OD_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_PP_OD_bits(void *handle, A3G4250D_GYRO_PP_OD_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG3, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_PP_OD_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_W_H_LACTIVE_bits
* Description    : Write H_LACTIVE
* Input          : A3G4250D_GYRO_H_LACTIVE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  A3G4250D_GYRO_W_H_LACTIVE_bits(void *handle, A3G4250D_GYRO_H_LACTIVE_t newValue)
{
  u8_t value;

  if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  value &= ~A3G4250D_GYRO_H_LACTIVE_MASK; 
  value |= newValue;
  
  if( !A3G4250D_GYRO_WriteReg(handle, A3G4250D_GYRO_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_H_LACTIVE_bits
* Description    : Read H_LACTIVE
* Input          : Pointer to A3G4250D_GYRO_H_LACTIVE_t
* Output         : Status of H_LACTIVE see A3G4250D_GYRO_H_LACTIVE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_H_LACTIVE_bits(void *handle, A3G4250D_GYRO_H_LACTIVE_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG3, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_H_LACTIVE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_W_I1_BOOT_bits
* Description    : Write I1_BOOT
* Input          : A3G4250D_GYRO_I1_BOOT_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  A3G4250D_GYRO_W_I1_BOOT_bits(void *handle, A3G4250D_GYRO_I1_BOOT_t newValue)
{
  u8_t value;

  if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  value &= ~A3G4250D_GYRO_I1_BOOT_MASK; 
  value |= newValue;
  
  if( !A3G4250D_GYRO_WriteReg(handle, A3G4250D_GYRO_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_I1_BOOT_bits
* Description    : Read I1_BOOT
* Input          : Pointer to A3G4250D_GYRO_I1_BOOT_t
* Output         : Status of I1_BOOT see A3G4250D_GYRO_I1_BOOT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_I1_BOOT_bits(void *handle, A3G4250D_GYRO_I1_BOOT_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG3, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_I1_BOOT_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_W_I1_INT1_bits
* Description    : Write I1_INT1
* Input          : A3G4250D_GYRO_I1_INT1_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  A3G4250D_GYRO_W_I1_INT1_bits(void *handle, A3G4250D_GYRO_I1_INT1_t newValue)
{
  u8_t value;

  if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  value &= ~A3G4250D_GYRO_I1_INT1_MASK; 
  value |= newValue;
  
  if( !A3G4250D_GYRO_WriteReg(handle, A3G4250D_GYRO_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_I1_INT1_bits
* Description    : Read I1_INT1
* Input          : Pointer to A3G4250D_GYRO_I1_INT1_t
* Output         : Status of I1_INT1 see A3G4250D_GYRO_I1_INT1_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_I1_INT1_bits(void *handle, A3G4250D_GYRO_I1_INT1_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG3, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_I1_INT1_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_W_SIM_bits
* Description    : Write SIM
* Input          : A3G4250D_GYRO_SIM_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  A3G4250D_GYRO_W_SIM_bits(void *handle, A3G4250D_GYRO_SIM_t newValue)
{
  u8_t value;

  if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;

  value &= ~A3G4250D_GYRO_SIM_MASK; 
  value |= newValue;
  
  if( !A3G4250D_GYRO_WriteReg(handle, A3G4250D_GYRO_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_SIM_bits
* Description    : Read SIM
* Input          : Pointer to A3G4250D_GYRO_SIM_t
* Output         : Status of SIM see A3G4250D_GYRO_SIM_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_SIM_bits(void *handle, A3G4250D_GYRO_SIM_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG4, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_SIM_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_W_ST_bits
* Description    : Write ST
* Input          : A3G4250D_GYRO_ST_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  A3G4250D_GYRO_W_ST_bits(void *handle, A3G4250D_GYRO_ST_t newValue)
{
  u8_t value;

  if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;

  value &= ~A3G4250D_GYRO_ST_MASK; 
  value |= newValue;
  
  if( !A3G4250D_GYRO_WriteReg(handle, A3G4250D_GYRO_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_ST_bits
* Description    : Read ST
* Input          : Pointer to A3G4250D_GYRO_ST_t
* Output         : Status of ST see A3G4250D_GYRO_ST_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_ST_bits(void *handle, A3G4250D_GYRO_ST_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG4, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_ST_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_W_BLE_bits
* Description    : Write BLE
* Input          : A3G4250D_GYRO_BLE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  A3G4250D_GYRO_W_BLE_bits(void *handle, A3G4250D_GYRO_BLE_t newValue)
{
  u8_t value;

  if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;

  value &= ~A3G4250D_GYRO_BLE_MASK; 
  value |= newValue;
  
  if( !A3G4250D_GYRO_WriteReg(handle, A3G4250D_GYRO_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_BLE_bits
* Description    : Read BLE
* Input          : Pointer to A3G4250D_GYRO_BLE_t
* Output         : Status of BLE see A3G4250D_GYRO_BLE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_BLE_bits(void *handle, A3G4250D_GYRO_BLE_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG4, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_BLE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_W_OUT_SEL_bits
* Description    : Write OUT_SEL
* Input          : A3G4250D_GYRO_OUT_SEL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  A3G4250D_GYRO_W_OUT_SEL_bits(void *handle, A3G4250D_GYRO_OUT_SEL_t newValue)
{
  u8_t value;

  if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG5, &value, 1) )
    return MEMS_ERROR;

  value &= ~A3G4250D_GYRO_OUT_SEL_MASK; 
  value |= newValue;
  
  if( !A3G4250D_GYRO_WriteReg(handle, A3G4250D_GYRO_CTRL_REG5, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_OUT_SEL_bits
* Description    : Read OUT_SEL
* Input          : Pointer to A3G4250D_GYRO_OUT_SEL_t
* Output         : Status of OUT_SEL see A3G4250D_GYRO_OUT_SEL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_OUT_SEL_bits(void *handle, A3G4250D_GYRO_OUT_SEL_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG5, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_OUT_SEL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_W_INT1_SEL_bits
* Description    : Write INT1_SEL
* Input          : A3G4250D_GYRO_INT1_SEL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  A3G4250D_GYRO_W_INT1_SEL_bits(void *handle, A3G4250D_GYRO_INT1_SEL_t newValue)
{
  u8_t value;

  if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG5, &value, 1) )
    return MEMS_ERROR;

  value &= ~A3G4250D_GYRO_INT1_SEL_MASK; 
  value |= newValue;
  
  if( !A3G4250D_GYRO_WriteReg(handle, A3G4250D_GYRO_CTRL_REG5, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_INT1_SEL_bits
* Description    : Read INT1_SEL
* Input          : Pointer to A3G4250D_GYRO_INT1_SEL_t
* Output         : Status of INT1_SEL see A3G4250D_GYRO_INT1_SEL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_INT1_SEL_bits(void *handle, A3G4250D_GYRO_INT1_SEL_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG5, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_INT1_SEL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_W_HPEN_bits
* Description    : Write HPEN
* Input          : A3G4250D_GYRO_HPEN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  A3G4250D_GYRO_W_HPEN_bits(void *handle, A3G4250D_GYRO_HPEN_t newValue)
{
  u8_t value;

  if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG5, &value, 1) )
    return MEMS_ERROR;

  value &= ~A3G4250D_GYRO_HPEN_MASK; 
  value |= newValue;
  
  if( !A3G4250D_GYRO_WriteReg(handle, A3G4250D_GYRO_CTRL_REG5, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_HPEN_bits
* Description    : Read HPEN
* Input          : Pointer to A3G4250D_GYRO_HPEN_t
* Output         : Status of HPEN see A3G4250D_GYRO_HPEN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_HPEN_bits(void *handle, A3G4250D_GYRO_HPEN_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG5, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_HPEN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_W_FIFO_EN_bits
* Description    : Write FIFO_EN
* Input          : A3G4250D_GYRO_FIFO_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  A3G4250D_GYRO_W_FIFO_EN_bits(void *handle, A3G4250D_GYRO_FIFO_EN_t newValue)
{
  u8_t value;

  if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG5, &value, 1) )
    return MEMS_ERROR;

  value &= ~A3G4250D_GYRO_FIFO_EN_MASK; 
  value |= newValue;
  
  if( !A3G4250D_GYRO_WriteReg(handle, A3G4250D_GYRO_CTRL_REG5, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_FIFO_EN_bits
* Description    : Read FIFO_EN
* Input          : Pointer to A3G4250D_GYRO_FIFO_EN_t
* Output         : Status of FIFO_EN see A3G4250D_GYRO_FIFO_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_FIFO_EN_bits(void *handle, A3G4250D_GYRO_FIFO_EN_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG5, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_FIFO_EN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_W_BOOT_bits
* Description    : Write BOOT
* Input          : A3G4250D_GYRO_BOOT_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  A3G4250D_GYRO_W_BOOT_bits(void *handle, A3G4250D_GYRO_BOOT_t newValue)
{
  u8_t value;

  if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG5, &value, 1) )
    return MEMS_ERROR;

  value &= ~A3G4250D_GYRO_BOOT_MASK; 
  value |= newValue;
  
  if( !A3G4250D_GYRO_WriteReg(handle, A3G4250D_GYRO_CTRL_REG5, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_BOOT_bits
* Description    : Read BOOT
* Input          : Pointer to A3G4250D_GYRO_BOOT_t
* Output         : Status of BOOT see A3G4250D_GYRO_BOOT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_BOOT_bits(void *handle, A3G4250D_GYRO_BOOT_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_CTRL_REG5, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_BOOT_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_XDA_bits
* Description    : Read XDA
* Input          : Pointer to A3G4250D_GYRO_XDA_t
* Output         : Status of XDA see A3G4250D_GYRO_XDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_XDA_bits(void *handle, A3G4250D_GYRO_XDA_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_XDA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_YDA_bits
* Description    : Read YDA
* Input          : Pointer to A3G4250D_GYRO_YDA_t
* Output         : Status of YDA see A3G4250D_GYRO_YDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_YDA_bits(void *handle, A3G4250D_GYRO_YDA_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_YDA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_ZDA_bits
* Description    : Read ZDA
* Input          : Pointer to A3G4250D_GYRO_ZDA_t
* Output         : Status of ZDA see A3G4250D_GYRO_ZDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_ZDA_bits(void *handle, A3G4250D_GYRO_ZDA_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_ZDA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_ZYXDA_bits
* Description    : Read ZYXDA
* Input          : Pointer to A3G4250D_GYRO_ZYXDA_t
* Output         : Status of ZYXDA see A3G4250D_GYRO_ZYXDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_ZYXDA_bits(void *handle, A3G4250D_GYRO_ZYXDA_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_ZYXDA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_XOR_bits
* Description    : Read XOR
* Input          : Pointer to A3G4250D_GYRO_XOR_t
* Output         : Status of XOR see A3G4250D_GYRO_XOR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_XOR_bits(void *handle, A3G4250D_GYRO_XOR_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_XOR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_YOR_bits
* Description    : Read YOR
* Input          : Pointer to A3G4250D_GYRO_YOR_t
* Output         : Status of YOR see A3G4250D_GYRO_YOR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_YOR_bits(void *handle, A3G4250D_GYRO_YOR_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_YOR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_ZOR_bits
* Description    : Read ZOR
* Input          : Pointer to A3G4250D_GYRO_ZOR_t
* Output         : Status of ZOR see A3G4250D_GYRO_ZOR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_ZOR_bits(void *handle, A3G4250D_GYRO_ZOR_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_ZOR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_ZYXOR_bits
* Description    : Read ZYXOR
* Input          : Pointer to A3G4250D_GYRO_ZYXOR_t
* Output         : Status of ZYXOR see A3G4250D_GYRO_ZYXOR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_ZYXOR_bits(void *handle, A3G4250D_GYRO_ZYXOR_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_ZYXOR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_W_WTM_bits
* Description    : Write WTM
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  A3G4250D_GYRO_W_THSD_WTM_bits(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << A3G4250D_GYRO_THSD_WTM_POSITION; //mask	
  newValue &= A3G4250D_GYRO_THSD_WTM_MASK; //coerce
  
  if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_FIFO_CTRL_REG, &value, 1) )
    return MEMS_ERROR;

  value &= ~A3G4250D_GYRO_WTM_MASK; 
  value |= newValue;
  
  if( !A3G4250D_GYRO_WriteReg(handle, A3G4250D_GYRO_FIFO_CTRL_REG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_WTM_bits
* Description    : Read WTM
* Input          : Pointer to u8_t
* Output         : Status of WTM 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_THSD_WTM_bits(void *handle, u8_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_FIFO_CTRL_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_THSD_WTM_MASK; //coerce	
  *value = *value >> A3G4250D_GYRO_THSD_WTM_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_W_FM_bits
* Description    : Write FM
* Input          : A3G4250D_GYRO_FM_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  A3G4250D_GYRO_W_FM_bits(void *handle, A3G4250D_GYRO_FM_t newValue)
{
  u8_t value;

  if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_FIFO_CTRL_REG, &value, 1) )
    return MEMS_ERROR;

  value &= ~A3G4250D_GYRO_FM_MASK; 
  value |= newValue;
  
  if( !A3G4250D_GYRO_WriteReg(handle, A3G4250D_GYRO_FIFO_CTRL_REG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_FM_bits
* Description    : Read FM
* Input          : Pointer to A3G4250D_GYRO_FM_t
* Output         : Status of FM see A3G4250D_GYRO_FM_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_FM_bits(void *handle, A3G4250D_GYRO_FM_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_FIFO_CTRL_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_FM_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_FSS_bits
* Description    : Read FSS
* Input          : Pointer to u8_t
* Output         : Status of FSS 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_FSS_bits(void *handle, u8_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_FIFO_SRC_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_FSS_MASK; //coerce	
  *value = *value >> A3G4250D_GYRO_FSS_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_EMPTY_bits
* Description    : Read EMPTY
* Input          : Pointer to A3G4250D_GYRO_EMPTY_t
* Output         : Status of EMPTY see A3G4250D_GYRO_EMPTY_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_EMPTY_bits(void *handle, A3G4250D_GYRO_EMPTY_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_FIFO_SRC_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_EMPTY_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_OVRN_bits
* Description    : Read OVRN
* Input          : Pointer to A3G4250D_GYRO_OVRN_t
* Output         : Status of OVRN see A3G4250D_GYRO_OVRN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_OVRN_bits(void *handle, A3G4250D_GYRO_OVRN_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_FIFO_SRC_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_OVRN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_WTM_bits
* Description    : Read WTM
* Input          : Pointer to A3G4250D_GYRO_WTM_t
* Output         : Status of WTM see A3G4250D_GYRO_WTM_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_WTM_bits(void *handle, A3G4250D_GYRO_WTM_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_FIFO_SRC_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_WTM_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_W_XLIE_bits
* Description    : Write XLIE
* Input          : A3G4250D_GYRO_XLIE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  A3G4250D_GYRO_W_XLIE_bits(void *handle, A3G4250D_GYRO_XLIE_t newValue)
{
  u8_t value;

  if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~A3G4250D_GYRO_XLIE_MASK; 
  value |= newValue;
  
  if( !A3G4250D_GYRO_WriteReg(handle, A3G4250D_GYRO_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_XLIE_bits
* Description    : Read XLIE
* Input          : Pointer to A3G4250D_GYRO_XLIE_t
* Output         : Status of XLIE see A3G4250D_GYRO_XLIE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_XLIE_bits(void *handle, A3G4250D_GYRO_XLIE_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_INT1_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_XLIE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_W_XHIE_bits
* Description    : Write XHIE
* Input          : A3G4250D_GYRO_XHIE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  A3G4250D_GYRO_W_XHIE_bits(void *handle, A3G4250D_GYRO_XHIE_t newValue)
{
  u8_t value;

  if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~A3G4250D_GYRO_XHIE_MASK; 
  value |= newValue;
  
  if( !A3G4250D_GYRO_WriteReg(handle, A3G4250D_GYRO_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_XHIE_bits
* Description    : Read XHIE
* Input          : Pointer to A3G4250D_GYRO_XHIE_t
* Output         : Status of XHIE see A3G4250D_GYRO_XHIE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_XHIE_bits(void *handle, A3G4250D_GYRO_XHIE_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_INT1_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_XHIE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_W_YLIE_bits
* Description    : Write YLIE
* Input          : A3G4250D_GYRO_YLIE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  A3G4250D_GYRO_W_YLIE_bits(void *handle, A3G4250D_GYRO_YLIE_t newValue)
{
  u8_t value;

  if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~A3G4250D_GYRO_YLIE_MASK; 
  value |= newValue;
  
  if( !A3G4250D_GYRO_WriteReg(handle, A3G4250D_GYRO_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_YLIE_bits
* Description    : Read YLIE
* Input          : Pointer to A3G4250D_GYRO_YLIE_t
* Output         : Status of YLIE see A3G4250D_GYRO_YLIE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_YLIE_bits(void *handle, A3G4250D_GYRO_YLIE_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_INT1_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_YLIE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_W_YHIE_bits
* Description    : Write YHIE
* Input          : A3G4250D_GYRO_YHIE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  A3G4250D_GYRO_W_YHIE_bits(void *handle, A3G4250D_GYRO_YHIE_t newValue)
{
  u8_t value;

  if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~A3G4250D_GYRO_YHIE_MASK; 
  value |= newValue;
  
  if( !A3G4250D_GYRO_WriteReg(handle, A3G4250D_GYRO_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_YHIE_bits
* Description    : Read YHIE
* Input          : Pointer to A3G4250D_GYRO_YHIE_t
* Output         : Status of YHIE see A3G4250D_GYRO_YHIE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_YHIE_bits(void *handle, A3G4250D_GYRO_YHIE_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_INT1_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_YHIE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_W_ZLIE_bits
* Description    : Write ZLIE
* Input          : A3G4250D_GYRO_ZLIE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  A3G4250D_GYRO_W_ZLIE_bits(void *handle, A3G4250D_GYRO_ZLIE_t newValue)
{
  u8_t value;

  if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~A3G4250D_GYRO_ZLIE_MASK; 
  value |= newValue;
  
  if( !A3G4250D_GYRO_WriteReg(handle, A3G4250D_GYRO_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_ZLIE_bits
* Description    : Read ZLIE
* Input          : Pointer to A3G4250D_GYRO_ZLIE_t
* Output         : Status of ZLIE see A3G4250D_GYRO_ZLIE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_ZLIE_bits(void *handle, A3G4250D_GYRO_ZLIE_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_INT1_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_ZLIE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_W_ZHIE_bits
* Description    : Write ZHIE
* Input          : A3G4250D_GYRO_ZHIE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  A3G4250D_GYRO_W_ZHIE_bits(void *handle, A3G4250D_GYRO_ZHIE_t newValue)
{
  u8_t value;

  if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~A3G4250D_GYRO_ZHIE_MASK; 
  value |= newValue;
  
  if( !A3G4250D_GYRO_WriteReg(handle, A3G4250D_GYRO_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_ZHIE_bits
* Description    : Read ZHIE
* Input          : Pointer to A3G4250D_GYRO_ZHIE_t
* Output         : Status of ZHIE see A3G4250D_GYRO_ZHIE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_ZHIE_bits(void *handle, A3G4250D_GYRO_ZHIE_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_INT1_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_ZHIE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_W_LIR_bits
* Description    : Write LIR
* Input          : A3G4250D_GYRO_LIR_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  A3G4250D_GYRO_W_LIR_bits(void *handle, A3G4250D_GYRO_LIR_t newValue)
{
  u8_t value;

  if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~A3G4250D_GYRO_LIR_MASK; 
  value |= newValue;
  
  if( !A3G4250D_GYRO_WriteReg(handle, A3G4250D_GYRO_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_LIR_bits
* Description    : Read LIR
* Input          : Pointer to A3G4250D_GYRO_LIR_t
* Output         : Status of LIR see A3G4250D_GYRO_LIR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_LIR_bits(void *handle, A3G4250D_GYRO_LIR_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_INT1_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_LIR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_W_AND_OR_bits
* Description    : Write AND_OR
* Input          : A3G4250D_GYRO_AND_OR_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  A3G4250D_GYRO_W_AND_OR_bits(void *handle, A3G4250D_GYRO_AND_OR_t newValue)
{
  u8_t value;

  if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~A3G4250D_GYRO_AND_OR_MASK; 
  value |= newValue;
  
  if( !A3G4250D_GYRO_WriteReg(handle, A3G4250D_GYRO_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_AND_OR_bits
* Description    : Read AND_OR
* Input          : Pointer to A3G4250D_GYRO_AND_OR_t
* Output         : Status of AND_OR see A3G4250D_GYRO_AND_OR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_AND_OR_bits(void *handle, A3G4250D_GYRO_AND_OR_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_INT1_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_AND_OR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_XL_bits
* Description    : Read XL
* Input          : Pointer to A3G4250D_GYRO_XL_t
* Output         : Status of XL see A3G4250D_GYRO_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_XL_bits(void *handle, A3G4250D_GYRO_XL_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_INT1_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_XH_bits
* Description    : Read XH
* Input          : Pointer to A3G4250D_GYRO_XH_t
* Output         : Status of XH see A3G4250D_GYRO_XH_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_XH_bits(void *handle, A3G4250D_GYRO_XH_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_INT1_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_XH_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_YL_bits
* Description    : Read YL
* Input          : Pointer to A3G4250D_GYRO_YL_t
* Output         : Status of YL see A3G4250D_GYRO_YL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_YL_bits(void *handle, A3G4250D_GYRO_YL_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_INT1_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_YL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_YH_bits
* Description    : Read YH
* Input          : Pointer to A3G4250D_GYRO_YH_t
* Output         : Status of YH see A3G4250D_GYRO_YH_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_YH_bits(void *handle, A3G4250D_GYRO_YH_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_INT1_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_YH_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_ZL_bits
* Description    : Read ZL
* Input          : Pointer to A3G4250D_GYRO_ZL_t
* Output         : Status of ZL see A3G4250D_GYRO_ZL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_ZL_bits(void *handle, A3G4250D_GYRO_ZL_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_INT1_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_ZL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_ZH_bits
* Description    : Read ZH
* Input          : Pointer to A3G4250D_GYRO_ZH_t
* Output         : Status of ZH see A3G4250D_GYRO_ZH_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_ZH_bits(void *handle, A3G4250D_GYRO_ZH_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_INT1_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_ZH_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_IA_bits
* Description    : Read IA
* Input          : Pointer to A3G4250D_GYRO_IA_t
* Output         : Status of IA see A3G4250D_GYRO_IA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_IA_bits(void *handle, A3G4250D_GYRO_IA_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_INT1_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_IA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_W_D_bits
* Description    : Write D
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  A3G4250D_GYRO_W_D_bits(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << A3G4250D_GYRO_D_POSITION; //mask	
  newValue &= A3G4250D_GYRO_D_MASK; //coerce
  
  if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_INT1_DURATION, &value, 1) )
    return MEMS_ERROR;

  value &= ~A3G4250D_GYRO_D_MASK; 
  value |= newValue;
  
  if( !A3G4250D_GYRO_WriteReg(handle, A3G4250D_GYRO_INT1_DURATION, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_D_bits
* Description    : Read D
* Input          : Pointer to u8_t
* Output         : Status of D 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_D_bits(void *handle, u8_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_INT1_DURATION, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_D_MASK; //coerce	
  *value = *value >> A3G4250D_GYRO_D_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_W_WAIT_bits
* Description    : Write WAIT
* Input          : A3G4250D_GYRO_WAIT_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  A3G4250D_GYRO_W_WAIT_bits(void *handle, A3G4250D_GYRO_WAIT_t newValue)
{
  u8_t value;

  if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_INT1_DURATION, &value, 1) )
    return MEMS_ERROR;

  value &= ~A3G4250D_GYRO_WAIT_MASK; 
  value |= newValue;
  
  if( !A3G4250D_GYRO_WriteReg(handle, A3G4250D_GYRO_INT1_DURATION, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : A3G4250D_GYRO_R_WAIT_bits
* Description    : Read WAIT
* Input          : Pointer to A3G4250D_GYRO_WAIT_t
* Output         : Status of WAIT see A3G4250D_GYRO_WAIT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_R_WAIT_bits(void *handle, A3G4250D_GYRO_WAIT_t *value)
{
 if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_INT1_DURATION, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= A3G4250D_GYRO_WAIT_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : status_t A3G4250D_GYRO_Set_Reference(u8_t *buff) 
* Description    : Set Reference data row
* Input          : pointer to [u8_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_Set_Reference(void *handle, u8_t *buff) 
{
  u8_t  i;

  for (i=0; i<1;i++ ) 
  {
	if( !A3G4250D_GYRO_WriteReg(handle, A3G4250D_GYRO_REFERENCE+i,  &buff[i], 1) )
		return MEMS_ERROR;
  }
  return MEMS_SUCCESS;  
}

/*******************************************************************************
* Function Name  : status_t A3G4250D_GYRO_Get_Reference(u8_t *buff)
* Description    : Read Reference output register
* Input          : pointer to [u8_t]
* Output         : Reference buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_Get_Reference(void *handle, u8_t *buff) 
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;
  
  numberOfByteForDimension=1/1;

  k=0;
  for (i=0; i<1;i++ ) 
  {
	for (j=0; j<numberOfByteForDimension;j++ )
	{	
		if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_REFERENCE+k, &buff[k], 1))
		  return MEMS_ERROR;
		k++;	
	}
  }

  return MEMS_SUCCESS; 
}

/*******************************************************************************
* Function Name  : status_t A3G4250D_GYRO_Get_Temperature(u8_t *buff)
* Description    : Read Temperature output register
* Input          : pointer to [u8_t]
* Output         : Temperature buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_Get_Temperature(void *handle, u8_t *buff) 
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;
  
  numberOfByteForDimension=1/1;

  k=0;
  for (i=0; i<1;i++ ) 
  {
	for (j=0; j<numberOfByteForDimension;j++ )
	{	
		if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_OUT_TEMP+k, &buff[k], 1))
		  return MEMS_ERROR;
		k++;	
	}
  }

  return MEMS_SUCCESS; 
}

/*******************************************************************************
* Function Name  : status_t A3G4250D_GYRO_Set_Threshold(u8_t *buff) 
* Description    : Set Threshold data row
* Input          : pointer to [u8_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_Set_Threshold(void *handle, u8_t *buff) 
{
  u8_t  i;
  u8_t numberOfByteForDimension;
  
  numberOfByteForDimension=6/3;
  A3G4250D_GYRO_SwapHighLowByte(buff, 6, numberOfByteForDimension);

  for (i=0; i<6;i++ ) 
  {
	if( !A3G4250D_GYRO_WriteReg(handle, A3G4250D_GYRO_INT1_TSH_XH+i,  &buff[i], 1) )
		return MEMS_ERROR;
  }
  return MEMS_SUCCESS;  
}

/*******************************************************************************
* Function Name  : status_t A3G4250D_GYRO_Get_Threshold(u8_t *buff)
* Description    : Read Threshold output register
* Input          : pointer to [u8_t]
* Output         : Threshold buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t A3G4250D_GYRO_Get_Threshold(void *handle, u8_t *buff) 
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;
  
  numberOfByteForDimension=6/3;

  k=0;
  for (i=0; i<3;i++ ) 
  {
	for (j=0; j<numberOfByteForDimension;j++ )
	{	
		if( !A3G4250D_GYRO_ReadReg(handle, A3G4250D_GYRO_INT1_TSH_XH+k, &buff[k], 1))
		  return MEMS_ERROR;
		k++;	
	}
  }
  
  A3G4250D_GYRO_SwapHighLowByte(buff, 6, numberOfByteForDimension);

  return MEMS_SUCCESS; 
}

