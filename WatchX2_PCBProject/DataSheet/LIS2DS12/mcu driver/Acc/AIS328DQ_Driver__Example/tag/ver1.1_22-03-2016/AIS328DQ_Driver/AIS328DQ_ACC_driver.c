/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
* File Name          : AIS328DQ_ACC_driver.c
* Author             : MEMS Application Team
* Version            : v1.1
* Date               : 22 Mar 2016  
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
#include "AIS328DQ_ACC_driver.h"
//#include "i2C_mems.h"												//[Example]

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/************** Generic Function  *******************/

/*******************************************************************************
* Function Name		: AIS328DQ_ACC_WriteReg
* Description		: Generic Writing function. It must be fullfilled with either
*					: I2C or SPI writing function
* Input				: Register Address, ptr to buffer to be written,
*                                 length of buffer
* Output			: None
* Return			: None
*******************************************************************************/
status_t AIS328DQ_ACC_WriteReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len)
{
   
  //To be completed with either I2c or SPI writing function
  //i.e.: SPI_Mems_Write_Reg(Reg, Data);
  //I2Cx_Write(Bufp, AIS328DQ_ACC_GYRO_I2C_ADDRESS, Reg, len);    //[Example]
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name		: AIS328DQ_ACC_ReadReg
* Description		: Generic Reading function. It must be fullfilled with either
*					: I2C or SPI writing function
* Input				: Register Address, ptr to buffer to be read,
*                                 length of buffer
* Output			: None
* Return			: None
*******************************************************************************/
status_t AIS328DQ_ACC_ReadReg( void *handle, u8_t Reg, u8_t *Bufp, u16_t len)
{
    
  //To be completed with either I2c or SPI writing function
  //i.e.: SPI_Mems_Write_Reg(Reg, Data);
  //I2Cx_Read(Bufp, AIS328DQ_ACC_GYRO_I2C_ADDRESS, Reg, len); //[Example]
  return MEMS_SUCCESS; 
}

/**************** Base Function  *******************/

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_WHO_AM_I_BIT_bits
* Description    : Read WHO_AM_I_BIT
* Input          : Pointer to u8_t
* Output         : Status of WHO_AM_I_BIT 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_WHO_AM_I_BIT_bits(void *handle, u8_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_WHO_AM_I, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_WHO_AM_I_BIT_MASK; //coerce	
  *value = *value >> AIS328DQ_ACC_WHO_AM_I_BIT_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_W_CTRL_REG1_ODR
* Description    : Write DR
* Input          : AIS328DQ_ACC_DR_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  AIS328DQ_ACC_W_CTRL_REG1_ODR(void *handle, AIS328DQ_ACC_DR_t newValue)
{
  u8_t value;

  if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  value &= ~AIS328DQ_ACC_DR_MASK; 
  value |= newValue;
  
  if( !AIS328DQ_ACC_WriteReg( handle, AIS328DQ_ACC_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_CTRL_REG1_ODR
* Description    : Read DR
* Input          : Pointer to AIS328DQ_ACC_DR_t
* Output         : Status of DR see AIS328DQ_ACC_DR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_CTRL_REG1_ODR(void *handle, AIS328DQ_ACC_DR_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_DR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_W_CTRL_REG4_FS
* Description    : Write FS
* Input          : AIS328DQ_ACC_FS_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  AIS328DQ_ACC_W_CTRL_REG4_FS(void *handle, AIS328DQ_ACC_FS_t newValue)
{
  u8_t value;

  if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;

  value &= ~AIS328DQ_ACC_FS_MASK; 
  value |= newValue;
  
  if( !AIS328DQ_ACC_WriteReg( handle, AIS328DQ_ACC_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_CTRL_REG4_FS
* Description    : Read FS
* Input          : Pointer to AIS328DQ_ACC_FS_t
* Output         : Status of FS see AIS328DQ_ACC_FS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_CTRL_REG4_FS(void *handle, AIS328DQ_ACC_FS_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG4, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_FS_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_W_CTRL_REG4_BDU
* Description    : Write BDU
* Input          : AIS328DQ_ACC_BDU_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  AIS328DQ_ACC_W_CTRL_REG4_BDU(void *handle, AIS328DQ_ACC_BDU_t newValue)
{
  u8_t value;

  if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;

  value &= ~AIS328DQ_ACC_BDU_MASK; 
  value |= newValue;
  
  if( !AIS328DQ_ACC_WriteReg( handle, AIS328DQ_ACC_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_CTRL_REG4_BDU
* Description    : Read BDU
* Input          : Pointer to AIS328DQ_ACC_BDU_t
* Output         : Status of BDU see AIS328DQ_ACC_BDU_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_CTRL_REG4_BDU(void *handle, AIS328DQ_ACC_BDU_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG4, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_BDU_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : status_t AIS328DQ_ACC_Get_Acceleration(u8_t *buff)
* Description    : Read Acceleration output register
* Input          : pointer to [u8_t]
* Output         : Acceleration buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_Get_Acceleration(void *handle, u8_t *buff) 
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;
  
  numberOfByteForDimension=6/3;

  k=0;
  for (i=0; i<3;i++ ) 
  {
	for (j=0; j<numberOfByteForDimension;j++ )
	{	
		if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_OUT_X_L+k, &buff[k], 1))
		  return MEMS_ERROR;
		k++;	
	}
  }

  return MEMS_SUCCESS; 
}

/**************** Advanced Function  *******************/

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_W_CTRL_REG1_XEN
* Description    : Write XEN
* Input          : AIS328DQ_ACC_XEN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  AIS328DQ_ACC_W_CTRL_REG1_XEN(void *handle, AIS328DQ_ACC_XEN_t newValue)
{
  u8_t value;

  if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  value &= ~AIS328DQ_ACC_XEN_MASK; 
  value |= newValue;
  
  if( !AIS328DQ_ACC_WriteReg( handle, AIS328DQ_ACC_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_CTRL_REG1_XEN
* Description    : Read XEN
* Input          : Pointer to AIS328DQ_ACC_XEN_t
* Output         : Status of XEN see AIS328DQ_ACC_XEN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_CTRL_REG1_XEN(void *handle, AIS328DQ_ACC_XEN_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_XEN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_W_CTRL_REG1_YEN
* Description    : Write YEN
* Input          : AIS328DQ_ACC_YEN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  AIS328DQ_ACC_W_CTRL_REG1_YEN(void *handle, AIS328DQ_ACC_YEN_t newValue)
{
  u8_t value;

  if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  value &= ~AIS328DQ_ACC_YEN_MASK; 
  value |= newValue;
  
  if( !AIS328DQ_ACC_WriteReg( handle, AIS328DQ_ACC_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_CTRL_REG1_YEN
* Description    : Read YEN
* Input          : Pointer to AIS328DQ_ACC_YEN_t
* Output         : Status of YEN see AIS328DQ_ACC_YEN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_CTRL_REG1_YEN(void *handle, AIS328DQ_ACC_YEN_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_YEN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_W_CTRL_REG1_ZEN
* Description    : Write ZEN
* Input          : AIS328DQ_ACC_ZEN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  AIS328DQ_ACC_W_CTRL_REG1_ZEN(void *handle, AIS328DQ_ACC_ZEN_t newValue)
{
  u8_t value;

  if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  value &= ~AIS328DQ_ACC_ZEN_MASK; 
  value |= newValue;
  
  if( !AIS328DQ_ACC_WriteReg( handle, AIS328DQ_ACC_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_CTRL_REG1_ZEN
* Description    : Read ZEN
* Input          : Pointer to AIS328DQ_ACC_ZEN_t
* Output         : Status of ZEN see AIS328DQ_ACC_ZEN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_CTRL_REG1_ZEN(void *handle, AIS328DQ_ACC_ZEN_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_ZEN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_W_CTRL_REG1_PM
* Description    : Write PM
* Input          : AIS328DQ_ACC_PM_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  AIS328DQ_ACC_W_CTRL_REG1_PM(void *handle, AIS328DQ_ACC_PM_t newValue)
{
  u8_t value;

  if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  value &= ~AIS328DQ_ACC_PM_MASK; 
  value |= newValue;
  
  if( !AIS328DQ_ACC_WriteReg( handle, AIS328DQ_ACC_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_CTRL_REG1_PM
* Description    : Read PM
* Input          : Pointer to AIS328DQ_ACC_PM_t
* Output         : Status of PM see AIS328DQ_ACC_PM_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_CTRL_REG1_PM(void *handle, AIS328DQ_ACC_PM_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_PM_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_W_CTRL_REG2_HPCF
* Description    : Write HPCF
* Input          : AIS328DQ_ACC_HPCF_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  AIS328DQ_ACC_W_CTRL_REG2_HPCF(void *handle, AIS328DQ_ACC_HPCF_t newValue)
{
  u8_t value;

  if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;

  value &= ~AIS328DQ_ACC_HPCF_MASK; 
  value |= newValue;
  
  if( !AIS328DQ_ACC_WriteReg( handle, AIS328DQ_ACC_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_CTRL_REG2_HPCF
* Description    : Read HPCF
* Input          : Pointer to AIS328DQ_ACC_HPCF_t
* Output         : Status of HPCF see AIS328DQ_ACC_HPCF_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_CTRL_REG2_HPCF(void *handle, AIS328DQ_ACC_HPCF_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_HPCF_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_W_CTRL_REG2_HPEN1
* Description    : Write HPEN1
* Input          : AIS328DQ_ACC_HPEN1_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  AIS328DQ_ACC_W_CTRL_REG2_HPEN1(void *handle, AIS328DQ_ACC_HPEN1_t newValue)
{
  u8_t value;

  if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;

  value &= ~AIS328DQ_ACC_HPEN1_MASK; 
  value |= newValue;
  
  if( !AIS328DQ_ACC_WriteReg( handle, AIS328DQ_ACC_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_CTRL_REG2_HPEN1
* Description    : Read HPEN1
* Input          : Pointer to AIS328DQ_ACC_HPEN1_t
* Output         : Status of HPEN1 see AIS328DQ_ACC_HPEN1_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_CTRL_REG2_HPEN1(void *handle, AIS328DQ_ACC_HPEN1_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_HPEN1_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_W_CTRL_REG2_HPEN2
* Description    : Write HPEN2
* Input          : AIS328DQ_ACC_HPEN2_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  AIS328DQ_ACC_W_CTRL_REG2_HPEN2(void *handle, AIS328DQ_ACC_HPEN2_t newValue)
{
  u8_t value;

  if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;

  value &= ~AIS328DQ_ACC_HPEN2_MASK; 
  value |= newValue;
  
  if( !AIS328DQ_ACC_WriteReg( handle, AIS328DQ_ACC_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_CTRL_REG2_HPEN2
* Description    : Read HPEN2
* Input          : Pointer to AIS328DQ_ACC_HPEN2_t
* Output         : Status of HPEN2 see AIS328DQ_ACC_HPEN2_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_CTRL_REG2_HPEN2(void *handle, AIS328DQ_ACC_HPEN2_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_HPEN2_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_W_CTRL_REG2_FDS
* Description    : Write FDS
* Input          : AIS328DQ_ACC_FDS_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  AIS328DQ_ACC_W_CTRL_REG2_FDS(void *handle, AIS328DQ_ACC_FDS_t newValue)
{
  u8_t value;

  if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;

  value &= ~AIS328DQ_ACC_FDS_MASK; 
  value |= newValue;
  
  if( !AIS328DQ_ACC_WriteReg( handle, AIS328DQ_ACC_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_CTRL_REG2_FDS
* Description    : Read FDS
* Input          : Pointer to AIS328DQ_ACC_FDS_t
* Output         : Status of FDS see AIS328DQ_ACC_FDS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_CTRL_REG2_FDS(void *handle, AIS328DQ_ACC_FDS_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_FDS_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_W_CTRL_REG2_HPM
* Description    : Write HPM
* Input          : AIS328DQ_ACC_HPM_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  AIS328DQ_ACC_W_CTRL_REG2_HPM(void *handle, AIS328DQ_ACC_HPM_t newValue)
{
  u8_t value;

  if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;

  value &= ~AIS328DQ_ACC_HPM_MASK; 
  value |= newValue;
  
  if( !AIS328DQ_ACC_WriteReg( handle, AIS328DQ_ACC_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_CTRL_REG2_HPM
* Description    : Read HPM
* Input          : Pointer to AIS328DQ_ACC_HPM_t
* Output         : Status of HPM see AIS328DQ_ACC_HPM_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_CTRL_REG2_HPM(void *handle, AIS328DQ_ACC_HPM_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_HPM_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_W_CTRL_REG2_BOOT
* Description    : Write BOOT
* Input          : AIS328DQ_ACC_BOOT_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  AIS328DQ_ACC_W_CTRL_REG2_BOOT(void *handle, AIS328DQ_ACC_BOOT_t newValue)
{
  u8_t value;

  if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;

  value &= ~AIS328DQ_ACC_BOOT_MASK; 
  value |= newValue;
  
  if( !AIS328DQ_ACC_WriteReg( handle, AIS328DQ_ACC_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_CTRL_REG2_BOOT
* Description    : Read BOOT
* Input          : Pointer to AIS328DQ_ACC_BOOT_t
* Output         : Status of BOOT see AIS328DQ_ACC_BOOT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_CTRL_REG2_BOOT(void *handle, AIS328DQ_ACC_BOOT_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_BOOT_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_W_CTRL_REG3_I1_CFG
* Description    : Write I1_CFG
* Input          : AIS328DQ_ACC_I1_CFG_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  AIS328DQ_ACC_W_CTRL_REG3_I1_CFG(void *handle, AIS328DQ_ACC_I1_CFG_t newValue)
{
  u8_t value;

  if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  value &= ~AIS328DQ_ACC_I1_CFG_MASK; 
  value |= newValue;
  
  if( !AIS328DQ_ACC_WriteReg( handle, AIS328DQ_ACC_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_CTRL_REG3_I1_CFG
* Description    : Read I1_CFG
* Input          : Pointer to AIS328DQ_ACC_I1_CFG_t
* Output         : Status of I1_CFG see AIS328DQ_ACC_I1_CFG_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_CTRL_REG3_I1_CFG(void *handle, AIS328DQ_ACC_I1_CFG_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG3, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_I1_CFG_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_W_CTRL_REG3_LIR1
* Description    : Write LIR1
* Input          : AIS328DQ_ACC_LIR1_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  AIS328DQ_ACC_W_CTRL_REG3_LIR1(void *handle, AIS328DQ_ACC_LIR1_t newValue)
{
  u8_t value;

  if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  value &= ~AIS328DQ_ACC_LIR1_MASK; 
  value |= newValue;
  
  if( !AIS328DQ_ACC_WriteReg( handle, AIS328DQ_ACC_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_CTRL_REG3_LIR1
* Description    : Read LIR1
* Input          : Pointer to AIS328DQ_ACC_LIR1_t
* Output         : Status of LIR1 see AIS328DQ_ACC_LIR1_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_CTRL_REG3_LIR1(void *handle, AIS328DQ_ACC_LIR1_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG3, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_LIR1_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_W_CTRL_REG3_I2_CFG
* Description    : Write I2_CFG
* Input          : AIS328DQ_ACC_I2_CFG_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  AIS328DQ_ACC_W_CTRL_REG3_I2_CFG(void *handle, AIS328DQ_ACC_I2_CFG_t newValue)
{
  u8_t value;

  if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  value &= ~AIS328DQ_ACC_I2_CFG_MASK; 
  value |= newValue;
  
  if( !AIS328DQ_ACC_WriteReg( handle, AIS328DQ_ACC_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_CTRL_REG3_I2_CFG
* Description    : Read I2_CFG
* Input          : Pointer to AIS328DQ_ACC_I2_CFG_t
* Output         : Status of I2_CFG see AIS328DQ_ACC_I2_CFG_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_CTRL_REG3_I2_CFG(void *handle, AIS328DQ_ACC_I2_CFG_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG3, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_I2_CFG_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_W_CTRL_REG3_LIR2
* Description    : Write LIR2
* Input          : AIS328DQ_ACC_LIR2_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  AIS328DQ_ACC_W_CTRL_REG3_LIR2(void *handle, AIS328DQ_ACC_LIR2_t newValue)
{
  u8_t value;

  if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  value &= ~AIS328DQ_ACC_LIR2_MASK; 
  value |= newValue;
  
  if( !AIS328DQ_ACC_WriteReg( handle, AIS328DQ_ACC_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_CTRL_REG3_LIR2
* Description    : Read LIR2
* Input          : Pointer to AIS328DQ_ACC_LIR2_t
* Output         : Status of LIR2 see AIS328DQ_ACC_LIR2_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_CTRL_REG3_LIR2(void *handle, AIS328DQ_ACC_LIR2_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG3, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_LIR2_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_W_CTRL_REG3_PPOD
* Description    : Write PP_OD
* Input          : AIS328DQ_ACC_PP_OD_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  AIS328DQ_ACC_W_CTRL_REG3_PPOD(void *handle, AIS328DQ_ACC_PP_OD_t newValue)
{
  u8_t value;

  if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  value &= ~AIS328DQ_ACC_PP_OD_MASK; 
  value |= newValue;
  
  if( !AIS328DQ_ACC_WriteReg( handle, AIS328DQ_ACC_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_CTRL_REG3_PPOD
* Description    : Read PP_OD
* Input          : Pointer to AIS328DQ_ACC_PP_OD_t
* Output         : Status of PP_OD see AIS328DQ_ACC_PP_OD_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_CTRL_REG3_PPOD(void *handle, AIS328DQ_ACC_PP_OD_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG3, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_PP_OD_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_W_CTRL_REG3_IHL
* Description    : Write IHL
* Input          : AIS328DQ_ACC_IHL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  AIS328DQ_ACC_W_CTRL_REG3_IHL(void *handle, AIS328DQ_ACC_IHL_t newValue)
{
  u8_t value;

  if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  value &= ~AIS328DQ_ACC_IHL_MASK; 
  value |= newValue;
  
  if( !AIS328DQ_ACC_WriteReg( handle, AIS328DQ_ACC_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_CTRL_REG3_IHL
* Description    : Read IHL
* Input          : Pointer to AIS328DQ_ACC_IHL_t
* Output         : Status of IHL see AIS328DQ_ACC_IHL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_CTRL_REG3_IHL(void *handle, AIS328DQ_ACC_IHL_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG3, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_IHL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_W_CTRL_REG4_SIM
* Description    : Write SIM
* Input          : AIS328DQ_ACC_SIM_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  AIS328DQ_ACC_W_CTRL_REG4_SIM(void *handle, AIS328DQ_ACC_SIM_t newValue)
{
  u8_t value;

  if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;

  value &= ~AIS328DQ_ACC_SIM_MASK; 
  value |= newValue;
  
  if( !AIS328DQ_ACC_WriteReg( handle, AIS328DQ_ACC_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_CTRL_REG4_SIM
* Description    : Read SIM
* Input          : Pointer to AIS328DQ_ACC_SIM_t
* Output         : Status of SIM see AIS328DQ_ACC_SIM_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_CTRL_REG4_SIM(void *handle, AIS328DQ_ACC_SIM_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG4, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_SIM_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_W_CTRL_REG4_ST
* Description    : Write ST
* Input          : AIS328DQ_ACC_ST_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  AIS328DQ_ACC_W_CTRL_REG4_ST(void *handle, AIS328DQ_ACC_ST_t newValue)
{
  u8_t value;

  if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;

  value &= ~AIS328DQ_ACC_ST_MASK; 
  value |= newValue;
  
  if( !AIS328DQ_ACC_WriteReg( handle, AIS328DQ_ACC_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_CTRL_REG4_ST
* Description    : Read ST
* Input          : Pointer to AIS328DQ_ACC_ST_t
* Output         : Status of ST see AIS328DQ_ACC_ST_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_CTRL_REG4_ST(void *handle, AIS328DQ_ACC_ST_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG4, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_ST_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_W_CTRL_REG4_STSIGN
* Description    : Write STSIGN
* Input          : AIS328DQ_ACC_STSIGN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  AIS328DQ_ACC_W_CTRL_REG4_STSIGN(void *handle, AIS328DQ_ACC_STSIGN_t newValue)
{
  u8_t value;

  if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;

  value &= ~AIS328DQ_ACC_STSIGN_MASK; 
  value |= newValue;
  
  if( !AIS328DQ_ACC_WriteReg( handle, AIS328DQ_ACC_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_CTRL_REG4_STSIGN
* Description    : Read STSIGN
* Input          : Pointer to AIS328DQ_ACC_STSIGN_t
* Output         : Status of STSIGN see AIS328DQ_ACC_STSIGN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_CTRL_REG4_STSIGN(void *handle, AIS328DQ_ACC_STSIGN_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG4, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_STSIGN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_W_CTRL_REG4_BLE
* Description    : Write BLE
* Input          : AIS328DQ_ACC_BLE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  AIS328DQ_ACC_W_CTRL_REG4_BLE(void *handle, AIS328DQ_ACC_BLE_t newValue)
{
  u8_t value;

  if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;

  value &= ~AIS328DQ_ACC_BLE_MASK; 
  value |= newValue;
  
  if( !AIS328DQ_ACC_WriteReg( handle, AIS328DQ_ACC_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_CTRL_REG4_BLE
* Description    : Read BLE
* Input          : Pointer to AIS328DQ_ACC_BLE_t
* Output         : Status of BLE see AIS328DQ_ACC_BLE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_CTRL_REG4_BLE(void *handle, AIS328DQ_ACC_BLE_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG4, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_BLE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_W_CTRL_REG5_TURNON
* Description    : Write TURNON
* Input          : AIS328DQ_ACC_TURNON_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  AIS328DQ_ACC_W_CTRL_REG5_TURNON(void *handle, AIS328DQ_ACC_TURNON_t newValue)
{
  u8_t value;

  if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG5, &value, 1) )
    return MEMS_ERROR;

  value &= ~AIS328DQ_ACC_TURNON_MASK; 
  value |= newValue;
  
  if( !AIS328DQ_ACC_WriteReg( handle, AIS328DQ_ACC_CTRL_REG5, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_CTRL_REG5_TURNON
* Description    : Read TURNON
* Input          : Pointer to AIS328DQ_ACC_TURNON_t
* Output         : Status of TURNON see AIS328DQ_ACC_TURNON_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_CTRL_REG5_TURNON(void *handle, AIS328DQ_ACC_TURNON_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_CTRL_REG5, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_TURNON_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_STATUS_XDA
* Description    : Read XDA
* Input          : Pointer to AIS328DQ_ACC_XDA_t
* Output         : Status of XDA see AIS328DQ_ACC_XDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_STATUS_XDA(void *handle, AIS328DQ_ACC_XDA_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_XDA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_STATUS_YDA
* Description    : Read YDA
* Input          : Pointer to AIS328DQ_ACC_YDA_t
* Output         : Status of YDA see AIS328DQ_ACC_YDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_STATUS_YDA(void *handle, AIS328DQ_ACC_YDA_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_YDA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_STATUS_ZDA
* Description    : Read ZDA
* Input          : Pointer to AIS328DQ_ACC_ZDA_t
* Output         : Status of ZDA see AIS328DQ_ACC_ZDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_STATUS_ZDA(void *handle, AIS328DQ_ACC_ZDA_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_ZDA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_STATUS_ZYXDA
* Description    : Read ZYXDA
* Input          : Pointer to AIS328DQ_ACC_ZYXDA_t
* Output         : Status of ZYXDA see AIS328DQ_ACC_ZYXDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_STATUS_ZYXDA(void *handle, AIS328DQ_ACC_ZYXDA_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_ZYXDA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_STATUS_XOR
* Description    : Read XOR
* Input          : Pointer to AIS328DQ_ACC_XOR_t
* Output         : Status of XOR see AIS328DQ_ACC_XOR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_STATUS_XOR(void *handle, AIS328DQ_ACC_XOR_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_XOR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_STATUS_YOR
* Description    : Read YOR
* Input          : Pointer to AIS328DQ_ACC_YOR_t
* Output         : Status of YOR see AIS328DQ_ACC_YOR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_STATUS_YOR(void *handle, AIS328DQ_ACC_YOR_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_YOR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_STATUS_ZOR
* Description    : Read ZOR
* Input          : Pointer to AIS328DQ_ACC_ZOR_t
* Output         : Status of ZOR see AIS328DQ_ACC_ZOR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_STATUS_ZOR(void *handle, AIS328DQ_ACC_ZOR_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_ZOR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_STATUS_ZYXOR
* Description    : Read ZYXOR
* Input          : Pointer to AIS328DQ_ACC_ZYXOR_t
* Output         : Status of ZYXOR see AIS328DQ_ACC_ZYXOR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_STATUS_ZYXOR(void *handle, AIS328DQ_ACC_ZYXOR_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_ZYXOR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_W_INT1_CFG_XLIE
* Description    : Write XLIE
* Input          : AIS328DQ_ACC_INT1_XLIE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  AIS328DQ_ACC_W_INT1_CFG_XLIE(void *handle, AIS328DQ_ACC_INT1_XLIE_t newValue)
{
  u8_t value;

  if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~AIS328DQ_ACC_INT1_XLIE_MASK; 
  value |= newValue;
  
  if( !AIS328DQ_ACC_WriteReg( handle, AIS328DQ_ACC_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_INT1_CFG_XLIE
* Description    : Read XLIE
* Input          : Pointer to AIS328DQ_ACC_INT1_XLIE_t
* Output         : Status of XLIE see AIS328DQ_ACC_XLIE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_INT1_CFG_XLIE(void *handle, AIS328DQ_ACC_INT1_XLIE_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT1_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_INT1_XLIE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_W_INT1_CFG_XHIE
* Description    : Write XHIE
* Input          : AIS328DQ_ACC_INT1_XHIE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  AIS328DQ_ACC_W_INT1_CFG_XHIE(void *handle, AIS328DQ_ACC_INT1_XHIE_t newValue)
{
  u8_t value;

  if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~AIS328DQ_ACC_INT1_XHIE_MASK; 
  value |= newValue;
  
  if( !AIS328DQ_ACC_WriteReg( handle, AIS328DQ_ACC_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_INT1_CFG_XHIE
* Description    : Read XHIE
* Input          : Pointer to AIS328DQ_ACC_INT1_XHIE_t
* Output         : Status of XHIE see AIS328DQ_ACC_INT1_XHIE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_INT1_CFG_XHIE(void *handle, AIS328DQ_ACC_INT1_XHIE_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT1_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_INT1_XHIE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_W_INT1_CFG_YLIE
* Description    : Write YLIE
* Input          : AIS328DQ_ACC_INT1_YLIE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  AIS328DQ_ACC_W_INT1_CFG_YLIE(void *handle, AIS328DQ_ACC_INT1_YLIE_t newValue)
{
  u8_t value;

  if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~AIS328DQ_ACC_INT1_YLIE_MASK; 
  value |= newValue;
  
  if( !AIS328DQ_ACC_WriteReg( handle, AIS328DQ_ACC_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_INT1_CFG_YLIE
* Description    : Read YLIE
* Input          : Pointer to AIS328DQ_ACC_INT1_YLIE_t
* Output         : Status of YLIE see AIS328DQ_ACC_INT1_YLIE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_INT1_CFG_YLIE(void *handle, AIS328DQ_ACC_INT1_YLIE_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT1_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_INT1_YLIE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_W_INT1_CFG_YHIE
* Description    : Write YHIE
* Input          : AIS328DQ_ACC_INT1_YHIE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  AIS328DQ_ACC_W_INT1_CFG_YHIE(void *handle, AIS328DQ_ACC_INT1_YHIE_t newValue)
{
  u8_t value;

  if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~AIS328DQ_ACC_INT1_YHIE_MASK; 
  value |= newValue;
  
  if( !AIS328DQ_ACC_WriteReg( handle, AIS328DQ_ACC_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_INT1_CFG_YHIE
* Description    : Read YHIE
* Input          : Pointer to AIS328DQ_INT1_ACC_YHIE_t
* Output         : Status of YHIE see AIS328DQ_INT1_ACC_YHIE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_INT1_CFG_YHIE(void *handle, AIS328DQ_ACC_INT1_YHIE_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT1_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_INT1_YHIE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_W_INT1_CFG_ZLIE
* Description    : Write ZLIE
* Input          : AIS328DQ_ACC_INT1_ZLIE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  AIS328DQ_ACC_W_INT1_CFG_ZLIE(void *handle, AIS328DQ_ACC_INT1_ZLIE_t newValue)
{
  u8_t value;

  if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~AIS328DQ_ACC_INT1_ZLIE_MASK; 
  value |= newValue;
  
  if( !AIS328DQ_ACC_WriteReg( handle, AIS328DQ_ACC_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_INT1_CFG_ZLIE
* Description    : Read ZLIE
* Input          : Pointer to AIS328DQ_ACC_ZLIE_t
* Output         : Status of ZLIE see AIS328DQ_ACC_ZLIE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_INT1_CFG_ZLIE(void *handle, AIS328DQ_ACC_INT1_ZLIE_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT1_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_INT1_ZLIE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_W_INT1_CFG_INT1_ZHIE
* Description    : Write ZHIE
* Input          : AIS328DQ_ACC_INT1_ZHIE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  AIS328DQ_ACC_W_INT1_CFG_ZHIE(void *handle, AIS328DQ_ACC_INT1_ZHIE_t newValue)
{
  u8_t value;

  if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~AIS328DQ_ACC_INT1_ZHIE_MASK; 
  value |= newValue;
  
  if( !AIS328DQ_ACC_WriteReg( handle, AIS328DQ_ACC_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_INT1_CFG_INT1_ZHIE
* Description    : Read ZHIE
* Input          : Pointer to AIS328DQ_ACC_INT1_ZHIE_t
* Output         : Status of ZHIE see AIS328DQ_ACC_ZHIE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_INT1_CFG_ZHIE(void *handle, AIS328DQ_ACC_INT1_ZHIE_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT1_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_INT1_ZHIE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_W_INT1_6D
* Description    : Write 6D
* Input          : AIS328DQ_ACC_INT1_6D_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  AIS328DQ_ACC_W_INT1_6D(void *handle, AIS328DQ_ACC_INT1_6D_t newValue)
{
  u8_t value;

  if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~AIS328DQ_ACC_INT1_6D_MASK; 
  value |= newValue;
  
  if( !AIS328DQ_ACC_WriteReg( handle, AIS328DQ_ACC_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_INT1_6D
* Description    : Read 6D
* Input          : Pointer to AIS328DQ_ACC_INT1_6D_t
* Output         : Status of 6D see AIS328DQ_ACC_6D_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_INT1_6D(void *handle, AIS328DQ_ACC_INT1_6D_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT1_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_INT1_6D_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_W_INT1_CFG_AOI
* Description    : Write AOI
* Input          : AIS328DQ_ACC_INT1_AOI_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  AIS328DQ_ACC_W_INT1_CFG_AOI(void *handle, AIS328DQ_ACC_INT1_AOI_t newValue)
{
  u8_t value;

  if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~AIS328DQ_ACC_INT1_AOI_MASK; 
  value |= newValue;
  
  if( !AIS328DQ_ACC_WriteReg( handle, AIS328DQ_ACC_INT1_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_INT1_CFG_INT1_AOI
* Description    : Read AOI
* Input          : Pointer to AIS328DQ_ACC_INT1_AOI_t
* Output         : Status of AOI see AIS328DQ_ACC_AOI_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_INT1_CFG_AOI(void *handle, AIS328DQ_ACC_INT1_AOI_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT1_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_INT1_AOI_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_INT1_SOURCE_XL
* Description    : Read XL
* Input          : Pointer to AIS328DQ_ACC_INT1_XL_t
* Output         : Status of XL see AIS328DQ_ACC_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_INT1_SOURCE_XL(void *handle, AIS328DQ_ACC_INT1_XL_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT1_SOURCE, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_INT1_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_INT1_SOURCE_XH
* Description    : Read XH
* Input          : Pointer to AIS328DQ_ACC_INT1_XH_t
* Output         : Status of XH see AIS328DQ_ACC_XH_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_INT1_SOURCE_XH(void *handle, AIS328DQ_ACC_INT1_XH_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT1_SOURCE, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_INT1_XH_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_INT1_SOURCE_YL
* Description    : Read YL
* Input          : Pointer to AIS328DQ_ACC_INT1_YL_t
* Output         : Status of YL see AIS328DQ_ACC_YL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_INT1_SOURCE_YL(void *handle, AIS328DQ_ACC_INT1_YL_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT1_SOURCE, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_INT1_YL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_INT1_SOURCE_YH
* Description    : Read YH
* Input          : Pointer to AIS328DQ_ACC_INT1_YH_t
* Output         : Status of YH see AIS328DQ_ACC_YH_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_INT1_SOURCE_YH(void *handle, AIS328DQ_ACC_INT1_YH_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT1_SOURCE, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_INT1_YH_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_INT1_SOURCE_ZL
* Description    : Read ZL
* Input          : Pointer to AIS328DQ_ACC_INT1_ZL_t
* Output         : Status of ZL see AIS328DQ_ACC_ZL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_INT1_SOURCE_ZL(void *handle, AIS328DQ_ACC_INT1_ZL_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT1_SOURCE, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_INT1_ZL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_INT1_SOURCE_ZH
* Description    : Read ZH
* Input          : Pointer to AIS328DQ_ACC_INT1_ZH_t
* Output         : Status of ZH see AIS328DQ_ACC_ZH_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_INT1_SOURCE_ZH(void *handle, AIS328DQ_ACC_INT1_ZH_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT1_SOURCE, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_INT1_ZH_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_INT1_SOURCE_IA
* Description    : Read IA
* Input          : Pointer to AIS328DQ_ACC_INT1_IA_t
* Output         : Status of IA see AIS328DQ_ACC_IA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_INT1_SOURCE_IA(void *handle, AIS328DQ_ACC_INT1_IA_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT1_SOURCE, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_INT1_IA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_W_INT1_THS
* Description    : Write THS
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  AIS328DQ_ACC_W_INT1_THS(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << AIS328DQ_ACC_INT1_THS_POSITION; //mask	
  newValue &= AIS328DQ_ACC_INT1_THS_MASK; //coerce
  
  if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT1_THS, &value, 1) )
    return MEMS_ERROR;

  value &= ~AIS328DQ_ACC_INT1_THS_MASK; 
  value |= newValue;
  
  if( !AIS328DQ_ACC_WriteReg( handle, AIS328DQ_ACC_INT1_THS, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_INT1_THS
* Description    : Read THS
* Input          : Pointer to u8_t
* Output         : Status of THS 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_INT1_THS(void *handle, u8_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT1_THS, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_INT1_THS_MASK; //coerce	
  *value = *value >> AIS328DQ_ACC_INT1_THS_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_W_INT1_DURATION
* Description    : Write D
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  AIS328DQ_ACC_W_INT1_DURATION(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << AIS328DQ_ACC_INT1_D_POSITION; //mask	
  newValue &= AIS328DQ_ACC_INT1_D_MASK; //coerce
  
  if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT1_DURATION, &value, 1) )
    return MEMS_ERROR;

  value &= ~AIS328DQ_ACC_INT1_D_MASK; 
  value |= newValue;
  
  if( !AIS328DQ_ACC_WriteReg( handle, AIS328DQ_ACC_INT1_DURATION, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_INT1_DURATION
* Description    : Read D
* Input          : Pointer to u8_t
* Output         : Status of D 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_INT1_DURATION(void *handle, u8_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT1_DURATION, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_INT1_D_MASK; //coerce	
  *value = *value >> AIS328DQ_ACC_INT1_D_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_W_INT2_CFG_XLIE
* Description    : Write XLIE
* Input          : AIS328DQ_ACC_INT2_XLIE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  AIS328DQ_ACC_W_INT2_CFG_XLIE(void *handle, AIS328DQ_ACC_INT2_XLIE_t newValue)
{
  u8_t value;

  if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT2_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~AIS328DQ_ACC_INT2_XLIE_MASK; 
  value |= newValue;
  
  if( !AIS328DQ_ACC_WriteReg( handle, AIS328DQ_ACC_INT2_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_INT2_CFG_XLIE
* Description    : Read XLIE
* Input          : Pointer to AIS328DQ_ACC_INT2_XLIE_t
* Output         : Status of XLIE see AIS328DQ_ACC_XLIE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_INT2_CFG_XLIE(void *handle, AIS328DQ_ACC_INT2_XLIE_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT2_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_INT2_XLIE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_W_INT2_CFG_XHIE
* Description    : Write XHIE
* Input          : AIS328DQ_ACC_INT2_XHIE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  AIS328DQ_ACC_W_INT2_CFG_XHIE(void *handle, AIS328DQ_ACC_INT2_XHIE_t newValue)
{
  u8_t value;

  if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT2_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~AIS328DQ_ACC_INT2_XHIE_MASK; 
  value |= newValue;
  
  if( !AIS328DQ_ACC_WriteReg( handle, AIS328DQ_ACC_INT2_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_INT2_CFG_XHIE
* Description    : Read XHIE
* Input          : Pointer to AIS328DQ_ACC_INT2_XHIE_t
* Output         : Status of XHIE see AIS328DQ_ACC_INT2_XHIE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_INT2_CFG_XHIE(void *handle, AIS328DQ_ACC_INT2_XHIE_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT2_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_INT2_XHIE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_W_INT2_CFG_YLIE
* Description    : Write YLIE
* Input          : AIS328DQ_ACC_INT2_YLIE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  AIS328DQ_ACC_W_INT2_CFG_YLIE(void *handle, AIS328DQ_ACC_INT2_YLIE_t newValue)
{
  u8_t value;

  if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT2_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~AIS328DQ_ACC_INT2_YLIE_MASK; 
  value |= newValue;
  
  if( !AIS328DQ_ACC_WriteReg( handle, AIS328DQ_ACC_INT2_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_INT2_CFG_YLIE
* Description    : Read YLIE
* Input          : Pointer to AIS328DQ_ACC_INT2_YLIE_t
* Output         : Status of YLIE see AIS328DQ_ACC_INT2_YLIE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_INT2_CFG_YLIE(void *handle, AIS328DQ_ACC_INT2_YLIE_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT2_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_INT2_YLIE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_W_INT2_CFG_YHIE
* Description    : Write YHIE
* Input          : AIS328DQ_ACC_INT2_YHIE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  AIS328DQ_ACC_W_INT2_CFG_YHIE(void *handle, AIS328DQ_ACC_INT2_YHIE_t newValue)
{
  u8_t value;

  if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT2_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~AIS328DQ_ACC_INT2_YHIE_MASK; 
  value |= newValue;
  
  if( !AIS328DQ_ACC_WriteReg( handle, AIS328DQ_ACC_INT2_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_INT2_CFG_YHIE
* Description    : Read YHIE
* Input          : Pointer to AIS328DQ_INT2_ACC_YHIE_t
* Output         : Status of YHIE see AIS328DQ_INT2_ACC_YHIE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_INT2_CFG_YHIE(void *handle, AIS328DQ_ACC_INT2_YHIE_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT2_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_INT2_YHIE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_W_INT2_CFG_ZLIE
* Description    : Write ZLIE
* Input          : AIS328DQ_ACC_INT2_ZLIE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  AIS328DQ_ACC_W_INT2_CFG_ZLIE(void *handle, AIS328DQ_ACC_INT2_ZLIE_t newValue)
{
  u8_t value;

  if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT2_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~AIS328DQ_ACC_INT2_ZLIE_MASK; 
  value |= newValue;
  
  if( !AIS328DQ_ACC_WriteReg( handle, AIS328DQ_ACC_INT2_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_INT2_CFG_ZLIE
* Description    : Read ZLIE
* Input          : Pointer to AIS328DQ_ACC_ZLIE_t
* Output         : Status of ZLIE see AIS328DQ_ACC_ZLIE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_INT2_CFG_ZLIE(void *handle, AIS328DQ_ACC_INT2_ZLIE_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT2_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_INT2_ZLIE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_W_INT2_CFG_INT2_ZHIE
* Description    : Write ZHIE
* Input          : AIS328DQ_ACC_INT2_ZHIE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  AIS328DQ_ACC_W_INT2_CFG_ZHIE(void *handle, AIS328DQ_ACC_INT2_ZHIE_t newValue)
{
  u8_t value;

  if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT2_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~AIS328DQ_ACC_INT2_ZHIE_MASK; 
  value |= newValue;
  
  if( !AIS328DQ_ACC_WriteReg( handle, AIS328DQ_ACC_INT2_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_INT2_CFG_INT2_ZHIE
* Description    : Read ZHIE
* Input          : Pointer to AIS328DQ_ACC_INT2_ZHIE_t
* Output         : Status of ZHIE see AIS328DQ_ACC_ZHIE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_INT2_CFG_ZHIE(void *handle, AIS328DQ_ACC_INT2_ZHIE_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT2_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_INT2_ZHIE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_W_INT2_6D
* Description    : Write 6D
* Input          : AIS328DQ_ACC_INT2_6D_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  AIS328DQ_ACC_W_INT2_6D(void *handle, AIS328DQ_ACC_INT2_6D_t newValue)
{
  u8_t value;

  if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT2_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~AIS328DQ_ACC_INT2_6D_MASK; 
  value |= newValue;
  
  if( !AIS328DQ_ACC_WriteReg( handle, AIS328DQ_ACC_INT2_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_INT2_6D
* Description    : Read 6D
* Input          : Pointer to AIS328DQ_ACC_INT2_6D_t
* Output         : Status of 6D see AIS328DQ_ACC_6D_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_INT2_6D(void *handle, AIS328DQ_ACC_INT2_6D_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT2_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_INT2_6D_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_W_INT2_CFG_AOI
* Description    : Write AOI
* Input          : AIS328DQ_ACC_INT2_AOI_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  AIS328DQ_ACC_W_INT2_CFG_AOI(void *handle, AIS328DQ_ACC_INT2_AOI_t newValue)
{
  u8_t value;

  if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT2_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~AIS328DQ_ACC_INT2_AOI_MASK; 
  value |= newValue;
  
  if( !AIS328DQ_ACC_WriteReg( handle, AIS328DQ_ACC_INT2_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_INT2_CFG_INT2_AOI
* Description    : Read AOI
* Input          : Pointer to AIS328DQ_ACC_INT2_AOI_t
* Output         : Status of AOI see AIS328DQ_ACC_AOI_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_INT2_CFG_AOI(void *handle, AIS328DQ_ACC_INT2_AOI_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT2_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_INT2_AOI_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_INT2_SOURCE_XL
* Description    : Read XL
* Input          : Pointer to AIS328DQ_ACC_INT2_XL_t
* Output         : Status of XL see AIS328DQ_ACC_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_INT2_SOURCE_XL(void *handle, AIS328DQ_ACC_INT2_XL_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT2_SOURCE, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_INT2_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_INT2_SOURCE_XH
* Description    : Read XH
* Input          : Pointer to AIS328DQ_ACC_INT2_XH_t
* Output         : Status of XH see AIS328DQ_ACC_XH_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_INT2_SOURCE_XH(void *handle, AIS328DQ_ACC_INT2_XH_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT2_SOURCE, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_INT2_XH_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_INT2_SOURCE_YL
* Description    : Read YL
* Input          : Pointer to AIS328DQ_ACC_INT2_YL_t
* Output         : Status of YL see AIS328DQ_ACC_YL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_INT2_SOURCE_YL(void *handle, AIS328DQ_ACC_INT2_YL_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT2_SOURCE, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_INT2_YL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_INT2_SOURCE_YH
* Description    : Read YH
* Input          : Pointer to AIS328DQ_ACC_INT2_YH_t
* Output         : Status of YH see AIS328DQ_ACC_YH_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_INT2_SOURCE_YH(void *handle, AIS328DQ_ACC_INT2_YH_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT2_SOURCE, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_INT2_YH_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_INT2_SOURCE_ZL
* Description    : Read ZL
* Input          : Pointer to AIS328DQ_ACC_INT2_ZL_t
* Output         : Status of ZL see AIS328DQ_ACC_ZL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_INT2_SOURCE_ZL(void *handle, AIS328DQ_ACC_INT2_ZL_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT2_SOURCE, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_INT2_ZL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_INT2_SOURCE_ZH
* Description    : Read ZH
* Input          : Pointer to AIS328DQ_ACC_INT2_ZH_t
* Output         : Status of ZH see AIS328DQ_ACC_ZH_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_INT2_SOURCE_ZH(void *handle, AIS328DQ_ACC_INT2_ZH_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT2_SOURCE, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_INT2_ZH_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_INT2_SOURCE_IA
* Description    : Read IA
* Input          : Pointer to AIS328DQ_ACC_INT2_IA_t
* Output         : Status of IA see AIS328DQ_ACC_IA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_INT2_SOURCE_IA(void *handle, AIS328DQ_ACC_INT2_IA_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT2_SOURCE, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_INT2_IA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_W_INT2_THS
* Description    : Write THS
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  AIS328DQ_ACC_W_INT2_THS(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << AIS328DQ_ACC_INT2_THS_POSITION; //mask	
  newValue &= AIS328DQ_ACC_INT2_THS_MASK; //coerce
  
  if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT2_THS, &value, 1) )
    return MEMS_ERROR;

  value &= ~AIS328DQ_ACC_INT2_THS_MASK; 
  value |= newValue;
  
  if( !AIS328DQ_ACC_WriteReg( handle, AIS328DQ_ACC_INT2_THS, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_INT2_THS
* Description    : Read THS
* Input          : Pointer to u8_t
* Output         : Status of THS 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_INT2_THS(void *handle, u8_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT2_THS, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_INT2_THS_MASK; //coerce	
  *value = *value >> AIS328DQ_ACC_INT2_THS_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_W_INT2_DURATION
* Description    : Write D
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  AIS328DQ_ACC_W_INT2_DURATION(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << AIS328DQ_ACC_INT2_D_POSITION; //mask	
  newValue &= AIS328DQ_ACC_INT2_D_MASK; //coerce
  
  if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT2_DURATION, &value, 1) )
    return MEMS_ERROR;

  value &= ~AIS328DQ_ACC_INT2_D_MASK; 
  value |= newValue;
  
  if( !AIS328DQ_ACC_WriteReg( handle, AIS328DQ_ACC_INT2_DURATION, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : AIS328DQ_ACC_R_INT2_DURATION
* Description    : Read D
* Input          : Pointer to u8_t
* Output         : Status of D 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_R_INT2_DURATION(void *handle, u8_t *value)
{
 if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_INT2_DURATION, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= AIS328DQ_ACC_INT2_D_MASK; //coerce	
  *value = *value >> AIS328DQ_ACC_INT2_D_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : status_t AIS328DQ_ACC_Set_Reference(u8_t *buff) 
* Description    : Set Reference data row
* Input          : pointer to [u8_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_Set_Reference(void *handle, u8_t *buff) 
{
  u8_t  i;

  for (i=0; i<1;i++ ) 
  {
	if( !AIS328DQ_ACC_WriteReg( handle, AIS328DQ_ACC_REFERENCE+i,  &buff[i], 1) )
		return MEMS_ERROR;
  }
  return MEMS_SUCCESS;  
}

/*******************************************************************************
* Function Name  : status_t AIS328DQ_ACC_Get_Reference(u8_t *buff)
* Description    : Read Reference output register
* Input          : pointer to [u8_t]
* Output         : Reference buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t AIS328DQ_ACC_Get_Reference(void *handle, u8_t *buff) 
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;
  
  numberOfByteForDimension=1/1;

  k=0;
  for (i=0; i<1;i++ ) 
  {
	for (j=0; j<numberOfByteForDimension;j++ )
	{	
		if( !AIS328DQ_ACC_ReadReg( handle, AIS328DQ_ACC_REFERENCE+k, &buff[k], 1))
		  return MEMS_ERROR;
		k++;	
	}
  }

  return MEMS_SUCCESS; 
}



