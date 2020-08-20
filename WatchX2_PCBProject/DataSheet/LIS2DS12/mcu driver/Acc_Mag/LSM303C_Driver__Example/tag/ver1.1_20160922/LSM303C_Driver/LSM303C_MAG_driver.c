/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
* File Name          : LSM303C_MAG_driver.c
* Author             : MEMS Application Team
* Version            : v1.1
* Date               : 22 Nov 2016  
* Description        : LSM303C Platform Independent Driver
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
#include "LSM303C_MAG_driver.h"
#include "i2c.h"											//[Example]

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/************** Generic Function  *******************/

/*******************************************************************************
* Function Name		: LSM303C_MAG_WriteReg
* Description		: Generic Writing function. It must be fullfilled with either
*					: I2C or SPI writing function
* Input				: Register Address, Data to be written
* Output			: None
* Return			: None
*******************************************************************************/
u8_t LSM303C_MAG_WriteReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len)
{
  /*Example*/
  //HAL_I2C_Mem_Write(&hi2c1, LSM303C_MAG_I2C_ADDRESS, Reg, I2C_MEMADD_SIZE_8BIT, 
  //                   Bufp, len, 1000);
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name		: LSM303C_MAG_ReadReg
* Description		: Generic Reading function. It must be fullfilled with either
*					: I2C or SPI reading functions					
* Input				: Register Address
* Output			: Data REad
* Return			: None
*******************************************************************************/
u8_t LSM303C_MAG_ReadReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len)
{
  /*Example*/
  //HAL_I2C_Mem_Read(&hi2c1, LSM303C_MAG_I2C_ADDRESS, Reg, I2C_MEMADD_SIZE_8BIT, 
  //                 Bufp, len, 1000);
  return MEMS_SUCCESS;                                      
}

/**************** Base Function  *******************/

/*******************************************************************************
* Function Name  : LSM303C_MAG_R_WHO_AM_I_
* Description    : Read WHO_AM_I_BIT
* Input          : Pointer to u8_t
* Output         : Status of WHO_AM_I_BIT 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_MAG_R_WHO_AM_I_(void *handle, u8_t *value)
{
 if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_WHO_AM_I_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303C_MAG_WHO_AM_I_BIT_MASK; //coerce	
  *value = *value >> LSM303C_MAG_WHO_AM_I_BIT_POSITION; //mask	

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_W_SystemOperatingMode
* Description    : Write MD
* Input          : LSM303C_MAG_MD_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303C_MAG_W_SystemOperatingMode(void *handle, LSM303C_MAG_MD_t newValue)
{
  u8_t value;

  if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_MAG_MD_MASK; 
  value |= newValue;
  
  if( !LSM303C_MAG_WriteReg(handle, LSM303C_MAG_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_R_SystemOperatingMode
* Description    : Read MD
* Input          : Pointer to LSM303C_MAG_MD_t
* Output         : Status of MD see LSM303C_MAG_MD_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_MAG_R_SystemOperatingMode(void *handle, LSM303C_MAG_MD_t *value)
{
 if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_CTRL_REG3, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303C_MAG_MD_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_W_BlockDataUpdate
* Description    : Write BDU
* Input          : LSM303C_MAG_BDU_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303C_MAG_W_BlockDataUpdate(void *handle, LSM303C_MAG_BDU_t newValue)
{
  u8_t value;

  if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_CTRL_REG5, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_MAG_BDU_MASK; 
  value |= newValue;
  
  if( !LSM303C_MAG_WriteReg(handle, LSM303C_MAG_CTRL_REG5, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_R_BlockDataUpdate
* Description    : Read BDU
* Input          : Pointer to LSM303C_MAG_BDU_t
* Output         : Status of BDU see LSM303C_MAG_BDU_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_MAG_R_BlockDataUpdate(void *handle, LSM303C_MAG_BDU_t *value)
{
 if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_CTRL_REG5, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303C_MAG_BDU_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_W_FullScale
* Description    : Write FS
* Input          : LSM303C_MAG_FS_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303C_MAG_W_FullScale(void *handle, LSM303C_MAG_FS_t newValue)
{
  u8_t value;

  if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_MAG_FS_MASK; 
  value |= newValue;
  
  if( !LSM303C_MAG_WriteReg(handle, LSM303C_MAG_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_R_FullScale
* Description    : Read FS
* Input          : Pointer to LSM303C_MAG_FS_t
* Output         : Status of FS see LSM303C_MAG_FS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_MAG_R_FullScale(void *handle, LSM303C_MAG_FS_t *value)
{
 if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_CTRL_REG2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303C_MAG_FS_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_W_OutputDataRate
* Description    : Write DO
* Input          : LSM303C_MAG_DO_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303C_MAG_W_OutputDataRate(void *handle, LSM303C_MAG_DO_t newValue)
{
  u8_t value;

  if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_MAG_DO_MASK; 
  value |= newValue;
  
  if( !LSM303C_MAG_WriteReg(handle, LSM303C_MAG_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_R_OutputDataRate
* Description    : Read DO
* Input          : Pointer to LSM303C_MAG_DO_t
* Output         : Status of DO see LSM303C_MAG_DO_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_MAG_R_OutputDataRate(void *handle, LSM303C_MAG_DO_t *value)
{
 if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_CTRL_REG1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303C_MAG_DO_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : status_t LSM303C_MAG_Get_Magnetic(u8_t *buff)
* Description    : Read Magnetic output register
* Input          : pointer to [u8_t]
* Output         : Magnetic buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_MAG_Get_Magnetic(void *handle, u8_t *buff) 
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;
  
  numberOfByteForDimension=6/3;

  k=0;
  for (i=0; i<3;i++ ) 
  {
	for (j=0; j<numberOfByteForDimension;j++ )
	{	
		if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_OUTX_L+k, &buff[k], 1))
		  return MEMS_ERROR;
		k++;	
	}
  }

  return MEMS_SUCCESS; 
}

/**************** Advanced Function  *******************/

/*******************************************************************************
* Function Name  : LSM303C_MAG_W_SelfTest
* Description    : Write ST
* Input          : LSM303C_MAG_ST_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303C_MAG_W_SelfTest(void *handle, LSM303C_MAG_ST_t newValue)
{
  u8_t value;

  if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_MAG_ST_MASK; 
  value |= newValue;
  
  if( !LSM303C_MAG_WriteReg(handle, LSM303C_MAG_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_R_SelfTest
* Description    : Read ST
* Input          : Pointer to LSM303C_MAG_ST_t
* Output         : Status of ST see LSM303C_MAG_ST_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_MAG_R_SelfTest(void *handle, LSM303C_MAG_ST_t *value)
{
 if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_CTRL_REG1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303C_MAG_ST_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_W_OperatingModeXY
* Description    : Write OM
* Input          : LSM303C_MAG_OM_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303C_MAG_W_OperatingModeXY(void *handle, LSM303C_MAG_OM_t newValue)
{
  u8_t value;

  if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_MAG_OM_MASK; 
  value |= newValue;
  
  if( !LSM303C_MAG_WriteReg(handle, LSM303C_MAG_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_R_OperatingModeXY
* Description    : Read OM
* Input          : Pointer to LSM303C_MAG_OM_t
* Output         : Status of OM see LSM303C_MAG_OM_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_MAG_R_OperatingModeXY(void *handle, LSM303C_MAG_OM_t *value)
{
 if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_CTRL_REG1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303C_MAG_OM_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_W_TemperatureSensor
* Description    : Write TEMP_EN
* Input          : LSM303C_MAG_TEMP_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303C_MAG_W_TemperatureSensor(void *handle, LSM303C_MAG_TEMP_EN_t newValue)
{
  u8_t value;

  if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_MAG_TEMP_EN_MASK; 
  value |= newValue;
  
  if( !LSM303C_MAG_WriteReg(handle, LSM303C_MAG_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_R_TemperatureSensor
* Description    : Read TEMP_EN
* Input          : Pointer to LSM303C_MAG_TEMP_EN_t
* Output         : Status of TEMP_EN see LSM303C_MAG_TEMP_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_MAG_R_TemperatureSensor(void *handle, LSM303C_MAG_TEMP_EN_t *value)
{
 if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_CTRL_REG1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303C_MAG_TEMP_EN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_W_SoftRST
* Description    : Write SOFT_RST
* Input          : LSM303C_MAG_SOFT_RST_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303C_MAG_W_SoftRST(void *handle, LSM303C_MAG_SOFT_RST_t newValue)
{
  u8_t value;

  if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_MAG_SOFT_RST_MASK; 
  value |= newValue;
  
  if( !LSM303C_MAG_WriteReg(handle, LSM303C_MAG_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_R_SoftRST
* Description    : Read SOFT_RST
* Input          : Pointer to LSM303C_MAG_SOFT_RST_t
* Output         : Status of SOFT_RST see LSM303C_MAG_SOFT_RST_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_MAG_R_SoftRST(void *handle, LSM303C_MAG_SOFT_RST_t *value)
{
 if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_CTRL_REG2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303C_MAG_SOFT_RST_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_W_Reboot
* Description    : Write REBOOT
* Input          : LSM303C_MAG_REBOOT_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303C_MAG_W_Reboot(void *handle, LSM303C_MAG_REBOOT_t newValue)
{
  u8_t value;

  if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_MAG_REBOOT_MASK; 
  value |= newValue;
  
  if( !LSM303C_MAG_WriteReg(handle, LSM303C_MAG_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_R_Reboot
* Description    : Read REBOOT
* Input          : Pointer to LSM303C_MAG_REBOOT_t
* Output         : Status of REBOOT see LSM303C_MAG_REBOOT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_MAG_R_Reboot(void *handle, LSM303C_MAG_REBOOT_t *value)
{
 if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_CTRL_REG2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303C_MAG_REBOOT_MASK; //mask

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LSM303C_MAG_W_SerialInterfaceMode
* Description    : Write SIM
* Input          : LSM303C_MAG_SIM_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303C_MAG_W_SerialInterfaceMode(void *handle, LSM303C_MAG_SIM_t newValue)
{
  u8_t value;

  if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_MAG_SIM_MASK; 
  value |= newValue;
  
  if( !LSM303C_MAG_WriteReg(handle, LSM303C_MAG_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_R_SerialInterfaceMode
* Description    : Read SIM
* Input          : Pointer to LSM303C_MAG_SIM_t
* Output         : Status of SIM see LSM303C_MAG_SIM_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_MAG_R_SerialInterfaceMode(void *handle, LSM303C_MAG_SIM_t *value)
{
 if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_CTRL_REG3, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303C_MAG_SIM_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_W_FastLowPowerXYZ
* Description    : Write LP
* Input          : LSM303C_MAG_LP_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303C_MAG_W_FastLowPowerXYZ(void *handle, LSM303C_MAG_LP_t newValue)
{
  u8_t value;

  if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_MAG_LP_MASK; 
  value |= newValue;
  
  if( !LSM303C_MAG_WriteReg(handle, LSM303C_MAG_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_R_FastLowPowerXYZ
* Description    : Read LP
* Input          : Pointer to LSM303C_MAG_LP_t
* Output         : Status of LP see LSM303C_MAG_LP_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_MAG_R_FastLowPowerXYZ(void *handle, LSM303C_MAG_LP_t *value)
{
 if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_CTRL_REG3, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303C_MAG_LP_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_W_LittleBigEndianInversion
* Description    : Write BLE
* Input          : LSM303C_MAG_BLE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303C_MAG_W_LittleBigEndianInversion(void *handle, LSM303C_MAG_BLE_t newValue)
{
  u8_t value;

  if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_MAG_BLE_MASK; 
  value |= newValue;
  
  if( !LSM303C_MAG_WriteReg(handle, LSM303C_MAG_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_R_LittleBigEndianInversion
* Description    : Read BLE
* Input          : Pointer to LSM303C_MAG_BLE_t
* Output         : Status of BLE see LSM303C_MAG_BLE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_MAG_R_LittleBigEndianInversion(void *handle, LSM303C_MAG_BLE_t *value)
{
 if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_CTRL_REG4, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303C_MAG_BLE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_W_OperatingModeZ
* Description    : Write OMZ
* Input          : LSM303C_MAG_OMZ_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303C_MAG_W_OperatingModeZ(void *handle, LSM303C_MAG_OMZ_t newValue)
{
  u8_t value;

  if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_MAG_OMZ_MASK; 
  value |= newValue;
  
  if( !LSM303C_MAG_WriteReg(handle, LSM303C_MAG_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_R_OperatingModeZ
* Description    : Read OMZ
* Input          : Pointer to LSM303C_MAG_OMZ_t
* Output         : Status of OMZ see LSM303C_MAG_OMZ_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_MAG_R_OperatingModeZ(void *handle, LSM303C_MAG_OMZ_t *value)
{
 if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_CTRL_REG4, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303C_MAG_OMZ_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_R_NewXData
* Description    : Read XDA
* Input          : Pointer to LSM303C_MAG_XDA_t
* Output         : Status of XDA see LSM303C_MAG_XDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_MAG_R_NewXData(void *handle, LSM303C_MAG_XDA_t *value)
{
 if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303C_MAG_XDA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_R_NewYData
* Description    : Read YDA
* Input          : Pointer to LSM303C_MAG_YDA_t
* Output         : Status of YDA see LSM303C_MAG_YDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_MAG_R_NewYData(void *handle, LSM303C_MAG_YDA_t *value)
{
 if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303C_MAG_YDA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_R_NewZData
* Description    : Read ZDA
* Input          : Pointer to LSM303C_MAG_ZDA_t
* Output         : Status of ZDA see LSM303C_MAG_ZDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_MAG_R_NewZData(void *handle, LSM303C_MAG_ZDA_t *value)
{
 if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303C_MAG_ZDA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_R_NewXYZData
* Description    : Read ZYXDA
* Input          : Pointer to LSM303C_MAG_ZYXDA_t
* Output         : Status of ZYXDA see LSM303C_MAG_ZYXDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_MAG_R_NewXYZData(void *handle, LSM303C_MAG_ZYXDA_t *value)
{
 if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303C_MAG_ZYXDA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_R_DataXOverrun
* Description    : Read XOR
* Input          : Pointer to LSM303C_MAG_XOR_t
* Output         : Status of XOR see LSM303C_MAG_XOR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_MAG_R_DataXOverrun(void *handle, LSM303C_MAG_XOR_t *value)
{
 if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303C_MAG_XOR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_R_DataYOverrun
* Description    : Read YOR
* Input          : Pointer to LSM303C_MAG_YOR_t
* Output         : Status of YOR see LSM303C_MAG_YOR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_MAG_R_DataYOverrun(void *handle, LSM303C_MAG_YOR_t *value)
{
 if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303C_MAG_YOR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_R_DataZOverrun
* Description    : Read ZOR
* Input          : Pointer to LSM303C_MAG_ZOR_t
* Output         : Status of ZOR see LSM303C_MAG_ZOR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_MAG_R_DataZOverrun(void *handle, LSM303C_MAG_ZOR_t *value)
{
 if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303C_MAG_ZOR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_R_DataXYZOverrun
* Description    : Read ZYXOR
* Input          : Pointer to LSM303C_MAG_ZYXOR_t
* Output         : Status of ZYXOR see LSM303C_MAG_ZYXOR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_MAG_R_DataXYZOverrun(void *handle, LSM303C_MAG_ZYXOR_t *value)
{
 if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303C_MAG_ZYXOR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_W_InterruptEnable
* Description    : Write IEN
* Input          : LSM303C_MAG_IEN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303C_MAG_W_InterruptEnable(void *handle, LSM303C_MAG_IEN_t newValue)
{
  u8_t value;

  if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_INT_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_MAG_IEN_MASK; 
  value |= newValue;
  
  if( !LSM303C_MAG_WriteReg(handle, LSM303C_MAG_INT_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_R_InterruptEnable
* Description    : Read IEN
* Input          : Pointer to LSM303C_MAG_IEN_t
* Output         : Status of IEN see LSM303C_MAG_IEN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_MAG_R_InterruptEnable(void *handle, LSM303C_MAG_IEN_t *value)
{
 if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_INT_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303C_MAG_IEN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_W_LatchInterruptRq
* Description    : Write LIR
* Input          : LSM303C_MAG_LIR_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303C_MAG_W_LatchInterruptRq(void *handle, LSM303C_MAG_LIR_t newValue)
{
  u8_t value;

  if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_INT_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_MAG_LIR_MASK; 
  value |= newValue;
  
  if( !LSM303C_MAG_WriteReg(handle, LSM303C_MAG_INT_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_R_LatchInterruptRq
* Description    : Read LIR
* Input          : Pointer to LSM303C_MAG_LIR_t
* Output         : Status of LIR see LSM303C_MAG_LIR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_MAG_R_LatchInterruptRq(void *handle, LSM303C_MAG_LIR_t *value)
{
 if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_INT_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303C_MAG_LIR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_W_InterruptActive
* Description    : Write IEA
* Input          : LSM303C_MAG_IEA_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303C_MAG_W_InterruptActive(void *handle, LSM303C_MAG_IEA_t newValue)
{
  u8_t value;

  if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_INT_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_MAG_IEA_MASK; 
  value |= newValue;
  
  if( !LSM303C_MAG_WriteReg(handle, LSM303C_MAG_INT_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_R_InterruptActive
* Description    : Read IEA
* Input          : Pointer to LSM303C_MAG_IEA_t
* Output         : Status of IEA see LSM303C_MAG_IEA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_MAG_R_InterruptActive(void *handle, LSM303C_MAG_IEA_t *value)
{
 if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_INT_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303C_MAG_IEA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_W_InterruptOnZ
* Description    : Write ZIEN
* Input          : LSM303C_MAG_ZIEN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303C_MAG_W_InterruptOnZ(void *handle, LSM303C_MAG_ZIEN_t newValue)
{
  u8_t value;

  if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_INT_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_MAG_ZIEN_MASK; 
  value |= newValue;
  
  if( !LSM303C_MAG_WriteReg(handle, LSM303C_MAG_INT_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_R_InterruptOnZ
* Description    : Read ZIEN
* Input          : Pointer to LSM303C_MAG_ZIEN_t
* Output         : Status of ZIEN see LSM303C_MAG_ZIEN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_MAG_R_InterruptOnZ(void *handle, LSM303C_MAG_ZIEN_t *value)
{
 if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_INT_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303C_MAG_ZIEN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_W_InterruptOnY
* Description    : Write YIEN
* Input          : LSM303C_MAG_YIEN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303C_MAG_W_InterruptOnY(void *handle, LSM303C_MAG_YIEN_t newValue)
{
  u8_t value;

  if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_INT_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_MAG_YIEN_MASK; 
  value |= newValue;
  
  if( !LSM303C_MAG_WriteReg(handle, LSM303C_MAG_INT_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_R_InterruptOnY
* Description    : Read YIEN
* Input          : Pointer to LSM303C_MAG_YIEN_t
* Output         : Status of YIEN see LSM303C_MAG_YIEN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_MAG_R_InterruptOnY(void *handle, LSM303C_MAG_YIEN_t *value)
{
 if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_INT_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303C_MAG_YIEN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_W_InterruptOnX
* Description    : Write XIEN
* Input          : LSM303C_MAG_XIEN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303C_MAG_W_InterruptOnX(void *handle, LSM303C_MAG_XIEN_t newValue)
{
  u8_t value;

  if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_INT_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_MAG_XIEN_MASK; 
  value |= newValue;
  
  if( !LSM303C_MAG_WriteReg(handle, LSM303C_MAG_INT_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_R_InterruptOnX
* Description    : Read XIEN
* Input          : Pointer to LSM303C_MAG_XIEN_t
* Output         : Status of XIEN see LSM303C_MAG_XIEN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_MAG_R_InterruptOnX(void *handle, LSM303C_MAG_XIEN_t *value)
{
 if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_INT_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303C_MAG_XIEN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_W_InterruptFlag
* Description    : Write INT
* Input          : LSM303C_MAG_INT_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303C_MAG_W_InterruptFlag(void *handle, LSM303C_MAG_INT_t newValue)
{
  u8_t value;

  if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_MAG_INT_MASK; 
  value |= newValue;
  
  if( !LSM303C_MAG_WriteReg(handle, LSM303C_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_R_InterruptFlag
* Description    : Read INT
* Input          : Pointer to LSM303C_MAG_INT_t
* Output         : Status of INT see LSM303C_MAG_INT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_MAG_R_InterruptFlag(void *handle, LSM303C_MAG_INT_t *value)
{
 if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_INT_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303C_MAG_INT_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_W_MagneticFieldOverflow
* Description    : Write MROI
* Input          : LSM303C_MAG_MROI_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303C_MAG_W_MagneticFieldOverflow(void *handle, LSM303C_MAG_MROI_t newValue)
{
  u8_t value;

  if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_MAG_MROI_MASK; 
  value |= newValue;
  
  if( !LSM303C_MAG_WriteReg(handle, LSM303C_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_R_MagneticFieldOverflow
* Description    : Read MROI
* Input          : Pointer to LSM303C_MAG_MROI_t
* Output         : Status of MROI see LSM303C_MAG_MROI_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_MAG_R_MagneticFieldOverflow(void *handle, LSM303C_MAG_MROI_t *value)
{
 if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_INT_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303C_MAG_MROI_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_W_NegativeThresholdFlagZ
* Description    : Write NTH_Z
* Input          : LSM303C_MAG_NTH_Z_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303C_MAG_W_NegativeThresholdFlagZ(void *handle, LSM303C_MAG_NTH_Z_t newValue)
{
  u8_t value;

  if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_MAG_NTH_Z_MASK; 
  value |= newValue;
  
  if( !LSM303C_MAG_WriteReg(handle, LSM303C_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_R_NegativeThresholdFlagZ
* Description    : Read NTH_Z
* Input          : Pointer to LSM303C_MAG_NTH_Z_t
* Output         : Status of NTH_Z see LSM303C_MAG_NTH_Z_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_MAG_R_NegativeThresholdFlagZ(void *handle, LSM303C_MAG_NTH_Z_t *value)
{
 if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_INT_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303C_MAG_NTH_Z_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_W_NegativeThresholdFlagY
* Description    : Write NTH_Y
* Input          : LSM303C_MAG_NTH_Y_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303C_MAG_W_NegativeThresholdFlagY(void *handle, LSM303C_MAG_NTH_Y_t newValue)
{
  u8_t value;

  if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_MAG_NTH_Y_MASK; 
  value |= newValue;
  
  if( !LSM303C_MAG_WriteReg(handle, LSM303C_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_R_NegativeThresholdFlagY
* Description    : Read NTH_Y
* Input          : Pointer to LSM303C_MAG_NTH_Y_t
* Output         : Status of NTH_Y see LSM303C_MAG_NTH_Y_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_MAG_R_NegativeThresholdFlagY(void *handle, LSM303C_MAG_NTH_Y_t *value)
{
 if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_INT_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303C_MAG_NTH_Y_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_W_NegativeThresholdFlagX
* Description    : Write NTH_X
* Input          : LSM303C_MAG_NTH_X_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303C_MAG_W_NegativeThresholdFlagX(void *handle, LSM303C_MAG_NTH_X_t newValue)
{
  u8_t value;

  if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_MAG_NTH_X_MASK; 
  value |= newValue;
  
  if( !LSM303C_MAG_WriteReg(handle, LSM303C_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_R_NegativeThresholdFlagX
* Description    : Read NTH_X
* Input          : Pointer to LSM303C_MAG_NTH_X_t
* Output         : Status of NTH_X see LSM303C_MAG_NTH_X_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_MAG_R_NegativeThresholdFlagX(void *handle, LSM303C_MAG_NTH_X_t *value)
{
 if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_INT_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303C_MAG_NTH_X_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_W_PositiveThresholdFlagZ
* Description    : Write PTH_Z
* Input          : LSM303C_MAG_PTH_Z_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303C_MAG_W_PositiveThresholdFlagZ(void *handle, LSM303C_MAG_PTH_Z_t newValue)
{
  u8_t value;

  if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_MAG_PTH_Z_MASK; 
  value |= newValue;
  
  if( !LSM303C_MAG_WriteReg(handle, LSM303C_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_R_PositiveThresholdFlagZ
* Description    : Read PTH_Z
* Input          : Pointer to LSM303C_MAG_PTH_Z_t
* Output         : Status of PTH_Z see LSM303C_MAG_PTH_Z_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_MAG_R_PositiveThresholdFlagZ(void *handle, LSM303C_MAG_PTH_Z_t *value)
{
 if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_INT_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303C_MAG_PTH_Z_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_W_PositiveThresholdFlagY
* Description    : Write PTH_Y
* Input          : LSM303C_MAG_PTH_Y_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303C_MAG_W_PositiveThresholdFlagY(void *handle, LSM303C_MAG_PTH_Y_t newValue)
{
  u8_t value;

  if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_MAG_PTH_Y_MASK; 
  value |= newValue;
  
  if( !LSM303C_MAG_WriteReg(handle, LSM303C_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_R_PositiveThresholdFlagY
* Description    : Read PTH_Y
* Input          : Pointer to LSM303C_MAG_PTH_Y_t
* Output         : Status of PTH_Y see LSM303C_MAG_PTH_Y_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_MAG_R_PositiveThresholdFlagY(void *handle, LSM303C_MAG_PTH_Y_t *value)
{
 if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_INT_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303C_MAG_PTH_Y_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_W_PositiveThresholdFlagX
* Description    : Write PTH_X
* Input          : LSM303C_MAG_PTH_X_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM303C_MAG_W_PositiveThresholdFlagX(void *handle, LSM303C_MAG_PTH_X_t newValue)
{
  u8_t value;

  if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_MAG_PTH_X_MASK; 
  value |= newValue;
  
  if( !LSM303C_MAG_WriteReg(handle, LSM303C_MAG_INT_SRC, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_MAG_R_PositiveThresholdFlagX
* Description    : Read PTH_X
* Input          : Pointer to LSM303C_MAG_PTH_X_t
* Output         : Status of PTH_X see LSM303C_MAG_PTH_X_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_MAG_R_PositiveThresholdFlagX(void *handle, LSM303C_MAG_PTH_X_t *value)
{
 if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_INT_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= LSM303C_MAG_PTH_X_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : status_t LSM303C_MAG_Get_Temperature(u8_t *buff)
* Description    : Read Temperature output register
* Input          : pointer to [u8_t]
* Output         : Temperature buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_MAG_Get_Temperature(void *handle, u8_t *buff) 
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;
  
  numberOfByteForDimension=2/1;

  k=0;
  for (i=0; i<1;i++ ) 
  {
	for (j=0; j<numberOfByteForDimension;j++ )
	{	
		if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_TEMP_OUT_L+k, &buff[k], 1))
		  return MEMS_ERROR;
		k++;	
	}
  }

  return MEMS_SUCCESS; 
}


/*******************************************************************************
* Function Name  : status_t LSM303C_MAG_Set_MagneticThreshold(u8_t *buff) 
* Description    : Set MagneticThreshold data row
* Input          : pointer to [u8_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_MAG_Set_MagneticThreshold(void *handle, u8_t *buff) 
{
  u8_t  i;

  for (i=0; i<2;i++ ) 
  {
	if( !LSM303C_MAG_WriteReg(handle, LSM303C_MAG_INT_THS_L+i,  &buff[i], 1) )
		return MEMS_ERROR;
  }
  return MEMS_SUCCESS;  
}

/*******************************************************************************
* Function Name  : status_t LSM303C_MAG_Get_MagneticThreshold(u8_t *buff)
* Description    : Read MagneticThreshold output register
* Input          : pointer to [u8_t]
* Output         : MagneticThreshold buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_MAG_Get_MagneticThreshold(void *handle, u8_t *buff) 
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;
  
  numberOfByteForDimension=2/1;

  k=0;
  for (i=0; i<1;i++ ) 
  {
	for (j=0; j<numberOfByteForDimension;j++ )
	{	
		if( !LSM303C_MAG_ReadReg(handle, LSM303C_MAG_INT_THS_L+k, &buff[k], 1))
		  return MEMS_ERROR;
		k++;	
	}
  }

  return MEMS_SUCCESS; 
}

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
void LSM303C_MAG_SwapHighLowByte(u8_t *bufferToSwap, u8_t numberOfByte, u8_t dimension)
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