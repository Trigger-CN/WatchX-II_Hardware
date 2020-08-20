/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
* File Name          : LSM303C_ACC_driver.c
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
#include "LSM303C_ACC_driver.h"

//EXAMPLE to fill LSM303C_ACC_ReadReg and LSM303C_ACC_WriteReg
#include "i2c.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/************** Generic Function  *******************/

/*******************************************************************************
* Function Name		: LSM303C_ACC_WriteReg
* Description		: Generic Writing function. It must be fullfilled with either
*					: I2C or SPI writing function
* Input				: Register Address, ptr to buffer to be written,
*                                 length of buffer
* Output			: None
* Return			: None
*******************************************************************************/
status_t LSM303C_ACC_WriteReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len)
{
  HAL_I2C_Mem_Write(&hi2c1, LSM303C_ACC_MEMS_I2C_ADDRESS, Reg, I2C_MEMADD_SIZE_8BIT, 
                       Bufp, len, 1000);
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name		: LSM303C_ACC_ReadReg
* Description		: Generic Reading function. It must be fullfilled with either
*					: I2C or SPI writing function
* Input				: Register Address, ptr to buffer to be read,
*                                 length of buffer
* Output			: None
* Return			: None
*******************************************************************************/
status_t LSM303C_ACC_ReadReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len)
{
  HAL_I2C_Mem_Read(&hi2c1, LSM303C_ACC_MEMS_I2C_ADDRESS, Reg, I2C_MEMADD_SIZE_8BIT, 
                   Bufp, len, 1000);
  return MEMS_SUCCESS; 
}

/**************** Base Function  *******************/

/*******************************************************************************
* Function Name  : LSM303C_ACC_SetODR
* Description    : Sets LIS2HH Output Data Rate
* Input          : Set Output data rate [LSM303C_ACC_ODR_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_ACC_SetODR(void *handle, LSM303C_ACC_ODR_t ov){
  u8_t value;

  if( !LSM303C_ACC_ReadReg(handle, LSM303C_ACC_CTRL1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_ACC_ODR_MASK; //mask
  value |= ov;	
	
  if( !LSM303C_ACC_WriteReg(handle, LSM303C_ACC_CTRL1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_ACC_SetFullScale
* Description    : set LIS2HH Full scale
* Input          : set LIS2HH Full scale [LSM303C_ACC_FS_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_ACC_SetFullScale(void *handle, LSM303C_ACC_FS_t ov){
  u8_t value;

  if( !LSM303C_ACC_ReadReg(handle, LSM303C_ACC_CTRL4, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_ACC_FS_8g; //mask
  value |= ov;	


  if( !LSM303C_ACC_WriteReg(handle, LSM303C_ACC_CTRL4, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_ACC_BlockDataUpdate
* Description    : Enable/Disable LIS2HH BlockDataUpdate
* Input          : Enable/Disable LIS2HH BlockDataUpdate[LSM303C_ACC_BDU_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_ACC_BlockDataUpdate(void *handle, LSM303C_ACC_BDU_t ov){
  u8_t value;

  if( !LSM303C_ACC_ReadReg(handle, LSM303C_ACC_CTRL1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_ACC_BDU_ENABLE; //mask
  value |= ov;	

  if( !LSM303C_ACC_WriteReg(handle, LSM303C_ACC_CTRL1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_ACC_GetAccRaw
* Description    : Read accelerometer output register
* Input          : pointer to AxesRaw_t
* Output         : Acceleration Output Registers buffer AxesRaw_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_ACC_GetAccRaw(void *handle, u8_t* buff) 
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;
  
  numberOfByteForDimension=6/3;

  k=0;
  for (i=0; i<3;i++ ) 
  {
	for (j=0; j<numberOfByteForDimension;j++ )
	{	
		if( !LSM303C_ACC_ReadReg(handle, LSM303C_ACC_OUT_X_L+k, &buff[k], 1))
		  return MEMS_ERROR;
		k++;	
	}
  }
  return MEMS_SUCCESS; 
}

/*******************************************************************************
* Function Name  : LSM303C_ACC_SelfTest
* Description    : Enable/Disable LIS2HH Self Test
* Input          : Self test [LSM303C_ACC_ST_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_ACC_SelfTest(void *handle, LSM303C_ACC_ST_t ov){
  u8_t value;

  if( !LSM303C_ACC_ReadReg(handle, LSM303C_ACC_CTRL5, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_ACC_ST_NA; //mask
  value |= ov;	

  if( !LSM303C_ACC_WriteReg(handle, LSM303C_ACC_CTRL5, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_ACC_AxOperativeMode
* Description    : Sets LIS2HH Output Data Rate
* Input          : Set Output data rate [LSM303C_ACC_HR_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//#WARNING: If you enable the HR bit Low pass cut off frequency will change
status_t LSM303C_ACC_AxOperativeMode(void *handle, LSM303C_ACC_HR_t ov){
  u8_t value;

  if( !LSM303C_ACC_ReadReg(handle, LSM303C_ACC_CTRL1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_ACC_HIGH_RES_ON; //mask
  value |= ov;

  if( !LSM303C_ACC_WriteReg(handle, LSM303C_ACC_CTRL1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_ACC_SoftReset
* Description    : Enable/Disable LIS2HH SoftReset
* Input          : SoftReset Enable/Disable [LSM303C_ACC_SOFT_RESET_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_ACC_SoftReset(void *handle, LSM303C_ACC_SOFT_RESET_t ov){
  u8_t value;

  if( !LSM303C_ACC_ReadReg(handle, LSM303C_ACC_CTRL5, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_ACC_SOFT_RESET_ENABLE; //mask
  value |= ov;	

  if( !LSM303C_ACC_WriteReg(handle, LSM303C_ACC_CTRL5, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_ACC_SerialInterfaceMode
* Description    : set LIS2HH SerialInterfaceMode
* Input          : set LIS2HH SerialInterfaceMode [LSM303C_ACC_SIM_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_ACC_SerialInterfaceMode(void *handle, LSM303C_ACC_SIM_t ov){
  u8_t value;

  if( !LSM303C_ACC_ReadReg(handle, LSM303C_ACC_CTRL4, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_ACC_SIM_3WIRE_INTERFACE; //mask
  value |= ov;		


  if( !LSM303C_ACC_WriteReg(handle, LSM303C_ACC_CTRL4, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_ACC_EnableInterruptGeneratorOne
* Description    : Enable/Disable LIS2HH interrupt generator one
* Input          : Enable/Disable LIS2HH interrupt generator one[LSM303C_ACC_IG_CONFIG_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//You can use LSM303C_ACC_IG_CONFIG_t with or condition see DS 4D and 6D interrupt
status_t LSM303C_ACC_EnableInterruptGeneratorOne(void *handle, LSM303C_ACC_IG_CONFIG_t ov){
  u8_t valueCTRL7, valueCFG1;
  
 	if( !LSM303C_ACC_ReadReg(handle, LSM303C_ACC_CTRL7, &valueCTRL7, 1) )
		return MEMS_ERROR; 
	if( !LSM303C_ACC_ReadReg(handle, LSM303C_ACC_IG_CFG1, &valueCFG1, 1) )
		return MEMS_ERROR;	
 
	if (ov&LSM303C_ACC_IG_4D)
	{
	  valueCFG1 &= 0x80; //disable all interrupt generation

	  if( !LSM303C_ACC_WriteReg(handle, LSM303C_ACC_IG_CFG1, &valueCFG1, 1) )
		return MEMS_ERROR; 

	  valueCTRL7 |= LSM303C_ACC_4D_INTGEN1_EN; //enable 4D recognition

	  if( !LSM303C_ACC_WriteReg(handle, LSM303C_ACC_CTRL7, &valueCTRL7, 1) )
		return MEMS_ERROR; 
	}
	else
	{

	  valueCFG1 &= ~0x7F; //enable selected interrupt
	  valueCFG1 |= ov;			

	  if( !LSM303C_ACC_WriteReg(handle, LSM303C_ACC_IG_CFG1, &valueCFG1, 1) )
		return MEMS_ERROR; 

	  valueCTRL7 &= ~LSM303C_ACC_4D_INTGEN1_EN; //disable 4d recognition

	  if( !LSM303C_ACC_WriteReg(handle, LSM303C_ACC_CTRL7, &valueCTRL7, 1) )
		return MEMS_ERROR; 
	}
        return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_ACC_EnableInterruptGeneratorTwo
* Description    : Enable/Disable LIS2HH interrupt generator two
* Input          : Enable/Disable LIS2HH interrupt generator two[LSM303C_ACC_IG_CONFIG_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//You can use LSM303C_ACC_IG_CONFIG_t with or condition see DS 4D and 6D interrupt
status_t LSM303C_ACC_EnableInterruptGeneratorTwo(void *handle, LSM303C_ACC_IG_CONFIG_t ov){
  u8_t valueCTRL7, valueCFG2;
  
 	if( !LSM303C_ACC_ReadReg(handle, LSM303C_ACC_CTRL7, &valueCTRL7, 1) )
		return MEMS_ERROR; 
	if( !LSM303C_ACC_ReadReg(handle, LSM303C_ACC_IG_CFG2, &valueCFG2, 1) )
		return MEMS_ERROR;	
 
	if (ov&LSM303C_ACC_IG_4D)
	{
	  valueCFG2 &= 0x80; //disable all interrupt generation

	  if( !LSM303C_ACC_WriteReg(handle, LSM303C_ACC_IG_CFG2, &valueCFG2, 1) )
		return MEMS_ERROR; 

	  valueCTRL7 |= LSM303C_ACC_4D_INTGEN2_EN; //enable 4D recognition

	  if( !LSM303C_ACC_WriteReg(handle, LSM303C_ACC_CTRL7, &valueCTRL7, 1) )
		return MEMS_ERROR; 
	}
	else
	{
	
	  valueCFG2 &= ~0x7F; //enable selected interrupt
	  valueCFG2 |= ov;	

	  if( !LSM303C_ACC_WriteReg(handle, LSM303C_ACC_IG_CFG2, &valueCFG2, 1) )
		return MEMS_ERROR; 

	  valueCTRL7 &= ~LSM303C_ACC_4D_INTGEN2_EN; //disable 4d recognition

	  if( !LSM303C_ACC_WriteReg(handle, LSM303C_ACC_CTRL7, &valueCTRL7, 1) )
		return MEMS_ERROR; 
	}
        return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_ACC_InterruptGeneratorOne_LogicCondition
* Description    : LIS2HH interrupt generator one LogicCondition
* Input          : LIS2HH interrupt generator one LogicCondition[LSM303C_ACC_AOI_IG_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_ACC_InterruptGeneratorOne_LogicCondition(void *handle, LSM303C_ACC_AOI_IG_t ov){
  u8_t value;

  if( !LSM303C_ACC_ReadReg(handle, LSM303C_ACC_IG_CFG1, &value, 1) )
    return MEMS_ERROR;

	value &= ~LSM303C_ACC_AOI_IG_AND; //mask
	value |= ov;	

  if( !LSM303C_ACC_WriteReg(handle, LSM303C_ACC_IG_CFG1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_ACC_InterruptGeneratorTwo_LogicCondition
* Description    : LIS2HH interrupt generator two LogicCondition
* Input          : LIS2HH interrupt generator two LogicCondition[LSM303C_ACC_AOI_IG_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_ACC_InterruptGeneratorTwo_LogicCondition(void *handle, LSM303C_ACC_AOI_IG_t ov){
  u8_t value;

  if( !LSM303C_ACC_ReadReg(handle, LSM303C_ACC_IG_CFG2, &value, 1) )
    return MEMS_ERROR;

	value &= ~LSM303C_ACC_AOI_IG_AND; //mask
	value |= ov;	
	
  if( !LSM303C_ACC_WriteReg(handle, LSM303C_ACC_IG_CFG2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_ACC_InterruptGeneratorOne_Flag
* Description    : read interrupt one generator flags
* Input          : pointer to LSM303C_ACC_IG_FLAGS_t
* Output         : LIS2HH XYZ Axis data overrun [LSM303C_ACC_IG_FLAGS_t]
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//You can use LSM303C_ACC_IG_FLAGS_t with or condition to interpret value
status_t LSM303C_ACC_InterruptGeneratorOne_Flag(void *handle, LSM303C_ACC_IG_FLAGS_t *value){

  if( !LSM303C_ACC_ReadReg(handle, LSM303C_ACC_IG_SRC1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= 0x7F; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_ACC_InterruptGeneratorTwo_Flag
* Description    : read interrupt two generator flags
* Input          : pointer to LSM303C_ACC_IG_FLAGS_t
* Output         : LIS2HH XYZ Axis data overrun [LSM303C_ACC_IG_FLAGS_t]
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//You can use LSM303C_ACC_IG_FLAGS_t with or condition to interpret value
status_t LSM303C_ACC_InterruptGeneratorTwo_Flag(void *handle, LSM303C_ACC_IG_FLAGS_t *value){

  if( !LSM303C_ACC_ReadReg(handle, LSM303C_ACC_IG_SRC2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= 0x7F; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_ACC_InterruptGeneratorOne_Wait
* Description    : Enable/Disable LIS2HH interrupt generator one Wait
* Input          : Enable/Disable LIS2HH interrupt generator one Wait[LSM303C_ACC_WAIT_IG_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_ACC_InterruptGeneratorOne_Wait(void *handle, LSM303C_ACC_WAIT_IG_t ov){
  u8_t value;

  if( !LSM303C_ACC_ReadReg(handle, LSM303C_ACC_IG_DUR1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_ACC_WAIT_IG_ON; //mask
  value |= ov;	

  if( !LSM303C_ACC_WriteReg(handle, LSM303C_ACC_IG_DUR1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_ACC_InterruptGeneratorTwo_Wait
* Description    : Enable/Disable LIS2HH interrupt generator two Wait
* Input          : Enable/Disable LIS2HH interrupt generator two Wait[LSM303C_ACC_WAIT_IG_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_ACC_InterruptGeneratorTwo_Wait(void *handle, LSM303C_ACC_WAIT_IG_t ov){
  u8_t value;

  if( !LSM303C_ACC_ReadReg(handle, LSM303C_ACC_IG_DUR2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_ACC_WAIT_IG_ON; //mask
  value |= ov;	
  
  if( !LSM303C_ACC_WriteReg(handle, LSM303C_ACC_IG_DUR2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_ACC_EnableInterruptPinOne
* Description    : Enable/Disable LIS2HH interrupt pin one
* Input          : Enable/Disable LIS2HH interrupt pin one[LSM303C_ACC_INT1_DRDY_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_ACC_EnableInterruptPinOne(void *handle, LSM303C_ACC_INT1_DRDY_t ov){
  u8_t value;

  if( !LSM303C_ACC_ReadReg(handle, LSM303C_ACC_CTRL3, &value, 1) )
    return MEMS_ERROR;

  value &= ~0x7F; //mask
  value |= ov;		

  if( !LSM303C_ACC_WriteReg(handle, LSM303C_ACC_CTRL3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_ACC_EnableInterruptPinTwo
* Description    : Enable/Disable LIS2HH interrupt pin two
* Input          : Enable/Disable LIS2HH interrupt pin two[LSM303C_ACC_INT2_DRDY_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_ACC_EnableInterruptPinTwo(void *handle, LSM303C_ACC_INT2_DRDY_t ov){
  u8_t value;

  if( !LSM303C_ACC_ReadReg(handle, LSM303C_ACC_CTRL6, &value, 1) )
    return MEMS_ERROR;

  value &= ~0x3F; //mask
  value |= ov;

  if( !LSM303C_ACC_WriteReg(handle, LSM303C_ACC_CTRL6, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_ACC_Reboot
* Description    : LSM303C_ACC_Reboot 
* Input          : None
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_ACC_Reboot(void *handle){
  u8_t value;

  if( !LSM303C_ACC_ReadReg(handle, LSM303C_ACC_CTRL6, &value, 1) )
    return MEMS_ERROR;

  value |= LSM303C_ACC_REBOOT; 

  if( !LSM303C_ACC_WriteReg(handle, LSM303C_ACC_CTRL6, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_ACC_EnableAxis
* Description    : LSM303C_ACC_EnableAxis
* Input          : Enable/Disable LIS2HH axis [LSM303C_ACC_AXIS_EN_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//You can use LSM303C_ACC_AXIS_EN_t with or condition
status_t LSM303C_ACC_EnableAxis(void *handle, u8_t ov){
  u8_t value;

  if( !LSM303C_ACC_ReadReg(handle, LSM303C_ACC_CTRL1, &value, 1) )
    return MEMS_ERROR;

  value &= ~0x07; //mask
  value |= ov;	

  if( !LSM303C_ACC_WriteReg(handle, LSM303C_ACC_CTRL1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_ACC_HighPassFilterMode
* Description    : LSM303C_ACC_HighPassFilterMode
* Input          : Select LIS2HH high pass filter mode [LSM303C_ACC_HPM_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//Referring DS to HP filter Usage
status_t LSM303C_ACC_HighPassFilterMode(void *handle, LSM303C_ACC_HPM_t ov){
  u8_t value;

  if( !LSM303C_ACC_ReadReg(handle, LSM303C_ACC_CTRL2, &value, 1) )
    return MEMS_ERROR;

  value &= ~0x18; //mask
  value |= ov;		

  if( !LSM303C_ACC_WriteReg(handle, LSM303C_ACC_CTRL2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_ACC_I2C_Mode
* Description    : Enable/Disable LIS2HH I2C
* Input          : Enable/Disable LIS2HH I2C[LSM303C_ACC_I2C_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_ACC_I2C_Mode(void *handle, LSM303C_ACC_I2C_t ov){
  u8_t value;

  if( !LSM303C_ACC_ReadReg(handle, LSM303C_ACC_CTRL4, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_ACC_I2C_DISABLE; //mask
  value |= ov;		

  if( !LSM303C_ACC_WriteReg(handle, LSM303C_ACC_CTRL4, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_ACC_AuotoInc
* Description    : Enable/Disable LIS2HH AuotoInc
* Input          : Enable/Disable LIS2HH AuotoInc[LSM303C_ACC_IF_ADD_INC_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_ACC_AuotoInc(void *handle, LSM303C_ACC_IF_ADD_INC_t ov){
  u8_t value;

  if( !LSM303C_ACC_ReadReg(handle, LSM303C_ACC_CTRL4, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_ACC_IF_ADD_INC_ENABLE; //mask
  value |= ov;	

  if( !LSM303C_ACC_WriteReg(handle, LSM303C_ACC_CTRL4, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_ACC_Select_Bandwidth
* Description    : LIS2HH LSM303C_ACC_Select_Bandwidth
* Input          : LIS2HH LSM303C_ACC_Select_Bandwidth[LSM303C_ACC_BW_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_ACC_Select_Bandwidth(void *handle, LSM303C_ACC_BW_t ov){
  u8_t value;

  if( !LSM303C_ACC_ReadReg(handle, LSM303C_ACC_CTRL4, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_ACC_BW_50_Hz; //mask
  value |= ov;	

  if( !LSM303C_ACC_WriteReg(handle, LSM303C_ACC_CTRL4, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_ACC_IntPin_Mode
* Description    : LIS2HH Interrupt pin mode
* Input          : LIS2HH Interrupt pin mode[LSM303C_ACC_INT_PIN_CFG_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_ACC_IntPin_Mode(void *handle, LSM303C_ACC_INT_PIN_CFG_t ov){
  u8_t value;

  if( !LSM303C_ACC_ReadReg(handle, LSM303C_ACC_CTRL5, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_ACC_OPEN_DRAIN_ACTIVE_LOW; //mask
  value |= ov;	

  if( !LSM303C_ACC_WriteReg(handle, LSM303C_ACC_CTRL5, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_ACC_DebugMode
* Description    : Enable/Disable LIS2HH Debug Mode
* Input          : Enable/Disable LIS2HH Debug Mode[LSM303C_ACC_DEBUG_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_ACC_DebugMode(void *handle, LSM303C_ACC_DEBUG_t ov){
  u8_t value;

  if( !LSM303C_ACC_ReadReg(handle, LSM303C_ACC_CTRL5, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_ACC_DEBUG_ENABLE; //mask
  value |= ov;	

  if( !LSM303C_ACC_WriteReg(handle, LSM303C_ACC_CTRL5, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_ACC_InterruptSignalsMode
* Description    : LIS2HH Interrupt Signals Mode
* Input          : LIS2HH Interrupt Signals Mode[LSM303C_ACC_LAT_SIG_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_ACC_InterruptSignalsMode(void *handle, LSM303C_ACC_LAT_SIG_t ov){
  u8_t value;

  if( !LSM303C_ACC_ReadReg(handle, LSM303C_ACC_CTRL7, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_ACC_INT1_LAT_INT2_LAT; //mask
  value |= ov;	

  if( !LSM303C_ACC_WriteReg(handle, LSM303C_ACC_CTRL7, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_ACC_ResetInterruptDuration
* Description    : LIS2HH Reset Interrupt Duration
* Input          : LIS2HH Reset Interrupt Duration[LSM303C_ACC_RESET_DUR_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_ACC_ResetInterruptDuration(void *handle, LSM303C_ACC_RESET_DUR_t ov){
  u8_t value;

  if( !LSM303C_ACC_ReadReg(handle, LSM303C_ACC_CTRL7, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_ACC_DUR1_RST_DUR2_RST; //mask
  value |= ov;	

  if( !LSM303C_ACC_WriteReg(handle, LSM303C_ACC_CTRL7, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_ACC_Status_Flags
* Description    : read LIS2HH Status Flags
* Input          : pointer to LSM303C_ACC_STATUS_FLAGS_t
* Output         : read LIS2HH Status Flags [LSM303C_ACC_STATUS_FLAGS_t]
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//You can use LSM303C_ACC_STATUS_FLAGS_t with or condition to interpret value
status_t LSM303C_ACC_Status_Flags(void *handle, u8_t *value){

  if( !LSM303C_ACC_ReadReg(handle, LSM303C_ACC_STATUS, (u8_t *)value, 1) )
    return MEMS_ERROR;
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_ACC_FIFO_Mode
* Description    : LIS2HH FIFO Mode
* Input          : LIS2HH FIFO Mode[LSM303C_ACC_FMODE_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//this function enable FIFO to disable use LSM303C_ACC_FIFO
status_t LSM303C_ACC_FIFO_Mode(void *handle, LSM303C_ACC_FMODE_t ov){
  u8_t value;

  if( !LSM303C_ACC_ReadReg(handle, LSM303C_ACC_CTRL3, &value, 1) )
  return MEMS_ERROR;

  value |= LSM303C_ACC_FIFO_ENABLE; //Enable FIFO

  if( !LSM303C_ACC_WriteReg(handle, LSM303C_ACC_CTRL3, &value, 1) )
    return MEMS_ERROR;
  
  if( !LSM303C_ACC_ReadReg(handle, LSM303C_ACC_FIFO_CTRL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_ACC_FMODE_BYPASS_TO_FIFO; //mask
  value |= ov;		

  if( !LSM303C_ACC_WriteReg(handle, LSM303C_ACC_FIFO_CTRL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_ACC_FIFO_Flags
* Description    : read LIS2HH FIFO Flags
* Input          : pointer to LSM303C_ACC_FIFO_FLAGS_t
* Output         : read LIS2HH FIFO Flags [LSM303C_ACC_FIFO_FLAGS_t]
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//You can use LSM303C_ACC_FIFO_FLAGS_t with or condition to interpret value
status_t LSM303C_ACC_FIFO_Flags(void *handle, LSM303C_ACC_FIFO_FLAGS_t *value){

  if( !LSM303C_ACC_ReadReg(handle, LSM303C_ACC_FIFO_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= 0xE0; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_ACC_FIFO_StopAtTh
* Description    : LIS2HH FIFO Stop at threshold
* Input          : LIS2HH FIFO Stop at threshold[LSM303C_ACC_STOP_FTH_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_ACC_FIFO_StopAtTh(void *handle, LSM303C_ACC_STOP_FTH_t ov){
  u8_t value;

  if( !LSM303C_ACC_ReadReg(handle, LSM303C_ACC_CTRL3, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_ACC_STOP_FTH_EN_FIFO_TH; //mask
  value |= ov;	

  if( !LSM303C_ACC_WriteReg(handle, LSM303C_ACC_CTRL3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_ACC_FIFO
* Description    : Enable/Disable LIS2HH FIFO
* Input          : Enable/Disable LIS2HH FIFO[LSM303C_ACC_FIFO_EN_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_ACC_FIFO(void *handle, LSM303C_ACC_FIFO_EN_t ov){
  u8_t value;

  if( !LSM303C_ACC_ReadReg(handle, LSM303C_ACC_CTRL3, &value, 1) )
    return MEMS_ERROR;

  value &= ~LSM303C_ACC_FIFO_ENABLE; //mask
  value |= ov;		

  if( !LSM303C_ACC_WriteReg(handle, LSM303C_ACC_CTRL3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_ACC_GetTemperatureRaw
* Description    : Read Temperature output register
* Input          : pointer to u16_t
* Output         : Temperature data row to u16_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_ACC_GetTemperatureRaw(void *handle, u16_t* buff) {
  u8_t valueL;
  u8_t valueH;
	
  if( !LSM303C_ACC_ReadReg(handle, LSM303C_ACC_TEMP_H, &valueL, 1) )
	  return MEMS_ERROR;
  
  if( !LSM303C_ACC_ReadReg(handle, LSM303C_ACC_TEMP_L, &valueH, 1) )
	  return MEMS_ERROR;
  
  *buff = (i16_t)( (valueH << 8) | valueL );

  return MEMS_SUCCESS;  
}

/*******************************************************************************
* Function Name  : LSM303C_ACC_SetInterrupt1_Threshold_X
* Description    : Set Interrupt Threshold data row
* Input          : threshold row value [u8_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_ACC_SetInterrupt1_Threshold_X(void *handle, u8_t buff) {
  
  if( !LSM303C_ACC_WriteReg(handle, LSM303C_ACC_IG_THS_X1, &buff, 1) )
      return MEMS_ERROR;
  return MEMS_SUCCESS;  
}

/*******************************************************************************
* Function Name  : LSM303C_ACC_SetInterrupt1_Threshold_Y
* Description    : Set Interrupt Threshold data row
* Input          : threshold row value [u8_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_ACC_SetInterrupt1_Threshold_Y(void *handle, u8_t buff) {
  
  if( !LSM303C_ACC_WriteReg(handle, LSM303C_ACC_IG_THS_Y1, &buff, 1) )
      return MEMS_ERROR;	  
  return MEMS_SUCCESS;  
}

/*******************************************************************************
* Function Name  : LSM303C_ACC_SetInterrupt1_Threshold_Z
* Description    : Set Interrupt Threshold data row
* Input          : threshold row value [u8_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_ACC_SetInterrupt1_Threshold_Z(void *handle, u8_t buff) {
  
  if( !LSM303C_ACC_WriteReg(handle, LSM303C_ACC_IG_THS_Z1, &buff, 1) )
      return MEMS_ERROR;	  
  return MEMS_SUCCESS;  
}

/*******************************************************************************
* Function Name  : LSM303C_ACC_SetInterrupt2_Threshold_ZYX
* Description    : Set Interrupt Threshold data row
* Input          : threshold row value [u8_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_ACC_SetInterrupt2_Threshold_ZYX(void *handle, u8_t buff) {
  
  if( !LSM303C_ACC_WriteReg(handle, LSM303C_ACC_IG_THS2, &buff, 1) )
      return MEMS_ERROR;	  
  return MEMS_SUCCESS;  
}

/*******************************************************************************
* Function Name  : LSM303C_ACC_SetInterrupt1_Duration
* Description    : Set Interrupt Duration data row
* Input          : Duration row value [u8_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//WARNING: value from 0x00->0x7F
status_t LSM303C_ACC_SetInterrupt1_Duration(void *handle, u8_t buff) {

  u8_t value;
  
  buff |= 0x7F; //coerce

  if( !LSM303C_ACC_ReadReg(handle, LSM303C_ACC_IG_DUR1, &value, 1) )
    return MEMS_ERROR;

  value &= ~0x7F; //mask
  value |= buff;

  if( !LSM303C_ACC_WriteReg(handle, LSM303C_ACC_IG_DUR1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_ACC_SetInterrupt2_Duration
* Description    : Set Interrupt Duration data row
* Input          : Duration row value [u8_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//WARNING: value from 0x00->0x7F
status_t LSM303C_ACC_SetInterrupt2_Duration(void *handle, u8_t buff) {
  
  u8_t value;
  
  buff |= 0x7F; //coerce

  if( !LSM303C_ACC_ReadReg(handle, LSM303C_ACC_IG_DUR2, &value, 1) )
    return MEMS_ERROR;

  value &= ~0x7F; //mask
  value |= buff;

  if( !LSM303C_ACC_WriteReg(handle, LSM303C_ACC_IG_DUR2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM303C_ACC_SetReferenceValue
* Description    : Set Reference data row
* Input          : Reference row value [AxesRaw_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_ACC_SetReferenceValue(void *handle, u8_t *buff) 
{
  u8_t  i;

  for (i=0; i<6;i++ ) 
  {
	if( !LSM303C_ACC_WriteReg(handle, LSM303C_ACC_XL_REFERENCE+i, &buff[i], 1) )
          return MEMS_ERROR;
  }
  return MEMS_SUCCESS;  
}

/*******************************************************************************
* Function Name  : LSM303C_ACC_GetReferenceValue
* Description    : Get Reference data row
* Input          : Reference row value [AxesRaw_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_ACC_GetReferenceValue(void *handle, u8_t *buff) 
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;
  
  numberOfByteForDimension=6/3;

  k=0;
  for (i=0; i<3;i++ ) 
  {
	for (j=0; j<numberOfByteForDimension;j++ )
	{	
		if( !LSM303C_ACC_ReadReg(handle, LSM303C_ACC_XL_REFERENCE+k, &buff[k], 1))
		  return MEMS_ERROR;
		k++;	
	}
  }

  return MEMS_SUCCESS; 
  
}

/*******************************************************************************
* Function Name  : LSM303C_ACC_SetActivity_Threshold
* Description    : Set Interrupt Threshold data row
* Input          : threshold row value [u8_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_ACC_SetActivity_Threshold(void *handle, u8_t buff) {
  
  if( !LSM303C_ACC_WriteReg(handle, LSM303C_ACC_ACT_TSH, &buff, 1) )
	  return MEMS_ERROR;	  
  return MEMS_SUCCESS;  
}

/*******************************************************************************
* Function Name  : LSM303C_ACC_SetActivity_Duration
* Description    : Set Interrupt Threshold data row
* Input          : threshold row value [u8_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM303C_ACC_SetActivity_Duration(void *handle, u8_t buff) {
  
  if( !LSM303C_ACC_WriteReg(handle, LSM303C_ACC_ACT_DUR, &buff, 1) )
	  return MEMS_ERROR;	  
  return MEMS_SUCCESS;  
}

/*******************************************************************************
* Function Name  : LSM303C_ACC_SetFIFO_threshold
* Description    : FIFO Threshold data row
* Input          : threshold row value [u8_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//WARNING: value from 0x00->0x1F
status_t LSM303C_ACC_SetFIFO_threshold(void *handle, u8_t buff) {

  u8_t value;
  
  buff &= 0x1F; //coerce

  if (!LSM303C_ACC_ReadReg(handle, LSM303C_ACC_ACT_DUR, &value, 1) )
    return MEMS_ERROR;

  value &= ~0x1F; //mask
  value |= buff;

  if( !LSM303C_ACC_WriteReg(handle, LSM303C_ACC_ACT_DUR, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;  
}


/*******************************************************************************
* Function Name  : LSM303C_ACC_GetFIFO_StoredData
* Description    : FIFO Stored data row
* Input          : Stored data row value [u8_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//WARNING: value from 0x00->0x1F
status_t LSM303C_ACC_GetFIFO_StoredData(void *handle, u8_t buff) {

  if (!LSM303C_ACC_ReadReg(handle, LSM303C_ACC_FIFO_SRC, &buff, 1) )
    return MEMS_ERROR;
	
  buff &= 0x1F; //coerce	

  return MEMS_SUCCESS;  
}


/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
