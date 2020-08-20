/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
* File Name          : LIS2HH12_driver.c
* Author             : MEMS Application Team
* Version            : v1.1
* Date               : 06 June 2016  
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
#include "LIS2HH12_driver.h"
//#include "i2C_mems.h"


/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/************** Generic Function  *******************/

/*******************************************************************************
* Function Name		: LIS2HH12_ACC_WriteReg
* Description		: Generic Writing function. It must be fullfilled with either
*					: I2C or SPI writing function
* Input				: Register Address, ptr to buffer to be written,
*                                 length of buffer
* Output			: None
* Return			: None
*******************************************************************************/
status_t LIS2HH12_ACC_WriteReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len)
{
   
  //To be completed with either I2c or SPI writing function
  //i.e.: SPI_Mems_Write_Reg(Reg, Data);
  //I2Cx_Write(Bufp, LIS2HH12_ACC_GYRO_I2C_ADDRESS, Reg, len);    //[Example]
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name		: LIS2HH12_ACC_ReadReg
* Description		: Generic Reading function. It must be fullfilled with either
*					: I2C or SPI writing function
* Input				: Register Address, ptr to buffer to be read,
*                                 length of buffer
* Output			: None
* Return			: None
*******************************************************************************/
status_t LIS2HH12_ACC_ReadReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len)
{
  //To be completed with either I2c or SPI writing function
  //i.e.: SPI_Mems_Write_Reg(Reg, Data);
  //I2Cx_Read(Bufp, LIS2HH12_ACC_GYRO_I2C_ADDRESS, Reg, len); //[Example]
  return MEMS_SUCCESS; 
}

/**************** Base Function  *******************/

/*******************************************************************************
* Function Name  : LIS2HH12_GetWHO_AM_I
* Description    : Read identification code by WHO_AM_I register
* Input          : Char to empty by Device identification Value
* Output         : None
* Return         : Status [value of FSS]
*******************************************************************************/
status_t LIS2HH12_GetWHO_AM_I(void *handle, u8_t* val){
  
  if( !LIS2DE12_ACC_ReadReg(handle, LIS2HH12_WHO_AM_I, val, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2HH12_SetODR
* Description    : Sets LIS2HH12 Output Data Rate
* Input          : Set Output data rate [LIS2HH12_ODR_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2HH12_SetODR(void *handle, LIS2HH12_ODR_t ov){
  u8_t value;

  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_CTRL1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS2HH12_ODR_MASK; //mask
  value |= ov;	
	
  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_CTRL1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2HH12_SetFullScale
* Description    : set LIS2HH12 Full scale
* Input          : set LIS2HH12 Full scale [LIS2HH12_FS_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2HH12_SetFullScale(void *handle, LIS2HH12_FS_t ov){
  u8_t value;

  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_CTRL4, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS2HH12_FS_8g; //mask
  value |= ov;	


  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_CTRL4, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2HH12_BlockDataUpdate
* Description    : Enable/Disable LIS2HH12 BlockDataUpdate
* Input          : Enable/Disable LIS2HH12 BlockDataUpdate[LIS2HH12_BDU_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2HH12_BlockDataUpdate(void *handle, LIS2HH12_BDU_t ov){
  u8_t value;

  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_CTRL1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS2HH12_BDU_ENABLE; //mask
  value |= ov;	

  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_CTRL1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2HH12_GetAccRaw
* Description    : Read accelerometer output register
* Input          : pointer to u8_t
* Output         : Acceleration Output Registers buffer 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2HH12_GetAccRaw(void *handle, u8_t* buff) {

  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_OUT_X_H, &buff[1], 1) )
	  return MEMS_ERROR;
  
  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_OUT_X_L, &buff[0], 1) )
	  return MEMS_ERROR;
  
  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_OUT_Y_H, &buff[3], 1) )
	  return MEMS_ERROR;
  
  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_OUT_Y_L, &buff[2], 1) )
	  return MEMS_ERROR;
  
   if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_OUT_Z_H, &buff[5], 1) )
	  return MEMS_ERROR;
  
  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_OUT_Z_L, &buff[4], 1) )
	  return MEMS_ERROR;
  
  return MEMS_SUCCESS;  
}

/**************** Advanced Function  *******************/

/*******************************************************************************
* Function Name  : LIS2HH12_SelfTest
* Description    : Enable/Disable LIS2HH12 Self Test
* Input          : Self test [LIS2HH12_ST_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2HH12_SelfTest(void *handle, LIS2HH12_ST_t ov){
  u8_t value;

  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_CTRL5, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS2HH12_ST_NA; //mask
  value |= ov;	

  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_CTRL5, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2HH12_AxOperativeMode
* Description    : Sets LIS2HH12 Output Data Rate
* Input          : Set Output data rate [LIS2HH12_HR_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//#WARNING: If you enable the HR bit Low pass cut off frequency will change
status_t LIS2HH12_AxOperativeMode(void *handle, LIS2HH12_HR_t ov){
  u8_t value;

  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_CTRL1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS2HH12_HIGH_RES_ON; //mask
  value |= ov;

  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_CTRL1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2HH12_SoftReset
* Description    : Enable/Disable LIS2HH12 SoftReset
* Input          : SoftReset Enable/Disable [LIS2HH12_SOFT_RESET_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2HH12_SoftReset(void *handle, LIS2HH12_SOFT_RESET_t ov){
  u8_t value;

  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_CTRL5, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS2HH12_SOFT_RESET_ENABLE; //mask
  value |= ov;	

  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_CTRL5, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2HH12_SerialInterfaceMode
* Description    : set LIS2HH12 SerialInterfaceMode
* Input          : set LIS2HH12 SerialInterfaceMode [LIS2HH12_SIM_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2HH12_SerialInterfaceMode(void *handle, LIS2HH12_SIM_t ov){
  u8_t value;

  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_CTRL4, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS2HH12_SIM_3WIRE_INTERFACE; //mask
  value |= ov;		


  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_CTRL4, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2HH12_EnableInterruptGeneratorOne
* Description    : Enable/Disable LIS2HH12 interrupt generator one
* Input          : Enable/Disable LIS2HH12 interrupt generator one[LIS2HH12_IG_CONFIG_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//You can use LIS2HH12_IG_CONFIG_t with or condition see DS 4D and 6D interrupt
status_t LIS2HH12_EnableInterruptGeneratorOne(void *handle, LIS2HH12_IG_CONFIG_t ov){
  u8_t valueCTRL7, valueCFG1;
  
 	if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_CTRL7, &valueCTRL7, 1) )
		return MEMS_ERROR; 
	if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_IG_CFG1, &valueCFG1, 1) )
		return MEMS_ERROR;	
 
	if (ov&LIS2HH12_IG_4D)
	{
	  valueCFG1 &= 0x80; //disable all interrupt generation

	  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_IG_CFG1, &valueCFG1, 1) )
		return MEMS_ERROR; 

	  valueCTRL7 |= LIS2HH12_4D_INTGEN1_EN; //enable 4D recognition

	  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_CTRL7, &valueCTRL7, 1) )
		return MEMS_ERROR; 
	}
	else
	{

	  valueCFG1 &= ~0x7F; //enable selected interrupt
	  valueCFG1 |= ov;			

	  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_IG_CFG1, &valueCFG1, 1) )
		return MEMS_ERROR; 

	  valueCTRL7 &= ~LIS2HH12_4D_INTGEN1_EN; //disable 4d recognition

	  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_CTRL7, &valueCTRL7, 1) )
		return MEMS_ERROR; 
	}
    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2HH12_EnableInterruptGeneratorTwo
* Description    : Enable/Disable LIS2HH12 interrupt generator two
* Input          : Enable/Disable LIS2HH12 interrupt generator two[LIS2HH12_IG_CONFIG_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//You can use LIS2HH12_IG_CONFIG_t with or condition see DS 4D and 6D interrupt
status_t LIS2HH12_EnableInterruptGeneratorTwo(void *handle, LIS2HH12_IG_CONFIG_t ov){
  u8_t valueCTRL7, valueCFG2;
  
 	if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_CTRL7, &valueCTRL7, 1) )
		return MEMS_ERROR; 
	if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_IG_CFG2, &valueCFG2, 1) )
		return MEMS_ERROR;	
 
	if (ov&LIS2HH12_IG_4D)
	{
	  valueCFG2 &= 0x80; //disable all interrupt generation

	  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_IG_CFG2, &valueCFG2, 1) )
		return MEMS_ERROR; 

	  valueCTRL7 |= LIS2HH12_4D_INTGEN2_EN; //enable 4D recognition

	  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_CTRL7, &valueCTRL7, 1) )
		return MEMS_ERROR; 
	}
	else
	{
	
	  valueCFG2 &= ~0x7F; //enable selected interrupt
	  valueCFG2 |= ov;	

	  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_IG_CFG2, &valueCFG2, 1) )
		return MEMS_ERROR; 

	  valueCTRL7 &= ~LIS2HH12_4D_INTGEN2_EN; //disable 4d recognition

	  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_CTRL7, &valueCTRL7, 1) )
		return MEMS_ERROR; 
	}
        return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2HH12_InterruptGeneratorOne_LogicCondition
* Description    : LIS2HH12 interrupt generator one LogicCondition
* Input          : LIS2HH12 interrupt generator one LogicCondition[LIS2HH12_AOI_IG_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2HH12_InterruptGeneratorOne_LogicCondition(void *handle, LIS2HH12_AOI_IG_t ov){
  u8_t value;

  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_IG_CFG1, &value, 1) )
    return MEMS_ERROR;

	value &= ~LIS2HH12_AOI_IG_AND; //mask
	value |= ov;	

  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_IG_CFG1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2HH12_InterruptGeneratorTwo_LogicCondition
* Description    : LIS2HH12 interrupt generator two LogicCondition
* Input          : LIS2HH12 interrupt generator two LogicCondition[LIS2HH12_AOI_IG_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2HH12_InterruptGeneratorTwo_LogicCondition(void *handle, LIS2HH12_AOI_IG_t ov){
  u8_t value;

  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_IG_CFG2, &value, 1) )
    return MEMS_ERROR;

	value &= ~LIS2HH12_AOI_IG_AND; //mask
	value |= ov;	
	
  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_IG_CFG2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2HH12_InterruptGeneratorOne_Flag
* Description    : read interrupt one generator flags
* Input          : pointer to LIS2HH12_IG_FLAGS_t
* Output         : LIS2HH12 XYZ Axis data overrun [LIS2HH12_IG_FLAGS_t]
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//You can use LIS2HH12_IG_FLAGS_t with or condition to interpret value
status_t LIS2HH12_InterruptGeneratorOne_Flag(void *handle, LIS2HH12_IG_FLAGS_t *value){

  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_IG_SRC1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= 0x7F; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2HH12_InterruptGeneratorTwo_Flag
* Description    : read interrupt two generator flags
* Input          : pointer to LIS2HH12_IG_FLAGS_t
* Output         : LIS2HH12 XYZ Axis data overrun [LIS2HH12_IG_FLAGS_t]
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//You can use LIS2HH12_IG_FLAGS_t with or condition to interpret value
status_t LIS2HH12_InterruptGeneratorTwo_Flag(void *handle, LIS2HH12_IG_FLAGS_t *value){

  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_IG_SRC2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= 0x7F; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2HH12_InterruptGeneratorOne_Wait
* Description    : Enable/Disable LIS2HH12 interrupt generator one Wait
* Input          : Enable/Disable LIS2HH12 interrupt generator one Wait[LIS2HH12_WAIT_IG_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2HH12_InterruptGeneratorOne_Wait(void *handle, LIS2HH12_WAIT_IG_t ov){
  u8_t value;

  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_IG_DUR1, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS2HH12_WAIT_IG_ON; //mask
  value |= ov;	

  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_IG_DUR1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2HH12_InterruptGeneratorTwo_Wait
* Description    : Enable/Disable LIS2HH12 interrupt generator two Wait
* Input          : Enable/Disable LIS2HH12 interrupt generator two Wait[LIS2HH12_WAIT_IG_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2HH12_InterruptGeneratorTwo_Wait(void *handle, LIS2HH12_WAIT_IG_t ov){
  u8_t value;

  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_IG_DUR2, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS2HH12_WAIT_IG_ON; //mask
  value |= ov;	
  
  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_IG_DUR2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2HH12_EnableInterruptPinOne
* Description    : Enable/Disable LIS2HH12 interrupt pin one
* Input          : Enable/Disable LIS2HH12 interrupt pin one[LIS2HH12_INT1_DRDY_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2HH12_EnableInterruptPinOne(void *handle, LIS2HH12_INT1_DRDY_t ov){
  u8_t value;

  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_CTRL3, &value, 1) )
    return MEMS_ERROR;

  value &= ~0x7F; //mask
  value |= ov;		

  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_CTRL3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2HH12_EnableInterruptPinTwo
* Description    : Enable/Disable LIS2HH12 interrupt pin two
* Input          : Enable/Disable LIS2HH12 interrupt pin two[LIS2HH12_INT2_DRDY_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2HH12_EnableInterruptPinTwo(void *handle, LIS2HH12_INT2_DRDY_t ov){
  u8_t value;

  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_CTRL6, &value, 1) )
    return MEMS_ERROR;

  value &= ~0x3F; //mask
  value |= ov;

  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_CTRL6, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2HH12_Reboot
* Description    : LIS2HH12_Reboot 
* Input          : None
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2HH12_Reboot(void *handle){
  u8_t value;

  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_CTRL6, &value, 1) )
    return MEMS_ERROR;

  value |= LIS2HH12_REBOOT; 

  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_CTRL6, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2HH12_EnableAxis
* Description    : LIS2HH12_EnableAxis
* Input          : Enable/Disable LIS2HH12 axis [LIS2HH12_AXIS_EN_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//You can use LIS2HH12_AXIS_EN_t with or condition
status_t LIS2HH12_EnableAxis(void *handle, u8_t ov){
  u8_t value;

  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_CTRL1, &value, 1) )
    return MEMS_ERROR;

  value &= ~0x07; //mask
  value |= ov;	

  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_CTRL1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2HH12_HighPassFilterMode
* Description    : LIS2HH12_HighPassFilterMode
* Input          : Select LIS2HH12 high pass filter mode [LIS2HH12_HPM_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//Referring DS to HP filter Usage
status_t LIS2HH12_HighPassFilterMode(void *handle, LIS2HH12_HPM_t ov){
  u8_t value;

  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_CTRL2, &value, 1) )
    return MEMS_ERROR;

  value &= ~0x18; //mask
  value |= ov;		

  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_CTRL2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2HH12_I2C_Mode
* Description    : Enable/Disable LIS2HH12 I2C
* Input          : Enable/Disable LIS2HH12 I2C[LIS2HH12_I2C_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2HH12_I2C_Mode(void *handle, LIS2HH12_I2C_t ov){
  u8_t value;

  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_CTRL4, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS2HH12_I2C_DISABLE; //mask
  value |= ov;		

  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_CTRL4, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2HH12_AuotoInc
* Description    : Enable/Disable LIS2HH12 AuotoInc
* Input          : Enable/Disable LIS2HH12 AuotoInc[LIS2HH12_IF_ADD_INC_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2HH12_AuotoInc(void *handle, LIS2HH12_IF_ADD_INC_t ov){
  u8_t value;

  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_CTRL4, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS2HH12_IF_ADD_INC_ENABLE; //mask
  value |= ov;	

  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_CTRL4, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2HH12_Select_Bandwidth
* Description    : LIS2HH12 LIS2HH12_Select_Bandwidth
* Input          : LIS2HH12 LIS2HH12_Select_Bandwidth[LIS2HH12_BW_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2HH12_Select_Bandwidth(void *handle, LIS2HH12_BW_t ov){
  u8_t value;

  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_CTRL4, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS2HH12_BW_50_Hz; //mask
  value |= ov;	

  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_CTRL4, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2HH12_IntPin_Mode
* Description    : LIS2HH12 Interrupt pin mode
* Input          : LIS2HH12 Interrupt pin mode[LIS2HH12_INT_PIN_CFG_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2HH12_IntPin_Mode(void *handle, LIS2HH12_INT_PIN_CFG_t ov){
  u8_t value;

  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_CTRL5, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS2HH12_OPEN_DRAIN_ACTIVE_LOW; //mask
  value |= ov;	

  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_CTRL5, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2HH12_DebugMode
* Description    : Enable/Disable LIS2HH12 Debug Mode
* Input          : Enable/Disable LIS2HH12 Debug Mode[LIS2HH12_DEBUG_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2HH12_DebugMode(void *handle, LIS2HH12_DEBUG_t ov){
  u8_t value;

  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_CTRL5, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS2HH12_DEBUG_ENABLE; //mask
  value |= ov;	

  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_CTRL5, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2HH12_InterruptSignalsMode
* Description    : LIS2HH12 Interrupt Signals Mode
* Input          : LIS2HH12 Interrupt Signals Mode[LIS2HH12_LAT_SIG_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2HH12_InterruptSignalsMode(void *handle, LIS2HH12_LAT_SIG_t ov){
  u8_t value;

  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_CTRL7, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS2HH12_INT1_LAT_INT2_LAT; //mask
  value |= ov;	

  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_CTRL7, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2HH12_ResetInterruptDuration
* Description    : LIS2HH12 Reset Interrupt Duration
* Input          : LIS2HH12 Reset Interrupt Duration[LIS2HH12_RESET_DUR_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2HH12_ResetInterruptDuration(void *handle, LIS2HH12_RESET_DUR_t ov){
  u8_t value;

  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_CTRL7, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS2HH12_DUR1_RST_DUR2_RST; //mask
  value |= ov;	

  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_CTRL7, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2HH12_Status_Flags
* Description    : read LIS2HH12 Status Flags
* Input          : pointer to LIS2HH12_STATUS_FLAGS_t
* Output         : read LIS2HH12 Status Flags [LIS2HH12_STATUS_FLAGS_t]
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//You can use LIS2HH12_STATUS_FLAGS_t with or condition to interpret value
status_t LIS2HH12_Status_Flags(void *handle, u8_t *value){

  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_STATUS, (u8_t *)value, 1) )
    return MEMS_ERROR;
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2HH12_FIFO_Mode
* Description    : LIS2HH12 FIFO Mode
* Input          : LIS2HH12 FIFO Mode[LIS2HH12_FMODE_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//this function enable FIFO to disable use LIS2HH12_FIFO
status_t LIS2HH12_FIFO_Mode(void *handle, LIS2HH12_FMODE_t ov){
  u8_t value;

  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_CTRL3, &value, 1) )
  return MEMS_ERROR;

  value |= LIS2HH12_FIFO_ENABLE; //Enable FIFO

  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_CTRL3, &value, 1) )
    return MEMS_ERROR;
  
  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_FIFO_CTRL, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS2HH12_FMODE_BYPASS_TO_FIFO; //mask
  value |= ov;		

  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_FIFO_CTRL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2HH12_FIFO_Flags
* Description    : read LIS2HH12 FIFO Flags
* Input          : pointer to LIS2HH12_FIFO_FLAGS_t
* Output         : read LIS2HH12 FIFO Flags [LIS2HH12_FIFO_FLAGS_t]
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//You can use LIS2HH12_FIFO_FLAGS_t with or condition to interpret value
status_t LIS2HH12_FIFO_Flags(void *handle, LIS2HH12_FIFO_FLAGS_t *value){

  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_FIFO_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= 0xE0; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2HH12_FIFO_StopAtTh
* Description    : LIS2HH12 FIFO Stop at threshold
* Input          : LIS2HH12 FIFO Stop at threshold[LIS2HH12_STOP_FTH_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2HH12_FIFO_StopAtTh(void *handle, LIS2HH12_STOP_FTH_t ov){
  u8_t value;

  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_CTRL3, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS2HH12_STOP_FTH_EN_FIFO_TH; //mask
  value |= ov;	

  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_CTRL3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2HH12_FIFO
* Description    : Enable/Disable LIS2HH12 FIFO
* Input          : Enable/Disable LIS2HH12 FIFO[LIS2HH12_FIFO_EN_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2HH12_FIFO(void *handle, LIS2HH12_FIFO_EN_t ov){
  u8_t value;

  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_CTRL3, &value, 1) )
    return MEMS_ERROR;

  value &= ~LIS2HH12_FIFO_ENABLE; //mask
  value |= ov;		

  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_CTRL3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2HH12_GetTemperatureRaw
* Description    : Read Temperature output register
* Input          : pointer to u16_t
* Output         : Temperature data row to u16_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2HH12_GetTemperatureRaw(void *handle, u16_t* buff) {
  u8_t valueL;
  u8_t valueH;
	
  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_TEMP_H, &valueL, 1) )
	  return MEMS_ERROR;
  
  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_TEMP_L, &valueH, 1) )
	  return MEMS_ERROR;
  
  *buff = (i16_t)( (valueH << 8) | valueL );

  return MEMS_SUCCESS;  
}

/*******************************************************************************
* Function Name  : LIS2HH12_SetInterrupt1_Threshold_X
* Description    : Set Interrupt Threshold data row
* Input          : threshold row value [u8_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2HH12_SetInterrupt1_Threshold_X(void *handle, u8_t buff) {
  
  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_IG_THS_X1, &buff, 1) )
      return MEMS_ERROR;
  return MEMS_SUCCESS;  
}

/*******************************************************************************
* Function Name  : LIS2HH12_SetInterrupt1_Threshold_Y
* Description    : Set Interrupt Threshold data row
* Input          : threshold row value [u8_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2HH12_SetInterrupt1_Threshold_Y(void *handle, u8_t buff) {
  
  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_IG_THS_Y1, &buff, 1) )
      return MEMS_ERROR;	  
  return MEMS_SUCCESS;  
}

/*******************************************************************************
* Function Name  : LIS2HH12_SetInterrupt1_Threshold_Z
* Description    : Set Interrupt Threshold data row
* Input          : threshold row value [u8_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2HH12_SetInterrupt1_Threshold_Z(void *handle, u8_t buff) {
  
  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_IG_THS_Z1, &buff, 1) )
      return MEMS_ERROR;	  
  return MEMS_SUCCESS;  
}

/*******************************************************************************
* Function Name  : LIS2HH12_SetInterrupt2_Threshold_ZYX
* Description    : Set Interrupt Threshold data row
* Input          : threshold row value [u8_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2HH12_SetInterrupt2_Threshold_ZYX(void *handle, u8_t buff) {
  
  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_IG_THS2, &buff, 1) )
      return MEMS_ERROR;	  
  return MEMS_SUCCESS;  
}

/*******************************************************************************
* Function Name  : LIS2HH12_SetInterrupt1_Duration
* Description    : Set Interrupt Duration data row
* Input          : Duration row value [u8_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//WARNING: value from 0x00->0x7F
status_t LIS2HH12_SetInterrupt1_Duration(void *handle, u8_t buff) {

  u8_t value;
  
  buff |= 0x7F; //coerce

  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_IG_DUR1, &value, 1) )
    return MEMS_ERROR;

  value &= ~0x7F; //mask
  value |= buff;

  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_IG_DUR1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2HH12_SetInterrupt2_Duration
* Description    : Set Interrupt Duration data row
* Input          : Duration row value [u8_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//WARNING: value from 0x00->0x7F
status_t LIS2HH12_SetInterrupt2_Duration(void *handle, u8_t buff) {
  
  u8_t value;
  
  buff |= 0x7F; //coerce

  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_IG_DUR2, &value, 1) )
    return MEMS_ERROR;

  value &= ~0x7F; //mask
  value |= buff;

  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_IG_DUR2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2HH12_SetReferenceValue
* Description    : Set Reference data row
* Input          : Reference row value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2HH12_SetReferenceValue(void *handle, u8_t *buff) {
  
  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_XL_REFERENCE, &buff[0], 1) )
    return MEMS_ERROR;
  
  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_XH_REFERENCE, &buff[1], 1) )
    return MEMS_ERROR;
  
  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_YL_REFERENCE, &buff[2], 1) )
    return MEMS_ERROR;
  
  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_YH_REFERENCE, &buff[3], 1) )
    return MEMS_ERROR;
  
  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_ZL_REFERENCE, &buff[4], 1) )
    return MEMS_ERROR;
  
  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_ZH_REFERENCE, &buff[5], 1) )
    return MEMS_ERROR;	
	  
  return MEMS_SUCCESS;  
}

/*******************************************************************************
* Function Name  : LIS2HH12_GetReferenceValue
* Description    : Get Reference data row
* Input          : Reference row value 
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2HH12_GetReferenceValue(void *handle, u8_t *buff) {
  
  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_XL_REFERENCE, &buff[0], 1) )
	  return MEMS_ERROR;
  
  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_XH_REFERENCE, &buff[1], 1) )
	  return MEMS_ERROR;
  
  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_YL_REFERENCE, &buff[2], 1) )
	  return MEMS_ERROR;
  
  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_YH_REFERENCE, &buff[3], 1) )
	  return MEMS_ERROR;

  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_ZL_REFERENCE, &buff[4], 1) )
	  return MEMS_ERROR;
  
  if( !LIS2HH12_ACC_ReadReg(handle, LIS2HH12_ZH_REFERENCE, &buff[5], 1) )
	  return MEMS_ERROR;
  
  return MEMS_SUCCESS;  
}

/*******************************************************************************
* Function Name  : LIS2HH12_SetActivity_Threshold
* Description    : Set Interrupt Threshold data row
* Input          : threshold row value [u8_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2HH12_SetActivity_Threshold(void *handle, u8_t buff) {
  
  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_ACT_TSH, &buff, 1) )
	  return MEMS_ERROR;	  
  return MEMS_SUCCESS;  
}

/*******************************************************************************
* Function Name  : LIS2HH12_SetActivity_Duration
* Description    : Set Interrupt Threshold data row
* Input          : threshold row value [u8_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2HH12_SetActivity_Duration(void *handle, u8_t buff) {
  
  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_ACT_DUR, &buff, 1) )
	  return MEMS_ERROR;	  
  return MEMS_SUCCESS;  
}

/*******************************************************************************
* Function Name  : LIS2HH12_SetFIFO_threshold
* Description    : FIFO Threshold data row
* Input          : threshold row value [u8_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//WARNING: value from 0x00->0x1F
status_t LIS2HH12_SetFIFO_threshold(void *handle, u8_t buff) {

  u8_t value;
  
  buff &= 0x1F; //coerce

  if (!LIS2HH12_ACC_ReadReg(handle, LIS2HH12_ACT_DUR, &value, 1) )
    return MEMS_ERROR;

  value &= ~0x1F; //mask
  value |= buff;

  if( !LIS2HH12_ACC_WriteReg(handle, LIS2HH12_ACT_DUR, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;  
}

/*******************************************************************************
* Function Name  : LIS2HH12_GetFIFO_StoredData
* Description    : FIFO Stored data row
* Input          : Stored data row value [u8_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//WARNING: value from 0x00->0x1F
status_t LIS2HH12_GetFIFO_StoredData(void *handle, u8_t buff) {

  if (!LIS2HH12_ACC_ReadReg(handle, LIS2HH12_FIFO_SRC, &buff, 1) )
    return MEMS_ERROR;
	
  buff &= 0x1F; //coerce	

  return MEMS_SUCCESS;  
}



