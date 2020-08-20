/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
* File Name          : H3LIS331DL_ACC_driver.c
* Author             : MEMS Application Team
* Version            : v1.1
* Date               : 26 Feb 2016  
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
#include "H3LIS331DL_ACC_driver.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/************** Generic Function  *******************/

/*******************************************************************************
* Function Name		: H3LIS331DL_ACC_WriteReg
* Description		: Generic Writing function. It must be fullfilled with either
*					: I2C or SPI writing function
* Input				: Register Address, ptr to buffer to be written,
*                                 length of buffer
* Output			: None
* Return			: None
*******************************************************************************/
status_t H3LIS331DL_ACC_WriteReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len)
{
   
  //To be completed with either I2c or SPI writing function
  //i.e.: SPI_Mems_Write_Reg(Reg, Data);
  //I2Cx_Write(Bufp, LSM6DS3H_ACC_GYRO_I2C_ADDRESS, Reg, len);    //[Example]
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name		: H3LIS331DL_ACC_ReadReg
* Description		: Generic Reading function. It must be fullfilled with either
*					: I2C or SPI writing function
* Input				: Register Address, ptr to buffer to be read,
*                                 length of buffer
* Output			: None
* Return			: None
*******************************************************************************/
status_t H3LIS331DL_ACC_ReadReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len)
{
    
  //To be completed with either I2c or SPI writing function
  //i.e.: SPI_Mems_Write_Reg(Reg, Data);
  //I2Cx_Read(Bufp, LSM6DS3H_ACC_GYRO_I2C_ADDRESS, Reg, len); //[Example]
  return MEMS_SUCCESS; 
}

/**************** Base Function  *******************/

/*******************************************************************************
* Function Name  : H3LIS331DL_GetWHO_AM_I
* Description    : Read identification code from H3LIS331DL_WHO_AM_I register
* Input          : char to be filled with the Device identification Value
* Output         : None
* Return         : Status [value of FSS]
*******************************************************************************/
status_t H3LIS331DL_GetWHO_AM_I(void *handle, u8_t* val){
  if( !H3LIS331DL_ACC_ReadReg ( handle, H3LIS331DL_WHO_AM_I, val, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : H3LIS331DL_SetBDU
* Description    : Enable/Disable Block Data Update Functionality
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_SetBDU(void *handle, State_t bdu) {
  u8_t value;
  
  if( !H3LIS331DL_ACC_ReadReg ( handle, H3LIS331DL_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;
  
  value &= 0x7F;
  value |= (bdu<<H3LIS331DL_BDU);
  
  if( !H3LIS331DL_ACC_WriteReg ( handle, H3LIS331DL_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : H3LIS331DL_SetFullScale
* Description    : Sets the H3LIS331DL FullScale
* Input          : H3LIS331DL_FULLSCALE_2/H3LIS331DL_FULLSCALE_4/H3LIS331DL_FULLSCALE_8
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_SetFullScale(void *handle, H3LIS331DL_Fullscale_t fs) {
  u8_t value;
  
  if( !H3LIS331DL_ACC_ReadReg ( handle, H3LIS331DL_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;
  
  value &= 0xCF;	
  value |= (fs<<H3LIS331DL_FS);
  
  if( !H3LIS331DL_ACC_WriteReg ( handle, H3LIS331DL_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : H3LIS331DL_GetAccAxesRaw
* Description    : Read the Acceleration Values Output Registers
* Input          : buffer to empity by AccAxesRaw_t Typedef
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_GetAccAxesRaw(void *handle, AxesRaw_t* buff) {
  i16_t value;
  u8_t *valueL = (u8_t *)(&value);
  u8_t *valueH = ((u8_t *)(&value)+1);
  
  if( !H3LIS331DL_ACC_ReadReg ( handle, H3LIS331DL_OUT_X_L, valueL, 1) )
    return MEMS_ERROR;
  
  if( !H3LIS331DL_ACC_ReadReg ( handle, H3LIS331DL_OUT_X_H, valueH, 1) )
    return MEMS_ERROR;
  
  buff->AXIS_X = value/16;
  
  if( !H3LIS331DL_ACC_ReadReg ( handle, H3LIS331DL_OUT_Y_L, valueL, 1) )
    return MEMS_ERROR;
  
  if( !H3LIS331DL_ACC_ReadReg ( handle, H3LIS331DL_OUT_Y_H, valueH, 1) )
    return MEMS_ERROR;
  
  buff->AXIS_Y = value/16;
  
  if( !H3LIS331DL_ACC_ReadReg ( handle, H3LIS331DL_OUT_Z_L, valueL, 1) )
    return MEMS_ERROR;
  
  if( !H3LIS331DL_ACC_ReadReg ( handle, H3LIS331DL_OUT_Z_H, valueH, 1) )
    return MEMS_ERROR;
  
  buff->AXIS_Z = value/16;
  
  return MEMS_SUCCESS;  
}

/*******************************************************************************
* Function Name  : H3LIS331DL_SetODR
* Description    : Sets H3LIS331DL Accelerometer Output Data Rate 
* Input          : Output Data Rate
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_SetODR(void *handle, H3LIS331DL_ODR_t dr){
  u8_t value;
  
  if( !H3LIS331DL_ACC_ReadReg ( handle, H3LIS331DL_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;
  
  value &= 0xE7;
  value |= dr<<H3LIS331DL_DR;
  
  if( !H3LIS331DL_ACC_WriteReg ( handle, H3LIS331DL_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/**************** Advanced Function  *******************/

/*******************************************************************************
* Function Name  : H3LIS331DL_SetMode
* Description    : Sets H3LIS331DL Accelerometer Operating Mode
* Input          : Modality (H3LIS331DL_LOW_POWER, H3LIS331DL_NORMAL, H3LIS331DL_POWER_DOWN...)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_SetMode(void *handle, H3LIS331DL_Mode_t pm) {
  u8_t value;
  
  if( !H3LIS331DL_ACC_ReadReg ( handle, H3LIS331DL_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;
  
  value &= 0x1F;
  value |= (pm<<H3LIS331DL_PM);   
  
  if( !H3LIS331DL_ACC_WriteReg ( handle, H3LIS331DL_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : H3LIS331DL_SetAxis
* Description    : Enable/Disable H3LIS331DL Axis
* Input          : H3LIS331DL_X_ENABLE/H3LIS331DL_X_DISABLE | H3LIS331DL_Y_ENABLE/H3LIS331DL_Y_DISABLE
                   | H3LIS331DL_Z_ENABLE/H3LIS331DL_Z_DISABLE
* Output         : None
* Note           : You MUST use all input variable in the argument, as example
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_SetAxis(void *handle, H3LIS331DL_Axis_t axis) {
  u8_t value;
  
  if( !H3LIS331DL_ACC_ReadReg ( handle, H3LIS331DL_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;
  
  value &= 0xF8;
  value |= (0x07 & axis);
  
  if( !H3LIS331DL_ACC_WriteReg ( handle, H3LIS331DL_CTRL_REG1, &value, 1) )
    return MEMS_ERROR;   
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : H3LIS331DL_SetBLE
* Description    : Set Endianess (MSB/LSB)
* Input          : H3LIS331DL_BLE_LSB / H3LIS331DL_BLE_MSB
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_SetBLE(void *handle, H3LIS331DL_Endianess_t ble) {
  u8_t value;
  
  if( !H3LIS331DL_ACC_ReadReg ( handle, H3LIS331DL_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;
  
  value &= 0xBF;	
  value |= (ble<<H3LIS331DL_BLE);
  
  if( !H3LIS331DL_ACC_WriteReg ( handle, H3LIS331DL_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : H3LIS331DL_SetFDS
* Description    : Set Filter Data Selection
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_SetFDS(void *handle, State_t fds) {
  u8_t value;
  
  if( !H3LIS331DL_ACC_ReadReg ( handle, H3LIS331DL_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;
  
  value &= 0xEF;	
  value |= (fds<<H3LIS331DL_FDS);
  
  if( !H3LIS331DL_ACC_WriteReg ( handle, H3LIS331DL_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : H3LIS331DL_SetBOOT
* Description    : Rebot memory content
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_SetBOOT(void *handle, State_t boot) {
  u8_t value;
  
  if( !H3LIS331DL_ACC_ReadReg ( handle, H3LIS331DL_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;
  
  value &= 0x7F;	
  value |= (boot<<H3LIS331DL_BOOT);
  
  if( !H3LIS331DL_ACC_WriteReg ( handle, H3LIS331DL_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : H3LIS331DL_SetSelfTest
* Description    : Set Self Test Modality
* Input          : MEMS_DISABLE/MEMS_ENABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_SetSelfTest(void *handle, State_t st) {
  u8_t value;
  
  if( !H3LIS331DL_ACC_ReadReg ( handle, H3LIS331DL_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;
  
  value &= 0xFD;
  value |= (st<<H3LIS331DL_ST);
  
  if( !H3LIS331DL_ACC_WriteReg ( handle, H3LIS331DL_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : H3LIS331DL_SetSelfTestSign
* Description    : Set Self Test Sign (Disable = st_plus, Enable = st_minus)
* Input          : MEMS_DISABLE/MEMS_ENABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_SetSelfTestSign(void *handle, State_t st_sign) {
  u8_t value;
  
  if( !H3LIS331DL_ACC_ReadReg ( handle, H3LIS331DL_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;
  
  value &= 0xF7;
  value |= (st_sign<<H3LIS331DL_ST_SIGN);
  
  if( !H3LIS331DL_ACC_WriteReg ( handle, H3LIS331DL_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : H3LIS331DL_SetIntHighLow
* Description    : Set Interrupt active state (Disable = active high, Enable = active low)
* Input          : MEMS_DISABLE/MEMS_ENABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_SetIntHighLow(void *handle, State_t ihl) {
  u8_t value;
  
  if( !H3LIS331DL_ACC_ReadReg ( handle, H3LIS331DL_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;
  
  value &= 0x7F;
  value |= (ihl<<H3LIS331DL_IHL);
  
  if( !H3LIS331DL_ACC_WriteReg ( handle, H3LIS331DL_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : H3LIS331DL_SetIntPPOD
* Description    : Set Interrupt Push-Pull/OpenDrain Pad (Disable = Push-Pull, Enable = OpenDrain)
* Input          : MEMS_DISABLE/MEMS_ENABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_SetIntPPOD(void *handle, State_t pp_od) {
  u8_t value;
  
  if( !H3LIS331DL_ACC_ReadReg ( handle, H3LIS331DL_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;
  
  value &= 0xBF;
  value |= (pp_od<<H3LIS331DL_PP_OD);
  
  if( !H3LIS331DL_ACC_WriteReg ( handle, H3LIS331DL_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : H3LIS331DL_SetInt1DataSign
* Description    : Set Data signal Interrupt 1 pad
* Input          : Modality by H3LIS331DL_INT_Conf_t Typedef 
                  (H3LIS331DL_INT_SOURCE, H3LIS331DL_INT_1OR2_SOURCE, H3LIS331DL_DATA_READY, H3LIS331DL_BOOT_RUNNING)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_SetInt1DataSign(void *handle, H3LIS331DL_INT_Conf_t i_cfg) {
  u8_t value;
  
  if( !H3LIS331DL_ACC_ReadReg ( handle, H3LIS331DL_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;
  
  value &= 0xFC;
  value |= (i_cfg<<H3LIS331DL_I1_CFG);
  
  if( !H3LIS331DL_ACC_WriteReg ( handle, H3LIS331DL_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : H3LIS331DL_SetInt2DataSign
* Description    : Set Data signal Interrupt 2 pad
* Input          : Modality by H3LIS331DL_INT_Conf_t Typedef 
                  (H3LIS331DL_INT_SOURCE, H3LIS331DL_INT_1OR2_SOURCE, H3LIS331DL_DATA_READY, H3LIS331DL_BOOT_RUNNING)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_SetInt2DataSign(void *handle, H3LIS331DL_INT_Conf_t i_cfg) {
  u8_t value;
  
  if( !H3LIS331DL_ACC_ReadReg ( handle, H3LIS331DL_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;
  
  value &= 0xE7;
  value |= (i_cfg<<H3LIS331DL_I2_CFG);
  
  if( !H3LIS331DL_ACC_WriteReg ( handle, H3LIS331DL_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : H3LIS331DL_SetSPI34Wire
* Description    : Set SPI mode 
* Input          : Modality by H3LIS331DL_SPIMode_t Typedef (H3LIS331DL_SPI_4_WIRE, H3LIS331DL_SPI_3_WIRE)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_SetSPI34Wire(void *handle, H3LIS331DL_SPIMode_t sim) {
  u8_t value;
  
  if( !H3LIS331DL_ACC_ReadReg ( handle, H3LIS331DL_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;
  
  value &= 0xFE;
  value |= (sim<<H3LIS331DL_SIM);
  
  if( !H3LIS331DL_ACC_WriteReg ( handle, H3LIS331DL_CTRL_REG4, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : H3LIS331DL_TurnONEnable
* Description    : TurnON Mode selection for sleep to wake function
* Input          : H3LIS331DL_SLEEP_TO_WAKE_DIS/H3LIS331DL_SLEEP_TO_WAKE_ENA
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_TurnONEnable(void *handle, H3LIS331DL_Sleep_To_Wake_Conf_t stw) {
  u8_t value;
  
  if( !H3LIS331DL_ACC_ReadReg ( handle, H3LIS331DL_CTRL_REG5, &value, 1) )
    return MEMS_ERROR;
  
  value &= 0x00;
  value |= (stw<<H3LIS331DL_TURN_ON);
  
  if( !H3LIS331DL_ACC_WriteReg ( handle, H3LIS331DL_CTRL_REG5, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : H3LIS331DL_HPFilterReset
* Description    : Reading register for reset the content of internal HP filter
* Input          : None
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_HPFilterReset(void *handle) {
  u8_t value;
  
  if( !H3LIS331DL_ACC_ReadReg ( handle, H3LIS331DL_HP_FILTER_RESET, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : H3LIS331DL_SetReference
* Description    : Sets Reference register acceleration value as a reference for HP filter
* Input          : Value of reference acceleration value (0-255)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_SetReference(void *handle, i8_t ref) {
  
  if( !H3LIS331DL_ACC_WriteReg ( handle, H3LIS331DL_REFERENCE_REG, (u8_t*) ref, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : H3LIS331DL_SetHPFMode
* Description    : Set High Pass Filter Modality
* Input          : H3LIS331DL_HPM_NORMAL_MODE_RES/H3LIS331DL_HPM_REF_SIGNAL/H3LIS331DL_HPM_NORMAL_MODE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_SetHPFMode(void *handle, H3LIS331DL_HPFMode_t hpm) {
  u8_t value;
  
  if( !H3LIS331DL_ACC_ReadReg ( handle, H3LIS331DL_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;
  
  value &= 0x9F;
  value |= (hpm<<H3LIS331DL_HPM);
  
  if( !H3LIS331DL_ACC_WriteReg ( handle, H3LIS331DL_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : H3LIS331DL_SetHPFCutOFF
* Description    : Set High Pass CUT OFF Freq
* Input          : H3LIS331DL_HPFCF [0,3]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_SetHPFCutOFF(void *handle, H3LIS331DL_HPFCutOffFreq_t hpf) {
  u8_t value;
  
  if( !H3LIS331DL_ACC_ReadReg ( handle, H3LIS331DL_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;
  
  value &= 0xFC;
  value |= (hpf<<H3LIS331DL_HPCF);
  
  if( !H3LIS331DL_ACC_WriteReg ( handle, H3LIS331DL_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
  
}


/*******************************************************************************
* Function Name  : H3LIS331DL_SetInt2HPEnable
* Description    : Set Interrupt2 hp filter enable/disable
* Input          : MEMS_ENABLE/MEMS_DISABLE
* example        : H3LIS331DL_SetInt2HPEnable(MEMS_ENABLE)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_SetInt2HPEnable(void *handle, State_t stat) {
  u8_t value;
  
  if( !H3LIS331DL_ACC_ReadReg ( handle, H3LIS331DL_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;
  
  value &= 0xF7;
  value |= stat<<H3LIS331DL_HPEN2 ;
  
  if( !H3LIS331DL_ACC_WriteReg ( handle, H3LIS331DL_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}     


/*******************************************************************************
* Function Name  : H3LIS331DL_SetInt1HPEnable
* Description    : Set Interrupt1 hp filter enable/disable
* Input          : MEMS_ENABLE/MEMS_DISABLE
* example        : H3LIS331DL_SetInt1HPEnable(MEMS_ENABLE)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_SetInt1HPEnable(void *handle, State_t stat) {
  u8_t value;
  
  if( !H3LIS331DL_ACC_ReadReg ( handle, H3LIS331DL_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;
  
  value &= 0xFB;
  value |= stat<<H3LIS331DL_HPEN1 ;
  
  if( !H3LIS331DL_ACC_WriteReg ( handle, H3LIS331DL_CTRL_REG2, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}  


/*******************************************************************************
* Function Name  : H3LIS331DL_Int1LatchEnable
* Description    : Enable Interrupt 1 Latching function
* Input          : ENABLE/DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_Int1LatchEnable(void *handle, State_t latch) {
  u8_t value;
  
  if( !H3LIS331DL_ACC_ReadReg ( handle, H3LIS331DL_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;
  
  value &= 0xFB;
  value |= latch<<H3LIS331DL_LIR1;
  
  if( !H3LIS331DL_ACC_WriteReg ( handle, H3LIS331DL_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : H3LIS331DL_Int2LatchEnable
* Description    : Enable Interrupt 2 Latching function
* Input          : ENABLE/DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_Int2LatchEnable(void *handle, State_t latch) {
  u8_t value;
  
  if( !H3LIS331DL_ACC_ReadReg ( handle, H3LIS331DL_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;
  
  value &= 0xDF;
  value |= latch<<H3LIS331DL_LIR2;
  
  if( !H3LIS331DL_ACC_WriteReg ( handle, H3LIS331DL_CTRL_REG3, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : H3LIS331DL_ResetInt1Latch
* Description    : Reset Interrupt 1 Latching function
* Input          : None
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_ResetInt1Latch(void *handle) {
  u8_t value;
  
  if( !H3LIS331DL_ACC_ReadReg ( handle, H3LIS331DL_INT1_SRC, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : H3LIS331DL_ResetInt2Latch
* Description    : Reset Interrupt 2 Latching function
* Input          : None
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_ResetInt2Latch(void *handle) {
  u8_t value;
  
  if( !H3LIS331DL_ACC_ReadReg ( handle, H3LIS331DL_INT2_SRC, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : H3LIS331DL_SetInt1Configuration
* Description    : Interrupt 1 Configuration (without 6D_INT)
* Input          : H3LIS331DL_INT_AND/OR | H3LIS331DL_INT_ZHIE_ENABLE/DISABLE | H3LIS331DL_INT_ZLIE_ENABLE/DISABLE...
* Output         : None
* Note           : You MUST use ALL input variable in the argument, as in example above
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_SetInt1Configuration(void *handle, H3LIS331DL_IntConf_t ic) {
  u8_t value;
  
  if( !H3LIS331DL_ACC_ReadReg ( handle, H3LIS331DL_INT1_CFG, &value, 1) )
    return MEMS_ERROR;
  
  value &= 0x40; 
  value |= ic;
  
  if( !H3LIS331DL_ACC_WriteReg ( handle, H3LIS331DL_INT1_CFG, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : H3LIS331DL_SetInt2Configuration
* Description    : Interrupt 2 Configuration (without 6D_INT)
* Input          : H3LIS331DL_INT_AND/OR | H3LIS331DL_INT_ZHIE_ENABLE/DISABLE | H3LIS331DL_INT_ZLIE_ENABLE/DISABLE...
* Output         : None
* Note           : You MUST use all input variable in the argument, as example
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_SetInt2Configuration(void *handle, H3LIS331DL_IntConf_t ic) {
  u8_t value;
  
  if( !H3LIS331DL_ACC_ReadReg ( handle, H3LIS331DL_INT2_CFG, &value, 1) )
    return MEMS_ERROR;
  
  value &= 0x40; 
  value |= ic;
  
  if( !H3LIS331DL_ACC_WriteReg ( handle, H3LIS331DL_INT2_CFG, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : H3LIS331DL_SetInt1Mode
* Description    : Interrupt 1 Configuration mode (OR, 6D Movement, AND, 6D Position)
* Input          : H3LIS331DL_INT_MODE_OR, H3LIS331DL_INT_MODE_6D_MOVEMENT, H3LIS331DL_INT_MODE_AND, H3LIS331DL_INT_MODE_6D_POSITION
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_SetInt1Mode(void *handle, H3LIS331DL_IntMode_t int_mode) {
  u8_t value;
  
  if( !H3LIS331DL_ACC_ReadReg ( handle, H3LIS331DL_INT1_CFG, &value, 1) )
    return MEMS_ERROR;
  
  value &= 0x3F; 
  value |= (int_mode<<H3LIS331DL_INT_6D);
  
  if( !H3LIS331DL_ACC_WriteReg ( handle, H3LIS331DL_INT1_CFG, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : H3LIS331DL_SetInt2Mode
* Description    : Interrupt 2 Configuration mode (OR, 6D Movement, AND, 6D Position)
* Input          : H3LIS331DL_INT_MODE_OR, H3LIS331DL_INT_MODE_6D_MOVEMENT, H3LIS331DL_INT_MODE_AND, H3LIS331DL_INT_MODE_6D_POSITION
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_SetInt2Mode(void *handle, H3LIS331DL_IntMode_t int_mode) {
  u8_t value;
  
  if( !H3LIS331DL_ACC_ReadReg ( handle, H3LIS331DL_INT2_CFG, &value, 1) )
    return MEMS_ERROR;
  
  value &= 0x3F; 
  value |= (int_mode<<H3LIS331DL_INT_6D);
  
  if( !H3LIS331DL_ACC_WriteReg ( handle, H3LIS331DL_INT2_CFG, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : H3LIS331DL_Get6DPositionInt1
* Description    : 6D Interrupt 1 Position Detect
* Input          : Byte to be filled with H3LIS331DL_POSITION_6D_t Typedef
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_Get6DPositionInt1(void *handle, u8_t* val){
  u8_t value;
  
  if( !H3LIS331DL_ACC_ReadReg ( handle, H3LIS331DL_INT1_SRC, &value, 1) )
    return MEMS_ERROR;
  
  value &= 0x7F;
  
  switch (value){
  case H3LIS331DL_UP_SX:   
    *val = H3LIS331DL_UP_SX;    
    break;
  case H3LIS331DL_UP_DX:   
    *val = H3LIS331DL_UP_DX;    
    break;
  case H3LIS331DL_DW_SX:   
    *val = H3LIS331DL_DW_SX;    
    break;
  case H3LIS331DL_DW_DX:   
    *val = H3LIS331DL_DW_DX;    
    break;
  case H3LIS331DL_TOP:     
    *val = H3LIS331DL_TOP;      
    break;
  case H3LIS331DL_BOTTOM:  
    *val = H3LIS331DL_BOTTOM;  
    break;
  }
  
  return MEMS_SUCCESS;  
}


/*******************************************************************************
* Function Name  : H3LIS331DL_Get6DPositionInt2
* Description    : 6D Interrupt 2 Position Detect
* Input          : Byte to be filled with H3LIS331DL_POSITION_6D_t Typedef
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_Get6DPositionInt2(void *handle, u8_t* val){
  u8_t value;
  
  if( !H3LIS331DL_ACC_ReadReg ( handle, H3LIS331DL_INT2_SRC, &value, 1) )
    return MEMS_ERROR;
  
  value &= 0x7F;
  
  switch (value){
  case H3LIS331DL_UP_SX:   
    *val = H3LIS331DL_UP_SX;    
    break;
  case H3LIS331DL_UP_DX:   
    *val = H3LIS331DL_UP_DX;    
    break;
  case H3LIS331DL_DW_SX:   
    *val = H3LIS331DL_DW_SX;    
    break;
  case H3LIS331DL_DW_DX:   
    *val = H3LIS331DL_DW_DX;    
    break;
  case H3LIS331DL_TOP:     
    *val = H3LIS331DL_TOP;      
    break;
  case H3LIS331DL_BOTTOM:  
    *val = H3LIS331DL_BOTTOM;   
    break;
  }
  
  return MEMS_SUCCESS;  
}


/*******************************************************************************
* Function Name  : H3LIS331DL_SetInt1Threshold
* Description    : Sets Interrupt 1 Threshold
* Input          : Threshold = [0,127]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_SetInt1Threshold(void *handle, u8_t ths) {
  if (ths > 127)
    return MEMS_ERROR;
  
  if( !H3LIS331DL_ACC_WriteReg ( handle, H3LIS331DL_INT1_THS, &ths, 1) )
    return MEMS_ERROR;    
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : H3LIS331DL_SetInt1Duration
* Description    : Sets Interrupt 1 Duration
* Input          : Duration = [0,127]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_SetInt1Duration(void *handle, u8_t id) {  
  if (id > 127)
    return MEMS_ERROR;
  
  if( !H3LIS331DL_ACC_WriteReg ( handle, H3LIS331DL_INT1_DURATION, &id, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : H3LIS331DL_SetInt2Threshold
* Description    : Sets Interrupt 2 Threshold
* Input          : Threshold = [0,127]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_SetInt2Threshold(void *handle, u8_t ths) {
  if (ths > 127)
    return MEMS_ERROR;
  
  if( !H3LIS331DL_ACC_WriteReg ( handle, H3LIS331DL_INT2_THS, &ths, 1) )
    return MEMS_ERROR;    
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : H3LIS331DL_SetInt2Duration
* Description    : Sets Interrupt 2 Duration
* Input          : Duration = [0,127]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_SetInt2Duration(void *handle, u8_t id) {  
  if (id > 127)
    return MEMS_ERROR;
  
  if( !H3LIS331DL_ACC_WriteReg ( handle, H3LIS331DL_INT2_DURATION, &id, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : H3LIS331DL_GetStatusReg
* Description    : Read the status register
* Input          : char to empty by Status Reg Value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_GetStatusReg(void *handle, u8_t* val) {
  
  if( !H3LIS331DL_ACC_ReadReg ( handle, H3LIS331DL_STATUS_REG, val, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;  
}

/*******************************************************************************
* Function Name  : H3LIS331DL_GetStatusBIT
* Description    : Read the status register BIT
* Input          : H3LIS331DL_STATUS_REG_ZYXOR, H3LIS331DL_STATUS_REG_ZOR, H3LIS331DL_STATUS_REG_YOR, H3LIS331DL_STATUS_REG_XOR,
                   H3LIS331DL_STATUS_REG_ZYXDA, H3LIS331DL_STATUS_REG_ZDA, H3LIS331DL_STATUS_REG_YDA, H3LIS331DL_STATUS_REG_XDA, 
                   H3LIS331DL_DATAREADY_BIT
* Output         : status register BIT
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_GetStatusBit(void *handle, u8_t statusBIT, u8_t *val) {
  u8_t value;  
  
  if( !H3LIS331DL_ACC_ReadReg ( handle, H3LIS331DL_STATUS_REG, &value, 1) )
    return MEMS_ERROR;
  
  switch (statusBIT){
  case H3LIS331DL_STATUS_REG_ZYXOR:     
    if(value &= H3LIS331DL_STATUS_REG_ZYXOR){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  case H3LIS331DL_STATUS_REG_ZOR:       
    if(value &= H3LIS331DL_STATUS_REG_ZOR){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  case H3LIS331DL_STATUS_REG_YOR:       
    if(value &= H3LIS331DL_STATUS_REG_YOR){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }                                 
  case H3LIS331DL_STATUS_REG_XOR:       
    if(value &= H3LIS331DL_STATUS_REG_XOR){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  case H3LIS331DL_STATUS_REG_ZYXDA:     
    if(value &= H3LIS331DL_STATUS_REG_ZYXDA){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }   
  case H3LIS331DL_STATUS_REG_ZDA:       
    if(value &= H3LIS331DL_STATUS_REG_ZDA){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }   
  case H3LIS331DL_STATUS_REG_YDA:       
    if(value &= H3LIS331DL_STATUS_REG_YDA){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }   
  case H3LIS331DL_STATUS_REG_XDA:       
    if(value &= H3LIS331DL_STATUS_REG_XDA){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }                                      
  }
  
  return MEMS_ERROR;
}

/*******************************************************************************
* Function Name  : H3LIS331DL_GetInt1Src
* Description    : Reset Interrupt 1 Latching function
* Input          : buffer to empty by Int1 Source Value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_GetInt1Src(void *handle, u8_t* val) {  
  if( !H3LIS331DL_ACC_ReadReg ( handle, H3LIS331DL_INT1_SRC, val, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : H3LIS331DL_GetInt2Src
* Description    : Reset Interrupt 2 Latching function
* Input          : buffer to empty by Int2 Source Value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_GetInt2Src(void *handle, u8_t* val) {  
  if( !H3LIS331DL_ACC_ReadReg ( handle, H3LIS331DL_INT2_SRC, val, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : H3LIS331DL_GetInt1SrcBit
* Description    : Reset Interrupt 1 Latching function
* Input          : H3LIS331DL_INT1_SRC_IA, H3LIS331DL_INT1_SRC_ZH, H3LIS331DL_INT1_SRC_ZL .....
* Output         : None
* Return         : Status of BIT [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_GetInt1SrcBit(void *handle, u8_t statusBIT, u8_t *val) {
  u8_t value;  
  
  if( !H3LIS331DL_ACC_ReadReg ( handle, H3LIS331DL_INT1_SRC, &value, 1) )
    return MEMS_ERROR;
  
  if(statusBIT == H3LIS331DL_INT_SRC_IA){
    if(value &= H3LIS331DL_INT_SRC_IA){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }    
  }
  
  if(statusBIT == H3LIS331DL_INT_SRC_ZH){
    if(value &= H3LIS331DL_INT_SRC_ZH){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }   
  }
  
  if(statusBIT == H3LIS331DL_INT_SRC_ZL){
    if(value &= H3LIS331DL_INT_SRC_ZL){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }   
  }
  
  if(statusBIT == H3LIS331DL_INT_SRC_YH){
    if(value &= H3LIS331DL_INT_SRC_YH){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }   
  }
  
  if(statusBIT == H3LIS331DL_INT_SRC_YL){
    if(value &= H3LIS331DL_INT_SRC_YL){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == H3LIS331DL_INT_SRC_XH){
    if(value &= H3LIS331DL_INT_SRC_XH){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }   
  }
  
  if(statusBIT == H3LIS331DL_INT_SRC_XL){
    if(value &= H3LIS331DL_INT_SRC_XL){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }    
  } 
  return MEMS_ERROR;
}


/*******************************************************************************
* Function Name  : H3LIS331DL_GetInt2SrcBit
* Description    : Reset Interrupt 2 Latching function
* Input          : H3LIS331DL_INT_SRC_IA, H3LIS331DL_INT_SRC_ZH, H3LIS331DL_INT_SRC_ZL .....
* Output         : None
* Return         : Status of BIT [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t H3LIS331DL_GetInt2SrcBit(void *handle, u8_t statusBIT, u8_t *val) {
  u8_t value;  
  
  if( !H3LIS331DL_ACC_ReadReg ( handle, H3LIS331DL_INT2_SRC, &value, 1) )
    return MEMS_ERROR;
  
  if(statusBIT == H3LIS331DL_INT_SRC_IA){
    if(value &= H3LIS331DL_INT_SRC_IA){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }   
  }
  
  if(statusBIT == H3LIS331DL_INT_SRC_ZH){
    if(value &= H3LIS331DL_INT_SRC_ZH){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == H3LIS331DL_INT_SRC_ZL){
    if(value &= H3LIS331DL_INT_SRC_ZL){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == H3LIS331DL_INT_SRC_YH){
    if(value &= H3LIS331DL_INT_SRC_YH){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }    
  }
  
  if(statusBIT == H3LIS331DL_INT_SRC_YL){
    if(value &= H3LIS331DL_INT_SRC_YL){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }   
  }
  
  if(statusBIT == H3LIS331DL_INT_SRC_XH){
    if(value &= H3LIS331DL_INT_SRC_XH){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == H3LIS331DL_INT_SRC_XL){
    if(value &= H3LIS331DL_INT_SRC_XL){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }    
  } 
  return MEMS_ERROR;
}
/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
