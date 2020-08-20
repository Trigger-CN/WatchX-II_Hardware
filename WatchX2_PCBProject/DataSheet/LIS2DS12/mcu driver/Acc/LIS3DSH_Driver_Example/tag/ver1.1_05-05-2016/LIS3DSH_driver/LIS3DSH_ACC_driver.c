/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
* File Name          : LIS3DSH_ACC_driver.c
* Author             : MEMS Application Team
* Version            : v1.1
* Date               : 05 Maggio 2016   
* Description        : LIS3DSH Platform Independent Driver
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
#include "LIS3DSH_ACC_driver.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/************** Generic Function  *******************/

/*******************************************************************************
* Function Name : LIS3DSH_ACC_ReadReg
* Description   : Generic Reading function. It must be fullfilled with either
*               : I2C or SPI reading functions
* Input         : Register Address
* Output        : Data REad
* Return			: None
*******************************************************************************/
status_t LIS3DSH_ACC_ReadReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len)
{
   
  //To be completed with either I2c or SPI writing function
  //i.e.: SPI_Mems_Write_Reg(Reg, Data);
  //I2Cx_Read(Bufp, LIS3DSH_ACC_I2C_ADDRESS_HIGH, Reg, len);    //[Example]
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name : LIS3DSH_ACC_WriteReg
* Description   : Generic Writing function. It must be fullfilled with either
*					: I2C or SPI writing function
* Input         : Register Address, Data to be written
* Output			: None
* Return			: None
*******************************************************************************/
status_t LIS3DSH_ACC_WriteReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len)
{
    
  //To be completed with either I2c or SPI writing function
  //i.e.: SPI_Mems_Write_Reg(Reg, Data);
  //I2Cx_Write(Bufp, LIS3DSH_ACC_I2C_ADDRESS_HIGH, Reg, len); //[Example]
  return MEMS_SUCCESS; 
}

/**************** Base Function  *******************/

/*******************************************************************************
* Function Name  : GetWHO_AM_I
* Description    : Read identification code by WHO_AM_I register
* Input          : Char to empty by Device identification Value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_GetWHO_AM_I(void *handle, u8_t* val){
  
  if( !LIS3DSH_ACC_ReadReg(handle, LIS3DSH_ACC_WHO_AM_I, val, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : GetOUT_T
* Description    : Read temperature register 1LSB/deg (00h = 25degC)
* Input          : Char to empty by temperature value (8bit 2's complement)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_GetOUT_T(void *handle, u8_t* val){
   if( !LIS3DSH_ACC_ReadReg(handle, LIS3DSH_ACC_OUT_T, val, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetODR
* Description    : Sets Output Data Rate
* Input          : Output Data Rate typedef
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_SetODR(void *handle, LIS3DSH_ACC_ODR_t ov){
  u8_t value;

  if( !LIS3DSH_ACC_ReadReg(handle, LIS3DSH_ACC_CNTL4, &value, 1) )
    return MEMS_ERROR;

  value &= 0x0f;
  value |= ov<<LIS3DSH_ACC_ODR_BIT;
    
    if( !LIS3DSH_ACC_WriteReg(handle, LIS3DSH_ACC_CNTL4, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetAxis
* Description    : Enable/Disable Acc. Axis
* Input          : X_ENABLE/X_DISABLE | Y_ENABLE/Y_DISABLE | Z_ENABLE/Z_DISABLE
* Output         : None
* Note           : You MUST use all input variable in the argument
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_SetAxis(void *handle, LIS3DSH_ACC_Axis_t axis) {
  u8_t value;
  
  if( !LIS3DSH_ACC_ReadReg(handle, LIS3DSH_ACC_CNTL4, &value, 1) )
    return MEMS_ERROR;
  value &= 0xF8;
  value |= (0x07 & axis);
   
  if( !LIS3DSH_ACC_WriteReg(handle, LIS3DSH_ACC_CNTL4, &value, 1) )
    return MEMS_ERROR;   
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetFullScale
* Description    : Set FullScale by typedef definition
* Input          : FULLSCALE_2/FULLSCALE_4/FULLSCALE_8/FULLSCALE_16
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_SetFullScale(void *handle, LIS3DSH_ACC_Fullscale_t fs) {
  u8_t value;
  
  if( !LIS3DSH_ACC_ReadReg(handle, LIS3DSH_ACC_CNTL5, &value, 1) )
    return MEMS_ERROR;
                  
  value &= 0xC7;	
  value |= (fs<<LIS3DSH_ACC_FSCALE);
  
  if( !LIS3DSH_ACC_WriteReg(handle, LIS3DSH_ACC_CNTL5, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetBDU
* Description    : Enable/Disable Block Data Update Functionality
* Input          : ENABLE/DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_SetBDU(void *handle, LIS3DSH_ACC_State_t bdu) {
  u8_t value;
  
  if( !LIS3DSH_ACC_ReadReg(handle, LIS3DSH_ACC_CNTL4, &value, 1) )
    return MEMS_ERROR;
 
  value &= 0xF7;
  value |= (bdu<<LIS3DSH_ACC_BDU);

  if( !LIS3DSH_ACC_WriteReg(handle, LIS3DSH_ACC_CNTL4, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetSelfTest
* Description    : Set Self Test Modality
* Input          : SELF_TEST_NORMAL/SELF_TEST_POSITIVE/SELF_TEST_NEGATIVE...
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_SetSelfTest(void *handle, LIS3DSH_ACC_SelfTest_t st) {
  u8_t value;
  
  if( !LIS3DSH_ACC_ReadReg(handle, LIS3DSH_ACC_CNTL5, &value, 1) )
    return MEMS_ERROR;
                  
  value &= 0xF9;
  value |= (st<<LIS3DSH_ACC_ST);
  
  if( !LIS3DSH_ACC_WriteReg(handle, LIS3DSH_ACC_CNTL5, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : BandWidth
* Description    : Set BandWidth filter by typedef definition
* Input          : BANDWIDTH_1/BANDWIDTH_2/BANDWIDTH_3...
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_BandWidth(void *handle, LIS3DSH_ACC_BandWidth_t bw) {
  u8_t value;
  
  bw &= 0x03; 
  if( !LIS3DSH_ACC_ReadReg(handle, LIS3DSH_ACC_CNTL5, &value, 1) )
    return MEMS_ERROR;
                  
  value &= 0x3F;
  value |= (bw<<LIS3DSH_ACC_BW);
  
  if( !LIS3DSH_ACC_WriteReg(handle, LIS3DSH_ACC_CNTL5, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : Int1Enable
* Description    : Set Interrupt1 Enable-Disable
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_Int1Enable(void *handle, LIS3DSH_ACC_State_t conf) {
  u8_t value;
  
  if( !LIS3DSH_ACC_ReadReg(handle, LIS3DSH_ACC_CNTL3, &value, 1) )
    return MEMS_ERROR;
                  
  value &= 0xF7;
  value |= (conf<<LIS3DSH_ACC_INT1_EN);
  
  if( !LIS3DSH_ACC_WriteReg(handle, LIS3DSH_ACC_CNTL3, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : Int2Enable
* Description    : Set Interrupt2 Enable-Disable
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_Int2Enable(void *handle, LIS3DSH_ACC_State_t conf) {
  u8_t value;
  
  if( !LIS3DSH_ACC_ReadReg(handle, LIS3DSH_ACC_CNTL3, &value, 1) )
    return MEMS_ERROR;
                  
  value &= 0xEF;
  value |= (conf<<LIS3DSH_ACC_INT2_EN);
  
  if( !LIS3DSH_ACC_WriteReg(handle, LIS3DSH_ACC_CNTL3, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

                 
/*******************************************************************************
* Function Name  : IntLatchEnable
* Description    : Enable Interrupt Latching function
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_IntLatchEnable(void *handle, LIS3DSH_ACC_State_t latch) {
  u8_t value;
  
  if( !LIS3DSH_ACC_ReadReg(handle, LIS3DSH_ACC_CNTL3, &value, 1) )
    return MEMS_ERROR;
                  
  value &= 0xDF;
  value |= (latch<<LIS3DSH_ACC_IEL);
  
  if( !LIS3DSH_ACC_WriteReg(handle, LIS3DSH_ACC_CNTL3, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : IntSignPol
* Description    : Interrupt Polarity
* Input          : POL_HIGH/POL_LOW
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_IntSignPol(void *handle, LIS3DSH_ACC_Polarity_t pol) {
  u8_t value;
  
  if( !LIS3DSH_ACC_ReadReg(handle, LIS3DSH_ACC_CNTL3, &value, 1) )
    return MEMS_ERROR;
                  
  value &= 0xDF;
  value |= (pol<<LIS3DSH_ACC_IEA);
  
  if( !LIS3DSH_ACC_WriteReg(handle, LIS3DSH_ACC_CNTL3, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : DataReadyInt
* Description    : Data ready connect to interrupt 1 Enable
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_DataReadyInt(void *handle, LIS3DSH_ACC_State_t drdy) {
  u8_t value;
  
  if( !LIS3DSH_ACC_ReadReg(handle, LIS3DSH_ACC_CNTL3, &value, 1) )
    return MEMS_ERROR;
                  
  value &= 0x7F;
  value |= (drdy<<LIS3DSH_ACC_DR_EN);
  
  if( !LIS3DSH_ACC_WriteReg(handle, LIS3DSH_ACC_CNTL3, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : ReBootEnable
* Description    : Force Reboot
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_ReBootEnable(void *handle, LIS3DSH_ACC_State_t boot) {
  u8_t value;
  
  if( !LIS3DSH_ACC_ReadReg(handle, LIS3DSH_ACC_CNTL6, &value, 1) )
    return MEMS_ERROR;
                  
  value &= 0x7F;
  value |= (boot<<LIS3DSH_ACC_BOOT);
  
  if( !LIS3DSH_ACC_WriteReg(handle, LIS3DSH_ACC_CNTL6, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : FIFOEnable
* Description    : FIFO enable/disable
* Input          : MEMS_ENABLE/MEMS_DISABLE, n max sample in FIFO (must be < 30)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_FIFOEnable(void *handle, LIS3DSH_ACC_State_t fifo, u8_t nMax) {
  u8_t value;
  
  //check n max of sample in FIFO
  if(nMax > 30) return MEMS_ERROR;
  
  //only stream mode fifo
  if(! LIS3DSH_ACC_FIFOMode(handle, LIS3DSH_ACC_FIFO_STREAM_MODE))
    return MEMS_ERROR;  
 
  //set WTM > n sample in FIFO 
  if(! LIS3DSH_ACC_SetWaterMark(handle, nMax))
    return MEMS_ERROR;
  
  if( !LIS3DSH_ACC_ReadReg(handle, LIS3DSH_ACC_CNTL6, &value, 1) )
    return MEMS_ERROR;
                  
  value &= 0xBF;
  value |= (fifo<<LIS3DSH_ACC_FIFO_EN);
  
  if( !LIS3DSH_ACC_WriteReg(handle, LIS3DSH_ACC_CNTL6, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : FIFOMode
* Description    : Sets FIFO Modality
* Input          : FIFO_BYPASS_MODE, FIFO_MODE...
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_FIFOMode(void *handle, LIS3DSH_ACC_FifoMode_t fm) {
  u8_t value;           
  
  if( !LIS3DSH_ACC_ReadReg(handle, LIS3DSH_ACC_FIFO_CTRL, &value, 1) )
      return MEMS_ERROR;
    
  value &= 0x1f;
  value |= (fm<<LIS3DSH_ACC_FMODE);                  
    
  if( !LIS3DSH_ACC_WriteReg(handle, LIS3DSH_ACC_FIFO_CTRL, &value, 1) )
      return MEMS_ERROR;
    
  return MEMS_SUCCESS;
}       


/*******************************************************************************
* Function Name  : SetWaterMark
* Description    : Sets Watermark Value
* Input          : Watermark = [0,31]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_SetWaterMark(void *handle, u8_t wtm) {
  u8_t value;
  
  if(wtm > 31)
    return MEMS_ERROR;  
  
  if( !LIS3DSH_ACC_ReadReg(handle, LIS3DSH_ACC_FIFO_CTRL, &value, 1) )
    return MEMS_ERROR;
                  
  value &= 0xE0;
  value |= wtm; 
  
  if( !LIS3DSH_ACC_WriteReg(handle, LIS3DSH_ACC_FIFO_CTRL, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : AddIncEnable
* Description    : Register address increment (during multiple byte access) enable/disable
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_AddIncEnable(void *handle, LIS3DSH_ACC_State_t addinc) {
  u8_t value;
  
  if( !LIS3DSH_ACC_ReadReg(handle, LIS3DSH_ACC_CNTL6, &value, 1) )
    return MEMS_ERROR;
                  
  value &= 0xEF;
  value |= (addinc<<LIS3DSH_ACC_ADD_INC);
  
  if( !LIS3DSH_ACC_WriteReg(handle, LIS3DSH_ACC_CNTL6, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : FifoEmptyInt1
* Description    : FIFO empty indication on INT1 enable/disable
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_FifoEmptyInt1(void *handle, LIS3DSH_ACC_State_t empty) {
  u8_t value;
  
  if( !LIS3DSH_ACC_ReadReg(handle, LIS3DSH_ACC_CNTL6, &value, 1) )
    return MEMS_ERROR;
                  
  value &= 0xF7;
  value |= (empty<<LIS3DSH_ACC_I1_EMPTY);
  
  if( !LIS3DSH_ACC_WriteReg(handle, LIS3DSH_ACC_CNTL6, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : FifoOvrInt1
* Description    : FIFO Overrun interrupt on INT1 enable/disable
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_FifoOvrInt1(void *handle, LIS3DSH_ACC_State_t overrun) {
  u8_t value;
  
  if( !LIS3DSH_ACC_ReadReg(handle, LIS3DSH_ACC_CNTL6, &value, 1) )
    return MEMS_ERROR;
                  
  value &= 0xFD;
  value |= (overrun<<LIS3DSH_ACC_I1_OVERRUN);
  
  if( !LIS3DSH_ACC_WriteReg(handle, LIS3DSH_ACC_CNTL6, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : BootInt2
* Description    : Boot Interrupt on INT2 enable/disable
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_BootInt2(void *handle, LIS3DSH_ACC_State_t booti2) {
  u8_t value;
  
  if( !LIS3DSH_ACC_ReadReg(handle, LIS3DSH_ACC_CNTL6, &value, 1) )
    return MEMS_ERROR;
                  
  value &= 0xFE;
  value |= (booti2<<LIS3DSH_ACC_I2_BOOT);
  
  if( !LIS3DSH_ACC_WriteReg(handle, LIS3DSH_ACC_CNTL6, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : VectFiltEnable
* Description    : Vector Filter Enable-Disable
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_VectFiltEnable(void *handle, LIS3DSH_ACC_State_t vfe) {
  u8_t value;
  
  if( !LIS3DSH_ACC_ReadReg(handle, LIS3DSH_ACC_CNTL3, &value, 1) )
    return MEMS_ERROR;
                  
  value &= 0x7F;
  value |= (vfe<<LIS3DSH_ACC_VFILT);
  
  if( !LIS3DSH_ACC_WriteReg(handle, LIS3DSH_ACC_CNTL3, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SoftReset
* Description    : Soft Reset BIT Enable-Disable
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_SoftReset(void *handle, LIS3DSH_ACC_State_t strt) {
  u8_t value;
  
  if( !LIS3DSH_ACC_ReadReg(handle, LIS3DSH_ACC_CNTL3, &value, 1) )
    return MEMS_ERROR;
                  
  value &= 0xFE;
  value |= (strt<<LIS3DSH_ACC_STRT);
  
  if( !LIS3DSH_ACC_WriteReg(handle, LIS3DSH_ACC_CNTL3, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}    


/*******************************************************************************
* Function Name  : SetOFFSET
* Description    : Set offset Value
* Input          : AXIS: SET_AXIS_X/SET_AXSIS_Y/SET_AXIS_Z, Offest value = [0,255]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_SetOFFSET(void *handle, LIS3DSH_ACC_SET_AXIS_t axis, u8_t val) {
  u8_t reg=0;
  
  if(!((axis==LIS3DSH_ACC_SET_AXIS_X)||(axis==LIS3DSH_ACC_SET_AXIS_Y)||(axis==LIS3DSH_ACC_SET_AXIS_Z)))
    return MEMS_ERROR;
  
  switch(axis){
  	case LIS3DSH_ACC_SET_AXIS_X: reg=LIS3DSH_ACC_OFF_X; break;
  	case LIS3DSH_ACC_SET_AXIS_Y: reg=LIS3DSH_ACC_OFF_Y; break;
  	case LIS3DSH_ACC_SET_AXIS_Z: reg=LIS3DSH_ACC_OFF_Z; break;
  }  
  
  if( !LIS3DSH_ACC_WriteReg(handle, reg, &val, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetCS
* Description    : Set Constant Shift Value
* Input          : AXIS: SET_AXIS_X/SET_AXSIS_Y/SET_AXIS_Z, Constant shift value = [0,255]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_SetCS(void *handle, LIS3DSH_ACC_SET_AXIS_t axis, u8_t val) {
  u8_t reg=0;
  
  if(!((axis==LIS3DSH_ACC_SET_AXIS_X)||(axis==LIS3DSH_ACC_SET_AXIS_Y)||(axis==LIS3DSH_ACC_SET_AXIS_Z)))
    return MEMS_ERROR;
  
  switch(axis){
  	case LIS3DSH_ACC_SET_AXIS_X: reg=LIS3DSH_ACC_CS_X; break;
  	case LIS3DSH_ACC_SET_AXIS_Y: reg=LIS3DSH_ACC_CS_Y; break;
  	case LIS3DSH_ACC_SET_AXIS_Z: reg=LIS3DSH_ACC_CS_Z; break;
  }  
  
  if( !LIS3DSH_ACC_WriteReg(handle, reg, &val, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetLC
* Description    : Set Long Counter Register Value
* Input          : Long Counter 16Bit value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_SetLC(void *handle, u16_t val) {
  u8_t val_L=0;
  u8_t val_H=0;
  
  val_L = (u8_t) (val & 0x00FF);
  val_H = (u8_t) ((val & 0xFF00)>>8);
  
  if( !LIS3DSH_ACC_WriteReg(handle, LIS3DSH_ACC_LC_L, &val_L, 1) )
    return MEMS_ERROR;
    
  if( !LIS3DSH_ACC_WriteReg(handle, LIS3DSH_ACC_LC_H, &val_H, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : GetLC
* Description    : Get Long Counter Register Value
* Input          : 16Bit Variable to empty by Counter value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_GetLC(void *handle, i16_t* val) {
  u8_t val_L=0;
  u8_t val_H=0;
 
  if( !LIS3DSH_ACC_ReadReg(handle, LIS3DSH_ACC_LC_L, &val_L, 1) )
    return MEMS_ERROR;
    
  if( !LIS3DSH_ACC_ReadReg(handle, LIS3DSH_ACC_LC_H, &val_H, 1) )
    return MEMS_ERROR;
 
  *val = (i16_t)((val_H<<8) + val_L);
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetVectorCoeff
* Description    : Set Vector Coefficient Value for Differential filter
* Input          : SET_VFC_1/SET_VFC_2/SET_VFC_3, Coefficient value = [0,255]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_SetVectorCoeff(void *handle, LIS3DSH_ACC_SET_VFC_t vfc, u8_t val) {
  u8_t reg=0;
  
  if(!((vfc==LIS3DSH_ACC_SET_VFC_1)||(vfc==LIS3DSH_ACC_SET_VFC_2)||(vfc==LIS3DSH_ACC_SET_VFC_3)||(vfc==LIS3DSH_ACC_SET_VFC_4)))
    return MEMS_ERROR;
  
  switch(vfc){
  	case LIS3DSH_ACC_SET_VFC_1: reg=LIS3DSH_ACC_VFC_1; break;
  	case LIS3DSH_ACC_SET_VFC_2: reg=LIS3DSH_ACC_VFC_2; break;
 	case LIS3DSH_ACC_SET_VFC_3: reg=LIS3DSH_ACC_VFC_3; break;
  	case LIS3DSH_ACC_SET_VFC_4: reg=LIS3DSH_ACC_VFC_4; break;
  }  
  
  if( !LIS3DSH_ACC_WriteReg(handle, reg, &val, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : GetVectorCoeff
* Description    : Get Vector Coefficient Value for Differential filter
* Input          : SET_VFC_1/SET_VFC_2/SET_VFC_3, variable to empty
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_GetVectorCoeff(void *handle, LIS3DSH_ACC_SET_VFC_t vfc, u8_t* val) {
  u8_t reg;
  
  if(!((vfc==LIS3DSH_ACC_SET_VFC_1)||(vfc==LIS3DSH_ACC_SET_VFC_2)||(vfc==LIS3DSH_ACC_SET_VFC_3)||(vfc==LIS3DSH_ACC_SET_VFC_4)))
    return MEMS_ERROR;
  
  switch(vfc){
  case LIS3DSH_ACC_SET_VFC_1: reg = LIS3DSH_ACC_VFC_1; break;
  case LIS3DSH_ACC_SET_VFC_2: reg = LIS3DSH_ACC_VFC_2; break;
  case LIS3DSH_ACC_SET_VFC_3: reg = LIS3DSH_ACC_VFC_3; break;
  case LIS3DSH_ACC_SET_VFC_4: reg = LIS3DSH_ACC_VFC_4; break;
  }  
  
  if( !LIS3DSH_ACC_ReadReg(handle, reg, val, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : SetThrs3
* Description    : set Threshold3 Coefficient Value
* Input          : Value of threshold [0,255]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_SetThrs3(void *handle, u8_t val) {
  
  if( !LIS3DSH_ACC_WriteReg(handle, LIS3DSH_ACC_THRS3, &val, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : GetThrs3
* Description    : Get Threshold3 Coefficient Value
* Input          : Variable to empty
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_GetThrs3(void *handle, u8_t* val) {
  
  if( !LIS3DSH_ACC_ReadReg(handle, LIS3DSH_ACC_THRS3, val, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : GetThrsSM
* Description    : Get Threshold 1 or 2 by SM1 or SM2
* Input          : SM1/SM2, THRS1/THRS2, Variable to empty
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_GetThrsSM(void *handle, LIS3DSH_ACC_SM_t sm, LIS3DSH_ACC_THRS_t thrs, u8_t* val) {
  u8_t reg=0;
  
  switch(thrs){
  case LIS3DSH_ACC_THRS_1:  if(sm==LIS3DSH_ACC_SM1) 	reg = LIS3DSH_ACC_THRS1_1; 
  		else 		reg = LIS3DSH_ACC_THRS1_2; 
		break;
  case LIS3DSH_ACC_THRS_2:  if(sm==LIS3DSH_ACC_SM1) 	reg = LIS3DSH_ACC_THRS2_1; 
  		else 		reg = LIS3DSH_ACC_THRS2_2; 
		break;
  }  
  
  if( !LIS3DSH_ACC_ReadReg(handle, reg, val, 1) )
    return MEMS_ERROR;
   
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetThrsSM
* Description    : Set Threshold 1 or 2 for SM1 or SM2
* Input          : SM1/SM2, THRS1/THRS2, Threshold Value [0,255]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_SetThrsSM(void *handle, LIS3DSH_ACC_SM_t sm, LIS3DSH_ACC_THRS_t thrs, u8_t val) {
  u8_t reg=0;
  
  switch(thrs){
  case LIS3DSH_ACC_THRS_1:  if(sm==LIS3DSH_ACC_SM1) reg = LIS3DSH_ACC_THRS1_1; 
  		else reg = LIS3DSH_ACC_THRS1_2; 
		break;
  case LIS3DSH_ACC_THRS_2:  if(sm==LIS3DSH_ACC_SM1) reg = LIS3DSH_ACC_THRS2_1; 
  		else reg = LIS3DSH_ACC_THRS2_2; 
		break;
  }  
  
  if( !LIS3DSH_ACC_WriteReg(handle, reg, &val, 1) )
    return MEMS_ERROR;
   
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetTimerSM
* Description    : Set Timer 1(16bit),2(16bit),3(8bit),4(8bit) for SM1 or SM2
* Input          : SM1/SM2, TIM1/TIM2..., Timer Value (8bit or 16bit)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_SetTimerSM(void *handle, LIS3DSH_ACC_SM_t sm, LIS3DSH_ACC_TIM_t timer, u16_t val) {
  u8_t reg=0;
  u8_t val_L=0;
  u8_t val_H=0;
  
  switch(timer){
  case LIS3DSH_ACC_TIM_1:  if(sm==LIS3DSH_ACC_SM1) 	reg = LIS3DSH_ACC_TIM1_1_L; 
  		else 		reg = LIS3DSH_ACC_TIM1_2_L; 
		break;
  case LIS3DSH_ACC_TIM_2:  if(sm==LIS3DSH_ACC_SM1) 	reg = LIS3DSH_ACC_TIM2_1_L; 
  		else 		reg = LIS3DSH_ACC_TIM2_2_L; 
		break;
  case LIS3DSH_ACC_TIM_3:  if(sm==LIS3DSH_ACC_SM1) 	reg = LIS3DSH_ACC_TIM3_1; 
  		else 		reg = LIS3DSH_ACC_TIM3_2; 
		break;
  case LIS3DSH_ACC_TIM_4:  if(sm==LIS3DSH_ACC_SM1) 	reg = LIS3DSH_ACC_TIM4_1; 
  		else 		reg = LIS3DSH_ACC_TIM4_2; 
		break;		
  } 
  //for 8bit register
  if((timer==LIS3DSH_ACC_TIM_3)||(timer==LIS3DSH_ACC_TIM_4)){
    val_L = (u8_t) val;
    if( !LIS3DSH_ACC_WriteReg(handle, reg, &val_L, 1) )
    	return MEMS_ERROR;
  }
  //for 16bit register
  else{
  val_L = (u8_t)  val;
  val_H = (u8_t) (val>>8);  
    
  if( !LIS3DSH_ACC_WriteReg(handle, reg, &val_L, 1) )
    return MEMS_ERROR;
  if( !LIS3DSH_ACC_WriteReg(handle, reg+1, &val_H, 1) )
    return MEMS_ERROR;   
  }
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetMaskSM
* Description    : Set Mask A or B for SM1 or SM2
* Input          : SM1/SM2, MASK_A/MASK_B, Mask Value [0,255]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_SetMaskSM(void *handle, LIS3DSH_ACC_SM_t sm, LIS3DSH_ACC_MASK_t mask, u8_t val) {
  u8_t reg=0;
  
  switch(mask){
  case LIS3DSH_ACC_MASK_A:  if(sm==LIS3DSH_ACC_SM1) 	reg = LIS3DSH_ACC_MASKA_1; 
  		else 		reg = LIS3DSH_ACC_MASKA_2; 
		break;
  case LIS3DSH_ACC_MASK_B:  if(sm==LIS3DSH_ACC_SM1) 	reg = LIS3DSH_ACC_MASKB_1; 
  		else 		reg = LIS3DSH_ACC_MASKB_2; 
		break;
  }  
  
  if( !LIS3DSH_ACC_WriteReg(handle, reg, &val, 1) )
    return MEMS_ERROR;
   
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : GetProgPointSM
* Description    : Get Program pointer for SM1 or SM2
* Input          : Byte to empty by Program pointer value (4bit)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_GetProgPointSM(void *handle, LIS3DSH_ACC_SM_t sm, u8_t* val) {
  u8_t reg=0; 
  
  switch(sm){
  case LIS3DSH_ACC_SM1 : reg = LIS3DSH_ACC_PR1; break;
  case LIS3DSH_ACC_SM2 : reg = LIS3DSH_ACC_PR2; break;
  }
  
  if( !LIS3DSH_ACC_ReadReg(handle, reg, val, 1) )
    return MEMS_ERROR;
 
  *val = (*val & 0xF0) >> 4;
  
  return MEMS_SUCCESS;
} 


/*******************************************************************************
* Function Name  : GetResetPointSM
* Description    : Get Reset pointer for SM1 or SM2
* Input          : Byte to empty by Reset pointer value (4bit)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_GetResetPointSM(void *handle, LIS3DSH_ACC_SM_t sm, u8_t* val) {
  u8_t reg=0; 
  
  switch(sm){
  case LIS3DSH_ACC_SM1 : reg = LIS3DSH_ACC_PR1; break;
  case LIS3DSH_ACC_SM2 : reg = LIS3DSH_ACC_PR2; break;
  }
  
  if( !LIS3DSH_ACC_ReadReg(handle, reg, val, 1) )
    return MEMS_ERROR;
 
  *val = (*val & 0x0F);
  
  return MEMS_SUCCESS;
} 


/*******************************************************************************
* Function Name  : GetTCSM
* Description    : Get 16bit general Timer Value for SM1 or SM2
* Input          : SM1/SM2, 16bit Variable to empty by timer value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_GetTCSM(void *handle, LIS3DSH_ACC_SM_t sm, u16_t* val) {
  u8_t val_L=0;
  u8_t val_H=0;
  u8_t reg=0;
  
  switch(sm){
  case LIS3DSH_ACC_SM1: reg = LIS3DSH_ACC_TC1_L;  break;
  case LIS3DSH_ACC_SM2: reg = LIS3DSH_ACC_TC2_L;  break;  
  }
  
  if( !LIS3DSH_ACC_ReadReg(handle, reg, &val_L, 1) )
    return MEMS_ERROR;
    
  if( !LIS3DSH_ACC_ReadReg(handle, reg+1, &val_H, 1) )
    return MEMS_ERROR;
 
  *val = (u16_t)((val_H<<8) + val_L);
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : GetOutSBitSM
* Description    : Read the output flags for interrupt by SM1 or SM2
* Input          : Out interrupt Bit to read (P_X/P_Y/N_Z/N_V....)
* Output         : None
* Return         : Status of BIT [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_GetOutSBitSM(void *handle, LIS3DSH_ACC_SM_t sm, u8_t FLAG_INT_OUT) {
  u8_t value; 
  u8_t reg;
 
  switch(sm){
  case LIS3DSH_ACC_SM1: reg = LIS3DSH_ACC_OUTS1;  break;
  case LIS3DSH_ACC_SM2: reg = LIS3DSH_ACC_OUTS2;  break;  
  }
  
  if( !LIS3DSH_ACC_ReadReg(handle, reg, &value, 1) )
      return MEMS_ERROR;
 
  switch (FLAG_INT_OUT){
  case LIS3DSH_ACC_F_P_X : if(value & LIS3DSH_ACC_F_P_X) return MEMS_SUCCESS;
             else  		return MEMS_ERROR;
  case LIS3DSH_ACC_F_N_X : if(value & LIS3DSH_ACC_F_N_X) return MEMS_SUCCESS;
             else  		return MEMS_ERROR;
  case LIS3DSH_ACC_F_P_Y : if(value & LIS3DSH_ACC_F_P_Y) return MEMS_SUCCESS;
             else  		return MEMS_ERROR;
  case LIS3DSH_ACC_F_N_Y : if(value & LIS3DSH_ACC_F_N_Y) return MEMS_SUCCESS;
             else  		return MEMS_ERROR;
  case LIS3DSH_ACC_F_P_Z : if(value & LIS3DSH_ACC_F_P_Z) return MEMS_SUCCESS;
             else  		return MEMS_ERROR;
  case LIS3DSH_ACC_F_N_Z : if(value & LIS3DSH_ACC_F_N_Z) return MEMS_SUCCESS;
             else  		return MEMS_ERROR;
  case LIS3DSH_ACC_F_P_V : if(value & LIS3DSH_ACC_F_P_V) return MEMS_SUCCESS;
             else  		return MEMS_ERROR;
  case LIS3DSH_ACC_F_N_V : if(value & LIS3DSH_ACC_F_N_V) return MEMS_SUCCESS;
             else  		return MEMS_ERROR;     
  }
  
return MEMS_ERROR;
}
  

/*******************************************************************************
* Function Name  : GetPeakSM
* Description    : Read the Peak detection Register value by SM1 or SM2
* Input          : SM1/SM2, Variable (8bit) to empty by Peak Register Value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_GetPeakSM(void *handle, LIS3DSH_ACC_SM_t sm, u8_t* val) {
  u8_t reg;
  
  switch(sm){
  case LIS3DSH_ACC_SM1: reg = LIS3DSH_ACC_PEAK1; break;
  case LIS3DSH_ACC_SM2: reg = LIS3DSH_ACC_PEAK2; break;  
  }
  
  if( !LIS3DSH_ACC_ReadReg(handle, reg, val, 1) )
      return MEMS_ERROR;
  
  return MEMS_SUCCESS;  
}


/*******************************************************************************
* Function Name  : GetDecimSM2
* Description    : Read the Decimator counter Register value by SM2
* Input          : Variable (8bit) to empty by Decimator counter Value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_GetDecimSM2(void *handle, u8_t* val) {
  
  if( !LIS3DSH_ACC_ReadReg(handle, LIS3DSH_ACC_DES2, val, 1) )
      return MEMS_ERROR;
  
  return MEMS_SUCCESS;  
}


/*******************************************************************************
* Function Name  : SetIntPinSM
* Description    : Set SMx Interrupt PIN routed to INT1 or INT2
* Input          : SMx, MEMS_DISABLE/ENABLE  (MEMS_DISABLE = routed INT1)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_SetIntPinSM(void *handle, LIS3DSH_ACC_SM_t sm, LIS3DSH_ACC_State_t state) {
  u8_t reg=0;
  u8_t value=0;
  
  switch(sm){
  case LIS3DSH_ACC_SM1:  reg = LIS3DSH_ACC_CNTL1; break;
  case LIS3DSH_ACC_SM2:  reg = LIS3DSH_ACC_CNTL2; break;
  }  
  
  if( !LIS3DSH_ACC_ReadReg(handle, reg, &value, 1) )
    return MEMS_ERROR;
 
  value &= 0xF7;
  value |= (state<<LIS3DSH_ACC_SM_PIN);   
  
  if( !LIS3DSH_ACC_WriteReg(handle, reg, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetIntEnaSM
* Description    : Set SMx Interrupt Enable for SM1 or SM2
* Input          : SMx, MEMS_DISABLE/ENABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_SetIntEnaSM(void *handle, LIS3DSH_ACC_SM_t sm, LIS3DSH_ACC_State_t state) {
  u8_t reg=0;
  u8_t value=0;
  
  switch(sm){
  case LIS3DSH_ACC_SM1:  reg = LIS3DSH_ACC_CNTL1; break;
  case LIS3DSH_ACC_SM2:  reg = LIS3DSH_ACC_CNTL2; break;
  }  
  
  if( !LIS3DSH_ACC_ReadReg(handle, reg, &value, 1) )
    return MEMS_ERROR;  
  
  value &= 0xFE;
  value |= (state<<LIS3DSH_ACC_SM_EN);  
  
  if( !LIS3DSH_ACC_WriteReg(handle, reg, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetPeakDetSM
* Description    : Set SMx Peak Detection Enable
* Input          : SMx, MEMS_DISABLE/ENABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_SetPeakDetSM(void *handle, LIS3DSH_ACC_SM_t sm, LIS3DSH_ACC_State_t state) {
  u8_t reg=0;    
  u8_t value;
  
  switch(sm){
  case LIS3DSH_ACC_SM1:  reg = LIS3DSH_ACC_SETT1; break;
  case LIS3DSH_ACC_SM2:  reg = LIS3DSH_ACC_SETT2; break;
  }  
 
  if( !LIS3DSH_ACC_ReadReg(handle, reg, &value, 1) )
    return MEMS_ERROR;
                  
  value &= 0x7F;
  value |= (state<<LIS3DSH_ACC_P_DET);
  
  if( !LIS3DSH_ACC_WriteReg(handle, reg, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetThr3SaSM
* Description    : Set SMx threshold3 limit value for axis and sign mask reset (MASKB_x)
* Input          : SMx, MEMS_DISABLE/ENABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_SetThr3SaSM(void *handle, LIS3DSH_ACC_SM_t sm, LIS3DSH_ACC_State_t state) {
  u8_t reg=0;    
  u8_t value;
  
  switch(sm){
  case LIS3DSH_ACC_SM1:  reg = LIS3DSH_ACC_SETT1; break;
  case LIS3DSH_ACC_SM2:  reg = LIS3DSH_ACC_SETT2; break;
  }  
 
  if( !LIS3DSH_ACC_ReadReg(handle, reg, &value, 1) )
    return MEMS_ERROR;
                  
  value &= 0xBF;
  value |= (state<<LIS3DSH_ACC_THR3_SA);
  
  if( !LIS3DSH_ACC_WriteReg(handle, reg, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetThr3MaSM
* Description    : Set SMx threshold3 limit value for axis and sign mask reset (MASKA_x)
* Input          : SMx, MEMS_DISABLE/ENABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_SetThr3MaSM(void *handle, LIS3DSH_ACC_SM_t sm, LIS3DSH_ACC_State_t state) {
  u8_t reg=0;    
  u8_t value;
  
  switch(sm){
  case LIS3DSH_ACC_SM1:  reg = LIS3DSH_ACC_SETT1; break;
  case LIS3DSH_ACC_SM2:  reg = LIS3DSH_ACC_SETT2; break;
  }  
 
  if( !LIS3DSH_ACC_ReadReg(handle, reg, &value, 1) )
    return MEMS_ERROR;
                  
  value &= 0xFB;
  value |= (state<<LIS3DSH_ACC_THR3_MA);
  
  if( !LIS3DSH_ACC_WriteReg(handle, reg, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetAbsSM
* Description    : Set SMx absolute value enable
* Input          : SMx, MEMS_DISABLE/ENABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_SetAbsSM(void *handle, LIS3DSH_ACC_SM_t sm, LIS3DSH_ACC_State_t state) {
  u8_t reg=0;    
  u8_t value;
  
  switch(sm){
  case LIS3DSH_ACC_SM1:  reg = LIS3DSH_ACC_SETT1; break;
  case LIS3DSH_ACC_SM2:  reg = LIS3DSH_ACC_SETT2; break;
  }  
 
  if( !LIS3DSH_ACC_ReadReg(handle, reg, &value, 1) )
    return MEMS_ERROR;
                  
  value &= 0xDF;
  value |= (state<<LIS3DSH_ACC_ABS);
  
  if( !LIS3DSH_ACC_WriteReg(handle, reg, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetRTamSM
* Description    : Set SMx next condition validation flag
* Input          : SMx, MEMS_DISABLE/ENABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_SetRTamSM(void *handle, LIS3DSH_ACC_SM_t sm, LIS3DSH_ACC_State_t state) {
  u8_t reg=0;    
  u8_t value;
  
  switch(sm){
  case LIS3DSH_ACC_SM1:  reg = LIS3DSH_ACC_SETT1; break;
  case LIS3DSH_ACC_SM2:  reg = LIS3DSH_ACC_SETT2; break;
  }  
 
  if( !LIS3DSH_ACC_ReadReg(handle, reg, &value, 1) )
    return MEMS_ERROR;
                  
  value &= 0xFD;
  value |= (state<<LIS3DSH_ACC_R_TAM);
  
  if( !LIS3DSH_ACC_WriteReg(handle, reg, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetSitrSM
* Description    : Set SMx program flow can be modified by STOP and COUNT
* Input          : SMx, MEMS_DISABLE/ENABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_SetSitrSM(void *handle, LIS3DSH_ACC_SM_t sm, LIS3DSH_ACC_State_t state) {
  u8_t reg=0;    
  u8_t value;
  
  switch(sm){
  case LIS3DSH_ACC_SM1:  reg = LIS3DSH_ACC_SETT1; break;
  case LIS3DSH_ACC_SM2:  reg = LIS3DSH_ACC_SETT2; break;
  }  
 
  if( !LIS3DSH_ACC_ReadReg(handle, reg, &value, 1) )
    return MEMS_ERROR;
                  
  value &= 0xFE;
  value |= (state<<LIS3DSH_ACC_SITR);
  
  if( !LIS3DSH_ACC_WriteReg(handle, reg, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;

}


/*******************************************************************************
* Function Name  : SetHystSM 
* Description    : Set Hysteresis for SM1 or SM2
* Input          : SM1/SM2, Hysteresis Value [0,7] (3bit)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_SetHystSM(void *handle, LIS3DSH_ACC_SM_t  sm, u8_t val) {
  u8_t reg=0;
  u8_t read=0;
  
  switch(sm){
  case LIS3DSH_ACC_SM1:  reg = LIS3DSH_ACC_CNTL1; break;
  case LIS3DSH_ACC_SM2:  reg = LIS3DSH_ACC_CNTL2; break;
  }  
  
  if( !LIS3DSH_ACC_ReadReg(handle, reg, &read, 1) )
    return MEMS_ERROR;
  
  read &= 0x1F;
  read |= (val<<LIS3DSH_ACC_HYST);
  
  if( !LIS3DSH_ACC_WriteReg(handle, reg, &read, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : GetSatusReg
* Description    : Read the status register
* Input          : char to empty by Status Reg Value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_GetSatusReg(void *handle, u8_t* val) {
 
  if( !LIS3DSH_ACC_ReadReg(handle, LIS3DSH_ACC_STATUS, val, 1) )
      return MEMS_ERROR;
  
  return MEMS_SUCCESS;  
}

      
/*******************************************************************************
* Function Name  : GetSatusBIT
* Description    : Read the status register BIT
* Input          : STATUS_REG_ZYXOR, STATUS_REG_ZOR, STATUS_REG_YOR, STATUS_REG_XOR,
                   STATUS_REG_ZYXDA, STATUS_REG_ZDA, STATUS_REG_YDA, STATUS_REG_XDA, DATAREADY_BIT
* Output         : status register BIT
* Return         : Status of BIT [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_GetSatusBit(void *handle, u8_t statusBIT) {
  u8_t value;  
  
  if( !LIS3DSH_ACC_ReadReg(handle, LIS3DSH_ACC_STATUS, &value, 1) )
      return MEMS_ERROR;
 
  switch (statusBIT){
    case LIS3DSH_ACC_STATUS_REG_ZYXOR:     if(value & LIS3DSH_ACC_STATUS_REG_ZYXOR) return MEMS_SUCCESS;
                               else  return MEMS_ERROR; 
    case LIS3DSH_ACC_STATUS_REG_ZOR:       if(value & LIS3DSH_ACC_STATUS_REG_ZOR) return MEMS_SUCCESS;
                               else  return MEMS_ERROR;
    case LIS3DSH_ACC_STATUS_REG_YOR:       if(value & LIS3DSH_ACC_STATUS_REG_YOR) return MEMS_SUCCESS;
                               else  return MEMS_ERROR;                               
    case LIS3DSH_ACC_STATUS_REG_XOR:       if(value & LIS3DSH_ACC_STATUS_REG_XOR) return MEMS_SUCCESS;
                               else  return MEMS_ERROR;   
    case LIS3DSH_ACC_STATUS_REG_ZYXDA:     if(value & LIS3DSH_ACC_STATUS_REG_ZYXDA) return MEMS_SUCCESS;
                               else  return MEMS_ERROR; 
    case LIS3DSH_ACC_STATUS_REG_ZDA:       if(value & LIS3DSH_ACC_STATUS_REG_ZDA) return MEMS_SUCCESS;
                               else  return MEMS_ERROR; 
    case LIS3DSH_ACC_STATUS_REG_YDA:       if(value & LIS3DSH_ACC_STATUS_REG_YDA) return MEMS_SUCCESS;
                               else  return MEMS_ERROR; 
    case LIS3DSH_ACC_STATUS_REG_XDA:       if(value & LIS3DSH_ACC_STATUS_REG_XDA) return MEMS_SUCCESS;
                               else  return MEMS_ERROR;                                
    
  }
return MEMS_ERROR;
}

   
/*******************************************************************************
* Function Name  : GetAccAxesRaw
* Description    : Read the Acceleration Values Output Registers
* Input          : buffer to empty by AccAxesRaw_t Typedef
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_GetAccAxesRaw(void *handle, Type3Axis16bit_U* buff) {
  u8_t valueL;
  u8_t valueH;
  
  if( !LIS3DSH_ACC_ReadReg(handle, LIS3DSH_ACC_OUT_X_L, &valueL, 1) )
      return MEMS_ERROR;
  
  if( !LIS3DSH_ACC_ReadReg(handle, LIS3DSH_ACC_OUT_X_H, &valueH, 1) )
      return MEMS_ERROR;
  
  buff->i16bit[0] = (i16_t)( (valueH << 8) | valueL )/16;
  
  if( !LIS3DSH_ACC_ReadReg(handle, LIS3DSH_ACC_OUT_Y_L, &valueL, 1) )
      return MEMS_ERROR;
  
  if( !LIS3DSH_ACC_ReadReg(handle, LIS3DSH_ACC_OUT_Y_H, &valueH, 1) )
      return MEMS_ERROR;
  
  buff->i16bit[1] = (i16_t)( (valueH << 8) | valueL )/16;
  
   if( !LIS3DSH_ACC_ReadReg(handle, LIS3DSH_ACC_OUT_Z_L, &valueL, 1) )
      return MEMS_ERROR;
  
  if( !LIS3DSH_ACC_ReadReg(handle, LIS3DSH_ACC_OUT_Z_H, &valueH, 1) )
      return MEMS_ERROR;
  
  buff->i16bit[2] = (i16_t)( (valueH << 8) | valueL )/16;
  
  return MEMS_SUCCESS;  
}


/*******************************************************************************
* Function Name  : GetStatBIT
* Description    : Read single BIT status of STAT register
* Input          : Stat BIT Mask Flag (F_LONG,F_SYNC1...)
* Output         : None
* Return         : Status of BIT [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_GetStatBIT(void *handle, u8_t StatBITMask) {
  u8_t value;
  
  if( !LIS3DSH_ACC_ReadReg(handle, LIS3DSH_ACC_STAT, &value, 1) )
    return MEMS_ERROR;
 
  if(value & StatBITMask)    return MEMS_SUCCESS;
    return MEMS_ERROR; 
}  


/*******************************************************************************
* Function Name  : GetFifoSourceReg
* Description    : Read Fifo source Register
* Input          : Byte to empty by FIFO source register value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_GetFifoSourceReg(void *handle, u8_t* val) {
  
  if( !LIS3DSH_ACC_ReadReg(handle, LIS3DSH_ACC_FIFO_SRC, val, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : GetFifoSourceBit
* Description    : Read Fifo WaterMark source bit
* Input          : FIFO_WTM_S, FIFO_EMPTY_S, FIFO_EMPTY_S...
* Output         : None
* Return         : Status of BIT [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_GetFifoSourceBit(void *handle, u8_t statusBIT){
  u8_t value;  
  
  if( !LIS3DSH_ACC_ReadReg(handle, LIS3DSH_ACC_FIFO_SRC, &value, 1) )
      return MEMS_ERROR;
 
  if(statusBIT == LIS3DSH_ACC_FIFO_WTM_S){
    if(value & LIS3DSH_ACC_FIFO_WTM_S)     return MEMS_SUCCESS;
    else  return MEMS_ERROR;  
  }
  
  if(statusBIT == LIS3DSH_ACC_FIFO_OVRN_S){
    if(value & LIS3DSH_ACC_FIFO_OVRN_S)   return MEMS_SUCCESS;
    else  return MEMS_ERROR;  
  }

  if(statusBIT == LIS3DSH_ACC_FIFO_EMPTY_S){
    if(value & LIS3DSH_ACC_FIFO_EMPTY_S)   return MEMS_SUCCESS;
    else  return MEMS_ERROR;  
  }
  
return MEMS_ERROR;
}


/*******************************************************************************
* Function Name  : GetFifoSourceFSS
* Description    : Read Fifo source Data Stored
* Input          : Byte to empty by FIFO source Data Stored value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_GetFifoSourceFSS(void *handle, u8_t* val) {
  
  if( !LIS3DSH_ACC_ReadReg(handle, LIS3DSH_ACC_FIFO_SRC, val, 1) )
    return MEMS_ERROR;
 
  *val &= 0x1F;
  
  return MEMS_SUCCESS;
}     


/*******************************************************************************
* Function Name  : ReadFifoData
* Description    : Read all Fifo Data stored
* Input          : AccAxesRaw_t Buffer to empty by FIFO Data Stored value, Byte to empty by depth of FIFO
* Note		 : Must call this function every [nMax sample * ODR] seconds max (or more fastly)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_ReadFifoData(void *handle, Type3Axis16bit_U* FifoBuff, u8_t* depth) {
  u8_t val=0;
  u8_t i=0;
  i8_t j=0;
  Type3Axis16bit_U data;
  
  
  if(! LIS3DSH_ACC_GetFifoSourceFSS(handle, &val))  //read FSS fifo value
  return MEMS_ERROR;

  if(val<1) return MEMS_ERROR; //there aren't fifo value

  //read n data from FIFO
  for(j=val;j>=0;j--){
        LIS3DSH_ACC_GetAccAxesRaw(handle, &data);
        FifoBuff[i].i16bit[0] = data.i16bit[0];
        FifoBuff[i].i16bit[1] = data.i16bit[1];
        FifoBuff[i].i16bit[2] = data.i16bit[2];
        i++;
	}
  
  *depth = val;
  
  return MEMS_SUCCESS;
}   


/*******************************************************************************
* Function Name  : SetSPIInterface
* Description    : Set SPI mode: 3 Wire Interface OR 4 Wire Interface
* Input          : SPI_3_WIRE, SPI_4_WIRE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_SetSPIInterface(void *handle, LIS3DSH_ACC_SPIMode_t spi) {
  u8_t value;
  
  if( !LIS3DSH_ACC_ReadReg(handle, LIS3DSH_ACC_CNTL5, &value, 1) )
    return MEMS_ERROR;
                  
  value &= 0xFE;
  value |= spi<<LIS3DSH_ACC_SIM;
  
  if( !LIS3DSH_ACC_WriteReg(handle, LIS3DSH_ACC_CNTL5, &value, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetSMCodeReg
* Description    : Set single SMx Code Register byte
* Input          : Code Address, Code (Byte)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_SetSMCodeReg(void *handle, u8_t CodeADD, u8_t CodeByte) {
 
  //check correct address
  if(! (((CodeADD >= 0x40)&&(CodeADD <= 0x4F)) || ((CodeADD >= 0x60)&&(CodeADD <= 0x6F))) )
	return MEMS_ERROR;
  
    if( !LIS3DSH_ACC_WriteReg(handle, CodeADD, &CodeByte, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : SetSMBufferCodeReg
* Description    : Set All SMx Code Registers by Buffer input
* Input          : SMx, Code Buffer[16]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_SetSMBufferCodeReg(void *handle, LIS3DSH_ACC_SM_t sm, u8_t* CodeBuff) {
  u8_t reg=0;
  u8_t i=0;
  
  switch(sm){
  case LIS3DSH_ACC_SM1:  reg = LIS3DSH_ACC_ST1_1; break;
  case LIS3DSH_ACC_SM2:  reg = LIS3DSH_ACC_ST1_2; break;
  }  
   
  for(i=0;i<16;i++){
    if( !LIS3DSH_ACC_WriteReg(handle, reg+i, &CodeBuff[i], 1) )
    return MEMS_ERROR;
  }
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : GetSMCodeRegister
* Description    : Get single code register number of SMx
* Input          : SMx, Register Number [1,16], variable to empty
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_ACC_GetSMCodeRegister(void *handle, LIS3DSH_ACC_SM_t sm, u8_t RegNumber, u8_t* val) {
  u8_t reg=0;
 
  if((RegNumber==0)||(RegNumber>16))    return MEMS_ERROR;
  
  switch(sm){
  case LIS3DSH_ACC_SM1:  reg = LIS3DSH_ACC_ST1_1; break;
  case LIS3DSH_ACC_SM2:  reg = LIS3DSH_ACC_ST1_2; break;
  }  
   
    if( !LIS3DSH_ACC_ReadReg(handle, reg + RegNumber-1, val, 1) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}