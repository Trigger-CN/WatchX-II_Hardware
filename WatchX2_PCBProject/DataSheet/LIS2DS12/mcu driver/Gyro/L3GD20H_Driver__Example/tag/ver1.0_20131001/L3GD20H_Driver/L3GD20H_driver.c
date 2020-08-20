/******************** (C) COPYRIGHT 2013 STMicroelectronics ********************
* File Name          : L3GD20H_driver.c
* Author             : MEMS Application Team
* Version            : v1.0
* Date               : October 2013  
* Description        : L3GD20H source driver file
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
#include "L3GD20H_driver.h"

//EXAMPLE to fill LIS2HH_ReadReg and LIS2HH_WriteReg
//#include "i2C_mems.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/*******************************************************************************
* Function Name		: L3GD20H_ReadReg
* Description		: Generic Reading function. It must be fullfilled with either
*					: I2C or SPI reading functions					
* Input				: Register Address
* Output			: Data REad
* Return			: None
*******************************************************************************/
u8_t L3GD20H_ReadReg(u8_t Reg, u8_t* Data) {
  
  //To be completed with either I2c or SPI reading function
  //i.e.: *Data = SPI_Mems_Read_Reg( Reg );
  //EXAMPLE
  //if(!I2C_BufferRead(Data, L3GD20H_MEMS_I2C_ADDRESS, Reg, 1)) return MEMS_ERROR;
 
  return 1;
}

/*******************************************************************************
* Function Name		: L3GD20H_WriteReg
* Description		: Generic Writing function. It must be fullfilled with either
*					: I2C or SPI writing function
* Input				: Register Address, Data to be written
* Output			: None
* Return			: None
*******************************************************************************/
u8_t L3GD20H_WriteReg(u8_t Reg, u8_t Data) {
    
  //To be completed with either I2c or SPI writing function
  //i.e.: SPI_Mems_Write_Reg(Reg, Data);
  //EXAMPLE
  //I2C_ByteWrite(&Data,  L3GD20H_MEMS_I2C_ADDRESS,  Reg); 
  
  return 1;
}
/* Private functions ---------------------------------------------------------*/


/*******************************************************************************
* Function Name  : L3GD20H_SetODR
* Description    : Sets L3GD20H Output Data Rate
* Input          : Output Data Rate
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t L3GD20H_SetODR(L3GD20H_ODR_t ov){
  u8_t value;

  if( !L3GD20H_ReadReg(L3GD20H_LOW_ODR, &value) )
	return MEMS_ERROR;
		
  if (ov&0x80) value |= 0x01;
  else value &= 0xFE;

  if( !L3GD20H_WriteReg(L3GD20H_LOW_ODR, value) )
	return MEMS_ERROR;  

  if( !L3GD20H_ReadReg(L3GD20H_CTRL_REG1, &value) )
	return MEMS_ERROR;

  value &= 0x0f;
  value |= ov<<4;

  if( !L3GD20H_WriteReg(L3GD20H_CTRL_REG1, value) )
		return MEMS_ERROR;
		
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : L3GD20H_SetMode
* Description    : Sets L3GD20H Operating Mode
* Input          : Modality (NORMAL, SLEEP, POWER_DOWN)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t L3GD20H_SetMode(L3GD20H_Mode_t md) {
  u8_t value;
  
  if( !L3GD20H_ReadReg(L3GD20H_CTRL_REG1, &value) )
    return MEMS_ERROR;
                  
  switch(md) {
  
  case L3GD20H_POWER_DOWN:		
    value &= 0xF7;
    value |= (MEMS_RESET<<L3GD20H_PD);
    break;
          
  case L3GD20H_NORMAL:
    value &= 0xF7;
    value |= (MEMS_SET<<L3GD20H_PD);
    break;
          
  case L3GD20H_SLEEP:		
    value &= 0xF0;
    value |= ( (MEMS_SET<<L3GD20H_PD) | (MEMS_RESET<<L3GD20H_ZEN) | (MEMS_RESET<<L3GD20H_YEN) | (MEMS_RESET<<L3GD20H_XEN) );
    break;
          
  default:
    return MEMS_ERROR;
  }
  
  if( !L3GD20H_WriteReg(L3GD20H_CTRL_REG1, value) )
    return MEMS_ERROR;
                  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : L3GD20H_SetAxis
* Description    : Enable/Disable L3GD20H Axis
* Input          : X_ENABLE/X_DISABLE | Y_ENABLE/Y_DISABLE | Z_ENABLE/Z_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t L3GD20H_SetAxis(L3GD20H_Axis_t axis) {
  u8_t value;
  
  if( !L3GD20H_ReadReg(L3GD20H_CTRL_REG1, &value) )
    return MEMS_ERROR;
    
  value &= 0xf8;
  value |= axis;
  
  if( !L3GD20H_WriteReg(L3GD20H_CTRL_REG1, value) )
    return MEMS_ERROR;  
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : L3GD20H_SetFullScale
* Description    : Sets the L3GD20H FullScale
* Input          : FS_250/FS_500/FS_2000
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t L3GD20H_SetFullScale(L3GD20H_Fullscale_t fs) {
  u8_t value;
  
  if( !L3GD20H_ReadReg(L3GD20H_CTRL_REG4, &value) )
    return MEMS_ERROR;
                  
  value &= 0xCF;	
  value |= (fs<<L3GD20H_FS);
  
  if( !L3GD20H_WriteReg(L3GD20H_CTRL_REG4, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : L3GD20H_SetBDU
* Description    : Enable/Disable Block Data Update Functionality
* Input          : ENABLE/DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t L3GD20H_SetBDU(State_t bdu) {
  u8_t value;
  
  if( !L3GD20H_ReadReg(L3GD20H_CTRL_REG4, &value) )
    return MEMS_ERROR;
 
  value &= 0x7F;
  value |= (bdu<<L3GD20H_BDU);

  if( !L3GD20H_WriteReg(L3GD20H_CTRL_REG4, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : L3GD20H_SetBLE
* Description    : Set Endianess (MSB/LSB)
* Input          : BLE_LSB / BLE_MSB
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t L3GD20H_SetBLE(L3GD20H_Endianess_t ble) {
  u8_t value;
  
  if( !L3GD20H_ReadReg(L3GD20H_CTRL_REG4, &value) )
    return MEMS_ERROR;
                  
  value &= 0xBF;	
  value |= (ble<<L3GD20H_BLE);
  
  if( !L3GD20H_WriteReg(L3GD20H_CTRL_REG4, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : L3GD20H_HPFEnable
* Description    : Enable/Disable High Pass Filter
* Input          : ENABLE/DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t L3GD20H_HPFEnable(State_t hpf) {
  u8_t value;
  
  if( !L3GD20H_ReadReg(L3GD20H_CTRL_REG5, &value) )
    return MEMS_ERROR;
                  
  value &= 0xEF;
  value |= (hpf<<L3GD20H_HPEN);
  
  if( !L3GD20H_WriteReg(L3GD20H_CTRL_REG5, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : L3GD20H_SetHPFMode
* Description    : Set High Pass Filter Modality
* Input          : HPM_NORMAL_MODE_RES/HPM_REF_SIGNAL/HPM_NORMAL_MODE/HPM_AUTORESET_INT
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t L3GD20H_SetHPFMode(L3GD20H_HPFMode_t hpf) {
  u8_t value;
  
  if( !L3GD20H_ReadReg(L3GD20H_CTRL_REG2, &value) )
    return MEMS_ERROR;
                  
  value &= 0xCF;
  value |= (hpf<<L3GD20H_HPM);
  
  if( !L3GD20H_WriteReg(L3GD20H_CTRL_REG2, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : L3GD20H_SetHPFCutOFF
* Description    : Set High Pass CUT OFF Freq
* Input          : HPFCF_0,HPFCF_1,HPFCF_2... See Table 27 of the datasheet
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t L3GD20H_SetHPFCutOFF(L3GD20H_HPFCutOffFreq_t hpf) {
  u8_t value;
  
  if( !L3GD20H_ReadReg(L3GD20H_CTRL_REG2, &value) )
    return MEMS_ERROR;
                  
  value &= 0xF0;
  value |= (hpf<<L3GD20H_HPFC0);
  
  if( !L3GD20H_WriteReg(L3GD20H_CTRL_REG2, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
  
}


/*******************************************************************************
* Function Name  : L3GD20H_SetIntPinMode
* Description    : Set Interrupt Pin Modality (push pull or Open drain)
* Input          : PUSH_PULL/OPEN_DRAIN
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t L3GD20H_SetIntPinMode(L3GD20H_IntPinMode_t pm) {
  u8_t value;
  
  if( !L3GD20H_ReadReg(L3GD20H_CTRL_REG3, &value) )
    return MEMS_ERROR;
                  
  value &= 0xEF;
  value |= (pm<<L3GD20H_PP_OD);
  
  if( !L3GD20H_WriteReg(L3GD20H_CTRL_REG3, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : L3GD20H_SetInt1Pin
* Description    : Set Interrupt1 pin Function
* Input          : L3GD20H_I1_ON_PIN_INT1_ENABLE | L3GD20H_I1_BOOT_ON_INT1 | L3GD20H_INT1_ACTIVE_HIGH
* example        : L3GD20H_SetInt1Pin(L3GD20H_I1_ON_PIN_INT1_ENABLE | L3GD20H_I1_BOOT_ON_INT1_ENABLE | L3GD20H_INT1_ACTIVE_LOW) 
* to enable Interrupt 1 or Bootsatus on interrupt 1 pin
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t L3GD20H_SetInt1Pin(L3GD20H_Int1PinConf_t pinConf) {
  u8_t value;
  
  if( !L3GD20H_ReadReg(L3GD20H_CTRL_REG3, &value) )
    return MEMS_ERROR;
                  
  value &= 0x1F;
  value |= pinConf;
  
  if( !L3GD20H_WriteReg(L3GD20H_CTRL_REG3, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : L3GD20H_SetInt2Pin
* Description    : Set Interrupt2 pin Function
* Input          : L3GD20H_I2_DRDY_ON_INT2_ENABLE/DISABLE | 
                   L3GD20H_WTM_ON_INT2_ENABLE/DISABLE | 
                   L3GD20H_OVERRUN_ON_INT2_ENABLE/DISABLE | 
                   L3GD20H_EMPTY_ON_INT2_ENABLE/DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t L3GD20H_SetInt2Pin(L3GD20H_Int2PinConf_t pinConf) {
  u8_t value;
  
  if( !L3GD20H_ReadReg(L3GD20H_CTRL_REG3, &value) )
    return MEMS_ERROR;
                  
  value &= 0xF0;
  value |= pinConf;
  
  if( !L3GD20H_WriteReg(L3GD20H_CTRL_REG3, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;  
}


/*******************************************************************************
* Function Name  : L3GD20H_Int1LatchEnable
* Description    : Enable Interrupt 1 Latching function
* Input          : ENABLE/DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t L3GD20H_Int1LatchEnable(State_t latch) {
  u8_t value;
  
  if( !L3GD20H_ReadReg(L3GD20H_INT1_CFG, &value) )
    return MEMS_ERROR;
                  
  value &= 0xBF;
  value |= latch<<L3GD20H_LIR;
  
  if( !L3GD20H_WriteReg(L3GD20H_INT1_CFG, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : L3GD20H_ResetInt1Latch
* Description    : Reset Interrupt 1 Latching function
* Input          : None
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t L3GD20H_ResetInt1Latch(void) {
  u8_t value;
  
  if( !L3GD20H_ReadReg(L3GD20H_INT1_SRC, &value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : L3GD20H_SetIntConfiguration
* Description    : Interrupt 1 Configuration
* Input          : AND/OR, INT1_LIR ZHIE_ENABLE/DISABLE | INT1_ZLIE_ENABLE/DISABLE...
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t L3GD20H_SetIntConfiguration(L3GD20H_Int1Conf_t ic) {
  u8_t value;
  
  value = ic;

  if( !L3GD20H_WriteReg(L3GD20H_INT1_CFG, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : L3GD20H_SetInt1Threshold
* Description    : Sets Interrupt 1 Threshold
* Input          : Threshold = [0,31]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t L3GD20H_SetInt1Threshold(L3GD20H_IntThsAxis axis, u16_t ths) {
  u8_t value;
  
  switch (axis) {
    
    case L3GD20H_THS_X:
      //write the threshold LSB
      value = (u8_t)( ths & 0x00ff); 
      if( !L3GD20H_WriteReg(L3GD20H_INT1_THS_XL, value) )
        return MEMS_ERROR;
      
      //write the threshold LSB
      value = (u8_t)( ths >> 8); 
      if( !L3GD20H_WriteReg(L3GD20H_INT1_THS_XH, value) )
        return MEMS_ERROR;
      
      break;
      
    case L3GD20H_THS_Y:
      //write the threshold LSB
      value = (u8_t)( ths & 0x00ff); 
      if( !L3GD20H_WriteReg(L3GD20H_INT1_THS_YL, value) )
        return MEMS_ERROR;
      
      //write the threshold LSB
      value = (u8_t)( ths >> 8); 
      if( !L3GD20H_WriteReg(L3GD20H_INT1_THS_YH, value) )
        return MEMS_ERROR;
      
      break;
      
    case L3GD20H_THS_Z:
      //write the threshold LSB
      value = (u8_t)( ths & 0x00ff); 
      if( !L3GD20H_WriteReg(L3GD20H_INT1_THS_ZL, value) )
        return MEMS_ERROR;
      
      //write the threshold LSB
      value = (u8_t)( ths >> 8); 
      if( !L3GD20H_WriteReg(L3GD20H_INT1_THS_ZH, value) )
        return MEMS_ERROR;
      
      break;     

        
  }

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : L3GD20H_SetInt1Duration
* Description    : Sets Interrupt 1 Duration
* Input          : Duration value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t L3GD20H_SetInt1Duration(L3GD20H_Int1Conf_t id) {
 
  if (id > 127)
    return MEMS_ERROR;

  if( !L3GD20H_WriteReg(L3GD20H_INT1_DURATION, id) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : L3GD20H_FIFOModeEnable
* Description    : Sets Fifo Modality
* Input          : 
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t L3GD20H_FIFOModeEnable(L3GD20H_FifoMode_t fm) {
  u8_t value;  
  
  if(fm == L3GD20H_FIFO_DISABLE) {
    
    if( !L3GD20H_ReadReg(L3GD20H_CTRL_REG5, &value) )
      return MEMS_ERROR;
                    
    value &= 0xBF;    
    
    if( !L3GD20H_WriteReg(L3GD20H_CTRL_REG5, value) )
      return MEMS_ERROR;
    
  }
  else {
    
    if( !L3GD20H_ReadReg(L3GD20H_CTRL_REG5, &value) )
      return MEMS_ERROR;
                    
    value &= 0xBF;
    value |= MEMS_SET<<L3GD20H_FIFO_EN;
    
    if( !L3GD20H_WriteReg(L3GD20H_CTRL_REG5, value) )
      return MEMS_ERROR;
    
    
    if( !L3GD20H_ReadReg(L3GD20H_FIFO_CTRL_REG, &value) )
      return MEMS_ERROR;
    
    value &= 0x1f;
    value |= (fm<<L3GD20H_FM0);
    
    if( !L3GD20H_WriteReg(L3GD20H_FIFO_CTRL_REG, value) )
      return MEMS_ERROR;
  }

  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : L3GD20H_SetWaterMark
* Description    : Sets Watermark Value
* Input          : Watermark = [0,31]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t L3GD20H_SetWaterMark(u8_t wtm) {
  u8_t value;
  
  if(wtm > 31)
    return MEMS_ERROR;  
  
  if( !L3GD20H_ReadReg(L3GD20H_FIFO_CTRL_REG, &value) )
    return MEMS_ERROR;
                  
  value &= 0xE0;
  value |= wtm; 
  
  if( !L3GD20H_WriteReg(L3GD20H_FIFO_CTRL_REG, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : L3GD20H_GetSatusReg
* Description    : Read the status register
* Input          : None
* Output         : status register buffer
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t L3GD20H_GetSatusReg(u8_t* buff) {
  if( !L3GD20H_ReadReg(L3GD20H_STATUS_REG, buff) )
      return MEMS_ERROR;
  
  return MEMS_SUCCESS;  
}


/*******************************************************************************
* Function Name  : L3GD20H_GetAngRateRaw
* Description    : Read the Angular Rate Registers
* Input          : None
* Output         : Angular Rate Registers buffer
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t L3GD20H_GetAngRateRaw(AxesRaw_t* buff) {
  u8_t valueL;
  u8_t valueH;
  

  if( !L3GD20H_ReadReg(L3GD20H_OUT_X_L, &valueL) )
      return MEMS_ERROR;
  
  if( !L3GD20H_ReadReg(L3GD20H_OUT_X_H, &valueH) )
      return MEMS_ERROR;
  
  buff->AXIS_X = (i16_t)( (valueH << 8) | valueL );
  
  if( !L3GD20H_ReadReg(L3GD20H_OUT_Y_L, &valueL) )
      return MEMS_ERROR;
  
  if( !L3GD20H_ReadReg(L3GD20H_OUT_Y_H, &valueH) )
      return MEMS_ERROR;
  
  buff->AXIS_Y = (i16_t)( (valueH << 8) | valueL );
  
   if( !L3GD20H_ReadReg(L3GD20H_OUT_Z_L, &valueL) )
      return MEMS_ERROR;
  
  if( !L3GD20H_ReadReg(L3GD20H_OUT_Z_H, &valueH) )
      return MEMS_ERROR;
  
  buff->AXIS_Z = (i16_t)( (valueH << 8) | valueL );
  
  return MEMS_SUCCESS;  
}


/*******************************************************************************
* Function Name  : L3GD20H_GetInt1Src
* Description    : Reset Interrupt 1 Latching function
* Input          : None
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t L3GD20H_GetInt1Src(u8_t* buff) {
  
  if( !L3GD20H_ReadReg(L3GD20H_INT1_SRC, buff) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : L3GD20H_GetFifoSourceReg
* Description    : Read Fifo source Register
* Input          : None
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t L3GD20H_GetFifoSourceReg(u8_t* buff) {
  
  if( !L3GD20H_ReadReg(L3GD20H_FIFO_SRC_REG, buff) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : L3GD20H_SetOutputDataAndFifoFilters
* Description    : ENABLE/DISABLE HIGH PASS and LOW PASS filters applied to output and fifo registers
*                : See Table 8 of AN3393 for more details
* Input          : L3GD20H_NONE, L3GD20H_HPH, L3GD20H_LPF2, L3GD20H_HPFLPF2
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t L3GD20H_SetOutputDataAndFifoFilters(L3GD20H_HPF_LPF2_Enable hpf){
  u8_t value;
  
  //HPF
  if( !L3GD20H_ReadReg(L3GD20H_CTRL_REG5, &value) )
    return MEMS_ERROR;
  
  switch(hpf) {
    
  case L3GD20H_NONE:
    value &= 0xfc;
    value |= 0x00; //hpen = x, Out_sel_1 = 0, Out_sel_0 = 0
    break;
    
  case L3GD20H_HPF:
    value &= 0xfc;
    value |= 0x01; //hpen = x, Out_sel_1 = 0, Out_sel_0 = 1
    break;

  case L3GD20H_LPF2:
    value &= 0xed;
    value |= 0x02; //hpen = 0, Out_sel_1 = 1, Out_sel_0 = x
    break;    
   
  case L3GD20H_HPFLPF2:
    value &= 0xed;
    value |= 0x12; //hpen = 1, Out_sel_1 = 1, Out_sel_0 = x
    break;    
  }
  
  if( !L3GD20H_WriteReg(L3GD20H_CTRL_REG5, value) )
    return MEMS_ERROR;
  
  
  return MEMS_SUCCESS;
  
}


/*******************************************************************************
* Function Name  : L3GD20H_SetInt1Filters
* Description    : ENABLE/DISABLE HIGH PASS and LOW PASS filters applied to Int1 circuitery
*                : See Table 9 of AN3393 for more details
* Input          : NONE, HPH, LPF2, HPFLPF2
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t L3GD20H_SetInt1Filters(L3GD20H_HPF_LPF2_Enable hpf){
  u8_t value;
  
  //HPF
  if( !L3GD20H_ReadReg(L3GD20H_CTRL_REG5, &value) )
    return MEMS_ERROR;
  
  switch(hpf) {
    
  case L3GD20H_NONE:
    value &= 0xf3;
    value |= 0x00<<L3GD20H_INT1_SEL0; //hpen = x, Int1_sel_1 = 0, Int1_sel_0 = 0
    break;
    
  case L3GD20H_HPF:
    value &= 0xf3;
    value |= 0x01<<L3GD20H_INT1_SEL0; //hpen = x, Int1_sel_1 = 0, Int1_sel_0 = 1
    break;

  case L3GD20H_LPF2:
    value &= 0xe7;
    value |= 0x02<<L3GD20H_INT1_SEL0; //hpen = 0, Int1_sel_1 = 1, Int1_sel_0 = x
    break;    
   
  case L3GD20H_HPFLPF2:
    value &= 0xe7;
    value |= 0x01<<L3GD20H_HPEN;
    value |= 0x02<<L3GD20H_INT1_SEL0; //hpen = 1, Int1_sel_1 = 1, Int1_sel_0 = x
    break;    
  }
  
  if( !L3GD20H_WriteReg(L3GD20H_CTRL_REG5, value) )
    return MEMS_ERROR;
  
  
  return MEMS_SUCCESS;
  
}


/*******************************************************************************
* Function Name  : L3GD20H_SetSPIInterface
* Description    : Set SPI mode: 3 Wire Interface OR 4 Wire Interface
* Input          : L3GD20H_SPI_3_WIRE, L3GD20H_SPI_4_WIRE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t L3GD20H_SetSPIInterface(L3GD20H_SPIMode_t spi) {
  u8_t value;
  
  if( !L3GD20H_ReadReg(L3GD20H_CTRL_REG4, &value) )
    return MEMS_ERROR;
                  
  value &= 0xFE;
  value |= spi<<L3GD20H_SIM;
  
  if( !L3GD20H_WriteReg(L3GD20H_CTRL_REG4, value) )
    return MEMS_ERROR;
  
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : L3GD20H_Set_TRG_mode
* Description    : Set Trigger mode
* Input          : L3GD20H_TRG_Mode_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t L3GD20H_Set_TRG_mode(L3GD20H_TRG_Mode_t ov) {
  u8_t value;
  
  switch ( ov ) {
	case L3GD20H_LEVEL_SENSITIVE:
	  
	  if( !L3GD20H_ReadReg(L3GD20H_CTRL_REG2, &value) )
		return MEMS_ERROR;
	  value &=~0xC0;
	  value |= 0x40;
	  if( !L3GD20H_WriteReg(L3GD20H_CTRL_REG2, value) )
		return MEMS_ERROR;
		
	  if( !L3GD20H_ReadReg(L3GD20H_CTRL_REG4, &value) )
		return MEMS_ERROR;
	  value &=~0x08;
	  value |= 0x00;
	  if( !L3GD20H_WriteReg(L3GD20H_CTRL_REG4, value) )
		return MEMS_ERROR;	
	  return MEMS_SUCCESS;	  
	  break;
	case L3GD20H_EDGE_SENSITIVE:

	  if( !L3GD20H_ReadReg(L3GD20H_CTRL_REG2, &value) )
		return MEMS_ERROR;
	  value &=~0xC0;
	  value |= 0x80;
	  if( !L3GD20H_WriteReg(L3GD20H_CTRL_REG2, value) )
		return MEMS_ERROR;
		
	  if( !L3GD20H_ReadReg(L3GD20H_CTRL_REG4, &value) )
		return MEMS_ERROR;
	  value &=~0x08;
	  value |= 0x00;
	  if( !L3GD20H_WriteReg(L3GD20H_CTRL_REG4, value) )
		return MEMS_ERROR;	
	  return MEMS_SUCCESS;		
	  break;
	case L3GD20H_PULSE_SENSITIVE:

	  if( !L3GD20H_ReadReg(L3GD20H_CTRL_REG2, &value) )
		return MEMS_ERROR;
	  value &=~0xC0;
	  value |= 0x40;
	  if( !L3GD20H_WriteReg(L3GD20H_CTRL_REG2, value) )
		return MEMS_ERROR;
		
	  if( !L3GD20H_ReadReg(L3GD20H_CTRL_REG4, &value) )
		return MEMS_ERROR;
	  value &=~0x08;
	  value |= 0x08;
	  if( !L3GD20H_WriteReg(L3GD20H_CTRL_REG4, value) )
		return MEMS_ERROR;	
	  return MEMS_SUCCESS;		
	  break;  
	default:

	  if( !L3GD20H_ReadReg(L3GD20H_CTRL_REG2, &value) )
		return MEMS_ERROR;
	  value &=~0xC0;
	  value |= 0x00;
	  if( !L3GD20H_WriteReg(L3GD20H_CTRL_REG2, value) )
		return MEMS_ERROR;
		
	  if( !L3GD20H_ReadReg(L3GD20H_CTRL_REG4, &value) )
		return MEMS_ERROR;
	  value &=~0x08;
	  value |= 0x00;
	  if( !L3GD20H_WriteReg(L3GD20H_CTRL_REG4, value) )
		return MEMS_ERROR;	
	  return MEMS_SUCCESS;
	  break;
	}
  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : L3GD20H_StopOnFTH
* Description    : Stop FIFO on Threshold
* Input          : Stop FIFO on Threshold [L3GD20H_StopOn_FTH_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t L3GD20H_StopOnFTH(L3GD20H_StopOn_FTH_t ov) {
  u8_t value;
  
  if( !L3GD20H_ReadReg(L3GD20H_CTRL_REG5, &value) )
    return MEMS_ERROR;
                  
  value &= ~L3GD20H_FTH_ENABLE; //Mask
  value |= ov;
  
  if( !L3GD20H_WriteReg(L3GD20H_CTRL_REG5, value) )
    return MEMS_ERROR;
  
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : L3GD20H_IntGen_CounterMode
* Description    : Interrupt Generator Counter Mode
* Input          : LInterrupt Generator Counter Mode [L3GD20H_DCRM_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t L3GD20H_IntGen_CounterMode(L3GD20H_DCRM_t ov) {
  u8_t value;
  
  if( !L3GD20H_ReadReg(L3GD20H_INT1_THS_XH, &value) )
    return MEMS_ERROR;
                  
  value &= ~L3GD20H_DEC; //Mask
  value |= ov;
  
  if( !L3GD20H_WriteReg(L3GD20H_INT1_THS_XH, value) )
    return MEMS_ERROR;
  
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : L3GD20H_SoftwareRST
* Description    : Generate a software rst
* Input          : Generate a software rst [L3GD20H_I2C_Mode_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t L3GD20H_SoftwareRST(L3GD20H_I2C_Mode_t ov) {
  u8_t value;
  
  if( !L3GD20H_ReadReg(L3GD20H_LOW_ODR, &value) )
    return MEMS_ERROR;
                  
 value |= 0x04;
  
  if( !L3GD20H_WriteReg(L3GD20H_LOW_ODR, value) )
    return MEMS_ERROR;
  
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : L3GD20H_SetDRDY
* Description    : Set DRDY polarity
* Input          : Set DRDY polarity [L3GD20H_DRDY_HL_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t L3GD20H_SetDRDY(L3GD20H_DRDY_HL_t ov) {
  u8_t value;
  
  if( !L3GD20H_ReadReg(L3GD20H_LOW_ODR, &value) )
    return MEMS_ERROR;

 value &= ~L3GD20H_ACTIVE_LOW; //Mask	
 value |= 0x04;
  
  if( !L3GD20H_WriteReg(L3GD20H_LOW_ODR, value) )
    return MEMS_ERROR;
  
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : L3GD20H_I2C_EnDis
* Description    : Enable/Disable I2C
* Input          : Enable/Disable I2C [L3GD20H_I2C_Mode_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t L3GD20H_I2C_EnDis(L3GD20H_I2C_Mode_t ov) {
  u8_t value;
  
  if( !L3GD20H_ReadReg(L3GD20H_LOW_ODR, &value) )
    return MEMS_ERROR;

 value &= ~L3GD20H_I2C_ENABLE; //Mask	
 value |= 0x04;
  
  if( !L3GD20H_WriteReg(L3GD20H_LOW_ODR, value) )
    return MEMS_ERROR;
  
  
  return MEMS_SUCCESS;
}