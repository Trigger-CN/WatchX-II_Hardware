/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
* File Name          : main_LIS3MDL_example.c
* Author             : MEMS Application Team
* Version            : v1.2
* Date               : 09 August 2016  
* Description        : LIS3MDL header driver file
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

#include "stm32f10x.h"
#include "hw_config.h"
#include "i2C_mems.h"
#include "LIS3MDL_MAG_driver.h"




/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define SENSITIVITY     1711.0              //LSB/Ga

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

status_t response;  
LIS3MDL_MAG_ZYXDA_t value;
Type3Axis16bit_U MagneticField;
float magneticFieldGauss[3];

/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : main.
* Description    : Simple LIS3MDL Example.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int main(void)
{ 
  InitHardware();
  I2C_MEMS_Init();
  
  //configure device
  response = LIS3MDL_MAG_W_FullScale(0, LIS3MDL_MAG_FS_16Ga);
  if(response==MEMS_ERROR) while(1); //manage here comunication error

  response = LIS3MDL_MAG_W_OutputDataRate(0, LIS3MDL_MAG_DO_0_625Hz);
  if(response==MEMS_ERROR) while(1); //manage here comunication error

  response = LIS3MDL_MAG_W_BlockDataUpdate(0, LIS3MDL_MAG_BDU_ENABLE);
  if(response==MEMS_ERROR) while(1); //manage here comunication error  
  
  response = LIS3MDL_MAG_W_OperatingModeXY(0, LIS3MDL_MAG_OM_HIGH);
  if(response==MEMS_ERROR) while(1); //manage here comunication error    

  response = LIS3MDL_MAG_W_OperatingModeZ(0, LIS3MDL_MAG_OMZ_HIGH);
  if(response==MEMS_ERROR) while(1); //manage here comunication error     
  
  response = LIS3MDL_MAG_W_SystemOperatingMode(0, LIS3MDL_MAG_MD_CONTINUOUS);;
  if(response==MEMS_ERROR) while(1); //manage here comunication error  
  

//main loop
  while(1)
  {
    //Read output only if new value is available
    response =  LIS3MDL_MAG_R_NewXYZData(0, &value);
    if(response==MEMS_ERROR) while(1); //manage here comunication error  
    
    if (LIS3MDL_MAG_ZYXDA_AVAILABLE==value)
    {
      LIS3MDL_MAG_Get_Magnetic(0, MagneticField.u8bit);
      magneticFieldGauss[0]=MagneticField.i16bit[0]*1000/SENSITIVITY;
      magneticFieldGauss[1]=MagneticField.i16bit[1]*1000/SENSITIVITY;
      magneticFieldGauss[2]=MagneticField.i16bit[2]*1000/SENSITIVITY;
    }
  }
} // end main


/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/