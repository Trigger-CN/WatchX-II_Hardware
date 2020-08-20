/******************** (C) COPYRIGHT 2017 STMicroelectronics ********************
* File Name          : ReadDRDY.c
* Author             : MEMS Application Team
* Version            : v1.0
* Date               : 27 February 2016  
* Description        : LIS2MDL header driver file
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
#include "..\..\Mag\LIS2MDL_Driver__Example\trunk\LIS2MDL_Driver\LIS2MDL_MAG_driver.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define SENSITIVITY     1.5              //mG/LSB

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

u8_t WhoAmI;
status_t response;  
LIS2MDL_MAG_STATUS_t value;
Type3Axis16bit_U MagneticField;
Type1Axis16bit_U Temperature;
float magneticFieldGauss[3];

/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : main.
* Description    : LIS2MDL Example reading in DRDY signal.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int example_main(void)
{ 
  response = LIS2MDL_MAG_R_WhoAmI_Bits(0, &WhoAmI);
  if(response==MEMS_ERROR) while(1); //manage here comunication error
  
  if ( WhoAmI != LIS2MDL_MAG_WHO_AM_I_VALUE ) 
  while(1); //manage here device not found
  
  response = LIS2MDL_MAG_W_DataRate(0, LIS2MDL_MAG_ODR_50_Hz);
  if(response==MEMS_ERROR) while(1); //manage here comunication error

  response = LIS2MDL_MAG_W_BlockDataUpdate(0, LIS2MDL_MAG_BDU_ENABLE);
  if(response==MEMS_ERROR) while(1); //manage here comunication error  
  
  response = LIS2MDL_MAG_W_TemperatureCompensation(0, LIS2MDL_MAG_COMP_TEMP_ENABLE);
  if(response==MEMS_ERROR) while(1); //manage here comunication error    

  response = LIS2MDL_MAG_W_OffsetCancellation(0, LIS2MDL_MAG_OFF_CANC_ENABLED);
  if(response==MEMS_ERROR) while(1); //manage here comunication error     
  
  response = LIS2MDL_MAG_W_Operating_Mode(0, LIS2MDL_MAG_MD_CONTINUOUS);
  if(response==MEMS_ERROR) while(1); //manage here comunication error  
  
  response = LIS2MDL_MAG_W_PinMode(0, LIS2MDL_MAG_INT_MAG_DIGITAL);//[AZ]
  if(response==MEMS_ERROR) while(1); //manage here comunication error    
  
  response = LIS2MDL_MAG_W_InterruptGenerator(0, LIS2MDL_MAG_INT_MAG_PIN_INTERNAL_ONLY);//[AZ]
  if(response==MEMS_ERROR) while(1); //manage here comunication error 
  
//main loop
  while(1)
  {
    //Polling on int pin driven by DRDY signal 
    if (HAL_GPIO_ReadPin(ST_GPIO_Port, ST_Pin))
    {
      LIS2MDL_MAG_Get_MagneticOutputs(0, MagneticField.u8bit);
      //Convert from LSB to Gauss
      magneticFieldGauss[0]=MagneticField.i16bit[0] * SENSITIVITY / 1000;
      magneticFieldGauss[1]=MagneticField.i16bit[1] * SENSITIVITY / 1000;
      magneticFieldGauss[2]=MagneticField.i16bit[2] * SENSITIVITY / 1000;
    }
  }
} // end main


/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/