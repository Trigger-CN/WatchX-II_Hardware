/******************** (C) COPYRIGHT 2017 STMicroelectronics ********************
* File Name          : Interrupt.c
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
#include "stm32f1xx_hal.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define SENSITIVITY     1.5              //mG/LSB

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

u8_t WhoAmI, int_src;
status_t response;  
LIS2MDL_MAG_STATUS_t value;
Type3Axis16bit_U MagneticField;
Type1Axis16bit_U Temperature, MagThreshold;
float magneticFieldGauss[3];

/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : main.
* Description    : Simple LIS2MDL Interrupt Example.
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
  
  response = LIS2MDL_MAG_W_DataRate(0, LIS2MDL_MAG_ODR_100_Hz);
  if(response==MEMS_ERROR) while(1); //manage here comunication error
  
  response = LIS2MDL_MAG_W_Operating_Mode(0, LIS2MDL_MAG_MD_CONTINUOUS);
  if(response==MEMS_ERROR) while(1); //manage here comunication error    
  
  response = LIS2MDL_MAG_W_PinMode(0, LIS2MDL_MAG_INT_MAG_DRDY);
  if(response==MEMS_ERROR) while(1); //manage here comunication error    
  response = LIS2MDL_MAG_W_InterruptGenerator(0, LIS2MDL_MAG_INT_MAG_PIN_ROUTED_ON_PIN);
  if(response==MEMS_ERROR) while(1); //manage here comunication error  
  
  response = LIS2MDL_MAG_W_InterruptOn_Xaxis(0, LIS2MDL_MAG_XIEN_DISABLE);
  if(response==MEMS_ERROR) while(1); //manage here comunication error
  response = LIS2MDL_MAG_W_InterruptOn_Yaxis(0, LIS2MDL_MAG_YIEN_DISABLE);
  if(response==MEMS_ERROR) while(1); //manage here comunication error
  response = LIS2MDL_MAG_W_InterruptOn_Zaxis(0, LIS2MDL_MAG_ZIEN_ENABLE);
  if(response==MEMS_ERROR) while(1); //manage here comunication error
  
  response = LIS2MDL_MAG_W_InterruptMode(0, LIS2MDL_MAG_IEL_LATCHED);
  if(response==MEMS_ERROR) while(1); //manage here comunication error
  response = LIS2MDL_MAG_W_InterruptPolarity(0, LIS2MDL_MAG_IEA_HIGH);
  if(response==MEMS_ERROR) while(1); //manage here comunication error
  
  response = LIS2MDL_MAG_W_InterruptGeneration(0, LIS2MDL_MAG_IEN_ENABLE);
  if(response==MEMS_ERROR) while(1); //manage here comunication error
  
  MagThreshold.i16bit = (i16_t) (200 / SENSITIVITY);
  LIS2MDL_MAG_Set_InterruptThreshold(0, MagThreshold.u8bit); 
 
//main loop
  while(1)
  {
    //Read output only if new value is available
    response =  LIS2MDL_MAG_R_STATUS_bits(0, &value);
    if(response==MEMS_ERROR) while(1); //manage here comunication error  
    
    //Polling on interrupt pin
    if (HAL_GPIO_ReadPin(ST_GPIO_Port, ST_Pin)) 
    {
      LIS2MDL_MAG_Get_MagneticOutputs(0, MagneticField.u8bit);
      
      //Convert from LSB to Gauss
      magneticFieldGauss[0]=MagneticField.i16bit[0] * SENSITIVITY / 1000;
      magneticFieldGauss[1]=MagneticField.i16bit[1] * SENSITIVITY / 1000;
      magneticFieldGauss[2]=MagneticField.i16bit[2] * SENSITIVITY / 1000;
      
      //read interrupt source
      LIS2MDL_MAG_R_InterruptSources(0, (LIS2MDL_MAG_INT_t*)&int_src);
    }
  }
} // end main


/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/