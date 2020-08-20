/******************** (C) COPYRIGHT 2017 STMicroelectronics ********************
* File Name          : ReadData.c
* Author             : MEMS Application Team
* Version            : v1.0
* Date               : 03 Mar 2017  
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
#include "..\..\Acc\LIS2DW12_Driver_Example\trunk\LIS2DW12_Driver\LIS2DW12_ACC_driver.h"
#include "stm32f1xx_hal.h"
#include "usbd_cdc_if.h"

/* Private macro -------------------------------------------------------------*/
#define SENSITIVITY_2g    (0.244) /* mg */
/* Private variables ---------------------------------------------------------*/
static status_t response;  
static Type3Axis16bit_U Acceleration;
static u8_t value_XL, WhoAmI, rst;
static i32_t Acceleration_mg[3];
static uint8_t USBbuffer[1000];

/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Test Acquisition of sensor samples */
int example_main(void)
{
  /* Check device ID */  
  response = LIS2DW12_ACC_R_WhoAmI(0, &WhoAmI);
  if(response==MEMS_ERROR) while(1); //manage here comunication error
  if ( WhoAmI != LIS2DW12_ACC_WHO_AM_I ) 
  while(1); //manage here device not found  
  
  /* Restore default configuration */
  response = LIS2DW12_ACC_W_SoftReset(0, LIS2DW12_ACC_SOFT_RESET_ENABLE);
  if(response==MEMS_ERROR) while(1); //manage here comunication error
  do 
  {
    response = LIS2DW12_ACC_R_SoftReset(0, (LIS2DW12_ACC_SOFT_RESET_t*) &rst);
    if(response==MEMS_ERROR) while(1); //manage here comunication error
  }
  while (rst);

  /* Configure Power Mode */
  response = LIS2DW12_ACC_W_ModeSelection(0, LIS2DW12_ACC_MODE_HIGH_PERFORMANCE);
  if(response==MEMS_ERROR) while(1); //manage here comunication error
   
  /* Set Acc Full Scale */
  response = LIS2DW12_ACC_W_FullScaleSelection(0, LIS2DW12_ACC_FS_2g);
  if(response==MEMS_ERROR) while(1); //manage here comunication error

  /* Enable Acc Block Data Update */
  response = LIS2DW12_ACC_W_BlockDataUpdate(0, LIS2DW12_ACC_BDU_ENABLE);
  if(response==MEMS_ERROR) while(1); //manage here comunication error  

  /* Set Acc Output Data Rate */
  response = LIS2DW12_ACC_W_OutputDataRate(0, LIS2DW12_ACC_ODR_LP_100Hz_HP_100Hz);
  if(response==MEMS_ERROR) while(1); //manage here comunication error

  
  /*
   * Read samples in polling mode (no int)
   */
  while(1)
  {
    /* Read output only if new value is available */
    response = LIS2DW12_ACC_R_GetStatus(0, (LIS2DW12_ACC_STATUS_t*) &value_XL);
    if(response==MEMS_ERROR) while(1); //manage here comunication error  
    
    if (LIS2DW12_ACC_DRDY_NEW_AVAILABLE == value_XL) 
    {
      LIS2DW12_ACC_Get_Acceleration(0, Acceleration.u8bit);

      /* Transorm LSB into mg @ 14 bit resolution*/
      Acceleration_mg[0] = (i32_t)((Acceleration.i16bit[0]>>2)*SENSITIVITY_2g);
      Acceleration_mg[1] = (i32_t)((Acceleration.i16bit[1]>>2)*SENSITIVITY_2g);
      Acceleration_mg[2] = (i32_t)((Acceleration.i16bit[2]>>2)*SENSITIVITY_2g);
      
      sprintf((char*)USBbuffer, "%6d\t%6d\t%6d\r\n", Acceleration_mg[0], Acceleration_mg[1], Acceleration_mg[2]);
      CDC_Transmit_FS( USBbuffer, strlen((char const*)USBbuffer) );
    }
  }  
}



