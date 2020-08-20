/******************** (C) COPYRIGHT 2017 STMicroelectronics ********************
* File Name          : ReadDRDY.c
* Author             : MEMS Application Team
* Version            : v1.0
* Date               : 11 May 2017  
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
#define SENSITIVITY_2g          (0.244) /* mg */
#define SENSITIVITY_2g_LP_M1    (0.976) /* mg */
/* Private variables ---------------------------------------------------------*/
static status_t response;  
static Type3Axis16bit_U Acceleration;
static u8_t WhoAmI, rst;
static i32_t Acceleration_mg[3];
static u8_t USBbuffer[1000];

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
  response = LIS2DW12_ACC_W_ModeSelection(0, LIS2DW12_ACC_MODE_LOW_POWER_STD);
  if(response==MEMS_ERROR) while(1); //manage here comunication error
  
  /* Set Acc Output Data Rate */
  response = LIS2DW12_ACC_W_OutputDataRate(0, LIS2DW12_ACC_ODR_LP_100Hz_HP_100Hz);
  if(response==MEMS_ERROR) while(1); //manage here comunication error
   
  /* Set Acc Full Scale */
  response = LIS2DW12_ACC_W_FullScaleSelection(0, LIS2DW12_ACC_FS_2g);
  if(response==MEMS_ERROR) while(1); //manage here comunication error

  /* Enable Acc Block Data Update */
  response = LIS2DW12_ACC_W_BlockDataUpdate(0, LIS2DW12_ACC_BDU_ENABLE);
  if(response==MEMS_ERROR) while(1); //manage here comunication error  
  
  /* DRDY on INT2 */  
  response = LIS2DW12_ACC_W_PinFunction_INT2(0, LIS2DW12_ACC_INT2_MODE_DATA_READY);
  if(response==MEMS_ERROR) while(1); //manage here comunication error  
  
  /* Select Power mode */ 
  response = LIS2DW12_ACC_W_LowPowerModeSelection(0, LIS2DW12_ACC_LP_MODE1_12bit);
  if(response==MEMS_ERROR) while(1); //manage here comunication error  
  
  /*
   * Read samples in polling mode (no int)
   */
  while(1)
  {
    //Polling on int pin driven by DRDY signal 
    //if (HAL_GPIO_ReadPin(INT1_GPIO_Port, INT1_Pin))
    if (HAL_GPIO_ReadPin(INT2_GPIO_Port, INT2_Pin))
    {
      LIS2DW12_ACC_Get_Acceleration(0, Acceleration.u8bit);

      /* Transorm LSB into mg @ 12 bit resolution*/
      Acceleration_mg[0]=(i32_t)((Acceleration.i16bit[0]>>4)*SENSITIVITY_2g_LP_M1);
      Acceleration_mg[1]=(i32_t)((Acceleration.i16bit[1]>>4)*SENSITIVITY_2g_LP_M1);
      Acceleration_mg[2]=(i32_t)((Acceleration.i16bit[2]>>4)*SENSITIVITY_2g_LP_M1);
      
      sprintf((char*)USBbuffer, "%6d\t%6d\t%6d\r\n", Acceleration_mg[0], Acceleration_mg[1], Acceleration_mg[2]);
      CDC_Transmit_FS( USBbuffer, strlen((char const*)USBbuffer) );
    }
  }  
}

