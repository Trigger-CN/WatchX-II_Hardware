/******************** (C) COPYRIGHT 2017 STMicroelectronics ********************
* File Name          : Wakeup.c
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

#define  WAKE_UP_THRESHOLD    5 /* 1 LSB = 1/64 of FS */

/* Private variables ---------------------------------------------------------*/
static status_t response;  
static u8_t WhoAmI, rst, WakeUpStatus;
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
  
  /* Set Acc Full Scale */
  response = LIS2DW12_ACC_W_FullScaleSelection(0, LIS2DW12_ACC_FS_2g);
  if(response==MEMS_ERROR) while(1); //manage here comunication error
  
  /* Configure Power Mode */
  response = LIS2DW12_ACC_W_ModeSelection(0, LIS2DW12_ACC_MODE_HIGH_PERFORMANCE);
  if(response==MEMS_ERROR) while(1); //manage here comunication error
  
  /* Wake Up Configuration */
  /* Set Wake Up Duration */
  response = LIS2DW12_ACC_W_UserOffsetOnWakeUp(0, LIS2DW12_ACC_USR_OFF_ON_WU_DISABLE);
  if(response==MEMS_ERROR) while(1); //manage here comunication error
  /* Set Wake Up  Threshold */
  response = LIS2DW12_ACC_W_WakeUpOffset_X(0, 0);
  if(response==MEMS_ERROR) while(1); //manage here comunication error  
  response = LIS2DW12_ACC_W_WakeUpOffset_Y(0, 0);
  if(response==MEMS_ERROR) while(1); //manage here comunication error  
  response = LIS2DW12_ACC_W_WakeUpOffset_Z(0, 0);
  if(response==MEMS_ERROR) while(1); //manage here comunication error  
  
  /* Enable Wake Up Interrupt pin */
  response = LIS2DW12_ACC_W_PinFunction_INT1(0, LIS2DW12_ACC_INT1_MODE_WAKE_UP);
  if(response==MEMS_ERROR) while(1); //manage here comunication error    
  /* set Wake up threshold */
  response = LIS2DW12_ACC_W_WakeUpThreshold(0, WAKE_UP_THRESHOLD);
  if(response==MEMS_ERROR) while(1); //manage here comunication error 
  
  /* PIN Configuration */
  /* Enable INT signals on PIN */
  response =  LIS2DW12_ACC_W_HardwarePin(0, LIS2DW12_ACC_INTERRUPTS_ENABLE_ENABLE);
  if(response==MEMS_ERROR) while(1); //manage here comunication error
  
  /* Set INT notification mode */
  response = LIS2DW12_ACC_W_LatchIntteruptRq(0, LIS2DW12_ACC_LIR_DISABLE);
  if(response==MEMS_ERROR) while(1); //manage here comunication error
  
  /* Powre on axl */
  /* Set ACC ODR  HR_14bit 50Hz*/
  response = LIS2DW12_ACC_W_OutputDataRate(0, LIS2DW12_ACC_ODR_LP_50Hz_HP_50Hz);
  if(response==MEMS_ERROR) while(1); //manage here comunication error

  /*
   * Read samples 
   */
  while(1)
  {
    //Polling on int pin driven by WU interrupt signal 
    if ( HAL_GPIO_ReadPin( INT1_GPIO_Port, INT1_Pin ) )
    {
        LIS2DW12_ACC_R_GetStatus( 0, (LIS2DW12_ACC_STATUS_t*) &WakeUpStatus );
        if (WakeUpStatus & LIS2DW12_ACC_WU_IA_DETECTED)
        {
            sprintf((char*)USBbuffer, "Wake Up Detect !\r\n");
            CDC_Transmit_FS( USBbuffer, strlen((char const*)USBbuffer) );
        }
    }
  }
}

