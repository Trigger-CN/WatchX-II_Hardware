/******************** (C) COPYRIGHT 2017 STMicroelectronics ********************
* File Name          : FreeFall.c
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
#define  FREE_FALL_DURATION     2 // ( 1 LSB = 1 TODR -> @ 50 Hz 20 ms)
#define  FREE_FALL_THRESHOLD    7 //LSB @ 2g in any case ( 0.061 mg/LSB )
/* Private variables ---------------------------------------------------------*/
static status_t response;  
static u8_t WhoAmI, rst, FreeFallStatus;
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
  
  /* FreeFall Configuration */
  /* Set FreeFall Duration */
  response = LIS2DW12_ACC_W_FreeFallDuration(0, FREE_FALL_DURATION);
  if(response==MEMS_ERROR) while(1); //manage here comunication error
  
  /* Set FreeFall Threshold */
  response = LIS2DW12_ACC_W_FreeFallThreshold(0, FREE_FALL_THRESHOLD);
  if(response==MEMS_ERROR) while(1); //manage here comunication error  
  
  /* Enable FreeFall Interrupt pin */
  response = LIS2DW12_ACC_W_PinFunction_INT1(0, LIS2DW12_ACC_INT1_MODE_FREE_FALL);
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
    //Polling on int pin driven by FF interrupt signal 
    if ( HAL_GPIO_ReadPin( INT1_GPIO_Port, INT1_Pin ) )
    {
        LIS2DW12_ACC_R_GetStatus( 0, (LIS2DW12_ACC_STATUS_t*) &FreeFallStatus );
        if (FreeFallStatus & LIS2DW12_ACC_FF_IA_DETECTED)
        {
          sprintf((char*)USBbuffer, "Free Fall Detect !\r\n");
          CDC_Transmit_FS( USBbuffer, strlen((char const*)USBbuffer) );
        }

    }
  }
}

























