/******************** (C) COPYRIGHT 2017 STMicroelectronics ********************
* File Name          : FIFO_continuous.c
* Author             : MEMS Application Team
* Version            : v1.0
* Date               : 07 June 2017
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

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define SENSITIVITY_2g    (0.244) /* mg */
#define SENSITIVITY_4g    (0.488) /* mg */
#define SENSITIVITY_8g    (0.976) /* mg */
#define SENSITIVITY_16g   (1.952) /* mg */
#define SENSITIVITY_2g_LPM1    (0.976) /* mg */
#define SENSITIVITY_4g_LPM1    (1.952) /* mg */
#define SENSITIVITY_8g_LPM1    (3.904) /* mg */
#define SENSITIVITY_16g_LPM1   (7.808) /* mg */

#define FIFO_SAMPLE_THRESHOLD  15 /* sample */

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static status_t response;  
static Type3Axis16bit_U Acceleration;
static u8_t who_am_I;
static i32_t Acceleration_mg[3];
static u8_t USBbuffer[1000];

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

int example_main(void)
{ 
  /* check the device */
  response = LIS2DW12_ACC_R_WhoAmI(0, &who_am_I);
  if(response==MEMS_ERROR) while(1); //manage here comunication error
  if(who_am_I!=LIS2DW12_ACC_WHO_AM_I) while(1); //manage here different who am I
  
  
  /* 
   * configure LIS2DW12 device
   */
  
  /* select FIFO mode */  
  LIS2DW12_ACC_W_FIFO_mode(0, LIS2DW12_ACC_FMODE_STREAM);  
  /* set FIFO threshold */  
  LIS2DW12_ACC_W_FIFO_Threshold(0, FIFO_SAMPLE_THRESHOLD);   
  
  /* Configure Power Mode */
  LIS2DW12_ACC_W_ModeSelection(0, LIS2DW12_ACC_MODE_LOW_POWER_STD);
  LIS2DW12_ACC_W_LowPowerModeSelection(0, LIS2DW12_ACC_LP_MODE1_12bit); 
  LIS2DW12_ACC_W_LowNoiseConfiguration(0, LIS2DW12_ACC_LOW_NOISE_ENABLE); 
  
  /* Set Acc Full Scale */
  LIS2DW12_ACC_W_FullScaleSelection(0, LIS2DW12_ACC_FS_2g);

  /* int 1 pin configuration */
  LIS2DW12_ACC_W_PinFunction_INT1(0, LIS2DW12_ACC_INT1_MODE_FIFO_TRESHOLD); 
    
  /* Set Acc Output Data Rate - Power on device*/
  LIS2DW12_ACC_W_OutputDataRate(0, LIS2DW12_ACC_ODR_LP_50Hz_HP_50Hz);
  
  while(1)
  {
    /* check interrupt of FIFO Threshold */  
    if (HAL_GPIO_ReadPin(INT1_GPIO_Port, INT1_Pin))
    {
      /* get data */
      LIS2DW12_ACC_Get_Acceleration(0, Acceleration.u8bit);
      /* Transorm LSB into mg @ 12 bit resolution*/
      Acceleration_mg[0]=(i32_t)((Acceleration.i16bit[0]>>4)*SENSITIVITY_2g_LPM1);
      Acceleration_mg[1]=(i32_t)((Acceleration.i16bit[1]>>4)*SENSITIVITY_2g_LPM1);
      Acceleration_mg[2]=(i32_t)((Acceleration.i16bit[2]>>4)*SENSITIVITY_2g_LPM1);
      /* Write data on communication port */
      sprintf((char*)USBbuffer, "%6d\t%6d\t%6d\r\n", Acceleration_mg[0], Acceleration_mg[1], Acceleration_mg[2]);
      CDC_Transmit_FS( USBbuffer, strlen((char const*)USBbuffer) );
    }
  }
} // end main


/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/