/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
* File Name          : example_Pedometer.c
* Author             : MEMS Application Team
* Version            : v1.5
* Date               : 17 May 2016  
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

#include "stm32f10x.h"
#include "hw_config.h"
#include "i2C_mems.h"
#include "interruptHandler.h"
#include "LSM6DSL_ACC_GYRO_driver.h"

/* This macro can be used to switch on/off the evaluation with interrupts */
//#define TEST_WITH_INTERRUPT     1

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*
 * PEDOMETER test
 */
u16_t Number_Of_Steps = 0;

/*
 * Callback to handle the Pedometer event.
 * It must be registered to be called at interrupt time.
 */
 #ifdef TEST_WITH_INTERRUPT
void LSM6DSL_ACC_GYRO_Pedo_Callback(u8_t intID)
{
  LSM6DSL_ACC_GYRO_PEDO_EV_STATUS_t PedoStatus;

  LSM6DSL_ACC_GYRO_R_PEDO_EV_STATUS(0, &PedoStatus);
  if (PedoStatus == LSM6DSL_ACC_GYRO_PEDO_EV_STATUS_DETECTED) {
    LSM6DSL_ACC_GYRO_Get_GetStepCounter(0, (u8_t *)&Number_Of_Steps);
  }
}
#endif


/*******************************************************************************
* Function Name  : main.
* Description    : Simple LIS3MDL Example.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/

int main(void)
{ 
  u8 who_am_i = 0x0;
  status_t response; 
  
  InitHardware();
  I2C_MEMS_Init();
  
  /* Read WHO_AM_I  and check if device is really the LSM6DSL */
  LSM6DSL_ACC_GYRO_R_WHO_AM_I(0, &who_am_i);
  if (who_am_i != LSM6DSL_ACC_GYRO_WHO_AM_I)
    while(1); //manage here comunication error  

  /* configure pedometer */
  /* Set ACC ODR  */
  response = LSM6DSL_ACC_GYRO_W_ODR_XL(0, LSM6DSL_ACC_GYRO_ODR_XL_26Hz);
  if(response==MEMS_ERROR) while(1); //manage here comunication error

  /* Enable Pedometer  */
  response = LSM6DSL_ACC_GYRO_W_PEDO(0, LSM6DSL_ACC_GYRO_PEDO_ENABLED);
  if(response==MEMS_ERROR) while(1); //manage here comunication error

  /* Pedometer on INT1 */
  response = LSM6DSL_ACC_GYRO_W_STEP_DET_on_INT1(0, LSM6DSL_ACC_GYRO_INT1_PEDO_ENABLED);
  if(response==MEMS_ERROR) while(1); //manage here comunication error

  /* Enable Embedded Functions */
  LSM6DSL_ACC_GYRO_W_FUNC_EN(0, LSM6DSL_ACC_GYRO_FUNC_EN_ENABLED);

  /* configure pedometer acceleration threshold */
  LSM6DSL_ACC_GYRO_W_PedoThreshold(0, 13);

  /* Clear the step counter */
  LSM6DSL_ACC_GYRO_W_PedoStepReset(0, LSM6DSL_ACC_GYRO_PEDO_RST_STEP_ENABLED);

#ifdef TEST_WITH_INTERRUPT
  RegisterInterrupt(LSM6DSL_ACC_GYRO_Pedo_Callback);

  while(1) {
    /* Event will be handled in driver callback */
  }
#else
  LSM6DSL_ACC_GYRO_PEDO_EV_STATUS_t PedoStatus;

  /*
   * Handle the event using polling mode
   */
  while(1) {
    LSM6DSL_ACC_GYRO_R_PEDO_EV_STATUS(0, &PedoStatus);
    if (PedoStatus == LSM6DSL_ACC_GYRO_PEDO_EV_STATUS_DETECTED) {
      LSM6DSL_ACC_GYRO_Get_GetStepCounter(0, (u8_t *)&Number_Of_Steps);
    }
  }
#endif

} // end main

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
