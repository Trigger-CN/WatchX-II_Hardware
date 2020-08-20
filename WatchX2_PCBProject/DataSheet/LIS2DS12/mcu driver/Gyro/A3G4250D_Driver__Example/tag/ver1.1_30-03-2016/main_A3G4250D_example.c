/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
* File Name          : main_A3G4250D_example.c
* Author             : MEMS Application Team
* Version            : v1.1
* Date               : 31 Mar 2016  
* Description        : A3G4250D source driver file
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
#include "A3G4250D_GYRO_driver.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define SENSITIVITY_245DPS    8.75/1000 /* dps/LSB */

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
A3G4250D_GYRO_ZYXDA_t value_G;
Type3Axis16bit_U AngularRate;
float AngularRate_dps[3];

status_t response;  

/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
void init_A3G4250D_GYRO(void)
{
  /* Gyro ODR */
  response = A3G4250D_GYRO_W_DR_bits(0, A3G4250D_GYRO_DR_100HZ);
  if(response==MEMS_ERROR) while(1); //manage here comunication error

  /* Device Enable */
  response = A3G4250D_GYRO_W_PD_bits(0, A3G4250D_GYRO_PD_NORMAL_MODE);
  if(response==MEMS_ERROR) while(1); //manage here comunication error
}

/*******************************************************************************
* Function Name  : main.
* Description    : Simple A3G4250D_GYRO Example.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int main(void)
{ 
  InitHardware();
  I2C_MEMS_Init();
  
  //configure device
  init_A3G4250D_GYRO();

//main loop
  while(1)
  {
    //Read GYRO output only if new gyro value is available
    response =  A3G4250D_GYRO_R_ZYXDA_bits(0, &value_G);
    if(response==MEMS_ERROR) while(1); //manage here comunication error  
    
    if (A3G4250D_GYRO_ZYXDA_AVAIL==value_G)
    {
      A3G4250D_GYRO_Get_AngularRate(0, AngularRate.u8bit);

      /* Transorm LSB into dps */
      AngularRate_dps[0]=AngularRate.i16bit[0]*SENSITIVITY_245DPS;
      AngularRate_dps[1]=AngularRate.i16bit[1]*SENSITIVITY_245DPS;
      AngularRate_dps[2]=AngularRate.i16bit[2]*SENSITIVITY_245DPS;
    }

  }
} // end main

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
