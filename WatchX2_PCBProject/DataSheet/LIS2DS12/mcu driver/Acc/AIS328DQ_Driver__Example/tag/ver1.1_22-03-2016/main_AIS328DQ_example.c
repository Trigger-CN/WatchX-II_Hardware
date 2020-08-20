/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
* File Name          : main_AIS328DQ_example.c
* Author             : MEMS Application Team
* Version            : v1.1
* Date               : 22 Mar 2016  
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
#include "AIS328DQ_ACC_driver.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define SENSITIVITY_4G    1.95/1000 /* G/LSB */
#define CONVERSION        SENSITIVITY_4G/16 /* sensor provides 12 bit samples left-aligned*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
AIS328DQ_ACC_ZYXDA_t value_XL;
Type3Axis16bit_U Acceleration;
float Acceleration_G[3];

status_t response;  

/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
void init_AIS328DQ_ACC(void)
{
  /* Acc ODR */
  response = AIS328DQ_ACC_W_CTRL_REG1_ODR(0, AIS328DQ_ACC_DR_100HZ);
  if(response==MEMS_ERROR) while(1); //manage here comunication error

  /* Set Full Scale to 4G */
  response = AIS328DQ_ACC_W_CTRL_REG4_FS(0, AIS328DQ_ACC_FS_4G);
  if(response==MEMS_ERROR) while(1); //manage here comunication error

  /* BDU Enable */
  response = AIS328DQ_ACC_W_CTRL_REG4_BDU(0, AIS328DQ_ACC_BDU_ENABLE);
  if(response==MEMS_ERROR) while(1); //manage here comunication error

  /* Device Enable */
  response = AIS328DQ_ACC_W_CTRL_REG1_PM(0, AIS328DQ_ACC_PM_NORMAL_MODE);
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
  init_AIS328DQ_ACC();

//main loop
  while(1)
  {
    //Read GYRO output only if new gyro value is available
    response =  AIS328DQ_ACC_R_STATUS_ZYXDA(0, &value_XL);
    if(response==MEMS_ERROR) while(1); //manage here comunication error  
    
    if (AIS328DQ_ACC_ZYXDA_AVAIL==value_XL)
    {
      AIS328DQ_ACC_Get_Acceleration(0, Acceleration.u8bit);

      /* Transorm LSB into dps */
      Acceleration_G[0]=Acceleration.i16bit[0]*CONVERSION;
      Acceleration_G[1]=Acceleration.i16bit[1]*CONVERSION;
      Acceleration_G[2]=Acceleration.i16bit[2]*CONVERSION;
    }

  }
} // end main

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
