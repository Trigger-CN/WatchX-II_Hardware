/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
* File Name          : main__LSM330_example.c
* Author             : MEMS Application Team
* Version            : v1.1
* Date               : 20 December 2016   
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

#include "LSM330_ACC_driver.h"
#include "LSM330_GYRO_driver.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define SENSITIVITY_GYRO     70              //mdps/lsb
#define SENSITIVITY_ACC     0.061             //mg/lsb


/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

status_t response;  
LSM330_ACC_DRDY_t value;
Type3Axis16bit_U acceleration,angularRate;
float acceleration_mg[3], angularRate_mpds[3];

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
int main(void)
{ 
  InitHardware();
  I2C_MEMS_Init();
  
  //configure device
  //accelerometer
  response =  LSM330_ACC_W_FullScale(0, LSM330_ACC_FSCALE_2g);
  if(response==MEMS_ERROR) while(1); //manage here comunication error

  response = LSM330_ACC_W_BlockDataUpdate(0, LSM330_ACC_BDU_ENABLE);
  if(response==MEMS_ERROR) while(1); //manage here comunication error  
  
  response = LSM330_ACC_W_AxisX(0, LSM330_ACC_XEN_ENABLE);
  if(response==MEMS_ERROR) while(1); //manage here comunication error    

  response = LSM330_ACC_W_AxisY(0, LSM330_ACC_YEN_ENABLE);
  if(response==MEMS_ERROR) while(1); //manage here comunication error     
  
  response = LSM330_ACC_W_AxisZ(0, LSM330_ACC_ZEN_ENABLE);
  if(response==MEMS_ERROR) while(1); //manage here comunication error  

  response = LSM330_ACC_W_OutputDataRate(0, LSM330_ACC_ODR_100Hz);
  if(response==MEMS_ERROR) while(1); //manage here comunication error  
  
  //gyro
  response = LSM330_GYRO_W_SystemStatus(0, LSM330_GYRO_PD_NORMAL_OR_SLEEP);
  if(response==MEMS_ERROR) while(1); //manage here comunication error

  response = LSM330_GYRO_W_Bandwidth(0, LSM330_GYRO_BW_HIGH);
  if(response==MEMS_ERROR) while(1); //manage here comunication error  
  
  response = LSM330_GYRO_W_DataRate(0, LSM330_GYRO_DR_190Hz);
  if(response==MEMS_ERROR) while(1); //manage here comunication error    

  response = LSM330_GYRO_W_FullScale(0, LSM330_GYRO_FS_2000dps);
  if(response==MEMS_ERROR) while(1); //manage here comunication error     
  
  response = LSM330_GYRO_W_BlockDataUpdate(0, LSM330_GYRO_BDU_ENABLE);
  if(response==MEMS_ERROR) while(1); //manage here comunication error  

//main loop 
  while(1)
  {
    //Read output only if new value is available
    response =  LSM330_ACC_R_DataReadyFlag(0, &value);
    if(response==MEMS_ERROR) while(1); //manage here comunication error  
    
    if (LSM330_ACC_DRDY_UP==value)
    {
      LSM330_GYRO_Get_AngularRate(0, angularRate.u8bit);
      angularRate_mpds[0]=angularRate.i16bit[0]*SENSITIVITY_GYRO;
      angularRate_mpds[1]=angularRate.i16bit[1]*SENSITIVITY_GYRO;
      angularRate_mpds[2]=angularRate.i16bit[2]*SENSITIVITY_GYRO;      
      
      LSM330_ACC_Get_Acceleration(0, acceleration.u8bit);
      acceleration_mg[0]=acceleration.i16bit[0]*SENSITIVITY_ACC;
      acceleration_mg[1]=acceleration.i16bit[1]*SENSITIVITY_ACC;
      acceleration_mg[2]=acceleration.i16bit[2]*SENSITIVITY_ACC;
    }
  }
} // end main


/******************* (C) COPYRIGHT 2013 STMicroelectronics *****END OF FILE****/