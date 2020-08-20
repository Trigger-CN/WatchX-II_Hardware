/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
* File Name          : main_LSM6DS0_example.c
* Author             : MEMS Application Team
* Version            : v1.1
* Date               : 10 August 2016  
* Description        : LSM6DS0 header driver file
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
#include "LSM6DS0_ACC_GYRO_driver.h"


/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define SENSITIVITY_ACC                    0.061 //   mg/LSb @ FS = 2 g
#define SENSITIVITY_GYRO                   70. //   mdps/LSb @ FS = 2000 dps
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

status_t response;  
LSM6DS0_ACC_GYRO_GDA_t gyro_DRDY;
LSM6DS0_ACC_GYRO_XLDA_t acc_DRDY;
Type3Axis16bit_U accData, gyroData;;
float gyroData_mdps[3];
float accData_mg[3];

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
  response = LSM6DS0_ACC_GYRO_W_GyroFullScale(0, LSM6DS0_ACC_GYRO_FS_G_2000dps);
  if(response==MEMS_ERROR) while(1); //manage here comunication error

  response = LSM6DS0_ACC_GYRO_W_GyroDataRate(0, LSM6DS0_ACC_GYRO_ODR_G_60Hz);
  if(response==MEMS_ERROR) while(1); //manage here comunication error

  response = LSM6DS0_ACC_GYRO_W_AccelerometerFullScale(0, LSM6DS0_ACC_GYRO_FS_XL_2g);
  if(response==MEMS_ERROR) while(1); //manage here comunication error  
  
  response = LSM6DS0_ACC_GYRO_W_AccelerometerDataRate(0, LSM6DS0_ACC_GYRO_ODR_XL_10Hz);
  if(response==MEMS_ERROR) while(1); //manage here comunication error    

  response = LSM6DS0_ACC_GYRO_W_BlockDataUpdate(0, LSM6DS0_ACC_GYRO_BDU_ENABLE);
  if(response==MEMS_ERROR) while(1); //manage here comunication error     
  
//main loop
  while(1)
  {
    //Read output only if new value is available
    response =  LSM6DS0_ACC_GYRO_R_GyroDataReadyFlag(0, &gyro_DRDY);
    if(response==MEMS_ERROR) while(1); //manage here comunication error  
    
    response =  LSM6DS0_ACC_GYRO_R_AccelerometerDataReadyFlag(0, &acc_DRDY);
    if(response==MEMS_ERROR) while(1); //manage here comunication error  
	
    if (LSM6DS0_ACC_GYRO_GDA_UP==gyro_DRDY)
    {
      LSM6DS0_ACC_GYRO_Get_AngularRate(0, gyroData.u8bit);
      gyroData_mdps[0]=gyroData.i16bit[0]*SENSITIVITY_GYRO;
      gyroData_mdps[1]=gyroData.i16bit[1]*SENSITIVITY_GYRO;
      gyroData_mdps[2]=gyroData.i16bit[2]*SENSITIVITY_GYRO;
    }
	
	if (LSM6DS0_ACC_GYRO_XLDA_UP==acc_DRDY)
    {
      LSM6DS0_ACC_GYRO_Get_Acceleration(0, accData.u8bit);
      accData_mg[0]=accData.i16bit[0]*SENSITIVITY_ACC;
      accData_mg[1]=accData.i16bit[1]*SENSITIVITY_ACC;
      accData_mg[2]=accData.i16bit[2]*SENSITIVITY_ACC;
	}
  }
} // end main


/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
