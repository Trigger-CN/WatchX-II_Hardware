/******************** (C) COPYRIGHT 2013 STMicroelectronics ********************
* File Name          : L3GD20H_example.c
* Author             : MEMS Application Team
* Version            : v1.0
* Date               : October 2013 
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
#include "L3GD20H_driver.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define SENSITIVITY    8.75             //mdps/LSB

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
status_t response;                          // mems error variable
AxesRaw_t data;                             // gyro row data
float mdpsData_X, mdpsData_Y,mdpsData_Z;    //data in mdps

/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : main.
* Description    : Main routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int main(void)
{ 

  //function for MKI109V1 board 
  InitHardware();
  I2C_MEMS_Init();
 
 //Initialize Sensor 
 //set Fullscale
 response = L3GD20H_SetFullScale(L3GD20H_FULLSCALE_250);
 if(response==MEMS_ERROR) while(1); //manage here comunication error
  
 //set PowerMode 
 response = L3GD20H_SetMode(L3GD20H_NORMAL);
 if(response==MEMS_ERROR) while(1); //manage here comunication error
  
 //Enable Axis 
 response = L3GD20H_SetAxis(L3GD20H_X_ENABLE | L3GD20H_Y_ENABLE | L3GD20H_Z_ENABLE);
 if(response==MEMS_ERROR) while(1); //manage here comunication error
  
 //set ODR (turn ON device)
 response = L3GD20H_SetODR(L3GD20H_ODR_100Hz_BW_25);
 if(response==MEMS_ERROR) while(1); //manage here comunication error
 

//main loop
  while(1)
  {
     //get Raw data  
     response = L3GD20H_GetAngRateRaw(&data);
        if(response==MEMS_ERROR) while(1); //manage here comunication error
     //convert from LSB to mdps
     mdpsData_X=data.AXIS_X*SENSITIVITY;
     mdpsData_Y=data.AXIS_Y*SENSITIVITY;
     mdpsData_Z=data.AXIS_Z*SENSITIVITY;
  }
} // end main


/******************* (C) COPYRIGHT 2013 STMicroelectronics *****END OF FILE****/