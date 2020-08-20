/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
* File Name          : LIS2HH12_example.c
* Author             : MEMS Application Team
* Version            : v1.1
* Date               : 06 June 2016  
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
#include "LIS2HH12_driver.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define SENSITIVITY    0.061              //LSB/mg

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
status_t response;                          // mems error variable
Type3Axis16bit_U data;                             // accelerometer row data
float mgData_X, mgData_Y,mgData_Z; 			//data in mg

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
 //this flag is used for reading data only when there are new data
 u8_t flag_LIS2HH12_STATUS_FLAGS; 
  
  //function for MKI109V1 board 
  InitHardware();
  I2C_MEMS_Init();
 
 //Initialize Sensor 
 //set Fullscale
 response = LIS2HH12_SetFullScale(0, LIS2HH12_FS_2g);
 if(response==MEMS_ERROR) while(1); //manage here comunication error
  
   //set Block Data Update
 response = LIS2HH12_BlockDataUpdate(0, LIS2HH12_BDU_ENABLE);
 if(response==MEMS_ERROR) while(1); //manage here comunication error
  
 //Enable Axis 
 response = LIS2HH12_EnableAxis(0, LIS2HH12_X_ENABLE|LIS2HH12_Y_ENABLE|LIS2HH12_Z_ENABLE);
 if(response==MEMS_ERROR) while(1); //manage here comunication error
  
 //set ODR (turn ON device)
 response = LIS2HH12_SetODR(0, LIS2HH12_ODR_100_Hz);
 if(response==MEMS_ERROR) while(1); //manage here comunication error
 

//main loop
  while(1)
  {
       //get Acceleration Raw data  
       response = LIS2HH12_Status_Flags(0, &flag_LIS2HH12_STATUS_FLAGS);
       if(response==MEMS_ERROR) while(1); //manage here comunication error
       //read only if new data are available  
       else if (flag_LIS2HH12_STATUS_FLAGS&LIS2HH12_ZYX_NEW_DATA_AVAILABLE)
       {
          response = LIS2HH12_GetAccRaw(0, data.u8bit);
          //convert from LSB to mg
          mgData_X=data.i16bit[0]*SENSITIVITY;
          mgData_Y=data.i16bit[1]*SENSITIVITY;
          mgData_Z=data.i16bit[2]*SENSITIVITY;
       }
  }
} // end main


/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/