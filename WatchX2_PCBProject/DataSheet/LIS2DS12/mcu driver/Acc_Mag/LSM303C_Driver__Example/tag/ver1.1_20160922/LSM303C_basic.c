/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
* File Name          : LSM303C_basic.c
* Author             : MEMS Application Team
* Version            : v1.1
* Date               : 22 Nov 2016  
* Description        : Simple sensor usage
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
//include files for LIS2DH12 adapter board
#include "stm32f4xx_hal.h"

//include MEMS driver
#include "..\Driver_MEMS\LSM303C\LSM303C_Driver\LSM303C_ACC_driver.h"
#include "..\Driver_MEMS\LSM303C\LSM303C_Driver\LSM303C_MAG_driver.h"
#include "i2c.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define SENSITIVITY_ACC     0.061               /* mg/LSB */
#define SENSITIVITY_MAG     0.580              /*  mgauss/LSB */

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
status_t response;                              // mems error variable
Type3Axis16bit_U data;                                 // accelerometer row data
float mgData_X, mgData_Y,mgData_Z;              //data in mg
float gaussData_X, gaussData_Y, gaussData_Z;    //data in Gauss
u8_t whoAmI;

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
int example_main(void)
{
 
  whoAmI = 0;
  LSM303C_MAG_R_WHO_AM_I_(0, &whoAmI);
  
  LSM303C_ACC_ReadReg(0, LSM303C_ACC_WHO_AM_I_REG, &whoAmI, 1);
  
     
 //Initialize Magnetometer 
 //set ODR (turn ON device)
 response = LSM303C_MAG_W_OutputDataRate(0, LSM303C_MAG_DO_10Hz);
 if(response==MEMS_ERROR) while(1); //manage here comunication error

 //set Fullscale
 response = LSM303C_MAG_W_FullScale(0, LSM303C_MAG_FS_16Ga);
 if(response==MEMS_ERROR) while(1); //manage here comunication error
  
   //set Block Data Update
 response = LSM303C_MAG_W_BlockDataUpdate(0, LSM303C_MAG_BDU_ENABLE);
 if(response==MEMS_ERROR) while(1); //manage here comunication error
  
     //set XY Axis Operation Mode
 response = LSM303C_MAG_W_OperatingModeXY(0, LSM303C_MAG_OM_HIGH);
 if(response==MEMS_ERROR) while(1) //manage here comunication error
  
     //set Z Axis Operation Mode
 response = LSM303C_MAG_W_OperatingModeZ(0, LSM303C_MAG_OMZ_HIGH);
 if(response==MEMS_ERROR) while(1); //manage here comunication error
  
     //set Continuous Mode
 response = LSM303C_MAG_W_SystemOperatingMode(0, LSM303C_MAG_MD_CONTINUOUS);
 if(response==MEMS_ERROR) while(1); //manage here comunication error  
  
 //this flag is used for reading data only when there are new data
 u8_t flag_LSM303C_ACC_STATUS_FLAGS;
 LSM303C_MAG_ZYXDA_t flag_LSM303C_MAG_XYZDA;  
 
 //Initialize Accelerometer 
 //set Fullscale
 response = LSM303C_ACC_SetFullScale(0, LSM303C_ACC_FS_2g);
 if(response==MEMS_ERROR) while(1); //manage here comunication error
  
   //set Block Data Update
 response = LSM303C_ACC_BlockDataUpdate(0, LSM303C_ACC_BDU_ENABLE);
 if(response==MEMS_ERROR) while(1); //manage here comunication error
  
 //Enable Axis 
 response = LSM303C_ACC_EnableAxis(0, LSM303C_ACC_X_ENABLE|LSM303C_ACC_Y_ENABLE|LSM303C_ACC_Z_ENABLE);
 if(response==MEMS_ERROR) while(1); //manage here comunication error
  
 //set ODR (turn ON device)
 response = LSM303C_ACC_SetODR(0, LSM303C_ACC_ODR_100_Hz);
 if(response==MEMS_ERROR) while(1); //manage here comunication error
 
//main loop
  while(1)
  {
       //get Acceleration Raw data  
       response = LSM303C_ACC_Status_Flags(0, &flag_LSM303C_ACC_STATUS_FLAGS);
       if(response==MEMS_ERROR) while(1); //manage here comunication error
       //read only if new data are available from accelerometer  
       else if (flag_LSM303C_ACC_STATUS_FLAGS&LSM303C_ACC_ZYX_NEW_DATA_AVAILABLE)
       {
          response = LSM303C_ACC_GetAccRaw(0, data.u8bit);
          //convert from LSB to mg
          mgData_X=data.i16bit[0]*SENSITIVITY_ACC;
          mgData_Y=data.i16bit[1]*SENSITIVITY_ACC;
          mgData_Z=data.i16bit[2]*SENSITIVITY_ACC;
       }
	   //get Magnetic Raw data 

       response = LSM303C_MAG_R_NewXYZData(0, (LSM303C_MAG_ZYXDA_t*) &flag_LSM303C_MAG_XYZDA);
       if(response==MEMS_ERROR) while(1); //manage here comunication error
       //read only if new data are available  
       else if ((LSM303C_MAG_ZYXDA_t) flag_LSM303C_MAG_XYZDA & LSM303C_MAG_ZYXDA_AVAILABLE)
       {
          response = LSM303C_MAG_Get_Magnetic(0, data.u8bit); 
          //convert from LSB to Gauss
          gaussData_X=data.i16bit[0]*SENSITIVITY_MAG;
          gaussData_Y=data.i16bit[1]*SENSITIVITY_MAG;
          gaussData_Z=data.i16bit[2]*SENSITIVITY_MAG;
       }
  }
} // end main


/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/