/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
* File Name          : main_LIS3DSH_example_.c
* Author             : MEMS Application Team
* Version            : v1.1
* Date               : 05 May 2016  
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
//include files for MKI109V1 board 
#include "stm32f10x.h"
#include "i2c_mems.h"
#include "hw_config.h"


//include MEMS driver
#include "LIS3DSH_ACC_driver.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void Function_Error(void);


/*******************************************************************************
* Function Name  : main.
* Description    : Main routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
  
int main(void)
{
  u8_t dummy=0;
  Type3Axis16bit_U data;

  //Initialize your hardware here
  
  //function for MKI109V1 board 
  InitHardware();
  I2C_MEMS_Init();    
  
  //Get device information to check
  if(! LIS3DSH_ACC_GetWHO_AM_I(0, &dummy))
    Function_Error();
  if ( dummy != LIS3DSH_ACC_I_AM_LIS3DSH )
    Function_Error();
  
  //Set device parameter
  if(! LIS3DSH_ACC_SetFullScale(0, LIS3DSH_ACC_FULLSCALE_2))
    	Function_Error();
  if(! LIS3DSH_ACC_SetAxis(0, LIS3DSH_ACC_X_ENABLE | LIS3DSH_ACC_Y_ENABLE | LIS3DSH_ACC_Z_ENABLE))
    	Function_Error();
  if(!LIS3DSH_ACC_SetODR(0, LIS3DSH_ACC_ODR_3_125)) 
     Function_Error();
  
  while(1){   
    //Get Device Status	
    if(! LIS3DSH_ACC_GetSatusReg(0, &dummy))
       Function_Error();
    
     //Check if new data are available	
     if ( dummy&LIS3DSH_ACC_STATUS_REG_ZYXDA ){
        //Get new data
        if(! LIS3DSH_ACC_GetAccAxesRaw(0, &data))
          Function_Error();
     }
  }  
  
} // end main


/*******************************************************************************
* Function Name  : Function_Error
* Description    : Generic Error function
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Function_Error(void)
{
    while(1); 
}



//function for MKI109V1 board 
#ifdef USE_FULL_ASSERT
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif

/******************* (C) COPYRIGHT 2013 STMicroelectronics *****END OF FILE****/
