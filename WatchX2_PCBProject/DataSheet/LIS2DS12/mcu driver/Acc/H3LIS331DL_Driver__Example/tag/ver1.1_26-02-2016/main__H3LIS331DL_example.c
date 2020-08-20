/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
* File Name          : main__H3LIS331DL_example.c
* Author             : MEMS Application Team
* Version            : v1.1
* Date               : 26 Feb 2016  
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
#include "hw_config.h"
#include "i2C_mems.h"	
#include <stdio.h>

//include MEMS driver
#include "H3LIS331DL_ACC_driver.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t response;

/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

//define for example1 or example2
#define __EXAMPLE1__H 
//#define __EXAMPLE2__H 


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
  
  //Inizialize MEMS Sensor
  //set ODR (turn ON device)
  H3LIS331DL_SetODR(0, H3LIS331DL_ODR_100Hz);
  //set PowerMode 
  H3LIS331DL_SetMode(0, H3LIS331DL_NORMAL);
  //set Fullscale
  H3LIS331DL_SetFullScale(0, H3LIS331DL_FULLSCALE_2);
  //set axis Enable
  H3LIS331DL_SetAxis(0, H3LIS331DL_X_ENABLE | H3LIS331DL_Y_ENABLE |  H3LIS331DL_Z_ENABLE);
   
 
/******Example 1******/ 
#ifdef __EXAMPLE1__H 
  AxesRaw_t data;
  while(1){
    //get Acceleration Raw data  
    H3LIS331DL_GetAccAxesRaw(0, &data);
    /* sprintf((char*)buffer, "X=%6d Y=%6d Z=%6d\r\n", data.AXIS_X, data.AXIS_Y, data.AXIS_Z);*/
  }
#endif /* __EXAMPLE1__H  */ 
 

 /******Example 2******/
#ifdef __EXAMPLE2__H
 //Inizialize MEMS Sensor
 //set Interrupt Threshold 
 response = H3LIS331DL_SetInt1Threshold(20);
 if(response==1){
   len = sprintf((char*)buffer,"SET_THRESHOLD_OK\n\r");
   USB_SIL_Write(EP1_IN, buffer, len);
   SetEPTxValid(ENDP1);
 }
 //set Interrupt configuration (all enabled)
 response = H3LIS331DL_SetInt1Configuration(0, H3LIS331DL_INT_ZHIE_ENABLE |  H3LIS331DL_INT_ZLIE_ENABLE |
										   H3LIS331DL_INT_YHIE_ENABLE |  H3LIS331DL_INT_YLIE_ENABLE |
											 H3LIS331DL_INT_XHIE_ENABLE |  H3LIS331DL_INT_XLIE_ENABLE ); 
 if(response==1){
   len = sprintf((char*)buffer,"SET_INT_CONF_OK \n\r");
   USB_SIL_Write(EP1_IN, buffer, len);
   SetEPTxValid(ENDP1);
 }
 //set Interrupt Mode
 response = H3LIS331DL_SetInt1Mode(0,  H3LIS331DL_INT_MODE_6D_POSITION);
 if(response==1){
   len = sprintf((char*)buffer,"SET_INT_MODE\n\r");
   USB_SIL_Write(EP1_IN, buffer, len);
   SetEPTxValid(ENDP1);
 }
 
 while(1) {
   //get 6D Position
   response = H3LIS331DL_Get6DPositionInt1(&position);
   if((response==1) && (old_position!=position)){
	 EKSTM32_LEDToggle(LED1);
	 switch (position){
	 case  H3LIS331DL_UP_SX:
	   len = sprintf((char*)buffer,"\n\rposition = UP_SX\n\r");   
	   break;
	 case  H3LIS331DL_UP_DX:
	   len = sprintf((char*)buffer,"\n\rposition = UP_DX\n\r");
	   break;
	 case  H3LIS331DL_DW_SX:
	   len = sprintf((char*)buffer,"\n\rposition = DW_SX\n\r");   
	   break;              
	 case  H3LIS331DL_DW_DX:   
	   len = sprintf((char*)buffer,"\n\rposition = DW_DX\n\r");   
	   break; 
	 case  H3LIS331DL_TOP:     
	   len = sprintf((char*)buffer,"\n\rposition = TOP\n\r");   
	   break; 
	 case  H3LIS331DL_BOTTOM:  
	   len = sprintf((char*)buffer,"\n\rposition = BOTTOM\n\r");   
	   break; 
	 default:      
	   len = sprintf((char*)buffer,"\n\rposition = unknown\n\r");   
	   break;	 
	 }
	 
	 //function for MKI109V1 board   
	 USB_SIL_Write(EP1_IN, buffer, len);
	 SetEPTxValid(ENDP1);  
	 old_position = position;
   }
 }
#endif /*__EXAMPLE2__H */ 
 
 
} // end main


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

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
