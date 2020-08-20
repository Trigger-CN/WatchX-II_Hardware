/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
* File Name          : main__LIS2DH12_example.c
* Author             : MEMS Application Team
* Version            : v2.2
* Date               : 18 April 2016  
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
//include files for LIS2DH12 adapter board
#include "stm32f10x.h"
#include "hw_config.h"
#include "spi_mems.h"


//include MEMS driver
#include "LIS2DH12_ACC_driver.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t response;
uint8_t USBbuffer[64];

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
  SPI_Mems_Init();
 
  //Inizialize MEMS Sensor
  //set ODR (turn ON device)
  LIS2DH12_SetODR(0, LIS2DH12_ODR_100Hz);
  //set PowerMode 
  LIS2DH12_SetMode(0, LIS2DH12_NORMAL);
  //set Fullscale
  LIS2DH12_SetFullScale(0, LIS2DH12_FULLSCALE_2);
    
/*********************/  
/******Example 1******/ 
#ifdef __EXAMPLE1__H 
  Type3Axis16bit_U data;
  while(1){
  //get Acceleration Raw data  
  LIS2DH12_GetAccAxesRaw(0, data.u8bit);
  /* sprintf((char*)buffer, "X=%6d Y=%6d Z=%6d \r\n", data.AXIS_X, data.AXIS_Y, data.AXIS_Z);*/
  }

#endif /* __EXAMPLE1__H  */ 
 
 
/*********************/
/******Example 2******/
#ifdef __EXAMPLE2__H
  uint8_t position=0, old_position=0;
 //configure Mems Sensor
 //set Interrupt Threshold 
 LIS2DH12_SetInt1Threshold(0, 20);

 //set Interrupt configuration (all enabled)
 LIS2DH12_SetInt1Configuration(0, LIS2DH12_INT1_ZHIE_ENABLE | LIS2DH12_INT1_ZLIE_ENABLE |
					         LIS2DH12_INT1_YHIE_ENABLE | LIS2DH12_INT1_YLIE_ENABLE |
					         LIS2DH12_INT1_XHIE_ENABLE | LIS2DH12_INT1_XLIE_ENABLE ); 

 //set Interrupt Mode
 LIS2DH12_SetInt1Mode(0, LIS2DH12_INT_MODE_6D_POSITION);

 while(1){

  //get 6D Position
  LIS2DH12_Get6DPosition(0, &position,INT1);
  if(old_position!=position){
    switch (position){
    case LIS2DH12_UP_SX: /* sprintf((char*)buffer,"\n\rposition = UP_SX  \n\r\0");  */  break;
    case LIS2DH12_UP_DX: /*  sprintf((char*)buffer,"\n\rposition = UP_DX  \n\r\0"); */  break;
    case LIS2DH12_DW_SX: /*  sprintf((char*)buffer,"\n\rposition = DW_SX  \n\r\0"); */  break;              
    case LIS2DH12_DW_DX: /*  sprintf((char*)buffer,"\n\rposition = DW_DX  \n\r\0"); */  break; 
    case LIS2DH12_TOP:   /*  sprintf((char*)buffer,"\n\rposition = TOP    \n\r\0"); */  break; 
    case LIS2DH12_BOTTOM:/* sprintf((char*)buffer,"\n\rposition = BOTTOM \n\r\0");  */  break; 
    default:     /* sprintf((char*)buffer,"\n\rposition = unknown\n\r\0"); */   break;
    }
  old_position = position;
  }
 }
#endif /*__EXAMPLE2__H */ 
 
} // end main


#ifdef USE_FULL_ASSERT


//function for MKI109V1 board 
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

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
