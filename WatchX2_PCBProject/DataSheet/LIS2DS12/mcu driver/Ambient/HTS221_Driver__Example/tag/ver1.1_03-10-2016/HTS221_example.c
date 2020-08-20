/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
* File Name          : HTS221_example.c
* Author             : MEMS Application Team
* Version            : v1.1
* Date               : 3 October 2016  
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
#include "i2c_mems.h"
#include "HTS221_driver.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
HTS221_Error_et response;               // mems error variable
HTS221_Init_st HTS221_Init;             // mems config type
uint8_t WhoAmI;
HTS221_BitStatus_et drdy_humidity, drdy_temperature;
uint16_t humidity, humidity_rel;
int16_t temperature, temperature_C;
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

  InitHardware();
  I2C_MEMS_Init();
 
  response = HTS221_Get_DeviceID(0, &WhoAmI);
  if ( response == HTS221_ERROR ) while(1); //manage here comunication error

  if ( WhoAmI != HTS221_WHO_AM_I_VAL ) while(1); //manage here device not found
  
  
  HTS221_DeActivate(0);
  if( response == HTS221_ERROR ) while(1); //manage here comunication error
    
 //Initialize Sensor 
  HTS221_Init.avg_h = HTS221_AVGH_16;
  HTS221_Init.avg_t = HTS221_AVGT_8;
  HTS221_Init.odr = HTS221_ODR_7HZ;
  HTS221_Init.bdu_status = HTS221_ENABLE;
  HTS221_Init.heater_status = HTS221_DISABLE;
  HTS221_Init.irq_level = HTS221_HIGH_LVL;
  HTS221_Init.irq_output_type = HTS221_PUSHPULL;
  HTS221_Init.irq_enable = HTS221_DISABLE;
 response = HTS221_Set_InitConfig(0, &HTS221_Init);
 if ( response == HTS221_ERROR ) while(1); //manage here comunication error

 //Activate 
  response = HTS221_Activate(0);
  if ( response == HTS221_ERROR ) while(1); //manage here comunication error
 
  //main loop
  while(1)
  {
    
    //get temperature /humidity data status  
    response = HTS221_Get_DataStatus(0, &drdy_humidity, &drdy_temperature);
    if(response==HTS221_ERROR) while(1); //manage here comunication error
    
    if ( drdy_temperature == HTS221_SET)
    {
       //get temperature data
       response = HTS221_Get_Temperature(0, &temperature);
       if ( response == HTS221_ERROR ) while(1); //manage here comunication error
       temperature_C = temperature / 10;
    }
    
    if ( drdy_humidity == HTS221_SET)
    {
       //get humidity data
       response = HTS221_Get_Humidity(0, &humidity);
       if ( response == HTS221_ERROR ) while(1); //manage here comunication error
       humidity_rel = humidity / 10;
    }
      
  } // end main loop
  
} // end main


/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/