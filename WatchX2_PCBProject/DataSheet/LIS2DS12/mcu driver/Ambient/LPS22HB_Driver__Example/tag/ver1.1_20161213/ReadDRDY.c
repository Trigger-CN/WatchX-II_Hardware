/******************** (C) COPYRIGHT 2017 STMicroelectronics ********************
* File Name          : ReadDRDY.c
* Author             : MEMS Application Team
* Version            : v1.0
* Date               : 18 May 2017  
* Description        : EKSTM32 main file
*
********************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "..\..\Ambient\LPS22HB_Driver__Example\trunk\LPS22HB_Driver\LPS22HB_Driver.h"
#include "stm32f1xx_hal.h"
#include "usbd_cdc_if.h"
#include "spi.h"

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

static uint8_t WhoAmI;
static uint8_t USBbuffer[1000];
static LPS22HB_MeasureTypeDef_st Measurement_Value;

/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Test Acquisition of sensor samples */
int example_main(void)
{
  LPS22HB_InterruptTypeDef_st SensorInterruptConfig;

  LPS22HB_Get_DeviceID(0, &WhoAmI);
  
  SensorInterruptConfig.AutoRifP = LPS22HB_DISABLE;
  SensorInterruptConfig.AutoZero = LPS22HB_DISABLE;
  SensorInterruptConfig.FIFO_FTH = LPS22HB_DISABLE;
  SensorInterruptConfig.FIFO_FULL = LPS22HB_DISABLE;
  SensorInterruptConfig.FIFO_OVR  = LPS22HB_DISABLE;
  SensorInterruptConfig.LatchIRQ = LPS22HB_DISABLE;
  
  SensorInterruptConfig.DRDY             = LPS22HB_ENABLE;
  SensorInterruptConfig.PP_OD            = LPS22HB_PushPull;
  SensorInterruptConfig.OutputSignal_INT = LPS22HB_DATA;
  SensorInterruptConfig.INT_H_L          = LPS22HB_ActiveHigh;
  
  LPS22HB_Init(0);
  LPS22HB_Set_InterruptConfig(0,&SensorInterruptConfig);  
  
  while(1)
  {
    if (HAL_GPIO_ReadPin(INT1_GPIO_Port, INT1_Pin))
    {
      LPS22HB_Get_Measurement(0, &Measurement_Value);
      sprintf((char*)USBbuffer, "%6d\t%6d\r\n", Measurement_Value.Pout, Measurement_Value.Tout);
      CDC_Transmit_FS( USBbuffer, strlen((char const*)USBbuffer) );  
    }
  }
}

/* User defined Read / Write Functions */

int32_t Sensor_IO_Write(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len)
{
  /* Example */
  HAL_GPIO_WritePin(CS_DEV_GPIO_Port, CS_DEV_Pin, GPIO_PIN_RESET);  
  HAL_SPI_Transmit(&hspi2, &Reg, 1, 1000);
  HAL_SPI_Transmit(&hspi2, Bufp, len, 1000);
  HAL_GPIO_WritePin(CS_DEV_GPIO_Port, CS_DEV_Pin, GPIO_PIN_SET);    
  return 0;
}


int32_t Sensor_IO_Read(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len)
{
  /* Example */
  HAL_GPIO_WritePin(CS_DEV_GPIO_Port, CS_DEV_Pin, GPIO_PIN_RESET); 
  uint8_t dummy = Reg|0x80;
  HAL_SPI_Transmit(&hspi2, &dummy, 1, 1000);    
  HAL_SPI_Receive(&hspi2, Bufp, len, 1000);
  HAL_GPIO_WritePin(CS_DEV_GPIO_Port, CS_DEV_Pin, GPIO_PIN_SET); 
  return 0; 
}