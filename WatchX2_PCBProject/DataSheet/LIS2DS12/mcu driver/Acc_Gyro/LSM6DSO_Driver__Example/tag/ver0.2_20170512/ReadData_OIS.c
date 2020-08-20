/******************** (C) COPYRIGHT 2017 STMicroelectronics ********************
* File Name          : ReadData_OIS.c
* Author             : MEMS Application Team
* Version            : v1.0
* Date               : 17 May 2017  
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
#include "..\..\Acc_Gyro\LSM6DSO_Driver__Example\trunk\LSM6DSL_Driver\lsm6dso_reg.h"
#include "stm32f1xx_hal.h"
#include "usbd_cdc_if.h"
#include "spi.h"

/* Private macro -------------------------------------------------------------*/
#define SENSITIVITY_2g         (0.061) /* mg */
#define SENSITIVITY_2000dps    (70)    /* mdps */
/* Private variables ---------------------------------------------------------*/
static axis3bit16_t Acceleration, AngularRate;
static uint8_t WhoAmI;
static int32_t Acceleration_mg[3], AngularRate_mdps[3];
static uint8_t USBbuffer[1000];
static lsm6dso_status_reg_t status_reg;

/* define custom structure in order to manage UI and OIS interface */
typedef struct {
  SPI_HandleTypeDef *handle ; 
  GPIO_TypeDef* cs_port;
  uint32_t cs_pin ;
} spi_interface_t;

/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/* User defined Read / Write Functions */
int32_t spi_write(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len);
int32_t spi_read(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len);

/* Test Acquisition of sensor samples */
int example_main(void)
{
  spi_interface_t spi_interface_ui, spi_interface_ois;
  lsm6dso_ctx_t   lsm6dso_ui_ctx, lsm6dso_ois_ctx;
  
  /* Define OIS interface */
  spi_interface_ois.handle  = &hspi1;
  spi_interface_ois.cs_port = CS_RF_GPIO_Port;
  spi_interface_ois.cs_pin  = CS_RF_Pin;
  
  lsm6dso_ois_ctx.write_reg = spi_write;
  lsm6dso_ois_ctx.read_reg = spi_read; 
  lsm6dso_ois_ctx.handle = (void*) &spi_interface_ois;
  
  /* Define UI interface */
  spi_interface_ui.handle  = &hspi2;
  spi_interface_ui.cs_port = CS_DEV_GPIO_Port;
  spi_interface_ui.cs_pin  = CS_DEV_Pin;
  
  lsm6dso_ui_ctx.write_reg = spi_write;
  lsm6dso_ui_ctx.read_reg = spi_read; 
  lsm6dso_ui_ctx.handle = (void*) &spi_interface_ui;
      
  /* Check device ID */  
  lsm6dso_read_reg(&lsm6dso_ui_ctx, LSM6DSO_WHO_AM_I, &WhoAmI, 1);
  if (WhoAmI != LSM6DSO_ID)
      while(1);
  
  /* UI side configuration */
  /* Set Full Scale */
  lsm6dso_xl_full_scale_set(&lsm6dso_ui_ctx, _2g);
  /* Enable Acc Block Data Update */
  lsm6dso_block_data_update_set(&lsm6dso_ui_ctx, PROPERTY_ENABLE);
  /* Set Acc Output Data Rate */
  lsm6dso_xl_odr_set(&lsm6dso_ui_ctx, ODR_XL__52Hz);
  
  /*OIS side configuration */
  lsm6dso_ctrl1_ois_t lsm6dso_ctrl1_ois;
  lsm6dso_ctrl1_ois.ois_en_spi2 = PROPERTY_ENABLE;
  lsm6dso_ctrl1_ois.fs_g_ois    = OIS_2000dps;
  lsm6dso_ctrl1_ois.ois_xl_en   = PROPERTY_DISABLE;
  lsm6dso_ctrl1_ois.lvl_ois     = PROPERTY_DISABLE;
  lsm6dso_ctrl1_ois.sim_ois     = AUX_4_WIRE;
  lsm6dso_write_reg(&lsm6dso_ois_ctx, LSM6DSO_CTRL1_OIS, (uint8_t*) (&lsm6dso_ctrl1_ois), 1);

  /*
   * Read samples in polling mode (on UI DRDY)
   */
  while(1)
  {
    /* Read output only if new value is available */
    lsm6dso_status_get(&lsm6dso_ui_ctx, &status_reg);
    
    if (status_reg.xlda) 
    {
      lsm6dso_acceleration_raw(&lsm6dso_ui_ctx, Acceleration.u8bit);

      /* Transorm LSB into mg @ 14 bit resolution*/
      Acceleration_mg[0] = (int32_t)((Acceleration.i16bit[0])*SENSITIVITY_2g);
      Acceleration_mg[1] = (int32_t)((Acceleration.i16bit[1])*SENSITIVITY_2g);
      Acceleration_mg[2] = (int32_t)((Acceleration.i16bit[2])*SENSITIVITY_2g);
      
      sprintf((char*)USBbuffer, "%6d\t%6d\t%6d\t", Acceleration_mg[0], Acceleration_mg[1], Acceleration_mg[2]);
      CDC_Transmit_FS( USBbuffer, strlen((char const*)USBbuffer) );
      
      lsm6dso_angular_rate_raw(&lsm6dso_ois_ctx, AngularRate.u8bit);
      
      AngularRate_mdps[0] = (int32_t)((AngularRate.i16bit[0])*SENSITIVITY_2000dps);
      AngularRate_mdps[1] = (int32_t)((AngularRate.i16bit[1])*SENSITIVITY_2000dps);
      AngularRate_mdps[2] = (int32_t)((AngularRate.i16bit[2])*SENSITIVITY_2000dps);
      
      sprintf((char*)USBbuffer, "%6d\t%6d\t%6d\r\n", AngularRate_mdps[0], AngularRate_mdps[1], AngularRate_mdps[2]);
      CDC_Transmit_FS( USBbuffer, strlen((char const*)USBbuffer) );      
      
    }
  }  
}

/* User defined Read / Write Functions */

int32_t spi_write(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len)
{
  spi_interface_t* interface = (spi_interface_t*) handle;
  /* Example */
  HAL_GPIO_WritePin(interface->cs_port, interface->cs_pin, GPIO_PIN_RESET);  
  HAL_SPI_Transmit(interface->handle, &Reg, 1, 1000);
  HAL_SPI_Transmit(interface->handle, Bufp, len, 1000);
  HAL_GPIO_WritePin(interface->cs_port, interface->cs_pin, GPIO_PIN_SET);    
  return 0;
}

int32_t spi_read(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len)
{
  spi_interface_t* interface = (spi_interface_t*) handle;    
  /* Example */
  HAL_GPIO_WritePin(interface->cs_port, interface->cs_pin, GPIO_PIN_RESET); 
  uint8_t dummy = Reg|0x80;
  HAL_SPI_Transmit(interface->handle, &dummy, 1, 1000);    
  HAL_SPI_Receive(interface->handle, Bufp, len, 1000);
  HAL_GPIO_WritePin(interface->cs_port, interface->cs_pin, GPIO_PIN_SET); 
  return 0; 
}