/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
* File Name          : example_Sensor_Hub_LSM303AGR.c
* Author             : MEMS Application Team
* Version            : v1.5
* Date               : 17 May 2016   
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
#include "spi_mems.h"
#include "interruptHandler.h"

#include "LSM303AGR_MAG_driver.h"
#include "LSM6DSL_ACC_GYRO_driver.h"

#define LSM303AGR_MAG_SENSITIVITY	1.5f

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
//#define MAX_PATTERN_NUM FIFO_THRESHOLD/6

int AngularRate_mdps[3];
int Acceleration_mG[3];
Type3Axis16bit_U magneticField;
float magneticFieldGauss[3];

status_t response;  

/* Macros for min/max.  */
/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* Test the sensorhub */
static void  Loop_Test_SensorHub(void)
{
  LSM6DSL_ACC_GYRO_SENS_HUB_END_t op_status;
  
  u8_t si_identity_mat[9] = {8,0,0,0,8,0,0,0,8};

  /* Program the six Soft Iron Matrix coefficients */
  LSM6DSL_ACC_GYRO_SH_init_SI_Matrix(0, si_identity_mat);

  /* program Sensor Hub to read from LSM303AGR Output register */
  LSM6DSL_ACC_GYRO_SH0_Program(0, LSM303AGR_MAG_I2C_ADDRESS | 0x01, LSM303AGR_MAG_OUTX_L_REG, 6);
  
  /* power on LSM6DSL xl */
  LSM6DSL_ACC_GYRO_W_ODR_XL (0, LSM6DSL_ACC_GYRO_ODR_XL_104Hz);
  
  /* power on LSM6DSL gyro */
  LSM6DSL_ACC_GYRO_W_ODR_G (0, LSM6DSL_ACC_GYRO_ODR_G_104Hz);
  
  while(1) {
    
    /* wait for sensor hub end_op event */
    LSM6DSL_ACC_GYRO_R_SENS_HUB_END(0, &op_status);
    if (op_status == LSM6DSL_ACC_GYRO_SENS_HUB_END_STILL_ONGOING)
      continue;

    /* Read LSM303AGR - MAG */
    LSM6DSL_ACC_GYRO_ReadReg(0, LSM6DSL_ACC_GYRO_SENSORHUB1_REG, magneticField.u8bit, 6);
    magneticFieldGauss[0] = magneticField.i16bit[0] * LSM303AGR_MAG_SENSITIVITY;
    magneticFieldGauss[1] = magneticField.i16bit[1] * LSM303AGR_MAG_SENSITIVITY;
    magneticFieldGauss[2] = magneticField.i16bit[2] * LSM303AGR_MAG_SENSITIVITY;
    /* Read LSM6DSL - ACC */
    LSM6DSL_ACC_Get_Acceleration(0, Acceleration_mG, 0);
    /* Read LSM6DSL - GYRO */
    LSM6DSL_ACC_Get_AngularRate(0, AngularRate_mdps, 0);
    
  }
}

/*******************************************************************************
* Function Name  : main.
* Description    : Simple LSM303AGR Example.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/

int main(void)
{ 
  u8 who_am_i;
  
  InitHardware();
  SPI_Mems_Init();
  
  /* Read WHO_AM_I  and check if device is really the LSM6DSL */
  who_am_i = 0x00;
  LSM6DSL_ACC_GYRO_R_WHO_AM_I(0, &who_am_i);
  if (who_am_i != LSM6DSL_ACC_GYRO_WHO_AM_I)
    while (1); /* manage here Identification error */ 
  
  /* Pull Up enable on LSM6DSL master interface */
  LSM6DSL_ACC_GYRO_W_PULL_UP_EN(0, LSM6DSL_ACC_GYRO_PULL_UP_EN_ENABLED);
  
  /* Read WHO_AM_I  and check if device is really the LSM303AGR */
  who_am_i = 0x00;
  LSM303AGR_MAG_R_WHO_AM_I(0, &who_am_i);
  if (who_am_i != LSM303AGR_MAG_WHO_AM_I)
    while (1); /* manage here Identification error */  
    
  /* Configure LSM303AGR */
  LSM303AGR_MAG_W_ODR(0, LSM303AGR_MAG_ODR_100Hz);
  LSM303AGR_MAG_W_MD(0, LSM303AGR_MAG_MD_CONTINUOS_MODE);

  LSM303AGR_MAG_ReadReg(0, 0x60, &who_am_i, 1);
  
  /* Test Sensorhub */
  Loop_Test_SensorHub();

} // end main


///*******************************************************************************
//* Function Name		: LSM303AGR_MAG_WriteReg
//* Description		: Generic Writing function. It must be fullfilled with either
//*					: I2C or SPI writing function
//* Input				: Register Address, ptr to buffer to be written,
//*                                 length of buffer
//* Output			: None
//* Return			: None
//*******************************************************************************/
//status_t LSM303AGR_MAG_WriteReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len)
//{ 
//  //[Example]
//  u16_t i;
//  for ( i = 0; i < len; i++)
//  {
//    LSM6DSL_ACC_GYRO_SH0_WriteByte(0, LSM303AGR_MAG_I2C_ADDRESS, Reg, Bufp[i]);
//  }
//  return MEMS_SUCCESS;                                             
//}
//
///*******************************************************************************
//* Function Name		: LSM303AGR_MAG_ReadReg
//* Description		: Generic Reading function. It must be fullfilled with either
//*					: I2C or SPI writing function
//* Input				: Register Address, ptr to buffer to be read,
//*                                 length of buffer
//* Output			: None
//* Return			: None
//*******************************************************************************/
//status_t LSM303AGR_MAG_ReadReg(void *handle, u8_t Reg, u8_t *Bufp, u16_t len)
//{
//  LSM6DSL_ACC_GYRO_SH0_ReadMem(handle, LSM303AGR_MAG_I2C_ADDRESS, Reg, Bufp, len, 1);
//  return MEMS_SUCCESS;                                             //[Example]
//}


/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
