/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
* File Name          : main__LSM6DS3_example.c
* Author             : MEMS Application Team
* Version            : v2.0
* Date               : 27 June 2016  
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
#include "interruptHandler.h"
#include "LSM6DS3_ACC_GYRO_driver.h"

/* This macro can be used to switch on/off the evaluation with interrupts */
#define TEST_WITH_INTERRUPT     1
#define TEST_FIFO_ENABLED     1

#define FIFO_THRESHOLD	3 * 2 * 512
#define FIFO_SIZE	FIFO_THRESHOLD

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
#define MAX_PATTERN_NUM FIFO_THRESHOLD/6

LSM6DS3_ACC_GYRO_GDA_t value_G;
int AngularRate_mdps[MAX_PATTERN_NUM][3];

LSM6DS3_ACC_GYRO_XLDA_t value_XL;
int Acceleration_mG[MAX_PATTERN_NUM][3];

Type3Axis16bit_U magneticField;
float magneticFieldGauss[3];

status_t response;  

/* Macros for min/max.  */
#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/


/*
 * ACC + Gyro test
 */
static   u16 pattern_len = 0;

/* 6ds3 Accelerometer test parameters */
static struct {
  u8_t				t_enable;
  LSM6DS3_ACC_GYRO_ODR_XL_t	t_odr;
  u16_t				t_odr_hz_val;
  LSM6DS3_ACC_GYRO_FS_XL_t	t_fs;
  u8_t                          t_decimation;
  u8_t                          t_samples_num_in_pattern;
} test_6ds3_xl = {
                  0,
                  LSM6DS3_ACC_GYRO_ODR_XL_416Hz,
                  0,
                  LSM6DS3_ACC_GYRO_FS_XL_8g,
                  0,
                  0,
                 };

/* 6ds3 Gyroscope test parameters */
static struct {
  u8_t				t_enable;
  LSM6DS3_ACC_GYRO_ODR_G_t	t_odr;
  u16_t				t_odr_hz_val;
  LSM6DS3_ACC_GYRO_FS_G_t	t_fs;
  u8_t                          t_decimation;
  u8_t                          t_samples_num_in_pattern;
} test_6ds3_gyro = {
                  0,
                  LSM6DS3_ACC_GYRO_ODR_G_104Hz,
                  0,
                  LSM6DS3_ACC_GYRO_FS_G_2000dps,
                  0,
                  0,
                };


#ifdef TEST_FIFO_ENABLED
/*
 * Following routine calculate the FIFO pattern composition based
 * on gyro and acc enable state and ODR freq.
 */
static u16 Calculate_FIFO_Pattern(u16_t *min_odr, u16_t *max_odr)
{
  u16 fifo_samples_tot_num = 0;

  /* calculate min_odr and max_odr for current configuration */
  *max_odr = 0;
  *min_odr = 0xffff;

  if (test_6ds3_gyro.t_enable) {
    LSM6DS3_ACC_GYRO_translate_ODR_G(test_6ds3_gyro.t_odr, &test_6ds3_gyro.t_odr_hz_val);
    *max_odr = MAX(*max_odr, test_6ds3_gyro.t_odr_hz_val);
    *min_odr = MIN(*min_odr, test_6ds3_gyro.t_odr_hz_val);
  }

  if (test_6ds3_xl.t_enable) {
    LSM6DS3_ACC_GYRO_translate_ODR_XL(test_6ds3_xl.t_odr, &test_6ds3_xl.t_odr_hz_val);
    *max_odr = MAX(*max_odr, test_6ds3_xl.t_odr_hz_val);
    *min_odr = MIN(*min_odr, test_6ds3_xl.t_odr_hz_val);
  }

  /* calculate how many samples for each sensor are in current FIFO pattern */

  if (test_6ds3_gyro.t_enable) {
    test_6ds3_gyro.t_samples_num_in_pattern = test_6ds3_gyro.t_odr_hz_val / *min_odr;
    test_6ds3_gyro.t_decimation =  *max_odr / test_6ds3_gyro.t_odr_hz_val;
    fifo_samples_tot_num += test_6ds3_gyro.t_samples_num_in_pattern;
  }

  if (test_6ds3_xl.t_enable) {
    test_6ds3_xl.t_samples_num_in_pattern = test_6ds3_xl.t_odr_hz_val / *min_odr;
    test_6ds3_xl.t_decimation =  *max_odr / test_6ds3_xl.t_odr_hz_val;
    fifo_samples_tot_num += test_6ds3_xl.t_samples_num_in_pattern;
  }

  /* return the total number of 16-bit samples in the pattern */
  return(6 * fifo_samples_tot_num);
}

/*
 * Following routine read a pattern from FIFO.
 */
static void Read_FIFO_Pattern(u8 pattern_idx)
{
  u8_t gy_i = 0;
  u8_t xl_i = 0;
  u8_t gy_num = test_6ds3_gyro.t_samples_num_in_pattern;
  u8 gy_idx = pattern_idx * gy_num;
  u8_t xl_num = test_6ds3_xl.t_samples_num_in_pattern;
  u8 xl_idx = pattern_idx * xl_num;

  /*
   * FIFO pattern is composed by gy_num gyroscope triplets and
   * xl_num accelerometer triplets. The sequence has always following order:
   * gyro first, accelerometer second.
   */
  while(gy_num > 0 || xl_num > 0) {
    /* read gyro samples */
    if (test_6ds3_gyro.t_enable && gy_num > 0) {
      LSM6DS3_ACC_Get_AngularRate(0, AngularRate_mdps[gy_idx + gy_i++], 1);
      gy_num--;
    }

    /* read XL samples */
    if (test_6ds3_xl.t_enable && xl_num > 0) {
      LSM6DS3_ACC_Get_Acceleration(0, Acceleration_mG[xl_idx + xl_i++], 1);
      xl_num--;
    }
  }
}

/*
 * Callback to handle the XL and Gyro event.
 * It must be registered to be called at interrupt time.
 *
 * Samples acquisition is triggered by FIFO threshold event.
 */
void LSM6DS3_ACC_GYRO_sample_Callback_fifo(u8_t intID)
{
  LSM6DS3_ACC_GYRO_FIFO_FULL_t full_ev;
  u16_t num = 0;

  LSM6DS3_ACC_GYRO_R_FIFOFull(0, &full_ev);
  if (LSM6DS3_ACC_GYRO_FIFO_FULL_FIFO_FULL == full_ev) {

    u16_t num_pattern = 0;
    u8 i = 0;

    LSM6DS3_ACC_GYRO_R_FIFONumOfEntries(0, &num);
    num_pattern = num/pattern_len;

    while (num_pattern-- > 0)
      Read_FIFO_Pattern(i++);
  }
}
#else
/*
 * Callback to handle the XL and Gyro event.
 * It must be registered to be called at interrupt time.
 *
 * Samples acquisition is triggered by XL DRDY event.
 */
void LSM6DS3_ACC_GYRO_sample_Callback(u8_t intID)
{
  LSM6DS3_ACC_GYRO_XLDA_t event_XL;
  LSM6DS3_ACC_GYRO_GDA_t event_G;

  /*
   * Read ACC output only if new ACC value is available
   */
  LSM6DS3_ACC_GYRO_R_XLDA(0,&event_XL);
  if (LSM6DS3_ACC_GYRO_XLDA_DATA_AVAIL==event_XL) {
      LSM6DS3_ACC_Get_Acceleration(0, Acceleration_mG[0], 0);
  }

  /*
   * Read ACC output only if new ACC value is available
   */
  LSM6DS3_ACC_GYRO_R_GDA(0,&event_G);
  if (LSM6DS3_ACC_GYRO_GDA_DATA_AVAIL==event_G) {
    LSM6DS3_ACC_Get_AngularRate(0, AngularRate_mdps[0], 0);
  }
}
#endif

/* Init the Gyroscope */
static void init_LSM6DS3_GYRO(void)
{
  /* Gyro ODR and full scale */
  response = LSM6DS3_ACC_GYRO_W_ODR_G(0, test_6ds3_gyro.t_odr);
  if(response==MEMS_ERROR) while(1); //manage here comunication error

  response = LSM6DS3_ACC_GYRO_W_FS_G(0, test_6ds3_gyro.t_fs);
  if(response==MEMS_ERROR) while(1); //manage here comunication error

  /* Enable gyro */
  test_6ds3_gyro.t_enable = 1;
}

/* Init the Accelerometer */
static void init_LSM6DS3_ACC(void)
{
  /* Set ACC ODR  */
  response = LSM6DS3_ACC_GYRO_W_ODR_XL(0, test_6ds3_xl.t_odr);
  if(response==MEMS_ERROR) while(1); //manage here comunication error
  
  /* Set ACC full scale */
  response = LSM6DS3_ACC_GYRO_W_FS_XL(0, test_6ds3_xl.t_fs);
  if(response==MEMS_ERROR) while(1); //manage here comunication error

  /* BDU Enable */
  response = LSM6DS3_ACC_GYRO_W_BDU(0, LSM6DS3_ACC_GYRO_BDU_BLOCK_UPDATE);
  if(response==MEMS_ERROR) while(1); //manage here comunication error

  /* Enable Acc */
  test_6ds3_xl.t_enable = 1;
}

/* Test Acquisition of sensor samples */
static  void Loop_Test_Sample_Aquisition(void)
{
  /* configure device */
  init_LSM6DS3_GYRO();
  init_LSM6DS3_ACC();

#ifdef TEST_WITH_INTERRUPT

#ifdef TEST_FIFO_ENABLED
  u16 min_odr, max_odr;

  /* Config FIFO interrupt callback */
  RegisterInterrupt(LSM6DS3_ACC_GYRO_sample_Callback_fifo);

  /* Calculate FIFO pattern composition */
  pattern_len = Calculate_FIFO_Pattern(&min_odr, &max_odr);

  /* Set FIFO threshold to a multiple of pattern length */
  LSM6DS3_ACC_GYRO_W_FIFO_Watermark(0, FIFO_THRESHOLD);

  /* Force FIFO to stop on programmed threshold. */
  LSM6DS3_ACC_GYRO_W_STOP_ON_FTH(0, LSM6DS3_ACC_GYRO_STOP_ON_FTH_ENABLED);

  /* Route FIFO FULL event on INT1 */
  LSM6DS3_ACC_GYRO_W_FSS5_on_INT1(0, LSM6DS3_ACC_GYRO_INT1_FSS5_ENABLED);

  /*
   * Set FIFO ODR to high frequency. The device will use the maximum ODR among
   * sensors.
   */
  LSM6DS3_ACC_GYRO_W_ODR_FIFO(0, LSM6DS3_ACC_GYRO_ODR_FIFO_800Hz);

  /* Set FIFO in continuos mode */
  LSM6DS3_ACC_GYRO_W_FIFO_MODE(0, LSM6DS3_ACC_GYRO_FIFO_MODE_DYN_STREAM_2);

  /* Set FIFO Decimation according to pattern composition */
  LSM6DS3_ACC_GYRO_W_DEC_FIFO_XL_val(0, test_6ds3_xl.t_decimation);
  LSM6DS3_ACC_GYRO_W_DEC_FIFO_G_val(0, test_6ds3_gyro.t_decimation);

#else
  /* Config XL interrupt (Gyro samples will be acquired @ XL ODR) */
  RegisterInterrupt(LSM6DS3_ACC_GYRO_sample_Callback);

  /* Route XL DRDY event on INT1 */
  LSM6DS3_ACC_GYRO_W_DRDY_XL_on_INT1(0, LSM6DS3_ACC_GYRO_INT1_DRDY_XL_ENABLED);
#endif

  while(1) {
    /* Event will be handled in driver callback */
  }
#else
  /*
   * Read samples in polling mode (no int)
   */
  while(1)
  {
    /*
     * Read ACC output only if new ACC value is available
     */
    response =  LSM6DS3_ACC_GYRO_R_XLDA(0, &value_XL);
    if(response==MEMS_ERROR) while(1); //manage here comunication error  
    
    if (LSM6DS3_ACC_GYRO_XLDA_DATA_AVAIL==value_XL)
    {
      LSM6DS3_ACC_Get_Acceleration(0, Acceleration_mG[0], 0);
    }

    /* 
     * Read GYRO output only if new gyro value is available
     */
    response =  LSM6DS3_ACC_GYRO_R_GDA(0, &value_G);
    if(response==MEMS_ERROR) while(1); //manage here comunication error  
    
    if (LSM6DS3_ACC_GYRO_GDA_DATA_AVAIL==value_G)
    {
      LSM6DS3_ACC_Get_AngularRate(0, AngularRate_mdps[0], 0);
    }
  }
#endif
}

/*
 * PEDOMETER test
 */
u16_t Number_Of_Steps = 0;

/*
 * Callback to handle the Pedometer event.
 * It must be registered to be called at interrupt time.
 */
void LSM6DS3_ACC_GYRO_Pedo_Callback(u8_t intID)
{
  LSM6DS3_ACC_GYRO_PEDO_EV_STATUS_t PedoStatus;

  LSM6DS3_ACC_GYRO_R_PEDO_EV_STATUS(0, &PedoStatus);
  if (PedoStatus == LSM6DS3_ACC_GYRO_PEDO_EV_STATUS_DETECTED) {
    LSM6DS3_ACC_GYRO_Get_GetStepCounter(0, (u8_t *)&Number_Of_Steps);
  }
}

/* Init the Pedometer */
static void init_LSM6DS3_Pedometer(void)
{
  /* Set ACC ODR  */
  response = LSM6DS3_ACC_GYRO_W_ODR_XL(0, LSM6DS3_ACC_GYRO_ODR_XL_26Hz);
  if(response==MEMS_ERROR) while(1); //manage here comunication error

  /* Enable Pedometer  */
  response = LSM6DS3_ACC_GYRO_W_PEDO_EN(0, LSM6DS3_ACC_GYRO_PEDO_EN_ENABLED);
  if(response==MEMS_ERROR) while(1); //manage here comunication error

  /* Pedometer on INT1 */
  response = LSM6DS3_ACC_GYRO_W_PEDO_STEP_on_INT1(0, LSM6DS3_ACC_GYRO_INT1_PEDO_ENABLED);
  if(response==MEMS_ERROR) while(1); //manage here comunication error

  /* Enable Embedded Functions */
  LSM6DS3_ACC_GYRO_W_FUNC_EN(0, LSM6DS3_ACC_GYRO_FUNC_EN_ENABLED);
}

/* Test pedometer */
static void  Loop_Test_Pedometer(void)
{
  /* configure pedometer */
  init_LSM6DS3_Pedometer();

  /* configure pedometer acceleration threshold */
  LSM6DS3_ACC_GYRO_W_PedoThreshold(0, 13);

  /* Clear the step counter */
  LSM6DS3_ACC_GYRO_W_PedoStepReset(0, LSM6DS3_ACC_GYRO_PEDO_RST_STEP_ENABLED);

#ifdef TEST_WITH_INTERRUPT
  RegisterInterrupt(LSM6DS3_ACC_GYRO_Pedo_Callback);

  while(1) {
    /* Event will be handled in driver callback */
  }
#else
  LSM6DS3_ACC_GYRO_PEDO_EV_STATUS_t PedoStatus;

  /*
   * Handle the event using polling mode
   */
  while(1) {
    LSM6DS3_ACC_GYRO_R_PEDO_EV_STATUS(0, &PedoStatus);
    if (PedoStatus == LSM6DS3_ACC_GYRO_PEDO_EV_STATUS_DETECTED) {
      LSM6DS3_ACC_GYRO_Get_GetStepCounter(0, (u8_t *)&Number_Of_Steps);
    }
  }
#endif
}

/*
 * Wakeup test
 */
u16_t Number_Of_Wakeup_events = 0;

/*
 * Callback to handle the Wakeup event.
 * It must be registered to be called at interrupt time.
 */
void LSM6DS3_ACC_GYRO_Wakeup_Callback(u8_t intID)
{
  LSM6DS3_ACC_GYRO_WU_EV_STATUS_t WakeupStatus;

  LSM6DS3_ACC_GYRO_R_WU_EV_STATUS(0, &WakeupStatus);
  if (WakeupStatus == LSM6DS3_ACC_GYRO_WU_EV_STATUS_DETECTED) {
    Number_Of_Wakeup_events++;

    /* handle event */
    WakeupStatus = LSM6DS3_ACC_GYRO_WU_EV_STATUS_NOT_DETECTED;
  }
}

/* Init the Wakeup */
static void init_LSM6DS3_Wakeup(void)
{
  /* Set ACC ODR  */
  response = LSM6DS3_ACC_GYRO_W_ODR_XL(0, LSM6DS3_ACC_GYRO_ODR_XL_104Hz);
  if(response==MEMS_ERROR) while(1); //manage here comunication error

  /* Set Wakeup threshold  */
  response = LSM6DS3_ACC_GYRO_W_WK_THS(0, 2);
  if(response==MEMS_ERROR) while(1); //manage here comunication error

  /* Set Wakeup duration  */
  response = LSM6DS3_ACC_GYRO_W_SLEEP_DUR(0, 0);
  if(response==MEMS_ERROR) while(1); //manage here comunication error

  /* Wakeup on INT1 */
  response = LSM6DS3_ACC_GYRO_W_WUEvOnInt1(0, LSM6DS3_ACC_GYRO_INT1_WU_ENABLED);
  if(response==MEMS_ERROR) while(1); //manage here comunication error
}

/* Test Wakeup */
static void  Loop_Test_Wakeup(void)
{
  Number_Of_Wakeup_events = 0;

  /* configure pedometer */
  init_LSM6DS3_Wakeup();

#ifdef TEST_WITH_INTERRUPT
  RegisterInterrupt(LSM6DS3_ACC_GYRO_Wakeup_Callback);

  while(1) {
    /* Event will be handled in driver callback */
  }
#else
  LSM6DS3_ACC_GYRO_WU_EV_STATUS_t WakeupStatus;

  /*
   * Handle the event using polling mode
   */
  while(1) {
    LSM6DS3_ACC_GYRO_R_WU_EV_STATUS(0, &WakeupStatus);
    if (WakeupStatus == LSM6DS3_ACC_GYRO_WU_EV_STATUS_DETECTED) {
      Number_Of_Wakeup_events++;

      /* handle event */
      WakeupStatus = LSM6DS3_ACC_GYRO_WU_EV_STATUS_NOT_DETECTED;
    }
  }
#endif
}

/*
 * Tilt test
 */
u16_t Number_Of_Tilt_events = 0;

/*
 * Callback to handle the Tilt event.
 * It must be registered to be called at interrupt time.
 */
void LSM6DS3_ACC_GYRO_Tilt_Callback(u8_t intID)
{
  LSM6DS3_ACC_GYRO_TILT_EV_STATUS_t TiltStatus;

  LSM6DS3_ACC_GYRO_R_TILT_EV_STATUS(0, &TiltStatus);
  if (TiltStatus == LSM6DS3_ACC_GYRO_TILT_EV_STATUS_DETECTED) {
    Number_Of_Tilt_events++;

    /* handle event */
    TiltStatus = LSM6DS3_ACC_GYRO_TILT_EV_STATUS_NOT_DETECTED;
  }
}

/* Init the Tilt */
static void init_LSM6DS3_Tilt(void)
{
  /* Set ACC ODR  */
  response = LSM6DS3_ACC_GYRO_W_ODR_XL(0, LSM6DS3_ACC_GYRO_ODR_XL_26Hz);
  if(response==MEMS_ERROR) while(1); //manage here comunication error

  /* Enable Tilt  */
  response = LSM6DS3_ACC_GYRO_W_TILT_EN(0, LSM6DS3_ACC_GYRO_TILT_EN_ENABLED);
  if(response==MEMS_ERROR) while(1); //manage here comunication error

  /* Tilt on INT1 */
  response = LSM6DS3_ACC_GYRO_W_TiltEvOnInt1(0, LSM6DS3_ACC_GYRO_INT1_TILT_ENABLED);
  if(response==MEMS_ERROR) while(1); //manage here comunication error

  /* Enable Embedded Functions */
  LSM6DS3_ACC_GYRO_W_FUNC_EN(0, LSM6DS3_ACC_GYRO_FUNC_EN_ENABLED);
}

/* Test Tilt */
static void  Loop_Test_Tilt(void)
{
  Number_Of_Tilt_events = 0;

  /* configure Tilt */
  init_LSM6DS3_Tilt();
  
#ifdef TEST_WITH_INTERRUPT
  RegisterInterrupt(LSM6DS3_ACC_GYRO_Tilt_Callback);

  while(1) {
    /* Event will be handled in driver callback */
  }
#else
  LSM6DS3_ACC_GYRO_TILT_EV_STATUS_t TiltStatus;

  /*
   * Handle the event using polling mode
   */
  while(1) {
    LSM6DS3_ACC_GYRO_R_TILT_EV_STATUS(0, &TiltStatus);
    if (TiltStatus == LSM6DS3_ACC_GYRO_TILT_EV_STATUS_DETECTED) {
      Number_Of_Tilt_events++;

      /* handle event */
      TiltStatus = LSM6DS3_ACC_GYRO_TILT_EV_STATUS_NOT_DETECTED;
    }
  }
#endif
}

/*
 * Significant Motion test
 */
u16_t Number_Of_SigMot_events = 0;

/*
 * Callback to handle the SigMotion event.
 * It must be registered to be called at interrupt time.
 */
void LSM6DS3_ACC_GYRO_SigMotion_Callback(u8_t intID)
{
  LSM6DS3_ACC_GYRO_SIGN_MOT_EV_STATUS_t SigMotStatus;

  LSM6DS3_ACC_GYRO_R_SIGN_MOT_EV_STATUS(0, &SigMotStatus);
  if (SigMotStatus == LSM6DS3_ACC_GYRO_SIGN_MOT_EV_STATUS_DETECTED) {
    Number_Of_SigMot_events++;

    /* handle event */
    SigMotStatus = LSM6DS3_ACC_GYRO_SIGN_MOT_EV_STATUS_NOT_DETECTED;
  }
}

/*
 * DoubleTap test
 */

static u32_t DoubleTapCount = 0;

/*
 * Callback to handle the Double Tap event.
 * It must be registered to be called at interrupt time.
 */
void LSM6DS3_ACC_GYRO_DoubleTap_Callback(u8_t intID)
{
  LSM6DS3_ACC_GYRO_DOUBLE_TAP_EV_STATUS_t DoubleTapStatus;

  LSM6DS3_ACC_GYRO_R_DOUBLE_TAP_EV_STATUS(0, &DoubleTapStatus);
  if (DoubleTapStatus == LSM6DS3_ACC_GYRO_DOUBLE_TAP_EV_STATUS_DETECTED) {
    /* handle event */
    DoubleTapCount++;
    DoubleTapStatus = LSM6DS3_ACC_GYRO_DOUBLE_TAP_EV_STATUS_NOT_DETECTED;
  }
}

/* Init the Double Tap */
static void init_LSM6DS3_DoubleTap(void)
{
  /* Set ACC ODR  */
  response = LSM6DS3_ACC_GYRO_W_ODR_XL(0, LSM6DS3_ACC_GYRO_ODR_XL_416Hz);
  if(response==MEMS_ERROR) while(1); //manage here comunication error

  /* Enable TAP x/y/z */
  LSM6DS3_ACC_GYRO_W_TAP_X_EN(0, LSM6DS3_ACC_GYRO_TAP_X_EN_ENABLED);
  LSM6DS3_ACC_GYRO_W_TAP_Y_EN(0, LSM6DS3_ACC_GYRO_TAP_Y_EN_ENABLED);
  LSM6DS3_ACC_GYRO_W_TAP_Z_EN(0, LSM6DS3_ACC_GYRO_TAP_Z_EN_ENABLED);

  /* Set the TAP threshold value */
  LSM6DS3_ACC_GYRO_W_TAP_THS(0, 12);

  /* Route TAP ev on INT1 */
  LSM6DS3_ACC_GYRO_W_TapEvOnInt1(0, LSM6DS3_ACC_GYRO_INT1_TAP_ENABLED);

  /* Set interrupt durations */
  LSM6DS3_ACC_GYRO_W_QUIET_Duration(0, 3);
  LSM6DS3_ACC_GYRO_W_SHOCK_Duration(0, 3);
  LSM6DS3_ACC_GYRO_W_DUR(0, 7);

  /* Enable Single/Double Tap */
  LSM6DS3_ACC_GYRO_W_SINGLE_DOUBLE_TAP_EV(0, LSM6DS3_ACC_GYRO_SINGLE_DOUBLE_TAP_SINGLE_TAP);

  /* Enable Embedded Functions */
  LSM6DS3_ACC_GYRO_W_FUNC_EN(0, LSM6DS3_ACC_GYRO_FUNC_EN_ENABLED);
}

/* Test Double Tap */
static void  Loop_Test_DoubleTap(void)
{
  /* configure Double Tap */
  init_LSM6DS3_DoubleTap();
  DoubleTapCount = 0;

#ifdef TEST_WITH_INTERRUPT
  RegisterInterrupt(LSM6DS3_ACC_GYRO_DoubleTap_Callback);

  while(1) {
    /* Event will be handled in driver callback */
  }
#else
  LSM6DS3_ACC_GYRO_DOUBLE_TAP_EV_STATUS_t DoubleTapStatus;

  /*
   * Handle the event using polling mode
   */
  while(1) {
    LSM6DS3_ACC_GYRO_R_DOUBLE_TAP_EV_STATUS(0, &DoubleTapStatus);
    if (DoubleTapStatus == LSM6DS3_ACC_GYRO_DOUBLE_TAP_EV_STATUS_DETECTED) {
      /* handle event */
      DoubleTapCount++;
      DoubleTapStatus = LSM6DS3_ACC_GYRO_DOUBLE_TAP_EV_STATUS_NOT_DETECTED;
    }
  }
#endif
}

/*
 * This test has been performed attaching a LIS3MDL magnetometer
 * to the I2C master bus provided by LSM6DS3 device.
 *
 * In this particular example the I2C master operations are triggered through
 * the internal Accelerometer dataready.
 */

/*
 * On this particular solution the SDO bit is set to '0', so the
 * LIS3MDL slave address is 0x38
 */
#define LIS3MDL_SLAVE_ADDR    0x38
#define LIS3MDL_WHO_AM_I_VAL  0x3D

#define LIS3MDL_WHO_AM_I      0x0F
#define LIS3MDL_CTRL_REG1     0x20
#define LIS3MDL_CTRL_REG2     0x21
#define LIS3MDL_CTRL_REG3     0x22
#define LIS3MDL_CTRL_REG4     0x23
#define LIS3MDL_CTRL_REG5     0x24

#define OUT_X_L               0x28
#define OUT_X_H               0x29
#define OUT_Y_L               0x2A
#define OUT_Y_H               0x2B
#define OUT_Z_L               0x2C
#define OUT_Z_H               0x2D

#define SENSITIVITY_LIS3MDL_16Ga     1711.0              //LSB/Ga

/* Init the sensorhub using I2C PassThrough mode */
static void init_LSM6DS3_PT_SensorHub(void)
{
  u8 who_am_i = 0x0, Data;

  /* Enable ACC x/y/x axis (for triggering the operation) */
  LSM6DS3_ACC_GYRO_W_ODR_XL(0, LSM6DS3_ACC_GYRO_ODR_XL_208Hz);

  /* Enable Passthru mode */
  LSM6DS3_ACC_GYRO_W_PASS_THRU_MODE(0, LSM6DS3_ACC_GYRO_PASS_THRU_MODE_ENABLED);

  /* read LIS3MDL who_am_i (0xF) */
  I2Cx_Read(&who_am_i, LIS3MDL_SLAVE_ADDR, LIS3MDL_WHO_AM_I, 1);
  if (who_am_i != LIS3MDL_WHO_AM_I_VAL)
    while(1); //manage here comunication error

  /* write 0x7C in LIS3MDL ctrl_reg1 */
  /* ODR 80Hz - XY at ultra-high perf  */
  Data = 0x7C;
  I2Cx_Write(&Data, LIS3MDL_SLAVE_ADDR,  LIS3MDL_CTRL_REG1, 1);

  /* write 0x60 in LIS3MDL ctrl_reg2 */
  /* Set FS to 16 Gauss  */
  Data = 0x60;
  I2Cx_Write(&Data, LIS3MDL_SLAVE_ADDR,  LIS3MDL_CTRL_REG2, 1);

  /* write 0x0C in LIS3MDL ctrl_reg4 */
  /* Set Z at ultra-high perf */
  Data = 0x0C;
  I2Cx_Write(&Data, LIS3MDL_SLAVE_ADDR,  LIS3MDL_CTRL_REG4, 1);

  /* write 0x00 in LIS3MDL ctrl_reg3 */
  /* continuos conversion mode */
  Data = 0x00;
  I2Cx_Write(&Data, LIS3MDL_SLAVE_ADDR,  LIS3MDL_CTRL_REG3, 1);

  /* write 0x40 in LIS3MDL ctrl_reg5 */
  /* set BDU */
  Data = 0x40;
  I2Cx_Write(&Data, LIS3MDL_SLAVE_ADDR,  LIS3MDL_CTRL_REG5, 1);

  /* Disable Passthru mode */
  LSM6DS3_ACC_GYRO_W_PASS_THRU_MODE(0, LSM6DS3_ACC_GYRO_PASS_THRU_MODE_DISABLED);
}

/* Test the sensorhub */
static void  Loop_Test_SensorHub(void)
{
  u8_t si_identity_mat[9] = {8,0,0,0,8,0,0,0,8};

  /* configure external sensor */
  init_LSM6DS3_PT_SensorHub();

  /* Program the six Soft Iron Matrix coefficients */
  LSM6DS3_ACC_GYRO_SH_init_SI_Matrix(0, si_identity_mat);

  /* program to .... */
  LSM6DS3_ACC_GYRO_SH0_Program(0, LIS3MDL_SLAVE_ADDR, OUT_X_L, 6);

  while(1) {
    LSM6DS3_ACC_GYRO_SENS_HUB_END_t op_status;

    /* wait for sensor hub end_op event */
    LSM6DS3_ACC_GYRO_R_SENS_HUB_END(0, &op_status);
    if (op_status == LSM6DS3_ACC_GYRO_SENS_HUB_END_STILL_ONGOING)
      continue;

    /* Read the result */
    LSM6DS3_ACC_GYRO_ReadReg(0, LSM6DS3_ACC_GYRO_SENSORHUB1_REG, magneticField.u8bit, 6);

    magneticFieldGauss[0]=magneticField.i16bit[0]*1000/SENSITIVITY_LIS3MDL_16Ga;
    magneticFieldGauss[1]=magneticField.i16bit[1]*1000/SENSITIVITY_LIS3MDL_16Ga;
    magneticFieldGauss[2]=magneticField.i16bit[2]*1000/SENSITIVITY_LIS3MDL_16Ga;
  }
}

/*******************************************************************************
* Function Name  : main.
* Description    : Simple LIS3MDL Example.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int main(void)
{ 
  u8 who_am_i = 0x0;

  InitHardware();
  I2C_MEMS_Init();

  /* Read WHO_AM_I  and check if device is really the LSM6DS3 */
  LSM6DS3_ACC_GYRO_R_WHO_AM_I(0, &who_am_i);
  if (who_am_i != LSM6DS3_ACC_GYRO_WHO_AM_I)
    while(1); //manage here comunication error

  /* Soft Reset the LSM6DS3 device */
  LSM6DS3_ACC_GYRO_W_SW_RESET(0, LSM6DS3_ACC_GYRO_SW_RESET_RESET_DEVICE);

  /*
   * Test routines.
   * Uncomment the one you need to exec.
   */

  /* Test sensor samples acquisition */
  Loop_Test_Sample_Aquisition();

  /* Test Pedometer */
  //Loop_Test_Pedometer();
  
  /* Test Wakup */
  //Loop_Test_Wakeup();

  /* Test Tilt */
  //Loop_Test_Tilt();

  /* Test Double Tap */
  //Loop_Test_DoubleTap();

  /* Test Sensorhub */
  //Loop_Test_SensorHub();

} // end main


/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
