/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
* File Name          : main_LSM303AH_example.c
* Author             : MEMS Application Team
* Version            : v1.1
* Date               : 15 Mar 2016  
* Description        : LSM303AH source driver file
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
#include "LSM303AH_ACC_driver.h"
#include "LSM303AH_MAG_driver.h"

/* This macro can be used to switch on/off the evaluation with interrupts */
#define TEST_WITH_FTH_INTERRUPT     1

#define FIFO_THRESHOLD	250
#define FIFO_SIZE	256

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
int Acceleration_G[FIFO_SIZE][3];
int Magnetic_mGa[3];

status_t response;  

/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/*
 * This part is needed in case we need to bypass the device ROM 
 */

/*
 * Accelerometer + FIFO in various modes
 */

static u32_t LSM303AH_ACC_sample_calls = 0;

/*
 * Callback to handle the XL event.
 * It must be registered to be called at interrupt time.
 *
 * This is specific version for FIFO in Continuos mode
 */
static void LSM303AH_ACC_sample_Callback(u8_t intID)
{
  u16_t sample_num = 0, cnt = 0;

  LSM303AH_ACC_sample_calls++;

  LSM303AH_ACC_R_SamplesNum(0, &sample_num);

  /* Read all the data. */
  while (sample_num-- > 0)
      LSM303AH_ACC_Get_Acceleration(0, Acceleration_G[cnt++]);

  /*
   * Read MAG samples
   */
    LSM303AH_MAG_Get_Magnetic(0, Magnetic_mGa);
}

/*
 * Callback to handle the XL event.
 * It must be registered to be called at interrupt time.
 *
 * This is specific version for FIFO in BTS mode
 */
static void LSM303AH_ACC_sample_Callback_bts(u8_t intID)
{
  u16_t sample_num = 0, cnt = 0;
  LSM303AH_ACC_FTH_STATUS_t FTH_status;

  LSM303AH_ACC_sample_calls++;

  LSM303AH_ACC_R_FTH(0, &FTH_status);
  if (FTH_status == LSM303AH_ACC_FTH_EV_ON) {
    LSM303AH_ACC_R_SamplesNum(0, &sample_num);

    /* Read all the data. */
    while (sample_num-- > 0)
        LSM303AH_ACC_Get_Acceleration(0, Acceleration_G[cnt++]);
  }
}

/*
 * Callback to handle the XL event.
 * It must be registered to be called at interrupt time.
 *
 * This is specific version for FIFO Mode
 */
static void LSM303AH_ACC_sample_Callback_fifo(u8_t intID)
{
  u16_t sample_num = 0, cnt = 0;
  LSM303AH_ACC_FIFO_OVR_t status = LSM303AH_ACC_FIFO_OVR_EV_OFF;

  LSM303AH_ACC_sample_calls++;

  /* wait until overrun gets set */
  while (status == LSM303AH_ACC_FIFO_OVR_EV_OFF)
    LSM303AH_ACC_R_FIFO_OVR(0, &status);

  LSM303AH_ACC_R_SamplesNum(0, &sample_num);

  /* Read all the data. */
  while (sample_num-- > 0)
      LSM303AH_ACC_Get_Acceleration(0, Acceleration_G[cnt++]);
}

/*
 * Callback to handle the XL event.
 * It must be registered to be called at interrupt time.
 *
 * This is specific version for FIFO in stream-to-FIFO mode
 */
static void LSM303AH_ACC_sample_Callback_stf(u8_t intID)
{
  u16_t sample_num = 0, cnt = 0;
  LSM303AH_ACC_FTH_STATUS_t FTH_status;
  LSM303AH_ACC_WU_IA_DUP2_t WakeupStatus;
  LSM303AH_ACC_FIFO_OVR_t OVR_status = LSM303AH_ACC_FIFO_OVR_EV_OFF;

  LSM303AH_ACC_sample_calls++;

  /* If wakeup event is on, means that the FIFO is now in FIFO_mode state */
  LSM303AH_ACC_R_WU_IA_DUP2(0, &WakeupStatus);
  if (WakeupStatus == LSM303AH_ACC_WU_IA_DUP2_EV_ON) {
    /* wait until overrun gets set */
    while (OVR_status == LSM303AH_ACC_FIFO_OVR_EV_OFF)
      LSM303AH_ACC_R_FIFO_OVR(0, &OVR_status);

    LSM303AH_ACC_R_SamplesNum(0, &sample_num);

    while (sample_num-- > 0)
        LSM303AH_ACC_Get_Acceleration(0, Acceleration_G[cnt++]);

    return;
  }

  /* FIFO is still streaming. Read all the data. */
  LSM303AH_ACC_R_FTH(0, &FTH_status);
  if (FTH_status == LSM303AH_ACC_FTH_EV_ON) {
    LSM303AH_ACC_R_SamplesNum(0, &sample_num);

    while (sample_num-- > 0)
        LSM303AH_ACC_Get_Acceleration(0, Acceleration_G[cnt++]);
  }
}

/*
 * Configure the WakeUp event.
 */
static void SetupWakeUpEvent(u8_t thrsld, u8_t slp_dur, u8_t lir_on)
{
  LSM303AH_ACC_WU_IA_DUP2_t WakeupStatus;

  /* Clear WU_IA event*/
  LSM303AH_ACC_R_WU_IA_DUP2(0, &WakeupStatus);

  /* Set interrupt latched mode */
  LSM303AH_ACC_W_LIR(0, LSM303AH_ACC_LIR_ON);

  /* Set Wakeup threshold  */
  LSM303AH_ACC_W_WU_THS(0, thrsld);

  /* Set Wakeup duration  */
  LSM303AH_ACC_W_SleepDuration(0, slp_dur);

  /* Wakeup on INT1 */
  LSM303AH_ACC_W_INT1_WU(0, LSM303AH_ACC_INT1_WU_ON);
}

/*
 * Configure the Tap event.
 */
static void SetupTapEvent(u8_t thrsld, u8_t int_dur, u8_t shock, u8_t lir_on)
{
  LSM303AH_ACC_TAP_IA_t TapStatus;

  /* Set interrupt latched mode */
  LSM303AH_ACC_W_LIR(0, LSM303AH_ACC_LIR_ON);

  /* Set Tap threshold  */
  LSM303AH_ACC_W_TAP_THS(0, thrsld);

  /* Set Tap duration  */
  LSM303AH_ACC_W_QUIET(0, int_dur);

  /* Set Tap shock  */
  LSM303AH_ACC_W_SHOCK(0, shock);

  /* Enable Tap axis */
  LSM303AH_ACC_W_TAP_Z_EN(0, LSM303AH_ACC_TAP_Z_EN_ON);
  LSM303AH_ACC_W_TAP_Y_EN(0, LSM303AH_ACC_TAP_Y_EN_ON);
  LSM303AH_ACC_W_TAP_X_EN(0, LSM303AH_ACC_TAP_X_EN_ON);

  /* Tap on INT1 */
  LSM303AH_ACC_W_INT1_TAP(0, LSM303AH_ACC_INT1_TAP_ON);
  LSM303AH_ACC_W_INT1_S_TAP(0, LSM303AH_ACC_INT1_S_TAP_ON);

  /* Clear WU_IA event*/
  LSM303AH_ACC_R_TAP_IA(0, &TapStatus);
}

/*
 * Configure the FreeFall event.
 */
static void SetupFreeFallEvent(u8_t ff_dur, u8_t lir_on)
{
  LSM303AH_ACC_FF_IA_DUP2_t FreeFallStatus;

  /* Set interrupt latched mode */
  if (lir_on)
    LSM303AH_ACC_W_LIR(0, LSM303AH_ACC_LIR_ON);

  /* Set FreeFall Duration */
  LSM303AH_ACC_W_FreeFallDuration(0, ff_dur);

  /* Enable FreeFall Interrupt */
  LSM303AH_ACC_W_INT1_FF(0, LSM303AH_ACC_INT1_FF_ON);

  /* Clear FreeFall event*/
  LSM303AH_ACC_R_FF_IA_DUP2(0, &FreeFallStatus);
}

/*
 * Configure the Tilt event.
 */
static void SetupTiltEvent(u8_t lir_on)
{
  LSM303AH_ACC_TILT_INT_t TiltStatus;

  /* Set interrupt latched mode */
  if (lir_on)
    LSM303AH_ACC_W_LIR(0, LSM303AH_ACC_LIR_ON);

  /* Enable Tilt */
  LSM303AH_ACC_W_TILT_ON(0, LSM303AH_ACC_TILT_ON_ON);

  /* Enable Tilt Interrupt */
  LSM303AH_ACC_W_INT2_TILT(0, LSM303AH_ACC_INT2_TILT_ON);

  /* Clear Tilt event*/
  LSM303AH_ACC_R_TILT_INT(0, &TiltStatus);
}

/*
 * Configure the SigMotion event.
 */
static void SetupSigMotionEvent(u8_t lir_on)
{
  LSM303AH_ACC_SIG_MOT_DETECT_t SigMotionStatus;

  /* Set interrupt latched mode */
  if (lir_on)
    LSM303AH_ACC_W_LIR(0, LSM303AH_ACC_LIR_ON);

  /* Enable SigMotion */
  LSM303AH_ACC_W_SIGN_MOT_ON(0, LSM303AH_ACC_SIGN_MOT_ON_ON);

  /* Enable SigMotion Interrupt */
  LSM303AH_ACC_W_INT2_SIG_MOT(0, LSM303AH_ACC_INT2_SIG_MOT_ON);

  /* Clear SigMotion event*/
  LSM303AH_ACC_R_SIG_MOT_DETECT(0, &SigMotionStatus);
}

/* 
 * Init the Accelerometer using specific FIFO mode.
 */
static void init_LSM303AH_FIFO(LSM303AH_ACC_FMODE_t mode)
{
#ifdef TEST_WITH_FTH_INTERRUPT
  LSM303AH_ACC_WU_IA_DUP2_t WakeupStatus;

  /* Turn the FIFO off */
  LSM303AH_ACC_W_FMODE(0, LSM303AH_ACC_FMODE_BYPASS);

  /* Configure FIFO elements */
  switch(mode) {
  /*
   * FIFO in continuos (stream) Mode.
   * Interrupt is generated on INT1 when a threshold is reached.
   */
  case LSM303AH_ACC_FMODE_STREAM:
    RegisterInterrupt(LSM303AH_ACC_sample_Callback);

    /* Config XL interrupt (FIFO FULL) on INT1 */
    LSM303AH_ACC_W_INT1_FIFO_FULL(0, LSM303AH_ACC_INT1_FIFO_FULL_ON);

    /* Set FIFO threshold */
    LSM303AH_ACC_W_FifoThsld(0, FIFO_THRESHOLD);
    break;

  /*
   * FIFO in Fifo Mode.
   * Interrupt is generated on INT2 when FIFO is full.
   */
  case LSM303AH_ACC_FMODE_FIFO:
    RegisterInterrupt(LSM303AH_ACC_sample_Callback_fifo);

    /* Config XL interrupt (FIFO threshold) on INT2 */
    LSM303AH_ACC_W_INT2_FTH(0, LSM303AH_ACC_INT2_FTH_ON);

    /* Set FIFO threshold to fifo full */
    LSM303AH_ACC_W_FifoThsld(0, FIFO_SIZE-1);
    break;

  /*
   * FIFO in Bypass-to-stream Mode.
   * Interrupt is generated on INT2 when threshold is reached.
   */
  case LSM303AH_ACC_FMODE_BTS:
    RegisterInterrupt(LSM303AH_ACC_sample_Callback_bts);

    /* Set Wakeup Event */
    SetupWakeUpEvent(2, 0, 1);

    /* Config XL interrupt (FIFO threshold) on INT2 */
    LSM303AH_ACC_W_INT2_FTH(0, LSM303AH_ACC_INT2_FTH_ON);

    /* Set FIFO threshold */
    LSM303AH_ACC_W_FifoThsld(0, FIFO_THRESHOLD);
    break;

  /*
   * FIFO in Stream-to-FIFO Mode.
   * Interrupt is generated on INT2 when threshold is reached.
   */
  case LSM303AH_ACC_FMODE_STF:
    RegisterInterrupt(LSM303AH_ACC_sample_Callback_stf);

    /* Set Wakeup Event */
    SetupWakeUpEvent(2, 0, 1);

    /* Config XL interrupt (FIFO threshold) */
    LSM303AH_ACC_W_INT2_FTH(0, LSM303AH_ACC_INT2_FTH_ON);

    /* Set FIFO threshold */
    LSM303AH_ACC_W_FifoThsld(0, FIFO_THRESHOLD);
    break;
  }

  /* Set FIFO mode according to argument */
  LSM303AH_ACC_W_FMODE(0, mode);

#endif

  /* Set ACC full scale @ 2g*/
  response = LSM303AH_ACC_W_FullScale(0, LSM303AH_ACC_FS_2G);
  if(response==MEMS_ERROR) while(1); //manage here comunication error

  /* BDU Enable */
  response = LSM303AH_ACC_W_BDU(0, LSM303AH_ACC_BDU_ON);
  if(response==MEMS_ERROR) while(1); //manage here comunication error

  /* Set ACC ODR  HR_14bit 100Hz*/
  response = LSM303AH_ACC_W_ODR(0, LSM303AH_ACC_ODR_HR_14bit_200Hz);
  if(response==MEMS_ERROR) while(1); //manage here comunication error
}

int init_LSM303AH_mag(LSM303AH_MAG_ODR_t odr)
{
  u8_t who_am_i;
  //u16_t value;

  /* Read WHO_AM_I  and check if device is really the correct one */
  LSM303AH_MAG_R_WHO_AM_I(0, &who_am_i);
  if (who_am_i != LSM303AH_MAG_WHO_AM_I)
    return -1;

  /* Enable the magnetometer */
  LSM303AH_MAG_W_ODR(0, odr);
  LSM303AH_MAG_W_MD(0, LSM303AH_MAG_MD_CONTINUOS_MODE);

  return(0);
}

/* Test Acquisition of sensor samples */
static  void Loop_Test_Sample_Aquisition(void)
{
  /* configure device according to what FIFO mode should be used */
  init_LSM303AH_FIFO(LSM303AH_ACC_FMODE_STREAM);
  //init_LSM303AH_FIFO(LSM303AH_ACC_FMODE_FIFO);
  //init_LSM303AH_FIFO(LSM303AH_ACC_FMODE_BTS);
  //init_LSM303AH_FIFO(LSM303AH_ACC_FMODE_STF);

  /*
   * Set mag to lowest ODR, because in this example we read one
   * mag samples every FIFO-threshold acc samples.
   */
  if (init_LSM303AH_mag(LSM303AH_MAG_ODR_10Hz) < 0)
    while(1); /* handle error */

#ifdef TEST_WITH_FTH_INTERRUPT
  while(1) {
    /* Event will be handled in driver callback */
  }
#else
  LSM303AH_ACC_DRDY_t value_XL;

  /*
   * Read samples in polling mode (no int)
   */
  while(1)
  {
    /*
     * Read ACC output only if new ACC value is available
     */
    response =  LSM303AH_ACC_R_DRDY(&value_XL);
    if(response==MEMS_ERROR) while(1); //manage here comunication error  
    
    if (LSM303AH_ACC_DRDY_EV_ON == value_XL)
    {
      LSM303AH_ACC_Get_Acceleration(Acceleration_G[0]);
    }

    /* Read LSM303AGR also the MAG sample */
    LSM303AH_MAG_Get_Magnetic(Magnetic_mGa);
  }
#endif
}

/*
 * Wakeup feature
 */

static u32_t LSM303AH_ACC_wakeup_ev_num = 0;

/*
 * Callback to handle the Wakeup event.
 * It must be registered to be called at interrupt time.
 */
static void LSM303AH_ACC_Wakeup_Callback(u8_t intID)
{
  LSM303AH_ACC_WU_IA_DUP2_t WakeupStatus;

  LSM303AH_ACC_R_WU_IA_DUP2(0, &WakeupStatus);
  if (WakeupStatus == LSM303AH_ACC_WU_IA_DUP2_EV_ON) {
    LSM303AH_ACC_wakeup_ev_num++;

    /* handle event */
    WakeupStatus = LSM303AH_ACC_WU_IA_DUP2_EV_OFF;
  }
}

/* Init the Wakeup feature */
static void init_LSM303AH_Wakeup(void)
{
#ifdef TEST_WITH_FTH_INTERRUPT
  /* register callback */
  RegisterInterrupt(LSM303AH_ACC_Wakeup_Callback);
#endif

  /* Set ACC ODR  HR_14bit 100Hz*/
  response = LSM303AH_ACC_W_ODR(0, LSM303AH_ACC_ODR_HR_14bit_100Hz);
  if(response==MEMS_ERROR) while(1); //manage here comunication error

  /* Set Wakeup Event */
  SetupWakeUpEvent(2, 0, 1);
}

/* Test Wakeup */
static void  Loop_Test_Wakeup(void)
{
  LSM303AH_ACC_WU_IA_DUP2_t WakeupStatus;

  /* Clear WU_IA event*/
  LSM303AH_ACC_R_WU_IA_DUP2(0, &WakeupStatus);

  /* configure wakeup */
  init_LSM303AH_Wakeup();

#ifdef TEST_WITH_FTH_INTERRUPT
  while(1) {
    /* Event will be handled in driver callback */
  }
#else
  /*
   * Handle the event using polling mode
   */
  while(1) {
    LSM303AH_ACC_R_WU_IA_DUP2(&WakeupStatus);
    if (WakeupStatus == LSM303AH_ACC_WU_IA_DUP2_EV_ON) {
      /* handle event */
      WakeupStatus = LSM303AH_ACC_WU_IA_DUP2_EV_OFF;
    }
  }
#endif
}

/*
 * TAP feature
 */

static u32_t LSM303AH_ACC_tap_ev_num = 0;

/*
 * Callback to handle the Tap event.
 * It must be registered to be called at interrupt time.
 */
static void LSM303AH_ACC_Tap_Callback(u8_t intID)
{
  LSM303AH_ACC_TAP_IA_t TapStatus;

  LSM303AH_ACC_R_TAP_IA(0, &TapStatus);
  if (TapStatus == LSM303AH_ACC_TAP_IA_EV_ON) {
    LSM303AH_ACC_tap_ev_num++;

    /* handle event */
    TapStatus = LSM303AH_ACC_TAP_IA_EV_OFF;
  }
}

/* Init the TAP feature */
static void init_LSM303AH_Tap(void)
{
  /* register callback */
  RegisterInterrupt(LSM303AH_ACC_Tap_Callback);

  /* Set ACC ODR  HR_14bit 100Hz*/
  response = LSM303AH_ACC_W_ODR(0, LSM303AH_ACC_ODR_HR_14bit_400Hz);
  if(response==MEMS_ERROR) while(1); //manage here comunication error

  /* Set Tap Event */
  SetupTapEvent(9, 1, 2, 1);
}

/* Test TAP */
static void  Loop_Test_Tap(void)
{
  LSM303AH_ACC_TAP_IA_t TapStatus;

  /* Clear event*/
  LSM303AH_ACC_R_TAP_IA(0, &TapStatus);

  /* configure tap */
  init_LSM303AH_Tap();

  while(1) {
    /* Event will be handled in driver callback */
  }
}

/*
 * FreeFall feature
 */

static u32_t LSM303AH_ACC_FreeFall_ev_num = 0;

/*
 * Callback to handle the FreeFall event.
 * It must be registered to be called at interrupt time.
 */
static void LSM303AH_ACC_FreeFall_Callback(u8_t intID)
{
  LSM303AH_ACC_FF_IA_DUP2_t FreeFallStatus;

  LSM303AH_ACC_R_FF_IA_DUP2(0, &FreeFallStatus);
  if (FreeFallStatus == LSM303AH_ACC_FF_IA_DUP2_EV_ON) {
    LSM303AH_ACC_FreeFall_ev_num++;

    /* handle event */
    FreeFallStatus = LSM303AH_ACC_FF_IA_DUP2_EV_OFF;
  }
}

/* Init the FreeFall feature */
static void init_LSM303AH_FreeFall(void)
{
  /* register callback */
  RegisterInterrupt(LSM303AH_ACC_FreeFall_Callback);

  /* Set ACC ODR  HR_14bit 100Hz*/
  response = LSM303AH_ACC_W_ODR(0, LSM303AH_ACC_ODR_HR_14bit_400Hz);
  if(response==MEMS_ERROR) while(1); //manage here comunication error

  /* Set FreeFall Event */
  SetupFreeFallEvent(9, 1);
}

/* Test FreeFall */
static void  Loop_Test_FreeFall(void)
{
  LSM303AH_ACC_FF_IA_DUP2_t FreeFallStatus;

  /* Clear event*/
  LSM303AH_ACC_R_FF_IA_DUP2(0, &FreeFallStatus);

  /* configure tap */
  init_LSM303AH_FreeFall();

  while(1) {
    /* Event will be handled in driver callback */
  }
}


/*
 * Tilt feature
 */

static u32_t LSM303AH_ACC_Tilt_ev_num = 0;

/*
 * Callback to handle the Tilt event.
 * It must be registered to be called at interrupt time.
 */
static void LSM303AH_ACC_Tilt_Callback(u8_t intID)
{
  LSM303AH_ACC_TILT_INT_t TiltStatus;

  LSM303AH_ACC_R_TILT_INT(0, &TiltStatus);
  if (TiltStatus == LSM303AH_ACC_TILT_INT_EV_ON) {
    LSM303AH_ACC_Tilt_ev_num++;

    /* handle event */
    TiltStatus = LSM303AH_ACC_TILT_INT_EV_OFF;
  }
}

/* Init the Tilt feature */
static void init_LSM303AH_Tilt(void)
{
  /* register callback */
  RegisterInterrupt(LSM303AH_ACC_Tilt_Callback);

  /* Set ACC ODR  HR_14bit 100Hz*/
  response = LSM303AH_ACC_W_ODR(0, LSM303AH_ACC_ODR_HR_14bit_400Hz);
  if(response==MEMS_ERROR) while(1); //manage here comunication error

  /* Set Tilt Event */
  SetupTiltEvent(1);
}

/* Test Tilt */
static void  Loop_Test_Tilt(void)
{
  LSM303AH_ACC_TILT_INT_t TiltStatus;

  /* Clear event*/
  LSM303AH_ACC_R_TILT_INT(0, &TiltStatus);

  /* configure tap */
  init_LSM303AH_Tilt();

  while(1) {
    /* Event will be handled in driver callback */
  }
}

/*
 * SigMotion feature
 */

static u32_t LSM303AH_ACC_SigMotion_ev_num = 0;

/*
 * Callback to handle the SigMotion event.
 * It must be registered to be called at interrupt time.
 */
static void LSM303AH_ACC_SigMotion_Callback(u8_t intID)
{
  LSM303AH_ACC_SIG_MOT_DETECT_t SigMotionStatus;

  LSM303AH_ACC_R_SIG_MOT_DETECT(0, &SigMotionStatus);
  if (SigMotionStatus == LSM303AH_ACC_SIG_MOT_DETECT_EV_ON) {
    LSM303AH_ACC_SigMotion_ev_num++;

    /* handle event */
    SigMotionStatus = LSM303AH_ACC_SIG_MOT_DETECT_EV_OFF;
  }
}

/* Init the SigMotion feature */
static void init_LSM303AH_SigMotion(void)
{
  /* register callback */
  RegisterInterrupt(LSM303AH_ACC_SigMotion_Callback);

  /* Set ACC ODR  HR_14bit 100Hz*/
  response = LSM303AH_ACC_W_ODR(0, LSM303AH_ACC_ODR_HR_14bit_400Hz);
  if(response==MEMS_ERROR) while(1); //manage here comunication error

  /* Set SigMotion Event */
  SetupSigMotionEvent(1);
}

/* Test SigMotion */
static void  Loop_Test_SigMotion(void)
{
  LSM303AH_ACC_SIG_MOT_DETECT_t SigMotionStatus;

  /* Clear event*/
  LSM303AH_ACC_R_SIG_MOT_DETECT(0, &SigMotionStatus);

  /* configure tap */
  init_LSM303AH_SigMotion();

  while(1) {
    /* Event will be handled in driver callback */
  }
}

/*
 * MAIN application
 */
int main(void)
{
  u8 who_am_i = 0x0;

  InitHardware();
  I2C_MEMS_Init();

  /* Read WHO_AM_I  and check if device is really the correct one */
  LSM303AH_ACC_R_WHO_AM_I_BIT(0, &who_am_i);
  if (who_am_i != LSM303AH_ACC_WHO_AM_I)
    while(1); //manage here comunication error

  /* Software reset the LSM303AH device */
  LSM303AH_ACC_W_SOFT_RESET(0, LSM303AH_ACC_SOFT_RESET_ON);

  /*
   * Test routines.
   * Uncomment the one you need to exec.
   */

  /* Test sensor samples acquisition */
  Loop_Test_Sample_Aquisition();

  /* Test Wakeup */
  //Loop_Test_Wakeup();

  /* Test TAP */
  //Loop_Test_Tap();

  /* Test Free Fall */
  //Loop_Test_FreeFall();

  /* Test Tilt */
  //Loop_Test_Tilt();

  /* Test SigMotion */
  //Loop_Test_SigMotion();

  return(0);
}

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
