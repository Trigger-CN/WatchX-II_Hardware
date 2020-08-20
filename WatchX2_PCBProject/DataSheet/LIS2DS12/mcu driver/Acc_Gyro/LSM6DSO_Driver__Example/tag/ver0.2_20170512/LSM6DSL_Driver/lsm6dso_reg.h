/**
 ******************************************************************************
 * @file    lsm6dso_reg.h
 * @author  MEMS Software Solution Team
 * @version V0.2
 * @date    12-May-2017
 * @brief   LSM6DSO driver file
 ******************************************************************************
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LSM6DSO_DRIVER__H
#define __LSM6DSO_DRIVER__H

#ifdef __cplusplus
  extern "C" {
#endif

/******************************************************************************/
/* Header */
/******************************************************************************/

#include <stdint.h> 
      
#ifndef __SHARED__TYPES
#define __SHARED__TYPES

typedef union{
	int16_t i16bit[3];
	uint8_t u8bit[6];
} axis3bit16_t;

typedef union{
	int16_t i16bit;
	uint8_t u8bit[2];
} axis1bit16_t;

typedef union{
	int32_t i32bit;
	uint8_t u8bit[4];
} axis1bit32_t;

#define PROPERTY_DISABLE                (0)
#define PROPERTY_ENABLE                 (1)

#endif /*__SHARED__TYPES*/

/* Exported types ------------------------------------------------------------*/
typedef int32_t (*lsm6dsl_write_ptr)(void *, uint8_t, uint8_t*, uint16_t);
typedef int32_t (*lsm6dsl_read_ptr) (void *, uint8_t, uint8_t*, uint16_t);

typedef struct {
  /* Component mandatory fields */
  lsm6dsl_write_ptr  write_reg;
  lsm6dsl_read_ptr read_reg;
  void *handle;
} lsm6dso_ctx_t;


/************** I2C Address *****************/

/************** Who am I  *******************/

#define LSM6DSO_ID            0x6C

/************** Device Register  *******************/

#define LSM6DSO_SDO_CRTL  0x02
typedef struct {
   unsigned char not_used_01    : 6; 
   unsigned char sdo_pu_en      : 1;
   unsigned char not_used_02    : 1;
} lsm6dso_sdo_crtl_t;

#define LSM6DSO_FIFO_CTRL1  0x07 /* + wtm8 */

#define LSM6DSO_FIFO_CTRL2  0x08
typedef struct {
   unsigned char wtm8              : 1; /* parte del registro LSM6DSO_FIFO_CTRL1 */
   unsigned char uncoptr_rate      : 2;
   unsigned char not_used_01       : 1;
   unsigned char odrchg_en         : 1;
   unsigned char not_used_02       : 1;
   unsigned char fifo_compr_rt_en  : 1;
   unsigned char stop_on_wtm       : 1;
} lsm6dso_fifo_ctrl2_t;

#define LSM6DSO_FIFO_CTRL3  0x09
typedef struct {
   unsigned char bdr_xl  : 4; 
   unsigned char bdr_gy  : 4;
} lsm6dso_fifo_ctrl3_t;

#define LSM6DSO_FIFO_CTRL4  0x0A
typedef struct {
   unsigned char fifo_mode     : 3; 
   unsigned char not_used_01   : 1;
   unsigned char odr_t_batch   : 2;
   unsigned char dec_ts_batch  : 2;
} lsm6dso_fifo_ctrl4_t;


#define LSM6DSO_COUNTER_BDR_REG1  0x0B
typedef struct {
   unsigned char cnt_bdr_th        : 3;  /* parte del registro COUNTER_BDR_REG2 */
   unsigned char not_used_01       : 1;
   unsigned char not_used_02       : 1;
   unsigned char trig_counter_bdr  : 1;
   unsigned char rst_counter_brd   : 1;
   unsigned char dataready_pulsed  : 1;
} lsm6dso_counter_bdr_reg1_t;


#define LSM6DSO_COUNTER_BDR_REG2  0x0C  /* + cnt_bdr_th*/


#define LSM6DSO_INT1_CTRL  0x0D
typedef struct {
   unsigned char int1_drdy_xl    : 1; 
   unsigned char int1_drdy_g     : 1;
   unsigned char int1_boot       : 1;
   unsigned char int1_fifo_th    : 1;
   unsigned char int1_fifo_ovr   : 1;
   unsigned char int1_fifo_full  : 1;
   unsigned char int1_cnt_bdr    : 1;
   unsigned char den_drdy_flag   : 1;
} lsm6dso_int1_ctrl_t;



#define LSM6DSO_INT2_CTRL  0x0E
typedef struct {
   unsigned char int2_drdy_xl    : 1; 
   unsigned char int2_drdy_g     : 1;
   unsigned char int2_drdy_temp  : 1;
   unsigned char int2_fifo_th    : 1;
   unsigned char int2_fifo_ovr   : 1;
   unsigned char int2_fifo_full  : 1;
   unsigned char int2_cnt_bdr    : 1;
   unsigned char not_used_01     : 1;
} lsm6dso_int2_ctrl_t;




#define LSM6DSO_WHO_AM_I  0x0F


#define LSM6DSO_CTRL1_XL  0x10
typedef struct {
   unsigned char not_used_01  : 1; 
   unsigned char hr_xl        : 1;
   unsigned char fs_xl        : 2;
   unsigned char odr_xl       : 4;
} lsm6dso_ctrl1_xl_t;


#define LSM6DSO_CTRL2_G  0x11
typedef struct {
   unsigned char not_used_01  : 1; 
   unsigned char fs_g         : 3; /* FS_125 is included */
   unsigned char odr_g        : 4;
} lsm6dso_ctrl2_g_t;


#define LSM6DSO_CTRL3_C  0x12
typedef struct {
   unsigned char sw_reset     : 1; 
   unsigned char not_used_01  : 1;
   unsigned char if_inc       : 1;
   unsigned char sim          : 1;
   unsigned char pp_od        : 1;
   unsigned char h_lactive    : 1;
   unsigned char bdu          : 1;
   unsigned char boot         : 1;
} lsm6dso_ctrl3_c_t;


#define LSM6DSO_CTRL4_C  0x13
typedef struct {
   unsigned char not_used_01   : 1; 
   unsigned char lpf1_sel_g    : 1;
   unsigned char i2c_disable   : 1;
   unsigned char drdy_mask     : 1;
   unsigned char not_used_02   : 1;
   unsigned char int2_on_int1  : 1;
   unsigned char sleep_g       : 1;
   unsigned char not_used_03   : 1;
} lsm6dso_ctrl4_c_t;



#define LSM6DSO_CTRL5_C  0x14
typedef struct {
   unsigned char st_xl        : 2; 
   unsigned char st_g         : 2;
   unsigned char not_used_01  : 1;
   unsigned char rounding     : 2;
   unsigned char not_used_02  : 1;
} lsm6dso_ctrl5_c_t;


#define LSM6DSO_CTRL6_C  0x15
typedef struct {
   unsigned char ftype  : 3; 
   unsigned char usr_off_w  : 1;
   unsigned char xl_hm_mode  : 1;
   unsigned char lvl_en  : 3; /* TRIG_EN bit included */
} lsm6dso_ctrl6_c_t;

#define LSM6DSO_CTRL7_G  0x16
typedef struct {
   unsigned char ois_enable_from_ui      : 1; 
   unsigned char not_used_01             : 1;
   unsigned char ois_enable_from_ui_sel  : 1;
   unsigned char not_used_02             : 4;
   unsigned char g_hm_mode               : 1;
} lsm6dso_ctrl7_g_t;


#define LSM6DSO_CTRL8_XL  0x17
typedef struct {
   unsigned char low_pass_on_6d     : 1; 
   unsigned char xl_fs_mode         : 1;
   unsigned char hp_slope_xl_en     : 1;
   unsigned char fastsettl_mode_xl  : 1;
   unsigned char hp_ref_mode_xl     : 1;
   unsigned char hplf2_xl_bw        : 3;
} lsm6dso_ctrl8_xl_t;


#define LSM6DSO_CTRL9_XL  0x18
typedef struct {
   unsigned char not_used_01  : 1; 
   unsigned char i3c_disable  : 1;
   unsigned char den_lh       : 1;
   unsigned char den_xl_en    : 1;
   unsigned char den_xl_g     : 1;
   unsigned char den_x        : 1;
   unsigned char den_y        : 1;
   unsigned char den_z        : 1;
} lsm6dso_ctrl9_xl_t;



#define LSM6DSO_CTRL10_C  0x19
typedef struct {
   unsigned char not_used_01   : 5; 
   unsigned char timestamp_en  : 1;
   unsigned char not_used_02   : 2; 
} lsm6dso_ctrl10_c_t;



#define LSM6DSO_ALL_INT_SRC  0x1A
typedef struct {
   unsigned char ff_ia_all           : 1; 
   unsigned char wu_ia_all           : 1;
   unsigned char single_tap_all      : 1;
   unsigned char double_tap_all      : 1;
   unsigned char d6d_ia_all          : 1;
   unsigned char sleep_change_all    : 1;
   unsigned char not_used_01         : 1;
   unsigned char timestamp_endcount  : 1;
} lsm6dso_all_int_src_t;

#define LSM6DSO_WAKE_UP_SRC  0x1B
typedef struct {
   unsigned char z_wu            : 1; 
   unsigned char y_wu            : 1;
   unsigned char x_wu            : 1;
   unsigned char wu_ia           : 1;
   unsigned char sleep_state_ia  : 1;
   unsigned char ff_ia           : 1;
   unsigned char not_used_01     : 2;
} lsm6dso_wake_up_src_t;



#define LSM6DSO_TAP_SRC  0x1C
typedef struct {
   unsigned char z_tap        : 1; 
   unsigned char y_tap        : 1;
   unsigned char x_tap        : 1;
   unsigned char tap_sign     : 1;
   unsigned char double_tap   : 1;
   unsigned char single_tap   : 1;
   unsigned char tap_ia       : 1;
   unsigned char not_used_01  : 1;
} lsm6dso_tap_src_t;



#define LSM6DSO_D6D_SRC  0x1D
typedef struct {
   unsigned char xl        : 1; 
   unsigned char xh        : 1;
   unsigned char yl        : 1;
   unsigned char yh        : 1;
   unsigned char zl        : 1;
   unsigned char zh        : 1;
   unsigned char d6d_ia    : 1;
   unsigned char den_drdy  : 1;
} lsm6dso_d6d_src_t;



#define LSM6DSO_STATUS_REG  0x1E
typedef struct {
   unsigned char xlda         : 1; 
   unsigned char gda          : 1;
   unsigned char tda          : 1;
   unsigned char not_used_01  : 5;
} lsm6dso_status_reg_t;



#define LSM6DSO_STATUS_SPIAux  0x1E
typedef struct {
   unsigned char xlda           : 1; 
   unsigned char gda            : 1;
   unsigned char gyro_settling  : 1;
   unsigned char not_used_01    : 5;
} lsm6dso_status_spiaux_t;


#define LSM6DSO_OUT_TEMP_L  0x20
#define LSM6DSO_OUT_TEMP_H  0x21


#define LSM6DSO_OUTX_L_G  0x22
#define LSM6DSO_OUTX_H_G  0x23
#define LSM6DSO_OUTY_L_G  0x24
#define LSM6DSO_OUTY_H_G  0x25
#define LSM6DSO_OUTZ_L_G  0x26
#define LSM6DSO_OUTZ_H_G  0x27


#define LSM6DSO_OUTX_L_A  0x28
#define LSM6DSO_OUTX_H_A  0x29
#define LSM6DSO_OUTY_L_A  0x2A
#define LSM6DSO_OUTY_H_A  0x2B
#define LSM6DSO_OUTZ_L_A  0x2C
#define LSM6DSO_OUTZ_H_A  0x2D


#define LSM6DSO_FIFO_STATUS1  0x3A  /* + diff_fifo */

#define LSM6DSO_FIFO_STATUS2  0x3B
typedef struct {
   unsigned char diff_fifo              : 3; /* parte del registro LSM6DSO_FIFO_STATUS1 */
   unsigned char over_run_latched       : 1;
   unsigned char counter_bdr_ia         : 1;
   unsigned char fifo_full_ia           : 1;
   unsigned char fifo_over_run_latched  : 1;
   unsigned char fifo_wtm_ia            : 1;
} lsm6dso_fifo_status2_t;



#define LSM6DSO_FIFO_CNT_BDR_L  0x3C
#define LSM6DSO_FIFO_CNT_BDR_H  0x3D /* + fifo_cnt_bdr*/
typedef struct {
   unsigned char fifo_cnt_bdr  : 3; /* parte del registro LSM6DSO_FIFO_CNT_BDR_L */
   unsigned char not_used_01   : 5;
} lsm6dso_fifo_cnt_bdr_h_t;


#define LSM6DSO_TIMESTAMP0  0x40
#define LSM6DSO_TIMESTAMP1  0x41
#define LSM6DSO_TIMESTAMP2  0x42
#define LSM6DSO_TIMESTAMP3  0x43


#define LSM6DSO_TAP_CFG0  0x56
typedef struct {
   unsigned char lir                  : 1; 
   unsigned char tap_z_en             : 1;
   unsigned char tap_y_en             : 1;
   unsigned char tap_x_en             : 1;
   unsigned char slope_fds            : 1;
   unsigned char sleep_status_on_int  : 1;
   unsigned char not_used_01          : 2;
} lsm6dso_tap_cfg0_t;


#define LSM6DSO_TAP_CFG1  0x57
typedef struct {
   unsigned char tap_ths_x     : 5; 
   unsigned char tap_priority  : 3;
} lsm6dso_tap_cfg1_t;



#define LSM6DSO_TAP_CFG2  0x58
typedef struct {
   unsigned char tap_ths_y          : 5; 
   unsigned char inact_en           : 2;
   unsigned char interrupts_enable  : 1;
} lsm6dso_tap_cfg2_t;


#define LSM6DSO_TAP_THS_6D  0x59
typedef struct {
   unsigned char tap_ths_z : 5; 
   unsigned char sixd_ths  : 2;
   unsigned char d4d_en    : 1;
} lsm6dso_tap_ths_6d_t;



#define LSM6DSO_INT_DUR2  0x5A
typedef struct {
   unsigned char shock  : 2; 
   unsigned char quiet  : 2;
   unsigned char dur    : 4;
} lsm6dso_int_dur2_t;



#define LSM6DSO_WAKE_UP_THS  0x5B
typedef struct {
   unsigned char wk_ths  : 6; 
   unsigned char usr_off_on_wu  : 1;
   unsigned char single_double_tap  : 1;
} lsm6dso_wake_up_ths_t;


#define LSM6DSO_WAKE_UP_DUR  0x5C
typedef struct {
   unsigned char sleep_dur    : 4; 
   unsigned char not_used_01  : 1;
   unsigned char wake_dur     : 2;
   unsigned char ff_dur       : 1; /*see also FREE_FALL (5Dh) register */
} lsm6dso_wake_up_dur_t;


#define LSM6DSO_FREE_FALL  0x5D
typedef struct {
   unsigned char ff_ths  : 3; 
   unsigned char ff_dur  : 5; /*see also LSM6DSO_WAKE_UP_DUR  0x5C register */
} lsm6dso_free_fall_t;



#define LSM6DSO_MD1_CFG  0x5E
typedef struct {
   unsigned char not_used_01       : 2; 
   unsigned char int1_6d           : 1;
   unsigned char int1_double_tap   : 1;
   unsigned char int1_ff           : 1;
   unsigned char int1_wu           : 1;
   unsigned char int1_single_tap   : 1;
   unsigned char int1_inact_state  : 1;
} lsm6dso_md1_cfg_t;




#define LSM6DSO_MD2_CFG  0x5F
typedef struct {
   unsigned char int2_timestamp     : 1; 
   unsigned char not_used_01        : 1;
   unsigned char int2_6d            : 1;
   unsigned char int2_double_tap    : 1;
   unsigned char int2_ff            : 1;
   unsigned char int2_wu            : 1;
   unsigned char int2_single_tap    : 1;
   unsigned char int2_sleep_change  : 1;
} lsm6dso_md2_cfg_t;



#define LSM6DSO_INT_OIS  0x6F
typedef struct {
   unsigned char st_xl_ois      : 2; 
   unsigned char not_used_01    : 3;
   unsigned char den_lh_ois     : 1;
   unsigned char lvl2_ois       : 1;
   unsigned char int2_drdy_ois  : 1;
} lsm6dso_int_ois_t;



#define LSM6DSO_CTRL1_OIS  0x70
typedef struct {
   unsigned char ois_en_spi2  : 1; 
   unsigned char fs_g_ois     : 3; /* fs_125_ois  is included */
   unsigned char ois_xl_en    : 1;
   unsigned char sim_ois      : 1;
   unsigned char lvl_ois      : 1;
   unsigned char not_used_01  : 1;
} lsm6dso_ctrl1_ois_t;


#define LSM6DSO_CTRL2_OIS  0x71
typedef struct {
   unsigned char hp_en_ois    : 1; 
   unsigned char ftype_ois    : 2;
   unsigned char not_used_01  : 1;
   unsigned char hpm_ois      : 2;
   unsigned char not_used_02  : 2;
} lsm6dso_ctrl2_ois_t;


#define LSM6DSO_CTRL3_OIS  0x72
typedef struct {
   unsigned char st_ois_clampdis     : 1; 
   unsigned char st_ois              : 2;
   unsigned char filter_xl_conf_ois  : 3;
   unsigned char fs_xl_ois           : 2;
} lsm6dso_ctrl3_ois_t;


#define LSM6DSO_X_OFS_USR  0x73
#define LSM6DSO_Y_OFS_USR  0x74
#define LSM6DSO_Z_OFS_USR  0x75




#define LSM6DSO_FIFO_DATA_OUT_TAG  0x78
typedef struct {
   unsigned char tag_parity  : 1; 
   unsigned char tag_cnt     : 2;
   unsigned char tag_sensor  : 5;
} lsm6dso_fifo_data_out_tag_t;





#define LSM6DSO_FIFO_DATA_OUT_X_L  0x79
#define LSM6DSO_FIFO_DATA_OUT_X_H  0x7A

#define LSM6DSO_FIFO_DATA_OUT_Y_L  0x7B
#define LSM6DSO_FIFO_DATA_OUT_Y_H  0x7C

#define LSM6DSO_FIFO_DATA_OUT_Z_L  0x7D
#define LSM6DSO_FIFO_DATA_OUT_Z_H  0x7E


typedef enum {
  NONE            = 0,
  EVERY_8_BATCH   = 2,
  EVERY_16_BATCH  = 3,
  EVERY_32_BATCH  = 1,    
} lsm6dso_uncoptr_rate_t;


typedef enum {
  BDR_XL_OFF     = 0, 
  BDR_XL_1Hz6    = 11,
  BDR_XL_12Hz5   = 1,
  BDR_XL_26Hz    = 2,
  BDR_XL_52Hz    = 3,  
  BDR_XL_104Hz   = 4,   
  BDR_XL_208Hz   = 5,  
  BDR_XL_417Hz   = 6,   
  BDR_XL_833Hz   = 7, 
  BDR_XL_1k67Hz  = 8, 
  BDR_XL_3k33Hz  = 9, 
  BDR_XL_6k67Hz  = 10,  
} lsm6dso_bdr_xl_t;


typedef enum {
  BDR_GY_OFF     = 0, 
  BDR_GY_6Hz5    = 11,
  BDR_GY_12Hz5   = 1,
  BDR_GY_26Hz    = 2,
  BDR_GY_52Hz    = 3,  
  BDR_GY_104Hz   = 4,   
  BDR_GY_208Hz   = 5,  
  BDR_GY_417Hz   = 6,   
  BDR_GY_833Hz   = 7, 
  BDR_GY_1k67Hz  = 8, 
  BDR_GY_3k33Hz  = 9, 
  _6k67Hz  = 10,  
} lsm6dso_bdr_gy_t;



typedef enum {
  OFF             = 0, 
  MAX_BDR_DIV_1   = 1,
  MAX_BDR_DIV_8   = 2,
  MAX_BDR_DIV_32  = 3,
} lsm6dso_dec_ts_batch_t;


typedef enum {
  TEMP_BATCH_OFF             = 0, 
  TEMP_BATCH_1Hz6   = 3,
  TEMP_BATCH_12Hz5  = 2,
  TEMP_BATCH_52Hz   = 1,
} lsm6dso_odr_t_batch_t;

typedef enum {
  BYPASS            = 0, 
  FIFO              = 1,
  DYNAMIC           = 2,
  DYNAMIC_TO_FIFO   = 3,
  BYPASS_TO_DYNAMIC = 4,
  BYPASS_TO_FIFO    = 7,
} lsm6dso_fifo_mode_t;

typedef enum {
  XL    = 0, 
  GYRO  = 1,
} lsm6dso_trig_counter_bdr_t;



typedef enum {
  ODR_XL_OFF                   = 0,  
  ODR_XL__1Hz6_LOW_POWER_ONLY  = 11,
  ODR_XL__12Hz5                = 1,
  ODR_XL__26Hz                 = 2,
  ODR_XL__52Hz                 = 3,  
  ODR_XL__104Hz                = 4,   
  ODR_XL__208Hz                = 5,  
  ODR_XL__416Hz                = 6,   
  ODR_XL__833Hz                = 7, 
  ODR_XL__1k66Hz               = 8, 
  ODR_XL__3k33Hz               = 9, 
  ODR_XL__6k66Hz               = 10,  
} lsm6dso_odr_xl_t;

typedef enum {
  _2g                  = 0,
  _4g                  = 2,
  _8g                  = 3,
  _16g_SPIT_OIS_IU_FS  = 1, /* check  XL_FS_MODE = ‘0’ in CTRL8_XL (17h) */
} lsm6dso_fs_xl_t;


typedef enum {
  ODR_GY_OFF                  = 0,  
  ODR_GY_12Hz5                = 1,
  ODR_GY_26Hz                 = 2,
  ODR_GY_52Hz                 = 3,  
  ODR_GY_104Hz                = 4,   
  ODR_GY_208Hz                = 5,  
  ODR_GY_416Hz                = 6,   
  ODR_GY_833Hz                = 7, 
  ODR_GY_1k66Hz               = 8, 
  ODR_GY_3k33Hz               = 9, 
  ODR_GY_6k66Hz               = 10,  
} lsm6dso_odr_g_t;


typedef enum {
  _125dps   = 1,
  _250dps   = 0,
  _500dps   = 2,
  _1000dps  = 4,
  _2000dps  = 6, 
} lsm6dso_fs_g_t;

typedef enum {
  _4_WIRE  = 0, 
  _3_WIRE  = 1,
} lsm6dso_sim_t;

typedef enum {
  ACTIVE_HIGH  = 0, 
  ACTIVE_LOW   = 1,
} lsm6dso_h_lactive_t;

typedef enum {
  PUSH_PULL   = 0, 
  OPEN_DRAIN  = 1,
} lsm6dso_pp_od_t;

typedef enum {
  RND_DISABLE    = 0, 
  XL_ONLY    = 1,
  GYRO_ONLY  = 2,
  XL_GYRO    = 3,
} lsm6dso_rounding_t;

typedef enum {
  ST_GY_DISABLE   = 0, 
  ST_GY_POSITIVE  = 1,
  ST_GY_NEGATIVE  = 3,
} lsm6dso_st_g_t;

typedef enum {
  ST_XL_DISABLE   = 0, 
  ST_XL_POSITIVE  = 1,
  ST_XL_NEGATIVE  = 2,
} lsm6dso_st_xl_t;

typedef enum {
  _4Hz14_ODR13Hz  = 0, 
  _4Hz15_ODR13Hz  = 1,
  _4Hz16_ODR13Hz  = 2,
  _4Hz15_II_ODR13Hz  = 3,
  _4Hz15_III_ODR13Hz  = 4,
  _4Hz16_II_ODR13Hz  = 5,
  _4Hz07_ODR13Hz  = 6,
  _3Hz87_ODR13Hz  = 7,
  _8Hz29_ODR26Hz  = 0, 
  _8Hz30_ODR26Hz  = 1,
  _8Hz31_ODR26Hz  = 2,
  _8Hz30_II_ODR26Hz  = 3,
  _8Hz31_II_ODR26Hz  = 4,
  _8Hz33_ODR26Hz  = 5,
  _7Hz74_ODR26Hz  = 6,
  _6Hz66_ODR26Hz  = 7,
  _16Hz58_ODR52Hz  = 0, 
  _16Hz61_ODR52Hz  = 1,
  _16Hz66_ODR52Hz  = 2,
  _16Hz60_ODR52Hz  = 3,
  _16Hz65_ODR52Hz  = 4,
  _16Hz77_ODR52Hz  = 5,
  _13Hz35_ODR52Hz  = 6,
  _9Hz61_ODR52Hz   = 7,  
  _33Hz26_ODR104Hz  = 0, 
  _33Hz35_ODR104Hz  = 1,
  _33Hz57_ODR104Hz  = 2,
  _33Hz28_ODR104Hz  = 3,
  _33Hz44_ODR104Hz  = 4,
  _33Hz31_ODR104Hz  = 5,
  _19Hz29_ODR104Hz  = 6,
  _11Hz47_ODR104Hz  = 7,   
  _67Hz29_ODR208Hz  = 0, 
  _67Hz46_ODR208Hz  = 1,
  _68Hz12_ODR208Hz  = 2,
  _67Hz11_ODR208Hz  = 3,
  _62Hz44_ODR208Hz  = 4,
  _43Hz23_ODR208Hz  = 5,
  _23Hz06_ODR208Hz  = 6,
  _12Hz19_ODR208Hz  = 7,     
  _138Hz_ODR416Hz  = 0,
  _131Hz_ODR416Hz  = 1,
  _121Hz_ODR416Hz  = 2,
  _138Hz_I_ODR416Hz  = 3,
  _87Hz9_ODR416Hz  = 4,
  _50Hz_ODR416Hz   = 5,
  _25Hz_ODR416Hz   = 6,
  _12Hz5_ODR416Hz  = 7,
  _245Hz_ODR833Hz  = 0,
  _195Hz_ODR833Hz  = 1,
  _155Hz_ODR833Hz  = 2,
  _293Hz_ODR833Hz  = 3,
  _96Hz9_ODR833Hz  = 4,
  _50Hz_ODR833Hz   = 5,
  _25Hz_ODR833Hz   = 6,
  _12Hz5_ODR833Hz  = 7,
  _315Hz_ODR1k67Hz  = 0,
  _225Hz_ODR1k67Hz  = 1,
  _168Hz_ODR1k67Hz  = 2,
  _506Hz_ODR1k67Hz  = 3,
  _100Hz_ODR1k67Hz  = 4,
  _50Hz_ODR1k67Hz   = 5,
  _25Hz_ODR1k67Hz   = 6,
  _12Hz5_ODR1k67Hz  = 7,
  _343Hz_ODR3k33Hz  = 0,
  _234Hz_ODR3k33Hz  = 1,
  _172Hz_ODR3k33Hz  = 2,  
  _925Hz_ODR3k33Hz  = 3,  
  _351Hz_ODR6k67Hz  = 0,
  _237Hz_ODR6k67Hz  = 1,
  _173Hz_ODR6k67Hz  = 2,  
  _938Hz_ODR6k67Hz  = 3,  
} lsm6dso_ftype_t;


typedef enum {
  DIV_1024  = 0, 
  DIV_64    = 1,
} lsm6dso_usr_off_w_t;

typedef enum {
  EDGE           = 4, 
  LEVEL          = 2,
  LEVEL_LATCHED  = 3, 
  LEVEL_FIFO     = 6,
} lsm6dso_lvl_en_t;

typedef enum {
  USE_AUX_SPI  = 0, 
  USE_UI_SPI   = 1,
} lsm6dso_ois_enable_from_ui_sel_t;

typedef enum {
  SLEEP_CHANGE  = 0, 
  SLEEP_STATUS  = 1,
} lsm6dso_sleep_status_on_int_t;

typedef enum {
  SLOPE  = 0, 
  HPF    = 1,
} lsm6dso_slope_fds_t;


typedef enum {
  XYZ  = 0,
  YXZ  = 1,
  XZY  = 2,
  ZYX  = 3,
  YZX  = 5,
  ZXY  = 6,
} lsm6dso_tap_priority_t;

typedef enum {
  INACT_DISABLE             = 0,
  XL_12Hz5_GYRO_NOT_CHANGE  = 1,
  XL_12Hz5_GYRO_TO_SLEEP    = 2,
  XL_12Hz5_GYRO_TO_PD       = 3,
} lsm6dso_inact_en_t;


typedef enum {
  _80_DEG  = 0,
  _70_DEG  = 1,
  _60_DEG  = 2,
  _50_DEG  = 3,
} lsm6dso_sixd_ths_t;



typedef enum {
  ONLY_DOUBLE  = 0, 
  ONLY_SINGLE  = 1,
} lsm6dso_single_double_tap_t;


typedef enum {
  _156mg  = 0,
  _219mg  = 1,
  _250mg  = 2,
  _312mg  = 3,
  _344mg  = 4,
  _406mg  = 5,
  _469mg  = 6,
  _500mg  = 7,
} lsm6dso_ff_ths_t;

typedef enum {
  ST_XL_OIS_DISABLE   = 0, 
  ST_XL_OIS_POSITIVE  = 1,
  ST_XL_OIS_NEGATIVE  = 2,
} lsm6dso_st_xl_ois_t;

typedef enum {
  OIS_125dps   = 1,
  OIS_250dps   = 0,
  OIS_500dps   = 2,
  OIS_1000dps  = 4,
  OIS_2000dps  = 6, 
} lsm6dsl_fs_g_ois_t;

typedef enum {
  AUX_4_WIRE  = 0, 
  AUX_3_WIRE  = 1,
} lsm6dso_sim_ois_t;


int32_t lsm6dso_read_reg(lsm6dso_ctx_t *ctx, uint8_t reg, uint8_t* data, uint16_t len);
int32_t lsm6dso_write_reg(lsm6dso_ctx_t *ctx, uint8_t reg, uint8_t* data, uint16_t len);

int32_t lsm6dso_uncompressed_data_rate_set(lsm6dso_ctx_t *ctx, lsm6dso_uncoptr_rate_t val);
int32_t lsm6dso_uncompressed_data_rate_get(lsm6dso_ctx_t *ctx, lsm6dso_uncoptr_rate_t *val);

int32_t lsm6dso_xl_batching_data_rate_set(lsm6dso_ctx_t *ctx, lsm6dso_bdr_xl_t val);
int32_t lsm6dso_xl_batching_data_rate_get(lsm6dso_ctx_t *ctx, lsm6dso_bdr_xl_t *val);

int32_t lsm6dso_gy_batching_data_rate_set(lsm6dso_ctx_t *ctx, lsm6dso_bdr_gy_t val);
int32_t lsm6dso_gy_batching_data_rate_get(lsm6dso_ctx_t *ctx, lsm6dso_bdr_gy_t *val);

int32_t lsm6dso_timestamp_decimation_set(lsm6dso_ctx_t *ctx, lsm6dso_dec_ts_batch_t val);
int32_t lsm6dso_timestamp_decimation_rate_get(lsm6dso_ctx_t *ctx, lsm6dso_dec_ts_batch_t *val);

int32_t lsm6dso_batch_temp_data_rate_set(lsm6dso_ctx_t *ctx, lsm6dso_odr_t_batch_t val);
int32_t lsm6dso_batch_temp_data_rate_get(lsm6dso_ctx_t *ctx, lsm6dso_odr_t_batch_t *val);

int32_t lsm6dso_fifo_mode_set(lsm6dso_ctx_t *ctx, lsm6dso_fifo_mode_t val);
int32_t lsm6dso_fifo_mode_get(lsm6dso_ctx_t *ctx, lsm6dso_fifo_mode_t *val);

int32_t lsm6dso_internal_trigger_set(lsm6dso_ctx_t *ctx, lsm6dso_trig_counter_bdr_t val);
int32_t lsm6dso_internal_trigger_get(lsm6dso_ctx_t *ctx, lsm6dso_trig_counter_bdr_t *val);

int32_t lsm6dso_xl_odr_set(lsm6dso_ctx_t *ctx, lsm6dso_odr_xl_t val);
int32_t lsm6dso_xl_odr_get(lsm6dso_ctx_t *ctx, lsm6dso_odr_xl_t *val);

int32_t lsm6dso_xl_full_scale_set(lsm6dso_ctx_t *ctx, lsm6dso_fs_xl_t val);
int32_t lsm6dso_xl_full_scale_get(lsm6dso_ctx_t *ctx, lsm6dso_fs_xl_t *val);

int32_t lsm6dso_gyro_odr_set(lsm6dso_ctx_t *ctx, lsm6dso_odr_g_t val);
int32_t lsm6dso_gyro_odr_get(lsm6dso_ctx_t *ctx, lsm6dso_odr_g_t *val);

int32_t lsm6dso_gyro_full_scale_set(lsm6dso_ctx_t *ctx, lsm6dso_fs_g_t val);
int32_t lsm6dso_gyro_full_scale_get(lsm6dso_ctx_t *ctx, lsm6dso_fs_g_t *val);

int32_t lsm6dso_sdo_pull_up_set(lsm6dso_ctx_t *ctx, uint8_t val);
int32_t lsm6dso_sdo_pull_up_get(lsm6dso_ctx_t *ctx, uint8_t *val);

int32_t lsm6dso_fifo_compression_set(lsm6dso_ctx_t *ctx, uint8_t val);
int32_t lsm6dso_fifo_compression_get(lsm6dso_ctx_t *ctx, uint8_t *val);

int32_t lsm6dso_fifo_compression_set(lsm6dso_ctx_t *ctx, uint8_t val);
int32_t lsm6dso_fifo_compression_get(lsm6dso_ctx_t *ctx, uint8_t *val);

int32_t lsm6dso_block_data_update_set(lsm6dso_ctx_t *ctx, uint8_t val);
int32_t lsm6dso_block_data_update_get(lsm6dso_ctx_t *ctx, uint8_t *val);

int32_t lsm6dso_int1_pin_conf_set(lsm6dso_ctx_t *ctx, lsm6dso_int1_ctrl_t val);
int32_t lsm6dso_int1_pin_conf_get(lsm6dso_ctx_t *ctx, lsm6dso_int1_ctrl_t *val);

int32_t lsm6dso_int2_pin_conf_set(lsm6dso_ctx_t *ctx, lsm6dso_int2_ctrl_t val);
int32_t lsm6dso_int2_pin_conf_get(lsm6dso_ctx_t *ctx, lsm6dso_int2_ctrl_t *val);

int32_t lsm6dso_status_get(lsm6dso_ctx_t *ctx, lsm6dso_status_reg_t *val);

int32_t lsm6dso_acceleration_raw(lsm6dso_ctx_t *ctx, uint8_t *buff);

int32_t lsm6dso_angular_rate_raw(lsm6dso_ctx_t *ctx, uint8_t *buff);

#ifdef __cplusplus
}
#endif

#endif
