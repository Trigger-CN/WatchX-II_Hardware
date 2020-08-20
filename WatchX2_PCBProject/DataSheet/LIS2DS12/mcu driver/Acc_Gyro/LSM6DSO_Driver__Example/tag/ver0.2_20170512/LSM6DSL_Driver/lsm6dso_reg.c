/**
 ******************************************************************************
 * @file    lsm6dso_reg.c
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
#include "lsm6dso_reg.h"


/** @addtogroup 
 * @{
 */

/**
  * @brief  
  *         
  * @param  
  *                
  * @retval 
  */
int32_t lsm6dso_read_reg(lsm6dso_ctx_t *ctx, uint8_t reg, uint8_t* data, uint16_t len)
{
  return ctx->read_reg(ctx->handle, reg, data, len);
}

/**
  * @brief  
  *         
  * @param  
  *                
  * @retval 
  */
int32_t lsm6dso_write_reg(lsm6dso_ctx_t *ctx, uint8_t reg, uint8_t* data, uint16_t len)
{
  return ctx->write_reg(ctx->handle, reg, data, len);
}

/**
  * @brief  
  *         
  * @param  
  *                
  * @retval 
  */
int32_t lsm6dso_uncompressed_data_rate_set(lsm6dso_ctx_t *ctx, lsm6dso_uncoptr_rate_t val)
{
  lsm6dso_fifo_ctrl2_t reg;
  int32_t mm_error;

  mm_error = lsm6dso_read_reg(ctx, LSM6DSO_FIFO_CTRL2, (uint8_t *)&reg, 1);
  reg.uncoptr_rate = val;
  mm_error = lsm6dso_write_reg(ctx, LSM6DSO_FIFO_CTRL2, (uint8_t *)&reg, 1);

  return mm_error;
}

int32_t lsm6dso_uncompressed_data_rate_get(lsm6dso_ctx_t *ctx, lsm6dso_uncoptr_rate_t *val)
{
  lsm6dso_fifo_ctrl2_t reg;
  int32_t mm_error;

  mm_error = lsm6dso_read_reg(ctx, LSM6DSO_FIFO_CTRL2, (uint8_t *)&reg, 1);
  *val = (lsm6dso_uncoptr_rate_t) reg.uncoptr_rate;

  return mm_error;
}


/**
  * @brief  
  *         
  * @param  
  *                
  * @retval 
  */
int32_t lsm6dso_xl_batching_data_rate_set(lsm6dso_ctx_t *ctx, lsm6dso_bdr_xl_t val)
{
  lsm6dso_fifo_ctrl3_t reg;
  int32_t mm_error;

  mm_error = lsm6dso_read_reg(ctx, LSM6DSO_FIFO_CTRL3, (uint8_t *)&reg, 1);
  reg.bdr_xl = val;
  mm_error = lsm6dso_write_reg(ctx, LSM6DSO_FIFO_CTRL3, (uint8_t *)&reg, 1);

  return mm_error;
}

int32_t lsm6dso_xl_batching_data_rate_get(lsm6dso_ctx_t *ctx, lsm6dso_bdr_xl_t *val)
{
  lsm6dso_fifo_ctrl3_t reg;
  int32_t mm_error;

  mm_error = lsm6dso_read_reg(ctx, LSM6DSO_FIFO_CTRL3, (uint8_t *)&reg, 1);
  *val = (lsm6dso_bdr_xl_t) reg.bdr_xl;

  return mm_error;
}

/**
  * @brief  
  *         
  * @param  
  *                
  * @retval 
  */
int32_t lsm6dso_gy_batching_data_rate_set(lsm6dso_ctx_t *ctx, lsm6dso_bdr_gy_t val)
{
  lsm6dso_fifo_ctrl3_t reg;
  int32_t mm_error;

  mm_error = lsm6dso_read_reg(ctx, LSM6DSO_FIFO_CTRL3, (uint8_t *)&reg, 1);
  reg.bdr_gy = val;
  mm_error = lsm6dso_write_reg(ctx, LSM6DSO_FIFO_CTRL3, (uint8_t *)&reg, 1);

  return mm_error;
}

int32_t lsm6dso_gy_batching_data_rate_get(lsm6dso_ctx_t *ctx, lsm6dso_bdr_gy_t *val)
{
  lsm6dso_fifo_ctrl3_t reg;
  int32_t mm_error;

  mm_error = lsm6dso_read_reg(ctx, LSM6DSO_FIFO_CTRL3, (uint8_t *)&reg, 1);
  *val = (lsm6dso_bdr_gy_t) reg.bdr_gy;

  return mm_error;
}


/**
  * @brief  
  *         
  * @param  
  *                
  * @retval 
  */
int32_t lsm6dso_timestamp_decimation_set(lsm6dso_ctx_t *ctx, lsm6dso_dec_ts_batch_t val)
{
  lsm6dso_fifo_ctrl4_t reg;
  int32_t mm_error;

  mm_error = lsm6dso_read_reg(ctx, LSM6DSO_FIFO_CTRL4, (uint8_t *)&reg, 1);
  reg.dec_ts_batch = val;
  mm_error = lsm6dso_write_reg(ctx, LSM6DSO_FIFO_CTRL4, (uint8_t *)&reg, 1);

  return mm_error;
}

int32_t lsm6dso_timestamp_decimation_rate_get(lsm6dso_ctx_t *ctx, lsm6dso_dec_ts_batch_t *val)
{
  lsm6dso_fifo_ctrl4_t reg;
  int32_t mm_error;

  mm_error = lsm6dso_read_reg(ctx, LSM6DSO_FIFO_CTRL4, (uint8_t *)&reg, 1);
  *val = (lsm6dso_dec_ts_batch_t)reg.dec_ts_batch;

  return mm_error;
}

/**
  * @brief  
  *         
  * @param  
  *                
  * @retval 
  */
int32_t lsm6dso_batch_temp_data_rate_set(lsm6dso_ctx_t *ctx, lsm6dso_odr_t_batch_t val)
{
  lsm6dso_fifo_ctrl4_t reg;
  int32_t mm_error;

  mm_error = lsm6dso_read_reg(ctx, LSM6DSO_FIFO_CTRL4, (uint8_t *)&reg, 1);
  reg.odr_t_batch = val;
  mm_error = lsm6dso_write_reg(ctx, LSM6DSO_FIFO_CTRL4, (uint8_t *)&reg, 1);

  return mm_error;
}

int32_t lsm6dso_batch_temp_data_rate_get(lsm6dso_ctx_t *ctx, lsm6dso_odr_t_batch_t *val)
{
  lsm6dso_fifo_ctrl4_t reg;
  int32_t mm_error;

  mm_error = lsm6dso_read_reg(ctx, LSM6DSO_FIFO_CTRL4, (uint8_t *)&reg, 1);
  *val = (lsm6dso_odr_t_batch_t)reg.odr_t_batch;

  return mm_error;
}


/**
  * @brief  
  *         
  * @param  
  *                
  * @retval 
  */
int32_t lsm6dso_fifo_mode_set(lsm6dso_ctx_t *ctx, lsm6dso_fifo_mode_t val)
{
  lsm6dso_fifo_ctrl4_t reg;
  int32_t mm_error;

  mm_error = lsm6dso_read_reg(ctx, LSM6DSO_FIFO_CTRL4, (uint8_t *)&reg, 1);
  reg.fifo_mode = val;
  mm_error = lsm6dso_write_reg(ctx, LSM6DSO_FIFO_CTRL4, (uint8_t *)&reg, 1);

  return mm_error;
}

int32_t lsm6dso_fifo_mode_get(lsm6dso_ctx_t *ctx, lsm6dso_fifo_mode_t *val)
{
  lsm6dso_fifo_ctrl4_t reg;
  int32_t mm_error;

  mm_error = lsm6dso_read_reg(ctx, LSM6DSO_FIFO_CTRL4, (uint8_t *)&reg, 1);
  *val = (lsm6dso_fifo_mode_t)reg.fifo_mode;

  return mm_error;
}

/**
  * @brief  
  *         
  * @param  
  *                
  * @retval 
  */
int32_t lsm6dso_internal_trigger_set(lsm6dso_ctx_t *ctx, lsm6dso_trig_counter_bdr_t val)
{
  lsm6dso_counter_bdr_reg1_t reg;
  int32_t mm_error;

  mm_error = lsm6dso_read_reg(ctx, LSM6DSO_COUNTER_BDR_REG1, (uint8_t *)&reg, 1);
  reg.trig_counter_bdr = val;
  mm_error = lsm6dso_write_reg(ctx, LSM6DSO_COUNTER_BDR_REG1, (uint8_t *)&reg, 1);

  return mm_error;
}

int32_t lsm6dso_internal_trigger_get(lsm6dso_ctx_t *ctx, lsm6dso_trig_counter_bdr_t *val)
{
  lsm6dso_counter_bdr_reg1_t reg;
  int32_t mm_error;

  mm_error = lsm6dso_read_reg(ctx, LSM6DSO_COUNTER_BDR_REG1, (uint8_t *)&reg, 1);
  *val = (lsm6dso_trig_counter_bdr_t)reg.trig_counter_bdr;

  return mm_error;
}

/**
  * @brief  
  *         
  * @param  
  *                
  * @retval 
  */
int32_t lsm6dso_xl_odr_set(lsm6dso_ctx_t *ctx, lsm6dso_odr_xl_t val)
{
  lsm6dso_ctrl1_xl_t reg;
  int32_t mm_error;

  mm_error = lsm6dso_read_reg(ctx, LSM6DSO_CTRL1_XL, (uint8_t *)&reg, 1);
  reg.odr_xl = val;
  mm_error = lsm6dso_write_reg(ctx, LSM6DSO_CTRL1_XL, (uint8_t *)&reg, 1);

  return mm_error;
}

int32_t lsm6dso_xl_odr_get(lsm6dso_ctx_t *ctx, lsm6dso_odr_xl_t *val)
{
  lsm6dso_ctrl1_xl_t reg;
  int32_t mm_error;

  mm_error = lsm6dso_read_reg(ctx, LSM6DSO_CTRL1_XL, (uint8_t *)&reg, 1);
  *val = (lsm6dso_odr_xl_t)reg.odr_xl;

  return mm_error;
}

/**
  * @brief  
  *         
  * @param  
  *                
  * @retval 
  */
int32_t lsm6dso_xl_full_scale_set(lsm6dso_ctx_t *ctx, lsm6dso_fs_xl_t val)
{
  lsm6dso_ctrl1_xl_t reg;
  int32_t mm_error;

  mm_error = lsm6dso_read_reg(ctx, LSM6DSO_CTRL1_XL, (uint8_t *)&reg, 1);
  reg.fs_xl = val;
  mm_error = lsm6dso_write_reg(ctx, LSM6DSO_CTRL1_XL, (uint8_t *)&reg, 1);

  return mm_error;
}

int32_t lsm6dso_xl_full_scale_get(lsm6dso_ctx_t *ctx, lsm6dso_fs_xl_t *val)
{
  lsm6dso_ctrl1_xl_t reg;
  int32_t mm_error;

  mm_error = lsm6dso_read_reg(ctx, LSM6DSO_CTRL1_XL, (uint8_t *)&reg, 1);
  *val = (lsm6dso_fs_xl_t)reg.fs_xl;

  return mm_error;
}



/**
  * @brief  
  *         
  * @param  
  *                
  * @retval 
  */
int32_t lsm6dso_gyro_odr_set(lsm6dso_ctx_t *ctx, lsm6dso_odr_g_t val)
{
  lsm6dso_ctrl2_g_t reg;
  int32_t mm_error;

  mm_error = lsm6dso_read_reg(ctx, LSM6DSO_CTRL2_G, (uint8_t *)&reg, 1);
  reg.odr_g = val;
  mm_error = lsm6dso_write_reg(ctx, LSM6DSO_CTRL2_G, (uint8_t *)&reg, 1);

  return mm_error;
  }

int32_t lsm6dso_gyro_odr_get(lsm6dso_ctx_t *ctx, lsm6dso_odr_g_t *val)
{
  lsm6dso_ctrl2_g_t reg;
  int32_t mm_error;

  mm_error = lsm6dso_read_reg(ctx, LSM6DSO_CTRL2_G, (uint8_t *)&reg, 1);
  *val = (lsm6dso_odr_g_t)reg.odr_g;

  return mm_error;
}


/**
  * @brief  
  *         
  * @param  
  *                
  * @retval 
  */
int32_t lsm6dso_gyro_full_scale_set(lsm6dso_ctx_t *ctx, lsm6dso_fs_g_t val)
{
  lsm6dso_ctrl2_g_t reg;
  int32_t mm_error;

  mm_error = lsm6dso_read_reg(ctx, LSM6DSO_CTRL2_G, (uint8_t *)&reg, 1);
  reg.fs_g = val;
  mm_error = lsm6dso_write_reg(ctx, LSM6DSO_CTRL2_G, (uint8_t *)&reg, 1);

  return mm_error;
  }

int32_t lsm6dso_gyro_full_scale_get(lsm6dso_ctx_t *ctx, lsm6dso_fs_g_t *val)
{
  lsm6dso_ctrl2_g_t reg;
  int32_t mm_error;

  mm_error = lsm6dso_read_reg(ctx, LSM6DSO_CTRL2_G, (uint8_t *)&reg, 1);
  *val = (lsm6dso_fs_g_t)reg.fs_g;

  return mm_error;
}

/**
  * @brief  
  *         
  * @param  
  *                
  * @retval 
  */
int32_t lsm6dso_sdo_pull_up_set(lsm6dso_ctx_t *ctx, uint8_t val)
{
  lsm6dso_sdo_crtl_t reg;
  int32_t mm_error;

  mm_error = lsm6dso_read_reg(ctx, LSM6DSO_SDO_CRTL, (uint8_t *)&reg, 1);
  reg.sdo_pu_en = val;
  mm_error = lsm6dso_write_reg(ctx, LSM6DSO_SDO_CRTL, (uint8_t *)&reg, 1);

  return mm_error;
}

int32_t lsm6dso_sdo_pull_up_get(lsm6dso_ctx_t *ctx, uint8_t *val)
{
  lsm6dso_sdo_crtl_t reg;
  int32_t mm_error;

  mm_error = lsm6dso_read_reg(ctx, LSM6DSO_SDO_CRTL, (uint8_t *)&reg, 1);
  *val = reg.sdo_pu_en;

  return mm_error;
}

/**
  * @brief  
  *         
  * @param  
  *                
  * @retval 
  */
int32_t lsm6dso_fifo_compression_set(lsm6dso_ctx_t *ctx, uint8_t val)
{
  lsm6dso_fifo_ctrl2_t reg;
  int32_t mm_error;

  mm_error = lsm6dso_read_reg(ctx, LSM6DSO_FIFO_CTRL2, (uint8_t *)&reg, 1);
  reg.fifo_compr_rt_en = val;
  mm_error = lsm6dso_write_reg(ctx, LSM6DSO_FIFO_CTRL2, (uint8_t *)&reg, 1);

  return mm_error;
}

int32_t lsm6dso_fifo_compression_get(lsm6dso_ctx_t *ctx, uint8_t *val)
{
  lsm6dso_fifo_ctrl2_t reg;
  int32_t mm_error;

  mm_error = lsm6dso_read_reg(ctx, LSM6DSO_FIFO_CTRL2, (uint8_t *)&reg, 1);
  *val = reg.fifo_compr_rt_en;

  return mm_error;
}


/**
  * @brief  
  *         
  * @param  
  *                
  * @retval 
  */
int32_t lsm6dso_fifo_stop_watermark_set(lsm6dso_ctx_t *ctx, uint8_t val)
{
  lsm6dso_fifo_ctrl2_t reg;
  int32_t mm_error;

  mm_error = lsm6dso_read_reg(ctx, LSM6DSO_FIFO_CTRL2, (uint8_t *)&reg, 1);
  reg.stop_on_wtm = val;
  mm_error = lsm6dso_write_reg(ctx, LSM6DSO_FIFO_CTRL2, (uint8_t *)&reg, 1);

  return mm_error;
}

int32_t lsm6dso_fifo_stop_watermark_get(lsm6dso_ctx_t *ctx, uint8_t *val)
{
  lsm6dso_fifo_ctrl2_t reg;
  int32_t mm_error;

  mm_error = lsm6dso_read_reg(ctx, LSM6DSO_FIFO_CTRL2, (uint8_t *)&reg, 1);
  *val = reg.stop_on_wtm;

  return mm_error;
}

/**
  * @brief  
  *         
  * @param  
  *                
  * @retval 
  */
int32_t lsm6dso_block_data_update_set(lsm6dso_ctx_t *ctx, uint8_t val)
{
  lsm6dso_ctrl3_c_t reg;
  int32_t mm_error;

  mm_error = lsm6dso_read_reg(ctx, LSM6DSO_FIFO_CTRL2, (uint8_t *)&reg, 1);
  reg.bdu = val;
  mm_error = lsm6dso_write_reg(ctx, LSM6DSO_FIFO_CTRL2, (uint8_t *)&reg, 1);

  return mm_error;
}

int32_t lsm6dso_block_data_update_get(lsm6dso_ctx_t *ctx, uint8_t *val)
{
  lsm6dso_ctrl3_c_t reg;
  int32_t mm_error;

  mm_error = lsm6dso_read_reg(ctx, LSM6DSO_FIFO_CTRL2, (uint8_t *)&reg, 1);
  *val = reg.bdu;

  return mm_error;
}

/**
  * @brief  
  *         
  * @param  
  *                
  * @retval 
  */
int32_t lsm6dso_int1_pin_conf_set(lsm6dso_ctx_t *ctx, lsm6dso_int1_ctrl_t val)
{
  int32_t mm_error;

  mm_error = lsm6dso_write_reg(ctx, LSM6DSO_INT1_CTRL, (uint8_t *)&val, 1);

  return mm_error;
}

int32_t lsm6dso_int1_pin_conf_get(lsm6dso_ctx_t *ctx, lsm6dso_int1_ctrl_t *val)
{
  int32_t mm_error;

  mm_error = lsm6dso_read_reg(ctx, LSM6DSO_INT1_CTRL, (uint8_t *)val, 1);

  return mm_error;
}


/**
  * @brief  
  *         
  * @param  
  *                
  * @retval 
  */
int32_t lsm6dso_int2_pin_conf_set(lsm6dso_ctx_t *ctx, lsm6dso_int2_ctrl_t val)
{
  int32_t mm_error;

  mm_error = lsm6dso_write_reg(ctx, LSM6DSO_INT2_CTRL, (uint8_t *)&val, 1);

  return mm_error;
}

int32_t lsm6dso_int2_pin_conf_get(lsm6dso_ctx_t *ctx, lsm6dso_int2_ctrl_t *val)
{
  int32_t mm_error;

  mm_error = lsm6dso_read_reg(ctx, LSM6DSO_INT2_CTRL, (uint8_t *)val, 1);

  return mm_error;
}

/**
  * @brief  
  *         
  * @param  
  *                
  * @retval 
  */
int32_t lsm6dso_status_get(lsm6dso_ctx_t *ctx, lsm6dso_status_reg_t *val)
{
  int32_t mm_error;

  mm_error = lsm6dso_read_reg(ctx, LSM6DSO_STATUS_REG, (uint8_t *)val, 1);

  return mm_error;
}

/**
  * @brief  
  *         
  * @param  
  *                
  * @retval 
  */
int32_t lsm6dso_acceleration_raw(lsm6dso_ctx_t *ctx, uint8_t *buff) 
{
  return lsm6dso_read_reg(ctx, LSM6DSO_OUTX_L_A, buff, 6);
}

/**
  * @brief  
  *         
  * @param  
  *                
  * @retval 
  */
int32_t lsm6dso_angular_rate_raw(lsm6dso_ctx_t *ctx, uint8_t *buff) 
{
  return lsm6dso_read_reg(ctx, LSM6DSO_OUTX_L_G, buff, 6);
}


















































