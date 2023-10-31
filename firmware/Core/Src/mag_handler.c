/*
 * mag_handler.c
 *
 *  Created on: Oct 20, 2023
 *      Author: Michael Fitzmayer
 */

#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "i2c.h"
#include "lis2mdl_reg.h"
#include "lsm6ds3tr-c_reg.h"
#include "mag_handler.h"
#include "main.h"
#include "motion_fx.h"
#include "stm32f1xx_hal.h"
#include "usart.h"
#include "usbd_custom_hid_if.h"

#define FROM_MGAUSS_TO_UT50 (0.1f/50.0f)
#define STATE_SIZE          (size_t)(2450)
#define CONFIG_FLASH_ADDR   0x0801fc00

typedef struct
{
  float hi_bias[3];

} __packed mag_config_t;

typedef struct
{
  /* I²C interface for LSM6DS3TR-C and LIS2MDL. */
  stmdev_ctx_t        ctx;

  /* MotionFX. */
  uint8_t             mfx[STATE_SIZE];
  MFX_knobs_t         knobs;
  MFX_input_t         data_in;
  MFX_output_t        data_out;
  MFX_MagCal_input_t  cal_data_in;
  MFX_MagCal_output_t cal_data_out;

  /* Raw acceleration rate data. */
  int16_t             a[3];

  /* Raw angular rate. */
  int16_t             g[3];

  /* Raw magnetometer data. */
  int16_t             m[3];

  uint8_t             sample_time_ms;
  float               sample_time_sec;
  uint8_t             i2c_addr;

  bool                calibration_mode;

} mag_t;

static mag_t hmag;

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static void    pass_through_enable(void);
static void    pass_through_disable(void);
static void    poll_data(void);
static void    config_read(__IO mag_config_t *config);
static void    config_write(mag_config_t *config);

void mag_update(mag_state_t* state)
{
  mag_config_t config      = { 0 };
  hid_report_t report      = { 0 };
  uint8_t      data_buffer = 0;
  uint8_t      x_axis      = 128;
  uint8_t      y_axis      = 128;

  switch (*state)
  {
    case MAG_INIT:
      memset(&hmag, 0, sizeof(mag_t));

      config_read(&config);

      hmag.cal_data_out.hi_bias[0] = config.hi_bias[0];
      hmag.cal_data_out.hi_bias[1] = config.hi_bias[1];
      hmag.cal_data_out.hi_bias[2] = config.hi_bias[2];

      hmag.sample_time_ms = 10;
      hmag.i2c_addr       = LSM6DS3TR_C_I2C_ADD_L;

      hmag.ctx.write_reg = platform_write;
      hmag.ctx.read_reg  = platform_read;
      hmag.ctx.handle    = &hi2c2;

      /* After the power supply is applied, the LSM6DS3TR-C performs a
       * 15ms boot procedure to load the trimming parameters.
       */
      HAL_Delay(15);

      /* Check if LSM6DS3TR-C is present. */
      lsm6ds3tr_c_device_id_get(&hmag.ctx, &data_buffer);
      if (LSM6DS3TR_C_ID != data_buffer)
      {
        /* Error: LSM6DS3TR-C not found. */
        return;
      }

      /* Restore default configuration. */
      lsm6ds3tr_c_reset_set(&hmag.ctx, PROPERTY_ENABLE);

      while(data_buffer)
      {
        lsm6ds3tr_c_reset_get(&hmag.ctx, &data_buffer);
      }

      /* Enable sensor hub pass-through feature. */
      pass_through_enable();

      /* Check if LIS2MDL is present. */
      lis2mdl_device_id_get(&hmag.ctx, &data_buffer);
      if (LIS2MDL_ID != data_buffer)
      {
        /* Error: LIS2MDL not found. */
        return;
      }

      /* Set operating mode. */
      lis2mdl_operating_mode_set(&hmag.ctx, LIS2MDL_CONTINUOUS_MODE);

      /* Set data rate. */
      lis2mdl_data_rate_set(&hmag.ctx, LIS2MDL_ODR_50Hz);

      /* Enable temperature compensation. */
      lis2mdl_offset_temp_comp_set(&hmag.ctx, 1);

      /* Enable Block Data Update. */
      lis2mdl_block_data_update_set(&hmag.ctx, 1);

      /* Select low-pass bandwidth. */
      lis2mdl_low_pass_bandwidth_set(&hmag.ctx, LIS2MDL_ODR_DIV_2);

      /* Enable DRDY pin. */
      lis2mdl_drdy_on_pin_set(&hmag.ctx, 1);

      /* Disable sensor hub pass-through feature. */
      pass_through_disable();

      /* Enable interrupts. */
      {
        lsm6ds3tr_c_int1_route_t int1_route = { 0 };

        int1_route.int1_drdy_xl = 1;
        int1_route.int1_drdy_g  = 1;

        lsm6ds3tr_c_pin_int1_route_set(&hmag.ctx, int1_route);
      }

      /* Enable Block Data Update.
       * Output registers not updated until MSB and LSB have been read.
       */
      lsm6ds3tr_c_block_data_update_set(&hmag.ctx, PROPERTY_ENABLE);

      /* Set full scale. */
      lsm6ds3tr_c_xl_full_scale_set(&hmag.ctx, LSM6DS3TR_C_2g);
      lsm6ds3tr_c_gy_full_scale_set(&hmag.ctx, LSM6DS3TR_C_2000dps);

      /* Configure filtering chain (no aux interface). */
      /* Accelerometer - analog filter */
      lsm6ds3tr_c_xl_filter_analog_set(&hmag.ctx, LSM6DS3TR_C_XL_ANA_BW_400Hz);
      /* Accelerometer - LPF1 + LPF2 path. */
      lsm6ds3tr_c_xl_lp2_bandwidth_set(&hmag.ctx, LSM6DS3TR_C_XL_LOW_NOISE_LP_ODR_DIV_400);
      /* Gyroscope - filtering chain. */
      lsm6ds3tr_c_gy_band_pass_set(&hmag.ctx, LSM6DS3TR_C_HP_260mHz_LP1_STRONG);

      /* Set Output Data Rate. */
      lsm6ds3tr_c_xl_data_rate_set(&hmag.ctx, LSM6DS3TR_C_XL_ODR_104Hz);
      lsm6ds3tr_c_gy_data_rate_set(&hmag.ctx, LSM6DS3TR_C_GY_ODR_104Hz);

      /* MotionFX. */
      MotionFX_initialize((MFXState_t *)hmag.mfx);

      MotionFX_enable_6X(hmag.mfx, MFX_ENGINE_DISABLE);
      MotionFX_enable_9X(hmag.mfx, MFX_ENGINE_ENABLE);

      MotionFX_getKnobs(hmag.mfx, &hmag.knobs);

      hmag.knobs.ATime                             = hmag.knobs.ATime;
      hmag.knobs.MTime                             = hmag.knobs.MTime;
      hmag.knobs.FrTime                            = hmag.knobs.FrTime;
      hmag.knobs.LMode                             = 0U;
      hmag.knobs.gbias_mag_th_sc                   = 0.00387913641f;
      hmag.knobs.gbias_acc_th_sc                   = 17.0277233f;
      hmag.knobs.gbias_gyro_th_sc                  = 0.00388081465f;
      hmag.knobs.modx                              = 1U;
      hmag.knobs.acc_orientation[0]                = 'e';
      hmag.knobs.acc_orientation[1]                = 'n';
      hmag.knobs.acc_orientation[2]                = 'u';
      hmag.knobs.gyro_orientation[0]               = 'e';
      hmag.knobs.gyro_orientation[1]               = 'n';
      hmag.knobs.gyro_orientation[2]               = 'u';
      hmag.knobs.mag_orientation[0]                = 'e';
      hmag.knobs.mag_orientation[1]                = 's';
      hmag.knobs.mag_orientation[2]                = 'u';
      hmag.knobs.output_type                       = MFX_ENGINE_OUTPUT_ENU;
      hmag.knobs.start_automatic_gbias_calculation = 0;

      MotionFX_setKnobs(hmag.mfx, &hmag.knobs);

      /* Done. */
      *state = MAG_RUN;

      break;
    case MAG_RUN:
      if (false == hmag.calibration_mode)
      {
        hmag.sample_time_ms  = 10; /* 100 Hz. */
        hmag.sample_time_sec = 0.01f;
      }
      else
      {
        hmag.sample_time_ms  = 40; /* 25 Hz. */
        hmag.sample_time_sec = 0.04f;
      }

      poll_data();

      MotionFX_propagate(hmag.mfx, &hmag.data_out, &hmag.data_in, &hmag.sample_time_sec);
      MotionFX_update(hmag.mfx, &hmag.data_out, &hmag.data_in, &hmag.sample_time_sec, NULL);

      break;
    default:
      break;
  }

  report.axis[0] = x_axis;
  report.axis[1] = y_axis;
  usb_send_report(&report);
  HAL_Delay(hmag.sample_time_ms);
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
  HAL_I2C_Mem_Read(handle, hmag.i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 10);
  return 0;
}

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
  HAL_I2C_Mem_Write(handle, hmag.i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 10);
  return 0;
}

static void pass_through_enable(void)
{
  lsm6ds3tr_c_sh_pass_through_set(&hmag.ctx, 1);
  hmag.i2c_addr = LIS2MDL_I2C_ADD;
}

static void pass_through_disable(void)
{
  hmag.i2c_addr = LSM6DS3TR_C_I2C_ADD_L;
  lsm6ds3tr_c_sh_pass_through_set(&hmag.ctx, 0);
}

static void poll_data(void)
{
  lis2mdl_status_reg_t     lis2mdl_status;
  lsm6ds3tr_c_status_reg_t lsm6ds3tr_status;

  lsm6ds3tr_c_status_reg_get(&hmag.ctx, &lsm6ds3tr_status);
  if (lsm6ds3tr_status.xlda)
  {
    lsm6ds3tr_c_acceleration_raw_get(&hmag.ctx, hmag.a);

    /* Acceleration in g */
    hmag.data_in.acc[0] = hmag.a[0];
    hmag.data_in.acc[1] = hmag.a[1];
    hmag.data_in.acc[2] = hmag.a[2];
  }

  if (lsm6ds3tr_status.gda)
  {
    lsm6ds3tr_c_angular_rate_raw_get(&hmag.ctx, hmag.g);

    /* Angular rate in dps */
    hmag.data_in.gyro[0] = hmag.g[0];
    hmag.data_in.gyro[1] = hmag.g[1];
    hmag.data_in.gyro[2] = hmag.g[2];
  }

  pass_through_enable();
  lis2mdl_status_get(&hmag.ctx, &lis2mdl_status);
  if (lis2mdl_status.xda)
  {
    lis2mdl_magnetic_raw_get(&hmag.ctx, hmag.m);

    /* Convert magnetic field data from LSB to mgauss to uT/50 (microtesla/50). */
    hmag.data_in.mag[0] = lis2mdl_from_lsb_to_mgauss(hmag.m[0]) * FROM_MGAUSS_TO_UT50;
    hmag.data_in.mag[1] = lis2mdl_from_lsb_to_mgauss(hmag.m[1]) * FROM_MGAUSS_TO_UT50;
    hmag.data_in.mag[2] = lis2mdl_from_lsb_to_mgauss(hmag.m[2]) * FROM_MGAUSS_TO_UT50;

    hmag.data_in.mag[0] -= hmag.cal_data_out.hi_bias[0];
    hmag.data_in.mag[1] -= hmag.cal_data_out.hi_bias[1];
    hmag.data_in.mag[2] -= hmag.cal_data_out.hi_bias[2];
  }

  if (true == hmag.calibration_mode)
  {
    hmag.cal_data_in.mag[0]      = hmag.data_in.mag[0];
    hmag.cal_data_in.mag[1]      = hmag.data_in.mag[1];
    hmag.cal_data_in.mag[2]      = hmag.data_in.mag[2];
    hmag.cal_data_in.time_stamp += hmag.sample_time_ms;

    MotionFX_MagCal_run(&hmag.cal_data_in);
    MotionFX_MagCal_getParams(&hmag.cal_data_out);
    if (MFX_MAGCALGOOD == hmag.cal_data_out.cal_quality)
    {
      mag_config_t config;

      hmag.calibration_mode = false;
      config.hi_bias[0]     = hmag.cal_data_out.hi_bias[0];
      config.hi_bias[1]     = hmag.cal_data_out.hi_bias[1];
      config.hi_bias[2]     = hmag.cal_data_out.hi_bias[2];

      config_write(&config);

      HAL_FLASH_Lock();
    }
  }

  pass_through_disable();
}

static void config_read(__IO mag_config_t *config)
{
  *config = *(__IO mag_config_t *)CONFIG_FLASH_ADDR;
}

static void config_write(mag_config_t *config)
{
  HAL_StatusTypeDef      status;
  FLASH_EraseInitTypeDef erase_init;
  uint32_t               page_error;

  HAL_FLASH_Unlock();

  erase_init.TypeErase   = FLASH_TYPEERASE_PAGES;
  erase_init.PageAddress = CONFIG_FLASH_ADDR;
  erase_init.NbPages     = 1;

  status = HAL_FLASHEx_Erase(&erase_init, &page_error);
  if (HAL_OK != status)
  {
    Error_Handler();
  }

  status = FLASH_WaitForLastOperation(100);
  if (HAL_OK != status)
  {
    Error_Handler();
  }

  for (int p = 0; p < 3; p += 1)
  {
    uint64_t data = 0;

    memcpy(&data, &config->hi_bias[p], sizeof(float));

    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, CONFIG_FLASH_ADDR + (p * sizeof(float)), data);
    status = FLASH_WaitForLastOperation(100);
    if (HAL_OK != status)
    {
      Error_Handler();
    }

    status = FLASH_WaitForLastOperation(100);
    if (HAL_OK != status)
    {
      Error_Handler();
    }
  }

  HAL_FLASH_Lock();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (CALIBRATE_Pin == GPIO_Pin)
  {
    if (false == hmag.calibration_mode)
    {
      MotionFX_MagCal_init(40, 1);
      hmag.calibration_mode = true;
    }
  }
}
