/*
 * mag_handler.c
 *
 *  Created on: Oct 20, 2023
 *      Author: Michael Fitzmayer
 *
 * TODO: Fix yaw axis, it is extremely unstable.
 */

#include <inttypes.h>
#include <math.h>
#include <stdint.h>
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

  /* Delta time calculation. */
  uint64_t            last_time;

  /* Raw acceleration rate data. */
  int16_t             a[3];

  /* Raw angular rate. */
  int16_t             g[3];

  /* Raw magnetometer data. */
  int16_t             m[3];

  int16_t             yaw;
  int16_t             pitch;
  int16_t             roll;

  uint8_t             sample_time;
  uint8_t             i2c_addr;

} mag_t;

static mag_t hmag;

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static void    pass_through_enable(void);
static void    pass_through_disable(void);
static void    poll_data(void);

void mag_update(mag_state_t* state)
{
  hid_report_t report       = { 0 };
  uint8_t      data_buffer  = 0;
  uint8_t      x_axis       = 128;
  uint8_t      y_axis       = 128;

  switch (*state)
  {
    case MAG_INIT:
      memset(&hmag, 0, sizeof(mag_t));

      hmag.last_time   = HAL_GetTick();
      hmag.sample_time = 10;
      hmag.i2c_addr    = LSM6DS3TR_C_I2C_ADD_L;

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
      lis2mdl_data_rate_set(&hmag.ctx, LIS2MDL_ODR_20Hz);

      /* Enable temperature compensation. */
      lis2mdl_offset_temp_comp_set(&hmag.ctx, 1);

      /* Enable Block Data Update. */
      lis2mdl_block_data_update_set(&hmag.ctx, 1);

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
      hmag.knobs.LMode                             = 2U;
      hmag.knobs.gbias_mag_th_sc                   = 0.00079999998;
      hmag.knobs.gbias_acc_th_sc                   = 0.000699999975;
      hmag.knobs.gbias_gyro_th_sc                  = 0.0027999999;
      hmag.knobs.modx                              = 2U;
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
      hmag.knobs.start_automatic_gbias_calculation = 1;

      MotionFX_setKnobs(hmag.mfx, &hmag.knobs);

      /* Done. */
      *state = MAG_RUN;

      break;
    case MAG_RUN:
      hmag.sample_time = 10; /* 100 Hz. */
      poll_data();

      {
        uint32_t current_time = HAL_GetTick();
        float    delta_time   = (current_time / 1000.f) - (hmag.last_time / 1000.f);
        MotionFX_propagate(hmag.mfx, &hmag.data_out, &hmag.data_in, &delta_time);
        hmag.last_time = HAL_GetTick();
      }

      hmag.yaw   = (int16_t)hmag.data_out.rotation[0];
      hmag.pitch = (int16_t)hmag.data_out.rotation[1];
      hmag.roll  = (int16_t)hmag.data_out.rotation[2];

      /* Todo. */

      break;
    case MAG_CALIBRATION:
      hmag.sample_time = 40; /* 25 Hz. */
      poll_data();

      /* Todo. */

      break;
    default:
      break;
  }

  report.axis[0] = x_axis;
  report.axis[1] = y_axis;
  usb_send_report(&report);
  HAL_Delay(hmag.sample_time);
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
  HAL_I2C_Mem_Read(handle, hmag.i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 100);
  return 0;
}

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
  HAL_I2C_Mem_Write(handle, hmag.i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 100);
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
  }

  pass_through_disable();
}

/* DRDY does not work yet, why? */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (CALIBRATE_Pin == GPIO_Pin)
  {
    MotionFX_MagCal_init(40, 1);
  }
}
