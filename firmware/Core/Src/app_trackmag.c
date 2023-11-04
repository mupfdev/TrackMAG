/*
 * app_trackmag.c
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
#include "app_trackmag.h"
#include "i2c.h"
#include "lis2mdl_reg.h"
#include "lsm6ds3tr-c_reg.h"
#include "main.h"
#include "motion_fx.h"
#include "stm32f1xx_hal.h"
#include "usart.h"
#include "usbd_custom_hid_if.h"

#define AXIS_CENTER                  128u
#define CALIB_MODE_PRESS_DURATION_MS 3000u
#define CONFIG_FLASH_ADDR            0x0801fc00
#define FROM_MDPS_TO_DPS             0.001f
#define FROM_MG_TO_G                 0.001f
#define FROM_MGAUSS_TO_UT50          (0.1f/50.0f)
#define FROM_UT50_TO_MGAUSS          500.0f
#define STATE_SIZE                   (size_t)(2450)

typedef struct
{
  float hi_bias[3];

} __packed app_config_t;

typedef struct
{
  /* Raw acceleration rate data. */
  int16_t             a[3];

  /* Raw angular rate. */
  int16_t             g[3];

  /* Raw magnetometer data. */
  int16_t             m[3];
  float               m_offset[3];

  /* I²C interface for LSM6DS3TR-C and LIS2MDL. */
  stmdev_ctx_t        ctx;

  /* MotionFX. */
  uint8_t             mfx[STATE_SIZE];
  MFX_knobs_t         knobs;
  MFX_input_t         data_in;
  MFX_output_t        data_out;
  MFX_MagCal_input_t  cal_data_in;
  MFX_MagCal_output_t cal_data_out;

  /* Calibration mode. */
  bool                calibration_mode;
  bool                start_calib_press_timer;
  uint32_t            calib_press_timer;
  uint32_t            calib_press_timer_init;

  /* Axis calculation. */
  uint32_t            view_direction;

  /* Miscellaneous. */
  uint8_t             sample_time_ms;
  float               sample_time_sec;
  uint8_t             i2c_addr;

} app_t;

static app_t happ;
uint8_t      debug;

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static void    pass_through_enable(void);
static void    pass_through_disable(void);
static void    accel_gyro_data_read(void);
static void    magnetometer_data_read(void);
static void    config_read(__IO app_config_t *config);
static void    config_write(void);
static uint8_t transform_delta_angle(float angle, float limit);

void app_update(app_state_t* state)
{
  app_config_t config      = { 0 };
  hid_report_t report      = { 0 };
  uint8_t      data_buffer = 0;
  uint8_t      x_axis      = AXIS_CENTER;
  uint8_t      y_axis      = AXIS_CENTER;

  switch (*state)
  {
    case APP_INIT:
      memset(&happ, 0, sizeof(app_t));

      config_read(&config);

      happ.m_offset[0] = isnan(config.hi_bias[0]) ? 0.f : config.hi_bias[0];
      happ.m_offset[1] = isnan(config.hi_bias[1]) ? 0.f : config.hi_bias[1];
      happ.m_offset[2] = isnan(config.hi_bias[2]) ? 0.f : config.hi_bias[2];

      happ.sample_time_ms = 10;
      happ.i2c_addr       = LSM6DS3TR_C_I2C_ADD_L;
      happ.ctx.write_reg  = platform_write;
      happ.ctx.read_reg   = platform_read;
      happ.ctx.handle     = &hi2c2;

      /* After the power supply is applied, the LSM6DS3TR-C performs a
       * 15ms boot procedure to load the trimming parameters.
       */
      HAL_Delay(15);

      /* Check if LSM6DS3TR-C is present. */
      lsm6ds3tr_c_device_id_get(&happ.ctx, &data_buffer);
      if (LSM6DS3TR_C_ID != data_buffer)
      {
        /* Error: LSM6DS3TR-C not found. */
        return;
      }

      /* Restore default configuration. */
      lsm6ds3tr_c_reset_set(&happ.ctx, PROPERTY_ENABLE);

      while(data_buffer)
      {
        lsm6ds3tr_c_reset_get(&happ.ctx, &data_buffer);
      }

      /* Enable sensor hub pass-through feature. */
      pass_through_enable();

      /* Check if LIS2MDL is present. */
      lis2mdl_device_id_get(&happ.ctx, &data_buffer);
      if (LIS2MDL_ID != data_buffer)
      {
        /* Error: LIS2MDL not found. */
        return;
      }

      /* Set operating mode. */
      lis2mdl_operating_mode_set(&happ.ctx, LIS2MDL_CONTINUOUS_MODE);

      /* Set data rate. */
      lis2mdl_data_rate_set(&happ.ctx, LIS2MDL_ODR_100Hz);

      /* Enable temperature compensation. */
      lis2mdl_offset_temp_comp_set(&happ.ctx, PROPERTY_ENABLE);

      /* Enable Block Data Update. */
      lis2mdl_block_data_update_set(&happ.ctx, PROPERTY_ENABLE);

      /* Select low-pass bandwidth. */
      lis2mdl_low_pass_bandwidth_set(&happ.ctx, LIS2MDL_ODR_DIV_2);

      /* Enable DRDY pin. */
      lis2mdl_drdy_on_pin_set(&happ.ctx, PROPERTY_ENABLE);

      /* Disable sensor hub pass-through feature. */
      pass_through_disable();

      /* Enable interrupts. */
      {
        lsm6ds3tr_c_int1_route_t int1_route = { 0 };

        int1_route.int1_drdy_xl = PROPERTY_ENABLE;
        int1_route.int1_drdy_g  = PROPERTY_ENABLE;

        lsm6ds3tr_c_pin_int1_route_set(&happ.ctx, int1_route);
      }

      /* Enable Block Data Update.
       * Output registers not updated until MSB and LSB have been read.
       */
      lsm6ds3tr_c_block_data_update_set(&happ.ctx, PROPERTY_ENABLE);

      /* Set full scale. */
      lsm6ds3tr_c_xl_full_scale_set(&happ.ctx, LSM6DS3TR_C_4g);
      lsm6ds3tr_c_gy_full_scale_set(&happ.ctx, LSM6DS3TR_C_2000dps);

      /* Set Output Data Rate. */
      lsm6ds3tr_c_xl_data_rate_set(&happ.ctx, LSM6DS3TR_C_XL_ODR_104Hz);
      lsm6ds3tr_c_gy_data_rate_set(&happ.ctx, LSM6DS3TR_C_XL_ODR_104Hz);

      /* MotionFX. */
      MotionFX_initialize((MFXState_t *)happ.mfx);

      MotionFX_enable_6X(happ.mfx, MFX_ENGINE_DISABLE);
      MotionFX_enable_9X(happ.mfx, MFX_ENGINE_ENABLE);

      MotionFX_getKnobs(happ.mfx, &happ.knobs);

      happ.knobs.ATime                             = happ.knobs.ATime;
      happ.knobs.MTime                             = happ.knobs.MTime;
      happ.knobs.FrTime                            = happ.knobs.FrTime;
      happ.knobs.LMode                             = 0U;
      happ.knobs.gbias_mag_th_sc                   = 0.00393244764f;
      happ.knobs.gbias_acc_th_sc                   = 0.000679513789f;
      happ.knobs.gbias_gyro_th_sc                  = 0.000155091839f;
      happ.knobs.modx                              = 1U;
      happ.knobs.acc_orientation[0]                = 'e';
      happ.knobs.acc_orientation[1]                = 'n';
      happ.knobs.acc_orientation[2]                = 'u';
      happ.knobs.gyro_orientation[0]               = 'e';
      happ.knobs.gyro_orientation[1]               = 'n';
      happ.knobs.gyro_orientation[2]               = 'u';
      happ.knobs.mag_orientation[0]                = 'e';
      happ.knobs.mag_orientation[1]                = 's';
      happ.knobs.mag_orientation[2]                = 'u';
      happ.knobs.output_type                       = MFX_ENGINE_OUTPUT_ENU;
      happ.knobs.start_automatic_gbias_calculation = 0;

      MotionFX_setKnobs(happ.mfx, &happ.knobs);

      HAL_Delay(50);

      accel_gyro_data_read();
      magnetometer_data_read();

      /* Done. */
      *state = APP_RUN;

      break;
    case APP_RUN:
#if defined (DEBUG)
      if (1 == happ.knobs.start_automatic_gbias_calculation)
      {
        if (HAL_GetTick() > 10000)
        {
          MotionFX_getKnobs(happ.mfx, &happ.knobs);
          asm("NOP");
        }
      }
#endif

      if (true == happ.start_calib_press_timer)
      {
        happ.calib_press_timer = HAL_GetTick() - happ.calib_press_timer_init;
      }

      if (false == happ.calibration_mode)
      {
        happ.sample_time_ms  = 10; /* 100 Hz. */
        happ.sample_time_sec = 0.01f;
      }
      else
      {
        happ.sample_time_ms  = 40; /* 25 Hz. */
        happ.sample_time_sec = 0.04f;
      }

      if (happ.calib_press_timer >= CALIB_MODE_PRESS_DURATION_MS)
      {
        if (false == happ.calibration_mode)
        {
          MotionFX_MagCal_init(happ.sample_time_ms, 1);
          happ.calibration_mode = true;
        }
      }

      if (true == happ.calibration_mode)
      {
        happ.cal_data_in.mag[0] = lis2mdl_from_lsb_to_mgauss(happ.m[0]) * FROM_MGAUSS_TO_UT50;
        happ.cal_data_in.mag[1] = lis2mdl_from_lsb_to_mgauss(happ.m[1]) * FROM_MGAUSS_TO_UT50;
        happ.cal_data_in.mag[2] = lis2mdl_from_lsb_to_mgauss(happ.m[2]) * FROM_MGAUSS_TO_UT50;
        happ.cal_data_in.time_stamp += happ.sample_time_ms;

        MotionFX_MagCal_run(&happ.cal_data_in);
        MotionFX_MagCal_getParams(&happ.cal_data_out);

        if (MFX_MAGCALGOOD == happ.cal_data_out.cal_quality)
        {
          happ.calibration_mode = false;
          MotionFX_MagCal_init(40, 0);
          config_write();
        }
      }

      MotionFX_propagate(happ.mfx, &happ.data_out, &happ.data_in, &happ.sample_time_sec);
      MotionFX_update(happ.mfx, &happ.data_out, &happ.data_in, &happ.sample_time_sec, NULL);

      if (true == happ.calibration_mode)
      {
        x_axis = AXIS_CENTER;
        y_axis = AXIS_CENTER;
      }
      else
      {
        int16_t yaw_delta = (360 - happ.view_direction + (int)happ.data_out.heading + 180) % 360 - 180;

        debug = transform_delta_angle(yaw_delta, 220.f);
        //float yaw_normalised = (yaw_delta - (-180.f)) / (360.f) * (2.f) + (-1.f); /* -1.f to 1.f */
        //debug = transform_normalised_angle(yaw_normalised, 1.5f);
      }

      {
        char output[30] = { 0 };
        snprintf(output, 30, "%.2f,%.2f,%.2f\r\n",
            happ.data_out.heading,
            happ.data_out.rotation[1],
            happ.data_out.rotation[2]);
        HAL_UART_Transmit_DMA(&huart2, (const uint8_t*)output, strnlen((const char*)output, 30));
      }
      break;
    default:
      break;
  }

  report.axis[0] = x_axis;
  report.axis[1] = y_axis;
  usb_send_report(&report);
  HAL_Delay(happ.sample_time_ms);
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
  HAL_I2C_Mem_Read(handle, happ.i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 10);
  return 0;
}

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
  HAL_I2C_Mem_Write(handle, happ.i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 10);
  return 0;
}

static void pass_through_enable(void)
{
  lsm6ds3tr_c_sh_pass_through_set(&happ.ctx, 1);
  happ.i2c_addr = LIS2MDL_I2C_ADD;
}

static void pass_through_disable(void)
{
  happ.i2c_addr = LSM6DS3TR_C_I2C_ADD_L;
  lsm6ds3tr_c_sh_pass_through_set(&happ.ctx, 0);
}

static void accel_gyro_data_read(void)
{
  lsm6ds3tr_c_acceleration_raw_get(&happ.ctx, happ.a);

  /* Acceleration in g */
  happ.data_in.acc[0] = lsm6ds3tr_c_from_fs4g_to_mg(happ.a[0]) * FROM_MG_TO_G;
  happ.data_in.acc[1] = lsm6ds3tr_c_from_fs4g_to_mg(happ.a[1]) * FROM_MG_TO_G;
  happ.data_in.acc[2] = lsm6ds3tr_c_from_fs4g_to_mg(happ.a[2]) * FROM_MG_TO_G;

  lsm6ds3tr_c_angular_rate_raw_get(&happ.ctx, happ.g);

  /* Angular rate in dps */
  happ.data_in.gyro[0] = lsm6ds3tr_c_from_fs2000dps_to_mdps(happ.g[0]) * FROM_MDPS_TO_DPS;
  happ.data_in.gyro[1] = lsm6ds3tr_c_from_fs2000dps_to_mdps(happ.g[1]) * FROM_MDPS_TO_DPS;
  happ.data_in.gyro[2] = lsm6ds3tr_c_from_fs2000dps_to_mdps(happ.g[2]) * FROM_MDPS_TO_DPS;
}

static void magnetometer_data_read(void)
{
  pass_through_enable();
  lis2mdl_magnetic_raw_get(&happ.ctx, happ.m);

  happ.data_in.mag[0] = lis2mdl_from_lsb_to_mgauss(happ.m[0] - happ.m_offset[0]) * FROM_MGAUSS_TO_UT50;
  happ.data_in.mag[1] = lis2mdl_from_lsb_to_mgauss(happ.m[1] - happ.m_offset[1]) * FROM_MGAUSS_TO_UT50;
  happ.data_in.mag[2] = lis2mdl_from_lsb_to_mgauss(happ.m[2] - happ.m_offset[2]) * FROM_MGAUSS_TO_UT50;

  pass_through_disable();
}

static void config_read(__IO app_config_t *config)
{
  *config = *(__IO app_config_t *)CONFIG_FLASH_ADDR;
}

static void config_write(void)
{
  HAL_StatusTypeDef      status;
  FLASH_EraseInitTypeDef erase_init;
  uint32_t               page_error;
  app_config_t           config;

  HAL_FLASH_Unlock();

  config.hi_bias[0]      = happ.cal_data_out.hi_bias[0];
  config.hi_bias[1]      = happ.cal_data_out.hi_bias[1];
  config.hi_bias[2]      = happ.cal_data_out.hi_bias[2];
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

    memcpy(&data, &config.hi_bias[p], sizeof(float));

    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, CONFIG_FLASH_ADDR + (p * sizeof(float)), data);
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

static uint8_t transform_delta_angle(float angle, float limit)
{
  float interim = angle / 180.f * limit;
  float result  = ((interim / limit) + 1) * 128.f;

  if (result < 0.f)
  {
    result = 0.f;
  }
  else if (result >= 255.f)
  {
    result = 255.f;
  }

  return (uint8_t)result;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
    case CALIBRATE_Pin:
      if (HAL_GPIO_ReadPin(CALIBRATE_GPIO_Port, CALIBRATE_Pin))
      {
        happ.start_calib_press_timer = false;
        happ.calib_press_timer       = 0;
      }
      else
      {
        happ.view_direction          = (uint32_t)happ.data_out.heading;
        happ.start_calib_press_timer = true;
        happ.calib_press_timer_init  = HAL_GetTick();
      }
      break;
    case DRDY_Pin:
      magnetometer_data_read();
      break;
    case INT1_Pin:
      accel_gyro_data_read();
      break;
    default:
      break;
  }
}
