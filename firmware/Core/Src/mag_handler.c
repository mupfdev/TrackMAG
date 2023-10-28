/*
 * mag_handler.c
 *
 *  Created on: Oct 20, 2023
 *      Author: Michael Fitzmayer
 *
 *  Todo: Fix interrupts, eliminate polling.
 */

#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "i2c.h"
#include "lis2mdl_reg.h"
#include "lsm6ds3tr-c_reg.h"
#include "mag_handler.h"
#include "main.h"
#include "motion_fx.h"
#include "motion_fx_cm0p.h"
#include "stm32f1xx_hal.h"
#include "usart.h"
#include "usbd_custom_hid_if.h"

#define STATE_SIZE (size_t)(2450)

typedef struct
{
  /* Raw acceleration rate data */
  int16_t a[3];

  /* Raw angular rate. */
  int16_t g[3];

  /* Raw magnetometer data. */
  int16_t m[3];

  /* The magnetometer calibrated data,
   * with both hard-iron and soft-iron correction.
   *
   */
  int16_t mc[3];

  int16_t yaw;
  int16_t pitch;
  int16_t roll;

} sen_t;

static uint32_t            time_a;
static uint8_t             mfx[STATE_SIZE];
static MFX_knobs_t         knobs;
static MFX_input_t         data_in          = { 0 };
static MFX_output_t        data_out         = { 0 };
static MFX_MagCal_input_t  cal_data_in      = { 0 };
static MFX_MagCal_output_t cal_data_out     = { 0 };
static bool                calibration_mode = false;


static stmdev_ctx_t dev_ctx;
static sen_t        sen;

static void    update_data(void);
static uint8_t read_reg(uint8_t reg);
static void    write_reg(uint8_t reg, uint8_t data);

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
  HAL_I2C_Mem_Write(handle, LSM6DS3TR_C_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 100);
  return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
  HAL_I2C_Mem_Read(handle, LSM6DS3TR_C_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 100);
  return 0;
}

void init_mag(void)
{
  uint8_t data_buffer = 0;

  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg  = platform_read;
  dev_ctx.handle    = &hi2c2;

  /* After the power supply is applied, the LSM6DS3TR-C performs a
   * 15ms boot procedure to load the trimming parameters.
   */
  HAL_Delay(15);

  /* Check if LSM6DS3TR-C is present. */
  lsm6ds3tr_c_device_id_get(&dev_ctx, &data_buffer);
  if (LSM6DS3TR_C_ID != data_buffer)
  {
    /* Error: LSM6DS3TR-C not found. */
    Error_Handler();
  }

  /* Restore default configuration. */
  lsm6ds3tr_c_reset_set(&dev_ctx, PROPERTY_ENABLE);

  while(data_buffer)
  {
    lsm6ds3tr_c_reset_get(&dev_ctx, &data_buffer);
  }

  /* Enable LSM6DS3TR-C high performance mode. */
  /* Enable Block Data Update */
  lsm6ds3tr_c_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

  /* Set Output Data Rate */
  lsm6ds3tr_c_xl_data_rate_set(&dev_ctx, LSM6DS3TR_C_XL_ODR_12Hz5);
  lsm6ds3tr_c_gy_data_rate_set(&dev_ctx, LSM6DS3TR_C_GY_ODR_12Hz5);

  /* Set full scale */
  lsm6ds3tr_c_xl_full_scale_set(&dev_ctx, LSM6DS3TR_C_2g);
  lsm6ds3tr_c_gy_full_scale_set(&dev_ctx, LSM6DS3TR_C_2000dps);

  /* Configure filtering chain(No aux interface) */
  /* Accelerometer - analog filter */
  lsm6ds3tr_c_xl_filter_analog_set(&dev_ctx, LSM6DS3TR_C_XL_ANA_BW_400Hz);
  /* Accelerometer - LPF1 + LPF2 path */
  lsm6ds3tr_c_xl_lp2_bandwidth_set(&dev_ctx, LSM6DS3TR_C_XL_LOW_NOISE_LP_ODR_DIV_100);
  /* Gyroscope - filtering chain */
  lsm6ds3tr_c_gy_band_pass_set(&dev_ctx, LSM6DS3TR_C_HP_260mHz_LP1_STRONG);

  /* magnetometer hard-iron / soft-iron correction.
   * Reference implementation, AN5130, page 72-73.
   *
   * 1. Write 80h to FUNC_CFG_ACCESS
   *    Enable access to embedded functions registers (bank A)
   */
  write_reg(LSM6DS3TR_C_FUNC_CFG_ACCESS, 0x80);

  /* 2. Write 3Ch to SLV0_ADD
   *    LIS2MDL slave address = 0011110b
   *    Enable write operation (rw_0=0)
   */
  write_reg(LSM6DS3TR_C_SLV0_ADD, 0x3c);

  /* 3. Write 60h to SLV0_SUBADD
   *    60h is the LIS2MDL register to be written
   */
  write_reg(LSM6DS3TR_C_SLV0_SUBADD, 0x60);

  /* 4. Write 8Ch to DATAWRITE_SRC_MODE_SUB_SLV0
   *    8Ch is the value to be written in register 60h of
   *    LIS2MDL to configure it in continuous mode,
   *    ODR = 100 Hz, temperature compensation enabled
   */
  write_reg(LSM6DS3TR_C_DATAWRITE_SRC_MODE_SUB_SLV0, 0x8c);

  /* 5. Write 10h to SLAVE0_CONFIG
   *    Set Aux_sens_on bits different from 00b
   */
  write_reg(LSM6DS3TR_C_SLAVE0_CONFIG, 0x10);

  /* 6. Write 20h to SLAVE1_CONFIG
   *    Enable write_once bit
   */
  write_reg(LSM6DS3TR_C_SLAVE1_CONFIG, 0x20);

  /* 7. Write 00h to FUNC_CFG_ACCESS
   *    Disable access to embedded functions registers (bank A)
   */
  write_reg(LSM6DS3TR_C_FUNC_CFG_ACCESS, 0x00);

  /* 8. Write 04h to CTRL10_C
   *    Enable embedded functions
   */
  write_reg(LSM6DS3TR_C_CTRL10_C, 0x04);

write_master_config:
  /* 9. Write 09h to MASTER_CONFIG
   *    Enable internal pull-up on SDx/SCx lines
   *    Sensor hub trigger signal is XL Data Ready
   *    Enable auxiliary I2C master
   */
  write_reg(LSM6DS3TR_C_MASTER_CONFIG, 0x09);

  /* 10. Write 80h to CTRL1_XL
   *     Turn on the accelerometer (for trigger signal)
   */
  write_reg(LSM6DS3TR_C_CTRL1_XL, 0x80);

  /* 11. Read FUNC_SRC1
   *     Wait for the sensor hub communication concluded
   *
   * 12. If SENSORHUB_END_OP = 0, go to 9
   */
  if (0 == (read_reg(LSM6DS3TR_C_FUNC_SRC1) & (1 << 0)))
  {
    /* Yes, I know but this is intentional. */
    goto write_master_config;
  }

  /* 13. Write 00h to CTRL10_C
   *     Disable embedded functions
   */
  write_reg(LSM6DS3TR_C_CTRL10_C, 0x00);

  /* 14. Write 00h to MASTER_CONFIG
   *     Disable auxiliary I2C master
   */
  write_reg(LSM6DS3TR_C_MASTER_CONFIG, 0x00);

  /* 15. Write 00h to CTRL1_XL
   *     Turn off the accelerometer
   */
  write_reg(LSM6DS3TR_C_CTRL1_XL, 0x00);

  /* 16. Write 80h to FUNC_CFG_ACCESS
   *     Enable access to embedded functions registers (bank A)
   */
  write_reg(LSM6DS3TR_C_FUNC_CFG_ACCESS, 0x80);

  /* 17. Write 3Dh to SLV0_ADD
   *     LIS2MDL slave address = 0011110b
   *     Enable read operation (rw_0=1)
   */
  write_reg(LSM6DS3TR_C_SLV0_ADD, 0x3d);

  /* 18. Write 68h to SLV0_SUBADD
   *     68h is the first LIS2MDL output register to be read
   */
  write_reg(LSM6DS3TR_C_SLV0_SUBADD, 0x68);

  /* 19. Write 06h to SLAVE0_CONFIG
   *     No decimation
   *     1 external sensor connected
   *     Number of registers to read = 6
   */
  write_reg(LSM6DS3TR_C_SLAVE0_CONFIG, 0x06);

  /* 20. Write FFh to MAG_OFFX_H
   *     X offset value initialization
   */
  write_reg(LSM6DS3TR_C_MAG_OFFX_H, 0xff);

  /* 21. Write 20h to MAG_OFFX_L
   *     X offset value initialization
   */
  write_reg(LSM6DS3TR_C_MAG_OFFX_L, 0x20);

  /* 22. Write 00h to MAG_OFFY_H
   *     Y offset value initialization
   */
  write_reg(LSM6DS3TR_C_MAG_OFFY_H, 0x00);

  /* 23. Write 54h to MAG_OFFY_L
   *     Y offset value initialization
   */
  write_reg(LSM6DS3TR_C_MAG_OFFY_L, 0x54);

  /* 24. Write FFh to MAG_OFFZ_H
   *     Z offset value initialization
   */
  write_reg(LSM6DS3TR_C_MAG_OFFZ_H, 0xff);

  /* 25. Write B4h to MAG_OFFZ_L
   *     Z offset value initialization
   */
  write_reg(LSM6DS3TR_C_MAG_OFFZ_L, 0xb4);

  /* 26. Write 0Ah to MAG_SI_XX
   *     XX soft-iron element
   */
  write_reg(LSM6DS3TR_C_MAG_SI_XX, 0x0a);

  /* 27. Write 01h to MAG_SI_XY
   *     XY soft-iron element
   */
  write_reg(LSM6DS3TR_C_MAG_SI_XY, 0x01);

  /* 28. Write 00h to MAG_SI_XZ
   *     XZ soft-iron element
   */
  write_reg(LSM6DS3TR_C_MAG_SI_XZ, 0x00);

  /* 29. Write 01h to MAG_SI_YX
   *     YX soft-iron element
   */
  write_reg(LSM6DS3TR_C_MAG_SI_YX, 0x01);

  /* 30. Write 08h to MAG_SI_YY
   *     YY soft-iron element
   */
  write_reg(LSM6DS3TR_C_MAG_SI_YY, 0x08);

  /* 31. Write 81h to MAG_SI_YZ
   *     YZ soft-iron element
   */
  write_reg(LSM6DS3TR_C_MAG_SI_YZ, 0x81);

  /* 32. Write 00h to MAG_SI_ZX
   *     ZX soft-iron element
   */
  write_reg(LSM6DS3TR_C_MAG_SI_ZX, 0x00);

  /* 33. Write 81h to MAG_SI_ZY
   *     ZY soft-iron element
   */
  write_reg(LSM6DS3TR_C_MAG_SI_ZY, 0x81);

  /* 34. Write 0Ah to MAG_SI_ZZ
   *     ZZ soft-iron element
   */
  write_reg(LSM6DS3TR_C_MAG_SI_ZZ, 0x0a);

  /* 35. Write 00h to FUNC_CFG_ACCESS
   *     Disable access to embedded functions registers (bank A)
   */
  write_reg(LSM6DS3TR_C_FUNC_CFG_ACCESS, 0x00);

  /* 36. Write 04h to CTRL10_C
   *     Enable embedded functions
   */
  write_reg(LSM6DS3TR_C_CTRL10_C, 0x04);

  /* 37. Write 0Bh to MASTER_CONFIG
   *     Enable internal pull-up on SDx/SCx lines
   *     Sensor hub trigger signal is XL data-ready
   *     Enable hard-iron correction
   *     Enable auxiliary I2C master
   */
  write_reg(LSM6DS3TR_C_MASTER_CONFIG, 0x0b);

  /* 38. Write 04h to CTRL9_XL
   *     Enable soft-iron correction
   */
  write_reg(LSM6DS3TR_C_CTRL9_XL, 0x04);

  /* 39. Write 80h to CTRL1_XL
   *     Turn on the accelerometer (for trigger signal)
   */
  write_reg(LSM6DS3TR_C_CTRL1_XL, 0x80);

  MotionFX_initialize((MFXState_t *)mfx);
  MotionFX_getKnobs(mfx, &knobs);

  MotionFX_setKnobs(mfx, &knobs);
  MotionFX_enable_6X(mfx, MFX_ENGINE_DISABLE);
  MotionFX_enable_9X(mfx, MFX_ENGINE_ENABLE);

  knobs.ATime                             = knobs.ATime;
  knobs.MTime                             = knobs.MTime;
  knobs.FrTime                            = knobs.FrTime;
  knobs.LMode                             = 2U;
  knobs.gbias_mag_th_sc                   = 1.11181867;
  knobs.gbias_acc_th_sc                   = 2.45545983;
  knobs.gbias_gyro_th_sc                  = 0.0115696611;
  knobs.modx                              = 2U;
  knobs.acc_orientation[0]                = 'e';
  knobs.acc_orientation[1]                = 'n';
  knobs.acc_orientation[2]                = 'u';
  knobs.gyro_orientation[0]               = 'e';
  knobs.gyro_orientation[1]               = 'n';
  knobs.gyro_orientation[2]               = 'u';
  knobs.mag_orientation[0]                = 'e';
  knobs.mag_orientation[1]                = 's';
  knobs.mag_orientation[2]                = 'u';
  knobs.output_type                       = MFX_ENGINE_OUTPUT_ENU;
  knobs.start_automatic_gbias_calculation = 0;

  MotionFX_setKnobs(mfx, &knobs);

  time_a = HAL_GetTick();
}

void update_mag(void)
{
  hid_report_t report;
  update_data();

  //report.axis[0] = sen.axis_average[0] + 128;
  //report.axis[1] = sen.axis_average[1] + 128;
  usb_send_report(&report);
}

static void update_data(void)
{
  uint32_t          time_b;
  float             delta_time;
  uint8_t           buffer[6];
  lsm6ds3tr_c_reg_t reg;

  lsm6ds3tr_c_status_reg_get(&dev_ctx, &reg.status_reg);

  if (reg.status_reg.xlda)
  {
    lsm6ds3tr_c_acceleration_raw_get(&dev_ctx, sen.a);

    /* Acceleration in g */
    data_in.acc[0] = sen.a[0];
    data_in.acc[1] = sen.a[1];
    data_in.acc[2] = sen.a[2];
  }

  if (reg.status_reg.gda)
  {
    lsm6ds3tr_c_angular_rate_raw_get(&dev_ctx, sen.g);

    /* Angular rate in dps */
    data_in.gyro[0] = sen.g[0];
    data_in.gyro[1] = sen.g[1];
    data_in.gyro[2] = sen.g[2];
  }

  if (reg.func_src1.sensorhub_end_op)
  {
    lsm6ds3tr_c_mag_calibrated_raw_get(&dev_ctx, sen.m);

    /* Magnetic field in uT/50 */
    data_in.mag[0] = sen.m[0];
    data_in.mag[1] = sen.m[1];
    data_in.mag[2] = sen.m[2];

    if (true == calibration_mode)
    {
      cal_data_in.mag[0] = sen.m[0];
      cal_data_in.mag[1] = sen.m[1];
      cal_data_in.mag[2] = sen.m[2];
    }
  }

  /* Get the magnetometer calibrated data,
   * with both hard-iron and soft-iron correction.
   *
   */
  if (reg.func_src1.si_end_op)
  {
    buffer[0] = read_reg(LSM6DS3TR_C_SENSORHUB1_REG);
    buffer[1] = read_reg(LSM6DS3TR_C_SENSORHUB2_REG);
    buffer[2] = read_reg(LSM6DS3TR_C_SENSORHUB3_REG);
    buffer[3] = read_reg(LSM6DS3TR_C_SENSORHUB4_REG);
    buffer[4] = read_reg(LSM6DS3TR_C_SENSORHUB5_REG);
    buffer[5] = read_reg(LSM6DS3TR_C_SENSORHUB6_REG);

    sen.mc[0] = (sen.mc[0] & 0xff00) | (buffer[0]);
    sen.mc[0] = (sen.mc[0] & 0x00ff) | (buffer[1] << 8);
    sen.mc[1] = (sen.mc[1] & 0xff00) | (buffer[2]);
    sen.mc[1] = (sen.mc[1] & 0x00ff) | (buffer[3] << 8);
    sen.mc[2] = (sen.mc[2] & 0xff00) | (buffer[4]);
    sen.mc[2] = (sen.mc[2] & 0x00ff) | (buffer[5] << 8);
  }

  time_b = HAL_GetTick();
  delta_time = (time_b / 1000.f) - (time_a / 1000.f);
  if (false == calibration_mode)
  {
    MotionFX_propagate(mfx, &data_out, &data_in, &delta_time);
    time_a = HAL_GetTick();
  }

  sen.yaw   = (int16_t)data_out.rotation[0];
  sen.pitch = (int16_t)data_out.rotation[1];
  sen.roll  = (int16_t)data_out.rotation[2];

  if (true == calibration_mode)
  {
    time_a = HAL_GetTick();

    MotionFX_MagCal_run(&cal_data_in);
    MotionFX_update(mfx, &data_out, &data_in, &delta_time, NULL);
    time_a = HAL_GetTick();
    MotionFX_MagCal_getParams(&cal_data_out);
    cal_data_in.time_stamp += 40;

    if (MFX_MAGCALGOOD == cal_data_out.cal_quality)
    {
      calibration_mode = false;
    }
    HAL_Delay(30);
  }
}

static uint8_t read_reg(uint8_t reg)
{
  uint8_t data;
  int32_t status = lsm6ds3tr_c_read_reg(&dev_ctx, reg, &data, 1);

  if (0 != status)
  {
    Error_Handler();
  }

  return data;
}

static void write_reg(uint8_t reg, uint8_t data)
{
  int32_t status = lsm6ds3tr_c_write_reg(&dev_ctx, reg, &data, 1);
  if (0 != status)
  {
    Error_Handler();
  }
}

/* Does not work yet, why? */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (DRDY_Pin == GPIO_Pin)
  {
    asm("NOP");
  }
  else if (INT1_Pin == GPIO_Pin)
  {
    asm("NOP");
  }
  else if (CALIBRATE_Pin == GPIO_Pin)
  {
    calibration_mode = true;
    MotionFX_MagCal_init(40, 1);
  }
}
