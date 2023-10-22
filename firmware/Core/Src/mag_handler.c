/*
 * mag_handler.c
 *
 *  Created on: Oct 20, 2023
 *      Author: Michael Fitzmayer
 */

#include <stdint.h>
#include <string.h>
#include "cmsis_os.h"
#include "i2c.h"
#include "lis2mdl_reg.h"
#include "lsm6ds3tr-c_reg.h"
#include "mag_handler.h"
#include "main.h"
#include "stm32f1xx_hal.h"
#include "usbd_custom_hid_if.h"

osThreadId_t mag_task_handle;
const osThreadAttr_t mag_task_attributes =
{
  .name = "mag_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

static void mag_task_handler(void *arg);

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

void init_mag_handler(void)
{
  mag_task_handle = osThreadNew(mag_task_handler, NULL, &mag_task_attributes);
}

static void mag_task_handler(void *arg)
{
  int32_t      status;
  uint8_t      data_buffer = 0;
  uint8_t      x_axis      = 127;
  uint8_t      y_axis      = 127;
  hid_report_t report;
  stmdev_ctx_t dev_ctx;

  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg  = platform_read;
  dev_ctx.handle    = &hi2c2;

  /* After the power supply is applied, the LSM6DS3TR-C performs a
   * 15ms boot procedure to load the trimming parameters.
   */
  osDelay(15);

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

  /* LSM6DS3TR-C Startup sequence. */

  /* Enable LSM6DS3TR-C high performance mode. */
  status = lsm6ds3tr_c_xl_power_mode_set(&dev_ctx, LSM6DS3TR_C_XL_HIGH_PERFORMANCE);
  if (0 != status)
  {
    Error_Handler();
  }

  while (1)
  {
    report.axis[0] = x_axis;
    report.axis[1] = y_axis;
    usb_send_report(&report);
    osDelay(1);
  }
}
