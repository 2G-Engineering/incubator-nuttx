/****************************************************************************
 * include/nuttx/sensors/ltr303als.h
 *
  * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_SENSORS_LTR303ALS
#define __INCLUDE_NUTTX_SENSORS_LTR303ALS

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sensors/ioctl.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_LTR303ALS)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s;

enum ltr303als_als_range_e
{
  LTR303ALS_ALS_RANGE_64000 =       0x0,
  LTR303ALS_ALS_RANGE_32000 =       0x1,
  LTR303ALS_ALS_RANGE_16000 =       0x2,
  LTR303ALS_ALS_RANGE_8000 =        0x3,
  LTR303ALS_ALS_RANGE_1300 =        0x6,
  LTR303ALS_ALS_RANGE_600 =         0x7,
};

enum ltr303als_operational_mode_e
{
  LTR303ALS_OP_MODE_POWER_DOWN =    0x0,
  LTR303ALS_OP_MODE_ACTIVE =        0x1,
};

enum ltr303als_integration_time_e
{
  LTR303ALS_ALS_INT_TIME_100MS =    0x0,
  LTR303ALS_ALS_INT_TIME_50MS =     0x1,
  LTR303ALS_ALS_INT_TIME_200MS =    0x2,
  LTR303ALS_ALS_INT_TIME_400MS =    0x3,
  LTR303ALS_ALS_INT_TIME_150MS =    0x4,
  LTR303ALS_ALS_INT_TIME_250MS =    0x5,
  LTR303ALS_ALS_INT_TIME_300MS =    0x6,
  LTR303ALS_ALS_INT_TIME_350MS =    0x7,
};

enum ltr303als_measurement_rate_e
{
  LTR303ALS_ALS_MEAS_RATE_50MS =   0x0,
  LTR303ALS_ALS_MEAS_RATE_100MS =  0x1,
  LTR303ALS_ALS_MEAS_RATE_200MS =  0x2,
  LTR303ALS_ALS_MEAS_RATE_500MS =  0x3,
  LTR303ALS_ALS_MEAS_RATE_1000MS = 0x4,
  LTR303ALS_ALS_MEAS_RATE_2000MS = 0x5,
};

/* Data transfer structure (this device has 2 sensor channels) */

struct ltr303als_data_s
{
  uint16_t lux[2];              /* Converted lux values */
  uint16_t raw[2];              /* Raw unconverted values */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: ltr303als_register
 *
 * Description:
 *   Register the LTR303ALS ALS device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/als0"
 *   i2c - An instance of the I2C interface to use to communicate with the ALS
 *   addr - The I2C address of the ALS.  The default for this device is 0x52.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ltr303als_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                      uint8_t addr);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_SENSORS_LTR303ALS */
#endif /* __INCLUDE_NUTTX_SENSORS_LTR303ALS */
