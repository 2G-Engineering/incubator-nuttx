/****************************************************************************
 * drivers/sensors/ltr303als.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <errno.h>
#include <endian.h>
#include <debug.h>
#include <stdio.h>
#include <string.h>

#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/random.h>

#include <nuttx/sensors/ltr303als.h>

#if 1 defined(CONFIG_I2C) && defined(CONFIG_SENSORS_LTR303ALS)

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#ifndef CONFIG_LTR303ALS_I2C_FREQUENCY
#  define CONFIG_LTR303ALS_I2C_FREQUENCY 400000
#endif

/* Registers definitions */

#define LTR303ALS_ALS_CONTR         0x80
#define LTR303ALS_ALS_MEAS_RATE     0x85
#define LTR303ALS_PART_ID           0x86
#define LTR303ALS_MANUFAC_ID        0x87
#define LTR303ALS_ALS_DATA_CH1_L    0x88
#define LTR303ALS_ALS_DATA_CH1_H    0x89
#define LTR303ALS_ALS_DATA_CH0_L    0x8A
#define LTR303ALS_ALS_DATA_CH0_H    0x8B
#define LTR303ALS_ALS_STATUS        0x8C
#define LTR303ALS_INTERRUPT         0x8F
#define LTR303ALS_ALS_THRES_UP_0    0x97
#define LTR303ALS_ALS_THRES_UP_1    0x98
#define LTR303ALS_ALS_THRES_LOW_0   0x99
#define LTR303ALS_ALS_THRES_LOW_1   0x9A
#define LTR303ALS_INTERRUPT_PERSIST 0x9E

/* Registers definitions */


#define LTR303ALS_ALS_RANGE_MASK      0x7
#define LTR303ALS_ALS_RANGE_SHIFT     0x2

#define LTR303ALS_OP_MODE_MASK        0x1
#define LTR303ALS_OP_MODE_SHIFT       0x0

#define LTR303ALS_ALS_INTTIME_MASK    0x7
#define LTR303ALS_ALS_INTTIME_SHIFT   0x3

#define LTR303ALS_ALS_MEASRATE_MASK   0x7
#define LTR303ALS_ALS_MEASRATE_SHIFT  0x0

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ltr303als_dev_s
{
  FAR struct i2c_master_s *i2c;
  uint8_t addr;                   /* Address on the I2C bus */
  uint8_t op_mode;                /* Defined by ltr303als_operational_mode_e */
  uint32_t range;                 /* Sensor range 600..64000 */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C Helpers */

static int ltr303als_i2c_write(FAR struct ltr303als_dev_s *dev,
                          FAR const uint8_t *buffer, ssize_t buflen);
static int ltr303als_i2c_read(FAR struct ltr303als_dev_s *dev,
                         FAR uint8_t *buffer, ssize_t buflen);
static int ltr303als_read_reg(FAR struct ltr303als_dev_s *dev,
                      const uint8_t regaddr, uint8_t *buffer, size_t buflen);
static int ltr303als_read_lux(FAR struct ltr303als_dev_s *dev,
                              FAR struct ltr303als_data_s *data);
static int ltr303als_set_op_mode(FAR struct ltr303als_dev_s *dev,
                                 uint8_t mode);
static int ltr303als_set_resolution(FAR struct ltr303als_dev_s *dev,
                                    uint8_t res_mode);
static int ltr303als_set_range(FAR struct ltr303als_dev_s *dev,
                               uint8_t range_mode);
static int ltr303als_set_integration_time(FAR struct ltr303als_dev_s *dev,
                                          uint8_t int_time);
static int ltr303als_set_meas_rate(FAR struct ltr303als_dev_s *dev,
                                   uint8_t meas_rate);
static int ltr303als_set_int_thresh(FAR struct ltr303als_dev_s *dev,
                                    uint16_t int_low, uint16_t int_up);
static int ltr303als_set_int_mode(FAR struct ltr303als_dev_s *dev,
                                  uint8_t int_mode);


/* Driver methods */

static int ltr303als_open(FAR struct file *filep);
static int ltr303als_close(FAR struct file *filep);
static ssize_t ltr303als_read(FAR struct file *filep,
                             FAR char *buffer,
                             size_t buflen);
static ssize_t ltr303als_write(FAR struct file *filep,
                              FAR const char *buffer,
                              size_t buflen);
static int ltr303als_ioctl(FAR struct file *filep,
                          int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint16_t ltr303_range[8] =
{
  64000,
  32000,
  16000,
  8000,
  1, /* Reserved */
  1, /* Reserved */
  1300,
  600,
};

static const struct file_operations g_ltr303alsfops =
{
  ltr303als_open,   /* open */
  ltr303als_close,  /* close */
  ltr303als_read,   /* read */
  ltr303als_write,  /* write */
  NULL,            /* seek */
  ltr303als_ioctl,  /* ioctl */
  NULL             /* poll */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ltr303als_i2c_write
 *
 * Description:
 *   Write to the I2C device.
 *
 ****************************************************************************/

static int ltr303als_i2c_write(FAR struct ltr303als_dev_s *dev,
                          FAR const uint8_t *buffer, ssize_t buflen)
{
  struct i2c_msg_s msg;
  int ret;

  /* Setup for the transfer */

  msg.frequency = CONFIG_LTR303ALS_I2C_FREQUENCY,
  msg.addr      = dev->addr;
  msg.flags     = 0;
  msg.buffer    = (FAR uint8_t *)buffer;  /* Override const */
  msg.length    = buflen;

  /* Then perform the transfer. */

  ret = I2C_TRANSFER(dev->i2c, &msg, 1);
  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: ltr303als_i2c_read
 *
 * Description:
 *   Read from the I2C device.
 *
 ****************************************************************************/

static int ltr303als_i2c_read(FAR struct ltr303als_dev_s *dev,
                         FAR uint8_t *buffer, ssize_t buflen)
{
  struct i2c_msg_s msg;
  int ret;

  /* Setup for the transfer */

  msg.frequency = CONFIG_LM75_I2C_FREQUENCY,
  msg.addr      = dev->addr,
  msg.flags     = I2C_M_READ;
  msg.buffer    = buffer;
  msg.length    = buflen;

  /* Then perform the transfer. */

  ret = I2C_TRANSFER(dev->i2c, &msg, 1);
  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: ltr303als_read_reg
 *
 * Description:
 *   Read register from the I2C device.
 *
 ****************************************************************************/

static int ltr303als_read_reg(FAR struct ltr303als_dev_s *dev,
                             const uint8_t regaddr, uint8_t *buffer,
                             size_t buflen)
{
  int ret;

  ret = ltr303als_i2c_write(dev, &regaddr, 1);
  if (ret < 0)
    {
      snerr("ERROR: i2c write failed: %d\n", ret);
      return ret;
    }

  ret = ltr303als_i2c_read(dev, buffer, buflen);
  if (ret < 0)
    {
      snerr("ERROR: i2c read failed: %d\n", ret);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Name: ltr303als_open
 *
 * Description:
 *   This function is called whenever the LTR303ALS device is opened.
 *
 ****************************************************************************/

static int ltr303als_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: ltr303als_close
 *
 * Description:
 *   This routine is called when the LTR303ALS device is closed.
 *
 ****************************************************************************/

static int ltr303als_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: ltr303als_read
 ****************************************************************************/

static ssize_t ltr303als_read(FAR struct file *filep, FAR char *buffer,
                             size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ltr303als_dev_s *priv = inode->i_private;
  int ret;
  struct ltr303als_data_s data;

  ret = ltr303als_read_lux(priv, &data);
  if (ret < 0)
    {
      snerr("ERROR: failed to read the sensor: %d\n", ret);
      return (ssize_t)ret;
    }
  else
    {
      if (buflen < sizeof(data))
        {
          return -EIO;
        }

      memcpy(buffer, &data, sizeof(data));
    }

  return buflen;
}

/****************************************************************************
 * Name: ltr303als_write
 ****************************************************************************/

static ssize_t ltr303als_write(FAR struct file *filep, FAR const char *buffer,
                              size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: ltr303als_read_lux
 ****************************************************************************/

static int ltr303als_read_lux(FAR struct ltr303als_dev_s *dev,
                              FAR struct ltr303als_data_s *data)
{
  int ret;
  uint8_t buffer[4];
  uint32_t tmp;

  ret = ltr303als_read_reg(dev, LTR303ALS_ALS_DATA_CH1_L, buffer, 4);
  if (ret < 0)
    {
      return ret;
    }

  data->raw[0] = letoh((buffer[3] << 8) | buffer[2]);
  data->raw[1] = letoh((buffer[1] << 8) | buffer[0]);

  add_sensor_randomness(data->raw[0]);
  add_sensor_randomness(data->raw[1]);

  tmp = (data->raw[0] * dev->range) / 65535;
  data->lux[0] = (uint16_t)tmp;
  tmp = (data->raw[1] * dev->range) / 65535;
  data->lux[1] = (uint16_t)tmp;

  sninfo("raw value 0: %8x, lux: %5u\n", data->raw[0], data->lux[0]);
  sninfo("raw value 1: %8x, lux: %5u\n", data->raw[1], data->lux[1]);

  return OK;
}

/****************************************************************************
 * Name: ltr303als_set_op_mode
 ****************************************************************************/

static int ltr303als_set_op_mode(FAR struct ltr303als_dev_s *dev, uint8_t mode)
{
  uint8_t buffer[2];
  int ret;

  ret = ltr303als_read_reg(dev, LTR303ALS_ALS_CONTR, &buffer[1], 1);
  if (ret < 0)
    {
      snerr("ERROR: i2c read reg failed: %d\n", ret);
      return ret;
    }

  /* Clear the mode bits */

  buffer[1] &= ~(LTR303ALS_OP_MODE_MASK << LTR303ALS_OP_MODE_SHIFT);
  mode &= LTR303ALS_OP_MODE_MASK;

  /* Modify mode bits */

  buffer[1] |= (mode & LTR303ALS_OP_MODE_MASK) << LTR303ALS_OP_MODE_SHIFT;
  buffer[0] = LTR303ALS_ALS_CONTR;

  dev->op_mode = mode;
  sninfo("mode: %x\n", dev->mode);

  return ltr303als_i2c_write(dev, buffer, 2);
}

/****************************************************************************
 * Name: ltr303als_set_resolution
 ****************************************************************************/

static int ltr303als_set_range(FAR struct ltr303als_dev_s *dev,
                              uint8_t range_mode)
{
  uint8_t buffer[2];
  int ret;

  ret = ltr303als_read_reg(dev, LTR303ALS_CONTR, &buffer[1], 1);
  if (ret < 0)
    {
      snerr("ERROR: i2c read reg failed: %d\n", ret);
      return ret;
    }

  /* Clear the range bits */

  buffer[1] &= ~(LTR303ALS_ALS_RANGE_MASK << LTR303ALS_ALS_RANGE_SHIFT);

  /* Modify range bits */

  range_mode &= LTR303ALS_ALS_RANGE_MASK;
  buffer[1] |= range_mode << LTR303ALS_ALS_RANGE_SHIFT;
  buffer[0] = LTR303ALS_CONTR;

  dev->range = ltr303_range[range_mode];
  sninfo("range: %u\n", dev->range);

  return ltr303als_i2c_write(dev, buffer, 2);
}

static int ltr303als_set_integration_time(FAR struct ltr303als_dev_s *dev,
                                          uint8_t int_time)
{
  uint8_t buffer[2];
  int ret;

  ret = ltr303als_read_reg(dev, LTR303ALS_ALS_MEAS_RATE, &buffer[1], 1);
  if (ret < 0)
    {
      snerr("ERROR: i2c read reg failed: %d\n", ret);
      return ret;
    }

  /* Clear the mode bits */

  buffer[1] &= ~(LTR303ALS_ALS_INTTIME_MASK << LTR303ALS_ALS_INTTIME_SHIFT);

  /* Modify mode bits */

  int_time &= LTR303ALS_ALS_INTTIME_MASK;
  buffer[1] |= int_time << LTR303ALS_ALS_INTTIME_SHIFT;
  buffer[0] = LTR303ALS_ALS_MEAS_RATE;

  sninfo("int time: %u\n", int_time);

  return ltr303als_i2c_write(dev, buffer, 2);
}

static int ltr303als_set_meas_rate(FAR struct ltr303als_dev_s *dev,
                                   uint8_t meas_rate)
{
  uint8_t buffer[2];
  int ret;

  ret = ltr303als_read_reg(dev, LTR303ALS_ALS_MEAS_RATE, &buffer[1], 1);
  if (ret < 0)
    {
      snerr("ERROR: i2c read reg failed: %d\n", ret);
      return ret;
    }

  /* Clear the mode bits */

  buffer[1] &= ~(LTR303ALS_ALS_MEASRATE_MASK << LTR303ALS_ALS_MEASRATE_SHIFT);

  /* Modify mode bits */

  meas_rate &= LTR303ALS_ALS_MEASRATE_MASK;
  buffer[1] |= meas_rate << LTR303ALS_ALS_MEASRATE_SHIFT;
  buffer[0] = LTR303ALS_ALS_MEAS_RATE;

  sninfo("meas rate: %u\n", meas_rate);

  return ltr303als_i2c_write(dev, buffer, 2);
}

static int ltr303als_set_int_thresh(FAR struct ltr303als_dev_s *dev,
                                    uint16_t int_low, uint16_t int_up)
{
  uint8_t buffer[5];
  int ret;

  int_low = htole(int_low);
  int_up = htole(int_up);

  buffer[1] = int_up & 0xFF;
  buffer[2] = (int_up >> 8) & 0xFF;
  buffer[3] = int_low & 0xFF;
  buffer[4] = (int_low >> 8) & 0xFF;
  buffer[0] = LTR303ALS_ALS_THRES_UP_0;

  return ltr303als_i2c_write(dev, buffer, 5);
}

static int ltr303als_set_int_mode(FAR struct ltr303als_dev_s *dev,
                                  uint8_t int_mode)
{
#error not implemented

}
/****************************************************************************
 * Name: ltr303als_ioctl
 ****************************************************************************/

static int ltr303als_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode            *inode = filep->f_inode;
  FAR struct ltr303als_dev_s   *priv  = inode->i_private;
  int ret = OK;

  switch (cmd)
    {
      /* Arg:  uint8_t value */

      case SNIOC_SET_OPERATIONAL_MODE:
        ret = ltr303als_set_op_mode(priv, (uint8_t)arg);
        sninfo("Set operation mode %d with result %d\n", (uint8_t)arg, ret);
        break;

      /* Arg:  uint8_t value */

      case SNIOC_SET_RANGE:
        ret = ltr303als_set_range(priv, (uint8_t)arg);
        sninfo("Set range mode %d with result %d\n", (uint8_t)arg, ret);
        break;

      default:
        sninfo("Unrecognized cmd: 0x%04x\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int ltr303als_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                      uint8_t addr)
{
  FAR struct ltr303als_dev_s *priv;
  int ret;

  priv = (FAR struct ltr303als_dev_s *)
              kmm_malloc(sizeof(struct ltr303als_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c        = i2c;
  priv->addr       = addr;
  priv->resolution = 0x10000;
  priv->range      = 64000;
  priv->op_mode    = LTR303ALS_OP_MODE_POWER_DOWN;

  /* Register the character driver */

  ret = register_driver(devpath, &g_ltr303alsfops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_LTR303ALS */
