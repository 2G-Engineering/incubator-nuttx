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
#include <debug.h>
#include <errno.h>
#include <inttypes.h>
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

/* Register bit definitions */

/* ALS_CONTR register */

#define LTR303ALS_ALS_RANGE_MASK      0x7
#define LTR303ALS_ALS_RANGE_SHIFT     0x2

#define LTR303ALS_ALS_SW_RESET_MASK   0x1
#define LTR303ALS_ALS_SW_RESET_SHIFT  0x1

#define LTR303ALS_OP_MODE_MASK        0x1
#define LTR303ALS_OP_MODE_SHIFT       0x0

/* ALS_MEAS_RATE register */

#define LTR303ALS_ALS_INTTIME_MASK    0x7
#define LTR303ALS_ALS_INTTIME_SHIFT   0x3

#define LTR303ALS_ALS_MEASRATE_MASK   0x7
#define LTR303ALS_ALS_MEASRATE_SHIFT  0x0

/* PART_ID register */

#define LTR303ALS_PART_ID_VALUE       0xA
#define LTR303ALS_PART_ID_MASK        0xF
#define LTR303ALS_PART_ID_SHIFT       0x4

/* MANUFAC_ID register */

#define LTR303ALS_MANUFAC_ID_VALUE    0x05

/* INTERRUPT register */

#define LTR303ALS_INT_POL_MASK        0x1
#define LTR303ALS_INT_POL_SHIFT       0x2

#define LTR303ALS_INT_MODE_MASK       0x1
#define LTR303ALS_INT_MODE_SHIFT      0x1

/* INTERRUPT PERSIST register */

#define LTR303ALS_INT_PERSIST_MASK    0x7
#define LTR303ALS_INT_PERSIST_SHIFT   0x0

/* Helpers ******************************************************************/

#ifndef MIN
#  define MIN(a,b) (a < b ? a : b)
#endif
#ifndef MAX
#  define MAX(a,b) (a > b ? a : b)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ltr303als_dev_s
{
  FAR struct i2c_master_s *i2c;
  uint8_t addr;                   /* Address on the I2C bus */
  uint8_t op_mode;                /* Defined by ltr303als_operational_mode_e */
  ltr303als_meas_type_e meas_type;/* Defined by ltr303als_meas_type_e */
  uint32_t range;                 /* Sensor range 600..64000 */
  uint32_t upper_int_thr;         /* Upper interrupt threshold (normalized) */
  uint32_t lower_int_thr;         /* Upper interrupt threshold (normalized) */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static uint16_t lux_to_raw(uint32_t range, uint16_t lux);
static uint16_t raw_to_lux(uint32_t range, uint16_t raw);

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
static void ltr303als_update_int_thresh(FAR struct ltr303als_dev_s *dev,
                                        FAR struct ltr303als_int_cfg_s *cfg);
static int ltr303als_write_int_thresh(FAR struct ltr303als_dev_s *dev);
static int ltr303als_set_int_config(FAR struct ltr303als_dev_s *dev,
                                    bool enable, bool polarity);
static int ltr303als_set_int_persist(FAR struct ltr303als_dev_s *dev,
                                     uint8_t persist);


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

/* Mapping between ALS Gain field values and actual gain value */

static const uint16_t ltr303_range[8] =
{
  64000,  /* 0x0 */
  32000,  /* 0x1 */
  16000,  /* 0x2 */
  8000,   /* 0x3 */
  1,      /* 0x4: Reserved */
  1,      /* 0x5: Reserved */
  1300,   /* 0x6 */
  600,    /* 0x7 */
};

static const struct file_operations g_ltr303alsfops =
{
  ltr303als_open,   /* open */
  ltr303als_close,  /* close */
  ltr303als_read,   /* read */
  ltr303als_write,  /* write */
  NULL,             /* seek */
  ltr303als_ioctl,  /* ioctl */
  NULL              /* poll */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint16_t lux_to_raw(uint32_t range, uint16_t lux)
{
  uint32_t tmp;
  tmp = ((uint32_t)lux * 65535) / range;
  return (uint16_t)tmp;
}

static uint16_t raw_to_lux(uint32_t range, uint16_t raw) {
  uint32_t tmp;
  tmp = ((uint32_t)raw * range) / 65535;
  return (uint16_t)tmp;
}
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

      /* Allow partial reads as long as they're reading
       * whole 16-bit words */

      if ((buflen % 2) != 0)
        {
          return -EIO;
        }

      memcpy(buffer, &data, MIN(buflen, sizeof(data)));
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

  data->raw[0] = (buffer[3] << 8) | buffer[2];
  data->raw[1] = (buffer[1] << 8) | buffer[0];

  add_sensor_randomness(data->raw[0]);
  add_sensor_randomness(data->raw[1]);

  /* Resolution is always 16 bits */

  data->lux[0] = raw_to_lux(dev->range, data->raw[0])
  data->lux[1] = raw_to_lux(dev->range, data->raw[1]);

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

/* Writes interrupt thresholds to device  */
static int ltr303als_write_int_thresh(FAR struct ltr303als_dev_s *dev)
{
  uint8_t buffer[5];
  uint16_t int_up;
  uint16_t int_low;
  int ret;

  if (cfg->thr_type == LTR303ALS_MEAS_LUX) {
      int_up = lux_to_raw(dev->range, dev->upper_int_thr);
      int_low = lux_to_raw(dev->range, dev->lower_int_thr);
  } else {
      int_up = dev->upper_int_thr;
      int_low = dev->lower_int_thr;
  }

  buffer[1] = int_up & 0xFF;
  buffer[2] = (int_up >> 8) & 0xFF;
  buffer[3] = int_low & 0xFF;
  buffer[4] = (int_low >> 8) & 0xFF;
  buffer[0] = LTR303ALS_ALS_THRES_UP_0;

  sninfo("int thresholds: %u %u\n", int_low, int_up);

  return ltr303als_i2c_write(dev, buffer, 5);
}

/* Stores interrupt thresholds to device struct.
 * Must be followed by a call to ltr303als_set_int_thresh for
 * the change to actually take effect. */
static void ltr303als_update_int_thresh(FAR struct ltr303als_dev_s *dev,
                                        FAR struct ltr303als_int_cfg_s *cfg)
{
    dev->lower_int_thr = cfg->lower_int_thresh;
    dev->upper_int_thr = cfg->lower_int_thresh;
    dev->meas_type = cfg->thr_type;
}

static int ltr303als_set_int_config(FAR struct ltr303als_dev_s *dev,
                                    bool enable, bool polarity)
{
    uint8_t buffer[2] = {0};
    int ret;

    buffer[1]  = enable ? 1 << LTR303ALS_INT_MODE_SHIFT : 0;
    buffer[1] |= polarity ? 1 << LTR303ALS_INT_POL_SHIFT : 0;
    buffer[0]  = LTR303ALS_INTERRUPT;

    sninfo("int config: %u %u\n", enable, polarity);

    return ltr303als_i2c_write(dev, buffer, 2);
}

static int ltr303als_set_int_persist(FAR struct ltr303als_dev_s *dev,
                                     uint8_t persist)
{
    uint8_t buffer[2] = {0};
    int ret;

    buffer[1]  = persist & LTR303ALS_INT_PERSIST_MASK;
    buffer[0]  = LTR303ALS_INTERRUPT;

    sninfo("int persist: %u\n", persist);

    return ltr303als_i2c_write(dev, buffer, 2);
}

static int ltr303als_config_interrupt(FAR struct ltr303als_dev_s *dev,
                                      FAR struct ltr303als_int_cfg_s *cfg)
{
  int ret;

  /* Disable interrupt pin while working on configuration */

  ret = ltr303als_set_int_config(priv, false, cfg->int_pol);
  if (ret < 0) {
      return ret;
  }

  ltr303als_update_int_thresh(priv, cfg);

  ret = ltr303als_write_int_thresh(priv, cfg->int_lower_thresh,
                                 cfg->int_upper_thresh);
  if (ret < 0) {
      return ret;
  }

  ret = ltr303als_set_int_persist(priv, cfg->persist);
  if (ret < 0) {
      return ret;
  }

  ret = ltr303als_set_int_config(priv, cfg->int_en, cfg->int_pol);
  return ret;
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
      /* Arg:  ltr303als_operational_mode_e value */

      case SNIOC_SET_OPERATIONAL_MODE:
        ret = ltr303als_set_op_mode(priv, (uint8_t)arg);
        sninfo("Set operation mode %d with result %d\n", (uint8_t)arg, ret);
        break;

      /* Arg:  ltr303als_als_range_e value */

      case SNIOC_SET_RANGE:
        ret = ltr303als_set_range(priv, (uint8_t)arg);
        sninfo("Set range mode %d with result %d\n", (uint8_t)arg, ret);
        break;

      /* Arg:  ltr303als_integration_time_e value */

      case SNIOC_SET_INTEG_TIME:
        ret = ltr303als_set_integration_time(priv, (uint8_t)arg);
        sninfo("Set integration time %d with result %d\n", (uint8_t)arg, ret);
        break;

      /* Arg:  ltr303als_measurement_rate_e value */

      case SNIOC_SET_MEAS_RATE:
        ret = ltr303als_set_meas_rate(priv, (uint8_t)arg);
        sninfo("Set measurement rate %d with result %d\n", (uint8_t)arg, ret);
        break;

     /* Arg: ltr303als_int_cfg_s* pointer */

      case SNIOC_CONFIGURE_INT:
        {
          FAR struct ltr303als_int_cfg_s *cfg;
          cfg = (ltr303als_int_cfg_s *) arg;
          ret = ltr303als_config_interrupt(priv, cfg);
        }
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
  uint8_t buffer[2];

  priv = (FAR struct ltr303als_dev_s *)
              kmm_malloc(sizeof(struct ltr303als_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c           = i2c;
  priv->addr          = addr;
  priv->range         = 64000;
  priv->lower_int_thr = 0;
  priv->upper_int_thr = 65535;
  priv->op_mode       = LTR303ALS_OP_MODE_POWER_DOWN;

  /* Verify that the device is connected by reading ID registers */
  ret = ltr303als_read_reg(dev, LTR303ALS_PART_ID, buffer, 2);
  if (ret < 0)
    {
      snerr("ERROR: Failed to read device identification: %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  /* Check that ID registers match expected values */

  if ((((buffer[0] >> LTR303ALS_PART_ID_SHIFT) & LTR303ALS_PART_ID_MASK) !=
      LTR303ALS_PART_ID_VALUE) || (buffer[1] != LTR303ALS_MANUFAC_ID_VALUE))
    {
      snerr("ERROR: LTR303ALS device not found (%"PRIx8", %"PRIx8"): %d\n",
            buffer[0], buffer[1], ret);
      kmm_free(priv);
      return ret;
    }

  /* Send software reset */

  buffer[0] = LTR303ALS_ALS_CONTR;
  buffer[1] = LTR303ALS_ALS_SW_RESET_MASK << LTR303ALS_ALS_SW_RESET_SHIFT;
  ret = ltr303als_i2c_write(dev, buffer, 2);
  if (ret < 0)
    {
      snerr("ERROR: Failed to reset LTR303ALS: %d\n", ret);
      kmm_free(priv);
      return ret;
    }

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
