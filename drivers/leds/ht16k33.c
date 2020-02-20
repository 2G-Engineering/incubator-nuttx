/****************************************************************************
 * drivers/leds/ht16k33.c
 *
 *   Copyright (C) 2020 2G Engineering. All rights reserved.
 *   Author: Joshua Lange <jlange@2g-eng.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/leds/ht16k33.h>

#if defined(CONFIG_I2C) && defined(CONFIG_HT16K33)

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

struct ht16k33_dev_s
{
  FAR struct i2c_master_s *i2c;
  uint8_t i2c_addr;
  uint8_t blink;
  bool on;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ht16k33_i2c_write_byte(FAR struct ht16k33_dev_s *priv,
                                    uint8_t const reg_addr,
                                    uint8_t const reg_val);
static int ht16k33_set_led_mode(FAR struct ht16k33_dev_s *priv,
                                  uint8_t const led_out_x_mode);

static int ht16k33_open(FAR struct file *filep);
static int ht16k33_close(FAR struct file *filep);
static int ht16k33_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static ssize_t ht16k33_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static ssize_t ht16k33_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_ht16k33_fileops =
{
  ht16k33_open,               /* open */
  ht16k33_close,              /* close */
  ht16k33_read,               /* read */
  ht16k33_write,              /* write */
  0,                          /* seek */
  ht16k33_ioctl,              /* ioctl */
  0                           /* poll */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ht16k33_i2c_write_byte
 *
 * Description:
 *   Write a single byte to one of the HT16K33 configuration registers.
 *
 ****************************************************************************/

static int ht16k33_i2c_write_byte(FAR struct ht16k33_dev_s *priv,
                                    uint8_t const reg_addr,
                                    uint8_t const reg_val)
{
  struct i2c_config_s config;
  int ret;

  /* assemble the 2 byte message comprised of reg_addr and reg_val */

  uint8_t const BUFFER_SIZE = 1;
  uint8_t buffer[BUFFER_SIZE];

  buffer[0] = reg_addr | reg_val;

  /* Setup up the I2C configuration */

  config.frequency = I2C_BUS_FREQ_HZ;
  config.address   = priv->i2c_addr;
  config.addrlen   = 7;

  /* Write the register address followed by the data (no RESTART) */

  ledinfo("i2c addr: 0x%02X reg addr: 0x%02X value: 0x%02X\n", priv->i2c_addr,
          reg_addr, reg_val);

  ret = i2c_write(priv->i2c, &config, buffer, BUFFER_SIZE);
  if (ret < 0)
    {
      lederr("ERROR: i2c_write returned error code %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: ht16k33_set_led_onoff
 *
 * Description:
 *   Enables or disables all LEDs at once.
 *
 ****************************************************************************/

static int ht16k33_set_led_onoff(FAR struct ht16k33_dev_s *priv,
                                      bool const on)
{
  int ret = -1;
  priv->on = on;

  ret = ht16k33_update_blink_onoff(priv);

  return ret;
}
/****************************************************************************
 * Name: ht16k33_set_led_blink_rate
 *
 * Description:
 *   Set the blink rate for all LEDs
 *
 ****************************************************************************/

static int ht16k33_set_led_blink_rate(FAR struct ht16k33_dev_s *priv,
                                      uint8_t const blinkrate)
{
  int ret = -1;
  priv->blink = blinkrate;

  ret = ht16k33_update_blink_onoff(priv);

  return OK;
}

/****************************************************************************
 * Name: ht16k33_open
 *
 * Description:
 *   This function is called whenever a HT16K33 device is opened.
 *
 ****************************************************************************/

static int ht16k33_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ht16k33_dev_s *priv = inode->i_private;
  int ret = -1;

  /* Wake up the HT16K33 by enabling the oscillator. */

  ret = ht16k33_i2c_write_byte(priv, HT16K33_SS, HT16K33_SS_NORMAL);
  if (ret < 0)
    {
      lederr("ERROR: Could not set initial config for HT16K33_SS\n");
      return ret;
    }

  /* Allow 25ms (~2 frame intervals) for the internal oscillator to come up */

  nxsig_usleep(25 * USEC_PER_MSEC);

  /* Configure the HT16K33 ROW/INT pin to row output mode */

  ret = ht16k33_i2c_write_byte(priv, HT16K33_RIS, HT16K33_RIS_OUT);
  if (ret < 0)
    {
      lederr("ERROR: Could not set initial config for HT16K33_RIS\n");
      return ret;
    }


  /* Enable LEDs & set blink to off */

  ret = ht16k33_i2c_write_byte(priv, HT16K33_DISP,
                               HT16K33_DISP_ON | HT16K33_DISP_BLINKOFF);
  if (ret < 0)
    {
      lederr("ERROR: Could not set initial config for HT16K33_DISP\n");
      return ret;
    }

  /* Set LEDs to full brightness */

  ret = ht16k33_i2c_write_byte(priv, HT16K33_DIM, HT16K33_DIM_MAX);
  if (ret < 0)
    {
      lederr("ERROR: Could not set initial config for HT16K33_DISP\n");
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: ht16k33_close
 *
 * Description:
 *   This function is called whenever a HT16K33 device is closed.
 *
 ****************************************************************************/

static int ht16k33_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ht16k33_dev_s *priv = inode->i_private;
  int ret = -1;

  /* Turn all led drivers off */

  ret = ht16k33_i2c_write_byte(priv, HT16K33_DISP,
                               HT16K33_DISP_OFF | HT16K33_DISP_BLINKOFF);
  if (ret < 0)
    {
      lederr("ERROR: Could not set HT16K33_DISP off\n");
      return ret;
    }

  /* Send the HT16K33 back to standby mode */

  ret = ht16k33_i2c_write_byte(priv, HT16K33_SS, HT16K33_SS_STANDBY);
  if (ret < 0)
    {
      lederr("ERROR: Could not set HT16K33_SS to standby\n");
      return ret;
    }
  if (ret < 0)
    {
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: ht16k33_close
 *
 * Description:
 *   This function is called whenever an ioctl call to a HT16K33 is performed.
 *
 ****************************************************************************/

static int ht16k33_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ht16k33_dev_s *priv = inode->i_private;
  int ret = OK;

  ledinfo("cmd: %d arg: %ld\n", cmd, arg);

  switch (cmd)
    {
      /* Set the brightness of an indivual LED. Arg: ht16k33_led_brightness_s
       * pointer.
       */

    case PWMIOC_SETLED_BRIGHTNESS:
      {
        /* Retrieve the information handed over as argument for this ioctl */

        FAR const uint8_t brightness = *((uint8_t*)arg);

        DEBUGASSERT(ptr != NULL);

        /* Set the brighntess of the led */

//        ret = ht16k33_i2c_write_byte(priv, ptr->led, ptr->brightness);
      }
      break;

      /* The used ioctl command was invalid */

    default:
      {
        lederr("ERROR: Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
      }
      break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ht16k33_register
 *
 * Description:
 *   Register the HT16K33 device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/leddrv0".
 *   i2c     - An instance of the I2C interface to use to communicate
 *             with the LM92.
 *   ht16k33_i2c_addr
 *           - The I2C address of the HT16K33.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ht16k33_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                       uint8_t const ht16k33_i2c_addr)
{
  /* Sanity check */

  DEBUGASSERT(i2c != NULL);

  /* Initialize the HT16K33 device structure */

  FAR struct ht16k33_dev_s *priv =
    (FAR struct ht16k33_dev_s *)kmm_malloc(sizeof(struct ht16k33_dev_s));

  if (priv == NULL)
    {
      lederr("ERROR: Failed to allocate instance of ht16k33_dev_s\n");
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->i2c_addr = ht16k33_i2c_addr;

  /* Register the character driver */

  int const ret = register_driver(devpath, &g_ht16k33_fileops, 666, priv);
  if (ret != OK)
    {
      lederr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  return OK;
}

#endif /* CONFIG_I2C && CONFIG_I2C_HT16K33 */
