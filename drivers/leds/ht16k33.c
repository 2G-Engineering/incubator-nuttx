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

static int ht16k33_i2c_write_reg(FAR struct ht16k33_dev_s *priv,
                                    uint8_t const reg_addr,
                                    uint8_t const reg_val);
static int ht16k33_i2c_write_data(FAR struct ht16k33_dev_s *priv,
                                    uint8_t const reg_addr,
                                    uint8_t const reg_val,
                                    FAR const char *data,
                                    uint32_t const data_len);

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
 * Name: ht16k33_i2c_write_data
 *
 * Description:
 *   Write multiple bytes to the HT16K33 device.
 *
 ****************************************************************************/

static int ht16k33_i2c_write_data(FAR struct ht16k33_dev_s *priv,
                                    uint8_t const reg_addr,
                                    uint8_t const reg_val,
                                    FAR const char *data,
                                    uint32_t data_len)
{
  struct i2c_config_s config;
  int ret;

  /* Can't send more data than our device's RAM size */
  if (data_len > HT16K33_DISP_RAM_SIZE)
    {
      data_len = HT16K33_DISP_RAM_SIZE;
    }

  /* Assemble the message comprised of reg_addr and reg_val, plus
   * the additional data we want to send. */

  uint8_t const BUFFER_SIZE = HT16K33_DISP_RAM_SIZE + 1;
  uint8_t buffer[BUFFER_SIZE];

  buffer[0] = reg_addr & HT16K33_REG_ADDR_MASK;
  buffer[0] |= reg_val & HT16K33_REG_DATA_MASK;

  if (data)
    {
      memcpy(&buffer[1], data, data_len);
    }

  /* Setup up the I2C configuration */

  config.frequency = I2C_BUS_FREQ_HZ;
  config.address   = priv->i2c_addr;
  config.addrlen   = 7;

  /* Write the register address followed by the data (no RESTART) */

  ledinfo("i2c addr: 0x%02X reg addr: 0x%02X value: 0x%02X (%d bytes additional)\n",
          priv->i2c_addr, reg_addr, reg_val, data_size);

  ret = i2c_write(priv->i2c, &config, buffer, BUFFER_SIZE);
  if (ret < 0)
    {
      lederr("ERROR: i2c_write returned error code %d\n", ret);
      return ret;
    }

  return data_len;
}


/****************************************************************************
 * Name: ht16k33_i2c_write_reg
 *
 * Description:
 *   Write a single byte to one of the HT16K33 configuration registers.
 *
 ****************************************************************************/

static int ht16k33_i2c_write_reg(FAR struct ht16k33_dev_s *priv,
                                    uint8_t const reg_addr,
                                    uint8_t const reg_val)
{
  return ht16k33_i2c_write_data(priv, reg_addr, reg_val, NULL, 0);
}

/****************************************************************************
 * Name: ht16k33_update_blink_onoff
 *
 * Description:
 *   Update the display setup register, which controls global LED enable
 *   and global blink settings.
 *
 ****************************************************************************/

static int ht16k33_update_blink_onoff(FAR struct ht16k33_dev_s *priv)
{
  int ret = -1;
  uint8_t disp_setup = 0;

  if (priv->on)
    {
      disp_setup |= HT16K33_DISP_ON;
    }

  disp_setup |= priv->blink;

  ret = ht16k33_i2c_write_reg(priv, HT16K33_DISP, disp_setup);
  if (ret < 0)
    {
      lederr("ERROR: Could not set HT16K33 display options.\n");
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
 * Input Parameters:
 *   priv       - Reference to an instance of the ht16k33 device
 *   on         - If true, all LEDs will be enabled.  If false, all
 *                LEDs will be disabled.
 *
 ****************************************************************************/

static int ht16k33_set_led_onoff(FAR struct ht16k33_dev_s *priv,
                                      bool const on)
{
  int ret;
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
 * Input Parameters:
 *   priv       - Reference to an instance of the ht16k33 device
 *   blinkrate  - one of HT16K33_DISP_BLINKOFF, HT16K33_DISP_BLINK2HZ,
 *                HT16K33_DISP_BLINK1HZ, or HT16K33_DISP_BLINK05HZ
 *
 ****************************************************************************/

static int ht16k33_set_led_blink_rate(FAR struct ht16k33_dev_s *priv,
                                      uint8_t const blinkrate)
{
  int ret;
  priv->blink = blinkrate;

  ret = ht16k33_update_blink_onoff(priv);

  return ret;
}

/****************************************************************************
 * Name: ht16k33_set_brightness
 *
 * Description:
 *   Set the brightness for all LEDs
 *
 * Input Parameters:
 *   priv       - Reference to an instance of the ht16k33 device
 *   brightness - Brightness in the range [0, 15]
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int ht16k33_set_led_brightness(FAR struct ht16k33_dev_s *priv,
                                      uint8_t brightness)
{
  int ret;

  if (brightness > HT16K33_DIM_MAX)
    {
      brightness = HT16K33_DIM_MAX;
    }

  ret = ht16k33_i2c_write_reg(priv, HT16K33_DIM, brightness);
  if (ret < 0)
    {
      lederr("ERROR: Could not set HT16K33 brightness.\n");
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: ht16k33_set_standby
 *
 * Description:
 *   Sets the standby state of the HT16K33 device.
 *
 * Input Parameters:
 *   priv       - Reference to an instance of the ht16k33 device
 *   standby    - if true, device will enter standby.  If false,
 *                device will enter normal operation mode.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int ht16k33_set_standby(FAR struct ht16k33_dev_s *priv,
                                      bool const standby)
{
  int ret;
  uint8_t system_setup = 0;

  if (!standby)
    {
      system_setup |= HT16K33_SS_NORMAL;
    }

  ret = ht16k33_i2c_write_reg(priv, HT16K33_SS, system_setup);
  if (ret < 0)
    {
      lederr("ERROR: Could not set HT16K33 standby.\n");
      return ret;
    }
  else
    {
      /* Allow 25ms (~2 frame intervals) for the internal
       * oscillator to change state */

      nxsig_usleep(25 * USEC_PER_MSEC);
    }

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
  int ret;

  /* Wake up the HT16K33 by enabling the oscillator. */

  ret = ht16k33_set_standby(priv, false);
  if (ret < 0)
    {
      lederr("ERROR: Could not set HT16K33 standby mode\n");
      return ret;
    }

  /* Configure the HT16K33 ROW/INT pin to row output mode */

  ret = ht16k33_i2c_write_reg(priv, HT16K33_RIS, HT16K33_RIS_OUT);
  if (ret < 0)
    {
      lederr("ERROR: Could not set initial config for HT16K33_RIS\n");
      return ret;
    }

  /* Set LEDs to full brightness */
  ret = ht16k33_set_led_brightness(priv, HT16K33_DIM_MAX);
  if (ret < 0)
    {
      lederr("ERROR: Could not set HT16K33 brightness\n");
      return ret;
    }

  /* Enable LEDs & set blink to off */
  priv->blink = HT16K33_DISP_BLINKOFF;
  priv->on = true;
  ret = ht16k33_update_blink_onoff(priv);
  if (ret < 0)
    {
      lederr("ERROR: Could not update HT16K33 system setup\n");
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
  int ret;

  /* Turn all led drivers off */

  ret = ht16k33_set_led_onoff(priv, false);
  if (ret < 0)
    {
      lederr("ERROR: Could not set HT16K33 LEDs to off\n");
      return ret;
    }

  /* Send the HT16K33 back to standby mode */

  ret = ht16k33_set_standby(priv, false);
  if (ret < 0)
    {
      lederr("ERROR: Could not set HT16K33 standby mode\n");
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
      /* Set the brightness of all LEDs. Arg: brightness (0-15) */

    case PWMIOC_SETLED_BRIGHTNESS:
      {

        /* Set the brightness of the led */

        ret = ht16k33_set_led_brightness(priv, (uint8_t) arg);
      }
      break;

      /* Set the global on/off state of the LEDs. Arg: 0 = off / 1 = on */

    case PWMIOC_SETLED_ONOFF:
      {

        ret = ht16k33_set_led_onoff(priv, (bool) arg);
      }
      break;

      /* Set the standby state of the device. Arg: 0 = run / 1 = standby */

    case PWMIOC_SETLED_STANDBY:
      {

        ret = ht16k33_set_standby(priv, (bool) arg);
      }
      break;

      /* Set the global blink rate for the LEDs.
       * Arg: one of HT16K33_DISP_BLINKOFF, HT16K33_DISP_BLINK2HZ,
       * HT16K33_DISP_BLINK1HZ, HT16K33_DISP_BLINK05HZ */

    case PWMIOC_SETLED_BLINK:
      {

        ret = ht16k33_set_led_blink_rate(priv, (uint8_t)arg);
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
 * Name: ht16k33_read
 ****************************************************************************/

static ssize_t ht16k33_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
    /* Revisit: This device can also read switches;
     * read could return switch data */
  return -ENOSYS;
}

/****************************************************************************
 * Name: ht16k33_write
 ****************************************************************************/

static ssize_t ht16k33_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ht16k33_dev_s *priv = inode->i_private;
  int written;

  if (buffer == NULL)
    {
      lederr("ERROR: Buffer is null\n");
      return -1;
    }

  if (buflen < 1)
    {
      lederr("ERROR: At least 1 byte required!\n");
      return -1;
    }

  /* Write to display RAM, always restarting from address 0 */

  written = ht16k33_i2c_write_data(priv, HT16K33_DDAP, 0, buffer, buflen);

  return written;
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
