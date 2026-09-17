/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <fixedmath.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/analog/ads111x.h>
#include <nuttx/mutex.h>
#include <nuttx/signal.h>

#if defined(CONFIG_ADC) && defined(CONFIG_I2C) && defined(CONFIG_ADC_ADS111X)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Timeouts for single-shot conversion (dependent on data rate) */
/* Max conversion time is 1/8SPS = 125ms */
#define ADS111X_MAX_CONVERSION_TIME_US (130 * 1000)
#define ADS111X_CONVERSION_POLL_DELAY_US (1 * 1000) /* 1 ms poll delay */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Represents the state of the ADS111x device */

struct ads111x_dev_s
{
  struct adc_dev_s            adc;      /* Generic ADC device */
  FAR struct i2c_master_s     *i2c;     /* I2C interface */
  uint8_t                     addr;     /* I2C address */
  uint32_t                    freq;     /* I2C frequency */
  mutex_t                     datalock; /* Protects device configuration */
  uint16_t                    config;   /* Last written config register value */
  uint16_t                    fsr_mv;   /* Full scale range in mV for scaling */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C Helpers */
static int ads111x_write_reg(FAR struct ads111x_dev_s *priv, uint8_t regaddr,
                             uint16_t regval);
static int ads111x_read_reg(FAR struct ads111x_dev_s *priv, uint8_t regaddr,
                            FAR uint16_t *regval);

/* Character driver methods */
static int     ads111x_open(FAR struct file *filep);
static int     ads111x_close(FAR struct file *filep);
static ssize_t ads111x_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static ssize_t ads111x_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);
static int     ads111x_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg);

/* ADC methods */
static int ads111x_setup(FAR struct adc_dev_s *dev);
static void ads111x_shutdown(FAR struct adc_dev_s *dev);
static void ads111x_rxint(FAR struct adc_dev_s *dev, bool enable);
static int ads111x_trigger(FAR struct adc_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_ads111xfops =
{
  .open   = ads111x_open,
  .close  = ads111x_close,
  .read   = ads111x_read,
  .write  = NULL, /* Not supported */
  .seek   = NULL,
  .ioctl  = ads111x_ioctl,
};

static const struct adc_ops_s g_ads111xadcops =
{
  .ao_setup    = ads111x_setup,
  .ao_shutdown = ads111x_shutdown,
  .ao_rxint    = ads111x_rxint,  /* Optional: Requires ALERT/RDY handling */
//  .ao_trigger  = ads111x_trigger,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ads111x_write_reg
 *
 * Description:
 * Write to a 16-bit register of the ADS111x.
 *
 ****************************************************************************/
static int ads111x_write_reg(FAR struct ads111x_dev_s *priv, uint8_t regaddr,
                             uint16_t regval)
{
  struct i2c_msg_s msg[1];
  uint8_t txbuffer[3];
  int ret;

  sninfo("regaddr: %02x regval: %04x\n", regaddr, regval);

  /* Point to the target register */
  txbuffer[0] = regaddr & 0x03; /* Pointer register address */
  txbuffer[1] = (regval >> 8) & 0xff; /* MSB */
  txbuffer[2] = regval & 0xff;        /* LSB */

  msg[0].frequency = priv->freq;
  msg[0].addr      = priv->addr;
  msg[0].flags     = 0; /* Write */
  msg[0].buffer    = txbuffer;
  msg[0].length    = 3;

  ret = I2C_TRANSFER(priv->i2c, msg, 1);
  if (ret < 0)
    {
      snerr("ERROR: I2C_TRANSFER failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: ads111x_read_reg
 *
 * Description:
 * Read from a 16-bit register of the ADS111x.
 *
 ****************************************************************************/
static int ads111x_read_reg(FAR struct ads111x_dev_s *priv, uint8_t regaddr,
                           FAR uint16_t *regval)
{
  struct i2c_msg_s msg[2];
  uint8_t pointer_reg = regaddr & 0x03;
  uint8_t rxbuffer[2];
  int ret;

  sninfo("regaddr: %02x\n", regaddr);

  /* Write the pointer register to select the register to read */
  msg[0].frequency = priv->freq;
  msg[0].addr      = priv->addr;
  msg[0].flags     = 0; /* Write */
  msg[0].buffer    = &pointer_reg;
  msg[0].length    = 1;

  /* Read the 16-bit register value */
  msg[1].frequency = priv->freq;
  msg[1].addr      = priv->addr;
  msg[1].flags     = I2C_M_READ; /* Read */
  msg[1].buffer    = rxbuffer;
  msg[1].length    = 2;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      snerr("ERROR: I2C_TRANSFER failed: %d\n", ret);
      return ret;
    }

  /* Combine MSB and LSB */
  *regval = ((uint16_t)rxbuffer[0] << 8) | rxbuffer[1];
  sninfo("regval: %04x\n", *regval);

  return OK;
}

/****************************************************************************
 * Name: ads111x_get_fsr_mv
 *
 * Description:
 * Get the Full Scale Range in millivolts based on PGA setting.
 *
 ****************************************************************************/
static uint16_t ads111x_get_fsr_mv(uint16_t pga_setting)
{
    uint16_t fsr_mv;

    switch (pga_setting)
    {
        case ADS111X_CONFIG_PGA_6_144V:
            fsr_mv = 6144;
            break;
        case ADS111X_CONFIG_PGA_4_096V:
            fsr_mv = 4096;
            break;
        case ADS111X_CONFIG_PGA_2_048V:
            fsr_mv = 2048;
            break;
        case ADS111X_CONFIG_PGA_1_024V:
            fsr_mv = 1024;
            break;
        case ADS111X_CONFIG_PGA_0_512V:
            fsr_mv = 512;
            break;
        case ADS111X_CONFIG_PGA_0_256V_1:
        case ADS111X_CONFIG_PGA_0_256V_2:
        case ADS111X_CONFIG_PGA_0_256V_3:
            fsr_mv = 256;
            break;
        default:
            snwarn("WARNING: Unknown PGA setting %04x, defaulting to 2.048V\n", pga_setting);
            fsr_mv = 2048;
            break;
    }
    return fsr_mv;
}

/****************************************************************************
 * Name: ads111x_open
 *
 * Description:
 * Standard character driver open method.
 *
 ****************************************************************************/
static int ads111x_open(FAR struct file *filep)
{
  /* Nothing specific to do on open */
  return OK;
}

/****************************************************************************
 * Name: ads111x_close
 *
 * Description:
 * Standard character driver close method.
 *
 ****************************************************************************/
static int ads111x_close(FAR struct file *filep)
{
  /* Nothing specific to do on close */
  return OK;
}

/****************************************************************************
 * Name: ads111x_read
 *
 * Description:
 * Standard character driver read method. Reads a single sample.
 *
 ****************************************************************************/
static ssize_t ads111x_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  FAR struct inode         *inode = filep->f_inode;
  FAR struct ads111x_dev_s *priv  = inode->i_private;
  struct adc_msg_s          adc_msg;
  int                       ret;
  int16_t                   raw_value; /* ADS111x uses 16-bit signed */

  /* Check if the buffer is large enough */
  if (buflen < sizeof(struct adc_msg_s))
    {
      snerr("ERROR: Not enough memory for reading data\n");
      return -ENOSPC;
    }

  ret = nxmutex_lock(&priv->datalock);
  if (ret < 0)
    {
      return ret;
    }

  /* Trigger a conversion if in single-shot mode */
  if ((priv->config & ADS111X_CONFIG_MODE_MASK) == ADS111X_CONFIG_MODE_SINGLESHOT)
    {
      ret = ads111x_trigger(&priv->adc);
      if (ret < 0)
        {
          snerr("ERROR: Failed to trigger conversion: %d\n", ret);
          nxmutex_unlock(&priv->datalock);
          return ret;
        }
        /* Wait for conversion complete - simple polling */
        time_t start_time = clock_systime_ticks();
        time_t timeout_ticks = USEC2TICK(ADS111X_MAX_CONVERSION_TIME_US);
        uint16_t config_read;
        do
        {
            ret = ads111x_read_reg(priv, ADS111X_REG_CONFIG, &config_read);
            if (ret < 0)
            {
                nxmutex_unlock(&priv->datalock);
                return ret;
            }
            /* Check OS bit */
            if ((config_read & ADS111X_CONFIG_OS_MASK) == ADS111X_CONFIG_OS_NOTBUSY)
            {
                break; /* Conversion complete */
            }
            nxsig_usleep(ADS111X_CONVERSION_POLL_DELAY_US);
        } while (clock_systime_ticks() - start_time < timeout_ticks);

        if ((config_read & ADS111X_CONFIG_OS_MASK) != ADS111X_CONFIG_OS_NOTBUSY)
        {
            snerr("ERROR: Conversion timeout\n");
            nxmutex_unlock(&priv->datalock);
            return -ETIMEDOUT;
        }
    }
    /* else: continuous mode, assume data is ready or will be soon */
    /* A more robust implementation might check the ALERT/RDY pin or OS bit */

  /* Read the conversion result */
  ret = ads111x_read_reg(priv, ADS111X_REG_CONVERSION, (uint16_t *)&raw_value);
  if (ret < 0)
    {
      snerr("ERROR: Failed to read conversion register: %d\n", ret);
      nxmutex_unlock(&priv->datalock);
      return ret;
    }

  nxmutex_unlock(&priv->datalock);

  /* Populate the ADC message */
  /* The ADS1114 only has one differential channel (AIN0/AIN1) or single-ended (AIN0 vs GND) */
  adc_msg.am_channel = 0; /* Only one channel */
  adc_msg.am_data = raw_value;

  memcpy(buffer, &adc_msg, sizeof(struct adc_msg_s));

  return sizeof(struct adc_msg_s);
}

/****************************************************************************
 * Name: ads111x_ioctl
 *
 * Description:
 * Handle IOCTL commands
 *
 ****************************************************************************/
static int ads111x_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ads111x_dev_s *priv = inode->i_private;
  int ret = OK;
  uint16_t new_config;
  uint16_t pga_setting;

  ret = nxmutex_lock(&priv->datalock);
  if (ret < 0)
    {
      return ret;
    }

  new_config = priv->config;

  switch (cmd)
    {
      case ANIOC_ADS111X_SET_RANGE: /* Assuming a generic range IOCTL */
        {
          uint16_t target_fsr_mv = (uint16_t)arg;
          uint16_t best_pga = priv->config & ADS111X_CONFIG_PGA_MASK;
          uint16_t best_fsr = priv->fsr_mv;
          uint16_t current_fsr;

          sninfo("ANIOC_SET_RANGE: target=%lu mV\n", arg);

          /* Find the smallest FSR that is >= target_fsr_mv */
          if (target_fsr_mv <= 256)
              { best_pga = ADS111X_CONFIG_PGA_0_256V_1; best_fsr = 256; }
          else if (target_fsr_mv <= 512)
              { best_pga = ADS111X_CONFIG_PGA_0_512V; best_fsr = 512; }
          else if (target_fsr_mv <= 1024)
              { best_pga = ADS111X_CONFIG_PGA_1_024V; best_fsr = 1024; }
          else if (target_fsr_mv <= 2048)
              { best_pga = ADS111X_CONFIG_PGA_2_048V; best_fsr = 2048; }
          else if (target_fsr_mv <= 4096)
              { best_pga = ADS111X_CONFIG_PGA_4_096V; best_fsr = 4096; }
          else /* (target_fsr_mv <= 6144 or greater) */
              { best_pga = ADS111X_CONFIG_PGA_6_144V; best_fsr = 6144; }

          /* Apply the new PGA setting */
          new_config &= ~ADS111X_CONFIG_PGA_MASK;
          new_config |= best_pga;
          priv->fsr_mv = best_fsr;
          sninfo("Set PGA=%04x, FSR=%u mV\n", best_pga, best_fsr);
        }
        break;

      /* Example: Setting Data Rate */
      case ANIOC_ADS111X_SET_DR:
        new_config &= ~ADS111X_CONFIG_DR_MASK;
        new_config |= (arg << ADS111X_CONFIG_DR_SHIFT) & ADS111X_CONFIG_DR_MASK;
        break;

      /* Example: Setting Mode */
      case ANIOC_ADS111X_SET_MODE:
        new_config &= ~ADS111X_CONFIG_MODE_MASK;
        new_config |= (arg << ADS111X_CONFIG_MODE_SHIFT) & ADS111X_CONFIG_MODE_MASK;
        break;

      /* Add other IOCTLs for comparator settings if needed */

      default:
        snerr("ERROR: Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        goto errout_with_lock;
    }

  /* Write the new configuration if it changed */
  if (new_config != priv->config)
    {
        sninfo("Writing new config: %04x (old: %04x)\n", new_config, priv->config);
        ret = ads111x_write_reg(priv, ADS111X_REG_CONFIG, new_config);
        if (ret < 0)
        {
            snerr("ERROR: Failed to write config register: %d\n", ret);
            /* Restore old FSR if config write failed */
            pga_setting = priv->config & ADS111X_CONFIG_PGA_MASK;
            priv->fsr_mv = ads111x_get_fsr_mv(pga_setting);
            goto errout_with_lock;
        }
        priv->config = new_config; /* Update cached config */
    }

errout_with_lock:
  nxmutex_unlock(&priv->datalock);
  return ret;
}

/****************************************************************************
 * Name: ads111x_setup
 *
 * Description:
 * Configure the ADC device. This method is called the first time that
 * the ADC device is opened.
 *
 ****************************************************************************/
static int ads111x_setup(FAR struct adc_dev_s *dev)
{
  FAR struct ads111x_dev_s *priv = (FAR struct ads111x_dev_s *)dev;
  int ret;
  uint16_t pga_setting;

  /* Write initial configuration */

  sninfo("Setting initial config: %04x\n", priv->config);

  ret = ads111x_write_reg(priv, ADS111X_REG_CONFIG, priv->config);
  if (ret < 0)
    {
      snerr("ERROR: Failed to write initial config: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: ads111x_shutdown
 *
 * Description:
 * Disable the ADC device. This method is called when the ADC device is
 * closed.
 *
 ****************************************************************************/
static void ads111x_shutdown(FAR struct adc_dev_s *dev)
{
  FAR struct ads111x_dev_s *priv = (FAR struct ads111x_dev_s *)dev;
  uint16_t shutdown_config;

  /* Put the device into power-down (single-shot mode) */
  shutdown_config = priv->config;
  shutdown_config &= ~ADS111X_CONFIG_MODE_MASK;
  shutdown_config |= ADS111X_CONFIG_MODE_SINGLESHOT;

  /* Optional: Set OS bit to ensure it's ready for next trigger if needed,
   * but generally power-down is sufficient. The default state after reset
   * is powered down */
  /* shutdown_config |= ADS111X_CONFIG_OS_NOTBUSY; */

  sninfo("Shutting down. Writing config: %04x\n", shutdown_config);

  /* Ignore return value - best effort shutdown */
  ads111x_write_reg(priv, ADS111X_REG_CONFIG, shutdown_config);
}

/****************************************************************************
 * Name: ads111x_rxint
 *
 * Description:
 * Enable or disable the ADC interrupt. (Placeholder - requires ALERT/RDY pin handling)
 *
 ****************************************************************************/
static void ads111x_rxint(FAR struct adc_dev_s *dev, bool enable)
{
  FAR struct ads111x_dev_s *priv = (FAR struct ads111x_dev_s *)dev;

  sninfo("rxint enable: %d\n", enable);

  /* Requires configuration of the ALERT/RDY pin and GPIO interrupt handling */
  /* Example: configure COMP_QUE=0, Hi_thresh MSB=1, Lo_thresh MSB=0 for RDY mode */
  /* Then attach/detach the GPIO interrupt handler */

  snwarn("WARNING: rxint functionality not fully implemented.\n");
}

/****************************************************************************
 * Name: ads111x_trigger
 *
 * Description:
 * Trigger a conversion in single-shot mode.
 *
 ****************************************************************************/
static int ads111x_trigger(FAR struct adc_dev_s *dev)
{
  FAR struct ads111x_dev_s *priv = (FAR struct ads111x_dev_s *)dev;
  int ret = OK;
  uint16_t reg_val;

  sninfo("Triggering conversion...\n");

  /* Check if in single-shot mode */
  if ((priv->config & ADS111X_CONFIG_MODE_MASK) != ADS111X_CONFIG_MODE_SINGLESHOT)
    {
      sninfo("Device in continuous mode, trigger has no effect.\n");
      return OK; /* Or return error? Depends on desired behavior */
    }

  /* Set the OS bit to start a conversion */
  reg_val = priv->config | ADS111X_CONFIG_OS_START;

  ret = ads111x_write_reg(priv, ADS111X_REG_CONFIG, reg_val);
  if (ret < 0)
    {
      snerr("ERROR: Failed to write config to trigger: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ads111x_register
 *
 * Description:
 * Register the ADS111x character device as 'devpath'
 *
 ****************************************************************************/
int ads111x_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                     FAR const struct ads111x_config_s *config)
{
  FAR struct ads111x_dev_s *priv;
  int ret;

  /* Sanity check */
  DEBUGASSERT(i2c != NULL);
  DEBUGASSERT(devpath != NULL);
  DEBUGASSERT(config != NULL);

  /* Initialize the device structure */
  priv = (FAR struct ads111x_dev_s *)kmm_zalloc(sizeof(struct ads111x_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c            = i2c;
  priv->addr           = config->address;
  priv->freq           = config->frequency;
  priv->adc.ad_ops     = &g_ads111xadcops;
  priv->adc.ad_priv    = priv;
  priv->config         = ADS1114_CONFIG_DEFAULT; /* Will be set in setup */
  priv->fsr_mv         = config->fsr_mv;

  nxmutex_init(&priv->datalock);

  /* Register the character driver */
#if 0
  ret = adc_register(devpath, &priv->adc);
#else
  ret = register_driver(devpath, &g_ads111xfops, 0666, priv);
#endif
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      nxmutex_destroy(&priv->datalock);
      kmm_free(priv);
      return ret;
    }

  sninfo("ADS111x driver registered at %s\n", devpath);
  return OK;
}

#endif /* CONFIG_ADC && CONFIG_I2C && CONFIG_SENSORS_ADS111X */
