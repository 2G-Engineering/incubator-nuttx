/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/analog/ioctl.h>

#ifndef __INCLUDE_NUTTX__ADS111X_H
#define __INCLUDE_NUTTX__ADS111X_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/
/* Prerequisites:
 * CONFIG_I2C
 * Must be defined to enable I2C support
 * CONFIG_ADC
 * Must be defined to enable ADC support
 */

#ifndef CONFIG_I2C
#  error "I2C support is required (CONFIG_I2C)"
#endif

#ifndef CONFIG_ADC
#  error "ADC support is required (CONFIG_ADC)"
#endif

/* ADS111x I2C Default Address (ADDR pin to GND) */
#define ADS111X_DEFAULT_ADDR    0x48

/* ADS111x Register Definitions ****************************/
#define ADS111X_REG_CONVERSION  0x00  /* Conversion Result Register */
#define ADS111X_REG_CONFIG      0x01  /* Configuration Register */
#define ADS111X_REG_LO_THRESH   0x02  /* Low Threshold Register (ADS1114/5 only) */
#define ADS111X_REG_HI_THRESH   0x03  /* High Threshold Register (ADS1114/5 only) */

/* ADS111x Config Register Bit Definitions *************/

/* Operational Status / Single-Shot Conversion Start (Bit 15) */
#define ADS111X_CONFIG_OS_SHIFT         15
#define ADS111X_CONFIG_OS_MASK          (1 << ADS111X_CONFIG_OS_SHIFT)
#  define ADS111X_CONFIG_OS_NOEFFECT    (0 << ADS111X_CONFIG_OS_SHIFT) /* Write: No effect */
#  define ADS111X_CONFIG_OS_START       (1 << ADS111X_CONFIG_OS_SHIFT) /* Write: Start single conversion */
#  define ADS111X_CONFIG_OS_BUSY        (0 << ADS111X_CONFIG_OS_SHIFT) /* Read: Device busy */
#  define ADS111X_CONFIG_OS_NOTBUSY     (1 << ADS111X_CONFIG_OS_SHIFT) /* Read: Device not busy (default) */

/* Programmable Gain Amplifier Configuration (Bits 11-9) */
#define ADS111X_CONFIG_PGA_SHIFT        9
#define ADS111X_CONFIG_PGA_MASK         (7 << ADS111X_CONFIG_PGA_SHIFT)
#  define ADS111X_CONFIG_PGA_6_144V     (0 << ADS111X_CONFIG_PGA_SHIFT) /* FSR = +/-6.144V */
#  define ADS111X_CONFIG_PGA_4_096V     (1 << ADS111X_CONFIG_PGA_SHIFT) /* FSR = +/-4.096V */
#  define ADS111X_CONFIG_PGA_2_048V     (2 << ADS111X_CONFIG_PGA_SHIFT) /* FSR = +/-2.048V (default) */
#  define ADS111X_CONFIG_PGA_1_024V     (3 << ADS111X_CONFIG_PGA_SHIFT) /* FSR = +/-1.024V */
#  define ADS111X_CONFIG_PGA_0_512V     (4 << ADS111X_CONFIG_PGA_SHIFT) /* FSR = +/-0.512V */
#  define ADS111X_CONFIG_PGA_0_256V_1   (5 << ADS111X_CONFIG_PGA_SHIFT) /* FSR = +/-0.256V */
#  define ADS111X_CONFIG_PGA_0_256V_2   (6 << ADS111X_CONFIG_PGA_SHIFT) /* FSR = +/-0.256V */
#  define ADS111X_CONFIG_PGA_0_256V_3   (7 << ADS111X_CONFIG_PGA_SHIFT) /* FSR = +/-0.256V */

/* Device Operating Mode (Bit 8) */
#define ADS111X_CONFIG_MODE_SHIFT       8
#define ADS111X_CONFIG_MODE_MASK        (1 << ADS111X_CONFIG_MODE_SHIFT)
#  define ADS111X_CONFIG_MODE_CONTINUOUS (0 << ADS111X_CONFIG_MODE_SHIFT) /* Continuous conversion mode */
#  define ADS111X_CONFIG_MODE_SINGLESHOT (1 << ADS111X_CONFIG_MODE_SHIFT) /* Single-shot mode (default) */

/* Data Rate (Bits 7-5) */
#define ADS111X_CONFIG_DR_SHIFT         5
#define ADS111X_CONFIG_DR_MASK          (7 << ADS111X_CONFIG_DR_SHIFT)
#  define ADS111X_CONFIG_DR_8SPS        (0 << ADS111X_CONFIG_DR_SHIFT) /* 8 Samples per second */
#  define ADS111X_CONFIG_DR_16SPS       (1 << ADS111X_CONFIG_DR_SHIFT) /* 16 Samples per second */
#  define ADS111X_CONFIG_DR_32SPS       (2 << ADS111X_CONFIG_DR_SHIFT) /* 32 Samples per second */
#  define ADS111X_CONFIG_DR_64SPS       (3 << ADS111X_CONFIG_DR_SHIFT) /* 64 Samples per second */
#  define ADS111X_CONFIG_DR_128SPS      (4 << ADS111X_CONFIG_DR_SHIFT) /* 128 Samples per second (default) */
#  define ADS111X_CONFIG_DR_250SPS      (5 << ADS111X_CONFIG_DR_SHIFT) /* 250 Samples per second */
#  define ADS111X_CONFIG_DR_475SPS      (6 << ADS111X_CONFIG_DR_SHIFT) /* 475 Samples per second */
#  define ADS111X_CONFIG_DR_860SPS      (7 << ADS111X_CONFIG_DR_SHIFT) /* 860 Samples per second */

/* Comparator Mode (Bit 4, ADS1114/5 only) */
#define ADS111X_CONFIG_COMP_MODE_SHIFT  4
#define ADS111X_CONFIG_COMP_MODE_MASK   (1 << ADS111X_CONFIG_COMP_MODE_SHIFT)
#  define ADS111X_CONFIG_COMP_MODE_TRAD  (0 << ADS111X_CONFIG_COMP_MODE_SHIFT) /* Traditional comparator (default) */
#  define ADS111X_CONFIG_COMP_MODE_WINDOW (1 << ADS111X_CONFIG_COMP_MODE_SHIFT) /* Window comparator */

/* Comparator Polarity (Bit 3, ADS1114/5 only) */
#define ADS111X_CONFIG_COMP_POL_SHIFT   3
#define ADS111X_CONFIG_COMP_POL_MASK    (1 << ADS111X_CONFIG_COMP_POL_SHIFT)
#  define ADS111X_CONFIG_COMP_POL_LOW   (0 << ADS111X_CONFIG_COMP_POL_SHIFT) /* Active low (default) */
#  define ADS111X_CONFIG_COMP_POL_HIGH  (1 << ADS111X_CONFIG_COMP_POL_SHIFT) /* Active high */

/* Latching Comparator (Bit 2, ADS1114/5 only) */
#define ADS111X_CONFIG_COMP_LAT_SHIFT   2
#define ADS111X_CONFIG_COMP_LAT_MASK    (1 << ADS111X_CONFIG_COMP_LAT_SHIFT)
#  define ADS111X_CONFIG_COMP_LAT_NON   (0 << ADS111X_CONFIG_COMP_LAT_SHIFT) /* Non-latching (default) */
#  define ADS111X_CONFIG_COMP_LAT_LATCH (1 << ADS111X_CONFIG_COMP_LAT_SHIFT) /* Latching */

/* Comparator Queue and Disable (Bits 1-0, ADS1114/5 only) */
#define ADS111X_CONFIG_COMP_QUE_SHIFT   0
#define ADS111X_CONFIG_COMP_QUE_MASK    (3 << ADS111X_CONFIG_COMP_QUE_SHIFT)
#  define ADS111X_CONFIG_COMP_QUE_1CONV (0 << ADS111X_CONFIG_COMP_QUE_SHIFT) /* Assert after 1 conversion */
#  define ADS111X_CONFIG_COMP_QUE_2CONV (1 << ADS111X_CONFIG_COMP_QUE_SHIFT) /* Assert after 2 conversions */
#  define ADS111X_CONFIG_COMP_QUE_4CONV (2 << ADS111X_CONFIG_COMP_QUE_SHIFT) /* Assert after 4 conversions */
#  define ADS111X_CONFIG_COMP_QUE_DISABLE (3 << ADS111X_CONFIG_COMP_QUE_SHIFT) /* Disable comparator (default) */

/* ADS1114 Default Config Value (Reset: 8583h) */
/* OS=1 (Not busy), MUX=N/A, PGA=2 (2.048V), MODE=1 (Single-shot),
 * DR=4 (128SPS), COMP_MODE=0 (Trad), COMP_POL=0 (Low), COMP_LAT=0 (Non),
 * COMP_QUE=3 (Disable) */
#define ADS1114_CONFIG_DEFAULT          (ADS111X_CONFIG_PGA_4_096V | ADS111X_CONFIG_DR_128SPS | ADS111X_CONFIG_MODE_SINGLESHOT)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IOCTL Commands
 * Cmd: ANIOC_ADS7828_SET_REF       Arg: enum ads7828_ref_e
 * Cmd: ANIOC_ADS7828_MODE          Arg: ads7828_mode_e
 * Cmd: ANIOC_ADS7828_POWER_SAVE    Arg: bool value
 * Cmd: ANIOC_ADS7828_ADD_CHAN      Arg: uint8_t value
 * Cmd: ANIOC_ADS7828_REMOVE_CHAN   ARG: uint8_t value
 * Cmd: ANIOC_ADS7828_READ_CHANNEL  Arg: struct adc_msg_s *channel
 */

#define ANIOC_ADS111X_SET_RANGE   _ANIOC(AN_ADS111X_FIRST + 0)
#define ANIOC_ADS111X_SET_DR      _ANIOC(AN_ADS111X_FIRST + 1)
#define ANIOC_ADS111X_SET_MODE    _ANIOC(AN_ADS111X_FIRST + 2)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Structure describing the configuration of the ADS111x device */

struct ads111x_config_s
{
  uint8_t address;      /* I2C address of the device */
  uint16_t fsr_mv;      /* Full scale range of the device, in millivolts */
  uint32_t frequency;   /* I2C frequency */
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
 * Name: ads111x_register
 *
 * Description:
 * Register the ADS111x character device as 'devpath'
 *
 * Input Parameters:
 * devpath - The full path to the driver to register. E.g., "/dev/adc0"
 * i2c     - An instance of the I2C interface to use to communicate with
 * the ADS111x.
 * config  - Device configuration.
 *
 * Returned Value:
 * Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ads111x_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                     FAR const struct ads111x_config_s *config);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_ADS111X_H */
