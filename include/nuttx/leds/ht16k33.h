/****************************************************************************
 * include/nuttx/leds/ht16k33.h
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

#ifndef __INCLUDE_NUTTX_LEDS_HT16K33_H
#define __INCLUDE_NUTTX_LEDS_HT16K33_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

/* Configuration
 * CONFIG_I2C - Enables support for I2C drivers
 * CONFIG_HT16K33 - Enables support for the HT16K33 driver
 */

#if defined(CONFIG_I2C) && defined(CONFIG_HT16K33)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C definitions */

#define I2C_BUS_FREQ_HZ            (400000)

/* HT16K33 register bitmasks */

#define HT16K33_REG_ADDR_MASK   (0xF0)
#define HT16K33_REG_DATA_MASK   (0x0F)

/* HT16K33 register addresses */

#define HT16K33_DDAP            (0x00) /* Display data address pointer*/
#define HT16K33_SS              (0x20) /* System setup */
#define HT16K33_KDAP            (0x40) /* Key data address pointer*/
#define HT16K33_IFAP            (0x60) /* Int flag address pointer*/
#define HT16K33_DISP            (0x80) /* Display setup */
#define HT16K33_RIS             (0xa0) /* ROW/INT set */
#define HT16K33_DIM             (0xe0) /* Dimming set */

/* HT16K33 register values */

#define HT16K33_DISP_RAM_SIZE   (0x0f) /* Number of bytes in display RAM */

#define HT16K33_SS_STANDBY      (0x00) /* Standby mode */
#define HT16K33_SS_NORMAL       (0x01) /* Normal operation mode */

#define HT16K33_DISP_OFF        (0x00) /* Display off */
#define HT16K33_DISP_ON         (0x01) /* Display on */
#define HT16K33_DISP_BLINKOFF   (0x00) /* Blink off */
#define HT16K33_DISP_BLINK2HZ   (0x02) /* Blink at 2Hz */
#define HT16K33_DISP_BLINK1HZ   (0x04) /* Blink at 1Hz */
#define HT16K33_DISP_BLINK05HZ  (0x06) /* Blink at 0.5Hz */

#define HT16K33_RIS_OUT         (0x00) /* INT/ROW pin is row driver output */
#define HT16K33_RIS_INTL        (0x01) /* INT/ROW pin is int output, active low */
#define HT16K33_RIS_INTH        (0x03) /* INT/ROW pin is int output, active high */

#define HT16K33_DIM_MIN         (0x00) /* Minimum brightness level */
#define HT16K33_DIM_MAX         (0x0f) /* Maximum brightness level */

/* IOCTL commands */

#define PWMIOC_SETLED_BRIGHTNESS   _PWMIOC(1) /* Arg: uint8_t between 0 and 15 */
#define PWMIOC_SETLED_ONOFF        _PWMIOC(2) /* Arg: bool: true for on, false for off */
#define PWMIOC_SETLED_STANDBY      _PWMIOC(3) /* Arg: bool: true for standby, false for run */
#define PWMIOC_SETLED_BLINK        _PWMIOC(4) /* Arg: uint8_t blink rate */

/****************************************************************************
 * Public Types
 ****************************************************************************/


/****************************************************************************
 * Forward declarations
 ****************************************************************************/

struct i2c_master_s;

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

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
                       uint8_t const ht16k33_i2c_addr);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_I2C_HT16K33 */
#endif /* __INCLUDE_NUTTX_LEDS_HT16K33_H */
