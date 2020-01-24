/************************************************************************************
 * arch/arm/src/lpc17xx_40xx/lpc17_40_rtc.h
 *
 *   Copyright (C) 2010, 2012-2013, 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC17XX_40XX_LPC17_40_RTC_H
#define __ARCH_ARM_SRC_LPC17XX_40XX_LPC17_40_RTC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "hardware/lpc17_40_rtc.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define RTC_MAGIC               (0xfacefeed)
#define RTC_MAGIC_TIME_SET      (0xf00dface)
#define RTC_MAGIC_REG            LPC17_40_RTC_GPREG0

/************************************************************************************
 * Public Types
 ************************************************************************************/

/* The form of an alarm callback */

typedef void (*alarmcb_t)(void);

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: lpc17_40_rtc_irqinitialize
 *
 * Description:
 *   Initialize IRQs for RTC, not possible during up_rtc_initialize because
 *   up_irqinitialize is called later.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

int lpc17_40_rtc_irqinitialize(void);

/************************************************************************************
 * Name: lpc17_40_rtc_setalarm
 *
 * Description:
 *   Set up an alarm.
 *
 * Input Parameters:
 *   tp - the time to set the alarm
 *   callback - the function to call when the alarm expires.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

#ifdef CONFIG_RTC_ALARM
struct tm;
int lpc17_40_rtc_setalarm(FAR const struct tm *tp, alarmcb_t callback);
#endif

/****************************************************************************
 * Name: lpc17_40_rtc_rdalarm
 *
 * Description:
 *   Query an alarm configured in hardware.
 *
 * Input Parameters:
 *  time - Current alarm setting.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
struct tm;
int lpc17_40_rtc_rdalarm(FAR struct tm *time);
#endif

/****************************************************************************
 * Name: lpc17_40_rtc_cancelalarm
 *
 * Description:
 *   Cancel a pending alarm
 *
 * Input Parameters:
 *   none
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
int lpc17_40_rtc_cancelalarm(void);
#endif

/****************************************************************************
 * Name: lpc17_40_rtc_lowerhalf
 *
 * Description:
 *   Instantiate the RTC lower half driver for the LPC54.  General usage:
 *
 *     #include <nuttx/timers/rtc.h>
 *     #include "lpc17_40_rtc.h"
 *
 *     struct rtc_lowerhalf_s *lower;
 *     lower = lpc17_40_rtc_lowerhalf();
 *     rtc_initialize(0, lower);
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, a non-NULL RTC lower interface is returned.  NULL is
 *   returned on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_DRIVER
struct rtc_lowerhalf_s;
FAR struct rtc_lowerhalf_s *lpc17_40_rtc_lowerhalf(void);
#endif

/************************************************************************************
 * Name: lpc17_40_rtc_setdatetime
 *
 * Description:
 *   Set the RTC to the provided time.
 *
 * Input Parameters:
 *   tp - the time to use
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/
struct tm;
int lpc17_40_rtc_setdatetime(FAR const struct tm *tp);

#endif /* __ARCH_ARM_SRC_LPC17XX_40XX_LPC17_40_RTC_H */
