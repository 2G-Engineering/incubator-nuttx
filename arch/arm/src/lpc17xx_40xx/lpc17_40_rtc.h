/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/lpc17_40_rtc.h
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

#ifndef __ARCH_ARM_SRC_LPC17XX_40XX_LPC17_40_RTC_H
#define __ARCH_ARM_SRC_LPC17XX_40XX_LPC17_40_RTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/lpc17_40_rtc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RTC_MAGIC               (0xfacefeed)
#define RTC_MAGIC_TIME_SET      (0xf00dface)
#define RTC_MAGIC_REG            LPC17_40_RTC_GPREG0

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The form of an alarm callback */

typedef void (*alarmcb_t)(void);

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
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
 ****************************************************************************/

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
