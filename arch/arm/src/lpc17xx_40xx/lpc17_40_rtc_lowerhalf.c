/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/lpc17_40_rtc_lowerhalf.c
 *
 *   Copyright (C) 2017-2019 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/timers/rtc.h>

#include "up_arch.h"

#include "hardware/lpc17_40_rtc.h"
#include "lpc17_40_rtc.h"

#ifdef CONFIG_RTC_DRIVER

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
struct lpc17_40_cbinfo_s
{
  volatile rtc_alarm_callback_t cb;  /* Callback when the alarm expires */
  volatile FAR void *priv;           /* Private argument to accompany callback */
};
#endif

/* This is the private type for the RTC state.  It must be cast compatible
 * with struct rtc_lowerhalf_s.
 */

struct lpc17_40_lowerhalf_s
{
  /* This is the contained reference to the read-only, lower-half
   * operations vtable (which may lie in FLASH or ROM)
   */

  FAR const struct rtc_ops_s *ops;

  /* Data following is private to this driver and not visible outside of
   * this file.
   */

  sem_t devsem;         /* Threads can only exclusively access the RTC */

#ifdef CONFIG_RTC_ALARM
  /* Alarm callback information */

  struct lpc17_40_cbinfo_s cbinfo;
#endif

#ifdef CONFIG_RTC_PERIODIC
  /* Periodic wakeup information */

  struct lower_setperiodic_s periodic;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Prototypes for static methods in struct rtc_ops_s */

static int lpc17_40_rdtime(FAR struct rtc_lowerhalf_s *lower,
             FAR struct rtc_time *rtctime);
static int lpc17_40_settime(FAR struct rtc_lowerhalf_s *lower,
             FAR const struct rtc_time *rtctime);
static bool lpc17_40_havesettime(FAR struct rtc_lowerhalf_s *lower);

#ifdef CONFIG_RTC_ALARM
static int lpc17_40_setalarm(FAR struct rtc_lowerhalf_s *lower,
             FAR const struct lower_setalarm_s *alarminfo);
static int lpc17_40_setrelative(FAR struct rtc_lowerhalf_s *lower,
             FAR const struct lower_setrelative_s *alarminfo);
static int lpc17_40_cancelalarm(FAR struct rtc_lowerhalf_s *lower,
             int alarmid);
static int lpc17_40_rdalarm(FAR struct rtc_lowerhalf_s *lower,
             FAR struct lower_rdalarm_s *alarminfo);
#endif

#ifdef CONFIG_RTC_PERIODIC
static int lpc17_40_setperiodic(FAR struct rtc_lowerhalf_s *lower,
             FAR const struct lower_setperiodic_s *alarminfo);
static int lpc17_40_cancelperiodic(FAR struct rtc_lowerhalf_s *lower, int id);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* LPC17xx/40xx RTC driver operations */

static const struct rtc_ops_s g_rtc_ops =
{
  .rdtime         = lpc17_40_rdtime,
  .settime        = lpc17_40_settime,
  .havesettime    = lpc17_40_havesettime,
#ifdef CONFIG_RTC_ALARM
  .setalarm       = lpc17_40_setalarm,
  .setrelative    = lpc17_40_setrelative,
  .cancelalarm    = lpc17_40_cancelalarm,
  .rdalarm        = lpc17_40_rdalarm,
#endif
#ifdef CONFIG_RTC_PERIODIC
  .setperiodic    = lpc17_40_setperiodic,
  .cancelperiodic = lpc17_40_cancelperiodic,
#endif
#ifdef CONFIG_RTC_IOCTL
  .ioctl          = NULL,
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  .destroy        = NULL,
#endif
};

/* LPC17xx/40xx RTC device state */

static struct lpc17_40_lowerhalf_s g_rtc_lowerhalf =
{
  .ops           = &g_rtc_ops,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc17_40_alarm_callback
 *
 * Description:
 *   This is the function that is called from the RTC driver when the alarm
 *   goes off.  It just invokes the upper half drivers callback.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static void lpc17_40_alarm_callback(void)
{
  FAR struct lpc17_40_cbinfo_s *cbinfo = &g_rtc_lowerhalf.cbinfo;

  /* Sample and clear the callback information to minimize the window in
   * time in which race conditions can occur.
   */

  rtc_alarm_callback_t cb = (rtc_alarm_callback_t)cbinfo->cb;
  FAR void *arg           = (FAR void *)cbinfo->priv;

  cbinfo->cb              = NULL;
  cbinfo->priv            = NULL;

  /* Perform the callback */

  if (cb != NULL)
    {
      cb(arg, 0);
    }
}
#endif /* CONFIG_RTC_ALARM */

/****************************************************************************
 * Name: lpc17_40_rdtime
 *
 * Description:
 *   Implements the rdtime() method of the RTC driver interface
 *
 * Input Parameters:
 *   lower   - A reference to RTC lower half driver state structure
 *   rcttime - The location in which to return the current RTC time.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

static int lpc17_40_rdtime(FAR struct rtc_lowerhalf_s *lower,
                        FAR struct rtc_time *rtctime)
{
  /* This operation depends on the fact that struct rtc_time is cast
   * compatible with struct tm.
   */

  return up_rtc_getdatetime((FAR struct tm *)rtctime);
}

/****************************************************************************
 * Name: lpc17_40_settime
 *
 * Description:
 *   Implements the settime() method of the RTC driver interface
 *
 * Input Parameters:
 *   lower   - A reference to RTC lower half driver state structure
 *   rcttime - The new time to set
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

static int lpc17_40_settime(FAR struct rtc_lowerhalf_s *lower,
                         FAR const struct rtc_time *rtctime)
{
    /* This operation depends on the fact that struct rtc_time is cast
     * compatible with struct tm.
     */

    return lpc17_40_rtc_setdatetime((FAR const struct tm *)rtctime);
}

/****************************************************************************
 * Name: lpc17_40_havesettime
 *
 * Description:
 *   Implements the havesettime() method of the RTC driver interface
 *
 * Input Parameters:
 *   lower   - A reference to RTC lower half driver state structure
 *
 * Returned Value:
 *   Returns true if RTC date-time have been previously set.
 *
 ****************************************************************************/

static bool lpc17_40_havesettime(FAR struct rtc_lowerhalf_s *lower)
{
  return getreg32(RTC_MAGIC_REG) == RTC_MAGIC_TIME_SET;
}

/****************************************************************************
 * Name: lpc17_40_setalarm
 *
 * Description:
 *   Set a new alarm.  This function implements the setalarm() method of the
 *   RTC driver interface
 *
 * Input Parameters:
 *   lower - A reference to RTC lower half driver state structure
 *   alarminfo - Provided information needed to set the alarm
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static int lpc17_40_setalarm(FAR struct rtc_lowerhalf_s *lower,
                          FAR const struct lower_setalarm_s *alarminfo)
{
  FAR struct lpc17_40_lowerhalf_s *priv;
  FAR struct lpc17_40_cbinfo_s *cbinfo;
  int ret;

  DEBUGASSERT(lower != NULL && alarminfo != NULL && alarminfo->id == 0);
  priv = (FAR struct lpc17_40_lowerhalf_s *)lower;

  ret = nxsem_wait(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  ret = -EINVAL;
  if (alarminfo->id == 0)
    {
      /* Remember the callback information */

      cbinfo           = &priv->cbinfo;
      cbinfo->cb       = alarminfo->cb;
      cbinfo->priv     = alarminfo->priv;

      /* And set the alarm */

      ret = lpc17_40_rtc_setalarm((FAR struct tm *)&alarminfo->time, lpc17_40_alarm_callback);
      if (ret < 0)
        {
          cbinfo->cb   = NULL;
          cbinfo->priv = NULL;
        }
    }

  nxsem_post(&priv->devsem);
  return ret;
}
#endif

/****************************************************************************
 * Name: lpc17_40_setrelative
 *
 * Description:
 *   Set a new alarm relative to the current time.  This function implements
 *   the setrelative() method of the RTC driver interface
 *
 * Input Parameters:
 *   lower - A reference to RTC lower half driver state structure
 *   alarminfo - Provided information needed to set the alarm
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static int lpc17_40_setrelative(FAR struct rtc_lowerhalf_s *lower,
                             FAR const struct lower_setrelative_s *alarminfo)
{
  FAR struct lpc17_40_lowerhalf_s *priv;
  FAR struct lpc17_40_cbinfo_s *cbinfo;
  struct lower_setalarm_s setalarm;
  struct tm time;
  time_t seconds;
  int ret = -EINVAL;

  DEBUGASSERT(lower != NULL && alarminfo != NULL && alarminfo->id == 0);
  priv = (FAR struct lpc17_40_lowerhalf_s *)lower;

  if (alarminfo->id == 0 && alarminfo->reltime > 0)
    {
      /* Disable pre-emption while we do this so that we don't have to worry
       * about being suspended and working on an old time.
       */

      sched_lock();

      /* Get the current time in broken out format */

      ret = up_rtc_getdatetime(&time);
      if (ret >= 0)
        {
          /* Convert to seconds since the epoch */

          seconds = mktime(&time);

          /* Add the seconds offset.  Add one to the number of seconds
           * because we are unsure of the phase of the timer.
           */

          seconds += (alarminfo->reltime + 1);

          /* And convert the time back to broken out format */

          gmtime_r(&seconds, (FAR struct tm *)&setalarm.time);

          /* The set the alarm using this absolute time */

          setalarm.id   = alarminfo->id;
          setalarm.cb   = alarminfo->cb;
          setalarm.priv = alarminfo->priv;

          ret = lpc17_40_setalarm(lower, &setalarm);
        }

      sched_unlock();
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: lpc17_40_cancelalarm
 *
 * Description:
 *   Cancel the current alarm.  This function implements the cancelalarm()
 *   method of the RTC driver interface
 *
 * Input Parameters:
 *   lower - A reference to RTC lower half driver state structure
 *   alarminfo - Provided information needed to set the alarm
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static int lpc17_40_cancelalarm(FAR struct rtc_lowerhalf_s *lower, int alarmid)
{
  FAR struct lpc17_40_lowerhalf_s *priv;
  FAR struct lpc17_40_cbinfo_s *cbinfo;

  DEBUGASSERT(lower != NULL);
  DEBUGASSERT(alarmid == 0);
  priv = (FAR struct lpc17_40_lowerhalf_s *)lower;

  /* Nullify callback information to reduce window for race conditions */

  cbinfo       = &priv->cbinfo;
  cbinfo->cb   = NULL;
  cbinfo->priv = NULL;

  /* Then cancel the alarm */

  return lpc17_40_rtc_cancelalarm();
}
#endif

/****************************************************************************
 * Name: lpc17_40_rdalarm
 *
 * Description:
 *   Query the RTC alarm.
 *
 * Input Parameters:
 *   lower - A reference to RTC lower half driver state structure
 *   alarminfo - Provided information needed to query the alarm
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static int lpc17_40_rdalarm(FAR struct rtc_lowerhalf_s *lower,
                         FAR struct lower_rdalarm_s *alarminfo)
{
  int ret = -EINVAL;

  DEBUGASSERT(lower != NULL && alarminfo != NULL && alarminfo->id == 0 &&
              alarminfo->time != NULL);

  if (alarminfo->id == 0)
    {
      ret = lpc17_40_rtc_rdalarm((FAR struct tm *)alarminfo->time);
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: lpc17_40_periodic_callback
 *
 * Description:
 *   This is the function that is called from the RTC driver when the periodic
 *   wakeup goes off.  It just invokes the upper half drivers callback.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_PERIODIC
static int lpc17_40_periodic_callback(void)
{
  FAR struct lpc17_40_lowerhalf_s *lower;
  struct lower_setperiodic_s *cbinfo;
  rtc_wakeup_callback_t cb;
  FAR void *priv;

  lower = (FAR struct lpc17_40_lowerhalf_s *)&g_rtc_lowerhalf;

  cbinfo = &lower->periodic;
  cb     = (rtc_wakeup_callback_t)cbinfo->cb;
  priv   = (FAR void *)cbinfo->priv;

  /* Perform the callback */

  if (cb != NULL)
    {
      cb(priv, 0);
    }

  return OK;
}
#endif /* CONFIG_RTC_PERIODIC */

/****************************************************************************
 * Name: lpc17_40_setperiodic
 *
 * Description:
 *   Set a new periodic wakeup relative to the current time, with a given
 *   period. This function implements the setperiodic() method of the RTC
 *   driver interface
 *
 * Input Parameters:
 *   lower - A reference to RTC lower half driver state structure
 *   alarminfo - Provided information needed to set the wakeup activity
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_PERIODIC
static int lpc17_40_setperiodic(FAR struct rtc_lowerhalf_s *lower,
                             FAR const struct lower_setperiodic_s *alarminfo)
{
  FAR struct lpc17_40_lowerhalf_s *priv;
  int ret;

  DEBUGASSERT(lower != NULL && alarminfo != NULL);
  priv = (FAR struct lpc17_40_lowerhalf_s *)lower;

  ret = nxsem_wait(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  memcpy(&priv->periodic, alarminfo, sizeof(struct lower_setperiodic_s));

  ret = lpc17_40_rtc_setperiodic(&alarminfo->period, lpc17_40_periodic_callback);

  nxsem_post(&priv->devsem);
  return ret;
}
#endif

/****************************************************************************
 * Name: lpc17_40_cancelperiodic
 *
 * Description:
 *   Cancel the current periodic wakeup activity.  This function implements
 *   the cancelperiodic() method of the RTC driver interface
 *
 * Input Parameters:
 *   lower - A reference to RTC lower half driver state structure
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_PERIODIC
static int lpc17_40_cancelperiodic(FAR struct rtc_lowerhalf_s *lower, int id)
{
  FAR struct lpc17_40_lowerhalf_s *priv;
  int ret;

  DEBUGASSERT(lower != NULL);
  priv = (FAR struct lpc17_40_lowerhalf_s *)lower;

  DEBUGASSERT(id == 0);

  ret = nxsem_wait(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  ret = lpc17_40_rtc_cancelperiodic();

  nxsem_post(&priv->devsem);
  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc17_40_rtc_lowerhalf
 *
 * Description:
 *   Instantiate the RTC lower half driver for the LPC17xx/40xx.  General usage:
 *
 *     #include <nuttx/timers/rtc.h>
 *     #include "lpc17_40_rtc.h>
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

FAR struct rtc_lowerhalf_s *lpc17_40_rtc_lowerhalf(void)
{
  nxsem_init(&g_rtc_lowerhalf.devsem, 0, 1);

  return (FAR struct rtc_lowerhalf_s *)&g_rtc_lowerhalf;
}

#endif /* CONFIG_RTC_DRIVER */
