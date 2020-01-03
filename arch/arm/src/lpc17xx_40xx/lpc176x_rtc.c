/************************************************************************************
 * arch/arm/src/lpc17xx_40xx/lpc176x_rtcc.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <time.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"

#include "chip.h"
#include "hardware/lpc17_40_syscon.h"

#include "lpc17_40_rtc.h"

#ifdef CONFIG_RTC

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/
/* This RTC implementation supports only date/time RTC hardware */

#ifndef CONFIG_RTC_DATETIME
#  error "CONFIG_RTC_DATETIME must be set to use this driver"
#endif

#ifdef CONFIG_RTC_HIRES
#  error "CONFIG_RTC_HIRES must NOT be set with this driver"
#endif

/************************************************************************************
 * Private Data
 ************************************************************************************/

/* Callback to use when the alarm expires */

#ifdef CONFIG_RTC_ALARM
static alarmcb_t g_alarmcb;
#endif

/************************************************************************************
 * Public Data
 ************************************************************************************/

/* g_rtc_enabled is set true after the RTC has successfully initialized */

volatile bool g_rtc_enabled = false;

/************************************************************************************
 * Private Functions
 ************************************************************************************/
/************************************************************************************
 * Name: rtc_dumpregs
 *
 * Description:
 *    Disable RTC write protection
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#ifdef CONFIG_DEBUG_RTC_INFO
static void rtc_dumpregs(FAR const char *msg)
{
  rtcinfo("%s:\n", msg);
  rtcinfo("  DOM : %08x\n", (getreg32(LPC17_40_RTC_DOM) & RTC_DOM_MASK));
  rtcinfo("  DOW : %08x\n", (getreg32(LPC17_40_RTC_DOW) & RTC_DOW_MASK));
}
#else
#  define rtc_dumpregs(msg)
#endif

/************************************************************************************
 * Name: rtc_dumptime
 *
 * Description:
 *    Disable RTC write protection
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#ifdef CONFIG_DEBUG_RTC_INFO
static void rtc_dumptime(FAR struct tm *tp, FAR const char *msg)
{
  rtcinfo("%s:\n", msg);
  rtcinfo("  tm_sec: %08x\n", tp->tm_sec);
  rtcinfo("  tm_min: %08x\n", tp->tm_min);
  rtcinfo(" tm_hour: %08x\n", tp->tm_hour);
  rtcinfo(" tm_mday: %08x\n", tp->tm_mday);
  rtcinfo("  tm_mon: %08x\n", tp->tm_mon);
  rtcinfo(" tm_year: %08x\n", tp->tm_year);
}
#else
#  define rtc_dumptime(tp, msg)
#endif

/************************************************************************************
 * Name: rtc_setup
 *
 * Description:
 *   Performs first time configuration of the RTC.  A special value written into
 *   back-up register 0 will prevent this function from being called on sub-sequent
 *   resets or power up.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

static int rtc_setup(void)
{
  /* Clear all registers to be default */

  putreg32((uint32_t)0x00, LPC17_40_RTC_ILR);
  putreg32((uint32_t)0x00, LPC17_40_RTC_CCR);
  putreg32((uint32_t)0x00, LPC17_40_RTC_CIIR);
  putreg32((uint32_t)0xff, LPC17_40_RTC_AMR);
  putreg32((uint32_t)0x00, LPC17_40_RTC_CALIB);

  /* Enable counters */
  putreg32(RTC_CCR_CLKEN, LPC17_40_RTC_CCR);
  return OK;
}

/************************************************************************************
 * Name: rtc_resume
 *
 * Description:
 *   Called when the RTC was already initialized on a previous power cycle.  This
 *   just brings the RTC back into full operation.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

static int rtc_resume(void)
{
  /* Clear the RTC alarm flags */

#ifdef CONFIG_RTC_ALARM
#endif
  return OK;
}

/************************************************************************************
 * Name: rtc_interrupt
 *
 * Description:
 *    RTC interrupt service routine
 *
 * Input Parameters:
 *   irq - The IRQ number that generated the interrupt
 *   context - Architecture specific register save information.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value on failure.
 *
 ************************************************************************************/

#ifdef CONFIG_RTC_ALARM
static int rtc_interrupt(int irq, void *context, FAR void *arg)
{
  uint32_t status = getreg32(LPC17_40_RTC_ILR);

  /* Clear pending status */

  putreg32(status | RTC_ILR_RTCALF | RTC_ILR_RTCCIF, LPC17_40_RTC_ILR);

  if ((status & RTC_ILR_RTCALF) != 0 && g_alarmcb != NULL)
    {
      /* Perform the alarm callback */

      g_alarmcb();
      g_alarmcb = NULL;
    }

  return OK;
}
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: up_rtc_initialize
 *
 * Description:
 *   Initialize the hardware RTC per the selected configuration.  This function is
 *   called once during the OS initialization sequence
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

int up_rtc_initialize(void)
{
  int ret;
  uint32_t regval;

  rtc_dumpregs("On reset");


  /* Enable power to the RTC module */

  regval  = getreg32(LPC17_40_SYSCON_PCONP);
  regval |= SYSCON_PCONP_PCRTC;
  putreg32(regval, LPC17_40_SYSCON_PCONP);


  /* Attach the RTC interrupt handler */

#ifdef CONFIG_RTC_ALARM
  ret = irq_attach(LPC17_40_IRQ_RTC, rtc_interrupt, NULL);
  if (ret == OK)
    {
      up_enable_irq(LPC17_40_IRQ_RTC);
    }
#endif /* CONFIG_RTC_ALARM */

  /* Perform the one-time setup of the RTC if we haven't already done so*/
  regval = getreg32(RTC_MAGIC_REG);
  if (regval != RTC_MAGIC && regval != RTC_MAGIC_TIME_SET)
    {
      rtcinfo("Initializing RTC\n");

      ret = rtc_setup();

      /* Remember that the RTC has been configured */

      putreg32(RTC_MAGIC, RTC_MAGIC_REG);
    }
  g_rtc_enabled = true;
  rtc_dumpregs("After Initialization");
  return OK;
}

/************************************************************************************
 * Name: up_rtc_getdatetime
 *
 * Description:
 *   Get the current date and time from the date/time RTC.  This interface
 *   is only supported by the date/time RTC hardware implementation.
 *   It is used to replace the system timer.  It is only used by the RTOS during
 *   initialization to set up the system time when CONFIG_RTC and CONFIG_RTC_DATETIME
 *   are selected (and CONFIG_RTC_HIRES is not).
 *
 *   NOTE: Some date/time RTC hardware is capable of sub-second accuracy.  That
 *   sub-second accuracy is lost in this interface.  However, since the system time
 *   is reinitialized on each power-up/reset, there will be no timing inaccuracy in
 *   the long run.
 *
 * Input Parameters:
 *   tp - The location to return the high resolution time value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

int up_rtc_getdatetime(FAR struct tm *tp)
{
  uint32_t seconds = 0xFF;

  /* Verify the seconds value didn't change during read.  If it did,
   * read the whole time again to ensure consistency.
   */
  while (seconds != (getreg32(LPC17_40_RTC_SEC) & RTC_SEC_MASK))
    {
      rtc_dumpregs("Reading Time");

      /* Convert the RTC time to fields in struct tm format.
       * All of the ranges of values correspond between struct tm and the time
       * register.
       */
      seconds = ((getreg32(LPC17_40_RTC_SEC) & RTC_SEC_MASK));
      tp->tm_sec  = seconds;
      tp->tm_min  = ((getreg32(LPC17_40_RTC_MIN) & RTC_MIN_MASK));
      tp->tm_hour = ((getreg32(LPC17_40_RTC_HOUR) & RTC_HOUR_MASK));

      /* Now convert the RTC date to fields in struct tm format:
       * Days: 1-31 match in both cases.
       * Month: RTC is 1-12, struct tm is 0-11.
       * Years: RTC is 0-4095, struct tm is years since 1900.
       */

      tp->tm_mday = ((getreg32(LPC17_40_RTC_DOM) & RTC_DOM_MASK));
      tp->tm_mon  = ((getreg32(LPC17_40_RTC_MONTH) & RTC_MONTH_MASK)) - 1;
      tp->tm_year = ((getreg32(LPC17_40_RTC_YEAR) & RTC_YEAR_MASK)-1900);
    }
  rtc_dumptime(tp, "Returning");
  return OK;
}

/************************************************************************************
 * Name: up_rtc_settime
 *
 * Description:
 *   Set the RTC to the provided time.  All RTC implementations must be able to
 *   set their time based on a standard timespec.
 *
 * Input Parameters:
 *   tp - the time to use
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

int up_rtc_settime(FAR const struct timespec *tp)
{
  FAR struct tm newtime;

  /* Break out the time values (note that the time is set only to units of seconds) */
  gmtime_r(&tp->tv_sec, &newtime);
  rtc_dumptime(&newtime, "Setting time");

  /* Then write the broken out values to the RTC */
  return lpc17_40_rtc_setdatetime(&newtime);
}


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

int lpc17_40_rtc_setdatetime(FAR const struct tm *tp)
{
  uint32_t regval;

  rtc_dumpregs("Setting Time");

  /* Disable the RTC counter while setting time */
  regval  = getreg32(LPC17_40_RTC_CCR);
  regval &= ~RTC_CCR_CLKEN & RTC_CCR_MASK;
  putreg32(regval, LPC17_40_RTC_CCR);

  /* Convert the time from struct tm format to RTC time registers
   * All of the ranges of values correspond between struct tm and the time
   * register.
   */

  putreg32(((tp->tm_sec) & RTC_SEC_MASK), LPC17_40_RTC_SEC);
  putreg32(((tp->tm_min) & RTC_MIN_MASK), LPC17_40_RTC_MIN);
  putreg32(((tp->tm_hour) & RTC_HOUR_MASK), LPC17_40_RTC_HOUR);

  /* Now convert the struct tm format fields to RTC date register format:
   * Days: 1-31 match in both cases.
   * Month: RTC is 1-12, struct tm is 0-11.
   * Years: RTC is 0-4095, struct tm is years since 1900.
   */
  putreg32(((tp->tm_mday) & RTC_DOM_MASK), LPC17_40_RTC_DOM);
  putreg32((((tp->tm_mon)+1) & RTC_MONTH_MASK), LPC17_40_RTC_MONTH);
  putreg32((((tp->tm_year)+1900) & RTC_YEAR_MASK), LPC17_40_RTC_YEAR);

  /* Resume counting after time has been set. */
  regval  = getreg32(LPC17_40_RTC_CCR);
  regval |= RTC_CCR_CLKEN;
  putreg32(regval, LPC17_40_RTC_CCR);

  /* Make sure that magic value is in the general purpose register */

  putreg32(RTC_MAGIC_TIME_SET, RTC_MAGIC_REG);

  rtc_dumptime(tp, "Returning");
  return OK;
}

/************************************************************************************
 * Name: lpc17_40_rtc_setalarm
 *
 * Description:
 *   Set up an alarm.  A single alarm is supported.
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
int lpc17_40_rtc_setalarm(FAR const struct tm *tp, alarmcb_t callback)
{
  int ret = -EBUSY;
  uint32_t regval;

  /* Is there already something waiting on the ALARM? */

  if (g_alarmcb == NULL)
    {
      /* No.. Save the callback function pointer */

      g_alarmcb = callback;

      rtc_dumptime(&tp, "Setting alarm");

      /* Disable the alarm while setting up */
      putreg32(RTC_AMR_MASK, LPC17_40_RTC_AMR);

      /* Then write the broken out values to the RTC alarm registers */

      putreg32(((tp->tm_sec) & RTC_SEC_MASK), LPC17_40_RTC_ALSEC);
      putreg32(((tp->tm_min) & RTC_MIN_MASK), LPC17_40_RTC_ALMIN);
      putreg32(((tp->tm_hour) & RTC_HOUR_MASK), LPC17_40_RTC_ALHOUR);
      putreg32(((tp->tm_mday) & RTC_DOM_MASK), LPC17_40_RTC_ALDOM);
      putreg32((((tp->tm_mon)+1) & RTC_MONTH_MASK), LPC17_40_RTC_ALMON);
      putreg32((((tp->tm_year)+1900) & RTC_YEAR_MASK), LPC17_40_RTC_ALYEAR);

      /* Enable alarm mask bits for the fields we set. */
      regval = ~(RTC_AMR_SEC | RTC_AMR_MIN | RTC_AMR_HOUR | RTC_AMR_DOM |
          RTC_AMR_MON | RTC_AMR_YEAR);
      putreg32(regval & RTC_AMR_MASK, LPC17_40_RTC_AMR);

      ret = OK;
    }
  return ret;
}
#endif

/************************************************************************************
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
 ************************************************************************************/

#ifdef CONFIG_RTC_ALARM
int lpc17_40_rtc_cancelalarm(void)
{
  irqstate_t flags;
  int ret = -ENODATA;

  flags = enter_critical_section();

  if (g_alarmcb != NULL)
    {
      /* Cancel the global callback function */

      g_alarmcb = NULL;

      /* Disable the alarm */

      putreg32(RTC_AMR_MASK, LPC17_40_RTC_AMR);

      ret = OK;
    }

  leave_critical_section(flags);

  return ret;
}
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
int lpc17_40_rtc_rdalarm(FAR struct tm *time)
{

  /* Read the alarm registers and place into the provided time value */
  time->tm_sec  = ((getreg32(LPC17_40_RTC_ALSEC) & RTC_SEC_MASK));
  time->tm_min  = ((getreg32(LPC17_40_RTC_ALMIN) & RTC_MIN_MASK));
  time->tm_hour = ((getreg32(LPC17_40_RTC_ALHOUR) & RTC_HOUR_MASK));
  time->tm_mday = ((getreg32(LPC17_40_RTC_ALDOM) & RTC_DOM_MASK));
  time->tm_mon  = ((getreg32(LPC17_40_RTC_ALMON) & RTC_MONTH_MASK)) - 1;
  time->tm_year = ((getreg32(LPC17_40_RTC_ALYEAR) & RTC_YEAR_MASK)-1900);

  return OK;
}
#endif


#endif /* CONFIG_RTC */
