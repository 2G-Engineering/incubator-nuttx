/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/lpc17_40_pmstandby.c
 *
 *   Copyright (C) 2012, 2020 Gregory Nutt. All rights reserved.
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

#include <stdbool.h>

#include "arm_internal.h"
#include "nvic.h"
#include "lpc17_40_pm.h"
#include "hardware/lpc17_40_syscon.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc17_40_pmdeepsleep
 *
 * Description:
 *   Enter deep sleep mode.  Note: PLL0 is is automatically disabled, but not
 *   disconnected from the CPU, when deep sleep mode is entered.  This will
 *   usually cause the CPU to lock up shortly after waking up.  The safest way
 *   to handle this is probably to disconnect PLL0 immediately before entering
 *   deep sleep mode.
 *
 * Input Parameters:
 *   sleeponexit - true:  SLEEPONEXIT bit is set when the WFI instruction is
 *                        executed, the MCU enters Sleep mode as soon as it
 *                        exits the lowest priority ISR.
 *               - false: SLEEPONEXIT bit is cleared, the MCU enters Sleep mode
 *                        as soon as WFI or WFE instruction is executed.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void lpc17_40_pmdeepsleep(bool sleeponexit)
{
  uint32_t regval;

  /* Configure PMU for sleep */

  regval = getreg32(LPC17_40_SYSCON_PCON);
  regval &= ~(SYSCON_PCON_PM0 | SYSCON_PCON_PM1);
  putreg32(regval, LPC17_40_SYSCON_PCON);

  /* Set SLEEPDEEP bit of Cortex System Control Register */

  regval  = getreg32(NVIC_SYSCON);
  regval |= NVIC_SYSCON_SLEEPDEEP;

  if (sleeponexit)
    {
      regval |= NVIC_SYSCON_SLEEPONEXIT;
    }
  else
    {
      regval &= ~NVIC_SYSCON_SLEEPONEXIT;
    }

  putreg32(regval, NVIC_SYSCON);

  /* Sleep until the wakeup interrupt or event occurs */

#ifdef CONFIG_PM_WFE
  /* Mode: SLEEP + Entry with WFE */

  asm("wfe");
#else
  /* Mode: SLEEP + Entry with WFI */

  asm("wfi");
#endif
}
