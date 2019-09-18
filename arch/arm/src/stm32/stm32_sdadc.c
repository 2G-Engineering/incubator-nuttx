/****************************************************************************
 * arch/arm/src/stm32/stm32_sdadc.c
 *
 *   Copyright (C) 2011, 2013, 2015-2017 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2016 Studelec. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Marc Rechté <mrechte@studelec-sa.com>
 *
 * derived from arch/arm/src/stm32/stm32_adc.c
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

#include <stdio.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <semaphore.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>
#include <unistd.h>

#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "stm32.h"
#include "stm32_dma.h"
#include "stm32_pwr.h"
#include "stm32_sdadc.h"

#ifdef CONFIG_STM32_SDADC

/* Some SDADC peripheral must be enabled */

#if defined(CONFIG_STM32_SDADC1) || defined(CONFIG_STM32_SDADC2) || \
    defined(CONFIG_STM32_SDADC3)

/* This implementation is for the STM32F37XX only */

#ifndef CONFIG_STM32_STM32F37XX
#  error "This chip is not yet supported"
#endif

/* TODO: At the moment there is no implementation
   for timer and external triggers */

#if defined(SDADC_HAVE_TIMER)
#  error "There is no proper implementation for TIMER TRIGGERS at the moment"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RCC reset ****************************************************************/

#define STM32_RCC_RSTR   STM32_RCC_APB2RSTR
#define RCC_RSTR_SDADC1RST RCC_APB2RSTR_SDADC1RST
#define RCC_RSTR_SDADC2RST RCC_APB2RSTR_SDADC2RST
#define RCC_RSTR_SDADC3RST RCC_APB2RSTR_SDADC3RST

/* SDADC interrupts *********************************************************/

#define SDADC_ISR_ALLINTS (SDADC_ISR_JEOCF | SDADC_ISR_JOVRF)

/* SDADC Channels/DMA *******************************************************/


#define SDADC_DMA_CONTROL_WORD (DMA_CCR_MSIZE_16BITS | \
                                DMA_CCR_PSIZE_16BITS | \
                                DMA_CCR_MINC | \
                                DMA_CCR_CIRC)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the state of one SDADC block */

struct stm32_dev_s
{
  FAR const struct adc_callback_s *cb;
  uint8_t irq;          /* Interrupt generated by this SDADC block */
  uint8_t nchannels;    /* Number of channels */
  uint8_t cchannels;    /* Number of configured channels */
  uint8_t intf;         /* SDADC interface number */
  uint8_t current;      /* Current SDADC channel being converted */
  uint8_t refv;         /* Reference voltage selection */
#ifdef SDADC_HAVE_DMA
  uint8_t dmachan;      /* DMA channel needed by this SDADC */
  bool    hasdma;       /* True: This channel supports DMA */
#endif
#ifdef SDADC_HAVE_TIMER
  uint8_t trigger;      /* Timer trigger selection: see SDADCx_JEXTSEL_TIMxx */
#endif
  uint32_t base;        /* Base address of registers unique to this SDADC
                         * block */
#ifdef SDADC_HAVE_TIMER
  uint32_t tbase;       /* Base address of timer used by this SDADC block */
  uint32_t jextsel      /* JEXTSEL value used by this SDADC block */
  uint32_t pclck;       /* The PCLK frequency that drives this timer */
  uint32_t freq;        /* The desired frequency of conversions */
#endif
#ifdef SDADC_HAVE_DMA
  DMA_HANDLE dma;       /* Allocated DMA channel */

  /* DMA transfer buffer */

  int16_t dmabuffer[SDADC_MAX_SAMPLES];
#endif

  /* List of selected SDADC injected channels to sample */

  uint8_t  chanlist[SDADC_MAX_SAMPLES];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* ADC Register access */

static uint32_t sdadc_getreg(FAR struct stm32_dev_s *priv, int offset);
static void     sdadc_putreg(FAR struct stm32_dev_s *priv, int offset,
                             uint32_t value);
static void     sdadc_modifyreg(FAR struct stm32_dev_s *priv, int offset,
                                uint32_t clrbits, uint32_t setbits);
#ifdef ADC_HAVE_TIMER
static uint16_t tim_getreg(FAR struct stm32_dev_s *priv, int offset);
static void     tim_putreg(FAR struct stm32_dev_s *priv, int offset,
                           uint16_t value);
static void     tim_modifyreg(FAR struct stm32_dev_s *priv, int offset,
                              uint16_t clrbits, uint16_t setbits);
static void     tim_dumpregs(FAR struct stm32_dev_s *priv,
                             FAR const char *msg);
#endif

static void sdadc_rccreset(FAR struct stm32_dev_s *priv, bool reset);

/* ADC Interrupt Handler */

static int  sdadc_interrupt(int irq, FAR void *context, FAR void *arg);

/* ADC Driver Methods */

static int  sdadc_bind(FAR struct adc_dev_s *dev,
                       FAR const struct adc_callback_s *callback);
static void sdadc_reset(FAR struct adc_dev_s *dev);
static int  sdadc_setup(FAR struct adc_dev_s *dev);
static void sdadc_shutdown(FAR struct adc_dev_s *dev);
static void sdadc_rxint(FAR struct adc_dev_s *dev, bool enable);
static int  sdadc_ioctl(FAR struct adc_dev_s *dev, int cmd,
                        unsigned long arg);
static void sdadc_enable(FAR struct stm32_dev_s *priv, bool enable);

static int  sdadc_set_ch(FAR struct adc_dev_s *dev, uint8_t ch);

#ifdef ADC_HAVE_TIMER
static void sdadc_timstart(FAR struct stm32_dev_s *priv, bool enable);
static int  sdadc_timinit(FAR struct stm32_dev_s *priv);
#endif

#ifdef ADC_HAVE_DMA
static void sdadc_dmaconvcallback(DMA_HANDLE handle, uint8_t isr,
                                  FAR void *arg);
#endif

static void sdadc_startconv(FAR struct stm32_dev_s *priv, bool enable);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* SDADC interface operations */

static const struct adc_ops_s g_sdadcops =
{
  .ao_bind        = sdadc_bind,
  .ao_reset       = sdadc_reset,
  .ao_setup       = sdadc_setup,
  .ao_shutdown    = sdadc_shutdown,
  .ao_rxint       = sdadc_rxint,
  .ao_ioctl       = sdadc_ioctl,
};

/* SDADC1 state */

#ifdef CONFIG_STM32_SDADC1
static struct stm32_dev_s g_sdadcpriv1 =
{
  .irq         = STM32_IRQ_SDADC1,
  .intf        = 1,
  .base        = STM32_SDADC1_BASE,
  .refv        = SDADC1_REFV,
#ifdef SDADC1_HAVE_TIMER
  .trigger     = CONFIG_STM32_SDADC1_TIMTRIG,
  .tbase       = SDADC1_TIMER_BASE,
  .extsel      = SDADC1_EXTSEL_VALUE,
  .pclck       = SDADC1_TIMER_PCLK_FREQUENCY,
  .freq        = CONFIG_STM32_SDADC1_SAMPLE_FREQUENCY,
#endif
#ifdef SDADC1_HAVE_DMA
  .dmachan     = DMACHAN_SDADC1,
  .hasdma      = true,
#endif
};

static struct adc_dev_s g_sdadcdev1 =
{
  .ad_ops      = &g_sdadcops,
  .ad_priv     = &g_sdadcpriv1,
};
#endif

/* SDADC2 state */

#ifdef CONFIG_STM32_SDADC2
static struct stm32_dev_s g_sdadcpriv2 =
{
  .irq         = STM32_IRQ_SDADC2,
  .base        = STM32_SDADC2_BASE,
  .refv        = SDADC2_REFV,
#ifdef SDADC2_HAVE_TIMER
  .trigger     = CONFIG_STM32_SDADC2_TIMTRIG,
  .tbase       = SDADC2_TIMER_BASE,
  .extsel      = SDADC2_EXTSEL_VALUE,
  .pclck       = SDADC2_TIMER_PCLK_FREQUENCY,
  .freq        = CONFIG_STM32_SDADC2_SAMPLE_FREQUENCY,
#endif
#ifdef SDADC2_HAVE_DMA
  .dmachan     = DMACHAN_SDADC2,
  .hasdma      = true,
#endif
};

static struct adc_dev_s g_sdadcdev2 =
{
  .ad_ops      = &g_sdadcops,
  .ad_priv     = &g_sdadcpriv2,
};
#endif

/* SDADC3 state */

#ifdef CONFIG_STM32_SDADC3
static struct stm32_dev_s g_sdadcpriv3 =
{
  .irq         = STM32_IRQ_SDADC3,
  .base        = STM32_SDADC3_BASE,
  .refv        = SDADC3_REFV,
#ifdef SDADC3_HAVE_TIMER
  .trigger     = CONFIG_STM32_SDADC3_TIMTRIG,
  .tbase       = SDADC3_TIMER_BASE,
  .extsel      = SDADC3_EXTSEL_VALUE,
  .pclck       = SDADC3_TIMER_PCLK_FREQUENCY,
  .freq        = CONFIG_STM32_SDADC3_SAMPLE_FREQUENCY,
#endif
#ifdef SDADC3_HAVE_DMA
  .dmachan     = DMACHAN_SDADC3,
  .hasdma      = true,
#endif
};

static struct adc_dev_s g_sdadcdev3 =
{
  .ad_ops      = &g_sdadcops,
  .ad_priv     = &g_sdadcpriv3,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sdadc_getreg
 *
 * Description:
 *   Read the value of an SDADC register.
 *
 * Input Parameters:
 *   priv   - A reference to the SDADC block state
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   The current contents of the specified register
 *
 ****************************************************************************/

static uint32_t sdadc_getreg(FAR struct stm32_dev_s *priv, int offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: sdadc_putreg
 *
 * Description:
 *   Write a value to an SDADC register.
 *
 * Input Parameters:
 *   priv   - A reference to the SDADC block state
 *   offset - The offset to the register to write to
 *   value  - The value to write to the register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sdadc_putreg(FAR struct stm32_dev_s *priv, int offset,
                       uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: sdadc_modifyreg
 *
 * Description:
 *   Modify the value of an SDADC register (not atomic).
 *
 * Input Parameters:
 *   priv   - A reference to the SDADC block state
 *   offset  - The offset to the register to modify
 *   clrbits - The bits to clear
 *   setbits - The bits to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sdadc_modifyreg(FAR struct stm32_dev_s *priv, int offset,
                          uint32_t clrbits, uint32_t setbits)
{
  sdadc_putreg(priv, offset, (sdadc_getreg(priv, offset) & ~clrbits) | setbits);
}

/****************************************************************************
 * Name: tim_getreg
 *
 * Description:
 *   Read the value of an SDADC timer register.
 *
 * Input Parameters:
 *   priv   - A reference to the SDADC block state
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   The current contents of the specified register
 *
 ****************************************************************************/

#ifdef SDADC_HAVE_TIMER
static uint16_t tim_getreg(FAR struct stm32_dev_s *priv, int offset)
{
  return getreg16(priv->tbase + offset);
}
#endif

/****************************************************************************
 * Name: tim_putreg
 *
 * Description:
 *   Write a value to an SDADC timer register.
 *
 * Input Parameters:
 *   priv   - A reference to the SDADC block state
 *   offset - The offset to the register to write to
 *   value  - The value to write to the register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef SDADC_HAVE_TIMER
static void tim_putreg(FAR struct stm32_dev_s *priv, int offset,
                       uint16_t value)
{
  putreg16(value, priv->tbase + offset);
}
#endif

/****************************************************************************
 * Name: tim_modifyreg
 *
 * Description:
 *   Modify the value of an SDADC timer register (not atomic).
 *
 * Input Parameters:
 *   priv   - A reference to the SDADC block state
 *   offset  - The offset to the register to modify
 *   clrbits - The bits to clear
 *   setbits - The bits to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef SDADC_HAVE_TIMER
static void tim_modifyreg(FAR struct stm32_dev_s *priv, int offset,
                          uint16_t clrbits, uint16_t setbits)
{
  tim_putreg(priv, offset, (tim_getreg(priv, offset) & ~clrbits) | setbits);
}
#endif

/****************************************************************************
 * Name: tim_dumpregs
 *
 * Description:
 *   Dump all timer registers.
 *
 * Input Parameters:
 *   priv   - A reference to the SDADC block state
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef SDADC_HAVE_TIMER
static void tim_dumpregs(FAR struct stm32_dev_s *priv, FAR const char *msg)
{
  ainfo("%s:\n", msg);

  /* TODO */
}
#endif

/****************************************************************************
 * Name: sdadc_timstart
 *
 * Description:
 *   Start (or stop) the timer counter
 *
 * Input Parameters:
 *   priv   - A reference to the SDADC block state
 *   enable - True: Start conversion
 *
 * Returned Value:
 *
 ****************************************************************************/

#ifdef SDADC_HAVE_TIMER
static void sdadc_timstart(FAR struct stm32_dev_s *priv, bool enable)
{
  ainfo("enable: %d\n", enable ? 1 : 0);

  if (enable)
    {
      /* Start the counter */

      tim_modifyreg(priv, STM32_GTIM_CR1_OFFSET, 0, GTIM_CR1_CEN);
    }
  else
    {
      /* Disable the counter */

      tim_modifyreg(priv, STM32_GTIM_CR1_OFFSET, GTIM_CR1_CEN, 0);
    }
}
#endif

/****************************************************************************
 * Name: sdadc_timinit
 *
 * Description:
 *   Initialize the timer that drivers the SDADC sampling for this channel
 *   using the pre-calculated timer divider definitions.
 *
 * Input Parameters:
 *   priv   - A reference to the SDADC block state
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef SDADC_HAVE_TIMER
static int sdadc_timinit(FAR struct stm32_dev_s *priv)
{
  /* TODO */

  aerr("ERROR: not implemented");
  return ERROR;
}
#endif

/****************************************************************************
 * Name: sdadc_startconv
 *
 * Description:
 *   Start (or stop) the SDADC conversion process
 *
 * Input Parameters:
 *   priv   - A reference to the SDADC block state
 *   enable - True: Start conversion
 *
 * Returned Value:
 *
 ****************************************************************************/

static void sdadc_startconv(FAR struct stm32_dev_s *priv, bool enable)
{
  ainfo("enable: %d\n", enable ? 1 : 0);

  if (enable)
    {
      /* Start the conversion of injected channels */

      sdadc_modifyreg(priv, STM32_SDADC_CR2_OFFSET, 0, SDADC_CR2_JSWSTART);
    }
  else
    {
      /* Wait for a possible conversion to stop */

      while ((sdadc_getreg(priv, STM32_SDADC_ISR_OFFSET) & SDADC_ISR_JCIP) != 0);
    }
}

/****************************************************************************
 * Name: sdadc_rccreset
 *
 * Description:
 *   (De)Initializes the SDADC block registers to their default
 *   reset values.
 *
 * Input Parameters:
 *   priv   - A reference to the SDADC block state
 *   reset  - true: to put in reset state, false: to revert to normal state
 *
 * Returned Value:
 *
 ****************************************************************************/

static void sdadc_rccreset(FAR struct stm32_dev_s *priv, bool reset)
{
  uint32_t adcbit;

  /* Pick the appropriate bit in the APB2 reset register.
   */

  switch (priv->intf)
    {
#ifdef CONFIG_STM32_SDADC1
      case 1:
        adcbit = RCC_RSTR_SDADC1RST;
        break;
#endif
#ifdef CONFIG_STM32_SDADC2
      case 2:
        adcbit = RCC_RSTR_SDADC2RST;
        break;
#endif
#ifdef CONFIG_STM32_SDADC3
      case 3:
        adcbit = RCC_RSTR_SDADC3RST;
        break;
#endif
      default:
        return;
    }

  /* Set or clear the selected bit in the APB2 reset register.
   * modifyreg32() disables interrupts.  Disabling interrupts is necessary
   * because the APB2RSTR register is used by several different drivers.
   */

  if (reset)
    {
      /* Enable SDADC reset state */

      modifyreg32(STM32_RCC_RSTR, 0, adcbit);
    }
  else
    {
      /* Release SDADC from reset state */

      modifyreg32(STM32_RCC_RSTR, adcbit, 0);
    }
}

/****************************************************************************
 * Name: sdadc_power_down_idle
 *
 * Description:
 *   Enables or disables power down during the idle phase.
 *
 * Input Parameters:
 *   priv     - A reference to the SDADC block state
 *   pdi_high - true:  The SDADC is powered down when waiting for a start event
 *              false: The SDADC is powered up when waiting for a start event
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#if 0
static void sdadc_power_down_idle(FAR struct stm32_dev_s *priv, bool pdi_high)
{
  uint32_t regval;

  ainfo("PDI: %d\n", pdi_high ? 1 : 0);

  regval = sdadc_getreg(priv, STM32_SDADC_CR2_OFFSET);

  if ((regval & SDADC_CR2_ADON) == 0)
    {
      regval = sdadc_getreg(priv, STM32_SDADC_CR1_OFFSET);
      if (pdi_high)
        {
          regval |= SDADC_CR1_PDI;
        }
      else
        {
          regval &= ~SDADC_CR1_PDI;
        }

      sdadc_putreg(priv, STM32_SDADC_CR1_OFFSET, regval);
    }
}
#endif

/****************************************************************************
 * Name: sdadc_enable
 *
 * Description:
 *   Enables or disables the specified SDADC peripheral.
 *                  Does not start conversion unless the SDADC is
 *                  triggered by timer
 *
 * Input Parameters:
 *   priv   - A reference to the SDADC block state
 *   enable - true:  enable SDADC conversion
 *            false: disable SDADC conversion
 *
 * Returned Value:
 *
 ****************************************************************************/

static void sdadc_enable(FAR struct stm32_dev_s *priv, bool enable)
{
  uint32_t regval;

  ainfo("enable: %d\n", enable ? 1 : 0);

  regval = sdadc_getreg(priv, STM32_SDADC_CR2_OFFSET);

  if (enable)
    {
      /* Enable the SDADC */

      sdadc_putreg(priv, STM32_SDADC_CR2_OFFSET, regval | SDADC_CR2_ADON);

      /* Wait for the SDADC to be stabilized */

      while (sdadc_getreg(priv, STM32_SDADC_ISR_OFFSET) & SDADC_ISR_STABIP);
    }
  else if ((regval & SDADC_CR2_ADON) != 0)
    {
      /* Ongoing conversions will be stopped implicitly */

      /* Disable the SDADC */

      sdadc_putreg(priv, STM32_SDADC_CR2_OFFSET, regval & ~SDADC_CR2_ADON);

    }
}

/****************************************************************************
 * Name: sdadc_dmaconvcallback
 *
 * Description:
 *   Callback for DMA.  Called from the DMA transfer complete interrupt after
 *   all channels have been converted and transferred with DMA.
 *
 * Input Parameters:
 *   handle - handle to DMA
 *   isr -
 *   arg - SDADC device
 *
 * Returned Value:
 *
 ****************************************************************************/

#ifdef SDADC_HAVE_DMA
static void sdadc_dmaconvcallback(DMA_HANDLE handle, uint8_t isr, FAR void *arg)
{
  FAR struct adc_dev_s   *dev  = (FAR struct adc_dev_s *)arg;
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;
  int i;

  /* Verify that the upper-half driver has bound its callback functions */

  if (priv->cb != NULL)
    {
      DEBUGASSERT(priv->cb->au_receive != NULL);

      for (i = 0; i < priv->nchannels; i++)
        {
          priv->cb->au_receive(dev, priv->chanlist[priv->current],
                               priv->dmabuffer[priv->current]);
          priv->current++;
          if (priv->current >= priv->nchannels)
            {
              /* Restart the conversion sequence from the beginning */

              priv->current = 0;
            }
        }
    }
}
#endif

/****************************************************************************
 * Name: sdadc_bind
 *
 * Description:
 *   Bind the upper-half driver callbacks to the lower-half implementation.
 *   This must be called early in order to receive SDADC event notifications.
 *
 ****************************************************************************/

static int sdadc_bind(FAR struct adc_dev_s *dev,
                    FAR const struct adc_callback_s *callback)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;

  DEBUGASSERT(priv != NULL);
  priv->cb = callback;
  return OK;
}

/****************************************************************************
 * Name: sdadc_reset
 *
 * Description:
 *   Reset the SDADC device.
 *   This is firstly called whenever the SDADC device is registered by
 *   sdadc_register()
 *   Does mostly the SDAC register setting.
 *   Leave the device in power down mode.
 *   Note that SDACx clock is already enable (for all SDADC) by the
 *   rcc_enableapb2()
 *
 * Input Parameters:
 *   dev     - pointer to the sdadc device structure
 *
 * Returned Value:
 *
 ****************************************************************************/

static void sdadc_reset(FAR struct adc_dev_s *dev)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;
  irqstate_t flags;
  uint32_t setbits = 0;

  ainfo("intf: %d\n", priv->intf);

  /* TODO: why critical ? */

  flags = enter_critical_section();

  /* Enable SDADC reset state */

  sdadc_rccreset(priv, true);

  /* Enable power */

  stm32_pwr_enablesdadc(priv->intf);

  /* Release SDADC from reset state */

  sdadc_rccreset(priv, false);

  /* Enable the SDADC (and wait it stabilizes) */

  sdadc_enable(priv, true);

  /* Put SDADC in in initialization mode */

  sdadc_putreg(priv, STM32_SDADC_CR1_OFFSET, SDADC_CR1_INIT);

  /* Wait for the SDADC to be ready */

  while ((sdadc_getreg(priv, STM32_SDADC_ISR_OFFSET) & SDADC_ISR_INITRDY) == 0);

  /* Load configurations */

  sdadc_putreg(priv, STM32_SDADC_CONF0R_OFFSET, SDADC_CONF0R_DEFAULT);
  sdadc_putreg(priv, STM32_SDADC_CONF1R_OFFSET, SDADC_CONF1R_DEFAULT);
  sdadc_putreg(priv, STM32_SDADC_CONF2R_OFFSET, SDADC_CONF2R_DEFAULT);

  sdadc_putreg(priv, STM32_SDADC_CONFCHR1_OFFSET, SDADC_CONFCHR1_DEFAULT);
  sdadc_putreg(priv, STM32_SDADC_CONFCHR2_OFFSET, SDADC_CONFCHR2_DEFAULT);

  /* Configuration of the injected channels group */

  sdadc_set_ch(dev, 0);

  /* CR1 ********************************************************************/

  /* Enable interrupt / dma flags, is done later by upper half when opening
   * device by calling sdadc_rxint()
   */

  setbits = SDADC_CR1_INIT; /* remains in init mode while configuring */

  /* Reference voltage */

  setbits |= priv->refv;

  /* Set CR1 configuration */

  sdadc_putreg(priv, STM32_SDADC_CR1_OFFSET, setbits);

  /* CR2 ********************************************************************/

  setbits = SDADC_CR2_ADON; // leave it ON !

  /* TODO: JEXTEN / JEXTSEL */

  /* Number of calibrations is for 3 configurations */

  setbits |= (2 << SDADC_CR2_CALIBCNT_SHIFT);

  /* Set CR2 configuration */

  sdadc_putreg(priv, STM32_SDADC_CR2_OFFSET, setbits);

  /* Release INIT mode ******************************************************/

  sdadc_modifyreg(priv, STM32_SDADC_CR1_OFFSET, SDADC_CR1_INIT, 0);

  /* Calibrate the SDADC */

  sdadc_modifyreg(priv, STM32_SDADC_CR2_OFFSET, 0, SDADC_CR2_STARTCALIB);

  /* Wait for the calibration to complete (may take up to 5ms) */

  while ((sdadc_getreg(priv, STM32_SDADC_ISR_OFFSET) & SDADC_ISR_EOCALF) == 0);

  /* Clear this flag */

  sdadc_modifyreg(priv, STM32_SDADC_CLRISR_OFFSET, SDADC_CLRISR_CLREOCALF, 0);

#ifdef SDADC_HAVE_TIMER
  if (priv->tbase != 0)
    {
      ret = sdadc_timinit(priv);
      if (ret < 0)
        {
          aerr("ERROR: sdadc_timinit failed: %d\n", ret);
        }
    }
#endif

  /* Put the device in low power mode until it is actually used by
   * application code.
   */

  sdadc_enable(priv, false);

  leave_critical_section(flags);

  ainfo("CR1:  0x%08x CR2:  0x%08x\n",
        sdadc_getreg(priv, STM32_SDADC_CR1_OFFSET),
        sdadc_getreg(priv, STM32_SDADC_CR2_OFFSET));

  ainfo("CONF0R: 0x%08x CONF1R: 0x%08x CONF3R: 0x%08x\n",
        sdadc_getreg(priv, STM32_SDADC_CONF0R_OFFSET),
        sdadc_getreg(priv, STM32_SDADC_CONF1R_OFFSET),
        sdadc_getreg(priv, STM32_SDADC_CONF2R_OFFSET));

  ainfo("CONFCHR1: 0x%08x CONFCHR2: 0x%08x JCHGR: 0x%08x\n",
        sdadc_getreg(priv, STM32_SDADC_CONFCHR1_OFFSET),
        sdadc_getreg(priv, STM32_SDADC_CONFCHR2_OFFSET),
        sdadc_getreg(priv, STM32_SDADC_JCHGR_OFFSET));
}

/****************************************************************************
 * Name: sdadc_setup
 *
 * Description:
 *   Configure the ADC. This method is called the first time that the SDADC
 *   device is opened.
 *   This is called by the upper half driver sdadc_open().
 *   This will occur when the port is first
 *   opened in the application code (/dev/sdadcN).
 *   It would be called again after closing all references to this file and
 *   reopening it.
 *   This function wakes up the device and setup the DMA / IRQ
 *
 * Input Parameters:
 *   dev     - pointer to the sdadc device structure
 *
 * Returned Value:
 *
 ****************************************************************************/

static int sdadc_setup(FAR struct adc_dev_s *dev)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;
  int ret;

  /* Wakes up the device */

  sdadc_enable(priv, true);

  /* Setup DMA or interrupt control. Note that either DMA or interrupt is
   * setup not both.
   */

#ifdef SDADC_HAVE_DMA
  if (priv->hasdma)
    {
      /* Setup DMA */
      /* Stop and free DMA if it was started before */

      if (priv->dma != NULL)
        {
          stm32_dmastop(priv->dma);
          stm32_dmafree(priv->dma);
        }

      priv->dma = stm32_dmachannel(priv->dmachan);

      stm32_dmasetup(priv->dma,
                     priv->base + STM32_SDADC_JDATAR_OFFSET,
                     (uint32_t)priv->dmabuffer,
                     priv->nchannels,
                     SDADC_DMA_CONTROL_WORD);

      stm32_dmastart(priv->dma, sdadc_dmaconvcallback, dev, false);
    }
  else
    {
      /* Attach the SDADC interrupt */

      ret = irq_attach(priv->irq, sdadc_interrupt, dev);
      if (ret < 0)
        {
          ainfo("irq_attach failed: %d\n", ret);
          return ret;
        }
    }
#else
  /* Attach the SDADC interrupt */

  ret = irq_attach(priv->irq, sdadc_interrupt, dev);
  if (ret < 0)
    {
      ainfo("irq_attach failed: %d\n", ret);
      return ret;
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: sdadc_shutdown
 *
 * Description:
 *   Disable the ADC.  This method is called when the last instance
 *   of the SDADC device is closed by the user application.
 *   This method reverses the operation the setup method.
 *
 * Input Parameters:
 *   dev     - pointer to the sdadc device structure
 *
 * Returned Value:
 *
 ****************************************************************************/

static void sdadc_shutdown(FAR struct adc_dev_s *dev)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;

  /* Put the device in low power mode */

  sdadc_enable(priv, false);

  /* Disable intrerrupt / dma  */

  sdadc_rxint(dev, false);

#ifdef SDADC_HAVE_DMA
  if (priv->hasdma)
    {
      /* Stop and free DMA if it was started before */

      if (priv->dma != NULL)
        {
          stm32_dmastop(priv->dma);
          stm32_dmafree(priv->dma);
        }
    }
  else
    {
      /* Disable ADC interrupts and detach the SDADC interrupt handler */

      up_disable_irq(priv->irq);
      irq_detach(priv->irq);
    }
#else
      /* Disable ADC interrupts and detach the SDADC interrupt handler */

      up_disable_irq(priv->irq);
      irq_detach(priv->irq);
#endif
}

/****************************************************************************
 * Name: sdadc_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts.
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static void sdadc_rxint(FAR struct adc_dev_s *dev, bool enable)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;
  uint32_t setbits;

  ainfo("intf: %d enable: %d\n", priv->intf, enable ? 1 : 0);

  /* DMA mode */

#ifdef SDADC_HAVE_DMA
  if (priv->hasdma)
    {
      setbits = SDADC_CR1_JDMAEN; // DMA enabled for injected channels group
    }
  else
    {
      /* Interrupt enable for injected channel group overrun
          and end of conversion */
      setbits = SDADC_CR1_JOVRIE | SDADC_CR1_JEOCIE;
    }
#else
  setbits = SDADC_CR1_JOVRIE | SDADC_CR1_JEOCIE;
#endif

  if (enable)
    {
      /* Enable */

      sdadc_modifyreg(priv, STM32_SDADC_CR1_OFFSET, 0, setbits);
    }
  else
    {
      /* Disable all ADC interrupts and DMA */

      sdadc_modifyreg(priv, STM32_SDADC_CR1_OFFSET,
                      SDADC_CR1_JOVRIE | SDADC_CR1_JEOCIE | SDADC_CR1_JDMAEN,
                      0);
    }
}

/****************************************************************************
 * Name: sdadc_set_ch
 *
 * Description:
 *   Sets the SDADC injected channel group.
 *
 * Input Parameters:
 *   dev - pointer to device structure used by the driver
 *   ch  - ADC channel number + 1. 0 reserved for all configured channels
 *
 * Returned Value:
 *   int - errno
 *
 ****************************************************************************/

static int sdadc_set_ch(FAR struct adc_dev_s *dev, uint8_t ch)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;
  uint32_t bits = 0;
  int i;

  if (ch == 0)
    {
      priv->current   = 0;
      priv->nchannels = priv->cchannels;
    }
  else
    {
      for (i = 0; i < priv->cchannels && priv->chanlist[i] != ch - 1; i++);

      if (i >= priv->cchannels)
        {
          return -ENODEV;
        }

      priv->current   = i;
      priv->nchannels = 1;
    }

  for (i = 0; i < priv->nchannels; i++)
    {
      bits |= (uint32_t)(1 << priv->chanlist[i]);
    }

  sdadc_putreg(priv, STM32_SDADC_JCHGR_OFFSET, bits);

  return OK;
}

/****************************************************************************
 * Name: sdadc_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method.
 *
 * Input Parameters:
 *   dev - pointer to device structure used by the driver
 *   cmd - command
 *   arg - arguments passed with command
 *
 * Returned Value:
 *
 ****************************************************************************/

static int sdadc_ioctl(FAR struct adc_dev_s *dev, int cmd, unsigned long arg)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;
  int ret = OK;

  switch (cmd)
    {
      case ANIOC_TRIGGER:
        sdadc_startconv(priv, true);
        break;

      default:
        aerr("ERROR: Unknown cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: sdadc_interrupt
 *
 * Description:
 *   Common SDADC interrupt handler.
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static int sdadc_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct adc_dev_s *dev = (FAR struct adc_dev_s *)arg;
  FAR struct stm32_dev_s *priv;
  uint32_t regval;
  uint32_t pending;
  int32_t  data;
  uint8_t  chan;

  DEBUGASSERT(dev != NULL && dev->ad_priv != NULL);
  priv = (FAR struct stm32_dev_s *)dev->ad_priv;

  regval  = sdadc_getreg(priv, STM32_SDADC_ISR_OFFSET);
  pending = regval & SDADC_ISR_ALLINTS;
  if (pending == 0)
    {
      return OK;
    }

  /* JOVRF: overrun flag */

  if ((regval & SDADC_ISR_JOVRF) != 0)
    {
      awarn("WARNING: Overrun has occurred!\n");
    }

  /* JEOCF: End of conversion */

  if ((regval & SDADC_ISR_JEOCF) != 0)
    {
      /* Read the converted value and clear JEOCF bit
       * (It is cleared by reading the SDADC_JDATAR) */

      data = sdadc_getreg(priv, STM32_SDADC_JDATAR_OFFSET) & SDADC_JDATAR_JDATA_MASK;
      chan = sdadc_getreg(priv, STM32_SDADC_JDATAR_OFFSET) & SDADC_JDATAR_JDATACH_MASK;

      DEBUGASSERT(priv->chanlist[priv->current] == chan);

      /* Verify that the upper-half driver has bound its callback functions */

      if (priv->cb != NULL)
        {
          /* Give the SDADC data to the ADC driver.  The ADC receive() method
           * accepts 3 parameters:
           *
           * 1) The first is the ADC device instance for this SDADC block.
           * 2) The second is the channel number for the data, and
           * 3) The third is the converted data for the channel.
           */

          DEBUGASSERT(priv->cb->au_receive != NULL);
          priv->cb->au_receive(dev, chan, data);
        }

      /* Set the channel number of the next channel that will complete
       * conversion.
       */

      priv->current++;

      if (priv->current >= priv->nchannels)
        {
          /* Restart the conversion sequence from the beginning */

          priv->current = 0;
        }

      /* do no clear this interrupt (cleared by reading data) */

      pending &= ~SDADC_ISR_JEOCF;
    }

  /* Clears interrupt flags, if any */

  if (pending)
    {
      sdadc_putreg(priv, STM32_SDADC_CLRISR_OFFSET, pending);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_sdadcinitialize
 *
 * Description:
 *   Initialize one SDADC block
 *
 *   The logic is, save and initialize the channel list in the private driver
 *   structure and return the corresponding adc device structure.
 *
 *   Each SDADC will convert the channels indicated each
 *   time a conversion is triggered either by sofware, timer or external event.
 *   Channels are numbered from 0 - 8 and must be given in order (contrarily
 *   to what says ST RM0313 doc !!!).
 *
 * Input Parameters:
 *   intf      - Could be {1,2,3} for SDADC1, SDADC2, or SDADC3
 *   chanlist  - The list of channels eg. { 0, 3, 7, 8 }
 *   cchannels - Number of channels
 *
 * Returned Value:
 *   Valid ADC device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct adc_dev_s *stm32_sdadcinitialize(int intf, FAR const uint8_t *chanlist,
                                        int cchannels)
{
  FAR struct adc_dev_s   *dev;
  FAR struct stm32_dev_s *priv;
  int i;

  ainfo("intf: %d cchannels: %d\n", intf, cchannels);

  switch (intf)
    {
#ifdef CONFIG_STM32_SDADC1
      case 1:
        ainfo("SDADC1 selected\n");
        dev = &g_sdadcdev1;
        break;
#endif
#ifdef CONFIG_STM32_SDADC2
      case 2:
        ainfo("SDADC2 selected\n");
        dev = &g_sdadcdev2;
        break;
#endif
#ifdef CONFIG_STM32_SDADC3
      case 3:
        ainfo("SDADC3 selected\n");
        dev = &g_sdadcdev3;
        break;
#endif
      default:
        aerr("ERROR: No SDADC interface defined\n");
        return NULL;
    }

  /* Check channel list in order */

  DEBUGASSERT((cchannels <= SDADC_MAX_SAMPLES) && (cchannels > 0));
  for (i = 0; i < cchannels - 1; i ++)
    {
      if (chanlist[i] >= chanlist[i+1])
        {
          aerr("ERROR: SDADC channel list must be given in order\n");
          return NULL;
        }
    }

  /* Configure the selected SDADC */

  priv = (FAR struct stm32_dev_s *)dev->ad_priv;

  priv->cb        = NULL;
  priv->cchannels = cchannels;

  memcpy(priv->chanlist, chanlist, cchannels);

  return dev;
}

#endif /* CONFIG_STM32_SDADC1 || CONFIG_STM32_SDADC2 ||
        * CONFIG_STM32_SDADC3  */
#endif /* CONFIG_STM32_SDADC */
