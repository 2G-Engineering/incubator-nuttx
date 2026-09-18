/****************************************************************************
 * arch/arm/src/stm32h7/stm32_adc.c
 *
 *   Copyright (C) 2017, 2019 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2015 Motorola Mobility, LLC. All rights reserved.
 *   Copyright (C) 2015 Omni Hoverboards Inc. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Diego Sanchez <dsanchez@nx-engineering.com>
 *            Paul Alexander Patience <paul-a.patience@polymtl.ca>
 *            Mateusz Szafoni <raiden00@railab.me>
 *            Juha Niskanen <juha.niskanen@haltian.com>
 *            David Sidrane <david_s5@nscdg.com>
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
#include <sys/ioctl.h>

#include <errno.h>
#include <assert.h>
#include <debug.h>
#include <string.h>

#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/power/pm.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>

#include "chip.h"
#include "stm32_rcc.h"
#include "stm32_tim.h"
#include "stm32_dma.h"
#include "stm32_adc.h"

/* ADC "upper half" support must be enabled */

#ifdef CONFIG_ADC

/* Some ADC peripheral must be enabled */

#if defined(CONFIG_STM32H7_ADC1) || defined(CONFIG_STM32H7_ADC2) || \
    defined(CONFIG_STM32H7_ADC3)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if STM32_RCC_D3CCIPR_ADCSEL == RCC_D3CCIPR_ADCSEL_PLL2
#define STM32_ADC_CLK_FREQ    STM32_PLL2P_FREQUENCY
#elif STM32_RCC_D3CCIPR_ADCSEL == RCC_D3CCIPR_ADCSEL_PLL3
#define STM32_ADC_CLK_FREQ    STM32_PLL3R_FREQUENCY
#elif STM32_RCC_D3CCIPR_ADCSEL == RCC_D3CCIPR_ADCSEL_PER
/* Need to get clock frequency from board.h */
#error PER_CK not supported
#else
#error Unknown ADC clock source
#endif

#ifndef container_of
#  define container_of(ptr, type, member) \
          ((type *)((intptr_t)(ptr) - offsetof(type, member)))
#endif

/* ADC Channels/DMA *********************************************************/

/* The maximum number of channels that can be sampled.  While DMA support is
 * very nice for reliable multi-channel sampling, the STM32H7 can function
 * without, although there is a risk of overrun.
 */

#define ADC_MAX_CHANNELS_DMA   20
#define ADC_MAX_CHANNELS_NODMA 20

#ifdef ADC_HAVE_DMA
#  if !defined(CONFIG_STM32H7_DMA1) && !defined(CONFIG_STM32H7_DMA2)
#    /* REVISIT: check accordingly to which one is configured in board.h */
#    error "STM32H7 ADC DMA support requires CONFIG_STM32H7_DMA1 or CONFIG_STM32H7_DMA2"
#  endif
#endif

#ifdef ADC_HAVE_DMA
#  define ADC_MAX_SAMPLES ADC_MAX_CHANNELS_DMA
#else
#  define ADC_MAX_SAMPLES ADC_MAX_CHANNELS_NODMA
#endif

#define ADC_DMA_CONTROL_WORD (DMA_CCR_MSIZE_16BITS | \
                              DMA_CCR_PSIZE_16BITS | \
                              DMA_CCR_MINC | \
                              DMA_CCR_CIRC)

/* DMA channels and interface values */

#define ADC_SMPR_DEFAULT    ADC_SMPR_810p5
#define ADC_SMPR1_DEFAULT   ((ADC_SMPR_DEFAULT << ADC_SMPR1_SMP0_SHIFT) | \
                             (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP1_SHIFT) | \
                             (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP2_SHIFT) | \
                             (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP3_SHIFT) | \
                             (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP4_SHIFT) | \
                             (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP5_SHIFT) | \
                             (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP6_SHIFT) | \
                             (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP7_SHIFT) | \
                             (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP8_SHIFT) | \
                             (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP9_SHIFT))
#define ADC_SMPR2_DEFAULT   ((ADC_SMPR_DEFAULT << ADC_SMPR2_SMP10_SHIFT) | \
                             (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP11_SHIFT) | \
                             (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP12_SHIFT) | \
                             (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP13_SHIFT) | \
                             (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP14_SHIFT) | \
                             (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP15_SHIFT) | \
                             (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP16_SHIFT) | \
                             (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP17_SHIFT) | \
                             (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP18_SHIFT) | \
                             (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP19_SHIFT))

/* The channels not in this range are internal channels for reading
 * Vbat ADC3:17, VSense (temperature) ADC3:18 and Vrefint ADC3:19
 *
 * N.B. DAC channels on ADC2 are not supported
 */

#define ADC_EXTERNAL_CHAN_MAX  16

/* Number of channels per ADC */

#define ADC_CHANNELS_NUMBER 19

/* Max 4 injected channels */

#define ADC_INJ_MAX_SAMPLES   4

/* ADC software trigger configuration */

#define ANIOC_TRIGGER_REGULAR  (1 << 0)
#define ANIOC_TRIGGER_INJECTED (1 << 1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the state of one ADC block */

struct stm32_dev_s
{
#ifdef CONFIG_STM32H7_ADC_LL_OPS
  const struct stm32_adc_ops_s *llops; /* Low-level ADC ops */
  struct adc_dev_s             *dev;   /* Upper-half ADC reference */
#endif
  const struct adc_callback_s *cb;
  uint8_t irq;          /* Interrupt generated by this ADC block */
  uint8_t rnchannels;    /* Number of regular channels */
  uint8_t cr_channels;    /* Number of configured regular channels */
#ifdef ADC_HAVE_INJECTED
  uint8_t cj_channels;  /* Number of configured injected channels */
#endif
  uint8_t intf;         /* ADC interface number */
  uint8_t current;      /* Current ADC channel being converted */
  uint8_t anioc_trg;    /* ANIOC_TRIGGER configuration */
  uint8_t resolution;   /* ADC resolution (0-3) */
#ifdef ADC_HAVE_DMA
  uint8_t dmachan;      /* DMA channel needed by this ADC */
  bool    hasdma;       /* True: This ADC supports DMA */
#endif
#ifdef ADC_HAVE_DFSDM
  bool    hasdfsdm;     /* True: This ADC routes its output to DFSDM */
#endif
#ifdef ADC_HAVE_TIMER
  uint8_t trigger;      /* Timer trigger channel: 0=CC1, 1=CC2, 2=CC3,
                         * 3=CC4, 4=TRGO, 5=TRGO2 */
#endif
#ifdef CONFIG_STM32H7_ADC_CHANGE_SAMPLETIME
  /* Sample time selection. These bits must be written only when ADON=0.
   * REVISIT: this takes too much space. We need only 3 bits per channel.
   */

  uint8_t sample_rate[ADC_CHANNELS_NUMBER];
  uint8_t adc_channels;      /* ADC channels number */
#endif
  xcpt_t   isr;         /* Interrupt handler for this ADC block */
  uint32_t base;        /* Base address of registers unique to this ADC
                         * block */
  uint32_t mbase;       /* Base address of master ADC (allows for access to
                         * shared common registers) */
  bool     initialized; /* Keeps track of the initialization status of the ADC */
#ifdef ADC_HAVE_TIMER
  uint32_t tbase;       /* Base address of timer used by this ADC block */
  uint32_t trcc_enr;    /* RCC ENR Register */
  uint32_t trcc_en;     /* RCC EN Bit in ENR Register */
  uint32_t extsel;      /* EXTSEL value used by this ADC block */
  uint32_t pclck;       /* The PCLK frequency that drives this timer */
  uint32_t freq;        /* The desired frequency of conversions */
#endif
#ifdef ADC_HAVE_EXTCFG
  uint32_t extcfg;           /* External event configuration for regular group */
#endif
#ifdef ADC_HAVE_JEXTCFG
  uint32_t jextcfg;          /* External event configuration for injected group */
#endif
#ifdef CONFIG_PM
  struct pm_callback_s pm_callback;
#endif

#ifdef ADC_HAVE_DMA
  DMA_HANDLE dma;       /* Allocated DMA channel */

  /* DMA transfer buffer */

  uint16_t dmabuffer[ADC_MAX_SAMPLES];
#endif

  /* List of selected ADC channels to sample */

  uint8_t  r_chanlist[ADC_MAX_SAMPLES];

#ifdef ADC_HAVE_INJECTED
  /* List of selected ADC injected channels to sample */

  uint8_t  j_chanlist[ADC_INJ_MAX_SAMPLES];
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* ADC Register access */

static uint32_t adc_getreg(struct stm32_dev_s *priv, int offset);
static void     adc_putreg(struct stm32_dev_s *priv, int offset,
                           uint32_t value);
static void     adc_modifyreg(struct stm32_dev_s *priv, int offset,
                              uint32_t clrbits, uint32_t setbits);

#ifdef ADC_HAVE_TIMER
static uint16_t tim_getreg(struct stm32_dev_s *priv, int offset);
static void     tim_putreg(struct stm32_dev_s *priv, int offset,
                           uint16_t value);
static void     tim_modifyreg(struct stm32_dev_s *priv, int offset,
                              uint16_t clrbits, uint16_t setbits);
static void     tim_dumpregs(struct stm32_dev_s *priv,
                             const char *msg);
#endif

/* ADC Miscellaneous Helpers */

static void adc_rccreset(struct stm32_dev_s *priv, bool reset);
static void adc_startup(struct stm32_dev_s *priv);
static void adc_enable(struct stm32_dev_s *priv, bool enable);
static uint32_t adc_sqrbits(struct stm32_dev_s *priv, int first,
                            int last, int offset);
static int      adc_set_ch(struct adc_dev_s *dev, uint8_t ch);
static bool     adc_internal(struct stm32_dev_s * priv,
                             uint32_t *adc_ccr);

static int adc_resolution_set(struct adc_dev_s *dev, uint8_t res);

#ifdef ADC_HAVE_TIMER
static void adc_timstart(struct stm32_dev_s *priv, bool enable);
static int  adc_timinit(struct stm32_dev_s *priv);
#endif

#ifdef ADC_HAVE_DMA
static void adc_dmaconvcallback(DMA_HANDLE handle, uint8_t isr,
                                void *arg);
#endif

#ifdef ADC_HAVE_DFSDM
static int adc_setoffset(struct stm32_dev_s *priv, uint8_t ch, uint8_t i,
                         uint16_t offset);
#endif

static void adc_reg_startconv(struct stm32_dev_s *priv, bool enable);
#ifdef ADC_HAVE_INJECTED
static void adc_inj_startconv(struct stm32_dev_s *priv, bool enable);
static int adc_inj_set_ch(struct adc_dev_s *dev, uint8_t ch);
#endif

#ifdef ADC_HAVE_EXTCFG
static int adc_extcfg_set(struct stm32_dev_s *priv, uint32_t extcfg);
#endif
#ifdef ADC_HAVE_JEXTCFG
static int adc_jextcfg_set(struct stm32_dev_s *priv, uint32_t jextcfg);
#endif

#ifdef CONFIG_PM
static int adc_pm_prepare(struct pm_callback_s *cb, int domain,
                          enum pm_state_e state);
#endif

/* ADC Interrupt Handler */

static int adc_interrupt(struct adc_dev_s *dev, uint32_t regval);
#if defined(CONFIG_STM32H7_ADC1) || defined(CONFIG_STM32H7_ADC2)
static int adc12_interrupt(int irq, void *context, void *arg);
#endif
#if defined(CONFIG_STM32H7_ADC3)
static int adc3_interrupt(int irq, void *context, void *arg);
#endif

/* ADC Driver Methods */

static int  adc_bind(struct adc_dev_s *dev,
                     const struct adc_callback_s *callback);
static void adc_reset(struct adc_dev_s *dev);
static int  adc_setup(struct adc_dev_s *dev);
static void adc_shutdown(struct adc_dev_s *dev);
static void adc_rxint(struct adc_dev_s *dev, bool enable);
static int  adc_ioctl(struct adc_dev_s *dev, int cmd, unsigned long arg);

static void adc_dumpregs(struct stm32_dev_s *priv);

#ifdef CONFIG_STM32H7_ADC_LL_OPS
static int adc_llops_setup(struct stm32_adc_dev_s *dev);
static void adc_llops_shutdown(struct stm32_adc_dev_s *dev);
static void adc_intack(struct stm32_adc_dev_s *dev, uint32_t source);
static void adc_inten(struct stm32_adc_dev_s *dev, uint32_t source);
static void adc_intdis(struct stm32_adc_dev_s *dev, uint32_t source);
static uint32_t adc_intget(struct stm32_adc_dev_s *dev);
static uint32_t adc_regget(struct stm32_adc_dev_s *dev);
static void adc_llops_reg_startconv(struct stm32_adc_dev_s *dev,
                                    bool enable);
static int adc_offset_set(struct stm32_adc_dev_s *dev, uint8_t ch,
                          uint8_t i, uint32_t offset);
#  ifdef ADC_HAVE_EXTCFG
static void adc_llops_extcfg_set(struct stm32_adc_dev_s *dev,
                                 uint32_t extcfg);
#  endif
#  ifdef ADC_HAVE_JEXTCFG
static void adc_llops_jextcfg_set(struct stm32_adc_dev_s *dev,
                                  uint32_t jextcfg);
#  endif
#  ifdef ADC_HAVE_DMA
static int adc_regbufregister(struct stm32_adc_dev_s *dev,
                              uint16_t *buffer, uint8_t len);
#  endif
#  ifdef ADC_HAVE_INJECTED
static uint32_t adc_injget(struct stm32_adc_dev_s *dev, uint8_t chan);
static void adc_llops_inj_startconv(struct stm32_adc_dev_s *dev,
                                    bool enable);
#  endif
#  ifdef CONFIG_STM32H7_ADC_CHANGE_SAMPLETIME
static void adc_sampletime_set(struct stm32_adc_dev_s *dev,
                               struct adc_sample_time_s *time_samples);
static void adc_sampletime_write(struct stm32_adc_dev_s *dev);
#  endif
static void adc_llops_dumpregs(struct stm32_adc_dev_s *dev);
static int  adc_llops_multicfg(struct stm32_adc_dev_s *dev, uint8_t mode);
static void adc_llops_enable(struct stm32_adc_dev_s *dev, bool enable);
#endif


/****************************************************************************
 * Private Data
 ****************************************************************************/

/* ADC interface operations */

static const struct adc_ops_s g_adcops =
{
  .ao_bind        = adc_bind,
  .ao_reset       = adc_reset,
  .ao_setup       = adc_setup,
  .ao_shutdown    = adc_shutdown,
  .ao_rxint       = adc_rxint,
  .ao_ioctl       = adc_ioctl,
};

/* Publicly visible ADC lower-half operations */

#ifdef CONFIG_STM32H7_ADC_LL_OPS
static const struct stm32_adc_ops_s g_adc_llops =
{
  .setup         = adc_llops_setup,
  .shutdown      = adc_llops_shutdown,
  .int_ack       = adc_intack,
  .int_get       = adc_intget,
  .int_en        = adc_inten,
  .int_dis       = adc_intdis,
  .val_get       = adc_regget,
  .reg_startconv = adc_llops_reg_startconv,
  .offset_set    = adc_offset_set,
#  ifdef ADC_HAVE_DMA
  .regbuf_reg    = adc_regbufregister,
#  endif
#  ifdef ADC_HAVE_EXTCFG
  .extcfg_set    = adc_llops_extcfg_set,
#  endif
#  ifdef ADC_HAVE_JEXTCFG
  .jextcfg_set   = adc_llops_jextcfg_set,
#  endif
#  ifdef ADC_HAVE_INJECTED
  .inj_get       = adc_injget,
  .inj_startconv = adc_llops_inj_startconv,
#  endif
#  ifdef CONFIG_STM32H7_ADC_CHANGE_SAMPLETIME
  .stime_set     = adc_sampletime_set,
  .stime_write   = adc_sampletime_write,
#  endif
  .dump_regs     = adc_llops_dumpregs,
  .multi_cfg     = adc_llops_multicfg,
  .enable        = adc_llops_enable
};
#endif

/* ADC1 state */

#ifdef CONFIG_STM32H7_ADC1
static struct stm32_dev_s g_adcpriv1 =
{
  .irq         = STM32_IRQ_ADC12,
  .isr         = adc12_interrupt,
  .intf        = 1,
  .base        = STM32_ADC1_BASE,
  .mbase       = STM32_ADC1_BASE,
  .initialized = false,
  .anioc_trg   = CONFIG_STM32H7_ADC1_ANIOC_TRIGGER,
  .resolution  = CONFIG_STM32H7_ADC1_RESOLUTION,
#ifdef ADC1_HAVE_EXTCFG
  .extcfg      = ADC1_EXTCFG_VALUE,
#endif
#ifdef ADC1_HAVE_JEXTCFG
  .jextcfg     = ADC1_JEXTCFG_VALUE,
#endif
#ifdef ADC1_HAVE_TIMER
  .trigger     = CONFIG_STM32H7_ADC1_TIMTRIG,
  .tbase       = ADC1_TIMER_BASE,
  .trcc_enr    = ADC1_TIMER_RCC_ENR,
  .trcc_en     = ADC1_TIMER_RCC_EN,
  .extsel      = ADC1_EXTSEL_VALUE,
  .pclck       = ADC1_TIMER_PCLK_FREQUENCY,
  .freq        = CONFIG_STM32H7_ADC1_SAMPLE_FREQUENCY,
#endif
#ifdef ADC1_HAVE_DMA
  .dmachan     = ADC1_DMA_CHAN,
  .hasdma      = true,
#endif
#ifdef ADC1_HAVE_DFSDM
  .hasdfsdm    = true,
#endif
#ifdef CONFIG_PM
  .pm_callback =
    {
      .prepare = adc_pm_prepare,
    }
#endif
#ifdef CONFIG_STM32H7_ADC_LL_OPS
  .llops       = &g_adc_llops,
#endif
};

static struct adc_dev_s g_adcdev1 =
{
  .ad_ops      = &g_adcops,
  .ad_priv     = &g_adcpriv1,
};
#endif

/* ADC2 state */

#ifdef CONFIG_STM32H7_ADC2
static struct stm32_dev_s g_adcpriv2 =
{
  .irq         = STM32_IRQ_ADC12,
  .isr         = adc12_interrupt,
  .intf        = 2,
  .base        = STM32_ADC2_BASE,
  .mbase       = STM32_ADC1_BASE,
  .initialized = false,
  .anioc_trg   = CONFIG_STM32H7_ADC2_ANIOC_TRIGGER,
  .resolution  = CONFIG_STM32H7_ADC2_RESOLUTION,
#ifdef ADC2_HAVE_EXTCFG
  .extcfg      = ADC2_EXTCFG_VALUE,
#endif
#ifdef ADC2_HAVE_JEXTCFG
  .jextcfg     = ADC2_JEXTCFG_VALUE,
#endif
#ifdef ADC2_HAVE_TIMER
  .trigger     = CONFIG_STM32H7_ADC2_TIMTRIG,
  .tbase       = ADC2_TIMER_BASE,
  .trcc_enr    = ADC2_TIMER_RCC_ENR,
  .trcc_en     = ADC2_TIMER_RCC_EN,
  .extsel      = ADC2_EXTSEL_VALUE,
  .pclck       = ADC2_TIMER_PCLK_FREQUENCY,
  .freq        = CONFIG_STM32H7_ADC2_SAMPLE_FREQUENCY,
#endif
#ifdef ADC2_HAVE_DMA
  .dmachan     = ADC2_DMA_CHAN,
  .hasdma      = true,
#endif
#ifdef ADC2_HAVE_DFSDM
  .hasdfsdm    = true,
#endif
#ifdef CONFIG_PM
  .pm_callback =
    {
      .prepare = adc_pm_prepare,
    }
#endif
#ifdef CONFIG_STM32H7_ADC_LL_OPS
  .llops       = &g_adc_llops,
#endif
};

static struct adc_dev_s g_adcdev2 =
{
  .ad_ops      = &g_adcops,
  .ad_priv     = &g_adcpriv2,
};
#endif

/* ADC3 state */

#ifdef CONFIG_STM32H7_ADC3
static struct stm32_dev_s g_adcpriv3 =
{
  .irq         = STM32_IRQ_ADC3,
  .isr         = adc3_interrupt,
  .intf        = 3,
  .base        = STM32_ADC3_BASE,
  .mbase       = STM32_ADC3_BASE,
  .initialized = false,
  .anioc_trg   = CONFIG_STM32H7_ADC3_ANIOC_TRIGGER,
  .resolution  = CONFIG_STM32H7_ADC3_RESOLUTION,
#ifdef ADC3_HAVE_EXTCFG
  .extcfg      = ADC3_EXTCFG_VALUE,
#endif
#ifdef ADC3_HAVE_JEXTCFG
  .jextcfg     = ADC3_JEXTCFG_VALUE,
#endif
#ifdef ADC3_HAVE_TIMER
  .trigger     = CONFIG_STM32H7_ADC3_TIMTRIG,
  .tbase       = ADC3_TIMER_BASE,
  .trcc_enr    = ADC3_TIMER_RCC_ENR,
  .trcc_en     = ADC3_TIMER_RCC_EN,
  .extsel      = ADC3_EXTSEL_VALUE,
  .pclck       = ADC3_TIMER_PCLK_FREQUENCY,
  .freq        = CONFIG_STM32H7_ADC3_SAMPLE_FREQUENCY,
#endif
#ifdef ADC3_HAVE_DMA
  .dmachan     = ADC3_DMA_CHAN,
  .hasdma      = true,
#endif
#ifdef ADC3_HAVE_DFSDM
  .hasdfsdm    = true,
#endif
#ifdef CONFIG_PM
  .pm_callback =
    {
      .prepare = adc_pm_prepare,
    }
#endif
#ifdef CONFIG_STM32H7_ADC_LL_OPS
  .llops       = &g_adc_llops,
#endif
};

static struct adc_dev_s g_adcdev3 =
{
  .ad_ops      = &g_adcops,
  .ad_priv     = &g_adcpriv3,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: adc_getreg
 *
 * Description:
 *   Read the value of an ADC register.
 *
 * Input Parameters:
 *   priv   - A reference to the ADC block status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   The current contents of the specified register
 *
 ****************************************************************************/

static uint32_t adc_getreg(struct stm32_dev_s *priv, int offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: adc_putreg
 *
 * Description:
 *   Write a value to an ADC register.
 *
 * Input Parameters:
 *   priv   - A reference to the ADC block status
 *   offset - The offset to the register to write to
 *   value  - The value to write to the register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void adc_putreg(struct stm32_dev_s *priv, int offset,
                       uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: adc_modifyreg
 *
 * Description:
 *   Modify the value of an ADC register (not atomic).
 *
 * Input Parameters:
 *   priv    - A reference to the ADC block status
 *   offset  - The offset to the register to modify
 *   clrbits - The bits to clear
 *   setbits - The bits to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void adc_modifyreg(struct stm32_dev_s *priv, int offset,
                          uint32_t clrbits, uint32_t setbits)
{
  adc_putreg(priv, offset, (adc_getreg(priv, offset) & ~clrbits) | setbits);
}

/****************************************************************************
 * Name: adc_getregm
 *
 * Description:
 *   Read the value of an ADC register from the associated ADC master.
 *
 * Input Parameters:
 *   priv   - A reference to the ADC block status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   The current contents of the specified register in the ADC master.
 *
 ****************************************************************************/

static uint32_t adc_getregm(struct stm32_dev_s *priv, int offset)
{
  return getreg32(priv->mbase + offset);
}

/****************************************************************************
 * Name: adc_putregm
 *
 * Description:
 *   Write a value to an ADC register in the associated ADC master.
 *
 * Input Parameters:
 *   priv   - A reference to the ADC block status
 *   offset - The offset to the register to write to
 *   value  - The value to write to the register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void adc_putregm(struct stm32_dev_s *priv, int offset,
                        uint32_t value)
{
  putreg32(value, priv->mbase + offset);
}

/****************************************************************************
 * Name: adc_modifyregm
 *
 * Description:
 *   Modify the value of an ADC register in the associated ADC master
 *  (not atomic).
 *
 * Input Parameters:
 *   priv    - A reference to the ADC block status
 *   offset  - The offset to the register to modify
 *   clrbits - The bits to clear
 *   setbits - The bits to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void adc_modifyregm(struct stm32_dev_s *priv, int offset,
                           uint32_t clrbits, uint32_t setbits)
{
  adc_putregm(priv, offset,
              (adc_getregm(priv, offset) & ~clrbits) | setbits);
}

/****************************************************************************
 * Name: tim_getreg
 *
 * Description:
 *   Read the value of an ADC timer register.
 *
 * Input Parameters:
 *   priv   - A reference to the ADC block status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   The current contents of the specified register
 *
 ****************************************************************************/

#ifdef ADC_HAVE_TIMER
static uint16_t tim_getreg(struct stm32_dev_s *priv, int offset)
{
  return getreg16(priv->tbase + offset);
}
#endif

/****************************************************************************
 * Name: tim_putreg
 *
 * Description:
 *   Write a value to an ADC timer register.
 *
 * Input Parameters:
 *   priv   - A reference to the ADC block status
 *   offset - The offset to the register to write to
 *   value  - The value to write to the register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef ADC_HAVE_TIMER
static void tim_putreg(struct stm32_dev_s *priv, int offset,
                       uint16_t value)
{
  putreg16(value, priv->tbase + offset);
}
#endif

/****************************************************************************
 * Name: tim_modifyreg
 *
 * Description:
 *   Modify the value of an ADC timer register (not atomic).
 *
 * Input Parameters:
 *   priv    - A reference to the ADC block status
 *   offset  - The offset to the register to modify
 *   clrbits - The bits to clear
 *   setbits - The bits to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef ADC_HAVE_TIMER
static void tim_modifyreg(struct stm32_dev_s *priv, int offset,
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
 *   priv - A reference to the ADC block status
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef ADC_HAVE_TIMER
static void tim_dumpregs(struct stm32_dev_s *priv, const char *msg)
{
  ainfo("%s:\n", msg);
  ainfo("  CR1: %04x CR2:  %04x SMCR:  %04x DIER:  %04x\n",
        tim_getreg(priv, STM32_GTIM_CR1_OFFSET),
        tim_getreg(priv, STM32_GTIM_CR2_OFFSET),
        tim_getreg(priv, STM32_GTIM_SMCR_OFFSET),
        tim_getreg(priv, STM32_GTIM_DIER_OFFSET));
  ainfo("   SR: %04x EGR:  0000 CCMR1: %04x CCMR2: %04x\n",
        tim_getreg(priv, STM32_GTIM_SR_OFFSET),
        tim_getreg(priv, STM32_GTIM_CCMR1_OFFSET),
        tim_getreg(priv, STM32_GTIM_CCMR2_OFFSET));
  ainfo(" CCER: %04x CNT:  %04x PSC:   %04x ARR:   %04x\n",
        tim_getreg(priv, STM32_GTIM_CCER_OFFSET),
        tim_getreg(priv, STM32_GTIM_CNT_OFFSET),
        tim_getreg(priv, STM32_GTIM_PSC_OFFSET),
        tim_getreg(priv, STM32_GTIM_ARR_OFFSET));
  ainfo(" CCR1: %04x CCR2: %04x CCR3:  %04x CCR4:  %04x\n",
        tim_getreg(priv, STM32_GTIM_CCR1_OFFSET),
        tim_getreg(priv, STM32_GTIM_CCR2_OFFSET),
        tim_getreg(priv, STM32_GTIM_CCR3_OFFSET),
        tim_getreg(priv, STM32_GTIM_CCR4_OFFSET));

  if (priv->tbase == STM32_TIM1_BASE || priv->tbase == STM32_TIM8_BASE)
    {
      ainfo("  RCR: %04x BDTR: %04x DCR:   %04x DMAR:  %04x\n",
            tim_getreg(priv, STM32_ATIM_RCR_OFFSET),
            tim_getreg(priv, STM32_ATIM_BDTR_OFFSET),
            tim_getreg(priv, STM32_ATIM_DCR_OFFSET),
            tim_getreg(priv, STM32_ATIM_DMAR_OFFSET));
    }
  else
    {
      ainfo("  DCR: %04x DMAR: %04x\n",
            tim_getreg(priv, STM32_GTIM_DCR_OFFSET),
            tim_getreg(priv, STM32_GTIM_DMAR_OFFSET));
    }
}
#endif

/****************************************************************************
 * Name: adc_timstart
 *
 * Description:
 *   Start (or stop) the timer counter
 *
 * Input Parameters:
 *   priv - A reference to the ADC block status
 *   enable - True: Start conversion
 *
 * Returned Value:
 *
 ****************************************************************************/

#ifdef ADC_HAVE_TIMER
static void adc_timstart(struct stm32_dev_s *priv, bool enable)
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
 * Name: adc_timinit
 *
 * Description:
 *   Initialize the timer that drivers the ADC sampling for this channel
 *   using the pre-calculated timer divider definitions.
 *
 * Input Parameters:
 *   priv - A reference to the ADC block status
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef ADC_HAVE_TIMER
static int adc_timinit(struct stm32_dev_s *priv)
{
  uint32_t prescaler;
  uint32_t reload;
  uint32_t timclk;

  uint16_t clrbits = 0;
  uint16_t setbits = 0;
  uint16_t cr2;
  uint16_t ccmr1;
  uint16_t ccmr2;
  uint16_t ocmode1;
  uint16_t ocmode2;
  uint16_t ccenable;
  uint16_t ccer;
  uint16_t egr;

  /* If the timer base address is zero, then this ADC was not configured to
   * use a timer.
   */

  if (priv->tbase == 0)
    {
      return ERROR;
    }

  /* EXTSEL selection: These bits select the external event used to trigger
   * the start of conversion of a regular group.  NOTE:
   *
   * - The position with of the EXTSEL field varies from one STM32 MCU
   *   to another.
   * - The width of the EXTSEL field varies from one STM32 MCU to another.
   * - The value in priv->extsel is already shifted into the correct bit
   *   position.
   */

  ainfo("Initializing timers extsel = 0x%08" PRIx32 "\n", priv->extsel);

  adc_modifyreg(priv, STM32_ADC_CFGR_OFFSET,
                ADC_CFGR_EXTEN_MASK | ADC_CFGR_EXTSEL_MASK,
                ADC_CFGR_EXTEN_RISING | priv->extsel);

  /* Configure the timer channel to drive the ADC */

  /* Enable Timer clocking */

  modifyreg32(priv->trcc_enr, 0, priv->trcc_en);

  /* Calculate optimal values for the timer prescaler and for the timer
   * reload register.  If freq is the desired frequency, then
   *
   *   reload = timclk / freq
   *   reload = (pclck / prescaler) / freq
   *
   * There are many solutions to do this, but the best solution will be the
   * one that has the largest reload value and the smallest prescaler value.
   * That is the solution that should give us the most accuracy in the timer
   * control.  Subject to:
   *
   *   0 <= prescaler  <= 65536
   *   1 <= reload <= 65535
   *
   * So ( prescaler = pclck / 65535 / freq ) would be optimal.
   */

  prescaler = (priv->pclck / priv->freq + 65534) / 65535;

  /* We need to decrement the prescaler value by one, but only, the value
   * does not underflow.
   */

  if (prescaler < 1)
    {
      awarn("WARNING: Prescaler underflowed.\n");
      prescaler = 1;
    }

  /* Check for overflow */

  else if (prescaler > 65536)
    {
      awarn("WARNING: Prescaler overflowed.\n");
      prescaler = 65536;
    }

  timclk = priv->pclck / prescaler;

  reload = timclk / priv->freq;
  if (reload < 1)
    {
      awarn("WARNING: Reload value underflowed.\n");
      reload = 1;
    }

  else if (reload > 65535)
    {
      awarn("WARNING: Reload value overflowed.\n");
      reload = 65535;
    }

  /* Disable the timer until we get it configured */

  adc_timstart(priv, false);

  /* Set up the timer CR1 register.
   *
   * Select the Counter Mode == count up:
   *
   * ATIM_CR1_EDGE: The counter counts up or down depending on the
   *                direction bit(DIR).
   * ATIM_CR1_DIR: 0: count up, 1: count down
   *
   * Set the clock division to zero for all
   */

  clrbits = GTIM_CR1_DIR | GTIM_CR1_CMS_MASK | GTIM_CR1_CKD_MASK;
  setbits = GTIM_CR1_EDGE;
  tim_modifyreg(priv, STM32_GTIM_CR1_OFFSET, clrbits, setbits);

  /* Set the reload and prescaler values */

  tim_putreg(priv, STM32_GTIM_PSC_OFFSET, prescaler - 1);
  tim_putreg(priv, STM32_GTIM_ARR_OFFSET, reload);

  /* Clear the advanced timers repetition counter in TIM1 */

  if (priv->tbase == STM32_TIM1_BASE || priv->tbase == STM32_TIM8_BASE)
    {
      tim_putreg(priv, STM32_ATIM_RCR_OFFSET, 0);
      tim_putreg(priv, STM32_ATIM_BDTR_OFFSET, ATIM_BDTR_MOE); /* Check me */
    }

  /* TIMx event generation: Bit 0 UG: Update generation */

  tim_putreg(priv, STM32_GTIM_EGR_OFFSET, GTIM_EGR_UG);

  /* Handle channel specific setup */

  ocmode1 = 0;
  ocmode2 = 0;

  switch (priv->trigger)
    {
      case 0: /* TimerX CC1 event */
        {
          ccenable = ATIM_CCER_CC1E;
          ocmode1  = (ATIM_CCMR_CCS_CCOUT << ATIM_CCMR1_CC1S_SHIFT) |
                     (ATIM_CCMR_MODE_PWM1 << ATIM_CCMR1_OC1M_SHIFT) |
                     ATIM_CCMR1_OC1PE;

          /* Set the event CC1 */

          egr      = ATIM_EGR_CC1G;

          /* Set the duty cycle by writing to the CCR register for this
           * channel
           */

          tim_putreg(priv, STM32_GTIM_CCR1_OFFSET, (uint16_t)(reload >> 1));
        }
        break;

      case 1: /* TimerX CC2 event */
        {
          ccenable = ATIM_CCER_CC2E;
          ocmode1  = (ATIM_CCMR_CCS_CCOUT << ATIM_CCMR1_CC2S_SHIFT) |
                     (ATIM_CCMR_MODE_PWM1 << ATIM_CCMR1_OC2M_SHIFT) |
                     ATIM_CCMR1_OC2PE;

          /* Set the event CC2 */

          egr      = ATIM_EGR_CC2G;

          /* Set the duty cycle by writing to the CCR register for this
           * channel
           */

          tim_putreg(priv, STM32_GTIM_CCR2_OFFSET, (uint16_t)(reload >> 1));
        }
        break;

      case 2: /* TimerX CC3 event */
        {
          ccenable = ATIM_CCER_CC3E;
          ocmode2  = (ATIM_CCMR_CCS_CCOUT << ATIM_CCMR2_CC3S_SHIFT) |
                     (ATIM_CCMR_MODE_PWM1 << ATIM_CCMR2_OC3M_SHIFT) |
                     ATIM_CCMR2_OC3PE;

          /* Set the event CC3 */

          egr      = ATIM_EGR_CC3G;

          /* Set the duty cycle by writing to the CCR register for this
           * channel
           */

          tim_putreg(priv, STM32_GTIM_CCR3_OFFSET, (uint16_t)(reload >> 1));
        }
        break;

      case 3: /* TimerX CC4 event */
        {
          ccenable = ATIM_CCER_CC4E;
          ocmode2  = (ATIM_CCMR_CCS_CCOUT << ATIM_CCMR2_CC4S_SHIFT) |
                     (ATIM_CCMR_MODE_PWM1 << ATIM_CCMR2_OC4M_SHIFT) |
                     ATIM_CCMR2_OC4PE;

          /* Set the event CC4 */

          egr      = ATIM_EGR_CC4G;

          /* Set the duty cycle by writing to the CCR register for this
           * channel
           */

          tim_putreg(priv, STM32_GTIM_CCR4_OFFSET, (uint16_t)(reload >> 1));
        }
        break;

      case 4: /* TimerX TRGO event */
        {
          /* TODO: TRGO support not yet implemented */

          /* Set the event TRGO */

          ccenable = 0;
          egr      = GTIM_EGR_TG;

          /* Set the duty cycle by writing to the CCR register for this
           * channel
           */

          tim_putreg(priv, STM32_GTIM_CCR4_OFFSET, (uint16_t)(reload >> 1));
        }
        break;

      default:
        aerr("ERROR: No such trigger: %d\n", priv->trigger);
        return -EINVAL;
    }

  /* Disable the Channel by resetting the CCxE Bit in the CCER register */

  ccer = tim_getreg(priv, STM32_GTIM_CCER_OFFSET);
  ccer &= ~ccenable;
  tim_putreg(priv, STM32_GTIM_CCER_OFFSET, ccer);

  /* Fetch the CR2, CCMR1, and CCMR2 register (already have ccer) */

  cr2   = tim_getreg(priv, STM32_GTIM_CR2_OFFSET);
  ccmr1 = tim_getreg(priv, STM32_GTIM_CCMR1_OFFSET);
  ccmr2 = tim_getreg(priv, STM32_GTIM_CCMR2_OFFSET);

  /* Reset the Output Compare Mode Bits and set the select output compare
   * mode
   */

  ccmr1 &= ~(ATIM_CCMR1_CC1S_MASK | ATIM_CCMR1_OC1M_MASK | ATIM_CCMR1_OC1PE |
             ATIM_CCMR1_CC2S_MASK | ATIM_CCMR1_OC2M_MASK | ATIM_CCMR1_OC2PE);
  ccmr2 &= ~(ATIM_CCMR2_CC3S_MASK | ATIM_CCMR2_OC3M_MASK | ATIM_CCMR2_OC3PE |
             ATIM_CCMR2_CC4S_MASK | ATIM_CCMR2_OC4M_MASK | ATIM_CCMR2_OC4PE);
  ccmr1 |= ocmode1;
  ccmr2 |= ocmode2;

  /* Reset the output polarity level of all channels (selects high
   * polarity)
   */

  ccer &= ~(ATIM_CCER_CC1P | ATIM_CCER_CC2P |
            ATIM_CCER_CC3P | ATIM_CCER_CC4P);

  /* Enable the output state of the selected channel (only) */

  ccer &= ~(ATIM_CCER_CC1E | ATIM_CCER_CC2E |
            ATIM_CCER_CC3E | ATIM_CCER_CC4E);
  ccer |= ccenable;

  if (priv->tbase == STM32_TIM1_BASE || priv->tbase == STM32_TIM8_BASE)
    {
      /* Reset output N polarity level, output N state, output compare state,
       * output compare N idle state.
       */

      ccer &= ~(ATIM_CCER_CC1NE | ATIM_CCER_CC1NP |
                ATIM_CCER_CC2NE | ATIM_CCER_CC2NP |
                ATIM_CCER_CC3NE | ATIM_CCER_CC3NP |
                ATIM_CCER_CC4NP);

      /* Reset the output compare and output compare N IDLE State */

      cr2 &= ~(ATIM_CR2_OIS1 | ATIM_CR2_OIS1N |
               ATIM_CR2_OIS2 | ATIM_CR2_OIS2N |
               ATIM_CR2_OIS3 | ATIM_CR2_OIS3N |
               ATIM_CR2_OIS4);
    }
  else
    {
      ccer &= ~(GTIM_CCER_CC1NP | GTIM_CCER_CC2NP | GTIM_CCER_CC3NP);
    }

  /* Save the modified register values */

  tim_putreg(priv, STM32_GTIM_CR2_OFFSET, cr2);
  tim_putreg(priv, STM32_GTIM_CCMR1_OFFSET, ccmr1);
  tim_putreg(priv, STM32_GTIM_CCMR2_OFFSET, ccmr2);
  tim_putreg(priv, STM32_GTIM_CCER_OFFSET, ccer);
  tim_putreg(priv, STM32_GTIM_EGR_OFFSET, egr);

  /* Set the ARR Preload Bit */

  tim_modifyreg(priv, STM32_GTIM_CR1_OFFSET, 0, GTIM_CR1_ARPE);

  /* Enable the timer counter */

  adc_timstart(priv, true);

  tim_dumpregs(priv, "After starting timers");

  return OK;
}
#endif

/****************************************************************************
 * Name: adc_pm_prepare
 *
 * Description:
 *   Called by power management framework when it wants to enter low power
 *   states. Check if ADC is in progress and if so prevent from entering
 *   STOP.
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static int adc_pm_prepare(struct pm_callback_s *cb, int domain,
                          enum pm_state_e state)
{
  struct stm32_dev_s *priv =
      container_of(cb, struct stm32_dev_s, pm_callback);
  uint32_t regval;

  regval = adc_getreg(priv, STM32_ADC_CR_OFFSET);
  if ((state >= PM_IDLE) && (regval & ADC_CR_ADSTART))
    {
      return -EBUSY;
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: adc_wdog_enable
 *
 * Description:
 *   Enable the analog watchdog.
 *
 ****************************************************************************/

static void adc_wdog_enable(struct stm32_dev_s *priv)
{
  uint32_t regval;

  /* Initialize the Analog watchdog enable */

  regval  = adc_getreg(priv, STM32_ADC_CFGR_OFFSET);
  regval |= ADC_CFGR_AWD1EN | ADC_CFGR_CONT | ADC_CFGR_OVRMOD;
  adc_putreg(priv, STM32_ADC_CFGR_OFFSET, regval);

  /* Switch to analog watchdog interrupt */

  regval = adc_getreg(priv, STM32_ADC_IER_OFFSET);
  regval |= ADC_INT_AWD1;
  regval &= ~ADC_INT_EOC;
  adc_putreg(priv, STM32_ADC_IER_OFFSET, regval);
}

/****************************************************************************
 * Name: adc_reg_startconv
 *
 * Description:
 *   Start (or stop) the ADC conversion process
 *
 * Input Parameters:
 *   priv - A reference to the ADC block status
 *   enable - True: Start conversion
 *
 * Returned Value:
 *
 ****************************************************************************/

static void adc_reg_startconv(struct stm32_dev_s *priv, bool enable)
{
  uint32_t regval;

  ainfo("enable: %d\n", enable ? 1 : 0);

  regval = adc_getreg(priv, STM32_ADC_CR_OFFSET);
  if (enable)
    {
      /* Start conversion of regular channels */

      regval |= ADC_CR_ADSTART;
    }
  else
    {
      /* Disable the conversion of regular channels */

      regval |= ADC_CR_ADSTP;
    }

  adc_putreg(priv, STM32_ADC_CR_OFFSET, regval);
}

#ifdef ADC_HAVE_INJECTED

/****************************************************************************
 * Name: adc_inj_startconv
 *
 * Description:
 *   Start (or stop) the ADC injected conversion process
 *
 * Input Parameters:
 *   priv - A reference to the ADC block status
 *   enable - True: Start conversion
 *
 * Returned Value:
 *
 ****************************************************************************/

static void adc_inj_startconv(struct stm32_dev_s *priv, bool enable)
{
  ainfo("inj enable: %d\n", enable ? 1 : 0);
  if (enable)
    {
      /* Start the conversion of injected channels */

      adc_modifyreg(priv, STM32_ADC_CR_OFFSET, 0, ADC_CR_JADSTART);
    }
  else
    {
      /* Stop the conversion */

      adc_modifyreg(priv, STM32_ADC_CR_OFFSET, 0, ADC_CR_JADSTP);
    }
}

#endif /* ADC_HAVE_INJECTED */

/****************************************************************************
 * Name: adc_rccreset
 *
 * Description:
 *   Deinitializes the ADCx peripheral registers to their default
 *   reset values. It could set all the ADCs configured.
 *
 * Input Parameters:
 *   priv - A reference to the ADC block status
 *   reset - Condition, set or reset
 *
 * Returned Value:
 *
 ****************************************************************************/

static void adc_rccreset(struct stm32_dev_s *priv, bool reset)
{
  irqstate_t flags;
  uint32_t regval;
  uint32_t reg = STM32_RCC_AHB4RSTR;
  uint32_t bit = RCC_AHB4RSTR_ADC3RST;

  /* Pick the appropriate bit in the AHBx reset register. */

  if (priv->intf != 3)
    {
      reg = STM32_RCC_AHB1RSTR;
      bit = RCC_AHB1RSTR_ADC12RST;
    }

  /* First must disable interrupts because the AHB2RSTR register is used by
   * several different drivers.
   */

  flags = enter_critical_section();

  /* Set or clear the selected bit in the AHB2 reset register */

  regval = getreg32(reg);
  if (reset)
    {
      regval |= bit;
    }
  else
    {
      regval &= ~bit;
    }

  putreg32(regval, reg);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: adc_startup
 *
 * Description:
 *   Perform one-time startup operations on the ADC peripheral.
 *
 * Input Parameters:
 *   priv - A reference to the ADC block status
 *
 * Returned Value:
 *
 ****************************************************************************/
static void adc_startup(struct stm32_dev_s *priv) {
    uint32_t regval;

    regval = adc_getreg(priv, STM32_ADC_CR_OFFSET);

    /* Exit deep power down mode and enable voltage regulator */

    regval &= ~ADC_CR_DEEPPWD;
    regval |= ADC_CR_ADVREGEN;
    adc_putreg(priv, STM32_ADC_CR_OFFSET, regval);

    /* Wait for voltage regulator to power up */

    up_udelay(20);

#if STM32_ADC_CLK_FREQ >= 20000000
    regval |= ADC_CR_BOOST;
#endif

    /* Enable ADC calibration. ADCALDIF == 0 so this is only for
     * single-ended conversions, not for differential ones.
     */

    regval |= ADC_CR_ADCAL;
    adc_putreg(priv, STM32_ADC_CR_OFFSET, regval);

    /* Wait for calibration to complete */

    while (adc_getreg(priv, STM32_ADC_CR_OFFSET) & ADC_CR_ADCAL);

}

/****************************************************************************
 * Name: adc_enable
 *
 * Description:
 *   Enables the specified ADC peripheral.
 *
 * Input Parameters:
 *   priv - A reference to the ADC block status
 *   enable - true: enable ADC conversion
 *            false: disable ADC conversion
 *
 * Returned Value:
 *
 ****************************************************************************/

static void adc_enable(struct stm32_dev_s *priv, bool enable)
{
  uint32_t regval;

  /* Enable ADC
   * Note: ADEN bit cannot be set during ADCAL=1 and 4 ADC clock cycle
   * after the ADCAL bit is cleared by hardware. If we are using SYSCLK
   * as ADC clock source, this is the same as time taken to execute 4
   * ARM instructions.
   */
  if (enable)
    {
      regval  = adc_getreg(priv, STM32_ADC_CR_OFFSET);
      regval |= ADC_CR_ADEN;
      adc_putreg(priv, STM32_ADC_CR_OFFSET, regval);
      /* Wait for hardware to be ready for conversions */

      while (!(adc_getreg(priv, STM32_ADC_ISR_OFFSET) & ADC_INT_ADRDY));
    } else {
      regval  = adc_getreg(priv, STM32_ADC_CR_OFFSET);
      regval &= ~ADC_CR_ADEN;
      adc_putreg(priv, STM32_ADC_CR_OFFSET, regval);
      /* Wait for hardware to be unready for conversions */

      while ((adc_getreg(priv, STM32_ADC_ISR_OFFSET) & ADC_INT_ADRDY));
    }

}

/****************************************************************************
 * Name: adc_bind
 *
 * Description:
 *   Bind the upper-half driver callbacks to the lower-half implementation.
 *   This must be called early in order to receive ADC event notifications.
 *
 ****************************************************************************/

static int adc_bind(struct adc_dev_s *dev,
                    const struct adc_callback_s *callback)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;

  DEBUGASSERT(priv != NULL);
  priv->cb = callback;
  return OK;
}

/****************************************************************************
 * Name: adc_reset
 *
 * Description:
 *   Reset the ADC device.  Called early to initialize the hardware. This
 *   is called, before adc_setup() and on error conditions.
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static void adc_reset(struct adc_dev_s *dev)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;

  ainfo("intf: ADC%d\n", priv->intf);

  /* Enable ADC reset state */

  adc_rccreset(priv, true);

  /* Release ADC from reset state */

  adc_rccreset(priv, false);
}

/****************************************************************************
 * Name: adc_setup
 *
 * Description:
 *   Configure the ADC. This method is called the first time that the ADC
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching ADC interrupts.
 *   Interrupts are all disabled upon return.
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static int adc_setup(struct adc_dev_s *dev)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;
  int ret;
  int i;
  irqstate_t flags;
  uint32_t clrbits;
  uint32_t setbits;

  /* Attach the ADC interrupt */

  ret = irq_attach(priv->irq, priv->isr, NULL);
  if (ret < 0)
    {
      ainfo("irq_attach failed: %d\n", ret);
      return ret;
    }

  flags = enter_critical_section();

  /* Make sure that the ADC device is in the powered up, reset state.
   * Since reset is shared between ADC1 and ADC2, don't reset one if the
   * other has already been reset. (We only need to worry about this if both
   * ADC1 and ADC2 are enabled.)
   */

#if defined(CONFIG_STM32H7_ADC1) && defined(CONFIG_STM32H7_ADC2)
  if ((dev == &g_adcdev1 &&
      !((struct stm32_dev_s *)g_adcdev2.ad_priv)->initialized) ||
     (dev == &g_adcdev2 &&
      !((struct stm32_dev_s *)g_adcdev1.ad_priv)->initialized))
#endif
    {
      adc_reset(dev);
    }

  /* Initialize the same sample time for each ADC.
   * During sample cycles channel selection bits must remain unchanged.
   */

  adc_putreg(priv, STM32_ADC_SMPR1_OFFSET, ADC_SMPR1_DEFAULT);
  adc_putreg(priv, STM32_ADC_SMPR2_OFFSET, ADC_SMPR2_DEFAULT);

  /* Set the resolution of the conversion. */

  clrbits = ADC_CFGR_RES_MASK | ADC_CFGR_DMNGT_MASK;
  setbits = ADC_CFGR_RES_16BIT;

#ifdef ADC_HAVE_DMA
  if (priv->hasdma)
    {
      /* Enable One shot DMA */

      setbits |= ADC_CFGR_DMNGT_DMA_ONESHOT;
    }
#endif

#ifdef ADC_HAVE_DFSDM
  if (priv->hasdfsdm)
    {
      /* Enable routing to DFSDM */

      setbits |= ADC_CFGR_DMNGT_DFSDM;
    }
#endif

  /* Disable continuous mode */

  clrbits |= ADC_CFGR_CONT;

  /* Disable external trigger for regular channels */

  clrbits |= ADC_CFGR_EXTEN_MASK;
  setbits |= ADC_CFGR_EXTEN_NONE;

  /* Set overrun mode to preserve the data register */

  clrbits |= ADC_CFGR_OVRMOD;

  /* Set CFGR configuration */

  adc_modifyreg(priv, STM32_ADC_CFGR_OFFSET, clrbits, setbits);

  /* Set CFGR2 configuration to align right no oversample */

  clrbits = ADC_CFGR2_ROVSE | ADC_CFGR2_JOVSE | ADC_CFGR2_OVSS_MASK \
          | ADC_CFGR2_OVSR_MASK | ADC_CFGR2_LSHIFT_MASK;
  setbits = 0;

  adc_modifyreg(priv, STM32_ADC_CFGR2_OFFSET, clrbits, setbits);

  /* Configuration of the channel conversions */

  if (priv->cr_channels > 0)
    {
      adc_set_ch(dev, 0);
    }

#ifdef ADC_HAVE_INJECTED
  /* Configuration of the injected channel conversions after adc enabled */

  if (priv->cj_channels > 0)
    {
      adc_inj_set_ch(dev, 0);
    }
#endif


  /* ADC CCR configuration */

  clrbits = ADC_CCR_PRESC_MASK | ADC_CCR_VREFEN |
            ADC_CCR_VSENSEEN | ADC_CCR_VBATEN;
  setbits = ADC_CCR_PRESC_NOT_DIV | ADC_CCR_CKMODE_ASYCH;

  adc_internal(priv, &setbits);

  adc_modifyregm(priv, STM32_ADC_CCR_OFFSET, clrbits, setbits);

#ifdef ADC_HAVE_DMA

  /* Enable DMA */

  if (priv->hasdma)
    {
      /* Stop and free DMA if it was started before */

      if (priv->dma != NULL)
        {
          stm32_dmastop(priv->dma);
          stm32_dmafree(priv->dma);
        }

      priv->dma = stm32_dmachannel(priv->dmachan);

      stm32_dmasetup(priv->dma,
                       priv->base + STM32_ADC_DR_OFFSET,
                       (uint32_t)priv->dmabuffer,
                       priv->rnchannels,
                       ADC_DMA_CONTROL_WORD);

      stm32_dmastart(priv->dma, adc_dmaconvcallback, dev, false);
    }

#endif

  /* Set up the PCSEL bits. */

  setbits = 0;
  clrbits = ADC_PCSEL_PCSEL_ALL;
  for (i = 0; i < priv->cr_channels; i++)
    {
      setbits |= 1 << priv->r_chanlist[i];
    }
#ifdef ADC_HAVE_INJECTED
  for (i = 0; i < priv->cj_channels; i++)
    {
      setbits |= 1 << priv->j_chanlist[i];
    }
#endif

  setbits &= ADC_PCSEL_PCSEL_ALL;

  adc_modifyreg(priv, STM32_ADC_PCSEL_OFFSET, clrbits, setbits);

  /* Configure ADC resolution */

  adc_resolution_set(dev, priv->resolution);

#ifdef ADC_HAVE_EXTCFG
  /* Configure external event for regular group */

  adc_extcfg_set(priv, priv->extcfg);
#endif

#ifdef ADC_HAVE_JEXTCFG
  /* Configure external event for injected group when ADC enabled */

  adc_jextcfg_set(priv, priv->jextcfg);
#endif

  /* Set ADEN to wake up the ADC from Power Down. */
  adc_startup(priv);
  adc_enable(priv, true);

#ifdef ADC_HAVE_TIMER
  if (priv->tbase != 0)
    {
      ret = adc_timinit(priv);
      if (ret < 0)
        {
          aerr("ERROR: adc_timinit failed: %d\n", ret);
        }

      adc_reg_startconv(priv, ret < 0 ? false : true);
    }
#endif

  leave_critical_section(flags);

  ainfo("ISR:   0x%08" PRIx32 " CR:    0x%08" PRIx32 " "
        "CFGR:  0x%08" PRIx32 " CFGR2: 0x%08" PRIx32 "\n",
        adc_getreg(priv, STM32_ADC_ISR_OFFSET),
        adc_getreg(priv, STM32_ADC_CR_OFFSET),
        adc_getreg(priv, STM32_ADC_CFGR_OFFSET),
        adc_getreg(priv, STM32_ADC_CFGR2_OFFSET));
  ainfo("SQR1:  0x%08" PRIx32 " SQR2:  0x%08" PRIx32 " "
        "SQR3:  0x%08" PRIx32 " SQR4:  0x%08" PRIx32 "\n",
        adc_getreg(priv, STM32_ADC_SQR1_OFFSET),
        adc_getreg(priv, STM32_ADC_SQR2_OFFSET),
        adc_getreg(priv, STM32_ADC_SQR3_OFFSET),
        adc_getreg(priv, STM32_ADC_SQR4_OFFSET));
  ainfo("CCR:   0x%08" PRIx32 "\n", adc_getregm(priv, STM32_ADC_CCR_OFFSET));

  /* Enable the ADC interrupt */

  ainfo("Enable the ADC interrupt: irq=%d\n", priv->irq);
  up_enable_irq(priv->irq);

  priv->initialized = true;

  return ret;
}

/****************************************************************************
 * Name: adc_shutdown
 *
 * Description:
 *   Disable the ADC.  This method is called when the ADC device is closed.
 *   This method reverses the operation the setup method.
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 *
 ****************************************************************************/

static void adc_shutdown(struct adc_dev_s *dev)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;

  /* Stop the ADC */

  adc_reg_startconv(priv, false);

  /* Disable ADC interrupts and detach the ADC interrupt handler */

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);

  /* Disable and reset the ADC module */

  adc_reset(dev);

  priv->initialized = false;
}

/****************************************************************************
 * Name: adc_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts.
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static void adc_rxint(struct adc_dev_s *dev, bool enable)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;
  uint32_t regval;

  ainfo("intf: %d enable: %d\n", priv->intf, enable ? 1 : 0);

  regval = adc_getreg(priv, STM32_ADC_IER_OFFSET);
  if (enable)
    {
      /* Enable end of conversion and overrun interrupts */

      regval |= ADC_INT_EOC | ADC_INT_JEOC | ADC_INT_OVR;
    }
  else
    {
      /* Disable all interrupts */

      regval &= ~ADC_INT_MASK;
    }

  adc_putreg(priv, STM32_ADC_IER_OFFSET, regval);
}

/****************************************************************************
 * Name: adc_resolution_set
 ****************************************************************************/

static int adc_resolution_set(struct adc_dev_s *dev, uint8_t res)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;
  int ret = OK;

  /* Check input */

  if (res > 3)
    {
      ret = -EINVAL;
      goto errout;
    }

  /* Modify appropriate register */

  adc_modifyreg(priv, STM32_ADC_CFGR_OFFSET, ADC_CFGR_RES_MASK,
                res << ADC_CFGR_RES_SHIFT);

errout:
  return ret;
}

/****************************************************************************
 * Name: adc_extcfg_set
 ****************************************************************************/

#ifdef ADC_HAVE_EXTCFG
static int adc_extcfg_set(struct stm32_dev_s *priv, uint32_t extcfg)
{
  uint32_t exten  = 0;
  uint32_t extsel = 0;
  uint32_t setbits = 0;
  uint32_t clrbits = 0;

  /* Get EXTEN and EXTSEL from input */

  exten = extcfg & ADC_EXTREG_EXTEN_MASK;
  extsel = extcfg & ADC_EXTREG_EXTSEL_MASK;

  /* EXTSEL selection: These bits select the external event used
   * to trigger the start of conversion of a regular group.  NOTE:
   *
   * - The position with of the EXTSEL field varies from one STM32 MCU
   *   to another.
   * - The width of the EXTSEL field varies from one STM32 MCU to another.
   */

  if (exten > 0)
    {
      setbits = extsel | exten;
      clrbits = ADC_EXTREG_EXTEN_MASK | ADC_EXTREG_EXTSEL_MASK;

      ainfo("Initializing extsel = 0x%08" PRIx32 "\n", extsel);

      /* Write register */

      adc_modifyreg(priv, STM32_ADC_EXTREG_OFFSET, clrbits, setbits);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: adc_jextcfg_set
 ****************************************************************************/

#ifdef ADC_HAVE_JEXTCFG
static int adc_jextcfg_set(struct stm32_dev_s *priv, uint32_t jextcfg)
{
  uint32_t jexten =  0;
  uint32_t jextsel = 0;
  uint32_t setbits = 0;
  uint32_t clrbits = 0;

  /* Get JEXTEN and JEXTSEL from input */

  jexten = jextcfg & ADC_JSQR_JEXTEN_MASK;
  jextsel = jextcfg & ADC_JSQR_JEXTSEL_MASK;

  /* JEXTSEL selection: These bits select the external event used
   * to trigger the start of conversion of a injected group.  NOTE:
   *
   * - The position with of the JEXTSEL field varies from one STM32 MCU
   *   to another.
   * - The width of the JEXTSEL field varies from one STM32 MCU to another.
   */

  if (jexten > 0)
    {
      setbits = jexten | jextsel;
      clrbits = ADC_JSQR_JEXTEN_MASK | ADC_JSQR_JEXTSEL_MASK;

      ainfo("Initializing jextsel = 0x%08" PRIx32 "\n", jextsel);

      /* Write register */

      adc_modifyreg(priv, STM32_ADC_JSQR_OFFSET, clrbits, setbits);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: adc_sqrbits
 ****************************************************************************/

static uint32_t adc_sqrbits(struct stm32_dev_s *priv, int first,
                            int last, int offset)
{
  uint32_t bits = 0;
  int i;

  for (i = first - 1;
       i < priv->rnchannels && i < last;
       i++, offset += ADC_SQ_OFFSET)
    {
      bits |= (uint32_t)priv->r_chanlist[i] << offset;
    }

  return bits;
}

/****************************************************************************
 * Name: adc_internal
 ****************************************************************************/

static bool adc_internal(struct stm32_dev_s * priv, uint32_t *adc_ccr)
{
  int i;
  bool internal = false;

  if (priv->intf == 3)
    {
      for (i = 0; i < priv->cr_channels; i++)
        {
          if (priv->r_chanlist[i] > ADC_EXTERNAL_CHAN_MAX)
            {
              internal = true;
              switch (priv->r_chanlist[i])
                {
                  case 17:
                    *adc_ccr |= ADC_CCR_VBATEN;
                    break;

                  case 18:
                    *adc_ccr |= ADC_CCR_VSENSEEN;
                    break;

                  case 19:
                     *adc_ccr |= ADC_CCR_VREFEN;
                    break;
                }
            }
        }
      /* Note: We should also probably do this for the injected channels */
    }

  return internal;
}

/****************************************************************************
 * Name: adc_set_offset
 ****************************************************************************/

#ifdef ADC_HAVE_DFSDM
static int adc_setoffset(struct stm32_dev_s *priv, uint8_t ch, uint8_t i,
                         uint16_t offset)
{
  uint32_t reg;
  uint32_t regval;

  if (i >= 4)
    {
      /* There are only four offset registers. */

      return -E2BIG;
    }

  reg = STM32_ADC_OFR1_OFFSET + i * 4;

  regval = ADC_OFR_OFFSETY_EN;
  adc_putreg(priv, reg, regval);

  regval |= ADC_OFR_OFFSETY_CH(ch) | ADC_OFR_OFFSETY(offset);
  adc_putreg(priv, reg, regval);
  return OK;
}
#endif

/****************************************************************************
 * Name: adc_set_ch
 *
 * Description:
 *   Sets the ADC channel.
 *
 * Input Parameters:
 *   dev - pointer to device structure used by the driver
 *   ch  - ADC channel number + 1. 0 reserved for all configured channels
 *
 * Returned Value:
 *   int - errno
 *
 ****************************************************************************/

static int adc_set_ch(struct adc_dev_s *dev, uint8_t ch)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;
  uint32_t bits;
  int i;

  if (ch == 0)
    {
      priv->current   = 0;
      priv->rnchannels = priv->cr_channels;
    }
  else
    {
      for (i = 0; i < priv->cr_channels && priv->r_chanlist[i] != ch - 1; i++);

      if (i >= priv->cr_channels)
        {
          return -ENODEV;
        }

      priv->current   = i;
      priv->rnchannels = 1;
    }

  DEBUGASSERT(priv->rnchannels <= ADC_MAX_SAMPLES);

  bits = adc_sqrbits(priv, ADC_SQR4_FIRST, ADC_SQR4_LAST,
                     ADC_SQR4_SQ_OFFSET);
  adc_modifyreg(priv, STM32_ADC_SQR4_OFFSET, ~ADC_SQR4_RESERVED, bits);

  bits = adc_sqrbits(priv, ADC_SQR3_FIRST, ADC_SQR3_LAST,
                     ADC_SQR3_SQ_OFFSET);
  adc_modifyreg(priv, STM32_ADC_SQR3_OFFSET, ~ADC_SQR3_RESERVED, bits);

  bits = adc_sqrbits(priv, ADC_SQR2_FIRST, ADC_SQR2_LAST,
                     ADC_SQR2_SQ_OFFSET);
  adc_modifyreg(priv, STM32_ADC_SQR2_OFFSET, ~ADC_SQR2_RESERVED, bits);

  bits = ((uint32_t)priv->rnchannels - 1) << ADC_SQR1_L_SHIFT |
         adc_sqrbits(priv, ADC_SQR1_FIRST, ADC_SQR1_LAST,
                     ADC_SQR1_SQ_OFFSET);
  adc_modifyreg(priv, STM32_ADC_SQR1_OFFSET, ~ADC_SQR1_RESERVED, bits);

#ifdef ADC_HAVE_DFSDM
  if (priv->hasdfsdm)
    {
      /* Convert 12-bit ADC result to signed 16-bit. */

      if (ch == 0)
        {
          for (i = 0; i < priv->cr_channels; i++)
            {
              adc_setoffset(priv, priv->r_chanlist[i], i, 0x800);
            }
        }
      else
        {
          adc_setoffset(priv, priv->current, 0, 0x800);
        }
    }
#endif

  return OK;
}

#ifdef ADC_HAVE_INJECTED

/****************************************************************************
 * Name: adc_inj_set_ch
 ****************************************************************************/

static int adc_inj_set_ch(struct adc_dev_s *dev, uint8_t ch)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;
  uint32_t clrbits;
  uint32_t setbits;
  int i;

  /* Configure injected sequence length */

  setbits = ADC_JSQR_JL(priv->cj_channels);
  clrbits = ADC_JSQR_JEXTSEL_MASK | ADC_JSQR_JL_MASK | ADC_JSQR_JSQ1_MASK | ADC_JSQR_JSQ2_MASK | ADC_JSQR_JSQ3_MASK | ADC_JSQR_JSQ4_MASK;

  /* Configure injected channels */

  for (i = 0 ; i < priv->cj_channels; i += 1)
    {
      /* Injected channels sequence:
       *
       *           1      2     3      4
       *   JL=1: JSQR1,
       *   JL=2: JSQR1, JSQR2
       *   JL=3: JSQR1, JSQR2, JSQR3
       *   JL=4: JSQR1, JSQR2, JSQR3, JSQR4
       */

      setbits |= ((priv->j_chanlist[i] & ADC_JSQR_JSQ_MASK) <<
                  (ADC_JSQR_JSQ1_SHIFT + (ADC_JSQR_JSQ_SHIFT * i)));
      ainfo("Inj set channel %d, 0x%08" PRIx32 "\n", i, setbits);
    }

  /* Write register */

  adc_modifyreg(priv, STM32_ADC_JSQR_OFFSET, clrbits, setbits);

  return OK;
}
#endif

/****************************************************************************
 * Name: adc_ioctl
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

static int adc_ioctl(struct adc_dev_s *dev, int cmd, unsigned long arg)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;
  uint32_t regval;
  uint32_t tmp;
  int ret = OK;

  switch (cmd)
    {
      case ANIOC_TRIGGER:
        {
          /* Start regular conversion if regular channels configured */

          if (priv->anioc_trg & ANIOC_TRIGGER_REGULAR)
            {
              if (priv->cr_channels > 0)
                {
                  adc_reg_startconv(priv, true);
                }
            }

#ifdef ADC_HAVE_INJECTED
          /* Start injected conversion if injected channels configured */

          if (priv->anioc_trg & ANIOC_TRIGGER_INJECTED)
            {
              if (priv->cj_channels > 0)
                {
                  adc_inj_startconv(priv, true);
                }
            }
#endif

        }
        break;

      case ANIOC_GET_NCHANNELS:
        {
          /* Return the number of configured channels */

          ret = priv->rnchannels;
        }
        break;

      case ANIOC_WDOG_UPPER: /* Set watchdog upper threshold */
        {
          regval = adc_getreg(priv, STM32_ADC_LTR1_OFFSET);

          /* Verify new upper threshold greater than lower threshold */

          tmp = (regval & ADC_LTR1_LT_MASK) >> ADC_LTR1_LT_SHIFT;
          if (arg < tmp)
            {
              ret = -EINVAL;
              break;
            }

          /* Set the watchdog threshold register */

          regval = ((arg << ADC_HTR1_HT_SHIFT) & ADC_HTR1_HT_MASK);
          adc_putreg(priv, STM32_ADC_HTR1_OFFSET, regval);

          /* Ensure analog watchdog is enabled */

          adc_wdog_enable(priv);
        }
        break;

      case ANIOC_WDOG_LOWER: /* Set watchdog lower threshold */
        {
          regval = adc_getreg(priv, STM32_ADC_HTR1_OFFSET);

          /* Verify new lower threshold less than upper threshold */

          tmp = (regval & ADC_HTR1_HT_MASK) >> ADC_HTR1_HT_SHIFT;
          if (arg > tmp)
            {
              ret = -EINVAL;
              break;
            }

          /* Set the watchdog threshold register */

          regval = ((arg << ADC_LTR1_LT_SHIFT) & ADC_LTR1_LT_MASK);
          adc_putreg(priv, STM32_ADC_LTR1_OFFSET, regval);

          /* Ensure analog watchdog is enabled */

          adc_wdog_enable(priv);
        }
        break;

      default:
        aerr("ERROR: Unknown cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: adc_interrupt
 *
 * Description:
 *   Common ADC interrupt handler.
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static int adc_interrupt(struct adc_dev_s *dev, uint32_t adcisr)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;
  int32_t value;
  /* Identifies the AWD interrupt */

  if ((adcisr & ADC_INT_AWD1) != 0)
    {
      value  = adc_getreg(priv, STM32_ADC_DR_OFFSET);
      value &= ADC_DR_MASK;

      awarn("WARNING: Analog Watchdog, Value (0x%03" PRIx32 ") "
            "out of range!\n", value);

      /* Stop ADC conversions to avoid continuous interrupts */

      adc_reg_startconv(priv, false);
    }

  /* OVR: Overrun */

  if ((adcisr & ADC_INT_OVR) != 0)
    {
      /* In case of a missed ISR - due to interrupt saturation -
       * the upper half needs to be informed to terminate properly.
       */

      awarn("WARNING: Overrun has occurred!\n");

      /* To make use of already sampled data the conversion needs to be
       * stopped first before reading out the data register.
       */

      adc_reg_startconv(priv, false);

      while ((adc_getreg(priv, STM32_ADC_CR_OFFSET) & ADC_CR_ADSTART) != 0);

      /* Verify that the upper-half driver has bound its callback functions */

      if ((priv->cb != NULL) && (priv->cb->au_reset != NULL))
        {
          /* Notify upper-half driver about the overrun */

          priv->cb->au_reset(dev);
        }

      adc_putreg(priv, STM32_ADC_ISR_OFFSET, ADC_INT_OVR);
    }

  /* EOC: End of conversion */

  if ((adcisr & ADC_INT_EOC) != 0)
    {
      /* Read from the ADC_DR register until 8 stage FIFO is empty.
       * The FIFO is first mentioned in STM32H7 Reference Manual
       * rev. 7, though, not yet indicated in the block diagram!
       */

      do
        {
          /* Read the converted value and clear EOC bit
           * (It is cleared by reading the ADC_DR)
           */

          value  = adc_getreg(priv, STM32_ADC_DR_OFFSET);
          value &= ADC_DR_MASK;

          /* Verify that the upper-half driver has bound its
           * callback functions
           */

          if (priv->cb != NULL)
            {
              /* Hand the ADC data to the ADC driver.  The ADC receive()
               * method accepts 3 parameters:
               *
               * 1) The first is the ADC device instance for this ADC block.
               * 2) The second is the channel number for the data, and
               * 3) The third is the converted data for the channel.
               */

              DEBUGASSERT(priv->cb->au_receive != NULL);
              priv->cb->au_receive(dev, priv->r_chanlist[priv->current],
                                   value);
            }

          /* Set the channel number of the next channel that will
           * complete conversion
           */

          priv->current++;

          if (priv->current >= priv->rnchannels)
            {
              /* Restart the conversion sequence from the beginning */

              priv->current = 0;
            }
        }
      while ((adc_getreg(priv, STM32_ADC_ISR_OFFSET) & ADC_INT_EOC) != 0);
    }
  if (adcisr & (~(ADC_INT_AWD1 | ADC_INT_OVR | ADC_INT_EOC | ADC_INT_EOS | ADC_INT_LDORDY | ADC_INT_EOSMP | ADC_INT_ADRDY))) {
  }
  return OK;
}

/****************************************************************************
 * Name: adc12_interrupt
 *
 * Description:
 *   ADC1/2 interrupt handler
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

#if defined(CONFIG_STM32H7_ADC1) || defined(CONFIG_STM32H7_ADC2)
static int adc12_interrupt(int irq, void *context, void *arg)
{
  uint32_t regval1;
  uint32_t regval2;
  uint32_t pending1;
  uint32_t pending2;

#ifdef CONFIG_STM32H7_ADC1
  regval1  = getreg32(STM32_ADC1_ISR);
  pending1 = regval1 & (ADC_INT_EOC | ADC_INT_JEOC | ADC_INT_OVR | ADC_INT_AWD1);
  if (pending1 != 0)
    {
      adc_interrupt(&g_adcdev1, regval1);

      /* Clear interrupts */

      putreg32(regval1, STM32_ADC1_ISR);
    }
#endif

#ifdef CONFIG_STM32H7_ADC2
  regval2  = getreg32(STM32_ADC2_ISR);
  pending2 = regval2 & (ADC_INT_EOC | ADC_INT_JEOC | ADC_INT_OVR | ADC_INT_AWD1);
  if (pending2 != 0)
    {
      adc_interrupt(&g_adcdev2, regval2);

      /* Clear interrupts */

      putreg32(regval2, STM32_ADC2_ISR);
    }
#endif

  return OK;
}
#endif

/****************************************************************************
 * Name: adc3_interrupt
 *
 * Description:
 *   ADC3 interrupt handler
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

#ifdef CONFIG_STM32H7_ADC3
static int adc3_interrupt(int irq, void *context, void *arg)
{
  uint32_t regval;
  uint32_t pending;
  uint32_t ier;
  regval  = getreg32(STM32_ADC3_ISR);
  pending = regval & ADC_INT_MASK;
  ier = getreg32(STM32_ADC3_IER);
  if (pending != 0)
    {
      adc_interrupt(&g_adcdev3, regval);

      /* Clear interrupts */

      putreg32(regval, STM32_ADC3_ISR);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: adc_dmaconvcallback
 *
 * Description:
 *   Callback for DMA.  Called from the DMA transfer complete interrupt after
 *   all channels have been converted and transferred with DMA.
 *
 * Input Parameters:
 *
 *   handle - handle to DMA
 *   isr -
 *   arg - adc device
 *
 * Returned Value:
 *
 ****************************************************************************/

#ifdef ADC_HAVE_DMA
static void adc_dmaconvcallback(DMA_HANDLE handle, uint8_t isr,
                                void *arg)
{
  struct adc_dev_s   *dev  = (struct adc_dev_s *)arg;
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;
  int i;

  /* Verify that the upper-half driver has bound its callback functions */

  if (priv->cb != NULL)
    {
      DEBUGASSERT(priv->cb->au_receive != NULL);

      for (i = 0; i < priv->rnchannels; i++)
        {
          priv->cb->au_receive(dev, priv->r_chanlist[priv->current],
                               priv->dmabuffer[priv->current]);
          priv->current++;
          if (priv->current >= priv->rnchannels)
            {
              /* Restart the conversion sequence from the beginning */

              priv->current = 0;
            }
        }
    }

  /* Restart DMA for the next conversion series */

  adc_modifyreg(priv, STM32_ADC_CFGR_OFFSET, ADC_CFGR_DMAEN, 0);
  adc_modifyreg(priv, STM32_ADC_CFGR_OFFSET, 0, ADC_CFGR_DMAEN);
}
#endif


#ifdef CONFIG_STM32H7_ADC_LL_OPS

/****************************************************************************
 * Name: adc_llops_setup
 ****************************************************************************/

static int adc_llops_setup(struct stm32_adc_dev_s *dev)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;

  return adc_setup(priv->dev);
}

/****************************************************************************
 * Name: adc_llops_shutdown
 ****************************************************************************/

static void adc_llops_shutdown(struct stm32_adc_dev_s *dev)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;

  adc_shutdown(priv->dev);
}

/****************************************************************************
 * Name: adc_intack
 ****************************************************************************/

static void adc_intack(struct stm32_adc_dev_s *dev, uint32_t source)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;

  /* Cleared by writing 1 to it */

  adc_modifyreg(priv, STM32_ADC_ISR_OFFSET, 0, (source & ADC_ISR_ALLINTS));
}

/****************************************************************************
 * Name: adc_inten
 ****************************************************************************/

static void adc_inten(struct stm32_adc_dev_s *dev, uint32_t source)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;

  /* Enable interrupts */

  adc_modifyreg(priv, STM32_ADC_IER_OFFSET, 0, (source & ADC_IER_ALLINTS));
}

/****************************************************************************
 * Name: adc_intdis
 ****************************************************************************/

static void adc_intdis(struct stm32_adc_dev_s *dev, uint32_t source)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;

  /* Disable interrupts */

  adc_modifyreg(priv, STM32_ADC_IER_OFFSET, (source & ADC_IER_ALLINTS), 0);
}

/****************************************************************************
 * Name: adc_ackget
 ****************************************************************************/

static uint32_t adc_intget(struct stm32_adc_dev_s *dev)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;
  uint32_t regval;
  uint32_t pending;

  regval  = adc_getreg(priv, STM32_ADC_ISR_OFFSET);
  pending = regval & ADC_ISR_ALLINTS;

  return pending;
}

/****************************************************************************
 * Name: adc_regget
 ****************************************************************************/

static uint32_t adc_regget(struct stm32_adc_dev_s *dev)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;

  return adc_getreg(priv, STM32_ADC_DR_OFFSET) & ADC_DR_MASK;
}

/****************************************************************************
 * Name: adc_llops_reg_startconv
 ****************************************************************************/

static void adc_llops_reg_startconv(struct stm32_adc_dev_s *dev,
                                    bool enable)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;

  adc_reg_startconv(priv, enable);
}

/****************************************************************************
 * Name: adc_offset_set
 ****************************************************************************/

static int adc_offset_set(struct stm32_adc_dev_s *dev, uint8_t ch,
                          uint8_t i, uint32_t offset)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;
  uint32_t reg = 0;
  int      ret = OK;

  UNUSED(ch);

  if (i >= 4)
    {
      /* There are only four offset registers. */

      ret = -E2BIG;
      goto errout;
    }

  reg = STM32_ADC_OFR1_OFFSET + i * 4;

  ainfo("ADC offset ch = %d, i = %d, off = %"PRIu32"\n", ch, i, offset);

  adc_putreg(priv, reg, (offset & ADC_OFR_OFFSETY_MASK) | (ADC_OFR_OFFSETY_CH(ch) & ADC_OFR_OFFSETY_CH_MASK));

errout:
  return ret;
}

/****************************************************************************
 * Name: adc_llops_extcfg_set
 ****************************************************************************/

#ifdef ADC_HAVE_EXTCFG
static void adc_llops_extcfg_set(struct stm32_adc_dev_s *dev,
                                 uint32_t extcfg)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;

  adc_extcfg_set(priv, extcfg);
}
#endif

/****************************************************************************
 * Name: adc_llops_jextcfg_set
 ****************************************************************************/

#ifdef ADC_HAVE_JEXTCFG
static void  adc_llops_jextcfg_set(struct stm32_adc_dev_s *dev,
                                   uint32_t jextcfg)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;

  adc_jextcfg_set(priv, jextcfg);
}
#endif

/****************************************************************************
 * Name: adc_dumpregs
 ****************************************************************************/

static void adc_dumpregs(struct stm32_dev_s *priv)
{
  UNUSED(priv);

  ainfo("ISR:   0x%08" PRIx32 " CR:  0x%08" PRIx32
        " CFGR:  0x%08" PRIx32 " CFGR2:  0x%08" PRIx32 "\n",
        adc_getreg(priv, STM32_ADC_ISR_OFFSET),
        adc_getreg(priv, STM32_ADC_CR_OFFSET),
        adc_getreg(priv, STM32_ADC_CFGR_OFFSET),
        adc_getreg(priv, STM32_ADC_CFGR2_OFFSET));

  ainfo("SQR1: 0x%08" PRIx32 " SQR2: 0x%08" PRIx32
        " SQR3: 0x%08" PRIx32 " SQR4: 0x%08" PRIx32 "\n",
        adc_getreg(priv, STM32_ADC_SQR1_OFFSET),
        adc_getreg(priv, STM32_ADC_SQR2_OFFSET),
        adc_getreg(priv, STM32_ADC_SQR3_OFFSET),
        adc_getreg(priv, STM32_ADC_SQR4_OFFSET));

  ainfo("OFR1: 0x%08" PRIx32 " OFR2: 0x%08" PRIx32
        " OFR3: 0x%08" PRIx32 " OFR4: 0x%08" PRIx32 "\n",
        adc_getreg(priv, STM32_ADC_OFR1_OFFSET),
        adc_getreg(priv, STM32_ADC_OFR2_OFFSET),
        adc_getreg(priv, STM32_ADC_OFR3_OFFSET),
        adc_getreg(priv, STM32_ADC_OFR4_OFFSET));

  ainfo("SMPR1: 0x%08" PRIx32 " SMPR2: 0x%08" PRIx32 "\n",
        adc_getreg(priv, STM32_ADC_SMPR1_OFFSET),
        adc_getreg(priv, STM32_ADC_SMPR2_OFFSET));

#ifdef ADC_HAVE_INJECTED
  ainfo("JSQR: 0x%08" PRIx32 "\n", adc_getreg(priv, STM32_ADC_JSQR_OFFSET));
#endif

  ainfo("CCR:  0x%08" PRIx32 "\n",
        adc_getreg(priv, STM32_ADC_CCR_OFFSET));
}

/****************************************************************************
 * Name: adc_regbufregister
 ****************************************************************************/

#ifdef ADC_HAVE_DMA
static int adc_regbufregister(struct stm32_adc_dev_s *dev,
                              uint16_t *buffer, uint8_t len)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;

  stm32_dmasetup(priv->dma,
                 priv->base + STM32_ADC_DR_OFFSET,
                 (uint32_t)buffer,
                 len,
                 ADC_DMA_CONTROL_WORD);

  /* No DMA callback */

  stm32_dmastart(priv->dma, NULL, dev, false);

  return OK;
}
#endif /* ADC_HAVE_DMA */

/****************************************************************************
 * Name: adc_injget
 ****************************************************************************/

#ifdef ADC_HAVE_INJECTED
static uint32_t adc_injget(struct stm32_adc_dev_s *dev, uint8_t chan)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;
  uint32_t regval = 0;

  if (chan > (priv->cj_channels - 1))
    {
      /* REVISIT: return valute with MSB set to indicate error ? */

      goto errout;
    }

  regval = adc_getreg(priv, STM32_ADC_JDR1_OFFSET + 4 * (chan)) &
           ADC_JDR_MASK;

errout:
  return regval;
}

/****************************************************************************
 * Name: adc_llops_inj_startconv
 ****************************************************************************/

static void adc_llops_inj_startconv(struct stm32_adc_dev_s *dev,
                                    bool enable)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;

  adc_inj_startconv(priv, enable);
}

#endif /* ADC_HAVE_INJECTED */

/****************************************************************************
 * Name: adc_sampletime_write
 *
 * Description:
 *   Writes previously defined values into ADC_SMPRx registers.
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

#ifdef CONFIG_STM32H7_ADC_CHANGE_SAMPLETIME
static void adc_sampletime_write(struct stm32_adc_dev_s *dev)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;
  uint32_t value = 0;
  uint8_t i;
  uint8_t shift;

  /* Sampling time individually for each channel */

  for (i = 0, shift = 0; i < priv->adc_channels; i++)
    {
      value |= priv->sample_rate[i] << (shift * 3);
      switch (i)
        {
          case (ADC_CHANNELS_NUMBER - 1):
            {
              adc_putreg(priv, STM32_ADC_SMPR2_OFFSET, value);
              shift = 0;
              value = 0;
              break;
            }

          case 9:
            {
              adc_putreg(priv, STM32_ADC_SMPR1_OFFSET, value);
              shift = 0;
              value = 0;
              break;
            }

          default:
            {
              shift++;
              break;
            }
        }
    }
}

/****************************************************************************
 * Name: adc_sampletime_set
 *
 * Description:
 *   Changes sample times for specified channels. This method
 *   doesn't make any register writing. So, it's only stores the information.
 *   Values provided by user will be written in registers only on the next
 *   ADC peripheral start, as it was told to do in manual. However, before
 *   very first start, user can call this method and override default values
 *   either for every channels or for only some predefined by user channel(s)
 *
 * Input Parameters:
 *   dev          - pointer to the adc device structure
 *   time_samples - pointe to the adc sample time configuration data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void adc_sampletime_set(struct stm32_adc_dev_s *dev,
                        struct adc_sample_time_s *time_samples)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;
  uint8_t ch_index;
  uint8_t i;

  /* Check if user wants to assign the same value for all channels
   * or just wants to change sample time values for certain channels
   */

  if (time_samples->all_same)
    {
      memset(priv->sample_rate, time_samples->all_ch_sample_time,
             ADC_CHANNELS_NUMBER);
    }
  else
    {
      for (i = 0; i < time_samples->channels_nbr; i++)
        {
          ch_index = time_samples->channel[i].channel;
          if (ch_index >= ADC_CHANNELS_NUMBER)
            {
              break;
            }

          priv->sample_rate[ch_index] = time_samples->channel[i].sample_time;
        }
    }
}
#endif /* CONFIG_STM32H7_ADC_CHANGE_SAMPLETIME */

/****************************************************************************
 * Name: adc_llops_dumpregs
 ****************************************************************************/

static void adc_llops_dumpregs(struct stm32_adc_dev_s *dev)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;

  adc_dumpregs(priv);
}

/****************************************************************************
 * Name: adc_llops_multicfg
 *
 * IMPORTANT: this interface is allowed only when the ADCs are disabled!
 *
 ****************************************************************************/

static int adc_llops_multicfg(struct stm32_adc_dev_s *dev, uint8_t mode)
{
  struct stm32_dev_s *priv    = (struct stm32_dev_s *)dev;
  int                 ret     = OK;
  uint32_t            setbits = 0;
  uint32_t            clrbits = 0;

  switch (mode)
    {
      case ADC_MULTIMODE_INDEP:
        setbits = ADC_CCR_DUAL_IND;
        break;

      case ADC_MULTIMODE_RSISM2:
        setbits = ADC_CCR_DUAL_DUAL;
        break;

      case ADC_MULTIMODE_RSATM2:
        setbits = ADC_CCR_DUAL_SIMALT;
        break;

      case ADC_MULTIMODE_ISM2:
        setbits = ADC_CCR_DUAL_INJECTED;
        break;

      case ADC_MULTIMODE_RSM2:
        setbits = ADC_CCR_DUAL_SIM;
        break;

      case ADC_MULTIMODE_IM2:
        setbits = ADC_CCR_DUAL_INTERLEAVE;
        break;

      case ADC_MULTIMODE_ATM2:
        setbits = ADC_CCR_DUAL_ALT;
        break;

      default:
        ret = -EINVAL;
        goto errout;
    }

  clrbits = ADC_CCR_DUAL_MASK;
  adc_modifyreg(priv, STM32_ADC_CCR_OFFSET, clrbits, setbits);

errout:
  return ret;
}

/****************************************************************************
 * Name: adc_llops_enable
 ****************************************************************************/

static void adc_llops_enable(struct stm32_adc_dev_s *dev, bool enable)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;

  adc_enable(priv, enable);

}

#endif /* CONFIG_STM32H7_ADC_LL_OPS */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32h7_adc_initialize
 *
 * Description:
 *   Initialize the ADC.
 *
 *   The logic allow initialize ADC regular and injected channels.
 *
 *   The number of injected channels for given ADC is selected from Kconfig
 *   with CONFIG_STM32H7_ADCx_INJECTED_CHAN definitions
 *
 *   The number of regular channels is obtained from the equation:
 *
 *     cr_channels = channels - cj_channels
 *
 *   where:
 *     cr_channels - regular channels
 *     cj_channels - injected channels
 *     channels    - this function parameter
 *
 *   The chanlist array store both regular channels and injected channels
 *   configuration so that regular channels are the first in order:
 *
 *     # regular channels start from here
 *     chanlist[0]                  -> ADC_SQRx_SQ1
 *     chanlist[1]                  -> ADC_SQRx_SQ2
 *     ...
 *     # injected channels start from here
 *     chanlist[channels - (y - 1)] -> ADC_JSQR_JSQ1
 *     ...
 *     chanlist[channels]           -> ADC_JSQR_ISQy
 *
 *   where:
 *      y = CONFIG_STM32H7_ADCx_INJECTED_CHAN, and y > 0
 *
 *   If CONFIG_STM32H7_ADCx_INJECTED_CHAN = 0, then all channels from
 *   chanlist are regular channels.
 *
 * Input Parameters:
 *   intf      - Could be {1,2,3} for ADC1, ADC2, ADC3
 *   chanlist  - The list of channels (regular + injected)
 *   channels  - Number of channels (regular + injected)
 *
 * Returned Value:
 *   Valid ADC device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct adc_dev_s *stm32h7_adc_initialize(int intf,
                                         const uint8_t *chanlist,
                                         int channels)
{
  struct adc_dev_s   *dev;
  struct stm32_dev_s *priv;
  uint8_t     cr_channels = 0;
  uint8_t     cj_channels = 0;
#ifdef ADC_HAVE_INJECTED
  uint8_t *j_chanlist = NULL;
#endif


  ainfo("intf: %d channels: %d\n", intf, channels);

  switch (intf)
    {
#ifdef CONFIG_STM32H7_ADC1
      case 1:
        ainfo("ADC1 selected\n");
        dev = &g_adcdev1;
        cj_channels = CONFIG_STM32H7_ADC1_INJECTED_CHAN;
        cr_channels = channels - cj_channels;
#  ifdef ADC_HAVE_INJECTED
        if (cj_channels > 0)
          {
            j_chanlist  = (uint8_t *)chanlist + cr_channels;
          }
#  endif
        break;
#endif
#ifdef CONFIG_STM32H7_ADC2
      case 2:
        ainfo("ADC2 selected\n");
        dev = &g_adcdev2;
        cj_channels = CONFIG_STM32H7_ADC2_INJECTED_CHAN;
        cr_channels = channels - cj_channels;
#  ifdef ADC_HAVE_INJECTED
        if (cj_channels > 0)
          {
            j_chanlist  = (uint8_t *)chanlist + cr_channels;
          }
#  endif
        break;
#endif
#ifdef CONFIG_STM32H7_ADC3
      case 3:
        ainfo("ADC3 selected\n");
        dev = &g_adcdev3;
        cj_channels = CONFIG_STM32H7_ADC3_INJECTED_CHAN;
        cr_channels = channels - cj_channels;
#  ifdef ADC_HAVE_INJECTED
        if (cj_channels > 0)
          {
            j_chanlist  = (uint8_t *)chanlist + cr_channels;
          }
#  endif
        break;
#endif
      default:
        aerr("ERROR: No ADC interface defined\n");
        return NULL;
    }

  /* Configure the selected ADC */

  priv = (struct stm32_dev_s *)dev->ad_priv;
  priv->cb = NULL;

  DEBUGASSERT(channels <= ADC_MAX_SAMPLES);
  if (cr_channels > ADC_MAX_SAMPLES)
    {
      cr_channels = ADC_MAX_SAMPLES;
    }

  priv->cr_channels = cr_channels;
  memcpy(priv->r_chanlist, chanlist, cr_channels);

#ifdef ADC_HAVE_INJECTED
  /* Configure injected channels */

  DEBUGASSERT(cj_channels <= ADC_INJ_MAX_SAMPLES);
  if (cj_channels > ADC_INJ_MAX_SAMPLES)
    {
      cj_channels = ADC_INJ_MAX_SAMPLES;
    }

  priv->cj_channels = cj_channels;
  memcpy(priv->j_chanlist, j_chanlist, cj_channels);
#endif

#ifdef CONFIG_PM
  if (pm_register(&priv->pm_callback) != OK)
    {
      aerr("Power management registration failed\n");
      return NULL;
    }
#endif

#ifdef CONFIG_STM32H7_ADC_CHANGE_SAMPLETIME
  /* Assign default values for the sample time table */

  memset(priv->sample_rate, ADC_SMPR_DEFAULT, ADC_CHANNELS_NUMBER);
  priv->adc_channels = ADC_CHANNELS_NUMBER;
#endif

#ifdef CONFIG_STM32H7_ADC_LL_OPS
  /* Store reference to the upper-half ADC device */

  priv->dev = dev;
#endif

#ifdef ADC_HAVE_INJECTED
  ainfo("intf: %d cr_channels: %d, cj_channels: %d\n",
        intf, priv->cr_channels, priv->cj_channels);
#else
  ainfo("intf: %d cr_channels: %d\n", intf, priv->cr_channels);
#endif

  return dev;
}

#endif /* CONFIG_STM32H7_ADC1 || CONFIG_STM32H7_ADC2 || CONFIG_STM32H7_ADC3 */
#endif /* CONFIG_ADC */
