/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/lpc17_40_can.c
 *
 *   Copyright (C) 2011 Li Zhuoyi. All rights reserved.
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Authors:
 *     Li Zhuoyi <lzyy.cn@gmail.com>
 *     Gregory Nutt <gnutt@nuttx.org>
 *   History:
 *     2011-07-12: Initial version (Li Zhuoyi)
 *     2011-08-03: Support CAN1/CAN2 (Li Zhuoyi)
 *     2012-01-02: Add support for CAN loopback mode (Gregory Nutt)
 *
 * This file is a part of NuttX:
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/can/can.h>

#include "arm_internal.h"
#include "chip.h"
#include "hardware/lpc17_40_syscon.h"
#include "lpc17_40_gpio.h"
#include "lpc17_40_can.h"

#if defined(CONFIG_LPC17_40_CAN1) || defined(CONFIG_LPC17_40_CAN2)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifdef CONFIG_LPC17_40_CAN1

/* A CAN bit rate must be provided */

#  ifndef CONFIG_LPC17_40_CAN1_BAUD
#    error "CONFIG_LPC17_40_CAN1_BAUD is not defined"
#  endif

/* If no divsor is provided, use a divisor of 4 */

#  ifndef CONFIG_LPC17_40_CAN1_DIVISOR
#    define CONFIG_LPC17_40_CAN1_DIVISOR 4
#  endif

/* Get the SYSCON_PCLKSEL value for CAN1 the implements this divisor */

#  if CONFIG_LPC17_40_CAN1_DIVISOR == 1
#    define CAN1_CCLK_DIVISOR SYSCON_PCLKSEL_CCLK
#  elif CONFIG_LPC17_40_CAN1_DIVISOR == 2
#    define CAN1_CCLK_DIVISOR SYSCON_PCLKSEL_CCLK2
#  elif CONFIG_LPC17_40_CAN1_DIVISOR == 4
#    define CAN1_CCLK_DIVISOR SYSCON_PCLKSEL_CCLK4
#  elif CONFIG_LPC17_40_CAN1_DIVISOR == 6
#    define CAN1_CCLK_DIVISOR SYSCON_PCLKSEL_CCLK6
#  else
#    error "Unsupported value of CONFIG_LPC17_40_CAN1_DIVISOR"
#  endif
#endif

#ifdef CONFIG_LPC17_40_CAN2

/* A CAN bit rate must be provided */

#  ifndef CONFIG_LPC17_40_CAN2_BAUD
#    error "CONFIG_LPC17_40_CAN2_BAUD is not defined"
#  endif

   /* If no divisor is provided, use a divisor of 4 */

#  ifndef CONFIG_LPC17_40_CAN2_DIVISOR
#    define CONFIG_LPC17_40_CAN2_DIVISOR 4
#  endif

/* Get the SYSCON_PCLKSEL value for CAN2 the implements this divisor */

#  if CONFIG_LPC17_40_CAN2_DIVISOR == 1
#    define CAN2_CCLK_DIVISOR SYSCON_PCLKSEL_CCLK
#  elif CONFIG_LPC17_40_CAN2_DIVISOR == 2
#    define CAN2_CCLK_DIVISOR SYSCON_PCLKSEL_CCLK2
#  elif CONFIG_LPC17_40_CAN2_DIVISOR == 4
#    define CAN2_CCLK_DIVISOR SYSCON_PCLKSEL_CCLK4
#  elif CONFIG_LPC17_40_CAN2_DIVISOR == 6
#    define CAN2_CCLK_DIVISOR SYSCON_PCLKSEL_CCLK6
#  else
#    error "Unsupported value of CONFIG_LPC17_40_CAN2_DIVISOR"
#  endif
#endif

/* User-defined TSEG1 and TSEG2 settings may be used.
 *
 * CONFIG_LPC17_40_CAN_TSEG1 = the number of CAN time quanta in segment 1
 * CONFIG_LPC17_40_CAN_TSEG2 = the number of CAN time quanta in segment 2
 * CAN_BIT_QUANTA   = The number of CAN time quanta in on bit time
 */

#ifndef CONFIG_LPC17_40_CAN_TSEG1
#  define CONFIG_LPC17_40_CAN_TSEG1 6
#endif

#if CONFIG_LPC17_40_CAN_TSEG1 < 1 || CONFIG_LPC17_40_CAN_TSEG1 > CAN_BTR_TSEG1_MAX
#  error "CONFIG_LPC17_40_CAN_TSEG1 is out of range"
#endif

#ifndef CONFIG_LPC17_40_CAN_TSEG2
#  define CONFIG_LPC17_40_CAN_TSEG2 7
#endif

#if CONFIG_LPC17_40_CAN_TSEG2 < 1 || CONFIG_LPC17_40_CAN_TSEG2 > CAN_BTR_TSEG2_MAX
#  error "CONFIG_LPC17_40_CAN_TSEG2 is out of range"
#endif

#define CAN_BIT_QUANTA (CONFIG_LPC17_40_CAN_TSEG1 + CONFIG_LPC17_40_CAN_TSEG2 + 1)

/* Debug ********************************************************************/

/* Non-standard debug that may be enabled just for testing CAN */

#ifndef CONFIG_DEBUG_CAN_INFO
#  undef CONFIG_LPC17_40_CAN_REGDEBUG
#endif

/* Timing *******************************************************************/

/* CAN clocking is provided at CCLK divided by the configured divisor */

#ifdef BOARD_CCLKSEL_DIVIDER
#  define CAN_CLOCK_FREQUENCY(d) \
    ((uint32_t)LPC17_40_CCLK * BOARD_CCLKSEL_DIVIDER / (uint32_t)(d))
#else
#  define CAN_CLOCK_FREQUENCY(d) ((uint32_t)LPC17_40_CCLK / (uint32_t)(d))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  uint8_t  port;    /* CAN port number */
  uint8_t  divisor; /* CCLK divisor (numeric value) */
  uint32_t baud;    /* Configured baud */
  uint32_t base;    /* CAN register base address */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* CAN Register access */

#ifdef CONFIG_LPC17_40_CAN_REGDEBUG
static void can_printreg(uint32_t addr, uint32_t value);
#endif

static uint32_t can_getreg(struct up_dev_s *priv, int offset);
static void can_putreg(struct up_dev_s *priv, int offset, uint32_t value);

#ifdef CONFIG_LPC17_40_CAN_REGDEBUG
static uint32_t can_getcommon(uint32_t addr);
static void can_putcommon(uint32_t addr, uint32_t value);
#else
#  define can_getcommon(addr)        getreg32(addr)
#  define can_putcommon(addr, value) putreg32(value, addr)
#endif

#ifdef CONFIG_CAN_EXTID
static int lpc17can_add_extfilter(FAR struct up_dev_s *priv,
              FAR struct canioc_extfilter_s *extconfig);
static int lpc17can_del_extfilter(FAR struct up_dev_s *priv, int ndx);
#endif
static int lpc17can_add_stdfilter(FAR struct up_dev_s *priv,
              FAR struct canioc_stdfilter_s *stdconfig);
static int lpc17can_del_stdfilter(FAR struct up_dev_s *priv, int ndx);


/* CAN methods */

static void lpc17can_reset(struct can_dev_s *dev);
static int  lpc17can_setup(struct can_dev_s *dev);
static void lpc17can_shutdown(struct can_dev_s *dev);
static void lpc17can_rxint(struct can_dev_s *dev, bool enable);
static void lpc17can_txint(struct can_dev_s *dev, bool enable);
static int  lpc17can_ioctl(struct can_dev_s *dev, int cmd,
                           unsigned long arg);
static int  lpc17can_remoterequest(struct can_dev_s *dev, uint16_t id);
static int  lpc17can_send(struct can_dev_s *dev,
                          struct can_msg_s *msg);
static bool lpc17can_txready(struct can_dev_s *dev);
static bool lpc17can_txempty(struct can_dev_s *dev);

/* CAN interrupts */

static void can_interrupt(struct can_dev_s *dev);
static int  can12_interrupt(int irq, void *context, void *arg);

/* Initialization */

static int can_bittiming(struct up_dev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct can_ops_s g_canops =
{
  .co_reset         = lpc17can_reset,
  .co_setup         = lpc17can_setup,
  .co_shutdown      = lpc17can_shutdown,
  .co_rxint         = lpc17can_rxint,
  .co_txint         = lpc17can_txint,
  .co_ioctl         = lpc17can_ioctl,
  .co_remoterequest = lpc17can_remoterequest,
  .co_send          = lpc17can_send,
  .co_txready       = lpc17can_txready,
  .co_txempty       = lpc17can_txempty,
};

#ifdef CONFIG_LPC17_40_CAN1
static struct up_dev_s g_can1priv =
{
  .port    = 1,
  .divisor = CONFIG_LPC17_40_CAN1_DIVISOR,
  .baud    = CONFIG_LPC17_40_CAN1_BAUD,
  .base    = LPC17_40_CAN1_BASE,
};

static struct can_dev_s g_can1dev =
{
  .cd_ops  = &g_canops,
  .cd_priv = &g_can1priv,
};
#endif

#ifdef CONFIG_LPC17_40_CAN2
static struct up_dev_s g_can2priv =
{
  .port    = 2,
  .divisor = CONFIG_LPC17_40_CAN2_DIVISOR,
  .baud    = CONFIG_LPC17_40_CAN2_BAUD,
  .base    = LPC17_40_CAN2_BASE,
};

static struct can_dev_s g_can2dev =
{
  .cd_ops  = &g_canops,
  .cd_priv = &g_can2priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: can_printreg
 *
 * Description:
 *   Print the value read from a register.
 *
 * Input Parameters:
 *   addr - The register address
 *   value - The register value
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_LPC17_40_CAN_REGDEBUG
static void can_printreg(uint32_t addr, uint32_t value)
{
  static uint32_t prevaddr = 0;
  static uint32_t preval   = 0;
  static uint32_t count    = 0;

  /* Is this the same value that we read from the same register last time?
   * Are we polling the register?  If so, suppress some of the output.
   */

  if (addr == prevaddr && value == preval)
    {
      if (count == 0xffffffff || ++count > 3)
        {
          if (count == 4)
            {
              caninfo("...\n");
            }

          return;
        }
    }

  /* No this is a new address or value */

  else
    {
      /* Did we print "..." for the previous value? */

      if (count > 3)
        {
          /* Yes.. then show how many times the value repeated */

          caninfo("[repeats %d more times]\n", count - 3);
        }

      /* Save the new address, value, and count */

      prevaddr = addr;
      preval   = value;
      count    = 1;
    }

  /* Show the register value read */

  caninfo("%08x->%08x\n", addr, value);
}
#endif

/****************************************************************************
 * Name: can_getreg
 *
 * Description:
 *   Read the value of an CAN1/2 register.
 *
 * Input Parameters:
 *   priv - A reference to the CAN block status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *
 ****************************************************************************/

#ifdef CONFIG_LPC17_40_CAN_REGDEBUG
static uint32_t can_getreg(struct up_dev_s *priv, int offset)
{
  uint32_t addr;
  uint32_t value;

  /* Read the value from the register */

  addr  = priv->base + offset;
  value = getreg32(addr);
  can_printreg(addr, value);
  return value;
}
#else
static uint32_t can_getreg(struct up_dev_s *priv, int offset)
{
  return getreg32(priv->base + offset);
}
#endif

/****************************************************************************
 * Name: can_putreg
 *
 * Description:
 *   Set the value of an CAN1/2 register.
 *
 * Input Parameters:
 *   priv - A reference to the CAN block status
 *   offset - The offset to the register to write
 *   value - The value to write to the register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_LPC17_40_CAN_REGDEBUG
static void can_putreg(struct up_dev_s *priv, int offset, uint32_t value)
{
  uint32_t addr = priv->base + offset;

  /* Show the register value being written */

  caninfo("%08x<-%08x\n", addr, value);

  /* Write the value */

  putreg32(value, addr);
}
#else
static void can_putreg(struct up_dev_s *priv, int offset, uint32_t value)
{
  putreg32(value, priv->base + offset);
}
#endif

/****************************************************************************
 * Name: can_getcommon
 *
 * Description:
 *   Get the value of common register.
 *
 * Input Parameters:
 *   addr - The address of the register to read
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_LPC17_40_CAN_REGDEBUG
static uint32_t can_getcommon(uint32_t addr)
{
  uint32_t value;

  /* Read the value from the register */

  value = getreg32(addr);
  can_printreg(addr, value);
  return value;
}
#endif

/****************************************************************************
 * Name: can_putcommon
 *
 * Description:
 *   Set the value of common register.
 *
 * Input Parameters:
 *   addr - The address of the register to write
 *   value - The value to write to the register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_LPC17_40_CAN_REGDEBUG
static void can_putcommon(uint32_t addr, uint32_t value)
{
  /* Show the register value being written */

  caninfo("%08x<-%08x\n", addr, value);

  /* Write the value */

  putreg32(value, addr);
}
#endif

/****************************************************************************
 * Name: lpc17can_reset
 *
 * Description:
 *   Reset the CAN device.  Called early to initialize the hardware. This
 *   function is called, before lpc17can_setup() and on error conditions.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

static void lpc17can_reset(struct can_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->cd_priv;
  irqstate_t flags;
  int ret;

  caninfo("CAN%d\n", priv->port);

  flags = enter_critical_section();

  /* Disable the CAN and stop ongong transmissions */

  can_putreg(priv, LPC17_40_CAN_MOD_OFFSET, CAN_MOD_RM);  /* Enter Reset Mode */
  can_putreg(priv, LPC17_40_CAN_IER_OFFSET, 0);           /* Disable interrupts */
  can_putreg(priv, LPC17_40_CAN_GSR_OFFSET, 0);           /* Clear status bits */
  can_putreg(priv, LPC17_40_CAN_CMR_OFFSET, CAN_CMR_AT);  /* Abort transmission */

  /* Set bit timing */

  ret = can_bittiming(priv);
  if (ret != OK)
    {
      canerr("ERROR: Failed to set bit timing: %d\n", ret);
    }

  /* Restart the CAN */

#ifdef CONFIG_CAN_LOOPBACK
  can_putreg(priv, LPC17_40_CAN_MOD_OFFSET, CAN_MOD_STM); /* Leave Reset Mode, enter Test Mode */
#else
  can_putreg(priv, LPC17_40_CAN_MOD_OFFSET, 0);           /* Leave Reset Mode */
#endif
  can_putcommon(LPC17_40_CANAF_AFMR, CANAF_AFMR_ACCBP);   /* All RX messages accepted */
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: lpc17can_setup
 *
 * Description:
 *   Configure the CAN. This method is called the first time that the CAN
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching CAN interrupts.
 *   All CAN interrupts are disabled upon return.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int lpc17can_setup(struct can_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->cd_priv;
  int ret;

  caninfo("CAN%d\n", priv->port);

  ret = irq_attach(LPC17_40_IRQ_CAN, can12_interrupt, NULL);
  if (ret == OK)
    {
      up_enable_irq(LPC17_40_IRQ_CAN);
    }

  return ret;
}

/****************************************************************************
 * Name: lpc17can_shutdown
 *
 * Description:
 *   Disable the CAN.  This method is called when the CAN device is closed.
 *   This method reverses the operation the setup method.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc17can_shutdown(struct can_dev_s *dev)
{
#ifdef CONFIG_DEBUG_CAN_INFO
  struct up_dev_s *priv = (struct up_dev_s *)dev->cd_priv;

  caninfo("CAN%d\n", priv->port);
#endif

  up_disable_irq(LPC17_40_IRQ_CAN);
  irq_detach(LPC17_40_IRQ_CAN);
}

/****************************************************************************
 * Name: lpc17can_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc17can_rxint(struct can_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->cd_priv;
  uint32_t regval;
  irqstate_t flags;

  caninfo("CAN%d enable: %d\n", priv->port, enable);

  /* The EIR register is also modified from the interrupt handler, so we have
   * to protect this code section.
   */

  flags = enter_critical_section();
  regval = can_getreg(priv, LPC17_40_CAN_IER_OFFSET);
  if (enable)
    {
      regval |= CAN_IER_RIE;
    }
  else
    {
      regval &= ~CAN_IER_RIE;
    }

  can_putreg(priv, LPC17_40_CAN_IER_OFFSET, regval);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: lpc17can_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc17can_txint(struct can_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->cd_priv;
  uint32_t regval;
  irqstate_t flags;

  caninfo("CAN%d enable: %d\n", priv->port, enable);

  /* Only disabling of the TX interrupt is supported here.  The TX interrupt
   * is automatically enabled just before a message is sent in order to avoid
   * lost TX interrupts.
   */

  if (!enable)
    {
      /* TX interrupts are also disabled from the interrupt handler, so we
       * have to protect this code section.
       */

      flags = enter_critical_section();

      /* Disable all TX interrupts */

      regval = can_getreg(priv, LPC17_40_CAN_IER_OFFSET);
      regval &= ~(CAN_IER_TIE1 | CAN_IER_TIE2 | CAN_IER_TIE3);
      can_putreg(priv, LPC17_40_CAN_IER_OFFSET, regval);
      leave_critical_section(flags);
    }
}

/****************************************************************************
 * Name: lpc17can_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int lpc17can_ioctl(struct can_dev_s *dev, int cmd,
                          unsigned long arg)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->cd_priv;
  int ret = -ENOTTY;

  caninfo("cmd=%04x arg=%lu\n", cmd, arg);

  DEBUGASSERT(dev && dev->cd_priv);
  priv = dev->cd_priv;

  /* Handle the command */

  switch (cmd)
      {
        /* CANIOC_GET_BITTIMING:
         *   Description:    Return the current bit timing settings
         *   Argument:       A pointer to a write-able instance of struct
         *                   canioc_bittiming_s in which current bit timing
         *                   values will be returned.
         *   Returned Value: Zero (OK) is returned on success.  Otherwise -1
         *                   (ERROR) is returned with the errno variable set
         *                   to indicate the nature of the error.
         *   Dependencies:   None
         */

        case CANIOC_GET_BITTIMING:
          {
            FAR struct canioc_bittiming_s *bt =
              (FAR struct canioc_bittiming_s *)arg;
            uint32_t regval;
            uint32_t brp;

            DEBUGASSERT(bt != NULL);
            regval       = can_getreg(priv, LPC17_40_CAN_BTR_OFFSET);
            bt->bt_sjw   = ((regval & CAN_BTR_SJW_MASK) >> CAN_BTR_SJW_SHIFT) + 1;
            bt->bt_tseg1 = ((regval & CAN_BTR_TSEG1_MASK) >> CAN_BTR_TSEG1_SHIFT) + 1;
            bt->bt_tseg2 = ((regval & CAN_BTR_TSEG2_MASK) >> CAN_BTR_TSEG2_SHIFT) + 1;

            brp          = ((regval & CAN_BTR_BRP_MASK) >> CAN_BTR_BRP_SHIFT) + 1;
            bt->bt_baud  = CAN_CLOCK_FREQUENCY(priv->divisor) /
                           (brp * (bt->bt_tseg1 + bt->bt_tseg2 + 1));
            ret = OK;
          }
          break;

        /* CANIOC_SET_BITTIMING:
         *   Description:    Set new current bit timing values
         *   Argument:       A pointer to a read-able instance of struct
         *                   canioc_bittiming_s in which the new bit timing
         *                   values are provided.
         *   Returned Value: Zero (OK) is returned on success.  Otherwise -1
         *                   (ERROR)is returned with the errno variable set
         *                    to indicate thenature of the error.
         *   Dependencies:   None
         *
         * REVISIT: There is probably a limitation here:  If there are multiple
         * threads trying to send CAN packets, when one of these threads
         * reconfigures the bitrate, the MCAN hardware will be reset and the
         * context of operation will be lost.  Hence, this IOCTL can only safely
         * be executed in quiescent time periods.
         */

        case CANIOC_SET_BITTIMING:
          {
            FAR const struct canioc_bittiming_s *bt =
              (FAR const struct canioc_bittiming_s *)arg;
            uint32_t brp;
            uint32_t can_bit_quanta;
            uint32_t tmp;
            uint32_t regval;
            uint32_t ier;
            irqstate_t flags;

            DEBUGASSERT(bt != NULL);
            DEBUGASSERT(bt->bt_baud < CAN_CLOCK_FREQUENCY(priv->divisor));
            DEBUGASSERT(bt->bt_sjw > 0 && bt->bt_sjw <= 4);
            DEBUGASSERT(bt->bt_tseg1 > 0 && bt->bt_tseg1 <= 16);
            DEBUGASSERT(bt->bt_tseg2 > 0 && bt->bt_tseg2 <=  8);

            regval = can_getreg(priv, LPC17_40_CAN_BTR_OFFSET);

            /* Extract bit timing data */
            /* tmp is in clocks per bit time */

            tmp = CAN_CLOCK_FREQUENCY(priv->divisor) / bt->bt_baud;

            /* This value is dynamic as requested by user */

            can_bit_quanta = bt->bt_tseg1 + bt->bt_tseg2 + 1;

            if (tmp < can_bit_quanta)
              {
                /* This timing is not possible */

                ret = -EINVAL;
                break;
              }

            /* Otherwise, nquanta is can_bit_quanta, ts1 and ts2 are
             * provided by the user and we calculate brp to achieve
             * can_bit_quanta quanta in the bit times
             */

            else
              {
                brp = (tmp + (can_bit_quanta/2)) / can_bit_quanta;
                DEBUGASSERT(brp >= 1 && brp <= CAN_BTR_BRP_MAX);
              }

            caninfo("TS1: %d TS2: %d BRP: %d\n",
                    bt->bt_tseg1, bt->bt_tseg2, brp);

            /* Configure bit timing. */

            regval &= ~(CAN_BTR_BRP_MASK | CAN_BTR_TSEG1_MASK |
                        CAN_BTR_TSEG2_MASK | CAN_BTR_SJW_MASK);
            regval |= ((brp          - 1) << CAN_BTR_BRP_SHIFT) |
                      ((bt->bt_tseg1 - 1) << CAN_BTR_TSEG1_SHIFT) |
                      ((bt->bt_tseg2 - 1) << CAN_BTR_TSEG2_SHIFT) |
                      ((bt->bt_sjw   - 1) << CAN_BTR_SJW_SHIFT);

            /* Bit timing can only be configured in reset mode. */

            flags = enter_critical_section();

            /* Save enabled interrupts */
            ier = can_getreg(priv, LPC17_40_CAN_IER_OFFSET);

            /* Disable the CAN and stop ongoing transmissions */
            while (can_getreg(priv, LPC17_40_CAN_GSR_OFFSET) & CAN_GSR_TS) {
                can_putreg(priv, LPC17_40_CAN_CMR_OFFSET, CAN_CMR_AT);  /* Abort transmission */
            }
            can_putreg(priv, LPC17_40_CAN_MOD_OFFSET, CAN_MOD_RM);  /* Enter Reset Mode */
            can_putreg(priv, LPC17_40_CAN_IER_OFFSET, 0);           /* Disable interrupts */
            can_putreg(priv, LPC17_40_CAN_GSR_OFFSET, 0);           /* Clear status bits */

            can_putreg(priv, LPC17_40_CAN_BTR_OFFSET, regval);

            can_putreg(priv, LPC17_40_CAN_MOD_OFFSET, 0);           /* Leave Reset Mode */

            can_putreg(priv, LPC17_40_CAN_IER_OFFSET, ier);         /* Restore enabled interrupts */

            leave_critical_section(flags);

            priv->baud  = CAN_CLOCK_FREQUENCY(priv->divisor) /
                (brp * (bt->bt_tseg1 + bt->bt_tseg2 + 1));
            ret = OK;
          }
          break;
#if 0
        /* CANIOC_GET_CONNMODES:
         *   Description:    Get the current bus connection modes
         *   Argument:       A pointer to a write-able instance of struct
         *                   canioc_connmodes_s in which the new bus modes will
         *                   be returned.
         *   Returned Value: Zero (OK) is returned on success.  Otherwise -1
         *                   (ERROR)is returned with the errno variable set
         *                   to indicate the nature of the error.
         *   Dependencies:   None
         */

        case CANIOC_GET_CONNMODES:
          {
            FAR struct canioc_connmodes_s *bm =
              (FAR struct canioc_connmodes_s *)arg;
            uint32_t regval;

            DEBUGASSERT(bm != NULL);

            regval          = stm32can_getreg(priv, STM32_CAN_BTR_OFFSET);

            bm->bm_loopback = ((regval & CAN_BTR_LBKM) == CAN_BTR_LBKM);
            bm->bm_silent   = ((regval & CAN_BTR_SILM) == CAN_BTR_SILM);
            ret = OK;
            break;
          }

        /* CANIOC_SET_CONNMODES:
         *   Description:    Set new bus connection modes values
         *   Argument:       A pointer to a read-able instance of struct
         *                   canioc_connmodes_s in which the new bus modes
         *                   are provided.
         *   Returned Value: Zero (OK) is returned on success.  Otherwise -1
         *                   (ERROR) is returned with the errno variable set
         *                   to indicate the nature of the error.
         *   Dependencies:   None
         */

        case CANIOC_SET_CONNMODES:
          {
            FAR struct canioc_connmodes_s *bm =
              (FAR struct canioc_connmodes_s *)arg;
            uint32_t regval;

            DEBUGASSERT(bm != NULL);

            regval = stm32can_getreg(priv, STM32_CAN_BTR_OFFSET);

            if (bm->bm_loopback)
              {
                regval |= CAN_BTR_LBKM;
              }
            else
              {
                regval &= ~CAN_BTR_LBKM;
              }

            if (bm->bm_silent)
              {
                regval |= CAN_BTR_SILM;
              }
            else
              {
                regval &= ~CAN_BTR_SILM;
              }

            /* This register can only be configured in init mode. */

            ret = stm32can_enterinitmode(priv);
            if (ret < 0)
              {
                break;
              }

            stm32can_putreg(priv, STM32_CAN_BTR_OFFSET, regval);

            ret = stm32can_exitinitmode(priv);
          }
          break;
#endif
  #ifdef CONFIG_CAN_EXTID
        /* CANIOC_ADD_EXTFILTER:
         *   Description:    Add an address filter for a extended 29 bit
         *                   address.
         *   Argument:       A reference to struct canioc_extfilter_s
         *   Returned Value: A non-negative filter ID is returned on success.
         *                   Otherwise -1 (ERROR) is returned with the errno
         *                   variable set to indicate the nature of the error.
         */

        case CANIOC_ADD_EXTFILTER:
          {
            DEBUGASSERT(arg != 0);
            ret = lpc17can_add_extfilter(priv,
                                        (FAR struct canioc_extfilter_s *)arg);
          }
          break;

        /* CANIOC_DEL_EXTFILTER:
         *   Description:    Remove an address filter for a standard 29 bit
         *                   address.
         *   Argument:       The filter index previously returned by the
         *                   CANIOC_ADD_EXTFILTER command
         *   Returned Value: Zero (OK) is returned on success.  Otherwise -1
         *                   (ERROR)is returned with the errno variable set
         *                   to indicate the nature of the error.
         */

        case CANIOC_DEL_EXTFILTER:
          {
  #if 0 /* Unimplemented */
            DEBUGASSERT(arg <= priv->config->nextfilters);
  #endif
            ret = lpc17can_del_extfilter(priv, (int)arg);
          }
          break;
  #endif

        /* CANIOC_ADD_STDFILTER:
         *   Description:    Add an address filter for a standard 11 bit
         *                   address.
         *   Argument:       A reference to struct canioc_stdfilter_s
         *   Returned Value: A non-negative filter ID is returned on success.
         *                   Otherwise -1 (ERROR) is returned with the errno
         *                   variable set to indicate the nature of the error.
         */

        case CANIOC_ADD_STDFILTER:
          {
            DEBUGASSERT(arg != 0);
            ret = lpc17can_add_stdfilter(priv,
                                        (FAR struct canioc_stdfilter_s *)arg);
          }
          break;

        /* CANIOC_DEL_STDFILTER:
         *   Description:    Remove an address filter for a standard 11 bit
         *                   address.
         *   Argument:       The filter index previously returned by the
         *                   CANIOC_ADD_STDFILTER command
         *   Returned Value: Zero (OK) is returned on success.  Otherwise -1
         *                   (ERROR) is returned with the errno variable set
         *                   to indicate the nature of the error.
         */

        case CANIOC_DEL_STDFILTER:
          {
  #if 0 /* Unimplemented */
            DEBUGASSERT(arg <= priv->config->nstdfilters);
  #endif
            ret = lpc17can_del_stdfilter(priv, (int)arg);
          }
          break;

        /* Unsupported/unrecognized command */

        default:
          canerr("ERROR: Unrecognized command: %04x\n", cmd);
          break;
      }

  return ret;

}

/****************************************************************************
 * Name: lpc17can_remoterequest
 *
 * Description:
 *   Send a remote request
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int lpc17can_remoterequest(struct can_dev_s *dev, uint16_t id)
{
  canerr("ERROR: Fix me -- Not Implemented\n");
  return 0;
}

/****************************************************************************
 * Name: lpc17can_send
 *
 * Description:
 *    Send one can message.
 *
 *    One CAN-message consists of a maximum of 10 bytes.  A message is
 *    composed of at least the first 2 bytes (when there are no data bytes).
 *
 *    Byte 0:      Bits 0-7: Bits 3-10 of the 11-bit CAN identifier
 *    Byte 1:      Bits 5-7: Bits 0-2 of the 11-bit CAN identifier
 *                 Bit 4:    Remote Transmission Request (RTR)
 *                 Bits 0-3: Data Length Code (DLC)
 *    Bytes 2-10: CAN data
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int lpc17can_send(struct can_dev_s *dev,
                         struct can_msg_s *msg)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->cd_priv;
  uint32_t tid = (uint32_t)msg->cm_hdr.ch_id;
  uint32_t tfi = (uint32_t)msg->cm_hdr.ch_dlc << 16;
  uint32_t regval;
  irqstate_t flags;
  int ret = OK;

  caninfo("CAN%d ID: %" PRId32 " DLC: %d\n",
          priv->port, (uint32_t)msg->cm_hdr.ch_id, msg->cm_hdr.ch_dlc);

  if (msg->cm_hdr.ch_rtr)
    {
      tfi |= CAN_TFI_RTR;
    }

  /* Set the FF bit in the TFI register if this message should be sent with
   * the extended frame format (and 29-bit extended ID).
   */

#ifdef CONFIG_CAN_EXTID
  if (msg->cm_hdr.ch_extid)
    {
      /* The provided ID should be 29 bits */

      DEBUGASSERT((tid & ~CAN_TID_ID29_MASK) == 0);
      tfi |= CAN_TFI_FF;
    }
  else
#endif
    {
      /* The provided ID should be 11 bits */

      DEBUGASSERT((tid & ~CAN_TID_ID11_MASK) == 0);
    }

  flags = enter_critical_section();

  /* Pick a transmit buffer */

  regval = can_getreg(priv, LPC17_40_CAN_SR_OFFSET);
  if ((regval & CAN_SR_TBS1) != 0)
    {
      /* Make sure that buffer 1 TX interrupts are enabled BEFORE sending the
       * message. The TX interrupt is generated when the TBSn bit in CANxSR
       * goes from 0 to 1 when the TIEn bit in CANxIER is 1.  If we don't
       * enable it now, we may miss the TIE1 interrupt.
       *
       * NOTE: The IER is also modified from the interrupt handler, but the
       * following is safe because interrupts are disabled here.
       */

      regval  = can_getreg(priv, LPC17_40_CAN_IER_OFFSET);
      regval |= CAN_IER_TIE1;
      can_putreg(priv, LPC17_40_CAN_IER_OFFSET, regval);

      /* Set up the transfer */

      can_putreg(priv, LPC17_40_CAN_TFI1_OFFSET, tfi);
      can_putreg(priv, LPC17_40_CAN_TID1_OFFSET, tid);
      can_putreg(priv, LPC17_40_CAN_TDA1_OFFSET,
                 *(uint32_t *)&msg->cm_data[0]);
      can_putreg(priv, LPC17_40_CAN_TDB1_OFFSET,
                 *(uint32_t *)&msg->cm_data[4]);

      /* Send the message */

#ifdef CONFIG_CAN_LOOPBACK
      can_putreg(priv, LPC17_40_CAN_CMR_OFFSET, CAN_CMR_STB1 | CAN_CMR_SRR);
#else
      can_putreg(priv, LPC17_40_CAN_CMR_OFFSET, CAN_CMR_STB1 | CAN_CMR_TR);
#endif
    }
  else if ((regval & CAN_SR_TBS2) != 0)
    {
      /* Make sure that buffer 2 TX interrupts are enabled BEFORE sending the
       * message. The TX interrupt is generated when the TBSn bit in CANxSR
       * goes from 0 to 1 when the TIEn bit in CANxIER is 1.  If we don't
       * enable it now, we may miss the TIE2 interrupt.
       *
       * NOTE: The IER is also modified from the interrupt handler, but the
       * following is safe because interrupts are disabled here.
       */

      regval  = can_getreg(priv, LPC17_40_CAN_IER_OFFSET);
      regval |= CAN_IER_TIE2;
      can_putreg(priv, LPC17_40_CAN_IER_OFFSET, regval);

      /* Set up the transfer */

      can_putreg(priv, LPC17_40_CAN_TFI2_OFFSET, tfi);
      can_putreg(priv, LPC17_40_CAN_TID2_OFFSET, tid);
      can_putreg(priv, LPC17_40_CAN_TDA2_OFFSET,
                 *(uint32_t *)&msg->cm_data[0]);
      can_putreg(priv, LPC17_40_CAN_TDB2_OFFSET,
                 *(uint32_t *)&msg->cm_data[4]);

      /* Send the message */

#ifdef CONFIG_CAN_LOOPBACK
      can_putreg(priv, LPC17_40_CAN_CMR_OFFSET, CAN_CMR_STB2 | CAN_CMR_SRR);
#else
      can_putreg(priv, LPC17_40_CAN_CMR_OFFSET, CAN_CMR_STB2 | CAN_CMR_TR);
#endif
    }
  else if ((regval & CAN_SR_TBS3) != 0)
    {
      /* Make sure that buffer 3 TX interrupts are enabled BEFORE sending the
       * message. The TX interrupt is generated when the TBSn bit in CANxSR
       * goes from 0 to 1 when the TIEn bit in CANxIER is 1.  If we don't
       * enable it now, we may miss the TIE3 interrupt.
       *
       * NOTE: The IER is also modified from the interrupt handler, but the
       * following is safe because interrupts are disabled here.
       */

      regval  = can_getreg(priv, LPC17_40_CAN_IER_OFFSET);
      regval |= CAN_IER_TIE3;
      can_putreg(priv, LPC17_40_CAN_IER_OFFSET, regval);

      /* Set up the transfer */

      can_putreg(priv, LPC17_40_CAN_TFI3_OFFSET, tfi);
      can_putreg(priv, LPC17_40_CAN_TID3_OFFSET, tid);
      can_putreg(priv, LPC17_40_CAN_TDA3_OFFSET,
                 *(uint32_t *)&msg->cm_data[0]);
      can_putreg(priv, LPC17_40_CAN_TDB3_OFFSET,
                 *(uint32_t *)&msg->cm_data[4]);

      /* Send the message */

#ifdef CONFIG_CAN_LOOPBACK
      can_putreg(priv, LPC17_40_CAN_CMR_OFFSET, CAN_CMR_STB3 | CAN_CMR_SRR);
#else
      can_putreg(priv, LPC17_40_CAN_CMR_OFFSET, CAN_CMR_STB3 | CAN_CMR_TR);
#endif
    }
  else
    {
      canerr("ERROR: No available transmission buffer, SR: %08" PRIx32 "\n",
             regval);
      ret = -EBUSY;
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: lpc17can_txready
 *
 * Description:
 *   Return true if the CAN hardware can accept another TX message.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   True if the CAN hardware is ready to accept another TX message.
 *
 ****************************************************************************/

static bool lpc17can_txready(struct can_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->cd_priv;
  uint32_t regval = can_getreg(priv, LPC17_40_CAN_SR_OFFSET);
  return ((regval & (CAN_SR_TBS1 | CAN_SR_TBS2 | CAN_SR_TBS3)) != 0);
}

/****************************************************************************
 * Name: lpc17can_txempty
 *
 * Description:
 *   Return true if all message have been sent.  If for example, the CAN
 *   hardware implements FIFOs, then this would mean the transmit FIFO is
 *   empty.  This method is called when the driver needs to make sure that
 *   all characters are "drained" from the TX hardware before calling
 *   co_shutdown().
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   True if there are no pending TX transfers in the CAN hardware.
 *
 ****************************************************************************/

static bool lpc17can_txempty(struct can_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->cd_priv;
  uint32_t regval = can_getreg(priv, LPC17_40_CAN_GSR_OFFSET);
  return ((regval & CAN_GSR_TBS) != 0);
}

/****************************************************************************
 * Name: can_interrupt
 *
 * Description:
 *   CAN1/2 RX/TX interrupt handler
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static void can_interrupt(struct can_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->cd_priv;
  struct can_hdr_s hdr;
  uint32_t data[2];
  uint32_t rfs;
  uint32_t rid;
  uint32_t regval;

  /* Read the interrupt and capture register (also clearing most status
   * bits)
   */

  regval = can_getreg(priv, LPC17_40_CAN_ICR_OFFSET);
  caninfo("CAN%d ICR: %08" PRIx32 "\n", priv->port, regval);

  /* Check for a receive interrupt */

  if ((regval & CAN_ICR_RI) != 0)
    {
      rfs     = can_getreg(priv, LPC17_40_CAN_RFS_OFFSET);
      rid     = can_getreg(priv, LPC17_40_CAN_RID_OFFSET);
      data[0] = can_getreg(priv, LPC17_40_CAN_RDA_OFFSET);
      data[1] = can_getreg(priv, LPC17_40_CAN_RDB_OFFSET);

      /* Release the receive buffer */

      can_putreg(priv, LPC17_40_CAN_CMR_OFFSET, CAN_CMR_RRB);

      /* Construct the CAN header */

      hdr.ch_id     = rid;
      hdr.ch_rtr    = ((rfs & CAN_RFS_RTR) != 0);
      hdr.ch_dlc    = (rfs & CAN_RFS_DLC_MASK) >> CAN_RFS_DLC_SHIFT;
#ifdef CONFIG_CAN_ERRORS
      hdr.ch_error  = 0; /* Error reporting not supported */
#endif
#ifdef CONFIG_CAN_EXTID
      hdr.ch_extid  = ((rfs & CAN_RFS_FF) != 0);
#else
      hdr.ch_unused = 0;

      if ((rfs & CAN_RFS_FF) != 0)
        {
          canerr("ERROR: Received message with extended identifier.  "
                 "Dropped\n");
        }
      else
#endif
        {
          /* Process the received CAN packet */

          can_receive(dev, &hdr, (uint8_t *)data);
        }
    }

  /* Check for TX buffer 1 complete */

  if ((regval & CAN_ICR_TI1) != 0)
    {
      /* Disable all further TX buffer 1 interrupts */

      regval  = can_getreg(priv, LPC17_40_CAN_IER_OFFSET);
      regval &= ~CAN_IER_TIE1;
      can_putreg(priv, LPC17_40_CAN_IER_OFFSET, regval);

      /* Indicate that the TX is done and a new TX buffer is available */

      can_txdone(dev);
    }

  /* Check for TX buffer 2 complete */

  if ((regval & CAN_ICR_TI2) != 0)
    {
      /* Disable all further TX buffer 2 interrupts */

      regval  = can_getreg(priv, LPC17_40_CAN_IER_OFFSET);
      regval &= ~CAN_IER_TIE2;
      can_putreg(priv, LPC17_40_CAN_IER_OFFSET, regval);

      /* Indicate that the TX is done and a new TX buffer is available */

      can_txdone(dev);
    }

  /* Check for TX buffer 3 complete */

  if ((regval & CAN_ICR_TI3) != 0)
    {
      /* Disable all further TX buffer 3 interrupts */

      regval = can_getreg(priv, LPC17_40_CAN_IER_OFFSET);
      regval &= ~CAN_IER_TIE3;
      can_putreg(priv, LPC17_40_CAN_IER_OFFSET, regval);

      /* Indicate that the TX is done and a new TX buffer is available */

      can_txdone(dev);
    }
}

/****************************************************************************
 * Name: can12_interrupt
 *
 * Description:
 *   CAN interrupt handler.  There is a single interrupt for both CAN1 and
 *   CAN2.
 *
 * Input Parameters:
 *   irq - The IRQ number of the interrupt.
 *   context - The register state save array at the time of the interrupt.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int can12_interrupt(int irq, void *context, void *arg)
{
  /* Handle CAN1/2 interrupts */

  caninfo("irq: %d\n",  irq);

#ifdef CONFIG_LPC17_40_CAN1
  can_interrupt(&g_can1dev);
#endif
#ifdef CONFIG_LPC17_40_CAN2
  can_interrupt(&g_can2dev);
#endif

  return OK;
}

/****************************************************************************
 * Name: can_bittiming
 *
 * Description:
 *   Set the CAN bit timing register (BTR) based on the configured BAUD.
 *
 * The bit timing logic monitors the serial bus-line and performs sampling
 * and adjustment of the sample point by synchronizing on the start-bit edge
 * and resynchronizing on the following edges.
 *
 * Its operation may be explained simply by splitting nominal bit time into
 * three segments as follows:
 *
 * 1. Synchronization segment (SYNC_SEG): a bit change is expected to occur
 *    within this time segment. It has a fixed length of one time quantum
 *    (1 x tCAN).
 * 2. Bit segment 1 (BS1): defines the location of the sample point. It
 *    includes the PROP_SEG and PHASE_SEG1 of the CAN standard. Its duration
 *    is programmable between 1 and 16 time quanta but may be automatically
 *    lengthened to compensate for positive phase drifts due to differences
 *    in the frequency of the various nodes of the network.
 * 3. Bit segment 2 (BS2): defines the location of the transmit point. It
 *    represents the PHASE_SEG2 of the CAN standard. Its duration is
 *    programmable between 1 and 8 time quanta but may also be automatically
 *    shortened to compensate for negative phase drifts.
 *
 * Pictorially:
 *
 *  |<----------------- NOMINAL BIT TIME ----------------->|
 *  |<- SYNC_SEG ->|<------ BS1 ------>|<------ BS2 ------>|
 *  |<---- Tq ---->|<----- Tbs1 ------>|<----- Tbs2 ------>|
 *
 * Where
 *   Tbs1 is the duration of the BS1 segment
 *   Tbs2 is the duration of the BS2 segment
 *   Tq is the "Time Quantum"
 *
 * Relationships:
 *
 *   baud = 1 / bit_time
 *   bit_time = Tq + Tbs1 + Tbs2
 *   Tbs1 = Tq * ts1
 *   Tbs2 = Tq * ts2
 *   Tq = brp * Tcan
 *
 * Where:
 *   Tcan is the period of the APB clock (PCLK =
 *   CCLK / CONFIG_LPC17_40_CAN1_DIVISOR).
 *
 * Input Parameters:
 *   priv - A reference to the CAN block status
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int can_bittiming(struct up_dev_s *priv)
{
  uint32_t btr;
  uint32_t nclks;
  uint32_t brp;
  uint32_t ts1;
  uint32_t ts2;
  uint32_t sjw;

  caninfo("CAN%d PCLK: %" PRId32 " baud: %" PRId32 "\n", priv->port,
          (uint32_t)CAN_CLOCK_FREQUENCY(priv->divisor), priv->baud);

  /* Try to get CAN_BIT_QUANTA quanta in one bit_time.
   *
   *   bit_time = Tq*(ts1 + ts2 + 1)
   *   nquanta  = bit_time/Tq
   *   Tq       = brp * Tcan
   *   nquanta  = (ts1 + ts2 + 1)
   *
   *   bit_time = brp * Tcan * (ts1 + ts2 + 1)
   *   nquanta  = bit_time / brp / Tcan
   *   brp      = Fcan / baud / nquanta;
   *
   * First, calculate the number of CAN clocks in one bit time: Fcan / baud
   */

  nclks = CAN_CLOCK_FREQUENCY(priv->divisor) / priv->baud;
  if (nclks < CAN_BIT_QUANTA)
    {
      /* At the smallest brp value (1), there are already too few bit times
       * (CAN_CLOCK / baud) to meet our goal.  brp must be one and we need
       * make some reasonable guesses about ts1 and ts2.
       */

      brp = 1;

      /* In this case, we have to guess a good value for ts1 and ts2 */

      ts1 = (nclks - 1) >> 1;
      ts2 = nclks - ts1 - 1;
      if (ts1 == ts2 && ts1 > 1 && ts2 < CAN_BTR_TSEG2_MAX)
        {
          ts1--;
          ts2++;
        }
    }

  /* Otherwise, nquanta is CAN_BIT_QUANTA, ts1 is CONFIG_LPC17_40_CAN_TSEG1,
   * ts2 is CONFIG_LPC17_40_CAN_TSEG2 and we calculate brp to achieve
   * CAN_BIT_QUANTA quanta in the bit time
   */

  else
    {
      ts1 = CONFIG_LPC17_40_CAN_TSEG1;
      ts2 = CONFIG_LPC17_40_CAN_TSEG2;
      brp = (nclks + (CAN_BIT_QUANTA / 2)) / CAN_BIT_QUANTA;
      DEBUGASSERT(brp >= 1 && brp <= CAN_BTR_BRP_MAX);
    }

  sjw = 1;

  caninfo("TS1: %" PRId32 " TS2: %" PRId32
          " BRP: %" PRId32 " SJW= %" PRId32 "\n",
          ts1, ts2, brp, sjw);

  /* Configure bit timing */

  btr = (((brp - 1) << CAN_BTR_BRP_SHIFT)   |
         ((ts1 - 1) << CAN_BTR_TSEG1_SHIFT) |
         ((ts2 - 1) << CAN_BTR_TSEG2_SHIFT) |
         ((sjw - 1) << CAN_BTR_SJW_SHIFT));

#ifdef CONFIG_LPC17_40_CAN_SAM
  /* The bus is sampled 3 times (recommended for low to medium speed buses
   * to spikes on the bus-line).
   */

  btr |= CAN_BTR_SAM;
#endif

  caninfo("Setting CANxBTR= 0x%08" PRIx32 "\n", btr);
  can_putreg(priv, LPC17_40_CAN_BTR_OFFSET, btr);        /* Set bit timing */
  return OK;
}




static uint16_t can_createStdIDEntry(CAN_STD_ID_ENTRY_T *pEntryInfo, bool IsFullCANEntry);
static inline uint16_t can_createUnUsedSTDEntry(uint8_t CtrlNo);
static void can_readStdIDEntry(uint16_t EntryVal, CAN_STD_ID_ENTRY_T *pEntryInfo);
static int can_setupSTDSection(uint32_t *pCANAFRamAddr, CAN_STD_ID_ENTRY_T *pStdCANSec,
                           uint16_t EntryCount, bool IsFullCANEntry);
static int can_setupSTDRangeSection(uint32_t *pCANAFRamAddr, CAN_STD_ID_RANGE_ENTRY_T *pStdRangeCANSec, uint16_t EntryCount);
static uint32_t can_createExtIDEntry(CAN_EXT_ID_ENTRY_T *pEntryInfo);
static void can_readExtIDEntry(uint32_t EntryVal, CAN_EXT_ID_ENTRY_T *pEntryInfo);
static int can_setupEXTSection(uint32_t *pCANAFRamAddr, CAN_EXT_ID_ENTRY_T *pExtCANSec, uint16_t EntryCount);
static int can_setupEXTRangeSection(uint32_t *pCANAFRamAddr, CAN_EXT_ID_RANGE_ENTRY_T *pExtRangeCANSec, uint16_t EntryCount);
static void dump_af_ram(void);


/****************************************************************************
 * Name: mcan_add_extfilter
 *
 * Description:
 *   Add an address filter for a extended 29 bit address.
 *
 * Input Parameters:
 *   priv      - An instance of the MCAN driver state structure.
 *   extconfig - The configuration of the extended filter
 *
 * Returned Value:
 *   A non-negative filter ID is returned on success.  Otherwise a negated
 *   errno value is returned to indicate the nature of the error.
 *
 ****************************************************************************/

#ifdef CONFIG_CAN_EXTID
static int lpc17can_add_extfilter(FAR struct up_dev_s *priv,
                              FAR struct canioc_extfilter_s *extconfig)
{

  int result;

  DEBUGASSERT(priv != NULL && extconfig != NULL);

  CAN_EXT_ID_RANGE_ENTRY_T range_ext;

  /* save current CAN acceptance filter mode */
  uint32_t cur_mode = can_getcommon(LPC17_40_CANAF_AFMR);

  /*  AF Off */
  can_putcommon(LPC17_40_CANAF_AFMR, CANAF_AFMR_ACCOFF);

  if (extconfig->xf_type == CAN_FILTER_RANGE) {
    range_ext.LowerID.ID_29 = extconfig->xf_id1;
    range_ext.UpperID.ID_29 = extconfig->xf_id2;
    range_ext.LowerID.CtrlNo = priv->port - 1;
    range_ext.UpperID.CtrlNo = priv->port - 1;
  }
  result = can_setupEXTRangeSection((uint32_t*)LPC17_40_CANAFRAM_BASE, &range_ext, 1);

  /* Return to previous mode */
  can_putcommon(LPC17_40_CANAF_AFMR, cur_mode);
  /* FIXME CHECK RESULT */
  return 0;
  return -EAGAIN;
}
#endif

/****************************************************************************
 * Name: mcan_del_extfilter
 *
 * Description:
 *   Remove an address filter for a standard 29 bit address.
 *
 * Input Parameters:
 *   priv - An instance of the MCAN driver state structure.
 *   ndx  - The filter index previously returned by the mcan_add_extfilter().
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the error.
 *
 ****************************************************************************/

#ifdef CONFIG_CAN_EXTID
static int lpc17can_del_extfilter(FAR struct up_dev_s *priv, int ndx)
{
  FAR const struct sam_config_s *config;
  FAR uint32_t *extfilter;
  uint32_t regval;
  int word;
  int bit;
  int ret;

  DEBUGASSERT(priv != NULL);

  return -ENOTTY;
}
#endif

/****************************************************************************
 * Name: mcan_add_stdfilter
 *
 * Description:
 *   Add an address filter for a standard 11 bit address.
 *
 * Input Parameters:
 *   priv      - An instance of the MCAN driver state structure.
 *   stdconfig - The configuration of the standard filter
 *
 * Returned Value:
 *   A non-negative filter ID is returned on success.  Otherwise a negated
 *   errno value is returned to indicate the nature of the error.
 *
 ****************************************************************************/

static int lpc17can_add_stdfilter(FAR struct up_dev_s *priv,
                              FAR struct canioc_stdfilter_s *stdconfig)
{
  FAR const struct sam_config_s *config;
  FAR uint32_t *stdfilter;
  uint32_t regval;
  int word;
  int bit;
  int ndx;
  int ret;
  int result;

  DEBUGASSERT(priv != NULL && stdconfig != NULL);

  CAN_STD_ID_RANGE_ENTRY_T range_std;

  /* save current CAN acceptance filter mode */
  uint32_t cur_mode = can_getcommon(LPC17_40_CANAF_AFMR);

  /*  AF Off */
  can_putcommon(LPC17_40_CANAF_AFMR, CANAF_AFMR_ACCOFF);

  if (stdconfig->sf_type == CAN_FILTER_RANGE) {
    range_std.LowerID.ID_11 = stdconfig->sf_id1;
    range_std.UpperID.ID_11 = stdconfig->sf_id2;
    range_std.LowerID.Disable = false;
    range_std.UpperID.Disable = false;
    range_std.LowerID.CtrlNo = priv->port - 1;
    range_std.UpperID.CtrlNo = priv->port - 1;
  }
  result = can_setupSTDRangeSection((uint32_t*)LPC17_40_CANAFRAM_BASE, &range_std, 1);

  /* Return to previous mode */
  can_putcommon(LPC17_40_CANAF_AFMR, cur_mode);
  /* FIXME CHECK RESULT */
  return 0;
  return -EAGAIN;
}

/****************************************************************************
 * Name: mcan_del_stdfilter
 *
 * Description:
 *   Remove an address filter for a standard 29 bit address.
 *
 * Input Parameters:
 *   priv - An instance of the MCAN driver state structure.
 *   ndx  - The filter index previously returned by the mcan_add_stdfilter().
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the error.
 *
 ****************************************************************************/

static int lpc17can_del_stdfilter(FAR struct up_dev_s *priv, int ndx)
{
  FAR const struct sam_config_s *config;
  FAR uint32_t *stdfilter;
  uint32_t regval;
  int word;
  int bit;
  int ret;

  DEBUGASSERT(priv != NULL);

  return -ENOTTY;
}

/****************************************************************************
 * Name: can_add_af_range
 *
 * Description:
 *   Adds a range of CAN message ids to the CAN acceptance filter.  This function only implements
 *   the minimum required functionality for our application.  The LPC17xx CAN module provides many
 *   more features that one could conceivably want to implement in the future.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *   start_id - the beginning of the ID range to match
 *   end_id  - the end of the ID range to match
 *   extid - if true, the IDs will be treated as extended IDs.  Otherwise, they will be treated as standard IDs.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/
int can_add_af_range(int port, uint32_t start_id, uint32_t end_id, bool extid) {
  int result;
  CAN_STD_ID_RANGE_ENTRY_T range_std;
  CAN_EXT_ID_RANGE_ENTRY_T range_ext;

  /* save current CAN acceptance filter mode */
  uint32_t CurMode = can_getcommon(LPC17_40_CANAF_AFMR);

  /*  AF Off */
  can_putcommon(LPC17_40_CANAF_AFMR, CANAF_AFMR_ACCOFF);

  if (port > 2) {
    return -1;//error
  }
  port -= 1;//convert to 0-indexed
//  dump_af_ram();
  if (extid) {
    range_ext.LowerID.ID_29 = start_id;
    range_ext.LowerID.CtrlNo = port;
    range_ext.UpperID.ID_29 = end_id;
    range_ext.UpperID.CtrlNo = port;
    result = can_setupEXTRangeSection((uint32_t*)LPC17_40_CANAFRAM_BASE, &range_ext, 1);
  } else { //standard ID
    range_std.LowerID.ID_11 = start_id;
    range_std.LowerID.Disable = false;
    range_std.LowerID.CtrlNo = port;
    range_std.UpperID.ID_11 = end_id;
    range_std.UpperID.Disable = false;
    range_std.UpperID.CtrlNo = port;
    result = can_setupSTDRangeSection((uint32_t*)LPC17_40_CANAFRAM_BASE, &range_std, 1);
  }
//  dump_af_ram();
  /* Return to previous mode */
  can_putcommon(LPC17_40_CANAF_AFMR, CurMode);
  return 0;
}

/****************************************************************************
 * Name: can_add_af_range
 *
 * Description:
 *  Erases all acceptance filters from the acceptance filter LUT RAM.
 *
 * Input Parameters:
 *  None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
void can_reset_af(void) {
  uint32_t i = 0;
  /* save current CAN acceptance filter mode */
  uint32_t CurMode = can_getcommon(LPC17_40_CANAF_AFMR);

  /*  AF Off */
  can_putcommon(LPC17_40_CANAF_AFMR, CANAF_AFMR_ACCOFF);

  /* Clear AF Ram region */
  for (i = 0; i < CANAF_RAM_ENTRY_NUM; i++) {
     putreg32(0, (LPC17_40_CANAFRAM_BASE + i));
  }

  /* Reset address registers */
  can_putcommon(LPC17_40_CANAF_SFFSA, 0);
  can_putcommon(LPC17_40_CANAF_SFFGRPSA, 0);
  can_putcommon(LPC17_40_CANAF_EFFSA, 0);
  can_putcommon(LPC17_40_CANAF_EFFGRPSA, 0);
  can_putcommon(LPC17_40_CANAF_EOT, 0);

  /* Return to previous mode */
  can_putcommon(LPC17_40_CANAF_AFMR, CurMode);
}
/*
 * Sets the CAN acceptance filter mode.
 */
void can_set_af_mode(uint32_t can_mode) {
  /* only allow valid bits to be used */
  can_mode &= CANAF_AFMR_MASK;
  /* set AF mode */
  can_putcommon(LPC17_40_CANAF_AFMR, can_mode);
}

/*
 * Configure the acceptance filter address tables assuming only one entry in the CAN address range section
 * (either standard or extended.
 * Minimum required to support the functionality we're using in the actuator.
 */
void can_configure_af_sections(bool extid) {
  if (extid) {
    can_putcommon(LPC17_40_CANAF_SFFSA, CANAF_ENDADDR(0));
    can_putcommon(LPC17_40_CANAF_SFFGRPSA, CANAF_ENDADDR(0));
    can_putcommon(LPC17_40_CANAF_EFFSA, CANAF_ENDADDR(0));
    can_putcommon(LPC17_40_CANAF_EFFGRPSA, CANAF_ENDADDR(0));//extended ID range entry is two words wide
    can_putcommon(LPC17_40_CANAF_EOT, CANAF_ENDADDR(3));
  } else {
    can_putcommon(LPC17_40_CANAF_SFFSA, CANAF_ENDADDR(0));
    can_putcommon(LPC17_40_CANAF_SFFGRPSA, CANAF_ENDADDR(0));//standard ID range entry is one word wide
    can_putcommon(LPC17_40_CANAF_EFFSA, CANAF_ENDADDR(1));
    can_putcommon(LPC17_40_CANAF_EFFGRPSA, CANAF_ENDADDR(1));
    can_putcommon(LPC17_40_CANAF_EOT, CANAF_ENDADDR(1));
  }
//  dump_af_ram();
}

uint8_t get_can_tx_error_count(int port) {
  struct up_dev_s *priv;
  switch (port) {
    case 0:
#ifdef CONFIG_LPC17_40_CAN1
      priv = g_can1dev.cd_priv;
#else
      return 0;
#endif
      break;
    case 1:
#ifdef CONFIG_LPC17_40_CAN2
      priv = g_can2dev.cd_priv;
#else
      return 0;
#endif
      break;
    default:
      return 0;
      break;
  }
  return (can_getreg(priv, LPC17_40_CAN_GSR_OFFSET) & CAN_GSR_RXERR_MASK) >> CAN_GSR_RXERR_SHIFT;

}

uint8_t get_can_rx_error_count(int port) {
  struct up_dev_s *priv;
  switch (port) {
    case 0:
#ifdef CONFIG_LPC17_40_CAN1
      priv = g_can1dev.cd_priv;
#else
      return 0;
#endif
      break;
    case 1:
#ifdef CONFIG_LPC17_40_CAN2
      priv = g_can2dev.cd_priv;
#else
      return 0;
#endif
      break;
    default:
      return 0;
      break;
  }
  return (can_getreg(priv, LPC17_40_CAN_GSR_OFFSET) & CAN_GSR_TXERR_MASK) >> CAN_GSR_TXERR_SHIFT;
}

uint8_t get_can_device_status(int port) {
  struct up_dev_s *priv;
  switch (port) {
    case 0:
#ifdef CONFIG_LPC17_40_CAN1
      priv = g_can1dev.cd_priv;
#else
      return 0;
#endif
      break;
    case 1:
#ifdef CONFIG_LPC17_40_CAN2
      priv = g_can2dev.cd_priv;
#else
      return 0;
#endif
      break;
    default:
      return 0;
      break;
  }
  return (can_getreg(priv, LPC17_40_CAN_GSR_OFFSET)  &  (CAN_GSR_ES | CAN_GSR_BS))  >> 6;
}

/* Create the standard ID entry */
static uint16_t can_createStdIDEntry(CAN_STD_ID_ENTRY_T *pEntryInfo, bool IsFullCANEntry) {
    uint16_t Entry = 0;
    Entry = (pEntryInfo->CtrlNo & CAN_STD_ENTRY_CTRL_NO_MASK) << CAN_STD_ENTRY_CTRL_NO_POS;
    Entry |= (pEntryInfo->Disable & CAN_STD_ENTRY_DISABLE_MASK) << CAN_STD_ENTRY_DISABLE_POS;
    Entry |= (pEntryInfo->ID_11 & CAN_STD_ENTRY_ID_MASK) << CAN_STD_ENTRY_ID_POS;
    if (IsFullCANEntry) {
        Entry |= 1 << CAN_STD_ENTRY_IE_POS;
    }
    return Entry;
}

static inline uint16_t can_createUnUsedSTDEntry(uint8_t CtrlNo) {
    return ((CtrlNo & CAN_STD_ENTRY_CTRL_NO_MASK) << CAN_STD_ENTRY_CTRL_NO_POS) | (1 << CAN_STD_ENTRY_DISABLE_POS);
}

/* Get information from the standard ID entry */
static void can_readStdIDEntry(uint16_t EntryVal, CAN_STD_ID_ENTRY_T *pEntryInfo) {
    pEntryInfo->CtrlNo = (EntryVal >> CAN_STD_ENTRY_CTRL_NO_POS) & CAN_STD_ENTRY_CTRL_NO_MASK;
    pEntryInfo->Disable = (EntryVal >> CAN_STD_ENTRY_DISABLE_POS) & CAN_STD_ENTRY_DISABLE_MASK;
    pEntryInfo->ID_11 = (EntryVal >> CAN_STD_ENTRY_ID_POS) & CAN_STD_ENTRY_ID_MASK;
}

/* Setup Standard ID section */
static int can_setupSTDSection(uint32_t *pCANAFRamAddr, CAN_STD_ID_ENTRY_T *pStdCANSec,
                           uint16_t EntryCount, bool IsFullCANEntry) {
    uint16_t i;
    uint16_t CurID = 0;
    uint16_t Entry;
    uint16_t EntryCnt = 0;

    /* Setup FullCAN section */
    for (i = 0; i < EntryCount; i += 2) {
        /* First Entry */
        if (CurID > pStdCANSec[i].ID_11) {
            return -1;//error
        }
        CurID = pStdCANSec[i].ID_11;
        Entry = can_createStdIDEntry(&pStdCANSec[i], IsFullCANEntry);
        pCANAFRamAddr[EntryCnt] = Entry << 16;

        /* Second Entry */
        if ((i + 1) < EntryCount) {
            if (CurID > pStdCANSec[i + 1].ID_11) {
                return -1;//error
            }
            CurID = pStdCANSec[i + 1].ID_11;
            Entry = can_createStdIDEntry(&pStdCANSec[i + 1], IsFullCANEntry);
            pCANAFRamAddr[EntryCnt] |= Entry;
        }
        else {
            pCANAFRamAddr[EntryCnt] |= can_createUnUsedSTDEntry(pStdCANSec[0].CtrlNo);
        }
        EntryCnt++;
    }
    return 0;//success
}

/* Setup the Group Standard ID section */
static int can_setupSTDRangeSection(uint32_t *pCANAFRamAddr, CAN_STD_ID_RANGE_ENTRY_T *pStdRangeCANSec, uint16_t EntryCount) {
    return can_setupSTDSection(pCANAFRamAddr, (CAN_STD_ID_ENTRY_T *) pStdRangeCANSec, EntryCount * 2, false);
}

static uint32_t can_createExtIDEntry(CAN_EXT_ID_ENTRY_T *pEntryInfo) {
    uint32_t Entry = 0;
    Entry = (pEntryInfo->CtrlNo & CAN_EXT_ENTRY_CTRL_NO_MASK) << CAN_EXT_ENTRY_CTRL_NO_POS;
    Entry |= (pEntryInfo->ID_29 & CAN_EXT_ENTRY_ID_MASK) << CAN_EXT_ENTRY_ID_POS;
    return Entry;
}

/* Get information from an extended ID entry */
static void can_readExtIDEntry(uint32_t EntryVal, CAN_EXT_ID_ENTRY_T *pEntryInfo) {
    pEntryInfo->CtrlNo = (EntryVal >> CAN_EXT_ENTRY_CTRL_NO_POS) & CAN_EXT_ENTRY_CTRL_NO_MASK;
    pEntryInfo->ID_29 = (EntryVal >> CAN_EXT_ENTRY_ID_POS) & CAN_EXT_ENTRY_ID_MASK;
}

/* Setup the Extended ID Section */
static int can_setupEXTSection(uint32_t *pCANAFRamAddr, CAN_EXT_ID_ENTRY_T *pExtCANSec, uint16_t EntryCount) {
    uint16_t i;
    uint32_t CurID = 0;
    uint32_t Entry;
    uint16_t EntryCnt = 0;

    /* Setup Extended ID section */
    for (i = 0; i < EntryCount; i++) {
        if (CurID > pExtCANSec[i].ID_29) {
            return -1;//error
        }
        CurID = pExtCANSec[i].ID_29;
        Entry = can_createExtIDEntry(&pExtCANSec[i]);
        pCANAFRamAddr[EntryCnt] = Entry;
        EntryCnt++;
    }
    return 0;//success

}

/* Setup Group Extended ID section */
static int can_setupEXTRangeSection(uint32_t *pCANAFRamAddr, CAN_EXT_ID_RANGE_ENTRY_T *pExtRangeCANSec, uint16_t EntryCount) {
    return can_setupEXTSection(pCANAFRamAddr, (CAN_EXT_ID_ENTRY_T *) pExtRangeCANSec, EntryCount * 2);
}

static void dump_af_ram(void) {
  int i;
  uint32_t tmp32;
  syslog(LOG_INFO, "---------------------------\n\n");
  /* print AF Ram region */
  for (i = 0; i < 10 /*CANAF_RAM_ENTRY_NUM*/; i++) {
     tmp32 = getreg32((LPC17_40_CANAFRAM_BASE + (i * 4)));
     syslog(LOG_INFO, "%08x\n", tmp32);
  }
  /* print address registers */
  tmp32 = getreg32(LPC17_40_CANAF_SFFSA);
  syslog(LOG_INFO, "LPC17_40_CANAF_SFFSA    %08x\n", tmp32);
  tmp32 = getreg32(LPC17_40_CANAF_SFFGRPSA);
  syslog(LOG_INFO, "LPC17_40_CANAF_SFFGRPSA %08x\n", tmp32);
  tmp32 = getreg32(LPC17_40_CANAF_EFFSA);
  syslog(LOG_INFO, "LPC17_40_CANAF_EFFSA    %08x\n", tmp32);
  tmp32 = getreg32(LPC17_40_CANAF_EFFGRPSA);
  syslog(LOG_INFO, "LPC17_40_CANAF_EFFGRPSA %08x\n", tmp32);
  tmp32 = getreg32(LPC17_40_CANAF_EOT);
  syslog(LOG_INFO, "LPC17_40_CANAF_EOT      %08x\n", tmp32);
  //Print AFMR
  tmp32 = getreg32(LPC17_40_CANAF_AFMR);
  syslog(LOG_INFO, "LPC17_40_CANAF_AFMR     %08x\n", tmp32);
}


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc17_40_caninitialize
 *
 * Description:
 *   Initialize the selected CAN port
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple CAN interfaces)
 *
 * Returned Value:
 *   Valid CAN device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct can_dev_s *lpc17_40_caninitialize(int port)
{
  struct can_dev_s *candev;
  irqstate_t flags;
  uint32_t regval;

  caninfo("CAN%d\n",  port);

  flags = enter_critical_section();

#ifdef CONFIG_LPC17_40_CAN1
  if (port == 1)
    {
      /* Enable power to the CAN module */

      regval  = can_getcommon(LPC17_40_SYSCON_PCONP);
      regval |= SYSCON_PCONP_PCCAN1;
      can_putcommon(LPC17_40_SYSCON_PCONP, regval);

      /* Enable clocking to the CAN module (not necessary... already done
       * in low level clock configuration logic).
       */

#ifdef LPC178x_40xx
      regval  = can_getcommon(LPC17_40_SYSCON_PCLKSEL);
      regval &= SYSCON_PCLKSEL_PCLKDIV_MASK;
      regval >>= SYSCON_PCLKSEL_PCLKDIV_SHIFT;
      g_can1priv.divisor = regval;
#else
      regval  = can_getcommon(LPC17_40_SYSCON_PCLKSEL0);
      regval &= ~SYSCON_PCLKSEL0_CAN1_MASK;
      regval |= (CAN1_CCLK_DIVISOR << SYSCON_PCLKSEL0_CAN1_SHIFT);
      can_putcommon(LPC17_40_SYSCON_PCLKSEL0, regval);
#endif
      /* Configure CAN GPIO pins */

      lpc17_40_configgpio(GPIO_CAN1_RD);
      lpc17_40_configgpio(GPIO_CAN1_TD);

      candev = &g_can1dev;
    }
  else
#endif
#ifdef CONFIG_LPC17_40_CAN2
  if (port == 2)
    {
      /* Enable power to the CAN module */

      regval  = can_getcommon(LPC17_40_SYSCON_PCONP);
      regval |= SYSCON_PCONP_PCCAN2;
      can_putcommon(LPC17_40_SYSCON_PCONP, regval);

      /* Enable clocking to the CAN module (not necessary... already done
       * in low level clock configuration logic).
       */

#ifdef LPC178x_40xx
      regval  = can_getcommon(LPC17_40_SYSCON_PCLKSEL);
      regval &= SYSCON_PCLKSEL_PCLKDIV_MASK;
      regval >>= SYSCON_PCLKSEL_PCLKDIV_SHIFT;
      g_can2priv.divisor = regval;
#else
      regval  = can_getcommon(LPC17_40_SYSCON_PCLKSEL0);
      regval &= ~SYSCON_PCLKSEL0_CAN2_MASK;
      regval |= (CAN2_CCLK_DIVISOR << SYSCON_PCLKSEL0_CAN2_SHIFT);
      can_putcommon(LPC17_40_SYSCON_PCLKSEL0, regval);
#endif
      /* Configure CAN GPIO pins */

      lpc17_40_configgpio(GPIO_CAN2_RD);
      lpc17_40_configgpio(GPIO_CAN2_TD);

      candev = &g_can2dev;
    }
  else
#endif
    {
      canerr("ERROR: Unsupported port: %d\n", port);
      leave_critical_section(flags);
      return NULL;
    }

  /* Then just perform a CAN reset operation */

  lpc17can_reset(candev);
  leave_critical_section(flags);
  return candev;
}
#endif
