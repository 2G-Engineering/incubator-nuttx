/****************************************************************************
 * drivers/ioexpander/mcp23sxx.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/spi/spi.h>
#include <nuttx/kmalloc.h>
#include <nuttx/ioexpander/ioexpander.h>

#include "mcp23sxx.h"

#if defined(CONFIG_IOEXPANDER_MCP23SXX)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SPI
#  warning SPI support is required (CONFIG_SPI)
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline int mcp23sxx_write(FAR struct mcp23sxx_dev_s *mcp,
             uint8_t reg, FAR const uint8_t *wbuffer, int wbuflen);
static inline int mcp23sxx_read(FAR struct mcp23sxx_dev_s *mcp,
             FAR const uint8_t reg, FAR uint8_t *rbuffer, int rbuflen);
static int mcp23sxx_direction(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             int dir);
static int mcp23sxx_option(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             int opt, void *val);
static int mcp23sxx_writepin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             bool value);
static int mcp23sxx_readpin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             FAR bool *value);
static int mcp23sxx_readbuf(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             FAR bool *value);
#ifdef CONFIG_IOEXPANDER_MULTIPIN
static int mcp23sxx_multiwritepin(FAR struct ioexpander_dev_s *dev,
             FAR uint8_t *pins, FAR bool *values, int count);
static int mcp23sxx_multireadpin(FAR struct ioexpander_dev_s *dev,
             FAR uint8_t *pins, FAR bool *values, int count);
static int mcp23sxx_multireadbuf(FAR struct ioexpander_dev_s *dev,
             FAR uint8_t *pins, FAR bool *values, int count);
#endif
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
static FAR void *mcp23sxx_attach(FAR struct ioexpander_dev_s *dev,
             ioe_pinset_t pinset, ioe_callback_t callback, FAR void *arg);
static int mcp23sxx_detach(FAR struct ioexpander_dev_s *dev,
             FAR void *handle);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifndef CONFIG_MCP23SXX_MULTIPLE
/* If only a single MCP23SXX device is supported, then the driver state
 * structure may as well be pre-allocated.
 */

static struct mcp23sxx_dev_s g_mcp23sxx;

/* Otherwise, we will need to maintain allocated driver instances in a list */

#else
static struct mcp23sxx_dev_s *g_mcp23sxxlist;
#endif

/* I/O expander vtable */

static const struct ioexpander_ops_s g_mcp23sxx_ops =
{
  mcp23sxx_direction,
  mcp23sxx_option,
  mcp23sxx_writepin,
  mcp23sxx_readpin,
  mcp23sxx_readbuf
#ifdef CONFIG_IOEXPANDER_MULTIPIN
  , mcp23sxx_multiwritepin
  , mcp23sxx_multireadpin
  , mcp23sxx_multireadbuf
#endif
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  , mcp23sxx_attach
  , mcp23sxx_detach
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mcp23sxx_lock
 *
 * Description:
 *   Get exclusive access to the MCP23SXX
 *
 ****************************************************************************/

static int mcp23sxx_lock(FAR struct mcp23sxx_dev_s *mcp)
{
  int ret;

  /* On SPI buses where there are multiple devices, it will be necessary to
   * lock SPI to have exclusive access to the buses for a sequence of
   * transfers.  The bus should be locked before the chip is selected.
   *
   * This is a blocking call and will not return until we have exclusive
   * access to the SPI bus.  We will retain that exclusive access until the
   * bus is unlocked.
   */

  SPI_LOCK(mcp->spi, true);

  ret =  nxsem_wait_uninterruptible(&mcp->exclsem);

  if (ret == OK)
    {
      /* Configure the SPI port for this device */

      SPI_SETMODE(mcp->spi, SPIDEV_MODE0);
      SPI_SETBITS(mcp->spi, 8);
      SPI_HWFEATURES(mcp->spi, 0);
      SPI_SETFREQUENCY(mcp->spi, mcp->config->frequency);
    }

  return ret;
}

/****************************************************************************
 * Name: mcp23sxx_unlock
 *
 * Description:
 *   Release exclusive access to the MCP23SXX
 *
 ****************************************************************************/

static int mcp23sxx_unlock(FAR struct mcp23sxx_dev_s *mcp)
{
  SPI_LOCK(mcp->spi, false);

  int ret = nxsem_post(&mcp->exclsem);

  return ret;
}

/****************************************************************************
 * Name: mcp23sxx_write
 *
 * Description:
 *   Write to the SPI device.
 *
 ****************************************************************************/

static inline int mcp23sxx_write(FAR struct mcp23sxx_dev_s *mcp,
                                 uint8_t reg,
                                FAR const uint8_t *wbuffer, int wbuflen)
{
  uint8_t opcode;

  SPI_SELECT(mcp->spi, SPIDEV_EXPANDER(mcp->index), true);

  opcode = MCP23SXX_CONST_ADDR |
           ((mcp->config->address << 1) & MCP23SXX_ADDR_MASK) |
           MCP23SXX_WRITE;

  SPI_SEND(mcp->spi, opcode);

  SPI_SEND(mcp->spi, reg);

  SPI_SNDBLOCK(mcp->spi, wbuffer, wbuflen);

  SPI_SELECT(mcp->spi, SPIDEV_EXPANDER(mcp->index), false);

  return OK;
}

/****************************************************************************
 * Name: mcp23sxx_read
 *
 * Description:
 *   Read from the SPI device.
 *
 ****************************************************************************/

static inline int mcp23sxx_read(FAR struct mcp23sxx_dev_s *mcp,
                                    FAR const uint8_t reg,
                                    FAR uint8_t *rbuffer, int rbuflen)
{
  uint8_t opcode;

  SPI_SELECT(mcp->spi, SPIDEV_EXPANDER(mcp->index), true);

  opcode = MCP23SXX_CONST_ADDR |
           ((mcp->config->address << 1) & MCP23SXX_ADDR_MASK) |
           MCP23SXX_READ;

  SPI_SEND(mcp->spi, opcode);

  SPI_SEND(mcp->spi, reg);

  SPI_RECVBLOCK(mcp->spi, rbuffer, rbuflen);

  SPI_SELECT(mcp->spi, SPIDEV_EXPANDER(mcp->index), false);

  return OK;
}

/****************************************************************************
 * Name: mcp23sxx_setbit
 *
 * Description:
 *  Write a bit in a register pair
 *
 ****************************************************************************/

static int mcp23sxx_setbit(FAR struct mcp23sxx_dev_s *mcp, uint8_t addr,
                          uint8_t pin, int bitval)
{
  uint8_t buf[1];
  int ret;

  if (pin >= mcp->config->pincount)
    {
      return -ENXIO;
    }

  /* Increment the address if necessary */

  if (pin >= 8)
    {
      addr += 1;
    }

  /* Bounds check */

  if (addr >= MCP23SXX_NUMREGS)
    {
      return -ENXIO;
    }

#ifdef CONFIG_MCP23SXX_SHADOW_MODE
  /* Get the shadowed register value */

  buf[0] = mcp->sreg[addr];

#else
  /* Get the register value from the IO-Expander */

  ret = mcp23sxx_read(mcp, addr, &buf[0], 1);
  if (ret < 0)
    {
      return ret;
    }
#endif

  if (bitval)
    {
      buf[0] |= (1 << pin);
    }
  else
    {
      buf[0] &= ~(1 << pin);
    }

#ifdef CONFIG_MCP23SXX_SHADOW_MODE
  /* Save the new register value in the shadow register */

  mcp->sreg[addr] = buf[0];
#endif

  ret = mcp23sxx_write(mcp, addr, buf, 1);
#ifdef CONFIG_MCP23SXX_RETRY
  if (ret != OK)
    {
      /* Try again (only once) */

      ret = mcp23sxx_write(mcp, addr, buf, 1);
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: mcp23sxx_getbit
 *
 * Description:
 *  Get a bit from a register pair
 *
 ****************************************************************************/

static int mcp23sxx_getbit(FAR struct mcp23sxx_dev_s *mcp, uint8_t addr,
                          uint8_t pin, FAR bool *val)
{
  uint8_t buf;
  int ret;

  if (pin >= mcp->config->pincount)
    {
      return -ENXIO;
    }

  /* Increment the address if necessary */

  if (pin >= 8)
    {
      addr += 1;
    }

  /* Bounds check */

  if (addr >= MCP23SXX_NUMREGS)
    {
      return -ENXIO;
    }

  ret = mcp23sxx_read(mcp, addr, &buf, 1);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_MCP23SXX_SHADOW_MODE
  /* Save the new register value in the shadow register */

  mcp->sreg[addr] = buf;
#endif

  *val = (buf >> pin) & 1;
  return OK;
}

/****************************************************************************
 * Name: mcp23sxx_direction
 *
 * Description:
 *   Set the direction of an ioexpander pin. Required.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   pin - The index of the pin to alter in this call
 *   dir - One of the IOEXPANDER_DIRECTION_ macros
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int mcp23sxx_direction(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                             int direction)
{
  FAR struct mcp23sxx_dev_s *mcp = (FAR struct mcp23sxx_dev_s *)dev;
  int ret;
  uint8_t reg;

  /* Get exclusive access to the MCP23SXX */

  ret = mcp23sxx_lock(mcp);
  if (ret < 0)
    {
      return ret;
    }

  if (pin < 8)
    {
      reg = MCP23SXX_IODIRA;
    }
  else
    {
      reg = MCP23SXX_IODIRB;
    }

  ret = mcp23sxx_setbit(mcp, reg, pin,
                       (direction == IOEXPANDER_DIRECTION_IN));
  gpioinfo("mcp23sxx pin %d set direction: %d\n", pin,
           direction == IOEXPANDER_DIRECTION_IN);
  mcp23sxx_unlock(mcp);
  return ret;
}

/****************************************************************************
 * Name: mcp23sxx_option
 *
 * Description:
 *   Set pin options. Required.
 *   Since all IO expanders have various pin options, this API allows setting
 *     pin options in a flexible way.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   pin - The index of the pin to alter in this call
 *   opt - One of the IOEXPANDER_OPTION_ macros
 *   val - The option's value
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int mcp23sxx_option(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                          int opt, FAR void *val)
{
  FAR struct mcp23sxx_dev_s *mcp = (FAR struct mcp23sxx_dev_s *)dev;
  int ret = -EINVAL;
  uint8_t reg;

  if (opt == IOEXPANDER_OPTION_INVERT)
    {
      int ival = (int)((intptr_t)val);

      /* Get exclusive access to the MCP23SXX */

      ret = mcp23sxx_lock(mcp);
      if (ret < 0)
        {
          return ret;
        }

      if (pin < 8)
        {
          reg = MCP23SXX_IPOLA;
        }
      else
        {
          reg = MCP23SXX_IPOLB;
        }

      ret = mcp23sxx_setbit(mcp, reg, pin, ival);
      gpioinfo("mcp23sxx pin %d set invert: %d\n", pin, ival);
      mcp23sxx_unlock(mcp);
    }

  return ret;
}

/****************************************************************************
 * Name: mcp23sxx_writepin
 *
 * Description:
 *   Set the pin level. Required.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   pin   - The index of the pin to alter in this call
 *   value - The pin level. Usually TRUE will set the pin high,
 *           except if OPTION_INVERT has been set on this pin.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int mcp23sxx_writepin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                            bool value)
{
  FAR struct mcp23sxx_dev_s *mcp = (FAR struct mcp23sxx_dev_s *)dev;
  int ret;
  uint8_t reg;

  /* Get exclusive access to the MCP23SXX */

  ret = mcp23sxx_lock(mcp);
  if (ret < 0)
    {
      return ret;
    }

  if (pin < 8)
    {
      reg = MCP23SXX_GPIOA;
    }
  else
    {
      reg = MCP23SXX_GPIOB;
    }

  ret = mcp23sxx_setbit(mcp, reg, pin, value);
  gpioinfo("mcp23sxx pin %d set bit: %d\n", pin, value);
  mcp23sxx_unlock(mcp);
  return ret;
}

/****************************************************************************
 * Name: mcp23sxx_readpin
 *
 * Description:
 *   Read the actual PIN level. This can be different from the last value
 *   written to this pin. Required.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   pin    - The index of the pin
 *   value  - Pointer to a buffer where the pin level is stored.
 *            Usually TRUE if the pin is high, except if OPTION_INVERT
 *            has been set on this pin.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int mcp23sxx_readpin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                           FAR bool *value)
{
  FAR struct mcp23sxx_dev_s *mcp = (FAR struct mcp23sxx_dev_s *)dev;
  int ret;
  uint8_t reg;

  /* Get exclusive access to the MCP23SXX */

  ret = mcp23sxx_lock(mcp);
  if (ret < 0)
    {
      return ret;
    }

  if (pin < 8)
    {
      reg = MCP23SXX_GPIOA;
    }
  else
    {
      reg = MCP23SXX_GPIOB;
    }

  ret = mcp23sxx_getbit(mcp, reg, pin, value);
  gpioinfo("mcp23sxx pin %d get bit: %d\n", pin, value);
  mcp23sxx_unlock(mcp);
  return ret;
}

/****************************************************************************
 * Name: mcp23sxx_readbuf
 *
 * Description:
 *   Read the buffered pin level.
 *   This can be different from the actual pin state. Required.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   pin    - The index of the pin
 *   value  - Pointer to a buffer where the level is stored.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int mcp23sxx_readbuf(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                           FAR bool *value)
{
  FAR struct mcp23sxx_dev_s *mcp = (FAR struct mcp23sxx_dev_s *)dev;
  int ret;
  uint8_t reg;

  /* Get exclusive access to the MCP23SXX */

  ret = mcp23sxx_lock(mcp);
  if (ret < 0)
    {
      return ret;
    }

  if (pin < 8)
    {
      reg = MCP23SXX_OLATA;
    }
  else
    {
      reg = MCP23SXX_OLATB;
    }

  ret = mcp23sxx_getbit(mcp, reg, pin, value);
  mcp23sxx_unlock(mcp);
  return ret;
}

#ifdef CONFIG_IOEXPANDER_MULTIPIN

/****************************************************************************
 * Name: mcp23sxx_getmultibits
 *
 * Description:
 *  Read multiple bits from MCP23SXX registers.
 *
 ****************************************************************************/

static int mcp23sxx_getmultibits(FAR struct mcp23sxx_dev_s *mcp,
                                 uint8_t addr, FAR uint8_t *pins,
                                 FAR bool *values, int count)
{
  uint8_t buf[2];
  int ret = OK;
  int i;
  int index;
  int pin;

  ret = mcp23sxx_read(mcp, addr, buf, 2);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_MCP23SXX_SHADOW_MODE
  /* Save the new register value in the shadow register */

  mcp->sreg[addr]   = buf[0];
  mcp->sreg[addr + 1] = buf[1];
#endif

  /* Read the requested bits */

  for (i = 0; i < count; i++)
    {
      pin   = pins[i];
      if (pin >= mcp->config->pincount)
        {
          return -ENXIO;
        }

      if (pin < 8)
        {
          index = 0;
        }
      else
        {
          index = 1;
        }

      values[i] = (buf[index] >> pin) & 1;
    }

  return OK;
}

/****************************************************************************
 * Name: mcp23sxx_multiwritepin
 *
 * Description:
 *   Set the pin level for multiple pins. This routine may be faster than
 *   individual pin accesses. Optional.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   pins   - The list of pin indexes to alter in this call
 *   values - The list of pin levels.
 *   count  - The number of pins to write.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int mcp23sxx_multiwritepin(FAR struct ioexpander_dev_s *dev,
                                 FAR uint8_t *pins, FAR bool *values,
                                 int count)
{
  FAR struct mcp23sxx_dev_s *mcp = (FAR struct mcp23sxx_dev_s *)dev;
  uint8_t addr = MCP23SXX_GPIOA;
  uint8_t buf[2];
  int ret;
  int i;
  int index;
  int pin;

  /* Get exclusive access to the MCP23SXX */

  ret = mcp23sxx_lock(mcp);
  if (ret < 0)
    {
      return ret;
    }

  /* Start by reading both registers, whatever the pins to change. We could
   * attempt to read one port only if all pins were on the same port, but
   * this would not save much.
   */

#ifndef CONFIG_MCP23SXX_SHADOW_MODE
  ret = mcp23sxx_read(mcp, addr, &buf[0], 2);
  if (ret < 0)
    {
      mcp23sxx_unlock(mcp);
      return ret;
    }
#else
  /* In Shadow-Mode we "read" the pin status from the shadow registers */

  buf[0] = mcp->sreg[addr];
  buf[1] = mcp->sreg[addr + 1];
#endif

  /* Apply the user defined changes */

  for (i = 0; i < count; i++)
    {
      pin = pins[i];
      if (pin >= mcp->config->pincount)
        {
          mcp23sxx_unlock(mcp);
          return -ENXIO;
        }

      if (pin < 8)
        {
          index = 0;
        }
      else
        {
          index = 1;
        }

      if (values[i])
        {
          buf[index] |= (1 << pin);
        }
      else
        {
          buf[index] &= ~(1 << pin);
        }
    }

  /* Now write back the new pins states */

#ifdef CONFIG_MCP23SXX_SHADOW_MODE
  /* Save the new register values in the shadow register */

  mcp->sreg[addr] = buf[0];
  mcp->sreg[addr + 1] = buf[1];
#endif
  ret = mcp23sxx_write(mcp, addr, buf, 3);

  mcp23sxx_unlock(mcp);
  return ret;
}

/****************************************************************************
 * Name: mcp23sxx_multireadpin
 *
 * Description:
 *   Read the actual level for multiple pins. This routine may be faster than
 *   individual pin accesses. Optional.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   pin    - The list of pin indexes to read
 *   values - List where read pin values will be stored.
 *   count  - Number of pins to read
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int mcp23sxx_multireadpin(FAR struct ioexpander_dev_s *dev,
                                FAR uint8_t *pins, FAR bool *values,
                                int count)
{
  FAR struct mcp23sxx_dev_s *mcp = (FAR struct mcp23sxx_dev_s *)dev;
  int ret;

  /* Get exclusive access to the MCP23SXX */

  ret = mcp23sxx_lock(mcp);
  if (ret < 0)
    {
      return ret;
    }

  ret = mcp23sxx_getmultibits(mcp, MCP23SXX_GPIOA,
                             pins, values, count);
  mcp23sxx_unlock(mcp);
  return ret;
}

/****************************************************************************
 * Name: mcp23sxx_multireadbuf
 *
 * Description:
 *   Read the buffered level of multiple pins. This routine may be faster
 *   than individual pin accesses. Optional.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   pin    - The index of the pin
 *   valptr - Pointer to a buffer where the buffered levels are stored.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int mcp23sxx_multireadbuf(FAR struct ioexpander_dev_s *dev,
                                FAR uint8_t *pins, FAR bool *values,
                                int count)
{
  FAR struct mcp23sxx_dev_s *mcp = (FAR struct mcp23sxx_dev_s *)dev;
  int ret;

  /* Get exclusive access to the MCP23SXX */

  ret = mcp23sxx_lock(mcp);
  if (ret < 0)
    {
      return ret;
    }

  ret = mcp23sxx_getmultibits(mcp, MCP23SXX_GPIOA,
                             pins, values, count);
  mcp23sxx_unlock(mcp);
  return ret;
}

#endif

#ifdef CONFIG_MCP23SXX_INT_ENABLE

/****************************************************************************
 * Name: mcp23sxx_attach
 *
 * Description:
 *   Attach and enable a pin interrupt callback function.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   pinset   - The set of pin events that will generate the callback
 *   callback - The pointer to callback function.  NULL will detach the
 *              callback.
 *   arg      - User-provided callback argument
 *
 * Returned Value:
 *   A non-NULL handle value is returned on success.  This handle may be
 *   used later to detach and disable the pin interrupt.
 *
 ****************************************************************************/

static FAR void *mcp23sxx_attach(FAR struct ioexpander_dev_s *dev,
                                ioe_pinset_t pinset, ioe_callback_t callback,
                                FAR void *arg)
{
  FAR struct mcp23sxx_dev_s *mcp = (FAR struct mcp23sxx_dev_s *)dev;
  FAR void *handle = NULL;
  int i;
  int ret;

  /* Get exclusive access to the MCP23SXX */

  ret = mcp23sxx_lock(mcp);
  if (ret < 0)
    {
      return ret;
    }

  /* Find and available in entry in the callback table */

  for (i = 0; i < CONFIG_MCP23SXX_INT_NCALLBACKS; i++)
    {
      /* Is this entry available (i.e., no callback attached) */

      if (mcp->cb[i].cbfunc == NULL)
        {
          /* Yes.. use this entry */

          mcp->cb[i].pinset = pinset;
          mcp->cb[i].cbfunc = callback;
          mcp->cb[i].cbarg  = arg;
          handle            = &mcp->cb[i];
          break;
        }
    }

  /* Add this callback to the table */

  mcp23sxx_unlock(mcp);
  return handle;
}

/****************************************************************************
 * Name: mcp23sxx_detach
 *
 * Description:
 *   Detach and disable a pin interrupt callback function.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   handle   - The non-NULL opaque value return by mcp23sxx_attch()
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int mcp23sxx_detach(FAR struct ioexpander_dev_s *dev,
                           FAR void *handle)
{
  FAR struct mcp23sxx_dev_s *mcp = (FAR struct mcp23sxx_dev_s *)dev;
  FAR struct mcp23sxx_callback_s *cb =
                                    (FAR struct mcp23sxx_callback_s *)handle;

  DEBUGASSERT(mcp != NULL && cb != NULL);
  DEBUGASSERT((uintptr_t)cb >= (uintptr_t)&mcp->cb[0] &&
    (uintptr_t)cb <= (uintptr_t)&mcp->cb[CONFIG_TCA64XX_INT_NCALLBACKS - 1]);

  UNUSED(mcp);

  cb->pinset = 0;
  cb->cbfunc = NULL;
  cb->cbarg  = NULL;
  return OK;
}

/****************************************************************************
 * Name: mcp23sxx_irqworker
 *
 * Description:
 *   Handle GPIO interrupt events (this function actually executes in the
 *   context of the worker thread).
 *
 ****************************************************************************/

static void mcp23sxx_irqworker(void *arg)
{
  FAR struct mcp23sxx_dev_s *mcp = (FAR struct mcp23sxx_dev_s *)arg;
  uint8_t addr = MCP23SXX_GPIOA;
  uint8_t buf[2];
  ioe_pinset_t pinset;
  int ret;
  int i;

  /* Read inputs */

  ret = mcp23sxx_read(mcp, addr, buf, 2);
  if (ret == OK)
    {
#ifdef CONFIG_MCP23SXX_SHADOW_MODE
      /* Don't forget to update the shadow registers at this point */

      mcp->sreg[addr]   = buf[0];
      mcp->sreg[addr + 1] = buf[1];
#endif
      /* Create a 16-bit pinset */

      pinset = ((unsigned int)buf[0] << 8) | buf[1];

      /* Perform pin interrupt callbacks */

      for (i = 0; i < CONFIG_MCP23SXX_INT_NCALLBACKS; i++)
        {
          /* Is this entry valid (i.e., callback attached)?  If so, did
           * any of the requested pin interrupts occur?
           */

          if (mcp->cb[i].cbfunc != NULL)
            {
              /* Did any of the requested pin interrupts occur? */

              ioe_pinset_t match = pinset & mcp->cb[i].pinset;
              if (match != 0)
                {
                  /* Yes.. perform the callback */

                  mcp->cb[i].cbfunc(&mcp->dev, match,
                                    mcp->cb[i].cbarg);
                }
            }
        }
    }

  /* Re-enable interrupts */

  mcp->config->enable(mcp->config, TRUE);
}

/****************************************************************************
 * Name: mcp23sxx_interrupt
 *
 * Description:
 *   Handle GPIO interrupt events (this function executes in the
 *   context of the interrupt).
 *
 ****************************************************************************/

static int mcp23sxx_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct mcp23sxx_dev_s *mcp = (FAR struct mcp23sxx_dev_s *)arg;

  /* In complex environments, we cannot do SPI transfers from the interrupt
   * handler because semaphores are probably used to lock the SPI bus.  In
   * this case, we will defer processing to the worker thread.  This is also
   * much kinder in the use of system resources and is, therefore, probably
   * a good thing to do in any event.
   */

  /* Notice that further GPIO interrupts are disabled until the work is
   * actually performed.  This is to prevent overrun of the worker thread.
   * Interrupts are re-enabled in mcp23sxx_irqworker() when the work is
   * completed.
   */

  if (work_available(&mcp->work))
    {
      mcp->config->enable(mcp->config, FALSE);
      work_queue(HPWORK, &mcp->work, mcp23sxx_irqworker,
                 (FAR void *)mcp, 0);
    }

  return OK;
}

#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mcp23sxx_initialize
 *
 * Description:
 *   Initialize a MCP23SXX SPI device.
 *
 ****************************************************************************/

FAR struct ioexpander_dev_s *mcp23sxx_initialize(
                             FAR struct spi_dev_s *spidev,
                             FAR const struct mcp23sxx_config_s *config)
{
  FAR struct mcp23sxx_dev_s *mcpdev;

  DEBUGASSERT(spidev != NULL && config != NULL &&
                config->set_nreset_pin != NULL &&
                config->pincount == 16 &&
                config->address < 8);

  config->set_nreset_pin(true);

#ifdef CONFIG_MCP23SXX_MULTIPLE
  /* Allocate the device state structure */

  mcpdev = (FAR struct mcp23sxx_dev_s *)kmm_zalloc
                                            (sizeof(struct mcp23sxx_dev_s));
  if (!mcpdev)
    {
      return NULL;
    }

  /* And save the device structure in the list of MCP23SXX so that we can
   * find it later.
   */

  mcpdev->flink = g_mcp23sxxlist;
  g_mcp23sxxlist = mcpdev;

  if (mcpdev->flink)
    {
      mcpdev->index = mcpdev->flink->index + 1;
    }
  else
    {
      mcpdev->index = 0;
    }

#else
  /* Use the one-and-only MCP23SXX driver instance */

  mcpdev = &g_mcp23sxx;
  mcpdev->index = 0;
#endif

  /* Initialize the device state structure */

  mcpdev->spi     = spidev;
  mcpdev->dev.ops = &g_mcp23sxx_ops;
  mcpdev->config  = config;

#ifdef CONFIG_MCP23SXX_INT_ENABLE
  DEBUGASSERT(mcpdev->config->attach != NULL &&
                mcpdev->config->enable != NULL);

  mcpdev->config->attach(mcpdev->config, mcp23sxx_interrupt, mcpdev);
  mcpdev->config->enable(mcpdev->config, TRUE);
#endif

  nxsem_init(&mcpdev->exclsem, 0, 1);
  return &mcpdev->dev;
}

#endif /* CONFIG_IOEXPANDER_MCP23SXX */
