/****************************************************************************
 * drivers/ioexpander/mcp23sxx.h
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

#ifndef __DRIVERS_IOEXPANDER_MCP23SXX_H
#define __DRIVERS_IOEXPANDER_MCP23SXX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/semaphore.h>
#include <nuttx/wqueue.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/ioexpander/mcp23sxx.h>

#include <nuttx/spi/spi.h>
#include <nuttx/irq.h>

#if defined(CONFIG_IOEXPANDER) && defined(CONFIG_IOEXPANDER_MCP23SXX)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Prerequisites:
 *   CONFIG_SPI
 *     SPI support is required
 *   CONFIG_IOEXPANDER
 *     Enables support for the MCP23SXX I/O expander
 *
 * CONFIG_IOEXPANDER_MCP23SXX
 *   Enables support for the MCP23SXX driver (Needs CONFIG_INPUT)
 * CONFIG_MCP23SXX_MULTIPLE
 *   Can be defined to support multiple MCP23SXX devices on board.
 * CONFIG_MCP23SXX_INT_NCALLBACKS
 *   Maximum number of supported pin interrupt callbacks.
 */

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
#  ifndef CONFIG_MCP23SXX_INT_NCALLBACKS
#    define CONFIG_MCP23SXX_INT_NCALLBACKS 4
#  endif
#endif

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
#  ifndef CONFIG_SCHED_WORKQUEUE
#    error Work queue support required.  CONFIG_SCHED_WORKQUEUE must be selected.
#  endif
#endif

#undef CONFIG_MCP23SXX_REFCNT

/* MCP23SXX Resources ********************************************************/

#ifndef CONFIG_SPI
#error "CONFIG_SPI is required by mcp23sxx"
#endif

#define MCP23SXX_MAXDEVS             8


/* MCP23SXX Registers ********************************************************/

/* POR of the MCP23SXX sets BANK0 for the address scheme */
#define MCP23SXX_USE_BANK0

#ifdef MCP23SXX_USE_BANK0
    #define MCP23SXX_IODIRA      (0x00)
    #define MCP23SXX_IODIRB      (0x01)
    #define MCP23SXX_IPOLA       (0x02)
    #define MCP23SXX_IPOLB       (0x03)
    #define MCP23SXX_GPINTENA    (0x04)
    #define MCP23SXX_GPINTENB    (0x05)
    #define MCP23SXX_DEFVALA     (0x06)
    #define MCP23SXX_DEFVALB     (0x07)
    #define MCP23SXX_INTCONA     (0x08)
    #define MCP23SXX_INTCONB     (0x09)
    #define MCP23SXX_IOCONA      (0x0A)
    #define MCP23SXX_IOCONB      (0x0B)
    #define MCP23SXX_GPPUA       (0x0C)
    #define MCP23SXX_GPPUB       (0x0D)
    #define MCP23SXX_INTFA       (0x0E)
    #define MCP23SXX_INTFB       (0x0F)
    #define MCP23SXX_INTCAPA     (0x10)
    #define MCP23SXX_INTCAPB     (0x11)
    #define MCP23SXX_GPIOA       (0x12)
    #define MCP23SXX_GPIOB       (0x13)
    #define MCP23SXX_OLATA       (0x14)
    #define MCP23SXX_OLATB       (0x15)

    #define MCP23SXX_NUMREGS     (0x16)
#elif defined(MCP23SXX_USE_BANK1)
    #define MCP23SXX_IODIRA      (0x00)
    #define MCP23SXX_IPOLA       (0x01)
    #define MCP23SXX_GPINTENA    (0x02)
    #define MCP23SXX_DEFVALA     (0x03)
    #define MCP23SXX_INTCONA     (0x04)
    #define MCP23SXX_IOCONA      (0x05)
    #define MCP23SXX_GPPUA       (0x06)
    #define MCP23SXX_INTFA       (0x07)
    #define MCP23SXX_INTCAPA     (0x08)
    #define MCP23SXX_GPIOA       (0x09)
    #define MCP23SXX_OLATA       (0x0A)
    #define MCP23SXX_IODIRB      (0x10)
    #define MCP23SXX_IPOLB       (0x11)
    #define MCP23SXX_GPINTENB    (0x12)
    #define MCP23SXX_DEFVALB     (0x13)
    #define MCP23SXX_INTCONB     (0x14)
    #define MCP23SXX_IOCONB      (0x15)
    #define MCP23SXX_GPPUB       (0x16)
    #define MCP23SXX_INTFB       (0x17)
    #define MCP23SXX_INTCAPB     (0x18)
    #define MCP23SXX_GPIOB       (0x19)
    #define MCP23SXX_OLATB       (0x1A)

    #define MCP23SXX_NUMREGS     (0x1B)
#endif

#define MCP23SXX_IOCON_INTPOL    (1 << 1)
#define MCP23SXX_IOCON_ODR       (1 << 2)
#define MCP23SXX_IOCON_HAEN      (1 << 3)
#define MCP23SXX_IOCON_DISSLW    (1 << 4)
#define MCP23SXX_IOCON_SEQOP     (1 << 5)
#define MCP23SXX_IOCON_MIRROR    (1 << 6)
#define MCP23SXX_IOCON_BANK      (1 << 7)

#define MCP23SXX_CONST_ADDR      (0b01000000)
#define MCP23SXX_READ            (0b1)
#define MCP23SXX_WRITE           (0b0)
#define MCP23SXX_ADDR_MASK       (0b00001110)


/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
/* This type represents on registered pin interrupt callback */

struct mcp23sxx_callback_s
{
  ioe_pinset_t pinset;        /* Set of pin interrupts that will generate
                                * the callback. */
  ioe_callback_t cbfunc;      /* The saved callback function pointer */
  FAR void *cbarg;            /* Callback argument */
};
#endif

/* This structure represents the state of the MCP23SXX driver */

struct mcp23sxx_dev_s
{
  struct ioexpander_dev_s      dev;     /* Nested structure to allow casting
                                         * as public gpio expander. */
#ifdef CONFIG_MCP23SXX_SHADOW_MODE
  uint8_t                      sreg[MCP23SXX_NUMREGS]; /* Shadowed registers of the MCP23SXX */
  uint16_t                     index;   /* Device number if multiple */
#endif
#ifdef CONFIG_MCP23SXX_MULTIPLE
  FAR struct mcp23sxx_dev_s    *flink;  /* Supports a singly linked list of drivers */
#endif
  FAR const struct mcp23sxx_config_s *config; /* Board configuration data */
  FAR struct spi_dev_s        *spi;     /* Saved SPI driver instance */
  sem_t                        exclsem; /* Mutual exclusion */

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  struct work_s work;                   /* Supports the interrupt handling "bottom half" */

  /* Saved callback information for each I/O expander client */

  struct mcp23sxx_callback_s cb[CONFIG_MCP23SXX_INT_NCALLBACKS];
#endif
};

#endif /* CONFIG_IOEXPANDER && CONFIG_IOEXPANDER_MCP23SXX */
#endif /* __DRIVERS_IOEXPANDER_MCP23SXX_H */
