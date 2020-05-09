/****************************************************************************
 * include/nuttx/ioexpander/mcp23sxx.h
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

#ifndef __INCLUDE_NUTTX_IOEXPANDER_MCP23SXX_H
#define __INCLUDE_NUTTX_IOEXPANDER_MCP23SXX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/spi/spi.h>

#include <stdbool.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the mcp23sxx
 * driver when the driver is instantiated. This structure provides
 * information about the configuration of the mcp23sxx and provides some
 * board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied by
 * the driver and is presumed to persist while the driver is active. The
 * memory must be writeable because, under certain circumstances, the driver
 * may modify the frequency.
 */

struct mcp23sxx_config_s
{
  /* Device characterization */

  uint8_t pincount; /* Number of GPIO pins on the device (8 or 16) */
  uint8_t address;     /* SPI device address */
  uint32_t frequency;  /* I2C or SPI frequency */

  /* Sets the state of the MCP23SXX's nReset pin */

  CODE void (*set_nreset_pin)(bool state);

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  /* If multiple mcp23sxx devices are supported, then an IRQ number must
   * be provided for each so that their interrupts can be distinguished.
   */

  /* IRQ/GPIO access callbacks.  These operations all hidden behind
   * callbacks to isolate the mcp23sxx driver from differences in GPIO
   * interrupt handling by varying boards and MCUs.
   *
   * attach  - Attach the mcp23sxx interrupt handler to the GPIO interrupt
   * enable  - Enable or disable the GPIO interrupt
   */

  CODE int  (*attach)(FAR struct mcp23sxx_config_s *state, xcpt_t isr,
                      FAR void *arg);
  CODE void (*enable)(FAR struct mcp23sxx_config_s *state, bool enable);
#endif
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: mcp23sxx_initialize
 *
 * Description:
 *   Instantiate and configure the mcp23sxx device driver to use the provided
 *   SPI device
 *   instance.
 *
 * Input Parameters:
 *   dev     - An SPI driver instance
 *   minor   - The device address
 *   config  - Persistent board configuration data
 *
 * Returned Value:
 *   an ioexpander_dev_s instance on success, NULL on failure.
 *
 ****************************************************************************/

FAR struct ioexpander_dev_s *mcp23sxx_initialize(FAR struct spi_dev_s *dev,
                                        FAR struct mcp23sxx_config_s *config);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_IOEXPANDER_MCP23SXX_H */
