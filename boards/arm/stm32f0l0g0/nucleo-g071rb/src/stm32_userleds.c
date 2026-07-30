/****************************************************************************
 * boards/arm/stm32f0l0g0/nucleo-g071rb/src/stm32_userleds.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include <nuttx/board.h>

#include "stm32_gpio.h"
#include "nucleo-g071rb.h"

#include <arch/board/board.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

uint32_t board_userled_initialize(void)
{
  stm32_configgpio(GPIO_LED1);
  return BOARD_NLEDS;
}

void board_userled(int led, bool ledon)
{
  if ((unsigned)led < BOARD_NLEDS)
    {
      stm32_gpiowrite(GPIO_LED1, ledon);
    }
}

void board_userled_all(uint32_t ledset)
{
  stm32_gpiowrite(GPIO_LED1, (ledset & BOARD_LED1_BIT) != 0);
}