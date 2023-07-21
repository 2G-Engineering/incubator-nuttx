/*
 * lpc17_40_boot_safety.h
 *
 *  Created on: Jun 27, 2023
 *      Author: jlange
 *
 *      Checks if the code is running on the CPU type for which it was built.
 *      If not, jumps to the ISP bootloader so there is a chance to load the correct firmware.
 */

#ifndef NUTTX_ARCH_ARM_SRC_LPC17XX_40XX_LPC17_40_BOOT_SAFETY_H_
#define NUTTX_ARCH_ARM_SRC_LPC17XX_40XX_LPC17_40_BOOT_SAFETY_H_

void lpc17_40_check_cpu_type(void);

#endif /* NUTTX_ARCH_ARM_SRC_LPC17XX_40XX_LPC17_40_BOOT_SAFETY_H_ */
