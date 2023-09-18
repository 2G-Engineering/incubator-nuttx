/*
 * lpc17_40_boot_safety.c
 *
 *  Created on: Jun 27, 2023
 *      Author: jlange
 */

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/init.h>

#include <arch/irq.h>

#include "arm_internal.h"
#include "nvic.h"

#include "lpc17_40_clockconfig.h"
#include "lpc17_40_lowputc.h"
#include "lpc17_40_userspace.h"
#include "lpc17_40_start.h"
#include "lpc17_40_progmem.h"
#include "lpc17_40_boot_safety.h"

#define array_size(a) (sizeof(a) / sizeof(a[0]))

#define PART_ID_LPC4088     0x481D3F47
#define PART_ID_LPC4078     0x47193F47
#define PART_ID_LPC4076     0x47191F43
#define PART_ID_LPC4074     0x47011132

#define PART_ID_LPC1788     0x281D3F47
#define PART_ID_LPC1787     0x281D3747
#define PART_ID_LPC1786     0x281D1F43
#define PART_ID_LPC1785     0x281D1743
#define PART_ID_LPC1778     0x27193F47
#define PART_ID_LPC1777     0x27193747
#define PART_ID_LPC1776     0x27191F43
#define PART_ID_LPC1774     0x27011132

#define PART_ID_LPC1769     0x26113F37
#define PART_ID_LPC1768     0x26013F37
#define PART_ID_LPC1767     0x26012837
#define PART_ID_LPC1766     0x26013F33
#define PART_ID_LPC1765     0x26013733
#define PART_ID_LPC1764     0x26011922
#define PART_ID_LPC1759     0x25113737
#define PART_ID_LPC1758     0x25013F37
#define PART_ID_LPC1756     0x25011723
#define PART_ID_LPC1754     0x25011722
#define PART_ID_LPC1752     0x25001121
#define PART_ID_LPC1751     0x25001118
#define PART_ID_LPC1751_ALT 0x25001110

#if defined(CONFIG_ARCH_FAMILY_LPC407X) || defined(CONFIG_ARCH_FAMILY_LPC408X)
static const uint32_t part_ids_lpc40xx[] = {
    PART_ID_LPC4088,
    PART_ID_LPC4078,
    PART_ID_LPC4076,
    PART_ID_LPC4074,
};
#endif

#if defined(CONFIG_ARCH_FAMILY_LPC177X) || defined(CONFIG_ARCH_FAMILY_LPC178X)
static const uint32_t part_ids_lpc177x_178x[] = {
    PART_ID_LPC1788,
    PART_ID_LPC1787,
    PART_ID_LPC1786,
    PART_ID_LPC1785,
    PART_ID_LPC1778,
    PART_ID_LPC1777,
    PART_ID_LPC1776,
    PART_ID_LPC1774,
};
#endif

#if defined(CONFIG_ARCH_FAMILY_LPC175X) || defined(CONFIG_ARCH_FAMILY_LPC176X)
static const uint32_t part_ids_lpc175x_176x[] = {
    PART_ID_LPC1769,
    PART_ID_LPC1768,
    PART_ID_LPC1767,
    PART_ID_LPC1766,
    PART_ID_LPC1765,
    PART_ID_LPC1764,
    PART_ID_LPC1759,
    PART_ID_LPC1758,
    PART_ID_LPC1756,
    PART_ID_LPC1754,
    PART_ID_LPC1752,
    PART_ID_LPC1751,
    PART_ID_LPC1751_ALT,
};
#endif

/* This particular function is only called immediately after boot, right after interrupts have been disabled.
 * This means that there is no need ot manually disable interrupts as is done in e.g. the progmem driver
 */
static inline void lpc17_40_iap(void *in, void *out)
{
  ((void (*)(void *, void *))LPC17_40_IAP_ENTRY_ADDR)(in, out);
}

void lpc17_40_check_cpu_type(void) {
    uint32_t inout[2];

    inout[0] = LPC17_40_IAP_CMD_READ_PART_ID;

    lpc17_40_iap(inout, inout);

    //compare returned part ID to known list of part IDs and this firmware's build config
    //only build in the checks for the expected CPU subfamily; we'll fail all the other checks anyway
#if defined(CONFIG_ARCH_FAMILY_LPC407X) || defined(CONFIG_ARCH_FAMILY_LPC408X)
    for (int i = 0; i < array_size(part_ids_lpc40xx); i+= 1) {
        if (inout[1] == part_ids_lpc40xx[i]) {
            //CPU type is in the correct family, return and continue with the normal boot process
            return;
        }
    }
#elif defined(CONFIG_ARCH_FAMILY_LPC177X) || defined(CONFIG_ARCH_FAMILY_LPC178X)
    for (int i = 0; i < array_size(part_ids_lpc177x_178x); i+= 1) {
        if (inout[1] == part_ids_lpc177x_178x[i]) {
            //CPU type is in the correct family, return and continue with the normal boot process
            return;
        }
    }
#elif defined(CONFIG_ARCH_FAMILY_LPC175X) || defined(CONFIG_ARCH_FAMILY_LPC176X)
    for (int i = 0; i < array_size(part_ids_lpc175x_176x); i+= 1) {
        if (inout[1] == part_ids_lpc175x_176x[i]) {
            //CPU type is in the correct family, return and continue with the normal boot process
            return;
        }
    }
#else
#error Unknown LPC17_40 CPU subfamily!
#endif
    //If we get this far, we've failed the CPU type check, go directly to the ISP bootloader
    inout[0] = LPC17_40_IAP_CMD_REINVOKE_ISP;
    lpc17_40_iap(inout, inout);
    //will not return from here
}
