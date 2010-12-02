/* 
 * linux/arch/arm/boot/compressed/head-mx21ads.S
 * 
 * Copyright (C) 2006 Jay Monkman <jtm@lopingdog.com>
 * 
 * Some code borrowed from Nicolas Pitre's 'head-sa1100.S' file. This
 * is merged with head.S by the linker.
 */

#include <linux/config.h>
#include <asm/mach-types.h>

#ifndef CONFIG_MACH_MX21ADS
#error What am I doing here...
#endif

	.section        ".start", "ax"

__MX21ADS_start:
	b	4f
1:	.word	MACH_TYPE_MX21    /* Machine type */
2:	.word   0x10027000        /* PLL regs phys addr */
3:	.word   0x2e000000        /* Value for PCCR1 - enable GPT clocks */

4:
	ldr	r1, 2b
	ldr	r2, 3b
	str	r2, [r1, #36]    /* Write to PCCR1 */

	ldr	r7, 1b           /* Set machine type */

