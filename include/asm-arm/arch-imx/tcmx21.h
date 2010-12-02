/*
 * linux/include/asm-arm/arch-imx21/mx21ads.h
 *
 * Copyright (c) 2006 Loping Dog Embedded Systems
 * Written by Jay Monkman <jtm@lopingdog.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef __ASM_ARCH_TCMX21_H
#define __ASM_ARCH_TCMX21_H

/* ------------------------------------------------------------------------ */
/* Memory Map for the Turbochef MX21 Board                               */
/* ------------------------------------------------------------------------ */

#define MX21ADS_FLASH_PHYS		0x10000000
#define MX21ADS_FLASH_SIZE		(16*1024*1024)

#define IMX_FB_PHYS			(0x0C000000 - 0x40000)

#define CLK32 32768

#define TCMX21_ETH_VIRT 0xea000000
#define TCMX21_ETH_PHYS 0xcc000000
#define TCMX21_ETH_SIZE 0x00200000
#define TCMX21_ETH_IRQ  IRQ_GPIOE(11)

#define TCMX21_UART_VIRT 0xea200000
#define TCMX21_UART_PHYS 0xcc200000
#define TCMX21_UART_SIZE 0x00200000

#define TCMX21_IN0_VIRT 0xea400000
#define TCMX21_IN0_PHYS 0xcc400000
#define TCMX21_IN0_SIZE 0x00200000

#define TCMX21_IN1_VIRT 0xea600000
#define TCMX21_IN1_PHYS 0xcc600000
#define TCMX21_IN1_SIZE 0x00200000

#define TCMX21_IO0_VIRT 0xea800000
#define TCMX21_IO0_PHYS 0xcc800000
#define TCMX21_IO0_SIZE 0x00200000

#define TCMX21_IO1_VIRT 0xeaa00000
#define TCMX21_IO1_PHYS 0xcca00000
#define TCMX21_IO1_SIZE 0x00200000

#endif /* __ASM_ARCH_TCMX21_H */
