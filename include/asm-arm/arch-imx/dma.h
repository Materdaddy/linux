/*
 *  linux/include/asm-arm/imxads/dma.h
 *
 *  Copyright (C) 1997,1998 Russell King
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
 */

#ifndef __ASM_ARCH_DMA_H
#define __ASM_ARCH_DMA_H

typedef enum {
#if defined(CONFIG_ARCH_IMX)
	DMA_PRIO_HIGH = 0,
	DMA_PRIO_MEDIUM = 1,
	DMA_PRIO_LOW = 2
#elif  defined(CONFIG_ARCH_IMX21)
	DMA_PRIO_HIGH = 2,
	DMA_PRIO_MEDIUM = 3,
	DMA_PRIO_LOW = 6
#else
#error No CONFIG_ARCH_ defined
#endif
} imx_dma_prio;

#if defined(CONFIG_ARCH_IMX)

# define DMA_REQ_UART3_T        2
# define DMA_REQ_UART3_R        3
# define DMA_REQ_SSI2_T         4
# define DMA_REQ_SSI2_R         5
# define DMA_REQ_CSI_STAT       6
# define DMA_REQ_CSI_R          7
# define DMA_REQ_MSHC           8
# define DMA_REQ_DSPA_DCT_DOUT  9
# define DMA_REQ_DSPA_DCT_DIN  10
# define DMA_REQ_DSPA_MAC      11
# define DMA_REQ_EXT           12
# define DMA_REQ_SDHC          13
# define DMA_REQ_SPI1_R        14
# define DMA_REQ_SPI1_T        15
# define DMA_REQ_SSI_T         16
# define DMA_REQ_SSI_R         17
# define DMA_REQ_ASP_DAC       18
# define DMA_REQ_ASP_ADC       19
# define DMA_REQ_USP_EP(x)    (20+(x))
# define DMA_REQ_SPI2_R        26
# define DMA_REQ_SPI2_T        27
# define DMA_REQ_UART2_T       28
# define DMA_REQ_UART2_R       29
# define DMA_REQ_UART1_T       30
# define DMA_REQ_UART1_R       31

#elif defined(CONFIG_ARCH_IMX21)

# define DMA_REQ_CSPI3_RX       1
# define DMA_REQ_CSPI3_TX       2
# define DMA_REQ_EXT            3
# define DMA_REQ_FIRI_RX        4
# define DMA_REQ_FIRI_TX        5
# define DMA_REQ_SDHC2          6
# define DMA_REQ_SDHC1          7
# define DMA_REQ_SSI2_RX0       8
# define DMA_REQ_SSI2_TX0       9
# define DMA_REQ_SSI2_RX1      10
# define DMA_REQ_SSI2_TX1      11
# define DMA_REQ_SSI1_RX0      12
# define DMA_REQ_SSI1_TX0      13
# define DMA_REQ_SSI1_RX1      14
# define DMA_REQ_SSI1_TX1      15
# define DMA_REQ_CSPI2_RX      16
# define DMA_REQ_CSPI2_TX      17
# define DMA_REQ_CSPI1_RX      18
# define DMA_REQ_CSPI1_TX      19
# define DMA_REQ_UART4_RX      20
# define DMA_REQ_UART4_TX      21
# define DMA_REQ_UART3_RX      22
# define DMA_REQ_UART3_TX      23
# define DMA_REQ_UART2_RX      24
# define DMA_REQ_UART2_TX      25
# define DMA_REQ_UART1_RX      26
# define DMA_REQ_UART1_TX      27
# define DMA_REQ_BMI_TX        28
# define DMA_REQ_BMI_RX        29
# define DMA_REQ_CSI_STAT      30
# define DMA_REQ_CSI_RX        31

#endif /* defined(CONFIG_ARCH_IMX21) */
#endif				/* _ASM_ARCH_DMA_H */
