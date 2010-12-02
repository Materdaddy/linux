/*
 *  linux/include/asm-arm/arch-imxads/irqs.h
 *
 *  Copyright (C) 1999 ARM Limited
 *  Copyright (C) 2000 Deep Blue Solutions Ltd.
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

#ifndef __ARM_IRQS_H__
#define __ARM_IRQS_H__

/* Use the imx definitions */
#include <asm/hardware.h>

/*
 *  IMX Interrupt numbers
 *
 */
#if defined(CONFIG_ARCH_IMX)

#define INT_SOFTINT                 0
#define CSI_INT                     6
#define DSPA_MAC_INT                7
#define DSPA_INT                    8
#define COMP_INT                    9
#define MSHC_XINT                   10
#define GPIO_INT_PORTA              11
#define GPIO_INT_PORTB              12
#define GPIO_INT_PORTC              13
#define LCDC_INT                    14
#define SIM_INT                     15
#define SIM_DATA_INT                16
#define RTC_INT                     17
#define RTC_SAMINT                  18
#define UART2_MINT_PFERR            19
#define UART2_MINT_RTS              20
#define UART2_MINT_DTR              21
#define UART2_MINT_UARTC            22
#define UART2_MINT_TX               23
#define UART2_MINT_RX               24
#define UART1_MINT_PFERR            25
#define UART1_MINT_RTS              26
#define UART1_MINT_DTR              27
#define UART1_MINT_UARTC            28
#define UART1_MINT_TX               29
#define UART1_MINT_RX               30
#define VOICE_DAC_INT               31
#define VOICE_ADC_INT               32
#define PEN_DATA_INT                33
#define PWM_INT                     34
#define SDHC_INT                    35
#define INT_I2C                     39
#define INT_CSPI2                   40
#define INT_CSPI1                   41
#define SSI_TX_INT                  42
#define SSI_TX_ERR_INT              43
#define SSI_RX_INT                  44
#define SSI_RX_ERR_INT              45
#define TOUCH_INT                   46
#define USBD_INT0                   47
#define USBD_INT1                   48
#define USBD_INT2                   49
#define USBD_INT3                   50
#define USBD_INT4                   51
#define USBD_INT5                   52
#define USBD_INT6                   53
#define BTSYS_INT                   55
#define BTTIM_INT                   56
#define BTWUI_INT                   57
#define TIM2_INT                    58
#define TIM1_INT                    59
#define DMA_ERR                     60
#define DMA_INT                     61
#define GPIO_INT_PORTD              62

#elif defined(CONFIG_ARCH_IMX21)

#define INT_CSPI3		    6
#define INT_SRCL		    7
#define INT_GPIO                    8
#define INT_FIRI                    9
#define INT_SDHC2                   10
#define INT_SDHC1                   11
#define INT_I2C                     12
#define INT_SSI2                    13
#define INT_SSI1                    14
#define INT_CSPI2                   15
#define INT_CSPI1                   16
#define INT_UART4                   17
#define INT_UART3                   18
#define INT_UART2                   19
#define INT_UART1                   20
#define INT_KPP_TX                  21
#define INT_RTC_RX                  22
#define INT_PWM                     23
#define TIM3_INT                    24
#define TIM2_INT                    25
#define TIM1_INT                    26
#define INT_WDOG                    27
#define INT_PCMCIA                  28
#define INT_NFC                     29
#define INT_BMI                     30
#define INT_CSI                     31
#define INT_DMACH0                  32
#define INT_DMACH1                  33
#define INT_DMACH2                  34
#define INT_DMACH3                  35
#define INT_DMACH4                  36
#define INT_DMACH5                  37
#define INT_DMACH6                  38
#define INT_DMACH7                  39
#define INT_DMACH8                  40
#define INT_DMACH9                  41
#define INT_DMACH10                 42
#define INT_DMACH11                 43
#define INT_DMACH12                 44
#define INT_DMACH13                 45
#define INT_DMACH14                 46
#define INT_DMACH15                 47
#define DMA_INT(x)                  (INT_DMACH0 + ((x) & 0xf))
#define INT_EMMAENC                 49
#define INT_EMMADEC                 50
#define INT_EMMAPRP                 51
#define INT_EMMAPP                  52
#define INT_USBWKUP                 53
#define INT_USBDMA                  54
#define INT_USBHOST                 55
#define INT_USBFUNC                 56
#define INT_USBMNP                  57
#define INT_USBCTRL                 58
#define INT_SAHARA                  59
#define INT_SLCDC                   60
#define INT_LCDC                    61

#else
#error No CONFIG_ARCH_ defined
#endif

#define IMX_IRQS                         (64)

/* note: the IMX has four gpio ports (A-D), but only
 *       the following pins are connected to the outside
 *       world:
 *
 * PORT A: bits 0-31
 * PORT B: bits 8-31
 * PORT C: bits 3-17
 * PORT D: bits 6-31
 *
 * We map these interrupts straight on. As a result we have
 * several holes in the interrupt mapping. We do this for two
 * reasons:
 *   - mapping the interrupts without holes would get
 *     far more complicated
 *   - Motorola could well decide to bring some processor
 *     with more pins connected
 */

#define IRQ_GPIOA(x)  (IMX_IRQS + x)
#define IRQ_GPIOB(x)  (IRQ_GPIOA(32) + x)
#define IRQ_GPIOC(x)  (IRQ_GPIOB(32) + x)
#define IRQ_GPIOD(x)  (IRQ_GPIOC(32) + x)

#if defined(CONFIG_ARCH_IMX)
# define NR_IRQS (IRQ_GPIOD(32) + 1)
#elif defined(CONFIG_ARCH_IMX21)
# define IRQ_GPIOE(x)  (IRQ_GPIOD(32) + x)
# define IRQ_GPIOF(x)  (IRQ_GPIOE(32) + x)
# define NR_IRQS (IRQ_GPIOF(32) + 1)
#endif

/* decode irq number to use with IMR(x), ISR(x) and friends */
#define IRQ_TO_REG(irq) ((irq - IMX_IRQS) >> 5)

#define IRQ_GPIO(x)
#endif
