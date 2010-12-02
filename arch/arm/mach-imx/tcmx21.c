/*
 * arch/arm/mach-imx21/tcmx21.c
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

#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/init.h>
#include <asm/system.h>
#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/pgtable.h>
#include <asm/page.h>

#include <asm/mach/map.h>
#include <asm/mach-types.h>

#include <asm/mach/arch.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include "generic.h"
#include <asm/serial.h>
#include <asm/arch/mmc.h>

#if defined (CONFIG_USB_IMX21_HCD)
#include <asm/arch/imx21-hcd.h>
#endif

#if defined (CONFIG_FB_IMX)
#include <asm/arch/imxfb.h>
static void tcmx21_lcd_power(int on);

static struct imxfb_mach_info tcmx21_fb_info __initdata = {
    .pixclock     = 62500,
    .bpp          = 32,
    .xres         = 640,
    .yres         = 240,
    .hsync_len    = 1,
    .vsync_len    = 1,
    .left_margin  = 320,
//    .left_margin  = 0,
    .upper_margin = 1,
    .right_margin = 1,
//    .right_margin = 240,
    .lower_margin = 10,
    
    .pcr = (PCR_COLOR    | 
            PCR_TFT      | 
            PCR_CLKPOL   |
            PCR_SCLKIDLE |
            PCR_BPIX_18  | 
            PCR_SCLK_SEL |
            PCR_END_SEL  |
//            PCR_REV_VS   |
            PCR_PCD(5)),   
    .pwmr       = 0, /* default value */
    .lscr1      = 0x400c0373, /* reset default */
    
    .lcd_power = tcmx21_lcd_power,
};

static void tcmx21_lcd_power(int on)
{
#if 0
    if(on) {
        printk("%s:%d\n", __FUNCTION__, __LINE__);
        /* set VEE line on port A */
        DR(0)   |= (1 << 30);
        /* set BKL line on port E */
        DR(4)   |= (1 << 5);

    } else {
        printk("%s:%d\n", __FUNCTION__, __LINE__);
        /* clear BKL line on port E */
        DR(4)   &= ~(1 << 5);
        /* clear VEE line on port A */
        DR(0)   &= ~(1 << 30);

    }
#endif
}

#endif /* defined(CONFIG_FB_IMX) */

#if defined(CONFIG_CIRRUS)
static struct resource cs8900_resources[] = {
    [0] = {
        .start  = TCMX21_ETH_PHYS,
        .end    = TCMX21_ETH_PHYS + TCMX21_ETH_SIZE - 1,
        .flags  = IORESOURCE_MEM,
    },
    [1] = {
        .start  = TCMX21_ETH_IRQ,
        .end    = TCMX21_ETH_IRQ,
        .flags  = IORESOURCE_IRQ,
    },
};

static struct platform_device cs8900_device = {
    .name            = "cs8900",
    .id              = 0,
    .num_resources   = ARRAY_SIZE(cs8900_resources),
    .resource        = cs8900_resources,
};
#endif /* defined(CONFIG_CIRRUS) */

extern void __init imx_set_mmc_info(struct imxmmc_platform_data *info);

static int tcmx21_mmc_card_present(void)
{

    if (SSR(3) & (1 << 25)) {
        return 0;
    } else {
        return 1;
    }
}

static struct imxmmc_platform_data tcmx21_mmc_info = {
    .card_present   = tcmx21_mmc_card_present,
};


#if defined(CONFIG_USB_IMX21_HCD)
static struct imx21_usb_platform_data csb535_usb_pdata = {
    .set_mode     = NULL,
    .set_speed    = NULL,
    .set_suspend  = NULL,
    .set_oe       = NULL,
};

static u64 imx21_usb_dmamask = 0xffffffffUL;
static struct platform_device imx_usb_hcd_device = {
    .name = "imx21-hc",
    .id   = 0,
    .dev  = {
        .platform_data = &csb535_usb_pdata,
        .dma_mask = &imx21_usb_dmamask,
        .coherent_dma_mask = 0xffffffff,
    }
};
#endif

static struct platform_device *devices[] __initdata = {
#if defined(CONFIG_CIRRUS)
	&cs8900_device,
#endif
#if defined(CONFIG_USB_IMX21_HCD)
        &imx_usb_hcd_device,
#endif
};

static void __init
tcmx21_init(void)
{
    /* Configure the system clocks */
    imx21_system_clk_init();	
	
    /* CrossBar (MAX) Settings */
    /* Slave port for SSI - DMA highest priority, ARM lowest */
    MAX_MPR(0) = 0x00103256;

    /* Slave port for EMI - LCD/SLCD highest priority, ARM lowest */
    MAX_MPR(3) = 0x00123056;	

#if defined(CONFIG_FB_IMX)
    printk("%s:%d\n", __FUNCTION__, __LINE__);
    /* Configure I/O for LCD */
    /* VEE_EN */
    imx_gpio_mode(30 | GPIO_OUT | GPIO_PORTA | GPIO_GPIO | GPIO_PUEN);
    /* PWM0 */
    imx_gpio_mode(5  | GPIO_OUT | GPIO_PORTE | GPIO_GPIO | GPIO_PUEN);
    
    set_imx_fb_info(&tcmx21_fb_info);
    tcmx21_lcd_power(0);
#endif

    /* For MMC */
    imx_gpio_mode(26 | GPIO_PORTD | GPIO_GPIO | GPIO_IN);       /* WP */
    imx_gpio_mode(25 | GPIO_PORTD | GPIO_GPIO | GPIO_IN);       /* CD */

    /* For Turbochef LCD */
    imx_gpio_mode(10 | GPIO_PORTB | GPIO_GPIO | GPIO_OUT);  /* LCD_BKL_OFF */
    imx_gpio_mode(11 | GPIO_PORTB | GPIO_GPIO | GPIO_OUT);  /* LCD_DIS_OFF */
    imx_gpio_mode(12 | GPIO_PORTB | GPIO_GPIO | GPIO_OUT);  /* LCD_STEP */
    imx_gpio_mode(13 | GPIO_PORTB | GPIO_GPIO | GPIO_OUT);  /* LCD_CS_BKL */
    imx_gpio_mode(19 | GPIO_PORTB | GPIO_GPIO | GPIO_OUT);  /* LCD_UP_DN */
    imx_gpio_mode(20 | GPIO_PORTB | GPIO_GPIO | GPIO_OUT);  /* LCD_LED1 */
    imx_gpio_mode(21 | GPIO_PORTB | GPIO_GPIO | GPIO_OUT);  /* LCD_LED2 */

    DR(1) |= (1 << 10) | (1 << 11);

    /* For Turbochef Audio amp */
    imx_gpio_mode(4 | GPIO_PORTE | GPIO_GPIO | GPIO_OUT);  /* AMP_EN */
    DR(4) |= (1 << 4);


#if defined(CONFIG_USB_IMX21_HCD)
    /* Enable the clocks to the USB OTG module */
    PCCR0 |= PCCR0_HCLK_USBOTG_EN | PCCR0_USBOTG_EN;

    /* Configure the GPIO */
    imx_gpio_mode(31 | GPIO_PORTB );    /* USB Host RX DP */
    imx_gpio_mode(30 | GPIO_PORTB );    /* USB Host RX DM */
    imx_gpio_mode(29 | GPIO_PORTB );   /* USB Host TX DP */
    imx_gpio_mode(28 | GPIO_PORTB );   /* USB Host TX DM */
    imx_gpio_mode(27 | GPIO_PORTB );   /* USB Host OE */
    imx_gpio_mode(26 | GPIO_PORTB );   /* USB Host Speed */

    imx_gpio_mode(12 | GPIO_PORTC | GPIO_OUT | GPIO_GPIO); /* USB Host Mode */
    imx_gpio_mode(13 | GPIO_PORTC | GPIO_OUT | GPIO_GPIO); /* USB Host Susp. */

    imx_gpio_mode(15 | GPIO_PORTB | GPIO_OUT | GPIO_GPIO); /* RESET_USB_B */

    /* Reset the USB OTG module */
    USBOTG_RST_CTRL = (USBOTG_RST_RSTCTRL |
                       USBOTG_RST_RSTFC   |
                       USBOTG_RST_RSTFSKE |
                       USBOTG_RST_RSTRH   |
                       USBOTG_RST_RSTHSIE |
                       USBOTG_RST_RSTHC);

    /* Wait for reset to finish */
    while (USBOTG_RST_CTRL != 0) {
        schedule_timeout(1);
    }

    /* Enable clock to the host controller */
    USBOTG_CLK_CTRL = USBOTG_CLK_CTRL_HST | USBOTG_CLK_CTRL_MAIN;

    /* Set USB Host xcvr in Differential mode */
    DR(2) |= (1 << 12);

    /* Make sure transceiver is not suspended */
    DR(2) &= ~(1 << 13);

    /* Reset the hub */
    DR(1) &= ~(1 << 15);
    udelay(1000);
    DR(1) |= (1 << 15);

    /*
     * Configure the USBOTG module:
     *   Host xcvr single ended TX, diff RX
     *   USBOTG xcvr single ended TX, diff RX
     *   USBOTG in func mode
     */
    USBOTG_HWMODE = (USBOTG_HWMODE_OTGXCVR_TS_RD |
                     USBOTG_HWMODE_HOSTXCVR_TD_RD |
                     USBOTG_HWMODE_CRECFG_FUNC);

#endif

    imx_set_mmc_info(&tcmx21_mmc_info);
    platform_add_devices(devices, ARRAY_SIZE(devices));
}

static struct map_desc tcmx21_io_desc[] __initdata = {
    {
        .virtual = IMX_CS0_VIRT,
        .pfn     = __phys_to_pfn(IMX_CS0_PHYS),
        .length  = IMX_CS0_SIZE,
        .type    =  MT_DEVICE
    },{
        .virtual = TCMX21_ETH_VIRT,
        .pfn     = __phys_to_pfn(TCMX21_ETH_PHYS),
        .length  = TCMX21_ETH_SIZE,
        .type    =  MT_DEVICE
    },{
        .virtual = TCMX21_UART_VIRT,
        .pfn     = __phys_to_pfn(TCMX21_UART_PHYS),
        .length  = TCMX21_UART_SIZE,
        .type    =  MT_DEVICE
    },{
        .virtual = TCMX21_IN0_VIRT,
        .pfn     = __phys_to_pfn(TCMX21_IN0_PHYS),
        .length  = TCMX21_IN0_SIZE,
        .type    =  MT_DEVICE
    },{
        .virtual = TCMX21_IN1_VIRT,
        .pfn     = __phys_to_pfn(TCMX21_IN1_PHYS),
        .length  = TCMX21_IN1_SIZE,
        .type    =  MT_DEVICE
    },{
        .virtual = TCMX21_IO0_VIRT,
        .pfn     = __phys_to_pfn(TCMX21_IO0_PHYS),
        .length  = TCMX21_IO0_SIZE,
        .type    =  MT_DEVICE
    },{
        .virtual = TCMX21_IO1_VIRT,
        .pfn     = __phys_to_pfn(TCMX21_IO1_PHYS),
        .length  = TCMX21_IO1_SIZE,
        .type    =  MT_DEVICE
    },
};

static void __init
tcmx21_map_io(void)
{
    imx21_map_io();
    iotable_init(tcmx21_io_desc, ARRAY_SIZE(tcmx21_io_desc));
}

MACHINE_START(MX21, "Freescale TCMX21")
/*	MAINTAINER("Jay Monkman <jtm@lopingdog.com>") */
	.phys_io	= 0x00200000,
	.io_pg_offst	= ((0xe0200000) >> 18) & 0xfffc,
	.boot_params	= 0xc0000000,

	.map_io		= tcmx21_map_io,
	.init_irq	= imx21_init_irq,
	.timer		= &imx_timer,
	.init_machine	= tcmx21_init,
MACHINE_END
