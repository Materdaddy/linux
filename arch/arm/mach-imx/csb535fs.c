/*
 * arch/arm/mach-imx21/csb535fs.c
 *
 * Initially based on:
 *	linux-2.6.7-imx/arch/arm/ mach-imx /scb9328.c
 *	Copyright (c) 2004 Sascha Hauer <sascha@saschahauer.de>
 *
 * 2004 (c) MontaVista Software, Inc.
 *
 * Modified By: Ron Melvin (ron.melvin@timesys.com)
 * Copyright (c) 2005 TimeSys Corporation 
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
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
#include "generic.h"
#include <asm/serial.h>
#include <asm/arch/mmc.h>

#if defined (CONFIG_USB_IMX21_HCD)
#include <asm/arch/imx21-hcd.h>
#endif

#if defined (CONFIG_FB_IMX)
#include <asm/arch/imxfb.h>
static void csb535_lcd_power(int on);

static struct imxfb_mach_info csb535_fb_info __initdata = {
    .pixclock     = 62500,
    .bpp          = 16,
    .xres         = 240,
    .yres         = 320,
    .hsync_len    = 0x3e,
    .vsync_len    = 1,
    .left_margin  = 0x12,
    .upper_margin = 0,
    .right_margin = 0x39,
    .lower_margin = 3,
    
    .pcr = (PCR_COLOR    | 
            PCR_TFT      | 
            PCR_FLMPOL   | 
            PCR_LPPOL    | 
            PCR_CLKPOL   |
            PCR_PBSIZ_8  |
            PCR_BPIX_16  | 
            PCR_SCLK_SEL |
            PCR_PCD(3)),
/*    .pwmr     = PWMR_SCR0 | PWMR_CC_EN | PWMR_PW(0x70), */
    .pwmr       = 0, /* default value */
    .lscr1      = 0x400c0373, /* reset default */
    
    .lcd_power = csb535_lcd_power,
};

static void csb535_lcd_power(int on)
{
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
}

#endif /* defined(CONFIG_FB_IMX) */

#if defined(CONFIG_CIRRUS)
static struct resource cs8900_resources[] = {
    [0] = {
        .start  = CSB535_ETH_PHYS,
        .end    = CSB535_ETH_PHYS + CSB535_ETH_SIZE - 1,
        .flags  = IORESOURCE_MEM,
    },
    [1] = {
        .start  = CSB535_ETH_IRQ,
        .end    = CSB535_ETH_IRQ,
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

static int csb535_mmc_card_present(void)
{

    if (SSR(3) & (1 << 25)) {
        return 0;
    } else {
        return 1;
    }
}

static void csb535_mmc_setpower(struct device *dev, unsigned int vdd)
{
    if (vdd) {
//        printk("%s: on\n", __FUNCTION__);
        DR(3) |= (1 << 27);                /* Set SD_VEN high */
    } else {
//        printk("%s: off\n", __FUNCTION__);
        DR(3) &= ~(1 << 27);               /* Set SD_VEN low */
    }
}

static struct imxmmc_platform_data csb535_mmc_info = {
    .card_present   = csb535_mmc_card_present,
    .setpower      = csb535_mmc_setpower,
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
csb535_init(void)
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
    
    set_imx_fb_info(&csb535_fb_info);
    csb535_lcd_power(0);
#endif

    /* For MMC */
    imx_gpio_mode(27 | GPIO_PORTD | GPIO_GPIO | GPIO_OUT);      /* MMC_VEN */
    imx_gpio_mode(26 | GPIO_PORTD | GPIO_GPIO | GPIO_IN);       /* WP */
    imx_gpio_mode(25 | GPIO_PORTD | GPIO_GPIO | GPIO_IN);       /* CD */
    DR(3) &= ~(1 << 27);                               /* Turn off MMC_VEN */

#if defined(CONFIG_CIRRUS)
    imx_gpio_mode(GPIO_PORTD | 31 | GPIO_IN);
#endif
	
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
    imx_gpio_mode(25 | GPIO_PORTB );   /* USB Host Suspend */

    imx_gpio_mode(22 | GPIO_PORTB | GPIO_OUT | GPIO_GPIO); /* USB Host Mode */

    imx_gpio_mode(13 | GPIO_PORTC );    /* USB Func RX DP */
    imx_gpio_mode(12 | GPIO_PORTC );    /* USB Func RX DM */
    imx_gpio_mode(11 | GPIO_PORTC );   /* USB Func TX DP */
    imx_gpio_mode(10 | GPIO_PORTC );   /* USB Func TX DM */
    imx_gpio_mode( 9 | GPIO_PORTC );   /* USB Func OE */
    imx_gpio_mode( 8 | GPIO_PORTC );   /* USB Func Speed */
    imx_gpio_mode( 7 | GPIO_PORTC );   /* USB Func Suspend */

    imx_gpio_mode( 6 | GPIO_PORTC | GPIO_OUT | GPIO_GPIO); /* USB Func Mode */
    imx_gpio_mode( 5 | GPIO_PORTC | GPIO_OUT | GPIO_GPIO); /* USB Func En */

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
    DR(1) |= (1 << 22);

    /* Set USB Func xcvr in SEO mode */
    DR(2) &= ~(1 << 6);

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

    imx_set_mmc_info(&csb535_mmc_info);
    platform_add_devices(devices, ARRAY_SIZE(devices));
}

static struct map_desc csb535_io_desc[] __initdata = {
    /* virtual     physical    length      type */
    {IMX_CS0_VIRT, __phys_to_pfn(IMX_CS0_PHYS), IMX_CS0_SIZE, MT_DEVICE},
    {IMX_CS1_VIRT, __phys_to_pfn(IMX_CS1_PHYS), IMX_CS1_SIZE, MT_DEVICE},
};

static void __init
csb535_map_io(void)
{
    imx21_map_io();
    iotable_init(csb535_io_desc, ARRAY_SIZE(csb535_io_desc));
}

MACHINE_START(CSB535, "Cogent CSB535")
/*	MAINTAINER("Jay Monkman <jtm@lopingdog.com>") */
	.phys_io	= 0x00200000,
	.io_pg_offst	= ((0xe0200000) >> 18) & 0xfffc,
	.boot_params	= 0xc0000000,

	.map_io		= csb535_map_io,
	.init_irq	= imx21_init_irq,
	.timer		= &imx_timer,
	.init_machine	= csb535_init,
MACHINE_END
