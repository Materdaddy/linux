/*
 *  arch/arm/mach-imx/generic.c
 *
 *  author: Sascha Hauer
 *  Created: april 20th, 2004
 *  Copyright: Synertronixx GmbH
 *
 *  Common code for i.MX machines
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
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <asm/arch/imxfb.h>
#include <asm/hardware.h>
#include <linux/interrupt.h>

void imx_gpio_mode(int gpio_mode)
{
	unsigned int pin = gpio_mode & GPIO_PIN_MASK;
	unsigned int port = (gpio_mode & GPIO_PORT_MASK) >> GPIO_PORT_POS;
	unsigned int ocr = (gpio_mode & GPIO_OCR_MASK) >> GPIO_OCR_POS;
	unsigned int tmp;

        if (port != (GPIO_PORTKP >> GPIO_PORT_POS)) {

            /* Pullup enable */
            if(gpio_mode & GPIO_PUEN)
		PUEN(port) |= (1<<pin);
            else
		PUEN(port) &= ~(1<<pin);
            
            /* Data direction */
            if(gpio_mode & GPIO_OUT)
		DDIR(port) |= 1<<pin;
            else
		DDIR(port) &= ~(1<<pin);
            
            /* Primary / alternate function */
            if(gpio_mode & GPIO_AF)
		GPR(port) |= (1<<pin);
            else
		GPR(port) &= ~(1<<pin);
            
            /* use as gpio? */
            if( ocr == 3 )
		GIUS(port) |= (1<<pin);
            else
		GIUS(port) &= ~(1<<pin);
            
            /* Output / input configuration */
            /* FIXME: I'm not very sure about OCR and ICONF, someone
             * should have a look over it
             */
            if(pin<16) {
		tmp = OCR1(port);
		tmp &= ~( 3<<(pin*2));
		tmp |= (ocr << (pin*2));
		OCR1(port) = tmp;
                
		if( gpio_mode &	GPIO_AOUT )
                    ICONFA1(port) &= ~( 3<<(pin*2));
		if( gpio_mode &	GPIO_BOUT )
                    ICONFB1(port) &= ~( 3<<(pin*2));
            } else {
		tmp = OCR2(port);
		tmp &= ~( 3<<((pin-16)*2));
		tmp |= (ocr << ((pin-16)*2));
		OCR2(port) = tmp;
                
		if( gpio_mode &	GPIO_AOUT )
                    ICONFA2(port) &= ~( 3<<((pin-16)*2));
		if( gpio_mode &	GPIO_BOUT )
                    ICONFB2(port) &= ~( 3<<((pin-16)*2));
            }

            if (gpio_mode & GPIO_IRQ_MASK) {
                int cfg;
                cfg = (gpio_mode & GPIO_IRQ_CFG_MASK) >> GPIO_IRQ_CFG_POS;

                /* Mask the interrupt */
                IMR(port) &= ~(1 << pin);

                if (pin < 16) {
                    tmp = ICR1(port);
                    tmp &= ~(3 << (pin * 2));
                    tmp |= cfg << (pin * 2);
                    ICR1(port) = tmp;
                } else {
                    tmp = ICR2(port);
                    tmp &= ~(3 << ((pin - 16) * 2));
                    tmp |= cfg << ((pin - 16) * 2);
                    ICR2(port) = tmp;
                }
            }

        } else { /* PORT KP */

            if (gpio_mode & GPIO_OCEN) {
                if (pin < 8) {
                    printk("Pin %d of GPIO_PORTKP"
                           " doesn't support open collector", pin);
                } else {
                    KPP_KPCR |= (1 << pin);
                }
            }
            if (gpio_mode & GPIO_OUT) {
                KPP_KDDR |= (1 << pin);
            } else {
                KPP_KDDR &= ~(1 << pin);
            }
        }
}
EXPORT_SYMBOL(imx_gpio_mode);

void imx_gpio_write(int mode,  int val)
{
    int pin  = (mode & GPIO_PIN_MASK);
    int port = (mode & GPIO_PORT_MASK) >> GPIO_PORT_POS;

    if (port != (GPIO_PORTKP >> GPIO_PORT_POS)) {
        if (val) {
            DR(port) |= (1 << pin);
        } else {
            DR(port) &= ~(1 << pin);
        }
    } else {
        if (val) {
            KPP_KPDR |= (1 << pin);
        } else {
            KPP_KPDR &= ~(1 << pin);
        }
    }
}
EXPORT_SYMBOL(imx_gpio_write);


int imx_gpio_read(int mode)
{
    int pin  = (mode & GPIO_PIN_MASK);
    int port = (mode & GPIO_PORT_MASK) >> GPIO_PORT_POS;

    if (port != (GPIO_PORTKP >> GPIO_PORT_POS)) {
        return SSR(port) & (1 << pin);
    } else {
        return KPP_KPDR & (1 << pin);
    }
}
EXPORT_SYMBOL(imx_gpio_read);


#if defined(CONFIG_ARCH_IMX)
#include <asm/mach/map.h>
#include <asm/arch/mmc.h>


/*
 *  get the system pll clock in Hz
 *
 *                  mfi + mfn / (mfd +1)
 *  f = 2 * f_ref * --------------------
 *                        pd + 1
 */
static unsigned int imx_decode_pll(unsigned int pll)
{
	u32 mfi = (pll >> 10) & 0xf;
	u32 mfn = pll & 0x3ff;
	u32 mfd = (pll >> 16) & 0x3ff;
	u32 pd =  (pll >> 26) & 0xf;
	u32 f_ref = (CSCR & CSCR_SYSTEM_SEL) ? 16000000 : (CLK32 * 512);

	mfi = mfi <= 5 ? 5 : mfi;

	return (2 * (f_ref>>10) * ( (mfi<<10) + (mfn<<10) / (mfd+1) )) / (pd+1);
}

unsigned int imx_get_system_clk(void)
{
	return imx_decode_pll(SPCTL0);
}
EXPORT_SYMBOL(imx_get_system_clk);

unsigned int imx_get_mcu_clk(void)
{
	return imx_decode_pll(MPCTL0);
}
EXPORT_SYMBOL(imx_get_mcu_clk);

/*
 *  get peripheral clock 1 ( UART[12], Timer[12], PWM )
 */
unsigned int imx_get_perclk1(void)
{
	return imx_get_system_clk() / (((PCDR) & 0xf)+1);
}
EXPORT_SYMBOL(imx_get_perclk1);

/*
 *  get peripheral clock 2 ( LCD, SD, SPI[12] )
 */
unsigned int imx_get_perclk2(void)
{
	return imx_get_system_clk() / (((PCDR>>4) & 0xf)+1);
}
EXPORT_SYMBOL(imx_get_perclk2);

/*
 *  get peripheral clock 3 ( SSI )
 */
unsigned int imx_get_perclk3(void)
{
	return imx_get_system_clk() / (((PCDR>>16) & 0x7f)+1);
}
EXPORT_SYMBOL(imx_get_perclk3);

/*
 *  get hclk ( SDRAM, CSI, Memory Stick, I2C, DMA )
 */
unsigned int imx_get_hclk(void)
{
	return imx_get_system_clk() / (((CSCR>>10) & 0xf)+1);
}
EXPORT_SYMBOL(imx_get_hclk);

static struct resource imx_mmc_resources[] = {
	[0] = {
		.start	= 0x00214000,
		.end	= 0x002140FF,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= (SDHC_INT),
		.end	= (SDHC_INT),
		.flags	= IORESOURCE_IRQ,
	},
};

static u64 imxmmc_dmamask = 0xffffffffUL;
static struct platform_device imx_mmc_device = {
    .name		= "imx-mmc",
    .id		= 0,
    .dev           = {
        .dma_mask = &imxmmc_dmamask,
        .coherent_dma_mask = 0xffffffff,
    },
    .num_resources	= ARRAY_SIZE(imx_mmc_resources),
    .resource	= imx_mmc_resources,
};

void __init imx_set_mmc_info(struct imxmmc_platform_data *info)
{
       imx_mmc_device.dev.platform_data = info;
}
EXPORT_SYMBOL(imx_set_mmc_info);

static struct resource imx_uart1_resources[] = {
	[0] = {
		.start	= 0x00206000,
		.end	= 0x002060FF,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= (UART1_MINT_RX),
		.end	= (UART1_MINT_RX),
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		.start	= (UART1_MINT_TX),
		.end	= (UART1_MINT_TX),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device imx_uart1_device = {
	.name		= "imx-uart",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(imx_uart1_resources),
	.resource	= imx_uart1_resources,
};

static struct resource imx_uart2_resources[] = {
	[0] = {
		.start	= 0x00207000,
		.end	= 0x002070FF,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= (UART2_MINT_RX),
		.end	= (UART2_MINT_RX),
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		.start	= (UART2_MINT_TX),
		.end	= (UART2_MINT_TX),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device imx_uart2_device = {
	.name		= "imx-uart",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(imx_uart2_resources),
	.resource	= imx_uart2_resources,
};

static struct imxfb_mach_info imx_fb_info;

void __init set_imx_fb_info(struct imxfb_mach_info *hard_imx_fb_info)
{
	memcpy(&imx_fb_info,hard_imx_fb_info,sizeof(struct imxfb_mach_info));
}
EXPORT_SYMBOL(set_imx_fb_info);

static struct resource imxfb_resources[] = {
	[0] = {
		.start	= 0x00205000,
		.end	= 0x002050FF,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= LCDC_INT,
		.end	= LCDC_INT,
		.flags	= IORESOURCE_IRQ,
	},
};

static u64 fb_dma_mask = ~(u64)0;

static struct platform_device imxfb_device = {
	.name		= "imx-fb",
	.id		= 0,
	.dev		= {
 		.platform_data	= &imx_fb_info,
		.dma_mask	= &fb_dma_mask,
		.coherent_dma_mask = 0xffffffff,
	},
	.num_resources	= ARRAY_SIZE(imxfb_resources),
	.resource	= imxfb_resources,
};

static struct platform_device *devices[] __initdata = {
	&imx_mmc_device,
	&imxfb_device,
	&imx_uart1_device,
	&imx_uart2_device,
};


static struct map_desc imx_io_desc[] __initdata = {
        {
                .virtual        = IMX_IO_BASE,
                .pfn            = __phys_to_pfn(IMX_IO_PHYS),
                .length         = IMX_IO_SIZE,
                .type           = MT_DEVICE
        }
};

void __init
imx_map_io(void)
{
	iotable_init(imx_io_desc, ARRAY_SIZE(imx_io_desc));
}

static int __init imx_init(void)
{
	return platform_add_devices(devices, ARRAY_SIZE(devices));
}

subsys_initcall(imx_init);

#elif defined(CONFIG_ARCH_IMX21)
#include <asm/mach/map.h>
#include <asm/arch/mmc.h>

/* List current CRM register contents */
#define DEBUG_CRM 1

#define USEHS_XTAL 1
void imx_set_enet_irq(void);
void imx_clear_enet_irq(void);

void imx21_system_clk_init(void)
{
	/*
	 * System clock initialization
	 *
	 * The following register settings are similar to the ones currently
	 * configured in the Freescale distributions of Redboot and Grub. 
	 * All common clock settings should go here and specific settings should
	 * go in their respective drivers. 
	 */
	
	printk("Initializing system clocks\n");

	//	printk( "+++enable HCLK\n" );
	/* Enable HCLK input to the BROM, DMA modules */
	PCCR0 |= (PCCR0_HCLK_BROM_EN | PCCR0_HCLK_DMA_EN | PCCR0_DMA_EN);

	//	printk( "+++enable KPP\n" );
        /* Enable HCLK to the Keypad */
        PCCR1 |= PCCR1_KPP_EN;

	//	printk( "+++set SPLL\n" );
	/*
Possible Combinations for DPLL settings:

Freq Input        = 16.780001MHz
Freq Output       = 288.000000MHz
Allowed Tolerance = 0.000100%

MFI	MFN	MFD	PD	UPCTL0/MPCTL0	actual   	actual
   	   	   	  	             	tolerance	frequency
===	===	===	==	=============	=========	=========
8	317	544	0	0x0220213d	0.000085%	288.000244
8	374	642	0	0x02822176	0.000042%	288.000122
8	431	740	0	0x02e421af	0.000021%	288.000061
8	488	838	0	0x034621e8	0.000000%	288.000000

	*/
#if USEHS_XTAL
	// SPLL settings are now all done in the BOOTLOADER for this generation
	//	SPCTL0 = 0x80811446;  // configure for 26 MHz operation
        CSCR &= ~ CSCR_USB_DIV_MASK;
        CSCR |= CSCR_USB_DIV(5);  /* Divide by 6 = 288/48 */

	//	CSCR |= CSCR_SP_SEL; // select the 26 MHz clock
	
	/* Restart the SPLL and wait for the CSCR_SPLL_RESTART bit to clear */
	//	CSCR |= CSCR_SPLL_RESTART;
	//	while (CSCR & CSCR_SPLL_RESTART)
	//	    ;
#else
	/* 
	 * Set the SPLL so the output frequency is 288MHz by setting
	 * PD=0, MFD=626, MFI=8, and MFN=365 when fref=32.768kHz
	 */
	SPCTL0 = 0x834621e8; // 288 MHz override by bunnie (makes previous line irrelevant)
        /*
         * Make sure the 48 MHz clock is 48 MHZ, assuming SPLL is set
         * to 288 MHz.
         */
        CSCR &= ~ CSCR_USB_DIV_MASK;
        CSCR |= CSCR_USB_DIV(5);  /* Divide by 6 = 288/48 */
	
	/* Restart the SPLL and wait for the CSCR_SPLL_RESTART bit to clear */
	CSCR |= CSCR_SPLL_RESTART;
	while (CSCR & CSCR_SPLL_RESTART)
	    ;
#endif	
	/* Set the peripheral clock divider 3 (PERDIV3) to divide by 6 */
//	PCDR1 &= ~PCDR1_PERDIV3_MASK;
//	PCDR1 |= PCDR1_PERDIV3(5);
        /* For 352 MHz clock - PCLCK3 = 44MHz */
	PCDR1 &= ~PCDR1_PERDIV3_MASK;
	PCDR1 |= PCDR1_PERDIV3(7);

	//	printk( "+++set PCLK2,4\n" );
        /* For 352 MHz clock - PCLCK2 = 33MHz */
	PCDR1 &= ~PCDR1_PERDIV2_MASK;
	PCDR1 |= PCDR1_PERDIV2(10);
        /* For 352 MHz clock - PCLCK4 = 66MHz */
	PCDR1 &= ~PCDR1_PERDIV4_MASK;
	PCDR1 |= PCDR1_PERDIV4(4);

	//	printk( "+++set SSI1,2\n" );
        /*
         * Set the SSI1 and SSI2 clocks to be 12.5 MHz - 288/23
         * This is about 2% off from the preferred 12.288 MHz
         */
        PCDR0 &= ~(PCDR0_SSI1DIV_MASK | PCDR0_SSI2DIV_MASK);
	//        PCDR0 |= PCDR0_SSI1DIV(22) | PCDR0_SSI2DIV(22);
        PCDR0 |= PCDR0_SSI1DIV(22) | PCDR0_SSI2DIV(23); // SSI2 is set to 12 MHz to source CLKO (needed for Ironforge 1.0 ONLY)
	
#if DEBUG_CRM
	/* Let's take a look at the current CRM register settings */
	printk("CSCR: 0x%08x\n",CSCR);
	printk("MPCTL0: 0x%08x\n",MPCTL0);
	printk("MPCTL1: 0x%08x\n",MPCTL1);
	printk("SPCTL0: 0x%08x\n",SPCTL0);
	printk("SPCTL1: 0x%08x\n",SPCTL1);
	printk("OSC26MCTL: 0x%08x\n",OSC26MCTL);
	printk("PCDR0: 0x%08x\n",PCDR0);
	printk("PCDR1: 0x%08x\n",PCDR1);
	printk("PCCR0: 0x%08x\n",PCCR0);
	printk("PCCR1: 0x%08x\n",PCCR1);
	printk("CCSR: 0x%08x\n",CCSR);
	printk("WKGDCTL: 0x%08x\n",WKGDCTL);
#endif
	
}

/*
 *  get the system pll clock in Hz
 *
 *                  mfi + mfn / (mfd +1)
 *  f = 2 * f_ref * --------------------
 *                        pd + 1
 */

/* TODO: Remove hardcoded values and write routines to retrieve all clk info. */
static unsigned int imx_decode_pll(unsigned int pll)
{
	u32 mfi = (pll >> 10) & 0xf;
	u32 mfn = pll & 0x3ff;
	u32 mfd = (pll >> 16) & 0x3ff;
	u32 pd =  (pll >> 26) & 0xf;
	u32 f_ref = (CSCR & CSCR_MCU_SEL) ? 26000000 : (CLK32 * 512);

	mfi = mfi <= 5 ? 5 : mfi;

	return (2 * (f_ref>>10) * ( (mfi<<10) + (mfn<<10) / (mfd+1) )) / (pd+1);
}

unsigned int imx_get_system_clk(void)
{
	return imx_decode_pll(MPCTL0);
}
EXPORT_SYMBOL(imx_get_system_clk);

unsigned int imx_get_mcu_clk(void)
{
	return imx_decode_pll(MPCTL0);
}
EXPORT_SYMBOL(imx_get_mcu_clk);

/*
 *  get peripheral clock 1 ( UART[12], Timer[12], PWM )
 */
unsigned int imx_get_perclk1(void)
{
	return imx_get_system_clk() / (((PCDR1 & PCDR1_PERDIV1_MASK) >> PCDR1_PERDIV1_POS)+1);
}
EXPORT_SYMBOL(imx_get_perclk1);

/*
 *  get peripheral clock 2 ( SDHC, CSPI )
 */
unsigned int imx_get_perclk2(void)
{
	return imx_get_system_clk() / (((PCDR1 & PCDR1_PERDIV2_MASK) >> PCDR1_PERDIV2_POS)+1);
}
EXPORT_SYMBOL(imx_get_perclk2);

/*
 *  get peripheral clock 3 ( LCDC )
 */
unsigned int imx_get_perclk3(void)
{
	return imx_get_system_clk() / (((PCDR1 & PCDR1_PERDIV3_MASK) >> PCDR1_PERDIV3_POS)+1);
}
EXPORT_SYMBOL(imx_get_perclk3);

/*
 *  get peripheral clock 4 (CSI)
 */
unsigned int imx_get_perclk4(void)
{
	return imx_get_system_clk() / (((PCDR1 & PCDR1_PERDIV4_MASK) >> PCDR1_PERDIV4_POS)+1);
}

EXPORT_SYMBOL(imx_get_perclk4);

/*
 *  get hclk ( SDRAM, CSI, Memory Stick, I2C, DMA )
 */
#if 0
unsigned int imx_get_hclk(void)
{
	return imx_get_system_clk() / (((CSCR>>10) & 0xf)+1);
}
EXPORT_SYMBOL(imx_get_hclk);

#endif


static struct resource imx_mmc_resources[] = {
	[0] = {
		.start	= 0x10013000,
		.end	= 0x100130FF,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= (INT_SDHC1),
		.end	= (INT_SDHC1),
		.flags	= IORESOURCE_IRQ,
	},
};

static u64 imxmmc_dmamask = 0xffffffffUL;
static struct platform_device imx_mmc_device = {
    .name		= "imx-mmc",
    .id		= 0,
    .dev           = {
        .dma_mask = &imxmmc_dmamask,
        .coherent_dma_mask = 0xffffffff,
    },
    .num_resources	= ARRAY_SIZE(imx_mmc_resources),
    .resource	= imx_mmc_resources,
};

void __init imx_set_mmc_info(struct imxmmc_platform_data *info)
{
       imx_mmc_device.dev.platform_data = info;
}
EXPORT_SYMBOL(imx_set_mmc_info);

static struct resource imx21_uart1_resources[] = {
	[0] = {
		.start	= 0x1000A000,
		.end	= 0x1000A0FF,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= (INT_UART1),
		.end	= (INT_UART1),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device imx21_uart1_device = {
	.name		= "imx-uart",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(imx21_uart1_resources),
	.resource	= imx21_uart1_resources,
};

////////////////////////////////
/////// THIS IS NOT THE ONLY PLACE YOU NEED TO LOOK
/////// change imx_ports[] in drivers/serial/imx.c as well
////////////////////////////////

static struct resource imx21_uart2_resources[] = {
	[0] = {
		.start	= 0x1000B000,
		.end	= 0x1000B0FF,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= (INT_UART2),
		.end	= (INT_UART2),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device imx21_uart2_device = {
	.name		= "imx-uart",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(imx21_uart2_resources),
	.resource	= imx21_uart2_resources,
};

static struct resource imx21_uart3_resources[] = {
	[0] = {
		.start	= 0x1000C000,
		.end	= 0x1000C0FF,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= (INT_UART3),
		.end	= (INT_UART3),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device imx21_uart3_device = {
	.name		= "imx-uart",
	.id		= 2,
	.num_resources	= ARRAY_SIZE(imx21_uart3_resources),
	.resource	= imx21_uart3_resources,
};


static struct imxfb_mach_info imx_fb_info;

void __init set_imx_fb_info(struct imxfb_mach_info *hard_imx_fb_info)
{
	memcpy(&imx_fb_info,hard_imx_fb_info,sizeof(struct imxfb_mach_info));
}
EXPORT_SYMBOL(set_imx_fb_info);

static struct resource imxfb_resources[] = {
	[0] = {
		.start	= 0x10021000,
		.end	= 0x100210ff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= INT_LCDC,
		.end	= INT_LCDC,
		.flags	= IORESOURCE_IRQ,
	},
};

static u64 fb_dma_mask = ~(u64)0;

static struct platform_device imxfb_device = {
	.name		= "imx-fb",
	.id		= 0,
	.dev		= {
 		.platform_data	= &imx_fb_info,
		.dma_mask	= &fb_dma_mask,
		.coherent_dma_mask = 0xffffffff,
	},
	.num_resources	= ARRAY_SIZE(imxfb_resources),
	.resource	= imxfb_resources,
};

static struct platform_device *devices[] __initdata = {
     	&imx_mmc_device,
	&imxfb_device,
	&imx21_uart1_device,
	&imx21_uart2_device,
	// add these later
	&imx21_uart3_device,
	//	&imx21_uart4_device,
};

static struct map_desc imx21_io_desc[] __initdata = {
        {
                .virtual        = IMX_IO_BASE,
                .pfn            = __phys_to_pfn(IMX_IO_PHYS),
                .length         = IMX_IO_SIZE,
                .type           = MT_DEVICE
        }, {
                .virtual        = IMX_EMI_VIRT,
                .pfn            = __phys_to_pfn(IMX_EMI_PHYS),
                .length         = IMX_EMI_SIZE,
                .type           = MT_DEVICE
        }

};

void __init
imx21_map_io(void)
{
	iotable_init(imx21_io_desc, ARRAY_SIZE(imx21_io_desc));
}

//#define LOGO_BACK_PTR 0xC1000000
//#define LOGO_SIZE     0x40000

static int __init imx21_init(void)
{
  //  int i;
  int retval;
  
  retval = platform_add_devices(devices, ARRAY_SIZE(devices));

#if 0
  // doesn't port. I don't care.
  for( i = 0; i < LOGO_SIZE; i++ ) {
    *((unsigned char *)LCDC_SSA + i) = *((unsigned char *)LOGO_BACK_PTR + i);
  }
#endif
    return retval;
}

subsys_initcall(imx21_init);
#endif
