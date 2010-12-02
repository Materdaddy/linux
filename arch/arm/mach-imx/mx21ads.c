/*
 * arch/arm/mach-imx21/mx21ads.c
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
#include "generic.h"
#include <asm/serial.h>
#include <asm/arch/mmc.h>

#if defined (CONFIG_USB_IMX21_HCD)
#include <asm/arch/imx21-hcd.h>
#define OTG_TXCVR_VENDOR_ID_REG0                0x00
#define OTG_TXCVR_VENDOR_ID_REG1                0x01
#define OTG_TXCVR_PRODUCT_ID_REG0               0x02
#define OTG_TXCVR_PRODUCT_ID_REG1               0x03
#define OTG_TXCVR_MODE_REG1_SET                 0x04
#define OTG_TXCVR_MODE_REG1_CLR                 0x05
#define OTG_TXCVR_CTRL_REG1_SET                 0x06
#define OTG_TXCVR_CTRL_REG1_CLR                 0x07
#define OTG_TXCVR_INT_SRC_REG                   0x08
#define OTG_TXCVR_INT_LAT_REG_SET               0x0a
#define OTG_TXCVR_INT_LAT_REG_CLR               0x0b
#define OTG_TXCVR_INT_FALSE_REG_SET             0x0c
#define OTG_TXCVR_INT_FALSE_REG_CLR             0x0d
#define OTG_TXCVR_INT_TRUE_REG_SET              0x0e
#define OTG_TXCVR_INT_TRUE_REG_CLR              0x0f
#define OTG_TXCVR_CTRL_REG2_SET                 0x10
//#define OTG_TXCVR_CTRL_REG2_CLR                 0x11
#define OTG_TXCVR_MODE_REG2_SET                 0x12
#define OTG_TXCVR_MODE_REG2_CLR                 0x13
#define OTG_TXCVR_BCD_DEV_REG0                  0x14
#define OTG_TXCVR_BCD_DEV_REG1                  0x15

#define OTG_TXCVR_DEV_WRITE_ADDR 0x2D
#define OTG_TXCVR_DEV_READ_ADDR (0x2D | (1<<7))

#define ISP1301_ID_GROUNDED  (1<<3)
#endif

#if defined (CONFIG_FB_IMX)
#include <asm/arch/imxfb.h>
static void mx21ads_lcd_power(int on);

static struct imxfb_mach_info mx21ads_fb_info __initdata = {
  // PERDIV3 = 7, 44 MHz clock rate
    .pixclock     = 62500,
    .bpp          = 16,
    .xres         = 320,
    .yres         = 240,
    .hsync_len    = 0x1,
    .vsync_len    = 0x1,
    .left_margin  = 0xf,
    .upper_margin = 0x9,
    .right_margin = 0x6,
    .lower_margin = 0x7,
    
    // 1100 1010 0000 1000 0000 0000 1000 0100
    .pcr = (PCR_COLOR    | 
            PCR_TFT      | 
            PCR_OEPOL    |
            PCR_PBSIZ_8  |
            PCR_BPIX_16  | 
	    //            PCR_PIXPOL   | 
	    PCR_SCLKIDLE |
	    //            PCR_ACD_SEL  |
	    //            PCR_ACD(11)  |
            PCR_SCLK_SEL |
            PCR_SHARP    |
            PCR_PCD(7)),   // 44/8 = 5.5 MHz  --> about a 60 Hz update rate...could lower this
/*    .pwmr     = PWMR_SCR0 | PWMR_CC_EN | PWMR_PW(0x70), */
//    .pwmr       = 0x00a903ff, 
    .pwmr       = 0x0, 
//    .lscr1      = 0x00120300,
    .lscr1      = 0x0,
    
    .lcd_power = mx21ads_lcd_power,
};

static void mx21ads_lcd_power(int on)
{
  //    u16 *io = (u16*) (IMX_CS1_VIRT + 0x800000);
  
  // LCD_PWRON   K19  1 for on  KPCOL2
  if(on) {
    KPP_KPDR |= 400;
    //*io |= 0x200;
  } else {
    KPP_KPDR &= ~0x400;
    //*io &= ~0x200;
  }
}

#endif /* defined(CONFIG_FB_IMX) */

#if defined(CONFIG_CIRRUS)
static struct resource cs8900_resources[] = {
    [0] = {
        .start  = MX21ADS_ETH_PHYS,
        .end    = MX21ADS_ETH_PHYS + MX21ADS_ETH_SIZE - 1,
        .flags  = IORESOURCE_MEM,
    },
    [1] = {
        .start  = MX21ADS_ETH_IRQ,
        .end    = MX21ADS_ETH_IRQ,
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

static int mx21ads_mmc_card_present(void)
{

    if (SSR(3) & (1 << 25)) {
        return 0;
    } else {
        return 1;
    }
}

static struct imxmmc_platform_data mx21ads_mmc_info = {
    .card_present   = mx21ads_mmc_card_present,
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

#if defined(CONFIG_USB_IMX21_HCD)
static int otg_i2c_wait_busy(void)
{
    unsigned long timeout;
    int cnt = 0;
    
    timeout = jiffies + HZ;
    
    while (USBOTG_I2C_OP_CTRL_REG & 0x80) {
        if (time_after(jiffies, timeout)) {
            return -ETIMEDOUT;
        }
        cnt++;
        if (!in_interrupt())
            schedule_timeout(1);
        
    }
    return 0;
}

static int otg_i2c_reg_write(u8 reg, u8 data)
{
    USBOTG_I2C_TXCVR_REG(reg) = data;

    /* set sequential read start address */
    USBOTG_I2C_SEQ_RD_STARTAD = reg;
    /* set number of sequential operations */

    USBOTG_I2C_SEQ_OP_REG = 1;

    /* set transceiver's I2C device address, start operation */
    USBOTG_I2C_XCVR_DEVAD = OTG_TXCVR_DEV_WRITE_ADDR;

    return otg_i2c_wait_busy();
}
#endif

static void __init
mx21ads_init(void)
{
    /* Configure the system clocks */
  //  printk("...clocks\n");
    imx21_system_clk_init();	
	
    /* CrossBar (MAX) Settings */
    //  printk("...xbar\n");
    /* Slave port for SSI - DMA highest priority, ARM lowest */
    MAX_MPR(0) = 0x00103256;

    /* Slave port for EMI - LCD/SLCD highest priority, ARM lowest */
    MAX_MPR(3) = 0x00123056;	

    /* initialize the debug LED */
    imx_gpio_mode(14 | GPIO_PORTC | GPIO_OUT | GPIO_GPIO); 
#if 0
    // led testing
    int i, j;
    for( j = 0; j < 4; j++ ) {
      for( i = 0; i < 400000; i++ ) {
	imx_gpio_write(14 | GPIO_PORTC, 0);
      }
      //      printk("i");
      for( i = 0; i < 200000; i++ ) {
	imx_gpio_write(14 | GPIO_PORTC, 1);
      }
      //      printk("o");
    }
#endif
    imx_gpio_write(14 | GPIO_PORTC, 0);  // turn the LED off as the bootloader turns it on
    
    // setup initial port settings!
    // WIFI_ON     J18  1 for on  KPCOL0
    // LCD_PWRON   K19  1 for on  KPCOL2
    // CPU_AUD_ENA K18  1 for on  KPCOL4
    // 0001 0101 0000 00000 is the bitmask pattern...
    PCCR1 |= PCCR1_KPP_EN;
    KPP_KPSR |= KPP_KPSR_KPPEN;
    KPP_KPCR = 0;
    KPP_KDDR = 0xFF00;
    KPP_KPDR = 0xFE00;

#if defined(CONFIG_FB_IMX)
    printk("%s:%d\n", __FUNCTION__, __LINE__);
    /* Configure I/O for LCD */
    /* VEE_EN */
    imx_gpio_mode(30 | GPIO_OUT | GPIO_PORTA | GPIO_GPIO | GPIO_PUEN);
    /* PWM0 */
    imx_gpio_mode(5  | GPIO_OUT | GPIO_PORTE | GPIO_GPIO | GPIO_PUEN);
    
    set_imx_fb_info(&mx21ads_fb_info);
    mx21ads_lcd_power(1);  // turn on the LCD backlight...
#endif

    /* For MMC */
    //  printk("...mmc\n");
    imx_gpio_mode(26 | GPIO_PORTD | GPIO_GPIO | GPIO_IN);       /* WP */
    imx_gpio_mode(25 | GPIO_PORTD | GPIO_GPIO | GPIO_IN);       /* CD */

    /* For NAND flash */
    imx_gpio_mode(0  | GPIO_PORTF);
    imx_gpio_mode(1  | GPIO_PORTF);
    imx_gpio_mode(2  | GPIO_PORTF);
    imx_gpio_mode(3  | GPIO_PORTF);
    imx_gpio_mode(4  | GPIO_PORTF);
    imx_gpio_mode(5  | GPIO_PORTF);
    imx_gpio_mode(6  | GPIO_PORTF);
    imx_gpio_mode(7  | GPIO_PORTF);
    imx_gpio_mode(8  | GPIO_PORTF);
    imx_gpio_mode(9  | GPIO_PORTF);
    imx_gpio_mode(10 | GPIO_PORTF);
    imx_gpio_mode(11 | GPIO_PORTF);
    imx_gpio_mode(12 | GPIO_PORTF);
    imx_gpio_mode(13 | GPIO_PORTF);
    imx_gpio_mode(14 | GPIO_PORTF);



#if defined(CONFIG_USB_IMX21_HCD)
  printk("...usb\n");
    /* Enable the clocks to the USB OTG module */
    PCCR0 |= PCCR0_HCLK_USBOTG_EN | PCCR0_USBOTG_EN;

    /* Configure the GPIO */
    imx_gpio_mode(23 | GPIO_PORTB );   /* USB Host PWR */

    imx_gpio_mode(13 | GPIO_PORTC );   /* USB Host RXDP */
    imx_gpio_mode(12 | GPIO_PORTC );   /* USB Host RXDM */
    imx_gpio_mode(11 | GPIO_PORTC );   /* USB Host TXDP */
    imx_gpio_mode(10 | GPIO_PORTC );   /* USB Host TXDM */
    imx_gpio_mode( 9 | GPIO_PORTC );   /* USB Host OE */
    //    imx_gpio_mode( 8 | GPIO_PORTC | GPIO_IN | GPIO_GPIO | GPIO_PUEN); /* FS */
    imx_gpio_mode( 8 | GPIO_PORTC ); /* FS */
    //    imx_gpio_mode( 7 | GPIO_PORTC );   /* USB Host ON */
    //    imx_gpio_mode( 6 | GPIO_PORTC );   /* USB Host SCL */
    //    imx_gpio_mode( 5 | GPIO_PORTC );   /* USB Host SDA */

    imx_gpio_mode(25 | GPIO_PORTB );  // USBH ON

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

    /*
     * Configure the USBOTG module:
     *   Host xcvr single ended TX, diff RX
     *   USBOTG xcvr single ended TX, diff RX
     *   USBOTG in func mode
     */
    /*    USBOTG_HWMODE = (USBOTG_HWMODE_OTGXCVR_TS_RD |
                     USBOTG_HWMODE_HOSTXCVR_TS_RD |
                     USBOTG_HWMODE_CRECFG_FUNC);*/
    USBOTG_HWMODE = (USBOTG_HWMODE_OTGXCVR_TD_RD |  
                     USBOTG_HWMODE_HOSTXCVR_TD_RD | // i believe that TD_RD is correct...was TS_RD
                     USBOTG_HWMODE_CRECFG_HOST);

    USBOTG_HNP_CSR = 0;

#if 0  // we have no OTG in this version of the hardware
    USBOTG_I2C_SCLK_TO_SCK_HPER = 0xf0; /* Clock = 100KHz */
    USBOTG_I2C_MASTER_INT_REG &= 0x0f;  /* disable interrupts */
    USBOTG_I2C_MASTER_INT_REG &= 0xf0;  /* clear interrupts */

    USBOTG_I2C_OP_CTRL_REG = 0x01;      /* Enable output, sw mode */

    otg_i2c_wait_busy();

    otg_i2c_reg_write(OTG_TXCVR_MODE_REG1_CLR, 0xFF);
    otg_i2c_reg_write(OTG_TXCVR_MODE_REG2_CLR, 0xFF);
    otg_i2c_reg_write(OTG_TXCVR_CTRL_REG1_SET, 0xC);
    otg_i2c_reg_write(OTG_TXCVR_CTRL_REG1_CLR, ~0xC);
    otg_i2c_reg_write(OTG_TXCVR_INT_FALSE_REG_CLR, 0xFF);
    otg_i2c_reg_write(OTG_TXCVR_INT_TRUE_REG_SET, ISP1301_ID_GROUNDED);
    otg_i2c_reg_write(OTG_TXCVR_INT_TRUE_REG_CLR, ~ISP1301_ID_GROUNDED);
    otg_i2c_reg_write(OTG_TXCVR_INT_FALSE_REG_SET, ISP1301_ID_GROUNDED);
    otg_i2c_reg_write(OTG_TXCVR_INT_FALSE_REG_CLR, ~ISP1301_ID_GROUNDED);
    otg_i2c_reg_write(OTG_TXCVR_INT_LAT_REG_CLR, 0xFF);
    
    /* Disable B device */
    otg_i2c_reg_write(OTG_TXCVR_CTRL_REG1_CLR, 0x1);
    otg_i2c_reg_write(OTG_TXCVR_CTRL_REG1_SET, 0x4);
    USBOTG_HNP_CSR &= ~(1 << 22);

    /* Enable A device */
    otg_i2c_reg_write(OTG_TXCVR_CTRL_REG1_SET, 0x20);
#endif

    // USB hub ports
    // H12 USBH1_TXDP  PB29
    // D11 USBH1_TXDM  PB28
    // D12 USBH1_RXDP  PB31
    // C12 USBH1_RXDM  PB30
    // A11 USBH1_FS    PB26
    // A12 USBH1_OE    PB27
    // G11 USBH_ON     PB25
    
    imx_gpio_mode(29 | GPIO_PORTB | GPIO_PF | GPIO_OUT );   // USBH1_TXDP
    imx_gpio_mode(28 | GPIO_PORTB | GPIO_PF | GPIO_OUT );   // USBH1_TXDM
    imx_gpio_mode(31 | GPIO_PORTB | GPIO_PF | GPIO_IN );   // USBH1_RXDP
    imx_gpio_mode(30 | GPIO_PORTB | GPIO_PF | GPIO_IN );   // USBH1_RXDM
    imx_gpio_mode(26 | GPIO_PORTB | GPIO_PF | GPIO_OUT );   /* USBH1 FS */
    imx_gpio_mode(27 | GPIO_PORTB | GPIO_PF | GPIO_OUT );   // USBH1_OE
    imx_gpio_mode(25 | GPIO_PORTB | GPIO_PF | GPIO_OUT );   // USBH_ON

    printk( "DDIR(1): %08X\n", DDIR(1));
    printk( "OCR2(1): %08X\n", OCR2(1));
    printk( "ICONFA2(1): %08X\n", ICONFA2(1));
    printk( "GIUS(1): %08X\n", GIUS(1));
    printk( "PUEN(1): %08X\n", PUEN(1));
    printk( "GPR(1): %08X\n", GPR(1));
    
    // GIUS programming is bugged, you must do it manually if you
    // use any of the A,B, or C alternate functions of a GPIO
    // D18 USBH3_FS    PD21  AIN   1
    // C19 USBH3_RXDP  PD20  AOUT  0
    // B19 USBH3_RXDM  PD19  AOUT  0
    // D19 USBH3_OE    PD22  AIN   1
    // E19 USBH3_TXDP  PD24  AIN   1
    // E17 USBH3_TXDM  PD23  AIN   1
    // A B C D E
    // 0 1 2 3 4
    DDIR(3) &= 0xFFE7FFFF;  // bits 19 and 20 clear to 0
    DDIR(3) |= 0x01E00000;  // bits 21, 22, 23, and 24 set to 1
    OCR2(3) &= 0xFFFC03FF;  // bits 21, 22, 23, and 24
    ICONFA2(3) &= 0xFFFFFC3F;  // bits 19 and 20
    GIUS(3) |= 0x01F80000;

    // turn off over current reporting, because we don't have it
    USBH_ROOTHUBA |= USBH_ROOTHUBA_NOOVRCURP;
                      
#endif

    // turn on CLKO to the CP
    // PF15, but defaults to CLKO
    imx_gpio_mode(15 | GPIO_PORTF | GPIO_PF | GPIO_OUT );
    CCSR &= ~0x1F;
    //    CCSR |= 0xF; // SSI2 baud, which is set to 12 MHz (ironforge 1.0 only; all others quiet)
    CCSR |= 0x14; // CLK48M
    PCCR0 |= PCCR0_SSI2_EN;
    PCCR0 |= PCCR0_SSI2_BAUD_EN;
    PCDR0 &= ~(PCDR0_SSI2DIV_MASK);
    PCDR0 |= PCDR0_SSI2DIV(23); // SSI2 is set to 12 MHz to source CLKO (needed for Ironforge 1.0 ONLY)

    // enable UART3 for the CP
    PCCR0 &= ~(PCCR0_UART2_EN);
    PCCR0 |= PCCR0_UART3_EN;

#if 0
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

#if 0  // there is no MMC in this version of the hardware
    imx_set_mmc_info(&mx21ads_mmc_info);
#endif
    //  printk("...adddevs\n");
    platform_add_devices(devices, ARRAY_SIZE(devices));
}

static struct map_desc mx21ads_io_desc[] __initdata = {
    {
        .virtual = IMX_CS0_VIRT,
        .pfn     = __phys_to_pfn(IMX_CS0_PHYS),
        .length  = IMX_CS0_SIZE,
        .type    =  MT_DEVICE
    },{
        .virtual = MX21ADS_ETH_VIRT,
        .pfn     = __phys_to_pfn(MX21ADS_ETH_PHYS),
        .length  = MX21ADS_ETH_SIZE,
        .type    =  MT_DEVICE
    },{
        .virtual = MX21ADS_UART_VIRT,
        .pfn     = __phys_to_pfn(MX21ADS_UART_PHYS),
        .length  = MX21ADS_UART_SIZE,
        .type    =  MT_DEVICE
    },{
        .virtual = MX21ADS_IN0_VIRT,
        .pfn     = __phys_to_pfn(MX21ADS_IN0_PHYS),
        .length  = MX21ADS_IN0_SIZE,
        .type    =  MT_DEVICE
    },{
        .virtual = MX21ADS_IN1_VIRT,
        .pfn     = __phys_to_pfn(MX21ADS_IN1_PHYS),
        .length  = MX21ADS_IN1_SIZE,
        .type    =  MT_DEVICE
    },{
        .virtual = MX21ADS_IO0_VIRT,
        .pfn     = __phys_to_pfn(MX21ADS_IO0_PHYS),
        .length  = MX21ADS_IO0_SIZE,
        .type    =  MT_DEVICE
    },{
        .virtual = MX21ADS_IO1_VIRT,
        .pfn     = __phys_to_pfn(MX21ADS_IO1_PHYS),
        .length  = MX21ADS_IO1_SIZE,
        .type    =  MT_DEVICE
    },
};

static void __init
mx21ads_map_io(void)
{
    imx21_map_io();
    iotable_init(mx21ads_io_desc, ARRAY_SIZE(mx21ads_io_desc));
}

MACHINE_START(MX21, "Freescale MX21ADS")
/*	MAINTAINER("Jay Monkman <jtm@lopingdog.com>") */
	.phys_io	= 0x00200000,
	.io_pg_offst	= ((0xe0200000) >> 18) & 0xfffc,
	.boot_params	= 0xc0000100,

	.map_io		= mx21ads_map_io,
	.init_irq	= imx21_init_irq,
	.timer		= &imx_timer,
	.init_machine	= mx21ads_init,
MACHINE_END
