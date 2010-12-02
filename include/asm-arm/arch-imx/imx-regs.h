#ifndef _IMX_REGS_H
#define _IMX_REGS_H
/* ------------------------------------------------------------------------
 *  Motorola IMX system registers
 * ------------------------------------------------------------------------
 *
 */

/*
 *  Register BASEs, based on OFFSETs
 *
 */
#if defined(CONFIG_ARCH_IMX)

#define IMX_AIPI1_BASE             (0x00000 + IMX_IO_BASE)
#define IMX_WDT_BASE               (0x01000 + IMX_IO_BASE)
#define IMX_TIM1_BASE              (0x02000 + IMX_IO_BASE)
#define IMX_TIM2_BASE              (0x03000 + IMX_IO_BASE)
#define IMX_RTC_BASE               (0x04000 + IMX_IO_BASE)
#define IMX_LCDC_BASE              (0x05000 + IMX_IO_BASE)
#define IMX_UART1_BASE             (0x06000 + IMX_IO_BASE)
#define IMX_UART2_BASE             (0x07000 + IMX_IO_BASE)
#define IMX_PWM_BASE               (0x08000 + IMX_IO_BASE)
#define IMX_DMAC_BASE              (0x09000 + IMX_IO_BASE)
#define IMX_AIPI2_BASE             (0x10000 + IMX_IO_BASE)
#define IMX_SIM_BASE               (0x11000 + IMX_IO_BASE)
#define IMX_USBD_BASE              (0x12000 + IMX_IO_BASE)
#define IMX_SPI1_BASE              (0x13000 + IMX_IO_BASE)
#define IMX_MMC_BASE               (0x14000 + IMX_IO_BASE)
#define IMX_ASP_BASE               (0x15000 + IMX_IO_BASE)
#define IMX_BTA_BASE               (0x16000 + IMX_IO_BASE)
#define IMX_I2C_BASE               (0x17000 + IMX_IO_BASE)
#define IMX_SSI_BASE               (0x18000 + IMX_IO_BASE)
#define IMX_SPI2_BASE              (0x19000 + IMX_IO_BASE)
#define IMX_MSHC_BASE              (0x1A000 + IMX_IO_BASE)
#define IMX_PLL_BASE               (0x1B000 + IMX_IO_BASE)
#define IMX_GPIO_BASE              (0x1C000 + IMX_IO_BASE)
#define IMX_EIM_BASE               (0x20000 + IMX_IO_BASE)
#define IMX_SDRAMC_BASE            (0x21000 + IMX_IO_BASE)
#define IMX_MMA_BASE               (0x22000 + IMX_IO_BASE)
#define IMX_AITC_BASE              (0x23000 + IMX_IO_BASE)
#define IMX_CSI_BASE               (0x24000 + IMX_IO_BASE)

#elif defined(CONFIG_ARCH_IMX21)

#define IMX_AIPI1_BASE             (0x00000)
#define IMX_DMAC_BASE              (0x01000)
#define IMX_WDT_BASE               (0x02000)
#define IMX_TIM1_BASE              (0x03000)
#define IMX_TIM2_BASE              (0x04000)
#define IMX_TIM3_BASE              (0x05000)
#define IMX_PWM_BASE               (0x06000)
#define IMX_RTC_BASE               (0x07000)
#define IMX_KPP_BASE               (0x08000)
#define IMX_OWIRE_BASE             (0x09000)
#define IMX_UART1_BASE             (0x0A000)
#define IMX_UART2_BASE             (0x0B000)
#define IMX_UART3_BASE             (0x0C000)
#define IMX_UART4_BASE             (0x0D000)
#define IMX_SPI_BASE(x)            (((x) == 0) ? 0x0e000 : (((x) == 1) ? 0x0f000 : 0x17000))

#define IMX_SPI1_BASE              (0x0E000)
#define IMX_SPI2_BASE              (0x0F000)
#define IMX_SSI1_BASE              (0x10000)
#define IMX_SSI2_BASE              (0x11000)
#define IMX_I2C_BASE               (0x12000)
#define IMX_SDHC1_BASE             (0x13000)
#define IMX_SDHC2_BASE             (0x14000)
#define IMX_GPIO_BASE              (0x15000)
#define IMX_AUDMUX_BASE            (0x16000)
#define IMX_SPI3_BASE              (0x17000)
#define IMX_AIPI2_BASE             (0x20000)
#define IMX_LCDC_BASE              (0x21000)
#define IMX_SLCDC_BASE             (0x22000)
#define IMX_SAHARA_BASE            (0x23000)
#define IMX_USBOTG_BASE            (0x24000)
#define IMX_EMMA_BASE              (0x26000)
#define IMX_CRM_BASE               (0x27000)
#define IMX_FIRI_BASE              (0x28000)
#define IMX_RNGA_BASE              (0x29000)
#define IMX_RTIC_BASE              (0x2A000)
#define IMX_JAM_BASE               (0x3E000)
#define IMX_MAX_BASE               (0x3F000)
#define IMX_AITC_BASE              (0x40000)

#endif /* defined(CONFIG_ARCH_IMX21) */


/* PLL registers */
#if defined(CONFIG_ARCH_IMX)
  
# define CSCR   __REG(IMX_PLL_BASE)
# define CSCR_SYSTEM_SEL (1<<16)
# define CSCR_MPLL_RESTART (1<<21)
  
# define MPCTL0 __REG(IMX_PLL_BASE + 0x4)
# define MPCTL1 __REG(IMX_PLL_BASE + 0x8)
# define SPCTL0 __REG(IMX_PLL_BASE + 0xc)
# define SPCTL1 __REG(IMX_PLL_BASE + 0x10)
# define PCDR   __REG(IMX_PLL_BASE + 0x20)
# define FMCR   __REG(IMX_PLL_BASE + 0x808) 

#elif defined (CONFIG_ARCH_IMX21)

# define CSCR                    __REG(IMX_CRM_BASE + 0x0)
# define CSCR_PRESC_MASK         (0x7<<29)
# define CSCR_PRESC(x)           (((x) & 0x7) << 29)
# define CSCR_USB_DIV_MASK       (0x7<<26)
# define CSCR_USB_DIV(x)         (((x) & 0x7) << 26)
# define CSCR_SD_CNT_MASK        (0x3<<24)
# define CSCR_SD_CNT(x)          (((x) & 0x3) << 24)
# define CSCR_SPLL_RESTART       (1<<22)
# define CSCR_MPLL_RESTART       (1<<21)
# define CSCR_SSI2_SEL           (1<<20)
# define CSCR_SSI1_SEL           (1<<19)
# define CSCR_FIR_SEL            (1<<18)
# define CSCR_SP_SEL             (1<<17)
# define CSCR_MCU_SEL            (1<<16)
# define CSCR_BCLKDIV_MASK       (0xf<<10)
# define CSCR_BCLKDIV(x)         (((x) & 0xf) << 10)
# define CSCR_IPDIV              (1<<9)
# define CSCR_OSC26M_DIV1P5      (1<<4)
# define CSCR_OSC28M_DIS         (1<<3)
# define CSCR_FPM_EN             (1<<2)
# define CSCR_SPEN               (1<<1)
# define CSCR_MPEN               (1<<0)
  
# define MPCTL0                  __REG(IMX_CRM_BASE + 0x4)
# define MPCTL0_CPLM             (1<<31)
# define MPCTL0_PD(x)            (((x) & 0xf) << 26)
# define MPCTL0_MFD(x)           (((x) & 0x3ff) << 16)
# define MPCTL0_MFI(x)           (((x) & 0xf) << 10)
# define MPCTL0_MFN(x)           (((x) & 0x3ff) << 0)
  
# define MPCTL1                   __REG(IMX_CRM_BASE + 0x8)
# define MPCTL1_LF               (1<<15)
# define MPCTL1_BRMO             (1<<6)
  
# define SPCTL0                  __REG(IMX_CRM_BASE + 0xc)
# define SPCTL0_CPLM             (1<<31)
# define SPCTL0_PD(x)            (((x) & 0xf) << 26)
# define SPCTL0_MFD(x)           (((x) & 0x3ff) << 16)
# define SPCTL0_MFI(x)           (((x) & 0xf) << 10)
# define SPCTL0_MFN(x)           (((x) & 0x3ff) << 0)
  
# define SPCTL1                  __REG(IMX_CRM_BASE + 0x10)
# define SPCTL1_LF               (1<<15)
# define SPCTL1_BRMO             (1<<6)
  
# define OSC26MCTL               __REG(IMX_CRM_BASE + 0x14)
# define OSC26MCTL_OSC26M_PEAK   (0x2<<16)
# define OSC25MCTL_AGC(x)        (((x) & 0x3f) << 8)
  
# define PCDR0                   __REG(IMX_CRM_BASE + 0x18)
# define PCDR0_SSI2DIV(x)        (((x) & 0x3f) << 26)
# define PCDR0_SSI2DIV_MASK      (0x3f << 26)
# define PCDR0_SSI1DIV(x)        (((x) & 0x3f) << 16)
# define PCDR0_SSI1DIV_MASK      (0x3f << 16)
# define PCDR0_NFCDIV(x)         (((x) & 0xf) << 12)
# define PCDR0_NFCDIV_MASK       (0xf << 12)
# define PCDR0_CLKO_48MDIV(x)    (((x) & 0x7) << 5)
# define PCDR0_CLKO_48MDIV_MASK  (0x7 << 5)
# define PCDR0_FIRI_DIV(x)       (((x) & 0x1f) << 0)
# define PCDR0_FIRI_DIV_MASK     (0x1f << 0)
                 
# define PCDR1                   __REG(IMX_CRM_BASE + 0x1c)
# define PCDR1_PERDIV4_POS       24
# define PCDR1_PERDIV4_MASK      (0x3f << PCDR1_PERDIV4_POS)
# define PCDR1_PERDIV4(x)        (((x) << PCDR1_PERDIV4_POS) & \
                                  PCDR1_PERDIV4_MASK)
# define PCDR1_PERDIV3_POS       16
# define PCDR1_PERDIV3_MASK      (0x3f << PCDR1_PERDIV3_POS)
# define PCDR1_PERDIV3(x)        (((x) << PCDR1_PERDIV3_POS) & \
                                  PCDR1_PERDIV3_MASK)
# define PCDR1_PERDIV2_POS       8
# define PCDR1_PERDIV2_MASK      (0x3f << PCDR1_PERDIV2_POS)
# define PCDR1_PERDIV2(x)        (((x) << PCDR1_PERDIV2_POS) & \
                                  PCDR1_PERDIV2_MASK)
# define PCDR1_PERDIV1_POS       0
# define PCDR1_PERDIV1_MASK      (0x3f << PCDR1_PERDIV1_POS)
# define PCDR1_PERDIV1(x)        (((x) << PCDR1_PERDIV1_POS) & \
                                  PCDR1_PERDIV1_MASK)
  
# define PCCR0                   __REG(IMX_CRM_BASE + 0x20) 
# define PCCR0_HCLK_CSI_EN       (1<<31)
# define PCCR0_HCLK_DMA_EN       (1<<30)
# define PCCR0_HCLK_BROM_EN      (1<<28)
# define PCCR0_HCLK_EMMA_EN      (1<<27)
# define PCCR0_HCLK_LCDC_EN      (1<<26)
# define PCCR0_HCLK_SLCDC_EN     (1<<25)
# define PCCR0_HCLK_USBOTG_EN    (1<<24)
# define PCCR0_HCLK_BMI_EN       (1<<23)
# define PCCR0_PERCLK4_EN        (1<<22)
# define PCCR0_SLCDC_EN          (1<<21)
# define PCCR0_FIRI_BAUD_EN      (1<<20)
# define PCCR0_NFC_EN            (1<<19)
# define PCCR0_PERCLK3_EN        (1<<18)
# define PCCR0_SSI1_BAUD_EN      (1<<17)
# define PCCR0_SSI2_BAUD_EN      (1<<16)
# define PCCR0_EMMA_EN           (1<<15)
# define PCCR0_USBOTG_EN         (1<<14)
# define PCCR0_DMA_EN            (1<<13)
# define PCCR0_I2C_EN            (1<<12)
# define PCCR0_GPIO_EN           (1<<11)
# define PCCR0_SDHC2_EN          (1<<10)
# define PCCR0_SDHC1_EN          (1<<9)
# define PCCR0_FIRI_EN           (1<<8)
# define PCCR0_SSI2_EN           (1<<7)
# define PCCR0_SSI1_EN           (1<<6)
# define PCCR0_CSPI2_EN          (1<<5)
# define PCCR0_CSPI1_EN          (1<<4)
# define PCCR0_UART4_EN          (1<<3)
# define PCCR0_UART3_EN          (1<<2)
# define PCCR0_UART2_EN          (1<<1)
# define PCCR0_UART1_EN          (1<<0)
  
# define PCCR1                   __REG(IMX_CRM_BASE + 0x24)
# define PCCR1_OWIRE_EN          (1<<31)
# define PCCR1_KPP_EN            (1<<30)
# define PCCR1_RTC_EN            (1<<29)
# define PCCR1_PWM_EN            (1<<28)
# define PCCR1_GPT3_EN           (1<<27)
# define PCCR1_GPT2_EN           (1<<26)
# define PCCR1_GPT1_EN           (1<<25)
# define PCCR1_WDT_EN            (1<<24)
# define PCCR1_CSPI3_EN          (1<<23)
# define PCCR1_RTIC_EN           (1<<22)
# define PCCR1_RNGA_EN           (1<<21)
  
# define CCSR                    __REG(IMX_CRM_BASE + 0x28)
# define CCSR_32K_SR             (1<<15)
# define CCSR_CLK0_SEL(x)        (((x) & 0x1f) << 0)
         
# define WKGDCTL                 __REG(IMX_CRM_BASE + 0x34)
# define WKGDCTL_WKDG_EN         (1<<0)

# define FMCR                    __REG(IMX_CRM_BASE + 0x814)
# define FMCR_UART4_RXD          (1 << 25)
# define FMCR_UART4_RTS          (1 << 24)
# define FMCR_KP_COL6            (1 << 18)
# define FMCR_KP_COL7            (1 << 17)
# define FMCR_KP_ROW6            (1 << 16)
# define FMCR_CRM_SPA            (1 << 12)
# define FMCR_NF_FMS             (1 << 5)
# define FMCR_NF_16BIT           (1 << 4)
# define FMCR_SLCDC              (1 << 2)
# define FMCR_SDCS1              (1 << 1)
# define FMCR_SDCS0              (1 << 0)

# define DSCR1                   __REG(IMX_CRM_BASE + 0x820)
# define DSCR2                   __REG(IMX_CRM_BASE + 0x824)
# define DSCR3                   __REG(IMX_CRM_BASE + 0x828)
# define DSCR4                   __REG(IMX_CRM_BASE + 0x82C)
# define DSCR5                   __REG(IMX_CRM_BASE + 0x830)
# define DSCR6                   __REG(IMX_CRM_BASE + 0x834)
# define DSCR7                   __REG(IMX_CRM_BASE + 0x838)
# define DSCR8                   __REG(IMX_CRM_BASE + 0x83C)
# define DSCR9                   __REG(IMX_CRM_BASE + 0x840)
# define DSCR10                   __REG(IMX_CRM_BASE + 0x844)
# define DSCR11                   __REG(IMX_CRM_BASE + 0x848)
# define DSCR12                   __REG(IMX_CRM_BASE + 0x84C)

#endif /* defined(CONFIG_ARCH_IMX21) */

/*
 *  GPIO Module and I/O Multiplexer
 *  x = 0..5 for reg_A, reg_B, reg_C, reg_D, reg_E, reg_F
 */

#define DDIR(x)    __REG2(IMX_GPIO_BASE + 0x00, ((x) & 7) << 8)
#define OCR1(x)    __REG2(IMX_GPIO_BASE + 0x04, ((x) & 7) << 8)
#define OCR2(x)    __REG2(IMX_GPIO_BASE + 0x08, ((x) & 7) << 8)
#define ICONFA1(x) __REG2(IMX_GPIO_BASE + 0x0c, ((x) & 7) << 8)
#define ICONFA2(x) __REG2(IMX_GPIO_BASE + 0x10, ((x) & 7) << 8)
#define ICONFB1(x) __REG2(IMX_GPIO_BASE + 0x14, ((x) & 7) << 8)
#define ICONFB2(x) __REG2(IMX_GPIO_BASE + 0x18, ((x) & 7) << 8)
#define DR(x)      __REG2(IMX_GPIO_BASE + 0x1c, ((x) & 7) << 8)
#define GIUS(x)    __REG2(IMX_GPIO_BASE + 0x20, ((x) & 7) << 8)
#define SSR(x)     __REG2(IMX_GPIO_BASE + 0x24, ((x) & 7) << 8)
#define ICR1(x)    __REG2(IMX_GPIO_BASE + 0x28, ((x) & 7) << 8)
#define ICR2(x)    __REG2(IMX_GPIO_BASE + 0x2c, ((x) & 7) << 8)
#define IMR(x)     __REG2(IMX_GPIO_BASE + 0x30, ((x) & 7) << 8)
#define ISR(x)     __REG2(IMX_GPIO_BASE + 0x34, ((x) & 7) << 8)
#define GPR(x)     __REG2(IMX_GPIO_BASE + 0x38, ((x) & 7) << 8)
#define SWR(x)     __REG2(IMX_GPIO_BASE + 0x3c, ((x) & 7) << 8)
#define PUEN(x)    __REG2(IMX_GPIO_BASE + 0x40, ((x) & 7) << 8)
#define PMASK      __REG(IMX_GPIO_BASE + 0x600)



/*
 *  GPIO Mode
 *
 *  The pin, port, data direction, pull-up enable, primary/alternate
 *  function, output configuration, and input configuration are encoded in a 
 *  single word as follows.
 *
 *  4:0 Pin (31-0)
 *  7:5 Port (F-A, KP)
 *  8 Direction
 *  9 PUEN
 *  10:11 Primary Function,Alternate Function
 *  13:12 OCR
 *  15:14 ICONF
 *  16    OC (KP only)
 *  19:17 IRQ configuration
 * 
 *  [ 17 18 19 | 16 | 15 14 | 13 12 | 11 10 | 9  |  8  | 7 6 5 | 4 3 2 1 0 ]
 *  [ IRQ CFG  | OC | ICONF |  OCR  | P/A   | PU | Dir | Port  |    Pin    ]
 */

#define GPIO_PIN_MASK           (0x1f<<0)

#define GPIO_PORT_POS           5
#define GPIO_PORT_MASK          (0x7 << GPIO_PORT_POS)
#define GPIO_PORTA              (0 << GPIO_PORT_POS)
#define GPIO_PORTB              (1 << GPIO_PORT_POS)
#define GPIO_PORTC              (2 << GPIO_PORT_POS)
#define GPIO_PORTD              (3 << GPIO_PORT_POS)
#define GPIO_PORTE              (4 << GPIO_PORT_POS)
#define GPIO_PORTF              (5 << GPIO_PORT_POS)
#define GPIO_PORTKP             (7 << GPIO_PORT_POS) /* For keyboard port */

#define GPIO_DIR_MASK           (1<<8)
#define GPIO_IN                 (0<<8)
#define GPIO_OUT                (1<<8)

#define GPIO_PU_MASK            (1<<9)
#define GPIO_PUDIS              (0<<9)
#define GPIO_PUEN               (1<<9)

#define GPIO_FUNC_MASK          (0x3<<10)
#define GPIO_PF                 (0<<10)
#define GPIO_AF                 (1<<10)

#define GPIO_OCR_POS            12
#define GPIO_OCR_MASK           (0x3 << GPIO_OCR_POS)
#define GPIO_AIN                (0 << GPIO_OCR_POS)
#define GPIO_BIN                (1 << GPIO_OCR_POS)
#define GPIO_CIN                (2 << GPIO_OCR_POS)
#define GPIO_GPIO               (3 << GPIO_OCR_POS)

#define GPIO_ICONF_MASK         (0x3<<14)
#define GPIO_AOUT               (1<<14)
#define GPIO_BOUT               (2<<14)

#define GPIO_OC_MASK            (1<<16)
#define GPIO_OCDIS              (0<<16)
#define GPIO_OCEN               (1<<16)

#define GPIO_IRQ_MASK           (7 << 17)
#define GPIO_IRQ_RISING         ((0 << 18) | (1 << 17))
#define GPIO_IRQ_FALLING        ((1 << 18) | (1 << 17))
#define GPIO_IRQ_HIGH           ((2 << 18) | (1 << 17))
#define GPIO_IRQ_LOW            ((3 << 18) | (1 << 17))
#define GPIO_IRQ_CFG_MASK       (0x3 << 18)
#define GPIO_IRQ_CFG_POS        18

/*
 *  The GPIO pin naming convention was developed by the original 
 *  unknown author.  Although using defines for variables is always 
 *  a good idea for portability,  in this case the names are as specific 
 *  as the values, and thus lose their portability. Ultimately the pin 
 *  names should be changed to reflect the signal name only.  
 *
 *  The current naming convention is as follows.
 *
 *  P(port)(pin)_(function)_(signal)
 *
 *  port = (A-F)
 *  pin = (0-31)
 *  function = (PF|AF|AIN|BIN|CIN|DR|AOUT|BOUT)
 *  signal = signal name from datasheet
 *
 *  Remember that when in GPIO mode, AIN, BIN, CIN, and DR are inputs to 
 *  the GPIO peripheral module and represent outputs to the pin. 
 *  Similarly AOUT, and BOUT are outputs from the GPIO peripheral 
 *  module and represent inputs to the physical  pin in question. 
 *  Refer to the multiplexing table in the section titled "Signal 
 *  Descriptions and Pin Assignments" in the reference manual.
 */


/* FIXME: This list is not completed. The correct directions are
 * missing on some (many) pins
 */
#if defined(CONFIG_ARCH_IMX)

#define PA0_AIN_SPI2_CLK     ( GPIO_PORTA | GPIO_OUT | 0 )
#define PA0_AF_ETMTRACESYNC  ( GPIO_PORTA | GPIO_AF | 0 )
#define PA1_AOUT_SPI2_RXD    ( GPIO_PORTA | GPIO_IN | 1 )
#define PA1_PF_TIN           ( GPIO_PORTA | GPIO_PF | 1 )
#define PA2_PF_PWM0          ( GPIO_PORTA | GPIO_OUT | GPIO_PF | 2 )
#define PA3_PF_CSI_MCLK      ( GPIO_PORTA | GPIO_PF | 3 )
#define PA4_PF_CSI_D0        ( GPIO_PORTA | GPIO_PF | 4 )
#define PA5_PF_CSI_D1        ( GPIO_PORTA | GPIO_PF | 5 )
#define PA6_PF_CSI_D2        ( GPIO_PORTA | GPIO_PF | 6 )
#define PA7_PF_CSI_D3        ( GPIO_PORTA | GPIO_PF | 7 )
#define PA8_PF_CSI_D4        ( GPIO_PORTA | GPIO_PF | 8 )
#define PA9_PF_CSI_D5        ( GPIO_PORTA | GPIO_PF | 9 )
#define PA10_PF_CSI_D6       ( GPIO_PORTA | GPIO_PF | 10 )
#define PA11_PF_CSI_D7       ( GPIO_PORTA | GPIO_PF | 11 )
#define PA12_PF_CSI_VSYNC    ( GPIO_PORTA | GPIO_PF | 12 )
#define PA13_PF_CSI_HSYNC    ( GPIO_PORTA | GPIO_PF | 13 )
#define PA14_PF_CSI_PIXCLK   ( GPIO_PORTA | GPIO_PF | 14 )
#define PA15_PF_I2C_SDA      ( GPIO_PORTA | GPIO_OUT | GPIO_PF | 15 )
#define PA16_PF_I2C_SCL      ( GPIO_PORTA | GPIO_OUT | GPIO_PF | 16 )
#define PA17_AF_ETMTRACEPKT4 ( GPIO_PORTA | GPIO_AF | 17 )
#define PA17_AIN_SPI2_SS     ( GPIO_PORTA | GPIO_OUT | 17 )
#define PA18_AF_ETMTRACEPKT5 ( GPIO_PORTA | GPIO_AF | 18 )
#define PA19_AF_ETMTRACEPKT6 ( GPIO_PORTA | GPIO_AF | 19 )
#define PA20_AF_ETMTRACEPKT7 ( GPIO_PORTA | GPIO_AF | 20 )
#define PA21_PF_A0           ( GPIO_PORTA | GPIO_PF | 21 )
#define PA22_PF_CS4          ( GPIO_PORTA | GPIO_PF | 22 )
#define PA23_PF_CS5          ( GPIO_PORTA | GPIO_PF | 23 )
#define PA24_PF_A16          ( GPIO_PORTA | GPIO_PF | 24 )
#define PA24_AF_ETMTRACEPKT0 ( GPIO_PORTA | GPIO_AF | 24 )
#define PA25_PF_A17          ( GPIO_PORTA | GPIO_PF | 25 )
#define PA25_AF_ETMTRACEPKT1 ( GPIO_PORTA | GPIO_AF | 25 )
#define PA26_PF_A18          ( GPIO_PORTA | GPIO_PF | 26 )
#define PA26_AF_ETMTRACEPKT2 ( GPIO_PORTA | GPIO_AF | 26 )
#define PA27_PF_A19          ( GPIO_PORTA | GPIO_PF | 27 )
#define PA27_AF_ETMTRACEPKT3 ( GPIO_PORTA | GPIO_AF | 27 )
#define PA28_PF_A20          ( GPIO_PORTA | GPIO_PF | 28 )
#define PA28_AF_ETMPIPESTAT0 ( GPIO_PORTA | GPIO_AF | 28 )
#define PA29_PF_A21          ( GPIO_PORTA | GPIO_PF | 29 )
#define PA29_AF_ETMPIPESTAT1 ( GPIO_PORTA | GPIO_AF | 29 )
#define PA30_PF_A22          ( GPIO_PORTA | GPIO_PF | 30 )
#define PA30_AF_ETMPIPESTAT2 ( GPIO_PORTA | GPIO_AF | 30 )
#define PA31_PF_A23          ( GPIO_PORTA | GPIO_PF | 31 )
#define PA31_AF_ETMTRACECLK  ( GPIO_PORTA | GPIO_AF | 31 )
#define PB8_PF_SD_DAT0       ( GPIO_PORTB | GPIO_OUT | GPIO_PF | GPIO_PUEN | 8 )
#define PB8_AF_MS_PIO        ( GPIO_PORTB | GPIO_AF | 8 )
#define PB9_PF_SD_DAT1       ( GPIO_PORTB | GPIO_OUT | GPIO_PF | GPIO_PUEN  | 9 )
#define PB9_AF_MS_PI1        ( GPIO_PORTB | GPIO_AF | 9 )
#define PB10_PF_SD_DAT2      ( GPIO_PORTB | GPIO_OUT | GPIO_PF | GPIO_PUEN  | 10 )
#define PB10_AF_MS_SCLKI     ( GPIO_PORTB | GPIO_AF | 10 )
#define PB11_PF_SD_DAT3      ( GPIO_PORTB | GPIO_OUT | GPIO_PF | 11 )
#define PB11_AF_MS_SDIO      ( GPIO_PORTB | GPIO_AF | 11 )
#define PB12_PF_SD_CLK       ( GPIO_PORTB | GPIO_OUT | GPIO_PF | 12 )
#define PB12_AF_MS_SCLK0     ( GPIO_PORTB | GPIO_AF | 12 )
#define PB13_PF_SD_CMD       ( GPIO_PORTB | GPIO_OUT | GPIO_PF | GPIO_PUEN | 13 )
#define PB13_AF_MS_BS        ( GPIO_PORTB | GPIO_AF | 13 )
#define PB14_AF_SSI_RXFS     ( GPIO_PORTB | GPIO_AF | 14 )
#define PB15_AF_SSI_RXCLK    ( GPIO_PORTB | GPIO_AF | 15 )
#define PB16_AF_SSI_RXDAT    ( GPIO_PORTB | GPIO_IN | GPIO_AF | 16 )
#define PB17_AF_SSI_TXDAT    ( GPIO_PORTB | GPIO_OUT | GPIO_AF | 17 )
#define PB18_AF_SSI_TXFS     ( GPIO_PORTB | GPIO_AF | 18 )
#define PB19_AF_SSI_TXCLK    ( GPIO_PORTB | GPIO_AF | 19 )
#define PB20_PF_USBD_AFE     ( GPIO_PORTB | GPIO_PF | 20 )
#define PB21_PF_USBD_OE      ( GPIO_PORTB | GPIO_PF | 21 )
#define PB22_PFUSBD_RCV      ( GPIO_PORTB | GPIO_PF | 22 )
#define PB23_PF_USBD_SUSPND  ( GPIO_PORTB | GPIO_PF | 23 )
#define PB24_PF_USBD_VP      ( GPIO_PORTB | GPIO_PF | 24 )
#define PB25_PF_USBD_VM      ( GPIO_PORTB | GPIO_PF | 25 )
#define PB26_PF_USBD_VPO     ( GPIO_PORTB | GPIO_PF | 26 )
#define PB27_PF_USBD_VMO     ( GPIO_PORTB | GPIO_PF | 27 )
#define PB28_PF_UART2_CTS    ( GPIO_PORTB | GPIO_OUT | GPIO_PF | 28 )
#define PB29_PF_UART2_RTS    ( GPIO_PORTB | GPIO_IN | GPIO_PF | 29 )
#define PB30_PF_UART2_TXD    ( GPIO_PORTB | GPIO_OUT | GPIO_PF | 30 )
#define PB31_PF_UART2_RXD    ( GPIO_PORTB | GPIO_IN | GPIO_PF | 31 )
#define PC3_PF_SSI_RXFS      ( GPIO_PORTC | GPIO_PF | 3 )
#define PC4_PF_SSI_RXCLK     ( GPIO_PORTC | GPIO_PF | 4 )
#define PC5_PF_SSI_RXDAT     ( GPIO_PORTC | GPIO_IN | GPIO_PF | 5 )
#define PC6_PF_SSI_TXDAT     ( GPIO_PORTC | GPIO_OUT | GPIO_PF | 6 )
#define PC7_PF_SSI_TXFS      ( GPIO_PORTC | GPIO_PF | 7 )
#define PC8_PF_SSI_TXCLK     ( GPIO_PORTC | GPIO_PF | 8 )
#define PC9_PF_UART1_CTS     ( GPIO_PORTC | GPIO_OUT | GPIO_PF | 9 )
#define PC10_PF_UART1_RTS    ( GPIO_PORTC | GPIO_IN | GPIO_PF | 10 )
#define PC11_PF_UART1_TXD    ( GPIO_PORTC | GPIO_OUT | GPIO_PF | 11 )
#define PC12_PF_UART1_RXD    ( GPIO_PORTC | GPIO_IN | GPIO_PF | 12 )
#define PC13_PF_SPI1_SPI_RDY ( GPIO_PORTC | GPIO_PF | 13 )
#define PC14_PF_SPI1_SCLK    ( GPIO_PORTC | GPIO_PF | 14 )
#define PC15_PF_SPI1_SS      ( GPIO_PORTC | GPIO_PF | 15 )
#define PC16_PF_SPI1_MISO    ( GPIO_PORTC | GPIO_PF | 16 )
#define PC17_PF_SPI1_MOSI    ( GPIO_PORTC | GPIO_PF | 17 )
#define PC24_BIN_UART3_RI    ( GPIO_PORTC | GPIO_OUT | GPIO_BIN | 24 )
#define PC25_BIN_UART3_DSR   ( GPIO_PORTC | GPIO_OUT | GPIO_BIN | 25 )
#define PC26_AOUT_UART3_DTR  ( GPIO_PORTC | GPIO_IN | 26 )
#define PC27_BIN_UART3_DCD   ( GPIO_PORTC | GPIO_OUT | GPIO_BIN | 27 )
#define PC28_BIN_UART3_CTS   ( GPIO_PORTC | GPIO_OUT | GPIO_BIN | 28 )
#define PC29_AOUT_UART3_RTS  ( GPIO_PORTC | GPIO_IN | 29 )
#define PC30_BIN_UART3_TX    ( GPIO_PORTC | GPIO_BIN | 30 )
#define PC31_AOUT_UART3_RX   ( GPIO_PORTC | GPIO_IN | 31)
#define PD6_PF_LSCLK         ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 6 )
#define PD7_PF_REV           ( GPIO_PORTD | GPIO_PF | 7 )
#define PD7_AF_UART2_DTR     ( GPIO_PORTD | GPIO_IN | GPIO_AF | 7 )
#define PD7_AIN_SPI2_SCLK    ( GPIO_PORTD | GPIO_AIN | 7 )
#define PD8_PF_CLS           ( GPIO_PORTD | GPIO_PF | 8 )
#define PD8_AF_UART2_DCD     ( GPIO_PORTD | GPIO_OUT | GPIO_AF | 8 )
#define PD8_AIN_SPI2_SS      ( GPIO_PORTD | GPIO_AIN | 8 )
#define PD9_PF_PS            ( GPIO_PORTD | GPIO_PF | 9 )
#define PD9_AF_UART2_RI      ( GPIO_PORTD | GPIO_OUT | GPIO_AF | 9 )
#define PD9_AOUT_SPI2_RXD    ( GPIO_PORTD | GPIO_IN | 9 )
#define PD10_PF_SPL_SPR      ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 10 )
#define PD10_AF_UART2_DSR    ( GPIO_PORTD | GPIO_OUT | GPIO_AF | 10 )
#define PD10_AIN_SPI2_TXD    ( GPIO_PORTD | GPIO_OUT | 10 )
#define PD11_PF_CONTRAST     ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 11 )
#define PD12_PF_ACD_OE       ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 12 )
#define PD13_PF_LP_HSYNC     ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 13 )
#define PD14_PF_FLM_VSYNC    ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 14 )
#define PD15_PF_LD0          ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 15 )
#define PD16_PF_LD1          ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 16 )
#define PD17_PF_LD2          ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 17 )
#define PD18_PF_LD3          ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 18 )
#define PD19_PF_LD4          ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 19 )
#define PD20_PF_LD5          ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 20 )
#define PD21_PF_LD6          ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 21 )
#define PD22_PF_LD7          ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 22 )
#define PD23_PF_LD8          ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 23 )
#define PD24_PF_LD9          ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 24 )
#define PD25_PF_LD10         ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 25 )
#define PD26_PF_LD11         ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 26 )
#define PD27_PF_LD12         ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 27 )
#define PD28_PF_LD13         ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 28 )
#define PD29_PF_LD14         ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 29 )
#define PD30_PF_LD15         ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 30 )
#define PD31_PF_TMR2OUT      ( GPIO_PORTD | GPIO_PF | 31 )
#define PD31_BIN_SPI2_TXD    ( GPIO_PORTD | GPIO_BIN | 31 )

#elif defined (CONFIG_ARCH_IMX21)
   
#define PE14_PF_UART1_CTS       ( GPIO_PORTE | 14 | GPIO_PF | GPIO_OUT )
#define PE15_PF_UART1_RTS       ( GPIO_PORTE | 15 | GPIO_PF | GPIO_IN )
#define PE12_PF_UART1_TXD       ( GPIO_PORTE | 12 | GPIO_PF | GPIO_OUT )
#define PE13_PF_UART1_RXD       ( GPIO_PORTE | 13 | GPIO_PF | GPIO_IN )

#define PA5_PF_LSCLK            ( GPIO_PORTA | 5 | GPIO_PF | GPIO_OUT )
#define PA6_PF_LD0              ( GPIO_PORTA | 6 | GPIO_PF | GPIO_OUT )
#define PA7_PF_LD1              ( GPIO_PORTA | 7 | GPIO_PF | GPIO_OUT )
#define PA8_PF_LD2              ( GPIO_PORTA | 8 | GPIO_PF | GPIO_OUT )
#define PA9_PF_LD3              ( GPIO_PORTA | 9 | GPIO_PF | GPIO_OUT )
#define PA10_PF_LD4             ( GPIO_PORTA | 10 | GPIO_PF | GPIO_OUT )
#define PA11_PF_LD5             ( GPIO_PORTA | 11 | GPIO_PF | GPIO_OUT )
#define PA12_PF_LD6             ( GPIO_PORTA | 12 | GPIO_PF | GPIO_OUT )
#define PA13_PF_LD7             ( GPIO_PORTA | 13 | GPIO_PF | GPIO_OUT )
#define PA14_PF_LD8             ( GPIO_PORTA | 14 | GPIO_PF | GPIO_OUT )
#define PA15_PF_LD9             ( GPIO_PORTA | 15 | GPIO_PF | GPIO_OUT )
#define PA16_PF_LD10            ( GPIO_PORTA | 16 | GPIO_PF | GPIO_OUT )
#define PA17_PF_LD11            ( GPIO_PORTA | 17 | GPIO_PF | GPIO_OUT )
#define PA18_PF_LD12            ( GPIO_PORTA | 18 | GPIO_PF | GPIO_OUT )
#define PA19_PF_LD13            ( GPIO_PORTA | 19 | GPIO_PF | GPIO_OUT )
#define PA20_PF_LD14            ( GPIO_PORTA | 20 | GPIO_PF | GPIO_OUT )
#define PA21_PF_LD15            ( GPIO_PORTA | 21 | GPIO_PF | GPIO_OUT )
#define PA22_PF_LD16            ( GPIO_PORTA | 22 | GPIO_PF | GPIO_OUT )
#define PA23_PF_LD17            ( GPIO_PORTA | 23 | GPIO_PF | GPIO_OUT )
#define PA24_PF_REV             ( GPIO_PORTA | 24 | GPIO_PF | GPIO_OUT )
#define PA25_PF_CLS             ( GPIO_PORTA | 25 | GPIO_PF | GPIO_OUT )
#define PA26_PF_PS              ( GPIO_PORTA | 26 | GPIO_PF | GPIO_OUT )
#define PA27_PF_SPL_SPR         ( GPIO_PORTA | 27 | GPIO_PF | GPIO_OUT )
#define PA28_PF_LP_HSYNC        ( GPIO_PORTA | 28 | GPIO_PF | GPIO_OUT )
#define PA29_PF_FLM_VSYNC       ( GPIO_PORTA | 29 | GPIO_PF | GPIO_OUT )
#define PA30_PF_CONTRAST        ( GPIO_PORTA | 30 | GPIO_PF | GPIO_OUT )
#define PA31_PF_ACD_OE          ( GPIO_PORTA | 31 | GPIO_PF | GPIO_OUT )
#define PE18_PF_SD_DAT0   ( GPIO_PORTE | GPIO_OUT | GPIO_PF | GPIO_PUEN | 18 )
#define PE19_PF_SD_DAT1   ( GPIO_PORTE | GPIO_OUT | GPIO_PF | GPIO_PUEN | 19 )
#define PE20_PF_SD_DAT2   ( GPIO_PORTE | GPIO_OUT | GPIO_PF | GPIO_PUEN | 20 )
#define PE21_PF_SD_DAT3   ( GPIO_PORTE | GPIO_OUT | GPIO_PF | GPIO_PUEN | 21 )
#define PE23_PF_SD_CLK    ( GPIO_PORTE | GPIO_OUT | GPIO_PF | 23 )
#define PE22_PF_SD_CMD    ( GPIO_PORTE | GPIO_OUT | GPIO_PF | GPIO_PUEN | 22 )
#define PD18_PF_I2C_SDA   ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 18 )
#define PD17_PF_I2C_SCL   ( GPIO_PORTD | GPIO_OUT | GPIO_PF | 17 )
#define PC27_PF_SSI2_CLK  ( GPIO_PORTC | GPIO_IN  | GPIO_PF | 27 )
#define PC26_PF_SSI2_TX   ( GPIO_PORTC | GPIO_OUT | GPIO_PF | 26 )
#define PC25_PF_SSI2_RX   ( GPIO_PORTC | GPIO_IN  | GPIO_PF | 25 )
#define PC24_PF_SSI2_FS   ( GPIO_PORTC | GPIO_IN  | GPIO_PF | 24 )
#define PC23_PF_SSI1_CLK  ( GPIO_PORTC | GPIO_IN  | GPIO_PF | 23 )
#define PC22_PF_SSI1_TX   ( GPIO_PORTC | GPIO_OUT | GPIO_PF | 22 )
#define PC21_PF_SSI1_RX   ( GPIO_PORTC | GPIO_IN  | GPIO_PF | 21 )
#define PC20_PF_SSI1_FS   ( GPIO_PORTC | GPIO_IN  | GPIO_PF | 20 )
#endif /* defined(CONFIG_ARCH_IMX21) */


/*
 * PWM controller
 */
#define PWMC	__REG(IMX_PWM_BASE + 0x00)	/* PWM Control Register		*/
#define PWMS	__REG(IMX_PWM_BASE + 0x04)	/* PWM Sample Register		*/
#define PWMP	__REG(IMX_PWM_BASE + 0x08)	/* PWM Period Register		*/
#define PWMCNT	__REG(IMX_PWM_BASE + 0x0C)	/* PWM Counter Register		*/

#define PWMC_HCTR		(0x01<<18)		/* Halfword FIFO Data Swapping	*/
#define PWMC_BCTR		(0x01<<17)		/* Byte FIFO Data Swapping	*/
#define PWMC_SWR		(0x01<<16)		/* Software Reset		*/
#define PWMC_CLKSRC		(0x01<<15)		/* Clock Source			*/
#define PWMC_PRESCALER(x)	(((x) & 0x7F) << 8)	/* PRESCALER			*/
#define PWMC_IRQ		(0x01<< 7)		/* Interrupt Request		*/
#define PWMC_IRQEN		(0x01<< 6)		/* Interrupt Request Enable	*/
#define PWMC_FIFOAV		(0x01<< 5)		/* FIFO Available		*/
#define PWMC_EN			(0x01<< 4)		/* Enables/Disables the PWM	*/
#define PWMC_REPEAT(x)		(((x) & 0x03) << 2)	/* Sample Repeats		*/
#define PWMC_CLKSEL(x)		(((x) & 0x03) << 0)	/* Clock Selection		*/

#define PWMS_SAMPLE(x)		((x) & 0xFFFF)		/* Contains a two-sample word	*/
#define PWMP_PERIOD(x)		((x) & 0xFFFF)		/* Represents the PWM's period	*/
#define PWMC_COUNTER(x)		((x) & 0xFFFF)		/* Represents the current count value	*/

/*
 *  Watchdog Timer
 */
#if defined(CONFIG_ARCH_IMX)
# define WCR         __REG(IMX_WDT_BASE + 0x00)
# define WSR         __REG(IMX_WDT_BASE + 0x04)
# define WSTR        __REG(IMX_WDT_BASE + 0x08)
# define WCR_WHALT   (1 << 16)
# define WCR_WT(x)   (((x) & 0x7f) << 8)
# define WCR_WT_MASK (0x7f << 8)
# define WCR_WIE     (1 << 4)
# define WCR_TMD     (1 << 3)
# define WCR_SWR     (1 << 2)
# define WCR_WDEC    (1 << 1)
# define WCR_WDE     (1 << 0)
# define WSTR_TINT   (1 << 8)
# define WSTR_TOUT   (1 << 0)
#elif defined(CONFIG_ARCH_IMX21)
# define WCR         __REG16(IMX_WDT_BASE + 0x00)
# define WSR         __REG16(IMX_WDT_BASE + 0x02)
# define WRSR        __REG16(IMX_WDT_BASE + 0x04)
# define WCR_WT(x)   (((x) & 0xff) << 8)
# define WCR_WT_MASK (0xff << 8)
# define WCR_WDA     (1 << 5)
# define WCR_SRS     (1 << 4)
# define WCR_WRE     (1 << 3)
# define WCR_WDE     (1 << 2)
# define WCR_WDBG    (1 << 1)
# define WCR_WDZST   (1 << 0)
# define WRSR_PWR    (1 << 4)
# define WRSR_EXT    (1 << 3)
# define WRSR_TOUT   (1 << 1)
# define WRSR_SFTW   (1 << 0)
#endif /* defined(CONFIG_ARCH_IMX21) */


/*
 *  DMA Controller
 */
#define DCR     __REG(IMX_DMAC_BASE + 0x00)
#define DISR    __REG(IMX_DMAC_BASE + 0x04)
#define DIMR    __REG(IMX_DMAC_BASE + 0x08)
#define DBTOSR  __REG(IMX_DMAC_BASE + 0x0c)
#define DRTOSR  __REG(IMX_DMAC_BASE + 0x10)
#define DSESR   __REG(IMX_DMAC_BASE + 0x14)
#define DBOSR   __REG(IMX_DMAC_BASE + 0x18)
#define DBTOCR  __REG(IMX_DMAC_BASE + 0x1c)
#define WSRA    __REG(IMX_DMAC_BASE + 0x40)
#define XSRA    __REG(IMX_DMAC_BASE + 0x44)
#define YSRA    __REG(IMX_DMAC_BASE + 0x48)
#define WSRB    __REG(IMX_DMAC_BASE + 0x4c)
#define XSRB    __REG(IMX_DMAC_BASE + 0x50)
#define YSRB    __REG(IMX_DMAC_BASE + 0x54)
#define SAR(x)    __REG2( IMX_DMAC_BASE + 0x80, (x) << 6)
#define DAR(x)    __REG2( IMX_DMAC_BASE + 0x84, (x) << 6)
#define CNTR(x)   __REG2( IMX_DMAC_BASE + 0x88, (x) << 6)
#define CCR(x)    __REG2( IMX_DMAC_BASE + 0x8c, (x) << 6)
#define RSSR(x)   __REG2( IMX_DMAC_BASE + 0x90, (x) << 6)
#define BLR(x)    __REG2( IMX_DMAC_BASE + 0x94, (x) << 6)
#define RTOR(x)   __REG2( IMX_DMAC_BASE + 0x98, (x) << 6)  /* RTOR and BUCR */
#define BUCR(x)   __REG2( IMX_DMAC_BASE + 0x98, (x) << 6)  /*  are the same */
#define CHCNTR(x) __REG2( IMX_DMAC_BASE + 0x9c, (x) << 6)  /* On IMX21 only */

#define DCR_DAM            (1<<2)  /* On IMX21 only */
#define DCR_DRST           (1<<1)
#define DCR_DEN            (1<<0)
#define DBTOCR_EN          (1<<15)
#define DBTOCR_CNT(x)      ((x) & 0x7fff )
#define CNTR_CNT(x)        ((x) & 0xffffff )
#define CCR_ACRPT          ( 0x1 << 14 )  /* On IMX21 only */
#define CCR_DMOD_LINEAR    ( 0x0 << 12 )
#define CCR_DMOD_2D        ( 0x1 << 12 )
#define CCR_DMOD_FIFO      ( 0x2 << 12 )
#define CCR_DMOD_EOBFIFO   ( 0x3 << 12 )
#define CCR_SMOD_LINEAR    ( 0x0 << 10 )
#define CCR_SMOD_2D        ( 0x1 << 10 )
#define CCR_SMOD_FIFO      ( 0x2 << 10 )
#define CCR_SMOD_EOBFIFO   ( 0x3 << 10 )
#define CCR_MDIR_INC       (0<<9)
#define CCR_MDIR_DEC       (1<<9)
#define CCR_MSEL_A         (0<<8)
#define CCR_MSEL_B         (1<<8)
#define CCR_DSIZ_32        ( 0x0 << 6 )
#define CCR_DSIZ_8         ( 0x1 << 6 )
#define CCR_DSIZ_16        ( 0x2 << 6 )
#define CCR_SSIZ_32        ( 0x0 << 4 )
#define CCR_SSIZ_8         ( 0x1 << 4 )
#define CCR_SSIZ_16        ( 0x2 << 4 )
#define CCR_REN            (1<<3)
#define CCR_RPT            (1<<2)
#define CCR_FRC            (1<<1)
#define CCR_CEN            (1<<0)
#define RTOR_EN            (1<<15)
#define RTOR_CLK           (1<<14)
#define RTOR_PSC           (1<<13)

/*
 *  General purpose timers
 */
#define IMX_TCTL(x)        __REG( 0x00 + (x))
#if defined(CONFIG_ARCH_IMX)
# define TCTL_SWR           (1<<15)
# define TCTL_FRR           (1<<8)
# define TCTL_CAP_RIS       (1<<6)
# define TCTL_CAP_FAL       (2<<6)
# define TCTL_CAP_RIS_FAL   (3<<6)
# define TCTL_OM            (1<<5)
# define TCTL_IRQEN         (1<<4)
# define TCTL_CLK_PCLK1     (1<<1)
# define TCTL_CLK_PCLK1_16  (2<<1)
# define TCTL_CLK_TIN       (3<<1)
# define TCTL_CLK_32        (4<<1)
# define TCTL_TEN           (1<<0)
#elif defined(CONFIG_ARCH_IMX21)
# define TCTL_SWR           (1 << 15)
# define TCTL_CC            (1 << 10)
# define TCTL_OM            (1 << 9)
# define TCTL_FRR           (1 << 8)
# define TCTL_CAP_RIS       (1 << 6)
# define TCTL_CAP_FAL       (2 << 6)
# define TCTL_CAP_RIS_FAL   (3 << 6)
# define TCTL_CAPTEN        (1 << 5)
# define TCTL_COMPEN        (1 << 4)
# define TCTL_IRQEN         (1 << 4)
# define TCTL_CLK_PCLK1     (1 << 1)
# define TCTL_CLK_PCLK1_4   (2 << 1)
# define TCTL_CLK_TIN       (3 << 1)
# define TCTL_CLK_32        (4 << 1)
# define TCTL_TEN           (1 << 0)
#endif

#define IMX_TPRER(x)       __REG( 0x04 + (x))
#define IMX_TCMP(x)        __REG( 0x08 + (x))
#define IMX_TCR(x)         __REG( 0x0C + (x))
#define IMX_TCN(x)         __REG( 0x10 + (x))
#define IMX_TSTAT(x)       __REG( 0x14 + (x))
#define TSTAT_CAPT         (1<<1)
#define TSTAT_COMP         (1<<0)



/* 
 * SSP Module.
 */
#define SSP_RX_REG(x)        __REG(IMX_SPI_BASE(x) + 0x00)
#define SSP_TX_REG(x)        __REG(IMX_SPI_BASE(x) + 0x04)
#define SSP_CTRL_REG(x)      __REG(IMX_SPI_BASE(x) + 0x08)
#define SSP_INT_REG(x)       __REG(IMX_SPI_BASE(x) + 0x0c)
#define SSP_TEST_REG(x)      __REG(IMX_SPI_BASE(x) + 0x10)
#define SSP_PER_REG(x)       __REG(IMX_SPI_BASE(x) + 0x14)
#define SSP_DMA_REG(x)       __REG(IMX_SPI_BASE(x) + 0x18)
#define SSP_RESET_REG(x)     __REG(IMX_SPI_BASE(x) + 0x1c)

#if defined(CONFIG_ARCH_IMX)

# define SSP_RATE_DIV4       (0 << 13)
# define SSP_RATE_DIV8       (1 << 13)
# define SSP_RATE_DIV16      (2 << 13)
# define SSP_RATE_DIV32      (3 << 13)
# define SSP_RATE_DIV64      (4 << 13)
# define SSP_RATE_DIV128     (5 << 13)
# define SSP_RATE_DIV256     (6 << 13)
# define SSP_RATE_DIV512     (7 << 13)
# define SSP_MODE_MASTER     (1 << 10)
# define SSP_MODE_SLAVE      (0 << 10)
# define SSP_ENABLE          (1 << 9)
# define SSP_DISABLE         (0 << 9)
# define SSP_XCH             (1 << 8)
# define SSP_SS_POL_HIGH     (1 << 7)
# define SSP_SS_POL_LOW      (0 << 7)
# define SSP_SS_PULSE        (1 << 6)
# define SSP_PHA0            (0 << 5)
# define SSP_PHA1            (1 << 5)
# define SSP_POL0            (0 << 4)
# define SSP_POL1            (1 << 4)
# define SSP_WS(x)           ((x-1) & 0xf)

#elif defined(CONFIG_ARCH_IMX21)

# define SSP_BURST           (1 << 23)
# define SSP_SDHC            (1 << 22)
# define SSP_SWAP            (1 << 21)
# define SSP_CS_SS0          (0 << 19)
# define SSP_CS_SS1          (1 << 19)
# define SSP_CS_SS2          (2 << 19)
# define SSP_RATE_DIV3       (1 << 14)
# define SSP_RATE_DIV4       (2 << 14)
# define SSP_RATE_DIV6       (3 << 14)
# define SSP_RATE_DIV8       (4 << 14)
# define SSP_RATE_DIV12      (5 << 14)
# define SSP_RATE_DIV16      (6 << 14)
# define SSP_RATE_DIV24      (7 << 14)
# define SSP_RATE_DIV32      (8 << 14)
# define SSP_RATE_DIV48      (9 << 14)
# define SSP_RATE_DIV64      (10 << 14)
# define SSP_RATE_DIV96      (11 << 14)
# define SSP_RATE_DIV128     (12 << 14)
# define SSP_RATE_DIV192     (13 << 14)
# define SSP_RATE_DIV256     (14 << 14)
# define SSP_RATE_DIV384     (15 << 14)
# define SSP_RATE_DIV512     (16 << 14)
# define SSP_RATE_DIV768     (17 << 14)
# define SSP_RATE_DIV1024    (18 << 14)
# define SSP_DR_IGNORE       (0 << 12)
# define SSP_DR_EDGE         (1 << 12)
# define SSP_DR_LEVEL        (2 << 12)
# define SSP_MODE_MASTER     (1 << 11)
# define SSP_MODE_SLAVE      (0 << 11)
# define SSP_ENABLE          (1 << 10)
# define SSP_DISABLE         (0 << 10)
# define SSP_XCH             (1 << 9)
# define SSP_SS_POL_HIGH     (1 << 8)
# define SSP_SS_POL_LOW      (0 << 8)
# define SSP_SS_PULSE        (1 << 7)
# define SSP_PHA0            (0 << 6)
# define SSP_PHA1            (1 << 6)
# define SSP_POL0            (0 << 5)
# define SSP_POL1            (1 << 5)
# define SSP_WS(x)           ((x-1) & 0x1f)

#endif /* defined(CONFIG_ARCH_IMX21) */

#if defined(CONFIG_ARCH_IMX)

# define SSP_INT_BOEN        (1 << 15)
# define SSP_INT_ROEN        (1 << 14)
# define SSP_INT_RFEN        (1 << 13)
# define SSP_INT_RHEN        (1 << 12)
# define SSP_INT_RREN        (1 << 11)
# define SSP_INT_TFEN        (1 << 10)
# define SSP_INT_THEN        (1 << 9)
# define SSP_INT_TEEN        (1 << 8)
# define SSP_INT_BO          (1 << 7)
# define SSP_INT_RO          (1 << 6)
# define SSP_INT_RF          (1 << 5)
# define SSP_INT_RH          (1 << 4)
# define SSP_INT_RR          (1 << 3)
# define SSP_INT_TF          (1 << 2)
# define SSP_INT_TH          (1 << 1)
# define SSP_INT_TE          (1 << 0)

#elif defined(CONFIG_ARCH_IMX21)

# define SSP_INT_BOEN        (1 << 17)
# define SSP_INT_ROEN        (1 << 16)
# define SSP_INT_RFEN        (1 << 15)
# define SSP_INT_RHEN        (1 << 14)
# define SSP_INT_RREN        (1 << 13)
# define SSP_INT_TSHFEEN     (1 << 12)
# define SSP_INT_TFEN        (1 << 11)
# define SSP_INT_THEN        (1 << 10)
# define SSP_INT_TEEN        (1 << 9)
# define SSP_INT_BO          (1 << 8)
# define SSP_INT_RO          (1 << 7)
# define SSP_INT_RF          (1 << 6)
# define SSP_INT_RH          (1 << 5)
# define SSP_INT_RR          (1 << 4)
# define SSP_INT_TSHFE       (1 << 3)
# define SSP_INT_TF          (1 << 2)
# define SSP_INT_TH          (1 << 1)
# define SSP_INT_TE          (1 << 0)

#endif /* defined(CONFIG_ARCH_IMX21) */

#define SSP_TEST_LOOPBACK   (1 << 14)
#define SSP_PER_CLK_BIT     (0 << 15)
#define SSP_PER_CLK_32KHZ   (1 << 15)
#define SSP_PER_WAIT(x)     ((x) & 0x7fff)
#define SSP_RESET_START     (1 << 0)



#if defined(CONFIG_ARCH_IMX)

/*
 * SSI Module
 */
#define SSI_STX   __REG(IMX_SSI_BASE + 0x00)
#define SSI_SRX   __REG(IMX_SSI_BASE + 0x04)
#define SSI_SCSR  __REG(IMX_SSI_BASE + 0x08)
#define SSI_STCR  __REG(IMX_SSI_BASE + 0x0c)
#define SSI_SRCR  __REG(IMX_SSI_BASE + 0x10)
#define SSI_STCCR __REG(IMX_SSI_BASE + 0x14)
#define SSI_SRCCR __REG(IMX_SSI_BASE + 0x18)
#define SSI_STSR  __REG(IMX_SSI_BASE + 0x1c)
#define SSI_SFCSR __REG(IMX_SSI_BASE + 0x20)
#define SSI_SOR   __REG(IMX_SSI_BASE + 0x28)

#define SSI_SCSR_SCR         (1 << 15)
#define SSI_SCSR_MODE_NORM   (0 << 13)
#define SSI_SCSR_MODE_MSTR   (1 << 13)
#define SSI_SCSR_MODE_SLAVE  (2 << 13)
#define SSI_SCSR_SYN         (1 << 12)
#define SSI_SCSR_NET         (1 << 11)
#define SSI_SCSR_RE          (1 << 10)
#define SSI_SCSR_TE          (1 <<  9)
#define SSI_SCSR_EN          (1 <<  8)
#define SSI_SCSR_RDR         (1 <<  7)
#define SSI_SCSR_TDE         (1 <<  6)
#define SSI_SCSR_ROE         (1 <<  5)
#define SSI_SCSR_TUE         (1 <<  4)
#define SSI_SCSR_TFS         (1 <<  3)
#define SSI_SCSR_RFS         (1 <<  2)
#define SSI_SCSR_RFF         (1 <<  1)
#define SSI_SCSR_TFE         (1 <<  0)

#define SSI_STCR_TXBIT0      (1 << 10)
#define SSI_STCR_TDMAE       (1 <<  9)
#define SSI_STCR_TIE         (1 <<  8)
#define SSI_STCR_TFEN        (1 <<  7)
#define SSI_STCR_TFDIR       (1 <<  6)
#define SSI_STCR_TXDIR       (1 <<  5)
#define SSI_STCR_TSHFD       (1 <<  4)
#define SSI_STCR_TSCKP       (1 <<  3)
#define SSI_STCR_TFSI        (1 <<  2)
#define SSI_STCR_TFSL        (1 <<  1)
#define SSI_STCR_TEFS        (1 <<  0)

#define SSI_SRCR_RXBIT0      (1 << 10)
#define SSI_SRCR_RDMAE       (1 <<  9)
#define SSI_SRCR_RIE         (1 <<  8)
#define SSI_SRCR_RFEN        (1 <<  7)
#define SSI_SRCR_RFDIR       (1 <<  6)
#define SSI_SRCR_RXDIR       (1 <<  5)
#define SSI_SRCR_RSHFD       (1 <<  4)
#define SSI_SRCR_RSCKP       (1 <<  3)
#define SSI_SRCR_RFSI        (1 <<  2)
#define SSI_SRCR_RFSL        (1 <<  1)
#define SSI_SRCR_REFS        (1 <<  0)

#define SSI_STCCR_PS         (1 << 15)
#define SSI_STCCR_WL8        (0 << 13)
#define SSI_STCCR_WL10       (1 << 13)
#define SSI_STCCR_WL12       (2 << 13)
#define SSI_STCCR_WL16       (3 << 13)
#define SSI_STCCR_DC(x)      (((x) & 0x1f) << 8)
#define SSI_STCCR_PM(x)      (((x) & 0xff) << 0)

#define SSI_SRCCR_PS         (1 << 15)
#define SSI_SRCCR_WL8        (0 << 13)
#define SSI_SRCCR_WL10       (1 << 13)
#define SSI_SRCCR_WL12       (2 << 13)
#define SSI_SRCCR_WL16       (3 << 13)
#define SSI_SRCCR_DC(x)      (((x) & 0x1f) << 8)
#define SSI_SRCCR_PM(x)      (((x) & 0xff) << 0)

#define SSI_SFCSR_RFCNT(x)   (((x) & 0xf) << 12)
#define SSI_SFCSR_TFCNT(x)   (((x) & 0xf) <<  8)
#define SSI_SFCSR_RFWM(x)    (((x) & 0xf) <<  4)
#define SSI_SFCSR_TFWM(x)    (((x) & 0xf) <<  0)

#define SSI_SOR_CLKOFF       (1 << 6)
#define SSI_SOR_RXCLR        (1 << 5)
#define SSI_SOR_TXCLR        (1 << 4)
#define SSI_SOR_SYNRST       (1 << 0)

#elif defined(CONFIG_ARCH_IMX21)

#define SSI1_STX0   __REG(IMX_SSI1_BASE + 0x00)
#define SSI1_STX0_PHYS   __PHYS_REG(IMX_SSI1_BASE + 0x00)
#define SSI1_STX1   __REG(IMX_SSI1_BASE + 0x04)
#define SSI1_STX1_PHYS   __PHYS_REG(IMX_SSI1_BASE + 0x04)
#define SSI1_SRX0   __REG(IMX_SSI1_BASE + 0x08)
#define SSI1_SRX0_PHYS   __PHYS_REG(IMX_SSI1_BASE + 0x08)
#define SSI1_SRX1   __REG(IMX_SSI1_BASE + 0x0c)
#define SSI1_SRX1_PHYS   __PHYS_REG(IMX_SSI1_BASE + 0x0c)
#define SSI1_SCR    __REG(IMX_SSI1_BASE + 0x10)
#define SSI1_SISR   __REG(IMX_SSI1_BASE + 0x14)
#define SSI1_SIER   __REG(IMX_SSI1_BASE + 0x18)
#define SSI1_STCR   __REG(IMX_SSI1_BASE + 0x1c)
#define SSI1_SRCR   __REG(IMX_SSI1_BASE + 0x20)
#define SSI1_STCCR  __REG(IMX_SSI1_BASE + 0x24)
#define SSI1_SRCCR  __REG(IMX_SSI1_BASE + 0x28)
#define SSI1_SFCSR  __REG(IMX_SSI1_BASE + 0x2c)
#define SSI1_STR    __REG(IMX_SSI1_BASE + 0x30)
#define SSI1_SOR    __REG(IMX_SSI1_BASE + 0x34)
#define SSI1_SACNT  __REG(IMX_SSI1_BASE + 0x38)
#define SSI1_SACADD __REG(IMX_SSI1_BASE + 0x3c)
#define SSI1_SACDAT __REG(IMX_SSI1_BASE + 0x40)
#define SSI1_SATAG  __REG(IMX_SSI1_BASE + 0x44)
#define SSI1_STMSK  __REG(IMX_SSI1_BASE + 0x48)
#define SSI1_SRMSK  __REG(IMX_SSI1_BASE + 0x4c)

#define SSI2_STX0   __REG(IMX_SSI2_BASE + 0x00)
#define SSI2_STX0_PHYS   __PHYS_REG(IMX_SSI2_BASE + 0x00)
#define SSI2_STX1   __REG(IMX_SSI2_BASE + 0x04)
#define SSI2_STX1_PHYS   __PHYS_REG(IMX_SSI2_BASE + 0x04)
#define SSI2_SRX0   __REG(IMX_SSI2_BASE + 0x08)
#define SSI2_SRX0_PHYS   __PHYS_REG(IMX_SSI2_BASE + 0x08)
#define SSI2_SRX1   __REG(IMX_SSI2_BASE + 0x0c)
#define SSI2_SRX1_PHYS   __PHYS_REG(IMX_SSI2_BASE + 0x0c)
#define SSI2_SCR    __REG(IMX_SSI2_BASE + 0x10)
#define SSI2_SISR   __REG(IMX_SSI2_BASE + 0x14)
#define SSI2_SIER   __REG(IMX_SSI2_BASE + 0x18)
#define SSI2_STCR   __REG(IMX_SSI2_BASE + 0x1c)
#define SSI2_SRCR   __REG(IMX_SSI2_BASE + 0x20)
#define SSI2_STCCR  __REG(IMX_SSI2_BASE + 0x24)
#define SSI2_SRCCR  __REG(IMX_SSI2_BASE + 0x28)
#define SSI2_SFCSR  __REG(IMX_SSI2_BASE + 0x2c)
#define SSI2_STR    __REG(IMX_SSI2_BASE + 0x30)
#define SSI2_SOR    __REG(IMX_SSI2_BASE + 0x34)
#define SSI2_SACNT  __REG(IMX_SSI2_BASE + 0x38)
#define SSI2_SACADD __REG(IMX_SSI2_BASE + 0x3c)
#define SSI2_SACDAT __REG(IMX_SSI2_BASE + 0x40)
#define SSI2_SATAG  __REG(IMX_SSI2_BASE + 0x44)
#define SSI2_STMSK  __REG(IMX_SSI2_BASE + 0x48)
#define SSI2_SRMSK  __REG(IMX_SSI2_BASE + 0x4c)

#define SSI_SCR_CLK_IST        (1 << 9)
#define SSI_SCR_TCH_EN         (1 << 8)
#define SSI_SCR_SYS_CLK_EN     (1 << 7)
#define SSI_SCR_I2S_MODE_NORM  (0 << 5)
#define SSI_SCR_I2S_MODE_MSTR  (1 << 5)
#define SSI_SCR_I2S_MODE_SLAVE (2 << 5)
#define SSI_SCR_SYN            (1 << 4)
#define SSI_SCR_NET            (1 << 3)
#define SSI_SCR_RE             (1 << 2)
#define SSI_SCR_TE             (1 << 1)
#define SSI_SCR_SSIEN          (1 << 0)

#define SSI_SISR_CMDAU         (1 << 18)
#define SSI_SISR_CMDDU         (1 << 17)
#define SSI_SISR_RXT           (1 << 16)
#define SSI_SISR_RDR1          (1 << 15)
#define SSI_SISR_RDR0          (1 << 14)
#define SSI_SISR_TDE1          (1 << 13)
#define SSI_SISR_TDE0          (1 << 12)
#define SSI_SISR_ROE1          (1 << 11)
#define SSI_SISR_ROE0          (1 << 10)
#define SSI_SISR_TUE1          (1 << 9)
#define SSI_SISR_TUE0          (1 << 8)
#define SSI_SISR_TFS           (1 << 7)
#define SSI_SISR_RFS           (1 << 6)
#define SSI_SISR_TLS           (1 << 5)
#define SSI_SISR_RLS           (1 << 4)
#define SSI_SISR_RFF1          (1 << 3)
#define SSI_SISR_RFF0          (1 << 2)
#define SSI_SISR_TFE1          (1 << 1)
#define SSI_SISR_TFE0          (1 << 0)

#define SSI_SIER_RDMAE         (1 << 22)
#define SSI_SIER_RIE           (1 << 21)
#define SSI_SIER_TDMAE         (1 << 20)
#define SSI_SIER_TIE           (1 << 19)
#define SSI_SIER_CMDAU_EN      (1 << 18)
#define SSI_SIER_CMDDU_EN      (1 << 17)
#define SSI_SIER_RXT_EN        (1 << 16)
#define SSI_SIER_RDR1_EN       (1 << 15)
#define SSI_SIER_RDR0_EN       (1 << 14)
#define SSI_SIER_TDE1_EN       (1 << 13)
#define SSI_SIER_TDE0_EN       (1 << 12)
#define SSI_SIER_ROE1_EN       (1 << 11)
#define SSI_SIER_ROE0_EN       (1 << 10)
#define SSI_SIER_TUE1_EN       (1 << 9)
#define SSI_SIER_TUE0_EN       (1 << 8)
#define SSI_SIER_TFS_EN        (1 << 7)
#define SSI_SIER_RFS_EN        (1 << 6)
#define SSI_SIER_TLS_EN        (1 << 5)
#define SSI_SIER_RLS_EN        (1 << 4)
#define SSI_SIER_RFF1_EN       (1 << 3)
#define SSI_SIER_RFF0_EN       (1 << 2)
#define SSI_SIER_TFE1_EN       (1 << 1)
#define SSI_SIER_TFE0_EN       (1 << 0)

#define SSI_STCR_TXBIT0        (1 << 9)
#define SSI_STCR_TFEN1         (1 << 8)
#define SSI_STCR_TFEN0         (1 << 7)
#define SSI_STCR_TFDIR         (1 << 6)
#define SSI_STCR_TXDIR         (1 << 5)
#define SSI_STCR_TSHFD         (1 << 4)
#define SSI_STCR_TSCKP         (1 << 3)
#define SSI_STCR_TFSI          (1 << 2)
#define SSI_STCR_TFSL          (1 << 1)
#define SSI_STCR_TEFS          (1 << 0)

#define SSI_SRCR_RXBIT0        (1 << 9)
#define SSI_SRCR_RFEN1         (1 << 8)
#define SSI_SRCR_RFEN0         (1 << 7)
#define SSI_SRCR_RFDIR         (1 << 6)
#define SSI_SRCR_RXDIR         (1 << 5)
#define SSI_SRCR_RSHFD         (1 << 4)
#define SSI_SRCR_RSCKP         (1 << 3)
#define SSI_SRCR_RFSI          (1 << 2)
#define SSI_SRCR_RFSL          (1 << 1)
#define SSI_SRCR_REFS          (1 << 0)

#define SSI_STCCR_DIV2         (1 << 18)
#define SSI_STCCR_PSR          (1 << 15)
#define SSI_STCCR_WL(x)        ((((x) - 2) >> 1) << 13)
#define SSI_STCCR_DC(x)        (((x) & 0x1f) << 8)
#define SSI_STCCR_PM(x)        (((x) & 0xff) << 0)

#define SSI_SRCCR_DIV2         (1 << 18)
#define SSI_SRCCR_PSR          (1 << 15)
#define SSI_SRCCR_WL(x)        ((((x) - 2) >> 1) << 13)
#define SSI_SRCCR_DC(x)        (((x) & 0x1f) << 8)
#define SSI_SRCCR_PM(x)        (((x) & 0xff) << 0)


#define SSI_SFCSR_RFCNT1(x)   (((x) & 0xf) << 28)
#define SSI_SFCSR_TFCNT1(x)   (((x) & 0xf) << 24)
#define SSI_SFCSR_RFWM1(x)    (((x) & 0xf) << 20)
#define SSI_SFCSR_TFWM1(x)    (((x) & 0xf) << 16)
#define SSI_SFCSR_RFCNT0(x)   (((x) & 0xf) << 12)
#define SSI_SFCSR_TFCNT0(x)   (((x) & 0xf) <<  8)
#define SSI_SFCSR_RFWM0(x)    (((x) & 0xf) <<  4)
#define SSI_SFCSR_TFWM0(x)    (((x) & 0xf) <<  0)

#define SSI_STR_TEST          (1 << 15)
#define SSI_STR_RCK2TCK       (1 << 14)
#define SSI_STR_RFS2TFS       (1 << 13)
#define SSI_STR_RXSTATE(x)    (((x) & 0xf) << 8)
#define SSI_STR_TXD2RXD       (1 <<  7)
#define SSI_STR_TCK2RCK       (1 <<  6)
#define SSI_STR_TFS2RFS       (1 <<  5)
#define SSI_STR_TXSTATE(x)    (((x) & 0xf) << 0)

#define SSI_SOR_CLKOFF        (1 << 6)
#define SSI_SOR_RX_CLR        (1 << 5)
#define SSI_SOR_TX_CLR        (1 << 4)
#define SSI_SOR_INIT          (1 << 3)
#define SSI_SOR_WAIT(x)       (((x) & 0x3) << 1)
#define SSI_SOR_SYNRST        (1 << 0)

#define SSI_SACNT_FRDIV(x)    (((x) & 0x3f) << 5)
#define SSI_SACNT_WR          (x << 4)
#define SSI_SACNT_RD          (x << 3)
#define SSI_SACNT_TIF         (x << 2)
#define SSI_SACNT_FV          (x << 1)
#define SSI_SACNT_A97EN       (x << 0)


/* AUDMUX registers */
#define AUDMUX_HPCR1         __REG(IMX_AUDMUX_BASE + 0x00)
#define AUDMUX_HPCR2         __REG(IMX_AUDMUX_BASE + 0x04)
#define AUDMUX_HPCR3         __REG(IMX_AUDMUX_BASE + 0x08)
#define AUDMUX_PPCR1         __REG(IMX_AUDMUX_BASE + 0x10)
#define AUDMUX_PPCR2         __REG(IMX_AUDMUX_BASE + 0x14)
#define AUDMUX_PPCR3         __REG(IMX_AUDMUX_BASE + 0x18)

#define AUDMUX_HPCR_TFSDIR         (1 << 31)
#define AUDMUX_HPCR_TCLKDIR        (1 << 30)
#define AUDMUX_HPCR_TFCSEL_TX      (0 << 26)
#define AUDMUX_HPCR_TFCSEL_RX      (8 << 26)
#define AUDMUX_HPCR_TFCSEL(x)      (((x) & 0x7) << 26)
#define AUDMUX_HPCR_RFSDIR         (1 << 25)
#define AUDMUX_HPCR_RCLKDIR        (1 << 24)
#define AUDMUX_HPCR_RFCSEL_TX      (0 << 20)
#define AUDMUX_HPCR_RFCSEL_RX      (8 << 20)
#define AUDMUX_HPCR_RFCSEL(x)      (((x) & 0x7) << 20)
#define AUDMUX_HPCR_RXDSEL(x)      (((x) & 0x7) << 13)
#define AUDMUX_HPCR_SYN            (1 << 12)
#define AUDMUX_HPCR_TXRXEN         (1 << 10)
#define AUDMUX_HPCR_INMEN          (1 <<  8)
#define AUDMUX_HPCR_INMMASK(x)     (((x) & 0xff) << 0)

#define AUDMUX_PPCR_TFSDIR         (1 << 31)
#define AUDMUX_PPCR_TCLKDIR        (1 << 30)
#define AUDMUX_PPCR_TFCSEL_TX      (0 << 26)
#define AUDMUX_PPCR_TFCSEL_RX      (8 << 26)
#define AUDMUX_PPCR_TFCSEL(x)      (((x) & 0x7) << 26)
#define AUDMUX_PPCR_RFSDIR         (1 << 25)
#define AUDMUX_PPCR_RCLKDIR        (1 << 24)
#define AUDMUX_PPCR_RFCSEL_TX      (0 << 20)
#define AUDMUX_PPCR_RFCSEL_RX      (8 << 20)
#define AUDMUX_PPCR_RFCSEL(x)      (((x) & 0x7) << 20)
#define AUDMUX_PPCR_RXDSEL(x)      (((x) & 0x7) << 13)
#define AUDMUX_PPCR_SYN            (1 << 12)
#define AUDMUX_PPCR_TXRXEN         (1 << 10)


#endif /* defined(CONFIG_ARCH_IMX21) */













#if defined(CONFIG_ARCH_IMX)
/*
 *  Interrupt controller
 */

#define IMX_INTCNTL        __REG(IMX_AITC_BASE+0x00)
#define INTCNTL_FIAD       (1<<19)
#define INTCNTL_NIAD       (1<<20)

#define IMX_NIMASK         __REG(IMX_AITC_BASE+0x04)
#define IMX_INTENNUM       __REG(IMX_AITC_BASE+0x08)
#define IMX_INTDISNUM      __REG(IMX_AITC_BASE+0x0c)
#define IMX_INTENABLEH     __REG(IMX_AITC_BASE+0x10)
#define IMX_INTENABLEL     __REG(IMX_AITC_BASE+0x14)

/*
 * LCD Controller
 */

#define LCDC_SSA	__REG(IMX_LCDC_BASE+0x00)

#define LCDC_SIZE	__REG(IMX_LCDC_BASE+0x04)
#define SIZE_XMAX(x)	((((x) >> 4) & 0x3f) << 20)
#define SIZE_YMAX(y)    ( (y) & 0x1ff )

#define LCDC_VPW	__REG(IMX_LCDC_BASE+0x08)
#define VPW_VPW(x)	( (x) & 0x3ff )

#define LCDC_CPOS	__REG(IMX_LCDC_BASE+0x0C)
#define CPOS_CC1        (1<<31)
#define CPOS_CC0        (1<<30)
#define CPOS_OP         (1<<28)
#define CPOS_CXP(x)     (((x) & 3ff) << 16)
#define CPOS_CYP(y)     ((y) & 0x1ff)

#define LCDC_LCWHB	__REG(IMX_LCDC_BASE+0x10)
#define LCWHB_BK_EN     (1<<31)
#define LCWHB_CW(w)     (((w) & 0x1f) << 24)
#define LCWHB_CH(h)     (((h) & 0x1f) << 16)
#define LCWHB_BD(x)     ((x) & 0xff)

#define LCDC_LCHCC	__REG(IMX_LCDC_BASE+0x14)
#define LCHCC_CUR_COL_R(r) (((r) & 0x1f) << 11)
#define LCHCC_CUR_COL_G(g) (((g) & 0x3f) << 5)
#define LCHCC_CUR_COL_B(b) ((b) & 0x1f)

#define LCDC_PCR	__REG(IMX_LCDC_BASE+0x18)
#define PCR_TFT         (1<<31)
#define PCR_COLOR       (1<<30)
#define PCR_PBSIZ_1     (0<<28)
#define PCR_PBSIZ_2     (1<<28)
#define PCR_PBSIZ_4     (2<<28)
#define PCR_PBSIZ_8     (3<<28)
#define PCR_BPIX_1      (0<<25)
#define PCR_BPIX_2      (1<<25)
#define PCR_BPIX_4      (2<<25)
#define PCR_BPIX_8      (3<<25)
#define PCR_BPIX_12     (4<<25)
#define PCR_BPIX_16     (4<<25)
#define PCR_PIXPOL      (1<<24)
#define PCR_FLMPOL      (1<<23)
#define PCR_LPPOL       (1<<22)
#define PCR_CLKPOL      (1<<21)
#define PCR_OEPOL       (1<<20)
#define PCR_SCLKIDLE    (1<<19)
#define PCR_END_SEL     (1<<18)
#define PCR_END_BYTE_SWAP (1<<17)
#define PCR_REV_VS      (1<<16)
#define PCR_ACD_SEL     (1<<15)
#define PCR_ACD(x)      (((x) & 0x7f) << 8)
#define PCR_SCLK_SEL    (1<<7)
#define PCR_SHARP       (1<<6)
#define PCR_PCD(x)      ((x) & 0x3f)

#define LCDC_HCR	__REG(IMX_LCDC_BASE+0x1C)
#define HCR_H_WIDTH(x)  (((x) & 0x3f) << 26)
#define HCR_H_WAIT_1(x) (((x) & 0xff) << 8)
#define HCR_H_WAIT_2(x) ((x) & 0xff)

#define LCDC_VCR	__REG(IMX_LCDC_BASE+0x20)
#define VCR_V_WIDTH(x)  (((x) & 0x3f) << 26)
#define VCR_V_WAIT_1(x) (((x) & 0xff) << 8)
#define VCR_V_WAIT_2(x) ((x) & 0xff)

#define LCDC_POS	__REG(IMX_LCDC_BASE+0x24)
#define POS_POS(x)      ((x) & 1f)

#define LCDC_LSCR1	__REG(IMX_LCDC_BASE+0x28)
#define LSCR1_PS_RISE_DELAY(x)    (((x) & 0x7f) << 26)
#define LSCR1_CLS_RISE_DELAY(x)   (((x) & 0x3f) << 16)
#define LSCR1_REV_TOGGLE_DELAY(x) (((x) & 0xf) << 8)
#define LSCR1_GRAY2(x)            (((x) & 0xf) << 4)
#define LSCR1_GRAY1(x)            (((x) & 0xf))

#define LCDC_PWMR	__REG(IMX_LCDC_BASE+0x2C)
#define PWMR_CLS(x)     (((x) & 0x1ff) << 16)
#define PWMR_LDMSK      (1<<15)
#define PWMR_SCR1       (1<<10)
#define PWMR_SCR0       (1<<9)
#define PWMR_CC_EN      (1<<8)
#define PWMR_PW(x)      ((x) & 0xff)

#define LCDC_DMACR	__REG(IMX_LCDC_BASE+0x30)
#define DMACR_BURST     (1<<31)
#define DMACR_HM(x)     (((x) & 0xf) << 16)
#define DMACR_TM(x)     ((x) &0xf)

#define LCDC_RMCR	__REG(IMX_LCDC_BASE+0x34)
#define RMCR_LCDC_EN		(1<<1)
#define RMCR_SELF_REF		(1<<0)

#define LCDC_LCDICR	__REG(IMX_LCDC_BASE+0x38)
#define LCDICR_INT_SYN  (1<<2)
#define LCDICR_INT_CON  (1)

#define LCDC_LCDISR	__REG(IMX_LCDC_BASE+0x40)
#define LCDISR_UDR_ERR (1<<3)
#define LCDISR_ERR_RES (1<<2)
#define LCDISR_EOF     (1<<1)
#define LCDISR_BOF     (1<<0)

#define LCDC_PALETTE(x) __REG2(IMX_LCDC_BASE+0x800, (x)<<2)

/*
 *  UART Module. Takes the UART base address as argument
 */
#define URXD0(x) __REG( 0x0 + (x)) /* Receiver Register */
#define URTX0(x) __REG( 0x40 + (x)) /* Transmitter Register */
#define UCR1(x)  __REG( 0x80 + (x)) /* Control Register 1 */
#define UCR2(x)  __REG( 0x84 + (x)) /* Control Register 2 */
#define UCR3(x)  __REG( 0x88 + (x)) /* Control Register 3 */
#define UCR4(x)  __REG( 0x8c + (x)) /* Control Register 4 */
#define UFCR(x)  __REG( 0x90 + (x)) /* FIFO Control Register */
#define USR1(x)  __REG( 0x94 + (x)) /* Status Register 1 */
#define USR2(x)  __REG( 0x98 + (x)) /* Status Register 2 */
#define UESC(x)  __REG( 0x9c + (x)) /* Escape Character Register */
#define UTIM(x)  __REG( 0xa0 + (x)) /* Escape Timer Register */
#define UBIR(x)  __REG( 0xa4 + (x)) /* BRM Incremental Register */
#define UBMR(x)  __REG( 0xa8 + (x)) /* BRM Modulator Register */
#define UBRC(x)  __REG( 0xac + (x)) /* Baud Rate Count Register */
#define BIPR1(x) __REG( 0xb0 + (x)) /* Incremental Preset Register 1 */
#define BIPR2(x) __REG( 0xb4 + (x)) /* Incremental Preset Register 2 */
#define BIPR3(x) __REG( 0xb8 + (x)) /* Incremental Preset Register 3 */
#define BIPR4(x) __REG( 0xbc + (x)) /* Incremental Preset Register 4 */
#define BMPR1(x) __REG( 0xc0 + (x)) /* BRM Modulator Register 1 */
#define BMPR2(x) __REG( 0xc4 + (x)) /* BRM Modulator Register 2 */
#define BMPR3(x) __REG( 0xc8 + (x)) /* BRM Modulator Register 3 */
#define BMPR4(x) __REG( 0xcc + (x)) /* BRM Modulator Register 4 */
#define UTS(x)   __REG( 0xd0 + (x)) /* UART Test Register */

/* UART Control Register Bit Fields.*/
#define  URXD_CHARRDY    (1<<15)
#define  URXD_ERR        (1<<14)
#define  URXD_OVRRUN     (1<<13)
#define  URXD_FRMERR     (1<<12)
#define  URXD_BRK        (1<<11)
#define  URXD_PRERR      (1<<10)
#define  UCR1_ADEN       (1<<15) /* Auto dectect interrupt */
#define  UCR1_ADBR       (1<<14) /* Auto detect baud rate */
#define  UCR1_TRDYEN     (1<<13) /* Transmitter ready interrupt enable */
#define  UCR1_IDEN       (1<<12) /* Idle condition interrupt */
#define  UCR1_RRDYEN     (1<<9)	 /* Recv ready interrupt enable */
#define  UCR1_RDMAEN     (1<<8)	 /* Recv ready DMA enable */
#define  UCR1_IREN       (1<<7)	 /* Infrared interface enable */
#define  UCR1_TXMPTYEN   (1<<6)	 /* Transimitter empty interrupt enable */
#define  UCR1_RTSDEN     (1<<5)	 /* RTS delta interrupt enable */
#define  UCR1_SNDBRK     (1<<4)	 /* Send break */
#define  UCR1_TDMAEN     (1<<3)	 /* Transmitter ready DMA enable */
#define  UCR1_UARTCLKEN  (1<<2)	 /* UART clock enabled */
#define  UCR1_DOZE       (1<<1)	 /* Doze */
#define  UCR1_UARTEN     (1<<0)	 /* UART enabled */
#define  UCR2_ESCI     	 (1<<15) /* Escape seq interrupt enable */
#define  UCR2_IRTS  	 (1<<14) /* Ignore RTS pin */
#define  UCR2_CTSC  	 (1<<13) /* CTS pin control */
#define  UCR2_CTS        (1<<12) /* Clear to send */
#define  UCR2_ESCEN      (1<<11) /* Escape enable */
#define  UCR2_PREN       (1<<8)  /* Parity enable */
#define  UCR2_PROE       (1<<7)  /* Parity odd/even */
#define  UCR2_STPB       (1<<6)	 /* Stop */
#define  UCR2_WS         (1<<5)	 /* Word size */
#define  UCR2_RTSEN      (1<<4)	 /* Request to send interrupt enable */
#define  UCR2_TXEN       (1<<2)	 /* Transmitter enabled */
#define  UCR2_RXEN       (1<<1)	 /* Receiver enabled */
#define  UCR2_SRST 	 (1<<0)	 /* SW reset */
#define  UCR3_DTREN 	 (1<<13) /* DTR interrupt enable */
#define  UCR3_PARERREN   (1<<12) /* Parity enable */
#define  UCR3_FRAERREN   (1<<11) /* Frame error interrupt enable */
#define  UCR3_DSR        (1<<10) /* Data set ready */
#define  UCR3_DCD        (1<<9)  /* Data carrier detect */
#define  UCR3_RI         (1<<8)  /* Ring indicator */
#define  UCR3_TIMEOUTEN  (1<<7)  /* Timeout interrupt enable */
#define  UCR3_RXDSEN	 (1<<6)  /* Receive status interrupt enable */
#define  UCR3_AIRINTEN   (1<<5)  /* Async IR wake interrupt enable */
#define  UCR3_AWAKEN	 (1<<4)  /* Async wake interrupt enable */
#define  UCR3_REF25 	 (1<<3)  /* Ref freq 25 MHz */
#define  UCR3_REF30 	 (1<<2)  /* Ref Freq 30 MHz */
#define  UCR3_INVT  	 (1<<1)  /* Inverted Infrared transmission */
#define  UCR3_BPEN  	 (1<<0)  /* Preset registers enable */
#define  UCR4_CTSTL_32   (32<<10) /* CTS trigger level (32 chars) */
#define  UCR4_INVR  	 (1<<9)  /* Inverted infrared reception */
#define  UCR4_ENIRI 	 (1<<8)  /* Serial infrared interrupt enable */
#define  UCR4_WKEN  	 (1<<7)  /* Wake interrupt enable */
#define  UCR4_REF16 	 (1<<6)  /* Ref freq 16 MHz */
#define  UCR4_IRSC  	 (1<<5)  /* IR special case */
#define  UCR4_TCEN  	 (1<<3)  /* Transmit complete interrupt enable */
#define  UCR4_BKEN  	 (1<<2)  /* Break condition interrupt enable */
#define  UCR4_OREN  	 (1<<1)  /* Receiver overrun interrupt enable */
#define  UCR4_DREN  	 (1<<0)  /* Recv data ready interrupt enable */
#define  UFCR_RXTL_SHF   0       /* Receiver trigger level shift */
#define  UFCR_RFDIV      (7<<7)  /* Reference freq divider mask */
#define  UFCR_TXTL_SHF   10      /* Transmitter trigger level shift */
#define  USR1_PARITYERR  (1<<15) /* Parity error interrupt flag */
#define  USR1_RTSS  	 (1<<14) /* RTS pin status */
#define  USR1_TRDY  	 (1<<13) /* Transmitter ready interrupt/dma flag */
#define  USR1_RTSD  	 (1<<12) /* RTS delta */
#define  USR1_ESCF  	 (1<<11) /* Escape seq interrupt flag */
#define  USR1_FRAMERR    (1<<10) /* Frame error interrupt flag */
#define  USR1_RRDY       (1<<9)	 /* Receiver ready interrupt/dma flag */
#define  USR1_TIMEOUT    (1<<7)	 /* Receive timeout interrupt status */
#define  USR1_RXDS  	 (1<<6)	 /* Receiver idle interrupt flag */
#define  USR1_AIRINT	 (1<<5)	 /* Async IR wake interrupt flag */
#define  USR1_AWAKE 	 (1<<4)	 /* Aysnc wake interrupt flag */
#define  USR2_ADET  	 (1<<15) /* Auto baud rate detect complete */
#define  USR2_TXFE  	 (1<<14) /* Transmit buffer FIFO empty */
#define  USR2_DTRF  	 (1<<13) /* DTR edge interrupt flag */
#define  USR2_IDLE  	 (1<<12) /* Idle condition */
#define  USR2_IRINT 	 (1<<8)	 /* Serial infrared interrupt flag */
#define  USR2_WAKE  	 (1<<7)	 /* Wake */
#define  USR2_RTSF  	 (1<<4)	 /* RTS edge interrupt flag */
#define  USR2_TXDC  	 (1<<3)	 /* Transmitter complete */
#define  USR2_BRCD  	 (1<<2)	 /* Break condition */
#define  USR2_ORE        (1<<1)	 /* Overrun error */
#define  USR2_RDR        (1<<0)	 /* Recv data ready */
#define  UTS_FRCPERR	 (1<<13) /* Force parity error */
#define  UTS_LOOP        (1<<12) /* Loop tx and rx */
#define  UTS_TXEMPTY	 (1<<6)	 /* TxFIFO empty */
#define  UTS_RXEMPTY	 (1<<5)	 /* RxFIFO empty */
#define  UTS_TXFULL 	 (1<<4)	 /* TxFIFO full */
#define  UTS_RXFULL 	 (1<<3)	 /* RxFIFO full */
#define  UTS_SOFTRST	 (1<<0)	 /* Software reset */


/*
 * External Interface Module
 */
#define EIM_CS0U   __REG(IMX_EIM_BASE +0x00)	/* Chip Select 0 Upper Control */
#define EIM_CS0L   __REG(IMX_EIM_BASE +0x04)	/* Chip Select 0 Lower Control */
#define EIM_CS1U   __REG(IMX_EIM_BASE +0x08)	/* Chip Select 1 Upper Control */
#define EIM_CS1L   __REG(IMX_EIM_BASE +0x0c)	/* Chip Select 1 Lower Control */
#define EIM_CS2U   __REG(IMX_EIM_BASE +0x10)	/* Chip Select 2 Upper Control */
#define EIM_CS2L   __REG(IMX_EIM_BASE +0x14)	/* Chip Select 2 Lower Control */
#define EIM_CS3U   __REG(IMX_EIM_BASE +0x18)	/* Chip Select 3 Upper Control */
#define EIM_CS3L   __REG(IMX_EIM_BASE +0x1c)	/* Chip Select 3 Lower Control */
#define EIM_CS4U   __REG(IMX_EIM_BASE +0x20)	/* Chip Select 4 Upper Control */
#define EIM_CS4L   __REG(IMX_EIM_BASE +0x24)	/* Chip Select 4 Lower Control */
#define EIM_CS5U   __REG(IMX_EIM_BASE +0x28)	/* Chip Select 5 Upper Control */
#define EIM_CS5L   __REG(IMX_EIM_BASE +0x2c)	/* Chip Select 5 Lower Control */
#define EIM_EIM    __REG(IMX_EIM_BASE +0x30)	/* EIM Configuration */

#define EIM_CSnU_DTACK_SEL (1 << 31)
#define EIM_CSnU_BCD(x)  (((x) & 0x3)  << 28)
#define EIM_CSnU_BCS(x)  (((x) & 0xf)  << 24)
#define EIM_CSnU_PSZ(x)  (((x) & 0x3)  << 22)
#define EIM_CSnU_PME     (1 << 21)
#define EIM_CSnU_SYNC    (1 << 20)
#define EIM_CSnU_DOL(x)  (((x) & 0xf)  << 16)
#define EIM_CSnU_CNC(x)  (((x) & 0x3)  << 14)
#define EIM_CSnU_WSC(x)  (((x) & 0x3f) <<  8)
#define EIM_CSnU_WSC_DTACK (0x3f <<  8)
#define EIM_CSnU_WWS(x)  (((x) & 0x7)  <<  4)
#define EIM_CSnU_EDC(x)  (((x) & 0xf)  <<  0)

#define EIM_CSnL_OEA(x)  (((x) & 0xf)  << 28)
#define EIM_CSnL_OEN(x)  (((x) & 0xf)  << 24)
#define EIM_CSnL_WEA(x)  (((x) & 0xf)  << 20)
#define EIM_CSnL_WEN(x)  (((x) & 0xf)  << 16)
#define EIM_CSnL_CSA(x)  (((x) & 0xf)  << 12)
#define EIM_CSnU_EBC     (1 << 11)
#define EIM_CSnL_DSZ(x)  (((x) & 0x7)  <<  8)
#define EIM_CSnU_SP      (1 << 6)
#define EIM_CSnU_WP      (1 << 4)
#define EIM_CSnU_CSEN    (1 << 0)
#define EIM_CSnU_PA      (1 << 1)

#elif defined(CONFIG_ARCH_IMX21)

 /* 
  * NAND memory controller
  */
#define NFC_MAIN_BUF0          __NFCPTR(0x000)
#define NFC_MAIN_BUF1          __NFCPTR(0x200)
#define NFC_MAIN_BUF2          __NFCPTR(0x400)
#define NFC_MAIN_BUF3          __NFCPTR(0x600)
#define NFC_SPARE_BUF0         __NFCPTR(0x800)
#define NFC_SPARE_BUF1         __NFCPTR(0x810)
#define NFC_SPARE_BUF2         __NFCPTR(0x820)
#define NFC_SPARE_BUF3         __NFCPTR(0x830)
#define NFC_BUFSIZE            __NFCREG16(0x840)
#define NFC_BLK_ADDR_LOCK      __NFCREG16(0xe02)
#define NFC_RAM_BUF_ADDR       __NFCREG16(0xe04)
#define NFC_FLASH_ADDR         __NFCREG16(0xe06)
#define NFC_FLASH_CMD          __NFCREG16(0xe08)
#define NFC_CONFIG             __NFCREG16(0xe0a)
#define NFC_ECC_RSLT_MAIN      __NFCREG16(0xe0c)
#define NFC_ECC_RSLT_SPARE     __NFCREG16(0xe10)
#define NFC_WR_PROT            __NFCREG16(0xe12)
#define NFC_UNLOCK_BLK_START   __NFCREG16(0xe14)
#define NFC_UNLOCK_BLK_END     __NFCREG16(0xe16)
#define NFC_WR_PROT_STAT       __NFCREG16(0xe18)
#define NFC_FLASH_CONFIG1      __NFCREG16(0xe1a)
#define NFC_FLASH_CONFIG2      __NFCREG16(0xe1c)
#define NFC_FLASH_CONFIG2_FCMD        (1 << 0)
#define NFC_FLASH_CONFIG2_FADD        (1 << 1)
#define NFC_FLASH_CONFIG2_FDI         (1 << 2)
#define NFC_FLASH_CONFIG2_FDO_PAGE    (1 << 3)
#define NFC_FLASH_CONFIG2_FDO_ID      (2 << 3)
#define NFC_FLASH_CONFIG2_FDO_STATUS  (4 << 3)
#define NFC_FLASH_CONFIG2_INT         (1 << 15)




 /*
 * Multi-layer AHB Crossbar Switch (MAX)
 */

#define MAX_MPR(x)    __REG(IMX_MAX_BASE + (((x) & 3) << 8))
#define MAX_MMPR(x)   __REG(IMX_MAX_BASE + (((x) & 3) << 8) + 0x4)
#define MAX_SGPCR(x)  __REG(IMX_MAX_BASE + (((x) & 3) << 8) + 0x10)
#define MAX_ASGPCR(x) __REG(IMX_MAX_BASE + (((x) & 3) << 8) + 0x14)
#define MAX_MGPCR(x)  __REG(IMX_MAX_BASE + (((x) & 7) << 8) + 0x800)


/*
 * LCD Controller
 */

#define LCDC_SSA        __REG(IMX_LCDC_BASE+0x00)

#define LCDC_SIZE       __REG(IMX_LCDC_BASE+0x04)
#define SIZE_XMAX(x)    ((((x) >> 4) & 0x3f) << 20)
#define SIZE_YMAX(y)    ( (y) & 0x1ff )

#define LCDC_VPW        __REG(IMX_LCDC_BASE+0x08)
#define VPW_VPW(x)      ( (x) & 0x3ff )

#define LCDC_CPOS       __REG(IMX_LCDC_BASE+0x0C)
#define CPOS_CC1        (1<<31)
#define CPOS_CC0        (1<<30)
#define CPOS_OP         (1<<28)
#define CPOS_CXP(x)     (((x) & 3ff) << 16)
#define CPOS_CYP(y)     ((y) & 0x3ff)

#define LCDC_LCWHB      __REG(IMX_LCDC_BASE+0x10)
#define LCWHB_BK_EN     (1<<31)
#define LCWHB_CW(w)     (((w) & 0x1f) << 24)
#define LCWHB_CH(h)     (((h) & 0x1f) << 16)
#define LCWHB_BD(x)     ((x) & 0xff)

#define LCDC_LCHCC      __REG(IMX_LCDC_BASE+0x14)
#define LCHCC_CUR_COL_R(r) (((r) & 0x1f) << 11)
#define LCHCC_CUR_COL_G(g) (((g) & 0x3f) << 5)
#define LCHCC_CUR_COL_B(b) ((b) & 0x1f)

#define LCDC_PCR        __REG(IMX_LCDC_BASE+0x18)
#define PCR_TFT         (1<<31)
#define PCR_COLOR       (1<<30)
#define PCR_PBSIZ_1     (0<<28)
#define PCR_PBSIZ_2     (1<<28)
#define PCR_PBSIZ_4     (2<<28)
#define PCR_PBSIZ_8     (3<<28)
#define PCR_BPIX_1      (0<<25)
#define PCR_BPIX_2      (1<<25)
#define PCR_BPIX_4      (2<<25)
#define PCR_BPIX_8      (3<<25)
#define PCR_BPIX_12     (4<<25)
#define PCR_BPIX_16     (5<<25)
#define PCR_BPIX_18     (6<<25)
#define PCR_PIXPOL      (1<<24)
#define PCR_FLMPOL      (1<<23)
#define PCR_LPPOL       (1<<22)
#define PCR_CLKPOL      (1<<21)
#define PCR_OEPOL       (1<<20)
#define PCR_SCLKIDLE    (1<<19)
#define PCR_END_SEL     (1<<18)
#define PCR_END_BYTE_SWAP (1<<17)
#define PCR_REV_VS      (1<<16)
#define PCR_ACD_SEL     (1<<15)
#define PCR_ACD(x)      (((x) & 0x7f) << 8)
#define PCR_SCLK_SEL    (1<<7)
#define PCR_SHARP       (1<<6)
#define PCR_PCD(x)      ((x) & 0x3f)

#define LCDC_HCR        __REG(IMX_LCDC_BASE+0x1C)
#define HCR_H_WIDTH(x)  (((x) & 0x3f) << 26)
#define HCR_H_WAIT_1(x) (((x) & 0xff) << 8)
#define HCR_H_WAIT_2(x) ((x) & 0xff)

#define LCDC_VCR        __REG(IMX_LCDC_BASE+0x20)
#define VCR_V_WIDTH(x)  (((x) & 0x3f) << 26)
#define VCR_V_WAIT_1(x) (((x) & 0xff) << 8)
#define VCR_V_WAIT_2(x) ((x) & 0xff)

#define LCDC_POS        __REG(IMX_LCDC_BASE+0x24)
#define POS_POS(x)      ((x) & 1f)

#define LCDC_LSCR1      __REG(IMX_LCDC_BASE+0x28)
#define LSCR1_PS_RISE_DELAY(x)    (((x) & 0x3f) << 26)
#define LSCR1_CLS_RISE_DELAY(x)   (((x) & 0xff) << 16)
#define LSCR1_REV_TOGGLE_DELAY(x) (((x) & 0xf) << 8)
#define LSCR1_GRAY2(x)            (((x) & 0xf) << 4)
#define LSCR1_GRAY1(x)            (((x) & 0xf))

#define LCDC_PWMR       __REG(IMX_LCDC_BASE+0x2C)
#define PWMR_CLS(x)     (((x) & 0x1ff) << 16)
#define PWMR_LDMSK      (1<<15)
#define PWMR_SCR1       (1<<10)
#define PWMR_SCR0       (1<<9)
#define PWMR_CC_EN      (1<<8)
#define PWMR_PW(x)      ((x) & 0xff)

#define LCDC_DMACR      __REG(IMX_LCDC_BASE+0x30)
#define DMACR_BURST     (1<<31)
#define DMACR_HM(x)     (((x) & 0x1f) << 16)
#define DMACR_TM(x)     ((x) &0x1f)

#define LCDC_RMCR       __REG(IMX_LCDC_BASE+0x34)
#define RMCR_LCDC_EN            (1<<1)
#define RMCR_SELF_REF           (1<<0)

#define LCDC_LCDICR     __REG(IMX_LCDC_BASE+0x38)
#define LCDICR_INT_SYN  (1<<2)
#define LCDICR_INT_CON  (1)

#define LCDC_LCDISR     __REG(IMX_LCDC_BASE+0x40)
#define LCDISR_UDR_ERR (1<<3)
#define LCDISR_ERR_RES (1<<2)
#define LCDISR_EOF     (1<<1)
#define LCDISR_BOF     (1<<0)

#define LCDC_LGWSAR     __REG(IMX_LCDC_BASE+0x50)
#define LCDC_LGWSR      __REG(IMX_LCDC_BASE+0x54)
#define LCDC_LGWVPWR    __REG(IMX_LCDC_BASE+0x58)
#define LCDC_LGWPOR     __REG(IMX_LCDC_BASE+0x5c)
#define LCDC_LGWPR      __REG(IMX_LCDC_BASE+0x60)
#define LGWPR_XPOS(x)   (((x) & 0x3ff) << 16)
#define LGWPR_YPOS(x)   (((x) & 0x3ff) << 0)

#define LCDC_LGWCR      __REG(IMX_LCDC_BASE+0x64)
#define LGWCR_ALPHA(x)  (((x) & 0xff) << 24)
#define LGWCR_GWAV(x)  (((x) & 0xff) << 24)
#define LGWCR_GWCKE     (1 << 23)
#define LGWCR_GWE       (1 << 22)
#define LGWCR_GW_RVS    (1 << 21)
#define LGWPCR_GWCKR(x) (((x) & 0x3f) << 12)
#define LGWPCR_GWCKG(x) (((x) & 0x3f) <<  6)
#define LGWPCR_GWCKB(x) (((x) & 0x3f) <<  0)

#define LCDC_LGWDCR     __REG(IMX_LCDC_BASE+0x68)

#define LCDC_PALETTE(x) __REG2(IMX_LCDC_BASE+0x800, (x)<<2)

/*
 *  Uart controller registers 
 */

#define UART_1  0
#define UART_2  1
#define UART_3  2
#define UART_4  3


#define URXD0(x) __REG( 0x0 + (x)) /* Receiver Register */
#define URTX0(x) __REG( 0x40 + (x)) /* Transmitter Register */
#define UCR1(x)  __REG( 0x80 + (x)) /* Control Register 1 */
#define UCR2(x)  __REG( 0x84 + (x)) /* Control Register 2 */
#define UCR3(x)  __REG( 0x88 + (x)) /* Control Register 3 */
#define UCR4(x)  __REG( 0x8c + (x)) /* Control Register 4 */
#define UFCR(x)  __REG( 0x90 + (x)) /* FIFO Control Register */
#define USR1(x)  __REG( 0x94 + (x)) /* Status Register 1 */
#define USR2(x)  __REG( 0x98 + (x)) /* Status Register 2 */
#define UESC(x)  __REG( 0x9c + (x)) /* Escape Character Register */
#define UTIM(x)  __REG( 0xa0 + (x)) /* Escape Timer Register */
#define UBIR(x)  __REG( 0xa4 + (x)) /* BRM Incremental Register */
#define UBMR(x)  __REG( 0xa8 + (x)) /* BRM Modulator Register */
#define UBRC(x)  __REG( 0xac + (x)) /* Baud Rate Count Register */
#define ONMES(x) __REG( 0xb0 + (x)) /* One Millisecond Register */
#define UTS(x)   __REG( 0xb4 + (x)) /* UART Test Register */



/* UART Control Register Bit Fields.*/
#define  URXD_CHARRDY    (1<<15)
#define  URXD_ERR        (1<<14)
#define  URXD_OVRRUN     (1<<13)
#define  URXD_FRMERR     (1<<12)
#define  URXD_BRK        (1<<11)
#define  URXD_PRERR      (1<<10)

#define  UCR1_ADEN       (1<<15) /* Auto dectect interrupt */
#define  UCR1_ADBR       (1<<14) /* Auto detect baud rate */
#define  UCR1_TRDYEN     (1<<13) /* Transmitter ready interrupt enable */
#define  UCR1_IDEN       (1<<12) /* Idle condition interrupt */
#define  UCR1_RRDYEN     (1<<9)  /* Recv ready interrupt enable */
#define  UCR1_RDMAEN     (1<<8)  /* Recv ready DMA enable */
#define  UCR1_IREN       (1<<7)  /* Infrared interface enable */
#define  UCR1_TXMPTYEN   (1<<6)  /* Transimitter empty interrupt enable */
#define  UCR1_RTSDEN     (1<<5)  /* RTS delta interrupt enable */
#define  UCR1_SNDBRK     (1<<4)  /* Send break */
#define  UCR1_TDMAEN     (1<<3)  /* Transmitter ready DMA enable */
// not on mx21 #define  UCR1_UARTCLKEN  (1<<2)   /* UART clock enabled */
#define  UCR1_DOZE       (1<<1)  /* Doze */
#define  UCR1_UARTEN     (1<<0)  /* UART enabled */

#define  UCR2_ESCI       (1<<15) /* Escape seq interrupt enable */
#define  UCR2_IRTS       (1<<14) /* Ignore RTS pin */
#define  UCR2_CTSC       (1<<13) /* CTS pin control */
#define  UCR2_CTS        (1<<12) /* Clear to send */
#define  UCR2_ESCEN      (1<<11) /* Escape enable */
#define  UCR2_PREN       (1<<8)  /* Parity enable */
#define  UCR2_PROE       (1<<7)  /* Parity odd/even */
#define  UCR2_STPB       (1<<6)  /* Stop */
#define  UCR2_WS         (1<<5)  /* Word size */
#define  UCR2_RTSEN      (1<<4)  /* Request to send interrupt enable */
#define  UCR2_ATEN       (1<<3)  /* Aging Timer Enable */
#define  UCR2_TXEN       (1<<2)  /* Transmitter enabled */
#define  UCR2_RXEN       (1<<1)  /* Receiver enabled */
#define  UCR2_SRST       (1<<0)  /* SW reset */

#define  UCR3_DTREN      (1<<13) /* DTR interrupt enable */
#define  UCR3_PARERREN   (1<<12) /* Parity enable */
#define  UCR3_FRAERREN   (1<<11) /* Frame error interrupt enable */
#define  UCR3_DSR        (1<<10) /* Data set ready */
#define  UCR3_DCD        (1<<9)  /* Data carrier detect */
#define  UCR3_RI         (1<<8)  /* Ring indicator */
#define  UCR3_ADNIMP     (1<<7)  /* Timeout interrupt enable */
#define  UCR3_RXDSEN     (1<<6)  /* Receive status interrupt enable */
#define  UCR3_AIRINTEN   (1<<5)  /* Async IR wake interrupt enable */
#define  UCR3_AWAKEN     (1<<4)  /* Async wake interrupt enable */
// not on mx21 #define  UCR3_REF25       (1<<3)  /* Ref freq 25 MHz */
#define  UCR3_RXDMUXSEL  (1<<2)  /* RXD Mux Input Select */
#define  UCR3_INVT       (1<<1)  /* Inverted Infrared transmission */
#define  UCR3_ACIEN      (1<<0)  /* Autobaud Counter  Interrupt Enable */


#define  UCR4_INVR       (1<<9)  /* Inverted infrared reception */
#define  UCR4_ENIRI      (1<<8)  /* Serial infrared interrupt enable */
#define  UCR4_WKEN       (1<<7)  /* Wake interrupt enable */
// not on mx21 #define  UCR4_REF16       (1<<6)  /* Ref freq 16 MHz */
#define  UCR4_IRSC       (1<<5)  /* IR special case */
#define  UCR4_LPBYP      (1<<5)  /* Low Power Bypass */
#define  UCR4_TCEN       (1<<3)  /* Transmit complete interrupt enable */
#define  UCR4_BKEN       (1<<2)  /* Break condition interrupt enable */
#define  UCR4_OREN       (1<<1)  /* Receiver overrun interrupt enable */
#define  UCR4_DREN       (1<<0)  /* Recv data ready interrupt enable */

#define  UFCR_RXTL_SHF   0       /* Receiver trigger level shift */
#define  UFCR_RFDIV      (7<<7)  /* Reference freq divider mask */
#define  UFCR_RFDIV_SHF  7       /* Reference freq divider shift */
#define  UFCR_TXTL_SHF   10      /* Transmitter trigger level shift */
#define  UFCR_DCEDTE     (1<<6)  /* DCE/DTE Mode */

#define  USR1_PARITYERR  (1<<15) /* Parity error interrupt flag */
#define  USR1_RTSS       (1<<14) /* RTS pin status */
#define  USR1_TRDY       (1<<13) /* Transmitter ready interrupt/dma flag */
#define  USR1_RTSD       (1<<12) /* RTS delta */
#define  USR1_ESCF       (1<<11) /* Escape seq interrupt flag */
#define  USR1_FRAMERR    (1<<10) /* Frame error interrupt flag */
#define  USR1_RRDY       (1<<9)  /* Receiver ready interrupt/dma flag */
#define  USR1_AGTIM      (1<<8)  /* Aging Timer Interrupt Flag */
// not on mx21 #define  USR1_TIMEOUT    (1<<7)   /* Receive timeout interrupt status */
#define  USR1_RXDS       (1<<6)  /* Receiver idle interrupt flag */
#define  USR1_AIRINT     (1<<5)  /* Async IR wake interrupt flag */
#define  USR1_AWAKE      (1<<4)  /* Aysnc wake interrupt flag */

#define  USR2_ADET       (1<<15) /* Auto baud rate detect complete */
#define  USR2_TXFE       (1<<14) /* Transmit buffer FIFO empty */
#define  USR2_DTRF       (1<<13) /* DTR edge interrupt flag */
#define  USR2_IDLE       (1<<12) /* Idle condition */
#define  USR2_ACST       (1<<11) /* Autobaud Controller Stopped*/
#define  USR2_RIDELT     (1<<10) /* Ring Indicator Delta */
#define  USR2_RIIN       (1<<9)  /* Ring Indicator Input*/
#define  USR2_IRINT      (1<<8)  /* Serial infrared interrupt flag */
#define  USR2_WAKE       (1<<7)  /* Wake */
#define  USR2_DCDDELT    (1<<6)  /* Data Carrier Delta Detect */
#define  USR2_DCDIN      (1<<5)  /* Data Carrier Detect Input */
#define  USR2_RTSF       (1<<4)  /* RTS edge interrupt flag */
#define  USR2_TXDC       (1<<3)  /* Transmitter complete */
#define  USR2_BRCD       (1<<2)  /* Break condition */
#define  USR2_ORE        (1<<1)  /* Overrun error */
#define  USR2_RDR        (1<<0)  /* Recv data ready */

#define  UTS_FRCPERR     (1<<13) /* Force parity error */
#define  UTS_LOOP        (1<<12) /* Loop tx and rx */
#define  UTS_DBGEN       (1<<11) /* /Debug Enable */
#define  UTS_LOOPIR      (1<<10) /* Loop tx and rx for IR */
#define  UTS_RXFIFO      (1<<9)  /* RXFifo Debug */
#define  UTS_TXEMPTY     (1<<6)  /* TxFIFO empty */
#define  UTS_RXEMPTY     (1<<5)  /* RxFIFO empty */
#define  UTS_TXFULL      (1<<4)  /* TxFIFO full */
#define  UTS_RXFULL      (1<<3)  /* RxFIFO full */
#define  UTS_SOFTRST     (1<<0)  /* Software reset */

/*
 * USB OTG controller
 */
#define USBOTG_HWMODE       __REG(IMX_USBOTG_BASE + 0x00)
#define USBOTG_HWMODE_ANASDBEN          (1 << 14)
#define USBOTG_HWMODE_OTGXCVR_MASK      (3 << 6)
#define USBOTG_HWMODE_OTGXCVR_TD_RD     (0 << 6)
#define USBOTG_HWMODE_OTGXCVR_TS_RD     (2 << 6)
#define USBOTG_HWMODE_OTGXCVR_TD_RS     (1 << 6)
#define USBOTG_HWMODE_OTGXCVR_TS_RS     (3 << 6)
#define USBOTG_HWMODE_HOSTXCVR_MASK     (3 << 4)
#define USBOTG_HWMODE_HOSTXCVR_TD_RD    (0 << 4)
#define USBOTG_HWMODE_HOSTXCVR_TS_RD    (2 << 4)
#define USBOTG_HWMODE_HOSTXCVR_TD_RS    (1 << 4)
#define USBOTG_HWMODE_HOSTXCVR_TS_RS    (3 << 4)
#define USBOTG_HWMODE_CRECFG_MASK       (3 << 0)
#define USBOTG_HWMODE_CRECFG_HOST       (1 << 0)
#define USBOTG_HWMODE_CRECFG_FUNC       (2 << 0)
#define USBOTG_HWMODE_CRECFG_HNP        (3 << 0)

#define USBOTG_CINT_STAT    __REG(IMX_USBOTG_BASE + 0x04)
#define USBOTG_CINT_STEN    __REG(IMX_USBOTG_BASE + 0x08)
#define USBOTG_ASHNPINT                 (1 << 5)
#define USBOTG_ASFCINT                  (1 << 4)
#define USBOTG_ASHCINT                  (1 << 3)
#define USBOTG_SHNPINT                  (1 << 2)
#define USBOTG_FCINT                    (1 << 1)
#define USBOTG_HCINT                    (1 << 0)

#define USBOTG_CLK_CTRL     __REG(IMX_USBOTG_BASE + 0x0c)
#define USBOTG_CLK_CTRL_FUNC            (1 << 2)
#define USBOTG_CLK_CTRL_HST             (1 << 1)
#define USBOTG_CLK_CTRL_MAIN            (1 << 0)

#define USBOTG_RST_CTRL     __REG(IMX_USBOTG_BASE + 0x10)
#define USBOTG_RST_RSTI2C               (1 << 15)
#define USBOTG_RST_RSTCTRL              (1 << 5)
#define USBOTG_RST_RSTFC                (1 << 4)
#define USBOTG_RST_RSTFSKE              (1 << 3)
#define USBOTG_RST_RSTRH                (1 << 2)
#define USBOTG_RST_RSTHSIE              (1 << 1)
#define USBOTG_RST_RSTHC                (1 << 0)

#define USBOTG_FRM_INTVL    __REG(IMX_USBOTG_BASE + 0x14)
#define USBOTG_FRM_REMAIN   __REG(IMX_USBOTG_BASE + 0x18)
#define USBOTG_HNP_CSR      __REG(IMX_USBOTG_BASE + 0x1c)
#define USBOTG_HNP_ISR      __REG(IMX_USBOTG_BASE + 0x2c)
#define USBOTG_HNP_IEN      __REG(IMX_USBOTG_BASE + 0x30)

#define USBOTG_I2C_TXCVR_REG(x)      __REG8(IMX_USBOTG_BASE + 0x100 + (x))
#define USBOTG_I2C_XCVR_DEVAD        __REG8(IMX_USBOTG_BASE + 0x118)
#define USBOTG_I2C_SEQ_OP_REG        __REG8(IMX_USBOTG_BASE + 0x119)
#define USBOTG_I2C_SEQ_RD_STARTAD    __REG8(IMX_USBOTG_BASE + 0x11a)
#define USBOTG_I2C_OP_CTRL_REG       __REG8(IMX_USBOTG_BASE + 0x11b)
#define USBOTG_I2C_SCLK_TO_SCK_HPER  __REG8(IMX_USBOTG_BASE + 0x11e)
#define USBOTG_I2C_MASTER_INT_REG    __REG8(IMX_USBOTG_BASE + 0x11f)

#define USBH_HOST_CTRL      __REG(IMX_USBOTG_BASE + 0x80)
#define USBH_HOST_CTRL_HCRESET                (1 << 31)
#define USBH_HOST_CTRL_SCHDOVR(x)             ((x) << 16)
#define USBH_HOST_CTRL_RMTWUEN                (1 << 4)
#define USBH_HOST_CTRL_HCUSBSTE_RESET         (0 << 2)
#define USBH_HOST_CTRL_HCUSBSTE_RESUME        (1 << 2)
#define USBH_HOST_CTRL_HCUSBSTE_OPERATIONAL   (2 << 2)
#define USBH_HOST_CTRL_HCUSBSTE_SUSPEND       (3 << 2)
#define USBH_HOST_CTRL_CTLBLKSR_1             (0 << 0)
#define USBH_HOST_CTRL_CTLBLKSR_2             (1 << 0)
#define USBH_HOST_CTRL_CTLBLKSR_3             (2 << 0)
#define USBH_HOST_CTRL_CTLBLKSR_4             (3 << 0)

#define USBH_SYSISR         __REG(IMX_USBOTG_BASE + 0x88)
#define USBH_SYSISR_PSCINT                    (1 << 6)
#define USBH_SYSISR_FMOFINT                   (1 << 5)
#define USBH_SYSISR_HERRINT                   (1 << 4)
#define USBH_SYSISR_RESDETINT                 (1 << 3)
#define USBH_SYSISR_SOFINT                    (1 << 2)
#define USBH_SYSISR_DONEINT                   (1 << 1)
#define USBH_SYSISR_SORINT                    (1 << 0)

#define USBH_SYSIEN         __REG(IMX_USBOTG_BASE + 0x8c)
#define USBH_SYSIEN_PSCINT                    (1 << 6)
#define USBH_SYSIEN_FMOFINT                   (1 << 5)
#define USBH_SYSIEN_HERRINT                   (1 << 4)
#define USBH_SYSIEN_RESDETINT                 (1 << 3)
#define USBH_SYSIEN_SOFINT                    (1 << 2)
#define USBH_SYSIEN_DONEINT                   (1 << 1)
#define USBH_SYSIEN_SORINT                    (1 << 0)

#define USBH_XBUFSTAT       __REG(IMX_USBOTG_BASE + 0x98)
#define USBH_YBUFSTAT       __REG(IMX_USBOTG_BASE + 0x9c)
#define USBH_XYINTEN        __REG(IMX_USBOTG_BASE + 0xa0)
#define USBH_XFILLSTAT      __REG(IMX_USBOTG_BASE + 0xa8)
#define USBH_YFILLSTAT      __REG(IMX_USBOTG_BASE + 0xac)
#define USBH_ETDENSET       __REG(IMX_USBOTG_BASE + 0xc0)
#define USBH_ETDENCLR       __REG(IMX_USBOTG_BASE + 0xc4)
#define USBH_IMMEDINT       __REG(IMX_USBOTG_BASE + 0xcc)
#define USBH_ETDDONESTAT    __REG(IMX_USBOTG_BASE + 0xd0)
#define USBH_ETDDONEEN      __REG(IMX_USBOTG_BASE + 0xd4)
#define USBH_FRMNUB         __REG(IMX_USBOTG_BASE + 0xe0)
#define USBH_LSTHRESH       __REG(IMX_USBOTG_BASE + 0xe4)
#define USBH_ROOTHUBA       __REG(IMX_USBOTG_BASE + 0xe8)
#define USBH_ROOTHUBA_PWRTOGOOD_MASK          (0xff)
#define USBH_ROOTHUBA_PWRTOGOOD_SHIFT         (24)
#define USBH_ROOTHUBA_NOOVRCURP               (1 << 12)
#define USBH_ROOTHUBA_OVRCURPM                (1 << 11)
#define USBH_ROOTHUBA_DEVTYPE                 (1 << 10)
#define USBH_ROOTHUBA_PWRSWTMD                (1 << 9)
#define USBH_ROOTHUBA_NOPWRSWT                (1 << 8)
#define USBH_ROOTHUBA_NDNSTMPRT_MASK          (0xff)

#define USBH_ROOTHUBB       __REG(IMX_USBOTG_BASE + 0xec)
#define USBH_ROOTHUBB_PRTPWRCM(x)             (1 << ((x) + 16))
#define USBH_ROOTHUBB_DEVREMOVE(x)            (1 << (x))

#define USBH_ROOTSTAT       __REG(IMX_USBOTG_BASE + 0xf0)
#define USBH_ROOTSTAT_CLRRMTWUE               (1 << 31)
#define USBH_ROOTSTAT_OVRCURCHG               (1 << 17)
#define USBH_ROOTSTAT_DEVCONWUE               (1 << 15)
#define USBH_ROOTSTAT_OVRCURI                 (1 << 1)
#define USBH_ROOTSTAT_LOCPWRS                 (1 << 0)

#define USBH_PORTSTAT(x)    __REG(IMX_USBOTG_BASE + 0xf4 + ((x) * 4))
#define USBH_PORTSTAT_PRTRSTSC                (1 << 20)
#define USBH_PORTSTAT_OVRCURIC                (1 << 19)
#define USBH_PORTSTAT_PRTSTATSC               (1 << 18)
#define USBH_PORTSTAT_PRTENBLSC               (1 << 17)
#define USBH_PORTSTAT_CONNECTSC               (1 << 16)
#define USBH_PORTSTAT_LSDEVCON                (1 << 9)
#define USBH_PORTSTAT_PRTPWRST                (1 << 8)
#define USBH_PORTSTAT_PRTRSTST                (1 << 4)
#define USBH_PORTSTAT_PRTOVRCURI              (1 << 3)
#define USBH_PORTSTAT_PRTSUSPST               (1 << 2)
#define USBH_PORTSTAT_PRTENABST               (1 << 1)
#define USBH_PORTSTAT_CURCONST                (1 << 0)

#define USB_DMAREV          __REG(IMX_USBOTG_BASE + 0x800)
#define USB_DMAINTSTAT      __REG(IMX_USBOTG_BASE + 0x804)
#define USB_DMAINTSTAT_EPERR                  (1 << 1)
#define USB_DMAINTSTAT_ETDERR                 (1 << 0)

#define USB_DMAINTEN        __REG(IMX_USBOTG_BASE + 0x808)
#define USB_DMAINTEN_EPERRINTEN               (1 << 1)
#define USB_DMAINTEN_ETDERRINTEN              (1 << 0)

#define USB_ETDDMAERSTAT    __REG(IMX_USBOTG_BASE + 0x80c)
#define USB_EPDMAERSTAT     __REG(IMX_USBOTG_BASE + 0x810)
#define USB_ETDDMAEN        __REG(IMX_USBOTG_BASE + 0x820)
#define USB_EPDMAEN         __REG(IMX_USBOTG_BASE + 0x824)
#define USB_ETDDMAXTEN      __REG(IMX_USBOTG_BASE + 0x828)
#define USB_EPDMAXTEN       __REG(IMX_USBOTG_BASE + 0x82c)
#define USB_ETDDMAENXYT     __REG(IMX_USBOTG_BASE + 0x830)
#define USB_EPDMAENXYT      __REG(IMX_USBOTG_BASE + 0x834)
#define USB_ETDDMABST4EN    __REG(IMX_USBOTG_BASE + 0x838)
#define USB_EPDMABST4EN     __REG(IMX_USBOTG_BASE + 0x83c)

#define USB_MISCCONTROL     __REG(IMX_USBOTG_BASE + 0x840)
#define USB_MISCCONTROL_ISOPREVFRM            (1 << 3)
#define USB_MISCCONTROL_SKPRTRY               (1 << 2)
#define USB_MISCCONTROL_ARBMODE               (1 << 1)
#define USB_MISCCONTROL_FILTCC                (1 << 0)

#define USB_ETDDMACHANLCLR  __REG(IMX_USBOTG_BASE + 0x848)
#define USB_EPDMACHANLCLR   __REG(IMX_USBOTG_BASE + 0x84c)
#define USB_ETDSMSA(x)      __REG(IMX_USBOTG_BASE + 0x900 + ((x) * 4))
#define USB_EPSMSA(x)       __REG(IMX_USBOTG_BASE + 0x980 + ((x) * 4))
#define USB_ETDDMABUFPTR(x) __REG(IMX_USBOTG_BASE + 0xa00 + ((x) * 4))
#define USB_EPDMABUFPTR(x)  __REG(IMX_USBOTG_BASE + 0xa80 + ((x) * 4))

#define USB_NUM_ETD         32
#define USB_ETD(x)          __REG(IMX_USBOTG_BASE + 0x200 + ((x) * 16))

#define USBCTRL             __REG(IMX_USBOTG_BASE + 0x600)
#define USBCTRL_I2C_WU_INT_STAT         (1 << 27)
#define USBCTRL_OTG_WU_INT_STAT         (1 << 26)
#define USBCTRL_HOST_WU_INT_STAT        (1 << 25)
#define USBCTRL_FNT_WU_INT_STAT         (1 << 24)
#define USBCTRL_I2C_WU_INT_EN           (1 << 19)
#define USBCTRL_OTG_WU_INT_EN           (1 << 18)
#define USBCTRL_HOST_WU_INT_EN          (1 << 17)
#define USBCTRL_FNT_WU_INT_EN           (1 << 16)
#define USBCTRL_OTC_RCV_RXDP            (1 << 13)
#define USBCTRL_HOST1_BYP_TLL           (1 << 12)
#define USBCTRL_OTG_BYP_VAL(x)          ((x) << 10)
#define USBCTRL_HOST1_BYP_VAL(x)        ((x) << 8)
#define USBCTRL_OTG_PWR_MASK            (1 << 6)
#define USBCTRL_HOST1_PWR_MASK          (1 << 5)
#define USBCTRL_HOST2_PWR_MASK          (1 << 4)
#define USBCTRL_USB_BYP                 (1 << 2)
#define USBCTRL_HOST1_TXEN_OE           (1 << 1)

/* Keypad port */
#define KPP_KPCR            __REG16(IMX_KPP_BASE + 0x0)
#define KPP_KPSR            __REG16(IMX_KPP_BASE + 0x2)
#define KPP_KPSR_KPKD          (1 <<  0)
#define KPP_KPSR_KPKR          (1 <<  1)
#define KPP_KPSR_KDSC          (1 <<  2)
#define KPP_KPSR_KRSS          (1 <<  3)
#define KPP_KPSR_KDIE          (1 <<  8)
#define KPP_KPSR_KRIE          (1 <<  9)
#define KPP_KPSR_KPPEN         (1 << 10)
#define KPP_KDDR            __REG16(IMX_KPP_BASE + 0x4)
#define KPP_KPDR            __REG16(IMX_KPP_BASE + 0x6)


// enhanced MultiMedia Accelerator (eMMA)

#define EMMA_PP_CNTL                    __REG(IMX_EMMA_BASE + 0x000)
#define EMMA_PP_INTRCNTL                __REG(IMX_EMMA_BASE + 0x004)
#define EMMA_PP_INTRSTATUS              __REG(IMX_EMMA_BASE + 0x008)
#define EMMA_PP_SOURCE_Y_PTR            __REG(IMX_EMMA_BASE + 0x00C)
#define EMMA_PP_SOURCE_CB_PTR           __REG(IMX_EMMA_BASE + 0x010)
#define EMMA_PP_SOURCE_CR_PTR           __REG(IMX_EMMA_BASE + 0x014)
#define EMMA_PP_DEST_RGB_PTR            __REG(IMX_EMMA_BASE + 0x018)
#define EMMA_PP_QUANTIZER_PTR           __REG(IMX_EMMA_BASE + 0x01C)
#define EMMA_PP_PROCESS_FRAME_PARA      __REG(IMX_EMMA_BASE + 0x020)
#define EMMA_PP_SOURCE_FRAME_WIDTH      __REG(IMX_EMMA_BASE + 0x024)
#define EMMA_PP_DEST_DISPLAY_WIDTH      __REG(IMX_EMMA_BASE + 0x028)
#define EMMA_PP_DEST_IMAGE_SIZE         __REG(IMX_EMMA_BASE + 0x02C)
#define EMMA_PP_DEST_FRAME_FMT_CNTL     __REG(IMX_EMMA_BASE + 0x030)
#define EMMA_PP_RESIZE_TBL_INDEX        __REG(IMX_EMMA_BASE + 0x034)
#define EMMA_PP_CSC_COEF_0123           __REG(IMX_EMMA_BASE + 0x038)
#define EMMA_PP_CSC_COEF_4              __REG(IMX_EMMA_BASE + 0x03C)
#define EMMA_PP_RESIZE_COEF_TBL(x)      __REG2(IMX_EMMA_BASE + 0x100, (x<<2))

#define EMMA_PrP_CNTL                   __REG(IMX_EMMA_BASE + 0x400)
#define EMMA_PrP_INTRCNTL               __REG(IMX_EMMA_BASE + 0x404)
#define EMMA_PrP_INTRSTATUS             __REG(IMX_EMMA_BASE + 0x408)
#define EMMA_PrP_SOURCE_Y_PTR           __REG(IMX_EMMA_BASE + 0x40C)
#define EMMA_PrP_SOURCE_CB_PTR          __REG(IMX_EMMA_BASE + 0x410)
#define EMMA_PrP_SOURCE_CR_PTR          __REG(IMX_EMMA_BASE + 0x414)
#define EMMA_PrP_DEST_RGB1_PTR          __REG(IMX_EMMA_BASE + 0x418)
#define EMMA_PrP_DEST_RGB2_PTR          __REG(IMX_EMMA_BASE + 0x41C)
#define EMMA_PrP_DEST_Y_PTR             __REG(IMX_EMMA_BASE + 0x420)
#define EMMA_PrP_DEST_CB_PTR            __REG(IMX_EMMA_BASE + 0x424)
#define EMMA_PrP_DEST_CR_PTR            __REG(IMX_EMMA_BASE + 0x428)
#define EMMA_PrP_SOURCE_FRAME_SIZE      __REG(IMX_EMMA_BASE + 0x42C)
#define EMMA_PrP_CH1_LINE_STRIDE        __REG(IMX_EMMA_BASE + 0x430)
#define EMMA_PrP_SRC_PIXEL_FORMAT_CNTL  __REG(IMX_EMMA_BASE + 0x434)
#define EMMA_PrP_CH1_PIXEL_FORMAT_CNTL  __REG(IMX_EMMA_BASE + 0x438)
#define EMMA_PrP_CH1_OUT_IMAGE_SIZE     __REG(IMX_EMMA_BASE + 0x43C)
#define EMMA_PrP_CH2_OUT_IMAGE_SIZE     __REG(IMX_EMMA_BASE + 0x440)
#define EMMA_PrP_SOURCE_LINE_STRIDE     __REG(IMX_EMMA_BASE + 0x444)
#define EMMA_PrP_CSC_COEF_012           __REG(IMX_EMMA_BASE + 0x448)
#define EMMA_PrP_CSC_COEF_345           __REG(IMX_EMMA_BASE + 0x44C)
#define EMMA_PrP_CSC_COEF_678           __REG(IMX_EMMA_BASE + 0x450)
#define EMMA_PrP_CH1_RZ_HORI_COEF1      __REG(IMX_EMMA_BASE + 0x454)
#define EMMA_PrP_CH1_RZ_HORI_COEF2      __REG(IMX_EMMA_BASE + 0x458)
#define EMMA_PrP_CH1_RZ_HORI_VALID      __REG(IMX_EMMA_BASE + 0x45C)
#define EMMA_PrP_CH1_RZ_VERT_COEF1      __REG(IMX_EMMA_BASE + 0x460)
#define EMMA_PrP_CH1_RZ_VERT_COEF2      __REG(IMX_EMMA_BASE + 0x464)
#define EMMA_PrP_CH1_RZ_VERT_VALID      __REG(IMX_EMMA_BASE + 0x468)
#define EMMA_PrP_CH2_RZ_HORI_COEF1      __REG(IMX_EMMA_BASE + 0x46C)
#define EMMA_PrP_CH2_RZ_HORI_COEF2      __REG(IMX_EMMA_BASE + 0x470)
#define EMMA_PrP_CH2_RZ_HORI_VALID      __REG(IMX_EMMA_BASE + 0x474)
#define EMMA_PrP_CH2_RZ_VERT_COEF1      __REG(IMX_EMMA_BASE + 0x478)
#define EMMA_PrP_CH2_RZ_VERT_COEF2      __REG(IMX_EMMA_BASE + 0x47C)
#define EMMA_PrP_CH2_RZ_VERT_VALID      __REG(IMX_EMMA_BASE + 0x480)

#else
#error No CONFIG_ARCH_ defined
#endif       

#endif                          // _IMX_REGS_H
