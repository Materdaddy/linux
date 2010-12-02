/*
 *  Driver for TSC2100 on the Chumby i.MX21
 *  Copyright (C) 2006 Jay Monkman <jtm@lopingdog.com>
 *                2007 Greg Hutchins <ghutchins@gmail.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License.
 *
 *  NOTES:
 *    This driver configures the CODEC to generate the clock and frame
 *         signals.
 *
 *
 *  TODO:
 *     handle power management
 */

// TODO: bunnie, comment this #define out once the speaker enable/disable 
// is controlled by the other driver
#define ENABLE_SPEAKER_KLUDGE_UNTIL_OTHER_DRIVER_PORTED

#include <linux/config.h>
#include <sound/driver.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/ioctl.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/dma.h>
#include <asm/arch/imx-dma.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/control.h>
#include <sound/initval.h>

#include <linux/mfd/tsc2100.h> 
#include "tsc2100.h"


/***********************************************************************
 * Module stuff
 ***********************************************************************/
static char *id = NULL;
module_param(id, charp, 0444);

MODULE_AUTHOR("Chumby <xxx@chumby.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Chumby i.MX21 ALSA Audio driver (TI TSC2100 CODEC)");
MODULE_PARM_DESC(id, "Chumby i.MX21/ALSA-TSC2100.");



/***********************************************************************
 * Data structures
 ***********************************************************************/
typedef struct {
    snd_card_t *card;
    snd_pcm_t *pcm;

    //playback
    imx_dmach_t p_dma;
    snd_pcm_substream_t *p_substream;
    int p_pos;
    u32 p_per_size;
    int p_period;

    // capture
    imx_dmach_t c_dma;
    snd_pcm_substream_t *c_substream;
    int c_pos;
    u32 c_per_size;
    int c_period;
} imx21_chip_t;



/***********************************************************************
 * Global Variables
 ***********************************************************************/
static unsigned long prev_jiffy;

static snd_device_ops_t g_mixer_ops = {
    .dev_free = NULL,
};



static unsigned int rates[] = {8000, 44100, 48000};
static snd_pcm_hw_constraint_list_t hw_constraints_rates = {
    .count = sizeof(rates) / sizeof(rates[0]),
    .list  = rates,
    .mask  = 0,
};

static snd_pcm_hardware_t snd_imx21_playback_hw = {
    .info = (SNDRV_PCM_INFO_INTERLEAVED    |
             SNDRV_PCM_INFO_BLOCK_TRANSFER |
             SNDRV_PCM_INFO_MMAP           |
             SNDRV_PCM_INFO_MMAP_VALID),
    .formats = SNDRV_PCM_FMTBIT_S16_LE,
    .rates = (SNDRV_PCM_RATE_8000  |
              SNDRV_PCM_RATE_44100 |
              SNDRV_PCM_RATE_48000),
    .rate_min = 8000,
    .rate_max = 48000,
    .channels_min = 1,
    .channels_max = 2,
    .buffer_bytes_max = 0x10000,
    .period_bytes_min = 0x4000,
    .period_bytes_max = 0x8000,
    .periods_min = 1,
    .periods_max = 2,
};

static snd_pcm_hardware_t snd_imx21_capture_hw = {
    .info = (SNDRV_PCM_INFO_INTERLEAVED    |
             SNDRV_PCM_INFO_BLOCK_TRANSFER |
             SNDRV_PCM_INFO_MMAP           |
             SNDRV_PCM_INFO_MMAP_VALID),
    .formats = SNDRV_PCM_FMTBIT_S16_LE,
    .rates = (SNDRV_PCM_RATE_8000  |
              SNDRV_PCM_RATE_44100 |
              SNDRV_PCM_RATE_48000),
    .rate_min = 8000,
    .rate_max = 48000,
    .channels_min = 1,
    .channels_max = 2,
    .buffer_bytes_max = 0x10000,
    .period_bytes_min = 0x4000,
    .period_bytes_max = 0x8000,
    .periods_min = 1,
    .periods_max = 2,
};


///////////////////////////////////////////////////////////////////////////////
// SPI ROUTINES -- NEED TO MERGE TOUCHSCREEN & ALSA SOMEHOW!                 //
///////////////////////////////////////////////////////////////////////////////
#define SPIDEV              2
#define PENIRQ              IRQ_GPIOE(19)

#define TSC2100_ADC_DEFAULT (TSC2100_ADC_RES(TSC2100_ADC_RES_12BITP)   | \
                             TSC2100_ADC_AVG(TSC2100_ADC_4AVG)         | \
                             TSC2100_ADC_CL(TSC2100_ADC_CL_1MHZ_12BIT) | \
                             TSC2100_ADC_PV(TSC2100_ADC_PV_500us)      | \
                             TSC2100_ADC_AVGFILT_MEAN) 




static void gpio_init( void )
{
    imx_gpio_mode( 18 | GPIO_PORTE | GPIO_AF | GPIO_IN );
    imx_gpio_mode( 21 | GPIO_PORTE | GPIO_AF | GPIO_OUT );
    imx_gpio_mode( 22 | GPIO_PORTE | GPIO_AF | GPIO_OUT );
    imx_gpio_mode( 23 | GPIO_PORTE | GPIO_AF | GPIO_OUT );
    imx_gpio_mode( 19 | GPIO_PORTE | GPIO_PF | GPIO_IN | GPIO_PUEN | GPIO_GPIO );

    imx_gpio_mode( PC23_PF_SSI1_CLK );
    imx_gpio_mode( PC22_PF_SSI1_TX );
    imx_gpio_mode( PC21_PF_SSI1_RX );
    imx_gpio_mode( PC20_PF_SSI1_FS );
    imx_gpio_mode( PC27_PF_SSI2_CLK );
    imx_gpio_mode( PC26_PF_SSI2_TX );
    imx_gpio_mode( PC25_PF_SSI2_RX );
    imx_gpio_mode( PC24_PF_SSI2_FS );

    #ifdef ENABLE_SPEAKER_KLUDGE_UNTIL_OTHER_DRIVER_PORTED
    printk( "%s/%s().%d: speaker enable kludge KPP_KDDR=%x, KPP_KPDR=%x\n",
           __FILE__, __FUNCTION__, __LINE__, KPP_KDDR, KPP_KPDR );
    imx_gpio_mode( GPIO_PORTKP | (4 + 8) | GPIO_OUT );
    imx_gpio_write( GPIO_PORTKP | (4 + 8), 0 );
    printk( "%s/%s().%d: after KPP_KDDR=%x, KPP_KPDR=%x\n", 
            __FILE__, __FUNCTION__, __LINE__, KPP_KDDR, KPP_KPDR );

    #endif
}


static void spi_wait_for_idle( void )
{
    while (SSP_CTRL_REG(SPIDEV) & SSP_XCH)
        ndelay(50);
}


static void spi_flush_fifo( void )
{
    volatile int junk;
    while (SSP_INT_REG(SPIDEV) & SSP_INT_RR)
        junk = SSP_RX_REG(SPIDEV);
}


static void spi_read_fifo(int* rxdata, int rxlen)
{
    while (rxlen > 0)
        while (SSP_INT_REG(SPIDEV) & SSP_INT_RR) {
            *rxdata++ = SSP_RX_REG(SPIDEV) & 0xffff;
            if (--rxlen <= 0)
                break;
        }
}


static void spi_write_zeros(int zeros)
{
    while (zeros-- > 0)
        SSP_TX_REG(SPIDEV) = 0;
}


static void spi_init( void )
{
    unsigned int rate;
    unsigned int rate_div;

    SSP_RESET_REG(SPIDEV) = 1;
    udelay(200);

    rate = ( imx_get_perclk2() / 10000000 ) + 1;
    rate_div = SSP_RATE_DIV4;
    if ( rate > 4 )   rate_div = SSP_RATE_DIV6;
    if ( rate > 6 )   rate_div = SSP_RATE_DIV8;
    if ( rate > 8 )   rate_div = SSP_RATE_DIV12;
    if ( rate > 12 )  rate_div = SSP_RATE_DIV16;
    if ( rate > 16 )  rate_div = SSP_RATE_DIV256;

    PCCR1 |= PCCR1_CSPI3_EN; // enable PERCLK to CSPI3
    SSP_CTRL_REG(SPIDEV) = (
        rate_div        |
        SSP_MODE_MASTER |
        SSP_SS_POL_LOW  |
        SSP_ENABLE      |
        SSP_PHA1        |
        SSP_POL0        |
        SSP_WS(16));

    spi_flush_fifo();
}


static void spi_exit( void )
{
    SSP_CTRL_REG(SPIDEV) = SSP_DISABLE;
    PCCR1 &= ~PCCR1_CSPI3_EN; // disable PERCLK to CSPI3
}


static int spi_wait_for_txspace( void ) {

  while( !(SSP_CTRL_REG(SPIDEV) & SSP_INT_TH) )
    ndelay(50); // wait 50 nsecs...
    
}

static int spi_write( int regaddr, int regdata )
{
    spi_wait_for_idle();
    spi_flush_fifo();
    spi_wait_for_txspace();
    SSP_TX_REG(SPIDEV) = regaddr | TSC2100_WRITE;
    SSP_TX_REG(SPIDEV) = regdata;
    SSP_CTRL_REG(SPIDEV) |= SSP_XCH;
    return 0;
}


static int spi_read( int regaddr, int* rxdata, int rxlen )
{
    int junk;

    if ( rxlen > 7 )  
        rxlen = 7;
    spi_wait_for_idle();
    spi_flush_fifo();
    SSP_TX_REG(SPIDEV) = regaddr | TSC2100_READ;
    spi_write_zeros(rxlen);
    SSP_CTRL_REG(SPIDEV) |= SSP_XCH;
    spi_read_fifo(&junk, 1);
    spi_read_fifo(rxdata, rxlen);
    return rxdata[0];
}


static int tsc2100_regread(int regnum)
{
    int reg;
    return spi_read(regnum, &reg, 1);
}


static void tsc2100_regwrite(int regnum, int value)
{
    spi_write( regnum, value );
}
///////////////////////////////////////////////////////////////////////////////
// SPI ROUTINE END                                                           //
///////////////////////////////////////////////////////////////////////////////

// the P/J/D values are based on a 12Mhz MCLK
// if N == 0 -> 48000 sample clock
// if N == 7 --> 8000 sample clock

static void pll_fsref_48000( int N )
{
    tsc2100_regwrite(TSC2100_REG_CONTROL1,  
                     0 << 14 |  // ADC high pass filter disabled
                     3 << 12 |  // ADC input mux single-ended AUX
                     0 << 10 |  // codec 16-bit word length
                     0 << 8  |  // I2S mode
                     N << 3  |  // DAC sampling rate Fsref/N (sort of)
                     N << 0);   // ADC sampling rate Fsref/N (sort of)
    tsc2100_regwrite(TSC2100_REG_CONTROL3,  
                     0 << 13 |  // Fsref == 48.0 kHz
                     0 << 12 |  // continuous data transfer mode
                     1 << 11 |  // TSC2100 is master codec
                     3 << 9);   // DAC max output signal swing 2.633 V
    tsc2100_regwrite(TSC2100_REG_PLL1,      
                     1 << 15 |  // enable PLL
                     1 << 8  |  // P value is 1 for 48.0 kHz
                     8 << 2);   // J value is 8 for 48.0 kHz
    tsc2100_regwrite(TSC2100_REG_PLL2,      
                     1920<<2);  // D value is 1920 for 48.0 kHz
}


static void pll_fsref_44100( void )
{
    tsc2100_regwrite(TSC2100_REG_CONTROL1,  
                     0 << 14 |  // ADC high pass filter disabled
                     3 << 12 |  // ADC input mux single-ended AUX
                     0 << 10 |  // codec 16-bit word length
                     0 << 8  |  // I2S mode
                     0 << 3  |  // DAC sampling rate Fsref/1
                     0 << 0);   // ADC sampling rate Fsref/1
    tsc2100_regwrite(TSC2100_REG_CONTROL3,  
                     1 << 13 |  // Fsref == 44.1 kHz
                     0 << 12 |  // continuous data transfer mode
                     1 << 11 |  // TSC2100 is master codec
                     3 << 9);   // DAC max output signal swing 2.633 V
    tsc2100_regwrite(TSC2100_REG_PLL1,      
                     1 << 15 |  // enable PLL
                     1 << 8  |  // P value is 1 for 44.1 kHz
                     7 << 2);   // J value is 7 for 44.1 kHz
    tsc2100_regwrite(TSC2100_REG_PLL2,      
                     5264<<2);  // D value is 5264 for 44.1 kHz
}


/************************************************************
 * PCM Functions
 ************************************************************/
static int snd_imx21_playback_open(snd_pcm_substream_t *substream)
{
    int rc;
    imx21_chip_t *chip = snd_pcm_substream_chip(substream);
    snd_pcm_runtime_t *runtime = substream->runtime;

    runtime->hw = snd_imx21_playback_hw;
    chip->p_substream = substream;

    rc = snd_pcm_hw_constraint_list(runtime, 0, 
                                    SNDRV_PCM_HW_PARAM_RATE,
                                    &hw_constraints_rates);
    if (rc < 0) {
        printk("Error setting rates constraint, rc %d\n", rc);
        return rc;
    }
    chip->p_pos = 0;

    return 0;
}


static int snd_imx21_playback_close(snd_pcm_substream_t *substream)
{
    imx21_chip_t *chip = snd_pcm_substream_chip(substream);
    chip->p_substream = NULL;
    return 0;
}


static int snd_imx21_pcm_hw_params(snd_pcm_substream_t *substream,
                                    snd_pcm_hw_params_t *hw_params)
{
    int err;
    err = snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(hw_params));
    //printk( "%s(): err %d\n", __FUNCTION__, err );
    return err;
}


static int snd_imx21_pcm_hw_free(snd_pcm_substream_t *substream)
{
    int err;

    err = snd_pcm_lib_free_pages(substream);
    //printk( "%s(): err %d\n", __FUNCTION__, err );
    return err;
}


static inline void setup_dma_xfer(int chan, u32 src, 
                                  u32 dest, int count)
{
    //printk( "%s(chan=%x, src=%x, dest=%x, count=%x): ...\n", 
    //        __FUNCTION__, chan, src, dest, count );
    SAR(chan) = src;
    DAR(chan) = dest;
    CNTR(chan) = count;
}


static int snd_imx21_pcm_playback_prepare(snd_pcm_substream_t *substream)
{
    imx21_chip_t *chip = snd_pcm_substream_chip(substream);
    snd_pcm_runtime_t *runtime = substream->runtime;

    //printk( "%s(): runtime->rate == %d\n", __FUNCTION__, runtime->rate );

    tsc2100_regwrite(TSC2100_REG_AUDIOADC,  
                     0x00 << 15 |  // ADC Channel not muted
                     0x77 << 8  |  // ADC PGA = 59.5 dB
                     0x07 << 5  |  // AGC target level -24dB
                     0x04 << 1  |  // AGC Time Constant -- attack 8ms, decay 200ms
                     0x00 << 0);   // AGC is on
    tsc2100_regwrite(TSC2100_REG_AUDIODAC,
                     0x00 << 15 |  // DAC left channel not muted
                     0x00 << 8  |  // DAC left channel volume control -30dB
                     0x00 << 7  |  // DAC right channel not muted
                     0x00 << 0);   // DAC right channel volume control -30dB
    tsc2100_regwrite(TSC2100_REG_POWERCON,
                     0 << 15  |    // codec powered up
                     1 << 13  |    // analog sidetone powered down
                     1 << 12  |    // output driver in high power mode
                     1 << 11  |    // sidetone power down complete (RO?)
                     0 << 10  |    // power up the DAC
                     0 << 9   |    // power up the ADC
                     1 << 8   |    // power down the VGND amp (?)
                     1 << 5);      // ADWS pin acts as ADC word-select
                     
    switch(runtime->rate) {
    case 8000:
        /* 16 bit, 8KHz, 24 samples/frame */
        SSI1_STCCR = (SSI_STCCR_WL(16) | SSI_STCCR_DC(23) | SSI_STCCR_PM(0));
        SSI1_SRCCR = (SSI_SRCCR_WL(16) | SSI_SRCCR_DC(23) | SSI_SRCCR_PM(0));
        SSI1_STMSK = ~((1 << 0) | (1 << 12));
        SSI1_SRMSK = ~((1 << 0) | (1 << 12));
        pll_fsref_48000(7);
        break;

    case 44100:
        /* 16 bit, 44.1KHz, 4 samples/frame */
        SSI1_STCCR = (SSI_STCCR_WL(16) | SSI_STCCR_DC(3) | SSI_STCCR_PM(0));
        SSI1_SRCCR = (SSI_SRCCR_WL(16) | SSI_SRCCR_DC(3) | SSI_SRCCR_PM(0));
        SSI1_STMSK = ~((1 << 0) | (1 << 2));
        SSI1_SRMSK = ~((1 << 0) | (1 << 2));
        pll_fsref_44100();
        break;

    case 48000:
        /* 16 bit, 48KHz, 4 samples/frame */
        SSI1_STCCR = (SSI_STCCR_WL(16) | SSI_STCCR_DC(3) | SSI_STCCR_PM(0));
        SSI1_SRCCR = (SSI_SRCCR_WL(16) | SSI_SRCCR_DC(3) | SSI_SRCCR_PM(0));
        SSI1_STMSK = ~((1 << 0) | (1 << 2));
        SSI1_SRMSK = ~((1 << 0) | (1 << 2));
        pll_fsref_48000(0);
        break;

    case 12000:
    case 16000:
    case 24000:
        // TODO: the hardware can support these rates, do we want to???

    default:
        printk("%s: Can't handle rate of %d\n", __FUNCTION__, runtime->rate);
        return -ENODEV;
        break;
    }

   /* Get physical address of data buffer */
    runtime->dma_addr = __pa(runtime->dma_area);

    chip->p_pos = 0;
    chip->p_period = 0;
    chip->p_per_size = frames_to_bytes(runtime, runtime->period_size);

    CCR(chip->p_dma) = (CCR_DMOD_FIFO   |
                        CCR_SMOD_LINEAR |
                        CCR_DSIZ_16     |
                        CCR_SSIZ_32     |
                        CCR_REN);

    RSSR(chip->p_dma) = DMA_REQ_SSI1_TX0;
    BLR(chip->p_dma) = 4;
    BUCR(chip->p_dma) = 0;

    return 0;
}


static void imx21_dma_dump(imx21_chip_t* chip)
{
    #if 0
    printk( "DMA(%d): CNTR=%x SAR=%x DAR=%x CCR=%x RSSR=%x BLR=%x BUCR=%x\n",
            chip->p_dma, CNTR(chip->p_dma), SAR(chip->p_dma), DAR(chip->p_dma),
            CCR(chip->p_dma), RSSR(chip->p_dma), BLR(chip->p_dma), 
            BUCR(chip->p_dma) );
    printk( "TSC2100: CONTROL1=%x CONTROL3=%x PLL1=%x PLL2=%x ADC=%x DAC=%x PWR=%x\n", 
            tsc2100_regread(TSC2100_REG_CONTROL1), 
            tsc2100_regread(TSC2100_REG_CONTROL3), 
            tsc2100_regread(TSC2100_REG_PLL1),      
            tsc2100_regread(TSC2100_REG_PLL2),
    	    tsc2100_regread(TSC2100_REG_AUDIOADC),
    	    tsc2100_regread(TSC2100_REG_AUDIODAC),
    	    tsc2100_regread(TSC2100_REG_POWERCON) );
    #endif
}


static int snd_imx21_pcm_playback_trigger(snd_pcm_substream_t *substream, int cmd)
{
    imx21_chip_t *chip = snd_pcm_substream_chip(substream);
    snd_pcm_runtime_t *runtime = substream->runtime;


    switch(cmd) {
    case SNDRV_PCM_TRIGGER_START:
	// printk("%d.%s(): SNDRV_PCM_TRIGGER_START called\n", 
    //       jiffies_to_usecs(jiffies-prev_jiffy), __FUNCTION__ );
    prev_jiffy = jiffies;
    //imx21_dma_dump(chip);

        /* If ALSA wants to give us two periods, set them both up  */

        while (CCR(chip->p_dma) & CCR_CEN) {
            printk("TRIGGER_START while DMA is already enabled\n");
        }
        setup_dma_xfer(chip->p_dma,
                       runtime->dma_addr, (u32)&SSI1_STX0_PHYS, 
                       chip->p_per_size);
        CCR(chip->p_dma) |= CCR_ACRPT;
        chip->p_period = 1; 
        SSI1_SCR |= SSI_SCR_TE;
        SSI1_SIER |= SSI_SIER_TDMAE;
        imx_dma_enable(chip->p_dma);

        if ((runtime->periods == 2)) {
            CCR(chip->p_dma) |= CCR_RPT;
            setup_dma_xfer(chip->p_dma, 
                           runtime->dma_addr + chip->p_per_size,
                           (u32)&SSI1_STX0_PHYS, 
                           chip->p_per_size);
            chip->p_period = 0;
        }
        break;
    case SNDRV_PCM_TRIGGER_STOP:
	// printk("%d.%s(): SNDRV_PCM_TRIGGER_STOP called\n", 
    //       jiffies_to_usecs(jiffies-prev_jiffy), __FUNCTION__ );
    //imx21_dma_dump(chip);
        imx_dma_disable(chip->p_dma);
        CCR(chip->p_dma) &= ~(CCR_RPT | CCR_ACRPT);
        SSI1_SIER &= ~SSI_SIER_TDMAE;
        SSI1_SCR &= ~SSI_SCR_TE;
        break;
    default:
        printk("%s(): unknown command %d\n", __FUNCTION__, cmd);
        return -EINVAL;
        break;
    }

    return 0;
}


static snd_pcm_uframes_t snd_imx21_pcm_playback_pointer(
    snd_pcm_substream_t *substream)
{
    imx21_chip_t *chip = snd_pcm_substream_chip(substream);
    snd_pcm_runtime_t *runtime = substream->runtime;
    unsigned int offset;

    //printk( "%s(): chip->p_pos=%x, chip->p_per_size=%x, runtime->buffer_size=%lx\n", 
	//	 __FUNCTION__, chip->p_pos, chip->p_per_size, runtime->buffer_size );
    offset = bytes_to_frames(runtime, chip->p_pos % (2 * chip->p_per_size));
    if (offset >= runtime->buffer_size)
        offset = 0;
    //printk( "%s(): offset=%x\n", __FUNCTION__, offset );
    return offset;
}


static int snd_imx21_capture_open(snd_pcm_substream_t *substream)
{
    imx21_chip_t *chip = snd_pcm_substream_chip(substream);
    snd_pcm_runtime_t *runtime = substream->runtime;

    // printk( "%s(): called\n", __FUNCTION__ );
    runtime->hw = snd_imx21_capture_hw;
    chip->c_substream = substream;
    chip->c_substream = NULL;
    return 0;
}


static int snd_imx21_capture_close(snd_pcm_substream_t *substream)
{
    imx21_chip_t *chip = snd_pcm_substream_chip(substream);
    chip->c_substream = NULL;
    printk( "%s(): called\n", __FUNCTION__ );
    return 0;
}


static int snd_imx21_pcm_capture_prepare(snd_pcm_substream_t *substream)
{
    imx21_chip_t *chip = snd_pcm_substream_chip(substream);
    snd_pcm_runtime_t *runtime = substream->runtime;

    // printk( "%s(): runtime->rate == %d\n", __FUNCTION__, runtime->rate );
    switch(runtime->rate) {
    case 8000:
        /* 16 bit, 8KHz, 24 samples/frame */
        SSI1_STCCR = (SSI_STCCR_WL(16) | SSI_STCCR_DC(23) | SSI_STCCR_PM(0));
        SSI1_SRCCR = (SSI_SRCCR_WL(16) | SSI_SRCCR_DC(23) | SSI_SRCCR_PM(0));
        SSI1_STMSK = ~((1 << 0) | (1 << 12));
        SSI1_SRMSK = ~((1 << 0) | (1 << 12));
        pll_fsref_48000(7);
        break;

    case 44100:
        /* 16 bit, 44.1KHz, 4 samples/frame */
        SSI1_STCCR = (SSI_STCCR_WL(16) | SSI_STCCR_DC(3) | SSI_STCCR_PM(0));
        SSI1_SRCCR = (SSI_SRCCR_WL(16) | SSI_SRCCR_DC(3) | SSI_SRCCR_PM(0));
        SSI1_STMSK = ~((1 << 0) | (1 << 2));
        SSI1_SRMSK = ~((1 << 0) | (1 << 2));
        pll_fsref_44100();
        break;

    case 48000:
        /* 16 bit, 48KHz, 4 samples/frame */
        SSI1_STCCR = (SSI_STCCR_WL(16) | SSI_STCCR_DC(3) | SSI_STCCR_PM(0));
        SSI1_SRCCR = (SSI_SRCCR_WL(16) | SSI_SRCCR_DC(3) | SSI_SRCCR_PM(0));
        SSI1_STMSK = ~((1 << 0) | (1 << 2));
        SSI1_SRMSK = ~((1 << 0) | (1 << 2));
        pll_fsref_48000(0);
        break;

    default:
        printk("%s: Can't handle rate of %d\n", __FUNCTION__, runtime->rate);
        return -ENODEV;
        break;
    }

   /* Get physical address of data buffer */
    runtime->dma_addr = __pa(runtime->dma_area);

    chip->c_pos = 0;
    chip->c_period = 0;
    chip->c_per_size = frames_to_bytes(runtime, runtime->period_size);

    CCR(chip->c_dma) = (CCR_SMOD_FIFO   |
                        CCR_DMOD_LINEAR |
                        CCR_SSIZ_16     |
                        CCR_DSIZ_32     |
                        CCR_REN);

    RSSR(chip->c_dma) = DMA_REQ_SSI1_RX0;
    BLR(chip->c_dma) = 4;
    BUCR(chip->c_dma) = 0;

    return 0;
}


static int snd_imx21_pcm_capture_trigger(snd_pcm_substream_t *substream, int cmd)
{
    imx21_chip_t *chip = snd_pcm_substream_chip(substream);
    snd_pcm_runtime_t *runtime = substream->runtime;

    printk( "%s(cmd=%d): called\n", __FUNCTION__, cmd );
    switch(cmd) {
    case SNDRV_PCM_TRIGGER_START:

        /* If ALSA wants to give us two periods, set them both up  */

        while (CCR(chip->c_dma) & CCR_CEN) {
            printk("TRIGGER_START while DMA is already enabled\n");
        }
        setup_dma_xfer(chip->c_dma,
                       (u32)&SSI1_SRX0_PHYS, runtime->dma_addr, 
                       chip->c_per_size);
        CCR(chip->c_dma) |= CCR_ACRPT;
        chip->c_period = 1; 
        SSI1_SCR |= SSI_SCR_RE;
        SSI1_SIER |= SSI_SIER_RDMAE;
        imx_dma_enable(chip->c_dma);

        if ((runtime->periods == 2)) {
            CCR(chip->c_dma) |= CCR_RPT;
            setup_dma_xfer(chip->c_dma, 
                           (u32)&SSI1_SRX0_PHYS, 
                           runtime->dma_addr + chip->c_per_size,
                           chip->c_per_size);
            chip->c_period = 0;
        }

        break;
    case SNDRV_PCM_TRIGGER_STOP:
        imx_dma_disable(chip->c_dma);
        CCR(chip->c_dma) &= ~(CCR_RPT | CCR_ACRPT);
        SSI1_SIER &= ~SSI_SIER_RDMAE;
        SSI1_SCR &= ~SSI_SCR_RE;
        break;
    default:
        printk("%s(): unknown command %d\n", __FUNCTION__, cmd);
        return -EINVAL;
        break;
    }

    return 0;
}


static snd_pcm_uframes_t snd_imx21_pcm_capture_pointer(
    snd_pcm_substream_t *substream)
{
    imx21_chip_t *chip = snd_pcm_substream_chip(substream);
    snd_pcm_runtime_t *runtime = substream->runtime;
    unsigned int offset;

    offset = bytes_to_frames(runtime, chip->c_pos % (2 * chip->c_per_size));
    if (offset >= runtime->buffer_size)
        offset = 0;
    printk( "%s(): offset=%x\n", __FUNCTION__, offset );
    return offset;
}


static snd_pcm_ops_t snd_imx21_playback_ops = {
    .open = snd_imx21_playback_open,
    .close = snd_imx21_playback_close,
    .ioctl = snd_pcm_lib_ioctl,
    .hw_params = snd_imx21_pcm_hw_params,
    .hw_free = snd_imx21_pcm_hw_free,
    .prepare = snd_imx21_pcm_playback_prepare,
    .trigger = snd_imx21_pcm_playback_trigger,
    .pointer = snd_imx21_pcm_playback_pointer,
};

static snd_pcm_ops_t snd_imx21_capture_ops = {
    .open = snd_imx21_capture_open,
    .close = snd_imx21_capture_close,
    .ioctl = snd_pcm_lib_ioctl,
    .hw_params = snd_imx21_pcm_hw_params,
    .hw_free = snd_imx21_pcm_hw_free,
    .prepare = snd_imx21_pcm_capture_prepare,
    .trigger = snd_imx21_pcm_capture_trigger,
    .pointer = snd_imx21_pcm_capture_pointer,
};


static int __init snd_imx21_pcm_new(imx21_chip_t *chip)
{
    snd_pcm_t *pcm;
    int rc;

    rc = snd_pcm_new(chip->card, "Chumby-iMX21-TSC2100-PCM", 0, 1, 1, &pcm);
    if (rc < 0) {
        return rc;
    }

    pcm->private_data = chip;
    strcpy(pcm->name, "Chumby-iMX21-TSC2100-PCM");
    chip->pcm = pcm;

    snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_CONTINUOUS,
                                          snd_dma_continuous_data(GFP_KERNEL),
                                          64*1024, 64*1024);

    snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK,
                    &snd_imx21_playback_ops);

    snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE,
                    &snd_imx21_capture_ops);

    return 0;
}
 

/************************************************************
 * Mixer Functions
 ************************************************************/
static int snd_imx21_mixer_mvol_info(snd_kcontrol_t *kcontrol,
                                      snd_ctl_elem_info_t *uinfo)
{
    printk( "%s(): called\n", __FUNCTION__ );
    uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
    uinfo->count = 2;
    uinfo->value.integer.min = 0x00;
    uinfo->value.integer.max = 0x7F;
    return 0;
}


static int snd_imx21_mixer_mvol_get(snd_kcontrol_t *kcontrol,
                                     snd_ctl_elem_value_t *uctrl)
{
    u16 val;

    printk( "%s(): called\n", __FUNCTION__ );
    val = tsc2100_regread( TSC2100_REG_AUDIODAC );
    uctrl->value.integer.value[0] = ( val >> 8 ) & 0x7f;
    uctrl->value.integer.value[1] = val & 0x7f;
    return 0;
}


static int snd_imx21_mixer_mvol_put(snd_kcontrol_t *kcontrol,
                                snd_ctl_elem_value_t *uctrl)
{
    int changed = 0;
    u16 val;
    int lvol;
    int rvol;

    printk( "%s(): called\n", __FUNCTION__ );
    val = tsc2100_regread( TSC2100_REG_AUDIODAC );
    lvol = ( val >> 8 ) & 0x7f;
    rvol = val & 0x7f;

    if ((uctrl->value.integer.value[0] != lvol) || 
        (uctrl->value.integer.value[1] != rvol)) {
        changed = 1;

        lvol = uctrl->value.integer.value[0] & 0x7F;
        rvol = uctrl->value.integer.value[1] & 0x7F;
    }

    tsc2100_regwrite( TSC2100_REG_AUDIODAC, (lvol<<8) | rvol );
    return changed;
}


static int snd_imx21_mixer_insel_info(snd_kcontrol_t *kcontrol,
                                       snd_ctl_elem_info_t *uinfo)
{
    printk( "%s(): called\n", __FUNCTION__ );
    uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
    uinfo->count = 1;
    uinfo->value.integer.min = 0;
    uinfo->value.integer.max = 1;
    return 0;
}


static int snd_imx21_mixer_insel_get(snd_kcontrol_t *kcontrol,
                                      snd_ctl_elem_value_t *uctrl)
{
    u16 val;

    printk( "%s(): called\n", __FUNCTION__ );
    val = tsc2100_regread(TSC2100_REG_CONTROL1);
    if (val & TSC2100_CONTROL1_ADCIN_MASK) {
        uctrl->value.integer.value[0] = 1;
    } else {
        uctrl->value.integer.value[0] = 0;
    }
    return 0;
}


static int snd_imx21_mixer_insel_put(snd_kcontrol_t *kcontrol,
                                snd_ctl_elem_value_t *uctrl)
{
    int changed;
    u16 val;

    printk( "%s(): called\n", __FUNCTION__ );
    val = tsc2100_regread(TSC2100_REG_CONTROL1);
    if ((val & TSC2100_CONTROL1_ADCIN_MASK) && uctrl->value.integer.value[0]) {
        changed = 0;
    } else {
        changed = 1;
    }

    val = (val & ~TSC2100_CONTROL1_ADCIN_MASK) | 
          TSC2100_CONTROL1_ADCIN(uctrl->value.integer.value[0]);
    tsc2100_regwrite(TSC2100_REG_CONTROL1, val);
    return changed;
}


static int snd_imx21_mixer_micboost_info(snd_kcontrol_t *kcontrol,
                                          snd_ctl_elem_info_t *uinfo)
{
    printk( "%s(): called\n", __FUNCTION__ );
    uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
    uinfo->count = 1;
    uinfo->value.integer.min = 0;
    uinfo->value.integer.max = 1;
    return 0;
}


static int snd_imx21_mixer_micboost_get(snd_kcontrol_t *kcontrol,
                                         snd_ctl_elem_value_t *uctrl)
{
    printk( "%s(): called\n", __FUNCTION__ );
    #if 0
    imx21_chip_t *chip = snd_kcontrol_chip(kcontrol);
    u16 val;

    // TODO: fixme
    tsc2100_regread(TSC2100_REG_AAP, &val);
    if (val & TSC2100_AAP_MICBOOST) {
        uctrl->value.integer.value[0] = 1;
    } else {
        uctrl->value.integer.value[0] = 0;
    }
    #endif
    return 0;
}


static int snd_imx21_mixer_micboost_put(snd_kcontrol_t *kcontrol,
                                snd_ctl_elem_value_t *uctrl)
{
    printk( "%s(): called\n", __FUNCTION__ );
    #if 1
    return 0;
    #else
    imx21_chip_t *chip = snd_kcontrol_chip(kcontrol);
    int changed;
    u16 val;

    // TODO: fixme
    tsc2100_regread(TSC2100_REG_AAP, &val);
    if ((val & TSC2100_AAP_MICBOOST) && uctrl->value.integer.value[0]) {
        changed = 0;
    } else {
        changed = 1;
    }

    if (uctrl->value.integer.value[0]) {
        val |= TSC2100_AAP_MICBOOST;
    } else {
        val &= ~TSC2100_AAP_MICBOOST;
    }
    
    tsc2100_regwrite(TSC2100_REG_AAP, val);
    return changed;
    #endif
}


static int snd_imx21_mixer_sidetone_info(snd_kcontrol_t *kcontrol,
                                          snd_ctl_elem_info_t *uinfo)
{
    printk( "%s(): called\n", __FUNCTION__ );
    uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
    uinfo->count = 1;
    uinfo->value.integer.min = 0;
    uinfo->value.integer.max = 1;
    return 0;
}


static int snd_imx21_mixer_sidetone_get(snd_kcontrol_t *kcontrol,
                                         snd_ctl_elem_value_t *uctrl)
{
    printk( "%s(): called\n", __FUNCTION__ );
    #if 0
    imx21_chip_t *chip = snd_kcontrol_chip(kcontrol);
    u16 val;

    // TODO: fixme
    tsc2100_regread(TSC2100_REG_AAP, &val);
    if (val & TSC2100_AAP_SIDETONE) {
        uctrl->value.integer.value[0] = 1;
    } else {
        uctrl->value.integer.value[0] = 0;
    }
    #endif
    return 0;
}


static int snd_imx21_mixer_sidetone_put(snd_kcontrol_t *kcontrol,
                                snd_ctl_elem_value_t *uctrl)
{
    printk( "%s(): called\n", __FUNCTION__ );
    #if 1
    return 0;
    #else
    imx21_chip_t *chip = snd_kcontrol_chip(kcontrol);
    int changed;
    u16 val;

    // TODO: fixme
    tsc2100_regread(TSC2100_REG_AAP, &val);
    if ((val & TSC2100_AAP_SIDETONE) && uctrl->value.integer.value[0]) {
        changed = 0;
    } else {
        changed = 1;
    }

    if (uctrl->value.integer.value[0]) {
        val |= TSC2100_AAP_SIDETONE;
    } else {
        val &= ~TSC2100_AAP_SIDETONE;
    }
    
    tsc2100_regwrite(TSC2100_REG_AAP, val);
    return changed;
    #endif
}


static int snd_imx21_mixer_bypass_info(snd_kcontrol_t *kcontrol,
                                        snd_ctl_elem_info_t *uinfo)
{
    printk( "%s(): called\n", __FUNCTION__ );
    uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
    uinfo->count = 1;
    uinfo->value.integer.min = 0;
    uinfo->value.integer.max = 1;
    return 0;
}


static int snd_imx21_mixer_bypass_get(snd_kcontrol_t *kcontrol,
                                       snd_ctl_elem_value_t *uctrl)
{
    printk( "%s(): called\n", __FUNCTION__ );
    #if 0
    imx21_chip_t *chip = snd_kcontrol_chip(kcontrol);
    u16 val;

    // TODO: fixme
    tsc2100_regread(TSC2100_REG_AAP, &val);
    if (val & TSC2100_AAP_BYPASS) {
        uctrl->value.integer.value[0] = 1;
    } else {
        uctrl->value.integer.value[0] = 0;
    }
    #endif
    return 0;
}


static int snd_imx21_mixer_bypass_put(snd_kcontrol_t *kcontrol,
                                snd_ctl_elem_value_t *uctrl)

{
    printk( "%s(): called\n", __FUNCTION__ );
    #if 1
    return 0;
    #else
    imx21_chip_t *chip = snd_kcontrol_chip(kcontrol);
    int changed;
    u16 val;

    // TODO: fixme
    tsc2100_regread(TSC2100_REG_AAP, &val);
    if ((val & TSC21001_AAP_BYPASS) && uctrl->value.integer.value[0]) {
        changed = 0;
    } else {
        changed = 1;
    }

    if (uctrl->value.integer.value[0]) {
        val |= TSC2100_AAP_BYPASS;
    } else {
        val &= ~TSC2100_AAP_BYPASS;
    }
    
    tsc2100_regwrite(TSC2100_REG_AAP, val);
    return changed;
    #endif
}


static snd_kcontrol_new_t imx21_mixer_controls[] = {
    {
        .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
        .name  = "Master Playback Volume",
        .info  = snd_imx21_mixer_mvol_info,
        .get   = snd_imx21_mixer_mvol_get,
        .put   = snd_imx21_mixer_mvol_put,
    }, {
        .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
        .name  = "Input Select Switch",
        .info  = snd_imx21_mixer_insel_info,
        .get   = snd_imx21_mixer_insel_get,
        .put   = snd_imx21_mixer_insel_put,
    }, {
        .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
        .name  = "Mic Boost",
        .info  = snd_imx21_mixer_micboost_info,
        .get   = snd_imx21_mixer_micboost_get,
        .put   = snd_imx21_mixer_micboost_put,
    }, {
        .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
        .name  = "Sidetone Switch",
        .info  = snd_imx21_mixer_sidetone_info,
        .get   = snd_imx21_mixer_sidetone_get,
        .put   = snd_imx21_mixer_sidetone_put,
    }, {
        .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
        .name  = "Bypass Switch",
        .info  = snd_imx21_mixer_bypass_info,
        .get   = snd_imx21_mixer_bypass_get,
        .put   = snd_imx21_mixer_bypass_put,
    },
};
    
    
static int __init snd_imx21_mixer_new(imx21_chip_t *chip)
{
    snd_card_t *card = chip->card;
    int rc;
    int i;

    printk( "%s(): called\n", __FUNCTION__ );
    rc = snd_device_new(card, SNDRV_DEV_CODEC, chip, &g_mixer_ops);

    for (i = 0; i < ARRAY_SIZE(imx21_mixer_controls); i++) {
        rc = snd_ctl_add(card, snd_ctl_new1(&imx21_mixer_controls[i], chip));
        if (rc < 0)
            return rc;
    }
    return rc;
}


static void imx21_dma_play_isr(int irq, void *data, struct pt_regs *regs)
{
    imx21_chip_t *chip = (imx21_chip_t *)data;
    snd_pcm_substream_t *substream = chip->p_substream;
    snd_pcm_runtime_t *runtime = substream->runtime;
    u32 cntr;

    if (DBTOSR != 0) printk("DMA Error: DBTOSR = 0x%x\n", DBTOSR);
    if (DRTOSR != 0) printk("DMA Error: DRTOSR = 0x%x\n", DRTOSR);
    if (DSESR != 0)  printk("DMA Error: DSESR = 0x%x\n", DSESR);
    if (DBOSR != 0)  printk("DMA Error: DBOSR = 0x%x\n", DBOSR);
    
    cntr = CNTR(chip->p_dma);
    //printk( "[%d.%s(): cntr=%x, chip->p_pos=%x]\n",  
    //        jiffies_to_usecs(jiffies-prev_jiffy), 
    //        __FUNCTION__, cntr, chip->p_pos );
    prev_jiffy = jiffies;
    imx21_dma_dump(chip);
    chip->p_pos += (cntr);

    setup_dma_xfer(chip->p_dma, 
                   runtime->dma_addr + (chip->p_period * chip->p_per_size),
                   (u32)&SSI1_STX0_PHYS,
                   chip->p_per_size);
    CCR(chip->p_dma) |= CCR_RPT;
    chip->p_period = (chip->p_period + 1) & 0x1;

    snd_pcm_period_elapsed(substream);
}


static void imx21_dma_capture_isr(int irq, void *data, struct pt_regs *regs)
{
    imx21_chip_t *chip = (imx21_chip_t *)data;
    snd_pcm_substream_t *substream = chip->c_substream;
    snd_pcm_runtime_t *runtime = substream->runtime;
    u32 cntr;

    printk( "%s(): called\n", __FUNCTION__ );
    if (DBTOSR != 0) printk("DMA Error: DBTOSR = 0x%x\n", DBTOSR);
    if (DRTOSR != 0) printk("DMA Error: DRTOSR = 0x%x\n", DRTOSR);
    if (DSESR != 0)  printk("DMA Error: DSESR = 0x%x\n", DSESR);
    if (DBOSR != 0)  printk("DMA Error: DBOSR = 0x%x\n", DBOSR);
    
    cntr = CNTR(chip->c_dma);
    chip->c_pos += (cntr);

    setup_dma_xfer(chip->c_dma, 
                   (u32)&SSI1_SRX0_PHYS,
                   runtime->dma_addr + (chip->c_period * chip->c_per_size),
                   chip->c_per_size);
    CCR(chip->c_dma) |= CCR_RPT;
    chip->c_period = (chip->c_period + 1) & 0x1;

    snd_pcm_period_elapsed(substream);
}


static int __init imx21_tsc2100_init(void)
{
    int rc = -ENODEV;
    snd_card_t *card;
    imx21_chip_t *chip;

    /* Register the sound card */
    card = snd_card_new(-1, id, THIS_MODULE, sizeof(imx21_chip_t));
    if (card == NULL) {
        return -ENOMEM;
    }

    chip = (imx21_chip_t *)card->private_data;
    chip->card = card;

    /*
     * Configure the Audio Mux:
     *   Use SSI1 controller - Host port 2 - port #1
     *   RX: Use SS1 pins - periph port 1 - port #3
     *   TX: Use SS2 pins - periph port 2 - port #4
     *
     *   Host port:
     *     connect RXCLK/TXCLK, RXFS/TXFS - synchronous mode
     *     clk and fs are outputs to SSI
     *
     *   Peripheral port:
     *     connect RXCLK/TXCLK, RXFS/TXFS - synchronous mode
     *     clk and fs are inputs from the pins
     */
    AUDMUX_HPCR1 = (
        AUDMUX_HPCR_TFSDIR    |
        AUDMUX_HPCR_TCLKDIR   |
        AUDMUX_HPCR_RFSDIR    |
        AUDMUX_HPCR_RCLKDIR   |
        AUDMUX_HPCR_TFCSEL(4) |   /* Peripheral port 2 */
        AUDMUX_HPCR_RFCSEL(3) |   /* Peripheral port 1 */
        AUDMUX_HPCR_RXDSEL(3)     /* Peripheral port 1 */
        );

    AUDMUX_PPCR1 = (
        AUDMUX_PPCR_TFCSEL(0) |   /* Host port 1 */
        AUDMUX_PPCR_RFCSEL(0) |   /* Host port 1 */
        0);

    AUDMUX_PPCR2 = (
        AUDMUX_PPCR_TFCSEL(0) |   /* Host port 1 */
        AUDMUX_PPCR_RFCSEL(0) |   /* Host port 1 */
        0);

    gpio_init();
    spi_init();

    /* Enable the clock to SSI1 */
    PCCR0 |= (PCCR0_SSI1_BAUD_EN | PCCR0_SSI1_EN);

    if (imx_dma_request_by_prio(
            &chip->p_dma, "ALSA-PLAYBACK", DMA_PRIO_HIGH) < 0) {
        goto init_err0;
    }
    imx_dma_setup_handlers(chip->p_dma, imx21_dma_play_isr, NULL, chip);

    if (imx_dma_request_by_prio(
            &chip->c_dma, "ALSA-CAPTURE", DMA_PRIO_HIGH) < 0) {
        goto init_err0;
    }
    imx_dma_setup_handlers(chip->c_dma, imx21_dma_capture_isr, NULL, chip);

    SSI1_SOR = SSI_SOR_INIT;
    udelay(1000);
    SSI1_SOR = 0;

    SSI1_SCR  = (SSI_SCR_I2S_MODE_SLAVE |
                 SSI_SCR_SYN | 
                 SSI_SCR_SSIEN);

    SSI1_STCR  = (SSI_STCR_TXBIT0 |
                  SSI_STCR_TFEN0  |
                  SSI_STCR_TSCKP  |
                  SSI_STCR_TFSI   |
                  SSI_STCR_TEFS );

    SSI1_SRCR  = (SSI_SRCR_RXBIT0 |
                  SSI_SRCR_RFEN0  |
                  SSI_SRCR_RSCKP  |
                  SSI_SRCR_RFSI   |
                  SSI_SRCR_REFS );

    SSI1_SFCSR = (SSI1_SFCSR & ~0xff) | 
                 (SSI_SFCSR_RFWM0(6)  |
                  SSI_SFCSR_TFWM0(4));

    if ((rc = snd_imx21_mixer_new(chip)) != 0) goto init_err1;
    if ((rc = snd_imx21_pcm_new(chip)) != 0)   goto init_err1;
    if ((rc = snd_card_register(card)) != 0)   goto init_err1;

    pll_fsref_48000(0);
                                    
    printk(KERN_INFO "Chumby i.MX21 TSC2100 audio support initialized\n");
    return rc;

 init_err1:
    printk( "%s(): failed init_err1\n", __FUNCTION__ );
    imx_dma_free(chip->p_dma);
 init_err0:
    printk( "%s(): failed init_err0, rc=%d\n", __FUNCTION__, rc );
    snd_card_free(card);
    return rc;
}


static void __exit imx21_tsc2100_exit(void)
{
    printk( "%s(): called\n", __FUNCTION__ );
    return;
}


module_init(imx21_tsc2100_init);
module_exit(imx21_tsc2100_exit);
