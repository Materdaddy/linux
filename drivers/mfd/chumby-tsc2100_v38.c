/*
 *  Driver for Chumby (iMX21) TSC2100, both ALSA audio and TouchScreen
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
 *  TODO:
 *     handle power management
 */

#include <linux/config.h>
#include <sound/driver.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/ioctl.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/dma.h>
#include <asm/arch/imx-dma.h>

#ifdef MAKE_L22 // needed to compile with linux 2.6.22+ compatibility
#include <sound/typedefs.h>
#endif
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/control.h>
#include <sound/initval.h>

#include "chumby-tsc2100.h"


//#define TS_DEBUG
#ifdef TS_DEBUG
# define TSDBG(fmt, args...) printk( KERN_ERR fmt, args )
#else
# define TSDBG(stuff...) do {} while(0)
#endif

//#define DEBUG
#ifdef DEBUG
#  define DBG(fmt, args...) \
            dbug_printk("%lu-%s().%d: " fmt, jiffies, __FUNCTION__, __LINE__, ## args)
//#  define DEBUG_ALL
#else
#  define DBG(stuff...) do {} while(0)
#endif
#define ERR(fmt, args...)   \
            printk(KERN_ERR "ERROR!!! %s/%s().%d: " fmt, \
                   __FILE__, __FUNCTION__, __LINE__, ## args)

#define ADCMODE(mode)  (TSC2100_ADC_DEFAULT | TSC2100_ADC_ADMODE(mode))

/***********************************************************************
 * Module stuff
 ***********************************************************************/
static char *id = NULL;
static int timeout_workaround = 1;
static int timeout_regdump = 0;
module_param(id, charp, 0444);
module_param(timeout_workaround, int, 1);
module_param(timeout_regdump, int, 0 );

MODULE_AUTHOR("Chumby <ghutchins@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Chumby iMX21 TI-TSC2100 Driver (ALSA Audio & TouchScreen)");
MODULE_PARM_DESC(id, "Chumby iMX21/TSC2100");


/***********************************************************************
 * Constants
 ***********************************************************************/
#define BUFFER_BYTES_MAX    (128 * 1024) // big mofo buffer (64K is probably more than enough)
#define PERIOD_BYTES_MIN    (16 * 1024)  // based on Chumby iMX21 system latency
#define PERIODS_MIN         4            // based on ALSA ring buffer math (must be at least 3)
#define PERIODS_MAX         (BUFFER_BYTES_MAX / PERIOD_BYTES_MIN)
#define PERIOD_BYTES_MAX    (BUFFER_BYTES_MAX / PERIODS_MIN)


// TouchScreen Constants
#define X_AXIS_MAX		    3830
#define X_AXIS_MIN		    150
#define Y_AXIS_MAX		    3830
#define Y_AXIS_MIN		    190
#define PRESSURE_MIN	    0
#define PRESSURE_MAX	    20000

#define PENIRQ              IRQ_GPIOE(19)

#define TSC2100_ADC_DEFAULT (TSC2100_ADC_RES(TSC2100_ADC_RES_12BITP)   | \
                             TSC2100_ADC_AVG(TSC2100_ADC_4AVG)         | \
                             TSC2100_ADC_CL(TSC2100_ADC_CL_1MHZ_12BIT) | \
                             TSC2100_ADC_PV(TSC2100_ADC_PV_500us)      | \
                             TSC2100_ADC_AVGFILT_MEAN)


/***********************************************************************
 * Data structures
 ***********************************************************************/
typedef struct {
    snd_card_t *card;
    snd_pcm_t *pcm;

    //playback
    imx_dmach_t p_dma;
    snd_pcm_substream_t *p_substream;
    u32 p_pos;
    u32 p_buf_size;
    u32 p_per_size;
    int p_period;

    // capture
    imx_dmach_t c_dma;
    snd_pcm_substream_t *c_substream;
    u32 c_pos;
    u32 c_buf_size;
    u32 c_per_size;
    int c_period;
} tsc2100_chip_t;



/***********************************************************************
 * Global Variables
 ***********************************************************************/
u16 reg_adc         = TSC2100_ADC_DEFAULT | TSC2100_ADC_ADMODE(0);
u16 reg_refctl      = 0x11;
u16 reg_control1    = 0 << 14  |    // ADC high pass filter disabled
                      2 << 12  |    // ADC input mux single-ended AUX
                      0 << 10  |    // codec 16-bit word length
                      0 << 8   |    // I2S mode
                      0 << 3   |    // DAC sampling rate Fsref/N (sort of)
                      0 << 0;       // ADC sampling rate Fsref/N (sort of)
u16 reg_audioadc    = 0x00 << 15 |  // ADC Channel not muted
                      0x7F << 8  |  // ADC PGA
                      0x03 << 5  |  // AGC target level -12dB
                      0x04 << 1  |  // AGC Time Constant -- attack 8ms, decay 200ms
                      0x00 << 0;    // AGC is off ( 0x01 << 0 to enable )
u16 reg_audiodac    = 0x00 << 15 |  // DAC left channel not muted
                      0x00 << 8  |  // DAC left channel volume control -30dB
                      0x00 << 7  |  // DAC right channel not muted
                      0x00 << 0;    // DAC right channel volume control -30dB
u16 reg_sidetone    = 0x8080;       // sidetone default muted
u16 reg_control2    = 0;
u16 reg_powercon    = 0 << 15  |    // codec powered up
                      1 << 13  |    // analog sidetone powered down
                      1 << 12  |    // output driver in high power mode
                      1 << 11  |    // sidetone power down complete (RO?)
                      0 << 10  |    // power up the DAC
                      0 << 9   |    // power up the ADC
                      1 << 8   |    // power down the VGND amp (?)
                      0 << 5   |    // ADWS pin acts as ADC hw powerdown
                      0 << 4;       // Vbias at 2.5V
u16 reg_control3    = 0 << 13  |    // Fsref == 48.0 kHz
                      0 << 12  |    // continuous data transfer mode
                      1 << 11  |    // TSC2100 is master codec
                      2 << 9;       // DAC max output signal swing 2.402 V
u16 reg_pll1        = 1 << 15  |    // enable PLL
                      1 << 8   |    // P value is 1 for 48.0 kHz
                      8 << 2;       // J value is 8 for 48.0 kHz
u16 reg_pll2        = 1920<<2;      // D value is 1920 for 48.0 kHz
u16 reg_control4    = 0;
u16 reg_control5    = 0xFE00;
int poll_interval   = 1;            // in seconds
int max_samples     = 2;            // max # of samples to average (low pass filter)
int dc_power_min      = 170;        // chumby unplugged if less than this value 
int battery_power_min = 110;        // installed min battery measurement
int battery_power_max = 175;        // installed max battery measurment
static int sigpwr_next = 10;        // # of secs to wait before next SIGPWR sig  // note change! this was 0, changed for debug to 10
static int sigpwr_holdoff = 30;     // # of secs to wait before generating
                                    // another SIGPWR signal
static int sigpwr_pid = 0;          // PID to send SIGPWR signal to when line
                                    // power is lost (not battery power)

module_param(reg_adc,      ushort, S_IRUGO|S_IWUSR);
module_param(reg_refctl,   ushort, S_IRUGO|S_IWUSR);
module_param(reg_control1, ushort, S_IRUGO|S_IWUSR);
module_param(reg_audioadc, ushort, S_IRUGO|S_IWUSR);
module_param(reg_audiodac, ushort, S_IRUGO|S_IWUSR);
module_param(reg_sidetone, ushort, S_IRUGO|S_IWUSR);
module_param(reg_control2, ushort, S_IRUGO|S_IWUSR);
module_param(reg_powercon, ushort, S_IRUGO|S_IWUSR);
module_param(reg_control3, ushort, S_IRUGO|S_IWUSR);
module_param(reg_pll1,     ushort, S_IRUGO|S_IWUSR);
module_param(reg_pll2,     ushort, S_IRUGO|S_IWUSR);
module_param(reg_control4, ushort, S_IRUGO|S_IWUSR);
module_param(reg_control5, ushort, S_IRUGO|S_IWUSR);
module_param(poll_interval, int, S_IRUGO|S_IWUSR);
module_param(max_samples, int, S_IRUGO|S_IWUSR);
module_param(dc_power_min, int, S_IRUGO|S_IWUSR);
module_param(battery_power_min, int, S_IRUGO|S_IWUSR);
module_param(battery_power_max, int, S_IRUGO|S_IWUSR);
module_param(sigpwr_holdoff, int, S_IRUGO|S_IWUSR);

static int playback_buffer_bytes_max = BUFFER_BYTES_MAX;
static int playback_period_bytes_min = PERIOD_BYTES_MIN;
static int playback_period_bytes_max = PERIOD_BYTES_MAX;
static int playback_periods_min = PERIODS_MIN;
static int playback_periods_max = PERIODS_MAX;
module_param(playback_buffer_bytes_max, int, S_IRUGO|S_IWUSR);
module_param(playback_period_bytes_min, int, S_IRUGO|S_IWUSR);
module_param(playback_period_bytes_max, int, S_IRUGO|S_IWUSR);
module_param(playback_periods_min, int, S_IRUGO|S_IWUSR);
module_param(playback_periods_max, int, S_IRUGO|S_IWUSR);


static snd_card_t           *tsc2100_sndcard = NULL;
struct tsc2100_data         *tsc2100_devdata = NULL;


static snd_device_ops_t g_mixer_ops = {
    .dev_free = NULL,
};



static unsigned int rates[] = {8000, 11025, 16000, 22050, 32000, 44100, 48000};
static snd_pcm_hw_constraint_list_t hw_constraints_rates = {
    .count = ARRAY_SIZE(rates),
    .list  = rates,
    .mask  = 0,
};


static snd_pcm_hardware_t snd_tsc2100_playback_hw = {
    .info = (SNDRV_PCM_INFO_INTERLEAVED    |
             SNDRV_PCM_INFO_BLOCK_TRANSFER |
             SNDRV_PCM_INFO_MMAP           |
             SNDRV_PCM_INFO_MMAP_VALID),
    .formats = SNDRV_PCM_FMTBIT_S16_LE,
    .rates = (SNDRV_PCM_RATE_8000  |
              SNDRV_PCM_RATE_11025 |
              SNDRV_PCM_RATE_16000 |
              SNDRV_PCM_RATE_22050 |
              SNDRV_PCM_RATE_32000 |
              SNDRV_PCM_RATE_44100 |
              SNDRV_PCM_RATE_48000),
    .rate_min = 8000,
    .rate_max = 48000,
    .channels_min = 1,
    .channels_max = 2,
    .buffer_bytes_max = BUFFER_BYTES_MAX,
    .period_bytes_min = PERIOD_BYTES_MIN,
    .period_bytes_max = PERIOD_BYTES_MAX,
    .periods_min = PERIODS_MIN,
    .periods_max = PERIODS_MAX,
};

static snd_pcm_hardware_t snd_tsc2100_capture_hw = {
    .info = (SNDRV_PCM_INFO_INTERLEAVED    |
             SNDRV_PCM_INFO_BLOCK_TRANSFER |
             SNDRV_PCM_INFO_MMAP           |
             SNDRV_PCM_INFO_MMAP_VALID),
    .formats = SNDRV_PCM_FMTBIT_S16_LE,
    .rates = (SNDRV_PCM_RATE_8000  |
              SNDRV_PCM_RATE_11025 |
              SNDRV_PCM_RATE_16000 |
              SNDRV_PCM_RATE_22050 |
              SNDRV_PCM_RATE_32000 |
              SNDRV_PCM_RATE_44100 |
              SNDRV_PCM_RATE_48000),
    .rate_min = 8000,
    .rate_max = 48000,
    .channels_min = 1,
    .channels_max = 2,
    .buffer_bytes_max = BUFFER_BYTES_MAX,
    .period_bytes_min = PERIOD_BYTES_MIN,
    .period_bytes_max = PERIOD_BYTES_MAX,
    .periods_min = PERIODS_MIN,
    .periods_max = PERIODS_MAX,
};


static void gpio_init( void )
{
    imx_gpio_write( 30 | GPIO_PORTC, 0 );  // turn off battery voltage gate
    imx_gpio_mode( 30 | GPIO_PORTC | GPIO_GPIO | GPIO_OUT );
    imx_gpio_mode( 28 | GPIO_PORTC | GPIO_GPIO | GPIO_OUT );
    imx_gpio_write( 28 | GPIO_PORTC, 0 );  // turn off line voltage gate

    imx_gpio_mode( 18 | GPIO_PORTE | GPIO_AF | GPIO_IN );
    imx_gpio_mode( 21 | GPIO_PORTE | GPIO_AF | GPIO_OUT );
    imx_gpio_mode( 22 | GPIO_PORTE | GPIO_AF | GPIO_OUT );
    imx_gpio_mode( 23 | GPIO_PORTE | GPIO_AF | GPIO_OUT );
    imx_gpio_mode( 19 | GPIO_PORTE | GPIO_PF | GPIO_IN | GPIO_PUEN | GPIO_GPIO );

    imx_gpio_mode( PC23_PF_SSI1_CLK | GPIO_PUEN );
    imx_gpio_mode( PC22_PF_SSI1_TX | GPIO_PUEN  );
    imx_gpio_mode( PC21_PF_SSI1_RX | GPIO_PUEN  );
    imx_gpio_mode( PC20_PF_SSI1_FS | GPIO_PUEN  );
    imx_gpio_mode( PC27_PF_SSI2_CLK | GPIO_PUEN  );
    imx_gpio_mode( PC26_PF_SSI2_TX | GPIO_PUEN  );
    imx_gpio_mode( PC25_PF_SSI2_RX | GPIO_PUEN  );
    imx_gpio_mode( PC24_PF_SSI2_FS | GPIO_PUEN  );

    #ifdef ENABLE_SPEAKER_KLUDGE_UNTIL_OTHER_DRIVER_PORTED
    imx_gpio_mode( GPIO_PORTKP | (4 + 8) | GPIO_OUT );
    disable_speaker();
    #endif
}


static void spi_init( void )
{
    unsigned int rate;
    unsigned int rate_div;

    SSP_RESET_REG(TSC2100_SPIDEV) = 1;
    udelay(200);

    rate = ( imx_get_perclk2() / 10000000 ) + 1;
    rate_div = SSP_RATE_DIV4;
    if ( rate > 4 )   rate_div = SSP_RATE_DIV6;
    if ( rate > 6 )   rate_div = SSP_RATE_DIV8;
    if ( rate > 8 )   rate_div = SSP_RATE_DIV12;
    if ( rate > 12 )  rate_div = SSP_RATE_DIV16;
    if ( rate > 16 )  rate_div = SSP_RATE_DIV256;

    PCCR1 |= PCCR1_CSPI3_EN; // enable PERCLK to CSPI3
    SSP_CTRL_REG(TSC2100_SPIDEV) = (
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
    SSP_CTRL_REG(TSC2100_SPIDEV) = SSP_DISABLE;
    PCCR1 &= ~PCCR1_CSPI3_EN; // disable PERCLK to CSPI3
}


// the P/J/D values are based on a 12Mhz(old hw) or 16Mhz(new hw) MCLK
// if N == 0 -> 48000 sample clock
// if N == 7 --> 8000 sample clock

static void pll_fsref_48000( int D, int A )
{
    u16 val = tsc2100_devdata->reg.control1 & ~0x3F;
    val |= ( ( D & 7 ) << 3 ) | ( A & 7 );
    tsc2100_devdata->reg.control1 = val;
    tsc2100_devdata->reg.control3 &= ~( 1 << 13 );
    tsc2100_devdata->dac_sample_rate = D;
    tsc2100_devdata->adc_sample_rate = A;
    tsc2100_regwrite(TSC2100_REG_CONTROL1, tsc2100_devdata->reg.control1);
    tsc2100_regwrite(TSC2100_REG_CONTROL3, tsc2100_devdata->reg.control3);

    // check to see if external crystal is being used on CPU
    if( CSCR & CSCR_MCU_SEL ) { // detect clock version by looking at where CPU gets its clock
      // we are in 16 MHz land (hardware version 1.6 and higher)
      // K/P = 6.144
      // P = 1
      // J = 6
      // D = 1440
      tsc2100_regwrite(TSC2100_REG_PLL1,
		       1 << 15 |  // enable PLL
		       1 << 8  |  // P value is 1 for 48.0 kHz
		       6 << 2);   // J value is 6 for 48.0 kHz
      tsc2100_regwrite(TSC2100_REG_PLL2,
		       1440<<2);  // D value is 1440 for 48.0 kHz
    } else {
      // we are in 12 MHz land (hardware version 1.5 and less)
      tsc2100_regwrite(TSC2100_REG_PLL1,
		       1 << 15 |  // enable PLL
		       1 << 8  |  // P value is 1 for 48.0 kHz
		       8 << 2);   // J value is 8 for 48.0 kHz
      tsc2100_regwrite(TSC2100_REG_PLL2,
		       1920<<2);  // D value is 1920 for 48.0 kHz
    }
}


static void pll_fsref_44100( int D, int A )
{
    u16 val = tsc2100_devdata->reg.control1 & ~0x3F;
    val |= ( ( D & 7 ) << 3 ) | ( A & 7 );
    tsc2100_devdata->reg.control1 = val;
    tsc2100_devdata->reg.control3 |= ( 1 << 13 );
    tsc2100_devdata->dac_sample_rate = D;
    tsc2100_devdata->adc_sample_rate = A;
    tsc2100_regwrite(TSC2100_REG_CONTROL1, tsc2100_devdata->reg.control1);
    tsc2100_regwrite(TSC2100_REG_CONTROL3, tsc2100_devdata->reg.control3);

    if( CSCR & CSCR_MCU_SEL ) { // detect clock version by looking at where CPU gets its clock
      // we are in 16 MHz land (hardware version 1.6 and higher)
      // K/P = 5.6448
      // P = 1
      // J = 5
      // D = 6448
      tsc2100_regwrite(TSC2100_REG_PLL1,
		       1 << 15 |  // enable PLL
		       1 << 8  |  // P value is 1 for 44.1 kHz
		       5 << 2);   // J value is 5 for 44.1 kHz
      tsc2100_regwrite(TSC2100_REG_PLL2,
		       6448<<2);  // D value is 6448 for 44.1 kHz
    } else {
      // we are in 12 MHz land (hardware version 1.5 and less)
      tsc2100_regwrite(TSC2100_REG_PLL1,
		       1 << 15 |  // enable PLL
		       1 << 8  |  // P value is 1 for 44.1 kHz
		       7 << 2);   // J value is 7 for 44.1 kHz
      tsc2100_regwrite(TSC2100_REG_PLL2,
		       5264<<2);  // D value is 5264 for 44.1 kHz
    }
}


/************************************************************
 * PCM Functions
 ************************************************************/
static int snd_tsc2100_playback_open(snd_pcm_substream_t *substream)
{
    int rc;
    tsc2100_chip_t *chip = snd_pcm_substream_chip(substream);
    snd_pcm_runtime_t *runtime = substream->runtime;

    snd_tsc2100_playback_hw.buffer_bytes_max = playback_buffer_bytes_max;
    snd_tsc2100_playback_hw.period_bytes_min = playback_period_bytes_min;
    snd_tsc2100_playback_hw.period_bytes_max = playback_period_bytes_max;
    snd_tsc2100_playback_hw.periods_min      = playback_periods_min;
    snd_tsc2100_playback_hw.periods_max      = playback_periods_max;
    printk( KERN_DEBUG
            "%s/%s(): playback buffer_bytes_max=0x%x, period_bytes_min=0x%x,"
            " period_bytes_max=0x%x, periods_min=0x%x, periods_max=0x%x\n",
            __FILE__, __FUNCTION__, playback_buffer_bytes_max, 
            playback_period_bytes_min, playback_period_bytes_max, 
            playback_periods_min, playback_periods_max);

    runtime->hw = snd_tsc2100_playback_hw;
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


static int snd_tsc2100_playback_close(snd_pcm_substream_t *substream)
{
    tsc2100_chip_t *chip = snd_pcm_substream_chip(substream);
    chip->p_substream = NULL;
    return 0;
}


static int snd_tsc2100_pcm_hw_params(snd_pcm_substream_t *substream,
                                    snd_pcm_hw_params_t *hw_params)
{
    return snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(hw_params));
}


static int snd_tsc2100_pcm_hw_free(snd_pcm_substream_t *substream)
{
    return snd_pcm_lib_free_pages(substream);
}


static inline void setup_dma_xfer(int chan, u32 src, u32 dest, int count)
{
    SAR(chan) = src;
    DAR(chan) = dest;
    CNTR(chan) = count;
}


static int snd_tsc2100_pcm_playback_prepare(snd_pcm_substream_t *substream)
{
    tsc2100_chip_t *chip = snd_pcm_substream_chip(substream);
    snd_pcm_runtime_t *runtime = substream->runtime;

    tsc2100_regwrite(TSC2100_REG_AUDIODAC, tsc2100_devdata->reg.audiodac);
    tsc2100_regwrite(TSC2100_REG_POWERCON, tsc2100_devdata->reg.powercon);

    switch(runtime->rate) {
    case 8000:
        /* 16 bit, 8KHz, 24 samples/frame */
        SSI1_STCCR = (SSI_STCCR_WL(16) | SSI_STCCR_DC(23) | SSI_STCCR_PM(0));
        SSI1_SRCCR = (SSI_SRCCR_WL(16) | SSI_SRCCR_DC(23) | SSI_SRCCR_PM(0));
        SSI1_STMSK = ~((1 << 0) | (1 << 12));
        SSI1_SRMSK = ~((1 << 0) | (1 << 12));
        pll_fsref_48000(7, tsc2100_devdata->adc_sample_rate);
        break;

    case 11025:
        /* 16 bit, 44.1KHz, 16 samples/frame */
        SSI1_STCCR = (SSI_STCCR_WL(16) | SSI_STCCR_DC(15) | SSI_STCCR_PM(0));
        SSI1_SRCCR = (SSI_SRCCR_WL(16) | SSI_SRCCR_DC(15) | SSI_SRCCR_PM(0));
        SSI1_STMSK = ~((1 << 0) | (1 << 2));
        SSI1_SRMSK = ~((1 << 0) | (1 << 2));
        pll_fsref_44100(4, tsc2100_devdata->adc_sample_rate);
        break;

    case 16000:
        /* 16 bit, 48KHz, 4 samples/frame */
        SSI1_STCCR = (SSI_STCCR_WL(16) | SSI_STCCR_DC(11) | SSI_STCCR_PM(0));
        SSI1_SRCCR = (SSI_SRCCR_WL(16) | SSI_SRCCR_DC(11) | SSI_SRCCR_PM(0));
        SSI1_STMSK = ~((1 << 0) | (1 << 2));
        SSI1_SRMSK = ~((1 << 0) | (1 << 2));
        pll_fsref_48000(3, tsc2100_devdata->adc_sample_rate);
        break;

    case 22050:
        /* 16 bit, 44.1KHz, 8 samples/frame */
        SSI1_STCCR = (SSI_STCCR_WL(16) | SSI_STCCR_DC(7) | SSI_STCCR_PM(0));
        SSI1_SRCCR = (SSI_SRCCR_WL(16) | SSI_SRCCR_DC(7) | SSI_SRCCR_PM(0));
        SSI1_STMSK = ~((1 << 0) | (1 << 2));
        SSI1_SRMSK = ~((1 << 0) | (1 << 2));
        pll_fsref_44100(2, tsc2100_devdata->adc_sample_rate);
        break;

    case 32000:
        /* 16 bit, 48KHz, 4 samples/frame */
        SSI1_STCCR = (SSI_STCCR_WL(16) | SSI_STCCR_DC(5) | SSI_STCCR_PM(0));
        SSI1_SRCCR = (SSI_SRCCR_WL(16) | SSI_SRCCR_DC(5) | SSI_SRCCR_PM(0));
        SSI1_STMSK = ~((1 << 0) | (1 << 2));
        SSI1_SRMSK = ~((1 << 0) | (1 << 2));
        pll_fsref_48000(1, tsc2100_devdata->adc_sample_rate);
        break;

    case 44100:
        /* 16 bit, 44.1KHz, 4 samples/frame */
        SSI1_STCCR = (SSI_STCCR_WL(16) | SSI_STCCR_DC(3) | SSI_STCCR_PM(0));
        SSI1_SRCCR = (SSI_SRCCR_WL(16) | SSI_SRCCR_DC(3) | SSI_SRCCR_PM(0));
        SSI1_STMSK = ~((1 << 0) | (1 << 2));
        SSI1_SRMSK = ~((1 << 0) | (1 << 2));
        pll_fsref_44100(0, tsc2100_devdata->adc_sample_rate);
        break;

    case 48000:
        /* 16 bit, 48KHz, 4 samples/frame */
        SSI1_STCCR = (SSI_STCCR_WL(16) | SSI_STCCR_DC(3) | SSI_STCCR_PM(0));
        SSI1_SRCCR = (SSI_SRCCR_WL(16) | SSI_SRCCR_DC(3) | SSI_SRCCR_PM(0));
        SSI1_STMSK = ~((1 << 0) | (1 << 2));
        SSI1_SRMSK = ~((1 << 0) | (1 << 2));
        pll_fsref_48000(0, tsc2100_devdata->adc_sample_rate);
        break;

    default:
        printk("%s: Can't handle rate of %d\n", __FUNCTION__, runtime->rate);
        return -ENODEV;
    }

   /* Get physical address of data buffer */
    runtime->dma_addr = __pa(runtime->dma_area);

    chip->p_pos = 0;
    chip->p_period = 0;
    chip->p_per_size = snd_pcm_lib_period_bytes(substream);
    chip->p_buf_size = snd_pcm_lib_buffer_bytes(substream);
    DBG("p_per_size=0x%x, p_buf_size=0x%x, periods=0x%x, dma_addr=0x%x\n", 
        chip->p_per_size, chip->p_buf_size, runtime->periods, runtime->dma_addr);
    printk( KERN_DEBUG
            "%s/%s(): p_per_size=0x%x, p_buf_size=0x%x, periods=0x%x\n",
            __FILE__, __FUNCTION__, 
            chip->p_per_size, chip->p_buf_size, runtime->periods );

    CCR(chip->p_dma) = (CCR_DMOD_FIFO   |
                        CCR_SMOD_LINEAR |
                        CCR_DSIZ_16     |
                        CCR_SSIZ_32     |
                        CCR_REN);

    RSSR(chip->p_dma) = DMA_REQ_SSI1_TX0;
    BLR(chip->p_dma) = 4;   /* TODO: is this correct? */
    BUCR(chip->p_dma) = 0;

    return 0;
}


static int snd_tsc2100_pcm_playback_trigger(snd_pcm_substream_t *substream, int cmd)
{
    tsc2100_chip_t *chip = snd_pcm_substream_chip(substream);
    snd_pcm_runtime_t *runtime = substream->runtime;

    switch(cmd) {
    case SNDRV_PCM_TRIGGER_START:
        enable_speaker();

        /* If ALSA wants to give us two periods, set them both up  */

        while (CCR(chip->p_dma) & CCR_CEN) {
            printk("TRIGGER_START while DMA is already enabled\n");
        }
        DBG("%d: dma=%lx ack=%x\n", 
            chip->p_period, chip->p_period * chip->p_per_size, chip->p_pos);
        setup_dma_xfer(chip->p_dma,
                       runtime->dma_addr + (chip->p_period * chip->p_per_size),
                       (u32)&SSI1_STX0_PHYS,
                       chip->p_per_size);
        CCR(chip->p_dma) |= CCR_ACRPT;
        chip->p_period = (chip->p_period + 1) % runtime->periods;
        SSI1_SCR |= SSI_SCR_TE;
        SSI1_SIER |= SSI_SIER_TDMAE;
        imx_dma_enable(chip->p_dma);

        if (runtime->periods >= 2) {
            CCR(chip->p_dma) |= CCR_RPT;
            DBG("%d: dma=%lx ack=%x\n", 
                chip->p_period, chip->p_period * chip->p_per_size, chip->p_pos);
            setup_dma_xfer(chip->p_dma,
                           runtime->dma_addr + (chip->p_period * chip->p_per_size),
                           (u32)&SSI1_STX0_PHYS,
                           chip->p_per_size);
            chip->p_period = (chip->p_period + 1) % runtime->periods;
        }
        break;
    case SNDRV_PCM_TRIGGER_STOP:
        disable_speaker();
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


// NOTE: the ALSA library is only interested in period granularity, the 
// only question it is trying to answer with this routine is what period is
// currently being DMAed.  Refer to the following code in the ALSA library
// for details (alsa-lib-1.0.9/src/pcm/pcm_dmix.c:snd_pcm_dmix_sync_area()):
// 
//	slave_bsize = dmix->shmptr->s.buffer_size;
//	slave_hw_ptr = dmix->slave_hw_ptr;
//	/* don't write on the last active period - this area may be cleared
//	 * by the driver during mix operation...
//	 */
//	slave_hw_ptr -= slave_hw_ptr % dmix->shmptr->s.period_size;
//	slave_hw_ptr += slave_bsize;
//	if (dmix->slave_hw_ptr > dmix->slave_appl_ptr)
//		slave_hw_ptr -= dmix->shmptr->s.boundary;
//	if (dmix->slave_appl_ptr + size >= slave_hw_ptr)
//		size = slave_hw_ptr - dmix->slave_appl_ptr;
//	if (! size)
//		return;

static snd_pcm_uframes_t snd_tsc2100_pcm_playback_pointer(
                                        snd_pcm_substream_t *substream)
{
    tsc2100_chip_t *chip = snd_pcm_substream_chip(substream);
    size_t ptr = chip->p_pos % chip->p_buf_size;
    return bytes_to_frames(substream->runtime, ptr);
}


static void tsc2100_dma_play_isr(int irq, void *data, struct pt_regs *regs)
{
    tsc2100_chip_t *chip = (tsc2100_chip_t *)data;
    snd_pcm_substream_t *substream = chip->p_substream;
    snd_pcm_runtime_t *runtime = substream->runtime;

    if (DBTOSR != 0) printk("DMA Error: DBTOSR = 0x%x\n", DBTOSR);
    if (DRTOSR != 0) printk("DMA Error: DRTOSR = 0x%x\n", DRTOSR);
    if (DSESR != 0)  printk("DMA Error: DSESR = 0x%x\n", DSESR);
    if (DBOSR != 0)  printk("DMA Error: DBOSR = 0x%x\n", DBOSR);

    chip->p_pos += chip->p_per_size;

    DBG("%d: dma=%lx ack=%x\n", 
        chip->p_period, chip->p_period * chip->p_per_size, chip->p_pos);
    setup_dma_xfer(chip->p_dma,
                   runtime->dma_addr + (chip->p_period * chip->p_per_size),
                   (u32)&SSI1_STX0_PHYS,
                   chip->p_per_size);
    CCR(chip->p_dma) |= CCR_RPT;
    chip->p_period = (chip->p_period + 1) % runtime->periods;

    snd_pcm_period_elapsed(substream);
}


static int snd_tsc2100_capture_open(snd_pcm_substream_t *substream)
{
    int rc;
    tsc2100_chip_t *chip = snd_pcm_substream_chip(substream);
    snd_pcm_runtime_t *runtime = substream->runtime;

    runtime->hw = snd_tsc2100_capture_hw;
    chip->c_substream = substream;

    rc = snd_pcm_hw_constraint_list(runtime, 0,
                                    SNDRV_PCM_HW_PARAM_RATE,
                                    &hw_constraints_rates);
    if (rc < 0) {
        printk("Error setting rates constraint\n");
        return rc;
    }
    chip->c_pos = 0;
    return 0;
}


static int snd_tsc2100_capture_close(snd_pcm_substream_t *substream)
{
    tsc2100_chip_t *chip = snd_pcm_substream_chip(substream);
    chip->c_substream = NULL;
    return 0;
}


static int snd_tsc2100_pcm_capture_prepare(snd_pcm_substream_t *substream)
{
    tsc2100_chip_t *chip = snd_pcm_substream_chip(substream);
    snd_pcm_runtime_t *runtime = substream->runtime;

    tsc2100_regwrite(TSC2100_REG_AUDIOADC, tsc2100_devdata->reg.audioadc);
    tsc2100_regwrite(TSC2100_REG_POWERCON, tsc2100_devdata->reg.powercon);

    switch(runtime->rate) {
    case 8000:
        /* 16 bit, 8KHz, 24 samples/frame */
        SSI1_STCCR = (SSI_STCCR_WL(16) | SSI_STCCR_DC(23) | SSI_STCCR_PM(0));
        SSI1_SRCCR = (SSI_SRCCR_WL(16) | SSI_SRCCR_DC(23) | SSI_SRCCR_PM(0));
        SSI1_STMSK = ~((1 << 0) | (1 << 12));
        SSI1_SRMSK = ~((1 << 0) | (1 << 12));
        pll_fsref_48000(tsc2100_devdata->dac_sample_rate,7);
        break;

    case 11025:
        /* 16 bit, 44.1KHz, 16 samples/frame */
        SSI1_STCCR = (SSI_STCCR_WL(16) | SSI_STCCR_DC(15) | SSI_STCCR_PM(0));
        SSI1_SRCCR = (SSI_SRCCR_WL(16) | SSI_SRCCR_DC(15) | SSI_SRCCR_PM(0));
        SSI1_STMSK = ~((1 << 0) | (1 << 2));
        SSI1_SRMSK = ~((1 << 0) | (1 << 2));
        pll_fsref_44100(tsc2100_devdata->dac_sample_rate, 4);
        break;

    case 16000:
        /* 16 bit, 48KHz, 4 samples/frame */
        SSI1_STCCR = (SSI_STCCR_WL(16) | SSI_STCCR_DC(11) | SSI_STCCR_PM(0));
        SSI1_SRCCR = (SSI_SRCCR_WL(16) | SSI_SRCCR_DC(11) | SSI_SRCCR_PM(0));
        SSI1_STMSK = ~((1 << 0) | (1 << 2));
        SSI1_SRMSK = ~((1 << 0) | (1 << 2));
        pll_fsref_48000(tsc2100_devdata->dac_sample_rate, 3);
        break;

    case 22050:
        /* 16 bit, 44.1KHz, 8 samples/frame */
        SSI1_STCCR = (SSI_STCCR_WL(16) | SSI_STCCR_DC(7) | SSI_STCCR_PM(0));
        SSI1_SRCCR = (SSI_SRCCR_WL(16) | SSI_SRCCR_DC(7) | SSI_SRCCR_PM(0));
        SSI1_STMSK = ~((1 << 0) | (1 << 2));
        SSI1_SRMSK = ~((1 << 0) | (1 << 2));
        pll_fsref_44100(tsc2100_devdata->dac_sample_rate, 2);
        break;

    case 32000:
        /* 16 bit, 48KHz, 4 samples/frame */
        SSI1_STCCR = (SSI_STCCR_WL(16) | SSI_STCCR_DC(5) | SSI_STCCR_PM(0));
        SSI1_SRCCR = (SSI_SRCCR_WL(16) | SSI_SRCCR_DC(5) | SSI_SRCCR_PM(0));
        SSI1_STMSK = ~((1 << 0) | (1 << 2));
        SSI1_SRMSK = ~((1 << 0) | (1 << 2));
        pll_fsref_48000(tsc2100_devdata->dac_sample_rate, 1);
        break;

    case 44100:
        /* 16 bit, 44.1KHz, 4 samples/frame */
        SSI1_STCCR = (SSI_STCCR_WL(16) | SSI_STCCR_DC(3) | SSI_STCCR_PM(0));
        SSI1_SRCCR = (SSI_SRCCR_WL(16) | SSI_SRCCR_DC(3) | SSI_SRCCR_PM(0));
        SSI1_STMSK = ~((1 << 0) | (1 << 2));
        SSI1_SRMSK = ~((1 << 0) | (1 << 2));
        pll_fsref_44100(tsc2100_devdata->dac_sample_rate,0);
        break;

    case 48000:
        /* 16 bit, 48KHz, 4 samples/frame */
        SSI1_STCCR = (SSI_STCCR_WL(16) | SSI_STCCR_DC(3) | SSI_STCCR_PM(0));
        SSI1_SRCCR = (SSI_SRCCR_WL(16) | SSI_SRCCR_DC(3) | SSI_SRCCR_PM(0));
        SSI1_STMSK = ~((1 << 0) | (1 << 2));
        SSI1_SRMSK = ~((1 << 0) | (1 << 2));
        pll_fsref_48000(tsc2100_devdata->dac_sample_rate,0);
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
    chip->c_per_size = snd_pcm_lib_period_bytes(substream);
    chip->c_buf_size = snd_pcm_lib_buffer_bytes(substream);

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


static int snd_tsc2100_pcm_capture_trigger(snd_pcm_substream_t *substream, int cmd)
{
    tsc2100_chip_t *chip = snd_pcm_substream_chip(substream);
    snd_pcm_runtime_t *runtime = substream->runtime;

    switch(cmd) {
    case SNDRV_PCM_TRIGGER_START:

        /* If ALSA wants to give us two periods, set them both up  */

        while (CCR(chip->c_dma) & CCR_CEN) {
            printk("TRIGGER_START while DMA is already enabled\n");
        }
        DBG("%d: dma=%lx ack=%x\n", 
            chip->c_period, chip->c_period * chip->c_per_size, chip->c_pos);
        setup_dma_xfer(chip->c_dma,
                       (u32)&SSI1_SRX0_PHYS, 
                       runtime->dma_addr + (chip->c_period * chip->c_per_size),
                       chip->c_per_size);
        CCR(chip->c_dma) |= CCR_ACRPT;
        chip->c_period = (chip->c_period + 1) % runtime->periods;
        SSI1_SCR |= SSI_SCR_RE;
        SSI1_SIER |= SSI_SIER_RDMAE;
        imx_dma_enable(chip->c_dma);

        if ((runtime->periods >= 2)) {
            CCR(chip->c_dma) |= CCR_RPT;
            DBG("%d: dma=%lx ack=%x\n", 
                chip->c_period, chip->c_period * chip->c_per_size, chip->c_pos);
            setup_dma_xfer(chip->c_dma,
                           (u32)&SSI1_SRX0_PHYS,
                           runtime->dma_addr + (chip->c_period * chip->c_per_size),
                           chip->c_per_size);
            chip->c_period = (chip->c_period + 1) % runtime->periods;
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


static snd_pcm_uframes_t snd_tsc2100_pcm_capture_pointer(
    snd_pcm_substream_t *substream)
{
    tsc2100_chip_t *chip = snd_pcm_substream_chip(substream);
    size_t ptr = chip->c_pos % chip->c_buf_size;
    return bytes_to_frames(substream->runtime, ptr);
}


static void tsc2100_dma_capture_isr(int irq, void *data, struct pt_regs *regs)
{
    tsc2100_chip_t *chip = (tsc2100_chip_t *)data;
    snd_pcm_substream_t *substream = chip->c_substream;
    snd_pcm_runtime_t *runtime = substream->runtime;

    if (DBTOSR != 0) printk("DMA Error: DBTOSR = 0x%x\n", DBTOSR);
    if (DRTOSR != 0) printk("DMA Error: DRTOSR = 0x%x\n", DRTOSR);
    if (DSESR != 0)  printk("DMA Error: DSESR = 0x%x\n", DSESR);
    if (DBOSR != 0)  printk("DMA Error: DBOSR = 0x%x\n", DBOSR);

    chip->c_pos += chip->c_per_size;

    DBG("%d: dma=%lx ack=%x\n", 
        chip->c_period, chip->c_period * chip->c_per_size, chip->c_pos);
    setup_dma_xfer(chip->c_dma,
                   (u32)&SSI1_SRX0_PHYS,
                   runtime->dma_addr + (chip->c_period * chip->c_per_size),
                   chip->c_per_size);
    CCR(chip->c_dma) |= CCR_RPT;
    chip->c_period = (chip->c_period + 1) % runtime->periods;

    snd_pcm_period_elapsed(substream);
}



static snd_pcm_ops_t snd_tsc2100_playback_ops = {
    .open      = snd_tsc2100_playback_open,
    .close     = snd_tsc2100_playback_close,
    .ioctl     = snd_pcm_lib_ioctl,
    .hw_params = snd_tsc2100_pcm_hw_params,
    .hw_free   = snd_tsc2100_pcm_hw_free,
    .prepare   = snd_tsc2100_pcm_playback_prepare,
    .trigger   = snd_tsc2100_pcm_playback_trigger,
    .pointer   = snd_tsc2100_pcm_playback_pointer,
};

static snd_pcm_ops_t snd_tsc2100_capture_ops = {
    .open      = snd_tsc2100_capture_open,
    .close     = snd_tsc2100_capture_close,
    .ioctl     = snd_pcm_lib_ioctl,
    .hw_params = snd_tsc2100_pcm_hw_params,
    .hw_free   = snd_tsc2100_pcm_hw_free,
    .prepare   = snd_tsc2100_pcm_capture_prepare,
    .trigger   = snd_tsc2100_pcm_capture_trigger,
    .pointer   = snd_tsc2100_pcm_capture_pointer,
};


static int __init snd_tsc2100_pcm_new(tsc2100_chip_t *chip)
{
    snd_pcm_t *pcm;
    int rc;

    rc = snd_pcm_new(chip->card, "Chumby-TSC2100-PCM", 0, 1, 1, &pcm);
    if (rc < 0) {
        return rc;
    }

    strcpy(pcm->name, "Chumby-TSC2100-PCM");
    pcm->private_data = chip;
    chip->pcm = pcm;

    snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_CONTINUOUS,
                                          snd_dma_continuous_data(GFP_KERNEL),
                                          BUFFER_BYTES_MAX, BUFFER_BYTES_MAX);

    snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &snd_tsc2100_playback_ops);
    snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &snd_tsc2100_capture_ops);
    return 0;
}


/************************************************************
 * Mixer Functions
 ************************************************************/
static int snd_tsc2100_mixer_mvol_info(snd_kcontrol_t *kcontrol,
                                      snd_ctl_elem_info_t *uinfo)
{
    uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
    uinfo->count = 2;
    uinfo->value.integer.min = 0x00;
    uinfo->value.integer.max = 0x7F;
    return 0;
}


static int snd_tsc2100_mixer_mvol_get(snd_kcontrol_t *kcontrol,
                                     snd_ctl_elem_value_t *uctrl)
{
    u16 val;

    val = ~tsc2100_regread( TSC2100_REG_AUDIODAC );
    uctrl->value.integer.value[0] = ( val >> 8 ) & 0x7f;
    uctrl->value.integer.value[1] = val & 0x7f;
    return 0;
}


static int snd_tsc2100_mixer_mvol_put(snd_kcontrol_t *kcontrol,
                                snd_ctl_elem_value_t *uctrl)
{
    int changed = 0;
    u16 val;
    int lvol;
    int rvol;

    val = ~tsc2100_regread( TSC2100_REG_AUDIODAC );
    lvol = ( val >> 8 ) & 0x7f;
    rvol = val & 0x7f;

    if ((uctrl->value.integer.value[0] != lvol) ||
        (uctrl->value.integer.value[1] != rvol)) {
        changed = 1;

        lvol = uctrl->value.integer.value[0] & 0x7F;
        rvol = uctrl->value.integer.value[1] & 0x7F;
    }

    tsc2100_devdata->reg.audiodac = ( tsc2100_devdata->reg.audiodac & 0x8080 ) |
                                     (~((lvol<<8) | rvol) & 0x7F7F);
    tsc2100_regwrite( TSC2100_REG_AUDIODAC, tsc2100_devdata->reg.audiodac );
    return changed;
}


static int snd_tsc2100_mixer_insel_info(snd_kcontrol_t *kcontrol,
                                       snd_ctl_elem_info_t *uinfo)
{
    uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
    uinfo->count = 1;
    uinfo->value.integer.min = 0;
    uinfo->value.integer.max = 1;
    return 0;
}


static int snd_tsc2100_mixer_insel_get(snd_kcontrol_t *kcontrol,
                                      snd_ctl_elem_value_t *uctrl)
{
    u16 val;

    val = tsc2100_regread(TSC2100_REG_CONTROL1);
    if (val & TSC2100_CONTROL1_ADCIN_MASK) {
        uctrl->value.integer.value[0] = 1;
    } else {
        uctrl->value.integer.value[0] = 0;
    }
    return 0;
}


static int snd_tsc2100_mixer_insel_put(snd_kcontrol_t *kcontrol,
                                snd_ctl_elem_value_t *uctrl)
{
    int changed;
    u16 val;

    val = tsc2100_regread(TSC2100_REG_CONTROL1);
    if ((val & TSC2100_CONTROL1_ADCIN_MASK) && uctrl->value.integer.value[0]) {
        changed = 0;
    } else {
        changed = 1;
    }

    val = (val & ~TSC2100_CONTROL1_ADCIN_MASK) |
          TSC2100_CONTROL1_ADCIN(uctrl->value.integer.value[0]);
    tsc2100_devdata->reg.control1 = val;
    tsc2100_regwrite(TSC2100_REG_CONTROL1, val);
    return changed;
}


static int snd_tsc2100_mixer_micboost_info(snd_kcontrol_t *kcontrol,
                                          snd_ctl_elem_info_t *uinfo)
{
    uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
    uinfo->count = 1;
    uinfo->value.integer.min = 0;
    uinfo->value.integer.max = 1;
    return 0;
}


static int snd_tsc2100_mixer_micboost_get(snd_kcontrol_t *kcontrol,
                                         snd_ctl_elem_value_t *uctrl)
{
    #if 0
    tsc2100_chip_t *chip = snd_kcontrol_chip(kcontrol);
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


static int snd_tsc2100_mixer_micboost_put(snd_kcontrol_t *kcontrol,
                                snd_ctl_elem_value_t *uctrl)
{
    #if 1
    return 0;
    #else
    tsc2100_chip_t *chip = snd_kcontrol_chip(kcontrol);
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


static int snd_tsc2100_mixer_sidetone_info(snd_kcontrol_t *kcontrol,
                                          snd_ctl_elem_info_t *uinfo)
{
    uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
    uinfo->count = 1;
    uinfo->value.integer.min = 0;
    uinfo->value.integer.max = 1;
    return 0;
}


static int snd_tsc2100_mixer_sidetone_get(snd_kcontrol_t *kcontrol,
                                         snd_ctl_elem_value_t *uctrl)
{
    #if 0
    tsc2100_chip_t *chip = snd_kcontrol_chip(kcontrol);
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


static int snd_tsc2100_mixer_sidetone_put(snd_kcontrol_t *kcontrol,
                                snd_ctl_elem_value_t *uctrl)
{
    #if 1
    return 0;
    #else
    tsc2100_chip_t *chip = snd_kcontrol_chip(kcontrol);
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


static int snd_tsc2100_mixer_bypass_info(snd_kcontrol_t *kcontrol,
                                        snd_ctl_elem_info_t *uinfo)
{
    uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
    uinfo->count = 1;
    uinfo->value.integer.min = 0;
    uinfo->value.integer.max = 1;
    return 0;
}


static int snd_tsc2100_mixer_bypass_get(snd_kcontrol_t *kcontrol,
                                       snd_ctl_elem_value_t *uctrl)
{
    #if 0
    tsc2100_chip_t *chip = snd_kcontrol_chip(kcontrol);
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


static int snd_tsc2100_mixer_bypass_put(snd_kcontrol_t *kcontrol,
                                snd_ctl_elem_value_t *uctrl)

{
    #if 1
    return 0;
    #else
    tsc2100_chip_t *chip = snd_kcontrol_chip(kcontrol);
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


static snd_kcontrol_new_t tsc2100_mixer_controls[] = {
    {
        .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
        .name  = "Master Playback Volume",
        .info  = snd_tsc2100_mixer_mvol_info,
        .get   = snd_tsc2100_mixer_mvol_get,
        .put   = snd_tsc2100_mixer_mvol_put,
    }, {
        .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
        .name  = "Input Select Switch",
        .info  = snd_tsc2100_mixer_insel_info,
        .get   = snd_tsc2100_mixer_insel_get,
        .put   = snd_tsc2100_mixer_insel_put,
    }, {
        .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
        .name  = "Mic Boost",
        .info  = snd_tsc2100_mixer_micboost_info,
        .get   = snd_tsc2100_mixer_micboost_get,
        .put   = snd_tsc2100_mixer_micboost_put,
    }, {
        .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
        .name  = "Sidetone Switch",
        .info  = snd_tsc2100_mixer_sidetone_info,
        .get   = snd_tsc2100_mixer_sidetone_get,
        .put   = snd_tsc2100_mixer_sidetone_put,
    }, {
        .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
        .name  = "Bypass Switch",
        .info  = snd_tsc2100_mixer_bypass_info,
        .get   = snd_tsc2100_mixer_bypass_get,
        .put   = snd_tsc2100_mixer_bypass_put,
    },
};


static int __init snd_tsc2100_mixer_new(tsc2100_chip_t *chip)
{
    snd_card_t *card = chip->card;
    int rc;
    int i;

    rc = snd_device_new(card, SNDRV_DEV_CODEC, chip, &g_mixer_ops);

    for (i = 0; i < ARRAY_SIZE(tsc2100_mixer_controls); i++) {
        rc = snd_ctl_add(card, snd_ctl_new1(&tsc2100_mixer_controls[i], chip));
        if (rc < 0)
            return rc;
    }
    return rc;
}


/***********************************************************************
 * TouchScreen Driver Routines
 ***********************************************************************/
static void tsc2100_touchscreen_report(struct tsc2100_data *tsc2100_ts,
                                       int x, int y, int p, int pendown)
{
    ringwrite( &tsc2100_ts->ring_x, x );
    ringwrite( &tsc2100_ts->ring_y, y );
    ringwrite( &tsc2100_ts->ring_p, p );
    ringwrite( &tsc2100_ts->ring_pen, pendown );

    input_report_abs(tsc2100_ts->inputdevice, ABS_X, y);
    input_report_abs(tsc2100_ts->inputdevice, ABS_Y, x);
    input_report_abs(tsc2100_ts->inputdevice, ABS_PRESSURE, p);
    input_report_key(tsc2100_ts->inputdevice, BTN_TOUCH, pendown);
    input_sync(tsc2100_ts->inputdevice);
}


static void tsc2100_touchscreen_setup(struct device *dev)
{
	struct tsc2100_data *tsc2100_ts = dev_get_drvdata(dev);

	tsc2100_ts->inputdevice->name = "tsc2100_ts";
	tsc2100_ts->inputdevice->evbit[0] = BIT(EV_KEY) | BIT(EV_ABS);
	tsc2100_ts->inputdevice->keybit[LONG(BTN_TOUCH)] |= BIT(BTN_TOUCH);
	input_set_abs_params(tsc2100_ts->inputdevice,
                         ABS_X, X_AXIS_MIN, X_AXIS_MAX, 0, 0);
	input_set_abs_params(tsc2100_ts->inputdevice,
                         ABS_Y, Y_AXIS_MIN, Y_AXIS_MAX, 0, 0);
	input_set_abs_params(tsc2100_ts->inputdevice,
                         ABS_PRESSURE, PRESSURE_MIN, PRESSURE_MAX, 0, 0);
	input_register_device(tsc2100_ts->inputdevice);
	printk("Chumby TI-TSC2100 TouchScreen Driver initialized\n");
}


static int tsc2100_pendown( void )
{
    return tsc2100_regread(TSC2100_REG_STATUS) & TSC2100_STATUS_DAVAIL;
}


static int average_power( struct tsc2100_ringbuf* r )
{
    int nxt = ringbeg(r);
    int cnt = ringcnt(r);
    int i, avg;

    if ( cnt > max_samples ) 
        cnt = max_samples;
    if ( cnt == 0 )
        return 0;
    for ( avg = i = 0; i < cnt; ++i )
        avg += ringread(r, &nxt);
    return avg / cnt;
}


static int average_dc_power( struct tsc2100_data* d )
{
    return average_power( &d->ring_bat1 );
}


static int average_battery_power( struct tsc2100_data* d )
{
    return average_power( &d->ring_bat2 );
}


static void tsc2100_readdata(struct tsc2100_data *devdata,
                             struct tsc2100_ts_event *ts_data)
{
    int z1,z2;
    u32 values[4],status;

    status = tsc2100_regread(TSC2100_REG_STATUS);
    if ( status & (TSC2100_STATUS_XSTAT  |
                   TSC2100_STATUS_YSTAT  |
                   TSC2100_STATUS_Z1STAT |
                   TSC2100_STATUS_Z2STAT) ) {
        /* Read X, Y, Z1 and Z2
         */
        spi_read(TSC2100_REG_X, &values[0], 4);
        ts_data->x=values[0];
        ts_data->y=values[1];
        z1=values[2];
        z2=values[3];

        /* Calculate Pressure
         */
        if ((z1 != 0) && (ts_data->x!=0) && (ts_data->y!=0))
            ts_data->p = ((ts_data->x * (z2 - z1) / z1));
        else
            ts_data->p = 0;
    }

    if ( status & ( TSC2100_STATUS_B1STAT | TSC2100_STATUS_B2STAT ) )
      imx_gpio_write( 30 | GPIO_PORTC, 0 ); // turn off the battery gate FET (it's turned on by tsc2100_get_miscdata)

    if ( status & TSC2100_STATUS_B1STAT )
    {
        devdata->miscdata.bat1 = tsc2100_regread(TSC2100_REG_BAT1);
        ringwrite( &devdata->ring_bat1, devdata->miscdata.bat1 );
    }
    if ( status & TSC2100_STATUS_B2STAT )
    {
        devdata->miscdata.bat2 = tsc2100_regread(TSC2100_REG_BAT2);
        ringwrite( &devdata->ring_bat2, devdata->miscdata.bat2 );
    }
    if ( status & TSC2100_STATUS_T1STAT )
    {
        devdata->miscdata.temp1 = tsc2100_regread(TSC2100_REG_TEMP1);
        ringwrite( &devdata->ring_temp1, devdata->miscdata.temp1 );
    }
    if ( status & TSC2100_STATUS_T2STAT )
    {
        devdata->miscdata.temp2 = tsc2100_regread(TSC2100_REG_TEMP2);
        ringwrite( &devdata->ring_temp2, devdata->miscdata.temp2 );
    }

    if ( devdata->mode == TSC2100_MODE_MISC ) {
        /* switch back to touchscreen autoscan mode
        */
        tsc2100_regwrite(TSC2100_REG_ADC, devdata->reg.adc);
        devdata->mode = TSC2100_MODE_TS;
    }

}


static void tsc2100_touchscreen_reset(struct tsc2100_data *devdata)
{
    tsc2100_regwrite(TSC2100_REG_RESETCTL, 0xbb00);
}


static void tsc2100_touchscreen_enable(struct tsc2100_data *devdata)
{
    /* PINTDAV is data available only
     */
    tsc2100_regwrite(TSC2100_REG_STATUS, 0x4000);
    /* hgroover - this should only occur with an explicit 1 write
     * to /proc or at init time. */
    TSDBG("chumby_tsc2100[%d]: tsc2100_touchscreen_enable() called!\n", __LINE__);

    /* TSC2100-controlled conversions, 12-bit samples, continuous X,Y,Z1,Z2
     * scan mode, average (mean) 4 samples per coordinate, 1 MHz internal
     * conversion clock, 500 usec panel voltage stabilization delay
     */
    tsc2100_regwrite(TSC2100_REG_ADC, devdata->reg.adc);
    devdata->mode = TSC2100_MODE_TS;

    /* Initialize the reference voltage generator (this eliminates excess noise
     * on sampling lines).
     */
    tsc2100_regwrite(TSC2100_REG_REFCTL, devdata->reg.refctl);
}


static void tsc2100_touchscreen_disable(struct tsc2100_data *devdata)
{
    /* stop conversions and power down
     */
    tsc2100_regwrite(TSC2100_REG_ADC, 0x4000);
}


static void touchscreen_interrupt_main(struct tsc2100_data *devdata,
                                       int isTimer, struct pt_regs *regs)
{
    unsigned long flags;
    u32 value;
    static int iter = 0;
    static int nonidle = 0;
    static int reporting = 0;
    static int state = 0;
    u32 status;

    if ( !devdata->tson )
        return;

    spin_lock_irqsave(&devdata->lock, flags);

    devdata->inactive_count = 0;

    if( isTimer ) {
      // this initiates a conversion
      if( devdata->state != TS_IDLE ) {
	// getting here means the timer triggered while a conversion is in progress
	// it's not a bad thing, but let's track how often this happens
	// if it happens very often we may want to optimize this case
	nonidle++;
	if( (nonidle % 256) == 0 )
	  printk( "?" );
      } else {
	// we're in idle, first check and see if we are looking for battery information
	if( ((iter % (poll_interval * HZ)) == 0) && (!reporting) ) { // do this only when not tracking the touchscreen
	  switch ( ++state & 3 ) {
	  case 1:  //bat1
            imx_gpio_write( 28 | GPIO_PORTC, 1 );    // this is for the line voltage sense
            tsc2100_regwrite(TSC2100_REG_ADC, ADCMODE(0x6));
            break;
	  case 2:  //bat2
            imx_gpio_write( 30 | GPIO_PORTC, 1 );    // this is for the battery voltage sense
            tsc2100_regwrite(TSC2100_REG_ADC, ADCMODE(0x7));
            break;
	  case 3:  //temp1
            tsc2100_regwrite(TSC2100_REG_ADC, ADCMODE(0xA));
            break;
	  default: //temp2
            tsc2100_regwrite(TSC2100_REG_ADC, ADCMODE(0xC));
            break;
	  }
	  devdata->state = TS_MISC;
	} else {
	  if( ((iter % 4) == 0) || (reporting) ) {  // when not in pendown, scan at 1/4 rate (25 Hz) to save CPU time
	    //	    printk( "*" );
	    devdata->reg.adc &= ~TSC2100_ADC_ADMODE_MASK;
	    devdata->reg.adc |= TSC2100_ADC_ADMODE(3); // convert X
	    tsc2100_regwrite(TSC2100_REG_ADC, devdata->reg.adc);
	    devdata->state = TS_XMODE;
	  }
	}
      }

      iter++; // always increment iter every timer tick
      // reschedule myself
      mod_timer(&(devdata->ts_timer), jiffies + HZ / 100);

    } else {
      // perhaps recode this so that the if/then statement is based solely on the conversion result available
      // and not on the "state" variable, which can be out of sync with the hardware
      status = tsc2100_regread(TSC2100_REG_STATUS);
      if( devdata->state == TS_MISC ) {
	//	printk(".");
	/// update this to reflect FET changes
	if ( status & ( TSC2100_STATUS_B1STAT | TSC2100_STATUS_B2STAT ) ) {
	  imx_gpio_write( 30 | GPIO_PORTC, 0 );  // just turn both off, it's safer that way
	  imx_gpio_write( 28 | GPIO_PORTC, 0 ); 
	}
	if ( status & TSC2100_STATUS_B1STAT )
	  {
	    devdata->miscdata.bat1 = tsc2100_regread(TSC2100_REG_BAT1);
	    ringwrite( &devdata->ring_bat1, devdata->miscdata.bat1 );
	  }
	if ( status & TSC2100_STATUS_B2STAT )
	  {
	    devdata->miscdata.bat2 = tsc2100_regread(TSC2100_REG_BAT2);
	    ringwrite( &devdata->ring_bat2, devdata->miscdata.bat2 );
	  }
	if ( status & TSC2100_STATUS_T1STAT ) 
	  {
	    devdata->miscdata.temp1 = tsc2100_regread(TSC2100_REG_TEMP1);
	    ringwrite( &devdata->ring_temp1, devdata->miscdata.temp1 );
	  }
	if ( status & TSC2100_STATUS_T2STAT ) 
	  {
	    devdata->miscdata.temp2 = tsc2100_regread(TSC2100_REG_TEMP2);
	    ringwrite( &devdata->ring_temp2, devdata->miscdata.temp2 );
	  }

	// now clear all scans until the next tick
	devdata->reg.adc &= ~TSC2100_ADC_ADMODE_MASK; // no scan, idle until next tick
	tsc2100_regwrite(TSC2100_REG_ADC, devdata->reg.adc);
	devdata->state = TS_IDLE;

	// is new since greg's changes to add SIGPWR
	// only allow a SIGPWR signal to be generated every sigpwr_holdoff seconds
	if ( sigpwr_next > 0 ) 
	  --sigpwr_next;
    
	if( sigpwr_next == 0 ) {  // put this around the outside so as to make this check more efficient
	  if ( ( status & TSC2100_STATUS_B1STAT ) && 
	       sigpwr_pid != 0                    &&
	       average_dc_power(devdata) < dc_power_min ) {
	    /*
	    ** Just performed a dc battery measurement and the power is less than
	    ** the minimum threshold (dc_power_min).  If a SIGPWR signal has not
	    ** been generated in sigpwr_holdoff seconds and a process has registered
	    ** to received SIGPWR signals (sigpwr_pid != 0) -- generate a SIGPWR sig
	    ** to the process that wrote its PID in the /proc/chumby/sigpwr_pid file 
	    */
	    sigpwr_next = sigpwr_holdoff; // this routine is only called at an interval defined by poll_interval
	    kill_proc(sigpwr_pid, SIGPWR, 1);
	  }
	}
	//	printk( "-" );

      } else if( devdata->state == TS_XMODE ) {
	//	printk("+");
	if(!(tsc2100_regread(TSC2100_REG_STATUS) & TSC2100_STATUS_XSTAT) ) {
	  printk( "interrupt trigger without data (mode=x)!\n" );
	}
	spi_read(TSC2100_REG_X, &value, 1);
	devdata->tsdata.x = value;
	
	devdata->reg.adc &= ~TSC2100_ADC_ADMODE_MASK;
	devdata->reg.adc |= TSC2100_ADC_ADMODE(4); // convert Y
	tsc2100_regwrite(TSC2100_REG_ADC, devdata->reg.adc);
	devdata->state = TS_YMODE;
      } else if( devdata->state == TS_YMODE ) {
	// y-stat should also automatically be triggered by the scan
	if(!(tsc2100_regread(TSC2100_REG_STATUS) & TSC2100_STATUS_YSTAT) ) {
	  printk( "interrupt trigger without data (mode=y)!\n" );
	}
	spi_read(TSC2100_REG_Y, &value, 1);
	devdata->tsdata.y = value;

	// now clear both scans until the next tick
	devdata->reg.adc &= ~TSC2100_ADC_ADMODE_MASK; // no scan, idle until next tick
	tsc2100_regwrite(TSC2100_REG_ADC, devdata->reg.adc);
	devdata->state = TS_IDLE;
      } else {
	printk("huh?");
	// how in the hell did we get here?
      }

      // now, we need a mechanism to determine if there is a pendown
      // and if there is, call tsc2100_touchscreen_report(devdata, ts_data.x, ts_data.y, ts_data.p, 1) or
      // tsc2100_touchscreen_report(devdata, 0, 0, 0, 0) depending on pen up or pendown condition

      if( devdata->tsdata.x != 0 && devdata->tsdata.y != 4095 ) {
	tsc2100_touchscreen_report(devdata, devdata->tsdata.y, devdata->tsdata.x, 42, 1);
	reporting = 1;
      } else if ( reporting ) {
	tsc2100_touchscreen_report(devdata, 0, 0, 0, 0);
	reporting = 0;
      } else {
	reporting = 0;
      }

    }
    spin_unlock_irqrestore(&devdata->lock, flags);

}


static void tsc2100_timer(unsigned long data)
{
    struct tsc2100_data *devdata = (struct tsc2100_data *) data;
    touchscreen_interrupt_main(devdata, 1, NULL);
}


static irqreturn_t touchscreen_handler(
                            int irq, void *dev_id, struct pt_regs *regs)
{
    struct tsc2100_data *devdata = dev_id;

    /* Switch from falling edge trigger to level, we are playing games with the
     * interrupt type to reduce the number of interrupts received while the PEN
     * is down, we want to read the first sample of the PEN selection and then
     * we want to report PEN up events some short sequence later (10-30ms) and
     * not have to remain in the interrupt handler reading all of the
     * new X/Y/Z1/Z2 samples...
     */
    touchscreen_interrupt_main(devdata, 0, regs);
    return IRQ_HANDLED;
}


static void RegDumpViaPrintk( const char *msg, struct tsc2100_data* d )
{
	printk( "Register dump%s\n", msg );

    printk( "PAGE0 REGISTERS: " );
    printk( "X=%x, Y=%x, Z1=%x, Z2=%x, ",
                     tsc2100_regread(TSC2100_REG_X),
                     tsc2100_regread(TSC2100_REG_Y),
                     tsc2100_regread(TSC2100_REG_Z1),
                     tsc2100_regread(TSC2100_REG_Z2) );
    printk( "BAT1=%x, BAT2=%x, AUX=%x, TEMP1=%x, TEMP2=%x\n",
                     tsc2100_regread(TSC2100_REG_BAT1),
                     tsc2100_regread(TSC2100_REG_BAT2),
                     tsc2100_regread(TSC2100_REG_AUX),
                     tsc2100_regread(TSC2100_REG_TEMP1),
                     tsc2100_regread(TSC2100_REG_TEMP2) );
    printk( "PAGE1 REGISTERS: " );
    printk("ADC=%x(%x), STATUS=%x, REFCTL=%x(%x)\n",
                     tsc2100_regread(TSC2100_REG_ADC), d->reg.adc,
                     tsc2100_regread(TSC2100_REG_STATUS),
                     tsc2100_regread(TSC2100_REG_REFCTL), d->reg.refctl );

    printk( "PAGE2 REGISTERS: " );
    printk( "CONTROL1=%x(%x), AUDIOADC=%x(%x), AUDIODAC=%x(%x), ",
                     tsc2100_regread(TSC2100_REG_CONTROL1), d->reg.control1,
                     tsc2100_regread(TSC2100_REG_AUDIOADC), d->reg.audioadc,
                     tsc2100_regread(TSC2100_REG_AUDIODAC), d->reg.audiodac );
    printk( "SIDETONE=%x(%x), CONTROL2=%x(%x), POWERCON=%x(%x)\n",
                     tsc2100_regread(TSC2100_REG_SIDETONE), d->reg.sidetone,
                     tsc2100_regread(TSC2100_REG_CONTROL2), d->reg.control2,
                     tsc2100_regread(TSC2100_REG_POWERCON), d->reg.powercon );
    printk( "  CONTROL3=%x(%x), PLL1=%x(%x), PLL2=%x(%x), ",
                     tsc2100_regread(TSC2100_REG_CONTROL3), d->reg.control3,
                     tsc2100_regread(TSC2100_REG_PLL1), d->reg.pll1,
                     tsc2100_regread(TSC2100_REG_PLL2), d->reg.pll2 );
    printk( "CONTROL4=%x(%x), CONTROL5=%x(%x)\n",
                     tsc2100_regread(TSC2100_REG_CONTROL4), d->reg.control4,
                     tsc2100_regread(TSC2100_REG_CONTROL5), d->reg.control5 );
	printk( "----- end register dump -----\n" );
}

static void tsc2100_get_miscdata( struct tsc2100_data* devdata )
{
    static int state = 0;
    unsigned long flags;

    spin_lock_irqsave(&(devdata->lock), flags); 
    if ( devdata->pendown == 0 && devdata->tson ) {
        devdata->mode = TSC2100_MODE_MISC;
        switch ( ++state & 3 ) {
        case 1:  //bat1
            imx_gpio_write( 30 | GPIO_PORTC, 1 );
            tsc2100_regwrite(TSC2100_REG_ADC, ADCMODE(0x6));
            break;
        case 2:  //bat2
            imx_gpio_write( 30 | GPIO_PORTC, 1 );
            tsc2100_regwrite(TSC2100_REG_ADC, ADCMODE(0x7));
            break;
        case 3:  //temp1
            tsc2100_regwrite(TSC2100_REG_ADC, ADCMODE(0xA));
            break;
        default: //temp2
            tsc2100_regwrite(TSC2100_REG_ADC, ADCMODE(0xC));
            break;
        }

    }
    spin_unlock_irqrestore(&devdata->lock, flags);
}


/***********************************************************************
 * Proc FileSystem Routines
 ***********************************************************************/

static struct proc_dir_entry* __init proc_file(
         char* name,
         struct proc_dir_entry* parent,
         int (*read_proc)( char* buf, char** start, off_t offset,
                           int count, int* eof, void* data ),
         int (*write_proc)( struct file* file, const char* buf,
                            unsigned long count, void* data ),
         void* data )
{
    struct proc_dir_entry* entry;

    entry = create_proc_entry( name, 0, parent );
    if ( entry ) {
        entry->read_proc = read_proc;
        entry->write_proc = write_proc;
        entry->data = data;
    }

    return entry;
}


static int getval_from_user( struct file* file,
                             const char* buf,
                             u32 count, u32* value )
{
    int   len = count;
    char  string[32];

    if ( len > sizeof(string)-1 )
        len = sizeof(string)-1;
    if ( copy_from_user( string, buf, len ) )
        return -EFAULT;

    string[len] = '\0';
    *value = simple_strtoul( string, NULL, 0 );
    return 0;
}


static int proc_get_touchclick_cb( char* buf, char** start, off_t offset,
                                   int count, int* eof, void* data )
{
    struct tsc2100_data* d = ( struct tsc2100_data* ) data;
    return sprintf( buf, "%d\n", d->touchclick );
}


static int proc_set_touchclick_cb( struct file* file, const char* buf,
                                   unsigned long count, void* data )
{
    struct tsc2100_data* d = ( struct tsc2100_data* ) data;
    u32  value;
    int  rc = getval_from_user(file,buf,count,&value);
    if ( rc )
        return rc;

    d->touchclick = value;

    TSDBG( "chumby_tsc2100: proc_set_touchclick_cb enable value %d\n", value );

    return count;
}


static int proc_get_tson_cb( char* buf, char** start, off_t offset,
                            int count, int* eof, void* data )
{
    struct tsc2100_data* d = ( struct tsc2100_data* ) data;
    return sprintf( buf, "%d\n", d->tson );
}


static int proc_set_tson_cb( struct file* file, const char* buf,
                             unsigned long count, void* data )
{
    struct tsc2100_data* d = ( struct tsc2100_data* ) data;
    u32  value;
    int  rc = getval_from_user(file,buf,count,&value);
    if ( rc )
        return rc;

    d->tson = value;
    /* hgroover - check for spurious calls to enable */
    TSDBG( "chumby_tsc2100: proc_set_tson_cb enable value %d\n", value );
    if ( value )
        tsc2100_touchscreen_enable(d);
    else
        tsc2100_touchscreen_disable(d);
    return count;
}


static int proc_get_coords_cb( char* buf, char** start, off_t offset,
                               int count, int* eof, void* data )
{
    struct tsc2100_data* d = ( struct tsc2100_data* ) data;
    int x, y, p, pen, cnt;
    int leng = 0;
    unsigned long flags;

    spin_lock_irqsave(&tsc2100_devdata->lock, flags);
    x   = ringbeg(&d->ring_x);
    y   = ringbeg(&d->ring_y);
    p   = ringbeg(&d->ring_p);
    pen = ringbeg(&d->ring_pen);
    cnt = ringcnt(&d->ring_x);
    spin_unlock_irqrestore(&tsc2100_devdata->lock, flags);

    while ( leng < count - 48 && cnt != 0 ) {
        leng += sprintf( buf + leng, "%d: ", cnt-- );
        leng += sprintf( buf + leng, "x=%d ", ringread(&d->ring_x,&x) );
        leng += sprintf( buf + leng, "y=%d ", ringread(&d->ring_y,&y) );
        leng += sprintf( buf + leng, "p=%d ", ringread(&d->ring_p,&p) );
        leng += sprintf( buf + leng, "%s\n",  ringread(&d->ring_pen, &pen)?
                                              "pen-down" : "pen-up" );
    }
    *eof = 1;
    return leng;
}


static int proc_get_tsregs_cb( char* buf, char** start, off_t offset,
                               int count, int* eof, void* data )
{
    struct tsc2100_data* d = ( struct tsc2100_data* ) data;
    int    leng = 0;

    leng += sprintf( buf + leng, "PAGE0 REGISTERS:\n" );
    leng += sprintf( buf + leng, "X=%x, Y=%x, Z1=%x, Z2=%x\n",
                     tsc2100_regread(TSC2100_REG_X),
                     tsc2100_regread(TSC2100_REG_Y),
                     tsc2100_regread(TSC2100_REG_Z1),
                     tsc2100_regread(TSC2100_REG_Z2) );
    leng += sprintf( buf + leng, "BAT1=%x, BAT2=%x, AUX=%x, TEMP1=%x, TEMP2=%x\n",
                     tsc2100_regread(TSC2100_REG_BAT1),
                     tsc2100_regread(TSC2100_REG_BAT2),
                     tsc2100_regread(TSC2100_REG_AUX),
                     tsc2100_regread(TSC2100_REG_TEMP1),
                     tsc2100_regread(TSC2100_REG_TEMP2) );
    leng += sprintf( buf + leng, "\nPAGE1 REGISTERS:\n" );
    leng += sprintf( buf + leng, "ADC=%x(%x), STATUS=%x, REFCTL=%x(%x)\n",
                     tsc2100_regread(TSC2100_REG_ADC), d->reg.adc,
                     tsc2100_regread(TSC2100_REG_STATUS),
                     tsc2100_regread(TSC2100_REG_REFCTL), d->reg.refctl );

    leng += sprintf( buf + leng, "\nPAGE2 REGISTERS:\n" );
    leng += sprintf( buf + leng, "CONTROL1=%x(%x), AUDIOADC=%x(%x), AUDIODAC=%x(%x)\n",
                     tsc2100_regread(TSC2100_REG_CONTROL1), d->reg.control1,
                     tsc2100_regread(TSC2100_REG_AUDIOADC), d->reg.audioadc,
                     tsc2100_regread(TSC2100_REG_AUDIODAC), d->reg.audiodac );
    leng += sprintf( buf + leng, "SIDETONE=%x(%x), CONTROL2=%x(%x), POWERCON=%x(%x)\n",
                     tsc2100_regread(TSC2100_REG_SIDETONE), d->reg.sidetone,
                     tsc2100_regread(TSC2100_REG_CONTROL2), d->reg.control2,
                     tsc2100_regread(TSC2100_REG_POWERCON), d->reg.powercon );
    leng += sprintf( buf + leng, "CONTROL3=%x(%x), PLL1=%x(%x), PLL2=%x(%x)\n",
                     tsc2100_regread(TSC2100_REG_CONTROL3), d->reg.control3,
                     tsc2100_regread(TSC2100_REG_PLL1), d->reg.pll1,
                     tsc2100_regread(TSC2100_REG_PLL2), d->reg.pll2 );
    leng += sprintf( buf + leng, "CONTROL4=%x(%x), CONTROL5=%x(%x)\n",
                     tsc2100_regread(TSC2100_REG_CONTROL4), d->reg.control4,
                     tsc2100_regread(TSC2100_REG_CONTROL5), d->reg.control5 );
    return leng;
}


static int proc_get_adc_cb( char* buf, char** start, off_t offset,
                            int count, int* eof, void* data )
{
    return sprintf( buf, "0x%x(%x)\n",
                    tsc2100_regread(TSC2100_REG_ADC),
                    tsc2100_devdata->reg.adc );
}


static int proc_set_adc_cb( struct file* file, const char* buf,
                            unsigned long count, void* data )
{
    u32  value;
    int  rc = getval_from_user(file,buf,count,&value);
    if ( rc )
        return rc;
    tsc2100_devdata->reg.adc = value;
    tsc2100_regwrite(TSC2100_REG_ADC,tsc2100_devdata->reg.adc);
    return count;
}


static int proc_get_refctl_cb( char* buf, char** start, off_t offset,
                               int count, int* eof, void* data )
{
    return sprintf( buf, "0x%x(%x)\n",
                    tsc2100_regread(TSC2100_REG_REFCTL),
                    tsc2100_devdata->reg.refctl );
}


static int proc_set_refctl_cb( struct file* file, const char* buf,
                               unsigned long count, void* data )
{
    u32  value;
    int  rc = getval_from_user(file,buf,count,&value);
    if ( rc )
        return rc;
    tsc2100_devdata->reg.refctl = value;
    tsc2100_regwrite(TSC2100_REG_REFCTL,tsc2100_devdata->reg.refctl);
    return count;
}


static int proc_get_control1_cb( char* buf, char** start, off_t offset,
                                 int count, int* eof, void* data )
{
    return sprintf( buf, "0x%x(%x)\n",
                    tsc2100_regread(TSC2100_REG_CONTROL1),
                    tsc2100_devdata->reg.control1 );
}


static int proc_set_control1_cb( struct file* file, const char* buf,
                                 unsigned long count, void* data )
{
    u32  value;
    int  rc = getval_from_user(file,buf,count,&value);
    if ( rc )
        return rc;
    tsc2100_devdata->reg.control1 = value;
    tsc2100_regwrite(TSC2100_REG_CONTROL1,tsc2100_devdata->reg.control1);
    return count;
}


static int proc_get_audioadc_cb( char* buf, char** start, off_t offset,
                                 int count, int* eof, void* data )
{
    return sprintf( buf, "0x%x(%x)\n",
                    tsc2100_regread(TSC2100_REG_AUDIOADC),
                    tsc2100_devdata->reg.audioadc );
}


static int proc_set_audioadc_cb( struct file* file, const char* buf,
                                 unsigned long count, void* data )
{
    u32  value;
    int  rc = getval_from_user(file,buf,count,&value);
    if ( rc )
        return rc;
    tsc2100_devdata->reg.audioadc = value;
    tsc2100_regwrite(TSC2100_REG_AUDIOADC,tsc2100_devdata->reg.audioadc);
    return count;
}


static int proc_get_audiodac_cb( char* buf, char** start, off_t offset,
                                 int count, int* eof, void* data )
{
    return sprintf( buf, "0x%x(%x)\n",
                    tsc2100_regread(TSC2100_REG_AUDIODAC),
                    tsc2100_devdata->reg.audiodac );
}


static int proc_set_audiodac_cb( struct file* file, const char* buf,
                                 unsigned long count, void* data )
{
    u32  value;
    int  rc = getval_from_user(file,buf,count,&value);
    if ( rc )
        return rc;
    tsc2100_devdata->reg.audiodac = value;
    tsc2100_regwrite(TSC2100_REG_AUDIODAC,tsc2100_devdata->reg.audiodac);
    return count;
}


static int proc_get_sidetone_cb( char* buf, char** start, off_t offset,
                                 int count, int* eof, void* data )
{
    return sprintf( buf, "0x%x(%x)\n",
                    tsc2100_regread(TSC2100_REG_SIDETONE),
                    tsc2100_devdata->reg.sidetone );
}


static int proc_set_sidetone_cb( struct file* file, const char* buf,
                                 unsigned long count, void* data )
{
    u32  value;
    int  rc = getval_from_user(file,buf,count,&value);
    if ( rc )
        return rc;
    tsc2100_devdata->reg.sidetone = value;
    tsc2100_regwrite(TSC2100_REG_SIDETONE,tsc2100_devdata->reg.sidetone);
    return count;
}


static int proc_get_control2_cb( char* buf, char** start, off_t offset,
                                 int count, int* eof, void* data )
{
    return sprintf( buf, "0x%x(%x)\n",
                    tsc2100_regread(TSC2100_REG_CONTROL2),
                    tsc2100_devdata->reg.control2 );
}


static int proc_set_control2_cb( struct file* file, const char* buf,
                                 unsigned long count, void* data )
{
    u32  value;
    int  rc = getval_from_user(file,buf,count,&value);
    if ( rc )
        return rc;
    tsc2100_devdata->reg.control2 = value;
    tsc2100_regwrite(TSC2100_REG_CONTROL2,tsc2100_devdata->reg.control2);
    return count;
}


static int proc_get_powercon_cb( char* buf, char** start, off_t offset,
                                 int count, int* eof, void* data )
{
    return sprintf( buf, "0x%x(%x)\n",
                    tsc2100_regread(TSC2100_REG_POWERCON),
                    tsc2100_devdata->reg.powercon );
}


static int proc_set_powercon_cb( struct file* file, const char* buf,
                                 unsigned long count, void* data )
{
    u32  value;
    int  rc = getval_from_user(file,buf,count,&value);
    if ( rc )
        return rc;
    tsc2100_devdata->reg.powercon = value;
    tsc2100_regwrite(TSC2100_REG_POWERCON,tsc2100_devdata->reg.powercon);
    return count;
}


static int proc_get_control3_cb( char* buf, char** start, off_t offset,
                                 int count, int* eof, void* data )
{
    return sprintf( buf, "0x%x(%x)\n",
                    tsc2100_regread(TSC2100_REG_CONTROL3),
                    tsc2100_devdata->reg.control3 );
}


static int proc_set_control3_cb( struct file* file, const char* buf,
                                 unsigned long count, void* data )
{
    u32  value;
    int  rc = getval_from_user(file,buf,count,&value);
    if ( rc )
        return rc;
    tsc2100_devdata->reg.control3 = value;
    tsc2100_regwrite(TSC2100_REG_CONTROL3,tsc2100_devdata->reg.control3);
    return count;
}


static int proc_get_pll1_cb( char* buf, char** start, off_t offset,
                             int count, int* eof, void* data )
{
    return sprintf( buf, "0x%x(%x)\n",
                    tsc2100_regread(TSC2100_REG_PLL1),
                    tsc2100_devdata->reg.pll1 );
}


static int proc_set_pll1_cb( struct file* file, const char* buf,
                             unsigned long count, void* data )
{
    u32  value;
    int  rc = getval_from_user(file,buf,count,&value);
    if ( rc )
        return rc;
    tsc2100_devdata->reg.pll1 = value;
    tsc2100_regwrite(TSC2100_REG_PLL1,tsc2100_devdata->reg.pll1);
    return count;
}


static int proc_get_pll2_cb( char* buf, char** start, off_t offset,
                             int count, int* eof, void* data )
{
    return sprintf( buf, "0x%x(%x)\n",
                    tsc2100_regread(TSC2100_REG_PLL2),
                    tsc2100_devdata->reg.pll2 );
}


static int proc_set_pll2_cb( struct file* file, const char* buf,
                             unsigned long count, void* data )
{
    u32  value;
    int  rc = getval_from_user(file,buf,count,&value);
    if ( rc )
        return rc;
    tsc2100_devdata->reg.pll2 = value;
    tsc2100_regwrite(TSC2100_REG_PLL2,tsc2100_devdata->reg.pll2);
    return count;
}


static int proc_get_control4_cb( char* buf, char** start, off_t offset,
                                 int count, int* eof, void* data )
{
    return sprintf( buf, "0x%x(%x)\n",
                    tsc2100_regread(TSC2100_REG_CONTROL4),
                    tsc2100_devdata->reg.control4 );
}


static int proc_set_control4_cb( struct file* file, const char* buf,
                                 unsigned long count, void* data )
{
    u32  value;
    int  rc = getval_from_user(file,buf,count,&value);
    if ( rc )
        return rc;
    tsc2100_devdata->reg.control4 = value;
    tsc2100_regwrite(TSC2100_REG_CONTROL4,tsc2100_devdata->reg.control4);
    return count;
}


static int proc_get_control5_cb( char* buf, char** start, off_t offset,
                                 int count, int* eof, void* data )
{
    return sprintf( buf, "0x%x(%x)\n",
                    tsc2100_regread(TSC2100_REG_CONTROL5),
                    tsc2100_devdata->reg.control5 );
}


static int proc_set_control5_cb( struct file* file, const char* buf,
                                 unsigned long count, void* data )
{
    u32  value;
    int  rc = getval_from_user(file,buf,count,&value);
    if ( rc )
        return rc;
    tsc2100_devdata->reg.control5 = value;
    tsc2100_regwrite(TSC2100_REG_CONTROL5,tsc2100_devdata->reg.control5);
    return count;
}


static int proc_get_left_mute_cb( char* buf, char** start, off_t offset,
                               int count, int* eof, void* data )
{
    unsigned short value = tsc2100_regread(TSC2100_REG_AUDIODAC);
    return sprintf( buf, "%d\n", (value>>15) & 1 );
}


static int proc_set_left_mute_cb( struct file* file, const char* buf,
                               unsigned long count, void* data )
{
    unsigned short regval = tsc2100_regread(TSC2100_REG_AUDIODAC);
    u32  value;
    int  rc = getval_from_user(file,buf,count,&value);
    if ( rc )
        return rc;

    regval = (regval & ~0x8000) | ((value & 1) << 15);
    tsc2100_devdata->reg.audiodac = regval;
    tsc2100_regwrite(TSC2100_REG_AUDIODAC,tsc2100_devdata->reg.audiodac);
    return count;
}


static int proc_get_right_mute_cb( char* buf, char** start, off_t offset,
                               int count, int* eof, void* data )
{
    unsigned short value = tsc2100_regread(TSC2100_REG_AUDIODAC);
    return sprintf( buf, "%d\n", (value>>7) & 1 );
}


static int proc_set_right_mute_cb( struct file* file, const char* buf,
                               unsigned long count, void* data )
{
    unsigned short regval = tsc2100_regread(TSC2100_REG_AUDIODAC);
    u32  value;
    int  rc = getval_from_user(file,buf,count,&value);
    if ( rc )
        return rc;

    regval = (regval & ~0x0080) | ((value & 1) << 7);
    tsc2100_devdata->reg.audiodac = regval;
    tsc2100_regwrite(TSC2100_REG_AUDIODAC,tsc2100_devdata->reg.audiodac);
    return count;
}


static int proc_get_both_mute_cb( char* buf, char** start, off_t offset,
                               int count, int* eof, void* data )
{
    unsigned short value = tsc2100_regread(TSC2100_REG_AUDIODAC);
    return sprintf( buf, "left=%d right=%d\n", (value>>15) & 1, (value>>7) & 1 );
}


static int proc_set_both_mute_cb( struct file* file, const char* buf,
                               unsigned long count, void* data )
{
    unsigned short regval = tsc2100_regread(TSC2100_REG_AUDIODAC);
    u32  value;
    int  rc = getval_from_user(file,buf,count,&value);
    if ( rc )
        return rc;

    value &= 1;
    regval = (regval & ~0x8080) | (value << 15) | (value << 7);
    tsc2100_devdata->reg.audiodac = regval;
    tsc2100_regwrite(TSC2100_REG_AUDIODAC,tsc2100_devdata->reg.audiodac);
    return count;
}


static int proc_get_left_volume_cb( char* buf, char** start, off_t offset,
                                    int count, int* eof, void* data )
{
    unsigned short value = ~tsc2100_regread(TSC2100_REG_AUDIODAC);
    return sprintf( buf, "%d\n", (value>>8) & 0x7F );
}


static int proc_set_left_volume_cb( struct file* file, const char* buf,
                                    unsigned long count, void* data )
{
    unsigned short regval = tsc2100_regread(TSC2100_REG_AUDIODAC);
    u32  value;
    int  rc = getval_from_user(file,buf,count,&value);
    if ( rc )
        return rc;

    regval = (regval & ~0x7F00) | ((~value & 0x7F) << 8);
    tsc2100_devdata->reg.audiodac = regval;
    tsc2100_regwrite(TSC2100_REG_AUDIODAC,tsc2100_devdata->reg.audiodac);
    return count;
}


static int proc_get_right_volume_cb( char* buf, char** start, off_t offset,
                                     int count, int* eof, void* data )
{
    unsigned short value = ~tsc2100_regread(TSC2100_REG_AUDIODAC);
    return sprintf( buf, "%d\n", value & 0x7F );
}


static int proc_set_right_volume_cb( struct file* file, const char* buf,
                                     unsigned long count, void* data )
{
    unsigned short regval = tsc2100_regread(TSC2100_REG_AUDIODAC);
    u32  value;
    int  rc = getval_from_user(file,buf,count,&value);
    if ( rc )
        return rc;

    regval = (regval & ~0x7F) | (~value & 0x7F);
    tsc2100_devdata->reg.audiodac = regval;
    tsc2100_regwrite(TSC2100_REG_AUDIODAC,tsc2100_devdata->reg.audiodac);
    return count;
}


static int proc_get_both_volume_cb( char* buf, char** start, off_t offset,
                                    int count, int* eof, void* data )
{
    unsigned short value = ~tsc2100_regread(TSC2100_REG_AUDIODAC);
    return sprintf( buf, "left=%d, right=%d\n",
                    (value>>8) & 0x7F, value & 0x7F );
}


static int proc_set_both_volume_cb( struct file* file, const char* buf,
                                    unsigned long count, void* data )
{
    unsigned short regval = tsc2100_regread(TSC2100_REG_AUDIODAC);
    u32  value;
    int  rc = getval_from_user(file,buf,count,&value);
    if ( rc )
        return rc;
    value  = ~value & 0x7F;
    regval = (regval & ~0x7F7F) | (value << 8) | value;
    tsc2100_devdata->reg.audiodac = regval;
    tsc2100_regwrite(TSC2100_REG_AUDIODAC,tsc2100_devdata->reg.audiodac);
    return count;
}


static int proc_get_batt_cb( char* buf, char** start, off_t offset,
                            int count, int* eof, void* data )
{
    struct tsc2100_data* d = ( struct tsc2100_data* ) data;
    int b1  = ringbeg(&d->ring_bat1);
    int b2  = ringbeg(&d->ring_bat2);
    int cnt = ringcnt(&d->ring_bat1);
    int leng = 0;

    while ( leng < count - 32 && cnt != 0 ) {
        leng += sprintf( buf + leng, "%d: ", cnt-- );
        leng += sprintf( buf + leng, "%d ", ringread(&d->ring_bat1,&b1) );
        leng += sprintf( buf + leng, "%d\n", ringread(&d->ring_bat2,&b2) );
    }

    if ( leng < count - 32 )
        leng += sprintf( buf + leng, "max_samples=%d\n", max_samples );
    if ( leng < count - 32 )
        leng += sprintf( buf + leng, "dc_power_min=%d\n", dc_power_min );
    if ( leng < count - 32 )
        leng += sprintf( buf + leng, "battery_power_min=%d\n", battery_power_min );
    if ( leng < count - 32 )
        leng += sprintf( buf + leng, "battery_power_max=%d\n", battery_power_max );
    if ( leng < count - 32 )
        leng += sprintf( buf + leng, "sigpwr_holdoff=%d\n", sigpwr_holdoff );
    if ( leng < count - 32 )
        leng += sprintf( buf + leng, "sigpwr_next=%d\n", sigpwr_next );

    *eof = 1;
    return leng;
}


static int proc_get_temp_cb( char* buf, char** start, off_t offset,
                            int count, int* eof, void* data )
{
    struct tsc2100_data* d = ( struct tsc2100_data* ) data;
    int t1  = ringbeg(&d->ring_temp1);
    int t2  = ringbeg(&d->ring_temp2);
    int cnt = ringcnt(&d->ring_temp1);
    int leng = 0;

    while ( leng < count - 32 && cnt != 0 ) {
        leng += sprintf( buf + leng, "%d: ", cnt-- );
        leng += sprintf( buf + leng, "%d ", ringread(&d->ring_temp1,&t1) );
        leng += sprintf( buf + leng, "%d\n", ringread(&d->ring_temp2,&t2) );
    }
    *eof = 1;
    return leng;
}


static int proc_get_analog_mute_cb( char* buf, char** start, off_t offset,
                                    int count, int* eof, void* data )
{
    unsigned short regval = tsc2100_regread(TSC2100_REG_SIDETONE);

    *eof = 1;
    return sprintf( buf, "%d\n", ( regval >> 15 ) & 1 );
}


static int proc_set_analog_mute_cb( struct file* file, const char* buf,
                                    unsigned long count, void* data )
{
    unsigned short regval = tsc2100_regread(TSC2100_REG_SIDETONE);
    u32  value;
    int  rc = getval_from_user(file,buf,count,&value);
    if ( rc )
        return rc;
    regval = (regval & ~0x8000) | ((value & 1) << 15);
    tsc2100_devdata->reg.sidetone = regval;
    tsc2100_regwrite(TSC2100_REG_SIDETONE,tsc2100_devdata->reg.sidetone);
    return count;
}


static int proc_get_analog_gain_cb( char* buf, char** start, off_t offset,
                                    int count, int* eof, void* data )
{
    unsigned short regval = tsc2100_regread(TSC2100_REG_SIDETONE);

    *eof = 1;
    return sprintf( buf, "%d\n", ( regval >> 8 ) & 0x7F );
}


static int proc_set_analog_gain_cb( struct file* file, const char* buf,
                                    unsigned long count, void* data )
{
    unsigned short regval = tsc2100_regread(TSC2100_REG_SIDETONE);
    u32  value;
    int  rc = getval_from_user(file,buf,count,&value);
    if ( rc )
        return rc;
    regval = (regval & ~0x7F00) | ((value & 0x7F) << 8);
    tsc2100_devdata->reg.sidetone = regval;
    tsc2100_regwrite(TSC2100_REG_SIDETONE,tsc2100_devdata->reg.sidetone);
    return count;
}


static int proc_get_digital_mute_cb( char* buf, char** start, off_t offset,
                                    int count, int* eof, void* data )
{
    unsigned short regval = tsc2100_regread(TSC2100_REG_SIDETONE);

    *eof = 1;
    return sprintf( buf, "%d\n", ( regval >> 7 ) & 1 );
}


static int proc_set_digital_mute_cb( struct file* file, const char* buf,
                                    unsigned long count, void* data )
{
    unsigned short regval = tsc2100_regread(TSC2100_REG_SIDETONE);
    u32  value;
    int  rc = getval_from_user(file,buf,count,&value);
    if ( rc )
        return rc;
    regval = (regval & ~0x0080) | ((value & 1) << 7);
    tsc2100_devdata->reg.sidetone = regval;
    tsc2100_regwrite(TSC2100_REG_SIDETONE,tsc2100_devdata->reg.sidetone);
    return count;
}


static int proc_get_digital_gain_cb( char* buf, char** start, off_t offset,
                                    int count, int* eof, void* data )
{
    unsigned short regval = tsc2100_regread(TSC2100_REG_SIDETONE);

    *eof = 1;
    return sprintf( buf, "%d\n", ( regval >> 1 ) & 0x3F );
}


static int proc_set_digital_gain_cb( struct file* file, const char* buf,
                                    unsigned long count, void* data )
{
    unsigned short regval = tsc2100_regread(TSC2100_REG_SIDETONE);
    u32  value;
    int  rc = getval_from_user(file,buf,count,&value);
    if ( rc )
        return rc;
    regval = (regval & ~0x007E) | ((value & 0x3F) << 1);
    tsc2100_devdata->reg.sidetone = regval;
    tsc2100_regwrite(TSC2100_REG_SIDETONE,tsc2100_devdata->reg.sidetone);
    return count;
}


static int proc_get_pollinterval_cb( char* buf, char** start, off_t offset,
                                     int count, int* eof, void* data )
{
    *eof = 1;
    return sprintf( buf, "%d\n", poll_interval );
}


static int proc_set_pollinterval_cb( struct file* file, const char* buf,
                                     unsigned long count, void* data )
{
    u32  value;
    int  rc = getval_from_user(file,buf,count,&value);
    if ( rc )
        return rc;

    if (value == 0)
        value = 1;

    poll_interval = value;
    return count;
}


static int proc_get_dc_power_cb( char* buf, char** start, off_t offset,
                                 int count, int* eof, void* data )
{
    struct tsc2100_data* d = ( struct tsc2100_data* ) data;
    *eof = 1;
    return sprintf( buf, "%d\n", average_dc_power(d) >= dc_power_min );
}


static int proc_get_battery_power_cb( char* buf, char** start, off_t offset,
                                      int count, int* eof, void* data )
{
    int avg = average_battery_power(( struct tsc2100_data* ) data);
    *eof = 1;
    return sprintf( buf, "%d\n", avg >= battery_power_min && avg <= battery_power_max );
}


static int proc_get_sigpwr_pid_cb( char* buf, char** start, off_t offset,
                                   int count, int* eof, void* data )
{
    *eof = 1;
    return sprintf( buf, "%d\n", sigpwr_pid );
}


static int proc_set_sigpwr_pid_cb( struct file* file, const char* buf,
                                   unsigned long count, void* data )
{
    u32  value;
    int  rc = getval_from_user(file,buf,count,&value);
    if ( rc )
        return rc;

    if (value == 0)
        value = 1;

    sigpwr_pid = value;
    return count;
}


#define VOID_DIR        -1
#define ROOT_DIR        0
#define CHUMBY_DIR      1
#define TOUCHSCREEN_DIR 2
#define AUDIO_DIR       3
#define MIXER_DIR       4
#define LEFTSPKR_DIR    5
#define RIGHTSPKR_DIR   6
#define BOTHSPKR_DIR    7
#define SIDETONE_DIR    8
#define TSC2100_DIR     9

static struct {
    char* name;
    int   parent;
} procdir[] = {
/*0*/    { "proc-root",     VOID_DIR },
/*1*/    { "chumby",        ROOT_DIR },
/*2*/    { "touchscreen",   CHUMBY_DIR },
/*3*/    { "audio",         CHUMBY_DIR },
/*4*/    { "mixer",         AUDIO_DIR },
/*5*/    { "left-speaker",  MIXER_DIR },
/*6*/    { "right-speaker", MIXER_DIR },
/*7*/    { "both-speakers", MIXER_DIR },
/*8*/    { "side-tone",     AUDIO_DIR },
/*9*/    { "tsc2100",       CHUMBY_DIR },
};


static struct {
    char* name;
    int   parent;
    int (*read_proc)( char* buf, char** start, off_t offset,
                      int count, int* eof, void* data );
    int (*write_proc)( struct file* file, const char* buf,
                       unsigned long count, void* data );
    int offset;
} procfile[] = {
 { "battery-voltage", CHUMBY_DIR,      proc_get_batt_cb,         NULL, 0 },
 { "temperature",     CHUMBY_DIR,      proc_get_temp_cb,         NULL, 0 },
 { "enable",          TOUCHSCREEN_DIR, proc_get_tson_cb,         proc_set_tson_cb, 0 },
 { "coordinates",     TOUCHSCREEN_DIR, proc_get_coords_cb,       NULL, 0 },
 { "poll-interval",   TOUCHSCREEN_DIR, proc_get_pollinterval_cb, proc_set_pollinterval_cb, 0 },
 { "touchclick",      TOUCHSCREEN_DIR, proc_get_touchclick_cb,   proc_set_touchclick_cb, 0 },
 { "mute",            LEFTSPKR_DIR,    proc_get_left_mute_cb,    proc_set_left_mute_cb, 0 },
 { "mute",            RIGHTSPKR_DIR,   proc_get_right_mute_cb,   proc_set_right_mute_cb, 0 },
 { "mute",            BOTHSPKR_DIR,    proc_get_both_mute_cb,    proc_set_both_mute_cb, 0 },
 { "volume",          LEFTSPKR_DIR,    proc_get_left_volume_cb,  proc_set_left_volume_cb, 0 },
 { "volume",          RIGHTSPKR_DIR,   proc_get_right_volume_cb, proc_set_right_volume_cb, 0 },
 { "volume",          BOTHSPKR_DIR,    proc_get_both_volume_cb,  proc_set_both_volume_cb, 0 },
 { "analog-mute",     SIDETONE_DIR,    proc_get_analog_mute_cb,  proc_set_analog_mute_cb, 0 },
 { "analog-gain",     SIDETONE_DIR,    proc_get_analog_gain_cb,  proc_set_analog_gain_cb, 0 },
 { "digital-mute",    SIDETONE_DIR,    proc_get_digital_mute_cb, proc_set_digital_mute_cb, 0 },
 { "digital-gain",    SIDETONE_DIR,    proc_get_digital_gain_cb, proc_set_digital_gain_cb, 0 },
 { "registers",       TSC2100_DIR,     proc_get_tsregs_cb,       NULL, 0 },
 { "adc-page1",       TSC2100_DIR,     proc_get_adc_cb,          proc_set_adc_cb, 0 },
 { "refctl-page1",    TSC2100_DIR,     proc_get_refctl_cb,       proc_set_refctl_cb, 0 },
 { "control1-page2",  TSC2100_DIR,     proc_get_control1_cb,     proc_set_control1_cb, 0 },
 { "audioadc-page2",  TSC2100_DIR,     proc_get_audioadc_cb,     proc_set_audioadc_cb, 0 },
 { "audiodac-page2",  TSC2100_DIR,     proc_get_audiodac_cb,     proc_set_audiodac_cb, 0 },
 { "sidetone-page2",  TSC2100_DIR,     proc_get_sidetone_cb,     proc_set_sidetone_cb, 0 },
 { "control2-page2",  TSC2100_DIR,     proc_get_control2_cb,     proc_set_control2_cb, 0 },
 { "powercon-page2",  TSC2100_DIR,     proc_get_powercon_cb,     proc_set_powercon_cb, 0 },
 { "control3-page2",  TSC2100_DIR,     proc_get_control3_cb,     proc_set_control3_cb, 0 },
 { "pll1-page2",      TSC2100_DIR,     proc_get_pll1_cb,         proc_set_pll1_cb, 0 },
 { "pll2-page2",      TSC2100_DIR,     proc_get_pll2_cb,         proc_set_pll2_cb, 0 },
 { "control4-page2",  TSC2100_DIR,     proc_get_control4_cb,     proc_set_control4_cb, 0 },
 { "control5-page2",  TSC2100_DIR,     proc_get_control5_cb,     proc_set_control5_cb, 0 },
 { "dc-power",        CHUMBY_DIR,      proc_get_dc_power_cb,     NULL, 0 },
 { "battery-power",   CHUMBY_DIR,      proc_get_battery_power_cb,NULL, 0 },
 { "sigpwr-pid",      CHUMBY_DIR,      proc_get_sigpwr_pid_cb,   proc_set_sigpwr_pid_cb, 0 }
};


static void __exit tsc2100_proc_rmdir(struct tsc2100_data* d)
{
    int i;

    for ( i = ARRAY_SIZE(procdir)-1; i != 0; --i ) {
        if (d->proc_dirs[i]) {
            remove_proc_entry( procdir[i].name, d->proc_dirs[procdir[i].parent] );
            d->proc_dirs[i] = NULL;
        }
    }
}


static void __exit tsc2100_proc_exit(struct tsc2100_data* d)
{
	int i;

    for ( i = 0; i < ARRAY_SIZE(procfile); ++i ) {
        if (d->proc_files[i]) {
            remove_proc_entry( procfile[i].name, d->proc_dirs[procfile[i].parent] );
            d->proc_files[i] = NULL;
        }
    }

    tsc2100_proc_rmdir(d);
}


static int __init tsc2100_proc_mkdir(struct tsc2100_data* d)
{
    int i;

    for ( i = 1; i < ARRAY_SIZE(procdir); ++i ) {
        d->proc_dirs[i] = proc_mkdir(procdir[i].name,
                                     d->proc_dirs[procdir[i].parent]);
        if (d->proc_dirs[i] == NULL) {
            tsc2100_proc_rmdir(d);
            return -1;
        }
    }

    return 0;
}


static int __init tsc2100_proc_init(struct tsc2100_data* d)
{
    int i;

    if (ARRAY_SIZE(d->proc_dirs) < ARRAY_SIZE(procdir)) {
        printk( KERN_ERR "%s(): ARRAY_SIZE(d->proc_dirs)[%d] < ARRAY_SIZE(procdir)[%d]\n",
                __FUNCTION__, ARRAY_SIZE(d->proc_dirs), ARRAY_SIZE(procdir) );
        return -1;
    }
    if (ARRAY_SIZE(d->proc_files) < ARRAY_SIZE(procfile)) {
        printk( KERN_ERR "%s(): ARRAY_SIZE(d->proc_files)[%d] < ARRAY_SIZE(procfile)[%d]\n",
                __FUNCTION__, ARRAY_SIZE(d->proc_files), ARRAY_SIZE(procfile) );
        return -1;
    }

    if ( tsc2100_proc_mkdir(d) )
        return -1;

    for ( i = 0; i < ARRAY_SIZE(procfile); ++i ) {
        struct proc_dir_entry* entry;
        void* data = (void*)(((char*) d) + procfile[i].offset);

        entry = proc_file( procfile[i].name, d->proc_dirs[procfile[i].parent],
                           procfile[i].read_proc, procfile[i].write_proc,
                           data );
        if ( entry == NULL ) {
            tsc2100_proc_exit(d);
            return -1;
        }

        d->proc_files[i] = entry;
    }

    return 0;
}


static int __init tsc2100_touchscreen_init(void)
{
    struct device  tsc2100dev;
    struct tsc2100_data *devdata;
    struct input_dev *input_dev;
    struct tsc2100_ts_event ts_data;
    int rc;

    devdata = kzalloc(sizeof(struct tsc2100_data), GFP_KERNEL);
    input_dev = input_allocate_device();
    if (!devdata || !input_dev) {
        input_free_device(input_dev);
        kfree(devdata);
        return -ENOMEM;
    }

    spin_lock_init(&devdata->lock);

    dev_set_drvdata(&tsc2100dev, devdata);
    devdata->inputdevice = input_dev;

    tsc2100_devdata = devdata;
    devdata->reg.adc      = reg_adc;
    devdata->reg.refctl   = reg_refctl;
    devdata->reg.control1 = reg_control1;
    devdata->reg.audioadc = reg_audioadc;
    devdata->reg.audiodac = reg_audiodac;
    devdata->reg.sidetone = reg_sidetone;
    devdata->reg.control2 = reg_control2;
    devdata->reg.powercon = reg_powercon;
    devdata->reg.control3 = reg_control3;

    devdata->state = TS_IDLE;

    if( CSCR & CSCR_MCU_SEL ) { // detect clock version by looking at where CPU gets its clock
      // we are in 16 MHz land (hardware version 1.6 and higher)
      reg_pll1        = 1 << 15  |    // enable PLL
                     	1 << 8   |    // P value is 1 for 48.0 kHz
                  	6 << 2;       // J value is 6 for 48.0 kHz
      reg_pll2        = 1440<<2;      // D value is 1440 for 48.0 kHz
    } else {
      // leave at default values
    }
    devdata->reg.pll1     = reg_pll1;
    devdata->reg.pll2     = reg_pll2;
    devdata->reg.control4 = reg_control4;
    devdata->reg.control5 = reg_control5;

    devdata->tson = 1;
    devdata->touchclick = 1;

    init_timer(&devdata->ts_timer);
    devdata->ts_timer.data = (unsigned long) devdata;
    devdata->ts_timer.function = tsc2100_timer;

    rc = request_irq(PENIRQ, touchscreen_handler, 0, "tsc2100", devdata);
    if (rc) {
        printk(KERN_ERR "tsc2100: Could not allocate touchscreen IRQ %d!\n",
               PENIRQ);
    	tsc2100_devdata  = NULL;
        kfree(devdata);
        return -EINVAL;
    }

    gpio_init();
    spi_init();

    tsc2100_touchscreen_setup(&tsc2100dev);
    tsc2100_touchscreen_reset(devdata);
    tsc2100_touchscreen_enable(devdata);

    /* Falling edge indicates PEN has been pressed and a new X/Y/Z1/Z2 sample
     * is now ready (note we read the data here simply to flush and discard).
     */
    set_irq_type(PENIRQ, IRQT_FALLING);
    tsc2100_readdata(devdata, &ts_data);

    mod_timer(&(devdata->ts_timer), jiffies + HZ / 100);

    return 0;
}


static int __init tsc2100_alsa_audio_init(void)
{
    int rc = -ENODEV;
    snd_card_t *card;
    tsc2100_chip_t *chip;

    printk( KERN_DEBUG
            "%s/%s(): playback buffer_bytes_max=0x%x, period_bytes_min=0x%x, "
            "period_bytes_max=0x%x, periods_min=0x%x, periods_max=0x%x\n",
            __FILE__, __FUNCTION__, playback_buffer_bytes_max, 
            playback_period_bytes_min, playback_period_bytes_max, 
            playback_periods_min, playback_periods_max);

    /* Register the sound card */
    card = snd_card_new(-1, id, THIS_MODULE, sizeof(tsc2100_chip_t));
    if (card == NULL) {
        return -ENOMEM;
    }

    chip = (tsc2100_chip_t *)card->private_data;
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
        AUDMUX_HPCR_RXDSEL(3));   /* Peripheral port 1 */

    AUDMUX_PPCR1 = (
        AUDMUX_PPCR_TFCSEL(0) |   /* Host port 1 */
        AUDMUX_PPCR_RFCSEL(0) |   /* Host port 1 */
	    AUDMUX_PPCR_SYN);

    AUDMUX_PPCR2 = (
        AUDMUX_PPCR_TFCSEL(0) |   /* Host port 1 */
        AUDMUX_PPCR_RFCSEL(0) |   /* Host port 1 */
        AUDMUX_PPCR_SYN);

    /* Enable the clock to SSI1 */
    PCCR0 |= (PCCR0_SSI1_BAUD_EN | PCCR0_SSI1_EN);

    if (imx_dma_request_by_prio(
            &chip->p_dma, "ALSA-PLAYBACK", DMA_PRIO_HIGH) < 0) {
        goto init_err0;
    }
    imx_dma_setup_handlers(chip->p_dma, tsc2100_dma_play_isr, NULL, chip);

    if (imx_dma_request_by_prio(
            &chip->c_dma, "ALSA-CAPTURE", DMA_PRIO_HIGH) < 0) {
        goto init_err0;
    }
    imx_dma_setup_handlers(chip->c_dma, tsc2100_dma_capture_isr, NULL, chip);

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

    if ((rc = snd_tsc2100_mixer_new(chip)) != 0) goto init_err1;
    if ((rc = snd_tsc2100_pcm_new(chip)) != 0)   goto init_err1;
    if ((rc = snd_card_register(card)) != 0)     goto init_err1;

    pll_fsref_48000(0,0);
    tsc2100_regwrite(TSC2100_REG_SIDETONE,tsc2100_devdata->reg.sidetone);
    tsc2100_regwrite(TSC2100_REG_CONTROL4,tsc2100_devdata->reg.control4);
    tsc2100_regwrite(TSC2100_REG_CONTROL5,tsc2100_devdata->reg.control5);
    tsc2100_regwrite(TSC2100_REG_AUDIOADC,tsc2100_devdata->reg.audioadc);
    tsc2100_regwrite(TSC2100_REG_AUDIODAC,tsc2100_devdata->reg.audiodac);

    printk(KERN_INFO "Chumby TI-TSC2100 ALSA Audio Driver initialized (PUEN)\n");
    printk(KERN_INFO "touchscreen timeout_workaround=%d timeout_regdump=%d\n", timeout_workaround, timeout_regdump);
    tsc2100_sndcard = card;
    return 0;

 init_err1:
    printk( "%s(): failed init_err1\n", __FUNCTION__ );
    imx_dma_free(chip->p_dma);
 init_err0:
    printk( "%s(): failed init_err0, rc=%d\n", __FUNCTION__, rc );
    snd_card_free(card);
    return rc;
}


static void __exit tsc2100_touchscreen_exit(void)
{
    if ( tsc2100_devdata ) {
        free_irq(PENIRQ, tsc2100_devdata);
        del_timer_sync(&tsc2100_devdata->ts_timer);
        del_timer_sync(&tsc2100_devdata->misc_timer);
        input_unregister_device(tsc2100_devdata->inputdevice);
        tsc2100_touchscreen_disable(tsc2100_devdata);
        spi_exit();
        kfree(tsc2100_devdata);
        tsc2100_devdata = NULL;
    }
}


static void __exit tsc2100_alsa_audio_exit(void)
{
    if ( tsc2100_sndcard ) {
        tsc2100_chip_t *chip = (tsc2100_chip_t *)tsc2100_sndcard->private_data;
        imx_dma_free(chip->p_dma);
        imx_dma_free(chip->c_dma);
        snd_card_free(tsc2100_sndcard);
        tsc2100_sndcard = NULL;
    }
}


static void __exit chumby_tsc2100_exit(void)
{
    tsc2100_proc_exit(tsc2100_devdata);
    tsc2100_alsa_audio_exit();
    tsc2100_touchscreen_exit();
    printk(KERN_INFO "Chumby TI-TSC2100 ALSA Audio and TouchScreen Driver removed\n");
}


static int __init chumby_tsc2100_init(void)
{
    int rc;

    rc = tsc2100_touchscreen_init();
    if ( rc )
        return rc;
    rc = tsc2100_alsa_audio_init();
    if ( rc )
        tsc2100_touchscreen_exit();
    else
        tsc2100_proc_init(tsc2100_devdata);
    return rc;
}


module_init(chumby_tsc2100_init);
module_exit(chumby_tsc2100_exit);
