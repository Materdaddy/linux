/*
 *  Driver for Chumby (iMX21) TSC2100, both ALSA audio and TouchScreen
 *  Copyright (C) 2006 Jay Monkman <jtm@lopingdog.com>
 *                2007 Greg Hutchins <ghutchins@gmail.com>
 *                2008 Greg Hutchins <ghutchins@gmail.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License.
 *
 *  NOTES:
 *    This driver configures the CODEC to generate the clock and frame signals.
 *
 *    2008 version of the driver completely revamped to fix a number of 
 *    deficiencies:
 *        1. interrupts disabled in both the interrupt handler and timer
 *           handler for extended period of time (the entire routine).
 *        2. voltage measurement state machine was kludgey in previous version.
 *        3. TSC2100-controlled mode was used to capture X/Y/Z data and 
 *           host-controlled mode was used to capture BAT/TEMP data.  Switching
 *           the TSC2100 between the two modes seems to occasionally put the 
 *           TSC100 into a funny state where it no longer scans.
 *        4. touchscreen events would stop working, presumably because of #3
 *           above.
 *        5. would play games with the interrupt type/level, switching from 
 *           FALLING-EDGE to LEVEL-TRIGGERED (may have caused problems)
 *
 *    How were these problems resolved?
 *        1. both interrupts and timer now schedule the same tasklet, so no
 *           longer need to protect the code with IRQ SPIN LOCK  -- the only
 *           IRQ SPIN LOCK is used to protect the SPI commands, making them
 *           atomic so that both the AUDIO and TOUCHSCREEN sub-systems can use
 *           them independently.
 *        2. changed the way the state machine works, now have a scan_mode and
 *           a scan_state.  The timer routine initiates a scan and the interrupt
 *           (or oneshot timer) processes the result of the scan and steps to 
 *           the next state in the state machine. 
 *        3. TSC2100 now *ALWAYS* operates in host-controlled mode, so once the
 *           PEN is down the X/Y coordinates must be scanned independently
 *        4. always operate in host-controlled mode and put in some checks in
 *           the state machine to detect problems where the TSC2100 stops
 *           operating correctly
 *        5. now always leave the IRQ as FALLING-EDGE (we control the number of
 *           interrupts by manually specifying the X and Y SCAN conversations)
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

#include <asm/mach/arch.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>

#ifdef MAKE_L22 // needed to compile with linux 2.6.22+ compatibility
#include <sound/typedefs.h>
#endif
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/control.h>
#include <sound/initval.h>


//#define PLAY_DEBUG_SOUND
#ifdef PLAY_DEBUG_SOUND
static unsigned char* debug_sound_buf = NULL;
static int debug_sound_last = 0;
static int debug_sound_next = 0;
#endif


//#define DEBUG
#ifdef DEBUG
extern int dbug_printk( const char* fmt, ... );

#define DBG(fmt, args...)                                                      \
    do {                                                                       \
        struct timeval _now_;                                                  \
        do_gettimeofday(&_now_);                                               \
        dbug_printk("%ld.%06ld-%s().%d: " fmt,                                 \
                _now_.tv_sec, _now_.tv_usec, __FUNCTION__, __LINE__, ## args); \
    } while ( 0 )

#else
#define DBG(stuff...) do {} while(0)
#endif

#if 1
#define DBG_PEN(fmt, args...) DBG(fmt, args)
#else
#define DBG_PEN(fmt, args...) do {} while (0)
#endif

#if 0
#define DBG_SCAN(fmt, args...) DBG(fmt, args)
#else
#define DBG_SCAN(stuff...) do {} while (0)
#endif

#if 1
#define ADBG(fmt, args...) DBG(fmt, args)
#else
#define ADBG(stuff...) do {} while(0)
#endif

#define ERR(fmt, args...)   \
            printk(KERN_ERR "ERROR!!! %s/%s().%d: " fmt, \
                   __FILE__, __FUNCTION__, __LINE__, ## args)

#include "chumby-tsc2100.h"


/***********************************************************************
 * Module stuff
 ***********************************************************************/
static char *id = NULL;
module_param(id, charp, 0444);


/***********************************************************************
 * Constants
 ***********************************************************************/
//#define FILL_WITH_SILENCE   1            // play ISR fills ring buf with silence
#define SYSTEM_LATENCY      4            // 44.1Khz 16-bit stereo (22.7us delay)
#define BUFFER_BYTES_MAX    (128 * 1024) // big mofo buffer (64K is probably more than enough)
#define PERIOD_BYTES_MIN    (8 * 1024)   // based on Chumby iMX21 system latency
#define PERIODS_MIN         4            // based on ALSA ring buffer math (must be at least 3)
#define PERIODS_MAX         (BUFFER_BYTES_MAX / PERIOD_BYTES_MIN)
#define PERIOD_BYTES_MAX    (BUFFER_BYTES_MAX / PERIODS_MIN)


// TouchScreen Constants
#define ADCMODE(mode)  (TSC2100_ADC_DEFAULT | TSC2100_ADC_ADMODE(mode))

#define X_AXIS_MAX            3830
#define X_AXIS_MIN            150
#define Y_AXIS_MAX            3830
#define Y_AXIS_MIN            190
#define PRESSURE_MIN          0
#define PRESSURE_MAX          20000

#define SCAN_TIMEOUT          3     // # of jiffies before timeout 
#define MIN_TIMER_INTERVAL    100   // minimum msec interval for SCAN timer

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
    u32 p_prevpos;
    u32 p_ccnr;
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
int version038      = 0;            // non-zero if version 3.8 hardware
u16 reg_adc         = ADCMODE(0);
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

int timer_interval  = 1000;         // timer poll interval in milliseconds
u32 timer_interval_jiffies;         // msecs_to_jiffies(timer_interval)
int unplugged_battery_poll_interval = 4000; // msecs between unplugged battery polls
int pluggedin_battery_poll_interval = 4000; // msecs between pluggedin battery polls
int dcline_poll_interval = 4000;    // msecs between DCLINE voltage polls
int battery_poll_next = 0;          // measure battery voltage when this is 0,
                                    // then reset to either unplugged_battery_poll_interval 
                                    // or pluggedin_battery_poll_interval
                                    // depending on the "unplugged" state
int dcline_poll_next = 0;           // measure dcline voltage when this is 0,
                                    // then reset to dcline_poll_interval
int voltage_sample_count = 5;       // max # of samples to average (low pass filter)
int voltage_samples = 0;            // current sample number 
int unplugged = 0;                  // if non-zero, chumby is unplugged
int dc_unplugged_threshold = 170;   // chumby unplugged if less than this value 
int dc_and_battery_dead_threshold = -1;// when plugged into wall, battery dead if less than this value
int battery_dead_threshold = -1;    // battery dead if less than this value
int raw9volts = -1;                 // raw voltage value representing 9 volts
int raw10volts = -1;                // because of bleed through -- 
                                    // used to determine if battery present
static int sigpwr_next = 0;         // # of secs to wait before next SIGPWR sig
static int sigpwr_holdoff = 30;     // # of secs to wait before generating
                                    // another SIGPWR signal
static int sigpwr_pid = 0;          // PID to send SIGPWR signal to when line
                                    // power is lost (not battery power)

module_param(version038, int, S_IRUGO|S_IWUSR);
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

module_param(timer_interval, int, S_IRUGO|S_IWUSR);
module_param(unplugged_battery_poll_interval, int, S_IRUGO|S_IWUSR);
module_param(pluggedin_battery_poll_interval, int, S_IRUGO|S_IWUSR);
module_param(dcline_poll_interval, int, S_IRUGO|S_IWUSR);
module_param(voltage_sample_count, int, S_IRUGO|S_IWUSR);
module_param(dc_unplugged_threshold, int, S_IRUGO|S_IWUSR);
module_param(dc_and_battery_dead_threshold, int, S_IRUGO|S_IWUSR);
module_param(battery_dead_threshold, int, S_IRUGO|S_IWUSR);
module_param(raw9volts, int, S_IRUGO|S_IWUSR);
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
struct tsc2100_context      *tsc2100_devdata = NULL;


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


#if 0
static unsigned long microseconds( void )
{
    extern struct sys_timer *system_timer;
    unsigned long flags;
    unsigned long seq;
    unsigned long usecs;

    do {
        seq = read_seqbegin_irqsave(&xtime_lock, flags);
        usecs = jiffies_to_usecs(jiffies) + system_timer->offset();
    } while (read_seqretry_irqrestore(&xtime_lock, seq, flags));

    return usecs;
}
#endif


static void gpio_init( void )
{
    imx_gpio_write( 30 | GPIO_PORTC, 0 );  // turn off battery voltage gate
    imx_gpio_mode( 30 | GPIO_PORTC | GPIO_GPIO | GPIO_OUT );
    if (version038) {
        imx_gpio_mode( 28 | GPIO_PORTC | GPIO_GPIO | GPIO_OUT );
        imx_gpio_write( 28 | GPIO_PORTC, 0 );  // turn off line voltage gate
    }

    imx_gpio_mode( 18 | GPIO_PORTE | GPIO_AF | GPIO_IN );
    imx_gpio_mode( 21 | GPIO_PORTE | GPIO_AF | GPIO_OUT );
    imx_gpio_mode( 22 | GPIO_PORTE | GPIO_AF | GPIO_OUT );
    imx_gpio_mode( 23 | GPIO_PORTE | GPIO_AF | GPIO_OUT );
    imx_gpio_mode( 19 | GPIO_PORTE | GPIO_PF | GPIO_IN | GPIO_PUEN | GPIO_GPIO );

    if (version038) {
        imx_gpio_mode( PC23_PF_SSI1_CLK | GPIO_PUEN );
        imx_gpio_mode( PC22_PF_SSI1_TX | GPIO_PUEN  );
        imx_gpio_mode( PC21_PF_SSI1_RX | GPIO_PUEN  );
        imx_gpio_mode( PC20_PF_SSI1_FS | GPIO_PUEN  );
        imx_gpio_mode( PC27_PF_SSI2_CLK | GPIO_PUEN  );
        imx_gpio_mode( PC26_PF_SSI2_TX | GPIO_PUEN  );
        imx_gpio_mode( PC25_PF_SSI2_RX | GPIO_PUEN  );
        imx_gpio_mode( PC24_PF_SSI2_FS | GPIO_PUEN  );
    } else {
        imx_gpio_mode( PC23_PF_SSI1_CLK );
        imx_gpio_mode( PC22_PF_SSI1_TX );
        imx_gpio_mode( PC21_PF_SSI1_RX );
        imx_gpio_mode( PC20_PF_SSI1_FS );
        imx_gpio_mode( PC27_PF_SSI2_CLK );
        imx_gpio_mode( PC26_PF_SSI2_TX );
        imx_gpio_mode( PC25_PF_SSI2_RX );
        imx_gpio_mode( PC24_PF_SSI2_FS );
    }

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

static void pll_fsref_48000( int N )
{
    u16 val = tsc2100_devdata->reg.control1 & ~0x3F;
    val |= ( ( N & 7 ) << 3 ) | ( N & 7 );
    tsc2100_devdata->reg.control1 = val;
    tsc2100_devdata->reg.control3 &= ~( 1 << 13 );
    tsc2100_devdata->dac_sample_rate = N;
    tsc2100_devdata->adc_sample_rate = N;
    tsc2100_regwrite(TSC2100_REG_CONTROL1, tsc2100_devdata->reg.control1);
    tsc2100_regwrite(TSC2100_REG_CONTROL3, tsc2100_devdata->reg.control3);

    // check to see if external crystal is being used on CPU
    if( CSCR & CSCR_MCU_SEL ) { // detect clock version by looking at where CPU gets its clock
      // we are in 16 MHz land (hardware version 1.6 and higher)
      // K/P = 6.144
      tsc2100_regwrite(TSC2100_REG_PLL1, 1 << 15 |  // enable PLL
		                                 1 << 8  |  // P value is 1 for 48.0 kHz
		                                 6 << 2);   // J value is 6 for 48.0 kHz
      tsc2100_regwrite(TSC2100_REG_PLL2, 1440<<2);  // D value 1440 for 48.0 kHz
    } else {
      // we are in 12 MHz land (hardware version 1.5 and less)
      tsc2100_regwrite(TSC2100_REG_PLL1, 1 << 15 |  // enable PLL
		                                 1 << 8  |  // P value is 1 for 48.0 kHz
		                                 8 << 2);   // J value is 8 for 48.0 kHz
      tsc2100_regwrite(TSC2100_REG_PLL2, 1920<<2);  // D value 1920 for 48.0 kHz
    }
}


static void pll_fsref_44100( int N )
{
    u16 val = tsc2100_devdata->reg.control1 & ~0x3F;
    val |= ( ( N & 7 ) << 3 ) | ( N & 7 );
    tsc2100_devdata->reg.control1 = val;
    tsc2100_devdata->reg.control3 |= ( 1 << 13 );
    tsc2100_devdata->dac_sample_rate = N;
    tsc2100_devdata->adc_sample_rate = N;
    tsc2100_regwrite(TSC2100_REG_CONTROL1, tsc2100_devdata->reg.control1);
    tsc2100_regwrite(TSC2100_REG_CONTROL3, tsc2100_devdata->reg.control3);

    if( CSCR & CSCR_MCU_SEL ) { // detect clock version by looking at where CPU gets its clock
      // we are in 16 MHz land (hardware version 1.6 and higher)
      // K/P = 5.6448
      tsc2100_regwrite(TSC2100_REG_PLL1, 1 << 15 |  // enable PLL
		                                 1 << 8  |  // P value is 1 for 44.1 kHz
		                                 5 << 2);   // J value is 5 for 44.1 kHz
      tsc2100_regwrite(TSC2100_REG_PLL2, 6448<<2);  // D value 6448 for 44.1 kHz
    } else {
      // we are in 12 MHz land (hardware version 1.5 and less)
      tsc2100_regwrite(TSC2100_REG_PLL1, 1 << 15 |  // enable PLL
		                                 1 << 8  |  // P value is 1 for 44.1 kHz
		                                 7 << 2);   // J value is 7 for 44.1 kHz
      tsc2100_regwrite(TSC2100_REG_PLL2, 5264<<2);  // D value 5264 for 44.1 kHz
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
        printk( KERN_ERR "%s/%s(): Error setting rates constraint, rc %d\n", 
                __FILE__, __FUNCTION__, rc);
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

    printk( KERN_DEBUG "%s(): rate=%d\n", __FUNCTION__, runtime->rate); 

    SSI1_STCCR = (SSI_STCCR_WL(16) | SSI_STCCR_DC(3) | SSI_STCCR_PM(0));
    SSI1_SRCCR = (SSI_SRCCR_WL(16) | SSI_SRCCR_DC(3) | SSI_SRCCR_PM(0));
    SSI1_STMSK = ~((1 << 0) | (1 << 2));
    SSI1_SRMSK = ~((1 << 0) | (1 << 2));

    switch(runtime->rate) {
    case 8000: pll_fsref_48000(7); break;
    case 11025:pll_fsref_44100(4); break;
    case 16000:pll_fsref_48000(3); break;
    case 22050:pll_fsref_44100(2); break;
    case 32000:pll_fsref_48000(1); break;
    case 44100:pll_fsref_44100(0); break;
    case 48000:pll_fsref_48000(0); break;

    default:
        printk( KERN_ERR "%s/%s(): Can't handle rate of %d\n", 
                __FILE__, __FUNCTION__, runtime->rate);
        return -ENODEV;
    }

   /* Get physical address of data buffer */
    runtime->dma_addr = __pa(runtime->dma_area);

    chip->p_pos = 0;
    chip->p_prevpos = -1;
    chip->p_period = 0;
    chip->p_per_size = snd_pcm_lib_period_bytes(substream);
    chip->p_buf_size = snd_pcm_lib_buffer_bytes(substream);
    ADBG("p_per_size=0x%x, p_buf_size=0x%x, periods=0x%x, dma_addr=0x%x\n", 
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
        ADBG("%s", "TRIGGER_START\n");
        enable_speaker();

        /* If ALSA wants to give us two periods, set them both up  */

        while (CCR(chip->p_dma) & CCR_CEN) {
            printk( KERN_ERR "%s/%s(): TRIGGER_START while DMA is already enabled\n",
                    __FILE__, __FUNCTION__ );
        }
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
            setup_dma_xfer(chip->p_dma,
                           runtime->dma_addr + (chip->p_period * chip->p_per_size),
                           (u32)&SSI1_STX0_PHYS,
                           chip->p_per_size);
            chip->p_period = (chip->p_period + 1) % runtime->periods;
        }
        break;
    case SNDRV_PCM_TRIGGER_STOP:
        ADBG("%s", "TRIGGER_STOP\n");
        disable_speaker();
        imx_dma_disable(chip->p_dma);
        CCR(chip->p_dma) &= ~(CCR_RPT | CCR_ACRPT);
        SSI1_SIER &= ~SSI_SIER_TDMAE;
        SSI1_SCR &= ~SSI_SCR_TE;
        break;
    default:
        printk(KERN_WARNING "%s/%s(): unknown command %d\n", 
               __FILE__, __FUNCTION__, cmd);
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
//
// The note above was written before I started debugging the Simon Game 
// audio glitches.  src/pcm/pcm_dmix.c:snd_pcm_dmix_start_timer() and
// snd_pcm_dmix_reset() (same module), actually uses more than period 
// granularity and will cause the system to overwrite AUDIO ring buffer if
// the correct slave_hw_ptr is not returned (by the following routine).
// 
// The code below may look a bit odd, the CHCNTR(chip->p_dma) can actually
// change to the next buffer *BEFORE* the DMA ISR routine services the interrupt
// because we are taking advantage of DMA chaining.
// So, we need to disable interrupts when sampling chip->p_pos (which is
// updated by the tsc2100_dma_play_isr() ISR routine below) and we need to
// verify CHCNTR(chip->p_dma) is relevant to the chip->p_pos period and *NOT*
// the next period.
// 
// The SYSTEM_LATENCY helps a minute amount, but I don't think it is really
// necessary, currently set to 4.  If it is set to 8 you will start to hear
// audio glitches.  (The bytes_to_frames() routine basically divides by 4
// in the standard Chumby configuration (44.1Khz, 16-bit stereo) so by setting
// SYSTEM_LATENCY to a 4 we are only padding the pointer by 1, about 22.7us
// delay.)

static snd_pcm_uframes_t snd_tsc2100_pcm_playback_pointer(
                                        snd_pcm_substream_t *substream)
{
    tsc2100_chip_t *chip = snd_pcm_substream_chip(substream);
    size_t          ppos;
    u32             ccnr;
    unsigned long   flags;

    local_irq_save(flags);
    ppos = chip->p_pos;
    ccnr = CHCNTR(chip->p_dma);
    if (chip->p_prevpos == ppos && ccnr < chip->p_ccnr)
        ccnr = chip->p_ccnr;
    else {
        chip->p_ccnr = ccnr;
        chip->p_prevpos = ppos;
    }
    local_irq_restore(flags);
    ppos = (ppos + ccnr + SYSTEM_LATENCY) % chip->p_buf_size;
    return bytes_to_frames(substream->runtime, ppos);
}


static inline void report_dma_errors( const char* filename, const char* funcname )
{
    if (DBTOSR != 0) 
        printk( KERN_ERR "%s/%s(): DMA Error: DBTOSR = 0x%x\n", 
                filename, funcname, DBTOSR);
    if (DRTOSR != 0) 
        printk( KERN_ERR "%s/%s(): DMA Error: DRTOSR = 0x%x\n", 
                filename, funcname, DRTOSR);
    if (DSESR != 0)  
        printk( KERN_ERR "%s/%s(): DMA Error: DSESR = 0x%x\n", 
                filename, funcname, DSESR);
    if (DBOSR != 0)  
        printk( KERN_ERR "%s/%s(): DMA Error: DBOSR = 0x%x\n", 
                filename, funcname, DBOSR);
}


static void tsc2100_dma_play_isr(int irq, void *data, struct pt_regs *regs)
{
    tsc2100_chip_t *chip = (tsc2100_chip_t *)data;
    snd_pcm_substream_t *substream = chip->p_substream;
    snd_pcm_runtime_t *runtime = substream->runtime;

    report_dma_errors( __FILE__, __FUNCTION__ );

    ADBG("%d: ack=%x--%x [SAR=%04x CNTR=%x]\n", 
         chip->p_period, chip->p_pos, chip->p_pos + chip->p_per_size-1,
         chip->p_period * chip->p_per_size, chip->p_per_size);

    setup_dma_xfer(chip->p_dma,
                   runtime->dma_addr + (chip->p_period * chip->p_per_size),
                   (u32)&SSI1_STX0_PHYS,
                   chip->p_per_size);
    CCR(chip->p_dma) |= CCR_RPT;

    #ifdef FILL_WITH_SILENCE
    memset( runtime->dma_area + (chip->p_pos % chip->p_buf_size), 0, 
            chip->p_per_size );
    #endif

    chip->p_period   = (chip->p_period + 1) % runtime->periods;
    chip->p_prevpos  = chip->p_pos;
    chip->p_pos     += chip->p_per_size;

    #ifdef PLAY_DEBUG_SOUND
    if (debug_sound_buf && debug_sound_last >= chip->p_per_size) {
        memcpy( runtime->dma_area + (chip->p_pos % chip->p_buf_size),
                debug_sound_buf + debug_sound_next,
                chip->p_per_size );
        debug_sound_next += chip->p_per_size;
        if (debug_sound_next > debug_sound_last) {
            // just keeps playing the sound over and over -- can only stop
            // by disabling through the /proc file system
            debug_sound_next = 0;
        }
    }
    #endif

    // wait until the next DMA starts to get CHCNTR(chip->p_dma) into the
    // next period so we can set chip->p_ccnr to a valid intitial value  --
    // at 22.7us/sample this should not take long.  Cannot put this code
    // *AFTER* the snd_pcm_period_elapsed() call below because that code calls
    // the snd_tsc2100_pcm_playback_pointer() routine which relies on 
    // chip->p_ccnr.  However, we put this code at the end of the handler so
    // hopefully it will not have to wait long for the CHCNTR(chip->p_dma) to
    // be within the next period.
    {
        int i, tout = 1000;

        if (runtime->rate <= 16000)
            tout *= 4;

        for (i=0; CHCNTR(chip->p_dma) > chip->p_per_size/2 && i < tout; ++i) {
            ADBG("%s(): %d -- CHCNTR(%d)=%x\n", 
                 __FUNCTION__, i, chip->p_dma, CHCNTR(chip->p_dma));
            continue;
        }
        if (i == tout) {
            printk(KERN_ERR "%s/%s(): timed-out wating for DMA to start.\n",
                   __FILE__, __FUNCTION__);
        }
        chip->p_ccnr = CHCNTR(chip->p_dma);
    }

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
        printk( KERN_ERR "%s/%s(): Error setting rates constraint\n",
                __FILE__, __FUNCTION__ );
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

    printk( KERN_DEBUG "%s(): rate=%d\n", __FUNCTION__, runtime->rate); 

    SSI1_STCCR = (SSI_STCCR_WL(16) | SSI_STCCR_DC(3) | SSI_STCCR_PM(0));
    SSI1_SRCCR = (SSI_SRCCR_WL(16) | SSI_SRCCR_DC(3) | SSI_SRCCR_PM(0));
    SSI1_STMSK = ~((1 << 0) | (1 << 2));
    SSI1_SRMSK = ~((1 << 0) | (1 << 2));

    switch(runtime->rate) {
    case 8000: pll_fsref_48000(7); break;
    case 11025:pll_fsref_44100(4); break;
    case 16000:pll_fsref_48000(3); break;
    case 22050:pll_fsref_44100(2); break;
    case 32000:pll_fsref_48000(1); break;
    case 44100:pll_fsref_44100(0); break;
    case 48000:pll_fsref_48000(0); break;

    default:
        printk( KERN_WARNING "%s/%s(): Can't handle rate of %d\n", 
                __FILE__, __FUNCTION__, runtime->rate);
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
            printk( KERN_ERR "%s/%s(): TRIGGER_START while DMA is already enabled\n",
                    __FILE__, __FUNCTION__ );
        }
        ADBG("%d: dma=%lx ack=%x\n", 
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
            ADBG("%d: dma=%lx ack=%x\n", 
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
        printk( KERN_WARNING "%s/%s(): unknown command %d\n", 
                __FILE__, __FUNCTION__, cmd);
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

    report_dma_errors( __FILE__, __FUNCTION__ );
    chip->c_pos += chip->c_per_size;

    ADBG("%d: dma=%lx ack=%x\n", 
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
static void touchscreen_reset(struct tsc2100_context *devdata)
{
    tsc2100_regwrite(TSC2100_REG_RESETCTL, 0xbb00);
}


static void touchscreen_enable(struct tsc2100_context *devdata)
{
    /* PINTDAV(active low) data available *OR* pen down (PINTDAV goes high
     * once ADC conversion is complete)
     */
    tsc2100_regwrite(TSC2100_REG_STATUS, 0x8000);

    /* Host-controlled conversions, 12-bit samples, scan mode disabled,
     * average (mean) 4 samples per coordinate, 1 MHz internal
     * conversion clock, 500 usec panel voltage stabilization delay
     *
     * NOTE: the previous driver switched back and forth between host
     * and TSC2100 controlled conversions.  TSC2100 controlled conversions to
     * get the X/Y values and host-controlled conversions to get the BAT/TEMP
     * values.  This seemed to cause the TSC2100 to get into a locked state.
     * So, this version of the driver always remains in host-controlled
     * conversions.
     */
    tsc2100_regwrite(TSC2100_REG_ADC, ADCMODE(8));
    devdata->scan_mode = TS_IDLE;

    /* Initialize the reference voltage generator (this eliminates excess noise
     * on sampling lines).
     */
    tsc2100_regwrite(TSC2100_REG_REFCTL, devdata->reg.refctl);
}


static void touchscreen_disable(struct tsc2100_context *devdata)
{
    /* should only occur with an explicit 1 write to /proc or at exit time. */
    printk( KERN_WARNING "%s/%s() called!\n", __FILE__, __FUNCTION__ );

    /* stop conversions and power down
     */
    tsc2100_regwrite(TSC2100_REG_ADC, 0x4000);
    devdata->scan_mode = TS_DISABLED;
}


static void touchscreen_error_recovery( struct tsc2100_context* dev,
                                        const char* funcname, 
                                        const char* scanmode )
{
    printk( KERN_ERR "%s/%s(): %s took too long, resetting touchscreen\n",
           __FILE__, funcname, scanmode );
    touchscreen_disable(dev);
    touchscreen_reset(dev);
    touchscreen_enable(dev);
}


static int touchscreen_pendown( void )
{
    return (tsc2100_regread(TSC2100_REG_ADC) & TSC2100_ADC_PSM) != 0;
}


static void touchscreen_report(struct tsc2100_context *tsc2100,
                               int x, int y, int p, int pendown)
{
    DBG_PEN("x=%d y=%d p=%d pendown=%d x[%d,%d,%d,%d] y[%d,%d,%d,%d]\n", 
            x, y, p, pendown,
            tsc2100->scan_data[0],
            tsc2100->scan_data[2],
            tsc2100->scan_data[4],
            tsc2100->scan_data[6],
            tsc2100->scan_data[1],
            tsc2100->scan_data[3],
            tsc2100->scan_data[5],
            tsc2100->scan_data[7]);
    if ( x != 0 || y != 0 ) {
        tsc2100->pendown_reports++;
        tsc2100->pendown_total++;
    }
    ringwrite( &tsc2100->ring_x, x );
    ringwrite( &tsc2100->ring_y, y );
    ringwrite( &tsc2100->ring_p, p );
    ringwrite( &tsc2100->ring_pen, pendown );

    input_report_abs(tsc2100->inputdevice, ABS_X, x);
    input_report_abs(tsc2100->inputdevice, ABS_Y, y);
    input_report_abs(tsc2100->inputdevice, ABS_PRESSURE, p);
    input_report_key(tsc2100->inputdevice, BTN_TOUCH, pendown);
    input_sync(tsc2100->inputdevice);
}


static void touchscreen_setup(struct device *dev)
{
    struct tsc2100_context *tsc2100 = dev_get_drvdata(dev);

    tsc2100->inputdevice->name = "tsc2100_ts";
    tsc2100->inputdevice->evbit[0] = BIT(EV_KEY) | BIT(EV_ABS);
    tsc2100->inputdevice->keybit[LONG(BTN_TOUCH)] |= BIT(BTN_TOUCH);
    input_set_abs_params(tsc2100->inputdevice,
                         ABS_X, X_AXIS_MIN, X_AXIS_MAX, 0, 0);
    input_set_abs_params(tsc2100->inputdevice,
                         ABS_Y, Y_AXIS_MIN, Y_AXIS_MAX, 0, 0);
    input_set_abs_params(tsc2100->inputdevice,
                         ABS_PRESSURE, PRESSURE_MIN, PRESSURE_MAX, 0, 0);
    input_register_device(tsc2100->inputdevice);
}


static void start_x_scan( struct tsc2100_context* dev )
{
    // Would be nice if we could set the SCAN mode to 1 here and get both the
    // X and Y values.  However, mode 1 will continue to send X/Y values until
    // the PEN is lifted (even if the mode is changed!).  The values are sent
    // approximately every 1.1ms, each generating an interrupt.  So, instead
    // we use modes 3 and 4 which will send one X and one Y value respectively
    // each on an interrupt.  But no interrupts after that.
    
    //DBG_PEN("%s", "...\n");
    dev->event.flags = SCAN_XY;
    dev->scan_mode   = TS_SCANXY;
    dev->scan_start  = jiffies;
    tsc2100_regwrite(TSC2100_REG_ADC, ADCMODE(0x3));
}


static void start_y_scan( struct tsc2100_context* dev )
{
    //DBG_PEN("%s", "...\n");
    dev->scan_start = jiffies;
    tsc2100_regwrite(TSC2100_REG_ADC, ADCMODE(0x4));
}


static void start_next_scan( struct tsc2100_context* dev )
{
    dev->scan_mode  = TS_SCAN;
    dev->scan_start = jiffies;

    switch ( dev->scan_state ) {
    case TS_TEMP1:
        DBG_SCAN("%s", "TEMP1\n");
        tsc2100_regwrite(TSC2100_REG_ADC, ADCMODE(0xA));
        break;
    case TS_TEMP2:
        DBG_SCAN("%s", "TEMP2\n");
        tsc2100_regwrite(TSC2100_REG_ADC, ADCMODE(0xC));
        break;
    case TS_DCLINE:
        DBG_SCAN("%s", "DCLINE\n");
        if (version038)
            imx_gpio_write( 28 | GPIO_PORTC, 1 );    // this is for the line voltage sense
        else
            imx_gpio_write( 30 | GPIO_PORTC, 1 );
        tsc2100_regwrite(TSC2100_REG_ADC, ADCMODE(0x6));
        break;
    case TS_BATTERY:
        DBG_SCAN("%s", "BATTERY\n");
        imx_gpio_write( 30 | GPIO_PORTC, 1 );   // for battery voltage
        tsc2100_regwrite(TSC2100_REG_ADC, ADCMODE(0x7));
        break;
    default:
        printk( KERN_CRIT "OOPS! scan_state=0x%x\n", dev->scan_state );
        BUG();
    }
}


static u32 read_scan_data(struct tsc2100_context *devdata,
                          struct tsc2100_event *event)
{
    u32 data;
    u32 status = tsc2100_regread(TSC2100_REG_STATUS);
    if ( (status & (TSC2100_STATUS_XSTAT  |
                    TSC2100_STATUS_YSTAT  |
                    TSC2100_STATUS_B1STAT |
                    TSC2100_STATUS_B2STAT |
                    TSC2100_STATUS_T1STAT |
                    TSC2100_STATUS_T2STAT)) == 0 ) {
        return status;
    }

    DBG_SCAN("status=%x (X=%d Y=%d B1=%d B2=%d T1=%d T2=%d)\n", 
        status, 
        (status & TSC2100_STATUS_XSTAT)!=0,
        (status & TSC2100_STATUS_YSTAT)!=0,
        (status & TSC2100_STATUS_B1STAT)!=0,
        (status & TSC2100_STATUS_B2STAT)!=0,
        (status & TSC2100_STATUS_T1STAT)!=0,
        (status & TSC2100_STATUS_T2STAT)!=0);

    if ( status & TSC2100_STATUS_XSTAT ) {
        spi_read(TSC2100_REG_X, &data, 1);
        event->x = data;
        event->flags |= HAVE_X;
    }
    if ( status & TSC2100_STATUS_YSTAT ) {
        spi_read(TSC2100_REG_Y, &data, 1);
        event->y = data;
        event->flags |= HAVE_Y;
    }

    #ifdef REAL_PEN_PRESSURE
    if ( status & TSC2100_STATUS_Z1STAT ) {
        spi_read(TSC2100_REG_Z1, &data, 1);
        event->z1 = data;
        event->flags |= HAVE_Z1;
    }
    if ( status & TSC2100_STATUS_Z2STAT ) {
        spi_read(TSC2100_REG_Z2, &data, 1);
        event->z1 = data;
        event->flags |= HAVE_Z2;
    }
    #endif

    if ( status & TSC2100_STATUS_B1STAT )
    {
        if (version038)
            imx_gpio_write( 28 | GPIO_PORTC, 0 ); // turn off DC line gate
        else
            imx_gpio_write( 30 | GPIO_PORTC, 0 ); // turn off the battery gate FET
        ringwrite( &devdata->ring_bat1, tsc2100_regread(TSC2100_REG_BAT1) );
    }
    if ( status & TSC2100_STATUS_B2STAT )
    {
        imx_gpio_write( 30 | GPIO_PORTC, 0 ); // turn off the battery gate FET
        ringwrite( &devdata->ring_bat2, tsc2100_regread(TSC2100_REG_BAT2) );
    }
    if ( status & TSC2100_STATUS_T1STAT )
        ringwrite( &devdata->ring_temp1, tsc2100_regread(TSC2100_REG_TEMP1) );
    if ( status & TSC2100_STATUS_T2STAT )
        ringwrite( &devdata->ring_temp2, tsc2100_regread(TSC2100_REG_TEMP2) );

    return status;
}


static void discard_scan_data( struct tsc2100_context *devdata )
{
    u32 status = tsc2100_regread(TSC2100_REG_STATUS);
    u32 trash;

    DBG_SCAN("status=%x (X=%d Y=%d B1=%d B2=%d T1=%d T2=%d)\n", 
        status, 
        (status & TSC2100_STATUS_XSTAT)!=0,
        (status & TSC2100_STATUS_YSTAT)!=0,
        (status & TSC2100_STATUS_B1STAT)!=0,
        (status & TSC2100_STATUS_B2STAT)!=0,
        (status & TSC2100_STATUS_T1STAT)!=0,
        (status & TSC2100_STATUS_T2STAT)!=0);

    if ( status & TSC2100_STATUS_XSTAT )
        spi_read(TSC2100_REG_X, &trash, 1);
    if ( status & TSC2100_STATUS_YSTAT )
        spi_read(TSC2100_REG_Y, &trash, 1);
    if ( status & TSC2100_STATUS_B1STAT )
        spi_read(TSC2100_REG_BAT1, &trash, 1);
    if ( status & TSC2100_STATUS_B2STAT )
        spi_read(TSC2100_REG_BAT2, &trash, 1);
    if ( status & TSC2100_STATUS_T1STAT )
        spi_read(TSC2100_REG_TEMP1, &trash, 1);
    if ( status & TSC2100_STATUS_T2STAT )
        spi_read(TSC2100_REG_TEMP2, &trash, 1);
}


static int average_power( struct tsc2100_ringbuf* r )
{
    int nxt = ringbeg(r);
    int cnt = ringcnt(r);
    int i, avg;

    if ( cnt < voltage_sample_count )
        return 0;
    for ( avg = i = 0; i < voltage_sample_count; ++i )
        avg += ringread(r, &nxt);
    return avg / voltage_sample_count;
}


static int average_dc_power( struct tsc2100_context* d )
{
    return average_power( &d->ring_bat1 );
}


static int average_battery_power( struct tsc2100_context* d )
{
    return average_power( &d->ring_bat2 );
}


static void process_dcline_measurement( struct tsc2100_context *devdata )
{
    int avg_dc_power;
    int previous;

    if ( ringcnt(&devdata->ring_bat1) < voltage_sample_count ) 
        return;

    avg_dc_power = average_dc_power(devdata);
    previous = unplugged;
    unplugged = avg_dc_power < dc_unplugged_threshold;

    if ( previous != unplugged ) {
        printk( KERN_WARNING "%s/%s(): %s\n", __FILE__, __FUNCTION__, 
                unplugged? "Chumby unplugged!" :  "Chumby plugged in!" );

        battery_poll_next = 0;  // scan battery voltage now
    }

    if ( !unplugged && (raw9volts <= 0 || raw10volts <= 0) ) 
    {
         if (raw9volts <= 0)  raw9volts  = (avg_dc_power * 9) / 12;
         if (raw10volts <= 0) raw10volts = (avg_dc_power * 10) / 12;

         if (raw9volts > 0) 
         {
             int raw8point5volts = ( raw9volts * 85 ) / 90;
             int raw7point5volts = ( raw9volts * 75 ) / 90;
             if ( battery_dead_threshold == -1 )
                 battery_dead_threshold = raw7point5volts;
             if ( battery_dead_threshold != raw7point5volts ) 
             {
                 int volts = (battery_dead_threshold * 900) / raw9volts;
                 printk( KERN_WARNING
                         "%s/%s(): battery-dead-threshold(%d) = %d.02%d volts, recommend(%d) 7.5 volts\n",
                         __FILE__, __FUNCTION__,
                         battery_dead_threshold, 
                         volts / 100, volts % 100, raw7point5volts );
             }
             if ( dc_and_battery_dead_threshold == -1 )
                 dc_and_battery_dead_threshold = raw8point5volts;
             if ( battery_dead_threshold != raw7point5volts ) 
             {
                 int volts = (dc_and_battery_dead_threshold * 900) / raw9volts;
                 printk( KERN_WARNING 
                         "%s/%s(): dc_and_battery-dead-threshold(%d) = %d.02%d volts, recommend(%d) 8.5 volts\n",
                         __FILE__, __FUNCTION__,
                         dc_and_battery_dead_threshold, 
                         volts / 100, volts % 100, raw8point5volts );
             }
             DBG("raw9volts=%d, raw10volts=%d, raw7point5volts=%d, battery_dead_threshold=%d\n",
                 raw9volts, raw10volts, raw7point5volts, battery_dead_threshold);
         }
    }

    if ( sigpwr_pid != 0 && sigpwr_next == 0 && unplugged ) 
    {
        /*
        ** Just performed a dc battery measurement and the power is less
        ** than the minimum threshold (dc_unplugged_threshold).  If a SIGPWR 
        ** signal has not been generated in sigpwr_holdoff seconds and a 
        ** process has registered to received SIGPWR signals 
        ** (sigpwr_pid != 0) -- generate a SIGPWR sig to the process that 
        ** wrote its PID in the /proc/chumby/sigpwr_pid file 
        */
        sigpwr_next = sigpwr_holdoff;
        kill_proc(sigpwr_pid, SIGPWR, 1);
    }
}


static void update_scan_state( struct tsc2100_context *dev, 
                               u32 status, int pendown )
{
    if ( dev->scan_mode == TS_SCANXY ) {
        if (!pendown) {
            if ( status & (TSC2100_STATUS_XSTAT|TSC2100_STATUS_YSTAT) ) {
                DBG("ABORTING XY SCAN, flags=%x\n", dev->event.flags);
                dev->scan_mode = TS_IDLE;
                dev->event.flags = 0;
            }
        } else {
            if ( status & TSC2100_STATUS_XSTAT ) {
                start_y_scan(dev);
                return;
            }

            if ( (dev->event.flags & HAVE_XY) == HAVE_XY ) {
                dev->event.p = 0;
                if (dev->event.x != 0 && dev->event.y != 0) {
                    #ifndef REAL_PEN_PRESSURE
                    static int which = 0;
                    static int ersatz[8] = { 
                        4974, 5069, 5129, 5030, 4973, 4998, 4992, 5015
                    };
                    dev->event.p = ersatz[(which++) & (ARRAY_SIZE(ersatz)-1)];
                    #else
                    // NOTE: would have to create HAVE_Z1, HAVE_Z2 and HAVE_XYZ
                    // and add all of the those states.  I just wanted this code
                    // here as a place holder in case we actually want real PEN
                    // pressue at some point...
                    int z1 = dev->event.z1;
                    int z2 = dev->event.z2;
                    if (z1 != 0)
                        dev->event.p = ((dev->event.x * (z2 - z1) / z1));
                    #endif
                }
                dev->scan_mode = TS_IDLE;

                dev->scan_data[dev->scan_sample] = dev->event.x;
                dev->scan_data[dev->scan_sample+1] = dev->event.y;
                dev->scan_sample += 2;
                if (dev->scan_sample < ARRAY_SIZE(dev->scan_data))
                    start_x_scan(dev);
            }
        }
    }

    if ( dev->scan_mode == TS_SCAN ) {
        switch ( dev->scan_state ) {
        case TS_TEMP1:
            if ( status & TSC2100_STATUS_T1STAT ) {
                dev->scan_state = TS_TEMP2;
                if (dev->pendown_active)
                    dev->scan_mode = TS_IDLE;
                else
                    start_next_scan( dev );
            }
            break;
        case TS_TEMP2:
            if ( status & TSC2100_STATUS_T2STAT )
                dev->scan_mode = TS_IDLE;
            break;
        case TS_DCLINE:
            if ( status & TSC2100_STATUS_B1STAT ) {
                dev->scan_mode = TS_IDLE;
                if ( dev->pendown_active )
                    dcline_poll_next = 0;
                else 
                if ( ++voltage_samples < voltage_sample_count ) 
                    start_next_scan( dev );
                process_dcline_measurement( dev );
            }
            break;
        case TS_BATTERY:
            if ( status & TSC2100_STATUS_B2STAT ) {
                dev->scan_mode = TS_IDLE;
                if ( dev->pendown_active )
                    battery_poll_next = 0;
                else
                if ( ++voltage_samples < voltage_sample_count ) 
                    start_next_scan( dev );
            }
            break;
        default:
            printk( KERN_CRIT "OOPS! scan_state=0x%x\n", dev->scan_state );
            BUG();
        }
    }

    if ( dev->scan_mode == TS_IDLE ) {
        if (!dev->pendown_active) {
            if ( dcline_poll_next <= 0 ) {
                dcline_poll_next = dcline_poll_interval;
                voltage_samples = 0;
                dev->scan_state = TS_DCLINE;
                start_next_scan( dev );
                return;
            }
            if ( battery_poll_next <= 0 ) {
                battery_poll_next = (unplugged? unplugged_battery_poll_interval :
                                                pluggedin_battery_poll_interval);
                voltage_samples = 0;
                dev->scan_state = TS_BATTERY;
                start_next_scan( dev );
                return;
            }
        }

        // no scans necessary at this point in time, go to idle mode
        tsc2100_regwrite(TSC2100_REG_ADC, pendown? ADCMODE(0) : ADCMODE(8));
        dev->scan_state = TS_TEMP1;
    }
}


static void handle_pendown( struct tsc2100_context* dev, int pendown )
{
    if (pendown) {
        
        // PEN is down, clear penup if the PEN was seen to bounce or
        // initialize for the first time
        dev->penup = 0;

        // generate a PEN DOWN report every 10ms
        if ( (dev->event.flags & REPORT_XY) && dev->oneshot_expired )
            dev->event.flags = 0;

        // PEN is down, if the XY SCAN has not been started yet
        // and the SCAN state machine is idle, start the XY SCAN
        if ((dev->event.flags & SCAN_XY)==0 && dev->scan_mode == TS_IDLE) {
            dev->scan_sample = 0;
            start_x_scan(dev);
        }

        if ((dev->event.flags & (SCAN_XY|HAVE_XY|REPORT_XY)) == (SCAN_XY|HAVE_XY)) {
            // XY SCAN is complete, generate the PEN DOWN event

            // throw out the first sample and average the remaining samples 
            u16 x = (dev->scan_data[2] + dev->scan_data[4] + dev->scan_data[6])/3;
            u16 y = (dev->scan_data[3] + dev->scan_data[5] + dev->scan_data[7])/3;

            dev->event.flags |= REPORT_XY;
            if (version038)
                touchscreen_report(dev, x, y, dev->event.p, 1);
            else
                touchscreen_report(dev, y, x, dev->event.p, 1);
        }
    } else
    if (dev->oneshot_expired && dev->penup++) {
        if (dev->pendown_reports) {
            // at least one PEN DOWN event was reported, issue a PEN up event
            touchscreen_report(dev, 0, 0, 0, 0);
        }

        // clear the state information associated with PEN DOWN
        // if the scan_mode is not IDLE something is wrong because the
        // entire system should have been quiet for at least 10ms, and
        // no scans are started while in the PEN DOWN state (only the 
        // X&Y scans are performed, but they are done only when the PEN
        // is DOWN, and we are currently in a situation where the PEN 
        // has been UP for at least 10ms)
        // 
        dev->pendown_active = 0;
        dev->event.flags = 0;
        if (dev->scan_mode != TS_IDLE)
            BUG();
        tsc2100_regwrite(TSC2100_REG_ADC, ADCMODE(8)); // ack PINTDAV* interrupt
    }
}


// handle the TSC2100 interrupt, either the PEN has been pressed or an X or Y
// SCAN is complete.  
// 
static void handle_interrupt( struct tsc2100_context* dev, int pendown )
{
    u32 status = read_scan_data(dev, &(dev->event));
    if (dev->scan_mode == TS_SCANXY && jiffies - dev->scan_start >= SCAN_TIMEOUT) {
        touchscreen_error_recovery( dev, __FUNCTION__, "SCANXY" );
        return;
    }

    if (pendown && !dev->pendown_active) {
        // if this is the first time the PEN DOWN state has been detected,
        // reset the flags
        dev->pendown_active = 1;
        dev->pendown_reports = 0;
        dev->event.flags = 0;
        if ( dev->touchclick )
            tsc2100_regwrite(TSC2100_REG_CONTROL2, 0x8610);  // d3-0 defaults and set to 0
    }

    update_scan_state(dev, status, pendown);

    if ( dev->pendown_active )
        handle_pendown(dev, pendown);
}


// this routine is called approximately every timer_interval milliseconds,
// it is responsible for scheduling all of the various SCANS for temp. and
// voltage (along with determining when to generate the SIGPWR signal)
// 
// this routine also checks to make sure a particular SCAN does not take too
// long and performs error recovery if things seem out of whack
// 
static void handle_timer( struct tsc2100_context* dev, int pendown )
{
    if ( dcline_poll_next > 0 )
        dcline_poll_next -= timer_interval;
    if ( battery_poll_next > 0 )
        battery_poll_next -= timer_interval;
    
    // only allow a SIGPWR signal to be generated every sigpwr_holdoff seconds
    if ( sigpwr_next > 0 ) {
        sigpwr_next -= (timer_interval + 999) / 1000;
        if (sigpwr_next < 0)
            sigpwr_next = 0;
    }

    switch (dev->scan_mode) {
    case TS_IDLE:
        if ( !dev->pendown_active )
            start_next_scan(dev);
        break;
    case TS_SCAN:
        if (jiffies - dev->scan_start >= SCAN_TIMEOUT)
            touchscreen_error_recovery( dev, __FUNCTION__, "SCAN" );
        break;
    case TS_SCANXY:
        break;
    default:
        printk( KERN_ERR "%s(): scan_mode=%d, scan_state=%d\n", 
                __FUNCTION__, dev->scan_mode, dev->scan_state );
        BUG();
    }
}


// this tasklet is scheduled by the PENIRQ ISR routine, the periodic timer and 
// the oneshot timer -- the idea is to serialize access to the TSC2100 and 
// associated data structures so that no locks or mutexes are necessary (other 
// than the IRQ SPIN LOCK used to protect the SPI routines that are used by both 
// the touchscreen and ALSA drivers) -- also, the tasklet offloads the ISR 
// routine reducing overall interrupt latency
// 
static void touchscreen_tasklet( unsigned long _dev )
{
    struct tsc2100_context* dev = (void*)_dev;
    unsigned long flags;
    int interrupt;
    int timer_expired;
    int pendown;

    spin_lock_irqsave(&dev->lock, flags);
    interrupt = dev->interrupt;
    timer_expired = dev->timer_expired;
    dev->interrupt = 0;
    dev->timer_expired = 0;
    spin_unlock_irqrestore(&dev->lock, flags);

    pendown = touchscreen_pendown();

    if (interrupt)
        handle_interrupt(dev, pendown);

    if (timer_expired)
        handle_timer(dev, pendown);

    // schedule the oneshot timer to go off 10ms from now if the
    // scan mode is not idle or the PEN is down (note, SCAN_X and
    // SCAN_Y will generate an interrupt when complete, scanning for
    // BAT and TEMP will *NOT*)
    if (dev->scan_mode == TS_SCAN || dev->pendown_active) {
        dev->oneshot_expired = 0;
        mod_timer(&(dev->oneshot), jiffies + HZ/100);
    }
}


// touchscreen interrupts occur when the PEN is pressed or when either the 
// X or Y SCAN is complete -- schedule handle_interrupt() to process the
// interrupt event at SOFT IRQ time
// 
static irqreturn_t touchscreen_interrupt(int irq, void *dev_id, 
                                         struct pt_regs *regs)
{
    struct tsc2100_context *dev = dev_id;

    if ( dev->tson ) {
        #if 0
        u32 status = tsc2100_regread(TSC2100_REG_STATUS);
        int pendown = touchscreen_pendown();
        DBG_PEN("status=%x, interrupt=%d, dev->scan_mode=%d, dev->pendown=%d, (X=%d Y=%d P=%d)\n", 
                status, dev->interrupt, dev->scan_mode, dev->pendown_active,
                (status & TSC2100_STATUS_XSTAT)!=0,
                (status & TSC2100_STATUS_YSTAT)!=0, 
                pendown);
        #endif
        dev->interrupt++;
        tasklet_schedule( &dev->tasklet );
    }

    return IRQ_HANDLED;
}


// note: oneshot timer is only scheduled by the touchscreen_tasklet() when
// a SCAN is active, this timer will go off 10ms after the SCAN has been
// activated to handle reading the SCAN result if ready (the XY SCANs generate
// an interrupt, the other SCANs don't and rely on this routine to complete the
// SCAN by reading the data -- that is, scheduling handle_interrupt() to do the
// work at SOFT IRQ time)
// 
static void touchscreen_oneshot(unsigned long data)
{
    struct tsc2100_context* dev = (struct tsc2100_context*) data;

    if ( dev->tson ) {
        dev->interrupt++;
        dev->oneshot_expired = 1;
        tasklet_schedule( &dev->tasklet );
    }
}


// this timer goes off every timer_interval milliseconds and is periodic, this
// routine starts a SCAN when the system is idle (well, it sets the 
// timer_expired bit so the handle_timer() routine will do all of the work at 
// SOFT IRQ time)
// 
static void touchscreen_timer(unsigned long data)
{
    struct tsc2100_context* dev = (struct tsc2100_context*) data;

    mod_timer(&(dev->timer), jiffies + timer_interval_jiffies);
    if ( dev->tson ) {
        dev->timer_expired++;
        tasklet_schedule( &dev->tasklet );
    }
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
    struct tsc2100_context* d = ( struct tsc2100_context* ) data;
    return sprintf( buf, "%d\n", d->touchclick );
}


static int proc_set_touchclick_cb( struct file* file, const char* buf,
                                   unsigned long count, void* data )
{
    struct tsc2100_context* d = ( struct tsc2100_context* ) data;
    u32  value;
    int  rc = getval_from_user(file,buf,count,&value);
    if ( rc )
        return rc;

    d->touchclick = value;
    return count;
}


static int proc_get_tson_cb( char* buf, char** start, off_t offset,
                            int count, int* eof, void* data )
{
    struct tsc2100_context* d = ( struct tsc2100_context* ) data;
    return sprintf( buf, "%d\n", d->tson );
}


static int proc_set_tson_cb( struct file* file, const char* buf,
                             unsigned long count, void* data )
{
    struct tsc2100_context* d = ( struct tsc2100_context* ) data;
    u32  value;
    int  rc = getval_from_user(file,buf,count,&value);
    if ( rc )
        return rc;

    d->tson = value;
    if ( value )
        touchscreen_enable(d);
    else
        touchscreen_disable(d);
    return count;
}


static int proc_get_coords_cb( char* buf, char** start, off_t offset,
                               int count, int* eof, void* data )
{
    struct tsc2100_context* d = ( struct tsc2100_context* ) data;
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
    struct tsc2100_context* d = ( struct tsc2100_context* ) data;
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
    struct tsc2100_context* d = ( struct tsc2100_context* ) data;
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
        leng += sprintf( buf + leng, "voltage_sample_count=%d\n", voltage_sample_count );
    if ( leng < count - 32 )
        leng += sprintf( buf + leng, "dc_unplugged_threshold=%d\n", dc_unplugged_threshold );
    if ( leng < count - 32 )
        leng += sprintf( buf + leng, "battery_dead_threshold=%d\n", battery_dead_threshold );
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
    struct tsc2100_context* d = ( struct tsc2100_context* ) data;
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


static int proc_get_timer_interval_cb( char* buf, char** start, off_t offset,
                                       int count, int* eof, void* data )
{
    *eof = 1;
    return sprintf( buf, "%d milliseconds\n", timer_interval );
}


static int proc_set_timer_interval_cb( struct file* file, const char* buf,
                                        unsigned long count, void* data )
{
    u32  value;
    int  rc = getval_from_user(file,buf,count,&value);
    if ( rc )
        return rc;

    if (value < MIN_TIMER_INTERVAL)
        value = MIN_TIMER_INTERVAL;
    if (pluggedin_battery_poll_interval < value)
        pluggedin_battery_poll_interval = value;
    if (unplugged_battery_poll_interval < value)
        unplugged_battery_poll_interval = value;
    if (dcline_poll_interval < value)
        dcline_poll_interval = value;
    timer_interval = value;
    timer_interval_jiffies = msecs_to_jiffies(timer_interval);
    return count;
}


static int plugged_into_wall( struct tsc2100_context* devdata )
{
    return average_dc_power(devdata) >= dc_unplugged_threshold;
}


static int battery_dead( struct tsc2100_context* devdata )
{
    int avg = average_battery_power(devdata);

    if (unplugged) 
        return battery_dead_threshold < 0 || 
               avg < battery_dead_threshold;
    else
        return dc_and_battery_dead_threshold < 0 || 
               avg < dc_and_battery_dead_threshold;
}


static int battery_present( struct tsc2100_context* devdata )
{
    int avg = average_battery_power(devdata);
    return avg > raw9volts/2 && avg <= raw10volts;
}


static int battery_voltage( struct tsc2100_context* devdata )
{
    int avg = average_battery_power(devdata);
    int voltage = 0;

    if ( raw9volts > 0 && raw10volts > 0 ) 
        voltage = avg >= raw10volts? 0 : ( avg * 900 ) / raw9volts;
    return voltage;
}


static int proc_get_battery_present_cb( char* buf, char** start, off_t offset,
                                      int count, int* eof, void* data )
{
    *eof = 1;
    return sprintf( buf, "%d\n", battery_present((struct tsc2100_context*)data) );
}


static int proc_get_battery_voltage_cb( char* buf, char** start, off_t offset,
                                        int count, int* eof, void* data )
{
    *eof = 1;
    return sprintf( buf, "%d\n", battery_voltage((struct tsc2100_context*) data) );
}


static int proc_get_battery_dead_cb( char* buf, char** start, off_t offset,
                                     int count, int* eof, void* data )
{
    *eof = 1;
    return sprintf( buf, "%d\n", battery_dead((struct tsc2100_context*) data) );
}


static int proc_get_dc_and_battery_dead_threshold_cb( 
        char* buf, char** start, off_t offset,int count, int* eof, void* data )
{
    *eof = 1;
    return sprintf( buf, "%d\n", dc_and_battery_dead_threshold );
}


static int proc_set_dc_and_battery_dead_threshold_cb( 
        struct file* file, const char* buf, unsigned long count, void* data )
{
    u32  value;
    int  rc = getval_from_user(file,buf,count,&value);
    if ( rc )
        return rc;

    dc_and_battery_dead_threshold = value;
    return count;
}


static int proc_get_battery_dead_threshold_cb( 
        char* buf, char** start, off_t offset,int count, int* eof, void* data )
{
    *eof = 1;
    return sprintf( buf, "%d\n", battery_dead_threshold );
}


static int proc_set_battery_dead_threshold_cb( 
        struct file* file, const char* buf, unsigned long count, void* data )
{
    u32  value;
    int  rc = getval_from_user(file,buf,count,&value);
    if ( rc )
        return rc;

    battery_dead_threshold = value;
    return count;
}


static int proc_get_raw_battery_voltage_cb( char* buf, char** start, off_t offset,
                                            int count, int* eof, void* data )
{
    *eof = 1;
    return sprintf( buf, "%d\n", average_battery_power(( struct tsc2100_context* ) data) );
}


static int proc_get_raw_dcline_voltage_cb( char* buf, char** start, off_t offset,
                                           int count, int* eof, void* data )
{
    *eof = 1;
    return sprintf( buf, "%d\n", average_dc_power(( struct tsc2100_context* ) data) );
}


static int proc_get_raw9volts_cb( char* buf, char** start, off_t offset,
                                  int count, int* eof, void* data )
{
    *eof = 1;
    return sprintf( buf, "%d\n", raw9volts );
}


static int proc_set_raw9volts_cb( struct file* file, const char* buf, 
                                  unsigned long count, void* data )
{
    u32  value;
    int  rc = getval_from_user(file,buf,count,&value);
    if ( rc )
        return rc;

    raw9volts  = value;
    raw10volts = ( raw9volts * 10 ) / 9;
    return count;
}


static int proc_get_plugged_into_wall_cb( char* buf, char** start, off_t offset,
                                          int count, int* eof, void* data )
{
    *eof = 1;
    return sprintf( buf, "%d\n", 
                    plugged_into_wall(( struct tsc2100_context* ) data) );
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

    sigpwr_pid = value;
    return count;
}


static int proc_set_measure_voltage_now_cb( struct file* file, const char* buf,
                                            unsigned long count, void* data )
{
    // force a voltage measurement now (in the next timer_interval)
    battery_poll_next = 0;
    dcline_poll_next = 0;
    return count;
}


static int proc_get_dcline_poll_interval_cb( 
        char* buf, char** start, off_t offset, int count, int* eof, void* data )
{
    *eof = 1;
    return sprintf( buf, "%d milliseconds\n", dcline_poll_interval );
}


static int proc_set_dcline_poll_interval_cb( 
        struct file* file, const char* buf, unsigned long count, void* data )
{
    u32  value;
    int  rc = getval_from_user(file,buf,count,&value);
    if ( rc )
        return rc;

    if ( value < timer_interval )
        value = timer_interval;
    dcline_poll_interval = value;
    return count;
}


static int proc_get_unplugged_battery_poll_interval_cb( 
        char* buf, char** start, off_t offset, int count, int* eof, void* data )
{
    *eof = 1;
    return sprintf( buf, "%d milliseconds\n", unplugged_battery_poll_interval );
}


static int proc_set_unplugged_battery_poll_interval_cb( 
        struct file* file, const char* buf, unsigned long count, void* data )
{
    u32  value;
    int  rc = getval_from_user(file,buf,count,&value);
    if ( rc )
        return rc;

    if ( value < timer_interval )
        value = timer_interval;
    unplugged_battery_poll_interval = value;
    return count;
}


static int proc_get_pluggedin_battery_poll_interval_cb( 
        char* buf, char** start, off_t offset, int count, int* eof, void* data )
{
    *eof = 1;
    return sprintf( buf, "%d milliseconds\n", pluggedin_battery_poll_interval );
}


static int proc_set_pluggedin_battery_poll_interval_cb( 
        struct file* file, const char* buf, unsigned long count, void* data )
{
    u32  value;
    int  rc = getval_from_user(file,buf,count,&value);
    if ( rc )
        return rc;

    if ( value < timer_interval )
        value = timer_interval;
    pluggedin_battery_poll_interval = value;
    return count;
}


static int proc_get_voltage_sample_count_cb( 
        char* buf, char** start, off_t offset, int count, int* eof, void* data )
{
    *eof = 1;
    return sprintf( buf, "%d\n", voltage_sample_count );
}


static int proc_set_voltage_sample_count_cb( 
        struct file* file, const char* buf, unsigned long count, void* data )
{
    u32  value;
    int  rc = getval_from_user(file,buf,count,&value);
    if ( rc )
        return rc;

    if ( value < 1 )
        value = 1;
    voltage_sample_count = value;
    return count;
}


static int proc_get_dc_unplugged_threshold_cb( 
        char* buf, char** start, off_t offset, int count, int* eof, void* data )
{
    *eof = 1;
    return sprintf( buf, "%d\n", dc_unplugged_threshold );
}


static int proc_set_dc_unplugged_threshold_cb( 
        struct file* file, const char* buf, unsigned long count, void* data )
{
    u32  value;
    int  rc = getval_from_user(file,buf,count,&value);
    if ( rc )
        return rc;

    dc_unplugged_threshold = value;
    return count;
}


static int proc_get_power_summary_cb( 
        char* buf, char** start, off_t offset, int count, int* eof, void* data )
{
    struct tsc2100_context* d = ( struct tsc2100_context* ) data;
    int    leng = 0;
    int    voltage = battery_voltage(d);

    leng += sprintf( buf + leng, "\n" );
    if (!battery_present(d)) 
    {
        if (plugged_into_wall(d)) 
            leng += sprintf( buf + leng, "Chumby is plugged into the wall and the battery is not installed.\n" );
        else
            leng += sprintf( buf + leng, "WOW! Chumby is *NOT* plugged into the wall and the battery is *NOT* installed!!!\n" );
    }
    else
    {
        if (battery_dead(d)) 
            leng += sprintf( buf + leng, "Chumby is %splugged into the wall and the battery is dead.\n", 
                             plugged_into_wall(d)? "" : "not " );
        else
            leng += sprintf( buf + leng, "Chumby is %splugged into the wall and the battery voltage is %d.%02d volts.\n",
                             plugged_into_wall(d)? "" : "not ", voltage / 100, voltage % 100 );
    }

    leng += sprintf( buf + leng, "\n" );
    leng += sprintf( buf + leng, "Timer Poll Interval...........................%d msecs\n", timer_interval );
    leng += sprintf( buf + leng, "DCLine Measurement Poll Interval..............%d msecs\n", dcline_poll_interval );
    leng += sprintf( buf + leng, "Unplugged Battery Measurement Poll Interval...%d msecs\n", unplugged_battery_poll_interval );
    leng += sprintf( buf + leng, "Plugged-in Battery Measurement Poll Interval..%d msecs\n", pluggedin_battery_poll_interval );
    leng += sprintf( buf + leng, "Time until next DCLine voltage measurement....%d msecs\n", dcline_poll_next );
    leng += sprintf( buf + leng, "Time until next battery voltage measurement...%d msecs\n", battery_poll_next );
    leng += sprintf( buf + leng, "Voltage Sample Count..........................%d\n", voltage_sample_count );
    leng += sprintf( buf + leng, "Voltage Samples...............................%d\n", voltage_samples );
    leng += sprintf( buf + leng, "[dc_unplugged_threshold]......................%d\n", dc_unplugged_threshold );
    leng += sprintf( buf + leng, "[dc_and_battery_dead_threshold]...............%d\n", dc_and_battery_dead_threshold );
    leng += sprintf( buf + leng, "[battery_dead_threshold]......................%d\n", battery_dead_threshold );
    leng += sprintf( buf + leng, "[raw9volts]...................................%d\n", raw9volts );
    leng += sprintf( buf + leng, "Time until next SIGPWR signal.................%d seconds\n", sigpwr_next );
    leng += sprintf( buf + leng, "[sigpwr_holdoff]..............................%d seconds\n", sigpwr_holdoff );
    leng += sprintf( buf + leng, "[sigpwr_pid]..................................%d\n", sigpwr_pid );
    leng += sprintf( buf + leng, "Plugged into wall? [avg_dc_power(%d) >= dc_unplugged_threshold(%d)?] -- %s\n", 
                     average_dc_power(d), dc_unplugged_threshold, plugged_into_wall(d)? "YES" : "NO" );
    leng += sprintf( buf + leng, "Battery installed? [avg_batt_power(%d) > raw9volts(%d)/2 && avg_batt_power <= raw10volts(%d)?] -- %s\n", 
                     average_battery_power(d), raw9volts, raw10volts, battery_present(d)? "YES" : "NO" );
    leng += sprintf( buf + leng, "Battery voltage %d.%02d [avg_batt_power(%d) * 900 / raw9volts(%d)]\n",
                     voltage / 100, voltage % 100, average_battery_power(d), raw9volts );
    if (plugged_into_wall(d)) 
        leng += sprintf( buf + leng, 
                   "Battery dead? [dc_and_battery_dead_threshold(%d) < 0 || avg_batt_power(%d) < dc_and_battery_dead_threshold?] -- %s\n",
                   dc_and_battery_dead_threshold, average_battery_power(d), battery_dead(d)? "YES" : "NO" );
    else
        leng += sprintf( buf + leng, 
                    "Battery dead? [battery_dead_threshold(%d) < 0 || avg_batt_power(%d) < battery_dead_threshold?] -- %s\n",
                    battery_dead_threshold, average_battery_power(d), battery_dead(d)? "YES" : "NO" );
    leng += sprintf( buf + leng, "\n" );
    return leng;
}


static int proc_get_context_cb( 
        char* buf, char** start, off_t offset, int count, int* eof, void* data )
{
    struct tsc2100_context* d = ( struct tsc2100_context* ) data;
    int    leng = 0;

    *eof = 1;
    leng += sprintf( buf + leng, "Hardware Version .... 1.%c\n", version038? '8' : 'x' );
    leng += sprintf( buf + leng, "timer_expired.........%d\n", d->timer_expired );
    leng += sprintf( buf + leng, "interrupt.............%d\n", d->interrupt );
    leng += sprintf( buf + leng, "tson..................%d\n", d->tson );
    leng += sprintf( buf + leng, "scan_mode.............%d\n", d->scan_mode );
    leng += sprintf( buf + leng, "scan_state............%d\n", d->scan_state );
    leng += sprintf( buf + leng, "pendown...............%d\n", d->pendown_active );
    leng += sprintf( buf + leng, "penup.................%d\n", d->penup );
    leng += sprintf( buf + leng, "touchclick............%d\n", d->touchclick );
    leng += sprintf( buf + leng, "event.flags...........0x%x\n", d->event.flags );
    leng += sprintf( buf + leng, "event.pressure........%d\n", d->event.p );
    leng += sprintf( buf + leng, "event.x...............%d\n", d->event.x );
    leng += sprintf( buf + leng, "event.y...............%d\n", d->event.y );
    leng += sprintf( buf + leng, "event.pen.............%d\n", d->event.pen );
    leng += sprintf( buf + leng, "x[0]..................%d\n", d->scan_data[0]);
    leng += sprintf( buf + leng, "x[1]..................%d\n", d->scan_data[2]);
    leng += sprintf( buf + leng, "x[2]..................%d\n", d->scan_data[4]);
    leng += sprintf( buf + leng, "x[3]..................%d\n", d->scan_data[6]);
    leng += sprintf( buf + leng, "y[0]..................%d\n", d->scan_data[1]);
    leng += sprintf( buf + leng, "y[1]..................%d\n", d->scan_data[3]);
    leng += sprintf( buf + leng, "y[2]..................%d\n", d->scan_data[5]);
    leng += sprintf( buf + leng, "y[3]..................%d\n", d->scan_data[7]);
    leng += sprintf( buf + leng, "pendown_count.........%d\n", d->pendown_total );
    
    return leng;
}


static int proc_get_pendown_count_cb( 
        char* buf, char** start, off_t offset, int count, int* eof, void* data )
{
    struct tsc2100_context* d = ( struct tsc2100_context* ) data;
    *eof = 1;
    return sprintf( buf, "%d\n", d->pendown_total );
}


static int proc_set_pendown_count_cb( 
        struct file* file, const char* buf, unsigned long count, void* data )
{
    struct tsc2100_context* d = ( struct tsc2100_context* ) data;
    u32  value;
    int  rc = getval_from_user(file,buf,count,&value);
    if ( rc )
        return rc;

    d->pendown_total = value;
    return count;
}


#ifdef PLAY_DEBUG_SOUND
void tsc2100_debug_sound_init( void )
{
    if (debug_sound_buf == NULL) {
        debug_sound_last = 0;
        debug_sound_next = 0;
        debug_sound_buf  = __get_free_pages( GFP_ATOMIC, 6 );
        if (debug_sound_buf)
            memset( debug_sound_buf, 0, 64*PAGE_SIZE );
    }
}


void tsc2100_debug_sound_append( unsigned char* data, int nbytes )
{
    if (debug_sound_buf == NULL) {
        tsc2100_debug_sound_init();
        if (debug_sound_buf == NULL)
            return;
    }

    if (debug_sound_last + nbytes > 64*PAGE_SIZE)
        nbytes = 64*PAGE_SIZE - debug_sound_last;

    if (nbytes > 0) {
        memcpy( debug_sound_buf + debug_sound_last, data, nbytes );
        debug_sound_last += nbytes;
    }
}


void tsc2100_debug_sound_done( void )
{
    if (debug_sound_buf) {
        debug_sound_next = 0;
        debug_sound_last = 0;
        free_pages(debug_sound_buf, 6);
        debug_sound_buf = NULL;
    }
}
EXPORT_SYMBOL(tsc2100_debug_sound_init);
EXPORT_SYMBOL(tsc2100_debug_sound_append);
EXPORT_SYMBOL(tsc2100_debug_sound_done);



static int proc_get_debug_sound_cb( 
        char* buf, char** start, off_t offset, int count, int* eof, void* data )
{
    *eof = 1;
    return sprintf( buf, "buf=%p, next=%x, last=%x\n", debug_sound_buf, debug_sound_next, debug_sound_last );
}


static int proc_set_debug_sound_cb( 
        struct file* file, const char* buf, unsigned long count, void* data )
{
    tsc2100_debug_sound_done();
    return count;
}
#endif


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
/*9*/    { "tsc2100",       CHUMBY_DIR }
         // 16 entries total -- update in chumby-tsc2100.h
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
/*0*/  { "temperature",               CHUMBY_DIR,      proc_get_temp_cb,         NULL, 0 },
/*1*/  { "enable",                    TOUCHSCREEN_DIR, proc_get_tson_cb,         proc_set_tson_cb, 0 },
/*2*/  { "coordinates",               TOUCHSCREEN_DIR, proc_get_coords_cb,       NULL, 0 },
/*3*/  { "timer-interval",            TOUCHSCREEN_DIR, proc_get_timer_interval_cb, proc_set_timer_interval_cb, 0 },
/*4*/  { "touchclick",                TOUCHSCREEN_DIR, proc_get_touchclick_cb,   proc_set_touchclick_cb, 0 },
/*5*/  { "mute",                      LEFTSPKR_DIR,    proc_get_left_mute_cb,    proc_set_left_mute_cb, 0 },
/*6*/  { "mute",                      RIGHTSPKR_DIR,   proc_get_right_mute_cb,   proc_set_right_mute_cb, 0 },
/*7*/  { "mute",                      BOTHSPKR_DIR,    proc_get_both_mute_cb,    proc_set_both_mute_cb, 0 },
/*8*/  { "volume",                    LEFTSPKR_DIR,    proc_get_left_volume_cb,  proc_set_left_volume_cb, 0 },
/*9*/  { "volume",                    RIGHTSPKR_DIR,   proc_get_right_volume_cb, proc_set_right_volume_cb, 0 },
/*10*/ { "volume",                    BOTHSPKR_DIR,    proc_get_both_volume_cb,  proc_set_both_volume_cb, 0 },
/*11*/ { "analog-mute",               SIDETONE_DIR,    proc_get_analog_mute_cb,  proc_set_analog_mute_cb, 0 },
/*12*/ { "analog-gain",               SIDETONE_DIR,    proc_get_analog_gain_cb,  proc_set_analog_gain_cb, 0 },
/*13*/ { "digital-mute",              SIDETONE_DIR,    proc_get_digital_mute_cb, proc_set_digital_mute_cb, 0 },
/*14*/ { "digital-gain",              SIDETONE_DIR,    proc_get_digital_gain_cb, proc_set_digital_gain_cb, 0 },
/*15*/ { "registers",                 TSC2100_DIR,     proc_get_tsregs_cb,       NULL, 0 },
/*16*/ { "adc-page1",                 TSC2100_DIR,     proc_get_adc_cb,          proc_set_adc_cb, 0 },
/*17*/ { "refctl-page1",              TSC2100_DIR,     proc_get_refctl_cb,       proc_set_refctl_cb, 0 },
/*18*/ { "control1-page2",            TSC2100_DIR,     proc_get_control1_cb,     proc_set_control1_cb, 0 },
/*19*/ { "audioadc-page2",            TSC2100_DIR,     proc_get_audioadc_cb,     proc_set_audioadc_cb, 0 },
/*20*/ { "audiodac-page2",            TSC2100_DIR,     proc_get_audiodac_cb,     proc_set_audiodac_cb, 0 },
/*21*/ { "sidetone-page2",            TSC2100_DIR,     proc_get_sidetone_cb,     proc_set_sidetone_cb, 0 },
/*22*/ { "control2-page2",            TSC2100_DIR,     proc_get_control2_cb,     proc_set_control2_cb, 0 },
/*23*/ { "powercon-page2",            TSC2100_DIR,     proc_get_powercon_cb,     proc_set_powercon_cb, 0 },
/*24*/ { "control3-page2",            TSC2100_DIR,     proc_get_control3_cb,     proc_set_control3_cb, 0 },
/*25*/ { "pll1-page2",                TSC2100_DIR,     proc_get_pll1_cb,         proc_set_pll1_cb, 0 },
/*26*/ { "pll2-page2",                TSC2100_DIR,     proc_get_pll2_cb,         proc_set_pll2_cb, 0 },
/*27*/ { "control4-page2",            TSC2100_DIR,     proc_get_control4_cb,     proc_set_control4_cb, 0 },
/*28*/ { "control5-page2",            TSC2100_DIR,     proc_get_control5_cb,     proc_set_control5_cb, 0 },
/*29*/ { "battery-voltage-history",   CHUMBY_DIR,      proc_get_batt_cb,         NULL, 0 },
/*30*/ { "battery-voltage",           CHUMBY_DIR,      proc_get_battery_voltage_cb, NULL, 0 },
/*31*/ { "battery-present",           CHUMBY_DIR,      proc_get_battery_present_cb,NULL, 0 },
/*32*/ { "battery-dead",              CHUMBY_DIR,      proc_get_battery_dead_cb, NULL, 0 },
/*33*/ { "battery-dead-threshold",    CHUMBY_DIR,      proc_get_battery_dead_threshold_cb, proc_set_battery_dead_threshold_cb, 0 },
/*34*/ { "dc-and-battery-dead-threshold",CHUMBY_DIR,   proc_get_dc_and_battery_dead_threshold_cb,proc_set_dc_and_battery_dead_threshold_cb, 0 },
/*35*/ { "plugged-into-wall",         CHUMBY_DIR,      proc_get_plugged_into_wall_cb, NULL, 0 },
/*36*/ { "dc-unplugged-threshold",    CHUMBY_DIR,      proc_get_dc_unplugged_threshold_cb, proc_set_dc_unplugged_threshold_cb, 0 },
/*37*/ { "raw-battery-voltage",       CHUMBY_DIR,      proc_get_raw_battery_voltage_cb, NULL, 0 },
/*38*/ { "raw-dcline-voltage",        CHUMBY_DIR,      proc_get_raw_dcline_voltage_cb, NULL, 0 },
/*39*/ { "raw9volts",                 CHUMBY_DIR,      proc_get_raw9volts_cb,    proc_set_raw9volts_cb, 0 },
/*40*/ { "sigpwr-pid",                CHUMBY_DIR,      proc_get_sigpwr_pid_cb,   proc_set_sigpwr_pid_cb, 0 },
/*41*/ { "measure-voltage-now",       CHUMBY_DIR,      NULL, proc_set_measure_voltage_now_cb, 0 },
/*42*/ { "dcline-poll-interval",      CHUMBY_DIR,      proc_get_dcline_poll_interval_cb, proc_set_dcline_poll_interval_cb, 0 },
/*43*/ { "unplugged-battery-poll-interval", CHUMBY_DIR,proc_get_unplugged_battery_poll_interval_cb, proc_set_unplugged_battery_poll_interval_cb, 0 },
/*44*/ { "pluggedin-battery-poll-interval", CHUMBY_DIR,proc_get_pluggedin_battery_poll_interval_cb, proc_set_pluggedin_battery_poll_interval_cb, 0 },
/*45*/ { "voltage-sample-count",      CHUMBY_DIR,      proc_get_voltage_sample_count_cb, proc_set_voltage_sample_count_cb, 0 },
/*46*/ { "power-summary",             CHUMBY_DIR,      proc_get_power_summary_cb,     NULL, 0 },
/*47*/ { "pendown-count",             TOUCHSCREEN_DIR, proc_get_pendown_count_cb, proc_set_pendown_count_cb, 0 },
/*48*/ { "context",                   TOUCHSCREEN_DIR, proc_get_context_cb, NULL, 0 },

#ifdef PLAY_DEBUG_SOUND
/*49*/ { "debug-sound",               CHUMBY_DIR,      proc_get_debug_sound_cb, proc_set_debug_sound_cb, 0 }
#endif

         // 64 entries total -- if you need more have to update 
         // "struct proc_dir_entry* proc_files[ 64 ];" in chumby-tsc2100.h 
};


static void __exit proc_rmdir(struct tsc2100_context* d)
{
    int i;

    for ( i = ARRAY_SIZE(procdir)-1; i != 0; --i ) {
        if (d->proc_dirs[i]) {
            remove_proc_entry( procdir[i].name, d->proc_dirs[procdir[i].parent] );
            d->proc_dirs[i] = NULL;
        }
    }
}


static void __exit proc_exit(struct tsc2100_context* d)
{
    int i;

    for ( i = 0; i < ARRAY_SIZE(procfile); ++i ) {
        if (d->proc_files[i]) {
            remove_proc_entry( procfile[i].name, d->proc_dirs[procfile[i].parent] );
            d->proc_files[i] = NULL;
        }
    }

    proc_rmdir(d);
}


static int __init proc_mkdirs(struct tsc2100_context* d)
{
    int i;

    for ( i = 1; i < ARRAY_SIZE(procdir); ++i ) {
        d->proc_dirs[i] = proc_mkdir(procdir[i].name,
                                     d->proc_dirs[procdir[i].parent]);
        if (d->proc_dirs[i] == NULL) {
            proc_rmdir(d);
            return -1;
        }
    }

    return 0;
}


static int __init proc_init(struct tsc2100_context* d)
{
    int i;

    if (ARRAY_SIZE(d->proc_dirs) < ARRAY_SIZE(procdir)) {
        printk( KERN_ERR "%s/%s(): ARRAY_SIZE(d->proc_dirs)[%d] < ARRAY_SIZE(procdir)[%d]\n",
                __FILE__, __FUNCTION__, 
                ARRAY_SIZE(d->proc_dirs), ARRAY_SIZE(procdir) );
        return -1;
    }
    if (ARRAY_SIZE(d->proc_files) < ARRAY_SIZE(procfile)) {
        printk( KERN_ERR "%s/%s(): ARRAY_SIZE(d->proc_files)[%d] < ARRAY_SIZE(procfile)[%d]\n",
                __FILE__, __FUNCTION__, 
                ARRAY_SIZE(d->proc_files), ARRAY_SIZE(procfile) );
        return -1;
    }

    if ( proc_mkdirs(d) )
        return -1;

    for ( i = 0; i < ARRAY_SIZE(procfile); ++i ) {
        struct proc_dir_entry* entry;
        void* data = (void*)(((char*) d) + procfile[i].offset);

        entry = proc_file( procfile[i].name, d->proc_dirs[procfile[i].parent],
                           procfile[i].read_proc, procfile[i].write_proc,
                           data );
        if ( entry == NULL ) {
            proc_exit(d);
            return -1;
        }

        d->proc_files[i] = entry;
    }

    return 0;
}


static int __init touchscreen_init(void)
{
    struct device  tsc2100dev;
    struct tsc2100_context *devdata;
    struct input_dev *input_dev;
    int rc;

    devdata = kzalloc(sizeof(struct tsc2100_context), GFP_KERNEL);
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
    if( CSCR & CSCR_MCU_SEL ) { // detect clock version by looking at where CPU gets its clock
      // we are in 16 MHz land (hardware version 1.6 and higher)
      reg_pll1 = 1 << 15  |    // enable PLL
                 1 << 8   |    // P value is 1 for 48.0 kHz
                 6 << 2;       // J value is 6 for 48.0 kHz
      reg_pll2 = 1440<<2;      // D value is 1440 for 48.0 kHz
    } else {
      // leave at default values
    }
    devdata->reg.pll1     = reg_pll1;
    devdata->reg.pll2     = reg_pll2;
    devdata->reg.control4 = reg_control4;
    devdata->reg.control5 = reg_control5;

    devdata->tson = 1;
    devdata->touchclick = 1;

    tasklet_init(&devdata->tasklet, touchscreen_tasklet, (unsigned long)devdata);

    init_timer(&devdata->timer);
    devdata->timer.data = (unsigned long) devdata;
    devdata->timer.function = touchscreen_timer;

    init_timer(&devdata->oneshot);
    devdata->oneshot.data = (unsigned long) devdata;
    devdata->oneshot.function = touchscreen_oneshot;

    rc = request_irq(PENIRQ, touchscreen_interrupt, 0, "tsc2100", devdata);
    if (rc) {
        printk(KERN_ERR "%s/%s(): Could not allocate touchscreen IRQ %d!\n",
               __FILE__, __FUNCTION__, PENIRQ);
        tsc2100_devdata  = NULL;
        kfree(devdata);
        return -EINVAL;
    }

    gpio_init();
    spi_init();

    touchscreen_setup(&tsc2100dev);
    touchscreen_reset(devdata);
    touchscreen_enable(devdata);

    /* Falling edge indicates either PEN has been pressed or ADC SCAN is 
     * complete. (note we read the data here simply to flush and discard)
     */
    set_irq_type(PENIRQ, IRQT_FALLING);
    discard_scan_data(devdata);

    timer_interval_jiffies = msecs_to_jiffies(timer_interval);
    mod_timer(&(devdata->timer), jiffies + timer_interval_jiffies);

    printk( KERN_INFO "Chumby TI-TSC2100 TouchScreen Driver v1.01 initialized (hw 1.%c)\n", 
            version038? '8':'x');
    return 0;
}


static int __init alsa_audio_init(void)
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

    pll_fsref_48000(0);
    tsc2100_regwrite(TSC2100_REG_SIDETONE,tsc2100_devdata->reg.sidetone);
    tsc2100_regwrite(TSC2100_REG_CONTROL4,tsc2100_devdata->reg.control4);
    tsc2100_regwrite(TSC2100_REG_CONTROL5,tsc2100_devdata->reg.control5);
    tsc2100_regwrite(TSC2100_REG_AUDIOADC,tsc2100_devdata->reg.audioadc);
    tsc2100_regwrite(TSC2100_REG_AUDIODAC,tsc2100_devdata->reg.audiodac);

    printk(KERN_INFO "Chumby TI-TSC2100 ALSA Audio Driver v1.02 initialized\n");
    tsc2100_sndcard = card;
    return 0;

 init_err1:
    printk( KERN_ERR "%s/%s(): failed init_err1\n", __FILE__, __FUNCTION__ );
    imx_dma_free(chip->p_dma);
 init_err0:
    printk( KERN_ERR "%s/%s(): failed init_err0, rc=%d\n", 
            __FILE__, __FUNCTION__, rc );
    snd_card_free(card);
    return rc;
}


static void __exit touchscreen_exit(void)
{
    if ( tsc2100_devdata ) {
        tasklet_disable( &tsc2100_devdata->tasklet );
        free_irq(PENIRQ, tsc2100_devdata);
        del_timer_sync(&tsc2100_devdata->timer);
        del_timer_sync(&tsc2100_devdata->oneshot);
        input_unregister_device(tsc2100_devdata->inputdevice);
        touchscreen_disable(tsc2100_devdata);
        spi_exit();
        tasklet_kill( &tsc2100_devdata->tasklet );
        kfree(tsc2100_devdata);
        tsc2100_devdata = NULL;
    }
}


static void __exit alsa_audio_exit(void)
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
    proc_exit(tsc2100_devdata);
    alsa_audio_exit();
    touchscreen_exit();
    #ifdef PLAY_DEBUG_SOUND
    tsc2100_debug_sound_done();
    #endif
    printk(KERN_INFO "Chumby TI-TSC2100 ALSA Audio and TouchScreen Driver removed\n");
}


static int __init chumby_tsc2100_init(void)
{
    int rc;

    rc = touchscreen_init();
    if ( rc )
        return rc;
    rc = alsa_audio_init();
    if ( rc )
        touchscreen_exit();
    else
        proc_init(tsc2100_devdata);
    return rc;
}


module_init(chumby_tsc2100_init);
module_exit(chumby_tsc2100_exit);

MODULE_AUTHOR("Chumby <ghutchins@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Chumby iMX21 TI-TSC2100 Driver (ALSA Audio & TouchScreen)");
MODULE_PARM_DESC(id, "Chumby iMX21/TSC2100");
