/*
 * Cloned from TSC2100 tsc2101.h for the Chumby TSC2100.
 *
 * TI TSC2100 Hardware definitions
 *
 * Copyright 2005 Openedhand Ltd.
 *
 * Author: Richard Purdie <richard@o-hand.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef _TSC2100_H_
#define _TSC2100_H_

// TODO: comment the ENABLE_SPEAKER_KLUDGE... #define out once the 
// speaker enable/disable is controlled by the other driver
#define ENABLE_SPEAKER_KLUDGE_UNTIL_OTHER_DRIVER_PORTED


/* Address constructs */
#define TSC2100_READ     (1 << 15)    /* Read Register */
#define TSC2100_WRITE    (0 << 15)    /* Write Register */
#define TSC2100_PAGE(x)  ((x & 0xf) << 11) /* Memory Page to access */
#define TSC2100_ADDR(x)  ((x & 0x3f) << 5) /* Memory Address to access */

#define TSC2100_P0_REG(x) (TSC2100_PAGE(0) | TSC2100_ADDR(x))
#define TSC2100_P1_REG(x) (TSC2100_PAGE(1) | TSC2100_ADDR(x))
#define TSC2100_P2_REG(x) (TSC2100_PAGE(2) | TSC2100_ADDR(x))
#define TSC2100_P3_REG(x) (TSC2100_PAGE(3) | TSC2100_ADDR(x))

/* Page 0 Registers */
#define TSC2100_REG_X      TSC2100_P0_REG(0x0)
#define TSC2100_REG_Y      TSC2100_P0_REG(0x1)
#define TSC2100_REG_Z1     TSC2100_P0_REG(0x2)
#define TSC2100_REG_Z2     TSC2100_P0_REG(0x3)
#define TSC2100_REG_BAT1   TSC2100_P0_REG(0x5)
#define TSC2100_REG_BAT2   TSC2100_P0_REG(0x6)
#define TSC2100_REG_AUX    TSC2100_P0_REG(0x7)
#define TSC2100_REG_TEMP1  TSC2100_P0_REG(0x9)
#define TSC2100_REG_TEMP2  TSC2100_P0_REG(0xa)

/* Page 1 Registers */
#define TSC2100_REG_ADC       TSC2100_P1_REG(0x0)
#define TSC2100_REG_STATUS    TSC2100_P1_REG(0x1)
#define TSC2100_REG_REFCTL    TSC2100_P1_REG(0x3)
#define TSC2100_REG_RESETCTL  TSC2100_P1_REG(0x4)
#define TSC2100_REG_CONFCTL   TSC2100_P1_REG(0x5)

/* Page 2 Registers */
#define TSC2100_REG_CONTROL1    TSC2100_P2_REG(0x0)
#define TSC2100_REG_AUDIOADC    TSC2100_P2_REG(0x1)
#define TSC2100_REG_AUDIODAC    TSC2100_P2_REG(0x2)
#define TSC2100_REG_SIDETONE    TSC2100_P2_REG(0x3)
#define TSC2100_REG_CONTROL2    TSC2100_P2_REG(0x4)
#define TSC2100_REG_POWERCON    TSC2100_P2_REG(0x5)
#define TSC2100_REG_CONTROL3    TSC2100_P2_REG(0x6)
#define TSC2100_REG_DAEFC(x)	TSC2100_P2_REG(0x7+x)
#define TSC2100_REG_PLL1        TSC2100_P2_REG(0x1b)
#define TSC2100_REG_PLL2        TSC2100_P2_REG(0x1c)
#define TSC2100_REG_CONTROL4    TSC2100_P2_REG(0x1d)
#define TSC2100_REG_CONTROL5    TSC2100_P2_REG(0x1e)

/* Page 2 Registers */
#define TSC2100_REG_BUFLOC(x)	TSC2100_P3_REG(x)

/* Status Register Masks */

#define TSC2100_STATUS_T2STAT   (1 << 1)
#define TSC2100_STATUS_T1STAT   (1 << 2)
#define TSC2100_STATUS_AXSTAT   (1 << 4)
#define TSC2100_STATUS_B2STAT   (1 << 5)
#define TSC2100_STATUS_B1STAT   (1 << 6)
#define TSC2100_STATUS_Z2STAT   (1 << 7)
#define TSC2100_STATUS_Z1STAT   (1 << 8)
#define TSC2100_STATUS_YSTAT    (1 << 9)
#define TSC2100_STATUS_XSTAT    (1 << 10)
#define TSC2100_STATUS_DAVAIL   (1 << 11)     
#define TSC2100_STATUS_HCTLM    (1 << 12)
#define TSC2100_STATUS_PWRDN    (1 << 13)
#define TSC2100_STATUS_PINTDAV_SHIFT (14)
#define TSC2100_STATUS_PINTDAV_MASK  (0x03)



// TSC2100_REG_ADC (page01 reg00 TouchScreen ADC Control)
#define TSC2100_ADC_PSM (1<<15) // pen status mode on ctrlreg adc
#define TSC2100_ADC_STS (1<<14) // stop continuous scanning.
#define TSC2100_ADC_AD3 (1<<13) 
#define TSC2100_ADC_AD2 (1<<12) 
#define TSC2100_ADC_AD1 (1<<11) 
#define TSC2100_ADC_AD0 (1<<10) 
#define TSC2100_ADC_ADMODE(x) ((x<<10) & TSC2100_ADC_ADMODE_MASK)
#define TSC2100_ADC_ADMODE_MASK (0xf<<10)  

#define TSC2100_ADC_RES(x) ((x<<8) & TSC2100_ADC_RES_MASK )
#define TSC2100_ADC_RES_MASK (0x3<<8)  
#define TSC2100_ADC_RES_12BITP (0) // 12-bit ADC resolution (default)
#define TSC2100_ADC_RES_8BIT (1) // 8-bit ADC resolution
#define TSC2100_ADC_RES_10BIT (2) // 10-bit ADC resolution
#define TSC2100_ADC_RES_12BIT (3) // 12-bit ADC resolution

#define TSC2100_ADC_AVG(x) ((x<<6) & TSC2100_ADC_AVG_MASK )
#define TSC2100_ADC_AVG_MASK (0x3<<6)  
#define TSC2100_ADC_NOAVG (0) //  a-d does no averaging
#define TSC2100_ADC_4AVG (1) //  a-d does averaging of 4 samples
#define TSC2100_ADC_8AVG (2) //  a-d does averaging of 8 samples
#define TSC2100_ADC_16AVG (3) //  a-d does averaging of 16 samples

#define TSC2100_ADC_CL(x) ((x<<4) & TSC2100_ADC_CL_MASK )
#define TSC2100_ADC_CL_MASK (0x3<<4)
#define TSC2100_ADC_CL_8MHZ_8BIT (0)
#define TSC2100_ADC_CL_4MHZ_10BIT (1)
#define TSC2100_ADC_CL_2MHZ_12BIT (2)
#define TSC2100_ADC_CL_1MHZ_12BIT (3)
#define TSC2100_ADC_CL0 (1<< 4)  

/* ADC - Panel Voltage Stabilisation Time */
#define TSC2100_ADC_PV(x)     ((x<<1) & TSC2100_ADC_PV_MASK )
#define TSC2100_ADC_PV_MASK   (0x7<<1)
#define TSC2100_ADC_PV_100ms  (0x7)  /* 100ms */
#define TSC2100_ADC_PV_50ms   (0x6)  /* 50ms  */
#define TSC2100_ADC_PV_10ms   (0x5)  /* 10ms  */
#define TSC2100_ADC_PV_5ms    (0x4)  /* 5ms   */
#define TSC2100_ADC_PV_1ms    (0x3)  /* 1ms   */
#define TSC2100_ADC_PV_500us  (0x2)  /* 500us */
#define TSC2100_ADC_PV_100us  (0x1)  /* 100us */
#define TSC2100_ADC_PV_0s     (0x0)  /* 0s    */

#define TSC2100_ADC_AVGFILT_MEAN     (0<<0)  /* Mean Average Filter */
#define TSC2100_ADC_AVGFILT_MEDIAN   (1<<0)  /* Median Average Filter */

#define TSC2100_ADC_x (1<< 0) // don't care

#define TSC2100_CONFIG_DAV (1<<6)

#define TSC2100_KEY_STC (1<<15) // keypad status
#define TSC2100_KEY_SCS (1<<14) // keypad scan status


// TSC2100_REG_CONTROL1 (page02 reg00 Audio Control1)
#define TSC2100_CONTROL1_ADCIN_MASK    (3<<12)
#define TSC2100_CONTROL1_ADCIN(x)      ((x)<<12)
#define TSC2100_CONTROL1_WLEN_MASK     (3<<10)
#define TSC2100_CONTROL1_WLEN(x)       ((x)<<10)
#define TSC2100_CONTROL1_DATFM_MASK    (3<<8)
#define TSC2100_CONTROL1_DATFM_I2S     (0<<8)


#define SCAN_XY      0x10           /* SCAN_XY issued */
#define HAVE_X       0x20           /* received X value */
#define HAVE_Y       0x40           /* received Y value */
#define HAVE_XY      (HAVE_X|HAVE_Y)/* have boty X&Y values */
#define REPORT_XY    0x80           /* generated PEN DOWN event to inputdev */

struct tsc2100_event {
    short flags;    /* SCAN_XY ... REPORT_XY */
	short p;        /* pen pressure */
	short x;        
	short y;
    short pen;      /* non-zero if PEN DOWN */

    #ifdef REAL_PEN_PRESSURE
    short z1;
    short z2;
    #endif
};


struct tsc2100_ringbuf {
    u16 data[ 256 ];
    int next;
    int wrap;
};


struct tsc2100_regs {
    u16 adc;
    u16 refctl;
    u16 control1;
    u16 audioadc;
    u16 audiodac;
    u16 sidetone;
    u16 control2;
    u16 powercon;
    u16 control3;
    u16 pll1;
    u16 pll2;
    u16 control4;
    u16 control5;
};


/* scan mode */
#define TS_IDLE     0   /* nothing happening */
#define TS_SCANXY   1   /* scanning for X/Y coordinates */
#define TS_SCAN     2   /* scanning for BAT1/BAT2/TEMP1/TEMP2 */
#define TS_DISABLED 3   /* touchscreen is disabled */

/* scan state when in TS_SCAN scan mode */
#define TS_TEMP1    0   /* scanning for temp1 */
#define TS_TEMP2    1   /* scanning for temp2 */
#define TS_DCLINE   2   /* scanning for DC power */
#define TS_BATTERY  3   /* scanning for battery power */


struct tsc2100_context {
    spinlock_t              lock;
    struct tasklet_struct   tasklet;
    struct timer_list       oneshot;
    struct timer_list       timer;

    int                     interrupt;
    int                     oneshot_expired;
    int                     timer_expired;
    int                     tson;

    int                     scan_mode;  /* TS_IDLE...TS_SCAN */
    int                     scan_state; /* TS_TEMP1...TS_BATTERY */
    unsigned long           scan_start; /* scan start in jiffies */
    int                     scan_sample;/* average TSC2100 provided raw data */
    u16                     scan_data[8];

    int                     penup;           /* # 10ms ticks pen has been up */
    int                     pendown_active;  /* non-zero if processing pen down */
    int                     pendown_reports; /* # of pendown reports for this PEN DOWN event */
    int                     pendown_total;   /* total # of pendown reports */

    int                     touchclick;

    int                     dac_sample_rate;
    int                     adc_sample_rate;

    struct tsc2100_regs     reg;

    struct tsc2100_ringbuf  ring_x;
    struct tsc2100_ringbuf  ring_y;
    struct tsc2100_ringbuf  ring_p;
    struct tsc2100_ringbuf  ring_pen;
    struct tsc2100_ringbuf  ring_bat1;
    struct tsc2100_ringbuf  ring_bat2;
    struct tsc2100_ringbuf  ring_temp1;
    struct tsc2100_ringbuf  ring_temp2;

    struct input_dev*       inputdevice;

    struct tsc2100_event    event;
    
    struct proc_dir_entry*  proc_dirs[ 16 ];
    struct proc_dir_entry*  proc_files[ 64 ];
};


#define TSC2100_SPIDEV 2

extern struct tsc2100_context *tsc2100_devdata;


static inline void ringwrite( struct tsc2100_ringbuf* ring, u16 newdata )
{
    ring->data[ ring->next ] = newdata;
    ring->next = ( ring->next + 1 ) % ARRAY_SIZE( ring->data );
    if ( ring->next == 0 ) ring->wrap = 1;
}


static inline u16 ringread( struct tsc2100_ringbuf* ring, int* index )
{
    u16 data = ring->data[ *index ];
    *index = ( *index - 1 ) % ARRAY_SIZE( ring->data );
    return data;
}


static inline int ringbeg( struct tsc2100_ringbuf* ring )
{
    return ( ring->next - 1 ) % ARRAY_SIZE( ring->data );
}


static inline int ringcnt( struct tsc2100_ringbuf* ring )
{
    return ring->wrap? ARRAY_SIZE( ring->data ) : ring->next;
}


static inline void enable_speaker( void )
{
    #ifdef ENABLE_SPEAKER_KLUDGE_UNTIL_OTHER_DRIVER_PORTED
    imx_gpio_write( GPIO_PORTKP | (4 + 8), 0 );
    #endif
}


static inline void disable_speaker( void )
{
    #ifdef ENABLE_SPEAKER_KLUDGE_UNTIL_OTHER_DRIVER_PORTED
    imx_gpio_write( GPIO_PORTKP | (4 + 8), 1 );
    #endif
}


static inline void spi_wait_for_idle( void )
{
    while (SSP_CTRL_REG(TSC2100_SPIDEV) & SSP_XCH)
        ndelay(50);
}


static inline void spi_flush_fifo( void )
{
    volatile int junk;
    while (SSP_INT_REG(TSC2100_SPIDEV) & SSP_INT_RR)
        junk = SSP_RX_REG(TSC2100_SPIDEV);
}


static inline void spi_read_fifo(int* rxdata, int rxlen)
{
    while (rxlen > 0)
        while (SSP_INT_REG(TSC2100_SPIDEV) & SSP_INT_RR) {
            *rxdata++ = SSP_RX_REG(TSC2100_SPIDEV) & 0xffff;
            if (--rxlen <= 0)
                break;
        }
}


static inline void spi_write_zeros(int zeros)
{
    while (zeros-- > 0)
        SSP_TX_REG(TSC2100_SPIDEV) = 0;
}


static inline void spi_wait_for_txspace( void ) 
{
  while( !(SSP_INT_REG(TSC2100_SPIDEV) & SSP_INT_TH ) )
    ndelay(50); // wait 50 nsecs...
}

    
static inline int spi_write( int regaddr, int regdata )
{
    unsigned long flags;

    spin_lock_irqsave(&tsc2100_devdata->lock, flags);
    spi_wait_for_txspace(); // check added due to ambiguity in datasheet about XCHG operation
    spi_wait_for_idle();
    spi_flush_fifo();
    SSP_TX_REG(TSC2100_SPIDEV) = regaddr | TSC2100_WRITE;
    SSP_TX_REG(TSC2100_SPIDEV) = regdata;
    if( !(((SSP_TEST_REG(TSC2100_SPIDEV) & 0xF) == 1)) )
      printk( "***** (w)TX fifo count: %d\n", SSP_TEST_REG(TSC2100_SPIDEV) & 0xF );
    SSP_CTRL_REG(TSC2100_SPIDEV) |= SSP_XCH;
    spin_unlock_irqrestore(&tsc2100_devdata->lock, flags);
    return 0;
}


static inline int spi_read( int regaddr, int* rxdata, int rxlen )
{
    int junk;
    unsigned long flags;

    spin_lock_irqsave(&tsc2100_devdata->lock, flags);
    if ( rxlen > 7 )  
        rxlen = 7;
    spi_wait_for_txspace(); // check added due to ambiguity in datasheet about XCHG operation
    spi_wait_for_idle();
    spi_flush_fifo();
    SSP_TX_REG(TSC2100_SPIDEV) = regaddr | TSC2100_READ;
    spi_write_zeros(rxlen);
    if ( (SSP_TEST_REG(TSC2100_SPIDEV) & 0xF) != rxlen )
      printk( "***** (r)TX fifo count: %d != %d\n", SSP_TEST_REG(TSC2100_SPIDEV) & 0xF, rxlen );
    SSP_CTRL_REG(TSC2100_SPIDEV) |= SSP_XCH;
    spi_read_fifo(&junk, 1);
    spi_read_fifo(rxdata, rxlen);
    spin_unlock_irqrestore(&tsc2100_devdata->lock, flags);
    return rxdata[0];
}


static inline int tsc2100_regread(int regnum)
{
    int reg;
    return spi_read(regnum, &reg, 1);
}


static inline void tsc2100_regwrite(int regnum, int value)
{
    spi_write( regnum, value );
}

#endif
