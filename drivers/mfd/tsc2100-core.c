/*
 * Cloned from TSC2101 driver for Chumby TSC2100
 *
 * TI TSC2102 Common Code
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

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/mfd/tsc2100.h> 
#include "tsc2100.h"


extern void tsc2100_ts_setup(struct device *dev);
extern void tsc2100_ts_report(struct tsc2100_data *devdata, 
                              int x, int y, int p, int pendown);

//#define TSC2100_DEBUG       1
#define SPIDEV              2
#define PENIRQ              IRQ_GPIOE(19)

#define TSC2100_ADC_DEFAULT (TSC2100_ADC_RES(TSC2100_ADC_RES_12BITP)   | \
                             TSC2100_ADC_AVG(TSC2100_ADC_4AVG)         | \
                             TSC2100_ADC_CL(TSC2100_ADC_CL_1MHZ_12BIT) | \
                             TSC2100_ADC_PV(TSC2100_ADC_PV_500us)      | \
                             TSC2100_ADC_AVGFILT_MEAN) 


static struct tsc2100_data *tsc2100_devdata = NULL;


static void gpio_init( void )
{
    imx_gpio_mode( 18 | GPIO_PORTE | GPIO_AF | GPIO_IN );
    imx_gpio_mode( 21 | GPIO_PORTE | GPIO_AF | GPIO_OUT );
    imx_gpio_mode( 22 | GPIO_PORTE | GPIO_AF | GPIO_OUT );
    imx_gpio_mode( 23 | GPIO_PORTE | GPIO_AF | GPIO_OUT );
    imx_gpio_mode( 19 | GPIO_PORTE | GPIO_PF | GPIO_IN | GPIO_PUEN | GPIO_GPIO );
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


static int tsc2100_pendown( void )
{
    return tsc2100_regread(TSC2100_REG_STATUS) & TSC2100_STATUS_DAVAIL;
}


static void tsc2100_readdata(struct tsc2100_data *devdata, 
                             struct tsc2100_ts_event *ts_data)
{
    int z1,z2;
    u32 values[4];

    if (!tsc2100_pendown())
        return;

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
        ts_data->p = ((ts_data->x * (z2 -z1) / z1));
    else
        ts_data->p=0;

    #ifdef TSC2100_DEBUG
    printk( "%s/%s(): X=%d, Y=%d, Z1=%d, Z2=%d, p=%d\n",
            __FILE__, __FUNCTION__, values[0], values[1], values[2], values[3], 
            ts_data->p );
    #endif
}


static void tsc2100_ts_enable(struct tsc2100_data *devdata)
{
    tsc2100_regwrite(TSC2100_REG_RESETCTL, 0xbb00);
    
    /* PINTDAV is data available only 
     */
    tsc2100_regwrite(TSC2100_REG_STATUS, 0x4000);

    /* TSC2100-controlled conversions
     * 12-bit samples
     * continuous X,Y,Z1,Z2 scan mode
     * average (mean) 4 samples per coordinate
     * 1 MHz internal conversion clock
     * 500 usec panel voltage stabilization delay
     */
    tsc2100_regwrite(TSC2100_REG_ADC, 
              TSC2100_ADC_DEFAULT | TSC2100_ADC_PSM | TSC2100_ADC_ADMODE(0x2));
}


static void tsc2100_ts_disable(struct tsc2100_data *devdata)
{
    /* stop conversions and power down 
     */
    tsc2100_regwrite(TSC2100_REG_ADC, 0x4000);
}


static void ts_interrupt_main(struct tsc2100_data *devdata, 
                              int isTimer, struct pt_regs *regs)
{
    unsigned long flags;
    struct tsc2100_ts_event ts_data;

    spin_lock_irqsave(&devdata->lock, flags);

    if (tsc2100_pendown()) {
        /* PEN is down, this is either the first PEN down (!isTimer), or
         * the PEN still down 10ms later (isTimer) -- in either case, report
         * the PEN down event
         */
        devdata->pendown = 1;
        tsc2100_readdata(devdata, &ts_data);
        tsc2100_ts_report(devdata, ts_data.x, ts_data.y, ts_data.p, 1);
        mod_timer(&(devdata->ts_timer), jiffies + HZ / 100);
    } 
    else if (devdata->pendown > 0 && devdata->pendown < 3) {
        /* time the 10ms timer events, after 3, report PEN up event
        */
        mod_timer(&(devdata->ts_timer), jiffies + HZ / 100);
        devdata->pendown++;
    } else {
        if (devdata->pendown) 
        {
            #ifdef TSC2100_DEBUG
            printk( "%s/%s(): PENUP(%d)!\n", __FILE__, __FUNCTION__, 
                    devdata->pendown );
            #endif
            devdata->pendown = 0;
            tsc2100_ts_report(devdata, 0, 0, 0, 0);
        }

        /* okay, PEN up event has been reported, now change back to falling
         * edge to report new PEN down events -- 
         * make sure no data is missed after set_irq_type()
         */
        set_irq_type(PENIRQ, IRQT_FALLING);
        if (tsc2100_pendown()) {
            tsc2100_readdata(devdata, &ts_data);
            mod_timer(&(devdata->ts_timer), jiffies + HZ / 100);
        } 
    }
    spin_unlock_irqrestore(&devdata->lock, flags);
}


static void tsc2100_timer(unsigned long data)
{
    struct tsc2100_data *devdata = (struct tsc2100_data *) data;

    ts_interrupt_main(devdata, 1, NULL);
}


static irqreturn_t tsc2100_handler(int irq, void *dev_id, struct pt_regs *regs)
{
    struct tsc2100_data *devdata = dev_id;

    /* Switch from falling edge trigger to level, we are playing games with the
     * interrupt type to reduce the number of interrupts received while the PEN
     * is down, we want to read the first sample of the PEN selection and then 
     * we want to report PEN up events some short sequence later (10-30ms) and 
     * not have to remain in the interrupt handler reading all of the 
     * new X/Y/Z1/Z2 samples...
     */
    set_irq_type(PENIRQ, IRQT_NOEDGE);
    ts_interrupt_main(devdata, 0, regs);
    return IRQ_HANDLED;
}


static int __init tsc2100_init(void)
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
    init_timer(&devdata->ts_timer);
    devdata->ts_timer.data = (unsigned long) devdata;
    devdata->ts_timer.function = tsc2100_timer;

    gpio_init();
    spi_init();
    rc = request_irq(PENIRQ, tsc2100_handler, 0, "tsc2100", devdata);
    if (rc) {
        printk(KERN_ERR "tsc2100: Could not allocate touchscreen IRQ %d!\n",
               PENIRQ);
        kfree(devdata);
        return -EINVAL;
    }

    tsc2100_ts_setup(&tsc2100dev);
    tsc2100_ts_enable(devdata);

    /* Falling edge indicates PEN has been pressed and a new X/Y/Z1/Z2 sample
     * is now ready (note we read the data here simply to flush and discard).
     */
    set_irq_type(PENIRQ, IRQT_FALLING);
    tsc2100_readdata(devdata, &ts_data);

    tsc2100_devdata = devdata;
    return 0;
}


static void __exit tsc2100_exit(void)
{
    struct tsc2100_data *devdata = tsc2100_devdata;

    free_irq(PENIRQ, devdata);
    del_timer_sync(&devdata->ts_timer);
    input_unregister_device(devdata->inputdevice);
    tsc2100_ts_disable(devdata);
    spi_exit();
    kfree(devdata);
}


module_init(tsc2100_init);
module_exit(tsc2100_exit);

MODULE_LICENSE("GPL");
