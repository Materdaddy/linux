/*
 *  linux/arch/arm/mach-imx/time.c
 *
 *  Copyright (C) 2000-2001 Deep Blue Solutions
 *  Copyright (C) 2002 Shane Nay (shane@minirl.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/time.h>

#include <linux/proc_fs.h>

#include <asm/procinfo.h>
#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/leds.h>
#include <asm/irq.h>
#include <asm/mach/time.h>

/*
 * On most boards, we use Timer 1 as the system timer. However,
 * the Turbochef board needs timer 1's output for a perifperal, so
 * it uses Timer 2 as the sytem timer.
 */
#if defined(CONFIG_MACH_TCMX21)
#  define TIMER_BASE IMX_TIM2_BASE
#  define TIMER_INT  TIM2_INT
#else
#  define TIMER_BASE IMX_TIM1_BASE
#  define TIMER_INT  TIM1_INT
#endif



static int missed_ticks = 0;
static int tick_queue   = 0;
static int created_proc = 0;
static int total_ticks  = 0;


static int missed_timer_proc_output (char *buf) {
  int printlen = 0;

  // insert proc debugging output here
  printlen = sprintf(buf, "Tick compensator v1.4\n"
                          "Synthesized %d missed ticks, %d ticks in queue\n",
                          missed_ticks, tick_queue);

  return printlen;
}


static int missed_timer_read_proc(char *page, char **start, off_t off,
                         int count, int *eof, void *data)
{
        int len = missed_timer_proc_output (page);
        if (len <= off+count) *eof = 1;
        *start = page + off;
        len -= off;
        if (len>count) len = count;
        if (len<0) len = 0;
        return len;
}




/*
 * Returns number of us since last clock interrupt.  Note that interrupts
 * will have been disabled by do_gettimeoffset()
 */
static unsigned long imx_gettimeoffset(void)
{
	unsigned long ticks;

	/*
	 * Get the current number of ticks.  Note that there is a race
	 * condition between us reading the timer and checking for
	 * an interrupt.  We get around this by ensuring that the
	 * counter has not reloaded between our two reads.
	 */
	ticks = IMX_TCN(TIMER_BASE);

	/*
	 * Interrupt pending?  If so, we've reloaded once already.
	 */
	if (IMX_TSTAT(TIMER_BASE) & TSTAT_COMP)
		ticks += LATCH;


	/*
	 * Convert the ticks to usecs
	 */
	return (1000000 / CLK32) * ticks;
}

/*
 * IRQ handler for the timer
 */
static irqreturn_t
imx_timer_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
    int _missed_ticks = 0;

	write_seqlock(&xtime_lock);

	/* clear the interrupt */
	if (IMX_TSTAT(TIMER_BASE))
#if defined(CONFIG_ARCH_IMX)
		IMX_TSTAT(TIMER_BASE) = 0;
#elif defined(CONFIG_ARCH_IMX21)
                IMX_TSTAT(TIMER_BASE) = TSTAT_CAPT | TSTAT_COMP;
#endif

    /* Figure out how many ticks have passed since the last interrupt.  Do
     * this by querying the secondary timer to see how much time has past.
     * Each LATCH unit means a single tick.  That means we lost
     * TIMER3_VAL/LATCH ticks.
     */
	_missed_ticks = (IMX_TCN(IMX_TIM3_BASE) / LATCH)-1;


    /* Reset the secondary timer. */
	IMX_TCTL(IMX_TIM3_BASE) = TCTL_CLK_32 |            TCTL_FRR | TCTL_CC;
	IMX_TCTL(IMX_TIM3_BASE) = TCTL_CLK_32 | TCTL_TEN | TCTL_FRR | TCTL_CC;


    /* Add the missed ticks to the queue of ticks to correct. */
    if( _missed_ticks > 2 )
        printk("Corrected %d missed timer ticks\n", _missed_ticks);
    if( _missed_ticks > 0 )
        tick_queue += _missed_ticks;


    /* Now simulate the missed ticks. */
    if(tick_queue > 0) {
        missed_ticks++;
        tick_queue--;
        timer_tick(regs);
    }
    else if(tick_queue < 0) {
        printk("Tick queue went negative!  Currently at %d\n", tick_queue);
        tick_queue=0;
    }


    /* Now do the /real/ tick. */
	timer_tick(regs);
	write_sequnlock(&xtime_lock);


    /* Create the proc entry, if it hasn't been created yet */
    total_ticks++;
    if( !created_proc && total_ticks > 1000 ) {
        created_proc = 1;
        /*
        * Allow users to determine how many ticks were fixed.
        */
        create_proc_read_entry ("missed_ticks", 0, 0, missed_timer_read_proc, NULL);
    }

	return IRQ_HANDLED;
}

static struct irqaction imx_timer_irq = {
	.name		= "i.MX Timer Tick",
	.flags		= SA_INTERRUPT | SA_TIMER,
	.handler	= imx_timer_interrupt,
};

/*
 * Set up timer interrupt, and return the current time in seconds.
 */
static void __init imx_timer_init(void)
{
	/*
	 * Initialise to a known state (all timers off, and timing reset)
	 */
	IMX_TCTL(TIMER_BASE) = 0;
	IMX_TPRER(TIMER_BASE) = 0;
	IMX_TCMP(TIMER_BASE) = LATCH - 1;
	IMX_TCTL(TIMER_BASE) = TCTL_CLK_32 | TCTL_IRQEN | TCTL_TEN;

    /*
     * Initialise a second timer that we can use to keep track of lost
     * interrupts
     */
	IMX_TCTL(IMX_TIM3_BASE) = 0;
	IMX_TPRER(IMX_TIM3_BASE) = 0;
//	IMX_TCMP(IMX_TIM3_BASE) = LATCH - 1;
	IMX_TCTL(IMX_TIM3_BASE) = TCTL_CLK_32 | TCTL_TEN | TCTL_FRR | TCTL_CC;


	/*
	 * Make irqs happen for the system timer
	 */
	setup_irq(TIMER_INT, &imx_timer_irq);


}

struct sys_timer imx_timer = {
	.init		= imx_timer_init,
	.offset		= imx_gettimeoffset,
};
