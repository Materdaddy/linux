/*
	chumby_sense1.c
	bunnie -- June 2006 -- 1.4 -- linux 2.4.20
	bunnie -- March 2007 -- 2.0 -- port to Ironforge linux 2.6.16
        bunnie -- December 2007 -- 2.1 -- port to v3.8 and inclusion of watchdog timer feature, reset reason reporting

	This file is part of the chumby sensor suite driver in the linux kernel.
	Copyright (c) Chumby Industries, 2007

	The sensor suite driver is free software; you can redistribute it and/or
	modify it under the terms of the GNU General Public License as published
	by the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	The sensor suite driver is distributed in the hope that it will be
	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License along
	with the Chumby; if not, write to the Free Software
	Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/

#define SENSE1_VERSION "2.1-Ironforge"

#include <linux/config.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>

#include <linux/kernel.h>	/* printk() */
#include <linux/slab.h>		/* kmalloc() */
#include <linux/fs.h>		/* everything... */
#include <linux/errno.h>	/* error codes */
#include <linux/types.h>	/* size_t */
#include <linux/proc_fs.h>
#include <linux/fcntl.h>	/* O_ACCMODE */
#include <linux/seq_file.h>
#include <linux/cdev.h>

#include <asm/io.h>
#include <asm/system.h>		/* cli(), *_flags */
#include <asm/uaccess.h>	/* copy_*_user */

#include <linux/miscdevice.h>
#include <linux/ioport.h>
#include <linux/poll.h>
#include <linux/sysctl.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/rtc.h>

#include <linux/timer.h>

#include <asm/arch/imx-regs.h>

#include "chumby_sense1.h"

/*
 * basic parameters
 */

int sense1_major = 0;		// dynamic allocation
int sense1_minor = 0;
int sense1_nr_devs = 1;

module_param(sense1_major, int, S_IRUGO);
module_param(sense1_minor, int, S_IRUGO);
module_param(sense1_nr_devs, int, S_IRUGO);

MODULE_AUTHOR("bunnie@chumby.com");
MODULE_LICENSE("GPL");

// static data
static int gDone = 0;
static unsigned long sense1_status = 0;	/* bitmapped status byte.       */

/*
 * Sense1 sensor data logs, tracked by tasks that are scheduled by the
 * task scheduler
 */

#define FIXEDPOINT_NORM 1000

struct sense1data {
	unsigned char bent;
	unsigned char isDebounce;
	int lastTransitionState;
	unsigned long lastTransitionTime;
	unsigned long currentTransitionTime;
	struct timer_list timer;
	struct cdev *sense1_cdev;
} sense1task_data;

#define USE_SYSCTL  1

#define SENSE1_BLOCKING 0	// turn off blocking read on sense1 sensor

#if 1
#   if USE_SYSCTL
#       define CHUMSENSE1_DEBUG( flag, fmt, args... ) do { if ( gDebug ## flag ) printk( "%s: " fmt, __FUNCTION__ , ## args ); } while (0)
#   else
#       define CHUMSENSE1_DEBUG( flag, fmt, args... ) printk( "%s: " fmt, __FUNCTION__ , ## args )
#   endif
#else
#   define CHUMSENSE1_DEBUG( flag, fmt, args... )
#endif

// function protos
static int chumby_sense1_open(struct inode *inode, struct file *file);
static int chumby_sense1_release(struct inode *inode, struct file *file);
static int sense1_read_proc(char *page, char **start, off_t off,
			    int count, int *eof, void *data);
static int resetReason_proc(char *page, char **start, off_t off,
			    int count, int *eof, void *data);
static ssize_t chumby_sense1_read(struct file *file, char *buf,
				  size_t count, loff_t * ppos);
//static ssize_t chumby_sense1_write(struct file *file, const unsigned char *buf,
//                               size_t count, loff_t *ppos );
//static unsigned int chumby_sense1_poll(struct file * filp, struct poll_table_struct * wait);

#if USE_SYSCTL

// proc debug structure
static int gDebugTrace = 0;
static int gDebugIoctl = 0;
static int gDebugError = 1;
static unsigned int gSpkMute = 0;
static int gDCmillivolts = -1;
static int gBTmillivolts = -1;
static unsigned int gDimLevel = 0xFFFF;
static unsigned int gHPin = 0;
static unsigned int gDebounce = 10;
static unsigned int gResetSerial = 0;

static unsigned short resetReason = 0;

// create /proc/sys/debug-trace, etc...which can be written to set behavior of globals
// in this driver on the fly
static struct ctl_table_header *gSysCtlHeader;
static struct ctl_table gSysCtlChumsense1[] = {
	{CTL_CHUMSENSE1_DEBUG_TRACE, "dbg-trace", &gDebugTrace, sizeof(int), 0644, NULL, &proc_dointvec},
	{CTL_CHUMSENSE1_DEBUG_IOCTL, "dbg-ioctl", &gDebugIoctl, sizeof(int), 0644, NULL, &proc_dointvec},
	{CTL_CHUMSENSE1_DEBUG_ERROR, "dbg-error", &gDebugError, sizeof(int), 0644, NULL, &proc_dointvec},
	{CTL_CHUMSENSE1_SPKMUTE, "spkmute", &gSpkMute, sizeof(unsigned int), 0644, NULL, &proc_dointvec},
	{CTL_CHUMSENSE1_DIMLEVEL, "dimlevel", &gDimLevel, sizeof(unsigned int), 0644, NULL, &proc_dointvec},
	{CTL_CHUMSENSE1_DCPOWER, "DCvolts", &gDCmillivolts, sizeof(unsigned int), 0644, NULL, &proc_dointvec},
	{CTL_CHUMSENSE1_BTPOWER, "BTvolts", &gBTmillivolts, sizeof(unsigned int), 0644, NULL, &proc_dointvec},
	{CTL_CHUMSENSE1_HPIN, "hpin", &gHPin, sizeof(unsigned int), 0644, NULL, &proc_dointvec},
	{CTL_CHUMSENSE1_DEBOUNCE, "debounce", &gDebounce, sizeof(unsigned int), 0644, NULL, &proc_dointvec},
	{CTL_CHUMSENSE1_RESETSERIAL, "resetSerial", &gResetSerial, sizeof(unsigned int), 0644, NULL, &proc_dointvec},
	{0}
};

static struct ctl_table gSysCtl[] = {
	{CTL_CHUMSENSE1, "sense1", NULL, 0, 0555, gSysCtlChumsense1},
	{0}
};
#endif				// USE_SYSCTL

// map into the generic driver infrastructure
static struct file_operations sense1_fops = {
	.owner =	THIS_MODULE,
//	.llseek =	chumby_sense1_llseek,
	.read =		chumby_sense1_read,
//	.poll =		chumby_sense1_poll,
//	.ioctl =	chumby_sense1_ioctl,
	.open =		chumby_sense1_open,
	.release =	chumby_sense1_release,
//	.write =	chumby_sense1_write,
//	.fasync =	chubmy_sense1_fasync,
};

///////////// code /////////////

static int chumby_sense1_release(struct inode *inode, struct file *file)
{
	CHUMSENSE1_DEBUG(Trace, "Top of release.\n");
	sense1_status &= ~SENSE1_IS_OPEN;
	return 0;
}

static int sense1_proc_output(char *buf)
{
	int printlen = 0;

	// insert proc debugging output here
	printlen =
	    sprintf(buf,
		    "Chumby sense1 driver version %s (bunnie@chumby.com)\n",
		    SENSE1_VERSION);
	buf += printlen;
	if (gDebugTrace)
		printlen += sprintf(buf, "  Debug trace is enabled.\n");
	else
		printlen += sprintf(buf, "  Debug trace is disabled.\n");

	// the code below is handy for debugging the watchdog timer, it crashes the kernel
	//	printk( "WSR: %04X, WRSR: %04X, WCR: %04X\n", WSR, WRSR, WCR );
	//	{ // force a hard crash of the kernel if possible....
	//	  int i;
	//	  for( i = 0xC0000000; i < 0xFFFFFF; i++ ) {
	//	    *(char *) i = 0x00;
	//	  }
	//	}
	return printlen;
}

static int sense1_read_proc(char *page, char **start, off_t off,
			    int count, int *eof, void *data)
{
	int len = sense1_proc_output(page);
	if (len <= off + count)
		*eof = 1;
	*start = page + off;
	len -= off;
	if (len > count)
		len = count;
	if (len < 0)
		len = 0;

	// insert printk's
	printk("UART test output\n");
	/*
	printk("URXD0(IMX_UART3_BASE): %08lX\n", (unsigned long)URXD0(IMX_UART3_BASE));
	printk("URTX0(IMX_UART3_BASE): %08lX\n", (unsigned long)URTX0(IMX_UART3_BASE));
	 */
	printk("UCR1 (IMX_UART3_BASE): %08lX\n", (unsigned long)UCR1(IMX_UART3_BASE));
	printk("UCR2 (IMX_UART3_BASE): %08lX\n", (unsigned long)UCR2(IMX_UART3_BASE));
	printk("UCR3 (IMX_UART3_BASE): %08lX\n", (unsigned long)UCR3(IMX_UART3_BASE));
	printk("UCR4 (IMX_UART3_BASE): %08lX\n", (unsigned long)UCR4(IMX_UART3_BASE));
	printk("UFCR (IMX_UART3_BASE): %08lX\n", (unsigned long)UFCR(IMX_UART3_BASE));
	printk("USR1 (IMX_UART3_BASE): %08lX\n", (unsigned long)USR1(IMX_UART3_BASE));
	printk("USR2 (IMX_UART3_BASE): %08lX\n", (unsigned long)USR2(IMX_UART3_BASE));
	printk("UESC (IMX_UART3_BASE): %08lX\n", (unsigned long)UESC(IMX_UART3_BASE));
	printk("UTIM (IMX_UART3_BASE): %08lX\n", (unsigned long)UTIM(IMX_UART3_BASE));
	printk("UBIR (IMX_UART3_BASE): %08lX\n", (unsigned long)UBIR(IMX_UART3_BASE));
	printk("UBMR (IMX_UART3_BASE): %08lX\n", (unsigned long)UBMR(IMX_UART3_BASE));
	printk("UBRC (IMX_UART3_BASE): %08lX\n", (unsigned long)UBRC(IMX_UART3_BASE));
	printk("ONMES(IMX_UART3_BASE): %08lX\n", (unsigned long)ONMES(IMX_UART3_BASE));
	printk("UTS  (IMX_UART3_BASE): %08lX\n", (unsigned long)UTS(IMX_UART3_BASE));

	return len;
}

static int resetReason_proc(char *page, char **start, off_t off,
			    int count, int *eof, void *data) {

  int len = 0;
  switch( (resetReason) & 0x1B ) {
  case 0x01:
    len = sprintf(page, "SOFTWARE_RESET\n");
    break;
  case 0x02:
    len = sprintf(page, "WATCHDOG_RESET\n");
    break;
  case 0x08:
    len = sprintf(page, "EXTERNAL_RESET\n");
    break;
  case 0x10:
    len = sprintf(page, "POWER_ON\n");
    break;
  default:
    len = sprintf(page, "INVALID_RESET\n");
    break;
  }

  if (len <= off + count)
    *eof = 1;
  *start = page + off;
  len -= off;
  if (len > count)
    len = count;
  if (len < 0)
    len = 0;

  return len;
}

static int chumby_sense1_open(struct inode *inode, struct file *file)
{
	// make sure we're not opened twice
	if (sense1_status & SENSE1_IS_OPEN)
		return -EBUSY;

	sense1_status |= SENSE1_IS_OPEN;
	return 0;
}

void sense1_task_record(unsigned long ptr)
{
	struct sense1data *data = (struct sense1data *)ptr;
	int state = 0;

	if (gResetSerial) {
		gResetSerial = 0;
		UCR2(IMX_UART3_BASE) &= 0xFFFFFFFE;
	}

	if (data->currentTransitionTime == 0xFFFFFFFF) {
		// handle roll-over case
		data->currentTransitionTime = 0;
		// this is cheesy but in practice this should work
		data->lastTransitionTime = 0;
	} else {
		data->currentTransitionTime++;
	}

	if (data->isDebounce) {
		// debouncing
		if (data->currentTransitionTime >
		    data->lastTransitionTime + gDebounce) {
			state = imx_gpio_read(GPIO_PORTKP | 0);

			if (state == data->lastTransitionState) {
				if ((state & 1) == 0)	// 0 if switch is depressed
					data->bent = 1;
				else
					data->bent = 0;
			} else {
				// ignore, it was spurious
			}
			// reset our state machine
			data->lastTransitionTime = data->currentTransitionTime;
			data->lastTransitionState = state;
			data->isDebounce = 0;
		} else {
			// just wait until our turn comes
		}
	} else {
		// not a debounce period
		state = imx_gpio_read(GPIO_PORTKP | 0);	// grab the current state
		if (state != data->lastTransitionState) {
			// something changed, enter the debounce state
			data->isDebounce = 1;
			data->lastTransitionState = state;
			data->lastTransitionTime = data->currentTransitionTime;
		} else {
			// else do nothing
		}
	}

	// Handle LCD brightness control
	// legacy translations of dimness settings; get rid of this once the control panel is updated
	if( gDimLevel == 1 ) {
	  gDimLevel = 720;
	} else if( gDimLevel == 0 ) {
	  gDimLevel = 0xFFFF;
	}
	// now the proper code
	if( (gDimLevel != 0) && (gDimLevel < 700) )
	  gDimLevel = 700;    // limit minimum duty cycle to about 1%
	if( gDimLevel > 0xFFFF )
	  gDimLevel = 0xFFFF;
	if( (PWMC & 0x20) == 0 ) {
	  CHUMSENSE1_DEBUG( Error, "FIFO is full, PWM write ignored.\n" );
	}
	PWMS = gDimLevel;
	PWMC = 0x00000010; // enable, no reset
	

	// handle speaker muting
	if (gSpkMute == 1) {
		imx_gpio_write(GPIO_PORTKP | (4 + 8), 1);	// speakers off
	} else {
		imx_gpio_write(GPIO_PORTKP | (4 + 8), 0);	// speakers on
	}

	// handle headphone plug-in status reporting
	if (imx_gpio_read(GPIO_PORTKP | 3 | GPIO_IN))
		gHPin = 0;
	else
		gHPin = 1;

	// handle watchdog timer update, very important!
	WSR = 0x5555; 
	WSR = 0xAAAA; // that should do it...

	// now handle retasking
	if (!gDone) {		// requires the done un-set to wait a few hundred ms to flush out the last timer added
		data->timer.expires += 1;	// next jiffie interval!
		add_timer(&data->timer);
	}
}

/*
  this driver handles the following sensors:

  * dimming level
  * speaker muting
  * DC voltage reading
  * battery voltage reading
  * headphone insertion status
  * bend sensor ('switch')

*/
static int __init chumby_sense1_init(void)
{
	dev_t dev = 0;
	int result, err;

	sense1task_data.sense1_cdev = cdev_alloc();

	// insert all device specific hardware initializations here
	printk("Chumby sensor suite 1 driver version %s initializing (bunnie@chumby.com)...\n", SENSE1_VERSION);

	/*
	 * Get a range of minor numbers to work with, asking for a dynamic
	 * major unless directed otherwise at load time.
	 */
	if (sense1_major) {
		dev = MKDEV(sense1_major, sense1_minor);
		result = register_chrdev_region(dev, sense1_nr_devs, "switch");
	} else {
		result = alloc_chrdev_region(&dev, sense1_minor, sense1_nr_devs, "switch");
		sense1_major = MAJOR(dev);
	}
	if (result < 0) {
		printk(KERN_WARNING "switch: can't get major %d\n",
		       sense1_major);
		return result;
	}

	create_proc_read_entry("sense1", 0, 0, sense1_read_proc, NULL);
	create_proc_read_entry("resetReason", 0, 0, resetReason_proc, NULL );

	cdev_init(sense1task_data.sense1_cdev, &sense1_fops);
	sense1task_data.sense1_cdev->owner = THIS_MODULE;
	sense1task_data.sense1_cdev->ops = &sense1_fops;
	err = cdev_add(sense1task_data.sense1_cdev, dev, 1);
	/* Fail gracefully if need be */
	if (err)
		printk(KERN_NOTICE "Error %d adding switch device\n", err);

	// enable KPP interface
	PCCR1 |= PCCR1_KPP_EN;
	KPP_KPSR |= KPP_KPSR_KPPEN;

	////////// audio shutdown init code, KP_COL4
	imx_gpio_mode(GPIO_PORTKP | (4 + 8) | GPIO_OUT);
	imx_gpio_write(GPIO_PORTKP | (4 + 8), 0);	// speakers on

	//              pin                            port bank and number   board pad
	// PWM0(primary)H19    TOUT2 (GPIO,BIN)        PE5                    J106
	// TOUT(primary)A14    TOUT  (primary)         PC14                   J108
	// xx xxxx
	imx_gpio_mode(GPIO_PORTE | 5 | GPIO_PF | GPIO_OUT);
	//	_reg_GPIO_GIUS(GPIOE) &= 0xFFFFFFDF; // set to non-GPIO (muxed) function
	//	_reg_GPIO_GPR(GPIOE)  &= 0xFFFFFFDF; // select primary function
	//	_reg_GPIO_DDIR(GPIOE) |= 0x20;       // set output
	// now we are set up to PWM!

	PCCR1 |= 0x10000000; // enable clock in PWM module
	/// PWM setup values
	/// Bit 18  HCTR   0
	/// Bit 17  BCTR   0
	/// Bit 16  SWR    1  << do a reset
	/// Bit 15  CLKSRC 0  PERCLK1 (16 MHz)
	/// Bit 14-8 PRE   0000000  don't prescale
	/// Bit 7   IRQ       readonly
	/// Bit 6   IRQEN  0  no interrupts
	/// Bit 5   FIFOAV    readonly
	/// Bit 4   EN     1  to enable, 0 to disable
	/// Bit 3-2 REPEAT 00  use each sample one time
	/// Bit 1-0 CLKSEL 00  divide by 2
	///                 v
	/// 0000 0000 0000 0001 1000 0000 0000 0000   // disable and reset, set clock
	/// 0x00018000
	/// 0000 0000 0000 0000 1000 0000 0001 0000   // enable, no reset
	/// 0x00008010
	/// 0000 0000 0000 0000 1000 0000 0000 0000   // disable, no reset
	/// 0x00008000
	///
	// the clock frequency into the block (perclk) is derived from the same clock that the UART clock comes from
	// which is 16 MHz (15.9MHz exactly).
	// divide by 2, which gives 8 MHz
	// so to achive a period of 122Hz, set PWMP to 65536
	PWMC = 0x00010000;  // disable and reset, set clock
	PWMP = 0xFFFC;      // set to 65534 ... so that 65535 written into the device forces continuous all-on

	////////// LCD power on code
	imx_gpio_mode(GPIO_PORTKP | (2 + 8) | GPIO_OUT);
	imx_gpio_write(GPIO_PORTKP | (2 + 8), 1);	// LCD on

	////////// LCD brightness control code, pin A19, PC28
	imx_gpio_mode(GPIO_PORTC | 28 | GPIO_OUT | GPIO_GPIO);
	imx_gpio_write(GPIO_PORTC | 28, 1);	// LCD bright

	///// headphone detect input pin, KP_ROW3
	imx_gpio_mode(GPIO_PORTKP | 3 | GPIO_IN);

	///// bend sensor input pin, KP_ROW0
	imx_gpio_mode(GPIO_PORTKP | 0 | GPIO_IN);

#if USE_SYSCTL
	gSysCtlHeader = register_sysctl_table(gSysCtl, 0);
	if (gSysCtlHeader != NULL)
		gSysCtlHeader->ctl_table->child->de->owner = THIS_MODULE;
#endif

	///// initialize periodic task queue
	sense1task_data.bent = 0;
	sense1task_data.lastTransitionTime = jiffies;
	sense1task_data.isDebounce = 0;
	sense1task_data.currentTransitionTime = sense1task_data.lastTransitionTime;
	sense1task_data.lastTransitionState = 1;	// default state of the sensor

	init_timer(&sense1task_data.timer);
	sense1task_data.timer.data = (unsigned long)&sense1task_data;
	sense1task_data.timer.function = sense1_task_record;
	sense1task_data.timer.expires = jiffies + 1;
	add_timer(&sense1task_data.timer);

	// initialize the watchdog timer module
	PCCR1 |= PCCR1_WDT_EN;
	WCR = 0x0430;  // set watchdog to 0x02 cycles (2 seconds) before trigger, plus force a system reset
	WCR = 0x0434;  // enable the watchdog timer! better have the update process running.

	resetReason = WRSR;

	return 0;
}

static ssize_t chumby_sense1_read(struct file *file, char *buf,
				  size_t count, loff_t * ppos)
{
	char retval = 0;
	size_t retlen = 1;

	retval = sense1task_data.bent;

	CHUMSENSE1_DEBUG(Trace, "sense1 read exit with: %d\n", retval);
	copy_to_user(buf, &retval, retlen);

	return retlen;
}

static void __exit chumby_sense1_exit(void)
{
	dev_t devno = MKDEV(sense1_major, sense1_minor);

	CHUMSENSE1_DEBUG(Trace, "Top of exit.\n");
	// set global flag that we're out of here and force a call to remove ourselves
	gDone = 1;

	// delete our kernel task
	del_timer(&(sense1task_data.timer));

	// wait to dequeue self; if kernel panics on rmmod try adding 100 to this value
	mdelay(200);

	// insert all cleanup stuff here
	cdev_del(sense1task_data.sense1_cdev);
	remove_proc_entry("sense1", NULL);
	unregister_chrdev_region(devno, sense1_nr_devs);

#if USE_SYSCTL
	if (gSysCtlHeader != NULL)
		unregister_sysctl_table(gSysCtlHeader);
#endif

}

// entry and exit mappings
module_init(chumby_sense1_init);
module_exit(chumby_sense1_exit);
