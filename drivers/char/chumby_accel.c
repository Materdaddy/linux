/*
       chumby_accel.c
       bunnie -- March 2007 -- 2.0 -- port to Ironforge
       bunnie -- April 2007 -- 2.1 -- fixed grange to 2500

       This file is part of the chumby accelerometer driver in the linux kernel.
       Copyright (c) Chumby Industries, 2007

       The accelerometer driver is free software; you can redistribute it and/or modify
       it under the terms of the GNU General Public License as published by
       the Free Software Foundation; either version 2 of the License, or
       (at your option) any later version.

       The accelerometer driver is distributed in the hope that it will be useful,
       but WITHOUT ANY WARRANTY; without even the implied warranty of
       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
       GNU General Public License for more details.

       You should have received a copy of the GNU General Public License
       along with the Chumby; if not, write to the Free Software
       Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/

#define ACCEL_VERSION "2.1-Kionix-Ironforge"
#define DCID_VERSION  "1.0-Atmel-25080A-Ironforge"

#include <linux/config.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>

#include <linux/kernel.h>       /* printk() */
#include <linux/slab.h>         /* kmalloc() */
#include <linux/fs.h>           /* everything... */
#include <linux/errno.h>        /* error codes */
#include <linux/types.h>        /* size_t */
#include <linux/proc_fs.h>
#include <linux/fcntl.h>        /* O_ACCMODE */
#include <linux/seq_file.h>
#include <linux/cdev.h>

#include <asm/io.h>
#include <asm/system.h>         /* cli(), *_flags */
#include <asm/uaccess.h>        /* copy_*_user */

#include <linux/miscdevice.h>
#include <linux/ioport.h>
#include <linux/poll.h>
#include <linux/sysctl.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/rtc.h>

#include <linux/timer.h>

#include <asm/arch/imx-regs.h>

#include "chumby_accel.h"

/*
 * basic parameters
 */

int accel_major = 0; // dynamic allocation
int accel_minor = 0;
int accel_nr_devs = 1;

int dcid_major = 0; // dynamic allocation
int dcid_minor = 0;
int dcid_nr_devs = 1;

module_param(accel_major, int, S_IRUGO);
module_param(accel_minor, int, S_IRUGO);
module_param(accel_nr_devs, int, S_IRUGO);

module_param(dcid_major, int, S_IRUGO);
module_param(dcid_minor, int, S_IRUGO);
module_param(dcid_nr_devs, int, S_IRUGO);

MODULE_AUTHOR("bunnie@chumby.com");
MODULE_LICENSE("GPL");

#define ACCEL_DEFAULT_THRESH 0xC0  // absolute value, +/- off of avg to trigger accel

// static data
static unsigned long accel_status = 0;	/* bitmapped status byte.	*/
static unsigned long dcid_status = 0;	/* bitmapped status byte.	*/
static int gDone = 0;
static unsigned int gOuchval = 375;
static unsigned int gBreakval = 825;
static unsigned int gAccelThresh = ACCEL_DEFAULT_THRESH;

static unsigned int impactHint = 0;
static unsigned int impactTime = 0;
static unsigned int impactVector[3] = {0,0,0};

static unsigned char gDCvers[VERS_LEN] =   "unsupported\n";
static unsigned char gDCserial[VERS_LEN] = "unsupported\n";
//static unsigned char gMBvers[VERS_LEN] =   "unsupported\n";  // get these from CP directly...
//static unsigned char gMBserial[VERS_LEN] = "unsupported\n";

static int recDelay = 0;

static int gRomBusy = 0;

#define IMPCT_HINT_OUCH  1
#define IMPCT_HINT_BREAK 2

/*
 * Accel sensor data logs, tracked by tasks that are scheduled by the
 * task scheduler
 */

#define ACCEL_INTERVAL 1  // jiffies elapsed between recordings

#define FIXEDPOINT_NORM 1000

#define MAX_ACCEL_DATA 16

#define NOT_MOVED 0
#define MOVED 1

#define CLK_DIVIDER  ((PCDR1 & PCDR1_PERDIV2_MASK) >> PCDR1_PERDIV2_POS)
#define BASE_FREQ    350
#define TARGET_FREQ  5

#define EASY_DEBUGGING 0   // causes read calls to spit out ASCII text instead of binary records

struct acceldata {
  int index; // always points at the next FREE location
  unsigned short *dataLogX; // initialize to buffer of proper length
  unsigned short *dataLogY; // initialize to buffer of proper length
  unsigned short *dataLogZ; // initialize to buffer of proper length
  unsigned short runningAvgX;
  unsigned short runningAvgY;
  unsigned short runningAvgZ;
  unsigned int logTotalX; // shortcut for maintaining runningAvg
  unsigned int logTotalY; // shortcut for maintaining runningAvg
  unsigned int logTotalZ; // shortcut for maintaining runningAvg
  unsigned short threshold;
  unsigned char moved;
  struct timer_list timer;
  struct cdev *accel_cdev;
} acceltask_data;

struct dciddata {
  struct cdev *dcid_cdev;
} dcidtask_data;

#define USE_SYSCTL  1

#define SPI_CHAN  0

#define ACCEL_BLOCKING 0  // turn off blocking read on accel sensor

#if USE_SYSCTL
#      define CHUMACCEL_DEBUG( flag, fmt, args... ) do { if ( gDebug ## flag ) printk( "%s: " fmt, __FUNCTION__ , ## args ); } while (0)
#else
#      define CHUMACCEL_DEBUG( flag, fmt, args... ) printk( "%s: " fmt, __FUNCTION__ , ## args )
#endif

#if USE_SYSCTL
#      define CHUMDCID_DEBUG( flag, fmt, args... ) do { if ( gDebug ## flag ) printk( "%s: " fmt, __FUNCTION__ , ## args ); } while (0)
#else
#      define CHUMDCID_DEBUG( flag, fmt, args... ) printk( "%s: " fmt, __FUNCTION__ , ## args )
#endif

// function protos
static int chumby_dcid_ioctl(struct inode *inode, struct file *file,
	  		     unsigned int cmd, unsigned long arg);
static int chumby_accel_open(struct inode *inode, struct file *file);
static int chumby_dcid_open(struct inode *inode, struct file *file);
static int chumby_accel_release(struct inode *inode, struct file *file);
static int chumby_dcid_release(struct inode *inode, struct file *file);
static int accel_read_proc(char *page, char **start, off_t off,
                           int count, int *eof, void *data);
static int dcid_read_proc(char *page, char **start, off_t off,
                           int count, int *eof, void *data);
static ssize_t chumby_accel_read(struct file *file, char *buf,
				 size_t count, loff_t *ppos);
//static ssize_t chumby_accel_write(struct file *file, const unsigned char *buf, 
//				 size_t count, loff_t *ppos );
//static unsigned int chumby_accel_poll(struct file * filp, struct poll_table_struct * wait);
static unsigned char readEEPROM(unsigned int addr);

#if USE_SYSCTL

// proc debug structure
static  int gDebugTrace = 0;
static  int gDebugIoctl = 0;
static  int gDebugError = 1;
static  int gEmpty = -1;

// create /proc/sys/debug-trace, etc...which can be written to set behavior of globals
// in this driver on the fly
static struct ctl_table_header    *gSysCtlHeader; 
static struct ctl_table_header    *gSysCtlHeader_dcid; 
static struct ctl_table gSysCtlChumaccel[] =
{
    { CTL_CHUMACCEL_DEBUG_TRACE,     "dbg-trace",  &gDebugTrace,   sizeof( int ), 0644, NULL, &proc_dointvec },
    { CTL_CHUMACCEL_DEBUG_IOCTL,     "dbg-ioctl",  &gDebugIoctl,   sizeof( int ), 0644, NULL, &proc_dointvec },
    { CTL_CHUMACCEL_DEBUG_ERROR,     "dbg-error",  &gDebugError,   sizeof( int ), 0644, NULL, &proc_dointvec },
    { CTL_CHUMACCEL_OUCHVAL,         "touchThresh", &gOuchval, sizeof( unsigned int ), 0644, NULL, &proc_dointvec },
    { CTL_CHUMACCEL_BREAKVAL,        "painThresh", &gBreakval, sizeof( unsigned int ), 0644, NULL, &proc_dointvec },
    { CTL_CHUMACCEL_ROMACT,          "romBusy", &gRomBusy, sizeof( unsigned int ), 0444, NULL, &proc_dointvec },
    { 0 }
};
// these are all read-only
// for now these are also all deprecated!
static struct ctl_table gSysCtlChumdcid[] =
{
  // placeholder 
      { CTL_CHUMDCID_EMPTY,             "empty", &gEmpty, sizeof( unsigned int ), 0444, NULL, &proc_dointvec },
  //    { CTL_CHUMDCID_VERS,             "dc_version", gDCvers, sizeof( unsigned char ) * VERS_LEN, 0444, NULL, &proc_dointvec },
  //    { CTL_CHUMDCID_SERIAL,           "dc_serial", gDCserial, sizeof( unsigned char ) * VERS_LEN, 0444, NULL, &proc_dointvec },
  //    { CTL_CHUMDCID_CORE_VERS,        "mb_version", gMBvers, sizeof( unsigned char ) * VERS_LEN, 0444, NULL, &proc_dointvec },
  //    { CTL_CHUMDCID_CORE_SERIAL,      "mb_serial", gMBserial, sizeof( unsigned char ) * VERS_LEN, 0444, NULL, &proc_dointvec },
    { 0 }
};

static struct ctl_table gSysCtl[] =
{
    { CTL_CHUMACCEL, "accel", NULL, 0, 0555, gSysCtlChumaccel },
    { 0 }
};

static struct ctl_table gSysCtl_dcid[] =
{
    { CTL_CHUMACCEL, "versions", NULL, 0, 0555, gSysCtlChumdcid },
    { 0 }
};
#endif  // USE_SYSCTL

// map into the generic driver infrastructure
static struct file_operations accel_fops = {
	owner:		THIS_MODULE,
	//llseek:	   chumby_accel_llseek,
	read:		chumby_accel_read,
	// poll:	   chumby_accel_poll,
	// ioctl:		chumby_accel_ioctl,
	open:		chumby_accel_open,
	release:	chumby_accel_release,
	// write:          chumby_accel_write,
	// fasync:         chubmy_accel_fasync,
};

// map into the generic driver infrastructure
static struct file_operations dcid_fops = {
	owner:		THIS_MODULE,
	ioctl:		chumby_dcid_ioctl,
	open:		chumby_dcid_open,
	release:	chumby_dcid_release,
};

///////////// code /////////////

static int spi_tx_fifo_empty(void)
{ return (SSP_INT_REG(SPI_CHAN) & SSP_INT_TE);}

static int spi_rx_fifo_data_ready(void)
{ return (SSP_INT_REG(SPI_CHAN) & SSP_INT_RR);}

static unsigned int spi_exchange_data(unsigned int dataTx) {
  while(!spi_tx_fifo_empty());

  SSP_TX_REG(SPI_CHAN)   = dataTx;	     // transfer data
  SSP_CTRL_REG(SPI_CHAN) |= SSP_XCH;         // exchange data
    
  while(!spi_rx_fifo_data_ready());

  return SSP_RX_REG(SPI_CHAN);
}

static unsigned int getX(void) {
  unsigned int sampled;
  ACCEL_SEL_CS3;
  ACCEL_SEL_CS2;
  SSP_CTRL_REG(SPI_CHAN) &= 0xFFFFFFE0;
  SSP_CTRL_REG(SPI_CHAN) |= SSP_WS(8);
  sampled = spi_exchange_data( (unsigned int) ADC_CHSEL(ADC_XCH) );
  udelay(50);
  SSP_CTRL_REG(SPI_CHAN) &= 0xFFFFFFE0;
  SSP_CTRL_REG(SPI_CHAN) |= SSP_WS(16);
  sampled = spi_exchange_data( (unsigned int) ADC_CHSEL(ADC_XCH) );
  ACCEL_SEL_CS3;
  
  sampled >>= 4;
  sampled &= 0xFFF;
  return sampled;
}
static unsigned int getY(void) {
  unsigned int sampled;
  ACCEL_SEL_CS3;
  ACCEL_SEL_CS2;
  SSP_CTRL_REG(SPI_CHAN) &= 0xFFFFFFE0;
  SSP_CTRL_REG(SPI_CHAN) |= SSP_WS(8);
  sampled = spi_exchange_data( (unsigned int) ADC_CHSEL(ADC_YCH) );
  udelay(50);
  SSP_CTRL_REG(SPI_CHAN) &= 0xFFFFFFE0;
  SSP_CTRL_REG(SPI_CHAN) |= SSP_WS(16);
  sampled = spi_exchange_data( (unsigned int) ADC_CHSEL(ADC_YCH) );
  ACCEL_SEL_CS3;

  sampled >>= 4;
  sampled &= 0xFFF;
  return sampled;
}
static unsigned int getZ(void) {
  unsigned int sampled;
  ACCEL_SEL_CS3;
  ACCEL_SEL_CS2;
  SSP_CTRL_REG(SPI_CHAN) &= 0xFFFFFFE0;
  SSP_CTRL_REG(SPI_CHAN) |= SSP_WS(8);
  sampled = spi_exchange_data( (unsigned int) ADC_CHSEL(ADC_ZCH) );
  udelay(50);
  SSP_CTRL_REG(SPI_CHAN) &= 0xFFFFFFE0;
  SSP_CTRL_REG(SPI_CHAN) |= SSP_WS(16);
  sampled = spi_exchange_data( (unsigned int) ADC_CHSEL(ADC_ZCH) );
  ACCEL_SEL_CS3;

  sampled >>= 4;
  sampled &= 0xFFF;
  return sampled;
}

static int chumby_accel_release(struct inode *inode, struct file *file) {
  CHUMACCEL_DEBUG( Trace, "Top of release.\n" );
  accel_status &= ~ACCEL_IS_OPEN;
  return 0;
}

static int chumby_dcid_release(struct inode *inode, struct file *file) {
  CHUMDCID_DEBUG( Trace, "Top of release.\n" );
  dcid_status &= ~DCID_IS_OPEN;
  return 0;
}

static int accel_proc_output (char *buf) {
  int printlen = 0;
  
  // insert proc debugging output here
  printlen = sprintf(buf, "Chumby accelerometer driver version %s (bunnie@chumby.com)\n", ACCEL_VERSION );
  if( gDebugTrace )
    printlen += sprintf(buf, "  Debug trace is enabled.\n" );
  else
    printlen += sprintf(buf, "  Debug trace is disabled.\n" );
  
  return(printlen);
}

static int dcid_proc_output (char *buf) {
  int printlen = 0;
  
  printlen = sprintf(buf, "DCversion: %s\nDCserial: %s\n", gDCvers, gDCserial );
  
  return(printlen);
}

static int accel_read_proc(char *page, char **start, off_t off,
                         int count, int *eof, void *data)
{
        int len = accel_proc_output (page);
        if (len <= off+count) *eof = 1;
        *start = page + off;
        len -= off;
        if (len>count) len = count;
        if (len<0) len = 0;
        return len;
}

static int dcid_read_proc(char *page, char **start, off_t off,
                         int count, int *eof, void *data)
{
        int len = dcid_proc_output (page);
        if (len <= off+count) *eof = 1;
        *start = page + off;
        len -= off;
        if (len>count) len = count;
        if (len<0) len = 0;
        return len;
}

static int chumby_accel_open(struct inode *inode, struct file *file) {
  // make sure we're not opened twice
  if( accel_status & ACCEL_IS_OPEN)
    return -EBUSY;
  
  accel_status |= ACCEL_IS_OPEN;
  return(0);
}

static int chumby_dcid_open(struct inode *inode, struct file *file) {
  // make sure we're not opened twice
  if( dcid_status & DCID_IS_OPEN)
    return -EBUSY;
  
  dcid_status |= DCID_IS_OPEN;
  return(0);
}

void accel_task_record(unsigned long ptr) {
  struct acceldata *data = (struct acceldata *)ptr;
  unsigned int sampledX, sampledY, sampledZ;
  int differenceX, differenceY, differenceZ;
  
  if( gRomBusy == 0 ) { // make sure that ROM isn't using the SPI bus
    sampledX = getX();
    sampledY = getY();
    sampledZ = getZ();

    ////// record data here
    // remove stale sample from running total bucket
    data->logTotalX -= data->dataLogX[data->index];
    data->logTotalY -= data->dataLogY[data->index];
    data->logTotalZ -= data->dataLogZ[data->index];

    // commit the new data
    data->dataLogX[data->index] = sampledX;
    data->dataLogY[data->index] = sampledY;
    data->dataLogZ[data->index] = sampledZ;
    data->logTotalX += data->dataLogX[data->index]; // add new sample to running total bucket
    data->logTotalY += data->dataLogY[data->index];
    data->logTotalZ += data->dataLogZ[data->index];
    // update indices
    data->index++;
    data->index %= MAX_ACCEL_DATA;  // reset modulus

    // process a little bit of data
    data->runningAvgX = data->logTotalX / MAX_ACCEL_DATA;
    data->runningAvgY = data->logTotalY / MAX_ACCEL_DATA;
    data->runningAvgZ = data->logTotalZ / MAX_ACCEL_DATA;

    differenceX = sampledX - data->runningAvgX;
    differenceY = sampledY - data->runningAvgY;
    differenceZ = sampledZ - data->runningAvgZ;
    differenceX = (differenceX < 0) ? (-differenceX) : differenceX;
    differenceY = (differenceY < 0) ? (-differenceY) : differenceY;
    differenceZ = (differenceZ < 0) ? (-differenceZ) : differenceZ;
    if( (differenceX > gAccelThresh) || (differenceY > gAccelThresh) || (differenceZ > gAccelThresh) ) {
      CHUMACCEL_DEBUG( Trace, "Got accel event.\n" );
    }
    if( (differenceX > gOuchval) || (differenceY > gOuchval) || (differenceZ > gOuchval) ) {
      if( (differenceX > gBreakval) || (differenceY > gBreakval) || (differenceZ > gBreakval) ) {
	impactHint = IMPCT_HINT_BREAK;
      } else {
	impactHint = IMPCT_HINT_OUCH;
      }
      impactTime = jiffies;
      impactVector[0] = sampledX;
      impactVector[1] = sampledY;
      impactVector[2] = sampledZ;
    }
  }

  recDelay++;  // legacy code to keep track of recording delays, keep around in case we want to use it again

  // now handle retasking
  if( !gDone ) {  // requires the done un-set to wait a few hundred ms to flush out the last timer added
    data->timer.expires += ACCEL_INTERVAL; // next jiffie interval!
    add_timer( &data->timer );
  }
}

/*
  this driver handles the following sensors:

  * accelerometer (kionix)

*/

static int __init chumby_accel_init(void) {
  dev_t dev = 0;
  unsigned int sampledX, sampledY, sampledZ;
  int result, err;
  char temp[DCID_SERIAL_LEN + 1];
  int i;

  acceltask_data.accel_cdev = cdev_alloc();

  // insert all device specific hardware initializations here
  printk( "Chumby accelerometer driver version %s initializing (bunnie@chumby.com)...\n", ACCEL_VERSION );

  /*
   * Get a range of minor numbers to work with, asking for a dynamic
   * major unless directed otherwise at load time.
   */
  if (accel_major) {
    dev = MKDEV(accel_major, accel_minor);
    result = register_chrdev_region(dev, accel_nr_devs, "accel");
  } else {
    result = alloc_chrdev_region(&dev, accel_minor, accel_nr_devs,
				 "accel");
    accel_major = MAJOR(dev);
  }
  if (result < 0) {
    printk(KERN_WARNING "accel: can't get major %d\n", accel_major);
    return result;
  }

  create_proc_read_entry ("accel", 0, 0, accel_read_proc, NULL);

  cdev_init(acceltask_data.accel_cdev, &accel_fops);
  acceltask_data.accel_cdev->owner = THIS_MODULE;
  acceltask_data.accel_cdev->ops = &accel_fops;
  err = cdev_add (acceltask_data.accel_cdev, dev, 1);
  /* Fail gracefully if need be */
  if (err)
    printk(KERN_NOTICE "Error %d adding accel device\n", err);

  // hardware init
  // map GPIO ports appropriately
  // CSPI1_SS1    F16    CSPI1_SS1 (GPIO   )     PD27  out 1  ** not controlled by CSPI 
  // CSPI1_SS0    F19    CSPI1_SS0 (GPIO   )     PD28  out 1  ** not controlled by CSPI
  // CSPI1_SCLK   H10    CSPI1_SCLK(primary)     PD29  out 1
  // CSPI1_MISO   F17    CSPI1_MISO(primary)     PD30  in  0
  // CSPI1_MOSI   J12    CSPI1_MOSI(primary)     PD31  out 1
  // ** note that SS0,1 is decoded on the sensor card to 4 individual chipselects
  //    so these values have to be manually controlled by the software
  
  imx_gpio_mode( GPIO_PORTD | 27 | GPIO_OUT | GPIO_GPIO );
  imx_gpio_mode( GPIO_PORTD | 28 | GPIO_OUT | GPIO_GPIO );
  ACCEL_SEL_CS3;

  imx_gpio_mode( GPIO_PORTD | 29 | GPIO_OUT | GPIO_PF );
  imx_gpio_mode( GPIO_PORTD | 30 | GPIO_IN  | GPIO_PF );
  imx_gpio_mode( GPIO_PORTD | 31 | GPIO_OUT | GPIO_PF );

  // turn on the clock
  PCCR0 |= PCCR0_CSPI1_EN;

  // setup the SPI interfaces
  // reset the interface
  SSP_RESET_REG(SPI_CHAN) = 0x00000001;
  udelay(200);      //wait

  // use 32.768kHz clock speed, with 0 clock insertion
  SSP_PER_REG(SPI_CHAN) = 0x00008000;

  // init the hardware config register
  // 31-24 00000000
  //    23 0     BURST off
  //    22 0     SDHC off
  //    21 0     SWAP off
  // 20-19 00    SS asserted
  // 18-14 00000 ((BASE_FREQ/CLK_DIVIDER) / (TARGET_FREQ) >> 1)
  // 13-12 00    DRCTL ignore ready
  //    11 1     MODE = master
  //    10 1*    SPIEN enable interface (toggle to reset Tx FIFO)
  //     9 0     XCH  exchange off for now
  //     8 0     SSPOL is active low
  //     7 1     SSCTL CS toggles between bursts
  //     6 1     PHA phase 1 operation (falling edge)
  //     5 1     POL (active 1 idle)
  //   4-0 01111 BITCOUNT 16-bit transfer for Kionix
  SSP_CTRL_REG(SPI_CHAN) = 0;
  SSP_CTRL_REG(SPI_CHAN) |= (SSP_MODE_MASTER | SSP_ENABLE | SSP_SS_PULSE | SSP_PHA1 | SSP_POL1 | SSP_WS(16) );
  SSP_CTRL_REG(SPI_CHAN) |= (((BASE_FREQ/CLK_DIVIDER) / (TARGET_FREQ) >> 1) + 1) << 14;
  SSP_CTRL_REG(SPI_CHAN) &= ~SSP_MODE_MASTER;  // reset fifo
  // printk( "accel clock init: base freq %d, clk_divider %d, target_freq %d, calc div %d\n", BASE_FREQ, CLK_DIVIDER, TARGET_FREQ, ((BASE_FREQ/CLK_DIVIDER) / (TARGET_FREQ) >> 1) + 1 );

  udelay(100);      //wait
  SSP_CTRL_REG(SPI_CHAN) &= 0xFFFFFFE0;
  SSP_CTRL_REG(SPI_CHAN) |= SSP_WS(16);
  SSP_CTRL_REG(SPI_CHAN) |= SSP_MODE_MASTER;  // reset fifo

  // write the accelerometer configuration word
  ACCEL_SEL_CS3;
  ACCEL_SEL_CS2;
  spi_exchange_data( (unsigned int) 0x0404 ); // set power mode to ON
  ACCEL_SEL_CS3;

  ///// initialize periodic task queue
  CHUMACCEL_DEBUG( Trace, "got to point 1\n" );
  acceltask_data.index = 0;
  acceltask_data.dataLogX = kmalloc( sizeof (unsigned short) * MAX_ACCEL_DATA, GFP_KERNEL );
  acceltask_data.dataLogY = kmalloc( sizeof (unsigned short) * MAX_ACCEL_DATA, GFP_KERNEL );
  acceltask_data.dataLogZ = kmalloc( sizeof (unsigned short) * MAX_ACCEL_DATA, GFP_KERNEL );
  CHUMACCEL_DEBUG( Trace, "got to point 2\n" );
  if( (acceltask_data.dataLogX == NULL) || (acceltask_data.dataLogY == NULL) || (acceltask_data.dataLogZ == NULL) ) {
    CHUMACCEL_DEBUG( Error, "Could not allocate acceltask_data structure, I see a panic soon...\n" );
    // todo: write code to handle this condition gracefully. for now, we just report and crash.
    // PS: it's very unlikely this will happen--we'd have to run out of memory at driver init time
    // which means we have bigger problems in reality.
  }
  CHUMACCEL_DEBUG( Trace, "got to point 3\n" );
  sampledX = getX();
  sampledY = getY();
  sampledZ = getZ();
  CHUMACCEL_DEBUG( Trace, "got to point 4\n" );
  acceltask_data.logTotalX = 0;
  acceltask_data.logTotalY = 0;
  acceltask_data.logTotalZ = 0;
  for( i = 0; i < MAX_ACCEL_DATA; i++ ) {
    acceltask_data.dataLogX[i] = sampledX;
    acceltask_data.dataLogY[i] = sampledY;
    acceltask_data.dataLogZ[i] = sampledZ;
    acceltask_data.logTotalX += acceltask_data.dataLogX[i];
    acceltask_data.logTotalY += acceltask_data.dataLogY[i];
    acceltask_data.logTotalZ += acceltask_data.dataLogZ[i];
  }
  CHUMACCEL_DEBUG( Trace, "got to point 5\n" );
  acceltask_data.runningAvgX = acceltask_data.logTotalX / MAX_ACCEL_DATA;
  acceltask_data.runningAvgY = acceltask_data.logTotalY / MAX_ACCEL_DATA;
  acceltask_data.runningAvgZ = acceltask_data.logTotalZ / MAX_ACCEL_DATA;
  acceltask_data.threshold = ACCEL_DEFAULT_THRESH;
  acceltask_data.moved = NOT_MOVED;
  CHUMACCEL_DEBUG( Trace, "got to point 6\n" );

  ///// initialize periodic task queue
  init_timer( &acceltask_data.timer );
  acceltask_data.timer.data = (unsigned long) &acceltask_data;
  acceltask_data.timer.function = accel_task_record;
  acceltask_data.timer.expires = jiffies + ACCEL_INTERVAL;
  add_timer( &(acceltask_data.timer) );

  // populate proc entries
#if USE_SYSCTL
  gSysCtlHeader = register_sysctl_table( gSysCtl, 0 );
  if ( gSysCtlHeader != NULL ) {
    gSysCtlHeader->ctl_table->child->de->owner = THIS_MODULE;
  }
#endif

  // insert all device specific hardware initializations here
  printk( "Chumby DC ID EEPROM driver version %s initializing (bunnie@chumby.com)...\n", DCID_VERSION );
  dcidtask_data.dcid_cdev = cdev_alloc();

  /*
   * Get a range of minor numbers to work with, asking for a dynamic
   * major unless directed otherwise at load time.
   */
  if (dcid_major) {
    dev = MKDEV(dcid_major, dcid_minor);
    result = register_chrdev_region(dev, dcid_nr_devs, "dcid");
  } else {
    result = alloc_chrdev_region(&dev, dcid_minor, dcid_nr_devs,
				 "dcid");
    dcid_major = MAJOR(dev);
  }
  if (result < 0) {
    printk(KERN_WARNING "dcid: can't get major %d\n", dcid_major);
    return result;
  }

  create_proc_read_entry ("DCversion", 0, 0, dcid_read_proc, NULL);

  cdev_init(dcidtask_data.dcid_cdev, &dcid_fops);
  dcidtask_data.dcid_cdev->owner = THIS_MODULE;
  dcidtask_data.dcid_cdev->ops = &dcid_fops;
  err = cdev_add (dcidtask_data.dcid_cdev, dev, 1);
  /* Fail gracefully if need be */
  if (err)
    printk(KERN_NOTICE "Error %d adding dcid device\n", err);


  // now populate the DCID read-only fields
  for( i = 0; i < DCID_REV_LEN; i++ ) {
    temp[i] = readEEPROM( DCID_REV_LOC + i );
  }
  sprintf( gDCvers, "%02X.%02X.%02X.%02X", temp[0], temp[1], temp[2], temp[3] );
  for( i = 0; i < DCID_SERIAL_LEN; i++ ) {
    temp[i] = readEEPROM( DCID_SERIAL_LOC + i );
  }
  temp[i] = '\0';
  sprintf( gDCserial, "%s", temp );
    
  // populate some proc entries
#if USE_SYSCTL
  gSysCtlHeader_dcid = register_sysctl_table( gSysCtl_dcid, 0 );
  if ( gSysCtlHeader_dcid != NULL ) {
    gSysCtlHeader_dcid->ctl_table->child->de->owner = THIS_MODULE;
  }
#endif

  gRomBusy = 0; // make sure the task can get its data...

  return (0);
}

static ssize_t chumby_accel_read(struct file *file, char *buf,
			size_t count, loff_t *ppos) {
  size_t retlen = 0;
  unsigned int sampledX, sampledY, sampledZ;
  struct accelReadData rd;
  
  CHUMACCEL_DEBUG( Trace, "Top of read.\n" );

  if(acceltask_data.index > 0) {
    sampledX = acceltask_data.dataLogX[acceltask_data.index - 1];
    sampledY = acceltask_data.dataLogY[acceltask_data.index - 1];
    sampledZ = acceltask_data.dataLogZ[acceltask_data.index - 1];
  } else {
    sampledX = acceltask_data.dataLogX[MAX_ACCEL_DATA - 1];
    sampledY = acceltask_data.dataLogY[MAX_ACCEL_DATA - 1];
    sampledZ = acceltask_data.dataLogZ[MAX_ACCEL_DATA - 1];
  }
  CHUMACCEL_DEBUG( Trace, "Got values %04X:%04X:%04X.\n", sampledX, sampledY, sampledZ );
#if EASY_DEBUGGING  // easy debugging
  char retval[200];
  retlen += sprintf( retval, "%04X:%04X:%04X|%04X:%04X:%04X\n",
		     acceltask_data.runningAvgX, acceltask_data.runningAvgY,
		     acceltask_data.runningAvgZ, sampledX, sampledY, sampledZ );

  retlen++; //+1 for null character

  CHUMACCEL_DEBUG( Trace, "Returning %d bytes\n", retlen );
  int remaining = copy_to_user(buf,retval,retlen);
  if(remaining != 0) {
    CHUMACCEL_DEBUG( Trace, "Copy failed with %d bytes remaining\n", remaining );
    return -EFAULT;
  }
  CHUMACCEL_DEBUG( Trace, "Successfully copied\n" );
#else
  rd.version = ACCEL_VERSION_NUM;
  rd.timestamp = jiffies;
  rd.inst[0] = sampledX; 
  rd.inst[1] = sampledY; 
  rd.inst[2] = sampledZ; 
  rd.avg[0] = acceltask_data.runningAvgX;
  rd.avg[1] = acceltask_data.runningAvgY;
  rd.avg[2] = acceltask_data.runningAvgZ;
  rd.impact[0] = impactVector[0];
  rd.impact[1] = impactVector[1];
  rd.impact[2] = impactVector[2];
  rd.impactTime = impactTime;
  rd.impactHint = impactHint;
  rd.gRange = GRANGE;
  retlen += sizeof(struct accelReadData);
  if(copy_to_user(buf, &rd, retlen))
    return -EFAULT;
#endif
  acceltask_data.moved = NOT_MOVED;

  return(retlen);
}

// ASSUME gRomBusy set before calling this
void issueWRDI(void) {
  ACCEL_SEL_CS3;
  ndelay(50);
  ACCEL_SEL_CS1;  // this is the SPI rom chip select code
  SSP_CTRL_REG(SPI_CHAN) &= 0xFFFFFFE0;
  SSP_CTRL_REG(SPI_CHAN) |= SSP_WS(8);
  spi_exchange_data( SPIROM_WRDI_CMD ); // disable write latch to prevent spurious writes on power state changes
  ACCEL_SEL_CS3;
}

// ASSUME gRomBusy set before calling this
void issueWREN(void) {
  ACCEL_SEL_CS3;
  ndelay(50); // min pulse time for CS
  ACCEL_SEL_CS1;  // this is the SPI rom chip select code
  SSP_CTRL_REG(SPI_CHAN) &= 0xFFFFFFE0;
  SSP_CTRL_REG(SPI_CHAN) |= SSP_WS(8);
  spi_exchange_data( SPIROM_WREN_CMD ); // issue WREN command
  ACCEL_SEL_CS3;
}

// ASSUME gRomBusy set before calling this
void waitEEPROMbusy(void) {
  unsigned int retdat = 0;
  
  do { // this is ugly...should I yield somewhere? I'm also totally stomping the accelerometer.
    ACCEL_SEL_CS3;
    ndelay(50);
    ACCEL_SEL_CS1;  // this is the SPI rom chip select code
    SSP_CTRL_REG(SPI_CHAN) &= 0xFFFFFFE0;
    SSP_CTRL_REG(SPI_CHAN) |= SSP_WS(16);
    retdat = spi_exchange_data( SPIROM_RDSR_CMD << 8 ); // check status register
    ACCEL_SEL_CS3;
  } while( retdat & 0x1 );

}

static unsigned char readEEPROM(unsigned int addr) {
  unsigned int retdat;

  gRomBusy = 1; // this shuts off the kernel task timer from using the SPI bus
  do {
    ACCEL_SEL_CS3;
    ndelay(50);
    ACCEL_SEL_CS1;  // this is the SPI rom chip select code
    SSP_CTRL_REG(SPI_CHAN) &= 0xFFFFFFE0;
    SSP_CTRL_REG(SPI_CHAN) |= SSP_WS(16);
    retdat = spi_exchange_data( SPIROM_RDSR_CMD << 8 ); // check status register
    ACCEL_SEL_CS3;
  } while( retdat & 0x1 );

  ACCEL_SEL_CS3;
  ndelay(50);
  ACCEL_SEL_CS1;
  SSP_CTRL_REG(SPI_CHAN) &= 0xFFFFFFE0;
  SSP_CTRL_REG(SPI_CHAN) |= SSP_WS(32);
  retdat = spi_exchange_data( ((SPIROM_READ_CMD & 0xFF) << 24) | 
			      ((addr & 0xFFFF) << 8) );
  ACCEL_SEL_CS3;
  gRomBusy = 0; // this shuts off the kernel task timer from using the SPI bus

  return( (unsigned char) retdat & 0xFF );
}


static int chumby_dcid_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
		     unsigned long arg) {
  struct eeprom_data dciddat;
  unsigned int retdat;
  int err;

  // safety code
  CHUMACCEL_DEBUG( Trace, "type: '%c' cmd: 0x%x\n", _IOC_TYPE( cmd ), _IOC_NR( cmd ));

  if (( _IOC_TYPE( cmd ) != ACCEL_IOCTL_MAGIC )
      ||  ( _IOC_NR( cmd ) < ACCEL_CMD_FIRST )
      ||  ( _IOC_NR( cmd ) >= ACCEL_CMD_LAST ))  {
    // Since we emulate some of the parallel port commands, we need to allow
    // those as well.
    return -ENOTTY;
  }

  // Note that _IOC_DIR Read/Write is from the perspective of userland. access_ok
  // is from the perspective of kernelland.

  err = 0;
  if (( _IOC_DIR( cmd ) & _IOC_READ ) != 0 )  {
    err |= !access_ok( VERIFY_WRITE, (void *)arg, _IOC_SIZE( cmd ));
  }
  if (( _IOC_DIR( cmd ) & _IOC_WRITE ) != 0 )  {
    err |= !access_ok( VERIFY_READ, (void *)arg, _IOC_SIZE( cmd ));
  }
  if ( err ) {
    CHUMACCEL_DEBUG( Error, "arg pointer is invalid\n" );
    return -EFAULT;
  }
  // end safety code

  err = 0;
  switch (cmd) {
  case ACCEL_IOCTL_SETROM:
    if( copy_from_user(&dciddat, (void *)arg, sizeof dciddat) )
      return -EFAULT;
    
    gRomBusy = 1; // this shuts off the kernel task timer from using the SPI bus
    waitEEPROMbusy();
    issueWREN();

    SSP_CTRL_REG(SPI_CHAN) &= 0xFFFFFFE0;
    SSP_CTRL_REG(SPI_CHAN) |= SSP_WS(32);
    ACCEL_SEL_CS3;
    ndelay(50);
    ACCEL_SEL_CS1; 
    spi_exchange_data( ((SPIROM_WRITE_CMD & 0xFF) << 24) |   // write the data
		       ((dciddat.address & 0xFFFF) << 8) |
		       (dciddat.data & 0xFF) );
    ACCEL_SEL_CS3;

    waitEEPROMbusy();
    issueWRDI();

    ACCEL_SEL_CS3;
    ndelay(50);
    ACCEL_SEL_CS1;
    SSP_CTRL_REG(SPI_CHAN) &= 0xFFFFFFE0;
    SSP_CTRL_REG(SPI_CHAN) |= SSP_WS(32);
    retdat = spi_exchange_data( ((SPIROM_READ_CMD & 0xFF) << 24) |   // read it back
				((dciddat.address & 0xFFFF) << 8) );
    ACCEL_SEL_CS3;

    CHUMDCID_DEBUG( Ioctl, "Got %02X from %04X\n", retdat & 0xFF, dciddat.address & 0xFFFF );
    dciddat.data = (unsigned char) retdat & 0xFF;
    gRomBusy = 0; // restores kernel task timer's ability to use SPI bus

    err = copy_to_user((void *)arg, &dciddat, sizeof dciddat) ? -EFAULT : 0;
    break;
  case ACCEL_IOCTL_READROM:
    if( copy_from_user(&dciddat, (void *)arg, sizeof dciddat) )
      return -EFAULT;
    gRomBusy = 1; // this shuts off the kernel task timer from using the SPI bus

    waitEEPROMbusy();

    ACCEL_SEL_CS3;
    ndelay(50);
    ACCEL_SEL_CS1;
    SSP_CTRL_REG(SPI_CHAN) &= 0xFFFFFFE0;
    SSP_CTRL_REG(SPI_CHAN) |= SSP_WS(32);
    retdat = spi_exchange_data( ((SPIROM_READ_CMD & 0xFF) << 24) | 
				((dciddat.address & 0xFFFF) << 8) );
    ACCEL_SEL_CS3;

    gRomBusy = 0; // restores kernel task timer's ability to use SPI bus
    dciddat.data = (unsigned char) retdat & 0xFF;

    err = copy_to_user((void *)arg, &dciddat, sizeof dciddat) ? -EFAULT : 0;
    break;

  case ACCEL_IOCTL_READSTAT:
    gRomBusy = 1; // this shuts off the kernel task timer from using the SPI bus

    ACCEL_SEL_CS3;
    ndelay(50);
    ACCEL_SEL_CS1;  // this is the SPI rom chip select code
    SSP_CTRL_REG(SPI_CHAN) &= 0xFFFFFFE0;
    SSP_CTRL_REG(SPI_CHAN) |= SSP_WS(16);
    retdat = spi_exchange_data( SPIROM_RDSR_CMD << 8 ); // check status register
    ACCEL_SEL_CS3;

    dciddat.data = (unsigned char) retdat & 0xFF;

    err = copy_to_user((void *)arg, &dciddat, sizeof dciddat) ? -EFAULT : 0;
    gRomBusy = 0; // restores kernel task timer's ability to use SPI bus
    break;
    
  case ACCEL_IOCTL_LOCKROM:
    gRomBusy = 1; // this shuts off the kernel task timer from using the SPI bus
    waitEEPROMbusy();
    issueWREN();

    ACCEL_SEL_CS3;
    ndelay(50);
    ACCEL_SEL_CS1;
    SSP_CTRL_REG(SPI_CHAN) &= 0xFFFFFFE0;
    SSP_CTRL_REG(SPI_CHAN) |= SSP_WS(16);  // WRSR operation
    spi_exchange_data((SPIROM_WRSR_CMD & 0xFF << 8) | 
		      0x04); // just write protect addresses 0x300-0x3FF
    ACCEL_SEL_CS3;

    waitEEPROMbusy();
    issueWRDI();
    err = 0;
    gRomBusy = 0; // restores kernel task timer's ability to use SPI bus
    break;
    
  case ACCEL_IOCTL_UNLOCKROM:
    gRomBusy = 1; // this shuts off the kernel task timer from using the SPI bus
    waitEEPROMbusy();
    issueWREN();

    ACCEL_SEL_CS3;
    ndelay(50);
    ACCEL_SEL_CS1;
    SSP_CTRL_REG(SPI_CHAN) &= 0xFFFFFFE0;
    SSP_CTRL_REG(SPI_CHAN) |= SSP_WS(16);  // WRSR operation
    spi_exchange_data((SPIROM_WRSR_CMD & 0xFF << 8) | 
		      0x00); // unlock everything
    ACCEL_SEL_CS3;

    waitEEPROMbusy();
    issueWRDI();
    err = 0;
    gRomBusy = 0; // restores kernel task timer's ability to use SPI bus
    break;

  default:
    return -EINVAL;
  }

  return err;
 
}


static void __exit chumby_accel_exit(void) {
  dev_t devno = MKDEV(accel_major, accel_minor);

  CHUMACCEL_DEBUG( Trace, "Top of exit.\n" );
  // set global flag that we're out of here and force a call to remove ourselves
  gDone = 1;
  del_timer( &(acceltask_data.timer) ); // delete our kernel task

  mdelay(200);  // wait to dequeue self; if kernel panics on rmmod try adding 100 to this value
  
  // insert all cleanup stuff here
  cdev_del( acceltask_data.accel_cdev );
  remove_proc_entry( "accel", NULL );
  unregister_chrdev_region(devno, accel_nr_devs);

  devno = MKDEV(dcid_major, dcid_minor);
  cdev_del( dcidtask_data.dcid_cdev );
  unregister_chrdev_region(devno, dcid_nr_devs);

#if USE_SYSCTL
  if ( gSysCtlHeader != NULL ) {
    unregister_sysctl_table( gSysCtlHeader );
  }
  if ( gSysCtlHeader_dcid != NULL ) {
    unregister_sysctl_table( gSysCtlHeader_dcid );
  }
#endif
  
}

// entry and exit mappings
module_init(chumby_accel_init);
module_exit(chumby_accel_exit);
