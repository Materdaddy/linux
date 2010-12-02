/*
       imx21_emma.c
       ghutchins -- August 2007 -- initial version

       This file is part of the chumby imx21 emma driver in the linux kernel.
       Copyright (c) Chumby Industries, 2007

       The emma driver is free software; you can redistribute it and/or modify
       it under the terms of the GNU General Public License as published by
       the Free Software Foundation; either version 2 of the License, or
       (at your option) any later version.

       The emma driver is distributed in the hope that it will be useful,
       but WITHOUT ANY WARRANTY; without even the implied warranty of
       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
       GNU General Public License for more details.

       You should have received a copy of the GNU General Public License
       along with the Chumby; if not, write to the Free Software
       Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/

#include <linux/config.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/ioctl.h>
#include <linux/device.h>
#include <linux/mm.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/mman.h>
#include <linux/random.h>
#include <linux/raw.h>
#include <linux/poll.h>
#include <linux/capability.h>
#include <linux/smp_lock.h>
#include <linux/devfs_fs_kernel.h>
#include <linux/ptrace.h>
#include <linux/cdev.h>
#include <linux/highmem.h>
#include <linux/bootmem.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/dma-mapping.h>

#include <asm/uaccess.h>
#include <asm/io.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/dma.h>
#include <asm/arch/imx-dma.h>
#include <asm/arch/imx-regs.h>


#include "chumby_emma.h"


MODULE_AUTHOR("ghutchins@gmail.com");
MODULE_LICENSE("GPL");



int emma_major = 0;          // 0 denotes dynamic allocation
module_param(emma_major, int, S_IRUGO);


static struct cdev*             emma_cdev = NULL;
static atomic_t                 opened = ATOMIC_INIT( -1 );
static struct fasync_struct*    fasync_queue = NULL;
static DECLARE_WAIT_QUEUE_HEAD(waitq);
static atomic_t                 pp_done = ATOMIC_INIT( 0 );
static atomic_t                 prp_done = ATOMIC_INIT( 0 );
static EMMA_PP_REGS             pp_regs = { 0 };
static EMMA_PrP_REGS            prp_regs = { 0 };


static int pphw_reset(void)
{
	int i;

	EMMA_PP_CNTL = PP_CNTL_SWRST;
	for (i = 0; i < 1000; i++) 
		if (!(EMMA_PP_CNTL & 0x100))
			break;
	return EMMA_PP_CNTL == PP_CNTL_RSTVAL? 0 : -1;
}


static int prphw_reset(void)
{
	int i;

	EMMA_PrP_CH1_RZ_HORI_COEF1 = 0x5A;
	if (EMMA_PrP_CH1_RZ_HORI_COEF1 != 0x5A)
        return -1;

	EMMA_PrP_CNTL = PrP_CNTL_SWRST;
	for (i = 0; i < 1000; i++)
		if (!(EMMA_PrP_CNTL & PrP_CNTL_SWRST))
			break;
    return EMMA_PrP_CNTL == PrP_CNTL_RSTVAL? 0 : -2;
}


static int emma_open(struct inode* inode, struct file* file)
{
    int rc;

    if ( !atomic_inc_and_test( &opened ) ) {
        atomic_dec( &opened );
        return -EBUSY;
    }

    rc = pphw_reset();
    if (rc) {
        printk( "%s/%s(): pphw_reset() returned %d\n",
                __FILE__, __FUNCTION__, rc );
        atomic_dec( &opened );
        return -EIO;
    }
    rc = prphw_reset();
    if (rc) {
        printk( "%s/%s(): prphw_reset() returned %d\n",
                __FILE__, __FUNCTION__, rc );
        atomic_dec( &opened );
        return -EIO;
    }
    return 0;
}


static int emma_release(struct inode* inode, struct file* file)
{
    if (fasync_queue != NULL)
        (void)fasync_helper(-1, file, 0, &fasync_queue);

    atomic_dec( &opened );
    return 0;
}


static int emma_ioctl(struct inode* inode, struct file* file, 
                      unsigned int cmd, unsigned long arg)
{
    EMMA_PP_STAT  pp_stat;
    EMMA_PrP_STAT prp_stat;
    unsigned long enflags;
    int i;

    switch (cmd) {
    case EMMA_PP_SET_REGS:
        printk( "EMMA_PP_SET_REGS...\n" );
        if (copy_from_user(&pp_regs, (void __user *)arg, sizeof(pp_regs)))
            return -EFAULT;
        EMMA_PP_SOURCE_Y_PTR            = pp_regs.source_y_ptr;
        EMMA_PP_SOURCE_CB_PTR           = pp_regs.source_cb_ptr;
        EMMA_PP_SOURCE_CR_PTR           = pp_regs.source_cr_ptr;
        EMMA_PP_DEST_RGB_PTR            = pp_regs.dest_rgb_ptr;
        EMMA_PP_QUANTIZER_PTR           = pp_regs.quantizer_ptr;
        EMMA_PP_PROCESS_FRAME_PARA      = pp_regs.process_frame_para;
        EMMA_PP_SOURCE_FRAME_WIDTH      = pp_regs.source_frame_width;
        EMMA_PP_DEST_DISPLAY_WIDTH      = pp_regs.dest_display_width;
        EMMA_PP_DEST_IMAGE_SIZE         = pp_regs.dest_image_size;
        EMMA_PP_DEST_FRAME_FMT_CNTL     = pp_regs.dest_frame_fmt_cntl;
        EMMA_PP_RESIZE_TBL_INDEX        = pp_regs.resize_table_index_reg;
        EMMA_PP_CSC_COEF_0123           = pp_regs.csc_coef_0123;
        for (i=0; i<ARRAY_SIZE(pp_regs.resize_coef_tbl); ++i) {
            EMMA_PP_RESIZE_COEF_TBL(i)  = pp_regs.resize_coef_tbl[i];
        }
        EMMA_PP_CNTL                    = pp_regs.cntl;

        #if 0
        printk( "SOURCE_Y_PTR=0x%lx SOURCE_CB_PTR=0x%lx SOURCE_CR_PTR=0x%lx\n",
                EMMA_PP_SOURCE_Y_PTR, EMMA_PP_SOURCE_CB_PTR, EMMA_PP_SOURCE_CR_PTR );
        printk( "DEST_RGB_PTR=0x%lx QUANTIZER_PTR=0x%lx PROCESS_FRAME_PARA=0x%lx\n",
                EMMA_PP_DEST_RGB_PTR, EMMA_PP_QUANTIZER_PTR, EMMA_PP_PROCESS_FRAME_PARA );
        printk( "SOURCE_FRAME_WIDTH=0x%lx DEST_DISPLAY_WIDTH=0x%lx DEST_IMAGE_SIZE=0x%lx\n",
                EMMA_PP_SOURCE_FRAME_WIDTH, EMMA_PP_DEST_DISPLAY_WIDTH, EMMA_PP_DEST_IMAGE_SIZE );
        printk( "DEST_FRAME_FMT_CNTL=0x%lx PP_CSC_COEF_0123=0x%lx PP_CNTL=0x%lx\n",
                EMMA_PP_DEST_FRAME_FMT_CNTL, EMMA_PP_CSC_COEF_0123, EMMA_PP_CNTL );
        #endif

        return 0;

    case EMMA_PP_GET_REGS:
        printk( "EMMA_PP_GET_REGS...\n" );
        pp_regs.cntl                    = EMMA_PP_CNTL;
        pp_regs.source_y_ptr            = EMMA_PP_SOURCE_Y_PTR;
        pp_regs.source_cb_ptr           = EMMA_PP_SOURCE_CB_PTR;
        pp_regs.source_cr_ptr           = EMMA_PP_SOURCE_CR_PTR;
        pp_regs.dest_rgb_ptr            = EMMA_PP_DEST_RGB_PTR;
        pp_regs.quantizer_ptr           = EMMA_PP_QUANTIZER_PTR;
        pp_regs.process_frame_para      = EMMA_PP_PROCESS_FRAME_PARA;
        pp_regs.source_frame_width      = EMMA_PP_SOURCE_FRAME_WIDTH;
        pp_regs.dest_display_width      = EMMA_PP_DEST_DISPLAY_WIDTH;
        pp_regs.dest_image_size         = EMMA_PP_DEST_IMAGE_SIZE;
        pp_regs.dest_frame_fmt_cntl     = EMMA_PP_DEST_FRAME_FMT_CNTL;
        pp_regs.resize_table_index_reg  = EMMA_PP_RESIZE_TBL_INDEX;
        pp_regs.csc_coef_0123           = EMMA_PP_CSC_COEF_0123;
        //pp_regs.resize_coef_tbl         = EMMA_PP_RESIZE_COEF_TBL; wo reg!
        return copy_to_user((void __user *)arg, &pp_regs, sizeof(pp_regs))? -EFAULT : 0;

    case EMMA_PP_STATUS:
        printk( "EMMA_PP_STATUS...\n" );
        pp_stat.cntl       = EMMA_PP_CNTL;
        pp_stat.intrcntl   = EMMA_PP_INTRCNTL;
        pp_stat.intrstatus = EMMA_PP_INTRSTATUS;
        pp_stat.done       = atomic_read( &pp_done );
        return copy_to_user((void __user *)arg, &pp_stat, sizeof(pp_stat))? -EFAULT : 0;

    case EMMA_PP_START:
        if ( EMMA_PP_CNTL & PP_CNTL_EN ) {
            printk( "%s/%s(EMMA_PP_START): already busy!\n", 
                    __FILE__, __FUNCTION__ );
            return -EBUSY;
        }
        EMMA_PP_INTRCNTL = PP_ERR_INTR | PP_FRAME_COMP_INTR;
        EMMA_PP_CNTL = pp_regs.cntl | PP_CNTL_EN;
        printk( "PP_CNTL=0x%x, PP_INTRCNTL=0x%x, PP_INTRSTATUS=0x%x\n",
                EMMA_PP_CNTL, EMMA_PP_INTRCNTL, EMMA_PP_INTRSTATUS );
        return 0;

    case EMMA_PP_ABORT:
        if ( ( EMMA_PP_CNTL & PP_CNTL_EN ) == 0 ) {
            return -EIO;
        }
        EMMA_PP_CNTL = ( pp_regs.cntl & ~PP_CNTL_EN ) | PP_CNTL_SWRST;
        return 0;

    case EMMA_PP_RESET:
        EMMA_PP_CNTL = ( pp_regs.cntl & ~PP_CNTL_EN ) | PP_CNTL_SWRST;
        return 0;

    case EMMA_PrP_SET_REGS:
        if (copy_from_user(&prp_regs, (void __user *)arg, sizeof(prp_regs)))
            return -EFAULT;
        EMMA_PrP_SOURCE_Y_PTR          = prp_regs.source_y_ptr;
        EMMA_PrP_SOURCE_CB_PTR         = prp_regs.source_cb_ptr;
        EMMA_PrP_SOURCE_CR_PTR         = prp_regs.source_cr_ptr;
        EMMA_PrP_DEST_RGB1_PTR         = prp_regs.dest_rgb1_ptr;
        EMMA_PrP_DEST_RGB2_PTR         = prp_regs.dest_rgb2_ptr;
        EMMA_PrP_DEST_Y_PTR            = prp_regs.dest_y_ptr;
        EMMA_PrP_DEST_CB_PTR           = prp_regs.dest_cb_ptr;
        EMMA_PrP_DEST_CR_PTR           = prp_regs.dest_cr_ptr;
        EMMA_PrP_SOURCE_FRAME_SIZE     = prp_regs.source_frame_size;
        EMMA_PrP_CH1_LINE_STRIDE       = prp_regs.ch1_line_stride;
        EMMA_PrP_SRC_PIXEL_FORMAT_CNTL = prp_regs.src_pixel_format_cntl;
        EMMA_PrP_CH1_PIXEL_FORMAT_CNTL = prp_regs.ch1_pixel_format_cntl;
        EMMA_PrP_CH1_OUT_IMAGE_SIZE    = prp_regs.ch1_out_image_size;
        EMMA_PrP_CH2_OUT_IMAGE_SIZE    = prp_regs.ch2_out_image_size;
        EMMA_PrP_SOURCE_LINE_STRIDE    = prp_regs.source_line_stride;
        EMMA_PrP_CSC_COEF_012          = prp_regs.csc_coef_012;
        EMMA_PrP_CSC_COEF_345          = prp_regs.csc_coef_345;
        EMMA_PrP_CSC_COEF_678          = prp_regs.csc_coef_678;
        EMMA_PrP_CH1_RZ_HORI_COEF1     = prp_regs.ch1_rz_hori_coef1;
        EMMA_PrP_CH1_RZ_HORI_COEF2     = prp_regs.ch1_rz_hori_coef2;
        EMMA_PrP_CH1_RZ_HORI_VALID     = prp_regs.ch1_rz_hori_valid;
        EMMA_PrP_CH1_RZ_VERT_COEF1     = prp_regs.ch1_rz_vert_coef1;
        EMMA_PrP_CH1_RZ_VERT_COEF2     = prp_regs.ch1_rz_vert_coef2;
        EMMA_PrP_CH1_RZ_VERT_VALID     = prp_regs.ch1_rz_vert_valid;
        EMMA_PrP_CH2_RZ_HORI_COEF1     = prp_regs.ch2_rz_hori_coef1;
        EMMA_PrP_CH2_RZ_HORI_COEF2     = prp_regs.ch2_rz_hori_coef2;
        EMMA_PrP_CH2_RZ_HORI_VALID     = prp_regs.ch2_rz_hori_valid;
        EMMA_PrP_CH2_RZ_VERT_COEF1     = prp_regs.ch2_rz_vert_coef1;
        EMMA_PrP_CH2_RZ_VERT_COEF2     = prp_regs.ch2_rz_vert_coef2;
        EMMA_PrP_CH2_RZ_VERT_VALID     = prp_regs.ch2_rz_vert_valid;
        EMMA_PrP_CNTL                  = prp_regs.cntl;

        #if 0
        printk( "SOURCE_Y_PTR=0x%lx SOURCE_CB_PTR=0x%lx SOURCE_CR_PTR=0x%lx\n",
                EMMA_PrP_SOURCE_Y_PTR, EMMA_PrP_SOURCE_CB_PTR, EMMA_PrP_SOURCE_CR_PTR );
        printk( "DEST_RGB1_PTR=0x%lx DEST_RGB2_PTR=0x%lx DEST_Y_PTR=0x%lx\n",
                EMMA_PrP_DEST_RGB1_PTR, EMMA_PrP_DEST_RGB2_PTR, EMMA_PrP_DEST_Y_PTR );
        printk( "DEST_CB_PTR=0x%lx DEST_CR_PTR=0x%lx SOURCE_FRAME_SIZE=0x%lx\n",
                EMMA_PrP_DEST_CB_PTR, EMMA_PrP_DEST_CR_PTR, EMMA_PrP_SOURCE_FRAME_SIZE );
        printk( "CH1_LINE_STRIDE=0x%lx SRC_PIXEL_FORMAT_CNTL=0x%lx CH1_PIXEL_FORMAT_CNTL=0x%lx\n",
                EMMA_PrP_CH1_LINE_STRIDE, EMMA_PrP_SRC_PIXEL_FORMAT_CNTL, EMMA_PrP_CH1_PIXEL_FORMAT_CNTL );
        printk( "CH1_OUT_IMAGE_SIZE=0x%lx CH2_OUT_IMAGE_SIZE=0x%lx SOURCE_LINE_STRIDE=0x%lx\n",
                EMMA_PrP_CH1_OUT_IMAGE_SIZE, EMMA_PrP_CH2_OUT_IMAGE_SIZE, EMMA_PrP_SOURCE_LINE_STRIDE );
        printk( "CSC_COEF_012=0x%lx CSC_COEF_345=0x%lx CSC_COEF_678=0x%lx\n",
                EMMA_PrP_CSC_COEF_012, EMMA_PrP_CSC_COEF_345, EMMA_PrP_CSC_COEF_678 );
        printk( "CH1_RZ_HORI_COEF1=0x%lx CH1_RZ_HORI_COEF2=0x%lx CH1_RZ_HORI_VALID=0x%lx\n",
                EMMA_PrP_CH1_RZ_HORI_COEF1, EMMA_PrP_CH1_RZ_HORI_COEF2, EMMA_PrP_CH1_RZ_HORI_VALID );
        printk( "CH1_RZ_VERT_COEF1=0x%lx CH1_RZ_VERT_COEF2=0x%lx CH1_RZ_VERT_VALID=0x%lx\n",
                EMMA_PrP_CH1_RZ_VERT_COEF1, EMMA_PrP_CH1_RZ_VERT_COEF2, EMMA_PrP_CH1_RZ_VERT_VALID );
        printk( "CH2_RZ_HORI_COEF1=0x%lx CH2_RZ_HORI_COEF2=0x%lx CH2_RZ_HORI_VALID=0x%lx\n",
                EMMA_PrP_CH2_RZ_HORI_COEF1, EMMA_PrP_CH2_RZ_HORI_COEF2, EMMA_PrP_CH2_RZ_HORI_VALID );
        printk( "CH2_RZ_VERT_COEF1=0x%lx CH2_RZ_VERT_COEF2=0x%lx CH2_RZ_VERT_VALID=0x%lx\n",
                EMMA_PrP_CH2_RZ_VERT_COEF1, EMMA_PrP_CH2_RZ_VERT_COEF2, EMMA_PrP_CH2_RZ_VERT_VALID );
        printk( "PrP_CNTL=0x%lx\n", EMMA_PrP_CNTL );
        #endif

        return 0;

    case EMMA_PrP_GET_REGS:
        prp_regs.source_y_ptr           = EMMA_PrP_SOURCE_Y_PTR;
        prp_regs.source_cb_ptr          = EMMA_PrP_SOURCE_CB_PTR;
        prp_regs.source_cr_ptr          = EMMA_PrP_SOURCE_CR_PTR;
        prp_regs.dest_rgb1_ptr          = EMMA_PrP_DEST_RGB1_PTR;
        prp_regs.dest_rgb2_ptr          = EMMA_PrP_DEST_RGB2_PTR;
        prp_regs.dest_y_ptr             = EMMA_PrP_DEST_Y_PTR;
        prp_regs.dest_cb_ptr            = EMMA_PrP_DEST_CB_PTR;
        prp_regs.dest_cr_ptr            = EMMA_PrP_DEST_CR_PTR;
        prp_regs.source_frame_size      = EMMA_PrP_SOURCE_FRAME_SIZE;
        prp_regs.ch1_line_stride        = EMMA_PrP_CH1_LINE_STRIDE;
        prp_regs.src_pixel_format_cntl  = EMMA_PrP_SRC_PIXEL_FORMAT_CNTL;
        prp_regs.ch1_pixel_format_cntl  = EMMA_PrP_CH1_PIXEL_FORMAT_CNTL;
        prp_regs.ch1_out_image_size     = EMMA_PrP_CH1_OUT_IMAGE_SIZE;
        prp_regs.ch2_out_image_size     = EMMA_PrP_CH2_OUT_IMAGE_SIZE;
        prp_regs.source_line_stride     = EMMA_PrP_SOURCE_LINE_STRIDE;
        prp_regs.csc_coef_012           = EMMA_PrP_CSC_COEF_012;
        prp_regs.csc_coef_345           = EMMA_PrP_CSC_COEF_345;
        prp_regs.csc_coef_678           = EMMA_PrP_CSC_COEF_678;
        prp_regs.ch1_rz_hori_coef1      = EMMA_PrP_CH1_RZ_HORI_COEF1;
        prp_regs.ch1_rz_hori_coef2      = EMMA_PrP_CH1_RZ_HORI_COEF2;
        prp_regs.ch1_rz_hori_valid      = EMMA_PrP_CH1_RZ_HORI_VALID;
        prp_regs.ch1_rz_vert_coef1      = EMMA_PrP_CH1_RZ_VERT_COEF1;
        prp_regs.ch1_rz_vert_coef2      = EMMA_PrP_CH1_RZ_VERT_COEF2;
        prp_regs.ch1_rz_vert_valid      = EMMA_PrP_CH1_RZ_VERT_VALID;
        prp_regs.ch2_rz_hori_coef1      = EMMA_PrP_CH2_RZ_HORI_COEF1;
        prp_regs.ch2_rz_hori_coef2      = EMMA_PrP_CH2_RZ_HORI_COEF2;
        prp_regs.ch2_rz_hori_valid      = EMMA_PrP_CH2_RZ_HORI_VALID;
        prp_regs.ch2_rz_vert_coef1      = EMMA_PrP_CH2_RZ_VERT_COEF1;
        prp_regs.ch2_rz_vert_coef2      = EMMA_PrP_CH2_RZ_VERT_COEF2;
        prp_regs.ch2_rz_vert_valid      = EMMA_PrP_CH2_RZ_VERT_VALID;
        prp_regs.cntl                   = EMMA_PrP_CNTL;
        return copy_to_user((void __user *)arg, &prp_regs, sizeof(prp_regs))? -EFAULT : 0;

    case EMMA_PrP_STATUS:
        prp_stat.cntl       = EMMA_PrP_CNTL;
        prp_stat.intrcntl   = EMMA_PrP_INTRCNTL;
        prp_stat.intrstatus = EMMA_PrP_INTRSTATUS;
        prp_stat.done       = atomic_read( &prp_done );
        return copy_to_user((void __user *)arg, &prp_stat, sizeof(prp_stat))? -EFAULT : 0;

    case EMMA_PrP_START:
        enflags = (unsigned long) arg;
        if ( EMMA_PrP_CNTL & enflags ) {
            printk( "%s/%s(): EMMA_PrP_START: sorry busy!\n", 
                    __FILE__, __FUNCTION__ );
            return -EBUSY;
        }
        EMMA_PrP_INTRCNTL = PrP_INT_MASK;
        EMMA_PrP_CNTL = prp_regs.cntl | enflags;
        return 0;

    case EMMA_PrP_ABORT:
        if ( ( EMMA_PrP_CNTL & PrP_CNTL_EN ) == 0 ) {
            return -EIO;
        }
        EMMA_PrP_CNTL = ( prp_regs.cntl & ~PrP_CNTL_EN ) | PrP_CNTL_SWRST;
        return 0;

    case EMMA_PrP_RESET:
        EMMA_PrP_CNTL = ( prp_regs.cntl & ~PrP_CNTL_EN ) | PrP_CNTL_SWRST;
        return 0;

    default:
        break;
    }

    return -ENXIO;
}


static int emma_fasync(int fd, struct file* file, int mode)
{
    return fasync_helper(fd, file, mode, &fasync_queue);
}


static unsigned int emma_poll(struct file* file, poll_table* wait)
{
    unsigned int mask = 0;

    poll_wait( file, &waitq, wait );
    if ( atomic_read( &pp_done ) ) {
        atomic_set( &pp_done, 0 ); // is this ok?  or use ioctl() to clear?
        mask |= POLLIN | POLLRDNORM;
    }
    if ( atomic_read( &prp_done ) ) {
        atomic_set( &prp_done, 0 ); // is this ok?  or use ioctl() to clear?
        mask |= POLLIN | POLLRDNORM;
    }
    return mask;
}


static irqreturn_t pp_irq_handler(int irq, void* data, struct pt_regs* regs)
{
	int	rt = EMMA_PP_INTRSTATUS & PP_INT_MASK;
    printk( "%s/%s(): rt=0x%x\n", __FILE__, __FUNCTION__, rt );
	if (!rt) {
		printk("%s/%s(): spurious interrupt\n", __FILE__, __FUNCTION__); 
		return IRQ_HANDLED;
	}

    if (rt & PP_ERR_INTR) {
		printk("%s/%s(): ERROR\n", __FILE__, __FUNCTION__); 
    }

    atomic_inc( &pp_done );
    if ( fasync_queue != NULL )
        kill_fasync( &fasync_queue, SIGIO, POLL_IN );
    wake_up_interruptible( &waitq );

	EMMA_PP_INTRSTATUS = rt;	/* clr irq */
    return IRQ_HANDLED;
}


static irqreturn_t prp_irq_handler(int irq, void* data, struct pt_regs* regs)
{
	int	rt = EMMA_PrP_INTRSTATUS & PrP_INT_MASK;
	if (!rt) {
		printk("%s/%s(): spurious interrupt\n", __FILE__, __FUNCTION__); 
		return IRQ_HANDLED;
	}

	if (rt & (PrP_INTRSTAT_RDERR  | 
              PrP_INTRSTAT_CH1ERR | 
			  PrP_INTRSTAT_CH2ERR))
		printk("%s/%s(): isr bus error\n", __FILE__, __FUNCTION__); 
	else if ((rt & (PrP_ISR_FLOW | PrP_ISR_OVR)))
		printk("%s/%s(): isr internal error\n", __FILE__, __FUNCTION__); 

    /* silicon bug?? enable bit does not self clear? */
    if (rt & (PrP_INTRSTAT_CH1BUF1 | PrP_INTRSTAT_CH1BUF2 | 
              PrP_INTRSTAT_CH2BUF1 | PrP_INTRSTAT_CH2BUF2)) {
        if(!(EMMA_PrP_CNTL & PrP_CNTL_CH1_LOOP))
            EMMA_PrP_CNTL &= ~PrP_CNTL_CH1EN;
        if(!(EMMA_PrP_CNTL & PrP_CNTL_CH2_LOOP))
            EMMA_PrP_CNTL &= ~PrP_CNTL_CH2EN;
    }

    atomic_inc( &prp_done );
    if ( fasync_queue != NULL )
        kill_fasync( &fasync_queue, SIGIO, POLL_IN );
    wake_up_interruptible( &waitq );

	EMMA_PrP_INTRSTATUS = rt;	/* clr irq */
    return IRQ_HANDLED;
}



static struct file_operations emma_fops = {
    .owner      =    THIS_MODULE,
    .open       =    emma_open,
    .release    =    emma_release,
    .ioctl      =    emma_ioctl,
    .fasync     =    emma_fasync,
    .poll       =    emma_poll,
};


static void __exit emma_exit(void) 
{
    if (emma_cdev != NULL) {
        cdev_del(emma_cdev);
        unregister_chrdev_region(MKDEV(emma_major, 1), 1);
        emma_cdev = NULL;

        free_irq( INT_EMMAPRP, NULL );
        free_irq( INT_EMMAPP, NULL );
    }
}


static int __init emma_init(void) 
{
    dev_t dev = 0;
    int err;

    emma_cdev = cdev_alloc();
    if (emma_cdev == NULL) {
        printk(KERN_WARNING "%s/%s(): cdev_alloc() failed!\n",
             __FILE__, __FUNCTION__);
        return -ENOMEM;
    }
    
    if (emma_major) {
        dev = MKDEV(emma_major, 1);
        err = register_chrdev_region(dev, 1, "emma");
    } else {
        err = alloc_chrdev_region(&dev, 1, 1, "emma");
        emma_major = MAJOR(dev);
    }
    if (err < 0) {
        printk(KERN_WARNING "%s/%s(): can't get major %d (err=%d)\n", 
               __FILE__, __FUNCTION__, emma_major, err);
        cdev_del(emma_cdev);
        emma_cdev = NULL;
        return err;
    }

    cdev_init(emma_cdev, &emma_fops);
    emma_cdev->owner = THIS_MODULE;
    emma_cdev->ops = &emma_fops;
    err = cdev_add(emma_cdev, dev, 1);
    if (err) {
        printk(KERN_NOTICE 
               "%s/%s(): cdev_add() failed with error %d\n", 
               __FILE__, __FUNCTION__, err);
        emma_exit();
        return err;
    }

    err = request_irq( INT_EMMAPRP, prp_irq_handler, 0, "EMMA-PRP", NULL );
    if (err < 0) {
        printk(KERN_NOTICE 
               "%s/%s(): request_irq(INT_EMMAPRP) failed with error %d\n", 
               __FILE__, __FUNCTION__, err);
        emma_exit();
        return err;
    }
    err = request_irq( INT_EMMAPP,  pp_irq_handler, 0, "EMMA-PP", NULL );
    if (err < 0) {
        printk(KERN_NOTICE 
               "%s/%s(): request_irq(INT_EMMAPRP) failed with error %d\n", 
               __FILE__, __FUNCTION__, err);
        emma_exit();
        return err;
    }

    MAX_MGPCR(4) = 1;
    __REG(IMX_AIPI1_BASE + 8) = 0;
    __REG(IMX_AIPI2_BASE + 8) = 0;
    PCCR0 |= PCCR0_EMMA_EN | PCCR0_HCLK_EMMA_EN;

    printk(KERN_NOTICE "EMMA device major=%d.\n", emma_major);
    return 0;
}


module_init(emma_init);
module_exit(emma_exit);

