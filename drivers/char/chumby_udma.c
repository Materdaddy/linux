/*
       linux/drivers/char/chumby_udma.c

       ghutchins -- August 2007 -- 1.0

       This file is part of the chumby user dma driver in the linux kernel.
       Copyright (c) Chumby Industries, 2007

       The user dma driver is free software; you can redistribute it and/or modify
       it under the terms of the GNU General Public License as published by
       the Free Software Foundation; either version 2 of the License, or
       (at your option) any later version.

       The user dma driver is distributed in the hope that it will be useful,
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


#include "chumby_udma.h"


MODULE_AUTHOR("ghutchins@gmail.com");
MODULE_LICENSE("GPL");



int udma_major = 0;          // 0 denotes dynamic allocation
int udma_bufsz = 0x180000;   // default to 1.5Mb DMA buffer

module_param(udma_major, int, S_IRUGO);
module_param(udma_bufsz, int, S_IRUGO);


static struct cdev*             udma_cdev = NULL;
static atomic_t                 opened = ATOMIC_INIT( -1 );
static void*                    buf_addr = NULL;
static dma_addr_t               dma_addr = 0;
static atomic_t                 dma_done = ATOMIC_INIT( 0 );
static int                      dma_lock[ IMX_DMA_CHANNELS ] = { 0 };
static DECLARE_WAIT_QUEUE_HEAD(dma_wait);
static struct fasync_struct*    fasync_queue = NULL;




static int udma_open(struct inode* inode, struct file* file)
{
    if ( atomic_inc_and_test( &opened ) )
        return 0;

    atomic_dec( &opened );
    return -EBUSY;
}


static int udma_release(struct inode* inode, struct file* file)
{
    imx_dmach_t dmac;

    /* abort all active DMAC transfers
    */
    for ( dmac = 0; dmac < IMX_DMA_CHANNELS; ++dmac ) {
        if ( dma_lock[dmac] ) {
            imx_dma_disable( dmac );
            imx_dma_free( dmac );
            dma_lock[dmac] = 0;
        }
    }

    if (fasync_queue != NULL /*&& file->f_flags & FASYNC*/)
        (void)fasync_helper(-1, file, 0, &fasync_queue);

    atomic_dec( &opened );
    return 0;
}


static void dma_isr(int irq, void* data, struct pt_regs* regs)
{
    imx_dmach_t dmac = ( imx_dmach_t ) data;
    unsigned long bits;

    if (DBTOSR != 0) {
        bits = DBTOSR;
        printk("DMA Error: DBTOSR = 0x%lx\n", bits);
        DBTOSR = bits;
    }
    if (DRTOSR != 0) {
        bits = DRTOSR;
        printk("DMA Error: DRTOSR = 0x%lx\n", bits);
        DRTOSR = bits;
    }
    if (DSESR != 0) {
        bits = DSESR;
        printk("DMA Error: DSESR = 0x%lx\n", bits);
        DSESR = bits;
    }
    if (DBOSR != 0) {
        bits = DBOSR;
        printk("DMA Error: DBOSR = 0x%lx\n", bits);
        DBOSR = bits;
    }

    imx_dma_disable(dmac);

    atomic_inc( &dma_done );
    if ( fasync_queue != NULL )
        kill_fasync( &fasync_queue, SIGIO, POLL_IN );
    wake_up_interruptible( &dma_wait );
}


static int udma_ioctl(struct inode* inode, struct file* file, 
                      unsigned int cmd, unsigned long arg)
{
    UDMA_DESC       desc;
    UDMA_REGS       regs;
    UDMA_STATUS     status;
    UDMA_LCDC_REGS  lcd_regs;
    imx_dmach_t     dmac = ( imx_dmach_t ) arg;
    unsigned long   pa;
    unsigned long   pgoff;
    int             rc;

    switch (cmd) {
    case UDMA_GETBUFSZ:         // return the size of the allocated DMA buffer
        return put_user( udma_bufsz, (unsigned long __user *)arg );

    case UDMA_ALLOC_DMA:        // allocate the specified DMAC 0 to 15
        if ( dmac >= IMX_DMA_CHANNELS )
            return -EINVAL;
        if ( imx_dma_request( dmac, "CHUMBY-UDMA" ) < 0 )
            return -EBUSY;

        imx_dma_setup_handlers( dmac, dma_isr, NULL/*dma_err*/, (void*)arg );
        dma_lock[ dmac ] = 1;
        return 0;

    case UDMA_FREE_DMA:         // free the specified DMAC 0 to 15
        if ( dmac >= IMX_DMA_CHANNELS )
            return -EINVAL;
        if ( !dma_lock[ dmac ] )
            return -EBADF;

        imx_dma_disable( dmac );
        imx_dma_free( dmac );
        dma_lock[ dmac ] = 0;
        return 0;

    case UDMA_GET_DMA_REGS:
        rc = get_user(regs.dmac, (unsigned long __user *)arg);
        if ( rc )
            return rc;
        if ( regs.dmac >= IMX_DMA_CHANNELS )
            return -EINVAL;
        if ( !dma_lock[ regs.dmac ] )
            return -EBADF;
        regs.ccr  = CCR(regs.dmac);
        regs.rssr = RSSR(regs.dmac);
        regs.blr  = BLR(regs.dmac);
        regs.bucr = BUCR(regs.dmac);
        regs.wsra = WSRA;
        regs.xsra = XSRA;
        regs.ysra = YSRA;
        regs.wsrb = WSRB;
        regs.xsrb = XSRB;
        regs.ysrb = YSRB;
        return copy_to_user((void __user *)arg, &regs, sizeof(regs))? -EFAULT : 0;

    case UDMA_SET_DMA_REGS:
        if (copy_from_user(&regs, (void __user *)arg, sizeof(regs)))
            return -EFAULT;
        if ( regs.dmac >= IMX_DMA_CHANNELS )
            return -EINVAL;
        if ( !dma_lock[ regs.dmac ] )
            return -EBADF;
        CCR(regs.dmac)  = regs.ccr;
        RSSR(regs.dmac) = regs.rssr;
        BLR(regs.dmac)  = regs.blr;
        BUCR(regs.dmac) = regs.bucr;
        WSRA = regs.wsra;
        XSRA = regs.xsra;
        YSRA = regs.ysra;
        WSRB = regs.wsrb;
        XSRB = regs.xsrb;
        YSRB = regs.ysrb;
        return 0;

    case UDMA_START_DMA:
        if (copy_from_user(&desc, (UDMA_REGS __user *)arg, sizeof(desc)))
            return -EFAULT;
        if ( desc.dmac >= IMX_DMA_CHANNELS )
            return -EINVAL;
        if ( !dma_lock[ desc.dmac ] )
            return -EBADF;

        while (CCR(desc.dmac) & CCR_CEN) {
            printk("DMA is already enabled\n");
        }
        SAR(desc.dmac)  = desc.sar;
        DAR(desc.dmac)  = desc.dar;
        CNTR(desc.dmac) = desc.cntr;
        imx_dma_enable(desc.dmac);
        return 0;

    case UDMA_ABORT_DMA:
        if ( dmac >= IMX_DMA_CHANNELS )
            return -EINVAL;
        if ( !dma_lock[ dmac ] )
            return -EBADF;
        imx_dma_disable(dmac);
        return 0;

    case UDMA_DMA_STATUS:
        rc = get_user(status.dmac, (unsigned long __user *)arg);
        if ( rc )
            return rc;
        if ( status.dmac >= IMX_DMA_CHANNELS )
            return -EINVAL;
        if ( !dma_lock[ status.dmac ] )
            return -EBADF;
        status.ccr  = CCR(status.dmac);
        return copy_to_user((void __user *)arg, &status, sizeof(status))? -EFAULT : 0;

    case UDMA_PA:
        rc = get_user(pgoff, (unsigned long __user *)arg);
        if ( rc )
            return rc;
        pa = dma_addr + (pgoff << PAGE_SHIFT);
        return __put_user(pa, (unsigned long __user *)arg);

    case UDMA_FBPA:
        lcd_regs.ssa = LCDC_SSA;
        lcd_regs.lgwsar = LCDC_LGWSAR;
        lcd_regs.lgwpor = LCDC_LGWPOR;
        return copy_to_user((void __user *)arg, &lcd_regs, sizeof(lcd_regs))? -EFAULT : 0;

    default:
        break;
    }

    return -ENXIO;
}


#ifndef ARCH_HAS_VALID_PHYS_ADDR_RANGE
static inline int valid_mmap_phys_addr_range(unsigned long addr, size_t *size)
{
    return 1;
}
#endif


static int udma_mmap(struct file * file, struct vm_area_struct * vma)
{
    size_t size = vma->vm_end - vma->vm_start;
    unsigned long dma_pfn = ((IMX_DMAC_BASE & ~IMX_IO_BASE) | IMX_IO_PHYS) >> PAGE_SHIFT;
    unsigned long map_pfn = __pa((u64)vma->vm_pgoff << PAGE_SHIFT) >> PAGE_SHIFT;

    /* two cases: 1. mapping DMAC, 1 page starting at DMAC address (&DCR),
                  2. mapping contigous buffer allocated by udma_init()
     */
    if ( map_pfn == dma_pfn ) {    
        // mapping DMAC
        if ( size != PAGE_SIZE )
            return -EIO;
    } else {
        // mapping udma_init() buf
        map_pfn = (dma_addr >> PAGE_SHIFT) + vma->vm_pgoff;   
        if (!pfn_valid(map_pfn)) {
            printk( "%s/%s(): PHYS_PFN_OFFSET=%x, max_mapnr=%lx\n",
                    __FILE__, __FUNCTION__, PHYS_PFN_OFFSET, max_mapnr );
            return -EIO;
        }
    }

    vma->vm_pgoff = map_pfn;
    if (!valid_mmap_phys_addr_range(vma->vm_pgoff << PAGE_SHIFT, &size))
        return -EINVAL;

    vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

    /* Remap-pfn-range will mark the range VM_IO and VM_RESERVED */
    if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff, size,
                        vma->vm_page_prot))
        return -EAGAIN;

    return 0;
}


static int udma_fasync(int fd, struct file* file, int mode)
{
    return fasync_helper(fd, file, mode, &fasync_queue);
}


static unsigned int udma_poll(struct file* file, poll_table* wait)
{
    unsigned int mask = 0;

    poll_wait( file, &dma_wait, wait );
    if ( atomic_read( &dma_done ) ) {
        atomic_set( &dma_done, 0 ); // is this ok?  or use ioctl() to clear?
        mask |= POLLIN | POLLRDNORM;
    }
    return mask;
}


static struct file_operations udma_fops = {
    .owner      =    THIS_MODULE,
    .open       =    udma_open,
    .release    =    udma_release,
    .ioctl      =    udma_ioctl,
    .fasync     =    udma_fasync,
    .mmap       =    udma_mmap,
    .poll       =    udma_poll,
};



static void __exit udma_exit(void) 
{
    if (udma_cdev != NULL) {
        cdev_del(udma_cdev);
        unregister_chrdev_region(MKDEV(udma_major, 1), 1);
        udma_cdev = NULL;

        if ( buf_addr != NULL ) {
            dma_free_coherent( NULL, udma_bufsz, buf_addr, dma_addr );
            buf_addr = NULL;
        }
    }
}


static int __init udma_init(void) 
{
    dev_t dev = 0;
    int err;

    udma_cdev = cdev_alloc();
    if (udma_cdev == NULL) {
        printk(KERN_WARNING "%s/%s(): cdev_alloc() failed!\n",
             __FILE__, __FUNCTION__);
        return -ENOMEM;
    }
    
    if (udma_major) {
        dev = MKDEV(udma_major, 1);
        err = register_chrdev_region(dev, 1, "udma");
    } else {
        err = alloc_chrdev_region(&dev, 1, 1, "udma");
        udma_major = MAJOR(dev);
    }
    if (err < 0) {
        printk(KERN_WARNING "%s/%s(): can't get major %d (err=%d)\n", 
               __FILE__, __FUNCTION__, udma_major, err);
        cdev_del(udma_cdev);
        udma_cdev = NULL;
        return err;
    }

    cdev_init(udma_cdev, &udma_fops);
    udma_cdev->owner = THIS_MODULE;
    udma_cdev->ops = &udma_fops;
    err = cdev_add(udma_cdev, dev, 1);
    if (err) {
        printk(KERN_NOTICE 
               "%s/%s(): cdev_add() failed with error %d\n", 
               __FILE__, __FUNCTION__, err);
        udma_exit();
        return err;
    }

    do {
        buf_addr = dma_alloc_coherent( NULL, udma_bufsz, &dma_addr, GFP_KERNEL );
        if ( buf_addr != NULL ) 
            break;
        udma_bufsz >>= 1;
    } while ( udma_bufsz > 0 );

    if ( buf_addr == NULL ) {
        printk( KERN_NOTICE "%s/%s(): Unable to allocate USER DMA memory\n",
                __FILE__, __FUNCTION__ );
        udma_exit();
        return -ENOMEM;
    }

    #if 1
    /* make DMAC registers accessable in user space -- not necessary if ioctl()
    ** interface is adequate or security issues with user space access
    */
    __REG(IMX_AIPI1_BASE + 8) = 0;
    __REG(IMX_AIPI2_BASE + 8) = 0;
    #endif

    printk(KERN_NOTICE "UserDMA device major=%d, allocated %d bytes. (buf_addr=%p, dma_addr=%x)\n",
           udma_major, udma_bufsz, buf_addr, dma_addr);
    return 0;
}


module_init(udma_init);
module_exit(udma_exit);
