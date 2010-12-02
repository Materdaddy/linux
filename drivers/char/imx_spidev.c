/*
 * User-space interface to the SPI bus on Freescale i.MX
 *
 * (c) SAN People (Pty) Ltd
 *
 * Based on SPI driver by Rick Bronson
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/config.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/highmem.h>
#include <linux/pagemap.h>
#include <linux/imx_spi.h>

#undef DEBUG_SPIDEV

#define SPI_D_C     9 | GPIO_PORTE | GPIO_GPIO | GPIO_OUT       // SPI data/command (data = 1)

#define LDISP_CMD       _IO(SPI_MAJOR, 1049)
#define LDISP_DATA      _IO(SPI_MAJOR, 1050)

static char bspiCmdData = 0;
                      
/* ......................................................................... */

/*
 * Read or Write to SPI bus.
 */
static ssize_t spidev_rd_wr(struct file *file, char *buf, size_t count, loff_t *offset)
{
    unsigned int spi_device = (unsigned int) file->private_data;

    char *rdata;
    char *wdata;
    struct spi_transfer_list* list;
    char *c;
    int j;

    if (!count) {
        return 0;
    }
	
    list = kmalloc(sizeof(struct spi_transfer_list), GFP_KERNEL);
    if (!list) {
        return -ENOMEM;
    }

    wdata = kmalloc(count, GFP_KERNEL);
    if (wdata == NULL) {
        kfree(list);
        return -ENOMEM;
    }

    rdata = kmalloc(count, GFP_KERNEL);
    if (rdata == NULL) {
        kfree(wdata);
        kfree(list);
        return -ENOMEM;
    }

    if (copy_from_user(wdata, buf, count) != 0) {
        kfree(rdata);
        kfree(wdata);
        kfree(list);
        return -EFAULT;
    }

    list->tx[0] = wdata;
    list->txlen[0] = count;
    list->rx[0] = rdata;
    list->rxlen[0] = count;
    list->nr_transfers = 1;
    list->curr = 0;
    list->txpos = 0;
    list->rxpos = 0;
    
    if(bspiCmdData)
    {
        imx_gpio_write(SPI_D_C,1);
    }
    else
    {
        imx_gpio_write(SPI_D_C,0);
    }
    /* Perform transfer on SPI bus */
    spi_access_bus(spi_device);
    spi_transfer(spi_device, list);
    spi_release_bus(spi_device);

    if (copy_to_user(buf, rdata, count) != 0) {
        kfree(rdata);
        kfree(wdata);
        kfree(list);
        return -EFAULT;
    }

    
    kfree(rdata);
    kfree(wdata);
    kfree(list);
    
    return count;
}

static int spidev_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
    int spi_device = MINOR(inode->i_rdev);

    if (spi_device >= NR_SPI_DEVICES)
            return -ENODEV;

    // TODO: This interface can be used to configure the SPI bus.
    // Configurable options could include: Speed, Clock Polarity, Clock Phase

    switch(cmd) 
    {
        case LDISP_CMD:
        {
            bspiCmdData = 0;
            break;
        }
        case LDISP_DATA:
        {
            bspiCmdData = 1;
            break;
        }
        
        default:
            return -ENOIOCTLCMD;
    }
    return -ENOIOCTLCMD;
}

/*
 * Open the SPI device
 */
static int spidev_open(struct inode *inode, struct file *file)
{
    unsigned int spi_device = MINOR(inode->i_rdev);

    if (spi_device >= NR_SPI_DEVICES)
        return -ENODEV;

    /*
     * 'private_data' is actually a pointer, but we overload it with the
     * value we want to store.
     */
    file->private_data = (void *)spi_device;

    return 0;
}

/*
 * Close the SPI device
 */
static int spidev_close(struct inode *inode, struct file *file)
{
    return 0;
}

/* ......................................................................... */

static struct file_operations spidev_fops = {
    owner:	THIS_MODULE,
    llseek:	no_llseek,
    read:	spidev_rd_wr,
    write:	(int (*) (struct file *file, const char *buf, size_t count, loff_t *offset))spidev_rd_wr,
    ioctl:	spidev_ioctl,
    open:	spidev_open,
    release:	spidev_close,
};

/*
 * Install the SPI /dev interface driver
 */
static int __init imx_spidev_init(void)
{
	
    if (register_chrdev(SPI_MAJOR, "spi", &spidev_fops)) {
        printk(KERN_ERR "imx_spidev: Unable to get major %d for SPI bus\n", 
               SPI_MAJOR);
        return -EIO;
    }

    printk(KERN_INFO "i.MX SPI driver loaded\n");

    return 0;
}

/*
 * Remove the SPI /dev interface driver
 */
static void __exit imx_spidev_exit(void)
{
	
    if (unregister_chrdev(SPI_MAJOR, "spi")) {
        printk(KERN_ERR "imx_spidev: Unable to release major %d for SPI bus\n",
               SPI_MAJOR);
        return;
    }
}

module_init(imx_spidev_init);
module_exit(imx_spidev_exit);

MODULE_LICENSE("GPL")
MODULE_AUTHOR("Jay Monkman")
MODULE_DESCRIPTION("SPI /dev interface for Freescale i.MX")
