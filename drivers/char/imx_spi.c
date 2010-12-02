/*
 * SPI character driver for the MX21
 *
 * Copyright (C) 2006 Loping Dog Embedded Systems
 * Jay Monkman <jtm@lopingdog.com>
 * 
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <asm/semaphore.h>
#include <linux/pci.h>
#include <linux/sched.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <asm/hardware.h>
#include <linux/imx_spi.h>
#include <linux/delay.h>

static struct spi_local spi_dev[NR_SPI_DEVICES];


/*
 * Access and enable the SPI bus.
 * This MUST be called before any transfers are performed.
 */
void spi_access_bus(short device)
{
    /* Ensure that requested device is valid */
    if ((device < 0) || (device >= NR_SPI_DEVICES)) {
        panic("imx_spi: spi_access_bus called with invalid device");
    }
    
//    SSP_CTRL_REG(device) |= SSP_ENABLE;
            
    spi_dev[device].spi_enabled++;

    /* Lock the SPI bus */
    down(&spi_dev[device].spi_lock);

}

/*
 * Relinquish control of the SPI bus.
 */
void spi_release_bus(short device)
{
    spi_dev[device].spi_enabled--;
//    if (spi_dev[device].spi_enabled == 0) {
//        SSP_CTRL_REG(device) &= ~SSP_ENABLE;
//    }

    /* Release the SPI bus */
    up(&spi_dev[device].spi_lock);

}

static void setup_xfer(int dev, struct spi_transfer_list *list)
{
    char *txdata;

    txdata = (char *)list->tx[list->curr];
    
    while((SSP_INT_REG(dev) & SSP_INT_TF) == 0) {
        SSP_TX_REG(dev) = txdata[list->txpos];
        list->txpos++;
        
        if (list->txpos >= list->txlen[list->curr]) {
            break;
        }
    }
#if 0
    if (list->txpos >= list->txlen[list->curr]) {
        SSP_INT_REG(dev) |= SSP_INT_TEEN;  /* FIFO empty */
        SSP_INT_REG(dev) &= ~SSP_INT_THEN;
    } else {
        SSP_INT_REG(dev) |= SSP_INT_THEN;  /* FIFO 1/2 empty */
        SSP_INT_REG(dev) &= ~SSP_INT_TEEN;  
    }
#else
    SSP_INT_REG(dev) |= SSP_INT_TSHFEEN;  /* Shift reg empty */
#endif
}
    
    
/*
 * Perform a data transfer over the SPI bus
 */
int spi_transfer(short dev, struct spi_transfer_list* list)
{
    struct spi_local *device = &spi_dev[dev];
     
    if (!list) {
        panic("imx_spi: spi_transfer called with NULL transfer list");
    }
    
    /* Store transfer list */
    device->xfers = list;
    list->rxpos = 0;
    list->txpos = 0;
    list->curr = 0;
 
    SSP_CTRL_REG(dev) &= ~SSP_XCH;
    setup_xfer(dev, list);
    SSP_CTRL_REG(dev) |= SSP_XCH;
    
    while (list->curr < list->nr_transfers) {
        udelay(10);
    }
//    wait_for_completion(&device->transfer_complete);
    
#ifdef DEBUG_SPI
    printk("SPI transfer end\n");
#endif
    
    return 0;
}


static irqreturn_t imx_spi_isr(int irq, void *dev_id, struct pt_regs *regs)
{
    int dev = (int) dev_id;
    struct spi_local *device = &spi_dev[dev];
    struct spi_transfer_list *list = device->xfers;
    char *rxdata;
    char *txdata;

    rxdata = (char *)list->rx[list->curr];
    while (SSP_INT_REG(dev) & SSP_INT_RR) {
        rxdata[list->rxpos] = SSP_TX_REG(dev) & 0xff;
        list->rxpos++;

        if (list->rxpos >= list->rxlen[list->curr]) {
            break;
        }
    }

    if (SSP_INT_REG(dev) & SSP_INT_TSHFE) {
        if (list->txpos >= list->txlen[list->curr]) {
            list->curr++;
            list->rxpos = 0;
            list->txpos = 0;
        }
        if (list->curr >= list->nr_transfers) {
            SSP_INT_REG(dev) &= ~SSP_INT_TSHFEEN;
//            complete(&device->transfer_complete);
        } else {
            setup_xfer(dev, list);
            SSP_CTRL_REG(dev) |= SSP_XCH;            
        }
    } else {
        txdata = (char *)list->tx[list->curr];
        setup_xfer(dev, list);
        SSP_CTRL_REG(dev) |= SSP_XCH;            
    }
        
    
    return IRQ_HANDLED;
}

static int __init imx_spi_init(void)
{
    int i;
    int rc;

    rc = request_irq(INT_CSPI1, &imx_spi_isr, 0, "IMX-SPI1", (void *)0);
    if (rc < 0) {
        return -ENODEV;
    }

    rc = request_irq(INT_CSPI2, &imx_spi_isr, 0, "IMX-SPI2", (void *)1);
    if (rc < 0) {
        free_irq(INT_CSPI1, 0);
        return rc;    }

    rc = request_irq(INT_CSPI3, &imx_spi_isr, 0, "IMX-SPI3", (void *)2);
    if (rc < 0) {
        free_irq(INT_CSPI1, 0);
        free_irq(INT_CSPI2, 0);
        return rc;
    }

    if (request_mem_region((unsigned long)&__PHYS_REG(IMX_SPI1_BASE), 
                           0x1000, "imx-spi1") == NULL) {
        printk("Error requesting region for SPI1\n");
        free_irq(INT_CSPI1, 0);
        free_irq(INT_CSPI2, 0);
        free_irq(INT_CSPI3, 0);
        return -ENODEV;
    }

    if (request_mem_region((unsigned long)&__PHYS_REG(IMX_SPI2_BASE),
                           0x1000, "imx-spi2") == NULL) {
        printk("Error requesting region for SPI2\n");
        release_mem_region(__PHYS_REG(IMX_SPI1_BASE), 0x2000);
        free_irq(INT_CSPI1, 0);
        free_irq(INT_CSPI2, 0);
        free_irq(INT_CSPI3, 0);
        return -ENODEV;
    }

    if (request_mem_region((unsigned long)&__PHYS_REG(IMX_SPI3_BASE),
                           0x1000, "imx-spi3") == NULL) {
        printk("Error requesting region for SPI3\n");
        release_mem_region(__PHYS_REG(IMX_SPI1_BASE), 0x1000);
        release_mem_region(__PHYS_REG(IMX_SPI2_BASE), 0x1000);
        free_irq(INT_CSPI1, 0);
        free_irq(INT_CSPI2, 0);
        free_irq(INT_CSPI3, 0);
        return -ENODEV;
    }

    memset(&spi_dev, 0, sizeof(spi_dev));

    for (i = 0; i < NR_SPI_DEVICES; i++) {
        init_MUTEX(&spi_dev[i].spi_lock);
        
        switch (i) {
        case 0: PCCR0 |= PCCR0_CSPI1_EN; break;
        case 1: PCCR0 |= PCCR0_CSPI2_EN; break;
        case 2: PCCR1 |= PCCR1_CSPI3_EN; break;
        }

        /* Configure SPI port */
        SSP_CTRL_REG(i) = (
            SSP_RATE_DIV256 |/* PCLK2 = SYSCLK/4 = 24MHz. 24MHZ/125KHz = 192 */
            SSP_MODE_MASTER |
            SSP_SS_POL_LOW  |
            SSP_ENABLE      |
            SSP_PHA1        |
            SSP_POL1        |
            SSP_WS(8));
    }

    return 0;
}

static void imx_spi_exit(void)
{
    int i;

    for (i = 0; i < NR_SPI_DEVICES; i++) {

        /* Configure SPI port */
        SSP_CTRL_REG(i) = SSP_DISABLE;

        switch (i) {
        case 0: PCCR0 &= ~PCCR0_CSPI1_EN; break;
        case 1: PCCR0 &= ~PCCR0_CSPI2_EN; break;
        case 2: PCCR1 &= ~PCCR1_CSPI3_EN; break;
        }

        init_completion(&spi_dev[i].transfer_complete);

    }

    release_mem_region(__PHYS_REG(IMX_SPI1_BASE), 0x1000);
    release_mem_region(__PHYS_REG(IMX_SPI2_BASE), 0x1000);
    release_mem_region(__PHYS_REG(IMX_SPI3_BASE), 0x1000);
    free_irq(INT_CSPI1, 0);
    free_irq(INT_CSPI2, 0);
    free_irq(INT_CSPI3, 0);

}


module_init(imx_spi_init);
module_exit(imx_spi_exit);


MODULE_LICENSE("GPL")
MODULE_AUTHOR("Jay Monkman")
MODULE_DESCRIPTION("SPI driver for Freescale i.MX")
