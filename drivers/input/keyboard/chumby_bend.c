/*
    chumby_bend.c
    bunnie -- June 2006 -- 1.4 -- linux 2.4.20
    bunnie -- March 2007 -- 2.0 -- port to Ironforge linux 2.6.16
    bunnie -- December 2007 -- 2.1 -- port to v3.8 and inclusion of watchdog timer feature, reset reason reporting
    sean   -- June 2009 -- 2.2 -- port to Falconwing, and removal of everything but bend sensor functionality.
    sean   -- August 2009 -- 2.3 -- Rewrote entirely and made a keyboard device

    This file is part of the chumby sensor suite driver in the linux kernel.
    Copyright (c) Chumby Industries, 2009

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

#define FW_BEND_VERSION "2.3-Falconwing"
#define BEND_KEYCODE 130

#include <linux/moduleparam.h>

#include <linux/kernel.h>   /* printk() */
#include <linux/slab.h>     /* kmalloc() */
#include <linux/errno.h>    /* error codes */
#include <linux/types.h>    /* size_t */
#include <linux/fcntl.h>    /* O_ACCMODE */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/io.h>

#include <mach/regs-pinctrl.h>



static int legacy_open(struct inode *inode, struct file *file);
static int legacy_release(struct inode *inode, struct file *file);
static ssize_t legacy_read(struct file *file, char *buf,
                           size_t count, loff_t * ppos);


// True or false depending on whether the switch is pressed or not.
static int pressed = 0;


// For legacy switch mode
#include <linux/cdev.h>
#include <asm/uaccess.h>    /* copy_*_user */
static struct cdev *legacy_switch;
static int sense1_major, sense1_minor;
dev_t legacy_dev;
static struct file_operations legacy_fops = {
    .owner =    THIS_MODULE,
    .read =     legacy_read,
    .open =     legacy_open,
    .release =  legacy_release,
};

// End legacy switch mode





#if 0
#define CHLOG(format, arg...)            \
    printk("chumby_bend.c - %s():%d - " format, __func__, __LINE__, ## arg)
#else
#define CHLOG(...)
#endif




// Do some debouncing.  Figure that events must happen 5 jiffies apart.
static unsigned int last_jiffies = 0;
static irqreturn_t chumby_bend_pressed(int irq, void *arg)
{
    struct input_dev *input_dev = arg;
    pressed = !(HW_PINCTRL_DIN1_RD()&0x40000000);


    // Set the key to fire when the polarity of the switch flips.
    if(pressed)
        HW_PINCTRL_IRQPOL1_SET(0x40000000);
    else
        HW_PINCTRL_IRQPOL1_CLR(0x40000000);

    // Fire only if the event happened more than 5 jiffies ago.
    if(jiffies > last_jiffies+5) {
        CHLOG(">>> Pressed switch: %d\n", pressed);
        input_event(input_dev, EV_KEY, BEND_KEYCODE, pressed);
    }
    else {
        CHLOG(">>> Debouncing switch, not sending event.  Jiffies: %ld  Last_jiffies: %d\n", jiffies, last_jiffies);
    }
    last_jiffies = jiffies;

    return IRQ_HANDLED;
}




// Legacy code.  Note that since the value will only be updated during an
// interrupt context, we don't need locking around these commands.
static int legacy_open(struct inode *inode, struct file *file) {
    return 0;
}
static int legacy_release(struct inode *inode, struct file *file) {
    return 0;
}
static ssize_t legacy_read(struct file *file, char *buf,
                           size_t count, loff_t * ppos) {
    char retval = !!pressed;
    if(copy_to_user(buf, &retval, sizeof(retval)))
        return -EFAULT;
    return sizeof(retval);
}




static int chumby_bend_probe(struct platform_device *pdev)
{
    int ret;
    struct input_dev *input_dev;

    // Let people know all about this lovely driver.
    printk("Chumby bend sensor driver version %s initializing "
           "(scross@chumby.com)...\n",
           FW_BEND_VERSION);


    // Allocate input-dev required structures.
    input_dev = input_allocate_device();
    platform_set_drvdata(pdev, input_dev);



    // Allocate the hardware by reassigning the pins.

    // Set up the bend sensor to be a GPIO.  It lives on bank 1, pin 30.
    HW_PINCTRL_MUXSEL3_SET(0x30000000);

    // Set up the bend sensor to fire an IRQ when pressed.
    HW_PINCTRL_IRQLEVEL1_CLR(0x40000000);

    // Set the IRQ to fire on rising edge, indicating the switch was pressed.
    HW_PINCTRL_IRQPOL1_CLR(0x40000000);

    // Set up the IRQ to fire.
    HW_PINCTRL_IRQSTAT1_CLR(0x40000000);
    HW_PINCTRL_PIN2IRQ1_SET(0x40000000);
    HW_PINCTRL_IRQEN1_SET(0x40000000);



    // Grab the IRQ for bank1.
    ret = request_irq(IRQ_GPIO1, chumby_bend_pressed, 0, "Bend sensor", input_dev);
    if(ret < 0) {
        CHLOG("Unable to initialize IRQ for bend sensor: %d\n", ret);
        goto fail;
    }


    // Indicate that we send key events.
    set_bit(EV_KEY, input_dev->evbit);
    bitmap_fill(input_dev->keybit, KEY_MAX);
    //input_set_capability(input, EV_KEY, BEND_KEYCODE);


    input_dev->name = pdev->name;
    input_dev->phys = "chumby_bend/input0";
    input_dev->dev.parent = &pdev->dev;

    input_dev->id.bustype = BUS_HOST;
    input_dev->id.vendor  = 0x0001;
    input_dev->id.product = 0x0001;
    input_dev->id.version = 0x0100;


    ret = input_register_device(input_dev);
    if(ret) {
        CHLOG("Unable to register device\n");
        goto fail;
    }




    // Backwards-compatibility mode:
    // Older software polls /dev/switch and obtains a binary value to
    // determine whether it's pressed or not.
    legacy_switch = cdev_alloc();
    if(!legacy_switch) {
        CHLOG("Error adding legacy switch mode\n");
        goto fail;
    }

    // Allocate device numbes for the legacy switch device.
    ret = alloc_chrdev_region(&legacy_dev, sense1_minor, 1, "switch");
    if(ret<0) {
        printk(KERN_WARNING 
               "bend: Can't get major number for compatibility mode: %d\n",
               ret);
        goto fail;
    }
    sense1_major = MAJOR(legacy_dev);


    // Create the /dev entry with all of the necessary file options.
    cdev_init(legacy_switch, &legacy_fops);
    legacy_switch->owner = THIS_MODULE;
    legacy_switch->ops   = &legacy_fops;
    ret = cdev_add(legacy_switch, legacy_dev, 1);
    if(ret) {
        printk(KERN_NOTICE "Error %d adding switch device\n", ret);
        goto fail;
    }





    return 0;

fail:
    CHLOG(KERN_NOTICE "Error %d adding switch device\n", ret);
    input_free_device(input_dev);
    return -EINVAL;
}

static int __devexit chumby_bend_remove(struct platform_device *pdev)
{
    struct input_dev *input_dev = platform_get_drvdata(pdev);

    // Legacy mode
    dev_t devno = MKDEV(sense1_major, sense1_minor);
    cdev_del(legacy_switch);
    unregister_chrdev_region(devno, 1);


    CHLOG("Removing bend sensor\n");
    free_irq(IRQ_GPIO1, input_dev);
    input_unregister_device(input_dev);
    platform_set_drvdata(pdev, NULL);
    return 0;
}


static int chumby_bend_suspend(struct platform_device *pdev, pm_message_t state)
{
    return 0;
}

static int chumby_bend_resume(struct platform_device *pdev)
{
    return 0;
}



static struct platform_driver chumby_bend_driver = {
    .probe  = chumby_bend_probe,
    .remove = __devexit_p(chumby_bend_remove),
    .suspend    = chumby_bend_suspend,
    .resume     = chumby_bend_resume,
    .driver = {
        .name  = "bend-sensor",
        .owner = THIS_MODULE,
    },
};



static int __devinit chumby_bend_init(void)
{
    CHLOG("Initializing driver\n");
    return platform_driver_register(&chumby_bend_driver);
}

static void __exit chumby_bend_exit(void) {
    platform_driver_unregister(&chumby_bend_driver);
}

// entry and exit mappings
module_init(chumby_bend_init);
module_exit(chumby_bend_exit);



MODULE_AUTHOR("scross@chumby.com");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:bend-sensor");
