/*
 * USB Host Controller Driver for IMX21
 *
 * Copyright (C) 2006 Loping Dog Embedded Systems
 * Written by Jay Monkman <jtm@lopingdog.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/config.h>

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/usb.h>
#include <linux/dma-mapping.h>
#include <asm/arch/imx21-hcd.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/byteorder.h>

#include "../core/hcd.h"

//#define DEBUG
//#define GRH_DBUG_PRINTK

#define DMEM_SIZE   4096
#define USB_XFERMEM 0xe0025000

#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#ifdef DEBUG
#  define DBG(fmt, args...) \
            dbug_printk("%lu-%s().%d: " fmt, jiffies, __FUNCTION__, __LINE__, ## args)
//#  define DEBUG_ALL
#else
#  define DBG(stuff...) do {} while(0)
#endif
#define ERR(fmt, args...)   \
            printk(KERN_ERR "ERROR!!! %s/%s().%d: " fmt, \
                   __FILE__, __FUNCTION__, __LINE__, ## args)

#include "imx21-hcd.h"

#ifdef CONFIG_PM
#error "IMX21 USB HCD driver doesn't support PM"
#endif

#ifdef GRH_DBUG_PRINTK
#include "grh-dbug.c"
#endif


static const char hcd_name[] = "imx21-hc";


/***************************************************************************
 * disable host controller interrupts
 **************************************************************************/
static inline void hc_stop_int(void)
{
    USBH_SYSIEN = 0;
}


static int imx21_hc_get_frame(struct usb_hcd *hcd)
{
    return USBH_FRMNUB & 0xFFFF;
}


static int alloc_etd(struct imx21 *imx21)
{
    int i;

    for (i = 0; i < USB_NUM_ETD; i++) {
        if (imx21->etd[i].alloc == 0) {
            imx21->etd[i].alloc = 1;
            return i;
        }
    }

    ERR("All ETDs are busy!\n");
    return -ENODEV;
}


static void free_etd(struct imx21 *imx21, int num)
{
    if (num < 0 || num >= USB_NUM_ETD) {
        ERR("BAD etd=%d!\n", num);
        return;
    }
    if (imx21->etd[num].alloc == 0)
        ERR("ETD %d already free!\n", num);
    else {
        etd_priv_t *etd = imx21->etd + num;
        volatile u32 *etd_addr = &USB_ETD(num);
        int etd_mask = (1<<num);

        USBH_ETDENCLR = etd_mask;
        USBH_ETDDONEEN &= ~etd_mask;
        mx2otg_clear_toggling_bit(USB_ETDDMAEN, etd_mask);
        USB_ETDSMSA(num) = 0;
        etd_addr[0] = 0;
        etd_addr[1] = 0;
        etd_addr[2] = 0;
        etd_addr[3] = 0;
        etd->td     = NULL;
        etd->buf    = NULL;
        etd->urb    = NULL;
        etd->ep     = NULL;
        etd->busy   = 0;
        etd->alloc  = 0;
    }
}


static int alloc_dmem(struct imx21 *imx21, int size, 
                      struct usb_host_endpoint *ep)
{
    int offset = 0;
    imx21_dmem_area_t *area;
    imx21_dmem_area_t **p;
    imx21_dmem_area_t *tmp;
    
    spin_lock(imx21->lock);
    
    /* Round the size up */
    size += (~size + 1) & 0x3;
    
    if (size > DMEM_SIZE) {
        spin_unlock(imx21->lock);
        ERR("size=%d > DMEM_SIZE(%d)\n", size, DMEM_SIZE);
        return -ENOMEM;
    }
    
    area = kmalloc(sizeof(imx21_dmem_area_t), GFP_ATOMIC);
    if ( area == NULL) {
        spin_unlock(imx21->lock);
        ERR("no kmalloc() memory!\n");
        return -ENOMEM;
    }
    
    p = &imx21->dmem_list;
    tmp = *p;
    while (tmp != NULL) {
        if ((size + offset) < offset) {
            goto fail;
        }
        if ((size + offset) <= tmp->offset) {
            break;
        }
        offset = tmp->size + tmp->offset;
        if ((offset + size) > DMEM_SIZE) {
            goto fail;
        }
        p = &tmp->next;
        tmp = *p;
    } 

    area->ep = ep;
    area->offset = offset;
    area->size = size;
    area->next = *p;
    *p = area;
    
    spin_unlock(imx21->lock);
    return offset;
    
 fail:
    spin_unlock(imx21->lock);
    kfree(area);
    ERR("no DMEM memory!\n");
    return -ENOMEM;
}


static void free_dmem(struct imx21 *imx21, int offset)
{
    imx21_dmem_area_t **p;
    imx21_dmem_area_t *tmp;
    
    spin_lock(imx21->lock);
    p = &imx21->dmem_list;
    tmp = *p;
    while (tmp != NULL) {
        if (tmp->offset == offset) {
            *p = tmp->next;
            kfree(tmp);
            spin_unlock(imx21->lock);
            return;
        }
        p = &tmp->next;
        tmp = *p;
    } 
    spin_unlock(imx21->lock);
    ERR("Trying to free data memory %d that wasn't allocated.\n", offset);
}    


static void free_epdmem(struct imx21 *imx21, struct usb_host_endpoint *ep)
{
    imx21_dmem_area_t **p;
    imx21_dmem_area_t *tmp;
    
    spin_lock(imx21->lock);
    p = &imx21->dmem_list;
    tmp = *p;
    while (tmp != NULL) {
        if (tmp->ep != ep) 
            p = &tmp->next;
        else {
            ERR("Active data memory %d for disabled ep=%p.\n", tmp->offset, ep);
            *p = tmp->next;
            kfree(tmp);
        }
        tmp = *p;
    } 
    spin_unlock(imx21->lock);
}    


static int get_hub_descriptor(struct usb_hcd *hcd,
                              struct usb_hub_descriptor *desc)
{
    desc->bDescriptorType = 0x29;  /* HUB descriptor */
    desc->bHubContrCurrent = 0;
    
    desc->bNbrPorts = USBH_ROOTHUBA & USBH_ROOTHUBA_NDNSTMPRT_MASK;
    desc->bDescLength = 9;
    
    desc->bPwrOn2PwrGood = 0;
    
    desc->wHubCharacteristics = (__force __u16)cpu_to_le16(
        0x0002 | /* No power switching */
        0x0010 | /* No over current protection */
        0);
    
    desc->bitmap[0] = 1 << 1;
    desc->bitmap[1] = ~0;
    return 0;
}


static void schedule_iso_etds(struct usb_hcd *hcd, struct usb_host_endpoint *ep)
{
    struct imx21 *imx21 = hcd_to_imx21(hcd);
    ep_priv_t *ep_priv = ep->hcpriv;
    etd_priv_t *etd;
    int etd_num;
    int etd_mask;
    volatile u32 *etd_addr;
    td_t *td;
    int i;
    int maxpacket;
    urb_priv_t *urb_priv;

    for (i = 0; i < NUM_ISO_ETDS; i++) {
        if (list_empty(&ep_priv->td_list)) {
            break;
        }

        etd_num = ep_priv->etd[i];
        if (etd_num < 0) {
            break;
        }

        etd = &imx21->etd[etd_num];
        if (etd->busy) {
            continue;
        }

        td = list_entry(ep_priv->td_list.next, td_t, list);
        maxpacket = usb_maxpacket(td->urb->dev, td->urb->pipe, 
                                  usb_pipeout(td->urb->pipe));
        td->buf_addr = alloc_dmem(imx21, maxpacket, ep);
        if (td->buf_addr < 0) {
            ERR("no memory\n");
            return;
        }

        /* Remove it from the list only after we know all will succeed. */
        list_del(&td->list);
        urb_priv = td->urb->hcpriv;
        urb_priv->active = 1;

        etd->busy = 1;
        etd->td   = td;
        etd->ep   = td->ep;
        etd->urb  = td->urb;
        etd->len  = td->len;
        etd->buf  = NULL;

        if (td->urb->transfer_flags & URB_ISO_ASAP) {
            ushort this_frame = (ushort) imx21_hc_get_frame(hcd);
            ushort next_frame = ep_priv->last_frame + td->urb->interval;
            ushort max_interval = td->urb->interval * NUM_ISO_ETDS;
            ushort delta = next_frame - this_frame;

            if (delta > (max_interval + NUM_ISO_ETDS)) {
                DBG("delta=%u, max_interval=%u, this_frame=%x, next_frame=%x, last_frame=%x\n",
                    delta, max_interval, this_frame, next_frame, ep_priv->last_frame);
                next_frame = this_frame + td->urb->interval;
            }
            td->urb->start_frame = next_frame;
            ep_priv->last_frame = next_frame;
        }

        //DBG("urb=%lx, ep=%lx, etd=%d, td=%lx, buf=%x, len=%x, frame=%x (curframe=%x)\n",
        //    td->urb, ep, etd_num, td, td->buf_addr, td->len, 
        //    td->urb->start_frame, imx21_hc_get_frame(hcd));

        etd_addr = &USB_ETD(etd_num);
        etd_addr[0] = (
            (u32) usb_pipedevice(td->urb->pipe) |
            ((u32) usb_pipeendpoint(td->urb->pipe) << 7) |
            ((u32) (usb_pipeout(td->urb->pipe) ? 
                    IMX_USB_TD_DIR_OUT : IMX_USB_TD_DIR_IN) << 11) |
            ((u32)((td->urb->dev->speed == USB_SPEED_LOW) ? 1  : 0) << 13) |
            ((u32) IMX_FMT_ISO << 14) |
            ((u32) maxpacket << 16));

        etd_addr[1] = td->buf_addr;  /* Only use X buffer */
        etd_addr[2] = (
            (0xf << 28) | /* clear completion code bits */
            ((td->urb->start_frame & 0xFFFF) << 0));
        etd_addr[3] = (
            (0xf << 12) | /* clear completion code bits */
            (td->len << 0));

        etd_mask = 1 << etd_num;

        /*clear etd done status */
        mx2otg_clear_toggling_bit(USBH_ETDDONESTAT, etd_mask);

        /*enable etd done interrupt */
        USBH_ETDDONEEN |= etd_mask;
        
        mx2otg_clear_toggling_bit(USBH_XFILLSTAT, etd_mask);
        mx2otg_clear_toggling_bit(USBH_YFILLSTAT, etd_mask);
        
        USBCTRL |= USBCTRL_HOST1_TXEN_OE;  

        if (td->dma) {
            mx2otg_set_toggling_bit(USB_ETDDMACHANLCLR, etd_mask);
            if (usb_pipeout(td->urb->pipe)) {
                consistent_sync(td->data, td->len, DMA_TO_DEVICE);
                mx2otg_set_toggling_bit(USBH_XFILLSTAT, etd_mask);
                mx2otg_set_toggling_bit(USBH_YFILLSTAT, etd_mask);
            } else {
                consistent_sync(td->data, td->len, DMA_FROM_DEVICE);
            }
            mx2otg_clear_toggling_bit(USBH_XBUFSTAT, etd_mask);
            mx2otg_clear_toggling_bit(USBH_YBUFSTAT, etd_mask);

            USB_ETDSMSA(etd_num) = virt_to_phys(td->data);
            mx2otg_set_toggling_bit(USB_ETDDMAEN, etd_mask);
        } else {
            mx2otg_clear_toggling_bit(USBH_XBUFSTAT, etd_mask);
            mx2otg_clear_toggling_bit(USBH_YBUFSTAT, etd_mask);

            if (usb_pipeout(td->urb->pipe)) {
                memcpy((void *)(USB_XFERMEM + td->buf_addr), td->data, td->len);
                mx2otg_set_toggling_bit(USBH_XFILLSTAT, etd_mask);
                mx2otg_set_toggling_bit(USBH_YFILLSTAT, etd_mask);
            }
        }

        USBH_ETDENSET = etd_mask;
    }
}           


/***************************************************************************
 * Setup a bulk, control or interrupt ETD. Configure DMA if necessary.
 * Trigger the processing of the ETD by the host controller. Return error
 * and not do anything in case there are not enough resources.
 **************************************************************************/
static int schedule_noniso_etd(struct imx21 *imx21, struct urb *urb, int state)
{
    unsigned int    pipe = urb->pipe;
    urb_priv_t      *urb_priv = urb->hcpriv;
    ep_priv_t       *ep_priv = urb_priv->ep->hcpriv;
    int             etd_num = ep_priv->etd_num;
    etd_priv_t      *etd;
    volatile u32    *etd_addr;
    u32             etd_mask;
    int             buf_addr;
    u16             buf_size;
    u8              *src_addr;
    u32             count;
    u8              dir;
    u8              bufround;
    u8              datatoggle;
    u16             maxpacket;
    u8              interval = 0;
    u8              relpolpos = 0;

    if (etd_num < 0) {
        ERR("cannot get a free ETD for non-iso\n");
        return etd_num;
    }

    etd = &imx21->etd[etd_num];
    if (!(maxpacket = usb_maxpacket(urb->dev, pipe, usb_pipeout(pipe))))
        maxpacket = 8;
    
    if (usb_pipecontrol(pipe) && (state != US_CTRL_DATA)) {
        if (state == US_CTRL_SETUP) {
            dir = IMX_USB_TD_DIR_SETUP;
            src_addr = (u8 *) urb->setup_packet;
            bufround = 0;
            count = 8;
            datatoggle = IMX_USB_TD_TOGGLE_DATA0;
        } else {  /* US_CTRL_ACK */
            dir = usb_pipeout(pipe) ? IMX_USB_TD_DIR_IN : IMX_USB_TD_DIR_OUT;
            if (urb->transfer_buffer != NULL) {
                src_addr = (u8 *) urb->transfer_buffer;
            } else {
                src_addr = (u8 *) urb->transfer_dma;
            }
            bufround = 0;
            count = 0;
            datatoggle = IMX_USB_TD_TOGGLE_DATA1;
        }
    } else {
        dir = usb_pipeout(pipe) ? IMX_USB_TD_DIR_OUT : IMX_USB_TD_DIR_IN;
        bufround = (dir == IMX_USB_TD_DIR_IN) ? 1 : 0;
        if (urb->transfer_buffer != NULL) {
            src_addr = (u8 *) urb->transfer_buffer;
        } else {
            src_addr = (u8 *) urb->transfer_dma;
        }
        if (usb_pipebulk(pipe) && (state == US_BULK0))
            count = 0;
        else
            count = urb->transfer_buffer_length;

        if (usb_pipecontrol(pipe)) {
            datatoggle = IMX_USB_TD_TOGGLE_DATA1;
        } else {
            datatoggle = (usb_gettoggle(urb->dev,
                                        usb_pipeendpoint(urb->pipe),
                                        usb_pipeout(urb->pipe))) ?
                IMX_USB_TD_TOGGLE_DATA1 : IMX_USB_TD_TOGGLE_DATA0;
        }
    }
    
    if (count > maxpacket)
        buf_size = maxpacket * 2;
    else
        buf_size = maxpacket;

    /* allocate x and y buffer space at once*/
    buf_addr = alloc_dmem(imx21, buf_size, urb_priv->ep);
    if (buf_addr < 0) {
        ERR("not enough DATA memory for non-iso\n");
        return buf_addr;
    }
    
    etd->buf = NULL;

    if (count > 0) {
        if ((count < maxpacket) && (count < 64)) {
            etd->buf = kmalloc(64, GFP_ATOMIC | GFP_DMA);
        } else {
            etd->buf = kmalloc(count, GFP_ATOMIC | GFP_DMA);
        }

        if (etd->buf == NULL) {
            ERR("not enough memory for non-iso DMA buffer, %d bytes needed\n", count);
            free_dmem(imx21, buf_addr);
            return -ENOMEM;
        }

        if (dir != IMX_USB_TD_DIR_IN) {
            memcpy(etd->buf, src_addr, count);
            consistent_sync(etd->buf, count, DMA_TO_DEVICE);
        } else {
            consistent_sync(etd->buf, count, DMA_FROM_DEVICE);
        }
    }

    // cannot fail from here: commit etd to urb/ep
    urb_priv->active = 1;
    etd->busy = 1;
    etd->urb  = urb;
    etd->ep   = urb_priv->ep;
    etd->len  = count;

    if (usb_pipeint(pipe)) {
        interval = urb->interval;
        relpolpos = (USBH_FRMNUB + 1) & 0xff;
    }
    
    src_addr = etd->buf;
    etd_addr = &USB_ETD(etd_num);   /*set up word 0*/

    /*Normal USB mode is "do not stop on NAK" (bit 30 ==0)*/
    etd_addr[0] = (u32) usb_pipedevice(pipe) |
        ((u32) usb_pipeendpoint(pipe) << 7) |
        ((u32) dir << 11) |
        ((u32)((urb->dev->speed == USB_SPEED_LOW) ? 1  : 0) << 13) |
        ((u32) fmt_urb_to_etd[usb_pipetype(pipe)] << 14) |
        ((u32) maxpacket << 16);
    
    etd_addr[1] = 
        (((u32) buf_addr + (u32) maxpacket) << 16) | (u32) buf_addr;
    
    /*delay interrupt option is disabled (bit 19-21 == 0)*/
    etd_addr[2] = (u32) interval |
        ((u32) relpolpos << 8) |
        ((u32) dir << 16) |
        ((u32) bufround << 18) |
        ((u32) datatoggle << 22) |
        ((u32) 0xf << 28);      /*reset completion code */
    
    /*X or Y buffer size is always == 1 maxpacket */
    etd_addr[3] = ((u32) (maxpacket - 1) << 21) | (u32) count;
    
    etd_mask = 1 << etd_num;

    /*clear etd done status */
    mx2otg_clear_toggling_bit(USBH_ETDDONESTAT, etd_mask);
    
    /*enable etd done interrupt */
    USBH_ETDDONEEN |= etd_mask;
    
    mx2otg_clear_toggling_bit(USBH_XFILLSTAT, etd_mask);
    mx2otg_clear_toggling_bit(USBH_YFILLSTAT, etd_mask);
    
    USBCTRL |= USBCTRL_HOST1_TXEN_OE;    
    
    if (count == 0) {
        if (dir != IMX_USB_TD_DIR_IN) {
            /*need to set it even if count == 0 */
            mx2otg_set_toggling_bit(USBH_XFILLSTAT, etd_mask);
            mx2otg_set_toggling_bit(USBH_YFILLSTAT, etd_mask);
        }
        USB_ETDSMSA(etd_num) = 0;
    } else {
        mx2otg_set_toggling_bit(USB_ETDDMACHANLCLR, etd_mask);
        
        mx2otg_clear_toggling_bit(USBH_XBUFSTAT, etd_mask);
        mx2otg_clear_toggling_bit(USBH_YBUFSTAT, etd_mask);
      
        USB_ETDSMSA(etd_num) = virt_to_phys(src_addr);
        mx2otg_set_toggling_bit(USB_ETDDMAEN, etd_mask);
    }
    
    USBH_ETDENSET = etd_mask;
    return 0;
}


/***************************************************************************
 * If the ETD is done, parse all status data. Copy received data with
 * memcpy if necessary. Report number of bytes actually transferred and
 * completion code
 **************************************************************************/
static void
noniso_etd_done(struct usb_hcd *hcd, struct urb *urb, 
                int etd_num, struct pt_regs *regs)
{
    struct imx21    *imx21 = hcd_to_imx21(hcd);
    etd_priv_t      *etd = &imx21->etd[etd_num];
    volatile u32    *etd_addr = &USB_ETD(etd_num);
    u32             etd_mask = 1 << etd_num;
    urb_priv_t      *urb_priv = urb->hcpriv;
    int             dir;
    u16             xbufaddr;
    int             cc;
    u32             bytes_xfrd;
    int             urb_state;
    int             etd_done;

    USBH_ETDENCLR = etd_mask;
    mx2otg_clear_toggling_bit(USBH_ETDDONESTAT, etd_mask);
    mx2otg_clear_toggling_bit(USB_ETDDMAEN, etd_mask);
    
    dir        = (etd_addr[0] >> 11) & 0x3;
    xbufaddr   = etd_addr[1] & 0xffff;
    cc         = (etd_addr[2] >> 28) & 0xf;
    bytes_xfrd = etd_addr[3] & 0x1fffff;  // bytes to transfer
    bytes_xfrd = etd->len - bytes_xfrd;   // # bytes transferred

    /*save toggle carry */
    usb_settoggle(urb->dev, usb_pipeendpoint(urb->pipe),
                  usb_pipeout(urb->pipe), (etd_addr[0] >> 28) & 0x1);
    
    if (dir == IMX_USB_TD_DIR_IN) {
        if (etd->buf) {
            if (urb->transfer_buffer != NULL) {
                memcpy((char *) urb->transfer_buffer, etd->buf, bytes_xfrd);
            } else {
                memcpy((char *) urb->transfer_dma, etd->buf, bytes_xfrd);
            }
        }
        mx2otg_clear_toggling_bit(USBH_XFILLSTAT, etd_mask);
        mx2otg_clear_toggling_bit(USBH_YFILLSTAT, etd_mask);
    }
    
    if (etd->buf) {
        kfree(etd->buf);
        etd->buf = NULL;
    }
    
    free_dmem(imx21, xbufaddr);
    etd->urb = NULL;

    urb->error_count = 0;
    if (!(urb->transfer_flags & URB_SHORT_NOT_OK) && (cc == TD_DATAUNDERRUN))
        cc = TD_CC_NOERROR;
    if (cc != 0) {
        DBG("cc is 0x%x\n", cc);
    }

    etd_done = (cc_to_error[cc] != 0);      /*stop if error */
    switch (usb_pipetype(urb->pipe)) {
    case PIPE_CONTROL:
        urb_state = urb_priv->state;
        if (urb_state == US_CTRL_SETUP) {
            if (urb->transfer_buffer_length > 0) {
                urb_state = US_CTRL_DATA;
            } else {
                urb_state = US_CTRL_ACK;
            }
        } else {
            if (urb_state == US_CTRL_DATA) {
                urb->actual_length += bytes_xfrd;
                urb_state = US_CTRL_ACK;
            } else {
                etd_done = 1;               /*US_CTRL_ACK */
            }
        }

        if (!etd_done) {
            urb_priv->state = urb_state;
            if (schedule_noniso_etd(imx21, urb, urb_state) < 0) {
                /*requeue in case of res. shortage */
//              list_add(&urb_priv->list, &imx21->ctrl_list);
            }
        }
        break;

    case PIPE_BULK:
        urb->actual_length += bytes_xfrd;
        if ((urb_priv->state == US_BULK)
            && (urb->transfer_flags & URB_ZERO_PACKET)
            && urb->transfer_buffer_length > 0
            && ((urb->transfer_buffer_length %
                 usb_maxpacket(urb->dev, urb->pipe, 
                               usb_pipeout(urb->pipe))) == 0)) {
            /*need a 0-packet */
            urb_priv->state = US_BULK0;

            /*requeue in case of res. shortage */
            if (schedule_noniso_etd(imx21, urb, US_BULK0) < 0) {
//              list_add(&urb_priv->list, &imx21->bulk_list);
            }
        } else {
            etd_done = 1;
        }
        break;

    case PIPE_INTERRUPT:
        urb->actual_length += bytes_xfrd;
        etd_done = 1;
        break;
    }

    if (etd_done) {
        struct usb_host_endpoint *ep = urb_priv->ep;
        ep_priv_t *ep_priv = ep->hcpriv;

        urb->status = cc_to_error[cc];
        kfree(urb_priv);
        urb->hcpriv = NULL;
        usb_hcd_giveback_urb(hcd, urb, regs);

        if (list_empty(&ep->urb_list)) {
            free_etd(imx21, ep_priv->etd_num);
            ep_priv->etd_num = -1;
        } else {
            /* Process next URB */
            urb = list_entry(ep->urb_list.next, struct urb, urb_list);
            urb_priv = urb->hcpriv;
            if (urb_priv != NULL)
                schedule_noniso_etd(imx21, urb, urb_priv->state);
            else
            {
                free_etd(imx21, ep_priv->etd_num);
                ep_priv->etd_num = -1;
            }
        }
    }
}


static void 
iso_etd_done(struct usb_hcd *hcd, struct urb *urb, 
             int etd_num, struct pt_regs *regs)
{
    struct imx21                *imx21 = hcd_to_imx21(hcd);
    int                         etd_mask = 1 << etd_num;
    etd_priv_t                  *etd = imx21->etd + etd_num;
    volatile u32                *etd_addr = &USB_ETD(etd_num);
    td_t                        *td = etd->td;
    struct usb_host_endpoint    *ep = etd->ep;
    urb_priv_t                  *urb_priv = urb->hcpriv;
    int                         iso_index = td->iso_index;
    unsigned int                pipe = urb->pipe;
    int                         dir_in = usb_pipein(pipe);
    int                         cc;
    int                         bytes_xfrd;
    unsigned long               flags;

    spin_lock_irqsave(&imx21->lock, flags);
    USBH_ETDENCLR = etd_mask;
    mx2otg_clear_toggling_bit(USBH_ETDDONESTAT, etd_mask);
    mx2otg_clear_toggling_bit(USB_ETDDMAEN, etd_mask);
    
    cc = (etd_addr[3] >> 12) & 0xf;
    bytes_xfrd = etd_addr[3] & 0x3ff;

    /* Input doesn't always fill the buffer, don't generate an error
     * when this happens.
     */
    if (dir_in && (cc == TD_DATAUNDERRUN)) {
        cc = TD_CC_NOERROR;
    }

    if (cc || bytes_xfrd < 0x100) {
        DBG("frame=%x, cc=%x, bytes_xfrd=%x\n", 
            etd_addr[2] & 0xffff, cc, bytes_xfrd);
    }

    if (dir_in) {
        if (!td->dma) {
            memcpy((char *)td->data,
                   (void *)(USB_XFERMEM + td->buf_addr), bytes_xfrd);
        }
        mx2otg_clear_toggling_bit(USBH_XFILLSTAT, etd_mask);
        mx2otg_clear_toggling_bit(USBH_YFILLSTAT, etd_mask);
    }

    urb->actual_length += bytes_xfrd;
    urb->iso_frame_desc[iso_index].actual_length = bytes_xfrd;
    urb->iso_frame_desc[iso_index].status = cc_to_error[cc];

    free_dmem(imx21, td->buf_addr);
    etd->busy = 0;
    etd->td = NULL;
    etd->buf = NULL;
    etd->urb = NULL;
    etd->ep = NULL;

    //DBG("urb=%lx, ep=%lx, td=%lx, buf=%x, len=%d, etd=%d, bytes_xfrd=%d, cc=%d, last=%d\n",
    //    urb, ep, td, td->buf_addr, td->len, etd_num, bytes_xfrd, cc, td->last);

    if (td->last) {
        urb->status = cc_to_error[cc];
        kfree(urb_priv);
        urb->hcpriv = NULL;
        usb_hcd_giveback_urb(hcd, urb, regs);
    }
    kfree(td);

    schedule_iso_etds(hcd, ep);
    spin_unlock_irqrestore(&imx21->lock, flags);
}


/***************************************************************************
 * Part of the Host Controller interrupt handler.
 * Scan through all ETD done states. Submit another URB transaction if
 * necessary, or report URB completion.
 **************************************************************************/
static void
process_etd_done(struct usb_hcd *hcd, struct imx21 *imx21, struct pt_regs *regs)
{
    int etd_num;
    
    for (etd_num = 0; etd_num < USB_NUM_ETD; etd_num++) {
        u32 etd_mask = 1 << etd_num;
        struct urb *urb;

        if (!(USBH_ETDDONESTAT & etd_mask))
            continue;

        if (imx21->etd[etd_num].ep == NULL || imx21->etd[etd_num].urb == NULL) {
            USBH_ETDENCLR = etd_mask;
            USBH_ETDDONEEN &= ~etd_mask;
            mx2otg_clear_toggling_bit(USBH_ETDDONESTAT, etd_mask);
            mx2otg_clear_toggling_bit(USB_ETDDMAEN, etd_mask);
            continue;
        }
        urb = imx21->etd[etd_num].urb;

        if (usb_pipeisoc(urb->pipe))
            iso_etd_done(hcd, urb, etd_num, regs);
        else
            noniso_etd_done(hcd, urb, etd_num, regs);
    }
}


static irqreturn_t imx21_irq(struct usb_hcd *hcd, struct pt_regs *regs)
{
    u32 ints = USBH_SYSISR;
    if (ints & USBH_SYSISR_SOFINT) {
        process_etd_done(hcd, hcd_to_imx21(hcd), regs);
    }
    USBH_SYSISR = ints;
    return IRQ_HANDLED;
}


static void imx21_hc_stop(struct usb_hcd *hcd)
{
    DBG("NOT IMPLEMENTED YET\n");
}


static int imx21_hc_reset(struct usb_hcd *hcd)
{
    DBG("NOT IMPLEMENTED YET\n");
    return 0;
}


static int __devinit imx21_hc_start(struct usb_hcd *hcd)
{
    struct usb_device *udev;
    
    udev = usb_alloc_dev(NULL, &hcd->self, 0);
    if (udev == NULL) {
        ERR("usb_alloc_dev() returned NULL!\n");
        return -ENOMEM;
    }
    
    udev->speed = USB_SPEED_FULL;
    hcd->state = HC_STATE_RUNNING;
    
    /* Enable host controller interrupts */
    USBOTG_CINT_STEN |= USBOTG_HCINT;
    
    return 0;
}


static int imx21_hc_urb_enqueue_isoc(struct usb_hcd *hcd,
                                     struct usb_host_endpoint *ep,
                                     struct urb *urb,
                                     gfp_t mem_flags)
{
    struct imx21 *imx21 = hcd_to_imx21(hcd);
    urb_priv_t *urb_priv;
    unsigned long flags;
    ep_priv_t *ep_priv;
    td_t *td = NULL;
    int i;

    urb_priv = kzalloc(sizeof(urb_priv_t), mem_flags);
    if (urb_priv == NULL) {
        ERR("Out of memory!\n");
        return -ENOMEM;
    }

    if (ep->hcpriv == NULL) {
        ep_priv = kzalloc(sizeof(ep_priv_t), mem_flags);
        DBG("ep=%p, ep->hcpriv=%p, new ep_priv=%p\n", ep, ep->hcpriv, ep_priv);
        if (ep_priv == NULL) {
            ERR("Out of memory!\n");
            kfree(urb_priv);
            return -ENOMEM;
        }

        /* Allocate the ETDs */
        spin_lock_irqsave(&imx21->lock, flags);
        for (i = 0; i < NUM_ISO_ETDS; i++) {
            ep_priv->etd[i] = alloc_etd(imx21);
            if (ep_priv->etd[i] < 0) {
                int j;
                ERR("Couldn't allocate etd\n");
                for (j = 0; j < i; j++) {
                    free_etd(imx21, ep_priv->etd[j]);
                }
                kfree(urb_priv);
                kfree(ep_priv);
                spin_unlock_irqrestore(&imx21->lock, flags);
                return -ENOMEM;
            } else {
                imx21->etd[ep_priv->etd[i]].busy = 0;
                imx21->etd[ep_priv->etd[i]].ep = ep;
                imx21->etd[ep_priv->etd[i]].urb = NULL;
                imx21->etd[ep_priv->etd[i]].td = NULL;
                DBG("ep=%p, ep_priv=%p, urb=%p, new etd ep_priv->etd[%d]=%d\n", 
                    ep, ep_priv, urb, i, ep_priv->etd[i]); 
            }
        }
        spin_unlock_irqrestore(&imx21->lock, flags);

        ep_priv->etd_num = -1;
        //INIT_LIST_HEAD(&ep_priv->list);
        INIT_LIST_HEAD(&ep_priv->td_list);
        ep_priv->hep = ep;
        ep->hcpriv = ep_priv;
        ep_priv->interval = urb->interval;
    } else {
        ep_priv = ep->hcpriv;
        if (ep_priv->interval != urb->interval) {
            ERR("mismatched interval - was %d, now %d\n",
                   ep_priv->interval, urb->interval);
        }
    }

    spin_lock_irqsave(&imx21->lock, flags);
    urb->status        = -EINPROGRESS;
    urb->actual_length = 0;
    urb->error_count   = 0;
    urb->hcpriv        = urb_priv;
    urb_priv->ep       = ep;

    /* set up transfers */
    for (i = 0; i < urb->number_of_packets; i++) {
        td = kzalloc(sizeof(td_t), mem_flags);
        if (td == NULL) {
            // FIXME
            spin_unlock_irqrestore(&imx21->lock, flags);
            ERR("Out of memory!\n");
            return -ENOMEM;
        }

        td->ep  = ep;
        td->urb = urb;
        td->len = urb->iso_frame_desc[i].length;
        td->iso_index = i;

        td->dma = urb->transfer_flags & URB_NO_TRANSFER_DMA_MAP;
        if (td->dma) {
            td->data = (void *)((unsigned long)urb->transfer_dma + 
                                urb->iso_frame_desc[i].offset);
        } else {
            td->data = (void *)((unsigned long)urb->transfer_buffer + 
                                urb->iso_frame_desc[i].offset);
        }

        list_add_tail(&td->list, &ep_priv->td_list);
        //DBG("urb=%lx, ep=%lx, td=%lx, len=%x, iso_index=%d, flags=%x, interval=%d, offset=%x\n", 
        //    urb, ep, td, td->len, td->iso_index, urb->transfer_flags, urb->interval, urb->iso_frame_desc[i].offset);
    }
    td->last = 1;

    schedule_iso_etds(hcd, ep);

    /* Enable ETD interrupts */
    USBH_SYSIEN |= USBH_SYSIEN_DONEINT | USBH_SYSIEN_SOFINT;
    spin_unlock_irqrestore(&imx21->lock, flags);
    return 0;
}


static int imx21_hc_urb_enqueue(struct usb_hcd *hcd,
                                struct usb_host_endpoint *ep,
                                struct urb *urb,
                                gfp_t mem_flags)
{
    struct imx21 *imx21 = hcd_to_imx21(hcd);
    urb_priv_t *urb_priv;
    unsigned long flags;
    ep_priv_t *ep_priv;

    if (usb_pipeisoc(urb->pipe)) {
        return imx21_hc_urb_enqueue_isoc(hcd, ep, urb, mem_flags);
    }
    
    urb_priv = kzalloc(sizeof(urb_priv_t), mem_flags);
    if (urb_priv == NULL) {
        ERR("Out of memory!\n");
        return -ENOMEM;
    }

    ep_priv = ep->hcpriv;
    if (ep_priv == NULL) {
        int i;

        ep_priv = kzalloc(sizeof(ep_priv_t), mem_flags);
        if (ep_priv == NULL) {
            ERR("Out of memory!\n");
            kfree(urb_priv);
            return -ENOMEM;
        }

        for (i = 0; i < NUM_ISO_ETDS; ++i) {
            ep_priv->etd[i] = -1;
        }

        ep_priv->etd_num = -1;
        //INIT_LIST_HEAD(&ep_priv->list);
        ep_priv->hep = ep;
        ep->hcpriv = ep_priv;
    }

    spin_lock_irqsave(&imx21->lock, flags);
    urb->status = -EINPROGRESS;
    urb->actual_length = 0;
    urb->error_count = 0;
    urb->hcpriv = urb_priv;
    urb_priv->ep = ep;
    switch (usb_pipetype(urb->pipe)) {
    case PIPE_CONTROL:
        urb_priv->state = US_CTRL_SETUP;
        break;
    case PIPE_BULK:
        urb_priv->state = US_BULK;
        break;
    }

    if (ep_priv->etd_num < 0) {
        ep_priv->etd_num = alloc_etd(imx21);
        if (ep_priv->etd_num < 0) {
            ERR("Unable to enqueue URB %p\n", urb);
//FIXME: need to queue up these URBs/EPs
            kfree(urb_priv);
            spin_unlock_irqrestore(&imx21->lock, flags);
            return -ENODEV;
        }
    }

    if (imx21->etd[ep_priv->etd_num].busy == 0) {
        schedule_noniso_etd(imx21, urb, urb_priv->state);
    }

    /* Enable ETD interrupts */
    USBH_SYSIEN |= USBH_SYSIEN_DONEINT | USBH_SYSIEN_SOFINT;
    spin_unlock_irqrestore(&imx21->lock, flags);
    return 0;
}


static int imx21_hc_urb_dequeue(struct usb_hcd *hcd, struct urb *urb)
{
    struct imx21 *imx21 = hcd_to_imx21(hcd);
    unsigned long flags;
    struct usb_host_endpoint *ep;
    ep_priv_t *ep_priv;
    urb_priv_t *urb_priv = urb->hcpriv;
    td_t *td;
    td_t *tmp;

    DBG("urb=%p iso=%d\n", urb, usb_pipeisoc(urb->pipe));
    spin_lock_irqsave(&imx21->lock, flags);

    if (urb_priv == NULL) {
        DBG("urb=%p, urb_priv == NULL\n", urb);
        goto fail;
    }

    ep = urb_priv->ep;
    if (ep == NULL) {
        DBG("urb=%p, ep == NULL\n", urb);
        ERR("urb=%p, ep is NULL!\n", urb);
        goto fail;
    }

    ep_priv = ep->hcpriv;
    if (ep_priv == NULL) {
        DBG("urb=%p, ep_priv == NULL\n", urb);
        ERR("urb=%p, ep_priv is NULL!\n", urb);
        goto fail;
    }

    if (usb_pipeisoc(urb->pipe)) {
        if (urb_priv->active) {
            int i;

            for (i = 0; i < NUM_ISO_ETDS; i++) {
                int etd_num = ep_priv->etd[i];
                if (etd_num != -1 && imx21->etd[etd_num].urb == urb) {
                    u32 etd_mask = 1 << etd_num;
                    etd_priv_t *etd = imx21->etd + etd_num;

                    DBG("ep=%p, ep_priv=%d, urb=%p, releasing etd=%d\n", 
                        ep, ep_priv, urb, etd_num);

                    USBH_ETDENCLR = etd_mask;
                    USBH_ETDDONEEN &= ~etd_mask;
                    mx2otg_clear_toggling_bit(USBH_ETDDONESTAT, etd_mask);
                    mx2otg_clear_toggling_bit(USB_ETDDMAEN, etd_mask);
                    
                    td = etd->td;
                    etd->td = NULL;
                    etd->buf = NULL;
                    etd->urb = NULL;
                    etd->ep = NULL;
                    etd->busy = 0;
                    
                    free_dmem(imx21, td->buf_addr);
                    kfree(td);
                }
            }
        } else {
            list_for_each_entry_safe(td, tmp, &ep_priv->td_list, list) {
                if (td->urb == urb) {
                    list_del(&td->list);
                    kfree(td);
                }
            }
        }
        schedule_iso_etds(hcd,ep);
    } else if (urb_priv->active) {
        int etd_num = ep_priv->etd_num;
        if (etd_num != -1) {
            volatile u32 *etd_addr = &USB_ETD(etd_num);
            u32 etd_mask = 1 << etd_num;
            u16 xbufaddr;

            USBH_ETDENCLR = etd_mask;
            USBH_ETDDONEEN &= ~etd_mask;

            xbufaddr = etd_addr[1] & 0xffff;
            free_dmem(imx21, xbufaddr);
            imx21->etd[etd_num].urb = NULL;
        }
    }

    kfree(urb_priv);
    urb->hcpriv = NULL;
    usb_hcd_giveback_urb(hcd, urb, NULL);

    if ((!usb_pipeisoc(urb->pipe)) && list_empty(&ep->urb_list)) {
        if (ep_priv->etd_num != -1) {
            free_etd(imx21, ep_priv->etd_num);
            ep_priv->etd_num = -1;
        }
    }
               
    spin_unlock_irqrestore(&imx21->lock, flags);
    return 0;

 fail:
    ERR("Invalid URB=%p\n", urb);
    spin_unlock_irqrestore(&imx21->lock, flags);
    return -EINVAL;
}


static void imx21_hc_endpoint_disable(struct usb_hcd *hcd, 
                                      struct usb_host_endpoint *ep)
{
    struct imx21 *imx21 = hcd_to_imx21(hcd);
    unsigned long flags;
    ep_priv_t *ep_priv;
    int i;

    if (ep == NULL) {
        ERR("ep is NULL!\n");
        return;
    }

    spin_lock_irqsave(&imx21->lock, flags);
    ep_priv = ep->hcpriv;
    DBG("ep=%p, ep->hcpriv=%p\n", ep, ep_priv);

    if (!list_empty(&ep->urb_list)) {
        ERR("ep's URB list is not empty\n");
    }

    if (ep_priv != NULL) {
        for (i = 0; i < NUM_ISO_ETDS; i++) {
            int etd_num = ep_priv->etd[i];
            if (etd_num >= 0) {
                free_etd(imx21, etd_num);
            }
        }
        if (ep_priv->etd_num != -1) {
            free_etd(imx21, ep_priv->etd_num);
        }
        kfree(ep_priv);
        ep->hcpriv = NULL;
    }

    for (i = 0; i < USB_NUM_ETD; i++) {
        if (imx21->etd[i].alloc && imx21->etd[i].ep == ep) {
            ERR("Active imx21->etd[%d] belongs to disabled ep=%p!\n", i, ep);
            free_etd(imx21, i);
        }
    }
    spin_unlock_irqrestore(&imx21->lock, flags);

    free_epdmem(imx21, ep);
}


/*
 * Checks the root hub's status for changes.
 *
 * If a port has changed, then the appropriate bit in buf is set.
 * The function returns the number of ports that have changed.
 *
 * For the IMX21, there is only one port. 
 */
static int imx21_hc_hub_status_data(struct usb_hcd *hcd, char *buf)
{
    struct imx21 *imx21 = hcd_to_imx21(hcd);
    int ports;
    int changed = 0;
    int i;
    unsigned long flags;
    
    spin_lock_irqsave(&imx21->lock, flags);
    ports = (USBH_ROOTHUBA & USBH_ROOTHUBA_NDNSTMPRT_MASK);
    if (ports > 7) {
        ERR("ports %d > 7\n", ports);
    }
    for (i = 0; i < ports; i++) {
        if (USBH_PORTSTAT(i) & (USBH_PORTSTAT_CONNECTSC |
                                USBH_PORTSTAT_PRTENBLSC |
                                USBH_PORTSTAT_PRTSTATSC |
                                USBH_PORTSTAT_OVRCURIC |
                                USBH_PORTSTAT_PRTRSTSC)) {
            changed = 1;
            buf[0] |= 1 << (i + 1);
        }
    }
    spin_unlock_irqrestore(&imx21->lock, flags);
    
    if (changed)
        printk(KERN_DEBUG "Hub status changed\n");
    return changed;
}


static int imx21_hc_hub_control(struct usb_hcd *hcd,
                                u16             typeReq,
                                u16             wValue,
                                u16             wIndex,
                                char            *buf,
                                u16             wLength)
{
    int rc = -ENODEV;
    
    switch(typeReq) {
    case ClearHubFeature:
        DBG("  ClearHubFeature\n");
        switch(wValue) {
        case C_HUB_OVER_CURRENT:
            DBG("    C_HUB_OVER_CURRENT\n");
            rc = 0;
            break;
        case C_HUB_LOCAL_POWER:
            DBG("    C_HUB_LOCAL_POWER\n");
            rc = 0;
            break;
        default:
            DBG("    default\n");
            break;
        }
        break;
        
    case ClearPortFeature:
        DBG("  ClearPortFeature\n");
// Need to check wIndex boundary
        switch (wValue) {
        case USB_PORT_FEAT_ENABLE:
            DBG("    USB_PORT_FEAT_ENABLE\n");
            USBH_PORTSTAT(wIndex - 1) = USBH_PORTSTAT_CURCONST;
            rc = 0;
            break;
        case USB_PORT_FEAT_SUSPEND:
            DBG("    USB_PORT_FEAT_SUSPEND\n");
            USBH_PORTSTAT(wIndex - 1) = USBH_PORTSTAT_PRTOVRCURI;
            rc = 0;
            break;
        case USB_PORT_FEAT_POWER:
            DBG("    USB_PORT_FEAT_POWER\n");
            USBH_PORTSTAT(wIndex - 1) = USBH_PORTSTAT_LSDEVCON;
            rc = 0;
            break;
        case USB_PORT_FEAT_C_ENABLE:
            DBG("    USB_PORT_FEAT_C_ENABLE\n");
            USBH_PORTSTAT(wIndex - 1) = USBH_PORTSTAT_PRTENBLSC;
            rc = 0;
            break;
        case USB_PORT_FEAT_C_SUSPEND:
            DBG("    USB_PORT_FEAT_C_SUSPEND\n");
            USBH_PORTSTAT(wIndex - 1) = USBH_PORTSTAT_PRTSTATSC;
            rc = 0;
            break;
        case USB_PORT_FEAT_C_CONNECTION:
            DBG("    USB_PORT_FEAT_C_CONNECTION\n");
            USBH_PORTSTAT(wIndex - 1) = USBH_PORTSTAT_CONNECTSC;
            rc = 0;
            break;
        case USB_PORT_FEAT_C_OVER_CURRENT:
            DBG("    USB_PORT_FEAT_C_OVER_CURRENT\n");
            USBH_PORTSTAT(wIndex - 1) = USBH_PORTSTAT_OVRCURIC;
            rc = 0;
            break;
        case USB_PORT_FEAT_C_RESET:
            DBG("    USB_PORT_FEAT_C_RESET\n");
            USBH_PORTSTAT(wIndex - 1) = USBH_PORTSTAT_PRTRSTSC;
            rc = 0;
            break;
        default:
            DBG("    default\n");
            break;
        }
        
        break;
        
    case GetHubDescriptor:
        DBG("  GetHubDescriptor\n");
        rc = get_hub_descriptor(hcd, (void *)buf);
        break;
        
    case GetHubStatus:
        DBG("  GetHubStatus\n");
        *(__le32*)buf = 0;
        rc = 0;
        break;
        
    case GetPortStatus:
        DBG("  GetPortStatus: port: %d, 0x%x\n", 
            wIndex, USBH_PORTSTAT(wIndex - 1));
// Need to check wIndex boundary
        *(__le32*)buf = USBH_PORTSTAT(wIndex - 1);
        rc = 0;
        break;
        
    case SetHubFeature:
        DBG("  SetHubFeature\n");
        switch (wValue) {
        case C_HUB_OVER_CURRENT:
            DBG("    C_HUB_OVER_CURRENT\n");
            break;
            
        case C_HUB_LOCAL_POWER:
            DBG("    C_HUB_LOCAL_POWER\n");
            break;
        default:
            DBG("    default\n");
            break;
        }
        
        break;
        
    case SetPortFeature:
        DBG("  SetPortFeature\n");
        switch (wValue) {
// Need to check wIndex boundary
        case USB_PORT_FEAT_SUSPEND:
            DBG("    USB_PORT_FEAT_SUSPEND\n");
            USBH_PORTSTAT(wIndex - 1) = USBH_PORTSTAT_PRTSUSPST;
            rc = 0;
            break;
        case USB_PORT_FEAT_POWER:
            DBG("    USB_PORT_FEAT_POWER\n");
            USBH_PORTSTAT(wIndex - 1) = USBH_PORTSTAT_PRTPWRST;
            rc = 0;
            break;
        case USB_PORT_FEAT_RESET:
            DBG("    USB_PORT_FEAT_RESET\n");
            dump_regs();
            USBH_PORTSTAT(wIndex - 1) = USBH_PORTSTAT_PRTRSTST;
            rc = 0;
            break;
        default:
            DBG("    default\n");
            break;
        }
        break;
        
    default:
        DBG("  default\n");
        break;
    }
    
    return -rc;
}


static struct hc_driver imx21_hc_driver = {
    .description = hcd_name,
    .product_desc = "IMX21 USB Host Controller",
    .hcd_priv_size = sizeof(struct imx21),
    
    .flags = HCD_USB11,   /* USB 1.1 */
    .irq = imx21_irq,
    
    .reset = imx21_hc_reset,
    .start = imx21_hc_start,
    .stop  = imx21_hc_stop,
    
    /* I/O requests */
    .urb_enqueue = imx21_hc_urb_enqueue,
    .urb_dequeue = imx21_hc_urb_dequeue,
    .endpoint_disable = imx21_hc_endpoint_disable,
    
    /* scheduling support */
    .get_frame_number = imx21_hc_get_frame,
    
    /* Root hub support */
    .hub_status_data = imx21_hc_hub_status_data,
    .hub_control = imx21_hc_hub_control,
/* For PM
   .hub_suspend = ;
   .hub_resume = ;
*/
};


static int imx21_remove(struct platform_device *pdev)
{
    #ifdef GRH_DBUG_PRINTK
    dbug_proc_exit( );
    #endif
    return 0;
}


static int imx21_probe(struct platform_device *pdev)
{
    struct usb_hcd *hcd;
    struct imx21 *imx21;
    int rc;
    int i;
    
    #ifdef GRH_DBUG_PRINTK
    dbug_proc_init( );
    #endif
    
    /* Clear the ETDs */
    for (i = 0; i < USB_NUM_ETD; i++) {
        volatile u32 *etd;
        etd = &USB_ETD(i);
        etd[0] = 0;
        etd[1] = 0;
        etd[2] = 0;
        etd[3] = 0;
    }
    
    /* Take the HC out of reset */
    USBH_HOST_CTRL = (USBH_HOST_CTRL_HCUSBSTE_OPERATIONAL | 
                      USBH_HOST_CTRL_CTLBLKSR_1);
    
    USBH_PORTSTAT(0) = USBH_PORTSTAT_PRTPWRST;
    USBH_PORTSTAT(1) = USBH_PORTSTAT_PRTPWRST;
    USBH_PORTSTAT(2) = USBH_PORTSTAT_PRTPWRST;
    
    USBH_PORTSTAT(0) = USBH_PORTSTAT_PRTENABST;
    USBH_PORTSTAT(1) = USBH_PORTSTAT_PRTENABST;
    USBH_PORTSTAT(2) = USBH_PORTSTAT_PRTENABST;
    
    hcd = usb_create_hcd(&imx21_hc_driver, &pdev->dev, pdev->dev.bus_id);
    if (hcd == NULL) {
        ERR("No platform_data available (bus_id=%s)\n", pdev->dev.bus_id);
        return -ENOMEM;
    }
    
    imx21 = hcd_to_imx21(hcd);
    memset(imx21, 0, sizeof(struct imx21));
    imx21->board = pdev->dev.platform_data;
    spin_lock_init(&imx21->lock);

    rc = usb_add_hcd(hcd, INT_USBHOST, SA_INTERRUPT);
    if (rc != 0) {
        ERR("usb_add_hcd() returned %d\n", rc);
        usb_remove_hcd(hcd);
        return rc;
    }
    
    return 0;
}


static struct platform_driver imx21_hcd_driver = {
    .driver = {
        .name = (char *)hcd_name,
    },
    .probe  = imx21_probe,
    .remove = imx21_remove,
    .suspend = NULL,
    .resume = NULL,
};


static int __init imx21_hcd_init(void)
{
    return platform_driver_register(&imx21_hcd_driver);
}


static void __exit imx21_hcd_cleanup(void)
{
    platform_driver_unregister(&imx21_hcd_driver);
}


module_init(imx21_hcd_init);
module_exit(imx21_hcd_cleanup);
