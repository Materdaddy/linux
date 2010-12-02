/*
 * Macros and prototypes for i.MX21
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
#ifndef __LINUX_IMX21_HCD_H__
#define __LINUX_IMX21_HCD_H__

#define NUM_ISO_ETDS 2

#define IMX_USB_TD_DIR_SETUP        0
#define IMX_USB_TD_DIR_OUT          1
#define IMX_USB_TD_DIR_IN           2
#define IMX_USB_TD_FORMAT_CONTROL   0
#define IMX_USB_TD_FORMAT_ISO       1
#define IMX_USB_TD_FORMAT_BULK      2
#define IMX_USB_TD_FORMAT_INT       3
#define IMX_USB_TD_TOGGLE_CARRY     0
#define IMX_USB_TD_TOGGLE_DATA0     2
#define IMX_USB_TD_TOGGLE_DATA1     3

/* control transfer states */
#define US_CTRL_SETUP   2
#define US_CTRL_DATA    1
#define US_CTRL_ACK     0

/* bulk transfer main state and 0-length packet */
#define US_BULK         1
#define US_BULK0        0


/*ETD format description*/
#define IMX_FMT_CTRL   0x0
#define IMX_FMT_ISO    0x1
#define IMX_FMT_BULK   0x2
#define IMX_FMT_INT    0x3

static char fmt_urb_to_etd[4] = {
/*PIPE_ISOCHRONOUS*/ IMX_FMT_ISO,
/*PIPE_INTERRUPT*/ IMX_FMT_INT,
/*PIPE_CONTROL*/ IMX_FMT_CTRL,
/*PIPE_BULK*/ IMX_FMT_BULK
};

/* condition (error) CC codes and mapping (OHCI like) */

#define TD_CC_NOERROR           0x00
#define TD_CC_CRC               0x01
#define TD_CC_BITSTUFFING       0x02
#define TD_CC_DATATOGGLEM       0x03
#define TD_CC_STALL             0x04
#define TD_DEVNOTRESP           0x05
#define TD_PIDCHECKFAIL         0x06
/*#define TD_UNEXPECTEDPID      0x07 - reserved, not active on MX2*/
#define TD_DATAOVERRUN          0x08
#define TD_DATAUNDERRUN         0x09
#define TD_BUFFEROVERRUN        0x0C
#define TD_BUFFERUNDERRUN       0x0D
#define TD_NOTACCESSED          0x0F

#define mx2otg_clear_toggling_bit(reg,mask) do { \
if (reg & mask) reg = mask;\
} while (0)

#define mx2otg_set_toggling_bit(reg,mask) do { \
if (!(reg & mask)) reg = mask;\
} while (0)

typedef struct {
    struct list_head list;
    struct urb *urb;
    void *data;
    int  len;
    struct usb_host_endpoint *ep;
    unsigned long buf_addr;
    int last;
    int frame;
    int dma;
    int iso_index;
} td_t;

typedef struct urb_priv_s {
    //struct list_head list;
    struct urb *urb;
    struct usb_host_endpoint *ep;
    int active;
    int state;
    int iso_index;
} urb_priv_t;

typedef struct ep_priv_s {
    struct usb_host_endpoint *hep;
    //struct list_head list;
    int etd_num;

    struct list_head td_list;
    int interval;
    ushort last_frame;
    int etd[NUM_ISO_ETDS];
} ep_priv_t;    
    
static const int cc_to_error [16] = {
    /* No  Error  */               0,
    /* CRC Error  */               -EILSEQ,
    /* Bit Stuff  */               -EPROTO,
    /* Data Togg  */               -EILSEQ,
    /* Stall      */               -EPIPE,
    /* DevNotResp */               -ETIMEDOUT,
    /* PIDCheck   */               -EPROTO,
    /* UnExpPID   */               -EPROTO,
    /* DataOver   */               -EOVERFLOW,
    /* DataUnder  */               -EREMOTEIO,
    /* (for hw)   */               -EIO,
    /* (for hw)   */               -EIO,
    /* BufferOver */               -ECOMM,
    /* BuffUnder  */               -ENOSR,
    /* (for HCD)  */               -EALREADY,
    /* (for HCD)  */               -EALREADY
};






typedef struct imx21_dmem_area_s{
    int offset;
    int size;
    struct usb_host_endpoint *ep;

    struct imx21_dmem_area_s *next;
} imx21_dmem_area_t;

typedef struct etd_priv_s {
    int alloc;
    struct usb_host_endpoint *ep;
    struct urb *urb;
    int len;
    char *buf;
    int busy;

    td_t *td;
} etd_priv_t;
    

struct imx21 {
    spinlock_t lock;
    
    imx21_dmem_area_t *dmem_list;

    etd_priv_t etd[USB_NUM_ETD];
    int active_urbs;
    
    struct imx21_usb_platform_data *board;
};

static inline struct imx21 *hcd_to_imx21(struct usb_hcd *hcd)
{
    return (struct imx21 *) hcd->hcd_priv;
}



void dump_regs(void)
{
#ifdef DEBUG_ALL
    printk(KERN_DEBUG "\n");
    printk(KERN_DEBUG "hwmode     = 0x%08x  cint_stat  = 0x%08x  cint_sten = 0x%08x\n",
           USBOTG_HWMODE, USBOTG_CINT_STAT, USBOTG_CINT_STEN);
    printk(KERN_DEBUG "clk_ctrl   = 0x%08x  rst_ctrl   = 0x%08x  frm_intvl = 0x%08x\n",
           USBOTG_CLK_CTRL, USBOTG_RST_CTRL, USBOTG_FRM_INTVL);
    printk(KERN_DEBUG "frm_remain = 0x%08x  hnp_csr   = 0x%08x  hint_isr = 0x%08x\n",
           USBOTG_FRM_REMAIN, USBOTG_HNP_CSR, USBOTG_HNP_ISR);
    printk(KERN_DEBUG "hnp_ien    = 0x%08x  usbctrl    = 0x%08x\n",
           USBOTG_HNP_IEN, USBCTRL);
    
    printk(KERN_DEBUG "\n");
    printk(KERN_DEBUG "host_ctrl  = 0x%08x  sysisr     = 0x%08x  sysien    = 0x%08x\n",
           USBH_HOST_CTRL, USBH_SYSISR, USBH_SYSIEN);
    printk(KERN_DEBUG "xbufstat   = 0x%08x  ybufstat   = 0x%08x  xyinten   = 0x%08x\n",
           USBH_XBUFSTAT, USBH_YBUFSTAT, USBH_XYINTEN);
    printk(KERN_DEBUG "xfillstat  = 0x%08x  yfillstat  = 0x%08x  etdenset  = 0x%08x\n",
           USBH_XFILLSTAT, USBH_YFILLSTAT, USBH_ETDENSET);
    printk(KERN_DEBUG "etdenclr   = 0x%08x  immedint   = 0x%08x  etddonest = 0x%08x\n",
           USBH_ETDENCLR, USBH_IMMEDINT, USBH_ETDDONESTAT);
    printk(KERN_DEBUG "etddoneen  = 0x%08x  frmnub     = 0x%08x  lsthresh  = 0x%08x\n",
           USBH_ETDDONEEN, USBH_FRMNUB, USBH_LSTHRESH);
    printk(KERN_DEBUG "roothuba   = 0x%08x  roothubb   = 0x%08x  rootstat  = 0x%08x\n",
           USBH_ROOTHUBA, USBH_ROOTHUBB, USBH_ROOTSTAT);
    printk(KERN_DEBUG "portst1    = 0x%08x  portst2    = 0x%08x  portst3   = 0x%08x\n",
           USBH_PORTSTAT(0), USBH_PORTSTAT(1), USBH_PORTSTAT(2));
#endif
}

static inline void dump_ep(struct usb_host_endpoint *ep)
{
#ifdef DEBUG_ALL
    int cnt = 0;
    struct list_head *tmp;

    printk(KERN_DEBUG "ep: %p\n", ep);
    printk(KERN_DEBUG "ep->hcpriv: %p\n", ep->hcpriv);
    list_for_each(tmp, &ep->urb_list) {
        cnt++;
    }

    printk(KERN_DEBUG "ep->urb_list: %p - %d entries\n", &ep->urb_list, cnt);
    if (cnt > 0) {
        printk(KERN_DEBUG "    entry[0] = %p\n", 
               list_entry(ep->urb_list.next, struct urb, urb_list));
    }
    printk(KERN_DEBUG "ep->descriptor:\n");
    printk(KERN_DEBUG "    bEndpointAddress: %d\n", ep->desc.bEndpointAddress);
    printk(KERN_DEBUG "    wMaxPacketSize: %d\n", ep->desc.wMaxPacketSize);
#endif
}

static inline void dump_etd(int etd_num)
{
#ifdef DEBUG_ALL
    volatile u32 *etd_addr = &USB_ETD(etd_num);
    printk(KERN_DEBUG "etd(%d): 0x%08x 0x%08x 0x%08x 0x%08x\n", etd_num,
           etd_addr[0], etd_addr[1], etd_addr[2], etd_addr[3]);
#endif
}

static inline void dump_urb(struct urb *urb)
{
#ifdef DEBUG_ALL
    printk(KERN_DEBUG "urb: %p   %d:%d\n", urb,
           usb_pipedevice(urb->pipe), usb_pipeendpoint(urb->pipe));
    printk(KERN_DEBUG "   ->dev->speed: %s\n", 
           urb->dev->speed == USB_SPEED_FULL ? "FULL" : "LOW");
    printk(KERN_DEBUG "   ->hcpriv: %p\n", urb->hcpriv);
    printk(KERN_DEBUG "   ->pipe: 0x%x  %s  %s\n", urb->pipe,
           usb_pipeout(urb->pipe) ? "OUT" : "IN",
           (usb_pipeisoc(urb->pipe) ? "ISO" :
            usb_pipeint(urb->pipe) ? "INT" :
            usb_pipecontrol(urb->pipe) ? "CTRL" :
            usb_pipebulk(urb->pipe) ? "BULK" : "UNK"));
    printk(KERN_DEBUG "   ->transfer_flags: 0x%x\n", urb->transfer_flags);
    printk(KERN_DEBUG "   ->transfer_buffer: %p\n", urb->transfer_buffer);
    printk(KERN_DEBUG "   ->transfer_dma: 0x%x\n", urb->transfer_dma);
    printk(KERN_DEBUG "   ->transfer_buffer_length: %d\n", urb->transfer_buffer_length);
    printk(KERN_DEBUG "   ->setup_packet: %p\n", urb->setup_packet);
    printk(KERN_DEBUG "   ->setup_dma: 0x%x\n", urb->setup_dma);
    printk(KERN_DEBUG "   ->status: %s\n",
           ((urb->status == 0) ? "OK" :
            (urb->status == -EINPROGRESS) ? "-EINPROGRESS" :
            (urb->status == -EILSEQ) ? "-EILSEQ" :
            (urb->status == -EPROTO) ? "-EPROTO" :
            (urb->status == -EPIPE) ? "-EPIPE" :
            (urb->status == -ETIMEDOUT) ? "-ETIMEDOUT" :
            (urb->status == -EPROTO) ? "-EPROTO" :
            (urb->status == -EOVERFLOW) ? "-EOVERFLOW" :
            (urb->status == -EREMOTEIO) ? "-EREMOTEIO" :
            (urb->status == -EIO) ? "-EIO" :
            (urb->status == -ECOMM) ? "-ECOMM" :
            (urb->status == -ENOSR) ? "-ENOSR" :
            (urb->status == -EALREADY) ? "-EALREADY" : "UNKNOWN"));
            
    if (usb_pipecontrol(urb->pipe)) {
        printk(KERN_DEBUG "       0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
               ((unsigned char*)urb->setup_packet)[0], 
               ((unsigned char*)urb->setup_packet)[1],
               ((unsigned char*)urb->setup_packet)[2],
               ((unsigned char*)urb->setup_packet)[3],
               ((unsigned char*)urb->setup_packet)[4],
               ((unsigned char*)urb->setup_packet)[5],
               ((unsigned char*)urb->setup_packet)[6],
               ((unsigned char*)urb->setup_packet)[7]);
    }
    printk(KERN_DEBUG "   maxpacket: %d\n", 
           usb_maxpacket(urb->dev, urb->pipe, usb_pipeout(urb->pipe)));

    if (usb_pipeisoc(urb->pipe)) {
        int i;
        printk(KERN_DEBUG "   ->interval: %d\n", urb->interval);
        printk(KERN_DEBUG "   ->number_of_packets: %d\n", urb->number_of_packets);

        for (i = 0; i < urb->number_of_packets; i++) {
            printk(KERN_DEBUG"      %d: len:%d  act_len:%d  offset:%d\n",
                   i, urb->iso_frame_desc[i].length,
                   urb->iso_frame_desc[i].actual_length,
                   urb->iso_frame_desc[i].offset);
        }
    }


#endif
}

#endif
