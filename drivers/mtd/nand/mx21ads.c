/******************************************************************************

    drivers/mtd/nand/mx2_nand.c --> drivers/mtd/nand/mx21ads.c
    driver for i.MX21 on-chip NAND Flash controller.
    tested with NAND Flash devices that come with MX21ADS board:
    SAMSUNG K9F1208Q0A (8bit i/o)
    SAMSUNG K9F5616Q0C (16Bit i/o)

    Copyright (c) 2004 MontaVista Software, Inc. <source@mvista.com>

    Based on mx2 nand support code from Motorola's MX21 BSP:

    Copyright (C) 2003 Motorola Inc. Ltd

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
    

********************************************************************************/

#include <linux/slab.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/spinlock.h>
#include <linux/ioport.h>
#include <asm/hardware.h>

//#define NAND_MX21ADS_DEBUG
#ifdef NAND_MX21ADS_DEBUG
#define DBUG(args...) do {dbug_printk(args);} while(0)
#else
#define DBUG(args...) do {} while(0)
#endif


#define NAND_TIMEOUT HZ*5

struct mx2nand_priv {
    u_char mode;
    u_char cmd;
    u32 offset;
    u32 max_offset;
};


#ifdef CONFIG_MTD_CMDLINE_PARTS
const char *mx2nand_probes[] = { "cmdlinepart", NULL };
#endif

static struct mtd_info *mx2nand_mtd = NULL;

static spinlock_t lock = SPIN_LOCK_UNLOCKED;

static uint8_t mx2nand_bi_pattern[] = { 0xff };    /*bad block indicator pattern - I saw only 0's */

static struct nand_oobinfo mx2nand_oob = {
    .useecc = MTD_NANDECC_AUTOPLACE,
    .eccbytes = 0,        /* 0 instead of 3 to prevent writing attempts to ECC area */
    .eccpos = {6, 7, 8},    /*no practical value - SW ECC write is disabled, handled by HW */
    .oobfree = {{0, 5},{11,5}},    /*accessible bytes in the spare area */
};
static struct nand_bbt_descr mx2nand_bbt_descr = {
    .options = 0,
    .offs = 5,
    .len = 1,
    .pattern = mx2nand_bi_pattern,
};


static int
mx2nand_scan_bbt(struct mtd_info *mtd)
{
    nand_scan_bbt(mtd, &mx2nand_bbt_descr);
    return 0;
}

static int
mx2nand_correct_data(struct mtd_info *mtd, u_char * dat,
             u_char * read_ecc, u_char * calc_ecc)
{
/*ecc is handled entirely by the MX2 _reg_NFC*/
/*report ECC Uncorrectable error*/
    if (NFC_ECC_RSLT_MAIN & 0xA)
        return -1;
    else
        return 0;
}

static void
mx2nand_enable_hwecc(struct mtd_info *mtd, int mode)
{
/*ecc is handled entirely by the MX2 _reg_NFC*/
}

static int
mx2nand_calculate_ecc(struct mtd_info *mtd, const u_char * dat,
              u_char * ecc_code)
{
/*ecc is handled entirely by the MX2 _reg_NFC*/
    return 0;
}

static void
mx2nand_wait(void)
{
    unsigned long timeout;
    timeout = jiffies + NAND_TIMEOUT;
    while (!(NFC_FLASH_CONFIG2 & NFC_FLASH_CONFIG2_INT)) {
        if (time_after(jiffies, timeout)) {
            printk(KERN_ERR
                   "MX2_NAND: timeout waiting for Nand command complete\n");
            return;
        }
    }
}

static void
mx2nand_request_data(int cmd)
{
    spin_lock(&lock);
    switch (cmd) {
    case NAND_CMD_READID:
        NFC_FLASH_CONFIG2 = NFC_FLASH_CONFIG2_FDO_ID;
        DBUG("%s(cmd=NAND_CMD_READID): FDO_ID(%x) -> NFC_FLASH_CONFIG2(%x)\n", 
             __FUNCTION__, NFC_FLASH_CONFIG2_FDO_ID, NFC_FLASH_CONFIG2);
        break;
    case NAND_CMD_STATUS:
        NFC_FLASH_CONFIG2 = NFC_FLASH_CONFIG2_FDO_STATUS;
        DBUG("%s(cmd=NAND_CMD_STATUS): FDO_STATUS(%x) -> NFC_FLASH_CONFIG2(%x)\n", 
             __FUNCTION__, NFC_FLASH_CONFIG2_FDO_STATUS, NFC_FLASH_CONFIG2);
        break;
    default:
        NFC_FLASH_CONFIG2 = NFC_FLASH_CONFIG2_FDO_PAGE;
        DBUG("%s(cmd=%x): FDO_PAGE(%x) -> NFC_FLASH_CONFIG2(%x)\n", 
             __FUNCTION__, cmd, NFC_FLASH_CONFIG2_FDO_PAGE, NFC_FLASH_CONFIG2);
    }
    mx2nand_wait();
    spin_unlock(&lock);
}

static u_char
mx2nand_read_byte(struct mtd_info *mtd)
{
    struct nand_chip *chip_priv = mtd->priv;
    struct mx2nand_priv *mx2_priv = chip_priv->priv;
    u32 offset = mx2_priv->offset;
    int cmd = mx2_priv->cmd;
    u16 buf;
    char b;

    if (offset == 0 || offset == 512) {
        mx2nand_request_data(cmd);
    }

    if (offset>=512)
        buf = *((u16 *) NFC_SPARE_BUF3 + ((offset-512) >> 1));      
    else if (cmd == NAND_CMD_READOOB)
        buf = *((u16 *) NFC_SPARE_BUF3 + (offset >> 1));
    else
        buf = *((u16 *) NFC_MAIN_BUF3 + (offset >> 1));   

    b =((u8 *) &buf)[offset & 0x1];

    if (++offset >= mx2_priv->max_offset)
        offset = 0;
    mx2_priv->offset = offset;
    return b;
}

static u16
mx2nand_read_word(struct mtd_info *mtd)
{
    u8 buf[2];

    buf[0] = mx2nand_read_byte(mtd);
    buf[1] = mx2nand_read_byte(mtd);
    return *(u16 *) buf;
}

#ifdef NAND_MX21ADS_DEBUG
static char* cmd2str( u8 cmd, char* cmdstr )
{
    switch (cmd) {
    case NAND_CMD_READ0:    return "NAND_CMD_READ0";
    case NAND_CMD_READ1:    return "NAND_CMD_READ1";
    case NAND_CMD_PAGEPROG: return "NAND_CMD_PAGEPROG";
    case NAND_CMD_READOOB:  return "NAND_CMD_READOOB";
    case NAND_CMD_ERASE1:   return "NAND_CMD_ERASE1";
    case NAND_CMD_STATUS:   return "NAND_CMD_STATUS";
    case NAND_CMD_STATUS_MULTI: return "NAND_CMD_STATUS_MULTI";
    case NAND_CMD_SEQIN:    return "NAND_CMD_SEQIN";
    case NAND_CMD_READID:   return "NAND_CMD_READID";
    case NAND_CMD_ERASE2:    return "NAND_CMD_ERASE2";
    case NAND_CMD_RESET:    return "NAND_CMD_RESET";
    default: sprintf( cmdstr, "cmd=0x%x", cmd ); return cmdstr;
    }
}
#endif

static void
mx2nand_send_command(struct mtd_info *mtd, u8 cmd)
{
    struct nand_chip *chip_priv = mtd->priv;
    struct mx2nand_priv *mx2_priv = chip_priv->priv;
    #ifdef NAND_MX21ADS_DEBUG
    char cmdstr[32];
    #endif

    DBUG("%s(%s[%x])...\n", __FUNCTION__, cmd2str(cmd,cmdstr), cmd);

    mx2_priv->cmd = cmd;    /*remember command for further internal handling */
    if (cmd == NAND_CMD_PAGEPROG) {
        spin_lock(&lock);
        NFC_FLASH_CONFIG2 = NFC_FLASH_CONFIG2_FDI;
        DBUG("%s(): FDI(%x) -> NFC_FLASH_CONFIG2(%x)\n",
             __FUNCTION__, NFC_FLASH_CONFIG2_FDI, NFC_FLASH_CONFIG2);
        mx2nand_wait();
        spin_unlock(&lock);
    }

    /*send command */
    spin_lock(&lock);
    NFC_FLASH_CMD = cmd;
    DBUG("%s(): cmd(%x) -> NFC_FLASH_CMD(%x)\n",
         __FUNCTION__, cmd, NFC_FLASH_CMD);
    NFC_FLASH_CONFIG2 = NFC_FLASH_CONFIG2_FCMD;
    DBUG("%s(): FCMD(%x) -> NFC_FLASH_CONFIG2(%x)\n",
         __FUNCTION__, NFC_FLASH_CONFIG2_FCMD, NFC_FLASH_CONFIG2);
    if (cmd != NAND_CMD_RESET)
        mx2nand_wait();
    spin_unlock(&lock);

    /*extra command processing for read/write-prepare commands */
    switch (cmd) {
    case NAND_CMD_READ0:
        NFC_FLASH_CONFIG1 = 0x8 | 0x2;    /*enable ECC, Spare + Main Area */
        DBUG("%s(): (0x8) -> NFC_FLASH_CONFIG1(%x)\n", 
             __FUNCTION__, NFC_FLASH_CONFIG1);
        mx2_priv->offset = 0;
        mx2_priv->max_offset = 528;
        break;
    case NAND_CMD_READOOB:
        NFC_FLASH_CONFIG1 = 0xC | 0x2;    /*enable ECC, Spare Area Only */
        DBUG("%s(): (0xC) -> NFC_FLASH_CONFIG1(%x)\n", 
             __FUNCTION__, NFC_FLASH_CONFIG1);
        mx2_priv->offset = 512;
        mx2_priv->max_offset = 528;
        break;
    case NAND_CMD_READID:
        NFC_FLASH_CONFIG1 = 0x8 | 0x2;    /*enable ECC, Spare + Main Area */
        DBUG("%s(): (0x8) -> NFC_FLASH_CONFIG1(%x)\n", 
             __FUNCTION__, NFC_FLASH_CONFIG1);
        mx2_priv->offset = 0;
        mx2_priv->max_offset = 6;
        break;
    case NAND_CMD_STATUS:
        NFC_FLASH_CONFIG1 = 0x8 | 0x2;    /*enable ECC, Spare + Main Area */
        DBUG("%s(): (0x8) -> NFC_FLASH_CONFIG1(%x)\n", 
             __FUNCTION__, NFC_FLASH_CONFIG1);
        mx2_priv->offset = 0;
        mx2_priv->max_offset = 1;
        break;
    case NAND_CMD_SEQIN:
        //NO: look at nand_base.c:nand_command(), this will be done by one of
        //    the modes above!  Don't want to change so we can support OOB!
        //mx2_priv->offset = 0;
        //mx2_priv->max_offset = 528;
        break;    
    }
}

static void
mx2nand_send_address(u8 addr)
{
    DBUG("%s(addr=%x)\n", __FUNCTION__, addr);
    NFC_FLASH_ADDR = addr;
    spin_lock(&lock);
    NFC_FLASH_CONFIG2 = NFC_FLASH_CONFIG2_FADD;
    mx2nand_wait();
    spin_unlock(&lock);
}

static void
mx2nand_write_byte(struct mtd_info *mtd, u_char byte)
{
    struct nand_chip *chip_priv = mtd->priv;
    struct mx2nand_priv *mx2_priv = chip_priv->priv;
    u32 offset;
    u16 buf;
    int cmd;
    volatile u16 *dst;

    switch (mx2_priv->mode) {
    case NAND_CTL_SETCLE:
        mx2nand_send_command(mtd, byte);
        break;
    case NAND_CTL_SETALE:
        mx2nand_send_address(byte);
        break;
    default:
        offset = mx2_priv->offset;
        cmd = mx2_priv->cmd;

        if (offset >= 512) 
            dst = ((u16 *) NFC_SPARE_BUF3 + ((offset-512) >> 1));
        else if (cmd == NAND_CMD_READOOB)
            dst = ((u16 *) NFC_SPARE_BUF3 + (offset >> 1));
        else
            dst = ((u16 *) NFC_MAIN_BUF3 + (offset >> 1));

        buf = *dst;
        ((u8 *) &buf)[offset & 0x1] = byte;
        *dst = buf;

        if (++offset >= mx2_priv->max_offset)
            offset = 0;
        mx2_priv->offset = offset;
    }
}

static void
mx2nand_write_word(struct mtd_info *mtd, u16 word)
{
    u8 *buf = (u8 *) &word;

    mx2nand_write_byte(mtd, buf[0]);
    mx2nand_write_byte(mtd, buf[1]);
}


/*
 * Why does this routine exist?  All accesses to the NAND internal memory
 * buffer *MUST* be either 16 or 32 bits, *NOT* 8 bits!  This is an iMX21 
 * requirement, read the reference manual to verify.  If 8 bit I/O is performed
 * on the NAND internal memory, an ARM DATA ABORT exception will occur with the 
 * following message "external abort on non-linefetch (0x8)".
 *
 * The caller of this routine insures that "src" is modulo 2 by using the slow 
 * byte routine, mx2nand_read_byte(), to read 1 byte and adjusting "offset".  
 * So, this routine deals with the cases where "dst" is on a byte boundary or
 * "len" is odd.
 *
 * (Oh and by the way, if you are thinking this could all have been done with
 * memcpy(), it is very hard to guarantee that memcpy() will *NOT* attempt to
 * copy bytes, I know, I tried to get memcpy() to do "the right thing" by 
 * massaging the input parameters and I still ended up with a case where 
 * memcpy() did a byte read from the NAND internal memory, albeit a lot less
 * frequently than I was seeing the condition when memcpy() was being used in
 * the calling routine (mx2nand_read_buf()).) 
 *
 * THIS CODE WAS OPTIMIZED BY LOOKING AT THE ASSEMBLY GENERATED BY THE C
 * COMPILER -- IF YOU WANT TO MAKE CHANGES PLEASE ANALYZE THE ASSEMBLY 
 * LANGUAGE CODE TO MAKE SURE YOU DID NOT SLOW THINGS DOWN, THANKS.
 */
static void
mx2nand_read_aligned(struct mtd_info *mtd, struct mx2nand_priv *mx2_priv,
                     u_char *dst, u_char *src, int len)
{
    int o_len = len;

    len &= ~1;  // make "len" even, only transfer 32bits/16bits during copy loops
    if (len > 0) {
        u32 offset = mx2_priv->offset;
        int src_32 = (((long)src) & 3) == 0; // src is 32-bit aligned
        int dst_32 = (((long)dst) & 3) == 0; // dst is 32-bit aligned
        int dst_16 = (((long)dst) & 1) == 0; // dst is 16-bit aligned
        u_char *end;

        if (dst_32 && src_32) {
            // dst & src aligned to 32-bit boundary, copy 32 bits at a time and
            // then copy the residual 16 bits if necessary
            end = dst + (len & ~3);
            while (dst + (8 * sizeof(u32)) <= end) {
                ((u32*)dst)[0] = ((u32*)src)[0];
                ((u32*)dst)[1] = ((u32*)src)[1];
                ((u32*)dst)[2] = ((u32*)src)[2];
                ((u32*)dst)[3] = ((u32*)src)[3];
                ((u32*)dst)[4] = ((u32*)src)[4];
                ((u32*)dst)[5] = ((u32*)src)[5];
                ((u32*)dst)[6] = ((u32*)src)[6];
                ((u32*)dst)[7] = ((u32*)src)[7];
                dst += 8 * sizeof(u32);
                src += 8 * sizeof(u32);
            }
            while (dst < end) {
                *(u32*)dst = *(u32*)src;
                dst += sizeof(u32);
                src += sizeof(u32);
            }

            if (len & 3) {
                *(u16*)dst = *(u16*)src;
                dst += sizeof(u16);
            }
        } else
        if (dst_16) {
            // dst & src aligned to a 16-bit boundary, copy 16 bits at a time
            end = dst + len;
            while (dst + (8*sizeof(u16)) <= end) {
                ((u16*)dst)[0] = ((u16*)src)[0];
                ((u16*)dst)[1] = ((u16*)src)[1];
                ((u16*)dst)[2] = ((u16*)src)[2];
                ((u16*)dst)[3] = ((u16*)src)[3];
                ((u16*)dst)[4] = ((u16*)src)[4];
                ((u16*)dst)[5] = ((u16*)src)[5];
                ((u16*)dst)[6] = ((u16*)src)[6];
                ((u16*)dst)[7] = ((u16*)src)[7];
                dst += 8 * sizeof(u16);
                src += 8 * sizeof(u16);
            }
            while (dst < end) {
                *(u16*)dst = *(u16*)src;
                dst += sizeof(u16);
                src += sizeof(u16);
            }
        } else {
            // dst is odd, must copy into dst a byte at a time, so align src
            // to a 32-bit boundary, copy 32-bits at a time and then copy the
            // residual 16-bits if necessary
            if (!src_32) {
                u16 w = *(u16*)src;
                dst[0] = ((u8*)&w)[0];
                dst[1] = ((u8*)&w)[1];
                dst += sizeof(u16);
                src += sizeof(u16);
                len -= sizeof(u16);
            }
            for (end = dst + (len & ~3); dst < end; ) {
                u32 l = *(u32*)src;
                dst[0] = ((u8*)&l)[0];
                dst[1] = ((u8*)&l)[1];
                dst[2] = ((u8*)&l)[2];
                dst[3] = ((u8*)&l)[3];
                dst += sizeof(u32);
                src += sizeof(u32);
            }
            if (len & 3) {
                u16 w = *(u16*)src;
                dst[0] = ((u8*)&w)[0];
                dst[1] = ((u8*)&w)[1];
                dst += sizeof(u16);
            }
            len = o_len & ~1;
        }
        if ((offset += len) >= mx2_priv->max_offset) {
            offset = 0;
        }
        mx2_priv->offset = offset;
    }

    // if length was an odd number of bytes, read the last byte
    if (o_len & 1) {
        *dst = mx2nand_read_byte(mtd);
    }
}


static void
mx2nand_read_buf(struct mtd_info *mtd, u_char * buf, int len)
{
    struct nand_chip *chip_priv = mtd->priv;
    struct mx2nand_priv *mx2_priv = chip_priv->priv;
    u32 offset = mx2_priv->offset;
    int cmd = mx2_priv->cmd;
    u_char *sbuf;

    DBUG("%s(): offset=0x%lx, len=0x%x, buf=0x%x\n", 
         __FUNCTION__, offset, len, buf);

    if (len == 0)
        return;

    if (offset == 0 || offset == 512) {
        mx2nand_request_data(cmd);
    }
    else
    if (offset & 1) {
        // word align offset and src
        *buf++ = mx2nand_read_byte(mtd);
        ++offset;
        if (--len == 0) {
            return;
        }
    }

    if (offset >= 512) {
        sbuf = (char *)NFC_SPARE_BUF3;
        mx2nand_read_aligned(mtd, mx2_priv, buf, &sbuf[offset-512], len);
    } else if (cmd == NAND_CMD_READOOB) {
        sbuf = (char *)NFC_SPARE_BUF3;
        mx2nand_read_aligned(mtd, mx2_priv, buf, &sbuf[offset], len);
    } else {
        sbuf = (char *)NFC_MAIN_BUF3;
        mx2nand_read_aligned(mtd, mx2_priv, buf, &sbuf[offset], len);
    }

    DBUG("%s(): offset=0x%lx, len=0x%x, buf[%02x %02x %02x %02x %02x %02x %02x %02x]\n", 
         __FUNCTION__, offset, len, 
         buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
}


/*
** Refer to notes in the mx2nand_read_aligned() routine above for details.
** (Switch src and dst when reading the details above.)
*/
static void
mx2nand_write_aligned(struct mtd_info *mtd, struct mx2nand_priv *mx2_priv,
                      u_char *dst, u_char *src, int len)
{
    int o_len = len;

    len &= ~1;  // make "len" even, only transfer 32bits/16bits during copy loops
    if (len > 0) {
        u32 offset = mx2_priv->offset;
        int dst_32 = (((long)dst) & 3) == 0; // dst is 32-bit aligned
        int src_32 = (((long)src) & 3) == 0; // src is 32-bit aligned
        int src_16 = (((long)src) & 1) == 0; // src is 16-bit aligned
        u_char *end;

        DBUG("%s(): offset=0x%lx, dst=0x%x, src=0x%x, len=0x%x\n", 
             __FUNCTION__, offset, dst, src, len);

        if (dst_32 && src_32) {
            // dst & src aligned to 32-bit boundary, copy 32 bits at a time and
            // then copy the residual 16 bits if necessary
            end = dst + (len & ~3);
            while (dst + (8 * sizeof(u32)) <= end) {
                ((u32*)dst)[0] = ((u32*)src)[0];
                ((u32*)dst)[1] = ((u32*)src)[1];
                ((u32*)dst)[2] = ((u32*)src)[2];
                ((u32*)dst)[3] = ((u32*)src)[3];
                ((u32*)dst)[4] = ((u32*)src)[4];
                ((u32*)dst)[5] = ((u32*)src)[5];
                ((u32*)dst)[6] = ((u32*)src)[6];
                ((u32*)dst)[7] = ((u32*)src)[7];
                dst += 8 * sizeof(u32);
                src += 8 * sizeof(u32);
            }
            while (dst < end) {
                *(u32*)dst = *(u32*)src;
                dst += sizeof(u32);
                src += sizeof(u32);
            }

            if (len & 2) {
                *(u16*)dst = *(u16*)src;
                src += sizeof(u16);
            }
        } else
        if (src_16) {
            // dst & src aligned to a 16-bit boundary, copy 16 bits at a time
            end = dst + len;
            while (dst + (8*sizeof(u16)) <= end) {
                ((u16*)dst)[0] = ((u16*)src)[0];
                ((u16*)dst)[1] = ((u16*)src)[1];
                ((u16*)dst)[2] = ((u16*)src)[2];
                ((u16*)dst)[3] = ((u16*)src)[3];
                ((u16*)dst)[4] = ((u16*)src)[4];
                ((u16*)dst)[5] = ((u16*)src)[5];
                ((u16*)dst)[6] = ((u16*)src)[6];
                ((u16*)dst)[7] = ((u16*)src)[7];
                dst += 8 * sizeof(u16);
                src += 8 * sizeof(u16);
            }
            while (dst < end) {
                *(u16*)dst = *(u16*)src;
                dst += sizeof(u16);
                src += sizeof(u16);
            }
        } else {
            // src is odd, must copy from src a byte at a time, so align dst
            // to a 32-bit boundary, copy 32-bits at a time and then copy the
            // residual 16-bits if necessary
            if (!dst_32) {
                u16 w;
                ((u8*)&w)[0] = src[0];
                ((u8*)&w)[1] = src[1];
                *(u16*)dst = w;
                dst += sizeof(u16);
                src += sizeof(u16);
                len -= sizeof(u16);
            }
            for (end = dst + (len & ~3); dst < end; ) {
                u32 l;
                ((u8*)&l)[0] = src[0];
                ((u8*)&l)[1] = src[1];
                ((u8*)&l)[2] = src[2];
                ((u8*)&l)[3] = src[3];
                *(u32*)dst = l;
                dst += sizeof(u32);
                src += sizeof(u32);
            }
            if (len & 2) {
                u16 w;
                ((u8*)&w)[0] = src[0];
                ((u8*)&w)[1] = src[1];
                *(u16*)dst = w;
                src += sizeof(u16);
            }
            len = o_len & ~1;
        }
        if ((offset += len) >= mx2_priv->max_offset) {
            offset = 0;
        }
        mx2_priv->offset = offset;
    }

    // if length was an odd number of bytes, write the last byte
    if (o_len & 1) {
        mx2nand_write_byte(mtd,*src);
    }
}


static void
mx2nand_write_buf(struct mtd_info *mtd, const u_char * buf, int len)
{
    struct nand_chip *chip_priv = mtd->priv;
    struct mx2nand_priv *mx2_priv = chip_priv->priv;
    u32 offset = mx2_priv->offset;
    char *sbuf;
    int i;

    DBUG("%s(): offset=0x%lx, len=0x%x, buf=0x%x[%02x %02x %02x %02x %02x %02x %02x %02x]\n", 
         __FUNCTION__, offset, len, buf,
         buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);

    if (len==5) {
        for (i = 0; i < 512; i++)
            mx2nand_write_byte(mtd, 0xff);
        for (i = 0; i < len; i++)
            mx2nand_write_byte(mtd, buf[i]);
    } else {
        if (len == 0)
            return;

        if (offset & 1) {
            // word align offset and dst
            mx2nand_write_byte(mtd, *buf++);
            ++offset;
            if (--len == 0) {
                return;
            }
        }
        if (offset >= 512) {
            sbuf = (char *)NFC_SPARE_BUF3;
            mx2nand_write_aligned(mtd, mx2_priv, &sbuf[offset-512], (u_char*)buf, len);
        } else {
            sbuf = (char *)NFC_MAIN_BUF3;
            mx2nand_write_aligned(mtd, mx2_priv, &sbuf[offset], (u_char*)buf, len);
        }
    }
}

static int
mx2nand_verify_buf(struct mtd_info *mtd, const u_char * buf, int len)
{
    int i;
    char b;
        
    for (i = 0; i < len; i++) {
        b = mx2nand_read_byte(mtd);
        if (buf[i] != b) {
            printk("%s/%s(): buf[%d]=%0x,b=%0x;\n",
                   __FILE__, __FUNCTION__, i,buf[i],b);
            return -EFAULT;
        }
    }
    return 0;
}

static void
mx2nand_hwcontrol(struct mtd_info *mtd, int cmd)
{
    struct nand_chip *chip_priv = mtd->priv;
    struct mx2nand_priv *mx2_priv = chip_priv->priv;

    switch (cmd) {
    case NAND_CTL_SETNCE:    /* Enable _reg_NFC Clock */
        DBUG("%s(NAND_CTL_SETNCE)\n", __FUNCTION__);
        PCCR0 |=  PCCR0_NFC_EN;
        mx2_priv->mode = 0;
        break;
    case NAND_CTL_CLRNCE:    /* Disable _reg_NFC Clock */
        DBUG("%s(NAND_CTL_CLRNCE)\n", __FUNCTION__);
        PCCR0 &=  ~PCCR0_NFC_EN;
        mx2_priv->mode = 0;
        break;
    case NAND_CTL_SETCLE:    /* Send Command mode start */
        mx2_priv->mode = cmd;
        NFC_CONFIG = 0x0002;            /* unlock internal buffer lock */
        NFC_UNLOCK_BLK_START = 0;       /* start of address */
        NFC_UNLOCK_BLK_END = 0xffff;    /* end of address -- unlock all */
        NFC_WR_PROT = 0x0004;           /* unlock NAND flash blocks */
        NFC_RAM_BUF_ADDR = 0x3;         /* use buffer 3 -- NFC_MAIN_BUF3 */
        DBUG("%s(NAND_CTL_SETCLE): NFC_WR_PROT_STAT=%x\n", 
             __FUNCTION__, NFC_WR_PROT_STAT);
        break;
    case NAND_CTL_CLRCLE:    /* Send Command mode end */
        DBUG("%s(NAND_CTL_CLRCLE)\n", __FUNCTION__);
        mx2_priv->mode = 0;
        break;
    case NAND_CTL_SETALE:    /* Send address mode start */
        DBUG("%s(NAND_CTL_SETALE)\n", __FUNCTION__);
        mx2_priv->mode = cmd;
        break;
    case NAND_CTL_CLRALE:    /* Send address mode end */
        DBUG("%s(NAND_CTL_CLRALE)\n", __FUNCTION__);
        mx2_priv->mode = 0;
        break;
    default:
        DBUG("%s(cmd=%x)\n", __FUNCTION__, cmd);
        mx2_priv->mode = 0;
    }
}


const static struct mtd_partition mx2nand_partition_info[] = {
    {name: "Boot Loader", offset: 0x0000000, size: 0x0100000},  /*0*/
    {name: "PSP",         offset: 0x0100000, size: 0x0200000},  /*1*/
    {name: "Kernel2",     offset: 0x0300000, size: 0x0280000},  /*2*/
    {name: "Root Disk2",  offset: 0x0580000, size: 0x0800000},  /*3*/
    {name: "Kernel1",     offset: 0x0D80000, size: 0x0300000},  /*4*/
    {name: "Root Disk1",  offset: 0x1080000, size: 0x1600000},  /*5*/
    {name: "Cache",       offset: 0x2680000, size: 0x1800000},  /*6*/
    {name: "MSP",         offset: 0x3F80000, size: 0x0080000},  /*7*/
};


/*NAND pinout:
_reg_NFCE_B   T15  PF1
_reg_NFCLE    U15  PF3
NFALE    U14  PF4
NFWE_B   T13  PF6
NFRE_B   W13  PF5
NFWP_B   L12  PF2
NFRB     M12  PF0
NFIO0..7  W12, U13, L11, T12, U12, T11, V13, M11  PF7..PF14
A13..A15, A21..A25  F3, F1, G3, G1, H3, L2, L1, M2

Note: PF0..PF14 port mux mode is PRIMARY
*/


/**
 * Module clean up routine
 */
static void __init
mx2nand_cleanup(void)
{
#ifdef CONFIG_MTD_PARTITIONS
    /* Clean up partitions */
    del_mtd_partitions(mx2nand_mtd);
#else
    /* Unregister the device */
    del_mtd_device(mx2nand_mtd);
#endif

    /* Free the MTD device structure */
    kfree(mx2nand_mtd);
}


/**
 * Module initialization routine
 */
int __init
mx2nand_init(void)
{
    struct nand_chip *this;
    int mtd_parts_nb = 0;
    int tmp;

    /* Set to 8 bit mode */
    printk(KERN_INFO "MX2 NAND: 8-bit i/o mode\n");
    FMCR &= ~FMCR_NF_16BIT;

    /* Allocate memory for MTD device structure and private data */
    mx2nand_mtd = kzalloc(sizeof (struct mtd_info) + 
                          sizeof (struct nand_chip) +
                          sizeof (struct mx2nand_priv), GFP_KERNEL);
    if (!mx2nand_mtd) {
        printk(KERN_ERR
               "MX2 NAND: Not enough memory for a MTD device structure\n");
        mx2nand_cleanup();
        return -ENOMEM;
    }

    /* Get private data pointer and set up private pointers */
    this = (struct nand_chip *) ((char*) mx2nand_mtd + sizeof(struct mtd_info));
    this->priv = (struct mx2nand_priv *) ((char *) this + sizeof(*this));
    mx2nand_mtd->priv = this;    /* Link private data with MTD struct */

    this->hwcontrol  = mx2nand_hwcontrol;
    this->read_byte  = mx2nand_read_byte;
    this->write_byte = mx2nand_write_byte;   /*used to send out a command or address */
    this->read_word  = mx2nand_read_word;    /*used once at initialization 16-bit */
    this->write_word = mx2nand_write_word;   /*never used */
    this->read_buf   = mx2nand_read_buf;
    this->write_buf  = mx2nand_write_buf;
    this->verify_buf = mx2nand_verify_buf;

    /* ECC handling */
    this->autooob       = &mx2nand_oob;
    this->enable_hwecc  = mx2nand_enable_hwecc;
    this->calculate_ecc = mx2nand_calculate_ecc;
    this->eccmode       = NAND_ECC_HW3_512;
    this->correct_data  = mx2nand_correct_data;
    /* end of ECC handling */

    /* BBT handling */
    this->scan_bbt = mx2nand_scan_bbt;

    /* Scan to find existence of the device */
    if (nand_scan(mx2nand_mtd, 1)) {
        printk(KERN_ERR "MX2 NAND: device not found\n");
        mx2nand_cleanup();
        return -ENXIO;
    }

    if (mx2nand_mtd->oobsize != 16 || mx2nand_mtd->oobblock != 512) {
        printk(KERN_ERR "MX2 NAND: Unsupported NAND Flash configuration\n");
        mx2nand_cleanup();
        return -EFAULT;
    }

    /* Register the partitions */
#ifdef CONFIG_MTD_PARTITIONS
#ifdef CONFIG_MTD_CMDLINE_PARTS
    {
        struct mtd_partition *mtd_parts;
        mx2nand_mtd->name = "mx21ads-nand";
        mtd_parts_nb = parse_mtd_partitions(mx2nand_mtd, mx2nand_probes,
                                            &mtd_parts, 0);
        if (mtd_parts_nb > 0) {
            tmp = add_mtd_partitions(mx2nand_mtd, mtd_parts, mtd_parts_nb);
        }
    }
#endif                /* end of CONFIG_MTD_CMDLINE_PARTS */
    if (mtd_parts_nb <= 0) {
        tmp = add_mtd_partitions(mx2nand_mtd, mx2nand_partition_info,
                     ARRAY_SIZE(mx2nand_partition_info));
    }
    if (tmp < 0) {
        printk(KERN_ERR "MX2 NAND: Error registering MTD partitions\n");
        mx2nand_cleanup();
        return tmp;
    }
#else                /* if !CONFIG_MTD_PARTITIONS */
    tmp = add_mtd_device(mx2nand_mtd);
    if (tmp < 0) {
        printk(KERN_ERR "MX2 NAND: Error registering MTD device\n");
        mx2nand_cleanup();
        return tmp;
    }
#endif                /* end of CONFIG_MTD_PARTITIONS */

    /* Return happy */
    return 0;
}

module_init(mx2nand_init);
module_exit(mx2nand_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("MontaVista Software, Inc");
MODULE_DESCRIPTION("MTD Driver for MX21 Nand Flash Controller");
