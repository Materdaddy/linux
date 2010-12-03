/*
 *  linux/drivers/video/imxfb.c
 *
 *  Freescale i.MX Frame Buffer device driver
 *
 *  Copyright (C) 2004 Sascha Hauer, Pengutronix
 *   Based on acornfb.c Copyright (C) Russell King.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 * Please direct your questions and comments on this driver to the following
 * email address:
 *
 *	linux-arm-kernel@lists.arm.linux.org.uk
 */

#define DEBUG 1

// pr_debug was replaced with printk

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/cpufreq.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/arch/imxfb.h>

/*
 * Complain if VAR is out of range.
 */
#define DEBUG_VAR 0

#include "imxfb.h"
extern unsigned int chumby_hwversion;

static struct imxfb_rgb def_rgb_32 = {
	.red	= { .offset = 16, .length = 8, },
	.green	= { .offset = 8,  .length = 8, },
	.blue	= { .offset = 0,  .length = 8, },
	.transp = { .offset = 0,  .length = 0, },
};

static struct imxfb_rgb def_rgb_16 = {
	.red	= { .offset = 11, .length = 5, },
	.green	= { .offset = 5,  .length = 6, },
	.blue	= { .offset = 0,  .length = 5, },
	.transp = { .offset = 0,  .length = 0, },
};

static struct imxfb_rgb def_rgb_8 = {
	.red	= { .offset = 0,  .length = 8, },
	.green	= { .offset = 0,  .length = 8, },
	.blue	= { .offset = 0,  .length = 8, },
	.transp = { .offset = 0,  .length = 0, },
};

static int imxfb_activate_var(struct fb_var_screeninfo *var, struct fb_info *info);
static int imxfbov_activate_var(struct fb_var_screeninfo *var, struct fb_info *info);

static inline u_int chan_to_field(u_int chan, struct fb_bitfield *bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}

#define LCDC_PALETTE(x) __REG2(IMX_LCDC_BASE+0x800, (x)<<2)
#define LCDCOV_PALETTE(x) __REG2(IMX_LCDC_BASE+0xc00, (x)<<2)
#define CNVT_TOHW(val,width) ((((val)<<(width))+0x7FFF-(val))>>16)
static int
imxfb_setpalettereg(u_int regno, u_int red, u_int green, u_int blue,
		       u_int trans, struct fb_info *info)
{
	struct imxfb_info *fbi = info->par;
	u_int val, ret = 1;

	printk( "imxfb_setpalettereg\n" );
	if (regno < fbi->palette_size) {
		val = (CNVT_TOHW(red, 4) << 8) |
		      (CNVT_TOHW(green,4) << 4) |
		      CNVT_TOHW(blue,  4);
		LCDC_PALETTE(regno) = val;
		ret = 0;
	}
	return ret;
}

static int
imxfbov_setpalettereg(u_int regno, u_int red, u_int green, u_int blue,
		       u_int trans, struct fb_info *info)
{
	struct imxfb_info *fbi = info->par;
	u_int val, ret = 1;

	printk( "imxfbov_setpalettereg\n" );
	if (regno < fbi->palette_size) {
		val = (CNVT_TOHW(red, 4) << 8) |
		      (CNVT_TOHW(green,4) << 4) |
		      CNVT_TOHW(blue,  4);

		LCDCOV_PALETTE(regno) = val;
		ret = 0;
	}
	return ret;
}

static int
imxfb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
		   u_int trans, struct fb_info *info)
{
	struct imxfb_info *fbi = info->par;
	unsigned int val;
	int ret = 1;

	/*
	 * If inverse mode was selected, invert all the colours
	 * rather than the register number.  The register number
	 * is what you poke into the framebuffer to produce the
	 * colour you requested.
	 */
	if (fbi->cmap_inverse) {
		red   = 0xffff - red;
		green = 0xffff - green;
		blue  = 0xffff - blue;
	}

	/*
	 * If greyscale is true, then we convert the RGB value
	 * to greyscale no mater what visual we are using.
	 */
	if (info->var.grayscale)
		red = green = blue = (19595 * red + 38470 * green +
					7471 * blue) >> 16;

	switch (info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		/*
		 * 12 or 16-bit True Colour.  We encode the RGB value
		 * according to the RGB bitfield information.
		 */
		if (regno < 16) {
			u32 *pal = info->pseudo_palette;

			val  = chan_to_field(red, &info->var.red);
			val |= chan_to_field(green, &info->var.green);
			val |= chan_to_field(blue, &info->var.blue);

			pal[regno] = val;
			ret = 0;
		}
		break;

	case FB_VISUAL_STATIC_PSEUDOCOLOR:
	case FB_VISUAL_PSEUDOCOLOR:
		ret = imxfb_setpalettereg(regno, red, green, blue, trans, info);
		break;
	}

	return ret;
}
static int
imxfbov_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
		   u_int trans, struct fb_info *info)
{
	struct imxfb_info *fbi = info->par;
	unsigned int val;
	int ret = 1;

	/*
	 * If inverse mode was selected, invert all the colours
	 * rather than the register number.  The register number
	 * is what you poke into the framebuffer to produce the
	 * colour you requested.
	 */
	if (fbi->cmap_inverse) {
		red   = 0xffff - red;
		green = 0xffff - green;
		blue  = 0xffff - blue;
	}

	/*
	 * If greyscale is true, then we convert the RGB value
	 * to greyscale no mater what visual we are using.
	 */
	if (info->var.grayscale)
		red = green = blue = (19595 * red + 38470 * green +
					7471 * blue) >> 16;

	switch (info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		/*
		 * 12 or 16-bit True Colour.  We encode the RGB value
		 * according to the RGB bitfield information.
		 */
		if (regno < 16) {
			u32 *pal = info->pseudo_palette;

			val  = chan_to_field(red, &info->var.red);
			val |= chan_to_field(green, &info->var.green);
			val |= chan_to_field(blue, &info->var.blue);

			pal[regno] = val;
			ret = 0;
		}
		break;

	case FB_VISUAL_STATIC_PSEUDOCOLOR:
	case FB_VISUAL_PSEUDOCOLOR:
		ret = imxfbov_setpalettereg(regno, red, green, blue, trans, info);
		break;
	}

	return ret;
}

/*
 *  imxfb_check_var():
 *    Round up in the following order: bits_per_pixel, xres,
 *    yres, xres_virtual, yres_virtual, xoffset, yoffset, grayscale,
 *    bitfields, horizontal timing, vertical timing.
 */
static int
imxfb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct imxfb_info *fbi = info->par;
	int rgbidx;

	if (var->xres < MIN_XRES)
		var->xres = MIN_XRES;
	if (var->yres < MIN_YRES)
		var->yres = MIN_YRES;
	if (var->xres > fbi->max_xres)
		var->xres = fbi->max_xres;
	if (var->yres > fbi->max_yres)
		var->yres = fbi->max_yres;
	var->xres_virtual = max(var->xres_virtual, var->xres);
	var->yres_virtual = max(var->yres_virtual, var->yres);

	printk("var->bits_per_pixel=%d\n", var->bits_per_pixel);
	switch (var->bits_per_pixel) {
	case 32:
		rgbidx = RGB_32;
		break;
	case 16:
		rgbidx = RGB_16;
		break;
	case 8:
		rgbidx = RGB_8;
		break;
	default:
		rgbidx = RGB_16;
	}

	/*
	 * Copy the RGB parameters for this display
	 * from the machine specific parameters.
	 */
	var->red    = fbi->rgb[rgbidx]->red;
	var->green  = fbi->rgb[rgbidx]->green;
	var->blue   = fbi->rgb[rgbidx]->blue;
	var->transp = fbi->rgb[rgbidx]->transp;

	printk("RGBT length = %d:%d:%d:%d\n",
		var->red.length, var->green.length, var->blue.length,
		var->transp.length);

	printk("RGBT offset = %d:%d:%d:%d\n",
		var->red.offset, var->green.offset, var->blue.offset,
		var->transp.offset);

	return 0;
}

/*
 * imxfb_set_par():
 *	Set the user defined part of the display for the specified console
 */

static int imxfb_set_par(struct fb_info *info)
{
	struct imxfb_info *fbi = info->par;
	struct fb_var_screeninfo *var = &info->var;

	printk("set_par\n");

	if (var->bits_per_pixel >= 16)
		info->fix.visual = FB_VISUAL_TRUECOLOR;
	else if (!fbi->cmap_static)
		info->fix.visual = FB_VISUAL_PSEUDOCOLOR;
	else {
		/*
		 * Some people have weird ideas about wanting static
		 * pseudocolor maps.  I suspect their user space
		 * applications are broken.
		 */
		info->fix.visual = FB_VISUAL_STATIC_PSEUDOCOLOR;
	}

	info->fix.line_length = var->xres_virtual *
				  var->bits_per_pixel / 8;
	fbi->palette_size = var->bits_per_pixel == 8 ? 256 : 16;

	imxfb_activate_var(var, info);

	return 0;
}

/*
 * imxfb_set_par():
 *	Set the user defined part of the display for the specified console
 */
static int imxfbov_set_par(struct fb_info *info)
{
	struct imxfb_info *fbi = info->par;
	struct fb_var_screeninfo *var = &info->var;

	printk("set_par\n");

	if (var->bits_per_pixel >= 16)
		info->fix.visual = FB_VISUAL_TRUECOLOR;
	else if (!fbi->cmap_static)
		info->fix.visual = FB_VISUAL_PSEUDOCOLOR;
	else {
		/*
		 * Some people have weird ideas about wanting static
		 * pseudocolor maps.  I suspect their user space
		 * applications are broken.
		 */
		info->fix.visual = FB_VISUAL_STATIC_PSEUDOCOLOR;
	}

	info->fix.line_length = var->xres_virtual *
				  var->bits_per_pixel / 8;
	fbi->palette_size = var->bits_per_pixel == 8 ? 256 : 16;

	imxfbov_activate_var(var, info);

	return 0;
}

static void imxfb_enable_controller(struct imxfb_info *fbi)
{
  int i;
	printk("Enabling LCD controller\n");

	/* initialize LCDC */
	LCDC_RMCR &= ~RMCR_LCDC_EN;		/* just to be safe... */

        if (fbi->pcr & PCR_REV_VS) {
            /* Vertical scan is reversed */
            int xres = fbi->max_xres;
            int yres = fbi->max_yres;
            int bpp  = fbi->max_bpp;
            int offset = (xres * (yres - 1) * (bpp / 8));

	    // LCDC frame buffer logo force setting code framebuffer -- bunnie
	    for( i = 0; i < 0x25800; i++ ) {
	      *((unsigned char *)fbi->screen_dma + offset + i) = *((unsigned char *)0xC2500000 + i);
	    }
            LCDC_SSA	= fbi->screen_dma + offset;
        } else {
	    // LCDC frame buffer logo force setting code framebuffer -- bunnie
	    for( i = 0; i < 0x25800; i++ ) {
	      *((unsigned char *)fbi->screen_dma + i) = *((unsigned char *)0xC2500000 + i);
	    }
            LCDC_SSA	= fbi->screen_dma;
        }

	/* physical screen start address */
	LCDC_VPW	= VPW_VPW(fbi->max_xres * fbi->max_bpp / 8 / 4);

        /* panning offset 0 (0 pixel offset) */
	LCDC_POS	= 0x00000000;

	/* disable hardware cursor */
	LCDC_CPOS	&= ~(CPOS_CC0 | CPOS_CC1);

        /* fixed burst length (see erratum 11) */
        LCDC_DMACR =  DMACR_HM(3) | DMACR_TM(8);

	//LCDC_RMCR = RMCR_LCDC_EN;
	LCDC_RMCR = 0;

	if(fbi->backlight_power)
		fbi->backlight_power(1);
	if(fbi->lcd_power)
		fbi->lcd_power(1);

	printk( "LCDC register set:\n" );
	for( i = 0; i < 0x68; i+= 4 ) {
	  printk( "LCDbase + 0x%02X: %08X\n", i, __REG(IMX_LCDC_BASE+i) );
	}

}
static void imxfbov_enable_controller(struct imxfb_info *fbi)
{
	printk("Enabling LCD controller\n");

	/* initialize LCDC */
	LCDC_RMCR &= ~RMCR_LCDC_EN;		/* just to be safe... */

        LCDC_LGWSAR	= fbi->ov_screen_dma;

	/* physical screen start address */
	LCDC_LGWVPWR	= VPW_VPW(fbi->max_xres * fbi->max_bpp / 8 / 4);

        /* panning offset 0 (0 pixel offset) */
	LCDC_LGWPOR	= 0x00000000;

        /* fixed burst length (see erratum 11) */
        LCDC_LGWDCR =  DMACR_HM(3) | DMACR_TM(8);

}

static void imxfb_disable_controller(struct imxfb_info *fbi)
{
	printk("Disabling LCD controller\n");

	if(fbi->backlight_power)
		fbi->backlight_power(0);
	if(fbi->lcd_power)
		fbi->lcd_power(0);

	LCDC_RMCR = 0;
}

static int imxfb_blank(int blank, struct fb_info *info)
{
	// struct imxfb_info *fbi = info->par;

	printk("imxfb_blank: blank=%d\n", blank);
	printk("imxfb_blank IGNORED because some stupid thing keeps on calling this to turn off my @#)$!)($ display.\n" );

#if 0 // HA! SCREW YOU PHANTOM POWER MANAGER!!!
	switch (blank) {
	case FB_BLANK_POWERDOWN:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_NORMAL:
		imxfb_disable_controller(fbi);
		break;

	case FB_BLANK_UNBLANK:
		imxfb_enable_controller(fbi);
		break;
	}
#endif
	return 0;
}

static struct fb_ops imxfb_ops = {
	.owner		= THIS_MODULE,
	.fb_check_var	= imxfb_check_var,
	.fb_set_par	= imxfb_set_par,
	.fb_setcolreg	= imxfb_setcolreg,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
	.fb_blank	= imxfb_blank,
};

static struct fb_ops imxfbov_ops = {
	.owner		= THIS_MODULE,
	.fb_check_var	= imxfb_check_var,
	.fb_set_par	= imxfbov_set_par,
	.fb_setcolreg	= imxfbov_setcolreg,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
};

/*
 * imxfb_activate_var():
 *	Configures LCD Controller based on entries in var parameter.  Settings are
 *	only written to the controller if changes were made.
 */
static int imxfb_activate_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct imxfb_info *fbi = info->par;
	printk( "imxfb_activate_var!\n" );

	printk("var: xres=%d hslen=%d lm=%d rm=%d\n",
		var->xres, var->hsync_len,
		var->left_margin, var->right_margin);
	printk("var: yres=%d vslen=%d um=%d bm=%d\n",
		var->yres, var->vsync_len,
		var->upper_margin, var->lower_margin);

#if DEBUG_VAR
	if (var->xres < 16        || var->xres > 1024)
		printk(KERN_ERR "%s: invalid xres %d\n",
			info->fix.id, var->xres);
	if (var->hsync_len < 1    || var->hsync_len > 64)
		printk(KERN_ERR "%s: invalid hsync_len %d\n",
			info->fix.id, var->hsync_len);
	if (var->left_margin > 255)
		printk(KERN_ERR "%s: invalid left_margin %d\n",
			info->fix.id, var->left_margin);
	if (var->right_margin > 255)
		printk(KERN_ERR "%s: invalid right_margin %d\n",
			info->fix.id, var->right_margin);
	if (var->yres < 1 || var->yres > 511)
		printk(KERN_ERR "%s: invalid yres %d\n",
			info->fix.id, var->yres);
	if (var->vsync_len > 100)
		printk(KERN_ERR "%s: invalid vsync_len %d\n",
			info->fix.id, var->vsync_len);
	if (var->upper_margin > 63)
		printk(KERN_ERR "%s: invalid upper_margin %d\n",
			info->fix.id, var->upper_margin);
	if (var->lower_margin > 255)
		printk(KERN_ERR "%s: invalid lower_margin %d\n",
			info->fix.id, var->lower_margin);
#endif

	LCDC_POS = 0x0;
	LCDC_CPOS = 0x0;
	LCDC_LCWHB = 0x0;
	LCDC_LCHCC = 0x0;
	__REG(IMX_LCDC_BASE+0x64) = 0x0; // LCDC_LGWCR

	LCDC_HCR	= HCR_H_WIDTH(var->hsync_len) |
	                  HCR_H_WAIT_1(var->left_margin) |
			  HCR_H_WAIT_2(var->right_margin);

	LCDC_VCR	= VCR_V_WIDTH(var->vsync_len) |
	                  VCR_V_WAIT_1(var->upper_margin) |
			  VCR_V_WAIT_2(var->lower_margin);

	LCDC_SIZE	= SIZE_XMAX(var->xres) | SIZE_YMAX(var->yres);
	LCDC_PCR	= fbi->pcr;
	LCDC_PWMR	= fbi->pwmr;
	LCDC_LSCR1	= fbi->lscr1;
	LCDC_DMACR	= fbi->dmacr;

	// crap code here, delete it later and fix what's wrong above
	// Disable self-refresh by clearing bit LRMCR[9]
	LCDC_RMCR = 0x00000000;

	if( chumby_hwversion == 0x38 ) {
	  // Setup LCDC panel configuration
	  //   TFT      (LPCR[31])    = 1  		:TFT display
	  //   COLOR    (LPCR[30])    = 1  		:Color display
	  //   PBSIZ    (LPCR[29:28]) = 00 		:Active matrix fixed at 16bpp
	  
	  //   BPIX     (LPCR[27:25]) = 101		:16pp in memory
	  //   PIXPOL   (LPCR[24])    = 0  		:Pixels are active high
	  
	  //   FLMPOL   (LPCR[23])    = 1  		:VSYNC is active low
	  //   LPPOL    (LPCR[22])    = 1  		:HSYNC is active low
	  //   CLKPOL   (LPCR[21])    = 0  		:LSCLK is active on positive edge
	  //   OEPOL    (LPCR[20])    = 1  		:OE is active low
	  
	  //   SCLKIDLE (LPCR[19])    = 1  		:Enable LSCLK when VSYNC is idle
	  //   END_SEL  (LPCR[18])    = 0  		:Image in memory is in little endian format
	  //   SWAP_SEL (LPCR[17])    = 0  		:Ignored for big endian format
	  //   REV_VS   (LPCR[16])    = 0  		:Vertical scan in normal direction
	  
	  //   ACDSEL   (LPCR[15])    = 0  		:Ignored for TFT display
	  //   ACD      (LPCR[14:8])  = 0000000 	:Ignored for TFT display
	  
	  //   SCLKSEL  (LPCR[7])     = 1  		:Enable OE and LSCLK when no data output
	  //   SHARP    (LPCR[6])     = 0  		:Disable Sharp signals
	  //   PCD      (LPCR[5:0])   = 000100  	:Pixel Clock = LSCLK = 33.25MHz/5 = 6.65MHz (DI demo board is @ 5.26MHz)
	  //                          = 000111  	:V and H timing calculated with Pixel Clock = LSCLK = 33.25MHz/8 = 4.15MHz, but less noise at higher LSCLK
	  // 1100 1010 1101 1000 0000 0000 1000 0100
	  LCDC_PCR = 0xCAD80084;
	  
	  // If we're not using a Sharp panel, clear this register to be safe
	  LCDC_LSCR1 = 0x00000000;
	  
	  // Configure horizontal LCD timing
	  // hwidth = 2, h_wait_2 = 68 - 3 - 3 = 62 = 0x3e, h_wait_1 = 19 = 0x13
	  // 0000 1000 0000 0000 0001 0011 0011 1110 
	  LCDC_HCR = 0x0800133E;
	  
	  // Configure vertical LCD timing
	  // v_width = 2, v_wait_1 = 18, v_wait_2 = 4
	  LCDC_VCR = 0x10001204;
	} else {
	  // Setup LCDC panel configuration
	  //   TFT      (LPCR[31])    = 1  		:TFT display
	  //   COLOR    (LPCR[30])    = 1  		:Color display
	  //   PBSIZ    (LPCR[29:28]) = 00 		:Active matrix fixed at 16bpp
	  
	  //   BPIX     (LPCR[27:25]) = 101		:16pp in memory
	  //   PIXPOL   (LPCR[24])    = 0  		:Pixels are active low
	  
	  //   FLMPOL   (LPCR[23])    = 0  		:VSYNC is active high
	  //   LPPOL    (LPCR[22])    = 0  		:HSYNC is active high
	  //   CLKPOL   (LPCR[21])    = 0  		:LSCLK is active on positive edge
	  //   OEPOL    (LPCR[20])    = 0  		:OE is active high
	  
	  //   SCLKIDLE (LPCR[19])    = 1  		:Enable LSCLK when VSYNC is idle
	  //   END_SEL  (LPCR[18])    = 0  		:Image in memory is in little endian format
	  //   SWAP_SEL (LPCR[17])    = 0  		:Ignored for big endian format
	  //   REV_VS   (LPCR[16])    = 0  		:Vertical scan in normal direction
	  
	  //   ACDSEL   (LPCR[15])    = 0  		:Ignored for TFT display
	  //   ACD      (LPCR[14:8])  = 0000000 	:Ignored for TFT display

	  //   SCLKSEL  (LPCR[7])     = 1  		:Enable OE and LSCLK when no data output
	  //   SHARP    (LPCR[6])     = 0  		:Disable Sharp signals
	  //   PCD      (LPCR[5:0])   = 000100  	:Pixel Clock = LSCLK = 33.25MHz/5 = 6.65MHz (DI demo board is @ 5.26MHz)
	  //                          = 000111  	:V and H timing calculated with Pixel Clock = LSCLK = 33.25MHz/8 = 4.15MHz, but less noise at higher LSCLK
	  LCDC_PCR = 0xCA080084;	
	  
	  // If we're not using a Sharp panel, clear this register to be safe
	  LCDC_LSCR1 = 0x00000000;
	  
	  // Configure horizontal LCD timing
	  LCDC_HCR = 0x48003F2F;
	  
	  // Configure vertical LCD timing
	  LCDC_VCR = 0x10000F0F;
	}
		
	// Setup DMA for main frame buffer
	//   BURST (LDCR[31])    = 0 :Dynamic burst length
	//   HM    (LDCR[20:16]) = 4 :DMA high mark is set to 4
	//   TM    (LDCR[4:0])   = 15 :Set DMA trigger mark to 15
	//LCDC_LDCR = 0x0004000F;
	LCDC_PWMR = 0x00A903C0;
	LCDC_RMCR = 0x0;
	LCDC_DMACR = 0x0;
	// end crap code

	return 0;
}

static int imxfbov_activate_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct imxfb_info *fbi = info->par;

	printk( "imxfbov_activate_var\n" );
	LCDC_LGWSR	= SIZE_XMAX(var->xres) | SIZE_YMAX(var->yres);
	LCDC_LGWDCR	= fbi->dmacr;
	LCDC_LGWPR      = 0;

	return 0;
}

static void imxfb_setup_gpio(struct imxfb_info *fbi)
{
	int width;
	
	printk( "imxfb_setup_gpio\n" );
	LCDC_RMCR	&= ~(RMCR_LCDC_EN | RMCR_SELF_REF);

	if( fbi->pcr & PCR_TFT )
		width = 16;
	else
		width = 1 << ((fbi->pcr >> 28) & 0x3);

#if defined(CONFIG_ARCH_IMX)
	switch(width) {
	case 16:
		imx_gpio_mode(PD30_PF_LD15);
		imx_gpio_mode(PD29_PF_LD14);
		imx_gpio_mode(PD28_PF_LD13);
		imx_gpio_mode(PD27_PF_LD12);
		imx_gpio_mode(PD26_PF_LD11);
		imx_gpio_mode(PD25_PF_LD10);
		imx_gpio_mode(PD24_PF_LD9);
		imx_gpio_mode(PD23_PF_LD8);
	case 8:
		imx_gpio_mode(PD22_PF_LD7);
		imx_gpio_mode(PD21_PF_LD6);
		imx_gpio_mode(PD20_PF_LD5);
		imx_gpio_mode(PD19_PF_LD4);
	case 4:
		imx_gpio_mode(PD18_PF_LD3);
		imx_gpio_mode(PD17_PF_LD2);
	case 2:
		imx_gpio_mode(PD16_PF_LD1);
	case 1:
		imx_gpio_mode(PD15_PF_LD0);
	}

	/* initialize GPIOs */
	imx_gpio_mode(PD6_PF_LSCLK);
	imx_gpio_mode(PD10_PF_SPL_SPR);
	imx_gpio_mode(PD11_PF_CONTRAST);
	imx_gpio_mode(PD14_PF_FLM_VSYNC);
	imx_gpio_mode(PD13_PF_LP_HSYNC);
	imx_gpio_mode(PD7_PF_REV);
	imx_gpio_mode(PD8_PF_CLS);

#ifndef CONFIG_MACH_PIMX1
	/* on PiMX1 used as buffers enable signal
	 */
	imx_gpio_mode(PD9_PF_PS);
#endif

#ifndef CONFIG_MACH_MX1FS2
	/* on mx1fs2 this pin is used to (de)activate the display, so we need
	 * it as a normal gpio
	 */
	imx_gpio_mode(PD12_PF_ACD_OE);
#endif
#else /* then defined(CONFIG_ARCH_IMX21) */
        if (fbi->max_bpp >= 18) {
               imx_gpio_mode(PA23_PF_LD17);
               imx_gpio_mode(PA22_PF_LD16);
        }
        if (fbi->max_bpp >= 16) {
               imx_gpio_mode(PA21_PF_LD15);
               imx_gpio_mode(PA20_PF_LD14);
               imx_gpio_mode(PA19_PF_LD13);
               imx_gpio_mode(PA18_PF_LD12);
               imx_gpio_mode(PA17_PF_LD11);
               imx_gpio_mode(PA16_PF_LD10);
               imx_gpio_mode(PA15_PF_LD9);
               imx_gpio_mode(PA14_PF_LD8);
        }
        if (fbi->max_bpp >= 8) {
               imx_gpio_mode(PA13_PF_LD7);
               imx_gpio_mode(PA12_PF_LD6);
               imx_gpio_mode(PA11_PF_LD5);
               imx_gpio_mode(PA10_PF_LD4);
        }
        if (fbi->max_bpp >= 4) {
               imx_gpio_mode(PA9_PF_LD3);
               imx_gpio_mode(PA8_PF_LD2);
        }
        if (fbi->max_bpp >= 2) {
               imx_gpio_mode(PA7_PF_LD1);
        }
        if (fbi->max_bpp >= 1) {
               imx_gpio_mode(PA6_PF_LD0);
        }

        /* initialize GPIOs */
        imx_gpio_mode(PA5_PF_LSCLK);
#ifndef CONFIG_MACH_CSB535
        imx_gpio_mode(PA27_PF_SPL_SPR);
        imx_gpio_mode(PA24_PF_REV);
        imx_gpio_mode(PA25_PF_CLS);

        /* Used for VEE control on CSB535 */
        imx_gpio_mode(PA30_PF_CONTRAST);

        imx_gpio_mode(PA26_PF_PS);
#endif

        imx_gpio_mode(PA29_PF_FLM_VSYNC);
        imx_gpio_mode(PA28_PF_LP_HSYNC);
        imx_gpio_mode(PA31_PF_ACD_OE);
        
#endif /* defined(CONFIG_ARCH_IMX) */

}

#ifdef CONFIG_PM
/*
 * Power management hooks.  Note that we won't be called from IRQ context,
 * unlike the blank functions above, so we may sleep.
 */
static int imxfb_suspend(struct platform_device *dev, pm_message_t state)
{
	struct imxfb_info *fbi = platform_get_drvdata(dev);
	printk("%s\n",__FUNCTION__);

	imxfb_disable_controller(fbi);
	return 0;
}

static int imxfb_resume(struct platform_device *dev)
{
	struct imxfb_info *fbi = platform_get_drvdata(dev);
	printk("%s\n",__FUNCTION__);

	imxfb_enable_controller(fbi);
	return 0;
}
#else
#define imxfb_suspend	NULL
#define imxfb_resume	NULL
#endif

static int __init imxfb_init_fbinfo(struct device *dev, struct fb_info *info)
{
	struct imxfb_mach_info *inf = dev->platform_data;
	struct imxfb_info *fbi = info->par;

	printk("%s\n",__FUNCTION__);

	info->pseudo_palette = kmalloc( sizeof(u32) * 16, GFP_KERNEL);
	if (!info->pseudo_palette)
		return -ENOMEM;

	memset(fbi, 0, sizeof(struct imxfb_info));
	fbi->dev = dev;

	strlcpy(info->fix.id, IMX_NAME, sizeof(info->fix.id));

	info->fix.type	= FB_TYPE_PACKED_PIXELS;
	info->fix.type_aux		= 0;
	info->fix.xpanstep		= 0;
	info->fix.ypanstep		= 0;
	info->fix.ywrapstep		= 0;
	info->fix.accel	= FB_ACCEL_NONE;

	info->var.nonstd		= 0;
	info->var.activate		= FB_ACTIVATE_NOW;
	info->var.height		= -1;
	info->var.width	= -1;
	info->var.accel_flags		= 0;
	info->var.vmode	= FB_VMODE_NONINTERLACED;

	info->fbops			= &imxfb_ops;
	info->flags			= FBINFO_FLAG_DEFAULT;
	info->pseudo_palette		= (fbi + 1);

	fbi->rgb[RGB_32]		= &def_rgb_32;
	fbi->rgb[RGB_16]		= &def_rgb_16;
	fbi->rgb[RGB_8]			= &def_rgb_8;

	fbi->max_xres			= inf->xres;
	info->var.xres			= inf->xres;
	info->var.xres_virtual		= inf->xres;
	fbi->max_yres			= inf->yres;
	info->var.yres			= inf->yres;
	info->var.yres_virtual		= inf->yres;
	fbi->max_bpp			= inf->bpp;
	info->var.bits_per_pixel	= inf->bpp;
	info->var.pixclock		= inf->pixclock;
	info->var.hsync_len		= inf->hsync_len;
	info->var.left_margin		= inf->left_margin;
	info->var.right_margin		= inf->right_margin;
	info->var.vsync_len		= inf->vsync_len;
	info->var.upper_margin		= inf->upper_margin;
	info->var.lower_margin		= inf->lower_margin;
	info->var.sync			= inf->sync;
	info->var.grayscale		= inf->cmap_greyscale;
	fbi->cmap_inverse		= inf->cmap_inverse;
	fbi->pcr			= inf->pcr;
	fbi->lscr1			= inf->lscr1;
	fbi->dmacr			= inf->dmacr;
	fbi->pwmr			= inf->pwmr;
	fbi->lcd_power			= inf->lcd_power;
	fbi->backlight_power		= inf->backlight_power;
	info->fix.smem_len		= fbi->max_xres * fbi->max_yres *
					  fbi->max_bpp / 8;

	return 0;
}

static int __init imxfbov_init_fbinfo(struct device *dev, struct fb_info *info)
{
	struct imxfb_mach_info *inf = dev->platform_data;
	struct imxfb_info *fbi = info->par;

	printk("%s\n",__FUNCTION__);

	info->pseudo_palette = kmalloc( sizeof(u32) * 16, GFP_KERNEL);
	if (!info->pseudo_palette)
		return -ENOMEM;

	memset(fbi, 0, sizeof(struct imxfb_info));
	fbi->dev = dev;

	strlcpy(info->fix.id, IMX_NAME, sizeof(info->fix.id));

	info->fix.type	= FB_TYPE_PACKED_PIXELS;
	info->fix.type_aux		= 0;
	info->fix.xpanstep		= 0;
	info->fix.ypanstep		= 0;
	info->fix.ywrapstep		= 0;
	info->fix.accel	= FB_ACCEL_NONE;

	info->var.nonstd		= 0;
	info->var.activate		= FB_ACTIVATE_NOW;
	info->var.height		= -1;
	info->var.width	= -1;
	info->var.accel_flags		= 0;
	info->var.vmode	= FB_VMODE_NONINTERLACED;

	info->fbops			= &imxfbov_ops;
	info->flags			= FBINFO_FLAG_DEFAULT;
	info->pseudo_palette		= (fbi + 1);

	fbi->rgb[RGB_32]		= &def_rgb_32;
	fbi->rgb[RGB_16]		= &def_rgb_16;
	fbi->rgb[RGB_8]			= &def_rgb_8;

	fbi->max_xres			= inf->xres;
	info->var.xres			= inf->xres;
	info->var.xres_virtual		= inf->xres;
	fbi->max_yres			= inf->yres;
	info->var.yres			= inf->yres;
	info->var.yres_virtual		= inf->yres;
	fbi->max_bpp			= inf->bpp;
	info->var.bits_per_pixel	= inf->bpp;
	info->var.pixclock		= inf->pixclock;
	info->var.hsync_len		= inf->hsync_len;
	info->var.left_margin		= inf->left_margin;
	info->var.right_margin		= inf->right_margin;
	info->var.vsync_len		= inf->vsync_len;
	info->var.upper_margin		= inf->upper_margin;
	info->var.lower_margin		= inf->lower_margin;
	info->var.sync			= inf->sync;
	info->var.grayscale		= inf->cmap_greyscale;
	fbi->cmap_inverse		= inf->cmap_inverse;
	fbi->pcr			= inf->pcr;
	fbi->lscr1			= inf->lscr1;
	fbi->dmacr			= inf->dmacr;
	fbi->pwmr			= inf->pwmr;
	fbi->lcd_power			= inf->lcd_power;
	fbi->backlight_power		= inf->backlight_power;
	info->fix.smem_len		= fbi->max_xres * fbi->max_yres *
					  fbi->max_bpp / 8;

	return 0;
}

/*
 *      Allocates the DRAM memory for the frame buffer.  This buffer is
 *	remapped into a non-cached, non-buffered, memory region to
 *      allow pixel writes to occur without flushing the cache.
 *      Once this area is remapped, all virtual memory access to the
 *      video memory should occur at the new region.
 */
static int __init imxfb_map_video_memory(struct fb_info *info)
{
	struct imxfb_info *fbi = info->par;

	fbi->map_size = PAGE_ALIGN(info->fix.smem_len);
	fbi->map_cpu = dma_alloc_writecombine(fbi->dev, fbi->map_size,
					&fbi->map_dma,GFP_KERNEL);

	if (fbi->map_cpu) {
		info->screen_base = fbi->map_cpu;
		fbi->screen_cpu = fbi->map_cpu;
		fbi->screen_dma = fbi->map_dma;
		info->fix.smem_start = fbi->screen_dma;
	}

	return fbi->map_cpu ? 0 : -ENOMEM;
}

static int __init imxfbov_map_video_memory(struct fb_info *info)
{
	struct imxfb_info *fbi = info->par;

	fbi->ov_map_size = PAGE_ALIGN(info->fix.smem_len);
	fbi->ov_map_cpu = dma_alloc_writecombine(fbi->dev, fbi->ov_map_size,
					&fbi->ov_map_dma,GFP_KERNEL);

	if (fbi->ov_map_cpu) {
		info->screen_base = fbi->ov_map_cpu;
		fbi->ov_screen_cpu = fbi->ov_map_cpu;
		fbi->ov_screen_dma = fbi->ov_map_dma;
		info->fix.smem_start = fbi->ov_screen_dma;
	}

	return fbi->ov_map_cpu ? 0 : -ENOMEM;
}

static int imxfb_proc_read_alpha(char *buf, char **start, off_t offset,
                                 int count, int *eof, void *data)
{
    int len;
    len = sprintf(buf, "0x%x\n", ((LCDC_LGWCR >> 24) & 0xff));
    *eof = 1;

    return len;
}

static int imxfb_proc_write_alpha(struct file *file, const char *buf,
                                  unsigned long count, void *data)
{
    unsigned long alpha;

    alpha = simple_strtoul(buf, NULL, 0);

    printk( "imxfb_proc_write_alpha\n" );
    LCDC_LGWCR = ((LCDC_LGWCR & ~(0xff << 24)) | LGWCR_ALPHA(alpha));
    
    return count;
}

static int imxfb_proc_read_enable(char *buf, char **start, off_t offset,
                                  int count, int *eof, void *data)
{
    int len;

    if (LCDC_LGWCR & LGWCR_GWE) {
        len = sprintf(buf, "1\n");
    } else {
        len = sprintf(buf, "0\n");
    }
    *eof = 1;

    return len;
}

static int imxfb_proc_write_enable(struct file *file, const char *buf,
                                   unsigned long count, void *data)
{
    unsigned long en;

    en = simple_strtoul(buf, NULL, 0);

    printk( "imxfb_proc_write_enable\n" );

    if (en) {
        uint32_t saveClock = PCCR0 & PCCR0_HCLK_LCDC_EN; // save state of HCLK
        PCCR0 &= ~ PCCR0_HCLK_LCDC_EN; // force HCLK off
	// msleep( 30 );
	
        LCDC_LGWCR |= LGWCR_GWE;
        PCCR0 |= saveClock; // restore the state of HCLK
    } else {
        LCDC_LGWCR &= ~LGWCR_GWE;
    }

    return count;
}

static int imxfb_proc_read_key(char *buf, char **start, off_t offset,
                               int count, int *eof, void *data)
{
    int len = 0;
    len += sprintf(buf + len, "0x%02x",   ((LCDC_LGWCR >> 12) & 0x3f));
    len += sprintf(buf + len,   "%02x",   ((LCDC_LGWCR >> 6) & 0x3f));
    len += sprintf(buf + len,   "%02x\n", ((LCDC_LGWCR >> 0) & 0x3f));
    *eof = 1;

    return len;
}

static int imxfb_proc_write_key(struct file *file, const char *buf,
                                unsigned long count, void *data)
{
    unsigned long key;
    int red;
    int green;
    int blue;
    uint32_t lgwcr = LCDC_LGWCR & ~(0x3ffff);

    key = simple_strtoul(buf, NULL, 0);

    red   = (key >> 16) & 0x3f;
    green = (key >>  8) & 0x3f;
    blue  = (key >>  0) & 0x3f;

    lgwcr |= ((red << 12) | (green << 6) | (blue));

    printk( "imxfb_proc_write_key %lx\n", (unsigned long) lgwcr );
    LCDC_LGWCR = lgwcr;
    
    return count;
}


static int imxfb_proc_read_key_en(char *buf, char **start, off_t offset,
                                  int count, int *eof, void *data)
{
    int len;

    if (LCDC_LGWCR & LGWCR_GWCKE) {
        len = sprintf(buf, "1\n");
    } else {
        len = sprintf(buf, "0\n");
    }
    *eof = 1;

    return len;
}

static int imxfb_proc_write_key_en(struct file *file, const char *buf,
                                   unsigned long count, void *data)
{
    unsigned long en;

    printk( "imxfb_proc_write_key_en\n" );
    en = simple_strtoul(buf, NULL, 0);

    if (en) {
        LCDC_LGWCR |= LGWCR_GWCKE;
    } else {
        LCDC_LGWCR &= ~LGWCR_GWCKE;
    }
    
    return count;
}

static void imxfb_proc_init(void)
{
    struct proc_dir_entry *pde;

    proc_mkdir("driver/imxfb", 0);

    pde = create_proc_read_entry("driver/imxfb/enable", 0, NULL, 
                                 imxfb_proc_read_enable, NULL);
    pde->write_proc = imxfb_proc_write_enable;

    pde = create_proc_read_entry("driver/imxfb/alpha", 0, NULL, 
                                 imxfb_proc_read_alpha, NULL);
    pde->write_proc = imxfb_proc_write_alpha;

    pde = create_proc_read_entry("driver/imxfb/key", 0, NULL, 
                                 imxfb_proc_read_key, NULL);
    pde->write_proc = imxfb_proc_write_key;

    pde = create_proc_read_entry("driver/imxfb/key_en", 0, NULL, 
                                 imxfb_proc_read_key_en, NULL);
    pde->write_proc = imxfb_proc_write_key_en;

}

static int __init imxfb_probe(struct platform_device *pdev)
{
	struct imxfb_info *fbi;
	struct fb_info *info;
	struct fb_info *ov_info;
	struct imxfb_mach_info *inf;
	struct resource *res;
	int ret;
	int i;

	printk("i.MX Framebuffer driver\n");

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(!res)
		return -ENODEV;

	inf = pdev->dev.platform_data;
	if(!inf) {
		dev_err(&pdev->dev,"No platform_data available\n");
		return -ENOMEM;
	}

	info = framebuffer_alloc(sizeof(struct imxfb_info), &pdev->dev);
	if(!info) {
		return -ENOMEM;
        }
	fbi = info->par;

	ov_info = framebuffer_alloc(0, &pdev->dev);
	if(!ov_info) {
            framebuffer_release(info);
            return -ENOMEM;
        }
        ov_info->par = fbi;
//FIXME
	platform_set_drvdata(pdev, info);

	ret = imxfb_init_fbinfo(&pdev->dev, info);
	if( ret < 0 )
		goto failed_init;

	ret = imxfbov_init_fbinfo(&pdev->dev, ov_info);
	if( ret < 0 )
		goto failed_init;

	res = request_mem_region(res->start, res->end - res->start + 1, "IMXFB");
	if (!res) {
		ret = -EBUSY;
		goto failed_regs;
	}

	if (!inf->fixed_screen_cpu) {
		ret = imxfb_map_video_memory(info);
		if (ret) {
			dev_err(&pdev->dev, "Failed to allocate video RAM: %d\n", ret);
			ret = -ENOMEM;
			goto failed_map;
		}
		ret = imxfbov_map_video_memory(ov_info);
		if (ret) {
			dev_err(&pdev->dev, "Failed to allocate video RAM: %d\n", ret);
			ret = -ENOMEM;
			goto failed_map;
		}
	} else {
		/* Fixed framebuffer mapping enables location of the screen in eSRAM */
		fbi->map_cpu = inf->fixed_screen_cpu;
		fbi->map_dma = inf->fixed_screen_dma;
		info->screen_base = fbi->map_cpu;
		fbi->screen_cpu = fbi->map_cpu;
		fbi->screen_dma = fbi->map_dma;
		info->fix.smem_start = fbi->screen_dma;

		fbi->ov_map_cpu = inf->fixed_screen_cpu;
		fbi->ov_map_dma = inf->fixed_screen_dma;
		ov_info->screen_base = fbi->ov_map_cpu;
		fbi->ov_screen_cpu = fbi->ov_map_cpu;
		fbi->ov_screen_dma = fbi->ov_map_dma;
		ov_info->fix.smem_start = fbi->ov_screen_dma;
	}

	/*
	 * This makes sure that our colour bitfield
	 * descriptors are correctly initialised.
	 */
	imxfb_check_var(&info->var, info);
 	imxfb_check_var(&info->var, ov_info);

//	ret = fb_alloc_cmap(&info->cmap, 1<<info->var.bits_per_pixel, 0);
	ret = fb_alloc_cmap(&info->cmap, 256, 0);
	if (ret < 0)
		goto failed_cmap;
 	ret = fb_alloc_cmap(&ov_info->cmap, 256, 0);
 	if (ret < 0)
 		goto failed_cmap;

 	imxfb_set_par(info);
 	imxfbov_set_par(info);

	ret = register_framebuffer(info);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register framebuffer\n");
		goto failed_register;
	}

 	ret = register_framebuffer(ov_info);
 	if (ret < 0) {
 		dev_err(&pdev->dev, "failed to register framebuffer\n");
 		goto failed_register;
 	}
 
	imxfb_enable_controller(fbi);
 	imxfbov_enable_controller(fbi);

	// doesn't port. I don't care.
	// LCDC frame buffer logo force setting code framebuffer -- bunnie
	//	for( i = 0; i < 0x40000; i++ ) {
	//	  *((unsigned char *)LCDC_SSA + i) = *((unsigned char *)0xC1000000 + i);
	//	}
	for( i = 0; i < 0x25800; i++ ) {
	  *((unsigned char *)LCDC_SSA + i) = *((unsigned char *)0xC2500000 + i);
	}
	// consider hoisting above the DMA setting code in imxfb_enable_controller

        imxfb_proc_init();
 	imxfb_setup_gpio(fbi);

        /* 
         * We have to disable the LCD clock to update the graphic window
         * settings.
         */
        // PCCR0 &= ~PCCR0_HCLK_LCDC_EN;
        // PCCR0 |= PCCR0_HCLK_LCDC_EN;

	return 0;

failed_register:
	fb_dealloc_cmap(&info->cmap);
 	fb_dealloc_cmap(&ov_info->cmap);
failed_cmap:
	if (!inf->fixed_screen_cpu) {
		dma_free_writecombine(&pdev->dev,fbi->map_size,fbi->map_cpu,
		           fbi->map_dma);
 		dma_free_writecombine(&pdev->dev,fbi->ov_map_size,fbi->ov_map_cpu,
 		           fbi->ov_map_dma);
         }
failed_map:
	kfree(info->pseudo_palette);
 	kfree(ov_info->pseudo_palette);
failed_regs:
	release_mem_region(res->start, res->end - res->start);
failed_init:
	platform_set_drvdata(pdev, NULL);
	framebuffer_release(info);
 	framebuffer_release(ov_info);
	return ret;
}

static int imxfb_remove(struct platform_device *pdev)
{
	struct fb_info *info = platform_get_drvdata(pdev);
	struct imxfb_info *fbi = info->par;
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	imxfb_disable_controller(fbi);

	unregister_framebuffer(info);

	fb_dealloc_cmap(&info->cmap);
	kfree(info->pseudo_palette);
	framebuffer_release(info);

	release_mem_region(res->start, res->end - res->start + 1);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

void  imxfb_shutdown(struct platform_device * dev)
{
	struct fb_info *info = platform_get_drvdata(dev);
	struct imxfb_info *fbi = info->par;
	imxfb_disable_controller(fbi);
}

static struct platform_driver imxfb_driver = {
	.probe		= imxfb_probe,
	.suspend	= imxfb_suspend,
	.resume		= imxfb_resume,
	.remove		= imxfb_remove,
	.shutdown	= imxfb_shutdown,
	.driver		= {
		.name	= "imx-fb",
	},
};

int __init imxfb_init(void)
{
	return platform_driver_register(&imxfb_driver);
}

static void __exit imxfb_cleanup(void)
{
	platform_driver_unregister(&imxfb_driver);
}

module_init(imxfb_init);
module_exit(imxfb_cleanup);

MODULE_DESCRIPTION("Motorola i.MX framebuffer driver");
MODULE_AUTHOR("Sascha Hauer, Pengutronix");
MODULE_LICENSE("GPL");
