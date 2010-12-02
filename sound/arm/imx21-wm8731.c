/*
 *  Driver for Wolfson WM8731 on i.MX21
 *  Copyright (C) 2006 Jay Monkman <jtm@lopingdog.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License.
 *
 *  NOTES:
 *    This driver configures the CODEC to generate the clock and frame
 *         signals.
 *
 *
 *  TODO:
 *     handle power management
 */

#include <linux/config.h>
#include <sound/driver.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/ioctl.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/dma.h>
#include <asm/arch/imx-dma.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/control.h>
#include <sound/initval.h>

#include <sound/wm8731.h>

#define WM8731_ADDR 0x1a


/***********************************************************************
 * Module stuff
 ***********************************************************************/
static char *id = NULL;
module_param(id, charp, 0444);

MODULE_AUTHOR("Jay Monkman <jtm@lopingdog.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("i.MX21 Audio driver (WM8731 CODEC)");
MODULE_PARM_DESC(id, "ID string for i.MX21/WM8731.");



/***********************************************************************
 * Data structures
 ***********************************************************************/
typedef struct {
    snd_card_t *card;
    snd_pcm_t *pcm;
    struct i2c_client *wm8731;

    imx_dmach_t p_dma;
    snd_pcm_substream_t *p_substream;
    int p_pos;
    u32 p_per_size;
    int p_period;

    imx_dmach_t c_dma;
    snd_pcm_substream_t *c_substream;
    int c_pos;
    u32 c_per_size;
    int c_period;
} imx21_chip_t;



/***********************************************************************
 * Function Prototypes
 ***********************************************************************/
static int wm8731_attach_adapter(struct i2c_adapter *adapter);
static int wm8731_detach_client(struct i2c_client *client);
static void wm8731_init_client(struct i2c_client *client);
static int wm8731_write_reg(struct i2c_client *client, u8 reg, u16 val);
static int wm8731_read_reg(struct i2c_client *client, u8 reg, u16 *val);


/***********************************************************************
 * Global Variables
 ***********************************************************************/
static snd_device_ops_t g_mixer_ops = {
    .dev_free = NULL,
};

/* The WM8731 is write only, so we mirror writes and fake read accesses */
static short wm8731_regs_g[WM8731_NUM_REGS];

struct i2c_client *imx21_wm8731_client = NULL;

static struct i2c_driver wm8731_driver = {
    .driver = {
        .name           = "Wolfson wm8731 CODEC driver",
    },
    .id             = I2C_DRIVERID_WM8731,
    .attach_adapter = wm8731_attach_adapter,
    .detach_client  = wm8731_detach_client,
};


static unsigned int rates[] = {8000, 48000};
static snd_pcm_hw_constraint_list_t hw_constraints_rates = {
    .count = sizeof(rates) / sizeof(rates[0]),
    .list  = rates,
    .mask  = 0,
};

static snd_pcm_hardware_t snd_imx21_playback_hw = {
    .info = (SNDRV_PCM_INFO_INTERLEAVED    |
             SNDRV_PCM_INFO_BLOCK_TRANSFER |
             SNDRV_PCM_INFO_MMAP           |
             SNDRV_PCM_INFO_MMAP_VALID),
    .formats = SNDRV_PCM_FMTBIT_S16_LE,
    .rates = (SNDRV_PCM_RATE_8000 |
              SNDRV_PCM_RATE_48000),
    .rate_min = 8000,
    .rate_max = 48000,
    .channels_min = 1,
    .channels_max = 2,
    .buffer_bytes_max = 0x10000,
    .period_bytes_min = 0x4000,
    .period_bytes_max = 0x8000,
    .periods_min = 1,
    .periods_max = 2,
};

static snd_pcm_hardware_t snd_imx21_capture_hw = {
    .info = (SNDRV_PCM_INFO_INTERLEAVED    |
             SNDRV_PCM_INFO_BLOCK_TRANSFER |
             SNDRV_PCM_INFO_MMAP           |
             SNDRV_PCM_INFO_MMAP_VALID),
    .formats = SNDRV_PCM_FMTBIT_S16_LE,
    .rates = (SNDRV_PCM_RATE_8000 |
              SNDRV_PCM_RATE_48000),
    .rate_min = 8000,
    .rate_max = 48000,
    .channels_min = 1,
    .channels_max = 2,
    .buffer_bytes_max = 0x10000,
    .period_bytes_min = 0x4000,
    .period_bytes_max = 0x8000,
    .periods_min = 1,
    .periods_max = 2,
};

/************************************************************
 * PCM Functions
 ************************************************************/
static int snd_imx21_playback_open(snd_pcm_substream_t *substream)
{
    int rc;
    imx21_chip_t *chip = snd_pcm_substream_chip(substream);
    snd_pcm_runtime_t *runtime = substream->runtime;

    runtime->hw = snd_imx21_playback_hw;
    chip->p_substream = substream;

    rc = snd_pcm_hw_constraint_list(runtime, 0, 
                                    SNDRV_PCM_HW_PARAM_RATE,
                                    &hw_constraints_rates);
    if (rc < 0) {
        printk("Error setting rates constraint\n");
        return rc;
    }
    chip->p_pos = 0;

    return 0;
}

static int snd_imx21_playback_close(snd_pcm_substream_t *substream)
{
    imx21_chip_t *chip = snd_pcm_substream_chip(substream);

    chip->p_substream = NULL;

    return 0;
}

static int snd_imx21_pcm_hw_params(snd_pcm_substream_t *substream,
                                    snd_pcm_hw_params_t *hw_params)
{
    return snd_pcm_lib_malloc_pages(substream,
                                    params_buffer_bytes(hw_params));
}


static int snd_imx21_pcm_hw_free(snd_pcm_substream_t *substream)
{
    return snd_pcm_lib_free_pages(substream);
}

static inline void setup_dma_xfer(int chan, u32 src, 
                                  u32 dest, int count)
{
    SAR(chan) = src;
    DAR(chan) = dest;
    CNTR(chan) = count;
}

static int snd_imx21_pcm_playback_prepare(snd_pcm_substream_t *substream)
{
    imx21_chip_t *chip = snd_pcm_substream_chip(substream);
    snd_pcm_runtime_t *runtime = substream->runtime;

    switch(runtime->rate) {
    case 8000:
        /* 16 bit, 8KHz, 24 samples/frame */
        SSI1_STCCR = (SSI_STCCR_WL(16) | SSI_STCCR_DC(23) | SSI_STCCR_PM(0));
        SSI1_SRCCR = (SSI_SRCCR_WL(16) | SSI_SRCCR_DC(23) | SSI_SRCCR_PM(0));
        SSI1_STMSK = ~((1 << 0) | (1 << 12));
        SSI1_SRMSK = ~((1 << 0) | (1 << 12));
        wm8731_write_reg(chip->wm8731, WM8731_REG_SMP, WM8731_SMP_SR(3));
        break;

    case 48000:
        /* 16 bit, 48KHz, 4 samples/frame */
        SSI1_STCCR = (SSI_STCCR_WL(16) | SSI_STCCR_DC(3) | SSI_STCCR_PM(0));
        SSI1_SRCCR = (SSI_SRCCR_WL(16) | SSI_SRCCR_DC(3) | SSI_SRCCR_PM(0));
        SSI1_STMSK = ~((1 << 0) | (1 << 2));
        SSI1_SRMSK = ~((1 << 0) | (1 << 2));
        wm8731_write_reg(chip->wm8731, WM8731_REG_SMP, WM8731_SMP_SR(0));
        break;

    default:
        printk("%s: Can't handle rate of %d\n", __FUNCTION__, runtime->rate);
        return -ENODEV;
        break;
    }

   /* Get physical address of data buffer */
    runtime->dma_addr = __pa(runtime->dma_area);

    chip->p_pos = 0;
    chip->p_period = 0;
    chip->p_per_size = frames_to_bytes(runtime, runtime->period_size);

    CCR(chip->p_dma) = (CCR_DMOD_FIFO |
                      CCR_SMOD_LINEAR |
                      CCR_DSIZ_16 |
                      CCR_SSIZ_32 |
                      CCR_REN);

    RSSR(chip->p_dma) = DMA_REQ_SSI1_TX0;
    BLR(chip->p_dma) = 4;
    BUCR(chip->p_dma) = 0;

    return 0;
}

static int snd_imx21_pcm_playback_trigger(snd_pcm_substream_t *substream, int cmd)
{
    imx21_chip_t *chip = snd_pcm_substream_chip(substream);
    snd_pcm_runtime_t *runtime = substream->runtime;

    switch(cmd) {
    case SNDRV_PCM_TRIGGER_START:

        /* If ALSA wants to give us two periods, set them both up  */

        while (CCR(chip->p_dma) & CCR_CEN) {
            printk("TRIGGER_START while DMA is already enabled\n");
        }
        setup_dma_xfer(chip->p_dma,
                       runtime->dma_addr, (u32)&SSI1_STX0_PHYS, 
                       chip->p_per_size);
        CCR(chip->p_dma) |= CCR_ACRPT;
        chip->p_period = 1; 
        SSI1_SCR |= SSI_SCR_TE;
        SSI1_SIER |= SSI_SIER_TDMAE;
        imx_dma_enable(chip->p_dma);

        if ((runtime->periods == 2)) {
            CCR(chip->p_dma) |= CCR_RPT;
            setup_dma_xfer(chip->p_dma, 
                           runtime->dma_addr + chip->p_per_size,
                           (u32)&SSI1_STX0_PHYS, 
                           chip->p_per_size);
            chip->p_period = 0;
        }

        break;
    case SNDRV_PCM_TRIGGER_STOP:
        imx_dma_disable(chip->p_dma);
        CCR(chip->p_dma) &= ~(CCR_RPT | CCR_ACRPT);
        SSI1_SIER &= ~SSI_SIER_TDMAE;
        SSI1_SCR &= ~SSI_SCR_TE;
        break;
    default:
        return -EINVAL;
        break;
    }

    return 0;
}

static snd_pcm_uframes_t snd_imx21_pcm_playback_pointer(
    snd_pcm_substream_t *substream)
{
    imx21_chip_t *chip = snd_pcm_substream_chip(substream);
    snd_pcm_runtime_t *runtime = substream->runtime;
    unsigned int offset;

 
    /* current_ptr = imx21_get_hw_point/home/jtm/er(chip) */
    offset = bytes_to_frames(runtime, 
                                 chip->p_pos % (2 * chip->p_per_size));
    if (offset >= runtime->buffer_size) {
        offset = 0;
    }
    return offset;

}


static int snd_imx21_capture_open(snd_pcm_substream_t *substream)
{
    int rc;
    imx21_chip_t *chip = snd_pcm_substream_chip(substream);
    snd_pcm_runtime_t *runtime = substream->runtime;

    runtime->hw = snd_imx21_capture_hw;
    chip->c_substream = substream;

    rc = snd_pcm_hw_constraint_list(runtime, 0, 
                                    SNDRV_PCM_HW_PARAM_RATE,
                                    &hw_constraints_rates);
    if (rc < 0) {
        printk("Error setting rates constraint\n");
        return rc;
    }
    chip->c_pos = 0;

    return 0;
}

static int snd_imx21_capture_close(snd_pcm_substream_t *substream)
{
    imx21_chip_t *chip = snd_pcm_substream_chip(substream);

    chip->c_substream = NULL;

    return 0;
}

static int snd_imx21_pcm_capture_prepare(snd_pcm_substream_t *substream)
{
    imx21_chip_t *chip = snd_pcm_substream_chip(substream);
    snd_pcm_runtime_t *runtime = substream->runtime;

    switch(runtime->rate) {
    case 8000:
        /* 16 bit, 8KHz, 24 samples/frame */
        SSI1_STCCR = (SSI_STCCR_WL(16) | SSI_STCCR_DC(23) | SSI_STCCR_PM(0));
        SSI1_SRCCR = (SSI_SRCCR_WL(16) | SSI_SRCCR_DC(23) | SSI_SRCCR_PM(0));
        SSI1_STMSK = ~((1 << 0) | (1 << 12));
        SSI1_SRMSK = ~((1 << 0) | (1 << 12));
        wm8731_write_reg(chip->wm8731, WM8731_REG_SMP, WM8731_SMP_SR(3));
        break;

    case 48000:
        /* 16 bit, 48KHz, 4 samples/frame */
        SSI1_STCCR = (SSI_STCCR_WL(16) | SSI_STCCR_DC(3) | SSI_STCCR_PM(0));
        SSI1_SRCCR = (SSI_SRCCR_WL(16) | SSI_SRCCR_DC(3) | SSI_SRCCR_PM(0));
        SSI1_STMSK = ~((1 << 0) | (1 << 2));
        SSI1_SRMSK = ~((1 << 0) | (1 << 2));
        wm8731_write_reg(chip->wm8731, WM8731_REG_SMP, WM8731_SMP_SR(0));
        break;

    default:
        printk("%s: Can't handle rate of %d\n", __FUNCTION__, runtime->rate);
        return -ENODEV;
        break;
    }

   /* Get physical address of data buffer */
    runtime->dma_addr = __pa(runtime->dma_area);

    chip->c_pos = 0;
    chip->c_period = 0;
    chip->c_per_size = frames_to_bytes(runtime, runtime->period_size);

    CCR(chip->c_dma) = (CCR_SMOD_FIFO |
                      CCR_DMOD_LINEAR |
                      CCR_SSIZ_16 |
                      CCR_DSIZ_32 |
                      CCR_REN);

    RSSR(chip->c_dma) = DMA_REQ_SSI1_RX0;
    BLR(chip->c_dma) = 4;
    BUCR(chip->c_dma) = 0;

    return 0;
}

static int snd_imx21_pcm_capture_trigger(snd_pcm_substream_t *substream, int cmd)
{
    imx21_chip_t *chip = snd_pcm_substream_chip(substream);
    snd_pcm_runtime_t *runtime = substream->runtime;

    switch(cmd) {
    case SNDRV_PCM_TRIGGER_START:

        /* If ALSA wants to give us two periods, set them both up  */

        while (CCR(chip->c_dma) & CCR_CEN) {
            printk("TRIGGER_START while DMA is already enabled\n");
        }
        setup_dma_xfer(chip->c_dma,
                       (u32)&SSI1_SRX0_PHYS, runtime->dma_addr, 
                       chip->c_per_size);
        CCR(chip->c_dma) |= CCR_ACRPT;
        chip->c_period = 1; 
        SSI1_SCR |= SSI_SCR_RE;
        SSI1_SIER |= SSI_SIER_RDMAE;
        imx_dma_enable(chip->c_dma);

        if ((runtime->periods == 2)) {
            CCR(chip->c_dma) |= CCR_RPT;
            setup_dma_xfer(chip->c_dma, 
                           (u32)&SSI1_SRX0_PHYS, 
                           runtime->dma_addr + chip->c_per_size,
                           chip->c_per_size);
            chip->c_period = 0;
        }

        break;
    case SNDRV_PCM_TRIGGER_STOP:
        imx_dma_disable(chip->c_dma);
        CCR(chip->c_dma) &= ~(CCR_RPT | CCR_ACRPT);
        SSI1_SIER &= ~SSI_SIER_RDMAE;
        SSI1_SCR &= ~SSI_SCR_RE;
        break;
    default:
        return -EINVAL;
        break;
    }

    return 0;
}

static snd_pcm_uframes_t snd_imx21_pcm_capture_pointer(
    snd_pcm_substream_t *substream)
{
    imx21_chip_t *chip = snd_pcm_substream_chip(substream);
    snd_pcm_runtime_t *runtime = substream->runtime;
    unsigned int offset;

 
    /* current_ptr = imx21_get_hw_point/home/jtm/er(chip) */
    offset = bytes_to_frames(runtime, 
                             chip->c_pos % (2 * chip->c_per_size));
    if (offset >= runtime->buffer_size) {
        offset = 0;
    }
    return offset;

}

static snd_pcm_ops_t snd_imx21_playback_ops = {
    .open = snd_imx21_playback_open,
    .close = snd_imx21_playback_close,
    .ioctl = snd_pcm_lib_ioctl,
    .hw_params = snd_imx21_pcm_hw_params,
    .hw_free = snd_imx21_pcm_hw_free,
    .prepare = snd_imx21_pcm_playback_prepare,
    .trigger = snd_imx21_pcm_playback_trigger,
    .pointer = snd_imx21_pcm_playback_pointer,
};

static snd_pcm_ops_t snd_imx21_capture_ops = {
    .open = snd_imx21_capture_open,
    .close = snd_imx21_capture_close,
    .ioctl = snd_pcm_lib_ioctl,
    .hw_params = snd_imx21_pcm_hw_params,
    .hw_free = snd_imx21_pcm_hw_free,
    .prepare = snd_imx21_pcm_capture_prepare,
    .trigger = snd_imx21_pcm_capture_trigger,
    .pointer = snd_imx21_pcm_capture_pointer,
};

static int __init snd_imx21_pcm_new(imx21_chip_t *chip)
{
    snd_pcm_t *pcm;
    int rc;


    rc = snd_pcm_new(chip->card, "i.MX21 PCM", 0, 1, 1, &pcm);
    if (rc < 0) {
        return rc;
    }

    pcm->private_data = chip;
    strcpy(pcm->name, "i.MX21 PCM");
    chip->pcm = pcm;

    snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_CONTINUOUS,
                                          snd_dma_continuous_data(GFP_KERNEL),
                                          64*1024, 64*1024);

    snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK,
                    &snd_imx21_playback_ops);

    snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE,
                    &snd_imx21_capture_ops);

    return 0;
}
 

/************************************************************
 * Mixer Functions
 ************************************************************/
static int snd_imx21_mixer_mvol_info(snd_kcontrol_t *kcontrol,
                                      snd_ctl_elem_info_t *uinfo)
{
    uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
    uinfo->count = 2;
    uinfo->value.integer.min = 0;
    uinfo->value.integer.max = 127;
    return 0;
}

static int snd_imx21_mixer_mvol_get(snd_kcontrol_t *kcontrol,
                                     snd_ctl_elem_value_t *uctrl)
{
    imx21_chip_t *chip = snd_kcontrol_chip(kcontrol);
    u16 val;

    wm8731_read_reg(chip->wm8731, WM8731_REG_LHO, &val);
    uctrl->value.integer.value[0] = val & 0x7f;

    wm8731_read_reg(chip->wm8731, WM8731_REG_RHO, &val);
    uctrl->value.integer.value[1] = val & 0x7f;

    return 0;
    
}

static int snd_imx21_mixer_mvol_put(snd_kcontrol_t *kcontrol,
                                snd_ctl_elem_value_t *uctrl)
{
    imx21_chip_t *chip = snd_kcontrol_chip(kcontrol);
    int changed = 0;
    u16 val;
    int lvol;
    int rvol;


    wm8731_read_reg(chip->wm8731, WM8731_REG_LHO, &val);
    lvol = val & 0x7f;

    wm8731_read_reg(chip->wm8731, WM8731_REG_RHO, &val);
    rvol = val & 0x7f;

    if ((uctrl->value.integer.value[0] != lvol) || 
        (uctrl->value.integer.value[1] != rvol)) {
        changed = 1;

        lvol = uctrl->value.integer.value[0];
        rvol = uctrl->value.integer.value[1];
    }

    if (lvol == rvol) {
        val = lvol | (1 << 8);
        wm8731_write_reg(chip->wm8731, WM8731_REG_LHO, val);
    } else {
        wm8731_write_reg(chip->wm8731, WM8731_REG_LHO, lvol);
        wm8731_write_reg(chip->wm8731, WM8731_REG_RHO, rvol);
    }

    return changed;
}

static int snd_imx21_mixer_insel_info(snd_kcontrol_t *kcontrol,
                                       snd_ctl_elem_info_t *uinfo)
{
    uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
    uinfo->count = 1;
    uinfo->value.integer.min = 0;
    uinfo->value.integer.max = 1;
    return 0;
}

static int snd_imx21_mixer_insel_get(snd_kcontrol_t *kcontrol,
                                      snd_ctl_elem_value_t *uctrl)
{
    imx21_chip_t *chip = snd_kcontrol_chip(kcontrol);
    u16 val;

    wm8731_read_reg(chip->wm8731, WM8731_REG_AAP, &val);
    if (val & WM8731_AAP_INSEL) {
        uctrl->value.integer.value[0] = 1;
    } else {
        uctrl->value.integer.value[0] = 0;
    }

    return 0;
    
}

static int snd_imx21_mixer_insel_put(snd_kcontrol_t *kcontrol,
                                snd_ctl_elem_value_t *uctrl)
{
    imx21_chip_t *chip = snd_kcontrol_chip(kcontrol);
    int changed;
    u16 val;

    wm8731_read_reg(chip->wm8731, WM8731_REG_AAP, &val);
    if ((val & WM8731_AAP_INSEL) && uctrl->value.integer.value[0]) {
        changed = 0;
    } else {
        changed = 1;
    }

    if (uctrl->value.integer.value[0]) {
        val |= WM8731_AAP_INSEL;
    } else {
        val &= ~WM8731_AAP_INSEL;
    }
    
    wm8731_write_reg(chip->wm8731, WM8731_REG_AAP, val);

    return changed;
}

static int snd_imx21_mixer_micboost_info(snd_kcontrol_t *kcontrol,
                                          snd_ctl_elem_info_t *uinfo)
{
    uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
    uinfo->count = 1;
    uinfo->value.integer.min = 0;
    uinfo->value.integer.max = 1;
    return 0;
}

static int snd_imx21_mixer_micboost_get(snd_kcontrol_t *kcontrol,
                                         snd_ctl_elem_value_t *uctrl)
{
    imx21_chip_t *chip = snd_kcontrol_chip(kcontrol);
    u16 val;

    wm8731_read_reg(chip->wm8731, WM8731_REG_AAP, &val);
    if (val & WM8731_AAP_MICBOOST) {
        uctrl->value.integer.value[0] = 1;
    } else {
        uctrl->value.integer.value[0] = 0;
    }

    return 0;
    
}

static int snd_imx21_mixer_micboost_put(snd_kcontrol_t *kcontrol,
                                snd_ctl_elem_value_t *uctrl)
{
    imx21_chip_t *chip = snd_kcontrol_chip(kcontrol);
    int changed;
    u16 val;

    wm8731_read_reg(chip->wm8731, WM8731_REG_AAP, &val);
    if ((val & WM8731_AAP_MICBOOST) && uctrl->value.integer.value[0]) {
        changed = 0;
    } else {
        changed = 1;
    }

    if (uctrl->value.integer.value[0]) {
        val |= WM8731_AAP_MICBOOST;
    } else {
        val &= ~WM8731_AAP_MICBOOST;
    }
    
    wm8731_write_reg(chip->wm8731, WM8731_REG_AAP, val);

    return changed;
}

static int snd_imx21_mixer_sidetone_info(snd_kcontrol_t *kcontrol,
                                          snd_ctl_elem_info_t *uinfo)
{
    uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
    uinfo->count = 1;
    uinfo->value.integer.min = 0;
    uinfo->value.integer.max = 1;
    return 0;
}

static int snd_imx21_mixer_sidetone_get(snd_kcontrol_t *kcontrol,
                                         snd_ctl_elem_value_t *uctrl)
{
    imx21_chip_t *chip = snd_kcontrol_chip(kcontrol);
    u16 val;

    wm8731_read_reg(chip->wm8731, WM8731_REG_AAP, &val);
    if (val & WM8731_AAP_SIDETONE) {
        uctrl->value.integer.value[0] = 1;
    } else {
        uctrl->value.integer.value[0] = 0;
    }
    return 0;
    
}

static int snd_imx21_mixer_sidetone_put(snd_kcontrol_t *kcontrol,
                                snd_ctl_elem_value_t *uctrl)
{
    imx21_chip_t *chip = snd_kcontrol_chip(kcontrol);
    int changed;
    u16 val;

    wm8731_read_reg(chip->wm8731, WM8731_REG_AAP, &val);
    if ((val & WM8731_AAP_SIDETONE) && uctrl->value.integer.value[0]) {
        changed = 0;
    } else {
        changed = 1;
    }

    if (uctrl->value.integer.value[0]) {
        val |= WM8731_AAP_SIDETONE;
    } else {
        val &= ~WM8731_AAP_SIDETONE;
    }
    
    wm8731_write_reg(chip->wm8731, WM8731_REG_AAP, val);

    return changed;
}
static int snd_imx21_mixer_bypass_info(snd_kcontrol_t *kcontrol,
                                        snd_ctl_elem_info_t *uinfo)
{
    uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
    uinfo->count = 1;
    uinfo->value.integer.min = 0;
    uinfo->value.integer.max = 1;
    return 0;
}

static int snd_imx21_mixer_bypass_get(snd_kcontrol_t *kcontrol,
                                       snd_ctl_elem_value_t *uctrl)
{
    imx21_chip_t *chip = snd_kcontrol_chip(kcontrol);
    u16 val;

    wm8731_read_reg(chip->wm8731, WM8731_REG_AAP, &val);
    if (val & WM8731_AAP_BYPASS) {
        uctrl->value.integer.value[0] = 1;
    } else {
        uctrl->value.integer.value[0] = 0;
    }
    return 0;
    
}

static int snd_imx21_mixer_bypass_put(snd_kcontrol_t *kcontrol,
                                snd_ctl_elem_value_t *uctrl)

{
    imx21_chip_t *chip = snd_kcontrol_chip(kcontrol);
    int changed;
    u16 val;

    wm8731_read_reg(chip->wm8731, WM8731_REG_AAP, &val);
    if ((val & WM8731_AAP_BYPASS) && uctrl->value.integer.value[0]) {
        changed = 0;
    } else {
        changed = 1;
    }

    if (uctrl->value.integer.value[0]) {
        val |= WM8731_AAP_BYPASS;
    } else {
        val &= ~WM8731_AAP_BYPASS;
    }
    
    wm8731_write_reg(chip->wm8731, WM8731_REG_AAP, val);

    return changed;
}
static snd_kcontrol_new_t imx21_mixer_controls[] = {
    {
        .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
        .name  = "Master Playback Volume",
        .info  = snd_imx21_mixer_mvol_info,
        .get   = snd_imx21_mixer_mvol_get,
        .put   = snd_imx21_mixer_mvol_put,
    }, {
        .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
        .name  = "Input Select Switch",
        .info  = snd_imx21_mixer_insel_info,
        .get   = snd_imx21_mixer_insel_get,
        .put   = snd_imx21_mixer_insel_put,
    }, {
        .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
        .name  = "Mic Boost",
        .info  = snd_imx21_mixer_micboost_info,
        .get   = snd_imx21_mixer_micboost_get,
        .put   = snd_imx21_mixer_micboost_put,
    }, {
        .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
        .name  = "Sidetone Switch",
        .info  = snd_imx21_mixer_sidetone_info,
        .get   = snd_imx21_mixer_sidetone_get,
        .put   = snd_imx21_mixer_sidetone_put,
    }, {
        .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
        .name  = "Bypass Switch",
        .info  = snd_imx21_mixer_bypass_info,
        .get   = snd_imx21_mixer_bypass_get,
        .put   = snd_imx21_mixer_bypass_put,
    },
};
    
    
static int __init snd_imx21_mixer_new(imx21_chip_t *chip)
{
    int rc;
    snd_card_t *card = chip->card;
    int i;

    rc = snd_device_new(card, 
                        SNDRV_DEV_CODEC, 
                        imx21_wm8731_client, 
                        &g_mixer_ops);

    for (i = 0; 
         i < (sizeof(imx21_mixer_controls)/sizeof(imx21_mixer_controls[0]));
         i++) {
        rc = snd_ctl_add(card, snd_ctl_new1(&imx21_mixer_controls[i], chip));
        if (rc < 0) {
            return rc;
        }
    }
    return rc;
}



/************************************************************
 * Misc Functions
 ************************************************************/
static void imx21_dma_play_isr(int irq, void *data, struct pt_regs *regs)
{
    imx21_chip_t *chip = (imx21_chip_t *)data;
    snd_pcm_substream_t *substream = chip->p_substream;
    snd_pcm_runtime_t *runtime = substream->runtime;
    u32 cntr;

    if (DBTOSR != 0) {
        printk("DMA Error: DBTOSR = 0x%x\n", DBTOSR);
    }

    if (DRTOSR != 0) {
        printk("DMA Error: DRTOSR = 0x%x\n", DRTOSR);
    }

    if (DSESR != 0) {
        printk("DMA Error: DSESR = 0x%x\n", DSESR);
    }

    if (DBOSR != 0) {
        printk("DMA Error: DBOSR = 0x%x\n", DBOSR);
    }


    
    cntr = CNTR(chip->p_dma);

    chip->p_pos += (cntr);

    setup_dma_xfer(chip->p_dma, 
                   runtime->dma_addr + (chip->p_period * chip->p_per_size),
                   (u32)&SSI1_STX0_PHYS,
                   chip->p_per_size);
    CCR(chip->p_dma) |= CCR_RPT;
    chip->p_period = (chip->p_period + 1) & 0x1;

    snd_pcm_period_elapsed(substream);

}
    
static void imx21_dma_capture_isr(int irq, void *data, struct pt_regs *regs)
{
    imx21_chip_t *chip = (imx21_chip_t *)data;
    snd_pcm_substream_t *substream = chip->c_substream;
    snd_pcm_runtime_t *runtime = substream->runtime;
    u32 cntr;

    if (DBTOSR != 0) {
        printk("DMA Error: DBTOSR = 0x%x\n", DBTOSR);
    }

    if (DRTOSR != 0) {
        printk("DMA Error: DRTOSR = 0x%x\n", DRTOSR);
    }

    if (DSESR != 0) {
        printk("DMA Error: DSESR = 0x%x\n", DSESR);
    }

    if (DBOSR != 0) {
        printk("DMA Error: DBOSR = 0x%x\n", DBOSR);
    }


    
    cntr = CNTR(chip->c_dma);

    chip->c_pos += (cntr);

    setup_dma_xfer(chip->c_dma, 
                   (u32)&SSI1_SRX0_PHYS,
                   runtime->dma_addr + (chip->c_period * chip->c_per_size),
                   chip->c_per_size);
    CCR(chip->c_dma) |= CCR_RPT;
    chip->c_period = (chip->c_period + 1) & 0x1;

    snd_pcm_period_elapsed(substream);

}
    
static int __init imx21_wm8731_init(void)
{
    int rc = -ENODEV;
    snd_card_t *card;
    imx21_chip_t *chip;

    /* Make sure we have a CODEC */
    rc = i2c_add_driver(&wm8731_driver);
    if (rc != 0) {
        printk(KERN_ERR "Cannot register I2C driver for WM8731\n");
        return rc;
    }

    /* Register the sound card */
    card = snd_card_new(-1, id, THIS_MODULE, sizeof(imx21_chip_t));
    if (card == NULL) {
        return -ENOMEM;
    }

    chip = (imx21_chip_t *)card->private_data;

    chip->card = card;
    chip->wm8731 = imx21_wm8731_client;

    /*
     * Configure the Audio Mux:
     *   Use SSI1 controller - Host port 2 - port #1
     *   RX: Use SS1 pins - periph port 1 - port #3
     *   TX: Use SS2 pins - periph port 2 - port #4
     *
     *   Host port:
     *     connect RXCLK/TXCLK, RXFS/TXFS - synchronous mode
     *     clk and fs are outputs to SSI
     *
     *   Peripheral port:
     *     connect RXCLK/TXCLK, RXFS/TXFS - synchronous mode
     *     clk and fs are inputs from the pins
     */
    AUDMUX_HPCR1 = (
        AUDMUX_HPCR_TFSDIR    |
        AUDMUX_HPCR_TCLKDIR   |
        AUDMUX_HPCR_RFSDIR    |
        AUDMUX_HPCR_RCLKDIR   |
        AUDMUX_HPCR_TFCSEL(4) |   /* Peripheral port 2 */
        AUDMUX_HPCR_RFCSEL(3) |   /* Peripheral port 1 */
        AUDMUX_HPCR_RXDSEL(3)     /* Peripheral port 1 */
        );

    AUDMUX_PPCR1 = (
        AUDMUX_PPCR_TFCSEL(0) |   /* Host port 1 */
        AUDMUX_PPCR_RFCSEL(0) |   /* Host port 1 */
        0);

    AUDMUX_PPCR2 = (
        AUDMUX_PPCR_TFCSEL(0) |   /* Host port 1 */
        AUDMUX_PPCR_RFCSEL(0) |   /* Host port 1 */
        0);

    /* Setup GPIO */
    imx_gpio_mode(PC23_PF_SSI1_CLK);
    imx_gpio_mode(PC22_PF_SSI1_TX);
    imx_gpio_mode(PC21_PF_SSI1_RX);
    imx_gpio_mode(PC20_PF_SSI1_FS);
    imx_gpio_mode(PC27_PF_SSI2_CLK);
    imx_gpio_mode(PC26_PF_SSI2_TX);
    imx_gpio_mode(PC25_PF_SSI2_RX);
    imx_gpio_mode(PC24_PF_SSI2_FS);

    /* Enable the clock to SSI1 */
    PCCR0 |= (PCCR0_SSI1_BAUD_EN | PCCR0_SSI1_EN);


    if (imx_dma_request_by_prio(&chip->p_dma,
                                "i.MX21-ALSA", 
                                DMA_PRIO_HIGH) < 0) {
        goto init_err0;
    }
    imx_dma_setup_handlers(chip->p_dma, imx21_dma_play_isr, 
                           NULL, chip);

    if (imx_dma_request_by_prio(&chip->c_dma,
                                "i.MX21-ALSA", 
                                DMA_PRIO_HIGH) < 0) {
        goto init_err0;
    }
    imx_dma_setup_handlers(chip->c_dma, imx21_dma_capture_isr, 
                           NULL, chip);

    SSI1_SOR = SSI_SOR_INIT;
    udelay(1000);
    SSI1_SOR = 0;

    SSI1_SCR  = (SSI_SCR_I2S_MODE_NORM |
                 SSI_SCR_SYN | 
                 SSI_SCR_NET | 
                 SSI_SCR_SSIEN);

    SSI1_STCR  = (SSI_STCR_TXBIT0 |
                  SSI_STCR_TFEN0 |
                  SSI_STCR_TSCKP |
                  0 );

    SSI1_SRCR  = (SSI_SRCR_RXBIT0 |
                  SSI_SRCR_RFEN0 |
                  SSI_SRCR_RSCKP |
                  0 );


    SSI1_SFCSR = ((SSI1_SFCSR & ~0xff) | 
                  (SSI_SFCSR_RFWM0(6) |
                   SSI_SFCSR_TFWM0(4)));

    rc = snd_imx21_mixer_new(chip);
    if (rc != 0) {
        goto init_err1;
    }

    rc = snd_imx21_pcm_new(chip);
    if (rc != 0) {
        goto init_err1;
    }

    rc = snd_card_register(card);
    if (rc != 0) {
        goto init_err1;
    }


    wm8731_write_reg(chip->wm8731, WM8731_REG_LLI,
                     (WM8731_LIN_VOL(0x18) | WM8731_LIN_BOTH));
    wm8731_write_reg(chip->wm8731, WM8731_REG_LHO,
                     (WM8731_HPO_VOL(0x60) | WM8731_HPO_BOTH));
    wm8731_write_reg(chip->wm8731, WM8731_REG_AAP,
                     (
                      WM8731_AAP_MICBOOST |
//                      WM8731_AAP_BYPASS | 
//                      WM8731_AAP_SIDETONE | 
                      WM8731_AAP_DACSEL | 
                      WM8731_AAP_INSEL));
    wm8731_write_reg(chip->wm8731, WM8731_REG_DAP, 0);
                     
    /* FIXME: Should turn features off, until needed */
    wm8731_write_reg(chip->wm8731, WM8731_REG_PDC, 0);

    wm8731_write_reg(chip->wm8731, WM8731_REG_DAI,
                     (WM8731_DAI_FMT_LEFT | 
                      WM8731_DAI_IWL16 | 
                      WM8731_DAI_MASTER |
                      0 ));

                     
    wm8731_write_reg(chip->wm8731, WM8731_REG_ACT, 1);

    printk(KERN_INFO "i.MX21 WM8731 audio support initialized\n");

    return rc;

 init_err1:
    imx_dma_free(chip->p_dma);

 init_err0:
    snd_card_free(card);

    return rc;
}

static void __exit imx21_wm8731_exit(void)
{
    return;
}


module_init(imx21_wm8731_init);
module_exit(imx21_wm8731_exit);


static int wm8731_write_reg(struct i2c_client *client, u8 reg, u16 val)
{
    u8 d0;
    u8 d1;
    int rc;

    if (reg & 0x80) {
        return -1;
    }

    d0 = reg << 1 | ((val >> 8) & 1);
    d1 = val & 0xff;

    rc = i2c_smbus_write_byte_data(client, d0, d1);

    if (rc == 0) {
        /* Successful write, mirror the data */

        /* 
         * Handle the L/R volume registers which can be both be set
         * at the same time.
         */
        if ((reg == WM8731_REG_LLI) || (reg == WM8731_REG_RLI)) {
            if (val & WM8731_LIN_BOTH) {
                wm8731_regs_g[WM8731_REG_LLI] = val & ~WM8731_LIN_BOTH;
                wm8731_regs_g[WM8731_REG_RLI] = val & ~WM8731_LIN_BOTH;
            }
        } else if ((reg == WM8731_REG_LHO) || (reg == WM8731_REG_RHO)) {
            if (val & WM8731_HPO_BOTH) {
                wm8731_regs_g[WM8731_REG_LHO] = val & ~WM8731_HPO_BOTH;
                wm8731_regs_g[WM8731_REG_RHO] = val & ~WM8731_HPO_BOTH;
            }
        } else {
            wm8731_regs_g[reg] = val;
        }
    }

    return rc;
}

static int wm8731_read_reg(struct i2c_client *client, u8 reg, u16 *val)
{
    if (reg & 0x80) {
        return -1;
    }

    *val = wm8731_regs_g[reg];
    return 0;
}


/* Probe for devices */
static int wm8731_attach_adapter(struct i2c_adapter *adapter)
{
    int err;

    struct i2c_client *new_client;
    new_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL);
    if (new_client == NULL) {
        return -ENOMEM;
    }

    new_client->adapter = adapter;
    new_client->addr = WM8731_ADDR;
    new_client->driver = &wm8731_driver;
    new_client->flags = 0;

    if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
        goto err1;
    }

    /* Fill in the remaining client fields and put it into the global list */
    strlcpy(new_client->name, "wm8731", I2C_NAME_SIZE);
        
    /* Tell the I2C layer a new client has arrived */
    err = i2c_attach_client(new_client);
    if (err != 0) {
        goto err1;
    }

    /* Initialize the WM8731 chip */
    wm8731_init_client(new_client);

    return 0;

 err1:
    kfree(new_client);

    return -ENODEV;
}

static int wm8731_detach_client(struct i2c_client *client)
{
    int err;

    if ((err = i2c_detach_client(client))) {
        dev_err(&client->dev,
                "Client deregistration failed, client not detached.\n");
        return err;
    }

    return 0;
}

/* Called when we have found a new WM8731. */
static void wm8731_init_client(struct i2c_client *client)
{
    /*
     * Write default values to registers, mainly to initialize 
     * mirror of them.
     */
    wm8731_write_reg(client, 15, 0x00);   /* Reset */
    udelay(1000);
    
    wm8731_write_reg(client,  0, 0x97);   /* Left line in */
    wm8731_write_reg(client,  1, 0x97);   /* Right line in */
    wm8731_write_reg(client,  2, 0x79);   /* Left headphone out */
    wm8731_write_reg(client,  3, 0x79);   /* Right headphone out */
    wm8731_write_reg(client,  4, 0x0a);   /* Analog audio path ctrl */
    wm8731_write_reg(client,  5, 0x08);   /* Digital audio path ctrl */
    wm8731_write_reg(client,  6, 0x9f);   /* Power down */
    wm8731_write_reg(client,  7, 0x0a);   /* Digital audio iface fmt */
    wm8731_write_reg(client,  8, 0x00);   /* Sampling ctrl */
    wm8731_write_reg(client,  9, 0x00);   /* Active ctrl */

    imx21_wm8731_client = client;
}

