#ifndef __CHUMBY_UDMA_H__
#define __CHUMBY_UDMA_H__

// cloned from imx-regs.h (for user space)

#define DCR_DAM            (1<<2)  /* On IMX21 only */
#define DCR_DRST           (1<<1)
#define DCR_DEN            (1<<0)
#define DBTOCR_EN          (1<<15)
#define DBTOCR_CNT(x)      ((x) & 0x7fff )
#define CNTR_CNT(x)        ((x) & 0xffffff )
#define CCR_ACRPT          ( 0x1 << 14 )  /* On IMX21 only */
#define CCR_DMOD_LINEAR    ( 0x0 << 12 )
#define CCR_DMOD_2D        ( 0x1 << 12 )
#define CCR_DMOD_FIFO      ( 0x2 << 12 )
#define CCR_DMOD_EOBFIFO   ( 0x3 << 12 )
#define CCR_SMOD_LINEAR    ( 0x0 << 10 )
#define CCR_SMOD_2D        ( 0x1 << 10 )
#define CCR_SMOD_FIFO      ( 0x2 << 10 )
#define CCR_SMOD_EOBFIFO   ( 0x3 << 10 )
#define CCR_MDIR_INC       (0<<9)
#define CCR_MDIR_DEC       (1<<9)
#define CCR_MSEL_A         (0<<8)
#define CCR_MSEL_B         (1<<8)
#define CCR_DSIZ_32        ( 0x0 << 6 )
#define CCR_DSIZ_8         ( 0x1 << 6 )
#define CCR_DSIZ_16        ( 0x2 << 6 )
#define CCR_SSIZ_32        ( 0x0 << 4 )
#define CCR_SSIZ_8         ( 0x1 << 4 )
#define CCR_SSIZ_16        ( 0x2 << 4 )
#define CCR_REN            (1<<3)
#define CCR_RPT            (1<<2)
#define CCR_FRC            (1<<1)
#define CCR_CEN            (1<<0)
#define RTOR_EN            (1<<15)
#define RTOR_CLK           (1<<14)
#define RTOR_PSC           (1<<13)


#define UDMA_IOC_MAGIC      'u'
#define UDMA_GETBUFSZ       _IOR( UDMA_IOC_MAGIC, 1, unsigned long )
#define UDMA_ALLOC_DMA      _IO( UDMA_IOC_MAGIC, 2 )
#define UDMA_FREE_DMA       _IO( UDMA_IOC_MAGIC, 3 )
#define UDMA_PA             _IOWR( UDMA_IOC_MAGIC, 4, unsigned long )
#define UDMA_GET_DMA_REGS   _IOR( UDMA_IOC_MAGIC, 5, UDMA_REGS ) 
#define UDMA_SET_DMA_REGS   _IOW( UDMA_IOC_MAGIC, 6, UDMA_REGS )
#define UDMA_START_DMA      _IOW( UDMA_IOC_MAGIC, 7, UDMA_DESC )
#define UDMA_ABORT_DMA      _IO( UDMA_IOC_MAGIC, 8 )
#define UDMA_DMA_STATUS     _IOR( UDMA_IOC_MAGIC, 9, UDMA_STATUS )
#define UDMA_FBPA           _IOR( UDMA_IOC_MAGIC, 10, UDMA_LCDC_REGS )

typedef struct {
    int             dmac;
    unsigned long   ccr;
    unsigned long   rssr;
    unsigned long   blr;
    unsigned long   bucr;
    unsigned long   wsra; // w >= x
    unsigned long   wsrb;
    unsigned long   xsra; // x/bl must be integer x >= bl
    unsigned long   xsrb;
    unsigned long   ysra;
    unsigned long   ysrb;
} UDMA_REGS;

typedef struct {
    int             dmac;
    unsigned long   sar;
    unsigned long   dar;
    unsigned long   cntr;
} UDMA_DESC;

typedef struct {
    int             dmac;
    unsigned long   ccr;
} UDMA_STATUS;

typedef struct {
    unsigned long   ssa;
    unsigned long   lgwsar;
    unsigned long   lgwpor;
} UDMA_LCDC_REGS;

#endif
