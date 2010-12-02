#ifndef __CHUMBY_EMMA_H__
#define __CHUMBY_EMMA_H__

#define EMMA_IOC_MAGIC      'e'

#define EMMA_PP_SET_REGS	_IOW( EMMA_IOC_MAGIC, 1, EMMA_PP_REGS )
#define EMMA_PP_GET_REGS    _IOR( EMMA_IOC_MAGIC, 2, EMMA_PP_REGS )
#define EMMA_PP_STATUS		_IOR( EMMA_IOC_MAGIC, 3, EMMA_PP_STAT )
#define EMMA_PP_START		_IO( EMMA_IOC_MAGIC, 4 )
#define EMMA_PP_ABORT		_IO( EMMA_IOC_MAGIC, 5 )
#define EMMA_PP_RESET		_IO( EMMA_IOC_MAGIC, 6 )

#define EMMA_PrP_SET_REGS   _IOW( EMMA_IOC_MAGIC, 7, EMMA_PrP_REGS )
#define EMMA_PrP_GET_REGS   _IOR( EMMA_IOC_MAGIC, 8, EMMA_PrP_REGS )
#define EMMA_PrP_STATUS	   	_IOR( EMMA_IOC_MAGIC, 9, EMMA_PrP_STAT )
#define EMMA_PrP_START		_IO( EMMA_IOC_MAGIC, 10 )
#define EMMA_PrP_ABORT		_IO( EMMA_IOC_MAGIC, 11 )
#define EMMA_PrP_RESET		_IO( EMMA_IOC_MAGIC, 12 )

/* EMMA PP Control Register bit settings
*/
#define PP_CNTL_EN              ( 1 << 0 )
#define PP_CNTL_SWRST           ( 1 << 8 )

/* EMMA PP Interrupt Control Register
*/
#define PP_CNTL_RSTVAL		    0x876
#define PP_INT_MASK             ( 7 )
#define PP_ERR_INTR             ( 1 << 2 )
#define PP_FRAME_COMP_INTR      ( 1 << 0 )


/* EMMP PreP Control Register bit settings
*/
#define PrP_CNTL_RSTVAL		    0x28
#define PrP_INT_MASK            ( 0x1FF )
#define PrP_CNTL_CH1EN          (1<<0)
#define PrP_CNTL_CH2EN          (1<<1)
#define PrP_CNTL_EN             (PrP_CNTL_CH1EN|PrP_CNTL_CH2EN)
#define PrP_CNTL_IN_32			(1<<3)
#define PrP_CNTL_IN_RGB		    (1<<4)
#define PrP_CNTL_IN_YUV420		0
#define PrP_CNTL_IN_YUV422		PrP_CNTL_IN_32
#define PrP_CNTL_IN_RGB16		PrP_CNTL_IN_RGB
#define PrP_CNTL_IN_RGB32		(PrP_CNTL_IN_RGB | PrP_CNTL_IN_32)
#define PrP_CNTL_CH1_RGB8		0
#define PrP_CNTL_CH1_RGB16		(1<<5)
#define PrP_CNTL_CH1_RGB32		(1<<6)
#define PrP_CNTL_CH1_YUV422		((1<<5) | (1<<6))
#define PrP_CNTL_CH2_YUV420		0
#define PrP_CNTL_CH2_YUV422		(1<<7)
#define PrP_CNTL_CH2_YUV444		(1<<8)
#define PrP_CNTL_CH1_LOOP		(1<<9)
#define PrP_CNTL_CH2_LOOP		(1<<10)
#define PrP_CNTL_AUTODROP		(1<<11)
#define PrP_CNTL_SWRST          (1<<12)
#define PrP_CNTL_RST			(1<<12)
#define PrP_CNTL_CNTREN		    (1<<13)
#define PrP_CNTL_WINEN			(1<<14)
#define PrP_CNTL_UNCHAIN		(1<<15)
#define PrP_CNTL_FIFO_I128		0
#define PrP_CNTL_FIFO_I96		(1<<25)
#define PrP_CNTL_FIFO_I64		(1<<26)
#define PrP_CNTL_FIFO_I32		((1<<25) | (1<<26))
#define PrP_CNTL_FIFO_O64		0
#define PrP_CNTL_FIFO_O48		(1<<27)
#define PrP_CNTL_FIFO_O32		(1<<28)
#define PrP_CNTL_FIFO_O16		((1<<27) | (1<<28))
#define PrP_CNTL_CH2B1			(1<<29)
#define PrP_CNTL_CH2B2			(1<<30)
#define PrP_CNTL_CH2_FLOWEN		(1<<31)

#define PrP_INTRCNTL_RDERR		(1<<0)
#define PrP_INTRCNTL_CH1ERR		(1<<1)
#define PrP_INTRCNTL_CH2ERR		(1<<2)
#define PrP_INTRCNTL_FRM		(1<<3)
#define PrP_ICR_CH1			    (1<<3)
#define PrP_ICR_CH2			    (1<<5)
#define PrP_ICR_OVR			    (1<<7)
#define PrP_ICR_FLOW			(1<<8)

#define PrP_INTRSTAT_RDERR		(1<<0)
#define PrP_INTRSTAT_CH1ERR		(1<<1)
#define PrP_INTRSTAT_CH2ERR		(1<<2)
#define PrP_INTRSTAT_CH2BUF2	(1<<3)
#define PrP_INTRSTAT_CH2BUF1	(1<<4)
#define PrP_INTRSTAT_CH1BUF2	(1<<5)
#define PrP_INTRSTAT_CH1BUF1	(1<<6)
#define PrP_ISR_OVR			    (1<<7)
#define PrP_ISR_FLOW			(1<<8)


typedef struct  {
	unsigned long cntl;
	unsigned long source_y_ptr;
	unsigned long source_cb_ptr;
	unsigned long source_cr_ptr;
	unsigned long dest_rgb_ptr;
	unsigned long quantizer_ptr;
	unsigned long process_frame_para;
	unsigned long source_frame_width;
	unsigned long dest_display_width;
	unsigned long dest_image_size;
	unsigned long dest_frame_fmt_cntl;
	unsigned long resize_table_index_reg;
	unsigned long csc_coef_0123;
	unsigned long csc_coef_4;
	unsigned long resize_coef_tbl[40];
} EMMA_PP_REGS;


typedef struct {
    unsigned long cntl;
    unsigned long intrcntl;
    unsigned long intrstatus;
    int           done;
} EMMA_PP_STAT;


typedef struct  {
	unsigned long cntl;
	unsigned long source_y_ptr;
	unsigned long source_cb_ptr;
	unsigned long source_cr_ptr;
	unsigned long dest_rgb1_ptr;
	unsigned long dest_rgb2_ptr;
	unsigned long dest_y_ptr;
	unsigned long dest_cb_ptr;
	unsigned long dest_cr_ptr;
	unsigned long source_frame_size;
	unsigned long ch1_line_stride;
	unsigned long src_pixel_format_cntl;
	unsigned long ch1_pixel_format_cntl;
	unsigned long ch1_out_image_size;
	unsigned long ch2_out_image_size;
	unsigned long source_line_stride;
	unsigned long csc_coef_012;
	unsigned long csc_coef_345;
	unsigned long csc_coef_678;
	unsigned long ch1_rz_hori_coef1;
	unsigned long ch1_rz_hori_coef2;
	unsigned long ch1_rz_hori_valid;
	unsigned long ch1_rz_vert_coef1;
	unsigned long ch1_rz_vert_coef2;
	unsigned long ch1_rz_vert_valid;
	unsigned long ch2_rz_hori_coef1;
	unsigned long ch2_rz_hori_coef2;
	unsigned long ch2_rz_hori_valid;
	unsigned long ch2_rz_vert_coef1;
	unsigned long ch2_rz_vert_coef2;
	unsigned long ch2_rz_vert_valid;
} EMMA_PrP_REGS;


typedef struct {
    unsigned long cntl;
    unsigned long intrcntl;
    unsigned long intrstatus;
    int           done;
} EMMA_PrP_STAT;


#endif
