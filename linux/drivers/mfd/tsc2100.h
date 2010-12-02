/*
 * Cloned from TSC2100 tsc2101.h for the Chumby TSC2100.
 *
 * TI TSC2100 Hardware definitions
 *
 * Copyright 2005 Openedhand Ltd.
 *
 * Author: Richard Purdie <richard@o-hand.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

/* Modes */
#define TSC2100_MODE_TS   1
#define TSC2100_MODE_MISC 2

/* Address constructs */
#define TSC2100_READ     (1 << 15)    /* Read Register */
#define TSC2100_WRITE    (0 << 15)    /* Write Register */
#define TSC2100_PAGE(x)  ((x & 0xf) << 11) /* Memory Page to access */
#define TSC2100_ADDR(x)  ((x & 0x3f) << 5) /* Memory Address to access */

#define TSC2100_P0_REG(x) (TSC2100_PAGE(0) | TSC2100_ADDR(x))
#define TSC2100_P1_REG(x) (TSC2100_PAGE(1) | TSC2100_ADDR(x))
#define TSC2100_P2_REG(x) (TSC2100_PAGE(2) | TSC2100_ADDR(x))
#define TSC2100_P3_REG(x) (TSC2100_PAGE(3) | TSC2100_ADDR(x))

/* Page 0 Registers */
#define TSC2100_REG_X      TSC2100_P0_REG(0x0)
#define TSC2100_REG_Y      TSC2100_P0_REG(0x1)
#define TSC2100_REG_Z1     TSC2100_P0_REG(0x2)
#define TSC2100_REG_Z2     TSC2100_P0_REG(0x3)
#define TSC2100_REG_BAT1   TSC2100_P0_REG(0x5)
#define TSC2100_REG_BAT2   TSC2100_P0_REG(0x6)
#define TSC2100_REG_AUX    TSC2100_P0_REG(0x7)
#define TSC2100_REG_TEMP1  TSC2100_P0_REG(0x9)
#define TSC2100_REG_TEMP2  TSC2100_P0_REG(0xa)

/* Page 1 Registers */
#define TSC2100_REG_ADC       TSC2100_P1_REG(0x0)
#define TSC2100_REG_STATUS    TSC2100_P1_REG(0x1)
#define TSC2100_REG_RESETCTL  TSC2100_P1_REG(0x4)

/* Page 2 Registers */
#define TSC2100_REG_AUDIOCON1   TSC2100_P2_REG(0x0)
#define TSC2100_REG_CODECADCGA  TSC2100_P2_REG(0x1)
#define TSC2100_REG_DACPGA      TSC2100_P2_REG(0x2)
#define TSC2100_REG_CODECSIDETONE   TSC2100_P2_REG(0x3)
#define TSC2100_REG_AUDIOCON2   TSC2100_P2_REG(0x4)
#define TSC2100_REG_PWRDOWN     TSC2100_P2_REG(0x5)
#define TSC2100_REG_AUDIOCON3   TSC2100_P2_REG(0x6)
#define TSC2100_REG_DAEFC(x)	TSC2100_P2_REG(0x7+x)
#define TSC2100_REG_PLL1        TSC2100_P2_REG(0x1b)
#define TSC2100_REG_PLL2        TSC2100_P2_REG(0x1c)
#define TSC2100_REG_AUDIOCON4   TSC2100_P2_REG(0x1d)
#define TSC2100_REG_AUDIOCON5   TSC2100_P2_REG(0x1e)

/* Page 2 Registers */
#define TSC2100_REG_BUFLOC(x)	TSC2100_P3_REG(x)

/* Status Register Masks */

#define TSC2100_STATUS_T2STAT   (1 << 1)
#define TSC2100_STATUS_T1STAT   (1 << 2)
#define TSC2100_STATUS_AXSTAT   (1 << 4)
#define TSC2100_STATUS_B2STAT   (1 << 5)
#define TSC2100_STATUS_B1STAT   (1 << 6)
#define TSC2100_STATUS_Z2STAT   (1 << 7)
#define TSC2100_STATUS_Z1STAT   (1 << 8)
#define TSC2100_STATUS_YSTAT    (1 << 9)
#define TSC2100_STATUS_XSTAT    (1 << 10)
#define TSC2100_STATUS_DAVAIL   (1 << 11)     
#define TSC2100_STATUS_HCTLM    (1 << 12)
#define TSC2100_STATUS_PWRDN    (1 << 13)
#define TSC2100_STATUS_PINTDAV_SHIFT (14)
#define TSC2100_STATUS_PINTDAV_MASK  (0x03)



// TSC2100_REG_ADC (page01 reg00 TouchScreen ADC Control)
#define TSC2100_ADC_PSM (1<<15) // pen status mode on ctrlreg adc
#define TSC2100_ADC_STS (1<<14) // stop continuous scanning.
#define TSC2100_ADC_AD3 (1<<13) 
#define TSC2100_ADC_AD2 (1<<12) 
#define TSC2100_ADC_AD1 (1<<11) 
#define TSC2100_ADC_AD0 (1<<10) 
#define TSC2100_ADC_ADMODE(x) ((x<<10) & TSC2100_ADC_ADMODE_MASK)
#define TSC2100_ADC_ADMODE_MASK (0xf<<10)  

#define TSC2100_ADC_RES(x) ((x<<8) & TSC2100_ADC_RES_MASK )
#define TSC2100_ADC_RES_MASK (0x3<<8)  
#define TSC2100_ADC_RES_12BITP (0) // 12-bit ADC resolution (default)
#define TSC2100_ADC_RES_8BIT (1) // 8-bit ADC resolution
#define TSC2100_ADC_RES_10BIT (2) // 10-bit ADC resolution
#define TSC2100_ADC_RES_12BIT (3) // 12-bit ADC resolution

#define TSC2100_ADC_AVG(x) ((x<<6) & TSC2100_ADC_AVG_MASK )
#define TSC2100_ADC_AVG_MASK (0x3<<6)  
#define TSC2100_ADC_NOAVG (0) //  a-d does no averaging
#define TSC2100_ADC_4AVG (1) //  a-d does averaging of 4 samples
#define TSC2100_ADC_8AVG (2) //  a-d does averaging of 8 samples
#define TSC2100_ADC_16AVG (3) //  a-d does averaging of 16 samples

#define TSC2100_ADC_CL(x) ((x<<4) & TSC2100_ADC_CL_MASK )
#define TSC2100_ADC_CL_MASK (0x3<<4)
#define TSC2100_ADC_CL_8MHZ_8BIT (0)
#define TSC2100_ADC_CL_4MHZ_10BIT (1)
#define TSC2100_ADC_CL_2MHZ_12BIT (2)
#define TSC2100_ADC_CL_1MHZ_12BIT (3)
#define TSC2100_ADC_CL0 (1<< 4)  

/* ADC - Panel Voltage Stabilisation Time */
#define TSC2100_ADC_PV(x)     ((x<<1) & TSC2100_ADC_PV_MASK )
#define TSC2100_ADC_PV_MASK   (0x7<<1)
#define TSC2100_ADC_PV_100ms  (0x7)  /* 100ms */
#define TSC2100_ADC_PV_50ms   (0x6)  /* 50ms  */
#define TSC2100_ADC_PV_10ms   (0x5)  /* 10ms  */
#define TSC2100_ADC_PV_5ms    (0x4)  /* 5ms   */
#define TSC2100_ADC_PV_1ms    (0x3)  /* 1ms   */
#define TSC2100_ADC_PV_500us  (0x2)  /* 500us */
#define TSC2100_ADC_PV_100us  (0x1)  /* 100us */
#define TSC2100_ADC_PV_0s     (0x0)  /* 0s    */

#define TSC2100_ADC_AVGFILT_MEAN     (0<<0)  /* Mean Average Filter */
#define TSC2100_ADC_AVGFILT_MEDIAN   (1<<0)  /* Median Average Filter */

#define TSC2100_ADC_x (1<< 0) // don't care

#define TSC2100_CONFIG_DAV (1<<6)

#define TSC2100_KEY_STC (1<<15) // keypad status
#define TSC2100_KEY_SCS (1<<14) // keypad scan status

//#include <linux/irq.h>
