/*
 * Cloned by Greg Hutchins from original TSC2101 Touchscreen Driver 
 * for the Chumby TSC2100.
 * 
 * Texas Instruments TSC2100 Touchscreen Driver
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

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/mfd/tsc2100.h>
#include "tsc2100.h"

#define X_AXIS_MAX		3830
#define X_AXIS_MIN		150
#define Y_AXIS_MAX		3830
#define Y_AXIS_MIN		190
#define PRESSURE_MIN	0
#define PRESSURE_MAX	20000

#define DRIVER_VERSION  0x1

void tsc2100_ts_report(struct tsc2100_data *tsc2100_ts, int x, int y, int p, int pendown)
{
	input_report_abs(tsc2100_ts->inputdevice, ABS_X, y);
	input_report_abs(tsc2100_ts->inputdevice, ABS_Y, x);
	input_report_abs(tsc2100_ts->inputdevice, ABS_PRESSURE, p);
	input_report_key(tsc2100_ts->inputdevice, BTN_TOUCH, pendown);
	input_sync(tsc2100_ts->inputdevice);
}

void tsc2100_ts_setup(struct device *dev)
{
	struct tsc2100_data *tsc2100_ts = dev_get_drvdata(dev);

	tsc2100_ts->inputdevice->name = "tsc2100_ts";
	tsc2100_ts->inputdevice->evbit[0] = BIT(EV_KEY) | BIT(EV_ABS);
	tsc2100_ts->inputdevice->keybit[LONG(BTN_TOUCH)] |= BIT(BTN_TOUCH);
	tsc2100_ts->inputdevice->id.version = DRIVER_VERSION;
	input_set_abs_params(tsc2100_ts->inputdevice, ABS_X, X_AXIS_MIN, X_AXIS_MAX, 0, 0);
	input_set_abs_params(tsc2100_ts->inputdevice, ABS_Y, Y_AXIS_MIN, Y_AXIS_MAX, 0, 0);
	input_set_abs_params(tsc2100_ts->inputdevice, ABS_PRESSURE, PRESSURE_MIN, PRESSURE_MAX, 0, 0);
	input_register_device(tsc2100_ts->inputdevice);
	printk("Chumby tsc2100 touchscreen driver initialized\n");
}
