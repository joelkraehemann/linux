/*
 * SPI HID support for Linux
 *
 * Copyright (c) 2017 Joël Krähemann
 */

/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 3 as published by
 * the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <asm/unaligned.h>
#include <asm/byteorder.h>
#include <linux/input.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/string.h>

#include <linux/spi/spi.h>

#include <linux/hid.h>
#include <linux/hiddev.h>
#include <linux/hid-debug.h>
#include <linux/hidraw.h>
#include "spihid.h"

/*
 * Version Information
 */

#define DRIVER_NAME "spihid-core"
#define DRIVER_DESC "SPI HID core driver"
#define DRIVER_LICENSE "GPLv3"

/*
 * Module parameters.
 */

static unsigned int spihid_core_mousepoll_interval;
module_param_named(mousepoll, spihid_core_mousepoll_interval, uint, 0644);
MODULE_PARM_DESC(mousepoll, "Polling interval of mice");

static unsigned int spihid_core_ignoreled;
module_param_named(ignoreled, spihid_core_ignoreled, uint, 0644);
MODULE_PARM_DESC(ignoreled, "Autosuspend with active leds");

/*
 * Input submission and I/O error handler.
 */
static DEFINE_MUTEX(spihid_core_open_mut);

static void spihid_core_io_error(struct hid_device *hid);
static int spihid_core_submit_out(struct hid_device *hid);
static int spihid_core_submit_ctrl(struct hid_device *hid);
static void spihid_core_cancel_delayed_stuff(struct spihid_device *spihid);

/* start up the input bitbang setup */
static int spihid_core_start_in(struct hid_device *hid)
{
	unsigned long flags;
	int status = 0;
	struct spihid_device *spihid = hid->driver_data;
	struct spi_device *spi = spihid->device;
	
	spin_lock_irqsave(&spihid->lock, flags);
	if ((hid->open > 0 || hid->quirks & HID_QUIRK_ALWAYS_POLL) &&
			!test_bit(HID_DISCONNECTED, &spihid->iofl) &&
			!test_bit(HID_SUSPENDED, &spihid->iofl) &&
			!test_and_set_bit(HID_IN_RUNNING, &spihid->iofl)) {

		status = spi_bitbang_setup(spi->device);
		if (status) {
			clear_bit(HID_IN_RUNNING, &spihid->iofl);
			if (!spi->controller_state)
				set_bit(HID_NO_BANDWIDTH, &spihid->iofl);
		}else{
			clear_bit(HID_NO_BANDWIDTH, &spihid->iofl);
		}

	}
	spin_unlock_irqrestore(&spihid->lock, flags);
	return status;
}

/* I/O retry timer routine */
static void spihid_core_retry_timeout(unsigned long _hid)
{
	struct hid_device *hid = (struct hid_device *) _hid;
	struct spihid_device *spihid = hid->driver_data;

	dev_dbg(&spihid->device->dev, "retrying bitbang\n");
	if (hid_start_in(hid))
		hid_io_error(hid);
}


static int __init spihid_core_init(void)
{
  printk("init_module spihid-core");
  
  return(0);
}
 
static void __exit spihid_core_exit(void)
{
  printk("cleanup_module spihid-core");
}

module_init(spihid_core_init);
module_exit(spihid_core_exit);

MODULE_AUTHOR("jkraehemann@gmail.com");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE(DRIVER_LICENSE);
MODULE_ALIAS("platform:"DRIVER_NAME);
