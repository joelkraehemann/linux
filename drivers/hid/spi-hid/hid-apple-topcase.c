/*
 * HID over SPI protocol implementation
 *
 * Copyright (c) 2017 Joël Krähemann <jkraehemann@gmail.com>
 *
 *  This code is partly based on "HID Apple Devices":
 * 
 *  Copyright (c) 1999 Andreas Gal
 *  Copyright (c) 2000-2005 Vojtech Pavlik <vojtech@suse.cz>
 *  Copyright (c) 2005 Michael Haboustak <mike-@cinci.rr.com> for Concept2, Inc
 *  Copyright (c) 2006-2007 Jiri Kosina
 *  Copyright (c) 2008 Jiri Slaby <jirislaby@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 */

#include <linux/device.h>
#include <linux/hid.h>
#include <linux/module.h>

#include "../hid-ids.h"

#define APPLE_TOPCASE_HAS_FN		0x0004
#define APPLE_TOPCASE_ISO_KEYBOARD	0x0010

struct apple_topcase {
	bool fn_on;
};

static __u8 *apple_topcase_report_fixup(struct hid_device *hdev, __u8 *rdesc,
					unsigned int *rsize)
{
	return rdesc;
}

static int apple_topcase_event(struct hid_device *hdev, struct hid_field *field,
			       struct hid_usage *usage, __s32 value)
{
	return 0;
}

static int apple_topcase_input_mapping(struct hid_device *hdev, struct hid_input *hi,
				       struct hid_field *field, struct hid_usage *usage,
				       unsigned long **bit, int *max)
{
	return 0;
}

static int apple_topcase_input_mapped(struct hid_device *hdev, struct hid_input *hi,
				      struct hid_field *field, struct hid_usage *usage,
				      unsigned long **bit, int *max)
{
	return 0;
}

static int apple_topcase_probe(struct hid_device *hdev,
			       const struct hid_device_id *id)
{
	unsigned long quirks = id->driver_data;
	struct apple_topcase *topcase;
	unsigned int connect_mask = HID_CONNECT_DEFAULT;
	int ret;

	topcase = kzalloc(sizeof(struct apple_topcase), GFP_KERNEL);
	if (topcase == NULL)
		return -ENOMEM;

	hid_set_drvdata(hdev, topcase);

	ret = hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "parse failed\n");
		return ret;
	}

	ret = hid_hw_start(hdev, connect_mask);
	if (ret) {
		hid_err(hdev, "hw start failed\n");
		return ret;
	}

	return 0;
}

static const struct hid_device_id apple_topcase_devices[] = {
	{ HID_SPI_DEVICE(SPI_VENDOR_ID_APPLE, SPI_DEVICE_ID_APPLE_INTERNAL_KEYBOARD_2016_ISO),
	  .driver_data = APPLE_TOPCASE_HAS_FN | APPLE_TOPCASE_ISO_KEYBOARD },
	{ }
};
MODULE_DEVICE_TABLE(hid, apple_topcase_devices);

static struct hid_driver apple_topcase_driver = {
	.name = "apple_topcase",
	.id_table = apple_topcase_devices,
	.report_fixup = apple_topcase_report_fixup,
	.probe = apple_topcase_probe,
	.event = apple_topcase_event,
	.input_mapping = apple_topcase_input_mapping,
	.input_mapped = apple_topcase_input_mapped,
};

module_hid_driver(apple_topcase_driver);

MODULE_DESCRIPTION("HID over SPI core driver");
MODULE_AUTHOR("Joël Krähemann <jkraehemann@gmail.com>");
MODULE_LICENSE("GPL");
