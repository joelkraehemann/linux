/*
 * HID over SPI protocol implementation
 *
 * Copyright (c) 2017 Joel Kraehemann <jkraehemann@gmail.com>
 *
 *  This code is partly based on "HID over I2C protocol implementation":
 *
 *  Copyright (c) 2012 Benjamin Tissoires <benjamin.tissoires@gmail.com>
 *  Copyright (c) 2012 Ecole Nationale de l'Aviation Civile, France
 *  Copyright (c) 2012 Red Hat, Inc
 *
 *  This code is partly based on "HIDDEV":
 * 
 *  Copyright (c) 2001 Paul Stewart
 *  Copyright (c) 2001 Vojtech Pavlik
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 */

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/err.h>
#include <linux/string.h>
#include <linux/list.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/hid.h>
#include <linux/mutex.h>
#include <linux/acpi.h>
#include <linux/of.h>
#include <uapi/linux/hiddev.h>

#include "../hid-ids.h"

#define spi_hid_dbg(shid, fmt, arg...)					\
	do {								\
		if (debug)						\
			dev_printk(KERN_DEBUG, &(shid)->spi->dev, fmt, ##arg); \
	} while (0)

#define SPI_HID_VROM_VERSION_ID 0x01

#define SPI_HID_IOBUF_LENGTH    2048

/* flags */
#define SPI_HID_STARTED		0
#define SPI_HID_RESET_PENDING	1
#define SPI_HID_READ_PENDING	2

#define SPI_HID_PWR_ON		0x00
#define SPI_HID_PWR_SLEEP	0x01

/* debug option */
static bool debug;
module_param(debug, bool, 0444);
MODULE_PARM_DESC(debug, "print a lot of debug information");

enum{
	SPI_HID_ACCELEROMETER,
	SPI_HID_ACTUATOR,
	SPI_HID_DEVICE_MANAGEMENT,
	SPI_HID_KEYBOARD_BOOT,
	SPI_HID_TRACKPAD_BOOT,
	SPI_HID_LAST,
} spi_hid_type;

struct spi_hid_report {
	__le16 length;	
	u8 data[SPI_HID_IOBUF_LENGTH];
};

struct spi_hid_desc {
	__le16 wHIDDescLength;
	__le16 bcdVersion;
	__le16 wReportDescLength;
	__le16 wReportDescRegister;
	__le16 wInputRegister;
	__le16 wMaxInputLength;
	__le16 wOutputRegister;
	__le16 wMaxOutputLength;
	__le16 wCommandRegister;
	__le16 wDataRegister;
	__le16 wVendorID;
	__le16 wProductID;
	__le16 wVersionID;
	__le32 reserved;
} __packed;

struct spi_hid_cmd {
	unsigned int registerIndex;
	__u8 opcode;
	unsigned int length;
	bool wait;
};

union command {
	u8 data[0];
	struct cmd {
		__le16 reg;
		__u8 reportTypeID;
		__u8 opcode;
	} __packed c;
};

#define SPI_HID_CMD(opcode_)						\
	.opcode = opcode_, .length = 4,					\
		.registerIndex = offsetof(struct spi_hid_desc, wCommandRegister)

/* fetch HID descriptor */
static const struct spi_hid_cmd hid_descr_cmd = { .length = 2 };
/* fetch report descriptors */
static const struct spi_hid_cmd hid_report_descr_cmd = {
	.registerIndex = offsetof(struct spi_hid_desc,
				  wReportDescRegister),
	.opcode = 0x00,
	.length = 2 };
/* commands */
static const struct spi_hid_cmd hid_reset_cmd =		{ SPI_HID_CMD(0x01),
							  .wait = true };
static const struct spi_hid_cmd hid_get_report_cmd =	{ SPI_HID_CMD(0x02) };
static const struct spi_hid_cmd hid_set_report_cmd =	{ SPI_HID_CMD(0x03) };
static const struct spi_hid_cmd hid_set_power_cmd =	{ SPI_HID_CMD(0x08) };
static const struct spi_hid_cmd hid_no_cmd =		{ .length = 0 };

/*
 * These definitions are not used here, but are defined by the spec.
 * Keeping them here for documentation purposes.
 *
 * static const struct spi_hid_cmd hid_get_idle_cmd = { SPI_HID_CMD(0x04) };
 * static const struct spi_hid_cmd hid_set_idle_cmd = { SPI_HID_CMD(0x05) };
 * static const struct spi_hid_cmd hid_get_protocol_cmd = { SPI_HID_CMD(0x06) };
 * static const struct spi_hid_cmd hid_set_protocol_cmd = { SPI_HID_CMD(0x07) };
 */

struct spi_hid_vrom_header {
	__le16 vrom_size;
};

struct spi_hid_vrom_entry {
	struct spi_hid_desc *hdesc;
	
	struct spi_hid_report report;

	unsigned char *input;
	unsigned char *output;
	unsigned char *command;
	unsigned char *data;
};

struct spi_hid_vrom {
	__u8 vrom_data[0];

	struct spi_hid_vrom_header header;
	
	struct spi_hid_vrom_entry accelerometer;
	struct spi_hid_vrom_entry actuator;
	struct spi_hid_vrom_entry device_management;
	struct spi_hid_vrom_entry trackpad_boot;
	struct spi_hid_vrom_entry keyboard_boot;
};

struct spi_hid_vrom spit_vrom = {
	.header = {
		.vrom_size = sizeof(struct spi_hid_vrom),
	},
	
	/* accelerometer report */
	.accelerometer.report = {
		.length = 27,
		.data = "\x06\x00\xff\x09\x03\xa1\x01\x06\x00\xff\x09\x03\x15\x00\x26\xff\x00\x85\xc0\x96\x6b\x00\x75\x08\x81\x02\xc0",
	}, 
	
	/* actuator report */
	.actuator.report = {
		.length = 36,
		.data = "\x06\x00\xff\x09\x0d\xa1\x01\x06\x00\xff\x09\x0d\x15\x00\x26\xff\x00\x75\x08\x85\x3f\x96\x0f\x00\x81\x02\x09\x0d\x85\x53\x96\x3f\x00\x91\x02\xc0",
	},

	/* device management report */
	.device_management.report = {
		.length = 27,
		.data = "\x06\x00\xff\x09\x0b\xa1\x01\x06\x00\xff\x09\x0b\x15\x00\x26\xff\x00\x75\x08\x96\x04\x00\x85\xe0\x81\x22\xc0",
	},
	
	/* trackpad/boot report */
	.trackpad_boot.report = {
		.length = 110,
		.data = "\x05\x01\x09\x02\xa1\x01\x09\x01\xa1\x00\x05\x09\x19\x01\x29\x03\x15\x00\x25\x01\x85\x02\x95\x03\x75\x01\x81\x02\x95\x01\x75\x05\x81\x01\x05\x01\x09\x30\x09\x31\x15\x81\x25\x7f\x75\x08\x95\x02\x81\x06\x95\x04\x75\x08\x81\x01\xc0\xc0\x05\x0d\x09\x05\xa1\x01\x06\x00\xff\x09\x0c\x15\x00\x26\xff\x00\x75\x08\x95\x10\x85\x3f\x81\x22\xc0\x06\x00\xff\x09\x0c\xa1\x01\x06\x00\xff\x09\x0c\x15\x00\x26\xff\x00\x85\x44\x75\x08\x96\x57\x05\x81\x00\xc0",
	},
	
	/* keyboard/boot report */
	.keyboard_boot.report = {
		.length = 208,
		.data = "\x05\x01\x09\x06\xa1\x01\x85\x01\x05\x07\x19\xe0\x29\xe7\x15\x00\x25\x01\x75\x01\x95\x08\x81\x02\x95\x01\x75\x08\x81\x01\x95\x05\x75\x01\x05\x08\x19\x01\x29\x05\x91\x02\x95\x01\x75\x03\x91\x01\x95\x06\x75\x08\x15\x00\x26\xff\x00\x05\x07\x19\x00\x29\xff\x81\x00\x05\x0c\x75\x01\x95\x01\x09\xb8\x15\x00\x25\x01\x81\x02\x05\xff\x09\x03\x75\x07\x95\x01\x81\x02\xc0\x05\x0c\x09\x01\xa1\x01\x85\x52\x15\x00\x25\x01\x75\x01\x95\x01\x09\xcd\x81\x02\x09\xb3\x81\x02\x09\xb4\x81\x02\x09\xb5\x81\x02\x09\xb6\x81\x02\x81\x01\x81\x01\x81\x01\x85\x09\x15\x00\x25\x01\x75\x08\x95\x01\x06\x01\xff\x09\x0b\xb1\x02\x75\x08\x95\x02\xb1\x01\xc0\x06\x00\xff\x09\x06\xa1\x01\x06\x00\xff\x09\x06\x15\x00\x26\xff\x00\x75\x08\x95\x40\x85\x3f\x81\x22\xc0\x06\x00\xff\x09\x0f\xa1\x01\x06\x00\xff\x09\x0f\x15\x00\x26\xff\x00\x75\x08\x95\x0f\x85\xbf\x81\x02\xc0",
	},
};

static const struct spi_hid_report *shid_report[SPI_HID_LAST] = {
	&spit_vrom.accelerometer.report,
	&spit_vrom.actuator.report,
	&spit_vrom.device_management.report,
	&spit_vrom.keyboard_boot.report,
	&spit_vrom.trackpad_boot.report,
};

struct spi_hid_desc shid_desc[SPI_HID_LAST] = {

	/* SPI_HID_ACCELEROMETER */
	{
		.wHIDDescLength = 0x1a,
		.bcdVersion = 0x03,
		.wReportDescLength = sizeof(spit_vrom.accelerometer.report.data),
		.wReportDescRegister = (void *) (&spit_vrom.accelerometer.report.data) - (void *) spit_vrom.vrom_data,
		.wInputRegister = (void *) (&spit_vrom.accelerometer.input) - (void *) spit_vrom.vrom_data,
		.wMaxInputLength = SPI_HID_IOBUF_LENGTH,
		.wOutputRegister = (void *) (&spit_vrom.accelerometer.output) - (void *) spit_vrom.vrom_data,
		.wMaxOutputLength = SPI_HID_IOBUF_LENGTH,
		.wCommandRegister = (void *) (&spit_vrom.accelerometer.command) - (void *) spit_vrom.vrom_data,
		.wDataRegister = (void *) (&spit_vrom.accelerometer.data) - (void *) spit_vrom.vrom_data,
		.wVendorID = SPI_VENDOR_ID_APPLE,
		.wProductID = SPI_DEVICE_ID_APPLE_INTERNAL_KEYBOARD_2016_ISO,
		.wVersionID = SPI_HID_VROM_VERSION_ID,
	},

	/* SPI_HID_ACTUATOR */
	{
		.wHIDDescLength = 0x1a,
		.bcdVersion = 0x03,
		.wReportDescLength = sizeof(spit_vrom.actuator.report.data),
		.wReportDescRegister = (void *) (&spit_vrom.actuator.report.data) - (void *) spit_vrom.vrom_data,
		.wInputRegister = (void *) (&spit_vrom.actuator.input) - (void *) spit_vrom.vrom_data,
		.wMaxInputLength = SPI_HID_IOBUF_LENGTH,
		.wOutputRegister = (void *) (&spit_vrom.actuator.output) - (void *) spit_vrom.vrom_data,
		.wMaxOutputLength = SPI_HID_IOBUF_LENGTH,
		.wCommandRegister = (void *) (&spit_vrom.actuator.command) - (void *) spit_vrom.vrom_data,
		.wDataRegister = (void *) (&spit_vrom.actuator.data) - (void *) spit_vrom.vrom_data,
		.wVendorID = SPI_VENDOR_ID_APPLE,
		.wProductID = SPI_DEVICE_ID_APPLE_INTERNAL_KEYBOARD_2016_ISO,
		.wVersionID = SPI_HID_VROM_VERSION_ID,
	},

	/* SPI_HID_DEVICE_MANAGEMENT */
	{
		.wHIDDescLength = 0x1a,
		.bcdVersion = 0x03,
		.wReportDescLength = sizeof(spit_vrom.device_management.report.data),
		.wReportDescRegister = (void *) (&spit_vrom.device_management.report.data) - (void *) spit_vrom.vrom_data,
		.wInputRegister = (void *) (&spit_vrom.device_management.input) - (void *) spit_vrom.vrom_data,
		.wMaxInputLength = SPI_HID_IOBUF_LENGTH,
		.wOutputRegister = (void *) (&spit_vrom.device_management.output) - (void *) spit_vrom.vrom_data,
		.wMaxOutputLength = SPI_HID_IOBUF_LENGTH,
		.wCommandRegister = (void *) (&spit_vrom.device_management.command) - (void *) spit_vrom.vrom_data,
		.wDataRegister = (void *) (&spit_vrom.device_management.data) - (void *) spit_vrom.vrom_data,
		.wVendorID = SPI_VENDOR_ID_APPLE,
		.wProductID = SPI_DEVICE_ID_APPLE_INTERNAL_KEYBOARD_2016_ISO,
		.wVersionID = SPI_HID_VROM_VERSION_ID,
	},

	/* SPI_HID_KEYBOARD_BOOT */
	{
		.wHIDDescLength = 0x1a,
		.bcdVersion = 0x03,
		.wReportDescLength = sizeof(spit_vrom.trackpad_boot.report.data),
		.wReportDescRegister = (void *) (&spit_vrom.trackpad_boot.report.data) - (void *) spit_vrom.vrom_data,
		.wInputRegister = (void *) (&spit_vrom.trackpad_boot.input) - (void *) spit_vrom.vrom_data,
		.wMaxInputLength = SPI_HID_IOBUF_LENGTH,
		.wOutputRegister = (void *) (&spit_vrom.trackpad_boot.output) - (void *) spit_vrom.vrom_data,
		.wMaxOutputLength = SPI_HID_IOBUF_LENGTH,
		.wCommandRegister = (void *) (&spit_vrom.trackpad_boot.command) - (void *) spit_vrom.vrom_data,
		.wDataRegister = (void *) (&spit_vrom.trackpad_boot.data) - (void *) spit_vrom.vrom_data,
		.wVendorID = SPI_VENDOR_ID_APPLE,
		.wProductID = SPI_DEVICE_ID_APPLE_INTERNAL_KEYBOARD_2016_ISO,
		.wVersionID = SPI_HID_VROM_VERSION_ID,
	},

	/* SPI_HID_TRACKPAD_BOOT */
	{
		.wHIDDescLength = 0x1a,
		.bcdVersion = 0x03,
		.wReportDescLength = sizeof(spit_vrom.keyboard_boot.report.data),
		.wReportDescRegister = (void *) (&spit_vrom.keyboard_boot.report.data) - (void *) spit_vrom.vrom_data,
		.wInputRegister = (void *) (&spit_vrom.keyboard_boot.input) - (void *) spit_vrom.vrom_data,
		.wMaxInputLength = SPI_HID_IOBUF_LENGTH,
		.wOutputRegister = (void *) (&spit_vrom.keyboard_boot.output) - (void *) spit_vrom.vrom_data,
		.wMaxOutputLength = SPI_HID_IOBUF_LENGTH,
		.wCommandRegister = (void *) (&spit_vrom.keyboard_boot.command) - (void *) spit_vrom.vrom_data,
		.wDataRegister = (void *) (&spit_vrom.keyboard_boot.data) - (void *) spit_vrom.vrom_data,
		.wVendorID = SPI_VENDOR_ID_APPLE,
		.wProductID = SPI_DEVICE_ID_APPLE_INTERNAL_KEYBOARD_2016_ISO,
		.wVersionID = SPI_HID_VROM_VERSION_ID,
	},
};

static DEFINE_MUTEX(spi_hid_open_mut);

/* the HID device */
struct spi_hid_device {
	void *parent;
	
	struct hid_device	*hid;                 /* pointer to corresponding HID dev */
	const struct spi_hid_report   *report;        /* Report provided in RAM */

	struct spi_hid_vrom_entry     *vrom_entry;    /* just in case you might want to match IO VROM register - it's thread-safe */
	__le16			wHIDDescRegister;     /* location of the SPIT VROM
						       * register of the HID
						       * descriptor. */
	
	union {
		__u8                          hdesc_buffer[sizeof(struct spi_hid_desc)];     
		struct spi_hid_desc           hdesc;                                         /* the HID Descriptor */
	};
};

/* The main device structure */
struct spi_hid {
	struct spi_device *spi;                 /* SPI device */
	struct mutex      driver_lock;          /* as you obtain driver_lock you may access struct */

	struct spi_hid_device   shid_device[SPI_HID_LAST];

	unsigned int		bufsize;	/* spi buffer size */
	char			*inbuf;		/* Input buffer */
	char			*rawbuf;	/* Raw Input buffer */
	char                    *outbuf;        /* Output buffer */
	char			*cmdbuf;	/* Command buffer */
	char			*argsbuf;	/* Command arguments buffer */

	unsigned long		flags;		/* device flags */

	wait_queue_head_t	wait;		/* For waiting the interrupt */
	int			irq;

	bool			irq_wake_enabled;
	struct mutex		reset_lock;
};

static int __spi_hid_command(struct spi_hid_device *shid_device,
			     const struct spi_hid_cmd *command, u8 reportID,
			     u8 reportType, u8 *args, int args_len,
			     unsigned char *buf_recv, int data_len)
{
	struct spi_hid *shid = shid_device->parent;
	struct spi_device *spi;
	struct spi_hid_vrom_entry *vrom_entry;
	union command *cmd = (union command *) shid->cmdbuf;
	union _tx_rx_buf {
		unsigned char tx_buf[command->length];
		unsigned char rx_buf[data_len];
		unsigned char hid_buf[SPI_HID_IOBUF_LENGTH];
	}iobuf_tx, iobuf_rx;
	unsigned int iobuf_limit = SPI_HID_IOBUF_LENGTH;
	int ret;

	int length = command->length;
	bool wait = command->wait;
	bool is_vio = 0;
	unsigned int registerIndex = command->registerIndex;

	/* initial read/write setup */
	mutex_lock(&shid->driver_lock);

	spi = shid->spi;
	vrom_entry = shid_device->vrom_entry;

	/* ignore power commands */
	if (command->opcode == 0x01 ||
	    command->opcode == 0x03 ||
	    command->opcode == 0x08) {
		mutex_unlock(&shid->driver_lock);
		
		return(0);
	}
	
	if (command->opcode == 0x02) {
		cmd->data[0] = shid_device->hdesc_buffer[registerIndex];
		cmd->data[1] = shid_device->hdesc_buffer[registerIndex + 1];

		is_vio = 1;
	}	
	
	/* special case for hid_descr_cmd */
	if (command == &hid_descr_cmd) {
		cmd->c.reg = shid_device->wHIDDescRegister;

		is_vio = 1;
	} else {
		cmd->data[0] = shid_device->hdesc_buffer[registerIndex];
		cmd->data[1] = shid_device->hdesc_buffer[registerIndex + 1];

		/* check all buffer locations in order to guess if it's virtual IO */
		if((&spit_vrom + cmd->c.reg != &vrom_entry->input) &&
		   (&spit_vrom + cmd->c.reg != &vrom_entry->output) &&
		   (&spit_vrom + cmd->c.reg != &vrom_entry->command) &&
		   (&spit_vrom + cmd->c.reg != &vrom_entry->data)){
			is_vio = 1;
		}
	}
	
	if (length > 2) {
		cmd->c.opcode = command->opcode;
		cmd->c.reportTypeID = reportID | reportType << 4;
	}

	memcpy(cmd->data + length, args, args_len);		
	length += args_len;

 	mutex_unlock(&shid->driver_lock);
	
	spi_hid_dbg(shid, "%s: cmd=%*ph\n", __func__, length, cmd->data);

	/* prepare IO */
	if (!is_vio) {
		memset(iobuf_tx.tx_buf, 0, iobuf_limit * sizeof(char));
		memset(iobuf_rx.rx_buf, 0, iobuf_limit * sizeof(char));
	}

	if (is_vio) {
		if (cmd->c.reg == shid_device->wHIDDescRegister) {
			__u8 *shid_desc;

			shid_desc = shid_device->hdesc_buffer;
			
			buf_recv[0] = HID_ITEM_FORMAT_SHORT;
			buf_recv[1] = shid_desc[registerIndex];
			buf_recv[2] = shid_desc[registerIndex + 1];
		} else if (&spit_vrom + cmd->c.reg == &vrom_entry->report.data) {
			buf_recv[0] = HID_ITEM_TAG_LONG;
			buf_recv[1] = 0xff & vrom_entry->report.length;
			buf_recv[2] = HID_MAIN_ITEM_TAG_OUTPUT;

			memcpy(buf_recv + 3, vrom_entry->report.data, buf_recv[1]);
		} else if (command->opcode == 0x02) {
			buf_recv[0] = HID_ITEM_TAG_LONG;
			buf_recv[1] = 0xff & vrom_entry->report.length;
			buf_recv[2] = HID_MAIN_ITEM_TAG_OUTPUT;

			memcpy(buf_recv + 3, vrom_entry->report.data, buf_recv[1]);
		}
	}
	
	mutex_lock(&shid->driver_lock);

	
	if (!is_vio) {
		memcpy(iobuf_tx.tx_buf, cmd->data, length * sizeof(unsigned char));

		set_bit(SPI_HID_READ_PENDING, &shid->flags);

		if (wait)
			set_bit(SPI_HID_RESET_PENDING, &shid->flags);
	}

	mutex_unlock(&shid->driver_lock);
	
	if (!is_vio) {
		ret = spi_write_then_read(spi,
					  iobuf_tx.tx_buf, length,
					  iobuf_rx.rx_buf, data_len);
		
		/* notify pending */
		if (data_len > 0) {
			clear_bit(SPI_HID_READ_PENDING, &shid->flags); //TODO:JK: verify thread-safety
			
			memcpy(buf_recv, iobuf_rx.rx_buf, data_len * sizeof(char));
		}
	}

	/* event timeout */
	if (wait) {
		spi_hid_dbg(shid, "%s: waiting...\n", __func__);
		if (!wait_event_timeout(shid->wait,
					!test_bit(SPI_HID_RESET_PENDING, &shid->flags),
					msecs_to_jiffies(5000)))
			ret = -ENODATA;
		spi_hid_dbg(shid, "%s: finished.\n", __func__);
	}
	
	return 0;
}

static int spi_hid_command(struct spi_hid_device *shid_device,
			   const struct spi_hid_cmd *command,
			   unsigned char *buf_recv, int data_len)
{
	return __spi_hid_command(shid_device, command, 0, 0, NULL, 0,
				 buf_recv, data_len);
}

static int spi_hid_get_report(struct spi_hid_device *shid_device, u8 reportType,
			      u8 reportID, unsigned char *buf_recv, int data_len)
{
	struct spi_hid *shid = shid_device->parent;
	u8 args[3];
	int ret;
	int args_len = 0;
	u16 readRegister = le16_to_cpu(shid_device->hdesc.wDataRegister);

	spi_hid_dbg(shid, "%s\n", __func__);

	if (reportID >= 0x0F) {
		args[args_len++] = reportID;
		reportID = 0x0F;
	}

	args[args_len++] = readRegister & 0xFF;
	args[args_len++] = readRegister >> 8;

	ret = __spi_hid_command(shid_device, &hid_get_report_cmd, reportID,
				reportType, args, args_len, buf_recv, data_len);
	if (ret) {
		dev_err(&shid->spi->dev,
			"failed to retrieve report from device.\n");
		return ret;
	}

	return 0;
}

/**
 * spi_hid_set_or_send_report: forward an incoming report to the device
 * @spi: the spi_device of the device
 * @reportType: 0x03 for HID_FEATURE_REPORT ; 0x02 for HID_OUTPUT_REPORT
 * @reportID: the report ID
 * @buf: the actual data to transfer, without the report ID
 * @len: size of buf
 * @use_data: true: use SET_REPORT HID command, false: send plain OUTPUT report
 */
static int spi_hid_set_or_send_report(struct spi_hid_device *shid_device, u8 reportType,
				      u8 reportID, unsigned char *buf, size_t data_len, bool use_data)
{
	struct spi_device *spi;
	struct spi_hid *shid = shid_device->parent;
	u8 *args;
	char *argsbuf_last;
	const struct spi_hid_cmd *hidcmd;
	int ret;
	u16 dataRegister = le16_to_cpu(shid_device->hdesc.wDataRegister);
	u16 outputRegister = le16_to_cpu(shid_device->hdesc.wOutputRegister);
	u16 maxOutputLength = le16_to_cpu(shid_device->hdesc.wMaxOutputLength);
	u16 size;
	unsigned int bufsize;
	int args_len;
	int index = 0;

	/* common properties */
	mutex_lock(&shid->driver_lock);

	spi = shid->spi;
	
	args = shid->argsbuf;
	bufsize = shid->bufsize;
	
	mutex_unlock(&shid->driver_lock);

	/* check quality */
	spi_hid_dbg(shid, "%s\n", __func__);


	if (data_len > bufsize)
		return -EINVAL;

	size =		2		/* size */ +
		(reportID ? 1 : 0)	/* reportID */ +
		data_len		/* buf */;
	args_len =	(reportID >= 0x0F ? 1 : 0) /* optional third byte */ +
		2			/* dataRegister */ +
		size			/* args */;

	if (!use_data && maxOutputLength == 0)
		return -ENOSYS;

	/* operate on argsbuf */
	mutex_lock(&shid->driver_lock);

	if (reportID >= 0x0F) {	
		args[index++] = reportID;
		reportID = 0x0F;
	}

	/*
	 * use the data register for feature reports or if the device does not
	 * support the output register
	 */
	if (use_data) {
		args[index++] = dataRegister & 0xFF;
		args[index++] = dataRegister >> 8;
		hidcmd = &hid_set_report_cmd;
	} else {
		args[index++] = outputRegister & 0xFF;
		args[index++] = outputRegister >> 8;
		hidcmd = &hid_no_cmd;
	}

	args[index++] = size & 0xFF;
	args[index++] = size >> 8;

	if (reportID)
		args[index++] = reportID;

	argsbuf_last = &args[index];
	
	mutex_unlock(&shid->driver_lock);

	/*  */
	memcpy(argsbuf_last, buf, data_len);

	ret = __spi_hid_command(shid_device, hidcmd, reportID,
				reportType, args, args_len, NULL, 0);
	if (ret) {
		dev_err(&spi->dev, "failed to set a report to device.\n");
		return ret;
	}

	return data_len;
}


static int spi_hid_set_power(struct spi_hid_device *shid_device, int power_state)
{
	struct spi_device *spi;
	struct spi_hid *shid = shid_device->parent;
	int ret;

	/* common properties */
	mutex_lock(&shid->driver_lock);

	spi = shid->spi;

	mutex_unlock(&shid->driver_lock);

	/* hid command */
	spi_hid_dbg(shid, "%s\n", __func__);

	ret = __spi_hid_command(shid_device, &hid_set_power_cmd, power_state,
				0, NULL, 0, NULL, 0);
	if (ret)
		dev_err(&spi->dev, "failed to change power setting.\n");

	return ret;
}

static int spi_hid_hwreset(struct spi_hid_device *shid_device)
{
	struct spi_hid *shid = shid_device->parent;
	struct spi_device *spi;
	int ret;

	/* common properties */
	mutex_lock(&shid->driver_lock);

	spi = shid->spi;

	mutex_unlock(&shid->driver_lock);

	/* hwreset */
	spi_hid_dbg(shid, "%s\n", __func__);

	/*
	 * This prevents sending feature reports while the device is
	 * being reset. Otherwise we may lose the reset complete
	 * interrupt.
	 */
	mutex_lock(&shid->reset_lock);

	ret = spi_hid_set_power(shid_device, SPI_HID_PWR_ON);
	if (ret)
		goto out_unlock;

	spi_hid_dbg(shid, "resetting...\n");

	ret = spi_hid_command(shid_device, &hid_reset_cmd, NULL, 0);
	if (ret) {
		dev_err(&spi->dev, "failed to reset device.\n");
		spi_hid_set_power(shid_device, SPI_HID_PWR_SLEEP);
	}

 out_unlock:
	mutex_unlock(&shid->reset_lock);
	return ret;
}

static void spi_hid_get_input(struct spi_hid_device *shid_device)
{
	struct spi_device *spi;
	struct spi_hid *shid = shid_device->parent;
	char *inbuf;
	unsigned int bufsize;
	int ret, ret_size;
	int size = le16_to_cpu(shid_device->hdesc.wMaxInputLength);

	/* common properties */
	mutex_lock(&shid->driver_lock);

	spi = shid->spi;

	inbuf = shid->inbuf;
	
	bufsize = shid->bufsize;
	
	mutex_unlock(&shid->driver_lock);

	/* check quality */	
	if (size > bufsize)
		size = bufsize;

	/* read data */
	mutex_lock(&shid->driver_lock);

	ret = spi_read(spi, inbuf, size);

	mutex_unlock(&shid->driver_lock);

	/* check successful read */
	if (ret != 0) {
		dev_err(&spi->dev, "%s: got %d data instead of %d\n",
			__func__, ret, size);
		return;
	}

	/* check return size */
	mutex_lock(&shid->driver_lock);

	ret_size = shid->inbuf[0] | shid->inbuf[1] << 8;

	if (!ret_size) {
		mutex_unlock(&shid->driver_lock);
		
		/* host or device initiated RESET completed */
		if (test_and_clear_bit(SPI_HID_RESET_PENDING, &shid->flags)) {
			wake_up(&shid->wait);
		}

		return;
	}

	if (ret_size > size) {
		dev_err(&spi->dev, "%s: incomplete report (%d/%d)\n",
			__func__, size, ret_size);

		mutex_unlock(&shid->driver_lock);

		return;
	}

	spi_hid_dbg(shid, "input: %*ph\n", ret_size, shid->inbuf);

	if (test_bit(SPI_HID_STARTED, &shid->flags))
		hid_input_report(shid_device->hid, HID_INPUT_REPORT, shid->inbuf + 2,
				 ret_size - 2, 1);

	mutex_unlock(&shid->driver_lock);

	return;
}

static irqreturn_t spi_hid_irq(int irq, void *dev_id)
{
	struct spi_hid *shid = dev_id;
	struct spi_hid_device *shid_device;

	unsigned int i;

	/* common properties */
	mutex_lock(&shid->driver_lock);

	shid_device = shid->shid_device;

	/* return if read pending */
	if (test_bit(SPI_HID_READ_PENDING, &shid->flags)) {
		mutex_unlock(&shid->driver_lock);
	
		return IRQ_HANDLED;
	}

	mutex_unlock(&shid->driver_lock);

	/*  */
	for (i = 0; i < SPI_HID_LAST; i++){
		spi_hid_get_input(&shid_device[i]);
	}

	return IRQ_HANDLED;
}

static int spi_hid_get_report_length(struct hid_report *report)
{
	return ((report->size - 1) >> 3) + 1 +
		report->device->report_enum[report->type].numbered + 2;
}

static void spi_hid_init_report(struct hid_report *report, u8 *buffer,
				size_t bufsize)
{
	struct hid_device *hid = report->device;
	struct spi_hid_device *shid_device = hid->driver_data;
	struct spi_hid *shid = shid_device->parent;
	struct spi_device *spi;
	unsigned int size, ret_size;

	/* common properties */
	mutex_lock(&shid->driver_lock);

	spi = shid->spi;
	
	mutex_unlock(&shid->driver_lock);

	/* get report */
	size = spi_hid_get_report_length(report);
	if (spi_hid_get_report(shid_device,
			       report->type == HID_FEATURE_REPORT ? 0x03 : 0x01,
			       report->id, buffer, size))
		return;

	spi_hid_dbg(shid, "report (len=%d): %*ph\n", size, size, buffer);

	ret_size = buffer[0] | (buffer[1] << 8);

	if (ret_size != size) {
		dev_err(&spi->dev, "error in %s size:%d / ret_size:%d\n",
			__func__, size, ret_size);
		return;
	}

	/* hid->driver_lock is held as we are in probe function,
	 * we just need to setup the input fields, so using
	 * hid_report_raw_event is safe. */
	hid_report_raw_event(hid, report->type, buffer + 2, size - 2, 1);
}

/*
 * Initialize all reports
 */
static void spi_hid_init_reports(struct hid_device *hid)
{
	struct hid_report *report;
	struct spi_hid_device *shid_device = hid->driver_data;
	struct spi_hid *shid = shid_device->parent;
	struct spi_device *spi;
	u8 *inbuf;
	unsigned int bufsize;
	
	/* common properties */
	mutex_lock(&shid->driver_lock);

	spi = shid->spi;

	bufsize = shid->bufsize;
	
	mutex_unlock(&shid->driver_lock);
	
	/* allocate buffer */
	inbuf = kzalloc(bufsize, GFP_KERNEL);
	if (!inbuf) {
		dev_err(&spi->dev, "can not retrieve initial reports\n");
		return;
	}
	
	/*
	 * The device must be powered on while we fetch initial reports
	 * from it.
	 */
	pm_runtime_get_sync(&spi->dev);

	list_for_each_entry(report,
			    &hid->report_enum[HID_FEATURE_REPORT].report_list, list);
	spi_hid_init_report(report, inbuf, bufsize);
	
	pm_runtime_put(&spi->dev);

	kfree(inbuf);
}

/*
 * Traverse the supplied list of reports and find the longest
 */
static void spi_hid_find_max_report(struct hid_device *hid, unsigned int type,
				    unsigned int *max)
{
	struct hid_report *report;
	unsigned int size;

	/* We should not rely on wMaxInputLength, as some devices may set it to
	 * a wrong length. */
	list_for_each_entry(report, &hid->report_enum[type].report_list, list) {
		size = spi_hid_get_report_length(report);
		if (*max < size)
			*max = size;
	}
}

static void spi_hid_free_buffers(struct spi_hid *shid)
{
	kfree(shid->inbuf);
	kfree(shid->rawbuf);
	kfree(shid->outbuf);
	kfree(shid->argsbuf);
	kfree(shid->cmdbuf);
	shid->inbuf = NULL;
	shid->rawbuf = NULL;
	shid->outbuf = NULL;
	shid->cmdbuf = NULL;
	shid->argsbuf = NULL;
	shid->bufsize = 0;
}

static int spi_hid_alloc_buffers(struct spi_hid *shid, size_t report_size)
{
	/* the worst case is computed from the set_report command with a
	 * reportID > 15 and the maximum report length */
	int args_len = sizeof(__u8) + /* optional ReportID byte */
		sizeof(__u16) + /* data register */
		sizeof(__u16) + /* size of the report */
		report_size; /* report */
	
	/* allocated buffers */
	mutex_lock(&shid->driver_lock);

	shid->inbuf = kzalloc(report_size, GFP_KERNEL);
	shid->rawbuf = kzalloc(report_size, GFP_KERNEL);
	shid->outbuf = kzalloc(report_size, GFP_KERNEL);
	shid->argsbuf = kzalloc(args_len, GFP_KERNEL);
	shid->cmdbuf = kzalloc(sizeof(union command) + args_len, GFP_KERNEL);

	if (!shid->inbuf || !shid->rawbuf || !shid->argsbuf || !shid->cmdbuf) {
		mutex_unlock(&shid->driver_lock);

		spi_hid_free_buffers(shid);

		return -ENOMEM;
	}

	shid->bufsize = report_size;
	
	mutex_unlock(&shid->driver_lock);

	return 0;
}

static int spi_hid_get_raw_report(struct hid_device *hid,
				  unsigned char report_number, __u8 *buf, size_t count,
				  unsigned char report_type)
{
	struct spi_hid_device *shid_device = hid->driver_data;
	struct spi_hid *shid = shid_device->parent;
	struct spi_device *spi;
	char *rawbuf;
	unsigned int bufsize;
	size_t ret_count, ask_count;
	int ret;

	if (report_type == HID_OUTPUT_REPORT)
		return -EINVAL;

	/* common propertis */
	mutex_lock(&shid->driver_lock);

	spi = shid->spi;

	bufsize = shid->bufsize;
	rawbuf = shid->rawbuf;
	
	mutex_unlock(&shid->driver_lock);

	/* +2 bytes to include the size of the reply in the query buffer */
	ask_count = min(count + 2, (size_t) bufsize);

	ret = spi_hid_get_report(shid_device,
				 report_type == HID_FEATURE_REPORT ? 0x03 : 0x01,
				 report_number, rawbuf, ask_count);

	if (ret < 0)
		return ret;

	/* get report */
	mutex_lock(&shid->driver_lock);

	ret_count = rawbuf[0] | (rawbuf[1] << 8);


	if (ret_count <= 2) {
		mutex_unlock(&shid->driver_lock);

		return 0;
	}

	ret_count = min(ret_count, ask_count);

	/* The query buffer contains the size, dropping it in the reply */
	count = min(count, ret_count - 2);
	memcpy(buf, rawbuf + 2, count);

	mutex_unlock(&shid->driver_lock);

	return count;
}

static int spi_hid_output_raw_report(struct hid_device *hid, __u8 *buf,
				     size_t count, unsigned char report_type, bool use_data)
{
	struct spi_hid_device *shid_device = hid->driver_data;
	struct spi_hid *shid = shid_device->parent;
	int report_id = buf[0];
	int ret;

	if (report_type == HID_INPUT_REPORT)
		return -EINVAL;
	
	mutex_lock(&shid->reset_lock);

	if (report_id) {
		buf++;
		count--;
	}

	ret = spi_hid_set_or_send_report(shid_device,
					 report_type == HID_FEATURE_REPORT ? 0x03 : 0x02,
					 report_id, buf, count, use_data);

	if (report_id && ret >= 0)
		ret++; /* add report_id to the number of transfered bytes */

	mutex_unlock(&shid->reset_lock);

	return ret;
}

static int spi_hid_output_report(struct hid_device *hid, __u8 *buf,
				 size_t count)
{
	return spi_hid_output_raw_report(hid, buf, count, HID_OUTPUT_REPORT,
					 false);
}

static int spi_hid_raw_request(struct hid_device *hid, unsigned char reportnum,
			       __u8 *buf, size_t len, unsigned char rtype,
			       int reqtype)
{
	switch (reqtype) {
	case HID_REQ_GET_REPORT:
		return spi_hid_get_raw_report(hid, reportnum, buf, len, rtype);
	case HID_REQ_SET_REPORT:
		if (buf[0] != reportnum)
			return -EINVAL;
		return spi_hid_output_raw_report(hid, buf, len, rtype, true);
	default:
		return -EIO;
	}
}

static int spi_hid_parse(struct hid_device *hid)
{
	struct spi_hid_device *shid_device = hid->driver_data;
	struct spi_hid_desc *hdesc = &shid_device->hdesc;
	struct spi_hid *shid = shid_device->parent;
	unsigned int rsize = 0;
	char *rdesc;
	int ret;
	int tries = 3;

	spi_hid_dbg(shid, "entering %s\n", __func__);

	rsize = le16_to_cpu(hdesc->wReportDescLength);
	if (!rsize || rsize > HID_MAX_DESCRIPTOR_SIZE) {
		dbg_hid("weird size of report descriptor (%u)\n", rsize);
		return -EINVAL;
	}

	do {
		ret = spi_hid_hwreset(shid_device);
		if (ret)
			msleep(1000);
	} while (tries-- > 0 && ret);

	if (ret)
		return ret;

	rdesc = kzalloc(rsize, GFP_KERNEL);

	if (!rdesc) {
		dbg_hid("couldn't allocate rdesc memory\n");
		return -ENOMEM;
	}

	spi_hid_dbg(shid, "asking HID report descriptor\n");

	ret = spi_hid_command(shid_device, &hid_report_descr_cmd, rdesc, rsize);
	if (ret) {
		hid_err(hid, "reading report descriptor failed\n");
		kfree(rdesc);
		return -EIO;
	}

	spi_hid_dbg(shid, "Report Descriptor: %*ph\n", rsize, rdesc);

	ret = hid_parse_report(hid, rdesc, rsize);
	kfree(rdesc);
	if (ret) {
		dbg_hid("parsing report descriptor failed\n");
		return ret;
	}

	return 0;
}

static int spi_hid_start(struct hid_device *hid)
{
	struct spi_hid_device *shid_device = hid->driver_data;
	struct spi_hid *shid = shid_device->parent;
	int ret;
	unsigned int bufsize;
	unsigned int minbufsize = SPI_HID_IOBUF_LENGTH;

	/* common propertis */
	mutex_lock(&shid->driver_lock);

	bufsize = shid->bufsize;
	
	mutex_unlock(&shid->driver_lock);

	/*  */
	spi_hid_find_max_report(hid, HID_INPUT_REPORT, &minbufsize);
	spi_hid_find_max_report(hid, HID_OUTPUT_REPORT, &minbufsize);
	spi_hid_find_max_report(hid, HID_FEATURE_REPORT, &minbufsize);

	if (minbufsize > bufsize) {
		spi_hid_free_buffers(shid);

		ret = spi_hid_alloc_buffers(shid, minbufsize);

		if (ret)
			return ret;
	}

	if (!(hid->quirks & HID_QUIRK_NO_INIT_REPORTS))
		spi_hid_init_reports(hid);

	return 0;
}

static void spi_hid_stop(struct hid_device *hid)
{
	hid->claimed = 0;
}

static int spi_hid_open(struct hid_device *hid)
{
	struct spi_hid_device *shid_device = hid->driver_data;
	struct spi_hid *shid = shid_device->parent;
	struct spi_device *spi;
	int ret = 0;
	int is_started = 0;

	/* common properties */
	mutex_lock(&shid->driver_lock);

	spi = shid->spi;
	
	mutex_unlock(&shid->driver_lock);

	/* power management */
	mutex_lock(&spi_hid_open_mut);
	if (!hid->open++) {
		ret = pm_runtime_get_sync(&spi->dev);
		if (ret < 0) {
			hid->open--;
			goto done;
		}

		is_started = 1;
	}
 done:
	mutex_unlock(&spi_hid_open_mut);

	/* set appropriate flag to indicate that we started */
	if (is_started) {
		mutex_lock(&shid->driver_lock);

		set_bit(SPI_HID_STARTED, &shid->flags);

		mutex_unlock(&shid->driver_lock);
	}
	
	return ret < 0 ? ret : 0;
}

static void spi_hid_close(struct hid_device *hid)
{
	struct spi_hid_device *shid_device = hid->driver_data;
	struct spi_hid *shid = shid_device->parent;
	struct spi_device *spi;
	int is_closed = 0;
	
	/* common properties */
	mutex_lock(&shid->driver_lock);

	spi = shid->spi;
	
	mutex_unlock(&shid->driver_lock);

	/* protecting hid->open to make sure we don't restart
	 * data acquistion due to a resumption we no longer
	 * care about
	 */
	mutex_lock(&spi_hid_open_mut);

	if (!--hid->open) {
		is_closed = 1;

		/* Save some power */
		pm_runtime_put(&spi->dev);
	}
	
	mutex_unlock(&spi_hid_open_mut);

	/* set appropriate flag to indicate that we started */
	if (is_closed) {
		mutex_lock(&shid->driver_lock);
		
		clear_bit(SPI_HID_STARTED, &shid->flags);
		
		mutex_unlock(&shid->driver_lock);
	}
}

static int spi_hid_power(struct hid_device *hid, int lvl)
{
	struct spi_hid_device *shid_device = hid->driver_data;
	struct spi_hid *shid = shid_device->parent;
	struct spi_device *spi;

	/* common properties */
	mutex_lock(&shid->driver_lock);

	spi = shid->spi;
	
	mutex_unlock(&shid->driver_lock);

	/* set power hint */
	spi_hid_dbg(shid, "%s lvl:%d\n", __func__, lvl);

	switch (lvl) {
	case PM_HINT_FULLON:
		pm_runtime_get_sync(&spi->dev);
		break;
	case PM_HINT_NORMAL:
		pm_runtime_put(&spi->dev);
		break;
	}
	return 0;
}

static struct hid_ll_driver spi_hid_ll_driver = {
	.parse = spi_hid_parse,
	.start = spi_hid_start,
	.stop = spi_hid_stop,
	.open = spi_hid_open,
	.close = spi_hid_close,
	.power = spi_hid_power,
	.output_report = spi_hid_output_report,
	.raw_request = spi_hid_raw_request,
};

static int spi_hid_init_irq(struct spi_device *spi)
{
	struct spi_hid *shid = spi_get_drvdata(spi);
	int ret;

	dev_dbg(&spi->dev, "Requesting IRQ: %d\n", shid->irq);

	ret = request_irq(shid->irq, spi_hid_irq, IRQF_PROBE_SHARED, spi->modalias,
			  shid);
	if (ret < 0) {
		dev_warn(&spi->dev,
			 "Could not register for %s interrupt, irq = %d,"
			 " ret = %d\n",
			 spi->modalias, shid->irq, ret);

		return ret;
	}

	return 0;
}

static int spi_hid_fetch_hid_descriptor(struct spi_hid_device *shid_device)
{
	struct spi_hid *shid = shid_device->parent;
	struct spi_device *spi;
	int ret;
	
	/* common properties */
	mutex_lock(&shid->driver_lock);

	spi = shid->spi;
	
	mutex_unlock(&shid->driver_lock);

	/* spi hid fetch descriptor */
	spi_hid_dbg(shid, "Fetching the HID descriptor\n");
	ret = spi_hid_command(shid_device, &hid_descr_cmd, shid_device->hdesc_buffer,
			      sizeof(struct spi_hid_desc));
	if (ret) {
		dev_err(&spi->dev, "hid_descr_cmd failed\n");
		return -ENODEV;
	}

	return(0);
}

#ifdef CONFIG_ACPI
static const struct acpi_device_id spi_hid_acpi_match[] = {
	{"APP000D", 0 },
	{ },
};
MODULE_DEVICE_TABLE(acpi, spi_hid_acpi_match);
#endif

#ifdef CONFIG_OF
static const struct of_device_id spi_hid_of_match[] = {
	{ .compatible = "APPLE-SPI-TOPCASE" },
	{},
};
MODULE_DEVICE_TABLE(of, spi_hid_of_match);
#endif

static void spi_hid_unregister_transport(void *data)
{
	/* TODO: implement me */
}

static int spi_hid_probe(struct spi_device *spi)
{
	int ret;
	struct spi_hid *shid;
	struct spi_hid_vrom_entry *vrom_entry = (struct spi_hid_vrom_entry *) &spit_vrom.vrom_data[sizeof(struct spi_hid_vrom_header)];
	int i;
	
	dbg_hid("HID probe called for spi 0x%02x\n", spi->chip_select);

	/*  */
	shid = kzalloc(sizeof(struct spi_hid), GFP_KERNEL);
	if (!shid)
		return -ENOMEM;
		
	spi_set_drvdata(spi, shid);

	shid->spi = spi;
	mutex_init(&shid->driver_lock);

	init_waitqueue_head(&shid->wait);
	mutex_init(&shid->reset_lock);

	/* we need to allocate the command buffer without knowing the maximum
	 * size of the reports. Let's use SPI_HID_IOBUF_LENGTH, then we do the
	 * real computation later. */
	shid->bufsize = SPI_HID_IOBUF_LENGTH;
		
	ret = spi_hid_alloc_buffers(shid, SPI_HID_IOBUF_LENGTH);
	if (ret < 0)
		goto err;

	pm_runtime_get_noresume(&spi->dev);
	pm_runtime_set_active(&spi->dev);
	pm_runtime_enable(&spi->dev);
	device_enable_async_suspend(&spi->dev);

	/* initialize interrupt */
	spi->irq = 0x0e; //TODO:JK: might be we should detect the interrupt
	shid->irq = spi->irq; 
	
	ret = spi_hid_init_irq(spi);
	if (ret < 0)
		goto err_pm;

	/* SPI HID devices */
	ret = 0;
	
	for (i = 0; i < SPI_HID_LAST; i++) {
		struct spi_hid_device *shid_device;
		struct hid_device *hid;

		shid_device = &shid->shid_device[i];
		
		/* SPI HID VROM entry */
		shid_device->vrom_entry = &vrom_entry[i];
		shid_device->wHIDDescRegister = (void *) &vrom_entry[i].hdesc - (void *) spit_vrom.vrom_data;
		
		vrom_entry[i].hdesc = &shid_device->hdesc;
		memcpy(&shid_device->hdesc, &shid_desc[i], sizeof(struct spi_hid_desc));
		
		vrom_entry[i].input = shid->inbuf;
		vrom_entry[i].output = shid->outbuf;
		vrom_entry[i].command = shid->cmdbuf;
		vrom_entry[i].data = shid->argsbuf;
		
		/* SPI HID device */
		shid_device->parent = shid;

		shid_device->report = shid_report[i];		

		ret = spi_hid_fetch_hid_descriptor(shid_device);
		if (ret != 0) {
			goto err_mem_free;
		}

		shid_device->hid = NULL;
		hid = hid_allocate_device();
		
		if (IS_ERR(hid)) {
			ret = PTR_ERR(hid);
			goto err_mem_free;
		}

		shid_device->hid = hid;
		
		hid->driver_data = shid_device;
		hid->ll_driver = &spi_hid_ll_driver;
		hid->dev.parent = &spi->dev;
		hid->bus = BUS_SPI;

		hid->version = le16_to_cpu(shid_device->hdesc.bcdVersion);
		hid->vendor = le16_to_cpu(shid_device->hdesc.wVendorID);
		hid->product = le16_to_cpu(shid_device->hdesc.wProductID);

		snprintf(hid->name, sizeof(hid->name), "%s %04hX:%04hX",
			 spi->modalias, hid->vendor, hid->product);
		strlcpy(hid->phys, dev_name(&spi->dev), sizeof(hid->phys));

		ret = hid_add_device(hid);
		if (ret) {
			if (ret != -ENODEV)
				hid_err(spi, "can't add hid device: %d\n", ret);
			goto err_mem_free;
		}

	}

	ret = devm_add_action_or_reset(&spi->dev,
				       spi_hid_unregister_transport,
				       shid);
	if (ret)
		goto err_mem_free;

	dev_info(&spi->dev, "registered HID SPI driver\n");
	
	pm_runtime_put(&spi->dev);
	
	return 0;

 err_mem_free:
	free_irq(shid->irq, shid);

	for (i = 0; i < SPI_HID_LAST; i++) {
		if(shid->shid_device[i].hid != NULL)
			hid_destroy_device(shid->shid_device[i].hid);
		else
			break;
	}

 err_pm:
	pm_runtime_put_noidle(&spi->dev);
	pm_runtime_disable(&spi->dev);

 err:
	spi_hid_free_buffers(shid);
	kfree(shid);
	return ret;
}

static int spi_hid_remove(struct spi_device *spi)
{
	struct spi_hid *shid = spi_get_drvdata(spi);
	struct spi_hid_device *shid_device;

	unsigned int i;

	pm_runtime_get_sync(&spi->dev);
	pm_runtime_disable(&spi->dev);
	pm_runtime_set_suspended(&spi->dev);
	pm_runtime_put_noidle(&spi->dev);

	/* common properties */
	mutex_lock(&shid->driver_lock);

	shid_device = shid->shid_device;
	
	mutex_unlock(&shid->driver_lock);

	/* do not disturb */
	free_irq(shid->irq, shid);

	for (i = 0; i < SPI_HID_LAST; i++){
		struct hid_device *hid;
		
		hid = shid_device[i].hid;
		hid_destroy_device(hid);
	}
	
	if (shid->bufsize)
		spi_hid_free_buffers(shid);

	kfree(shid);

	return 0;
}

static void spi_hid_shutdown(struct spi_device *spi)
{
	struct spi_hid *shid = spi_get_drvdata(spi);
	struct spi_hid_device *shid_device;
	unsigned int i;

	/* common properties */
	mutex_lock(&shid->driver_lock);

	shid_device = shid->shid_device;
	
	mutex_unlock(&shid->driver_lock);

	/* power sleep */
	free_irq(shid->irq, shid);

	for (i = 0; i < SPI_HID_LAST; i++){
		spi_hid_set_power(&shid_device[i], SPI_HID_PWR_SLEEP);
	}
}

#ifdef CONFIG_PM_SLEEP
static int spi_hid_suspend(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct spi_hid *shid = spi_get_drvdata(spi);
	struct spi_hid_device *shid_device;
	unsigned int i;
	int ret;

	/* common properties */
	mutex_lock(&shid->driver_lock);

	shid_device = shid->shid_device;
	
	mutex_unlock(&shid->driver_lock);

	/* suspend */
	for (i = 0; i < SPI_HID_LAST; i++){
			struct hid_device *hid;
			
			hid = shid_device[i].hid;
			
			if (hid->driver && hid->driver->suspend) {
				/*
				 * Wake up the device so that IO issues in
				 * HID driver's suspend code can succeed.
				 */
				ret = pm_runtime_resume(dev);
				if (ret < 0)
					continue;

				ret = hid->driver->suspend(hid, PMSG_SUSPEND);
				if (ret < 0)
					continue;
			}
	}

	if (!pm_runtime_suspended(dev)) {
		for (i = 0; i < SPI_HID_LAST; i++){
			/* Save some power */
			spi_hid_set_power(&shid_device[i], SPI_HID_PWR_SLEEP);
		}
	}

	return 0;
}

static int spi_hid_resume(struct device *dev)
{
	int ret;
	struct spi_device *spi = to_spi_device(dev);
	struct spi_hid *shid = spi_get_drvdata(spi);
	struct spi_hid_device *shid_device;

	unsigned int i;
	
	/* We'll resume to full power */
	pm_runtime_disable(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	/* common properties */
	mutex_lock(&shid->driver_lock);

	shid_device = shid->shid_device;
	
	mutex_unlock(&shid->driver_lock);

	/* resume */
	for(i = 0; i < SPI_HID_LAST; i++){
		struct hid_device *hid;

		ret = spi_hid_hwreset(&shid_device[i]);
		if (ret)
			continue;

		hid = shid_device[i].hid;
		
		if (hid->driver && hid->driver->reset_resume) {
			ret = hid->driver->reset_resume(hid);
			continue;
		}
	}

	return 0;
}
#endif

#ifdef CONFIG_PM
static int spi_hid_runtime_suspend(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct spi_hid *shid = spi_get_drvdata(spi);
	struct spi_hid_device *shid_device;
	unsigned int i;
	
	/* common properties */
	mutex_lock(&shid->driver_lock);

	shid_device = shid->shid_device;
	
	mutex_unlock(&shid->driver_lock);

	/* power sleep */
	for (i = 0; i < SPI_HID_LAST; i++){
		/* Save some power */
		spi_hid_set_power(&shid_device[i], SPI_HID_PWR_SLEEP);
	}

	return 0;
}

static int spi_hid_runtime_resume(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct spi_hid *shid = spi_get_drvdata(spi);
	struct spi_hid_device *shid_device;
	unsigned int i;

	/* common properties */
	mutex_lock(&shid->driver_lock);

	shid_device = shid->shid_device;
	
	mutex_unlock(&shid->driver_lock);

	/* power on */
	for (i = 0; i < SPI_HID_LAST; i++){
		spi_hid_set_power(&shid_device[i], SPI_HID_PWR_ON);
	}

	return 0;
}
#endif

static const struct dev_pm_ops spi_hid_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(spi_hid_suspend, spi_hid_resume)
	SET_RUNTIME_PM_OPS(spi_hid_runtime_suspend, spi_hid_runtime_resume,
			   NULL)
};

static const struct spi_device_id spi_hid_id_table[] = {
	{ "hid", 0 },
	{ "hid-over-spi", 0 },
	{ "apple-spi-topcase", 0 },
	{ },
};
MODULE_DEVICE_TABLE(spi, spi_hid_id_table);

static struct spi_driver spi_hid_driver = {
	.id_table = spi_hid_id_table,
	
	.driver = {
		.name	= "spi_hid",
		.bus    = &spi_bus_type,
		.owner  = THIS_MODULE,
		.pm	= &spi_hid_pm,
		.acpi_match_table = ACPI_PTR(spi_hid_acpi_match),
		.of_match_table = of_match_ptr(spi_hid_of_match),
	},

	.probe		= spi_hid_probe,
	.remove		= spi_hid_remove,
	.shutdown	= spi_hid_shutdown,
	.id_table	= spi_hid_id_table,
};

module_spi_driver(spi_hid_driver);

MODULE_DESCRIPTION("HID over SPI core driver");
MODULE_AUTHOR("Joel Kraehemann <jkraehemann@gmail.com>");
MODULE_LICENSE("GPL");
