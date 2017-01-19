/*
 * HID over SPI protocol implementation
 *
 * Copyright (c) 2017 Joël Krähemann <jkraehemann@gmail.com>
 *
 * This code is partly based on "HID over I2C protocol implementation":
 *
 *  Copyright (c) 2012 Benjamin Tissoires <benjamin.tissoires@gmail.com>
 *  Copyright (c) 2012 Ecole Nationale de l'Aviation Civile, France
 *  Copyright (c) 2012 Red Hat, Inc
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
#include <linux/gpio/consumer.h>

#include <linux/spi/spi-hid.h>

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

#define spi_hid_dbg(ihid, fmt, arg...)					  \
do {									  \
	if (debug)							  \
		dev_printk(KERN_DEBUG, &(ihid)->spi->dev, fmt, ##arg); \
} while (0)

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

#define SPI_HID_CMD(opcode_) \
	.opcode = opcode_, .length = 4, \
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

static DEFINE_MUTEX(spi_hid_open_mut);

/* The main device structure */
struct spi_hid {
	struct spi_device *spi;                                      /* SPI device */
	struct spi_bitbang *bitbang;

	struct spi_transfer *transfer;
	struct spi_message *message;

	struct hid_device	*hid;	/* pointer to corresponding HID dev */
	union {
		__u8 hdesc_buffer[sizeof(struct spi_hid_desc)];
		struct spi_hid_desc hdesc;	/* the HID Descriptor */
	};
	__le16			wHIDDescRegister; /* location of the spi
						   * register of the HID
						   * descriptor. */
	unsigned int		bufsize;	/* spi buffer size */
	char			*inbuf;		/* Input buffer */
	char			*rawbuf;	/* Raw Input buffer */
	char			*cmdbuf;	/* Command buffer */
	char			*argsbuf;	/* Command arguments buffer */

	unsigned long		flags;		/* device flags */

	wait_queue_head_t	wait;		/* For waiting the interrupt */
	struct gpio_desc	*desc;
	int			irq;

	struct spi_hid_platform_data pdata;

	bool			irq_wake_enabled;
	struct mutex		reset_lock;
};

static int __spi_hid_command(struct spi_device *spi,
		const struct spi_hid_cmd *command, u8 reportID,
		u8 reportType, u8 *args, int args_len,
		unsigned char *buf_recv, int data_len)
{
	struct spi_hid *ihid = spi_get_drvdata(client);
	union command *cmd = (union command *)ihid->cmdbuf;
	struct spi_transfer t[2];
	int num_xfers = 1;
	int ret;

	int length = command->length;
	bool wait = command->wait;
	unsigned int registerIndex = command->registerIndex;

	/* special case for hid_descr_cmd */
	if (command == &hid_descr_cmd) {
		cmd->c.reg = ihid->wHIDDescRegister;
	} else {
		cmd->data[0] = ihid->hdesc_buffer[registerIndex];
		cmd->data[1] = ihid->hdesc_buffer[registerIndex + 1];
	}

	if (length > 2) {
		cmd->c.opcode = command->opcode;
		cmd->c.reportTypeID = reportID | reportType << 4;
	}

	memcpy(cmd->data + length, args, args_len);
	length += args_len;

	spi_hid_dbg(ihid, "%s: cmd=%*ph\n", __func__, length, cmd->data);
	
	t[0].tx_buf = cmd->data;
	t[0].len = length;
	
	if (data_len > 0) {
		t[1].rx_buf = buf_recv;
		t[1].len = data_len;
		num_xfers++;
		
		set_bit(SPI_HID_READ_PENDING, &ihid->flags);
	}

	if (wait)
		set_bit(SPI_HID_RESET_PENDING, &ihid->flags);

	ret = spi_sync_transfer(spi, t, num_xfers);

	if (data_len > 0)
		clear_bit(SPI_HID_READ_PENDING, &ihid->flags);

	
	if (ret != num_xfers)
		return ret < 0 ? ret : -EIO;

	ret = 0;

	if (wait) {
		spi_hid_dbg(ihid, "%s: waiting...\n", __func__);
		if (!wait_event_timeout(ihid->wait,
				!test_bit(SPI_HID_RESET_PENDING, &ihid->flags),
				msecs_to_jiffies(5000)))
			ret = -ENODATA;
		spi_hid_dbg(ihid, "%s: finished.\n", __func__);
	}

	return ret;
}

module_spi_driver(spi_hid_driver);

MODULE_DESCRIPTION("HID over SPI core driver");
MODULE_AUTHOR("Joël Krähemann <jkraehemann@gmail.com>");
MODULE_LICENSE("GPL");
