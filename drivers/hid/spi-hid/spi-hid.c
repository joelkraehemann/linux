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

#define spi_hid_dbg(shid, fmt, arg...)					\
	do {								\
		if (debug)						\
			dev_printk(KERN_DEBUG, &(shid)->spi->dev, fmt, ##arg); \
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

static DEFINE_MUTEX(spi_hid_open_mut);

/* The main device structure */
struct spi_hid {
	struct spi_device *spi;                                      /* SPI device */

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
	struct spi_hid *shid = spi_get_drvdata(client);
	union command *cmd = (union command *)shid->cmdbuf;
	struct spi_transfer t[2];
	int num_xfers = 1;
	int ret;

	int length = command->length;
	bool wait = command->wait;
	unsigned int registerIndex = command->registerIndex;

	/* special case for hid_descr_cmd */
	if (command == &hid_descr_cmd) {
		cmd->c.reg = shid->wHIDDescRegister;
	} else {
		cmd->data[0] = shid->hdesc_buffer[registerIndex];
		cmd->data[1] = shid->hdesc_buffer[registerIndex + 1];
	}

	if (length > 2) {
		cmd->c.opcode = command->opcode;
		cmd->c.reportTypeID = reportID | reportType << 4;
	}

	memcpy(cmd->data + length, args, args_len);
	length += args_len;

	spi_hid_dbg(shid, "%s: cmd=%*ph\n", __func__, length, cmd->data);
	
	t[0].tx_buf = cmd->data;
	t[0].len = length;
	
	if (data_len > 0) {
		t[1].rx_buf = buf_recv;
		t[1].len = data_len;
		num_xfers++;
		
		set_bit(SPI_HID_READ_PENDING, &shid->flags);
	}

	if (wait)
		set_bit(SPI_HID_RESET_PENDING, &shid->flags);

	ret = spi_sync_transfer(spi, t, num_xfers);

	if (data_len > 0)
		clear_bit(SPI_HID_READ_PENDING, &shid->flags);

	
	if (ret != num_xfers)
		return ret < 0 ? ret : -EIO;

	ret = 0;

	if (wait) {
		spi_hid_dbg(shid, "%s: waiting...\n", __func__);
		if (!wait_event_timeout(shid->wait,
					!test_bit(SPI_HID_RESET_PENDING, &shid->flags),
					msecs_to_jiffies(5000)))
			ret = -ENODATA;
		spi_hid_dbg(shid, "%s: finished.\n", __func__);
	}

	return ret;
}

static int spi_hid_command(struct spi_device *spi,
			   const struct spi_hid_cmd *command,
			   unsigned char *buf_recv, int data_len)
{
	return __spi_hid_command(spi, command, 0, 0, NULL, 0,
				 buf_recv, data_len);
}

static int spi_hid_get_report(struct spi_device *spi, u8 reportType,
			      u8 reportID, unsigned char *buf_recv, int data_len)
{
	struct spi_hid *shid = spi_get_drvdata(spi);
	u8 args[3];
	int ret;
	int args_len = 0;
	u16 readRegister = le16_to_cpu(shid->hdesc.wDataRegister);

	spi_hid_dbg(shid, "%s\n", __func__);

	if (reportID >= 0x0F) {
		args[args_len++] = reportID;
		reportID = 0x0F;
	}

	args[args_len++] = readRegister & 0xFF;
	args[args_len++] = readRegister >> 8;

	ret = __spi_hid_command(spi, &hid_get_report_cmd, reportID,
				reportType, args, args_len, buf_recv, data_len);
	if (ret) {
		dev_err(&spi->dev,
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
static int spi_hid_set_or_send_report(struct spi_device *spi, u8 reportType,
				      u8 reportID, unsigned char *buf, size_t data_len, bool use_data)
{
	struct spi_hid *shid = spi_get_drvdata(spi);
	u8 *args = shid->argsbuf;
	const struct spi_hid_cmd *hidcmd;
	int ret;
	u16 dataRegister = le16_to_cpu(shid->hdesc.wDataRegister);
	u16 outputRegister = le16_to_cpu(shid->hdesc.wOutputRegister);
	u16 maxOutputLength = le16_to_cpu(shid->hdesc.wMaxOutputLength);
	u16 size;
	int args_len;
	int index = 0;

	spi_hid_dbg(shid, "%s\n", __func__);

	if (data_len > shid->bufsize)
		return -EINVAL;

	size =		2			/* size */ +
		(reportID ? 1 : 0)	/* reportID */ +
		data_len		/* buf */;
	args_len =	(reportID >= 0x0F ? 1 : 0) /* optional third byte */ +
		2			/* dataRegister */ +
		size			/* args */;

	if (!use_data && maxOutputLength == 0)
		return -ENOSYS;

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

	memcpy(&args[index], buf, data_len);

	ret = __spi_hid_command(spi, hidcmd, reportID,
				reportType, args, args_len, NULL, 0);
	if (ret) {
		dev_err(&spi->dev, "failed to set a report to device.\n");
		return ret;
	}

	return data_len;
}


static int i2c_hid_set_power(struct i2c_client *client, int power_state)
{
	struct i2c_hid *shid = i2c_get_clientdata(client);
	int ret;

	i2c_hid_dbg(shid, "%s\n", __func__);

	ret = __i2c_hid_command(client, &hid_set_power_cmd, power_state,
				0, NULL, 0, NULL, 0);
	if (ret)
		dev_err(&client->dev, "failed to change power setting.\n");

	return ret;
}

static int spi_hid_hwreset(struct spi_device *spi)
{
	struct spi_hid *shid = spi_get_drvdata(spi);
	int ret;

	spi_hid_dbg(shid, "%s\n", __func__);

	/*
	 * This prevents sending feature reports while the device is
	 * being reset. Otherwise we may lose the reset complete
	 * interrupt.
	 */
	mutex_lock(&shid->reset_lock);

	ret = spi_hid_set_power(spi, SPI_HID_PWR_ON);
	if (ret)
		goto out_unlock;

	spi_hid_dbg(shid, "resetting...\n");

	ret = spi_hid_command(spi, &hid_reset_cmd, NULL, 0);
	if (ret) {
		dev_err(&spi->dev, "failed to reset device.\n");
		spi_hid_set_power(spi, SPI_HID_PWR_SLEEP);
	}

 out_unlock:
	mutex_unlock(&shid->reset_lock);
	return ret;
}

static void spi_hid_get_input(struct spi_hid *shid)
{
	int ret, ret_size;
	int size = le16_to_cpu(shid->hdesc.wMaxInputLength);

	if (size > shid->bufsize)
		size = shid->bufsize;

	ret = spi_read(shid->spi, shid->inbuf, size);
	if (ret != size) {
		if (ret < 0)
			return;

		dev_err(&shid->spi->dev, "%s: got %d data instead of %d\n",
			__func__, ret, size);
		return;
	}

	ret_size = shid->inbuf[0] | shid->inbuf[1] << 8;

	if (!ret_size) {
		/* host or device initiated RESET completed */
		if (test_and_clear_bit(SPI_HID_RESET_PENDING, &shid->flags))
			wake_up(&shid->wait);
		return;
	}

	if (ret_size > size) {
		dev_err(&shid->spi->dev, "%s: incomplete report (%d/%d)\n",
			__func__, size, ret_size);
		return;
	}

	spi_hid_dbg(shid, "input: %*ph\n", ret_size, shid->inbuf);

	if (test_bit(SPI_HID_STARTED, &shid->flags))
		hid_input_report(shid->hid, HID_INPUT_REPORT, shid->inbuf + 2,
				 ret_size - 2, 1);

	return;
}

static irqreturn_t spi_hid_irq(int irq, void *dev_id)
{
	struct spi_hid *shid = dev_id;

	if (test_bit(SPI_HID_READ_PENDING, &shid->flags))
		return IRQ_HANDLED;

	spi_hid_get_input(shid);

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
	struct i2c_device *spi = hid->driver_data;
	struct spi_hid *shid = spi_get_drvdata(spi);
	unsigned int size, ret_size;

	size = spi_hid_get_report_length(report);
	if (spi_hid_get_report(spi,
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
	struct spi_device *spi = hid->driver_data;
	struct spi_hid *shid = spi_get_drvdata(spi);
	u8 *inbuf = kzalloc(shid->bufsize, GFP_KERNEL);

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
			    &hid->report_enum[HID_FEATURE_REPORT].report_list, list)
		spi_hid_init_report(report, inbuf, shid->bufsize);

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
	kfree(shid->argsbuf);
	kfree(shid->cmdbuf);
	shid->inbuf = NULL;
	shid->rawbuf = NULL;
	shid->cmdbuf = NULL;
	shid->argsbuf = NULL;
	shid->bufsize = 0;
}


module_spi_driver(spi_hid_driver);

MODULE_DESCRIPTION("HID over SPI core driver");
MODULE_AUTHOR("Joël Krähemann <jkraehemann@gmail.com>");
MODULE_LICENSE("GPL");
