#ifndef __SPIHID_H
#define __SPIHID_H

/*
 *  Copyright (c) 2017 Joël Krähemann
 */

#include <linux/types.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/timer.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/input.h>

/*  API provided by hid-core.c for SPI HID drivers */
void spihid_close(struct hid_device *hid);
int spihid_open(struct hid_device *hid);
void spihid_init_reports(struct hid_device *hid);
int spihid_get_power(struct hid_device *hid);
void spihid_put_power(struct hid_device *hid);

/* iofl flags */
#define SPIHID_CTRL_RUNNING	1
#define SPIHID_OUT_RUNNING	2
#define SPIHID_IN_RUNNING	3
#define SPIHID_RESET_PENDING	4
#define SPIHID_SUSPENDED	5
#define SPIHID_CLEAR_HALT	6
#define SPIHID_DISCONNECTED	7
#define SPIHID_STARTED		8
#define SPIHID_KEYS_PRESSED	10
#define SPIHID_NO_BANDWIDTH	11
#define SPIHID_RESUME_RUNNING	12

/*
 * SPI-specific HID struct, to be pointed to
 * from struct hid_device->driver_data
 */

struct spihid_device {
	struct hid_device *hid;						/* pointer to corresponding HID dev */
	
	struct spi_device *device;                                      /* SPI device */
	struct spi_bitbang *bitbang;

	struct spi_transfer *transfer;
	struct spi_message *message;

	char *inbuf;                                                    /* Input buffer */
	dma_addr_t inbuf_dma;                                           /* Input buffer dma */

	struct hid_control_fifo ctrl[HID_CONTROL_FIFO_SIZE];  		/* Control fifo */
	unsigned char ctrlhead, ctrltail;                               /* Control fifo head & tail */
	char *ctrlbuf;                                                  /* Control buffer */
	dma_addr_t ctrlbuf_dma;                                         /* Control buffer dma */
	unsigned long last_ctrl;					/* record of last output for timeouts */

	struct hid_output_fifo out[HID_CONTROL_FIFO_SIZE];              /* Output pipe fifo */
	unsigned char outhead, outtail;                                 /* Output pipe fifo head & tail */
	char *outbuf;                                                   /* Output buffer */
	dma_addr_t outbuf_dma;                                          /* Output buffer dma */
	unsigned long last_out;						/* record of last output for timeouts */
	
	spinlock_t lock;						/* fifo spinlock */
	unsigned long iofl;                                             /* I/O flags (CTRL_RUNNING, OUT_RUNNING) */
	struct timer_list io_retry;                                     /* Retry timer */
	unsigned long stop_retry;                                       /* Time to give up, in jiffies */
	unsigned int retry_delay;                                       /* Delay length in ms */
	struct work_struct reset_work;                                  /* Task context for resets */
	wait_queue_head_t wait;						/* For sleeping */
};

#define	spihid_to_spi_dev(spihid_dev) \
	to_spi_device(spihid_dev->dev.parent->parent)

#endif
