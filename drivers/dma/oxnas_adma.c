/*
 * drivers/dma/oxnas_adma.c
 *
 * Copyright (C) 2008 Oxford Semiconductor Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/dmaengine.h>
#include <linux/platform_device.h>
#include <asm/dma.h>

/* MODULE API */
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Oxford Semiconductor Ltd.");

typedef struct oxnas_adma_device {
	struct dma_device common;
	struct platform_device *platform_device;
} oxnas_adma_device_t;

typedef struct oxnas_adma_channel {
	struct dma_chan common;
	oxnas_dma_channel_t *oxnas_channel;
	/* Need a queue for pending descriptors */
	/* May need a queue for completed descriptors that haven't been acked yet */
} oxnas_adma_channel_t;

typedef struct oxnas_adma_desc {
	struct dma_async_tx_descriptor async_desc;
	size_t len;
	dma_addr_t src_adr;
	dma_addr_t dst_adr;
} oxnas_adma_desc_t;

static int __devexit oxnas_adma_remove(struct platform_device *dev)
{
	oxnas_adma_device_t *oxnas_adma_device = platform_get_drvdata(dev);
	struct dma_device *dma_device = &oxnas_adma_device->common;
	struct dma_chan *channel, *_channel;

	dma_async_device_unregister(dma_device);

	list_for_each_entry_safe(channel, _channel, &dma_device->channels, device_node) {
		list_del(&channel->device_node);
		kfree(channel);
	}
	kfree(oxnas_adma_device);

	return 0;
}

static void oxnas_adma_set_src(dma_addr_t addr, struct dma_async_tx_descriptor *tx, int index)
{
	oxnas_adma_desc_t *desc = container_of(tx, oxnas_adma_desc_t, async_desc);
	desc->src_adr = addr;
}

static void oxnas_adm_set_dest(dma_addr_t addr, struct dma_async_tx_descriptor *tx, int index)
{
	oxnas_adma_desc_t *desc = container_of(tx, oxnas_adma_desc_t, async_desc);
	desc->dst_adr = addr;
}

static void oxnas_dma_callback(
	oxnas_dma_channel_t *channel,
	oxnas_callback_arg_t arg,
	oxnas_dma_callback_status_t status,
	u16 checksum,
	int interrupt_count)
{
	oxnas_adma_desc_t *desc = (oxnas_adma_desc_t*)arg;

	/* Use cookies to record that this descriptor's transfer has completed */

	/* Store the completion status with the descriptor */

	/* If there is a queued descriptor, start its transfer now - if that's
	   possible from a DMA callback - with the callback arg updated */
}

static dma_cookie_t oxnas_adma_submit_tx(struct dma_async_tx_descriptor *tx)
{
	oxnas_adma_desc_t *desc = container_of(tx, oxnas_adma_desc_t, async_desc);
	oxnas_adma_channel_t *channel = container_of(tx->chan, oxnas_adma_channel_t, common);
	dma_cookie_t cookie;

	if (oxnas_dma_set(channel->oxnas_channel,
					  (unsigned char*)desc->src_adr,
					  desc->len,
					  (unsigned char*)desc->dst_adr,
					  OXNAS_DMA_MODE_INC,
					  OXNAS_DMA_MODE_INC,
					  0, 0)) {
		return -1;
	}

	/* Allocate a cookie for this descriptor */
	cookie = -1;

	/* Be careful to syn. properly with DMA callback here */
	if (oxnas_dma_is_active(channel->oxnas_channel)) {
		/* Queue the new descriptor to be started when current transfer completes */
	} else {
		/* Start the new transfer */
		oxnas_dma_set_callback(channel->oxnas_channel, oxnas_dma_callback, desc)
		oxnas_dma_start(channel->oxnas_channel);
	}

	return cookie;
}

/** Allocate a DMA channel and prepare it for memory to memory transfers. Could
 *  preallocate descriptors here
 */
static int oxnas_adma_alloc_chan_resources(struct dma_chan *chan)
{
	oxnas_adma_channel_t *channel = container_of(chan, oxnas_adma_channel_t, common);

	channel->oxnas_channel = oxnas_dma_request(0);
	if (!channel->oxnas_channel) {
		return 0;
	}

	/* Pretend we've allocated one descriptor */
	return 1;
}

static void oxnas_adma_free_chan_resources(struct dma_chan *chan)
{
	oxnas_adma_channel_t *channel = container_of(chan, oxnas_adma_channel_t, common);
	oxnas_dma_free(channel->oxnas_channel);

	/* May need to free leftover descriptors here as well */
}

/** Poll for the DMA channel's active status. There can be multiple transfers
 *  queued with the DMA channel identified by cookies, so should be checking
 *  lists containing all pending transfers and all completed transfers that have
 *  not yet been polled for completion
 */
static enum dma_status oxnas_adma_is_tx_complete(
	struct dma_chan *chan,
	dma_cookie_t     cookie,
	dma_cookie_t    *last,
	dma_cookie_t    *used)
{
	oxnas_adma_channel_t *channel = container_of(chan, oxnas_adma_channel_t, common);

	/* Use cookies to report completion status */

	return oxnas_dma_is_active(channel->oxnas_channel) ? DMA_IN_PROGRESS : DMA_SUCCESS;
}

/** To push outstanding transfers to h/w. This should use the list of pending
 *  transfers identified by cookies to select the next transfer and pass this to
 *  the hardware
 */
static void oxnas_adma_issue_pending(struct dma_chan *chan)
{
	/* If there isn't a transfer in progress and one is queued start it now,
	   being careful to sync. with DMA callback function */
}

static void oxnas_adma_dependency_added(struct dma_chan *chan)
{
	/* What is supposed to happen here? */
}

/** Allocate descriptors capable of mapping the requested length of memory */
static struct dma_async_tx_descriptor *oxnas_adma_prep_dma_memcpy(struct dma_chan *chan, size_t len, int int_en)
{
	oxnas_adma_desc_t *desc = kzalloc(sizeof(oxnas_adma_desc_t), GFP_KERNEL);
	if (unlikely(!desc)) {
		return NULL;
	}

	desc->async_desc.tx_set_src  = oxnas_adma_set_src;
	desc->async_desc.tx_set_dest = oxnas_adm_set_dest;
	desc->async_desc.tx_submit   = oxnas_adma_submit_tx;
	desc->len = len;

	return &desc->async_desc;
}

static int enumerate_dma_channels(struct dma_device *dma_device)
{
	int i;

	dma_device->chancnt = 3;

	for (i = 0; i < dma_device->chancnt; i++) {
		oxnas_adma_channel_t *channel = kzalloc(sizeof(oxnas_adma_channel_t), GFP_KERNEL);
		if (!channel) {
			dma_device->chancnt = i;
			break;
		}

		channel->common.device = dma_device;
		list_add_tail(&channel->common.device_node, &dma_device->channels);
	}

	return dma_device->chancnt;
}

static int __devinit oxnas_adma_probe(struct platform_device *platform_device)
{
	oxnas_adma_device_t *oxnas_adma_device;
	struct dma_device *dma_device;

	oxnas_adma_device = kzalloc(sizeof(oxnas_adma_device_t), GFP_KERNEL);
	if (!oxnas_adma_device) {
		return -ENOMEM;
	}

	oxnas_adma_device->platform_device = platform_device;
	dma_device = &oxnas_adma_device->common;

	platform_set_drvdata(platform_device, oxnas_adma_device);

	INIT_LIST_HEAD(&dma_device->channels);
	enumerate_dma_channels(dma_device);

	dma_cap_set(DMA_MEMCPY, dma_device->cap_mask);
	dma_device->device_alloc_chan_resources = oxnas_adma_alloc_chan_resources;
	dma_device->device_free_chan_resources = oxnas_adma_free_chan_resources;
	dma_device->device_is_tx_complete = oxnas_adma_is_tx_complete;
	dma_device->device_issue_pending = oxnas_adma_issue_pending;
	dma_device->device_dependency_added = oxnas_adma_dependency_added;
	dma_device->dev = &platform_device->dev;
	dma_device->device_prep_dma_memcpy = oxnas_adma_prep_dma_memcpy;

	dma_async_device_register(dma_device);

	return 0;
}

static struct platform_driver oxnas_adma_driver = {
	.probe		= oxnas_adma_probe,
	.remove		= oxnas_adma_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "oxnas-adma",
	},
};

static int __init oxnas_adma_init_module(void)
{
	return platform_driver_register(&oxnas_adma_driver);
}

module_init(oxnas_adma_init_module);

static void __exit oxnas_adma_exit_module(void)
{
	platform_driver_unregister(&oxnas_adma_driver);
	return;
}

module_exit(oxnas_adma_exit_module);
