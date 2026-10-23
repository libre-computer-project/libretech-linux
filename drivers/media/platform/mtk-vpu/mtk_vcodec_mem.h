/*
 * Copyright (c) 2017 MediaTek Inc.
 * Author: Yunfei Dong <yunfei.dong@mediatek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef MTK_VCODEC_MEM_H
#define MTK_VCODEC_MEM_H

#include <media/videobuf2-dma-contig.h>
#include <uapi/linux/mtk_vcu_controls.h>
#include <linux/slab.h>

#define DEC_MAX_BUFFER 32U
#define DEC_ALLOCATE_MAX_BUFFER_SIZE 0x1B00000UL

/**
 * struct mtk_vcu_mem - memory buffer allocated in kernel
 *
 * @mem_priv:	vb2_dc_buf
 * @size:	allocated buffer size
 */
struct mtk_vcu_mem {
	void *mem_priv;
	size_t size;
};

/**
 * struct mtk_vcu_queue - the allocated buffer queue
 *
 * @vcu:	struct mtk_vcu
 * @mmap_lock:	the lock to protect allocated buffer
 * @dev:	device
 * @num_buffers:	allocated buffer number
 * @mem_ops:	the file operation of memory allocated
 * @bufs:	store the information of allocated buffers
 */
struct mtk_vcu_queue {
	void *vcu;
	struct vb2_alloc_ctx *alloc_ctx;
	struct mutex mmap_lock;
	struct device *dev;
	unsigned int num_buffers;
	const struct vb2_mem_ops *mem_ops;
	struct mtk_vcu_mem bufs[DEC_MAX_BUFFER];
	int map_buf;
	int map_type;
	/*
	 * PORT 5.10->6.18: vb2_mem_ops->alloc()/vaddr()/cookie() gained a
	 * leading "struct vb2_buffer *vb" argument, and vb2_dc_alloc() now
	 * dereferences vb->vb2_queue->{dma_attrs,dma_dir,non_coherent_mem}.
	 * The vendor uses the mem_ops directly to allocate scratch DMA
	 * buffers without a real vb2 queue/buffer, so provide a minimal
	 * embedded queue+buffer with sane defaults to satisfy the new ABI.
	 */
	struct vb2_queue fake_vb2_q;
	struct vb2_buffer fake_vb;
};

/**
 * mtk_vcu_dec_init - just init vcu_queue
 *
 * @dev:	vcu device.
 *
 * Return:	Return NULL if it is failed.
 * otherwise it is vcu queue to store the allocated buffer
 **/
struct mtk_vcu_queue *mtk_vcu_dec_init(struct device *dev);

/**
 * mtk_vcu_dec_release - just release the vcu_queue
 *
 * @vcu_queue:	the queue to store allocated buffer.
 *
 * Return: void
 **/
void mtk_vcu_dec_release(struct mtk_vcu_queue *vcu_queue);

/**
 * mtk_vcu_get_buffer - get the allocated buffer iova/va
 *
 * @vcu_queue:	the queue to store allocated buffer.
 * @mem_buff_data:	store iova/va.
 *
 * Return: Return real address if it is ok, otherwise failed
 **/
void *mtk_vcu_get_buffer(struct mtk_vcu_queue *vcu_queue, struct mem_obj *mem_buff_data);

/**
 * mtk_vcu_free_buffer - just free unused buffer iova/va
 *
 * @vcu_queue:	the queue to store allocated buffer.
 * @mem_buff_data:	store iova/va to free.
 *
 * Return:	Return 0 if it is ok, otherwise failed
 **/
int mtk_vcu_free_buffer(struct mtk_vcu_queue *vcu_queue, struct mem_obj *mem_buff_data);

#endif
