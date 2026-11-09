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

#include "mtk_vcodec_mem.h"

struct mtk_vcu_queue *mtk_vcu_dec_init(struct device *dev)
{
	struct mtk_vcu_queue *vcu_queue;

	pr_debug("Allocate new vcu queue !\n");
	vcu_queue = kzalloc(sizeof(struct mtk_vcu_queue), GFP_KERNEL);
	if (vcu_queue == NULL) {
		pr_err("Allocate new vcu queue fail!\n");
		return NULL;
	}

	vcu_queue->mem_ops = &vb2_dma_contig_memops;
	vcu_queue->dev = dev;
	vcu_queue->num_buffers = 0;
	vcu_queue->map_buf = 0;
	vcu_queue->map_type = 0;
	mutex_init(&vcu_queue->mmap_lock);

	/*
	 * PORT 5.10->6.18: set up the fake vb2 queue/buffer that the new
	 * vb2_mem_ops->alloc() ABI requires. vb2_dc_alloc() reads
	 * vb->vb2_queue->{dma_attrs,dma_dir,non_coherent_mem}; a zeroed queue
	 * (dma_attrs=0, non_coherent_mem=false) plus BIDIRECTIONAL dma_dir and
	 * the real device matches the old behaviour (plain coherent DMA alloc).
	 */
	vcu_queue->fake_vb2_q.dev = dev;
	vcu_queue->fake_vb2_q.dma_dir = DMA_BIDIRECTIONAL;
	vcu_queue->fake_vb.vb2_queue = &vcu_queue->fake_vb2_q;

	return vcu_queue;
}

void mtk_vcu_dec_release(struct mtk_vcu_queue *vcu_queue)
{
	struct mtk_vcu_mem *vcu_buffer;
	unsigned int buffer;

	mutex_lock(&vcu_queue->mmap_lock);
	pr_debug("Release vcu queue !\n");
	if (vcu_queue->num_buffers != 0) {
		for (buffer = 0; buffer < vcu_queue->num_buffers; buffer++) {
			vcu_buffer = &vcu_queue->bufs[buffer];
			vcu_queue->mem_ops->put(vcu_buffer->mem_priv);
			vcu_queue->bufs[buffer].mem_priv = NULL;
			vcu_queue->bufs[buffer].size = 0;
		}
	}
	mutex_unlock(&vcu_queue->mmap_lock);
	kfree(vcu_queue);
	vcu_queue = NULL;
}

void *mtk_vcu_get_buffer(struct mtk_vcu_queue *vcu_queue,
	    struct mem_obj *mem_buff_data)
{
	void *cook, *dma_addr;
	struct mtk_vcu_mem *vcu_buffer;
	unsigned int buffers;

	mutex_lock(&vcu_queue->mmap_lock);

	buffers = vcu_queue->num_buffers;
	if (mem_buff_data->len > DEC_ALLOCATE_MAX_BUFFER_SIZE ||
		mem_buff_data->len == 0U || buffers >= DEC_MAX_BUFFER) {
		pr_err("Get buffer fail: buffer len = %llu num_buffers = %d !!\n",
			mem_buff_data->len, buffers);
		mutex_unlock(&vcu_queue->mmap_lock);
		return ERR_PTR(-EINVAL);
	}

	vcu_buffer = &vcu_queue->bufs[buffers];
	/* PORT 5.10->6.18: alloc() now takes (vb, dev, size) */
	vcu_buffer->mem_priv = vcu_queue->mem_ops->alloc(&vcu_queue->fake_vb,
		vcu_queue->dev, mem_buff_data->len);
	vcu_buffer->size = mem_buff_data->len;
	if (IS_ERR(vcu_buffer->mem_priv)) {
		mutex_unlock(&vcu_queue->mmap_lock);
		goto free;
	}

	/* PORT 5.10->6.18: vaddr()/cookie() now take a leading vb arg */
	cook = vcu_queue->mem_ops->vaddr(&vcu_queue->fake_vb,
		vcu_buffer->mem_priv);
	dma_addr = vcu_queue->mem_ops->cookie(&vcu_queue->fake_vb,
		vcu_buffer->mem_priv);

	mem_buff_data->iova = *(dma_addr_t *)dma_addr;
	mem_buff_data->va = (unsigned long)cook;
	vcu_queue->num_buffers++;
	mutex_unlock(&vcu_queue->mmap_lock);

	return vcu_buffer->mem_priv;

free:
	vcu_queue->mem_ops->put(vcu_buffer->mem_priv);

	return ERR_PTR(-ENOMEM);
}

int mtk_vcu_free_buffer(struct mtk_vcu_queue *vcu_queue,
	    struct mem_obj *mem_buff_data)
{
	struct mtk_vcu_mem *vcu_buffer;
	void *cook, *dma_addr;
	unsigned int buffer, num_buffers, last_buffer;
	int ret = -EINVAL;

	mutex_lock(&vcu_queue->mmap_lock);
	num_buffers = vcu_queue->num_buffers;
	if (num_buffers != 0U) {
		for (buffer = 0; buffer < num_buffers; buffer++) {
			vcu_buffer = &vcu_queue->bufs[buffer];
			/* PORT 5.10->6.18: vaddr()/cookie() now take a leading vb arg */
			cook = vcu_queue->mem_ops->vaddr(&vcu_queue->fake_vb,
				vcu_buffer->mem_priv);
			dma_addr = vcu_queue->mem_ops->cookie(&vcu_queue->fake_vb,
				vcu_buffer->mem_priv);

			if (mem_buff_data->va == (unsigned long)cook &&
				mem_buff_data->iova == *(dma_addr_t *)dma_addr &&
				mem_buff_data->len == vcu_buffer->size) {
				pr_debug("Free buff = %d pa = %llx va = %llx, queue_num = %d\n",
					buffer, mem_buff_data->iova, mem_buff_data->va,
					num_buffers);
				vcu_queue->mem_ops->put(vcu_buffer->mem_priv);
				last_buffer = num_buffers - 1U;
				if (last_buffer != buffer)
					vcu_queue->bufs[buffer] =
					    vcu_queue->bufs[last_buffer];
				vcu_queue->bufs[last_buffer].mem_priv = NULL;
				vcu_queue->bufs[last_buffer].size = 0;
				vcu_queue->num_buffers--;
				ret = 0;
				break;
			}
		}
	}
	mutex_unlock(&vcu_queue->mmap_lock);

	if (ret != 0)
		pr_err("Can not free memory va %llx iova %llx len %llu!\n",
			mem_buff_data->va, mem_buff_data->iova, mem_buff_data->len);

	return ret;
}
