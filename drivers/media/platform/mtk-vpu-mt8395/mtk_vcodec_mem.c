// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2017 MediaTek Inc.
 * Author: Yunfei Dong <yunfei.dong@mediatek.com>
 */
#include "mtk_vcodec_mem.h"

#if ENABLE_GCE
static void *vcu_cmdq_mbox_buf_alloc(struct cmdq_client *client,
				   size_t size, dma_addr_t *pa_out)
{
	struct device *dev;
	dma_addr_t dma_addr;
	void *mem_priv = NULL;

	dev = client->chan->mbox->dev;

	if(dev)
		mem_priv = dma_alloc_coherent(dev, PAGE_SIZE, &dma_addr, GFP_KERNEL);

	if (!mem_priv) {
		pr_err("%s %d alloc alloc dma buffer faild dev:0x%p", __func__, __LINE__, dev);
		dump_stack();
		return NULL;
	}

	*pa_out = dma_addr;
	pr_debug("[VCU] %s(%d)  va_base:%p pa_base:%p dma_addr:%llx\n",
		__func__, __LINE__,mem_priv, pa_out, dma_addr);

	return mem_priv;
}


static void vcu_cmdq_mbox_buf_free(struct cmdq_client *cl, void *va, dma_addr_t pa)
{

	pr_debug("[VCU] %s(%d) va:%p pa_base:%p\n",
		__func__, __LINE__, va, pa);
	dma_free_coherent(cl->chan->mbox->dev, PAGE_SIZE, va, pa);
}
#endif

struct mtk_vcu_queue *mtk_vcu_mem_init(struct device *dev,
	struct device *cmdq_dev)
{
	struct mtk_vcu_queue *vcu_queue = NULL;

	vcu_queue = kzalloc(sizeof(struct mtk_vcu_queue), GFP_KERNEL);
	if (vcu_queue == NULL) {
		pr_info("Allocate new vcu queue fail!\n");
		return NULL;
	}
	INIT_LIST_HEAD(&vcu_queue->pa_pages.list);
	vcu_queue->mem_ops = &vb2_dma_contig_memops;
	vcu_queue->dev = dev;
	vcu_queue->cmdq_dev = cmdq_dev;
	vcu_queue->num_buffers = 0;
	vcu_queue->map_buf_pa = 0;
	mutex_init(&vcu_queue->mmap_lock);
	mutex_init(&vcu_queue->dev_lock);

	vcu_queue->vb_queue.mem_ops         = &vb2_dma_contig_memops;
	vcu_queue->vb_queue.dma_dir         = 0;
	vcu_queue->vb_queue.dma_attrs       = 0;
	vcu_queue->vb_queue.gfp_flags       = 0;
/*
 *	vcu_queue->vb_queue.type            = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
 *	vcu_queue->vb_queue.io_modes        = VB2_DMABUF | VB2_MMAP;
 *	vcu_queue->vb_queue.drv_priv        = ctx;
 *	vcu_queue->vb_queue.buf_struct_size = sizeof(struct mtk_video_dec_buf);
 *	vcu_queue->vb_queue.ops             = &mtk_vdec_vb2_ops;
 *	vcu_queue->vb_queue.mem_ops         = &vb2_dma_contig_memops;
 *	vcu_queue->vb_queue.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
 *	vcu_queue->vb_queue.lock            = &ctx->q_mutex;
 *	vcu_queue->vb_queue.dev             = &ctx->dev->plat_dev->dev;
 *	vcu_queue->vb_queue.allow_zero_bytesused = 1;
 *	ret = vb2_queue_init(&vcu_queue->vb_queue);
 *	if (ret) {
 *		mtk_v4l2_err("Failed to initialize videobuf2 queue(capture)");
 *	}
 */
	return vcu_queue;
}

void mtk_vcu_mem_release(struct mtk_vcu_queue *vcu_queue, struct cmdq_client *client)
{
	struct mtk_vcu_mem *vcu_buffer;
	unsigned int buffer;
#if ENABLE_GCE
	struct vcu_pa_pages *tmp;
	struct list_head *p, *q;
#endif

	mutex_lock(&vcu_queue->mmap_lock);
	pr_debug("Release vcu queue !\n");
	if (vcu_queue->num_buffers != 0) {
		for (buffer = 0; buffer < vcu_queue->num_buffers; buffer++) {
			vcu_buffer = &vcu_queue->bufs[buffer];
			if (vcu_buffer->dbuf == NULL)
				vcu_queue->mem_ops->put(vcu_buffer->mem_priv);
			else
				fput(vcu_buffer->dbuf->file);

			pr_debug("Free %d dbuf = %p size = %d mem_priv = %lx ref_cnt = %d\n",
				 buffer, vcu_buffer->dbuf,
				 (unsigned int)vcu_buffer->size,
				 (unsigned long)vcu_buffer->mem_priv,
				 atomic_read(&vcu_buffer->ref_cnt));
		}
	}
#if ENABLE_GCE
	list_for_each_safe(p, q, &vcu_queue->pa_pages.list) {
		tmp = list_entry(p, struct vcu_pa_pages, list);
		vcu_cmdq_mbox_buf_free(client,
			(void *)(unsigned long)tmp->kva,
			(dma_addr_t)tmp->pa);
		pr_debug("Free cmdq pa %llx ref_cnt = %d\n", tmp->pa,
			atomic_read(&tmp->ref_cnt));
		list_del(p);
		kfree(tmp);
	}
#endif
	mutex_unlock(&vcu_queue->mmap_lock);
	kfree(vcu_queue);
	vcu_queue = NULL;
}

void *mtk_vcu_set_buffer(struct mtk_vcu_queue *vcu_queue,
	struct mem_obj *mem_buff_data, struct vb2_buffer *src_vb,
	struct vb2_buffer *dst_vb)
{
	struct mtk_vcu_mem *vcu_buffer;
	unsigned int num_buffers, plane;
	unsigned int buffer;
	dma_addr_t *dma_addr = NULL;
	struct dma_buf *dbuf = NULL;
	int op;

	mutex_lock(&vcu_queue->mmap_lock);
	pr_debug("[%s] %d iova = %llx src_vb = %p dst_vb = %p\n",
		__func__, vcu_queue->num_buffers, mem_buff_data->iova,
		src_vb, dst_vb);

	num_buffers = vcu_queue->num_buffers;
	if (mem_buff_data->len > CODEC_ALLOCATE_MAX_BUFFER_SIZE ||
		mem_buff_data->len == 0U || num_buffers >= CODEC_MAX_BUFFER) {
		pr_info("Set buffer fail: buffer len = %u num_buffers = %d !!\n",
			   mem_buff_data->len, num_buffers);
		mutex_unlock(&vcu_queue->mmap_lock);
		return ERR_PTR(-EINVAL);
	}

	for (buffer = 0; buffer < num_buffers; buffer++) {
		vcu_buffer = &vcu_queue->bufs[buffer];
		if (mem_buff_data->iova == (u64)vcu_buffer->iova) {
			atomic_inc(&vcu_buffer->ref_cnt);
			mutex_unlock(&vcu_queue->mmap_lock);
			return (void *)vcu_buffer->iova;
		}
	}

	vcu_buffer = &vcu_queue->bufs[num_buffers];
	if (dbuf == NULL && src_vb != NULL)
		for (plane = 0; plane < src_vb->num_planes; plane++) {
			dma_addr = src_vb->vb2_queue->mem_ops->cookie(src_vb,
				src_vb->planes[plane].mem_priv);
			if (*dma_addr == mem_buff_data->iova) {
				dbuf = src_vb->planes[plane].dbuf;
				vcu_buffer->size = src_vb->planes[plane].length;
				vcu_buffer->mem_priv = src_vb->planes[plane].mem_priv;
				op = DMA_TO_DEVICE;
				pr_debug("src size = %d mem_buff_data len = %d\n",
					(unsigned int)vcu_buffer->size,
					(unsigned int)mem_buff_data->len);
			}
		}
	if (dbuf == NULL && dst_vb != NULL)
		for (plane = 0; plane < dst_vb->num_planes; plane++) {
			dma_addr = dst_vb->vb2_queue->mem_ops->cookie(dst_vb,
				dst_vb->planes[plane].mem_priv);
			if (*dma_addr == mem_buff_data->iova) {
				dbuf = dst_vb->planes[plane].dbuf;
				vcu_buffer->size = dst_vb->planes[plane].length;
				vcu_buffer->mem_priv = dst_vb->planes[plane].mem_priv;
				op = DMA_FROM_DEVICE;
				pr_debug("dst size = %d mem_buff_data len = %d\n",
					(unsigned int)vcu_buffer->size,
					(unsigned int)mem_buff_data->len);
			}
		}

	if (dbuf == NULL) {
		mutex_unlock(&vcu_queue->mmap_lock);
		pr_debug("Set buffer not found: buffer len = %u iova = %llx !!\n",
			   mem_buff_data->len, mem_buff_data->iova);
		return ERR_PTR(-ENOMEM);
	}
	vcu_buffer->dbuf = dbuf;
	vcu_buffer->iova = *dma_addr;
	get_file(dbuf->file);
	vcu_queue->num_buffers++;
	atomic_set(&vcu_buffer->ref_cnt, 1);
	mutex_unlock(&vcu_queue->mmap_lock);

	pr_debug("[%s, %d] Num_buffers = %d iova = %llx dbuf = %p size = %d mem_priv = %llx, ref_cnt:%d\n",
		__func__, __LINE__, vcu_queue->num_buffers, mem_buff_data->iova,
		vcu_buffer->dbuf, (unsigned int)vcu_buffer->size,
		(unsigned long)vcu_buffer->mem_priv, atomic_read(&vcu_buffer->ref_cnt));

	return (void *)vcu_buffer->iova;
}

void *mtk_vcu_get_buffer(struct mtk_vcu_queue *vcu_queue,
						 struct mem_obj *mem_buff_data)
{
	void *cook, *dma_addr;
	struct mtk_vcu_mem *vcu_buffer;
	unsigned int buffers;

	mutex_lock(&vcu_queue->mmap_lock);
	buffers = vcu_queue->num_buffers;
	if (mem_buff_data->len > CODEC_ALLOCATE_MAX_BUFFER_SIZE ||
		mem_buff_data->len == 0U || buffers >= CODEC_MAX_BUFFER) {
		pr_info("Get buffer fail: buffer len = %u num_buffers = %d !!\n",
			   mem_buff_data->len, buffers);
		mutex_unlock(&vcu_queue->mmap_lock);
		return ERR_PTR(-EINVAL);
	}

	vcu_buffer = &vcu_queue->bufs[buffers];
	vcu_buffer->vb.vb2_queue = &vcu_queue->vb_queue;
	vcu_buffer->mem_priv = vcu_queue->mem_ops->alloc(&vcu_buffer->vb, vcu_queue->dev,
		mem_buff_data->len);
	vcu_buffer->size = mem_buff_data->len;
	vcu_buffer->dbuf = NULL;
	if (IS_ERR_OR_NULL(vcu_buffer->mem_priv)) {
		mutex_unlock(&vcu_queue->mmap_lock);
		return ERR_PTR(-ENOMEM);
	}

	cook = vcu_queue->mem_ops->vaddr(&vcu_buffer->vb, vcu_buffer->mem_priv);
	dma_addr = vcu_queue->mem_ops->cookie(&vcu_buffer->vb, vcu_buffer->mem_priv);

	mem_buff_data->iova = *(dma_addr_t *)dma_addr;
	vcu_buffer->iova = *(dma_addr_t *)dma_addr;
	mem_buff_data->va = CODEC_MSK((unsigned long)cook);
	mem_buff_data->pa = 0;
	vcu_queue->num_buffers++;
	mutex_unlock(&vcu_queue->mmap_lock);
	atomic_set(&vcu_buffer->ref_cnt, 1);

	pr_debug("[%s] Num_buffers = %d iova = %llx va = %llx size = %d mem_priv = %llx, ref_cnt:%d\n",
		__func__, vcu_queue->num_buffers, mem_buff_data->iova,
		mem_buff_data->va, (unsigned int)vcu_buffer->size,
		(unsigned long)vcu_buffer->mem_priv, atomic_read(&vcu_buffer->ref_cnt));

	return vcu_buffer->mem_priv;
}

void *mtk_vcu_get_page(struct mtk_vcu_queue *vcu_queue,
						 struct cmdq_client *client,
						 struct mem_obj *mem_buff_data)
{
	void *mem_priv = NULL;
#if ENABLE_GCE
	dma_addr_t temp_pa = 0;
	struct vcu_pa_pages *tmp;
	int ret;

	mem_priv = vcu_cmdq_mbox_buf_alloc(client, GFP_KERNEL, &temp_pa);
	if (mem_priv == NULL) {
		pr_info("%s failed to create cmdq buffer\n", __func__);
		return ERR_PTR(-ENOMEM);
	}

	tmp = kmalloc(sizeof(struct vcu_pa_pages), GFP_KERNEL);
	if (!tmp) {
		vcu_cmdq_mbox_buf_free(client, (void *)(unsigned long)mem_priv, temp_pa);
		pr_info("%s failed to create vcu_pa_pages buffer\n", __func__);
		return ERR_PTR(-ENOMEM);
	}

	mutex_lock(&vcu_queue->mmap_lock);
	tmp->pa = temp_pa;
	mem_buff_data->pa = temp_pa;
	tmp->kva = (unsigned long)mem_priv;
	mem_buff_data->va = CODEC_MSK((unsigned long)mem_priv);
	mem_buff_data->iova = 0;
	atomic_set(&tmp->ref_cnt, 1);
	list_add_tail(&tmp->list, &vcu_queue->pa_pages.list);
	mutex_unlock(&vcu_queue->mmap_lock);
	pr_debug("%s client=%p mem_priv=%p temp_pa=%p list:%p tmp:%p ref_cnt:%d\n",
		__func__, client, mem_priv, temp_pa, &tmp->list, tmp, tmp->ref_cnt);
#endif
	return mem_priv;
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
			if (vcu_buffer->dbuf != NULL)
				continue;
			cook = vcu_queue->mem_ops->vaddr(&vcu_buffer->vb, vcu_buffer->mem_priv);
			dma_addr =
				vcu_queue->mem_ops->cookie(&vcu_buffer->vb, vcu_buffer->mem_priv);

			if (mem_buff_data->va == CODEC_MSK((unsigned long)cook) &&
				mem_buff_data->iova == *(dma_addr_t *)dma_addr &&
				mem_buff_data->len == vcu_buffer->size &&
				atomic_read(&vcu_buffer->ref_cnt) == 1) {
				pr_debug("Free buff = %d iova = %llx va = %llx, queue_num = %d\n",
						 buffer, mem_buff_data->iova,
						 mem_buff_data->va,
						 num_buffers);
				vcu_queue->mem_ops->put(vcu_buffer->mem_priv);
				atomic_dec(&vcu_buffer->ref_cnt);

				last_buffer = num_buffers - 1U;
				if (last_buffer != buffer)
					vcu_queue->bufs[buffer] =
						vcu_queue->bufs[last_buffer];
				vcu_queue->bufs[last_buffer].mem_priv = NULL;
				vcu_queue->bufs[last_buffer].size = 0;
				vcu_queue->bufs[last_buffer].dbuf = NULL;
				vcu_queue->num_buffers--;
				ret = 0;
				break;
			}
		}
	}
	mutex_unlock(&vcu_queue->mmap_lock);

	if (ret != 0)
		pr_info("Can not free memory va %llx iova %llx len %u!\n",
			   mem_buff_data->va, mem_buff_data->iova,
			   mem_buff_data->len);

	return ret;
}

int mtk_vcu_free_page(struct mtk_vcu_queue *vcu_queue,
						struct cmdq_client *cl,
						struct mem_obj *mem_buff_data)
{
	int ret = -EINVAL;
#if ENABLE_GCE
	struct vcu_pa_pages *tmp;
	struct list_head *p, *q;
	struct mtk_vcu *vcu_dev;

	vcu_dev = (struct mtk_vcu *)vcu_queue->vcu;
	mutex_lock(&vcu_queue->mmap_lock);
	list_for_each_safe(p, q, &vcu_queue->pa_pages.list) {
		tmp = list_entry(p, struct vcu_pa_pages, list);
		if (tmp->pa == mem_buff_data->pa &&
			CODEC_MSK(tmp->kva) == mem_buff_data->va &&
			atomic_read(&tmp->ref_cnt) == 1) {
			ret = 0;
			vcu_cmdq_mbox_buf_free(cl, (void *)(unsigned long)tmp->kva,
								(dma_addr_t)mem_buff_data->pa);
			atomic_dec(&tmp->ref_cnt);
			list_del(p);
			kfree(tmp);
			break;
		}
	}
	mutex_unlock(&vcu_queue->mmap_lock);

	if (ret != 0)
		pr_info("Can not free memory va %llx pa %llx len %u!\n",
			   mem_buff_data->va, mem_buff_data->pa,
			   mem_buff_data->len);
#endif
	return ret;
}

void mtk_vcu_buffer_ref_dec(struct mtk_vcu_queue *vcu_queue,
	void *mem_priv)
{
	struct mtk_vcu_mem *vcu_buffer;
	unsigned int buffer, num_buffers;//last_buffer;

	mutex_lock(&vcu_queue->mmap_lock);
	num_buffers = vcu_queue->num_buffers;
	for (buffer = 0; buffer < num_buffers; buffer++) {
		vcu_buffer = &vcu_queue->bufs[buffer];
		if (vcu_buffer->iova == (dma_addr_t)mem_priv) {
			if (atomic_read(&vcu_buffer->ref_cnt) > 0) {
				atomic_dec(&vcu_buffer->ref_cnt);
				if (atomic_read(&vcu_buffer->ref_cnt) == 0
					&& vcu_buffer->dbuf != NULL) {
					fput(vcu_buffer->dbuf->file);
					vcu_queue->bufs[buffer].mem_priv =
						vcu_queue->bufs[num_buffers-1].mem_priv;
					vcu_queue->bufs[buffer].dbuf =
						vcu_queue->bufs[num_buffers-1].dbuf;
					vcu_queue->bufs[buffer].iova =
						vcu_queue->bufs[num_buffers-1].iova;
					vcu_queue->bufs[buffer].size =
						vcu_queue->bufs[num_buffers-1].size;
					vcu_queue->bufs[buffer].ref_cnt =
						vcu_queue->bufs[num_buffers-1].ref_cnt;
					vcu_queue->num_buffers--;
					break;
				}
			} else
				pr_info("[VCU][Error] %s fail\n", __func__);
		}
	}
	mutex_unlock(&vcu_queue->mmap_lock);
}

void vcu_io_buffer_cache_sync(struct device *dev,
	struct dma_buf *dbuf, int op)
{
	struct dma_buf_attachment *buf_att;
	struct sg_table *sgt;

	buf_att = dma_buf_attach(dbuf, dev);
	if (IS_ERR(buf_att)) {
		pr_info("failed to attach dmabuf\n");
		return;
	}
	sgt = dma_buf_map_attachment(buf_att, op);
	if (IS_ERR_OR_NULL(sgt)) {
		pr_info("%s Error getting dmabuf scatterlist %d\n", __func__, sgt);
		dma_buf_detach(dbuf, buf_att);
		return;
	}
	dma_sync_sg_for_device(dev, sgt->sgl, sgt->orig_nents, op);
	dma_buf_unmap_attachment(buf_att, sgt, op);
	dma_buf_detach(dbuf, buf_att);
}

int vcu_buffer_flush_all(struct device *dev, struct mtk_vcu_queue *vcu_queue)
{
	struct mtk_vcu_mem *vcu_buffer;
	unsigned int buffer, num_buffers;
	struct dma_buf *dbuf = NULL;
	unsigned long flags = 0;

	mutex_lock(&vcu_queue->mmap_lock);

	num_buffers = vcu_queue->num_buffers;
	if (num_buffers == 0U) {
		mutex_unlock(&vcu_queue->mmap_lock);
		return 0;
	}
	for (buffer = 0; buffer < num_buffers; buffer++) {
		vcu_buffer = &vcu_queue->bufs[buffer];
		pr_debug("Cache clean %s buffer=%d iova=%lx size=%d num=%d\n",
			(vcu_buffer->dbuf == NULL) ? "working" : "io",
			buffer, (unsigned int long)vcu_buffer->iova,
			(unsigned int)vcu_buffer->size, num_buffers);

		if (vcu_buffer->dbuf == NULL)
			dbuf = vcu_queue->mem_ops->get_dmabuf(
				&vcu_buffer->vb, vcu_buffer->mem_priv, flags);
		else
			dbuf = vcu_buffer->dbuf;

		vcu_io_buffer_cache_sync(dev, dbuf, DMA_TO_DEVICE);

		if (vcu_buffer->dbuf == NULL)
			dma_buf_put(dbuf);
	}
	mutex_unlock(&vcu_queue->mmap_lock);

	return 0;
}

int vcu_buffer_cache_sync(struct device *dev, struct mtk_vcu_queue *vcu_queue,
	dma_addr_t dma_addr, size_t size, int op)
{
	struct mtk_vcu_mem *vcu_buffer;
	unsigned int num_buffers = 0;
	unsigned int buffer = 0;
	struct dma_buf *dbuf = NULL;
	unsigned long flags = 0;

	mutex_lock(&vcu_queue->mmap_lock);

	num_buffers = vcu_queue->num_buffers;
	if (num_buffers == 0U) {
		pr_info("Cache %s buffer fail, iova = %lx, size = %d, vcu no buffers\n",
			(op == DMA_TO_DEVICE) ? "flush" : "invalidate",
			(unsigned long)dma_addr, (unsigned int)size);
		mutex_unlock(&vcu_queue->mmap_lock);
		return -1;
	}

	for (buffer = 0; buffer < num_buffers; buffer++) {
		vcu_buffer = &vcu_queue->bufs[buffer];
		if ((dma_addr + size) <=
			(vcu_buffer->iova + vcu_buffer->size) &&
			dma_addr >= vcu_buffer->iova) {
			pr_debug("Cache %s %s buffer iova=%lx range=%d (%lx %d)\n",
				(op == DMA_TO_DEVICE) ?
				"clean" : "invalidate",
				(vcu_buffer->dbuf == NULL) ?
				"working" : "io",
				(unsigned long)dma_addr, (unsigned int)size,
				(unsigned long)vcu_buffer->iova,
				(unsigned int)vcu_buffer->size);

			if (vcu_buffer->dbuf == NULL)
				dbuf = vcu_queue->mem_ops->get_dmabuf(
					&vcu_buffer->vb, vcu_buffer->mem_priv, flags);
			else
				dbuf = vcu_buffer->dbuf;

			vcu_io_buffer_cache_sync(dev, dbuf, op);

			if (vcu_buffer->dbuf == NULL)
				dma_buf_put(dbuf);

			mutex_unlock(&vcu_queue->mmap_lock);
			return 0;
		}
	}

	pr_info("Cache %s buffer fail, iova = %lx, size = %d\n",
		(op == DMA_TO_DEVICE) ? "flush" : "invalidate",
		(unsigned long)dma_addr, (unsigned int)size);

	mutex_unlock(&vcu_queue->mmap_lock);
	return -1;
}

static int vcu_dmabuf_get_dma_addr(struct device *dev,
					   struct mtk_vcu_queue *vcu_queue,
					   int share_fd, dma_addr_t *dma_addr, int *is_sec,
					   int add_fcnt)
{
	struct dma_buf *buf = dma_buf_get(share_fd);		/* f_cnt +1 */
	struct dma_buf_attachment *attach;
	struct sg_table *table;
	int ret = 0;
	int sec = 0;
	const char *reason;

	if (IS_ERR(buf)) {
		reason = "buf_get";
		ret = (int)PTR_ERR(buf);
		goto out;
	}
	attach = dma_buf_attach(buf, dev);
	if (IS_ERR(attach)) {
		reason = "buf_attach";
		ret = (int)PTR_ERR(attach);
		goto put_buf;
	}
	table = dma_buf_map_attachment(attach, 0);
	if (IS_ERR(table)) {
		reason = "buf_map";
		ret = (int)PTR_ERR(table);
		goto detach_buf;
	}

	*dma_addr = sg_dma_address(table->sgl);
	sec = sg_page(table->sgl) ? 0 : 1;
	if (is_sec)
		*is_sec = sec;

	if (add_fcnt) {
		pr_info("%s: %s, share_id %d, f_cnt +1\n", __func__,
			 dev_name(dev), share_fd);
		get_dma_buf(buf);
	}

	dma_buf_unmap_attachment(attach, table, 0);
detach_buf:
	dma_buf_detach(buf, attach);
put_buf:
	dma_buf_put(buf);

out:
	if (ret)
		pr_info("%s: %s fail(%s:%d) share_id %d\n", __func__,
		       dev_name(dev), reason, ret, share_fd);
	else
		pr_info("%s: %s, share_id %d, dma_addr 0x%lx(sec %d, add_cnt %d)\n",
			 __func__, dev_name(dev),
			 share_fd, (unsigned long)(*dma_addr),
			 sec, add_fcnt);
	return ret;
}

int mtk_vcu_get_dma_addr(struct device *dev, struct mtk_vcu_queue *vcu_queue,
				   int share_fd, dma_addr_t *dma_addr, int *is_sec)
{
	if (!dma_addr)
		return -EINVAL;
	return vcu_dmabuf_get_dma_addr(dev, vcu_queue, share_fd, dma_addr, is_sec,
					       0);
}
