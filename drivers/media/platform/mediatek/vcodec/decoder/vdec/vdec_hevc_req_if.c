// SPDX-License-Identifier: GPL-2.0
/*
 * Single-core stateless HEVC over SCP IPI, for MT8183 with the open kukui SCP
 * firmware. The firmware programs the VDEC (its HEVC handler,
 * kukui_vdec_hevc.c); this backend fills the VSI, sends START, waits for the
 * decode interrupt and sends END. The shape is vdec_h264_req_if.c's: one IPI
 * client on SCP_IPI_VDEC_H264, told apart by codec_type 'S265'.
 *
 * Contract (VSI layout, message meaning, status words):
 * mediatek/software/codec/decoder/hevc/design/kukui-scp-hevc-ipi.md, and the
 * firmware's kukui_hevc_abi.h, which this file's VSI mirrors.
 *
 * Only an SCP image that publishes MTK_VDEC_FORMAT_HEVC_FRAME gets an S265
 * context (mtk_vcodec_get_supported_formats), so the stock kukui SCP never
 * sees this backend.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "../mtk_vcodec_dec.h"
#include "../../common/mtk_vcodec_intr.h"
#include "../vdec_drv_base.h"
#include "../vdec_drv_if.h"
#include "../vdec_vpu_if.h"

#define HEVC_SC_MV_NUM		32
#define HEVC_SC_VSI_VERSION	1

/* KUKUI_HEVC_VSI_BEGIN -- the host layout check extracts this block. */
struct vdec_hevc_sc_dec {
	u32 vsi_version;
	u32 vsi_size;
	u32 status;
	u32 pics;
	u32 lcu_xy;
	u32 err;
	u32 done;
	u32 irq;
	u32 crc[8];
};

struct vdec_hevc_sc_bs {
	u64 dma;
	u32 bytes;
	u32 alloc;
};

struct vdec_hevc_sc_fb {
	u64 y_dma;
	u64 c_dma;
	u64 mv_dma;
	u64 vdec_fb_va;
};

struct vdec_hevc_sc_hdr {
	u32 sc0_off;
	u32 slice_sc_off;
	u32 slice_nal_off;
	u32 data_byte_offset;
	u32 ep_bytes;
	u32 hdr_rbsp_bits;
	u32 pic_bytes;
	u32 reserved;
};

struct vdec_hevc_sc_pic {
	u32 pic_w;
	u32 pic_h;
	u32 buf_w;
	u32 buf_h;
	u32 cap_num_planes;
	u32 reserved;
};

struct vdec_hevc_sc_ref {
	u64 y_dma;
	u64 c_dma;
	u64 mv_dma;
};

/**
 * struct vdec_hevc_sc_vsi - shared memory between the SCP and the host
 * @dec:     SCP-W: version, size, status and the completion words of END
 * @bs:      OUTPUT buffer (AP-W, as is everything below)
 * @fb:      CAPTURE buffer and its colocated-MV buffer
 * @hdr:     slice positions located in the OUTPUT buffer
 * @pic:     picture and buffer size
 * @ref:     CAPTURE / MV buffers of decode_params.dpb[i]
 * @sps:     V4L2_CID_STATELESS_HEVC_SPS, verbatim
 * @pps:     V4L2_CID_STATELESS_HEVC_PPS, verbatim
 * @slice:   V4L2_CID_STATELESS_HEVC_SLICE_PARAMS entry 0, verbatim
 * @decode:  V4L2_CID_STATELESS_HEVC_DECODE_PARAMS, verbatim
 * @scaling: V4L2_CID_STATELESS_HEVC_SCALING_MATRIX, verbatim
 */
struct vdec_hevc_sc_vsi {
	struct vdec_hevc_sc_dec dec;
	struct vdec_hevc_sc_bs bs;
	struct vdec_hevc_sc_fb fb;
	struct vdec_hevc_sc_hdr hdr;
	struct vdec_hevc_sc_pic pic;
	struct vdec_hevc_sc_ref ref[V4L2_HEVC_DPB_ENTRIES_NUM_MAX];
	struct v4l2_ctrl_hevc_sps sps;
	struct v4l2_ctrl_hevc_pps pps;
	struct v4l2_ctrl_hevc_slice_params slice;
	struct v4l2_ctrl_hevc_decode_params decode;
	struct v4l2_ctrl_hevc_scaling_matrix scaling;
} __aligned(8);
/* KUKUI_HEVC_VSI_END */

/* The numbers kukui_hevc_abi.h asserts on the firmware side. */
static_assert(offsetof(struct vdec_hevc_sc_vsi, bs) == 64);
static_assert(offsetof(struct vdec_hevc_sc_vsi, sps) == 552);
static_assert(offsetof(struct vdec_hevc_sc_vsi, scaling) == 1264);
static_assert(sizeof(struct vdec_hevc_sc_vsi) == 2264);

#define HEVC_SC_AP_OFF	offsetof(struct vdec_hevc_sc_vsi, bs)

/**
 * struct vdec_hevc_sc_inst - single-core HEVC instance
 * @num_pics: pictures sent
 * @ctx:      the decode context
 * @vpu:      IPI client (SCP_IPI_VDEC_H264, codec_type S265)
 * @vsi_ctx:  local VSI; the AP-W part is copied to the SCP before START
 * @mv_buf:   colocated-MV buffer per CAPTURE buffer index, made on first use
 * @mv_sz:    size of each @mv_buf
 * @end_bad:  END answers that were not 0
 */
struct vdec_hevc_sc_inst {
	unsigned int num_pics;
	struct mtk_vcodec_dec_ctx *ctx;
	struct vdec_vpu_inst vpu;
	struct vdec_hevc_sc_vsi vsi_ctx;
	struct mtk_vcodec_mem mv_buf[HEVC_SC_MV_NUM];
	unsigned int mv_sz;
	unsigned int end_bad;
};

static int hevc_slice_strict = 1;
module_param(hevc_slice_strict, int, 0644);
MODULE_PARM_DESC(hevc_slice_strict,
		 "HEVC (SCP single-core): 1 (default) = fail the picture when the firmware's END reports it incomplete; 0 = log it and return the picture");

static void *vdec_hevc_sc_ctrl(struct mtk_vcodec_dec_ctx *ctx, u32 id)
{
	struct v4l2_ctrl *ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl, id);

	if (!ctrl || !ctrl->p_cur.p)
		return ERR_PTR(-EINVAL);
	return ctrl->p_cur.p;
}

static unsigned int vdec_hevc_sc_mv_size(unsigned int w, unsigned int h)
{
	/* vdec_hevc_req_multi_if.c vdec_hevc_get_mv_buf_size() */
	return 64 * ((w / 16) * (h / 16) + 8);
}

/*
 * The firmware sizes the picture from the SPS and writes the CTB-aligned
 * recon into the negotiated CAPTURE buffer and its colocated-MV buffer.
 * Refuse before START when that size is larger than picinfo, so a client
 * that skipped the resolution change does not DMA past those buffers.
 * HEVC coding-tree blocks are 16, 32 or 64; anything else is not a picture
 * this backend can bound.
 */
static int vdec_hevc_sc_sps_fits(const struct v4l2_ctrl_hevc_sps *sps,
				 u32 buf_w, u32 buf_h)
{
	u32 log2_ctb, ctb, w, h;

	log2_ctb = sps->log2_min_luma_coding_block_size_minus3 + 3 +
		   sps->log2_diff_max_min_luma_coding_block_size;
	if (log2_ctb < 4 || log2_ctb > 6)
		return -EINVAL;
	ctb = 1U << log2_ctb;
	w = ALIGN(sps->pic_width_in_luma_samples, ctb);
	h = ALIGN(sps->pic_height_in_luma_samples, ctb);
	if (!w || !h || w > buf_w || h > buf_h)
		return -EINVAL;
	return 0;
}

static void vdec_hevc_sc_free_mv(struct vdec_hevc_sc_inst *inst)
{
	int i;

	for (i = 0; i < HEVC_SC_MV_NUM; i++)
		if (inst->mv_buf[i].va)
			mtk_vcodec_mem_free(inst->ctx, &inst->mv_buf[i]);
}

static u64 vdec_hevc_sc_mv(struct vdec_hevc_sc_inst *inst, unsigned int idx)
{
	struct mtk_vcodec_mem *mem;
	unsigned int sz;

	if (idx >= HEVC_SC_MV_NUM)
		return 0;
	sz = vdec_hevc_sc_mv_size(inst->ctx->picinfo.buf_w,
				  inst->ctx->picinfo.buf_h);
	if (sz != inst->mv_sz) {
		vdec_hevc_sc_free_mv(inst);
		inst->mv_sz = sz;
	}
	mem = &inst->mv_buf[idx];
	if (!mem->va) {
		mem->size = sz;
		if (mtk_vcodec_mem_alloc(inst->ctx, mem)) {
			mtk_vdec_err(inst->ctx, "hevc mv buf %u alloc failed", idx);
			return 0;
		}
	}
	return mem->dma_addr;
}

/*
 * The slice positions of kukui-scp-hevc-ipi.md "Bitstream positions": the
 * first start code, the first VCL NAL (its first_slice_segment_in_pic_flag
 * must be set), and the emulation-prevention bytes between its NAL header
 * and slice data, which data_byte_offset counts from the NAL header.
 */
static int vdec_hevc_sc_locate(struct vdec_hevc_sc_inst *inst, const u8 *buf,
			       u32 size, u32 dbo, struct vdec_hevc_sc_hdr *h)
{
	bool have_sc0 = false;
	u32 i, nal, end, zeros, ep;

	memset(h, 0, sizeof(*h));
	for (i = 0; i + 3 < size; i++) {
		if (buf[i] || buf[i + 1] || buf[i + 2] != 1)
			continue;
		if (!have_sc0) {
			h->sc0_off = i;
			have_sc0 = true;
		}
		nal = i + 3;
		if (((buf[nal] >> 1) & 0x3f) >= 32) {
			i += 2;
			continue;
		}
		if (nal + 2 >= size || !(buf[nal + 2] & 0x80)) {
			mtk_vdec_err(inst->ctx, "hevc: first VCL NAL at %u is not a first slice segment", i);
			return -EINVAL;
		}
		h->slice_sc_off = i;
		h->slice_nal_off = nal;
		break;
	}
	if (!h->slice_nal_off)
		return -EINVAL;
	if (dbo < 3 || dbo > size - h->slice_nal_off) {
		mtk_vdec_err(inst->ctx, "hevc: data_byte_offset %u outside the slice (%u bytes)",
			     dbo, size - h->slice_nal_off);
		return -EINVAL;
	}
	end = h->slice_nal_off + dbo;
	zeros = 0;
	ep = 0;
	for (i = h->slice_nal_off + 2; i < end; i++) {
		if (zeros >= 2 && buf[i] == 3) {
			ep++;
			zeros = 0;
			continue;
		}
		zeros = buf[i] ? 0 : zeros + 1;
	}
	if (8 * (dbo - 2 - ep) == 0)
		return -EINVAL;
	h->data_byte_offset = dbo;
	h->ep_bytes = ep;
	h->hdr_rbsp_bits = 8 * (dbo - 2 - ep);
	h->pic_bytes = size - h->sc0_off;
	return 0;
}

static void vdec_hevc_sc_fill_refs(struct vdec_hevc_sc_inst *inst)
{
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;
	const struct v4l2_ctrl_hevc_decode_params *dp = &inst->vsi_ctx.decode;
	struct vb2_queue *vq;
	struct vb2_buffer *vb;
	int i;

	memset(inst->vsi_ctx.ref, 0, sizeof(inst->vsi_ctx.ref));
	vq = v4l2_m2m_get_vq(ctx->m2m_ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	for (i = 0; i < dp->num_active_dpb_entries && i < V4L2_HEVC_DPB_ENTRIES_NUM_MAX; i++) {
		struct vdec_hevc_sc_ref *r = &inst->vsi_ctx.ref[i];

		vb = vb2_find_buffer(vq, dp->dpb[i].timestamp);
		if (!vb) {
			mtk_vdec_debug(ctx, "hevc dpb %d ts %llu: no capture buffer",
				       i, dp->dpb[i].timestamp);
			continue;
		}
		r->y_dma = vb2_dma_contig_plane_dma_addr(vb, 0);
		if (ctx->q_data[MTK_Q_DATA_DST].fmt->num_planes == 2)
			r->c_dma = vb2_dma_contig_plane_dma_addr(vb, 1);
		else
			r->c_dma = r->y_dma + ctx->picinfo.fb_sz[0];
		r->mv_dma = vdec_hevc_sc_mv(inst, vb->index);
	}
}

static int vdec_hevc_sc_fill(struct vdec_hevc_sc_inst *inst, struct mtk_vcodec_mem *bs,
			     struct vdec_fb *fb, struct mtk_video_dec_buf *dst)
{
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;
	struct vdec_hevc_sc_vsi *v = &inst->vsi_ctx;
	struct mtk_video_dec_buf *src = container_of(bs, struct mtk_video_dec_buf, bs_buffer);
	const void *sps, *pps, *slice, *dec, *sm;
	int err;

	sps = vdec_hevc_sc_ctrl(ctx, V4L2_CID_STATELESS_HEVC_SPS);
	pps = vdec_hevc_sc_ctrl(ctx, V4L2_CID_STATELESS_HEVC_PPS);
	slice = vdec_hevc_sc_ctrl(ctx, V4L2_CID_STATELESS_HEVC_SLICE_PARAMS);
	dec = vdec_hevc_sc_ctrl(ctx, V4L2_CID_STATELESS_HEVC_DECODE_PARAMS);
	sm = vdec_hevc_sc_ctrl(ctx, V4L2_CID_STATELESS_HEVC_SCALING_MATRIX);
	if (IS_ERR(sps) || IS_ERR(pps) || IS_ERR(slice) || IS_ERR(dec) || IS_ERR(sm))
		return -EINVAL;

	memcpy(&v->sps, sps, sizeof(v->sps));
	memcpy(&v->pps, pps, sizeof(v->pps));
	memcpy(&v->slice, slice, sizeof(v->slice));
	memcpy(&v->decode, dec, sizeof(v->decode));
	memcpy(&v->scaling, sm, sizeof(v->scaling));

	err = vdec_hevc_sc_sps_fits(&v->sps, ctx->picinfo.buf_w,
				    ctx->picinfo.buf_h);
	if (err) {
		mtk_vdec_err(ctx, "hevc SPS %ux%u does not fit CAPTURE %ux%u",
			     v->sps.pic_width_in_luma_samples,
			     v->sps.pic_height_in_luma_samples,
			     ctx->picinfo.buf_w, ctx->picinfo.buf_h);
		return err;
	}

	if (!bs->va)
		return -EINVAL;
	err = vdec_hevc_sc_locate(inst, bs->va, bs->size, v->slice.data_byte_offset, &v->hdr);
	if (err)
		return err;

	v->bs.dma = bs->dma_addr;
	v->bs.bytes = bs->size;
	v->bs.alloc = vb2_plane_size(&src->m2m_buf.vb.vb2_buf, 0);

	v->fb.y_dma = fb->base_y.dma_addr;
	v->fb.c_dma = fb->base_c.dma_addr;
	v->fb.mv_dma = vdec_hevc_sc_mv(inst, dst->m2m_buf.vb.vb2_buf.index);
	v->fb.vdec_fb_va = (u64)(uintptr_t)fb;
	if (!v->fb.mv_dma)
		return -ENOMEM;

	v->pic.pic_w = ctx->picinfo.pic_w;
	v->pic.pic_h = ctx->picinfo.pic_h;
	v->pic.buf_w = ctx->picinfo.buf_w;
	v->pic.buf_h = ctx->picinfo.buf_h;
	v->pic.cap_num_planes = ctx->q_data[MTK_Q_DATA_DST].fmt->num_planes;

	vdec_hevc_sc_fill_refs(inst);
	return 0;
}

static int vdec_hevc_sc_init(struct mtk_vcodec_dec_ctx *ctx)
{
	struct vdec_hevc_sc_inst *inst;
	const struct vdec_hevc_sc_vsi *vsi;
	int err;

	inst = kzalloc(sizeof(*inst), GFP_KERNEL);
	if (!inst)
		return -ENOMEM;

	inst->ctx = ctx;
	inst->vpu.id = SCP_IPI_VDEC_H264;
	inst->vpu.ctx = ctx;
	inst->vpu.codec_type = V4L2_PIX_FMT_HEVC_SLICE;

	err = vpu_dec_init(&inst->vpu);
	if (err) {
		mtk_vdec_err(ctx, "vdec_hevc (scp) init err=%d", err);
		goto error_free_inst;
	}

	/* The firmware states its VSI layout at INIT; refuse any other. */
	vsi = inst->vpu.vsi;
	if (vsi->dec.vsi_version != HEVC_SC_VSI_VERSION ||
	    vsi->dec.vsi_size != sizeof(struct vdec_hevc_sc_vsi)) {
		mtk_vdec_err(ctx, "vdec_hevc (scp): firmware VSI v%u size %u, expected v%u size %zu",
			     vsi->dec.vsi_version, vsi->dec.vsi_size,
			     HEVC_SC_VSI_VERSION, sizeof(struct vdec_hevc_sc_vsi));
		err = -EINVAL;
		goto error_deinit;
	}

	ctx->drv_handle = inst;
	return 0;

error_deinit:
	vpu_dec_deinit(&inst->vpu);
error_free_inst:
	kfree(inst);
	return err;
}

static void vdec_hevc_sc_deinit(void *h_vdec)
{
	struct vdec_hevc_sc_inst *inst = h_vdec;

	if (inst->end_bad)
		dev_info(&inst->ctx->dev->plat_dev->dev,
			 "vdec hevc (scp): ctx %d pictures %u, END not 0 on %u\n",
			 inst->ctx->id, inst->num_pics, inst->end_bad);
	vpu_dec_deinit(&inst->vpu);
	vdec_hevc_sc_free_mv(inst);
	kfree(inst);
}

static int vdec_hevc_sc_decode(void *h_vdec, struct mtk_vcodec_mem *bs,
			       struct vdec_fb *unused, bool *res_chg)
{
	struct vdec_hevc_sc_inst *inst = h_vdec;
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;
	struct vdec_vpu_inst *vpu = &inst->vpu;
	struct vdec_hevc_sc_vsi *shared = vpu->vsi;
	struct mtk_video_dec_buf *src_buf_info;
	struct mtk_video_dec_buf *dst_buf_info;
	struct vdec_fb *fb;
	u32 data[3];
	int err;

	*res_chg = false;
	/* bs NULL means flush decoder */
	if (!bs)
		return vpu_dec_reset(vpu);

	fb = ctx->dev->vdec_pdata->get_cap_buffer(ctx);
	if (!fb) {
		mtk_vdec_err(ctx, "fb buffer is NULL");
		return -ENOMEM;
	}
	src_buf_info = container_of(bs, struct mtk_video_dec_buf, bs_buffer);
	dst_buf_info = container_of(fb, struct mtk_video_dec_buf, frame_buffer);
	v4l2_m2m_buf_copy_metadata(&src_buf_info->m2m_buf.vb,
				   &dst_buf_info->m2m_buf.vb, true);

	inst->num_pics++;
	err = vdec_hevc_sc_fill(inst, bs, fb, dst_buf_info);
	if (err)
		goto err_out;

	data[0] = inst->vsi_ctx.bs.bytes;
	data[1] = inst->vsi_ctx.hdr.data_byte_offset;
	data[2] = inst->vsi_ctx.hdr.hdr_rbsp_bits;

	/* The AP-W part only: dec is the firmware's. */
	memcpy((u8 *)shared + HEVC_SC_AP_OFF, (u8 *)&inst->vsi_ctx + HEVC_SC_AP_OFF,
	       sizeof(inst->vsi_ctx) - HEVC_SC_AP_OFF);
	err = vpu_dec_start(vpu, data, 3);
	if (err)
		goto err_out;

	err = mtk_vcodec_wait_for_done_ctx(ctx, MTK_INST_IRQ_RECEIVED,
					   WAIT_INTR_TIMEOUT_MS, 0);
	if (err)
		goto err_out;

	err = vpu_dec_end(vpu);
	memcpy(&inst->vsi_ctx.dec, &shared->dec, sizeof(inst->vsi_ctx.dec));
	if (err) {
		inst->end_bad++;
		mtk_vdec_err(ctx, "hevc pic %u END status %d: lcu 0x%08x err 0x%08x done 0x%08x irq 0x%08x",
			     inst->num_pics, err, inst->vsi_ctx.dec.lcu_xy,
			     inst->vsi_ctx.dec.err, inst->vsi_ctx.dec.done,
			     inst->vsi_ctx.dec.irq);
		if (hevc_slice_strict)
			goto err_out;
	}
	mtk_vdec_debug(ctx, "hevc pic %u crc %08x %08x", inst->num_pics,
		       inst->vsi_ctx.dec.crc[0], inst->vsi_ctx.dec.crc[4]);
	return 0;

err_out:
	/* A positive err is the firmware's ack status (design doc table). */
	mtk_vdec_err(ctx, "hevc pic %u err=%d", inst->num_pics, err);
	return err > 0 ? -EIO : err;
}

static void vdec_hevc_sc_get_pic_info(struct vdec_hevc_sc_inst *inst,
				      struct vdec_pic_info *pic)
{
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;

	ctx->picinfo.buf_w = ALIGN(ctx->picinfo.pic_w, VCODEC_DEC_ALIGNED_64);
	ctx->picinfo.buf_h = ALIGN(ctx->picinfo.pic_h, VCODEC_DEC_ALIGNED_64);
	ctx->picinfo.fb_sz[0] = ctx->picinfo.buf_w * ctx->picinfo.buf_h;
	ctx->picinfo.fb_sz[1] = ctx->picinfo.fb_sz[0] >> 1;
	*pic = ctx->picinfo;
}

static int vdec_hevc_sc_get_param(void *h_vdec, enum vdec_get_param_type type,
				  void *out)
{
	struct vdec_hevc_sc_inst *inst = h_vdec;
	struct v4l2_rect *cr;

	switch (type) {
	case GET_PARAM_PIC_INFO:
		vdec_hevc_sc_get_pic_info(inst, out);
		break;
	case GET_PARAM_DPB_SIZE:
		*(unsigned int *)out = V4L2_HEVC_DPB_ENTRIES_NUM_MAX;
		break;
	case GET_PARAM_CROP_INFO:
		cr = out;
		cr->left = 0;
		cr->top = 0;
		cr->width = inst->ctx->picinfo.pic_w;
		cr->height = inst->ctx->picinfo.pic_h;
		break;
	default:
		mtk_vdec_err(inst->ctx, "invalid get parameter type=%d", type);
		return -EINVAL;
	}
	return 0;
}

const struct vdec_common_if vdec_hevc_slice_if = {
	.init		= vdec_hevc_sc_init,
	.decode		= vdec_hevc_sc_decode,
	.get_param	= vdec_hevc_sc_get_param,
	.deinit		= vdec_hevc_sc_deinit,
};
