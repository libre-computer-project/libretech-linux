// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 MediaTek Inc.
 * Author: Ping-Hsun Wu <ping-hsun.wu@mediatek.com>
 */

#include <linux/platform_device.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-event.h>
#include <media/videobuf2-dma-contig.h>
#include "mtk-mdp3-m2m.h"
#include "mtk-mdp3-regs.h"

/* Private MediaTek MDP3 control: the WROT programmable color-correction matrix
 * (CCM). A compound s32[MDP_CCM_NELEMS] array carrying the 3x3 matrix + pre/post
 * -add (see mtk-mdp3-m2m.h). Settable per-context via
 *   v4l2-ctl -d <node> --set-ctrl='mtk_mdp_ccm=<15 comma-separated s32>'
 * All-zero (the default) => the WROT keeps its built-in CSC (production byte-
 * identical); any non-zero element activates the ext-matrix for RGB-output CSCs. */
#define V4L2_CID_MTK_MDP_CCM		(V4L2_CID_USER_MTK_MDP_BASE + 0)
#define V4L2_CID_MTK_MDP_RSZ_COEFF_TABLE	(V4L2_CID_USER_MTK_MDP_BASE + 1)

/*
 * 4:4:4 (I444/YV24) output via a HW+CPU hybrid. The mt8195 WROT cannot write 4:4:4
 * (mapColorFormat has an I444 case but the 1:1 datapath never delivers full-H chroma
 * -> SCP watchdog / WROT GCE hang; board-proven, mdp-walls-mt8195.md). Instead the HW
 * produces 4:2:2 (NV12->I422 -- a proven op whose vertical chroma upsample is done in
 * HW), and the CPU does the remaining HORIZONTAL chroma 2x upsample to reach 4:4:4.
 * Genuine HW-assist: the HW performs the Y plane (plus any scale/CSC) and the vertical
 * chroma reconstruction; only the horizontal chroma doubling is CPU. Gated; default on.
 */
static int mdp_kd_444hybrid = 1;
module_param(mdp_kd_444hybrid, int, 0644);
MODULE_PARM_DESC(mdp_kd_444hybrid,
	"mt8195 4:4:4 hybrid: HW 4:2:2 + CPU horizontal chroma upsample (1=on default, 0=off)");

static inline struct mdp_m2m_ctx *file_to_ctx(struct file *filp)
{
	return container_of(file_to_v4l2_fh(filp), struct mdp_m2m_ctx, fh);
}

static inline struct mdp_m2m_ctx *ctrl_to_ctx(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct mdp_m2m_ctx, ctrl_handler);
}

static inline struct mdp_frame *ctx_get_frame(struct mdp_m2m_ctx *ctx,
					      enum v4l2_buf_type type)
{
	if (V4L2_TYPE_IS_OUTPUT(type))
		return &ctx->curr_param.output;
	else
		return &ctx->curr_param.captures[0];
}

static inline void mdp_m2m_ctx_set_state(struct mdp_m2m_ctx *ctx, u32 state)
{
	atomic_or(state, &ctx->curr_param.state);
}

static inline bool mdp_m2m_ctx_is_state_set(struct mdp_m2m_ctx *ctx, u32 mask)
{
	return ((atomic_read(&ctx->curr_param.state) & mask) == mask);
}

static void mdp_m2m_process_done(void *priv, int vb_state)
{
	struct mdp_m2m_ctx *ctx = priv;
	struct vb2_v4l2_buffer *src_vbuf, *dst_vbuf;

	src_vbuf = (struct vb2_v4l2_buffer *)
			v4l2_m2m_src_buf_remove(ctx->m2m_ctx);
	dst_vbuf = (struct vb2_v4l2_buffer *)
			v4l2_m2m_dst_buf_remove(ctx->m2m_ctx);
	ctx->curr_param.frame_no = ctx->frame_count[MDP_M2M_SRC];
	src_vbuf->sequence = ctx->frame_count[MDP_M2M_SRC]++;
	dst_vbuf->sequence = ctx->frame_count[MDP_M2M_DST]++;
	v4l2_m2m_buf_copy_metadata(src_vbuf, dst_vbuf, true);

	v4l2_m2m_buf_done(src_vbuf, vb_state);
	v4l2_m2m_buf_done(dst_vbuf, vb_state);
	v4l2_m2m_job_finish(ctx->mdp_dev->m2m_dev, ctx->m2m_ctx);
}

/* The full decoded per-op img_config dump below emits hundreds of lines per op
 * (every component, every subframe, a 320B hex per comp, ctrls). It is the OLD
 * pre-debugfs oracle, SUPERSEDED by the lossless on-demand debugfs snapshot
 * (mtk-mdp3/cfg8195_snapshot). Kept for the decoded field view but behind its
 * OWN default-off knob so a mdp_kd_debug=1 STREAM can never trigger it (single-op
 * opt-in: set it, run ONE op, clear it). */
static bool mdp_kd_dump_config;
module_param(mdp_kd_dump_config, bool, 0644);
MODULE_PARM_DESC(mdp_kd_dump_config,
	"dump full decoded per-op img_config to dmesg (default 0; single-op use only, verbose)");

/* Whole-img_config oracle dump (mt8195), gated by mdp_kd_dump_config. Prints every
 * component, EVERY subframe (luma/chroma phase int+subpix + tile index), the
 * per-engine udata head, and ctrls[]. For the multi-tile scale RE: with
 * mdp_kernel_direct=0 the SCP fills vpu->config, and this dumps the SCP-computed
 * "tpipe" horizontal tile-split + per-tile phase accumulation we need to derive.
 * Throwaway instrumentation -- not for mainline. */
bool mdp_kd_debug_on(void);
static void mdp_dbg_dump_config(struct mdp_dev *mdp, struct img_config *cfg)
{
	struct img_config_8195 *c;
	u32 i;

	if (!mdp_kd_dump_config || IS_ERR_OR_NULL(cfg))
		return;
	c = &cfg->config_8195;
	pr_info("MDPDBG BEGIN num_comp=%u num_subfrm=%u\n",
		c->num_components, c->num_subfrms);
	for (i = 0; i < c->num_components && i < IMG_MAX_COMPONENTS; i++) {
		struct img_compparam_8195 *p = &c->components[i];
		u32 k;

		pr_info("MDPDBG c%u type=%u id=%u in=%ux%u out=%ux%u crop=(%d,%d %ux%u) totw(in=%u out=%u) nsub=%u outdis=%u byp=%u\n",
			i, p->type, p->id,
			p->frame.in_width, p->frame.in_height,
			p->frame.out_width, p->frame.out_height,
			p->frame.crop.left, p->frame.crop.top,
			p->frame.crop.width, p->frame.crop.height,
			p->frame.in_total_width, p->frame.out_total_width,
			p->num_subfrms, p->frame.output_disable, p->frame.bypass);
		for (k = 0; k < p->num_subfrms && k < IMG_MAX_SUBFRAMES_8195; k++) {
			struct img_comp_subfrm_8195 *s = &p->subfrms[k];

			pr_info("MDPDBG c%u sub%u in[l%d r%d t%d b%d] out[l%d r%d t%d b%d] luma(l%d t%d sl%u st%u) chroma(l%d t%d sl%u st%u) ov%d oh%d tiledis=%u\n",
				i, k, s->in.left, s->in.right, s->in.top, s->in.bottom,
				s->out.left, s->out.right, s->out.top, s->out.bottom,
				s->luma.left, s->luma.top, s->luma.left_subpix, s->luma.top_subpix,
				s->chroma.left, s->chroma.top, s->chroma.left_subpix, s->chroma.top_subpix,
				s->out_vertical, s->out_horizontal, s->tile_disable);
		}
		/* per-engine register-data head (overlaid union; 320 B reaches the
		 * per-tile subfrms[k] -- RSZ subfrms start at ~u[15], +12 u32/tile). */
		print_hex_dump(KERN_INFO, "MDPDBG udata ", DUMP_PREFIX_OFFSET,
			       16, 4, &p->rdma, 320, false);
	}
	for (i = 0; i < c->num_subfrms && i < IMG_MAX_SUBFRAMES_8195; i++) {
		struct img_mmsys_ctrl *mc = &c->ctrls[i];
		u32 j;

		pr_info("MDPDBG ctrl[%u] num_sets=%u\n", i, mc->num_sets);
		for (j = 0; j < mc->num_sets && j < 32; j++)
			pr_info("MDPDBG   set[%u] reg=%#x val=%#x subsys=%u\n", j,
				mc->sets[j].reg, mc->sets[j].value,
				mc->sets[j].subsys_id);
	}
	pr_info("MDPDBG END\n");
}

/*
 * Hybrid NV12 rotate, CPU chroma half: the WROT 3-plane I420 write is broken on
 * mt8183, so the interleaved-CbCr plane is rotated on the CPU. Reads the input UV
 * (hc rows x istride), writes the rotated output UV (hc rows x ostride), copying
 * each 16-bit CbCr pair intact. rev_rows/rev_cols match the HW Y rotation:
 *   hflip -> rev_cols; vflip -> rev_rows; rot180 -> both. Buffers are dma-contig
 * coherent (same as vpu->config), so CPU writes are visible without an explicit
 * sync; the HW writes only the Y plane (disjoint), so the two never race.
 */
static void mdp_hybrid_chroma_rotate(u8 *dst_uv, const u8 *src_uv,
				     u32 wc, u32 hc, u32 istride, u32 ostride,
				     bool rev_rows, bool rev_cols)
{
	u32 r, c;

	for (r = 0; r < hc; r++) {
		const u8 *srow = src_uv + (rev_rows ? (hc - 1 - r) : r) * istride;
		u8 *drow = dst_uv + r * ostride;

		for (c = 0; c < wc; c++) {
			u32 sc = rev_cols ? (wc - 1 - c) : c;

			drow[2 * c]     = srow[2 * sc];		/* Cb */
			drow[2 * c + 1] = srow[2 * sc + 1];	/* Cr */
		}
	}
}

/*
 * Hybrid rot90/270, CPU chroma DIM-SWAP half: the chroma transposes like the frame.
 * Input chroma is wc x hc CbCr pairs (stride istride); output is hc x wc pairs
 * (stride ostride, dim-swapped). Clockwise rot90: out[or][oc] = in[hc-1-oc][or];
 * counter-clockwise rot270 (ccw): out[or][oc] = in[oc][wc-1-or]. Each CbCr pair
 * (16 bit) is copied intact. Matches the HW GREY Y rotate (V4L2 rotate=90 = cw).
 */
static void mdp_hybrid_chroma_swap(u8 *dst_uv, const u8 *src_uv,
				   u32 wc, u32 hc, u32 istride, u32 ostride, bool ccw)
{
	u32 orow, ocol;

	/* output chroma: hc cols x wc rows (frame dims swapped) */
	for (orow = 0; orow < wc; orow++) {
		u8 *drow = dst_uv + orow * ostride;

		for (ocol = 0; ocol < hc; ocol++) {
			u32 sr = ccw ? ocol : (hc - 1 - ocol);
			u32 sc = ccw ? (wc - 1 - orow) : orow;
			const u8 *spair = src_uv + sr * istride + 2 * sc;

			drow[2 * ocol]     = spair[0];	/* Cb */
			drow[2 * ocol + 1] = spair[1];	/* Cr */
		}
	}
}

/*
 * Hybrid 4:4:4 CPU half: horizontal 2x chroma upsample, IN PLACE. The HW wrote a
 * 4:2:2 chroma plane (w/2 samples per row at stride istride, h rows) into a plane
 * buffer sized for 4:4:4 (w samples per row at stride ostride). Expand each row to
 * full width by duplicating columns (nearest -- matches the validated SW reference).
 * Walk BOTTOM-UP and RIGHT-TO-LEFT so an in-place write (offset 2*c) never clobbers a
 * not-yet-read source (offset c): within a row 2*c >= c, and for ostride > istride a
 * higher row's writes (base r'*ostride) sit entirely above row r's source (r*istride).
 * The Y plane is HW-final and untouched; chroma planes are disjoint per-plane buffers.
 */
static void mdp_hybrid_444_hupsample(u8 *plane, u32 w, u32 h,
				     u32 istride, u32 ostride)
{
	u32 wc = w / 2;
	int r, c;

	for (r = (int)h - 1; r >= 0; r--) {
		const u8 *srow = plane + (u32)r * istride;
		u8 *drow = plane + (u32)r * ostride;

		for (c = (int)wc - 1; c >= 0; c--) {
			u8 v = srow[c];

			drow[2 * c]     = v;
			drow[2 * c + 1] = v;
		}
	}
}

/*
 * I420/YV12-dst hybrid CPU post-pass: the HW wrote 2-plane NV12 (Y + interleaved
 * CbCr) into the single output dma-buf; split the CbCr plane into the separate U/V
 * planes. The Y plane [0, ystride*h) is HW-final and untouched. The NV12 UV plane
 * sits at offset ystride*h (h/2 rows x ystride bytes = ystride/2 CbCr pairs/row);
 * I420/YV12 want two half-width planes (ystride/2 bytes/row). Copy the whole UV
 * plane to scratch first (an in-place forward split would clobber not-yet-read
 * source bytes), then write U (Cb, even) + V (Cr, odd): I420 = U-plane then V-plane,
 * YV12 = V-plane then U-plane. Pure byte reshuffle -- bit-exact vs a CPU NV12->I420.
 * The output dma-buf is dma-contig coherent (same as the 4:4:4 hybrid); the HW wrote
 * only the disjoint Y+UV region, so this runs after the HW job with no race.
 */
static int mdp_hybrid_i420_deinterleave(u8 *base, u32 w, u32 h, u32 ystride, bool yv12)
{
	u32 wc = w / 2, hc = h / 2;
	u32 cstride = ystride / 2;		/* I420/YV12 chroma plane pitch */
	u8 *uv = base + (size_t)ystride * h;	/* HW-written NV12 interleaved UV plane */
	u8 *plane0 = uv;			/* first chroma plane in the buffer */
	u8 *plane1 = uv + (size_t)cstride * hc;	/* second chroma plane */
	u8 *cb_plane = yv12 ? plane1 : plane0;	/* U = Cb (even bytes) */
	u8 *cr_plane = yv12 ? plane0 : plane1;	/* V = Cr (odd bytes) */
	size_t uvsz = (size_t)ystride * hc;
	u8 *tmp;
	u32 r, c;

	tmp = kvmalloc(uvsz, GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;
	memcpy(tmp, uv, uvsz);
	for (r = 0; r < hc; r++) {
		const u8 *srow = tmp + (size_t)r * ystride;
		u8 *cbrow = cb_plane + (size_t)r * cstride;
		u8 *crrow = cr_plane + (size_t)r * cstride;

		for (c = 0; c < wc; c++) {
			cbrow[c] = srow[2 * c];		/* Cb -> U */
			crrow[c] = srow[2 * c + 1];	/* Cr -> V */
		}
	}
	kvfree(tmp);
	return 0;
}

/*
 * Multi-planar (YM12/YM21) I420/YV12-dst hybrid CPU post-pass. The HW wrote the
 * 2-plane NV12: Y directly into the real Y-plane dma-buf, and the interleaved
 * CbCr plane into a SCRATCH buffer (uv, uvstride bytes/row, hc rows). Split the
 * scratch CbCr into the two SEPARATE chroma-plane dma-bufs: cb_dst (Cb, even
 * bytes) and cr_dst (Cr, odd bytes), each cstride bytes/row. The caller maps
 * cb_dst/cr_dst to vb2 plane 1/2 in I420 (U,V) or YV12 (V,U) order. Source
 * (scratch) and destinations (plane bufs) are disjoint, so no bounce buffer is
 * needed -- pure byte reshuffle, bit-exact vs a CPU NV12->I420. Void: it cannot
 * fail (all buffers are pre-validated non-NULL by the caller).
 */
static void mdp_hybrid_i420_deinterleave_mp(const u8 *uv, u8 *cb_dst, u8 *cr_dst,
					    u32 w, u32 h, u32 uvstride, u32 cstride)
{
	u32 wc = w / 2, hc = h / 2;
	u32 r, c;

	for (r = 0; r < hc; r++) {
		const u8 *srow = uv + (size_t)r * uvstride;
		u8 *cbrow = cb_dst + (size_t)r * cstride;
		u8 *crrow = cr_dst + (size_t)r * cstride;

		for (c = 0; c < wc; c++) {
			cbrow[c] = srow[2 * c];		/* Cb -> U */
			crrow[c] = srow[2 * c + 1];	/* Cr -> V */
		}
	}
}

/*
 * SCP-tail class #2: 3-plane 4:2:2 dst (YM16/YM61) CPU post-pass. Identical to the
 * multi-planar i420 de-interleave EXCEPT the chroma is FULL height (4:2:2): the HW
 * wrote 2-plane NV16 (Y direct to the real Y-plane, interleaved CbCr to scratch,
 * uvstride bytes/row x h rows). Split the scratch CbCr -> cb_dst (Cb, even) and
 * cr_dst (Cr, odd), each cstride bytes/row x h rows (half-width, full-height). The
 * caller maps cb_dst/cr_dst to vb2 plane 1/2 in YM16 (U,V) or YM61 (V,U) order.
 */
static void mdp_hybrid_nv16_deinterleave_mp(const u8 *uv, u8 *cb_dst, u8 *cr_dst,
					    u32 w, u32 h, u32 uvstride, u32 cstride)
{
	u32 wc = w / 2;
	u32 r, c;

	for (r = 0; r < h; r++) {		/* FULL height (4:2:2), not h/2 */
		const u8 *srow = uv + (size_t)r * uvstride;
		u8 *cbrow = cb_dst + (size_t)r * cstride;
		u8 *crrow = cr_dst + (size_t)r * cstride;

		for (c = 0; c < wc; c++) {
			cbrow[c] = srow[2 * c];		/* Cb -> U */
			crrow[c] = srow[2 * c + 1];	/* Cr -> V */
		}
	}
}

/* mdp_kd_pktreuse: two pre-compute frameparams describe the SAME transform (so
 * the built GCE packet is byte-identical except the src/dst iovas) iff every
 * config-determining field matches. iovas are excluded (they are what reuse
 * patches). Any S_FMT / S_SELECTION / rotation change flips a compared field,
 * so a stale cache naturally misses and rebuilds. */
static bool mdp_reuse_match(const struct img_ipi_frameparam *a,
			    const struct img_ipi_frameparam *b)
{
	return a->type == b->type &&
	       a->inputs[0].flags == b->inputs[0].flags &&
	       a->inputs[0].buffer.usage == b->inputs[0].buffer.usage &&
	       a->outputs[0].flags == b->outputs[0].flags &&
	       a->outputs[0].rotation == b->outputs[0].rotation &&
	       a->outputs[0].buffer.usage == b->outputs[0].buffer.usage &&
	       !memcmp(&a->inputs[0].buffer.format, &b->inputs[0].buffer.format,
		       sizeof(struct img_pix_format)) &&
	       !memcmp(&a->outputs[0].buffer.format, &b->outputs[0].buffer.format,
		       sizeof(struct img_pix_format)) &&
	       !memcmp(&a->outputs[0].crop, &b->outputs[0].crop,
		       sizeof(struct img_crop));
}

/* Drop the cached reuse packet. Waits out any in-flight job first (the
 * completion callback keeps the cmd alive, so it must be idle before we free). */
static void mdp_ctx_reuse_invalidate(struct mdp_m2m_ctx *ctx)
{
	if (!ctx->reuse_cmd)
		return;
	if (refcount_read(&ctx->mdp_dev->job_count))
		wait_event_timeout(ctx->mdp_dev->callback_wq,
				   !refcount_read(&ctx->mdp_dev->job_count), 2 * HZ);
	mdp_cmdq_reuse_free(ctx->reuse_cmd);
	ctx->reuse_cmd = NULL;
}

/*
 * mdp_iova_guard: reject (and log) a job whose imported SOURCE plane iova is
 * structurally invalid BEFORE it is programmed into the SVPP2 RDMA, so a
 * stale/zero dmabuf-import plane iova becomes a clean per-job -EIO instead of a
 * console-flooding, CPU-stalling mtk-iommu fault (region-2 master =
 * M4U_PORT_L5_SVPP2_MDP_RDMA). The MTK MM IOMMU is bank/region-based: a master
 * issues a 32-bit iova (0..4G) that HW prepends with its 4G region base, so a
 * valid plane iova is in (0, 4G-8M) -- iova 0 faults at the region base
 * (0x2_0000_0000, the observed layer-0 signature) and anything in the top 8 MB
 * per-region gap is unmappable. Belt-and-suspenders ALONGSIDE the
 * mdp_buffer_planes_contig reuse fix below (which removes the stale-chroma-iova
 * SOURCE); it also logs the offending plane iovas so an induced fault is
 * captured (Fault B's stale value never trips the range check, only the fix
 * prevents it). Default on; instrumentation-logs every frame under mdp_kd_debug.
 */
static bool mdp_iova_guard = true;
module_param(mdp_iova_guard, bool, 0644);
MODULE_PARM_DESC(mdp_iova_guard,
	"reject + log a job whose source plane iova is 0 or in the IOMMU top guard (default 1)");

/* 8 MB per-region top gap (drivers/iommu/mtk_iommu.c MTK_IOMMU_IOVA_SZ_4G). */
#define MDP_IOVA_TOP_GUARD	((u32)(SZ_4G - SZ_8M))	/* 0xFF800000 */

static inline bool mdp_iova_valid(u32 iova)
{
	return iova != 0 && iova < MDP_IOVA_TOP_GUARD;
}

/*
 * The packet-reuse fast-path (mdp_cmdq_reuse) delta-patches EVERY recorded
 * plane-iova write by a SINGLE plane-0 base delta. That is only correct when the
 * buffer's planes are contiguous in the IOMMU (iova[k] == iova[k-1] +
 * plane_fmt[k-1].size) -- a single-fd / MMAP-contiguous buffer where the whole
 * allocation moves as one. A MULTI-fd dmabuf-import whose chroma plane is an
 * INDEPENDENT allocation moves plane 1 by a DIFFERENT delta, so the reused
 * packet keeps a STALE chroma base -> once that chroma buffer is
 * recycled/unmapped the SVPP2 RDMA faults on the stale in-region iova (the
 * observed layer=1 signature). Only cache/reuse when both src and dst are
 * plane-contiguous; a non-contiguous buffer takes the full rebuild (fresh
 * per-plane iovas via mdp_prepare_buffer), which is always correct. Mirrors the
 * existing block-mode (detile) reuse exclusion; single-plane / single-fd reuse
 * (the board-validated case) is unaffected.
 */
static bool mdp_buffer_planes_contig(const struct img_image_buffer *b)
{
	u32 n = MDP_COLOR_GET_PLANE_COUNT(b->format.colorformat);
	u32 k;

	for (k = 1; k < n && k < IMG_MAX_PLANES; k++)
		if (b->iova[k] != b->iova[k - 1] + b->format.plane_fmt[k - 1].size)
			return false;
	return true;
}

static void mdp_m2m_device_run(void *priv)
{
	struct mdp_m2m_ctx *ctx = priv;
	struct mdp_frame *frame;
	struct vb2_v4l2_buffer *src_vb, *dst_vb;
	struct img_ipi_frameparam param = {};
	struct img_ipi_frameparam param_snap;
	struct mdp_cmdq_param task = {};
	enum vb2_buffer_state vb_state = VB2_BUF_STATE_ERROR;
	int ret;
	u32 nchunks;
	bool is_444_hybrid = false;
	bool can_cache = false;

	if (mdp_m2m_ctx_is_state_set(ctx, MDP_M2M_CTX_ERROR)) {
		dev_err(&ctx->mdp_dev->pdev->dev,
			"mdp_m2m_ctx is in error state\n");
		goto worker_end;
	}

	param.frame_no = ctx->curr_param.frame_no;
	param.type = ctx->curr_param.type;
	param.num_inputs = 1;
	param.num_outputs = 1;

	frame = ctx_get_frame(ctx, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
	src_vb = v4l2_m2m_next_src_buf(ctx->m2m_ctx);
	mdp_set_src_config(&param.inputs[0], frame, &src_vb->vb2_buf);

	/*
	 * Pin the imported source mapping for the whole async job (SVPP2 MDP_RDMA
	 * use-after-unmap iommu-fault fix): hand the source plane dma_bufs to the
	 * submit path, which takes a get_dma_buf() ref held until the GCE completion
	 * callback, so a concurrent context (decoder/pool churn under load) cannot
	 * free/unmap the imported buffer while the in-flight RDMA still reads the
	 * baked iova. Only MEMORY_DMABUF sources (the dmabuf-import detile that
	 * faults) have a dma_buf to pin; MMAP sources are vb2-allocated with a
	 * persistent mapping that never faults, so they pin nothing (src_dbuf_n=0).
	 * src_vb is held by v4l2-m2m until this job's completion, so its plane
	 * dma_bufs are valid to reference here.
	 */
	if (src_vb->vb2_buf.memory == VB2_MEMORY_DMABUF) {
		u32 p;

		for (p = 0; p < src_vb->vb2_buf.num_planes && p < IMG_MAX_PLANES; p++) {
			task.src_dbuf[p] = src_vb->vb2_buf.planes[p].dbuf;
			task.src_dbuf_n++;
		}
	}

	frame = ctx_get_frame(ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	dst_vb = v4l2_m2m_next_dst_buf(ctx->m2m_ctx);
	mdp_set_dst_config(&param.outputs[0], frame, &dst_vb->vb2_buf);

	/*
	 * Guard: never program the RDMA with a structurally-invalid source plane
	 * iova (0 / IOMMU top-guard) -- that is what floods mtk-iommu with SVPP2
	 * read faults. Reject the job cleanly and log the plane iovas so an induced
	 * fault is captured (also logs every frame under mdp_kd_debug to catch the
	 * stale-but-in-range chroma value). Covers both the fresh-compute and the
	 * reuse fast-path (evaluated on the fresh param before either branch).
	 */
	if (mdp_iova_guard) {
		struct img_image_buffer *ib = &param.inputs[0].buffer;
		struct img_image_buffer *ob = &param.outputs[0].buffer;
		u32 innp = MDP_COLOR_GET_PLANE_COUNT(ib->format.colorformat);
		bool bad = false;
		u32 pl;

		for (pl = 0; pl < innp && pl < IMG_MAX_PLANES; pl++)
			if (!mdp_iova_valid(ib->iova[pl]))
				bad = true;

		if (bad || mdp_kd_debug_on())
			dev_err_ratelimited(&ctx->mdp_dev->pdev->dev,
				"MDP-IOVA%s frame=%u type=%u src %ux%u cf=%#x iova[%#x,%#x,%#x] dst cf=%#x iova[%#x,%#x,%#x]\n",
				bad ? "-BAD" : "", param.frame_no, param.type,
				ib->format.width, ib->format.height,
				ib->format.colorformat,
				ib->iova[0], ib->iova[1], ib->iova[2],
				ob->format.colorformat,
				ob->iova[0], ob->iova[1], ob->iova[2]);
		if (bad)
			goto worker_end;
	}

	/* Job-batching (backlog #1) queue-depth probe: how many src/dst pairs are
	 * ALREADY QBUF'd-and-ready at this device_run entry -- i.e. the batch factor
	 * a driver-side coalescer could fold into one GCE packet. min(src,dst)>1 is
	 * the precondition for any batching win; a 1-deep client leaves it at 1
	 * (batching never triggers). Gated on mdp_kd_debug -- measurement only, no
	 * behaviour change. */
	if (mdp_kd_debug_on()) {
		unsigned int rs = v4l2_m2m_num_src_bufs_ready(ctx->m2m_ctx);
		unsigned int rd = v4l2_m2m_num_dst_bufs_ready(ctx->m2m_ctx);

		dev_info(&ctx->mdp_dev->pdev->dev,
			 "MDPKD-BATCH ready src=%u dst=%u pairs=%u\n",
			 rs, rd, min(rs, rd));
	}

	/*
	 * 4:4:4 hybrid: the dst is I444/YV24, which the WROT cannot write. Reconfigure
	 * the HW output to 4:2:2 (I422/YV16 -- proven) writing compact half-width chroma
	 * into the SAME 4:4:4 plane buffers (iova[] already set above, Y unchanged); the
	 * CPU horizontal-upsamples the chroma to 4:4:4 after the HW job completes.
	 */
	if (frame->mdp_fmt &&
	    (frame->mdp_fmt->mdp_color == MDP_COLOR_I444 ||
	     frame->mdp_fmt->mdp_color == MDP_COLOR_YV24)) {
		struct img_image_buffer *b = &param.outputs[0].buffer;
		u32 ow = b->format.width;
		u32 oh = b->format.height;

		if (!mdp_kd_444hybrid) {
			dev_err(&ctx->mdp_dev->pdev->dev,
				"4:4:4 output requires mdp_kd_444hybrid=1\n");
			goto worker_end;
		}
		b->format.colorformat =
			(frame->mdp_fmt->mdp_color == MDP_COLOR_YV24) ?
				MDP_COLOR_YV16 : MDP_COLOR_I422;
		b->format.plane_fmt[1].stride = ow / 2;
		b->format.plane_fmt[1].size   = (ow / 2) * oh;
		b->format.plane_fmt[2].stride = ow / 2;
		b->format.plane_fmt[2].size   = (ow / 2) * oh;
		is_444_hybrid = true;
	}

	if (mdp_check_pp_enable(ctx->mdp_dev, frame))
		param.type = MDP_STREAM_TYPE_DUAL_BITBLT;

	/* Packet-reuse fast-path: same transform as the cached packet -> patch only
	 * the src/dst plane iovas and re-submit, skipping mdp_vpu_process + the whole
	 * GCE-packet rebuild. Only the simple single-pp path is ever cached (see the
	 * can_cache gate below), so a hit is non-4:4:4-hybrid / non-detile / non-DUAL;
	 * the block-mode/compress guard here is defensive (a block-mode source never
	 * matches a cached linear packet -- colorformat is in the mdp_reuse_match key
	 * -- and is never cached, so this makes that invariant explicit). */
	if (mdp_kd_pktreuse && ctx->reuse_cmd && !is_444_hybrid && !ctx->ccm.en &&
	    ctx->rsz_table < 0 &&
	    param.type != MDP_STREAM_TYPE_DUAL_BITBLT &&
	    !MDP_COLOR_IS_BLOCK_MODE(param.inputs[0].buffer.format.colorformat) &&
	    !MDP_COLOR_IS_COMPRESS(param.inputs[0].buffer.format.colorformat) &&
	    mdp_buffer_planes_contig(&param.inputs[0].buffer) &&
	    mdp_buffer_planes_contig(&param.outputs[0].buffer) &&
	    mdp_reuse_match(&ctx->reuse_param, &param)) {
		if (refcount_read(&ctx->mdp_dev->job_count)) {
			ret = wait_event_timeout(ctx->mdp_dev->callback_wq,
						 !refcount_read(&ctx->mdp_dev->job_count),
						 2 * HZ);
			if (ret == 0) {
				dev_err(&ctx->mdp_dev->pdev->dev,
					"reuse: previous job not done\n");
				goto worker_end;
			}
		}
		ret = mdp_cmdq_reuse(ctx->mdp_dev, ctx->reuse_cmd, &param);
		if (!ret)
			return;
		/* reuse submit failed -> drop the cache, fall through to full rebuild */
		dev_warn(&ctx->mdp_dev->pdev->dev,
			 "reuse submit failed %d, rebuilding\n", ret);
		mdp_cmdq_reuse_free(ctx->reuse_cmd);
		ctx->reuse_cmd = NULL;
	}

	/* Snapshot the pre-compute (geometry) frameparam for the reuse-match key. */
	param_snap = param;

	ret = mdp_vpu_process(&ctx->mdp_dev->vpu, &param);
	if (ret) {
		dev_err(&ctx->mdp_dev->pdev->dev,
			"VPU MDP process failed: %d\n", ret);
		goto worker_end;
	}

	mdp_dbg_dump_config(ctx->mdp_dev,
			    (struct img_config *)ctx->mdp_dev->vpu.config); /* MDPDBG oracle */

	/* GREY (Y8) SOURCE pre-expand: mdp_vpu_process reconfigured the source to plain
	 * NV12 @ scratch_b (the config already reads it); here the CPU builds that
	 * scratch NV12 from the real GREY input before the GCE submit -- copy the Y plane
	 * verbatim, fill a neutral (0x80) interleaved UV plane. The source stride was
	 * preserved, so Y is a straight istride*h copy and the UV plane follows it. */
	if (ctx->mdp_dev->vpu.grey_src) {
		struct mdp_vpu_dev *vpu = &ctx->mdp_dev->vpu;
		u32 h = param.inputs[0].buffer.format.height;
		u32 istride = param.inputs[0].buffer.format.plane_fmt[0].stride;
		size_t ysz = (size_t)istride * h;
		u8 *sva = vb2_plane_vaddr(&src_vb->vb2_buf, 0);
		u8 *nv = vpu->scratch_b;

		if (!sva || !nv) {
			dev_warn(&ctx->mdp_dev->pdev->dev,
				 "GREY src-expand: no vaddr (src=%p scratch=%p)\n",
				 sva, nv);
			goto worker_end;
		}
		memcpy(nv, sva, ysz);		/* Y plane, verbatim */
		memset(nv + ysz, 0x80, ysz / 2);	/* neutral interleaved UV */
		wmb();		/* drain the WC scratch before the GCE reads it */
	}

	/* Hybrid NV12 rotate: HW does the Y plane (GREY), the CPU does the chroma
	 * here (the WROT 3-plane I420 write is broken). Disjoint planes, so this runs
	 * before the async HW Y submit without racing it. */
	if (ctx->mdp_dev->vpu.hybrid_chroma) {
		struct mdp_vpu_dev *vpu = &ctx->mdp_dev->vpu;
		u32 w = param.inputs[0].buffer.format.width;
		u32 h = param.inputs[0].buffer.format.height;
		u32 oh = param.outputs[0].buffer.format.height;	/* = w for rot90/270 */
		u32 istride = param.inputs[0].buffer.format.plane_fmt[0].stride;
		u32 ostride = param.outputs[0].buffer.format.plane_fmt[0].stride;
		u8 *sva = vb2_plane_vaddr(&src_vb->vb2_buf, 0);
		u8 *dva = vb2_plane_vaddr(&dst_vb->vb2_buf, 0);

		if (!sva || !dva)
			dev_warn(&ctx->mdp_dev->pdev->dev,
				 "hybrid chroma: no CPU vaddr (src=%p dst=%p)\n", sva, dva);
		else if (vpu->hybrid_swap)	/* rot90/270: dim-swap the chroma */
			mdp_hybrid_chroma_swap(dva + ostride * oh, sva + istride * h,
					       w / 2, h / 2, istride, ostride,
					       vpu->hybrid_ccw);
		else
			mdp_hybrid_chroma_rotate(dva + ostride * oh, sva + istride * h,
						 w / 2, h / 2, istride, ostride,
						 vpu->hybrid_rev_rows,
						 vpu->hybrid_rev_cols);
	}

	/* Multi-job detile: mdp_vpu_process published detile_nchunks (1 for every
	 * path except an mt8183 kernel-direct detile whose 64x32 tile grid exceeds
	 * IMG_MAX_SUBFRAMES_8183). For nchunks>1 the whole frame is ONE async batch
	 * -- mdp_cmdq_send_detile builds nchunks packets and submits them all to the
	 * single GCE thread (chained in submit order, so the inter-chunk WROT-EOF ->
	 * RDMA barrier holds), with ONE pm ref + ONE job_count, buf_done on the last
	 * packet. No per-chunk wait: the worker returns after submit (0 CPU while the
	 * MDP HW runs). All chunks write the same in/out dma-buf (per-tile offsets). */
	nchunks = ctx->mdp_dev->vpu.detile_nchunks;

	task.config = ctx->mdp_dev->vpu.config;	/* chunk 0 from mdp_vpu_process */
	task.param = &param;
	task.composes[0] = &frame->compose;
	task.cmdq_cb = NULL;
	task.cb_data = NULL;
	/* send_detile gives it to the last packet only. For the 4:4:4 hybrid AND the
	 * I420/YV12-dst hybrid we suppress the auto buf_done (mdp_ctx == NULL) and
	 * finalize manually below, AFTER the CPU chroma pass. */
	task.mdp_ctx = (is_444_hybrid || ctx->mdp_dev->vpu.i420_deint) ? NULL : ctx;
	/* Per-context WROT programmable CCM (V4L2_CID_MTK_MDP_CCM). NULL when the
	 * control is unset => config_wrot_frame keeps the built-in CSC. */
	task.ccm = ctx->ccm.en ? &ctx->ccm : NULL;
	/* Per-context RSZ coeff-table select (V4L2_CID_MTK_MDP_RSZ_COEFF_TABLE).
	 * -1 (the ctx default) => config_rsz_frame keeps the driver-default table.
	 * NB: the mdp_cmdq_param default is 0 (= table 0), so this MUST be set by
	 * every builder that runs an active RSZ -- m2m is the only such builder. */
	task.rsz_table = ctx->rsz_table;
	/* Gate the RSZ coeff-table override KERNEL-DIRECT-only: mdp_vpu_process (called
	 * above) set vpu.kd_used=false iff it fell back to the SCP. On the SCP path the
	 * override must NOT run (it would patch the SCP-computed config on the fragile
	 * SCP GCE path -- WARN/wedge risk), so config_rsz_frame keys off cmd->rsz_kd. */
	task.rsz_kd = ctx->mdp_dev->vpu.kd_used;

	/* Per-FRAME wait: previous frame's job must finish before we reuse
	 * vpu->config + job_count (the m2m core already serializes frames, so this
	 * is normally satisfied immediately -- it is a defensive backstop). */
	if (refcount_read(&ctx->mdp_dev->job_count)) {
		ret = wait_event_timeout(ctx->mdp_dev->callback_wq,
					 !refcount_read(&ctx->mdp_dev->job_count),
					 2 * HZ);
		if (ret == 0) {
			dev_err(&ctx->mdp_dev->pdev->dev, "%d jobs not yet done\n",
				refcount_read(&ctx->mdp_dev->job_count));
			goto worker_end;
		}
	}
	/* Debug: previous frame's job is now done -- dump its de-interleaved scratch
	 * to isolate the 5-pass de-interleave (write) from re-interleave (read). */
	mdp_vpu_scratch_dump(&ctx->mdp_dev->vpu);

	/* Cache for reuse only the simple single-pp path (non-4:4:4-hybrid, non-
	 * detile, non-DUAL, no hybrid chroma) -- the common blit / scale / crop /
	 * CSC / fmtconv transform a streaming session repeats with fixed geometry.
	 * mdp_cmdq_send stashes the built cmd into ctx->reuse_cmd via reuse_slot and
	 * marks it so the completion callback keeps it alive.
	 *
	 * Detile MUST be excluded: an MM21/MT21 (block-mode/compressed) source is a
	 * tiled detile whose reuse iova-patch is NOT bit-exact (the WROT/RDMA tile
	 * addressing is not the simple SRC_BASE/END + VIDO_BASE patch reuse scans for).
	 * On mt8195 a single-pipe detile is nchunks==1, so nchunks>1 does NOT exclude
	 * it (only the mt8183 >14-tile detile is nchunks>1) -- the earlier "detile ==
	 * nchunks>1" assumption was wrong and defaulting reuse on corrupted frames 2..N
	 * of the codec detile path (board-caught: kd-defaulton 1280x720-single exact=0).
	 * Gate explicitly on the block-mode/compressed source instead. */
	can_cache = mdp_kd_pktreuse && !is_444_hybrid && nchunks == 1 &&
		    !ctx->mdp_dev->vpu.i420_deint && !ctx->ccm.en &&
		    ctx->rsz_table < 0 &&
		    !ctx->mdp_dev->vpu.grey_src &&
		    param.type != MDP_STREAM_TYPE_DUAL_BITBLT &&
		    !ctx->mdp_dev->vpu.hybrid_chroma &&
		    !MDP_COLOR_IS_BLOCK_MODE(param.inputs[0].buffer.format.colorformat) &&
		    !MDP_COLOR_IS_COMPRESS(param.inputs[0].buffer.format.colorformat) &&
		    mdp_buffer_planes_contig(&param.inputs[0].buffer) &&
		    mdp_buffer_planes_contig(&param.outputs[0].buffer);
	if (can_cache) {
		mdp_ctx_reuse_invalidate(ctx);	/* drop any stale geometry cmd */
		task.reuse_slot = &ctx->reuse_cmd;
	}

	if (nchunks > 1)
		ret = mdp_cmdq_send_detile(ctx->mdp_dev, &task, nchunks,
					   mdp_vpu_compute_8183_chunk);
	else
		ret = mdp_cmdq_send(ctx->mdp_dev, &task);
	if (ret) {
		dev_err(&ctx->mdp_dev->pdev->dev, "CMDQ send failed: %d\n", ret);
		goto worker_end;
	}

	if (can_cache && ctx->reuse_cmd)
		ctx->reuse_param = param_snap;

	if (is_444_hybrid) {
		/* The HW wrote 4:2:2 into the 4:4:4 plane buffers. Wait for that job,
		 * then CPU horizontal-upsample each chroma plane (compact ow/2-wide ->
		 * full ow-wide, in place) to complete the 4:4:4 frame, then finalize. */
		struct mdp_vpu_dev *vpu = &ctx->mdp_dev->vpu;
		u32 ow = param.outputs[0].buffer.format.width;
		u32 oh = param.outputs[0].buffer.format.height;
		u8 *cb = vb2_plane_vaddr(&dst_vb->vb2_buf, 1);
		u8 *cr = vb2_plane_vaddr(&dst_vb->vb2_buf, 2);

		if (refcount_read(&ctx->mdp_dev->job_count)) {
			ret = wait_event_timeout(ctx->mdp_dev->callback_wq,
						 !refcount_read(&ctx->mdp_dev->job_count),
						 2 * HZ);
			if (ret == 0) {
				dev_err(&ctx->mdp_dev->pdev->dev,
					"4:4:4 hybrid: HW job timed out\n");
				goto worker_end;
			}
		}
		if (cb && cr) {
			/*
			 * KI-014: the compute routes the reconfigured 3-plane I422/YV16
			 * dst through the ym16dst arm (mdp_kd_ym16dst8195): the HW wrote
			 * 2-plane NV16 (interleaved CbCr, FULL-height) to scratch_a and set
			 * ym16_deint, expecting the CPU de-interleave (the i420_deint branch
			 * below) to split scratch -> the compact 4:2:2 chroma planes 1/2.
			 * is_444_hybrid takes THIS branch instead, so run that de-interleave
			 * HERE first (compact 4:2:2 into plane 1/2), then horizontal-upsample
			 * each to 4:4:4 in place. Without it plane 1/2 stay zero -> the Y
			 * plane is bit-exact but U/V are all-zero. YV24 (i420_deint_yv12)
			 * selects the U/V plane order exactly as the i420_deint branch does.
			 */
			if (vpu->ym16_deint) {
				u32 ostride =
					param.outputs[0].buffer.format.plane_fmt[0].stride;
				bool yv = vpu->i420_deint_yv12;
				u8 *cb_dst = yv ? cr : cb;	/* Cb -> U plane */
				u8 *cr_dst = yv ? cb : cr;	/* Cr -> V plane */

				mdp_hybrid_nv16_deinterleave_mp(vpu->scratch_a, cb_dst,
								cr_dst, ow, oh,
								ostride, ow / 2);
			}
			mdp_hybrid_444_hupsample(cb, ow, oh, ow / 2, ow);
			mdp_hybrid_444_hupsample(cr, ow, oh, ow / 2, ow);
			mdp_m2m_process_done(ctx, VB2_BUF_STATE_DONE);
		} else {
			dev_warn(&ctx->mdp_dev->pdev->dev,
				 "4:4:4 hybrid: no chroma vaddr (cb=%p cr=%p)\n", cb, cr);
			goto worker_end;
		}
	} else if (ctx->mdp_dev->vpu.i420_deint) {
		/* The HW wrote 2-plane NV12 (Y + interleaved CbCr). Wait for that job, then
		 * CPU de-interleave the CbCr plane into the separate I420/YV12 U/V planes,
		 * then finalize. Two layouts:
		 *  - single-buffer (i420_deint_mp==false): Y+UV are in ONE dma-buf; the CbCr
		 *    plane at ystride*h is split in place into the contiguous U/V region.
		 *  - multi-planar (i420_deint_mp==true, YM12/YM21): Y went to the real Y-plane
		 *    dma-buf, the interleaved CbCr went to scratch_a; split scratch_a -> the
		 *    two SEPARATE chroma-plane dma-bufs (vb2 plane 1/2). */
		struct mdp_vpu_dev *vpu = &ctx->mdp_dev->vpu;
		u32 ow = param.outputs[0].buffer.format.width;
		u32 oh = param.outputs[0].buffer.format.height;
		u32 ostride = param.outputs[0].buffer.format.plane_fmt[0].stride;
		bool yv12 = vpu->i420_deint_yv12;
		u8 *base = vb2_plane_vaddr(&dst_vb->vb2_buf, 0);

		if (refcount_read(&ctx->mdp_dev->job_count)) {
			ret = wait_event_timeout(ctx->mdp_dev->callback_wq,
						 !refcount_read(&ctx->mdp_dev->job_count),
						 2 * HZ);
			if (ret == 0) {
				dev_err(&ctx->mdp_dev->pdev->dev,
					"I420/YV12-dst hybrid: HW job timed out\n");
				goto worker_end;
			}
		}
		if (vpu->i420_deint_mp) {
			/* multi-planar: scratch_a holds the interleaved UV (uvstride =
			 * NV12 UV pitch = ostride); de-interleave to plane 1/2. */
			const u8 *uv = vpu->scratch_a;
			u8 *p1 = vb2_plane_vaddr(&dst_vb->vb2_buf, 1);	/* I420 U / YV12 V */
			u8 *p2 = vb2_plane_vaddr(&dst_vb->vb2_buf, 2);	/* I420 V / YV12 U */
			u8 *cb_dst = yv12 ? p2 : p1;			/* Cb -> U plane */
			u8 *cr_dst = yv12 ? p1 : p2;			/* Cr -> V plane */

			if (!uv || !p1 || !p2) {
				dev_warn(&ctx->mdp_dev->pdev->dev,
					 "multi-planar I420/YV12-dst hybrid: no vaddr (uv=%p p1=%p p2=%p)\n",
					 uv, p1, p2);
				goto worker_end;
			}
			if (vpu->ym16_deint)	/* 3-plane 4:2:2: FULL-height chroma */
				mdp_hybrid_nv16_deinterleave_mp(uv, cb_dst, cr_dst,
								ow, oh, ostride, ostride / 2);
			else
				mdp_hybrid_i420_deinterleave_mp(uv, cb_dst, cr_dst,
								ow, oh, ostride, ostride / 2);
		} else {
			if (!base) {
				dev_warn(&ctx->mdp_dev->pdev->dev,
					 "I420/YV12-dst hybrid: no CPU vaddr\n");
				goto worker_end;
			}
			if (mdp_hybrid_i420_deinterleave(base, ow, oh, ostride, yv12)) {
				dev_warn(&ctx->mdp_dev->pdev->dev,
					 "I420/YV12-dst hybrid: de-interleave alloc failed\n");
				goto worker_end;
			}
		}
		mdp_m2m_process_done(ctx, VB2_BUF_STATE_DONE);
	}

	return;

worker_end:
	mdp_m2m_process_done(ctx, vb_state);
}

static int mdp_m2m_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct mdp_m2m_ctx *ctx = vb2_get_drv_priv(q);
	struct mdp_frame *capture;
	struct vb2_queue *vq;
	int ret;
	bool out_streaming, cap_streaming;

	if (V4L2_TYPE_IS_OUTPUT(q->type))
		ctx->frame_count[MDP_M2M_SRC] = 0;

	if (V4L2_TYPE_IS_CAPTURE(q->type))
		ctx->frame_count[MDP_M2M_DST] = 0;

	capture = ctx_get_frame(ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	vq = v4l2_m2m_get_src_vq(ctx->m2m_ctx);
	out_streaming = vb2_is_streaming(vq);
	vq = v4l2_m2m_get_dst_vq(ctx->m2m_ctx);
	cap_streaming = vb2_is_streaming(vq);

	/* Check to see if scaling ratio is within supported range */
	if ((V4L2_TYPE_IS_OUTPUT(q->type) && cap_streaming) ||
	    (V4L2_TYPE_IS_CAPTURE(q->type) && out_streaming)) {
		ret = mdp_check_scaling_ratio(&capture->crop.c,
					      &capture->compose,
					      capture->rotation,
					      ctx->curr_param.limit);
		if (ret) {
			dev_err(&ctx->mdp_dev->pdev->dev,
				"Out of scaling range\n");
			return ret;
		}
	}

	if (!mdp_m2m_ctx_is_state_set(ctx, MDP_VPU_INIT)) {
		ret = mdp_vpu_get_locked(ctx->mdp_dev);
		if (ret) {
			dev_err(&ctx->mdp_dev->pdev->dev,
				"VPU init failed %d\n", ret);
			return -EINVAL;
		}
		mdp_m2m_ctx_set_state(ctx, MDP_VPU_INIT);
	}

	return 0;
}

static struct vb2_v4l2_buffer *mdp_m2m_buf_remove(struct mdp_m2m_ctx *ctx,
						  unsigned int type)
{
	if (V4L2_TYPE_IS_OUTPUT(type))
		return (struct vb2_v4l2_buffer *)
			v4l2_m2m_src_buf_remove(ctx->m2m_ctx);
	else
		return (struct vb2_v4l2_buffer *)
			v4l2_m2m_dst_buf_remove(ctx->m2m_ctx);
}

static void mdp_m2m_stop_streaming(struct vb2_queue *q)
{
	struct mdp_m2m_ctx *ctx = vb2_get_drv_priv(q);
	struct vb2_v4l2_buffer *vb;

	mdp_ctx_reuse_invalidate(ctx);	/* drop the cached reuse packet */

	vb = mdp_m2m_buf_remove(ctx, q->type);
	while (vb) {
		v4l2_m2m_buf_done(vb, VB2_BUF_STATE_ERROR);
		vb = mdp_m2m_buf_remove(ctx, q->type);
	}
}

static int mdp_m2m_queue_setup(struct vb2_queue *q,
			       unsigned int *num_buffers,
			       unsigned int *num_planes, unsigned int sizes[],
			       struct device *alloc_devs[])
{
	struct mdp_m2m_ctx *ctx = vb2_get_drv_priv(q);
	struct v4l2_pix_format_mplane *pix_mp;
	u32 i;

	pix_mp = &ctx_get_frame(ctx, q->type)->format.fmt.pix_mp;

	/* from VIDIOC_CREATE_BUFS */
	if (*num_planes) {
		if (*num_planes != pix_mp->num_planes)
			return -EINVAL;
		for (i = 0; i < pix_mp->num_planes; ++i)
			if (sizes[i] < pix_mp->plane_fmt[i].sizeimage)
				return -EINVAL;
	} else {/* from VIDIOC_REQBUFS */
		*num_planes = pix_mp->num_planes;
		for (i = 0; i < pix_mp->num_planes; ++i)
			sizes[i] = pix_mp->plane_fmt[i].sizeimage;
	}

	return 0;
}

static int mdp_m2m_buf_prepare(struct vb2_buffer *vb)
{
	struct mdp_m2m_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct v4l2_pix_format_mplane *pix_mp;
	struct vb2_v4l2_buffer *v4l2_buf = to_vb2_v4l2_buffer(vb);
	u32 i;

	v4l2_buf->field = V4L2_FIELD_NONE;

	if (V4L2_TYPE_IS_CAPTURE(vb->type)) {
		pix_mp = &ctx_get_frame(ctx, vb->type)->format.fmt.pix_mp;
		for (i = 0; i < pix_mp->num_planes; ++i) {
			vb2_set_plane_payload(vb, i,
					      pix_mp->plane_fmt[i].sizeimage);
		}
	}
	return 0;
}

static int mdp_m2m_buf_out_validate(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *v4l2_buf = to_vb2_v4l2_buffer(vb);

	v4l2_buf->field = V4L2_FIELD_NONE;

	return 0;
}

static void mdp_m2m_buf_queue(struct vb2_buffer *vb)
{
	struct mdp_m2m_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *v4l2_buf = to_vb2_v4l2_buffer(vb);

	v4l2_buf->field = V4L2_FIELD_NONE;

	v4l2_m2m_buf_queue(ctx->m2m_ctx, to_vb2_v4l2_buffer(vb));
}

static const struct vb2_ops mdp_m2m_qops = {
	.queue_setup	= mdp_m2m_queue_setup,
	.buf_prepare	= mdp_m2m_buf_prepare,
	.start_streaming = mdp_m2m_start_streaming,
	.stop_streaming	= mdp_m2m_stop_streaming,
	.buf_queue	= mdp_m2m_buf_queue,
	.buf_out_validate = mdp_m2m_buf_out_validate,
};

static int mdp_m2m_querycap(struct file *file, void *fh,
			    struct v4l2_capability *cap)
{
	strscpy(cap->driver, MDP_MODULE_NAME, sizeof(cap->driver));
	strscpy(cap->card, MDP_DEVICE_NAME, sizeof(cap->card));

	return 0;
}

static int mdp_m2m_enum_fmt_mplane(struct file *file, void *fh,
				   struct v4l2_fmtdesc *f)
{
	struct mdp_m2m_ctx *ctx = file_to_ctx(file);

	return mdp_enum_fmt_mplane(ctx->mdp_dev, f);
}

static int mdp_m2m_g_fmt_mplane(struct file *file, void *fh,
				struct v4l2_format *f)
{
	struct mdp_m2m_ctx *ctx = file_to_ctx(file);
	struct mdp_frame *frame;
	struct v4l2_pix_format_mplane *pix_mp;

	frame = ctx_get_frame(ctx, f->type);
	*f = frame->format;
	pix_mp = &f->fmt.pix_mp;
	pix_mp->colorspace = ctx->curr_param.colorspace;
	pix_mp->xfer_func = ctx->curr_param.xfer_func;
	pix_mp->ycbcr_enc = ctx->curr_param.ycbcr_enc;
	pix_mp->quantization = ctx->curr_param.quant;

	return 0;
}

/*
 * Set the crop (output queue) / compose (capture queue) rectangle on S_FMT.
 * The V4L2 default is the full frame. We only preserve a rectangle across a
 * later S_FMT when the client *explicitly* set it via S_SELECTION (@keep) and
 * it still fits -- e.g. v4l2-ctl re-issuing S_FMT during stream setup. When the
 * client never set one (@keep false), S_FMT must grow it to the full frame:
 * otherwise the initial 64x64 minimum-format default is kept forever (it "fits"
 * any larger format), which both violates the V4L2 full-frame default and, on
 * MT8195, fails mdp_frame_is_mm21_detile()'s full-frame-crop check -- forcing
 * the codec readback onto the SCP detile path (KI-009, wedges on the
 * cherry_scp_v2.0.26100 firmware) instead of the firmware-independent
 * kernel-direct detile.
 */
static void mdp_default_or_keep_rect(struct v4l2_rect *r, u32 w, u32 h, bool keep)
{
	if (keep && r->width && r->height &&
	    r->left + r->width <= w && r->top + r->height <= h)
		return;
	r->left = 0;
	r->top = 0;
	r->width = w;
	r->height = h;
}

static int mdp_m2m_s_fmt_mplane(struct file *file, void *fh,
				struct v4l2_format *f)
{
	struct mdp_m2m_ctx *ctx = file_to_ctx(file);
	struct mdp_frame *frame = ctx_get_frame(ctx, f->type);
	struct mdp_frame *capture;
	const struct mdp_format *fmt;
	struct vb2_queue *vq;

	fmt = mdp_try_fmt_mplane(ctx->mdp_dev, f, &ctx->curr_param, ctx->id);
	if (!fmt)
		return -EINVAL;

	vq = v4l2_m2m_get_vq(ctx->m2m_ctx, f->type);
	if (vb2_is_busy(vq))
		return -EBUSY;

	frame->format = *f;
	frame->mdp_fmt = fmt;
	frame->ycbcr_prof = mdp_map_ycbcr_prof_mplane(f, fmt->mdp_color);
	frame->usage = V4L2_TYPE_IS_OUTPUT(f->type) ?
		MDP_BUFFER_USAGE_HW_READ : MDP_BUFFER_USAGE_MDP;

	capture = ctx_get_frame(ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		mdp_default_or_keep_rect(&capture->crop.c,
					 f->fmt.pix_mp.width, f->fmt.pix_mp.height,
					 capture->crop_set);
		ctx->curr_param.colorspace = f->fmt.pix_mp.colorspace;
		ctx->curr_param.ycbcr_enc = f->fmt.pix_mp.ycbcr_enc;
		ctx->curr_param.quant = f->fmt.pix_mp.quantization;
		ctx->curr_param.xfer_func = f->fmt.pix_mp.xfer_func;
	} else {
		mdp_default_or_keep_rect(&capture->compose,
					 f->fmt.pix_mp.width, f->fmt.pix_mp.height,
					 capture->compose_set);
	}

	return 0;
}

static int mdp_m2m_try_fmt_mplane(struct file *file, void *fh,
				  struct v4l2_format *f)
{
	struct mdp_m2m_ctx *ctx = file_to_ctx(file);

	if (!mdp_try_fmt_mplane(ctx->mdp_dev, f, &ctx->curr_param, ctx->id))
		return -EINVAL;

	return 0;
}

static int mdp_m2m_g_selection(struct file *file, void *fh,
			       struct v4l2_selection *s)
{
	struct mdp_m2m_ctx *ctx = file_to_ctx(file);
	struct mdp_frame *frame;
	bool valid = false;

	if (s->type == V4L2_BUF_TYPE_VIDEO_OUTPUT)
		valid = mdp_target_is_crop(s->target);
	else if (s->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
		valid = mdp_target_is_compose(s->target);

	if (!valid)
		return -EINVAL;

	switch (s->target) {
	case V4L2_SEL_TGT_CROP:
		if (s->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
			return -EINVAL;
		frame = ctx_get_frame(ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
		s->r = frame->crop.c;
		return 0;
	case V4L2_SEL_TGT_COMPOSE:
		if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
			return -EINVAL;
		frame = ctx_get_frame(ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
		s->r = frame->compose;
		return 0;
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		if (s->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
			return -EINVAL;
		frame = ctx_get_frame(ctx, s->type);
		s->r.left = 0;
		s->r.top = 0;
		s->r.width = frame->format.fmt.pix_mp.width;
		s->r.height = frame->format.fmt.pix_mp.height;
		return 0;
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
		if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
			return -EINVAL;
		frame = ctx_get_frame(ctx, s->type);
		s->r.left = 0;
		s->r.top = 0;
		s->r.width = frame->format.fmt.pix_mp.width;
		s->r.height = frame->format.fmt.pix_mp.height;
		return 0;
	}
	return -EINVAL;
}

static int mdp_m2m_s_selection(struct file *file, void *fh,
			       struct v4l2_selection *s)
{
	struct mdp_m2m_ctx *ctx = file_to_ctx(file);
	struct mdp_frame *frame = ctx_get_frame(ctx, s->type);
	struct mdp_frame *capture;
	struct v4l2_rect r;
	struct device *dev = &ctx->mdp_dev->pdev->dev;
	bool valid = false;
	int ret;

	if (s->type == V4L2_BUF_TYPE_VIDEO_OUTPUT)
		valid = (s->target == V4L2_SEL_TGT_CROP);
	else if (s->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
		valid = (s->target == V4L2_SEL_TGT_COMPOSE);

	if (!valid) {
		dev_dbg(dev, "[%s:%d] invalid type:%u target:%u", __func__,
			ctx->id, s->type, s->target);
		return -EINVAL;
	}

	ret = mdp_try_crop(ctx, &r, s, frame);
	if (ret)
		return ret;
	capture = ctx_get_frame(ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);

	if (mdp_target_is_crop(s->target)) {
		capture->crop.c = r;
		capture->crop_set = 1;
	} else {
		capture->compose = r;
		capture->compose_set = 1;
	}

	s->r = r;

	return 0;
}

static const struct v4l2_ioctl_ops mdp_m2m_ioctl_ops = {
	.vidioc_querycap		= mdp_m2m_querycap,
	.vidioc_enum_fmt_vid_cap	= mdp_m2m_enum_fmt_mplane,
	.vidioc_enum_fmt_vid_out	= mdp_m2m_enum_fmt_mplane,
	.vidioc_g_fmt_vid_cap_mplane	= mdp_m2m_g_fmt_mplane,
	.vidioc_g_fmt_vid_out_mplane	= mdp_m2m_g_fmt_mplane,
	.vidioc_s_fmt_vid_cap_mplane	= mdp_m2m_s_fmt_mplane,
	.vidioc_s_fmt_vid_out_mplane	= mdp_m2m_s_fmt_mplane,
	.vidioc_try_fmt_vid_cap_mplane	= mdp_m2m_try_fmt_mplane,
	.vidioc_try_fmt_vid_out_mplane	= mdp_m2m_try_fmt_mplane,
	.vidioc_reqbufs			= v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf		= v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf			= v4l2_m2m_ioctl_qbuf,
	.vidioc_expbuf			= v4l2_m2m_ioctl_expbuf,
	.vidioc_dqbuf			= v4l2_m2m_ioctl_dqbuf,
	.vidioc_create_bufs		= v4l2_m2m_ioctl_create_bufs,
	.vidioc_streamon		= v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff		= v4l2_m2m_ioctl_streamoff,
	.vidioc_g_selection		= mdp_m2m_g_selection,
	.vidioc_s_selection		= mdp_m2m_s_selection,
	.vidioc_subscribe_event		= v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event	= v4l2_event_unsubscribe,
};

static int mdp_m2m_queue_init(void *priv,
			      struct vb2_queue *src_vq,
			      struct vb2_queue *dst_vq)
{
	struct mdp_m2m_ctx *ctx = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->ops = &mdp_m2m_qops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->drv_priv = ctx;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->dev = &ctx->mdp_dev->pdev->dev;
	src_vq->lock = &ctx->ctx_lock;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->ops = &mdp_m2m_qops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->drv_priv = ctx;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->dev = &ctx->mdp_dev->pdev->dev;
	dst_vq->lock = &ctx->ctx_lock;

	return vb2_queue_init(dst_vq);
}

static int mdp_m2m_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mdp_m2m_ctx *ctx = ctrl_to_ctx(ctrl);
	struct mdp_frame *capture;

	capture = ctx_get_frame(ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	switch (ctrl->id) {
	case V4L2_CID_HFLIP:
		capture->hflip = ctrl->val;
		break;
	case V4L2_CID_VFLIP:
		capture->vflip = ctrl->val;
		break;
	case V4L2_CID_ROTATE:
		capture->rotation = ctrl->val;
		break;
	case V4L2_CID_MTK_MDP_CCM: {
		/* Decode the s32[15] payload into the per-context CCM cfg. Layout:
		 * [0..8] 3x3 coeffs (Q12, masked 0x1fff at HW-write time in
		 * config_wrot_frame); [9..11] pre-add, [12..14] post-add -- each a
		 * signed 9-bit offset packed <<0/<<10/<<20 into VIDO_MAT_PRE/POSTADD.
		 * The control is "active" iff any element is non-zero (an all-zero
		 * matrix is never a wanted transform, so it doubles as the unset
		 * sentinel => built-in CSC). */
		const s32 *v = ctrl->p_new.p_s32;
		struct mdp_ccm_cfg cfg = {};
		bool any = false;
		int i;

		for (i = 0; i < MDP_CCM_NELEMS; i++)
			any |= (v[i] != 0);
		for (i = 0; i < 9; i++)
			cfg.coef[i] = v[i];
		cfg.preadd  = ((u32)(v[9]  & 0x1ff))        |
			      ((u32)(v[10] & 0x1ff) << 10)  |
			      ((u32)(v[11] & 0x1ff) << 20);
		cfg.postadd = ((u32)(v[12] & 0x1ff))        |
			      ((u32)(v[13] & 0x1ff) << 10)  |
			      ((u32)(v[14] & 0x1ff) << 20);
		cfg.en = any;
		ctx->ccm = cfg;
		break;
	}
	case V4L2_CID_MTK_MDP_RSZ_COEFF_TABLE:
		/* -1 => driver default (byte-identical); 0..27 selects a HW RSZ
		 * coeff table (0=sharpest..6=softest on a downscale). */
		ctx->rsz_table = ctrl->val;
		break;
	}

	return 0;
}

static const struct v4l2_ctrl_ops mdp_m2m_ctrl_ops = {
	.s_ctrl	= mdp_m2m_s_ctrl,
};

static int mdp_m2m_ctrls_create(struct mdp_m2m_ctx *ctx)
{
	v4l2_ctrl_handler_init(&ctx->ctrl_handler, MDP_MAX_CTRLS);
	ctx->ctrls.hflip = v4l2_ctrl_new_std(&ctx->ctrl_handler,
					     &mdp_m2m_ctrl_ops, V4L2_CID_HFLIP,
					     0, 1, 1, 0);
	ctx->ctrls.vflip = v4l2_ctrl_new_std(&ctx->ctrl_handler,
					     &mdp_m2m_ctrl_ops, V4L2_CID_VFLIP,
					     0, 1, 1, 0);
	ctx->ctrls.rotate = v4l2_ctrl_new_std(&ctx->ctrl_handler,
					      &mdp_m2m_ctrl_ops,
					      V4L2_CID_ROTATE, 0, 270, 90, 0);

	/* Private compound control: WROT programmable color-correction matrix.
	 * s32[MDP_CCM_NELEMS] array; default all-zero => built-in CSC (opt-in). */
	{
		static const struct v4l2_ctrl_config ccm_cfg = {
			.ops	= &mdp_m2m_ctrl_ops,
			.id	= V4L2_CID_MTK_MDP_CCM,
			.name	= "mtk_mdp_ccm",
			.type	= V4L2_CTRL_TYPE_INTEGER,
			.min	= -8192,
			.max	= 8191,
			.step	= 1,
			.def	= 0,
			.dims	= { MDP_CCM_NELEMS },
		};

		ctx->ctrls.ccm = v4l2_ctrl_new_custom(&ctx->ctrl_handler,
						      &ccm_cfg, NULL);
	}

	/* Private control: mt8195 RSZ coefficient-table select (scaler sharpness/
	 * anti-alias). -1 => driver default (byte-identical); 0..27 selects a HW
	 * coeff table (0=sharpest..6=softest on a downscale, board-characterized). */
	{
		static const struct v4l2_ctrl_config rsz_tbl_cfg = {
			.ops	= &mdp_m2m_ctrl_ops,
			.id	= V4L2_CID_MTK_MDP_RSZ_COEFF_TABLE,
			.name	= "mtk_mdp_rsz_coeff_table",
			.type	= V4L2_CTRL_TYPE_INTEGER,
			.min	= -1,
			.max	= 27,
			.step	= 1,
			.def	= -1,
		};

		ctx->ctrls.rsz_table = v4l2_ctrl_new_custom(&ctx->ctrl_handler,
							    &rsz_tbl_cfg, NULL);
	}

	if (ctx->ctrl_handler.error) {
		int err = ctx->ctrl_handler.error;

		v4l2_ctrl_handler_free(&ctx->ctrl_handler);
		dev_err(&ctx->mdp_dev->pdev->dev,
			"Failed to register controls\n");
		return err;
	}
	return 0;
}

static int mdp_m2m_open(struct file *file)
{
	struct video_device *vdev = video_devdata(file);
	struct mdp_dev *mdp = video_get_drvdata(vdev);
	struct mdp_m2m_ctx *ctx;
	struct device *dev = &mdp->pdev->dev;
	int ret;
	struct v4l2_format default_format = {};
	const struct mdp_limit *limit = mdp->mdp_data->def_limit;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	if (mutex_lock_interruptible(&mdp->m2m_lock)) {
		ret = -ERESTARTSYS;
		goto err_free_ctx;
	}

	ret = ida_alloc(&mdp->mdp_ida, GFP_KERNEL);
	if (ret < 0)
		goto err_unlock_mutex;
	ctx->id = ret;
	ctx->rsz_table = -1;	/* default: driver-default RSZ coeff table */

	ctx->mdp_dev = mdp;

	v4l2_fh_init(&ctx->fh, vdev);
	ret = mdp_m2m_ctrls_create(ctx);
	if (ret)
		goto err_exit_fh;

	/* Use separate control handler per file handle */
	ctx->fh.ctrl_handler = &ctx->ctrl_handler;
	v4l2_fh_add(&ctx->fh, file);

	mutex_init(&ctx->ctx_lock);
	ctx->m2m_ctx = v4l2_m2m_ctx_init(mdp->m2m_dev, ctx, mdp_m2m_queue_init);
	if (IS_ERR(ctx->m2m_ctx)) {
		dev_err(dev, "Failed to initialize m2m context\n");
		ret = PTR_ERR(ctx->m2m_ctx);
		goto err_release_handler;
	}
	ctx->fh.m2m_ctx = ctx->m2m_ctx;

	ctx->curr_param.ctx = ctx;
	ret = mdp_frameparam_init(mdp, &ctx->curr_param);
	if (ret) {
		dev_err(dev, "Failed to initialize mdp parameter\n");
		goto err_release_m2m_ctx;
	}

	mutex_unlock(&mdp->m2m_lock);

	/* Default format */
	default_format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	default_format.fmt.pix_mp.width = limit->out_limit.wmin;
	default_format.fmt.pix_mp.height = limit->out_limit.hmin;
	default_format.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_YUV420M;
	mdp_m2m_s_fmt_mplane(file, &ctx->fh, &default_format);
	default_format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	mdp_m2m_s_fmt_mplane(file, &ctx->fh, &default_format);

	dev_dbg(dev, "%s:[%d]", __func__, ctx->id);

	return 0;

err_release_m2m_ctx:
	v4l2_m2m_ctx_release(ctx->m2m_ctx);
err_release_handler:
	v4l2_ctrl_handler_free(&ctx->ctrl_handler);
	v4l2_fh_del(&ctx->fh, file);
err_exit_fh:
	v4l2_fh_exit(&ctx->fh);
	ida_free(&mdp->mdp_ida, ctx->id);
err_unlock_mutex:
	mutex_unlock(&mdp->m2m_lock);
err_free_ctx:
	kfree(ctx);

	return ret;
}

static int mdp_m2m_release(struct file *file)
{
	struct mdp_m2m_ctx *ctx = file_to_ctx(file);
	struct mdp_dev *mdp = video_drvdata(file);
	struct device *dev = &mdp->pdev->dev;

	mutex_lock(&mdp->m2m_lock);
	mdp_ctx_reuse_invalidate(ctx);	/* free any cached reuse packet */
	v4l2_m2m_ctx_release(ctx->m2m_ctx);
	if (mdp_m2m_ctx_is_state_set(ctx, MDP_VPU_INIT))
		mdp_vpu_put_locked(mdp);

	v4l2_ctrl_handler_free(&ctx->ctrl_handler);
	v4l2_fh_del(&ctx->fh, file);
	v4l2_fh_exit(&ctx->fh);
	ida_free(&mdp->mdp_ida, ctx->id);
	mutex_unlock(&mdp->m2m_lock);

	dev_dbg(dev, "%s:[%d]", __func__, ctx->id);
	kfree(ctx);

	return 0;
}

static const struct v4l2_file_operations mdp_m2m_fops = {
	.owner		= THIS_MODULE,
	.poll		= v4l2_m2m_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= v4l2_m2m_fop_mmap,
	.open		= mdp_m2m_open,
	.release	= mdp_m2m_release,
};

static const struct v4l2_m2m_ops mdp_m2m_ops = {
	.device_run	= mdp_m2m_device_run,
};

int mdp_m2m_device_register(struct mdp_dev *mdp)
{
	struct device *dev = &mdp->pdev->dev;
	int ret = 0;

	mdp->m2m_vdev = video_device_alloc();
	if (!mdp->m2m_vdev) {
		dev_err(dev, "Failed to allocate video device\n");
		ret = -ENOMEM;
		goto err_video_alloc;
	}
	mdp->m2m_vdev->device_caps = V4L2_CAP_VIDEO_M2M_MPLANE |
		V4L2_CAP_STREAMING;
	mdp->m2m_vdev->fops = &mdp_m2m_fops;
	mdp->m2m_vdev->ioctl_ops = &mdp_m2m_ioctl_ops;
	mdp->m2m_vdev->release = mdp_video_device_release;
	mdp->m2m_vdev->lock = &mdp->m2m_lock;
	mdp->m2m_vdev->vfl_dir = VFL_DIR_M2M;
	mdp->m2m_vdev->v4l2_dev = &mdp->v4l2_dev;
	snprintf(mdp->m2m_vdev->name, sizeof(mdp->m2m_vdev->name), "%s:m2m",
		 MDP_MODULE_NAME);
	video_set_drvdata(mdp->m2m_vdev, mdp);

	mdp->m2m_dev = v4l2_m2m_init(&mdp_m2m_ops);
	if (IS_ERR(mdp->m2m_dev)) {
		dev_err(dev, "Failed to initialize v4l2-m2m device\n");
		ret = PTR_ERR(mdp->m2m_dev);
		goto err_m2m_init;
	}

	ret = video_register_device(mdp->m2m_vdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		dev_err(dev, "Failed to register video device\n");
		goto err_video_register;
	}

	v4l2_info(&mdp->v4l2_dev, "Driver registered as /dev/video%d",
		  mdp->m2m_vdev->num);
	return 0;

err_video_register:
	v4l2_m2m_release(mdp->m2m_dev);
err_m2m_init:
	video_device_release(mdp->m2m_vdev);
err_video_alloc:

	return ret;
}

void mdp_m2m_device_unregister(struct mdp_dev *mdp)
{
	video_unregister_device(mdp->m2m_vdev);
}

void mdp_m2m_job_finish(struct mdp_m2m_ctx *ctx)
{
	enum vb2_buffer_state vb_state = VB2_BUF_STATE_DONE;

	mdp_m2m_process_done(ctx, vb_state);
}
