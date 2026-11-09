// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek MT8188 DPE (depth engine): stereo disparity (DVS) as a V4L2 mem2mem device.
 *
 * The DPE of the ISP 7 generation has two engines: DVS matches a rectified left/right pair
 * and writes a disparity map, DVP post-processes it. This driver runs DVS only.
 *
 * Programming follows MediaTek's public ISP 7s driver (camera_dpe.c: DPE_Config_DVS(),
 * CmdqDPEHW(), DPE_Reset()), DVS-only mode, issued as MMIO writes. Geometry and every
 * DVS_ME_* tuning word are left at the block's reset values, which the G5 measured to be a
 * working configuration: frame 384x448 with a 480-byte pitch, engine 352x428 from y = 10,
 * right eye as the main eye. A job takes about 3.1 ms. The block's interrupt (SPI 789)
 * fires at the start of a job (status bit 1) and about 2.5 ms before done (bit 2), not at
 * done itself: the driver sleeps until bit 2, then polls DVS_CTRL_STATUS0 bit 31. Waiting
 * this way left one non-disparity column (343) unwritten in an earlier, unwindowed build;
 * the windowed driver copies only the 320 disparity columns of each window, so that column
 * never reaches the CAPTURE buffer.
 *
 * Wider images run as windows, the way the vendor's tile path does: each window is one job
 * on the 384-wide reset geometry, 320 px further right in the input, writing its own
 * 384x428 region of a driver-owned buffer; its first 320 columns are disparity and are
 * copied into the CAPTURE buffer. Only the input pitch (DVS_DRAM_PITCH bits 0-19) changes.
 * Two windows stitch without a seam.
 *
 * Heights: each job sets the frame height (DVS_SRC_01), the engine height, frame - 20
 * (DVS_SRC_02), and the out_adj_dv height in DVS_SRC_46, whose reset 480 otherwise caps
 * the output rows. Even heights 256-1000 were measured (audits
 * 2026-11-08-mt8188-dpe-round{6,7}.md).
 *
 * Formats:
 *   OUTPUT  GREY W x 2H, W = 64 + 320 k (k = 1..6), H even in 256-1000 (default 448),
 *           bytesperline >= W, a multiple of 16: the left image in rows 0..H-1, the right
 *           image in rows H..2H-1, 8 bits a pixel.
 *   CAPTURE GREY 320 k x (H - 20): disparity in the right eye's coordinates, output column c
 *           and row r at right-image x = c + 32, y = r + 10, in 1/8 pixel, 0xff where the
 *           engine found no confident match.
 */
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#define DPE7_NAME		"mtk-dpe7"

#define DVS_CTRL00		0x000
#define DVS_CTRL00_MODE_MASK	0xdbf5fc00
#define DVS_CTRL00_DVS_ONLY	0xd1043400
#define DVS_CTRL00_FW_TRIG	BIT(29)
#define DVS_CTRL01		0x004
#define DVS_CTRL01_RESET	GENMASK(30, 28)
#define DVS_CTRL02		0x008
#define DVS_CTRL02_DEFAULT	0x70310001
#define DVS_IRQ_00		0x040
#define DVS_IRQ_00_EN_MASK	0xffff	/* low enables, bits 8-11 masks */
#define DVS_IRQ_00_DONE_EN	0x0001
#define DVS_IRQ_00_CLEAR	0x040000f0
#define DVS_IRQ_STATUS		0x05c
#define DVS_IRQ_STATUS_LATE	BIT(2)
#define DPE7_DONE_POLL_US	20000	/* after the late event, for the done bit */
#define DVS_CTRL_STATUS0	0x050
#define DVS_CTRL_STATUS0_DONE	BIT(31)
#define DVS_DRAM_PITCH		0x09c
#define DVS_SRC_00		0x100
#define DVS_SRC_01		0x104	/* frame height 0-10, width 12-22 */
#define DVS_SRC_02		0x108	/* engine height 0-10, width 12-22 */
#define DVS_SRC_46		0x1b8
#define DVS_SRC_00_VMAP_ALL_VLD	BIT(9)
#define DVS_SRC_05_L		0x114	/* left frames 0-3 */
#define DVS_SRC_09_R		0x124	/* right frames 0-3 */
#define DVS_SRC_13_LV		0x134	/* left valid maps 0-3 */
#define DVS_SRC_17_RV		0x144	/* right valid maps 0-3 */
#define DVS_SRC_21_P4L		0x154
#define DVS_SRC_26_OCC		0x168	/* disparity out 0-3 */
#define DVS_SRC_34_P4R		0x188
#define DVS_SRC_42_EXT		0x1a8	/* extension out 0-3 */
#define DVS_DRAM_SEC		0x1d0
#define DVS_CTRL_ATPG		0x2fc
#define DVP_CTRL00		0x800
#define DVP_CTRL00_EN		BIT(31)
#define DVP_CTRL01		0x804

#define DPE7_DEF_H		448	/* eye height: the reset frame height */
#define DPE7_MIN_H		256	/* heights measured: 256-1000 */
#define DPE7_MAX_H		1000
#define DPE7_H_MARGIN		20	/* output rows = eye height - 20 (engine from y = 10) */
#define DPE7_WIN_W		384	/* one window: the reset frame width */
#define DPE7_WIN_OCC		320	/* its disparity columns */
#define DPE7_WIN_STRIDE		384	/* its output stride (reset) */
#define DPE7_WIN_OUT		(DPE7_WIN_STRIDE * (DPE7_MAX_H - DPE7_H_MARGIN))
#define DPE7_MAX_WIN		6
#define DPE7_MAX_BPL		2048
#define DPE7_PITCH_OUT_RESET	(30u << 20)	/* bits 20-29 as at reset; bit 31 clear */
/* DVS_SRC_46 at reset is 009681e0: out_adj_dv height 480 in bits 0-10 caps the output */
#define DVS_SRC_46_RESET_HI	0x00968000

/* driver-owned buffers the engine needs but the API does not carry */
#define DPE7_AUX_VMAP		0		/* valid map, all 0xff, both eyes */
#define DPE7_AUX_SCRATCH	SZ_2M		/* unused frame slots */
#define DPE7_AUX_EXT		(SZ_2M + SZ_1M)	/* extension output */
#define DPE7_AUX_SIZE		SZ_4M
/*
 * Per-window disparity output, read back by the CPU for the copy: a cacheable
 * (non-coherent) buffer, synced around each job. Read from coherent memory, which is
 * uncached here, the copy cost about 1.7 ms per window.
 */
#define DPE7_WIN_SIZE		(DPE7_MAX_WIN * DPE7_WIN_OUT)

#define DPE7_TIMEOUT_US		200000

struct mtk_dpe7 {
	struct device *dev;
	void __iomem *base;
	struct clk_bulk_data *clks;
	int num_clks;
	struct v4l2_device v4l2_dev;
	struct video_device vdev;
	struct v4l2_m2m_dev *m2m_dev;
	struct mutex dev_lock;		/* serialises open/release and the video device */
	struct workqueue_struct *wq;
	struct work_struct job_work;
	struct completion done;
	int irq;
	void *aux;
	dma_addr_t aux_iova;
	void *win;
	dma_addr_t win_iova;
};

struct mtk_dpe7_ctx {
	struct v4l2_fh fh;
	struct mtk_dpe7 *dpe;
	struct mutex vb_lock;		/* the vb2 queues' lock */
	u32 windows;			/* k: OUTPUT width 64 + 320 k */
	u32 bpl;			/* OUTPUT bytesperline */
	u32 height;			/* eye height: OUTPUT is twice it */
};

static inline struct mtk_dpe7_ctx *file_to_ctx(struct file *file)
{
	return container_of(file_to_v4l2_fh(file), struct mtk_dpe7_ctx, fh);
}

static void dpe7_mask(struct mtk_dpe7 *dpe, u32 off, u32 val, u32 mask)
{
	writel((readl(dpe->base + off) & ~mask) | (val & mask), dpe->base + off);
}

static void dpe7_reset(struct mtk_dpe7 *dpe)
{
	dpe7_mask(dpe, DVS_CTRL01, DVS_CTRL01_RESET, DVS_CTRL01_RESET);
	dpe7_mask(dpe, DVP_CTRL01, DVS_CTRL01_RESET, DVS_CTRL01_RESET);
	dpe7_mask(dpe, DVS_CTRL01, 0, DVS_CTRL01_RESET);
	dpe7_mask(dpe, DVP_CTRL01, 0, DVS_CTRL01_RESET);
}

static void dpe7_addr(struct mtk_dpe7 *dpe, u32 off, dma_addr_t iova)
{
	writel(iova >> 4, dpe->base + off);	/* address registers hold IOVA / 16 */
}

/* one window: left/right at their window's x, disparity to dst at the reset stride */
static int dpe7_run_hw(struct mtk_dpe7 *dpe, dma_addr_t left, dma_addr_t right,
		       dma_addr_t dst, u32 bpl, u32 h)
{
	u32 out_rows = h - DPE7_H_MARGIN;
	dma_addr_t vmap = dpe->aux_iova + DPE7_AUX_VMAP;
	dma_addr_t ext = dpe->aux_iova + DPE7_AUX_EXT;
	dma_addr_t scr = dpe->aux_iova + DPE7_AUX_SCRATCH;
	u32 st;
	int i, ret;

	dpe7_reset(dpe);
	dpe7_mask(dpe, DVP_CTRL00, DVP_CTRL00_EN, DVP_CTRL00_EN);
	dpe7_mask(dpe, DVS_CTRL00, DVS_CTRL00_DVS_ONLY, DVS_CTRL00_MODE_MASK);
	writel(DVS_CTRL02_DEFAULT, dpe->base + DVS_CTRL02);
	dpe7_mask(dpe, DVS_IRQ_00, DVS_IRQ_00_DONE_EN, DVS_IRQ_00_EN_MASK);
	writel((bpl >> 4) | (bpl >> 4) << 10 | DPE7_PITCH_OUT_RESET,
	       dpe->base + DVS_DRAM_PITCH);
	/* height: frame, engine (frame - 20), and the out_adj_dv height that caps the rows */
	writel(h | DPE7_WIN_W << 12, dpe->base + DVS_SRC_01);
	writel(out_rows | (DPE7_WIN_W - 32) << 12, dpe->base + DVS_SRC_02);
	writel(DVS_SRC_46_RESET_HI | out_rows, dpe->base + DVS_SRC_46);
	dpe7_mask(dpe, DVS_SRC_00, DVS_SRC_00_VMAP_ALL_VLD, DVS_SRC_00_VMAP_ALL_VLD);
	for (i = 0; i < 4; i++) {
		dpe7_addr(dpe, DVS_SRC_05_L + 4 * i, i ? scr : left);
		dpe7_addr(dpe, DVS_SRC_09_R + 4 * i, i ? scr : right);
		dpe7_addr(dpe, DVS_SRC_13_LV + 4 * i, i ? scr : vmap);
		dpe7_addr(dpe, DVS_SRC_17_RV + 4 * i, i ? scr : vmap);
		dpe7_addr(dpe, DVS_SRC_26_OCC + 4 * i, i ? scr : dst);
		dpe7_addr(dpe, DVS_SRC_42_EXT + 4 * i, i ? scr : ext);
	}
	dpe7_addr(dpe, DVS_SRC_21_P4L, scr);
	dpe7_addr(dpe, DVS_SRC_34_P4R, scr);
	writel(0, dpe->base + DVS_DRAM_SEC);
	writel(BIT(31), dpe->base + DVS_CTRL_ATPG);

	reinit_completion(&dpe->done);
	/* the line is enabled only while the block is powered and a job runs */
	enable_irq(dpe->irq);
	dpe7_mask(dpe, DVS_CTRL00, DVS_CTRL00_FW_TRIG, DVS_CTRL00_FW_TRIG);
	ret = wait_for_completion_timeout(&dpe->done, usecs_to_jiffies(DPE7_TIMEOUT_US)) ?
		0 : -ETIMEDOUT;
	disable_irq(dpe->irq);
	if (!ret)
		ret = readl_poll_timeout(dpe->base + DVS_CTRL_STATUS0, st,
					 st & DVS_CTRL_STATUS0_DONE, 20, DPE7_DONE_POLL_US);
	dpe7_mask(dpe, DVS_CTRL00, 0, DVS_CTRL00_FW_TRIG);
	writel(DVS_IRQ_00_CLEAR, dpe->base + DVS_IRQ_00);
	writel(0x04000f00, dpe->base + DVS_IRQ_00);
	if (ret) {
		dev_err(dpe->dev, "DVS job failed (%d), status %08x\n", ret,
			readl(dpe->base + DVS_CTRL_STATUS0));
		dpe7_reset(dpe);
	}
	return ret;
}

static irqreturn_t dpe7_irq(int irq, void *data)
{
	struct mtk_dpe7 *dpe = data;
	u32 irqst = readl(dpe->base + DVS_IRQ_STATUS);
	u32 st = readl(dpe->base + DVS_CTRL_STATUS0);

	if (!irqst)
		return IRQ_NONE;
	/* the vendor's clear drops DVS_IRQ_STATUS and the level line and turns the enables off */
	writel(DVS_IRQ_00_CLEAR, dpe->base + DVS_IRQ_00);
	/*
	 * Measured on the G5: status bit 1 at the start of a job, bit 2 about 2.5 ms before
	 * done; the done bit raises no interrupt of its own. Bit 2 (or done) ends the wait and
	 * the job thread polls done; an earlier event re-arms.
	 */
	if ((irqst & DVS_IRQ_STATUS_LATE) || (st & DVS_CTRL_STATUS0_DONE))
		complete(&dpe->done);
	else
		dpe7_mask(dpe, DVS_IRQ_00, DVS_IRQ_00_DONE_EN, DVS_IRQ_00_EN_MASK);
	return IRQ_HANDLED;
}

static void dpe7_job_work(struct work_struct *work)
{
	struct mtk_dpe7 *dpe = container_of(work, struct mtk_dpe7, job_work);
	struct mtk_dpe7_ctx *ctx = v4l2_m2m_get_curr_priv(dpe->m2m_dev);
	struct vb2_v4l2_buffer *src, *dst;
	enum vb2_buffer_state state = VB2_BUF_STATE_ERROR;
	u32 k, w, row, out_w, out_h;
	dma_addr_t in;
	u8 *out;
	int ret;

	if (!ctx)
		return;
	src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	k = ctx->windows;
	out_w = k * DPE7_WIN_OCC;
	out_h = ctx->height - DPE7_H_MARGIN;
	in = vb2_dma_contig_plane_dma_addr(&src->vb2_buf, 0);
	out = vb2_plane_vaddr(&dst->vb2_buf, 0);

	ret = out ? pm_runtime_resume_and_get(dpe->dev) : -ENOMEM;
	if (!ret) {
		dma_sync_single_for_device(dpe->dev, dpe->win_iova, k * DPE7_WIN_OUT,
					   DMA_FROM_DEVICE);
		for (w = 0; w < k && !ret; w++)
			ret = dpe7_run_hw(dpe, in + w * DPE7_WIN_OCC,
					  in + ctx->height * ctx->bpl + w * DPE7_WIN_OCC,
					  dpe->win_iova + w * DPE7_WIN_OUT, ctx->bpl,
					  ctx->height);
		pm_runtime_put_autosuspend(dpe->dev);
	}
	if (!ret) {
		dma_sync_single_for_cpu(dpe->dev, dpe->win_iova, k * DPE7_WIN_OUT,
					DMA_FROM_DEVICE);
		/* each window's first 320 columns, side by side */
		for (w = 0; w < k; w++)
			for (row = 0; row < out_h; row++)
				memcpy(out + row * out_w + w * DPE7_WIN_OCC,
				       dpe->win + w * DPE7_WIN_OUT + row * DPE7_WIN_STRIDE,
				       DPE7_WIN_OCC);
		state = VB2_BUF_STATE_DONE;
		vb2_set_plane_payload(&dst->vb2_buf, 0, out_w * out_h);
	}
	v4l2_m2m_buf_copy_metadata(src, dst, true);
	v4l2_m2m_buf_done_and_job_finish(dpe->m2m_dev, ctx->fh.m2m_ctx, state);
}

static void dpe7_device_run(void *priv)
{
	struct mtk_dpe7_ctx *ctx = priv;

	queue_work(ctx->dpe->wq, &ctx->dpe->job_work);
}

static const struct v4l2_m2m_ops dpe7_m2m_ops = {
	.device_run = dpe7_device_run,
};

static u32 dpe7_in_width(u32 windows)
{
	return DPE7_WIN_W - DPE7_WIN_OCC + windows * DPE7_WIN_OCC;	/* 64 + 320 k */
}

static void dpe7_fill_fmt(u32 type, u32 windows, u32 bpl, u32 height,
			  struct v4l2_pix_format *pix)
{
	memset(pix, 0, sizeof(*pix));
	pix->pixelformat = V4L2_PIX_FMT_GREY;
	pix->field = V4L2_FIELD_NONE;
	pix->colorspace = V4L2_COLORSPACE_RAW;
	if (V4L2_TYPE_IS_OUTPUT(type)) {
		pix->width = dpe7_in_width(windows);
		pix->height = 2 * height;
		pix->bytesperline = bpl;
	} else {
		pix->width = windows * DPE7_WIN_OCC;
		pix->height = height - DPE7_H_MARGIN;
		pix->bytesperline = pix->width;
	}
	pix->sizeimage = pix->bytesperline * pix->height;
}

/*
 * The nearest supported OUTPUT size: width 64 + 320 k, a bytesperline that fits it, and an
 * eye height (half the buffer height) that is even and within the measured 256-1000.
 */
static void dpe7_try_out(struct v4l2_pix_format *pix, u32 *windows, u32 *bpl, u32 *height)
{
	u32 w = max_t(u32, pix->width, dpe7_in_width(1));

	*windows = clamp_t(u32, DIV_ROUND_CLOSEST(w - dpe7_in_width(0), DPE7_WIN_OCC), 1,
			   DPE7_MAX_WIN);
	*bpl = clamp_t(u32, ALIGN(max(pix->bytesperline, dpe7_in_width(*windows)), 16),
		       dpe7_in_width(*windows), DPE7_MAX_BPL);
	*height = pix->height ? clamp_t(u32, ALIGN_DOWN(pix->height / 2, 2), DPE7_MIN_H,
					DPE7_MAX_H) : DPE7_DEF_H;
}

static int dpe7_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
	strscpy(cap->driver, DPE7_NAME, sizeof(cap->driver));
	strscpy(cap->card, "MediaTek MT8188 DPE stereo disparity", sizeof(cap->card));
	return 0;
}

static int dpe7_enum_fmt(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;
	f->pixelformat = V4L2_PIX_FMT_GREY;
	return 0;
}

/* G (both queues) and TRY/S on CAPTURE: the CAPTURE format follows the OUTPUT width */
static int dpe7_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct mtk_dpe7_ctx *ctx = file_to_ctx(file);

	dpe7_fill_fmt(f->type, ctx->windows, ctx->bpl, ctx->height, &f->fmt.pix);
	return 0;
}

static int dpe7_try_fmt_out(struct file *file, void *priv, struct v4l2_format *f)
{
	u32 windows, bpl, height;

	dpe7_try_out(&f->fmt.pix, &windows, &bpl, &height);
	dpe7_fill_fmt(f->type, windows, bpl, height, &f->fmt.pix);
	return 0;
}

static int dpe7_s_fmt_out(struct file *file, void *priv, struct v4l2_format *f)
{
	struct mtk_dpe7_ctx *ctx = file_to_ctx(file);
	u32 windows, bpl, height;

	if (vb2_is_busy(v4l2_m2m_get_vq(ctx->fh.m2m_ctx, V4L2_BUF_TYPE_VIDEO_OUTPUT)) ||
	    vb2_is_busy(v4l2_m2m_get_vq(ctx->fh.m2m_ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE)))
		return -EBUSY;
	dpe7_try_out(&f->fmt.pix, &windows, &bpl, &height);
	ctx->windows = windows;
	ctx->bpl = bpl;
	ctx->height = height;
	dpe7_fill_fmt(f->type, windows, bpl, height, &f->fmt.pix);
	return 0;
}

static int dpe7_enum_framesizes(struct file *file, void *priv,
				struct v4l2_frmsizeenum *fs)
{
	if (fs->index || fs->pixel_format != V4L2_PIX_FMT_GREY)
		return -EINVAL;
	/* the OUTPUT buffer: two eyes stacked, so twice an even eye height */
	fs->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fs->stepwise.min_width = dpe7_in_width(1);
	fs->stepwise.max_width = dpe7_in_width(DPE7_MAX_WIN);
	fs->stepwise.step_width = DPE7_WIN_OCC;
	fs->stepwise.min_height = 2 * DPE7_MIN_H;
	fs->stepwise.max_height = 2 * DPE7_MAX_H;
	fs->stepwise.step_height = 4;
	return 0;
}

static const struct v4l2_ioctl_ops dpe7_ioctl_ops = {
	.vidioc_querycap = dpe7_querycap,
	.vidioc_enum_fmt_vid_cap = dpe7_enum_fmt,
	.vidioc_enum_fmt_vid_out = dpe7_enum_fmt,
	.vidioc_g_fmt_vid_cap = dpe7_g_fmt,
	.vidioc_try_fmt_vid_cap = dpe7_g_fmt,
	.vidioc_s_fmt_vid_cap = dpe7_g_fmt,
	.vidioc_g_fmt_vid_out = dpe7_g_fmt,
	.vidioc_try_fmt_vid_out = dpe7_try_fmt_out,
	.vidioc_s_fmt_vid_out = dpe7_s_fmt_out,
	.vidioc_enum_framesizes = dpe7_enum_framesizes,
	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_prepare_buf = v4l2_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs = v4l2_m2m_ioctl_create_bufs,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,
	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,
};

static int dpe7_queue_setup(struct vb2_queue *vq, unsigned int *nbuffers,
			    unsigned int *nplanes, unsigned int sizes[],
			    struct device *alloc_devs[])
{
	struct mtk_dpe7_ctx *ctx = vb2_get_drv_priv(vq);
	struct v4l2_pix_format pix;

	dpe7_fill_fmt(vq->type, ctx->windows, ctx->bpl, ctx->height, &pix);
	if (*nplanes)
		return (*nplanes == 1 && sizes[0] >= pix.sizeimage) ? 0 : -EINVAL;
	*nplanes = 1;
	sizes[0] = pix.sizeimage;
	return 0;
}

static int dpe7_buf_prepare(struct vb2_buffer *vb)
{
	struct mtk_dpe7_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct v4l2_pix_format pix;

	dpe7_fill_fmt(vb->vb2_queue->type, ctx->windows, ctx->bpl, ctx->height, &pix);
	if (vb2_plane_size(vb, 0) < pix.sizeimage)
		return -EINVAL;
	if (V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type) &&
	    vb2_get_plane_payload(vb, 0) < pix.sizeimage)
		return -EINVAL;
	/* the engine addresses buffers in 16-byte units */
	if (vb2_dma_contig_plane_dma_addr(vb, 0) & 0xf)
		return -EINVAL;
	return 0;
}

static void dpe7_buf_queue(struct vb2_buffer *vb)
{
	struct mtk_dpe7_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, to_vb2_v4l2_buffer(vb));
}

static void dpe7_stop_streaming(struct vb2_queue *vq)
{
	struct mtk_dpe7_ctx *ctx = vb2_get_drv_priv(vq);
	struct vb2_v4l2_buffer *vb;

	for (;;) {
		vb = V4L2_TYPE_IS_OUTPUT(vq->type) ?
			v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx) :
			v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
		if (!vb)
			break;
		v4l2_m2m_buf_done(vb, VB2_BUF_STATE_ERROR);
	}
}

static const struct vb2_ops dpe7_vb2_ops = {
	.queue_setup = dpe7_queue_setup,
	.buf_prepare = dpe7_buf_prepare,
	.buf_queue = dpe7_buf_queue,
	.stop_streaming = dpe7_stop_streaming,
};

static int dpe7_queue_init(void *priv, struct vb2_queue *src, struct vb2_queue *dst)
{
	struct mtk_dpe7_ctx *ctx = priv;
	int ret;

	src->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	src->io_modes = VB2_MMAP | VB2_DMABUF;
	src->drv_priv = ctx;
	src->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src->ops = &dpe7_vb2_ops;
	src->mem_ops = &vb2_dma_contig_memops;
	src->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src->lock = &ctx->vb_lock;
	src->dev = ctx->dpe->dev;
	ret = vb2_queue_init(src);
	if (ret)
		return ret;

	dst->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dst->io_modes = VB2_MMAP | VB2_DMABUF;
	dst->drv_priv = ctx;
	dst->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst->ops = &dpe7_vb2_ops;
	dst->mem_ops = &vb2_dma_contig_memops;
	dst->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst->lock = &ctx->vb_lock;
	dst->dev = ctx->dpe->dev;
	return vb2_queue_init(dst);
}

static int dpe7_open(struct file *file)
{
	struct mtk_dpe7 *dpe = video_drvdata(file);
	struct mtk_dpe7_ctx *ctx;
	int ret;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	ctx->dpe = dpe;
	mutex_init(&ctx->vb_lock);
	ctx->windows = 1;		/* 384 wide */
	ctx->bpl = 480;			/* the block's reset pitch */
	ctx->height = DPE7_DEF_H;
	v4l2_fh_init(&ctx->fh, video_devdata(file));

	mutex_lock(&dpe->dev_lock);
	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(dpe->m2m_dev, ctx, dpe7_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		mutex_unlock(&dpe->dev_lock);
		v4l2_fh_exit(&ctx->fh);
		mutex_destroy(&ctx->vb_lock);
		kfree(ctx);
		return ret;
	}
	v4l2_fh_add(&ctx->fh, file);
	mutex_unlock(&dpe->dev_lock);
	return 0;
}

static int dpe7_release(struct file *file)
{
	struct mtk_dpe7_ctx *ctx = file_to_ctx(file);
	struct mtk_dpe7 *dpe = ctx->dpe;

	mutex_lock(&dpe->dev_lock);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	mutex_unlock(&dpe->dev_lock);
	v4l2_fh_del(&ctx->fh, file);
	v4l2_fh_exit(&ctx->fh);
	mutex_destroy(&ctx->vb_lock);
	kfree(ctx);
	return 0;
}

static const struct v4l2_file_operations dpe7_fops = {
	.owner = THIS_MODULE,
	.open = dpe7_open,
	.release = dpe7_release,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
};

static int dpe7_runtime_suspend(struct device *dev)
{
	struct mtk_dpe7 *dpe = dev_get_drvdata(dev);

	clk_bulk_disable_unprepare(dpe->num_clks, dpe->clks);
	return 0;
}

static int dpe7_runtime_resume(struct device *dev)
{
	struct mtk_dpe7 *dpe = dev_get_drvdata(dev);

	return clk_bulk_prepare_enable(dpe->num_clks, dpe->clks);
}

static void dpe7_free_win(void *data)
{
	struct mtk_dpe7 *dpe = data;

	dma_free_noncoherent(dpe->dev, DPE7_WIN_SIZE, dpe->win, dpe->win_iova, DMA_FROM_DEVICE);
}

static int dpe7_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_dpe7 *dpe;
	int ret;

	dpe = devm_kzalloc(dev, sizeof(*dpe), GFP_KERNEL);
	if (!dpe)
		return -ENOMEM;
	dpe->dev = dev;
	platform_set_drvdata(pdev, dpe);
	mutex_init(&dpe->dev_lock);
	INIT_WORK(&dpe->job_work, dpe7_job_work);
	init_completion(&dpe->done);

	dpe->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dpe->base))
		return PTR_ERR(dpe->base);
	dpe->irq = platform_get_irq(pdev, 0);
	if (dpe->irq < 0)
		return dpe->irq;
	ret = devm_request_irq(dev, dpe->irq, dpe7_irq, IRQF_NO_AUTOEN, DPE7_NAME, dpe);
	if (ret)
		return dev_err_probe(dev, ret, "IRQ %d\n", dpe->irq);
	dpe->num_clks = devm_clk_bulk_get_all(dev, &dpe->clks);
	if (dpe->num_clks < 0)
		return dev_err_probe(dev, dpe->num_clks, "clocks\n");
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(36));
	if (ret)
		return dev_err_probe(dev, ret, "DMA mask\n");

	dpe->aux = dmam_alloc_coherent(dev, DPE7_AUX_SIZE, &dpe->aux_iova, GFP_KERNEL);
	if (!dpe->aux)
		return -ENOMEM;
	memset(dpe->aux + DPE7_AUX_VMAP, 0xff, DPE7_AUX_SCRATCH - DPE7_AUX_VMAP);
	dpe->win = dma_alloc_noncoherent(dev, DPE7_WIN_SIZE, &dpe->win_iova, DMA_FROM_DEVICE,
					 GFP_KERNEL);
	if (!dpe->win)
		return -ENOMEM;
	ret = devm_add_action_or_reset(dev, dpe7_free_win, dpe);
	if (ret)
		return ret;

	dpe->wq = alloc_ordered_workqueue(DPE7_NAME, WQ_MEM_RECLAIM);
	if (!dpe->wq)
		return -ENOMEM;

	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		goto err_wq;

	ret = v4l2_device_register(dev, &dpe->v4l2_dev);
	if (ret)
		goto err_wq;
	dpe->m2m_dev = v4l2_m2m_init(&dpe7_m2m_ops);
	if (IS_ERR(dpe->m2m_dev)) {
		ret = PTR_ERR(dpe->m2m_dev);
		goto err_v4l2;
	}

	dpe->vdev.fops = &dpe7_fops;
	dpe->vdev.ioctl_ops = &dpe7_ioctl_ops;
	dpe->vdev.release = video_device_release_empty;
	dpe->vdev.lock = &dpe->dev_lock;
	dpe->vdev.v4l2_dev = &dpe->v4l2_dev;
	dpe->vdev.vfl_dir = VFL_DIR_M2M;
	dpe->vdev.device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING;
	strscpy(dpe->vdev.name, DPE7_NAME, sizeof(dpe->vdev.name));
	video_set_drvdata(&dpe->vdev, dpe);
	ret = video_register_device(&dpe->vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_m2m;
	dev_info(dev, "DVS stereo disparity on %s\n", video_device_node_name(&dpe->vdev));
	return 0;

err_m2m:
	v4l2_m2m_release(dpe->m2m_dev);
err_v4l2:
	v4l2_device_unregister(&dpe->v4l2_dev);
err_wq:
	destroy_workqueue(dpe->wq);
	return ret;
}

static void dpe7_remove(struct platform_device *pdev)
{
	struct mtk_dpe7 *dpe = platform_get_drvdata(pdev);

	video_unregister_device(&dpe->vdev);
	v4l2_m2m_release(dpe->m2m_dev);
	v4l2_device_unregister(&dpe->v4l2_dev);
	destroy_workqueue(dpe->wq);
}

static const struct dev_pm_ops dpe7_pm_ops = {
	RUNTIME_PM_OPS(dpe7_runtime_suspend, dpe7_runtime_resume, NULL)
};

static const struct of_device_id dpe7_of_match[] = {
	{ .compatible = "mediatek,mt8188-dpe" },
	{ }
};
MODULE_DEVICE_TABLE(of, dpe7_of_match);

static struct platform_driver dpe7_driver = {
	.probe = dpe7_probe,
	.remove = dpe7_remove,
	.driver = {
		.name = DPE7_NAME,
		.of_match_table = dpe7_of_match,
		.pm = pm_ptr(&dpe7_pm_ops),
	},
};
module_platform_driver(dpe7_driver);

MODULE_DESCRIPTION("MediaTek MT8188 DPE stereo disparity (DVS) driver");
MODULE_LICENSE("GPL");
