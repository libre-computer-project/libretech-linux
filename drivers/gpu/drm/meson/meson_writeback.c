// SPDX-License-Identifier: GPL-2.0+
/*
 * Amlogic Meson DRM Writeback Connector
 * Uses GE2D 2D engine to capture primary plane framebuffer content.
 *
 * SCOPE (intentional, documented limitation -- not a bug): this captures
 * ONLY the OSD1 primary-plane buffer, not the composited CRTC/VPP output.
 * VD1/VD2 video, the OSD2 overlay, blending, CTM/gamma and PQ are absent
 * from the captured image. The full rationale is finding #22 of an internal
 * HDMI/VPU gap review; no genuine post-blend capture source was found to
 * switch to.
 *
 * Copyright (C) 2025 Libre Computer
 */

#include <linux/list.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

#include <linux/soc/amlogic/meson-ge2d.h>

#include <drm/drm_atomic.h>
#include <drm/drm_edid.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_encoder.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_writeback.h>

#include "meson_drv.h"
#include "meson_writeback.h"

static const u32 meson_wb_formats[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
};

/*
 * #23 fix: the GE2D capture is deferred off the commit-tail synchronous
 * path (see meson_wb_atomic_commit()) until the vsync IRQ has actually
 * latched priv->viu.osd1_addr into the live OSD1 canvas (meson_crtc_irq(),
 * right after the osd1_commit block programs VIU_OSD1 canvas 0). Before
 * that latch priv->viu.osd1_addr is only a software shadow of the NEXT
 * frame meson_crtc_irq() will program -- reading it synchronously in the
 * commit tail (the previous implementation) could capture a buffer ahead
 * of what is actually driving the display, or race a following commit's
 * shadow write before this one's data had been consumed.
 *
 * One pending job is queued per meson_wb_atomic_commit() call (destination
 * side only -- fixed for the life of the job). meson_writeback_vsync_latched()
 * fills in the source side from the just-latched priv->viu.osd1_* shadow +
 * the primary plane's live drm_framebuffer, then schedules
 * meson_wb_blit_work() to run the actual (sleeping) GE2D call and signal
 * completion.
 */
struct meson_wb_pending_job {
	struct list_head node;
	struct drm_writeback_connector *wb_conn;

	/* Destination (writeback target fb): fixed at atomic_commit time. */
	dma_addr_t dst_addr;
	u32 dst_w, dst_h, dst_stride;
	enum meson_ge2d_fmt dst_fmt;

	/*
	 * Source (OSD1): filled in by meson_writeback_vsync_latched() once
	 * priv->viu.osd1_addr is confirmed latched, not before.
	 */
	bool src_ready;
	bool src_fmt_supported;
	dma_addr_t src_addr;
	u32 src_w, src_h, src_stride;
	enum meson_ge2d_fmt src_fmt;
};

struct meson_wb_connector {
	struct drm_writeback_connector base;
	struct meson_drm *priv;

	/* Protects pending_jobs against meson_wb_atomic_commit() (process
	 * context) vs. meson_writeback_vsync_latched() (hard IRQ).
	 */
	spinlock_t lock;
	struct list_head pending_jobs;
	struct work_struct blit_work;
};

/*
 * #22 fix (Option A -- see meson_wb_atomic_commit()): map the OSD1 plane's
 * ACTUAL live DRM format to the meson_ge2d kernel API's format enum,
 * instead of hardcoding GE2D_FMT_XRGB8888 for every source pixel layout.
 * Only the three formats meson-ge2d.h can represent with the correct
 * channel order/depth are mapped; meson_ge2d_fmt has no distinct
 * enumerator for the byte-swapped BGR variants, the alpha-first RGBA/BGRA
 * variants, or RGB565's different bit depth that OSD1 also advertises
 * (supported_drm_formats[] in meson_plane.c), so those report
 * "unsupported" rather than being silently reinterpreted as XRGB8888.
 */
static bool meson_wb_ge2d_src_fmt(u32 drm_fmt, enum meson_ge2d_fmt *out)
{
	switch (drm_fmt) {
	case DRM_FORMAT_XRGB8888:
		*out = GE2D_FMT_XRGB8888;
		return true;
	case DRM_FORMAT_ARGB8888:
		*out = GE2D_FMT_ARGB8888;
		return true;
	case DRM_FORMAT_RGB888:
		*out = GE2D_FMT_RGB888;
		return true;
	default:
		return false;
	}
}

static const struct drm_connector_funcs meson_wb_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int meson_wb_atomic_check(struct drm_connector *connector,
				 struct drm_atomic_state *state)
{
	struct drm_connector_state *conn_state =
		drm_atomic_get_new_connector_state(state, connector);
	struct drm_crtc_state *crtc_state;
	struct drm_framebuffer *fb;
	int ret;

	if (!conn_state->writeback_job || !conn_state->writeback_job->fb)
		return 0;

	if (!conn_state->crtc)
		return 0;

	crtc_state = drm_atomic_get_new_crtc_state(state, conn_state->crtc);
	fb = conn_state->writeback_job->fb;

	/* Writeback FB must match display mode dimensions */
	if (fb->width != crtc_state->mode.hdisplay ||
	    fb->height != crtc_state->mode.vdisplay)
		return -EINVAL;

	ret = drm_atomic_helper_check_wb_connector_state(connector, state);
	if (ret < 0)
		return ret;

	return 0;
}

static int meson_wb_get_modes(struct drm_connector *connector)
{
	struct drm_device *dev = connector->dev;

	return drm_add_modes_noedid(connector, dev->mode_config.max_width,
				    dev->mode_config.max_height);
}

static int meson_wb_prepare_job(struct drm_writeback_connector *wb_connector,
				struct drm_writeback_job *job)
{
	struct drm_gem_dma_object *gem;

	if (!job->fb)
		return 0;

	gem = drm_fb_dma_get_gem_obj(job->fb, 0);
	if (!gem)
		return -EINVAL;

	/* Store DMA address for use in commit */
	job->priv = (void *)(uintptr_t)gem->dma_addr;

	return 0;
}

static void meson_wb_cleanup_job(struct drm_writeback_connector *wb_connector,
				 struct drm_writeback_job *job)
{
	/* Nothing to clean up — we used the DMA address directly */
}

static void meson_wb_atomic_commit(struct drm_connector *connector,
				   struct drm_atomic_state *state)
{
	struct drm_connector_state *conn_state =
		drm_atomic_get_new_connector_state(state, connector);
	struct meson_drm *priv = connector->dev->dev_private;
	struct meson_wb_connector *mwb =
		container_of(connector, struct meson_wb_connector, base.base);
	struct drm_writeback_connector *wb_conn = &mwb->base;
	struct drm_framebuffer *fb = conn_state->writeback_job->fb;
	dma_addr_t dst_addr = (dma_addr_t)(uintptr_t)conn_state->writeback_job->priv;
	struct meson_wb_pending_job *pjob;
	unsigned long flags;

	/*
	 * Queue the job before any early return so that every
	 * drm_writeback_signal_completion() below is balanced by exactly one
	 * queued job -- drm_writeback_signal_completion() does WARN_ON(!job)
	 * and pops from an empty queue otherwise. fb/dst_addr are captured
	 * above before drm_writeback_queue_job() clears conn_state->writeback_job.
	 * (Previously the -ENODEV path signalled an unqueued job, tripping the
	 * WARN whenever the ge2d module was not loaded.)
	 */
	drm_writeback_queue_job(wb_conn, conn_state);

	/* Lazy GE2D acquisition — module may have loaded since init */
	if (!priv->ge2d) {
		priv->ge2d = meson_ge2d_get(priv->dev);
		if (IS_ERR(priv->ge2d)) {
			priv->ge2d = NULL;
			drm_writeback_signal_completion(wb_conn, -ENODEV);
			return;
		}
	}

	/*
	 * #23: do NOT read priv->viu.osd1_addr or blit here. It is only a
	 * software shadow of the next VIU_OSD1 canvas write, which
	 * meson_crtc_irq() performs at most once per vsync, gated on
	 * osd1_commit -- see meson_writeback_vsync_latched() below, called
	 * from meson_crtc_irq() right after that canvas write. Stash the
	 * destination side (fixed for this job, independent of OSD1) and
	 * let the post-latch hook fill in and dispatch the source side.
	 *
	 * #22 (Option A -- honest OSD1-plane capture, not full-CRTC): this
	 * writeback path captures ONLY the OSD1 primary-plane buffer. See
	 * the file banner and finding #22 in the VPU/plane audit doc.
	 */
	pjob = kzalloc(sizeof(*pjob), GFP_KERNEL);
	if (!pjob) {
		drm_writeback_signal_completion(wb_conn, -ENOMEM);
		return;
	}

	pjob->wb_conn = wb_conn;
	pjob->dst_addr = dst_addr;
	pjob->dst_w = fb->width;
	pjob->dst_h = fb->height;
	pjob->dst_stride = fb->pitches[0];
	pjob->dst_fmt = (fb->format->format == DRM_FORMAT_ARGB8888) ?
			 GE2D_FMT_ARGB8888 : GE2D_FMT_XRGB8888;

	spin_lock_irqsave(&mwb->lock, flags);
	list_add_tail(&pjob->node, &mwb->pending_jobs);
	spin_unlock_irqrestore(&mwb->lock, flags);
}

/*
 * Runs the deferred GE2D blit + drm_writeback_signal_completion() for
 * every job meson_writeback_vsync_latched() has marked src_ready. Process
 * context (workqueue) -- meson_ge2d_blit() sleeps (mutex + completion
 * wait), which meson_writeback_vsync_latched()'s hard-IRQ caller cannot do.
 */
static void meson_wb_blit_work(struct work_struct *work)
{
	struct meson_wb_connector *mwb =
		container_of(work, struct meson_wb_connector, blit_work);
	struct meson_drm *priv = mwb->priv;

	for (;;) {
		struct meson_wb_pending_job *pjob = NULL, *iter;
		unsigned long flags;
		int ret;

		spin_lock_irqsave(&mwb->lock, flags);
		list_for_each_entry(iter, &mwb->pending_jobs, node) {
			if (iter->src_ready) {
				pjob = iter;
				list_del(&pjob->node);
				break;
			}
		}
		spin_unlock_irqrestore(&mwb->lock, flags);

		if (!pjob)
			break;

		if (!pjob->src_fmt_supported) {
			ret = -EINVAL;
		} else {
			struct meson_ge2d_job job = {
				.src_addr = pjob->src_addr,
				.src_addr_uv = 0,
				.src_w = pjob->src_w,
				.src_h = pjob->src_h,
				.src_stride = pjob->src_stride,
				.src_fmt = pjob->src_fmt,
				.dst_addr = pjob->dst_addr,
				.dst_addr_uv = 0,
				.dst_w = pjob->dst_w,
				.dst_h = pjob->dst_h,
				.dst_stride = pjob->dst_stride,
				.dst_fmt = pjob->dst_fmt,
			};

			ret = meson_ge2d_blit(priv->ge2d, &job);
		}

		drm_writeback_signal_completion(pjob->wb_conn, ret);
		kfree(pjob);
	}
}

/*
 * Called from meson_crtc_irq() (hard IRQ) immediately after it programs
 * priv->viu.osd1_addr into the live OSD1 canvas (the osd1_commit block) --
 * i.e. exactly when priv->viu.osd1_* stops being a pending shadow and
 * becomes the buffer actually driving OSD1 scanout. Snapshots that state
 * plus the primary plane's live format into every not-yet-ready pending
 * writeback job, then hands them to meson_wb_blit_work() to actually
 * capture (GE2D sleeps, so it cannot run here).
 *
 * Hard-IRQ safe: only pointer/field reads and queue_work(), no allocation,
 * no sleeping. priv->primary_plane->state is always valid post-init (DRM
 * seeds it via plane->reset()); it holds whichever drm_framebuffer is
 * currently assigned to OSD1, matching what fed priv->viu.osd1_addr.
 */
void meson_writeback_vsync_latched(struct meson_drm *priv)
{
	struct meson_wb_connector *mwb;
	struct meson_wb_pending_job *iter;
	struct drm_framebuffer *osd1_fb;
	unsigned long flags;
	bool any_ready = false;

	if (!priv->writeback)
		return;

	mwb = container_of(priv->writeback, struct meson_wb_connector, base);
	osd1_fb = priv->primary_plane->state ? priv->primary_plane->state->fb : NULL;

	spin_lock_irqsave(&mwb->lock, flags);
	list_for_each_entry(iter, &mwb->pending_jobs, node) {
		if (iter->src_ready)
			continue;

		iter->src_addr = priv->viu.osd1_addr;
		iter->src_stride = priv->viu.osd1_stride;
		iter->src_w = priv->viu.osd1_width;
		iter->src_h = priv->viu.osd1_height;
		iter->src_fmt_supported = osd1_fb &&
			meson_wb_ge2d_src_fmt(osd1_fb->format->format,
					      &iter->src_fmt);
		iter->src_ready = true;
		any_ready = true;
	}
	spin_unlock_irqrestore(&mwb->lock, flags);

	if (any_ready)
		queue_work(system_unbound_wq, &mwb->blit_work);
}

static const struct drm_connector_helper_funcs meson_wb_conn_helper_funcs = {
	.get_modes = meson_wb_get_modes,
	.prepare_writeback_job = meson_wb_prepare_job,
	.cleanup_writeback_job = meson_wb_cleanup_job,
	.atomic_commit = meson_wb_atomic_commit,
	.atomic_check = meson_wb_atomic_check,
};

int meson_writeback_create(struct meson_drm *priv)
{
	struct meson_wb_connector *mwb;
	struct drm_writeback_connector *wb;
	int ret;

	/* GE2D acquired lazily on first writeback job — module may load later */
	priv->ge2d = NULL;

	mwb = devm_kzalloc(priv->dev, sizeof(*mwb), GFP_KERNEL);
	if (!mwb)
		return -ENOMEM;

	mwb->priv = priv;
	spin_lock_init(&mwb->lock);
	INIT_LIST_HEAD(&mwb->pending_jobs);
	INIT_WORK(&mwb->blit_work, meson_wb_blit_work);

	wb = &mwb->base;

	drm_connector_helper_add(&wb->base, &meson_wb_conn_helper_funcs);

	ret = drm_writeback_connector_init(priv->drm, wb,
					   &meson_wb_connector_funcs,
					   NULL,
					   meson_wb_formats,
					   ARRAY_SIZE(meson_wb_formats),
					   1);
	if (ret)
		return ret;

	/*
	 * The writeback encoder must be cloneable with all other encoders
	 * on the same CRTC, otherwise drm_atomic_helper_check_modeset()
	 * rejects the commit. Set possible_clones on all encoders sharing
	 * this CRTC to include each other, including each encoder's own bit.
	 */
	{
		struct drm_encoder *enc;
		u32 wb_mask = drm_encoder_mask(&wb->encoder);

		wb->encoder.possible_clones = 0;
		drm_for_each_encoder(enc, priv->drm) {
			if (enc->possible_crtcs & wb->encoder.possible_crtcs) {
				wb->encoder.possible_clones |=
					drm_encoder_mask(enc);
				enc->possible_clones |=
					drm_encoder_mask(enc) | wb_mask;
			}
		}
	}

	priv->writeback = wb;

	return 0;
}
