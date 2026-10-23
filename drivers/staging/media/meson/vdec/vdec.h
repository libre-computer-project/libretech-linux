/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2018 BayLibre, SAS
 * Author: Maxime Jourdan <mjourdan@baylibre.com>
 */

#ifndef __MESON_VDEC_CORE_H_
#define __MESON_VDEC_CORE_H_

#include <linux/irqreturn.h>
#include <linux/kref.h>
#include <linux/regmap.h>
#include <linux/list.h>
#include <linux/wait.h>
#include <media/media-device.h>
#include <media/videobuf2-v4l2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <linux/soc/amlogic/meson-canvas.h>

#include "vdec_platform.h"
#include "vdec_debugfs.h"

/**
 * struct amvdec_fw_version - identity + quirks for one known firmware binary
 * @label: human-readable identifier (e.g. "BSP V2603 V2.5")
 * @provenance: upstream commit or BSP release string
 * @size: expected byte size of the firmware
 * @sha256: full 32-byte SHA256 of the firmware
 * @quirks: codec-specific bitmask (each codec defines its own bits)
 * @latest: true on the newest entry for this firmware file
 */
struct amvdec_fw_version {
	const char *label;
	const char *provenance;
	size_t size;
	u8 sha256[32];
	u32 quirks;
	bool latest;
};

/* Custom V4L2 controls for meson-vdec tuning */
#define V4L2_CID_MESON_VDEC_MAX_ALLOC_BUFS  (V4L2_CID_USER_BASE + 0x1004)
#define V4L2_CID_MESON_VDEC_DPB_MARGIN      (V4L2_CID_USER_BASE + 0x1005)

/* 32 buffers in 3-plane YUV420 */
#define MAX_CANVAS (32 * 3)

struct amvdec_buffer {
	struct list_head list;
	struct vb2_buffer *vb;
};

/**
 * struct amvdec_timestamp - stores a src timestamp along with a VIFIFO offset
 *
 * @list: used to make lists out of this struct
 * @tc: timecode from the v4l2 buffer
 * @ts: timestamp from the VB2 buffer
 * @offset: offset in the VIFIFO where the associated packet was written
 * @flags: flags from the v4l2 buffer
 * @used_count: times this timestamp was checked for a match with a dst buffer
 */
struct amvdec_timestamp {
	struct list_head list;
	struct v4l2_timecode tc;
	u64 ts;
	u32 offset;
	u32 flags;
	u32 used_count;
	/* HEVC: this access unit's first slice is an IDR or BLA, i.e. it
	 * starts a new coded video sequence (set by esparser, KI-175).
	 */
	bool cvs_start;
};

struct amvdec_session;
struct amvenc_ctx;

/**
 * struct amvdec_core - device parameters, singleton
 *
 * @dos_base: DOS memory base address
 * @esparser_base: PARSER memory base address
 * @regmap_ao: regmap for the AO bus
 * @dev: core device
 * @dev_dec: decoder device
 * @platform: platform-specific data
 * @canvas: canvas provider reference
 * @dos_parser_clk: DOS_PARSER clock
 * @dos_clk: DOS clock
 * @vdec_1_clk: VDEC_1 clock
 * @vdec_hevc_clk: VDEC_HEVC clock
 * @vdec_hevcf_clk: VDEC_HEVCF clock
 * @esparser_reset: RESET for the PARSER
 * @vdev_dec: video device for the decoder
 * @v4l2_dev: v4l2 device
 * @cur_sess: current decoding session
 * @lock: video device lock
 */
struct amvdec_core {
	void __iomem *dos_base;
	void __iomem *esparser_base;
	struct regmap *regmap_ao;

	struct device *dev;
	struct device *dev_dec;
	const struct vdec_platform *platform;

	struct meson_canvas *canvas;

	/* Scratch buffer for safe decoder output before real buffers exist */
	void *scratch_vaddr;
	dma_addr_t scratch_paddr;
	u8 scratch_canvas[3]; /* Y, U/UV, V canvas IDs */

	struct clk *dos_parser_clk;
	struct clk *dos_clk;
	struct clk *vdec_1_clk;
	struct clk *vdec_hevc_clk;
	struct clk *vdec_hevcf_clk;
	struct clk *fclk_div2p5;	/* 800 MHz fixed divider (G12A+) */
	struct clk *gp0_pll;		/* programmable PLL for overclock (G12A+) */
	atomic_t hevc_clk_ref;		/* refcount: HEVC *clock* users (VDEC_1 needs the HEVC clock for the MDEC DC cache on G12A+, HEVC decode needs it too).  Distinct from hevc_power_users below, which refcounts the HEVC ENGINE power domain (KI-118 block manager). */

	struct reset_control *esparser_reset;
	struct reset_control *hevcf_dmc_pipl_reset;

	struct video_device *vdev_dec;
	struct video_device *vdev_mjpeg;	/* MJPEG-only device node */
	struct v4l2_device v4l2_dev;
	struct media_device mdev;
	struct v4l2_m2m_dev *m2m_dev;
	struct v4l2_m2m_dev *m2m_mjpeg;	/* m2m device for MJPEG node */

	/*
	 * Every open session, and the lifetime rules that make an unbind
	 * survivable (KI-162).
	 *
	 * @sessions/@sessions_lock: vdec_open() adds, vdec_close() removes.
	 * cur_sess alone is not enough for teardown: it names only the session
	 * that holds the single decode engine, while EVERY open session owns a
	 * decode_watchdog timer and may own a codec-private one, and a timer
	 * that outlives the device's iomem faults on its first DOS read.
	 *
	 * @unbound: set at the top of vdec_remove().  The V4L2 core already
	 * refuses ioctl/mmap/read/write on an unregistered node, so this only
	 * has to catch an open() that got past videodev_lock before the
	 * unregister.
	 *
	 * @ref: the m2m devices and this struct are CORE-owned but SESSION-
	 * referenced -- vdec_close() calls v4l2_m2m_ctx_release(), which
	 * dereferences m2m_ctx->m2m_dev, so neither may be freed while a file
	 * handle is open.  An unbind can happen with fds open, so devm/remove
	 * lifetime is wrong for them: probe takes the initial reference and
	 * hands it to devres, each open takes one, and the last put frees.
	 */
	struct list_head sessions;
	struct mutex sessions_lock;
	bool unbound;
	struct kref ref;

	struct amvdec_session *cur_sess;
	/*
	 * Serializes the cur_sess single-engine claim/release.  The vb2
	 * queues use the PER-SESSION sess->lock (m2m_queue_init), so two
	 * concurrent opens run vdec_start_streaming() under different locks;
	 * without this, the cur_sess "is the engine free?" check and the
	 * claim are not atomic across sessions -- both pass, both power on the
	 * single VDEC_1/HEVC engine, and the double-init wedges it (BUG-CONC-DEC).
	 */
	spinlock_t cur_sess_lock;
	int irq;			/* vdec IRQ number for synchronize_irq() */
	struct mutex lock;

	/* Diagnostic hot-reload test bench (vdec_diag.c).  NULL when
	 * diag=0; opaque struct vdec_diag * when diag=1. */
	void *diag;

	/* Thermal throttling */
	struct thermal_cooling_device *cooling;
	unsigned long thermal_state;	/* 0=max, higher=slower */

	/* HCODEC encoder */
	struct clk *hcodec_clk;
	struct regmap *regmap_hhi;	/* HHI syscon for direct clock control */
	struct video_device *vdev_enc;
	struct v4l2_m2m_dev *m2m_enc;
	/*
	 * @enc_ctx: the single live encoder session, or NULL.  HCODEC is
	 * single-instance (KI-028), so one pointer is the whole population.
	 * Published and cleared under @sessions_lock -- the same lock
	 * vdec_remove()'s quiesce holds -- so an unbind either sees the
	 * session and quiesces it, or the open has not published yet and
	 * @unbound makes it fail with -ENODEV.  Without this the encoder had
	 * no teardown at all across an unbind: amvenc_open() took no core
	 * reference and no @unbound check, so devres freed struct amvdec_core
	 * out from under a still-open encoder fd (N104).
	 */
	struct amvenc_ctx *enc_ctx;
	struct mutex enc_lock;		/* encoder video device lock */
	struct mutex hw_lock;		/* serialize HCODEC HW access */
	atomic_t enc_in_use;		/* single-instance enforcement (KI-028) */
	wait_queue_head_t enc_wq;	/* wait for a transient release to clear
					 * enc_in_use before -EBUSY (BUG-CONC-MODEC-GXL) */
	struct completion enc_done;
	struct completion enc_run_done;	/* device_run finished (KI-025) */
	atomic_t enc_aborting;		/* signal device_run to abort (KI-025) */
	u32 enc_status;
	bool enc_need_reset;
	bool hcodec_powered;		/* true while HCODEC domain is on */
	struct delayed_work hcodec_poweroff_work; /* idle autosuspend */
	bool vdec1_powered;		/* true while VDEC_1 domain is on */
	int vdec1_power_users;		/* KI-118: refcount of VDEC_1 power holders (decode sessions + the HCODEC cross-engine pin); guarded by hw_lock */
	struct delayed_work vdec1_poweroff_work; /* VDEC_1 idle autosuspend */
	bool hcodec_holds_vdec1;	/* KI-118: true while HCODEC pins VDEC_1 powered (block manager) */
	bool vdec1_enc_owned;		/* KI-118: VDEC_1 cold-powered by the HCODEC pin, not a decode -- so it lacks the decode's power-on reset; a decode must wait for a clean power-cycle.  Guarded by hw_lock; cleared on poweroff. */
	wait_queue_head_t vdec1_dec_wq;	/* KI-118: decodes blocked behind an encoder-owned VDEC_1 wait here for the encoder to release + the domain to power-cycle */
	bool hevc_powered;		/* KI-118 block manager: true while the HEVC ENGINE is OWNED by an HEVC decode session or the HCODEC cross-engine pin.  N102: this is refcount OWNERSHIP, not the domain's power state -- on G12A+ vdec_1_poweron() and codec_h264_start()'s MMU gate both bring the shared HEVC domain up without setting it.  It gates vdec_hevc_power_get/_put's refcount and vdec_1_poweron's pwr_hevc / hevc_warm_idle decisions, so it must keep meaning ownership.  For "may I do MMIO into the HEVC block right now", read hevc_domain_on below. */
	bool hevc_domain_on;		/* N102: true while the HEVC DOS block is powered, un-isolated AND clocked -- i.e. an MMIO read of an HEVC-block register is safe (KI-161).  Written under hw_lock by EVERY path that changes that hardware state: vdec_hevc_power_on/_off, vdec_1_poweron/_poweroff (G12A+ shared domain), and codec_h264's MMU gate.  Distinct from hevc_powered (ownership) and from hevc_clk_ref (clocks alone: vdec_hevc_power_off isolates the domain while VDEC_1 may still hold a clock ref, and vdec_diag takes a clock ref without powering anything). */
	struct delayed_work hevc_poweroff_work; /* HEVC idle autosuspend */
	int hevc_power_users;		/* KI-118: refcount of HEVC-engine power holders (HEVC decode + the HCODEC cross-engine pin); guarded by hw_lock */
	bool hcodec_holds_hevc;		/* KI-118: true while HCODEC pins the HEVC engine powered (block manager) */
	bool hevc_enc_owned;		/* KI-118: HEVC engine cold-powered by the HCODEC pin, not a decode -- lacks the decode power-on reset; an HEVC decode must wait for a clean power-cycle.  Guarded by hw_lock; cleared on poweroff. */
	wait_queue_head_t hevc_dec_wq;	/* KI-118: HEVC decodes blocked behind an encoder-owned HEVC engine wait here for the encoder to release + the domain to power-cycle */

	/* Debugfs diagnostic interface (/sys/kernel/debug/meson-vdec/) */
	struct vdec_debugfs debugfs;

	/*
	 * HEVC-engine (HEVC + VP9) CMA-feasible resolution cap.  Computed
	 * at probe from totalcma_pages.  HEVC-engine formats advertise
	 * max_width/max_height = silicon max (8192x4320 on SM1); actual CMA
	 * determines what board-size fits.  hevc_engine_cma_max_pixels is
	 * the product cap enforced in enum_framesizes + try_fmt.  Zero =
	 * uncapped (fall back to compile-time max).
	 *
	 * PARTIAL RESOLUTION: this clamp exists because VDEC buffers are
	 * backed by contiguous CMA.  Once IOMMU / scatter-gather lands
	 * (cma-opt-sg in vp9/todo.md), FBC bodies can be backed by 4K-page
	 * SG lists via the HEVC MMU and contiguous-CMA is no longer the
	 * binding constraint.  At that point remove this cap or raise it
	 * to the silicon max.
	 */
	u64 hevc_engine_cma_max_pixels;

	/*
	 * MJPEG CMA-feasible resolution cap.  Computed at probe from the
	 * board CMA estimate, same proxy as hevc_engine_cma_max_pixels
	 * (totalcma_pages is not exported to modules).  MJPEG advertises
	 * max_width/max_height = the firmware HW max (8192x2048); this is
	 * the product cap (in pixels) that bounds the advertised + accepted
	 * resolution to what one I420 capture set (min_buffers frames) plus
	 * the VIFIFO actually fits in CMA.  Zero = uncapped (fall back to
	 * the compile-time HW max).  Enforced in enum_framesizes + try_fmt
	 * via vdec_cma_clamp_mjpeg().  MJPEG has no FBC; its footprint is
	 * planar I420 capture only, so the per-pixel budget is much smaller
	 * than the HEVC-engine FBC case.
	 */
	u64 mjpeg_cma_max_pixels;

	/*
	 * HCODEC/JPEG encoder CMA-feasible resolution cap (KI-124).
	 * Computed at probe from the same board CMA estimate as the
	 * decoder caps above (totalcma_pages is not exported to modules).
	 * The encoder's V4L2 vb2 OUTPUT plane is a single contiguous
	 * dma_contig allocation whose size scales with resolution
	 * (sizeimage = bytesperline * ALIGN(height,16) * 3/2 for the
	 * single-plane source formats), and the H.264 internal DCT
	 * sub-buffer is a single contiguous dma_alloc_coherent of
	 * mb_w*mb_h*1024 = 4 B/px.  At large resolutions (the uncapped
	 * JPEG path reaches 16368x8192) one of these single chunks
	 * exceeds what a fragmented small-CMA pool (256 MB S805X) can
	 * give, tripping WARN_ON in __alloc_pages_noprof (taint bit 9).
	 * This is the product cap (in pixels) enforced in
	 * amvenc_enum_framesizes + amvenc_try_fmt_out so REQBUFS / the
	 * start_streaming sub-buffer allocs are bounded to what the pool
	 * can satisfy.  Zero = uncapped (fall back to the compile-time HW
	 * max HCODEC_MAX_WIDTH x HCODEC_MAX_HEIGHT).  Budgeted at the
	 * largest single contiguous chunk per pixel (see amvenc_cma_clamp).
	 */
	u64 enc_cma_max_pixels;

	/*
	 * Woken when cur_sess is released after a session's hardware teardown
	 * has completed (KI-148).  vdec_start_streaming() waits here, bounded,
	 * so a sequential client that re-opens the device faster than the
	 * outgoing session's deferred teardown completes hands off cleanly
	 * instead of claiming an engine that is about to be powered down.
	 *
	 * Deliberately the LAST member: the codec modules are built against
	 * this header and are reloaded independently of meson-vdec-core (and
	 * on boards with wave420l the core may not unload at all), so a new
	 * field must not move the offset of any existing one.
	 */
	wait_queue_head_t cur_sess_wq;
};

/**
 * struct amvdec_ops - vdec operations
 *
 * @start: mandatory call when the vdec needs to initialize
 * @stop: mandatory call when the vdec needs to stop
 * @conf_esparser: mandatory call to let the vdec configure the ESPARSER
 * @vififo_level: mandatory call to get the current amount of data
 *		  in the VIFIFO
 */
struct amvdec_ops {
	int (*start)(struct amvdec_session *sess);
	int (*stop)(struct amvdec_session *sess);
	void (*conf_esparser)(struct amvdec_session *sess);
	u32 (*vififo_level)(struct amvdec_session *sess);
};

/**
 * struct amvdec_codec_ops - codec operations
 *
 * @start: mandatory call when the codec needs to initialize
 * @post_start: optional call after firmware is running (MPSR=1 + settle)
 * @stop: mandatory call when the codec needs to stop
 * @load_extended_firmware: optional call to load additional firmware bits
 * @num_pending_bufs: optional call to get the number of dst buffers on hold
 * @can_recycle: optional call to know if the codec is ready to recycle
 *		 a dst buffer
 * @recycle: optional call to tell the codec to recycle a dst buffer. Must go
 *	     in pair with @can_recycle
 * @drain: optional call if the codec has a custom way of draining
 * @resume: optional call to resume after a resolution change
 * @eos_sequence: optional call to get an end sequence to send to esparser
 *		  for flush. Mutually exclusive with @drain.
 * @setup_buffers: optional call to set up capture buffers (canvases, etc.)
 *		   before the first frame is decoded. Called once after
 *		   STATUS_RUNNING is entered. Stateless codecs use this.
 * @set_dst_buf: optional call to tell firmware which dst buffer to write to.
 *		 Called per frame for stateless codecs.
 * @isr: mandatory call when the ISR triggers
 * @threaded_isr: mandatory call for the threaded ISR
 * @early_irq_enable: if true, MBOX1 IRQ is enabled BEFORE firmware start
 *   (MPSR=1) rather than after.  Use only for codecs whose firmware does
 *   NOT fire spurious MBOX1 interrupts during init.  MJPEG is safe:
 *   firmware busy-waits for MREG_TO_AMRISC before decoding.  MPEG12 is
 *   NOT safe (fires MCPU_INTR_REQ=4 at addr 0x052f when VIFIFO has data).
 */
struct amvdec_codec_ops {
	int (*start)(struct amvdec_session *sess);
	void (*post_start)(struct amvdec_session *sess);
	int (*stop)(struct amvdec_session *sess);
	int (*load_extended_firmware)(struct amvdec_session *sess,
				      const u8 *data, u32 len, u32 mc_size);
	u32 (*num_pending_bufs)(struct amvdec_session *sess);
	int (*can_recycle)(struct amvdec_core *core);
	void (*recycle)(struct amvdec_core *core, u32 buf_idx);
	void (*drain)(struct amvdec_session *sess);
	/*
	 * Optional.  Called by vdec_drain_work() after all post-drain waits
	 * (esparser counter, vdec_wait_inactive, flush_pending_frames) and
	 * BEFORE the LAST buffer is delivered.  Intended for codecs whose
	 * firmware does not self-halt after the last frame is consumed
	 * (HEVC-engine VIFIFO-search loops on gxl_vp9_mmu / gxl_hevc_mmu).
	 *
	 * A drain_finalize implementation should (a) halt the firmware to
	 * end the post-EOS CPU-waste / spurious-IRQ behavior and (b) close
	 * the esparser_queued_bufs protocol (atomic_set to 0) now that no
	 * more ISRs can decrement it.  After drain_finalize returns, the
	 * firmware must be in a state where the session teardown
	 * (codec_ops->stop at STREAMOFF) is a no-op or idempotent.
	 *
	 * Leaving this NULL preserves the legacy "firmware runs until
	 * STREAMOFF" behavior.  See KI-VDEC-ARCH-001.
	 */
	void (*drain_finalize)(struct amvdec_session *sess);
	void (*resume)(struct amvdec_session *sess);
	const u8 * (*eos_sequence)(u32 *len);
	int (*pre_queue)(struct amvdec_session *sess, struct vb2_buffer *vb);
	int (*setup_buffers)(struct amvdec_session *sess);
	void (*set_dst_buf)(struct amvdec_session *sess, u32 fw_idx);
	irqreturn_t (*isr)(struct amvdec_session *sess);
	irqreturn_t (*threaded_isr)(struct amvdec_session *sess);
	/*
	 * KI-115 tail: optional.  Called from the esparser feed path after
	 * pushing source into the VIFIFO.  Lets a codec resume a firmware it
	 * left PAUSED on a VIFIFO-underrun BUFEMPTY -- instead of acking in the
	 * hardirq (which re-arms the firmware to re-post BUFEMPTY and
	 * read-hammer the cold SM1 DOS bus until it wedges).  NULL = legacy.
	 */
	void (*resume_after_feed)(struct amvdec_session *sess);
	/*
	 * Optional: the per-device_run feed window -- the max in-flight pictures
	 * the codec can accept.  The MPEG2 contiguous feed
	 * (vdec_mpeg2_feed_contiguous) gates `in_flight < feed_window`.  Default
	 * (NULL) = free V4L2 CAPTURE buffers (vdec_stateless_free_dst), correct for
	 * the zero-copy bridge where a V4L2 buffer IS a firmware canvas.  The
	 * NON-ZERO-COPY mpeg12_libre path decodes into a decoupled scratch-canvas
	 * pool, so it returns the scratch pool size instead -- gating the feed on
	 * free V4L2 buffers there collapses the window when the consumer lags and
	 * deadlocks the firmware (it never gets pic N+1's start code to terminate
	 * pic N).  See analysis.md 2026-06-24.
	 */
	u32 (*feed_window)(struct amvdec_session *sess);
	/*
	 * Optional: the arm-on-available wake.  Called from
	 * vdec_vb2_buf_queue() for a CAPTURE QBUF; a codec that deferred
	 * arming its parsed frame because the m2m dst list was empty
	 * schedules the re-arm from here.  Must not touch the engine.
	 * NULL = the codec never defers.
	 */
	void (*arm_pending)(struct amvdec_session *sess);
	bool early_irq_enable;
};

/**
 * struct amvdec_format - describes one of the OUTPUT (src) format supported
 *
 * @pixfmt: V4L2 pixel format
 * @min_buffers: minimum amount of CAPTURE (dst) buffers
 * @max_buffers: maximum amount of CAPTURE (dst) buffers
 * @min_width: minimum picture width supported (0 = use default 256)
 * @max_width: maximum picture width supported
 * @max_height: maximum picture height supported
 * @flags: enum flags associated with this pixfmt
 * @vdec_ops: the VDEC operations that support this format
 * @codec_ops: the codec operations that support this format
 * @firmware_path: Path to the firmware that supports this format
 * @firmware_tag: Identifies firmware binary for skip-reload optimization.
 *   0 = always reload (stateful, one firmware per codec).
 *   Non-zero = skip IMEM reload if tag matches what is already loaded
 *   (libre stateless multi-codec firmware: one binary per engine).
 * @pixfmts_cap: list of CAPTURE pixel formats available with pixfmt
 */
struct amvdec_format {
	u32 pixfmt;
	u32 min_buffers;
	u32 max_buffers;
	u32 min_width;
	u32 max_width;
	u32 max_height;
	u32 max_mb_total;	/* firmware MB budget limit (0 = no limit); checked in
				 * pre_queue from SPS-parsed dims. Replaces per-dimension
				 * firmware limits where constraint is total MBs not W or H. */
	u32 flags;

	struct amvdec_ops *vdec_ops;
	struct amvdec_codec_ops *codec_ops;

	char *firmware_path;
	u32 firmware_tag;
	u32 pixfmts_cap[8];

	/* Firmware version identification table (per-codec, optional) */
	const struct amvdec_fw_version *fw_versions;
	size_t fw_versions_count;

	/*
	 * Emit a SOURCE_CHANGE event at OUTPUT STREAMON from the coded
	 * resolution (sess->width/height, set by S_FMT) instead of from the
	 * first decode ISR.  Lets a non-DYN_RESOLUTION codec defer firmware
	 * poweron to CAPTURE STREAMON (BSP continuous-decode model) while
	 * still giving userspace its CAPTURE-allocation trigger.  Used by
	 * MPEG-4 (continuous single-instance firmware).
	 */
	bool early_src_change;

	/*
	 * V4L2_CID_MIN_BUFFERS_FOR_CAPTURE reported before the firmware has
	 * parsed the stream header (0 = 1).  A client that sets CAPTURE up
	 * before the first SOURCE_CHANGE (GStreamer 1.22 does) sizes its
	 * pool from this value and never reallocates on a SOURCE_CHANGE
	 * whose resolution is unchanged, so it must already cover the
	 * codec's largest DPB (KI-171).  Clients that wait for the event
	 * read the real DPB that amvdec_src_change() sets.
	 */
	u32 prehdr_min_capture;
};

enum amvdec_status {
	STATUS_STOPPED,
	STATUS_NEEDS_START, /* VIFIFO allocated, waiting for capture buffers */
	STATUS_INIT,
	STATUS_RUNNING,
	STATUS_NEEDS_RESUME,
};

/**
 * struct amvdec_session - decoding session parameters
 *
 * @core: reference to the vdec core struct
 * @fh: v4l2 file handle
 * @m2m_dev: v4l2 m2m device
 * @m2m_ctx: v4l2 m2m context
 * @ctrl_handler: V4L2 control handler
 * @ctrl_min_buf_capture: V4L2 control V4L2_CID_MIN_BUFFERS_FOR_CAPTURE
 * @lock: cap & out queues lock
 * @fmt_out: vdec pixel format for the OUTPUT queue
 * @pixfmt_cap: V4L2 pixel format for the CAPTURE queue
 * @src_buffer_size: size in bytes of the OUTPUT buffers' only plane
 * @width: current picture width
 * @height: current picture height
 * @colorspace: current colorspace
 * @ycbcr_enc: current ycbcr_enc
 * @quantization: current quantization
 * @xfer_func: current transfer function
 * @pixelaspect: Pixel Aspect Ratio reported by the decoder
 * @esparser_queued_bufs: number of buffers currently queued into ESPARSER
 * @esparser_queue_work: work struct for the ESPARSER to process src buffers
 * @streamon_cap: stream on flag for capture queue
 * @streamon_out: stream on flag for output queue
 * @sequence_cap: capture sequence counter
 * @sequence_out: output sequence counter
 * @should_stop: flag set if userspace signaled EOS via command
 *		 or empty buffer
 * @firmware_halted: set by codec_ops->drain_finalize after it halts the
 *		 firmware via amvdec_hevc_halt_firmware() (HEVC engine) or
 *		 amvdec_vdec1_halt_firmware() (VDEC_1, H.264).  Cleared on
 *		 firmware restart (vdec_resume_halted_firmware(): at
 *		 V4L2_DEC_CMD_START for the HEVC engine, on the first new
 *		 input for VDEC_1) or implicitly at fresh STREAMON.  See
 *		 KI-VDEC-ARCH-001 and KI-152.
 * @aborted: session abort flag (unsupported stream); firmware halted
 * @keyframe_found: flag set once a keyframe has been parsed
 * @num_dst_bufs: number of destination buffers
 * @changed_format: the format changed
 * @canvas_alloc: array of all the canvas IDs allocated
 * @canvas_num: number of canvas IDs allocated
 * @vififo_vaddr: virtual address for the VIFIFO
 * @vififo_paddr: physical address for the VIFIFO
 * @vififo_size: size of the VIFIFO dma alloc
 * @bufs_recycle: list of buffers that need to be recycled
 * @bufs_recycle_lock: lock for the bufs_recycle list
 * @recycle_thread: task struct for the recycling thread
 * @timestamps: chronological list of src timestamps
 * @ts_spinlock: spinlock for the timestamps list
 * @last_irq_jiffies: tracks last time the vdec triggered an IRQ
 * @last_offset: tracks last offset of vififo
 * @wrap_count: number of times the vififo wrapped around
 * @fw_idx_to_vb2_idx: firmware buffer index to vb2 buffer index
 * @status: current decoding status
 * @priv: codec private data
 */

struct amvdec_pending_frame {
	struct list_head list;
	u32 buf_idx;
	u32 offset;
	u32 field;
};

struct amvdec_session {
	struct amvdec_core *core;

	/* Membership of core->sessions, guarded by core->sessions_lock. */
	struct list_head core_entry;

	struct v4l2_fh fh;
	struct v4l2_m2m_dev *m2m_dev;
	struct v4l2_m2m_ctx *m2m_ctx;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *ctrl_min_buf_capture;
	struct v4l2_ctrl *ctrl_i_only;
	struct mutex lock;

	const struct amvdec_format *fmt_out;
	u32 pixfmt_cap;
	u32 src_buffer_size;

	u32 width;
	u32 height;
	u32 colorspace;
	u8 ycbcr_enc;
	u8 quantization;
	u8 xfer_func;

	struct v4l2_fract pixelaspect;

	atomic_t esparser_queued_bufs;
	wait_queue_head_t esparser_queue_done_wq;
	struct work_struct esparser_queue_work;

	unsigned int streamon_cap, streamon_out;
	unsigned int sequence_cap, sequence_out;
	unsigned int should_stop;
	/* DEC_CMD_STOP's EOS sequence waits for the queued src (KI-172) */
	bool eos_pending;
	unsigned int firmware_halted;
	unsigned int aborted;
	/* Per-session arm-on-available counts: CAPTURE QBUFs made by the
	 * client vs frames delivered to it (logged at CAPTURE stop).
	 */
	unsigned int cap_qbufs;
	unsigned int dst_delivered;
	/*
	 * KI-115 tail: set in codec_hevc_isr when the firmware posts a
	 * VIFIFO-underrun BUFEMPTY.  The firmware is left PAUSED (un-acked) and
	 * resumed by codec_ops->resume_after_feed once source is re-fed -- so
	 * the hardirq does not re-arm the spin that read-hammers the cold bus.
	 */
	bool bufempty_pending;
	unsigned int keyframe_found;
	unsigned int num_dst_bufs;
	unsigned int changed_format;

	u8 canvas_alloc[MAX_CANVAS];
	u32 canvas_num;

	void *vififo_vaddr;
	dma_addr_t vififo_paddr;
	u32 vififo_size;

	struct list_head bufs_recycle;
	struct mutex bufs_recycle_lock; /* bufs_recycle list lock */
	struct task_struct *recycle_thread;
	wait_queue_head_t recycle_wq; /* wakes recycle_thread on QBUF */

	/* Frames decoded by firmware but not yet delivered because all
	 * capture buffers were in userspace.  Retried by recycle_thread.
	 */
	struct list_head pending_frames; /* struct amvdec_pending_frame */
	spinlock_t pending_frames_lock;

	struct list_head timestamps;
	spinlock_t ts_spinlock; /* timestamp list lock */

	u64 last_irq_jiffies;
	u64 decode_start_jiffies; /* watchdog baseline before the first IRQ (stateless first-decode hang) */
	u64 last_ts_done; /* last timestamp successfully delivered, for fallback */
	u32 last_offset;
	u32 wrap_count;
	u32 fw_idx_to_vb2_idx[32];

	/*
	 * Stateless: count of frames fed via device_run this session. Used to
	 * re-arm the VLD start-code search only for frame N>0 (the gxl_mpeg12
	 * bridge GOP continuation kick -- see vdec_m2m_device_run), avoiding any
	 * perturbation of frame 0's in-flight decode. Reset when the firmware
	 * (re)starts (STATUS_STOPPED -> RUNNING in device_run).
	 */
	u32 stateless_frames_fed;

	/*
	 * Stateless MPEG2_SLICE: count of pictures DELIVERED to userspace (bumped
	 * in the codec threaded ISR per BUFFEROUT). (stateless_frames_fed -
	 * stateless_frames_done) is the in-flight depth, used by the contiguous
	 * feed to pace: feed while a free CAPTURE buffer exists beyond the
	 * in-flight set, so each picture's terminating next-start-code is resident
	 * without saturating the firmware busy mask. Reset with frames_fed.
	 */
	u32 stateless_frames_done;

	struct work_struct drain_work;

	enum amvdec_status status;
	atomic_t resumed; /* guards against double resume() calls */
	u32 fw_size; /* firmware size in bytes, set by vdec_hevc_load_firmware */

	/* Matched firmware version entry, set by vdec_1_load_firmware */
	const struct amvdec_fw_version *fw_match;
	unsigned int fast_output_enable; /* DPB output mode bitmask (bit3=IP-only) */
	void *priv;

	/*
	 * Decode stall timeout in ms (0 = disabled).
	 * Host-side timer checks firmware macroblock progress.
	 * BSP vmh264.c default: 100.  Exposed via
	 * V4L2_CID_MESON_VDEC_DECODE_TIMEOUT.
	 */
	u32 decode_timeout_ms;

	/*
	 * Framework-level decode watchdog. Fires every 200ms; if
	 * last_irq_jiffies hasn't advanced in decode_timeout_ms, aborts.
	 * Works for ALL codecs (H.264 has an additional per-MB watchdog
	 * in codec_h264.c that catches MBY_MBX stalls within a frame).
	 */
	struct timer_list decode_watchdog;

	/*
	 * Per-session no-IRQ watchdog timeout in ms (vdec_watchdog_cb).
	 * Default WATCHDOG_TIMEOUT_MS (5000), sized for DPB fill on the
	 * slowest platform (H.264/HEVC).  MJPEG overrides it at stream
	 * start with a resolution-shaped value, since single-frame MJPEG
	 * decode is far faster than a flat 5s.
	 */
	unsigned int watchdog_timeout_ms;

	/**
	 * @error_proc_policy: 32-bit bitmask controlling error handling.
	 *
	 * BSP vmh264.c default: 0x7fcff6.  Exposed via
	 * V4L2_CID_MESON_VDEC_ERROR_POLICY.
	 */
	u32 error_proc_policy;
	unsigned int max_alloc_buf_count;      /* V4L2 ctrl: 0 = unlimited */
	unsigned int reorder_dpb_size_margin;  /* V4L2 ctrl: default 7 */

	/* True if this session was opened on the MJPEG device node. */
	bool jpeg_node;
};

static inline struct amvdec_session *file_to_amvdec_session(struct file *filp)
{
	return container_of(file_to_v4l2_fh(filp), struct amvdec_session, fh);
}

u32 amvdec_get_output_size(struct amvdec_session *sess);
bool amvdec_cap_bufs_fit(struct amvdec_session *sess, u32 width, u32 height);

/**
 * amvdec_register_codec() - Register a codec's ops for a given pixel format.
 *
 * Scans all platform format tables and fills in codec_ops for entries whose
 * pixfmt matches. Called from per-codec module_init().
 *
 * @pixfmt: V4L2 pixel format (e.g. V4L2_PIX_FMT_H264)
 * @ops: codec operations struct
 * Returns 0 on success, -ENOENT if no format entry matches.
 */
int amvdec_register_codec(u32 pixfmt, struct amvdec_codec_ops *ops);

/**
 * amvdec_unregister_codec() - Unregister a codec's ops.
 *
 * NULLs out codec_ops for all format entries matching pixfmt + ops.
 * Refuses if a session is actively using this codec (returns -EBUSY).
 *
 * @pixfmt: V4L2 pixel format
 * @ops: codec operations struct (must match what was registered)
 * Returns 0 on success, -EBUSY if codec is in active use.
 */
int amvdec_unregister_codec(u32 pixfmt, struct amvdec_codec_ops *ops);

/**
 * amvdec_clock_rate() - get the target VDEC clock rate respecting thermal cap
 * @core: the amvdec_core instance
 *
 * Returns the maximum clock rate (Hz) that vdec_1_start / vdec_hevc_start
 * should request.  Normally 800 MHz; reduced when the thermal framework
 * throttles the VDEC cooling device.
 */
unsigned long amvdec_clock_rate(struct amvdec_core *core);

/**
 * vdec_core_get() / vdec_core_put() - reference struct amvdec_core.
 *
 * Exposed for venc.c: an encoder file handle outlives an unbind exactly the
 * way a decoder one does, and dereferences core-owned state (m2m_enc, the
 * v4l2_device) from its ->release.  See the @ref comment on struct
 * amvdec_core.
 */
void vdec_core_get(struct amvdec_core *core);
void vdec_core_put(struct amvdec_core *core);

#endif
