// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2020 BayLibre, SAS
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/bitfield.h>
#include <linux/dma-buf.h>
#include <linux/interrupt.h>
#include <linux/math.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/reset.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>
#include <linux/regmap.h>
#include <linux/soc/amlogic/meson-canvas.h>

#include <linux/pm_runtime.h>
#include <linux/thermal.h>
#include <linux/platform_device.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/v4l2-ctrls.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>

#include <linux/completion.h>
#include <linux/of_platform.h>
#include <linux/soc/amlogic/meson-ge2d.h>

#include "ge2d-regs.h"

static unsigned int ge2d_settle_us;
module_param(ge2d_settle_us, uint, 0644);
MODULE_PARM_DESC(ge2d_settle_us,
	"Reset settle delay in microseconds (default 0)");

/*
 * SRC2 compositing ioctl (F-04).
 *
 * Custom ioctl to specify a second source buffer for hardware compositing.
 * When SRC2 is set, the GE2D ALU blends SRC1 and SRC2 per-pixel and writes
 * the result to DST.  The blend operation is defined by blend_op (packed
 * ALU factors matching the BSP blendop() layout).
 */
#define MESON_GE2D_SRC2_FLAG_FILL_COLOR		BIT(0)
#define MESON_GE2D_SRC2_FLAG_SRC2_X_REV	BIT(1)
#define MESON_GE2D_SRC2_FLAG_SRC2_Y_REV	BIT(2)
#define MESON_GE2D_SRC2_FLAG_SRC1_PREMULTIPLIED	BIT(3)

struct meson_ge2d_src2_info {
	__s32 fd;		/* DMABUF fd, -1 disable, -2 keep current */
	__u32 pixelformat;	/* V4L2 fourcc (packed single-plane only) */
	__u32 width;		/* SRC2 surface width in pixels */
	__u32 height;		/* SRC2 surface height in pixels */
	__u32 bytesperline;	/* SRC2 stride in bytes */
	__s32 x_offset;		/* SRC2 read start X within the surface */
	__s32 y_offset;		/* SRC2 read start Y within the surface */
	__u32 blend_op;		/* packed ALU factors */
	__u32 src2_gb_alpha;	/* SRC2 global alpha 0-255 (G12B+, 0=off) */
	__u32 const_color;	/* ALU constant color ARGB8888 */
	__u32 def_color;	/* SRC2 default fill color ARGB8888 */
	__u32 color_key;	/* SRC2 color key value (0 = disabled) */
	__u32 color_key_mask;	/* SRC2 color key mask */
	__u32 repeat_x;		/* SRC2 X pixel repeat: 0=1x,1=2x,2=4x,3=8x */
	__u32 repeat_y;		/* SRC2 Y pixel repeat: 0=1x,1=2x,2=4x,3=8x */
	__u32 flags;		/* MESON_GE2D_SRC2_FLAG_* */
	__u32 reserved[2];	/* must be zero */
};

#define VIDIOC_MESON_GE2D_SET_SRC2 \
	_IOW('V', BASE_VIDIOC_PRIVATE + 0, struct meson_ge2d_src2_info)
#define VIDIOC_MESON_GE2D_GET_SRC2 \
	_IOR('V', BASE_VIDIOC_PRIVATE + 1, struct meson_ge2d_src2_info)

/*
 * CLUT8 palette ioctl (F-10).
 *
 * Uploads a 256-entry palette used when SRC1 format is V4L2_PIX_FMT_PAL8.
 * The hardware reads each SRC1 byte as an index, looks up the 32-bit
 * palette entry, and feeds it into the normal SRC1 path (CSC, scaler,
 * ALU).  The palette persists per file-handle across blits until
 * replaced or until the handle is closed.  Only SRC1 uses the LUT;
 * SRC2 has no equivalent.
 *
 * Palette entry byte layout (hardware-defined, probed on G12B
 * 2026-04-24):
 *
 *   bits [31:24] -> output B (blue)
 *   bits [23:16] -> output G (green)
 *   bits [15:8]  -> output R (red)
 *   bits [7:0]   -> ignored (no alpha representation in the LUT)
 *
 * The LUT pipeline has no alpha channel -- output alpha / unused-X is
 * fixed to 0xFF regardless of palette contents.  The LSB of each entry
 * is ignored; userspace should populate bits [31:8] of data[i] with a
 * 24-bit BGR color (blue in the MSB) and leave bits [7:0] zero.
 *
 * The palette is engaged automatically whenever SRC1 format is PAL8.
 * SET_CLUT8 must be issued before STREAMON on the OUTPUT queue or the
 * first PAL8 blit will use undefined palette contents.
 */
struct meson_ge2d_clut8_info {
	/*
	 * Palette entries.  Each is a 32-bit word; the hardware uses only
	 * bits [31:8]: [31:24]=B, [23:16]=G, [15:8]=R, [7:0] ignored.
	 * Output alpha is fixed to 0xFF by the LUT pipeline.
	 */
	__u32 data[256];
	__u32 count;		/* number of valid entries, 1-256 */
	__u32 flags;		/* reserved, must be 0 */
	__u32 reserved[3];	/* reserved, must be 0 */
};

#define VIDIOC_MESON_GE2D_SET_CLUT8 \
	_IOW('V', BASE_VIDIOC_PRIVATE + 2, struct meson_ge2d_clut8_info)

#define GE2D_NAME	"meson-ge2d"

#define DEFAULT_WIDTH	128
#define DEFAULT_HEIGHT	128
#define DEFAULT_STRIDE	512

#define MAX_WIDTH	8192
#define MAX_HEIGHT	8192

/* meson-canvas stride field is 12-bit: max (stride+7)/8 = 4095, max stride = 32760 */
#define MAX_STRIDE	32760

/*
 * Custom V4L2 control: GE2D ALU logic operation mode.
 *
 * Selects the bitwise logic operation applied when performing a blit
 * with no alpha blending (alpha == 255).  When alpha < 255, this
 * control is ignored and Porter-Duff source-over blending is used.
 *
 * Control ID: V4L2_CID_USER_MESON_GE2D_BASE + 0
 * Type: menu
 * Values: see GE2D_LOGIC_OP_* enum below.
 *
 * Default: GE2D_LOGIC_OP_COPY (standard copy blit, hardware default).
 *
 * Note: CLEAR (0x00) and SET (0xFF) do not use the source buffer data
 * and produce a uniform output regardless of source pixels.  XOR
 * produces the bitwise XOR of source and destination.  AND/OR operate
 * bitwise between source and destination pixels.
 */
#define V4L2_CID_USER_MESON_GE2D_BASE		(V4L2_CID_USER_BASE + 0x1080)
#define V4L2_CID_USER_MESON_GE2D_LOGIC_OP	(V4L2_CID_USER_MESON_GE2D_BASE + 0)
#define V4L2_CID_USER_MESON_GE2D_COLORKEY	(V4L2_CID_USER_MESON_GE2D_BASE + 1)
#define V4L2_CID_USER_MESON_GE2D_COLORKEY_MASK	(V4L2_CID_USER_MESON_GE2D_BASE + 2)
#define V4L2_CID_USER_MESON_GE2D_FILLCOLOR	(V4L2_CID_USER_MESON_GE2D_BASE + 3)
#define V4L2_CID_USER_MESON_GE2D_SCALE_FILTER	(V4L2_CID_USER_MESON_GE2D_BASE + 4)
#define V4L2_CID_USER_MESON_GE2D_BITMASK	(V4L2_CID_USER_MESON_GE2D_BASE + 5)
#define V4L2_CID_USER_MESON_GE2D_ANTIFLICKER	(V4L2_CID_USER_MESON_GE2D_BASE + 6)

enum ge2d_logic_op_menu {
	GE2D_LOGIC_OP_COPY  = 0,	/* LOGIC_OPERATION_COPY  -- src copy (default) */
	GE2D_LOGIC_OP_CLEAR = 1,	/* LOGIC_OPERATION_CLEAR -- output zero */
	GE2D_LOGIC_OP_SET   = 2,	/* LOGIC_OPERATION_SET   -- output 0xFF */
	GE2D_LOGIC_OP_XOR   = 3,	/* LOGIC_OPERATION_XOR   -- src ^ dst */
	GE2D_LOGIC_OP_AND   = 4,	/* LOGIC_OPERATION_AND   -- src & dst */
	GE2D_LOGIC_OP_OR    = 5,	/* LOGIC_OPERATION_OR    -- src | dst */
};

/* Map from GE2D_LOGIC_OP_* menu index to hardware LOGIC_OPERATION_* value */
static const u8 ge2d_logic_op_hw[] = {
	[GE2D_LOGIC_OP_COPY]  = LOGIC_OPERATION_COPY,
	[GE2D_LOGIC_OP_CLEAR] = LOGIC_OPERATION_CLEAR,
	[GE2D_LOGIC_OP_SET]   = LOGIC_OPERATION_SET,
	[GE2D_LOGIC_OP_XOR]   = LOGIC_OPERATION_XOR,
	[GE2D_LOGIC_OP_AND]   = LOGIC_OPERATION_AND,
	[GE2D_LOGIC_OP_OR]    = LOGIC_OPERATION_OR,
};

static const char * const ge2d_logic_op_menu_items[] = {
	"Copy",
	"Clear",
	"Set",
	"XOR",
	"AND",
	"OR",
	NULL,
};

/*
 * Color space conversion (CSC) is supported for BT.601 and BT.709.
 * The V4L2 colorspace/ycbcr_enc fields on the source format select
 * the matrix.  ALU blending is supported via the V4L2 alpha component
 * control -- when alpha < 255, the hardware performs Porter-Duff
 * source-over blending using the global alpha value.
 * Logic op mode is exposed via V4L2_CID_USER_MESON_GE2D_LOGIC_OP and
 * applies only when alpha == 255.
 */

struct meson_ge2d_match_data {
	bool has_baddr;
	/*
	 * GE2D register base offset within the mapped region.
	 * GXL: registers at (0xa0 + x) * 4, so reg_base = 0xa0 * 4 = 0x280
	 *       minus the GE2D_REG(0) offset of 0x8a0 * 4 = 0x2280 gives -0x2000.
	 *       We add reg_base to the ioremap pointer so GE2D_REG(x) hits the
	 *       right physical address.
	 * G12A/SM1/AXG: registers at (0x8a0 + x) * 4, reg_base = 0 (no adjustment).
	 */
	int reg_base;
	/*
	 * has_src2_alp: GE2D_ALU_OP_CTRL uses the extended SRC2 alpha layout.
	 * When true (G12A/G12B/SM1): src2_cmult_asel is 2-bit [24:23] and
	 * src2_cmult_ad bit [27] is present.
	 * When false (GXL/AXG): src2_cmult_asel is 1-bit [24] only.
	 * (BSP ge2d_hw.c:1484-1486, ge2d.h:642-646)
	 */
	bool has_src2_alp;
	/*
	 * has_hang_flag: BSP sets GE2D_GEN_CTRL4 bit 0 (hang_flag) to 1 on
	 * G12A/G12B/SM1 per blit to disable the dp_out_done/scale_out_done
	 * hang fix that exists on those SoCs. GXL and AXG do not have the
	 * hang fix hardware, so they leave bit 0 = 0 (no effect).
	 * Register probe confirmed: GXL bits [15:0] live; G12B/SM1 bits [28:0] live.
	 * (BSP ge2d_hw.c: ge2d_set_gen() sets hang_flag from device data)
	 */
	bool has_hang_flag;
	/*
	 * has_deep_color: GEN_CTRL2 bit 2 switches the pixel pipeline to
	 * 10/12-bit processing. Available on GXM, G12A, G12B, SM1 and newer.
	 * Not available on GXBB or GXL. AXG has it per BSP but we have no
	 * AXG GE2D hardware to test, so leave disabled there for now.
	 */
	bool has_deep_color;
	unsigned long max_rate;	/* max VAPB clock rate (Hz) */
	unsigned long turbo_rate;	/* overclock rate (Hz), 0 = no turbo */
};

static const struct meson_ge2d_match_data ge2d_axg  = { .has_baddr = true,  .reg_base = 0,       .has_src2_alp = false, .has_hang_flag = false, .has_deep_color = false, .max_rate = 400000000, .turbo_rate = 667000000 };
static const struct meson_ge2d_match_data ge2d_gxl  = { .has_baddr = false, .reg_base = -0x2000, .has_src2_alp = false, .has_hang_flag = false, .has_deep_color = false, .max_rate = 400000000, .turbo_rate = 667000000 };
static const struct meson_ge2d_match_data ge2d_g12a = { .has_baddr = false, .reg_base = 0,       .has_src2_alp = true,  .has_hang_flag = true,  .has_deep_color = true,  .max_rate = 500000000, .turbo_rate = 667000000 };
static const struct meson_ge2d_match_data ge2d_sm1  = { .has_baddr = false, .reg_base = 0,       .has_src2_alp = true,  .has_hang_flag = true,  .has_deep_color = true,  .max_rate = 500000000, .turbo_rate = 667000000 };

struct ge2d_fmt {
	u32 fourcc;
	bool alpha;
	bool yuv;
	bool le;
	bool nv;		/* NV12/NV21/NV16/NV61 semi-planar */
	bool planar;		/* 3-plane I420/YV12/YUV422P */
	bool deep_color;	/* requires GEN_CTRL2 bit 2 (10/12-bit pipeline) */
	unsigned int depth;	/* bits per pixel (Y plane for multi-plane) */
	unsigned int hw_fmt;
	unsigned int hw_map;
	/*
	 * For semi-planar (nv): UV plane height divisor relative to Y.
	 *   1 = same height (NV16/NV61), 2 = half height (NV12/NV21).
	 * For planar: Cb/Cr plane height divisor.
	 *   1 = same height (YUV422P), 2 = half height (I420/YV12).
	 * Unused (0) for single-plane formats.
	 */
	unsigned int uv_vdiv;
	/*
	 * For planar: Cb/Cr plane width divisor relative to Y.
	 *   2 = half width (I420/YV12/YUV422P).
	 * For semi-planar: unused (chroma is interleaved at full width).
	 * Unused (0) for single-plane formats.
	 */
	unsigned int uv_hdiv;
	unsigned int num_planes;
};

struct ge2d_frame {
	struct vb2_v4l2_buffer *buf;

	/* Image Format */
	struct v4l2_pix_format_mplane pix_mp;

	/* Crop */
	struct v4l2_rect crop;

	/* Image format */
	const struct ge2d_fmt *fmt;
};

struct ge2d_ctx {
	struct v4l2_fh fh;
	struct meson_ge2d *ge2d;
	struct ge2d_frame in;
	struct ge2d_frame out;
	struct v4l2_ctrl_handler ctrl_handler;

	unsigned long sequence_out, sequence_cap;

	/* Control values */
	u32 hflip;
	u32 vflip;
	u32 xy_swap;
	u32 alpha;
	/*
	 * logic_op: ALU logic operation for LOGIC mode blits.
	 * Set via V4L2_CID_USER_MESON_GE2D_LOGIC_OP control.
	 * LOGIC_OPERATION_COPY (1) is the default (standard blit behaviour).
	 * Only applies when alpha == 255 (no blend mode active).
	 */
	u32 logic_op;

	/* Color keying: key_enable derived from colorkey_mask != 0 */
	u32 colorkey;       /* SRC1 key color value (ARGB) */
	u32 colorkey_mask;  /* SRC1 key mask (0 = disabled) */
	u32 fillcolor;      /* SRC1 fill color (0 = disabled, nonzero = fill mode) */
	u32 scale_filter;   /* scaling filter: 0=bilinear 1=bicubic 2=triangle */
	u32 bitmask;        /* destination bitmask (0 = disabled) */
	u32 antiflicker;    /* anti-flicker filter (0 = disabled, 1 = enabled) */

	/* CLUT8 palette (F-10): 256-entry ARGB8888 LUT for SRC1 PAL8 */
	u32 clut8[256];
	u32 clut8_count;
	bool clut8_present;

	/* Canvas IDs for canvas mode cleanup */
	u8 src1_canvas, src2_canvas, dst1_canvas;
	/* Extra canvases for semi-planar chroma planes */
	u8 src1_canvas_uv, dst2_canvas;
	/* Extra canvases for 3-plane (planar) Cb and Cr */
	u8 src1_canvas_cb, src1_canvas_cr;
	u8 dst_canvas_cb, dst_canvas_cr;
	bool has_src_uv_canvas;
	bool has_dst2_canvas;
	bool has_src_planar_canvas;
	bool has_dst_planar_canvas;
	/*
	 * canvases_allocated: true after the first blit allocates canvas IDs.
	 * Canvas IDs are cached across blits within one streaming session;
	 * only the DMA addresses (meson_canvas_config) are updated per blit.
	 * Freed in ge2d_stop_streaming() on the OUTPUT queue.
	 */
	bool canvases_allocated;

	/* SRC2 compositing (F-04) */
	struct dma_buf *src2_dmabuf;
	struct dma_buf_attachment *src2_attach;
	struct sg_table *src2_sgt;
	dma_addr_t src2_dma_addr;
	int src2_bound_fd;		/* fd currently imported, -1 if none */
	const struct ge2d_fmt *src2_fmt;
	u32 src2_width, src2_height, src2_bytesperline;
	s32 src2_x_offset, src2_y_offset;
	u32 blend_op;
	u32 src2_gb_alpha;
	u32 const_color;		/* GE2D_ALU_CONST_COLOR ARGB8888 */
	u32 def_color;			/* GE2D_SRC2_DEF_COLOR ARGB8888 */
	u32 src2_color_key, src2_color_key_mask;
	u32 src2_repeat_x, src2_repeat_y;
	u32 src2_flags;			/* MESON_GE2D_SRC2_FLAG_* */
	bool blend_mode;
};

/*
 * 6.18: v4l2_ioctl_ops handlers are dispatched with priv/fh == NULL
 * (v4l2-ioctl.c passes (file, NULL, ...) at every site), so the context
 * must be recovered from the struct file.  Mirrors upstream commit
 * "media: meson-ge2d: use file_to_v4l2_fh()".
 */
static inline struct ge2d_ctx *file_to_ge2d_ctx(struct file *filp)
{
	return container_of(file_to_v4l2_fh(filp), struct ge2d_ctx, fh);
}

struct meson_ge2d {
	struct v4l2_device v4l2_dev;
	struct v4l2_m2m_dev *m2m_dev;
	struct video_device *vfd;

	struct device *dev;
	struct regmap *map;
	struct clk *clk;
	struct clk *clk_gate;	/* HHI_GCLK_MPEG1 bus gate (optional) */
	struct reset_control *rst;

	/* vb2 queue lock */
	struct mutex mutex;

	struct ge2d_ctx *curr;

	/*
	 * Protects ge2d->curr against the watchdog-timeout path racing the
	 * completion ISR.  The ISR (hardirq) and the timeout work (process
	 * context) both try to claim the in-flight job by clearing ge2d->curr;
	 * whoever sets it NULL under this lock owns the completion, the loser
	 * sees NULL and bails.  Taken with spin_lock_irqsave so the ISR can
	 * acquire it.
	 */
	spinlock_t hw_lock;

	/*
	 * Set true (under hw_lock) at the start of ge2d_remove().  device_run()
	 * checks it under hw_lock before arming job_timer, so a job that runs
	 * after the remove-path timer_delete_sync (the v4l2-m2m scheduler is
	 * still live until v4l2_m2m_release) can never re-arm the timer -- which
	 * would otherwise fire from the devm-freed struct (use-after-free;
	 * captured SP/PC-alignment panic 2026-06-06, list-poison fn pointer in
	 * __run_timer_base).
	 */
	bool removing;

	/*
	 * Per-job watchdog.  Armed in device_run() after a successful
	 * ge2d_hw_start(); cancelled in ge2d_isr() when the blit completes.
	 * If it fires, the blit's completion IRQ never arrived: the timer cb
	 * schedules job_timeout_work, which resets the IP in place and errors
	 * out the stuck buffers so DQBUF returns instead of hanging forever.
	 * The heavy recovery (v4l2_m2m_*, reset_control_*, pm_runtime_*) runs
	 * from the work, not the timer softirq, because those calls can sleep.
	 */
	struct timer_list job_timer;
	struct work_struct job_timeout_work;

	/* CMA-constrained usable max resolution (<= MAX_WIDTH/HEIGHT), computed
	 * at probe from the board's CMA pool so a src/dst DMA buffer always fits.
	 */
	u32 max_width;
	u32 max_height;

	bool has_baddr;
	/*
	 * has_src2_alp: extended ALU SRC2 alpha layout (G12A/G12B/SM1+).
	 * When true, GE2D_ALU_OP_CTRL bits [27] and [24:23] are present.
	 * When false (GXL/AXG), only bit [24] exists.
	 */
	bool has_src2_alp;
	/*
	 * has_hang_flag: set GE2D_GEN_CTRL4 bit 0 to 1 per blit on G12A/G12B/SM1
	 * to disable the dp_out_done/scale_out_done hang fix. False for GXL/AXG
	 * which lack the hang fix hardware (bit 0 has no effect there).
	 */
	bool has_hang_flag;
	/*
	 * has_deep_color: GEN_CTRL2 bit 2 enables 10/12-bit pixel pipeline.
	 * G12A/G12B/SM1+. Not on GXL.
	 */
	bool has_deep_color;
	unsigned long max_rate;
	unsigned long thermal_state;
	struct thermal_cooling_device *cooling;
	unsigned long turbo_rate;
	bool turbo;
	u32 burst_ctrl4;	/* GEN_CTRL4 bits [28:16] for AXI burst config */
	struct meson_canvas *canvas;

	/* Kernel API */
	struct mutex kapi_lock;
	struct completion kapi_done;
	bool kapi_active;
};

#define FMT(_fourcc, _alpha, _depth, _map)		\
{							\
	.fourcc = _fourcc,				\
	.alpha = (_alpha),				\
	.depth = (_depth),				\
	.hw_fmt = GE2D_FORMAT_ ## _depth ## BIT,	\
	.hw_map = GE2D_COLOR_MAP_ ## _map,		\
	.num_planes = 1,				\
}

#define FMTYUV(_fourcc, _alpha, _depth, _map)		\
{							\
	.fourcc = _fourcc,				\
	.alpha = (_alpha),				\
	.yuv = true,					\
	.depth = (_depth),				\
	.hw_fmt = GE2D_FORMAT_ ## _depth ## BIT,	\
	.hw_map = GE2D_COLOR_MAP_ ## _map,		\
	.num_planes = 1,				\
}

/* Semi-planar NV12/NV21 (4:2:0): UV height = Y height / 2 */
#define FMTNV(_fourcc, _map)				\
{							\
	.fourcc = _fourcc,				\
	.yuv = true,					\
	.nv = true,					\
	.depth = 8,					\
	.hw_fmt = GE2D_FORMAT_24BIT,			\
	.hw_map = GE2D_COLOR_MAP_ ## _map,		\
	.uv_vdiv = 2,					\
	.num_planes = 1,				\
}

/* Semi-planar NV16/NV61 (4:2:2): UV height = Y height */
#define FMTNV422(_fourcc, _map)				\
{							\
	.fourcc = _fourcc,				\
	.yuv = true,					\
	.nv = true,					\
	.depth = 8,					\
	.hw_fmt = GE2D_FORMAT_24BIT,			\
	.hw_map = GE2D_COLOR_MAP_ ## _map,		\
	.uv_vdiv = 1,					\
	.num_planes = 1,				\
}

/*
 * 3-plane planar YUV: Y + Cb + Cr in separate planes.
 * _vdiv: vertical subsampling of chroma (2 = 4:2:0, 1 = 4:2:2)
 * _hdiv: horizontal subsampling of chroma (2 = half width)
 * hw_fmt: GE2D_FORMAT_8BIT -- each canvas is 8-bit per component
 */
#define FMTPLANAR(_fourcc, _vdiv, _hdiv)		\
{							\
	.fourcc = _fourcc,				\
	.yuv = true,					\
	.planar = true,					\
	.depth = 8,					\
	.hw_fmt = GE2D_FORMAT_8BIT,			\
	.hw_map = 0,					\
	.uv_vdiv = _vdiv,				\
	.uv_hdiv = _hdiv,				\
	.num_planes = 1,				\
}

static const struct ge2d_fmt formats[] = {
	/*
	 * 32-bit RGB formats.
	 *
	 * V4L2 names describe MEMORY byte order on LE (e.g. ARGB32 =
	 * {B,G,R,A} in memory = 0xAARRGGBB as LE u32).
	 *
	 * GE2D color map names describe the 32-bit REGISTER WORD order
	 * (MSB to LSB). ARGB8888 = A[31:24] R[23:16] G[15:8] B[7:0]
	 * = 0xAARRGGBB. On LE this stores as {B,G,R,A} in memory.
	 *
	 * So V4L2 ARGB32 (memory {B,G,R,A}) = GE2D ARGB8888 (register
	 * 0xAARRGGBB). The mapping is by the 32-bit word interpretation,
	 * not by memory byte name.
	 *
	 * BSP ge2d.h confirms: GE2D_FORMAT_S32_ARGB uses COLOR_MAP_ARGB8888.
	 */
	FMT(V4L2_PIX_FMT_XRGB32, false, 32, ARGB8888),
	FMT(V4L2_PIX_FMT_RGB32, true, 32, ARGB8888),
	FMT(V4L2_PIX_FMT_ARGB32, true, 32, ARGB8888),
	FMT(V4L2_PIX_FMT_RGBX32, false, 32, RGBA8888),
	FMT(V4L2_PIX_FMT_RGBA32, true, 32, RGBA8888),
	/*
	 * GI-015: the BGR-family V4L2 formats were mapped to swapped GE2D
	 * color maps. Per the V4L2 byte layout (pixfmt-rgb.rst) and the
	 * channel probe (tests/ge2d/ge2d-fmt-channel-probe.c):
	 *   - memory {R,G,B,*} (R at byte0) -> ABGR8888 (reg 0xAABBGGRR,
	 *     LE mem {R,G,B,A}): BGRX32, BGRA32.
	 *   - memory {*,R,G,B} (R at byte1) -> BGRA8888 (reg 0xBBGGRRAA,
	 *     LE mem {A,R,G,B}): XBGR32, ABGR32.
	 * (RGB/ARGB/RGBX/RGBA families were already correct.) BGR32 is the
	 * deprecated alias and is left unchanged (not covered by the probe).
	 */
	FMT(V4L2_PIX_FMT_BGRX32, false, 32, ABGR8888),
	FMT(V4L2_PIX_FMT_BGRA32, true, 32, ABGR8888),
	FMT(V4L2_PIX_FMT_BGR32, true, 32, ABGR8888),
	FMT(V4L2_PIX_FMT_ABGR32, true, 32, BGRA8888),
	FMT(V4L2_PIX_FMT_XBGR32, false, 32, BGRA8888),

	FMT(V4L2_PIX_FMT_RGB24, false, 24, BGR888),
	FMT(V4L2_PIX_FMT_BGR24, false, 24, RGB888),

	FMT(V4L2_PIX_FMT_XRGB555X, false, 16, ARGB1555),
	FMT(V4L2_PIX_FMT_ARGB555X, true, 16, ARGB1555),
	FMT(V4L2_PIX_FMT_RGB565, false, 16, RGB565),
	FMT(V4L2_PIX_FMT_RGBX444, false, 16, RGBA4444),
	FMT(V4L2_PIX_FMT_RGBA444, true, 16, RGBA4444),
	FMT(V4L2_PIX_FMT_XRGB444, false, 16, ARGB4444),
	FMT(V4L2_PIX_FMT_ARGB444, true, 16, ARGB4444),

	/*
	 * 16-bit packed YUV422. The hw color_map (=0) is VYUY-ordered
	 * ([Cr,Y0,Cb,Y1] -- chroma-first AND Cb<->Cr-swapped vs UYVY), NOT YUYV.
	 * RE 2026-06-07: YUYV fed here decoded chroma-first (black->green); a
	 * relabel to UYVY fixed black, but the colour harness (red/blue test,
	 * GI-023) then showed the chroma lanes are also Cb<->Cr swapped -- ge2d
	 * consumes [Cr,Y0,Cb,Y1] = VYUY (ffmpeg uyvy422 red rendered blue). No
	 * tunable SRC1 register changes the byte order. So advertise VYUY; a
	 * YUYV/UYVY source gets an automatic videoconvert before ge2d.
	 */
	FMTYUV(V4L2_PIX_FMT_VYUY, false, 16, YUV422),

	/*
	 * 16-bit packed YUV565 (V4L2_PIX_FMT_YUV565, hw color_map 5 + yuv flag)
	 * REMOVED 2026-06-07 (GI-026): the source datapath is colour-broken on
	 * this HW -- every input (incl. white, a non-zero 0xEC10 packed word)
	 * decodes to BLACK (0,0,0) regardless of byte order (LE and BE both
	 * tested via ge2d-color-characterize.sh). Do not advertise a corrupting
	 * format (the GI-023 footgun); re-add only behind a confirmed datapath.
	 */

	/*
	 * 24-bit packed YUV444 (V4L2_PIX_FMT_YUV24) REMOVED 2026-06-07 (GI-024):
	 * colour-broken on HW -- black decodes (32,126,49), gradients 6-13 dB, no
	 * byte-order round-trips. Do not advertise a corrupting format; re-add only
	 * behind a confirmed datapath + colour probe.
	 */

	/* 32-bit packed YUV/YUVA variants */
	FMTYUV(V4L2_PIX_FMT_YUV32, false, 32, YUVA8888),
	FMTYUV(V4L2_PIX_FMT_YUVA32, true, 32, YUVA8888),
	FMTYUV(V4L2_PIX_FMT_YUVX32, false, 32, YUVA8888),
	FMTYUV(V4L2_PIX_FMT_AYUV32, true, 32, AYUV8888),
	FMTYUV(V4L2_PIX_FMT_VUYA32, true, 32, VUYA8888),
	FMTYUV(V4L2_PIX_FMT_VUYX32, false, 32, VUYA8888),
	/*
	 * XYUV32 is the X-alpha twin of AYUV32: identical memory layout
	 * (byte0 X/A, byte1 Y, byte2 Cb, byte3 Cr per pixfmt-packed-yuv.rst)
	 * with the alpha byte don't-care. AYUV32 already maps to AYUV8888
	 * (register MSB->LSB A:Y:U:V), so XYUV32 takes the same hw map --
	 * exactly as YUVX32 mirrors YUVA32 above. (GE2D format coverage F-25.)
	 * VERIFIED 2026-06-07 (GI-026): black/white/red/green/blue round-trip
	 * (deltas <=2) via ge2d-color-characterize.sh on SM1.
	 */
	FMTYUV(V4L2_PIX_FMT_XYUV32, false, 32, AYUV8888),

	/* NV12/NV21 semi-planar YUV420 -- multi-canvas addressing */
	FMTNV(V4L2_PIX_FMT_NV12, NV12),
	FMTNV(V4L2_PIX_FMT_NV21, NV21),

	/* Multi-planar NV12M/NV21M -- separate Y and UV DMA buffers */
	{ .fourcc = V4L2_PIX_FMT_NV12M, .yuv = true, .nv = true, .depth = 8,
	  .hw_fmt = GE2D_FORMAT_24BIT, .hw_map = GE2D_COLOR_MAP_NV12,
	  .uv_vdiv = 2, .num_planes = 2 },
	{ .fourcc = V4L2_PIX_FMT_NV21M, .yuv = true, .nv = true, .depth = 8,
	  .hw_fmt = GE2D_FORMAT_24BIT, .hw_map = GE2D_COLOR_MAP_NV21,
	  .uv_vdiv = 2, .num_planes = 2 },

	/* NV16/NV61 semi-planar YUV422 -- multi-canvas, full-height UV */
	FMTNV422(V4L2_PIX_FMT_NV16, NV12),
	FMTNV422(V4L2_PIX_FMT_NV61, NV21),

	/* 3-plane planar YUV420 (I420 / YV12) */
	FMTPLANAR(V4L2_PIX_FMT_YUV420, 2, 2),
	FMTPLANAR(V4L2_PIX_FMT_YVU420, 2, 2),

	/* 3-plane planar YUV422 */
	FMTPLANAR(V4L2_PIX_FMT_YUV422P, 1, 2),

	/*
	 * 8-bit indexed palette (F-10).
	 *
	 * SRC1 reads one byte per pixel.  When this format is the OUTPUT
	 * fourcc and a palette has been uploaded via VIDIOC_MESON_GE2D_SET_CLUT8,
	 * the hardware sets GE2D_GEN_CTRL0 bit 3 (SRC1_LUT_EN) and looks each
	 * SRC1 byte up in the 256-entry ARGB8888 LUT before feeding it into
	 * the rest of the SRC1 path (CSC, scaler, ALU).  Only valid on the
	 * OUTPUT queue -- the hardware has no LUT on the DST path.
	 */
	{ .fourcc = V4L2_PIX_FMT_PAL8, .depth = 8, .hw_fmt = GE2D_FORMAT_8BIT,
	  .hw_map = 0, .num_planes = 1 },

	/*
	 * 10-bit / 12-bit deep color formats (G12A+ only).
	 * Placeholder entries: fourcc + hw_fmt/hw_map are correct, but
	 * ge2d_hw_start() does not yet set GEN_CTRL2 deep_color bit.
	 * These formats will be rejected at S_FMT until the deep_color
	 * path is wired and gated by has_deep_color in match_data.
	 *
	 * 10-bit YUV444: 30-bit packed, format=24BIT, map=YUV444.
	 * 10-bit YUV422: 20-bit packed, format=16BIT, map=YUV422.
	 * 12-bit YUV422: 24-bit packed, format=16BIT, map=12BIT_YUV422.
	 * 10-bit RGB:    30-bit packed, format=24BIT, map=RGB888.
	 */
	{ .fourcc = V4L2_PIX_FMT_MESON_Y10P444, .yuv = true, .le = true, .num_planes = 1,
	  .deep_color = true, .depth = 32, .hw_fmt = GE2D_FORMAT_24BIT,
	  .hw_map = GE2D_COLOR_MAP_10BIT_YUV444 },
	{ .fourcc = V4L2_PIX_FMT_MESON_Y10P422, .yuv = true, .le = true, .num_planes = 1,
	  .deep_color = true, .depth = 20, .hw_fmt = GE2D_FORMAT_16BIT,
	  .hw_map = GE2D_COLOR_MAP_10BIT_YUV422 },
	{ .fourcc = V4L2_PIX_FMT_MESON_Y12P422, .yuv = true, .le = true, .num_planes = 1,
	  .deep_color = true, .depth = 24, .hw_fmt = GE2D_FORMAT_16BIT,
	  .hw_map = GE2D_COLOR_MAP_12BIT_YUV422 },
	{ .fourcc = V4L2_PIX_FMT_MESON_R10P, .le = true, .num_planes = 1,
	  .deep_color = true, .depth = 32, .hw_fmt = GE2D_FORMAT_24BIT,
	  .hw_map = GE2D_COLOR_MAP_RGB888 },

	/* Little-endian ARGB/XRGB 1555 (native byte order) */
	FMT(V4L2_PIX_FMT_XRGB555, false, 16, ARGB1555),
	FMT(V4L2_PIX_FMT_ARGB555, true, 16, ARGB1555),
};

#define NUM_FORMATS ARRAY_SIZE(formats)

static const struct ge2d_fmt *find_fmt(struct v4l2_format *f)
{
	unsigned int i;

	for (i = 0; i < NUM_FORMATS; i++) {
		if (formats[i].fourcc == f->fmt.pix_mp.pixelformat)
			return &formats[i];
	}

	/*
	 * Unknown fourcc -- return NULL so vidioc_try_fmt_{cap,out} can
	 * surface -EINVAL.  The V4L2 spec allows TRY_FMT/S_FMT to rewrite
	 * an unsupported request to a working alternative, but the prior
	 * fallback (`return &formats[0]` = `V4L2_PIX_FMT_XRGB32`) combined
	 * with `def_frame.pix_mp.width/height = 128` silently masked every
	 * userspace bug as "stuck 128x128 BX24" (KI-054).  Strict rejection
	 * here lets v4l2-compliance / userspace diagnose the request.
	 */
	return NULL;
}

static const struct ge2d_fmt *find_fmt_by_fourcc(u32 pixelformat)
{
	unsigned int i;

	for (i = 0; i < NUM_FORMATS; i++) {
		if (formats[i].fourcc == pixelformat)
			return &formats[i];
	}
	return NULL;
}

static struct ge2d_frame *get_frame(struct ge2d_ctx *ctx,
				    enum v4l2_buf_type type)
{
	switch (type) {
	/*
	 * The V4L2 core (v4l2-ioctl.c v4l_g/s_selection) maps an _MPLANE
	 * buffer type to its non-MPLANE counterpart before dispatching to the
	 * driver, so accept both forms here. (S_FMT/REQBUFS still arrive as
	 * _MPLANE on this mplane-only node; selection ioctls do not.)
	 */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		return &ctx->in;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		return &ctx->out;
	default:
		/* This should never happen, warn and return OUTPUT frame */
		dev_warn(ctx->ge2d->dev, "%s: invalid buffer type\n", __func__);
		return &ctx->in;
	}
}

static const u32 filt_coef_bicubic[] = {
	0x00800000, 0x007f0100, 0xff7f0200, 0xfe7f0300,
	0xfd7e0500, 0xfc7e0600, 0xfb7d0800, 0xfb7c0900,
	0xfa7b0b00, 0xfa7a0dff, 0xf9790fff, 0xf97711ff,
	0xf87613ff, 0xf87416fe, 0xf87218fe, 0xf8701afe,
	0xf76f1dfd, 0xf76d1ffd, 0xf76b21fd, 0xf76824fd,
	0xf76627fc, 0xf76429fc, 0xf7612cfc, 0xf75f2ffb,
	0xf75d31fb, 0xf75a34fb, 0xf75837fa, 0xf7553afa,
	0xf8523cfa, 0xf8503ff9, 0xf84d42f9, 0xf84a45f9,
	0xf84848f8,
};

static const u32 filt_coef_triangle[] = {
	0x40400000, 0x3f400100, 0x3d410200, 0x3c410300,
	0x3a420400, 0x39420500, 0x37430600, 0x36430700,
	0x35430800, 0x33450800, 0x32450900, 0x31450a00,
	0x30450b00, 0x2e460c00, 0x2d460d00, 0x2c470d00,
	0x2b470e00, 0x29480f00, 0x28481000, 0x27481100,
	0x26491100, 0x25491200, 0x24491300, 0x234a1300,
	0x224a1400, 0x214a1500, 0x204a1600, 0x1f4b1600,
	0x1e4b1700, 0x1d4b1800, 0x1c4c1800, 0x1b4c1900,
	0x1a4c1a00,
};

static const u32 filt_coef_bilinear[] = {
	0x00800000, 0x007e0200, 0x007c0400, 0x007a0600,
	0x00780800, 0x00760a00, 0x00740c00, 0x00720e00,
	0x00701000, 0x006e1200, 0x006c1400, 0x006a1600,
	0x00681800, 0x00661a00, 0x00641c00, 0x00621e00,
	0x00602000, 0x005e2200, 0x005c2400, 0x005a2600,
	0x00582800, 0x00562a00, 0x00542c00, 0x00522e00,
	0x00503000, 0x004e3200, 0x004c3400, 0x004a3600,
	0x00483800, 0x00463a00, 0x00443c00, 0x00423e00,
	0x00404000,
};

static const u32 *filt_coef_tables[] = {
	[0] = filt_coef_bilinear,   /* BILINEAR (default) */
	[1] = filt_coef_bicubic,    /* BICUBIC (sharper) */
	[2] = filt_coef_triangle,   /* TRIANGLE (smoother) */
};

static void ge2d_load_scale_coef(struct meson_ge2d *ge2d, const u32 *coef)
{
	int i;

	regmap_write(ge2d->map, GE2D_SCALE_COEF_IDX, 0x0000);
	for (i = 0; i < 33; i++)
		regmap_write(ge2d->map, GE2D_SCALE_COEF, coef[i]);

	regmap_write(ge2d->map, GE2D_SCALE_COEF_IDX, 0x0100);
	for (i = 0; i < 33; i++)
		regmap_write(ge2d->map, GE2D_SCALE_COEF, coef[i]);
}

/*
 * CSC matrix coefficients.  The hardware uses 13-bit signed
 * fixed-point values (S2.10) for the 3x3 matrix.  Values >= 0x1000
 * represent negative numbers in two's-complement.
 *
 * Pre-offsets are 10-bit signed packed (Y:20, Cb:10, Cr:0).
 * Post-offsets are 10-bit unsigned packed the same way.
 *
 * Sources: BSP ge2d_hw.c for BT.601, BSP vdin_ctl.c for BT.709
 * (VDIN_MATRIX_YUV709_RGB and VDIN_MATRIX_RGB_YUV709).
 */

/* BT.601 YCbCr(16-235) -> RGB(0-255) (BSP ge2d_hw.c:938-946) */
static const u16 csc_bt601_yuv2rgb[9] = {
	0x04a8, 0x0000, 0x0662,
	0x04a8, 0x1e6f, 0x1cbf,
	0x04a8, 0x0811, 0x0000,
};

/* BT.601 RGB(0-255) -> YCbCr(16-235) (BSP ge2d_hw.c:973-981) */
static const u16 csc_bt601_rgb2yuv[9] = {
	0x0107, 0x0204, 0x0064,
	0x1f68, 0x1ed6, 0x01c2,
	0x01c2, 0x1e87, 0x1fb7,
};

/*
 * BT.709 YCbCr(16-235) -> RGB(0-255) (BSP vdin_ctl.c:404-408)
 * -16   1.164  0      1.793    0
 * -128  1.164 -0.213 -0.534    0
 * -128  1.164  2.115  0        0
 */
static const u16 csc_bt709_yuv2rgb[9] = {
	0x04a8, 0x0000, 0x072c,
	0x04a8, 0x1f26, 0x1ddd,
	0x04a8, 0x0876, 0x0000,
};

/*
 * BT.709 RGB(0-255) -> YCbCr(16-235) (BSP ge2d_hw.c:959-967)
 * 0.183  0.614  0.062    16
 * -0.101 -0.338  0.439   128
 * 0.439 -0.399 -0.04    128
 */
static const u16 csc_bt709_rgb2yuv[9] = {
	0x00bb, 0x0275, 0x003f,
	0x1f99, 0x1ea6, 0x01c2,
	0x01c2, 0x1e67, 0x1fd7,
};

/*
 * Program the GE2D color space conversion matrix.
 * @ge2d: device
 * @yuv2rgb: true for YUV->RGB, false for RGB->YUV
 * @bt709: true for BT.709 matrix, false for BT.601
 */
static void ge2d_set_csc_matrix(struct meson_ge2d *ge2d, bool yuv2rgb,
				bool bt709)
{
	const u16 *coef;

	if (yuv2rgb)
		coef = bt709 ? csc_bt709_yuv2rgb : csc_bt601_yuv2rgb;
	else
		coef = bt709 ? csc_bt709_rgb2yuv : csc_bt601_rgb2yuv;

	/*
	 * Pre-offset: subtract bias from input before matrix multiply.
	 * YUV->RGB: Y-16, Cb-128, Cr-128 (limited range YUV input).
	 * RGB->YUV: no pre-offset (full range RGB input).
	 * Same pre-offsets for BT.601 and BT.709 (BSP vdin_ctl.c:408).
	 */
	if (yuv2rgb) {
		regmap_write(ge2d->map, GE2D_MATRIX_PRE_OFFSET,
			     (0x1f0 << 20) | (0x180 << 10) | 0x180);
	} else {
		regmap_write(ge2d->map, GE2D_MATRIX_PRE_OFFSET, 0);
	}

	regmap_write(ge2d->map, GE2D_MATRIX_COEF00_01,
		     (coef[0] << 16) | coef[1]);
	regmap_write(ge2d->map, GE2D_MATRIX_COEF02_10,
		     (coef[2] << 16) | coef[3]);
	regmap_write(ge2d->map, GE2D_MATRIX_COEF11_12,
		     (coef[4] << 16) | coef[5]);
	regmap_write(ge2d->map, GE2D_MATRIX_COEF20_21,
		     (coef[6] << 16) | coef[7]);
	regmap_write(ge2d->map, GE2D_MATRIX_COEF22_CTRL,
		     (coef[8] << 16) |
		     (yuv2rgb ? GE2D_MATRIX_SAT_IN_EN : 0) |
		     GE2D_MATRIX_EN);

	/*
	 * Post-offset: add bias after matrix multiply.
	 * YUV->RGB: no post-offset (output is full range RGB).
	 * RGB->YUV: +16 for Y, +128 for Cb/Cr (limited range YUV output).
	 * Same post-offsets for BT.601 and BT.709.
	 */
	if (yuv2rgb) {
		regmap_write(ge2d->map, GE2D_MATRIX_OFFSET, 0);
	} else {
		regmap_write(ge2d->map, GE2D_MATRIX_OFFSET,
			     (16 << 20) | (128 << 10) | 128);
	}
}

/*
 * Determine whether BT.709 CSC should be used based on the V4L2
 * colorspace and ycbcr_enc fields.  The source format's colorspace
 * is authoritative -- userspace sets it when providing YUV data or
 * when requesting specific YUV output encoding.
 */
static bool ge2d_is_bt709(const struct v4l2_pix_format_mplane *pix)
{
	if (pix->ycbcr_enc == V4L2_YCBCR_ENC_709)
		return true;
	if (pix->colorspace == V4L2_COLORSPACE_REC709)
		return true;
	return false;
}

/*
 * GI-018: round-trippable colorimetry on the OUTPUT/CAPTURE queues.
 *
 * GE2D only runs a CSC matrix on YUV<->RGB conversions; a YUV->YUV blit
 * (the dmabuf rotate/scale path a HW encoder sits behind) takes the
 * MATRIX_COEF22_CTRL=0 / no-CSC branch in the blit setup, so the pixel
 * values -- and therefore the colorimetry (colorspace / ycbcr_enc /
 * xfer_func / quantization) -- pass through unchanged.  The CAPTURE
 * (DST) queue MUST be allowed to advertise the SAME colorimetry the
 * caller set on the OUTPUT (SRC) side, so an identity bt709->bt709
 * pairing round-trips and gst links the encoder with no CPU
 * videoconvert.
 *
 * The previous code unconditionally OVERWROTE all four fields (and never
 * set xfer_func, leaving it 0/DEFAULT) which forced a fixed
 * 2:3:<default>:1 result and rejected the caller's request.  We instead
 * HONOR each caller-supplied field, supplying a sane default only when
 * the caller left a field unset (DEFAULT == 0).  No CSC matrix is keyed
 * off these fields for YUV->YUV, so honoring them cannot mis-program the
 * hardware; ge2d_is_bt709() (used only for the YUV<->RGB matrix select)
 * still reads colorspace/ycbcr_enc independently.
 */
static void ge2d_fixup_colorimetry(struct v4l2_pix_format_mplane *pix,
				   bool yuv)
{
	if (!yuv) {
		/*
		 * RGB: default to full-range sRGB but honor explicitly-set
		 * colorimetry so every field round-trips (V4L2 colorimetry
		 * conformance -- the OUTPUT colorimetry is caller metadata).
		 * ycbcr_enc/xfer_func DEFAULT are valid round-trip values --
		 * leave them untouched.
		 */
		if (pix->colorspace == V4L2_COLORSPACE_DEFAULT)
			pix->colorspace = V4L2_COLORSPACE_SRGB;
		if (pix->quantization == V4L2_QUANTIZATION_DEFAULT)
			pix->quantization = V4L2_QUANTIZATION_FULL_RANGE;
		return;
	}

	/*
	 * YUV: honor what the caller asked for, defaulting per-field only
	 * when unset.  Keep colorspace and ycbcr_enc INDEPENDENT so a
	 * (SMPTE170M colorspace + ENC_709) request -- the combination gst
	 * builds from a bt709 decoder -- round-trips instead of being
	 * coerced to REC709.
	 */
	if (pix->colorspace == V4L2_COLORSPACE_DEFAULT)
		pix->colorspace = V4L2_COLORSPACE_SMPTE170M;

	if (pix->ycbcr_enc == V4L2_YCBCR_ENC_DEFAULT) {
		pix->ycbcr_enc = (pix->colorspace == V4L2_COLORSPACE_REC709) ?
				 V4L2_YCBCR_ENC_709 : V4L2_YCBCR_ENC_601;
	}

	/* xfer_func DEFAULT (0) is a valid round-trip value -- leave it. */

	if (pix->quantization == V4L2_QUANTIZATION_DEFAULT)
		pix->quantization = V4L2_QUANTIZATION_LIM_RANGE;
}

/* SRC2 compositing helpers (F-04) */

static void ge2d_src2_cleanup(struct ge2d_ctx *ctx)
{
	if (ctx->src2_sgt) {
		dma_buf_unmap_attachment(ctx->src2_attach, ctx->src2_sgt,
					DMA_TO_DEVICE);
		ctx->src2_sgt = NULL;
	}
	if (ctx->src2_attach) {
		dma_buf_detach(ctx->src2_dmabuf, ctx->src2_attach);
		ctx->src2_attach = NULL;
	}
	if (ctx->src2_dmabuf) {
		dma_buf_put(ctx->src2_dmabuf);
		ctx->src2_dmabuf = NULL;
	}
	ctx->src2_bound_fd = -1;
	ctx->blend_mode = false;
}

static int ge2d_set_src2(struct ge2d_ctx *ctx, void *arg)
{
	struct meson_ge2d_src2_info *info = arg;
	const struct ge2d_fmt *fmt;

	if (info->reserved[0] || info->reserved[1])
		return -EINVAL;

	/* fd == -1: disable SRC2, release buffer */
	if (info->fd == -1) {
		ge2d_src2_cleanup(ctx);
		ctx->blend_mode = false;
		return 0;
	}

	/* FLAG_FILL_COLOR: no buffer needed, skip DMABUF handling */
	if (info->flags & MESON_GE2D_SRC2_FLAG_FILL_COLOR)
		goto update_params;

	/* fd == -2: keep current buffer, update parameters only */
	if (info->fd == -2) {
		if (!ctx->blend_mode)
			return -EINVAL;
		goto update_params;
	}

	/* fd >= 0: bind new buffer (skip re-import if same fd) */
	if (info->fd != ctx->src2_bound_fd) {
		ge2d_src2_cleanup(ctx);

		fmt = find_fmt_by_fourcc(info->pixelformat);
		if (!fmt || fmt->nv || fmt->planar)
			return -EINVAL;

		ctx->src2_dmabuf = dma_buf_get(info->fd);
		if (IS_ERR(ctx->src2_dmabuf)) {
			int ret = PTR_ERR(ctx->src2_dmabuf);

			ctx->src2_dmabuf = NULL;
			return ret;
		}
		ctx->src2_attach = dma_buf_attach(ctx->src2_dmabuf,
						  ctx->ge2d->dev);
		if (IS_ERR(ctx->src2_attach)) {
			int ret = PTR_ERR(ctx->src2_attach);

			ctx->src2_attach = NULL;
			dma_buf_put(ctx->src2_dmabuf);
			ctx->src2_dmabuf = NULL;
			return ret;
		}
		ctx->src2_sgt = dma_buf_map_attachment(ctx->src2_attach,
						       DMA_TO_DEVICE);
		if (IS_ERR(ctx->src2_sgt)) {
			int ret = PTR_ERR(ctx->src2_sgt);

			ctx->src2_sgt = NULL;
			dma_buf_detach(ctx->src2_dmabuf, ctx->src2_attach);
			ctx->src2_attach = NULL;
			dma_buf_put(ctx->src2_dmabuf);
			ctx->src2_dmabuf = NULL;
			return ret;
		}
		ctx->src2_dma_addr = sg_dma_address(ctx->src2_sgt->sgl);
		ctx->src2_bound_fd = info->fd;
		ctx->src2_fmt = fmt;
		ctx->src2_width = info->width;
		ctx->src2_height = info->height;
		ctx->src2_bytesperline = info->bytesperline;
	}

update_params:
	ctx->src2_x_offset = info->x_offset;
	ctx->src2_y_offset = info->y_offset;
	ctx->blend_op = info->blend_op;
	ctx->src2_gb_alpha = info->src2_gb_alpha;
	ctx->const_color = info->const_color;
	ctx->def_color = info->def_color;
	ctx->src2_color_key = info->color_key;
	ctx->src2_color_key_mask = info->color_key_mask;
	ctx->src2_repeat_x = min_t(u32, info->repeat_x, 3);
	ctx->src2_repeat_y = min_t(u32, info->repeat_y, 3);
	ctx->src2_flags = info->flags;
	ctx->blend_mode = true;
	return 0;
}

static int ge2d_get_src2(struct ge2d_ctx *ctx, void *arg)
{
	struct meson_ge2d_src2_info *info = arg;

	memset(info, 0, sizeof(*info));
	if (!ctx->blend_mode) {
		info->fd = -1;
		return 0;
	}
	info->fd = ctx->src2_bound_fd;
	info->pixelformat = ctx->src2_fmt ? ctx->src2_fmt->fourcc : 0;
	info->width = ctx->src2_width;
	info->height = ctx->src2_height;
	info->bytesperline = ctx->src2_bytesperline;
	info->x_offset = ctx->src2_x_offset;
	info->y_offset = ctx->src2_y_offset;
	info->blend_op = ctx->blend_op;
	info->src2_gb_alpha = ctx->src2_gb_alpha;
	info->const_color = ctx->const_color;
	info->def_color = ctx->def_color;
	info->color_key = ctx->src2_color_key;
	info->color_key_mask = ctx->src2_color_key_mask;
	info->repeat_x = ctx->src2_repeat_x;
	info->repeat_y = ctx->src2_repeat_y;
	info->flags = ctx->src2_flags;
	return 0;
}

/*
 * CLUT8 palette upload handler (F-10).
 *
 * Copies the user-supplied ARGB8888 palette into ctx->clut8[].  The
 * palette is uploaded to hardware lazily in ge2d_hw_start() when the
 * SRC1 format is V4L2_PIX_FMT_PAL8.
 */
static int ge2d_set_clut8(struct ge2d_ctx *ctx, void *arg)
{
	struct meson_ge2d_clut8_info *info = arg;

	if (info->flags != 0)
		return -EINVAL;
	if (info->reserved[0] != 0 || info->reserved[1] != 0 ||
	    info->reserved[2] != 0)
		return -EINVAL;
	if (info->count == 0 || info->count > 256)
		return -EINVAL;

	memcpy(ctx->clut8, info->data, info->count * sizeof(u32));
	/* Zero any unset tail so the hardware sees stable contents. */
	if (info->count < 256)
		memset(&ctx->clut8[info->count], 0,
		       (256 - info->count) * sizeof(u32));
	ctx->clut8_count = info->count;
	ctx->clut8_present = true;
	return 0;
}

static long ge2d_ioctl_default(struct file *file, void *fh,
			       bool valid_prio, unsigned int cmd, void *arg)
{
	/* 6.18: v4l2-ioctl.c dispatches .vidioc_default with fh == NULL */
	struct ge2d_ctx *ctx = file_to_ge2d_ctx(file);

	switch (cmd) {
	case VIDIOC_MESON_GE2D_SET_SRC2:
		return ge2d_set_src2(ctx, arg);
	case VIDIOC_MESON_GE2D_GET_SRC2:
		return ge2d_get_src2(ctx, arg);
	case VIDIOC_MESON_GE2D_SET_CLUT8:
		return ge2d_set_clut8(ctx, arg);
	default:
		return -ENOTTY;
	}
}

static int ge2d_hw_start(struct meson_ge2d *ge2d)
{
	struct ge2d_ctx *ctx = ge2d->curr;
	bool src_yuv = ctx->in.fmt->yuv;
	bool dst_yuv = ctx->out.fmt->yuv;
	bool src_nv = ctx->in.fmt->nv;
	bool dst_nv = ctx->out.fmt->nv;
	bool src_planar = ctx->in.fmt->planar;
	bool dst_planar = ctx->out.fmt->planar;
	bool src_pal8 = ctx->in.fmt->fourcc == V4L2_PIX_FMT_PAL8;

	/*
	 * SRC1 PAL8 requires a palette uploaded via VIDIOC_MESON_GE2D_SET_CLUT8
	 * before the first blit.  Reject the job rather than program undefined
	 * LUT contents; device_run() will report this to v4l2-m2m.
	 */
	if (src_pal8 && !ctx->clut8_present) {
		dev_err(ge2d->dev,
			"SRC1 PAL8 requires VIDIOC_MESON_GE2D_SET_CLUT8 first\n");
		return -EINVAL;
	}
	/*
	 * Source has horizontal chroma subsampling (4:2:2 or 4:2:0):
	 *   - semi-planar (NV12/NV21/NM12/NM21/NV16/NV61): always subsampled
	 *   - planar (YU12/YV12/422P): always subsampled
	 *   - packed 16-bit YUV (YUYV): 4:2:2
	 *   - MESON deep-color YUV422 (AM2A Y10P422, AM2C Y12P422): 4:2:2
	 *     packed in wider containers (depth 20 / 24), so the depth==16
	 *     check above misses them. Without HFMT_EN the hardware reads
	 *     subsampled chroma as 4:4:4 and stalls the SRC1 pipeline --
	 *     observed hang 2026-04-24 on G12B/SM1 (KI-2026-04-24-A). The
	 *     BSP derives this from GE2D_FORMAT_16BIT with the deep_color
	 *     bit (ge2dgen.c:19 / ge2d_hw.c:447).
	 *     AM4A (Y10P444, GE2D_FORMAT_24BIT) is native 4:4:4 and must
	 *     not set this bit.
	 */
	bool src_x_yc_ratio = src_yuv &&
			       (src_nv || src_planar ||
				ctx->in.fmt->depth == 16 ||
				(ctx->in.fmt->deep_color &&
				 ctx->in.fmt->hw_fmt == GE2D_FORMAT_16BIT));
	/* Source has vertical chroma subsampling (4:2:0 only) */
	bool src_y_yc_ratio = (src_nv || src_planar) &&
			       ctx->in.fmt->uv_vdiv == 2;
	u32 widthi, heighti, widtho, heighto;
	u32 hsc_en = 0, vsc_en = 0;
	u32 reg;

	/* Reset */
	regmap_update_bits(ge2d->map, GE2D_GEN_CTRL1,
			   GE2D_SOFT_RST, GE2D_SOFT_RST);
	regmap_update_bits(ge2d->map, GE2D_GEN_CTRL1,
			   GE2D_SOFT_RST, 0);

	usleep_range(100, 200);

	/*
	 * Clear stale SRC2 blend state from a prior blend blit (this ctx
	 * or another).  The non-blend path's full GEN_CTRL2 write overwrites
	 * SRC2 format bits [15:8], and CMD_CTRL is written fresh every blit.
	 * Only GEN_CTRL0 bit 29 (SRC2 key) and GEN_CTRL5 bit 8 (SRC2
	 * gb_alpha) persist and must be explicitly cleared.
	 */
	if (!ctx->blend_mode) {
		regmap_update_bits(ge2d->map, GE2D_GEN_CTRL0, BIT(29), 0);
		if (ge2d->has_src2_alp)
			regmap_update_bits(ge2d->map, GE2D_GEN_CTRL5,
					   BIT(8), 0);
	}

	/* Set SRC1 outside-region alpha from the V4L2 alpha control.
	 * BSP ge2d_hw.c programs bits [7:0] = SRC1 outside_alpha.
	 * When alpha < 255 (blend mode active), outside-region pixels
	 * use the same global alpha for clean compositing borders.
	 * When alpha == 255 (copy mode), outside alpha is 255 (opaque).
	 */
	regmap_write(ge2d->map, GE2D_SRC_OUTSIDE_ALPHA,
		     ctx->alpha & 0xff);

	/*
	 * GE2D_GEN_CTRL4: bit 0 is hang_flag (set per blit on G12A/G12B/SM1 to
	 * disable the dp_out_done/scale_out_done hang fix; no effect on GXL/AXG).
	 * Bits [28:16] are the AXI FIFO/burst config (ge2d->burst_ctrl4).
	 *
	 * GI-016: the 64-byte AXI max-burst (live on G12B/SM1, dead on GXL)
	 * corrupts destination lines whose byte stride is not 64-byte-aligned --
	 * odd lines then start mid-burst and the right portion is mis-written
	 * (non-deterministic). Drop the burst for this blit when either the src
	 * or dst plane-0 stride is unaligned; aligned strides (e.g. 4K, the
	 * perf-critical case) keep the full burst and its +27%-at-4K gain.
	 * Confirmed: burst_config=0 eliminates the corruption.
	 */
	{
		u32 burst = ge2d->burst_ctrl4;

		if ((ctx->in.pix_mp.plane_fmt[0].bytesperline & 63) ||
		    (ctx->out.pix_mp.plane_fmt[0].bytesperline & 63))
			burst = 0;
		regmap_write(ge2d->map, GE2D_GEN_CTRL4,
			     (ge2d->has_hang_flag ? BIT(0) : 0) | burst);
	}

	/* Datapath duty-cycle: BSP inits to 0 (always on, no duty-cycle gating).
	 * bit 31 = dp_onoff_mode, bits [30:16] = dp_on_cnt,
	 * bit 15 = vfmt_onoff_en, bits [14:0] = dp_off_cnt.
	 */
	regmap_write(ge2d->map, GE2D_DP_ONOFF_CTRL, 0);

	if (ge2d->has_baddr) {
		dma_addr_t src_dma = vb2_dma_contig_plane_dma_addr(&ctx->in.buf->vb2_buf, 0);
		dma_addr_t dst_dma = vb2_dma_contig_plane_dma_addr(&ctx->out.buf->vb2_buf, 0);

		dev_info(ge2d->dev,
			 "blit: src=%pad/%u dst=%pad/%u %ux%u->%ux%u\n",
			 &src_dma, ctx->in.pix_mp.plane_fmt[0].bytesperline,
			 &dst_dma, ctx->out.pix_mp.plane_fmt[0].bytesperline,
			 ctx->in.pix_mp.width, ctx->in.pix_mp.height,
			 ctx->out.pix_mp.width, ctx->out.pix_mp.height);

		regmap_write(ge2d->map, GE2D_SRC1_BADDR_CTRL,
			     (src_dma + 7) >> 3);
		regmap_write(ge2d->map, GE2D_SRC1_STRIDE_CTRL,
			     (ctx->in.pix_mp.plane_fmt[0].bytesperline + 7) >> 3);
		/* SRC2 address: from SRC2 DMABUF when blending, from DST for readback */
		if (ctx->blend_mode &&
		    !(ctx->src2_flags & MESON_GE2D_SRC2_FLAG_FILL_COLOR)) {
			regmap_write(ge2d->map, GE2D_SRC2_BADDR_CTRL,
				     (ctx->src2_dma_addr + 7) >> 3);
			regmap_write(ge2d->map, GE2D_SRC2_STRIDE_CTRL,
				     (ctx->src2_bytesperline + 7) >> 3);
		} else {
			regmap_write(ge2d->map, GE2D_SRC2_BADDR_CTRL,
				     (dst_dma + 7) >> 3);
			regmap_write(ge2d->map, GE2D_SRC2_STRIDE_CTRL,
				     (ctx->out.pix_mp.plane_fmt[0].bytesperline + 7) >> 3);
		}
		regmap_write(ge2d->map, GE2D_DST1_BADDR_CTRL,
			     (dst_dma + 7) >> 3);
		regmap_write(ge2d->map, GE2D_DST1_STRIDE_CTRL,
			     (ctx->out.pix_mp.plane_fmt[0].bytesperline + 7) >> 3);

		/*
		 * Multi-plane output (NV12/NV21/NV16/NV61/planar):
		 * Program DST2 base address and stride via C1 registers
		 * so the chroma plane DMA has a valid target address.
		 * Without this, GE2D_DST2_ENABLE in GEN_CTRL3 tells the
		 * hardware to write chroma but the DMA address is 0,
		 * hanging the hardware.
		 */
		if (dst_nv || dst_planar) {
			u32 y_size = ctx->out.pix_mp.plane_fmt[0].bytesperline *
				     ctx->out.pix_mp.height;
			dma_addr_t dst2_dma;
			u32 dst2_stride;

			if (ctx->out.fmt->num_planes > 1)
				dst2_dma = vb2_dma_contig_plane_dma_addr(
						&ctx->out.buf->vb2_buf, 1);
			else
				dst2_dma = dst_dma + y_size;
			dst2_stride = dst_planar ?
				ctx->out.pix_mp.plane_fmt[0].bytesperline /
				ctx->out.fmt->uv_hdiv :
				ctx->out.pix_mp.plane_fmt[0].bytesperline;

			regmap_write(ge2d->map, GE2D_C1_DST2_BADDR_CTRL,
				     (dst2_dma + 7) >> 3);
			regmap_write(ge2d->map, GE2D_C1_DST2_STRIDE_CTRL,
				     (dst2_stride + 7) >> 3);
		}
	} else {
		u8 src1_canvas, src2_canvas, dst1_canvas;
		dma_addr_t src1_addr, dst1_addr;
		u32 src1_canvas_reg, dst_canvas_reg;

		src1_addr = vb2_dma_contig_plane_dma_addr(&ctx->in.buf->vb2_buf, 0);
		dst1_addr = vb2_dma_contig_plane_dma_addr(&ctx->out.buf->vb2_buf, 0);

		/*
		 * Canvas ID caching: allocate IDs only on the first blit of a
		 * streaming session.  Format and resolution are fixed while
		 * streaming, so the same IDs are reused for every subsequent
		 * blit.  Only meson_canvas_config() (DMA address update) runs
		 * every blit because the vb2 buffer address changes each time.
		 * IDs are freed in ge2d_stop_streaming() on the OUTPUT queue.
		 */
		if (!ctx->canvases_allocated) {
			ctx->has_src_uv_canvas = false;
			ctx->has_dst2_canvas = false;
			ctx->has_src_planar_canvas = false;
			ctx->has_dst_planar_canvas = false;

			meson_canvas_alloc(ge2d->canvas, &ctx->src1_canvas);
			meson_canvas_alloc(ge2d->canvas, &ctx->src2_canvas);
			meson_canvas_alloc(ge2d->canvas, &ctx->dst1_canvas);

			if (src_nv) {
				meson_canvas_alloc(ge2d->canvas,
						   &ctx->src1_canvas_uv);
				ctx->has_src_uv_canvas = true;
			} else if (src_planar) {
				meson_canvas_alloc(ge2d->canvas,
						   &ctx->src1_canvas_cb);
				meson_canvas_alloc(ge2d->canvas,
						   &ctx->src1_canvas_cr);
				ctx->has_src_planar_canvas = true;
			}

			if (dst_nv) {
				meson_canvas_alloc(ge2d->canvas,
						   &ctx->dst2_canvas);
				ctx->has_dst2_canvas = true;
			} else if (dst_planar) {
				meson_canvas_alloc(ge2d->canvas,
						   &ctx->dst_canvas_cb);
				meson_canvas_alloc(ge2d->canvas,
						   &ctx->dst_canvas_cr);
				ctx->has_dst_planar_canvas = true;
			}

			ctx->canvases_allocated = true;
		}

		/* Read cached IDs into locals for config calls below */
		src1_canvas = ctx->src1_canvas;
		src2_canvas = ctx->src2_canvas;
		dst1_canvas = ctx->dst1_canvas;

		/*
		 * Source canvas setup:
		 * - NV12/NV21/NV16/NV61: Y canvas + CbCr canvas (2 planes)
		 * - I420/YV12/YUV422P: Y canvas + Cb canvas + Cr canvas (3 planes)
		 * - Single-plane: just one canvas
		 * Always re-config: DMA addresses change per blit.
		 */
		if (src_nv) {
			dma_addr_t uv_addr;
			u32 y_size;
			u32 uv_height;

			y_size = ctx->in.pix_mp.plane_fmt[0].bytesperline *
				 ctx->in.pix_mp.height;
			if (ctx->in.fmt->num_planes > 1)
				uv_addr = vb2_dma_contig_plane_dma_addr(
						&ctx->in.buf->vb2_buf, 1);
			else
				uv_addr = src1_addr + y_size;
			uv_height = ctx->in.pix_mp.height /
				    ctx->in.fmt->uv_vdiv;

			meson_canvas_config(ge2d->canvas, src1_canvas,
					    src1_addr,
					    ctx->in.pix_mp.plane_fmt[0].bytesperline,
					    ctx->in.pix_mp.height,
					    MESON_CANVAS_WRAP_NONE,
					    MESON_CANVAS_BLKMODE_LINEAR, 0);

			meson_canvas_config(ge2d->canvas, ctx->src1_canvas_uv,
					    uv_addr,
					    ctx->in.pix_mp.plane_fmt[0].bytesperline,
					    uv_height,
					    MESON_CANVAS_WRAP_NONE,
					    MESON_CANVAS_BLKMODE_LINEAR, 0);
		} else if (src_planar) {
			dma_addr_t cb_addr, cr_addr;
			u32 y_size, cb_size;
			u32 uv_stride, uv_height;

			uv_stride = ctx->in.pix_mp.plane_fmt[0].bytesperline /
				    ctx->in.fmt->uv_hdiv;
			uv_height = ctx->in.pix_mp.height /
				    ctx->in.fmt->uv_vdiv;
			y_size = ctx->in.pix_mp.plane_fmt[0].bytesperline *
				 ctx->in.pix_mp.height;
			cb_size = uv_stride * uv_height;

			/*
			 * I420: Y + Cb + Cr (U before V)
			 * YV12: Y + Cr + Cb (V before U)
			 * The hw_map field is unused for planar; the
			 * canvas order determines Cb/Cr assignment.
			 * For YV12 (V4L2_PIX_FMT_YVU420), V comes
			 * before U in memory, so swap the addresses.
			 */
			if (ctx->in.fmt->fourcc == V4L2_PIX_FMT_YVU420) {
				cr_addr = src1_addr + y_size;
				cb_addr = cr_addr + cb_size;
			} else {
				cb_addr = src1_addr + y_size;
				cr_addr = cb_addr + cb_size;
			}

			meson_canvas_config(ge2d->canvas, src1_canvas,
					    src1_addr,
					    ctx->in.pix_mp.plane_fmt[0].bytesperline,
					    ctx->in.pix_mp.height,
					    MESON_CANVAS_WRAP_NONE,
					    MESON_CANVAS_BLKMODE_LINEAR, 0);

			meson_canvas_config(ge2d->canvas, ctx->src1_canvas_cb,
					    cb_addr, uv_stride, uv_height,
					    MESON_CANVAS_WRAP_NONE,
					    MESON_CANVAS_BLKMODE_LINEAR, 0);

			meson_canvas_config(ge2d->canvas, ctx->src1_canvas_cr,
					    cr_addr, uv_stride, uv_height,
					    MESON_CANVAS_WRAP_NONE,
					    MESON_CANVAS_BLKMODE_LINEAR, 0);
		} else {
			meson_canvas_config(ge2d->canvas, src1_canvas,
					    src1_addr,
					    ctx->in.pix_mp.plane_fmt[0].bytesperline,
					    ctx->in.pix_mp.height,
					    MESON_CANVAS_WRAP_NONE,
					    MESON_CANVAS_BLKMODE_LINEAR, 0);
		}

		/*
		 * Destination canvas setup:
		 * - NV12/NV21/NV16/NV61: Y canvas (DST1) + CbCr canvas (DST2)
		 * - I420/YV12/YUV422P: Y (DST1) + Cb (DST2) + Cr via extra canvas
		 * - Single-plane: just DST1
		 */
		if (dst_nv) {
			dma_addr_t dst_uv_addr;
			u32 dst_y_size;
			u32 dst_uv_height;

			dst_y_size = ctx->out.pix_mp.plane_fmt[0].bytesperline *
				     ctx->out.pix_mp.height;
			if (ctx->out.fmt->num_planes > 1)
				dst_uv_addr = vb2_dma_contig_plane_dma_addr(
						&ctx->out.buf->vb2_buf, 1);
			else
				dst_uv_addr = dst1_addr + dst_y_size;
			dst_uv_height = ctx->out.pix_mp.height /
					ctx->out.fmt->uv_vdiv;

			if (ctx->blend_mode &&
			    !(ctx->src2_flags & MESON_GE2D_SRC2_FLAG_FILL_COLOR)) {
				meson_canvas_config(ge2d->canvas, src2_canvas,
						    ctx->src2_dma_addr,
						    ctx->src2_bytesperline,
						    ctx->src2_height,
						    MESON_CANVAS_WRAP_NONE,
						    MESON_CANVAS_BLKMODE_LINEAR, 0);
			} else {
				meson_canvas_config(ge2d->canvas, src2_canvas,
						    dst1_addr,
						    ctx->out.pix_mp.plane_fmt[0].bytesperline,
						    ctx->out.pix_mp.height,
						    MESON_CANVAS_WRAP_NONE,
						    MESON_CANVAS_BLKMODE_LINEAR, 0);
			}
			meson_canvas_config(ge2d->canvas, dst1_canvas,
					    dst1_addr,
					    ctx->out.pix_mp.plane_fmt[0].bytesperline,
					    ctx->out.pix_mp.height,
					    MESON_CANVAS_WRAP_NONE,
					    MESON_CANVAS_BLKMODE_LINEAR, 0);

			meson_canvas_config(ge2d->canvas, ctx->dst2_canvas,
					    dst_uv_addr,
					    ctx->out.pix_mp.plane_fmt[0].bytesperline,
					    dst_uv_height,
					    MESON_CANVAS_WRAP_NONE,
					    MESON_CANVAS_BLKMODE_LINEAR, 0);
		} else if (dst_planar) {
			dma_addr_t dst_cb_addr, dst_cr_addr;
			u32 dst_y_size, dst_cb_size;
			u32 dst_uv_stride, dst_uv_height;

			dst_uv_stride = ctx->out.pix_mp.plane_fmt[0].bytesperline /
					ctx->out.fmt->uv_hdiv;
			dst_uv_height = ctx->out.pix_mp.height /
					ctx->out.fmt->uv_vdiv;
			dst_y_size = ctx->out.pix_mp.plane_fmt[0].bytesperline *
				     ctx->out.pix_mp.height;
			dst_cb_size = dst_uv_stride * dst_uv_height;

			if (ctx->out.fmt->fourcc == V4L2_PIX_FMT_YVU420) {
				dst_cr_addr = dst1_addr + dst_y_size;
				dst_cb_addr = dst_cr_addr + dst_cb_size;
			} else {
				dst_cb_addr = dst1_addr + dst_y_size;
				dst_cr_addr = dst_cb_addr + dst_cb_size;
			}

			/* SRC2: from SRC2 buffer when blending, else DST Y plane */
			if (ctx->blend_mode &&
			    !(ctx->src2_flags & MESON_GE2D_SRC2_FLAG_FILL_COLOR)) {
				meson_canvas_config(ge2d->canvas, src2_canvas,
						    ctx->src2_dma_addr,
						    ctx->src2_bytesperline,
						    ctx->src2_height,
						    MESON_CANVAS_WRAP_NONE,
						    MESON_CANVAS_BLKMODE_LINEAR, 0);
			} else {
				meson_canvas_config(ge2d->canvas, src2_canvas,
						    dst1_addr,
						    ctx->out.pix_mp.plane_fmt[0].bytesperline,
						    ctx->out.pix_mp.height,
						    MESON_CANVAS_WRAP_NONE,
						    MESON_CANVAS_BLKMODE_LINEAR, 0);
			}
			meson_canvas_config(ge2d->canvas, dst1_canvas,
					    dst1_addr,
					    ctx->out.pix_mp.plane_fmt[0].bytesperline,
					    ctx->out.pix_mp.height,
					    MESON_CANVAS_WRAP_NONE,
					    MESON_CANVAS_BLKMODE_LINEAR, 0);

			/* DST2 = Cb plane */
			meson_canvas_config(ge2d->canvas, ctx->dst_canvas_cb,
					    dst_cb_addr, dst_uv_stride,
					    dst_uv_height,
					    MESON_CANVAS_WRAP_NONE,
					    MESON_CANVAS_BLKMODE_LINEAR, 0);

			/* Extra canvas for Cr plane */
			meson_canvas_config(ge2d->canvas, ctx->dst_canvas_cr,
					    dst_cr_addr, dst_uv_stride,
					    dst_uv_height,
					    MESON_CANVAS_WRAP_NONE,
					    MESON_CANVAS_BLKMODE_LINEAR, 0);
		} else {
			/* SRC2 canvas: blend from SRC2 buffer, else from DST */
			if (ctx->blend_mode &&
			    !(ctx->src2_flags & MESON_GE2D_SRC2_FLAG_FILL_COLOR)) {
				meson_canvas_config(ge2d->canvas, src2_canvas,
						    ctx->src2_dma_addr,
						    ctx->src2_bytesperline,
						    ctx->src2_height,
						    MESON_CANVAS_WRAP_NONE,
						    MESON_CANVAS_BLKMODE_LINEAR, 0);
			} else {
				meson_canvas_config(ge2d->canvas, src2_canvas,
						    dst1_addr,
						    ctx->out.pix_mp.plane_fmt[0].bytesperline,
						    ctx->out.pix_mp.height,
						    MESON_CANVAS_WRAP_NONE,
						    MESON_CANVAS_BLKMODE_LINEAR, 0);
			}
			meson_canvas_config(ge2d->canvas, dst1_canvas,
					    dst1_addr,
					    ctx->out.pix_mp.plane_fmt[0].bytesperline,
					    ctx->out.pix_mp.height,
					    MESON_CANVAS_WRAP_NONE,
					    MESON_CANVAS_BLKMODE_LINEAR, 0);
		}

		/*
		 * SRC1_CANVAS: bits 31:24 = Y canvas, 23:16 = Cb canvas,
		 *              15:8 = Cr canvas.
		 * For NV: Y + CbCr (Cr slot unused).
		 * For planar: Y + Cb + Cr.
		 * For single-plane: only Y canvas is set.
		 */
		src1_canvas_reg = FIELD_PREP(GE2D_SRC1_CANVAS_ADDR,
					     src1_canvas);
		if (src_nv)
			src1_canvas_reg |= (u32)ctx->src1_canvas_uv << 16;
		else if (src_planar)
			src1_canvas_reg |= (u32)ctx->src1_canvas_cb << 16 |
					   (u32)ctx->src1_canvas_cr << 8;

		regmap_write(ge2d->map, GE2D_SRC1_CANVAS, src1_canvas_reg);

		/*
		 * SRC2_DST_CANVAS: bits 23:16 = DST2 canvas (chroma out),
		 *                  bits 15:8 = SRC2 canvas,
		 *                  bits 7:0 = DST1 canvas (Y out).
		 */
		dst_canvas_reg = FIELD_PREP(GE2D_SRC2_CANVAS_ADDR,
					    src2_canvas) |
				 FIELD_PREP(GE2D_DST1_CANVAS_ADDR,
					    dst1_canvas);
		if (dst_nv)
			dst_canvas_reg |= FIELD_PREP(GE2D_DST2_CANVAS_ADDR,
						     ctx->dst2_canvas);
		else if (dst_planar)
			dst_canvas_reg = FIELD_PREP(GE2D_DST2_CANVAS_ADDR,
						    ctx->dst_canvas_cr) |
					 FIELD_PREP(GE2D_SRC2_CANVAS_ADDR,
						    ctx->dst_canvas_cb) |
					 FIELD_PREP(GE2D_DST1_CANVAS_ADDR,
						    dst1_canvas);

		regmap_write(ge2d->map, GE2D_SRC2_DST_CANVAS, dst_canvas_reg);
	}

	/*
	 * GEN_CTRL0: SRC1 YC ratio flags and separate enable.
	 * Bits 10-11 control the SRC1 chroma formatter despite their
	 * position in the register (confirmed by BSP ge2d_hw.c:418-420).
	 * SRC1_X_YC_RATIO: source has horizontal chroma subsampling.
	 * SRC1_Y_YC_RATIO: source has vertical chroma subsampling.
	 * SRC1_SEP_EN: separate canvas for chroma (semi-planar/planar).
	 */
	reg = src_x_yc_ratio ? GE2D_SRC1_X_YC_RATIO : 0;
	if (src_nv || src_planar)
		reg |= GE2D_SRC1_SEP_EN;
	if (src_y_yc_ratio)
		reg |= GE2D_SRC1_Y_YC_RATIO;
	/* Color keying: enable SRC1 key when mask is non-zero */
	if (ctx->colorkey_mask) {
		reg |= GE2D_SRC1_KEY_EN;
		regmap_write(ge2d->map, GE2D_SRC1_KEY, ctx->colorkey);
		regmap_write(ge2d->map, GE2D_SRC1_KEY_MASK, ctx->colorkey_mask);
	}

	/* Fill mode: use constant color instead of source buffer */
	if (ctx->fillcolor) {
		reg |= GE2D_SRC1_FILL_MODE;
		regmap_write(ge2d->map, GE2D_SRC1_DEF_COLOR, ctx->fillcolor);
	}

	/* Bitmask: selective per-pixel write control */
	if (ctx->bitmask) {
		reg |= GE2D_DST_BITMASK_EN;
		regmap_write(ge2d->map, GE2D_DST_BITMASK, ctx->bitmask);
	}

	/*
	 * CLUT8 (F-10): when SRC1 format is PAL8, engage the 256-entry
	 * ARGB8888 LUT.  GEN_CTRL2 below programs GE2D_FORMAT_8BIT for
	 * SRC1 (via fmt->hw_fmt) so the engine reads one byte per pixel,
	 * then substitutes the 32-bit LUT entry before CSC / scaler / ALU.
	 */
	if (src_pal8)
		reg |= GE2D_SRC1_LUT_EN;

	regmap_write(ge2d->map, GE2D_GEN_CTRL0, reg);

	/*
	 * Upload the CLUT8 palette after enabling LUT_EN.  The write
	 * pointer is reset by writing GE2D_SRC1_LUT_ADDR = 0 (GE2D_LUT_READ
	 * bit clear = write mode, address 0); the 256 data writes then
	 * auto-increment through the table.  ctx->clut8[] is zero-padded
	 * for counts < 256, so the full 256 entries always have defined
	 * contents.
	 */
	if (src_pal8) {
		unsigned int i;

		regmap_write(ge2d->map, GE2D_SRC1_LUT_ADDR, 0);
		for (i = 0; i < 256; i++)
			regmap_write(ge2d->map, GE2D_SRC1_LUT_DAT,
				     ctx->clut8[i]);
	}

	/* Anti-flicker filter: BSP ge2d_hw.c:807-880 default coefficients */
	if (ctx->antiflicker) {
		regmap_write(ge2d->map, GE2D_ANTIFLICK_CTRL0, 0x80000000);
		regmap_write(ge2d->map, GE2D_ANTIFLICK_CTRL1,
			     (1 << 25) | (1 << 24) | (0x42 << 16) | (0x81 << 8) | 0x19);
		regmap_write(ge2d->map, GE2D_ANTIFLICK_COLOR_FILT0,
			     (8 << 24) | (0 << 16) | (128 << 8) | 0);
		regmap_write(ge2d->map, GE2D_ANTIFLICK_COLOR_FILT1,
			     (16 << 24) | (16 << 16) | (96 << 8) | 16);
		regmap_write(ge2d->map, GE2D_ANTIFLICK_COLOR_FILT2,
			     (64 << 24) | (32 << 16) | (64 << 8) | 32);
		regmap_write(ge2d->map, GE2D_ANTIFLICK_COLOR_FILT3,
			     (32 << 16) | (64 << 8) | 32);
		regmap_write(ge2d->map, GE2D_ANTIFLICK_ALPHA_FILT0,
			     (8 << 24) | (0 << 16) | (128 << 8) | 0);
		regmap_write(ge2d->map, GE2D_ANTIFLICK_ALPHA_FILT1,
			     (16 << 24) | (16 << 16) | (96 << 8) | 16);
		regmap_write(ge2d->map, GE2D_ANTIFLICK_ALPHA_FILT2,
			     (64 << 24) | (32 << 16) | (64 << 8) | 32);
		regmap_write(ge2d->map, GE2D_ANTIFLICK_ALPHA_FILT3,
			     (32 << 16) | (64 << 8) | 32);
	} else {
		regmap_write(ge2d->map, GE2D_ANTIFLICK_CTRL0, 0);
	}


	/* Field interlacing (F-11): pic_struct controls field traversal */
	{
		u32 src_ps = 0, dst_ps = 0;
		switch (ctx->in.pix_mp.field) {
		case V4L2_FIELD_TOP: case V4L2_FIELD_INTERLACED_TB:
			src_ps = 1; break;
		case V4L2_FIELD_BOTTOM: case V4L2_FIELD_INTERLACED_BT:
			src_ps = 2; break;
		default: break;
		}
		switch (ctx->out.pix_mp.field) {
		case V4L2_FIELD_TOP: case V4L2_FIELD_INTERLACED_TB:
			dst_ps = 1; break;
		case V4L2_FIELD_BOTTOM: case V4L2_FIELD_INTERLACED_BT:
			dst_ps = 2; break;
		default: break;
		}
		regmap_update_bits(ge2d->map, GE2D_GEN_CTRL0,
				   GENMASK(2, 1), src_ps << 1);
		regmap_update_bits(ge2d->map, GE2D_GEN_CTRL1,
				   GENMASK(15, 14), dst_ps << 14);
	}
	regmap_write(ge2d->map, GE2D_GEN_CTRL1,
		     FIELD_PREP(GE2D_INTERRUPT_CTRL, 2) |
		     FIELD_PREP(GE2D_SRC2_BURST_SIZE_CTRL, 3) |
		     FIELD_PREP(GE2D_SRC1_BURST_SIZE_CTRL, 0x3f) |
		     FIELD_PREP(GE2D_SRC1_GB_ALPHA, ctx->alpha));

	/*
	 * GEN_CTRL2: format and color map for all ports.
	 * BSP: NV12 dst uses format=0 (8-bit), NV12 src uses format=2 (24-bit).
	 * Planar dst also uses format=0 (8-bit per canvas).
	 * SRC2 format depends on blend_mode:
	 *   blend_mode=true: SRC2 format from ctx->src2_fmt (the user's buffer)
	 *   blend_mode=false: SRC2 reads DST for readback, so same format as DST.
	 */
	regmap_write(ge2d->map, GE2D_GEN_CTRL2,
		     /*
		      * deep_color: bit 2 enables 10/12-bit SRC pipeline.
		      * BSP sets this from the SRC format only (ge2dgen.c:32).
		      * DST deep_color formats (MESON_Y10P444 etc.) do not
		      * require this bit -- their 10/12-bit packing comes from
		      * the color_map value, not the pipeline width.
		      * Setting deep_color=1 with an 8-bit SRC corrupts the
		      * input interpretation and produces garbage CSC output
		      * (probed 2026-04-23 on G12B).
		      */
		     (ctx->in.fmt->deep_color ? BIT(2) : 0) |
		     /*
		      * SRC1 endian selects the packed-component byte order. The BSP
		      * encodes it per format: GE2D_FMT_S16_YUV422 = 0x20102 has the
		      * endian bit (24) CLEAR (BIG_ENDIAN), while GE2D_FMT_S16_RGB =
		      * GE2D_LITTLE_ENDIAN|0x100 (libge2d ge2d.h:336-337). i.e. ALL
		      * packed YUV reads BIG; only packed RGB reads LITTLE. The old
		      * (src_nv||src_planar) gate left packed YUYV (depth-16 YUV) on
		      * LITTLE, which swapped the Y<->chroma byte lanes -- the hw read
		      * memory-YUYV [Y,Cb,Y,Cr] as UYVY [Cb,Y,Cr,Y], so black came out
		      * NV12 Y=128,U=V=16 and decoded to pure green (KI 2026-06-06).
		      * All YUV (nv/planar/packed) -> BIG (0); RGB -> LITTLE. Deep-color
		      * YUV422 (10/12-bit placeholder formats) keeps its prior endian.
		      */
		     ((src_yuv && !ctx->in.fmt->deep_color) ? 0 :
		      GE2D_SRC1_LITTLE_ENDIAN) |
		     (ctx->blend_mode ?
			GE2D_SRC2_LITTLE_ENDIAN :
			((dst_nv || dst_planar) ? 0 : GE2D_SRC2_LITTLE_ENDIAN)) |
		     ((dst_nv || dst_planar) ? 0 : GE2D_DST_LITTLE_ENDIAN) |
		     (ctx->alpha < 255 ? GE2D_SRC1_GB_ALPHA_EN : 0) |
		     GE2D_SRC2_COLOR_EXPAND_MODE |
		     GE2D_SRC1_COLOR_EXPAND_MODE |
		     FIELD_PREP(GE2D_DST1_COLOR_MAP, ctx->out.fmt->hw_map) |
		     FIELD_PREP(GE2D_DST1_FORMAT,
				(dst_nv || dst_planar) ? GE2D_FORMAT_8BIT :
				ctx->out.fmt->hw_fmt) |
		     FIELD_PREP(GE2D_SRC2_COLOR_MAP,
				(ctx->blend_mode && ctx->src2_fmt) ?
				ctx->src2_fmt->hw_map :
				ctx->out.fmt->hw_map) |
		     FIELD_PREP(GE2D_SRC2_FORMAT,
				(ctx->blend_mode && ctx->src2_fmt) ?
				ctx->src2_fmt->hw_fmt :
				(dst_nv || dst_planar) ? GE2D_FORMAT_8BIT :
				ctx->out.fmt->hw_fmt) |
		     FIELD_PREP(GE2D_SRC1_COLOR_MAP, ctx->in.fmt->hw_map) |
		     FIELD_PREP(GE2D_SRC1_FORMAT, ctx->in.fmt->hw_fmt));

	/* GEN_CTRL3: DST1 enable + DST2 for multi-plane output */
	if (dst_nv) {
		u32 dst2_map = ctx->out.fmt->hw_map >= 5 ?
			       ctx->out.fmt->hw_map - 5 : 0;
		u32 x_discard = 3;  /* NV420: discard every other sample */
		u32 y_discard = (ctx->out.fmt->uv_vdiv == 2) ? 3 : 0;

		regmap_write(ge2d->map, GE2D_GEN_CTRL3,
			     GE2D_DST1_ENABLE |
			     GE2D_DST2_ENABLE |
			     FIELD_PREP(GE2D_DST2_X_DISCARD_MODE, x_discard) |
			     FIELD_PREP(GE2D_DST2_Y_DISCARD_MODE, y_discard) |
			     FIELD_PREP(GE2D_DST2_COLOR_MAP, dst2_map) |
			     FIELD_PREP(GE2D_DST2_FORMAT, GE2D_FORMAT_16BIT));
	} else if (dst_planar) {
		/*
		 * Planar output: DST1 = Y, DST2 = Cb.
		 * Cr plane is handled via the third canvas (BSP uses
		 * 3-canvas mode where the hardware writes Y/Cb/Cr to
		 * three separate canvas addresses).
		 * X discard = 3 (horizontal 2:1 subsampling for Cb/Cr).
		 * Y discard depends on vertical subsampling.
		 */
		u32 y_discard = (ctx->out.fmt->uv_vdiv == 2) ? 3 : 0;

		regmap_write(ge2d->map, GE2D_GEN_CTRL3,
			     GE2D_DST1_ENABLE |
			     GE2D_DST2_ENABLE |
			     FIELD_PREP(GE2D_DST2_X_DISCARD_MODE, 3) |
			     FIELD_PREP(GE2D_DST2_Y_DISCARD_MODE, y_discard) |
			     FIELD_PREP(GE2D_DST2_FORMAT, GE2D_FORMAT_8BIT));
	} else {
		regmap_write(ge2d->map, GE2D_GEN_CTRL3,
			     GE2D_DST1_ENABLE);
	}

	/*
	 * Colorspace conversion: select BT.601 or BT.709 matrix based
	 * on the source colorspace.  For YUV->RGB, the source YUV encoding
	 * determines the matrix.  For RGB->YUV, the destination encoding
	 * determines the matrix (the source is the YUV side for the
	 * inverse transform, so check the capture format).
	 */
	if (src_yuv && !dst_yuv)
		ge2d_set_csc_matrix(ge2d, true,
				    ge2d_is_bt709(&ctx->in.pix_mp));
	else if (!src_yuv && dst_yuv)
		ge2d_set_csc_matrix(ge2d, false,
				    ge2d_is_bt709(&ctx->out.pix_mp));
	else
		regmap_write(ge2d->map, GE2D_MATRIX_COEF22_CTRL, 0);

	/*
	 * SRC1 chroma formatter: only enable horizontal/vertical formatters
	 * when the source actually has subsampled chroma.  The BSP gates
	 * HFMT_EN on x_yc_ratio (ge2d_hw.c:447-452).  Enabling HFMT_EN
	 * without x_yc_ratio set in GEN_CTRL0 stalls the SRC1 reader.
	 */
	if (src_x_yc_ratio) {
		u32 fmt_ctrl;

		/*
		 * X_CHR_PHASE: horizontal chroma-formatter phase, chosen by the BSP
		 * per destination type (ge2d_hw.c:1157-1205, non-reversed): YUV->RGB
		 * uses 0xc4 (:1167), YUV->YUV uses 0x08 (:1190). The driver hardcoded
		 * 0xc4 for both -- benign for semi-planar sources (chroma in its own
		 * plane) but WRONG for PACKED YUYV (chroma interleaved with luma): the
		 * wrong phase mis-extracts the interleaved lanes so the hw reads YUYV
		 * [Y,Cb,Y,Cr] as UYVY [Cb,Y,Cr,Y] and black decodes to pure green
		 * (NV12 Y=128,U=V=16). Use 0x08 for YUV->YUV, 0xc4 for YUV->RGB.
		 */
		fmt_ctrl = GE2D_SRC1_HFMT_EN |
			   FIELD_PREP(GE2D_SRC1_X_CHR_PHASE, dst_yuv ? 0x08 : 0xc4);

		/*
		 * Multi-plane with vertical subsampling (4:2:0):
		 * enable vertical chroma upsampling.
		 * BSP (ge2d_hw.c:1170-1178) sets y_chr_phase to 0xc4
		 * for YUV->RGB with y_yc_ratio (non-reversed case).
		 * For YUV->YUV it uses 0x4c (ge2d_hw.c:1194-1202).
		 */
		if (src_y_yc_ratio) {
			u8 y_phase = !dst_yuv ? 0xc4 : 0x4c;

			fmt_ctrl |= GE2D_SRC1_VFMT_EN |
				    FIELD_PREP(GE2D_SRC1_Y_CHR_PHASE, y_phase);
		}

		if (ctx->in.crop.width == ctx->out.crop.width &&
		    ctx->in.crop.height == ctx->out.crop.height) {
			fmt_ctrl |= GE2D_SRC1_CHFMT_RPT_PIX |
				    GE2D_SRC1_CVFMT_RPT_PIX;
		}

		regmap_write(ge2d->map, GE2D_SRC1_FMT_CTRL, fmt_ctrl);
	} else {
		regmap_write(ge2d->map, GE2D_SRC1_FMT_CTRL, 0);
	}

	regmap_write(ge2d->map, GE2D_SRC1_CLIPY_START_END,
		     FIELD_PREP(GE2D_START, ctx->in.crop.top) |
		     FIELD_PREP(GE2D_END, ctx->in.crop.top + ctx->in.crop.height - 1));
	regmap_write(ge2d->map, GE2D_SRC1_CLIPX_START_END,
		     FIELD_PREP(GE2D_START, ctx->in.crop.left) |
		     FIELD_PREP(GE2D_END, ctx->in.crop.left + ctx->in.crop.width - 1));
	regmap_write(ge2d->map, GE2D_SRC2_CLIPY_START_END,
		     FIELD_PREP(GE2D_START, ctx->out.crop.top) |
		     FIELD_PREP(GE2D_END, ctx->out.crop.top + ctx->out.crop.height - 1));
	regmap_write(ge2d->map, GE2D_SRC2_CLIPX_START_END,
		     FIELD_PREP(GE2D_START, ctx->out.crop.left) |
		     FIELD_PREP(GE2D_END, ctx->out.crop.left + ctx->out.crop.width - 1));
	regmap_write(ge2d->map, GE2D_DST_CLIPY_START_END,
		     FIELD_PREP(GE2D_START, ctx->out.crop.top) |
		     FIELD_PREP(GE2D_END, ctx->out.crop.top + ctx->out.crop.height - 1));
	regmap_write(ge2d->map, GE2D_DST_CLIPX_START_END,
		     FIELD_PREP(GE2D_START, ctx->out.crop.left) |
		     FIELD_PREP(GE2D_END, ctx->out.crop.left + ctx->out.crop.width - 1));

	/*
	 * SRC1_X/Y_START_END is the SRC1 *read window* and MUST be the
	 * cropped source rectangle, not the full frame -- the polyphase
	 * scaler input dimensions (widthi/heighti below) are derived from
	 * ctx->in.crop, so the read-window extents have to agree with them.
	 * The BSP programs this register from cfg->src1_{x,y}_{start,end}
	 * (the crop rect) and derives the scaler input from the same
	 * start/end (ge2d_hw.c:1206-1218 + 1222-1223: widthi =
	 * src1_x_end - src1_x_start + 1). Upstream previously wrote
	 * START=0 / END=pix_mp.{width,height}-1 (the full frame), which is
	 * self-consistent only when the crop origin is 0. With a non-zero
	 * S_SELECTION crop origin *and* a scale, the SRC1 reader is told to
	 * fetch the whole frame while the scaler is configured for the
	 * smaller cropped window -- the SRC1 read command then stalls
	 * (STATUS0.BUSY=1, STATUS1.SRC1_CMD_BUSY=1, no completion edge),
	 * the true crop-offset+scale hang (GI-022, GE2D ge2d/known-issues.md).
	 *
	 * Y end gets the BSP's +1 expansion when y_yc_ratio is set
	 * (ge2d_hw.c:1144 expands src1_y_end by one line so the vertical
	 * chroma formatter has an extra line to interpolate); the
	 * y_extra_bit_start=2 / y_extra_bit_end=3 chroma-addressing bits
	 * are BSP ge2d_hw.c:1170-1179/1194-1203 (non-reversed 4:2:0).
	 */
	regmap_write(ge2d->map, GE2D_SRC1_Y_START_END,
		     (src_y_yc_ratio ? (2 << 30) : 0) |
		     FIELD_PREP(GE2D_START, ctx->in.crop.top) |
		     FIELD_PREP(GE2D_END,
				src_y_yc_ratio ?
					ctx->in.crop.top + ctx->in.crop.height :
					ctx->in.crop.top + ctx->in.crop.height - 1) |
		     (src_y_yc_ratio ? (3 << 14) : 0));
	/*
	 * SRC1_X_START_END: same crop-window rule. The x_extra_bit_start=2 /
	 * x_extra_bit_end=3 chroma-addressing bits are set whenever the source
	 * has horizontal chroma subsampling -- the BSP sets them in BOTH the
	 * YUV->RGB branch (ge2d_hw.c:1165-1166) AND the YUV->YUV branch
	 * (:1188-1189) for the non-reversed case. The earlier `!dst_yuv` gate
	 * (YUV->RGB only) missed the YUV->YUV branch, so for PACKED YUYV->NV12
	 * the chroma extraction was mis-addressed: the hw read YUYV [Y,Cb,Y,Cr]
	 * as UYVY [Cb,Y,Cr,Y] and black decoded to pure green. (Benign for
	 * semi-planar sources, where chroma is in a separate plane.)
	 */
	regmap_write(ge2d->map, GE2D_SRC1_X_START_END,
		     (src_x_yc_ratio ? (2 << 30) : 0) |
		     FIELD_PREP(GE2D_START, ctx->in.crop.left) |
		     FIELD_PREP(GE2D_END,
				ctx->in.crop.left + ctx->in.crop.width - 1) |
		     (src_x_yc_ratio ? (3 << 14) : 0));
	regmap_write(ge2d->map, GE2D_SRC2_Y_START_END,
		     FIELD_PREP(GE2D_END, ctx->out.pix_mp.height - 1));
	regmap_write(ge2d->map, GE2D_SRC2_X_START_END,
		     FIELD_PREP(GE2D_END, ctx->out.pix_mp.width - 1));
	regmap_write(ge2d->map, GE2D_DST_Y_START_END,
		     FIELD_PREP(GE2D_END, ctx->out.pix_mp.height - 1));
	regmap_write(ge2d->map, GE2D_DST_X_START_END,
		     FIELD_PREP(GE2D_END, ctx->out.pix_mp.width - 1));

	/* Scaler setup */
	widthi  = ctx->in.crop.width;
	heighti = ctx->in.crop.height;
	widtho  = ctx->xy_swap ? ctx->out.crop.height : ctx->out.crop.width;
	heighto = ctx->xy_swap ? ctx->out.crop.width : ctx->out.crop.height;

	if (widthi != widtho || heighti != heighto) {
		u32 sc_prehsc_en, sc_prevsc_en;
		u32 tmp_widthi, tmp_heighti;
		u32 hsc_phase_step, vsc_phase_step;
		u32 hsc_ini_phase, vsc_ini_phase;
		u32 hsc_rpt_p0_num, vsc_rpt_l0_num;
		u32 rate_w, rate_h;
		u32 hsc_div_length_val;
		u32 hsc_adv_num, hsc_adv_phase;

		/*
		 * The polyphase scaler cannot decimate: a source step > 1.0
		 * deadlocks the blit (the m2m device wedges, needs a reset).
		 * The pre-decimator (/2) was gated on >2x only, leaving every
		 * 1x-2x downscale to drive the polyphase at step > 1 -> stall.
		 * Force the pre-decimator for ANY downscale so the polyphase
		 * always sees step <= 1.0.  (A single /2 still leaves >2x at
		 * step > 1; that remains unsupported -- it needs cascaded
		 * decimation -- but it already stalled before this change.)
		 */
		sc_prehsc_en = (widthi > widtho) ? 1 : 0;
		sc_prevsc_en = (heighti > heighto) ? 1 : 0;

		tmp_widthi = sc_prehsc_en ? ((widthi + 1) >> 1) : widthi;
		tmp_heighti = sc_prevsc_en ? ((heighti + 1) >> 1) : heighti;

		hsc_phase_step = (u32)(((u64)tmp_widthi << 18) / widtho) << 6;
		vsc_phase_step = (u32)(((u64)tmp_heighti << 18) / heighto) << 6;

		/*
		 * Horizontal scaler divider + advance controller.  Required
		 * by the pipeline whenever HSC is active: the pre-decimator
		 * stage needs a divider length so its output rate matches
		 * the polyphase kernel input rate.  Without this, >2x
		 * downscale stalls the scaler on GXL (KI-2026-04-24-B).
		 *
		 * BSP ge2d_hw.c:1246-1264 formulas:
		 *   hsc_div_length = (124 << 24) / hsc_phase_step (G12A+)
		 *                  = (120 << 24) / hsc_phase_step (pre-G12A)
		 *   multo          = hsc_phase_step * hsc_div_length
		 *   hsc_adv_num    = multo >> 24
		 *   hsc_adv_phase  = multo & 0xffffff
		 *
		 * We use 124 unconditionally: the two BSP-gated cases
		 * (pre-G12A and two C2-family SoCs) are either out of
		 * scope or non-GE2D-upstream targets.
		 */
		{
			u64 multo;
			u32 hsc_div_length = (124U << 24) / hsc_phase_step;

			hsc_div_length_val = hsc_div_length;
			multo = (u64)hsc_phase_step * hsc_div_length;
			hsc_adv_num = multo >> 24;
			hsc_adv_phase = multo & 0xffffff;
		}

		rate_w = (widtho * 10) / widthi;
		rate_h = (heighto * 10) / heighti;

		if (rate_h < 10) {
			vsc_rpt_l0_num = 1;
			vsc_ini_phase = rate_h ? (0x5000000 / rate_h - 0x800000) : 0x5000000;
		} else {
			vsc_rpt_l0_num = 2;
			vsc_ini_phase = 0x800000 + 0x5000000 / rate_h;
		}
		vsc_en = 1;

		if (rate_w < 10) {
			hsc_rpt_p0_num = 1;
			hsc_ini_phase = rate_w ? (0x5000000 / rate_w - 0x800000) : 0x5000000;
		} else {
			hsc_rpt_p0_num = 2;
			hsc_ini_phase = 0x800000 + 0x5000000 / rate_w;
		}
		hsc_en = 1;

		ge2d_load_scale_coef(ge2d, filt_coef_tables[ctx->scale_filter]);

		regmap_write(ge2d->map, GE2D_SC_MISC_CTRL,
			     (hsc_en ? GE2D_SC_HSC_DIV_EN : 0) |
			     FIELD_PREP(GE2D_SC_HSC_DIV_LENGTH, hsc_div_length_val) |
			     FIELD_PREP(GE2D_SC_PREHSC_EN, sc_prehsc_en) |
			     FIELD_PREP(GE2D_SC_PREVSC_EN, sc_prevsc_en) |
			     (vsc_en ? GE2D_SC_VSC_EN : 0) |
			     (hsc_en ? GE2D_SC_HSC_EN : 0) |
			     FIELD_PREP(GE2D_SC_VSC_BANK_LENGTH, 4) |
			     FIELD_PREP(GE2D_SC_HSC_BANK_LENGTH, 4));

		regmap_write(ge2d->map, GE2D_HSC_START_PHASE_STEP, hsc_phase_step);
		regmap_write(ge2d->map, GE2D_HSC_PHASE_SLOPE, 0);
		regmap_write(ge2d->map, GE2D_HSC_INI_CTRL,
			     (hsc_rpt_p0_num << 29) |
			     (hsc_ini_phase & 0xffffff));
		regmap_write(ge2d->map, GE2D_HSC_ADV_CTRL,
			     FIELD_PREP(GE2D_HSC_ADV_NUM, hsc_adv_num & 0xff) |
			     FIELD_PREP(GE2D_HSC_ADV_PHASE, hsc_adv_phase));

		regmap_write(ge2d->map, GE2D_VSC_START_PHASE_STEP, vsc_phase_step);
		regmap_write(ge2d->map, GE2D_VSC_PHASE_SLOPE, 0);
		regmap_write(ge2d->map, GE2D_VSC_INI_CTRL,
			     (vsc_rpt_l0_num << 29) |
			     (vsc_ini_phase & 0xffffff));
		regmap_write(ge2d->map, GE2D_VSC_NRND_POINT, 0);
		regmap_write(ge2d->map, GE2D_VSC_NRND_PHASE, 0);
		regmap_write(ge2d->map, GE2D_HSC_NRND_POINT, 0);
		regmap_write(ge2d->map, GE2D_HSC_NRND_PHASE, 0);
	} else {
		regmap_write(ge2d->map, GE2D_SC_MISC_CTRL, 0);
		regmap_write(ge2d->map, GE2D_HSC_START_PHASE_STEP, 0);
		regmap_write(ge2d->map, GE2D_VSC_START_PHASE_STEP, 0);
		regmap_write(ge2d->map, GE2D_HSC_INI_CTRL, 0);
		regmap_write(ge2d->map, GE2D_VSC_INI_CTRL, 0);
		regmap_write(ge2d->map, GE2D_HSC_ADV_CTRL, 0);
		regmap_write(ge2d->map, GE2D_VSC_NRND_POINT, 0);
		regmap_write(ge2d->map, GE2D_VSC_NRND_PHASE, 0);
		regmap_write(ge2d->map, GE2D_HSC_NRND_POINT, 0);
		regmap_write(ge2d->map, GE2D_HSC_NRND_PHASE, 0);
	}

	/*
	 * ALU blending setup.
	 *
	 * Three modes, in priority order:
	 * 1. blend_mode (SRC2 ioctl): user-defined ALU factors from blend_op
	 * 2. alpha < 255: Porter-Duff source-over with DST readback
	 * 3. alpha == 255: logic operation (copy/clear/xor/and/or)
	 */
	if (ctx->blend_mode) {
		/* SRC2 compositing: parse blend_op into ALU factors */
		u32 color_mode  = (ctx->blend_op >> 24) & 0x7;
		u32 color_src_f = (ctx->blend_op >> 20) & 0xf;
		u32 color_dst_f = (ctx->blend_op >> 16) & 0xf;
		u32 alpha_mode  = (ctx->blend_op >> 8) & 0x7;
		u32 alpha_src_f = (ctx->blend_op >> 4) & 0xf;
		u32 alpha_dst_f = (ctx->blend_op >> 0) & 0xf;

		if (color_mode == OPERATION_LOGIC)
			reg = GE2D_ALU_DO_COLOR_OPERATION_LOGIC(
				color_dst_f, color_src_f);
		else
			reg = GE2D_ALU_COLOR_OP(color_mode,
						color_src_f, color_dst_f);

		if (alpha_mode == OPERATION_LOGIC)
			reg |= GE2D_ALU_DO_ALPHA_OPERATION_LOGIC(
				alpha_dst_f, alpha_src_f);
		else
			reg |= GE2D_ALU_ALPHA_OP(alpha_mode,
						  alpha_src_f, alpha_dst_f);

		/*
		 * Premultiplication control:
		 * FLAG_SRC1_PREMULTIPLIED=0 (default): hardware premultiplies
		 *   SRC1 color by SRC1 alpha (cmult_asel=1).
		 * FLAG_SRC1_PREMULTIPLIED=1: skip premultiply (cmult_asel=0).
		 */
		if (!(ctx->src2_flags & MESON_GE2D_SRC2_FLAG_SRC1_PREMULTIPLIED))
			reg |= FIELD_PREP(GE2D_SRC1_COLOR_MULT_ALPHA_SEL, 1);

		regmap_write(ge2d->map, GE2D_ALU_CONST_COLOR, ctx->const_color);
	} else if (ctx->alpha < 255) {
		/* Porter-Duff source-over: Cd = Cs*As + Cd*(1-As) */
		reg = GE2D_ALU_COLOR_OP(OPERATION_ADD,
					COLOR_FACTOR_SRC_ALPHA,
					COLOR_FACTOR_ONE_MINUS_SRC_ALPHA);
		reg |= GE2D_ALU_ALPHA_OP(OPERATION_ADD,
					  ALPHA_FACTOR_ONE,
					  ALPHA_FACTOR_ONE_MINUS_SRC_ALPHA);
		/*
		 * Premultiply source color by source alpha for correct
		 * Porter-Duff compositing (BSP ge2d_hw.c uses
		 * SRC1_COLOR_MULT_ALPHA_SEL = 1 for blend operations).
		 */
		reg |= FIELD_PREP(GE2D_SRC1_COLOR_MULT_ALPHA_SEL, 1);
	} else {
		/*
		 * No blending (alpha == 255).  Apply the selected logic
		 * operation.  For COPY (default) and CLEAR/SET/XOR/AND/OR the
		 * color channel uses the requested logic op.  The alpha channel
		 * always preserves or fills correctly regardless of the logic op.
		 */
		u8 hw_op = ge2d_logic_op_hw[ctx->logic_op];

		reg = GE2D_ALU_DO_COLOR_OPERATION_LOGIC(
			hw_op, COLOR_FACTOR_SRC_COLOR);

		if (ctx->in.fmt->alpha && ctx->out.fmt->alpha)
			/* Take source alpha */
			reg |= GE2D_ALU_DO_ALPHA_OPERATION_LOGIC(
				LOGIC_OPERATION_COPY,
				COLOR_FACTOR_SRC_ALPHA);
		else if (!ctx->out.fmt->alpha)
			/* Set alpha to 0 */
			reg |= GE2D_ALU_DO_ALPHA_OPERATION_LOGIC(
				LOGIC_OPERATION_SET, COLOR_FACTOR_ZERO);
		else
			/* Keep original alpha */
			reg |= GE2D_ALU_DO_ALPHA_OPERATION_LOGIC(
				LOGIC_OPERATION_COPY,
				COLOR_FACTOR_DST_ALPHA);
	}

	regmap_write(ge2d->map, GE2D_ALU_OP_CTRL, reg);

	/* Step 8: SRC2 global alpha (G12B/SM1+ only) */
	if (ctx->blend_mode && ctx->src2_gb_alpha > 0 && ge2d->has_src2_alp)
		regmap_update_bits(ge2d->map, GE2D_GEN_CTRL5,
				   GENMASK(8, 0),
				   BIT(8) | (ctx->src2_gb_alpha & 0xff));

	/* Step 9: SRC2 coordinates when blending */
	if (ctx->blend_mode) {
		u32 dst_w = ctx->out.pix_mp.width;
		u32 dst_h = ctx->out.pix_mp.height;
		u32 out_w = ctx->xy_swap ? dst_h : dst_w;
		u32 out_h = ctx->xy_swap ? dst_w : dst_h;
		u32 src2_read_w = out_w >> ctx->src2_repeat_x;
		u32 src2_read_h = out_h >> ctx->src2_repeat_y;
		u32 src2_x_start = (u32)ctx->src2_x_offset;
		u32 src2_y_start = (u32)ctx->src2_y_offset;
		u32 src2_x_end = src2_x_start + src2_read_w - 1;
		u32 src2_y_end = src2_y_start + src2_read_h - 1;

		/* Clamp to SRC2 surface bounds */
		if (src2_x_end >= ctx->src2_width)
			src2_x_end = ctx->src2_width - 1;
		if (src2_y_end >= ctx->src2_height)
			src2_y_end = ctx->src2_height - 1;

		regmap_write(ge2d->map, GE2D_SRC2_X_START_END,
			     FIELD_PREP(GE2D_START, src2_x_start) |
			     FIELD_PREP(GE2D_END, src2_x_end));
		regmap_write(ge2d->map, GE2D_SRC2_Y_START_END,
			     FIELD_PREP(GE2D_START, src2_y_start) |
			     FIELD_PREP(GE2D_END, src2_y_end));
		regmap_write(ge2d->map, GE2D_SRC2_CLIPX_START_END,
			     FIELD_PREP(GE2D_START, src2_x_start) |
			     FIELD_PREP(GE2D_END, src2_x_end));
		regmap_write(ge2d->map, GE2D_SRC2_CLIPY_START_END,
			     FIELD_PREP(GE2D_START, src2_y_start) |
			     FIELD_PREP(GE2D_END, src2_y_end));

		/* SRC2 default color (always written in blend mode) */
		regmap_write(ge2d->map, GE2D_SRC2_DEF_COLOR, ctx->def_color);

		/* SRC2 color key */
		if (ctx->src2_color_key_mask) {
			regmap_write(ge2d->map, GE2D_SRC2_KEY,
				     ctx->src2_color_key);
			regmap_write(ge2d->map, GE2D_SRC2_KEY_MASK,
				     ctx->src2_color_key_mask);
			regmap_update_bits(ge2d->map, GE2D_GEN_CTRL0,
					   BIT(29), BIT(29));
		} else {
			regmap_update_bits(ge2d->map, GE2D_GEN_CTRL0,
					   BIT(29), 0);
		}
	}

	/* Start -- CMD_CTRL with blend-mode flags */
	{
		u32 cmd = (ctx->xy_swap ? GE2D_DST_XY_SWAP : 0) |
			  (ctx->hflip ? GE2D_SRC1_X_REV : 0) |
			  (ctx->vflip ? GE2D_SRC1_Y_REV : 0) |
			  GE2D_CBUS_CMD_WR;

		if (ctx->blend_mode) {
			cmd |= (ctx->src2_repeat_x & 0x3) << 12;
			cmd |= (ctx->src2_repeat_y & 0x3) << 10;
			if (ctx->src2_flags & MESON_GE2D_SRC2_FLAG_FILL_COLOR)
				cmd |= GE2D_SRC2_FILL_COLOR_EN;
			if (ctx->src2_flags & MESON_GE2D_SRC2_FLAG_SRC2_X_REV)
				cmd |= GE2D_SRC2_X_REV;
			if (ctx->src2_flags & MESON_GE2D_SRC2_FLAG_SRC2_Y_REV)
				cmd |= GE2D_SRC2_Y_REV;
		}

		regmap_write(ge2d->map, GE2D_CMD_CTRL, cmd);
	}

	return 0;
}

/*
 * Watchdog work: runs in process context when a V4L2 blit's completion IRQ
 * never fired (ge2d_job_timeout expired).  Claim the in-flight job under
 * hw_lock (so we don't double-finish a job the ISR is completing on another
 * CPU at the same instant), reset the IP in place, then error out the stuck
 * buffers and finish the job so DQBUF returns an errored buffer instead of
 * hanging forever and the next job can run.  This mirrors device_run()'s
 * start-failure recovery exactly.
 */
static void ge2d_job_timeout_work(struct work_struct *work)
{
	struct meson_ge2d *ge2d =
		container_of(work, struct meson_ge2d, job_timeout_work);
	struct vb2_v4l2_buffer *src, *dst;
	struct ge2d_ctx *ctx;
	unsigned long flags;

	spin_lock_irqsave(&ge2d->hw_lock, flags);
	ctx = ge2d->curr;
	if (!ctx) {
		/* The completion IRQ won the race and already finished the job. */
		spin_unlock_irqrestore(&ge2d->hw_lock, flags);
		return;
	}
	ge2d->curr = NULL;
	spin_unlock_irqrestore(&ge2d->hw_lock, flags);

	dev_err(ge2d->dev, "GE2D V4L2 blit timeout -- reset\n");

	/*
	 * Reset the IP in place so a wedged engine doesn't carry state into
	 * the next job.  Mirror ge2d_runtime_resume()'s reset sequence.  The
	 * pm_runtime_put below would also trigger a resume-time reset on the
	 * next get, but reset here too so the HW is quiesced immediately.
	 */
	reset_control_assert(ge2d->rst);
	if (ge2d_settle_us)
		udelay(ge2d_settle_us);
	reset_control_deassert(ge2d->rst);
	if (ge2d_settle_us)
		udelay(ge2d_settle_us);

	src = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
	dst = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
	if (src)
		v4l2_m2m_buf_done(src, VB2_BUF_STATE_ERROR);
	if (dst)
		v4l2_m2m_buf_done(dst, VB2_BUF_STATE_ERROR);
	v4l2_m2m_job_finish(ge2d->m2m_dev, ctx->fh.m2m_ctx);
	pm_runtime_mark_last_busy(ge2d->dev);
	pm_runtime_put_autosuspend(ge2d->dev);
}

/*
 * Timer callback (softirq).  v4l2_m2m_*, reset_control_*, and the sleeping
 * pm_runtime_* calls are NOT safe from here, so just hand off to process
 * context.
 */
static void ge2d_job_timeout(struct timer_list *t)
{
	struct meson_ge2d *ge2d = timer_container_of(ge2d, t, job_timer);

	schedule_work(&ge2d->job_timeout_work);
}

static void device_run(void *priv)
{
	struct ge2d_ctx *ctx = priv;
	struct meson_ge2d *ge2d = ctx->ge2d;
	unsigned long flags;
	int ret;

	pm_runtime_get_sync(ge2d->dev);

	ge2d->curr = ctx;

	ctx->in.buf = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	ctx->out.buf = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);

	ret = ge2d_hw_start(ge2d);
	if (ret) {
		struct vb2_v4l2_buffer *src, *dst;

		/*
		 * Surface the setup error through v4l2-m2m: mark both
		 * buffers as errored, release the hardware, and finish
		 * the job so the next one can run.
		 */
		ge2d->curr = NULL;
		src = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
		dst = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
		if (src)
			v4l2_m2m_buf_done(src, VB2_BUF_STATE_ERROR);
		if (dst)
			v4l2_m2m_buf_done(dst, VB2_BUF_STATE_ERROR);
		v4l2_m2m_job_finish(ge2d->m2m_dev, ctx->fh.m2m_ctx);
		pm_runtime_mark_last_busy(ge2d->dev);
		pm_runtime_put_autosuspend(ge2d->dev);
		return;
	}

	/*
	 * Blit started.  Arm the watchdog: if the completion IRQ never fires,
	 * ge2d_job_timeout() reschedules onto ge2d_job_timeout_work() which
	 * resets the IP and errors out the buffers.  700ms is > the 500ms KAPI
	 * timeout (so it never pre-empts a slow-but-live blit) and < a
	 * human-visible hang.
	 *
	 * Arm under hw_lock and skip if the device is being removed: ge2d_remove()
	 * sets ->removing under the same lock and then timer_delete_sync()s, so
	 * pairing here makes the arm and the final cancel mutually exclusive --
	 * this can never re-arm the timer after that sync-delete (which would fire
	 * from the freed struct).  mod_timer is non-sleeping, safe under the lock.
	 */
	spin_lock_irqsave(&ge2d->hw_lock, flags);
	if (!ge2d->removing)
		mod_timer(&ge2d->job_timer, jiffies + msecs_to_jiffies(700));
	spin_unlock_irqrestore(&ge2d->hw_lock, flags);
}

static irqreturn_t ge2d_isr(int irq, void *priv)
{
	struct meson_ge2d *ge2d = priv;
	u32 intr;

	regmap_read(ge2d->map, GE2D_STATUS0, &intr);

	if (!(intr & GE2D_GE2D_BUSY)) {
		/* Kernel API blit completion */
		if (ge2d->kapi_active) {
			ge2d->kapi_active = false;
			pm_runtime_mark_last_busy(ge2d->dev);
			pm_runtime_put_autosuspend(ge2d->dev);
			complete(&ge2d->kapi_done);
			return IRQ_HANDLED;
		}

		/* V4L2 M2M completion */
		{
			struct vb2_v4l2_buffer *src, *dst;
			struct ge2d_ctx *ctx;
			unsigned long flags;

			/*
			 * Cancel the watchdog FIRST.  timer_delete() (NOT the
			 * _sync variant) because we are in hardirq context: the
			 * timeout work runs in process context and the _sync
			 * wait would deadlock.  If the timer already expired and
			 * its work is in flight, the hw_lock claim below
			 * resolves the race -- whoever NULLs ge2d->curr wins.
			 */
			timer_delete(&ge2d->job_timer);

			spin_lock_irqsave(&ge2d->hw_lock, flags);
			ctx = ge2d->curr;
			ge2d->curr = NULL;
			spin_unlock_irqrestore(&ge2d->hw_lock, flags);

			/*
			 * NULL means the timeout work already claimed and
			 * finished this job (it raced us and won).  Nothing left
			 * to do -- the buffers are gone and the job is finished.
			 */
			if (!ctx)
				return IRQ_HANDLED;

			src = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
			dst = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);

			src->sequence = ctx->sequence_out++;
			dst->sequence = ctx->sequence_cap++;

			dst->timecode = src->timecode;
			dst->vb2_buf.timestamp = src->vb2_buf.timestamp;
			dst->flags = src->flags;

			/*
			 * Canvas IDs are cached across blits (canvases_allocated).
			 * Do NOT free here -- they are freed in ge2d_stop_streaming()
			 * on the OUTPUT queue after all blits are done.
			 */

			v4l2_m2m_buf_done(src, VB2_BUF_STATE_DONE);
			v4l2_m2m_buf_done(dst, VB2_BUF_STATE_DONE);
			v4l2_m2m_job_finish(ge2d->m2m_dev, ctx->fh.m2m_ctx);
			/*
			 * Keep-power across the m2m queue (GI-021): if more
			 * OUTPUT buffers are still queued, the next device_run()
			 * follows immediately, so hold the power domain + clock
			 * up instead of letting the 50ms autosuspend gate fire
			 * between blits.  This mirrors the BSP ge2d_wq model
			 * (power on once, run the whole queue, power off) and
			 * removes the per-blit runtime-suspend->resume cycle --
			 * the cycle that re-runs ge2d_runtime_resume()'s
			 * reset_control_assert/deassert per blit and is the
			 * window in which a busy->idle completion edge (an EDGE
			 * IRQ with NO pending/clear register) can be gated away.
			 *
			 * We still mark_last_busy + put_autosuspend so the
			 * pm_runtime refcount taken in device_run() is always
			 * balanced; the autosuspend timer (now 1s, see probe)
			 * only actually gates the clock once the queue drains
			 * and no fresh blit refreshes last_busy.  When more bufs
			 * are ready we mark_last_busy WITHOUT a put-delay change;
			 * the long autosuspend delay keeps the burst powered.
			 */
			pm_runtime_mark_last_busy(ge2d->dev);
			pm_runtime_put_autosuspend(ge2d->dev);
		}
	}

	return IRQ_HANDLED;
}

static const struct v4l2_m2m_ops ge2d_m2m_ops = {
	.device_run = device_run,
};

static int ge2d_queue_setup(struct vb2_queue *vq,
			    unsigned int *nbuffers, unsigned int *nplanes,
			    unsigned int sizes[], struct device *alloc_devs[])
{
	struct ge2d_ctx *ctx = vb2_get_drv_priv(vq);
	struct ge2d_frame *f = get_frame(ctx, vq->type);
	unsigned int i;

	if (*nplanes) {
		if (*nplanes != f->fmt->num_planes)
			return -EINVAL;
		for (i = 0; i < *nplanes; i++)
			if (sizes[i] < f->pix_mp.plane_fmt[i].sizeimage)
				return -EINVAL;
		return 0;
	}

	*nplanes = f->fmt->num_planes;
	for (i = 0; i < *nplanes; i++)
		sizes[i] = f->pix_mp.plane_fmt[i].sizeimage;

	return 0;
}

static int ge2d_buf_prepare(struct vb2_buffer *vb)
{
	struct ge2d_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct ge2d_frame *f = get_frame(ctx, vb->vb2_queue->type);
	unsigned int i;

	vbuf->field = V4L2_FIELD_NONE;

	for (i = 0; i < f->fmt->num_planes; i++)
		vb2_set_plane_payload(vb, i, f->pix_mp.plane_fmt[i].sizeimage);

	return 0;
}

static void ge2d_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct ge2d_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, vbuf);
}

static int ge2d_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct ge2d_ctx *ctx = vb2_get_drv_priv(vq);

	if (V4L2_TYPE_IS_OUTPUT(vq->type))
		ctx->sequence_out = 0;
	else
		ctx->sequence_cap = 0;

	return 0;
}

static void ge2d_stop_streaming(struct vb2_queue *vq)
{
	struct ge2d_ctx *ctx = vb2_get_drv_priv(vq);
	struct meson_ge2d *ge2d = ctx->ge2d;
	struct vb2_v4l2_buffer *vbuf;

	/*
	 * Cancel any pending watchdog before reclaiming buffers, so a timeout
	 * that fired mid-teardown can't double-finish a buffer we are about to
	 * return here.  timer_delete_sync + cancel_work_sync run in process
	 * context (stop_streaming sleeps under ge2d->mutex); the work fn does
	 * not take that mutex, so there is no lock-ordering deadlock.
	 */
	timer_delete_sync(&ge2d->job_timer);
	cancel_work_sync(&ge2d->job_timeout_work);

	for (;;) {
		if (V4L2_TYPE_IS_OUTPUT(vq->type))
			vbuf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
		else
			vbuf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
		if (!vbuf)
			break;
		v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);
	}

	/*
	 * Free cached canvas IDs when the OUTPUT queue stops.  Only the
	 * OUTPUT queue triggers the free to avoid a double-free from the
	 * capture queue's stop_streaming calling this too.  The canvas
	 * driver uses spinlock_irqsave internally; safe to call here.
	 */
	if (V4L2_TYPE_IS_OUTPUT(vq->type) && ctx->canvases_allocated &&
	    !ge2d->has_baddr && ge2d->canvas) {
		meson_canvas_free(ge2d->canvas, ctx->src1_canvas);
		meson_canvas_free(ge2d->canvas, ctx->src2_canvas);
		meson_canvas_free(ge2d->canvas, ctx->dst1_canvas);
		if (ctx->has_src_uv_canvas)
			meson_canvas_free(ge2d->canvas, ctx->src1_canvas_uv);
		if (ctx->has_dst2_canvas)
			meson_canvas_free(ge2d->canvas, ctx->dst2_canvas);
		if (ctx->has_src_planar_canvas) {
			meson_canvas_free(ge2d->canvas, ctx->src1_canvas_cb);
			meson_canvas_free(ge2d->canvas, ctx->src1_canvas_cr);
		}
		if (ctx->has_dst_planar_canvas) {
			meson_canvas_free(ge2d->canvas, ctx->dst_canvas_cb);
			meson_canvas_free(ge2d->canvas, ctx->dst_canvas_cr);
		}
		ctx->canvases_allocated = false;
	}

	/* Release SRC2 DMABUF on OUTPUT queue stop */
	if (V4L2_TYPE_IS_OUTPUT(vq->type))
		ge2d_src2_cleanup(ctx);
}

static const struct vb2_ops ge2d_qops = {
	.queue_setup = ge2d_queue_setup,
	.buf_prepare = ge2d_buf_prepare,
	.buf_queue = ge2d_buf_queue,
	.start_streaming = ge2d_start_streaming,
	.stop_streaming = ge2d_stop_streaming,
};

static int
queue_init(void *priv, struct vb2_queue *src_vq, struct vb2_queue *dst_vq)
{
	struct ge2d_ctx *ctx = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->ops = &ge2d_qops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &ctx->ge2d->mutex;
	src_vq->dev = ctx->ge2d->v4l2_dev.dev;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->ops = &ge2d_qops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &ctx->ge2d->mutex;
	dst_vq->dev = ctx->ge2d->v4l2_dev.dev;

	return vb2_queue_init(dst_vq);
}

static int
vidioc_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
	strscpy(cap->driver, GE2D_NAME, sizeof(cap->driver));
	strscpy(cap->card, GE2D_NAME, sizeof(cap->card));
	strscpy(cap->bus_info, "platform:" GE2D_NAME, sizeof(cap->bus_info));

	return 0;
}

static int vidioc_enum_fmt(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	if (f->index >= NUM_FORMATS)
		return -EINVAL;

	f->pixelformat = formats[f->index].fourcc;

	return 0;
}

static int vidioc_g_selection(struct file *file, void *priv,
			      struct v4l2_selection *s)
{
	struct ge2d_ctx *ctx = file_to_ge2d_ctx(file);
	struct ge2d_frame *f;
	bool use_frame = false;

	/*
	 * The V4L2 core remaps the _MPLANE buffer type to its non-MPLANE
	 * counterpart before dispatching selection ioctls (v4l2-ioctl.c
	 * v4l_g_selection). Normalize back to _MPLANE so the per-target type
	 * checks below stay simple; otherwise a userspace _MPLANE crop request
	 * arrives here as plain VIDEO_OUTPUT and falls through to -EINVAL.
	 */
	if (s->type == V4L2_BUF_TYPE_VIDEO_OUTPUT)
		s->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	else if (s->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
		s->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

	if (s->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE &&
	    s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return -EINVAL;

	f = get_frame(ctx, s->type);

	switch (s->target) {
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
		if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
			return -EINVAL;
		break;
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		if (s->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
			return -EINVAL;
		break;
	case V4L2_SEL_TGT_COMPOSE:
		if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
			return -EINVAL;
		use_frame = true;
		break;
	case V4L2_SEL_TGT_CROP:
		if (s->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
			return -EINVAL;
		use_frame = true;
		break;
	default:
		return -EINVAL;
	}

	if (use_frame) {
		s->r = f->crop;
	} else {
		s->r.left = 0;
		s->r.top = 0;
		s->r.width = f->pix_mp.width;
		s->r.height = f->pix_mp.height;
	}

	return 0;
}

static int vidioc_s_selection(struct file *file, void *priv,
			      struct v4l2_selection *s)
{
	struct ge2d_ctx *ctx = file_to_ge2d_ctx(file);
	struct meson_ge2d *ge2d = ctx->ge2d;
	struct ge2d_frame *f;
	int ret = 0;

	/*
	 * The V4L2 core remaps the _MPLANE buffer type to its non-MPLANE
	 * counterpart before dispatching selection ioctls (v4l2-ioctl.c
	 * v4l_s_selection). Normalize back to _MPLANE so the per-target type
	 * checks below stay simple; otherwise a userspace _MPLANE crop request
	 * arrives here as plain VIDEO_OUTPUT and falls through to -EINVAL.
	 */
	if (s->type == V4L2_BUF_TYPE_VIDEO_OUTPUT)
		s->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	else if (s->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
		s->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

	if (s->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE &&
	    s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return -EINVAL;

	f = get_frame(ctx, s->type);

	switch (s->target) {
	case V4L2_SEL_TGT_COMPOSE:
		/*
		 * COMPOSE target is only valid for capture buffer type, return
		 * error for output buffer type
		 */
		if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
			return -EINVAL;
		break;
	case V4L2_SEL_TGT_CROP:
		/*
		 * CROP target is only valid for output buffer type, return
		 * error for capture buffer type
		 */
		if (s->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
			return -EINVAL;
		break;
	/*
	 * bound and default crop/compose targets are invalid targets to
	 * try/set
	 */
	default:
		return -EINVAL;
	}

	if (s->r.top < 0 || s->r.left < 0) {
		v4l2_err(&ge2d->v4l2_dev,
			 "doesn't support negative values for top & left.\n");
		return -EINVAL;
	}

	if (s->r.left + s->r.width > f->pix_mp.width ||
	    s->r.top + s->r.height > f->pix_mp.height) {
		v4l2_err(&ge2d->v4l2_dev, "unsupported rectangle value.\n");
		return -EINVAL;
	}

	f->crop = s->r;

	return ret;
}

static void vidioc_setup_cap_fmt(struct ge2d_ctx *ctx,
				 struct v4l2_pix_format_mplane *f)
{
	struct ge2d_frame *frm_out = get_frame(ctx,
					       V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);

	*f = frm_out->pix_mp;

	if (ctx->xy_swap) {
		f->width = frm_out->pix_mp.height;
		f->height = frm_out->pix_mp.width;
	}
}

static int vidioc_try_fmt_cap(struct file *file, void *priv, struct v4l2_format *f)
{
	const struct ge2d_fmt *fmt = find_fmt(f);
	struct ge2d_ctx *ctx = file_to_ge2d_ctx(file);

	if (!fmt)
		return -EINVAL;

	/*
	 * Drop format flags not supported by the chosen fmt.
	 * PREMUL_ALPHA is only meaningful on alpha-bearing formats; SET_CSC
	 * only on YUV (we own the CSC matrix).  Sanitize before the rest of
	 * the try_fmt path uses pix_mp.flags so userspace cannot pass
	 * unsupported flags through to S_FMT.
	 */
	if (!fmt->alpha)
		f->fmt.pix_mp.flags &= ~V4L2_PIX_FMT_FLAG_PREMUL_ALPHA;
	if (!fmt->yuv)
		f->fmt.pix_mp.flags &= ~V4L2_PIX_FMT_FLAG_SET_CSC;

	/* Reject deep_color formats on SoCs without has_deep_color */
	if (fmt->deep_color) {
		if (!ctx->ge2d->has_deep_color)
			return -EINVAL;
	}

	/*
	 * PAL8 requires the SRC1 LUT which exists only on the OUTPUT (SRC1)
	 * path.  The DST path has no palette hardware, so reject PAL8 on the
	 * CAPTURE queue.
	 */
	if (fmt->fourcc == V4L2_PIX_FMT_PAL8)
		return -EINVAL;


	/* Accept progressive and field-based interlacing modes.
	 * pic_struct register bits handle field addressing in hardware.
	 */
	switch (f->fmt.pix_mp.field) {
	case V4L2_FIELD_NONE:
	case V4L2_FIELD_TOP:
	case V4L2_FIELD_BOTTOM:
	case V4L2_FIELD_INTERLACED_TB:
	case V4L2_FIELD_INTERLACED_BT:
		break;
	default:
		f->fmt.pix_mp.field = V4L2_FIELD_NONE;
		break;
	}
	f->fmt.pix_mp.pixelformat = fmt->fourcc;

	if (f->fmt.pix_mp.width > ctx->ge2d->max_width)
		f->fmt.pix_mp.width = ctx->ge2d->max_width;
	if (f->fmt.pix_mp.height > ctx->ge2d->max_height)
		f->fmt.pix_mp.height = ctx->ge2d->max_height;

	f->fmt.pix_mp.plane_fmt[0].bytesperline = max(f->fmt.pix_mp.plane_fmt[0].bytesperline,
				      ALIGN((f->fmt.pix_mp.width * fmt->depth) >> 3, 32));

	/* Clamp to canvas stride limit (12-bit field, max 32760 bytes) */
	while (f->fmt.pix_mp.plane_fmt[0].bytesperline > MAX_STRIDE && f->fmt.pix_mp.width > 1) {
		f->fmt.pix_mp.width -= 1;
		f->fmt.pix_mp.plane_fmt[0].bytesperline = ALIGN((f->fmt.pix_mp.width * fmt->depth) >> 3, 32);
	}

	f->fmt.pix_mp.num_planes = fmt->num_planes;

	if (fmt->nv && fmt->num_planes > 1) {
		/* Multi-planar NV12M/NV21M: Y in plane 0, UV in plane 1 */
		u32 uv_height = f->fmt.pix_mp.height / fmt->uv_vdiv;

		f->fmt.pix_mp.plane_fmt[0].sizeimage =
			f->fmt.pix_mp.plane_fmt[0].bytesperline *
			f->fmt.pix_mp.height;
		f->fmt.pix_mp.plane_fmt[1].bytesperline =
			f->fmt.pix_mp.plane_fmt[0].bytesperline;
		f->fmt.pix_mp.plane_fmt[1].sizeimage =
			f->fmt.pix_mp.plane_fmt[0].bytesperline * uv_height;
	} else if (fmt->nv) {
		/* Single-buffer semi-planar: Y + UV contiguous */
		u32 uv_height = f->fmt.pix_mp.height / fmt->uv_vdiv;

		f->fmt.pix_mp.plane_fmt[0].sizeimage =
			f->fmt.pix_mp.plane_fmt[0].bytesperline *
			f->fmt.pix_mp.height +
			f->fmt.pix_mp.plane_fmt[0].bytesperline * uv_height;
	} else if (fmt->planar) {
		/* Planar: Y + Cb + Cr in one contiguous buffer */
		u32 uv_stride = f->fmt.pix_mp.plane_fmt[0].bytesperline /
				fmt->uv_hdiv;
		u32 uv_height = f->fmt.pix_mp.height / fmt->uv_vdiv;

		f->fmt.pix_mp.plane_fmt[0].sizeimage =
			f->fmt.pix_mp.plane_fmt[0].bytesperline *
			f->fmt.pix_mp.height +
			2 * uv_stride * uv_height;
	} else {
		f->fmt.pix_mp.plane_fmt[0].sizeimage =
			max(f->fmt.pix_mp.plane_fmt[0].sizeimage,
			    f->fmt.pix_mp.height *
			    f->fmt.pix_mp.plane_fmt[0].bytesperline);
	}

	/*
	 * GI-018: advertise a round-trippable colorimetry on the CAPTURE
	 * (DST) queue.  Honor the caller's colorspace/ycbcr_enc/xfer_func/
	 * quantization (YUV->YUV uses no CSC matrix) so an identity
	 * bt709->bt709 pairing is accepted and gst links a downstream HW
	 * encoder with no CPU videoconvert.
	 */
	ge2d_fixup_colorimetry(&f->fmt.pix_mp, fmt->yuv);

	return 0;
}

static int vidioc_s_fmt_cap(struct file *file, void *priv, struct v4l2_format *f)
{
	struct ge2d_ctx *ctx = file_to_ge2d_ctx(file);
	struct meson_ge2d *ge2d = ctx->ge2d;
	struct vb2_queue *vq;
	struct ge2d_frame *frm;
	int ret = 0;

	/* Adjust all values accordingly to the hardware capabilities
	 * and chosen format.
	 */
	ret = vidioc_try_fmt_cap(file, priv, f);
	if (ret)
		return ret;

	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	if (vb2_is_busy(vq)) {
		v4l2_err(&ge2d->v4l2_dev, "queue (%d) bust\n", f->type);
		return -EBUSY;
	}

	frm = get_frame(ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);

	frm->pix_mp = f->fmt.pix_mp;
	frm->fmt = find_fmt(f);
	if (WARN_ON(!frm->fmt))
		return -EINVAL;
	f->fmt.pix_mp.pixelformat = frm->fmt->fourcc;

	/* Reset crop settings */
	frm->crop.left = 0;
	frm->crop.top = 0;
	frm->crop.width = frm->pix_mp.width;
	frm->crop.height = frm->pix_mp.height;

	return 0;
}

static int vidioc_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct ge2d_ctx *ctx = file_to_ge2d_ctx(file);
	struct vb2_queue *vq;
	struct ge2d_frame *frm;

	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;

	frm = get_frame(ctx, f->type);

	f->fmt.pix_mp = frm->pix_mp;
	f->fmt.pix_mp.pixelformat = frm->fmt->fourcc;

	return 0;
}

static int vidioc_try_fmt_out(struct file *file, void *priv, struct v4l2_format *f)
{
	const struct ge2d_fmt *fmt = find_fmt(f);
	struct ge2d_ctx *ctx = file_to_ge2d_ctx(file);

	if (!fmt)
		return -EINVAL;

	/* Same flag sanitization as vidioc_try_fmt_cap. */
	if (!fmt->alpha)
		f->fmt.pix_mp.flags &= ~V4L2_PIX_FMT_FLAG_PREMUL_ALPHA;
	if (!fmt->yuv)
		f->fmt.pix_mp.flags &= ~V4L2_PIX_FMT_FLAG_SET_CSC;

	/* Reject deep_color formats on SoCs without has_deep_color */
	if (fmt->deep_color) {
		if (!ctx->ge2d->has_deep_color)
			return -EINVAL;
	}


	switch (f->fmt.pix_mp.field) {
	case V4L2_FIELD_NONE:
	case V4L2_FIELD_TOP:
	case V4L2_FIELD_BOTTOM:
	case V4L2_FIELD_INTERLACED_TB:
	case V4L2_FIELD_INTERLACED_BT:
		break;
	default:
		f->fmt.pix_mp.field = V4L2_FIELD_NONE;
		break;
	}
	f->fmt.pix_mp.pixelformat = fmt->fourcc;

	if (f->fmt.pix_mp.width > ctx->ge2d->max_width)
		f->fmt.pix_mp.width = ctx->ge2d->max_width;
	if (f->fmt.pix_mp.height > ctx->ge2d->max_height)
		f->fmt.pix_mp.height = ctx->ge2d->max_height;

	f->fmt.pix_mp.plane_fmt[0].bytesperline = max(f->fmt.pix_mp.plane_fmt[0].bytesperline,
				      ALIGN((f->fmt.pix_mp.width * fmt->depth) >> 3, 32));

	/* Clamp to canvas stride limit (12-bit field, max 32760 bytes) */
	while (f->fmt.pix_mp.plane_fmt[0].bytesperline > MAX_STRIDE && f->fmt.pix_mp.width > 1) {
		f->fmt.pix_mp.width -= 1;
		f->fmt.pix_mp.plane_fmt[0].bytesperline = ALIGN((f->fmt.pix_mp.width * fmt->depth) >> 3, 32);
	}

	f->fmt.pix_mp.num_planes = fmt->num_planes;

	if (fmt->nv && fmt->num_planes > 1) {
		/* Multi-planar NV12M/NV21M: Y in plane 0, UV in plane 1 */
		u32 uv_height = f->fmt.pix_mp.height / fmt->uv_vdiv;

		f->fmt.pix_mp.plane_fmt[0].sizeimage =
			f->fmt.pix_mp.plane_fmt[0].bytesperline *
			f->fmt.pix_mp.height;
		f->fmt.pix_mp.plane_fmt[1].bytesperline =
			f->fmt.pix_mp.plane_fmt[0].bytesperline;
		f->fmt.pix_mp.plane_fmt[1].sizeimage =
			f->fmt.pix_mp.plane_fmt[0].bytesperline * uv_height;
	} else if (fmt->nv) {
		/* Single-buffer semi-planar: Y + UV contiguous */
		u32 uv_height = f->fmt.pix_mp.height / fmt->uv_vdiv;

		f->fmt.pix_mp.plane_fmt[0].sizeimage =
			f->fmt.pix_mp.plane_fmt[0].bytesperline *
			f->fmt.pix_mp.height +
			f->fmt.pix_mp.plane_fmt[0].bytesperline * uv_height;
	} else if (fmt->planar) {
		u32 uv_stride = f->fmt.pix_mp.plane_fmt[0].bytesperline /
				fmt->uv_hdiv;
		u32 uv_height = f->fmt.pix_mp.height / fmt->uv_vdiv;

		f->fmt.pix_mp.plane_fmt[0].sizeimage =
			f->fmt.pix_mp.plane_fmt[0].bytesperline *
			f->fmt.pix_mp.height +
			2 * uv_stride * uv_height;
	} else {
		f->fmt.pix_mp.plane_fmt[0].sizeimage =
			max(f->fmt.pix_mp.plane_fmt[0].sizeimage,
			    f->fmt.pix_mp.height *
			    f->fmt.pix_mp.plane_fmt[0].bytesperline);
	}

	/* GI-018: honor caller colorimetry -- see vidioc_try_fmt_cap. */
	ge2d_fixup_colorimetry(&f->fmt.pix_mp, fmt->yuv);

	return 0;
}

static int vidioc_s_fmt_out(struct file *file, void *priv, struct v4l2_format *f)
{
	struct ge2d_ctx *ctx = file_to_ge2d_ctx(file);
	struct meson_ge2d *ge2d = ctx->ge2d;
	struct vb2_queue *vq;
	struct ge2d_frame *frm, *frm_cap;
	int ret = 0;

	/* Adjust all values accordingly to the hardware capabilities
	 * and chosen format.
	 */
	ret = vidioc_try_fmt_out(file, priv, f);
	if (ret)
		return ret;

	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	if (vb2_is_busy(vq)) {
		v4l2_err(&ge2d->v4l2_dev, "queue (%d) bust\n", f->type);
		return -EBUSY;
	}

	frm = get_frame(ctx, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
	frm_cap = get_frame(ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);

	frm->pix_mp = f->fmt.pix_mp;
	frm->fmt = find_fmt(f);
	if (WARN_ON(!frm->fmt))
		return -EINVAL;
	f->fmt.pix_mp.pixelformat = frm->fmt->fourcc;

	/* Reset crop settings */
	frm->crop.left = 0;
	frm->crop.top = 0;
	frm->crop.width = frm->pix_mp.width;
	frm->crop.height = frm->pix_mp.height;

	/* Propagate settings to capture */
	vidioc_setup_cap_fmt(ctx, &frm_cap->pix_mp);

	return 0;
}

static const struct v4l2_ioctl_ops ge2d_ioctl_ops = {
	.vidioc_querycap = vidioc_querycap,

	.vidioc_enum_fmt_vid_cap = vidioc_enum_fmt,
	.vidioc_g_fmt_vid_cap_mplane = vidioc_g_fmt,
	.vidioc_try_fmt_vid_cap_mplane = vidioc_try_fmt_cap,
	.vidioc_s_fmt_vid_cap_mplane = vidioc_s_fmt_cap,

	.vidioc_enum_fmt_vid_out = vidioc_enum_fmt,
	.vidioc_g_fmt_vid_out_mplane = vidioc_g_fmt,
	.vidioc_try_fmt_vid_out_mplane = vidioc_try_fmt_out,
	.vidioc_s_fmt_vid_out_mplane = vidioc_s_fmt_out,

	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_prepare_buf = v4l2_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs = v4l2_m2m_ioctl_create_bufs,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,

	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,

	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,

	.vidioc_g_selection = vidioc_g_selection,
	.vidioc_s_selection = vidioc_s_selection,

	.vidioc_default = ge2d_ioctl_default,
};

static int ge2d_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ge2d_ctx *ctx = container_of(ctrl->handler, struct ge2d_ctx,
					   ctrl_handler);
	struct v4l2_pix_format_mplane fmt;
	struct vb2_queue *vq;

	switch (ctrl->id) {
	case V4L2_CID_HFLIP:
		ctx->hflip = ctrl->val;
		break;
	case V4L2_CID_VFLIP:
		ctx->vflip = ctrl->val;
		break;
	case V4L2_CID_ALPHA_COMPONENT:
		ctx->alpha = ctrl->val;
		break;
	case V4L2_CID_USER_MESON_GE2D_LOGIC_OP:
		if (ctrl->val >= ARRAY_SIZE(ge2d_logic_op_hw))
			return -EINVAL;
		ctx->logic_op = ctrl->val;
		break;
	case V4L2_CID_USER_MESON_GE2D_COLORKEY:
		ctx->colorkey = ctrl->val;
		break;
	case V4L2_CID_USER_MESON_GE2D_COLORKEY_MASK:
		ctx->colorkey_mask = ctrl->val;
		break;
	case V4L2_CID_USER_MESON_GE2D_FILLCOLOR:
		ctx->fillcolor = ctrl->val;
		break;
	case V4L2_CID_USER_MESON_GE2D_SCALE_FILTER:
		if (ctrl->val > 2)
			return -EINVAL;
		ctx->scale_filter = ctrl->val;
		break;
	case V4L2_CID_USER_MESON_GE2D_BITMASK:
		ctx->bitmask = ctrl->val;
		break;
	case V4L2_CID_USER_MESON_GE2D_ANTIFLICKER:
		ctx->antiflicker = ctrl->val;
		break;
	case V4L2_CID_ROTATE:
		vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
		if (vb2_is_busy(vq))
			return -EBUSY;

		if (ctrl->val == 90) {
			ctx->hflip = 1;
			ctx->vflip = 0;
			ctx->xy_swap = 1;
		} else if (ctrl->val == 180) {
			ctx->hflip = 1;
			ctx->vflip = 1;
			ctx->xy_swap = 0;
		} else if (ctrl->val == 270) {
			ctx->hflip = 0;
			ctx->vflip = 1;
			ctx->xy_swap = 1;
		} else {
			ctx->hflip = 0;
			ctx->vflip = 0;
			ctx->xy_swap = 0;
		}

		vidioc_setup_cap_fmt(ctx, &fmt);

		/*
		 * If the rotation parameter changes the OUTPUT frames
		 * parameters, take them in account
		 */
		ctx->out.pix_mp = fmt;

		break;
	}

	return 0;
}

static const struct v4l2_ctrl_ops ge2d_ctrl_ops = {
	.s_ctrl = ge2d_s_ctrl,
};

static const struct v4l2_ctrl_config ge2d_ctrl_colorkey = {
	.ops  = &ge2d_ctrl_ops,
	.id   = V4L2_CID_USER_MESON_GE2D_COLORKEY,
	.name = "Color Key",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min  = 0, .max = 0x7FFFFFFF, .step = 1, .def = 0,
};

static const struct v4l2_ctrl_config ge2d_ctrl_colorkey_mask = {
	.ops  = &ge2d_ctrl_ops,
	.id   = V4L2_CID_USER_MESON_GE2D_COLORKEY_MASK,
	.name = "Color Key Mask",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min  = 0, .max = 0x7FFFFFFF, .step = 1, .def = 0,
};

static const struct v4l2_ctrl_config ge2d_ctrl_fillcolor = {
	.ops  = &ge2d_ctrl_ops,
	.id   = V4L2_CID_USER_MESON_GE2D_FILLCOLOR,
	.name = "Fill Color",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min  = 0, .max = 0x7FFFFFFF, .step = 1, .def = 0,
};

static const struct v4l2_ctrl_config ge2d_ctrl_antiflicker = {
	.ops  = &ge2d_ctrl_ops,
	.id   = V4L2_CID_USER_MESON_GE2D_ANTIFLICKER,
	.name = "Anti-Flicker Filter",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min  = 0, .max = 1, .step = 1, .def = 0,
};

static const struct v4l2_ctrl_config ge2d_ctrl_bitmask = {
	.ops  = &ge2d_ctrl_ops,
	.id   = V4L2_CID_USER_MESON_GE2D_BITMASK,
	.name = "Destination Bitmask",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min  = 0, .max = 0x7FFFFFFF, .step = 1, .def = 0,
};

static const struct v4l2_ctrl_config ge2d_ctrl_scale_filter = {
	.ops  = &ge2d_ctrl_ops,
	.id   = V4L2_CID_USER_MESON_GE2D_SCALE_FILTER,
	.name = "Scaling Filter",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min  = 0, .max = 2, .step = 1, .def = 0,
};

static int ge2d_setup_ctrls(struct ge2d_ctx *ctx)
{
	struct meson_ge2d *ge2d = ctx->ge2d;

	v4l2_ctrl_handler_init(&ctx->ctrl_handler, 10);

	v4l2_ctrl_new_std(&ctx->ctrl_handler, &ge2d_ctrl_ops,
			  V4L2_CID_HFLIP, 0, 1, 1, 0);

	v4l2_ctrl_new_std(&ctx->ctrl_handler, &ge2d_ctrl_ops,
			  V4L2_CID_VFLIP, 0, 1, 1, 0);

	v4l2_ctrl_new_std(&ctx->ctrl_handler, &ge2d_ctrl_ops,
			  V4L2_CID_ROTATE, 0, 270, 90, 0);

	v4l2_ctrl_new_std(&ctx->ctrl_handler, &ge2d_ctrl_ops,
			  V4L2_CID_ALPHA_COMPONENT, 0, 255, 1, 255);

	/* Color keying: match source pixels against key color with mask */
	v4l2_ctrl_new_custom(&ctx->ctrl_handler, &ge2d_ctrl_colorkey, NULL);
	v4l2_ctrl_new_custom(&ctx->ctrl_handler, &ge2d_ctrl_colorkey_mask, NULL);
	/* Fill mode: non-zero value fills destination with constant color */
	v4l2_ctrl_new_custom(&ctx->ctrl_handler, &ge2d_ctrl_fillcolor, NULL);
	/* Scaling filter: bilinear(0), bicubic(1), triangle(2) */
	v4l2_ctrl_new_custom(&ctx->ctrl_handler, &ge2d_ctrl_scale_filter, NULL);
	/* Destination bitmask: selective per-pixel write masking */
	v4l2_ctrl_new_custom(&ctx->ctrl_handler, &ge2d_ctrl_bitmask, NULL);
	/* Anti-flicker: vertical low-pass filter for interlaced output */
	v4l2_ctrl_new_custom(&ctx->ctrl_handler, &ge2d_ctrl_antiflicker, NULL);

	if (ctx->ctrl_handler.error) {
		int err = ctx->ctrl_handler.error;

		v4l2_err(&ge2d->v4l2_dev, "%s failed\n", __func__);
		v4l2_ctrl_handler_free(&ctx->ctrl_handler);
		return err;
	}

	return 0;
}

static const struct ge2d_frame def_frame = {
	.pix_mp = {
		.width = DEFAULT_WIDTH,
		.height = DEFAULT_HEIGHT,
		.pixelformat = V4L2_PIX_FMT_XRGB32,
		.field = V4L2_FIELD_NONE,
		.num_planes = 1,
		.plane_fmt = {
			{ .bytesperline = DEFAULT_STRIDE,
			  .sizeimage = DEFAULT_STRIDE * DEFAULT_HEIGHT },
		},
	},
	.crop.width = DEFAULT_WIDTH,
	.crop.height = DEFAULT_HEIGHT,
	.fmt = &formats[0],
};

static int ge2d_open(struct file *file)
{
	struct meson_ge2d *ge2d = video_drvdata(file);
	struct ge2d_ctx *ctx = NULL;
	int ret = 0;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	ctx->ge2d = ge2d;
	ctx->src2_bound_fd = -1;

	/* Set default formats */
	ctx->in = def_frame;
	ctx->out = def_frame;

	/*
	 * GI-020: normalize the default colorimetry up front.  vidioc_g_fmt
	 * returns the stored format verbatim, but vidioc_try_fmt/s_fmt run
	 * ge2d_fixup_colorimetry (which forces colorspace=SRGB +
	 * quantization=FULL_RANGE on the RGB default).  Without this the very
	 * first TRY_FMT(G_FMT)/S_FMT(G_FMT) after open flips those two fields,
	 * which fails the V4L2 format-idempotency conformance check.  Run the
	 * same fixup once so G_FMT and TRY_FMT agree from the start.
	 */
	ge2d_fixup_colorimetry(&ctx->in.pix_mp, ctx->in.fmt->yuv);
	ge2d_fixup_colorimetry(&ctx->out.pix_mp, ctx->out.fmt->yuv);

	if (mutex_lock_interruptible(&ge2d->mutex)) {
		kfree(ctx);
		return -ERESTARTSYS;
	}
	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(ge2d->m2m_dev, ctx, &queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		mutex_unlock(&ge2d->mutex);
		kfree(ctx);
		return ret;
	}
	v4l2_fh_init(&ctx->fh, video_devdata(file));
	v4l2_fh_add(&ctx->fh, file);

	ge2d_setup_ctrls(ctx);

	/* Write the default values to the ctx struct */
	v4l2_ctrl_handler_setup(&ctx->ctrl_handler);

	ctx->fh.ctrl_handler = &ctx->ctrl_handler;
	mutex_unlock(&ge2d->mutex);

	return 0;
}

static int ge2d_release(struct file *file)
{
	struct ge2d_ctx *ctx = file_to_ge2d_ctx(file);
	struct meson_ge2d *ge2d = ctx->ge2d;

	mutex_lock(&ge2d->mutex);

	ge2d_src2_cleanup(ctx);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);

	v4l2_ctrl_handler_free(&ctx->ctrl_handler);
	v4l2_fh_del(&ctx->fh, file);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);

	mutex_unlock(&ge2d->mutex);

	return 0;
}

static const struct v4l2_file_operations ge2d_fops = {
	.owner = THIS_MODULE,
	.open = ge2d_open,
	.release = ge2d_release,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
};

static const struct video_device ge2d_videodev = {
	.name = "meson-ge2d",
	.fops = &ge2d_fops,
	.ioctl_ops = &ge2d_ioctl_ops,
	.minor = -1,
	.release = video_device_release,
	.vfl_dir = VFL_DIR_M2M,
	.device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING,
};

static const struct regmap_config meson_ge2d_regmap_conf = {
	.reg_bits = 8,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = GE2D_SRC2_STRIDE_CTRL,
};

static const struct meson_ge2d_ops ge2d_kapi_ops;

/*
 * Thermal cooling -- throttle GE2D clock on thermal pressure.
 * Follows the VDEC cooling pattern (vdec.c).
 */
static const unsigned long ge2d_cooling_rates[] = {
	667000000,	/* state 0: turbo (overclock) */
	500000000,	/* state 1: BSP rated */
	400000000,	/* state 2: throttled */
	250000000,	/* state 3: safe */
};

#define GE2D_COOLING_MAX_STATE (ARRAY_SIZE(ge2d_cooling_rates) - 1)

static unsigned long ge2d_clock_rate(struct meson_ge2d *ge2d)
{
	unsigned long state = READ_ONCE(ge2d->thermal_state);
	unsigned long rate;

	if (state > GE2D_COOLING_MAX_STATE)
		state = GE2D_COOLING_MAX_STATE;
	rate = ge2d_cooling_rates[state];

	/* Turbo only applies at thermal state 0 (no throttle) */
	if (state == 0 && READ_ONCE(ge2d->turbo) && ge2d->turbo_rate)
		rate = ge2d->turbo_rate;

	/* Clamp to platform max (BSP rated) when not in turbo */
	if (!READ_ONCE(ge2d->turbo) && rate > ge2d->max_rate)
		rate = ge2d->max_rate;

	return rate;
}

static int ge2d_cooling_get_max_state(struct thermal_cooling_device *cdev,
				      unsigned long *state)
{
	*state = GE2D_COOLING_MAX_STATE;
	return 0;
}

static int ge2d_cooling_get_cur_state(struct thermal_cooling_device *cdev,
				      unsigned long *state)
{
	struct meson_ge2d *ge2d = cdev->devdata;

	*state = READ_ONCE(ge2d->thermal_state);
	return 0;
}

static int ge2d_cooling_set_cur_state(struct thermal_cooling_device *cdev,
				      unsigned long state)
{
	struct meson_ge2d *ge2d = cdev->devdata;

	if (state > GE2D_COOLING_MAX_STATE)
		return -EINVAL;

	if (state != READ_ONCE(ge2d->thermal_state)) {
		WRITE_ONCE(ge2d->thermal_state, state);
		dev_info(ge2d->dev,
			 "thermal throttle: state %lu -> %lu MHz\n",
			 state, ge2d_clock_rate(ge2d) / 1000000);
	}
	return 0;
}

static const struct thermal_cooling_device_ops ge2d_cooling_ops = {
	.get_max_state = ge2d_cooling_get_max_state,
	.get_cur_state = ge2d_cooling_get_cur_state,
	.set_cur_state = ge2d_cooling_set_cur_state,
};

static ssize_t turbo_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct meson_ge2d *ge2d = dev_get_drvdata(dev);
	return sysfs_emit(buf, "%d\n", ge2d->turbo);
}

static ssize_t turbo_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct meson_ge2d *ge2d = dev_get_drvdata(dev);
	bool val;
	int ret = kstrtobool(buf, &val);
	if (ret)
		return ret;
	WRITE_ONCE(ge2d->turbo, val);
	dev_info(dev, "turbo %s (rate target: %lu MHz)\n",
		 val ? "enabled" : "disabled",
		 ge2d_clock_rate(ge2d) / 1000000);
	return count;
}

static DEVICE_ATTR_RW(turbo);

static ssize_t burst_config_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct meson_ge2d *ge2d = dev_get_drvdata(dev);
	return sysfs_emit(buf, "0x%x\n", ge2d->burst_ctrl4);
}

static ssize_t burst_config_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct meson_ge2d *ge2d = dev_get_drvdata(dev);
	u32 val;
	int ret = kstrtou32(buf, 0, &val);
	if (ret)
		return ret;
	ge2d->burst_ctrl4 = val & GENMASK(28, 16);
	return count;
}

static DEVICE_ATTR_RW(burst_config);

static struct attribute *ge2d_attrs[] = {
	&dev_attr_turbo.attr,
	&dev_attr_burst_config.attr,
	NULL,
};
ATTRIBUTE_GROUPS(ge2d);

static int ge2d_probe(struct platform_device *pdev)
{
	const struct meson_ge2d_match_data *match;
	struct video_device *vfd;
	struct meson_ge2d *ge2d;
	void __iomem *regs;
	int ret = 0;
	int irq;

	if (!pdev->dev.of_node)
		return -ENODEV;

	match = of_device_get_match_data(&pdev->dev);

	ge2d = devm_kzalloc(&pdev->dev, sizeof(*ge2d), GFP_KERNEL);
	if (!ge2d)
		return -ENOMEM;

	ge2d->dev = &pdev->dev;

	/*
	 * Constrain the advertised max resolution by the board's CMA pool.  The
	 * 8K hardware max (MAX_WIDTH/HEIGHT) implies an 8192x8192x4 = 256 MB
	 * single DMA buffer, which fits S905X/SM1 (980 MB CMA) but NOT S805X
	 * (256 MB CMA) -- there an S_FMT at that size makes dma_alloc fail with a
	 * page-alloc WARN.  totalcma_pages is not exported to modules, so estimate
	 * CMA as totalram/4 (CMA defaults to ~25% of RAM on Amlogic: cma=1G on 4GB,
	 * cma=256M on 1GB) with 70% headroom -- the same proxy the meson-vdec
	 * driver uses (vdec.c).  A ge2d M2M op needs a src AND a dst contiguous
	 * buffer at worst case 4 bytes/pixel, so budget both within `usable`
	 * (divide by 2*4=8) and derive the square max dimension.  Result: ~4672 on
	 * S805X (1GB RAM/256 MB CMA), 8192 (== hw max) on S905X/SM1 (4GB/980 MB) --
	 * the clamp only ever reduces below the hw max on small-CMA boards.  Per
	 * amlogic codec CLAUDE.md "Resolution Limits: Hardware Max, CMA-Constrained".
	 */
	{
		u64 ram_bytes = (u64)totalram_pages() << PAGE_SHIFT;
		u64 usable = (ram_bytes / 4) * 70 / 100;   /* CMA est * headroom */
		u32 dim = int_sqrt(usable / 8);            /* /(src+dst * 4 Bpp) */

		dim = min_t(u32, dim, MAX_WIDTH) & ~63u;   /* clamp to hw max, 64-align */
		if (dim < 256)
			dim = 256;                         /* sanity floor */
		ge2d->max_width = dim;
		ge2d->max_height = dim;
		dev_info(&pdev->dev,
			 "max resolution %ux%u (RAM %llu MiB, CMA est %llu MiB)\n",
			 dim, dim, ram_bytes >> 20, (ram_bytes / 4) >> 20);
	}

	mutex_init(&ge2d->mutex);
	mutex_init(&ge2d->kapi_lock);
	init_completion(&ge2d->kapi_done);
	spin_lock_init(&ge2d->hw_lock);
	timer_setup(&ge2d->job_timer, ge2d_job_timeout, 0);
	INIT_WORK(&ge2d->job_timeout_work, ge2d_job_timeout_work);

	regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(regs))
		return PTR_ERR(regs);

	/*
	 * Adjust base for GXL register layout.
	 * GXL uses CBUS-relative register indices (0xa0+) while G12A/SM1/AXG
	 * use VAPB3-relative indices (0x8a0+). The GE2D_REG() macro always
	 * uses the 0x8a0 base. Adding reg_base (-0x2000 for GXL) to the
	 * ioremap pointer compensates so GE2D_REG(x) hits the right offset.
	 */
	if (match)
		regs += match->reg_base;

	ge2d->map = devm_regmap_init_mmio(ge2d->dev, regs,
					  &meson_ge2d_regmap_conf);
	if (IS_ERR(ge2d->map))
		return PTR_ERR(ge2d->map);

	irq = platform_get_irq(pdev, 0);
	ret = devm_request_irq(ge2d->dev, irq, ge2d_isr, 0,
			       dev_name(ge2d->dev), ge2d);
	if (ret < 0) {
		dev_err(ge2d->dev, "failed to request irq\n");
		return ret;
	}

	ge2d->rst = devm_reset_control_get(ge2d->dev, NULL);
	if (IS_ERR(ge2d->rst)) {
		dev_err(ge2d->dev, "failed to get core reset controller\n");
		return PTR_ERR(ge2d->rst);
	}

	ge2d->clk = devm_clk_get(ge2d->dev, "vapb");
	if (IS_ERR(ge2d->clk)) {
		/* Fallback: unnamed clock (for old DT compat) */
		ge2d->clk = devm_clk_get(ge2d->dev, NULL);
		if (IS_ERR(ge2d->clk)) {
			dev_err(ge2d->dev, "failed to get vapb clock\n");
			return PTR_ERR(ge2d->clk);
		}
	}

	/* Optional bus gate clock (HHI_GCLK_MPEG1 GE2D gate) */
	ge2d->clk_gate = devm_clk_get_optional(ge2d->dev, "g2d");
	if (IS_ERR(ge2d->clk_gate))
		return PTR_ERR(ge2d->clk_gate);

	ge2d->has_baddr     = match ? match->has_baddr     : true;
	ge2d->has_src2_alp  = match ? match->has_src2_alp  : false;
	ge2d->has_hang_flag = match ? match->has_hang_flag : false;
	ge2d->has_deep_color = match ? match->has_deep_color : false;
	ge2d->max_rate      = match ? match->max_rate      : 250000000;
	ge2d->turbo_rate    = match ? match->turbo_rate    : 0;
	/* Default burst: max on G12B/SM1 (live bits), harmless on GXL (dead bits) */
	ge2d->burst_ctrl4   = (3 << 16) | BIT(28);

	if (!ge2d->has_baddr) {
		ge2d->canvas = meson_canvas_get(ge2d->dev);
		if (IS_ERR(ge2d->canvas)) {
			dev_err(ge2d->dev, "failed to get canvas provider\n");
			return PTR_ERR(ge2d->canvas);
		}
	}

	platform_set_drvdata(pdev, ge2d);

	pm_runtime_enable(&pdev->dev);
	pm_runtime_use_autosuspend(&pdev->dev);
	/*
	 * Keep-power across the m2m queue (GI-021).  The autosuspend delay is
	 * the window the power domain + VAPB clock stay up after the last blit
	 * before the runtime-suspend gate fires.  At the old 50ms a streaming
	 * blit burst that spaces blits >50ms apart got a full
	 * runtime_suspend->runtime_resume cycle BETWEEN consecutive blits, and
	 * each runtime_resume re-runs reset_control_assert/deassert(RESET_GE2D)
	 * -- the per-blit reset/clock-gate that is the busy->idle EDGE-IRQ
	 * drop window (the IRQ has no pending/clear register, so an edge lost
	 * while the clock is gated is gone forever).  Raise it to 1s so a
	 * back-to-back burst stays powered and the IP is reset ONCE per real
	 * power-on, mirroring the BSP "power on once, run the queue, power off"
	 * model.  An idle ge2d still autosuspends after 1s, so the static-power
	 * cost when truly idle is unchanged.
	 */
	pm_runtime_set_autosuspend_delay(&pdev->dev, 1000);
	ret = pm_runtime_resume_and_get(&pdev->dev);
	if (ret) {
		dev_err(ge2d->dev, "failed to enable power: %d\n", ret);
		goto disable_pm;
	}

	dev_info(&pdev->dev, "GE2D clock: %lu MHz (max %lu MHz, turbo %lu MHz)\n",
		 clk_get_rate(ge2d->clk) / 1000000,
		 ge2d->max_rate / 1000000,
		 ge2d->turbo_rate / 1000000);

	pm_runtime_put(&pdev->dev);

	/* Register thermal cooling device */
	ge2d->cooling = thermal_of_cooling_device_register(
			pdev->dev.of_node, "meson-ge2d", ge2d,
			&ge2d_cooling_ops);
	if (IS_ERR(ge2d->cooling)) {
		dev_warn(&pdev->dev,
			 "failed to register thermal cooling device: %ld\n",
			 PTR_ERR(ge2d->cooling));
		ge2d->cooling = NULL;
	}

	ret = v4l2_device_register(&pdev->dev, &ge2d->v4l2_dev);
	if (ret)
		goto disable_pm;

	vfd = video_device_alloc();
	if (!vfd) {
		v4l2_err(&ge2d->v4l2_dev, "Failed to allocate video device\n");
		ret = -ENOMEM;
		goto unreg_v4l2_dev;
	}

	*vfd = ge2d_videodev;
	vfd->lock = &ge2d->mutex;
	vfd->v4l2_dev = &ge2d->v4l2_dev;

	video_set_drvdata(vfd, ge2d);
	ge2d->vfd = vfd;

	ge2d->m2m_dev = v4l2_m2m_init(&ge2d_m2m_ops);
	if (IS_ERR(ge2d->m2m_dev)) {
		v4l2_err(&ge2d->v4l2_dev, "Failed to init mem2mem device\n");
		ret = PTR_ERR(ge2d->m2m_dev);
		goto rel_vdev;
	}

	ret = video_register_device(vfd, VFL_TYPE_VIDEO, -1);
	if (ret) {
		v4l2_err(&ge2d->v4l2_dev, "Failed to register video device\n");
		goto rel_m2m;
	}

	v4l2_info(&ge2d->v4l2_dev, "Registered %s as /dev/%s\n",
		  vfd->name, video_device_node_name(vfd));

	meson_ge2d_register(&ge2d_kapi_ops);

	return 0;

rel_m2m:
	v4l2_m2m_release(ge2d->m2m_dev);
rel_vdev:
	video_device_release(ge2d->vfd);
unreg_v4l2_dev:
	v4l2_device_unregister(&ge2d->v4l2_dev);
disable_pm:
	pm_runtime_disable(&pdev->dev);

	return ret;
}

/* Kernel API */

struct meson_ge2d_dev {
	struct meson_ge2d *ge2d;
};

static const struct {
	unsigned int hw_fmt;
	unsigned int hw_map;
	unsigned int depth;
	bool yuv;
	bool alpha;
} ge2d_kapi_fmts[] = {
	[GE2D_FMT_XRGB8888] = { GE2D_FORMAT_32BIT, GE2D_COLOR_MAP_ARGB8888,
				 32, false, false },
	[GE2D_FMT_ARGB8888] = { GE2D_FORMAT_32BIT, GE2D_COLOR_MAP_ARGB8888,
				 32, false, true },
	[GE2D_FMT_RGB888]   = { GE2D_FORMAT_24BIT, GE2D_COLOR_MAP_BGR888,
				 24, false, false },
	[GE2D_FMT_YUYV]     = { GE2D_FORMAT_16BIT, GE2D_COLOR_MAP_YUV422,
				 16, true, false },
	[GE2D_FMT_NV12]     = { GE2D_FORMAT_24BIT, GE2D_COLOR_MAP_NV12,
				 8, true, false },
	[GE2D_FMT_NV21]     = { GE2D_FORMAT_24BIT, GE2D_COLOR_MAP_NV21,
				 8, true, false },
};

static struct meson_ge2d_dev *ge2d_kapi_get(struct device *dev)
{
	struct device_node *np;
	struct platform_device *pdev;
	struct meson_ge2d *ge2d;
	struct meson_ge2d_dev *gdev;

	np = of_parse_phandle(dev->of_node, "amlogic,ge2d", 0);
	if (!np)
		return ERR_PTR(-ENODEV);

	pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!pdev)
		return ERR_PTR(-EPROBE_DEFER);

	ge2d = platform_get_drvdata(pdev);
	if (!ge2d) {
		put_device(&pdev->dev);
		return ERR_PTR(-EPROBE_DEFER);
	}

	gdev = devm_kzalloc(dev, sizeof(*gdev), GFP_KERNEL);
	if (!gdev) {
		put_device(&pdev->dev);
		return ERR_PTR(-ENOMEM);
	}
	gdev->ge2d = ge2d;

	return gdev;
}

static bool ge2d_kapi_fmt_is_nv(enum meson_ge2d_fmt fmt)
{
	return fmt == GE2D_FMT_NV12 || fmt == GE2D_FMT_NV21;
}

static int ge2d_kapi_setup_addr(struct meson_ge2d *ge2d,
				struct meson_ge2d_job *job,
				bool src_nv, bool dst_nv,
				u8 *src1_canvas, u8 *src1_canvas_uv,
				u8 *src2_canvas,
				u8 *dst1_canvas, u8 *dst2_canvas)
{
	u32 src1_canvas_reg, dst_canvas_reg;
	int ret;

	if (ge2d->has_baddr) {
		regmap_write(ge2d->map, GE2D_SRC1_BADDR_CTRL,
			     (job->src_addr + 7) >> 3);
		regmap_write(ge2d->map, GE2D_SRC1_STRIDE_CTRL,
			     (job->src_stride + 7) >> 3);
		regmap_write(ge2d->map, GE2D_SRC2_BADDR_CTRL,
			     (job->dst_addr + 7) >> 3);
		regmap_write(ge2d->map, GE2D_SRC2_STRIDE_CTRL,
			     (job->dst_stride + 7) >> 3);
		regmap_write(ge2d->map, GE2D_DST1_BADDR_CTRL,
			     (job->dst_addr + 7) >> 3);
		regmap_write(ge2d->map, GE2D_DST1_STRIDE_CTRL,
			     (job->dst_stride + 7) >> 3);
		/* NV12: BADDR mode only provides Y plane; UV is ignored. */
	} else {
		ret = meson_canvas_alloc(ge2d->canvas, src1_canvas);
		if (ret)
			return ret;
		ret = meson_canvas_alloc(ge2d->canvas, src2_canvas);
		if (ret)
			goto free_src1;
		ret = meson_canvas_alloc(ge2d->canvas, dst1_canvas);
		if (ret)
			goto free_src2;

		meson_canvas_config(ge2d->canvas, *src1_canvas,
				    job->src_addr, job->src_stride,
				    job->src_h, MESON_CANVAS_WRAP_NONE,
				    MESON_CANVAS_BLKMODE_LINEAR, 0);
		/* SRC2 reads the destination Y plane for blend feedback */
		meson_canvas_config(ge2d->canvas, *src2_canvas,
				    job->dst_addr, job->dst_stride,
				    job->dst_h, MESON_CANVAS_WRAP_NONE,
				    MESON_CANVAS_BLKMODE_LINEAR, 0);
		meson_canvas_config(ge2d->canvas, *dst1_canvas,
				    job->dst_addr, job->dst_stride,
				    job->dst_h, MESON_CANVAS_WRAP_NONE,
				    MESON_CANVAS_BLKMODE_LINEAR, 0);

		/* NV12/NV21 source: allocate UV canvas (interleaved CbCr) */
		if (src_nv) {
			ret = meson_canvas_alloc(ge2d->canvas, src1_canvas_uv);
			if (ret)
				goto free_dst1;
			meson_canvas_config(ge2d->canvas, *src1_canvas_uv,
					    job->src_addr_uv, job->src_stride,
					    job->src_h / 2,
					    MESON_CANVAS_WRAP_NONE,
					    MESON_CANVAS_BLKMODE_LINEAR, 0);
		}

		/* NV12/NV21 destination: allocate UV canvas (interleaved CbCr) */
		if (dst_nv) {
			ret = meson_canvas_alloc(ge2d->canvas, dst2_canvas);
			if (ret)
				goto free_src1_uv;
			meson_canvas_config(ge2d->canvas, *dst2_canvas,
					    job->dst_addr_uv, job->dst_stride,
					    job->dst_h / 2,
					    MESON_CANVAS_WRAP_NONE,
					    MESON_CANVAS_BLKMODE_LINEAR, 0);
		}

		/*
		 * SRC1_CANVAS: bits 31:24 = Y canvas, 23:16 = UV canvas
		 * (bits 23:16 are the CbCr canvas for NV; zero for packed).
		 */
		src1_canvas_reg = FIELD_PREP(GE2D_SRC1_CANVAS_ADDR, *src1_canvas);
		if (src_nv)
			src1_canvas_reg |= (u32)(*src1_canvas_uv) << 16;
		regmap_write(ge2d->map, GE2D_SRC1_CANVAS, src1_canvas_reg);

		/*
		 * SRC2_DST_CANVAS: bits 23:16 = DST2 (UV out), 15:8 = SRC2,
		 *                  7:0 = DST1 (Y out).
		 */
		dst_canvas_reg = FIELD_PREP(GE2D_SRC2_CANVAS_ADDR, *src2_canvas) |
				 FIELD_PREP(GE2D_DST1_CANVAS_ADDR, *dst1_canvas);
		if (dst_nv)
			dst_canvas_reg |= FIELD_PREP(GE2D_DST2_CANVAS_ADDR,
						     *dst2_canvas);
		regmap_write(ge2d->map, GE2D_SRC2_DST_CANVAS, dst_canvas_reg);
	}
	return 0;

free_src1_uv:
	if (src_nv)
		meson_canvas_free(ge2d->canvas, *src1_canvas_uv);
free_dst1:
	meson_canvas_free(ge2d->canvas, *dst1_canvas);
free_src2:
	meson_canvas_free(ge2d->canvas, *src2_canvas);
free_src1:
	meson_canvas_free(ge2d->canvas, *src1_canvas);
	return ret;
}

static void ge2d_kapi_hw_start(struct meson_ge2d *ge2d,
			       struct meson_ge2d_job *job)
{
	bool src_yuv = ge2d_kapi_fmts[job->src_fmt].yuv;
	bool dst_yuv = ge2d_kapi_fmts[job->dst_fmt].yuv;
	bool src_alpha = ge2d_kapi_fmts[job->src_fmt].alpha;
	bool dst_alpha = ge2d_kapi_fmts[job->dst_fmt].alpha;
	bool src_nv = ge2d_kapi_fmt_is_nv(job->src_fmt);
	bool dst_nv = ge2d_kapi_fmt_is_nv(job->dst_fmt);
	/* Source has horizontal chroma subsampling (422 or 420) */
	bool src_x_yc_ratio = src_yuv &&
			       (src_nv ||
				ge2d_kapi_fmts[job->src_fmt].depth == 16);
	u32 reg;

	/* Reset */
	regmap_update_bits(ge2d->map, GE2D_GEN_CTRL1,
			   GE2D_SOFT_RST, GE2D_SOFT_RST);
	regmap_update_bits(ge2d->map, GE2D_GEN_CTRL1,
			   GE2D_SOFT_RST, 0);
	usleep_range(100, 200);

	/* Init outside-source alpha to 0 (transparent) for both SRC1 and SRC2.
	 * BSP ge2d_hw.c programs bits [7:0] = SRC1 outside_alpha,
	 * bits [23:16] = SRC2 outside_alpha. Zero = fully transparent
	 * outside the source clip rectangle.
	 */
	regmap_write(ge2d->map, GE2D_SRC_OUTSIDE_ALPHA, 0);

	/*
	 * GE2D_GEN_CTRL4: bit 0 is hang_flag. BSP sets it to 1 on G12A/G12B/SM1
	 * per blit to disable the dp_out_done/scale_out_done hang fix present on
	 * those SoCs (which can trigger at high downscale ratios). GXL/AXG do not
	 * have the hang fix hardware so bit 0 has no effect and is left 0.
	 * Bits [28:16] are FIFO/burst config; BSP defaults all to 0 (512-entry
	 * FIFO, 24x64 burst, canvas mode), so we write only the hang_flag bit.
	 */
	regmap_write(ge2d->map, GE2D_GEN_CTRL4,
		     (ge2d->has_hang_flag ? BIT(0) : 0) | ge2d->burst_ctrl4);

	/* Datapath duty-cycle: BSP inits to 0 (always on, no duty-cycle gating).
	 * bit 31 = dp_onoff_mode, bits [30:16] = dp_on_cnt,
	 * bit 15 = vfmt_onoff_en, bits [14:0] = dp_off_cnt.
	 */
	regmap_write(ge2d->map, GE2D_DP_ONOFF_CTRL, 0);

	/* GEN_CTRL0: SRC1 YC ratio flags and separate enable */
	reg = src_x_yc_ratio ? GE2D_SRC1_X_YC_RATIO : 0;
	if (src_nv)
		reg |= GE2D_SRC1_Y_YC_RATIO | GE2D_SRC1_SEP_EN;
	regmap_write(ge2d->map, GE2D_GEN_CTRL0, reg);

	regmap_write(ge2d->map, GE2D_GEN_CTRL1,
		     FIELD_PREP(GE2D_INTERRUPT_CTRL, 2) |
		     FIELD_PREP(GE2D_SRC2_BURST_SIZE_CTRL, 3) |
		     FIELD_PREP(GE2D_SRC1_BURST_SIZE_CTRL, 0x3f) |
		     FIELD_PREP(GE2D_SRC1_GB_ALPHA, 255));
	regmap_write(ge2d->map, GE2D_GEN_CTRL2,
		     (src_nv ? 0 : GE2D_SRC1_LITTLE_ENDIAN) |
		     (dst_nv ? 0 : GE2D_SRC2_LITTLE_ENDIAN) |
		     (dst_nv ? 0 : GE2D_DST_LITTLE_ENDIAN) |
		     GE2D_SRC2_COLOR_EXPAND_MODE |
		     GE2D_SRC1_COLOR_EXPAND_MODE |
		     FIELD_PREP(GE2D_DST1_COLOR_MAP,
				ge2d_kapi_fmts[job->dst_fmt].hw_map) |
		     FIELD_PREP(GE2D_DST1_FORMAT,
				dst_nv ? GE2D_FORMAT_8BIT :
				ge2d_kapi_fmts[job->dst_fmt].hw_fmt) |
		     FIELD_PREP(GE2D_SRC2_COLOR_MAP,
				ge2d_kapi_fmts[job->dst_fmt].hw_map) |
		     FIELD_PREP(GE2D_SRC2_FORMAT,
				dst_nv ? GE2D_FORMAT_8BIT :
				ge2d_kapi_fmts[job->dst_fmt].hw_fmt) |
		     FIELD_PREP(GE2D_SRC1_COLOR_MAP,
				ge2d_kapi_fmts[job->src_fmt].hw_map) |
		     FIELD_PREP(GE2D_SRC1_FORMAT,
				ge2d_kapi_fmts[job->src_fmt].hw_fmt));

	/* GEN_CTRL3: DST1 enable + DST2 for NV12/NV21 output */
	if (dst_nv) {
		u32 dst2_map = ge2d_kapi_fmts[job->dst_fmt].hw_map >= 5 ?
			       ge2d_kapi_fmts[job->dst_fmt].hw_map - 5 : 0;

		regmap_write(ge2d->map, GE2D_GEN_CTRL3,
			     GE2D_DST1_ENABLE |
			     GE2D_DST2_ENABLE |
			     FIELD_PREP(GE2D_DST2_X_DISCARD_MODE, 3) |
			     FIELD_PREP(GE2D_DST2_Y_DISCARD_MODE, 3) |
			     FIELD_PREP(GE2D_DST2_COLOR_MAP, dst2_map) |
			     FIELD_PREP(GE2D_DST2_FORMAT, GE2D_FORMAT_16BIT));
	} else {
		regmap_write(ge2d->map, GE2D_GEN_CTRL3, GE2D_DST1_ENABLE);
	}

	/* CSC -- kapi always uses BT.601 (no colorspace in job struct) */
	if (src_yuv && !dst_yuv)
		ge2d_set_csc_matrix(ge2d, true, false);
	else if (!src_yuv && dst_yuv)
		ge2d_set_csc_matrix(ge2d, false, false);
	else
		regmap_write(ge2d->map, GE2D_MATRIX_COEF22_CTRL, 0);

	/* Chroma formatter: only for sources with chroma subsampling */
	if (src_x_yc_ratio) {
		u32 fmt_ctrl = GE2D_SRC1_HFMT_EN |
			       FIELD_PREP(GE2D_SRC1_X_CHR_PHASE, 0xc4) |
			       GE2D_SRC1_CHFMT_RPT_PIX |
			       GE2D_SRC1_CVFMT_RPT_PIX;

		/*
		 * NV12/NV21 sources have vertical chroma subsampling.
		 * BSP (ge2d_hw.c:1170-1178) sets y_chr_phase to 0xc4
		 * for YUV->RGB, 0x4c for YUV->YUV.
		 */
		if (src_nv) {
			u8 y_phase = !dst_yuv ? 0xc4 : 0x4c;

			fmt_ctrl |= GE2D_SRC1_VFMT_EN |
				    FIELD_PREP(GE2D_SRC1_Y_CHR_PHASE, y_phase);
		}

		regmap_write(ge2d->map, GE2D_SRC1_FMT_CTRL, fmt_ctrl);
	} else {
		regmap_write(ge2d->map, GE2D_SRC1_FMT_CTRL, 0);
	}

	/* Clip/window */
	regmap_write(ge2d->map, GE2D_SRC1_CLIPY_START_END,
		     FIELD_PREP(GE2D_END, job->src_h - 1));
	regmap_write(ge2d->map, GE2D_SRC1_CLIPX_START_END,
		     FIELD_PREP(GE2D_END, job->src_w - 1));
	regmap_write(ge2d->map, GE2D_SRC2_CLIPY_START_END,
		     FIELD_PREP(GE2D_END, job->dst_h - 1));
	regmap_write(ge2d->map, GE2D_SRC2_CLIPX_START_END,
		     FIELD_PREP(GE2D_END, job->dst_w - 1));
	regmap_write(ge2d->map, GE2D_DST_CLIPY_START_END,
		     FIELD_PREP(GE2D_END, job->dst_h - 1));
	regmap_write(ge2d->map, GE2D_DST_CLIPX_START_END,
		     FIELD_PREP(GE2D_END, job->dst_w - 1));
	/*
	 * SRC1_Y_START_END: same fix as v4l2 path -- set extra bits
	 * and expand Y end by +1 for 4:2:0 vertical chroma formatter.
	 */
	regmap_write(ge2d->map, GE2D_SRC1_Y_START_END,
		     (src_nv ? (2 << 30) : 0) |
		     FIELD_PREP(GE2D_END,
				src_nv ? job->src_h : job->src_h - 1) |
		     (src_nv ? (3 << 14) : 0));
	regmap_write(ge2d->map, GE2D_SRC1_X_START_END,
		     (src_x_yc_ratio && !dst_yuv ? (2 << 30) : 0) |
		     FIELD_PREP(GE2D_END, job->src_w - 1) |
		     (src_x_yc_ratio && !dst_yuv ? (3 << 14) : 0));
	regmap_write(ge2d->map, GE2D_SRC2_Y_START_END,
		     FIELD_PREP(GE2D_END, job->dst_h - 1));
	regmap_write(ge2d->map, GE2D_SRC2_X_START_END,
		     FIELD_PREP(GE2D_END, job->dst_w - 1));
	regmap_write(ge2d->map, GE2D_DST_Y_START_END,
		     FIELD_PREP(GE2D_END, job->dst_h - 1));
	regmap_write(ge2d->map, GE2D_DST_X_START_END,
		     FIELD_PREP(GE2D_END, job->dst_w - 1));

	/* Scaler */
	if (job->src_w != job->dst_w || job->src_h != job->dst_h) {
		u32 hsc_phase_step, vsc_phase_step;

		hsc_phase_step = (u32)(((u64)job->src_w << 18) /
				       job->dst_w) << 6;
		vsc_phase_step = (u32)(((u64)job->src_h << 18) /
				       job->dst_h) << 6;

		ge2d_load_scale_coef(ge2d, filt_coef_bilinear);

		regmap_write(ge2d->map, GE2D_SC_MISC_CTRL,
			     GE2D_SC_VSC_EN | GE2D_SC_HSC_EN |
			     FIELD_PREP(GE2D_SC_VSC_BANK_LENGTH, 4) |
			     FIELD_PREP(GE2D_SC_HSC_BANK_LENGTH, 4));
		regmap_write(ge2d->map, GE2D_HSC_START_PHASE_STEP,
			     hsc_phase_step);
		regmap_write(ge2d->map, GE2D_HSC_PHASE_SLOPE, 0);
		regmap_write(ge2d->map, GE2D_HSC_INI_CTRL, 0);
		regmap_write(ge2d->map, GE2D_HSC_ADV_CTRL, 0);
		regmap_write(ge2d->map, GE2D_VSC_START_PHASE_STEP,
			     vsc_phase_step);
		regmap_write(ge2d->map, GE2D_VSC_PHASE_SLOPE, 0);
		regmap_write(ge2d->map, GE2D_VSC_INI_CTRL, 0);
		regmap_write(ge2d->map, GE2D_VSC_NRND_POINT, 0);
		regmap_write(ge2d->map, GE2D_VSC_NRND_PHASE, 0);
		regmap_write(ge2d->map, GE2D_HSC_NRND_POINT, 0);
		regmap_write(ge2d->map, GE2D_HSC_NRND_PHASE, 0);
	} else {
		regmap_write(ge2d->map, GE2D_SC_MISC_CTRL, 0);
		regmap_write(ge2d->map, GE2D_HSC_START_PHASE_STEP, 0);
		regmap_write(ge2d->map, GE2D_VSC_START_PHASE_STEP, 0);
		regmap_write(ge2d->map, GE2D_HSC_INI_CTRL, 0);
		regmap_write(ge2d->map, GE2D_VSC_INI_CTRL, 0);
		regmap_write(ge2d->map, GE2D_HSC_ADV_CTRL, 0);
		regmap_write(ge2d->map, GE2D_VSC_NRND_POINT, 0);
		regmap_write(ge2d->map, GE2D_VSC_NRND_PHASE, 0);
		regmap_write(ge2d->map, GE2D_HSC_NRND_POINT, 0);
		regmap_write(ge2d->map, GE2D_HSC_NRND_PHASE, 0);
	}

	/* ALU: copy source color and alpha */
	reg = GE2D_ALU_DO_COLOR_OPERATION_LOGIC(LOGIC_OPERATION_COPY,
						COLOR_FACTOR_SRC_COLOR);
	if (src_alpha && dst_alpha)
		reg |= GE2D_ALU_DO_ALPHA_OPERATION_LOGIC(LOGIC_OPERATION_COPY,
							  COLOR_FACTOR_SRC_ALPHA);
	else
		reg |= GE2D_ALU_DO_ALPHA_OPERATION_LOGIC(LOGIC_OPERATION_SET,
							  COLOR_FACTOR_ZERO);
	regmap_write(ge2d->map, GE2D_ALU_OP_CTRL, reg);

	/* Start */
	regmap_write(ge2d->map, GE2D_CMD_CTRL, GE2D_CBUS_CMD_WR);
}

static int ge2d_kapi_blit(struct meson_ge2d_dev *gdev,
			  struct meson_ge2d_job *job)
{
	struct meson_ge2d *ge2d = gdev->ge2d;
	u8 src1_canvas = 0, src1_canvas_uv = 0;
	u8 src2_canvas = 0;
	u8 dst1_canvas = 0, dst2_canvas = 0;
	bool src_nv, dst_nv;
	unsigned long timeout;
	int ret;

	if (!ge2d->has_baddr && !ge2d->canvas)
		return -ENOTSUPP;

	if (job->src_fmt >= ARRAY_SIZE(ge2d_kapi_fmts) ||
	    job->dst_fmt >= ARRAY_SIZE(ge2d_kapi_fmts))
		return -EINVAL;

	src_nv = ge2d_kapi_fmt_is_nv(job->src_fmt);
	dst_nv = ge2d_kapi_fmt_is_nv(job->dst_fmt);

	mutex_lock(&ge2d->kapi_lock);

	pm_runtime_get_sync(ge2d->dev);

	ret = ge2d_kapi_setup_addr(ge2d, job, src_nv, dst_nv,
				   &src1_canvas, &src1_canvas_uv,
				   &src2_canvas,
				   &dst1_canvas, &dst2_canvas);
	if (ret) {
		pm_runtime_mark_last_busy(ge2d->dev);
		pm_runtime_put_autosuspend(ge2d->dev);
		mutex_unlock(&ge2d->kapi_lock);
		return ret;
	}

	reinit_completion(&ge2d->kapi_done);
	ge2d->kapi_active = true;

	ge2d_kapi_hw_start(ge2d, job);

	timeout = wait_for_completion_timeout(&ge2d->kapi_done,
					      msecs_to_jiffies(500));

	if (!ge2d->has_baddr) {
		if (dst_nv)
			meson_canvas_free(ge2d->canvas, dst2_canvas);
		meson_canvas_free(ge2d->canvas, dst1_canvas);
		meson_canvas_free(ge2d->canvas, src2_canvas);
		if (src_nv)
			meson_canvas_free(ge2d->canvas, src1_canvas_uv);
		meson_canvas_free(ge2d->canvas, src1_canvas);
	}

	mutex_unlock(&ge2d->kapi_lock);

	if (!timeout) {
		ge2d->kapi_active = false;
		pm_runtime_mark_last_busy(ge2d->dev);
		pm_runtime_put_autosuspend(ge2d->dev);
		dev_err(ge2d->dev, "GE2D kernel blit timeout\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static const struct meson_ge2d_ops ge2d_kapi_ops = {
	.get = ge2d_kapi_get,
	.blit = ge2d_kapi_blit,
};

static void ge2d_remove(struct platform_device *pdev)
{
	struct meson_ge2d *ge2d = platform_get_drvdata(pdev);
	unsigned long flags;

	if (ge2d->cooling)
		thermal_cooling_device_unregister(ge2d->cooling);

	/*
	 * Tear down the watchdog.  Set ->removing under hw_lock FIRST so a
	 * concurrent device_run() (the v4l2-m2m scheduler stays live until
	 * v4l2_m2m_release() below) refuses to re-arm job_timer; then
	 * timer_delete_sync() catches any timer already armed.  Without the
	 * flag, a device_run() between this sync-delete and v4l2_m2m_release()
	 * re-arms the timer, which fires from the devm-freed struct after
	 * .remove() returns -- the captured SP/PC-alignment use-after-free
	 * panic (list-poison fn pointer in __run_timer_base, 2026-06-06).
	 */
	spin_lock_irqsave(&ge2d->hw_lock, flags);
	ge2d->removing = true;
	spin_unlock_irqrestore(&ge2d->hw_lock, flags);
	timer_delete_sync(&ge2d->job_timer);
	cancel_work_sync(&ge2d->job_timeout_work);

	meson_ge2d_unregister(&ge2d_kapi_ops);
	video_unregister_device(ge2d->vfd);
	v4l2_m2m_release(ge2d->m2m_dev);
	v4l2_device_unregister(&ge2d->v4l2_dev);
	pm_runtime_disable(&pdev->dev);
}

static int ge2d_runtime_suspend(struct device *dev)
{
	struct meson_ge2d *ge2d = dev_get_drvdata(dev);

	dev_dbg(dev, "runtime_suspend: disabling clk\n");
	clk_disable_unprepare(ge2d->clk);
	clk_disable_unprepare(ge2d->clk_gate);
	return 0;
}

static int ge2d_runtime_resume(struct device *dev)
{
	struct meson_ge2d *ge2d = dev_get_drvdata(dev);
	int ret;

	dev_dbg(dev, "runtime_resume: enabling clk+reset\n");

	/* Enable bus gate first so registers become accessible */
	ret = clk_prepare_enable(ge2d->clk_gate);
	if (ret) {
		dev_err(dev, "Cannot enable ge2d bus gate: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(ge2d->clk);
	if (ret) {
		dev_err(dev, "Cannot enable ge2d sclk: %d\n", ret);
		clk_disable_unprepare(ge2d->clk_gate);
		return ret;
	}

	clk_set_rate(ge2d->clk, ge2d_clock_rate(ge2d));
	dev_dbg(dev, "DFS: %luMHz (turbo=%d state=%lu)\n",
		clk_get_rate(ge2d->clk) / 1000000,
		ge2d->turbo, ge2d->thermal_state);

	reset_control_assert(ge2d->rst);
	if (ge2d_settle_us)
		udelay(ge2d_settle_us);
	reset_control_deassert(ge2d->rst);
	if (ge2d_settle_us)
		udelay(ge2d_settle_us);

	return 0;
}

static const struct dev_pm_ops ge2d_pm_ops = {
	SET_RUNTIME_PM_OPS(ge2d_runtime_suspend, ge2d_runtime_resume, NULL)
};

static const struct of_device_id meson_ge2d_match[] = {
	{ .compatible = "amlogic,axg-ge2d", .data = &ge2d_axg },
	{ .compatible = "amlogic,gxl-ge2d", .data = &ge2d_gxl },
	{ .compatible = "amlogic,g12a-ge2d", .data = &ge2d_g12a },
	{ .compatible = "amlogic,sm1-ge2d", .data = &ge2d_sm1 },
	{},
};

MODULE_DEVICE_TABLE(of, meson_ge2d_match);

static struct platform_driver ge2d_drv = {
	.probe = ge2d_probe,
	.remove = ge2d_remove,
	.driver = {
		.name = "meson-ge2d",
		.of_match_table = meson_ge2d_match,
		.pm = &ge2d_pm_ops,
		.dev_groups = ge2d_groups,
	},
};

module_platform_driver(ge2d_drv);

MODULE_AUTHOR("Neil Armstrong <narmstrong@baylibre.com>");
MODULE_DESCRIPTION("Amlogic 2D Graphic Acceleration Unit");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("DMA_BUF");
