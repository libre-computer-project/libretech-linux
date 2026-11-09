/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Amlogic Meson HCODEC H.264 Encoder
 *
 * Copyright (C) 2024 LibreTech
 */

#ifndef __MESON_HCODEC_H
#define __MESON_HCODEC_H

#include <linux/completion.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-v4l2.h>

/*
 * JPEG hardware max: 1023 MCUs/axis (10-bit counter).
 *   Width: 1023 * 16 = 16368 px. Height: 512 * 16 = 8192 px full-quality.
 * H.264 max: 2048 px wide (7-bit MB counter), enforced separately.
 * CMA limits the usable max at runtime (dma_alloc_coherent -ENOMEM).
 */
#define HCODEC_MAX_WIDTH	16368
#define HCODEC_MAX_HEIGHT	8192

/*
 * H.264 firmware width limit: 128 MBs/row = 2048 px.
 * Encode produces garbage at 129+ MBs (7-bit column counter overflow).
 * JPEG firmware on the same HW supports up to 1023 MCUs/row = 16368 px.
 */
#define HCODEC_H264_MAX_WIDTH	2048
#define HCODEC_MIN_WIDTH	160
#define HCODEC_MIN_HEIGHT	128

/* Fixed-size firmware buffers (do not scale with resolution) */
#define HCODEC_ASSIST_SIZE	0xc0000		/* 768 KB */
#define HCODEC_DUMP_INFO_SIZE	0xa0000		/* 640 KB */

/* Firmware size */
#define HCODEC_MC_SIZE		(4096 * 8)

/* Default QP */
#define HCODEC_DEFAULT_QP	26

/* Maximum JPEG header size (SOI+DQT*2+DHT+SOF0+SOS, measured ~620 bytes) */
#define JPEGENC_HEADER_MAX	1024

/* CBR buffer sizes (fixed, after dump_info in layout) */
#define HCODEC_CBR_INFO_SIZE	0x2000		/* 8 KB */
#define HCODEC_CBR_TABLE_SIZE	0x800		/* 2 KB: QP table section */

/*
 * HCODEC working-set sub-buffers.  Each is allocated with its OWN
 * dma_alloc_coherent (KI-116 CMA-fragmentation fix) so contiguity is
 * only required per sub-buffer, not across the whole ~16 MB working set.
 * The engine registers already take independent base addresses, so this
 * is purely an allocator change.
 *
 * DCT / DEC0 / DEC1 are the H.264-only buffers (QDCT MB ring + two
 * reconstruction/reference frames).  JPEG is intra-only (no ME/QDCT-MB
 * ring, no reference frames), so the JPEG layout skips them entirely,
 * dropping the largest contiguous need from ~16 MB to ~1 MB.
 */
enum hcodec_sub_buf {
	HCODEC_SUB_DCT,		/* H.264 QDCT MB ring (skipped for JPEG) */
	HCODEC_SUB_DEC0,	/* H.264 reconstruction frame 0 (skipped for JPEG) */
	HCODEC_SUB_DEC1,	/* H.264 reference frame 1 (skipped for JPEG) */
	HCODEC_SUB_BS,		/* VLC bitstream workspace (SPS/PPS gen) */
	HCODEC_SUB_ASSIST,	/* firmware ASSIST scratch */
	HCODEC_SUB_DUMP_INFO,	/* firmware per-MB info dump */
	HCODEC_SUB_CBR,		/* CBR QP/block table */
	HCODEC_SUB_COUNT,
};

/**
 * struct hcodec_buf_layout - runtime-computed DMA buffer layout
 * @jpeg:        true for the JPEG (intra-only) layout: DCT/DEC0/DEC1 omitted
 * @dct_size:    DCT macroblock buffer (scales with MB count)
 * @dec_size:    DEC0/DEC1 reference frame size each (scales with canvas)
 * @bs_size:     VLC bitstream output ring (scales with pixel count)
 * @sub_size:    per-sub-buffer allocation size (0 = sub-buffer not used)
 * @total_size:  sum of all sub_size (informational; for sizeimage hints)
 *
 * Each sub-buffer is allocated independently (see &enum hcodec_sub_buf);
 * the per-allocation bases live in &struct amvenc_ctx sub_vaddr/sub_paddr.
 */
struct hcodec_buf_layout {
	bool jpeg;
	u32 dct_size;
	u32 dec_size;
	u32 bs_size;
	u32 sub_size[HCODEC_SUB_COUNT];
	u32 total_size;
};

/* Weight/SAD defaults (overridable via module params in venc.c) */
#define ME_WEIGHT_DEFAULT	0x520
#define I4MB_WEIGHT_DEFAULT	0x655
#define I16MB_WEIGHT_DEFAULT	0x560

struct amvdec_core;

struct amvenc_ctx {
	struct v4l2_fh fh;
	struct amvdec_core *core;
	struct v4l2_ctrl_handler ctrl_handler;

	/* Encoding parameters */
	u32 width;
	u32 height;
	u32 gop_size;
	u32 qp;
	u32 jpeg_quality;	/* JPEG quality 1-100, default 85 */
	u32 jpeg_chroma;	/* V4L2_JPEG_CHROMA_SUBSAMPLING_420/422, default 420 */
	u32 h264_level;		/* V4L2_MPEG_VIDEO_H264_LEVEL_* enum value */
	bool force_keyframe;
	bool current_idr;	/* set in device_run for completion */

	/* H.264 state */
	u32 frame_number;
	u32 idr_pic_id;
	u32 pic_order_cnt_lsb;
	u32 frames_since_idr;

	/*
	 * Internal working buffers (DMA coherent), one independent
	 * dma_alloc_coherent per sub-buffer (KI-116 CMA-frag fix).
	 * Indexed by enum hcodec_sub_buf; NULL/0 = sub-buffer not allocated.
	 */
	void *sub_vaddr[HCODEC_SUB_COUNT];
	dma_addr_t sub_paddr[HCODEC_SUB_COUNT];

	/* Runtime DMA buffer layout (computed at S_FMT time) */
	struct hcodec_buf_layout buf_layout;
	/* sizes the sub-buffers were allocated at (for grow detection) */
	u32 sub_alloc_size[HCODEC_SUB_COUNT];

	/* JPEG custom quantization tables (user-provided; all-zero = use standard) */
	u8 jpeg_luma_quant_user[64];
	u8 jpeg_chroma_quant_user[64];

	/* JPEG effective quantization tables (computed by jpegenc_compute_tables) */
	u8 jpeg_eff_luma_qt[64];
	u8 jpeg_eff_chroma_qt[64];

	/* JPEG zero-copy header state (computed before encode, used at completion) */
	u8 jpeg_hdr[JPEGENC_HEADER_MAX];
	u32 jpeg_hdr_len;
	u32 jpeg_vlc_off;	/* ALIGN(hdr_len, 8) offset for VLC start */

	/* H.264 zero-copy header state (computed before encode, used at completion) */
	u32 h264_hdr_len;	/* pre-computed SPS/PPS length for current frame */
	u32 h264_vlc_off;	/* ALIGN(hdr_len, 8) VLC start offset in CAPTURE buffer */

	/* Canvas packed IDs (Y|U|V as 24-bit packed) */
	u32 dblk_canvas;
	u32 ref_canvas;

	/* Global canvas IDs: [0..2]=dblk Y/UV/UV, [3..5]=ref Y/UV/UV,
	 * [6..8]=input Y/U(UV)/V */
	u8 canvas_ids[9];
	u32 canvas_num;

	/* Quantization tables */
	u32 quant_tbl_i4[8];
	u32 quant_tbl_i16[8];
	u32 quant_tbl_me[8];

	/* Sequence counters */
	u32 sequence_out;
	u32 sequence_cap;

	/* MFDIN mode: 0=H264_I_PIC_ALL_4x4, 2=JPEG_ALL_8x8 */
	u32 mfdin_mode;

	/* Noise reduction mode: 0=off, 1=SNR, 2=TNR, 3=3DNR */
	u32 nr_mode;

	/* ME/IE weight offsets (BSP tuning parameters) */
	u32 me_weight;
	u32 i4mb_weight;
	u32 i16mb_weight;

	/* Rate control */
	u32 bitrate;		/* target bits/sec, 0 = disabled */
	u32 bitrate_mode;	/* V4L2_MPEG_VIDEO_BITRATE_MODE_* */
	bool rc_enable;		/* frame-level rate control active */
	u32 max_qp;		/* upper QP bound for RC */
	u32 target_bits;	/* bits per frame */
	u32 last_frame_bits;	/* actual bits of previous frame */
	u32 fps;		/* frame rate from VIDIOC_S_PARM, default 30 */

	/* Multi-slice */
	u32 slice_max_mb;	/* max MBs per slice, 0 = single slice */

	/* CBR block dimensions (in macroblocks) */
	u16 block_w;		/* MBs per block in X */
	u16 block_h;		/* MBs per block in Y */
	u16 block_w_n;		/* number of blocks in X */
	u16 block_h_n;		/* number of blocks in Y */

	/* Firmware-generated SPS/PPS cache */
	u8 sps_pps[128];
	u32 sps_pps_len;

	/* Cached firmware for per-frame reload (DMA coherent) */
	void *fw_vaddr;
	dma_addr_t fw_paddr;
	u32 fw_size;

	/* Dummy frame for IE SRAM seeding (deterministic encode) */
	bool ie_sram_seeded;

	/* Format info */
	struct v4l2_pix_format_mplane src_fmt;
	struct v4l2_pix_format_mplane dst_fmt;
};

/* Hardware ops (hcodec.c) */
int hcodec_poweron(struct amvdec_core *core);
void hcodec_poweroff(struct amvdec_core *core);
int hcodec_load_firmware(struct amvdec_core *core);
int hcodec_cache_firmware(struct amvdec_core *core,
			  struct amvenc_ctx *ctx);
void hcodec_free_cached_firmware(struct amvenc_ctx *ctx);
int hcodec_reload_firmware(struct amvdec_core *core,
			   struct amvenc_ctx *ctx);
void hcodec_reset(struct amvdec_core *core);
void hcodec_start(struct amvdec_core *core);
void hcodec_stop(struct amvdec_core *core);

int hcodec_alloc_canvases(struct amvdec_core *core,
			  struct amvenc_ctx *ctx);
void hcodec_free_canvases(struct amvdec_core *core,
			  struct amvenc_ctx *ctx);
void hcodec_setup_buffers(struct amvdec_core *core,
			  struct amvenc_ctx *ctx,
			  dma_addr_t vlc_dst, u32 vlc_dst_size);
void hcodec_setup_canvases(struct amvdec_core *core,
			   struct amvenc_ctx *ctx);
void hcodec_setup_encoder(struct amvdec_core *core,
			  struct amvenc_ctx *ctx, bool idr);
void hcodec_setup_mfdin(struct amvdec_core *core,
			struct amvenc_ctx *ctx,
			dma_addr_t y_addr, dma_addr_t uv_addr,
			dma_addr_t v_addr, u32 canvas_w);
void hcodec_setup_ie_me(struct amvdec_core *core,
			struct amvenc_ctx *ctx, u32 quant, bool idr);
void hcodec_encode(struct amvdec_core *core,
		   struct amvenc_ctx *ctx, bool idr,
		   dma_addr_t y_addr, dma_addr_t uv_addr,
		   dma_addr_t v_addr, u32 canvas_w,
		   dma_addr_t vlc_dst, u32 vlc_dst_size);
u32 hcodec_build_sps_pps(struct amvdec_core *core,
			  struct amvenc_ctx *ctx,
			  u8 *buf, u32 buf_size);
int hcodec_fw_generate_sps_pps(struct amvdec_core *core,
			       struct amvenc_ctx *ctx);
int hcodec_seed_ie_sram(struct amvdec_core *core,
			struct amvenc_ctx *ctx);

void hcodec_compute_buf_layout(u32 width, u32 height, bool jpeg,
			       struct hcodec_buf_layout *layout);

/* Rate control (hcodec.c) */
void hcodec_compute_block_dims(struct amvenc_ctx *ctx);
void hcodec_rc_init(struct amvenc_ctx *ctx, u32 fps);
void hcodec_rc_get_qp(struct amvenc_ctx *ctx, bool idr);
void hcodec_rc_update(struct amvenc_ctx *ctx, u32 frame_bits);

/* Thermal (hcodec.c) */
void hcodec_thermal_update(struct amvdec_core *core, unsigned long rate);
int hcodec_power_get(struct amvdec_core *core);
void hcodec_power_put(struct amvdec_core *core);
void hcodec_power_init(struct amvdec_core *core);

/* Encoder V4L2 (venc.c) */
int amvenc_init(struct amvdec_core *core);
void amvenc_cleanup(struct amvdec_core *core);
/* N104: stop a live encode session during unbind, before the rails drop. */
void amvenc_quiesce(struct amvdec_core *core);
irqreturn_t amvenc_isr(int irq, void *data);
irqreturn_t amvenc_threaded_isr(int irq, void *data);

#endif /* __MESON_HCODEC_H */
