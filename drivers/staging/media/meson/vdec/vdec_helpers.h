/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2018 BayLibre, SAS
 * Author: Maxime Jourdan <mjourdan@baylibre.com>
 */

#ifndef __MESON_VDEC_HELPERS_H_
#define __MESON_VDEC_HELPERS_H_

#include "vdec.h"

/**
 * amvdec_fw_identify - Compute SHA256 of a firmware buffer and look it up.
 *
 * @dev: device for dev_err/dev_info/dev_warn logging
 * @fwname: firmware file name (for log messages; ALSO used for the
 *          static-vs-dynamic firmware split -- see "Return values").
 * @data: firmware buffer
 * @size: buffer size in bytes
 * @table: per-codec known-versions table (may be NULL)
 * @n: number of entries in @table
 *
 * Logs the computed SHA256 + size.  If @table is non-NULL, searches for a
 * matching entry (size + full SHA256).  If matched, logs the label and any
 * "not latest" warning.
 *
 * Return values:
 *   valid pointer  -- @table non-NULL and hash matched an entry; caller
 *                     continues and sets sess->fw_match = match.
 *   NULL           -- @table was NULL or empty (unwired codec, backward
 *                     compat), OR @table non-NULL but hash not found AND
 *                     @fwname starts with "libre_" (dynamic libre firmware
 *                     dev mode -- warn-and-load).  Caller continues with
 *                     no quirks dispatch.
 *   ERR_PTR(-ENOENT) -- @table non-NULL, hash not found, AND @fwname does
 *                     NOT start with "libre_" (static BSP / upstream
 *                     firmware -- the only legitimate hash misses are
 *                     operator error or fuzzed binaries).  Caller MUST
 *                     refuse to load (check with IS_ERR()).
 *
 * Static-vs-dynamic split (added 2026-04-23 per /codec-stateless skill
 * rule 1a after a contradiction surfaced):
 *   - BSP / upstream firmware (gxl_h264.bin, g12a_hevc_mmu.bin, etc.):
 *     SHAs are static; ship from linux-firmware or vendor BSP. Refuse
 *     unknown SHAs to force operator verification (2026-04-16 HEVC G12B
 *     wrong-firmware incident).
 *   - libre stateless firmware (libre_<plat>_<codec>.bin): built fresh
 *     from amlogic codec/decoder/stateless/firmware/ on every M4.x
 *     iteration; SHA changes per build.  Refusing every dev iteration
 *     would gate firmware development on driver bookkeeping.  Warn-
 *     and-load with NULL fw_match (no quirks).
 */
const struct amvdec_fw_version *
amvdec_fw_identify(struct device *dev, const char *fwname,
		   const u8 *data, size_t size,
		   const struct amvdec_fw_version *table, size_t n);

/**
 * amvdec_set_canvases() - Map VB2 buffers to canvases
 *
 * @sess: current session
 * @reg_base: Registry bases of where to write the canvas indexes
 * @reg_num: number of contiguous registers after each reg_base (including it)
 */
int amvdec_set_canvases(struct amvdec_session *sess,
			u32 reg_base[], u32 reg_num[]);

/**
 * amvdec_set_scratch_canvases() - Program firmware canvas registers from raw
 * bridge-owned scratch paddrs (NON-ZERO-COPY MPEG-1/2 bridge).
 *
 * @sess: current session
 * @regs: array of canvas-ID register offsets (e.g. AV_SCRATCH_0 + i*4)
 * @paddrs: array of contiguous NV12 scratch buffer physical addresses
 * @count: number of canvases to program
 */
int amvdec_set_scratch_canvases(struct amvdec_session *sess,
				u32 regs[], dma_addr_t paddrs[], u32 count);

/**
 * amvdec_bind_dst_canvas() - Bind one CAPTURE buffer's canvas + fw-index map
 * on demand (for stateless clients that CREATE_BUFS buffers after STREAMON)
 *
 * @sess: current session
 * @vb: the CAPTURE vb2 buffer to bind
 * @reg_base: canvas-ID register block (e.g. AV_SCRATCH_0), 1 canvas per buffer
 * @max_slots: capacity of that register block (e.g. 8 for AV_SCRATCH_0..7)
 *
 * Returns the firmware index (existing if already mapped, else newly assigned)
 * or a negative errno.
 */
int amvdec_bind_dst_canvas(struct amvdec_session *sess, struct vb2_buffer *vb,
			   u32 reg_base, u32 max_slots);

void amvdec_flush_pending_frames(struct amvdec_session *sess);

/* True if the session uses the libre stateless V4L2 Request API path. */
bool amvdec_is_stateless(struct amvdec_session *sess);

/* Helpers to read/write to the various IPs (DOS, PARSER) */
u32 amvdec_read_dos(struct amvdec_core *core, u32 reg);
void amvdec_write_dos(struct amvdec_core *core, u32 reg, u32 val);
void amvdec_write_dos_bits(struct amvdec_core *core, u32 reg, u32 val);
void amvdec_clear_dos_bits(struct amvdec_core *core, u32 reg, u32 val);
u32 amvdec_read_parser(struct amvdec_core *core, u32 reg);
void amvdec_write_parser(struct amvdec_core *core, u32 reg, u32 val);

/*
 * FBC compressed body size per block.
 *
 * 8-bit decode uses "mem_saving_mode" which compresses each 64x32 block
 * to 3200 bytes (enabled via SAO_CTRL5 BIT(9), already set by the HEVC
 * codec init path).  10-bit decode uses 4096 bytes/block.  The allocator
 * MUST match what the hardware actually writes; undersized buffers
 * corrupt the next-adjacent allocation when decoding completes.
 *
 * When `is_10bit` is ignored or when you cannot cheaply know bit depth,
 * pass true to get the conservative 4096-byte sizing (always large
 * enough; wastes ~20% for 8-bit).
 */
u32 amvdec_am21c_body_size(u32 width, u32 height, bool is_10bit);
u32 amvdec_am21c_head_size(u32 width, u32 height);
u32 amvdec_am21c_size(u32 width, u32 height, bool is_10bit);

/**
 * amvdec_dst_buf_done_idx() - Signal that a buffer is done decoding
 *
 * @sess: current session
 * @buf_idx: hardware buffer index
 * @offset: VIFIFO bitstream offset corresponding to the buffer
 * @field: V4L2 interlaced field
 */
void amvdec_dst_buf_done_idx(struct amvdec_session *sess, u32 buf_idx,
			     u32 offset, u32 field);
void amvdec_dst_buf_done(struct amvdec_session *sess,
			 struct vb2_v4l2_buffer *vbuf, u32 field);
void amvdec_dst_buf_done_ts(struct amvdec_session *sess,
			    struct vb2_v4l2_buffer *vbuf, u32 field,
			    u64 timestamp, struct v4l2_timecode timecode,
			    u32 flags);
void amvdec_dst_buf_done_offset(struct amvdec_session *sess,
				struct vb2_v4l2_buffer *vbuf,
				u32 offset, u32 field, bool allow_drop);

/**
 * amvdec_add_ts() - Add a timestamp to the list
 *
 * @sess: current session
 * @ts: timestamp to add
 * @tc: timecode to add
 * @offset: offset in the VIFIFO where the associated packet was written
 * @flags: the vb2_v4l2_buffer flags
 */
int amvdec_add_ts(struct amvdec_session *sess, u64 ts,
		  struct v4l2_timecode tc, u32 offset, u32 flags);
void amvdec_remove_ts(struct amvdec_session *sess, u64 ts);
int amvdec_drop_stale_ts(struct amvdec_session *sess);

/**
 * amvdec_set_par_from_dar() - Set Pixel Aspect Ratio from Display Aspect Ratio
 *
 * @sess: current session
 * @dar_num: numerator of the DAR
 * @dar_den: denominator of the DAR
 */
void amvdec_set_par_from_dar(struct amvdec_session *sess,
			     u32 dar_num, u32 dar_den);

/**
 * amvdec_src_change() - Notify new resolution/DPB size to the core
 *
 * @sess: current session
 * @width: picture width detected by the hardware
 * @height: picture height detected by the hardware
 * @dpb_size: Decoded Picture Buffer size (= amount of buffers for decoding)
 */
void amvdec_src_change(struct amvdec_session *sess, u32 width,
		       u32 height, u32 dpb_size);

/**
 * amvdec_abort() - Abort the current decoding session
 *
 * @sess: current session
 */
void amvdec_abort(struct amvdec_session *sess);

/**
 * amvdec_scratch_init() - Allocate scratch DMA buffer and canvas entries
 *
 * @core: amvdec core
 *
 * Allocates a shared scratch buffer used by decoder engines to prevent
 * firmware from writing decoded frames to uninitialized canvas addresses
 * (which default to canvas ID 0 = DRM framebuffer after engine reset).
 */
int amvdec_scratch_init(struct amvdec_core *core);
void amvdec_scratch_free(struct amvdec_core *core);

/**
 * amvdec_fill_scratch_canvases() - Fill VDEC_1 canvas registers with scratch
 *
 * @core: amvdec core
 *
 * Writes the scratch canvas packed ID to all VDEC_1 output canvas registers
 * (AV_SCRATCH_0..L, ANC0_CANVAS_ADDR..+23) so firmware has safe DMA targets
 * before real capture buffers are configured.
 */
void amvdec_fill_scratch_canvases(struct amvdec_core *core);

/**
 * amvdec_fill_scratch_hevc() - Fill HEVC AXI table with scratch buffer
 *
 * @core: amvdec core
 *
 * Fills the HEVC ANC2AXI table and canvas data with safe scratch addresses
 * so the firmware doesn't write to uninitialized locations before real
 * capture buffers are configured via codec_hevc_setup_buffers().
 */
void amvdec_fill_scratch_hevc(struct amvdec_core *core);
#endif
