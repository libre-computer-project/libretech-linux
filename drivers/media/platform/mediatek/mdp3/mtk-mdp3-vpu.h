/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 MediaTek Inc.
 * Author: Ping-Hsun Wu <ping-hsun.wu@mediatek.com>
 */

#ifndef __MTK_MDP3_VPU_H__
#define __MTK_MDP3_VPU_H__

#include <linux/platform_device.h>
#include "mtk-img-ipi.h"

enum mdp_ipi_result {
	MDP_IPI_SUCCESS	= 0,
	MDP_IPI_ENOMEM	= 12,
	MDP_IPI_EBUSY	= 16,
	MDP_IPI_EINVAL	= 22,
	MDP_IPI_EMINST	= 24,
	MDP_IPI_ERANGE	= 34,
	MDP_IPI_NR_ERRNO,

	MDP_IPI_EOTHER	= MDP_IPI_NR_ERRNO,
	MDP_IPI_PATH_CANT_MERGE,
	MDP_IPI_OP_FAIL,
};

struct mdp_ipi_init_msg {
	u32	status;
	u64	drv_data;
	u32	work_addr;	/* [in] working buffer address */
	u32	work_size;	/* [in] working buffer size */
} __packed;

struct mdp_ipi_deinit_msg {
	u32	status;
	u64	drv_data;
	u32	work_addr;
} __packed;

struct mdp_vpu_dev {
	/* synchronization protect for accessing vpu working buffer info */
	struct mutex		*lock;
	/* NULL until a frame needs the SCP config fallback (kernel-direct
	 * mode); set by mdp_vpu_scp_attach, cleared by mdp_vpu_scp_detach. */
	struct mtk_scp		*scp;
	/* serializes the SCP attach in the job path against detach */
	struct mutex		attach_lock;
	struct completion	ipi_acked;
	void			*param;
	dma_addr_t		param_addr;
	size_t			param_size;
	void			*work;
	dma_addr_t		work_addr;
	size_t			work_size;
	void			*config;
	dma_addr_t		config_addr;
	size_t			config_size;
	u32			status;
	/* mt8183 kernel-direct multi-job detile plan, published by mdp_vpu_process
	 * and consumed by mdp_m2m_device_run. detile_nchunks == 1 for every other
	 * path (SCP, mt8195 kernel-direct). A frame whose 64x32 tile grid exceeds
	 * IMG_MAX_SUBFRAMES_8183 (14) is split into detile_nchunks sequential GCE
	 * submissions, each <= 14 tiles, accumulating into the shared output dma-buf. */
	u32			detile_nchunks;
	u32			detile_ntiles;
	/* mt8183 separate-plane NV12 rotate: two I420-sized scratch buffers for the
	 * 5-pass pipeline (de-interleave -> rot Y/Cb/Cr single-plane -> re-interleave).
	 * dma_alloc_wc on the MDP pdev -> engine-accessible iovas. Allocated lazily in
	 * mdp_vpu_process, freed in mdp_vpu_shared_mem_free. */
	void			*scratch_a;
	dma_addr_t		scratch_a_addr;
	void			*scratch_b;
	dma_addr_t		scratch_b_addr;
	size_t			scratch_size;
	/* mt8183 HYBRID NV12 rotate: HW rotates the Y plane (GREY, bit-exact), the
	 * CPU rotates the chroma (de-interleave CbCr -> rotate -> re-interleave) into
	 * the output UV plane -- the WROT 3-plane I420 write is broken, so the chroma
	 * is done on the CPU in mdp_m2m_device_run. Published by mdp_vpu_process. */
	bool			hybrid_chroma;
	bool			hybrid_rev_rows;	/* vflip / rot180 (no-swap) */
	bool			hybrid_rev_cols;	/* hflip / rot180 (no-swap) */
	bool			hybrid_swap;		/* rot90/270: dim-swap chroma */
	bool			hybrid_ccw;		/* rot270 (vs rot90 clockwise) */
	/* mt8183 I420/YV12-DST HYBRID: the WROT cannot write a 3-plane planar dst
	 * (I420/YV12 -- confirmed HW wall). Instead the HW writes 2-plane NV12 (Y +
	 * interleaved CbCr, a proven op) into the SAME single output dma-buf (whose
	 * Y+UV region exactly spans the I420 Y+U+V planes), then the CPU de-interleaves
	 * the CbCr plane into the separate U/V planes (I420 = U then V, YV12 = V then U
	 * -- a pure byte reshuffle, bit-exact). Published by mdp_vpu_process, consumed
	 * by mdp_m2m_device_run's finalize. */
	bool			i420_deint;
	bool			i420_deint_yv12;	/* dst was YV12 (V,U order) vs I420 (U,V) */
	/* mt8183 MULTI-PLANAR YM12/YM21 dst variant: the U/V planes are SEPARATE
	 * dma-bufs, so the HW writes the interleaved NV12 UV plane into scratch_a
	 * (not contiguously after Y), and the CPU de-interleaves scratch_a -> the two
	 * separate plane vaddrs (vb2 plane 1 = U, plane 2 = V, swapped for YM21). */
	bool			i420_deint_mp;
	/* SCP-tail class #2: YM16/YM61 (3-plane planar 4:2:2) dst hybrid. Same
	 * HW-NV16-write + CPU-de-interleave as i420_deint_mp, but the chroma is
	 * FULL-height (4:2:2, hc == h) not half-height (4:2:0). Set alongside
	 * i420_deint + i420_deint_mp; selects the full-height CPU de-interleave. */
	bool			ym16_deint;
	/* mt8183 GREY (Y8, luma-only) SOURCE via CPU pre-expand: the RDMA has no Y8
	 * read mode that presents neutral chroma, so the CPU copies the Y plane and
	 * fills a neutral (0x80) UV plane into scratch_b, and the source descriptor is
	 * reconfigured to plain NV12 @ scratch_b -- every existing NV12->X compute then
	 * yields GREY->X. Set by mdp_vpu_process; the CPU Y-copy/UV-fill (which needs the
	 * src vaddr) runs in mdp_m2m_device_run before the GCE submit. scratch_b (not
	 * scratch_a) holds the NV12 so the YM12/YM21 mp arm keeps scratch_a for its UV. */
	bool			grey_src;
	/* True when mdp_vpu_process computed this job KERNEL-DIRECT (no SCP round-trip);
	 * false when it fell back to the SCP (SCP_IPI_MDP_FRAME send). Consumed by
	 * mdp_m2m_device_run to gate the per-context RSZ coeff-table override KD-only
	 * (config_rsz_frame must NOT touch the fragile SCP-computed config). */
	bool			kd_used;
};

void mdp_vpu_shared_mem_free(struct mdp_vpu_dev *vpu);
int mdp_vpu_dev_init(struct mdp_vpu_dev *vpu, struct mtk_scp *scp,
		     struct mutex *lock /* for sync */);
int mdp_vpu_dev_deinit(struct mdp_vpu_dev *vpu);
void mdp_vpu_scp_detach(struct mdp_vpu_dev *vpu);
int mdp_vpu_process(struct mdp_vpu_dev *vpu, struct img_ipi_frameparam *param);
void mdp_vpu_compute_8183_chunk(struct mdp_vpu_dev *vpu,
				struct img_ipi_frameparam *param, u32 chunk_idx);
void mdp_vpu_scratch_dump(struct mdp_vpu_dev *vpu);

#endif  /* __MTK_MDP3_VPU_H__ */
