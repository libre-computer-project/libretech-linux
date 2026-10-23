/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * Userspace ABI for the MediaTek MT8183 VP6 (IPU / Cadence Vision-P6) driver.
 *
 * One character device per VP6 core (/dev/vp6-0, /dev/vp6-1). See the internal
 * VP6 host-ABI contract for the details.
 */
#ifndef _UAPI_LINUX_MTK_VP6_H
#define _UAPI_LINUX_MTK_VP6_H

#include <linux/types.h>
#include <linux/ioctl.h>

#define VP6_IOC_MAGIC		0xB6

#define VP6_FW_MAGIC		0x49365056	/* 'V''P''6''I' */
#define VP6_FW_ABI_VER		1

#define VP6_IRAM_SIZE		(192 * 1024)	/* core-local IMEM (informational) */
#define VP6_DRAM_SIZE		(256 * 1024)	/* core-local DMEM (informational) */
#define VP6_IPI_DATA_MAX	48
#define VP6_FW_VER_LEN		16

/* core lifecycle state (struct vp6_info.state) */
#define VP6_STATE_OFF		0	/* not powered */
#define VP6_STATE_IDLE		1	/* powered, no firmware */
#define VP6_STATE_LOADED	2	/* firmware staged in DDR, halted */
#define VP6_STATE_RUNNING	3	/* released from reset */
#define VP6_STATE_CRASHED	4	/* watchdog / fault */

/*
 * Firmware image header, at file offset 0 of a VP6I image. Little-endian.
 *
 * The MT8183 VP6 (NeuroPilot lineage) boots from a DDR-resident image: the whole
 * payload is copied into a DMA buffer and the core's reset vector register
 * (xtensa_altresetvec) is set to (buffer + entry). The 192 KB IRAM / 256 KB DRAM
 * are the core's own L1 memories, not a host load window -- so the image is one
 * contiguous blob, not a PTCM/DTCM split (that was the older mt8173 model).
 */
struct vp6_fw_header {
	__u32 magic;			/* VP6_FW_MAGIC */
	__u32 abi_version;		/* VP6_FW_ABI_VER */
	__u32 entry;			/* reset-vector byte offset within the payload */
	__u32 flags;			/* reserved, 0 */
	__u32 image_size;		/* payload bytes following this header */
	__u32 load_align;		/* required DDR alignment (0 = 4 KB default) */
	__u8  fw_ver[VP6_FW_VER_LEN];	/* NUL-terminated build tag */
	__u32 reserved[6];		/* must be 0 */
};

/* Fixed-size mailbox message (see vp6-host-abi.md §3). */
struct vp6_ipi_msg {
	__s32 id;			/* channel id; <0 invalid */
	__u32 len;			/* payload bytes, <= VP6_IPI_DATA_MAX */
	__u8  data[VP6_IPI_DATA_MAX];
};

/* reserved channel ids (>=16 are runtime-defined / opaque) */
#define VP6_IPI_INIT		0	/* core->host: boot done, data=fw_ver */
#define VP6_IPI_NOP		1	/* ping */
#define VP6_IPI_SUBMIT		2	/* host->core: job */
#define VP6_IPI_COMPLETE	3	/* core->host: job done */

struct vp6_info {
	__u32 core_index;		/* 0 or 1 */
	__u32 state;			/* VP6_STATE_* */
	__u32 iram_size;
	__u32 dram_size;
	__u8  fw_ver[VP6_FW_VER_LEN];
	__u32 abi_version;		/* driver-supported VP6_FW_ABI_VER */
	__u32 caps;			/* bitmask, see VP6_CAP_* */
};

/* vp6_info.caps: which hardware paths the driver can drive on this silicon */
#define VP6_CAP_LOAD		(1u << 0) /* DDR firmware staging + altresetvec works */
#define VP6_CAP_CTRL		(1u << 1) /* reset/boot control regs known: START works */
#define VP6_CAP_MAILBOX		(1u << 2) /* doorbell/info offsets known: IPI works */
#define VP6_CAP_LOAD_ELF	(1u << 3) /* fixed-MVA multi-segment ELF loader (M4U) */

/*
 * vp6_fw_load.flags for VP6_IOC_LOAD_ELF:
 *
 * A vendor VP6 firmware image (cam_vpu*.img) is a multi-PT_LOAD Xtensa ELF whose
 * external code/data segments carry ABSOLUTE literals at fixed VMAs
 * (0x7da00000 reset vector, 0x7db00000 main program) that must be placed at
 * those exact M4U IOVAs, PLUS on-core L1 segments at/above the DSP-local base
 * (0x7ff00000: IRAM/DMEM) whose load address is the core's OWN local memory.
 *
 * The MT8183 "direct-host" VP6 generation exposes NO AP-visible window into the
 * core's local IRAM/DMEM (the mt6771 == MT8183-die BSP stages the IRAM copy in
 * DRAM at a separate MVA and relies on the DSP's own startup to self-copy it --
 * a protocol the RV33-generation mt8188 blob, built to be placed directly by the
 * apusys_rv microcontroller, does not implement). So this loader can place the
 * external segments but NOT the on-core ones. By default a blob that carries
 * on-core segments is rejected (-EOPNOTSUPP); set ALLOW_PARTIAL to load the
 * placeable segments anyway and observe the on-silicon boot behaviour.
 *
 * BOOTSTRAP supplies the missing DSP-self-copy handshake so the on-core segments
 * DO get placed. The loader stages each on-core segment's bytes in DRAM (an M4U
 * IOVA), builds a descriptor table in DRAM, places a small base-ISA "L1 self-copy"
 * bootstrap (open Xtensa GCC, no Cadence config), and points the reset vector at
 * the bootstrap instead of the blob entry. On START the DSP boots the bootstrap,
 * copies DRAM->on-core IRAM/DMEM with base-ISA l32i/s32i (the proven Phase I/J
 * path), then jumps to the blob's real reset vector with its L1 populated. This is
 * the only path to running the RV33-generation vendor blob on the direct-host VP6
 * without the RV33/apusys stack. Mutually exclusive with ALLOW_PARTIAL.
 */
#define VP6_LOAD_ELF_ALLOW_PARTIAL	(1u << 0)
#define VP6_LOAD_ELF_BOOTSTRAP		(1u << 1)

/*
 * BOOTSTRAP_DIAG -- a diagnostic variant of BOOTSTRAP that does NOT copy L1 or
 * jump to the blob. It places a probe bootstrap that issues four independently
 * observable base-ISA stores to separate the two candidate causes of the Phase-O
 * copy fault: (1) the fixed-MVA iommu_map cluster not serving the DSP like a
 * dynamic dma_alloc_coherent IOVA, vs (2) an on-core L1 s32i escaping out the
 * AXI-via-M4U master. The driver reads the DRAM markers back via VP6_IOC_READ_DIAG;
 * the on-core store is judged by the dmesg M4U fault. Mutually exclusive with
 * BOOTSTRAP / ALLOW_PARTIAL. See mt8183-vp6-status.md Phase P.
 */
#define VP6_LOAD_ELF_BOOTSTRAP_DIAG	(1u << 2)

/*
 * BOOTSTRAP_DIAG2 -- a CONTROL-FLOW disambiguation probe (Phase Q). Phase P
 * REFUTED both memory-path causes (fixed-MVA mapping serves the DSP; a base-ISA
 * s32i into on-core DMEM lands) and localized the Phase-O bootstrap fault to the
 * VP6 FLIX config mis-decoding base-ISA control-flow (the bootstrap is the only
 * proven-VP6 code that uses beqz / a counted loop / jx; every clean path is
 * straight-line). This probe runs three isolated micro-tests -- a beqz pair, a
 * counted loop, and a jx to a driver-patched runtime target -- each writing
 * host-readable markers to the DYNAMIC scratch, so VP6_IOC_READ_DIAG2 + a dmesg
 * M4U-fault check pin EXACTLY which control-flow form the FLIX mis-decodes.
 * Mutually exclusive with BOOTSTRAP / BOOTSTRAP_DIAG / ALLOW_PARTIAL.
 */
#define VP6_LOAD_ELF_BOOTSTRAP_DIAG2	(1u << 3)

/*
 * BOOTSTRAP_DIAG3 -- an L1-COPY-OP disambiguation probe (Phase R). Phase Q REFUTED
 * control-flow (beqz/loop/jx all decode) yet the fully-unrolled straight-line copy
 * bootstrap STILL faults, so the residual is the L1 COPY op. This probe isolates the
 * TWO axes that separate the faulting copy from every proven-good path (DIAG1 tested
 * NEITHER): (A) the MEMORY TARGET -- the copy READs the fixed-MVA STAGING buffer
 * (0x7dc20000; DIAG1 read only the descriptor 0x7dc10000) and STOREs to on-core IRAM
 * (0x7ff40000; DIAG1 only stored to DMEM 0x7ff3e000, which LANDED); (B) the ENCODING
 * WIDTH -- every proven path is NARROW density (l32i.n/s32i.n) but the generator
 * emits WIDE 24-bit l32i/s32i (op0=2). It runs seven sub-ops crossing both axes
 * (staging read; narrow copy->DMEM; WIDE store/load to SAFE targets; narrow
 * copy->IRAM; WIDE copy->DMEM; WIDE copy->IRAM) writing host-readable markers to the
 * DYNAMIC scratch; VP6_IOC_READ_DIAG2 reads them back + a dmesg M4U-fault check pins
 * which sub-op faults. Mutually exclusive with BOOTSTRAP / DIAG / DIAG2 / ALLOW_PARTIAL.
 */
#define VP6_LOAD_ELF_BOOTSTRAP_DIAG3	(1u << 4)

/*
 * BOOTSTRAP_DIAG4 -- a MULTI-WORD copy disambiguation probe (Phase R, 2nd lever).
 * DIAG3 proved every SINGLE copy sub-op lands (staging read, narrow+wide copy to
 * DMEM and to on-core IRAM) yet the production unrolled copy STILL faults, so the
 * residual is multi-word/interaction-specific. This probe isolates the two elements
 * DIAG3's 1-word tests never exercised: the `addmi` walking-base op (emitted only
 * for segments >256 words -- cam_vpu1's IRAM segment) and a long unbroken run of
 * back-to-back l32i/s32i. It runs: T1 an addmi and stores the RESULT register value
 * (== 0x7ff3e400 iff addmi decodes); T2 a 160-word unbroken copy->DMEM with no addmi;
 * T3 a >256-word copy->IRAM that fires one addmi walk (the production pattern). The
 * driver seeds VP6_MVA_L1_STAGE[0..511] = VP6_DIAG_MARKER. VP6_IOC_READ_DIAG2 reads
 * the markers back + a dmesg M4U-fault check pins addmi-mis-decode vs run-length.
 * Mutually exclusive with BOOTSTRAP / DIAG / DIAG2 / DIAG3 / ALLOW_PARTIAL.
 */
#define VP6_LOAD_ELF_BOOTSTRAP_DIAG4	(1u << 5)

/*
 * BOOTSTRAP_DIAG5 -- a DESCRIPTOR-l32i-BASE copy disambiguation probe (Phase S).
 * DIAG3/DIAG4 proved every isolated copy sub-op lands (staging read; narrow+wide copy
 * to DMEM/IRAM; the addmi walk; a long run; the full >256+addmi->IRAM production shape)
 * yet the production unrolled copy STILL faults. Every prior probe loaded the src/dst
 * copy bases via `l32r` LITERALS; the production vp6_gen_bootstrap loads them via
 * `l32i a5,a2,off` FROM A LIVE DESCRIPTOR held in a2 -- the ONE remaining untested
 * delta. The driver builds a real 11-word descriptor at VP6_MVA_BOOT_DESC (desc[0] =
 * the dynamic scratch IOVA; desc[1..10] = the five on-core-dst/staging-src pairs) and
 * seeds VP6_MVA_L1_STAGE[0..1023] = VP6_DIAG_MARKER. T1 stores a marker through the
 * l32i-descriptor-loaded base (w2) while w0/w1 use an l32r-literal base -- w0/w1 land +
 * w2 does not => the l32i-loaded-base store is the fault (DECODE). T2 then runs the full
 * 5-segment copy with descriptor-l32i bases across three interleaved pointer regions
 * (descriptor 0x7dc1 / staging 0x7dc2 / on-core 0x7ff4) -- an M4U TRANSLATION fault =
 * fixable decode; a WEDGE / mtk-wdt reset with no fault = the feasibility.md 3.1 depth-4
 * read-queue overflow (hardware). VP6_IOC_READ_DIAG2 reads w0..w11 back. Mutually
 * exclusive with BOOTSTRAP / DIAG / DIAG2 / DIAG3 / DIAG4 / ALLOW_PARTIAL.
 */
#define VP6_LOAD_ELF_BOOTSTRAP_DIAG5	(1u << 6)

/*
 * BOOTSTRAP_DIAG6 -- a jx-HANDOFF isolation probe (Phase S 2nd lever). DIAG5 proved the
 * full descriptor-l32i multi-region L1 copy lands clean, yet the production bootstrap
 * (copy + `jx a3` into the blob entry 0x7da00000) still faults `read 0xbdb0cc00` on the
 * same kernel -- so the residual is the jx handoff, not the copy. The driver writes a
 * base-ISA LANDING PAD over the head of the blob's reset-vector cluster
 * (VP6_MVA_RESET_VECTOR) and places a mini-bootstrap that does NO copy, writes w0=0x5a,
 * then `jx a3` (a3 = desc[0] = the pad's runtime entry). If the pad runs it writes
 * w1=0x66. w1==0x66 => the jx-to-external-blob-entry handoff WORKS (the production fault
 * is the blob's own code = terminal RV33); w1==0 + a dmesg read fault => the handoff
 * itself faults (fixable). VP6_IOC_READ_DIAG2 reads the words back. Mutually exclusive
 * with BOOTSTRAP / DIAG..DIAG5 / ALLOW_PARTIAL.
 */
#define VP6_LOAD_ELF_BOOTSTRAP_DIAG6	(1u << 7)

/*
 * RPROC -- load + boot the vendor OpenAMP remoteproc firmware (rproc-vp6-fw /
 * xann_0) as a minimal host virtio-rpmsg peer. Unlike the cam_vpu LOAD_ELF path
 * (fixed reset vector 0x7da00000, absolute per-PT_LOAD literals), this firmware
 * is a standard DRAM-resident remoteproc ELF: entry 0x50000000, all code/data in
 * host DRAM CARVEOUTS declared by its .resource_table (System ROM 0x50000000/2M,
 * System RAM 0x60000000/16M with the stack+heap ABOVE the loaded image, plus the
 * runtime rpmsg carveouts), and one 52-byte on-core .dram0.data at 0x7ff00000
 * (skipped -- the direct-host L1 wall; PARTIAL). The RPROC loader allocates +
 * M4U-maps each resource-table carveout at its FULL fixed device address (not
 * per-PT_LOAD), copies the ELF segments in by da, records the trace/log ring and
 * the VDEV status/vring fields, and sets altresetvec = e_entry. After START,
 * VP6_IOC_RPROC_READY drives the virtio ACK->DRIVER->FEATURES_OK->DRIVER_OK
 * handshake so the firmware's rpmsg_virtio_wait_remote_ready proceeds + kicks the
 * doorbell; VP6_IOC_READ_TRACE reads the firmware log ring (the RTOS-up banner).
 * See the internal VP6 rproc-shim record. Full rpmsg message framing +
 * flk_* dispatch is a later milestone. Mutually exclusive with the cam_vpu
 * BOOTSTRAP / DIAG / ALLOW_PARTIAL flags.
 */
#define VP6_LOAD_ELF_RPROC		(1u << 8)

/*
 * VP6_IOC_READ_DIAG result: the three DRAM marker words the BOOTSTRAP_DIAG probe
 * writes, read back from the driver's kernel VAs after START.
 *   scratch0 -- marker via the DYNAMIC dma_alloc_coherent IOVA (0x5a if the probe
 *               code was fetched from its fixed MVA and a dynamic store works).
 *   scratch1 -- the descriptor marker READ over the fixed MVA, echoed to scratch
 *               (== VP6 diag marker if a fixed-MVA read works).
 *   staging0 -- marker stored to the fixed-MVA staging buffer (== marker if a
 *               fixed-MVA write works).
 */
struct vp6_diag {
	__u32 scratch0;
	__u32 scratch1;
	__u32 staging0;
	__u32 flags;			/* reserved 0 */
};

/*
 * VP6_IOC_READ_DIAG2 result: the first 16 words of the BOOTSTRAP_DIAG2 probe's
 * dynamic scratch (see the probe's word map in mt8183-vp6-status.md Phase Q).
 *   w0  0x5a   entry sentinel (straight-line)
 *   w1  0xB1   before beqz test
 *   w2  0xEE   POISON: beqz-taken failed to branch (else 0)
 *   w3  0xB2   progress past beqz-taken
 *   w4  0xB4 ok / 0xEF wrong  beqz-not-taken
 *   w5  0xC1   before counted loop
 *   w6  0x44   loop accumulator (4*0x11 if the loop decodes)
 *   w7  0xD1   before jx
 *   w8  0xEC   POISON: jx did not jump (else 0)
 *   w9  0xD2   jx landed at the driver-patched runtime target
 *   w10 0x99   all tests completed
 */
struct vp6_diag2 {
	__u32 words[16];
};

/*
 * VP6_IOC_READ_TRACE (RPROC path): copy up to `size` bytes of the firmware's
 * trace/log ring (the RSC_TRACE carveout parsed from the loaded resource table,
 * 0x61110000 for rproc-vp6-fw) into the userspace buffer `buf`. `len` returns the
 * full ring length; `copied` the bytes copied. A non-empty RTOS/libmetal banner
 * here = the firmware booted (the Track-B gate). Valid after a VP6_LOAD_ELF_RPROC
 * load; -ENODEV otherwise.
 */
struct vp6_trace {
	__u64 buf;			/* in: userspace destination */
	__u32 size;			/* in: destination capacity */
	__u32 copied;			/* out: bytes copied */
	__u32 len;			/* out: full trace-ring length */
	__u32 reserved;
};

/*
 * VP6_IOC_RPMSG_SEND / VP6_IOC_RPMSG_RECV (RPROC path, after VP6_IOC_RPROC_READY):
 * one rpmsg message to / from the firmware's virtio-rpmsg peer. SEND frames `len`
 * payload bytes from `buf` behind a 16-byte rpmsg header {src, dst, 0, len, 0},
 * posts it on vring1 (the firmware's receive queue) and kicks the DSP. RECV copies
 * the next message the firmware returned on vring0 (payload up to `len`, header
 * fields into src / dst / len) and re-posts that buffer; -EAGAIN when none is
 * pending. The payload limit is VP6_RPMSG_PAYLOAD_MAX.
 */
#define VP6_RPMSG_PAYLOAD_MAX	496	/* 512-byte rpmsg buffer - 16-byte header */

struct vp6_rpmsg {
	__u64 buf;			/* userspace payload */
	__u32 len;			/* SEND: payload bytes; RECV: in capacity, out bytes */
	__u32 src;			/* SEND: host source address; RECV: sender's */
	__u32 dst;			/* SEND: firmware endpoint; RECV: our address */
	__u32 reserved;
};

struct vp6_fw_load {
	__u64 image;			/* userspace pointer to a VP6I image (LOAD_FW)
					 * or a raw Xtensa ELF (LOAD_ELF) */
	__u32 size;			/* image byte length */
	__u32 flags;
};

struct vp6_ipi_xfer {
	struct vp6_ipi_msg msg;
	__u32 timeout_ms;		/* SEND: ACK wait (0 = default) */
	__u32 flags;
};

struct vp6_buf_alloc {
	__u64 size;			/* in: bytes (page-rounded) */
	__u32 handle;			/* out: buffer id */
	__u32 flags;
	__u64 dma_addr;			/* out: address the VP6 iDMA sees */
	__u64 mmap_offset;		/* out: mmap() offset argument */
};

#define VP6_IOC_GET_INFO	_IOR (VP6_IOC_MAGIC, 0, struct vp6_info)
#define VP6_IOC_LOAD_FW		_IOW (VP6_IOC_MAGIC, 1, struct vp6_fw_load)
#define VP6_IOC_START		_IO  (VP6_IOC_MAGIC, 2)
#define VP6_IOC_HALT		_IO  (VP6_IOC_MAGIC, 3)
#define VP6_IOC_RESET		_IO  (VP6_IOC_MAGIC, 4)
#define VP6_IOC_SEND_IPI	_IOW (VP6_IOC_MAGIC, 5, struct vp6_ipi_xfer)
#define VP6_IOC_RECV_IPI	_IOR (VP6_IOC_MAGIC, 6, struct vp6_ipi_xfer)
#define VP6_IOC_ALLOC_BUF	_IOWR(VP6_IOC_MAGIC, 7, struct vp6_buf_alloc)
#define VP6_IOC_FREE_BUF	_IOW (VP6_IOC_MAGIC, 8, __u32)
#define VP6_IOC_LOAD_FW_NAME	_IOW (VP6_IOC_MAGIC, 9, char[64])
#define VP6_IOC_LOAD_ELF	_IOW (VP6_IOC_MAGIC, 10, struct vp6_fw_load)
#define VP6_IOC_READ_DIAG	_IOR (VP6_IOC_MAGIC, 11, struct vp6_diag)
#define VP6_IOC_READ_DIAG2	_IOR (VP6_IOC_MAGIC, 12, struct vp6_diag2)
#define VP6_IOC_RPROC_READY	_IO  (VP6_IOC_MAGIC, 13)		    /* RPROC: virtio DRIVER_OK + kick */
#define VP6_IOC_READ_TRACE	_IOWR(VP6_IOC_MAGIC, 14, struct vp6_trace)
#define VP6_IOC_RPMSG_SEND	_IOW (VP6_IOC_MAGIC, 15, struct vp6_rpmsg)
#define VP6_IOC_RPMSG_RECV	_IOWR(VP6_IOC_MAGIC, 16, struct vp6_rpmsg)

#endif /* _UAPI_LINUX_MTK_VP6_H */
