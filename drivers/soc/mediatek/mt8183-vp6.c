// SPDX-License-Identifier: GPL-2.0
/*
 * mtk-vp6: MediaTek MT8183 / MT8365 VP6 (Cadence Vision-P6) driver.
 *
 * The MT8183 (Helio-P60 / Genio 500 / i500) "IPU" is a pair of Cadence
 * Tensilica Vision-P6 (VP6) DSP cores (ipu_core0 @ 0x19180000, ipu_core1 @
 * 0x19280000) under the VPU_TOP power domain, fed by the ipu_conn fabric. The
 * MT8365 (Genio 350 / g350 / i350) carries a SINGLE VP6 core (apu_core @
 * 0x19180000, same cfg-block IP) off the apu syscon with NO ipu_conn fabric.
 * The per-SoC differences (core-syscon phandle name, fabric presence, base
 * clock set) are carried in struct vp6_soc_data (of_match .data); the core
 * register/boot/mailbox model is shared. This driver powers, loads firmware
 * onto, boots, and provides a host<->core mailbox + shared DDR buffers for each
 * core, exposed as /dev/vp6-N. On mt8183 the two cores are independent.
 *
 * Firmware: unlike mt8183 (no image existed), the MT8365 VP6 HAS a vendor
 * prebuilt -- an OpenAMP/remoteproc Xtensa image `rproc-vp6-fw` (installed as
 * firmware `xann_0`), consumable via the VP6_LOAD_ELF_RPROC path. It is built
 * for the i350 Cadence config MVPU6_0226 (NOT the i500's MVPU6_0413). Provenance
 * + sha256: the internal firmware inventory.
 *
 * It builds on the proven mt8183-ipu-probe reachability scaffold (same genpd
 * power sequence + register window) and adapts MediaTek's own NeuroPilot VP6
 * register/boot/mailbox model (the "mt67xx" P90 platform data, the closest
 * published VP6 sibling to the mt8183 P60 -- its clock-gate offsets match the
 * in-tree clk-mt8183-ipu0 driver exactly). The control-register offsets are that
 * template and are marked for on-silicon verification in the internal VP6
 * status record.
 *
 * Userspace ABI: the internal VP6 host-ABI contract and <uapi/linux/mtk-vp6.h>.
 *
 * NOTE: there is no MT8183 VP6 firmware image (building one needs the proprietary
 * Cadence Xtensa VP6 toolchain -- terminal BLOCKED-EXTERNAL, see the feasibility
 * doc). So the START -> core-runs -> IPI-back handshake cannot complete until a
 * firmware exists; every other path (power, clocks, register R/W, reset toggle,
 * doorbell write, buffer alloc/mmap) is exercisable without firmware.
 */
#include <linux/build_bug.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/elf.h>
#include <linux/firmware.h>
#include <linux/idr.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/miscdevice.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/poll.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/unaligned.h>

#include <uapi/linux/mtk-vp6.h>

/* ---- VP6 core control-register map (mt67xx / P90 template; see file header) -- */
#define VP6_CG_CON		0x000	/* clock-gate control (== clk-mt8183-ipu0 sta) */
#define VP6_CG_CLR		0x008	/* clock-gate clear    (== clk-mt8183-ipu0 clr) */
#define VP6_SW_RST		0x00c	/* software reset */
#define VP6_DONE_ST		0x10c	/* done / pwaitmode status */
#define VP6_CTRL		0x110	/* core control */
#define VP6_XTENSA_INT		0x114	/* core->host doorbell; plain R/W, the line follows it: write 0 = ack */
#define VP6_CTL_XTENSA_INT	0x118	/* host->core doorbell; set bit0 */
#define VP6_DEFAULT0		0x13c	/* AXI AR/AW user + qos */
#define VP6_DEFAULT1		0x140	/* iDMA AR/AW user */
#define VP6_DEFAULT2		0x144	/* debug enable */
#define VP6_INFO(n)		(0x150 + (n) * 4)	/* xtensa_info00..31 */
#define VP6_ALTRESETVEC		0x1f8	/* reset-vector (boot PC) */

/* RPROC diag: bounded M4U fault-poll window across the DSP release (vp6_hw_boot).
 * ~200k device readl's ~= tens of ms -- wide enough to latch a during-START M4U
 * fault, short enough to bound a clean boot. */
#define VP6_M4U_POLL_ITERS	200000

/* VP6_CTRL bits */
#define VP6_CTRL_P_DEBUG_EN	BIT(31)
#define VP6_CTRL_PBCLK_EN	BIT(26)
#define VP6_CTRL_STALL		BIT(23)
#define VP6_CTRL_STATE_VEC_SEL	BIT(19)
#define VP6_CTRL_PIF_GATED	BIT(17)
#define VP6_CTRL_PRID		0x1fffe
/* VP6_SW_RST bits */
#define VP6_RST_OCDHALT		BIT(12)
#define VP6_RST_APU_D		BIT(8)
#define VP6_RST_APU_B		BIT(4)
/* VP6_CG_CLR */
#define VP6_CG_JTAG		0x2
/*
 * VP6_DEFAULT2: the core's debug-module enables (mt67xx vpu_hw.c writes 0xf here behind
 * its jtag knob). The Xtensa performance monitor lives in the debug module: with these
 * clear, the core's ERI reads of PMG/PMCTRL/PM return 0 and nothing counts.
 */
#define VP6_D2_DBG_EN		0xf
/* VP6_DEFAULT0/1 */
#define VP6_D0_ARUSER		(0x8 << 23)
#define VP6_D0_AWUSER		(0x8 << 18)
#define VP6_D0_QOS_SWAP		BIT(28)
#define VP6_D1_ARUSER_IDMA	(0x8 << 0)
#define VP6_D1_AWUSER_IDMA	(0x8 << 5)
/*
 * "AXI Request via M4U" -- route the DSP's instruction/data AXI master through
 * the M4U so it fetches from M4U IOVAs (mt6771 vpu_hw.c:2065-2069, ptr_axi_0
 * bits 22/27 + ptr_axi_1 bits 4/9, guarded by !BYPASS_M4U_DBG). Without these
 * the reset-vector fetch of the (IOMMU) altresetvec resolves nowhere.
 */
#define VP6_D0_VIA_M4U		(BIT(22) | BIT(27))
#define VP6_D1_VIA_M4U		(BIT(4) | BIT(9))

/*
 * Fixed-MVA firmware layout for the ELF loader (VP6_IOC_LOAD_ELF).
 *
 * A vendor VP6 firmware (cam_vpu*.img) is a multi-PT_LOAD Xtensa ELF. Its
 * external code/data segments carry ABSOLUTE literals resolved at fixed VMAs,
 * so they must be placed at those exact M4U IOVAs -- reset vector at
 * VP6_MVA_RESET_VECTOR (== the mt6771 == MT8183-die BSP's VPU_MVA_RESET_VECTOR,
 * vpu/mt6771/vpu_hw.h:26), main program at VP6_MVA_MAIN_PROGRAM. This differs
 * from the flat LOAD_FW path, which stages a position-independent VP6I blob at a
 * dynamic IOVA.
 *
 * Segments at or above VP6_DSP_LOCAL_BASE are the core's OWN local L1 memories
 * (Xtensa IRAM/DMEM, DMEM base 0x7ff00000). On the MT8183 "direct-host"
 * generation there is no AP-visible window into them: the BSP stages the IRAM
 * image in DRAM at a separate MVA (vpu_hw.c:1504-1521, M4U_FLAGS_SG_READY) and
 * relies on the DSP firmware to self-copy it during startup. An RV33-generation
 * blob (mt8188 cam_vpu*) is placed directly by the apusys_rv microcontroller and
 * carries no such self-copy, so this loader cannot place its on-core segments.
 */
#define VP6_MVA_RESET_VECTOR	0x7da00000u
#define VP6_MVA_MAIN_PROGRAM	0x7db00000u
#define VP6_DSP_LOCAL_BASE	0x7ff00000u	/* Xtensa local IRAM/DMEM base */
#define VP6_ELF_MAX_CLUSTERS	12
#define VP6_ELF_MERGE_GAP	SZ_64K		/* merge PT_LOADs closer than this */
#define VP6_ELF_MAX_IMAGE	SZ_16M

/*
 * L1 self-copy bootstrap MVAs (VP6_LOAD_ELF_BOOTSTRAP path). The direct-host VP6
 * has no AP window into the core's local IRAM/DMEM, so the on-core PT_LOADs of an
 * RV33-generation blob cannot be placed by the AP. Instead the loader stages them
 * in DRAM and points the reset vector at a tiny base-ISA bootstrap that runs ON
 * the DSP and copies DRAM -> on-core L1 (the DSP CAN store into 0x7ff.. -- Phase
 * I/J), then jumps to the blob entry -- supplying the DSP-self-copy handshake the
 * mt6771 == MT8183-die BSP relies on (vpu/mt6771/vpu_hw.c:1504-1521) that the
 * mt8188 blob lacks. These three fixed MVAs sit below VP6_DSP_LOCAL_BASE and above
 * the blob's own segments (reset 0x7da00000, main+bss up to ~0x7dbf76c8), so they
 * never collide. The bootstrap and the descriptor MVA are compile-time constants
 * agreed with the internal VP6 bootstrap asm (lit_desc == VP6_MVA_BOOT_DESC), so no
 * run-time literal patching is needed.
 */
#define VP6_MVA_BOOTSTRAP	0x7dc00000u	/* bootstrap code (altresetvec target) */
#define VP6_MVA_BOOT_DESC	0x7dc10000u	/* descriptor table (matches lit_desc) */
#define VP6_MVA_L1_STAGE	0x7dc20000u	/* staged on-core file bytes */
#define VP6_BOOT_MAX_SEG	16		/* max on-core segment records */

/*
 * OpenAMP / remoteproc resource-table subset for the virtio-rpmsg shim
 * (VP6_LOAD_ELF_RPROC path, rproc-vp6-fw / xann_0). Layout mirrors the
 * firmware-side apu-docker/openamp/lsp/rsc_table.c: the table is
 * { u32 ver, num, reserved[2], offset[num] } and each offset[i] points at a
 * u32 type followed by that entry's body. Only the fields the host shim
 * touches are modelled; the bodies below START AT the field after `type`
 * (read separately at offset[i]).
 */
#define VP6_RSC_CARVEOUT	0
#define VP6_RSC_TRACE		2
#define VP6_RSC_VDEV		3	/* mainline fw_resource_type RSC_VDEV; the
					 * firmware's VDEV entry is type=3 (its id
					 * field is 7 = VIRTIO_ID_RPMSG -- the
					 * earlier `7` was that RE transcription
					 * error, so the VDEV scan never matched
					 * and rproc load returned -EINVAL). */
#define VP6_RSC_NAME_LEN	32
#define VP6_RSC_MAX_ENTRIES	64

struct vp6_rsc_table {
	u32 ver;
	u32 num;
	u32 reserved[2];
	u32 offset[];
};
struct vp6_fw_rsc_carveout {		/* body after the u32 type */
	u32 da, pa, len, flags, reserved;
	u8  name[VP6_RSC_NAME_LEN];
};
struct vp6_fw_rsc_trace {		/* body after the u32 type */
	u32 da, len, reserved;
	u8  name[VP6_RSC_NAME_LEN];
};
struct vp6_fw_rsc_vdev_vring {
	u32 da, align, num, notifyid, pa;
};
struct vp6_fw_rsc_vdev {		/* body after the u32 type */
	u32 id, notifyid, dfeatures, gfeatures, config_len;
	u8  status, num_vrings, reserved[2];
	struct vp6_fw_rsc_vdev_vring vring[];
};

/* virtio config status bits (openamp/virtio) driven by VP6_IOC_RPROC_READY */
#define VP6_VIRTIO_ACK		1
#define VP6_VIRTIO_DRIVER	2
#define VP6_VIRTIO_DRIVER_OK	4
#define VP6_VIRTIO_FEATURES_OK	8

#define VP6_RPROC_MAX_VRINGS	2

/*
 * L1 self-copy bootstrap -- FULLY-UNROLLED, STRAIGHT-LINE (Phase Q, driver-generated).
 *
 * The earlier LOOPED bootstrap (vp6-bootstrap.S, beqz + counted loop + jx) FAULTED
 * on silicon (Phase O). Phase P then REFUTED both memory-path causes (the fixed-MVA
 * iommu_map serves the DSP; a base-ISA s32i into on-core DMEM lands) and localized
 * the fault to the VP6 custom FLIX config MIS-DECODING base-ISA control-flow: every
 * clean VP6 path (Phase I/J stores, the MAC, the diag probe) is straight-line, and
 * the bootstrap alone used conditional branches + a counted-loop back-edge.
 *
 * So vp6_gen_bootstrap() below EMITS the copy as a fully-unrolled straight-line
 * sequence of l32i/s32i pairs (zero-fills as movi+s32i) with NO beqz and NO loop --
 * the proven Phase-J/diag idiom -- for every on-core L1 word. The only unavoidable
 * control-flow is the final `jx entry` handoff (a ~2 MB jump, out of `j` range),
 * whose decode is characterized separately by the BOOTSTRAP_DIAG2 probe. The
 * generated code layout at VP6_MVA_BOOTSTRAP:
 *   +0x00  lit_desc = VP6_MVA_BOOT_DESC (l32r literal; PC-relative, position-indep)
 *   +0x10  _start   (VP6_BOOTSTRAP_ENTRY_OFF): l32r a2,lit_desc; l32i a3,entry;
 *          then per segment {l32i dst; l32i src; unrolled copy/zero}; memw; isync;
 *          jx a3.
 * Descriptor (VP6_MVA_BOOT_DESC): [entry, dst0,src0, dst1,src1, ...] (src==0 => zero
 * fill). The bootstrap and the descriptor MVA are the compile-time constants the
 * generator bakes; no run-time literal patching. Instruction encodings are
 * assembler-validated (internal VP6 toolchain, build-diag2.sh / genval).
 */
#define VP6_BOOTSTRAP_ENTRY_OFF	0x00000010u	/* _start off; lit_desc at +0 */
#define VP6_BOOT_BLK_WORDS	256u		/* l32i/s32i imm8*4 spans 0..1020 = 256 words */

/*
 * Base-ISA L1-copy DISAMBIGUATION probe (VP6_LOAD_ELF_BOOTSTRAP_DIAG), assembled
 * from the internal VP6 diag bootstrap asm (build-bootstrap.sh
 * flow, open xtensa-elf-gcc 13.3.0, sha256
 * 05acf9e7dffe8747329cc0b866ad94cd354a73a91e4af38c1f48b97d1f2bd633). Same entry
 * offset (_start at VP6_BOOTSTRAP_ENTRY_OFF, after two literals) as the production
 * bootstrap. Payload word 0 (lit_scratch, VP6_DIAG_LIT_SCRATCH_OFF) is PATCHED at
 * load time with the dynamic dma_alloc_coherent scratch IOVA; word 1 (lit_desc) is
 * the fixed descriptor MVA VP6_MVA_BOOT_DESC. It issues four base-ISA stores --
 * scratch[0]=0x5a (dynamic IOVA), scratch[1]=marker (fixed-MVA read echo),
 * staging[0]=marker (fixed-MVA write), L1[0]=marker (on-core write) -- then spins.
 * The driver reads scratch[0]/scratch[1]/staging[0] back (VP6_IOC_READ_DIAG); the
 * on-core store is judged by the dmesg M4U fault. See mt8183-vp6-status.md Phase P.
 */
#define VP6_DIAG_LIT_SCRATCH_OFF	0x0u		/* payload off of lit_scratch */
#define VP6_DIAG_MARKER			0x0d1a6ed0u	/* descriptor marker word */
static const u8 vp6_bootstrap_diag_bin[64] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc1, 0x7d, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x21, 0xfc, 0xff, 0x5c, 0xa4, 0x49, 0x02, 0xc0,
	0x20, 0x00, 0x31, 0xfa, 0xff, 0x58, 0x03, 0x59, 0x12, 0xc0, 0x20, 0x00,
	0x68, 0x23, 0x59, 0x06, 0xc0, 0x20, 0x00, 0x78, 0x13, 0x59, 0x07, 0xc0,
	0x20, 0x00, 0x06, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
};

/*
 * Base-ISA CONTROL-FLOW disambiguation probe (VP6_LOAD_ELF_BOOTSTRAP_DIAG2),
 * assembled from the internal VP6 diag2 asm
 * (build-diag2.sh, open xtensa-elf-gcc 13.3.0, sha256
 * bb2e4adab8abf6d720397eb1e3fed65789ef38334afe2e2b0590ee8eb30c821f). Runs three
 * isolated micro-tests (beqz pair / counted loop / jx) writing host-readable
 * markers to the dynamic scratch (word map: internal VP6 status Phase Q); the
 * driver reads them back via VP6_IOC_READ_DIAG2. Two literals are patched at load:
 * lit_scratch (payload +0, VP6_DIAG_LIT_SCRATCH_OFF) = dynamic scratch IOVA, and
 * lit_jxtgt (payload +4) = VP6_MVA_BOOTSTRAP + VP6_DIAG2_JXTGT_OFF (the runtime
 * address of the jx target -- absolute, since the code runs at VP6_MVA_BOOTSTRAP).
 */
#define VP6_DIAG2_ENTRY_OFF	0x00000010u
#define VP6_DIAG2_JXTGT_OFF	0x00000084u
#define VP6_DIAG2_LIT_JXTGT_OFF	0x4u
static const u8 vp6_bootstrap_diag2_bin[160] = {
	0x40, 0x7c, 0x0a, 0x5c, 0x01, 0x67, 0xc7, 0x71, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x21, 0xfc, 0xff, 0x5c, 0xa3, 0x39, 0x02, 0xc0,
	0x20, 0x00, 0x32, 0xa0, 0xb1, 0x39, 0x12, 0xc0, 0x20, 0x00, 0x0c, 0x04,
	0x16, 0x84, 0x00, 0x32, 0xa0, 0xee, 0x32, 0x62, 0x02, 0xc0, 0x20, 0x00,
	0x32, 0xa0, 0xb2, 0x39, 0x32, 0xc0, 0x20, 0x00, 0x0c, 0x14, 0x8c, 0xa4,
	0x32, 0xa0, 0xb4, 0x39, 0x42, 0xc0, 0x20, 0x00, 0x06, 0x02, 0x00, 0x00,
	0x32, 0xa0, 0xef, 0x39, 0x42, 0xc0, 0x20, 0x00, 0x32, 0xa0, 0xc1, 0x39,
	0x52, 0xc0, 0x20, 0x00, 0x0c, 0x45, 0x0c, 0x06, 0x8c, 0x65, 0x62, 0xc6,
	0x11, 0x0b, 0x55, 0x46, 0xfd, 0xff, 0x69, 0x62, 0xc0, 0x20, 0x00, 0x32,
	0xa0, 0xd1, 0x39, 0x72, 0xc0, 0x20, 0x00, 0x71, 0xe4, 0xff, 0xa0, 0x07,
	0x00, 0x32, 0xa0, 0xec, 0x39, 0x82, 0xc0, 0x20, 0x00, 0x00, 0x00, 0x00,
	0x32, 0xa0, 0xd2, 0x39, 0x92, 0xc0, 0x20, 0x00, 0x32, 0xa0, 0x99, 0x39,
	0xa2, 0xc0, 0x20, 0x00, 0x06, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
};

/*
 * Base-ISA L1-COPY-OP disambiguation probe (VP6_LOAD_ELF_BOOTSTRAP_DIAG3, Phase R),
 * assembled from the internal VP6 diag3 asm
 * (build-diag3.sh, open xtensa-elf-gcc 13.3.0, sha256
 * 7a584f5e2280a7ffb6764c1e947f8cf0c8abe51babc0cd55e375b377e9ab7fdb). Phase Q
 * refuted control-flow yet the unrolled straight-line copy still faults; this probe
 * isolates the copy op across TWO axes -- the MEMORY TARGET (fixed-MVA STAGING read
 * @0x7dc20000 + on-core IRAM store @0x7ff40000, neither tested by DIAG1) and the
 * ENCODING WIDTH (WIDE 24-bit l32i/s32i op0=2, the generator's form, vs the
 * proven-good NARROW density). Seven sub-ops write host-readable markers (NARROW
 * stores, the proven path) to the DYNAMIC scratch; VP6_IOC_READ_DIAG2 reads the 16
 * words back + a dmesg M4U fault pins which sub-op faults. The two WIDE-encoding
 * controls (w6/w8) run on SAFE targets BEFORE any on-core test so a wide-decode
 * fault is never confounded with the IRAM target. Word map: mt8183-vp6-status.md
 * Phase R. Payload word 0 (lit_scratch, VP6_DIAG_LIT_SCRATCH_OFF) is PATCHED at load
 * with the dynamic scratch IOVA; words 1-3 (stage/dmem/iram) are fixed MVAs. The
 * driver seeds VP6_MVA_L1_STAGE[0] = VP6_DIAG_MARKER so reads/copies echo a known
 * value.
 */
#define VP6_DIAG3_ENTRY_OFF	0x00000010u
static const u8 vp6_bootstrap_diag3_bin[208] = {
	0x40, 0x7c, 0x0a, 0x5c, 0x00, 0x00, 0xc2, 0x7d, 0x00, 0xe0, 0xf3, 0x7f,
	0x00, 0x00, 0xf4, 0x7f, 0x21, 0xfc, 0xff, 0x41, 0xfc, 0xff, 0x5c, 0xa3,
	0x39, 0x02, 0xc0, 0x20, 0x00, 0x32, 0xa0, 0xa1, 0x39, 0x12, 0xc0, 0x20,
	0x00, 0x68, 0x04, 0x69, 0x22, 0xc0, 0x20, 0x00, 0x32, 0xa0, 0xb1, 0x39,
	0x32, 0xc0, 0x20, 0x00, 0x68, 0x04, 0x51, 0xf4, 0xff, 0x69, 0x05, 0xc0,
	0x20, 0x00, 0x78, 0x05, 0x79, 0x42, 0xc0, 0x20, 0x00, 0x32, 0xa0, 0xc1,
	0x39, 0x52, 0xc0, 0x20, 0x00, 0x88, 0x04, 0x82, 0x62, 0x06, 0xc0, 0x20,
	0x00, 0x32, 0xa0, 0xc2, 0x39, 0x72, 0xc0, 0x20, 0x00, 0x62, 0x24, 0x00,
	0x69, 0x82, 0xc0, 0x20, 0x00, 0x32, 0xa0, 0xd1, 0x39, 0x92, 0xc0, 0x20,
	0x00, 0x68, 0x04, 0x51, 0xe7, 0xff, 0x69, 0x05, 0xc0, 0x20, 0x00, 0x00,
	0x20, 0x00, 0x78, 0x05, 0x79, 0xa2, 0xc0, 0x20, 0x00, 0x32, 0xa0, 0xe1,
	0x39, 0xb2, 0xc0, 0x20, 0x00, 0x62, 0x24, 0x00, 0x51, 0xdf, 0xff, 0x62,
	0x65, 0x00, 0xc0, 0x20, 0x00, 0x78, 0x05, 0x79, 0xc2, 0xc0, 0x20, 0x00,
	0x32, 0xa0, 0xf1, 0x39, 0xd2, 0xc0, 0x20, 0x00, 0x62, 0x24, 0x00, 0x51,
	0xd9, 0xff, 0x62, 0x65, 0x00, 0xc0, 0x20, 0x00, 0x00, 0x20, 0x00, 0x78,
	0x05, 0x79, 0xe2, 0xc0, 0x20, 0x00, 0x32, 0xa0, 0x99, 0x39, 0xf2, 0xc0,
	0x20, 0x00, 0x06, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
};

/*
 * Base-ISA MULTI-WORD copy disambiguation probe (VP6_LOAD_ELF_BOOTSTRAP_DIAG4,
 * Phase R 2nd lever), assembled from
 * the internal VP6 diag4 asm (build-diag4.sh, open
 * xtensa-elf-gcc 13.3.0). DIAG3 proved every SINGLE copy sub-op lands yet the
 * production unrolled copy still faults; this probe isolates the two elements
 * DIAG3's 1-word tests never exercised -- the `addmi` walking-base op (emitted only
 * for segments >256 words, e.g. cam_vpu1's IRAM segment) and a long unbroken run of
 * back-to-back l32i/s32i. T1 stores the addmi RESULT register value (== 0x7ff3e400
 * iff addmi decodes); T2 a 160-word unbroken wide copy->DMEM (no addmi); T3 a
 * >256-word wide copy->IRAM that fires one addmi walk (the production pattern). The
 * driver seeds VP6_MVA_L1_STAGE[0..511] = VP6_DIAG_MARKER. Markers -> the dynamic
 * scratch (narrow stores); VP6_IOC_READ_DIAG2 reads them back. Word map:
 * mt8183-vp6-status.md Phase R. Payload word 0 (lit_scratch) PATCHED at load.
 */
#define VP6_DIAG4_ENTRY_OFF	0x00000010u
static const u8 vp6_bootstrap_diag4_bin[2624] = {
	0x40, 0x7c, 0x0a, 0x5c, 0x00, 0x00, 0xc2, 0x7d, 0x00, 0xe0, 0xf3, 0x7f,
	0x00, 0x00, 0xf4, 0x7f, 0x21, 0xfc, 0xff, 0x41, 0xfc, 0xff, 0x5c, 0xa3,
	0x39, 0x02, 0xc0, 0x20, 0x00, 0x32, 0xa0, 0xa1, 0x39, 0x12, 0xc0, 0x20,
	0x00, 0x51, 0xf8, 0xff, 0x52, 0xd5, 0x04, 0x59, 0x22, 0xc0, 0x20, 0x00,
	0x32, 0xa0, 0xb1, 0x39, 0x32, 0xc0, 0x20, 0x00, 0x51, 0xf4, 0xff, 0x62,
	0x24, 0x00, 0x62, 0x65, 0x00, 0x62, 0x24, 0x01, 0x62, 0x65, 0x01, 0x62,
	0x24, 0x02, 0x62, 0x65, 0x02, 0x62, 0x24, 0x03, 0x62, 0x65, 0x03, 0x62,
	0x24, 0x04, 0x62, 0x65, 0x04, 0x62, 0x24, 0x05, 0x62, 0x65, 0x05, 0x62,
	0x24, 0x06, 0x62, 0x65, 0x06, 0x62, 0x24, 0x07, 0x62, 0x65, 0x07, 0x62,
	0x24, 0x08, 0x62, 0x65, 0x08, 0x62, 0x24, 0x09, 0x62, 0x65, 0x09, 0x62,
	0x24, 0x0a, 0x62, 0x65, 0x0a, 0x62, 0x24, 0x0b, 0x62, 0x65, 0x0b, 0x62,
	0x24, 0x0c, 0x62, 0x65, 0x0c, 0x62, 0x24, 0x0d, 0x62, 0x65, 0x0d, 0x62,
	0x24, 0x0e, 0x62, 0x65, 0x0e, 0x62, 0x24, 0x0f, 0x62, 0x65, 0x0f, 0x62,
	0x24, 0x10, 0x62, 0x65, 0x10, 0x62, 0x24, 0x11, 0x62, 0x65, 0x11, 0x62,
	0x24, 0x12, 0x62, 0x65, 0x12, 0x62, 0x24, 0x13, 0x62, 0x65, 0x13, 0x62,
	0x24, 0x14, 0x62, 0x65, 0x14, 0x62, 0x24, 0x15, 0x62, 0x65, 0x15, 0x62,
	0x24, 0x16, 0x62, 0x65, 0x16, 0x62, 0x24, 0x17, 0x62, 0x65, 0x17, 0x62,
	0x24, 0x18, 0x62, 0x65, 0x18, 0x62, 0x24, 0x19, 0x62, 0x65, 0x19, 0x62,
	0x24, 0x1a, 0x62, 0x65, 0x1a, 0x62, 0x24, 0x1b, 0x62, 0x65, 0x1b, 0x62,
	0x24, 0x1c, 0x62, 0x65, 0x1c, 0x62, 0x24, 0x1d, 0x62, 0x65, 0x1d, 0x62,
	0x24, 0x1e, 0x62, 0x65, 0x1e, 0x62, 0x24, 0x1f, 0x62, 0x65, 0x1f, 0x62,
	0x24, 0x20, 0x62, 0x65, 0x20, 0x62, 0x24, 0x21, 0x62, 0x65, 0x21, 0x62,
	0x24, 0x22, 0x62, 0x65, 0x22, 0x62, 0x24, 0x23, 0x62, 0x65, 0x23, 0x62,
	0x24, 0x24, 0x62, 0x65, 0x24, 0x62, 0x24, 0x25, 0x62, 0x65, 0x25, 0x62,
	0x24, 0x26, 0x62, 0x65, 0x26, 0x62, 0x24, 0x27, 0x62, 0x65, 0x27, 0x62,
	0x24, 0x28, 0x62, 0x65, 0x28, 0x62, 0x24, 0x29, 0x62, 0x65, 0x29, 0x62,
	0x24, 0x2a, 0x62, 0x65, 0x2a, 0x62, 0x24, 0x2b, 0x62, 0x65, 0x2b, 0x62,
	0x24, 0x2c, 0x62, 0x65, 0x2c, 0x62, 0x24, 0x2d, 0x62, 0x65, 0x2d, 0x62,
	0x24, 0x2e, 0x62, 0x65, 0x2e, 0x62, 0x24, 0x2f, 0x62, 0x65, 0x2f, 0x62,
	0x24, 0x30, 0x62, 0x65, 0x30, 0x62, 0x24, 0x31, 0x62, 0x65, 0x31, 0x62,
	0x24, 0x32, 0x62, 0x65, 0x32, 0x62, 0x24, 0x33, 0x62, 0x65, 0x33, 0x62,
	0x24, 0x34, 0x62, 0x65, 0x34, 0x62, 0x24, 0x35, 0x62, 0x65, 0x35, 0x62,
	0x24, 0x36, 0x62, 0x65, 0x36, 0x62, 0x24, 0x37, 0x62, 0x65, 0x37, 0x62,
	0x24, 0x38, 0x62, 0x65, 0x38, 0x62, 0x24, 0x39, 0x62, 0x65, 0x39, 0x62,
	0x24, 0x3a, 0x62, 0x65, 0x3a, 0x62, 0x24, 0x3b, 0x62, 0x65, 0x3b, 0x62,
	0x24, 0x3c, 0x62, 0x65, 0x3c, 0x62, 0x24, 0x3d, 0x62, 0x65, 0x3d, 0x62,
	0x24, 0x3e, 0x62, 0x65, 0x3e, 0x62, 0x24, 0x3f, 0x62, 0x65, 0x3f, 0x62,
	0x24, 0x40, 0x62, 0x65, 0x40, 0x62, 0x24, 0x41, 0x62, 0x65, 0x41, 0x62,
	0x24, 0x42, 0x62, 0x65, 0x42, 0x62, 0x24, 0x43, 0x62, 0x65, 0x43, 0x62,
	0x24, 0x44, 0x62, 0x65, 0x44, 0x62, 0x24, 0x45, 0x62, 0x65, 0x45, 0x62,
	0x24, 0x46, 0x62, 0x65, 0x46, 0x62, 0x24, 0x47, 0x62, 0x65, 0x47, 0x62,
	0x24, 0x48, 0x62, 0x65, 0x48, 0x62, 0x24, 0x49, 0x62, 0x65, 0x49, 0x62,
	0x24, 0x4a, 0x62, 0x65, 0x4a, 0x62, 0x24, 0x4b, 0x62, 0x65, 0x4b, 0x62,
	0x24, 0x4c, 0x62, 0x65, 0x4c, 0x62, 0x24, 0x4d, 0x62, 0x65, 0x4d, 0x62,
	0x24, 0x4e, 0x62, 0x65, 0x4e, 0x62, 0x24, 0x4f, 0x62, 0x65, 0x4f, 0x62,
	0x24, 0x50, 0x62, 0x65, 0x50, 0x62, 0x24, 0x51, 0x62, 0x65, 0x51, 0x62,
	0x24, 0x52, 0x62, 0x65, 0x52, 0x62, 0x24, 0x53, 0x62, 0x65, 0x53, 0x62,
	0x24, 0x54, 0x62, 0x65, 0x54, 0x62, 0x24, 0x55, 0x62, 0x65, 0x55, 0x62,
	0x24, 0x56, 0x62, 0x65, 0x56, 0x62, 0x24, 0x57, 0x62, 0x65, 0x57, 0x62,
	0x24, 0x58, 0x62, 0x65, 0x58, 0x62, 0x24, 0x59, 0x62, 0x65, 0x59, 0x62,
	0x24, 0x5a, 0x62, 0x65, 0x5a, 0x62, 0x24, 0x5b, 0x62, 0x65, 0x5b, 0x62,
	0x24, 0x5c, 0x62, 0x65, 0x5c, 0x62, 0x24, 0x5d, 0x62, 0x65, 0x5d, 0x62,
	0x24, 0x5e, 0x62, 0x65, 0x5e, 0x62, 0x24, 0x5f, 0x62, 0x65, 0x5f, 0x62,
	0x24, 0x60, 0x62, 0x65, 0x60, 0x62, 0x24, 0x61, 0x62, 0x65, 0x61, 0x62,
	0x24, 0x62, 0x62, 0x65, 0x62, 0x62, 0x24, 0x63, 0x62, 0x65, 0x63, 0x62,
	0x24, 0x64, 0x62, 0x65, 0x64, 0x62, 0x24, 0x65, 0x62, 0x65, 0x65, 0x62,
	0x24, 0x66, 0x62, 0x65, 0x66, 0x62, 0x24, 0x67, 0x62, 0x65, 0x67, 0x62,
	0x24, 0x68, 0x62, 0x65, 0x68, 0x62, 0x24, 0x69, 0x62, 0x65, 0x69, 0x62,
	0x24, 0x6a, 0x62, 0x65, 0x6a, 0x62, 0x24, 0x6b, 0x62, 0x65, 0x6b, 0x62,
	0x24, 0x6c, 0x62, 0x65, 0x6c, 0x62, 0x24, 0x6d, 0x62, 0x65, 0x6d, 0x62,
	0x24, 0x6e, 0x62, 0x65, 0x6e, 0x62, 0x24, 0x6f, 0x62, 0x65, 0x6f, 0x62,
	0x24, 0x70, 0x62, 0x65, 0x70, 0x62, 0x24, 0x71, 0x62, 0x65, 0x71, 0x62,
	0x24, 0x72, 0x62, 0x65, 0x72, 0x62, 0x24, 0x73, 0x62, 0x65, 0x73, 0x62,
	0x24, 0x74, 0x62, 0x65, 0x74, 0x62, 0x24, 0x75, 0x62, 0x65, 0x75, 0x62,
	0x24, 0x76, 0x62, 0x65, 0x76, 0x62, 0x24, 0x77, 0x62, 0x65, 0x77, 0x62,
	0x24, 0x78, 0x62, 0x65, 0x78, 0x62, 0x24, 0x79, 0x62, 0x65, 0x79, 0x62,
	0x24, 0x7a, 0x62, 0x65, 0x7a, 0x62, 0x24, 0x7b, 0x62, 0x65, 0x7b, 0x62,
	0x24, 0x7c, 0x62, 0x65, 0x7c, 0x62, 0x24, 0x7d, 0x62, 0x65, 0x7d, 0x62,
	0x24, 0x7e, 0x62, 0x65, 0x7e, 0x62, 0x24, 0x7f, 0x62, 0x65, 0x7f, 0x62,
	0x24, 0x80, 0x62, 0x65, 0x80, 0x62, 0x24, 0x81, 0x62, 0x65, 0x81, 0x62,
	0x24, 0x82, 0x62, 0x65, 0x82, 0x62, 0x24, 0x83, 0x62, 0x65, 0x83, 0x62,
	0x24, 0x84, 0x62, 0x65, 0x84, 0x62, 0x24, 0x85, 0x62, 0x65, 0x85, 0x62,
	0x24, 0x86, 0x62, 0x65, 0x86, 0x62, 0x24, 0x87, 0x62, 0x65, 0x87, 0x62,
	0x24, 0x88, 0x62, 0x65, 0x88, 0x62, 0x24, 0x89, 0x62, 0x65, 0x89, 0x62,
	0x24, 0x8a, 0x62, 0x65, 0x8a, 0x62, 0x24, 0x8b, 0x62, 0x65, 0x8b, 0x62,
	0x24, 0x8c, 0x62, 0x65, 0x8c, 0x62, 0x24, 0x8d, 0x62, 0x65, 0x8d, 0x62,
	0x24, 0x8e, 0x62, 0x65, 0x8e, 0x62, 0x24, 0x8f, 0x62, 0x65, 0x8f, 0x62,
	0x24, 0x90, 0x62, 0x65, 0x90, 0x62, 0x24, 0x91, 0x62, 0x65, 0x91, 0x62,
	0x24, 0x92, 0x62, 0x65, 0x92, 0x62, 0x24, 0x93, 0x62, 0x65, 0x93, 0x62,
	0x24, 0x94, 0x62, 0x65, 0x94, 0x62, 0x24, 0x95, 0x62, 0x65, 0x95, 0x62,
	0x24, 0x96, 0x62, 0x65, 0x96, 0x62, 0x24, 0x97, 0x62, 0x65, 0x97, 0x62,
	0x24, 0x98, 0x62, 0x65, 0x98, 0x62, 0x24, 0x99, 0x62, 0x65, 0x99, 0x62,
	0x24, 0x9a, 0x62, 0x65, 0x9a, 0x62, 0x24, 0x9b, 0x62, 0x65, 0x9b, 0x62,
	0x24, 0x9c, 0x62, 0x65, 0x9c, 0x62, 0x24, 0x9d, 0x62, 0x65, 0x9d, 0x62,
	0x24, 0x9e, 0x62, 0x65, 0x9e, 0x62, 0x24, 0x9f, 0x62, 0x65, 0x9f, 0x72,
	0x25, 0x9f, 0x79, 0x42, 0xc0, 0x20, 0x00, 0x32, 0xa0, 0xc1, 0x39, 0x52,
	0xc0, 0x20, 0x00, 0x51, 0x00, 0xff, 0x62, 0x24, 0x00, 0x62, 0x65, 0x00,
	0x62, 0x24, 0x01, 0x62, 0x65, 0x01, 0x62, 0x24, 0x02, 0x62, 0x65, 0x02,
	0x62, 0x24, 0x03, 0x62, 0x65, 0x03, 0x62, 0x24, 0x04, 0x62, 0x65, 0x04,
	0x62, 0x24, 0x05, 0x62, 0x65, 0x05, 0x62, 0x24, 0x06, 0x62, 0x65, 0x06,
	0x62, 0x24, 0x07, 0x62, 0x65, 0x07, 0x62, 0x24, 0x08, 0x62, 0x65, 0x08,
	0x62, 0x24, 0x09, 0x62, 0x65, 0x09, 0x62, 0x24, 0x0a, 0x62, 0x65, 0x0a,
	0x62, 0x24, 0x0b, 0x62, 0x65, 0x0b, 0x62, 0x24, 0x0c, 0x62, 0x65, 0x0c,
	0x62, 0x24, 0x0d, 0x62, 0x65, 0x0d, 0x62, 0x24, 0x0e, 0x62, 0x65, 0x0e,
	0x62, 0x24, 0x0f, 0x62, 0x65, 0x0f, 0x62, 0x24, 0x10, 0x62, 0x65, 0x10,
	0x62, 0x24, 0x11, 0x62, 0x65, 0x11, 0x62, 0x24, 0x12, 0x62, 0x65, 0x12,
	0x62, 0x24, 0x13, 0x62, 0x65, 0x13, 0x62, 0x24, 0x14, 0x62, 0x65, 0x14,
	0x62, 0x24, 0x15, 0x62, 0x65, 0x15, 0x62, 0x24, 0x16, 0x62, 0x65, 0x16,
	0x62, 0x24, 0x17, 0x62, 0x65, 0x17, 0x62, 0x24, 0x18, 0x62, 0x65, 0x18,
	0x62, 0x24, 0x19, 0x62, 0x65, 0x19, 0x62, 0x24, 0x1a, 0x62, 0x65, 0x1a,
	0x62, 0x24, 0x1b, 0x62, 0x65, 0x1b, 0x62, 0x24, 0x1c, 0x62, 0x65, 0x1c,
	0x62, 0x24, 0x1d, 0x62, 0x65, 0x1d, 0x62, 0x24, 0x1e, 0x62, 0x65, 0x1e,
	0x62, 0x24, 0x1f, 0x62, 0x65, 0x1f, 0x62, 0x24, 0x20, 0x62, 0x65, 0x20,
	0x62, 0x24, 0x21, 0x62, 0x65, 0x21, 0x62, 0x24, 0x22, 0x62, 0x65, 0x22,
	0x62, 0x24, 0x23, 0x62, 0x65, 0x23, 0x62, 0x24, 0x24, 0x62, 0x65, 0x24,
	0x62, 0x24, 0x25, 0x62, 0x65, 0x25, 0x62, 0x24, 0x26, 0x62, 0x65, 0x26,
	0x62, 0x24, 0x27, 0x62, 0x65, 0x27, 0x62, 0x24, 0x28, 0x62, 0x65, 0x28,
	0x62, 0x24, 0x29, 0x62, 0x65, 0x29, 0x62, 0x24, 0x2a, 0x62, 0x65, 0x2a,
	0x62, 0x24, 0x2b, 0x62, 0x65, 0x2b, 0x62, 0x24, 0x2c, 0x62, 0x65, 0x2c,
	0x62, 0x24, 0x2d, 0x62, 0x65, 0x2d, 0x62, 0x24, 0x2e, 0x62, 0x65, 0x2e,
	0x62, 0x24, 0x2f, 0x62, 0x65, 0x2f, 0x62, 0x24, 0x30, 0x62, 0x65, 0x30,
	0x62, 0x24, 0x31, 0x62, 0x65, 0x31, 0x62, 0x24, 0x32, 0x62, 0x65, 0x32,
	0x62, 0x24, 0x33, 0x62, 0x65, 0x33, 0x62, 0x24, 0x34, 0x62, 0x65, 0x34,
	0x62, 0x24, 0x35, 0x62, 0x65, 0x35, 0x62, 0x24, 0x36, 0x62, 0x65, 0x36,
	0x62, 0x24, 0x37, 0x62, 0x65, 0x37, 0x62, 0x24, 0x38, 0x62, 0x65, 0x38,
	0x62, 0x24, 0x39, 0x62, 0x65, 0x39, 0x62, 0x24, 0x3a, 0x62, 0x65, 0x3a,
	0x62, 0x24, 0x3b, 0x62, 0x65, 0x3b, 0x62, 0x24, 0x3c, 0x62, 0x65, 0x3c,
	0x62, 0x24, 0x3d, 0x62, 0x65, 0x3d, 0x62, 0x24, 0x3e, 0x62, 0x65, 0x3e,
	0x62, 0x24, 0x3f, 0x62, 0x65, 0x3f, 0x62, 0x24, 0x40, 0x62, 0x65, 0x40,
	0x62, 0x24, 0x41, 0x62, 0x65, 0x41, 0x62, 0x24, 0x42, 0x62, 0x65, 0x42,
	0x62, 0x24, 0x43, 0x62, 0x65, 0x43, 0x62, 0x24, 0x44, 0x62, 0x65, 0x44,
	0x62, 0x24, 0x45, 0x62, 0x65, 0x45, 0x62, 0x24, 0x46, 0x62, 0x65, 0x46,
	0x62, 0x24, 0x47, 0x62, 0x65, 0x47, 0x62, 0x24, 0x48, 0x62, 0x65, 0x48,
	0x62, 0x24, 0x49, 0x62, 0x65, 0x49, 0x62, 0x24, 0x4a, 0x62, 0x65, 0x4a,
	0x62, 0x24, 0x4b, 0x62, 0x65, 0x4b, 0x62, 0x24, 0x4c, 0x62, 0x65, 0x4c,
	0x62, 0x24, 0x4d, 0x62, 0x65, 0x4d, 0x62, 0x24, 0x4e, 0x62, 0x65, 0x4e,
	0x62, 0x24, 0x4f, 0x62, 0x65, 0x4f, 0x62, 0x24, 0x50, 0x62, 0x65, 0x50,
	0x62, 0x24, 0x51, 0x62, 0x65, 0x51, 0x62, 0x24, 0x52, 0x62, 0x65, 0x52,
	0x62, 0x24, 0x53, 0x62, 0x65, 0x53, 0x62, 0x24, 0x54, 0x62, 0x65, 0x54,
	0x62, 0x24, 0x55, 0x62, 0x65, 0x55, 0x62, 0x24, 0x56, 0x62, 0x65, 0x56,
	0x62, 0x24, 0x57, 0x62, 0x65, 0x57, 0x62, 0x24, 0x58, 0x62, 0x65, 0x58,
	0x62, 0x24, 0x59, 0x62, 0x65, 0x59, 0x62, 0x24, 0x5a, 0x62, 0x65, 0x5a,
	0x62, 0x24, 0x5b, 0x62, 0x65, 0x5b, 0x62, 0x24, 0x5c, 0x62, 0x65, 0x5c,
	0x62, 0x24, 0x5d, 0x62, 0x65, 0x5d, 0x62, 0x24, 0x5e, 0x62, 0x65, 0x5e,
	0x62, 0x24, 0x5f, 0x62, 0x65, 0x5f, 0x62, 0x24, 0x60, 0x62, 0x65, 0x60,
	0x62, 0x24, 0x61, 0x62, 0x65, 0x61, 0x62, 0x24, 0x62, 0x62, 0x65, 0x62,
	0x62, 0x24, 0x63, 0x62, 0x65, 0x63, 0x62, 0x24, 0x64, 0x62, 0x65, 0x64,
	0x62, 0x24, 0x65, 0x62, 0x65, 0x65, 0x62, 0x24, 0x66, 0x62, 0x65, 0x66,
	0x62, 0x24, 0x67, 0x62, 0x65, 0x67, 0x62, 0x24, 0x68, 0x62, 0x65, 0x68,
	0x62, 0x24, 0x69, 0x62, 0x65, 0x69, 0x62, 0x24, 0x6a, 0x62, 0x65, 0x6a,
	0x62, 0x24, 0x6b, 0x62, 0x65, 0x6b, 0x62, 0x24, 0x6c, 0x62, 0x65, 0x6c,
	0x62, 0x24, 0x6d, 0x62, 0x65, 0x6d, 0x62, 0x24, 0x6e, 0x62, 0x65, 0x6e,
	0x62, 0x24, 0x6f, 0x62, 0x65, 0x6f, 0x62, 0x24, 0x70, 0x62, 0x65, 0x70,
	0x62, 0x24, 0x71, 0x62, 0x65, 0x71, 0x62, 0x24, 0x72, 0x62, 0x65, 0x72,
	0x62, 0x24, 0x73, 0x62, 0x65, 0x73, 0x62, 0x24, 0x74, 0x62, 0x65, 0x74,
	0x62, 0x24, 0x75, 0x62, 0x65, 0x75, 0x62, 0x24, 0x76, 0x62, 0x65, 0x76,
	0x62, 0x24, 0x77, 0x62, 0x65, 0x77, 0x62, 0x24, 0x78, 0x62, 0x65, 0x78,
	0x62, 0x24, 0x79, 0x62, 0x65, 0x79, 0x62, 0x24, 0x7a, 0x62, 0x65, 0x7a,
	0x62, 0x24, 0x7b, 0x62, 0x65, 0x7b, 0x62, 0x24, 0x7c, 0x62, 0x65, 0x7c,
	0x62, 0x24, 0x7d, 0x62, 0x65, 0x7d, 0x62, 0x24, 0x7e, 0x62, 0x65, 0x7e,
	0x62, 0x24, 0x7f, 0x62, 0x65, 0x7f, 0x62, 0x24, 0x80, 0x62, 0x65, 0x80,
	0x62, 0x24, 0x81, 0x62, 0x65, 0x81, 0x62, 0x24, 0x82, 0x62, 0x65, 0x82,
	0x62, 0x24, 0x83, 0x62, 0x65, 0x83, 0x62, 0x24, 0x84, 0x62, 0x65, 0x84,
	0x62, 0x24, 0x85, 0x62, 0x65, 0x85, 0x62, 0x24, 0x86, 0x62, 0x65, 0x86,
	0x62, 0x24, 0x87, 0x62, 0x65, 0x87, 0x62, 0x24, 0x88, 0x62, 0x65, 0x88,
	0x62, 0x24, 0x89, 0x62, 0x65, 0x89, 0x62, 0x24, 0x8a, 0x62, 0x65, 0x8a,
	0x62, 0x24, 0x8b, 0x62, 0x65, 0x8b, 0x62, 0x24, 0x8c, 0x62, 0x65, 0x8c,
	0x62, 0x24, 0x8d, 0x62, 0x65, 0x8d, 0x62, 0x24, 0x8e, 0x62, 0x65, 0x8e,
	0x62, 0x24, 0x8f, 0x62, 0x65, 0x8f, 0x62, 0x24, 0x90, 0x62, 0x65, 0x90,
	0x62, 0x24, 0x91, 0x62, 0x65, 0x91, 0x62, 0x24, 0x92, 0x62, 0x65, 0x92,
	0x62, 0x24, 0x93, 0x62, 0x65, 0x93, 0x62, 0x24, 0x94, 0x62, 0x65, 0x94,
	0x62, 0x24, 0x95, 0x62, 0x65, 0x95, 0x62, 0x24, 0x96, 0x62, 0x65, 0x96,
	0x62, 0x24, 0x97, 0x62, 0x65, 0x97, 0x62, 0x24, 0x98, 0x62, 0x65, 0x98,
	0x62, 0x24, 0x99, 0x62, 0x65, 0x99, 0x62, 0x24, 0x9a, 0x62, 0x65, 0x9a,
	0x62, 0x24, 0x9b, 0x62, 0x65, 0x9b, 0x62, 0x24, 0x9c, 0x62, 0x65, 0x9c,
	0x62, 0x24, 0x9d, 0x62, 0x65, 0x9d, 0x62, 0x24, 0x9e, 0x62, 0x65, 0x9e,
	0x62, 0x24, 0x9f, 0x62, 0x65, 0x9f, 0x62, 0x24, 0xa0, 0x62, 0x65, 0xa0,
	0x62, 0x24, 0xa1, 0x62, 0x65, 0xa1, 0x62, 0x24, 0xa2, 0x62, 0x65, 0xa2,
	0x62, 0x24, 0xa3, 0x62, 0x65, 0xa3, 0x62, 0x24, 0xa4, 0x62, 0x65, 0xa4,
	0x62, 0x24, 0xa5, 0x62, 0x65, 0xa5, 0x62, 0x24, 0xa6, 0x62, 0x65, 0xa6,
	0x62, 0x24, 0xa7, 0x62, 0x65, 0xa7, 0x62, 0x24, 0xa8, 0x62, 0x65, 0xa8,
	0x62, 0x24, 0xa9, 0x62, 0x65, 0xa9, 0x62, 0x24, 0xaa, 0x62, 0x65, 0xaa,
	0x62, 0x24, 0xab, 0x62, 0x65, 0xab, 0x62, 0x24, 0xac, 0x62, 0x65, 0xac,
	0x62, 0x24, 0xad, 0x62, 0x65, 0xad, 0x62, 0x24, 0xae, 0x62, 0x65, 0xae,
	0x62, 0x24, 0xaf, 0x62, 0x65, 0xaf, 0x62, 0x24, 0xb0, 0x62, 0x65, 0xb0,
	0x62, 0x24, 0xb1, 0x62, 0x65, 0xb1, 0x62, 0x24, 0xb2, 0x62, 0x65, 0xb2,
	0x62, 0x24, 0xb3, 0x62, 0x65, 0xb3, 0x62, 0x24, 0xb4, 0x62, 0x65, 0xb4,
	0x62, 0x24, 0xb5, 0x62, 0x65, 0xb5, 0x62, 0x24, 0xb6, 0x62, 0x65, 0xb6,
	0x62, 0x24, 0xb7, 0x62, 0x65, 0xb7, 0x62, 0x24, 0xb8, 0x62, 0x65, 0xb8,
	0x62, 0x24, 0xb9, 0x62, 0x65, 0xb9, 0x62, 0x24, 0xba, 0x62, 0x65, 0xba,
	0x62, 0x24, 0xbb, 0x62, 0x65, 0xbb, 0x62, 0x24, 0xbc, 0x62, 0x65, 0xbc,
	0x62, 0x24, 0xbd, 0x62, 0x65, 0xbd, 0x62, 0x24, 0xbe, 0x62, 0x65, 0xbe,
	0x62, 0x24, 0xbf, 0x62, 0x65, 0xbf, 0x62, 0x24, 0xc0, 0x62, 0x65, 0xc0,
	0x62, 0x24, 0xc1, 0x62, 0x65, 0xc1, 0x62, 0x24, 0xc2, 0x62, 0x65, 0xc2,
	0x62, 0x24, 0xc3, 0x62, 0x65, 0xc3, 0x62, 0x24, 0xc4, 0x62, 0x65, 0xc4,
	0x62, 0x24, 0xc5, 0x62, 0x65, 0xc5, 0x62, 0x24, 0xc6, 0x62, 0x65, 0xc6,
	0x62, 0x24, 0xc7, 0x62, 0x65, 0xc7, 0x62, 0x24, 0xc8, 0x62, 0x65, 0xc8,
	0x62, 0x24, 0xc9, 0x62, 0x65, 0xc9, 0x62, 0x24, 0xca, 0x62, 0x65, 0xca,
	0x62, 0x24, 0xcb, 0x62, 0x65, 0xcb, 0x62, 0x24, 0xcc, 0x62, 0x65, 0xcc,
	0x62, 0x24, 0xcd, 0x62, 0x65, 0xcd, 0x62, 0x24, 0xce, 0x62, 0x65, 0xce,
	0x62, 0x24, 0xcf, 0x62, 0x65, 0xcf, 0x62, 0x24, 0xd0, 0x62, 0x65, 0xd0,
	0x62, 0x24, 0xd1, 0x62, 0x65, 0xd1, 0x62, 0x24, 0xd2, 0x62, 0x65, 0xd2,
	0x62, 0x24, 0xd3, 0x62, 0x65, 0xd3, 0x62, 0x24, 0xd4, 0x62, 0x65, 0xd4,
	0x62, 0x24, 0xd5, 0x62, 0x65, 0xd5, 0x62, 0x24, 0xd6, 0x62, 0x65, 0xd6,
	0x62, 0x24, 0xd7, 0x62, 0x65, 0xd7, 0x62, 0x24, 0xd8, 0x62, 0x65, 0xd8,
	0x62, 0x24, 0xd9, 0x62, 0x65, 0xd9, 0x62, 0x24, 0xda, 0x62, 0x65, 0xda,
	0x62, 0x24, 0xdb, 0x62, 0x65, 0xdb, 0x62, 0x24, 0xdc, 0x62, 0x65, 0xdc,
	0x62, 0x24, 0xdd, 0x62, 0x65, 0xdd, 0x62, 0x24, 0xde, 0x62, 0x65, 0xde,
	0x62, 0x24, 0xdf, 0x62, 0x65, 0xdf, 0x62, 0x24, 0xe0, 0x62, 0x65, 0xe0,
	0x62, 0x24, 0xe1, 0x62, 0x65, 0xe1, 0x62, 0x24, 0xe2, 0x62, 0x65, 0xe2,
	0x62, 0x24, 0xe3, 0x62, 0x65, 0xe3, 0x62, 0x24, 0xe4, 0x62, 0x65, 0xe4,
	0x62, 0x24, 0xe5, 0x62, 0x65, 0xe5, 0x62, 0x24, 0xe6, 0x62, 0x65, 0xe6,
	0x62, 0x24, 0xe7, 0x62, 0x65, 0xe7, 0x62, 0x24, 0xe8, 0x62, 0x65, 0xe8,
	0x62, 0x24, 0xe9, 0x62, 0x65, 0xe9, 0x62, 0x24, 0xea, 0x62, 0x65, 0xea,
	0x62, 0x24, 0xeb, 0x62, 0x65, 0xeb, 0x62, 0x24, 0xec, 0x62, 0x65, 0xec,
	0x62, 0x24, 0xed, 0x62, 0x65, 0xed, 0x62, 0x24, 0xee, 0x62, 0x65, 0xee,
	0x62, 0x24, 0xef, 0x62, 0x65, 0xef, 0x62, 0x24, 0xf0, 0x62, 0x65, 0xf0,
	0x62, 0x24, 0xf1, 0x62, 0x65, 0xf1, 0x62, 0x24, 0xf2, 0x62, 0x65, 0xf2,
	0x62, 0x24, 0xf3, 0x62, 0x65, 0xf3, 0x62, 0x24, 0xf4, 0x62, 0x65, 0xf4,
	0x62, 0x24, 0xf5, 0x62, 0x65, 0xf5, 0x62, 0x24, 0xf6, 0x62, 0x65, 0xf6,
	0x62, 0x24, 0xf7, 0x62, 0x65, 0xf7, 0x62, 0x24, 0xf8, 0x62, 0x65, 0xf8,
	0x62, 0x24, 0xf9, 0x62, 0x65, 0xf9, 0x62, 0x24, 0xfa, 0x62, 0x65, 0xfa,
	0x62, 0x24, 0xfb, 0x62, 0x65, 0xfb, 0x62, 0x24, 0xfc, 0x62, 0x65, 0xfc,
	0x62, 0x24, 0xfd, 0x62, 0x65, 0xfd, 0x62, 0x24, 0xfe, 0x62, 0x65, 0xfe,
	0x62, 0x24, 0xff, 0x62, 0x65, 0xff, 0x42, 0xd4, 0x04, 0x52, 0xd5, 0x04,
	0x62, 0x24, 0x00, 0x62, 0x65, 0x00, 0x62, 0x24, 0x01, 0x62, 0x65, 0x01,
	0x62, 0x24, 0x02, 0x62, 0x65, 0x02, 0x62, 0x24, 0x03, 0x62, 0x65, 0x03,
	0x72, 0x25, 0x03, 0x79, 0x62, 0xc0, 0x20, 0x00, 0x32, 0xa0, 0x99, 0x39,
	0x72, 0xc0, 0x20, 0x00, 0x06, 0xff, 0xff, 0x00,
};

/*
 * Base-ISA DESCRIPTOR-l32i-BASE copy disambiguation probe
 * (VP6_LOAD_ELF_BOOTSTRAP_DIAG5, Phase S), assembled from
 * the internal VP6 diag5 asm (build-diag5.sh, open
 * xtensa-elf-gcc 13.3.0). Phase-R DIAG3/DIAG4 proved every isolated copy sub-op lands
 * (staging read; narrow+wide copy->DMEM/IRAM; addmi walking-base; 160-word run; the
 * full >256-word+addmi->IRAM production shape) yet the production unrolled bootstrap
 * still faults. The ONE remaining untested delta: DIAG3/4 loaded the src/dst copy
 * bases via `l32r` LITERALS; the production vp6_gen_bootstrap loads them via
 * `l32i a5,a2,off` FROM A LIVE DESCRIPTOR held in a2. This probe isolates that:
 *   T1 (the key test): `l32r a2,lit_desc`; `l32i a5,a2,0` (== the driver-patched
 *     dynamic scratch IOVA at desc[0]) then STORE a marker through that l32i-loaded
 *     base -> w2. The progress markers w0/w1 store through an l32r-LITERAL base (a11=
 *     lit_scratch, the DIAG3/4-proven path). w0/w1 land + w2 does not => the
 *     l32i-descriptor-loaded-base store is the fault (DECODE finding; fix = materialize
 *     bases via l32r literals patched at load, drop the l32i-from-descriptor idiom).
 *   T2 (if T1 lands): the exact production copy shape over the FULL 5-segment table
 *     with bases via `l32i` from the live descriptor -- interleaving three pointer
 *     regions (descriptor 0x7dc1 re-read at each segment boundary while a2 stays live,
 *     staging 0x7dc2 src, on-core 0x7ff4/0x7ff3e dst). seg0 is >256 words firing one
 *     addmi walk (feasibility.md 3.1's depth-4 M4U read queue). A T2 M4U TRANSLATION
 *     fault = a DECODE/addressing bug (fixable); a WEDGE / mtk-wdt reset with no
 *     translation fault = the 3.1 read-queue overflow (HARDWARE, terminal on the
 *     single-rank i500). Each segment reads back its last written word -> w4/w6/w8/w9/
 *     w10; w11=0x99 done. VP6_IOC_READ_DIAG2 reads all words back. The driver builds
 *     the 11-word descriptor at VP6_MVA_BOOT_DESC (desc[0]=dynamic scratch IOVA, the
 *     rest fixed on-core/staging MVAs) and seeds VP6_MVA_L1_STAGE[0..1023]=marker.
 *     Payload word 0 (lit_scratch) PATCHED at load. Word map: mt8183-vp6-status.md
 *     Phase S. Mutually exclusive with BOOTSTRAP / DIAG..DIAG4 / ALLOW_PARTIAL.
 */
#define VP6_DIAG5_ENTRY_OFF	0x00000010u
static const u8 vp6_bootstrap_diag5_bin[3840] = {
	0x40, 0x7c, 0x0a, 0x5c, 0x00, 0x00, 0xc1, 0x7d, 0xd0, 0x6e, 0x1a, 0x0d,
	0x00, 0x00, 0x00, 0x00, 0xb1, 0xfc, 0xff, 0x21, 0xfc, 0xff, 0x31, 0xfc,
	0xff, 0x5c, 0xa8, 0x89, 0x0b, 0xc0, 0x20, 0x00, 0x82, 0xa0, 0xa1, 0x89,
	0x1b, 0xc0, 0x20, 0x00, 0x52, 0x22, 0x00, 0x39, 0x25, 0xc0, 0x20, 0x00,
	0x82, 0xa0, 0xb1, 0x89, 0x3b, 0xc0, 0x20, 0x00, 0x52, 0x22, 0x01, 0x42,
	0x22, 0x02, 0x62, 0x24, 0x00, 0x62, 0x65, 0x00, 0x62, 0x24, 0x01, 0x62,
	0x65, 0x01, 0x62, 0x24, 0x02, 0x62, 0x65, 0x02, 0x62, 0x24, 0x03, 0x62,
	0x65, 0x03, 0x62, 0x24, 0x04, 0x62, 0x65, 0x04, 0x62, 0x24, 0x05, 0x62,
	0x65, 0x05, 0x62, 0x24, 0x06, 0x62, 0x65, 0x06, 0x62, 0x24, 0x07, 0x62,
	0x65, 0x07, 0x62, 0x24, 0x08, 0x62, 0x65, 0x08, 0x62, 0x24, 0x09, 0x62,
	0x65, 0x09, 0x62, 0x24, 0x0a, 0x62, 0x65, 0x0a, 0x62, 0x24, 0x0b, 0x62,
	0x65, 0x0b, 0x62, 0x24, 0x0c, 0x62, 0x65, 0x0c, 0x62, 0x24, 0x0d, 0x62,
	0x65, 0x0d, 0x62, 0x24, 0x0e, 0x62, 0x65, 0x0e, 0x62, 0x24, 0x0f, 0x62,
	0x65, 0x0f, 0x62, 0x24, 0x10, 0x62, 0x65, 0x10, 0x62, 0x24, 0x11, 0x62,
	0x65, 0x11, 0x62, 0x24, 0x12, 0x62, 0x65, 0x12, 0x62, 0x24, 0x13, 0x62,
	0x65, 0x13, 0x62, 0x24, 0x14, 0x62, 0x65, 0x14, 0x62, 0x24, 0x15, 0x62,
	0x65, 0x15, 0x62, 0x24, 0x16, 0x62, 0x65, 0x16, 0x62, 0x24, 0x17, 0x62,
	0x65, 0x17, 0x62, 0x24, 0x18, 0x62, 0x65, 0x18, 0x62, 0x24, 0x19, 0x62,
	0x65, 0x19, 0x62, 0x24, 0x1a, 0x62, 0x65, 0x1a, 0x62, 0x24, 0x1b, 0x62,
	0x65, 0x1b, 0x62, 0x24, 0x1c, 0x62, 0x65, 0x1c, 0x62, 0x24, 0x1d, 0x62,
	0x65, 0x1d, 0x62, 0x24, 0x1e, 0x62, 0x65, 0x1e, 0x62, 0x24, 0x1f, 0x62,
	0x65, 0x1f, 0x62, 0x24, 0x20, 0x62, 0x65, 0x20, 0x62, 0x24, 0x21, 0x62,
	0x65, 0x21, 0x62, 0x24, 0x22, 0x62, 0x65, 0x22, 0x62, 0x24, 0x23, 0x62,
	0x65, 0x23, 0x62, 0x24, 0x24, 0x62, 0x65, 0x24, 0x62, 0x24, 0x25, 0x62,
	0x65, 0x25, 0x62, 0x24, 0x26, 0x62, 0x65, 0x26, 0x62, 0x24, 0x27, 0x62,
	0x65, 0x27, 0x62, 0x24, 0x28, 0x62, 0x65, 0x28, 0x62, 0x24, 0x29, 0x62,
	0x65, 0x29, 0x62, 0x24, 0x2a, 0x62, 0x65, 0x2a, 0x62, 0x24, 0x2b, 0x62,
	0x65, 0x2b, 0x62, 0x24, 0x2c, 0x62, 0x65, 0x2c, 0x62, 0x24, 0x2d, 0x62,
	0x65, 0x2d, 0x62, 0x24, 0x2e, 0x62, 0x65, 0x2e, 0x62, 0x24, 0x2f, 0x62,
	0x65, 0x2f, 0x62, 0x24, 0x30, 0x62, 0x65, 0x30, 0x62, 0x24, 0x31, 0x62,
	0x65, 0x31, 0x62, 0x24, 0x32, 0x62, 0x65, 0x32, 0x62, 0x24, 0x33, 0x62,
	0x65, 0x33, 0x62, 0x24, 0x34, 0x62, 0x65, 0x34, 0x62, 0x24, 0x35, 0x62,
	0x65, 0x35, 0x62, 0x24, 0x36, 0x62, 0x65, 0x36, 0x62, 0x24, 0x37, 0x62,
	0x65, 0x37, 0x62, 0x24, 0x38, 0x62, 0x65, 0x38, 0x62, 0x24, 0x39, 0x62,
	0x65, 0x39, 0x62, 0x24, 0x3a, 0x62, 0x65, 0x3a, 0x62, 0x24, 0x3b, 0x62,
	0x65, 0x3b, 0x62, 0x24, 0x3c, 0x62, 0x65, 0x3c, 0x62, 0x24, 0x3d, 0x62,
	0x65, 0x3d, 0x62, 0x24, 0x3e, 0x62, 0x65, 0x3e, 0x62, 0x24, 0x3f, 0x62,
	0x65, 0x3f, 0x62, 0x24, 0x40, 0x62, 0x65, 0x40, 0x62, 0x24, 0x41, 0x62,
	0x65, 0x41, 0x62, 0x24, 0x42, 0x62, 0x65, 0x42, 0x62, 0x24, 0x43, 0x62,
	0x65, 0x43, 0x62, 0x24, 0x44, 0x62, 0x65, 0x44, 0x62, 0x24, 0x45, 0x62,
	0x65, 0x45, 0x62, 0x24, 0x46, 0x62, 0x65, 0x46, 0x62, 0x24, 0x47, 0x62,
	0x65, 0x47, 0x62, 0x24, 0x48, 0x62, 0x65, 0x48, 0x62, 0x24, 0x49, 0x62,
	0x65, 0x49, 0x62, 0x24, 0x4a, 0x62, 0x65, 0x4a, 0x62, 0x24, 0x4b, 0x62,
	0x65, 0x4b, 0x62, 0x24, 0x4c, 0x62, 0x65, 0x4c, 0x62, 0x24, 0x4d, 0x62,
	0x65, 0x4d, 0x62, 0x24, 0x4e, 0x62, 0x65, 0x4e, 0x62, 0x24, 0x4f, 0x62,
	0x65, 0x4f, 0x62, 0x24, 0x50, 0x62, 0x65, 0x50, 0x62, 0x24, 0x51, 0x62,
	0x65, 0x51, 0x62, 0x24, 0x52, 0x62, 0x65, 0x52, 0x62, 0x24, 0x53, 0x62,
	0x65, 0x53, 0x62, 0x24, 0x54, 0x62, 0x65, 0x54, 0x62, 0x24, 0x55, 0x62,
	0x65, 0x55, 0x62, 0x24, 0x56, 0x62, 0x65, 0x56, 0x62, 0x24, 0x57, 0x62,
	0x65, 0x57, 0x62, 0x24, 0x58, 0x62, 0x65, 0x58, 0x62, 0x24, 0x59, 0x62,
	0x65, 0x59, 0x62, 0x24, 0x5a, 0x62, 0x65, 0x5a, 0x62, 0x24, 0x5b, 0x62,
	0x65, 0x5b, 0x62, 0x24, 0x5c, 0x62, 0x65, 0x5c, 0x62, 0x24, 0x5d, 0x62,
	0x65, 0x5d, 0x62, 0x24, 0x5e, 0x62, 0x65, 0x5e, 0x62, 0x24, 0x5f, 0x62,
	0x65, 0x5f, 0x62, 0x24, 0x60, 0x62, 0x65, 0x60, 0x62, 0x24, 0x61, 0x62,
	0x65, 0x61, 0x62, 0x24, 0x62, 0x62, 0x65, 0x62, 0x62, 0x24, 0x63, 0x62,
	0x65, 0x63, 0x62, 0x24, 0x64, 0x62, 0x65, 0x64, 0x62, 0x24, 0x65, 0x62,
	0x65, 0x65, 0x62, 0x24, 0x66, 0x62, 0x65, 0x66, 0x62, 0x24, 0x67, 0x62,
	0x65, 0x67, 0x62, 0x24, 0x68, 0x62, 0x65, 0x68, 0x62, 0x24, 0x69, 0x62,
	0x65, 0x69, 0x62, 0x24, 0x6a, 0x62, 0x65, 0x6a, 0x62, 0x24, 0x6b, 0x62,
	0x65, 0x6b, 0x62, 0x24, 0x6c, 0x62, 0x65, 0x6c, 0x62, 0x24, 0x6d, 0x62,
	0x65, 0x6d, 0x62, 0x24, 0x6e, 0x62, 0x65, 0x6e, 0x62, 0x24, 0x6f, 0x62,
	0x65, 0x6f, 0x62, 0x24, 0x70, 0x62, 0x65, 0x70, 0x62, 0x24, 0x71, 0x62,
	0x65, 0x71, 0x62, 0x24, 0x72, 0x62, 0x65, 0x72, 0x62, 0x24, 0x73, 0x62,
	0x65, 0x73, 0x62, 0x24, 0x74, 0x62, 0x65, 0x74, 0x62, 0x24, 0x75, 0x62,
	0x65, 0x75, 0x62, 0x24, 0x76, 0x62, 0x65, 0x76, 0x62, 0x24, 0x77, 0x62,
	0x65, 0x77, 0x62, 0x24, 0x78, 0x62, 0x65, 0x78, 0x62, 0x24, 0x79, 0x62,
	0x65, 0x79, 0x62, 0x24, 0x7a, 0x62, 0x65, 0x7a, 0x62, 0x24, 0x7b, 0x62,
	0x65, 0x7b, 0x62, 0x24, 0x7c, 0x62, 0x65, 0x7c, 0x62, 0x24, 0x7d, 0x62,
	0x65, 0x7d, 0x62, 0x24, 0x7e, 0x62, 0x65, 0x7e, 0x62, 0x24, 0x7f, 0x62,
	0x65, 0x7f, 0x62, 0x24, 0x80, 0x62, 0x65, 0x80, 0x62, 0x24, 0x81, 0x62,
	0x65, 0x81, 0x62, 0x24, 0x82, 0x62, 0x65, 0x82, 0x62, 0x24, 0x83, 0x62,
	0x65, 0x83, 0x62, 0x24, 0x84, 0x62, 0x65, 0x84, 0x62, 0x24, 0x85, 0x62,
	0x65, 0x85, 0x62, 0x24, 0x86, 0x62, 0x65, 0x86, 0x62, 0x24, 0x87, 0x62,
	0x65, 0x87, 0x62, 0x24, 0x88, 0x62, 0x65, 0x88, 0x62, 0x24, 0x89, 0x62,
	0x65, 0x89, 0x62, 0x24, 0x8a, 0x62, 0x65, 0x8a, 0x62, 0x24, 0x8b, 0x62,
	0x65, 0x8b, 0x62, 0x24, 0x8c, 0x62, 0x65, 0x8c, 0x62, 0x24, 0x8d, 0x62,
	0x65, 0x8d, 0x62, 0x24, 0x8e, 0x62, 0x65, 0x8e, 0x62, 0x24, 0x8f, 0x62,
	0x65, 0x8f, 0x62, 0x24, 0x90, 0x62, 0x65, 0x90, 0x62, 0x24, 0x91, 0x62,
	0x65, 0x91, 0x62, 0x24, 0x92, 0x62, 0x65, 0x92, 0x62, 0x24, 0x93, 0x62,
	0x65, 0x93, 0x62, 0x24, 0x94, 0x62, 0x65, 0x94, 0x62, 0x24, 0x95, 0x62,
	0x65, 0x95, 0x62, 0x24, 0x96, 0x62, 0x65, 0x96, 0x62, 0x24, 0x97, 0x62,
	0x65, 0x97, 0x62, 0x24, 0x98, 0x62, 0x65, 0x98, 0x62, 0x24, 0x99, 0x62,
	0x65, 0x99, 0x62, 0x24, 0x9a, 0x62, 0x65, 0x9a, 0x62, 0x24, 0x9b, 0x62,
	0x65, 0x9b, 0x62, 0x24, 0x9c, 0x62, 0x65, 0x9c, 0x62, 0x24, 0x9d, 0x62,
	0x65, 0x9d, 0x62, 0x24, 0x9e, 0x62, 0x65, 0x9e, 0x62, 0x24, 0x9f, 0x62,
	0x65, 0x9f, 0x62, 0x24, 0xa0, 0x62, 0x65, 0xa0, 0x62, 0x24, 0xa1, 0x62,
	0x65, 0xa1, 0x62, 0x24, 0xa2, 0x62, 0x65, 0xa2, 0x62, 0x24, 0xa3, 0x62,
	0x65, 0xa3, 0x62, 0x24, 0xa4, 0x62, 0x65, 0xa4, 0x62, 0x24, 0xa5, 0x62,
	0x65, 0xa5, 0x62, 0x24, 0xa6, 0x62, 0x65, 0xa6, 0x62, 0x24, 0xa7, 0x62,
	0x65, 0xa7, 0x62, 0x24, 0xa8, 0x62, 0x65, 0xa8, 0x62, 0x24, 0xa9, 0x62,
	0x65, 0xa9, 0x62, 0x24, 0xaa, 0x62, 0x65, 0xaa, 0x62, 0x24, 0xab, 0x62,
	0x65, 0xab, 0x62, 0x24, 0xac, 0x62, 0x65, 0xac, 0x62, 0x24, 0xad, 0x62,
	0x65, 0xad, 0x62, 0x24, 0xae, 0x62, 0x65, 0xae, 0x62, 0x24, 0xaf, 0x62,
	0x65, 0xaf, 0x62, 0x24, 0xb0, 0x62, 0x65, 0xb0, 0x62, 0x24, 0xb1, 0x62,
	0x65, 0xb1, 0x62, 0x24, 0xb2, 0x62, 0x65, 0xb2, 0x62, 0x24, 0xb3, 0x62,
	0x65, 0xb3, 0x62, 0x24, 0xb4, 0x62, 0x65, 0xb4, 0x62, 0x24, 0xb5, 0x62,
	0x65, 0xb5, 0x62, 0x24, 0xb6, 0x62, 0x65, 0xb6, 0x62, 0x24, 0xb7, 0x62,
	0x65, 0xb7, 0x62, 0x24, 0xb8, 0x62, 0x65, 0xb8, 0x62, 0x24, 0xb9, 0x62,
	0x65, 0xb9, 0x62, 0x24, 0xba, 0x62, 0x65, 0xba, 0x62, 0x24, 0xbb, 0x62,
	0x65, 0xbb, 0x62, 0x24, 0xbc, 0x62, 0x65, 0xbc, 0x62, 0x24, 0xbd, 0x62,
	0x65, 0xbd, 0x62, 0x24, 0xbe, 0x62, 0x65, 0xbe, 0x62, 0x24, 0xbf, 0x62,
	0x65, 0xbf, 0x62, 0x24, 0xc0, 0x62, 0x65, 0xc0, 0x62, 0x24, 0xc1, 0x62,
	0x65, 0xc1, 0x62, 0x24, 0xc2, 0x62, 0x65, 0xc2, 0x62, 0x24, 0xc3, 0x62,
	0x65, 0xc3, 0x62, 0x24, 0xc4, 0x62, 0x65, 0xc4, 0x62, 0x24, 0xc5, 0x62,
	0x65, 0xc5, 0x62, 0x24, 0xc6, 0x62, 0x65, 0xc6, 0x62, 0x24, 0xc7, 0x62,
	0x65, 0xc7, 0x62, 0x24, 0xc8, 0x62, 0x65, 0xc8, 0x62, 0x24, 0xc9, 0x62,
	0x65, 0xc9, 0x62, 0x24, 0xca, 0x62, 0x65, 0xca, 0x62, 0x24, 0xcb, 0x62,
	0x65, 0xcb, 0x62, 0x24, 0xcc, 0x62, 0x65, 0xcc, 0x62, 0x24, 0xcd, 0x62,
	0x65, 0xcd, 0x62, 0x24, 0xce, 0x62, 0x65, 0xce, 0x62, 0x24, 0xcf, 0x62,
	0x65, 0xcf, 0x62, 0x24, 0xd0, 0x62, 0x65, 0xd0, 0x62, 0x24, 0xd1, 0x62,
	0x65, 0xd1, 0x62, 0x24, 0xd2, 0x62, 0x65, 0xd2, 0x62, 0x24, 0xd3, 0x62,
	0x65, 0xd3, 0x62, 0x24, 0xd4, 0x62, 0x65, 0xd4, 0x62, 0x24, 0xd5, 0x62,
	0x65, 0xd5, 0x62, 0x24, 0xd6, 0x62, 0x65, 0xd6, 0x62, 0x24, 0xd7, 0x62,
	0x65, 0xd7, 0x62, 0x24, 0xd8, 0x62, 0x65, 0xd8, 0x62, 0x24, 0xd9, 0x62,
	0x65, 0xd9, 0x62, 0x24, 0xda, 0x62, 0x65, 0xda, 0x62, 0x24, 0xdb, 0x62,
	0x65, 0xdb, 0x62, 0x24, 0xdc, 0x62, 0x65, 0xdc, 0x62, 0x24, 0xdd, 0x62,
	0x65, 0xdd, 0x62, 0x24, 0xde, 0x62, 0x65, 0xde, 0x62, 0x24, 0xdf, 0x62,
	0x65, 0xdf, 0x62, 0x24, 0xe0, 0x62, 0x65, 0xe0, 0x62, 0x24, 0xe1, 0x62,
	0x65, 0xe1, 0x62, 0x24, 0xe2, 0x62, 0x65, 0xe2, 0x62, 0x24, 0xe3, 0x62,
	0x65, 0xe3, 0x62, 0x24, 0xe4, 0x62, 0x65, 0xe4, 0x62, 0x24, 0xe5, 0x62,
	0x65, 0xe5, 0x62, 0x24, 0xe6, 0x62, 0x65, 0xe6, 0x62, 0x24, 0xe7, 0x62,
	0x65, 0xe7, 0x62, 0x24, 0xe8, 0x62, 0x65, 0xe8, 0x62, 0x24, 0xe9, 0x62,
	0x65, 0xe9, 0x62, 0x24, 0xea, 0x62, 0x65, 0xea, 0x62, 0x24, 0xeb, 0x62,
	0x65, 0xeb, 0x62, 0x24, 0xec, 0x62, 0x65, 0xec, 0x62, 0x24, 0xed, 0x62,
	0x65, 0xed, 0x62, 0x24, 0xee, 0x62, 0x65, 0xee, 0x62, 0x24, 0xef, 0x62,
	0x65, 0xef, 0x62, 0x24, 0xf0, 0x62, 0x65, 0xf0, 0x62, 0x24, 0xf1, 0x62,
	0x65, 0xf1, 0x62, 0x24, 0xf2, 0x62, 0x65, 0xf2, 0x62, 0x24, 0xf3, 0x62,
	0x65, 0xf3, 0x62, 0x24, 0xf4, 0x62, 0x65, 0xf4, 0x62, 0x24, 0xf5, 0x62,
	0x65, 0xf5, 0x62, 0x24, 0xf6, 0x62, 0x65, 0xf6, 0x62, 0x24, 0xf7, 0x62,
	0x65, 0xf7, 0x62, 0x24, 0xf8, 0x62, 0x65, 0xf8, 0x62, 0x24, 0xf9, 0x62,
	0x65, 0xf9, 0x62, 0x24, 0xfa, 0x62, 0x65, 0xfa, 0x62, 0x24, 0xfb, 0x62,
	0x65, 0xfb, 0x62, 0x24, 0xfc, 0x62, 0x65, 0xfc, 0x62, 0x24, 0xfd, 0x62,
	0x65, 0xfd, 0x62, 0x24, 0xfe, 0x62, 0x65, 0xfe, 0x62, 0x24, 0xff, 0x62,
	0x65, 0xff, 0x42, 0xd4, 0x04, 0x52, 0xd5, 0x04, 0x62, 0x24, 0x00, 0x62,
	0x65, 0x00, 0x62, 0x24, 0x01, 0x62, 0x65, 0x01, 0x62, 0x24, 0x02, 0x62,
	0x65, 0x02, 0x62, 0x24, 0x03, 0x62, 0x65, 0x03, 0x72, 0x25, 0x03, 0x79,
	0x4b, 0xc0, 0x20, 0x00, 0x82, 0xa0, 0xc1, 0x89, 0x5b, 0xc0, 0x20, 0x00,
	0x52, 0x22, 0x03, 0x42, 0x22, 0x04, 0x62, 0x24, 0x00, 0x62, 0x65, 0x00,
	0x62, 0x24, 0x01, 0x62, 0x65, 0x01, 0x62, 0x24, 0x02, 0x62, 0x65, 0x02,
	0x62, 0x24, 0x03, 0x62, 0x65, 0x03, 0x62, 0x24, 0x04, 0x62, 0x65, 0x04,
	0x62, 0x24, 0x05, 0x62, 0x65, 0x05, 0x62, 0x24, 0x06, 0x62, 0x65, 0x06,
	0x62, 0x24, 0x07, 0x62, 0x65, 0x07, 0x62, 0x24, 0x08, 0x62, 0x65, 0x08,
	0x62, 0x24, 0x09, 0x62, 0x65, 0x09, 0x62, 0x24, 0x0a, 0x62, 0x65, 0x0a,
	0x62, 0x24, 0x0b, 0x62, 0x65, 0x0b, 0x62, 0x24, 0x0c, 0x62, 0x65, 0x0c,
	0x62, 0x24, 0x0d, 0x62, 0x65, 0x0d, 0x62, 0x24, 0x0e, 0x62, 0x65, 0x0e,
	0x62, 0x24, 0x0f, 0x62, 0x65, 0x0f, 0x62, 0x24, 0x10, 0x62, 0x65, 0x10,
	0x62, 0x24, 0x11, 0x62, 0x65, 0x11, 0x62, 0x24, 0x12, 0x62, 0x65, 0x12,
	0x62, 0x24, 0x13, 0x62, 0x65, 0x13, 0x62, 0x24, 0x14, 0x62, 0x65, 0x14,
	0x62, 0x24, 0x15, 0x62, 0x65, 0x15, 0x62, 0x24, 0x16, 0x62, 0x65, 0x16,
	0x62, 0x24, 0x17, 0x62, 0x65, 0x17, 0x62, 0x24, 0x18, 0x62, 0x65, 0x18,
	0x62, 0x24, 0x19, 0x62, 0x65, 0x19, 0x62, 0x24, 0x1a, 0x62, 0x65, 0x1a,
	0x62, 0x24, 0x1b, 0x62, 0x65, 0x1b, 0x62, 0x24, 0x1c, 0x62, 0x65, 0x1c,
	0x62, 0x24, 0x1d, 0x62, 0x65, 0x1d, 0x62, 0x24, 0x1e, 0x62, 0x65, 0x1e,
	0x62, 0x24, 0x1f, 0x62, 0x65, 0x1f, 0x62, 0x24, 0x20, 0x62, 0x65, 0x20,
	0x62, 0x24, 0x21, 0x62, 0x65, 0x21, 0x62, 0x24, 0x22, 0x62, 0x65, 0x22,
	0x62, 0x24, 0x23, 0x62, 0x65, 0x23, 0x62, 0x24, 0x24, 0x62, 0x65, 0x24,
	0x62, 0x24, 0x25, 0x62, 0x65, 0x25, 0x62, 0x24, 0x26, 0x62, 0x65, 0x26,
	0x62, 0x24, 0x27, 0x62, 0x65, 0x27, 0x62, 0x24, 0x28, 0x62, 0x65, 0x28,
	0x62, 0x24, 0x29, 0x62, 0x65, 0x29, 0x62, 0x24, 0x2a, 0x62, 0x65, 0x2a,
	0x62, 0x24, 0x2b, 0x62, 0x65, 0x2b, 0x62, 0x24, 0x2c, 0x62, 0x65, 0x2c,
	0x62, 0x24, 0x2d, 0x62, 0x65, 0x2d, 0x62, 0x24, 0x2e, 0x62, 0x65, 0x2e,
	0x62, 0x24, 0x2f, 0x62, 0x65, 0x2f, 0x62, 0x24, 0x30, 0x62, 0x65, 0x30,
	0x62, 0x24, 0x31, 0x62, 0x65, 0x31, 0x62, 0x24, 0x32, 0x62, 0x65, 0x32,
	0x62, 0x24, 0x33, 0x62, 0x65, 0x33, 0x62, 0x24, 0x34, 0x62, 0x65, 0x34,
	0x62, 0x24, 0x35, 0x62, 0x65, 0x35, 0x62, 0x24, 0x36, 0x62, 0x65, 0x36,
	0x62, 0x24, 0x37, 0x62, 0x65, 0x37, 0x62, 0x24, 0x38, 0x62, 0x65, 0x38,
	0x62, 0x24, 0x39, 0x62, 0x65, 0x39, 0x62, 0x24, 0x3a, 0x62, 0x65, 0x3a,
	0x62, 0x24, 0x3b, 0x62, 0x65, 0x3b, 0x62, 0x24, 0x3c, 0x62, 0x65, 0x3c,
	0x62, 0x24, 0x3d, 0x62, 0x65, 0x3d, 0x62, 0x24, 0x3e, 0x62, 0x65, 0x3e,
	0x62, 0x24, 0x3f, 0x62, 0x65, 0x3f, 0x62, 0x24, 0x40, 0x62, 0x65, 0x40,
	0x62, 0x24, 0x41, 0x62, 0x65, 0x41, 0x62, 0x24, 0x42, 0x62, 0x65, 0x42,
	0x62, 0x24, 0x43, 0x62, 0x65, 0x43, 0x62, 0x24, 0x44, 0x62, 0x65, 0x44,
	0x62, 0x24, 0x45, 0x62, 0x65, 0x45, 0x62, 0x24, 0x46, 0x62, 0x65, 0x46,
	0x62, 0x24, 0x47, 0x62, 0x65, 0x47, 0x62, 0x24, 0x48, 0x62, 0x65, 0x48,
	0x62, 0x24, 0x49, 0x62, 0x65, 0x49, 0x62, 0x24, 0x4a, 0x62, 0x65, 0x4a,
	0x62, 0x24, 0x4b, 0x62, 0x65, 0x4b, 0x62, 0x24, 0x4c, 0x62, 0x65, 0x4c,
	0x62, 0x24, 0x4d, 0x62, 0x65, 0x4d, 0x62, 0x24, 0x4e, 0x62, 0x65, 0x4e,
	0x62, 0x24, 0x4f, 0x62, 0x65, 0x4f, 0x62, 0x24, 0x50, 0x62, 0x65, 0x50,
	0x62, 0x24, 0x51, 0x62, 0x65, 0x51, 0x62, 0x24, 0x52, 0x62, 0x65, 0x52,
	0x62, 0x24, 0x53, 0x62, 0x65, 0x53, 0x62, 0x24, 0x54, 0x62, 0x65, 0x54,
	0x62, 0x24, 0x55, 0x62, 0x65, 0x55, 0x62, 0x24, 0x56, 0x62, 0x65, 0x56,
	0x62, 0x24, 0x57, 0x62, 0x65, 0x57, 0x62, 0x24, 0x58, 0x62, 0x65, 0x58,
	0x62, 0x24, 0x59, 0x62, 0x65, 0x59, 0x62, 0x24, 0x5a, 0x62, 0x65, 0x5a,
	0x62, 0x24, 0x5b, 0x62, 0x65, 0x5b, 0x62, 0x24, 0x5c, 0x62, 0x65, 0x5c,
	0x62, 0x24, 0x5d, 0x62, 0x65, 0x5d, 0x62, 0x24, 0x5e, 0x62, 0x65, 0x5e,
	0x62, 0x24, 0x5f, 0x62, 0x65, 0x5f, 0x62, 0x24, 0x60, 0x62, 0x65, 0x60,
	0x62, 0x24, 0x61, 0x62, 0x65, 0x61, 0x62, 0x24, 0x62, 0x62, 0x65, 0x62,
	0x62, 0x24, 0x63, 0x62, 0x65, 0x63, 0x62, 0x24, 0x64, 0x62, 0x65, 0x64,
	0x62, 0x24, 0x65, 0x62, 0x65, 0x65, 0x62, 0x24, 0x66, 0x62, 0x65, 0x66,
	0x62, 0x24, 0x67, 0x62, 0x65, 0x67, 0x62, 0x24, 0x68, 0x62, 0x65, 0x68,
	0x62, 0x24, 0x69, 0x62, 0x65, 0x69, 0x62, 0x24, 0x6a, 0x62, 0x65, 0x6a,
	0x62, 0x24, 0x6b, 0x62, 0x65, 0x6b, 0x62, 0x24, 0x6c, 0x62, 0x65, 0x6c,
	0x62, 0x24, 0x6d, 0x62, 0x65, 0x6d, 0x62, 0x24, 0x6e, 0x62, 0x65, 0x6e,
	0x62, 0x24, 0x6f, 0x62, 0x65, 0x6f, 0x62, 0x24, 0x70, 0x62, 0x65, 0x70,
	0x62, 0x24, 0x71, 0x62, 0x65, 0x71, 0x62, 0x24, 0x72, 0x62, 0x65, 0x72,
	0x62, 0x24, 0x73, 0x62, 0x65, 0x73, 0x62, 0x24, 0x74, 0x62, 0x65, 0x74,
	0x62, 0x24, 0x75, 0x62, 0x65, 0x75, 0x62, 0x24, 0x76, 0x62, 0x65, 0x76,
	0x62, 0x24, 0x77, 0x62, 0x65, 0x77, 0x62, 0x24, 0x78, 0x62, 0x65, 0x78,
	0x62, 0x24, 0x79, 0x62, 0x65, 0x79, 0x62, 0x24, 0x7a, 0x62, 0x65, 0x7a,
	0x62, 0x24, 0x7b, 0x62, 0x65, 0x7b, 0x62, 0x24, 0x7c, 0x62, 0x65, 0x7c,
	0x62, 0x24, 0x7d, 0x62, 0x65, 0x7d, 0x62, 0x24, 0x7e, 0x62, 0x65, 0x7e,
	0x62, 0x24, 0x7f, 0x62, 0x65, 0x7f, 0x62, 0x24, 0x80, 0x62, 0x65, 0x80,
	0x62, 0x24, 0x81, 0x62, 0x65, 0x81, 0x62, 0x24, 0x82, 0x62, 0x65, 0x82,
	0x62, 0x24, 0x83, 0x62, 0x65, 0x83, 0x62, 0x24, 0x84, 0x62, 0x65, 0x84,
	0x62, 0x24, 0x85, 0x62, 0x65, 0x85, 0x62, 0x24, 0x86, 0x62, 0x65, 0x86,
	0x62, 0x24, 0x87, 0x62, 0x65, 0x87, 0x62, 0x24, 0x88, 0x62, 0x65, 0x88,
	0x62, 0x24, 0x89, 0x62, 0x65, 0x89, 0x62, 0x24, 0x8a, 0x62, 0x65, 0x8a,
	0x62, 0x24, 0x8b, 0x62, 0x65, 0x8b, 0x62, 0x24, 0x8c, 0x62, 0x65, 0x8c,
	0x62, 0x24, 0x8d, 0x62, 0x65, 0x8d, 0x62, 0x24, 0x8e, 0x62, 0x65, 0x8e,
	0x62, 0x24, 0x8f, 0x62, 0x65, 0x8f, 0x62, 0x24, 0x90, 0x62, 0x65, 0x90,
	0x62, 0x24, 0x91, 0x62, 0x65, 0x91, 0x62, 0x24, 0x92, 0x62, 0x65, 0x92,
	0x62, 0x24, 0x93, 0x62, 0x65, 0x93, 0x62, 0x24, 0x94, 0x62, 0x65, 0x94,
	0x62, 0x24, 0x95, 0x62, 0x65, 0x95, 0x62, 0x24, 0x96, 0x62, 0x65, 0x96,
	0x62, 0x24, 0x97, 0x62, 0x65, 0x97, 0x62, 0x24, 0x98, 0x62, 0x65, 0x98,
	0x62, 0x24, 0x99, 0x62, 0x65, 0x99, 0x62, 0x24, 0x9a, 0x62, 0x65, 0x9a,
	0x62, 0x24, 0x9b, 0x62, 0x65, 0x9b, 0x62, 0x24, 0x9c, 0x62, 0x65, 0x9c,
	0x62, 0x24, 0x9d, 0x62, 0x65, 0x9d, 0x62, 0x24, 0x9e, 0x62, 0x65, 0x9e,
	0x62, 0x24, 0x9f, 0x62, 0x65, 0x9f, 0x72, 0x25, 0x9f, 0x79, 0x6b, 0xc0,
	0x20, 0x00, 0x82, 0xa0, 0xd1, 0x89, 0x7b, 0xc0, 0x20, 0x00, 0x52, 0x22,
	0x05, 0x42, 0x22, 0x06, 0x62, 0x24, 0x00, 0x62, 0x65, 0x00, 0x62, 0x24,
	0x01, 0x62, 0x65, 0x01, 0x62, 0x24, 0x02, 0x62, 0x65, 0x02, 0x62, 0x24,
	0x03, 0x62, 0x65, 0x03, 0x62, 0x24, 0x04, 0x62, 0x65, 0x04, 0x62, 0x24,
	0x05, 0x62, 0x65, 0x05, 0x62, 0x24, 0x06, 0x62, 0x65, 0x06, 0x62, 0x24,
	0x07, 0x62, 0x65, 0x07, 0x62, 0x24, 0x08, 0x62, 0x65, 0x08, 0x62, 0x24,
	0x09, 0x62, 0x65, 0x09, 0x62, 0x24, 0x0a, 0x62, 0x65, 0x0a, 0x62, 0x24,
	0x0b, 0x62, 0x65, 0x0b, 0x62, 0x24, 0x0c, 0x62, 0x65, 0x0c, 0x62, 0x24,
	0x0d, 0x62, 0x65, 0x0d, 0x62, 0x24, 0x0e, 0x62, 0x65, 0x0e, 0x62, 0x24,
	0x0f, 0x62, 0x65, 0x0f, 0x62, 0x24, 0x10, 0x62, 0x65, 0x10, 0x62, 0x24,
	0x11, 0x62, 0x65, 0x11, 0x62, 0x24, 0x12, 0x62, 0x65, 0x12, 0x62, 0x24,
	0x13, 0x62, 0x65, 0x13, 0x62, 0x24, 0x14, 0x62, 0x65, 0x14, 0x62, 0x24,
	0x15, 0x62, 0x65, 0x15, 0x62, 0x24, 0x16, 0x62, 0x65, 0x16, 0x62, 0x24,
	0x17, 0x62, 0x65, 0x17, 0x62, 0x24, 0x18, 0x62, 0x65, 0x18, 0x62, 0x24,
	0x19, 0x62, 0x65, 0x19, 0x62, 0x24, 0x1a, 0x62, 0x65, 0x1a, 0x62, 0x24,
	0x1b, 0x62, 0x65, 0x1b, 0x62, 0x24, 0x1c, 0x62, 0x65, 0x1c, 0x62, 0x24,
	0x1d, 0x62, 0x65, 0x1d, 0x62, 0x24, 0x1e, 0x62, 0x65, 0x1e, 0x62, 0x24,
	0x1f, 0x62, 0x65, 0x1f, 0x62, 0x24, 0x20, 0x62, 0x65, 0x20, 0x62, 0x24,
	0x21, 0x62, 0x65, 0x21, 0x62, 0x24, 0x22, 0x62, 0x65, 0x22, 0x62, 0x24,
	0x23, 0x62, 0x65, 0x23, 0x62, 0x24, 0x24, 0x62, 0x65, 0x24, 0x62, 0x24,
	0x25, 0x62, 0x65, 0x25, 0x62, 0x24, 0x26, 0x62, 0x65, 0x26, 0x62, 0x24,
	0x27, 0x62, 0x65, 0x27, 0x62, 0x24, 0x28, 0x62, 0x65, 0x28, 0x62, 0x24,
	0x29, 0x62, 0x65, 0x29, 0x62, 0x24, 0x2a, 0x62, 0x65, 0x2a, 0x62, 0x24,
	0x2b, 0x62, 0x65, 0x2b, 0x62, 0x24, 0x2c, 0x62, 0x65, 0x2c, 0x62, 0x24,
	0x2d, 0x62, 0x65, 0x2d, 0x62, 0x24, 0x2e, 0x62, 0x65, 0x2e, 0x62, 0x24,
	0x2f, 0x62, 0x65, 0x2f, 0x62, 0x24, 0x30, 0x62, 0x65, 0x30, 0x62, 0x24,
	0x31, 0x62, 0x65, 0x31, 0x62, 0x24, 0x32, 0x62, 0x65, 0x32, 0x62, 0x24,
	0x33, 0x62, 0x65, 0x33, 0x62, 0x24, 0x34, 0x62, 0x65, 0x34, 0x62, 0x24,
	0x35, 0x62, 0x65, 0x35, 0x62, 0x24, 0x36, 0x62, 0x65, 0x36, 0x62, 0x24,
	0x37, 0x62, 0x65, 0x37, 0x62, 0x24, 0x38, 0x62, 0x65, 0x38, 0x62, 0x24,
	0x39, 0x62, 0x65, 0x39, 0x62, 0x24, 0x3a, 0x62, 0x65, 0x3a, 0x62, 0x24,
	0x3b, 0x62, 0x65, 0x3b, 0x62, 0x24, 0x3c, 0x62, 0x65, 0x3c, 0x62, 0x24,
	0x3d, 0x62, 0x65, 0x3d, 0x62, 0x24, 0x3e, 0x62, 0x65, 0x3e, 0x62, 0x24,
	0x3f, 0x62, 0x65, 0x3f, 0x72, 0x25, 0x3f, 0x79, 0x8b, 0xc0, 0x20, 0x00,
	0x52, 0x22, 0x07, 0x42, 0x22, 0x08, 0x62, 0x24, 0x00, 0x62, 0x65, 0x00,
	0x62, 0x24, 0x01, 0x62, 0x65, 0x01, 0x62, 0x24, 0x02, 0x62, 0x65, 0x02,
	0x62, 0x24, 0x03, 0x62, 0x65, 0x03, 0x62, 0x24, 0x04, 0x62, 0x65, 0x04,
	0x62, 0x24, 0x05, 0x62, 0x65, 0x05, 0x62, 0x24, 0x06, 0x62, 0x65, 0x06,
	0x62, 0x24, 0x07, 0x62, 0x65, 0x07, 0x62, 0x24, 0x08, 0x62, 0x65, 0x08,
	0x62, 0x24, 0x09, 0x62, 0x65, 0x09, 0x62, 0x24, 0x0a, 0x62, 0x65, 0x0a,
	0x62, 0x24, 0x0b, 0x62, 0x65, 0x0b, 0x62, 0x24, 0x0c, 0x62, 0x65, 0x0c,
	0x62, 0x24, 0x0d, 0x62, 0x65, 0x0d, 0x62, 0x24, 0x0e, 0x62, 0x65, 0x0e,
	0x62, 0x24, 0x0f, 0x62, 0x65, 0x0f, 0x62, 0x24, 0x10, 0x62, 0x65, 0x10,
	0x62, 0x24, 0x11, 0x62, 0x65, 0x11, 0x62, 0x24, 0x12, 0x62, 0x65, 0x12,
	0x62, 0x24, 0x13, 0x62, 0x65, 0x13, 0x62, 0x24, 0x14, 0x62, 0x65, 0x14,
	0x62, 0x24, 0x15, 0x62, 0x65, 0x15, 0x62, 0x24, 0x16, 0x62, 0x65, 0x16,
	0x62, 0x24, 0x17, 0x62, 0x65, 0x17, 0x62, 0x24, 0x18, 0x62, 0x65, 0x18,
	0x62, 0x24, 0x19, 0x62, 0x65, 0x19, 0x62, 0x24, 0x1a, 0x62, 0x65, 0x1a,
	0x62, 0x24, 0x1b, 0x62, 0x65, 0x1b, 0x62, 0x24, 0x1c, 0x62, 0x65, 0x1c,
	0x62, 0x24, 0x1d, 0x62, 0x65, 0x1d, 0x62, 0x24, 0x1e, 0x62, 0x65, 0x1e,
	0x62, 0x24, 0x1f, 0x62, 0x65, 0x1f, 0x62, 0x24, 0x20, 0x62, 0x65, 0x20,
	0x62, 0x24, 0x21, 0x62, 0x65, 0x21, 0x62, 0x24, 0x22, 0x62, 0x65, 0x22,
	0x62, 0x24, 0x23, 0x62, 0x65, 0x23, 0x62, 0x24, 0x24, 0x62, 0x65, 0x24,
	0x62, 0x24, 0x25, 0x62, 0x65, 0x25, 0x62, 0x24, 0x26, 0x62, 0x65, 0x26,
	0x62, 0x24, 0x27, 0x62, 0x65, 0x27, 0x62, 0x24, 0x28, 0x62, 0x65, 0x28,
	0x62, 0x24, 0x29, 0x62, 0x65, 0x29, 0x62, 0x24, 0x2a, 0x62, 0x65, 0x2a,
	0x62, 0x24, 0x2b, 0x62, 0x65, 0x2b, 0x62, 0x24, 0x2c, 0x62, 0x65, 0x2c,
	0x62, 0x24, 0x2d, 0x62, 0x65, 0x2d, 0x62, 0x24, 0x2e, 0x62, 0x65, 0x2e,
	0x62, 0x24, 0x2f, 0x62, 0x65, 0x2f, 0x62, 0x24, 0x30, 0x62, 0x65, 0x30,
	0x62, 0x24, 0x31, 0x62, 0x65, 0x31, 0x62, 0x24, 0x32, 0x62, 0x65, 0x32,
	0x62, 0x24, 0x33, 0x62, 0x65, 0x33, 0x62, 0x24, 0x34, 0x62, 0x65, 0x34,
	0x62, 0x24, 0x35, 0x62, 0x65, 0x35, 0x62, 0x24, 0x36, 0x62, 0x65, 0x36,
	0x62, 0x24, 0x37, 0x62, 0x65, 0x37, 0x62, 0x24, 0x38, 0x62, 0x65, 0x38,
	0x62, 0x24, 0x39, 0x62, 0x65, 0x39, 0x62, 0x24, 0x3a, 0x62, 0x65, 0x3a,
	0x62, 0x24, 0x3b, 0x62, 0x65, 0x3b, 0x62, 0x24, 0x3c, 0x62, 0x65, 0x3c,
	0x62, 0x24, 0x3d, 0x62, 0x65, 0x3d, 0x62, 0x24, 0x3e, 0x62, 0x65, 0x3e,
	0x62, 0x24, 0x3f, 0x62, 0x65, 0x3f, 0x72, 0x25, 0x3f, 0x79, 0x9b, 0xc0,
	0x20, 0x00, 0x52, 0x22, 0x09, 0x42, 0x22, 0x0a, 0x62, 0x24, 0x00, 0x62,
	0x65, 0x00, 0x62, 0x24, 0x01, 0x62, 0x65, 0x01, 0x62, 0x24, 0x02, 0x62,
	0x65, 0x02, 0x62, 0x24, 0x03, 0x62, 0x65, 0x03, 0x62, 0x24, 0x04, 0x62,
	0x65, 0x04, 0x62, 0x24, 0x05, 0x62, 0x65, 0x05, 0x62, 0x24, 0x06, 0x62,
	0x65, 0x06, 0x62, 0x24, 0x07, 0x62, 0x65, 0x07, 0x62, 0x24, 0x08, 0x62,
	0x65, 0x08, 0x62, 0x24, 0x09, 0x62, 0x65, 0x09, 0x62, 0x24, 0x0a, 0x62,
	0x65, 0x0a, 0x62, 0x24, 0x0b, 0x62, 0x65, 0x0b, 0x62, 0x24, 0x0c, 0x62,
	0x65, 0x0c, 0x62, 0x24, 0x0d, 0x62, 0x65, 0x0d, 0x62, 0x24, 0x0e, 0x62,
	0x65, 0x0e, 0x62, 0x24, 0x0f, 0x62, 0x65, 0x0f, 0x62, 0x24, 0x10, 0x62,
	0x65, 0x10, 0x62, 0x24, 0x11, 0x62, 0x65, 0x11, 0x62, 0x24, 0x12, 0x62,
	0x65, 0x12, 0x62, 0x24, 0x13, 0x62, 0x65, 0x13, 0x62, 0x24, 0x14, 0x62,
	0x65, 0x14, 0x62, 0x24, 0x15, 0x62, 0x65, 0x15, 0x62, 0x24, 0x16, 0x62,
	0x65, 0x16, 0x62, 0x24, 0x17, 0x62, 0x65, 0x17, 0x62, 0x24, 0x18, 0x62,
	0x65, 0x18, 0x62, 0x24, 0x19, 0x62, 0x65, 0x19, 0x62, 0x24, 0x1a, 0x62,
	0x65, 0x1a, 0x62, 0x24, 0x1b, 0x62, 0x65, 0x1b, 0x62, 0x24, 0x1c, 0x62,
	0x65, 0x1c, 0x62, 0x24, 0x1d, 0x62, 0x65, 0x1d, 0x62, 0x24, 0x1e, 0x62,
	0x65, 0x1e, 0x62, 0x24, 0x1f, 0x62, 0x65, 0x1f, 0x62, 0x24, 0x20, 0x62,
	0x65, 0x20, 0x62, 0x24, 0x21, 0x62, 0x65, 0x21, 0x62, 0x24, 0x22, 0x62,
	0x65, 0x22, 0x62, 0x24, 0x23, 0x62, 0x65, 0x23, 0x62, 0x24, 0x24, 0x62,
	0x65, 0x24, 0x62, 0x24, 0x25, 0x62, 0x65, 0x25, 0x62, 0x24, 0x26, 0x62,
	0x65, 0x26, 0x62, 0x24, 0x27, 0x62, 0x65, 0x27, 0x62, 0x24, 0x28, 0x62,
	0x65, 0x28, 0x62, 0x24, 0x29, 0x62, 0x65, 0x29, 0x62, 0x24, 0x2a, 0x62,
	0x65, 0x2a, 0x62, 0x24, 0x2b, 0x62, 0x65, 0x2b, 0x62, 0x24, 0x2c, 0x62,
	0x65, 0x2c, 0x62, 0x24, 0x2d, 0x62, 0x65, 0x2d, 0x62, 0x24, 0x2e, 0x62,
	0x65, 0x2e, 0x62, 0x24, 0x2f, 0x62, 0x65, 0x2f, 0x62, 0x24, 0x30, 0x62,
	0x65, 0x30, 0x62, 0x24, 0x31, 0x62, 0x65, 0x31, 0x62, 0x24, 0x32, 0x62,
	0x65, 0x32, 0x62, 0x24, 0x33, 0x62, 0x65, 0x33, 0x62, 0x24, 0x34, 0x62,
	0x65, 0x34, 0x62, 0x24, 0x35, 0x62, 0x65, 0x35, 0x62, 0x24, 0x36, 0x62,
	0x65, 0x36, 0x62, 0x24, 0x37, 0x62, 0x65, 0x37, 0x62, 0x24, 0x38, 0x62,
	0x65, 0x38, 0x62, 0x24, 0x39, 0x62, 0x65, 0x39, 0x62, 0x24, 0x3a, 0x62,
	0x65, 0x3a, 0x62, 0x24, 0x3b, 0x62, 0x65, 0x3b, 0x62, 0x24, 0x3c, 0x62,
	0x65, 0x3c, 0x62, 0x24, 0x3d, 0x62, 0x65, 0x3d, 0x62, 0x24, 0x3e, 0x62,
	0x65, 0x3e, 0x62, 0x24, 0x3f, 0x62, 0x65, 0x3f, 0x72, 0x25, 0x3f, 0x79,
	0xab, 0xc0, 0x20, 0x00, 0x82, 0xa0, 0x99, 0x89, 0xbb, 0xc0, 0x20, 0x00,
	0x06, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

/*
 * Base-ISA jx-HANDOFF isolation probe (VP6_LOAD_ELF_BOOTSTRAP_DIAG6, Phase S 2nd lever),
 * assembled from the internal VP6 diag6 asm (mini-
 * bootstrap) + vp6-diag6-pad.S (landing pad) via build-diag6.sh (open xtensa-elf-gcc
 * 13.3.0). Phase-S DIAG5 proved the full descriptor-l32i multi-region L1 COPY lands
 * clean, yet the production bootstrap (copy + `jx a3` into the blob entry 0x7da00000)
 * still faults `read 0xbdb0cc00` / `write 0xc02` on the SAME kernel. So the residual is
 * the jx handoff into the blob's external reset-vector region, NOT the copy. This probe
 * isolates it: the mini-bootstrap (at VP6_MVA_BOOTSTRAP) does NO copy, writes w0=0x5a,
 * then `memw; isync; jx a3` to a3 = desc[0] = a base-ISA LANDING PAD the driver writes
 * over the blob's reset-vector cluster at VP6_MVA_RESET_VECTOR + pad _start; the pad
 * writes w1=0x66. w0=0x5a + w1=0x66 => the jx-to-external-0x7da handoff WORKS (=> the
 * production fault is the BLOB's OWN code = terminal RV33); w0=0x5a + w1=0 + a dmesg
 * `read 0xbdb0cc00` => the handoff ITSELF faults (a fixable decode/handoff bug). Both
 * payloads' lit_scratch (payload +0) are PATCHED at load with the dynamic scratch IOVA.
 * Mutually exclusive with BOOTSTRAP / DIAG..DIAG5 / ALLOW_PARTIAL.
 */
#define VP6_DIAG6_ENTRY_OFF	0x00000010u
static const u8 vp6_bootstrap_diag6_bin[48] = {
	0x40, 0x7c, 0x0a, 0x5c, 0x00, 0x00, 0xc1, 0x7d, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0xb1, 0xfc, 0xff, 0x21, 0xfc, 0xff, 0x5c, 0xa8,
	0x89, 0x0b, 0xc0, 0x20, 0x00, 0x38, 0x02, 0xc0, 0x20, 0x00, 0x00, 0x20,
	0x00, 0xa0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
#define VP6_DIAG6_PAD_ENTRY_OFF	0x00000010u
static const u8 vp6_diag6_pad_bin[32] = {
	0x40, 0x7c, 0x0a, 0x5c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0xb1, 0xfc, 0xff, 0x82, 0xa0, 0x66, 0x89, 0x1b,
	0xc0, 0x20, 0x00, 0x06, 0xff, 0xff, 0x00, 0x00,
};



/*
 * Generic register-based mailbox marshaling (this driver's ABI v1 convention;
 * the runtime firmware honors it). The mt8183-lineage VP6 mailbox is the
 * xtensa_info register file, not a DRAM region:
 *   INFO00 = msg.id, INFO01 = msg.len, INFO02..INFO13 = msg.data[0..47].
 * INFO16..31 stay free for firmware device-state/debug.
 */
#define VP6_MBOX_ID		0
#define VP6_MBOX_LEN		1
#define VP6_MBOX_DATA0		2
#define VP6_MBOX_DATA_WORDS	(VP6_IPI_DATA_MAX / 4)	/* 12 */

#define VP6_IPI_ACK_TIMEOUT_MS	2000U
#define VP6_RX_RING		16

struct vp6_buf {
	u32 handle;
	size_t size;
	void *va;
	dma_addr_t dma;
};

/* Max base ("always on") clocks across SoCs: mt8183 has 3 (axi/ipu/jtag),
 * mt8365 has 6 (ipu/axi/jtag/ahb/if/edma). */
#define VP6_MAX_CLKS		6

/*
 * Per-SoC match data (of_match .data). The two supported parts differ in the
 * VP6 subsystem plumbing, not in the core cfg-block register model (the
 * 0x19180000 apu_core / ipu_core0 offsets are shared IP):
 *
 *  - mt8183 (i500): a DUAL-core "IPU" fed by the shared ipu_conn fabric. Each
 *    core is one platform device with its own ipu_core syscon (mediatek,ipu-core);
 *    the fabric syscon (mediatek,ipu-conn) + 9 fabric clocks + the GALS/toprgu
 *    resets are shared and required by the vendor RTOS (rproc) START path.
 *  - mt8365 (g350): a SINGLE-core VP6. No ipu_conn fabric -- the core cfg block
 *    (mediatek,apu-core @0x19180000) and its clocks come off the apu syscon
 *    (apu @0x19020000, clk-mt8365-apu). No fabric syscon, no fabric clocks; the
 *    GALS/toprgu reset levers are i500-specific and absent here.
 */
struct vp6_soc_data {
	const char *core_syscon;	/* phandle prop for the core cfg-block syscon */
	bool has_fabric;		/* shared ipu_conn fabric (mt8183 only) */
	const char * const *clk_names;	/* base ("always on") clock-names */
	unsigned int num_clks;
};

struct vp6_core {
	struct device *dev;
	const struct vp6_soc_data *soc;	/* per-SoC match data (of_match .data) */
	struct regmap *cfg;		/* core cfg block (ipu_core0/1 / apu_core syscon) */
	struct regmap *conn;		/* shared ipu_conn fabric syscon (mt8183 only; NULL on mt8365) */
	struct clk_bulk_data clks[VP6_MAX_CLKS];	/* per-core base clocks (always on) */
	int num_clks;
	struct clk_bulk_data fabric_clks[9];	/* shared ipu_conn fabric (mt8183 rproc only) */
	int num_fabric_clks;
	bool fabric_on;			/* fabric bulk enabled (rproc START..HALT) */
	struct reset_control *rst_gals;	/* infracfg_ao IPU_GALS (DSP<->AP GALS bridge) */
	struct reset_control *rst_ipu;	/* toprgu IPU0 subsystem reset */
	int irq;
	bool irq_on;			/* mailbox line unmasked (core live) */
	u32 core_index;
	u32 caps;

	struct miscdevice mdev;
	struct mutex lock;		/* serializes ioctls + guards state */
	bool busy;			/* single owner */
	u32 state;
	char fw_ver[VP6_FW_VER_LEN];

	/* firmware DDR image (flat LOAD_FW path) */
	void *fw_va;
	dma_addr_t fw_dma;
	size_t fw_size;
	size_t fw_alloc;		/* bytes behind fw_va (>= fw_size); the tail past fw_size is zero */
	u32 fw_entry;

	/* fixed-MVA ELF image (LOAD_ELF path): one entry per placed cluster */
	struct vp6_elf_cluster {
		void		*va;
		dma_addr_t	dyn_dma;	/* the coherent alloc's own IOVA */
		unsigned long	iova;		/* the fixed MVA we mapped it at */
		size_t		size;
	} elf_clusters[VP6_ELF_MAX_CLUSTERS];
	unsigned int elf_ncluster;
	bool elf_loaded;

	/* BOOTSTRAP_DIAG probe: dynamic host-readable scratch + a ptr into the
	 * fixed-MVA staging cluster VA, for VP6_IOC_READ_DIAG readback. */
	void		*diag_scratch_va;
	dma_addr_t	diag_scratch_dma;
	size_t		diag_scratch_size;
	void		*diag_staging_va;	/* borrowed elf_cluster VA, not owned */

	/*
	 * virtio-rpmsg shim (VP6_LOAD_ELF_RPROC). Host VAs point into the mapped
	 * carveout clusters (owned by elf_clusters[], freed by vp6_unload_elf);
	 * this driver only writes the VDEV status/vring fields + reads the trace.
	 */
	bool		rproc;			/* an rproc firmware is loaded */
	u8		*rsc_status;		/* VDEV status byte (host-writable) */
	u32		*rsc_gfeatures;		/* VDEV negotiated-features word */
	u32		*rsc_vring_da[VP6_RPROC_MAX_VRINGS];	/* VDEV vring[].da fields */
	u32		rproc_vring_da[VP6_RPROC_MAX_VRINGS];	/* vring carveout das */
	void		*rproc_vring_va[VP6_RPROC_MAX_VRINGS];	/* vring ring memory */
	u32		rproc_vring_len[VP6_RPROC_MAX_VRINGS];
	struct vp6_fw_rsc_vdev_vring *rsc_vring[VP6_RPROC_MAX_VRINGS];	/* VDEV vring[] (num, align) */
	u32		rproc_buf_da;		/* vdev0buffer carveout: rpmsg payload pool */
	void		*rproc_buf_va;
	u32		rproc_buf_len;
	u16		rproc_used_seen;	/* vring0 used idx last reported */
	u16		rproc_rx_num;		/* rx buffers posted on vring0 */
	u16		rproc_rx_used;		/* vring0 used idx consumed by RPMSG_RECV */
	u16		rproc_rx_avail;		/* vring0 avail idx (host-owned) */
	u16		rproc_tx_avail;		/* vring1 avail idx (host-owned) */
	u32		rproc_dfeatures;	/* features the firmware offered */
	void		*trace_va;		/* trace/log ring host VA */
	u32		trace_len;

	/* mailbox */
	spinlock_t rx_lock;		/* guards rx ring */
	struct vp6_ipi_msg rx_ring[VP6_RX_RING];
	unsigned int rx_head, rx_tail;
	wait_queue_head_t rx_wq;

	/* userspace shared buffers */
	struct idr buf_idr;
	struct mutex buf_lock;
};

/*
 * cfg is the ipu_core0/1 syscon regmap (mutex-locked, no fast_io), so all
 * register access -- including the mailbox IRQ path -- runs in process /
 * threaded-IRQ context.
 */
static inline u32 vp6_rd(struct vp6_core *c, u32 off)
{
	u32 v = 0;

	regmap_read(c->cfg, off, &v);
	return v;
}

static inline void vp6_wr(struct vp6_core *c, u32 off, u32 val)
{
	regmap_write(c->cfg, off, val);
}

static inline void vp6_set(struct vp6_core *c, u32 off, u32 mask)
{
	regmap_update_bits(c->cfg, off, mask, mask);
}

static inline void vp6_clr(struct vp6_core *c, u32 off, u32 mask)
{
	regmap_update_bits(c->cfg, off, mask, 0);
}

/* ---- boot / halt / reset ------------------------------------------------- */

/*
 * Release the core from reset to run the loaded image. Faithful to MediaTek's
 * vpu_hw_boot_sequence (mt6771 == MT8183 die, vpu/mt6771/vpu_hw.c:1986-2100):
 * program the reset vector to the M4U IOVA of the image, select the alt reset
 * vector (STATE_VEC_SEL / CTRL bit19 -- "force to boot based on
 * XTENSA_ALTRESETVEC"), pulse ocdhalt then the APU B/D resets, ungate PIF, set
 * the AXI/iDMA user bits AND the "AXI Request via M4U" bits, then let it run.
 *
 * M4U boot model (supersedes the raw-DDR-physaddr model): with the vp6 node's
 * `iommus` phandle the DMA API returns M4U IOVAs, so c->fw_dma is already the
 * IOVA the DSP fetches from -- altresetvec = fw_dma + entry now points into the
 * M4U-mapped image, and VP6_D{0,1}_VIA_M4U route the fetch through the M4U.
 * (mt6771 vpu_hw.c:2016 xtensa_altresetvec = VPU_MVA_RESET_VECTOR; the fw is
 * m4u_alloc_mva'd to that MVA, vpu_hw.c:1450-1454.)
 *
 * [firmware-gated] Without a valid image the register writes still take effect
 * (validatable: readback, reset toggles) but the core has nothing to execute.
 */
static void vp6_hw_boot(struct vp6_core *c)
{
	/*
	 * Shared ipu_conn fabric clocks: enable ONLY for the rproc RTOS, whose
	 * early cfg-block access needs the APB reg-port bridge + DAP_RX + CAB*
	 * crossbar gates the VPU_TOP genpd leaves unreferenced. Gated on c->rproc
	 * so the plain (cam_vpu/GEMM) LOAD_ELF+START path never enables them --
	 * enabling them there is what wedges the fabric -> TOPRGU reset
	 * (6a0fee809f2b). Balanced by vp6_hw_halt's disable. c->rproc is set in
	 * vp6_load_rproc before this call, so all 12 gates are on by START. The
	 * fabric_on guard keeps enable/disable balanced across repeat HALT/RESET.
	 */
	if (c->rproc && !c->fabric_on && c->num_fabric_clks) {
		clk_bulk_prepare_enable(c->num_fabric_clks, c->fabric_clks);
		c->fabric_on = true;
	}

	/*
	 * RPROC diag lever: deassert the DSP<->AP GALS bridge resets BEFORE the
	 * STALL-clear DSP release. The RE (bf784242) found the rproc RTOS's first
	 * tie2apb cfg write (0x1918012c) wedges the ipu_conn fabric -> TOPRGU HW
	 * watchdog (WDT_STATUS=0x80000000); the untested host lever is that the
	 * GALS bridge is held in reset. Deassert both via the reset framework (the
	 * controllers do the correct writes: infracfg_ao IPU_GALS = MTK_RST_SET_CLR
	 * INFRA_RST3(+0x4 CLR) bit26; toprgu IPU0 = key'd WDT_SWSYSRST bit14). Log
	 * the pre/post SW_RST regs (read-only ioremap) so the run shows whether the
	 * bits were actually asserted -- they may be POR-deasserted (no-op). Gated
	 * on c->rproc; the cam_vpu paths are unaffected.
	 */
	if (c->rproc && (c->rst_gals || c->rst_ipu)) {
		void __iomem *infra = ioremap(0x10001000, 0x200);  /* infracfg_ao */
		void __iomem *rgu = ioremap(0x10007000, 0x100);    /* toprgu */
		u32 g0 = 0, g1 = 0, r0 = 0, r1 = 0;

		if (infra)
			g0 = readl_relaxed(infra + 0x150);  /* INFRA_RST3, bit26=IPU_GALS */
		if (rgu)
			r0 = readl_relaxed(rgu + 0x18);     /* WDT_SWSYSRST, bit14=IPU0 */
		reset_control_deassert(c->rst_gals);
		reset_control_deassert(c->rst_ipu);
		if (infra)
			g1 = readl_relaxed(infra + 0x150);
		if (rgu)
			r1 = readl_relaxed(rgu + 0x18);
		dev_info(c->dev,
			 "vp6-%u: GALS deassert: INFRA_RST3@0x10001150 0x%08x->0x%08x [IPU_GALS b26 %u->%u]; WDT_SWSYSRST@0x10007018 0x%08x->0x%08x [IPU0 b14 %u->%u]\n",
			 c->core_index, g0, g1, !!(g0 & BIT(26)), !!(g1 & BIT(26)),
			 r0, r1, !!(r0 & BIT(14)), !!(r1 & BIT(14)));
		if (infra)
			iounmap(infra);
		if (rgu)
			iounmap(rgu);
	}

	vp6_wr(c, VP6_ALTRESETVEC, (u32)(c->fw_dma + c->fw_entry));

	vp6_set(c, VP6_CTRL, VP6_CTRL_P_DEBUG_EN | VP6_CTRL_STATE_VEC_SEL);
	vp6_set(c, VP6_CTRL, VP6_CTRL_PBCLK_EN);
	vp6_clr(c, VP6_CTRL, VP6_CTRL_PRID);
	vp6_set(c, VP6_CTRL, (c->core_index << 1) & VP6_CTRL_PRID);

	vp6_set(c, VP6_SW_RST, VP6_RST_OCDHALT);
	vp6_clr(c, VP6_SW_RST, VP6_RST_OCDHALT);
	vp6_set(c, VP6_SW_RST, VP6_RST_APU_B | VP6_RST_APU_D);
	vp6_clr(c, VP6_SW_RST, VP6_RST_APU_B | VP6_RST_APU_D);

	vp6_clr(c, VP6_CTRL, VP6_CTRL_PIF_GATED);
	vp6_set(c, VP6_DEFAULT0, VP6_D0_AWUSER | VP6_D0_ARUSER | VP6_D0_VIA_M4U);
	vp6_set(c, VP6_DEFAULT1,
		VP6_D1_ARUSER_IDMA | VP6_D1_AWUSER_IDMA | VP6_D1_VIA_M4U);
	vp6_set(c, VP6_DEFAULT0, VP6_D0_QOS_SWAP);
	/* debug module on, so the core's performance counters (VP6_PMU) count */
	vp6_set(c, VP6_DEFAULT2, VP6_D2_DBG_EN);
	vp6_clr(c, VP6_CTRL, VP6_CTRL_STALL);

	/*
	 * Unmask the core->host mailbox IRQ only now that the core is powered,
	 * clocked and out of reset. The IRQ is requested with IRQF_NO_AUTOEN
	 * (probe) so the level-high line -- which reads asserted while the
	 * VPU_TOP block is unpowered/pre-firmware (xtensa_int == 0xffffffff) --
	 * cannot storm the GIC before this point. Clear any stale doorbell so we
	 * start from a deasserted line, then enable. Balanced by vp6_hw_halt's
	 * disable. On MT8183 xtensa_int is NOT write-1-to-clear: it holds what is
	 * written and the line follows it, so writing 1 here (the vendor mt67xx
	 * ack) asserts the line and the ISR's own ack keeps it asserted -- a
	 * ~1.3e5/s storm even under a `j .` image.
	 */
	if (c->irq > 0 && !c->irq_on) {
		vp6_wr(c, VP6_XTENSA_INT, 0);
		enable_irq(c->irq);
		c->irq_on = true;
	}

	/*
	 * RPROC diag: the vendor RTOS warm-resets the SoC during its first
	 * firmware execution just after this STALL release -- too fast for the
	 * mtk-iommu IRQ handler + console to flush. Poll the M4U fault-status
	 * register in a bounded window right after the release so a during-START
	 * bus/permission fault is latched + dev_err'd before any reset (pair this
	 * with the diag DTB's watchdog=disabled, which turns a wdt-reset into an
	 * inspectable hang). Offsets cited to mtk_iommu.c: REG_MMU_FAULT_ST1 0x134,
	 * REG_MMU0_FAULT_VA 0x13c, REG_MMU0_INVLD_PA 0x140, REG_MMU0_INT_ID 0x150,
	 * at the mt8183 M4U base 0x10205000 (iommu@10205000). Plain ioremap:
	 * read-only, the region stays owned by mtk-iommu (no request_mem_region).
	 * No-op unless an rproc fw is loaded (the cam_vpu paths are unaffected).
	 * Also fire for a BOOTSTRAP_DIAG probe (c->diag_scratch_va set at LOAD) so
	 * a DSP store-fault during the probe is latched + dev_err'd -- the
	 * marker-absent case then distinguishes "store faulted" from "not executing".
	 */
	if (c->rproc || c->diag_scratch_va) {
		void __iomem *m4u = ioremap(0x10205000, 0x200);

		if (m4u) {
			u32 st = 0;
			int i;

			for (i = 0; i < VP6_M4U_POLL_ITERS; i++) {
				st = readl_relaxed(m4u + 0x134);
				if (st)
					break;
				cpu_relax();
			}
			if (st)
				dev_err(c->dev,
					"vp6-%u: M4U FAULT during START: FAULT_ST1=0x%08x FAULT_VA=0x%08x INVLD_PA=0x%08x INT_ID=0x%08x (poll iter %d)\n",
					c->core_index, st,
					readl_relaxed(m4u + 0x13c),
					readl_relaxed(m4u + 0x140),
					readl_relaxed(m4u + 0x150), i);
			else
				dev_info(c->dev,
					 "vp6-%u: no M4U fault in the START poll window (%d iters)\n",
					 c->core_index, VP6_M4U_POLL_ITERS);
			iounmap(m4u);
		}
	}
}

static void vp6_hw_halt(struct vp6_core *c)
{
	if (c->irq > 0 && c->irq_on) {
		disable_irq(c->irq);
		c->irq_on = false;
	}
	vp6_set(c, VP6_CTRL, VP6_CTRL_STALL);
	vp6_set(c, VP6_SW_RST, VP6_RST_APU_B | VP6_RST_APU_D);

	/* Balance the rproc-only fabric-clock enable from vp6_hw_boot. */
	if (c->fabric_on) {
		clk_bulk_disable_unprepare(c->num_fabric_clks, c->fabric_clks);
		c->fabric_on = false;
	}
}

/* ---- mailbox ------------------------------------------------------------- */

static int vp6_ipi_send(struct vp6_core *c, struct vp6_ipi_msg *m, u32 timeout_ms)
{
	unsigned int i;

	if (!(c->caps & VP6_CAP_MAILBOX))
		return -EOPNOTSUPP;
	if (m->id < 0 || m->len > VP6_IPI_DATA_MAX)
		return -EINVAL;
	if (c->state != VP6_STATE_RUNNING)
		return -ENODEV;

	vp6_wr(c, VP6_INFO(VP6_MBOX_ID), (u32)m->id);
	vp6_wr(c, VP6_INFO(VP6_MBOX_LEN), m->len);
	for (i = 0; i < VP6_MBOX_DATA_WORDS; i++) {
		u32 w;

		memcpy(&w, &m->data[i * 4], 4);
		vp6_wr(c, VP6_INFO(VP6_MBOX_DATA0 + i), w);
	}
	/* ring host->core doorbell */
	vp6_set(c, VP6_CTL_XTENSA_INT, 1);

	/*
	 * [firmware-gated] the core drains the doorbell + ACKs via its IRQ only
	 * once firmware runs. With no firmware this cannot complete a round-trip;
	 * the INFO writes + doorbell write themselves are validatable. When
	 * firmware exists, a real ACK path waits (up to timeout_ms) on a
	 * completion signalled from the ISR.
	 */
	(void)timeout_ms;
	return 0;
}

static bool vp6_rx_pop(struct vp6_core *c, struct vp6_ipi_msg *out)
{
	bool got = false;
	unsigned long flags;

	spin_lock_irqsave(&c->rx_lock, flags);
	if (c->rx_head != c->rx_tail) {
		*out = c->rx_ring[c->rx_tail % VP6_RX_RING];
		c->rx_tail++;
		got = true;
	}
	spin_unlock_irqrestore(&c->rx_lock, flags);
	return got;
}

static irqreturn_t vp6_isr(int irq, void *data)
{
	struct vp6_core *c = data;
	struct vp6_ipi_msg m = {};
	unsigned long flags;
	unsigned int i;
	u32 status;

	status = vp6_rd(c, VP6_XTENSA_INT);
	/*
	 * 0 = no doorbell pending; all-ones = the VPU_TOP block is unpowered /
	 * the line is floating (no real IRQ), not a message -- treat as spurious
	 * so a de-powered core can never inject a bogus mailbox entry or spin.
	 */
	if (!status || status == 0xffffffff)
		return IRQ_NONE;

	m.id = (s32)vp6_rd(c, VP6_INFO(VP6_MBOX_ID));
	m.len = vp6_rd(c, VP6_INFO(VP6_MBOX_LEN));
	if (m.len > VP6_IPI_DATA_MAX)
		m.len = VP6_IPI_DATA_MAX;
	for (i = 0; i < VP6_MBOX_DATA_WORDS; i++) {
		u32 w = vp6_rd(c, VP6_INFO(VP6_MBOX_DATA0 + i));

		memcpy(&m.data[i * 4], &w, 4);
	}

	spin_lock_irqsave(&c->rx_lock, flags);
	if (c->rx_head - c->rx_tail < VP6_RX_RING) {
		c->rx_ring[c->rx_head % VP6_RX_RING] = m;
		c->rx_head++;
	}
	spin_unlock_irqrestore(&c->rx_lock, flags);

	/* ack: clear core->host doorbell (re-arm); 0 deasserts, 1 would re-assert */
	vp6_wr(c, VP6_XTENSA_INT, 0);
	wake_up_interruptible(&c->rx_wq);
	return IRQ_HANDLED;
}

/* ---- firmware load ------------------------------------------------------- */

/* Tear down a fixed-MVA ELF image: unmap each cluster from the M4U, free it. */
static void vp6_unload_elf(struct vp6_core *c)
{
	struct iommu_domain *dom = iommu_get_domain_for_dev(c->dev);
	unsigned int i;

	for (i = 0; i < c->elf_ncluster; i++) {
		struct vp6_elf_cluster *cl = &c->elf_clusters[i];

		if (dom && cl->iova)
			iommu_unmap(dom, cl->iova, cl->size);
		if (cl->va)
			dma_free_coherent(c->dev, cl->size, cl->va, cl->dyn_dma);
		memset(cl, 0, sizeof(*cl));
	}
	c->elf_ncluster = 0;
	c->elf_loaded = false;

	if (c->diag_scratch_va) {
		dma_free_coherent(c->dev, c->diag_scratch_size,
				  c->diag_scratch_va, c->diag_scratch_dma);
		c->diag_scratch_va = NULL;
	}
	c->diag_staging_va = NULL;

	/* rproc VAs borrow the (now-freed) carveout clusters -- just forget them */
	c->rproc = false;
	c->rsc_status = NULL;
	c->rsc_gfeatures = NULL;
	c->rsc_vring_da[0] = c->rsc_vring_da[1] = NULL;
	c->rsc_vring[0] = c->rsc_vring[1] = NULL;
	c->rproc_vring_va[0] = c->rproc_vring_va[1] = NULL;
	c->rproc_buf_va = NULL;
	c->rproc_buf_len = 0;
	c->trace_va = NULL;
	c->trace_len = 0;
}

/* Drop whichever firmware image is currently staged (flat or ELF). */
static void vp6_free_fw(struct vp6_core *c)
{
	if (c->fw_va) {
		dma_free_coherent(c->dev, c->fw_alloc, c->fw_va, c->fw_dma);
		c->fw_va = NULL;
		c->fw_alloc = 0;
	}
	if (c->elf_loaded)
		vp6_unload_elf(c);
	c->fw_size = 0;
	c->fw_entry = 0;
	c->fw_dma = 0;
}

static int vp6_load_image(struct vp6_core *c, const void *img, size_t len)
{
	const struct vp6_fw_header *h = img;
	size_t align;

	if (len < sizeof(*h))
		return -EINVAL;
	if (h->magic != VP6_FW_MAGIC || h->abi_version != VP6_FW_ABI_VER)
		return -EINVAL;
	if (h->image_size == 0 || h->image_size > SZ_16M)
		return -EFBIG;
	if (sizeof(*h) + (size_t)h->image_size > len)
		return -EINVAL;
	if (h->entry >= h->image_size)
		return -EINVAL;

	align = h->load_align ? h->load_align : SZ_4K;
	if (align & (align - 1))
		return -EINVAL;

	/*
	 * Keep the previous flat image's buffer when the new image fits and its
	 * IOVA still meets the new alignment: the dma_free_coherent() +
	 * dma_alloc_coherent() pair is an M4U unmap + map on every load, and a
	 * graph run issues one LOAD_FW per kernel. The IOVA is unchanged, so
	 * START's ALTRESETVEC = fw_dma + entry needs no new mapping; the buffer
	 * is coherent, so the memcpy below needs no cache maintenance. Anything
	 * else (first load, a bigger image, an ELF/bootstrap image staged) frees
	 * and allocates as before. The buffer is page-rounded so a run of
	 * slightly growing images does not re-allocate every time.
	 */
	if (!c->fw_va || c->elf_loaded || h->image_size > c->fw_alloc ||
	    (c->fw_dma & (align - 1))) {
		size_t alloc = PAGE_ALIGN(h->image_size);

		/* free any previous image (flat or ELF) */
		vp6_free_fw(c);

		c->fw_va = dma_alloc_coherent(c->dev, alloc, &c->fw_dma, GFP_KERNEL);
		if (!c->fw_va)
			return -ENOMEM;
		c->fw_alloc = alloc;
		if (c->fw_dma & (align - 1)) {
			dev_warn(c->dev, "fw buffer 0x%llx not %zu-aligned\n",
				 (u64)c->fw_dma, align);
		}
	} else if (h->image_size < c->fw_size) {
		/* a fresh buffer is zero-filled: keep that for the tail the previous image wrote */
		memset((u8 *)c->fw_va + h->image_size, 0, c->fw_size - h->image_size);
	}

	memcpy(c->fw_va, (const u8 *)img + sizeof(*h), h->image_size);
	c->fw_size = h->image_size;
	c->fw_entry = h->entry;
	memcpy(c->fw_ver, h->fw_ver, VP6_FW_VER_LEN);
	c->fw_ver[VP6_FW_VER_LEN - 1] = '\0';
	c->state = VP6_STATE_LOADED;

	/* dev_dbg, not dev_info: a graph run loads once per kernel and this line is ~105 B on a 115200 console */
	dev_dbg(c->dev, "vp6-%u: staged fw '%s' %zu B @ dma 0x%llx entry +0x%x\n",
		c->core_index, c->fw_ver, c->fw_size, (u64)c->fw_dma, c->fw_entry);
	return 0;
}

/*
 * Allocate a force-contiguous coherent DRAM cluster and iommu_map it at a fixed
 * IOVA in the VP6 M4U domain, zero-filled, appended to c->elf_clusters (freed by
 * vp6_unload_elf). Returns the kernel VA in *va_out. Used both for the ELF's
 * external segment clusters and for the bootstrap staging/descriptor/code clusters.
 */
static int vp6_elf_map_cluster(struct vp6_core *c, struct iommu_domain *dom,
			       unsigned long iova, size_t size, void **va_out)
{
	struct vp6_elf_cluster *dst = &c->elf_clusters[c->elf_ncluster];
	phys_addr_t phys;
	int ret;

	if (c->elf_ncluster >= VP6_ELF_MAX_CLUSTERS)
		return -E2BIG;

	dst->va = dma_alloc_attrs(c->dev, size, &dst->dyn_dma, GFP_KERNEL,
				  DMA_ATTR_FORCE_CONTIGUOUS);
	if (!dst->va)
		return -ENOMEM;
	dst->size = size;
	memset(dst->va, 0, size);		/* gaps + .bss */

	phys = iommu_iova_to_phys(dom, dst->dyn_dma);
	if (!phys) {
		ret = -ENXIO;
		goto free;
	}
	ret = iommu_map(dom, iova, phys, size, IOMMU_READ | IOMMU_WRITE,
			GFP_KERNEL);
	if (ret)
		goto free;

	dst->iova = iova;
	c->elf_ncluster++;
	if (va_out)
		*va_out = dst->va;
	return 0;

free:
	dma_free_attrs(c->dev, size, dst->va, dst->dyn_dma,
		       DMA_ATTR_FORCE_CONTIGUOUS);
	dst->va = NULL;
	return ret;
}

/* On-core L1 segment record: src == 0 => zero-fill. */
struct vp6_seg { u32 dst, src, len; };

/*
 * Xtensa base-ISA encoders for the fully-unrolled bootstrap (Phase Q). All forms
 * are RRI8 load/store-group (op0 = 2) except the fixed 3-byte l32r/memw/isync/jx
 * literals; encodings are assembler-validated against open xtensa-elf-gcc 13.3.0
 * (internal VP6 toolchain genval byte-diff). *out may be NULL to size.
 */
static void vp6_put3(u8 *out, unsigned int *n, u32 w)
{
	if (out) {
		out[*n]     = w & 0xff;
		out[*n + 1] = (w >> 8) & 0xff;
		out[*n + 2] = (w >> 16) & 0xff;
	}
	*n += 3;
}
static void vp6_emit_l32i(u8 *o, unsigned int *n, unsigned int t, unsigned int s, unsigned int off)
{ vp6_put3(o, n, ((off / 4) << 16) | (0x2u << 12) | (s << 8) | (t << 4) | 0x2u); }
static void vp6_emit_s32i(u8 *o, unsigned int *n, unsigned int t, unsigned int s, unsigned int off)
{ vp6_put3(o, n, ((off / 4) << 16) | (0x6u << 12) | (s << 8) | (t << 4) | 0x2u); }
static void vp6_emit_addmi(u8 *o, unsigned int *n, unsigned int t, unsigned int s, unsigned int add)
{ vp6_put3(o, n, ((add / 256) << 16) | (0xdu << 12) | (s << 8) | (t << 4) | 0x2u); }
static void vp6_emit_movi0(u8 *o, unsigned int *n, unsigned int t)
{ vp6_put3(o, n, (0xau << 12) | (t << 4) | 0x2u); }

/*
 * Emit the FULLY-UNROLLED, STRAIGHT-LINE self-copy bootstrap into *out (NULL to
 * size only); returns the code byte count from _start. No beqz, no loop -- only the
 * proven Phase-J/diag l32i/s32i idiom, plus the single unavoidable final jx handoff.
 * Registers: a2=desc, a3=entry, a4=src, a5=dst, a6=tmp. It reads the descriptor
 * [entry, dst0,src0, dst1,src1, ...] (l32r + l32i, both proven-decode); for each
 * segment it materializes the dst/src bases then unrolls the copy (l32i/s32i pairs)
 * or zero-fill (movi + s32i), walking the base with addmi every 256 words (the
 * l32i/s32i imm8*4 range). Ends memw/isync/jx a3.
 */
static unsigned int vp6_gen_bootstrap(u8 *out, const struct vp6_seg *seg, unsigned int nseg)
{
	unsigned int n = 0, i;

	vp6_put3(out, &n, 0xfffc21u);		/* l32r a2, lit_desc (PC-rel, fixed) */
	vp6_emit_l32i(out, &n, 3, 2, 0);	/* a3 = entry (desc[0]) */
	for (i = 0; i < nseg; i++) {
		unsigned int rem = seg[i].len / 4, off = 4 + 8 * i;
		bool zero = seg[i].src == 0;

		vp6_emit_l32i(out, &n, 5, 2, off);	/* a5 = dst (desc[1+2i]) */
		vp6_emit_l32i(out, &n, 4, 2, off + 4);	/* a4 = src (desc[2+2i]) */
		if (zero)
			vp6_emit_movi0(out, &n, 6);	/* a6 = 0 */
		while (rem) {
			unsigned int blk = min_t(unsigned int, rem, VP6_BOOT_BLK_WORDS), w;

			for (w = 0; w < blk; w++) {
				if (!zero)
					vp6_emit_l32i(out, &n, 6, 4, w * 4);
				vp6_emit_s32i(out, &n, 6, 5, w * 4);
			}
			rem -= blk;
			if (rem) {
				if (!zero)
					vp6_emit_addmi(out, &n, 4, 4, VP6_BOOT_BLK_WORDS * 4);
				vp6_emit_addmi(out, &n, 5, 5, VP6_BOOT_BLK_WORDS * 4);
			}
		}
	}
	vp6_put3(out, &n, 0x0020c0u);		/* memw */
	vp6_put3(out, &n, 0x002000u);		/* isync */
	vp6_put3(out, &n, 0x0003a0u);		/* jx a3 (handoff to the blob entry) */
	return n;
}

/*
 * Bridge the on-core-L1 blob-generation gap (VP6_LOAD_ELF_BOOTSTRAP). The blob's
 * external segments are already placed; here we handle its on-core (>= DSP-local
 * base) PT_LOADs, which the AP cannot write directly on the direct-host VP6.
 *
 * Stage every on-core segment's file bytes in a DRAM cluster (VP6_MVA_L1_STAGE),
 * build a descriptor [entry, dst0,src0, ...] (VP6_MVA_BOOT_DESC) -- one copy record
 * per on-core segment with file bytes (src = its DRAM IOVA), one zero-fill record
 * (src = 0) per on-core .bss gap -- then emit a FULLY-UNROLLED straight-line
 * bootstrap (VP6_MVA_BOOTSTRAP) that self-copies DRAM -> on-core L1 with no
 * control-flow but the final jx (Phase Q; the looped bootstrap FLIX-mis-decoded,
 * Phase O/P). Finally repoint the boot: fw_dma/entry make altresetvec =
 * VP6_MVA_BOOTSTRAP + _start, so the DSP runs the bootstrap and jumps to the blob
 * entry (from the descriptor).
 *
 * Lengths are rounded up to 4 (the bootstrap copies/zeros in words); the staging
 * slots are zero-padded so a rounded copy reads only zeros past filesz.
 */
static int vp6_elf_stage_bootstrap(struct vp6_core *c, const void *img, size_t len,
				   const struct elf32_hdr *eh,
				   const struct elf32_phdr *ph,
				   struct iommu_domain *dom)
{
	struct vp6_seg seg[VP6_BOOT_MAX_SEG];
	unsigned int i, nseg = 0;
	size_t stage_total = 0, stage_off = 0, code_sz, boot_sz;
	void *stage_va = NULL, *desc_va = NULL, *boot_va = NULL;
	u32 *d;
	int ret;

	/* Pass A: enumerate on-core records + total staged (word-rounded) bytes. */
	for (i = 0; i < eh->e_phnum; i++) {
		u32 pa = ph[i].p_paddr;
		size_t fsz, zsz;

		if (ph[i].p_type != PT_LOAD || ph[i].p_memsz == 0 ||
		    pa < VP6_DSP_LOCAL_BASE)
			continue;

		fsz = round_up(ph[i].p_filesz, 4);
		if (fsz) {
			if (nseg == VP6_BOOT_MAX_SEG)
				return -E2BIG;
			seg[nseg].dst = pa;
			seg[nseg].src = VP6_MVA_L1_STAGE + stage_total; /* filled Pass B */
			seg[nseg].len = fsz;
			nseg++;
			stage_total += fsz;
		}
		zsz = round_up(ph[i].p_memsz, 4) - fsz;
		if (zsz) {			/* on-core .bss / zero-fill tail */
			if (nseg == VP6_BOOT_MAX_SEG)
				return -E2BIG;
			seg[nseg].dst = pa + fsz;
			seg[nseg].src = 0;	/* zero-fill marker */
			seg[nseg].len = zsz;
			nseg++;
		}
	}
	if (!nseg)
		return 0;		/* nothing on-core -- normal ELF boot */

	/* Size the generated straight-line bootstrap; must not reach the desc MVA. */
	code_sz = VP6_BOOTSTRAP_ENTRY_OFF + vp6_gen_bootstrap(NULL, seg, nseg);
	boot_sz = PAGE_ALIGN(code_sz);
	if (boot_sz > VP6_MVA_BOOT_DESC - VP6_MVA_BOOTSTRAP)
		return -E2BIG;

	/* Stage cluster (may be zero if every on-core segment is pure .bss). */
	if (stage_total) {
		ret = vp6_elf_map_cluster(c, dom, VP6_MVA_L1_STAGE,
					  PAGE_ALIGN(stage_total), &stage_va);
		if (ret)
			return ret;
	}
	ret = vp6_elf_map_cluster(c, dom, VP6_MVA_BOOT_DESC, PAGE_SIZE, &desc_va);
	if (ret)
		return ret;
	ret = vp6_elf_map_cluster(c, dom, VP6_MVA_BOOTSTRAP, boot_sz, &boot_va);
	if (ret)
		return ret;

	/* Pass B: copy each on-core segment's file bytes into the stage cluster. */
	for (i = 0; i < eh->e_phnum; i++) {
		u32 pa = ph[i].p_paddr;

		if (ph[i].p_type != PT_LOAD || ph[i].p_filesz == 0 ||
		    pa < VP6_DSP_LOCAL_BASE)
			continue;
		memcpy((u8 *)stage_va + stage_off,
		       (const u8 *)img + ph[i].p_offset, ph[i].p_filesz);
		stage_off += round_up(ph[i].p_filesz, 4);
	}

	/* Descriptor: entry, then the {dst,src} records (LE); no count/len needed. */
	d = desc_va;
	*d++ = eh->e_entry;
	for (i = 0; i < nseg; i++) {
		*d++ = seg[i].dst;
		*d++ = seg[i].src;
	}

	/* Emit the unrolled bootstrap: lit_desc at +0, _start at +ENTRY_OFF. */
	*(u32 *)boot_va = VP6_MVA_BOOT_DESC;
	vp6_gen_bootstrap((u8 *)boot_va + VP6_BOOTSTRAP_ENTRY_OFF, seg, nseg);

	/* Repoint the boot at the bootstrap; it jumps to the blob entry itself. */
	c->fw_dma = VP6_MVA_BOOTSTRAP;
	c->fw_entry = VP6_BOOTSTRAP_ENTRY_OFF;

	dev_info(c->dev,
		 "vp6-%u: L1 unrolled bootstrap: %u seg(s), %zu staged B, %zu code B, entry 0x%x via 0x%x\n",
		 c->core_index, nseg, stage_total, code_sz, eh->e_entry, VP6_MVA_BOOTSTRAP);
	return 0;
}

/*
 * BOOTSTRAP_DIAG: place the base-ISA L1-copy DISAMBIGUATION probe instead of the
 * production bootstrap. It does NOT copy L1 or jump to the blob; it writes four
 * markers to independently-observable targets and spins, to separate the two
 * candidate causes of the Phase-O copy fault:
 *   (1) the fixed-MVA iommu_map cluster not serving the DSP like a dynamic
 *       dma_alloc_coherent IOVA, vs
 *   (2) an on-core L1 s32i escaping out the AXI-via-M4U master.
 *
 * Targets: a DYNAMIC dma_alloc_coherent scratch (the Phase-J-proven known-good
 * control, host-readable via its kernel VA), the fixed-MVA staging buffer
 * (VP6_MVA_L1_STAGE), and the first on-core segment's dst (0x7ff..). The DRAM
 * markers are read back with VP6_IOC_READ_DIAG; the on-core store is judged by
 * the dmesg M4U fault. See mt8183-vp6-status.md Phase P.
 */
static int vp6_elf_stage_bootstrap_diag(struct vp6_core *c,
					const struct elf32_hdr *eh,
					const struct elf32_phdr *ph,
					struct iommu_domain *dom)
{
	void *stage_va = NULL, *desc_va = NULL, *boot_va = NULL;
	u32 diag_l1 = 0;
	unsigned int i;
	u32 *d, *bs;
	int ret;

	/* First on-core segment dst -> the L1 write target. */
	for (i = 0; i < eh->e_phnum; i++) {
		if (ph[i].p_type == PT_LOAD && ph[i].p_memsz &&
		    ph[i].p_paddr >= VP6_DSP_LOCAL_BASE) {
			diag_l1 = ph[i].p_paddr;
			break;
		}
	}
	if (!diag_l1)
		return 0;		/* nothing on-core -- nothing to probe */

	/* Dynamic host-readable scratch (the known-good control, Phase-J path). */
	c->diag_scratch_size = PAGE_SIZE;
	c->diag_scratch_va = dma_alloc_coherent(c->dev, c->diag_scratch_size,
						&c->diag_scratch_dma, GFP_KERNEL);
	if (!c->diag_scratch_va)
		return -ENOMEM;
	memset(c->diag_scratch_va, 0, c->diag_scratch_size);

	/* Fixed-MVA staging (write target) + descriptor + bootstrap clusters. */
	ret = vp6_elf_map_cluster(c, dom, VP6_MVA_L1_STAGE, PAGE_SIZE, &stage_va);
	if (ret)
		return ret;
	ret = vp6_elf_map_cluster(c, dom, VP6_MVA_BOOT_DESC, PAGE_SIZE, &desc_va);
	if (ret)
		return ret;
	ret = vp6_elf_map_cluster(c, dom, VP6_MVA_BOOTSTRAP, PAGE_SIZE, &boot_va);
	if (ret)
		return ret;
	c->diag_staging_va = stage_va;

	/* Descriptor: [0] marker, [1] on-core L1 dst, [2] fixed-MVA staging target. */
	d = desc_va;
	d[0] = VP6_DIAG_MARKER;
	d[1] = diag_l1;
	d[2] = VP6_MVA_L1_STAGE;

	/* Place the probe + patch lit_scratch (payload +0) = dynamic scratch IOVA. */
	memcpy(boot_va, vp6_bootstrap_diag_bin, sizeof(vp6_bootstrap_diag_bin));
	bs = boot_va;
	bs[VP6_DIAG_LIT_SCRATCH_OFF / 4] = lower_32_bits(c->diag_scratch_dma);

	c->fw_dma = VP6_MVA_BOOTSTRAP;
	c->fw_entry = VP6_BOOTSTRAP_ENTRY_OFF;

	dev_info(c->dev,
		 "vp6-%u: L1 DIAG probe: scratch dma 0x%llx, L1 dst 0x%x, fixed-stage 0x%x, marker 0x%08x, entry 0x%x via 0x%x\n",
		 c->core_index, (u64)c->diag_scratch_dma, diag_l1,
		 VP6_MVA_L1_STAGE, VP6_DIAG_MARKER, VP6_BOOTSTRAP_ENTRY_OFF,
		 VP6_MVA_BOOTSTRAP);
	return 0;
}

/*
 * BOOTSTRAP_DIAG2: place the base-ISA CONTROL-FLOW disambiguation probe (Phase Q).
 * Self-contained: it does NOT copy L1, use the descriptor, or jump to the blob --
 * it runs a beqz pair, a counted loop, and a jx to a driver-patched runtime target,
 * writing host-readable markers to the DYNAMIC scratch (the Phase-J-proven store
 * path), then spins. VP6_IOC_READ_DIAG2 reads the scratch words back; a dmesg M4U
 * fault + which markers landed pin exactly which control-flow form the FLIX
 * mis-decodes. Two literals are patched: lit_scratch (payload +0) = dynamic scratch
 * IOVA, lit_jxtgt (payload +4) = VP6_MVA_BOOTSTRAP + VP6_DIAG2_JXTGT_OFF (the jx
 * target's absolute runtime address). See mt8183-vp6-status.md Phase Q.
 */
static int vp6_elf_stage_bootstrap_diag2(struct vp6_core *c, struct iommu_domain *dom)
{
	void *boot_va = NULL;
	u32 *bs;
	int ret;

	/* Dynamic host-readable scratch (the Phase-J known-good store path). */
	c->diag_scratch_size = PAGE_SIZE;
	c->diag_scratch_va = dma_alloc_coherent(c->dev, c->diag_scratch_size,
						&c->diag_scratch_dma, GFP_KERNEL);
	if (!c->diag_scratch_va)
		return -ENOMEM;
	memset(c->diag_scratch_va, 0, c->diag_scratch_size);

	ret = vp6_elf_map_cluster(c, dom, VP6_MVA_BOOTSTRAP, PAGE_SIZE, &boot_va);
	if (ret)
		return ret;

	memcpy(boot_va, vp6_bootstrap_diag2_bin, sizeof(vp6_bootstrap_diag2_bin));
	bs = boot_va;
	bs[VP6_DIAG_LIT_SCRATCH_OFF / 4] = lower_32_bits(c->diag_scratch_dma);
	bs[VP6_DIAG2_LIT_JXTGT_OFF / 4]  = VP6_MVA_BOOTSTRAP + VP6_DIAG2_JXTGT_OFF;

	c->fw_dma = VP6_MVA_BOOTSTRAP;
	c->fw_entry = VP6_DIAG2_ENTRY_OFF;

	dev_info(c->dev,
		 "vp6-%u: control-flow DIAG2 probe: scratch dma 0x%llx, jxtgt 0x%x, entry 0x%x via 0x%x\n",
		 c->core_index, (u64)c->diag_scratch_dma,
		 VP6_MVA_BOOTSTRAP + VP6_DIAG2_JXTGT_OFF,
		 VP6_DIAG2_ENTRY_OFF, VP6_MVA_BOOTSTRAP);
	return 0;
}

/*
 * BOOTSTRAP_DIAG3: place the base-ISA L1-COPY-OP disambiguation probe (Phase R).
 * Phase Q refuted control-flow yet the unrolled straight-line copy still faults, so
 * the residual is the L1 COPY op. The probe runs seven isolated sub-ops that cross
 * the two axes separating the faulting copy from every proven-good path -- the
 * MEMORY TARGET (fixed-MVA STAGING read + on-core IRAM store, neither tested by
 * DIAG1) and the ENCODING WIDTH (WIDE 24-bit l32i/s32i, the generator's form, vs the
 * proven NARROW density) -- writing host-readable markers to the DYNAMIC scratch.
 * VP6_IOC_READ_DIAG2 reads the words back; the dmesg M4U fault (read/write attr)
 * pins which sub-op faults. It uses a real fixed-MVA staging cluster (seeded [0] =
 * VP6_DIAG_MARKER so the staging read/copies echo a known value) but no descriptor
 * (stage/dmem/iram are the probe's own fixed-MVA literals). See mt8183-vp6-status.md
 * Phase R.
 */
static int vp6_elf_stage_bootstrap_diag3(struct vp6_core *c, struct iommu_domain *dom)
{
	void *stage_va = NULL, *boot_va = NULL;
	u32 *bs;
	int ret;

	/* Dynamic host-readable scratch (the Phase-I/J known-good marker-store path). */
	c->diag_scratch_size = PAGE_SIZE;
	c->diag_scratch_va = dma_alloc_coherent(c->dev, c->diag_scratch_size,
						&c->diag_scratch_dma, GFP_KERNEL);
	if (!c->diag_scratch_va)
		return -ENOMEM;
	memset(c->diag_scratch_va, 0, c->diag_scratch_size);

	/* Fixed-MVA staging cluster (the real copy src) -- seed [0] = the marker. */
	ret = vp6_elf_map_cluster(c, dom, VP6_MVA_L1_STAGE, PAGE_SIZE, &stage_va);
	if (ret)
		return ret;
	*(u32 *)stage_va = VP6_DIAG_MARKER;
	c->diag_staging_va = stage_va;

	ret = vp6_elf_map_cluster(c, dom, VP6_MVA_BOOTSTRAP, PAGE_SIZE, &boot_va);
	if (ret)
		return ret;

	memcpy(boot_va, vp6_bootstrap_diag3_bin, sizeof(vp6_bootstrap_diag3_bin));
	bs = boot_va;
	bs[VP6_DIAG_LIT_SCRATCH_OFF / 4] = lower_32_bits(c->diag_scratch_dma);

	c->fw_dma = VP6_MVA_BOOTSTRAP;
	c->fw_entry = VP6_DIAG3_ENTRY_OFF;

	dev_info(c->dev,
		 "vp6-%u: L1-copy DIAG3 probe: scratch dma 0x%llx, staging 0x%x (marker 0x%08x), entry 0x%x via 0x%x\n",
		 c->core_index, (u64)c->diag_scratch_dma, VP6_MVA_L1_STAGE,
		 VP6_DIAG_MARKER, VP6_DIAG3_ENTRY_OFF, VP6_MVA_BOOTSTRAP);
	return 0;
}

/*
 * BOOTSTRAP_DIAG4: place the base-ISA MULTI-WORD copy disambiguation probe (Phase R,
 * 2nd lever). DIAG3 proved every single copy sub-op lands yet the production copy
 * still faults, so the residual is multi-word/interaction-specific. This probe
 * isolates the `addmi` walking-base op (emitted only for >256-word segments) and a
 * long unbroken l32i/s32i run -- the two elements DIAG3 never exercised. It seeds the
 * fixed-MVA staging cluster's first 512 words with VP6_DIAG_MARKER so the copy runs
 * read a known value, and writes host-readable markers (incl. the addmi RESULT
 * register value) to the DYNAMIC scratch. VP6_IOC_READ_DIAG2 reads them back. See
 * mt8183-vp6-status.md Phase R.
 */
static int vp6_elf_stage_bootstrap_diag4(struct vp6_core *c, struct iommu_domain *dom)
{
	void *stage_va = NULL, *boot_va = NULL;
	u32 *bs, *st;
	unsigned int i;
	int ret;

	/* Dynamic host-readable scratch (the Phase-I/J known-good marker-store path). */
	c->diag_scratch_size = PAGE_SIZE;
	c->diag_scratch_va = dma_alloc_coherent(c->dev, c->diag_scratch_size,
						&c->diag_scratch_dma, GFP_KERNEL);
	if (!c->diag_scratch_va)
		return -ENOMEM;
	memset(c->diag_scratch_va, 0, c->diag_scratch_size);

	/* Fixed-MVA staging cluster (copy src) -- seed the first 512 words = marker. */
	ret = vp6_elf_map_cluster(c, dom, VP6_MVA_L1_STAGE, PAGE_SIZE, &stage_va);
	if (ret)
		return ret;
	st = stage_va;
	for (i = 0; i < PAGE_SIZE / 4; i++)
		st[i] = VP6_DIAG_MARKER;
	c->diag_staging_va = stage_va;

	ret = vp6_elf_map_cluster(c, dom, VP6_MVA_BOOTSTRAP, PAGE_SIZE, &boot_va);
	if (ret)
		return ret;

	memcpy(boot_va, vp6_bootstrap_diag4_bin, sizeof(vp6_bootstrap_diag4_bin));
	bs = boot_va;
	bs[VP6_DIAG_LIT_SCRATCH_OFF / 4] = lower_32_bits(c->diag_scratch_dma);

	c->fw_dma = VP6_MVA_BOOTSTRAP;
	c->fw_entry = VP6_DIAG4_ENTRY_OFF;

	dev_info(c->dev,
		 "vp6-%u: multi-word DIAG4 probe: scratch dma 0x%llx, staging 0x%x (marker 0x%08x), entry 0x%x via 0x%x\n",
		 c->core_index, (u64)c->diag_scratch_dma, VP6_MVA_L1_STAGE,
		 VP6_DIAG_MARKER, VP6_DIAG4_ENTRY_OFF, VP6_MVA_BOOTSTRAP);
	return 0;
}

/*
 * BOOTSTRAP_DIAG5: place the base-ISA DESCRIPTOR-l32i-BASE copy disambiguation probe
 * (Phase S). DIAG3/DIAG4 proved every isolated copy sub-op lands (staging read; narrow
 * +wide copy->DMEM/IRAM; addmi walk; long run; the full >256+addmi->IRAM shape) yet the
 * production copy still faults. All prior probes loaded the copy bases via `l32r`
 * LITERALS; the production vp6_gen_bootstrap loads them via `l32i a5,a2,off` FROM A LIVE
 * DESCRIPTOR held in a2 -- the ONE remaining untested delta. This probe builds a REAL
 * 11-word descriptor at VP6_MVA_BOOT_DESC (desc[0] = the dynamic scratch IOVA for T1's
 * l32i-base store; desc[1..10] = the on-core dst / staging src MVAs for the 5-segment
 * T2 copy) and seeds VP6_MVA_L1_STAGE[0..1023] = VP6_DIAG_MARKER so the copies read a
 * known value. It stores host-readable markers (narrow, l32r-literal base -- the proven
 * path) to the DYNAMIC scratch and, for T1, ALSO stores through the l32i-descriptor-
 * loaded base. VP6_IOC_READ_DIAG2 reads the words back; a dmesg M4U translation fault
 * (fixable decode) vs a wedge / mtk-wdt reset with no fault (feasibility.md 3.1 depth-4
 * read-queue overflow -- hardware) discriminates the residual. See mt8183-vp6-status.md
 * Phase S. Payload word 0 (lit_scratch) is PATCHED at load. Mutually exclusive with
 * BOOTSTRAP / DIAG..DIAG4 / ALLOW_PARTIAL.
 */
static int vp6_elf_stage_bootstrap_diag5(struct vp6_core *c, struct iommu_domain *dom)
{
	void *stage_va = NULL, *desc_va = NULL, *boot_va = NULL;
	u32 *bs, *st, *d;
	unsigned int i;
	int ret;

	/* Dynamic host-readable scratch (the Phase-I/J known-good marker-store path). */
	c->diag_scratch_size = PAGE_SIZE;
	c->diag_scratch_va = dma_alloc_coherent(c->dev, c->diag_scratch_size,
						&c->diag_scratch_dma, GFP_KERNEL);
	if (!c->diag_scratch_va)
		return -ENOMEM;
	memset(c->diag_scratch_va, 0, c->diag_scratch_size);

	/* Fixed-MVA staging cluster (copy src) -- seed the whole page = marker. */
	ret = vp6_elf_map_cluster(c, dom, VP6_MVA_L1_STAGE, PAGE_SIZE, &stage_va);
	if (ret)
		return ret;
	st = stage_va;
	for (i = 0; i < PAGE_SIZE / 4; i++)
		st[i] = VP6_DIAG_MARKER;
	c->diag_staging_va = stage_va;

	/* Real descriptor at VP6_MVA_BOOT_DESC -- the probe reads bases via l32i a5,a2,off
	 * from here (a2 = VP6_MVA_BOOT_DESC, held live), exactly the production idiom.
	 * desc[0] = dynamic scratch IOVA (T1's l32i-loaded store base); desc[1..10] =
	 * the five (dst,src) pairs: on-core IRAM/DMEM dst + fixed-MVA staging src. */
	ret = vp6_elf_map_cluster(c, dom, VP6_MVA_BOOT_DESC, PAGE_SIZE, &desc_va);
	if (ret)
		return ret;
	d = desc_va;
	d[0]  = lower_32_bits(c->diag_scratch_dma);	/* T1 l32i-base store target */
	d[1]  = 0x7ff40000u;	d[2]  = VP6_MVA_L1_STAGE;	/* seg0: IRAM,   staging */
	d[3]  = 0x7ff3e000u;	d[4]  = VP6_MVA_L1_STAGE;	/* seg1: DMEM,   staging */
	d[5]  = 0x7ff41000u;	d[6]  = VP6_MVA_L1_STAGE;	/* seg2: IRAM+4K,staging */
	d[7]  = 0x7ff3e800u;	d[8]  = VP6_MVA_L1_STAGE;	/* seg3: DMEM+2K,staging */
	d[9]  = 0x7ff42000u;	d[10] = VP6_MVA_L1_STAGE;	/* seg4: IRAM+8K,staging */

	ret = vp6_elf_map_cluster(c, dom, VP6_MVA_BOOTSTRAP, PAGE_SIZE, &boot_va);
	if (ret)
		return ret;

	memcpy(boot_va, vp6_bootstrap_diag5_bin, sizeof(vp6_bootstrap_diag5_bin));
	bs = boot_va;
	bs[VP6_DIAG_LIT_SCRATCH_OFF / 4] = lower_32_bits(c->diag_scratch_dma);

	c->fw_dma = VP6_MVA_BOOTSTRAP;
	c->fw_entry = VP6_DIAG5_ENTRY_OFF;

	dev_info(c->dev,
		 "vp6-%u: descriptor-l32i-base DIAG5 probe: scratch dma 0x%llx, desc 0x%x, staging 0x%x (marker 0x%08x), entry 0x%x via 0x%x\n",
		 c->core_index, (u64)c->diag_scratch_dma, VP6_MVA_BOOT_DESC,
		 VP6_MVA_L1_STAGE, VP6_DIAG_MARKER, VP6_DIAG5_ENTRY_OFF,
		 VP6_MVA_BOOTSTRAP);
	return 0;
}

/*
 * BOOTSTRAP_DIAG6: place the base-ISA jx-HANDOFF isolation probe (Phase S 2nd lever).
 * DIAG5 proved the full descriptor-l32i multi-region L1 copy lands clean, yet the
 * production bootstrap (copy + `jx a3` into the blob entry 0x7da00000) still faults on
 * the same kernel. This isolates the jx handoff: it writes a base-ISA LANDING PAD over
 * the head of the blob's already-loaded reset-vector cluster (at VP6_MVA_RESET_VECTOR),
 * builds a descriptor whose desc[0] = the pad's runtime entry, and places a mini-
 * bootstrap (NO copy) that writes w0=0x5a then `jx a3` to that entry. If the pad runs it
 * writes w1=0x66. w1==0x66 => the jx-to-external-blob-entry handoff WORKS (the production
 * fault is then the BLOB's own code = terminal RV33); w1==0 + a dmesg read fault =>
 * the handoff itself faults (fixable). Runs AFTER the external clusters are loaded, so
 * the reset-vector cluster's VA is looked up from c->elf_clusters. See Phase S.
 */
static int vp6_elf_stage_bootstrap_diag6(struct vp6_core *c, struct iommu_domain *dom)
{
	void *desc_va = NULL, *boot_va = NULL, *pad_va = NULL;
	u32 *bs, *ps, *d;
	unsigned int j;
	int ret;

	/* Dynamic host-readable scratch (the Phase-I/J known-good marker-store path). */
	c->diag_scratch_size = PAGE_SIZE;
	c->diag_scratch_va = dma_alloc_coherent(c->dev, c->diag_scratch_size,
						&c->diag_scratch_dma, GFP_KERNEL);
	if (!c->diag_scratch_va)
		return -ENOMEM;
	memset(c->diag_scratch_va, 0, c->diag_scratch_size);

	/* Overwrite the head of the already-loaded reset-vector cluster (0x7da00000) with
	 * the base-ISA landing pad -- so `jx a3` (a3 = 0x7da00000 + pad _start) lands on
	 * OUR marker-writing pad instead of the blob's real first instruction. */
	for (j = 0; j < c->elf_ncluster; j++) {
		struct vp6_elf_cluster *cle = &c->elf_clusters[j];

		if (VP6_MVA_RESET_VECTOR >= cle->iova &&
		    VP6_MVA_RESET_VECTOR < cle->iova + cle->size) {
			pad_va = (u8 *)cle->va + (VP6_MVA_RESET_VECTOR - cle->iova);
			break;
		}
	}
	if (!pad_va) {
		dev_warn(c->dev, "vp6-%u: DIAG6: no reset-vector cluster @0x%x to overwrite\n",
			 c->core_index, VP6_MVA_RESET_VECTOR);
		return -ENXIO;
	}
	memcpy(pad_va, vp6_diag6_pad_bin, sizeof(vp6_diag6_pad_bin));
	ps = pad_va;
	ps[VP6_DIAG_LIT_SCRATCH_OFF / 4] = lower_32_bits(c->diag_scratch_dma);

	/* Descriptor: desc[0] = the landing-pad runtime entry (reset-vector MVA + _start). */
	ret = vp6_elf_map_cluster(c, dom, VP6_MVA_BOOT_DESC, PAGE_SIZE, &desc_va);
	if (ret)
		return ret;
	d = desc_va;
	d[0] = VP6_MVA_RESET_VECTOR + VP6_DIAG6_PAD_ENTRY_OFF;

	/* Mini-bootstrap at VP6_MVA_BOOTSTRAP: write w0, then jx a3 (= desc[0]). */
	ret = vp6_elf_map_cluster(c, dom, VP6_MVA_BOOTSTRAP, PAGE_SIZE, &boot_va);
	if (ret)
		return ret;
	memcpy(boot_va, vp6_bootstrap_diag6_bin, sizeof(vp6_bootstrap_diag6_bin));
	bs = boot_va;
	bs[VP6_DIAG_LIT_SCRATCH_OFF / 4] = lower_32_bits(c->diag_scratch_dma);

	c->fw_dma = VP6_MVA_BOOTSTRAP;
	c->fw_entry = VP6_DIAG6_ENTRY_OFF;

	dev_info(c->dev,
		 "vp6-%u: jx-handoff DIAG6 probe: scratch dma 0x%llx, pad @0x%x+0x%x, jx tgt 0x%x, entry 0x%x via 0x%x\n",
		 c->core_index, (u64)c->diag_scratch_dma, VP6_MVA_RESET_VECTOR,
		 VP6_DIAG6_PAD_ENTRY_OFF, VP6_MVA_RESET_VECTOR + VP6_DIAG6_PAD_ENTRY_OFF,
		 VP6_DIAG6_ENTRY_OFF, VP6_MVA_BOOTSTRAP);
	return 0;
}

/*
 * Any BOOTSTRAP_DIAG variant. These base-ISA execution probes live in
 * vp6_load_elf (they replace the production bootstrap), so route a
 * DRAM-carveout-entry (RPROC) firmware -- e.g. xann_0, entry 0x50000000 --
 * through vp6_load_elf too when a DIAG flag is set, and relax the fixed-MVA
 * entry-range check for it. This lets the DIAG2 control-flow probe run against
 * the xann_0 altresetvec boot path on g350, disambiguating "the DSP executes
 * base-ISA" from "the DSP is not executing" INDEPENDENT of the (unknown)
 * core->host mailbox -- the marker lands in the host-readable dynamic scratch,
 * not via any RTOS/rpmsg/trace path.
 */
#define VP6_LOAD_ELF_DIAG_MASK \
	(VP6_LOAD_ELF_BOOTSTRAP_DIAG | VP6_LOAD_ELF_BOOTSTRAP_DIAG2 | \
	 VP6_LOAD_ELF_BOOTSTRAP_DIAG3 | VP6_LOAD_ELF_BOOTSTRAP_DIAG4 | \
	 VP6_LOAD_ELF_BOOTSTRAP_DIAG5 | VP6_LOAD_ELF_BOOTSTRAP_DIAG6)

/*
 * Load a vendor multi-segment Xtensa ELF at its fixed MVAs (VP6_IOC_LOAD_ELF).
 *
 * A cam_vpu*-class firmware is an ET_EXEC Xtensa ELF whose external code/data
 * PT_LOADs carry ABSOLUTE literals resolved at fixed VMAs, so each must land at
 * its exact p_paddr in the VP6 M4U domain -- reset vector at
 * VP6_MVA_RESET_VECTOR, main program at VP6_MVA_MAIN_PROGRAM. This mirrors the
 * mt6771 == MT8183-die BSP's vpu_map_mva_of_bin (reset/main m4u_alloc_mva'd to
 * fixed MVAs; vpu/mt6771/vpu_hw.c:1440-1517), ported onto mainline mtk-iommu:
 * adjacent PT_LOADs are merged into page-aligned clusters, each a
 * force-contiguous coherent DRAM alloc iommu_map'd at its fixed IOVA; gaps and
 * .bss (memsz > filesz) are zero-filled. altresetvec := e_entry.
 *
 * Segments at/above VP6_DSP_LOCAL_BASE are the core's own L1 IRAM/DMEM, which
 * the direct-host MT8183 generation gives the AP no window to write (the BSP
 * DRAM-stages the IRAM image + the DSP self-copies it -- a handshake an
 * RV33-generation blob does not carry). With VP6_LOAD_ELF_BOOTSTRAP we supply
 * that handshake (vp6_elf_stage_bootstrap: DRAM-stage the on-core bytes + a
 * base-ISA DSP self-copy bootstrap). Otherwise they cannot be placed; unless
 * VP6_LOAD_ELF_ALLOW_PARTIAL is set the load is refused so the caller is not
 * misled into START-ing an image that will fault fetching uninitialised L1.
 */
static int vp6_load_elf(struct vp6_core *c, const void *img, size_t len,
			u32 flags)
{
	const struct elf32_hdr *eh = img;
	const struct elf32_phdr *ph;
	struct { u32 lo, hi; } cl[VP6_ELF_MAX_CLUSTERS];
	struct iommu_domain *dom;
	unsigned int i, ncl = 0, oncore = 0;
	int ret;

	if (len < sizeof(*eh))
		return -EINVAL;
	if (memcmp(eh->e_ident, ELFMAG, SELFMAG) ||
	    eh->e_ident[EI_CLASS] != ELFCLASS32 ||
	    eh->e_ident[EI_DATA] != ELFDATA2LSB ||
	    eh->e_type != ET_EXEC || eh->e_machine != EM_XTENSA)
		return -EINVAL;
	if (!eh->e_phnum || eh->e_phentsize != sizeof(*ph))
		return -EINVAL;
	if ((size_t)eh->e_phoff + (size_t)eh->e_phnum * sizeof(*ph) > len)
		return -EINVAL;
	/*
	 * The cam_vpu-class blobs have their reset vector in the fixed-MVA range
	 * [VP6_MVA_RESET_VECTOR, VP6_DSP_LOCAL_BASE). The RPROC firmware
	 * (rproc-vp6-fw) instead enters at its DRAM-carveout reset vector
	 * (e_entry 0x50000000, a resource-table CARVEOUT placed as an external
	 * cluster below VP6_DSP_LOCAL_BASE), so VP6_LOAD_ELF_RPROC relaxes ONLY
	 * this entry-vector check -- Pass 1 still classifies its segments as
	 * external clusters (p_paddr < VP6_DSP_LOCAL_BASE) and maps them normally.
	 */
	if (!(flags & (VP6_LOAD_ELF_RPROC | VP6_LOAD_ELF_DIAG_MASK)) &&
	    (eh->e_entry < VP6_MVA_RESET_VECTOR ||
	     eh->e_entry >= VP6_DSP_LOCAL_BASE)) {
		dev_err(c->dev,
			"vp6-%u: rejecting e_entry %#x (outside MVA reset range); pass VP6_LOAD_ELF_RPROC for a DRAM-carveout entry\n",
			c->core_index, eh->e_entry);
		return -EINVAL;
	}

	dom = iommu_get_domain_for_dev(c->dev);
	if (!dom)
		return -ENODEV;		/* needs the vp6 node's `iommus` (M4U) */

	ph = (const struct elf32_phdr *)((const u8 *)img + eh->e_phoff);

	/* Pass 1: classify + build page-aligned external clusters. */
	for (i = 0; i < eh->e_phnum; i++) {
		u32 lo, hi;

		if (ph[i].p_type != PT_LOAD || ph[i].p_memsz == 0)
			continue;
		if ((size_t)ph[i].p_offset + ph[i].p_filesz > len)
			return -EINVAL;
		if (ph[i].p_paddr >= VP6_DSP_LOCAL_BASE) {
			if (ph[i].p_filesz)	/* has bytes we would need to place */
				oncore++;
			continue;
		}
		lo = round_down(ph[i].p_paddr, PAGE_SIZE);
		hi = round_up(ph[i].p_paddr + ph[i].p_memsz, PAGE_SIZE);
		if (ncl && lo <= cl[ncl - 1].hi + VP6_ELF_MERGE_GAP) {
			cl[ncl - 1].hi = max(cl[ncl - 1].hi, hi);
		} else {
			if (ncl == VP6_ELF_MAX_CLUSTERS)
				return -E2BIG;
			cl[ncl].lo = lo;
			cl[ncl].hi = hi;
			ncl++;
		}
	}
	if (!ncl)
		return -EINVAL;

	if (oncore &&
	    !(flags & (VP6_LOAD_ELF_ALLOW_PARTIAL | VP6_LOAD_ELF_BOOTSTRAP |
		       VP6_LOAD_ELF_BOOTSTRAP_DIAG | VP6_LOAD_ELF_BOOTSTRAP_DIAG2 |
		       VP6_LOAD_ELF_BOOTSTRAP_DIAG3 | VP6_LOAD_ELF_BOOTSTRAP_DIAG4 |
		       VP6_LOAD_ELF_BOOTSTRAP_DIAG5 |
		       VP6_LOAD_ELF_BOOTSTRAP_DIAG6))) {
		dev_warn(c->dev,
			 "vp6-%u: ELF carries %u on-core (IRAM/DMEM) segment(s) the AP cannot place on this direct-host VP6 (needs the RV33/apusys stack); pass VP6_LOAD_ELF_BOOTSTRAP to bridge via the DSP L1 self-copy, or VP6_LOAD_ELF_ALLOW_PARTIAL to load the external segments only\n",
			 c->core_index, oncore);
		return -EOPNOTSUPP;
	}

	vp6_free_fw(c);		/* drop any previous image */

	/* Pass 2: allocate + fixed-MVA map each external cluster. */
	for (i = 0; i < ncl; i++) {
		ret = vp6_elf_map_cluster(c, dom, cl[i].lo,
					  cl[i].hi - cl[i].lo, NULL);
		if (ret)
			goto err;
	}

	/* Pass 3: copy each external PT_LOAD's file bytes into its cluster. */
	for (i = 0; i < eh->e_phnum; i++) {
		u32 pa = ph[i].p_paddr;
		unsigned int j;

		if (ph[i].p_type != PT_LOAD || ph[i].p_filesz == 0 ||
		    pa >= VP6_DSP_LOCAL_BASE)
			continue;
		for (j = 0; j < c->elf_ncluster; j++) {
			struct vp6_elf_cluster *cle = &c->elf_clusters[j];

			if (pa >= cle->iova && pa < cle->iova + cle->size) {
				memcpy((u8 *)cle->va + (pa - cle->iova),
				       (const u8 *)img + ph[i].p_offset,
				       ph[i].p_filesz);
				break;
			}
		}
	}

	/* Default boot: reset vector = the blob entry (e_entry at 0x7da00000). */
	c->fw_dma = VP6_MVA_RESET_VECTOR;
	c->fw_entry = eh->e_entry - VP6_MVA_RESET_VECTOR;
	c->fw_size = len;
	c->elf_loaded = true;		/* set before the call so err path frees clusters */

	/*
	 * BOOTSTRAP: stage the on-core L1 segments in DRAM + place the base-ISA
	 * self-copy bootstrap, and repoint the reset vector at it (overrides
	 * fw_dma/fw_entry above). A no-op if the blob has no on-core segments.
	 */
	if ((flags & VP6_LOAD_ELF_BOOTSTRAP_DIAG6) && oncore) {
		ret = vp6_elf_stage_bootstrap_diag6(c, dom);
		if (ret)
			goto err;
	} else if ((flags & VP6_LOAD_ELF_BOOTSTRAP_DIAG5) && oncore) {
		ret = vp6_elf_stage_bootstrap_diag5(c, dom);
		if (ret)
			goto err;
	} else if ((flags & VP6_LOAD_ELF_BOOTSTRAP_DIAG4) && oncore) {
		ret = vp6_elf_stage_bootstrap_diag4(c, dom);
		if (ret)
			goto err;
	} else if ((flags & VP6_LOAD_ELF_BOOTSTRAP_DIAG3) && oncore) {
		ret = vp6_elf_stage_bootstrap_diag3(c, dom);
		if (ret)
			goto err;
	} else if ((flags & VP6_LOAD_ELF_BOOTSTRAP_DIAG2) && oncore) {
		ret = vp6_elf_stage_bootstrap_diag2(c, dom);
		if (ret)
			goto err;
	} else if ((flags & VP6_LOAD_ELF_BOOTSTRAP_DIAG) && oncore) {
		ret = vp6_elf_stage_bootstrap_diag(c, eh, ph, dom);
		if (ret)
			goto err;
	} else if ((flags & VP6_LOAD_ELF_BOOTSTRAP) && oncore) {
		ret = vp6_elf_stage_bootstrap(c, img, len, eh, ph, dom);
		if (ret)
			goto err;
	}

	strscpy(c->fw_ver, "vendor-elf", VP6_FW_VER_LEN);
	c->state = VP6_STATE_LOADED;

	dev_info(c->dev,
		 "vp6-%u: staged ELF: %u cluster(s) at fixed MVA, entry 0x%x%s\n",
		 c->core_index, c->elf_ncluster, eh->e_entry,
		 (flags & VP6_LOAD_ELF_BOOTSTRAP_DIAG6) && oncore ? " [BOOTSTRAP_DIAG6: jx-handoff isolation probe]" :
		 (flags & VP6_LOAD_ELF_BOOTSTRAP_DIAG5) && oncore ? " [BOOTSTRAP_DIAG5: descriptor-l32i-base copy probe, no jump]" :
		 (flags & VP6_LOAD_ELF_BOOTSTRAP_DIAG4) && oncore ? " [BOOTSTRAP_DIAG4: multi-word copy probe, no jump]" :
		 (flags & VP6_LOAD_ELF_BOOTSTRAP_DIAG3) && oncore ? " [BOOTSTRAP_DIAG3: L1-copy-op probe, no jump]" :
		 (flags & VP6_LOAD_ELF_BOOTSTRAP_DIAG2) && oncore ? " [BOOTSTRAP_DIAG2: control-flow probe, no jump]" :
		 (flags & VP6_LOAD_ELF_BOOTSTRAP_DIAG) && oncore ? " [BOOTSTRAP_DIAG: L1-copy probe, no jump]" :
		 (flags & VP6_LOAD_ELF_BOOTSTRAP) && oncore ? " [BOOTSTRAP: on-core L1 DSP-self-copied]" :
		 oncore ? " [PARTIAL: on-core segments unplaced]" : "");
	return 0;

err:
	vp6_unload_elf(c);
	return ret;
}

/* ---- rproc virtio-rpmsg shim (VP6_LOAD_ELF_RPROC) ------------------------ */

/* The mapped carveout cluster whose IOVA range contains da, else NULL. */
static struct vp6_elf_cluster *vp6_cluster_for_da(struct vp6_core *c, u32 da)
{
	unsigned int i;

	for (i = 0; i < c->elf_ncluster; i++) {
		struct vp6_elf_cluster *cl = &c->elf_clusters[i];

		if (da >= cl->iova && da < cl->iova + cl->size)
			return cl;
	}
	return NULL;
}

/* Host VA for a device address within an already-mapped carveout cluster. */
static void *vp6_da_to_va(struct vp6_core *c, u32 da)
{
	struct vp6_elf_cluster *cl = vp6_cluster_for_da(c, da);

	return cl ? (u8 *)cl->va + (da - cl->iova) : NULL;
}

/*
 * Load the vendor OpenAMP remoteproc firmware (rproc-vp6-fw / xann_0) as a
 * minimal virtio-rpmsg peer. This is a standard DRAM-resident remoteproc ELF
 * (entry 0x50000000, absolute addresses) whose code/data live in the CARVEOUTS
 * declared by its .resource_table -- NOT one PT_LOAD per fixed literal like the
 * cam_vpu LOAD_ELF path. We allocate + fixed-MVA map every carveout at its FULL
 * device address (System RAM is 16 MB, far bigger than the ~4.5 MB image,
 * because the firmware's stack + heap live in its upper part -- per-PT_LOAD
 * mapping would leave them unbacked), copy the ELF segments in by da, skip the
 * 52-byte on-core .dram0.data (the direct-host L1 wall -- PARTIAL), and record
 * the trace ring + the VDEV status/vring fields for VP6_IOC_RPROC_READY.
 * altresetvec = e_entry. See the internal VP6 rproc-shim record.
 */
static int vp6_load_rproc(struct vp6_core *c, const void *img, size_t len)
{
	const struct elf32_hdr *eh = img;
	const struct elf32_phdr *ph;
	const struct elf32_shdr *sh, *rsc_sh = NULL;
	const struct vp6_rsc_table *rt;
	struct iommu_domain *dom;
	const char *shstr;
	void *rsc_va;
	unsigned int i, v, oncore = 0;
	bool have_vdev = false;
	int ret;

	if (len < sizeof(*eh))
		return -EINVAL;
	if (memcmp(eh->e_ident, ELFMAG, SELFMAG) ||
	    eh->e_ident[EI_CLASS] != ELFCLASS32 ||
	    eh->e_ident[EI_DATA] != ELFDATA2LSB ||
	    eh->e_type != ET_EXEC || eh->e_machine != EM_XTENSA)
		return -EINVAL;
	if (!eh->e_phnum || eh->e_phentsize != sizeof(*ph) ||
	    (size_t)eh->e_phoff + (size_t)eh->e_phnum * sizeof(*ph) > len)
		return -EINVAL;
	if (!eh->e_shnum || eh->e_shentsize != sizeof(*sh) ||
	    (size_t)eh->e_shoff + (size_t)eh->e_shnum * sizeof(*sh) > len ||
	    eh->e_shstrndx >= eh->e_shnum)
		return -EINVAL;
	/* The rproc entry lives in a DRAM carveout (e.g. 0x50000000), not L1. */
	if (eh->e_entry >= VP6_DSP_LOCAL_BASE)
		return -EINVAL;

	dom = iommu_get_domain_for_dev(c->dev);
	if (!dom)
		return -ENODEV;		/* needs the vp6 node's `iommus` (M4U) */

	ph = (const struct elf32_phdr *)((const u8 *)img + eh->e_phoff);
	sh = (const struct elf32_shdr *)((const u8 *)img + eh->e_shoff);

	/* Locate .resource_table via the section headers (fw is not stripped). */
	if ((size_t)sh[eh->e_shstrndx].sh_offset + sh[eh->e_shstrndx].sh_size > len)
		return -EINVAL;
	shstr = (const char *)img + sh[eh->e_shstrndx].sh_offset;
	for (i = 0; i < eh->e_shnum; i++) {
		if (sh[i].sh_name >= sh[eh->e_shstrndx].sh_size)
			continue;
		if (!strcmp(shstr + sh[i].sh_name, ".resource_table")) {
			rsc_sh = &sh[i];
			break;
		}
	}
	if (!rsc_sh || (size_t)rsc_sh->sh_offset + rsc_sh->sh_size > len ||
	    rsc_sh->sh_size < sizeof(*rt))
		return -EINVAL;
	rt = (const struct vp6_rsc_table *)((const u8 *)img + rsc_sh->sh_offset);
	if (rt->ver != 1 || rt->num == 0 || rt->num > VP6_RSC_MAX_ENTRIES ||
	    struct_size(rt, offset, rt->num) > rsc_sh->sh_size)
		return -EINVAL;

	vp6_free_fw(c);		/* drop any previous image */

	/*
	 * Pass 1: allocate + fixed-MVA map every CARVEOUT at its FULL declared
	 * device address. Skip those already covered by an earlier carveout.
	 */
	for (i = 0; i < rt->num; i++) {
		u32 off = rt->offset[i];
		const struct vp6_fw_rsc_carveout *co;

		if (off + sizeof(u32) > rsc_sh->sh_size)
			goto einval;
		if (*(const u32 *)((const u8 *)rt + off) != VP6_RSC_CARVEOUT)
			continue;
		if (off + sizeof(u32) + sizeof(*co) > rsc_sh->sh_size)
			goto einval;
		co = (const void *)((const u8 *)rt + off + sizeof(u32));
		if (!co->len || co->da >= VP6_DSP_LOCAL_BASE ||
		    vp6_cluster_for_da(c, co->da))
			continue;
		c->elf_loaded = true;	/* err path frees via vp6_unload_elf */
		ret = vp6_elf_map_cluster(c, dom, round_down(co->da, PAGE_SIZE),
					  round_up(co->da + co->len, PAGE_SIZE) -
						  round_down(co->da, PAGE_SIZE),
					  NULL);
		if (ret)
			goto err;
	}
	if (!c->elf_ncluster)
		goto einval;

	/* Pass 2: copy each external PT_LOAD's file bytes into its carveout. */
	for (i = 0; i < eh->e_phnum; i++) {
		u32 pa = ph[i].p_paddr;
		void *dst;

		if (ph[i].p_type != PT_LOAD || ph[i].p_memsz == 0)
			continue;
		if ((size_t)ph[i].p_offset + ph[i].p_filesz > len)
			goto einval;
		if (pa >= VP6_DSP_LOCAL_BASE) {		/* on-core .dram0.data */
			if (ph[i].p_filesz)
				oncore++;		/* skip -- PARTIAL */
			continue;
		}
		if (!ph[i].p_filesz)
			continue;		/* .bss: cluster already zeroed */
		dst = vp6_da_to_va(c, pa);
		if (!dst || vp6_da_to_va(c, pa + ph[i].p_filesz - 1) !=
			    (u8 *)dst + ph[i].p_filesz - 1)
			goto einval;		/* segment escapes its carveout */
		memcpy(dst, (const u8 *)img + ph[i].p_offset, ph[i].p_filesz);
	}

	/* Pass 3a: the trace/log ring + the vring carveout addresses. */
	for (i = 0; i < rt->num; i++) {
		u32 off = rt->offset[i], type;

		type = *(const u32 *)((const u8 *)rt + off);
		if (type == VP6_RSC_TRACE) {
			const struct vp6_fw_rsc_trace *tr =
				(const void *)((const u8 *)rt + off + sizeof(u32));

			if (off + sizeof(u32) + sizeof(*tr) > rsc_sh->sh_size)
				goto einval;
			c->trace_va = vp6_da_to_va(c, tr->da);
			c->trace_len = c->trace_va ? tr->len : 0;
		} else if (type == VP6_RSC_CARVEOUT) {
			const struct vp6_fw_rsc_carveout *co =
				(const void *)((const u8 *)rt + off + sizeof(u32));

			/* the rpmsg payload pool the host posts buffers from */
			if (!strncmp((const char *)co->name, "vdev0buffer", 11)) {
				c->rproc_buf_da = co->da;
				c->rproc_buf_va = vp6_da_to_va(c, co->da);
				c->rproc_buf_len = c->rproc_buf_va ? co->len : 0;
				continue;
			}
			/* vring memory = the vdev0vringN carveouts (by name) */
			if (strncmp((const char *)co->name, "vdev0vring", 10))
				continue;
			v = co->name[10] - '0';
			if (v >= VP6_RPROC_MAX_VRINGS)
				continue;
			c->rproc_vring_da[v] = co->da;
			c->rproc_vring_va[v] = vp6_da_to_va(c, co->da);
			c->rproc_vring_len[v] = co->len;
		}
	}

	/* Pass 3b: the VDEV status + per-vring da fields (live in System RAM). */
	rsc_va = vp6_da_to_va(c, rsc_sh->sh_addr);
	if (!rsc_va)
		goto einval;
	for (i = 0; i < rt->num; i++) {
		u32 off = rt->offset[i];
		struct vp6_fw_rsc_vdev *vd;
		unsigned int nv;

		if (*(const u32 *)((const u8 *)rt + off) != VP6_RSC_VDEV)
			continue;
		if (off + sizeof(u32) + sizeof(*vd) > rsc_sh->sh_size)
			goto einval;
		vd = (struct vp6_fw_rsc_vdev *)((u8 *)rsc_va + off + sizeof(u32));
		nv = min_t(unsigned int, vd->num_vrings, VP6_RPROC_MAX_VRINGS);
		if (off + sizeof(u32) + struct_size(vd, vring, nv) > rsc_sh->sh_size)
			goto einval;
		c->rsc_status = &vd->status;
		c->rsc_gfeatures = &vd->gfeatures;
		c->rproc_dfeatures = vd->dfeatures;
		for (v = 0; v < nv; v++) {
			c->rsc_vring_da[v] = &vd->vring[v].da;
			c->rsc_vring[v] = &vd->vring[v];
		}
		have_vdev = true;
		break;
	}
	if (!have_vdev)
		goto einval;		/* not an rpmsg remoteproc firmware */

	/*
	 * altresetvec = e_entry by default. But the firmware's on-core
	 * .dram0.data (p_paddr >= VP6_DSP_LOCAL_BASE, 0x7ff00000) is load-bearing
	 * at the reset vector, and the AP cannot write DSP-local memory directly
	 * (the direct-host L1 wall, Phase H). If we leave it unplaced the DSP
	 * wild-runs into a hard SoC reset the instant START releases it. Deliver
	 * it through the same proven base-ISA BOOTSTRAP L1 self-copy the cam_vpu
	 * VP6_LOAD_ELF path uses (the copy AND the jx handoff are clean on silicon
	 * -- DIAG5/DIAG6, Phase S): vp6_elf_stage_bootstrap stages the on-core
	 * segment(s) in DRAM, emits an unrolled DRAM->L1 copy, and repoints
	 * altresetvec (c->fw_dma) at the bootstrap, which jx's to e_entry
	 * (0x50000000) once L1 is populated. It leaves fw_dma = e_entry when the
	 * firmware has no on-core segment. Carveout/VDEV/vring setup stays intact.
	 */
	c->fw_dma = eh->e_entry;
	c->fw_entry = 0;
	ret = vp6_elf_stage_bootstrap(c, img, len, eh, ph, dom);
	if (ret)
		goto err;
	c->fw_size = len;
	c->rproc = true;
	strscpy(c->fw_ver, "xann0-rproc", VP6_FW_VER_LEN);
	c->state = VP6_STATE_LOADED;

	dev_info(c->dev,
		 "vp6-%u: staged rproc fw: %u carveout(s), entry 0x%x, trace %u B%s\n",
		 c->core_index, c->elf_ncluster, eh->e_entry, c->trace_len,
		 oncore ? " [on-core .dram0.data via BOOTSTRAP L1 self-copy]" : "");
	return 0;

einval:
	ret = -EINVAL;
err:
	vp6_unload_elf(c);
	return ret;
}

/*
 * Split-virtqueue (legacy vring) layout of a VDEV vring: num 16-byte descriptors,
 * then the avail ring (flags, idx, ring[num], used_event), then -- aligned to the
 * vring's align -- the used ring (flags, idx, {id, len}[num]).
 */
#define VP6_RPMSG_BUF_SIZE	512	/* OpenAMP RPMSG_BUFFER_SIZE, virtio_rpmsg_bus MAX_RPMSG_BUF_SIZE */
#define VP6_VRING_DESC_F_WRITE	2

struct vp6_vring_desc {
	__le64 addr;
	__le32 len;
	__le16 flags;
	__le16 next;
};

static u32 vp6_vring_used_off(u32 num, u32 align)
{
	return ALIGN(num * sizeof(struct vp6_vring_desc) + (3 + num) * sizeof(u16), align);
}

/*
 * The firmware is the OpenAMP remote: its send queue is vring0, and
 * rpmsg_virtio_get_tx_buffer takes buffers from that vring's AVAIL ring, which
 * only the host fills (as Linux virtio_rpmsg_bus fills its RX queue). With none
 * posted, rpmsg_create_ept's name-service announce has no buffer and the
 * firmware logs "Failed to create endpoint." Post one VP6_RPMSG_BUF_SIZE buffer
 * per vring0 entry from the first half of the vdev0buffer pool (the second half
 * is left for host->DSP messages on vring1). The firmware maps that pool with
 * pa == da, so a descriptor carries the buffer's device address.
 */
static void vp6_rproc_post_rx(struct vp6_core *c)
{
	struct vp6_fw_rsc_vdev_vring *vr = c->rsc_vring[0];
	struct vp6_vring_desc *desc = c->rproc_vring_va[0];
	u16 *avail;
	u32 num, i;

	if (!vr || !desc || !c->rproc_buf_va)
		return;
	num = min_t(u32, vr->num, c->rproc_buf_len / 2 / VP6_RPMSG_BUF_SIZE);
	if (!num || vp6_vring_used_off(vr->num, vr->align ? vr->align : 4096) +
		    (2 + 2 * vr->num) * sizeof(u32) > c->rproc_vring_len[0])
		return;
	avail = (u16 *)&desc[vr->num];
	for (i = 0; i < num; i++) {
		desc[i].addr = cpu_to_le64(c->rproc_buf_da + i * VP6_RPMSG_BUF_SIZE);
		desc[i].len = cpu_to_le32(VP6_RPMSG_BUF_SIZE);
		desc[i].flags = cpu_to_le16(VP6_VRING_DESC_F_WRITE);
		desc[i].next = 0;
		avail[2 + i] = cpu_to_le16(i);
	}
	c->rproc_used_seen = 0;
	c->rproc_rx_num = num;
	c->rproc_rx_used = 0;
	c->rproc_rx_avail = num;
	c->rproc_tx_avail = 0;
	avail[0] = 0;					/* flags: interrupts wanted */
	wmb();
	avail[1] = cpu_to_le16(num);			/* idx */
	wmb();
	dev_info(c->dev, "vp6-%u: rproc vring0: posted %u x %u B rx buffers @ da 0x%08x (num %u align %u)\n",
		 c->core_index, num, VP6_RPMSG_BUF_SIZE, c->rproc_buf_da, vr->num, vr->align);
}

/*
 * Diagnostic: report what the firmware returned on vring0 (its sends to the
 * host) -- the used index and the first used buffer's rpmsg header plus the
 * first 32 payload bytes (a name-service announce carries the endpoint name).
 */
static void vp6_rproc_report_used(struct vp6_core *c)
{
	struct vp6_fw_rsc_vdev_vring *vr = c->rsc_vring[0];
	u8 *ring = c->rproc_vring_va[0];
	u32 off, id, len;
	u16 idx;
	const u8 *b;

	if (!vr || !ring || !c->rproc_buf_va)
		return;
	off = vp6_vring_used_off(vr->num, vr->align ? vr->align : 4096);
	idx = le16_to_cpu(*(__le16 *)(ring + off + 2));
	if (idx == c->rproc_used_seen)
		return;
	c->rproc_used_seen = idx;
	id = le32_to_cpu(*(__le32 *)(ring + off + 4));
	len = le32_to_cpu(*(__le32 *)(ring + off + 8));
	if (id * VP6_RPMSG_BUF_SIZE + 48 > c->rproc_buf_len)
		return;
	b = (const u8 *)c->rproc_buf_va + id * VP6_RPMSG_BUF_SIZE;
	dev_info(c->dev,
		 "vp6-%u: rproc vring0 used idx %u: id %u len %u src 0x%x dst 0x%x plen %u payload '%.32s'\n",
		 c->core_index, idx, id, len, get_unaligned_le32(b),
		 get_unaligned_le32(b + 4), get_unaligned_le16(b + 12), b + 16);
}

#define VP6_RPMSG_HDR_SIZE	16	/* struct rpmsg_hdr {src, dst, reserved, u16 len, u16 flags} */

/*
 * RPMSG_SEND: host -> firmware. The firmware's receive queue is vring1 (OpenAMP
 * remote rvq); it takes each message from that vring's avail ring and returns
 * it on the used ring. Host TX buffers are the second half of the vdev0buffer
 * pool, used round-robin by avail index (one per vring1 entry), so a slot is
 * reused only after vring1's num further sends.
 */
static int vp6_rpmsg_send(struct vp6_core *c, const struct vp6_rpmsg *m)
{
	struct vp6_fw_rsc_vdev_vring *vr = c->rsc_vring[1];
	struct vp6_vring_desc *desc = c->rproc_vring_va[1];
	u32 num, slot, da, half;
	u16 *avail;
	u8 *b;

	if (!c->rproc || !vr || !desc || !c->rproc_buf_va || !c->rproc_rx_num)
		return -ENODEV;
	if (m->len > VP6_RPMSG_PAYLOAD_MAX)
		return -EMSGSIZE;
	half = c->rproc_buf_len / 2;
	num = min_t(u32, vr->num, half / VP6_RPMSG_BUF_SIZE);
	if (!num)
		return -ENODEV;
	slot = c->rproc_tx_avail % num;
	da = c->rproc_buf_da + half + slot * VP6_RPMSG_BUF_SIZE;
	b = (u8 *)c->rproc_buf_va + half + slot * VP6_RPMSG_BUF_SIZE;
	put_unaligned_le32(m->src, b);
	put_unaligned_le32(m->dst, b + 4);
	put_unaligned_le32(0, b + 8);
	put_unaligned_le16(m->len, b + 12);
	put_unaligned_le16(0, b + 14);
	if (m->len && copy_from_user(b + VP6_RPMSG_HDR_SIZE, u64_to_user_ptr(m->buf), m->len))
		return -EFAULT;
	desc[slot].addr = cpu_to_le64(da);
	desc[slot].len = cpu_to_le32(VP6_RPMSG_HDR_SIZE + m->len);
	desc[slot].flags = 0;
	desc[slot].next = 0;
	avail = (u16 *)&desc[vr->num];
	avail[2 + c->rproc_tx_avail % vr->num] = cpu_to_le16(slot);
	wmb();
	c->rproc_tx_avail++;
	avail[1] = cpu_to_le16(c->rproc_tx_avail);
	wmb();
	vp6_clr(c, VP6_CTL_XTENSA_INT, 1);		/* kick the DSP: a fresh 0 -> 1 */
	vp6_set(c, VP6_CTL_XTENSA_INT, 1);
	return 0;
}

/*
 * RPMSG_RECV: firmware -> host. Take the next entry of vring0's used ring past
 * the last one consumed, copy its payload out and re-post the buffer on the
 * avail ring.
 */
static int vp6_rpmsg_recv(struct vp6_core *c, struct vp6_rpmsg *m)
{
	struct vp6_fw_rsc_vdev_vring *vr = c->rsc_vring[0];
	struct vp6_vring_desc *desc = c->rproc_vring_va[0];
	u8 *ring = c->rproc_vring_va[0];
	u32 off, id, plen, n;
	u16 *avail;
	u8 *b;

	if (!c->rproc || !vr || !desc || !c->rproc_buf_va || !c->rproc_rx_num)
		return -ENODEV;
	off = vp6_vring_used_off(vr->num, vr->align ? vr->align : 4096);
	rmb();
	if (le16_to_cpu(*(__le16 *)(ring + off + 2)) == c->rproc_rx_used)
		return -EAGAIN;
	id = le32_to_cpu(*(__le32 *)(ring + off + 4 + 8 * (c->rproc_rx_used % vr->num)));
	if (id >= c->rproc_rx_num)
		return -EIO;
	b = (u8 *)c->rproc_buf_va + id * VP6_RPMSG_BUF_SIZE;
	plen = min_t(u32, get_unaligned_le16(b + 12), VP6_RPMSG_PAYLOAD_MAX);
	n = min(plen, m->len);
	if (n && copy_to_user(u64_to_user_ptr(m->buf), b + VP6_RPMSG_HDR_SIZE, n))
		return -EFAULT;
	m->src = get_unaligned_le32(b);
	m->dst = get_unaligned_le32(b + 4);
	m->len = plen;
	c->rproc_rx_used++;
	/* re-post the buffer */
	avail = (u16 *)&desc[vr->num];
	avail[2 + c->rproc_rx_avail % vr->num] = cpu_to_le16(id);
	wmb();
	c->rproc_rx_avail++;
	avail[1] = cpu_to_le16(c->rproc_rx_avail);
	wmb();
	return 0;
}

/*
 * Bring the loaded rproc firmware's virtio-rpmsg peer up: init the vrings, drive
 * the virtio status to DRIVER_OK (the firmware spins in
 * rpmsg_virtio_wait_remote_ready on this byte), and kick the doorbell. The
 * resource table + vrings are cache-BYPASS on the DSP side (rsc_table.c
 * resource_table_cache_init), so a wmb() to the coherent mapping suffices.
 */
static int vp6_rproc_ready(struct vp6_core *c)
{
	unsigned int v;

	if (!c->rproc || !c->rsc_status)
		return -ENODEV;
	if (c->state != VP6_STATE_RUNNING)
		return -ENODEV;

	/* Clean vring state + publish the vring device addresses. */
	for (v = 0; v < VP6_RPROC_MAX_VRINGS; v++) {
		if (c->rproc_vring_va[v] && c->rproc_vring_len[v])
			memset(c->rproc_vring_va[v], 0, c->rproc_vring_len[v]);
		if (c->rsc_vring_da[v])
			*c->rsc_vring_da[v] = c->rproc_vring_da[v];
	}

	vp6_rproc_post_rx(c);

	/* virtio handshake: ACK -> DRIVER -> (features) -> FEATURES_OK -> DRIVER_OK */
	*c->rsc_status = VP6_VIRTIO_ACK;
	wmb();
	*c->rsc_status = VP6_VIRTIO_ACK | VP6_VIRTIO_DRIVER;
	wmb();
	if (c->rsc_gfeatures)
		*c->rsc_gfeatures = c->rproc_dfeatures;	/* accept all offered (F_NS) */
	wmb();
	*c->rsc_status = VP6_VIRTIO_ACK | VP6_VIRTIO_DRIVER | VP6_VIRTIO_FEATURES_OK;
	wmb();
	*c->rsc_status = VP6_VIRTIO_ACK | VP6_VIRTIO_DRIVER | VP6_VIRTIO_FEATURES_OK |
			 VP6_VIRTIO_DRIVER_OK;
	wmb();

	/* Kick the DSP so it re-checks wait_remote_ready (it also polls). */
	vp6_set(c, VP6_CTL_XTENSA_INT, 1);

	dev_info(c->dev, "vp6-%u: rproc virtio DRIVER_OK (status 0x%02x), kicked\n",
		 c->core_index, *c->rsc_status);
	return 0;
}

/* ---- shared buffers ------------------------------------------------------ */

static struct vp6_buf *vp6_buf_find(struct vp6_core *c, u32 handle)
{
	struct vp6_buf *b;

	mutex_lock(&c->buf_lock);
	b = idr_find(&c->buf_idr, handle);
	mutex_unlock(&c->buf_lock);
	return b;
}

static int vp6_buf_alloc(struct vp6_core *c, struct vp6_buf_alloc *a)
{
	struct vp6_buf *b;
	size_t size = PAGE_ALIGN(a->size);
	int id;

	if (!size || size > SZ_256M)
		return -EINVAL;

	b = kzalloc(sizeof(*b), GFP_KERNEL);
	if (!b)
		return -ENOMEM;

	b->va = dma_alloc_coherent(c->dev, size, &b->dma, GFP_KERNEL);
	if (!b->va) {
		kfree(b);
		return -ENOMEM;
	}
	b->size = size;

	mutex_lock(&c->buf_lock);
	id = idr_alloc(&c->buf_idr, b, 1, 0, GFP_KERNEL);
	mutex_unlock(&c->buf_lock);
	if (id < 0) {
		dma_free_coherent(c->dev, size, b->va, b->dma);
		kfree(b);
		return id;
	}
	b->handle = id;

	a->handle = id;
	a->dma_addr = b->dma;
	a->mmap_offset = (u64)id << PAGE_SHIFT;
	return 0;
}

static void vp6_buf_free(struct vp6_core *c, struct vp6_buf *b)
{
	mutex_lock(&c->buf_lock);
	idr_remove(&c->buf_idr, b->handle);
	mutex_unlock(&c->buf_lock);
	dma_free_coherent(c->dev, b->size, b->va, b->dma);
	kfree(b);
}

/* ---- file ops ------------------------------------------------------------ */

static int vp6_power_up(struct vp6_core *c)
{
	int ret;

	ret = pm_runtime_resume_and_get(c->dev);
	if (ret)
		return ret;
	ret = clk_bulk_prepare_enable(c->num_clks, c->clks);
	if (ret) {
		pm_runtime_put(c->dev);
		return ret;
	}
	c->state = VP6_STATE_IDLE;
	return 0;
}

static void vp6_power_down(struct vp6_core *c)
{
	clk_bulk_disable_unprepare(c->num_clks, c->clks);
	pm_runtime_put(c->dev);
	c->state = VP6_STATE_OFF;
}

static int vp6_open(struct inode *ino, struct file *f)
{
	struct vp6_core *c = container_of(f->private_data, struct vp6_core, mdev);
	int ret;

	mutex_lock(&c->lock);
	if (c->busy) {
		mutex_unlock(&c->lock);
		return -EBUSY;
	}
	ret = vp6_power_up(c);
	if (ret) {
		mutex_unlock(&c->lock);
		return ret;
	}
	c->busy = true;
	c->rx_head = c->rx_tail = 0;
	f->private_data = c;
	mutex_unlock(&c->lock);
	return 0;
}

static int vp6_release(struct inode *ino, struct file *f)
{
	struct vp6_core *c = f->private_data;
	struct vp6_buf *b;
	int id;

	mutex_lock(&c->lock);
	if (c->state == VP6_STATE_RUNNING)
		vp6_hw_halt(c);

	mutex_lock(&c->buf_lock);
	idr_for_each_entry(&c->buf_idr, b, id) {
		idr_remove(&c->buf_idr, b->handle);
		dma_free_coherent(c->dev, b->size, b->va, b->dma);
		kfree(b);
	}
	mutex_unlock(&c->buf_lock);

	vp6_free_fw(c);
	vp6_power_down(c);
	c->busy = false;
	mutex_unlock(&c->lock);
	return 0;
}

static long vp6_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	struct vp6_core *c = f->private_data;
	void __user *uarg = (void __user *)arg;
	int ret = 0;

	mutex_lock(&c->lock);
	switch (cmd) {
	case VP6_IOC_GET_INFO: {
		struct vp6_info info = {
			.core_index = c->core_index,
			.state = c->state,
			.iram_size = VP6_IRAM_SIZE,
			.dram_size = VP6_DRAM_SIZE,
			.abi_version = VP6_FW_ABI_VER,
			.caps = c->caps,
		};
		memcpy(info.fw_ver, c->fw_ver, VP6_FW_VER_LEN);
		if (copy_to_user(uarg, &info, sizeof(info)))
			ret = -EFAULT;
		break;
	}
	case VP6_IOC_LOAD_FW: {
		struct vp6_fw_load fw;
		void *img;

		if (copy_from_user(&fw, uarg, sizeof(fw))) {
			ret = -EFAULT;
			break;
		}
		if (!fw.size || fw.size > SZ_16M + sizeof(struct vp6_fw_header)) {
			ret = -EINVAL;
			break;
		}
		img = vmemdup_user(u64_to_user_ptr(fw.image), fw.size);
		if (IS_ERR(img)) {
			ret = PTR_ERR(img);
			break;
		}
		ret = vp6_load_image(c, img, fw.size);
		kvfree(img);
		break;
	}
	case VP6_IOC_LOAD_FW_NAME: {
		char name[64];
		const struct firmware *fw;

		if (copy_from_user(name, uarg, sizeof(name))) {
			ret = -EFAULT;
			break;
		}
		name[sizeof(name) - 1] = '\0';
		ret = request_firmware(&fw, name, c->dev);
		if (ret)
			break;
		ret = vp6_load_image(c, fw->data, fw->size);
		release_firmware(fw);
		break;
	}
	case VP6_IOC_LOAD_ELF: {
		struct vp6_fw_load fw;
		void *img;

		if (!(c->caps & VP6_CAP_LOAD_ELF)) {
			ret = -EOPNOTSUPP;
			break;
		}
		if (copy_from_user(&fw, uarg, sizeof(fw))) {
			ret = -EFAULT;
			break;
		}
		if (!fw.size || fw.size > VP6_ELF_MAX_IMAGE) {
			ret = -EINVAL;
			break;
		}
		img = vmemdup_user(u64_to_user_ptr(fw.image), fw.size);
		if (IS_ERR(img)) {
			ret = PTR_ERR(img);
			break;
		}
		if ((fw.flags & VP6_LOAD_ELF_RPROC) &&
		    !(fw.flags & VP6_LOAD_ELF_DIAG_MASK))
			ret = vp6_load_rproc(c, img, fw.size);
		else
			ret = vp6_load_elf(c, img, fw.size, fw.flags);
		kvfree(img);
		break;
	}
	case VP6_IOC_RPROC_READY:
		ret = vp6_rproc_ready(c);
		break;
	case VP6_IOC_RPMSG_SEND: {
		struct vp6_rpmsg m;

		if (copy_from_user(&m, uarg, sizeof(m))) {
			ret = -EFAULT;
			break;
		}
		ret = vp6_rpmsg_send(c, &m);
		break;
	}
	case VP6_IOC_RPMSG_RECV: {
		struct vp6_rpmsg m;

		if (copy_from_user(&m, uarg, sizeof(m))) {
			ret = -EFAULT;
			break;
		}
		ret = vp6_rpmsg_recv(c, &m);
		if (!ret && copy_to_user(uarg, &m, sizeof(m)))
			ret = -EFAULT;
		break;
	}
	case VP6_IOC_READ_TRACE: {
		struct vp6_trace t;
		u32 n;

		if (copy_from_user(&t, uarg, sizeof(t))) {
			ret = -EFAULT;
			break;
		}
		if (!c->trace_va || !c->trace_len) {
			ret = -ENODEV;
			break;
		}
		rmb();				/* DSP wrote the ring (coherent) */
		if (c->rproc)
			vp6_rproc_report_used(c);
		t.len = c->trace_len;
		n = min(t.size, c->trace_len);
		if (n && copy_to_user(u64_to_user_ptr(t.buf), c->trace_va, n)) {
			ret = -EFAULT;
			break;
		}
		t.copied = n;
		t.reserved = 0;
		if (copy_to_user(uarg, &t, sizeof(t)))
			ret = -EFAULT;
		break;
	}
	case VP6_IOC_READ_DIAG: {
		struct vp6_diag dg = {};

		if (c->diag_scratch_va) {
			const u32 *s = c->diag_scratch_va;

			dg.scratch0 = s[0];
			dg.scratch1 = s[1];
		}
		if (c->diag_staging_va)
			dg.staging0 = *(const u32 *)c->diag_staging_va;
		dev_info(c->dev,
			 "vp6-%u: DIAG readback scratch0=0x%08x scratch1=0x%08x staging0=0x%08x\n",
			 c->core_index, dg.scratch0, dg.scratch1, dg.staging0);
		if (copy_to_user(uarg, &dg, sizeof(dg)))
			ret = -EFAULT;
		break;
	}
	case VP6_IOC_READ_DIAG2: {
		struct vp6_diag2 dg = {};

		if (c->diag_scratch_va)
			memcpy(dg.words, c->diag_scratch_va, sizeof(dg.words));
		dev_info(c->dev,
			 "vp6-%u: DIAG2 readback w0..w10 = %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x\n",
			 c->core_index, dg.words[0], dg.words[1], dg.words[2],
			 dg.words[3], dg.words[4], dg.words[5], dg.words[6],
			 dg.words[7], dg.words[8], dg.words[9], dg.words[10]);
		if (copy_to_user(uarg, &dg, sizeof(dg)))
			ret = -EFAULT;
		break;
	}
	case VP6_IOC_START:
		if (!(c->caps & VP6_CAP_CTRL)) {
			ret = -EOPNOTSUPP;
			break;
		}
		if (c->state != VP6_STATE_LOADED) {
			ret = -ENODEV;
			break;
		}
		vp6_hw_boot(c);
		c->state = VP6_STATE_RUNNING;
		break;
	case VP6_IOC_HALT:
		if (!(c->caps & VP6_CAP_CTRL)) {
			ret = -EOPNOTSUPP;
			break;
		}
		vp6_hw_halt(c);
		if (c->fw_va)
			c->state = VP6_STATE_LOADED;
		else
			c->state = VP6_STATE_IDLE;
		break;
	case VP6_IOC_RESET:
		if (!(c->caps & VP6_CAP_CTRL)) {
			ret = -EOPNOTSUPP;
			break;
		}
		vp6_hw_halt(c);
		c->state = c->fw_va ? VP6_STATE_LOADED : VP6_STATE_IDLE;
		break;
	case VP6_IOC_SEND_IPI: {
		struct vp6_ipi_xfer x;

		if (copy_from_user(&x, uarg, sizeof(x))) {
			ret = -EFAULT;
			break;
		}
		ret = vp6_ipi_send(c, &x.msg, x.timeout_ms);
		break;
	}
	case VP6_IOC_RECV_IPI: {
		struct vp6_ipi_xfer x = {};

		if (!vp6_rx_pop(c, &x.msg)) {
			ret = -EAGAIN;
			break;
		}
		if (copy_to_user(uarg, &x, sizeof(x)))
			ret = -EFAULT;
		break;
	}
	case VP6_IOC_ALLOC_BUF: {
		struct vp6_buf_alloc a;

		if (copy_from_user(&a, uarg, sizeof(a))) {
			ret = -EFAULT;
			break;
		}
		ret = vp6_buf_alloc(c, &a);
		if (!ret && copy_to_user(uarg, &a, sizeof(a)))
			ret = -EFAULT;
		break;
	}
	case VP6_IOC_FREE_BUF: {
		u32 handle;
		struct vp6_buf *b;

		if (copy_from_user(&handle, uarg, sizeof(handle))) {
			ret = -EFAULT;
			break;
		}
		b = vp6_buf_find(c, handle);
		if (!b) {
			ret = -ENOENT;
			break;
		}
		vp6_buf_free(c, b);
		break;
	}
	default:
		ret = -ENOTTY;
	}
	mutex_unlock(&c->lock);
	return ret;
}

static __poll_t vp6_poll(struct file *f, poll_table *wait)
{
	struct vp6_core *c = f->private_data;
	__poll_t mask = 0;

	poll_wait(f, &c->rx_wq, wait);
	spin_lock_irq(&c->rx_lock);
	if (c->rx_head != c->rx_tail)
		mask |= EPOLLIN | EPOLLRDNORM;
	spin_unlock_irq(&c->rx_lock);
	return mask;
}

static int vp6_mmap(struct file *f, struct vm_area_struct *vma)
{
	struct vp6_core *c = f->private_data;
	u32 handle = vma->vm_pgoff;
	size_t size = vma->vm_end - vma->vm_start;
	struct vp6_buf *b;

	b = vp6_buf_find(c, handle);
	if (!b)
		return -EINVAL;
	if (size > b->size)
		return -EINVAL;

	vma->vm_pgoff = 0;
	return dma_mmap_coherent(c->dev, vma, b->va, b->dma, size);
}

static const struct file_operations vp6_fops = {
	.owner		= THIS_MODULE,
	.open		= vp6_open,
	.release	= vp6_release,
	.unlocked_ioctl	= vp6_ioctl,
	.compat_ioctl	= compat_ptr_ioctl,
	.poll		= vp6_poll,
	.mmap		= vp6_mmap,
};

/* ---- probe --------------------------------------------------------------- */

/*
 * Two clock groups, enabled at DIFFERENT lifetimes:
 *
 *  - base (per-core ipu_core0/1: axi/ipu/jtag): always required, enabled for
 *    every open in vp6_power_up() -- both the plain cam_vpu/GEMM path and rproc.
 *
 *  - fabric (shared ipu_conn: APB reg-port bridge APB2AXI/APB2AHB + DAP_RX +
 *    the core<->fabric crossbar CAB*): needed ONLY by the vendor RTOS (rproc).
 *    The RTOS's early mt8183_irq_init reaches the cfg block (0x19180000) via
 *    these gates; the VPU_TOP genpd leaves them unreferenced (wires only
 *    ipu_conn 0-5) so they die at clk_disable_unused and the first cfg access
 *    back-pressures the fabric -> TOPRGU watchdog -> SoC warm reset. Refcounting
 *    them on before the rproc START (vp6_hw_boot, gated on c->rproc; shared, so
 *    refcounting is fine) closes that gap.
 *
 * They are split because enabling the fabric group on the PLAIN (non-rproc)
 * LOAD_ELF+START path is itself what wedges the fabric -> TOPRGU reset (the
 * regression from 6a0fee809f2b, which enabled all 12 unconditionally in
 * vp6_power_up). Names match the vp6 DT clock-names.
 */
/* mt8183 (i500) base clocks: per-core axi/ipu/jtag (see vp6_mt8183_data). */
static const char * const vp6_mt8183_clk_names[] = {
	"axi", "ipu", "jtag",
};
/* mt8365 (g350) base clocks: the six apu-syscon gates (clk-mt8365-apu):
 * ipu(IPU_CK)/axi(AXI)/jtag(JTAG)/ahb(AHB)/if(IF_CK)/edma(EDMA the iDMA).
 * Names match the &vp6 node's clock-names in mt8365.dtsi. */
static const char * const vp6_mt8365_clk_names[] = {
	"ipu", "axi", "jtag", "ahb", "if", "edma",
};
/* mt8183 shared ipu_conn fabric clocks (rproc START path only). */
static const char * const vp6_fabric_clk_names[] = {
	"dap_rx", "apb2axi", "apb2ahb",
	"ipu_cab1to2", "ipu1_cab1to2", "ipu2_cab1to2",
	"cab3to3", "cab2to1", "cab3to1_slice",
};

static const struct vp6_soc_data vp6_mt8183_data = {
	.core_syscon = "mediatek,ipu-core",
	.has_fabric  = true,
	.clk_names   = vp6_mt8183_clk_names,
	.num_clks    = ARRAY_SIZE(vp6_mt8183_clk_names),
};
static const struct vp6_soc_data vp6_mt8365_data = {
	.core_syscon = "mediatek,apu-core",
	.has_fabric  = false,
	.clk_names   = vp6_mt8365_clk_names,
	.num_clks    = ARRAY_SIZE(vp6_mt8365_clk_names),
};

static_assert(ARRAY_SIZE(vp6_mt8183_clk_names) <= VP6_MAX_CLKS);
static_assert(ARRAY_SIZE(vp6_mt8365_clk_names) <= VP6_MAX_CLKS);
static_assert(ARRAY_SIZE(vp6_fabric_clk_names) ==
	      ARRAY_SIZE(((struct vp6_core *)0)->fabric_clks));

static int vp6_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct vp6_core *c;
	int ret, i;

	c = devm_kzalloc(dev, sizeof(*c), GFP_KERNEL);
	if (!c)
		return -ENOMEM;
	c->dev = dev;
	mutex_init(&c->lock);
	mutex_init(&c->buf_lock);
	spin_lock_init(&c->rx_lock);
	init_waitqueue_head(&c->rx_wq);
	idr_init(&c->buf_idr);

	c->soc = of_device_get_match_data(dev);
	if (!c->soc)
		return dev_err_probe(dev, -EINVAL, "missing SoC match data\n");

	if (of_property_read_u32(dev->of_node, "mediatek,core-index",
				 &c->core_index))
		c->core_index = 0;

	/*
	 * cfg = the core's own cfg-block syscon. The phandle property name is
	 * per-SoC (mediatek,ipu-core on mt8183, mediatek,apu-core on mt8365) but
	 * the underlying 0x19180000 register block + offset map is shared IP.
	 */
	c->cfg = syscon_regmap_lookup_by_phandle(dev->of_node,
						 c->soc->core_syscon);
	if (IS_ERR(c->cfg))
		return dev_err_probe(dev, PTR_ERR(c->cfg), "%s syscon\n",
				     c->soc->core_syscon);

	/* Shared ipu_conn fabric syscon: mt8183 only. mt8365 has no fabric. */
	if (c->soc->has_fabric) {
		c->conn = syscon_regmap_lookup_by_phandle(dev->of_node,
							  "mediatek,ipu-conn");
		if (IS_ERR(c->conn))
			return dev_err_probe(dev, PTR_ERR(c->conn),
					     "ipu-conn syscon\n");
	}

	c->num_clks = c->soc->num_clks;
	for (i = 0; i < c->num_clks; i++)
		c->clks[i].id = c->soc->clk_names[i];
	ret = devm_clk_bulk_get(dev, c->num_clks, c->clks);
	if (ret)
		return dev_err_probe(dev, ret, "get clocks\n");

	/* Shared ipu_conn fabric clocks (mt8183 only): enabled only for the rproc
	 * START path (vp6_hw_boot), NOT in vp6_power_up -- enabling them on the
	 * plain LOAD_ELF+START path wedges the fabric -> TOPRGU reset
	 * (6a0fee809f2b). Absent on mt8365 (num_fabric_clks stays 0). */
	if (c->soc->has_fabric) {
		c->num_fabric_clks = ARRAY_SIZE(vp6_fabric_clk_names);
		for (i = 0; i < c->num_fabric_clks; i++)
			c->fabric_clks[i].id = vp6_fabric_clk_names[i];
		ret = devm_clk_bulk_get(dev, c->num_fabric_clks, c->fabric_clks);
		if (ret)
			return dev_err_probe(dev, ret, "get fabric clocks\n");
	}

	/* Optional: the DSP<->AP GALS bridge (infracfg_ao IPU_GALS) + the toprgu
	 * IPU0 subsystem reset. Deasserted before the rproc START (vp6_hw_boot) --
	 * the RE lever for the tie2apb fabric-hang. Optional so a DT without them
	 * still probes (deassert is then a no-op). */
	c->rst_gals = devm_reset_control_get_optional_exclusive(dev, "ipu_gals");
	if (IS_ERR(c->rst_gals))
		return dev_err_probe(dev, PTR_ERR(c->rst_gals), "ipu_gals reset\n");
	c->rst_ipu = devm_reset_control_get_optional_exclusive(dev, "ipu0");
	if (IS_ERR(c->rst_ipu))
		return dev_err_probe(dev, PTR_ERR(c->rst_ipu), "ipu0 reset\n");

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(dev, ret, "dma mask\n");

	/*
	 * Threaded IRQ: the mailbox ISR reads/acks via the mutex-locked syscon
	 * regmap, so it must run in thread context.
	 *
	 * Requested with IRQF_NO_AUTOEN -- left MASKED at registration. The
	 * VP6's level-high core->host line (xtensa_int, 0x114) reads asserted
	 * (0xffffffff) while the VPU_TOP power domain is off and before any
	 * firmware clears it, so an auto-enabled level IRQ storms the GIC the
	 * instant the module loads (threaded handler re-fires forever ->
	 * mtk-wdt reset). We unmask it in vp6_hw_boot (core powered + running,
	 * doorbell cleared) and re-mask it in vp6_hw_halt. The IRQ still appears
	 * in /proc/interrupts at probe; its count stays 0 until the core runs.
	 */
	c->irq = platform_get_irq_optional(pdev, 0);
	if (c->irq > 0) {
		ret = devm_request_threaded_irq(dev, c->irq, NULL, vp6_isr,
						IRQF_ONESHOT | IRQF_NO_AUTOEN,
						dev_name(dev), c);
		if (ret)
			return dev_err_probe(dev, ret, "request irq\n");
	}

	/*
	 * Capabilities. LOAD (DDR staging) + CTRL (reset/boot) + MAILBOX
	 * (doorbell + info regs) are all wired from the mt67xx-template control
	 * map; the mailbox additionally needs the core->host IRQ. Real execution
	 * is still firmware-gated (no MT8183 VP6 image exists) and the exact
	 * offsets await on-silicon verification (see the board test plan).
	 */
	c->caps = VP6_CAP_LOAD | VP6_CAP_CTRL;
	if (c->irq > 0)
		c->caps |= VP6_CAP_MAILBOX;
	/*
	 * The fixed-MVA ELF loader needs the M4U: with the vp6 node's `iommus`
	 * the DMA API places segments at absolute IOVAs the DSP fetches through
	 * the "AXI Request via M4U" path.
	 */
	if (device_iommu_mapped(dev))
		c->caps |= VP6_CAP_LOAD_ELF;

	pm_runtime_enable(dev);

	c->mdev.minor = MISC_DYNAMIC_MINOR;
	c->mdev.fops = &vp6_fops;
	c->mdev.name = devm_kasprintf(dev, GFP_KERNEL, "vp6-%u", c->core_index);
	if (!c->mdev.name) {
		ret = -ENOMEM;
		goto err_pm;
	}
	ret = misc_register(&c->mdev);
	if (ret) {
		dev_err_probe(dev, ret, "misc_register\n");
		goto err_pm;
	}

	platform_set_drvdata(pdev, c);
	dev_info(dev, "%s VP6 core%u ready as /dev/%s (caps 0x%x, irq %d)\n",
		 c->soc->has_fabric ? "MT8183" : "MT8365",
		 c->core_index, c->mdev.name, c->caps, c->irq);
	return 0;

err_pm:
	pm_runtime_disable(dev);
	return ret;
}

static void vp6_remove(struct platform_device *pdev)
{
	struct vp6_core *c = platform_get_drvdata(pdev);

	misc_deregister(&c->mdev);
	pm_runtime_disable(c->dev);
	idr_destroy(&c->buf_idr);
}

static const struct of_device_id vp6_match[] = {
	{ .compatible = "mediatek,mt8183-vp6", .data = &vp6_mt8183_data },
	{ .compatible = "mediatek,mt8365-vp6", .data = &vp6_mt8365_data },
	{}
};
MODULE_DEVICE_TABLE(of, vp6_match);

static struct platform_driver vp6_driver = {
	.probe = vp6_probe,
	.remove = vp6_remove,
	.driver = {
		.name = "mtk-vp6",
		.of_match_table = vp6_match,
	},
};
module_platform_driver(vp6_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MediaTek MT8183/MT8365 VP6 (IPU/APU / Cadence Vision-P6) driver");
