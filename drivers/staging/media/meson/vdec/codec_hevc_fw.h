/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * HEVC firmware known-versions table -- declarations.
 */

#ifndef __CODEC_HEVC_FW_H_
#define __CODEC_HEVC_FW_H_

#include "vdec.h"

/* Quirk bits for struct amvdec_fw_version.quirks in HEVC firmware table.
 *
 * Two classes of bits:
 *   PROTO_* : selects driver-firmware interaction protocol.  Drivers
 *             use these to dispatch into the matching code-path bucket
 *             (see hevc/task004-architecture-plan-20260420.md).
 *             Exactly ONE PROTO_* bit must be set per table entry.
 *   Other bits : informational / per-variant workaround flags.
 *
 * Bit 0: BSP swap-protocol crash on G12A+.  The 36KB BSP HEVC firmware
 *        uses a swap protocol that crashed the upstream G12A+ driver
 *        in TASK-003 (2026-04-10).  Informational -- recognized so the
 *        driver does not refuse-to-load if a user switches to BSP
 *        firmware, but BSP_PROTO support is a separate driver-port
 *        task (TASK-004 architecture plan Phase A.2-A.5).
 */
#define AMVDEC_FW_QUIRK_HEVC_BSP_SWAP_CRASH_G12A  BIT(0)
/* Bit 1: BSP 36KB variant (informational; ships full dispatch path).
 */
#define AMVDEC_FW_QUIRK_HEVC_BSP_36K  BIT(1)
/* Bit 2 (PROTO): upstream 16KB linux-firmware variant.  Driver runs
 * the minimal protocol path: direct LMEM read for slice params,
 * reduced MPRED register set, no RPM buffer, no canvas-indexed
 * reference setup, no firmware-poke watchdog.  Some upstream variants
 * are functionally incomplete (G12A+ 16KB strips the LMEM[0xf9]
 * dispatch handler, causing TASK-004 wedge after N POCs).
 */
#define AMVDEC_FW_QUIRK_HEVC_UPSTREAM_PROTO  BIT(2)
/* Bit 3 (PROTO): BSP-derived variant (36KB or 40KB).  Driver runs the
 * BSP protocol path: per-slice RPM-buffer read for slice params, full
 * MPRED register suite, canvas-indexed reference indirection,
 * HEVC_WAIT_FLAG management, periodic firmware-poke watchdog.  See
 * TASK-004 architecture plan Path A buckets for the full divergence.
 */
#define AMVDEC_FW_QUIRK_HEVC_BSP_PROTO  BIT(3)

/* Number of entries in hevc_fw_versions[] */
#define HEVC_FW_VERSIONS_COUNT 6

extern const struct amvdec_fw_version hevc_fw_versions[];
extern const size_t hevc_fw_versions_count;

#endif
