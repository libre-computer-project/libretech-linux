/* SPDX-License-Identifier: GPL-2.0 */
/*
 * vdec_diag.h -- AMRISC diagnostic hot-reload interface
 *
 * Tier 1 of the stateless test bench plan: debugfs-based firmware
 * hot-upload, engine reset, register peek/poke, and scratch register
 * snapshot without a module reload.  Gated by the `diag=1` module
 * parameter on meson_vdec_core; the debugfs directory is not created
 * when diag is disabled.  Default: disabled (fleet boards never
 * accidentally expose the interface).
 *
 * Design and rationale:
 *   the internal stateless test-bench plan.
 */
#ifndef VDEC_DIAG_H
#define VDEC_DIAG_H

struct amvdec_core;

/* Module parameter gate -- defined in vdec_diag.c, referenced from vdec.c */
extern bool vdec_diag_enabled;

int vdec_diag_init(struct amvdec_core *core);
void vdec_diag_exit(struct amvdec_core *core);

#endif /* VDEC_DIAG_H */
