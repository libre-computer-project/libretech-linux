/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2018 Maxime Jourdan <maxi.jourdan@wanadoo.fr>
 */

#ifndef __MESON_VDEC_VDEC_HEVC_H_
#define __MESON_VDEC_VDEC_HEVC_H_

#include "vdec.h"

extern struct amvdec_ops vdec_hevc_ops;

int vdec_hevc_power_on(struct amvdec_core *core);
void vdec_hevc_power_off(struct amvdec_core *core);
void amvdec_hevc_halt_firmware(struct amvdec_core *core);
void amvdec_hevc_restart_firmware(struct amvdec_core *core);
void vdec_hevc_thermal_update(struct amvdec_core *core, unsigned long rate);
int vdec_hevc_clk_get(struct amvdec_core *core);
void vdec_hevc_clk_put(struct amvdec_core *core);
void vdec_hevc_power_init(struct amvdec_core *core);
int vdec_hevc_power_get(struct amvdec_core *core, bool hcodec_pin);
void vdec_hevc_power_put(struct amvdec_core *core);

/*
 * N95 sub-block control-register instrument.  Both callers must already have
 * the VDEC_HEVC block clocked and out of reset -- see the comment on the table
 * in vdec_hevc.c and N72 in the internal GXL boot status record.
 */
struct seq_file;
void amvdec_hevc_dump_subblock_regs(struct amvdec_core *core, const char *tag);
void amvdec_hevc_seq_subblock_regs(struct seq_file *s,
				   struct amvdec_core *core);
void amvdec_hevc_seq_progress_regs(struct seq_file *s,
				   struct amvdec_core *core);

#endif
