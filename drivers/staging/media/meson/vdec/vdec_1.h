/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2018 BayLibre, SAS
 * Author: Maxime Jourdan <mjourdan@baylibre.com>
 */

#ifndef __MESON_VDEC_VDEC_1_H_
#define __MESON_VDEC_VDEC_1_H_

#include "vdec.h"

extern struct amvdec_ops vdec_1_ops;
void vdec_1_thermal_update(struct amvdec_core *core, unsigned long rate);
void vdec_1_power_init(struct amvdec_core *core);
int vdec_1_power_get(struct amvdec_core *core, bool hcodec_pin);
void vdec_1_power_put(struct amvdec_core *core);
void vdec_1_poweroff(struct amvdec_core *core);
void amvdec_vdec1_halt_firmware(struct amvdec_core *core);
void amvdec_vdec1_restart_firmware(struct amvdec_core *core);

#endif
