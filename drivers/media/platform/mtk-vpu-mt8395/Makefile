# SPDX-License-Identifier: GPL-2.0-only
#
# Vendor MT8188/MT8370/MT8195 (mt8395 generation) /dev/vcu provider.
# Imported from the mtk-vcu-driver repo, branch mtk-v6.6-mt8395 (SRCREV
# 6592c350). Builds the mtk-vcu module: the char-dev (/dev/vcu) the proprietary
# userspace (vpud) opens, plus its memory helper. The BSP self-prefixed MOST
# globals with a _mt8395 suffix (vcu_*_mt8395, the exported vcu_func_mt8395
# dispatch table), so this dir has 0 collisions with mainline
# drivers/media/platform/mediatek/{vpu,vcodec}. But it does NOT self-prefix the
# mem helpers mtk_vcu_get_buffer / mtk_vcu_free_buffer or the inline
# ipi_id_to_inst_id, which collide with the MT8365/Genio-350 VCU stack
# (drivers/media/platform/mtk-vpu/), so the shared coexistence rename header
# (force-included below) prefixes those with m8395_ here too. Block (B) of that
# header lists them; the consumer mtk-vcodec-mt8395 dir includes the same header.

mtk-vcu-mt8395-y := mtk_vcu.o mtk_vcodec_mem.o

obj-$(CONFIG_VIDEO_MEDIATEK_VCU_MT8395) += mtk-vcu-mt8395.o

ccflags-y := -Wno-enum-conversion
# mtk_vcu.h lives in this module's root dir, but mtk_vcu_camif.h (under
# external_include/uapi/linux/) #include "mtk_vcu.h" relative to itself -- so the
# module root must be on the search path or the build fails with
# "mtk_vcu_camif.h: fatal error: mtk_vcu.h: No such file or directory".
ccflags-y += -I$(src)
# mtk_vcu_controls.h / mtk_vcu_camif.h are pulled from the vendor uapi copy kept
# local to this dir (the global include/uapi/linux/mtk_vcu_controls.h is the
# DIFFERENT mt8365 version, owned by the mtk-vcodec-mt8167 port -- must not clash).
ccflags-y += -I$(src)/external_include/uapi
ccflags-y += -I$(src)/external_include/uapi/linux
# Coexistence: prefix the vendor globals that the BSP did NOT self-suffix
# (mem helpers + ipi_id_to_inst_id) so they don't clash with the MT8365 VCU
# stack (drivers/media/platform/mtk-vpu/) at vmlinux link. Shared with the
# consumer dir; the relative path reaches its single copy.
ccflags-y += -include $(src)/../mtk-vcodec-mt8395/mtk_vcodec_rename.h
