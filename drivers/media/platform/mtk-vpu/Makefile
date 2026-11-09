# SPDX-License-Identifier: GPL-2.0-only
# Module names carry the SoC token right after the driver-family base
# (mtk-vpu-mt8365 / mtk-vcu-mt8365), matching the sibling vendor port
# drivers/media/platform/mtk-vpu-mt8395/ (mtk-vcu-mt8395). Mainline
# drivers/media/platform/mediatek/vpu/ builds mtk-vpu.ko, so the un-suffixed
# name collides at modules_check as soon as CONFIG_MEDIA_SUPPORT=m makes both
# trees modular. mtk-vcu is renamed alongside it to keep the pair consistent.
mtk-vpu-mt8365-y += mtk_vpu.o
mtk-vcu-mt8365-y += mtk_vcu.o mtk_vcodec_mem.o

ccflags-y := -Wno-enum-conversion
# Coexistence: prefix vendor globals so they don't collide with mainline mtk-vcodec
ccflags-y += -include $(srctree)/drivers/media/platform/mtk-vcodec-mt8167/mtk_vcodec_rename.h

obj-$(CONFIG_VIDEO_MEDIATEK_VCU_MT8365) += mtk-vpu-mt8365.o
obj-$(CONFIG_VIDEO_MEDIATEK_VCU_MT8365) += mtk-vcu-mt8365.o
