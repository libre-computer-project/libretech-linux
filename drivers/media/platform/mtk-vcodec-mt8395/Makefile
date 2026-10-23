# SPDX-License-Identifier: GPL-2.0-only
#
# Vendor MT8188/MT8370/MT8195 (mt8395 generation) V4L2 m2m dec/enc driver.
# Imported from the mtk-vcodec-driver repo, branch mtk-v6.6-mt8395 (SRCREV
# 7e7fa9a3). Talks to the codec engine through the mtk-vcu-mt8395 /dev/vcu
# provider (CONFIG_VIDEO_MEDIATEK_VCU_MT8395) via the exported vcu_func_mt8395
# dispatch table.
#
# Object selection mirrors the out-of-tree Makefile for TARGET_PLATFORM=mt8395:
#   MTK_PLATFORM_VER = v1  -> build the v1/ DVFS/BW platform variant. The OOT
#     Makefile picks v1 for mt8395 ("support multiple platform with v1
#     architecture while DVFS & EMI_BW are disabled"); DEC_DVFS/DEC_EMI_BW/
#     ENC_DVFS/ENC_EMI_BW are all 0 in the *_pm_plat.h headers, so the
#     interconnect / pm_opp paths are compiled out and v1/vcodec_bw.o is NOT
#     built (matches the OOT obj list -- only *_pm_plat.o + vcodec_dvfs.o).
#   VCU transport (vdec/vdec_vcu_if.o + venc/venc_vcu_if.o + the *_common_if.o)
#     is selected because CONFIG_VIDEO_MEDIATEK_VCU_MT8395 is set. The VCP
#     transport objects (vdec/vdec_vcp_if.o, venc/venc_vcp_if.o) are NOT built:
#     they need the Tinysys VCP headers (CONFIG_MTK_TINYSYS_VCP_SUPPORT) we do
#     not provide; vdec_drv_if.c / venc_drv_if.c fall through to the VCU path
#     (get_dec_vcu_if()/get_enc_vcu_if()) when VCP is unset.

obj-$(CONFIG_VIDEO_MEDIATEK_VCODEC_MT8395) += mtk-vcodec-mt8395-dec.o \
				       mtk-vcodec-mt8395-enc.o \
				       mtk-vcodec-mt8395-common.o

mtk-vcodec-mt8395-dec-y := v1/mtk_vcodec_dec_pm_plat.o \
		v1/vcodec_dvfs.o \
		mtk_vcodec_dec_drv.o \
		vdec_drv_if.o \
		mtk_vcodec_dec.o \
		mtk_vcodec_dec_pm.o \
		vdec/vdec_common_if.o \
		vdec/vdec_vcu_if.o \

mtk-vcodec-mt8395-enc-y := v1/mtk_vcodec_enc_pm_plat.o \
		v1/vcodec_dvfs.o \
		mtk_vcodec_enc_drv.o \
		venc_drv_if.o \
		mtk_vcodec_enc.o \
		mtk_vcodec_enc_pm.o \
		venc/venc_common_if.o \
		venc/venc_vcu_if.o \

mtk-vcodec-mt8395-common-y := mtk_vcodec_intr.o \
		mtk_vcodec_util.o \

ccflags-y += -I$(src)
ccflags-y += -I$(src)/external_include/mtk-vcu-mt8395
ccflags-y += -I$(src)/external_include/iommu
# mtk_vcu_controls.h: pull the vendor mt8395 uapi copy kept local to this dir,
# NOT the global include/uapi/linux/mtk_vcu_controls.h (that is the DIFFERENT
# mt8365 version owned by the mtk-vcodec-mt8167 port). ccflags -I is prepended
# before LINUXINCLUDE, so <linux/mtk_vcu_controls.h> resolves here first.
ccflags-y += -I$(src)/external_include/uapi
ccflags-y += -I$(src)/external_include/uapi/linux
# Coexistence: prefix the colliding vendor globals so they don't clash with
# mainline drivers/media/platform/mediatek/vcodec at vmlinux link.
ccflags-y += -include $(src)/mtk_vcodec_rename.h
