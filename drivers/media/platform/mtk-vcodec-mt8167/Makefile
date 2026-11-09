
# Module names carry the SoC token right after the driver-family base
# (mtk-vcodec-mt8167-*), matching the sibling vendor port
# drivers/media/platform/mtk-vcodec-mt8395/ (mtk-vcodec-mt8395-dec/-enc/-common).
# Mainline drivers/media/platform/mediatek/vcodec/ builds mtk-vcodec-dec.ko /
# -enc.ko / -common.ko, so the un-suffixed names collide at modules_check as
# soon as CONFIG_MEDIA_SUPPORT=m makes both trees modular.
obj-$(CONFIG_VIDEO_MEDIATEK_VCODEC_MT8167) += mtk-vcodec-mt8167-dec.o \
				       mtk-vcodec-mt8167-enc.o \
				       mtk-vcodec-mt8167-common.o

mtk-vcodec-mt8167-dec-y := vdec/vdec_common_if.o \
		mtk_vcodec_dec_drv.o \
		vdec_drv_if.o \
		vdec_vcu_if.o \
		mtk_vcodec_dec.o \
		mtk_vcodec_dec_pm.o \


mtk-vcodec-mt8167-enc-y := venc/venc_common_if.o \
		mtk_vcodec_enc.o \
		mtk_vcodec_enc_drv.o \
		mtk_vcodec_enc_pm.o \
		venc_drv_if.o \
		venc_vcu_if.o \


mtk-vcodec-mt8167-common-y := mtk_vcodec_intr.o \
		mtk_vcodec_util.o\

ccflags-y += -I$(srctree)/drivers/media/platform/mtk-vcodec-mt8167
ccflags-y += -I$(srctree)/drivers/media/platform/mtk-vpu
# Coexistence: prefix vendor globals so they don't collide with mainline mtk-vcodec
ccflags-y += -include $(srctree)/drivers/media/platform/mtk-vcodec-mt8167/mtk_vcodec_rename.h
