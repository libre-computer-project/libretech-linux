# SPDX-License-Identifier: GPL-2.0
# Makefile for Amlogic meson video decoder driver

# Core module -- platform driver, V4L2 M2M, esparser, ISR, CMA, encoder
meson-vdec-core-objs = esparser.o vdec.o vdec_helpers.o vdec_platform.o
meson-vdec-core-objs += vdec_1.o vdec_hevc.o
meson-vdec-core-objs += codec_hevc_common.o
meson-vdec-core-objs += hcodec.o jpegenc.o venc.o
meson-vdec-core-objs += vdec_debugfs.o
meson-vdec-core-objs += vdec_diag.o
meson-vdec-core-objs += codec_avs_fw.o
meson-vdec-core-objs += codec_hevc_fw.o
meson-vdec-core-objs += codec_vp9_fw.o
meson-vdec-core-objs += codec_avs2_fw.o
meson-vdec-core-objs += codec_h264_fw.o
meson-vdec-core-objs += codec_h264_mvc_fw.o
meson-vdec-core-objs += codec_mpeg12_fw.o
meson-vdec-core-objs += codec_mpeg4_fw.o
meson-vdec-core-objs += codec_mjpeg_fw.o
meson-vdec-core-objs += codec_h263_fw.o
meson-vdec-core-objs += codec_hcodec_fw.o

# Per-codec modules -- hot-swappable without V4L2 teardown
meson-vdec-h264-objs = codec_h264.o
meson-vdec-hevc-objs = codec_hevc.o codec_hevc_bsp.o
meson-vdec-vp9-objs = codec_vp9.o
meson-vdec-mpeg12-objs = codec_mpeg12.o
meson-vdec-mpeg12-libre-objs = codec_mpeg12_libre.o
meson-vdec-h264-libre-objs = codec_h264_libre.o
meson-vdec-mpeg4-objs = codec_mpeg4.o
meson-vdec-mjpeg-objs = codec_mjpeg.o
meson-vdec-h264-mvc-objs = codec_h264_mvc.o
meson-vdec-avs-objs = codec_avs.o

obj-$(CONFIG_VIDEO_MESON_VDEC) += meson-vdec-core.o
obj-$(CONFIG_VIDEO_MESON_VDEC) += meson-vdec-h264.o
obj-$(CONFIG_VIDEO_MESON_VDEC) += meson-vdec-hevc.o
obj-$(CONFIG_VIDEO_MESON_VDEC) += meson-vdec-vp9.o
obj-$(CONFIG_VIDEO_MESON_VDEC) += meson-vdec-mpeg12.o
obj-$(CONFIG_VIDEO_MESON_VDEC) += meson-vdec-mpeg12-libre.o
obj-$(CONFIG_VIDEO_MESON_VDEC) += meson-vdec-h264-libre.o
obj-$(CONFIG_VIDEO_MESON_VDEC) += meson-vdec-mpeg4.o
obj-$(CONFIG_VIDEO_MESON_VDEC) += meson-vdec-mjpeg.o
obj-$(CONFIG_VIDEO_MESON_VDEC) += meson-vdec-h264-mvc.o
obj-$(CONFIG_VIDEO_MESON_VDEC) += meson-vdec-avs.o
