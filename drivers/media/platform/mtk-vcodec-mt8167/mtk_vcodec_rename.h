/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Coexistence rename header for the out-of-tree MT8365 (Genio 350) vendor codec
 * stack (mtk-vpu/ + mtk-vcodec-mt8167/). The vendor sources were copied from a
 * common ancestor of the mainline drivers/media/platform/mediatek/ codec, so
 * dozens of global symbols collide at vmlinux link when BOTH are built =y (the
 * mainline stack serves the MediaTek Chromebooks; the vendor stack serves g350).
 *
 * Force-included into every vendor .c via the Makefiles (ccflags -include), this
 * prefixes the vendor's colliding globals with m8167_ so the two stacks coexist.
 * It renames definitions, declarations AND callers in one shot (preprocessor),
 * and only ever applies to the vendor TUs -- mainline is untouched.
 */
#ifndef _MTK_VCODEC_MT8167_RENAME_H
#define _MTK_VCODEC_MT8167_RENAME_H

#define mtk_v4l2_dbg_level		m8167_mtk_v4l2_dbg_level
#define mtk_vcodec_dbg			m8167_mtk_vcodec_dbg

#define vdec_if_init			m8167_vdec_if_init
#define vdec_if_decode			m8167_vdec_if_decode
#define vdec_if_get_param		m8167_vdec_if_get_param
#define vdec_if_deinit			m8167_vdec_if_deinit

#define venc_if_init			m8167_venc_if_init
#define venc_if_encode			m8167_venc_if_encode
#define venc_if_set_param		m8167_venc_if_set_param
#define venc_if_deinit			m8167_venc_if_deinit

#define mtk_vdec_lock			m8167_mtk_vdec_lock
#define mtk_vdec_unlock			m8167_mtk_vdec_unlock
#define mtk_venc_lock			m8167_mtk_venc_lock
#define mtk_venc_unlock			m8167_mtk_venc_unlock

#define mtk_vdec_ioctl_ops		m8167_mtk_vdec_ioctl_ops
#define mtk_vdec_m2m_ops		m8167_mtk_vdec_m2m_ops
#define mtk_venc_ioctl_ops		m8167_mtk_venc_ioctl_ops
#define mtk_venc_m2m_ops		m8167_mtk_venc_m2m_ops

#define mtk_vcodec_dec_queue_init	m8167_mtk_vcodec_dec_queue_init
#define mtk_vcodec_dec_release		m8167_mtk_vcodec_dec_release
#define mtk_vcodec_dec_set_default_params m8167_mtk_vcodec_dec_set_default_params

#define mtk_vcodec_enc_queue_init	m8167_mtk_vcodec_enc_queue_init
#define mtk_vcodec_enc_release		m8167_mtk_vcodec_enc_release
#define mtk_vcodec_enc_set_default_params m8167_mtk_vcodec_enc_set_default_params
#define mtk_vcodec_enc_ctrls_setup	m8167_mtk_vcodec_enc_ctrls_setup
#define mtk_vcodec_enc_clock_on		m8167_mtk_vcodec_enc_clock_on
#define mtk_vcodec_enc_clock_off	m8167_mtk_vcodec_enc_clock_off

#define mtk_vcodec_get_curr_ctx		m8167_mtk_vcodec_get_curr_ctx
#define mtk_vcodec_set_curr_ctx		m8167_mtk_vcodec_set_curr_ctx
#define mtk_vcodec_get_reg_addr		m8167_mtk_vcodec_get_reg_addr
#define mtk_vcodec_wait_for_done_ctx	m8167_mtk_vcodec_wait_for_done_ctx
#define mtk_vcodec_mem_alloc		m8167_mtk_vcodec_mem_alloc
#define mtk_vcodec_mem_free		m8167_mtk_vcodec_mem_free

#define vpu_get_plat_device		m8167_vpu_get_plat_device

#endif /* _MTK_VCODEC_MT8167_RENAME_H */
