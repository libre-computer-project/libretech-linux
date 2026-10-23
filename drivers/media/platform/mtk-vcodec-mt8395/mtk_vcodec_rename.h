/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Coexistence rename header for the out-of-tree MT8188/MT8370/MT8195 (mt8395
 * generation) vendor codec stack (mtk-vpu-mt8395/ + mtk-vcodec-mt8395/). The
 * vendor sources were copied from a common ancestor of TWO other in-tree codec
 * stacks, so the vendor's non-self-prefixed globals collide at vmlinux link when
 * the others are built in:
 *
 *   (A) mainline drivers/media/platform/mediatek/vcodec (the SCP stateless
 *       stack that serves the MediaTek Chromebooks; CONFIG_VIDEO_MEDIATEK_VCODEC).
 *   (B) the MT8365/Genio-350 VCU stack -- drivers/media/platform/mtk-vcodec-mt8167/
 *       (CONFIG_VIDEO_MEDIATEK_VCODEC_MT8167) + drivers/media/platform/mtk-vpu/
 *       (CONFIG_VIDEO_MEDIATEK_VCU_MT8365). This older VCU port did NOT suffix
 *       its globals, so the vendor stack's matching un-suffixed names collide
 *       (mtk_vcodec_dec_clock_on, vcu_dec_ipi_handler, mtk_vcu_get_buffer, ...).
 *
 * The Genio 510 (MT8188/MT8370) uses THIS vendor VCU path; (A) serves the
 * Chromebooks and (B) serves the Genio 350 -- all three must build + coexist,
 * each binding only its own boards' DT nodes (by of_match/compatible, never by
 * driver name). Force-included into every vendor mtk-vcodec-mt8395 .c AND every
 * mtk-vpu-mt8395 .c via the Makefiles (ccflags -include), this prefixes the
 * vendor's colliding globals with m8395_ so all three stacks coexist. It renames
 * definitions, declarations AND callers in one shot (preprocessor), and only
 * ever applies to the vendor TUs -- mainline and the mt8167 stack are untouched.
 *
 * NOTE on what is and is NOT here: the mt8395-generation BSP already self-
 * prefixed many globals with a _mt8395 suffix (mtk_v4l2_dbg_level_mt8395,
 * mtk_vcodec_set_curr_ctx_mt8395, the whole vcu_*_mt8395 provider API, the
 * vcu_func_mt8395 dispatch table, the dec/enc IRQ handlers, ...), and the
 * cross-module VCU API is reached through the exported vcu_func_mt8395 function-
 * pointer table (VCU_FPTR(x) = vcu_func_mt8395.x) rather than by direct symbol
 * reference -- those need no rename. The two blocks below are the globals the
 * BSP did NOT self-prefix and that are defined non-static in (A) [first block]
 * or in (B) [second block]. Each symbol was verified non-static in BOTH the
 * mt8395 sources and the corresponding colliding stack (name-based symbol
 * intersection, 2026-06-25).
 */
#ifndef _MTK_VCODEC_MT8395_RENAME_H
#define _MTK_VCODEC_MT8395_RENAME_H

#define vdec_if_init			m8395_vdec_if_init
#define vdec_if_decode			m8395_vdec_if_decode
#define vdec_if_get_param		m8395_vdec_if_get_param
#define vdec_if_deinit			m8395_vdec_if_deinit

#define venc_if_init			m8395_venc_if_init
#define venc_if_encode			m8395_venc_if_encode
#define venc_if_set_param		m8395_venc_if_set_param
#define venc_if_deinit			m8395_venc_if_deinit

#define mtk_vdec_lock			m8395_mtk_vdec_lock
#define mtk_vdec_unlock			m8395_mtk_vdec_unlock
#define mtk_venc_lock			m8395_mtk_venc_lock
#define mtk_venc_unlock			m8395_mtk_venc_unlock

#define mtk_vdec_ioctl_ops		m8395_mtk_vdec_ioctl_ops
#define mtk_vdec_m2m_ops		m8395_mtk_vdec_m2m_ops
#define mtk_venc_ioctl_ops		m8395_mtk_venc_ioctl_ops
#define mtk_venc_m2m_ops		m8395_mtk_venc_m2m_ops

#define mtk_vcodec_dec_queue_init	m8395_mtk_vcodec_dec_queue_init
#define mtk_vcodec_dec_release		m8395_mtk_vcodec_dec_release
#define mtk_vcodec_dec_set_default_params m8395_mtk_vcodec_dec_set_default_params

#define mtk_vcodec_enc_queue_init	m8395_mtk_vcodec_enc_queue_init
#define mtk_vcodec_enc_release		m8395_mtk_vcodec_enc_release
#define mtk_vcodec_enc_set_default_params m8395_mtk_vcodec_enc_set_default_params
#define mtk_vcodec_enc_ctrls_setup	m8395_mtk_vcodec_enc_ctrls_setup
#define mtk_vcodec_enc_clock_on		m8395_mtk_vcodec_enc_clock_on
#define mtk_vcodec_enc_clock_off	m8395_mtk_vcodec_enc_clock_off

#define mtk_vcodec_get_curr_ctx		m8395_mtk_vcodec_get_curr_ctx
#define mtk_vcodec_mem_alloc		m8395_mtk_vcodec_mem_alloc
#define mtk_vcodec_mem_free		m8395_mtk_vcodec_mem_free

/*
 * Block (B): globals that collide with the MT8365/Genio-350 VCU stack
 * (mtk-vcodec-mt8167/ + mtk-vpu/). These are the un-suffixed names the older
 * VCU port shares with the vendor mt8395 stack. The mtk_vcu_{get,free}_buffer
 * and ipi_id_to_inst_id symbols are defined in the mtk-vpu-mt8395 PROVIDER dir,
 * so this header is force-included there too (see that dir's Makefile).
 */
#define vdec_if_set_param		m8395_vdec_if_set_param

#define vcu_dec_init			m8395_vcu_dec_init
#define vcu_dec_start			m8395_vcu_dec_start
#define vcu_dec_end			m8395_vcu_dec_end
#define vcu_dec_deinit			m8395_vcu_dec_deinit
#define vcu_dec_reset			m8395_vcu_dec_reset
#define vcu_dec_set_param		m8395_vcu_dec_set_param
#define vcu_dec_ipi_handler		m8395_vcu_dec_ipi_handler

#define vcu_enc_init			m8395_vcu_enc_init
#define vcu_enc_encode			m8395_vcu_enc_encode
#define vcu_enc_set_param		m8395_vcu_enc_set_param
#define vcu_enc_deinit			m8395_vcu_enc_deinit

#define mtk_vcodec_dec_clock_on		m8395_mtk_vcodec_dec_clock_on
#define mtk_vcodec_dec_clock_off	m8395_mtk_vcodec_dec_clock_off
#define mtk_vcodec_dec_pw_on		m8395_mtk_vcodec_dec_pw_on
#define mtk_vcodec_dec_pw_off		m8395_mtk_vcodec_dec_pw_off
#define mtk_vcodec_dec_ctrls_setup	m8395_mtk_vcodec_dec_ctrls_setup

#define mtk_vcodec_init_dec_pm		m8395_mtk_vcodec_init_dec_pm
#define mtk_vcodec_release_dec_pm	m8395_mtk_vcodec_release_dec_pm
#define mtk_vcodec_init_enc_pm		m8395_mtk_vcodec_init_enc_pm
#define mtk_vcodec_release_enc_pm	m8395_mtk_vcodec_release_enc_pm

#define mtk_vcu_get_buffer		m8395_mtk_vcu_get_buffer
#define mtk_vcu_free_buffer		m8395_mtk_vcu_free_buffer
#define ipi_id_to_inst_id		m8395_ipi_id_to_inst_id

#endif /* _MTK_VCODEC_MT8395_RENAME_H */
