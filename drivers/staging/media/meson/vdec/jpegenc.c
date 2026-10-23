// SPDX-License-Identifier: GPL-2.0+
/*
 * Amlogic Meson HCODEC JPEG Encoder - Hardware Operations
 *
 * Based on Amlogic BSP JPEG encoder driver.
 * Copyright (C) 2024 LibreTech
 */

#include <linux/err.h>
#include <linux/firmware.h>
#include "vdec.h"
#include "vdec_helpers.h"
#include "hcodec.h"
#include "hcodec_regs.h"
#include "jpegenc.h"
#include "codec_hcodec_fw.h"

/* Standard JPEG quantization tables (JPEG spec Annex K, Tables K.1/K.2) */
static const u8 jpeg_luma_quant[64] = {
	16, 11, 10, 16,  24,  40,  51,  61,
	12, 12, 14, 19,  26,  58,  60,  55,
	14, 13, 16, 24,  40,  57,  69,  56,
	14, 17, 22, 29,  51,  87,  80,  62,
	18, 22, 37, 56,  68, 109, 103,  77,
	24, 35, 55, 64,  81, 104, 113,  92,
	49, 64, 78, 87, 103, 121, 120, 101,
	72, 92, 95, 98, 112, 100, 103,  99,
};

static const u8 jpeg_chroma_quant[64] = {
	17, 18, 24, 47, 99, 99, 99, 99,
	18, 21, 26, 66, 99, 99, 99, 99,
	24, 26, 56, 99, 99, 99, 99, 99,
	47, 66, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99,
};

/* JPEG zigzag order */
static const u8 zigzag_order[64] = {
	 0,  1,  8, 16,  9,  2,  3, 10,
	17, 24, 32, 25, 18, 11,  4,  5,
	12, 19, 26, 33, 40, 48, 41, 34,
	27, 20, 13,  6,  7, 14, 21, 28,
	35, 42, 49, 56, 57, 50, 43, 36,
	29, 22, 15, 23, 30, 37, 44, 51,
	58, 59, 52, 45, 38, 31, 39, 46,
	53, 60, 61, 54, 47, 55, 62, 63,
};

/*
 * Standard Huffman tables (JPEG spec Annex K)
 * Format: 16 code-length counts + symbol values
 */
static const u8 dc_luma_table[] = {
	/* Code length counts (1-16) */
	0x00, 0x01, 0x05, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* Symbol values */
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
	0x08, 0x09, 0x0a, 0x0b,
};

static const u8 dc_chroma_table[] = {
	0x00, 0x03, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
	0x08, 0x09, 0x0a, 0x0b,
};

static const u8 ac_luma_table[] = {
	/* Code length counts (1-16) */
	0x00, 0x02, 0x01, 0x03, 0x03, 0x02, 0x04, 0x03,
	0x05, 0x05, 0x04, 0x04, 0x00, 0x00, 0x01, 0x7d,
	/* Symbol values (162 total) */
	0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12,
	0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07,
	0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08,
	0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0,
	0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16,
	0x17, 0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28,
	0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
	0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
	0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
	0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
	0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
	0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
	0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98,
	0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7,
	0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6,
	0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5,
	0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4,
	0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2,
	0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea,
	0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
	0xf9, 0xfa,
};

static const u8 ac_chroma_table[] = {
	0x00, 0x02, 0x01, 0x02, 0x04, 0x04, 0x03, 0x04,
	0x07, 0x05, 0x04, 0x04, 0x00, 0x01, 0x02, 0x77,
	0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21,
	0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71,
	0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91,
	0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33, 0x52, 0xf0,
	0x15, 0x62, 0x72, 0xd1, 0x0a, 0x16, 0x24, 0x34,
	0xe1, 0x25, 0xf1, 0x17, 0x18, 0x19, 0x1a, 0x26,
	0x27, 0x28, 0x29, 0x2a, 0x35, 0x36, 0x37, 0x38,
	0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
	0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
	0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68,
	0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78,
	0x79, 0x7a, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
	0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96,
	0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5,
	0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4,
	0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3,
	0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2,
	0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda,
	0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9,
	0xea, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
	0xf9, 0xfa,
};

static int jpeg_quality_scaling(int quality)
{
	if (quality <= 0)
		quality = 1;
	if (quality > 100)
		quality = 100;
	if (quality < 50)
		return 5000 / quality;
	return 200 - quality * 2;
}

static void jpeg_scale_quant(const u8 *base, u8 *out, int quality)
{
	int scale = jpeg_quality_scaling(quality);
	int i, v;

	for (i = 0; i < 64; i++) {
		v = ((int)base[i] * scale + 50) / 100;
		if (v < 1)
			v = 1;
		if (v > 255)
			v = 255;
		out[i] = v;
	}
}

/*
 * Build DC Huffman LUT for hardware.
 * BSP algorithm: construct codewords from length counts and symbol values,
 * write 12 entries as ((code_len-1) << 16) | code_word.
 */
static void jpeg_write_dc_huff_lut(struct amvdec_core *core,
				    const u8 *table)
{
	u32 lut[12] = {};
	u32 code_len = 1, code_word = 1, pos = 16;
	u32 i, j, num;

	for (i = 0; i < 16; i++) {
		num = table[i];
		for (j = 0; j < num; j++) {
			code_word = (code_word + 1) & ((1 << code_len) - 1);
			if (code_len < i + 1) {
				code_word <<= (i + 1 - code_len);
				code_len = i + 1;
			}
			lut[table[pos]] = ((code_len - 1) << 16) | code_word;
			pos++;
		}
	}

	for (i = 0; i < 12; i++)
		amvdec_write_dos(core, HCODEC_VLC_HUFFMAN_DATA, lut[i]);
}

/*
 * Build AC Huffman LUT for hardware.
 * 162 entries indexed by: EOB=0, 1+16*(size-1)+run, ZRL=161.
 */
static void jpeg_write_ac_huff_lut(struct amvdec_core *core,
				    const u8 *table)
{
	u32 lut[162] = {};
	u32 code_len = 1, code_word = 1, pos = 16;
	u32 i, j, num, run, size, addr, data;

	for (i = 0; i < 16; i++) {
		num = table[i];
		for (j = 0; j < num; j++) {
			code_word = (code_word + 1) & ((1 << code_len) - 1);
			if (code_len < i + 1) {
				code_word <<= (i + 1 - code_len);
				code_len = i + 1;
			}
			run = table[pos] >> 4;
			size = table[pos] & 0xf;
			data = ((code_len - 1) << 16) | code_word;
			if (size == 0) {
				if (run == 0)
					addr = 0;	/* EOB */
				else
					addr = 161;	/* ZRL */
			} else {
				addr = 1 + 16 * (size - 1) + run;
			}
			lut[addr] = data;
			pos++;
		}
	}

	for (i = 0; i < 162; i++)
		amvdec_write_dos(core, HCODEC_VLC_HUFFMAN_DATA, lut[i]);
}

/*
 * Write quantization table to JPEG quant LUT.
 * Hardware expects reciprocal values: 65535/q, packed 2 per word.
 */
static void jpeg_write_quant_lut(struct amvdec_core *core, const u8 *qtable)
{
	u32 data;
	int i;

	for (i = 0; i < 64; i += 2) {
		data = (qtable[i] > 0) ? (65535 / qtable[i]) : 0;
		data |= ((qtable[i + 1] > 0) ?
			 (65535 / qtable[i + 1]) : 0) << 16;
		amvdec_write_dos(core, HCODEC_QDCT_JPEG_QUANT_DATA, data);
	}
}

/*
 * jpegenc_compute_tables - fill ctx->jpeg_eff_luma_qt / jpeg_eff_chroma_qt.
 *
 * If the user has provided a custom table (any entry > 1 is the sentinel for
 * "user supplied"), copy it directly.  Otherwise scale the standard Annex K
 * tables by ctx->jpeg_quality.
 *
 * Called from amvenc_device_run() before jpegenc_build_header() and
 * jpegenc_encode() so that both use the same resolved tables.
 */
void jpegenc_compute_tables(struct amvenc_ctx *ctx)
{
	bool has_custom_luma = false, has_custom_chroma = false;
	int i;

	for (i = 0; i < 64 && !has_custom_luma; i++)
		if (ctx->jpeg_luma_quant_user[i] > 1)
			has_custom_luma = true;
	for (i = 0; i < 64 && !has_custom_chroma; i++)
		if (ctx->jpeg_chroma_quant_user[i] > 1)
			has_custom_chroma = true;

	if (has_custom_luma)
		memcpy(ctx->jpeg_eff_luma_qt, ctx->jpeg_luma_quant_user, 64);
	else
		jpeg_scale_quant(jpeg_luma_quant, ctx->jpeg_eff_luma_qt,
				 ctx->jpeg_quality);

	if (has_custom_chroma)
		memcpy(ctx->jpeg_eff_chroma_qt, ctx->jpeg_chroma_quant_user, 64);
	else
		jpeg_scale_quant(jpeg_chroma_quant, ctx->jpeg_eff_chroma_qt,
				 ctx->jpeg_quality);
}

/*
 * Build JFIF header for JPEG output.
 * Returns header size in bytes.
 * The firmware only outputs compressed scan data (after SOS),
 * so the driver prepends SOI+DQT+DHT+SOF0+SOS.
 * luma_qt and chroma_qt are pre-computed 64-entry tables (from
 * jpegenc_compute_tables).
 */
u32 jpegenc_build_header(u8 *buf, u32 width, u32 height,
			 const u8 *luma_qt, const u8 *chroma_qt, u32 chroma)
{
	u32 off = 0;
	int i;
	/*
	 * Y sampling factor in SOF0:
	 *   4:4:4 -> H=1,V=1 -> 0x11
	 *   4:2:2 -> H=2,V=1 -> 0x21
	 *   4:2:0 -> H=2,V=2 -> 0x22
	 */
	u8 y_samp;

	if (chroma == V4L2_JPEG_CHROMA_SUBSAMPLING_444)
		y_samp = 0x11;
	else if (chroma == V4L2_JPEG_CHROMA_SUBSAMPLING_422)
		y_samp = 0x21;
	else
		y_samp = 0x22;

	/* SOI */
	buf[off++] = 0xff;
	buf[off++] = 0xd8;

	/* DQT marker - luma table (id=0) */
	buf[off++] = 0xff;
	buf[off++] = 0xdb;
	buf[off++] = 0x00;	/* Lq high */
	buf[off++] = 0x43;	/* Lq low = 2 + 65 = 67 */
	buf[off++] = 0x00;	/* Pq=0 (8-bit), Tq=0 */
	for (i = 0; i < 64; i++)
		buf[off++] = luma_qt[zigzag_order[i]];

	/* DQT marker - chroma table (id=1) */
	buf[off++] = 0xff;
	buf[off++] = 0xdb;
	buf[off++] = 0x00;
	buf[off++] = 0x43;
	buf[off++] = 0x01;	/* Pq=0, Tq=1 */
	for (i = 0; i < 64; i++)
		buf[off++] = chroma_qt[zigzag_order[i]];

	/* DHT marker - all 4 tables in one segment */
	{
		u32 dht_len = 2 +
			(1 + 16 + 12) +	/* DC luma */
			(1 + 16 + 12) +	/* DC chroma */
			(1 + 16 + 162) +	/* AC luma */
			(1 + 16 + 162);		/* AC chroma */

		buf[off++] = 0xff;
		buf[off++] = 0xc4;
		buf[off++] = (dht_len >> 8) & 0xff;
		buf[off++] = dht_len & 0xff;

		/* DC luma: class=0, id=0 */
		buf[off++] = 0x00;
		for (i = 0; i < 16 + 12; i++)
			buf[off++] = dc_luma_table[i];

		/* DC chroma: class=0, id=1 */
		buf[off++] = 0x01;
		for (i = 0; i < 16 + 12; i++)
			buf[off++] = dc_chroma_table[i];

		/* AC luma: class=1, id=0 */
		buf[off++] = 0x10;
		for (i = 0; i < 16 + 162; i++)
			buf[off++] = ac_luma_table[i];

		/* AC chroma: class=1, id=1 */
		buf[off++] = 0x11;
		for (i = 0; i < 16 + 162; i++)
			buf[off++] = ac_chroma_table[i];
	}

	/* SOF0 (Baseline DCT) */
	buf[off++] = 0xff;
	buf[off++] = 0xc0;
	buf[off++] = 0x00;
	buf[off++] = 0x11;	/* Lf = 8 + 3*3 = 17 */
	buf[off++] = 0x08;	/* P = 8 bits */
	buf[off++] = (height >> 8) & 0xff;
	buf[off++] = height & 0xff;
	buf[off++] = (width >> 8) & 0xff;
	buf[off++] = width & 0xff;
	buf[off++] = 0x03;	/* Nf = 3 components */
	/* Y: id=1, sampling factor, qt=0 */
	buf[off++] = 0x01;
	buf[off++] = y_samp;	/* H=1,V=1 for YUV444 (0x11); H=2,V=1 for YUV422 (0x21); H=2,V=2 for YUV420 (0x22) */
	buf[off++] = 0x00;	/* Tq=0 (luma quant table) */
	/* Cb: id=2, sampling=1x1, qt=1 */
	buf[off++] = 0x02;
	buf[off++] = 0x11;
	buf[off++] = 0x01;
	/* Cr: id=3, sampling=1x1, qt=1 */
	buf[off++] = 0x03;
	buf[off++] = 0x11;
	buf[off++] = 0x01;

	/* SOS */
	buf[off++] = 0xff;
	buf[off++] = 0xda;
	buf[off++] = 0x00;
	buf[off++] = 0x0c;	/* Ls = 6 + 2*3 = 12 */
	buf[off++] = 0x03;	/* Ns = 3 */
	/* Y: Cs=1, Td=0, Ta=0 */
	buf[off++] = 0x01;
	buf[off++] = 0x00;
	/* Cb: Cs=2, Td=1, Ta=1 */
	buf[off++] = 0x02;
	buf[off++] = 0x11;
	/* Cr: Cs=3, Td=1, Ta=1 */
	buf[off++] = 0x03;
	buf[off++] = 0x11;
	buf[off++] = 0x00;	/* Ss */
	buf[off++] = 0x3f;	/* Se */
	buf[off++] = 0x00;	/* Ah=0, Al=0 */

	return off;
}

int jpegenc_load_firmware(struct amvdec_core *core)
{
	const struct firmware *fw;
	const struct amvdec_fw_version *match;
	void *mc_addr;
	dma_addr_t mc_paddr;
	u32 i;
	int ret;

	ret = request_firmware(&fw, JPEGENC_FW_NAME, core->dev);
	if (ret) {
		dev_err(core->dev, "Failed to load JPEG firmware %s: %d\n",
			JPEGENC_FW_NAME, ret);
		return ret;
	}

	match = amvdec_fw_identify(core->dev, JPEGENC_FW_NAME,
				   fw->data, fw->size,
				   hcodec_fw_versions, HCODEC_FW_VERSIONS_COUNT);
	if (IS_ERR(match)) {
		ret = PTR_ERR(match);
		release_firmware(fw);
		return ret;
	}
	/* Encoders have no quirk bits yet; don't persist match. */

	if (fw->size < 4096 || fw->size % 4) {
		dev_err(core->dev, "Invalid JPEG firmware size: %zu\n",
			fw->size);
		ret = -EINVAL;
		goto out;
	}

	mc_addr = dma_alloc_coherent(core->dev, fw->size,
				     &mc_paddr, GFP_KERNEL);
	if (!mc_addr) {
		ret = -ENOMEM;
		goto out;
	}

	memcpy(mc_addr, fw->data, fw->size);

	amvdec_write_dos(core, HCODEC_MPSR, 0);
	amvdec_write_dos(core, HCODEC_CPSR, 0);

	amvdec_write_dos(core, HCODEC_ASSIST_MMC_CTRL1, ASSIST_MMC_CTRL1_INIT);

	amvdec_write_dos(core, HCODEC_IMEM_DMA_ADR, mc_paddr);
	amvdec_write_dos(core, HCODEC_IMEM_DMA_COUNT, 0x1000); /* JPEG FW: 4096 words */
	amvdec_write_dos(core, HCODEC_IMEM_DMA_CTRL,
			 IMEM_DMA_CTRL_FILL | IMEM_DMA_CTRL_CTRL_BITS);

	for (i = 0; i < 1000; i++) {
		if (!(amvdec_read_dos(core, HCODEC_IMEM_DMA_CTRL) & IMEM_DMA_CTRL_FILL))
			break;
		usleep_range(100, 200);
	}

	if (i >= 1000) {
		dev_err(core->dev, "JPEG firmware DMA timeout\n");
		ret = -ETIMEDOUT;
	}

	dma_free_coherent(core->dev, fw->size, mc_addr, mc_paddr);
out:
	release_firmware(fw);
	return ret;
}

static void jpeg_setup_encoder(struct amvdec_core *core,
			       struct amvenc_ctx *ctx)
{
	u32 data32;
	/* Derive chroma from source format (same logic as venc.c effective_chroma) */
	bool is_422 = (ctx->src_fmt.pixelformat == V4L2_PIX_FMT_YUYV);
	bool is_444 = (ctx->src_fmt.pixelformat == V4L2_PIX_FMT_ABGR32);
	/*
	 * pic_format: 1=YUV444, 2=YUV422, 3=YUV420 (VLC_PIC_POSITION bits[1:0])
	 * v_factor_comp0: 0=no vertical sub (444/422), 1=2x vertical (420)
	 * h_factor_comp0: 0=no horizontal sub (444), 1=2x horizontal (422/420)
	 * comp0_cnt_max: MCU luma blocks minus 1 (444: 0, 422: 1, 420: 3)
	 */
	u32 pic_format, v_factor_comp0, h_factor_comp0, comp0_cnt_max;

	if (is_444) {
		pic_format     = 1;
		v_factor_comp0 = 0;
		h_factor_comp0 = 0;
		comp0_cnt_max  = 0;
	} else if (is_422) {
		pic_format     = 2;
		v_factor_comp0 = 0;
		h_factor_comp0 = 1;
		comp0_cnt_max  = 1;
	} else { /* 420 */
		pic_format     = 3;
		v_factor_comp0 = 1;
		h_factor_comp0 = 1;
		comp0_cnt_max  = 3;
	}

	/* Use pre-computed effective tables (set by jpegenc_compute_tables) */

	/* Picture size and format */
	amvdec_write_dos(core, HCODEC_VLC_PIC_SIZE,
			 ctx->width | (ctx->height << 16));
	/* pic_format: 2=YUV422, 3=YUV420; lastcoeff_sel=1 (end of MCU) */
	amvdec_write_dos(core, HCODEC_VLC_PIC_POSITION,
			 pic_format | BIT(4));

	amvdec_write_dos(core, HCODEC_QDCT_JPEG_X_START_END,
			 ((ctx->width - 1) << 16) | 0);
	amvdec_write_dos(core, HCODEC_QDCT_JPEG_Y_START_END,
			 ((ctx->height - 1) << 16) | 0);

	/* Load quantization tables (from pre-computed effective tables) */
	amvdec_write_dos(core, HCODEC_QDCT_JPEG_QUANT_ADDR, 0);
	jpeg_write_quant_lut(core, ctx->jpeg_eff_luma_qt);
	jpeg_write_quant_lut(core, ctx->jpeg_eff_chroma_qt);

	/* Load DC Huffman tables (start addr = 0) */
	amvdec_write_dos(core, HCODEC_VLC_HUFFMAN_ADDR, 0);
	jpeg_write_dc_huff_lut(core, dc_luma_table);
	jpeg_write_dc_huff_lut(core, dc_chroma_table);

	/* Load AC Huffman tables (start addr = 24) */
	amvdec_write_dos(core, HCODEC_VLC_HUFFMAN_ADDR, 24);
	jpeg_write_ac_huff_lut(core, ac_luma_table);
	jpeg_write_ac_huff_lut(core, ac_chroma_table);

	/*
	 * QDCT_JPEG_CTRL config
	 * [19:18] dct_inflow_ctrl = 0
	 * [17:16] lastcoeff_sel = 1 (end of MCU)
	 * [15] quant_sel_comp2 = 1 (chroma uses table 1)
	 * [14] v_factor_comp2 = 0
	 * [13] h_factor_comp2 = 0
	 * [12] comp2_en = 1
	 * [11] quant_sel_comp1 = 1 (chroma uses table 1)
	 * [10] v_factor_comp1 = 0
	 * [9] h_factor_comp1 = 0
	 * [8] comp1_en = 1
	 * [7] quant_sel_comp0 = 0 (luma uses table 0)
	 * [6] v_factor_comp0 = 1 for YUV420, 0 for YUV422/YUV444
	 * [5] h_factor_comp0 = 1 for YUV420/YUV422, 0 for YUV444
	 * [4] comp0_en = 1
	 * [3:1] jdct_intr_sel = 0
	 * [0] jpeg_en = 1
	 */
	data32 = (1 << 16) |				/* lastcoeff_sel */
		 (1 << 15) |				/* quant_sel_comp2 */
		 (1 << 12) |				/* comp2_en */
		 (1 << 11) |				/* quant_sel_comp1 */
		 (1 << 8) |				/* comp1_en */
		 (v_factor_comp0 << 6) |		/* v_factor_comp0 */
		 (h_factor_comp0 << 5) |		/* h_factor_comp0 */
		 (1 << 4) |				/* comp0_en */
		 (1 << 0);				/* jpeg_en */
	amvdec_write_dos(core, HCODEC_QDCT_JPEG_CTRL, data32);

	/*
	 * VLC_JPEG_CTRL config
	 * [29] comp2_ac_table_sel = 1 (uses AC table 1)
	 * [28] comp2_dc_table_sel = 1 (uses DC table 1)
	 * [26:25] comp2_cnt_max = 0 (1x1 - 1)
	 * [24] comp2_en = 1
	 * [21] comp1_ac_table_sel = 1
	 * [20] comp1_dc_table_sel = 1
	 * [18:17] comp1_cnt_max = 0
	 * [16] comp1_en = 1
	 * [13] comp0_ac_table_sel = 0
	 * [12] comp0_dc_table_sel = 0
	 * [10:9] comp0_cnt_max = 3 for YUV420 (2x2-1), 1 for YUV422 (2x1-1)
	 * [8] comp0_en = 1
	 * [0] jpeg_en = 0 (enabled by firmware)
	 */
	data32 = (1 << 29) | (1 << 28) |	/* comp2 huffman sel */
		 (1 << 24) |			/* comp2_en */
		 (1 << 21) | (1 << 20) |	/* comp1 huffman sel */
		 (1 << 16) |			/* comp1_en */
		 (comp0_cnt_max << 9) |		/* comp0_cnt_max: 3=420, 1=422 */
		 (1 << 8);			/* comp0_en */
	amvdec_write_dos(core, HCODEC_VLC_JPEG_CTRL, data32);

	/* MB control: reset (bit9=soft_reset, bit0=mb_reset) */
	amvdec_write_dos(core, HCODEC_QDCT_MB_CONTROL,
			 BIT(9) | BIT(0));
	/* MB control: run (bit13=q_enable, bit12=dct_enable) */
	amvdec_write_dos(core, HCODEC_QDCT_MB_CONTROL,
			 BIT(13) | BIT(12));

	/*
	 * VLC pipeline init.
	 * Do NOT write VLC_CONFIG here -- it is H.264-specific (BSP only
	 * writes it in avc_init_encoder). The firmware handles VLC_TOTAL_BYTES
	 * (copies from SCRATCH_1) and VLC_INT_CONTROL internally.
	 */
	amvdec_write_dos(core, HCODEC_VLC_TOTAL_BYTES, 0);
	amvdec_write_dos(core, HCODEC_VLC_INT_CONTROL, 0);
}

void jpegenc_encode(struct amvdec_core *core, struct amvenc_ctx *ctx,
		    dma_addr_t y_addr, dma_addr_t uv_addr, u32 canvas_w,
		    dma_addr_t vlc_dst, u32 vlc_dst_size)
{
	hcodec_stop(core);
	hcodec_reset(core);

	/* JPEG uses 8x8 block mode for MFDIN */
	ctx->mfdin_mode = 2;

	/* Configure MFDIN input (BSP does mfdin before encoder init) */
	hcodec_setup_mfdin(core, ctx, y_addr, uv_addr, 0, canvas_w);

	/* Enable MFDIN - required for JPEG, H.264 firmware does this itself */
	amvdec_write_dos(core, HCODEC_MFDIN_REG7_SCMD,
			 amvdec_read_dos(core, HCODEC_MFDIN_REG7_SCMD) |
			 BIT(28)); /* MFDIN_ENABLE */

	/* Setup JPEG encoder hardware (uses ctx->jpeg_eff_*_qt, set by jpegenc_compute_tables) */
	jpeg_setup_encoder(core, ctx);

	/*
	 * No QDCT_MB ring for JPEG.  The QDCT_MB_START/END/WR/RD/BUFF writes
	 * are H.264-specific: the JPEG (intra-only, 8x8) firmware does not
	 * consume the DCT buffer, confirmed no-effect on JPEG output
	 * (jpeg/analysis.md, tested 2026-04-11, caf55cd50b80).  Dropping
	 * them lets the JPEG layout omit the (8 MB @1080p) DCT sub-buffer
	 * entirely (KI-116 CMA-fragmentation fix) -- previously this path
	 * dereferenced ctx->buf_layout.dct_size, forcing JPEG to keep the
	 * DCT alloc it never used.
	 */

	/* VLC bitstream output buffer */
	amvdec_write_dos(core, HCODEC_VLC_VB_MEM_CTL, VLC_VB_MEM_CTL_INIT);
	/* VLC output: zero-copy into CAPTURE buffer (8-byte aligned) */
	amvdec_write_dos(core, HCODEC_VLC_VB_START_PTR, vlc_dst);
	amvdec_write_dos(core, HCODEC_VLC_VB_WR_PTR,    vlc_dst);
	amvdec_write_dos(core, HCODEC_VLC_VB_SW_RD_PTR, vlc_dst);
	amvdec_write_dos(core, HCODEC_VLC_VB_END_PTR,   vlc_dst + vlc_dst_size - 1);
	amvdec_write_dos(core, HCODEC_VLC_VB_CONTROL, 1);
	amvdec_write_dos(core, HCODEC_VLC_VB_CONTROL, VLC_VB_CTRL_INIT);

	/*
	 * Clear SCRATCH_1 - firmware copies it to VLC_TOTAL_BYTES during init.
	 * Must be 0 so VLC_TOTAL_BYTES starts at 0.
	 */
	amvdec_write_dos(core, HCODEC_HENC_SCRATCH_1, 0);

	/* Clear and enable mailbox IRQ */
	amvdec_write_dos(core, HCODEC_ASSIST_MBOX2_CLR_REG, 1);
	amvdec_write_dos(core, HCODEC_ASSIST_MBOX2_MASK, 1);

	hcodec_start(core);
}
