/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2022 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _MTK_VCU_IF_H_
#define _MTK_VCU_IF_H_

#include <linux/dma-buf.h>
#include "mtk_vcu.h"

#define VIDEO_CAPTURE_MAX_FB	32
#define MTK_VIDEO_MAX_PLANES	3

enum cam_ipi_msg_status {
	CAM_IPI_MSG_STATUS_OK		= 0,
	CAM_IPI_MSG_STATUS_FAIL		= -1,
	CAM_IPI_MSG_TIMEOUT		= -2,
};

enum cam_ipi_msgid {
	AP_IPIMSG_CAM_INIT               = 0xC000,
	AP_IPIMSG_CAM_START_STREAM       = 0xC001,
	AP_IPIMSG_CAM_INIT_BUFFER        = 0xC002,
	AP_IPIMSG_CAM_DEINIT_BUFFER      = 0xC003,
	AP_IPIMSG_CAM_START              = 0xC004,
	AP_IPIMSG_CAM_END                = 0xC005,
	AP_IPIMSG_CAM_STOP_STREAM        = 0xC006,
	AP_IPIMSG_CAM_DEINIT             = 0xC007,
	AP_IPIMSG_CAM_SET_PARAM          = 0xC008,
	AP_IPIMSG_CAM_GET_PARAM          = 0xC009,
	VCU_IPIMSG_CAM_INIT_ACK          = 0xD000,
	VCU_IPIMSG_CAM_START_STREAM_ACK  = 0xD001,
	VCU_IPIMSG_CAM_INIT_BUFFER_ACK   = 0xD002,
	VCU_IPIMSG_CAM_DEINIT_BUFFER_ACK = 0xD003,
	VCU_IPIMSG_CAM_START_ACK         = 0xD004,
	VCU_IPIMSG_CAM_END_ACK           = 0xD005,
	VCU_IPIMSG_CAM_STOP_STREAM_ACK   = 0xD006,
	VCU_IPIMSG_CAM_DEINIT_ACK        = 0xD007,
	VCU_IPIMSG_CAM_SET_PARAM_ACK     = 0xD008,
	VCU_IPIMSG_CAM_GET_PARAM_ACK     = 0xD009,
};

struct cam_vcu_inst {
	struct mtk_camera_ctx *ctx;
	struct platform_device *dev;
	struct list_head queue;
	enum   ipi_id id;
	uint64_t inst_addr;//take care of it
	int32_t  signaled;
	int32_t  failure;
};

#pragma pack(push, 4)

struct fb_info_in {
	uint32_t index;
	uint64_t dma_addr[MTK_VIDEO_MAX_PLANES];
	uint32_t dma_size[MTK_VIDEO_MAX_PLANES];
	uint32_t num_planes;
	union {
		uint64_t ptr[MTK_VIDEO_MAX_PLANES];
		int fd[MTK_VIDEO_MAX_PLANES];
	} dbuf;
};

struct fb_info_out {
	uint32_t index;
	uint64_t dma_addr;
	uint32_t dma_size;
	uint32_t status;
};

/**
 * struct cam_ap_ipi_cmd - generic AP to VCU ipi command format
 * @msg_id		: camera_ipi_msgid
 * @vcu_inst_addr : VCU camera instance address
 */
struct cam_ap_ipi_cmd {
	uint32_t msg_id;
	uint32_t ipi_id;
	uint64_t vcu_inst_addr;
	uint64_t ap_inst_addr;
	struct fb_info_in info;
	uint32_t stream_id;
};

/**
 * struct cam_vcu_ipi_ack - generic VPU to AP ipi command format
 * @msg_id		: camera_ipi_msgid
 * @status		: VPU execution result
 * @ap_inst_addr	: AP camera instance address
 */
struct cam_vcu_ipi_ack {
	uint32_t msg_id;
	uint32_t ipi_id;
	uint64_t vcu_inst_addr;
	uint64_t ap_inst_addr;
	struct fb_info_out info;
	int32_t  status;
	uint32_t stream_id;
};

/**
 * struct cam_ap_ipi_init - for AP_IPIMSG_CAPTURE_INIT
 * @msg_id		: AP_IPIMSG_DEC_INIT
 * @reserved		: Reserved field
 * @ap_inst_addr	: AP camera instance address
 */
struct cam_ap_ipi_init {
	uint32_t msg_id;
	uint32_t ipi_id;
	uint64_t ap_inst_addr;
	uint32_t stream_id;
};

/**
 * struct cam_ap_ipi_set_param - for AP_IPIMSG_CAPTURE_SET_PARAM
 * @msg_id		: AP_IPIMSG_DEC_SET_PARAM
 * @vcu_inst_addr	: VCU decoder instance address
 * @id			: set param  type
 * @data		: param data
 */
struct cam_ap_ipi_set_param {
	uint32_t msg_id;
	uint32_t ipi_id;
	uint32_t id;
	uint64_t vcu_inst_addr;
	uint64_t ap_inst_addr;
	uint32_t data[4];
	uint32_t stream_id;
};


#pragma pack(pop)

#endif
