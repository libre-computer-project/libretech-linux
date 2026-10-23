/*
 * MediaTek Controls Header
 *
 * Copyright (c) 2017 MediaTek Inc.
 * Author: Yunfei Dong <yunfei.dong@mediatek.com>
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

#ifndef __UAPI_MTK_VCU_CONTROLS_H__
#define __UAPI_MTK_VCU_CONTROLS_H__

#define SHARE_BUF_SIZE 48

#define VCUD_SET_OBJECT	_IOW('v', 0, struct share_obj)
#define VCUD_MVA_ALLOCATION	_IOWR('v', 1, struct mem_obj)
#define VCUD_MVA_FREE		_IOWR('v', 2, struct mem_obj)
#define VCUD_CACHE_FLUSH_ALL	_IOWR('v', 3, struct mem_obj)
#define VCUD_GET_OBJECT	_IOWR('v', 4, struct share_obj)
#define VCUD_GET_LOG_OBJECT	_IOW('v', 5, struct log_test_nofuse)
#define VCUD_MVA_MAP_CACHE	_IOWR('v', 6, struct mem_obj)
#define VCUD_SET_MMAP_TYPE	_IOW('v', 8, struct map_obj)

/**
 * struct mem_obj - memory buffer allocated in kernel
 *
 * @iova:	iova of buffer
 * @len:	buffer length
 * @va: kernel virtual address
 */
struct mem_obj {
	u64 iova;
	u64 len;
	u64 va;
};

/**
 * struct map_obj - memory buffer mmaped in kernel
 *
 * @map_buf:	iova of buffer
 * @map_type:	the type of mmap
 * @reserved: reserved
 */
struct map_obj {
	unsigned long map_buf;
	unsigned long map_type;
	u64 reserved;
};

/**
 * struct share_obj - DTCM (Data Tightly-Coupled Memory) buffer shared with
 *		      AP and VCU
 *
 * @id:		IPI id
 * @len:	share buffer length
 * @share_buf:	share buffer data
 */
struct share_obj {
	s32 id;
	u32 len;
	unsigned char share_buf[SHARE_BUF_SIZE];
};

struct log_test_nofuse {
	char log_info[1024];
};

#endif
