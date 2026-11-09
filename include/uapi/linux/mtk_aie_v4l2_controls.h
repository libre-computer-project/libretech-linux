/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * MediaTek AI Engine (AIE) V4L2 control definitions
 *
 * Copyright (c) 2020 MediaTek Inc.
 * Author: Fish Wu <fish.wu@mediatek.com>
 */

#ifndef __UAPI_MTK_AIE_V4L2_CONTROLS_H__
#define __UAPI_MTK_AIE_V4L2_CONTROLS_H__

#include <linux/videodev2.h>

/*
 * The base for the MediaTek AIE driver controls.
 * We reserve 16 controls for this driver.
 */
#define V4L2_CID_USER_MTK_FD_BASE	(V4L2_CID_USER_BASE + 0x1fd0)

#define V4L2_CID_MTK_AIE_INIT		(V4L2_CID_USER_MTK_FD_BASE + 1)
#define V4L2_CID_MTK_AIE_PARAM		(V4L2_CID_USER_MTK_FD_BASE + 2)

#endif /* __UAPI_MTK_AIE_V4L2_CONTROLS_H__ */
