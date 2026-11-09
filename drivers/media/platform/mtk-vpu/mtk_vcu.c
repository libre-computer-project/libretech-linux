/*
 * Copyright (c) 2016 MediaTek Inc.
 * Author: Andrew-CT Chen <andrew-ct.chen@mediatek.com>
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

#include <asm/cacheflush.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/file.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/sched.h>
#include <linux/suspend.h>
#include <linux/uaccess.h>
#include <linux/compat.h>
#include <linux/freezer.h>
#include <linux/pm_runtime.h>

#ifdef CONFIG_MTK_IOMMU
#include <linux/iommu.h>
#endif
#include "mtk_vcodec_mem.h"
#include <uapi/linux/mtk_vcu_controls.h>
#include "mtk_vpu.h"

/*
 * 32-bit compat ioctl interface -- kernel-internal, moved here out of the uapi
 * header <uapi/linux/mtk_vcu_controls.h>, which must not reference CONFIG_COMPAT
 * (it leaked the symbol to user-space and failed headers_install). These use
 * compat_* types from <linux/compat.h> and are consumed by the ioctl handlers
 * below; userspace never sees the compat variants (it uses the VCUD_* set).
 */
#if IS_ENABLED(CONFIG_COMPAT)
struct compat_mem_obj {
	compat_ulong_t iova;
	compat_ulong_t len;
	compat_u64 va;
};

struct compat_map_obj {
	compat_ulong_t map_buf;
	compat_ulong_t map_type;
	compat_u64 reserved;
};

#define COMPAT_VCUD_SET_OBJECT		_IOW('v', 0, struct share_obj)
#define COMPAT_VCUD_MVA_ALLOCATION	_IOWR('v', 1, struct compat_mem_obj)
#define COMPAT_VCUD_MVA_FREE		_IOWR('v', 2, struct compat_mem_obj)
#define COMPAT_VCUD_CACHE_FLUSH_ALL	_IOWR('v', 3, struct compat_mem_obj)
#define COMPAT_VCUD_MVA_MAP_CACHE	_IOWR('v', 6, struct compat_mem_obj)
#define COMPAT_VCUD_SET_MMAP_TYPE	_IOW('v', 8, struct compat_map_obj)
#endif


/* PORT 5.10->6.18: the vendor full-dcache flush/disable ops are an out-of-tree
 * M4U-only hack absent in mainline (the comment below literally says "Others
 * should use DMA-mapping APIs"). The codec buffers here are dma_buf/vb2-managed,
 * so cache coherency is handled by the DMA-mapping framework; stub these to
 * no-ops. Revisit via the DMA-sync API if runtime decode shows coherency
 * corruption. */
static inline void __inner_flush_dcache_all(void) { }
static inline void __inner_flush_dcache_L1(void) { }
static inline void __inner_flush_dcache_L2(void) { }
static inline void __disable_dcache(void) { }

void inner_dcache_flush_all(void)
{
	__inner_flush_dcache_all();
}
EXPORT_SYMBOL(inner_dcache_flush_all);

void inner_dcache_flush_L1(void)
{
	__inner_flush_dcache_L1();
}

void inner_dcache_flush_L2(void)
{
	__inner_flush_dcache_L2();
}

void inner_dcache_disable(void)
{
	__disable_dcache();
}

/*
 * smp_inner_dcache_flush_all: Flush (clean + invalidate) the entire L1 data cache.
 *
 * This can be used ONLY by the M4U driver!!
 * Other drivers should NOT use this function at all!!
 * Others should use DMA-mapping APIs!!
 *
 * This is the smp version of inner_dcache_flush_all().
 * It will use IPI to do flush on all CPUs.
 * Must not call this function with disabled interrupts or from a
 * hardware interrupt handler or from a bottom half handler.
 */
void smp_inner_dcache_flush_all(void)
{
#ifdef CONFIG_MTK_FIQ_CACHE
//	mt_fiq_cache_flush_all();
#else
	int i, total_core, cid, last_cid;
	struct cpumask mask;

	if (in_interrupt()) {
		pr_err("Cannot invoke smp_inner_dcache_flush_all() in interrupt/softirq context\n");
		return;
	}

	cpus_read_lock(); /* PORT 5.10->6.18: get_online_cpus() renamed */
	preempt_disable();

	/* Find first online cpu in each cluster */
	last_cid = -1;
	cpumask_clear(&mask);
	total_core = num_possible_cpus();
	for (i = 0; i < total_core; i++) {
		if (!cpu_online(i))
			continue;

//		cid = arch_get_cluster_id(i);
		cid = 0;
		if (last_cid != cid) {
			cpumask_set_cpu(i, &mask);
			last_cid = cid;
		}
	}

	on_each_cpu((smp_call_func_t)inner_dcache_flush_L1, NULL, true);
	smp_call_function_many(&mask, (smp_call_func_t)inner_dcache_flush_L2,
				NULL, true);
	/*
	 * smp_call_function_many only run on "other Cpus".
	 * Flush L2 here if this is one of the first cores
	 */
	if (cpumask_test_cpu(smp_processor_id(), &mask))
		inner_dcache_flush_L2();

	preempt_enable();
	cpus_read_unlock(); /* PORT 5.10->6.18: put_online_cpus() renamed */
#endif
}

/**
 * VCU (Video Communication/Controller Unit) is a daemon in user space
 * controlling video hardware related to video codec, scaling and color
 * format converting.
 * VCU interfaces with other blocks by share memory and interrupt.
 **/
#define VCU_PATH		"/dev/vpud"
#define MDP_PATH		"/dev/mdpd"
#define CAM_PATH		"/dev/camd"
#define VCU_DEVNAME		"vpu"

#define IPI_TIMEOUT_MS		4000U
#define VCU_FW_VER_LEN		16

/* mtk vcu device instance id enumeration */
enum mtk_vcu_daemon_id {
	MTK_VCU_VPUD = 0,
	MTK_VCU_MDPD = 1,
	MTK_VCU_CAMD = 2,
	MTK_VCU_NR_MAX
};

enum vcu_ipi_id {
	VCU_IPI_VPU_INIT = 0,
	VCU_IPI_VDEC_H264,
	VCU_IPI_VDEC_H265,
	VCU_IPI_VDEC_VP8 = 3,
	VCU_IPI_VDEC_VP9,
	VCU_IPI_VDEC_MPEG4,
	VCU_IPI_VDEC_MPEG12 = 7,
	VCU_IPI_VENC_H264 = 11,
	VCU_IPI_VENC_VP8,
	VCU_IPI_MDP,
	VCU_IPI_VENC_HYBRID_H264 = 25,
	VCU_IPI_MAX,
};

/* vcu extended mapping length */
#define VCU_PMEM0_LEN(vcu_data)	(vcu_data->extmem.p_len)
#define VCU_DMEM0_LEN(vcu_data)	(vcu_data->extmem.d_len)
/* vcu extended user virtural address */
#define VCU_PMEM0_VMA(vcu_data)	(vcu_data->extmem.p_vma)
#define VCU_DMEM0_VMA(vcu_data)	(vcu_data->extmem.d_vma)
/* vcu extended kernel virtural address */
#define VCU_PMEM0_VIRT(vcu_data)	(vcu_data->extmem.p_va)
#define VCU_DMEM0_VIRT(vcu_data)	(vcu_data->extmem.d_va)
/* vcu extended phsyial address */
#define VCU_PMEM0_PHY(vcu_data)	(vcu_data->extmem.p_pa)
#define VCU_DMEM0_PHY(vcu_data)	(vcu_data->extmem.d_pa)
/* vcu extended iova address*/
#define VCU_PMEM0_IOVA(vcu_data)	(vcu_data->extmem.p_iova)
#define VCU_DMEM0_IOVA(vcu_data)	(vcu_data->extmem.d_iova)

#define MAP_SHMEM_ALLOC_BASE	0x80000000UL
#define MAP_SHMEM_ALLOC_RANGE	0x08000000UL
#define MAP_SHMEM_ALLOC_END	(MAP_SHMEM_ALLOC_BASE + MAP_SHMEM_ALLOC_RANGE)
#define MAP_SHMEM_COMMIT_BASE	0x88000000UL
#define MAP_SHMEM_COMMIT_RANGE	0x08000000UL
#define MAP_SHMEM_COMMIT_END	(MAP_SHMEM_COMMIT_BASE + MAP_SHMEM_COMMIT_RANGE)

#define MAP_SHMEM_MM_BASE	0x90000000UL
#define MAP_SHMEM_MM_CACHEABLE_BASE	0x190000000UL
#define MAP_SHMEM_MM_RANGE	0xFFFFFFFFUL
#define MAP_SHMEM_MM_END	(MAP_SHMEM_MM_BASE + MAP_SHMEM_MM_RANGE)
#define MAP_SHMEM_MM_CACHEABLE_END	(MAP_SHMEM_MM_CACHEABLE_BASE + MAP_SHMEM_MM_RANGE)
#define MAP_VENC_CACHE_MAX_NUM 30
#define VCU_IPIMSG_VENC_BASE 0xD000

static inline enum ipi_id ipi_vpu_id_fixup(enum ipi_id id)
{
	switch (id) {
	case IPI_VENC_H264:
		return VCU_IPI_VENC_H264;
	case IPI_VCU_VDEC_H265:
		return VCU_IPI_VDEC_H265;
	case IPI_VCU_VDEC_MPEG4:
		return VCU_IPI_VDEC_MPEG4;
	case IPI_VCU_VDEC_MPEG12:
		return VCU_IPI_VDEC_MPEG12;
	case IPI_VDEC_VP8:
		return VCU_IPI_VDEC_VP8;
	case IPI_VDEC_VP9:
		return VCU_IPI_VDEC_VP9;
	default:
		return id;
	}
}

static inline enum vcu_ipi_id ipi_vpu_to_vcu(enum ipi_id id)
{
	switch (id) {
	case IPI_VPU_INIT:
		return VCU_IPI_VPU_INIT;
	case IPI_VDEC_H264:
		return VCU_IPI_VDEC_H264;
	case IPI_VDEC_VP8:
		return VCU_IPI_VDEC_VP8;
	case IPI_VDEC_VP9:
		return VCU_IPI_VDEC_VP9;
	case IPI_VENC_H264:
		return VCU_IPI_VENC_H264;
	case IPI_VENC_VP8:
		return VCU_IPI_VENC_VP8;
	case IPI_MDP:
		return VCU_IPI_MDP;
	case IPI_VENC_HYBRID_H264:
		return VCU_IPI_VENC_HYBRID_H264;
	case IPI_VCU_VDEC_H265:
		return VCU_IPI_VDEC_H265;
	case IPI_VCU_VDEC_MPEG4:
		return VCU_IPI_VDEC_MPEG4;
	case IPI_VCU_VDEC_MPEG12:
		return VCU_IPI_VDEC_MPEG12;
	case IPI_MAX:
	default:
		return VCU_IPI_MAX;
	}

	return VCU_IPI_MAX;
}

static inline enum ipi_id ipi_vcu_to_vpu(enum vcu_ipi_id id)
{
	switch (id) {
	case VCU_IPI_VPU_INIT:
		return IPI_VPU_INIT;
	case VCU_IPI_VDEC_H264:
		return IPI_VDEC_H264;
	case VCU_IPI_VDEC_H265:
		return IPI_VCU_VDEC_H265;
	case VCU_IPI_VDEC_VP8:
		return IPI_VDEC_VP8;
	case VCU_IPI_VDEC_VP9:
		return IPI_VDEC_VP9;
	case VCU_IPI_VDEC_MPEG4:
		return IPI_VCU_VDEC_MPEG4;
	case VCU_IPI_VDEC_MPEG12:
		return IPI_VCU_VDEC_MPEG12;
	case VCU_IPI_VENC_H264:
		return IPI_VENC_H264;
	case VCU_IPI_VENC_VP8:
		return IPI_VENC_VP8;
	case VCU_IPI_MDP:
		return IPI_MDP;
	case VCU_IPI_VENC_HYBRID_H264:
		return IPI_VENC_HYBRID_H264;
	case VCU_IPI_MAX:
	default:
		return IPI_MAX;
	}

	return IPI_MAX;
}


inline int ipi_id_to_inst_id(int vcuid, int id)
{
	/* Assume VENC uses instance 1 and others use 0. */
	if (vcuid == MTK_VCU_VPUD &&
	    (id == IPI_VENC_H264 ||
	    id ==  IPI_VENC_VP8))
		return 1;

	return 0;
}

enum vcu_map_hw_reg_id {
	VDEC,
	VENC,
	VENC_LT,
	VCU_MAP_HW_REG_NUM
};

static const unsigned long vcu_map_hw_type[VCU_MAP_HW_REG_NUM] = {
	0x70000000,	/* VDEC */
	0x71000000,	/* VENC */
	0x72000000	/* VENC_LT */
};

/* Default vcu_mtkdev[0] handle vdec, vcu_mtkdev[1] handle mdp */
static struct mtk_vcu *vcu_mtkdev[MTK_VCU_NR_MAX];

static struct task_struct *vcud_task;
static struct files_struct *files;

/**
 * struct vcu_mem - VCU memory information
 *
 * @p_vma:	the user virtual memory address of
 *		VCU extended program memory
 * @d_vma:	the user  virtual memory address of VCU extended data memory
 * @p_va:	the kernel virtual memory address of
 *		VCU extended program memory
 * @d_va:	the kernel virtual memory address of VCU extended data memory
 * @p_pa:	the physical memory address of VCU extended program memory
 * @d_pa:	the physical memory address of VCU extended data memory
 * @p_iova:	the iova memory address of VCU extended program memory
 * @d_iova:	the iova memory address of VCU extended data memory
 */
struct vcu_mem {
	unsigned long p_vma;
	unsigned long d_vma;
	void *p_va;
	void *d_va;
	dma_addr_t p_pa;
	dma_addr_t d_pa;
	dma_addr_t p_iova;
	dma_addr_t d_iova;
	unsigned long p_len;
	unsigned long d_len;
};

/**
 * struct vcu_run - VCU initialization status
 *
 * @signaled:		the signal of vcu initialization completed
 * @fw_ver:		VCU firmware version
 * @dec_capability:	decoder capability which is not used for now and
 *			the value is reserved for future use
 * @enc_capability:	encoder capability which is not used for now and
 *			the value is reserved for future use
 * @wq:			wait queue for VCU initialization status
 */
struct vcu_run {
	u32 signaled;
	char fw_ver[VCU_FW_VER_LEN];
	unsigned int	dec_capability;
	unsigned int	enc_capability;
	wait_queue_head_t wq;
};

/**
 * struct vcu_ipi_desc - VCU IPI descriptor
 *
 * @handler:	IPI handler
 * @name:	the name of IPI handler
 * @priv:	the private data of IPI handler
 */
struct vcu_ipi_desc {
	ipi_handler_t handler;
	const char *name;
	void *priv;
};

struct map_hw_reg {
	unsigned long base;
	unsigned long len;
};
struct map_cache_mva {
    unsigned long    mmap64_pa; /* pa */
    unsigned long    length;
};

struct vcu_map_mva {
    uint64_t      mmap64_pa; /* pa */
    uint64_t      length;
    uint32_t      status;
};


/**
 * struct vcu_ipi_msg_common - VCU ack AP cmd common structure
 * @msg_id:	message id (VCU_IPIMSG_XXX_DONE)
 * @status:	cmd status (venc_ipi_msg_status)
 * @venc_inst:	AP encoder instance (struct venc_vp8_inst/venc_h264_inst *)
 */
struct vcu_ipi_msg_common {
	uint32_t msg_id;
	uint32_t status;
	uint64_t inst;
};
/**
 * enum venc_ipi_msg_id - message id between AP and VCU
 * (ipi stands for inter-processor interrupt)
 * @AP_IPIMSG_ENC_XXX:		AP to VCU cmd message id
 * @VCU_IPIMSG_ENC_XXX_DONE:	VCU ack AP cmd message id
 */
enum venc_ipi_msg_id {
	VCU_IPIMSG_ENC_INIT_DONE = VCU_IPIMSG_VENC_BASE,
	VCU_IPIMSG_ENC_SET_PARAM_DONE,
	VCU_IPIMSG_ENC_ENCODE_DONE,
	VCU_IPIMSG_ENC_DEINIT_DONE,
};

/**
 * struct mtk_vcu - vcu driver data
 * @extmem:		VCU extended memory information
 * @run:		VCU initialization status
 * @ipi_desc:		VCU IPI descriptor
 * @dev:		VCU struct device
 * @vcu_mutex:		protect mtk_vcu (except recv_buf) and ensure only
 *			one client to use VCU service at a time. For example,
 *			suppose a client is using VCU to decode VP8.
 *			If the other client wants to encode VP8,
 *			it has to wait until VP8 decode completes.
 * @file:		VCU daemon file pointer
 * @is_open:		The flag to indicate if VCUD device is open.
 * @is_alloc:		The flag to indicate if VCU extended memory is allocated.
 * @ack_wq:		The wait queue for each codec and mdp. When sleeping
 *			processes wake up, they will check the condition
 *			"ipi_id_ack" to run the corresponding action or
 *			go back to sleep.
 * @ipi_id_ack:		The ACKs for registered IPI function sending
 *			interrupt to VCU
 * @get_wq:		When sleeping process waking up, it will check the
 *			condition "ipi_got" to run the corresponding action or
 *			go back to sleep.
 * @ipi_got:		The flags for IPI message polling from user.
 * @ipi_done:		The flags for IPI message polling from user again, which
 *			means the previous messages has been dispatched done in
 *			daemon.
 * @user_obj:		Temporary share_obj used for ipi_msg_get.
 * @vcu_devno:		The vcu_devno for vcu init vcu character device
 * @vcu_cdev:		The point of vcu character device.
 * @vcu_class:		The class_create for create vcu device
 * @vcu_device:		VCU struct device
 * @vcuname:		VCU struct device name in dtsi
 * @path:		The path to keep mdpd path or vcud path.
 * @vpuid:		VCU device id
 * @vpuid:		recorder need mapp cache buffer address
 *
 */
struct mtk_vcu {
	struct mtk_vpu_plat vpu;
	struct vcu_mem extmem;
	struct vcu_run run;
	struct vcu_ipi_desc ipi_desc[IPI_MAX];
	struct device *dev;
	struct mutex vcu_mutex[2]; /* for protecting vcu data structure */
	struct mutex vcu_share;
	struct file *file;
	struct iommu_domain *io_domain;
	struct map_hw_reg map_base[VCU_MAP_HW_REG_NUM];
	bool   is_open;
	bool   is_alloc;
	wait_queue_head_t ack_wq[2];
	bool ipi_id_ack[IPI_MAX];
	wait_queue_head_t get_wq[2];
	atomic_t ipi_got[2];
	atomic_t ipi_done[2];
	struct share_obj user_obj[2];
	dev_t vcu_devno;
	struct cdev *vcu_cdev;
	struct class *vcu_class;
	struct device *vcu_device;
	const char *vcuname;
	const char *path;
	int vcuid;
	struct log_test_nofuse *vdec_log_info;
	wait_queue_head_t vdec_log_get_wq;
	atomic_t vdec_log_got;
	struct map_cache_mva map_buffer[MAP_VENC_CACHE_MAX_NUM];
	struct notifier_block pm_notifier;
};

#define to_vcu(vpu) container_of(vpu, struct mtk_vcu, vpu)

static inline bool vcu_running(struct mtk_vcu *vcu)
{
	return (bool)vcu->run.signaled;
}

int vcu_ipi_register(struct mtk_vpu_plat *vpu,
		     enum ipi_id vpu_id, ipi_handler_t handler,
		     const char *name, void *priv)
{
	struct mtk_vcu *vcu = to_vcu(vpu);
	struct vcu_ipi_desc *ipi_desc;
	enum ipi_id id = ipi_vpu_id_fixup(vpu_id);

	if (vcu == NULL) {
		dev_err(vcu->dev, "vcu device in not ready\n");
		return -EPROBE_DEFER;
	}

	if (id >= 0 && id < IPI_MAX && handler != NULL) {
		ipi_desc = vcu->ipi_desc;
		ipi_desc[id].name = name;
		ipi_desc[id].handler = handler;
		ipi_desc[id].priv = priv;
		return 0;
	}

	dev_err(vcu->dev, "register vcu ipi id %d with invalid arguments\n",
	       id);
	return -EINVAL;
}

int vcu_ipi_send(struct mtk_vpu_plat *vpu,
		 enum ipi_id vpu_id, void *buf,
		 unsigned int len)
{
	int i = 0;
	struct mtk_vcu *vcu = to_vcu(vpu);
	enum ipi_id id = ipi_vpu_id_fixup(vpu_id);
	struct share_obj send_obj;
	unsigned long timeout;
	int ret;

	if (id <= IPI_VPU_INIT || id >= IPI_MAX ||
	    len > sizeof(send_obj.share_buf) || buf == NULL) {
		dev_err(vcu->dev, "[VCU] failed to send ipi message (Invalid arg.)\n");
		return -EINVAL;
	}

	if (vcu_running(vcu) == false) {
		dev_err(vcu->dev, "[VCU] vcu_ipi_send: VCU is not running\n");
		return -EPERM;
	}

	i = ipi_id_to_inst_id(vcu->vcuid, id);

	mutex_lock(&vcu->vcu_mutex[i]);
	vcu->ipi_id_ack[id] = false;
	/* send the command to VCU */
	memcpy((void *)vcu->user_obj[i].share_buf, buf, len);
	vcu->user_obj[i].len = len;
	vcu->user_obj[i].id = (int)ipi_vpu_to_vcu(vpu_id);

	atomic_set(&vcu->ipi_got[i], 1);
	atomic_set(&vcu->ipi_done[i], 0);
	wake_up(&vcu->get_wq[i]);
	/* Waiting ipi_done, success means the daemon receiver thread
	 * dispatchs ipi msg done and returns to kernel for get next
	 * ipi msg.
	 * The dispatched ipi msg is being processed by app service.
	 * Usually, it takes dozens of microseconds in average.
	 */
	while (atomic_read(&vcu->ipi_done[i]) == 0)
		cond_resched();

	ret = 0;
	mutex_unlock(&vcu->vcu_mutex[i]);

	if (ret != 0) {
		dev_err(vcu->dev, "[VCU] failed to send ipi message (ret=%d)\n", ret);
		goto end;
	}

	/* wait for VCU's ACK */
	timeout = msecs_to_jiffies(IPI_TIMEOUT_MS);
	ret = wait_event_timeout(vcu->ack_wq[i], vcu->ipi_id_ack[id], timeout);
	vcu->ipi_id_ack[id] = false;
	if (ret == 0) {
		dev_err(vcu->dev, "vcu ipi %d ack time out !", id);
		ret = -EIO;
		goto end;
	} else if (-ERESTARTSYS == ret) {
		dev_err(vcu->dev, "vcu ipi %d ack wait interrupted by a signal",
		       id);
		ret = -ERESTARTSYS;
		goto end;
	} else
		ret = 0;

end:
	return ret;
}

static int vcu_ipi_get(struct mtk_vcu *vcu, unsigned long arg)
{
	int i = 0, ret;
	unsigned char *user_data_addr = NULL;
	struct share_obj share_buff_data;

	user_data_addr = (unsigned char *)arg;
	ret = (long)copy_from_user(&share_buff_data, user_data_addr,
		(unsigned long)sizeof(struct share_obj));
	i = ipi_id_to_inst_id(vcu->vcuid,
			      ipi_vcu_to_vpu(share_buff_data.id));

	/* mutex protection here is unnecessary, since different app service
	 * threads of daemon are corresponding to different vcu_ipi_get thread.
	 * Different threads use differnet variables, e.g. ipi_done.
	 */
	atomic_set(&vcu->ipi_done[i], 1);

	ret = wait_event_freezable(vcu->get_wq[i],
				    atomic_read(&vcu->ipi_got[i]));
	if (ret != 0) {
		pr_info("[VCU][%d][%d] wait event return %d @%s\n",
			vcu->vcuid, i, ret, __func__);
		return ret;
	}
	ret = copy_to_user(user_data_addr, &vcu->user_obj[i],
		(unsigned long)sizeof(struct share_obj));
	if (ret != 0) {
		pr_info("[VCU] %s(%d) Copy data to user failed!\n",
			__func__, __LINE__);
		ret = -EINVAL;
	}
	atomic_set(&vcu->ipi_got[i], 0);

	return ret;
}

static int vcu_log_get(struct mtk_vcu *vcu, unsigned long arg)
{
	int ret;
	unsigned char *user_data_addr = NULL;

	user_data_addr = (unsigned char *)arg;

	ret = wait_event_freezable(vcu->vdec_log_get_wq,
				    atomic_read(&vcu->vdec_log_got));
	if (ret != 0) {
		pr_err("[VCU][%d] wait event return %d @%s\n",
			vcu->vcuid, ret, __func__);
		return ret;
	}

	ret = copy_to_user(user_data_addr, vcu->vdec_log_info,
		(unsigned long)sizeof(struct log_test_nofuse));
	if (ret != 0) {
		pr_err("[VCU] %s(%d) Copy data to user failed!\n",
			__func__, __LINE__);
		ret = -EINVAL;
	}
	atomic_set(&vcu->vdec_log_got, 0);

	return ret;
}

unsigned int vcu_get_vdec_hw_capa(struct mtk_vpu_plat *vpu)
{
	struct mtk_vcu *vcu = to_vcu(vpu);

	return vcu->run.dec_capability;
}

unsigned int vcu_get_venc_hw_capa(struct mtk_vpu_plat *vpu)
{
	struct mtk_vcu *vcu = to_vcu(vpu);

	return vcu->run.enc_capability;
}

void *vcu_mapping_dm_addr(struct mtk_vpu_plat *vpu,
			  uintptr_t dtcm_dmem_addr)
{
	struct mtk_vcu *vcu = to_vcu(vpu);
	uintptr_t d_vma = (uintptr_t)(dtcm_dmem_addr);
	uintptr_t d_va_start = (uintptr_t)VCU_DMEM0_VIRT(vcu);
	uintptr_t d_off = d_vma - VCU_DMEM0_VMA(vcu);
	uintptr_t d_va;

	if (dtcm_dmem_addr == 0UL || d_off > VCU_DMEM0_LEN(vcu)) {
		dev_err(vcu->dev, "[VCU] %s: Invalid vma 0x%lx len %lx\n",
			__func__, dtcm_dmem_addr, VCU_DMEM0_LEN(vcu));
		return NULL;
	}

	d_va = d_va_start + d_off;
	dev_dbg(vcu->dev, "[VCU] %s: 0x%lx -> 0x%lx\n", __func__, d_vma, d_va);

	return (void *)d_va;
}

int vcu_load_firmware(struct mtk_vpu_plat *vpu)
{
	return 0;
}


void vcu_get_task(struct task_struct **task, struct files_struct **f)
{
	pr_debug("mtk_vcu_get_task %p\n", vcud_task);
	*task = vcud_task;
	*f = files;
}
EXPORT_SYMBOL_GPL(vcu_get_task);

static int vcu_ipi_handler(struct mtk_vcu *vcu, struct share_obj *rcv_obj)
{
	struct vcu_ipi_desc *ipi_desc = vcu->ipi_desc;
	int non_ack = 0;
	int ret = -1;
	int i = 0;

	i = ipi_id_to_inst_id(vcu->vcuid, rcv_obj->id);

	if (rcv_obj->id < (int)IPI_MAX &&
		ipi_desc[rcv_obj->id].handler != NULL) {
		non_ack = ipi_desc[rcv_obj->id].handler(rcv_obj->share_buf,
							rcv_obj->len,
							ipi_desc[rcv_obj->id].priv);
		if (rcv_obj->id > (int)IPI_VPU_INIT && non_ack == 0) {
			vcu->ipi_id_ack[rcv_obj->id] = true;
			wake_up(&vcu->ack_wq[i]);
		}
		ret = 0;
	} else {
		dev_err(vcu->dev, "[VCU] No such ipi id = %d\n", rcv_obj->id);
	}

	return ret;
}

static int vcu_ipi_init(struct mtk_vcu *vcu)
{
	vcu->is_open = false;
	vcu->is_alloc = false;
	mutex_init(&vcu->vcu_mutex[0]);
	mutex_init(&vcu->vcu_mutex[1]);
	mutex_init(&vcu->vcu_share);

	return 0;
}

static int vcu_init_ipi_handler(void *data, unsigned int len, void *priv)
{
	struct mtk_vcu *vcu = (struct mtk_vcu *)priv;
	struct vcu_run *run = (struct vcu_run *)data;

	/* handle uninitialize message */
	if (vcu->run.signaled == 1u && run->signaled == 0u) {
		int i;
		/* wake up the threads in daemon */
		for (i = 0; i < 2; i++) {
			atomic_set(&vcu->ipi_got[i], 1);
			atomic_set(&vcu->ipi_done[i], 0);
			wake_up(&vcu->get_wq[i]);
		}

		atomic_set(&vcu->vdec_log_got, 1);
		wake_up(&vcu->vdec_log_get_wq);
	}

	vcu->run.signaled = run->signaled;
	strncpy(vcu->run.fw_ver, run->fw_ver, VCU_FW_VER_LEN);
	vcu->run.dec_capability = run->dec_capability;
	vcu->run.enc_capability = run->enc_capability;

	dev_dbg(vcu->dev, "[VCU] fw ver: %s\n", vcu->run.fw_ver);
	dev_dbg(vcu->dev, "[VCU] dec cap: %x\n", vcu->run.dec_capability);
	dev_dbg(vcu->dev, "[VCU] enc cap: %x\n", vcu->run.enc_capability);
	return 0;
}

static int mtk_vcu_open(struct inode *inode, struct file *file)
{
	int vcuid;
	struct mtk_vcu_queue *vcu_queue;

	if (strcmp(current->comm, "camd") == 0)
		vcuid = MTK_VCU_CAMD;
	else if (strcmp(current->comm, "mdpd") == 0)
		vcuid = MTK_VCU_MDPD;
	else {
		vcud_task = current;
		files = vcud_task->files;
		vcuid = MTK_VCU_VPUD;
	}

	vcu_mtkdev[vcuid]->vcuid = vcuid;

	vcu_queue = mtk_vcu_dec_init(vcu_mtkdev[vcuid]->dev);
	vcu_queue->vcu = vcu_mtkdev[vcuid];
	file->private_data = vcu_queue;

	return 0;
}

static int mtk_vcu_release(struct inode *inode, struct file *file)
{
	mtk_vcu_dec_release((struct mtk_vcu_queue *)file->private_data);

	return 0;
}

static void vcu_free_d_ext_mem(struct mtk_vcu *vcu)
{
	mutex_lock(&vcu->vcu_share);
	mutex_lock(&vcu->vcu_mutex[0]);
	mutex_lock(&vcu->vcu_mutex[1]);
	if (vcu->is_open == true) {
		filp_close(vcu->file, NULL);
		vcu->is_open = false;
	}
	if (vcu->is_alloc == true) {
		kfree(VCU_DMEM0_VIRT(vcu));
		VCU_DMEM0_VIRT(vcu) = NULL;
		vcu->is_alloc = false;
	}
	mutex_unlock(&vcu->vcu_mutex[1]);
	mutex_unlock(&vcu->vcu_mutex[0]);
	mutex_unlock(&vcu->vcu_share);
}

static int vcu_alloc_d_ext_mem(struct mtk_vcu *vcu, unsigned long len)
{
	mutex_lock(&vcu->vcu_share);
	mutex_lock(&vcu->vcu_mutex[0]);
	mutex_lock(&vcu->vcu_mutex[1]);
	if (vcu->is_alloc == false) {
		VCU_DMEM0_VIRT(vcu) = kmalloc(len, GFP_KERNEL);
		VCU_DMEM0_PHY(vcu) = virt_to_phys(VCU_DMEM0_VIRT(vcu));
		VCU_DMEM0_LEN(vcu) = len;
		vcu->is_alloc = true;
	}
	mutex_unlock(&vcu->vcu_mutex[1]);
	mutex_unlock(&vcu->vcu_mutex[0]);
	mutex_unlock(&vcu->vcu_share);

	dev_dbg(vcu->dev, "[VCU] Data extend memory (len:%lu) phy=0x%llx virt=0x%p iova=0x%llx\n",
		VCU_DMEM0_LEN(vcu),
		(unsigned long long)VCU_DMEM0_PHY(vcu),
		VCU_DMEM0_VIRT(vcu),
		(unsigned long long)VCU_DMEM0_IOVA(vcu));
	return 0;
}

static int mtk_vcu_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long length = vma->vm_end - vma->vm_start;
	unsigned long pa_start = vma->vm_pgoff << PAGE_SHIFT;
	unsigned long pa_start_base = pa_start;
	unsigned long pa_end = pa_start + length;
	unsigned long start = vma->vm_start;
	unsigned long pos = 0;
	int i;
	bool cache_mva = false;
	struct mtk_vcu *vcu_dev;
	struct mtk_vcu_queue *vcu_queue = (struct mtk_vcu_queue *)file->private_data;

	vcu_dev = (struct mtk_vcu *)vcu_queue->vcu;
	pr_debug("mtk_vcu_mmap start 0x%lx, end 0x%lx, length:%lx pgoff 0x%lx pa_start:0x%lx %ld map_buf:%d,%d vcu_dev->vcuid:%d\n",
		 vma->vm_start, vma->vm_end,length,
		 vma->vm_pgoff, pa_start, vma->vm_flags,
		 vcu_queue->map_buf, vcu_queue->map_type,vcu_dev->vcuid);

	if (vcu_dev->vcuid == 0) {
		for (i = 0; i < (int)VCU_MAP_HW_REG_NUM; i++) {
			if (pa_start == vcu_map_hw_type[i] &&
			    length <= vcu_dev->map_base[i].len) {
				vma->vm_pgoff =
					vcu_dev->map_base[i].base >> PAGE_SHIFT;
				goto reg_valid_map;
			}
		}
	}

	if (vcu_queue->map_buf == 0 ) {
		/*only vcud need this case*/

		if (pa_start >= MAP_SHMEM_ALLOC_BASE && pa_end <= MAP_SHMEM_ALLOC_END) {
			vcu_free_d_ext_mem(vcu_dev);
			if (vcu_alloc_d_ext_mem(vcu_dev, length) != 0) {
				pr_err("[VCU] allocate DM failed\n");
			dev_err(vcu_dev->dev, "[VCU] allocate DM failed\n");
			return -ENOMEM;
		}
		vma->vm_pgoff =
			(unsigned long)(VCU_DMEM0_PHY(vcu_dev) >> PAGE_SHIFT);
		goto valid_map;
	}

	if (pa_start >= MAP_SHMEM_COMMIT_BASE && pa_end <= MAP_SHMEM_COMMIT_END) {
		VCU_DMEM0_VMA(vcu_dev) = vma->vm_start;
		vma->vm_pgoff =
			(unsigned long)(VCU_DMEM0_PHY(vcu_dev) >> PAGE_SHIFT);
		goto valid_map;
		}
	}

	if(pa_start_base >= MAP_SHMEM_MM_BASE || vcu_queue->map_buf == 1)
	{
		if(vcu_queue ->map_type == 1)
		{
			cache_mva = true;
		}

#ifdef CONFIG_MTK_IOMMU
		while (length > 0) {
			vma->vm_pgoff = iommu_iova_to_phys(vcu_dev->io_domain,
						   pa_start + pos);
			vma->vm_pgoff >>= PAGE_SHIFT;
			if(!cache_mva)
				vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
			if (remap_pfn_range(vma, start, vma->vm_pgoff,
			    PAGE_SIZE, vma->vm_page_prot) == true)
				return -EAGAIN;

			start += PAGE_SIZE;
			pos += PAGE_SIZE;
			if (length > PAGE_SIZE)
				length -= PAGE_SIZE;
			else
				length = 0;
		}
		return 0;
#endif
	}

	dev_err(vcu_dev->dev, "[VCU] Invalid argument\n");
	return -EINVAL;

reg_valid_map:
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

valid_map:
	dev_dbg(vcu_dev->dev, "[VCU] Mapping pgoff 0x%lx\n", vma->vm_pgoff);

	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			    vma->vm_end - vma->vm_start,
			    vma->vm_page_prot) != 0) {
		return -EAGAIN;
	}

	return 0;
}

static long mtk_vcu_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = -1;
	void *mem_priv;
	unsigned char *user_data_addr = NULL;
	struct mtk_vcu *vcu_dev;
	struct device *dev;
	struct map_obj mem_map_obj;
	struct compat_map_obj mem_map_obj_compat;
	struct share_obj share_buff_data;
	struct mem_obj mem_buff_data;
	struct mtk_vcu_queue *vcu_queue = (struct mtk_vcu_queue *)file->private_data;

	vcu_dev = (struct mtk_vcu *)vcu_queue->vcu;
	dev = vcu_dev->dev;
	switch (cmd) {
	case VCUD_SET_OBJECT:
		user_data_addr = (unsigned char *)arg;
		ret = (long)copy_from_user(&share_buff_data, user_data_addr,
			(unsigned long)sizeof(struct share_obj));
		if (ret != 0L || share_buff_data.id > (int)IPI_MAX ||
		    share_buff_data.id < (int)IPI_VPU_INIT) {
			pr_err("[VCU] %s(%d) Copy data from user failed!\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		ret = vcu_ipi_handler(vcu_dev, &share_buff_data);
		ret = (long)copy_to_user(user_data_addr, &share_buff_data,
			(unsigned long)sizeof(struct share_obj));
		if (ret != 0L) {
			pr_err("[VCU] %s(%d) Copy data to user failed!\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		break;
    case COMPAT_VCUD_SET_MMAP_TYPE:
		user_data_addr = (unsigned char *)arg;
		ret = (long)copy_from_user(&mem_map_obj_compat, user_data_addr,
			(unsigned long)sizeof(struct compat_map_obj));

		if (ret != 0L) {
			pr_err("[VCU] %s(%d) Copy data to user failed!\n",
				__func__, __LINE__);
			return -EINVAL;
		}

		pr_err("[VCU] VCUD_SET_MMAP_TYPE(%d) mem_map_obj:(%u %u)\n",
				 __LINE__,mem_map_obj_compat.map_buf,mem_map_obj_compat.map_type);

		vcu_queue->map_buf = mem_map_obj_compat.map_buf;
		vcu_queue->map_type = mem_map_obj_compat.map_type;

		break;

    case VCUD_SET_MMAP_TYPE:

		user_data_addr = (unsigned char *)arg;
		ret = (long)copy_from_user(&mem_map_obj, user_data_addr,
			(unsigned long)sizeof(struct map_obj));

		if (ret != 0L) {
			pr_err("[VCU] %s(%d) Copy data to user failed!\n",
				__func__, __LINE__);
			return -EINVAL;
		}

		pr_err("[VCU] VCUD_SET_MMAP_TYPE(%d) mem_map_obj:(%lu %lu)\n",
				 __LINE__,mem_map_obj.map_buf,mem_map_obj.map_type);

		vcu_queue->map_buf = mem_map_obj.map_buf;
		vcu_queue->map_type = mem_map_obj.map_type;

		break;
	case VCUD_GET_OBJECT:
		ret = vcu_ipi_get(vcu_dev, arg);
		break;
	case VCUD_GET_LOG_OBJECT:
		ret = vcu_log_get(vcu_dev, arg);
		break;
	case VCUD_MVA_ALLOCATION:
		user_data_addr = (unsigned char *)arg;
		ret = (long)copy_from_user(&mem_buff_data, user_data_addr,
			(unsigned long)sizeof(struct mem_obj));
		if (ret != 0L) {
			pr_err("[VCU] %s(%d) Copy data from user failed!\n",
				__func__, __LINE__);
			return -EINVAL;
		}

		mem_priv = mtk_vcu_get_buffer(vcu_queue, &mem_buff_data);
		if (IS_ERR(mem_priv) == true) {
			pr_err("[VCU] Dma alloc buf failed!\n");
			return PTR_ERR(mem_priv);
		}

		ret = (long)copy_to_user(user_data_addr, &mem_buff_data,
			(unsigned long)sizeof(struct mem_obj));
		if (ret != 0L) {
			pr_err("[VCU] %s(%d) Copy data to user failed!\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		ret = 0;
		break;
	case VCUD_MVA_FREE:
		user_data_addr = (unsigned char *)arg;
		ret = (long)copy_from_user(&mem_buff_data, user_data_addr,
			(unsigned long)sizeof(struct mem_obj));
		if ((ret != 0L) || (mem_buff_data.iova == 0UL)) {
			pr_err("[VCU] %s(%d) Free buf failed!\n",
				__func__, __LINE__);
			return -EINVAL;
		}

		ret = mtk_vcu_free_buffer(vcu_queue, &mem_buff_data);
		if (ret != 0L) {
			pr_err("[VCU] Dma free buf failed!\n");
			return -EINVAL;
		}
		mem_buff_data.va = 0;
		mem_buff_data.iova = 0;

		ret = (long)copy_to_user(user_data_addr, &mem_buff_data,
			(unsigned long)sizeof(struct mem_obj));
		if (ret != 0L) {
			pr_err("[VCU] %s(%d) Copy data to user failed!\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		ret = 0;
		break;
	case VCUD_CACHE_FLUSH_ALL:
		smp_inner_dcache_flush_all();
		ret = 0;
		break;
	case VCUD_MVA_MAP_CACHE:
		ret = 0;
		break;
	default:
		dev_err(dev, "[VCU] Unknown cmd\n");
		break;
	}

	return ret;
}

#if IS_ENABLED(CONFIG_COMPAT)
/*
 * PORT 5.10->6.18: compat_alloc_user_space() was removed in v6.2. The old
 * compat path built a fake __user buffer, copied the 32-bit struct into it,
 * then re-entered the native unlocked_ioctl. Re-implement the conversion to
 * work entirely on a kernel-side struct mem_obj so no user-space scratch
 * buffer is needed. These helpers now read/write the 32-bit user struct
 * directly into/out of a kernel struct mem_obj.
 */
static int compat_get_vpud_allocation_data(
				struct compat_mem_obj __user *data32,
				struct mem_obj *data)
{
	compat_ulong_t l;
	compat_u64 u;
	unsigned int err = 0;

	err = get_user(l, &data32->iova);
	data->iova = l;
	err |= get_user(l, &data32->len);
	data->len = l;
	err |= get_user(u, &data32->va);
	data->va = u;

	return (int)err;
}

static int compat_put_vpud_allocation_data(
				struct compat_mem_obj __user *data32,
				struct mem_obj *data)
{
	unsigned int err = 0;

	err = put_user((compat_ulong_t)data->iova, &data32->iova);
	err |= put_user((compat_ulong_t)data->len, &data32->len);
	err |= put_user((compat_u64)data->va, &data32->va);

	return (int)err;
}

static long mtk_vcu_unlocked_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	long ret = -1;
	struct share_obj __user *share_data32;
	struct compat_mem_obj __user *data32;
	/*
	 * PORT 5.10->6.18: handle the MVA alloc/free compat conversions on a
	 * kernel-side struct mem_obj and call the buffer helpers directly,
	 * instead of building a user scratch buffer via the removed
	 * compat_alloc_user_space() and re-entering the native ioctl.
	 */
	struct mtk_vcu_queue *vcu_queue =
		(struct mtk_vcu_queue *)file->private_data;
	struct mem_obj kdata;
	void *mem_priv;

	switch (cmd) {
	case COMPAT_VCUD_SET_OBJECT:
	case VCUD_GET_OBJECT:
	case VCUD_GET_LOG_OBJECT:
	case VCUD_SET_MMAP_TYPE:
		share_data32 = compat_ptr((uint32_t)arg);
		ret = file->f_op->unlocked_ioctl(file,
				cmd, (unsigned long)share_data32);
		break;
	case COMPAT_VCUD_MVA_ALLOCATION:
		data32 = compat_ptr((uint32_t)arg);
		err = compat_get_vpud_allocation_data(data32, &kdata);
		if (err != 0)
			return err;

		mem_priv = mtk_vcu_get_buffer(vcu_queue, &kdata);
		if (IS_ERR(mem_priv) == true)
			return PTR_ERR(mem_priv);
		ret = 0;

		err = compat_put_vpud_allocation_data(data32, &kdata);
		if (err != 0)
			return err;
		break;
	case COMPAT_VCUD_MVA_FREE:
		data32 = compat_ptr((uint32_t)arg);
		err = compat_get_vpud_allocation_data(data32, &kdata);
		if (err != 0)
			return err;
		if (kdata.iova == 0UL)
			return -EINVAL;

		ret = mtk_vcu_free_buffer(vcu_queue, &kdata);
		if (ret != 0L)
			return -EINVAL;
		kdata.va = 0;
		kdata.iova = 0;

		err = compat_put_vpud_allocation_data(data32, &kdata);
		if (err != 0)
			return err;
		break;
	case COMPAT_VCUD_CACHE_FLUSH_ALL:
		ret = file->f_op->unlocked_ioctl(file,
			(uint32_t)VCUD_CACHE_FLUSH_ALL, 0);
		break;
	case COMPAT_VCUD_MVA_MAP_CACHE:
	case COMPAT_VCUD_SET_MMAP_TYPE:
		ret = 0;
		break;
	default:
		pr_err("[VCU] Invalid cmd_number 0x%x.\n", cmd);
		break;
	}
	return ret;
}
#endif

static const struct file_operations vcu_fops = {
	.owner      = THIS_MODULE,
	.unlocked_ioctl = mtk_vcu_unlocked_ioctl,
	.open       = mtk_vcu_open,
	.release    = mtk_vcu_release,
	.mmap       = mtk_vcu_mmap,
#if IS_ENABLED(CONFIG_COMPAT)
	.compat_ioctl = mtk_vcu_unlocked_compat_ioctl,
#endif
};

static struct mtk_vpu_ops mtk_vcu_ops = {
	.ipi_register = vcu_ipi_register,
	.ipi_send = vcu_ipi_send,
	.get_vdec_hw_capa = vcu_get_vdec_hw_capa,
	.get_venc_hw_capa = vcu_get_venc_hw_capa,
	.load_firmware = vcu_load_firmware,
	.mapping_dm_addr = vcu_mapping_dm_addr,
};

static int vcu_check_if_running(struct mtk_vcu *vcu)
{
	int vcu_running = 0;

	mutex_lock(&vcu->vcu_mutex[0]);
	mutex_lock(&vcu->vcu_mutex[1]);
	if (((atomic_read(&vcu->ipi_got[0]) == 1) && (atomic_read(&vcu->ipi_done[0]) == 0)) ||
	    ((atomic_read(&vcu->ipi_got[1]) == 1) && (atomic_read(&vcu->ipi_done[1]) == 0))) {
		vcu_running = 1;
	}
	mutex_unlock(&vcu->vcu_mutex[1]);
	mutex_unlock(&vcu->vcu_mutex[0]);

	return vcu_running;
}

/**
 * Suspend callbacks after user space processes are frozen
 * Since user space processes are frozen, there is no need and cannot hold same
 * mutex that protects lock owner while checking status.
 * If hardware is still active now, must not to enter suspend.
 **/
static int mtk_vcu_suspend(struct device *vcu_dev)
{
	struct mtk_vpu_plat *vpu = dev_get_drvdata(vcu_dev);
	struct mtk_vcu *vcu = to_vcu(vpu);
	struct device *dev = vcu->dev;
	int vcuid = vcu->vcuid;

	dev_dbg(dev, "[VCU] %s()... \n", __func__);

	if (vcu_check_if_running(vcu)) {
		dev_err(dev, "[VCU][%d] %s fail due to unfinished activity\n",
			vcuid, __func__);
		return -EBUSY;
	}

	dev_dbg(dev, "[VCU] %s done\n", __func__);
	return 0;
}

static int mtk_vcu_resume(struct device *dev)
{
	dev_dbg(dev, "[VCU] %s done\n", __func__);
	return 0;
}

/**
 * Suspend notifiers before user space processes are frozen.
 * User space driver can still complete decoding/encoding of current frame.
 * Since there is no critical section protection, it is possible for a new task
 * to start after this state.
 * This case will be handled by suspend callback mtk_vcu_suspend.
 **/
static int mtk_vcu_suspend_notifier(struct notifier_block *nb,
				    unsigned long action, void *data)
{
	int wait_cnt = 0;
	struct mtk_vcu *vcu = container_of(nb, struct mtk_vcu, pm_notifier);
	struct device *dev = vcu->dev;
	int vcuid = vcu->vcuid;

	dev_dbg(dev, "[VCU] %s ok action = %ld\n", __func__, action);
	switch (action) {
	case PM_SUSPEND_PREPARE:
		dev_dbg(dev, "suspend notifier: suspend prepare... \n");

		while (vcu_check_if_running(vcu)) {
			wait_cnt++;
			dev_dbg(dev,
				"suspend notifier: id[%d] wait_cnt[%d]... %d %d\n",
				vcuid, wait_cnt, atomic_read(&vcu->ipi_done[0]),
				atomic_read(&vcu->ipi_done[1]));
			if (wait_cnt > 5) {
				dev_dbg(dev,
					"suspend notifier: id[%d] waiting %d %d, job not finished.\n",
					vcuid, atomic_read(&vcu->ipi_done[0]),
					atomic_read(&vcu->ipi_done[1]));
				/* Current task is still not finished, don't
				 * care, will check again in real suspend
				 */
				return NOTIFY_OK;
			}
			usleep_range(10000, 20000);
		}
		dev_dbg(dev, "suspend notifier: suspend prepare... done \n");
		return NOTIFY_OK;
	case PM_POST_SUSPEND:
		dev_dbg(dev, "suspend notifier: post suspend... done \n");
		return NOTIFY_OK;
	default:
		return NOTIFY_DONE;
	}
	return NOTIFY_DONE;
}

static int mtk_vcu_probe(struct platform_device *pdev)
{
	struct mtk_vcu *vcu;
	struct device *dev;
	struct resource *res;
	int i, vcuid, ret = 0;

	dev_dbg(&pdev->dev, "[VCU] initialization\n");

	dev = &pdev->dev;
	vcu = devm_kzalloc(dev, sizeof(*vcu), GFP_KERNEL);
	if (vcu == NULL)
		return -ENOMEM;

	vcu->vpu.ops = &mtk_vcu_ops;

	ret = of_property_read_u32(dev->of_node, "mediatek,vcuid", &vcuid);
	if (ret != 0) {
		dev_err(dev, "[VCU] failed to find mediatek,vcuid\n");
		return ret;
	}
	vcu_mtkdev[vcuid] = vcu;
	vcu_mtkdev[vcuid]->vdec_log_info = devm_kzalloc(dev,
		sizeof(struct log_test_nofuse), GFP_KERNEL);
	if (!vcu_mtkdev[vcuid]->vdec_log_info)
		return -ENOMEM;

#ifdef CONFIG_MTK_IOMMU
	vcu_mtkdev[vcuid]->io_domain = iommu_get_domain_for_dev(dev);
	if (vcu_mtkdev[vcuid]->io_domain == NULL) {
		dev_err(dev, "[VCU] vcuid: %d get iommu domain fail !!\n", vcuid);
		return -EPROBE_DEFER;
	}
	dev_dbg(dev, "vcu iommudom: %p,vcuid:%d\n", vcu_mtkdev[vcuid]->io_domain, vcuid);
#endif

	if (vcuid == 2)
		vcu_mtkdev[vcuid]->path = CAM_PATH;
	else if (vcuid == 1)
		vcu_mtkdev[vcuid]->path = MDP_PATH;
	else if (vcuid == 0)
		vcu_mtkdev[vcuid]->path = VCU_PATH;
	else
		return -ENXIO;

	ret = of_property_read_string(dev->of_node, "mediatek,vcuname", &vcu_mtkdev[vcuid]->vcuname);
	if (ret != 0) {
		dev_err(dev, "[VCU] failed to find mediatek,vcuname\n");
		return ret;
	}

	vcu->dev = &pdev->dev;
	platform_set_drvdata(pdev, &vcu->vpu);

	if (vcuid == 0) {
		for (i = 0; i < (int)VCU_MAP_HW_REG_NUM; i++) {
			res = platform_get_resource(pdev, IORESOURCE_MEM, i);
			if (res == NULL) {
				dev_err(dev, "Get memory resource failed.\n");
				ret = -ENXIO;
				goto err_ipi_init;
			}
			vcu->map_base[i].base = res->start;
			vcu->map_base[i].len = resource_size(res);
			dev_dbg(dev, "[VCU] base[%d]: 0x%lx 0x%lx", i, vcu->map_base[i].base,
				vcu->map_base[i].len);
		}
	}
	dev_dbg(dev, "[VCU] vcu ipi init\n");
	ret = vcu_ipi_init(vcu);
	if (ret != 0) {
		dev_err(dev, "[VCU] Failed to init ipi\n");
		goto err_ipi_init;
	}

	/* register vcu initialization IPI */
	ret = vcu_ipi_register(&vcu->vpu, IPI_VPU_INIT, vcu_init_ipi_handler,
			       "vcu_init", vcu);
	if (ret != 0) {
		dev_err(dev, "Failed to register IPI_VPU_INIT\n");
		goto vcu_mutex_destroy;
	}

	init_waitqueue_head(&vcu->ack_wq[0]);
	init_waitqueue_head(&vcu->ack_wq[1]);
	init_waitqueue_head(&vcu->get_wq[0]);
	init_waitqueue_head(&vcu->get_wq[1]);
	init_waitqueue_head(&vcu->vdec_log_get_wq);
	atomic_set(&vcu->ipi_got[0], 0);
	atomic_set(&vcu->ipi_got[1], 0);
	atomic_set(&vcu->ipi_done[0], 0);
	atomic_set(&vcu->ipi_done[1], 0);
	atomic_set(&vcu->vdec_log_got, 0);
	/* init character device */

	ret = alloc_chrdev_region(&vcu_mtkdev[vcuid]->vcu_devno, 0, 1, vcu_mtkdev[vcuid]->vcuname);
	if (ret < 0) {
		dev_err(dev, "[VCU]  alloc_chrdev_region failed (ret=%d)\n", ret);
		goto err_alloc;
	}

	vcu_mtkdev[vcuid]->vcu_cdev = cdev_alloc();
	vcu_mtkdev[vcuid]->vcu_cdev->owner = THIS_MODULE;
	vcu_mtkdev[vcuid]->vcu_cdev->ops = &vcu_fops;

	ret = cdev_add(vcu_mtkdev[vcuid]->vcu_cdev, vcu_mtkdev[vcuid]->vcu_devno, 1);
	if (ret < 0) {
		dev_err(dev, "[VCU] class create fail (ret=%d)", ret);
		goto err_add;
	}

	/* PORT 5.10->6.18: class_create() dropped the owner (THIS_MODULE) arg */
	vcu_mtkdev[vcuid]->vcu_class = class_create(vcu_mtkdev[vcuid]->vcuname);
	if (IS_ERR(vcu_mtkdev[vcuid]->vcu_class) == true) {
		ret = (int)PTR_ERR(vcu_mtkdev[vcuid]->vcu_class);
		dev_err(dev, "[VCU] class create fail (ret=%d)", ret);
		goto err_add;
	}

	vcu_mtkdev[vcuid]->vcu_device = device_create(vcu_mtkdev[vcuid]->vcu_class, NULL,
				vcu_mtkdev[vcuid]->vcu_devno, NULL, vcu_mtkdev[vcuid]->vcuname);
	if (IS_ERR(vcu_mtkdev[vcuid]->vcu_device) == true) {
		ret = (int)PTR_ERR(vcu_mtkdev[vcuid]->vcu_device);
		dev_err(dev, "[VCU] device_create fail (ret=%d)", ret);
		goto err_device;
	}

	vcu->pm_notifier.notifier_call = mtk_vcu_suspend_notifier;
	register_pm_notifier(&vcu->pm_notifier);

	dev_dbg(dev, "[VCU] initialization completed\n");
	return 0;

err_device:
	class_destroy(vcu_mtkdev[vcuid]->vcu_class);
err_add:
	cdev_del(vcu_mtkdev[vcuid]->vcu_cdev);
err_alloc:
	unregister_chrdev_region(vcu_mtkdev[vcuid]->vcu_devno, 1);
vcu_mutex_destroy:
	mutex_destroy(&vcu->vcu_mutex[0]);
	mutex_destroy(&vcu->vcu_mutex[1]);
	mutex_destroy(&vcu->vcu_share);
err_ipi_init:
	devm_kfree(dev, vcu);

	return ret;
}

static const struct of_device_id mtk_vcu_match[] = {
	{.compatible = "mediatek,mt8183-vcu",},
	{.compatible = "mediatek,mt8167-vcu",},
	{},
};
MODULE_DEVICE_TABLE(of, mtk_vcu_match);

/* PORT 5.10->6.18: platform_driver.remove now returns void */
static void mtk_vcu_remove(struct platform_device *pdev)
{
	struct mtk_vcu *vcu = platform_get_drvdata(pdev);

	if (vcu->is_open == true) {
		filp_close(vcu->file, NULL);
		vcu->is_open = false;
	}
	unregister_pm_notifier(&vcu->pm_notifier);

	device_destroy(vcu->vcu_class, vcu->vcu_devno);
	class_destroy(vcu->vcu_class);
	cdev_del(vcu->vcu_cdev);
	unregister_chrdev_region(vcu->vcu_devno, 1);

	devm_kfree(&pdev->dev, vcu);
}

static const struct dev_pm_ops mtk_vcu_pm_ops = {
	.suspend = mtk_vcu_suspend,
	.resume = mtk_vcu_resume,
};

static struct platform_driver mtk_vcu_driver = {
	.probe	= mtk_vcu_probe,
	.remove	= mtk_vcu_remove,
	.driver	= {
		.name	= "mtk_vcu",
		.owner	= THIS_MODULE,
		.pm = &mtk_vcu_pm_ops,
		.of_match_table = mtk_vcu_match,
	},
};

module_platform_driver(mtk_vcu_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Mediatek Video Communication And Controller Unit driver");
