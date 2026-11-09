// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2016 MediaTek Inc.
 * Author: Andrew-CT Chen <andrew-ct.chen@mediatek.com>
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
#include <linux/list.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/sched.h>
#include <linux/sched/signal.h>
#include <linux/sched/task.h>
#include <linux/semaphore.h>
#include <linux/suspend.h>
#include <linux/uaccess.h>
#include <linux/compat.h>
#include <linux/freezer.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/pm_wakeup.h>
#include <linux/mailbox_controller.h>
#include <linux/signal.h>
#include <trace/events/signal.h>
#include <linux/string.h>

#if IS_ENABLED(CONFIG_MTK_IOMMU)
#include <linux/iommu.h>
#endif
#if IS_ENABLED(CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT)
#include "mailbox/cmdq-sec.h"
#endif
#include <mtk_vcu_controls.h>
#include <mtk_vcu_camif.h>
#include "mtk_vcu.h"
//#include "vcp_helper.h"
#include <linux/soc/mediatek/mtk-cmdq.h>
#include <linux/mailbox/mtk-cmdq-mailbox.h>

/**
 * VCU (Video Communication/Controller Unit) is a tiny processor
 * controlling video hardware related to video codec, scaling and color
 * format converting.
 * VCU interfaces with other blocks by share memory and interrupt.
 **/
#define VCU_PATH                "/dev/vpud"
#define MDP_PATH                "/dev/mdpd"
#define CAM_PATH                "/dev/camd"
#define VCU_DEVNAME             "vcu"

#ifdef CONFIG_MTK_ENG_BUILD
#define IPI_TIMEOUT_MS          16000U
#else
#define IPI_TIMEOUT_MS          5000U
#endif

#define VCU_FW_VER_LEN          16
#define VCODEC_INST_MAX         64
#define GCE_EVENT_MAX           64
#define GCE_THNUM_MAX           2
#define GCE_PENDING_CNT         16
/*mtk vcu support mpd max value*/
#define MTK_VCU_NR_MAX       3

/* vcu extended mapping length */
#define VCU_PMEM0_LEN(vcu_data) (vcu_data->extmem.p_len)
#define VCU_DMEM0_LEN(vcu_data) (vcu_data->extmem.d_len)
/* vcu extended user virtural address */
#define VCU_PMEM0_VMA(vcu_data) (vcu_data->extmem.p_vma)
#define VCU_DMEM0_VMA(vcu_data) (vcu_data->extmem.d_vma)
/* vcu extended kernel virtural address */
#define VCU_PMEM0_VIRT(vcu_data)        (vcu_data->extmem.p_va)
#define VCU_DMEM0_VIRT(vcu_data)        (vcu_data->extmem.d_va)
/* vcu extended phsyial address */
#define VCU_PMEM0_PHY(vcu_data) (vcu_data->extmem.p_pa)
#define VCU_DMEM0_PHY(vcu_data) (vcu_data->extmem.d_pa)
/* vcu extended iova address*/
#define VCU_PMEM0_IOVA(vcu_data)        (vcu_data->extmem.p_iova)
#define VCU_DMEM0_IOVA(vcu_data)        (vcu_data->extmem.d_iova)
#define VCU_SHMEM_SIZE 0x300000

#define MAP_SHMEM_ALLOC_BASE    0x80000000UL
#define MAP_SHMEM_ALLOC_RANGE   VCU_SHMEM_SIZE
#define MAP_SHMEM_ALLOC_END     (MAP_SHMEM_ALLOC_BASE + MAP_SHMEM_ALLOC_RANGE)
#define MAP_SHMEM_COMMIT_BASE   0x88000000UL
#define MAP_SHMEM_COMMIT_RANGE  VCU_SHMEM_SIZE
#define MAP_SHMEM_COMMIT_END    (MAP_SHMEM_COMMIT_BASE + MAP_SHMEM_COMMIT_RANGE)

#define MAP_SHMEM_MM_BASE       0x2000000000UL
#define MAP_SHMEM_MM_CACHEABLE_BASE     0x4000000000UL
#define MAP_SHMEM_PA_BASE       0x8000000000UL
#define MAP_SHMEM_MM_RANGE      0x1FFFFFFFFFUL
#define MAP_SHMEM_MM_END        (MAP_SHMEM_MM_BASE + MAP_SHMEM_MM_RANGE)
#define MAP_SHMEM_MM_CACHEABLE_END (MAP_SHMEM_MM_CACHEABLE_BASE \
+ MAP_SHMEM_MM_RANGE)

static char *vcodec_param_string = "";

inline unsigned int ipi_id_to_inst_id(int id)
{
	if (id == IPI_VDEC_RESOURCE)
		return VCU_RESOURCE;

	if (id < IPI_VENC_COMMON && id >= IPI_VCU_INIT)
		return VCU_VDEC;
	else
		return VCU_VENC;
}

/* for vcu dbg log*/
int enable_vcu_dbg_log;

#define vcu_dbg_log(fmt, arg...) do { \
		if (enable_vcu_dbg_log) \
			pr_info(fmt, ##arg); \
	} while (0)


#undef CONFIG_COMPAT /* TODO: undefined for not fixing compat_alloc_user_space() related code */

#undef pr_debug
#define pr_debug vcu_dbg_log

#define MAP_PA_BASE_1GB  0x40000000 /* < 1GB registers */
#define VCU_MAP_HW_REG_NUM 5
/* VDEC VDEC_LAT VENC_CORE0 VENC_CORE1 */

/* Default vcu_mtkdev[0] handle vdec, vcu_mtkdev[1] handle mdp */
static struct mtk_vcu *vcu_mtkdev[MTK_VCU_NR_MAX];

static __attribute__((used)) unsigned int time_ms_s, time_ms_e;
#define time_check_start() { \
		time_ms_s = jiffies_to_msecs(jiffies); \
	}
#define time_check_end(timeout_ms, debug) do { \
		time_ms_e = jiffies_to_msecs(jiffies); \
		if ((time_ms_e - time_ms_s) > timeout_ms) \
			pr_info("[VCU][Info] %s L:%d take %u timeout %u ms", \
				__func__, __LINE__, \
				time_ms_e - time_ms_s, \
				timeout_ms); \
	} while (0)

/**
 * struct vcu_mem - VCU memory information
 *
 * @p_vma:      the user virtual memory address of
 *              VCU extended program memory
 * @d_vma:      the user  virtual memory address of VCU extended data memory
 * @p_va:       the kernel virtual memory address of
 *              VCU extended program memory
 * @d_va:       the kernel virtual memory address of VCU extended data memory
 * @p_pa:       the physical memory address of VCU extended program memory
 * @d_pa:       the physical memory address of VCU extended data memory
 * @p_iova:     the iova memory address of VCU extended program memory
 * @d_iova:     the iova memory address of VCU extended data memory
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
 * @signaled:           the signal of vcu initialization completed
 * @fw_ver:             VCU firmware version
 * @dec_capability:     decoder capability which is not used for now and
 *                      the value is reserved for future use
 * @enc_capability:     encoder capability which is not used for now and
 *                      the value is reserved for future use
 * @wq:                 wait queue for VCU initialization status
 */
struct vcu_run {
	u32 signaled;
	char fw_ver[VCU_FW_VER_LEN];
	unsigned int    dec_capability;
	unsigned int    enc_capability;
	wait_queue_head_t wq;
};

/**
 * struct vcu_ipi_desc - VCU IPI descriptor
 *
 * @handler:    IPI handler
 * @name:       the name of IPI handler
 * @priv:       the private data of IPI handler
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
#if ENABLE_GCE
struct gce_callback_data {
	struct gce_cmdq_obj cmdq_buff;
	struct mtk_vcu *vcu_ptr;
	struct cmdq_pkt pkt;
	struct mtk_vcu_queue *vcu_queue;
	int sta;
	struct work_struct gce_callback_thread_work;
};

struct gce_ctx_info {
	void *v4l2_ctx;
	u64 user_hdl;
	atomic_t flush_done[GCE_THNUM_MAX];
	/* gce callbacked but user not waited cnt */
	struct gce_callback_data buff[GCE_PENDING_CNT];
	struct semaphore buff_sem[GCE_PENDING_CNT];
	atomic_t flush_pending;
	/* gce not callbacked cnt */
	struct vcu_page_info used_pages[GCE_PENDING_CNT];
};

struct vcodec_gce_event {
	enum gce_event_id id;
	const char *name;
};

static const struct vcodec_gce_event vcodec_gce_event_mapping_table[] = {
#if 0 //vdec not used gce now
	{.id = VDEC_EVENT_0, .name = "vdec_pic_start"},
	{.id = VDEC_EVENT_1, .name = "vdec_decode_done"},
	{.id = VDEC_EVENT_2, .name = "vdec_pause"},
	{.id = VDEC_EVENT_3, .name = "vdec_dec_error"},
	{.id = VDEC_EVENT_4, .name = "vdec_mc_busy_overflow_timeout"},
	{.id = VDEC_EVENT_5, .name = "vdec_all_dram_req_done"},
	{.id = VDEC_EVENT_6, .name = "vdec_ini_fetch_rdy"},
	{.id = VDEC_EVENT_7, .name = "vdec_process_flag"},
	{.id = VDEC_EVENT_8, .name = "vdec_search_start_code_done"},
	{.id = VDEC_EVENT_9, .name = "vdec_ref_reorder_done"},
	{.id = VDEC_EVENT_10, .name = "vdec_wp_tble_done"},
	{.id = VDEC_EVENT_11, .name = "vdec_count_sram_clr_done"},
	{.id = VDEC_EVENT_15, .name = "vdec_gce_cnt_op_threshold"},
	{.id = VDEC_LAT_EVENT_0, .name = "vdec_lat_pic_start"},
	{.id = VDEC_LAT_EVENT_1, .name = "vdec_lat_decode_done"},
	{.id = VDEC_LAT_EVENT_2, .name = "vdec_lat_pause"},
	{.id = VDEC_LAT_EVENT_3, .name = "vdec_lat_dec_error"},
	{.id = VDEC_LAT_EVENT_4, .name = "vdec_lat_mc_busy_overflow_timeout"},
	{.id = VDEC_LAT_EVENT_5, .name = "vdec_lat_all_dram_req_done"},
	{.id = VDEC_LAT_EVENT_6, .name = "vdec_lat_ini_fetch_rdy"},
	{.id = VDEC_LAT_EVENT_7, .name = "vdec_lat_process_flag"},
	{.id = VDEC_LAT_EVENT_8, .name = "vdec_lat_search_start_code_done"},
	{.id = VDEC_LAT_EVENT_9, .name = "vdec_lat_ref_reorder_done"},
	{.id = VDEC_LAT_EVENT_10, .name = "vdec_lat_wp_tble_done"},
	{.id = VDEC_LAT_EVENT_11, .name = "vdec_lat_count_sram_clr_done"},
	{.id = VDEC_LAT_EVENT_15, .name = "vdec_lat_gce_cnt_op_threshold"},
#endif
	{.id = VENC_EOF, .name = "venc_eof"},
	{.id = VENC_CMDQ_PAUSE_DONE, .name = "venc_cmdq_pause_done"},
	{.id = VENC_MB_DONE, .name = "venc_mb_done"},
	{.id = VENC_128BYTE_CNT_DONE, .name = "venc_128B_cnt_done"},
	{.id = VENC_EOF_C1, .name = "venc_eof_c1"},
	{.id = VENC_WP_2ND_DONE, .name = "venc_wp_2nd_done"},
	{.id = VENC_WP_3ND_DONE, .name = "venc_wp_3nd_done"},
	{.id = VENC_SPS_DONE, .name = "venc_sps_done"},
	{.id = VENC_PPS_DONE, .name = "venc_pps_done"},
	{.id = VENC_SLC_EOF, .name = "venc_slc_eof"},
};

#if ENABLE_GCE
static void vcu_gce_timeout_callback(struct gce_callback_data * data);
static int vcu_gce_get_inst_id(struct mtk_vcu *vcu, u64 gce_handle, u32 core_id);
static void vcu_cmdq_pkt_destroy(struct cmdq_pkt *pkt);
#endif

static int find_gce_event_id(const char *gce_event_name)
{
	int i;
	int total_count =
		sizeof(vcodec_gce_event_mapping_table) / sizeof(struct vcodec_gce_event);
	for (i = 0; i < total_count; i++) {
		if (!strcmp(vcodec_gce_event_mapping_table[i].name, gce_event_name))
			return vcodec_gce_event_mapping_table[i].id;
	}
	return -1;
}
#endif
/**
 * struct mtk_vcu - vcu driver data
 * @extmem:             VCU extended memory information
 * @run:                VCU initialization status
 * @ipi_desc:           VCU IPI descriptor
 * @dev:                VCU struct device
 * @vcu_mutex:          protect mtk_vcu (except recv_buf) and ensure only
 *                      one client to use VCU service at a time. For example,
 *                      suppose a client is using VCU to decode VP8.
 *                      If the other client wants to encode VP8,
 *                      it has to wait until VP8 decode completes.
 * @vcu_gce_mutex       protect mtk_vcu gce flush & callback power sequence
 * @file:               VCU daemon file pointer
 * @is_open:            The flag to indicate if VCUD device is open.
 * @ack_wq:             The wait queue for each codec and mdp. When sleeping
 *                      processes wake up, they will check the condition
 *                      "ipi_id_ack" to run the corresponding action or
 *                      go back to sleep.
 * @ipi_id_ack:         The ACKs for registered IPI function sending
 *                      interrupt to VCU
 * @get_wq:             When sleeping process waking up, it will check the
 *                      condition "ipi_got" to run the corresponding action or
 *                      go back to sleep.
 * @ipi_got:            The flags for IPI message polling from user.
 * @ipi_done:           The flags for IPI message polling from user again, which
 *                      means the previous messages has been dispatched done in
 *                      daemon.
 * @user_obj:           Temporary share_obj used for ipi_msg_get.
 * @vcu_devno:          The vcu_devno for vcu init vcu character device
 * @vcu_cdev:           The point of vcu character device.
 * @vcu_class:          The class_create for create vcu device
 * @vcu_device:         VCU struct device
 * @vcuname:            VCU struct device name in dtsi
 * @path:               The path to keep mdpd path or vcud path.
 * @vpuid:              VCU device id
 *
 */
struct mtk_vcu {
	struct vcu_mem extmem;
	struct vcu_run run;
	struct vcu_ipi_desc ipi_desc[IPI_MAX];
	struct device *dev;
	struct device *dev_io_enc;
	struct mutex vcu_mutex[VCU_CODEC_MAX];
	struct mutex vcu_gce_mutex[VCU_CODEC_MAX];
	struct mutex ctx_ipi_binding[VCU_CODEC_MAX];
	/* for protecting vcu data structure */
	struct mutex vcu_share;
	struct file *file;
	struct iommu_domain *io_domain;
	struct iommu_domain *io_domain_enc;
	bool   iommu_padding;
	/* temp for 33bits larb adding bits "1" iommu */
	struct map_hw_reg map_base[VCU_MAP_HW_REG_NUM];
	bool   is_open;
	wait_queue_head_t ack_wq[VCU_CODEC_MAX];
	bool ipi_id_ack[IPI_MAX];
	wait_queue_head_t get_wq[VCU_CODEC_MAX];
	atomic_t ipi_got[VCU_CODEC_MAX];
	atomic_t ipi_done[VCU_CODEC_MAX];
	struct share_obj user_obj[VCU_CODEC_MAX];
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
	struct cmdq_client *clt_vdec[GCE_THNUM_MAX];
	struct cmdq_client *clt_venc[GCE_THNUM_MAX];
	struct cmdq_client *clt_venc_sec[GCE_THNUM_MAX];
	int gce_th_num[VCU_CODEC_MAX];
	int gce_codec_eid[GCE_EVENT_MAX];
	struct gce_cmds *gce_cmds[VCU_CODEC_MAX];
	struct mutex gce_cmds_mutex[VCU_CODEC_MAX];
	void *curr_ctx[VCU_CODEC_MAX];
	struct vb2_buffer *curr_src_vb[VCU_CODEC_MAX];
	struct vb2_buffer *curr_dst_vb[VCU_CODEC_MAX];
#if ENABLE_GCE
	wait_queue_head_t gce_wq[VCU_CODEC_MAX][GCE_THNUM_MAX];
	struct gce_ctx_info gce_info[VCODEC_INST_MAX];
#endif
	atomic_t gce_job_cnt[VCU_CODEC_MAX][GCE_THNUM_MAX];
	struct vcu_v4l2_callback_func cbf;
	unsigned long flags[VCU_CODEC_MAX];
	atomic_t open_cnt;
	/* VDEC0/VDEC1/VENC, held while /dev/vcu is open (see mtk_vcu_open) */
	struct dev_pm_domain_list *pd_list;
	struct mutex vcu_dev_mutex;
	bool abort;
	struct semaphore vpud_killed;
	bool is_entering_suspend;
	u32 gce_gpr[GCE_THNUM_MAX];
	/* for gce poll timer, multi-thread sync */

	/* for vpud sig check */
	spinlock_t vpud_sig_lock;
	int vpud_is_going_down;

	struct notifier_block pm_notifier;

	/* for vcu dbg log*/
	int enable_vcu_dbg_log;
	/* process gce callback */
	struct workqueue_struct *gce_callback_thread_wq[GCE_THNUM_MAX];

	struct task_struct *vcud_task;
	/* for protecting vpud file struct */
	struct mutex vpud_task_mutex;
};

struct node_iova_t {
	uintptr_t wdma_dma_buf_addr;
	dma_addr_t mapped_iova;
	struct dma_buf_attachment *buf_att;
	struct sg_table *sgt;
	struct node_iova_t *next;
};

struct node_list_t {
	struct node_iova_t *first_node;
	struct node_iova_t *last_node;
	struct mutex node_iova_lock;
};

static struct node_list_t iova_node_list;

static void remove_all_iova_node(struct mtk_vcu *vcu)
{
	struct node_iova_t *curr_node, *next_node;
	struct dma_buf *display_dma_buf;

	vcu_dbg_log("[VCU] %s +++", __func__);

	mutex_lock(&iova_node_list.node_iova_lock);
	curr_node = iova_node_list.first_node;
	if (curr_node == NULL) {
		mutex_unlock(&iova_node_list.node_iova_lock);
		return;
	}
	next_node = curr_node->next;

	// detach
	display_dma_buf = (struct dma_buf *)(uintptr_t)curr_node->wdma_dma_buf_addr;
	dma_buf_unmap_attachment(curr_node->buf_att, curr_node->sgt, DMA_TO_DEVICE);
	dma_buf_detach(display_dma_buf, curr_node->buf_att);

	kfree(curr_node);

	while (next_node != NULL) {
		curr_node = next_node;
		next_node = next_node->next;

		// detach
		display_dma_buf = (struct dma_buf *)(uintptr_t)curr_node->wdma_dma_buf_addr;
		dma_buf_unmap_attachment(curr_node->buf_att, curr_node->sgt, DMA_TO_DEVICE);
		dma_buf_detach(display_dma_buf, curr_node->buf_att);

		vcu_dbg_log("[VCU] free node with mapped_iova:x%lx wdma_dma_buf_addr:0x%lx",
			curr_node->mapped_iova, curr_node->wdma_dma_buf_addr);
		kfree(curr_node);
	}
	mutex_unlock(&iova_node_list.node_iova_lock);

	vcu_dbg_log("[VCU] %s ---", __func__);
}

static void add_new_iova_node(
	u64 in_wdma_dam_buf_addr,
	dma_addr_t in_mapped_iova,
	struct dma_buf_attachment *buf_att,
	struct sg_table *sgt)
{
	struct node_iova_t *curr_node = kmalloc(sizeof(struct node_iova_t), GFP_KERNEL);

	if (curr_node == NULL)
		return;

	mutex_lock(&iova_node_list.node_iova_lock);
	vcu_dbg_log("%s wdma_dam_buf_addr:0x%lx", __func__, in_wdma_dam_buf_addr);

	curr_node->mapped_iova = in_mapped_iova;
	curr_node->wdma_dma_buf_addr = in_wdma_dam_buf_addr;
	curr_node->buf_att = buf_att;
	curr_node->sgt = sgt;

	curr_node->next = NULL;

	if (iova_node_list.first_node == NULL) {// 1st node
		iova_node_list.first_node = curr_node;
		iova_node_list.last_node = NULL;
	} else if (iova_node_list.last_node == NULL) {// 2nd node
		iova_node_list.last_node = curr_node;
		iova_node_list.first_node->next = iova_node_list.last_node;
	} else {// update reference
		iova_node_list.last_node->next = curr_node;
		iova_node_list.last_node = iova_node_list.last_node->next;
	}

	mutex_unlock(&iova_node_list.node_iova_lock);
}

static dma_addr_t find_iova_node_by_dam_buf(uintptr_t in_wdma_dma_buf_addr)
{
	dma_addr_t ret = 0;
	struct node_iova_t *curr_node;

	mutex_lock(&iova_node_list.node_iova_lock);

	curr_node = iova_node_list.first_node;

	while (curr_node != NULL) {
		if (curr_node->wdma_dma_buf_addr == in_wdma_dma_buf_addr) {
			vcu_dbg_log("This dma_buf 0x%lx has been mapped, iova is 0x%lx",
				in_wdma_dma_buf_addr, curr_node->mapped_iova);
			ret = curr_node->mapped_iova;
			break;
		}
		curr_node = curr_node->next;
	}

	mutex_unlock(&iova_node_list.node_iova_lock);

	return ret;
}

static inline bool vcu_running(struct mtk_vcu *vcu)
{
	return (bool)vcu->run.signaled;
}

int vcu_ipi_register_mt8395(struct platform_device *pdev,
		     enum ipi_id id, ipi_handler_t handler,
		     const char *name, void *priv)
{
	struct mtk_vcu *vcu = platform_get_drvdata(pdev);
	struct vcu_ipi_desc *ipi_desc;
	unsigned int i = 0;

	if (vcu == NULL) {
		dev_info(&pdev->dev, "vcu device in not ready\n");
		return -EPROBE_DEFER;
	}

	if (id < IPI_VCU_INIT || id >= IPI_MAX) {
		dev_info(&pdev->dev, "[VCU] failed to register ipi message (Invalid arg.)\n");
		return -EINVAL;
	}

	i = ipi_id_to_inst_id(id);
	mutex_lock(&vcu->vcu_mutex[i]);

	if (id >= IPI_VCU_INIT && id < IPI_MAX && handler != NULL) {
		ipi_desc = vcu->ipi_desc;
		ipi_desc[id].name = name;
		ipi_desc[id].handler = handler;
		ipi_desc[id].priv = priv;
		mutex_unlock(&vcu->vcu_mutex[i]);
		return 0;
	}
	mutex_unlock(&vcu->vcu_mutex[i]);

	dev_info(&pdev->dev, "register vcu ipi id %d with invalid arguments\n",
		id);
	return -EINVAL;
}
EXPORT_SYMBOL_GPL(vcu_ipi_register_mt8395);

int vcu_ipi_send_mt8395(struct platform_device *pdev,
		 enum ipi_id id, void *buf,
		 unsigned int len, void *priv)
{
	unsigned int i = 0;
	struct mtk_vcu *vcu = platform_get_drvdata(pdev);
	struct vcu_ipi_desc *ipi_desc;
	struct share_obj send_obj;
	unsigned long timeout;
	int ret;

	if (id <= IPI_VCU_INIT || id >= IPI_MAX ||
	    len > sizeof(send_obj.share_buf) || buf == NULL) {
		dev_info(&pdev->dev, "[VCU] failed to send ipi message (Invalid arg.)\n");
		return -EINVAL;
	}

	if (vcu_running(vcu) == false) {
		dev_info(&pdev->dev, "[VCU] %s: VCU is not running\n", __func__);
		return -EPERM;
	}

	i = ipi_id_to_inst_id(id);
	timeout = msecs_to_jiffies(IPI_TIMEOUT_MS);

	mutex_lock(&vcu->vcu_mutex[i]);
	if (vcu->abort) {
		if (atomic_read(&vcu->open_cnt) > 0) {
			dev_info(vcu->dev, "wait for vpud killed %d\n",
				vcu->vpud_killed.count);
			ret = down_timeout(&vcu->vpud_killed, timeout);
			if (ret)
				dev_info(vcu->dev, "timeout %d\n", ret);
		}
		dev_info(&pdev->dev, "[VCU] vpud killed before IPI\n");
		mutex_unlock(&vcu->vcu_mutex[i]);
		return -EIO;
	}
	vcu->ipi_id_ack[id] = false;

	if (id >= IPI_VCU_INIT && id < IPI_MAX) {
		ipi_desc = vcu->ipi_desc;
		ipi_desc[id].priv = priv;
	}

	/* send the command to VCU */
	memcpy((void *)vcu->user_obj[i].share_buf, buf, len);
	vcu->user_obj[i].len = len;
	vcu->user_obj[i].id = (int)id;
	atomic_set(&vcu->ipi_got[i], 1);
	atomic_set(&vcu->ipi_done[i], 0);
	wake_up(&vcu->get_wq[i]);

	/* wait for VCU's ACK */
	ret = wait_event_timeout(vcu->ack_wq[i], vcu->ipi_id_ack[id], timeout);
	vcu->ipi_id_ack[id] = false;

	if (vcu->abort || ret == 0) {
		dev_info(&pdev->dev, "vcu ipi %d ack time out !%d", id, ret);
		mutex_lock(&vcu->vpud_task_mutex);
		if (!vcu->abort) {
			if (vcu->vcud_task) {
				send_sig(SIGTERM, vcu->vcud_task, 0);
				send_sig(SIGKILL, vcu->vcud_task, 0);
			}
		}
		mutex_unlock(&vcu->vpud_task_mutex);
		if (atomic_read(&vcu->open_cnt) > 0) {
			dev_info(vcu->dev, "wait for vpud killed %d\n",
				vcu->vpud_killed.count);
			ret = down_timeout(&vcu->vpud_killed, timeout);
			if (ret)
				dev_info(vcu->dev, "timeout %d\n", ret);
		}
		dev_info(&pdev->dev, "[VCU] vpud killed IPI fail\n");
		ret = -EIO;
		mutex_unlock(&vcu->vcu_mutex[i]);
		goto end;
	} else if (-ERESTARTSYS == ret) {
		dev_info(&pdev->dev, "vcu ipi %d ack wait interrupted by a signal",
			id);
		ret = -ERESTARTSYS;
		mutex_unlock(&vcu->vcu_mutex[i]);
		goto end;
	} else {
		ret = 0;
		mutex_unlock(&vcu->vcu_mutex[i]);
	}

	/* Waiting ipi_done, success means the daemon receiver thread
	 * dispatchs ipi msg done and returns to kernel for get next
	 * ipi msg.
	 * The dispatched ipi msg is being processed by app service.
	 * Usually, it takes dozens of microseconds in average.
	 */
	while (atomic_read(&vcu->ipi_done[i]) == 0)
		cond_resched();

end:
	return ret;
}
EXPORT_SYMBOL_GPL(vcu_ipi_send_mt8395);

static int vcu_ipi_get(struct mtk_vcu *vcu, unsigned long arg)
{
	unsigned int i = 0, p = 0;
	int ret;
	unsigned char *user_data_addr = NULL;
	struct share_obj share_buff_data = {0};
	struct share_obj *tmp;
	struct cam_ap_ipi_cmd *cmd;
	struct dma_buf *dbuf;

	user_data_addr = (unsigned char *)arg;
	ret = (long)copy_from_user(&share_buff_data, user_data_addr,
				   (unsigned long)sizeof(struct share_obj));
	if (ret != 0) {
		pr_info("[VCU] %s(%d) Copy data from user failed!\n",
			__func__, __LINE__);
		return -EINVAL;
	}

	i = ipi_id_to_inst_id(share_buff_data.id);

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

	// If it's cam buffer init, install fd
	tmp = &vcu->user_obj[i];
	if (tmp->id > IPI_CAMERA && tmp->id < IPI_CAMERA_MAX) {
		cmd = (struct cam_ap_ipi_cmd *)tmp->share_buf;
		if (cmd->msg_id == AP_IPIMSG_CAM_INIT_BUFFER) {
			for (p = 0; p < cmd->info.num_planes; ++p) {
				dbuf = (struct dma_buf *)cmd->info.dbuf.ptr[p];
				cmd->info.dbuf.fd[p] = dma_buf_fd(dbuf, O_CLOEXEC | O_RDWR);
				if (cmd->info.dbuf.fd[p] < 0)
					pr_err("[CAM-VCU] %s(%d) Invalid dma_buf!\n",
						__func__, __LINE__);
			}
		}
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
		pr_info("[VCU][%d] wait event return %d @%s\n",
			vcu->vcuid, ret, __func__);
		return ret;
	}
	ret = copy_to_user(user_data_addr, vcu->vdec_log_info,
			   (unsigned long)sizeof(struct log_test_nofuse));
	if (ret != 0) {
		pr_info("[VCU] %s(%d) Copy data to user failed!\n",
			__func__, __LINE__);
		ret = -EINVAL;
	}
	atomic_set(&vcu->vdec_log_got, 0);

	return ret;
}

static int vcu_sec_handle_get(struct mtk_vcu *vcu,
	struct mtk_vcu_queue *vcu_queue, unsigned long arg)
{
	int ret;
	unsigned char *user_data_addr = NULL;
	struct sec_handle_obj sec_handle_data;

	user_data_addr = (unsigned char *)arg;
	ret = (long)copy_from_user(&sec_handle_data, user_data_addr,
		(unsigned long)sizeof(struct sec_handle_obj));
	if (ret != 0L) {
		pr_info("[VCU] %s(%d) Copy data from user failed!\n",
			__func__, __LINE__);
		return -EINVAL;
	}
	mtk_vcu_get_dma_addr(vcu->dev, vcu_queue, sec_handle_data.fd,
		(dma_addr_t *)(&sec_handle_data.iova), &sec_handle_data.is_sec);

	ret = (long)copy_to_user(user_data_addr, &sec_handle_data,
		(unsigned long)sizeof(struct sec_handle_obj));
	if (ret != 0) {
		pr_info("[VCU] %s(%d) Copy data to user failed!\n",
			__func__, __LINE__);
		ret = -EINVAL;
	}

	return ret;
}

#if ENABLE_GCE

static void vcu_gce_add_used_page(struct vcu_page_info *used_pages,
	struct vcu_pa_pages *page)
{
	struct vcu_page_info *page_info;

	page_info = kmalloc(sizeof(struct vcu_page_info), GFP_KERNEL);
	if (!page_info)
		return;

	atomic_inc(&page->ref_cnt);
	page_info->page = page;
	list_add_tail(&page_info->list, &used_pages->list);
}

static void vcu_gce_release_used_pages(struct vcu_page_info *used_pages)
{
	struct vcu_page_info *page_info;
	struct list_head *p, *q;

	list_for_each_safe(p, q, &used_pages->list) {
		page_info = list_entry(p, struct vcu_page_info, list);
		list_del(p);
		atomic_dec(&page_info->page->ref_cnt);
		kfree(page_info);
	}
}

static void vcu_handle_gce_flush_callback(struct work_struct *work)
{
	int i, j;
	struct mtk_vcu *vcu;
	unsigned int core_id;
	struct gce_callback_data *buff = container_of(work, struct gce_callback_data,
		gce_callback_thread_work);
	unsigned int gce_order;
	struct cmdq_client *clt;

	if (buff->sta < 0)
	{
		pr_info("[VCU] flush_callback data->sta:%d\n", buff->sta);
		if(buff->cmdq_buff.secure == 0)
			vcu_gce_timeout_callback(buff);
	}
	i = (buff->cmdq_buff.codec_type == VCU_VDEC) ? VCU_VDEC : VCU_VENC;
	core_id = buff->cmdq_buff.core_id;

	vcu = buff->vcu_ptr;
	/*
	 * Drop the runtime-PM reference taken in vcu_gce_cmd_flush() for this
	 * GCE job now that the GCE thread done-IRQ has fired (this work is
	 * queued from the cmdq rx_callback). Balances pm_runtime_get_sync() on
	 * every exit path of this function.
	 */
	clt = (i == VCU_VDEC) ? vcu->clt_vdec[core_id] : vcu->clt_venc[core_id];

	j = vcu_gce_get_inst_id(vcu, buff->cmdq_buff.gce_handle, core_id);

	if (j < 0) {
		pr_info("[VCU] flush_callback get_inst_id fail!!%d\n", j);
		if (clt) {
			pm_runtime_mark_last_busy(clt->chan->mbox->dev);
			pm_runtime_put_autosuspend(clt->chan->mbox->dev);
		}
		return;
	}

	atomic_inc(&vcu->gce_info[j].flush_done[core_id]);
	atomic_dec(&vcu->gce_info[j].flush_pending);

	mutex_lock(&vcu->vcu_gce_mutex[i]);
	if (i == VCU_VENC && vcu->cbf.enc_pmqos_gce_end != NULL)
		vcu->cbf.enc_pmqos_gce_end(vcu->gce_info[j].v4l2_ctx, core_id,
				vcu->gce_job_cnt[i][core_id].counter);
	if (atomic_dec_and_test(&vcu->gce_job_cnt[i][core_id]) &&
		vcu->gce_info[j].v4l2_ctx != NULL){
		if (i == VCU_VENC && vcu->cbf.enc_unprepare != NULL &&
		    buff->cmdq_buff.event_type != VENC_SLC_EOF &&
		    buff->cmdq_buff.event_type != VENC_SLC_EOF_C1) {
			vcu->cbf.enc_unprepare(vcu->gce_info[j].v4l2_ctx,
				buff->cmdq_buff.core_id, &vcu->flags[i]);
#if IS_ENABLED(CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT)
			if (buff->cmdq_buff.secure != 0)
				cmdq_sec_mbox_switch_normal(vcu->clt_venc_sec[core_id]);
#endif
		}
	}

	mutex_unlock(&vcu->vcu_gce_mutex[i]);
	gce_order = buff->cmdq_buff.flush_order % GCE_PENDING_CNT;
	vcu_gce_release_used_pages(&vcu->gce_info[j].used_pages[gce_order]);

	wake_up(&vcu->gce_wq[i][core_id]);

	vcu_dbg_log("[VCU] %s: buff %p type %d order %d handle %llx core_id %d\n",
		__func__, buff, buff->cmdq_buff.codec_type,
		buff->cmdq_buff.flush_order, buff->cmdq_buff.gce_handle, core_id);

	vcu_cmdq_pkt_destroy(&buff->pkt);
	up(&vcu->gce_info[j].buff_sem[gce_order]);

	if (clt) {
		pm_runtime_mark_last_busy(clt->chan->mbox->dev);
		pm_runtime_put_autosuspend(clt->chan->mbox->dev);
	}
}

static int vcu_gce_set_inst_id(struct mtk_vcu *vcu, void *ctx, u64 gce_handle)
{
	int i, j;
	char data;

	mutex_lock(&vcu->vcu_share);
	for (i = 0; i < VCODEC_INST_MAX; i++) {
		if (vcu->gce_info[i].v4l2_ctx == NULL &&
			!copy_from_kernel_nofault((char *)&data, ctx, sizeof(char))) {
			vcu->gce_info[i].v4l2_ctx = ctx;
			vcu->gce_info[i].user_hdl = gce_handle;
			for(j = 0; j< GCE_PENDING_CNT; j++)
				INIT_WORK(&vcu->gce_info[i].buff[j].gce_callback_thread_work, vcu_handle_gce_flush_callback);
			mutex_unlock(&vcu->vcu_share);
			vcu_dbg_log("[VCU] %s ctx %p hndl %llu create id %d\n",
				__func__, ctx, gce_handle, i);
			return i;
		}
	}
	mutex_unlock(&vcu->vcu_share);
	pr_info("[VCU] %s fail ctx %p hndl %llu\n",
		__func__, ctx, gce_handle);

	return -1;
}

static int vcu_gce_get_inst_id(struct mtk_vcu *vcu, u64 gce_handle, u32 core_id)
{
	int i, temp;

	mutex_lock(&vcu->vcu_share);
	for (i = 0; i < VCODEC_INST_MAX; i++) {
		if (vcu->gce_info[i].user_hdl == gce_handle) {
			temp = atomic_read(&vcu->gce_info[i].flush_done[core_id]);
			mutex_unlock(&vcu->vcu_share);
			vcu_dbg_log("[VCU] %s hndl %llu get id %d cnt %d\n",
				__func__, gce_handle, i, temp);
			return i;
		}
	}
	mutex_unlock(&vcu->vcu_share);

	return -1;
}

static void vcu_gce_clear_inst_id(struct mtk_vcu *vcu, void *ctx)
{
	int i, j, temp, temp2;
	u64 gce_handle;

	mutex_lock(&vcu->vcu_share);
	for (i = 0; i < VCODEC_INST_MAX; i++) {
		if (vcu->gce_info[i].v4l2_ctx == ctx) {
			gce_handle = vcu->gce_info[i].user_hdl;
			vcu->gce_info[i].v4l2_ctx = NULL;
			vcu->gce_info[i].user_hdl = 0;
			temp = atomic_read(&vcu->gce_info[i].flush_pending);
			/* flush_pending > 0, ctx hw not unprepared */
			for (j = 0; j < GCE_THNUM_MAX; j++) {
				temp2 = atomic_read(&vcu->gce_info[i].flush_done[j]);
				atomic_set(&vcu->gce_info[i].flush_done[j], 0);
			}

			/* flush_done > 0, user event not waited */
			atomic_set(&vcu->gce_info[i].flush_pending, 0);
			mutex_unlock(&vcu->vcu_share);
			if (temp > 0)
				vcu_aee_print(
					"%s %p hndl %llu free id %d cnt %d %d\n",
					__func__, ctx, gce_handle,
					i, temp, temp2);
			else if (temp2 > 0)
				pr_info("%s %p hndl %llu free id %d cnt %d %d\n",
					__func__, ctx, gce_handle,
					i, temp, temp2);
			else
				vcu_dbg_log(
					"%s %p hndl %llu free id %d cnt %d %d\n",
					__func__, ctx, gce_handle,
					i, temp, temp2);
			return;
		}
	}
	mutex_unlock(&vcu->vcu_share);

	vcu_dbg_log("%s ctx %p not found!\n", __func__, ctx);
}

static void *vcu_check_gce_pa_base(struct mtk_vcu_queue *vcu_queue,
	u64 addr, u64 length, bool null_err)
{
	struct vcu_pa_pages *tmp;
	struct list_head *p, *q;

	list_for_each_safe(p, q, &vcu_queue->pa_pages.list) {
		tmp = list_entry(p, struct vcu_pa_pages, list);
		if (addr >= (u64)tmp->pa &&
			addr + length <= (u64)tmp->pa + PAGE_SIZE)
			return tmp;
	}

	if (null_err)
		pr_info("%s addr %llx length %llx not found!\n", __func__, addr, length);
	else
		vcu_dbg_log("%s addr %llx length %llx not found!\n", __func__, addr, length);

	return NULL;

}

#endif

static int vcu_check_reg_base(struct mtk_vcu *vcu, u64 addr, u64 length)
{
	int i;

	if (vcu->vcuid != VCU_ID_VDEC || addr >= MAP_PA_BASE_1GB)
		return -EINVAL;

	for (i = 0; i < (int)VCU_MAP_HW_REG_NUM; i++)
		if (addr >= (u64)vcu->map_base[i].base &&
			addr + length <= (u64)vcu->map_base[i].base + vcu->map_base[i].len)
			return 0;
	pr_info("%s addr %x length %x not found!\n", __func__, addr, length);

	return -EINVAL;
}
#if ENABLE_GCE
int vcu_cmdq_pkt_read_addr(struct cmdq_pkt *pkt, dma_addr_t addr, u16 dst_reg_idx)
{
	int ret = 0;
	if (pkt) {
		cmdq_pkt_assign(pkt, CMDQ_THR_SPR_IDX0, CMDQ_ADDR_HIGH(addr));
		cmdq_pkt_read_s(pkt, CMDQ_THR_SPR_IDX0, CMDQ_ADDR_LOW(addr), CMDQ_THR_SPR_IDX1);
	} else {
		pr_info("%s Error: pkt is NULL !", __func__);
		ret = -EINVAL;
	}
	return ret;
}


int vcu_cmdq_pkt_write(struct cmdq_pkt *pkt, dma_addr_t addr, u32 value, u32 mask)
{
	int ret = 0;

	if (pkt) {
		if(mask != 0xFFFFFFFF) {
			cmdq_pkt_assign(pkt, CMDQ_THR_SPR_IDX0, CMDQ_ADDR_HIGH(addr));
			cmdq_pkt_write_s_mask_value(pkt, CMDQ_THR_SPR_IDX0, CMDQ_ADDR_LOW(addr), value, mask);
		}
		else {
			cmdq_pkt_assign(pkt, CMDQ_THR_SPR_IDX0, CMDQ_ADDR_HIGH(addr));
			cmdq_pkt_write_s_value(pkt, CMDQ_THR_SPR_IDX0, CMDQ_ADDR_LOW(addr), value);
		}
	} else {
		pr_info("%s Error: pkt is NULL !", __func__);
		ret = -EINVAL;
	}

	return ret;
}

int vcu_cmdq_pkt_poll_addr(struct cmdq_pkt *pkt, u32 data, u32 addr, u32 mask,
	u8 reg_gpr)
{
	int ret = 0;

	if (pkt) {
		cmdq_pkt_move(pkt, reg_gpr, addr);

		if(mask != 0xFFFFFFFF)
			cmdq_pkt_poll_mask(pkt, reg_gpr, (u16)(addr), data, mask);
		else
			cmdq_pkt_poll(pkt, reg_gpr, (u16)(addr), data);
	} else {
		pr_info("%s Error: pkt is NULL !", __func__);
		ret = -EINVAL;
	}

	return ret;

}

int vcu_cmdq_pkt_mem_move(struct cmdq_pkt *pkt, dma_addr_t src_addr,
	dma_addr_t dst_addr, u16 swap_reg_idx)
{
	const u16 src_reg_idx = CMDQ_THR_SPR_IDX0;
	const u16 tmp_reg_idx = swap_reg_idx;
	int ret = 0;

	if (pkt) {
		cmdq_pkt_assign(pkt, src_reg_idx, CMDQ_ADDR_HIGH(src_addr));
		cmdq_pkt_read_s(pkt, src_reg_idx, CMDQ_ADDR_LOW(src_addr), swap_reg_idx);
		cmdq_pkt_assign(pkt, src_reg_idx, CMDQ_ADDR_HIGH(dst_addr));
		cmdq_pkt_write_s(pkt, src_reg_idx, CMDQ_ADDR_LOW(dst_addr), swap_reg_idx);
	} else {
		pr_info("%s Error: pkt is NULL !", __func__);
		ret = -EINVAL;
	}
	return ret;
}

static void vcu_set_gce_cmd(struct cmdq_pkt *pkt,
	struct mtk_vcu *vcu, unsigned int gce_index, unsigned int gce_order,
	struct mtk_vcu_queue *q,
	unsigned char cmd, u64 addr, u64 data, u32 mask, u32 gpr, u32 dma_offset, u32 dma_size)
{
	void *src_page, *dst_page;
	int reg_check;

	switch (cmd) {
	case CMD_READ:
		if (vcu_check_reg_base(vcu, addr, 4) == 0)
			vcu_cmdq_pkt_read_addr(pkt, addr, CMDQ_THR_SPR_IDX1);
		else
			pr_info("[VCU] CMD_READ wrong addr: 0x%x\n", addr);
	break;
	case CMD_WRITE:
		if (vcu_check_reg_base(vcu, addr, 4) == 0)
			vcu_cmdq_pkt_write(pkt, addr, data, mask);
		else
			pr_info("[VCU] CMD_WRITE wrong addr: 0x%x 0x%x 0x%x\n",
				addr, data, mask);
	break;
#if IS_ENABLED(CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT)
	case CMD_SEC_WRITE:
#if IS_ENABLED(CONFIG_MTK_CMDQ_MBOX_EXT)
		if (vcu_check_reg_base(vcu, addr, 4) == 0)
			cmdq_sec_pkt_write_reg(pkt,
				addr,
				data,
				CMDQ_IWC_H_2_MVA,
				dma_offset,
				dma_size,
				0);
		else
			pr_info("[VCU] CMD_SEC_WRITE wrong addr: 0x%x 0x%x 0x%x 0x%x\n",
				addr, data, dma_offset, dma_size);
#endif
		pr_debug("[VCU] %s addr: 0x%x, data: 0x%x, offset: 0x%x, size: 0x%x\n",
			__func__, addr, data, dma_offset, dma_size);
	break;
#endif
	case CMD_POLL_REG:
		if (vcu_check_reg_base(vcu, addr, 4) == 0)
			vcu_cmdq_pkt_poll_addr(pkt, data, addr, mask, gpr);
		else
			pr_info("[VCU] CMD_POLL_REG wrong addr: 0x%x 0x%x 0x%x\n",
				addr, data, mask);
	break;
	case CMD_WAIT_EVENT:
		if (data < GCE_EVENT_MAX)
			cmdq_pkt_wfe(pkt, vcu->gce_codec_eid[data], true);
		else
			pr_info("[VCU] %s got wrong eid %llu\n", __func__, data);
	break;
	case CMD_MEM_MV:
		mutex_lock(&q->mmap_lock);
		reg_check = vcu_check_reg_base(vcu, addr, 4);
		src_page = vcu_check_gce_pa_base(q, addr, 4, reg_check != 0);
		dst_page = vcu_check_gce_pa_base(q, data, 4, true);
		if ((reg_check == 0 || src_page != NULL) && dst_page != NULL) {
			if (src_page != NULL)
				vcu_gce_add_used_page(
				    &vcu->gce_info[gce_index].used_pages[gce_order], src_page);
			vcu_gce_add_used_page(
				    &vcu->gce_info[gce_index].used_pages[gce_order], dst_page);
			vcu_cmdq_pkt_mem_move(pkt, addr,
				data, CMDQ_THR_SPR_IDX1);
		} else
			pr_info("[VCU] CMD_MEM_MV wrong addr/data: 0x%llx 0x%llx\n",
				addr, data);
		mutex_unlock(&q->mmap_lock);
	break;
	case CMD_POLL_ADDR:
		mutex_lock(&q->mmap_lock);
		reg_check = vcu_check_reg_base(vcu, addr, 4);
		src_page = vcu_check_gce_pa_base(q, addr, 4, reg_check != 0);
		if (reg_check == 0 || src_page != NULL) {
			if (src_page != NULL)
				vcu_gce_add_used_page(
					&vcu->gce_info[gce_index].used_pages[gce_order], src_page);
			cmdq_pkt_poll_timeout(pkt, data, SUBSYS_NO_SUPPORT,
				addr, mask, ~0, gpr);
		} else
			pr_info("[VCU] CMD_POLL_REG wrong addr: 0x%llx 0x%llx 0x%x\n",
				addr, data, mask);
		mutex_unlock(&q->mmap_lock);
	break;
	default:
		vcu_dbg_log("[VCU] unknown GCE cmd %d\n", cmd);
	break;
	}
}

static void vcu_gce_timeout_callback(struct gce_callback_data * data)
{
	struct gce_callback_data *buff = data;
	struct mtk_vcu *vcu;
	struct list_head *p, *q;
	struct mtk_vcu_queue *vcu_queue;
	struct vcu_pa_pages *tmp;


	vcu = buff->vcu_ptr;
	vcu_queue = buff->vcu_queue;
	vcu_dbg_log("%s: buff %p vcu: %p, codec_typ: %d\n",
	    __func__, buff, vcu, buff->cmdq_buff.codec_type);

	if (buff->cmdq_buff.codec_type == VCU_VENC
		&& vcu->cbf.gce_timeout_dump != NULL)
		vcu->cbf.gce_timeout_dump(vcu->curr_ctx[VCU_VENC]);
	else if (buff->cmdq_buff.codec_type == VCU_VDEC
		&& vcu->cbf.gce_timeout_dump != NULL)
		vcu->cbf.gce_timeout_dump(vcu->curr_ctx[VCU_VDEC]);

	mutex_lock(&vcu_queue->mmap_lock);
	list_for_each_safe(p, q, &vcu_queue->pa_pages.list) {
	    tmp = list_entry(p, struct vcu_pa_pages, list);
	    pr_info("%s: vcu_pa_pages %lx kva %lx data %lx\n",
	        __func__, tmp->pa, tmp->kva,
	        *(unsigned long *)tmp->kva);
	}
	mutex_unlock(&vcu_queue->mmap_lock);

}

static int vcu_cmdq_pkt_create(struct cmdq_client *client, struct cmdq_pkt *pkt,
			       size_t size)
{
	struct device *dev;
	dma_addr_t dma_addr;

	pkt->va_base = kzalloc(size, GFP_KERNEL);
	if (!pkt->va_base)
		return -ENOMEM;

	pkt->buf_size = size;
	pkt->cl = (void *)client;

	dev = client->chan->mbox->dev;
	dma_addr = dma_map_single(dev, pkt->va_base, pkt->buf_size,
				  DMA_TO_DEVICE);
	if (dma_mapping_error(dev, dma_addr)) {
		dev_err(dev, "dma map failed, size=%u\n", (u32)(u64)size);
		kfree(pkt->va_base);
		return -ENOMEM;
	}

	pkt->pa_base = dma_addr;

	return 0;
}

static void vcu_cmdq_pkt_destroy(struct cmdq_pkt *pkt)
{
	struct cmdq_client *client = (struct cmdq_client *)pkt->cl;

	dma_unmap_single(client->chan->mbox->dev, pkt->pa_base, pkt->buf_size,
			 DMA_TO_DEVICE);
	kfree(pkt->va_base);
	pkt->va_base = NULL;
}

static void vcu_gce_flush_callback(struct mbox_client *cl, void *mssg)
{
	struct cmdq_cb_data *data;
	struct gce_callback_data *buff;
	struct mtk_vcu *vcu;
	unsigned int core_id;

	data = (struct cmdq_cb_data *)mssg;
	buff = container_of(data->pkt, struct gce_callback_data, pkt);
	vcu = buff->vcu_ptr;
	core_id = buff->cmdq_buff.core_id;
	buff->sta = data->sta;

	queue_work(vcu->gce_callback_thread_wq[core_id], &buff->gce_callback_thread_work);
}

static int vcu_gce_cmd_flush(struct mtk_vcu *vcu,
	struct mtk_vcu_queue *q, unsigned long arg)
{
	int i, codec_type, gce_idx, ret;
	unsigned char *user_data_addr = NULL;
	struct gce_callback_data buff = {0};
	struct cmdq_pkt *pkt_ptr = NULL;
	struct cmdq_client *cl = NULL;
	struct gce_cmds *cmds = NULL;
	unsigned int core_id;
	unsigned int gce_order;

	vcu_dbg_log("[VCU] %s +\n", __func__);

	time_check_start();
	user_data_addr = (unsigned char *)arg;
	ret = (long)copy_from_user(&buff.cmdq_buff, user_data_addr,
				   (unsigned long)sizeof(struct gce_cmdq_obj));
	if (ret != 0L) {
		pr_info("[VCU] %s(%d) gce_cmdq_obj copy_from_user failed!%d\n",
			__func__, __LINE__, ret);
		return -EINVAL;
	}

	codec_type = (buff.cmdq_buff.codec_type == VCU_VDEC) ? VCU_VDEC : VCU_VENC;
	gce_order = buff.cmdq_buff.flush_order % GCE_PENDING_CNT;
	cmds = vcu->gce_cmds[codec_type];

	mutex_lock(&vcu->gce_cmds_mutex[codec_type]);
	if (buff.cmdq_buff.cmds_user_ptr > 0) {
		user_data_addr = (unsigned char *)
			(unsigned long)buff.cmdq_buff.cmds_user_ptr;
		ret = (long)copy_from_user(cmds, user_data_addr,
			(unsigned long)sizeof(struct gce_cmds));
		if (ret != 0L) {
			pr_info("[VCU] %s(%d) gce_cmds copy_from_user failed!%d\n",
				__func__, __LINE__, ret);
			mutex_unlock(&vcu->gce_cmds_mutex[codec_type]);
			return -EINVAL;
		}
	} else {
		mutex_unlock(&vcu->gce_cmds_mutex[codec_type]);
		return -EINVAL;
	}


	buff.cmdq_buff.cmds_user_ptr = (u64)(unsigned long)cmds;
	core_id = buff.cmdq_buff.core_id;

	if (buff.cmdq_buff.codec_type >= VCU_CODEC_MAX ||
		core_id >=
		vcu->gce_th_num[buff.cmdq_buff.codec_type]) {
		pr_info("[VCU] %s invalid core(th) id %d\n",
			__func__, core_id);
		mutex_unlock(&vcu->gce_cmds_mutex[codec_type]);
		return -EINVAL;
	}

	cl = (buff.cmdq_buff.codec_type == VCU_VDEC) ?
		vcu->clt_vdec[core_id] :
		vcu->clt_venc[core_id];

#if IS_ENABLED(CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT)
	if (buff.cmdq_buff.codec_type == VCU_VENC) {
		if (buff.cmdq_buff.secure != 0)
			cl = vcu->clt_venc_sec[0];
	}
#endif

	if (cl == NULL) {
		pr_info("[VCU] %s gce thread is null id %d type %d\n",
			__func__, core_id,
			buff.cmdq_buff.codec_type);
		mutex_unlock(&vcu->gce_cmds_mutex[codec_type]);
		return -EINVAL;
	}

	buff.vcu_ptr = vcu;
	buff.vcu_queue = q;

	gce_idx = vcu_gce_get_inst_id(vcu, buff.cmdq_buff.gce_handle, core_id);

	if (gce_idx < 0)
		gce_idx = vcu_gce_set_inst_id(vcu, vcu->curr_ctx[codec_type],
			buff.cmdq_buff.gce_handle);
	if (gce_idx < 0) {
		mutex_unlock(&vcu->gce_cmds_mutex[codec_type]);
		return -EINVAL;
	}

	time_check_end(100, strlen(vcodec_param_string));

	time_check_start();
	mutex_lock(&vcu->vcu_gce_mutex[codec_type]);

	if (atomic_read(&vcu->gce_job_cnt[codec_type][core_id]) == 0 &&
		vcu->gce_info[gce_idx].v4l2_ctx != NULL){
		if (codec_type == VCU_VENC && vcu->cbf.enc_prepare != NULL) {
			vcu->cbf.enc_prepare(vcu->gce_info[gce_idx].v4l2_ctx,
				core_id, &vcu->flags[codec_type]);
		}
	}
	vcu_dbg_log("vcu gce_info[%d].v4l2_ctx %p\n",
		gce_idx, vcu->gce_info[gce_idx].v4l2_ctx);

	if (codec_type == VCU_VENC && vcu->cbf.enc_pmqos_gce_begin != NULL) {
		vcu->cbf.enc_pmqos_gce_begin(vcu->gce_info[gce_idx].v4l2_ctx, core_id,
			vcu->gce_job_cnt[codec_type][core_id].counter);
	}
	atomic_inc(&vcu->gce_job_cnt[codec_type][core_id]);
	mutex_unlock(&vcu->vcu_gce_mutex[codec_type]);
	time_check_end(100, strlen(vcodec_param_string));

	time_check_start();
	if (cmds->cmd_cnt >= VCODEC_CMDQ_CMD_MAX) {
		pr_info("[VCU] cmd_cnt (%d) overflow!!\n", cmds->cmd_cnt);
		cmds->cmd_cnt = VCODEC_CMDQ_CMD_MAX;
		/* it might performs with error, but keep going */
	}

	ret = down_interruptible(&vcu->gce_info[gce_idx].buff_sem[gce_order]);
	if (ret < 0) {
		pr_info("[VCU] get buff_sem fail!! idx[%d], order[%d]\n", gce_idx, gce_order);
		mutex_unlock(&vcu->gce_cmds_mutex[codec_type]);
		return -ERESTARTSYS;
	}

	//it does not need to copy gce_callback_thread_work item
	memcpy(&vcu->gce_info[gce_idx].buff[gce_order], &buff, sizeof(buff) - sizeof(struct work_struct));

	/* create & assign cmdq pkt buffer */
	pkt_ptr = &vcu->gce_info[gce_idx].buff[gce_order].pkt;
	ret = vcu_cmdq_pkt_create(cl, pkt_ptr, SZ_16K);
	if (unlikely(ret < 0)) {
		pr_info("[VCU] cmdq_pkt_create fail\n");
		up(&vcu->gce_info[gce_idx].buff_sem[gce_order]);
		mutex_unlock(&vcu->gce_cmds_mutex[codec_type]);
		return ret;
	}

#if IS_ENABLED(CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT)
	if (buff.cmdq_buff.codec_type == VCU_VENC) {
		if (buff.cmdq_buff.secure != 0) {
			const u64 dapc_engine =
				(1LL << CMDQ_SEC_VENC_BSDMA) |
				(1LL << CMDQ_SEC_VENC_CUR_LUMA) |
				(1LL << CMDQ_SEC_VENC_CUR_CHROMA) |
				(1LL << CMDQ_SEC_VENC_REF_LUMA) |
				(1LL << CMDQ_SEC_VENC_REF_CHROMA) |
				(1LL << CMDQ_SEC_VENC_REC) |
				(1LL << CMDQ_SEC_VENC_SV_COMV) |
				(1LL << CMDQ_SEC_VENC_RD_COMV);

			const u64 port_sec_engine =
				(1LL << CMDQ_SEC_VENC_BSDMA) |
				(1LL << CMDQ_SEC_VENC_CUR_LUMA) |
				(1LL << CMDQ_SEC_VENC_CUR_CHROMA) |
				(1LL << CMDQ_SEC_VENC_REF_LUMA) |
				(1LL << CMDQ_SEC_VENC_REF_CHROMA) |
				(1LL << CMDQ_SEC_VENC_REC) |
				(1LL << CMDQ_SEC_VENC_SV_COMV) |
				(1LL << CMDQ_SEC_VENC_RD_COMV);

			pr_debug("[VCU] dapc_engine: 0x%llx, port_sec_engine: 0x%llx\n",
				dapc_engine, port_sec_engine);
#if IS_ENABLED(CONFIG_MTK_CMDQ_MBOX_EXT)
			cmdq_sec_pkt_set_data(pkt_ptr, dapc_engine,
				port_sec_engine, CMDQ_SEC_KERNEL_CONFIG_GENERAL,
				CMDQ_METAEX_VENC);
#endif
		}
	}
#endif

	for (i = 0; i < cmds->cmd_cnt; i++) {
		vcu_set_gce_cmd(pkt_ptr, vcu, gce_idx, gce_order, q, cmds->cmd[i],
					cmds->addr[i], cmds->data[i],
					cmds->mask[i], vcu->gce_gpr[core_id],
					cmds->dma_offset[i], cmds->dma_size[i]);

		if (cmds->cmd[i] == CMD_WAIT_EVENT)
			buff.cmdq_buff.event_type = cmds->data[i];
	}

	pr_debug("[VCU][%d] %s: buff %p type %d cnt %d order %d hndl %llx %d %d\n",
		core_id, __func__, &vcu->gce_info[gce_idx].buff[gce_order],
		buff.cmdq_buff.codec_type,
		cmds->cmd_cnt, buff.cmdq_buff.flush_order,
		buff.cmdq_buff.gce_handle, ret, gce_idx);
	mutex_unlock(&vcu->gce_cmds_mutex[codec_type]);

	/* flush cmd async */
	cmdq_pkt_finalize(pkt_ptr);
	dma_sync_single_for_device(cl->chan->mbox->dev, pkt_ptr->pa_base,
				   pkt_ptr->cmd_buf_size, DMA_TO_DEVICE);
	/*
	 * Hold a runtime-PM reference on the GCE/cmdq mailbox device across the
	 * whole async GCE job. Since mainline commit 3f39f5652037 ("mailbox:
	 * mtk-cmdq: Remove pm_runtime APIs from cmdq_mbox_send_data()") the
	 * mailbox no longer keeps the GCE clock on for an in-flight task --
	 * power management is delegated to the client. The mainline clients do
	 * this via cmdq_mbox_flush()/cmdq_pkt_flush(); this driver submits the
	 * packet with a raw mbox_send_message() and waits on its own gce_wq, so
	 * it must take the reference itself. Without it the GCE clock is gated
	 * by autosuspend while the venc Write-Path packet executes, the thread
	 * done-IRQ never fires, flush_done is never incremented, and
	 * vcu_wait_gce_callback() hangs (returns -512 / vpud killed). The
	 * reference is dropped in vcu_handle_gce_flush_callback() once the GCE
	 * job has completed (or below, on the synchronous submit-fail path).
	 * Runs from an ioctl (non-atomic), so the sleeping get is safe here.
	 */
	pm_runtime_get_sync(cl->chan->mbox->dev);
	ret = mbox_send_message(cl->chan, (void *)pkt_ptr);
	if (unlikely(ret < 0)) {
		pr_info("[VCU] cmdq flush fail pkt %p\n", pkt_ptr);
		pm_runtime_mark_last_busy(cl->chan->mbox->dev);
		pm_runtime_put_autosuspend(cl->chan->mbox->dev);
		vcu_gce_release_used_pages(&vcu->gce_info[gce_idx].used_pages[gce_order]);
		vcu_cmdq_pkt_destroy(pkt_ptr);
		up(&vcu->gce_info[gce_idx].buff_sem[gce_order]);
	} else {
		mbox_client_txdone(cl->chan, 0);
		atomic_inc(&vcu->gce_info[gce_idx].flush_pending);
	}

	time_check_end(100, strlen(vcodec_param_string));

	return ret;
}

static int vcu_wait_gce_callback(struct mtk_vcu *vcu, unsigned long arg)
{
	int ret, i, j;
	int core_id;
	unsigned char *user_data_addr = NULL;
	struct gce_obj obj;

	user_data_addr = (unsigned char *)arg;
	ret = (long)copy_from_user(&obj, user_data_addr,
				   (unsigned long)sizeof(struct gce_obj));
	if (ret != 0L) {
		pr_info("[VCU] %s(%d) copy_from_user failed!%d\n",
			__func__, __LINE__, ret);
		return -EINVAL;
	}

	i = (obj.codec_type == VCU_VDEC) ? VCU_VDEC : VCU_VENC;
	vcu_dbg_log("[VCU] %s: type %d handle %llx\n",
		__func__, obj.codec_type, obj.gce_handle);

	core_id = obj.core_id;
	/* use wait_event_interruptible not freezable due to
	 * slowmotion GCE case vcu_gce_cmd_flush will hold
	 * mutex in user process which cannot be freezed
	 */
	j = vcu_gce_get_inst_id(vcu, obj.gce_handle, core_id);

	if (j < 0)
		return -EINVAL;

	ret = wait_event_interruptible(vcu->gce_wq[i][core_id],
		atomic_read(&vcu->gce_info[j].flush_done[core_id]) > 0);
	if (ret != 0) {
		pr_info("[VCU][%d][%d] wait event return %d @%s\n",
			vcu->vcuid, i, ret, __func__);
		return ret;
	}
	atomic_dec(&vcu->gce_info[j].flush_done[core_id]);

	return ret;
}
#endif
//extern u64 mtk_drm_get_wdma_dma_buf(void);
//extern u32 mtk_drm_get_wdma_y_buf(void);

static long vcu_get_disp_mapped_iova(struct mtk_vcu *vcu, unsigned long arg)
{
	long ret = 0;
	uintptr_t wdma_dma_buf_addr;

	struct dma_buf *display_dma_buf;
	struct dma_buf_attachment *buf_att;
	struct sg_table *sgt;
	dma_addr_t mapped_iova = 0xdeadbeef;
	dma_addr_t mapped_iova_old = 0;

	vcu_dbg_log("[VCU] %s +\n", __func__);

	wdma_dma_buf_addr = 0;//(uintptr_t)mtk_drm_get_wdma_dma_buf();
	display_dma_buf = (struct dma_buf *)(uintptr_t)wdma_dma_buf_addr;
	if (wdma_dma_buf_addr != 0) {
		// get display dma_buf from dram
		vcu_dbg_log("[VCU] wdma_dma_buf address = 0x%lx\n", wdma_dma_buf_addr);
	} else {
		pr_info("[VCU] Invalid wdma_dma_buf address 0x%lx\n", wdma_dma_buf_addr);
		return 0;
	}

	// check if this dma_buf has been mapped; if yes, return its mapped iova directly
	mapped_iova_old = find_iova_node_by_dam_buf(wdma_dma_buf_addr);
	if (mapped_iova_old > 0)
		return (long)mapped_iova_old;

	// obtain iova on this device
	buf_att = dma_buf_attach(display_dma_buf, vcu->dev);
	vcu_dbg_log("%s %d", __func__, __LINE__);
	sgt = dma_buf_map_attachment(buf_att, DMA_FROM_DEVICE);
	vcu_dbg_log("%s %d", __func__, __LINE__);
	mapped_iova = sg_dma_address(sgt->sgl);
	vcu_dbg_log("[VCU] mapped_iova=0x%lx", mapped_iova);

	// add this mapped iova in list
	add_new_iova_node(wdma_dma_buf_addr, mapped_iova, buf_att, sgt);

	// return iova to caller
	ret = (long)mapped_iova;

	vcu_dbg_log("[VCU] %s -\n", __func__);

	return ret;
}

static void vcu_clear_disp_mapped_iova(struct mtk_vcu *vcu, unsigned long arg)
{
	remove_all_iova_node(vcu);
}

static long vcu_get_disp_wdma_y_addr(struct mtk_vcu *vcu, unsigned long arg)
{
	return 0;//mtk_drm_get_wdma_y_buf();
}

#if ENABLE_GCE
int vcu_set_v4l2_callback_mt8395(struct platform_device *pdev,
	struct vcu_v4l2_callback_func *call_back)
{
	struct mtk_vcu *vcu = platform_get_drvdata(pdev);

	if (call_back->enc_prepare != NULL)
		vcu->cbf.enc_prepare = call_back->enc_prepare;
	if (call_back->enc_unprepare != NULL)
		vcu->cbf.enc_unprepare = call_back->enc_unprepare;
	if (call_back->enc_lock != NULL)
		vcu->cbf.enc_lock = call_back->enc_lock;
	if (call_back->enc_unlock != NULL)
		vcu->cbf.enc_unlock = call_back->enc_unlock;
	if (call_back->enc_pmqos_gce_begin != NULL)
		vcu->cbf.enc_pmqos_gce_begin = call_back->enc_pmqos_gce_begin;
	if (call_back->enc_pmqos_gce_end != NULL)
		vcu->cbf.enc_pmqos_gce_end = call_back->enc_pmqos_gce_end;
	if (call_back->gce_timeout_dump != NULL)
		vcu->cbf.gce_timeout_dump = call_back->gce_timeout_dump;
	if (call_back->vdec_realease_lock != NULL)
		vcu->cbf.vdec_realease_lock = call_back->vdec_realease_lock;

	return 0;
}
EXPORT_SYMBOL_GPL(vcu_set_v4l2_callback_mt8395);
#endif
int vcu_get_ctx_ipi_binding_lock_mt8395(struct platform_device *pdev,
	struct mutex **mutex, unsigned long type)
{
	struct mtk_vcu *vcu = platform_get_drvdata(pdev);

	*mutex = &vcu->ctx_ipi_binding[type];

	return 0;
}
EXPORT_SYMBOL_GPL(vcu_get_ctx_ipi_binding_lock_mt8395);

int vcu_set_codec_ctx_mt8395(struct platform_device *pdev,
		 void *codec_ctx, struct vb2_buffer *src_vb,
		 struct vb2_buffer *dst_vb, unsigned long type)
{
	struct mtk_vcu *vcu = platform_get_drvdata(pdev);

	vcu_dbg_log("[VCU] %s %p type %lu src_vb %p dst_vb %p\n",
		__func__, codec_ctx, type, src_vb, dst_vb);

	vcu->curr_ctx[type] = codec_ctx;
	vcu->curr_src_vb[type] = src_vb;
	vcu->curr_dst_vb[type] = dst_vb;

	return 0;
}
EXPORT_SYMBOL_GPL(vcu_set_codec_ctx_mt8395);

int vcu_clear_codec_ctx_mt8395(struct platform_device *pdev,
		 void *codec_ctx, unsigned long type)
{
	struct mtk_vcu *vcu = platform_get_drvdata(pdev);

	vcu_dbg_log("[VCU] %s %p type %lu\n", __func__, codec_ctx, type);

	mutex_lock(&vcu->vcu_gce_mutex[type]);
#if ENABLE_GCE
	vcu_gce_clear_inst_id(vcu, codec_ctx);
#endif
	vcu->curr_ctx[type] = NULL;
	vcu->curr_src_vb[type] = NULL;
	vcu->curr_dst_vb[type] = NULL;
	mutex_unlock(&vcu->vcu_gce_mutex[type]);

	return 0;
}
EXPORT_SYMBOL_GPL(vcu_clear_codec_ctx_mt8395);

void *vcu_mapping_dm_addr_mt8395(struct platform_device *pdev,
			  uintptr_t dtcm_dmem_addr)
{
	struct mtk_vcu *vcu;
	uintptr_t d_vma, d_va_start;
	uintptr_t d_off, d_va;

	if (!IS_ERR_OR_NULL(pdev))
		vcu = platform_get_drvdata(pdev);
	else {
		dev_info(&pdev->dev, "[VCU] %s: Invalid pdev %p\n",
			__func__, pdev);
		return NULL;
	}

	d_vma = (uintptr_t)(dtcm_dmem_addr);
	d_va_start = (uintptr_t)VCU_DMEM0_VIRT(vcu);
	d_off = d_vma - VCU_DMEM0_VMA(vcu);

	if (dtcm_dmem_addr == 0UL || d_off >= VCU_DMEM0_LEN(vcu)) {
		dev_info(&pdev->dev, "[VCU] %s: Invalid vma 0x%lx len %lx\n",
			__func__, dtcm_dmem_addr, VCU_DMEM0_LEN(vcu));
		return NULL;
	}

	d_va = d_va_start + d_off;
	dev_dbg(&pdev->dev, "[VCU] %s: 0x%lx -> 0x%lx\n",
		__func__, d_vma, d_va);
	return (void *)d_va;
}
EXPORT_SYMBOL_GPL(vcu_mapping_dm_addr_mt8395);

struct platform_device *vcu_get_plat_device_mt8395(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *vcu_node = NULL;
	struct platform_device *vcu_pdev = NULL;

	dev_dbg(&pdev->dev, "[VCU] %s\n", __func__);

	vcu_node = of_parse_phandle(dev->of_node, "mediatek,vcu", 0);
	if (vcu_node == NULL) {
		dev_info(dev, "[VCU] can't get vcu node\n");
		return NULL;
	}

	vcu_pdev = of_find_device_by_node(vcu_node);
	if (WARN_ON(vcu_pdev == NULL) == true) {
		dev_info(dev, "[VCU] vcu pdev failed\n");
		of_node_put(vcu_node);
		return NULL;
	}

	return vcu_pdev;
}
EXPORT_SYMBOL_GPL(vcu_get_plat_device_mt8395);

int vcu_load_firmware_mt8395(struct platform_device *pdev)
{
	return 0;
}
EXPORT_SYMBOL_GPL(vcu_load_firmware_mt8395);

int vcu_compare_version_mt8395(struct platform_device *pdev,
			const char *expected_version)
{
	return 0;
}
EXPORT_SYMBOL_GPL(vcu_compare_version_mt8395);

void vcu_get_task_mt8395(struct task_struct **task, int reset)
{
	struct mtk_vcu *vcu = vcu_mtkdev[0];

	vcu_dbg_log("mtk_vcu_get_task %p\n", vcu->vcud_task);

	mutex_lock(&vcu->vpud_task_mutex);

	if (reset == 1)
		vcu->vcud_task = NULL;
	else if (reset == 2 && current == vcu->vcud_task)
		vcu->vcud_task = NULL;

	if (task)
		*task = vcu->vcud_task;

	if (reset)
		mutex_unlock(&vcu->vpud_task_mutex);
}
EXPORT_SYMBOL_GPL(vcu_get_task_mt8395);

/* need to put task for unlock mutex when get task reset == 0 */
void vcu_put_task_mt8395(void)
{
	struct mtk_vcu *vcu = vcu_mtkdev[0];

	mutex_unlock(&vcu->vpud_task_mutex);
}
EXPORT_SYMBOL_GPL(vcu_put_task_mt8395);

static int vcu_ipi_handler(struct mtk_vcu *vcu, struct share_obj *rcv_obj)
{
	struct vcu_ipi_desc *ipi_desc = vcu->ipi_desc;
	int non_ack = 0;
	int ret = -1;
	unsigned int i = 0;

	i = ipi_id_to_inst_id(rcv_obj->id);

	if (vcu->abort) {
		pr_info("[VCU] aborted not handled: %s %d %d: ipi %d\n",
			current->comm, current->tgid, current->pid, rcv_obj->id);
		return -1;
	}

	if (rcv_obj->id < (int)IPI_MAX &&
	    ipi_desc[rcv_obj->id].handler != NULL) {
		non_ack = ipi_desc[rcv_obj->id].handler(rcv_obj->share_buf,
				rcv_obj->len,
				ipi_desc[rcv_obj->id].priv);
		if (rcv_obj->id > (int)IPI_VCU_INIT && non_ack == 0) {
			vcu->ipi_id_ack[rcv_obj->id] = true;
			wake_up(&vcu->ack_wq[i]);
		}
		ret = 0;
	} else
		dev_info(vcu->dev, "[VCU] No such ipi id = %d\n", rcv_obj->id);

	return ret;
}

static int vcu_ipi_init(struct mtk_vcu *vcu)
{
	vcu->is_open = false;
	mutex_init(&vcu->vcu_mutex[VCU_VDEC]);
#if ENABLE_GCE
	mutex_init(&vcu->vcu_gce_mutex[VCU_VDEC]);
	mutex_init(&vcu->vcu_gce_mutex[VCU_VENC]);
	mutex_init(&vcu->vcu_gce_mutex[VCU_RESOURCE]);
#endif
	mutex_init(&vcu->ctx_ipi_binding[VCU_VDEC]);
	mutex_init(&vcu->vcu_mutex[VCU_VENC]);
	mutex_init(&vcu->ctx_ipi_binding[VCU_VENC]);
	mutex_init(&vcu->vcu_mutex[VCU_RESOURCE]);
	mutex_init(&vcu->ctx_ipi_binding[VCU_RESOURCE]);
	mutex_init(&vcu->vcu_share);
	mutex_init(&vcu->vpud_task_mutex);

	return 0;
}

static int vcu_ipi_deinit(struct mtk_vcu *vcu)
{
	vcu->is_open = false;
	mutex_destroy(&vcu->vcu_mutex[VCU_VDEC]);
#if ENABLE_GCE
	mutex_destroy(&vcu->vcu_gce_mutex[VCU_VDEC]);
	mutex_destroy(&vcu->vcu_gce_mutex[VCU_VENC]);
	mutex_destroy(&vcu->vcu_gce_mutex[VCU_RESOURCE]);
#endif
	mutex_destroy(&vcu->ctx_ipi_binding[VCU_VDEC]);
	mutex_destroy(&vcu->vcu_mutex[VCU_VENC]);
	mutex_destroy(&vcu->ctx_ipi_binding[VCU_VENC]);
	mutex_destroy(&vcu->vcu_mutex[VCU_RESOURCE]);
	mutex_destroy(&vcu->ctx_ipi_binding[VCU_RESOURCE]);
	mutex_destroy(&vcu->vcu_share);
	mutex_destroy(&vcu->vpud_task_mutex);

	return 0;
}

static int vcu_init_ipi_handler(void *data, unsigned int len, void *priv)
{
	struct mtk_vcu *vcu = (struct mtk_vcu *)priv;
	struct vcu_run *run = (struct vcu_run *)data;
	int wait_cnt = 0;

	/* handle uninitialize message */
	if (vcu->run.signaled == 1u && run->signaled == 0u) {
		/* smi debug dump before wake up ack to worker
		 * which will send error event to omx
		 * to avoid omx release and disable larb
		 * which may cause smi dump devapc
		 */
		//smi_debug_bus_hang_detect(0, "VDEC");

		int i;
		/* wake up the threads in daemon
		 * clear all pending ipi_msg
		 * release worker waiting timeout
		 */
		vcu->abort = true;
		for (i = 0; i < IPI_MAX; i++)
			vcu->ipi_id_ack[i] = true;

		/* wait for GCE done & let IPI ack power off */
		while (
		atomic_read(&vcu->gce_job_cnt[VCU_VDEC][0]) > 0 ||
		atomic_read(&vcu->gce_job_cnt[VCU_VDEC][1]) > 0 ||
		atomic_read(&vcu->gce_job_cnt[VCU_VENC][0]) > 0 ||
		atomic_read(&vcu->gce_job_cnt[VCU_VENC][1]) > 0) {
			wait_cnt++;
			if (wait_cnt > 5) {
				pr_info("[VCU] Vpud killed gce status %d %d\n",
				atomic_read(
				&vcu->gce_job_cnt[VCU_VDEC][0]),
				atomic_read(
				&vcu->gce_job_cnt[VCU_VENC][0]));
				break;
			}
			usleep_range(10000, 20000);
		}

		for (i = 0; i < VCU_CODEC_MAX; i++) {
			atomic_set(&vcu->ipi_got[i], 1);
			atomic_set(&vcu->ipi_done[i], 0);
			memset(&vcu->user_obj[i], 0,
				sizeof(struct share_obj));
			wake_up(&vcu->get_wq[i]);
			wake_up(&vcu->ack_wq[i]);
		}

		atomic_set(&vcu->vdec_log_got, 1);
		wake_up(&vcu->vdec_log_get_wq);
		if (vcu->vcuid == VCU_ID_VDEC)
			vcu_get_task_mt8395(NULL, 1);

		dev_info(vcu->dev, "[VCU] vpud killing\n");

		return 0;
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
	int vcuid = 0;
	struct mtk_vcu_queue *vcu_queue;
	struct mtk_vcu *vcu;

	if (strcmp(current->comm, "camd") == 0)
		vcuid = VCU_ID_CAM;
	else if (strcmp(current->comm, "mdpd") == 0)
		vcuid = VCU_ID_MDP;
	else if (strcmp(current->comm, "vpud") == 0) {
		vcuid = VCU_ID_VDEC;
		vcu = vcu_mtkdev[vcuid];
		mutex_lock(&vcu->vpud_task_mutex);
		if (vcu->vcud_task &&
			(current->tgid != vcu->vcud_task->tgid ||
			current->group_leader != vcu->vcud_task->group_leader)) {
			mutex_unlock(&vcu->vpud_task_mutex);
			return -EACCES;
		}
		vcu->vcud_task = current->group_leader;
		mutex_unlock(&vcu->vpud_task_mutex);
	} else if (strcmp(current->comm, "vdec_srv") == 0 ||
		strcmp(current->comm, "venc_srv") == 0) {
		vcuid = VCU_ID_VDEC;
	} else {
		pr_info("[VCU] thread name: %s\n", current->comm);
	}

	vcu_mtkdev[vcuid]->vcuid = vcuid;

	/* Codec power domains on before the opener can map their banks. */
	if (vcu_mtkdev[vcuid]->pd_list) {
		int ret = pm_runtime_resume_and_get(vcu_mtkdev[vcuid]->dev);

		if (ret < 0) {
			dev_err(vcu_mtkdev[vcuid]->dev,
				"[VCU] codec power domains on: %d\n", ret);
			return ret;
		}
	}

	if (strcmp(current->comm, "vdec_srv") == 0 || vcu_mtkdev[vcuid]->dev_io_enc == NULL)
		vcu_queue = mtk_vcu_mem_init(vcu_mtkdev[vcuid]->dev, NULL);
	else
		vcu_queue = mtk_vcu_mem_init(vcu_mtkdev[vcuid]->dev_io_enc, NULL);

	if (vcu_queue == NULL) {
		pr_info("Allocate new vcu queue fail!\n");
		if (vcu_mtkdev[vcuid]->pd_list)
			pm_runtime_put(vcu_mtkdev[vcuid]->dev);
		return -ENOMEM;
	}

	vcu_queue->vcu = vcu_mtkdev[vcuid];
	vcu = vcu_mtkdev[vcuid];
	mutex_lock(&vcu->vcu_dev_mutex);
	vcu_queue->enable_vcu_dbg_log = vcu->enable_vcu_dbg_log;
	file->private_data = vcu_queue;
	vcu->vpud_killed.count = 0;
	atomic_inc(&vcu->open_cnt);
	vcu->abort = false;
	vcu->vpud_is_going_down = 0;

	mutex_unlock(&vcu->vcu_dev_mutex);

	pr_info("[VCU] %s name: %s pid %d tgid %d open_cnt %d current %p group_leader %p\n",
		__func__, current->comm, current->pid, current->tgid,
		atomic_read(&vcu->open_cnt), current, current->group_leader);

	return 0;
}

static int mtk_vcu_release(struct inode *inode, struct file *file)
{
	unsigned long flags;
	struct mtk_vcu *vcu;
	struct mtk_vcu_queue *vcu_queue;

	vcu_queue = (struct mtk_vcu_queue *)file->private_data;
	if (!vcu_queue) {
		pr_info("error vcu_queue is null\n");
		return -1;
	}

	vcu = vcu_queue->vcu;
	if (!vcu) {
		pr_info("error vcu is null\n");
		return -1;
	}
	mutex_lock(&vcu->vcu_dev_mutex);

	pr_info("[VCU] %s name: %s pid %d open_cnt %d\n", __func__,
		current->comm, current->tgid, atomic_read(&vcu->open_cnt));
	if (atomic_dec_and_test(&vcu->open_cnt)) {
		/* reset vpud due to abnormal situations. */
		vcu->abort = true;
		if (vcu->vcud_task && vcu->vcuid == VCU_ID_VDEC)
			vcu_get_task_mt8395(NULL, 1);
		up(&vcu->vpud_killed);  /* vdec worker */
		up(&vcu->vpud_killed);  /* venc worker */

		/* reset vpud_is_going_down only on abnormal situations */
		spin_lock_irqsave(&vcu->vpud_sig_lock, flags);
		vcu->vpud_is_going_down = 0;
		spin_unlock_irqrestore(&vcu->vpud_sig_lock, flags);
#if ENABLE_GCE
		if (vcu->curr_ctx[VCU_VDEC])
			vcu->cbf.vdec_realease_lock(vcu->curr_ctx[VCU_VDEC]);
#endif
	} else {
		if (vcu->vcuid == VCU_ID_VDEC)
			vcu_get_task_mt8395(NULL, 2);
	}
	mtk_vcu_mem_release(vcu_queue, vcu->clt_venc[0]);

	mutex_unlock(&vcu->vcu_dev_mutex);

	/* Balances the get in mtk_vcu_open. */
	if (vcu->pd_list)
		pm_runtime_put(vcu->dev);

	return 0;
}

static void vcu_free_d_ext_mem(struct mtk_vcu *vcu)
{
	mutex_lock(&vcu->vcu_share);
	mutex_lock(&vcu->vcu_mutex[VCU_VDEC]);
	mutex_lock(&vcu->vcu_mutex[VCU_VENC]);
	kfree(VCU_DMEM0_VIRT(vcu));
	VCU_DMEM0_VIRT(vcu) = NULL;
	mutex_unlock(&vcu->vcu_mutex[VCU_VENC]);
	mutex_unlock(&vcu->vcu_mutex[VCU_VDEC]);
	mutex_unlock(&vcu->vcu_share);
}

static int vcu_alloc_d_ext_mem(struct mtk_vcu *vcu, unsigned long len)
{
	mutex_lock(&vcu->vcu_share);
	mutex_lock(&vcu->vcu_mutex[VCU_VDEC]);
	mutex_lock(&vcu->vcu_mutex[VCU_VENC]);
	VCU_DMEM0_VIRT(vcu) = kmalloc(len, GFP_KERNEL);
	VCU_DMEM0_PHY(vcu) = virt_to_phys(VCU_DMEM0_VIRT(vcu));
	VCU_DMEM0_LEN(vcu) = len;
	mutex_unlock(&vcu->vcu_mutex[VCU_VENC]);
	mutex_unlock(&vcu->vcu_mutex[VCU_VDEC]);
	mutex_unlock(&vcu->vcu_share);

	if (!VCU_DMEM0_VIRT(vcu))
		return -1;

	dev_dbg(vcu->dev,
		"[VCU] Data extend memory (len:%lu) phy=0x%llx virt=0x%p iova=0x%llx\n",
		VCU_DMEM0_LEN(vcu),
		(unsigned long long)VCU_DMEM0_PHY(vcu),
		VCU_DMEM0_VIRT(vcu),
		(unsigned long long)VCU_DMEM0_IOVA(vcu));
	return 0;
}

static void mtk_vcu_page_vm_open(struct vm_area_struct *vma)
{
	struct vcu_pa_pages *vcu_page = (struct vcu_pa_pages *)vma->vm_private_data;
	atomic_inc(&vcu_page->ref_cnt);
	vcu_dbg_log("[VCU] %s vma->start 0x%lx, end 0x%lx, pgoff 0x%lx, ref_cnt %d\n",
		__func__, vma->vm_start, vma->vm_end, vma->vm_pgoff,
		atomic_read(&vcu_page->ref_cnt));
}

static void mtk_vcu_page_vm_close(struct vm_area_struct *vma)
{
	struct vcu_pa_pages *vcu_page = (struct vcu_pa_pages *)vma->vm_private_data;

	if (atomic_read(&vcu_page->ref_cnt) > 0)
		atomic_dec(&vcu_page->ref_cnt);
	else
		pr_info("[VCU][Error] %s ummap fail\n", __func__);

	vcu_dbg_log("[VCU] %s vma->start 0x%lx, end 0x%lx, pgoff 0x%lx, ref_cnt %d\n",
		__func__, vma->vm_start, vma->vm_end, vma->vm_pgoff,
		atomic_read(&vcu_page->ref_cnt));
}

const struct vm_operations_struct mtk_vcu_page_vm_ops = {
	.open = mtk_vcu_page_vm_open,
	.close = mtk_vcu_page_vm_close,
};

static void mtk_vcu_buf_vm_close(struct vm_area_struct *vma)
{
	void *mem_priv = (void *)vma->vm_private_data;
	struct file *file = vma->vm_file;
	struct mtk_vcu_queue *vcu_queue =
		(struct mtk_vcu_queue *)file->private_data;

	mtk_vcu_buffer_ref_dec(vcu_queue, mem_priv);
	vcu_dbg_log("[VCU] %s vma->start 0x%lx, end 0x%lx, pgoff 0x%lx mem_priv %lx\\n",
		 __func__, vma->vm_start, vma->vm_end,
		 vma->vm_pgoff, (unsigned long)mem_priv);
}

const struct vm_operations_struct mtk_vcu_buf_vm_ops = {
	.close = mtk_vcu_buf_vm_close,
};

static int mtk_vcu_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long length = vma->vm_end - vma->vm_start;
	unsigned long pa_start = vma->vm_pgoff << PAGE_SHIFT;
	unsigned long pa_start_base = pa_start;
	unsigned long pa_end = pa_start + (int)length;
#if IS_ENABLED(CONFIG_MTK_IOMMU)
	unsigned long start = vma->vm_start;
	unsigned long pos = 0;
#endif
	struct mtk_vcu *vcu_dev;
	struct mtk_vcu_queue *vcu_queue =
		(struct mtk_vcu_queue *)file->private_data;
	struct mem_obj mem_buff_data;
	struct vb2_buffer *src_vb, *dst_vb;
	void *ret = NULL;
	struct iommu_domain *io_domain;

	vcu_dev = (struct mtk_vcu *)vcu_queue->vcu;
	vcu_dbg_log("[VCU] %s vma->start 0x%lx, end 0x%lx, pgoff 0x%lx, pa_start:0x%lx, length:%lld\n",
		 __func__, vma->vm_start, vma->vm_end, vma->vm_pgoff, pa_start, (vma->vm_end - vma->vm_start));
	if (vcu_queue->dev == vcu_dev->dev_io_enc)
		io_domain = vcu_dev->io_domain_enc;
	else
		io_domain = vcu_dev->io_domain;
	// First handle map pa case, because maybe pa will smaller than
	// MAP_PA_BASE_1GB in 32bit project
	if (vcu_queue->map_buf_pa >= MAP_SHMEM_PA_BASE) {
		mutex_lock(&vcu_queue->mmap_lock);
		vcu_queue->map_buf_pa = 0;
		pa_start -= MAP_SHMEM_PA_BASE;
#if ENABLE_GCE
		ret = vcu_check_gce_pa_base(vcu_queue, pa_start, length, true);
#endif
		if (ret != NULL) {
			vma->vm_ops = &mtk_vcu_page_vm_ops;
			vma->vm_private_data = ret;
			vma->vm_pgoff = pa_start >> PAGE_SHIFT;
			vma->vm_page_prot =
				pgprot_writecombine(vma->vm_page_prot);
			mtk_vcu_page_vm_open(vma);
			mutex_unlock(&vcu_queue->mmap_lock);
			goto valid_map;
		}
		mutex_unlock(&vcu_queue->mmap_lock);
		pr_info("[VCU] map pa fail with pa_start=0x%lx\n",
			pa_start);
		return -EINVAL;
	}

	/*only vcud need this case*/
	if (pa_start < MAP_PA_BASE_1GB) {
		if (vcu_check_reg_base(vcu_dev, pa_start, length) == 0) {
			vma->vm_pgoff = pa_start >> PAGE_SHIFT;
			goto reg_valid_map;
		}
	}

	if (pa_start >= MAP_SHMEM_ALLOC_BASE && pa_end <= MAP_SHMEM_ALLOC_END) {
		vma->vm_pgoff =
			(unsigned long)(VCU_DMEM0_PHY(vcu_dev) >> PAGE_SHIFT);
		goto valid_map;
	}

	if (pa_start >= MAP_SHMEM_COMMIT_BASE &&
		pa_end <= MAP_SHMEM_COMMIT_END) {
		VCU_DMEM0_VMA(vcu_dev) = vma->vm_start;
		vma->vm_pgoff =
			(unsigned long)(VCU_DMEM0_PHY(vcu_dev) >> PAGE_SHIFT);
		goto valid_map;
	}

	if (pa_start_base >= MAP_SHMEM_MM_BASE &&
		pa_start_base < MAP_SHMEM_PA_BASE) {
		if (pa_start_base >= MAP_SHMEM_MM_CACHEABLE_BASE)
			pa_start -= MAP_SHMEM_MM_CACHEABLE_BASE;
		else
			pa_start -= MAP_SHMEM_MM_BASE;

		mem_buff_data.iova = pa_start;
		mem_buff_data.len = length;
		src_vb = NULL;
		dst_vb = NULL;
		if (strcmp(current->comm, "vdec_srv") == 0) {
			src_vb = vcu_dev->curr_src_vb[VCU_VDEC];
			dst_vb = vcu_dev->curr_dst_vb[VCU_VDEC];
		} else if (strcmp(current->comm, "venc_srv") == 0) {
			src_vb = vcu_dev->curr_src_vb[VCU_VENC];
			dst_vb = vcu_dev->curr_dst_vb[VCU_VENC];
		}

		ret = mtk_vcu_set_buffer(vcu_queue, &mem_buff_data,
			src_vb, dst_vb);
		if (!IS_ERR_OR_NULL(ret)) {
			vcu_dbg_log("[VCU] mtk_vcu_buf_vm_mmap mem_priv %lx iova %llx\n",
				 (unsigned long)ret, pa_start);

			vma->vm_ops = &mtk_vcu_buf_vm_ops;
			vma->vm_private_data = ret;
			vma->vm_file = file;
		}
#if IS_ENABLED(CONFIG_MTK_IOMMU)
		while (length > 0) {
			vma->vm_pgoff = iommu_iova_to_phys(io_domain,
				(pa_start + pos));
			if (vma->vm_pgoff == 0) {
				dev_info(vcu_dev->dev, "[VCU] iommu_iova_to_phys fail vcu_ptr->iommu_padding = %d pa_start = 0x%lx\n",
					vcu_dev->iommu_padding, pa_start);
				return -EINVAL;
			}
			vma->vm_pgoff >>= PAGE_SHIFT;
			if (pa_start_base < MAP_SHMEM_MM_CACHEABLE_BASE) {
				vma->vm_page_prot =
					pgprot_writecombine(vma->vm_page_prot);
			}
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
	dev_info(vcu_dev->dev, "%s, [VCU] Invalid argument\n", __func__);

	return -EINVAL;

reg_valid_map:
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

valid_map:
	dev_dbg(vcu_dev->dev, "[VCU] Mapping pgoff 0x%lx\n", vma->vm_pgoff);

	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			    vma->vm_end - vma->vm_start,
			    vma->vm_page_prot) != 0)
		return -EAGAIN;

	return 0;
}

static long mtk_vcu_unlocked_ioctl(struct file *file, unsigned int cmd,
				   unsigned long arg)
{
	long ret = -1;
	void *mem_priv;
	unsigned char *user_data_addr = NULL;
	struct mtk_vcu *vcu_dev;
	struct device *dev;
	struct share_obj share_buff_data;
	struct mem_obj mem_buff_data;
	struct mtk_vcu_queue *vcu_queue =
		(struct mtk_vcu_queue *)file->private_data;

	vcu_dev = (struct mtk_vcu *)vcu_queue->vcu;
	dev = vcu_dev->dev;
	switch (cmd) {
	case VCU_SET_OBJECT:
		user_data_addr = (unsigned char *)arg;
		ret = (long)copy_from_user(&share_buff_data, user_data_addr,
			(unsigned long)sizeof(struct share_obj));
		if (ret != 0L || share_buff_data.id >= (int)IPI_MAX ||
		    share_buff_data.id < (int)IPI_VCU_INIT) {
			pr_info("[VCU] %s(%d) Copy data from user failed!\n",
			       __func__, __LINE__);
			return -EINVAL;
		}
		ret = vcu_ipi_handler(vcu_dev, &share_buff_data);
		ret = (long)copy_to_user(user_data_addr, &share_buff_data,
			(unsigned long)sizeof(struct share_obj));
		if (ret != 0L) {
			pr_info("[VCU] %s(%d) Copy data to user failed!\n",
			       __func__, __LINE__);
			return -EINVAL;
		}
		break;
	case VCU_GET_OBJECT:
		ret = vcu_ipi_get(vcu_dev, arg);
		break;
	case VCU_GET_LOG_OBJECT:
		ret = vcu_log_get(vcu_dev, arg);
		break;
	case VCU_MVA_ALLOCATION:
	case VCU_UBE_MVA_ALLOCATION:
	case VCU_PA_ALLOCATION:
		user_data_addr = (unsigned char *)arg;
		ret = (long)copy_from_user(&mem_buff_data, user_data_addr,
			(unsigned long)sizeof(struct mem_obj));
		if (ret != 0L) {
			pr_info("[VCU] %s(%d) Copy data from user failed!\n",
			       __func__, __LINE__);
			return -EINVAL;
		}

		if (cmd == VCU_MVA_ALLOCATION) {
			mutex_lock(&vcu_queue->dev_lock);
			mem_priv = mtk_vcu_get_buffer(vcu_queue, &mem_buff_data);
			mutex_unlock(&vcu_queue->dev_lock);
			mem_buff_data.pa = 0;
		} else if (cmd == VCU_UBE_MVA_ALLOCATION) {
			struct device *io_dev = vcu_queue->dev;
#if IS_ENABLED(CONFIG_MTK_TINYSYS_VCP_SUPPORT)
			vcu_queue->dev = vcp_get_io_device(VCP_IOMMU_UBE_LAT);
#endif
			if (vcu_queue->dev == NULL)
				vcu_queue->dev = io_dev;
			mem_priv = mtk_vcu_get_buffer(vcu_queue, &mem_buff_data);
			vcu_queue->dev = io_dev;
		} else {
			mem_priv =
				mtk_vcu_get_page(vcu_queue, vcu_dev->clt_venc[0], &mem_buff_data);
		}
		if (IS_ERR_OR_NULL(mem_priv) == true) {
			mem_buff_data.va = (unsigned long)-1;
			mem_buff_data.pa = (unsigned long)-1;
			mem_buff_data.iova = (unsigned long)-1;
			ret = (long)copy_to_user(user_data_addr,
				&mem_buff_data,
				(unsigned long)sizeof(struct mem_obj));
			pr_info("[VCU] ALLOCATION %d failed!\n", cmd == VCU_MVA_ALLOCATION);
			return PTR_ERR(mem_priv);
		}

		vcu_dbg_log("[VCU] ALLOCATION %d va %llx, pa %llx, iova %llx\n",
			cmd == VCU_MVA_ALLOCATION, mem_buff_data.va,
			mem_buff_data.pa, mem_buff_data.iova);

		ret = (long)copy_to_user(user_data_addr, &mem_buff_data,
					 (unsigned long)sizeof(struct mem_obj));
		if (ret != 0L) {
			pr_info("[VCU] %s(%d) Copy data to user failed!\n",
			       __func__, __LINE__);
			return -EINVAL;
		}

		/* store map pa buffer type flag which will use in mmap*/
		if (cmd == VCU_PA_ALLOCATION)
			vcu_queue->map_buf_pa = mem_buff_data.pa + MAP_SHMEM_PA_BASE;

		ret = 0;
		break;
	case VCU_MVA_FREE:
	case VCU_UBE_MVA_FREE:
	case VCU_PA_FREE:
		user_data_addr = (unsigned char *)arg;
		ret = (long)copy_from_user(&mem_buff_data, user_data_addr,
			(unsigned long)sizeof(struct mem_obj));
		if ((ret != 0L) ||
			(mem_buff_data.iova == 0UL &&
			mem_buff_data.va == 0UL)) {
			pr_info("[VCU] %s(%d) Free buf failed!\n",
			       __func__, __LINE__);
			return -EINVAL;
		}

		if (cmd == VCU_MVA_FREE) {
			mutex_lock(&vcu_queue->dev_lock);
			ret = mtk_vcu_free_buffer(vcu_queue, &mem_buff_data);
			mutex_unlock(&vcu_queue->dev_lock);
		} else if (cmd == VCU_UBE_MVA_FREE) {
			struct device *io_dev = vcu_queue->dev;
#if IS_ENABLED(CONFIG_MTK_TINYSYS_VCP_SUPPORT)
			vcu_queue->dev = vcp_get_io_device(VCP_IOMMU_UBE_LAT);
#endif
			if (vcu_queue->dev == NULL)
				vcu_queue->dev = io_dev;
			ret = mtk_vcu_free_buffer(vcu_queue, &mem_buff_data);
			vcu_queue->dev = io_dev;
		} else {
			ret = mtk_vcu_free_page(vcu_queue, vcu_dev->clt_venc[0], &mem_buff_data);
		}

		if (ret != 0L) {
			pr_info("[VCU] VCU_FREE failed %d va %llx, pa %llx, iova %llx\n",
				cmd == VCU_MVA_FREE, mem_buff_data.va,
				mem_buff_data.pa, mem_buff_data.iova);
			return -EINVAL;
		}

		vcu_dbg_log("[VCU] FREE %d va %llx, pa %llx, iova %llx\n",
			cmd == VCU_MVA_FREE, mem_buff_data.va,
			mem_buff_data.pa, mem_buff_data.iova);

		mem_buff_data.va = 0;
		mem_buff_data.iova = 0;
		mem_buff_data.pa = 0;

		ret = (long)copy_to_user(user_data_addr, &mem_buff_data,
					 (unsigned long)sizeof(struct mem_obj));
		if (ret != 0L) {
			pr_info("[VCU] %s(%d) Copy data to user failed!\n",
			       __func__, __LINE__);
			return -EINVAL;
		}
		ret = 0;
		break;
	case VCU_CACHE_FLUSH_ALL:
		dev_dbg(dev, "[VCU] Flush cache in kernel\n");
		vcu_buffer_flush_all(dev, vcu_queue);
		ret = 0;
		break;
	case VCU_CACHE_FLUSH_BUFF:
	case VCU_CACHE_INVALIDATE_BUFF:
		user_data_addr = (unsigned char *)arg;
		ret = (long)copy_from_user(&mem_buff_data, user_data_addr,
			(unsigned long)sizeof(struct mem_obj));
		if (ret != 0L) {
			pr_info("[VCU] %s(%d) Copy data from user failed!\n",
			       __func__, __LINE__);
			return -EINVAL;
		}
		ret = vcu_buffer_cache_sync(dev, vcu_queue,
			(dma_addr_t)mem_buff_data.iova,
			(size_t)mem_buff_data.len,
			(cmd == VCU_CACHE_FLUSH_BUFF) ?
				DMA_TO_DEVICE : DMA_FROM_DEVICE);
		if (ret < 0)
			return -EINVAL;

		dev_dbg(dev, "[VCU] Cache flush buffer pa = %llx, size = %d\n",
			mem_buff_data.iova, (unsigned int)mem_buff_data.len);

		ret = (long)copy_to_user(user_data_addr, &mem_buff_data,
					 (unsigned long)sizeof(struct mem_obj));
		if (ret != 0L) {
			pr_info("[VCU] %s(%d) Copy data to user failed!\n",
			       __func__, __LINE__);
			return -EINVAL;
		}
		ret = 0;
		break;
#if ENABLE_GCE
	case VCU_GCE_SET_CMD_FLUSH:
		ret = vcu_gce_cmd_flush(vcu_dev, vcu_queue, arg);
		break;
	case VCU_GCE_WAIT_CALLBACK:
		ret = vcu_wait_gce_callback(vcu_dev, arg);
		break;
#endif
	case VCU_GET_DISP_MAPPED_IOVA:
		ret = vcu_get_disp_mapped_iova(vcu_dev, arg);
		break;
	case VCU_CLEAR_DISP_MAPPED_IOVA:
		vcu_clear_disp_mapped_iova(vcu_dev, arg);
		ret = 0;
		break;
	case VCU_GET_DISP_WDMA_Y_ADDR:
		ret = vcu_get_disp_wdma_y_addr(vcu_dev, arg);
		break;
	case VCU_GET_SEC_HANDLE:
		ret = vcu_sec_handle_get(vcu_dev, vcu_queue, arg);
		break;
	default:
		dev_info(dev, "[VCU] Unknown cmd\n");
		break;
	}

	return ret;
}

#if IS_ENABLED(CONFIG_COMPAT)
static int compat_get_vpud_allocation_data(
	struct compat_mem_obj __user *data32,
	struct mem_obj __user *data)
{
	compat_ulong_t l;
	compat_u64 u;
	unsigned int err = 0;

	err = get_user(u, &data32->iova);
	err |= put_user(u, &data->iova);
	err |= get_user(l, &data32->len);
	err |= put_user(l, &data->len);
	err |= get_user(u, &data32->pa);
	err |= put_user(u, &data->pa);
	err |= get_user(u, &data32->va);
	err |= put_user(u, &data->va);

	return (int)err;
}

static int compat_put_vpud_allocation_data(
	struct compat_mem_obj __user *data32,
	struct mem_obj __user *data)
{
	compat_ulong_t l;
	compat_u64 u;
	unsigned int err = 0;

	err = get_user(u, &data->iova);
	err |= put_user(u, &data32->iova);
	err |= get_user(l, &data->len);
	err |= put_user(l, &data32->len);
	err |= get_user(u, &data->pa);
	err |= put_user(u, &data32->pa);
	err |= get_user(u, &data->va);
	err |= put_user(u, &data32->va);

	return (int)err;
}

static long mtk_vcu_unlocked_compat_ioctl(struct file *file, unsigned int cmd,
					  unsigned long arg)
{
	int err = 0;
	long ret = -1;
	struct share_obj __user *share_data32;
	struct compat_mem_obj __user *data32;
	struct mem_obj __user *data;
	struct sec_handle_obj __user *sec_handle_data;

	switch (cmd) {
	case COMPAT_VCU_SET_OBJECT:
	case VCU_GET_OBJECT:
	case VCU_GET_LOG_OBJECT:
	case VCU_GCE_SET_CMD_FLUSH:
	case VCU_GCE_WAIT_CALLBACK:
	case VCU_GET_DISP_MAPPED_IOVA:
	case VCU_CLEAR_DISP_MAPPED_IOVA:
	case VCU_GET_DISP_WDMA_Y_ADDR:
		share_data32 = compat_ptr((uint32_t)arg);
		ret = file->f_op->unlocked_ioctl(file,
			cmd, (unsigned long)share_data32);
		break;
	case COMPAT_VCU_MVA_ALLOCATION:
	case COMPAT_VCU_UBE_MVA_ALLOCATION:
	case COMPAT_VCU_PA_ALLOCATION:
		data32 = compat_ptr((uint32_t)arg);
		data = compat_alloc_user_space(sizeof(struct mem_obj));
		if (data == NULL)
			return -EFAULT;

		err = compat_get_vpud_allocation_data(data32, data);
		if (err != 0)
			return err;
		if (cmd == COMPAT_VCU_MVA_ALLOCATION)
			ret = file->f_op->unlocked_ioctl(file,
				(uint32_t)VCU_MVA_ALLOCATION,
				(unsigned long)data);
		else if (cmd == COMPAT_VCU_UBE_MVA_ALLOCATION)
			ret = file->f_op->unlocked_ioctl(file,
				(uint32_t)VCU_UBE_MVA_ALLOCATION,
				(unsigned long)data);
		else
			ret = file->f_op->unlocked_ioctl(file,
				(uint32_t)VCU_PA_ALLOCATION,
				(unsigned long)data);
		err = compat_put_vpud_allocation_data(data32, data);
		if (err != 0)
			return err;
		break;
	case COMPAT_VCU_MVA_FREE:
	case COMPAT_VCU_UBE_MVA_FREE:
	case COMPAT_VCU_PA_FREE:
		data32 = compat_ptr((uint32_t)arg);
		data = compat_alloc_user_space(sizeof(struct mem_obj));
		if (data == NULL)
			return -EFAULT;

		err = compat_get_vpud_allocation_data(data32, data);
		if (err != 0)
			return err;
		if (cmd == COMPAT_VCU_MVA_FREE)
			ret = file->f_op->unlocked_ioctl(file,
				(uint32_t)VCU_MVA_FREE, (unsigned long)data);
		else if (cmd == COMPAT_VCU_UBE_MVA_FREE)
			ret = file->f_op->unlocked_ioctl(file,
				(uint32_t)VCU_UBE_MVA_FREE, (unsigned long)data);
		else
			ret = file->f_op->unlocked_ioctl(file,
				(uint32_t)VCU_PA_FREE, (unsigned long)data);
		err = compat_put_vpud_allocation_data(data32, data);
		if (err != 0)
			return err;
		break;
	case COMPAT_VCU_CACHE_FLUSH_ALL:
		ret = file->f_op->unlocked_ioctl(file,
			(uint32_t)VCU_CACHE_FLUSH_ALL, 0);
		break;
	case COMPAT_VCU_CACHE_FLUSH_BUFF:
	case COMPAT_VCU_CACHE_INVALIDATE_BUFF:
		data32 = compat_ptr((uint32_t)arg);
		data = compat_alloc_user_space(sizeof(struct mem_obj));
		if (data == NULL)
			return -EFAULT;

		err = compat_get_vpud_allocation_data(data32, data);
		if (err != 0)
			return err;
		if (cmd == COMPAT_VCU_CACHE_FLUSH_BUFF) {
			ret = file->f_op->unlocked_ioctl(file,
				(uint32_t)VCU_CACHE_FLUSH_BUFF,
				(unsigned long)data);
		} else {
			ret = file->f_op->unlocked_ioctl(file,
				(uint32_t)VCU_CACHE_INVALIDATE_BUFF,
				(unsigned long)data);
		}

		err = compat_put_vpud_allocation_data(data32, data);
		if (err != 0)
			return err;
		break;
	case VCU_GET_SEC_HANDLE:
		sec_handle_data = compat_ptr((uint32_t)arg);
		ret = file->f_op->unlocked_ioctl(file,
			cmd, (unsigned long)sec_handle_data);
		break;
	default:
		pr_info("[VCU] Invalid cmd_number 0x%x\n", cmd);
		break;
	}
	return ret;
}
#endif

static int mtk_vcu_write(const char *val, const struct kernel_param *kp)
{
	long ret = -1;
	int i;
	struct mtk_vcu *vcu;

	for (i = 0; i < MTK_VCU_NR_MAX; i++) {
		vcu = vcu_mtkdev[i];

		if (vcu != NULL &&
			vcu->vdec_log_info != NULL &&
			val != NULL) {
			ret = param_set_charp(val, kp);
			if (ret != 0)
				return -EINVAL;

			memcpy(vcu->vdec_log_info->log_info,
				val, strnlen(val, LOG_INFO_SIZE - 1) + 1);
		} else
			return -EFAULT;

		vcu->vdec_log_info->log_info[LOG_INFO_SIZE - 1] = '\0';

		// check if need to enable VCU debug log
		if (strstr(vcu->vdec_log_info->log_info, "vcu_log 1")) {
			vcu->enable_vcu_dbg_log = 1;
			enable_vcu_dbg_log = 1;
			return 0;
		} else if (strstr(vcu->vdec_log_info->log_info, "vcu_log 0")) {
			vcu->enable_vcu_dbg_log = 0;
			enable_vcu_dbg_log = 0;
			return 0;
		}

		pr_info("[log wakeup VPUD] log_info %p vcu_ptr %p val %p: %s %lu\n",
			(char *)vcu->vdec_log_info->log_info,
			vcu, val, val,
			(unsigned long)strnlen(val, LOG_INFO_SIZE - 1) + 1);

		atomic_set(&vcu->vdec_log_got, 1);
		wake_up(&vcu->vdec_log_get_wq);
	}

	return 0;
}

static struct kernel_param_ops log_param_ops = {
	.set = mtk_vcu_write,
	.get = param_get_charp,
};

module_param_cb(test_info, &log_param_ops, &vcodec_param_string, 0644);

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

/**
 * Suspend callbacks after user space processes are frozen
 * Since user space processes are frozen, there is no need and cannot hold same
 * mutex that protects lock owner while checking status.
 * If video codec hardware is still active now, must not to enter suspend.
 **/
static int mtk_vcu_suspend(struct device *pDev)
{
	struct mtk_vcu *vcu = (struct mtk_vcu *)pDev->driver_data;

	if (!vcu) {
		pr_info("%s invalid driver data\n", __func__);
		return 0;
	}

	if (atomic_read(&vcu->ipi_done[VCU_VDEC]) == 0 ||
		atomic_read(&vcu->ipi_done[VCU_VENC]) == 0 ||
		atomic_read(&vcu->gce_job_cnt[VCU_VDEC][0]) > 0 ||
		atomic_read(&vcu->gce_job_cnt[VCU_VDEC][1]) > 0 ||
		atomic_read(&vcu->gce_job_cnt[VCU_VENC][0]) > 0 ||
		atomic_read(&vcu->gce_job_cnt[VCU_VENC][1]) > 0) {
		pr_info("[VCU] %s fail due to videocodec activity\n", __func__);
		return -EBUSY;
	}
	pr_info("[VCU] %s done\n", __func__);
	return 0;
}

static int mtk_vcu_resume(struct device *pDev)
{
	pr_info("[VCU] %s done\n", __func__);
	return 0;
}

/**
 * Suspend notifiers before user space processes are frozen.
 * User space driver can still complete decoding/encoding of current frame.
 * Change state to is_entering_suspend to stop send ipi_msg but allow current
 * wait ipi_msg to be done.
 * Since there is no critical section proection, it is possible for a new task
 * to start after changing to is_entering_suspend state. This case will be
 * handled by suspend callback mtk_vcu_suspend.
 **/
static int mtk_vcu_suspend_notifier(struct notifier_block *nb,
					unsigned long action, void *data)
{
	int wait_cnt = 0;
	struct mtk_vcu *vcu =
			container_of(nb, struct mtk_vcu, pm_notifier);

	pr_info("[VCU] %s ok action = %ld\n", __func__, action);
	switch (action) {
	case PM_SUSPEND_PREPARE:
		vcu->is_entering_suspend = 1;
		while (atomic_read(&vcu->ipi_done[VCU_VDEC]) == 0 ||
			atomic_read(&vcu->ipi_done[VCU_VENC]) == 0 ||
			atomic_read(&vcu->gce_job_cnt[VCU_VDEC][0]) > 0 ||
			atomic_read(&vcu->gce_job_cnt[VCU_VDEC][1]) > 0 ||
			atomic_read(&vcu->gce_job_cnt[VCU_VENC][0]) > 0 ||
			atomic_read(&vcu->gce_job_cnt[VCU_VENC][1]) > 0) {
			wait_cnt++;
			if (wait_cnt > 5) {
				pr_info("vcodec_pm_suspend waiting %d %d %d %d %d %d\n",
				  atomic_read(&vcu->ipi_done[VCU_VDEC]),
				  atomic_read(&vcu->ipi_done[VCU_VENC]),
				  atomic_read(
				    &vcu->gce_job_cnt[VCU_VDEC][0]),
				  atomic_read(
				    &vcu->gce_job_cnt[VCU_VDEC][1]),
				  atomic_read(
				    &vcu->gce_job_cnt[VCU_VENC][0]),
				  atomic_read(
				    &vcu->gce_job_cnt[VCU_VENC][1]));
				/* Current task is still not finished, don't
				 * care, will check again in real suspend
				 */
				return NOTIFY_DONE;
			}
			usleep_range(10000, 20000);
		}
		return NOTIFY_OK;
	case PM_POST_SUSPEND:
		vcu->is_entering_suspend = 0;
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
	int i, index, ret = 0;
	unsigned int vcuid, off = 0;

#if ENABLE_GCE
	const char *gce_event_name;
	int gce_event_id;
	int j;
#endif
	dev_dbg(&pdev->dev, "[VCU] initialization\n");

	dev = &pdev->dev;

	ret = of_property_read_u32(pdev->dev.of_node, "mediatek,vcu-off", &off);
	if (off) {
		mtk_vcodec_vcp_mt8395 = 3;
		pr_info("[VCU] VCU off\n");
	}

	ret = of_property_read_u32(dev->of_node, "mediatek,vcuid", &vcuid);
	if (ret != 0) {
		dev_info(dev, "[VCU] failed to find mediatek,vcuid\n");
		return ret;
	}

	vcu = devm_kzalloc(dev, sizeof(*vcu), GFP_KERNEL);
	if (vcu == NULL)
		return -ENOMEM;

	vcu_mtkdev[vcuid] = vcu;

	/*
	 * vpud maps the VDEC and VENC register banks through /dev/vcu and
	 * touches them directly, outside the codec drivers' runtime PM. Hold
	 * the three codec power domains for as long as /dev/vcu is open
	 * (mtk_vcu_open/mtk_vcu_release), so they are on whenever vpud can
	 * reach those banks, and genpd is free to power them off otherwise --
	 * in system suspend in particular.
	 */
	if (of_property_present(dev->of_node, "power-domains")) {
		static const char * const pd_names[] = { "vdec0", "vdec1", "venc" };
		const struct dev_pm_domain_attach_data pd_data = {
			.pd_names = pd_names,
			.num_pd_names = ARRAY_SIZE(pd_names),
		};

		ret = devm_pm_domain_attach_list(dev, &pd_data, &vcu->pd_list);
		if (ret < 0) {
			dev_err_probe(dev, ret, "[VCU] codec power domains\n");
			goto err_ipi_init;
		}
		ret = devm_pm_runtime_enable(dev);
		if (ret) {
			dev_err_probe(dev, ret, "[VCU] runtime PM\n");
			goto err_ipi_init;
		}
	}

#if IS_ENABLED(CONFIG_MTK_IOMMU)
	vcu_mtkdev[vcuid]->io_domain = iommu_get_domain_for_dev(dev);
	if (vcu_mtkdev[vcuid]->io_domain == NULL) {
		dev_info(dev,
			"[VCU] vcuid: %d get io_domain fail !!\n", vcuid);
		return -EPROBE_DEFER;
	}
	dev_dbg(dev, "vcu io_domain: %p,vcuid:%d\n",
		vcu_mtkdev[vcuid]->io_domain,
		vcuid);
	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(34));
	if (ret) {
		ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(34));
		if (ret) {
			dev_info(&pdev->dev, "64-bit DMA enable failed\n");
			return ret;
		}
		vcu->iommu_padding = 0;
	} else
		vcu->iommu_padding = 0;
#endif

	if (vcuid == VCU_ID_CAM)
		vcu_mtkdev[vcuid]->path = CAM_PATH;
	else if (vcuid == VCU_ID_MDP)
		vcu_mtkdev[vcuid]->path = MDP_PATH;
	else if (vcuid == VCU_ID_VDEC) {
		vcu_mtkdev[vcuid]->vdec_log_info = devm_kzalloc(dev,
			sizeof(struct log_test_nofuse), GFP_KERNEL);
		pr_info("[VCU] vdec_log_info %p %d vcuid %d vcu_ptr %p, iommu_padding:%d\n",
		vcu_mtkdev[vcuid]->vdec_log_info,
		(int)sizeof(struct log_test_nofuse),
		(int)vcuid, vcu,
		vcu->iommu_padding);
		vcu_mtkdev[vcuid]->path = VCU_PATH;
	} else
		return -ENXIO;

	ret = of_property_read_string(dev->of_node, "mediatek,vcuname",
				      &vcu_mtkdev[vcuid]->vcuname);
	if (ret != 0) {
		dev_info(dev, "[VCU] failed to find mediatek,vcuname\n");
		return ret;
	}

	vcu->dev = &pdev->dev;
	platform_set_drvdata(pdev, vcu_mtkdev[vcuid]);

	if (vcuid == VCU_ID_VDEC) {
		for (i = 0; i < (int)VCU_MAP_HW_REG_NUM; i++) {
			res = platform_get_resource(pdev, IORESOURCE_MEM, i);
			if (res == NULL)
				break;

			vcu->map_base[i].base = res->start;
			vcu->map_base[i].len = resource_size(res);
			dev_dbg(dev, "[VCU] base[%d]: 0x%lx 0x%lx",
				i, vcu->map_base[i].base,
				vcu->map_base[i].len);
		}
	}
	dev_dbg(dev, "[VCU] vcu ipi init\n");
	ret = vcu_ipi_init(vcu);
	if (ret != 0) {
		dev_info(dev, "[VCU] Failed to init ipi\n");
		goto err_ipi_init;
	}

	/* register vcu initialization IPI */
	ret = vcu_ipi_register_mt8395(pdev, IPI_VCU_INIT, vcu_init_ipi_handler,
			       "vcu_init", vcu);
	if (ret != 0) {
		dev_info(dev, "Failed to register IPI_VCU_INIT\n");
		goto vcu_mutex_destroy;
	}

	for (i = 0; i < (int)VCU_CODEC_MAX; i++) {
		init_waitqueue_head(&vcu->ack_wq[i]);
		init_waitqueue_head(&vcu->get_wq[i]);
#if ENABLE_GCE
		for (j = 0; j < (int)GCE_THNUM_MAX; j++)
			init_waitqueue_head(&vcu->gce_wq[i][j]);
#endif
		atomic_set(&vcu->ipi_got[i], 0);
		atomic_set(&vcu->ipi_done[i], 1);
#if ENABLE_GCE
		for (j = 0; j < (int)GCE_THNUM_MAX; j++)
			atomic_set(&vcu->gce_job_cnt[i][j], 0);
#endif
	}
	init_waitqueue_head(&vcu->vdec_log_get_wq);
	atomic_set(&vcu->vdec_log_got, 0);
#if ENABLE_GCE
	for (i = 0; i < (int)VCODEC_INST_MAX; i++) {
		for (j = 0; j < GCE_THNUM_MAX; j++)
			atomic_set(&vcu->gce_info[i].flush_done[j], 0);
		atomic_set(&vcu->gce_info[i].flush_pending, 0);
		vcu->gce_info[i].user_hdl = 0;
		vcu->gce_info[i].v4l2_ctx = NULL;
		for (index = 0; index < (int)GCE_PENDING_CNT; index++) {
			INIT_LIST_HEAD(&vcu->gce_info[i].used_pages[index].list);
			sema_init(&vcu->gce_info[i].buff_sem[index], 1);
		}
	}
#endif

	atomic_set(&vcu->open_cnt, 0);
	mutex_init(&vcu->vcu_dev_mutex);

#if ENABLE_GCE
	ret = of_property_read_u32(dev->of_node, "mediatek,dec_gce_th_num",
				      &vcu->gce_th_num[VCU_VDEC]);
	if (!ret && vcu->gce_th_num[VCU_VDEC] > GCE_THNUM_MAX)
		vcu->gce_th_num[VCU_VDEC] = 1;

	ret = of_property_read_u32(dev->of_node, "mediatek,enc_gce_th_num",
				      &vcu->gce_th_num[VCU_VENC]);
	if (!ret && vcu->gce_th_num[VCU_VENC] > GCE_THNUM_MAX)
		vcu->gce_th_num[VCU_VENC] = 1;

	for (i = 0; i < ARRAY_SIZE(vcu->gce_gpr); i++) {
		ret = of_property_read_u32_index(dev->of_node, "gce-gpr",
			i, &vcu->gce_gpr[i]);
		if (ret < 0)
			break;
	}

	for (i = 0; i < vcu->gce_th_num[VCU_VDEC]; i++) {
		vcu->clt_vdec[i] = cmdq_mbox_create(dev, i);
		if (IS_ERR(vcu->clt_vdec[i])) {
			dev_warn(dev, "Failed to create cmdq client for vdec %d\n", i);
			vcu->clt_vdec[i] = NULL;
		}

		if (vcu->clt_vdec[i])
			vcu->clt_vdec[i]->client.rx_callback = vcu_gce_flush_callback;
	}
	for (i = 0; i < vcu->gce_th_num[VCU_VENC]; i++) {
		vcu->clt_venc[i] =
			cmdq_mbox_create(dev, i + vcu->gce_th_num[VCU_VDEC]);
		if (IS_ERR(vcu->clt_venc[i])) {
			dev_warn(dev, "Failed to create cmdq client for venc %d\n", i);
			vcu->clt_venc[i] = NULL;
		}

		if (vcu->clt_venc[i])
			vcu->clt_venc[i]->client.rx_callback = vcu_gce_flush_callback;

		vcu->gce_callback_thread_wq[i] =
		alloc_ordered_workqueue(VCU_DEVNAME,
								WQ_MEM_RECLAIM |
								WQ_FREEZABLE);
		if (!vcu->gce_callback_thread_wq[i]) {
			pr_err("[VCU]Failed to create gce loop workqueue");
			break;;
		}
	}

#if IS_ENABLED(CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT)
	for (i = 0; i < vcu->gce_th_num[VCU_VENC]; i++)
		vcu->clt_venc_sec[i] =
			cmdq_mbox_create(dev,
			i + vcu->gce_th_num[VCU_VDEC] + vcu->gce_th_num[VCU_VENC]);
#endif

	dev_dbg(dev, "[VCU] GCE clt_vdec %d %p %p clt_venc %d %p %p sec %p %p dev %p",
		vcu->gce_th_num[VCU_VDEC],
		vcu->clt_vdec[0], vcu->clt_vdec[1],
		vcu->gce_th_num[VCU_VENC], vcu->clt_venc[0],
		vcu->clt_venc[1], vcu->clt_venc_sec[0], vcu->clt_venc_sec[1], dev);
	for (i = 0; i < GCE_EVENT_MAX; i++)
		vcu->gce_codec_eid[i] = -1;

	i = 0;
	while (!of_property_read_string_index(
			dev->of_node, "gce-event-names", i, &gce_event_name)) {
		gce_event_id = find_gce_event_id(gce_event_name);
		if (gce_event_id != -1) {
			if (of_property_read_u32_index(dev->of_node,
							   "gce-events",
							   i, &vcu->gce_codec_eid[gce_event_id])) {
				dev_err(dev, "can't parse gce-events property");
				return -ENODEV;
			}
			dev_info(dev, "%s %d gce event id: %d, name: %s gce_codec_eid:%d",
				__func__, __LINE__, gce_event_id, gce_event_name, vcu->gce_codec_eid[gce_event_id]);
		} else {
			dev_info(dev, "cannot find gce event id by name: %s, need check dts settings",
				gce_event_name);
			return -EINVAL;
		}
		i++;
	}

	for (i = 0; i < (int)VCU_CODEC_MAX; i++) {
		vcu->gce_cmds[i] = devm_kzalloc(dev,
			sizeof(struct gce_cmds), GFP_KERNEL);
		if (vcu->gce_cmds[i] == NULL)
			goto vcu_mutex_destroy;
	}

	for (i = 0; i < (int)VCU_CODEC_MAX; i++)
		mutex_init(&vcu->gce_cmds_mutex[i]);
#endif
	sema_init(&vcu->vpud_killed, 1);

	for (i = 0; i < (int)VCU_CODEC_MAX; i++) {
		vcu->curr_ctx[i] = NULL;
		vcu->curr_src_vb[i] = NULL;
		vcu->curr_dst_vb[i] = NULL;
	}
	vcu->is_entering_suspend = 0;
	vcu->pm_notifier.notifier_call = mtk_vcu_suspend_notifier;
	register_pm_notifier(&vcu->pm_notifier);

	ret = vcu_alloc_d_ext_mem(vcu, VCU_SHMEM_SIZE);
	if (ret != 0) {
		dev_dbg(dev, "[VCU] allocate SHMEM failed\n");
		goto err_after_gce;
	}

	//register_trace_signal_generate(probe_death_signal, NULL);
	spin_lock_init(&vcu->vpud_sig_lock);
	vcu->vpud_is_going_down = 0;
	vcu->enable_vcu_dbg_log = 0;

	vcu_func_mt8395.vcu_get_plat_device = vcu_get_plat_device_mt8395;
	vcu_func_mt8395.vcu_load_firmware = vcu_load_firmware_mt8395;
	vcu_func_mt8395.vcu_compare_version = vcu_compare_version_mt8395;
	vcu_func_mt8395.vcu_get_task = vcu_get_task_mt8395;
	vcu_func_mt8395.vcu_put_task = vcu_put_task_mt8395;
#if ENABLE_GCE
	vcu_func_mt8395.vcu_set_v4l2_callback = vcu_set_v4l2_callback_mt8395;
#endif
	vcu_func_mt8395.vcu_get_ctx_ipi_binding_lock = vcu_get_ctx_ipi_binding_lock_mt8395;
	vcu_func_mt8395.vcu_clear_codec_ctx = vcu_clear_codec_ctx_mt8395;
	vcu_func_mt8395.vcu_mapping_dm_addr = vcu_mapping_dm_addr_mt8395;
	vcu_func_mt8395.vcu_ipi_register = vcu_ipi_register_mt8395;
	vcu_func_mt8395.vcu_ipi_send = vcu_ipi_send_mt8395;
	vcu_func_mt8395.vcu_set_codec_ctx = vcu_set_codec_ctx_mt8395;

	/* init character device */
	ret = alloc_chrdev_region(&vcu_mtkdev[vcuid]->vcu_devno, 0, 1,
				  vcu_mtkdev[vcuid]->vcuname);
	if (ret < 0) {
		dev_info(dev,
			"[VCU] alloc_chrdev_region failed (%d)\n", ret);
		goto err_alloc_chrdev;
	}

	vcu_mtkdev[vcuid]->vcu_cdev = cdev_alloc();
	vcu_mtkdev[vcuid]->vcu_cdev->owner = THIS_MODULE;
	vcu_mtkdev[vcuid]->vcu_cdev->ops = &vcu_fops;

	ret = cdev_add(vcu_mtkdev[vcuid]->vcu_cdev,
		vcu_mtkdev[vcuid]->vcu_devno, 1);
	if (ret < 0) {
		dev_info(dev, "[VCU] class create fail (ret=%d)", ret);
		goto err_cdev_add;
	}

	vcu_mtkdev[vcuid]->vcu_class = class_create(vcu_mtkdev[vcuid]->vcuname);
	if (IS_ERR_OR_NULL(vcu_mtkdev[vcuid]->vcu_class) == true) {
		ret = (int)PTR_ERR(vcu_mtkdev[vcuid]->vcu_class);
		dev_info(dev, "[VCU] class create fail (ret=%d)", ret);
		goto err_class_create;
	}

	vcu_mtkdev[vcuid]->vcu_device =
		device_create(vcu_mtkdev[vcuid]->vcu_class,
			      NULL,
			      vcu_mtkdev[vcuid]->vcu_devno,
			      NULL, vcu_mtkdev[vcuid]->vcuname);
	if (IS_ERR_OR_NULL(vcu_mtkdev[vcuid]->vcu_device) == true) {
		ret = (int)PTR_ERR(vcu_mtkdev[vcuid]->vcu_device);
		dev_info(dev, "[VCU] device_create fail (ret=%d)", ret);
		goto err_device_create;
	}

	dev_info(dev, "[VCU] initialization completed\n");
	return 0;

err_device_create:
	class_destroy(vcu_mtkdev[vcuid]->vcu_class);
err_class_create:
	cdev_del(vcu_mtkdev[vcuid]->vcu_cdev);
err_cdev_add:
	unregister_chrdev_region(vcu_mtkdev[vcuid]->vcu_devno, 1);
err_alloc_chrdev:
	vcu_free_d_ext_mem(vcu);
err_after_gce:
	for (i = 0; i < (int)VCU_CODEC_MAX; i++)
		mutex_destroy(&vcu->gce_cmds_mutex[i]);
vcu_mutex_destroy:
	mutex_destroy(&vcu->vcu_dev_mutex);
	vcu_ipi_deinit(vcu);
err_ipi_init:
	vcu_mtkdev[vcuid] = NULL;
	devm_kfree(dev, vcu);

	return ret;
}

static const struct of_device_id mtk_vcu_match[] = {
	{.compatible = "mediatek,mt8395-vcu",},
	{},
};

MODULE_DEVICE_TABLE(of, mtk_vcu_match);

static void mtk_vcu_remove(struct platform_device *pdev)
{
	struct mtk_vcu *vcu = platform_get_drvdata(pdev);
	int i = 0;

	vcu_free_d_ext_mem(vcu);
	for (i = 0; i < vcu->gce_th_num[VCU_VENC]; i++) {
#if ENABLE_GCE
		flush_workqueue(vcu->gce_callback_thread_wq[i]);
		destroy_workqueue(vcu->gce_callback_thread_wq[i]);
#endif
	}

	if (vcu->is_open == true) {
		vcu->is_open = false;
	}
	devm_kfree(&pdev->dev, vcu);

	device_destroy(vcu->vcu_class, vcu->vcu_devno);
	class_destroy(vcu->vcu_class);
	cdev_del(vcu->vcu_cdev);
	unregister_chrdev_region(vcu->vcu_devno, 1);

	return;
}

static const struct dev_pm_ops mtk_vcu_pm_ops = {
	.suspend = mtk_vcu_suspend,
	.resume = mtk_vcu_resume,
};

static struct platform_driver mtk_vcu_driver = {
	.probe  = mtk_vcu_probe,
	.remove = mtk_vcu_remove,
	.driver = {
		/*
		 * Renamed from "mtk_vcu" so this OOT VCU provider can coexist
		 * with the MT8365/Genio-350 VCU provider (drivers/media/platform/
		 * mtk-vpu/mtk_vcu.c), which registers the same "mtk_vcu" platform-
		 * driver name. Binding is by of_match (mediatek,mt8395-vcu),
		 * NOT by name, so the Genio 510 vcu DT node still binds this
		 * driver. The /dev/vcu char device (VCU_DEVNAME "vcu") that vpud
		 * opens is unchanged.
		 */
		.name   = "mtk_vcu_8395",
		.owner  = THIS_MODULE,
		.pm = &mtk_vcu_pm_ops,
		.of_match_table = mtk_vcu_match,
	},
};


static int mtk_vcu_io_probe(struct platform_device *pdev)
{
	struct mtk_vcu *vcu;
	struct device *dev;
	int ret = 0;
	unsigned int vcuid;

	dev_dbg(&pdev->dev, "[VCU][IO] initialization\n");

	dev = &pdev->dev;
	vcu = devm_kzalloc(dev, sizeof(*vcu), GFP_KERNEL);
	if (vcu == NULL)
		return -ENOMEM;

	vcu->dev_io_enc = &pdev->dev;

	ret = of_property_read_u32(dev->of_node, "mediatek,vcuid", &vcuid);
	if (ret != 0) {
		dev_info(dev, "[VCU][IO] failed to find mediatek,vcuid\n");
		return ret;
	}
	vcu_mtkdev[vcuid] = vcu;

#if IS_ENABLED(CONFIG_MTK_IOMMU)
	vcu_mtkdev[vcuid]->io_domain_enc = iommu_get_domain_for_dev(dev);
	if (vcu_mtkdev[vcuid]->io_domain_enc == NULL) {
		dev_info(dev,
			"[VCU][IO] vcuid: %d get io_domain_enc fail !!\n", vcuid);
		return -EPROBE_DEFER;
	}
	dev_dbg(dev, "vcu io_domain: %p,vcuid:%d\n",
		vcu_mtkdev[vcuid]->io_domain_enc, vcuid);
	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(34));
	if (ret) {
		ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(34));
		if (ret) {
			dev_info(dev, "64-bit DMA enable failed\n");
			return ret;
		}
	}
#endif

	dev_dbg(dev, "[VCU][IO] initialization completed\n");
	return 0;
}

static const struct of_device_id mtk_vcu_io_match[] = {
	{.compatible = "mediatek,vcu-io-venc",},
	{},
};

MODULE_DEVICE_TABLE(of, mtk_vcu_io_match);

static void mtk_vcu_io_remove(struct platform_device *pdev)
{
	return;
}

static struct platform_driver mtk_vcu_io_driver = {
	.probe  = mtk_vcu_io_probe,
	.remove = mtk_vcu_io_remove,
	.driver = {
		.name   = "mtk_vcu_io",
		.owner  = THIS_MODULE,
		.pm = &mtk_vcu_pm_ops,
		.of_match_table = mtk_vcu_io_match,
	},
};


/*
 * driver initialization entry point
 */
static int __init mtk_vcu_driver_init(void)
{
	int ret;

	ret = platform_driver_register(&mtk_vcu_driver);
	if (ret) {
		pr_info("[VCU] mtk_vcu_driver probe fail\n");
		return ret;
	}
	ret = platform_driver_register(&mtk_vcu_io_driver);
	if (ret) {
		pr_info("[VCU] mtk_vcu_io_driver probe fail\n");
		return ret;
	}
	return ret;
}

module_init(mtk_vcu_driver_init);

/*
 * driver exit point
 */
static void __exit mtk_vcu_driver_exit(void)
{
	platform_driver_unregister(&mtk_vcu_driver);
	platform_driver_unregister(&mtk_vcu_io_driver);
}

module_exit(mtk_vcu_driver_exit);

MODULE_LICENSE("GPL v2");
MODULE_IMPORT_NS("DMA_BUF");
MODULE_DESCRIPTION("Mediatek Video Communication And Controller Unit driver");
