/*
 * linux/arch/arm/mach-oxnas/gmac-offload.h
 *
 * Copyright (C) 2006 Oxford Semiconductor Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifdef CONFIG_LEON_COPRO

#if !defined(__GMAC_OFFLOAD_H__)
#define __GMAC_OFFLOAD_H__

#include <linux/kernel.h>
#include <linux/list.h>

#define GMAC_CMD_QUE_OWN_BIT 31 // 0-Owned by ARM, 1-Owned by CoPro
#define GMAC_CMD_QUE_ACK_BIT 30
#define GMAC_CMD_QUE_SKP_BIT 29

typedef struct gmac_cmd_que_ent {
    u32 opcode_;
    u32 operand_;
} __attribute ((aligned(4),packed)) gmac_cmd_que_ent_t;

#define GMAC_CMD_STOP               0
#define GMAC_CMD_START              1
#define GMAC_CMD_INT_EN_SET         2
#define GMAC_CMD_INT_EN_CLR         3
#define GMAC_CMD_HEARTBEAT          4
#define GMAC_CMD_UPDATE_PARAMS      5
#define GMAC_CMD_CHANGE_GIG_MODE    6
#define GMAC_CMD_CHANGE_RX_ENABLE   7
#define GMAC_CMD_CLEAR_RX_INTS      8
#define GMAC_CMD_CHANGE_PAUSE_MODE  9

typedef void (*cmd_que_ack_callback)(volatile gmac_cmd_que_ent_t* entry);

typedef struct cmd_que_ack {
    struct list_head             list_;
    volatile gmac_cmd_que_ent_t *entry_;
    cmd_que_ack_callback         callback_;
} cmd_que_ack_t;

typedef struct cmd_que {
    gmac_cmd_que_ent_t          *head_;
    gmac_cmd_que_ent_t          *tail_;
    volatile gmac_cmd_que_ent_t *w_ptr_;
    struct list_head             ack_list_;
} cmd_que_t;

extern void cmd_que_init(
    cmd_que_t          *queue,
    gmac_cmd_que_ent_t *start,
    int                 num_entries);

extern int cmd_que_dequeue_ack(cmd_que_t *queue);

extern int cmd_que_queue_cmd(
    cmd_que_t            *queue,
    u32                   opcode,
    u32                   operand,
    cmd_que_ack_callback  callback);    // non-zero if ack. required

#define COPRO_SEM_INT_CMD 0
#define COPRO_SEM_INT_TX  1

typedef struct gmac_fwd_intrs {
    u32 status_;
} __attribute ((aligned(4),packed)) gmac_fwd_intrs_t;

#define TX_JOB_FLAGS_OWN_BIT              0
#define TX_JOB_FLAGS_COPRO_RESERVED_1_BIT 1
#define TX_JOB_FLAGS_ACCELERATE_BIT       2

#define TX_JOB_STATS_BYTES_BIT 0
#define TX_JOB_STATS_BYTES_NUM_BITS 16
#define TX_JOB_STATS_BYTES_MASK (((1UL << TX_JOB_STATS_BYTES_NUM_BITS) - 1) << TX_JOB_STATS_BYTES_BIT)
#define TX_JOB_STATS_PACKETS_BIT 16
#define TX_JOB_STATS_PACKETS_NUM_BITS 6
#define TX_JOB_STATS_PACKETS_MASK (((1UL << TX_JOB_STATS_PACKETS_NUM_BITS) - 1) << TX_JOB_STATS_PACKETS_BIT)
#define TX_JOB_STATS_ABORT_BIT 22
#define TX_JOB_STATS_ABORT_NUM_BITS 3
#define TX_JOB_STATS_ABORT_MASK (((1UL << TX_JOB_STATS_ABORT_NUM_BITS) - 1) << TX_JOB_STATS_ABORT_BIT)
#define TX_JOB_STATS_CARRIER_BIT 25
#define TX_JOB_STATS_CARRIER_NUM_BITS 3
#define TX_JOB_STATS_CARRIER_MASK (((1UL << TX_JOB_STATS_CARRIER_NUM_BITS) - 1) << TX_JOB_STATS_CARRIER_BIT)
#define TX_JOB_STATS_COLLISION_BIT 28
#define TX_JOB_STATS_COLLISION_NUM_BITS 4
#define TX_JOB_STATS_COLLISION_MASK (((1UL << TX_JOB_STATS_COLLISION_NUM_BITS) - 1) << TX_JOB_STATS_COLLISION_BIT)

/* Make even number else gmac_tx_que_ent_t struct below alignment will be wrong */
#define COPRO_NUM_TX_FRAGS_DIRECT 18

typedef struct gmac_tx_que_ent {
    u32 skb_;
    u32 len_;
    u32 data_len_;
    u32 ethhdr_;
    u32 iphdr_;
    u16 iphdr_csum_;
    u16 tso_segs_;
    u16 tso_size_;
    u16 flags_;
    u32 frag_ptr_[COPRO_NUM_TX_FRAGS_DIRECT];
    u16 frag_len_[COPRO_NUM_TX_FRAGS_DIRECT];
    u32 statistics_;
} __attribute ((aligned(4),packed)) gmac_tx_que_ent_t;

typedef struct tx_que {
    gmac_tx_que_ent_t          *head_;
    gmac_tx_que_ent_t          *tail_;
    volatile gmac_tx_que_ent_t *w_ptr_;
    volatile gmac_tx_que_ent_t *r_ptr_;
    int                         full_;
} tx_que_t;

extern void tx_que_init(
    tx_que_t          *queue,
    gmac_tx_que_ent_t *start,
    int                num_entries);

static inline int tx_que_not_empty(tx_que_t *queue)
{
    return (queue->r_ptr_ != queue->w_ptr_) || queue->full_;
}

static inline int tx_que_is_full(tx_que_t *queue)
{
    return queue->full_;
}

static inline void tx_que_inc_r_ptr(tx_que_t *queue)
{
    if (++queue->r_ptr_ == queue->tail_) {
        queue->r_ptr_ = queue->head_;
    }
    if (queue->full_) {
        queue->full_ = 0;
    }
}

extern volatile gmac_tx_que_ent_t* tx_que_get_finished_job(struct net_device *dev);

extern volatile gmac_tx_que_ent_t* tx_que_get_idle_job(struct net_device *dev);

extern void tx_que_new_job(
    struct net_device *dev,
    volatile gmac_tx_que_ent_t *entry);

#endif        //  #if !defined(__GMAC_OFFLOAD_H__)
#endif // CONFIG_LEON_COPRO
