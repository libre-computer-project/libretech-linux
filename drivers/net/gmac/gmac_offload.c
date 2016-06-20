/*
 * linux/arch/arm/mach-oxnas/gmac-offload.c
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

#include <linux/slab.h>
#include "gmac.h"

void cmd_que_init(
    cmd_que_t          *queue,
    gmac_cmd_que_ent_t *start,
    int                 num_entries)
{
    // Initialise queue management metadata
    INIT_LIST_HEAD(&queue->ack_list_);
    queue->head_ = start;
    queue->tail_ = start + num_entries;
    queue->w_ptr_ = queue->head_;

    // Zeroise all entries in the queue
    memset(start, 0, num_entries * sizeof(gmac_cmd_que_ent_t));
}

int cmd_que_dequeue_ack(cmd_que_t *queue)
{
    struct list_head *list_entry;
    cmd_que_ack_t    *ack;

    if (list_empty(&queue->ack_list_)) {
        return -1;
    }

    // Remove the first entry on the acknowledgement list
    list_entry = queue->ack_list_.next;
    BUG_ON(!list_entry);

    // Get pointer to ack entry from it's list_head member        
    ack = list_entry(list_entry, cmd_que_ack_t, list_);
    BUG_ON(!ack);
    BUG_ON(!ack->entry_);
    BUG_ON(!ack->callback_);

    // Has the CoPro acknowledged the command yet?
    if (!(ack->entry_->opcode_ & (1UL << GMAC_CMD_QUE_SKP_BIT))) {
        // No, so no further acknowledgements can be pending as CoPro executes
        // commands/acks in order
        return -1;
    }

    // Going to process the acknowledgement, so remove it from the pending list
    list_del(list_entry);

//printk("ak=0x%08x:0x%08x\n", ack->entry_->opcode_, ack->entry_->operand_);
    // Invoke the acknowledgement handler routine
    ack->callback_(ack->entry_);

    // Reset ACK flag in command queue entry
    ack->entry_->opcode_ &= ~(1UL << GMAC_CMD_QUE_ACK_BIT);

    kfree(ack);
    return 0;
}

#define OPCODE_FLAGS_MASK ((1UL << (GMAC_CMD_QUE_OWN_BIT)) |\
                           (1UL << (GMAC_CMD_QUE_ACK_BIT)) |\
                           (1UL << (GMAC_CMD_QUE_SKP_BIT)))

int cmd_que_queue_cmd(
    cmd_que_t            *queue,
    u32                   opcode,
    u32                   operand,
    cmd_que_ack_callback  callback)
{
    int result = -1;

    do {
        volatile gmac_cmd_que_ent_t* entry = queue->w_ptr_;
        u32 old_opcode = entry->opcode_;

        if (old_opcode & (1UL << GMAC_CMD_QUE_OWN_BIT)) {
            // Queue is full as we've run into an entry still owned by the CoPro
            break;
        }

        if (!(old_opcode & (1UL << GMAC_CMD_QUE_ACK_BIT))) {
            // We've found an entry we own that isn't waiting for the contained
            // ack to be processed, so we can use it for the new command
            opcode &= ~(OPCODE_FLAGS_MASK);
            opcode |= (1UL << GMAC_CMD_QUE_OWN_BIT);

            if (callback) {
                // Register ack. handler before releasing entry to CoPro
                cmd_que_ack_t *ack = kmalloc(sizeof(cmd_que_ack_t), GFP_ATOMIC);
                BUG_ON(!ack);

                ack->entry_ = queue->w_ptr_;
                ack->callback_ = callback;
                INIT_LIST_HEAD(&ack->list_);
                list_add_tail(&ack->list_, &queue->ack_list_);

                // Mark the entry as requiring an ack.
                opcode |= (1UL << GMAC_CMD_QUE_ACK_BIT);
            }

            // Copy the command into the queue entry and pass ownership to the
            // CoPro, being sure to set the OWN flag last
//printk("op=0x%08x:0x%08x\n", opcode, operand);
            queue->w_ptr_->operand_ = operand;
            wmb();
            queue->w_ptr_->opcode_  = opcode;
            // Ensure the OWN flag gets to memory before any following interrupt
            // to the CoPro is issued
            wmb();

            result = 0;
        }

        // Make the write pointer point to the next potentially available entry
        if (++queue->w_ptr_ == queue->tail_) {
            queue->w_ptr_ = queue->head_;
        }
    } while (result);

    return result;
}

void tx_que_init(
    tx_que_t          *queue,
    gmac_tx_que_ent_t *start,
    int                num_entries)
{
    // Initialise queue management metadata
    queue->head_ = start;
    queue->tail_ = start + num_entries;
    queue->w_ptr_ = queue->head_;
    queue->r_ptr_ = queue->head_;
    queue->full_ = 0;

    // Zeroise all entries in the queue
    memset(start, 0, num_entries * sizeof(gmac_tx_que_ent_t));
}

static inline void tx_que_inc_w_ptr(tx_que_t *queue)
{
    if (++queue->w_ptr_ == queue->tail_) {
        queue->w_ptr_ = queue->head_;
    }
    queue->full_ = (queue->w_ptr_ == queue->r_ptr_);
}

volatile gmac_tx_que_ent_t* tx_que_get_finished_job(struct net_device *dev)
{
    gmac_priv_t* priv = (gmac_priv_t*)netdev_priv(dev);
    tx_que_t *queue = &priv->tx_queue_;
    volatile gmac_tx_que_ent_t *entry = 0;

    if (tx_que_not_empty(queue)) {
        entry = queue->r_ptr_;
        if (entry->flags_ & (1UL << TX_JOB_FLAGS_OWN_BIT)) {
            entry = (volatile gmac_tx_que_ent_t*)0;
        } else {
            tx_que_inc_r_ptr(queue);
        }
    }

    return entry;
}

/**
 * A call to tx_que_get_idle_job() must be followed by a call to tx_que_new_job()
 * before any subsequent call to tx_que_get_idle_job()
 */
volatile gmac_tx_que_ent_t* tx_que_get_idle_job(struct net_device *dev)
{
    gmac_priv_t* priv = (gmac_priv_t*)netdev_priv(dev);
    tx_que_t *queue = &priv->tx_queue_;
    volatile gmac_tx_que_ent_t *entry = 0;

    // Must not reuse completed Tx packets returned by CoPro until the queue
    // reader has had a chance to process them
    if (!tx_que_is_full(queue)) {
        entry = queue->w_ptr_;
    }

    return entry;
}

/**
 * A call to tx_que_get_idle_job() must be followed by a call to tx_que_new_job()
 * before any subsequent call to tx_que_get_idle_job()
 */
void tx_que_new_job(
    struct net_device *dev,
    volatile gmac_tx_que_ent_t *entry)
{
    gmac_priv_t* priv = (gmac_priv_t*)netdev_priv(dev);

    // Make sure any modifications to Tx job structures make it to memory before
    // setting the OWN flag to pass ownership to the CoPro
    wmb();

    entry->flags_ |= (1UL << TX_JOB_FLAGS_OWN_BIT);

    tx_que_inc_w_ptr(&priv->tx_queue_);
}
#endif // #ifdef CONFIG_LEON_COPRO

