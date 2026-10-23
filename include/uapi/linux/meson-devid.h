/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/*
 * Amlogic EL3 device credential service -- user interface.
 *
 * /dev/devid-enroll   IDENTIFY, BURN; one opener at a time, because a second
 *                     Identify replaces the key a pending Burn needs
 * /dev/devid          ANSWER
 *
 * Results (the `result` fields) are the service's own codes: 0 success,
 * negative refusals as defined by the firmware's protocol header.
 */

#ifndef _UAPI_LINUX_MESON_DEVID_H
#define _UAPI_LINUX_MESON_DEVID_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define MESON_DEVID_BLOB_BYTES		216
#define MESON_DEVID_NONCE_MAX		64

struct meson_devid_ident {
	__u8 magic[4];			/* "DIDI" */
	__u32 board_id;
	__u8 uid_len;
	__u8 uid[16];
	__u8 zero[3];
	__u8 E[32];			/* X25519 public key */
} __attribute__((packed));

struct meson_devid_burn {
	__u8 blob[MESON_DEVID_BLOB_BYTES];
	__s32 result;
};

struct meson_devid_answer {
	__u32 nonce_len;		/* 16..64 */
	__u8 nonce[MESON_DEVID_NONCE_MAX];
	__s32 result;
	__u32 serial;
	__u8 mac[32];
};

#define MESON_DEVID_IOC_MAGIC		'D'
#define MESON_DEVID_IOC_IDENTIFY	_IOR(MESON_DEVID_IOC_MAGIC, 1, struct meson_devid_ident)
#define MESON_DEVID_IOC_BURN		_IOWR(MESON_DEVID_IOC_MAGIC, 2, struct meson_devid_burn)
#define MESON_DEVID_IOC_ANSWER		_IOWR(MESON_DEVID_IOC_MAGIC, 3, struct meson_devid_answer)

#endif /* _UAPI_LINUX_MESON_DEVID_H */
