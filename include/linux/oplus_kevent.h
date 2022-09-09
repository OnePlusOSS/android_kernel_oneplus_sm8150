/* SPDX-License-Identifier: GPL-2.0-only */
/**************************************************************
* Copyright (c)  2008- 2020  Oplus. All rights reserved.
* VENDOR_EDIT
* File       : oplus_kevent.h
* Description: For kevent action upload upload to user layer
* Version   : 1.0
* Date        : 2019-12-19
* Author    :
* TAG         :
****************************************************************/

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/skbuff.h>
#include <linux/types.h>
#include <linux/netlink.h>
#include <net/net_namespace.h>
#include <net/sock.h>
#include <linux/version.h>

#ifdef CONFIG_OPLUS_KEVENT_UPLOAD

#define NETLINK_OPLUS_KEVENT 34

struct kernel_packet_info
{
	int type;    /* 0:root,1:only string,other number represent other type */
	char log_tag[32];   /* logTag */
	char event_id[20];    /*eventID */
	size_t payload_length;    /* Length of packet data */
	unsigned char payload[0];   /* Optional packet data */
}__attribute__((packed));

int kevent_send_to_user(struct kernel_packet_info *userinfo);
void kernel_kevent_receive(struct sk_buff *__skbbr);
#endif /* CONFIG_OPLUS_KEVENT_UPLOAD */

#ifdef CONFIG_OPLUS_KEVENT_TEST
ssize_t demo_kevent_write(struct file *filp, const char __user *buffer, size_t count, loff_t *f_pos);
#endif /* CONFIG_OPLUS_KEVENT_TEST */

