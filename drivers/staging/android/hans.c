/***********************************************************
** Copyright (C), 2008-2019, OPLUS Mobile Comm Corp., Ltd.
** VENDOR_EDIT
** File: hans.c
** Description: Add for hans freeze manager
**
** Version: 1.0
** Date : 2019/09/23
**
** ------------------ Revision History:------------------------
** <author>      <data>      <version >       <desc>
** Kun Zhou    2019/09/23      1.0       OPLUS_ARCH_EXTENDS
** Kun Zhou    2020/05/27      1.1       OPLUS_FEATURE_HANS_FREEZE
****************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netlink.h>
#include <linux/skbuff.h>
#include <net/sock.h>
#include <linux/hans.h>

#define NETLINK_PORT_HANS        (0x15356)

static struct sock *sock_handle = NULL;
static atomic_t hans_deamon_port;

/*
 * netlink report function to tell HANS native deamon unfreeze process info
 * if the parameters is empty, fill it with (pid/uid with -1)
 */
int hans_report(enum message_type type, int caller_pid, int caller_uid, int target_pid, int target_uid, const char *rpc_name, int code)
{
	int len = 0;
	int ret = 0;
	struct hans_message *data = NULL;
	struct sk_buff *skb = NULL;
	struct nlmsghdr *nlh = NULL;

	if (atomic_read(&hans_deamon_port) == -1) {
		pr_err("%s: hans_deamon_port invalid!\n", __func__);
                return HANS_ERROR;
	}

	if (sock_handle == NULL) {
		pr_err("%s: sock_handle invalid!\n", __func__);
                return HANS_ERROR;
	}

	if (type >= TYPE_MAX) {
		pr_err("%s: type = %d invalid!\n", __func__, type);
		return HANS_ERROR;
	}

	len = sizeof(struct hans_message);
	skb = nlmsg_new(len, GFP_ATOMIC);
	if (skb == NULL) {
		pr_err("%s: type =%d, nlmsg_new failed!\n", __func__, type);
		return HANS_ERROR;
	}

	nlh = nlmsg_put(skb, 0, 0, 0, len, 0);
	if (nlh == NULL) {
		pr_err("%s: type =%d, nlmsg_put failed!\n", __func__, type);
		kfree_skb(skb);
		return HANS_ERROR;
	}

	data = nlmsg_data(nlh);
	if(data == NULL) {
		pr_err("%s: type =%d, nlmsg_data failed!\n", __func__, type);
		return HANS_ERROR;
	}
	data->type = type;
	data->port = NETLINK_PORT_HANS;
	data->caller_pid = caller_pid;
	data->caller_uid = caller_uid;
	data->target_pid = target_pid;
	data->target_uid = target_uid;
	data->pkg_cmd = -1; /*invalid package cmd*/
	data->code = code;
	strlcpy(data->rpc_name, rpc_name, INTERFACETOKEN_BUFF_SIZE);
	nlmsg_end(skb, nlh);

	if ((ret = nlmsg_unicast(sock_handle, skb, (u32)atomic_read(&hans_deamon_port))) < 0) {
		pr_err("%s: nlmsg_unicast failed! err = %d\n", __func__ , ret);
		return HANS_ERROR;
	}

	return HANS_NOERROR;
}

/*HANS kernel module handle the message from HANS native deamon*/
static void hans_handler(struct sk_buff *skb)
{
	struct hans_message *data = NULL;
	struct nlmsghdr *nlh = NULL;
	unsigned int len  = 0;
	int uid = -1;

	if (!skb) {
		pr_err("%s: recv skb NULL!\n", __func__);
		return;
	}

	uid = (*NETLINK_CREDS(skb)).uid.val;
	if (uid != 1000) {
		pr_err("%s: uid: %d, permission denied\n", __func__, uid);
		return;
	}

	if (skb->len >= NLMSG_SPACE(0)) {
		nlh = nlmsg_hdr(skb);
		len = NLMSG_PAYLOAD(nlh, 0);
		data = (struct hans_message *)NLMSG_DATA(nlh);

		if (len < sizeof(struct hans_message)) {
			pr_err("%s: hans_message len check faied! len = %d  min_expected_len = %lu!\n", __func__, len, sizeof(struct hans_message));
			return;
		}

		if (data->port < 0) {
			pr_err("%s: portid = %d invalid!\n", __func__, data->port);
			return;
		}
		if (data->type >= TYPE_MAX) {
			pr_err("%s: type = %d invalid!\n", __func__, data->type);
			return;
		}
		if (atomic_read(&hans_deamon_port) == -1 && data->type != LOOP_BACK) {
			pr_err("%s: handshake not setup, type = %d!\n", __func__, data->type);
                        return;
		}

        switch (data->type) {
        case LOOP_BACK:  /*Loop back message, only for native deamon and kernel handshake*/
				atomic_set(&hans_deamon_port, data->port);
				hans_report(LOOP_BACK, -1, -1, -1, -1, "loop back", CPUCTL_VERSION);
				printk(KERN_ERR "%s: --> LOOP_BACK, port = %d\n", __func__, data->port);
				break;
        case PKG:
				printk(KERN_ERR "%s: --> PKG, uid = %d, pkg_cmd = %d\n", __func__, data->target_uid, data->pkg_cmd);
				hans_network_cmd_parse(data->target_uid, data->pkg_cmd);
				break;
        case FROZEN_TRANS:
        case CPUCTL_TRANS:
				printk(KERN_ERR "%s: --> FROZEN_TRANS, uid = %d\n", __func__, data->target_uid);
				hans_check_frozen_transcation(data->target_uid, data->type);
				break;

		default:
			pr_err("%s: hans_messag type invalid %d\n", __func__, data->type);
			break;
		}
	}
}

static int __init hans_core_init(void)
{
	struct netlink_kernel_cfg cfg = {
		.input = hans_handler,
	};

	atomic_set(&hans_deamon_port, -1);

        sock_handle = netlink_kernel_create(&init_net, NETLINK_OPLUS_HANS, &cfg);
	if (sock_handle == NULL) {
		pr_err("%s: create netlink socket failed!\n", __func__);
		return HANS_ERROR;
	}

	if (hans_netfilter_init() == HANS_ERROR) {
		pr_err("%s: netfilter init failed!\n", __func__);
		netlink_kernel_release(sock_handle);  /*release socket*/
                return HANS_ERROR;
	}

	printk(KERN_INFO "%s: -\n", __func__);
	return HANS_NOERROR;
}

static void __exit hans_core_exit(void)
{
	if (sock_handle)
		netlink_kernel_release(sock_handle);

	hans_netfilter_deinit();
	printk(KERN_INFO "%s: -\n", __func__);
}

module_init(hans_core_init);
module_exit(hans_core_exit);

MODULE_LICENSE("GPL");
