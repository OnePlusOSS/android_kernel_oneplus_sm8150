/***********************************************************
** Copyright (C), 2008-2019, OPLUS Mobile Comm Corp., Ltd.
** VENDOR_EDIT
** File: hans_netfilter.c
** Description: Add for hans freeze manager
**
** Version: 1.0
** Date : 2019/09/23
**
** ------------------ Revision History:------------------------
** <author>      <data>      <version >       <desc>
** Kun Zhou    2019/09/23      1.0       OPLUS_ARCH_EXTENDS
** Kun Zhou    2019/09/23      1.1       OPLUS_FEATURE_HANS_FREEZE
****************************************************************/

#include <linux/hans.h>
#include <linux/netfilter.h>
#include <linux/netfilter_ipv4.h>
#include <linux/netfilter_ipv6.h>
#include <net/rtnetlink.h>
#include <net/sock.h>
#include <net/ip.h>

#define MAX_SLOT (100)
static uid_t monitored_uids[MAX_SLOT];
spinlock_t uids_lock;

static inline uid_t sock2uid(struct sock *sk)
{
	if(sk && sk->sk_socket)
		return SOCK_INODE(sk->sk_socket)->i_uid.val;
	else
		return 0;
}

/*Add netlink monitor uid. When the monitored UID has incoming network package, tell HANS native deamon*/
static void hans_add_monitored_uid(uid_t target_uid)
{
	int i = 0;
	int fisrt_empty_slot = MAX_SLOT;
	unsigned long flags;

	spin_lock_irqsave(&uids_lock, flags);
	for (i = 0; i < MAX_SLOT; i++) {
		if (monitored_uids[i] == target_uid) {  /*already in the monitored array*/
			spin_unlock_irqrestore(&uids_lock, flags);
			/*printk(KERN_WARNING "%s: uid = %d already in array\n", __func__, target_uid);*/
			return;
		} else if (monitored_uids[i] == 0 && fisrt_empty_slot == MAX_SLOT) {  /*first empty slot for monitoring uid*/
			fisrt_empty_slot = i;
		}
	}

	if (fisrt_empty_slot >= MAX_SLOT) {
		spin_unlock_irqrestore(&uids_lock, flags);
		pr_err("%s: monitored uid = %d add failed!\n", __func__, target_uid);
		return;
	}
	monitored_uids[fisrt_empty_slot] = target_uid;
	spin_unlock_irqrestore(&uids_lock, flags);
}

static void hans_remove_monitored_uid(uid_t target_uid)
{
	int i = 0;
	unsigned long flags;

	spin_lock_irqsave(&uids_lock, flags);
	for (i = 0; i < MAX_SLOT; i++) {
		if (monitored_uids[i] == target_uid) {
			monitored_uids[i] =  0;
			spin_unlock_irqrestore(&uids_lock, flags);
			return;
		}
	}
	spin_unlock_irqrestore(&uids_lock, flags);
	printk(KERN_WARNING "%s: uid = %d remove uid not found\n", __func__, target_uid);
}

static void hans_remove_all_monitored_uid(void)
{
	int i;
        unsigned long flags;

        spin_lock_irqsave(&uids_lock, flags);
	for (i = 0; i < MAX_SLOT; i++) {
		monitored_uids[i] = 0;
	}
	spin_unlock_irqrestore(&uids_lock, flags);
}

static bool hans_find_remove_monitored_uid(uid_t target_uid)
{
	bool found = false;
	int i = 0;
        unsigned long flags;

        spin_lock_irqsave(&uids_lock, flags);
	for (i = 0; i < MAX_SLOT; i++) {
		if (unlikely(monitored_uids[i] == target_uid)) {
			found = true;
			monitored_uids[i] = 0;
			break;
		}
	}
	spin_unlock_irqrestore(&uids_lock, flags);

	if (found)
		printk(KERN_WARNING "%s: uid = %d found and removed\n", __func__, target_uid);
	return found;
}

void hans_network_cmd_parse(uid_t uid, enum pkg_cmd cmd)
{
	switch (cmd) {
	case ADD_ONE_UID:
		hans_add_monitored_uid(uid);
		break;
	case DEL_ONE_UID:
		hans_remove_monitored_uid(uid);
		break;
	case DEL_ALL_UID:
		hans_remove_all_monitored_uid();
		break;
	default:
		pr_err("%s: pkg_cmd type invalid %d\n", __func__, cmd);
		break;
	}
}

/*Moniter the uid by netlink filter hook function.*/
static unsigned int hans_nf_ipv4v6_in(void *priv,
					struct sk_buff *skb,
					const struct nf_hook_state *state)
{
	struct sock *sk;
	uid_t uid;
	unsigned int thoff = 0;
	unsigned short frag_off = 0;
	bool found = false;

	if (ip_hdr(skb)->version == 4) {
		if (ip_hdr(skb)->protocol != IPPROTO_TCP)
			return NF_ACCEPT;
#if IS_ENABLED(CONFIG_IPV6)
	} else if (ip_hdr(skb)->version == 6) {
		if (ipv6_find_hdr(skb, &thoff, -1, &frag_off, NULL) != IPPROTO_TCP)
			return NF_ACCEPT;
#endif
	} else {
		return NF_ACCEPT;
	}

	sk = skb_to_full_sk(skb);
	if (sk == NULL) return NF_ACCEPT;
	if (!sk_fullsock(sk)) return NF_ACCEPT;

	uid = sock2uid(sk);
	if (uid < MIN_USERAPP_UID) return NF_ACCEPT;

	/*Find the monitored UID and clear it from the monitor array*/
	found = hans_find_remove_monitored_uid(uid);
	if (!found)
		return NF_ACCEPT;
	if (hans_report(PKG, -1, -1, -1, uid, "PKG", -1) != HANS_NOERROR)
		pr_err("%s: hans_report PKG failed!, uid = %d\n", __func__, uid);

	return NF_ACCEPT;
}

/*Only monitor input network packages*/
static struct nf_hook_ops hans_nf_ops[] = {
	{
		.hook     = hans_nf_ipv4v6_in,
		.pf       = NFPROTO_IPV4,
		.hooknum  = NF_INET_LOCAL_IN,
		.priority = NF_IP_PRI_SELINUX_LAST + 1,
	},
#if IS_ENABLED(CONFIG_IPV6)
	{
		.hook     = hans_nf_ipv4v6_in,
		.pf       = NFPROTO_IPV6,
		.hooknum  = NF_INET_LOCAL_IN,
		.priority = NF_IP6_PRI_SELINUX_LAST + 1,
	},
#endif
};

void hans_netfilter_deinit(void)
{
        struct net *net;

        rtnl_lock();
        for_each_net(net) {
                nf_unregister_net_hooks(net, hans_nf_ops, ARRAY_SIZE(hans_nf_ops));
        }
        rtnl_unlock();
}

int hans_netfilter_init(void)
{
	struct net *net = NULL;
	int err = 0;

	spin_lock_init(&uids_lock);
	hans_remove_all_monitored_uid();

	rtnl_lock();
	for_each_net(net) {
		err = nf_register_net_hooks(net, hans_nf_ops, ARRAY_SIZE(hans_nf_ops));
		if (err != 0) {
			pr_err("%s: register netfilter hooks failed!\n", __func__);
			break;
		}
	}
	rtnl_unlock();

	if (err != 0) {
		hans_netfilter_deinit();
		return HANS_ERROR;
	}
	return HANS_NOERROR;
}
