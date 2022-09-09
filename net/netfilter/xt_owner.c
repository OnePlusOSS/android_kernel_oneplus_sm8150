/*
 * Kernel module to match various things tied to sockets associated with
 * locally generated outgoing packets.
 *
 * (C) 2000 Marc Boucher <marc@mbsi.ca>
 *
 * Copyright © CC Computer Consultants GmbH, 2007 - 2008
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/file.h>
#include <linux/cred.h>

#include <net/sock.h>
#include <net/inet_sock.h>
#include <linux/netfilter/x_tables.h>
#include <linux/netfilter/xt_owner.h>


#ifndef OPLUS_FEATURE_XTOWNER_INPUT
#define OPLUS_FEATURE_XTOWNER_INPUT
#endif

#ifdef OPLUS_FEATURE_XTOWNER_INPUT
//add for BUG 2212301
#include <net/netfilter/ipv4/nf_defrag_ipv4.h>
#if IS_ENABLED(CONFIG_IP6_NF_IPTABLES)
#include <linux/netfilter_ipv6/ip6_tables.h>
#include <net/inet6_hashtables.h>
#include <net/netfilter/ipv6/nf_defrag_ipv6.h>
#endif
#include <net/netfilter/nf_socket.h>
#include <linux/netfilter/xt_socket.h>
#define XT_SOCKET_SUPPORTED_HOOKS \
    ((1 << NF_INET_PRE_ROUTING) | (1 << NF_INET_LOCAL_IN))
#endif  /*OPLUS_FEATURE_XTOWNER_INPUT*/

static int owner_check(const struct xt_mtchk_param *par)
{
	struct xt_owner_match_info *info = par->matchinfo;
	struct net *net = par->net;

	/* Only allow the common case where the userns of the writer
	 * matches the userns of the network namespace.
	 */
	if ((info->match & (XT_OWNER_UID|XT_OWNER_GID)) &&
	    (current_user_ns() != net->user_ns))
		return -EINVAL;

	/* Ensure the uids are valid */
	if (info->match & XT_OWNER_UID) {
		kuid_t uid_min = make_kuid(net->user_ns, info->uid_min);
		kuid_t uid_max = make_kuid(net->user_ns, info->uid_max);

		if (!uid_valid(uid_min) || !uid_valid(uid_max) ||
		    (info->uid_max < info->uid_min) ||
		    uid_lt(uid_max, uid_min)) {
			return -EINVAL;
		}
	}

	/* Ensure the gids are valid */
	if (info->match & XT_OWNER_GID) {
		kgid_t gid_min = make_kgid(net->user_ns, info->gid_min);
		kgid_t gid_max = make_kgid(net->user_ns, info->gid_max);

		if (!gid_valid(gid_min) || !gid_valid(gid_max) ||
		    (info->gid_max < info->gid_min) ||
		    gid_lt(gid_max, gid_min)) {
			return -EINVAL;
		}
	}

	return 0;
}

#ifdef OPLUS_FEATURE_XTOWNER_INPUT
//add for BUG 2212301
static struct sock *oem_qtaguid_find_sk(const struct sk_buff *skb,
				struct xt_action_param *par)
{
	const struct nf_hook_state *parst = par->state;
	struct sock *sk;
	unsigned int hook_mask = (1 << parst->hook);


	pr_debug("qtaguid[%d]: find_sk(skb=%pK) family=%d\n",
		parst->hook, skb, parst->pf);

	/*
	* Let's not abuse the the xt_socket_get*_sk(), or else it will
	* return garbage SKs.
	*/
	if (!(hook_mask & XT_SOCKET_SUPPORTED_HOOKS))
		return NULL;

	switch (parst->pf) {
		case NFPROTO_IPV6:
			sk = nf_sk_lookup_slow_v6(dev_net(skb->dev), skb, parst->in);
			break;
		case NFPROTO_IPV4:
			sk = nf_sk_lookup_slow_v4(dev_net(skb->dev), skb, parst->in);
			break;
		default:
			return NULL;
	}

	if (sk) {
		pr_debug("qtaguid[%d]: %p->sk_proto=%u->sk_state=%d\n", parst->hook, sk, sk->sk_protocol, sk->sk_state);
	}

	return sk;
}
#endif  /*OPLUS_FEATURE_XTOWNER_INPUT*/

static bool
owner_mt(const struct sk_buff *skb, struct xt_action_param *par)
{
	const struct xt_owner_match_info *info = par->matchinfo;
	const struct file *filp;
	struct sock *sk = skb_to_full_sk(skb);
	struct net *net = xt_net(par);

	#ifdef OPLUS_FEATURE_XTOWNER_INPUT
	//add for BUG 2212301
	/*
	* When in TCP_TIME_WAIT the sk is not a "struct sock" but
	* "struct inet_timewait_sock" which is missing fields
	* So we ignore it
	*/
	if (sk && sk->sk_state == TCP_TIME_WAIT){
		pr_debug("owner_mt 1 : sk: %p, sk->sk_state: %d \n", sk, sk->sk_state);
		sk = NULL;
	}
	if (sk == NULL) {
		/*
		* A missing sk->sk_socket happens when packets are in-flight
		* and the matching socket is already closed and gone.
		*/
		sk = oem_qtaguid_find_sk(skb, par);
		/*
		* TCP_NEW_SYN_RECV are not "struct sock" but "struct request_sock"
		* where we can get a pointer to a full socket to retrieve uid/gid.
		* When in TCP_TIME_WAIT, sk is a struct inet_timewait_sock
		* which is missing fields and does not contain any reference
		* to a full socket, so just ignore the socket
		*/
		if (sk && sk->sk_state == TCP_NEW_SYN_RECV) {
			pr_debug("owner_mt 2 : sk: %p, sk->sk_state: %d \n", sk, sk->sk_state);
			sock_gen_put(sk);
			sk = sk_to_full_sk(sk);
		} else if (sk && (!sk_fullsock(sk) || sk->sk_state == TCP_TIME_WAIT)) {
			pr_debug("owner_mt 3 : sk: %p, sk->sk_state: %d \n", sk, sk->sk_state);
			sock_gen_put(sk);
			sk = NULL;
		}
		//#ifdef OPLUS_FEATURE_XTOWNER_INPUT
		else if (sk && sk->sk_state == TCP_CLOSE) {
			sock_gen_put(sk);
		} else if (sk) {
			pr_debug("owner_mt: sk: %px, sk->sk_state: %d \n", sk, sk->sk_state);
			//WARN_ON(1);
			sock_gen_put(sk);
		}
		//#endif
	}

	if(sk) {
		pr_debug("owner_mt: sk: %p, sk->sk_state: %d, sk->sk_socket: %p\n", sk, sk->sk_state, sk->sk_socket);
	}
	#endif  /*OPLUS_FEATURE_XTOWNER_INPUT*/

	if (sk == NULL || sk->sk_socket == NULL)
		return (info->match ^ info->invert) == 0;
	else if (info->match & info->invert & XT_OWNER_SOCKET)
		/*
		 * Socket exists but user wanted ! --socket-exists.
		 * (Single ampersands intended.)
		 */
		return false;

	filp = sk->sk_socket->file;
	if (filp == NULL)
		return ((info->match ^ info->invert) &
				(XT_OWNER_UID | XT_OWNER_GID)) == 0;

	if (info->match & XT_OWNER_UID) {
		kuid_t uid_min = make_kuid(net->user_ns, info->uid_min);
		kuid_t uid_max = make_kuid(net->user_ns, info->uid_max);
		if ((uid_gte(filp->f_cred->fsuid, uid_min) &&
			uid_lte(filp->f_cred->fsuid, uid_max)) ^
			!(info->invert & XT_OWNER_UID))
			return false;
	}

	if (info->match & XT_OWNER_GID) {
		kgid_t gid_min = make_kgid(net->user_ns, info->gid_min);
		kgid_t gid_max = make_kgid(net->user_ns, info->gid_max);
		if ((gid_gte(filp->f_cred->fsgid, gid_min) &&
			gid_lte(filp->f_cred->fsgid, gid_max)) ^
		 	!(info->invert & XT_OWNER_GID))
			return false;
	}

	return true;
}

static struct xt_match owner_mt_reg __read_mostly = {
	.name       = "owner",
	.revision   = 1,
	.family     = NFPROTO_UNSPEC,
	.checkentry = owner_check,
	.match      = owner_mt,
	.matchsize  = sizeof(struct xt_owner_match_info),
#ifndef OPLUS_FEATURE_XTOWNER_INPUT
	.hooks      = (1 << NF_INET_LOCAL_OUT) |
	              (1 << NF_INET_POST_ROUTING),
#else
	.hooks      = (1 << NF_INET_LOCAL_OUT) |
	              (1 << NF_INET_POST_ROUTING) |
	              (1 << NF_INET_LOCAL_IN),
#endif
	.me         = THIS_MODULE,
};

static int __init owner_mt_init(void)
{
	return xt_register_match(&owner_mt_reg);
}

static void __exit owner_mt_exit(void)
{
	xt_unregister_match(&owner_mt_reg);
}

module_init(owner_mt_init);
module_exit(owner_mt_exit);
MODULE_AUTHOR("Jan Engelhardt <jengelh@medozas.de>");
MODULE_DESCRIPTION("Xtables: socket owner matching");
MODULE_LICENSE("GPL");
MODULE_ALIAS("ipt_owner");
MODULE_ALIAS("ip6t_owner");
