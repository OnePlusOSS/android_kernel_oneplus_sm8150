/************************************************************************************
** File: - oplus_nf_hooks.c
** VENDOR_EDIT
** Copyright (C), 2008-2020, OPLUS Mobile Comm Corp., Ltd
**
** Description:
**      1. Add for WeChat lucky money recognition
**
** Version: 1.0
** Date :   2020-03-20
** TAG  :   OPLUS_FEATURE_WIFI_LUCKYMONEY
**
** ---------------------Revision History: ---------------------
**  <author>                      <data>     <version >   <desc>
** ---------------------------------------------------------------
**
************************************************************************************/
#include <linux/types.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/netfilter.h>
#include <linux/netfilter_ipv6.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/icmp.h>
#include <linux/sysctl.h>
#include <net/route.h>
#include <net/ip.h>
#include <net/ipv6.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/version.h>
#include <net/tcp.h>
#include <linux/random.h>
#include <net/sock.h>
#include <net/dst.h>
#include <linux/file.h>
#include <net/tcp_states.h>
#include <linux/netlink.h>
#include <net/sch_generic.h>
#include <net/pkt_sched.h>
#include <net/netfilter/nf_queue.h>
#include <linux/netfilter/xt_state.h>
#include <linux/netfilter/x_tables.h>
#include <linux/netfilter/xt_owner.h>
#include <net/netfilter/nf_conntrack.h>
#include <net/netfilter/nf_conntrack_core.h>
#include <net/netfilter/ipv4/nf_conntrack_ipv4.h>


/*NLMSG_MIN_TYPE is 0x10,so we start at 0x11*/
enum{
	NF_HOOKS_ANDROID_PID    = 0x11,
	NF_HOOKS_WECHAT_PARAM   = 0x12,
	NF_HOOKS_LM_DETECTED    = 0x13,
};

#define MAX_FIXED_VALUE_LEN 20
#define MAX_WECHAT_PARAMS 10
#define UID_MASK   100000

struct wechat_pattern_info {
	u32 max_tot_len;
	u32 min_tot_len;
	int offset;
	u32 len;
	u8 fixed_value[MAX_FIXED_VALUE_LEN];
};

struct wechat_pattern_info wechat_infos[MAX_WECHAT_PARAMS];


static DEFINE_MUTEX(nf_hooks_netlink_mutex);
static struct ctl_table_header *oplus_nf_hooks_table_hrd;

static rwlock_t nf_hooks_lock;

#define nf_hooks_read_lock() 			read_lock_bh(&nf_hooks_lock);
#define nf_hooks_read_unlock() 			read_unlock_bh(&nf_hooks_lock);
#define nf_hooks_write_lock() 			write_lock_bh(&nf_hooks_lock);
#define nf_hooks_write_unlock()			write_unlock_bh(&nf_hooks_lock);

static u32 wechat_uid;
static u32 wechat_param_count;

static u32 nf_hooks_debug = 0;

/* portid of android netlink socket */
static u32 oplus_nf_hooks_pid;
/* kernel sock */
static struct sock *oplus_nf_hooks_sock;


/* send to user space */
static int oplus_nf_hooks_send_to_user(int msg_type, char *payload, int payload_len)
{
	int ret = 0;
	struct sk_buff *skbuff;
	struct nlmsghdr *nlh;

	/*allocate new buffer cache */
	skbuff = alloc_skb(NLMSG_SPACE(payload_len), GFP_ATOMIC);
	if (skbuff == NULL) {
		printk("oplus_nf_hooks_netlink: skbuff alloc_skb failed\n");
		return -1;
	}

	/* fill in the data structure */
	nlh = nlmsg_put(skbuff, 0, 0, msg_type, NLMSG_ALIGN(payload_len), 0);
	if (nlh == NULL) {
		printk("oplus_nf_hooks_netlink:nlmsg_put failaure\n");
		nlmsg_free(skbuff);
		return -1;
	}

	/* compute nlmsg length */
	nlh->nlmsg_len = NLMSG_HDRLEN + NLMSG_ALIGN(payload_len);

	if (NULL != payload) {
		memcpy((char *)NLMSG_DATA(nlh), payload, payload_len);
	}

	/* set control field,sender's pid */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 7, 0))
	NETLINK_CB(skbuff).pid = 0;
#else
	NETLINK_CB(skbuff).portid = 0;
#endif

	NETLINK_CB(skbuff).dst_group = 0;

	/* send data */
	if (oplus_nf_hooks_pid) {
		ret = netlink_unicast(oplus_nf_hooks_sock, skbuff, oplus_nf_hooks_pid, MSG_DONTWAIT);
	} else {
		printk(KERN_ERR "oplus_nf_hooks_netlink: can not unicast skbuff, oplus_nf_hooks_pid=0\n");
		kfree_skb(skbuff);
	}
	if (ret < 0) {
		printk(KERN_ERR "oplus_nf_hooks_netlink: can not unicast skbuff,ret = %d\n", ret);
		return 1;
	}

	return 0;
}


static bool is_wechat_skb(struct nf_conn *ct, struct sk_buff *skb)
{
	kuid_t sk_uid;
	struct sock *sk = NULL;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
	const struct file *filp = NULL;
#endif

	if (ct->oplus_app_uid == -1 || wechat_uid == 0) {
		return false;
	} else if (ct->oplus_app_uid == 0) {
		sk = skb_to_full_sk(skb);
		if (NULL == sk || !sk_fullsock(sk) || NULL == sk->sk_socket) {
			return false;
		}
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
		filp = sk->sk_socket->file;
		if (NULL == filp) {
			return false;
		}
		sk_uid = filp->f_cred->fsuid;
#else
		sk_uid = sk->sk_uid;
#endif

		if ((sk_uid.val % UID_MASK) == (wechat_uid % UID_MASK)) {
		    ct->oplus_app_uid = wechat_uid;
		    /* if (nf_hooks_debug) printk("oplus_nf_hooks_lm:this is wechat skb...\n"); */
		    return true;
		} else {
	        ct->oplus_app_uid = -1;
	        /* if (nf_hooks_debug) printk("oplus_nf_hooks_lm:this is NOT wechat skb!!!\n"); */
	        return false;
	    }
	} else if ((ct->oplus_app_uid % UID_MASK) == (wechat_uid % UID_MASK)) {
		return true;
	}

	return false;
}

/*
 *To detect incoming Lucky Money event.
*/
static unsigned int oplus_nf_hooks_lm_detect(void *priv,
				      struct sk_buff *skb,
				      const struct nf_hook_state *state)
{
	struct nf_conn *ct = NULL;
	enum ip_conntrack_info ctinfo;
	struct iphdr *iph = NULL;
	struct tcphdr *tcph = NULL;
	u32 header_len, i;
	u8 *payload = NULL;
	u16 tot_len;

	ct = nf_ct_get(skb, &ctinfo);

	if (NULL == ct) {
		return NF_ACCEPT;
	}

	if ((iph = ip_hdr(skb)) != NULL && iph->protocol == IPPROTO_TCP) {
	    if (is_wechat_skb(ct, skb)) {
			tot_len = ntohs(iph->tot_len);
			if (unlikely(skb_linearize(skb))) {
				return NF_ACCEPT;
			}
			iph = ip_hdr(skb);
			tcph = tcp_hdr(skb);
			header_len = iph->ihl * 4 + tcph->doff * 4;
			payload = (u8 *)(skb->data + header_len);
			for (i = 0; i < wechat_param_count; i++) {
				if (tot_len >= wechat_infos[i].min_tot_len && tot_len <= wechat_infos[i].max_tot_len) {
					if (memcmp(payload + wechat_infos[i].offset, wechat_infos[i].fixed_value, wechat_infos[i].len) == 0) {
						printk("oplus_nf_hooks_lm:i=%d received hong bao...\n", i);
						oplus_nf_hooks_send_to_user(NF_HOOKS_LM_DETECTED, NULL, 0);
						break;
					} else {
						if (nf_hooks_debug) printk("oplus_nf_hooks_lm:i=%d fixed value not match!!\n", i);
					}
				} else {
					if (nf_hooks_debug) printk("oplus_nf_hooks_lm:i=%d incorrect tot_len=%d\n", i, tot_len);
				}
			}
		}
	}

	return NF_ACCEPT;
}

/*
    *To detect incoming Lucky Money event with IPv6.
    */
static unsigned int oplus_nf_hooks_v6_lm_detect(void *priv,
						struct sk_buff *skb,
						const struct nf_hook_state *state)
{
		struct nf_conn *ct = NULL;
		enum ip_conntrack_info ctinfo;
		struct ipv6hdr *ipv6h = NULL;
		struct tcphdr *tcph = NULL;
		u32 header_len, i;
		__be16 fo = 0;
		u8 ip_proto;
		int ihl = 0;
		u8 *payload = NULL;
		u16 tot_len;

		ct = nf_ct_get(skb, &ctinfo);

		if (NULL == ct) {
			return NF_ACCEPT;
		}

		if(skb->protocol == htons(ETH_P_IPV6) && (ipv6h = ipv6_hdr(skb)) != NULL && ipv6h->nexthdr== NEXTHDR_TCP) {
		    if (is_wechat_skb(ct, skb)) {
				tot_len = ntohs(ipv6h->payload_len);
				ip_proto = ipv6h->nexthdr;
				if (unlikely(skb_linearize(skb))) {
					return NF_ACCEPT;
				}

				ihl = ipv6_skip_exthdr(skb, sizeof(struct ipv6hdr), &ip_proto, &fo);    /* ipv6 header length */

				tcph = tcp_hdr(skb);
				if (NULL == tcph) {
					return NF_ACCEPT;
				}

				header_len = ihl + tcph->doff * 4;  /* total length of ipv6 header and tcp header */
				payload = (u8 *)(skb->data + header_len);   /* tcp payload buffer */
				for (i = 0; i < wechat_param_count; i++) {
					if (tot_len >= wechat_infos[i].min_tot_len && tot_len <= wechat_infos[i].max_tot_len) {
						if (memcmp(payload + wechat_infos[i].offset, wechat_infos[i].fixed_value, wechat_infos[i].len) == 0) {
							printk("oplus_nf_hooks_lm:i=%d received hong bao from ipv6...\n", i);
							oplus_nf_hooks_send_to_user(NF_HOOKS_LM_DETECTED, NULL, 0);
							break;
						} else {
							if (nf_hooks_debug) printk("oplus_nf_hooks_lm:i=%d fixed value not match from ipv6!!\n", i);
						}
					} else {
						if (nf_hooks_debug) printk("oplus_nf_hooks_lm:i=%d incorrect tot_len=%d from ipv6\n", i, tot_len);
					}
				}
			}
		}
		return NF_ACCEPT;
}


static struct nf_hook_ops oplus_nf_hooks_ops[] __read_mostly = {
	{
		.hook		= oplus_nf_hooks_lm_detect,
		.pf		    = NFPROTO_IPV4,
		.hooknum	= NF_INET_LOCAL_IN,
		.priority	= NF_IP_PRI_FILTER + 1,
	},
	{
		.hook       = oplus_nf_hooks_v6_lm_detect,
		.pf         = NFPROTO_IPV6,
		.hooknum    = NF_INET_LOCAL_IN,
		.priority   = NF_IP6_PRI_FILTER + 1,
	},
};

static struct ctl_table oplus_nf_hooks_sysctl_table[] = {
	{
		.procname	= "wechat_uid",
		.data		= &wechat_uid,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec,
	},
	{
		.procname	= "nf_hooks_debug",
		.data		= &nf_hooks_debug,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec,
	},
	{ }
};

static int oplus_nf_hooks_sysctl_init(void)
{
	oplus_nf_hooks_table_hrd = register_net_sysctl(&init_net, "net/oplus_nf_hooks",
		                                          oplus_nf_hooks_sysctl_table);
	return oplus_nf_hooks_table_hrd == NULL ? -ENOMEM : 0;
}

static int oplus_nf_hooks_set_android_pid(struct sk_buff *skb)
{
	oplus_nf_hooks_pid = NETLINK_CB(skb).portid;
	printk("oplus_nf_hooks_set_android_pid pid=%d\n", oplus_nf_hooks_pid);
	return 0;
}

static int oplus_nf_hooks_set_wechat_param(struct nlmsghdr *nlh)
{
	int i, j;
	u32 *data;
	struct wechat_pattern_info * info = NULL;
	data = (u32 *)NLMSG_DATA(nlh);
	wechat_uid = *data;
	wechat_param_count = *(data + 1);
	if (nlh->nlmsg_len == NLMSG_HDRLEN + 2*sizeof(u32) + wechat_param_count*sizeof(struct wechat_pattern_info)) {
		for (i = 0; i < wechat_param_count && i < MAX_WECHAT_PARAMS; i++) {
			info = (struct wechat_pattern_info *)(data + 2) + i;
			memset(&(wechat_infos[i]), 0, sizeof(struct wechat_pattern_info));
			/* wechat_infos[i].uid = info->uid; */
			wechat_infos[i].max_tot_len = info->max_tot_len;
			wechat_infos[i].min_tot_len = info->min_tot_len;
			wechat_infos[i].offset = info->offset;
			wechat_infos[i].len = info->len;
			memcpy(wechat_infos[i].fixed_value, info->fixed_value, info->len);
			if (nf_hooks_debug) {
				printk("oplus_nf_hooks_set_wechat_param i=%d uid=%d,max=%d,min=%d,offset=%d,len=%d,value=",
					i, wechat_uid, wechat_infos[i].max_tot_len, wechat_infos[i].min_tot_len,
						wechat_infos[i].offset, wechat_infos[i].len);
				for (j = 0; j < wechat_infos[i].len; j++) {
					printk("%d -> %02x  ", j, wechat_infos[i].fixed_value[j]);
				}
				printk("\n");
			}
		}
		return 0;
	} else {
	    if (nf_hooks_debug) printk("oplus_nf_hooks_set_wechat_param invalid param!! nlmsg_len=%d\n", nlh->nlmsg_len);
	    return -1;
	}
}

static int nf_hooks_netlink_rcv_msg(struct sk_buff *skb, struct nlmsghdr *nlh, struct netlink_ext_ack *extack)
{
	int ret = 0;

	switch (nlh->nlmsg_type) {
	case NF_HOOKS_ANDROID_PID:
		ret = oplus_nf_hooks_set_android_pid(skb);
		break;
	case NF_HOOKS_WECHAT_PARAM:
		ret = oplus_nf_hooks_set_wechat_param(nlh);
		break;
	default:
		return -EINVAL;
	}
	return ret;
}


static void nf_hooks_netlink_rcv(struct sk_buff *skb)
{
	mutex_lock(&nf_hooks_netlink_mutex);
	netlink_rcv_skb(skb, &nf_hooks_netlink_rcv_msg);
	mutex_unlock(&nf_hooks_netlink_mutex);
}

static int oplus_nf_hooks_netlink_init(void)
{
	struct netlink_kernel_cfg cfg = {
		.input	= nf_hooks_netlink_rcv,
	};

	oplus_nf_hooks_sock = netlink_kernel_create(&init_net, NETLINK_OPLUS_NF_HOOKS, &cfg);
	return oplus_nf_hooks_sock == NULL ? -ENOMEM : 0;
}

static void oplus_nf_hooks_netlink_exit(void)
{
	netlink_kernel_release(oplus_nf_hooks_sock);
	oplus_nf_hooks_sock = NULL;
}

static int __init oplus_nf_hooks_init(void)
{
	int ret = 0;

	ret = oplus_nf_hooks_netlink_init();
	if (ret < 0) {
		printk("oplus_nf_hooks_init module failed to init netlink.\n");
	} else {
		printk("oplus_nf_hooks_init module init netlink successfully.\n");
	}

	ret |= oplus_nf_hooks_sysctl_init();

	ret |= nf_register_net_hooks(&init_net, oplus_nf_hooks_ops, ARRAY_SIZE(oplus_nf_hooks_ops));
	if (ret < 0) {
		printk("oplus_nf_hooks_init module failed to register netfilter ops.\n");
	} else {
		printk("oplus_nf_hooks_init module register netfilter ops successfully.\n");
	}

	return ret;
}

static void __exit oplus_nf_hooks_fini(void)
{
	rwlock_init(&nf_hooks_lock);
	oplus_nf_hooks_netlink_exit();
	if (oplus_nf_hooks_table_hrd) {
		unregister_net_sysctl_table(oplus_nf_hooks_table_hrd);
	}
	nf_unregister_net_hooks(&init_net, oplus_nf_hooks_ops, ARRAY_SIZE(oplus_nf_hooks_ops));
}

module_init(oplus_nf_hooks_init);
module_exit(oplus_nf_hooks_fini);
