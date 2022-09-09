/******************************************************************************
** Copyright (C), 2019-2029, OPLUS Mobile Comm Corp., Ltd
** OPLUS_FEATURE, All rights reserved.
** File: - opluswificap center.c
** Description: wificapcenter (wcc)
**
** Version: 1.0
** Date : 2020/07/05
** TAG: OPLUS_FEATURE_WIFI_CAPCENTER
** ------------------------------- Revision History: ----------------------------
** <author>                                <data>        <version>       <desc>
** ------------------------------------------------------------------------------
 *******************************************************************************/
#include <linux/types.h>
#include <linux/ip.h>
#include <linux/netfilter.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/icmp.h>
#include <linux/sysctl.h>
#include <net/route.h>
#include <net/ip.h>
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

#include <net/oplus/oplus_wfd_wlan.h>

#define LOG_TAG "[oplus_wificapcenter] %s line:%d "
#define debug(fmt, args...) printk(LOG_TAG fmt, __FUNCTION__, __LINE__, ##args)

/*NLMSG_MIN_TYPE is 0x10,so we start at 0x11*/
/*If response event is close contact with request,*/
/*response  can be same with req to reduce msg*/
enum{
        /*common msg for sync and async from 0x11-0x29*/
        OPLUS_COMMON_MSG_BASE                    = 0x11,
	OPLUS_WIFI_CAP_CENTER_NOTIFY_PID	        = OPLUS_COMMON_MSG_BASE,

        /*sync msg from 0x30-0x79;*/
	OPLUS_SYNC_MSG_BASE                      = 0x30,
	OPLUS_SAMPLE_SYNC_GET	                = OPLUS_SYNC_MSG_BASE,
	OPLUS_SAMPLE_SYNC_GET_NO_RESP	        = OPLUS_SYNC_MSG_BASE + 1,

	OPLUS_SYNC_REMOVE_HE_IE_FROM_PROBE_REQUEST = OPLUS_SYNC_MSG_BASE + 2,
	OPLUS_SYNC_DBS_CAPACITY_GET              = OPLUS_SYNC_MSG_BASE + 3,
	OPLUS_SYNC_PHY_CAPACITY_GET 				= OPLUS_SYNC_MSG_BASE + 4,
	OPLUS_SYNC_SUPPORTED_CHANNELS_GET 		= OPLUS_SYNC_MSG_BASE + 5,
	OPLUS_SYNC_AVOID_CHANNELS_GET 			= OPLUS_SYNC_MSG_BASE + 6,

        /*async msg from 0x80-(max-1)*/
        OPLUS_ASYNC_MSG_BASE                     = 0x80,
	OPLUS_SAMPLE_ASYNC_GET	                = OPLUS_ASYNC_MSG_BASE,

	OPLUS_WIFI_CAP_CENTER_MAX                = 0x100,
};

static DEFINE_MUTEX(oplus_wcc_sync_nl_mutex);
static DEFINE_MUTEX(oplus_wcc_async_nl_mutex);
static struct ctl_table_header *oplus_table_hrd;
static rwlock_t oplus_sync_nl_lock;
static rwlock_t oplus_async_nl_lock;
static u32 oplus_wcc_debug = 1;
/*user space pid*/
static u32 oplus_sync_nl_pid = 0;
static u32 oplus_async_nl_pid = 0;
/*kernel sock*/
static struct sock *oplus_sync_nl_sock;
static struct sock *oplus_async_nl_sock;
static struct timer_list oplus_timer;
static int async_msg_type = 0;

/*check msg_type in range of sync & async, 1 stands in range, 0 not in range*/
static int check_msg_in_range(struct sock *nl_sock, int msg_type)
{
        debug("nl_sock: %p, msg_type:%d", nl_sock, msg_type);
        if (msg_type >= OPLUS_COMMON_MSG_BASE && msg_type < OPLUS_SYNC_MSG_BASE) {
                return 1;
        }

        /*not in common part*/
        if (nl_sock == oplus_sync_nl_sock) {
                if (msg_type >= OPLUS_SYNC_MSG_BASE && msg_type< OPLUS_ASYNC_MSG_BASE) {
                        return 1;
                } else {
                        return 0;
                }
        } else if (nl_sock == oplus_async_nl_sock) {
                if (msg_type >= OPLUS_ASYNC_MSG_BASE && msg_type < OPLUS_WIFI_CAP_CENTER_MAX) {
                        return 1;
                } else {
                        return 0;
                }
        } else {
                return 0;
        }
}

/* send to user space */
static int oplus_wcc_send_to_user(struct sock *oplus_sock,
        u32 oplus_pid, int msg_type, char *payload, int payload_len)
{
	int ret = 0;
	struct sk_buff *skbuff;
	struct nlmsghdr *nlh = NULL;

	if (!check_msg_in_range(oplus_sock, msg_type)) {
	        debug("msg_type:%d not in range\n", msg_type);
	        return -1;
	}

	/*allocate new buffer cache */
	skbuff = alloc_skb(NLMSG_SPACE(payload_len), GFP_ATOMIC);
	if (skbuff == NULL) {
		printk("oplus_wcc_netlink: skbuff alloc_skb failed\n");
		return -1;
	}

	/* fill in the data structure */
	nlh = nlmsg_put(skbuff, 0, 0, msg_type, NLMSG_ALIGN(payload_len), 0);
	if (nlh == NULL) {
		printk("oplus_wcc_netlink:nlmsg_put failaure\n");
		nlmsg_free(skbuff);
		return -1;
	}

	/*compute nlmsg length*/
	nlh->nlmsg_len = NLMSG_HDRLEN + NLMSG_ALIGN(payload_len);

	if(NULL != payload) {
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
	if(oplus_pid) {
		ret = netlink_unicast(oplus_sock, skbuff, oplus_pid, MSG_DONTWAIT);
	} else {
		printk(KERN_ERR "oplus_wcc_netlink: can not unicast skbuff, oplus_pid=0\n");
		kfree_skb(skbuff);
	}
	if(ret < 0) {
		printk(KERN_ERR "oplus_wcc_netlink: can not unicast skbuff,ret = %d\n", ret);
		return 1;
	}

	return 0;
}

static void oplus_wcc_sample_resp(struct sock *oplus_sock, u32 oplus_pid, int msg_type)
{
        int payload[4];
        payload[0] = 5;
        payload[1] = 6;
        payload[2] = 7;
        payload[3] = 8;

        oplus_wcc_send_to_user(oplus_sock, oplus_pid, msg_type, (char *)payload, sizeof(payload));
        if (oplus_wcc_debug) {
                debug("msg_type = %d, sample_resp =%d%d%d%d\n", msg_type, payload[0], payload[1], payload[2], payload[3]);
        }

        return;
}

static int oplus_wcc_sample_sync_get(struct nlmsghdr *nlh)
{
	u32 *data = (u32 *)NLMSG_DATA(nlh);

	debug("sample_sync_get: %u%u%u%u\n", data[0], data[1], data[2], data[3]);
	oplus_wcc_sample_resp(oplus_sync_nl_sock, oplus_sync_nl_pid, OPLUS_SAMPLE_SYNC_GET);
	return 0;
}

static int oplus_wcc_sample_sync_get_no_resp(struct nlmsghdr *nlh)
{
	u32 *data = (u32 *)NLMSG_DATA(nlh);

	debug("sample_sync_get_no_resp: %u%u%u%u\n", data[0], data[1], data[2], data[3]);
	return 0;
}

/*#ifdef OPLUS_FEATURE_WIFI_OPLUSWFD*/
#define OPLUS_WFD_FREQS_NUM_MAX 100
/*the first one for length*/
static int s_oplus_wfd_freqs[OPLUS_WFD_FREQS_NUM_MAX + 1];
enum oplus_wfd_band
{
	oplus_wfd_band_2g = 0,
	oplus_wfd_band_5g,
	oplus_wfd_band_6g,
	oplus_wfd_band_max
};

static void remove_he_ie_from_probe_request_stub(int remove) {
	debug("remove_he_ie_from_probe_request_stub");
}

static int get_dbs_capacity_stub() {
	debug("get_dbs_capacity_stub");
	return 0;
}

static int get_phy_capacity_stub(int band)
{
	debug("get_phy_capacity_stub");
	return 0;
}

static void get_supported_channels_stub(int band, int *len, int *freqs, int max_num)
{
	debug("get_supported_channels_stub");
	*len = 0;
}
static void get_avoid_channels_stub(int *len, int *freqs, int max_num)
{
	debug("get_avoid_channels_stub");
	*len = 0;
}

struct oplus_wfd_wlan_ops_t oplus_wfd_wlan_ops = {
	.remove_he_ie_from_probe_request = remove_he_ie_from_probe_request_stub,
	.get_dbs_capacity = get_dbs_capacity_stub,
	.get_phy_capacity = get_phy_capacity_stub,
	.get_supported_channels = get_supported_channels_stub,
	.get_avoid_channels = get_avoid_channels_stub
};

void register_oplus_wfd_wlan_ops(struct oplus_wfd_wlan_ops_t *ops) {
	if (ops == NULL)
		return;
	if (ops->remove_he_ie_from_probe_request)
		oplus_wfd_wlan_ops.remove_he_ie_from_probe_request = ops->remove_he_ie_from_probe_request;
	if (ops->get_dbs_capacity)
		oplus_wfd_wlan_ops.get_dbs_capacity = ops->get_dbs_capacity;
	if (ops->get_phy_capacity)
		oplus_wfd_wlan_ops.get_phy_capacity = ops->get_phy_capacity;
	if (ops->get_supported_channels)
		oplus_wfd_wlan_ops.get_supported_channels = ops->get_supported_channels;
	if (ops->get_avoid_channels)
		oplus_wfd_wlan_ops.get_avoid_channels = ops->get_avoid_channels;
}

static void oplus_wcc_remove_He_IE_from_probe_request(struct nlmsghdr *nlh)
{
	if (nlmsg_len(nlh) > 0) {
		u32 *data = (u32 *)NLMSG_DATA(nlh);
		debug("remove he from probe rquest: %d", *data);
		oplus_wfd_wlan_ops.remove_he_ie_from_probe_request(*data);
	}
}

static void oplus_wcc_get_dbs_capacity(struct nlmsghdr *nlh)
{
	s32 cap = oplus_wfd_wlan_ops.get_dbs_capacity();
	oplus_wcc_send_to_user(oplus_sync_nl_sock, oplus_sync_nl_pid, OPLUS_SYNC_DBS_CAPACITY_GET, (char*)&cap, sizeof(cap));
}
static void oplus_wcc_get_phy_capacity(struct nlmsghdr *nlh)
{
	u32 band = 0;
	u32 cap = 0;
	if (nlmsg_len(nlh) > 0) {
		u32* data = (u32 *)NLMSG_DATA(nlh);
		band = *data;
		if (band < oplus_wfd_band_max) {
			cap = oplus_wfd_wlan_ops.get_phy_capacity(band);
			debug("oplus_wcc_get_phy_capacity, cap= %d", cap);
		}
	}
	oplus_wcc_send_to_user(oplus_sync_nl_sock, oplus_sync_nl_pid, OPLUS_SYNC_PHY_CAPACITY_GET, (char*)&cap, sizeof(cap));
}

static void oplus_wcc_get_supported_channels(struct nlmsghdr *nlh)
{
	u32 band = 0;
	u32 len = 0;

	memset(s_oplus_wfd_freqs, 0, sizeof(s_oplus_wfd_freqs));
	if (nlmsg_len(nlh) > 0) {
		u32* data = (u32 *)NLMSG_DATA(nlh);
		band = *data;
		if (band < oplus_wfd_band_max) {
			oplus_wfd_wlan_ops.get_supported_channels(band, &len, s_oplus_wfd_freqs + 1, OPLUS_WFD_FREQS_NUM_MAX);
			s_oplus_wfd_freqs[0] = len;
			debug("get supported channels, num = %d", len);
		}
	}
	oplus_wcc_send_to_user(oplus_sync_nl_sock, oplus_sync_nl_pid, OPLUS_SYNC_SUPPORTED_CHANNELS_GET, (char*)s_oplus_wfd_freqs, (len + 1)*sizeof(u32));
}

static void oplus_wcc_get_avoid_channels(struct nlmsghdr *nlh)
{
	u32 len = 0;

	memset(s_oplus_wfd_freqs, 0, sizeof(s_oplus_wfd_freqs));

	oplus_wfd_wlan_ops.get_avoid_channels(&len, s_oplus_wfd_freqs + 1, OPLUS_WFD_FREQS_NUM_MAX);
	s_oplus_wfd_freqs[0] = len;
	debug("get avoid channels, num = %d", len);

	oplus_wcc_send_to_user(oplus_sync_nl_sock, oplus_sync_nl_pid, OPLUS_SYNC_AVOID_CHANNELS_GET, (char*)s_oplus_wfd_freqs, (len + 1)*sizeof(u32));
}

EXPORT_SYMBOL(register_oplus_wfd_wlan_ops);

/*#endif OPLUS_FEATURE_WIFI_OPLUSWFD*/
static int oplus_wcc_sample_async_get(struct nlmsghdr *nlh)
{
	u32 *data = (u32 *)NLMSG_DATA(nlh);

        async_msg_type = OPLUS_SAMPLE_ASYNC_GET;
	oplus_timer.expires = jiffies + HZ;/* timer expires in ~1s*/
	add_timer(&oplus_timer);

	debug("sample_async_set: %u%u%u%u\n", data[0], data[1], data[2], data[3]);

	return 0;
}

static void oplus_wcc_timer_function(void)
{
        if (async_msg_type == OPLUS_SAMPLE_ASYNC_GET) {
                oplus_wcc_sample_resp(oplus_async_nl_sock, oplus_async_nl_pid, OPLUS_SAMPLE_ASYNC_GET);
                async_msg_type = 0;
        }
}

static void oplus_wcc_timer_init(void)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 19, 0))
	init_timer(&oplus_timer);
	oplus_timer.function = (void*)oplus_wcc_timer_function;
#else
	timer_setup(&oplus_timer, (void*)oplus_wcc_timer_function, 0);
#endif
}

static void oplus_wcc_timer_fini(void)
{
	del_timer(&oplus_timer);
}

static int oplus_wcc_sync_nl_rcv_msg(struct sk_buff *skb, struct nlmsghdr *nlh, struct netlink_ext_ack *extack)
{
	u32 portid = NETLINK_CB(skb).portid;

	if (nlh->nlmsg_type == OPLUS_WIFI_CAP_CENTER_NOTIFY_PID) {
	        oplus_sync_nl_pid = portid;
	        debug("oplus_sync_nl_pid pid=%d\n", oplus_sync_nl_pid);
	}
	/* only recv msg from target pid*/
	if (portid != oplus_sync_nl_pid) {
		return -1;
	}
	if (!check_msg_in_range(oplus_sync_nl_sock, nlh->nlmsg_type)) {
	        debug("msg_type:%d not in range\n", nlh->nlmsg_type);
	        return -1;
	}

        switch (nlh->nlmsg_type) {
        case OPLUS_SAMPLE_SYNC_GET:
                oplus_wcc_sample_sync_get(nlh);
                break;
        case OPLUS_SAMPLE_SYNC_GET_NO_RESP:
                oplus_wcc_sample_sync_get_no_resp(nlh);
                break;
        case OPLUS_SYNC_REMOVE_HE_IE_FROM_PROBE_REQUEST:
                oplus_wcc_remove_He_IE_from_probe_request(nlh);
                break;
        case OPLUS_SYNC_DBS_CAPACITY_GET:
                oplus_wcc_get_dbs_capacity(nlh);
                break;
        case OPLUS_SYNC_PHY_CAPACITY_GET:
                oplus_wcc_get_phy_capacity(nlh);
                break;
        case OPLUS_SYNC_SUPPORTED_CHANNELS_GET:
                oplus_wcc_get_supported_channels(nlh);
                break;
        case OPLUS_SYNC_AVOID_CHANNELS_GET:
                oplus_wcc_get_avoid_channels(nlh);
                break;
        default:
                return -EINVAL;
        }

	return 0;
}

static int oplus_wcc_async_nl_rcv_msg(struct sk_buff *skb, struct nlmsghdr *nlh, struct netlink_ext_ack *extack)
{
	u32 portid = NETLINK_CB(skb).portid;

	if (nlh->nlmsg_type == OPLUS_WIFI_CAP_CENTER_NOTIFY_PID) {
	        oplus_async_nl_pid = portid;
	        debug("oplus_async_nl_pid pid=%d\n", oplus_async_nl_pid);
	}
	/* only recv msg from target pid*/
	if (portid != oplus_async_nl_pid) {
		return -1;
	}
	if (!check_msg_in_range(oplus_async_nl_sock, nlh->nlmsg_type)) {
	        debug("msg_type:%d not in range\n", nlh->nlmsg_type);
	        return -1;
	}

        switch (nlh->nlmsg_type) {
        case OPLUS_SAMPLE_ASYNC_GET:
                oplus_wcc_sample_async_get(nlh);
                break;
        default:
                return -EINVAL;
        }

	return 0;
}

static void oplus_wcc_sync_nl_rcv(struct sk_buff *skb)
{
	mutex_lock(&oplus_wcc_sync_nl_mutex);
	netlink_rcv_skb(skb, &oplus_wcc_sync_nl_rcv_msg);
	mutex_unlock(&oplus_wcc_sync_nl_mutex);
}

static void oplus_wcc_async_nl_rcv(struct sk_buff *skb)
{
	mutex_lock(&oplus_wcc_async_nl_mutex);
	netlink_rcv_skb(skb, &oplus_wcc_async_nl_rcv_msg);
	mutex_unlock(&oplus_wcc_async_nl_mutex);
}

static int oplus_wcc_netlink_init(void)
{
	struct netlink_kernel_cfg cfg1 = {
		.input	= oplus_wcc_sync_nl_rcv,
	};
	struct netlink_kernel_cfg cfg2 = {
		.input	= oplus_wcc_async_nl_rcv,
	};

	oplus_sync_nl_sock = netlink_kernel_create(&init_net, NETLINK_OPLUS_WIFI_CAP_CENTER_SYNC, &cfg1);
	oplus_async_nl_sock = netlink_kernel_create(&init_net, NETLINK_OPLUS_WIFI_CAP_CENTER_ASYNC, &cfg2);
	debug("oplus_sync_nl_sock = %p,  oplus_async_nl_sock = %p\n", oplus_sync_nl_sock, oplus_async_nl_sock);

	if (oplus_sync_nl_sock == NULL || oplus_async_nl_sock == NULL) {
		return -ENOMEM;
	} else {
	        return 0;
        }
}

static void oplus_wcc_netlink_exit(void)
{
	netlink_kernel_release(oplus_sync_nl_sock);
	oplus_sync_nl_sock = NULL;

	netlink_kernel_release(oplus_async_nl_sock);
	oplus_async_nl_sock = NULL;
}

static struct ctl_table oplus_wcc_sysctl_table[] = {
	{
		.procname	= "oplus_wcc_debug",
		.data		= &oplus_wcc_debug,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec,
	},
	{ }
};

static int oplus_wcc_sysctl_init(void)
{
	oplus_table_hrd = register_net_sysctl
	        (&init_net, "net/oplus_wcc", oplus_wcc_sysctl_table);
	return oplus_table_hrd == NULL ? -ENOMEM : 0;
}

static void oplus_wcc_sysctl_fini(void)
{
	if(oplus_table_hrd) {
		unregister_net_sysctl_table(oplus_table_hrd);
		oplus_table_hrd = NULL;
	}
}

static int __init oplus_wcc_init(void)
{
	int ret = 0;
	rwlock_init(&oplus_sync_nl_lock);
	rwlock_init(&oplus_async_nl_lock);

	ret = oplus_wcc_netlink_init();
	if (ret < 0) {
		debug("oplus_wcc_init module failed to init netlink.\n");
	} else {
		debug("oplus_wcc_init module init netlink successfully.\n");
	}

	ret = oplus_wcc_sysctl_init();
	if (ret < 0) {
		debug("oplus_wcc_init module failed to init sysctl.\n");
	}
        else {
                debug("oplus_wcc_init module init sysctl successfully.\n");
        }

	oplus_wcc_timer_init();

	return ret;
}

static void __exit oplus_wcc_fini(void)
{
	oplus_wcc_sysctl_fini();
	oplus_wcc_netlink_exit();
	oplus_wcc_timer_fini();
}

module_init(oplus_wcc_init);
module_exit(oplus_wcc_fini);

