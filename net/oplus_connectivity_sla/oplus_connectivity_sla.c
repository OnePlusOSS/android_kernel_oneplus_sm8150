/************************************************************************************
** File: - oplus_connectivity_sla.c
** Copyright (C), 2008-2020, OPLUS Mobile Comm Corp., Ltd
**
** Description:
**		1. Add for SLA GKI
**
** Version: 1.0
** Date :	2021-04-19
** TAG	:	OPLUS_FEATURE_WIFI_CONNECTIVITY_SLA
**
** ---------------------Revision History: ---------------------
**	<author>					  <data>	 <version >   <desc>
** ---------------------------------------------------------------
**
************************************************************************************/
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
#include <linux/workqueue.h>
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

#include <net/genetlink.h>
#define LOG_TAG "[oplus_connectivity_sla] %s line:%d "
#define debug(fmt,args...) printk(LOG_TAG fmt,__FUNCTION__,__LINE__,##args)

#define IFACE_NUM       3
#define WLAN_NUM        2
#define WLAN0_INDEX     0
#define WLAN1_INDEX     1
#define CELL_INDEX      2

#define MARK_MASK    0x0fff
#define GAME_UNSPEC_MASK 0x8000
#define CT_MARK_APP_TYPE_MASK 0xfff00000   //mark for detect white list
#define CT_MARK_APP_TYPE_OFFSET 20

#define GAME_NUM 7
#define IFACE_LEN 16
#define WHITE_APP_BASE    100
#define DUAL_STA_APP_BASE 200
#define WHITE_APP_NUM     64
#define DUAL_STA_APP_NUM  256

//#define DNS_TIME            10

#define UID_MASK   100000
#define INIT_APP_TYPE      0

#define OPLUS_SLA_CMD_MAX (__OPLUS_SLA_CMD_MAX - 1)
#define OPLUS_SLA_FAMILY_VERSION	1
#define OPLUS_SLA_FAMILY "oplus_sla"
#define NLA_DATA(na)		((char *)((char*)(na) + NLA_HDRLEN))
#define GENL_ID_GENERATE	0

enum {
	OPLUS_SLA_CMD_UNSPEC                      = 1,
	OPLUS_SLA_CMD_SET_ANDROID_PID             = 2,
	OPLUS_SLA_CMD_ENABLE                      = 3,
	OPLUS_SLA_CMD_DISABLE                     = 4,
	OPLUS_SLA_CMD_IFACE_CHANGED               = 5,
	OPLUS_SLA_CMD_SET_WHITE_LIST_APP          = 6,
	OPLUS_SLA_CMD_SET_DUAL_STA_LIST_APP       = 7,
	OPLUS_SLA_CMD_SET_GAME_LIST_APP           = 8,
	OPLUS_SLA_CMD_SET_VIDEO_LIST_APP          = 9,
	OPLUS_SLA_CMD_CHANGE_GAME_NETWORK         = 10,
	OPLUS_SLA_CMD_CHANGE_VIDEO_APP_NETWORK    = 11,
	OPLUS_SLA_CMD_CHANGE_DNS_NETWORK          = 12,
	OPLUS_SLA_CMD_UPDATE_WEIGHT               = 13,
	OPLUS_SLA_CMD_SET_KERNEL_DEBUG            = 14,
	OPLUS_SLA_CMD_CHANGE_VPN_STATE            = 15,
	OPLUS_SLA_CMD_CHANGE_DEFAULT_NETWORK      = 16,
	OPLUS_SLA_ENABLED_EVENT                   = 18,
	OPLUS_SLA_DISABLED_EVENT                  = 19,
	OPLUS_SLA_GAME_NETWORK_CHANGED_EVENT      = 20,
	__SLA_MSG_MAX,
};

enum {
	OPLUS_SLA_UNSPEC,
	OPLUS_SLA_CMD_DOWNLINK,
	OPLUS_SLA_CMD_UPLINK,
	__OPLUS_SLA_CMD_MAX
};


#define OPLUS_SLA_MSG_MAX (__SLA_MSG_MAX - 1)


/* dev info struct
  * if we need to consider wifi RSSI ?if we need consider screen state?
*/
struct oplus_dev_info {
	int if_up;
	int weight;
	//int weight_state;
	u32 mark;
	char dev_name[IFACE_LEN];
};

struct oplus_white_app_info {
	u32 count;
	u32 uid[WHITE_APP_NUM];
	u64 cell_bytes[WHITE_APP_NUM];
	u64 cell_bytes_normal[WHITE_APP_NUM];
};

struct oplus_dual_sta_info {
	u32 count;
	u32 uid[DUAL_STA_APP_NUM];
};

struct oplus_sla_game_info {
	u32 game_type;
	u32 uid;
	u32 mark;
};

enum {
	SLA_SKB_ACCEPT,
	SLA_SKB_CONTINUE,
	SLA_SKB_MARKED,
	SLA_SKB_REMARK,
	SLA_SKB_DROP,
};

enum {
	GAME_UNSPEC = 0,
	GAME_WZRY = 1,
	GAME_CJZC,
	GAME_QJCJ,
	GAME_HYXD_NM,
	GAME_HYXD,
	GAME_HYXD_ALI,
};

enum {

	WLAN0_MARK_BIT = 8,            //WLAN mark value,mask 0x0fff
	WLAN0_MARK = (1 << WLAN0_MARK_BIT),

	WLAN1_MARK_BIT = 9,            //WLAN mark value,mask 0x0fff
	WLAN1_MARK = (1 << WLAN1_MARK_BIT),

	CELL_MARK_BIT = 10,       //cellular mark value  mask 0x0fff
	CELL_MARK = (1 << CELL_MARK_BIT),

	RETRAN_BIT = 12,             //first retran mark value,  mask 0xf000
	RETRAN_MARK = (1 << RETRAN_BIT),

	RETRAN_SECOND_BIT = 13,     //second retran mark value, mask 0xf000
	RETRAN_SECOND_MARK = (1 << RETRAN_SECOND_BIT),

	RTT_MARK_BIT = 14,          //one ct only statitisc once rtt,mask 0xf000
	RTT_MARK = (1 << RTT_MARK_BIT),

	GAME_UNSPEC_MARK_BIT = 15,          //mark game skb when game not start
	GAME_UNSPEC_MARK = (1 << GAME_UNSPEC_MARK_BIT),
};

enum {
	SLA_MODE_INIT = 0,
	SLA_MODE_DUAL_WIFI = 1,
	/*if the dual wifi is enable,please do not send
	disable msg to kernel when rcv SLA_MODE_WIFI_CELL msg*/
	SLA_MODE_WIFI_CELL = 2,
	SLA_MODE_DUAL_WIFI_CELL = 3,
	SLA_MODE_RESERVER = 4,
};

static volatile u32 oplus_sla_netlink_pid;
static int MAIN_WLAN = WLAN0_INDEX;
static int SECOND_WLAN = WLAN1_INDEX;

static int oplus_sla_enable_status;
static int oplus_sla_screen_on;
static int oplus_sla_debug = 0;
static int oplus_sla_def_net = 0;    //WLAN0->0 WLAN1->1 CELL->2
static int sla_work_mode = SLA_MODE_INIT;
static int dns_network = WLAN0_INDEX;
static int video_app_network = WLAN0_INDEX; //init

static struct oplus_white_app_info white_app_list;
static struct oplus_dual_sta_info dual_wifi_app_list;
static struct oplus_dual_sta_info video_app_list;
static struct oplus_sla_game_info game_info[GAME_NUM];
static int game_mark = 0; //for proc debug
static int oplus_sla_vpn_connected = 0;
static struct ctl_table_header *oplus_sla_table_hrd;

static rwlock_t sla_lock;
static rwlock_t sla_game_lock;

static int MAIN_WLAN_MARK = WLAN0_MARK;
static int SECOND_WLAN_MARK = WLAN1_MARK;
static struct oplus_dev_info oplus_sla_info[IFACE_NUM];

#define sla_read_lock() 			read_lock_bh(&sla_lock);
#define sla_read_unlock() 			read_unlock_bh(&sla_lock);
#define sla_write_lock() 			write_lock_bh(&sla_lock);
#define sla_write_unlock()			write_unlock_bh(&sla_lock);

#define sla_game_write_lock()       write_lock_bh(&sla_game_lock);
#define sla_game_write_unlock()     write_unlock_bh(&sla_game_lock);



static u32 get_app_type(struct nf_conn *ct)
{
	if (ct != NULL) {
		return (ct->mark & CT_MARK_APP_TYPE_MASK)  >>  CT_MARK_APP_TYPE_OFFSET;
	}

	return 0;
}

static void set_app_type(struct nf_conn *ct, int type)
{
	if (ct != NULL) {
		ct->mark = (ct->mark & ~CT_MARK_APP_TYPE_MASK) | ((type <<
					CT_MARK_APP_TYPE_OFFSET) & CT_MARK_APP_TYPE_MASK);
	}
}

static u32 get_ct_mark(struct nf_conn *ct)
{
	if (ct != NULL) {
		return (ct->mark & ~CT_MARK_APP_TYPE_MASK);
	}
    return 0;
}

static void set_ct_mark(struct nf_conn *ct, int mark)
{
	if (ct != NULL) {
		ct->mark = (ct->mark & ~MARK_MASK) | mark;
	}
}

static void print_stream_info(struct sk_buff *skb)
{
	u32 uid = 0;
	int srcport = 0;
	int dstport = 0;
	unsigned char *dstip;

	struct sock *sk = NULL;
	struct iphdr *iph = NULL;
	struct nf_conn *ct = NULL;
	struct net_device *dev = NULL;
	enum ip_conntrack_info ctinfo;
	const struct file *filp = NULL;

	if (oplus_sla_debug) {
		ct = nf_ct_get(skb, &ctinfo);

		if (NULL == ct) {
			return;
		}

		if (ctinfo == IP_CT_NEW) {
			sk = skb_to_full_sk(skb);

			if (sk && sk_fullsock(sk)) {
				if (NULL == sk->sk_socket) {
					return;
				}

				filp = sk->sk_socket->file;

				if (NULL == filp) {
					return;
				}

				iph = ip_hdr(skb);

				if (NULL != iph &&
					(iph->protocol == IPPROTO_TCP || iph->protocol == IPPROTO_UDP)) {

					dev = skb_dst(skb)->dev;
					uid = filp->f_cred->fsuid.val;
					dstport = ntohs(udp_hdr(skb)->dest);
					srcport = ntohs(udp_hdr(skb)->source);
					dstip = (unsigned char *)&iph->daddr;

					debug("screen_on[%d] uid[%u] proto[%d] srcport[%d] dstport[%d]"
						" dstip[%d.%d.%d.%d] dev[%s] mark[%x] ct_mark[%x] app_type[%d]\n",
						oplus_sla_screen_on, uid, iph->protocol, srcport, dstport,
						dstip[0], dstip[1], dstip[2], dstip[3], dev ? dev->name : "null", skb->mark,
						get_ct_mark(ct), get_app_type(ct));
				}
			}
		}
	}

	return;
}

static bool is_skb_pre_bound(struct sk_buff *skb)
{
	u32 pre_mark = skb->mark & 0x10000;

	if (0x10000 == pre_mark) {
		return true;
	}

	return false;
}

static bool is_sla_white_or_game_app(struct nf_conn *ct, struct sk_buff *skb)
{
	u32 oplus_app_type = get_app_type(ct);
	//debug("type : %u \n", get_app_type(ct));
	if (oplus_app_type > 0 &&
		oplus_app_type < DUAL_STA_APP_BASE) { /* game app skb */
		return true;
	}

	return false;
}

static bool is_dual_sta_white_app(struct nf_conn *ct, struct sk_buff *skb,
	kuid_t *app_uid)
{
	int i = 0;
	int last_type = INIT_APP_TYPE;
	kuid_t uid;
	struct sock *sk = NULL;
	const struct file *filp = NULL;

	//debug("type : %u \n", get_app_type(ct));

	if (get_app_type(ct) >= DUAL_STA_APP_BASE) {
		return true;
	}

	if (INIT_APP_TYPE != get_app_type(ct)) {

		last_type = get_app_type(ct);

		sk = skb_to_full_sk(skb);

		if (NULL == sk || NULL == sk->sk_socket) {
			return false;
		}

		filp = sk->sk_socket->file;

		if (NULL == filp) {
			return false;
		}

		*app_uid = filp->f_cred->fsuid;
		uid = filp->f_cred->fsuid;

		for (i = 0; i < dual_wifi_app_list.count; i++) {
			if (dual_wifi_app_list.uid[i]) {
				if ((uid.val % UID_MASK) == (dual_wifi_app_list.uid[i] % UID_MASK)) {

					set_app_type(ct, i + DUAL_STA_APP_BASE);
					return true;
				}
			}
		}

		set_app_type(ct, last_type);
	}

	return false;
}


static bool is_video_app(kuid_t app_uid)
{
	int i = 0;
	kuid_t uid;

	for (i = 0; i < video_app_list.count; i++) {
		if (video_app_list.uid[i]) {
			uid = make_kuid(&init_user_ns, video_app_list.uid[i]);

			if (uid_eq(app_uid, uid)) {
				return true;
			}
		}
	}

	return false;
}

static int mark_video_app(struct sock *sk,
	kuid_t app_uid,
	struct nf_conn *ct,
	struct sk_buff *skb)
{
	int choose_mark = 0;
	int ret = SLA_SKB_CONTINUE;

	if (SLA_MODE_DUAL_WIFI == sla_work_mode &&
		is_video_app(app_uid)) {
		if (video_app_network == MAIN_WLAN) {
			choose_mark = MAIN_WLAN_MARK;

		} else if (video_app_network == SECOND_WLAN) {
			choose_mark = SECOND_WLAN_MARK;
		}
	}

	if (choose_mark) {
		skb->mark = choose_mark;
		set_ct_mark(ct, skb->mark);
		return SLA_SKB_MARKED;
	}

	return ret;
}


/*
LAN IP:
A:10.0.0.0-10.255.255.255
B:172.16.0.0-172.31.255.255
C:192.168.0.0-192.168.255.255
*/
static bool dst_is_lan_ip(struct sk_buff *skb)
{
	struct iphdr *iph = NULL;
	unsigned char *dstip = NULL;

	iph = ip_hdr(skb);

	if (NULL != iph) {
		dstip = (unsigned char *)&iph->daddr;

		if ((10 == dstip[0]) ||
			(192 == dstip[0] && 168 == dstip[1]) ||
			(172 == dstip[0] && dstip[1] >= 16 && dstip[1] <= 31)) {
			return true;
		}
	}

	return false;
}

static int sla_skb_reroute(struct sk_buff *skb, struct nf_conn *ct,
	const struct nf_hook_state *state)
{
	int err;

	//err = ip_route_me_harder(state->net, state->sk, skb, RTN_UNSPEC);
	err = ip_route_me_harder(state->net, skb, RTN_UNSPEC);

	if (err < 0) {
		return NF_DROP_ERR(err);
	}

	return NF_ACCEPT;
}


static u32 get_skb_mark_by_weight(void)
{
	int i = 0;
	u32 sla_random = prandom_u32() & 0x7FFFFFFF;

	/*0x147AE15 = 0x7FFFFFFF /100 + 1; for we let the weight * 100 to void
	  *decimal point operation at linux kernel
	  */
	for (i = 0; i < IFACE_NUM; i++) {
		if (oplus_sla_info[i].if_up &&
			oplus_sla_info[i].weight) {
			if (sla_random < (0x147AE15 * oplus_sla_info[i].weight)) {
				return oplus_sla_info[i].mark;
			}
		}
	}

	return oplus_sla_info[MAIN_WLAN].mark;
}

static bool is_need_change_dns_network(int index)
{
	if (dns_network != 0) {
		return true;

	} else {
		return false;
	}

}

static int dns_skb_need_sla(struct nf_conn *ct, struct sk_buff *skb)
{
	int ret = SLA_SKB_CONTINUE;
	struct iphdr *iph = NULL;
	u_int32_t dns_ct_mark = MAIN_WLAN_MARK;

	iph = ip_hdr(skb);

	if (NULL != iph &&
		(iph->protocol == IPPROTO_TCP || iph->protocol == IPPROTO_UDP) &&
		53 == ntohs(udp_hdr(skb)->dest)) {

		ret = SLA_SKB_ACCEPT;

		if (SLA_MODE_DUAL_WIFI == sla_work_mode) {
			if (is_need_change_dns_network(SECOND_WLAN)) {
				//for  the dns packet will do DNAT at iptables,if do DNAT,the packet
				//will be reroute,so here just mark and accept it
				dns_ct_mark = SECOND_WLAN_MARK;
				skb->mark = SECOND_WLAN_MARK;
				//todo: same dst ip should do this ,differ dst ip just let it go and reroute by DNAT
				//debug("dual sta reroute skb->mark = %x\n", skb->mark);
				ret = SLA_SKB_MARKED;
			}

		} else if (SLA_MODE_WIFI_CELL == sla_work_mode) {
			if (is_need_change_dns_network(CELL_INDEX)) {
				dns_ct_mark = CELL_MARK;
				skb->mark = CELL_MARK;
				//todo: same dst ip should do this
				//ret = SLA_SKB_MARKED;
			}
		}

		//debug("dns mark = %x\n", skb->mark);
		ct->mark = dns_ct_mark;
	}

	return ret;
}

static bool is_game_app_skb(struct nf_conn *ct, struct sk_buff *skb,
	enum ip_conntrack_info ctinfo)
{
	int i = 0;
	kuid_t uid;
	struct sock *sk = NULL;
	struct iphdr *iph = NULL;
	const struct file *filp = NULL;

	if (INIT_APP_TYPE == get_app_type(ct)) {

		sk = skb_to_full_sk(skb);

		if (NULL == sk || NULL == sk->sk_socket) {
			return false;
		}

		filp = sk->sk_socket->file;

		if (NULL == filp) {
			return false;
		}

		iph = ip_hdr(skb);

		uid = filp->f_cred->fsuid;

		for (i = 1; i < GAME_NUM; i++) {
			if (game_info[i].uid) {
				if ((uid.val % UID_MASK) == (game_info[i].uid % UID_MASK)) {
					set_app_type(ct, i);

					if (game_mark) {
						set_ct_mark(ct, game_mark);

					} else {
						set_ct_mark(ct, game_info[i].mark);
					}

					return true;
				}
			}
		}

	} else if (get_app_type(ct) > 0 && get_app_type(ct) < GAME_NUM) {
		i = get_app_type(ct);
		return true;
	}

	return false;

}

static int detect_game_skb(struct sk_buff *skb)
{
	//struct iphdr *iph = NULL;
	struct nf_conn *ct = NULL;
	int ret = SLA_SKB_ACCEPT;
	enum ip_conntrack_info ctinfo;

	if (oplus_sla_vpn_connected) {
		return SLA_SKB_CONTINUE;
	}

	ct = nf_ct_get(skb, &ctinfo);

	if (NULL == ct) {
		return SLA_SKB_ACCEPT;
	}

	if (!is_game_app_skb(ct, skb, ctinfo)) {
		return SLA_SKB_CONTINUE;
	}

	//TCP and udp need to switch network
	ret = SLA_SKB_CONTINUE;
	return ret;
}


static void detect_white_list_app_skb(struct sk_buff *skb)
{
	int i = 0;
	int index = -1;
	kuid_t uid;
	struct nf_conn *ct = NULL;
	enum ip_conntrack_info ctinfo;
	struct sock *sk = NULL;
	const struct file *filp = NULL;

	ct = nf_ct_get(skb, &ctinfo);

	if (NULL == ct) {
		return;
	}

	/*when the app type is dual sta app,but the work mode is not
	   SLA_MODE_DUAL_WIFI, we should detect it is WIFI+CELL white
	   list app again
	*/
	if (SLA_MODE_DUAL_WIFI != sla_work_mode &&
		get_app_type(ct) >= DUAL_STA_APP_BASE) {
		set_app_type(ct, INIT_APP_TYPE);
	}

	if (INIT_APP_TYPE == get_app_type(ct)) {
		sk = skb_to_full_sk(skb);

		if (NULL == sk || NULL == sk->sk_socket) {
			return;
		}

		filp = sk->sk_socket->file;

		if (NULL == filp) {
			return;
		}

		uid = filp->f_cred->fsuid;

		for (i = 0; i < white_app_list.count; i++) {
			if (white_app_list.uid[i]) {
				if ((uid.val % UID_MASK) == (white_app_list.uid[i] % UID_MASK)) {
					set_app_type(ct, i + WHITE_APP_BASE);
					return;
				}
			}
		}

		/* we need to detect the whether it is dual sta white list app */
		if (SLA_MODE_DUAL_WIFI != sla_work_mode) {
			set_app_type(ct, INIT_APP_TYPE);
		}

	} else if (get_app_type(ct) >= WHITE_APP_BASE &&
		get_app_type(ct) < DUAL_STA_APP_BASE) {
		/*calc white app cell bytes when sla is not enable,
		    when the default network is change to cell,we should
		    disable dual sta from framework
		*/
		if (oplus_sla_info[CELL_INDEX].if_up) {
			if (!oplus_sla_info[MAIN_WLAN].if_up ||
				oplus_sla_def_net == CELL_INDEX) {
				index = get_app_type(ct) - WHITE_APP_BASE;

				if (index < WHITE_APP_NUM) {
					white_app_list.cell_bytes_normal[index] += skb->len;
				}
			}
		}
	}

	return;
}

static int mark_game_app_skb(struct nf_conn *ct, struct sk_buff *skb,
	enum ip_conntrack_info ctinfo)
{
	int game_index = -1;
	struct iphdr *iph = NULL;
	u32 ct_mark = 0;
	int ret = SLA_SKB_CONTINUE;
	int oplus_app_type = get_app_type(ct);

	if (oplus_app_type > 0 && oplus_app_type < GAME_NUM) {
		ret = SLA_SKB_ACCEPT;
		game_index = oplus_app_type;

		if (GAME_WZRY != game_index &&
			GAME_CJZC != game_index) {
			return ret;
		}

		iph = ip_hdr(skb);

		if (iph &&
			(IPPROTO_UDP == iph->protocol ||
				IPPROTO_TCP == iph->protocol)) {

			//WZRY can not switch tcp packets
			if (GAME_WZRY == game_index &&
				IPPROTO_TCP == iph->protocol) {
				return SLA_SKB_ACCEPT;
			}

			ct_mark	= get_ct_mark(ct) & MARK_MASK;

			if (GAME_CJZC == game_index &&
				IPPROTO_TCP == iph->protocol &&
				((XT_STATE_BIT(ctinfo) & XT_STATE_BIT(IP_CT_ESTABLISHED)) ||
					(XT_STATE_BIT(ctinfo) & XT_STATE_BIT(IP_CT_RELATED)))) {
				if (MAIN_WLAN_MARK == ct_mark) {
					return SLA_SKB_ACCEPT;

				} else if (CELL_MARK == ct_mark) {
					skb->mark = CELL_MARK;
					return SLA_SKB_MARKED;
				}
			}

			if (game_mark) {
				skb->mark = game_mark;

			} else {
				skb->mark = game_info[game_index].mark;
			}

			if (ct_mark && skb->mark &&
				ct_mark != skb->mark) {

				debug("oplus_sla_game:reset game ct proto= %u,srcport = %d,"
					"ct dying = %d,ct confirmed = %d,game type = %d,ct mark = %x,skb mark = %x\n",
					iph->protocol, ntohs(udp_hdr(skb)->source),
					nf_ct_is_dying(ct), nf_ct_is_confirmed(ct), game_index, ct_mark, skb->mark);

				if (!nf_ct_is_dying(ct) &&
					nf_ct_is_confirmed(ct)) {
					nf_ct_kill(ct);
					return SLA_SKB_DROP;

				} else {
					skb->mark = ct_mark;
					ret = SLA_SKB_MARKED;
				}
			}

			if (!ct_mark) {
				//set_ct_mark(ct, game_info[game_index].mark);
				set_ct_mark(ct, skb->mark);
			}

			ret = SLA_SKB_MARKED;
		}
	}

	return ret;
}


static int sla_mark_skb(struct sk_buff *skb, const struct nf_hook_state *state)
{
	int ret = SLA_SKB_CONTINUE;
	//int index = 0;
	kuid_t app_uid = {0};
	//u32 ct_mark = 0x0;
	struct sock *sk = NULL;
	struct nf_conn *ct = NULL;
	enum ip_conntrack_info ctinfo;

	//if wlan assistant has change network to cell,do not mark SKB
	if (oplus_sla_def_net == CELL_INDEX) {
		return NF_ACCEPT;
	}

	ct = nf_ct_get(skb, &ctinfo);

	if (NULL == ct) {
		return NF_ACCEPT;
	}

	/*
	  * when the wifi is poor,the dns request allways can not rcv respones,
	  * so please let the dns packet with the cell network mark.
	  */
	ret = dns_skb_need_sla(ct, skb);

	if (SLA_SKB_ACCEPT == ret) {
		return NF_ACCEPT;

	} else if (SLA_SKB_MARKED == ret) {
		goto sla_reroute;
	}

	if (is_skb_pre_bound(skb) || dst_is_lan_ip(skb)) {
		return NF_ACCEPT;
	}

	if (SLA_MODE_WIFI_CELL == sla_work_mode &&
		!is_sla_white_or_game_app(ct, skb)) {
		return NF_ACCEPT;
	}

	if (SLA_MODE_DUAL_WIFI == sla_work_mode &&
		!is_dual_sta_white_app(ct, skb, &app_uid)) {
		return NF_ACCEPT;
	}

	ret = mark_game_app_skb(ct, skb, ctinfo);

	if (SLA_SKB_MARKED == ret) {
		goto sla_reroute;

	} else if (SLA_SKB_ACCEPT == ret) {
		return NF_ACCEPT;

	} else if (SLA_SKB_DROP ==  ret) {
		return NF_DROP;
	}

	if (ctinfo == IP_CT_NEW) {
		sk = skb_to_full_sk(skb);

		if (NULL != sk) {
#if 0
			// add for download app stream
			ret = mark_download_app(sk, app_uid, ct, skb);

			if (SLA_SKB_MARKED == ret) {
				goto sla_reroute;
			}

#endif
			// add for video app stream
			ret = mark_video_app(sk, app_uid, ct, skb);

			if (SLA_SKB_MARKED == ret) {
				goto sla_reroute;
			}

			sla_read_lock();
			skb->mark = get_skb_mark_by_weight();
			sla_read_unlock();
			set_ct_mark(ct, skb->mark);
			if (oplus_sla_debug) {
				debug("skb->mark = %x ct->mark = %x get_ct_mark = %x\n", skb->mark, ct->mark,
					get_ct_mark(ct));
			}
		}

	} else if ((XT_STATE_BIT(ctinfo) & XT_STATE_BIT(IP_CT_ESTABLISHED)) ||
		(XT_STATE_BIT(ctinfo) & XT_STATE_BIT(IP_CT_RELATED))) {

		skb->mark = get_ct_mark(ct) & MARK_MASK;
		/*
		if (oplus_sla_debug) {
			debug("skb->mark = %x ct->mark = %x get_ct_mark = %x\n", skb->mark, ct->mark,
				get_ct_mark(ct));
		}
		*/
	}

//If the mark value of the packet is equal to WLAN0_MARK, no re routing is required
	if (MAIN_WLAN_MARK == skb->mark) {
		return NF_ACCEPT;
	}

#if 0

//calc white list app cell bytes
	if (ct->oplus_app_type >= WHITE_APP_BASE &&
		ct->oplus_app_type < DUAL_STA_APP_BASE) {
		ct_mark = ct->mark & MARK_MASK;

		if (CELL_MARK == ct_mark) {
			index = ct->oplus_app_type - WHITE_APP_BASE;

			if (index < WHITE_APP_NUM) {
				white_app_list.cell_bytes[index] += skb->len;
			}
		}
	}

#endif

sla_reroute:
	ret = sla_skb_reroute(skb, ct, state);
	return ret;

}
/*
oplus sla hook function, mark skb and rerout skb
*/
static unsigned int oplus_sla_output_hook(void *priv,
	struct sk_buff *skb,
	const struct nf_hook_state *state)
{
	int ret = NF_ACCEPT;
	int game_ret = NF_ACCEPT;

	game_ret = detect_game_skb(skb);

	if (SLA_SKB_ACCEPT == game_ret) {
		goto end_sla_output;
	}

	//we need to calc white list app cell bytes when sla not enabled
	detect_white_list_app_skb(skb);

	if (oplus_sla_enable_status) {

		ret = sla_mark_skb(skb, state);

	}/* else {

		if (!oplus_sla_screen_on) {
			goto end_sla_output;
		}
	}*/

end_sla_output:
	//add for android Q statictis tcp tx and tx
	//statistics_wlan_tcp_tx_rx(state, skb);
	print_stream_info(skb);
	return ret;
}

static int TIMESTAMP_ERROR_THRESHOLD = 10;
static int ts_error_count = 0;
static void handle_timestamp_error_case(struct sk_buff *skb, const struct nf_hook_state *state)
{
	struct sock *sk = NULL;
	struct iphdr *iph;
	struct tcp_options_received tcp_opt;
    const __be32 *ptr;

	if ((iph = ip_hdr(skb)) != NULL && iph->protocol == IPPROTO_TCP) {
		sk = skb_to_full_sk(skb);

		if (sk && (!sk_fullsock(sk) || sk->sk_state == TCP_TIME_WAIT || sk->sk_state == TCP_NEW_SYN_RECV)) {
			return;
		}

		if (NULL != sk) {
			struct tcp_sock *tp = tcp_sk(sk);
			struct tcphdr *th = tcp_hdr(skb);

			//handle syn + ack
			if (NULL != tp && NULL != th && th->ack && th->syn) {
				//fast path
				memset(&tcp_opt, 0, sizeof(tcp_opt));
				ptr = (const __be32 *)(th + 1);

				if (*ptr == htonl((TCPOPT_NOP << 24) | (TCPOPT_NOP << 16)
						| (TCPOPT_TIMESTAMP << 8) | TCPOLEN_TIMESTAMP)) {
					ptr += 2;

					if (*ptr) {
						tcp_opt.rcv_tsecr = ntohl(*ptr) - tp->tsoffset;
						tcp_opt.saw_tstamp = 1;
						//debug("fast path rcv_tsecr = %u\n", tcp_opt.rcv_tsecr);
					}

				}/*else {
					//fall back slow path
					debug("state->net addr: %p\n", state->net);
                                        debug("sk addr: %p\n", sock_net(sk));
					tcp_parse_options(state->net, skb, &tcp_opt, 0, NULL);
					tcp_opt.rcv_tsecr -= tp->tsoffset;
				}*/
                /*
				if (tcp_opt.saw_tstamp && tcp_opt.rcv_tsecr) {
					debug("rcv_tsecr = %u retrans_stamp = %u tcp_time_stamp(tp) = %u ipid = %x\n",
						tcp_opt.rcv_tsecr,
						tp->retrans_stamp, tcp_time_stamp(tp), ntohs(iph->id));
				}
                */
				if (tcp_opt.saw_tstamp && tcp_opt.rcv_tsecr &&
					!between(tcp_opt.rcv_tsecr, tp->retrans_stamp, tcp_time_stamp(tp))) {
					debug("timestamp error count : %d\n", ts_error_count);
					ts_error_count ++;

					if (ts_error_count >= TIMESTAMP_ERROR_THRESHOLD) {
						debug("TimeStamp error:disable tcp timestamp\n");
						(state->net)->ipv4.sysctl_tcp_timestamps = 0;
						ts_error_count = 0;
					}

				} else {
					if (ts_error_count > 0) {
						ts_error_count --;
					}
				}
			}
		}
	}
}


static int oplus_sla_genl_nlmsg_handle(struct sk_buff *skb,
	struct genl_info *info);
static const struct genl_ops oplus_sla_genl_ops[] = {
	{
		.cmd = OPLUS_SLA_CMD_DOWNLINK,
		.flags = 0,
		.doit = oplus_sla_genl_nlmsg_handle,
		.dumpit = NULL,
	},
};


static struct genl_family oplus_sla_genl_family = {
	.id = 0,
	.hdrsize = 0,
	.name = OPLUS_SLA_FAMILY,
	.version = OPLUS_SLA_FAMILY_VERSION,
	.maxattr = OPLUS_SLA_MSG_MAX,
	.ops = oplus_sla_genl_ops,
	.n_ops = ARRAY_SIZE(oplus_sla_genl_ops),
};

static inline int genl_msg_prepare_usr_msg(u8 cmd, size_t size, pid_t pid,
	struct sk_buff **skbp)
{
	struct sk_buff *skb;
	/* create a new netlink msg */
	skb = genlmsg_new(size, GFP_ATOMIC);

	if (skb == NULL) {
		return -ENOMEM;
	}

	/* Add a new netlink message to an skb */
	genlmsg_put(skb, pid, 0, &oplus_sla_genl_family, 0, cmd);
	*skbp = skb;
	return 0;
}

static inline int genl_msg_mk_usr_msg(struct sk_buff *skb, int type, void *data,
	int len)
{
	int ret;

	/* add a netlink attribute to a socket buffer */
	if ((ret = nla_put(skb, type, len, data)) != 0) {
		return ret;
	}

	return 0;
}

/* send to user space */
static int oplus_sla_send_to_user(int msg_type, char *payload, int payload_len)
{
	int ret = 0;
	void *head;
	struct sk_buff *skbuff;
	size_t size;

	if (!oplus_sla_netlink_pid) {
		debug("oplus_sla_netlink_pid == 0!!\n");
		return -1;
	}

	/*allocate new buffer cache */
	size = nla_total_size(payload_len);
	ret = genl_msg_prepare_usr_msg(OPLUS_SLA_CMD_UPLINK, size,
			oplus_sla_netlink_pid, &skbuff);

	if (ret) {
		return ret;
	}

	ret = genl_msg_mk_usr_msg(skbuff, msg_type, payload, payload_len);

	if (ret) {
		kfree_skb(skbuff);
		return ret;
	}

	head = genlmsg_data(nlmsg_data(nlmsg_hdr(skbuff)));
	genlmsg_end(skbuff, head);

	/* send data */
	ret = genlmsg_unicast(&init_net, skbuff, oplus_sla_netlink_pid);

	if (ret < 0) {
		printk(KERN_ERR
			"oplus_connectivity_sla: oplus_sla_send_to_user, can not unicast skbuff, ret = %d\n", ret);
		return -1;
	}

	return 0;
}

static int oplus_sla_set_android_pid(struct sk_buff *skb)
{
	/*apps_monitor_netlink_pid = NETLINK_CB(skb).portid;*/
	struct nlmsghdr *nlhdr = nlmsg_hdr(skb);
	oplus_sla_netlink_pid = nlhdr->nlmsg_pid;
	debug("oplus_sla_set_android_pid oplus_sla_netlink_pid=%d\n",
		oplus_sla_netlink_pid);
	return 0;
}

static int oplus_sla_iface_changed(struct nlattr *nla)
{
	int index = -1;
	int up = 0;
	char *p;
	struct oplus_dev_info *node = NULL;
	u32 mark = 0x0;

	int *data = (int *)NLA_DATA(nla);
	index = data[0];
	up = data[1];
	p = (char *)(data + 2);
	debug("oplus_sla_iface_changed index:%d, up:%d, ifname:%s\n",
		index, up, p);

	if (index >= 0 && index < IFACE_NUM) {
		if (up) {
			sla_write_lock();
			oplus_sla_info[index].if_up = 0;

			if (index == MAIN_WLAN) {
				mark = MAIN_WLAN_MARK;
				oplus_sla_info[MAIN_WLAN].if_up = 1;

			} else if (index == SECOND_WLAN) {
				mark = SECOND_WLAN_MARK;
				oplus_sla_info[SECOND_WLAN].if_up = 1;

			} else if (index == CELL_INDEX) {
				mark = CELL_MARK;
				oplus_sla_info[CELL_INDEX].if_up = 1;
			}

			if (p) {
				node = &oplus_sla_info[index];
				node->mark = mark;
				memcpy(node->dev_name, p, IFACE_LEN);
				debug("ifname = %s,ifup = %d\n", node->dev_name, node->if_up);
			}

			sla_write_unlock();

		} else {
			sla_write_lock();
			memset(&oplus_sla_info[index], 0x0, sizeof(struct oplus_dev_info));
			sla_write_unlock();
		}
	}

	return 0;
}

static int oplus_sla_set_white_list_app_uid(struct nlattr *nla)
{
	u32 *info = (u32 *)NLA_DATA(nla);
	memset(&white_app_list, 0x0, sizeof(struct oplus_white_app_info));
	white_app_list.count = info[0];

	if (white_app_list.count > 0 && white_app_list.count < WHITE_APP_NUM) {
		int i;

		for (i = 0; i < white_app_list.count; i++) {
			white_app_list.uid[i] = info[i + 1];
			debug("oplus_sla_set_white_list_app_uid count=%d, uid[%d]=%d\n",
				white_app_list.count, i, white_app_list.uid[i]);
		}
	}

	return 0;
}

static int oplus_sla_set_dual_wifi_app_uid(struct nlattr *nla)
{
	u32 *info = (u32 *)NLA_DATA(nla);
	memset(&dual_wifi_app_list, 0x0, sizeof(struct oplus_dual_sta_info));
	dual_wifi_app_list.count = info[0];

	if (dual_wifi_app_list.count > 0
		&& dual_wifi_app_list.count < DUAL_STA_APP_NUM) {
		int i;

		for (i = 0; i < dual_wifi_app_list.count; i++) {
			dual_wifi_app_list.uid[i] = info[i + 1];
			debug("set_dual_wifi_app_uid count=%d, uid[%d]=%d\n",
				dual_wifi_app_list.count, i, dual_wifi_app_list.uid[i]);
		}
	}

	return 0;
}

static int oplus_sla_set_game_app_uid(struct nlattr *nla)
{
	u32 *uidInfo = (u32 *)NLA_DATA(nla);
	u32 index = uidInfo[0];
	u32 uid = uidInfo[1];

	sla_game_write_lock();
	if (index < GAME_NUM) {
		game_info[index].uid = uid;
		game_info[index].game_type = index;
		game_info[index].mark = MAIN_WLAN_MARK;
		debug("oplus_sla_set_game_app_uid:index=%d uid=%d\n", index, uid);
	}
	sla_game_write_unlock();

	return 0;
}

static void init_game_info(void)
{
    int i = 0;

    sla_game_write_lock();
    for(i = 1; i < GAME_NUM; i++){
        game_info[i].mark = MAIN_WLAN_MARK;
    }
    sla_game_write_unlock();
    if(oplus_sla_debug){
        debug("oplus_sla:init_game_online_info\n");
    }
}

static int oplus_sla_change_game_network(struct nlattr *nla)
{
	u32 *data = (u32 *)NLA_DATA(nla);
	u32 game_index = data[0];
	u32 network_index = data[1];
	int game_feed_back[2];

	sla_game_write_lock();
	if (CELL_INDEX == network_index) {
		game_info[game_index].mark = CELL_MARK;

	} else if (WLAN0_INDEX == network_index) {
		game_info[game_index].mark = MAIN_WLAN_MARK;
	}
	sla_game_write_unlock();

	game_feed_back[0] = game_index;
	game_feed_back[1] = network_index;
	oplus_sla_send_to_user(OPLUS_SLA_GAME_NETWORK_CHANGED_EVENT,
		(char *)game_feed_back, sizeof(game_feed_back));
	debug("game_index = %d network_index = %d \n", game_index, network_index);
	return 0;
}

static int oplus_sla_set_video_app_uid(struct nlattr *nla)
{
	u32 *info = (u32 *)NLA_DATA(nla);
	memset(&video_app_list, 0x0, sizeof(struct oplus_dual_sta_info));
	video_app_list.count = info[0];

	if (video_app_list.count > 0 && video_app_list.count < DUAL_STA_APP_NUM) {
		int i;

		for (i = 0; i < video_app_list.count; i++) {
			video_app_list.uid[i] = info[i + 1];
			debug("video_app_list count=%d, uid[%d]=%d\n",
				video_app_list.count, i, video_app_list.uid[i]);
		}
	}

	return 0;
}

static int oplus_sla_enable(struct nlattr *nla)
{
	int *data = (int *)NLA_DATA(nla);
	int enable_type = data[0];

	sla_write_lock();

	oplus_sla_enable_status = 1;
	sla_work_mode = enable_type;

	debug("oplus_sla_enable: enable type = %d\n", enable_type);
	oplus_sla_send_to_user(OPLUS_SLA_ENABLED_EVENT, (char *)&enable_type,
		sizeof(int));

	sla_write_unlock();
	return 0;
}

static int oplus_sla_disable(struct nlattr *nla)
{
	int disable_type = 0;

	int *data = (int *)NLA_DATA(nla);
	disable_type = data[0];

	debug("type[%d] disable,oplus_sla_enable_status[%d],work_mode[%d]\n",
		disable_type, oplus_sla_enable_status, sla_work_mode);

	init_game_info();
	sla_write_lock();

	if (oplus_sla_enable_status && disable_type) {
		if (disable_type == sla_work_mode) {
			oplus_sla_enable_status = 0;
			sla_work_mode = SLA_MODE_INIT;
			debug("type[%d] disabled\n", disable_type);
		}

		dns_network = 0;
		video_app_network = 0;
		oplus_sla_send_to_user(OPLUS_SLA_DISABLED_EVENT, (char *)&disable_type,
			sizeof(int));
	}
	sla_write_unlock();
	return 0;
}

static int oplus_sla_update_weight(struct nlattr *nla)
{
	int *weight = (int *)NLA_DATA(nla);
	sla_write_lock();
	oplus_sla_info[0].weight = weight[0];
	oplus_sla_info[1].weight = weight[1];
	oplus_sla_info[2].weight = weight[2];
	debug("weight: %u:%u:%u\n", oplus_sla_info[0].weight, oplus_sla_info[1].weight,
		oplus_sla_info[2].weight);
	sla_write_unlock();
	return 0;
}

static int oplus_sla_change_default_network(struct nlattr *nla)
{
	oplus_sla_def_net = *(u32 *)NLA_DATA(nla);
	debug("oplus_sla_change_default_network = %d\n", oplus_sla_def_net);
    return 0;
}

static int oplus_sla_set_kernel_debug(struct nlattr *nla)
{
	oplus_sla_debug = *(u32 *)NLA_DATA(nla);
	debug("oplus_sla_set_kernel_debug = %d\n", oplus_sla_debug);
	return 0;
}

static int oplus_sla_change_vpn_state(struct nlattr *nla)
{
	oplus_sla_vpn_connected = *(u32 *)NLA_DATA(nla);
	debug("oplus_sla_vpn_connected = %d\n", oplus_sla_vpn_connected);
	return 0;
}

static int oplus_sla_change_video_app_network(struct nlattr *nla)
{
	video_app_network = *(u32 *)NLA_DATA(nla);
	debug("video_app_network = %d\n", video_app_network);
	return 0;
}

static int oplus_sla_change_dns_network(struct nlattr *nla)
{
	dns_network = *(u32 *)NLA_DATA(nla);
	debug("dns_network = %d\n", dns_network);
	return 0;
}

static int oplus_sla_genl_nlmsg_handle(struct sk_buff *skb,
	struct genl_info *info)
{
	int ret = 0;
	struct nlmsghdr *nlhdr;
	struct genlmsghdr *genlhdr;
	struct nlattr *nla;

	nlhdr = nlmsg_hdr(skb);
	genlhdr = nlmsg_data(nlhdr);
	nla = genlmsg_data(genlhdr);

	if (oplus_sla_debug) {
		debug("oplus_sla_genl_nlmsg_handle, the nla->nla_type = %u, len = %u\n",
			nla->nla_type, nla->nla_len);
	}

	switch (nla->nla_type) {
	case OPLUS_SLA_CMD_SET_ANDROID_PID:
		ret = oplus_sla_set_android_pid(skb);
		break;

	case OPLUS_SLA_CMD_IFACE_CHANGED:
		ret = oplus_sla_iface_changed(nla);
		break;

	case OPLUS_SLA_CMD_ENABLE:
		ret = oplus_sla_enable(nla);
		break;

	case OPLUS_SLA_CMD_DISABLE:
		ret = oplus_sla_disable(nla);
		break;

	case OPLUS_SLA_CMD_SET_WHITE_LIST_APP:
		ret = oplus_sla_set_white_list_app_uid(nla);
		break;

	case OPLUS_SLA_CMD_SET_DUAL_STA_LIST_APP:
		ret = oplus_sla_set_dual_wifi_app_uid(nla);
		break;

	case OPLUS_SLA_CMD_SET_VIDEO_LIST_APP:
		ret = oplus_sla_set_video_app_uid(nla);
		break;

	case OPLUS_SLA_CMD_SET_GAME_LIST_APP:
		ret = oplus_sla_set_game_app_uid(nla);
		break;

	case OPLUS_SLA_CMD_CHANGE_GAME_NETWORK:
		ret = oplus_sla_change_game_network(nla);
		break;

	case OPLUS_SLA_CMD_UPDATE_WEIGHT:
		ret = oplus_sla_update_weight(nla);
		break;

	case OPLUS_SLA_CMD_CHANGE_DEFAULT_NETWORK:
		ret = oplus_sla_change_default_network(nla);
		break;

	case OPLUS_SLA_CMD_SET_KERNEL_DEBUG:
		ret = oplus_sla_set_kernel_debug(nla);
		break;

	case OPLUS_SLA_CMD_CHANGE_VPN_STATE:
		ret = oplus_sla_change_vpn_state(nla);
		break;

	case OPLUS_SLA_CMD_CHANGE_VIDEO_APP_NETWORK:
		ret = oplus_sla_change_video_app_network(nla);
		break;

	case OPLUS_SLA_CMD_CHANGE_DNS_NETWORK:
		ret = oplus_sla_change_dns_network(nla);
		break;

	default:
		break;
	}

	return ret;
}

static unsigned int oplus_sla_input_hook(void *priv,
	struct sk_buff *skb,
	const struct nf_hook_state *state)
{
	handle_timestamp_error_case(skb, state);
	return NF_ACCEPT;
}

static struct nf_hook_ops oplus_sla_ops[] __read_mostly = {
	{
		.hook		= oplus_sla_output_hook,
		.pf		    = NFPROTO_IPV4,
		.hooknum	= NF_INET_LOCAL_OUT,
		//must be here,for  dns packet will do DNAT at mangle table with skb->mark
		.priority	= NF_IP_PRI_CONNTRACK + 1,
	},
	{
		.hook		= oplus_sla_input_hook,
		.pf		    = NFPROTO_IPV4,
		.hooknum	= NF_INET_LOCAL_IN,
		.priority	= NF_IP_PRI_FILTER + 1,
	},
};

static struct ctl_table oplus_sla_sysctl_table[] = {
	{
		.procname	= "oplus_sla_enable",
		.data		= &oplus_sla_enable_status,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec,
	},
	{
		.procname	= "oplus_sla_debug",
		.data		= &oplus_sla_debug,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec,
	},
	{
		.procname	= "oplus_sla_vpn_connected",
		.data		= &oplus_sla_vpn_connected,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec,
	},
	{
		.procname	= "game_mark",
		.data		= &game_mark,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec,
	},
	{ }
};

static int oplus_sla_sysctl_init(void)
{
	oplus_sla_table_hrd = register_net_sysctl(&init_net, "net/oplus_sla",
			oplus_sla_sysctl_table);
	return oplus_sla_table_hrd == NULL ? -ENOMEM : 0;
}


static int oplus_sla_genl_init(void)
{
	int ret;
	ret = genl_register_family(&oplus_sla_genl_family);

	if (ret) {
		debug("genl_register_family:%s error,ret = %d\n", OPLUS_SLA_FAMILY, ret);
		return ret;

	} else {
		debug("genl_register_family complete, id = %d!\n", oplus_sla_genl_family.id);
	}

	return 0;
}

static void oplus_sla_genl_fini(void)
{

	genl_unregister_family(&oplus_sla_genl_family);
}

static int __init oplus_sla_init(void)
{
	int ret = 0;
	rwlock_init(&sla_lock);
	rwlock_init(&sla_game_lock);

	ret = oplus_sla_genl_init();

	if (ret < 0) {
		debug(" module can not init  sla netlink.\n");
	}

	ret |= oplus_sla_sysctl_init();
	ret |= nf_register_net_hooks(&init_net, oplus_sla_ops,
			ARRAY_SIZE(oplus_sla_ops));

	debug(" enter.\n");
	return ret;
}

static void __exit oplus_sla_exit(void)
{
	debug(" exit.\n");
	oplus_sla_genl_fini();

	if (oplus_sla_table_hrd) {
		unregister_net_sysctl_table(oplus_sla_table_hrd);
	}

	nf_unregister_net_hooks(&init_net, oplus_sla_ops, ARRAY_SIZE(oplus_sla_ops));
}

module_init(oplus_sla_init);
module_exit(oplus_sla_exit);
MODULE_LICENSE("GPL v2");
