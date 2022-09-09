/***********************************************************
** Copyright (C), 2008-2019, oplus Mobile Comm Corp., Ltd.
** File: oplus_score.c
** Description: Add for kernel data info send to user space.
**
** Version: 1.0
** Date : 2019/10/02
**
** ------------------ Revision History:------------------------
** <author> <data> <version > <desc>
** penghao 2019/10/02 1.0 build this module
****************************************************************/

#include <linux/types.h>
#include <linux/ip.h>
#include <linux/netfilter.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/icmp.h>
#include <linux/kernel.h>
#include <net/route.h>
#include <net/ip.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/version.h>
#include <net/tcp.h>
#include <linux/random.h>
#include <net/dst.h>
#include <linux/file.h>
#include <net/tcp_states.h>
#include <linux/netlink.h>
#include <net/genetlink.h>
#include <linux/netfilter_ipv4.h>
#include <linux/tcp.h>
#include <net/inet_connection_sock.h>
#include <linux/spinlock.h>
#include <linux/ipv6.h>
#include <net/ipv6.h>
#include <linux/preempt.h>

#define IFNAME_LEN 16
#define IPV4ADDRTOSTR(addr) \
((unsigned char *)&addr)[0], \
((unsigned char *)&addr)[1], \
((unsigned char *)&addr)[2], \
((unsigned char *)&addr)[3]

#define	OPLUS_TRUE	1
#define	OPLUS_FALSE	0
#define REPORT_PERIOD 1
#define SCORE_WINDOW 3
#define OPLUS_UPLINK 1
#define OPLUS_DOWNLINK 0

struct link_score_msg_st
{
	u32 link_index;
	s32 uplink_score;
	s32 downlink_score;
};

struct score_param_st
{
	u32 score_debug;
	u32 threshold_retansmit;
	u32 threshold_normal;
	u32 smooth_factor;
	u32 protect_score;
	u32 threshold_gap;
};

struct uplink_score_info_st{
	u32 link_index;
	u32 uplink_rtt_stamp;
	u32 uplink_retrans_packets;
	u32 uplink_packets;
	s32 uplink_score_save[SCORE_WINDOW];
	u32 uplink_score_index;
	u32 uplink_srtt;
	u32 uplink_rtt_num;
	u32 seq;
	s32 uplink_score_count;
	u32 uplink_nodata_count;
	char ifname[IFNAME_LEN];
};

struct downlink_score_info_st{
	u32 link_index;
	u32 downlink_update_stamp;
	u32 downlink_retrans_packets;
	u32 downlink_packets;
	s32 downlink_score_save[SCORE_WINDOW];
	u32 downlink_score_index;
	u32 downlink_srtt;
	u32 downlink_rtt_num;
	u32 seq;
	s32 downlink_score_count;
	char ifname[IFNAME_LEN];
};

#define MAX_LINK_SCORE 100
#define MAX_LINK_NUM 4
#define FOREGROUND_UID_MAX_NUM 10

static int oplus_score_uplink_num = 0;
static int oplus_score_downlink_num = 0;
static int oplus_score_enable_flag = 1;
static u32 oplus_score_foreground_uid[FOREGROUND_UID_MAX_NUM];
static u32 oplus_score_user_pid = 0;
static spinlock_t uplink_score_lock;
static struct uplink_score_info_st uplink_score_info[MAX_LINK_NUM];
static spinlock_t downlink_score_lock;
static struct downlink_score_info_st downlink_score_info[MAX_LINK_NUM];
static struct score_param_st oplus_score_param_info;
static struct ctl_table_header *oplus_score_table_hrd = NULL;
//static u32 time_gap = 5;  /* sec */
/* static u32 last_report_time = 0; */
static struct timer_list oplus_score_report_timer;
static int oplus_score_debug = 0;
static u32 para_rtt = 8;
static u32 para_rate = 8;
static u32 para_loss = 16;

#define WORST_VIDEO_RTT_THRESH 1000
#define WORST_RTT_THRESH 400
#define WORSE_RTT_THRESH 300
#define NORMAL_RTT_THRESH 200
#define BAD_GAME_RTT_THREAD 170
#define WORST_BASE_SCORE 59
#define WORSE_BASE_SCORE 70
#define NORMAL_BASE_SCORE 80
#define GOOD_BASE_SCORE  100

/*for test*/
static u32 test_foreground_uid = 0;
static u32 test_link_index = 0;
#define PACKET_PER_SEC 240
#define VIDEO_GOOD_RATE 400
#define LOWER_RATE_PACKET  20
#define LOWEST_RATE_PACKET  10
#define SCORE_KEEP 3
#define VALID_RTT_THRESH 10


enum score_msg_type_et{
	OPLUS_SCORE_MSG_UNSPEC,
	OPLUS_SCORE_MSG_ENABLE,
	OPLUS_SCORE_MSG_FOREGROUND_ANDROID_UID,
	OPLUS_SCORE_MSG_REQUEST_SCORE,
	OPLUS_SCORE_MSG_ADD_LINK,
	OPLUS_SCORE_MSG_DEL_LINK,
	OPLUS_SCORE_MSG_CLEAR_LINK,
	OPLUS_SCORE_MSG_CONFIG,
	OPLUS_SCORE_MSG_REPORT_NETWORK_SCORE,
	__OPLUS_SCORE_MSG_MAX,
};
#define OPLUS_SCORE_MSG_MAX (__OPLUS_SCORE_MSG_MAX - 1)

enum score_cmd_type_et{
	OPLUS_SCORE_CMD_UNSPEC,
	OPLUS_SCORE_CMD_DOWNLINK,
	__OPLUS_SCORE_CMD_MAX,
};
#define OPLUS_SCORE_CMD_MAX (__OPLUS_SCORE_CMD_MAX - 1)

#define OPLUS_SCORE_FAMILY_VERSION	1
#define OPLUS_SCORE_FAMILY_NAME "net_score"
#define NLA_DATA(na)		((char *)((char*)(na) + NLA_HDRLEN))
#define GENL_ID_GENERATE	0
static int oplus_score_netlink_rcv_msg(struct sk_buff *skb, struct genl_info *info);
static const struct genl_ops oplus_score_genl_ops[] =
{
	{
		.cmd = OPLUS_SCORE_CMD_DOWNLINK,
		.flags = 0,
		.doit = oplus_score_netlink_rcv_msg,
		.dumpit = NULL,
	},
};

static struct genl_family oplus_score_genl_family =
{
	.id = 0,
	.hdrsize = 0,
	.name = OPLUS_SCORE_FAMILY_NAME,
	.version = OPLUS_SCORE_FAMILY_VERSION,
	.maxattr = OPLUS_SCORE_MSG_MAX,
	.ops = oplus_score_genl_ops,
	.n_ops = ARRAY_SIZE(oplus_score_genl_ops),
};

static int oplus_score_send_netlink_msg(int msg_type, char *payload, int payload_len);
/* score = 100 - rtt * 10 / 500 - 100 * loss_rate * 8*/

static s32 oplus_get_smooth_score(int link, int flag)
{
	int i;
	int j = 0;
	u32 score_sum = 0;
	if (flag) {
		for (i = 0; i < SCORE_WINDOW; i++) {
			if (uplink_score_info[link].uplink_score_save[i] > 0) {
				score_sum += uplink_score_info[link].uplink_score_save[i];
				j++;
			}
		}
	} else {
		for (i = 0; i < SCORE_WINDOW; i++) {
			if (downlink_score_info[link].downlink_score_save[i] > 0) {
				score_sum += downlink_score_info[link].downlink_score_save[i];
				j++;
			}
		}
	}

	if (j == 0) {
		return -1;
	} else {
		return score_sum / j;
	}
}

static int oplus_update_score_count(int link, int score, int flag)
{
	int ret = OPLUS_FALSE;
	if (score == -1) {
		return OPLUS_TRUE;
	}

	if (flag) {
		if(score > WORSE_BASE_SCORE) {
			uplink_score_info[link].uplink_score_count++;
			if (uplink_score_info[link].uplink_score_count > SCORE_KEEP) {
				uplink_score_info[link].uplink_score_count = SCORE_KEEP;
			}
		}
		else if (score <= WORST_BASE_SCORE && score >= 0) {
			uplink_score_info[link].uplink_score_count--;
			if (uplink_score_info[link].uplink_score_count < -SCORE_KEEP) {
				uplink_score_info[link].uplink_score_count = -SCORE_KEEP;
			}
		}

		if (((score > WORST_BASE_SCORE) && (uplink_score_info[link].uplink_score_count >= 0)) ||
			((score <= WORST_BASE_SCORE) && (uplink_score_info[link].uplink_score_count <= 0))) {
			ret = OPLUS_TRUE;
		}

		if (oplus_score_debug) {
			printk("[oplus_score]:uplink_report=%d,score=%d,score_count=%d\n",
					ret, score, uplink_score_info[link].uplink_score_count);
		}
	} else {
		if(score > WORSE_BASE_SCORE) {
			downlink_score_info[link].downlink_score_count++;
			if (downlink_score_info[link].downlink_score_count > SCORE_KEEP) {
				downlink_score_info[link].downlink_score_count = SCORE_KEEP;
			}
		}
		else if (score <= WORST_BASE_SCORE && score >= 0) {
			downlink_score_info[link].downlink_score_count--;
			if (downlink_score_info[link].downlink_score_count < -SCORE_KEEP) {
				downlink_score_info[link].downlink_score_count = -SCORE_KEEP;
			}
		}

		if (((score <= WORST_BASE_SCORE) && (downlink_score_info[link].downlink_score_count <= 0)) ||
			((score > WORST_BASE_SCORE) && (downlink_score_info[link].downlink_score_count >= 0))) {
			ret = OPLUS_TRUE;
		}

		if (oplus_score_debug) {
			printk("[oplus_score]:downlink_report=%d,score=%d,score_count=%d\n",
					ret, score, downlink_score_info[link].downlink_score_count);
			}
	}

	return ret;
}

void oplus_score_calc_and_report(void)
{
	struct link_score_msg_st link_score_msg;
	int i;
	u32 uplink_index, downlink_index;
	u32 uplink_packets, uplink_retrans_packets, uplink_srtt;
	u32 downlink_packets, downlink_retrans_packets, downlink_srtt;
	s32 uplink_score, downlink_score;
	u32 uplink_seq, downlink_seq;
	u32 retrans_rate = 0;
	s32 uplink_smooth_score, downlink_smooth_score;
	u32 index = 0;
	int uplink_report, downlink_report;
	char ifname[IFNAME_LEN];
	u32 uplink_nodata_count;
	int downlink_rate = 0;
	u32 uplink_total_packets = 0;
	u32 downlink_total_packets = 0;

	/* printk("[oplus_score]:enter oplus_score_calc_and_report,jiffies=%llu\n", jiffies);*/
	for (i = 0; i < MAX_LINK_NUM; i++) {
		uplink_smooth_score = -1;
		downlink_smooth_score = -1;
		uplink_report = 0;
		downlink_report = 0;
		spin_lock_bh(&downlink_score_lock);
		if (downlink_score_info[i].link_index == 0) {
			spin_unlock_bh(&downlink_score_lock);
			continue;
		}
		downlink_index = downlink_score_info[i].link_index;
		downlink_packets = downlink_score_info[i].downlink_packets;
		downlink_retrans_packets = downlink_score_info[i].downlink_retrans_packets;
		downlink_total_packets = downlink_packets + downlink_retrans_packets;
		downlink_srtt = downlink_score_info[i].downlink_srtt;
		downlink_seq = downlink_score_info[i].seq;
		downlink_score_info[i].downlink_packets = 0;
		downlink_score_info[i].downlink_retrans_packets = 0;
		/*downlink_score_info[i].downlink_srtt = 0;*/
		downlink_score_info[i].downlink_rtt_num = 1;
		spin_unlock_bh(&downlink_score_lock);

		spin_lock_bh(&uplink_score_lock);
		if (uplink_score_info[i].link_index == 0) {
			spin_unlock_bh(&uplink_score_lock);
			continue;
		}
		uplink_index = uplink_score_info[i].link_index;
		if (uplink_index != downlink_index) {
			printk("[oplus_score]:link error:uplink_index=%u,downlink_index=%u\n",
				uplink_index, downlink_index);
			continue;
		}
		memcpy((void*)ifname, (void*)uplink_score_info[i].ifname, IFNAME_LEN);
		uplink_packets = uplink_score_info[i].uplink_packets;
		uplink_retrans_packets = uplink_score_info[i].uplink_retrans_packets;
		uplink_total_packets = uplink_packets + uplink_retrans_packets;
		uplink_srtt = uplink_score_info[i].uplink_srtt;
		uplink_seq = uplink_score_info[i].seq;
		uplink_nodata_count = uplink_score_info[i].uplink_nodata_count;
		uplink_score_info[i].uplink_packets = 0;
		uplink_score_info[i].uplink_retrans_packets = 0;
		/*uplink_score_info[i].uplink_srtt = 0;*/
		if (uplink_score_info[i].uplink_rtt_num) {
			uplink_score_info[i].uplink_rtt_num = 1;
		}

		if (uplink_total_packets == 0) {
			if (oplus_score_debug) {
				printk("[oplus_score]:uplink no_data\n");
			}
			uplink_score = -1;
			uplink_score_info[i].uplink_nodata_count++;
		} else {
			uplink_score_info[i].uplink_nodata_count = 0;
			retrans_rate = 100 * uplink_retrans_packets / uplink_total_packets;
			if (uplink_packets > ((para_rate * PACKET_PER_SEC) >> 3)) {
				uplink_score = (s32)(GOOD_BASE_SCORE - (retrans_rate * para_loss) / 8);
			} else if (uplink_srtt > BAD_GAME_RTT_THREAD && (uplink_total_packets < 10) && (downlink_total_packets < 10)) {
				uplink_score = (s32)(WORST_BASE_SCORE - (WORST_BASE_SCORE * retrans_rate * para_loss) / 800);
			} else {
				if (uplink_srtt > WORST_VIDEO_RTT_THRESH) {
					uplink_score = (s32)(WORST_BASE_SCORE - (WORST_BASE_SCORE * retrans_rate * para_loss) / 800);
				} else if (uplink_srtt > WORST_RTT_THRESH) {
					downlink_rate = downlink_packets * 1000 / uplink_srtt;
					if (downlink_rate == 0) {
						uplink_score = -1;
					} else if (downlink_rate > VIDEO_GOOD_RATE) {
						uplink_score = (s32)(GOOD_BASE_SCORE - (GOOD_BASE_SCORE * retrans_rate * para_loss) / 800);
					} else if (downlink_rate > PACKET_PER_SEC) {
						uplink_score = (s32)(NORMAL_BASE_SCORE - (NORMAL_BASE_SCORE * retrans_rate * para_loss) / 800);
					} else {
						uplink_score = (s32)(WORST_BASE_SCORE - (WORST_BASE_SCORE * retrans_rate * para_loss) / 800);
					}
				} else if (uplink_srtt > WORSE_RTT_THRESH) {
					uplink_score = (s32)(WORSE_BASE_SCORE - (WORSE_BASE_SCORE * retrans_rate * para_loss) / 800);
				} else if (uplink_srtt > NORMAL_RTT_THRESH) {
					uplink_score = (s32)(NORMAL_BASE_SCORE - (NORMAL_BASE_SCORE * retrans_rate * para_loss) / 800);
				} else {
					uplink_score = (s32)(GOOD_BASE_SCORE - (retrans_rate * para_loss) / 8);
				}
			}
			if (uplink_score <= 0) {
				uplink_score = 1;
			}
		}

		uplink_report = oplus_update_score_count(i, uplink_score, OPLUS_UPLINK);
		if (uplink_report) {
			if (uplink_score != -1) {
				index = uplink_score_info[i].uplink_score_index++ % SCORE_WINDOW;
				uplink_score_info[i].uplink_score_save[index] = uplink_score;
			}
		}
		uplink_smooth_score = oplus_get_smooth_score(i, OPLUS_UPLINK);
		if (uplink_smooth_score == -1) {
			uplink_report = 0;
		}
		spin_unlock_bh(&uplink_score_lock);

		if (oplus_score_debug) {
			printk("[oplus_score]:up_score:link=%u,if=%s,up_pack=%u,up_retran=%u,up_rtt=%u,score=%d,s_score=%d,seq=%u,uid=%u,retrans_rate=%u,index=%u,nodata=%u\n",
				uplink_index, ifname, uplink_packets, uplink_retrans_packets, uplink_srtt, uplink_score,
				uplink_smooth_score, uplink_seq, oplus_score_foreground_uid[0], retrans_rate, index, uplink_nodata_count);
		}

		/*start downlink score calc*/
		if (downlink_packets + downlink_retrans_packets == 0) {
			if (oplus_score_debug) {
				printk("[oplus_score]:downlink no_data,if=%s\n", ifname);
			}
			downlink_score = -1;
		} else {
			retrans_rate = 100 * downlink_retrans_packets / downlink_total_packets;
			if (downlink_packets > ((para_rate * PACKET_PER_SEC) >> 3)) {
				downlink_score = (s32)(GOOD_BASE_SCORE - retrans_rate * para_loss / 4);
			} else {
				if (uplink_srtt > WORST_RTT_THRESH) {
					downlink_score = (s32)(WORST_BASE_SCORE - (WORST_BASE_SCORE * retrans_rate * para_loss) / 800);
				} else {
					downlink_score = (s32)(100 - (2 * downlink_retrans_packets * uplink_srtt) / 10);
				}
			}

			if (downlink_score < 0) {
				downlink_score = 1;
			}
		}

		if ((downlink_total_packets < 15) && (downlink_score > WORST_BASE_SCORE)) {
			downlink_score = -1;
		}

		spin_lock_bh(&downlink_score_lock);
		downlink_report = oplus_update_score_count(i, downlink_score, OPLUS_DOWNLINK);
		if (downlink_report) {
			if (downlink_score != -1) {
				index = downlink_score_info[i].downlink_score_index++ % SCORE_WINDOW;
				downlink_score_info[i].downlink_score_save[index] = downlink_score;
			}
		}
		downlink_smooth_score = oplus_get_smooth_score(i, OPLUS_DOWNLINK);
		if (downlink_smooth_score == -1) {
				downlink_report = 0;
		}

		spin_unlock_bh(&downlink_score_lock);

		if (oplus_score_debug) {
			printk("[oplus_score]:down_score:link=%u,if=%s,down_pack=%u,down_retran=%u,down_rtt=%u,score=%d,s_score=%d,seq=%u,uid=%u,retrans_rate=%u,index=%u\n",
				downlink_index, ifname, downlink_packets, downlink_retrans_packets, downlink_srtt, downlink_score,
				downlink_smooth_score, downlink_seq, oplus_score_foreground_uid[0], retrans_rate, index);
		}

		if (uplink_report || downlink_report) {
			link_score_msg.link_index = uplink_index;
			if (uplink_smooth_score == -1) {
				link_score_msg.uplink_score = downlink_smooth_score;
			} else {
				link_score_msg.uplink_score = uplink_smooth_score;
			}

			if (downlink_smooth_score == -1) {
				link_score_msg.downlink_score = uplink_smooth_score;
			} else {
				link_score_msg.downlink_score = downlink_smooth_score;
			}

			oplus_score_send_netlink_msg(OPLUS_SCORE_MSG_REPORT_NETWORK_SCORE, (char *)&link_score_msg, sizeof(link_score_msg));
			if (oplus_score_debug) {
				printk("[oplus_score]:report_score1:link=%u,if=%s,up_score=%d,down_score=%d,uid=%u,ul_p=%d,dl_p=%d\n",
					uplink_index, ifname, link_score_msg.uplink_score, link_score_msg.downlink_score,
					oplus_score_foreground_uid[0], uplink_report, downlink_report);
			}
		}

		if (oplus_score_debug) {
				printk("[oplus_score]:report_score_all:link=%u,uplink_score=%d,us_score=%d,downlink_score=%d,ds_score=%d,uid=%u,ul_p=%d,dl_p=%d\n",
					uplink_index, uplink_score, uplink_smooth_score, downlink_score,
					downlink_smooth_score, oplus_score_foreground_uid[0], uplink_report, downlink_report);
		}
	}

	return;
}

static void oplus_score_report_timer_function(struct timer_list *t)
{
	oplus_score_calc_and_report();
	mod_timer(&oplus_score_report_timer, jiffies + REPORT_PERIOD * HZ);
}

static void oplus_score_report_timer_init(void)
{
	printk("[oplus_score]:report_timer_init\n");
	timer_setup(&oplus_score_report_timer, oplus_score_report_timer_function, 0);
}

static void oplus_score_report_timer_start(void)
{
	printk("[oplus_score]:report_timer_start\n");
	/*oplus_score_report_timer.function = (void *)oplus_score_report_timer_function;*/
	oplus_score_report_timer.expires = jiffies + REPORT_PERIOD * HZ;
	mod_timer(&oplus_score_report_timer, oplus_score_report_timer.expires);
}

static void oplus_score_report_timer_del(void)
{
	printk("[oplus_score]:report_timer_del\n");
	del_timer_sync(&oplus_score_report_timer);
}

static inline int genl_msg_prepare_usr_msg(u8 cmd, size_t size, pid_t pid, struct sk_buff **skbp)
{
	struct sk_buff *skb;
	/* create a new netlink msg */
	skb = genlmsg_new(size, GFP_ATOMIC);
	if (skb == NULL) {
		return -ENOMEM;
	}

	/* Add a new netlink message to an skb */
	genlmsg_put(skb, pid, 0, &oplus_score_genl_family, 0, cmd);
	*skbp = skb;
	return 0;
}

static inline int genl_msg_mk_usr_msg(struct sk_buff *skb, int type, void *data, int len)
{
	int ret;
	/* add a netlink attribute to a socket buffer */
	if ((ret = nla_put(skb, type, len, data)) != 0) {
		return ret;
	}

	return 0;
}

static inline int uplink_get_array_index_by_link_index(int link_index)
{
	int array_index = -1;
	int i;

	for (i = 0; i < MAX_LINK_NUM; i++) {
		if (uplink_score_info[i].link_index == link_index) {
			return i;
		}
	}

	return array_index;
}

static inline int downlink_get_array_index_by_link_index(int link_index)
{
	int array_index = -1;
	int i;

	for (i = 0; i < MAX_LINK_NUM; i++) {
		if (downlink_score_info[i].link_index == link_index) {
			return i;
		}
	}

	return array_index;
}

static inline int is_foreground_uid(int uid)
{
	int i;
	for (i = 0; i < FOREGROUND_UID_MAX_NUM; i++) {
		if (uid == oplus_score_foreground_uid[i]) {
			return OPLUS_TRUE;
		}
	}

	return OPLUS_FALSE;
}

/* send to user space */
static int oplus_score_send_netlink_msg(int msg_type, char *payload, int payload_len)
{
	int ret = 0;
	void * head;
	struct sk_buff *skbuff;
	size_t size;

	if (!oplus_score_user_pid) {
		printk("[oplus_score]: oplus_score_send_netlink_msg,oplus_score_user_pid=0\n");
		return -1;
	}

	/* allocate new buffer cache */
	size = nla_total_size(payload_len);
	ret = genl_msg_prepare_usr_msg(OPLUS_SCORE_CMD_DOWNLINK, size, oplus_score_user_pid, &skbuff);
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
	ret = genlmsg_unicast(&init_net, skbuff, oplus_score_user_pid);
	if(ret < 0) {
		printk("[oplus_score]:oplus_score_send_netlink_msg error, ret = %d\n", ret);
		return -1;
	}

	return 0;
}

static void oplus_score_uplink_stat(struct sk_buff *skb)
{
	int link_index;
	int i;
	struct sock *sk;
	struct tcp_sock *tp;
	struct tcphdr *tcph = tcp_hdr(skb);
	struct inet_connection_sock *icsk;
	u32 srtt;

	sk = skb_to_full_sk(skb);
	tp = tcp_sk(sk);
	icsk = inet_csk(sk);
	link_index = skb->dev->ifindex;

	spin_lock_bh(&uplink_score_lock);
	i = uplink_get_array_index_by_link_index(link_index);
	if (i < 0) {
		if (oplus_score_debug) {
			printk("[oplus_score]:upskb_linkerr,link=%d,if=%s,seq=%u,ack=%u,sport=%u,dport=%u,uid=%u\n",
				link_index, skb->dev->name, ntohl(tcph->seq), ntohl(tcph->ack_seq), ntohs(tcph->source),
				ntohs(tcph->dest), (u32)(sk->sk_uid.val));
		}
		spin_unlock_bh(&uplink_score_lock);
		return;
	}

	if (icsk->icsk_ca_state >= TCP_CA_Recovery && tp->high_seq !=0 && before(ntohl(tcph->seq), tp->high_seq)) {
		uplink_score_info[i].uplink_retrans_packets++;
	} else {
		uplink_score_info[i].uplink_packets++;
	}
	if (oplus_score_debug) {
		printk("[oplus_score]:uplink=%d,if=%s,seq=%u,high_seq=%u,uplink_retran=%u,npacket=%u,uid=%u,sport=%u,dport=%u,state=%d,len=%u\n",
				link_index, skb->dev->name, ntohl(tcph->seq), tp->high_seq, uplink_score_info[i].uplink_retrans_packets,
				uplink_score_info[i].uplink_packets, (u32)(sk->sk_uid.val), ntohs(tcph->source),
				ntohs(tcph->dest), sk->sk_state, skb->len);
	}

	uplink_score_info[i].seq = ntohl(tcph->seq);
	srtt = (tp->srtt_us >> 3) / 1000;
	if (oplus_score_debug) {
		printk("[oplus_score]:rtt_sample_up:rtt=%u,rtt_num=%u,uid=%u\n",
			srtt, uplink_score_info[i].uplink_rtt_num, (u32)(sk->sk_uid.val));
	}

	if (srtt > VALID_RTT_THRESH) {
		uplink_score_info[i].uplink_rtt_num++;
		if (uplink_score_info[i].uplink_rtt_num != 0) {
			uplink_score_info[i].uplink_srtt =
				(uplink_score_info[i].uplink_srtt * (uplink_score_info[i].uplink_rtt_num -1) + srtt) / uplink_score_info[i].uplink_rtt_num;
				uplink_score_info[i].uplink_rtt_stamp = (u32)jiffies;
		}
	}
	spin_unlock_bh(&uplink_score_lock);

	return;
}


static int is_downlink_retrans_pack(u32 skb_seq, struct sock *sk)
{
	struct tcp_sock *tp = (struct tcp_sock*)sk;
	struct inet_connection_sock *icsk = inet_csk(sk);
	u32 now = (u32)jiffies;

	if((skb_seq == tp->rcv_nxt) && (!RB_EMPTY_ROOT(&tp->out_of_order_queue))) {
		int m = (int)(now - icsk->icsk_ack.lrcvtime) * 1000 / HZ;
		int half_rtt = (tp->srtt_us / 8000) >> 1;
		if ((tp->srtt_us != 0) && (m > 50)) {
			if (oplus_score_debug) {
				printk("[oplus_score]:now=%u,lrcttime=%u,half_rtt=%d,m=%d,Hz=%u,rtt=%u,seq=%u\n",
					now, icsk->icsk_ack.lrcvtime, half_rtt, m, HZ, tp->srtt_us, skb_seq);
			}
			return OPLUS_TRUE;
		}
	}

	return OPLUS_FALSE;
}

static void oplus_score_downlink_stat(struct sk_buff *skb)
{
	int link_index;
	int i;
	struct sock *sk;
	struct tcp_sock *tp;
	struct tcphdr *tcph = tcp_hdr(skb);
	struct inet_connection_sock *icsk;
	u32 srtt;

	sk = skb_to_full_sk(skb);
	tp = tcp_sk(sk);
	icsk = inet_csk(sk);
	link_index = skb->dev->ifindex;

	spin_lock_bh(&downlink_score_lock);
	i = downlink_get_array_index_by_link_index(link_index);
	if (i < 0) {
		if (oplus_score_debug) {
			printk("[oplus_score]:downskb_linkerr,link=%d,if=%s,seq=%u,ack=%u,sport=%u,dport=%u,uid=%u\n",
				link_index, skb->dev->name, ntohl(tcph->seq), ntohl(tcph->ack_seq), ntohs(tcph->source),
				ntohs(tcph->dest), (u32)(sk->sk_uid.val));
		}
		spin_unlock_bh(&downlink_score_lock);
		return;
	}

	if ((sk->sk_state != TCP_SYN_SENT) && (is_downlink_retrans_pack(ntohl(tcph->seq), sk))) {
		downlink_score_info[i].downlink_retrans_packets++;
	} else {
		downlink_score_info[i].downlink_packets++;
	}
	if (oplus_score_debug) {
		printk("[oplus_score]:downlink=%d,if=%s,seq=%u,rcv_nxt=%u,downlink_retran=%u,npacket=%u,uid=%u,sport=%u,dport=%u,state=%d,len=%u\n",
				link_index, skb->dev->name, ntohl(tcph->seq), tp->rcv_nxt, downlink_score_info[i].downlink_retrans_packets,
				downlink_score_info[i].downlink_packets, (u32)(sk->sk_uid.val), ntohs(tcph->source),
				ntohs(tcph->dest), sk->sk_state, skb->len);
	}

	downlink_score_info[i].seq = ntohl(tcph->seq);
	srtt = (tp->rcv_rtt_est.rtt_us >> 3) / 1000;
	if (oplus_score_debug) {
		printk("[oplus_score]:rtt_sample_down:rtt=%u,rtt_num=%u,uid=%u\n",
			srtt, downlink_score_info[i].downlink_rtt_num, (u32)(sk->sk_uid.val));
	}

	if (srtt) {
		downlink_score_info[i].downlink_rtt_num++;
		if (downlink_score_info[i].downlink_rtt_num != 0) {
			downlink_score_info[i].downlink_srtt =
				(downlink_score_info[i].downlink_srtt * (downlink_score_info[i].downlink_rtt_num -1) + srtt) / downlink_score_info[i].downlink_rtt_num;
		}
		downlink_score_info[i].downlink_update_stamp = (u32)jiffies;
	}
	spin_unlock_bh(&downlink_score_lock);

	return;
}

static uid_t get_uid_from_sock(const struct sock *sk)
{
	uid_t sk_uid;
	#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
	const struct file *filp = NULL;
	#endif
	if (NULL == sk || !sk_fullsock(sk) || NULL == sk->sk_socket) {
		return 0;
	}
	#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
	filp = sk->sk_socket->file;
	if (NULL == filp) {
		return 0;
	}
	sk_uid = __kuid_val(filp->f_cred->fsuid);
	#else
	sk_uid = __kuid_val(sk->sk_uid);
	#endif
	return sk_uid;
}

static inline int skb_v4_check(struct sk_buff *skb)
{
	uid_t sk_uid;
	struct sock *sk;
	struct iphdr *iph = NULL;
	struct tcphdr *tcph = NULL;
	struct net_device *dev;

	iph = ip_hdr(skb);
	tcph = tcp_hdr(skb);
	if (skb->protocol != htons(ETH_P_IP) || (!iph))
		return OPLUS_FALSE;

	if (iph->protocol != IPPROTO_TCP || !tcph)
		return OPLUS_FALSE;

	sk = skb_to_full_sk(skb);
	if (!sk) {
		return OPLUS_FALSE;
	}

	if (sk->sk_state > TCP_SYN_SENT) {
		return OPLUS_FALSE;
	}

	/* skb is pure ack*/
	if ((ntohs(iph ->tot_len) == (iph->ihl + tcph->doff) * 4) && (!tcph->syn || !tcph->fin))
		return OPLUS_FALSE;

	sk_uid = get_uid_from_sock(sk);
	if(sk_uid == 0) {
		return OPLUS_FALSE;
	}

	/* check uid is foreground*/
	if (!is_foreground_uid(sk_uid))
		return OPLUS_FALSE;

	dev = skb->dev;
	if (!dev) {
		return OPLUS_FALSE;
	}

	return OPLUS_TRUE;
}

static inline int skb_v6_check(struct sk_buff *skb)
{
	uid_t sk_uid;
	struct sock *sk;
	struct tcphdr *tcph = NULL;
	struct ipv6hdr *ipv6h = NULL;
	struct net_device *dev;

	ipv6h = ipv6_hdr(skb);
	tcph = tcp_hdr(skb);
	if (skb->protocol != htons(ETH_P_IPV6) || (!ipv6h))
		return OPLUS_FALSE;

	if ((ipv6h->nexthdr != NEXTHDR_TCP) || (!tcph))
		return OPLUS_FALSE;

	sk = skb_to_full_sk(skb);
	if (!sk) {
		return OPLUS_FALSE;
	}

	if (sk->sk_state > TCP_SYN_SENT) {
		return OPLUS_FALSE;
	}

	/* skb is pure ack*/
	if ((ntohs(ipv6h ->payload_len) == tcph->doff * 4) && (!tcph->syn || !tcph->fin))
		return OPLUS_FALSE;

	/* check uid is foreground*/
	sk_uid = get_uid_from_sock(sk);
	if (!is_foreground_uid(sk_uid))
		return OPLUS_FALSE;

	dev = skb->dev;
	if (!dev) {
		return OPLUS_FALSE;
	}

	return OPLUS_TRUE;
}

static unsigned int oplus_score_postrouting_hook4(void *priv, struct sk_buff *skb, const struct nf_hook_state *state)
{
	if (!oplus_score_enable_flag)
		return NF_ACCEPT;

	if (skb_v4_check(skb) == OPLUS_FALSE) {
		return NF_ACCEPT;
	}

	oplus_score_uplink_stat(skb);

	return NF_ACCEPT;
}

static unsigned int oplus_score_postrouting_hook6(void *priv, struct sk_buff *skb, const struct nf_hook_state *state)
{
	if (!oplus_score_enable_flag)
		return NF_ACCEPT;

	if (skb_v6_check(skb) == OPLUS_FALSE) {
		return NF_ACCEPT;
	}

	oplus_score_uplink_stat(skb);

	return NF_ACCEPT;
}

static unsigned int oplus_score_input_hook4(void *priv, struct sk_buff *skb, const struct nf_hook_state *state)
{
	if (!oplus_score_enable_flag) {
		return NF_ACCEPT;
	}

	if (skb_v4_check(skb) == OPLUS_FALSE) {
		return NF_ACCEPT;
	}

	oplus_score_downlink_stat(skb);

	return NF_ACCEPT;
}

static unsigned int oplus_score_input_hook6(void *priv, struct sk_buff *skb, const struct nf_hook_state *state)
{
	if (!oplus_score_enable_flag)
		return NF_ACCEPT;

	if (skb_v6_check(skb) == OPLUS_FALSE)
		return NF_ACCEPT;

	oplus_score_downlink_stat(skb);

	return NF_ACCEPT;
}

static struct nf_hook_ops oplus_score_netfilter_ops[] __read_mostly =
{
	{
		.hook		= oplus_score_postrouting_hook4,
		.pf			= NFPROTO_IPV4,
		.hooknum	= NF_INET_POST_ROUTING,
		.priority	= NF_IP_PRI_FILTER + 1,
	},
	{
		.hook		= oplus_score_input_hook4,
		.pf			= NFPROTO_IPV4,
		.hooknum	= NF_INET_LOCAL_IN,
		.priority	= NF_IP_PRI_FILTER + 1,
	},
		{
		.hook		= oplus_score_postrouting_hook6,
		.pf			= NFPROTO_IPV6,
		.hooknum	= NF_INET_POST_ROUTING,
		.priority	= NF_IP_PRI_FILTER + 1,
	},
	{
		.hook		= oplus_score_input_hook6,
		.pf			= NFPROTO_IPV6,
		.hooknum	= NF_INET_LOCAL_IN,
		.priority	= NF_IP_PRI_FILTER + 1,
	},
};

static void oplus_score_enable(struct nlattr *nla)
{
	u32 *data = (u32*)NLA_DATA(nla);
	oplus_score_enable_flag = data[0];
	printk("[oplus_score]:oplus_score_enable_flag = %u", oplus_score_enable_flag);
	return;
}

static void oplus_score_set_foreground_uid(struct nlattr *nla)
{
	u32 *data;
	int i, num;

	data = (u32 *)NLA_DATA(nla);
	num = data[0];
	if (num <= 0 || num > FOREGROUND_UID_MAX_NUM) {
		printk("[oplus_score]: foreground uid num out of range, num = %d", num);
		return;
	}

	memset(oplus_score_foreground_uid, 0, sizeof(oplus_score_foreground_uid));
	for (i = 0; i < num; i++) {
		oplus_score_foreground_uid[i] = data[i + 1];
		printk("[oplus_score]: add uid, num = %d, index = %d, uid=%u\n", num, i, data[i + 1]);
	}

	/* forground uid change, so reset score static */
	spin_lock_bh(&uplink_score_lock);
	for (i = 0; i < MAX_LINK_NUM; i++) {
		uplink_score_info[i].uplink_retrans_packets = 0;
		uplink_score_info[i].uplink_packets = 0;
		uplink_score_info[i].uplink_nodata_count = 0;
		/*uplink_score_info[i].uplink_score = MAX_LINK_SCORE;*/
		if (uplink_score_info[i].uplink_rtt_num) {
			uplink_score_info[i].uplink_rtt_num = 1;
		}
	}
	spin_unlock_bh(&uplink_score_lock);

	spin_lock_bh(&downlink_score_lock);
	for (i = 0; i < MAX_LINK_NUM; i++) {
		downlink_score_info[i].downlink_retrans_packets = 0;
		downlink_score_info[i].downlink_packets = 0;
		/*downlink_score_info[i].downlink_score = MAX_LINK_SCORE;*/
	}
	spin_unlock_bh(&downlink_score_lock);

	return;
}

static void oplus_score_request_score(struct nlattr *nla)
{
	u32 *data;
	u32 link_index;
	int i;

	struct link_score_msg_st link_score_msg;
	data = (u32 *)NLA_DATA(nla);
	link_index = data[0];

	spin_lock_bh(&uplink_score_lock);
	i = uplink_get_array_index_by_link_index(link_index);
	if (i < 0) {
		/* printk("[oplus_score]:uplink get index falure!\n"); */
		spin_unlock_bh(&uplink_score_lock);
		return;
	}
	link_score_msg.uplink_score = oplus_get_smooth_score(link_index, OPLUS_UPLINK);
	spin_unlock_bh(&uplink_score_lock);

	spin_lock_bh(&downlink_score_lock);
	i = downlink_get_array_index_by_link_index(link_index);
	if (i < 0) {
		/* printk("[oplus_score]:downlink get index falure!\n");*/
		spin_unlock_bh(&downlink_score_lock);
		return;
	}
	link_score_msg.downlink_score = oplus_get_smooth_score(link_index, OPLUS_DOWNLINK);
	spin_unlock_bh(&downlink_score_lock);

	link_score_msg.link_index = link_index;
	printk("[oplus_score]:request_report:link=%d,uscore=%d,dscore=%d\n", link_index, link_score_msg.uplink_score, link_score_msg.downlink_score);
	oplus_score_send_netlink_msg(OPLUS_SCORE_MSG_REPORT_NETWORK_SCORE, (char *)&link_score_msg, sizeof(link_score_msg));
	return;
}

static void oplus_score_add_uplink(u32 link_index, struct net_device *dev)
{
	int i, j;
	if (oplus_score_uplink_num == MAX_LINK_NUM) {
		printk("[oplus_score]:error, uplink num reach max.\n");
		return;
	}

	for (i = 0; i < MAX_LINK_NUM; i++) {
		if (uplink_score_info[i].link_index == link_index) {
			printk("[oplus_score]:warning,uplink already exist,index = %u.\n", link_index);
			return;
		}
	}

	for (i =0; i < MAX_LINK_NUM; i++) {
		if (uplink_score_info[i].link_index != 0)
			continue;

		uplink_score_info[i].link_index = link_index;
		memcpy((void*)uplink_score_info[i].ifname, (void*)dev->name, IFNAME_LEN);
		uplink_score_info[i].uplink_retrans_packets = 0;
		uplink_score_info[i].uplink_packets = 0;
		uplink_score_info[i].uplink_nodata_count = 0;
		for (j = 0; j < SCORE_WINDOW; j++) {
			uplink_score_info[i].uplink_score_save[j] = -1;
		}
		uplink_score_info[i].uplink_score_index = 0;
		oplus_score_uplink_num++;
		printk("[oplus_score]:up:add_link finish,link_index=%u,i=%d,link_num=%u", link_index, i, oplus_score_uplink_num);
		break;
	}

	return;
}

static void oplus_score_add_downlink(u32 link_index, struct net_device *dev)
{
	int i, j;
	if (oplus_score_downlink_num == MAX_LINK_NUM) {
		printk("[oplus_score]:error, downlink num reach max.\n");
		return;
	}

	for (i = 0; i < MAX_LINK_NUM; i++) {
		if (downlink_score_info[i].link_index == link_index) {
			printk("[oplus_score]:warning,downlink already exist,index = %u.\n", link_index);
			return;
		}
	}

	for (i =0; i < MAX_LINK_NUM; i++) {
		if (downlink_score_info[i].link_index != 0)
			continue;

		downlink_score_info[i].link_index = link_index;
		memcpy((void*)uplink_score_info[i].ifname, (void*)dev->name, IFNAME_LEN);
		downlink_score_info[i].downlink_retrans_packets = 0;
		downlink_score_info[i].downlink_packets = 0;
		for (j = 0; j < SCORE_WINDOW; j++) {
			downlink_score_info[i].downlink_score_save[j] = -1;
		}
		uplink_score_info[i].uplink_score_index = 0;
		oplus_score_downlink_num++;
		printk("[oplus_score]:down:add_link finish,link_index=%u,i=%d,link_num=%u", link_index, i, oplus_score_downlink_num);
		break;
	}
}

static void oplus_score_add_link(struct nlattr *nla)
{
	u32 *data;
	u32 link_index;
	struct net_device *dev;

	data = (u32 *)NLA_DATA(nla);
	link_index = data[0];

	if (link_index == 0) {
		printk("[oplus_score]:error, link index is 0!\n");
		return;
	}

	dev = dev_get_by_index(&init_net, link_index);
	if(!dev) {
		printk("[oplus_score]:dev is null,index=%d\n", link_index);
		return;
	}

	printk("[oplus_score]:to add_link index=%u,dev_name=%s, uplink_num=%d,downlink_num=%d!\n",
			link_index, dev->name, oplus_score_uplink_num, oplus_score_downlink_num);

	spin_lock_bh(&uplink_score_lock);
	oplus_score_add_uplink(link_index, dev);
	spin_unlock_bh(&uplink_score_lock);

	spin_lock_bh(&downlink_score_lock);
	oplus_score_add_downlink(link_index, dev);
	spin_unlock_bh(&downlink_score_lock);

	dev_put(dev);

	return;
}

static void oplus_score_del_link(struct nlattr *nla)
{
	u32 *data;
	u32 i, j, link_index;

	data = (u32 *)NLA_DATA(nla);
	link_index = data[0];

	printk("[oplus_score]:to del_link index=%u,uplink_num=%d,downlink_num=%d!\n",
			link_index, oplus_score_uplink_num, oplus_score_downlink_num);

	if (link_index == 0) {
		printk("[oplus_score]:error, link index is 0!\n");
		return;
	}

	spin_lock_bh(&uplink_score_lock);
	for (i = 0; i < MAX_LINK_NUM; i++) {
		if (uplink_score_info[i].link_index == link_index) {
			uplink_score_info[i].link_index = 0;
			uplink_score_info[i].uplink_retrans_packets = 0;
			uplink_score_info[i].uplink_packets = 0;
			uplink_score_info[i].uplink_nodata_count = 0;
			for (j = 0; j < SCORE_WINDOW; j++) {
				uplink_score_info[i].uplink_score_save[j] = -1;
			}
			uplink_score_info[i].uplink_score_index = 0;
			oplus_score_uplink_num--;
		}
	}
	spin_unlock_bh(&uplink_score_lock);

	spin_lock_bh(&downlink_score_lock);
	for (i = 0; i < MAX_LINK_NUM; i++) {
		if (downlink_score_info[i].link_index == link_index) {
			downlink_score_info[i].link_index = 0;
			downlink_score_info[i].downlink_retrans_packets = 0;
			downlink_score_info[i].downlink_packets = 0;
			for (j = 0; j < SCORE_WINDOW; j++) {
				downlink_score_info[i].downlink_score_save[j] = -1;
			}
			downlink_score_info[i].downlink_score_index = 0;
			oplus_score_downlink_num--;
		}
	}
	spin_unlock_bh(&downlink_score_lock);

	return;
}

static void oplus_score_clear_link(struct nlattr *nla)
{
	int i, j;

	spin_lock_bh(&uplink_score_lock);
	for (i = 0; i < MAX_LINK_NUM; i++) {
		uplink_score_info[i].link_index = 0;
		uplink_score_info[i].uplink_retrans_packets = 0;
		uplink_score_info[i].uplink_packets = 0;
		uplink_score_info[i].uplink_nodata_count = 0;
		for (j = 0; j < SCORE_WINDOW; j++) {
			uplink_score_info[i].uplink_score_save[j] = -1;
		}
		uplink_score_info[i].uplink_score_index = 0;
	}
	oplus_score_uplink_num = 0;
	spin_unlock_bh(&uplink_score_lock);

	spin_lock_bh(&downlink_score_lock);
	for (i = 0; i < MAX_LINK_NUM; i++) {
		downlink_score_info[i].link_index = 0;
		downlink_score_info[i].downlink_retrans_packets = 0;
		downlink_score_info[i].downlink_packets = 0;
		for (j = 0; j < SCORE_WINDOW; j++) {
			downlink_score_info[i].downlink_score_save[j] = -1;
		}
		downlink_score_info[i].downlink_score_index = 0;
	}
	oplus_score_downlink_num = 0;
	spin_unlock_bh(&downlink_score_lock);

	return;
}

static void oplus_score_config(struct nlattr *nla)
{
	struct score_param_st *config;
	config = (struct score_param_st*)NLA_DATA(nla);
	oplus_score_param_info = *config;
	oplus_score_debug = oplus_score_param_info.score_debug;
	return;
}

static int oplus_score_netlink_rcv_msg(struct sk_buff *skb, struct genl_info *info)
{
	int ret = 0;
	struct nlmsghdr *nlhdr;
	struct genlmsghdr *genlhdr;
	struct nlattr *nla;

	nlhdr = nlmsg_hdr(skb);
	genlhdr = nlmsg_data(nlhdr);
	nla = genlmsg_data(genlhdr);

	if (oplus_score_user_pid == 0) {
		oplus_score_user_pid = nlhdr->nlmsg_pid;
		if (oplus_score_debug) {
			printk("[oplus_score]:set oplus_score_user_pid=%u.\n", oplus_score_user_pid);
		}
	}

	/* to do: may need to some head check here*/
	if (oplus_score_debug) {
		printk("[oplus_score]:score_netlink_rcv_msg type=%u.\n", nla->nla_type);
	}
	switch (nla->nla_type) {
	case OPLUS_SCORE_MSG_ENABLE:
		oplus_score_enable(nla);
		break;
	case OPLUS_SCORE_MSG_FOREGROUND_ANDROID_UID:
		oplus_score_set_foreground_uid(nla);
		break;
	case OPLUS_SCORE_MSG_REQUEST_SCORE:
		oplus_score_request_score(nla);
		break;
	case OPLUS_SCORE_MSG_ADD_LINK:
		oplus_score_add_link(nla);
		break;
	case OPLUS_SCORE_MSG_DEL_LINK:
		oplus_score_del_link(nla);
		break;
	case OPLUS_SCORE_MSG_CLEAR_LINK:
		oplus_score_clear_link(nla);
		break;
	case OPLUS_SCORE_MSG_CONFIG:
		oplus_score_config(nla);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static int oplus_score_netlink_init(void)
{
	int ret;
	ret = genl_register_family(&oplus_score_genl_family);
	if (ret) {
		printk("[oplus_score]:genl_register_family:%s failed,ret = %d\n", OPLUS_SCORE_FAMILY_NAME, ret);
		return ret;
	} else {
		printk("[oplus_score]:genl_register_family complete, id = %d!\n", oplus_score_genl_family.id);
	}

	return 0;
}

static void oplus_score_netlink_exit(void)
{
	genl_unregister_family(&oplus_score_genl_family);
}

static int proc_set_test_foreground_uid(struct ctl_table *ctl, int write, void __user *buffer, size_t *lenp,loff_t *ppos)
{
    int ret;
	u32 data[3];
	struct nlattr *nla = (struct nlattr*)data;

    ret = proc_dointvec(ctl, write, buffer, lenp, ppos);
	printk("[oplus_score]:proc_set_test_foreground_uid,write=%d,ret=%d\n", write, ret);
    if (ret == 0) {
		data[1] = 1;
		data[2] = test_foreground_uid;
        if (test_foreground_uid) {
            oplus_score_set_foreground_uid(nla);
        }
    }

    return ret;
}

static int proc_set_test_link_index(struct ctl_table *ctl, int write, void __user *buffer, size_t *lenp,loff_t *ppos)
{
    int ret;
	u32 data[2];
	struct nlattr *nla = (struct nlattr*)data;
	u32 old_link_index = test_link_index;

    ret = proc_dointvec(ctl, write, buffer, lenp, ppos);
	if (oplus_score_debug) {
		printk("[oplus_score]:proc_set_test_link,write=%d,ret=%d\n", write, ret);
	}
    if (ret == 0) {
        if (test_link_index) {
			data[1] = test_link_index;
            oplus_score_add_link(nla);
        }
        else{
			data[1] = old_link_index;
            oplus_score_del_link(nla);
        }
    }

    return ret;
}

static struct ctl_table oplus_score_sysctl_table[] =
{
	{
		.procname	= "oplus_score_debug",
		.data		= &oplus_score_debug,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec,
	},
	{
		.procname	= "oplus_para_rtt",
		.data		= &para_rtt,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec,
	},
	{
		.procname	= "oplus_score_para_rate",
		.data		= &para_rate,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec,
	},
	{
		.procname	= "oplus_score_para_loss",
		.data		= &para_loss,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec,
	},
	{
		.procname	= "test_foreground_uid",
		.data		= &test_foreground_uid,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_set_test_foreground_uid,
	},
	{
		.procname	= "test_link_index",
		.data		= &test_link_index,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_set_test_link_index,
	},
	{ }
};

static int oplus_score_sysctl_init(void)
{
	oplus_score_table_hrd = register_net_sysctl(&init_net, "net/oplus_score", oplus_score_sysctl_table);
	return oplus_score_table_hrd == NULL ? -ENOMEM : 0;
}

static void oplus_score_param_init(void)
{
	oplus_score_uplink_num = 0;
	oplus_score_uplink_num = 0;
	oplus_score_enable_flag = 1;
	oplus_score_user_pid = 0;
	memset(oplus_score_foreground_uid, 0, sizeof(oplus_score_foreground_uid));
	memset(&uplink_score_info, 0, sizeof(uplink_score_info));
	memset(&downlink_score_info, 0, sizeof(downlink_score_info));
	oplus_score_param_info.score_debug = 0;
	oplus_score_param_info.threshold_retansmit = 10;
	oplus_score_param_info.threshold_normal = 100;
	oplus_score_param_info.smooth_factor = 20;
	oplus_score_param_info.protect_score = 60;
	oplus_score_param_info.threshold_gap = 5;
}

static int __init oplus_score_init(void)
{
	int ret = 0;

	ret = oplus_score_netlink_init();
	if (ret < 0) {
		printk("[oplus_score]:init module failed to init netlink, ret =% d\n",  ret);
		return ret;
	} else {
		printk("[oplus_score]:init module init netlink successfully.\n");
	}

	oplus_score_param_init();
	spin_lock_init(&uplink_score_lock);
	spin_lock_init(&downlink_score_lock);
	ret = nf_register_net_hooks(&init_net, oplus_score_netfilter_ops, ARRAY_SIZE(oplus_score_netfilter_ops));
	if (ret < 0) {
		printk("oplus_score_init netfilter register failed, ret=%d\n", ret);
		oplus_score_netlink_exit();
		return ret;
	} else {
		printk("oplus_score_init netfilter register successfully.\n");
	}

	oplus_score_sysctl_init();
	oplus_score_report_timer_init();
	oplus_score_report_timer_start();
	return ret;
}

static void __exit oplus_score_fini(void)
{
	oplus_score_netlink_exit();
	nf_unregister_net_hooks(&init_net, oplus_score_netfilter_ops, ARRAY_SIZE(oplus_score_netfilter_ops));
	if (oplus_score_table_hrd) {
		unregister_net_sysctl_table(oplus_score_table_hrd);
	}
	oplus_score_report_timer_del();
}

module_init(oplus_score_init);
module_exit(oplus_score_fini);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("oplus_score");
