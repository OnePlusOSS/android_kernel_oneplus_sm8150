/***********************************************************
** Copyright (C), 2009-2019, OPLUS Mobile Comm Corp., Ltd.
** VENDOR_EDIT
** File: - oplus_nwpower.c
** Description: BugID:2120730, Add for FEATURE_DATA_NWPOWER
**
** Version: 1.0
** Date : 2019/07/31
**
** ------------------ Revision History:------------------------
** <author> <data> <version > <desc>
** Asiga 2019/07/31 1.0 build this module
****************************************************************/
#include <linux/module.h>
#include <linux/qrtr.h>
#include <linux/ipc_logging.h>
#include <linux/atomic.h>
#include <linux/skbuff.h>
#include <linux/err.h>
#include <net/sock.h>
#include <linux/netlink.h>
#include <linux/version.h>
#include <net/oplus_nwpower.h>

#define GLINK_MODEM_NODE_ID    0x0
#define GLINK_ADSP_NODE_ID     0x5
#define GLINK_CDSP_NODE_ID     0xa
#define GLINK_SLPI_NODE_ID     0x9
#define GLINK_NPU_NODE_ID      0xb

#define UNSL_MSG_MAX           50

#define QRTR_SERVICE_SUM       120
#define MODEM_WAKEUP_SRC_NUM   3
#define MODEM_IPA_WS_INDEX     1
#define MODEM_QMI_WS_INDEX     2

u64 nw_modem_wakeup_times = 0;
u64 nw_wifi_wakeup_times = 0;
u64 nw_adsp_wakeup_times = 0;
u64 nw_cdsp_wakeup_times = 0;
u64 nw_slpi_wakeup_times = 0;
u64 nw_alarm_wakeup_times = 0;
u64 nw_qrtr_wakeup_times = 0;
u64 nw_pcie2_wakeup_times = 0;/*Add for SM8250*/
u64 nw_ipcc_wakeup_times = 0;/*Add for SM7250*/

static u64 tcp_input_wakeup_times = 0;
static u64 tcp_output_wakeup_times = 0;
static u64 tcp_input_retrans_wakeup_times = 0;
static u64 tcp_output_retrans_wakeup_times = 0;

static DEFINE_SPINLOCK(oplus_qrtrhook_lock);
static atomic_t qrtr_wakeup_hook_boot = ATOMIC_INIT(0);
static u64 service_wakeup_times[QRTR_SERVICE_SUM][4] = {{0}};

static int oplus_match_tcp_hook(struct oplus_tcp_hook_struct *pval);
static void oplus_tcp_hook_insert_sort(struct oplus_tcp_hook_struct *pval);
static void oplus_tcp_output_hook_work_callback(struct work_struct *work);
static void oplus_tcp_input_hook_work_callback(struct work_struct *work);
static void oplus_tcp_output_tcpsynretrans_hook_work_callback(
	struct work_struct *work);
static void oplus_tcp_input_tcpsynretrans_hook_work_callback(
	struct work_struct *work);
static void oplus_match_qrtr_new_service_port(int id, int port);
static void oplus_match_qrtr_del_service_port(int id);
static void oplus_match_glink_wakeup(int src_node);
static void oplus_print_qrtr_wakeup(bool unsl);
static int oplus_print_tcp_wakeup(bool unsl, int unsl_first_index,
	const char *type, struct oplus_tcp_hook_struct *pval);
static void oplus_print_ipa_wakeup(bool unsl);
static void oplus_reset_qrtr_wakeup(void);
static void oplus_reset_ipa_wakeup(void);
static int oplus_nwpower_send_to_user(int msg_type, char *msg_data, int msg_len);
static void oplus_nwpower_netlink_rcv(struct sk_buff *skb);
static int oplus_nwpower_netlink_rcv_msg(struct sk_buff *skb,
	struct nlmsghdr *nlh, struct netlink_ext_ack *extack);
static int oplus_nwpower_netlink_init(void);
static void oplus_nwpower_netlink_exit(void);

extern int qrtr_first_msg;

extern u64 wakeup_source_count_modem;
extern int modem_wakeup_src_count[MODEM_WAKEUP_SRC_NUM];

u64 oplus_tcp_last_rcv_packet_stamp = 0;
struct timespec oplus_tcp_last_transmission_stamp;
atomic_t oplus_tcp_is_input = ATOMIC_INIT(
		0);/*1=v4_input,2=v6_input,3=output,0=default*/

atomic_t ipa_wakeup_hook_boot = ATOMIC_INIT(0);
struct oplus_tcp_hook_struct oplus_tcp_output_hook = {
	.is_ipv6 = false,
	.set = {0},
};
struct oplus_tcp_hook_struct oplus_tcp_input_hook = {
	.is_ipv6 = false,
	.set = {0},
};
DECLARE_WORK(oplus_tcp_output_hook_work, oplus_tcp_output_hook_work_callback);
DECLARE_WORK(oplus_tcp_input_hook_work, oplus_tcp_input_hook_work_callback);

atomic_t tcpsynretrans_hook_boot = ATOMIC_INIT(0);
struct oplus_tcp_hook_struct oplus_tcp_output_tcpsynretrans_hook = {
	.is_ipv6 = false,
	.set = {0},
};
struct oplus_tcp_hook_struct oplus_tcp_input_tcpsynretrans_hook = {
	.is_ipv6 = false,
	.set = {0},
};
DECLARE_WORK(oplus_tcp_output_tcpsynretrans_hook_work,
	oplus_tcp_output_tcpsynretrans_hook_work_callback);
DECLARE_WORK(oplus_tcp_input_tcpsynretrans_hook_work,
	oplus_tcp_input_tcpsynretrans_hook_work_callback);

/*Netlink*/
enum {
	NW_POWER_ANDROID_PID                    = 0x11,
	NW_POWER_BOOT_MONITOR                   = 0x12,
	NW_POWER_STOP_MONITOR                   = 0x13,
	NW_POWER_STOP_MONITOR_UNSL              = 0x14,
	NW_POWER_UNSL_MONITOR                   = 0x15,
	NW_POWER_BOOT_MONITOR_SCREEN_ON         = 0x16,
};
static DEFINE_MUTEX(netlink_mutex);
static u32 oplus_nwpower_pid = 0;
static struct sock *oplus_nwpower_sock;
static u64 unsl_msg[UNSL_MSG_MAX] = {0};

uid_t get_uid_from_sock(const struct sock *sk)
{
	uid_t sk_uid;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
	const struct file *filp = NULL;
#endif

	if(NULL == sk || !sk_fullsock(sk) || NULL == sk->sk_socket) {
		return 0;
	}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
	filp = sk->sk_socket->file;
	if(NULL == filp) {
		return 0;
	}
	sk_uid = __kuid_val(filp->f_cred->fsuid);
#else
	sk_uid = __kuid_val(sk->sk_uid);
#endif
	return sk_uid;
}

static int oplus_match_tcp_hook(struct oplus_tcp_hook_struct *pval)
{
	int i;
	u32 uid = 0;
	u64 count = 0;
	bool handle = false;

	for (i = 0; i < OPLUS_TOTAL_IP_SUM; ++i) {
		if (pval->is_ipv6) {
			if (pval->ipv6_addr1 == pval->set[3 * i]
				&& pval->ipv6_addr2 == pval->set[3 * i + 1]) {
				count = pval->set[3 * i + 2] & 0xFFFFFFFF;
				uid = (pval->set[3 * i + 2] & 0xFFFFFFFF00000000) >> 32;

				if (uid == 0) {
					pval->set[3 * i + 2] = (u64)(pval->uid) << 32 | (u32)(++count);

				} else {
					pval->set[3 * i + 2] = (pval->set[3 * i + 2] & 0xFFFFFFFF00000000) | (u32)(
							++count);
				}

				handle = true;
				break;

			} else if (pval->set[3 * i + 2] == 0) {
				pval->set[3 * i] = pval->ipv6_addr1;
				pval->set[3 * i + 1] = pval->ipv6_addr2;
				count = 1;
				pval->set[3 * i + 2] = (u64)(pval->uid) << 32 | (u32)(count);
				handle = true;
				break;
			}

		} else {
			if (pval->ipv4_addr == (pval->set[3 * i + 2] & 0xFFFFFFFF)) {
				count = (pval->set[3 * i + 2] & 0xFFFC000000000000) >> 50;
				uid = (pval->set[3 * i + 2] & 0x3FFFF00000000) >> 32;

				if (uid == 0) {
					count++;
					count = count << 18 | (pval->uid & 0x3FFFF);
					pval->set[3 * i + 2] = count << 32 | (pval->set[3 * i + 2] & 0xFFFFFFFF);

				} else {
					pval->set[3 * i + 2] = (++count) << 50 | (pval->set[3 * i + 2] &
							0x3FFFFFFFFFFFF);
				}

				handle = true;
				break;

			} else if (pval->set[3 * i + 2] == 0) {
				count = 1 << 18 | (pval->uid & 0x3FFFF);
				pval->set[3 * i + 2] = count << 32 | pval->ipv4_addr;
				handle = true;
				break;
			}
		}
	}

	if (!handle) {
		oplus_tcp_hook_insert_sort(pval);
		i = OPLUS_TOTAL_IP_SUM - 1;

		if (pval->is_ipv6) {
			pval->set[3 * i] = pval->ipv6_addr1;
			pval->set[3 * i + 1] = pval->ipv6_addr2;
			count = 1;
			pval->set[3 * i + 2] = (u64)(pval->uid) << 32 | (u32)(count);

		} else {
			pval->set[3 * i] = 0;
			pval->set[3 * i + 1] = 0;
			count = 1 << 18 | (pval->uid & 0x3FFFF);
			pval->set[3 * i + 2] = (u64)count << 32 | pval->ipv4_addr;
		}
	}

	return i;
}

static void oplus_tcp_hook_insert_sort(struct oplus_tcp_hook_struct *pval)
{
	int i;
	int j;
	u64 count = 0;
	u64 temp_sort[3] = {0};

	/*Insert sort*/
	for (i = 1; i < OPLUS_TOTAL_IP_SUM; ++i) {
		temp_sort[0] = pval->set[3 * i];
		temp_sort[1] = pval->set[3 * i + 1];
		temp_sort[2] = pval->set[3 * i + 2];

		/*If IPv4*/
		if (temp_sort[0] == 0 && temp_sort[1] == 0 && temp_sort[2] != 0) {
			count = (temp_sort[2] & 0xFFFC000000000000) >> 50;

		} else {
			count = temp_sort[2] & 0xFFFFFFFF;
		}

		j = i - 1;

		while (j >= 0) {
			if ((pval->set[3 * j] == 0) && (pval->set[3 * j + 1] == 0) && (pval->set[3 * j + 2] != 0)) {
				if (count > (pval->set[3 * j + 2] & 0xFFFC000000000000) >> 50) {
					pval->set[3 * (j + 1)] = pval->set[3 * j];
					pval->set[3 * (j + 1) + 1] = pval->set[3 * j + 1];
					pval->set[3 * (j + 1) + 2] = pval->set[3 * j + 2];
					--j;

				} else {
					break;
				}

			} else {
				if (count > (pval->set[3 * j + 2] & 0xFFFFFFFF)) {
					pval->set[3 * (j + 1)] = pval->set[3 * j];
					pval->set[3 * (j + 1) + 1] = pval->set[3 * j + 1];
					pval->set[3 * (j + 1) + 2] = pval->set[3 * j + 2];
					--j;

				} else {
					break;
				}
			}
		}

		pval->set[3 * (j + 1)] = temp_sort[0];
		pval->set[3 * (j + 1) + 1] = temp_sort[1];
		pval->set[3 * (j + 1) + 2] = temp_sort[2];
	}
}

static void oplus_tcp_output_hook_work_callback(struct work_struct *work)
{
	int i = oplus_match_tcp_hook(&oplus_tcp_output_hook);

	if (oplus_tcp_output_hook.is_ipv6) {
		printk("[oplus_nwpower] IPAOutputWakeup: [%ld,%ld], %d, %d, %d",
			oplus_tcp_output_hook.ipv6_addr1, oplus_tcp_output_hook.ipv6_addr2,
			oplus_tcp_output_hook.pid, oplus_tcp_output_hook.uid,
			oplus_tcp_output_hook.set[3 * i + 2] & 0xFFFFFFFF);

	} else {
		printk("[oplus_nwpower] IPAOutputWakeup: %d, %d, %d, %d",
			oplus_tcp_output_hook.ipv4_addr, oplus_tcp_output_hook.pid,
			oplus_tcp_output_hook.uid,
			(oplus_tcp_output_hook.set[3 * i + 2] & 0xFFFC000000000000) >> 50);
	}

	atomic_set(&oplus_tcp_is_input, 0);
}

static void oplus_tcp_input_hook_work_callback(struct work_struct *work)
{
	int i = oplus_match_tcp_hook(&oplus_tcp_input_hook);

	/*wakeup_source_count_modem++;*/
	/*modem_wakeup_src_count[MODEM_IPA_WS_INDEX]++;*/
	if (oplus_tcp_input_hook.is_ipv6) {
		printk("[oplus_nwpower] IPAInputWakeup: [%ld,%ld], %d, %d, %d",
			oplus_tcp_input_hook.ipv6_addr1, oplus_tcp_input_hook.ipv6_addr2,
			oplus_tcp_input_hook.pid, oplus_tcp_input_hook.uid,
			oplus_tcp_input_hook.set[3 * i + 2] & 0xFFFFFFFF);

	} else {
		printk("[oplus_nwpower] IPAInputWakeup: %d, %d, %d, %d",
			oplus_tcp_input_hook.ipv4_addr, oplus_tcp_input_hook.pid, oplus_tcp_input_hook.uid,
			(oplus_tcp_input_hook.set[3 * i + 2] & 0xFFFC000000000000) >> 50);
	}

	atomic_set(&oplus_tcp_is_input, 0);
}

static void oplus_tcp_output_tcpsynretrans_hook_work_callback(
	struct work_struct *work)
{
	int i = oplus_match_tcp_hook(&oplus_tcp_output_tcpsynretrans_hook);

	if (oplus_tcp_output_tcpsynretrans_hook.is_ipv6) {
		if (oplus_tcp_output_tcpsynretrans_hook.ipv6_addr1 == 0
			&& oplus_tcp_output_tcpsynretrans_hook.ipv6_addr2 == 0) {
			return;
		}

		printk("[oplus_nwpower] TCPOutputRetrans: [%ld,%ld], %d, %d, %d",
			oplus_tcp_output_tcpsynretrans_hook.ipv6_addr1,
			oplus_tcp_output_tcpsynretrans_hook.ipv6_addr2,
			oplus_tcp_output_tcpsynretrans_hook.pid, oplus_tcp_output_tcpsynretrans_hook.uid,
			oplus_tcp_output_tcpsynretrans_hook.set[3 * i + 2] & 0xFFFFFFFF);

	} else {
		if (oplus_tcp_output_tcpsynretrans_hook.ipv4_addr == 0) {
			return;
		}

		printk("[oplus_nwpower] TCPOutputRetrans: %d, %d, %d, %d",
			oplus_tcp_output_tcpsynretrans_hook.ipv4_addr,
			oplus_tcp_output_tcpsynretrans_hook.pid, oplus_tcp_output_tcpsynretrans_hook.uid,
			(oplus_tcp_output_tcpsynretrans_hook.set[3 * i + 2] & 0xFFFC000000000000) >> 50);
	}
}

static void oplus_tcp_input_tcpsynretrans_hook_work_callback(
	struct work_struct *work)
{
	int i = oplus_match_tcp_hook(&oplus_tcp_input_tcpsynretrans_hook);

	if (oplus_tcp_input_tcpsynretrans_hook.is_ipv6) {
		if (oplus_tcp_input_tcpsynretrans_hook.ipv6_addr1 == 0
			&& oplus_tcp_input_tcpsynretrans_hook.ipv6_addr2 == 0) {
			return;
		}

		printk("[oplus_nwpower] TCPInputRetrans: [%ld,%ld], %d, %d, %d",
			oplus_tcp_input_tcpsynretrans_hook.ipv6_addr1,
			oplus_tcp_input_tcpsynretrans_hook.ipv6_addr2,
			oplus_tcp_input_tcpsynretrans_hook.pid, oplus_tcp_input_tcpsynretrans_hook.uid,
			oplus_tcp_input_tcpsynretrans_hook.set[3 * i + 2] & 0xFFFFFFFF);

	} else {
		if (oplus_tcp_input_tcpsynretrans_hook.ipv4_addr == 0) {
			return;
		}

		printk("[oplus_nwpower] TCPInputRetrans: %d, %d, %d, %d",
			oplus_tcp_input_tcpsynretrans_hook.ipv4_addr,
			oplus_tcp_input_tcpsynretrans_hook.pid, oplus_tcp_input_tcpsynretrans_hook.uid,
			(oplus_tcp_input_tcpsynretrans_hook.set[3 * i + 2] & 0xFFFC000000000000) >> 50);
	}
}

extern void oplus_match_qrtr_service_port(int type, int id, int port)
{
	unsigned long flags;
	spin_lock_irqsave(&oplus_qrtrhook_lock, flags);

	if (type == QRTR_TYPE_NEW_SERVER) {
		oplus_match_qrtr_new_service_port(id, port);

	} else if (type == QRTR_TYPE_DEL_SERVER) {
		oplus_match_qrtr_del_service_port(id);
	}

	spin_unlock_irqrestore(&oplus_qrtrhook_lock, flags);
}

static void oplus_match_qrtr_new_service_port(int id, int port)
{
	int i;

	for (i = 0; i < QRTR_SERVICE_SUM; ++i) {
		if (service_wakeup_times[i][0] == 0) {
			service_wakeup_times[i][0] = 1;
			service_wakeup_times[i][1] = id;
			service_wakeup_times[i][2] = port;
			/*printk("[oplus_nwpower] QrtrNewService[%d]: ServiceID: %d, PortID: %d", i, id, port);*/
			break;

		} else {
			if (service_wakeup_times[i][1] == id && service_wakeup_times[i][2] == port) {
				/*printk("[oplus_nwpower] QrtrNewService[%d]: Ignore.");*/
				break;
			}
		}
	}
}

static void oplus_match_qrtr_del_service_port(int id)
{
	int i;

	for (i = 0; i < QRTR_SERVICE_SUM; ++i) {
		if (service_wakeup_times[i][0] == 1 && service_wakeup_times[i][1] == id) {
			service_wakeup_times[i][0] = 0;
			/*printk("[oplus_nwpower] QrtrDelService[%d]: ServiceID: %d", i, id);*/
			break;
		}
	}
}

extern void oplus_match_qrtr_wakeup(int src_node, int src_port, int dst_port,
	unsigned int arg1, unsigned int arg2)
{
	int i;
	int repeat[4] = {0};
	int repeat_index = 0;
	unsigned long flags;

	if (atomic_read(&qrtr_wakeup_hook_boot) == 1 && (qrtr_first_msg == 1)) {
		spin_lock_irqsave(&oplus_qrtrhook_lock, flags);

		for (i = 0; i < QRTR_SERVICE_SUM; ++i) {
			if (service_wakeup_times[i][0] == 1 && (service_wakeup_times[i][2] == src_port
					|| service_wakeup_times[i][2] == dst_port)) {
				if (repeat_index < 4) {
					repeat[repeat_index++] = i;
				}
			}
		}

		if (repeat_index == 1) {
			service_wakeup_times[repeat[0]][3]++;
			printk("[oplus_nwpower] QrtrWakeup: ServiceID: %d, NodeID: %d, PortID: %d, Msg: [%08x %08x], Count: %d",
				service_wakeup_times[repeat[0]][1], src_node,
				service_wakeup_times[repeat[0]][2], arg1, arg2,
				service_wakeup_times[repeat[0]][3]);

		} else if (repeat_index > 1) {
			service_wakeup_times[repeat[repeat_index - 1]][3]++;
			printk("[oplus_nwpower] QrtrWakeup: ServiceID: [%d/%d/%d/%d], NodeID: %d, PortID: %d, Msg: [%08x %08x], Count: %d",
				service_wakeup_times[repeat[0]][1], service_wakeup_times[repeat[1]][1],
				repeat_index > 2 ? service_wakeup_times[repeat[2]][1] : -1,
				repeat_index > 3 ? service_wakeup_times[repeat[3]][1] : -1,
				src_node, service_wakeup_times[repeat[repeat_index - 1]][2], arg1, arg2,
				service_wakeup_times[repeat[repeat_index - 1]][3]);

		} else {
			printk("[oplus_nwpower] QrtrWakeup: ServiceID: %d, NodeID: %d, PortID: %d, Msg: [%08x %08x], Count: %d",
				-1, src_node, -1, arg1, arg2, -1);
		}

		oplus_match_glink_wakeup(src_node);
		qrtr_first_msg = 0;
		spin_unlock_irqrestore(&oplus_qrtrhook_lock, flags);
	}
}

static void oplus_match_glink_wakeup(int src_node)
{
	nw_qrtr_wakeup_times++;

	if (src_node == GLINK_MODEM_NODE_ID) {
		/*wakeup_source_count_modem++;*/
		/*modem_wakeup_src_count[MODEM_QMI_WS_INDEX]++;*/
		nw_modem_wakeup_times++;

	} else if (src_node == GLINK_ADSP_NODE_ID) {
		/*wakeup_source_count_adsp++;*/
		/*glink_adsp_wakeup_times++;*/
	} else if (src_node == GLINK_CDSP_NODE_ID) {
		/*wakeup_source_count_cdsp++;*/
		/*glink_cdsp_wakeup_times++;*/
	} else if (src_node == GLINK_SLPI_NODE_ID) {
		/*glink_slpi_wakeup_times++;*/
	} else if (src_node == GLINK_NPU_NODE_ID) {
		/*glink_npu_wakeup_times++;*/
	}

	/*
	printk("[oplus_nwpower] GlinkWakeup: NodeID: %d, Modem: %d, Qrtr: %d, WiFi: %d, Adsp: %d, Cdsp: %d, Slpi: %d, Alarm: %d, Mpss(PCIE2): %d",
		src_node, nw_modem_wakeup_times, nw_qrtr_wakeup_times, nw_wifi_wakeup_times, nw_adsp_wakeup_times,
		nw_cdsp_wakeup_times, nw_slpi_wakeup_times, nw_alarm_wakeup_times, nw_pcie2_wakeup_times);
	*/
}

static void oplus_print_qrtr_wakeup(bool unsl)
{
	u64 temp[5][4] = {{0}};
	u64 max_count = 0;
	u64 max_count_id = 0;
	int j;
	int i;
	int k;

	for (j = 0; j < 5; ++j) {
		for (i = 0; i < QRTR_SERVICE_SUM; ++i) {
			if (service_wakeup_times[i][0] == 1 && service_wakeup_times[i][3] > max_count) {
				max_count = service_wakeup_times[i][3];
				max_count_id = i;
			}
		}

		for (k = 0; k < 4; ++k) {
			temp[j][k] = service_wakeup_times[max_count_id][k];
		}

		max_count = 0;
		service_wakeup_times[max_count_id][3] = 0;

		if (unsl) {
			if (temp[j][3] > 0) {
				unsl_msg[j] = temp[j][2] << 32 | ((u32)temp[j][1] << 16 | (u16)temp[j][3]);
			}
		}

		if (temp[j][3] > 0)
			printk("[oplus_nwpower] QrtrWakeupMax[%d]: ServiceID: %d, PortID: %d, Count: %d",
				j, temp[j][1], temp[j][2], temp[j][3]);
	}

	if (unsl) {
		unsl_msg[5] = (nw_modem_wakeup_times << 48) |
			((nw_qrtr_wakeup_times & 0xFFFF) << 32) |
			((nw_wifi_wakeup_times & 0xFFFF) << 16) |
			(nw_alarm_wakeup_times & 0xFFFF);
		unsl_msg[6] = (nw_adsp_wakeup_times << 48) |
			((nw_cdsp_wakeup_times & 0xFFFF) << 32) |
			((nw_slpi_wakeup_times & 0xFFFF) << 16) |
			(nw_pcie2_wakeup_times & 0xFFFF);
	}

	printk("[oplus_nwpower] AllWakeups: Modem: %d, Qrtr: %d, WiFi: %d, Adsp: %d, Cdsp: %d, Slpi: %d, Alarm: %d, IPCC: %d",
		nw_modem_wakeup_times, nw_qrtr_wakeup_times, nw_wifi_wakeup_times,
		nw_adsp_wakeup_times,
		nw_cdsp_wakeup_times, nw_slpi_wakeup_times, nw_alarm_wakeup_times,
		nw_ipcc_wakeup_times);
}

static int oplus_print_tcp_wakeup(bool unsl, int unsl_first_index,
	const char *type, struct oplus_tcp_hook_struct *pval)
{
	int i;
	u32 count;
	u32 tcp_wakeup_times = 0;
	oplus_tcp_hook_insert_sort(pval);

	for (i = 0; i < 5; ++i) {
		if (unsl) {
			if (i < 3) {
				unsl_msg[unsl_first_index + 3 * i] = pval->set[3 * i];
				unsl_msg[unsl_first_index + 3 * i + 1] = pval->set[3 * i + 1];
				unsl_msg[unsl_first_index + 3 * i + 2] = pval->set[3 * i + 2];
			}
		}

		if ((pval->set[3 * i] == 0) && (pval->set[3 * i + 1] == 0) && (pval->set[3 * i + 2] != 0)) {
			count = (pval->set[3 * i + 2] & 0xFFFC000000000000) >> 50;

			if (count > 0) {
				printk("[oplus_nwpower] IPA%sMAX[%d]: %d, %d, %d",
					type, i, pval->set[3 * i + 2] & 0xFFFFFFFF,
					(pval->set[3 * i + 2] & 0x3FFFF00000000) >> 32, count);
			}

		} else {
			count = pval->set[3 * i + 2] & 0xFFFFFFFF;

			if (count > 0) {
				printk("[oplus_nwpower] IPA%sMAX[%d]: [%ld,%ld], %d, %d",
					type, i, pval->set[3 * i], pval->set[3 * i + 1],
					pval->set[3 * i + 2] >> 32, count);
			}
		}
	}

	for (i = 0; i < OPLUS_TOTAL_IP_SUM; ++i) {
		if ((pval->set[3 * i] == 0) && (pval->set[3 * i + 1] == 0) && (pval->set[3 * i + 2] != 0)) {
			count = (pval->set[3 * i + 2] & 0xFFFC000000000000) >> 50;

		} else {
			count = pval->set[3 * i + 2] & 0xFFFFFFFF;
		}

		if (count > 0) {
			tcp_wakeup_times += count;
		}
	}

	return tcp_wakeup_times;
}

static void oplus_print_ipa_wakeup(bool unsl)
{
	tcp_input_wakeup_times = oplus_print_tcp_wakeup(unsl, 7, "Input",
			&oplus_tcp_input_hook);
	tcp_output_wakeup_times = oplus_print_tcp_wakeup(unsl, 16, "Output",
			&oplus_tcp_output_hook);
	tcp_input_retrans_wakeup_times = oplus_print_tcp_wakeup(unsl, 25, "InputRetrans",
			&oplus_tcp_input_tcpsynretrans_hook);
	tcp_output_retrans_wakeup_times = oplus_print_tcp_wakeup(unsl, 34,
			"OutputRetrans", &oplus_tcp_output_tcpsynretrans_hook);
	printk("[oplus_nwpower] IPAAllWakeups: TCPInput: %d, TCPOutput: %d, TCPInputRetrans: %d, TCPOutputRetrans: %d",
		tcp_input_wakeup_times, tcp_output_wakeup_times, tcp_input_retrans_wakeup_times,
		tcp_output_retrans_wakeup_times);

	if (unsl) {
		unsl_msg[43] = (tcp_input_wakeup_times << 48) |
			((tcp_output_wakeup_times & 0xFFFF) << 32) |
			((tcp_input_retrans_wakeup_times & 0xFFFF) << 16) |
			(tcp_output_retrans_wakeup_times & 0xFFFF);
	}
}

static void oplus_reset_qrtr_wakeup()
{
	int i;

	for (i = 0; i < QRTR_SERVICE_SUM; ++i) {
		if (service_wakeup_times[i][0] == 1) {
			service_wakeup_times[i][3] = 0;
		}
	}

	nw_modem_wakeup_times = 0;
	nw_wifi_wakeup_times = 0;
	nw_adsp_wakeup_times = 0;
	nw_cdsp_wakeup_times = 0;
	nw_slpi_wakeup_times = 0;
	nw_alarm_wakeup_times = 0;
	nw_qrtr_wakeup_times = 0;
	nw_pcie2_wakeup_times = 0;
}

static void oplus_reset_ipa_wakeup()
{
	int i;
	int j;

	for (i = 0; i < OPLUS_TOTAL_IP_SUM; ++i) {
		for (j = 0; j < 3; ++j) {
			oplus_tcp_input_hook.set[i + j] = 0;
			oplus_tcp_output_hook.set[i + j] = 0;
			oplus_tcp_input_tcpsynretrans_hook.set[i + j] = 0;
			oplus_tcp_output_tcpsynretrans_hook.set[i + j] = 0;
		}
	}

	tcp_input_wakeup_times = 0;
	tcp_output_wakeup_times = 0;
	tcp_input_retrans_wakeup_times = 0;
	tcp_output_retrans_wakeup_times = 0;
}

extern void oplus_nwpower_hook_on(bool normal)
{
	/* if own Netlink is ok, ignore sla */
	if (normal && oplus_nwpower_pid > 0) {
		return;
	}

	atomic_set(&qrtr_wakeup_hook_boot, 1);
	atomic_set(&ipa_wakeup_hook_boot, 1);
	atomic_set(&tcpsynretrans_hook_boot, 1);
}

extern void oplus_nwpower_hook_off(bool normal, bool unsl)
{
	/* if own Netlink is ok, ignore sla */
	unsigned long flags;

	if (normal && oplus_nwpower_pid > 0) {
		return;
	}

	atomic_set(&qrtr_wakeup_hook_boot, 0);
	atomic_set(&ipa_wakeup_hook_boot, 0);
	atomic_set(&tcpsynretrans_hook_boot, 0);

	spin_lock_irqsave(&oplus_qrtrhook_lock, flags);
	oplus_print_qrtr_wakeup(unsl);
	oplus_reset_qrtr_wakeup();
	spin_unlock_irqrestore(&oplus_qrtrhook_lock, flags);

	oplus_print_ipa_wakeup(unsl);
	oplus_reset_ipa_wakeup();

	if (unsl) {
		oplus_nwpower_send_to_user(NW_POWER_UNSL_MONITOR, (char *)unsl_msg,
			sizeof(unsl_msg));
		memset(unsl_msg, 0x0, sizeof(unsl_msg));
	}
}

static int oplus_nwpower_send_to_user(int msg_type, char *msg_data, int msg_len)
{
	int ret = 0;
	struct sk_buff *skb;
	struct nlmsghdr *nlh;

	if (oplus_nwpower_pid == 0) {
		printk("[oplus_nwpower] netlink: oplus_nwpower_pid = 0.\n");
		return -1;
	}

	skb = alloc_skb(NLMSG_SPACE(msg_len), GFP_ATOMIC);

	if (skb == NULL) {
		printk("[oplus_nwpower] netlink: alloc_skb failed.\n");
		return -2;
	}

	nlh = nlmsg_put(skb, 0, 0, msg_type, NLMSG_ALIGN(msg_len), 0);

	if (nlh == NULL) {
		printk("[oplus_nwpower] netlink: nlmsg_put failed.\n");
		nlmsg_free(skb);
		return -3;
	}

	nlh->nlmsg_len = NLMSG_HDRLEN + NLMSG_ALIGN(msg_len);

	if (msg_data != NULL) {
		memcpy((char *)NLMSG_DATA(nlh), msg_data, msg_len);
	}

	NETLINK_CB(skb).portid = 0;
	NETLINK_CB(skb).dst_group = 0;
	ret = netlink_unicast(oplus_nwpower_sock, skb, oplus_nwpower_pid, MSG_DONTWAIT);

	if (ret < 0) {
		printk(KERN_ERR "[oplus_nwpower] netlink: netlink_unicast failed, ret = %d.\n",
			ret);
		return -4;
	}

	return 0;
}

static void oplus_nwpower_netlink_rcv(struct sk_buff *skb)
{
	mutex_lock(&netlink_mutex);
	netlink_rcv_skb(skb, &oplus_nwpower_netlink_rcv_msg);
	mutex_unlock(&netlink_mutex);
}

static int oplus_nwpower_netlink_rcv_msg(struct sk_buff *skb,
	struct nlmsghdr *nlh, struct netlink_ext_ack *extack)
{
	int ret = 0;

	switch (nlh->nlmsg_type) {
	case NW_POWER_ANDROID_PID:
		oplus_nwpower_pid = NETLINK_CB(skb).portid;
		printk("[oplus_nwpower] netlink: oplus_nwpower_pid = %d", oplus_nwpower_pid);
		break;

	case NW_POWER_BOOT_MONITOR:
		oplus_nwpower_hook_on(false);
		printk("[oplus_nwpower] netlink: hook_on");
		break;

	case NW_POWER_STOP_MONITOR:
		oplus_nwpower_hook_off(false, false);
		printk("[oplus_nwpower] netlink: hook_off");
		break;

	case NW_POWER_STOP_MONITOR_UNSL:
		oplus_nwpower_hook_off(false, true);
		printk("[oplus_nwpower] netlink: hook_off_unsl");
		break;

	case NW_POWER_BOOT_MONITOR_SCREEN_ON:
		atomic_set(&ipa_wakeup_hook_boot, 2);
		printk("[oplus_nwpower] netlink: hook_screen_on");
		break;

	default:
		return -EINVAL;
	}

	return ret;
}

static int oplus_nwpower_netlink_init(void)
{
	struct netlink_kernel_cfg cfg = {
		.input = oplus_nwpower_netlink_rcv,
	};
	oplus_nwpower_sock = netlink_kernel_create(&init_net, NETLINK_OPLUS_NWPOWERSTATE,
			&cfg);
	return oplus_nwpower_sock == NULL ? -ENOMEM : 0;
}

static void oplus_nwpower_netlink_exit(void)
{
	netlink_kernel_release(oplus_nwpower_sock);
	oplus_nwpower_sock = NULL;
}

static int __init oplus_nwpower_init(void)
{
	int ret = 0;
	ret = oplus_nwpower_netlink_init();

	if (ret < 0) {
		printk("[oplus_nwpower] netlink: failed to init netlink.\n");

	} else {
		printk("[oplus_nwpower] netlink: init netlink successfully.\n");
	}

	return ret;
}

static void __exit oplus_nwpower_fini(void)
{
	oplus_nwpower_netlink_exit();
}

module_init(oplus_nwpower_init);
module_exit(oplus_nwpower_fini);

