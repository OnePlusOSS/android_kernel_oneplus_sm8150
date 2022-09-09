/***********************************************************
** Copyright (C), 2009-2019, OPLUS Mobile Comm Corp., Ltd.
** VENDOR_EDIT
** File: - oplus_nwpower.h
** Description: BugID:2120730, Add for FEATURE_DATA_NWPOWER
**
** Version: 1.0
** Date : 2019/07/31
** TAG: OPLUS_ARCH_EXTENDS
**
** ------------------ Revision History:------------------------
** <author> <data> <version > <desc>
** Asiga 2019/07/31 1.0 build this module
****************************************************************/
#ifndef __OPLUS_NWPOWER_H_
#define __OPLUS_NWPOWER_H_

#include <linux/types.h>

#define OPLUS_TOTAL_IP_SUM                  60
#define OPLUS_TRANSMISSION_INTERVAL         3 * 1000/* 3s */
#define OPLUS_TCP_RETRANSMISSION_INTERVAL   1 * 1000/* 1s */

/* Add for IPA wakeup */
struct oplus_tcp_hook_struct {
	u32 uid;
	u32 pid;
	bool is_ipv6;
	u32 ipv4_addr;
	u64 ipv6_addr1;
	u64 ipv6_addr2;
	u64 set[OPLUS_TOTAL_IP_SUM * 3];
};

/* Add for QMI wakeup */
extern void oplus_match_qrtr_service_port(int type, int id, int port);
extern void oplus_match_qrtr_wakeup(int src_node, int src_port, int dst_port,
	unsigned int arg1, unsigned int arg2);

extern void oplus_nwpower_hook_on(bool normal);
extern void oplus_nwpower_hook_off(bool normal, bool unsl);

uid_t get_uid_from_sock(const struct sock *sk);

#endif /* __OPLUS_NWPOWER_H_ */
