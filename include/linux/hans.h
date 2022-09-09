/***********************************************************
** Copyright (C), 2008-2019, OPLUS Mobile Comm Corp., Ltd.
** VENDOR_EDIT
** File: hans.h
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

#ifndef _HANS_H
#define _HANS_H

#include <linux/freezer.h>
#include <linux/cgroup.h>
#include "../../kernel/sched/sched.h"

#define HANS_NOERROR             (0)
#define HANS_ERROR               (-1)
#define MIN_USERAPP_UID (10000)
#define HANS_SYSTEM_UID (1000)
#define INTERFACETOKEN_BUFF_SIZE (140)
#define PARCEL_OFFSET (16) /* sync with the writeInterfaceToken */
#define CPUCTL_VERSION (2)

/* hans_message for comunication with HANS native deamon
 * type: async binder/sync binder/signal/pkg/loopback
 *      Only loop back type is duplex (native deamon <---> kernel) for handshake
 * port: native deamon pid
 * caller_pid: binder, caller -> unfreeze (target) UID
 * target_uid: UID want to be unfrozen
 * pkg_cmd: Add/Remove monitored UID
 */
struct hans_message {
        int type;
        int port;  /*pid*/

	int caller_uid;
	int caller_pid;
	int target_pid;
	int target_uid;

        int pkg_cmd;     /*Add/remove monitored uid*/

        int code;
        char rpc_name[INTERFACETOKEN_BUFF_SIZE];
};

/*hans message type definition*/
enum message_type {
        /*kernel --> native deamon*/
        ASYNC_BINDER,
        SYNC_BINDER,
        FROZEN_TRANS,
        SIGNAL,
        PKG,
        SYNC_BINDER_CPUCTL,
        SIGNAL_CPUCTL,
        CPUCTL_TRANS,

        /*kernel <--> native deamon*/
        LOOP_BACK,
        TYPE_MAX
};

/*pkg cmd type*/
enum pkg_cmd {
        ADD_ONE_UID,
        DEL_ONE_UID,
        DEL_ALL_UID,
        PKG_CMD_MAX
};

/*Check if the thread group is frozen*/
static inline bool is_frozen_tg(struct task_struct *task)
{
        return (freezing(task->group_leader) || frozen(task->group_leader));
}

int hans_report(enum message_type type, int caller_pid, int caller_uid, int target_pid, int target_uid, const char *rpc_name, int code);
void hans_network_cmd_parse(uid_t uid, enum pkg_cmd cmd);
void hans_check_frozen_transcation(uid_t uid, enum message_type type);
int hans_netfilter_init(void);
void hans_netfilter_deinit(void);

#if defined(CONFIG_CFS_BANDWIDTH)
static inline bool is_belong_cpugrp(struct task_struct *task)
{
	if (task->sched_task_group != NULL) {
		struct cfs_bandwidth cfs_b = task->sched_task_group->cfs_bandwidth;
		if (cfs_b.quota != -1) {
			return true;
		} else if (cfs_b.quota == -1) {
			return false;
		}
	}

	return false;
}
#else
static inline bool is_belong_cpugrp(struct task_struct *task)
{
	return false;
}
#endif

#endif /*_HANS_H*/
