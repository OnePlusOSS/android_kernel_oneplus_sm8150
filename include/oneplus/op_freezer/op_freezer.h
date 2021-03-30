/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_OP_FREEZER_H
#define _LINUX_OP_FREEZER_H

#include <linux/freezer.h>

#define OP_FREEZER_NOERROR             (0)
#define OP_FREEZER_ERROR               (-1)
#define MIN_USERAPP_UID (10000)
#define INTERFACETOKEN_BUFF_SIZE (100)
#define PARCEL_OFFSET (16) // sync with the writeInterfaceToken

/* op_freezer_message for communication with freezer native daemon
 * type: async binder/sync binder/signal/pkg/loopback
 *      Only loop back type is duplex (native daemon <---> kernel) for handshake
 * port: native daemon pid
 * caller_pid: binder, caller -> unfreeze (target) UID
 * target_uid: UID want to be unfrozen
 * pkg_cmd: Add/Remove monitored UID
 */
struct op_freezer_message {
	int type;
	int port;  // pid

	int caller_pid;  // caller -> unfreeze UID
	int target_uid;         // unfreeze UID, pkg add/remove UID

	int pkg_cmd;     //Add/remove monitored uid

	int code;
	char rpc_name[INTERFACETOKEN_BUFF_SIZE];
};

// op_freezer message type definition
enum message_type {
	//kernel --> native daemon
	ASYNC_BINDER,
	SYNC_BINDER,
	FROZEN_TRANS,
	SIGNAL,
	PKG,

	// kernel <--> native daemon
	LOOP_BACK,
	TYPE_MAX
};

// pkg cmd type
enum pkg_cmd {
	ADD_ONE_UID,
	DEL_ONE_UID,
	DEL_ALL_UID,

	PKG_CMD_MAX
};

//Check if the thread group is frozen
static inline bool is_frozen_tg(struct task_struct *task)
{
	return (freezing(task->group_leader) || frozen(task->group_leader));
}

int op_freezer_report(enum message_type type, int caller_pid, int target_uid, const char *rpc_name, int code);
void op_freezer_network_cmd_parse(uid_t uid, enum pkg_cmd cmd);
void op_freezer_check_frozen_transcation(uid_t uid);
int op_freezer_netfilter_init(void);
void op_freezer_netfilter_deinit(void);

#endif

