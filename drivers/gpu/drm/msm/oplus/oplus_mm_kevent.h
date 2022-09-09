/*
 * oplus_kevent.h - for kevent action upload upload to user layer
 *	author by pdl@oplus.com
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/skbuff.h>
#include <linux/types.h>
#include <linux/netlink.h>
#include <net/net_namespace.h>
#include <net/sock.h>
#include <linux/version.h>

#ifndef CONFIG_OPLUS_MM_KEVENT
#define CONFIG_OPLUS_MM_KEVENT

#define MAX_PAYLOAD_TAG 			 (128)
#define MAX_PAYLOAD_EVENTID 		 (128)
#define MAX_PAYLOAD_DATASIZE		 (1024)
#define OPLUS_NETLINK_MM_KEVENT 	 (41)

#define MM_KEVENT_MODULE_SIZE_MAX	 (16)
#define MM_KEVENT_MODULE_LEN_MAX	 (64)

#define MM_KEVENT_BAD_VALUE 		 (-1)
#define MM_KEVENT_NO_ERROR			 (0)

//#define OPLUS_NETLINK_MM_KEVENT_TEST
#define OPLUS_NETLINK_MM_DBG_LV1     0x1
#define OPLUS_NETLINK_MM_DBG_LV2     0x2

#define DP_FB_EVENT 	"mm_kevent_dp"
#define AUDIO_EVENT 	"mm_kevent_ad"

enum mm_kevent_type {
	MM_KEVENT_NOME = 0x0,
	MM_KEVENT_CONNECT,
};

struct mm_kevent_module {
	u32 pid;
	char modl[MM_KEVENT_MODULE_LEN_MAX];
};

typedef void (*mm_kevent_recv_user_func)(int type, int flags, char* data);

struct mm_kevent_packet {
	int  type;							/* 0:warrning,1:error,2:hw error*/
	char tag[MAX_PAYLOAD_TAG];			/* logTag */
	char event_id[MAX_PAYLOAD_EVENTID]; /* eventID */
	size_t len; 						/* Length of packet data */
	unsigned char data[0];				/* Optional packet data */
}__attribute__((packed));

int mm_kevent_send_to_user(struct mm_kevent_packet *userinfo);
void mm_kevent_set_recv_user(mm_kevent_recv_user_func recv_func);
void mm_kevent_receive(struct sk_buff *__skbbr);
#endif /* CONFIG_OPLUS_KEVENT_UPLOAD */

