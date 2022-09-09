/*
 * oplus_mm_kevent.c - for kevent action upload,root action upload to user layer
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
#include <linux/proc_fs.h>
#include <net/sock.h>
#include <linux/vmalloc.h>
#include <linux/version.h>
#include <linux/uaccess.h>

#include "oplus_mm_kevent.h"

#ifdef CONFIG_OPLUS_MM_KEVENT

static struct sock *mm_netlink_fd = NULL;
static struct mm_kevent_module mm_modules[MM_KEVENT_MODULE_SIZE_MAX];
static u32 mm_kevent_pid;
static mm_kevent_recv_user_func mm_kevent_recv_fb = NULL;

/* record connect pid and modules*/
void mm_kevent_add_module(u32 pid, char* module) {
	int i	= 0x0;
	int len = 0x0;

	if (!module) {
		return;
	}

	len = strlen(module);
	if (len > MM_KEVENT_MODULE_LEN_MAX) {
		return;
	}

	for (i = 0; i < MM_KEVENT_MODULE_SIZE_MAX; i++) {
		if ((!mm_modules[i].pid) || (!strcmp(mm_modules[i].modl, module))) {
			mm_modules[i].pid = pid;
			memcpy(mm_modules[i].modl, module, len);
			mm_modules[i].modl[len] = 0x0;
			return;
		}
	}

	return;
}

/* record connect pid and modules*/
int mm_kevent_get_pid(char* module) {
	int i = 0;

	if (!module) {
		return MM_KEVENT_BAD_VALUE;
	}

	for (i = 0; i < MM_KEVENT_MODULE_SIZE_MAX; i++) {
		if (!strcmp(mm_modules[i].modl, module)) {
			return mm_modules[i].pid;
		}
	}

	return MM_KEVENT_BAD_VALUE;
}

/* send to user space */
int mm_kevent_send_to_user(struct mm_kevent_packet *userinfo) {
	int ret, size, size_use;
	unsigned int o_tail;
	struct sk_buff *skbuff;
	struct nlmsghdr *nlh;
	struct mm_kevent_packet *packet;
	int pid;

	if (!mm_netlink_fd) {
		printk(KERN_ERR "mm_netlink_fd is null, error return\n");
		return MM_KEVENT_BAD_VALUE;
	}

	/* protect payload too long problem*/
	if (userinfo->len >= MAX_PAYLOAD_DATASIZE) {
		printk(KERN_ERR "mm_kevent payload_length out of range\n");
		return MM_KEVENT_BAD_VALUE;
	}

	size = NLMSG_SPACE(sizeof(struct mm_kevent_packet) + userinfo->len);

	/*allocate new buffer cache */
	skbuff = alloc_skb(size, GFP_ATOMIC);
	if (skbuff == NULL) {
		printk(KERN_ERR "mm_kevent skbuff alloc_skb failed\n");
		return MM_KEVENT_BAD_VALUE;
	}

	/* fill in the data structure */
	nlh = nlmsg_put(skbuff, 0, 0, 0, size - sizeof(*nlh), 0);
	if (nlh == NULL) {
		printk(KERN_ERR "mm_kevent nlmsg_put failaure\n");
		nlmsg_free(skbuff);
		return MM_KEVENT_BAD_VALUE;
	}

	o_tail = skbuff->tail;

	size_use = sizeof(struct mm_kevent_packet) + userinfo->len;

	/* use struct mm_kernel_packet_info for data of nlmsg */
	packet = NLMSG_DATA(nlh);
	memset(packet, 0, size_use);

	/* copy the payload content */
	memcpy(packet, userinfo, size_use);

	//compute nlmsg length
	nlh->nlmsg_len = skbuff->tail - o_tail;

	/* set control field,sender's pid */
	#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0))
	NETLINK_CB(skbuff).pid = 0;
	#else
	NETLINK_CB(skbuff).portid = 0;
	#endif

	NETLINK_CB(skbuff).dst_group = 0;

	/* send data */
	pid = mm_kevent_get_pid(userinfo->tag);
	if (pid == MM_KEVENT_BAD_VALUE) {
		printk("mm_kevent chang send pid=%d,%s \n", mm_kevent_pid, userinfo->tag);
		pid = mm_kevent_pid;
	}
	ret = netlink_unicast(mm_netlink_fd, skbuff, pid, MSG_DONTWAIT);
	if (ret < 0) {
		printk(KERN_ERR "mm_kevent send fail=%d, pid=%d \n", ret, pid);
		return MM_KEVENT_BAD_VALUE;
	}

	return MM_KEVENT_NO_ERROR;
}

EXPORT_SYMBOL(mm_kevent_send_to_user);

void mm_kevent_set_recv_user(mm_kevent_recv_user_func recv_func) {
	mm_kevent_recv_fb = recv_func;
}

EXPORT_SYMBOL(mm_kevent_set_recv_user);

/* kernel receive message from user space */
void mm_kevent_receive(struct sk_buff *__skbbr) {
	struct sk_buff *skbu = NULL;
	struct nlmsghdr *nlh;
	char* pmesg   = NULL;
	uint32_t size = 0x0;

	skbu = skb_get(__skbbr);

	if (skbu->len >= sizeof(struct nlmsghdr)) {
		nlh = (struct nlmsghdr *)skbu->data;
		if ((nlh->nlmsg_len >= sizeof(struct nlmsghdr))
			&& (__skbbr->len >= nlh->nlmsg_len)) {
			u32 mm_kevent_type	= nlh->nlmsg_type;
			u32 mm_kevent_flags = nlh->nlmsg_flags;
			mm_kevent_pid	= nlh->nlmsg_pid;
			size = (nlh->nlmsg_len - NLMSG_LENGTH(0));
			if (size) {
				pmesg = (char*)kmalloc(size + 0x10, GFP_KERNEL);
				if (pmesg) {
					memcpy(pmesg, NLMSG_DATA(nlh), size);
					pmesg[size] = 0x0;
					if (mm_kevent_type == MM_KEVENT_CONNECT) {
						mm_kevent_add_module(mm_kevent_pid, pmesg);
					}
					if (mm_kevent_flags) {
						printk("mm_kevent recv pid=%d, type=%d, flags=%d, data:%s\n",
						mm_kevent_pid, mm_kevent_type, mm_kevent_flags, pmesg);
					}
				}
			}
			if (mm_kevent_recv_fb) {
				mm_kevent_recv_fb(mm_kevent_type, mm_kevent_flags, pmesg);
			}
		}
	}

	if (pmesg) {
		kfree(pmesg);
	}
	if (skbu) {
		kfree_skb(skbu);
	}
}

int __init mm_kevent_module_init(void) {
	struct netlink_kernel_cfg cfg = {
		.groups = 0x0,
		.input	= mm_kevent_receive,
	};

	mm_netlink_fd = netlink_kernel_create(&init_net, OPLUS_NETLINK_MM_KEVENT, &cfg);
	if (!mm_netlink_fd) {
		printk(KERN_ERR "mm_kevent can not create a socket\n");
		return MM_KEVENT_BAD_VALUE;
	}

	memset(mm_modules, 0x0, sizeof(mm_modules));

	printk(KERN_ERR "mm_kevent ok\n");
	return MM_KEVENT_NO_ERROR;
}

void __exit mm_kevent_module_exit(void) {
	sock_release(mm_netlink_fd->sk_socket);
	printk("mm_kevent exit\n");
}

module_init(mm_kevent_module_init);
module_exit(mm_kevent_module_exit);

MODULE_AUTHOR("deliang.peng");
MODULE_DESCRIPTION("mm_kevent@1.0");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL");

#endif /* CONFIG_OPLUS_MM_KEVENT */

