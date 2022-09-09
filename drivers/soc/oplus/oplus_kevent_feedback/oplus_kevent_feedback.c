/***************************************************************
** Copyright (C),  2018,  OPLUS Mobile Comm Corp.,  Ltd
** VENDOR_EDIT
** File : oplus_kevent_feedback.h
** Description : oplus kevent feedback data
** Version : 1.2
** Date : 2019/03/09
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**   Guo.Ling          2018/12/03        1.0           Build this moudle
**   LiPing-M          2019/01/29        1.0           ADD smmu and diplay log
**   GaoTing.Gan       2019/03/08        1.2           Add venus dump feedback and public the interface
******************************************************************/
#include <linux/err.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/oplus_kevent.h>
#include <soc/oplus/oplus_kevent_feedback.h>
#include <linux/delay.h>
#include <linux/mutex.h>
/* static DEFINE_MUTEX(mm_kevent_lock); */

int upload_mm_kevent_feedback_data(enum OPLUS_MM_DIRVER_FB_EVENT_MODULE module, unsigned char *payload) {
	struct kernel_packet_info *user_msg_info;
	char log_tag[32] = "psw_multimedia";
	char event_id_display[20] = "20181203";
	char event_id_audio[20] = "20181205";
	char event_id_video[20] = "20181202";
	void * buffer = NULL;
	int len, size;

	/* GaoTing.Gan del, don't need lock */
	/* mutex_lock(&mm_kevent_lock); */

	len = strlen(payload);

	size = sizeof(struct kernel_packet_info) + len + 1;
	printk(KERN_INFO "kevent_send_to_user:size=%d\n", size);

	buffer = kmalloc(size, GFP_ATOMIC);
	memset(buffer, 0, size);
	user_msg_info = (struct kernel_packet_info *)buffer;
	user_msg_info->type = 1;

	memcpy(user_msg_info->log_tag, log_tag, strlen(log_tag) + 1);
	switch (module) {
	default:
		break;
	case OPLUS_MM_DIRVER_FB_EVENT_MODULE_DISPLAY:
		memcpy(user_msg_info->event_id, event_id_display, strlen(event_id_display) + 1);
		break;
	case OPLUS_MM_DIRVER_FB_EVENT_MODULE_AUDIO:
		memcpy(user_msg_info->event_id, event_id_audio, strlen(event_id_audio) + 1);
		break;
	case OPLUS_MM_DIRVER_FB_EVENT_MODULE_VIDEO:
		memcpy(user_msg_info->event_id, event_id_video, strlen(event_id_video) + 1);
		break;
	}

	user_msg_info->payload_length = len + 1;
	memcpy(user_msg_info->payload, payload, len + 1);

	kevent_send_to_user(user_msg_info);
	msleep(20);
	kfree(buffer);
	/* mutex_unlock(&mm_kevent_lock); */
	return 0;
}
