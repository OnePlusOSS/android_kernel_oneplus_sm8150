#include <linux/sysfs.h>
#include "oplus_mm_kevent.h"
#include "oplus_mm_kevent_fb.h"
#include <linux/delay.h>
#include <linux/mutex.h>

static void mm_kevent_upload_jobs(struct work_struct *work);

static LIST_HEAD(mm_kevent_list);
static DEFINE_MUTEX(mm_kevent_lock);
static DECLARE_DELAYED_WORK(mm_kevent_upload_work_thread, mm_kevent_upload_jobs);
static struct workqueue_struct *mm_kevent_wq;
static int mm_kevent_len = 0;

struct mm_kevent {
	struct list_head head;
	enum OPLUS_MM_DIRVER_FB_EVENT_MODULE module;
	struct mutex lock;
	int count;
	int count_total;
	u32 count_limit;
	int rate_limit_ms;
	ktime_t first;
	ktime_t last;
	ktime_t last_upload;
	struct delayed_work dwork;
	char *payload;
	char name[0];
};

static void mm_kevent_upload_jobs(struct work_struct *work)
{
	struct mm_kevent *kevent = NULL, *n = NULL;
	unsigned char payload[150] = "";
	int cnt;

	list_for_each_entry_safe(kevent, n, &mm_kevent_list, head) {
		if (ktime_before(kevent->last, kevent->last_upload))
			continue;

		if (kevent->count_limit && (kevent->count_total % kevent->count_limit == 0)) {
			kevent->count_limit <<= 1;
			if (kevent->count_limit > 4096)
				kevent->count_limit = 4096;
		} else if (!kevent->rate_limit_ms || (kevent->rate_limit_ms &&
			   ktime_before(ktime_get(), ktime_add_ms(kevent->last_upload, kevent->rate_limit_ms)))) {
				continue;
		}

		mutex_lock(&kevent->lock);
		cnt = 0;
		#define PAYLOAD(fmt, ...) \
			if (sizeof(payload) > cnt) \
				cnt += scnprintf(payload + cnt, sizeof(payload) - cnt, fmt, ##__VA_ARGS__);
		PAYLOAD("%s$$EventID@@%d$$", kevent->name, OPLUS_MM_DIRVER_FB_EVENT_ID_ERROR);
		PAYLOAD("CT@@%d$$", kevent->count);
		PAYLOAD("FT@@%lu$$", ktime_to_ms(kevent->first) / 1000);
		PAYLOAD("ET@@%lu$$", ktime_to_ms(kevent->last) / 1000);
		PAYLOAD("MSG@@%s", kevent->payload ? kevent->payload : "NULL");

		if (kevent->payload) {
			kfree(kevent->payload);
			kevent->payload = NULL;
		}

		kevent->count = 0;
		kevent->last_upload = ktime_get();
		mutex_unlock(&kevent->lock);
		upload_mm_kevent_fb_data(kevent->module, payload);
	}

	mod_delayed_work(mm_kevent_wq, &mm_kevent_upload_work_thread, 60 * 60 * HZ);
}

void mm_kevent_upload_recv_user(int type, int flags, char* data) {
	printk(KERN_INFO "mm_kevent fb recv user type=0x%x, flags=0x%x, data=%s\n",
		type, flags, data);
    //#ifdef OPLUS_NETLINK_MM_KEVENT_TEST

	if ((type == OPLUS_MM_DIRVER_FB_EVENT_MODULE_DISPLAY)
		&& (flags & OPLUS_NETLINK_MM_DBG_LV2)) {
		printk(KERN_INFO "mm_kevent: send to user \n");
		upload_mm_kevent_fb_data(OPLUS_MM_DIRVER_FB_EVENT_MODULE_DISPLAY, data);
	}
    //#endif

}

int mm_kevent_init(void)
{
	mm_kevent_wq = create_workqueue("mm_kevent");
	if (!mm_kevent_wq)
		return -ENOMEM;
	queue_delayed_work(mm_kevent_wq, &mm_kevent_upload_work_thread, 0);

    mm_kevent_set_recv_user(mm_kevent_upload_recv_user);

	return 0;
}

void mm_kevent_deinit(void)
{
	if (mm_kevent_wq)
		destroy_workqueue(mm_kevent_wq);

	mm_kevent_set_recv_user(NULL);
}

int upload_mm_kevent_fb_data(enum OPLUS_MM_DIRVER_FB_EVENT_MODULE module, unsigned char *payload) {
	struct mm_kevent_packet *user_msg_info;
	char log_tag[32] = DP_FB_EVENT;
	char event_id_display[20] = "20181203";
	char event_id_audio[20] = "20181205";
	void* buffer = NULL;
	int len, size;

	mutex_lock(&mm_kevent_lock);

	len = strlen(payload);

	size = sizeof(struct mm_kevent_packet) + len + 1;
	printk(KERN_INFO "kevent_send_to_user:size=%d\n", size);

	buffer = kmalloc(size, GFP_ATOMIC);
	memset(buffer, 0, size);
	user_msg_info = (struct mm_kevent_packet *)buffer;
	user_msg_info->type = 1;

	memcpy(user_msg_info->tag, log_tag, strlen(log_tag) + 1);
	if (OPLUS_MM_DIRVER_FB_EVENT_MODULE_DISPLAY == module) {
		memcpy(user_msg_info->event_id, event_id_display, strlen(event_id_display) + 1);
	} else {
		memcpy(user_msg_info->event_id, event_id_audio, strlen(event_id_audio) + 1);
	}

	user_msg_info->len = len + 1;
	memcpy(user_msg_info->data, payload, len + 1);

	mm_kevent_send_to_user(user_msg_info);
	//msleep(20);
	kfree(buffer);
	mutex_unlock(&mm_kevent_lock);
	return 0;
}

static void mm_kevent_upload_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct mm_kevent *new_kevent = container_of(dwork, struct mm_kevent, dwork);
	struct mm_kevent *kevent = NULL, *n = NULL;
	bool found = false;

	list_for_each_entry_safe(kevent, n, &mm_kevent_list, head) {
		if (!strcmp(kevent->name, new_kevent->name)) {
			found = true;
			break;
		}
	}

	if (!found) {
		if (mm_kevent_len > 200) {
			unsigned char payload[150] = "";
			pr_err("mm_kevent large than 200");
			scnprintf(payload, sizeof(payload), "OutOfEvent$$EventID@@420$$MSG@@%s", new_kevent->name);
			upload_mm_kevent_fb_data(kevent->module, payload);
			goto done;
		}

		kevent = new_kevent;
		kevent->count = 1;
		kevent->count_total = 1;
		new_kevent = NULL;
		mm_kevent_len++;
		list_add_tail(&kevent->head, &mm_kevent_list);
		goto done;
	}

	if (WARN_ON(!kevent))
		goto done;

	mutex_lock(&kevent->lock);
	kevent->count++;
	kevent->count_total++;
	kevent->last = new_kevent->first;
	kfree(kevent->payload);
	kevent->payload = new_kevent->payload;
	new_kevent->payload = NULL;
	mutex_unlock(&kevent->lock);
done:
	mm_kevent_upload_jobs(NULL);
	if (new_kevent)
		kfree(new_kevent->payload);
	kfree(new_kevent);
}

int mm_kevent_upload(enum OPLUS_MM_DIRVER_FB_EVENT_MODULE module, const char *name,
		     int rate_limit_ms, char *payload)
{
	struct mm_kevent *kevent = NULL;
	int size;

	if (!mm_kevent_wq)
		return -EINVAL;

	size = strlen(name) + sizeof(*kevent) + 1;
	kevent = kzalloc(size, GFP_ATOMIC);
	if (!kevent)
		return -ENOMEM;

	kevent->count_limit = 1;
	kevent->first = ktime_get();
	kevent->last = ktime_get();
	kevent->last_upload = ktime_get();
	kevent->rate_limit_ms = rate_limit_ms;
	memcpy(kevent->name, name, strlen(name) + 1);
	kevent->payload = kmemdup(payload, strlen(payload) + 1, GFP_ATOMIC);
	mutex_init(&kevent->lock);
	INIT_DELAYED_WORK(&kevent->dwork, mm_kevent_upload_work);
	queue_delayed_work(mm_kevent_wq, &kevent->dwork, 0);

	return 0;
}
