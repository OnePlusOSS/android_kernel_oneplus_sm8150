/***************************************************************
** Copyright (C),  2020,  OPLUS Mobile Comm Corp.,  Ltd
** VENDOR_EDIT
** File : oplus_ffl.c
** Description : oplus ffl feature
** Version : 1.0
** Date : 2020/04/23
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**   Qianxu         2020/04/23        1.0           Build this moudle
******************************************************************/

#include <linux/mutex.h>
#include "dsi_display.h"
#include "oplus_dsi_support.h"
#include "oplus_onscreenfingerprint.h"
#include "oplus_mm_kevent_fb.h"


#define FFL_LEVEL_START 2
#define FFL_LEVEL_END  236
#define FFLUPRARE  1
#define BACKUPRATE 6
#define FFL_PENDING_END 600
#define FFL_EXIT_CONTROL 0
#define FFL_TRIGGLE_CONTROL 1
#define FFL_EXIT_FULLY_CONTROL 2

bool oplus_ffl_trigger_finish = true;
bool ffl_work_running = false;
int is_ffl_enable = FFL_EXIT_CONTROL;
struct task_struct *oplus_ffl_thread;
struct kthread_worker oplus_ffl_worker;
struct kthread_work oplus_ffl_work;
static DEFINE_MUTEX(oplus_ffl_lock);


void oplus_ffl_set(int enable)
{
	unsigned char payload[150] = "";

	mutex_lock(&oplus_ffl_lock);

	if (enable != is_ffl_enable) {
		pr_debug("set_ffl_setting need change is_ffl_enable\n");
		is_ffl_enable = enable;

		if ((is_ffl_enable == FFL_TRIGGLE_CONTROL) && ffl_work_running) {
			oplus_ffl_trigger_finish = false;
			kthread_queue_work(&oplus_ffl_worker, &oplus_ffl_work);
		}
	}

	mutex_unlock(&oplus_ffl_lock);

	if ((is_ffl_enable == FFL_TRIGGLE_CONTROL) && ffl_work_running) {
		scnprintf(payload, sizeof(payload), "NULL$$EventID@@%d$$fflset@@%d",
			OPLUS_MM_DIRVER_FB_EVENT_ID_FFLSET, enable);
		upload_mm_kevent_fb_data(OPLUS_MM_DIRVER_FB_EVENT_MODULE_DISPLAY, payload);
	}
}

int oplus_display_panel_get_ffl(void *buf)
{
	unsigned int *enable = buf;

	(*enable) = is_ffl_enable;

	return 0;
}

int oplus_display_panel_set_ffl(void *buf)
{
	unsigned int *enable = buf;

	printk(KERN_INFO "%s oplus_set_ffl_setting = %d\n", __func__, (*enable));
	oplus_ffl_set(*enable);

	return 0;
}

void oplus_ffl_setting_thread(struct kthread_work *work)
{
	struct dsi_display *display = get_main_display();
	int index = 0;
	int pending = 0;
	int system_backlight_target;
	int rc;

	if (get_oplus_display_power_status() == OPLUS_DISPLAY_POWER_OFF) {
		return;
	}

	if (is_ffl_enable != FFL_TRIGGLE_CONTROL) {
		return;
	}

	if (!display || !display->panel) {
		pr_err("failed to find display panel \n");
		return;
	}

	if (!ffl_work_running) {
		return;
	}

	rc = dsi_display_clk_ctrl(display->dsi_clk_handle,
			DSI_CORE_CLK, DSI_CLK_ON);

	if (rc) {
		pr_err("[%s] failed to enable DSI core clocks, rc=%d\n",
			display->name, rc);
		return;
	}

	for (index = FFL_LEVEL_START; index < FFL_LEVEL_END;
		index = index + FFLUPRARE) {
		if ((is_ffl_enable == FFL_EXIT_CONTROL) ||
			(is_ffl_enable == FFL_EXIT_FULLY_CONTROL) ||
			!ffl_work_running) {
			break;
		}

		/*
		 * On Onscreenfingerprint mode, max backlight level should be FFL_FP_LEVEL
		 */
		if (display->panel->is_hbm_enabled && index > FFL_FP_LEVEL) {
			break;
		}

		mutex_lock(&display->panel->panel_lock);
		dsi_panel_set_backlight(display->panel, index);
		mutex_unlock(&display->panel->panel_lock);
		usleep_range(1000, 1100);
	}

	for (pending = 0; pending <= FFL_PENDING_END; pending++) {
		if ((is_ffl_enable == FFL_EXIT_CONTROL) ||
			(is_ffl_enable == FFL_EXIT_FULLY_CONTROL) ||
			!ffl_work_running) {
			break;
		}

		usleep_range(8000, 8100);
	}

	system_backlight_target = display->panel->bl_config.bl_level;

	if (index < system_backlight_target) {
		for (index; index < system_backlight_target; index = index + BACKUPRATE) {
			if ((is_ffl_enable == FFL_EXIT_FULLY_CONTROL) ||
				!ffl_work_running) {
				break;
			}

			mutex_lock(&display->panel->panel_lock);
			dsi_panel_set_backlight(display->panel, index);
			mutex_unlock(&display->panel->panel_lock);
			usleep_range(6000, 6100);
		}

	} else if (index > system_backlight_target) {
		for (index; index > system_backlight_target; index = index - BACKUPRATE) {
			if ((is_ffl_enable == FFL_EXIT_FULLY_CONTROL) ||
				!ffl_work_running) {
				break;
			}

			mutex_lock(&display->panel->panel_lock);
			dsi_panel_set_backlight(display->panel, index);
			mutex_unlock(&display->panel->panel_lock);
			usleep_range(6000, 6100);
		}
	}

	mutex_lock(&display->panel->panel_lock);
	system_backlight_target = display->panel->bl_config.bl_level;
	dsi_panel_set_backlight(display->panel, system_backlight_target);
	oplus_ffl_trigger_finish = true;
	mutex_unlock(&display->panel->panel_lock);

	rc = dsi_display_clk_ctrl(display->dsi_clk_handle,
			DSI_CORE_CLK, DSI_CLK_OFF);

	if (rc) {
		pr_err("[%s] failed to disable DSI core clocks, rc=%d\n",
			display->name, rc);
	}
}

void oplus_start_ffl_thread(void)
{
	mutex_lock(&oplus_ffl_lock);

	ffl_work_running = true;

	if (is_ffl_enable == FFL_TRIGGLE_CONTROL) {
		oplus_ffl_trigger_finish = false;
		kthread_queue_work(&oplus_ffl_worker, &oplus_ffl_work);
	}

	mutex_unlock(&oplus_ffl_lock);
}

void oplus_stop_ffl_thread(void)
{
	mutex_lock(&oplus_ffl_lock);

	oplus_ffl_trigger_finish = true;
	ffl_work_running = false;
	kthread_flush_worker(&oplus_ffl_worker);

	mutex_unlock(&oplus_ffl_lock);
}

int oplus_ffl_thread_init(void)
{
	kthread_init_worker(&oplus_ffl_worker);
	kthread_init_work(&oplus_ffl_work, &oplus_ffl_setting_thread);
	oplus_ffl_thread = kthread_run(kthread_worker_fn,
			&oplus_ffl_worker, "oplus_ffl");

	if (IS_ERR(oplus_ffl_thread)) {
		pr_err("fail to start oplus_ffl_thread\n");
		oplus_ffl_thread = NULL;
		return -1;
	}

	return 0;
}

void oplus_ffl_thread_exit(void)
{
	if (oplus_ffl_thread) {
		is_ffl_enable = FFL_EXIT_FULLY_CONTROL;
		kthread_flush_worker(&oplus_ffl_worker);
		kthread_stop(oplus_ffl_thread);
		oplus_ffl_thread = NULL;
	}
}

