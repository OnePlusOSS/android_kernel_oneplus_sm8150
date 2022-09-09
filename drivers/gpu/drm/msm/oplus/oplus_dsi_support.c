/***************************************************************
** Copyright (C),  2018,  OPLUS Mobile Comm Corp.,  Ltd
** VENDOR_EDIT
** File : oplus_dsi_support.c
** Description : display driver private management
** Version : 1.0
** Date : 2018/03/17
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**   Hu.Jie          2018/03/17        1.0           Build this moudle
******************************************************************/
#include "oplus_dsi_support.h"
#include <soc/oplus/boot_mode.h>
#include <soc/oplus/system/oplus_project.h>
#include <soc/oplus/device_info.h>
#include <linux/notifier.h>
#include <linux/module.h>

enum oplus_display_support_list  oplus_display_vendor =
	OPLUS_DISPLAY_UNKNOW;
static enum oplus_display_power_status oplus_display_status =
	OPLUS_DISPLAY_POWER_OFF;
static enum oplus_display_scene oplus_siaplay_save_scene =
	OPLUS_DISPLAY_NORMAL_SCENE;

static BLOCKING_NOTIFIER_HEAD(oplus_display_notifier_list);

int oplus_display_register_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&oplus_display_notifier_list,
			nb);
}
EXPORT_SYMBOL(oplus_display_register_client);


int oplus_display_unregister_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&oplus_display_notifier_list,
			nb);
}
EXPORT_SYMBOL(oplus_display_unregister_client);

static int oplus_display_notifier_call_chain(unsigned long val, void *v)
{
	return blocking_notifier_call_chain(&oplus_display_notifier_list, val,
			v);
}

bool is_oplus_correct_display(enum oplus_display_support_list lcd_name)
{
	return (oplus_display_vendor == lcd_name ? true : false);
}

bool is_silence_reboot(void)
{
	if ((MSM_BOOT_MODE__SILENCE == get_boot_mode())
		|| (MSM_BOOT_MODE__SAU == get_boot_mode())) {
		return true;

	} else {
		return false;
	}
}

int set_oplus_display_vendor(const char *display_name)
{
	if (display_name == NULL) {
		return -1;
	}
/* add lcd info begin */
	if (!strcmp(display_name, "dsi_oplus19365samsung_ams644vk01_1080_2400_cmd_display")) {
		oplus_display_vendor = OPLUS_SAMSUNG_AMS644VK01_DISPALY_FHD_PLUS_CMD_PANEL;
		register_device_proc("lcd", "AMS644VK01", "samsung1024");
	} else if (!strcmp(display_name, "dsi_oplus19567samsung_amb655uv01_1080_2400_cmd_display")) {
		oplus_display_vendor = OPLUS_SAMSUNG_AMB655UV01_DISPALY_FHD_PLUS_90FPS_PANEL;
		register_device_proc("lcd", "AMB655UV01", "samsung1024");
	} else if (!strcmp(display_name, "dsi_oplus19365samsung_ams643xf01_1080_2400_cmd_display")) {
		oplus_display_vendor = OPLUS_SAMSUNG_AMS643XF01_DISPALY_FHD_PLUS_CMD_PANEL;
		register_device_proc("lcd", "AMS643XF01", "samsung1024");
	} else if (!strcmp(display_name, "dsi_oplus19031samsung_ams641rw01_1080_2400_cmd_display")) {
		oplus_display_vendor = OPLUS_SAMSUNG_AMS641RW01_DISPLAY_FHD_PLUS_CMD_PANEL;
		register_device_proc("lcd", "AMS641RW01", "samsung1024");
	} else if (!strcmp(display_name, "dsi_oplus19031samsung_ams641rw01_1080_2400_61fps_cmd_display")) {
		oplus_display_vendor = OPLUS_SAMSUNG_AMS641RW01_DISPLAY_FHD_PLUS_61FPS_PANEL;
		register_device_proc("lcd", "AMS641RW01", "samsung1024");
	} else if (!strcmp(display_name, "dsi_oplus19031samsung_ams655ue01_1080_2400_61fps_cmd_display")) {
		oplus_display_vendor = OPLUS_SAMSUNG_AMS655UE01_DISPLAY_FHD_PLUS_61FPS_PANEL;
		register_device_proc("lcd", "AMS655UE01", "samsung1024");
	} else if (!strcmp(display_name, "dsi_oplus19111samsung_ams641rw16_1080_2340_cmd_display")) {
		oplus_display_vendor = OPLUS_SAMSUNG_AMS641RW16_DISPLAY_FHD_PLUS_CMD_PANEL;
		register_device_proc("lcd", "AMS641RW16", "samsung1024");
	} else if (!strcmp(display_name, "dsi_samsung_fhd_plus_dsc_cmd_90fps_display")) {
		oplus_display_vendor = OPLUS_SAMSUNG_SOFEF03F_M_DISPLAY_FHD_DSC_CMD_PANEL;
		register_device_proc("lcd", "SOFEF03F_M", "samsung1024");
	}  else {
		oplus_display_vendor = OPLUS_DISPLAY_UNKNOW;
		pr_err("%s panel vendor info set failed!", __func__);
	}
/* end */
	return 0;
}

void notifier_oplus_display_early_status(enum oplus_display_power_status
	power_status)
{
	int blank;
	OPLUS_DISPLAY_NOTIFIER_EVENT oplus_notifier_data;

	switch (power_status) {
	case OPLUS_DISPLAY_POWER_ON:
		blank = OPLUS_DISPLAY_POWER_ON;
		oplus_notifier_data.data = &blank;
		oplus_notifier_data.status = OPLUS_DISPLAY_POWER_ON;
		oplus_display_notifier_call_chain(OPLUS_DISPLAY_EARLY_EVENT_BLANK,
			&oplus_notifier_data);
		break;

	case OPLUS_DISPLAY_POWER_DOZE:
		blank = OPLUS_DISPLAY_POWER_DOZE;
		oplus_notifier_data.data = &blank;
		oplus_notifier_data.status = OPLUS_DISPLAY_POWER_DOZE;
		oplus_display_notifier_call_chain(OPLUS_DISPLAY_EARLY_EVENT_BLANK,
			&oplus_notifier_data);
		break;

	case OPLUS_DISPLAY_POWER_DOZE_SUSPEND:
		blank = OPLUS_DISPLAY_POWER_DOZE_SUSPEND;
		oplus_notifier_data.data = &blank;
		oplus_notifier_data.status = OPLUS_DISPLAY_POWER_DOZE_SUSPEND;
		oplus_display_notifier_call_chain(OPLUS_DISPLAY_EARLY_EVENT_BLANK,
			&oplus_notifier_data);
		break;

	case OPLUS_DISPLAY_POWER_OFF:
		blank = OPLUS_DISPLAY_POWER_OFF;
		oplus_notifier_data.data = &blank;
		oplus_notifier_data.status = OPLUS_DISPLAY_POWER_OFF;
		oplus_display_notifier_call_chain(OPLUS_DISPLAY_EARLY_EVENT_BLANK,
			&oplus_notifier_data);
		break;

	default:
		break;
	}
}

void notifier_oplus_display_status(enum oplus_display_power_status power_status)
{
	int blank;
	OPLUS_DISPLAY_NOTIFIER_EVENT oplus_notifier_data;

	switch (power_status) {
	case OPLUS_DISPLAY_POWER_ON:
		blank = OPLUS_DISPLAY_POWER_ON;
		oplus_notifier_data.data = &blank;
		oplus_notifier_data.status = OPLUS_DISPLAY_POWER_ON;
		oplus_display_notifier_call_chain(OPLUS_DISPLAY_EVENT_BLANK,
			&oplus_notifier_data);
		break;

	case OPLUS_DISPLAY_POWER_DOZE:
		blank = OPLUS_DISPLAY_POWER_DOZE;
		oplus_notifier_data.data = &blank;
		oplus_notifier_data.status = OPLUS_DISPLAY_POWER_DOZE;
		oplus_display_notifier_call_chain(OPLUS_DISPLAY_EVENT_BLANK,
			&oplus_notifier_data);
		break;

	case OPLUS_DISPLAY_POWER_DOZE_SUSPEND:
		blank = OPLUS_DISPLAY_POWER_DOZE_SUSPEND;
		oplus_notifier_data.data = &blank;
		oplus_notifier_data.status = OPLUS_DISPLAY_POWER_DOZE_SUSPEND;
		oplus_display_notifier_call_chain(OPLUS_DISPLAY_EVENT_BLANK,
			&oplus_notifier_data);
		break;

	case OPLUS_DISPLAY_POWER_OFF:
		blank = OPLUS_DISPLAY_POWER_OFF;
		oplus_notifier_data.data = &blank;
		oplus_notifier_data.status = OPLUS_DISPLAY_POWER_OFF;
		oplus_display_notifier_call_chain(OPLUS_DISPLAY_EVENT_BLANK,
			&oplus_notifier_data);
		break;

	default:
		break;
	}
}

void set_oplus_display_power_status(enum oplus_display_power_status power_status)
{
	oplus_display_status = power_status;
}

enum oplus_display_power_status get_oplus_display_power_status(void)
{
	return oplus_display_status;
}
EXPORT_SYMBOL(get_oplus_display_power_status);

void set_oplus_display_scene(enum oplus_display_scene display_scene)
{
	oplus_siaplay_save_scene = display_scene;
}

enum oplus_display_scene get_oplus_display_scene(void)
{
	return oplus_siaplay_save_scene;
}

EXPORT_SYMBOL(get_oplus_display_scene);

bool is_oplus_display_support_feature(enum oplus_display_feature feature_name)
{
	bool ret = false;

	switch (feature_name) {
	case OPLUS_DISPLAY_HDR:
		ret = false;
		break;

	case OPLUS_DISPLAY_SEED:
		ret = true;
		break;

	case OPLUS_DISPLAY_HBM:
		ret = true;
		break;

	case OPLUS_DISPLAY_LBR:
		ret = true;
		break;

	case OPLUS_DISPLAY_AOD:
		ret = true;
		break;

	case OPLUS_DISPLAY_ULPS:
		ret = false;
		break;

	case OPLUS_DISPLAY_ESD_CHECK:
		ret = true;
		break;

	case OPLUS_DISPLAY_DYNAMIC_MIPI:
		ret = true;
		break;

	case OPLUS_DISPLAY_PARTIAL_UPDATE:
		ret = false;
		break;

	default:
		break;
	}

	return ret;
}


