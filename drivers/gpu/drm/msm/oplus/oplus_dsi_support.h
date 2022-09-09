/***************************************************************
** Copyright (C),  2018,  OPLUS Mobile Comm Corp.,  Ltd
** VENDOR_EDIT
** File : oplus_dsi_support.h
** Description : display driver private management
** Version : 1.0
** Date : 2018/03/17
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**   Hu.Jie          2018/03/17        1.0           Build this moudle
******************************************************************/
#ifndef _OPLUS_DSI_SUPPORT_H_
#define _OPLUS_DSI_SUPPORT_H_

#include <linux/err.h>
#include <linux/string.h>
#include <linux/notifier.h>

/* A hardware display blank change occurred */
#define OPLUS_DISPLAY_EVENT_BLANK			0x01

/* A hardware display blank early change occurred */
#define OPLUS_DISPLAY_EARLY_EVENT_BLANK		0x02


enum oplus_display_support_list {
	OPLUS_SAMSUNG_S6E3FC2X01_DISPLAY_FHD_DSC_CMD_PANEL = 0,
	OPLUS_SAMSUNG_S6E8FC1X01_DISPALY_FHD_PLUS_VIDEO_PANEL = 1,
	OPLUS_BOE_HX83112A_FHD_PLUS_VIDEO_PANEL = 2,
	OPLUS_HUAXING_HX83112A_DISPALY_FHD_PLUS_VIDEO_PANEL = 3,
	OPLUS_HLT_HX83112A_DISPALY_FHD_PLUS_VIDEO_PANEL = 4,
	OPLUS_TM_NT36672A_DISPALY_FHD_PLUS_VIDEO_PANEL = 5,
	OPLUS_SAMSUNG_AMS644VK01_DISPALY_FHD_PLUS_CMD_PANEL = 6,
	OPLUS_SAMSUNG_AMB655UV01_DISPALY_FHD_PLUS_90FPS_PANEL = 7,
	OPLUS_SAMSUNG_AMS643XF01_DISPALY_FHD_PLUS_CMD_PANEL = 8,
	OPLUS_SAMSUNG_AMS641RW01_DISPLAY_FHD_PLUS_CMD_PANEL = 9,
	OPLUS_SAMSUNG_AMS641RW01_DISPLAY_FHD_PLUS_61FPS_PANEL = 10,
	OPLUS_SAMSUNG_AMS655UE01_DISPLAY_FHD_PLUS_61FPS_PANEL = 11,
	OPLUS_SAMSUNG_AMS641RW16_DISPLAY_FHD_PLUS_CMD_PANEL = 12,
	OPLUS_SAMSUNG_SOFEF03F_M_DISPLAY_FHD_DSC_CMD_PANEL = 13,
	OPLUS_DISPLAY_UNKNOW,
};

enum oplus_display_power_status {
	OPLUS_DISPLAY_POWER_OFF = 0,
	OPLUS_DISPLAY_POWER_DOZE,
	OPLUS_DISPLAY_POWER_ON,
	OPLUS_DISPLAY_POWER_DOZE_SUSPEND,
	OPLUS_DISPLAY_POWER_ON_UNKNOW,
};

enum oplus_display_scene {
	OPLUS_DISPLAY_NORMAL_SCENE = 0,
	OPLUS_DISPLAY_NORMAL_HBM_SCENE,
	OPLUS_DISPLAY_AOD_SCENE,
	OPLUS_DISPLAY_AOD_HBM_SCENE,
	OPLUS_DISPLAY_UNKNOW_SCENE,
};

enum oplus_display_feature {
	OPLUS_DISPLAY_HDR = 0,
	OPLUS_DISPLAY_SEED,
	OPLUS_DISPLAY_HBM,
	OPLUS_DISPLAY_LBR,
	OPLUS_DISPLAY_AOD,
	OPLUS_DISPLAY_ULPS,
	OPLUS_DISPLAY_ESD_CHECK,
	OPLUS_DISPLAY_DYNAMIC_MIPI,
	OPLUS_DISPLAY_PARTIAL_UPDATE,
	OPLUS_DISPLAY_FEATURE_MAX,
};

typedef struct panel_serial_info
{
	int reg_index;
	uint64_t year;
	uint64_t month;
	uint64_t day;
	uint64_t hour;
	uint64_t minute;
	uint64_t second;
	uint64_t reserved[2];
} PANEL_SERIAL_INFO;


typedef struct oplus_display_notifier_event {
	enum oplus_display_power_status status;
	void *data;
}OPLUS_DISPLAY_NOTIFIER_EVENT;

int oplus_display_register_client(struct notifier_block *nb);

int oplus_display_unregister_client(struct notifier_block *nb);

void notifier_oplus_display_early_status(enum oplus_display_power_status power_status);

void notifier_oplus_display_status(enum oplus_display_power_status power_status);

bool is_oplus_correct_display(enum oplus_display_support_list lcd_name);

bool is_silence_reboot(void);

int set_oplus_display_vendor(const char * display_name);

void set_oplus_display_power_status(enum oplus_display_power_status power_status);

enum oplus_display_power_status get_oplus_display_power_status(void);

void set_oplus_display_scene(enum oplus_display_scene display_scene);

enum oplus_display_scene get_oplus_display_scene(void);

bool is_oplus_display_support_feature(enum oplus_display_feature feature_name);

int oplus_display_get_resolution(unsigned int *xres, unsigned int *yres);

#endif /* _OPLUS_DSI_SUPPORT_H_ */

