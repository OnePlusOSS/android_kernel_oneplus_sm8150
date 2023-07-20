/***************************************************
 * File:touch.c
 * VENDOR_EDIT
 * Copyright (c)  2008- 2030  Oplus Mobile communication Corp.ltd.
 * Description:
 *             tp dev
 * Version:1.0:
 * Date created:2016/09/02
 * TAG: BSP.TP.Init
*/

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/serio.h>
#include <linux/init.h>
#include "oplus_touchscreen/tp_devices.h"
#include "oplus_touchscreen/touchpanel_common.h"
#include <soc/oplus/system/oplus_project.h>
#include <soc/oplus/device_info.h>
#include "touch.h"

#define MAX_LIMIT_DATA_LENGTH         100

extern char *saved_command_line;
int g_tp_dev_vendor = TP_UNKNOWN;
int g_tp_prj_id = 0;
static int lcd_id = 0;

/*if can not compile success, please update vendor/oplus_touchsreen*/
struct tp_dev_name tp_dev_names[] = {
    {TP_OFILM, "OFILM"},
    {TP_BIEL, "BIEL"},
    {TP_TRULY, "TRULY"},
    {TP_BOE, "BOE"},
    {TP_G2Y, "G2Y"},
    {TP_TPK, "TPK"},
    {TP_JDI, "JDI"},
    {TP_TIANMA, "TIANMA"},
    {TP_SAMSUNG, "SAMSUNG"},
    {TP_DSJM, "DSJM"},
    {TP_BOE_B8, "BOEB8"},
    {TP_UNKNOWN, "UNKNOWN"},
};

#define GET_TP_DEV_NAME(tp_type) ((tp_dev_names[tp_type].type == (tp_type))?tp_dev_names[tp_type].name:"UNMATCH")

static int get_cmdlinelcd_id(void)
{
	if (strstr(saved_command_line, "panel_type=normal")) {
		lcd_id = 0;
	} else if (strstr(saved_command_line, "panel_type=x_talk")) {
		lcd_id = 1;
	} else if (strstr(saved_command_line, "panel_type=tx")) {
		lcd_id = 1;
	}
	pr_err("[TP] %s enter, lcd_id:%d\n", __func__, lcd_id);
	return 0;
}

bool __init tp_judge_ic_match(char *tp_ic_name) {
    return true;
}
 bool tp_judge_ic_match_commandline(struct panel_info *panel_data)
{
    int prj_id = 0;
    int i = 0;
    prj_id = get_project();
    pr_err("[TP] boot_command_line = %s \n", saved_command_line);
    for(i = 0; i<panel_data->project_num; i++){
        if(prj_id == panel_data->platform_support_project[i]){
            g_tp_prj_id = panel_data->platform_support_project_dir[i];
            if(strstr(saved_command_line, panel_data->platform_support_commandline[i])||strstr("default_commandline", panel_data->platform_support_commandline[i]) ){
                pr_err("[TP] Driver match the project\n");
                return true;
            }
            else{
                continue;
            }
        }
    }
    pr_err("[TP] Driver does not match the project\n");
    pr_err("Lcd module not found\n");
    return false;
}

int tp_util_get_vendor(struct hw_resource *hw_res, struct panel_info *panel_data) {
    char* vendor = NULL;
    panel_data->test_limit_name = kzalloc(MAX_LIMIT_DATA_LENGTH, GFP_KERNEL);
    if (panel_data->test_limit_name == NULL) {
        pr_err("[TP]panel_data.test_limit_name kzalloc error\n");
    }

    if (strstr(saved_command_line, "dsi_oplus19696boe_nt36672c_1080_2400_90fps_vid")) {
        panel_data->tp_type = TP_BOE;
    }
    if (strstr(saved_command_line, "dsi_oplus19696jdi_nt36672c_1080_2400_90fps_vid")) {
        panel_data->tp_type = TP_JDI;
    }
    if (panel_data->tp_type == TP_UNKNOWN) {
        pr_err("[TP]%s type is unknown\n", __func__);
        return 0;
    }
	pr_err("[TP]  type is %d \n", panel_data->tp_type);
    vendor = GET_TP_DEV_NAME(panel_data->tp_type);
    if (vendor == NULL) {
        pr_err("[TP]%s can not get vendor\n", __func__);
        return 0;
    }
    strcpy(panel_data->manufacture_info.manufacture, vendor);
    /*support for 20261 ft3518*/
    if (strstr(saved_command_line, "dsi_oplus20261samsung_ams643xf01_1080_2400_cmd_display")) {
        snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
            "tp/%d/FW_%s_%s_NEW.img",
            g_tp_prj_id, panel_data->chip_name, vendor);
    } else if (strstr(saved_command_line, "dsi_oplus19365samsung_ams643xf01_1080_2400_cmd_display")) {
        snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
            "tp/%d/FW_%s_%s_NEW.img",
            g_tp_prj_id, panel_data->chip_name, vendor);
    } else if (strstr(saved_command_line, "dsi_oplus18097samsung")) {
        snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
            "tp/%d/FW_%s_%s_TMP.img",
            g_tp_prj_id, panel_data->chip_name, vendor);
    } else {
        snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
            "tp/%d/FW_%s_%s.img",
            g_tp_prj_id, panel_data->chip_name, vendor);
    }

    if (panel_data->test_limit_name) {
        snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
        "tp/%d/LIMIT_%s_%s.img",
        g_tp_prj_id, panel_data->chip_name, vendor);
    }
    if(g_tp_prj_id == 0x206B1){
	pr_err("[TP]project is 0x206B1\n", __func__);
	snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
              "tp/%x/FW_%s_%s.img",
              g_tp_prj_id, panel_data->chip_name, vendor);
      if (panel_data->test_limit_name) {
        snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
              "tp/%x/LIMIT_%s_%s.img",
        g_tp_prj_id, panel_data->chip_name, vendor);
      }
	pr_err("[TP]panel_data fw name is %s\n", panel_data->fw_name);
    }

	if (strstr(saved_command_line, "evt_panel") && g_tp_prj_id == 19081) {
		snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
			"tp/%d/FW_%s_%s_OLD.img", g_tp_prj_id, panel_data->chip_name, vendor);
		if (panel_data->test_limit_name) {
			snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
				"tp/%d/LIMIT_%s_%s_OLD.img", g_tp_prj_id, panel_data->chip_name, vendor);
		}
	}

	if (g_tp_prj_id == 18865 || g_tp_prj_id == 19801 || g_tp_prj_id == 19863 || g_tp_prj_id == 19861) {
		pr_err("[TP] %s project is :%d\n", __func__, g_tp_prj_id);
		get_cmdlinelcd_id();
		if (lcd_id) {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
			"tp/%d/FW_%s_%s_NEW.img", g_tp_prj_id, panel_data->chip_name, vendor);

			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
				"tp/%d/LIMIT_%s_%s_NEW.img", g_tp_prj_id, panel_data->chip_name, vendor);
			}
		} else {
			snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
			"tp/%d/FW_%s_%s.img", g_tp_prj_id, panel_data->chip_name, vendor);

			if (panel_data->test_limit_name) {
				snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
				"tp/%d/LIMIT_%s_%s.img", g_tp_prj_id, panel_data->chip_name, vendor);
			}
		}
	}

    if (strstr(saved_command_line, "dsi_oplus19365samsung")) {
        memcpy(panel_data->manufacture_info.version, "0xbd3320000", 11);
    }

    if (strstr(saved_command_line, "dsi_oplus19567samsung_amb655uv01_1080_2400_cmd_display")) {
        memcpy(panel_data->manufacture_info.version, "0xad1220000", 11);
    }
    if (strstr(saved_command_line, "dsi_oplus20261samsung_ams643xf01_1080_2400_cmd_display")) {
        memcpy(panel_data->manufacture_info.version, "0xbd3650000", 11);
    }
    if (g_tp_prj_id == 0x206B1){
        memcpy(panel_data->manufacture_info.version, "0xbd3650000", 11);
    }

	if ((g_tp_prj_id == 20711) || (g_tp_prj_id == 20712) || (g_tp_prj_id == 20713) || (g_tp_prj_id == 20714)) {
        memcpy(panel_data->manufacture_info.version, "focalt_", 7);
	}

    if (panel_data->tp_type == TP_DSJM) {
        memcpy(panel_data->manufacture_info.version, "HX_DSJM", 7);
        panel_data->firmware_headfile.firmware_data = FW_18621_HX83112A_NF_DSJM;
        panel_data->firmware_headfile.firmware_size = sizeof(FW_18621_HX83112A_NF_DSJM);
    } else if ( panel_data->tp_type == TP_BOE ) {
        memcpy(panel_data->manufacture_info.version, "NVT_BOE", 7);
        panel_data->firmware_headfile.firmware_data = FW_17951_NT36525C_BOE;
        panel_data->firmware_headfile.firmware_size = sizeof(FW_17951_NT36525C_BOE);
    }else if(panel_data->tp_type == TP_JDI){
        memcpy(panel_data->manufacture_info.version, "NVT_JDI", 7);
        panel_data->firmware_headfile.firmware_data = FW_17951_NT36525C_JDI;
        panel_data->firmware_headfile.firmware_size = sizeof(FW_17951_NT36525C_JDI);
    }
    panel_data->manufacture_info.fw_path = panel_data->fw_name;

    pr_info("[TP]vendor:%s fw:%s limit:%s\n",
        vendor,
        panel_data->fw_name,
        panel_data->test_limit_name == NULL ? "NO Limit" : panel_data->test_limit_name);

    return 0;
}

int reconfig_power_control(struct touchpanel_data *ts) {
    int ret = 0;

    if (ts->panel_data.tp_type == TP_JDI) {
        ts->hw_res.TX_NUM = 16;
        ts->hw_res.RX_NUM = 36;
        pr_info("[TP] Use JDI TX,RX=[%d],[%d]\n", ts->hw_res.TX_NUM, ts->hw_res.RX_NUM);
    }else{
        pr_info("[TP] Use BOE TX,RX=[%d],[%d]\n", ts->hw_res.TX_NUM, ts->hw_res.RX_NUM);
    }
    return ret;
}
