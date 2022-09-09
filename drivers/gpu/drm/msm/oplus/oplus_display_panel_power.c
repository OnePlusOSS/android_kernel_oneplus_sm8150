/***************************************************************
** Copyright (C),  2020,  OPLUS Mobile Comm Corp.,  Ltd
** VENDOR_EDIT
** File : oplus_display_panel_power.c
** Description : oplus display panel power control
** Version : 1.0
** Date : 2020/06/13
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**  Li.Sheng       2020/06/13        1.0           Build this moudle
******************************************************************/
#include "oplus_display_panel_power.h"

PANEL_VOLTAGE_BAK panel_vol_bak[PANEL_VOLTAGE_ID_MAX] = {{0}, {0}, {2, 0, 1, 2, ""}};
u32 panel_pwr_vg_base = 0;
int oplus_request_power_status = OPLUS_DISPLAY_POWER_OFF;
DEFINE_MUTEX(oplus_power_status_lock);

static int oplus_panel_find_vreg_by_name(const char *name)
{
	int count = 0, i = 0;
	struct dsi_vreg *vreg = NULL;
	struct dsi_regulator_info *dsi_reg = NULL;
	struct dsi_display *display = get_main_display();

	if (!display) {
		return -ENODEV;
	}

	if (!display->panel) {
		return -EINVAL;
	}

	dsi_reg = &display->panel->power_info;
        count = dsi_reg->count;
	for (i = 0; i < count; i++) {
		vreg = &dsi_reg->vregs[i];
		pr_err("%s : find  %s", __func__, vreg->vreg_name);
		if (!strcmp(vreg->vreg_name, name)) {
			pr_err("%s : find the vreg %s", __func__, name);
			return i;
		} else {
			continue;
		}
	}
	pr_err("%s : dose not find the vreg [%s]", __func__, name);

	return -EINVAL;
}


int dsi_panel_parse_panel_power_cfg(struct dsi_panel *panel)
{
	int rc = 0, i = 0;
	const char *name_vddi = NULL;
	const char *name_vddr = NULL;
	u32 *panel_vol = NULL;
	struct dsi_parser_utils *utils = &panel->utils;

	pr_err("[%s] \n", __func__);

	if (!strcmp(panel->type, "primary")) {
		panel_vol = &panel_vol_bak[PANEL_VOLTAGE_ID_VDDI].voltage_id;
		rc = utils->read_u32_array(utils->data, "qcom,panel_voltage_vddi",
					panel_vol, PANEL_VOLTAGE_VALUE_COUNT);
		if (rc) {
			pr_err("[%s] failed to parse panel_voltage vddi\n", panel->name);
			goto error;
		}

		rc = utils->read_string(utils->data, "qcom,panel_voltage_vddi_name",
					&name_vddi);
		if (rc) {
			pr_err("[%s] failed to parse vddi name\n", panel->name);
			goto error;
		} else {
			pr_err("[%s] surccess to parse vddi name %s\n", panel->name, name_vddi);
			strcpy(panel_vol_bak[PANEL_VOLTAGE_ID_VDDI].pwr_name, name_vddi);
		}

		panel_vol = &panel_vol_bak[PANEL_VOLTAGE_ID_VDDR].voltage_id;
		rc = utils->read_u32_array(utils->data, "qcom,panel_voltage_vddr",
					panel_vol, PANEL_VOLTAGE_VALUE_COUNT);
		if (rc) {
			pr_err("[%s] failed to parse panel_voltage vddr\n", panel->name);
			goto error;
		}

		rc = utils->read_string(utils->data, "qcom,panel_voltage_vddr_name",
					&name_vddr);
		if (rc) {
			pr_err("[%s] failed to parse vddr name\n", panel->name);
			goto error;
		} else {
			pr_err("[%s] surccess to parse vddr name %s\n", panel->name, name_vddr);
			strcpy(panel_vol_bak[PANEL_VOLTAGE_ID_VDDR].pwr_name, name_vddr);
		}

		/*add for debug*/
		for(i; i < PANEL_VOLTAGE_ID_MAX; i++) {
			pr_err("[%s] panel_voltage[%d] = %d,%d,%d,%d,%s\n", __func__, i, panel_vol_bak[i].voltage_id,
				panel_vol_bak[i].voltage_min, panel_vol_bak[i].voltage_current,
				panel_vol_bak[i].voltage_max, panel_vol_bak[i].pwr_name);
		}
	}

	error:
		return rc;
}

static u32 oplus_panel_update_current_voltage(u32 id)
{
	int vol_current = 0, pwr_id = 0;
	struct dsi_vreg *dsi_reg = NULL;
	struct dsi_regulator_info *dsi_reg_info = NULL;
	struct dsi_display *display = get_main_display();

	if (!display) {
		return -ENODEV;
	}
	if (!display->panel || !display->drm_conn) {
		return -EINVAL;
	}

	dsi_reg_info = &display->panel->power_info;
	pwr_id = oplus_panel_find_vreg_by_name(panel_vol_bak[id].pwr_name);
	if (pwr_id < 0) {
		pr_err("%s: can't find the pwr_id, please check the vreg name\n", __func__);
		return pwr_id;
	}

	dsi_reg = &dsi_reg_info->vregs[pwr_id];
	if (!dsi_reg) {
		return -EINVAL;
	}

	vol_current = regulator_get_voltage(dsi_reg->vreg);

	return vol_current;
}

int oplus_display_panel_get_pwr(void *data)
{
	int ret = 0;
	struct panel_vol_get *panel_vol = data;
	int pid = (panel_vol->panel_id - 1);
	pr_err("%s : [id] = %d\n", __func__, pid);

	panel_vol->panel_min = panel_vol_bak[pid].voltage_min;
	panel_vol->panel_max = panel_vol_bak[pid].voltage_max;
	panel_vol->panel_cur = panel_vol_bak[pid].voltage_current;

	if (pid < PANEL_VOLTAGE_ID_VG_BASE &&
		pid >= PANEL_VOLTAGE_ID_VDDI) {
		ret = oplus_panel_update_current_voltage(pid);
		if (ret < 0) {
			pr_err("%s : update_current_voltage error = %d\n", __func__, ret);
			return ret;
		} else {
			panel_vol->panel_cur = ret;
			pr_err("%s : [id min cur max] = [%u32, %u32, %u32, %u32]\n", __func__,
				pid, panel_vol->panel_min,
				panel_vol->panel_cur, panel_vol->panel_max);
			return 0;
		}
	}

	return ret;
}

int oplus_display_panel_set_pwr(void *data)
{
	struct panel_vol_set *panel_vol = data;
	u32 panel_vol_value = 0, rc = 0, panel_vol_id = 0, pwr_id = 0;
	struct dsi_vreg *dsi_reg = NULL;
	struct dsi_regulator_info *dsi_reg_info = NULL;
	struct dsi_display *display = get_main_display();

	panel_vol_id = ((panel_vol->panel_id & 0x0F)-1);
	panel_vol_value = panel_vol->panel_vol;

	pr_err("debug for %s, id = %d value = %d\n",
		__func__, panel_vol_id, panel_vol_value);

	if (panel_vol_id < 0 || panel_vol_id > PANEL_VOLTAGE_ID_MAX) {
		return -EINVAL;
	}

	if (panel_vol_value < panel_vol_bak[panel_vol_id].voltage_min ||
		panel_vol_id > panel_vol_bak[panel_vol_id].voltage_max)
		return -EINVAL;

	if (!display) {
		return -ENODEV;
	}
	if (!display->panel || !display->drm_conn) {
		return -EINVAL;
	}

	if (panel_vol_id == PANEL_VOLTAGE_ID_VG_BASE) {
		pr_err("%s: set the VGH_L pwr = %d \n", __func__, panel_vol_value);
		panel_pwr_vg_base = panel_vol_value;
		return rc;
	}

	dsi_reg_info = &display->panel->power_info;

	pwr_id = oplus_panel_find_vreg_by_name(panel_vol_bak[panel_vol_id].pwr_name);
	if (pwr_id < 0) {
		pr_err("%s: can't find the vreg name, please re-check vreg name: %s \n",
			__func__, panel_vol_bak[panel_vol_id].pwr_name);
		return pwr_id;
	}
	dsi_reg = &dsi_reg_info->vregs[pwr_id];

	rc = regulator_set_voltage(dsi_reg->vreg, panel_vol_value, panel_vol_value);
	if (rc) {
		pr_err("Set voltage(%s) fail, rc=%d\n",
			 dsi_reg->vreg_name, rc);
		return -EINVAL;
	}

	return rc;
}

int __oplus_display_set_power_status(int status) {
	mutex_lock(&oplus_power_status_lock);
	if(status != oplus_request_power_status) {
		oplus_request_power_status = status;
		printk(KERN_INFO "%s oplus_display_set_power_status = %d\n", __func__, status);
	}
	mutex_unlock(&oplus_power_status_lock);
	return 0;
}

int oplus_display_panel_get_power_status(void *data) {
	uint32_t *power_status = data;

	printk(KERN_DEBUG "oplus_display_get_power_status = %d\n", get_oplus_display_power_status());
	(*power_status) = get_oplus_display_power_status();

	return 0;
}

int oplus_display_panel_set_power_status(void *data) {
	uint32_t *temp_save = data;

	__oplus_display_set_power_status((*temp_save));

	return 0;
}
