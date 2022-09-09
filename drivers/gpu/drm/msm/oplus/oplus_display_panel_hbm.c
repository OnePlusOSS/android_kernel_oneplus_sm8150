/***************************************************************
** Copyright (C),  2020,  OPLUS Mobile Comm Corp.,  Ltd
** VENDOR_EDIT
** File : oplus_display_panel_hbm.c
** Description : oplus display panel hbm feature
** Version : 1.0
** Date : 2020/07/06
**
** ------------------------------- Revision History: -----------
**  <author>		<data>		<version >		<desc>
**  Li.Sheng	   2020/07/06		1.0		   Build this feature
******************************************************************/
#include "oplus_display_panel_hbm.h"
#include "oplus_dsi_support.h"
#include "oplus_dc_diming.h"

int hbm_mode = 0;
DEFINE_MUTEX(oplus_hbm_lock);

int dsi_panel_hbm_on(struct dsi_panel *panel)
{
	int rc = 0;

	if (!panel) {
		pr_err("Invalid params\n");
		return -EINVAL;
	}

	mutex_lock(&panel->panel_lock);

	if (!dsi_panel_initialized(panel)) {
		rc = -EINVAL;
		goto error;
	}

	rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_HBM_ON);
	if (rc) {
		pr_err("[%s] failed to send DSI_CMD_HBM_ON cmds, rc=%d\n",
				panel->name, rc);
	}

error:
	mutex_unlock(&panel->panel_lock);
	return rc;
}

int dsi_panel_normal_hbm_on(struct dsi_panel *panel)
{
	int rc = 0;

	if (!panel) {
		pr_err("Invalid params\n");
		return -EINVAL;
	}

	mutex_lock(&panel->panel_lock);

	if (!dsi_panel_initialized(panel)) {
		rc = -EINVAL;
		goto error;
	}

	rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_NORMAL_HBM_ON);
	if (rc) {
		pr_err("[%s] failed to send DSI_CMD_NORMAL_HBM_ON cmds, rc=%d\n",
				panel->name, rc);
	}

error:
	mutex_unlock(&panel->panel_lock);
	return rc;
}

int dsi_panel_hbm_off(struct dsi_panel *panel)
{
	int rc = 0;

	if (!panel) {
		pr_err("Invalid params\n");
		return -EINVAL;
	}

	mutex_lock(&panel->panel_lock);

	if (!dsi_panel_initialized(panel)) {
		rc = -EINVAL;
		goto error;
	}

	/* if hbm already set by onscreenfinger, keep it */
	if (panel->is_hbm_enabled) {
		rc = 0;
		goto error;
	}

	dsi_panel_set_backlight(panel, panel->bl_config.bl_level);

	if (!strcmp(panel->oplus_priv.vendor_name, "AMS643YE01") && (panel->bl_config.bl_level > 2047)) {
		if (!strcmp(panel->name, "samsung 20261 ams643ye01 amoled fhd+ panel without DSC") ||
			!strcmp(panel->name, "samsung 20331 ams643ye01 amoled fhd+ panel without DSC")) {
			rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_HBM_ENTER1_SWITCH);
			oplus_dsi_display_enable_and_waiting_for_next_te_irq();
			rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_HBM_ENTER2_SWITCH);
		} else {
			rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_HBM_ENTER_SWITCH);
		}
	} else
		rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_HBM_OFF);
	if (rc) {
		pr_err("[%s] failed to send DSI_CMD_HBM_OFF cmds, rc=%d\n",
				panel->name, rc);
	}

error:
	mutex_unlock(&panel->panel_lock);
	return rc;
}

int dsi_display_hbm_on(struct dsi_display *display)
{
	int rc = 0;
	if (!display || !display->panel) {
		pr_err("Invalid params\n");
		return -EINVAL;
	}

	mutex_lock(&display->display_lock);

	/* enable the clk vote for CMD mode panels */
	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
		dsi_display_clk_ctrl(display->dsi_clk_handle,
			DSI_CORE_CLK, DSI_CLK_ON);
	}

	rc = dsi_panel_hbm_on(display->panel);
		if (rc) {
			pr_err("[%s] failed to dsi_panel_hbm_on, rc=%d\n",
					display->name, rc);
	}

	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
	rc = dsi_display_clk_ctrl(display->dsi_clk_handle,
				DSI_CORE_CLK, DSI_CLK_OFF);
	}
	mutex_unlock(&display->display_lock);

	return rc;
}

int dsi_display_normal_hbm_on(struct dsi_display *display)
{
	int rc = 0;
	if (!display || !display->panel) {
		pr_err("Invalid params\n");
		return -EINVAL;
	}

	mutex_lock(&display->display_lock);

	/* enable the clk vote for CMD mode panels */
	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
		dsi_display_clk_ctrl(display->dsi_clk_handle,
			DSI_CORE_CLK, DSI_CLK_ON);
	}

	rc = dsi_panel_normal_hbm_on(display->panel);
		if (rc) {
			pr_err("[%s] failed to dsi_panel_normal_hbm_on, rc=%d\n",
					display->name, rc);
	}

	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
	rc = dsi_display_clk_ctrl(display->dsi_clk_handle,
							  DSI_CORE_CLK, DSI_CLK_OFF);
	}
	mutex_unlock(&display->display_lock);
	return rc;
}

int dsi_display_hbm_off(struct dsi_display *display)
{
	int rc = 0;
	if (!display || !display->panel) {
		pr_err("Invalid params\n");
		return -EINVAL;
	}

	mutex_lock(&display->display_lock);

	/* enable the clk vote for CMD mode panels */
	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
		dsi_display_clk_ctrl(display->dsi_clk_handle,
			DSI_CORE_CLK, DSI_CLK_ON);
	}

	rc = dsi_panel_hbm_off(display->panel);
		if (rc) {
			pr_err("[%s] failed to dsi_panel_hbm_off, rc=%d\n",
					display->name, rc);
	}

	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
	rc = dsi_display_clk_ctrl(display->dsi_clk_handle,
				DSI_CORE_CLK, DSI_CLK_OFF);
	}
	mutex_unlock(&display->display_lock);
	return rc;
}

int oplus_display_get_hbm_mode(void)
{
	return hbm_mode;
}

int __oplus_display_set_hbm(int mode)
{
	mutex_lock(&oplus_hbm_lock);
	if (mode != hbm_mode) {
		hbm_mode = mode;
	}
	mutex_unlock(&oplus_hbm_lock);
	return 0;
}

int oplus_display_panel_set_hbm(void *buf)
{
	struct dsi_display *display = get_main_display();
	unsigned int *temp_save = buf;
	int ret = 0;

	sscanf(buf, "%du", &temp_save);
	printk(KERN_INFO "%s oplus_display_set_hbm = %d\n", __func__, (*temp_save));
	if (get_oplus_display_power_status() != OPLUS_DISPLAY_POWER_ON) {
		printk(KERN_ERR	 "%s oplus_display_set_hbm = %d, but now display panel status is not on\n", __func__, (*temp_save));
		return -EFAULT;
	}

	if (!display) {
		printk(KERN_INFO "oplus_display_set_hbm and main display is null");
		return -EINVAL;
	}
	__oplus_display_set_hbm((*temp_save));

	if ((hbm_mode > 1) &&(hbm_mode <= 10)) {
		ret = dsi_display_normal_hbm_on(get_main_display());
	} else if (hbm_mode == 1) {
		ret = dsi_display_hbm_on(get_main_display());
	} else if (hbm_mode == 0) {
		ret = dsi_display_hbm_off(get_main_display());
	}

	if (ret) {
		pr_err("failed to set hbm status ret=%d", ret);
		return ret;
	}

	return 0;
}

int oplus_display_panel_get_hbm(void *buf)
{
	unsigned int *hbm_temp = buf;

	printk(KERN_INFO "oplus_display_get_hbm = %d\n", hbm_mode);
	(*hbm_temp) = hbm_mode;

	return 0;
}

#ifdef OPLUS_BUG_STABILITY
/*Add for solve backlight issue for hbm*/
int oplus_dsi_hbm_backlight_setting(bool enabled)
{
	struct dsi_display *display = get_main_display();
	int ret = 0;

	if (!display) {
		pr_err("failed for: %s %d\n", __func__, __LINE__);
		return -EINVAL;
	}
	if(enabled) {
		ret = dsi_panel_tx_cmd_set(display->panel, DSI_CMD_HBM_BACKLIGHT_ON);
		if (ret) {
			pr_err("[%s] failed to send DSI_CMD_HBM_BACKLIGHT_ON cmds, rc=%d\n", display->panel->name, ret);
		}
	} else {
        ret = dsi_panel_tx_cmd_set(display->panel, DSI_CMD_HBM_BACKLIGHT_OFF);
		if (ret) {
			pr_err("[%s] failed to send DSI_CMD_HBM_BACKLIGHT_OFF cmds, rc=%d\n", display->panel->name, ret);
		}
	}
	return ret;
}
#endif /*OPLUS_BUG_STABILITY*/
