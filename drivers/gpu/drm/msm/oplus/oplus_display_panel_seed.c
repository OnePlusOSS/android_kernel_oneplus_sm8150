/***************************************************************
** Copyright (C),  2020,  OPLUS Mobile Comm Corp.,  Ltd
** VENDOR_EDIT
** File : oplus_display_panel_seed.c
** Description : oplus display panel seed feature
** Version : 1.0
** Date : 2020/06/13
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**  Li.Sheng       2020/06/13        1.0           Build this moudle
******************************************************************/
#include "oplus_display_panel_seed.h"
#include "oplus_dsi_support.h"

int seed_mode = 0;
extern bool oplus_dc_v2_on;
DEFINE_MUTEX(oplus_seed_lock);

int oplus_display_get_seed_mode(void)
{
	return seed_mode;
}

int __oplus_display_set_seed(int mode)
{
	mutex_lock(&oplus_seed_lock);

	if (mode != seed_mode) {
		seed_mode = mode;
	}

	mutex_unlock(&oplus_seed_lock);
	return 0;
}

int dsi_panel_seed_mode_unlock(struct dsi_panel *panel, int mode)
{
	int rc = 0;

	if (!dsi_panel_initialized(panel)) {
		return -EINVAL;
	}

	if (oplus_dc_v2_on) {
		rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_SEED_ENTER);

		if (rc) {
			pr_err("[%s] failed to send DSI_CMD_SEED_ENTER cmds, rc=%d\n",
				panel->name, rc);
		}
	}


	switch (mode) {
	case 0:
		rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_SEED_MODE0);

		if (rc) {
			pr_err("[%s] failed to send DSI_CMD_SEED_MODE0 cmds, rc=%d\n",
				panel->name, rc);
		}
		dsi_panel_tx_cmd_set(panel, DSI_CMD_LOADING_EFFECT_OFF);
		break;

	case 1:
		rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_SEED_MODE1);

		if (rc) {
			pr_err("[%s] failed to send DSI_CMD_SEED_MODE1 cmds, rc=%d\n",
				panel->name, rc);
		}
		dsi_panel_tx_cmd_set(panel, DSI_CMD_LOADING_EFFECT_OFF);
		break;

	case 2:
		rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_SEED_MODE2);

		if (rc) {
			pr_err("[%s] failed to send DSI_CMD_SEED_MODE2 cmds, rc=%d\n",
				panel->name, rc);
		}
		dsi_panel_tx_cmd_set(panel, DSI_CMD_LOADING_EFFECT_OFF);
		break;

	case 3:
		rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_SEED_MODE3);

		if (rc) {
			pr_err("[%s] failed to send DSI_CMD_SEED_MODE3 cmds, rc=%d\n",
				panel->name, rc);
		}
		dsi_panel_tx_cmd_set(panel, DSI_CMD_LOADING_EFFECT_ON);
		break;

	case 4:
		rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_SEED_MODE4);

		if (rc) {
			pr_err("[%s] failed to send DSI_CMD_SEED_MODE4 cmds, rc=%d\n",
				panel->name, rc);
		}
		dsi_panel_tx_cmd_set(panel, DSI_CMD_LOADING_EFFECT_OFF);
		break;

	default:
		rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_SEED_OFF);

		if (rc) {
			pr_err("[%s] failed to send DSI_CMD_SEED_OFF cmds, rc=%d\n",
				panel->name, rc);
		}

		pr_err("[%s] seed mode Invalid %d\n",
			panel->name, mode);
	}

	if (!oplus_dc_v2_on) {
		rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_SEED_EXIT);
		if (rc) {
			pr_err("[%s] failed to send DSI_CMD_SEED_EXIT cmds, rc=%d\n",
				panel->name, rc);
		}
	}

	return rc;
}

int dsi_panel_seed_mode(struct dsi_panel *panel, int mode)
{
	int rc = 0;

	if (!panel) {
		pr_err("Invalid params\n");
		return -EINVAL;
	}

	mutex_lock(&panel->panel_lock);

	rc = dsi_panel_seed_mode_unlock(panel, mode);

	mutex_unlock(&panel->panel_lock);
	return rc;
}

int dsi_display_seed_mode(struct dsi_display *display, int mode)
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

	rc = dsi_panel_seed_mode(display->panel, mode);

	if (rc) {
		pr_err("[%s] failed to dsi_panel_seed_on, rc=%d\n",
			display->name, rc);
	}

	if (display->config.panel_mode == DSI_OP_CMD_MODE) {
		rc = dsi_display_clk_ctrl(display->dsi_clk_handle,
				DSI_CORE_CLK, DSI_CLK_OFF);
	}

	mutex_unlock(&display->display_lock);
	return rc;
}

int oplus_dsi_update_seed_mode(void)
{
	struct dsi_display *display = get_main_display();
	int ret = 0;

	if (!display) {
		pr_err("failed for: %s %d\n", __func__, __LINE__);
		return -EINVAL;
	}

	ret = dsi_display_seed_mode(display, seed_mode);

	return ret;
}

int oplus_display_panel_get_seed(void *data)
{
	uint32_t *temp = data;
	printk(KERN_INFO "oplus_display_get_seed = %d\n", seed_mode);

	(*temp) = seed_mode;
	return 0;
}

int oplus_display_panel_set_seed(void *data)
{
	uint32_t *temp_save = data;

	printk(KERN_INFO "%s oplus_display_set_seed = %d\n", __func__, *temp_save);
	seed_mode = *temp_save;

	__oplus_display_set_seed(*temp_save);

	if (get_oplus_display_power_status() == OPLUS_DISPLAY_POWER_ON) {
		if (get_main_display() == NULL) {
			printk(KERN_INFO "oplus_display_set_seed and main display is null");
			return -EINVAL;
		}

		dsi_display_seed_mode(get_main_display(), seed_mode);

	} else {
		printk(KERN_ERR
			"%s oplus_display_set_seed = %d, but now display panel status is not on\n",
			__func__, *temp_save);
	}

	return 0;
}
