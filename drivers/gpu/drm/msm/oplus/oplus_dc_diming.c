/***************************************************************
** Copyright (C),  2020,  OPLUS Mobile Comm Corp.,  Ltd
** VENDOR_EDIT
** File : oplus_dc_diming.c
** Description : oplus dc_diming feature
** Version : 1.0
** Date : 2020/04/15
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**   Qianxu         2020/04/15        1.0           Build this moudle
******************************************************************/

#include "oplus_display_private_api.h"
#include "oplus_dc_diming.h"
#include "oplus_onscreenfingerprint.h"
#include "oplus_aod.h"
#include "oplus_display_panel_seed.h"
#include "sde_trace.h"


int oplus_dimlayer_bl = 0;
int oplus_dimlayer_bl_enabled = 0;
int oplus_datadimming_v3_skip_frame = 2;
int oplus_panel_alpha = 0;
int oplus_underbrightness_alpha = 0;
int fod_dimlayer_flag = -1;                 /*Flag to check if FOD dimlayer is about to be disabled or enabled*/
					    /*-1 = undefined; 1 = enabling dimlayer; 0 = disabling dimlayer*/
static struct dsi_panel_cmd_set oplus_priv_seed_cmd_set;

extern int oplus_dimlayer_bl_on_vblank;
extern int oplus_dimlayer_bl_off_vblank;
extern int oplus_dimlayer_bl_delay;
extern int oplus_dimlayer_bl_delay_after;
extern int oplus_dimlayer_bl_enable_v2;
extern int oplus_dimlayer_bl_enable_v2_real;
extern int oplus_dimlayer_bl_enable_v3;
extern int oplus_dimlayer_bl_enable_v3_real;
extern int oplus_datadimming_vblank_count;
extern atomic_t oplus_datadimming_vblank_ref;
extern int oplus_fod_on_vblank;
extern int oplus_fod_off_vblank;
extern bool oplus_skip_datadimming_sync;
extern int oplus_dimlayer_hbm_vblank_count;
extern atomic_t oplus_dimlayer_hbm_vblank_ref;
extern int oplus_dc2_alpha;
extern int oplus_seed_backlight;
extern int oplus_panel_alpha;
extern oplus_dc_v2_on;
extern ktime_t oplus_backlight_time;
#ifdef OPLUS_FEATURE_AOD_RAMLESS
extern int oplus_display_mode;
extern atomic_t aod_onscreenfp_status;
#endif /* OPLUS_FEATURE_AOD_RAMLESS */
extern int cmp_display_panel_name(char *istr);
extern int seed_mode;

static struct oplus_brightness_alpha brightness_seed_alpha_lut_dc[] = {
	{0, 0xff},
	{1, 0xfc},
	{2, 0xfb},
	{3, 0xfa},
	{4, 0xf9},
	{5, 0xf8},
	{6, 0xf7},
	{8, 0xf6},
	{10, 0xf4},
	{15, 0xf0},
	{20, 0xea},
	{30, 0xe0},
	{45, 0xd0},
	{70, 0xbc},
	{100, 0x98},
	{120, 0x80},
	{140, 0x70},
	{160, 0x58},
	{180, 0x48},
	{200, 0x30},
	{220, 0x20},
	{240, 0x10},
	{260, 0x00},
};

/*Jiasong.ZhongPSW.MM.Display.LCD.Stable,2020-09-17 add for DC backlight */
int dsi_panel_parse_oplus_dc_config(struct dsi_panel *panel)
{
	int rc = 0;
	int i;
	u32 length = 0;
	u32 count = 0;
	u32 size = 0;
	u32 *arr_32 = NULL;
	const u32 *arr;
	struct dsi_parser_utils *utils = &panel->utils;
	struct oplus_brightness_alpha *seq;

	arr = utils->get_property(utils->data, "oplus,dsi-dc-brightness", &length);
	if (!arr) {
		DSI_ERR("[%s] oplus,dsi-dc-brightness  not found\n", panel->name);
		return -EINVAL;
	}

	if (length & 0x1) {
		DSI_ERR("[%s] oplus,dsi-dc-brightness length error\n", panel->name);
		return -EINVAL;
	}

	DSI_DEBUG("RESET SEQ LENGTH = %d\n", length);
	length = length / sizeof(u32);
	size = length * sizeof(u32);

	arr_32 = kzalloc(size, GFP_KERNEL);
	if (!arr_32) {
		rc = -ENOMEM;
		goto error;
	}

	rc = utils->read_u32_array(utils->data, "oplus,dsi-dc-brightness",
					arr_32, length);
	if (rc) {
		DSI_ERR("[%s] cannot read dsi-dc-brightness\n", panel->name);
		goto error_free_arr_32;
	}

	count = length / 2;
	size = count * sizeof(*seq);
	seq = kzalloc(size, GFP_KERNEL);
	if (!seq) {
		rc = -ENOMEM;
		goto error_free_arr_32;
	}

	panel->dc_ba_seq = seq;
	panel->dc_ba_count = count;

	for (i = 0; i < length; i += 2) {
		seq->brightness = arr_32[i];
		seq->alpha = arr_32[i + 1];
		seq++;
	}

error_free_arr_32:
	kfree(arr_32);
error:
	return rc;
}

extern void oplus_dsi_display_change_te_irq_status(void *disp, bool enable);

int oplus_dsi_display_enable_and_waiting_for_next_te_irq(void)
{
	int const switch_te_timeout = msecs_to_jiffies(18);
	struct dsi_display *display = get_main_display();
	SDE_ATRACE_BEGIN("wait_te_irq");
	/* enable te irq */

	if (display->panel->cur_mode->timing.refresh_rate == 60) {
		msleep(9);
	} else if (display->panel->cur_mode->timing.refresh_rate == 90) {
		msleep(11);
	}

	oplus_dsi_display_change_te_irq_status(display, true);
	pr_info("Waiting for the next TE to switch\n");

	display->vsync_switch_pending = true;
	reinit_completion(&display->switch_te_gate);

	if (!wait_for_completion_timeout(&display->switch_te_gate, switch_te_timeout)) {
		DSI_ERR("hbm vsync switch TE check failed\n");
		oplus_dsi_display_change_te_irq_status(display, false);
		return -EINVAL;
	}
	/* disable te irq */
	oplus_dsi_display_change_te_irq_status(display, false);
	SDE_ATRACE_END("wait_te_irq");

	return 0;
}

int sde_connector_update_backlight(struct drm_connector *connector, bool post)
{
	struct sde_connector *c_conn = to_sde_connector(connector);
	struct dsi_display *dsi_display;

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI)
		return 0;

	dsi_display = c_conn->display;
	if (!dsi_display || !dsi_display->panel || !dsi_display->panel->cur_mode) {
		SDE_ERROR("Invalid params(s) dsi_display %pK, panel %pK\n",
				dsi_display,
				((dsi_display) ? dsi_display->panel : NULL));
		return -EINVAL;
	}

	if (!connector->state || !connector->state->crtc)
		return 0;

	if (oplus_dimlayer_bl != oplus_dimlayer_bl_enabled) {
		struct sde_connector *c_conn = to_sde_connector(connector);
		struct drm_crtc *crtc = connector->state->crtc;
		struct dsi_panel *panel = dsi_display->panel;
		int bl_lvl = dsi_display->panel->bl_config.bl_level;
		u32 current_vblank;
		int on_vblank = 0;
		int off_vblank = 0;
		int vblank = 0;
		int ret = 0;
		int vblank_get = -EINVAL;
		int on_delay = 0, on_delay_after = 0;
		int off_delay = 0, off_delay_after = 0;
		int delay = 0, delay_after = 0;

		if (sde_crtc_get_fingerprint_mode(crtc->state)) {
			oplus_dimlayer_bl_enabled = oplus_dimlayer_bl;
			goto done;
		}

		if (bl_lvl < panel->cur_mode->priv_info->fod_th_brightness) {
				on_vblank = panel->cur_mode->priv_info->fod_on_vblank;
				off_vblank = panel->cur_mode->priv_info->fod_off_vblank;
				on_delay = panel->cur_mode->priv_info->fod_on_delay;
				off_delay = panel->cur_mode->priv_info->fod_off_delay;
		} else {
				on_vblank = panel->cur_mode->priv_info->fod_on_vblank_above_th;
				off_vblank = panel->cur_mode->priv_info->fod_off_vblank_above_th;
				on_delay = panel->cur_mode->priv_info->fod_on_delay_above_th;
				off_delay = panel->cur_mode->priv_info->fod_off_delay_above_th;
		}

		if (oplus_dimlayer_bl_on_vblank != INT_MAX)
			on_vblank = oplus_dimlayer_bl_on_vblank;

		if (oplus_dimlayer_bl_off_vblank != INT_MAX)
			off_vblank = oplus_dimlayer_bl_off_vblank;


		if (oplus_dimlayer_bl) {
			vblank = on_vblank;
			delay = on_delay;
			delay_after = on_delay_after;
		} else {
			vblank = off_vblank;
			delay = off_delay;
			delay_after = off_delay_after;
		}

		if (oplus_dimlayer_bl_delay >= 0)
			delay = oplus_dimlayer_bl_delay;

		if (oplus_dimlayer_bl_delay_after >= 0)
			delay_after = oplus_dimlayer_bl_delay_after;

		vblank_get = drm_crtc_vblank_get(crtc);
		if (vblank >= 0) {
			if (!post) {
				oplus_dimlayer_bl_enabled = oplus_dimlayer_bl;
				current_vblank = drm_crtc_vblank_count(crtc);
				ret = wait_event_timeout(*drm_crtc_vblank_waitqueue(crtc),
						current_vblank != drm_crtc_vblank_count(crtc),
						msecs_to_jiffies(34));
				current_vblank = drm_crtc_vblank_count(crtc) + vblank;
				if (delay > 0)
					usleep_range(delay, delay + 100);
				_sde_connector_update_bl_scale_(c_conn);
				if (delay_after)
					usleep_range(delay_after, delay_after + 100);
				if (vblank > 0) {
					ret = wait_event_timeout(*drm_crtc_vblank_waitqueue(crtc),
							current_vblank == drm_crtc_vblank_count(crtc),
							msecs_to_jiffies(17 * 3));
				}
			}
		} else {
			if (!post) {
				current_vblank = drm_crtc_vblank_count(crtc);
				ret = wait_event_timeout(*drm_crtc_vblank_waitqueue(crtc),
						current_vblank != drm_crtc_vblank_count(crtc),
						msecs_to_jiffies(34));
			} else {
				if (vblank < -1) {
					current_vblank = drm_crtc_vblank_count(crtc) + 1 - vblank;
					ret = wait_event_timeout(*drm_crtc_vblank_waitqueue(crtc),
							current_vblank == drm_crtc_vblank_count(crtc),
							msecs_to_jiffies(17 * 3));
				}
				oplus_dimlayer_bl_enabled = oplus_dimlayer_bl;

				if (delay > 0)
					usleep_range(delay, delay + 100);
				_sde_connector_update_bl_scale_(c_conn);
				if (delay_after)
					usleep_range(delay_after, delay_after + 100);
			}
		}
		if (!vblank_get)
			drm_crtc_vblank_put(crtc);
	}

	if (oplus_dimlayer_bl_enable_v2 != oplus_dimlayer_bl_enable_v2_real) {
		struct sde_connector *c_conn = to_sde_connector(connector);

		oplus_dimlayer_bl_enable_v2_real = oplus_dimlayer_bl_enable_v2;
		_sde_connector_update_bl_scale_(c_conn);
	}

done:
	if (post) {
		if (oplus_datadimming_vblank_count> 0) {
			oplus_datadimming_vblank_count--;
		} else {
			while (atomic_read(&oplus_datadimming_vblank_ref) > 0) {
				drm_crtc_vblank_put(connector->state->crtc);
				atomic_dec(&oplus_datadimming_vblank_ref);
			}
		}
	}

	return 0;
}

#ifdef OPLUS_BUG_STABILITY
extern u32 flag_writ;
#endif /*OPLUS_BUG_STABILITY*/

int sde_connector_update_hbm(struct drm_connector *connector)
{
	struct sde_connector *c_conn = to_sde_connector(connector);
	struct dsi_display *dsi_display;
	struct sde_connector_state *c_state;
	int rc = 0;
	int fingerprint_mode;
	static int oplus_old_refresh_rate = 0;

	if (!c_conn) {
		SDE_ERROR("Invalid params sde_connector null\n");
		return -EINVAL;
	}

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI)
		return 0;

	c_state = to_sde_connector_state(connector->state);

	dsi_display = c_conn->display;
	if (!dsi_display || !dsi_display->panel) {
		SDE_ERROR("Invalid params(s) dsi_display %pK, panel %pK\n",
			dsi_display,
			((dsi_display) ? dsi_display->panel : NULL));
		return -EINVAL;
	}

	if (!c_conn->encoder || !c_conn->encoder->crtc ||
	    !c_conn->encoder->crtc->state) {
		return 0;
	}

	if (sde_crtc_get_fingerprint_mode(c_conn->encoder->crtc->state) && !dsi_display->panel->is_hbm_enabled &&
			dsi_display->panel->cur_mode->timing.refresh_rate != oplus_old_refresh_rate) {
		SDE_ATRACE_BEGIN("delay_hbm_on_one_frame");
		pr_err("do not send HBM ON while switch fps");
		oplus_old_refresh_rate = dsi_display->panel->cur_mode->timing.refresh_rate;
		SDE_ATRACE_END("delay_hbm_on_one_frame");
		return 0;
	}

	oplus_old_refresh_rate = dsi_display->panel->cur_mode->timing.refresh_rate;

	fingerprint_mode = sde_crtc_get_fingerprint_mode(c_conn->encoder->crtc->state);

	if (OPLUS_DISPLAY_AOD_SCENE == get_oplus_display_scene()) {
		if (sde_crtc_get_fingerprint_pressed(c_conn->encoder->crtc->state)) {
			sde_crtc_set_onscreenfinger_defer_sync(c_conn->encoder->crtc->state, true);
		} else {
			sde_crtc_set_onscreenfinger_defer_sync(c_conn->encoder->crtc->state, false);
			fingerprint_mode = false;
		}
	} else {
		sde_crtc_set_onscreenfinger_defer_sync(c_conn->encoder->crtc->state, false);
	}

	if (fingerprint_mode != dsi_display->panel->is_hbm_enabled) {
		struct drm_crtc *crtc = c_conn->encoder->crtc;
		struct dsi_panel *panel = dsi_display->panel;
		int vblank = 0;
		u32 target_vblank, current_vblank;
		int ret;

		if (oplus_fod_on_vblank >= 0)
			panel->cur_mode->priv_info->fod_on_vblank = oplus_fod_on_vblank;
		if (oplus_fod_off_vblank >= 0)
			panel->cur_mode->priv_info->fod_off_vblank = oplus_fod_off_vblank;

		pr_err("OnscreenFingerprint mode: %s",
		       fingerprint_mode ? "Enter" : "Exit");
		fod_dimlayer_flag = fingerprint_mode;
		dsi_display->panel->is_hbm_enabled = fingerprint_mode;
		if (fingerprint_mode) {
#ifdef OPLUS_FEATURE_AOD_RAMLESS
			if (!dsi_display->panel->oplus_priv.is_aod_ramless || oplus_display_mode) {
#endif /* OPLUS_FEATURE_AOD_RAMLESS */
				mutex_lock(&dsi_display->panel->panel_lock);

				if (!dsi_display->panel->panel_initialized) {
					dsi_display->panel->is_hbm_enabled = false;
					pr_err("panel not initialized, failed to Enter OnscreenFingerprint\n");
					mutex_unlock(&dsi_display->panel->panel_lock);
					return 0;
				}
				dsi_display_clk_ctrl(dsi_display->dsi_clk_handle,
						DSI_CORE_CLK, DSI_CLK_ON);

				if (oplus_seed_backlight) {
					int frame_time_us;

					frame_time_us = mult_frac(1000, 1000, panel->cur_mode->timing.refresh_rate);
					oplus_panel_process_dimming_v2(panel, panel->bl_config.bl_level, true);
					mipi_dsi_dcs_set_display_brightness(&panel->mipi_device, panel->bl_config.bl_level);
					oplus_panel_process_dimming_v2_post(panel, true);
					usleep_range(frame_time_us, frame_time_us + 100);
				}
#ifdef OPLUS_FEATURE_AOD_RAMLESS
				else if (dsi_display->panel->oplus_priv.is_aod_ramless) {
					ktime_t delta = ktime_sub(ktime_get(), oplus_backlight_time);
					s64 delta_us = ktime_to_us(delta);
					if (delta_us < 34000 && delta_us >= 0)
						usleep_range(34000 - delta_us, 34000 - delta_us + 100);
				}
#endif /* OPLUS_FEATURE_AOD_RAMLESS */

				if (OPLUS_DISPLAY_AOD_SCENE != get_oplus_display_scene() &&
						dsi_display->panel->bl_config.bl_level) {
					if (dsi_display->config.panel_mode != DSI_OP_VIDEO_MODE) {
						current_vblank = drm_crtc_vblank_count(crtc);
						ret = wait_event_timeout(*drm_crtc_vblank_waitqueue(crtc),
								current_vblank != drm_crtc_vblank_count(crtc),
								msecs_to_jiffies(17));
					}
					if (!strcmp(panel->oplus_priv.vendor_name, "AMS643YE01") || cmp_display_panel_name("SOFEF03F_M"))
						usleep_range(5500, 5600);
					if(cmp_display_panel_name("S6E3HC2"))   /*For 18821 and 19801*/
						usleep_range(6500, 6600);
					vblank = panel->cur_mode->priv_info->fod_on_vblank;
					target_vblank = drm_crtc_vblank_count(crtc) + vblank;
					rc = dsi_panel_tx_cmd_set(dsi_display->panel, DSI_CMD_AOD_HBM_ON);


					if (vblank) {
						ret = wait_event_timeout(*drm_crtc_vblank_waitqueue(crtc),
								target_vblank == drm_crtc_vblank_count(crtc),
								msecs_to_jiffies((vblank + 1) * 17));
						if (!ret) {
							pr_err("OnscreenFingerprint failed to wait vblank timeout target_vblank=%d current_vblank=%d\n",
									target_vblank, drm_crtc_vblank_count(crtc));
						}
					}
				} else {
					rc = dsi_panel_tx_cmd_set(dsi_display->panel, DSI_CMD_AOD_HBM_ON);
				}

				dsi_display_clk_ctrl(dsi_display->dsi_clk_handle,
						DSI_CORE_CLK, DSI_CLK_OFF);

				mutex_unlock(&dsi_display->panel->panel_lock);
				if (rc) {
					pr_err("failed to send DSI_CMD_HBM_ON cmds, rc=%d\n", rc);
					return rc;
				}
#ifdef OPLUS_FEATURE_AOD_RAMLESS
			}
#endif /* OPLUS_FEATURE_AOD_RAMLESS */
		} else {
			bool aod_hbm_off = true;
#ifdef OPLUS_FEATURE_AOD_RAMLESS
			if(dsi_display->panel->oplus_priv.is_aod_ramless) {
				if(atomic_read(&aod_onscreenfp_status)) {
					aod_hbm_off = false;
					pr_err("%s, skip AOD_HBM_OFF for ramless panel\n", __func__);
				}
			}
#endif /* OPLUS_FEATURE_AOD_RAMLESS */

			mutex_lock(&dsi_display->panel->panel_lock);

			if (!dsi_display->panel->panel_initialized) {
				dsi_display->panel->is_hbm_enabled = true;
				pr_err("panel not initialized, failed to Exit OnscreenFingerprint\n");
				mutex_unlock(&dsi_display->panel->panel_lock);
				return 0;
			}

			current_vblank = drm_crtc_vblank_count(crtc);

			ret = wait_event_timeout(*drm_crtc_vblank_waitqueue(crtc),
					current_vblank != drm_crtc_vblank_count(crtc),
					msecs_to_jiffies(17));
			/*add for solve hbm backlight issue*/
			flag_writ = 3;

			if(!dsi_display->panel->oplus_priv.prj_flag) {
				oplus_skip_datadimming_sync = true;
				oplus_panel_update_backlight_unlock(panel);
				oplus_skip_datadimming_sync = false;
			}
			fod_dimlayer_flag = fingerprint_mode;
			vblank = panel->cur_mode->priv_info->fod_off_vblank;
			target_vblank = drm_crtc_vblank_count(crtc) + vblank;

			dsi_display_clk_ctrl(dsi_display->dsi_clk_handle,
					     DSI_CORE_CLK, DSI_CLK_ON);
			if (OPLUS_DISPLAY_AOD_HBM_SCENE == get_oplus_display_scene()) {
				if (OPLUS_DISPLAY_POWER_DOZE_SUSPEND == get_oplus_display_power_status() ||
					OPLUS_DISPLAY_POWER_DOZE == get_oplus_display_power_status()) {
					rc = dsi_panel_tx_cmd_set(dsi_display->panel, DSI_CMD_AOD_HBM_OFF);

#ifdef OPLUS_FEATURE_AOD_RAMLESS
					if (dsi_display->panel->oplus_priv.is_aod_ramless) {
						oplus_update_aod_light_mode_unlock(panel);
					}
#endif /* OPLUS_FEATURE_AOD_RAMLESS */
					set_oplus_display_scene(OPLUS_DISPLAY_AOD_SCENE);
				} else {
					rc = dsi_panel_tx_cmd_set(dsi_display->panel, DSI_CMD_SET_NOLP);

					/* set nolp would exit hbm, restore when panel status on hbm */
					if(panel->bl_config.bl_level > panel->bl_config.brightness_normal_max_level) {
						if (!strcmp(panel->name, "samsung 20261 ams643ye01 amoled fhd+ panel without DSC") ||
							!strcmp(panel->name, "samsung 20331 ams643ye01 amoled fhd+ panel without DSC")) {
							rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_HBM_ENTER1_SWITCH);
							oplus_dsi_display_enable_and_waiting_for_next_te_irq();
							rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_HBM_ENTER2_SWITCH);
						} else {
							rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_HBM_ENTER_SWITCH);
						}
					}

					set_oplus_display_scene(OPLUS_DISPLAY_NORMAL_SCENE);
					oplus_panel_update_backlight_unlock(panel);
					if (oplus_display_get_hbm_mode()) {
						rc = dsi_panel_tx_cmd_set(dsi_display->panel, DSI_CMD_AOD_HBM_ON);
					}
				}
			} else if (oplus_display_get_hbm_mode()) {
				/* Do nothing to skip hbm off */
			} else if (OPLUS_DISPLAY_AOD_SCENE == get_oplus_display_scene()) {
				if(aod_hbm_off) {
					rc = dsi_panel_tx_cmd_set(dsi_display->panel, DSI_CMD_AOD_HBM_OFF);

#ifdef OPLUS_FEATURE_AOD_RAMLESS
					if (dsi_display->panel->oplus_priv.is_aod_ramless) {
						oplus_update_aod_light_mode_unlock(panel);
					}
#endif /* OPLUS_FEATURE_AOD_RAMLESS */
				}
			} else {
				rc = dsi_panel_tx_cmd_set(dsi_display->panel, DSI_CMD_HBM_OFF);
				/*add for hbm mode*/
				if(panel->bl_config.bl_level > panel->bl_config.brightness_normal_max_level) {
					if (!strcmp(panel->name, "samsung 20261 ams643ye01 amoled fhd+ panel without DSC") ||
						!strcmp(panel->name, "samsung 20331 ams643ye01 amoled fhd+ panel without DSC")) {
						rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_HBM_ENTER1_SWITCH);
						oplus_dsi_display_enable_and_waiting_for_next_te_irq();
						rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_HBM_ENTER2_SWITCH);
					} else {
						rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_HBM_ENTER_SWITCH);
					}
				}
				dsi_panel_set_backlight(panel, panel->bl_config.bl_level);
			}

			dsi_display_clk_ctrl(dsi_display->dsi_clk_handle,
					     DSI_CORE_CLK, DSI_CLK_OFF);

			if(dsi_display->panel->oplus_priv.prj_flag) {
				oplus_skip_datadimming_sync = true;
				oplus_panel_update_backlight_unlock(panel);
				oplus_skip_datadimming_sync = false;
			}

			mutex_unlock(&dsi_display->panel->panel_lock);
			if (vblank) {
				ret = wait_event_timeout(*drm_crtc_vblank_waitqueue(crtc),
						target_vblank == drm_crtc_vblank_count(crtc),
						msecs_to_jiffies((vblank + 1) * 17));
				if (!ret) {
					pr_err("OnscreenFingerprint failed to wait vblank timeout target_vblank=%d current_vblank=%d\n",
							target_vblank, drm_crtc_vblank_count(crtc));
				}
			}
		}
	}

	if (oplus_dimlayer_hbm_vblank_count > 0) {
		oplus_dimlayer_hbm_vblank_count--;
	} else {
		while (atomic_read(&oplus_dimlayer_hbm_vblank_ref) > 0) {
			drm_crtc_vblank_put(connector->state->crtc);
			atomic_dec(&oplus_dimlayer_hbm_vblank_ref);
		}
	}

	return 0;
}

int oplus_seed_bright_to_alpha(int brightness)
{
	struct dsi_display *display = get_main_display();
	struct oplus_brightness_alpha *lut = NULL;
	int count = 0;
	int i = 0;
	int alpha;

	if (!display)
		return 0;

	if (oplus_panel_alpha)
		return oplus_panel_alpha;

	if (display->panel->dc_ba_seq && display->panel->dc_ba_count) {
		count = display->panel->dc_ba_count;
		lut = display->panel->dc_ba_seq;
	} else {
		count = ARRAY_SIZE(brightness_seed_alpha_lut_dc);
		lut = brightness_seed_alpha_lut_dc;
	}

	for (i = 0; i < count; i++) {
		if (lut[i].brightness >= brightness)
			break;
	}

	if (i == 0)
		alpha = lut[0].alpha;
	else if (i == count)
		alpha = lut[count - 1].alpha;
	else
		alpha = interpolate(brightness, lut[i-1].brightness,
				    lut[i].brightness, lut[i-1].alpha,
				    lut[i].alpha);

	return alpha;
}
struct dsi_panel_cmd_set *
oplus_dsi_update_seed_backlight(struct dsi_panel *panel, int brightness,
				enum dsi_cmd_set_type type)
{
	enum dsi_cmd_set_state state;
	struct dsi_cmd_desc *cmds;
	struct dsi_cmd_desc *oplus_cmd;
	u8 *tx_buf;
	int count, rc = 0;
	int i = 0;
	int k = 0;
	int alpha = oplus_seed_bright_to_alpha(brightness);
	int seed_bl_max = panel->oplus_priv.seed_bl_max;

	if (type != DSI_CMD_SEED_MODE0 &&
		type != DSI_CMD_SEED_MODE1 &&
		type != DSI_CMD_SEED_MODE2 &&
		type != DSI_CMD_SEED_MODE3 &&
		type != DSI_CMD_SEED_MODE4 &&
		type != DSI_CMD_SEED_OFF) {
		return NULL;
	}

	if (type == DSI_CMD_SEED_OFF)
		type = DSI_CMD_SEED_MODE0;

	cmds = panel->cur_mode->priv_info->cmd_sets[type].cmds;
	count = panel->cur_mode->priv_info->cmd_sets[type].count;
	state = panel->cur_mode->priv_info->cmd_sets[type].state;

	oplus_cmd = kmemdup(cmds, sizeof(*cmds) * count, GFP_KERNEL);
	if (!oplus_cmd) {
		rc = -ENOMEM;
		goto error;
	}

	for (i = 0; i < count; i++)
		oplus_cmd[i].msg.tx_buf = NULL;

	for (i = 0; i < count; i++) {
		u32 size;

		size = oplus_cmd[i].msg.tx_len * sizeof(u8);

		oplus_cmd[i].msg.tx_buf = kmemdup(cmds[i].msg.tx_buf, size, GFP_KERNEL);
		if (!oplus_cmd[i].msg.tx_buf) {
			rc = -ENOMEM;
			goto error;
		}
	}

	for (i = 0; i < count; i++) {
		if (oplus_cmd[i].msg.tx_len != 0x16)
			continue;
		tx_buf = (u8 *)oplus_cmd[i].msg.tx_buf;
		for (k = 0; k < oplus_cmd[i].msg.tx_len; k++) {
			if (k == 0) {
				continue;
			}
			tx_buf[k] = tx_buf[k] * (seed_bl_max - alpha) / seed_bl_max;
		}
	}

	if (oplus_priv_seed_cmd_set.cmds) {
		for (i = 0; i < oplus_priv_seed_cmd_set.count; i++)
			kfree(oplus_priv_seed_cmd_set.cmds[i].msg.tx_buf);
		kfree(oplus_priv_seed_cmd_set.cmds);
	}

	oplus_priv_seed_cmd_set.cmds = oplus_cmd;
	oplus_priv_seed_cmd_set.count = count;
	oplus_priv_seed_cmd_set.state = state;
	oplus_dc2_alpha = alpha;

	return &oplus_priv_seed_cmd_set;

error:
	if (oplus_cmd) {
		for (i = 0; i < count; i++)
			kfree(oplus_cmd[i].msg.tx_buf);
		kfree(oplus_cmd);
	}
	return ERR_PTR(rc);
}

int oplus_display_panel_get_dim_alpha(void *buf) {
	unsigned int *temp_alpha = buf;
	struct dsi_display *display = get_main_display();

	if (!display->panel->is_hbm_enabled ||
		(get_oplus_display_power_status() != OPLUS_DISPLAY_POWER_ON)) {
		(*temp_alpha) = 0;
		return 0;
	}

	(*temp_alpha) = oplus_underbrightness_alpha;
	return 0;
}

int oplus_display_panel_set_dim_alpha(void *buf) {
	unsigned int *temp_alpha = buf;

	(*temp_alpha) = oplus_panel_alpha;

	return 0;
}

int oplus_display_panel_get_dim_dc_alpha(void *buf) {
	int ret = 0;
	unsigned int *temp_dim_alpha = buf;
	struct dsi_display *display = get_main_display();

	if (!display || !display->panel) {
		pr_err("%s main display is NULL\n", __func__);
		(*temp_dim_alpha) = 0;
		return 0;
	}

	if (display->panel->is_hbm_enabled ||
		get_oplus_display_power_status() != OPLUS_DISPLAY_POWER_ON) {
		ret = 0;
	}
	if (oplus_dc2_alpha != 0) {
		ret = oplus_dc2_alpha;
	} else if (oplus_underbrightness_alpha != 0) {
		ret = oplus_underbrightness_alpha;
	} else if (oplus_dimlayer_bl_enable_v3_real) {
		ret = 1;
	}

	(*temp_dim_alpha) = ret;
	return 0;
}
