/***************************************************************
** Copyright (C),  2020,  OPLUS Mobile Comm Corp.,  Ltd
** VENDOR_EDIT
** File : oplus_dc_diming.h
** Description : oplus dc_diming feature
** Version : 1.0
** Date : 2020/04/15
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**   Qianxu         2020/04/15        1.0           Build this moudle
******************************************************************/
#ifndef _OPLUS_DC_DIMING_H_
#define _OPLUS_DC_DIMING_H_

#include <drm/drm_connector.h>

#include "dsi_panel.h"
#include "dsi_defs.h"
#include "oplus_display_panel_hbm.h"

int sde_connector_update_backlight(struct drm_connector *connector, bool post);

int sde_connector_update_hbm(struct drm_connector *connector);

int oplus_seed_bright_to_alpha(int brightness);

struct dsi_panel_cmd_set * oplus_dsi_update_seed_backlight(struct dsi_panel *panel, int brightness,
				enum dsi_cmd_set_type type);
int oplus_display_panel_get_dim_alpha(void *buf);
int oplus_display_panel_set_dim_alpha(void *buf);
int oplus_display_panel_get_dim_dc_alpha(void *buf);
int dsi_panel_parse_oplus_dc_config(struct dsi_panel *panel);
int oplus_dsi_display_enable_and_waiting_for_next_te_irq(void);
#endif /*_OPLUS_DC_DIMING_H_*/
