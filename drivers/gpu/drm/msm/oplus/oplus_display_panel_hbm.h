/***************************************************************
** Copyright (C),  2020,  OPLUS Mobile Comm Corp.,  Ltd
** VENDOR_EDIT
** File : oplus_display_panel_hbm.h
** Description : oplus display panel hbm feature
** Version : 1.0
** Date : 2020/07/06
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**  Li.Sheng       2020/07/06        1.0           Build this feature
******************************************************************/
#ifndef _OPLUS_DISPLAY_PANEL_HBM_H_
#define _OPLUS_DISPLAY_PANEL_HBM_H_

#include <linux/err.h>
#include "dsi_display.h"
#include "dsi_panel.h"
#include "dsi_ctrl.h"
#include "dsi_ctrl_hw.h"
#include "dsi_drm.h"
#include "dsi_clk.h"
#include "dsi_pwr.h"
#include "sde_dbg.h"

int oplus_display_get_hbm_mode(void);
int __oplus_display_set_hbm(int mode);
int dsi_display_hbm_off(struct dsi_display *display);
int dsi_display_normal_hbm_on(struct dsi_display *display);
int dsi_display_hbm_on(struct dsi_display *display);
int dsi_panel_normal_hbm_on(struct dsi_panel *panel);
int dsi_panel_hbm_off(struct dsi_panel *panel);
int dsi_panel_hbm_on(struct dsi_panel *panel);
int oplus_display_panel_set_hbm(void *buf);
int oplus_display_panel_get_hbm(void *buf);
#endif /*_OPLUS_DISPLAY_PANEL_HBM_H_*/
