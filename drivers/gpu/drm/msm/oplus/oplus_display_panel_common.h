/***************************************************************
** Copyright (C),  2020,  OPLUS Mobile Comm Corp.,  Ltd
** VENDOR_EDIT
** File : oplus_display_panel_common.c
** Description : oplus display panel common feature
** Version : 1.0
** Date : 2020/06/13
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**  Li.Sheng       2020/06/13        1.0           Build this moudle
******************************************************************/
#ifndef _OPLUS_DISPLAY_PANEL_COMMON_H_
#define _OPLUS_DISPLAY_PANEL_COMMON_H_

#include <linux/err.h>
#include "dsi_display.h"
#include "dsi_panel.h"
#include "dsi_ctrl.h"
#include "dsi_ctrl_hw.h"
#include "dsi_drm.h"
#include "dsi_clk.h"
#include "dsi_pwr.h"
#include "sde_dbg.h"
#include "oplus_dsi_support.h"
#include "oplus_display_panel.h"
struct panel_id
{
	uint32_t DA;
	uint32_t DB;
	uint32_t DC;
};

struct panel_info
{
	char version[32];
	char manufacture[32];
};

struct panel_serial_number
{
	uint32_t date;		 /*year:month:day:hour  8bit uint*/
	uint32_t precise_time; /* minute:second:reserved:reserved 8bit uint*/
};

int oplus_display_panel_get_id(void *buf);
int oplus_display_panel_get_max_brightness(void *buf);
int oplus_display_panel_set_max_brightness(void *buf);
int oplus_display_panel_get_vendor(void *buf);
int oplus_display_panel_get_ccd_check(void *buf);
int oplus_display_panel_get_serial_number(void *buf);
int oplus_display_get_panel_parameters(struct dsi_panel *panel, struct dsi_parser_utils *utils);
int oplus_display_panel_get_closebl_flag(void *data);
int oplus_display_panel_set_closebl_flag(void *data);
int oplus_display_panel_get_brightness(void *buf);
int oplus_display_set_aod_area(void *buf);
int oplus_display_panel_set_dimlayer_hbm(void *data);
#endif /*_OPLUS_DISPLAY_PANEL_COMMON_H_*/
