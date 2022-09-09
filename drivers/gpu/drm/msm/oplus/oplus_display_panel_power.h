/***************************************************************
** Copyright (C),  2020,  OPLUS Mobile Comm Corp.,  Ltd
** VENDOR_EDIT
** File : oplus_display_panel_power.h
** Description : oplus display panel power control
** Version : 1.0
** Date : 2020/06/13
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**  Li.Sheng       2020/06/13        1.0           Build this moudle
******************************************************************/
#ifndef _OPLUS_DISPLAY_PANEL_POWER_H_
#define _OPLUS_DISPLAY_PANEL_POWER_H_

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

struct panel_vol_set {
	uint32_t panel_id;
	uint32_t panel_vol;
};

struct panel_vol_get {
	uint32_t panel_id;
	uint32_t panel_min;
	uint32_t panel_cur;
	uint32_t panel_max;
};

enum PANEL_VOLTAGE_ENUM {
	PANEL_VOLTAGE_ID_VDDI = 0,
	PANEL_VOLTAGE_ID_VDDR,
	PANEL_VOLTAGE_ID_VG_BASE,
	PANEL_VOLTAGE_ID_MAX,
};

#define PANEL_VOLTAGE_VALUE_COUNT 4

typedef struct panel_voltage_bak {
	u32 voltage_id;
	u32 voltage_min;
	u32 voltage_current;
	u32 voltage_max;
	char pwr_name[20];
} PANEL_VOLTAGE_BAK;

int oplus_display_panel_set_pwr(void *data);
int oplus_display_panel_get_pwr(void *data);
int oplus_display_panel_get_power_status(void *data);
int oplus_display_panel_set_power_status(void *data);
int __oplus_display_set_power_status(int status);
#endif /* _OPLUS_DISPLAY_PANEL_POWER_H_ */
