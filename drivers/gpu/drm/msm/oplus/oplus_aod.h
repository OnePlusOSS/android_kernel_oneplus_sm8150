/***************************************************************
** Copyright (C),  2020,  OPLUS Mobile Comm Corp.,  Ltd
** VENDOR_EDIT
** File : oplus_aod.h
** Description : oplus aod feature
** Version : 1.0
** Date : 2020/04/23
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**   Qianxu         2020/04/23        1.0           Build this moudle
******************************************************************/
#ifndef _OPLUS_AOD_H_
#define _OPLUS_AOD_H_

#include "dsi_display.h"
#include "oplus_dsi_support.h"

int dsi_display_aod_on(struct dsi_display *display);

int dsi_display_aod_off(struct dsi_display *display);

int oplus_update_aod_light_mode_unlock(struct dsi_panel *panel);

int oplus_update_aod_light_mode(void);

int oplus_panel_set_aod_light_mode(void *buf);
int oplus_panel_get_aod_light_mode(void *buf);
int __oplus_display_set_aod_light_mode(int mode);

#endif /* _OPLUS_AOD_H_ */
