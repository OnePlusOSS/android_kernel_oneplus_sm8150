/***************************************************************
** Copyright (C),  2020,  OPLUS Mobile Comm Corp.,  Ltd
** VENDOR_EDIT
** File : oplus_ffl.h
** Description : oplus ffl feature
** Version : 1.0
** Date : 2020/04/23
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**   Qianxu         2020/04/23        1.0           Build this moudle
******************************************************************/
#ifndef _OPLUS_FFL_H_
#define _OPLUS_FFL_H_

#include <linux/kthread.h>


void oplus_ffl_set(int enable);

void oplus_ffl_setting_thread(struct kthread_work *work);

void oplus_start_ffl_thread(void);

void oplus_stop_ffl_thread(void);

int oplus_ffl_thread_init(void);

void oplus_ffl_thread_exit(void);

int oplus_display_panel_set_ffl(void *buf);
int oplus_display_panel_get_ffl(void *buf);

#endif /* _OPLUS_FFL_H_ */
