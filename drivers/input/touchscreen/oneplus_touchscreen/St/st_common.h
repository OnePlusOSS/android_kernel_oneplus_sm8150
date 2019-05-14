/***************************************************
 * File:st_common.h
 * Copyright (c)  2008- 2030  Oppo Mobile communication Corp.ltd.
 * Description:
 *             st common driver
 * Version:1.0:
 * Date created:2018/10/26
 * Author: Zengpeng.Chen@Bsp.Driver
 * TAG: BSP.TP.Init
 * *
 * -------------- Revision History: -----------------
 *  <author >  <data>  <version>  <desc>
 ***************************************************/

#ifndef ST_H
#define ST_H

/*********PART1:Head files**********************/
#include <linux/firmware.h>
#include <linux/rtc.h>
#include <linux/syscalls.h>
#include <linux/timer.h>
#include <linux/time.h>

#include "../touchpanel_common.h"


//st common proc file ops
struct st_proc_operations {
    void (*auto_test)    (struct seq_file *s, void *chip_data, char *path_limits);
    int (*init_test_iterm)  (char *path_limits);
	void (*calibrate)    		(struct seq_file *s, void *chip_data);
    void (*verify_calibration)    (struct seq_file *s, void *chip_data);
};

//funtion declaration
int st_create_proc(struct touchpanel_data *ts, struct st_proc_operations *st_ops);
void st_proc_remove(struct touchpanel_data *ts);
int st_get_usb_state(void);

#endif
