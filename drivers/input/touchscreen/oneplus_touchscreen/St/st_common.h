/***********************************************************
* Description : OnePlus touchpanel driver
* 
* File		  : st_common.h 
*
* Function	  : third party interface
* 
* Source	  : provide by fts
*
* Version	  : V1.0
*
***********************************************************/
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
