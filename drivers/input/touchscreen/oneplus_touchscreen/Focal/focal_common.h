/***************************************************
 * File:synaptics_common.h
 * Copyright (c)  2008- 2030  Oppo Mobile communication Corp.ltd.
 * Description:
 *             focal common driver
 * Version:1.0:
 * Date created:2017/06/18
 * Author: Cong.Dai@Bsp.Driver
 * TAG: BSP.TP.Init
 * *
 * -------------- Revision History: -----------------
 *  <author >  <data>  <version>  <desc>
 ***************************************************/

#ifndef FOCAL_H
#define FOCAL_H

/*********PART1:Head files**********************/
#include <linux/firmware.h>
#include <linux/rtc.h>
#include <linux/syscalls.h>
#include <linux/timer.h>
#include <linux/time.h>

#include "../touchpanel_common.h"

/*********PART2:Define Area**********************/
/*create apk debug channel*/
#define PROC_UPGRADE                            0
#define PROC_READ_REGISTER                      1
#define PROC_WRITE_REGISTER                     2
#define PROC_AUTOCLB                            4
#define PROC_UPGRADE_INFO                       5
#define PROC_WRITE_DATA                         6
#define PROC_READ_DATA                          7
#define PROC_SET_TEST_FLAG                      8
#define PROC_SET_SLAVE_ADDR                     10
#define PROC_HW_RESET                           11

#define WRITE_BUF_SIZE                          512
#define READ_BUF_SIZE                           512
#define FILE_NAME_LENGTH                        128

/*********PART3:Struct Area**********************/
struct focal_debug_func
{
    void (*esd_check_enable)(void *chip_data, bool enable);
    bool (*get_esd_check_flag)(void *chip_data);
    void (*reset)(void *chip_data, int msecond);
    int (*get_fw_version)(void *chip_data);
    int (*dump_reg_sate)(void *chip_data, char *buf);
};

/*********PART4:function declare*****************/
int focal_create_sysfs(struct i2c_client * client);
int focal_create_apk_debug_channel(struct touchpanel_data *ts);

#endif
