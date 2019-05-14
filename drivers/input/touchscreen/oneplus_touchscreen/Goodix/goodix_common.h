/***************************************************
 * File:goodix_common.h
 * Copyright(C) 2008-2012 OPPO Mobile Comm Corp., Ltd
 * Description:
 *             goodix common driver
 * Version:1.0:
 * Date created:2016/09/02
 * Author: Tong.han@Bsp.Driver
 * TAG: BSP.TP.Init
 * *
 * -------------- Revision History: -----------------
 *  <author >  <data>  <version>  <desc>
 ***************************************************/

#ifndef _TOUCHPANEL_COMMON_GOODIX_H_
#define _TOUCHPANEL_COMMON_GOODIX_H_

#include "../touchpanel_common.h"

#define IS_NUM_OR_CHAR(x) (((x) >= 'A' && (x) <= 'Z') || ((x) >= '0' && (x) <= '9'))

/****************************PART1:FW Update define*************************************/
#define FW_HEAD_SIZE                         128
#define FW_HEAD_SUBSYSTEM_INFO_SIZE          8
#define FW_HEAD_OFFSET_SUBSYSTEM_INFO_BASE   32
#define PACK_SIZE                            256
#define UPDATE_STATUS_IDLE                   0
#define UPDATE_STATUS_RUNNING                1
#define UPDATE_STATUS_ABORT                  2
#define FW_SECTION_TYPE_SS51_PATCH           0x02

typedef enum {
    GTP_RAWDATA,
    GTP_DIFFDATA,
    GTP_BASEDATA,
}debug_type;

struct goodix_proc_operations {
    void (*goodix_config_info_read)(struct seq_file *s, void *chip_data);
    size_t (*goodix_water_protect_write) (struct file *file, const char *buff,  size_t len, loff_t *pos);
    size_t (*goodix_water_protect_read) (struct file *file, char *buff, size_t len, loff_t *pos);
};

struct fw_update_info {
    u8 *buffer;
    u8 *firmware_file_data;
    u32 fw_length;

    int status;
    int progress;
    int max_progress;
    struct fw_info *firmware;
};

/****************************PART2:FUNCTION*************************************/
void GetCirclePoints(struct Coordinate *input_points, int number, struct Coordinate  *pPnts);
int  ClockWise(struct Coordinate *p, int n);
int Goodix_create_proc(struct touchpanel_data *ts, struct goodix_proc_operations *goodix_ops);

#endif

