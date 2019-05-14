/***************************************************
 * File:nova_common.h
 * Copyright (c)  2008- 2030  Oppo Mobile communication Corp.ltd.
 * Description:
 *             nova common driver
 * Version:1.0:
 * Date created:2017/09/18
 * Author: Cong.Dai@Bsp.Driver
 * TAG: BSP.TP.Init
 * *
 * -------------- Revision History: -----------------
 *  <author >  <data>  <version>  <desc>
 ***************************************************/

#ifndef NOVA_H
#define NOVA_H

/*********PART1:Head files**********************/
#include <linux/firmware.h>
#include <linux/rtc.h>
#include <linux/syscalls.h>
#include <linux/timer.h>
#include <linux/time.h>

#include "../touchpanel_common.h"

/*********PART2:Define Area**********************/
struct nvt_testdata{
    int TX_NUM;
    int RX_NUM;
    int fd;
    int irq_gpio;
    int key_TX;
    int key_RX;
    uint64_t  TP_FW;
    const struct firmware *fw;
};

struct nvt_test_header {
    unsigned int magic1;
    unsigned int magic2;
    //normal mode test
    unsigned int array_fw_rawdata_P_offset;
    unsigned int array_fw_rawdata_N_offset;
    unsigned int array_open_rawdata_P_offset;
    unsigned int array_open_rawdata_N_offset;
    signed int   config_Lmt_Short_Rawdata_P;
    signed int   config_Lmt_Short_Rawdata_N;
    unsigned int config_Diff_Test_Frame;
    signed int   config_Lmt_FW_Diff_P;
    signed int   config_Lmt_FW_Diff_N;
    signed int   config_Lmt_FW_CC_P;
    signed int   config_Lmt_FW_CC_N;
    //doze mode test
    unsigned int doze_X_Channel;
    signed int   config_Lmt_Doze_Rawdata_P;
    signed int   config_Lmt_Doze_Rawdata_N;
    unsigned int config_Doze_Noise_Test_Frame;
    signed int   config_Lmt_Doze_Diff_P;
    signed int   config_Lmt_Doze_Diff_N;
    //lpwg mode test
    signed int   config_Lmt_LPWG_Rawdata_P;
    signed int   config_Lmt_LPWG_Rawdata_N;
    signed int   config_Lmt_LPWG_Diff_P;
    signed int   config_Lmt_LPWG_Diff_N;
    //offset
    unsigned int   array_Short_Rawdata_P_offset;
    unsigned int   array_Short_Rawdata_N_offset;
    unsigned int   array_FW_CC_P_offset;
    unsigned int   array_FW_CC_N_offset;
    unsigned int   array_FW_Diff_P_offset;
    unsigned int   array_FW_Diff_N_offset;
    unsigned int   array_Doze_Diff_P_offset;
    unsigned int   array_Doze_Diff_N_offset;
    unsigned int   array_Doze_Rawdata_P_offset;
    unsigned int   array_Doze_Rawdata_N_offset;
    unsigned int   array_LPWG_Rawdata_P_offset;
    unsigned int   array_LPWG_Rawdata_N_offset;
    unsigned int   array_LPWG_Diff_P_offset;
    unsigned int   array_LPWG_Diff_N_offset;
    //reserve space
    signed int   reserve[16];
};

/*********PART3:Struct Area**********************/
struct nvt_proc_operations {
    void (*auto_test)    (struct seq_file *s, void *chip_data, struct nvt_testdata *nvt_testdata);
};

/*********PART4:function declare*****************/
int nvt_create_proc(struct touchpanel_data *ts, struct nvt_proc_operations *nvt_ops);
void nvt_flash_proc_init(struct touchpanel_data *ts, const char *name);
void nvt_limit_read(struct seq_file *s, struct touchpanel_data *ts);

#endif
