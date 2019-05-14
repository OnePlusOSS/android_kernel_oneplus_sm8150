/***************************************************
 * File:focal_FT8006.h
 * Copyright (c)  2008- 2030  Oppo Mobile communication Corp.ltd.
 * Description:
 *             focal FT8006 driver
 * Version:1.0:
 * Date created:2017/06/12
 * Author: Cong.Dai@Bsp.Driver
 * TAG: BSP.TP.Init
 * *
 * -------------- Revision History: -----------------
 *  <author >  <data>  <version>  <desc>
 ***************************************************/

#ifndef FOCAL_H_FT8006
#define FOCAL_H_FT8006

/*********PART1:Head files**********************/
#include <linux/i2c.h>
#ifdef CONFIG_FB
#include <linux/fb.h>
#include <linux/notifier.h>
#endif

#include "../focal_common.h"

/*********PART2:Define Area**********************/

#define RESET_TO_NORMAL_TIME 270    /*Sleep time after reset*/
#define POWEWRUP_TO_RESET_TIME 10

#define SPURIOUS_FP_LIMIT 60
#define SPURIOUS_FP_RX_NUM 11
#define SPURIOUS_FP_BASE_DATA_RETRY 10

#define GESTURE_LEFT                            0x20
#define GESTURE_RIGHT                           0x21
#define GESTURE_UP                              0x22
#define GESTURE_DOWN                            0x23
#define GESTURE_DOUBLECLICK                     0x24
#define GESTURE_O                               0x30
#define GESTURE_CLOCKWISE_O                     0x57
#define GESTURE_W                               0x31
#define GESTURE_M                               0x32
#define GESTURE_E                               0x33
#define GESTURE_L                               0x44
#define GESTURE_S                               0x46
#define GESTURE_V                               0x54
#define GESTURE_Z                               0x41
#define GESTURE_C                               0x34

#define GESTURE_LEFT_V                          0x52
#define GESTURE_RIGHT_V                         0x51
#define GESTURE_UP_V                            0x54
#define GESTURE_DOWN_V                          0x53
#define GESTURE_DOUBLE_LINE                     0x77


#define APP_OFFSET                  0x5000
#define APP_FILE_MAX_SIZE           (116 * 1024)
#define APP_FILE_MIN_SIZE           (8)
#define APP_FILE_VER_MAPPING        (0x10E + APP_OFFSET)
#define FTS_PACKET_LENGTH           128
#define FTS_UPGRADE_LOOP            30
#define FTS_FW_WRITE_CMD            0xBF

//3-gamma
#define MAX_BANK_DATA               0x80
#define MAX_GAMMA_LEN               0x180
int gamma_analog[] = { 0x003A, 0x85, 0x00, 0x00, 0x2C, 0x2B };
int gamma_digital1[] = { 0x0355, 0x8D, 0x00, 0x00, 0x80, 0x80 };
int gamma_digital2[] = { 0x03d9, 0x8D, 0x80, 0x00, 0x14, 0x13 };
int gamma_enable[] = { 0x040d, 0x91, 0x80, 0x00, 0x19, 0x01 };
union short_bits{
    u16 dshort;
    struct bits{
        u16 bit0:1;
        u16 bit1:1;
        u16 bit2:1;
        u16 bit3:1;
        u16 bit4:1;
        u16 bit5:1;
        u16 bit6:1;
        u16 bit7:1;
        u16 bit8:1;
        u16 bit9:1;
        u16 bit10:1;
        u16 bit11:1;
        u16 bit12:1;
        u16 bit13:1;
        u16 bit14:1;
        u16 bit15:1;
    }bits;
};

/*********PART3:Struct Area**********************/
enum FW_STATUS
{
    FTS_RUN_IN_ERROR,
    FTS_RUN_IN_APP,
    FTS_RUN_IN_ROM,
    FTS_RUN_IN_PRAM,
    FTS_RUN_IN_BOOTLOADER
};

struct focal_register
{
    uint8_t FTS_REG_GESTURE_EN;         /*black gesture control register*/
    uint8_t FTS_REG_CHIP_ID;            /*chip id*/
    uint8_t FTS_REG_FW_VER;             /*firamware version*/
    uint8_t FTS_REG_TOUCH_DATA;         /*touch point data*/
    uint8_t FTS_REG_GESTURE_DATA;       /*gesture point data*/
    uint8_t FTS_REG_POWER_MODE;         /*power mode*/
    uint8_t FTS_REG_EDG_CTR;            /*edge prevent mistake touch*/
    uint8_t FTS_RST_CMD_REG;            /*before upgrade, need send soft upgrade commond*/
    uint8_t FTS_ERASE_APP_REG;          /*earse app and panel paramenter*/
    uint8_t FTS_REG_VENDOR_ID;          /*save vendor id*/
    uint8_t FTS_REG_BASELINE_DATA;      /*use for reading baseline data*/
    uint8_t FTS_REG_FLOW_WORK_CNT;      /*work flow cnt*/
    uint8_t FTS_REG_CHARGER_MODE_EN;    /*charge mode state*/
    uint8_t FTS_REG_INT_CNT;            /*IC interrupt count*/
    uint8_t FTS_REG_INITCODE_VER;       /*init code version*/
    uint8_t FTS_REG_RAWDIFF_CONTR;      /*control read rawdata or delta*/
    uint8_t FTS_REG_RAW_DATA;           /*read out rawdata from this register*/
    uint8_t FTS_REG_DIFF_DATA;          /*read out diffdata from this register*/
};


struct chip_data_ft8006 {
    tp_dev                          tp_type;
    struct i2c_client               *client;
    struct focal_register           reg_info;
    struct hw_resource              *hw_res;
    int16_t                         *spuri_fp_data;
    bool                            is_sleep_reg_write;    /*true:ic in sleep mode*/
    bool                            esd_check_need_stop;   /*true:esd check do nothing*/
};
#endif
