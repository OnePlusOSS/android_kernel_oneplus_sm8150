/**************************************************************
 * Copyright (c)  2008- 2030  Oppo Mobile communication Corp.ltd.
 * File       : goodix_drivers_gt9286.h
 * Description: header file for Goodix GT9286 driver
 * Version   : 1.0
 * Date        : 2017-08-01
 * Author    : Cong.Dai@Bsp.Group.Tp
 * TAG         : BSP.TP.Init
 * ---------------- Revision History: --------------------------
 *   <version>    <date>          < author >                            <desc>
 ****************************************************************/

#ifndef TOUCHPANEL_GOODIX_OPPO_H
#define TOUCHPANEL_GOODIX_OPPO_H

#include "../goodix_common.h"
#include "../goodix_tool.h"

/****************************Start of define declare****************************/

#define set_reg_bit(reg, pos, val)        ((reg) = ((reg) & (~(1 << (pos)))) | (!!(val) << (pos)))

#define MAX_POINT_NUM          10       //max touch point number this ic support
#define MAX_GT_IRQ_DATA_LENGTH 90       //irq data(points,key,checksum) size read from irq
#define MAX_GESTURE_POINT_NUM  64       //max point number of black gesture

#define GTP_DRIVER_SEND_CFG   1         // send config to TP while initializing (for no config built in TP's flash)

//Request type define
#define GTP_RQST_CONFIG                 0x01
#define GTP_RQST_BAK_REF                0x02
#define GTP_RQST_RESET                  0x03
#define GTP_RQST_MAIN_CLOCK             0x04
#define GTP_RQST_HOTKNOT_CODE           0x20
#define GTP_RQST_RESPONDED              0x00
#define GTP_RQST_IDLE                   0xFF

//command define
#define GTP_CMD_SLEEP                   0x05
#define GTP_CMD_GESTURE                 0x08
#define GTP_CMD_CHARGER_ON              0x06
#define GTP_CMD_CHARGER_OFF             0x07
#define GTP_CMD_GESTURE_WAKEUP          0x08
#define GTP_CMD_CLEAR_CFG               0x10
#define GTP_CMD_ESD                     0xAA
#define GTP_CMD_HN_TRANSFER             0x22
#define GTP_CMD_HN_EXIT_SLAVE           0x28

#define GTP_CMD_ENABLE_EDGE             0X41
#define GTP_CMD_DISABLE_EDGE            0X42

#define GTP_CMD_ENTER_WATER_PROTECT     0X44
#define GTP_CMD_EXIT_WATER_PROTECT      0X45

//gesture type fw send
#define DTAP_DETECT                     0xCC
#define UP_VEE_DETECT                   0x76
#define DOWN_VEE_DETECT                 0x5e
#define LEFT_VEE_DETECT                 0x3e
#define RIGHT_VEE_DETECT                0x63 //this gesture is C
#define RIGHT_VEE_DETECT2               0x3c //this gesture is <
#define CIRCLE_DETECT                   0x6f
#define DOUSWIP_DETECT                  0x48
#define RIGHT_SLIDE_DETECT              0xAA
#define LEFT_SLIDE_DETECT               0xbb
#define DOWN_SLIDE_DETECT               0xAB
#define UP_SLIDE_DETECT                 0xBA
#define M_DETECT                        0x6D
#define W_DETECT                        0x77
/****************************End of define declare***************************/

/****************************Start of config data****************************/
#define GTP_CONFIG_MIN_LENGTH    186
#define GTP_CONFIG_MAX_LENGTH    240

//define offset in the config
#define RESOLUTION_LOCATION      1
#define TRIGGER_LOCATION         6
#define MODULE_SWITCH3_LOCATION  8

//Normal config setting
u8 GTP_CFG_GROUP_SAMSUNG[] = {\
    0x02,0x38,0x04,0x70,0x08,0x3A,0x3D,0x04,0xA8,0xE2,0x0E,0x6C,0x50,0x32,0x50,0x01,0x11,0x0C,0x10,0x00,\
    0x46,0x80,0x96,0xDC,0x48,0x0A,0x03,0x75,0x1E,0x3C,0x00,0x44,0x00,0x51,0x00,0x00,0x12,0x14,0x00,0x40,\
    0x3C,0x1E,0x17,0x28,0x48,0x00,0x87,0x28,0x1E,0x4D,0x4F,0xDF,0x07,0x38,0x7E,0x18,0x2D,0x54,0x24,0x44,\
    0x03,0x28,0x7D,0xC0,0x54,0xF2,0x00,0x0B,0x04,0xC8,0x36,0xA3,0x44,0x8D,0x52,0x80,0x60,0x78,0x6E,0x74,\
    0x42,0x4B,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0F,0x1E,0x08,0x00,0x46,0xF5,0x40,\
    0x69,0x6C,0x6D,0x53,0x26,0x75,0x00,0x00,0x01,0x73,0x0A,0x32,0xFC,0x9C,0x84,0x00,0x00,0x00,0x00,0x00,\
    0x00,0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x56,0xDF,0x07,0x50,0x28,\
    0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x1A,0x1B,0x1C,0x1D,0x1E,0x1F,0x0F,0x0C,0x0D,0x0E,\
    0x09,0x08,0x07,0x06,0x02,0x04,0x03,0x05,0x0A,0x0B,0xFF,0xFF,0x1A,0x18,0x19,0x04,0x05,0x06,0x07,0x08,\
    0x09,0x0B,0x0F,0x0C,0x0E,0x0D,0x0A,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x30,0x40,0x83,0x00,0x05,\
    0x04,0x0C,0x81,0x23,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x14,0x88,0x34,0x65,\
    0x78,0x00,0x14,0x50,0x6D,0x31,0x14,0x50,0x00,0x44,0x10,0x00,0x23,0x9D,0x09,0x26,0xF5,0xCF,0x01};

/*****************************End of config data*****************************/

/***************************Start of struct declare**************************/
struct fw_subsystem_info {
    int type;
    int length;
    u32 address;
    int offset;
};

struct fw_info {
    u32 length;
    u16 checksum;
    u8 target_mask[6];
    u8 target_mask_version[3];
    u8 pid[6];
    u8 version[3];
    u8 subsystem_count;
    u8 chip_type;
    u8 reserved[6];
    struct fw_subsystem_info subsystem[12];
};


struct goodix_register {
    uint16_t GTP_REG_FW_CHK_MAINSYS;        /*mainsys reg used to check fw status*/
    uint16_t GTP_REG_FW_CHK_SUBSYS;         /*subsys reg used to check fw status*/
    uint16_t GTP_REG_CONFIG_DATA;           /*configure firmware*/
    uint16_t GTP_REG_READ_COOR;             /*touch state and info*/
    uint16_t GTP_REG_PRODUCT_VER;           /*product id & version*/
    uint16_t GTP_REG_WAKEUP_GESTURE;        /*gesture type*/
    uint16_t GTP_REG_GESTURE_COOR;          /*gesture point data*/
    uint16_t GTP_REG_CMD;                   /*recevice cmd from host*/
    uint16_t GTP_REG_RQST;                  /*request from ic*/
    uint16_t GTP_REG_NOISE_DETECT;          /*noise state*/
    uint16_t GTP_REG_WATER_PROTECT_QUERY;    /*water protect state*/

    uint16_t GTP_REG_RAWDATA;
    uint16_t GTP_REG_DIFFDATA;
    uint16_t GTP_REG_BASEDATA;

    uint16_t _rRW_MISCTL__SWRST_B0_;
    uint16_t _bRW_MISCTL__DSP_MCU_PWR_;
    uint16_t _bRW_MISCTL__TMR0_EN;
    uint16_t _bRW_MISCTL__CACHE_EN;
    uint16_t _bWO_MISCTL__CPU_SWRST_PULSE;
    uint16_t _rRW_MISCTL__BOOT_OPT_B0_;
    uint16_t _bRW_MISCTL__SRAM_BANK;
    uint16_t _bRW_MISCTL__PATCH_AREA_EN_;
};


struct config_info {
    char    *goodix_config_ver;
    u8      *goodix_config;
    u8      goodix_int_type;
    u8      goodix_wakeup_level;
    u32     goodix_abs_x_max;
    u32     goodix_abs_y_max;
    u32     goodix_config_len;
};

struct goodix_version_info {
    u8   product_id[5];
    u32  patch_id;
    u32  mask_id;
    u8   sensor_id;
    u8   match_opt;
};

struct  chip_data_gt9286 {
    bool                                halt_status;                    //1: need ic reset
    u8                                  *touch_data;
    tp_dev                              tp_type;
    u16                                 *spuri_fp_touch_raw_data;
    struct i2c_client                   *client;
    struct goodix_version_info          ver_info;
    struct config_info                  config_info;
    struct goodix_register              reg_info;
    struct fw_update_info               update_info;
    struct hw_resource                  *hw_res;
    struct goodix_proc_operations       *goodix_ops;                    //goodix func provide for debug
    bool                                esd_check_enabled;
    unsigned long                       irq_timer;
    uint8_t                             esd_retry;
};

/****************************End of struct declare***************************/

extern int gt1x_rawdiff_mode;

#endif/*TOUCHPANEL_GOODIX_OPPO_H*/


