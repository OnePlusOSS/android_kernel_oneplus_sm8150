/***************************************************
 * File:gt1x_oppo.h
 * Copyright (c)  2008- 2030  Oppo Mobile communication Corp.ltd.
 * Description:
 *             goodix gt5688 driver
 * Version:1.0:
 * Date created:2016/09/02
 * Author: Tong.han@Bsp.Driver
 * TAG: BSP.TP.Init
 * *
 * -------------- Revision History: -----------------
 *  <author >  <data>  <version>  <desc>
 ***************************************************/

#ifndef TOUCHPANEL_GOODIX_DRIVER_H
#define TOUCHPANEL_GOODIX_DRIVER_H

#include "../goodix_common.h"
#include "../goodix_tool.h"
#include "gt1x_generic_driver.h"

#define set_reg_bit(reg, pos, val)        ((reg) = ((reg) & (~(1 << (pos)))) | (!!(val) << (pos)))

#define MAX_GT_IRQ_DATA_LENGTH 90  //irq data(points,key,checksum) size read from irq.

/****************************PART1:ON/OFF define****************************/
#define GTP_DRIVER_SEND_CFG   1    // send config to TP while initializing (for no config built in TP's flash)

/****************************PART2:Request type define*******************************/
#define GTP_RQST_CONFIG                 0x01
#define GTP_RQST_BAK_REF                0x02
#define GTP_RQST_RESET                  0x03
#define GTP_RQST_MAIN_CLOCK             0x04
#define GTP_RQST_HOTKNOT_CODE           0x20
#define GTP_RQST_RESPONDED              0x00
#define GTP_RQST_IDLE                   0xFF

/****************************PART3:CMD define*******************************/
#define GTP_CMD_SLEEP                   0x05
#define GTP_CMD_CHARGER_ON              0x06
#define GTP_CMD_CHARGER_OFF             0x07
#define GTP_CMD_GESTURE_WAKEUP          0x08
#define GTP_CMD_CLEAR_CFG               0x10
#define GTP_CMD_ESD                     0xAA
#define GTP_CMD_HN_TRANSFER             0x22
#define GTP_CMD_HN_EXIT_SLAVE           0x28

/****************************PART4:Gesture detect****************************/
#define DTAP_DETECT                     0xCC
#define UP_VEE_DETECT                   0x76
#define DOWN_VEE_DETECT                 0x5e
#define LEFT_VEE_DETECT                 0x3e 
#define RIGHT_VEE_DETECT                0x63
#define CIRCLE_DETECT                   0x6f
#define DOUSWIP_DETECT                  0x48
#define DOUUPSWIP_DETECT                0x4E
#define RIGHT_SLIDE_DETECT              0xAA
#define LEFT_SLIDE_DETECT               0xbb
#define DOWN_SLIDE_DETECT               0xAB
#define UP_SLIDE_DETECT                 0xBA
#define M_DETECT                        0x6D
#define W_DETECT                        0x77

/*****************************PART5:SPURIOUS_FP and edge_limit********************/
#define SPURIOUS_FP_RX_NUM              6
#define GOODIX_ENABLE_EDGE              0X51
#define GOODIX_DISABLE_EDGE             0X52


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

/****************************PART6:Reg *************************************/
struct Goodix_register {
    /*Register define */
#define GTP_READ_COOR_ADDR              0x814E
#define GTP_REG_CMD                     0x8040
#define GTP_REG_SENSOR_ID               0x814A
#define GTP_REG_CONFIG_DATA             0x8050
#define GTP_REG_CONFIG_RESOLUTION       0x8051
#define GTP_REG_CONFIG_TRIGGER          0x8056
#define GTP_REG_CONFIG_CHECKSUM         0x813C
#define GTP_REG_CONFIG_UPDATE           0x813E
#define GTP_REG_VERSION                 0x8140
#define GTP_REG_HW_INFO                 0x4220
#define GTP_REG_REFRESH_RATE            0x8056
#define GTP_REG_ESD_CHECK               0x8043
#define GTP_REG_FLASH_PASSBY            0x8006
#define GTP_REG_HN_PAIRED               0x81AA
#define GTP_REG_HN_MODE                 0x81A8
#define GTP_REG_MODULE_SWITCH3          0x8058
#define GTP_REG_FW_CHK_MAINSYS          0x41E4
#define GTP_REG_FW_CHK_SUBSYS           0x5095
#define GTP_NOISE_DETECT_REG            0x804B

#define GTP_REG_WAKEUP_GESTURE          0x814C
#define GTP_REG_WAKEUP_GESTURE_DETAIL   0x8A40

#define GTP_REG_MATRIX_DRVNUM           0x8069
#define GTP_REG_MATRIX_SENNUM           0x806A
#define GTP_REG_RQST                    0x8044
#define GTP_REG_BAK_REF                 0x90EC
#define GTP_REG_MAIN_CLK                0x8020
#define GTP_REG_HAVE_KEY                0x8057
#define GTP_REG_HN_STATE                0x8800

#define _bRW_MISCTL__SRAM_BANK          0x4048
#define _bRW_MISCTL__MEM_CD_EN          0x4049
#define _bRW_MISCTL__CACHE_EN           0x404B
#define _bRW_MISCTL__TMR0_EN            0x40B0
#define _rRW_MISCTL__SWRST_B0_          0x4180
#define _bWO_MISCTL__CPU_SWRST_PULSE    0x4184
#define _rRW_MISCTL__BOOTCTL_B0_        0x4190
#define _rRW_MISCTL__BOOT_OPT_B0_       0x4218
#define _rRW_MISCTL__BOOT_CTL_          0x5094
#define _bRW_MISCTL__DSP_MCU_PWR_       0x4010
#define _bRW_MISCTL__PATCH_AREA_EN_     0x404D

#define GTP_REG_RAWDATA                 0xCBB4
#define GTP_REG_DIFFDATA                0xC8DC
#define GTP_REG_BASEDATA                0x84D0
};

typedef enum {
    CFG_NORMAL,
    CFG_NOISE,
    CFG_CHARGER,
}cfg_config;

struct config_info {
    char *config_version;
    u8   *gt1x_config;
    u8   gt1x_int_type;
    u8   gt1x_wakeup_level;
    u32  gt1x_abs_x_max;
    u32  gt1x_abs_y_max;
    u32  gt1x_cfg_length;
};

struct gt1x_version_info {
    u8   product_id[5];
    u32  patch_id;
    u32  mask_id;
    u8   sensor_id;
    u8   match_opt;
};

struct  chip_data_gt5688 {
    bool halt_status;
    u8   coor_base;
    u8   vk_status ;         /*state of key "select|select(keycode)|menu|menu(keycode)|home|home(keycode)|back|back(keycode)"*/
    u8   *touch_data;
    u16  *spuri_fp_touch_raw_data;                  /*save spurious_finger_support raw data*/

    tp_dev tp_type;
    struct i2c_client           *client;

    struct gt1x_version_info    ver_info;
    struct config_info          config_info;
    struct Goodix_register      reg_info;
    struct fw_update_info       update_info;
    struct hw_resource          *hw_res;
    struct goodix_proc_operations *goodix_ops;     //goodix func provide for debug
};

extern int gt1x_rawdiff_mode;

#endif/*TOUCHPANEL_GOODIX_DRIVER_H*/


