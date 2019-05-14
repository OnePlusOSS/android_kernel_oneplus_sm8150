/*****************************************************************************************
 * Copyright (c)  2008- 2030  Oppo Mobile communication Corp.ltd.
 * File       : focal_drivers_ft8006.c
 * Description: Source file for focal FT8006 driver
 * Version   : 1.0
 * Date        : 2017-06-12
 * Author    : Cong.Dai@Bsp.Group.Tp
 * TAG         : BSP.TP.Init
 * ---------------- Revision History: --------------------------
 *   <version>    <date>          < author >                            <desc>
 *******************************************************************************************/

#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/hrtimer.h>
#include <linux/kthread.h>
#include <linux/random.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/sched/rt.h>
#include <linux/task_work.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/machine.h>
#include <linux/regulator/consumer.h>

#ifdef CONFIG_FB
#include <linux/fb.h>
#include <linux/notifier.h>
#endif

#include "focal_drivers_ft8006.h"

/****************** Start of Log Tag Declear and level define*******************************/
#define TPD_DEVICE "focal-ft8006"
#define TPD_INFO(a, arg...)  pr_err("[TP]"TPD_DEVICE ": " a, ##arg)
#define TPD_DEBUG(a, arg...)\
    do{\
        if (LEVEL_DEBUG == tp_debug)\
            pr_err("[TP]"TPD_DEVICE ": " a, ##arg);\
    }while(0)

#define TPD_DETAIL(a, arg...)\
    do{\
        if (LEVEL_BASIC != tp_debug)\
            pr_err("[TP]"TPD_DEVICE ": " a, ##arg);\
    }while(0)

#define TPD_DEBUG_NTAG(a, arg...)\
    do{\
        if (tp_debug)\
            printk(a, ##arg);\
    }while(0)
/******************** End of Log Tag Declear and level define*********************************/

struct chip_data_ft8006 *g_chip_info = NULL;
/*************************** start of function delcare****************************************/
static int focal_upgrade_use_buf(void *chip_data, u8 *pbt_buf, u32 dw_lenth);
static int focal_lcd_cfg_upgrade_use_buf(void *chip_data, u8* pbt_buf, u32 dw_lenth);
static void focal_esd_check_enable(void *chip_data, bool enable);
static int focal_get_chip_info(void *chip_data);
void __attribute__((weak)) lcd_esd_recovery_mutex(bool enable) {return;}    //lcd esd recovery mutex

/**************************** end of function delcare*****************************************/


/****** Start of other functions that work for oppo_touchpanel_operations callbacks***********/
static int focal_enter_sleep(struct chip_data_ft8006 *chip_info, bool config)
{
    int ret = 1;

    if (config) {
        ret = touch_i2c_write_byte(chip_info->client, chip_info->reg_info.FTS_REG_POWER_MODE, 0x03);
        if (ret < 0) {
            TPD_INFO("%s: enter sleep mode failed!\n", __func__);
            return -1;
        } else {
            chip_info->is_sleep_reg_write = true;
            TPD_INFO("%s: enter sleep mode sucess!\n", __func__);
        }
    }

    return ret;
}

void focal_rst_pin_output(bool enable)
{
    static bool rst_pulled_down = false;

    if (g_chip_info && gpio_is_valid(g_chip_info->hw_res->reset_gpio)) {
        gpio_direction_output(g_chip_info->hw_res->reset_gpio, enable);
        if (!enable) {
            rst_pulled_down = true;
        } else {
            if (rst_pulled_down) {
                g_chip_info->is_sleep_reg_write = false;
            }
            rst_pulled_down = false;
        }
        TPD_INFO("%s: enable = %d\n", __func__, enable);
    }
}

static void focal_reset_via_gpio(void *chip_data, int mscond)
{
    struct chip_data_ft8006 *chip_info = (struct chip_data_ft8006 *)chip_data;

    if (gpio_is_valid(chip_info->hw_res->reset_gpio)) {
        gpio_direction_output(chip_info->hw_res->reset_gpio, 0);
        msleep(20);
        gpio_direction_output(chip_info->hw_res->reset_gpio, 1);
        msleep(mscond);
        chip_info->is_sleep_reg_write = false;
    }

}

static int focal_reset(void *chip_data)
{
    struct chip_data_ft8006 *chip_info = (struct chip_data_ft8006 *)chip_data;
    focal_reset_via_gpio(chip_info, RESET_TO_NORMAL_TIME);

    TPD_INFO("%s.\n", __func__);
    return 0;
}

static int focal_enable_black_gesture(struct chip_data_ft8006 *chip_info, bool enable)
{
    int state = -1;
    int ret = -1;
    int i = 0;

    TPD_INFO("%s, enable = %d\n", __func__, enable);
    if (chip_info->is_sleep_reg_write) {
        focal_reset(chip_info);
    }

    for (i = 0; i < 5; i++) {
        ret = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.FTS_REG_CHIP_ID);
        if (ret != 0x80) {
            TPD_INFO("%s: read chip id failed,ret:0x%02x, try %d(<4 is ok) times\n", __func__, ret, i);
            msleep(10);
        } else {
            break;
        }
    }

    if (enable) {
        for (i = 0; i < 5; i++) {
            touch_i2c_write_byte(chip_info->client, 0xd1, 0xff);
            touch_i2c_write_byte(chip_info->client, 0xd2, 0xff);
            touch_i2c_write_byte(chip_info->client, 0xd5, 0xff);
            touch_i2c_write_byte(chip_info->client, 0xd6, 0xff);
            touch_i2c_write_byte(chip_info->client, 0xd7, 0xff);
            touch_i2c_write_byte(chip_info->client, chip_info->reg_info.FTS_REG_GESTURE_EN, 0x01);
            msleep(1);
            state = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.FTS_REG_GESTURE_EN);
            if (state == 1)
                break;
        }
    } else {
        for (i = 0; i < 5; i++)
        {
            touch_i2c_write_byte(chip_info->client, chip_info->reg_info.FTS_REG_GESTURE_EN, 0x00);
            msleep(1);
            state = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.FTS_REG_GESTURE_EN);
            if (state == 0)
                break;
        }
    }

    if (i >= 5) {
        ret = -1;
        TPD_INFO("%s: enter black gesture state: %d failed\n", __func__, enable);
    } else {
        TPD_INFO("%s: enter black gesture state: %d success\n", __func__, enable);
    }
    return ret;
}

static int focal_enable_edge_limit(struct chip_data_ft8006 *chip_info, bool enable)
{
    int ret = -1;

    TPD_INFO("%s: edge limit enable = %d\n", __func__, enable);
    if (enable) {
        ret = touch_i2c_write_byte(chip_info->client, chip_info->reg_info.FTS_REG_EDG_CTR, 0x00);
    } else {
        ret = touch_i2c_write_byte(chip_info->client, chip_info->reg_info.FTS_REG_EDG_CTR, 0x01);
    }

    if (ret < 0) {
        TPD_INFO("%s: enable edge limit state: %d failed!\n", __func__, enable);
    } else {
        TPD_INFO("%s: enable edge limit state: %d success!\n", __func__, enable);
    }
    return ret;
}

static int focal_enable_charge_mode(struct chip_data_ft8006 *chip_info, bool enable)
{
    int ret = -1;

    if (enable) {
        ret = touch_i2c_write_byte(chip_info->client, chip_info->reg_info.FTS_REG_CHARGER_MODE_EN, 0x01);
    } else {
        ret = touch_i2c_write_byte(chip_info->client, chip_info->reg_info.FTS_REG_CHARGER_MODE_EN, 0x00);
    }

    if (ret < 0) {
        TPD_INFO("%s: enable charge state: %d failed!\n", __func__, enable);
    } else {
        TPD_INFO("%s: enable charge state: %d success!\n", __func__, enable);
    }
    return ret;
}

static void getSpecialPoint(uint8_t *buf, int n, struct Coordinate *point)
{
    int x, y, i;

    point[0].x = (buf[0] & 0x0F) << 8 | (buf[1] & 0xFF);
    point[0].y = (buf[2] & 0x0F) << 8 | (buf[3] & 0xFF);
    point[1] = point[0];
    point[2] = point[0];
    point[3] = point[0];
    for (i = 0; i < n; i++) {
        x = (buf[0 + 4*i] & 0x0F) << 8 | (buf[1 + 4*i] & 0xFF);
        y = (buf[2 + 4*i] & 0x0F) << 8 | (buf[3 + 4*i] & 0xFF);
        if (point[3].x < x) {   //xmax
            point[3].x = x;
            point[3].y = y;
        }
        if (point[1].x > x) {   //xmin
            point[1].x = x;
            point[1].y = y;
        }
        if (point[2].y < y) {   //ymax
            point[2].y = y;
            point[2].x = x;
        }
        if (point[0].y > y) {   //ymin
            point[0].y = y;
            point[0].x = x;
        }
    }
}
/****** End of other functions that work for oppo_touchpanel_operations callbacks*************/

/********* Start of implementation of oppo_touchpanel_operations callbacks********************/
static int focal_ftm_process(void *chip_data)
{
    int ret = -1;
    struct chip_data_ft8006 *chip_info = (struct chip_data_ft8006 *)chip_data;

    TPD_INFO("%s is called!\n", __func__);
    ret = focal_get_chip_info(chip_info);
    if (!ret) {
        ret = focal_enter_sleep(chip_info, true);
    }

    return ret;
}

static int focal_get_vendor(void *chip_data, struct panel_info *panel_data)
{
    int len = 0;
    char manu_temp[MAX_DEVICE_MANU_LENGTH] = FOCAL_PREFIX;
    struct chip_data_ft8006 *chip_info = (struct chip_data_ft8006 *)chip_data;

    len = strlen(panel_data->fw_name);
    if ((len > 3) && (panel_data->fw_name[len-3] == 'i') && \
        (panel_data->fw_name[len-2] == 'm') && (panel_data->fw_name[len-1] == 'g')) {
        panel_data->fw_name[len-3] = 'b';
        panel_data->fw_name[len-2] = 'i';
        panel_data->fw_name[len-1] = 'n';
    }
    chip_info->tp_type = panel_data->tp_type;
    strlcat(manu_temp, panel_data->manufacture_info.manufacture, MAX_DEVICE_MANU_LENGTH);
    strncpy(panel_data->manufacture_info.manufacture, manu_temp, MAX_DEVICE_MANU_LENGTH);
    TPD_INFO("chip_info->tp_type = %d, panel_data->fw_name = %s\n", chip_info->tp_type, panel_data->fw_name);

    return 0;
}

static int focal_get_chip_info(void *chip_data)
{
    int i = 0;
    int ret = -1;
    struct chip_data_ft8006 *chip_info = (struct chip_data_ft8006 *)chip_data;
    struct focal_register   *reg_info = &chip_info->reg_info;

    reg_info->FTS_REG_GESTURE_EN = 0xD0;
    reg_info->FTS_REG_CHIP_ID = 0xA3;
    reg_info->FTS_REG_FW_VER = 0xA6;
    reg_info->FTS_REG_INITCODE_VER = 0xE4;
    reg_info->FTS_REG_TOUCH_DATA = 0x00;
    reg_info->FTS_REG_POWER_MODE = 0xA5;
    reg_info->FTS_REG_EDG_CTR = 0x8C;
    reg_info->FTS_REG_GESTURE_DATA = 0xD3;
    reg_info->FTS_RST_CMD_REG = 0xFC;
    reg_info->FTS_ERASE_APP_REG = 0x61;
    reg_info->FTS_REG_VENDOR_ID = 0xA8;
    reg_info->FTS_REG_BASELINE_DATA = 0x7B;
    reg_info->FTS_REG_FLOW_WORK_CNT = 0x91;
    reg_info->FTS_REG_CHARGER_MODE_EN = 0x8B;
    reg_info->FTS_REG_INT_CNT = 0x8F;
    reg_info->FTS_REG_RAWDIFF_CONTR = 0xEF;
    reg_info->FTS_REG_RAW_DATA = 0x75;
    reg_info->FTS_REG_DIFF_DATA = 0x77;

    for (i = 0; i < 3; i++) {
        ret = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.FTS_REG_CHIP_ID);
        if (ret == 0x80) {  //0x80 is ok
            break;
        }
        msleep(90);
    }
    if (i == 3) {
        TPD_INFO("%s: read chip id failed(ret:%d), ic work abnormal\n", __func__, ret);
    }

    return 0;
}

static int focal_power_control(void *chip_data, bool enable)
{
    struct chip_data_ft8006 *chip_info = (struct chip_data_ft8006 *)chip_data;

    if (enable) {
        gpio_direction_output(chip_info->hw_res->reset_gpio, 1);
        //msleep(RESET_TO_NORMAL_TIME);   //after lcd reset, tp need wait ic work
    }
    return 0;
}

static fw_check_state focal_fw_check(void *chip_data, struct resolution_info *resolution_info, struct panel_info *panel_data)
{
    int ret = 0;
    uint8_t ver_len = 0;
    char dev_version[MAX_DEVICE_VERSION_LENGTH] = {0};
    struct chip_data_ft8006 *chip_info = (struct chip_data_ft8006 *)chip_data;

    ret = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.FTS_REG_CHIP_ID);
    if (ret != 0x80) {
        TPD_INFO("Read chip Id failed, need Update the Firmware, ret: %d\n", ret);
        return FW_ABNORMAL;
    }

    ret = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.FTS_REG_FW_VER);
    if (ret <= 0) {
        TPD_INFO("Read firmware version failed, need Update the Firmware, ret: %d\n", ret);
        return FW_ABNORMAL;
    } else {
        panel_data->TP_FW = ret;
        sprintf(dev_version, "%02x", panel_data->TP_FW);
        if (panel_data->manufacture_info.version) {
            ver_len = strlen(panel_data->manufacture_info.version);
            if (ver_len <= 8) {
                strlcat(panel_data->manufacture_info.version, dev_version, MAX_DEVICE_VERSION_LENGTH);
            } else {
                strlcpy(&panel_data->manufacture_info.version[8], dev_version, MAX_DEVICE_VERSION_LENGTH - 8);
            }
        }
    }
    return FW_NORMAL;
}

static fw_update_state focal_fw_update(void *chip_data, const struct firmware *fw, bool force)
{
    int ret = 0;
    int i = 0;
    int initcode_len = 0;
    fw_update_state update_state = FW_NO_NEED_UPDATE;
    uint8_t PANEL_ID_IN_FLASH = 0, PANEL_ID_IN_BIN = 0;
    uint8_t TP_CURRENT_FIRMWARE_ID = 0, TP_FIRMWARE_ID = 0;
    uint8_t LCD_CURRENT_FIRMWARE_ID = 0, LCD_FIRMWARE_ID = 0;
    struct chip_data_ft8006 *chip_info = (struct chip_data_ft8006 *)chip_data;
    u8 *fw_data = NULL;
    int fw_len = fw->size;

    if (!chip_info) {
        TPD_INFO("Chip info is NULL\n");
        return 0;
    }

    TPD_INFO("%s is called, force update:%d\n", __func__, force);

    if (fw_len < APP_FILE_MIN_SIZE || fw_len > APP_FILE_MAX_SIZE) {
        TPD_INFO("%s: firmware size have some problem, no need update\n", __func__);
        return FW_NO_NEED_UPDATE;
    }

    fw_data = kzalloc(fw_len, GFP_KERNEL);
    if (!fw_data) {
        TPD_INFO("%s: kzalloc memory failed\n", __func__);
        return FW_UPDATE_ERROR;
    }
    memcpy(fw_data, fw->data, fw_len);

    PANEL_ID_IN_BIN = fw_data[0x0F9E];  //read out panel id in fw.bin
    PANEL_ID_IN_FLASH = touch_i2c_read_byte(chip_info->client, 0xE3);   //read out panel id in flash
    TPD_INFO("PANEL_ID_IN_BIN = %02x, PANEL_ID_IN_FLASH= %02x [0x8a-tianma/0xb0-truly/0xc3-boeb3/0xc8-boeb8]\n", PANEL_ID_IN_BIN, PANEL_ID_IN_FLASH);
    if (PANEL_ID_IN_BIN != PANEL_ID_IN_FLASH) {
        force = 1;  //set force update flag
        TPD_INFO("panel id not match, will force update\n");
    }

    TP_CURRENT_FIRMWARE_ID = fw_data[APP_FILE_VER_MAPPING];
    TP_FIRMWARE_ID = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.FTS_REG_FW_VER);
    TPD_INFO("%s, TP_FIRMWARE_ID(in .bin) = %02x, TP_FIRMWARE_ID(in flash) = %02x\n", __func__, TP_CURRENT_FIRMWARE_ID, TP_FIRMWARE_ID);
    if (!force) {
        if (TP_CURRENT_FIRMWARE_ID == TP_FIRMWARE_ID) {
            update_state = FW_NO_NEED_UPDATE;
            TPD_INFO("%s: tp fw no need update!\n", __func__);
        } else {
            ret = focal_upgrade_use_buf(chip_info, fw_data + APP_OFFSET, fw_len - APP_OFFSET);
            if (ret != 0) {
                update_state = FW_UPDATE_ERROR;
                TPD_INFO("%s: tp fw update failed!\n", __func__);
            } else {
                update_state =  FW_UPDATE_SUCCESS;
                TPD_INFO("%s: tp fw update success!\n", __func__);
            }
        }
    } else {
        ret = focal_upgrade_use_buf(chip_info, fw_data + APP_OFFSET, fw_len - APP_OFFSET);
        if (ret != 0) {
            update_state = FW_UPDATE_ERROR;
            TPD_INFO("%s: tp fw update failed!\n", __func__);
        } else {
            update_state =  FW_UPDATE_SUCCESS;
            TPD_INFO("%s: tp fw update success!\n", __func__);
        }
    }

    for (i = 0; i < 3; i++) {
        ret = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.FTS_REG_CHIP_ID);
        if (ret == 0x80) {  //0x80 is ok
            break;
        }
        msleep(10);
    }
    if (i == 3) {
        TPD_INFO("%s: Read chip Id failed(ret:%d), no need Update the Firmware\n", __func__, ret);
        update_state = FW_UPDATE_ERROR;
        goto OUT;
    }

    initcode_len = (uint32_t)((uint32_t)(fw_data[2] << 8) + fw_data[3]);
    if (0xFF != fw_data[initcode_len] + fw_data[initcode_len + 1]) {
        TPD_INFO("%s: init code check failed, no need update\n", __func__);
    } else {
        TPD_INFO("%s: init code check success\n", __func__);

        LCD_CURRENT_FIRMWARE_ID = fw_data[initcode_len];
        LCD_FIRMWARE_ID = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.FTS_REG_INITCODE_VER);
        TPD_INFO("%s: LCD_FIRMWARE_ID(in .bin) = %02x, LCD_FIRMWARE_ID(in flash) = %02x\n", __func__, LCD_CURRENT_FIRMWARE_ID, LCD_FIRMWARE_ID);
        if (!force) {
            if (LCD_FIRMWARE_ID == 0xA5) {  //0xA5: the lcd initcode is stable
                TPD_INFO("%s : check lcd version is A5, must no update initcode\n", __func__);
                goto OUT;
            }
            if ((LCD_CURRENT_FIRMWARE_ID != LCD_FIRMWARE_ID) || (LCD_FIRMWARE_ID == 0xFF)) {    //0xFF: lcd initcode must update
                ret = focal_lcd_cfg_upgrade_use_buf(chip_info, fw_data, 4096);
                if (ret != 0) {
                    update_state = FW_UPDATE_ERROR;
                    TPD_INFO("%s: init code update failed!\n", __func__);
                } else {
                    TPD_INFO("%s: init code update success!\n", __func__);
                }
            } else {
                TPD_INFO("%s: init code no need update!\n", __func__);
            }
        } else {
            ret = focal_lcd_cfg_upgrade_use_buf(chip_info, fw_data, 4096);
            if (ret != 0) {
                update_state = FW_UPDATE_ERROR;
                TPD_INFO("%s: init code update failed!\n", __func__);
            } else {
                TPD_INFO("%s: init code update success!\n", __func__);
            }
        }
    }

OUT:
    kfree(fw_data);
    return update_state;
}

static u8 focal_trigger_reason(void *chip_data, int gesture_enable, int is_suspended)
{
    int state = 0;
    struct chip_data_ft8006 *chip_info = (struct chip_data_ft8006 *)chip_data;

    state = touch_i2c_read_word(chip_info->client, 0x01);
    if (state == 0xFFFF) {  //all 0xff when fw reset
        return IRQ_FW_AUTO_RESET;
    }

    if ((gesture_enable == 1) && is_suspended) {
        state = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.FTS_REG_GESTURE_EN);
        if (state == 1) {
            return IRQ_GESTURE;
        } else {
            touch_i2c_write_byte(chip_info->client, chip_info->reg_info.FTS_REG_GESTURE_EN, 0x01);
            return IRQ_IGNORE;
        }
    } else if (is_suspended) {
        return IRQ_IGNORE;
    }

    return IRQ_TOUCH;
}

static int focal_get_touch_points(void *chip_data, struct point_info *points, int max_num)
{
    int obj_attention = 0;
    uint8_t pointid = 0;
    uint8_t point_state = 0;
    uint8_t point_num = 0;
    int ret = 0;
    int i = 0;
    struct chip_data_ft8006 *chip_info = (struct chip_data_ft8006 *)chip_data;
    char buf[6 * max_num + 3];

    memset(buf, 0, sizeof(buf));
    focal_esd_check_enable(chip_info, false);
    ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.FTS_REG_TOUCH_DATA, 3, buf);
    if (ret < 0) {
        TPD_INFO("read touch data header failed\n");
        focal_esd_check_enable(chip_info, true);
        return -1;
    }

    point_num = buf[2] & 0x0F;
    if (point_num > max_num) {
        TPD_INFO("read invalid point number, no need handle\n");
        focal_esd_check_enable(chip_info, true);
        return -1;
    } else if (point_num) {
        ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.FTS_REG_TOUCH_DATA + 3, 6 * point_num, buf + 3);
        if (ret < 0) {
            TPD_INFO("read touch data content failed\n");
            focal_esd_check_enable(chip_info, true);
            return -1;
        }
    }
    focal_esd_check_enable(chip_info, true);

    if ((0x0F != point_num) && (0x00 != point_num)) {
        for (i = 0; i < point_num; i++) {
            pointid = buf[i * 6 + 5] >> 4;
            if (pointid >= max_num) {
                break;
            }
            points[pointid].x = ((buf[i * 6 + 3] & 0x0f) << 8) | (buf[i * 6 + 4] & 0xff);
            points[pointid].y = ((buf[i * 6 + 5] & 0x0f) << 8) | (buf[i * 6 + 6] & 0xff);
            points[pointid].z = buf[i * 6 + 7];
            points[pointid].width_major = buf[i * 6 + 8] >> 4;
            points[pointid].status = 1;
            point_state = buf[i * 6 + 3] >> 6;  /*0&2 is down, 1 is up*/
            if ((0 == point_state) || (2 == point_state)) {
                //SET_BIT(obj_attention, pointid);
                obj_attention = obj_attention | (1 << pointid);
            }
        }
    }

    return obj_attention;
}

static int focal_get_gesture_info(void *chip_data, struct gesture_info * gesture)
{
    int i = 0, ret = 0;
    uint8_t buf[255 * 4];
    struct Coordinate limitPoint[4];
    uint8_t point_num = 0;
    int gesture_id = 0;
    int stepX = 0;
    struct chip_data_ft8006 *chip_info = (struct chip_data_ft8006 *)chip_data;

    memset(buf, 0, sizeof(buf));
    focal_esd_check_enable(chip_info, false);
    ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.FTS_REG_GESTURE_DATA, 8, &(buf[0]));
    if (ret < 0) {
        TPD_INFO("Read gesture header data failed!\n");
        focal_esd_check_enable(chip_info, true);
        return ret;
    }

    gesture_id = buf[0];
    point_num = buf[1];
    ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.FTS_REG_GESTURE_DATA, 4 * point_num + 2, &(buf[0]));
    focal_esd_check_enable(chip_info, true);
    if (ret < 0) {
        TPD_INFO("Read gesture point data failed!\n");
        return ret;
    }

    if (LEVEL_BASIC != tp_debug) {
        TPD_INFO("gesture pointNum(%d), points:", point_num);
        for (i = 0; i < point_num; i++) {
            printk("(%d, %d) ",(buf[4*i+2] & 0x0F) << 8 | (buf[4*i+3] & 0xFF), (buf[4*i+4] & 0x0F) << 8 | (buf[4*i+5] & 0xFF));
        }
    }

    //judge the gesture mode
    gesture->clockwise = 2;
    switch (gesture_id)
    {
        case GESTURE_LEFT:
            gesture->gesture_type = Right2LeftSwip;
            gesture->Point_start.x = (buf[2] & 0x0F) << 8 | (buf[3] & 0xFF);
            gesture->Point_start.y = (buf[4] & 0x0F) << 8 | (buf[5] & 0xFF);
            gesture->Point_end.x = (buf[4*(point_num-1)+2] & 0x0F) << 8 | (buf[4*(point_num-1)+3] & 0xFF);
            gesture->Point_end.y = (buf[4*(point_num-1)+4] & 0x0F) << 8 | (buf[4*(point_num-1)+5] & 0xFF);
            break;
        case GESTURE_RIGHT:
            gesture->gesture_type = Left2RightSwip;
            gesture->Point_start.x = (buf[2] & 0x0F) << 8 | (buf[3] & 0xFF);
            gesture->Point_start.y = (buf[4] & 0x0F) << 8 | (buf[5] & 0xFF);
            gesture->Point_end.x = (buf[4*(point_num-1)+2] & 0x0F) << 8 | (buf[4*(point_num-1)+3] & 0xFF);
            gesture->Point_end.y = (buf[4*(point_num-1)+4] & 0x0F) << 8 | (buf[4*(point_num-1)+5] & 0xFF);
            break;
        case GESTURE_UP:
            gesture->gesture_type = Down2UpSwip;
            gesture->Point_start.x = (buf[2] & 0x0F) << 8 | (buf[3] & 0xFF);
            gesture->Point_start.y = (buf[4] & 0x0F) << 8 | (buf[5] & 0xFF);
            gesture->Point_end.x = (buf[4*(point_num-1)+2] & 0x0F) << 8 | (buf[4*(point_num-1)+3] & 0xFF);
            gesture->Point_end.y = (buf[4*(point_num-1)+4] & 0x0F) << 8 | (buf[4*(point_num-1)+5] & 0xFF);
            break;
        case GESTURE_DOWN:
            gesture->gesture_type = Up2DownSwip;
            gesture->Point_start.x = (buf[2] & 0x0F) << 8 | (buf[3] & 0xFF);
            gesture->Point_start.y = (buf[4] & 0x0F) << 8 | (buf[5] & 0xFF);
            gesture->Point_end.x = (buf[4*(point_num-1)+2] & 0x0F) << 8 | (buf[4*(point_num-1)+3] & 0xFF);
            gesture->Point_end.y = (buf[4*(point_num-1)+4] & 0x0F) << 8 | (buf[4*(point_num-1)+5] & 0xFF);
            break;
        case GESTURE_DOUBLECLICK:
            gesture->gesture_type = DouTap;
            gesture->Point_start.x = (buf[2] & 0x0F) << 8 | (buf[3] & 0xFF);
            gesture->Point_start.y = (buf[4] & 0x0F) << 8 | (buf[5] & 0xFF);
            gesture->Point_end.x = (buf[4*(point_num-1)+2] & 0x0F) << 8 | (buf[4*(point_num-1)+3] & 0xFF);
            gesture->Point_end.y = (buf[4*(point_num-1)+4] & 0x0F) << 8 | (buf[4*(point_num-1)+5] & 0xFF);
            break;
        case GESTURE_O:
            gesture->gesture_type = Circle;
            gesture->Point_start.x = (buf[2] & 0x0F) << 8 | (buf[3] & 0xFF);
            gesture->Point_start.y = (buf[4] & 0x0F) << 8 | (buf[5] & 0xFF);
            gesture->Point_end.x = (buf[4*(point_num-1)+2] & 0x0F) << 8 | (buf[4*(point_num-1)+3] & 0xFF);
            gesture->Point_end.y = (buf[4*(point_num-1)+4] & 0x0F) << 8 | (buf[4*(point_num-1)+5] & 0xFF);
            gesture->clockwise = 0;
            getSpecialPoint(&buf[2], point_num, &limitPoint[0]);
            gesture->Point_1st = limitPoint[0]; //ymin
            gesture->Point_2nd = limitPoint[1]; //xmin
            gesture->Point_3rd = limitPoint[2]; //ymax
            gesture->Point_4th = limitPoint[3]; //xmax
            break;
        case GESTURE_CLOCKWISE_O:
            gesture->gesture_type = Circle;
            gesture->Point_start.x = (buf[2] & 0x0F) << 8 | (buf[3] & 0xFF);
            gesture->Point_start.y = (buf[4] & 0x0F) << 8 | (buf[5] & 0xFF);
            gesture->Point_end.x = (buf[4*(point_num-1)+2] & 0x0F) << 8 | (buf[4*(point_num-1)+3] & 0xFF);
            gesture->Point_end.y = (buf[4*(point_num-1)+4] & 0x0F) << 8 | (buf[4*(point_num-1)+5] & 0xFF);
            gesture->clockwise = 1;
            getSpecialPoint(&buf[2], point_num, &limitPoint[0]);
            gesture->Point_1st = limitPoint[0]; //ymin
            gesture->Point_2nd = limitPoint[1]; //xmin
            gesture->Point_3rd = limitPoint[2]; //ymax
            gesture->Point_4th = limitPoint[3]; //xmax
            break;
        case GESTURE_W:
            gesture->gesture_type = Wgestrue;
            getSpecialPoint(&buf[2], point_num, &limitPoint[0]);
            stepX = abs(limitPoint[3].x - limitPoint[1].x)/4;
            gesture->Point_start.x = limitPoint[1].x;
            gesture->Point_start.y = limitPoint[0].y;
            gesture->Point_end.x = limitPoint[3].x;
            gesture->Point_end.y = limitPoint[0].y;
            gesture->Point_1st.x = limitPoint[1].x + stepX;
            gesture->Point_1st.y = limitPoint[2].y;
            gesture->Point_2nd.x = limitPoint[1].x + 2 * stepX;
            gesture->Point_2nd.y = limitPoint[0].y;
            gesture->Point_3rd.x = limitPoint[1].x + 3 * stepX;
            gesture->Point_3rd.y = limitPoint[2].y;
            break;
        case GESTURE_M:
            gesture->gesture_type = Mgestrue;
            getSpecialPoint(&buf[2], point_num, &limitPoint[0]);
            gesture->Point_start.x = limitPoint[1].x;
            gesture->Point_start.y = limitPoint[2].y;
            gesture->Point_end.x = limitPoint[3].x;
            gesture->Point_end.y = limitPoint[2].y;
            gesture->Point_1st.x = limitPoint[1].x + stepX;
            gesture->Point_1st.y = limitPoint[0].y;
            gesture->Point_2nd.x = limitPoint[1].x + 2 * stepX;
            gesture->Point_2nd.y = limitPoint[2].y;
            gesture->Point_3rd.x = limitPoint[1].x + 3 * stepX;
            gesture->Point_3rd.y = limitPoint[0].y;
            break;
        case GESTURE_LEFT_V:
            gesture->gesture_type = LeftVee;
            getSpecialPoint(&buf[2], point_num, &limitPoint[0]);
            gesture->Point_start.x = (buf[2] & 0x0F) << 8 | (buf[3] & 0xFF);
            gesture->Point_start.y = (buf[4] & 0x0F) << 8 | (buf[5] & 0xFF);
            gesture->Point_end.x = (buf[4*(point_num-1)+2] & 0x0F) << 8 | (buf[4*(point_num-1)+3] & 0xFF);
            gesture->Point_end.y = (buf[4*(point_num-1)+4] & 0x0F) << 8 | (buf[4*(point_num-1)+5] & 0xFF);
            gesture->Point_1st = limitPoint[3]; //xmax point
            break;
        case GESTURE_RIGHT_V:
            gesture->gesture_type = RightVee;
            getSpecialPoint(&buf[2], point_num, &limitPoint[0]);
            gesture->Point_start.x = (buf[2] & 0x0F) << 8 | (buf[3] & 0xFF);
            gesture->Point_start.y = (buf[4] & 0x0F) << 8 | (buf[5] & 0xFF);
            gesture->Point_end.x = (buf[4*(point_num-1)+2] & 0x0F) << 8 | (buf[4*(point_num-1)+3] & 0xFF);
            gesture->Point_end.y = (buf[4*(point_num-1)+4] & 0x0F) << 8 | (buf[4*(point_num-1)+5] & 0xFF);
            gesture->Point_1st = limitPoint[1]; //xmin point
            break;
        case GESTURE_UP_V:
            gesture->gesture_type = UpVee;
            getSpecialPoint(&buf[2], point_num, &limitPoint[0]);
            gesture->Point_start.x = (buf[2] & 0x0F) << 8 | (buf[3] & 0xFF);
            gesture->Point_start.y = (buf[4] & 0x0F) << 8 | (buf[5] & 0xFF);
            gesture->Point_end.x = (buf[4*(point_num-1)+2] & 0x0F) << 8 | (buf[4*(point_num-1)+3] & 0xFF);
            gesture->Point_end.y = (buf[4*(point_num-1)+4] & 0x0F) << 8 | (buf[4*(point_num-1)+5] & 0xFF);
            gesture->Point_1st = limitPoint[2]; //ymax point
            break;
        case GESTURE_DOWN_V:
            gesture->gesture_type = DownVee;
            getSpecialPoint(&buf[2], point_num, &limitPoint[0]);
            gesture->Point_start.x = (buf[2] & 0x0F) << 8 | (buf[3] & 0xFF);
            gesture->Point_start.y = (buf[4] & 0x0F) << 8 | (buf[5] & 0xFF);
            gesture->Point_end.x = (buf[4*(point_num-1)+2] & 0x0F) << 8 | (buf[4*(point_num-1)+3] & 0xFF);
            gesture->Point_end.y = (buf[4*(point_num-1)+4] & 0x0F) << 8 | (buf[4*(point_num-1)+5] & 0xFF);
            gesture->Point_1st = limitPoint[0]; //ymin point
            break;
        case GESTURE_DOUBLE_LINE:
            gesture->gesture_type = DouSwip;
            gesture->Point_start.x = (buf[2] & 0x0F) << 8 | (buf[3] & 0xFF);
            gesture->Point_start.y = (buf[4] & 0x0F) << 8 | (buf[5] & 0xFF);
            gesture->Point_end.x = (buf[6] & 0x0F) << 8 | (buf[7] & 0xFF);
            gesture->Point_end.y = (buf[8] & 0x0F) << 8 | (buf[9] & 0xFF);
            gesture->Point_1st.x = (buf[10] & 0x0F) << 8 | (buf[11] & 0xFF);
            gesture->Point_1st.y = (buf[12] & 0x0F) << 8 | (buf[13] & 0xFF);
            gesture->Point_2nd.x = (buf[14] & 0x0F) << 8 | (buf[15] & 0xFF);
            gesture->Point_2nd.y = (buf[16] & 0x0F) << 8 | (buf[17] & 0xFF);
            break;
        default:
            gesture->gesture_type = UnkownGesture;
    }

    TPD_INFO("%s, gesture_id: 0x%x, gesture_type: %d, clockwise: %d, points: (%d, %d)(%d, %d)(%d, %d)(%d, %d)(%d, %d)(%d, %d)\n", \
                __func__, gesture_id, gesture->gesture_type, gesture->clockwise, \
                gesture->Point_start.x, gesture->Point_start.y, \
                gesture->Point_end.x, gesture->Point_end.y, \
                gesture->Point_1st.x, gesture->Point_1st.y, \
                gesture->Point_2nd.x, gesture->Point_2nd.y, \
                gesture->Point_3rd.x, gesture->Point_3rd.y, \
                gesture->Point_4th.x, gesture->Point_4th.y);

    return 0;
}

static int focal_mode_switch(void *chip_data, work_mode mode, bool flag)
{
    int ret = -1;
    struct chip_data_ft8006 *chip_info = (struct chip_data_ft8006 *)chip_data;

    switch(mode) {
        case MODE_NORMAL:
            ret = 0;
            break;

        case MODE_SLEEP:
            ret = focal_enter_sleep(chip_info, true);
            if (ret < 0) {
                TPD_INFO("%s: focal enter sleep failed\n", __func__);
            }
            break;

        case MODE_GESTURE:
            ret = focal_enable_black_gesture(chip_info, flag);
            if (ret < 0) {
                TPD_INFO("%s: focal enable gesture failed.\n", __func__);
                return ret;
            }
            break;

        case MODE_EDGE:
            ret = focal_enable_edge_limit(chip_info, flag);
            if (ret < 0) {
                TPD_INFO("%s: focal enable edg limit failed.\n", __func__);
                return ret;
            }
            break;

        case MODE_CHARGE:
            ret = focal_enable_charge_mode(chip_info, flag);
            if (ret < 0) {
                TPD_INFO("%s: enable charge mode : %d failed\n", __func__, flag);
            }
            break;

        default:
            TPD_INFO("%s: Wrong mode.\n", __func__);
    }

    return ret;
}

static int focal_esd_handle(void *chip_data)
{
    s32 ret = -1;
    int i = 0;
    static int flow_work_cnt_last = 0;
    static int err_cnt = 0;
    static int i2c_err = 0;
    struct chip_data_ft8006 *chip_info = (struct chip_data_ft8006 *)chip_data;

    if (chip_info->esd_check_need_stop) {
        goto NORMAL_END;
    }

    ret = touch_i2c_read_byte(chip_info->client, 0x00);
    if ((ret & 0x70) == 0x40) { //work in factory mode
        goto NORMAL_END;
    }

    lcd_esd_recovery_mutex(true);

    for (i = 0; i < 3; i++) {
        ret = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.FTS_REG_CHIP_ID);
        if (ret != 0x80) {
            TPD_INFO("%s: read chip_id failed!(ret:%d)\n", __func__, ret);
            msleep(10);
            i2c_err++;
        } else {
            i2c_err = 0;
            break;
        }
    }
    ret = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.FTS_REG_FLOW_WORK_CNT);
    if (ret < 0) {
        TPD_INFO("%s: read FTS_REG_FLOW_WORK_CNT failed!\n", __func__);
        i2c_err++;
    }

    if (flow_work_cnt_last == ret) {
        err_cnt++;
    } else {
        err_cnt = 0;
    }
    flow_work_cnt_last = ret;

    if ((err_cnt >= 5) || (i2c_err >= 3)) {
        TPD_INFO("esd check failed, start reset!\n");
        disable_irq_nosync(chip_info->client->irq);
        tp_touch_btnkey_release();
        focal_reset(chip_info);
        enable_irq(chip_info->client->irq);
        flow_work_cnt_last = 0;
        err_cnt = 0;
        i2c_err = 0;
    }

    lcd_esd_recovery_mutex(false);
NORMAL_END:
    return 0;
}

static void focal_black_screen_test(void *chip_data, char *message)
{
    int state = -1;
    uint8_t retry = 6;
    struct chip_data_ft8006 *chip_info = (struct chip_data_ft8006 *)chip_data;

    //wait until ic get into gesture mode
    while (--retry) {
        state = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.FTS_REG_GESTURE_EN);
        if (state == 1) {
            break;
        }
        msleep(10);
    }

    if (retry) {
        TPD_INFO("%s,:try %d times, get into gesture mode success!\n", __func__, 5-retry);
        sprintf(message, "0 errors. get into gesture mode success\n");
    } else {
        TPD_INFO("%s,:try %d times, get into gesture mode failed!\n", __func__, 5-retry);
        sprintf(message, "1 errors. get into gesture mode failed\n");
    }
}

static fp_touch_state focal_spurious_fp_check(void *chip_data)
{
    int x = 0, y = 0, z = 0, err_count = 0;
    int ret = 0, TX_NUM = 0, RX_NUM = 0;
    int16_t temp_data = 0, delta_data = 0;
    uint8_t *raw_data = NULL;
    fp_touch_state fp_touch_state = FINGER_PROTECT_TOUCH_UP;

    struct chip_data_ft8006 *chip_info = (struct chip_data_ft8006 *)chip_data;
    TPD_INFO(" synaptics_spurious_fp_check  start\n");

    if(TX_NUM*RX_NUM*(sizeof(int16_t)) > 1800){
        TPD_INFO("%s,TX_NUM*RX_NUM*(sizeof(int16_t)>1800,There is not enough space\n", __func__);
        return FINGER_PROTECT_NOTREADY ;
    }

    if (!chip_info->spuri_fp_data) {
        TPD_INFO("chip_info->spuri_fp_data kzalloc error\n");
        return fp_touch_state;
    }

    raw_data = kzalloc(TX_NUM * SPURIOUS_FP_RX_NUM * 2 * (sizeof(uint8_t)), GFP_KERNEL);
    if (!raw_data) {
            TPD_INFO("raw_data kzalloc error\n");
            return fp_touch_state;
    }
    TX_NUM = chip_info->hw_res->TX_NUM;
    RX_NUM = chip_info->hw_res->RX_NUM;

    ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.FTS_REG_BASELINE_DATA, TX_NUM*SPURIOUS_FP_RX_NUM*2, raw_data); //read baseline data
    if (ret < 0) {
        TPD_INFO("%s: read baseline failed\n", __func__);
        return fp_touch_state;
    }

    for (x = 1; x < SPURIOUS_FP_RX_NUM; x++) {
        TPD_DEBUG_NTAG("[%2d]: ", x);
        for (y = 0; y < TX_NUM; y++) {
            z = TX_NUM * x + y;
            temp_data = (raw_data[z * 2] << 8) | raw_data[z * 2 + 1];
            delta_data = temp_data - chip_info->spuri_fp_data[z];
            TPD_DEBUG_NTAG("%4d, ", delta_data);
            if ((delta_data + SPURIOUS_FP_LIMIT) < 0) {
                if (!tp_debug)
                    TPD_INFO("delta_data too large, delta_data = %d TX[%d] RX[%d]\n", delta_data, x, y);
                err_count++;
            }
        }
        TPD_DEBUG_NTAG("\n");
        if(err_count > 2) {
            fp_touch_state = FINGER_PROTECT_TOUCH_DOWN;
            err_count = 0;
            if (!tp_debug)
                break;
        }
    }

    TPD_INFO("finger protect trigger fp_touch_state= %d\n", fp_touch_state);

    return fp_touch_state;
}

static void focal_finger_proctect_data_get(void * chip_data)
{
    int ret = 0, x = 0, y = 0, z = 0;
    uint8_t *raw_data = NULL;
    static uint8_t retry_time = 3;
    struct chip_data_ft8006 *chip_info = (struct chip_data_ft8006 *)chip_data;

    int TX_NUM = chip_info->hw_res->TX_NUM;
    int RX_NUM = chip_info->hw_res->RX_NUM;

    if (TX_NUM*RX_NUM*(sizeof(int16_t)) > 1800) {
        TPD_INFO("%s,TX_NUM*RX_NUM*(sizeof(int16_t)>1800,There is not enough space\n", __func__);
        return;
    }

    raw_data = kzalloc(TX_NUM * SPURIOUS_FP_RX_NUM * 2 * (sizeof(uint8_t)), GFP_KERNEL);
    if (!raw_data) {
            TPD_INFO("raw_data kzalloc error\n");
            return;
    }

    chip_info->spuri_fp_data = kzalloc(TX_NUM*SPURIOUS_FP_RX_NUM*(sizeof(int16_t)), GFP_KERNEL);
    if (!chip_info->spuri_fp_data) {
        TPD_INFO("chip_info->spuri_fp_data kzalloc error\n");
        kfree(raw_data);
        ret = -ENOMEM;
        return;
    }

RE_TRY:
    ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.FTS_REG_BASELINE_DATA, TX_NUM*SPURIOUS_FP_RX_NUM*2, raw_data);     //read data
    if (ret < 0) {
        if (retry_time) {
            TPD_INFO("%s touch_i2c_read_block error\n",__func__);
            retry_time--;
            msleep(10);
            goto RE_TRY;
        }
    }

    for (x = 0; x < SPURIOUS_FP_RX_NUM; x++) {
        printk("[%2d]: ", x);
        for (y = 0; y < TX_NUM; y++) {
            z = TX_NUM*x + y;
            chip_info->spuri_fp_data[z] = (raw_data[z *2] << 8) | raw_data[z*2 + 1];
            printk("%5d,",chip_info->spuri_fp_data[z]);
        }
        printk("\n");
    }

    kfree(raw_data);

}

static void focal_register_info_read(void * chip_data, uint16_t register_addr, uint8_t * result, uint8_t length)
{
    int ret = 0;
    uint8_t addr = 0;
    struct chip_data_ft8006 *chip_info = (struct chip_data_ft8006 *)chip_data;

    addr = register_addr & 0xFF;
    ret = touch_i2c_read_block(chip_info->client, addr, length, result);
}

#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
extern unsigned int upmu_get_rgs_chrdet(void);
static int focal_get_usb_state(void)
{
    return upmu_get_rgs_chrdet();
}
#else
#endif

static struct oppo_touchpanel_operations focal_ops = {
    .ftm_process                = focal_ftm_process,
    .get_vendor                 = focal_get_vendor,
    .get_chip_info              = focal_get_chip_info,
    .reset                      = focal_reset,
    .power_control              = focal_power_control,
    .fw_check                   = focal_fw_check,
    .fw_update                  = focal_fw_update,
    .trigger_reason             = focal_trigger_reason,
    .get_touch_points           = focal_get_touch_points,
    .get_gesture_info           = focal_get_gesture_info,
    .mode_switch                = focal_mode_switch,
    .esd_handle                 = focal_esd_handle,
    .black_screen_test          = focal_black_screen_test,
    .spurious_fp_check          = focal_spurious_fp_check,
    .finger_proctect_data_get   = focal_finger_proctect_data_get,
    .register_info_read         = focal_register_info_read,
    .get_usb_state              = focal_get_usb_state,
};
/********* End of implementation of oppo_touchpanel_operations callbacks**********************/

/****************** Start of function work for focal debug callbacks**************************/
enum FW_STATUS focal_get_pram_or_rom_id(void *chip_data)
{
    u8 buf[4];
    u8 reg_val[2] = {0};
    enum FW_STATUS inRomBoot = FTS_RUN_IN_ERROR;
    struct chip_data_ft8006 *chip_info = (struct chip_data_ft8006 *)chip_data;

    /*Enter upgrade mode*/
    /*send 0x55 in time windows*/
    buf[0] = 0x55;
    buf[1] = 0xAA;
    touch_i2c_write(chip_info->client, buf, 2);

    msleep(20);

    buf[0] = 0x90;
    buf[1] = buf[2] = buf[3] =0x00;
    touch_i2c_read(chip_info->client, buf, 4, reg_val, 2);

    TPD_INFO("[UPGRADE] Read ROM/PRAM/Bootloader id:0x%02x%02x\n", reg_val[0], reg_val[1]);
    if ((reg_val[0] == 0x00) || (reg_val[0] == 0xFF)) {
        inRomBoot = FTS_RUN_IN_ERROR;
    } else if (reg_val[0] == 0x80 && reg_val[1] == 0xC6) {
        inRomBoot = FTS_RUN_IN_PRAM;
    } else if (reg_val[0] == 0x80 && reg_val[1] == 0x06) {
        inRomBoot = FTS_RUN_IN_ROM;
    } else if (reg_val[0] == 0x80 && reg_val[1] == 0xB6) {
        inRomBoot = FTS_RUN_IN_BOOTLOADER;
    }

    return inRomBoot;
}

bool focal_check_run_state(void *chip_data, int rstate)
{
    int i = 0;
    enum FW_STATUS cstate = FTS_RUN_IN_ERROR;
    struct chip_data_ft8006 *chip_info = (struct chip_data_ft8006 *)chip_data;

    for (i = 0; i < FTS_UPGRADE_LOOP; i++)
    {
        cstate = focal_get_pram_or_rom_id(chip_info);
        TPD_DEBUG( "[UPGRADE]: run state = %d", cstate);

        if (cstate == rstate)
            return true;
        msleep(20);
    }

    return false;
}

int focal_start_fw_upgrade(void *chip_data)
{
    int i_ret = 0;
    struct chip_data_ft8006 *chip_info = (struct chip_data_ft8006 *)chip_data;

    /*send the soft upgrade commond to FW, and start upgrade*/
    i_ret = touch_i2c_write_byte(chip_info->client, chip_info->reg_info.FTS_RST_CMD_REG, 0xAA);
    msleep(10);
    i_ret = touch_i2c_write_byte(chip_info->client, chip_info->reg_info.FTS_RST_CMD_REG, 0x55);
    msleep(200);

    return i_ret;
}

int focal_erase_flash(void *chip_data)
{
    u32 i = 0;
    u8 auc_i2c_write_buf[10];
    u8 reg_val[4] = {0};
    struct chip_data_ft8006 *chip_info = (struct chip_data_ft8006 *)chip_data;

    TPD_INFO("[UPGRADE]**********erase app now**********\n");

    /*send to erase flash*/
    auc_i2c_write_buf[0] = 0x61;
    touch_i2c_write(chip_info->client, auc_i2c_write_buf, 1);
    msleep(1350);

    for (i = 0; i < 15; i++) {
        /*get the erase app status, if get 0xF0AA£¬erase flash success*/
        auc_i2c_write_buf[0] = 0x6a;
        reg_val[0] = reg_val[1] = 0x00;
        touch_i2c_read(chip_info->client, auc_i2c_write_buf, 1, reg_val, 2);

        if (0xF0==reg_val[0] && 0xAA==reg_val[1]) { /*erase flash success*/
            break;
        }
        msleep(50);
    }

    /*erase flash fail*/
    if ((0xF0!=reg_val[0] || 0xAA!=reg_val[1]) && (i >= 15)) {
        TPD_INFO("[UPGRADE]: erase app error.need reset tp and reload FW!!\n");
        return -EIO;
    }
    TPD_INFO("[UPGRADE]: erase app ok!!\n");

    return 0;
}

void focal_upgrade_delay(u32 i)
{
    do {
        i--;
    } while (i > 0);
}

static int print_data(u8 *buf, u32 len)
{
    int i = 0;
    int n = 0;
    u8 *p = NULL;

    p = kmalloc(len*4, GFP_KERNEL);
    memset(p, 0, len*4);

    for (i = 0; i < len; i++) {
        n += sprintf(p + n, "%02x ", buf[i]);
    }

    TPD_INFO("%s \n", p);

    kfree(p);
    return 0;
}


static int focal_read_3gamma(struct i2c_client *client, u8 **gamma, u16 *len)
{
    int ret = 0;
    int i = 0;
    int packet_num = 0;
    int packet_len = 0;
    int remainder = 0;
    u8 cmd[4] = { 0 };
    u32 addr = 0x01D000;
    u8 gamma_header[0x20] = { 0 };
    u16 gamma_len = 0;
    u16 gamma_len_n = 0;
    u16 pos = 0;
    bool gamma_has_enable = false;
    u8 *pgamma = NULL;
    int j = 0;
    u8 gamma_ecc = 0;
    
    cmd[0] = 0x03;
    cmd[1] = (u8)(addr >> 16);
    cmd[2] = (u8)(addr >> 8);
    cmd[3] = (u8)addr;
    ret = touch_i2c_write(client, cmd, 4);
    msleep(10);
    ret = touch_i2c_read(client, NULL, 0, gamma_header, 0x20);
    if(ret < 0) {
         TPD_INFO("read 3-gamma header fail\n");
           return ret;
    }

    gamma_len = (u16)((u16)gamma_header[0] << 8) + gamma_header[1];
    gamma_len_n = (u16)((u16)gamma_header[2] << 8) + gamma_header[3];

    if((gamma_len + gamma_len_n) != 0xFFFF) {
        TPD_INFO("gamma length check fail:%x %x\n", gamma_len, gamma_len);
        return -EIO;
    }

    if((gamma_header[4] + gamma_header[5]) != 0xFF) {
        TPD_INFO("gamma ecc check fail:%x %x\n", gamma_header[4], gamma_header[5]);
        return -EIO;
    }

    if(gamma_len > MAX_GAMMA_LEN) {
        TPD_INFO("gamma data len(%d) is too long\n", gamma_len);
        return -EINVAL;
    }

    pgamma = *gamma;
    packet_num = gamma_len/FTS_PACKET_LENGTH;
    packet_len = FTS_PACKET_LENGTH;
    remainder = gamma_len%FTS_PACKET_LENGTH;
    if(remainder) {
        packet_num++;
    }
    TPD_INFO("3-gamma len:%d\n", gamma_len);
    cmd[0] = 0x03;
    addr += 0x20;
    for(i = 0; i < packet_num; i++) {
        addr += i * FTS_PACKET_LENGTH;
        cmd[1] = (u8)(addr >> 16);
        cmd[2] = (u8)(addr >> 8);
        cmd[3] = (u8)addr;
        if ((i == packet_num -1) && remainder) {
            packet_len = remainder;
        }
        ret = touch_i2c_write(client, cmd, 4);
        msleep(10);
        ret = touch_i2c_read(client, NULL, 0, pgamma + i*FTS_PACKET_LENGTH, packet_len);
        if(ret < 0) {
            TPD_INFO("read 3-gamma data fail\n");
            return ret;
        }
    }

    // ecc
    for(j = 0; j < gamma_len; j++) {
        gamma_ecc ^= pgamma[j];
    }
    TPD_INFO("back_3gamma_ecc: 0x%x, 0x%x\n",gamma_ecc,gamma_header[0x04]);
    if(gamma_ecc != gamma_header[0x04]) {
        TPD_INFO("back gamma ecc check fail:%x %x\n", gamma_ecc, gamma_header[0x04]);
        return -EIO;
    }

    /* check last byte is 91 80 00 19 01 */
    pos = gamma_len - 5;
    if((gamma_enable[1] == pgamma[pos]) && (gamma_enable[2] == pgamma[pos+1])
        && (gamma_enable[3] == pgamma[pos+2]) && (gamma_enable[4] == pgamma[pos+3])) {
        gamma_has_enable = true;
    }

    if(false == gamma_has_enable) {
        TPD_INFO("3-gamma has no gamma enable info\n");
        pgamma[gamma_len++] = gamma_enable[1];
        pgamma[gamma_len++] = gamma_enable[2];
        pgamma[gamma_len++] = gamma_enable[3];
        pgamma[gamma_len++] = gamma_enable[4];
        pgamma[gamma_len++] = gamma_enable[5];
    }

    *len = gamma_len;
    
    if (tp_debug == 2) {
        TPD_DEBUG("read 3-gamma data:");
        print_data(*gamma, gamma_len);
    }

    return 0;
}

static int focal_replace_3gamma(u8 *initcode, u8 *gamma, u16 gamma_len)
{
    u16 gamma_pos = 0;

    /* Analog Gamma */
    if((initcode[gamma_analog[0]] == gamma[gamma_pos]) && (initcode[gamma_analog[0] + 1] == gamma[gamma_pos + 1])) {
        memcpy(initcode + gamma_analog[0] + 4 , gamma + gamma_pos + 4, gamma_analog[5]);
        gamma_pos += gamma_analog[5] + 4;
    } else {
        goto find_gamma_bank_err;
    }

    /* Digital1 Gamma */
    if((initcode[gamma_digital1[0]] == gamma[gamma_pos]) && (initcode[gamma_digital1[0] + 1] == gamma[gamma_pos + 1])) {
        memcpy(initcode + gamma_digital1[0] + 4 , gamma + gamma_pos + 4, gamma_digital1[5]);
        gamma_pos += gamma_digital1[5] + 4;
    } else {
        goto find_gamma_bank_err;
    }

    /* Digital2 Gamma */
    if((initcode[gamma_digital2[0]] == gamma[gamma_pos]) && (initcode[gamma_digital2[0] + 1] == gamma[gamma_pos + 1])) {
        memcpy(initcode + gamma_digital2[0] + 4 , gamma + gamma_pos + 4, gamma_digital2[5]);
        gamma_pos += gamma_digital2[5] + 4;
    } else {
        goto find_gamma_bank_err;
    }

    /* enable Gamma */
    if((initcode[gamma_enable[0]] == gamma[gamma_pos]) && (initcode[gamma_enable[0] + 1] == gamma[gamma_pos + 1])) {
        if(gamma[gamma_pos + 4]) {
            initcode[gamma_enable[0] + 4 + 15] |= 0x01;
        } else {
            initcode[gamma_enable[0] + 4 + 15] &= 0xFE;
        }
        gamma_pos += 1 + 4;
    } else {
        goto find_gamma_bank_err;
    }

    if (tp_debug == 2) {
        TPD_DEBUG("replace 3-gamma data:");
        print_data(initcode, 1100);
    }

    return 0;

find_gamma_bank_err:
    TPD_INFO("3-gamma bank(%02x %02x) not find", gamma[gamma_pos], gamma[gamma_pos+1]);
    return -ENODATA;
}


/* calculate lcd init code ecc */
static int focal_cal_lcdinitcode_ecc(u8 *buf, u16 *ecc_val)
{
    u32 bank_crc_en = 0;
    u8 bank_data[MAX_BANK_DATA] = { 0 };
    u16 bank_len = 0;
    u16 bank_addr = 0;
    u32 bank_num = 0;
    u16 file_len = 0;
    u16 pos = 0;
    int i = 0;
    union short_bits ecc;
    union short_bits ecc_last;
    union short_bits temp_byte;
    u8 bank_mapping[] = { 0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9,
        0xA, 0xB, 0xC, 0xD, 0xE, 0xF, 0x10, 0x11, 0x12, 0x13, 0x14, 0x18, 
        0x19, 0x1A, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x22, 0x23, 0x24}; //Actaul mipi bank
    u8 banknum_8006 = 0;

    ecc.dshort = 0;
    ecc_last.dshort = 0;
    temp_byte.dshort = 0;

    file_len = (u16)(((u16)buf[2] << 8) + buf[3]);
    bank_crc_en = (u32)(((u32)buf[9] << 24) + ((u32)buf[8] << 16) + ((u32)buf[7] << 8) + (u32)buf[6]);
    TPD_INFO("lcd init code len=%x bank en=%x\n", file_len, bank_crc_en);

    pos = 0x0A; // addr of first bank
    while(pos < file_len) {
        bank_addr = (u16)(((u16)buf[pos + 0] << 8 ) + buf[pos + 1]);
        bank_len = (u16)(((u16)buf[pos + 2] << 8 ) + buf[pos + 3]);
        TPD_INFO("bank pos=%x bank_addr=%x bank_len=%x\n", pos, bank_addr, bank_len);
        if(bank_len > MAX_BANK_DATA) {
            return -EINVAL;
        }
        memset(bank_data, 0, MAX_BANK_DATA);
        memcpy(bank_data, buf + pos + 4, bank_len);

        bank_num = (bank_addr - 0x8000)/MAX_BANK_DATA;
        TPD_INFO("actual mipi bank number = %x\n", bank_num);
        for(i = 0; i < sizeof(bank_mapping)/sizeof(u8); i++) {
            if(bank_num == bank_mapping[i]) {
                banknum_8006 = i;
                break;
            }
        }
        if(i >= sizeof(bank_mapping)/sizeof(u8)) {
            TPD_INFO("actual mipi bank(%d) not find in bank mapping, need jump\n", bank_num);
        } else {
            TPD_INFO("8006 bank number = %d\n", banknum_8006);
            if((bank_crc_en >> banknum_8006) & 0x01) {
                for(i = 0; i < MAX_BANK_DATA; i++) {
                    temp_byte.dshort = (u16)bank_data[i];
                    if(i == 0) {
                        TPD_INFO("data0=%x, %d %d %d %d %d %d %d %d", temp_byte.dshort, temp_byte.bits.bit0, 
                            temp_byte.bits.bit1, temp_byte.bits.bit2, temp_byte.bits.bit3, temp_byte.bits.bit4, 
                            temp_byte.bits.bit5, temp_byte.bits.bit6, temp_byte.bits.bit7);
                    }

                    ecc.bits.bit0 = ecc_last.bits.bit8 ^ ecc_last.bits.bit9 ^ ecc_last.bits.bit10 ^ ecc_last.bits.bit11
                        ^ ecc_last.bits.bit12 ^ ecc_last.bits.bit13 ^ ecc_last.bits.bit14 ^ ecc_last.bits.bit15
                        ^ temp_byte.bits.bit0 ^ temp_byte.bits.bit1 ^ temp_byte.bits.bit2 ^ temp_byte.bits.bit3
                        ^ temp_byte.bits.bit4 ^ temp_byte.bits.bit5 ^ temp_byte.bits.bit6 ^ temp_byte.bits.bit7;
                    ecc.bits.bit1 = ecc_last.bits.bit9 ^ ecc_last.bits.bit10 ^ ecc_last.bits.bit11 ^ ecc_last.bits.bit12
                        ^ ecc_last.bits.bit13 ^ ecc_last.bits.bit14 ^ ecc_last.bits.bit15
                        ^ temp_byte.bits.bit1 ^ temp_byte.bits.bit2 ^ temp_byte.bits.bit3 ^ temp_byte.bits.bit4
                        ^ temp_byte.bits.bit5 ^ temp_byte.bits.bit6 ^ temp_byte.bits.bit7;
                    ecc.bits.bit2 = ecc_last.bits.bit8 ^ ecc_last.bits.bit9 ^ temp_byte.bits.bit0 ^ temp_byte.bits.bit1;
                    ecc.bits.bit3 = ecc_last.bits.bit9 ^ ecc_last.bits.bit10 ^ temp_byte.bits.bit1 ^ temp_byte.bits.bit2;
                    ecc.bits.bit4 = ecc_last.bits.bit10 ^ ecc_last.bits.bit11 ^ temp_byte.bits.bit2 ^ temp_byte.bits.bit3;
                    ecc.bits.bit5 = ecc_last.bits.bit11 ^ ecc_last.bits.bit12 ^ temp_byte.bits.bit3 ^ temp_byte.bits.bit4;
                    ecc.bits.bit6 = ecc_last.bits.bit12 ^ ecc_last.bits.bit13 ^ temp_byte.bits.bit4 ^ temp_byte.bits.bit5;
                    ecc.bits.bit7 = ecc_last.bits.bit13 ^ ecc_last.bits.bit14 ^ temp_byte.bits.bit5 ^ temp_byte.bits.bit6;
                    ecc.bits.bit8 = ecc_last.bits.bit0 ^ ecc_last.bits.bit14^ ecc_last.bits.bit15 ^ temp_byte.bits.bit6 ^ temp_byte.bits.bit7;
                    ecc.bits.bit9 = ecc_last.bits.bit1 ^ ecc_last.bits.bit15 ^ temp_byte.bits.bit7;
                    ecc.bits.bit10 = ecc_last.bits.bit2;
                    ecc.bits.bit11 = ecc_last.bits.bit3;
                    ecc.bits.bit12 = ecc_last.bits.bit4;
                    ecc.bits.bit13 = ecc_last.bits.bit5;
                    ecc.bits.bit14 = ecc_last.bits.bit6;
                    ecc.bits.bit15 = ecc_last.bits.bit7 ^ ecc_last.bits.bit8 ^ ecc_last.bits.bit9 ^ ecc_last.bits.bit10
                        ^ ecc_last.bits.bit11 ^ ecc_last.bits.bit12 ^ ecc_last.bits.bit13 ^ ecc_last.bits.bit14 ^ ecc_last.bits.bit15
                        ^ temp_byte.bits.bit0 ^ temp_byte.bits.bit1 ^ temp_byte.bits.bit2 ^ temp_byte.bits.bit3
                        ^ temp_byte.bits.bit4 ^ temp_byte.bits.bit5 ^ temp_byte.bits.bit6 ^ temp_byte.bits.bit7;
                    ecc_last.dshort = ecc.dshort;
                    
                }
            }
        }
        pos += bank_len + 4;        
    }

    *ecc_val = ecc.dshort;
    return 0;
}


/* calculate lcd init code checksum */
static unsigned short focal_cal_lcdinitcode_checksum(u8 *ptr , int length)
{
    //CRC16
    u16 cFcs = 0;
    int i, j;

    if (length%2) {
        return 0xFFFF;
    }

    for ( i = 0; i < length; i += 2) {
        cFcs ^= ((ptr[i] << 8) + ptr[i+1]);
        for (j = 0; j < 16; j ++) {
            if (cFcs & 1) {
                cFcs = (unsigned short)((cFcs >> 1) ^ ((1 << 15) + (1 << 10) + (1 << 3)));
            } else {
                cFcs >>= 1;
            }
        }
    }
    return cFcs;
}

static int focal_read_replace_3gamma(struct i2c_client *client, u8 *buf)
{
    int ret = 0;
    u16 initcode_ecc = 0;
    u16 initcode_checksum = 0;
    u8 *gamma = NULL;
    u16 gamma_len = 0;

    gamma = kmalloc(MAX_GAMMA_LEN, GFP_KERNEL);
    if(NULL == gamma) {
        TPD_INFO("malloc gamma memory fail\n");
        return -ENOMEM;
    }

    ret = focal_read_3gamma(client, &gamma, &gamma_len); //read out 3gamma data from ic backup
    if(ret < 0) {
        TPD_INFO("no valid 3-gamma data, not replace\n");
        goto OUT;
    }

    ret = focal_replace_3gamma(buf, gamma, gamma_len); //replace initcode in fw with reading out 3gamma
    if(ret < 0) {
        TPD_INFO("replace 3-gamma fail\n");
        goto OUT;
    }

    ret = focal_cal_lcdinitcode_ecc(buf, &initcode_ecc);
    if (ret < 0) {
        TPD_INFO("lcd init code ecc calculate fail\n");
        goto OUT;
    }
    TPD_INFO("lcd init code cal ecc:%04x\n", initcode_ecc);
    buf[4] = (u8)(initcode_ecc >> 8);
    buf[5] = (u8)(initcode_ecc);
    buf[0x43d] = (u8)(initcode_ecc >> 8);
    buf[0x43c] = (u8)(initcode_ecc);

    initcode_checksum = focal_cal_lcdinitcode_checksum(buf + 2, 0x43e - 2);
    TPD_INFO("lcd init code calc checksum:%04x\n", initcode_checksum);
    buf[0] = (u8)(initcode_checksum >> 8);
    buf[1] = (u8)(initcode_checksum);

OUT:
    kfree(gamma);
    return ret < 0 ? ret : 0;
}

int focal_check_initialCode_valid(struct i2c_client *client, u8 *buf)
{
    int ret = 0;
    u16 initcode_ecc = 0;
    u16 initcode_checksum = 0;

    initcode_checksum = focal_cal_lcdinitcode_checksum(buf + 2, 0x43e - 2);
    TPD_INFO("lcd init code calc checksum:%04x\n", initcode_checksum);
    if (initcode_checksum != ((u16)((u16)buf[0] << 8) + buf[1])) {
        TPD_INFO("Initial Code checksum fail\n");
        return -EINVAL;
    }

    ret = focal_cal_lcdinitcode_ecc(buf, &initcode_ecc);
    if (ret < 0) {
        TPD_INFO("lcd init code ecc calculate fail\n");
        return ret;
    }
    TPD_INFO("lcd init code cal ecc:%04x\n", initcode_ecc);
    if(initcode_ecc != ((u16)((u16)buf[4] << 8) + buf[5])) {
        TPD_INFO("Initial Code ecc check fail\n");
        return -EINVAL;
    }

    return 0;
}
/******************* End of function work for focal debug callbacks***************************/

/****************** Start of implementation of focal debug callbacks**************************/
static int focal_upgrade_use_buf(void *chip_data, u8 *pbt_buf, u32 dw_lenth)
{
    u8 reg_val[4] = {0};
    u32 i = 0;
    u32 packet_number;
    u32 j = 0;
    u32 temp;
    u32 lenght;
    u8 packet_buf[FTS_PACKET_LENGTH + 6];
    u8 auc_i2c_write_buf[10];
    u8 upgrade_ecc;
    int i_ret = 0;
    bool inbootloader = false;
    struct chip_data_ft8006 *chip_info = (struct chip_data_ft8006 *)chip_data;

    TPD_INFO("[UPGRADE]**********send 0xAA and 0x55 to FW, start upgrade**********\n");
    i_ret = focal_start_fw_upgrade(chip_info);
    if (i_ret < 0) {
        TPD_INFO( "[UPGRADE]: send upgrade cmd to FW error!!\n");
        return i_ret;
    }

    TPD_INFO("[UPGRADE]**********check if run in bootloader mode**********\n");
    inbootloader = focal_check_run_state(chip_info, FTS_RUN_IN_BOOTLOADER);
    if (!inbootloader) {
        TPD_INFO( "[UPGRADE]: not run in bootloader, upgrade fail!!\n");
        return -EIO;
    }

    /*send upgrade type to reg 0x09: 0x0B: upgrade; 0x0A: download*/
    auc_i2c_write_buf[0] = 0x09;
    auc_i2c_write_buf[1] = 0x0B;
    touch_i2c_write(chip_info->client, auc_i2c_write_buf, 2);

    /* All.bin <= 128K
     * APP.bin <= 94K
     * LCD_CFG <= 4K */
    auc_i2c_write_buf[0] = 0xB0;
    auc_i2c_write_buf[1] = (u8) ((dw_lenth >> 16) & 0xFF);
    auc_i2c_write_buf[2] = (u8) ((dw_lenth >> 8) & 0xFF);
    auc_i2c_write_buf[3] = (u8) (dw_lenth & 0xFF);
    touch_i2c_write(chip_info->client, auc_i2c_write_buf, 4);

    /*erase the app erea in flash*/
    TPD_INFO("[UPGRADE]**********start earse flash area**********\n");
    i_ret = focal_erase_flash(chip_info);
    if (i_ret < 0) {
        TPD_INFO( "[UPGRADE]: erase flash error!!\n");
        return i_ret;
    }

    /*write FW to ctpm flash*/
    upgrade_ecc = 0;
    temp = 0;
    packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
    packet_buf[0] = FTS_FW_WRITE_CMD;

    TPD_INFO("[UPGRADE]**********start write fw to flash**********\n");
    for (j = 0; j < packet_number; j++) {
        temp = 0x5000 + j * FTS_PACKET_LENGTH;
        packet_buf[1] = (u8) (temp >> 16);
        packet_buf[2] = (u8) (temp >> 8);
        packet_buf[3] = (u8) temp;
        lenght = FTS_PACKET_LENGTH;
        packet_buf[4] = (u8) (lenght >> 8);
        packet_buf[5] = (u8) lenght;
        for (i = 0; i < FTS_PACKET_LENGTH; i++) {
            packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
            upgrade_ecc ^= packet_buf[6 + i];
        }
        touch_i2c_write(chip_info->client, packet_buf, FTS_PACKET_LENGTH + 6);
        //msleep(1);

        for (i = 0; i < 30; i++) {
            auc_i2c_write_buf[0] = 0x6a;
            reg_val[0] = reg_val[1] = 0x00;
            touch_i2c_read(chip_info->client, auc_i2c_write_buf, 1, reg_val, 2);

            if ((j + 0x1000 + (0x5000/FTS_PACKET_LENGTH)) == (((reg_val[0]) << 8) | reg_val[1])) {
                break;
            }

            if (i > 15) {
                msleep(1);
                TPD_INFO("[UPGRADE]: write flash: host : %x status : %x!!\n", (j + 0x1000 + (0x5000/FTS_PACKET_LENGTH)), (((reg_val[0]) << 8) | reg_val[1]));
            }
            //msleep(1);
            focal_upgrade_delay(10000);
        }
    }

    if ((dw_lenth) % FTS_PACKET_LENGTH > 0) {
        temp = 0x5000 + packet_number * FTS_PACKET_LENGTH;
        packet_buf[1] = (u8) (temp >> 16);
        packet_buf[2] = (u8) (temp >> 8);
        packet_buf[3] = (u8) temp;
        temp = (dw_lenth) % FTS_PACKET_LENGTH;
        packet_buf[4] = (u8) (temp >> 8);
        packet_buf[5] = (u8) temp;
        for (i = 0; i < temp; i++) {
            packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
            upgrade_ecc ^= packet_buf[6 + i];
        }
        touch_i2c_write(chip_info->client, packet_buf, temp + 6);
        //msleep(1);

        for (i = 0; i < 30; i++) {
            auc_i2c_write_buf[0] = 0x6a;
            reg_val[0] = reg_val[1] = 0x00;
            touch_i2c_read(chip_info->client, auc_i2c_write_buf, 1, reg_val, 2);

            if ((0x1000 + ((0x5000 + packet_number * FTS_PACKET_LENGTH)/((dw_lenth) % FTS_PACKET_LENGTH))) == (((reg_val[0]) << 8) | reg_val[1])) {
                break;
            }

            if (i > 15) {
                msleep(1);
                TPD_INFO("[UPGRADE]: write flash: host : %x status : %x!!\n", (j + 0x1000 + (0x5000/FTS_PACKET_LENGTH)), (((reg_val[0]) << 8) | reg_val[1]));
            }
            //msleep(1);
            focal_upgrade_delay(10000);
        }
    }

    msleep(50);

    /*********Step 6: read out checksum***********************/
    /*send the opration head */
    TPD_INFO("[UPGRADE]**********start read checksum**********\n");
    auc_i2c_write_buf[0] = 0x64;
    touch_i2c_write(chip_info->client, auc_i2c_write_buf, 1);
    msleep(300);

    temp = 0x5000;
    auc_i2c_write_buf[0] = 0x65;
    auc_i2c_write_buf[1] = (u8)(temp >> 16);
    auc_i2c_write_buf[2] = (u8)(temp >> 8);
    auc_i2c_write_buf[3] = (u8)(temp);
    temp = (64*1024-1);
    auc_i2c_write_buf[4] = (u8)(temp >> 8);
    auc_i2c_write_buf[5] = (u8)(temp);
    i_ret = touch_i2c_write(chip_info->client, auc_i2c_write_buf, 6);
    msleep(dw_lenth/256);

    temp = (0x5000+(64*1024-1));
    auc_i2c_write_buf[0] = 0x65;
    auc_i2c_write_buf[1] = (u8)(temp >> 16);
    auc_i2c_write_buf[2] = (u8)(temp >> 8);
    auc_i2c_write_buf[3] = (u8)(temp);
    temp = (dw_lenth-(64*1024-1));
    auc_i2c_write_buf[4] = (u8)(temp >> 8);
    auc_i2c_write_buf[5] = (u8)(temp);
    i_ret = touch_i2c_write(chip_info->client, auc_i2c_write_buf, 6);
    msleep(dw_lenth/256);

    for (i = 0; i < 100; i++) {
        auc_i2c_write_buf[0] = 0x6a;
        reg_val[0] = reg_val[1] = 0x00;
        touch_i2c_read(chip_info->client, auc_i2c_write_buf, 1, reg_val, 2);

        if (0xF0==reg_val[0] && 0x55==reg_val[1]) {
            TPD_DEBUG("[UPGRADE]: reg_val[0]=%02x reg_val[0]=%02x!!", reg_val[0], reg_val[1]);
            break;
        }
        msleep(1);
    }
    auc_i2c_write_buf[0] = 0x66;
    touch_i2c_read(chip_info->client, auc_i2c_write_buf, 1, reg_val, 1);
    if (reg_val[0] != upgrade_ecc) {
        TPD_INFO("[UPGRADE]: ecc error! FW=%02x upgrade_ecc=%02x!!\n",reg_val[0],upgrade_ecc);
        return -EIO;
    }
    TPD_INFO("[UPGRADE]: checksum %x %x!!\n",reg_val[0],upgrade_ecc);

    TPD_INFO("[UPGRADE]: reset the new FW!!\n");
    auc_i2c_write_buf[0] = 0x07;    //FTS_REG_RESET_FW
    touch_i2c_write(chip_info->client, auc_i2c_write_buf, 1);
    msleep(1000);

    TPD_INFO("[UPGRADE]**********end of fw upgrade sucess**********\n");
    return 0;
}

static int focal_lcd_cfg_upgrade_use_buf(void *chip_data, u8* pbt_buf, u32 dw_lenth)
{
    u8 reg_val[4] = {0};
    u32 i = 0;
    u32 packet_number;
    u32 j = 0;
    u32 temp;
    u32 lenght;
    bool inbootloader = false;
    u8 packet_buf[FTS_PACKET_LENGTH + 6];
    u8 auc_i2c_write_buf[10];
    u8 upgrade_ecc;
    int i_ret;
    struct chip_data_ft8006 *chip_info = (struct chip_data_ft8006 *)chip_data;

    TPD_INFO("[UPGRADE]**********send 0xAA and 0x55 to FW, start upgrade**********\n");
    i_ret = focal_start_fw_upgrade(chip_info);
    if (i_ret < 0) {
        TPD_INFO( "[UPGRADE]: send upgrade cmd to FW error!!\n");
    }

    TPD_INFO("[UPGRADE]**********check if run in bootloader mode**********\n");
    inbootloader = focal_check_run_state(chip_info, FTS_RUN_IN_BOOTLOADER);
    if (!inbootloader) {
        TPD_INFO( "[UPGRADE]: not run in bootloader, upgrade fail!!\n");
        return -EIO;
    }

    i_ret = focal_read_replace_3gamma(chip_info->client, pbt_buf);
    if(i_ret < 0) {
        TPD_INFO("replace 3-gamma fail, not upgrade lcd init code\n");
        return i_ret;
    }

    i_ret = focal_check_initialCode_valid(chip_info->client, pbt_buf);
    if(i_ret < 0) {
        TPD_INFO("initial code invalid, not upgrade lcd init code");
        return i_ret;
    }

    /*send upgrade type to reg 0x09: 0x0B: upgrade; 0x0A: download*/
    auc_i2c_write_buf[0] = 0x09;
    auc_i2c_write_buf[1] = 0x0C;
    touch_i2c_write(chip_info->client, auc_i2c_write_buf, 2);

    /*Step 4:erase app and panel paramenter area*/
    TPD_INFO("[UPGRADE]**********erase app and panel paramenter area**********\n");
    auc_i2c_write_buf[0] = chip_info->reg_info.FTS_ERASE_APP_REG;
    touch_i2c_write(chip_info->client, auc_i2c_write_buf, 1);
    msleep(1000);

    for (i = 0; i < 15; i++) {
        auc_i2c_write_buf[0] = 0x6a;
        reg_val[0] = reg_val[1] = 0x00;
        touch_i2c_read(chip_info->client, auc_i2c_write_buf, 1, reg_val, 2);
        if (0xF0==reg_val[0] && 0xAA==reg_val[1]) {
            break;
        }
        msleep(50);
    }
    TPD_INFO("[UPGRADE]: erase app area reg_val[0] = %x reg_val[1] = %x!!\n", reg_val[0], reg_val[1]);

    auc_i2c_write_buf[0] = 0xB0;
    auc_i2c_write_buf[1] = 0;
    auc_i2c_write_buf[2] = (u8) ((dw_lenth >> 8) & 0xFF);
    auc_i2c_write_buf[3] = (u8) (dw_lenth & 0xFF);
    touch_i2c_write(chip_info->client, auc_i2c_write_buf, 4);

    /*write FW to ctpm flash*/
    upgrade_ecc = 0;
    TPD_INFO("[UPGRADE]**********write fw to lcd flash area**********\n");
    temp = 0;
    packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
    packet_buf[0] = FTS_FW_WRITE_CMD;
    packet_buf[1] = 0;
    for (j = 0; j < packet_number; j++) {
        temp = j * FTS_PACKET_LENGTH;
        packet_buf[2] = (u8) (temp >> 8);
        packet_buf[3] = (u8) temp;
        lenght = FTS_PACKET_LENGTH;
        packet_buf[4] = (u8) (lenght >> 8);
        packet_buf[5] = (u8) lenght;
        for (i = 0; i < FTS_PACKET_LENGTH; i++) {
            packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
            upgrade_ecc ^= packet_buf[6 + i];
        }
        touch_i2c_write(chip_info->client, packet_buf, FTS_PACKET_LENGTH + 6);

        for (i = 0; i < 30; i++) {
            auc_i2c_write_buf[0] = 0x6a;
            reg_val[0] = reg_val[1] = 0x00;
            touch_i2c_read(chip_info->client, auc_i2c_write_buf, 1, reg_val, 2);

            if ((j + 0x1000) == (((reg_val[0]) << 8) | reg_val[1])) {
                break;
            }

            if (i > 15) {
                msleep(1);
                TPD_INFO("[UPGRADE]: write flash: host : %x status : %x!!\n", (j + 0x1000 + (0x5000/FTS_PACKET_LENGTH)), (((reg_val[0]) << 8) | reg_val[1]));
            }
            focal_upgrade_delay(10000);
        }
    }

    if ((dw_lenth) % FTS_PACKET_LENGTH > 0) {
        temp = packet_number * FTS_PACKET_LENGTH;
        packet_buf[2] = (u8) (temp >> 8);
        packet_buf[3] = (u8) temp;
        temp = (dw_lenth) % FTS_PACKET_LENGTH;
        packet_buf[4] = (u8) (temp >> 8);
        packet_buf[5] = (u8) temp;
        for (i = 0; i < temp; i++) {
            packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
            upgrade_ecc ^= packet_buf[6 + i];
        }
        touch_i2c_write(chip_info->client, packet_buf, temp + 6);

        for (i = 0; i < 30; i++) {
            auc_i2c_write_buf[0] = 0x6a;
            reg_val[0] = reg_val[1] = 0x00;
            touch_i2c_read(chip_info->client, auc_i2c_write_buf, 1, reg_val, 2);

            if ((0x1000 + ((packet_number * FTS_PACKET_LENGTH)/((dw_lenth) % FTS_PACKET_LENGTH))) == (((reg_val[0]) << 8) | reg_val[1])) {
                break;
            }

            if (i > 15) {
                msleep(1);
                TPD_INFO("[UPGRADE]: write flash: host : %x status : %x!!\n", (j + 0x1000 + (0x5000/FTS_PACKET_LENGTH)), (((reg_val[0]) << 8) | reg_val[1]));
            }
            focal_upgrade_delay(10000);
        }
    }

    msleep(50);

    /*********Step 6: read out checksum***********************/
    /*send the opration head */
    TPD_INFO("[UPGRADE]: read out checksum!!\n");
    auc_i2c_write_buf[0] = 0x64;
    touch_i2c_write(chip_info->client, auc_i2c_write_buf, 1);
    msleep(300);

    temp = 0x00;
    auc_i2c_write_buf[0] = 0x65;
    auc_i2c_write_buf[1] = 0;
    auc_i2c_write_buf[2] = (u8)(temp >> 8);
    auc_i2c_write_buf[3] = (u8)(temp);
    temp = dw_lenth;
    auc_i2c_write_buf[4] = (u8)(temp >> 8);
    auc_i2c_write_buf[5] = (u8)(temp);
    i_ret = touch_i2c_write(chip_info->client, auc_i2c_write_buf, 6);
    msleep(dw_lenth/256);

    for (i = 0; i < 100; i++) {
        auc_i2c_write_buf[0] = 0x6a;
        reg_val[0] = reg_val[1] = 0x00;
        touch_i2c_read(chip_info->client, auc_i2c_write_buf, 1, reg_val, 2);

        if (0xF0==reg_val[0] && 0x55==reg_val[1]) {
            TPD_INFO("[UPGRADE]: reg_val[0]=%02x reg_val[0]=%02x!!\n", reg_val[0], reg_val[1]);
            break;
        }
        msleep(1);
    }
    auc_i2c_write_buf[0] = 0x66;
    touch_i2c_read(chip_info->client, auc_i2c_write_buf, 1, reg_val, 1);
    if (reg_val[0] != upgrade_ecc) {
        TPD_INFO("[UPGRADE]: ecc error! FW=%02x upgrade_ecc=%02x!!\n",reg_val[0],upgrade_ecc);
        return -EIO;
    }
    TPD_INFO("[UPGRADE]: checksum %x %x!!\n",reg_val[0],upgrade_ecc);

    TPD_INFO("[UPGRADE]: reset the new FW!!\n");
    auc_i2c_write_buf[0] = 0x07;    //FTS_REG_RESET_FW
    touch_i2c_write(chip_info->client, auc_i2c_write_buf, 1);
    msleep(1000);

    return 0;
}

static void focal_esd_check_enable(void *chip_data, bool enable)
{
    struct chip_data_ft8006 *chip_info = (struct chip_data_ft8006 *)chip_data;
    chip_info->esd_check_need_stop = !enable;
}

static bool focal_get_esd_check_flag(void *chip_data)
{
    struct chip_data_ft8006 *chip_info = (struct chip_data_ft8006 *)chip_data;
    return chip_info->esd_check_need_stop;
}

static int focal_get_fw_version(void *chip_data)
{
    struct chip_data_ft8006 *chip_info = (struct chip_data_ft8006 *)chip_data;
    return touch_i2c_read_byte(chip_info->client, chip_info->reg_info.FTS_REG_FW_VER);
}

static int focal_dump_reg_state(void *chip_data, char *buf)
{
    int count = 0;
    u8 regvalue = 0;
    struct chip_data_ft8006 *chip_info = (struct chip_data_ft8006 *)chip_data;

    //power mode 0:active 1:monitor 3:sleep
    regvalue = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.FTS_REG_POWER_MODE);
    count += sprintf(buf + count, "Power Mode:0x%02x\n", regvalue);

    //FW version
    regvalue = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.FTS_REG_FW_VER);
    count += sprintf(buf + count, "FW Ver:0x%02x\n",regvalue);

    //Vendor ID
    regvalue = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.FTS_REG_VENDOR_ID);
    count += sprintf(buf + count, "Vendor ID:0x%02x\n", regvalue);

    // 1 Gesture mode,0 Normal mode
    regvalue = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.FTS_REG_GESTURE_EN);
    count += sprintf(buf + count, "Gesture Mode:0x%02x\n", regvalue);

    // 3 charge in
    regvalue = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.FTS_REG_CHARGER_MODE_EN);
    count += sprintf(buf + count, "charge stat:0x%02x\n", regvalue);

    //Interrupt counter
    regvalue = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.FTS_REG_INT_CNT);
    count += sprintf(buf + count, "INT count:0x%02x\n", regvalue);

    //Flow work counter
    regvalue = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.FTS_REG_FLOW_WORK_CNT);
    count += sprintf(buf + count, "ESD count:0x%02x\n", regvalue);

    return count;
}

struct focal_debug_func focal_debug_ops =
{
    .esd_check_enable               = focal_esd_check_enable,
    .get_esd_check_flag             = focal_get_esd_check_flag,
    .reset                          = focal_reset_via_gpio,
    .get_fw_version                 = focal_get_fw_version,
    .dump_reg_sate                  = focal_dump_reg_state,
};
/******************* End of implementation of focal debug callbacks***************************/

/**************** Start of implementation of debug_info proc callbacks************************/
static void focal_delta_read(struct seq_file *s, void *chip_data)
{
    int ret = 0, x = 0, y = 0, z = 0, retry = 21;
    int16_t temp_delta = 0;
    uint8_t *raw_data = NULL;
    struct chip_data_ft8006 *chip_info = (struct chip_data_ft8006 *)chip_data;

    if (!chip_info)
        return;

    focal_esd_check_enable(chip_info, false);   //no allowed esd check
    //wait data can be read
    while (--retry) {
        ret = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.FTS_REG_RAWDIFF_CONTR);
        if (!ret) {
            break;
        }
        msleep(1);
    }
    if (!retry) {
        seq_printf(s, "wait data be ready timed out\n");
        focal_esd_check_enable(chip_info, true);    //allowed esd check
        return;
    }

    raw_data = kzalloc(chip_info->hw_res->TX_NUM * chip_info->hw_res->RX_NUM * 2 * (sizeof(uint8_t)), GFP_KERNEL);
    if (!raw_data) {
        TPD_INFO("raw_data kzalloc error\n");
        focal_esd_check_enable(chip_info, true);    //allowed esd check
        return;
    }

    //open interface to read delta and rawdata
    ret = touch_i2c_write_byte(chip_info->client, chip_info->reg_info.FTS_REG_RAWDIFF_CONTR, 0x01);
    msleep(10);
    ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.FTS_REG_DIFF_DATA, chip_info->hw_res->TX_NUM * chip_info->hw_res->RX_NUM * 2, raw_data);     //read data
    for (x = 0; x < chip_info->hw_res->TX_NUM; x++) {
        seq_printf(s, "\n[%2d]", x);
        for (y = 0; y < chip_info->hw_res->RX_NUM; y++) {
            z = chip_info->hw_res->RX_NUM * x + y;
            temp_delta = raw_data[z * 2 + 1] | (raw_data[z * 2] << 8);
            seq_printf(s, "%4d, ", temp_delta);
        }
    }
    seq_printf(s, "\n");
    ret = touch_i2c_write_byte(chip_info->client, chip_info->reg_info.FTS_REG_RAWDIFF_CONTR, 0x00);

    focal_esd_check_enable(chip_info, true);    //allowed esd check
    kfree(raw_data);
}

static void focal_baseline_read(struct seq_file *s, void *chip_data)
{
    int ret = 0, x = 0, y = 0, z = 0, retry = 21;
    int16_t temp_delta = 0;
    uint8_t *baseline_data = NULL;
    struct chip_data_ft8006 *chip_info = (struct chip_data_ft8006 *)chip_data;

    if (!chip_info)
        return;

    focal_esd_check_enable(chip_info, false);   //no allowed esd check
    //wait data can be read
    while (--retry) {
        ret = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.FTS_REG_RAWDIFF_CONTR);
        if (!ret) {
            break;
        }
        msleep(1);
    }
    if (!retry) {
        seq_printf(s, "wait data be ready timed out\n");
        focal_esd_check_enable(chip_info, true);    //allowed esd check
        return;
    }

    baseline_data = kzalloc(chip_info->hw_res->TX_NUM * chip_info->hw_res->RX_NUM * 2 * (sizeof(uint8_t)), GFP_KERNEL);
    if (!baseline_data) {
        TPD_INFO("baseline_data kzalloc error\n");
        focal_esd_check_enable(chip_info, true);    //allowed esd check
        return;
    }

    //open interface to read delta and rawdata
    ret = touch_i2c_write_byte(chip_info->client, chip_info->reg_info.FTS_REG_RAWDIFF_CONTR, 0x01);
    msleep(10);
    ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.FTS_REG_RAW_DATA, chip_info->hw_res->TX_NUM * chip_info->hw_res->RX_NUM * 2, baseline_data);  //read data
    for (x = 0; x < chip_info->hw_res->TX_NUM; x++) {
        seq_printf(s, "\n[%2d]", x);
        for (y = 0; y < chip_info->hw_res->RX_NUM; y++) {
            z = chip_info->hw_res->RX_NUM * x + y;
            temp_delta = baseline_data[z * 2 + 1] | (baseline_data[z * 2] << 8);
            seq_printf(s, "%4d, ", temp_delta);
        }
    }
    seq_printf(s, "\n");
    ret = touch_i2c_write_byte(chip_info->client, chip_info->reg_info.FTS_REG_RAWDIFF_CONTR, 0x00);

    focal_esd_check_enable(chip_info, true);    //allowed esd check
    kfree(baseline_data);
}

static void focal_main_register_read(struct seq_file *s, void *chip_data)
{
    u8 regvalue = 0;
    struct chip_data_ft8006 *chip_info = (struct chip_data_ft8006 *)chip_data;

    focal_esd_check_enable(chip_info, false);   //no allowed esd check

    //power mode 0:active 1:monitor 3:sleep
    regvalue = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.FTS_REG_POWER_MODE);
    seq_printf(s, "Power Mode:0x%02x\n", regvalue);

    //TP FW version
    regvalue = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.FTS_REG_FW_VER);
    seq_printf(s, "TP FW Ver:0x%02x\n",regvalue);

    //initcode version
    regvalue = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.FTS_REG_INITCODE_VER);
    seq_printf(s, "initcode Ver:0x%02x\n",regvalue);

    //Vendor ID
    regvalue = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.FTS_REG_VENDOR_ID);
    seq_printf(s, "Vendor ID(0x8d/0xda/0x5a):0x%02x\n", regvalue);

    // 1 Gesture mode,0 Normal mode
    regvalue = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.FTS_REG_GESTURE_EN);
    seq_printf(s, "Gesture Mode:0x%02x\n", regvalue);

    // 3 charge in
    regvalue = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.FTS_REG_CHARGER_MODE_EN);
    seq_printf(s, "charge state:0x%02x\n", regvalue);

    //Interrupt counter
    regvalue = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.FTS_REG_INT_CNT);
    seq_printf(s, "INT count:0x%02x\n", regvalue);

    //Flow work counter
    regvalue = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.FTS_REG_FLOW_WORK_CNT);
    seq_printf(s, "ESD count:0x%02x\n", regvalue);

    //Panel ID
    regvalue = touch_i2c_read_byte(chip_info->client, 0xE3);
    seq_printf(s, "PANEL ID(0x8a-tianma/0xb0-truly/0xc3-boeb3/0xc8-boeb8):0x%02x\n", regvalue);

    //Work Freq
    regvalue = touch_i2c_read_byte(chip_info->client, 0xFE);
    seq_printf(s, "work Freq:0x%02x\n", regvalue);

    focal_esd_check_enable(chip_info, true);    //allowed esd check

    return;
}

static void focal_cb_read(struct seq_file *s, void *chip_data)
{
    int ret = 0, x = 0, y = 0, z = 0, i = 0;
    u8 regvalue = 0;
    int16_t temp_delta = 0;
    uint8_t *cb_data = NULL;
    struct chip_data_ft8006 *chip_info = (struct chip_data_ft8006 *)chip_data;

    if (!chip_info)
        return;

    focal_esd_check_enable(chip_info, false); //no allowed esd check
    cb_data = kzalloc(chip_info->hw_res->TX_NUM * chip_info->hw_res->RX_NUM * 2 * (sizeof(uint8_t)), GFP_KERNEL);
    if (!cb_data) {
        TPD_INFO("cb_data kzalloc error\n");
        focal_esd_check_enable(chip_info, true);
        return;
    }

    // enter factory mode
    ret = touch_i2c_write_byte(chip_info->client, 0x00, 0x40);
    msleep(200);

    //auto clb
    ret = touch_i2c_write_byte(chip_info->client, 0x04, 0x04);
    for (i = 0; i < 50; i++) {
      msleep(100);
      ret = touch_i2c_write_byte(chip_info->client, 0x00, 0x04 << 4);
      regvalue = touch_i2c_read_byte(chip_info->client, 0x04);
      if( regvalue == 0x02)
            break;
    }
    if (i == 50) {
      seq_printf(s, "calibration failed\n");
      goto OUT;
    }

    // read cb
    seq_printf(s, "cb data:");
    ret = touch_i2c_write_byte(chip_info->client, 0x18, 0x00);//Address offset high 8 bit
    ret = touch_i2c_write_byte(chip_info->client, 0x19, 0x00);//Address offset low 8 bit 
    ret = touch_i2c_read_block(chip_info->client, 0x6E, chip_info->hw_res->TX_NUM * chip_info->hw_res->RX_NUM, cb_data);  //read data
    for (x = 0; x < chip_info->hw_res->TX_NUM; x++) {
        seq_printf(s, "\n[%2d]", x);
        for (y = 0; y < chip_info->hw_res->RX_NUM; y++) {
            z = chip_info->hw_res->RX_NUM * x + y;
            temp_delta = cb_data[z];
            seq_printf(s, "%4d, ", temp_delta);
        }
    }
    seq_printf(s, "\n");

OUT:
    // enter work mode
    ret = touch_i2c_write_byte(chip_info->client, 0x00, 0x00);
    msleep(100);

    focal_esd_check_enable(chip_info, true); //allowed esd check
    kfree(cb_data);
}

static struct debug_info_proc_operations debug_info_proc_ops = {
    .delta_read    = focal_delta_read,
    .baseline_read = focal_baseline_read,
    .main_register_read = focal_main_register_read,
    .reserve_read = focal_cb_read,
};
/***************** End of implementation of debug_info proc callbacks*************************/

/*********** Start of I2C Driver and Implementation of it's callbacks*************************/
static int focal_tp_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct chip_data_ft8006 *chip_info = NULL;
    struct touchpanel_data *ts = NULL;
    int ret = -1;

    TPD_INFO("%s  is called\n", __func__);

    /* 1. alloc chip info */
    chip_info = kzalloc(sizeof(struct chip_data_ft8006), GFP_KERNEL);
    if (chip_info == NULL) {
        TPD_INFO("chip info kzalloc error\n");
        ret = -ENOMEM;
        return ret;
    }
    memset(chip_info, 0, sizeof(*chip_info));

    /* 2. Alloc common ts */
    ts = common_touch_data_alloc();
    if (ts == NULL) {
        TPD_INFO("ts kzalloc error\n");
        goto ts_malloc_failed;
    }
    memset(ts, 0, sizeof(*ts));

    /* 3. bind client and dev for easy operate */
    chip_info->client = client;
    ts->debug_info_ops = &debug_info_proc_ops;
    ts->client = client;
    ts->irq = client->irq;
    i2c_set_clientdata(client, ts);
    ts->dev = &client->dev;
    ts->chip_data = chip_info;
    chip_info->hw_res = &ts->hw_res;
    chip_info->esd_check_need_stop = false;
    chip_info->is_sleep_reg_write = false;
    g_chip_info = chip_info;

    /* 4. file_operations callbacks binding */
    ts->ts_ops = &focal_ops;
    ts->private_data = &focal_debug_ops;

    /* 5. register common touch device*/
    ret = register_common_touch_device(ts);
    if (ret < 0) {
        goto err_register_driver;
    }
    ts->skip_reset_in_resume = true;
    ts->skip_suspend_operate = true;
    ts->tp_suspend_order = LCD_TP_SUSPEND;
    ts->tp_resume_order = LCD_TP_RESUME;

    //reset esd handle time interval
    if (ts->esd_handle_support) {
        ts->esd_info.esd_work_time = HZ; // change esd check interval to 1s
        TPD_INFO("%s:change esd handle time to %d s\n", __func__, ts->esd_info.esd_work_time/HZ);
    }

    /* 6. collect data for supurious_fp_touch */
    if (ts->spurious_fp_support) {
        mutex_lock(&ts->mutex);
        focal_finger_proctect_data_get(chip_info);
        mutex_unlock(&ts->mutex);
    }

    /* 7. create focal apk debug files */
    focal_create_apk_debug_channel(ts);

    /* 8. create sys info file */
    focal_create_sysfs(client);

    TPD_INFO("%s, probe normal end\n", __func__);
    return 0;

err_register_driver:
    common_touch_data_free(ts);
    ts = NULL;

ts_malloc_failed:
    kfree(chip_info);
    chip_info = NULL;
    ret = -1;

    TPD_INFO("%s, probe error\n", __func__);
    return ret;
}

static int focal_tp_remove(struct i2c_client *client)
{
    struct touchpanel_data *ts = i2c_get_clientdata(client);

    TPD_INFO("%s is called\n", __func__);
    kfree(ts);

    return 0;
}

static int focal_i2c_suspend(struct device *dev)
{
    struct touchpanel_data *ts = dev_get_drvdata(dev);

    TPD_INFO("%s: is called\n", __func__);
    tp_i2c_suspend(ts);

    return 0;
}

static int focal_i2c_resume(struct device *dev)
{
    struct touchpanel_data *ts = dev_get_drvdata(dev);

    TPD_INFO("%s is called\n", __func__);
    tp_i2c_resume(ts);

    return 0;
}

static const struct i2c_device_id tp_id[] =
{
    {TPD_DEVICE, 0},
    {},
};

static struct of_device_id tp_match_table[] =
{
    { .compatible = TPD_DEVICE, },
    { },
};

static const struct dev_pm_ops tp_pm_ops = {
#ifdef CONFIG_FB
    .suspend = focal_i2c_suspend,
    .resume = focal_i2c_resume,
#endif
};

static struct i2c_driver tp_i2c_driver =
{
    .probe = focal_tp_probe,
    .remove = focal_tp_remove,
    .id_table = tp_id,
    .driver = {
        .name = TPD_DEVICE,
        .owner = THIS_MODULE,
        .of_match_table = tp_match_table,
        .pm = &tp_pm_ops,
    },
};
/******************* End of I2C Driver and It's dev_pm_ops***********************/

/***********************Start of module init and exit****************************/
static int __init tp_driver_init(void)
{
    TPD_INFO("%s is called\n", __func__);

    if (!tp_judge_ic_match(TPD_DEVICE))
        return -1;

    if (i2c_add_driver(&tp_i2c_driver)!= 0) {
        TPD_INFO("unable to add i2c driver.\n");
        return -1;
    }
    return 0;
}

static void __exit tp_driver_exit(void)
{
    i2c_del_driver(&tp_i2c_driver);
}

module_init(tp_driver_init);
module_exit(tp_driver_exit);
/***********************End of module init and exit*******************************/

MODULE_AUTHOR("FocalTech Driver");
MODULE_DESCRIPTION("FocalTech Touchscreen Driver");
MODULE_LICENSE("GPL v2");
