/**************************************************************
 * Copyright (c)  2008- 2030  Oppo Mobile communication Corp.ltd.
 * File       : GT1x_oppo.c
 * Description: Source file for Goodix GT5688 driver
 * Version   : 1.0
 * Date        : 2016-09-02
 * Author    : Tong.han@Bsp.Group.Tp
 * TAG         : BSP.TP.Init
 * ---------------- Revision History: --------------------------
 *   <version>    <date>          < author >                            <desc>
 * Revision 1.1, 2016-09-09, Tong.han@Bsp.Group.Tp, modify based on gerrit review result(http://gerrit.scm.adc.com:8080/#/c/223721/)
 ****************************************************************/
#include <linux/of_gpio.h>
#include <linux/sysfs.h>
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/hrtimer.h>
#include <linux/regulator/consumer.h>
#include <linux/kthread.h>
#include <linux/random.h>
#include <linux/sched.h>
#include <linux/sched/rt.h>
#include <linux/task_work.h>
#include <linux/jiffies.h>

#include "gt1x_oppo.h"

#define GTP_I2C_NAME                "Goodix-TS"

struct chip_data_gt5688 *g_chip_info = NULL;
unsigned long timeout = 0;

#define TPD_DEVICE "Goodix-gt5688"
#define TPD_INFO(fmt, arg...)        pr_err("[TP]"TPD_DEVICE ": " fmt, ##arg)
#define TPD_DEBUG(fmt, arg...)       do{\
    if (tp_debug)\
    pr_err("[TP]"TPD_DEVICE ": " fmt, ##arg);\
}while(0)

static int gpio_set_int_level(struct hw_resource *hw_res, bool level);

/**
 * gt1x_send_cmd -  send cmd to Goodix ic for spcial device control.
 * @client: i2c device struct.
 * @cmd: value for cmd reg(0x8040)
 * @data:value for data reg(0x8041)
 * Return  0: succeed, non-0: failed.
 */
static s32 gt1x_send_cmd(struct i2c_client *client, u8 cmd, u8 data)
{
    s32 ret;
    u8 buffer[3] = { cmd, data, 0 };
    static DEFINE_MUTEX(cmd_mutex);

    /**
     * cmd format:
     * reg:0x8040  cmd control(diffrent cmd indicates diffrent setting for touch IC )
     * reg:0x8041  data reg(Correspond with 0x8040)
     * reg:0x8042  checksum (sum(0x8040~0x8042) == 0)
     */
    mutex_lock(&cmd_mutex);
    buffer[2] = (u8) ((0 - cmd - data) & 0xFF);
    ret = touch_i2c_write_block(client, GTP_REG_CMD + 1, 2, &buffer[1]);
    ret = touch_i2c_write_byte(client, GTP_REG_CMD, buffer[0]);
    msleep(50);
    mutex_unlock(&cmd_mutex);

    return ret;
}

static int gesture_enter_doze(struct i2c_client *client)
{
    int retry = 0;

    TPD_DEBUG("Entering doze mode...\n");

    while (retry++ < 5) {
        if (!gt1x_send_cmd(client, 0x08, 0)) {
            TPD_INFO("Working in doze mode!\n");
            return 0;
        }
        msleep(10);
    }

    TPD_INFO("Send doze cmd failed.\n");
    return -1;
}

static s32 gt1x_enter_sleep(struct chip_data_gt5688 *chip_info)
{
    s32 retry = 0;

    gpio_set_int_level(chip_info->hw_res, false);
    mdelay(5);
    while (retry++ < 3) {
        if (!gt1x_send_cmd(chip_info->client, GTP_CMD_SLEEP, 0)) {
            TPD_INFO("Enter sleep mode!\n");
            return 0;
        }
        msleep(10);
    }

    TPD_INFO("Enter sleep mode failed.\n");

    return -1;
}

static int gpio_set_int_level(struct hw_resource *hw_res, bool level)
{
    int ret = -1;

    if (level) {
        ret = pinctrl_select_state(hw_res->pinctrl, hw_res->pin_set_high);
        if (ret)
            TPD_INFO("cannot set pin to high state");
    } else {
        ret = pinctrl_select_state(hw_res->pinctrl, hw_res->pin_set_low);
        if (ret)
            TPD_INFO("cannot set pin to low state");
    }

    return ret;
}

static int gpio_set_int_nopull(struct hw_resource *hw_res)
{
    int ret = -1;

    ret = pinctrl_select_state(hw_res->pinctrl, hw_res->pin_set_nopull);
    if (ret)
        TPD_INFO("cannot set pin to input state");

    return ret;
}

static void gt1x_select_addr(struct i2c_client *client, struct hw_resource *hw_res)
{
    gpio_direction_output(hw_res->reset_gpio, 0);
    if (client->addr == 0x14) {
        gpio_set_int_level(hw_res, true);
    } else {
        gpio_set_int_level(hw_res, false);
    }
    mdelay(2);
    gpio_direction_output(hw_res->reset_gpio, 1);
    mdelay(8);
    gpio_set_int_nopull(hw_res);
}

/*
 * Return 0: succeed, < 0 : failed.
 */
static s32 gt1x_set_reset_status(struct i2c_client* client)
{
    /* 0x8040 ~ 0x8043 */
    u8 value[] = {0xAA, 0x00, 0x56, 0xAA};
    int ret;

    ret = touch_i2c_write_block(client, GTP_REG_CMD + 1, 3, &value[1]);
    if (ret < 0) {
        TPD_DEBUG("TP set_reset_status failed\n");
        return ret;
    }

    return touch_i2c_write_byte(client, GTP_REG_CMD, value[0]);
}

/*
 * return success: 0 ; fail : negative
 */
static int goodix_reset(void *chip_data)
{
    int ret = 0;
    struct chip_data_gt5688 *chip_info = (struct chip_data_gt5688 *)chip_data;

    TPD_INFO("%s reset happened!\n", __func__);
    gt1x_select_addr(chip_info->client, chip_info->hw_res);
    msleep(50);
    ret = gt1x_set_reset_status(chip_info->client);
    chip_info->halt_status = false;
    return ret;
}

/**
 * gt1x_send_cfg - Send gt1x_config Function.
 * @config: pointer of the configuration array.
 * @cfg_len: length of configuration array.
 * Return 0:success, non-0:fail.
 */
static s32 gt1x_send_cfg(struct i2c_client *client, u8 * config, int cfg_len)
{
    int i;
    s32 ret = 0;
    s32 retry = 0;
    u16 checksum = 0;

    TPD_DEBUG("Driver send config, length:%d\n", cfg_len);

    for (i = 0; i < cfg_len - 3; i += 2) {
        checksum += (config[i] << 8) + config[i + 1];
    }
    if (!checksum) {
        TPD_INFO("Invalid config, all of the bytes is zero!\n");
        return -1;
    }

    checksum = 0 - checksum;
    TPD_DEBUG("Config checksum: 0x%04X\n", checksum);
    config[cfg_len - 3] = (checksum >> 8) & 0xFF;
    config[cfg_len - 2] = checksum & 0xFF;
    config[cfg_len - 1] = 0x01;

    while (retry++ < 5) {
        ret = touch_i2c_write_block(client, GTP_REG_CONFIG_DATA, cfg_len, config);
        if (ret == cfg_len) {
            msleep(200);    /* at least 200ms, wait for storing config into flash. */
            TPD_DEBUG("Send config successfully!\n");
            return 0;
        }
    }

    TPD_INFO("Send config failed!\n");

    return ret;
}

/**
 * gt1x_init_panel - Prepare config data for touch ic, don't call this function
 * after initialization.
 *
 * Return 0:success, < 0:fail.
 */
static int gt1x_init_panel(struct chip_data_gt5688 *chip_info, cfg_config cfg_config)
{
    s32 ret = 0;

    switch (cfg_config) {
        case CFG_NORMAL:
            chip_info->config_info.gt1x_config = GTP_CFG_GROUP_OFLIM;
            chip_info->config_info.gt1x_cfg_length = sizeof(GTP_CFG_GROUP_OFLIM);
            if (chip_info->tp_type == TP_BIEL) {
                chip_info->config_info.gt1x_config = GTP_CFG_GROUP_BIEL;
                chip_info->config_info.gt1x_cfg_length = sizeof(GTP_CFG_GROUP_BIEL);
            } else if (chip_info->tp_type == TP_TRULY) {
                chip_info->config_info.gt1x_config = GTP_CFG_GROUP_TRULY;
                chip_info->config_info.gt1x_cfg_length = sizeof(GTP_CFG_GROUP_TRULY);
            } else if (chip_info->tp_type == TP_BOE) {
                chip_info->config_info.gt1x_config = GTP_CFG_GROUP_BOE;
                chip_info->config_info.gt1x_cfg_length = sizeof(GTP_CFG_GROUP_BOE);
            }
            break;

        case CFG_NOISE:
            chip_info->config_info.gt1x_config = GTP_NOISE_CFG_GROUP_OFLIM;
            chip_info->config_info.gt1x_cfg_length = sizeof(GTP_NOISE_CFG_GROUP_OFLIM);
            if (chip_info->tp_type == TP_BIEL) {
                chip_info->config_info.gt1x_config = GTP_NOISE_CFG_GROUP_BIEL;
                chip_info->config_info.gt1x_cfg_length = sizeof(GTP_NOISE_CFG_GROUP_BIEL);
            } else if (chip_info->tp_type == TP_TRULY) {
                chip_info->config_info.gt1x_config = GTP_NOISE_CFG_GROUP_TRULY;
                chip_info->config_info.gt1x_cfg_length = sizeof(GTP_NOISE_CFG_GROUP_TRULY);
            } else if (chip_info->tp_type == TP_BOE) {
                chip_info->config_info.gt1x_config = GTP_NOISE_CFG_GROUP_BOE;
                chip_info->config_info.gt1x_cfg_length = sizeof(GTP_NOISE_CFG_GROUP_BOE);
            }
            break;

        default:
            break;
    }
    TPD_DEBUG("cfg_config %d sizeof(chip_info->gt1x_config) is %d\n", cfg_config, chip_info->config_info.gt1x_cfg_length);

    if (chip_info->config_info.gt1x_cfg_length < GTP_CONFIG_MIN_LENGTH || chip_info->config_info.gt1x_cfg_length > GTP_CONFIG_MAX_LENGTH) {
        TPD_INFO("Config is INVALID! You need to check you header file CFG_GROUP section!\n");
        return -1;
    }

    /* clear the flag, avoid failure when send the_config of driver. */
    chip_info->config_info.gt1x_config[0] &= 0x7F;

    /* match resolution when gt1x_abs_x_max & gt1x_abs_y_max have been set already */
    if ((chip_info->config_info.gt1x_abs_x_max == 0) && (chip_info->config_info.gt1x_abs_y_max == 0)) {
        chip_info->config_info.gt1x_abs_x_max = (chip_info->config_info.gt1x_config[RESOLUTION_LOC + 1] << 8) + chip_info->config_info.gt1x_config[RESOLUTION_LOC];
        chip_info->config_info.gt1x_abs_y_max = (chip_info->config_info.gt1x_config[RESOLUTION_LOC + 3] << 8) + chip_info->config_info.gt1x_config[RESOLUTION_LOC + 2];
        chip_info->config_info.gt1x_int_type = (chip_info->config_info.gt1x_config[TRIGGER_LOC]) & 0x03;
        chip_info->config_info.gt1x_wakeup_level = !(chip_info->config_info.gt1x_config[MODULE_SWITCH3_LOC] & 0x20);
    } else {
        chip_info->config_info.gt1x_config[RESOLUTION_LOC] = (u8) chip_info->config_info.gt1x_abs_x_max;
        chip_info->config_info.gt1x_config[RESOLUTION_LOC + 1] = (u8) (chip_info->config_info.gt1x_abs_x_max >> 8);
        chip_info->config_info.gt1x_config[RESOLUTION_LOC + 2] = (u8) chip_info->config_info.gt1x_abs_y_max;
        chip_info->config_info.gt1x_config[RESOLUTION_LOC + 3] = (u8) (chip_info->config_info.gt1x_abs_y_max >> 8);
        set_reg_bit(chip_info->config_info.gt1x_config[MODULE_SWITCH3_LOC], 5, !chip_info->config_info.gt1x_wakeup_level);
        chip_info->config_info.gt1x_config[TRIGGER_LOC] = (chip_info->config_info.gt1x_config[TRIGGER_LOC] & 0xFC) | chip_info->config_info.gt1x_int_type;
    }

    TPD_INFO("X_MAX = %d, Y_MAX = %d, TRIGGER = 0x%02x, WAKEUP_LEVEL = %d\n",
            chip_info->config_info.gt1x_abs_x_max, chip_info->config_info.gt1x_abs_y_max,
            chip_info->config_info.gt1x_int_type, chip_info->config_info.gt1x_wakeup_level);

    ret = gt1x_send_cfg(chip_info->client, chip_info->config_info.gt1x_config, chip_info->config_info.gt1x_cfg_length);

    return ret;
}

/**
 * Return 0: success, < 0: i2c read fail, >0: check fail
 */
s32 gt1x_i2c_read_dbl_check(struct i2c_client *client, u16 addr, u8 * buffer, s32 len)
{
    u8 buf[16] = {0};
    u8 confirm_buf[16] = {0};
    int ret = -1;

    if (len > 16) {
        TPD_INFO("i2c_read_dbl_check length %d is too long, exceed %zu\n", len, sizeof(buf));
        return ret;
    }

    memset(buf, 0xAA, sizeof(buf));
    ret = touch_i2c_read_block(client, addr, len, buf);
    if (ret < 0) {
        return ret;
    }

    msleep(5);
    memset(confirm_buf, 0, sizeof(confirm_buf));
    ret = touch_i2c_read_block(client, addr, len, confirm_buf);
    if (ret < 0) {
        return ret;
    }

    if (!memcmp(buf, confirm_buf, len)) {
        memcpy(buffer, confirm_buf, len);
        return 0;
    }

    TPD_INFO("i2c read 0x%04X, %d bytes, double check failed!\n", addr, len);

    return 1;
}

static int gt1x_i2c_write_with_readback(struct i2c_client *client, u16 addr, u8 * buffer, int length)
{
    u8 buf[100];

    int ret = touch_i2c_write_block(client, addr, length, buffer);
    if (ret < 0) {
        return ret;
    }

    ret = touch_i2c_read_block(client, addr, length, buf);
    if (ret < 0) {
        return ret;
    }

    if (memcmp(buf, buffer, length)) {
        return -1;
    }

    return 0;
}

static s32 gt1x_read_version(struct chip_data_gt5688 *chip_info)
{
    s32 ret = -1;
    u8 buf[12] = {0};
    u32 mask_id = 0;
    u32 patch_id = 0;
    u8 product_id[5] = {0};
    u8 sensor_id = 0;
    u8 match_opt = 0;
    int i, retry = 3;
    u8 checksum = 0;
    struct gt1x_version_info * ver_info = &chip_info->ver_info;

    while (retry--) {
        ret = gt1x_i2c_read_dbl_check(chip_info->client, GTP_REG_VERSION, buf, sizeof(buf));
        if (!ret) {
            checksum = 0;

            for (i = 0; i < sizeof(buf); i++) {
                checksum += buf[i];
            }

            if (checksum == 0 &&    /* first 3 bytes must be number or char */
                    IS_NUM_OR_CHAR(buf[0]) && IS_NUM_OR_CHAR(buf[1]) && IS_NUM_OR_CHAR(buf[2]) && buf[10] != 0xFF) {    /*sensor id == 0xFF, retry */
                break;
            } else {
                TPD_INFO("Read version failed!(checksum error)\n");
            }
        } else {
            TPD_INFO("Read version failed!\n");
        }

        TPD_DEBUG("Read version retry = %d\n", retry);
        msleep(100);
    }

    if (retry <= 0) {
        if (ver_info)
            ver_info->sensor_id = 0;
        TPD_INFO("Maybe the firmware of ic is error\n");
        //return -1;
    }

    mask_id = (u32) ((buf[7] << 16) | (buf[8] << 8) | buf[9]);
    patch_id = (u32) ((buf[4] << 16) | (buf[5] << 8) | buf[6]);
    memcpy(product_id, buf, 4);
    sensor_id = buf[10] & 0x0F;
    match_opt = (buf[10] >> 4) & 0x0F;

    TPD_INFO("IC VERSION:GT%s(Product)_%06X(Patch)_%04X(Mask)_%02X(SensorID)\n", product_id, patch_id, mask_id >> 8, sensor_id);

    if (ver_info != NULL) {
        ver_info->mask_id = mask_id;
        ver_info->patch_id = patch_id;
        memcpy(ver_info->product_id, product_id, 5);
        ver_info->sensor_id = sensor_id;
        ver_info->match_opt = match_opt;
    }

    return 0;
}

static u16 gt1x_calc_checksum(u8 * fw, u32 length)
{
    u32 i = 0;
    u32 checksum = 0;

    for (i = 0; i < length; i += 2) {
        checksum += (((int)fw[i]) << 8);
        checksum += fw[i + 1];
    }

    return (checksum & 0xFFFF);
}

static int gt1x_read_flash(struct i2c_client* client, u32 addr, int length)
{
    int wait_time;
    int ret = 0;
    u8 buffer[4];
    u16 read_addr = (addr >> 8);

    TPD_INFO("Read flash: 0x%04X, length: %d\n", addr, length);

    buffer[0] = 0;
    ret = gt1x_i2c_write_with_readback(client, 0x8022, buffer, 1);

    buffer[0] = ((length >> 8) & 0xFF);
    buffer[1] = (length & 0xFF);
    buffer[2] = ((read_addr >> 8) & 0xFF);
    buffer[3] = (read_addr & 0xFF);
    ret |= gt1x_i2c_write_with_readback(client, 0x8100, buffer, 4);

    buffer[0] = 0xAA;
    buffer[1] = 0xAA;
    ret |= touch_i2c_write_block(client, 0x8020, 2, buffer);
    if (ret < 0) {
        TPD_INFO("Error occured.\n");
        return ret;
    }

    wait_time = 200;
    while (wait_time > 0) {
        wait_time--;
        msleep(5);
        ret = gt1x_i2c_read_dbl_check(client, 0x8022, buffer, 1);
        if (ret) {
            continue;
        }
        if (buffer[0] == 0xBB) {
            TPD_INFO("Read success(addr: 0x%04X, length: %d)\n", addr, length);
            break;
        }
    }
    if (wait_time == 0) {
        TPD_INFO("Read Flash FAIL!\n");
        return -1;
    }

    return 0;
}

static int gt1x_recall_check(struct i2c_client* client, u8 * chk_src, u16 start_addr, u16 chk_length)
{
    u8 rd_buf[PACK_SIZE];
    s32 ret = 0;
    u16 len = 0;
    u32 compared_length = 0;

    while (chk_length > 0) {
        len = (chk_length > PACK_SIZE ? PACK_SIZE : chk_length);

        ret = touch_i2c_read_block(client, start_addr + compared_length, len, rd_buf);
        if (ret < 0) {
            TPD_INFO("recall i2c error, exit!\n");
            return ret;
        }

        if (memcmp(rd_buf, &chk_src[compared_length], len)) {
            TPD_INFO("Recall frame not equal(addr: 0x%04X)\n", start_addr + compared_length);
            return -1;
        }

        chk_length -= len;
        compared_length += len;
    }

    TPD_DEBUG("Recall check %d bytes(address: 0x%04X) success.\n", compared_length, start_addr);

    return 0;
}

#define getU32(a) ((u32)getUint((u8 *)(a), 4))
#define getU16(a) ((u16)getUint((u8 *)(a), 2))
static u32 getUint(u8 * buffer, int len)
{
    u32 num = 0;
    int i;
    for (i = 0; i < len; i++) {
        num <<= 8;
        num += buffer[i];
    }

    return num;
}

/*
 * gt1x_get_fw_data - Read firmware file data to buffer
 * Return: return a pointer pointed at the content of firmware
 */
static u8 *gt1x_get_fw_data(struct chip_data_gt5688 *chip_info, u32 offset, int length)
{
    memcpy(chip_info->update_info.buffer, chip_info->update_info.firmware_file_data + offset, length);

    return chip_info->update_info.buffer;
}

static int __gt1x_hold_ss51_dsp_20(struct i2c_client* client)
{
    int ret = -1;
    int retry = 0;
    u8 buf[1];
    int hold_times = 0;

    while (retry++ < 30) {
        // Hold ss51 & dsp
        buf[0] = 0x0C;
        ret = touch_i2c_write_byte(client, _rRW_MISCTL__SWRST_B0_, buf[0]);
        if (ret < 0) {
            TPD_INFO("Hold ss51 & dsp I2C error, retry:%d\n", retry);
            continue;
        }
        // Confirm hold
        ret = touch_i2c_read_byte(client, _rRW_MISCTL__SWRST_B0_);
        if (ret < 0) {
            TPD_INFO("Hold ss51 & dsp I2C error, retry:%d\n", retry);
            continue;
        }
        if (0x0C == ret) {
            if (hold_times++ < 20) {
                continue;
            } else {
                break;
            }
        }
        TPD_INFO("Hold ss51 & dsp confirm 0x4180 failed, value:%d\n", buf[0]);
    }

    if (retry >= 30) {
        TPD_INFO("Hold ss51&dsp failed!\n");
        return -1;
    }

    TPD_INFO("Hold ss51&dsp successfully.\n");

    return 0;
}

/**
 * Return 0: success, < 0: i2c read fail, >0: check fail
 */
static int gt1x_hold_ss51_dsp(struct i2c_client* client)
{
    int ret = 0, retry = 5;
    u8 buffer[2];

    do {
        ret = touch_i2c_read_block(client, 0x4220, 1, buffer);
    } while (retry-- && ret < 0);

    if (ret < 0)
        return -1;

    //hold ss51_dsp
    ret = __gt1x_hold_ss51_dsp_20(client);
    if (ret) {
        return ret;
    }
    // enable dsp & mcu power
    buffer[0] = 0x00;
    ret = gt1x_i2c_write_with_readback(client, _bRW_MISCTL__DSP_MCU_PWR_, buffer, 1);
    if (ret) {
        TPD_INFO("enabel dsp & mcu power fail!\n");
        return ret;
    }
    // disable watchdog
    buffer[0] = 0x00;
    ret = gt1x_i2c_write_with_readback(client, _bRW_MISCTL__TMR0_EN, buffer, 1);
    if (ret) {
        TPD_INFO("disable wdt fail!\n");
        return ret;
    }
    // clear cache
    buffer[0] = 0x00;
    ret = gt1x_i2c_write_with_readback(client, _bRW_MISCTL__CACHE_EN, buffer, 1);
    if (ret) {
        TPD_INFO("clear cache fail!\n");
        return ret;
    }
    // soft reset
    buffer[0] = 0x01;
    ret = touch_i2c_write_block(client, _bWO_MISCTL__CPU_SWRST_PULSE, 1, buffer);
    if (ret < 0) {
        TPD_INFO("software reset fail!\n");
        return ret;
    }
    // set scramble
    buffer[0] = 0x00;
    ret = gt1x_i2c_write_with_readback(client, _rRW_MISCTL__BOOT_OPT_B0_, buffer, 1);
    if (ret) {
        TPD_INFO("set scramble fail!\n");
        return ret;
    }

    return 0;
}

static int gt1x_run_ss51_isp(struct i2c_client* client, u8 * ss51_isp, int length)
{
    int ret;
    u8 buffer[10];

    ret = gt1x_hold_ss51_dsp(client);
    if (ret) {
        return ret;
    }
    // select bank4
    buffer[0] = 0x04;
    ret = gt1x_i2c_write_with_readback(client, _bRW_MISCTL__SRAM_BANK, buffer, 1);
    if (ret) {
        TPD_INFO("select bank4 fail.\n");
        return ret;
    }
    // enable patch area access
    buffer[0] = 0x01;
    ret = gt1x_i2c_write_with_readback(client, _bRW_MISCTL__PATCH_AREA_EN_, buffer, 1);
    if (ret) {
        TPD_INFO("enable patch area access fail!\n");
        return ret;
    }

    TPD_INFO("ss51_isp length: %d, checksum: 0x%04X\n", length, gt1x_calc_checksum(ss51_isp, length));
    // load ss51 isp
    ret = touch_i2c_write_block(client, 0xC000, length, ss51_isp);
    if (ret < 0) {
        TPD_INFO("load ss51 isp fail!\n");
        return ret;
    }
    // recall compare
    ret = gt1x_recall_check(client, ss51_isp, 0xC000, length);
    if (ret) {
        TPD_INFO("recall check ss51 isp fail!\n");
        return ret;
    }

    memset(buffer, 0xAA, 10);
    ret = gt1x_i2c_write_with_readback(client, 0x8140, buffer, 10);

    // disable patch area access
    buffer[0] = 0x00;
    ret = gt1x_i2c_write_with_readback(client, _bRW_MISCTL__PATCH_AREA_EN_, buffer, 1);
    if (ret) {
        TPD_INFO("disable patch area access fail!\n");
        return ret;
    }
    // set 0x8006
    memset(buffer, 0x55, 8);
    ret = gt1x_i2c_write_with_readback(client, 0x8006, buffer, 8);
    if (ret) {
        TPD_INFO("set 0x8006[0~7] 0x55 fail!\n");
        return ret;
    }
    // release ss51
    buffer[0] = 0x08;
    ret = gt1x_i2c_write_with_readback(client, _rRW_MISCTL__SWRST_B0_, buffer, 1);
    if (ret) {
        TPD_INFO("release ss51 fail!\n");
        return ret;
    }

    msleep(100);
    // check run state
    ret = touch_i2c_read_block(client, 0x8006, 2, buffer);
    if (ret < 0) {
        TPD_INFO("read 0x8006 fail!\n");
        return ret;
    }
    if (!(buffer[0] == 0xAA && buffer[1] == 0xBB)) {
        TPD_INFO("ERROR: isp is not running! 0x8006: %02X %02X\n", buffer[0], buffer[1]);
        return -1;
    }

    return 0;
}

static int gt1x_error_erase(struct chip_data_gt5688 *chip_info)
{
    int block_len;
    u16 checksum;
    u16 erase_addr;
    u8 buffer[10];
    int ret;
    int wait_time;
    int burn_state = -1;
    int retry = 5;
    u8 *fw = NULL;

    TPD_INFO("Erase flash area of ss51.\n");

    goodix_reset(chip_info);

    fw = gt1x_get_fw_data(chip_info, chip_info->update_info.firmware->subsystem[0].offset,
            chip_info->update_info.firmware->subsystem[0].length);
    if (fw == NULL) {
        TPD_INFO("get isp fail\n");
        return -1;
    }
    gt1x_select_addr(chip_info->client, chip_info->hw_res);
    ret = gt1x_run_ss51_isp(chip_info->client, fw, chip_info->update_info.firmware->subsystem[0].length);
    if (ret) {
        TPD_INFO("run isp fail\n");
        return -1;
    }

    fw = kmalloc(1024 * 4, GFP_KERNEL);
    if (!fw) {
        TPD_INFO("error when alloc mem.\n");
        return -1;
    }

    memset(fw, 0xFF, 1024 * 4);
    erase_addr = 0x00;
    block_len = 1024 * 4;

    while (retry-- > 0) {

        checksum = 0;
        checksum += block_len;
        checksum += erase_addr;
        checksum += gt1x_calc_checksum(fw, block_len);
        checksum = (0 - checksum);

        buffer[0] = ((block_len >> 8) & 0xFF);
        buffer[1] = (block_len & 0xFF);
        buffer[2] = ((erase_addr >> 8) & 0xFF);
        buffer[3] = (erase_addr & 0xFF);

        ret = gt1x_i2c_write_with_readback(chip_info->client, 0x8100, buffer, 4);
        if (ret) {
            TPD_INFO("write length & address fail!\n");
            continue;
        }

        ret = touch_i2c_write_block(chip_info->client, 0x8100 + 4, block_len, fw);
        if (ret < 0) {
            TPD_INFO("write fw data fail!\n");
            continue;
        }

        ret = gt1x_recall_check(chip_info->client, fw, 0x8100 + 4, block_len);
        if (ret) {
            continue;
        }

        buffer[0] = ((checksum >> 8) & 0xFF);
        buffer[1] = (checksum & 0xFF);
        ret = gt1x_i2c_write_with_readback(chip_info->client, 0x8100 + 4 + block_len, buffer, 2);
        if (ret) {
            TPD_INFO("write checksum fail!\n");
            continue;
        }

        buffer[0] = 0;
        ret = gt1x_i2c_write_with_readback(chip_info->client, 0x8022, buffer, 1);
        if (ret) {
            TPD_INFO("clear control flag fail!\n");
            continue;
        }

        buffer[0] = FW_SECTION_TYPE_SS51_PATCH;
        buffer[1] = FW_SECTION_TYPE_SS51_PATCH;
        ret = gt1x_i2c_write_with_readback(chip_info->client, 0x8020, buffer, 2);
        if (ret) {
            TPD_INFO("write subsystem type fail!\n");
            continue;
        }
        burn_state = -1;
        wait_time = 200;
        while (wait_time > 0) {
            wait_time--;
            msleep(5);
            ret = gt1x_i2c_read_dbl_check(chip_info->client, 0x8022, buffer, 1);
            if (ret) {
                continue;
            }

            if (buffer[0] == 0xAA) {
                TPD_DEBUG("burning.....\n");
                continue;
            } else if (buffer[0] == 0xDD) {
                TPD_INFO("checksum error!\n");
                break;
            } else if (buffer[0] == 0xBB) {
                TPD_INFO("burning success.\n");
                burn_state = 0;
                break;
            } else if (buffer[0] == 0xCC) {
                TPD_INFO("burning failed!\n");
                break;
            } else {
                TPD_DEBUG("unknown state!(0x8022: 0x%02X)\n", buffer[0]);
            }
        }
    }

    kfree(fw);
    if (burn_state == 0) {
        return 0;
    } else {
        return -1;
    }
}

static int gt1x_burn_subsystem(struct chip_data_gt5688 *chip_info, struct fw_subsystem_info *subsystem)
{
    int block_len;
    u16 checksum;
    int burn_len = 0;
    u16 cur_addr;
    u32 length = subsystem->length;
    u8 buffer[10];
    int ret;
    int wait_time;
    int burn_state;
    int retry = 5;
    u8 *fw;

    TPD_INFO("Subsystem: %d, Length: %d, Address: 0x%08X\n", subsystem->type, subsystem->length, subsystem->address);

    while (length > 0 && retry > 0) {
        retry--;

        block_len = length > 1024 * 4 ? 1024 * 4 : length;

        TPD_INFO("Burn block ==> length: %d, address: 0x%08X\n", block_len, subsystem->address + burn_len);
        fw = gt1x_get_fw_data(chip_info, subsystem->offset + burn_len, block_len);
        if (fw == NULL) {
            return -1;
        }

        cur_addr = ((subsystem->address + burn_len) >> 8);

        checksum = 0;
        checksum += block_len;
        checksum += cur_addr;
        checksum += gt1x_calc_checksum(fw, block_len);
        checksum = (0 - checksum);

        buffer[0] = ((block_len >> 8) & 0xFF);
        buffer[1] = (block_len & 0xFF);
        buffer[2] = ((cur_addr >> 8) & 0xFF);
        buffer[3] = (cur_addr & 0xFF);

        ret = gt1x_i2c_write_with_readback(chip_info->client, 0x8100, buffer, 4);
        if (ret) {
            TPD_INFO("write length & address fail!\n");
            continue;
        }

        ret = touch_i2c_write_block(chip_info->client, 0x8100 + 4, block_len, fw);
        if (ret < 0) {
            TPD_INFO("write fw data fail!\n");
            continue;
        }

        buffer[0] = ((checksum >> 8) & 0xFF);
        buffer[1] = (checksum & 0xFF);
        ret = gt1x_i2c_write_with_readback(chip_info->client, 0x8100 + 4 + block_len, buffer, 2);
        if (ret) {
            TPD_INFO("write checksum fail!\n");
            continue;
        }

        buffer[0] = 0;
        ret = gt1x_i2c_write_with_readback(chip_info->client, 0x8022, buffer, 1);
        if (ret) {
            TPD_INFO("clear control flag fail!\n");
            continue;
        }

        buffer[0] = subsystem->type;
        buffer[1] = subsystem->type;
        ret = gt1x_i2c_write_with_readback(chip_info->client, 0x8020, buffer, 2);
        if (ret) {
            TPD_INFO("write subsystem type fail!\n");
            continue;
        }
        burn_state = -1;
        wait_time = 200;
        msleep(5);

        while (wait_time-- > 0) {
            u8 confirm = 0x55;

            ret = touch_i2c_read_block(chip_info->client, 0x8022, 1, buffer);
            if (ret < 0) {
                continue;
            }
            msleep(5);
            ret = touch_i2c_read_block(chip_info->client, 0x8022, 1, &confirm);
            if (ret < 0) {
                continue;
            }
            if (buffer[0] != confirm) {
                continue;
            }

            if (buffer[0] == 0xAA) {
                TPD_DEBUG("burning.....\n");
                continue;
            } else if (buffer[0] == 0xDD) {
                TPD_INFO("checksum error!\n");
                break;
            } else if (buffer[0] == 0xBB) {
                TPD_INFO("burning success.\n");
                burn_state = 0;
                break;
            } else if (buffer[0] == 0xCC) {
                TPD_INFO("burning failed!\n");
                break;
            } else {
                TPD_DEBUG("unknown state!(0x8022: 0x%02X)\n", buffer[0]);
            }
        }

        if (!burn_state) {
            length -= block_len;
            burn_len += block_len;
            retry = 5;
        }
    }
    if (length == 0) {
        return 0;
    } else {
        return -1;
    }
}

static int gt1x_check_subsystem_in_flash(struct chip_data_gt5688 *chip_info, struct fw_subsystem_info *subsystem)
{
    int block_len;
    int checked_len = 0;
    u32 length = subsystem->length;
    int ret;
    int check_state = 0;
    int retry = 5;
    u8 *fw;

    TPD_INFO("Subsystem: %d, Length: %d, Address: 0x%08X", subsystem->type, subsystem->length, subsystem->address);

    while (length > 0) {
        block_len = length > 1024 * 4 ? 1024 * 4 : length;

        TPD_INFO("Check block ==> length: %d, address: 0x%08X\n", block_len, subsystem->address + checked_len);
        fw = gt1x_get_fw_data(chip_info, subsystem->offset + checked_len, block_len);
        if (fw == NULL) {
            return -1;
        }
        ret = gt1x_read_flash(chip_info->client, subsystem->address + checked_len, block_len);
        if (ret) {
            check_state |= ret;
        }

        ret = gt1x_recall_check(chip_info->client, fw, 0x8100, block_len);
        if (ret) {
            TPD_INFO("Block in flash is broken!\n");
            check_state |= ret;
        }

        length -= block_len;
        checked_len += block_len;
        retry = 5;
    }

    if (check_state) {
        TPD_INFO("Subsystem in flash is broken!\n");
    } else {
        TPD_INFO("Subsystem in flash is correct!\n");
    }
    return check_state;
}

static int gt1x_check_firmware_data(struct chip_data_gt5688 *chip_info)
{
    u16 checksum;
    u16 checksum_in_firmware_image;
    u8 *p;
    struct fw_info *firmware;
    int i;
    int offset;

    TPD_INFO("%s enter.\n", __func__);

    // compare file length with the length field in the firmware header
    if (chip_info->update_info.fw_length < FW_HEAD_SIZE) {
        TPD_INFO("Bad firmware!(file length: %d)\n", chip_info->update_info.fw_length);
        return -1;
    }

    p = gt1x_get_fw_data(chip_info, 0, 6);
    if (p == NULL) {
        return -1;
    }

    if (getU32(p) + 6 != chip_info->update_info.fw_length) {
        TPD_INFO("Bad firmware!(file length: %d, header define: %d)\n", chip_info->update_info.fw_length, getU32(p));
        return -1;
    }
    // check firmware's checksum
    checksum_in_firmware_image = getU16(&p[4]);
    checksum = 0;
    for (i = 6; i < chip_info->update_info.fw_length; i++) {
        p = gt1x_get_fw_data(chip_info, i, 1);
        if (p == NULL) {
            return -1;
        }
        checksum += p[0];
    }

    TPD_INFO("firmware checksum: 0x%04X, firmware image define: 0x%04X\n", checksum, checksum_in_firmware_image);

    if (checksum != checksum_in_firmware_image) {
        return -1;
    }

    // parse firmware
    p = gt1x_get_fw_data(chip_info, 0, FW_HEAD_SIZE);
    if (p == NULL) {
        return -1;
    }
    // get fireware
    memcpy((u8 *) chip_info->update_info.firmware, p, FW_HEAD_SIZE - 8 * 12);
    chip_info->update_info.firmware->pid[5] = 0;

    p = &p[FW_HEAD_OFFSET_SUBSYSTEM_INFO_BASE];
    firmware = chip_info->update_info.firmware;
    offset = FW_HEAD_SIZE;
    for (i = 0; i < firmware->subsystem_count; i++) {
        firmware->subsystem[i].type = p[i * FW_HEAD_SUBSYSTEM_INFO_SIZE];
        firmware->subsystem[i].length = getU16(&p[i * FW_HEAD_SUBSYSTEM_INFO_SIZE + 1]);
        firmware->subsystem[i].address = getU16(&p[i * FW_HEAD_SUBSYSTEM_INFO_SIZE + 3]) * 256;
        firmware->subsystem[i].offset = offset;
        offset += firmware->subsystem[i].length;
    }

    // print update information
    TPD_INFO("Firmware length: %d\n", chip_info->update_info.fw_length);
    TPD_INFO("Firmware product: GT%s\n", chip_info->update_info.firmware->pid);
    TPD_INFO("Firmware patch: %02X%02X%02X\n", chip_info->update_info.firmware->version[0], chip_info->update_info.firmware->version[1], chip_info->update_info.firmware->version[2]);
    TPD_INFO("Firmware chip: 0x%02X\n", chip_info->update_info.firmware->chip_type);
    TPD_INFO("Subsystem count: %d\n", chip_info->update_info.firmware->subsystem_count);
    for (i = 0; i < chip_info->update_info.firmware->subsystem_count; i++) {
        TPD_DEBUG("------------------------------------------\n");
        TPD_DEBUG("Subsystem: %d\n", i);
        TPD_DEBUG("Type: %d\n", chip_info->update_info.firmware->subsystem[i].type);
        TPD_DEBUG("Length: %d\n", chip_info->update_info.firmware->subsystem[i].length);
        TPD_DEBUG("Address: 0x%08X\n", chip_info->update_info.firmware->subsystem[i].address);
        TPD_DEBUG("Offset: %d\n", chip_info->update_info.firmware->subsystem[i].offset);
    }

    TPD_INFO("%s end.\n", __func__);

    return 0;
}

s32 gt1x_request_event_handler(struct chip_data_gt5688 *chip_info)
{
    s32 ret = -1;
    u8 rqst_data = 0;
    u8 noise_data = 0;
    ret = touch_i2c_read_block(chip_info->client, GTP_REG_RQST, 1, &rqst_data);
    if (ret < 0) {
        TPD_INFO("I2C transfer error. errno:%d\n", ret);
        return -1;
    }

    TPD_INFO("Request state:0x%02x.\n", rqst_data);

    switch (rqst_data & 0x0F) {
        case GTP_RQST_CONFIG:
            TPD_INFO("Request Config.\n");
            ret = gt1x_send_cfg(chip_info->client, chip_info->config_info.gt1x_config, chip_info->config_info.gt1x_cfg_length);
            if (ret) {
                TPD_INFO("Send gt1x_config error.\n");
            } else {
                TPD_INFO("Send gt1x_config success.\n");
                rqst_data = GTP_RQST_RESPONDED;
                ret = touch_i2c_write_block(chip_info->client, GTP_REG_RQST, 1, &rqst_data);
            }
            break;

        case GTP_RQST_RESET:
            TPD_INFO("Request Reset.\n");
            //  gt1x_reset_guitar();
            rqst_data = GTP_RQST_RESPONDED;
            ret = touch_i2c_write_block(chip_info->client, GTP_REG_RQST, 1, &rqst_data);
            break;

        case GTP_RQST_BAK_REF:
            TPD_INFO("Request Ref.\n");
            break;

        case GTP_RQST_MAIN_CLOCK:
            TPD_INFO("Request main clock.\n");
            break;

        case GTP_RQST_IDLE:
            TPD_INFO("Idle, invalid request.\n");
            return 0;
        default:
            break;
    }

    //noise
    ret = touch_i2c_read_block(chip_info->client, GTP_NOISE_DETECT_REG, 1, &noise_data);
    if (ret < 0) {
        TPD_INFO("I2C transfer error. errno:%d\n", ret);
        return -1;
    }

    TPD_INFO("randy:noise_data = 0x%x\n", noise_data);

    if ((noise_data & 0x20) == 0x20) {
        TPD_INFO("Send Noise config!\n");
        if ((gt1x_init_panel(chip_info, CFG_NOISE)) != 0) {
            TPD_INFO("Send Noise config failed.\n");
        }
    } else if ((noise_data & 0x40) == 0x40) {
        TPD_DEBUG("Send normal config!\n");
        if ((gt1x_init_panel(chip_info, CFG_NORMAL)) != 0) {
            TPD_INFO("Send normal config failed.\n");
        }
    }

    return 0;
}

static void goodix_finger_protect_data_get(struct chip_data_gt5688 *chip_info)
{
    int ret = -1;
    int addr, i = 0, j = 0;
    u8 raw_data[1024] = {0};
    int TX_NUM = chip_info->hw_res->TX_NUM;
    int RX_NUM = chip_info->hw_res->RX_NUM;

    chip_info->spuri_fp_touch_raw_data = kzalloc(TX_NUM * RX_NUM * sizeof(u16), GFP_KERNEL);
    addr = GTP_REG_RAWDATA;

    TPD_INFO("%s start\n", __func__);
    gt1x_rawdiff_mode = 1;
    gt1x_send_cmd(chip_info->client, 1, 0);
    touch_i2c_write_byte(chip_info->client, GTP_READ_COOR_ADDR, 0);

    //wait for data ready
    while(i++ < 10) {
        ret = touch_i2c_read_block(chip_info->client, GTP_READ_COOR_ADDR, 1, raw_data);
        TPD_INFO("ret = %d \t kernel_buf = %d\n", ret, raw_data[0]);
        if(ret && ((raw_data[0] & 0x80) == 0x80)) {
            TPD_INFO("Data ready OK");
            break;
        }
        msleep(5);
    }
    if(i >= 10) {
        TPD_INFO("data not ready, quit!\n");
        goto read_data_exit;
    }

    ret = touch_i2c_read_block(chip_info->client, addr, TX_NUM * RX_NUM * 2, raw_data);

    for(i = 0; i < RX_NUM; i++) {
        //TPD_INFO("[%2d] ", i);
        for(j = 0; j < TX_NUM; j++) {
            chip_info->spuri_fp_touch_raw_data[i * TX_NUM + j] = (s16)(raw_data[i * TX_NUM * 2 + j * 2] << 8) + raw_data[i * TX_NUM * 2 + j * 2 + 1];
            //printk("%d ",chip_info->spuri_fp_touch_raw_data[i * TX_NUM + j]);
        }
        //printk("\n");
    }

read_data_exit:
    gt1x_send_cmd(chip_info->client, 0, 0);
    gt1x_rawdiff_mode = 0;
    touch_i2c_write_byte(chip_info->client, GTP_READ_COOR_ADDR, 0);
    return;
}


/*
 * return success: 0 ; fail : negative
 */
static int goodix_get_chip_info(void *chip_data)
{
    int ret = 0;
    struct chip_data_gt5688 *chip_info = (struct chip_data_gt5688 *)chip_data;

    /* read version information */
    ret = gt1x_read_version(chip_info);
    if (ret != 0) {
        TPD_INFO("Get verision failed!\n");
        return ret;
    }

    /* send config data */
    ret = gt1x_init_panel(chip_info, CFG_NORMAL);
    if (ret != 0) {
        TPD_INFO("Init panel failed.\n");
        return ret;
    }

    return ret;
}

static int goodix_power_control(void *chip_data, bool enable)
{
    int ret = 0;
    struct chip_data_gt5688 *chip_info = (struct chip_data_gt5688 *)chip_data;

    if (true == enable) {
        ret = tp_powercontrol_2v8(chip_info->hw_res, true);
        if (ret)
            return -1;

        udelay(2);

        ret = tp_powercontrol_1v8(chip_info->hw_res, true);
        if (ret)
            return -1;

        /*if (gpio_is_valid(chip_info->hw_res->reset_gpio)) {
            TPD_INFO("Set the reset_gpio.\n");
            gpio_direction_output(chip_info->hw_res->reset_gpio, 1);
        }*/

        mdelay(20);

        goodix_reset(chip_data);
    } else {
        ret = tp_powercontrol_1v8(chip_info->hw_res, false);
        if (ret)
            return -1;

        udelay(2);

        ret = tp_powercontrol_2v8(chip_info->hw_res, false);
        if (ret)
            return -1;
    }

    return 0;
}


static int goodix_ftm_process(void *chip_data)
{
    struct chip_data_gt5688 *chip_info = (struct chip_data_gt5688 *)chip_data;

    TPD_INFO("FTM regulator_disable is called\n");
    tp_powercontrol_2v8(chip_info->hw_res, false);

    return 0;
}

static int goodix_get_vendor(void *chip_data, struct panel_info *panel_data)
{
    struct chip_data_gt5688 *chip_info = (struct chip_data_gt5688 *)chip_data;

    chip_info->tp_type = panel_data->tp_type;
    chip_info->config_info.config_version = panel_data->manufacture_info.version;

    return 0;
}

static fw_check_state goodix_fw_check(void *chip_data, struct resolution_info *resolution_info, struct panel_info *panel_data)
{
    int ret = 0;
    int retry = 2;
    u8 reg_val[2] = {0};
    char fw_id[4] = {0};

    struct chip_data_gt5688 *chip_info = (struct chip_data_gt5688 *)chip_data;

    do {
        gt1x_i2c_read_dbl_check(chip_info->client, GTP_REG_FW_CHK_MAINSYS, reg_val, 1);
        gt1x_i2c_read_dbl_check(chip_info->client, GTP_REG_FW_CHK_SUBSYS, &reg_val[1], 1);

        if (reg_val[0] != 0xBE || reg_val[1] == 0xAA) {
            TPD_INFO("Check fw status reg not pass, reg[0x814E] = 0x%2X, reg[0x5095] = 0x%2X!, retry = %d\n", reg_val[0], reg_val[1], retry);
        } else {
            break;
        }
    } while(--retry);

    if (!retry)
        return FW_ABNORMAL;

    ret = touch_i2c_read_byte(chip_info->client, GTP_REG_CONFIG_DATA);

    snprintf(fw_id, 4, "%02x", ret);
    if (panel_data->manufacture_info.version)
        strlcpy(&panel_data->manufacture_info.version[8], fw_id, MAX_DEVICE_VERSION_LENGTH - 8);

    TPD_INFO("%s: TP_FW = 0x%x \n", __func__, panel_data->TP_FW);

    return FW_NORMAL;
}

/*
 * Return 0 : updata firmware success; < 0 : updata failed; > 0 : no need to update
 */
static fw_update_state goodix_fw_update(void *chip_data, const struct firmware *fw, bool force)
{
    int i = 0;
    int ret = 0;
    int retry = 2;
    u8 *p;
    u8 reg_val[2] = {0};
    struct gt1x_version_info fw_ver_info;
    struct chip_data_gt5688 *chip_info = (struct chip_data_gt5688 *)chip_data;
    struct gt1x_version_info ver_info = chip_info->ver_info;

    TPD_INFO("%s enter!\n", __func__);

    if (chip_info->update_info.status != UPDATE_STATUS_IDLE) {
        TPD_INFO("Update process is running!\n");
        return FW_NO_NEED_UPDATE;
    }
    chip_info->update_info.status = UPDATE_STATUS_RUNNING;
    chip_info->update_info.progress = 0;
    chip_info->update_info.max_progress = 0;

    chip_info->update_info.fw_length = fw->size;
    chip_info->update_info.firmware_file_data = (u8 *)fw->data;

    chip_info->update_info.firmware = (struct fw_info *)kzalloc(sizeof(struct fw_info), GFP_KERNEL);
    if (chip_info->update_info.firmware == NULL) {
        TPD_INFO("Alloc %zu bytes memory fail.\n", sizeof(struct fw_info));

        return FW_NO_NEED_UPDATE;
    }

    chip_info->update_info.buffer = (u8 *) kzalloc(1024 * 4, GFP_KERNEL);
    if (chip_info->update_info.buffer == NULL) {
        TPD_INFO("Alloc %d bytes memory fail.\n", 1024 * 4);
        kfree(chip_info->update_info.firmware);
        return FW_NO_NEED_UPDATE;
    }

    ret = gt1x_check_firmware_data(chip_info);
    if (ret < 0) {
        chip_info->update_info.status = UPDATE_STATUS_ABORT;
         TPD_INFO("gt1x_check_firmware_data fail\n");
        goto gt1x_no_update_exit;
    }

    chip_info->update_info.max_progress = 6 + chip_info->update_info.firmware->subsystem_count;
    chip_info->update_info.progress++;

    do {
        gt1x_i2c_read_dbl_check(chip_info->client, GTP_REG_FW_CHK_MAINSYS, reg_val, 1);
        gt1x_i2c_read_dbl_check(chip_info->client, GTP_REG_FW_CHK_SUBSYS, &reg_val[1], 1);

        if (reg_val[0] != 0xBE || reg_val[1] == 0xAA) {
            TPD_INFO("Check fw status reg not pass, reg[0x814E] = 0x%2X, reg[0x5095] = 0x%2X!, retry = %d\n", reg_val[0], reg_val[1], retry);
        } else {
            break;
        }
    } while(--retry);

    if (retry) { // if tp firmware is ok, judge the fw id, else  update immediately
        fw_ver_info.mask_id = (chip_info->update_info.firmware->target_mask_version[0] << 16)
            | (chip_info->update_info.firmware->target_mask_version[1] << 8)
            | (chip_info->update_info.firmware->target_mask_version[2]);
        fw_ver_info.patch_id = (chip_info->update_info.firmware->version[0] << 16)
            | (chip_info->update_info.firmware->version[1] << 8)
            | (chip_info->update_info.firmware->version[2]);
        memcpy(fw_ver_info.product_id, chip_info->update_info.firmware->pid, 4);
        fw_ver_info.product_id[4] = 0;

        if (memcmp(fw_ver_info.product_id, ver_info.product_id, 4)) {
            TPD_INFO("Product id is not match!\n");
            goto gt1x_no_update_exit;
        }

        if ((fw_ver_info.mask_id & 0xFFFFFF00) != (ver_info.mask_id & 0xFFFFFF00)) {
            TPD_INFO("Mask id is not match!\n");
            goto gt1x_no_update_exit;
        }

        if (!force) {
            if (fw_ver_info.patch_id  == ver_info.patch_id)
                goto gt1x_no_update_exit;
        }
    }

    chip_info->update_info.progress++;

    p = gt1x_get_fw_data(chip_info, chip_info->update_info.firmware->subsystem[0].offset, chip_info->update_info.firmware->subsystem[0].length);
    if (p == NULL) {
        TPD_INFO("get isp fail\n");
        chip_info->update_info.status = UPDATE_STATUS_ABORT;
        goto gt1x_update_exit;
    }

    chip_info->update_info.progress++;
    gt1x_select_addr(chip_info->client, chip_info->hw_res);
    ret = gt1x_run_ss51_isp(chip_info->client, p, chip_info->update_info.firmware->subsystem[0].length);
    if (ret) {
        TPD_INFO("run isp fail\n");
        goto gt1x_update_exit;
    }

    chip_info->update_info.progress++;

    msleep(800);

    for (i = 1; i < chip_info->update_info.firmware->subsystem_count; i++) {
        TPD_INFO("subsystem: %d\n", chip_info->update_info.firmware->subsystem[i].type);
        TPD_INFO("Length: %d\n", chip_info->update_info.firmware->subsystem[i].length);
        TPD_INFO("Address: %d\n", chip_info->update_info.firmware->subsystem[i].address);

        ret = gt1x_burn_subsystem(chip_info, &(chip_info->update_info.firmware->subsystem[i]));
        if (ret) {
            TPD_INFO("burn subsystem fail!\n");
            goto gt1x_update_exit;
        }
        chip_info->update_info.progress++;
    }

    goodix_reset(chip_data);

    p = gt1x_get_fw_data(chip_info, chip_info->update_info.firmware->subsystem[0].offset, chip_info->update_info.firmware->subsystem[0].length);
    if (p == NULL) {
        TPD_INFO("get isp fail\n");
        goto gt1x_update_exit;
    }
    chip_info->update_info.progress++;

    gt1x_select_addr(chip_info->client, chip_info->hw_res);
    ret = gt1x_run_ss51_isp(chip_info->client, p, chip_info->update_info.firmware->subsystem[0].length);
    if (ret) {
        TPD_INFO("run isp fail\n");
        goto gt1x_update_exit;
    }
    chip_info->update_info.progress++;

    TPD_INFO("Reset guitar & check firmware in flash.\n");
    for (i = 1; i < chip_info->update_info.firmware->subsystem_count; i++) {
        TPD_INFO("subsystem: %d\n", chip_info->update_info.firmware->subsystem[i].type);
        TPD_INFO("Length: %d\n", chip_info->update_info.firmware->subsystem[i].length);
        TPD_INFO("Address: %d\n", chip_info->update_info.firmware->subsystem[i].address);

        ret = gt1x_check_subsystem_in_flash(chip_info, &(chip_info->update_info.firmware->subsystem[i]));
        if (ret) {
            gt1x_error_erase(chip_info);
            break;
        }
    }
    chip_info->update_info.progress++;

    TPD_INFO("%s end.\n", __func__);

gt1x_update_exit:
    kfree(chip_info->update_info.buffer);
    kfree(chip_info->update_info.firmware);
    chip_info->update_info.status = UPDATE_STATUS_IDLE;

    if (ret) {
        chip_info->update_info.progress = 2 * chip_info->update_info.max_progress;
        TPD_INFO("Update firmware failed!\n");
        return FW_UPDATE_ERROR;
    } else {
        if (!retry){  // when firmware check fail , it need to init panel again.
            /* send config data */
            gt1x_init_panel(chip_info, CFG_NORMAL);
        }
        TPD_INFO("Update firmware succeefully!\n");
        return FW_UPDATE_SUCCESS;
    }

gt1x_no_update_exit:
    kfree(chip_info->update_info.buffer);
    kfree(chip_info->update_info.firmware);
    chip_info->update_info.status = UPDATE_STATUS_IDLE;
    return FW_NO_NEED_UPDATE;
}

static int goodix_clear_irq(struct i2c_client*client)
{
    int ret = -1;

    if (!gt1x_rawdiff_mode) {
        ret = touch_i2c_write_byte(client, GTP_READ_COOR_ADDR, 0);
        if (ret < 0) {
            TPD_INFO("I2C write end_cmd  error!\n");
        }
    }

    return ret;
}

static u8 goodix_trigger_reason(void *chip_data, int gesture_enable, int is_suspended)
{
    int ret = 0;
    static u8 trigger_event = 0;
    u8 cur_event = 0;
    u8 result_event = 0;
    u8 touch_num = 0;
    u8 check_sum = 0;
    int i;
    struct chip_data_gt5688 *chip_info = (struct chip_data_gt5688 *)chip_data;

    //step1:detect whether gesture trigger irq
    if (gesture_enable && is_suspended)
        return IRQ_GESTURE;
    else if ((!gesture_enable) && is_suspended)
        return IRQ_IGNORE;
    //step2:Read irq status reg detect trigger reason of IRQ_EXCEPTION/IRQ_FW_CONFIG/IRQ_IGNORE
    memset(chip_info->touch_data, 0, MAX_GT_IRQ_DATA_LENGTH);
    ret = touch_i2c_read_byte(chip_info->client, GTP_READ_COOR_ADDR);
    if (ret < 0) {
        TPD_INFO("I2C transfer error!\n");
        trigger_event = 0;

        return IRQ_EXCEPTION;
    }
    TPD_DEBUG("0x814e is 0x%x\n", ret);

    chip_info->touch_data[0] = ret & 0xff;
    touch_num = chip_info->touch_data[0] & 0x0F;

    if (chip_info->touch_data[0] == 0x00) {
        return IRQ_FW_CONFIG;
    }
    if ((chip_info->touch_data[0] & 0x80) == 0) {
        goto IGNORE_CLEAR_IRQ;
    }

    //step3:Read COOR && do checksum

    /* read the remaining coor data
     * 0x814E(touch status) + 8(every coordinate consist of 8 bytes data) * touch num +
     * keycode + checksum
     */
    ret = touch_i2c_read_block(chip_info->client, (GTP_READ_COOR_ADDR + 1), 8 * touch_num + 2, &chip_info->touch_data[1]);
    if (ret < 0) {
        TPD_INFO("read coor_addr failed!\n");
        return IRQ_IGNORE;
    }
    /* cacl checksum */
    for (i = 0; i < 1 + 8 * touch_num + 2; i++) {
        check_sum += chip_info->touch_data[i];
    }
    if (check_sum) { /* checksum error*/
        ret = touch_i2c_read_block(chip_info->client, GTP_READ_COOR_ADDR, 3 + 8 * touch_num, chip_info->touch_data);
        if (ret < 0) {
            return IRQ_IGNORE;
        }

        for (i = 0, check_sum = 0; i < 3 + 8 * touch_num; i++) {
            check_sum += chip_info->touch_data[i];
        }
        if (check_sum) {
            TPD_INFO("Checksum error[%x]\n", check_sum);
            goto IGNORE_CLEAR_IRQ;
        }
    }

    //step4:Touch /Key Trigger Bit setting
    touch_num = chip_info->touch_data[0] & 0x0F;
    chip_info->vk_status = chip_info->touch_data[8 * touch_num + 1];

    if (touch_num && (!(chip_info->touch_data[0] & 0x10))) {
        SET_BIT(cur_event, IRQ_TOUCH);
    } else if ((chip_info->touch_data[0] & 0x10) && (chip_info->vk_status & 0x0F)) {
        SET_BIT(cur_event, IRQ_BTN_KEY);
    }

    TPD_DEBUG("cur_event is %d\t trigger_event = %d\n", cur_event, trigger_event);

    if (CHK_BIT(cur_event, IRQ_TOUCH) || CHK_BIT(trigger_event, IRQ_TOUCH)) {
        SET_BIT(result_event, IRQ_TOUCH);
    }

    if ((chip_info->touch_data[0] & 0x10) || CHK_BIT(trigger_event, IRQ_BTN_KEY)) {
        SET_BIT(result_event, IRQ_BTN_KEY);
    }

    if (result_event == IRQ_IGNORE) {
        goto IGNORE_CLEAR_IRQ;
    }

    trigger_event = cur_event;
    TPD_DEBUG("point data is 0x%2x %hhu, vk_status:%hhu\n", ret, result_event, chip_info->vk_status);

    return result_event;

IGNORE_CLEAR_IRQ:
    ret = goodix_clear_irq(chip_info->client);
    return IRQ_IGNORE;
}

static int goodix_get_touch_points(void *chip_data, struct point_info *points, int max_num)
{
    int ret, i;
    int touch_map = 0;
    struct chip_data_gt5688 *chip_info = (struct chip_data_gt5688 *)chip_data;
    u8 touch_num = 0;
    u8 finger_processed = 0;
    uint16_t input_x = 0;
    uint16_t input_y = 0;
    uint16_t input_w = 0;
    u8 *coor_data = NULL;
    s32 id = 0;

    touch_num = chip_info->touch_data[0] & 0x0F;
    if (touch_num > max_num) {
        TPD_INFO("Illegal finger number!");
        goto END_TOUCH;
    }
    if (touch_num == 0) { //Up event
        goto END_TOUCH;
    }

    coor_data = &chip_info->touch_data[1];
    id = coor_data[0] & 0x0F;
    for (i = 0; i < max_num; i++) {
        if (i == id) {
            input_x = coor_data[1] | (coor_data[2] << 8);
            input_y = coor_data[3] | (coor_data[4] << 8);
            input_w = coor_data[5] | (coor_data[6] << 8);
            points[i].x = input_x;
            points[i].y = input_y;
            points[i].z = input_w;
            points[i].width_major = 30; // any value
            points[i].status = 1;
            touch_map |= 0x01 << i;
            coor_data += 8;

            if (finger_processed++ < touch_num) {
                id = coor_data[0] & 0x0F;
            }
        }
    }
END_TOUCH:
    ret = goodix_clear_irq(chip_info->client);
    return touch_map;
}

static u8 goodix_get_keycode(void *chip_data)
{
    int ret = 0;
    u8 bitmap = 0;
    u8 bitmap_result = 0;
    struct chip_data_gt5688 *chip_info = (struct chip_data_gt5688 *)chip_data;

    bitmap = chip_info->vk_status & 0x0F;
    if (bitmap & 0x01)
        SET_BIT(bitmap_result, BIT_MENU);
    if (bitmap & 0x02)
        SET_BIT(bitmap_result, BIT_HOME);
    if (bitmap & 0x04)
        SET_BIT(bitmap_result, BIT_BACK);
    ret = goodix_clear_irq(chip_info->client);

    return bitmap_result;
}

static int goodix_fw_handle(void *chip_data)
{
    int ret = 0;
    struct chip_data_gt5688 *chip_info = (struct chip_data_gt5688 *)chip_data;

    ret = gt1x_request_event_handler(chip_info);
    return ret;
}

static int goodix_get_gesture_info(void *chip_data, struct gesture_info * gesture)
{
    u8 doze_buf[3] = {0};
    u8 coordinate_single[MAX_GESTURE_COORD*4];
    u8 coordinate_size = 0;
    int i = 0, j = 0, ret = 0;
    struct Coordinate Point_output[6];
    struct Coordinate Point_input[6];
    struct chip_data_gt5688 *chip_info = (struct chip_data_gt5688 *)chip_data;
    struct i2c_client *client = chip_info->client;

    ret = touch_i2c_read_block(client, GTP_REG_WAKEUP_GESTURE, 2, doze_buf);
    TPD_DEBUG("0x814C = 0x%02X, ret = %d\n", doze_buf[0], ret);
    if (ret < 0) {
        TPD_INFO("Read Gesture info i2c faild\n");
        return -1;
    }

    if (doze_buf[0] != 0) {
        memset(coordinate_single, 0, MAX_GESTURE_COORD*4);
        memset(Point_output, 0, sizeof(Point_output));
        memset(Point_input, 0, sizeof(Point_input));

        coordinate_size = doze_buf[1];
        if (coordinate_size > MAX_GESTURE_COORD)
            coordinate_size = MAX_GESTURE_COORD;

        ret = touch_i2c_read_block(client, GTP_REG_WAKEUP_GESTURE_DETAIL, coordinate_size*4, coordinate_single);
        touch_i2c_write_byte(client, GTP_REG_WAKEUP_GESTURE, 0x00);
        TPD_INFO("clear irq\n");
        switch (doze_buf[0]) {
            case DTAP_DETECT:
                gesture->gesture_type  = DouTap;
                gesture->Point_start.x = coordinate_single[0]  | (coordinate_single[1] << 8);
                gesture->Point_start.y = coordinate_single[2]  | (coordinate_single[3] << 8);
                gesture->Point_end.x   = coordinate_single[8]  | (coordinate_single[9] << 8);
                gesture->Point_end.y   = coordinate_single[10] | (coordinate_single[11] << 8);
                gesture->Point_1st.x   = 0;
                gesture->Point_1st.y   = 0;
                gesture->clockwise = 0 ;
                break;

            case UP_VEE_DETECT :
                gesture->gesture_type  = UpVee;
                gesture->Point_start.x = coordinate_single[0]  | (coordinate_single[1]  << 8);
                gesture->Point_start.y = coordinate_single[2]  | (coordinate_single[3]  << 8);
                gesture->Point_end.x   = coordinate_single[8]  | (coordinate_single[9]  << 8);
                gesture->Point_end.y   = coordinate_single[10] | (coordinate_single[11] << 8);
                gesture->Point_1st.x   = coordinate_single[4]  | (coordinate_single[5]  << 8);
                gesture->Point_1st.y   = coordinate_single[6]  | (coordinate_single[7]  << 8);
                break;

            case DOWN_VEE_DETECT :
                gesture->gesture_type  = DownVee;
                gesture->Point_start.x = coordinate_single[0]  | (coordinate_single[1]  << 8);
                gesture->Point_start.y = coordinate_single[2]  | (coordinate_single[3]  << 8);
                gesture->Point_end.x   = coordinate_single[8]  | (coordinate_single[9]  << 8);
                gesture->Point_end.y   = coordinate_single[10] | (coordinate_single[11] << 8);
                gesture->Point_1st.x   = coordinate_single[4]  | (coordinate_single[5]  << 8);
                gesture->Point_1st.y   = coordinate_single[6]  | (coordinate_single[7]  << 8);
                break;

            case LEFT_VEE_DETECT:
                gesture->gesture_type = LeftVee;
                gesture->Point_start.x = coordinate_single[0]  | (coordinate_single[1]  << 8);
                gesture->Point_start.y = coordinate_single[2]  | (coordinate_single[3]  << 8);
                gesture->Point_end.x   = coordinate_single[8]  | (coordinate_single[9]  << 8);
                gesture->Point_end.y   = coordinate_single[10] | (coordinate_single[11] << 8);
                gesture->Point_1st.x   = coordinate_single[4]  | (coordinate_single[5]  << 8);
                gesture->Point_1st.y   = coordinate_single[6]  | (coordinate_single[7]  << 8);
                break;

            case RIGHT_VEE_DETECT :
                gesture->gesture_type  = RightVee;
                gesture->Point_start.x = coordinate_single[0]  | (coordinate_single[1]  << 8);
                gesture->Point_start.y = coordinate_single[2]  | (coordinate_single[3]  << 8);
                gesture->Point_end.x   = coordinate_single[16] | (coordinate_single[17] << 8);
                gesture->Point_end.y   = coordinate_single[18] | (coordinate_single[19] << 8);
                gesture->Point_1st.x   = coordinate_single[8]  | (coordinate_single[9]  << 8);
                gesture->Point_1st.y   = coordinate_single[10] | (coordinate_single[11] << 8);
                break;

            case CIRCLE_DETECT  :
                gesture->gesture_type = Circle;
                j = 0;
                for (i = 0; i < coordinate_size;i++) {
                    Point_input[i].x = coordinate_single[j]   | (coordinate_single[j + 1] << 8);
                    Point_input[i].y = coordinate_single[j + 2] | (coordinate_single[j + 3] << 8);
                    j = j + 4;
                    //TPD_INFO("Point_input[%d].x = %d, Point_input[%d].y = %d\n", i, Point_input[i].x, i, Point_input[i].y);
                }

                gesture->clockwise = ClockWise(&Point_input[0], coordinate_size - 2);
                GetCirclePoints(&Point_input[0], coordinate_size, Point_output);
                gesture->Point_start.x = Point_input[0].x;
                gesture->Point_start.y = Point_input[0].y;
                gesture->Point_end.x   = Point_input[coordinate_size-1].x;
                gesture->Point_end.y   = Point_input[coordinate_size-1].y;
                gesture->Point_1st.x   = Point_output[0].x;
                gesture->Point_1st.y   = Point_output[0].y;
                gesture->Point_2nd.x   = Point_output[1].x;
                gesture->Point_2nd.y   = Point_output[1].y;
                gesture->Point_3rd.x   = Point_output[2].x;
                gesture->Point_3rd.y   = Point_output[2].y;
                gesture->Point_4th.x   = Point_output[3].x;
                gesture->Point_4th.y   = Point_output[3].y;
                break;

            case DOUSWIP_DETECT  :
                gesture->gesture_type  = DouSwip;
                gesture->Point_start.x = coordinate_single[0]  | (coordinate_single[1]  << 8);
                gesture->Point_start.y = coordinate_single[2]  | (coordinate_single[3]  << 8);
                gesture->Point_end.x   = coordinate_single[4]  | (coordinate_single[5]  << 8);
                gesture->Point_end.y   = coordinate_single[6]  | (coordinate_single[7]  << 8);
                gesture->Point_1st.x   = coordinate_single[8]  | (coordinate_single[9]  << 8);
                gesture->Point_1st.y   = coordinate_single[10] | (coordinate_single[11] << 8);
                gesture->Point_2nd.x   = coordinate_single[12] | (coordinate_single[13] << 8);
                gesture->Point_2nd.y   = coordinate_single[14] | (coordinate_single[15] << 8);
                break;

            case DOUUPSWIP_DETECT:
                gesture->gesture_type  = DouSwip;
                gesture->Point_start.x = coordinate_single[0]  | (coordinate_single[1]  << 8);
                gesture->Point_start.y = coordinate_single[2]  | (coordinate_single[3]  << 8);
                gesture->Point_end.x   = coordinate_single[4]  | (coordinate_single[5]  << 8);
                gesture->Point_end.y   = coordinate_single[6]  | (coordinate_single[7]  << 8);
                gesture->Point_1st.x   = coordinate_single[8]  | (coordinate_single[9]  << 8);
                gesture->Point_1st.y   = coordinate_single[10] | (coordinate_single[11] << 8);
                gesture->Point_2nd.x   = coordinate_single[12] | (coordinate_single[13] << 8);
                gesture->Point_2nd.y   = coordinate_single[14] | (coordinate_single[15] << 8);
                break;

            case RIGHT_SLIDE_DETECT :
                gesture->gesture_type  = Left2RightSwip;
                gesture->Point_start.x = coordinate_single[0] | (coordinate_single[1] << 8);
                gesture->Point_start.y = coordinate_single[2] | (coordinate_single[3] << 8);
                gesture->Point_end.x   = coordinate_single[4] | (coordinate_single[5] << 8);
                gesture->Point_end.y   = coordinate_single[6] | (coordinate_single[7] << 8);
                break;

            case LEFT_SLIDE_DETECT :
                gesture->gesture_type  = Right2LeftSwip;
                gesture->Point_start.x = coordinate_single[0] | (coordinate_single[1] << 8);
                gesture->Point_start.y = coordinate_single[2] | (coordinate_single[3] << 8);
                gesture->Point_end.x   = coordinate_single[4] | (coordinate_single[5] << 8);
                gesture->Point_end.y   = coordinate_single[6] | (coordinate_single[7] << 8);
                break;

            case DOWN_SLIDE_DETECT  :
                gesture->gesture_type  = Up2DownSwip;
                gesture->Point_start.x = coordinate_single[0] | (coordinate_single[1] << 8);
                gesture->Point_start.y = coordinate_single[2] | (coordinate_single[3] << 8);
                gesture->Point_end.x   = coordinate_single[4] | (coordinate_single[5] << 8);
                gesture->Point_end.y   = coordinate_single[6] | (coordinate_single[7] << 8);
                break;

            case UP_SLIDE_DETECT :
                gesture->gesture_type  = Down2UpSwip;
                gesture->Point_start.x = coordinate_single[0] | (coordinate_single[1] << 8);
                gesture->Point_start.y = coordinate_single[2] | (coordinate_single[3] << 8);
                gesture->Point_end.x   = coordinate_single[4] | (coordinate_single[5] << 8);
                gesture->Point_end.y   = coordinate_single[6] | (coordinate_single[7] << 8);
                break;

            case M_DETECT  :
                gesture->gesture_type  = Mgestrue;
                gesture->Point_start.x = coordinate_single[0]  | (coordinate_single[1]  << 8);
                gesture->Point_start.y = coordinate_single[2]  | (coordinate_single[3]  << 8);
                gesture->Point_end.x   = coordinate_single[16] | (coordinate_single[17] << 8);
                gesture->Point_end.y   = coordinate_single[18] | (coordinate_single[19] << 8);
                gesture->Point_1st.x   = coordinate_single[4]  | (coordinate_single[5]  << 8);
                gesture->Point_1st.y   = coordinate_single[6]  | (coordinate_single[7]  << 8);
                gesture->Point_2nd.x   = coordinate_single[8]  | (coordinate_single[9]  << 8);
                gesture->Point_2nd.y   = coordinate_single[10] | (coordinate_single[11] << 8);
                gesture->Point_3rd.x   = coordinate_single[12] | (coordinate_single[13] << 8);
                gesture->Point_3rd.y   = coordinate_single[14] | (coordinate_single[15] << 8);
                break;

            case W_DETECT :
                gesture->gesture_type  = Wgestrue;
                gesture->Point_start.x = coordinate_single[0]  | (coordinate_single[1]  << 8);
                gesture->Point_start.y = coordinate_single[2]  | (coordinate_single[3]  << 8);
                gesture->Point_end.x   = coordinate_single[16] | (coordinate_single[17] << 8);
                gesture->Point_end.y   = coordinate_single[18] | (coordinate_single[19] << 8);
                gesture->Point_1st.x   = coordinate_single[4]  | (coordinate_single[5]  << 8);
                gesture->Point_1st.y   = coordinate_single[6]  | (coordinate_single[7]  << 8);
                gesture->Point_2nd.x   = coordinate_single[8]  | (coordinate_single[9]  << 8);
                gesture->Point_2nd.y   = coordinate_single[10] | (coordinate_single[11] << 8);
                gesture->Point_3rd.x   = coordinate_single[12] | (coordinate_single[13] << 8);
                gesture->Point_3rd.y   = coordinate_single[14] | (coordinate_single[15] << 8);
                break;

            default:
                gesture->gesture_type = UnkownGesture;
                break;
        }

        TPD_INFO("%s, gesture_sign = 0x%x, gesture_type = %d | %d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d\n",
            __func__, doze_buf[0], gesture->gesture_type,
            gesture->Point_start.x, gesture->Point_start.y, gesture->Point_end.x, gesture->Point_end.y,
            gesture->Point_1st.x, gesture->Point_1st.y, gesture->Point_2nd.x, gesture->Point_2nd.y,
            gesture->Point_3rd.x, gesture->Point_3rd.y, gesture->Point_4th.x, gesture->Point_4th.y,
            gesture->clockwise);
    }
    ret = goodix_clear_irq(chip_info->client);      //add 9-17
    return 0;
}

static int goodix_enable_edge_limit(struct chip_data_gt5688 *chip_info, bool enable)
{
    int ret = -1;

    TPD_INFO("%s, edge limit enable = %d\n", __func__, enable);

    if (enable) {
        ret = gt1x_send_cmd(chip_info->client, GOODIX_ENABLE_EDGE, 0);
    } else  {
        ret = gt1x_send_cmd(chip_info->client, GOODIX_DISABLE_EDGE, 0);
    }

    return ret;
}

static int goodix_set_charger_state(struct chip_data_gt5688 *chip_info, bool enable)
{
    int ret = -1;

    if (enable) {
        ret = gt1x_send_cmd(chip_info->client, GTP_CMD_CHARGER_ON, 0);
    } else {
        ret = gt1x_send_cmd(chip_info->client, GTP_CMD_CHARGER_OFF, 0);
    }

    return ret;
}

static int goodix_mode_switch(void *chip_data, work_mode mode, bool flag)
{
    int ret = -1;
    struct chip_data_gt5688 *chip_info = (struct chip_data_gt5688 *)chip_data;

    if (chip_info->halt_status && (mode != MODE_NORMAL)) {
        goodix_reset(chip_info);
    }

    switch(mode) {
        case MODE_NORMAL:
            //After reset no need to control the Touch IC
            ret = 0;
            break;

        case MODE_SLEEP:
            ret = gt1x_enter_sleep(chip_info);
            chip_info->halt_status = true;
            break;

        case MODE_GESTURE:
            if (flag) {
                ret = gesture_enter_doze(chip_info->client);
                chip_info->halt_status = true;
            }
            break;

        case MODE_GLOVE:
            break;

        case MODE_EDGE:
            ret = goodix_enable_edge_limit(chip_info, flag);
            if (ret < 0) {
                TPD_INFO("%s: goodix enable edge limit failed.\n", __func__);
                return ret;
            }
            break;
        case MODE_CHARGE:
            ret = goodix_set_charger_state(chip_info, flag);
            if (ret < 0) {
                TPD_INFO("%s: set charger state failed %d\n", __func__, flag);
            }
            break;
        default:
            TPD_INFO("%s: Wrong mode.\n", __func__);
    }

    return ret;
}

static int goodix_esd_handle(void *chip_data)
{
    s32 i = 0;
    s32 ret = -1;
    u8 esd_buf[4] = {0};
    struct chip_data_gt5688 *chip_info = (struct chip_data_gt5688 *)chip_data;

    for (i = 0; i < 3; i++) {
        ret = touch_i2c_read_block(chip_info->client, GTP_REG_CMD, 4, esd_buf);
        TPD_DEBUG("[Esd]0x8040 = 0x%02X, 0x8043 = 0x%02X gt1x_rawdiff_mode = %d\n", esd_buf[0], esd_buf[3], gt1x_rawdiff_mode);
        if (ret && esd_buf[0] != 0xAA && esd_buf[3] == 0xAA) {
            break;
        }
        TPD_INFO("[Esd]0x8040 = 0x%02X, 0x8043 = 0x%02X gt1x_rawdiff_mode = %d\n", esd_buf[0], esd_buf[3], gt1x_rawdiff_mode);
        msleep(50);
    }

    if (likely(i < 3)) {
        /* IC works normally, Write 0x8040 0xAA, feed the watchdog */
        TPD_DEBUG("time_before(jiffies, timeout) is %d(%lu,%lu), 0x8040 = 0x%02X, 0x8043 = 0x%02X gt1x_rawdiff_mode = %d\n", time_before(jiffies, timeout), jiffies, timeout, esd_buf[0], esd_buf[3], gt1x_rawdiff_mode);
        if (gt1x_rawdiff_mode == 1 || (time_before(jiffies, timeout) && esd_buf[0] == 0x00 && esd_buf[3] == 0xAA)) {
            TPD_DEBUG("quit kick dog!\n");
        } else {
            TPD_DEBUG("kick dog! 0x8040 = 0x%02X\n", esd_buf[0]);
            gt1x_send_cmd(chip_info->client, GTP_CMD_ESD, 0);
        }
    } else {
        TPD_INFO("IC works abnormally! Process esd reset.");

        memset(esd_buf, 0x01, sizeof(esd_buf));
        touch_i2c_write_block(chip_info->client, 0x4226, sizeof(esd_buf), esd_buf);
        msleep(50);

        disable_irq_nosync(chip_info->client->irq);

        tp_powercontrol_2v8(chip_info->hw_res, false);
        msleep(30);
        tp_powercontrol_2v8(chip_info->hw_res, true);
        msleep(10);

        for (i = 0; i < 5; i++) {
            if (goodix_reset(chip_data)) {
                continue;
            }
            if (gt1x_send_cfg(chip_info->client, chip_info->config_info.gt1x_config, chip_info->config_info.gt1x_cfg_length)) {
                msleep(500);
                continue;
            }
            tp_touch_btnkey_release();
            break;
        }

        enable_irq(chip_info->client->irq);

        TPD_INFO("Goodix esd reset over.");
        return -1;
    }

    return 0;
}

static void goodix_debug_info_read(struct seq_file *s, void *chip_data, debug_type debug_type)
{
    int ret = -1, i = 0, j = 0;
    u8 *kernel_buf = NULL;
    int addr;
    struct chip_data_gt5688 *chip_info = (struct chip_data_gt5688 *)chip_data;
    int TX_NUM = chip_info->hw_res->TX_NUM;
    int RX_NUM = chip_info->hw_res->RX_NUM;
    struct i2c_client *client = chip_info->client;

    kernel_buf = kzalloc(4096,GFP_KERNEL);
    if(kernel_buf == NULL)
    {
        TPD_INFO("%s kmalloc error\n", __func__);
        return ;
    }
    switch (debug_type) {
        case GTP_RAWDATA:
            addr = GTP_REG_RAWDATA;
            break;
        case GTP_DIFFDATA:
            addr = GTP_REG_DIFFDATA;
            break;
        default:
            addr = GTP_REG_BASEDATA;
            break;
    }
    gt1x_rawdiff_mode = 1;
    gt1x_send_cmd(client, 1, 0);
    msleep(20);
    touch_i2c_write_byte(client, GTP_READ_COOR_ADDR, 0);
    TPD_INFO("%d,%s\n", __LINE__, __func__);

    //wait for data ready
    while(i++ < 10) {
        ret = touch_i2c_read_block(client, GTP_READ_COOR_ADDR, 1, kernel_buf);
        TPD_INFO("ret = %d \t kernel_buf = %d\n", ret, kernel_buf[0]);
        if(ret && ((kernel_buf[0] & 0x80)==0x80)) {
            TPD_INFO("Data ready OK");
            break;
        }
        msleep(20);
    }
    if(i >= 10) {
        TPD_INFO("data not ready, quit!\n");
        goto read_data_exit;
    }

    ret = touch_i2c_read_block(client, addr, TX_NUM * RX_NUM * 2, kernel_buf);
    msleep(5);

    for(i = 0; i < RX_NUM; i++) {
        seq_printf(s, "[%2d] ", i);
        for(j = 0; j < TX_NUM; j++) {
            //printk("%d ",(s16)(((s16)kernel_buf[i * TX_NUM * 2 + j * 2] << 8) + kernel_buf[i * TX_NUM * 2 + j * 2 + 1]));
            seq_printf(s, "%4d ", (s16)(((s16)kernel_buf[i * TX_NUM * 2 + j * 2] << 8) + kernel_buf[i * TX_NUM * 2 + j * 2 + 1]));
    }
            //printk("\n");
            seq_printf(s, "\n");
    }
read_data_exit:
    gt1x_send_cmd(client, 0, 0);
    gt1x_rawdiff_mode = 0;
    touch_i2c_write_byte(client, GTP_READ_COOR_ADDR, 0);
    kfree(kernel_buf);
    return ;
}

static void goodix_config_info_read(struct seq_file *s, void *chip_data)
{
    int ret = 0, i = 0;
    struct chip_data_gt5688 *chip_info = (struct chip_data_gt5688 *)chip_data;
    struct i2c_client *client = chip_info->client;
    char temp_data[GTP_CONFIG_MAX_LENGTH] = {0};

    seq_printf(s, "==== GT1X default config setting in driver====\n");
    for(i = 0; i < GTP_CONFIG_MAX_LENGTH; i++) {
        seq_printf(s, "0x%02X, ", chip_info->config_info.gt1x_config[i]);
        if(i % 10 == 9)
            seq_printf(s, "\n");
    }

    seq_printf(s, "\n");
    seq_printf(s, "==== GT1X config read from chip====\n");
    ret = touch_i2c_read_block(client, GTP_REG_CONFIG_DATA, GTP_CONFIG_MAX_LENGTH, temp_data);
    TPD_INFO("I2C TRANSFER: %d", ret);

    for(i = 0; i < GTP_CONFIG_MAX_LENGTH; i++) {
        seq_printf(s, "0x%02X, ", temp_data[i]);
        if(i % 10 == 9)
            seq_printf(s, "\n");
    }

    seq_printf(s, "\n");
    seq_printf(s, "==== GT1X Version Info ====\n");

    touch_i2c_read_block(client, GTP_REG_VERSION, 12, temp_data);
    seq_printf(s, "ProductID: GT%c%c%c%c\n", temp_data[0], temp_data[1], temp_data[2], temp_data[3]);
    seq_printf(s, "PatchID: %02X%02X%02X\n", temp_data[4], temp_data[5], temp_data[6]);
    seq_printf(s, "MaskID: %02X%02X%02X\n", temp_data[7], temp_data[8], temp_data[9]);
    seq_printf(s, "SensorID: %02X\n", chip_info->ver_info.sensor_id);

    return;
}

fp_touch_state goodix_spurious_fp_check(void *chip_data)
{
    int ret = 0, TX_NUM = 0, RX_NUM = 0;
    int addr = GTP_REG_RAWDATA, i = 0, j = 0;
    int raw_data_get = 0, raw_data_base = 0, delta_data = 0;
    int error_count = 0;
    uint8_t raw_data[1024] = {0};
    fp_touch_state fp_touch_state = FINGER_PROTECT_TOUCH_UP;

    struct chip_data_gt5688 *chip_info = (struct chip_data_gt5688 *)chip_data;

    TX_NUM = chip_info->hw_res->TX_NUM;
    RX_NUM = chip_info->hw_res->RX_NUM;

    TPD_INFO("%s start!\n", __func__);
    disable_irq_nosync(chip_info->client->irq);
    goodix_reset(chip_info);
    gt1x_rawdiff_mode = 1;
    gt1x_send_cmd(chip_info->client, 1, 0);
    touch_i2c_write_byte(chip_info->client, GTP_READ_COOR_ADDR, 0);

    //wait for data ready
    while(i++ < 10) {
        ret = touch_i2c_read_block(chip_info->client, GTP_READ_COOR_ADDR, 1, raw_data);
        TPD_INFO("ret = %d \t kernel_buf = %d\n", ret, raw_data[0]);
        if(ret && ((raw_data[0] & 0x80) == 0x80)) {
            TPD_INFO("Data ready OK");
            break;
        }
        msleep(5);
    }
    if(i >= 10) {
        TPD_INFO("data not ready, quit!\n");
        goto read_data_exit;
    }

    ret = touch_i2c_read_block(chip_info->client, addr, TX_NUM * RX_NUM * 2, raw_data);

    for(i = RX_NUM - SPURIOUS_FP_RX_NUM; i < RX_NUM - 1; i++) {
        TPD_INFO("[%d]:", i);
        for(j = 1; j < TX_NUM - 1; j++) {
            raw_data_get = (s16)(raw_data [i * TX_NUM * 2 + j * 2] << 8) + raw_data [i * TX_NUM * 2 + j * 2 + 1];
            raw_data_base = chip_info->spuri_fp_touch_raw_data[i * TX_NUM + j];
            delta_data = raw_data_get - raw_data_base;
            printk("%d ",delta_data);
            if(delta_data + 90 < 0) {
                TPD_INFO("delta_data too large, delta_data = %d\n", delta_data);
                error_count++;
            }
        }
        printk("\n");
        if(error_count > 2) {
            fp_touch_state = FINGER_PROTECT_TOUCH_DOWN;
            TPD_INFO("finger protect trigger!report_finger_protect = %d\n", fp_touch_state);
            error_count = 0;
            break;
        } else {
            fp_touch_state = FINGER_PROTECT_TOUCH_UP;
        }
    }
read_data_exit:
    enable_irq(chip_info->client->irq);
    timeout = jiffies + 6 * HZ / 5;
    gt1x_send_cmd(chip_info->client, 0, 0);
    gt1x_rawdiff_mode = 0;
    touch_i2c_write_byte(chip_info->client, GTP_READ_COOR_ADDR, 0);
    return fp_touch_state;
}

//proc/touchpanel/debug_info/delta
static void goodix_delta_read(struct seq_file *s, void *chip_data)
{
    goodix_debug_info_read(s, chip_data, GTP_DIFFDATA);
}

//proc/touchpanel/debug_info/baseline
static void goodix_baseline_read(struct seq_file *s, void *chip_data)
{
    goodix_debug_info_read(s, chip_data, GTP_RAWDATA);
}

//proc/touchpanel/debug_info/main_register
static void goodix_main_register_read(struct seq_file *s, void *chip_data)
{
    uint8_t log_sector[65] = {0};
    uint8_t noise_detect[392] = {0};
    uint8_t noise_rawdata[128] = {0};
    uint8_t XDATA1[256] = {0};
    uint8_t* XDATA2_top = NULL;
    uint8_t* XDATA2_bottom = NULL;
    struct chip_data_gt5688 *chip_info = (struct chip_data_gt5688 *)chip_data;
    int i = 0, j = 0, ret = 0;

    XDATA2_top = kzalloc(2464, GFP_KERNEL);
    XDATA2_bottom = kzalloc(2432, GFP_KERNEL);
    seq_printf(s, "==== GT1X LOG SECTOR 65 bytes FROM 0x8400====\n");
    ret = touch_i2c_read_block(chip_info->client, 0x8400, 65, log_sector);
    for (i = 0; i < 65; i++) {
        seq_printf(s, "0x%02x, ", log_sector[i]);
        if (i % 16 == 15)
            seq_printf(s, "\n");
    }
    seq_printf(s, "\n");

    seq_printf(s, "==== GT1X NOISE DETECT 392 bytes FROM 0x88B8====\n");
    ret = touch_i2c_read_block(chip_info->client, 0x88B8, 392, noise_detect);
    for (i = 0; i < 392; i++) {
        seq_printf(s, "0x%02x, ", noise_detect[i]);
        if (i % 16 == 15)
            seq_printf(s, "\n");
    }
    seq_printf(s, "\n");

    seq_printf(s, "==== GT1X NOISE RAWDATA 128 bytes FROM 0xD62C====\n");
    ret = touch_i2c_read_block(chip_info->client, 0xD62C, 128, noise_rawdata);
    for (i = 0; i < 128; i++) {
        seq_printf(s, "0x%02x, ", noise_rawdata[i]);
        if (i % 16 == 15)
            seq_printf(s, "\n");
    }
    seq_printf(s, "\n");

    seq_printf(s, "==== GT1X XDATA1 256 bytes FROM 0x8200====\n");
    ret = touch_i2c_read_block(chip_info->client, 0x8200, 256, XDATA1);
    for (i = 0; i < 256; i++) {
        seq_printf(s, "0x%02x, ", XDATA1[i]);
        if (i % 16 == 15)
            seq_printf(s, "\n");
    }
    seq_printf(s, "\n");

    seq_printf(s, "==== GT1X XDATA2_TOP 2464 bytes FROM 0x84E0====\n");
    ret = touch_i2c_read_block(chip_info->client, 0x84E0, 2464, XDATA2_top);
    for (i = 0; i < 77; i++) {
        seq_printf(s, "[%d]:", i);
        for(j = 0; j < 32; j++){
            seq_printf(s, "0x%02x, ", XDATA2_top[i * 32 + j]);
        }
        seq_printf(s, "\n");
    }
    seq_printf(s, "\n");

    seq_printf(s, "==== GT1X XDATA2_BOTTOM 2432 bytes FROM 0x8E80====\n");
    ret = touch_i2c_read_block(chip_info->client, 0x8E80, 2432, XDATA2_bottom);
    for (i = 0; i < 76; i++) {
        seq_printf(s, "[%d]:", i);
        for(j = 0; j < 32; j++){
            seq_printf(s, "0x%02x, ", XDATA2_bottom[i * 32 + j]);
        }
        seq_printf(s, "\n");
    }
    seq_printf(s, "\n");

    kfree(XDATA2_top);
    kfree(XDATA2_bottom);
    return;
}

static void goodix_reserve_read(struct seq_file *s, void *chip_data)
{
    return;
}


#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
extern unsigned int upmu_get_rgs_chrdet(void);
static int goodix_get_usb_state(void)
{
    return upmu_get_rgs_chrdet();
}
#else
extern bool oppo_chg_is_usb_present(void);
static int goodix_get_usb_state(void)
{
    return oppo_chg_is_usb_present();
}
#endif

struct oppo_touchpanel_operations goodix_gt5688_ops = {
    .ftm_process      = goodix_ftm_process,
    .get_vendor       = goodix_get_vendor,
    .get_chip_info    = goodix_get_chip_info,
    .reset            = goodix_reset,
    .power_control    = goodix_power_control,
    .fw_check         = goodix_fw_check,
    .fw_update        = goodix_fw_update,
    .trigger_reason   = goodix_trigger_reason,
    .get_touch_points = goodix_get_touch_points,
    .get_gesture_info = goodix_get_gesture_info,
    .get_keycode      = goodix_get_keycode,
    .mode_switch      = goodix_mode_switch,
    .esd_handle       = goodix_esd_handle,
    .fw_handle        = goodix_fw_handle,
    .spurious_fp_check= goodix_spurious_fp_check,
    .get_usb_state    = goodix_get_usb_state,
};

struct goodix_proc_operations goodix_gt5688_proc_ops = {
    .goodix_config_info_read= goodix_config_info_read,
};

static struct debug_info_proc_operations debug_info_proc_ops = {
    .delta_read    = goodix_delta_read,
    .baseline_read = goodix_baseline_read,
    .main_register_read = goodix_main_register_read,
    .reserve_read = goodix_reserve_read,
};

/**
 * gt1x_ts_probe -   I2c probe.
 * @client: i2c device struct.
 * @id: device id.
 * Return  0: succeed, -1: failed.
 */
static int gt1x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    s32 ret = -1;
    struct chip_data_gt5688 *chip_info;
    struct touchpanel_data *ts = NULL;

    TPD_INFO("%s is called\n", __func__);
    //step1:Alloc chip_info
    chip_info = kzalloc(sizeof(struct chip_data_gt5688), GFP_KERNEL);
    if (chip_info == NULL) {
        TPD_INFO("chip info kzalloc error\n");
        ret = -ENOMEM;
        return ret;
    }
    g_chip_info = chip_info;

    //step2:Alloc common ts
    ts = common_touch_data_alloc();
    if (ts == NULL) {
        TPD_INFO("ts kzalloc error\n");
        goto ts_malloc_failed;
    }
    memset(ts, 0, sizeof(*ts));

    //step3:binding client && dev for easy operate
    chip_info->touch_data = kzalloc(MAX_GT_IRQ_DATA_LENGTH, GFP_KERNEL);
    if (chip_info->touch_data == NULL) {
        ret = -ENOMEM;
        TPD_INFO("chip_info->touch_data kzalloc error\n");
        goto touch_data_malloc_failed;
    }

    chip_info->client = client;
    chip_info->goodix_ops = &goodix_gt5688_proc_ops;
    ts->debug_info_ops = &debug_info_proc_ops;
    ts->client = client;
    ts->irq = client->irq;
    ts->dev = &client->dev;
    ts->chip_data = chip_info;
    chip_info->hw_res = &ts->hw_res;
    i2c_set_clientdata(client, ts);

    //step4:file_operations callback binding
    ts->ts_ops = &goodix_gt5688_ops;

    //step5:register common touch
    ret = register_common_touch_device(ts);
    if (ret < 0) {
        goto touch_data_alloc_err;
    }

    ts->tp_suspend_order = TP_LCD_SUSPEND; /* for oncell panel, set tp suspend first */
    //step6: debug tools
    gt1x_init_tool_node(ts, &chip_info->update_info);

    //step7: TP_AUTO_TEST_ID for diffrent IC auto tet
    Goodix_create_proc(ts , chip_info->goodix_ops);

    //step8: get baseline data for finger protect
    if(ts->spurious_fp_support){
        mutex_lock(&ts->mutex);
        goodix_finger_protect_data_get(chip_info);
        mutex_unlock(&ts->mutex);
    }
    return 0;

touch_data_alloc_err:
    kfree(chip_info->touch_data);
    chip_info->touch_data = NULL;

touch_data_malloc_failed:
    common_touch_data_free(ts);
    ts = NULL;

ts_malloc_failed:
    kfree(chip_info);
    chip_info = NULL;
    ret = -1;

    TPD_INFO("%s, probe error\n", __func__);

    return ret;
}

/**
 * gt1x_ts_suspend - i2c suspend callback function.
 * @dev: i2c device.
 * Return  0: succeed, -1: failed.
 */
static int gt1x_pm_suspend(struct device *dev)
{
    struct touchpanel_data *ts = dev_get_drvdata(dev);

    TPD_INFO("%s: is called\n", __func__);
    tp_i2c_suspend(ts);

    return 0;
}

/**
 * gt1x_ts_resume - i2c resume callback function.
 * @dev: i2c device.
 * Return  0: succeed, -1: failed.
 */
static int gt1x_pm_resume(struct device *dev)
{
    struct touchpanel_data *ts = dev_get_drvdata(dev);

    TPD_INFO("%s is called\n", __func__);
    tp_i2c_resume(ts);

    return 0;
}

/* bus control the suspend/resume procedure */
static const struct dev_pm_ops gt1x_ts_pm_ops = {
    .suspend = gt1x_pm_suspend,
    .resume = gt1x_pm_resume,
};

static const struct of_device_id gt1x_match_table[] = {
    {.compatible = "goodix,gt1x", },
    { },
};

static const struct i2c_device_id gt1x_ts_id[] = {
    {GTP_I2C_NAME, 0},
    {}
};

/**
 * gt1x_ts_remove -  Goodix touchscreen driver release function.
 * @client: i2c device struct.
 * Return  0: succeed, -1: failed.
 */
static int gt1x_ts_remove(struct i2c_client *client)
{
    struct touchpanel_data *ts = i2c_get_clientdata(client);

    TPD_INFO("%s is called\n", __func__);
    kfree(ts);

    return 0;
}

static struct i2c_driver gt1x_ts_driver = {
    .probe = gt1x_ts_probe,
    .remove = gt1x_ts_remove,
    .id_table = gt1x_ts_id,
    .driver = {
        .name = GTP_I2C_NAME,
        .owner = THIS_MODULE,
        .of_match_table = gt1x_match_table,
#if defined(CONFIG_FB)
        .pm = &gt1x_ts_pm_ops,
#endif
    },
};

static int __init gt1x_ts_init(void)
{
    TPD_INFO("%s is called\n", __func__);

    tp_judge_ic_match("gt5688");

    if (i2c_add_driver(&gt1x_ts_driver) != 0) {
        TPD_INFO("unable to add i2c driver.\n");
        return -1;
    }

    return 0;
}

/* should never be called */
static void __exit gt1x_ts_exit(void)
{
    i2c_del_driver(&gt1x_ts_driver);

    return;
}

module_init(gt1x_ts_init);
module_exit(gt1x_ts_exit);

MODULE_DESCRIPTION("GTP Series Driver");
MODULE_LICENSE("GPL");

