/**************************************************************
 * Copyright (c)  2008- 2030  Oppo Mobile communication Corp.ltd.
 * File       : goodix_drivers_gt9286.c
 * Description: Source file for Goodix GT9286 driver
 * Version   : 1.0
 * Date        : 2017-08-01
 * Author    : Cong.Dai@Bsp.Group.Tp
 * TAG         : BSP.TP.Init
 * ---------------- Revision History: --------------------------
 *   <version>    <date>          < author >                            <desc>
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

#include "goodix_drivers_gt9286.h"

struct chip_data_gt9286 *g_chip_info = NULL;
unsigned long timeout = 0;
extern int tp_register_times;

#define TPD_DEVICE "Goodix-gt9286"
#define TPD_INFO(fmt, arg...)        pr_err("[TP]"TPD_DEVICE ": " fmt, ##arg)
#define TPD_DEBUG(fmt, arg...)       do{\
    if (tp_debug)\
    pr_err("[TP]"TPD_DEVICE ": " fmt, ##arg);\
}while(0)


//function delcare
static int goodix_reset(void *chip_data);

/********** Start of special i2c tranfer interface used for goodix read/write*****************/

/*
 *goodix_i2c_read_dbl_check----used to double read and check the reading data
 *@client : Handle to slave device
 *@addr   : address of register where to start read
 *@buffer : buf used to store data read from register
 *@len    : data length we want to read
 return  0: success, non-0: failed
*/
static int goodix_i2c_read_dbl_check(struct i2c_client *client, u16 addr, u8 * buffer, s32 len)
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

/*
 *goodix_i2c_write_with_readback----used to write and check the readback data
 *@client : Handle to slave device
 *@addr   : address of register where to start write
 *@buffer : buf we want to send
 *@len    : data length we want to send
 return  0: success, non-0: failed
*/
static int goodix_i2c_write_with_readback(struct i2c_client *client, u16 addr, u8 * buffer, int length)
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

/*
 *goodix_recall_check----used to check register data with source data
 *@client : Handle to slave device
 *@chk_src: source data to be checked
 *@addr   : start adress of reading
 *@chk_len: total check length
 return  0: success, non-0: failed
*/
static int goodix_recall_check(struct i2c_client* client, u8 *chk_src, u16 addr, u16 chk_len)
{
    u8 rd_buf[PACK_SIZE];
    s32 ret = 0;
    u16 len = 0;
    u32 compared_len = 0;

    while (chk_len > 0) {
        len = (chk_len > PACK_SIZE ? PACK_SIZE : chk_len);

        ret = touch_i2c_read_block(client, addr + compared_len, len, rd_buf);
        if (ret < 0) {
            TPD_INFO("recall i2c error, exit!\n");
            return ret;
        }

        if (memcmp(rd_buf, &chk_src[compared_len], len)) {
            TPD_INFO("Recall frame not equal(addr: 0x%04X)\n", addr + compared_len);
            return -1;
        }

        chk_len -= len;
        compared_len += len;
    }

    TPD_DEBUG("Recall check %d bytes(address: 0x%04X) success.\n", compared_len, addr);

    return 0;
}

#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
extern unsigned int upmu_get_rgs_chrdet(void);
static int goodix_get_usb_state(void)
{
    int ret = 0;
    ret = upmu_get_rgs_chrdet();
    TPD_INFO("%s get usb status %d\n", __func__, ret);
    return ret;
}
#else
#endif

/*
 * goodix_send_cmd----send cmd to Goodix ic to control it's function
 * @client: Handle to slave device
 * @cmd: value for cmd reg(0x8040)
 * @data:value for data reg(0x8041)
 * Return  0: succeed, non-0: failed.
 */
static s32 goodix_send_cmd(void *chip_data, u8 cmd, u8 data)
{
    s32 ret;
    struct chip_data_gt9286 *chip_info = (struct chip_data_gt9286 *)chip_data;
    u8 buffer[3] = { cmd, data, 0 };
    static DEFINE_MUTEX(cmd_mutex);

    /* cmd format:
     * reg:0x8040  cmd control(diffrent cmd indicates diffrent setting for touch IC )
     * reg:0x8041  data reg(Correspond with 0x8040)
     * reg:0x8042  checksum (sum(0x8040~0x8042) == 0) */
    mutex_lock(&cmd_mutex);
    buffer[2] = (u8) ((0 - cmd - data) & 0xFF);
    ret = touch_i2c_write_block(chip_info->client, chip_info->reg_info.GTP_REG_CMD + 1, 2, &buffer[1]);
    ret |= touch_i2c_write_byte(chip_info->client, chip_info->reg_info.GTP_REG_CMD, buffer[0]);
    if (ret < 0) {
        TPD_INFO("send cmd failed, ret: %d\n", ret);
    } else {
        ret = 0;
    }
    msleep(50);
    mutex_unlock(&cmd_mutex);

    return ret;
}
/*********** End of special i2c tranfer interface used for goodix read/write******************/


/********* Start of function that work for oppo_touchpanel_operations callbacks***************/
static int goodix_clear_irq(void *chip_data)
{
    int ret = -1;
    struct chip_data_gt9286 *chip_info = (struct chip_data_gt9286 *)chip_data;

    if (!gt1x_rawdiff_mode) {
        ret = touch_i2c_write_byte(chip_info->client, chip_info->reg_info.GTP_REG_READ_COOR, 0);
        if (ret < 0) {
            TPD_INFO("I2C write end_cmd  error!\n");
        }
    }

    return ret;
}

static void getSpecialCornerPoint(uint8_t *buf, int n, struct Coordinate *point)
{
    int x, y, i;

    point[0].x = (buf[0] & 0xFF) | (buf[1] & 0x0F) << 8;
    point[0].y = (buf[2] & 0xFF) | (buf[3] & 0x0F) << 8;
    point[1] = point[0];
    point[2] = point[0];
    point[3] = point[0];
    for (i = 0; i < n; i++) {
        x = (buf[0 + 4*i] & 0xFF) | (buf[1 + 4*i] & 0x0F) << 8;
        y = (buf[2 + 4*i] & 0xFF) | (buf[3 + 4*i] & 0x0F) << 8;
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

static int clockWise(uint8_t *buf, int n)
{
    int i, j, k;
    int count = 0;
    struct Coordinate p[3];
    long int z;

    if (n < 3)
        return 1;
    for (i = 0; i < n; i++) {
        j = (i + 1) % n;
        k = (i + 2) % n;
        p[0].x = (buf[0 + 4*i] & 0xFF) | (buf[1 + 4*i] & 0x0F) << 8;
        p[0].y = (buf[2 + 4*i] & 0xFF) | (buf[3 + 4*i] & 0x0F) << 8;
        p[1].x = (buf[0 + 4*j] & 0xFF) | (buf[1 + 4*j] & 0x0F) << 8;
        p[1].y = (buf[2 + 4*j] & 0xFF) | (buf[3 + 4*j] & 0x0F) << 8;
        p[2].x = (buf[0 + 4*k] & 0xFF) | (buf[1 + 4*k] & 0x0F) << 8;
        p[2].y = (buf[2 + 4*k] & 0xFF) | (buf[3 + 4*k] & 0x0F) << 8;
        if ((p[0].x == p[1].x) && (p[1].x == p[1].y))
            continue;
        z = (p[1].x - p[0].x) * (p[2].y - p[1].y);
        z -= (p[1].y - p[0].y) * (p[2].x - p[1].x);
        if (z < 0)
            count--;
        else if (z > 0)
            count++;
    }

    TPD_INFO("ClockWise count = %d\n", count);

    if (count > 0)
        return 1;
    else
        return 0;
}

static void goodix_esd_check_enable(struct chip_data_gt9286 *chip_info, bool enable)
{
    TPD_INFO("%s %s\n", __func__, enable ? "enable" : "disable");
    /* enable/disable esd check flag */
    chip_info->esd_check_enabled = enable;
    /* update interrupt timer */
    chip_info->irq_timer = jiffies;
    /* clear esd_retry counter, if protect function is enabled */
    chip_info->esd_retry = enable ? 0 : chip_info->esd_retry;
}

static int goodix_enter_sleep(struct chip_data_gt9286 *chip_info, bool config)
{
    s32 retry = 0;

    if (config) {
        while (retry++ < 3) {
            if (!goodix_send_cmd(chip_info, GTP_CMD_SLEEP, 0)) {
                chip_info->halt_status = true;
                TPD_INFO("enter sleep mode!\n");
                return 0;
            }
            msleep(10);
        }
    }

    if (retry >= 3) {
        TPD_INFO("Enter sleep mode failed.\n");
    }

    return -1;
}

static int goodix_enter_doze(struct chip_data_gt9286 *chip_info)
{
    int retry = 0;

    while (retry++ < 5) {
        if (!goodix_send_cmd(chip_info, GTP_CMD_GESTURE, 0)) {
            chip_info->halt_status = true;
            TPD_INFO("enter doze mode!\n");
            return 0;
        }
        msleep(10);
    }

    TPD_INFO("enter doze mode failed.\n");
    return -1;
}

static int goodix_enable_edge_limit(struct chip_data_gt9286 *chip_info, bool enable)
{
    int ret = -1;

    TPD_INFO("%s, edge limit enable = %d\n", __func__, enable);

    if (enable) {
        ret = goodix_send_cmd(chip_info, GTP_CMD_ENABLE_EDGE, 0);
    } else {
        ret = goodix_send_cmd(chip_info, GTP_CMD_DISABLE_EDGE, 0);
    }

    return ret;
}

static int goodix_enable_charge_mode(struct chip_data_gt9286 *chip_info, bool enable)
{
    int ret = -1;

    TPD_INFO("%s, charge mode enable = %d\n", __func__, enable);

    if (enable) {
        ret = goodix_send_cmd(chip_info, GTP_CMD_CHARGER_ON, 0);
    } else {
        ret = goodix_send_cmd(chip_info, GTP_CMD_CHARGER_OFF, 0);
    }

    return ret;
}

static int goodix_set_reset_status(struct chip_data_gt9286 *chip_info)
{
    /* 0x8040 ~ 0x8043 */
    u8 value[] = {0xAA, 0x00, 0x56, 0xAA};
    int ret;

    ret = touch_i2c_write_block(chip_info->client, chip_info->reg_info.GTP_REG_CMD + 1, 3, &value[1]);
    if (ret < 0) {
        TPD_INFO("%s: TP set_reset_status failed\n", __func__);
        return ret;
    }

    return touch_i2c_write_byte(chip_info->client, chip_info->reg_info.GTP_REG_CMD, value[0]);
}

static s32 goodix_read_version(struct chip_data_gt9286 *chip_info, struct panel_info *panel_data)
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
    struct goodix_version_info * ver_info = &chip_info->ver_info;

    while (retry--) {
        ret = goodix_i2c_read_dbl_check(chip_info->client, chip_info->reg_info.GTP_REG_PRODUCT_VER, buf, sizeof(buf));
        if (!ret) {
            checksum = 0;

            for (i = 0; i < sizeof(buf); i++) {
                checksum += buf[i];
            }

            if (checksum == 0 &&    /* first 3 bytes must be number or char */
                    IS_NUM_OR_CHAR(buf[0]) && IS_NUM_OR_CHAR(buf[1]) && IS_NUM_OR_CHAR(buf[2]) && buf[10] != 0xFF) {    /*sensor id == 0xFF, retry */
                break;
            } else {
                TPD_INFO("product version data is error\n");
            }
        } else {
            TPD_INFO("Read product version from 0x8140 failed\n");
        }

        TPD_DEBUG("Read product version retry = %d\n", retry);
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

    TPD_INFO("goodix fw VERSION:GT%s(Product)_%06X(Patch)_%04X(Mask)_%02X(SensorID)\n", product_id, patch_id, mask_id >> 8, sensor_id);

    if (panel_data->manufacture_info.version) {
        snprintf(&(panel_data->manufacture_info.version[8]), MAX_DEVICE_VERSION_LENGTH - 6, "%6x", patch_id);//MAX_DEVICE_VERSION_LENGTH = 16
    }
    TPD_INFO("%s\n", panel_data->manufacture_info.version);

    if (ver_info != NULL) {
        ver_info->mask_id = mask_id;
        ver_info->patch_id = patch_id;
        memcpy(ver_info->product_id, product_id, 5);
        ver_info->sensor_id = sensor_id;
        ver_info->match_opt = match_opt;
    }

    return 0;
}

static s32 goodix_send_config(struct chip_data_gt9286 *chip_info, u8 * config, int cfg_len)
{
    int i;
    s32 ret = 0;
    s32 retry = 0;
    u16 checksum = 0;

    for (i = 0; i < cfg_len - 3; i += 2) {
        checksum += (config[i] << 8) + config[i + 1];
    }
    if (!checksum) {
        TPD_INFO("invalid config, all of the bytes is zero!\n");
        return -1;
    }

    checksum = 0 - checksum;
    TPD_DEBUG("Config checksum: 0x%04X\n", checksum);
    config[cfg_len - 3] = (checksum >> 8) & 0xFF;
    config[cfg_len - 2] = checksum & 0xFF;
    config[cfg_len - 1] = 0x01;

    while (retry++ < 5) {
        ret = touch_i2c_write_block(chip_info->client, chip_info->reg_info.GTP_REG_CONFIG_DATA, cfg_len, config);
        if (ret == cfg_len) {
            msleep(200);    /* at least 200ms, wait for storing config into flash. */
            TPD_INFO("%s: Send config successfully!\n", __func__);
            return 0;
        }
    }

    TPD_INFO("Send config failed!\n");

    return ret;
}

static int goodix_init_panel(struct chip_data_gt9286 *chip_info)
{
    s32 ret = 0;

    chip_info->config_info.goodix_config = GTP_CFG_GROUP_SAMSUNG;
    chip_info->config_info.goodix_config_len = sizeof(GTP_CFG_GROUP_SAMSUNG);

    TPD_DEBUG("config sizeof(chip_info->goodix_config) is %d\n", chip_info->config_info.goodix_config_len);

    if (chip_info->config_info.goodix_config_len < GTP_CONFIG_MIN_LENGTH || chip_info->config_info.goodix_config_len > GTP_CONFIG_MAX_LENGTH) {
        TPD_INFO("Config is INVALID! You need to check you header file CFG_GROUP section!\n");
        return -1;
    }

    /* clear the flag, avoid failure when send the_config of driver. */
    //chip_info->config_info.goodix_config[0] &= 0x7F;

    /* match resolution when gt1x_abs_x_max & gt1x_abs_y_max have been set already */
    if ((chip_info->config_info.goodix_abs_x_max == 0) && (chip_info->config_info.goodix_abs_y_max == 0)) {
        chip_info->config_info.goodix_abs_x_max = (chip_info->config_info.goodix_config[RESOLUTION_LOCATION + 1] << 8) + chip_info->config_info.goodix_config[RESOLUTION_LOCATION];
        chip_info->config_info.goodix_abs_y_max = (chip_info->config_info.goodix_config[RESOLUTION_LOCATION + 3] << 8) + chip_info->config_info.goodix_config[RESOLUTION_LOCATION + 2];
        chip_info->config_info.goodix_int_type = (chip_info->config_info.goodix_config[TRIGGER_LOCATION]) & 0x03;
        chip_info->config_info.goodix_wakeup_level = !(chip_info->config_info.goodix_config[MODULE_SWITCH3_LOCATION] & 0x20);
    } else {
        chip_info->config_info.goodix_config[RESOLUTION_LOCATION] = (u8) chip_info->config_info.goodix_abs_x_max;
        chip_info->config_info.goodix_config[RESOLUTION_LOCATION + 1] = (u8) (chip_info->config_info.goodix_abs_x_max >> 8);
        chip_info->config_info.goodix_config[RESOLUTION_LOCATION + 2] = (u8) chip_info->config_info.goodix_abs_y_max;
        chip_info->config_info.goodix_config[RESOLUTION_LOCATION + 3] = (u8) (chip_info->config_info.goodix_abs_y_max >> 8);
        set_reg_bit(chip_info->config_info.goodix_config[MODULE_SWITCH3_LOCATION], 5, !chip_info->config_info.goodix_wakeup_level);
        chip_info->config_info.goodix_config[TRIGGER_LOCATION] = (chip_info->config_info.goodix_config[TRIGGER_LOCATION] & 0xFC) | chip_info->config_info.goodix_int_type;
    }

    TPD_INFO("X_MAX = %d, Y_MAX = %d, TRIGGER = 0x%02x, WAKEUP_LEVEL = %d\n",
            chip_info->config_info.goodix_abs_x_max, chip_info->config_info.goodix_abs_y_max,
            chip_info->config_info.goodix_int_type, chip_info->config_info.goodix_wakeup_level);

    ret = goodix_send_config(chip_info, chip_info->config_info.goodix_config, chip_info->config_info.goodix_config_len);

    return ret;
}

static s32 goodix_request_event_handler(struct chip_data_gt9286 *chip_info)
{
    s32 ret = -1;
    u8 rqst_data = 0;
    ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.GTP_REG_RQST, 1, &rqst_data);
    if (ret < 0) {
        TPD_INFO("%s: i2c transfer error.\n", __func__);
        return -1;
    }

    TPD_INFO("%s: request state:0x%02x.\n", __func__, rqst_data);

    switch (rqst_data & 0x0F) {
        case GTP_RQST_CONFIG:
            TPD_INFO("Request Config.\n");
            ret = goodix_send_config(chip_info, chip_info->config_info.goodix_config, chip_info->config_info.goodix_config_len);
            if (ret) {
                TPD_INFO("Send goodix_config error.\n");
            } else {
                TPD_INFO("Send goodix_config success.\n");
                rqst_data = GTP_RQST_RESPONDED;
                ret = touch_i2c_write_block(chip_info->client, chip_info->reg_info.GTP_REG_RQST, 1, &rqst_data);
            }
            break;

        case GTP_RQST_RESET:
            TPD_INFO("%s: Request Reset.\n", __func__);
            goodix_reset(chip_info);
            //  gt1x_reset_guitar();
            rqst_data = GTP_RQST_RESPONDED;
            ret = touch_i2c_write_block(chip_info->client, chip_info->reg_info.GTP_REG_RQST, 1, &rqst_data);
            break;

        case GTP_RQST_BAK_REF:
            TPD_INFO("%s: Request Ref.\n", __func__);
            break;

        case GTP_RQST_MAIN_CLOCK:
            TPD_INFO("%s: Request main clock.\n", __func__);
            break;

        default:
            break;
    }

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

static void goodix_reset_via_gpio(struct chip_data_gt9286 *chip_info)
{
    if (gpio_is_valid(chip_info->hw_res->reset_gpio)) {
        gpio_direction_output(chip_info->hw_res->reset_gpio, 0);
        msleep(2);
        gpio_direction_output(chip_info->hw_res->reset_gpio, 1);
        msleep(50);
        msleep(10);

        chip_info->halt_status = false; //reset this flag when ic reset
    }
}

static int goodix_reset(void *chip_data)
{
    int ret = -1;
    struct chip_data_gt9286 *chip_info = (struct chip_data_gt9286 *)chip_data;

    TPD_INFO("%s.\n", __func__);

    goodix_reset_via_gpio(chip_info);
    ret = goodix_set_reset_status(chip_info);
    
    if (ret < 0) {
        TPD_INFO("goodix set reset status failed\n");
        return ret;
    }

    return 0;
}

static u8 *goodix_get_fw_data(struct chip_data_gt9286 *chip_info, u32 offset, int length)
{
    memcpy(chip_info->update_info.buffer, chip_info->update_info.firmware_file_data + offset, length);

    return chip_info->update_info.buffer;
}


static int goodix_check_firmware_data(struct chip_data_gt9286 *chip_info)
{
    u16 actual_checksum;
    u16 checksum_in_header;
    u8 *p;
    struct fw_info *firmware;
    int i;
    int offset;

    TPD_INFO("%s enter.\n", __func__);

    // compare file length with the length field in the firmware header
    if (chip_info->update_info.fw_length < FW_HEAD_SIZE) {
        TPD_INFO("firmware error, (file length: %d) too short\n", chip_info->update_info.fw_length);
        return -1;
    }

    p = goodix_get_fw_data(chip_info, 0, 6);
    if (p == NULL) {
        return -1;
    }

    if (getU32(p) + 6 != chip_info->update_info.fw_length) {
        TPD_INFO("firmware error, (file actual length: %d, header define length: %d)\n", chip_info->update_info.fw_length, getU32(p));
        return -1;
    }
    // check firmware's checksum
    checksum_in_header = getU16(&p[4]);
    actual_checksum = 0;
    for (i = 6; i < chip_info->update_info.fw_length; i++) {
        p = goodix_get_fw_data(chip_info, i, 1);
        if (p == NULL) {
            return -1;
        }
        actual_checksum += p[0];
    }

    TPD_INFO("firmware actual checksum: 0x%04X, header define checksum: 0x%04X\n", actual_checksum, checksum_in_header);

    if (actual_checksum != checksum_in_header) {
        return -1;
    }

    // parse firmware
    p = goodix_get_fw_data(chip_info, 0, FW_HEAD_SIZE);
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

static int __goodix_hold_ss51_dsp_20(struct chip_data_gt9286 *chip_info)
{
    int ret = -1;
    int retry = 0;
    u8 buf[1];
    int hold_times = 0;

    while (retry++ < 30) {
        // Hold ss51 & dsp
        buf[0] = 0x0C;
        ret = touch_i2c_write_byte(chip_info->client, chip_info->reg_info._rRW_MISCTL__SWRST_B0_, buf[0]);
        if (ret < 0) {
            TPD_INFO("Hold ss51 & dsp I2C error, retry:%d\n", retry);
            continue;
        }
        // Confirm hold
        ret = touch_i2c_read_byte(chip_info->client, chip_info->reg_info._rRW_MISCTL__SWRST_B0_);
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


static int goodix_hold_ss51_dsp(struct chip_data_gt9286 *chip_info)
{
    int ret = 0, retry = 5;
    u8 buffer[2];

    do {
        ret = touch_i2c_read_block(chip_info->client, 0x4220, 1, buffer);
    } while (retry-- && ret < 0);

    if (ret < 0)
        return -1;

    //hold ss51_dsp
    ret = __goodix_hold_ss51_dsp_20(chip_info);
    if (ret) {
        return ret;
    }
    // enable dsp & mcu power
    buffer[0] = 0x00;
    ret = goodix_i2c_write_with_readback(chip_info->client, chip_info->reg_info._bRW_MISCTL__DSP_MCU_PWR_, buffer, 1);
    if (ret) {
        TPD_INFO("enabel dsp & mcu power fail!\n");
        return ret;
    }
    // disable watchdog
    buffer[0] = 0x00;
    ret = goodix_i2c_write_with_readback(chip_info->client, chip_info->reg_info._bRW_MISCTL__TMR0_EN, buffer, 1);
    if (ret) {
        TPD_INFO("disable wdt fail!\n");
        return ret;
    }
    // clear cache
    buffer[0] = 0x00;
    ret = goodix_i2c_write_with_readback(chip_info->client, chip_info->reg_info._bRW_MISCTL__CACHE_EN, buffer, 1);
    if (ret) {
        TPD_INFO("clear cache fail!\n");
        return ret;
    }
    // soft reset
    buffer[0] = 0x01;
    ret = touch_i2c_write_block(chip_info->client, chip_info->reg_info._bWO_MISCTL__CPU_SWRST_PULSE, 1, buffer);
    if (ret < 0) {
        TPD_INFO("software reset fail!\n");
        return ret;
    }
    // set scramble
    buffer[0] = 0x00;
    ret = goodix_i2c_write_with_readback(chip_info->client, chip_info->reg_info._rRW_MISCTL__BOOT_OPT_B0_, buffer, 1);
    if (ret) {
        TPD_INFO("set scramble fail!\n");
        return ret;
    }

    return 0;
}

static u16 goodix_calc_checksum(u8 * fw, u32 length)
{
    u32 i = 0;
    u32 checksum = 0;

    for (i = 0; i < length; i += 2) {
        checksum += (((int)fw[i]) << 8);
        checksum += fw[i + 1];
    }

    return (checksum & 0xFFFF);
}

static int goodix_run_ss51_isp(struct chip_data_gt9286 *chip_info, u8 * ss51_isp, int length)
{
    int ret;
    u8 buffer[10];

    ret = goodix_hold_ss51_dsp(chip_info);
    if (ret) {
        return ret;
    }
    // select bank4
    buffer[0] = 0x04;
    ret = goodix_i2c_write_with_readback(chip_info->client, chip_info->reg_info._bRW_MISCTL__SRAM_BANK, buffer, 1);
    if (ret) {
        TPD_INFO("select bank4 fail.\n");
        return ret;
    }
    // enable patch area access
    buffer[0] = 0x01;
    ret = goodix_i2c_write_with_readback(chip_info->client, chip_info->reg_info._bRW_MISCTL__PATCH_AREA_EN_, buffer, 1);
    if (ret) {
        TPD_INFO("enable patch area access fail!\n");
        return ret;
    }

    TPD_INFO("ss51_isp length: %d, checksum: 0x%04X\n", length, goodix_calc_checksum(ss51_isp, length));
    // load ss51 isp
    ret = touch_i2c_write_block(chip_info->client, 0xC000, length, ss51_isp);
    if (ret < 0) {
        TPD_INFO("load ss51 isp fail!\n");
        return ret;
    }
    // recall compare
    ret = goodix_recall_check(chip_info->client, ss51_isp, 0xC000, length);
    if (ret) {
        TPD_INFO("recall check ss51 isp fail!\n");
        return ret;
    }

    memset(buffer, 0xAA, 10);
    ret = goodix_i2c_write_with_readback(chip_info->client, 0x8140, buffer, 10);

    // disable patch area access
    buffer[0] = 0x00;
    ret = goodix_i2c_write_with_readback(chip_info->client, chip_info->reg_info._bRW_MISCTL__PATCH_AREA_EN_, buffer, 1);
    if (ret) {
        TPD_INFO("disable patch area access fail!\n");
        return ret;
    }
    // set 0x8006
    memset(buffer, 0x55, 8);
    ret = goodix_i2c_write_with_readback(chip_info->client, 0x8006, buffer, 8);
    if (ret) {
        TPD_INFO("set 0x8006[0~7] 0x55 fail!\n");
        return ret;
    }
    // release ss51
    buffer[0] = 0x08;
    ret = goodix_i2c_write_with_readback(chip_info->client, chip_info->reg_info._rRW_MISCTL__SWRST_B0_, buffer, 1);
    if (ret) {
        TPD_INFO("release ss51 fail!\n");
        return ret;
    }

    msleep(100);
    // check run state
    ret = touch_i2c_read_block(chip_info->client, 0x8006, 2, buffer);
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

static int goodix_error_erase(struct chip_data_gt9286 *chip_info)
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

    fw = goodix_get_fw_data(chip_info, chip_info->update_info.firmware->subsystem[0].offset,
            chip_info->update_info.firmware->subsystem[0].length);
    if (fw == NULL) {
        TPD_INFO("get isp fail\n");
        return -1;
    }
    ret = goodix_run_ss51_isp(chip_info, fw, chip_info->update_info.firmware->subsystem[0].length);
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
        checksum += goodix_calc_checksum(fw, block_len);
        checksum = (0 - checksum);

        buffer[0] = ((block_len >> 8) & 0xFF);
        buffer[1] = (block_len & 0xFF);
        buffer[2] = ((erase_addr >> 8) & 0xFF);
        buffer[3] = (erase_addr & 0xFF);

        ret = goodix_i2c_write_with_readback(chip_info->client, 0x8100, buffer, 4);
        if (ret) {
            TPD_INFO("write length & address fail!\n");
            continue;
        }

        ret = touch_i2c_write_block(chip_info->client, 0x8100 + 4, block_len, fw);
        if (ret < 0) {
            TPD_INFO("write fw data fail!\n");
            continue;
        }

        ret = goodix_recall_check(chip_info->client, fw, 0x8100 + 4, block_len);
        if (ret) {
            continue;
        }

        buffer[0] = ((checksum >> 8) & 0xFF);
        buffer[1] = (checksum & 0xFF);
        ret = goodix_i2c_write_with_readback(chip_info->client, 0x8100 + 4 + block_len, buffer, 2);
        if (ret) {
            TPD_INFO("write checksum fail!\n");
            continue;
        }

        buffer[0] = 0;
        ret = goodix_i2c_write_with_readback(chip_info->client, 0x8022, buffer, 1);
        if (ret) {
            TPD_INFO("clear control flag fail!\n");
            continue;
        }

        buffer[0] = FW_SECTION_TYPE_SS51_PATCH;
        buffer[1] = FW_SECTION_TYPE_SS51_PATCH;
        ret = goodix_i2c_write_with_readback(chip_info->client, 0x8020, buffer, 2);
        if (ret) {
            TPD_INFO("write subsystem type fail!\n");
            continue;
        }
        burn_state = -1;
        wait_time = 200;
        while (wait_time > 0) {
            wait_time--;
            msleep(5);
            ret = goodix_i2c_read_dbl_check(chip_info->client, 0x8022, buffer, 1);
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

static int goodix_burn_subsystem(struct chip_data_gt9286 *chip_info, struct fw_subsystem_info *subsystem)
{
    int block_len;
    u16 checksum;
    int burn_len = 0;
    u16 cur_addr;
    u32 length = subsystem->length;
    u8 buffer[10];
    int ret;
    int k, pack_num = 0;
    int wait_time;
    int burn_state;
    int retry = 5;
    u8 *fw;

    TPD_INFO("Subsystem: %d, Length: %d, Address: 0x%08X\n", subsystem->type, subsystem->length, subsystem->address);

    while (length > 0 && retry > 0) {
        retry--;

        block_len = length > 4096 ? 4096 : length;

        TPD_INFO("Burn block ==> length: %d, address: 0x%08X\n", block_len, subsystem->address + burn_len);
        fw = goodix_get_fw_data(chip_info, subsystem->offset + burn_len, block_len);
        if (fw == NULL) {
            return -1;
        }

        cur_addr = ((subsystem->address + burn_len) >> 8);

        checksum = 0;
        checksum += block_len;
        checksum += cur_addr;
        checksum += goodix_calc_checksum(fw, block_len);
        checksum = (0 - checksum);

        buffer[0] = ((block_len >> 8) & 0xFF);
        buffer[1] = (block_len & 0xFF);
        buffer[2] = ((cur_addr >> 8) & 0xFF);
        buffer[3] = (cur_addr & 0xFF);

        ret = goodix_i2c_write_with_readback(chip_info->client, 0x8100, buffer, 4);
        if (ret) {
            TPD_INFO("write length & address fail!\n");
            continue;
        }

        pack_num = block_len/2048;
        for (k = 0; k < pack_num; k++) {
            ret = touch_i2c_write_block(chip_info->client, 0x8100 + 4 + 2048*k, 2048, fw + 2048*k);
            if (ret < 0) {
                TPD_INFO("the %d pack, write fw data fail!\n", k);
            }
        }
        if (block_len%2048) {
            ret = touch_i2c_write_block(chip_info->client, 0x8100 + 4 + 2048*k, block_len%2048, fw + 2048*k);
            if (ret < 0) {
                TPD_INFO("the last pack, write fw data fail!\n");
            }
        }

        buffer[0] = ((checksum >> 8) & 0xFF);
        buffer[1] = (checksum & 0xFF);
        ret = goodix_i2c_write_with_readback(chip_info->client, 0x8100 + 4 + block_len, buffer, 2);
        if (ret) {
            TPD_INFO("write checksum fail!\n");
            continue;
        }

        buffer[0] = 0;
        ret = goodix_i2c_write_with_readback(chip_info->client, 0x8022, buffer, 1);
        if (ret) {
            TPD_INFO("clear control flag fail!\n");
            continue;
        }

        buffer[0] = subsystem->type;
        buffer[1] = subsystem->type;
        ret = goodix_i2c_write_with_readback(chip_info->client, 0x8020, buffer, 2);
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

static int goodix_read_flash(struct i2c_client* client, u32 addr, int length)
{
    int wait_time;
    int ret = 0;
    u8 buffer[4];
    u16 read_addr = (addr >> 8);

    TPD_INFO("Read flash: 0x%04X, length: %d\n", addr, length);

    buffer[0] = 0;
    ret = goodix_i2c_write_with_readback(client, 0x8022, buffer, 1);

    buffer[0] = ((length >> 8) & 0xFF);
    buffer[1] = (length & 0xFF);
    buffer[2] = ((read_addr >> 8) & 0xFF);
    buffer[3] = (read_addr & 0xFF);
    ret |= goodix_i2c_write_with_readback(client, 0x8100, buffer, 4);

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
        ret = goodix_i2c_read_dbl_check(client, 0x8022, buffer, 1);
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

static int goodix_check_subsystem_in_flash(struct chip_data_gt9286 *chip_info, struct fw_subsystem_info *subsystem)
{
    int block_len;
    int checked_len = 0;
    u32 length = subsystem->length;
    int ret;
    int check_state = 0;
    u8 *fw;

    TPD_INFO("Subsystem: %d, Length: %d, Address: 0x%08X", subsystem->type, subsystem->length, subsystem->address);

    while (length > 0) {
        block_len = length > 4096 ? 4096 : length;

        TPD_INFO("Check block ==> length: %d, address: 0x%08X\n", block_len, subsystem->address + checked_len);
        fw = goodix_get_fw_data(chip_info, subsystem->offset + checked_len, block_len);
        if (fw == NULL) {
            return -1;
        }
        ret = goodix_read_flash(chip_info->client, subsystem->address + checked_len, block_len);
        if (ret) {
            check_state |= ret;
        }

        ret = goodix_recall_check(chip_info->client, fw, 0x8100, block_len);
        if (ret) {
            TPD_INFO("Block in flash is broken!\n");
            check_state |= ret;
        }

        length -= block_len;
        checked_len += block_len;
    }

    if (check_state) {
        TPD_INFO("Subsystem in flash is broken!\n");
    } else {
        TPD_INFO("Subsystem in flash is correct!\n");
    }
    return check_state;
}

static void goodix_finger_protect_data_get(struct chip_data_gt9286 *chip_info)
{
    int ret = -1;
    int addr, i = 0, j = 0;
    u8 raw_data[1024] = {0};
    int TX_NUM = chip_info->hw_res->TX_NUM;
    int RX_NUM = chip_info->hw_res->RX_NUM;

    chip_info->spuri_fp_touch_raw_data = kzalloc(TX_NUM * RX_NUM * sizeof(u16), GFP_KERNEL);
    addr = chip_info->reg_info.GTP_REG_RAWDATA;

    TPD_INFO("%s start\n", __func__);
    gt1x_rawdiff_mode = 1;
    goodix_send_cmd(chip_info, 1, 0);
    touch_i2c_write_byte(chip_info->client, chip_info->reg_info.GTP_REG_READ_COOR, 0);

    //wait for data ready
    while(i++ < 10) {
        ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.GTP_REG_READ_COOR, 1, raw_data);
        TPD_INFO("ret = %d \t kernel_buf = %d\n", ret, raw_data[0]);
        if((ret > 0) && ((raw_data[0] & 0x80) == 0x80)) {
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
    goodix_send_cmd(chip_info, 0, 0);
    gt1x_rawdiff_mode = 0;
    touch_i2c_write_byte(chip_info->client, chip_info->reg_info.GTP_REG_READ_COOR, 0);
    return;
}

/*********** End of function that work for oppo_touchpanel_operations callbacks***************/


/********* Start of implementation of oppo_touchpanel_operations callbacks********************/
static int goodix_ftm_process(void *chip_data)
{
    struct chip_data_gt9286 *chip_info = (struct chip_data_gt9286 *)chip_data;

    TPD_INFO("%s is called!\n", __func__);
    tp_powercontrol_2v8(chip_info->hw_res, false);

    return 0;
}

static int goodix_get_vendor(void *chip_data, struct panel_info *panel_data)
{
    int len = 0;
    char manu_temp[MAX_DEVICE_MANU_LENGTH] = GOODIX_PREFIX;
    struct chip_data_gt9286 *chip_info = (struct chip_data_gt9286 *)chip_data;

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

static int goodix_get_chip_info(void *chip_data)
{
    struct chip_data_gt9286 *chip_info = (struct chip_data_gt9286 *)chip_data;
    struct goodix_register   *reg_info = &chip_info->reg_info;

    reg_info->GTP_REG_FW_CHK_MAINSYS        = 0x41E4;
    reg_info->GTP_REG_FW_CHK_SUBSYS         = 0x5095;
    reg_info->GTP_REG_CONFIG_DATA           = 0x8050;
    reg_info->GTP_REG_READ_COOR             = 0x814E;
    reg_info->GTP_REG_WAKEUP_GESTURE        = 0x814C;
    reg_info->GTP_REG_GESTURE_COOR          = 0xBDA8;
    reg_info->GTP_REG_CMD                   = 0x8040;
    reg_info->GTP_REG_RQST                  = 0x8044;
    reg_info->GTP_REG_NOISE_DETECT          = 0x804B;
    reg_info->GTP_REG_WATER_PROTECT_QUERY   = 0x8F89;
    reg_info->GTP_REG_PRODUCT_VER           = 0x8140;

    reg_info->GTP_REG_RAWDATA               = 0xB53C;
    reg_info->GTP_REG_DIFFDATA              = 0xAB6C;
    reg_info->GTP_REG_BASEDATA              = 0xB054;

    reg_info->_rRW_MISCTL__SWRST_B0_        = 0x4180;
    reg_info->_bRW_MISCTL__DSP_MCU_PWR_     = 0x4010;
    reg_info->_bRW_MISCTL__TMR0_EN          = 0x40B0;
    reg_info->_bRW_MISCTL__CACHE_EN         = 0x404B;
    reg_info->_bWO_MISCTL__CPU_SWRST_PULSE  = 0x4184;
    reg_info->_rRW_MISCTL__BOOT_OPT_B0_     = 0x4218;
    reg_info->_bRW_MISCTL__SRAM_BANK        = 0x4048;
    reg_info->_bRW_MISCTL__PATCH_AREA_EN_   = 0x404D;

    return goodix_set_reset_status(chip_info);
}

static int goodix_power_control(void *chip_data, bool enable)
{
    int ret = 0;
    struct chip_data_gt9286 *chip_info = (struct chip_data_gt9286 *)chip_data;

    if (true == enable) {
        ret = tp_powercontrol_2v8(chip_info->hw_res, true);
        if (ret)
            return -1;
        ret = tp_powercontrol_1v8(chip_info->hw_res, true);
        if (ret)
            return -1;

        msleep(20);
        goodix_reset_via_gpio(chip_info);
    } else {
        ret = tp_powercontrol_1v8(chip_info->hw_res, false);
        if (ret)
            return -1;
        ret = tp_powercontrol_2v8(chip_info->hw_res, false);
        if (ret)
            return -1;
        if (gpio_is_valid(chip_info->hw_res->reset_gpio)) {
            gpio_direction_output(chip_info->hw_res->reset_gpio, 0);
        }
    }

    return ret;
}

static fw_check_state goodix_fw_check(void *chip_data, struct resolution_info *resolution_info, struct panel_info *panel_data)
{
    int retry = 2;
    int ret = -1;
    u8 reg_val[2] = {0};
    struct chip_data_gt9286 *chip_info = (struct chip_data_gt9286 *)chip_data;

    do {
        goodix_i2c_read_dbl_check(chip_info->client, chip_info->reg_info.GTP_REG_FW_CHK_MAINSYS, reg_val, 1);
        goodix_i2c_read_dbl_check(chip_info->client, chip_info->reg_info.GTP_REG_FW_CHK_SUBSYS, &reg_val[1], 1);

        if (reg_val[0] != 0xBE || reg_val[1] == 0xAA) {
            TPD_INFO("Check fw status failed: reg[0x41E4] = 0x%2X, reg[0x5095] = 0x%2X!\n", reg_val[0], reg_val[1]);
        } else {
            break;
        }
    } while(--retry);

    if (!retry)
        return FW_ABNORMAL;

    ret = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.GTP_REG_CONFIG_DATA);
    if (ret < 0) {
        TPD_INFO("%s: read config version failed, need update firmware!\n", __func__);
        return FW_ABNORMAL;
    } else {
        panel_data->TP_FW = ret & 0x7F; /*bit0-6 is fw version*/
    }

    ret = goodix_read_version(chip_info, panel_data);
    ret = goodix_init_panel(chip_info);
    if (ret != 0) {
        TPD_INFO("%s: init panel failed\n", __func__);
    }

    TPD_INFO("%s: goodix config version = 0x%x \n", __func__, panel_data->TP_FW);
    return FW_NORMAL;
}

static fw_update_state goodix_fw_update(void *chip_data, const struct firmware *fw, bool force)
{
    int i = 0;
    int ret = 0;
    int retry = 2;
    u8 *p;
    u8 reg_val[2] = {0};
    struct goodix_version_info fw_ver_info;
    struct chip_data_gt9286 *chip_info = (struct chip_data_gt9286 *)chip_data;
    struct goodix_version_info ver_info = chip_info->ver_info;

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

    goodix_esd_check_enable(chip_info, false);
    ret = goodix_check_firmware_data(chip_info);
    if (ret < 0) {
        chip_info->update_info.status = UPDATE_STATUS_ABORT;
         TPD_INFO("goodix_check_firmware_data fail\n");
        goto no_update_exit;
    }

    chip_info->update_info.max_progress = 6 + chip_info->update_info.firmware->subsystem_count;
    chip_info->update_info.progress++;

    do {
        goodix_i2c_read_dbl_check(chip_info->client, chip_info->reg_info.GTP_REG_FW_CHK_MAINSYS, reg_val, 1);
        goodix_i2c_read_dbl_check(chip_info->client, chip_info->reg_info.GTP_REG_FW_CHK_SUBSYS, &reg_val[1], 1);

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
            goto no_update_exit;
        }

        if ((fw_ver_info.mask_id & 0xFFFFFF00) != (ver_info.mask_id & 0xFFFFFF00)) {
            TPD_INFO("Mask id is not match!\n");
            goto no_update_exit;
        }

        if (!force) {
            if (fw_ver_info.patch_id  == ver_info.patch_id)
                goto no_update_exit;
        }
    }

    chip_info->update_info.progress++;

    p = goodix_get_fw_data(chip_info, chip_info->update_info.firmware->subsystem[0].offset, chip_info->update_info.firmware->subsystem[0].length);
    if (p == NULL) {
        TPD_INFO("get isp fail\n");
        chip_info->update_info.status = UPDATE_STATUS_ABORT;
        goto update_exit;
    }

    chip_info->update_info.progress++;
    ret = goodix_run_ss51_isp(chip_info, p, chip_info->update_info.firmware->subsystem[0].length);
    if (ret) {
        TPD_INFO("run isp fail\n");
        goto update_exit;
    }

    chip_info->update_info.progress++;

    msleep(800);

    for (i = 1; i < chip_info->update_info.firmware->subsystem_count; i++) {
        TPD_INFO("subsystem: %d\n", chip_info->update_info.firmware->subsystem[i].type);
        TPD_INFO("Length: %d\n", chip_info->update_info.firmware->subsystem[i].length);
        TPD_INFO("Address: %d\n", chip_info->update_info.firmware->subsystem[i].address);

        ret = goodix_burn_subsystem(chip_info, &(chip_info->update_info.firmware->subsystem[i]));
        if (ret) {
            TPD_INFO("burn subsystem fail!\n");
            goto update_exit;
        }
        chip_info->update_info.progress++;
    }

    goodix_reset(chip_data);

    p = goodix_get_fw_data(chip_info, chip_info->update_info.firmware->subsystem[0].offset, chip_info->update_info.firmware->subsystem[0].length);
    if (p == NULL) {
        TPD_INFO("get isp fail\n");
        goto update_exit;
    }
    chip_info->update_info.progress++;

    ret = goodix_run_ss51_isp(chip_info, p, chip_info->update_info.firmware->subsystem[0].length);
    if (ret) {
        TPD_INFO("run isp fail\n");
        goto update_exit;
    }
    chip_info->update_info.progress++;

    TPD_INFO("Reset guitar & check firmware in flash.\n");
    for (i = 1; i < chip_info->update_info.firmware->subsystem_count; i++) {
        TPD_INFO("subsystem: %d\n", chip_info->update_info.firmware->subsystem[i].type);
        TPD_INFO("Length: %d\n", chip_info->update_info.firmware->subsystem[i].length);
        TPD_INFO("Address: %d\n", chip_info->update_info.firmware->subsystem[i].address);

        ret = goodix_check_subsystem_in_flash(chip_info, &(chip_info->update_info.firmware->subsystem[i]));
        if (ret) {
            goodix_error_erase(chip_info);
            break;
        }
    }
    chip_info->update_info.progress++;

    TPD_INFO("%s end.\n", __func__);

update_exit:
    goodix_esd_check_enable(chip_info, true);
    kfree(chip_info->update_info.buffer);
    kfree(chip_info->update_info.firmware);
    chip_info->update_info.status = UPDATE_STATUS_IDLE;

    if (ret) {
        chip_info->update_info.progress = 2 * chip_info->update_info.max_progress;
        TPD_INFO("Update firmware failed!\n");
        return FW_UPDATE_ERROR;
    } else {
        goodix_init_panel(chip_info);   //after update, need send config again
        TPD_INFO("Update firmware succeefully!\n");
        return FW_UPDATE_SUCCESS;
    }

no_update_exit:
    goodix_esd_check_enable(chip_info, true);
    kfree(chip_info->update_info.buffer);
    kfree(chip_info->update_info.firmware);
    chip_info->update_info.status = UPDATE_STATUS_IDLE;
    return FW_NO_NEED_UPDATE;
}

static u8 goodix_trigger_reason(void *chip_data, int gesture_enable, int is_suspended)
{
    int ret = -1;
    u8 touch_num = 0;
    u8 check_sum = 0;
    int i;
    struct chip_data_gt9286 *chip_info = (struct chip_data_gt9286 *)chip_data;

    if (gesture_enable && is_suspended) {   //check whether the gesture trigger
        return IRQ_GESTURE;
    } else if (is_suspended) {
        return IRQ_IGNORE;
    }

    memset(chip_info->touch_data, 0, MAX_GT_IRQ_DATA_LENGTH);
    ret = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.GTP_REG_READ_COOR);
    if (ret < 0) {
        TPD_INFO("%s: i2c transfer error!\n", __func__);
        return IRQ_EXCEPTION;
    }

    chip_info->touch_data[0] = ret & 0xFF;
    touch_num = chip_info->touch_data[0] & 0x0F;

    if (chip_info->touch_data[0] == 0x00) {     //int request
        return IRQ_FW_CONFIG;
    }
    if ((chip_info->touch_data[0] & 0x80) == 0) {   //touch data not ready, wait for next interrupt
        return IRQ_IGNORE;
    }

    touch_num = touch_num < MAX_POINT_NUM ? touch_num : MAX_POINT_NUM;
    ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.GTP_REG_READ_COOR + 1, \
                                                    8 * touch_num + 2, &chip_info->touch_data[1]); //read out point data
    if (ret < 0) {
        TPD_INFO("read touch point data from coor_addr failed!\n");
        return IRQ_IGNORE;
    }

    for (i = 0; i < 1 + 8 * touch_num + 2; i++) {   //do checksum
        check_sum += chip_info->touch_data[i];
    }
    if (check_sum) { /* checksum should be zero, check again when it's no zero */
        ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.GTP_REG_READ_COOR, 3 + 8 * touch_num, chip_info->touch_data);
        if (ret < 0) {
            return IRQ_IGNORE;
        }

        for (i = 0, check_sum = 0; i < 3 + 8 * touch_num; i++) {
            check_sum += chip_info->touch_data[i];
        }
        if (check_sum) {
            TPD_INFO("second checksum error: %x\n", check_sum);
            goto IGNORE_CLEAR_IRQ;
        }
    }
    return IRQ_TOUCH;

IGNORE_CLEAR_IRQ:
    ret = goodix_clear_irq(chip_info);
    return IRQ_IGNORE;
}

static int goodix_get_touch_points(void *chip_data, struct point_info *points, int max_num)
{
    int ret, i;
    int touch_map = 0;
    struct chip_data_gt9286 *chip_info = (struct chip_data_gt9286 *)chip_data;
    u8 touch_num = 0;
    u8 finger_processed = 0;
    u8 *coor_data = NULL;
    s32 id = 0;

    touch_num = chip_info->touch_data[0] & 0x0F;
    if (touch_num == 0) { //Up event
        goto END_TOUCH;
    }

    coor_data = &chip_info->touch_data[1];
    id = coor_data[0] & 0x0F;
    for (i = 0; i < max_num; i++) {
        if (i == id) {
            points[i].x = coor_data[1] | (coor_data[2] << 8);
            points[i].y = coor_data[3] | (coor_data[4] << 8);
            points[i].z = coor_data[5] | (coor_data[6] << 8);
            points[i].width_major = 30; // any value
            points[i].status = 1;
            points[i].touch_major = max(coor_data[5], coor_data[6]);
            touch_map |= 0x01 << i;
            coor_data += 8;

            if (finger_processed++ < touch_num) {
                id = coor_data[0] & 0x0F;
            }
        }
    }
END_TOUCH:
    ret = goodix_clear_irq(chip_info);
    return touch_map;
}

static int goodix_get_gesture_info(void *chip_data, struct gesture_info * gesture)
{
    int ret = -1;
    uint8_t doze_buf[3] = {0};
    uint8_t point_data[MAX_GESTURE_POINT_NUM*4];
    uint8_t point_num = 0;
    uint8_t gesture_id = 0;
    struct Coordinate limitPoint[4];
    struct chip_data_gt9286 *chip_info = (struct chip_data_gt9286 *)chip_data;

    ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.GTP_REG_WAKEUP_GESTURE, 2, doze_buf);
    if (ret < 0) {
        TPD_INFO("%s: read gesture info i2c faild\n", __func__);
        return -1;
    }

    gesture_id = doze_buf[0];
    if (gesture_id == 0) { //no gesture type, no need handle
        goto END_GESTURE;
    }

    memset(point_data, 0, MAX_GESTURE_POINT_NUM*4);
    point_num = doze_buf[1];
    point_num = point_num < MAX_GESTURE_POINT_NUM ? point_num : MAX_GESTURE_POINT_NUM;

    ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.GTP_REG_GESTURE_COOR, point_num * 4, point_data);
    touch_i2c_write_byte(chip_info->client, chip_info->reg_info.GTP_REG_WAKEUP_GESTURE, 0x00);  //clear gesture int
    if (ret < 0) {
        TPD_INFO("%s: read gesture data i2c faild\n", __func__);
        return -1;
    }

    gesture->clockwise = 2;
    switch (gesture_id)     //judge gesture type
    {
        case RIGHT_SLIDE_DETECT :
            gesture->gesture_type  = Left2RightSwip;
            gesture->Point_start.x = (point_data[0] & 0xFF) | (point_data[1] & 0x0F) << 8;
            gesture->Point_start.y = (point_data[2] & 0xFF) | (point_data[3] & 0x0F) << 8;
            gesture->Point_end.x   = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
            gesture->Point_end.y   = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
            break;

        case LEFT_SLIDE_DETECT :
            gesture->gesture_type  = Right2LeftSwip;
            gesture->Point_start.x = (point_data[0] & 0xFF) | (point_data[1] & 0x0F) << 8;
            gesture->Point_start.y = (point_data[2] & 0xFF) | (point_data[3] & 0x0F) << 8;
            gesture->Point_end.x   = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
            gesture->Point_end.y   = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
            break;

        case DOWN_SLIDE_DETECT  :
            gesture->gesture_type  = Up2DownSwip;
            gesture->Point_start.x = (point_data[0] & 0xFF) | (point_data[1] & 0x0F) << 8;
            gesture->Point_start.y = (point_data[2] & 0xFF) | (point_data[3] & 0x0F) << 8;
            gesture->Point_end.x   = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
            gesture->Point_end.y   = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
            break;

        case UP_SLIDE_DETECT :
            gesture->gesture_type  = Down2UpSwip;
            gesture->Point_start.x = (point_data[0] & 0xFF) | (point_data[1] & 0x0F) << 8;
            gesture->Point_start.y = (point_data[2] & 0xFF) | (point_data[3] & 0x0F) << 8;
            gesture->Point_end.x   = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
            gesture->Point_end.y   = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
            break;

        case DTAP_DETECT:
            gesture->gesture_type  = DouTap;
            gesture->Point_start.x = (point_data[0] & 0xFF) | (point_data[1] & 0x0F) << 8;
            gesture->Point_start.y = (point_data[2] & 0xFF) | (point_data[3] & 0x0F) << 8;
            gesture->Point_end.x   = (point_data[0] & 0xFF) | (point_data[1] & 0x0F) << 8;
            gesture->Point_end.y   = (point_data[2] & 0xFF) | (point_data[3] & 0x0F) << 8;
            break;

        case UP_VEE_DETECT :
            gesture->gesture_type  = UpVee;
            gesture->Point_start.x = (point_data[0] & 0xFF) | (point_data[1] & 0x0F) << 8;
            gesture->Point_start.y = (point_data[2] & 0xFF) | (point_data[3] & 0x0F) << 8;
            gesture->Point_end.x   = (point_data[8] & 0xFF) | (point_data[9] & 0x0F) << 8;
            gesture->Point_end.y   = (point_data[10] & 0xFF) | (point_data[11] & 0x0F) << 8;
            gesture->Point_1st.x   = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
            gesture->Point_1st.y   = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
            break;

        case DOWN_VEE_DETECT :
            gesture->gesture_type  = DownVee;
            getSpecialCornerPoint(&point_data[0], point_num, &limitPoint[0]);
            gesture->Point_start.x = (point_data[0] & 0xFF) | (point_data[1] & 0x0F) << 8;
            gesture->Point_start.y = (point_data[2] & 0xFF) | (point_data[3] & 0x0F) << 8;
            gesture->Point_end.x   = (point_data[8] & 0xFF) | (point_data[9] & 0x0F) << 8;
            gesture->Point_end.y   = (point_data[10] & 0xFF) | (point_data[11] & 0x0F) << 8;
            gesture->Point_1st.x   = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
            gesture->Point_1st.y   = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
            break;

        case LEFT_VEE_DETECT:
            gesture->gesture_type = LeftVee;
            getSpecialCornerPoint(&point_data[0], point_num, &limitPoint[0]);
            gesture->Point_start.x = (point_data[0] & 0xFF) | (point_data[1] & 0x0F) << 8;
            gesture->Point_start.y = (point_data[2] & 0xFF) | (point_data[3] & 0x0F) << 8;
            gesture->Point_end.x   = (point_data[8] & 0xFF) | (point_data[9] & 0x0F) << 8;
            gesture->Point_end.y   = (point_data[10] & 0xFF) | (point_data[11] & 0x0F) << 8;
            gesture->Point_1st.x   = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
            gesture->Point_1st.y   = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
            break;

        case RIGHT_VEE_DETECT ://this gesture is C
        case RIGHT_VEE_DETECT2://this gesture is <
            gesture->gesture_type  = RightVee;
            getSpecialCornerPoint(&point_data[0], point_num, &limitPoint[0]);
            gesture->Point_start.x = (point_data[0] & 0xFF) | (point_data[1] & 0x0F) << 8;
            gesture->Point_start.y = (point_data[2] & 0xFF) | (point_data[3] & 0x0F) << 8;
            gesture->Point_end.x   = (point_data[8] & 0xFF) | (point_data[9] & 0x0F) << 8;
            gesture->Point_end.y   = (point_data[10] & 0xFF) | (point_data[11] & 0x0F) << 8;
            gesture->Point_1st.x   = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
            gesture->Point_1st.y   = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
            break;

        case CIRCLE_DETECT  :
            gesture->gesture_type = Circle;
            gesture->clockwise = clockWise(&point_data[0], point_num);
            getSpecialCornerPoint(&point_data[0], point_num, &limitPoint[0]);
            gesture->Point_start.x = (point_data[0] & 0xFF) | (point_data[1] & 0x0F) << 8;
            gesture->Point_start.y = (point_data[2] & 0xFF) | (point_data[3] & 0x0F) << 8;
            gesture->Point_end.x   = (point_data[20] & 0xFF) | (point_data[21] & 0x0F) << 8;
            gesture->Point_end.y   = (point_data[22] & 0xFF) | (point_data[23] & 0x0F) << 8;
            gesture->Point_1st = limitPoint[0]; //ymin
            gesture->Point_2nd = limitPoint[1]; //xmin
            gesture->Point_3rd = limitPoint[2]; //ymax
            gesture->Point_4th = limitPoint[3]; //xmax
            break;

        case DOUSWIP_DETECT  :
            gesture->gesture_type  = DouSwip;
            gesture->Point_start.x = (point_data[0] & 0xFF) | (point_data[1] & 0x0F) << 8;
            gesture->Point_start.y = (point_data[2] & 0xFF) | (point_data[3] & 0x0F) << 8;
            gesture->Point_end.x   = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
            gesture->Point_end.y   = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
            gesture->Point_1st.x   = (point_data[8] & 0xFF) | (point_data[9] & 0x0F) << 8;
            gesture->Point_1st.y   = (point_data[10] & 0xFF) | (point_data[11] & 0x0F) << 8;
            gesture->Point_2nd.x   = (point_data[12] & 0xFF) | (point_data[13] & 0x0F) << 8;
            gesture->Point_2nd.y   = (point_data[14] & 0xFF) | (point_data[15] & 0x0F) << 8;
            break;

        case M_DETECT  :
            gesture->gesture_type  = Mgestrue;
            gesture->Point_start.x = (point_data[0] & 0xFF) | (point_data[1] & 0x0F) << 8;
            gesture->Point_start.y = (point_data[2] & 0xFF) | (point_data[3] & 0x0F) << 8;
            gesture->Point_end.x = (point_data[16] & 0xFF) | (point_data[17] & 0x0F) << 8;
            gesture->Point_end.y = (point_data[18] & 0xFF) | (point_data[19] & 0x0F) << 8;
            gesture->Point_1st.x = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
            gesture->Point_1st.y = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
            gesture->Point_2nd.x = (point_data[8] & 0xFF) | (point_data[9] & 0x0F) << 8;
            gesture->Point_2nd.y = (point_data[10] & 0xFF) | (point_data[11] & 0x0F) << 8;
            gesture->Point_3rd.x = (point_data[12] & 0xFF) | (point_data[13] & 0x0F) << 8;
            gesture->Point_3rd.y = (point_data[14] & 0xFF) | (point_data[15] & 0x0F) << 8;
            break;

        case W_DETECT :
            gesture->gesture_type  = Wgestrue;
            gesture->Point_start.x = (point_data[0] & 0xFF) | (point_data[1] & 0x0F) << 8;
            gesture->Point_start.y = (point_data[2] & 0xFF) | (point_data[3] & 0x0F) << 8;
            gesture->Point_end.x = (point_data[16] & 0xFF) | (point_data[17] & 0x0F) << 8;
            gesture->Point_end.y = (point_data[18] & 0xFF) | (point_data[19] & 0x0F) << 8;
            gesture->Point_1st.x = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
            gesture->Point_1st.y = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
            gesture->Point_2nd.x = (point_data[8] & 0xFF) | (point_data[9] & 0x0F) << 8;
            gesture->Point_2nd.y = (point_data[10] & 0xFF) | (point_data[11] & 0x0F) << 8;
            gesture->Point_3rd.x = (point_data[12] & 0xFF) | (point_data[13] & 0x0F) << 8;
            gesture->Point_3rd.y = (point_data[14] & 0xFF) | (point_data[15] & 0x0F) << 8;
            break;

        default:
            gesture->gesture_type = UnkownGesture;
            break;
    }

    TPD_INFO("%s: gesture_id = 0x%x, gesture_type = %d, clockWise = %d, point:(%d %d)(%d %d)(%d %d)(%d %d)(%d %d)(%d %d)\n",
        __func__, gesture_id, gesture->gesture_type, gesture->clockwise,
        gesture->Point_start.x, gesture->Point_start.y, gesture->Point_end.x, gesture->Point_end.y,
        gesture->Point_1st.x, gesture->Point_1st.y, gesture->Point_2nd.x, gesture->Point_2nd.y,
        gesture->Point_3rd.x, gesture->Point_3rd.y, gesture->Point_4th.x, gesture->Point_4th.y);

END_GESTURE:
    ret = goodix_clear_irq(chip_info);  //clear int
    return 0;
}

static int goodix_mode_switch(void *chip_data, work_mode mode, bool flag)
{
    int ret = -1;
    struct chip_data_gt9286 *chip_info = (struct chip_data_gt9286 *)chip_data;

    if (chip_info->halt_status && (mode != MODE_NORMAL)) {
        goodix_reset(chip_info);
    }

    switch(mode) {
        case MODE_NORMAL:
            ret = 0;    //after reset, it's already normal
            break;

        case MODE_SLEEP:
            ret = goodix_enter_sleep(chip_info, true);
            if (ret < 0) {
                TPD_INFO("%s: goodix enter sleep failed\n", __func__);
            }
            break;

        case MODE_GESTURE:
            if (flag) {
                ret = goodix_enter_doze(chip_info);
                if (ret < 0) {
                    TPD_INFO("%s: goodix enter doze failed\n", __func__);
                }
            }
            break;

        case MODE_EDGE:
            ret = goodix_enable_edge_limit(chip_info, flag);
            if (ret < 0) {
                TPD_INFO("%s: goodix enable:(%d) edge limit failed.\n", __func__, flag);
                return ret;
            }
            break;

        case MODE_CHARGE:
            ret = goodix_enable_charge_mode(chip_info, flag);
            if (ret < 0) {
                TPD_INFO("%s: enable charge mode : %d failed\n", __func__, flag);
            }
            break;

        default:
            TPD_INFO("%s: mode %d not support.\n", __func__, mode);
    }

    return ret;
}

static int goodix_esd_handle(void *chip_data)
{
    s32 i = 0;
    s32 ret = -1;
    u8 esd_buf[4] = {0};
    struct chip_data_gt9286 *chip_info = (struct chip_data_gt9286 *)chip_data;

    if (!chip_info->esd_check_enabled) {
        TPD_INFO("goodix_esd_handle close\n");
        return 0;
    }

    for (i = 0; i < 3; i++) {
        ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.GTP_REG_CMD, 4, esd_buf);
        TPD_INFO("[Esd]0x8040 = 0x%02X, 0x8043 = 0x%02X goodix_rawdiff_mode = %d\n", esd_buf[0], esd_buf[3], gt1x_rawdiff_mode);
        if ((ret > 0) && esd_buf[0] != 0xAA && esd_buf[3] == 0xAA) {
            break;
        }
        msleep(50);
    }

    if (likely(i < 3)) {
        /* IC works normally, Write 0x8040 0xAA, feed the watchdog */
        TPD_DEBUG("time_before(jiffies, timeout) is %d(%lu,%lu), 0x8040 = 0x%02X, 0x8043 = 0x%02X gt1x_rawdiff_mode = %d\n", time_before(jiffies, timeout), jiffies, timeout, esd_buf[0], esd_buf[3], gt1x_rawdiff_mode);
        if (gt1x_rawdiff_mode == 1 || (time_before(jiffies, timeout) && esd_buf[0] == 0x00 && esd_buf[3] == 0xAA)) {
            TPD_DEBUG("quit kick dog!\n");
        } else {
            TPD_DEBUG("kick dog! 0x8040 = 0x%02X\n", esd_buf[0]);
            goodix_send_cmd(chip_info, GTP_CMD_ESD, 0);
        }
    } else {
        TPD_INFO("IC works abnormally! Process esd reset.");

        memset(esd_buf, 0x01, sizeof(esd_buf));
        touch_i2c_write_block(chip_info->client, 0x4226, sizeof(esd_buf), esd_buf);
        msleep(50);

        disable_irq_nosync(chip_info->client->irq);

        goodix_power_control(chip_info, false);
        msleep(30);
        goodix_power_control(chip_info, true);
        msleep(10);

        for (i = 0; i < 5; i++) {
            if (goodix_reset(chip_data)) {
                continue;
            }
            if (goodix_send_config(chip_info, chip_info->config_info.goodix_config, chip_info->config_info.goodix_config_len)) {
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

static int goodix_fw_handle(void *chip_data)
{
    int ret = 0;
    struct chip_data_gt9286 *chip_info = (struct chip_data_gt9286 *)chip_data;

    ret = goodix_request_event_handler(chip_info);
    return ret;
}

static void goodix_register_info_read(void * chip_data, uint16_t register_addr, uint8_t * result, uint8_t length)
{
    int ret = 0;
    struct chip_data_gt9286 *chip_info = (struct chip_data_gt9286 *)chip_data;

    ret = touch_i2c_read_block(chip_info->client, register_addr, length, result);         /*read data*/
}

static void goodix_specific_resume_operate(void *chip_data)
{
    struct chip_data_gt9286 *chip_info = (struct chip_data_gt9286 *)chip_data;
    TPD_DEBUG("%s call\n", __func__);
    goodix_esd_check_enable(chip_info, true);
}

struct oppo_touchpanel_operations goodix_ops = {
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
    .mode_switch      = goodix_mode_switch,
    .esd_handle       = goodix_esd_handle,
    .fw_handle        = goodix_fw_handle,
    .register_info_read = goodix_register_info_read,
    .get_usb_state    = goodix_get_usb_state,
    .specific_resume_operate = goodix_specific_resume_operate,
};
/********* End of implementation of oppo_touchpanel_operations callbacks**********************/

/******** Start of implementation of debug_info_proc_operations callbacks*********************/
static void goodix_debug_info_read(struct seq_file *s, void *chip_data, debug_type debug_type)
{
    int ret = -1, i = 0, j = 0;
    u8 *kernel_buf = NULL;
    int addr;
    struct chip_data_gt9286 *chip_info = (struct chip_data_gt9286 *)chip_data;
    int TX_NUM = chip_info->hw_res->TX_NUM;
    int RX_NUM = chip_info->hw_res->RX_NUM;

    kernel_buf = kzalloc(4096,GFP_KERNEL);
    if(kernel_buf == NULL)
    {
        TPD_INFO("%s kmalloc error\n", __func__);
        return ;
    }
    switch (debug_type) {
        case GTP_RAWDATA:
            addr = chip_info->reg_info.GTP_REG_RAWDATA;
            break;
        case GTP_DIFFDATA:
            addr = chip_info->reg_info.GTP_REG_DIFFDATA;
            break;
        default:
            addr = chip_info->reg_info.GTP_REG_BASEDATA;
            break;
    }
    gt1x_rawdiff_mode = 1;
    goodix_send_cmd(chip_info, 1, 0);
    msleep(20);
    touch_i2c_write_byte(chip_info->client, chip_info->reg_info.GTP_REG_READ_COOR, 0);
    TPD_INFO("%d,%s\n", __LINE__, __func__);

    //wait for data ready
    while(i++ < 10) {
        ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.GTP_REG_READ_COOR, 1, kernel_buf);
        TPD_INFO("ret = %d \t kernel_buf = %d\n", ret, kernel_buf[0]);
        if((ret > 0) && ((kernel_buf[0] & 0x80)==0x80)) {
            TPD_INFO("Data ready OK");
            break;
        }
        msleep(20);
    }
    if(i >= 10) {
        TPD_INFO("data not ready, quit!\n");
        goto read_data_exit;
    }

    if (debug_type == GTP_DIFFDATA) {
        ret = touch_i2c_read_block(chip_info->client, addr, TX_NUM * RX_NUM, kernel_buf);
        msleep(5);

        for(i = 0; i < RX_NUM; i++) {
            seq_printf(s, "[%2d] ", i);
            for(j = 0; j < TX_NUM; j++) {
                seq_printf(s, "%4d ", kernel_buf[j * RX_NUM + i]);
            }
            //printk("\n");
            seq_printf(s, "\n");
        }
    } else {
        ret = touch_i2c_read_block(chip_info->client, addr, TX_NUM * RX_NUM * 2, kernel_buf);
        msleep(5);

        for(i = 0; i < RX_NUM; i++) {
            seq_printf(s, "[%2d] ", i);
            for(j = 0; j < TX_NUM; j++) {
                //printk("%d ",(s16)(((s16)kernel_buf[i * TX_NUM * 2 + j * 2] << 8) + kernel_buf[i * TX_NUM * 2 + j * 2 + 1]));
                seq_printf(s, "%4d ", (kernel_buf[j * RX_NUM * 2 + i * 2] << 8) + kernel_buf[j * RX_NUM * 2 + i * 2 + 1]);
        }
                //printk("\n");
                seq_printf(s, "\n");
        }
    }
read_data_exit:
    goodix_send_cmd(chip_info, 0, 0);
    gt1x_rawdiff_mode = 0;
    touch_i2c_write_byte(chip_info->client, chip_info->reg_info.GTP_REG_READ_COOR, 0);
    kfree(kernel_buf);
    return ;
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
    struct chip_data_gt9286 *chip_info = (struct chip_data_gt9286 *)chip_data;
    int i = 0, j = 0, ret = 0;
    u32 patch_id = 0;
    int retry = 3;
    u8 buf[12] = {0};
    u8 checksum = 0;
    struct goodix_version_info * ver_info = &chip_info->ver_info;

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

    seq_printf(s, "==== water protect status ====\n");
    ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.GTP_REG_WATER_PROTECT_QUERY, 1, XDATA1);
    seq_printf(s, "0x%02x, ", XDATA1[0]);
    seq_printf(s, "\n");

    seq_printf(s, "==== Goodix default config setting in driver====\n");
    for(i = 0; i < GTP_CONFIG_MAX_LENGTH; i++) {
        seq_printf(s, "0x%02X, ", chip_info->config_info.goodix_config[i]);
        if(i % 10 == 9)
            seq_printf(s, "\n");
    }
    seq_printf(s, "\n");

    seq_printf(s, "==== Goodix fw version from tp ic====\n");
    while (retry--) {
        ret = goodix_i2c_read_dbl_check(chip_info->client, chip_info->reg_info.GTP_REG_PRODUCT_VER, buf, sizeof(buf));
        if (!ret) {
            checksum = 0;

            for (i = 0; i < sizeof(buf); i++) {
                checksum += buf[i];
            }

            if (checksum == 0 &&    /* first 3 bytes must be number or char */
                IS_NUM_OR_CHAR(buf[0]) && IS_NUM_OR_CHAR(buf[1]) && IS_NUM_OR_CHAR(buf[2]) && buf[10] != 0xFF) {    /*sensor id == 0xFF, retry */
                break;
            } else {
                TPD_INFO("product version data is error\n");
            }
        } else {
            TPD_INFO("Read product version from 0x8140 failed\n");
        }

        TPD_DEBUG("Read product version retry = %d\n", retry);
        msleep(100);
    }

    if (retry <= 0) {
        if (ver_info)
            ver_info->sensor_id = 0;
        TPD_INFO("Maybe the firmware of ic is error\n");
    }

    patch_id = (u32) ((buf[4] << 16) | (buf[5] << 8) | buf[6]);
    seq_printf(s, "fw version from tp ic : %06X, ", patch_id);
    seq_printf(s, "\n");
    seq_printf(s, "\n");

    kfree(XDATA2_top);
    kfree(XDATA2_bottom);
    return;
}

static struct debug_info_proc_operations debug_info_proc_ops = {
    .delta_read    = goodix_delta_read,
    .baseline_read = goodix_baseline_read,
    .main_register_read = goodix_main_register_read,
};
/********* End of implementation of debug_info_proc_operations callbacks**********************/

/************** Start of callback of proc/Goodix/config_version node**************************/
static void goodix_config_info_read(struct seq_file *s, void *chip_data)
{
    int ret = 0, i = 0;
    struct chip_data_gt9286 *chip_info = (struct chip_data_gt9286 *)chip_data;
    char temp_data[GTP_CONFIG_MAX_LENGTH] = {0};

    seq_printf(s, "==== Goodix default config setting in driver====\n");
    for(i = 0; i < GTP_CONFIG_MAX_LENGTH; i++) {
        seq_printf(s, "0x%02X, ", chip_info->config_info.goodix_config[i]);
        if(i % 10 == 9)
            seq_printf(s, "\n");
    }

    seq_printf(s, "\n");
    seq_printf(s, "==== Goodix config read from chip====\n");
    ret = touch_i2c_read_block(chip_info->client, chip_info->reg_info.GTP_REG_CONFIG_DATA, GTP_CONFIG_MAX_LENGTH, temp_data);
    TPD_INFO("I2C TRANSFER: %d", ret);

    for(i = 0; i < GTP_CONFIG_MAX_LENGTH; i++) {
        seq_printf(s, "0x%02X, ", temp_data[i]);
        if(i % 10 == 9)
            seq_printf(s, "\n");
    }

    seq_printf(s, "\n");
    seq_printf(s, "==== Goodix Version Info ====\n");

    touch_i2c_read_block(chip_info->client, chip_info->reg_info.GTP_REG_PRODUCT_VER, 12, temp_data);
    seq_printf(s, "ProductID: GT%c%c%c%c\n", temp_data[0], temp_data[1], temp_data[2], temp_data[3]);
    seq_printf(s, "PatchID: %02X%02X%02X\n", temp_data[4], temp_data[5], temp_data[6]);
    seq_printf(s, "MaskID: %02X%02X%02X\n", temp_data[7], temp_data[8], temp_data[9]);
    seq_printf(s, "SensorID: %02X\n", chip_info->ver_info.sensor_id);

    return;
}
/*************** End of callback of proc/Goodix/config_version node***************************/

/************** Start of callback of proc/Goodix/waterproofing_mode mode**************************/

static size_t goodix_water_protect_mode_read(struct file *file, char *buf, size_t len, loff_t *pos)
{
    int ret = 0;
    char *temp_buf;
    static bool send_flag = 0;
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));
    struct chip_data_gt9286 *chip_info = (struct chip_data_gt9286 *)ts->chip_data;

    if (!send_flag) {
        send_flag = true;
        temp_buf = kzalloc(len,GFP_KERNEL);
        ret = touch_i2c_read_byte(chip_info->client, chip_info->reg_info.GTP_REG_WATER_PROTECT_QUERY);
        TPD_INFO("%s:%d read GTP_REG_WATER_PROTECT_QUERY = 0x%x\n", __func__, __LINE__, ret);

        ret = snprintf(temp_buf, len, "[%s]water_protect_mode = 0x%x \n", TPD_DEVICE, ret);
        if (copy_to_user(buf, temp_buf, ret))
            TPD_INFO("%s:%d ok\n", __func__, __LINE__);

        kfree(temp_buf);
    } else {
        send_flag = false;
    }
    return ret;
}

static size_t goodix_water_protect_mode_write(struct file *file, const char *buff, size_t len, loff_t *pos)
{
    int ret = 0;
    char buf_tmp[20];
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));
    struct chip_data_gt9286 *chip_info = (struct chip_data_gt9286 *)ts->chip_data;

    TPD_INFO("%s:%d\n",__func__,__LINE__);
    if (len >= 20) {
        TPD_INFO("%s: no command exceeds 20 chars.\n", __func__);
        return -EFAULT;
    }

    if (copy_from_user(buf_tmp, buff, 20)) {
        return -EFAULT;
    }

    if(buf_tmp[0] == '1') {
        ret = goodix_send_cmd(chip_info, GTP_CMD_ENTER_WATER_PROTECT, 0);
        TPD_INFO("%s: GTP_CMD_ENTER_WATER_PROTECT\n", __func__);
    } else {
        ret = goodix_send_cmd(chip_info, GTP_CMD_EXIT_WATER_PROTECT, 0);
        TPD_INFO("%s: GTP_CMD_EXIT_WATER_PROTECT\n", __func__);
    }

    return len;
}
/*************** End of callback of proc/Goodix/waterproofing_mode mode***************************/

struct goodix_proc_operations goodix_gt9286_proc_ops = {
    .goodix_config_info_read= goodix_config_info_read,
    .goodix_water_protect_write = goodix_water_protect_mode_write,
    .goodix_water_protect_read = goodix_water_protect_mode_read,
};

/*********** Start of I2C Driver and Implementation of it's callbacks*************************/
static int goodix_tp_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct chip_data_gt9286 *chip_info = NULL;
    struct touchpanel_data *ts = NULL;
    int ret = -1;
    bool read_cfg_from_dts = false;

    TPD_INFO("%s is called\n", __func__);

    if (tp_register_times > 0) {
        TPD_INFO("TP driver have success loaded %d times, exit\n", tp_register_times);
        return -1;
    }

    /* 1. Alloc chip_info */
    chip_info = kzalloc(sizeof(struct chip_data_gt9286), GFP_KERNEL);
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

    /* 3. alloc touch data space */
    chip_info->touch_data = kzalloc(MAX_GT_IRQ_DATA_LENGTH, GFP_KERNEL);
    if (chip_info->touch_data == NULL) {
        TPD_INFO("touch_data kzalloc error\n");
        goto err_touch_data_alloc;
    }

    /* 4. bind client and dev for easy operate */
    chip_info->client = client;
    chip_info->goodix_ops = &goodix_gt9286_proc_ops;
    ts->debug_info_ops = &debug_info_proc_ops;
    ts->client = client;
    ts->irq = client->irq;
    ts->dev = &client->dev;
    ts->chip_data = chip_info;
    chip_info->hw_res = &ts->hw_res;
    i2c_set_clientdata(client, ts);

    /* 5. file_operations callbacks binding */
    ts->ts_ops = &goodix_ops;

    //read goodix cfg group from dts
    read_cfg_from_dts = of_property_read_bool(ts->dev->of_node, "read_cfg_group_from_dts");
    if (read_cfg_from_dts) {
        ret = of_property_read_u8_array(ts->dev->of_node, "goodix,gtp_cfg_group",
                                        GTP_CFG_GROUP_SAMSUNG, sizeof(GTP_CFG_GROUP_SAMSUNG));
        if (ret) {
            TPD_INFO("read goodix cfg group from dts error\n");
            goto err_touch_data_alloc;
        } else {
            TPD_INFO("read goodix cfg group from dts ok\n");
        }
    }

    /* 6. register common touch device*/
    ret = register_common_touch_device(ts);
    if (ret < 0) {
        goto err_register_driver;
    }

    /* 7. collect data for supurious_fp_touch */
    if (ts->spurious_fp_support) {
        mutex_lock(&ts->mutex);
        goodix_finger_protect_data_get(chip_info);
        mutex_unlock(&ts->mutex);
    }

    /* 8. create goodix tool node */
    gt1x_init_tool_node(ts, &chip_info->update_info);

    /* 9. create goodix debug files */
    Goodix_create_proc(ts , chip_info->goodix_ops);

    goodix_esd_check_enable(chip_info, true);

    TPD_INFO("%s, probe normal end\n", __func__);
    return 0;

err_touch_data_alloc:
    kfree(chip_info->touch_data);
    chip_info->touch_data = NULL;

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

static int goodix_tp_remove(struct i2c_client *client)
{
    struct touchpanel_data *ts = i2c_get_clientdata(client);

    TPD_INFO("%s is called\n", __func__);
    kfree(ts);

    return 0;
}

static int goodix_i2c_suspend(struct device *dev)
{
    struct touchpanel_data *ts = dev_get_drvdata(dev);

    TPD_INFO("%s: is called\n", __func__);
    tp_i2c_suspend(ts);

    return 0;
}

static int goodix_i2c_resume(struct device *dev)
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
    .suspend = goodix_i2c_suspend,
    .resume = goodix_i2c_resume,
#endif
};

static struct i2c_driver tp_i2c_driver = {
    .probe = goodix_tp_probe,
    .remove = goodix_tp_remove,
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

    if (i2c_add_driver(&tp_i2c_driver) != 0) {
        TPD_INFO("unable to add i2c driver.\n");
        return -1;
    }

    return 0;
}


static void __exit tp_driver_exit(void)
{
    i2c_del_driver(&tp_i2c_driver);

    return;
}

module_init(tp_driver_init);
module_exit(tp_driver_exit);
/***********************End of module init and exit*******************************/

MODULE_DESCRIPTION("GTP Touchpanel Driver");
MODULE_LICENSE("GPL v2");

