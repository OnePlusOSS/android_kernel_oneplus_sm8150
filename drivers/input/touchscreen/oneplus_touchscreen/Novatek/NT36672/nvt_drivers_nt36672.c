/*****************************************************************************************
 * Copyright (c)  2017 - 2030  Oppo Mobile communication Corp.ltd.
 * File       : novatek_drivers_nt36672.c
 * Description: Source file for novatek nt36672 driver
 * Version   : 1.0
 * Date        : 2017-09-15
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

#include "nvt_drivers_nt36672.h"

/****************** Start of Log Tag Declear and level define*******************************/
#define TPD_DEVICE "nova-nt36672"
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

static void nvt_change_mode(struct chip_data_nt36672 *chip_info, uint8_t mode);
static int32_t nvt_read_fw_noise(struct chip_data_nt36672 *chip_info, int32_t config_Diff_Test_Frame, int32_t *xdata, int32_t *xdata_n, int32_t xdata_len);
static int8_t nvt_switch_FreqHopEnDis(struct chip_data_nt36672 *chip_info, uint8_t FreqHopEnDis);
void nvt_read_mdata(struct chip_data_nt36672 *chip_info, uint32_t xdata_addr, int32_t *xdata, int32_t xdata_len);
static uint8_t nvt_get_fw_pipe(struct chip_data_nt36672 *chip_info);
static int32_t nvt_check_fw_status(struct chip_data_nt36672 *chip_info);
static void nvt_change_mode(struct chip_data_nt36672 *chip_info, uint8_t mode);
static int32_t nvt_clear_fw_status(struct chip_data_nt36672 *chip_info);
static void store_to_file(int fd, char* format, ...);
static int nvt_get_chip_info(void *chip_data);

/***************************** start of id map table******************************************/
static const struct nvt_mem_map NT36772_memory_map = {
    .EVENT_BUF_ADDR           = 0x11E00,
    .RAW_PIPE0_ADDR           = 0x10000,
    .RAW_PIPE0_Q_ADDR         = 0,
    .RAW_PIPE1_ADDR           = 0x12000,
    .RAW_PIPE1_Q_ADDR         = 0,
    .BASELINE_ADDR            = 0x10E70,
    .BASELINE_Q_ADDR          = 0,
    .BASELINE_BTN_ADDR        = 0x12E70,
    .BASELINE_BTN_Q_ADDR      = 0,
    .DIFF_PIPE0_ADDR          = 0x10830,
    .DIFF_PIPE0_Q_ADDR        = 0,
    .DIFF_PIPE1_ADDR          = 0x12830,
    .DIFF_PIPE1_Q_ADDR        = 0,
    .RAW_BTN_PIPE0_ADDR       = 0x10E60,
    .RAW_BTN_PIPE0_Q_ADDR     = 0,
    .RAW_BTN_PIPE1_ADDR       = 0x12E60,
    .RAW_BTN_PIPE1_Q_ADDR     = 0,
    .DIFF_BTN_PIPE0_ADDR      = 0x10E68,
    .DIFF_BTN_PIPE0_Q_ADDR    = 0,
    .DIFF_BTN_PIPE1_ADDR      = 0x12E68,
    .DIFF_BTN_PIPE1_Q_ADDR    = 0,
    .READ_FLASH_CHECKSUM_ADDR = 0x14000,
    .RW_FLASH_DATA_ADDR       = 0x14002,
    .DOZE_GM_S1D_SCAN_RAW_ADDR= 0x143D0,
    .DOZE_GM_BTN_SCAN_RAW_ADDR= 0x11DF0,
};

static const struct nvt_mem_map NT36672A_memory_map = {
    .EVENT_BUF_ADDR           = 0x21C00,
    .RAW_PIPE0_ADDR           = 0x20000,
    .RAW_PIPE0_Q_ADDR         = 0,
    .RAW_PIPE1_ADDR           = 0x23000,
    .RAW_PIPE1_Q_ADDR         = 0,
    .BASELINE_ADDR            = 0x20BFC,
    .BASELINE_Q_ADDR          = 0,
    .BASELINE_BTN_ADDR        = 0x23BFC,
    .BASELINE_BTN_Q_ADDR      = 0,
    .DIFF_PIPE0_ADDR          = 0x206DC,
    .DIFF_PIPE0_Q_ADDR        = 0,
    .DIFF_PIPE1_ADDR          = 0x236DC,
    .DIFF_PIPE1_Q_ADDR        = 0,
    .RAW_BTN_PIPE0_ADDR       = 0x20510,
    .RAW_BTN_PIPE0_Q_ADDR     = 0,
    .RAW_BTN_PIPE1_ADDR       = 0x23510,
    .RAW_BTN_PIPE1_Q_ADDR     = 0,
    .DIFF_BTN_PIPE0_ADDR      = 0x20BF0,
    .DIFF_BTN_PIPE0_Q_ADDR    = 0,
    .DIFF_BTN_PIPE1_ADDR      = 0x23BF0,
    .DIFF_BTN_PIPE1_Q_ADDR    = 0,
    .READ_FLASH_CHECKSUM_ADDR = 0x24000,
    .RW_FLASH_DATA_ADDR       = 0x24002,
    .DOZE_GM_S1D_SCAN_RAW_ADDR= 0x23C1C,
    .DOZE_GM_BTN_SCAN_RAW_ADDR= 0x23CAC,
};

static const struct nvt_mem_map NT36525_memory_map = {
    .EVENT_BUF_ADDR           = 0x11A00,
    .RAW_PIPE0_ADDR           = 0x10000,
    .RAW_PIPE0_Q_ADDR         = 0,
    .RAW_PIPE1_ADDR           = 0x12000,
    .RAW_PIPE1_Q_ADDR         = 0,
    .BASELINE_ADDR            = 0x10B08,
    .BASELINE_Q_ADDR          = 0,
    .BASELINE_BTN_ADDR        = 0x12B08,
    .BASELINE_BTN_Q_ADDR      = 0,
    .DIFF_PIPE0_ADDR          = 0x1064C,
    .DIFF_PIPE0_Q_ADDR        = 0,
    .DIFF_PIPE1_ADDR          = 0x1264C,
    .DIFF_PIPE1_Q_ADDR        = 0,
    .RAW_BTN_PIPE0_ADDR       = 0x10634,
    .RAW_BTN_PIPE0_Q_ADDR     = 0,
    .RAW_BTN_PIPE1_ADDR       = 0x12634,
    .RAW_BTN_PIPE1_Q_ADDR     = 0,
    .DIFF_BTN_PIPE0_ADDR      = 0x10AFC,
    .DIFF_BTN_PIPE0_Q_ADDR    = 0,
    .DIFF_BTN_PIPE1_ADDR      = 0x12AFC,
    .DIFF_BTN_PIPE1_Q_ADDR    = 0,
    .READ_FLASH_CHECKSUM_ADDR = 0x14000,
    .RW_FLASH_DATA_ADDR       = 0x14002,
    .DOZE_GM_S1D_SCAN_RAW_ADDR= 0x12B28,
    .DOZE_GM_BTN_SCAN_RAW_ADDR= 0x12BB0,
};

static const struct nvt_mem_map NT36870_memory_map = {
    .EVENT_BUF_ADDR           = 0x25000,
    .RAW_PIPE0_ADDR           = 0x20000,
    .RAW_PIPE0_Q_ADDR         = 0x204C8,
    .RAW_PIPE1_ADDR           = 0x23000,
    .RAW_PIPE1_Q_ADDR         = 0x234C8,
    .BASELINE_ADDR            = 0x21350,
    .BASELINE_Q_ADDR          = 0x21818,
    .BASELINE_BTN_ADDR        = 0x24350,
    .BASELINE_BTN_Q_ADDR      = 0x24358,
    .DIFF_PIPE0_ADDR          = 0x209B0,
    .DIFF_PIPE0_Q_ADDR        = 0x20E78,
    .DIFF_PIPE1_ADDR          = 0x239B0,
    .DIFF_PIPE1_Q_ADDR        = 0x23E78,
    .RAW_BTN_PIPE0_ADDR       = 0x20990,
    .RAW_BTN_PIPE0_Q_ADDR     = 0x20998,
    .RAW_BTN_PIPE1_ADDR       = 0x23990,
    .RAW_BTN_PIPE1_Q_ADDR     = 0x23998,
    .DIFF_BTN_PIPE0_ADDR      = 0x21340,
    .DIFF_BTN_PIPE0_Q_ADDR    = 0x21348,
    .DIFF_BTN_PIPE1_ADDR      = 0x24340,
    .DIFF_BTN_PIPE1_Q_ADDR    = 0x24348,
    .READ_FLASH_CHECKSUM_ADDR = 0x24000,
    .RW_FLASH_DATA_ADDR       = 0x24002,
    .DOZE_GM_S1D_SCAN_RAW_ADDR= 0x29910,
    .DOZE_GM_BTN_SCAN_RAW_ADDR= 0x29CC8,
};

static const struct nvt_mem_map NT36676F_memory_map = {
    .EVENT_BUF_ADDR           = 0x11A00,
    .RAW_PIPE0_ADDR           = 0x10000,
    .RAW_PIPE0_Q_ADDR         = 0,
    .RAW_PIPE1_ADDR           = 0x12000,
    .RAW_PIPE1_Q_ADDR         = 0,
    .BASELINE_ADDR            = 0x10B08,
    .BASELINE_Q_ADDR          = 0,
    .BASELINE_BTN_ADDR        = 0x12B08,
    .BASELINE_BTN_Q_ADDR      = 0,
    .DIFF_PIPE0_ADDR          = 0x1064C,
    .DIFF_PIPE0_Q_ADDR        = 0,
    .DIFF_PIPE1_ADDR          = 0x1264C,
    .DIFF_PIPE1_Q_ADDR        = 0,
    .RAW_BTN_PIPE0_ADDR       = 0x10634,
    .RAW_BTN_PIPE0_Q_ADDR     = 0,
    .RAW_BTN_PIPE1_ADDR       = 0x12634,
    .RAW_BTN_PIPE1_Q_ADDR     = 0,
    .DIFF_BTN_PIPE0_ADDR      = 0x10AFC,
    .DIFF_BTN_PIPE0_Q_ADDR    = 0,
    .DIFF_BTN_PIPE1_ADDR      = 0x12AFC,
    .DIFF_BTN_PIPE1_Q_ADDR    = 0,
    .READ_FLASH_CHECKSUM_ADDR = 0x14000,
    .RW_FLASH_DATA_ADDR       = 0x14002,
    .DOZE_GM_S1D_SCAN_RAW_ADDR= 0x12B28,
    .DOZE_GM_BTN_SCAN_RAW_ADDR= 0x12BB0,
};

static const struct nvt_trim_id_table trim_id_table[] = {
    {.id = {0x0A, 0xFF, 0xFF, 0x72, 0x66, 0x03}, .mask = {1, 0, 0, 1, 1, 1},
        .mmap = &NT36672A_memory_map, .carrier_system = 0},
    {.id = {0x55, 0x00, 0xFF, 0x00, 0x00, 0x00}, .mask = {1, 1, 0, 1, 1, 1},
        .mmap = &NT36772_memory_map, .carrier_system = 0},
    {.id = {0x55, 0x72, 0xFF, 0x00, 0x00, 0x00}, .mask = {1, 1, 0, 1, 1, 1},
        .mmap = &NT36772_memory_map, .carrier_system = 0},
    {.id = {0xAA, 0x00, 0xFF, 0x00, 0x00, 0x00}, .mask = {1, 1, 0, 1, 1, 1},
        .mmap = &NT36772_memory_map, .carrier_system = 0},
    {.id = {0xAA, 0x72, 0xFF, 0x00, 0x00, 0x00}, .mask = {1, 1, 0, 1, 1, 1},
        .mmap = &NT36772_memory_map, .carrier_system = 0},
    {.id = {0xFF, 0xFF, 0xFF, 0x72, 0x67, 0x03}, .mask = {0, 0, 0, 1, 1, 1},
        .mmap = &NT36772_memory_map, .carrier_system = 0},
    {.id = {0xFF, 0xFF, 0xFF, 0x70, 0x66, 0x03}, .mask = {0, 0, 0, 1, 1, 1},
        .mmap = &NT36772_memory_map, .carrier_system = 0},
    {.id = {0xFF, 0xFF, 0xFF, 0x70, 0x67, 0x03}, .mask = {0, 0, 0, 1, 1, 1},
        .mmap = &NT36772_memory_map, .carrier_system = 0},
    {.id = {0xFF, 0xFF, 0xFF, 0x72, 0x66, 0x03}, .mask = {0, 0, 0, 1, 1, 1},
        .mmap = &NT36772_memory_map, .carrier_system = 0},
    {.id = {0xFF, 0xFF, 0xFF, 0x25, 0x65, 0x03}, .mask = {0, 0, 0, 1, 1, 1},
        .mmap = &NT36525_memory_map, .carrier_system = 0},
    {.id = {0xFF, 0xFF, 0xFF, 0x70, 0x68, 0x03}, .mask = {0, 0, 0, 1, 1, 1},
        .mmap = &NT36870_memory_map, .carrier_system = 1},
    {.id = {0xFF, 0xFF, 0xFF, 0x76, 0x66, 0x03}, .mask = {0, 0, 0, 1, 1, 1},
        .mmap = &NT36676F_memory_map, .carrier_system = 0}
};

int32_t nvt_i2c_write(struct i2c_client *client, uint16_t address, uint8_t *buf, uint16_t len)
{
    int ret = -1;
    uint16_t addr_cpy = client->addr;

    client->addr = address;
    ret = touch_i2c_write(client, buf, len);
    if (ret < 0) {
        TPD_INFO("%s %d failed\n", __func__, address);
    }
    client->addr = addr_cpy;

    return ret;
}

int32_t nvt_i2c_read(struct i2c_client *client, uint16_t address, uint8_t *buf, uint16_t len)
{
    int ret = -1;
    uint16_t addr_cpy = client->addr;

    client->addr = address;
    ret = touch_i2c_read(client, &buf[0], 1, &buf[1], len - 1);
    if (ret < 0) {
        TPD_INFO("%s %d failed\n", __func__, address);
    }
    client->addr = addr_cpy;

    return ret;
}
/****************************** End of id map table*******************************************/

/****** Start of other functions that work for oppo_touchpanel_operations callbacks***********/
void nvt_bootloader_reset(struct chip_data_nt36672 *chip_info)
{
    int ret = -1;
    uint8_t buf[8] = {0};

    //---write i2c cmds to reset---
    buf[0] = 0x00;
    buf[1] = 0x69;
    ret = nvt_i2c_write(chip_info->client, I2C_HW_Address, buf, 2);
    if (ret < 0) {
        TPD_INFO("write bootloader reset cmds failed!\n");
    }

    // need 35ms delay after bootloader reset
    msleep(35);
}

void nvt_sw_reset_idle(struct chip_data_nt36672 *chip_info)
{
    int ret = -1;
    uint8_t buf[4]={0};

    //---write i2c cmds to reset idle---
    buf[0]=0x00;
    buf[1]=0xA5;
    ret = nvt_i2c_write(chip_info->client, I2C_HW_Address, buf, 2);
    if (ret < 0) {
        TPD_INFO("write reset idle cmds failed!\n");
    }

    msleep(15);
}

void nvt_stop_crc_reboot(struct chip_data_nt36672 *chip_info)
{
    uint8_t buf[8] = {0};
    int32_t retry = 0;

    TPD_INFO("%s: stop crc reboot\n", __func__);
    //read dummy buffer to check CRC fail reboot is happening or not

    //---change I2C index to prevent geting 0xFF, but not 0xFC---
    buf[0] = 0xFF;
    buf[1] = 0x01;
    buf[2] = 0xF6;
    nvt_i2c_write(chip_info->client, I2C_BLDR_Address, buf, 3);

    //---read to check if buf is 0xFC which means IC is in CRC reboot ---
    buf[0] = 0x4E;
    nvt_i2c_read(chip_info->client, I2C_BLDR_Address, buf, 4);

    if (((buf[1] == 0xFC) && (buf[2] == 0xFC) && (buf[3] == 0xFC)) ||
        ((buf[1] == 0xFF) && (buf[2] == 0xFF) && (buf[3] == 0xFF))) {

        //IC is in CRC fail reboot loop, needs to be stopped!
        for (retry = 5; retry > 0; retry--) {
            //---write i2c cmds to reset idle : 1st---
            buf[0]=0x00;
            buf[1]=0xA5;
            nvt_i2c_write(chip_info->client, I2C_HW_Address, buf, 2);

            //---write i2c cmds to reset idle : 2rd---
            buf[0]=0x00;
            buf[1]=0xA5;
            nvt_i2c_write(chip_info->client, I2C_HW_Address, buf, 2);
            msleep(1);

            //---clear CRC_ERR_FLAG---
            buf[0] = 0xFF;
            buf[1] = 0x03;
            buf[2] = 0xF1;
            nvt_i2c_write(chip_info->client, I2C_BLDR_Address, buf, 3);

            buf[0] = 0x35;
            buf[1] = 0xA5;
            nvt_i2c_write(chip_info->client, I2C_BLDR_Address, buf, 2);

            //---check CRC_ERR_FLAG---
            buf[0] = 0xFF;
            buf[1] = 0x03;
            buf[2] = 0xF1;
            nvt_i2c_write(chip_info->client, I2C_BLDR_Address, buf, 3);

            buf[0] = 0x35;
            buf[1] = 0x00;
            nvt_i2c_read(chip_info->client, I2C_BLDR_Address, buf, 2);

            if (buf[1] == 0xA5)
                break;
        }

        if (retry == 0)
            TPD_INFO("CRC auto reboot is not able to be stopped! buf[1]=0x%02X\n", buf[1]);
        }

    return;
}

static int8_t nvt_check_chip_ver_trim(struct chip_data_nt36672 *chip_info)
{
    uint8_t buf[8] = {0};
    int32_t retry = 0;
    int32_t list = 0;
    int32_t i = 0;
    int32_t found_nvt_chip = 0;
    int32_t ret = -1;

    nvt_bootloader_reset(chip_info);

    //---Check for 5 times---
    for (retry = 5; retry > 0; retry--) {
        nvt_sw_reset_idle(chip_info);

        buf[0] = 0x00;
        buf[1] = 0x35;
        ret = nvt_i2c_write(chip_info->client, I2C_HW_Address, buf, 2);
        msleep(10);

        buf[0] = 0xFF;
        buf[1] = 0x01;
        buf[2] = 0xF6;
        ret |= nvt_i2c_write(chip_info->client, I2C_BLDR_Address, buf, 3);

        buf[0] = 0x4E;
        buf[1] = 0x00;
        buf[2] = 0x00;
        buf[3] = 0x00;
        buf[4] = 0x00;
        buf[5] = 0x00;
        buf[6] = 0x00;
        ret |= nvt_i2c_read(chip_info->client, I2C_BLDR_Address, buf, 7);
        if (ret < 0) {
            TPD_INFO("write or read failed in %s \n", __func__);
        }
        TPD_INFO("buf[1]=0x%02X, buf[2]=0x%02X, buf[3]=0x%02X, buf[4]=0x%02X, buf[5]=0x%02X, buf[6]=0x%02X\n",
            buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]);

        // compare read chip id on supported list
        for (list = 0; list < (sizeof(trim_id_table) / sizeof(struct nvt_trim_id_table)); list++) {
            found_nvt_chip = 0;

            // compare each byte
            for (i = 0; i < NVT_ID_BYTE_MAX; i++) {
                if (trim_id_table[list].mask[i]) {
                    if (buf[i + 1] != trim_id_table[list].id[i])
                        break;
                }
            }

            if (i == NVT_ID_BYTE_MAX) {
                found_nvt_chip = 1;
            }

            if (found_nvt_chip) {
                TPD_INFO("This is NVT touch IC, Chip Index %d\n", list);
                chip_info->mmap = trim_id_table[list].mmap;
                chip_info->carrier_system = trim_id_table[list].carrier_system;
                memcpy(chip_info->id, trim_id_table[list].id, NVT_ID_BYTE_MAX); /* store id */
                ret = 0;
                goto out;
            } else {
                chip_info->mmap = NULL;
                ret = -1;
            }
        }

        /*stop crc check to prevent deivce reboot*/
        if (((buf[1] == 0xFC) && (buf[2] == 0xFC) && (buf[3] == 0xFC))||
            ((buf[1] == 0xFF) && (buf[2] == 0xFF) && (buf[3] == 0xFF))) {
            nvt_stop_crc_reboot(chip_info);
        }
        msleep(10);
    }

    if (chip_info->mmap == NULL) {  //set default value
        chip_info->mmap = &NT36772_memory_map;
        chip_info->carrier_system = 0;
        ret = 0;
    }
out:
    return ret;
}

int32_t nvt_check_fw_reset_state(struct chip_data_nt36672 *chip_info, RST_COMPLETE_STATE check_reset_state)
{
    uint8_t buf[8] = {0};
    int32_t ret = 0;
    int32_t retry = 0;

    while (1) {
        msleep(10);

        //---read reset state---
        buf[0] = EVENT_MAP_RESET_COMPLETE;
        buf[1] = 0x00;
        nvt_i2c_read(chip_info->client, I2C_FW_Address, buf, 6);

        if ((buf[1] >= check_reset_state) && (buf[1] <= RESET_STATE_MAX)) {
            ret = 0;
            break;
        }

        retry++;
        if(unlikely(retry > 100)) {
            TPD_INFO("error, retry=%d, buf[1]=0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X\n", retry, buf[1], buf[2], buf[3], buf[4], buf[5]);
            ret = -1;
            break;
        }
    }

    return ret;
}

int32_t nvt_read_pid(struct chip_data_nt36672 *chip_info)
{
    int ret = -1;
    uint8_t buf[3] = {0};

    //---set xdata index to EVENT BUF ADDR---
    buf[0] = 0xFF;
    buf[1] = (chip_info->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
    buf[2] = (chip_info->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
    nvt_i2c_write(chip_info->client, I2C_FW_Address, buf, 3);

    //---read project id---
    buf[0] = EVENT_MAP_PROJECTID;
    buf[1] = 0x00;
    buf[2] = 0x00;
    ret = nvt_i2c_read(chip_info->client, I2C_FW_Address, buf, 3);
    if (ret < 0) {
        TPD_INFO("read pid failed\n");
    } else {
        chip_info->nvt_pid = (buf[2] << 8) + buf[1];
    }

    TPD_INFO("PID=%04X\n", chip_info->nvt_pid);

    return ret;
}

int32_t nvt_get_fw_info(struct chip_data_nt36672 *chip_info)
{
    uint8_t buf[64] = {0};
    uint32_t retry_count = 0;
    int32_t ret = 0;

info_retry:
    //---set xdata index to EVENT BUF ADDR---
    buf[0] = 0xFF;
    buf[1] = (chip_info->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
    buf[2] = (chip_info->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
    nvt_i2c_write(chip_info->client, I2C_FW_Address, buf, 3);

    //---read fw info---
    buf[0] = EVENT_MAP_FWINFO;
    nvt_i2c_read(chip_info->client, I2C_FW_Address, buf, 17);
    chip_info->fw_ver = buf[1];
    // x_num = buf[3];
    // y_num = buf[4];
    // abs_x_max = (uint16_t)((buf[5] << 8) | buf[6]);
    // abs_y_max = (uint16_t)((buf[7] << 8) | buf[8]);
    // max_button_num = buf[11];

    //---clear x_num, y_num if fw info is broken---
    if ((buf[1] + buf[2]) != 0xFF) {
        TPD_INFO("FW info is broken! fw_ver=0x%02X, ~fw_ver=0x%02X\n", buf[1], buf[2]);
        chip_info->fw_ver = 0;

        if(retry_count < 3) {
            retry_count++;
            TPD_INFO("%s retry_count=%d\n", __func__, retry_count);
            goto info_retry;
        } else {
            TPD_INFO("Set default fw_ver=0, x_num=18, y_num=32, abs_x_max=1080, abs_y_max=1920, max_button_num=0!\n");
            ret = -1;
        }
    } else {
        ret = 0;
    }

    //---Get nvttek PID---
    nvt_read_pid(chip_info);

    return ret;
}

static int nvt_enter_sleep(struct chip_data_nt36672 *chip_info, bool config)
{
    int ret = -1;
    uint8_t buf[3];

    if (config) {
        //---set xdata index to EVENT BUF ADDR---
        buf[0] = 0xFF;
        buf[1] = (chip_info->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
        buf[2] = (chip_info->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
        nvt_i2c_write(chip_info->client, I2C_FW_Address, buf, 3);

        buf[0] = EVENT_MAP_HOST_CMD;
        buf[1] = 0x11;
        ret = nvt_i2c_write(chip_info->client, I2C_FW_Address, buf, 2);
        if (ret < 0) {
            TPD_INFO("%s: enter sleep mode failed!\n", __func__);
            return -1;
        } else {
            chip_info->is_sleep_writed = true;
            TPD_INFO("%s: enter sleep mode sucess!\n", __func__);
        }
    }

    return ret;
}

static int nvt_reset(void *chip_data)
{
    int ret = -1;
    struct chip_data_nt36672 *chip_info = (struct chip_data_nt36672 *)chip_data;

    TPD_INFO("%s.\n", __func__);

    nvt_bootloader_reset(chip_info);
    ret = nvt_check_fw_reset_state(chip_info, RESET_STATE_INIT);
    if (ret < 0) {
        TPD_INFO("%s: check reset state(INIT) failed\n", __func__);
    } else {
        chip_info->is_sleep_writed = false;
    }

    return ret;
}

static int8_t nvt_cmd_store(struct chip_data_nt36672 *chip_info, uint8_t u8Cmd)
{
    int i, retry = 5;
    uint8_t buf[3] = {0};

    //---set xdata index to EVENT BUF ADDR---
    buf[0] = 0xFF;
    buf[1] = (chip_info->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
    buf[2] = (chip_info->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
    nvt_i2c_write(chip_info->client, I2C_FW_Address, buf, 3);

    for (i = 0; i < retry; i++) {
        //---set cmd status---
        buf[0] = EVENT_MAP_HOST_CMD;
        buf[1] = u8Cmd;
        nvt_i2c_write(chip_info->client, I2C_FW_Address, buf, 2);

        msleep(20);

        //---read cmd status---
        buf[0] = EVENT_MAP_HOST_CMD;
        buf[1] = 0xFF;
        nvt_i2c_read(chip_info->client, I2C_FW_Address, buf, 2);
        if (buf[1] == 0x00)
            break;
    }

    if (unlikely(i == retry)) {
        TPD_INFO("send Cmd 0x%02X failed, buf[1]=0x%02X\n", u8Cmd, buf[1]);
        return -1;
    } else {
        TPD_INFO("send Cmd 0x%02X success, tried %d times\n", u8Cmd, i);
    }

    return 0;
}

static int nvt_enable_black_gesture(struct chip_data_nt36672 *chip_info, bool enable)
{
    int8_t ret = -1;

    TPD_INFO("%s:enable = %d, chip_info->is_sleep_writed = %d\n", __func__, enable, chip_info->is_sleep_writed);

    if (enable) {
        if (chip_info->is_sleep_writed) {
            nvt_reset(chip_info);
        }

        ret = nvt_cmd_store(chip_info, 0x13);
    } else {
        ret = 0;
    }

    return ret;
}

static int nvt_enable_edge_limit(struct chip_data_nt36672 *chip_info, bool enable)
{
    int8_t ret = -1;

    TPD_INFO("%s:enable = %d, chip_info->is_sleep_writed = %d\n", __func__, enable, chip_info->is_sleep_writed);

    if (chip_info->is_sleep_writed) {
        nvt_reset(chip_info);
    }

    if (enable) {
        ret = nvt_cmd_store(chip_info, EVENTBUFFER_EDGE_LIMIT_ON);
    } else {
        ret = nvt_cmd_store(chip_info, EVENTBUFFER_EDGE_LIMIT_OFF);
    }

    return ret;
}

static int nvt_enable_charge_mode(struct chip_data_nt36672 *chip_info, bool enable)
{
    int8_t ret = -1;

    TPD_INFO("%s:enable = %d, chip_info->is_sleep_writed = %d\n", __func__, enable, chip_info->is_sleep_writed);

    if (chip_info->is_sleep_writed) {
        nvt_reset(chip_info);
    }

    if (enable) {
        ret = nvt_cmd_store(chip_info, EVENTBUFFER_PWR_PLUG_IN);
    } else {
        ret = nvt_cmd_store(chip_info, EVENTBUFFER_PWR_PLUG_OUT);
    }

    return ret;
}

int32_t Resume_PD(struct chip_data_nt36672 *chip_info)
{
    uint8_t buf[8] = {0};
    int32_t ret = 0;
    int32_t retry = 0;

    // Resume Command
    buf[0] = 0x00;
    buf[1] = 0xAB;
    ret = nvt_i2c_write(chip_info->client, I2C_HW_Address, buf, 2);
    if (ret < 0) {
        TPD_INFO("Write Resume Command Enable error!\n");
        return ret;
    }

    // Check 0xAA (Resume Command)
    retry = 0;
    while(1) {
        msleep(1);
        buf[0] = 0x00;
        buf[1] = 0x00;
        ret = nvt_i2c_read(chip_info->client, I2C_HW_Address, buf, 2);
        if (ret < 0) {
            TPD_INFO("Check 0xAA (Resume Command) error!\n");
            return ret;
        }
        if (buf[1] == 0xAA) {
            break;
        }
        retry++;
        if (unlikely(retry > 20)) {
            TPD_INFO("Check 0xAA (Resume Command) error!! status=0x%02X\n", buf[1]);
            return -1;
        }
    }
    msleep(10);

    TPD_INFO("Resume PD OK\n");
    return 0;
}

int32_t Check_CheckSum(struct chip_data_nt36672 *chip_info, const struct firmware *fw)
{
    uint8_t buf[64] = {0};
    uint32_t XDATA_Addr = chip_info->mmap->READ_FLASH_CHECKSUM_ADDR;
    int32_t ret = 0;
    int32_t i = 0;
    int32_t k = 0;
    uint16_t WR_Filechksum[BLOCK_64KB_NUM] = {0};
    uint16_t RD_Filechksum[BLOCK_64KB_NUM] = {0};
    size_t fw_bin_size = 0;
    size_t len_in_blk = 0;
    int32_t retry = 0;

    if (Resume_PD(chip_info)) {
        TPD_INFO("Resume PD error!!\n");
        return -1;
    }

    fw_bin_size = fw->size;

    for (i = 0; i < BLOCK_64KB_NUM; i++) {
        if (fw_bin_size > (i * SIZE_64KB)) {
            // Calculate WR_Filechksum of each 64KB block
            len_in_blk = min(fw_bin_size - i * SIZE_64KB, (size_t)SIZE_64KB);
            WR_Filechksum[i] = i + 0x00 + 0x00 + (((len_in_blk - 1) >> 8) & 0xFF) + ((len_in_blk - 1) & 0xFF);
            for (k = 0; k < len_in_blk; k++) {
                WR_Filechksum[i] += fw->data[k + i * SIZE_64KB];
            }
            WR_Filechksum[i] = 65535 - WR_Filechksum[i] + 1;

            // Fast Read Command
            buf[0] = 0x00;
            buf[1] = 0x07;
            buf[2] = i;
            buf[3] = 0x00;
            buf[4] = 0x00;
            buf[5] = ((len_in_blk - 1) >> 8) & 0xFF;
            buf[6] = (len_in_blk - 1) & 0xFF;
            ret = nvt_i2c_write(chip_info->client, I2C_HW_Address, buf, 7);
            if (ret < 0) {
                TPD_INFO("write Fast Read Command error!\n");
                return ret;
            }
            // Check 0xAA (Fast Read Command)
            retry = 0;
            while (1) {
                msleep(80);
                buf[0] = 0x00;
                buf[1] = 0x00;
                ret = nvt_i2c_read(chip_info->client, I2C_HW_Address, buf, 2);
                if (ret < 0) {
                    TPD_INFO("Check 0xAA (Fast Read Command) error!\n");
                    return ret;
                }
                if (buf[1] == 0xAA) {
                    break;
                }
                retry++;
                if (unlikely(retry > 5)) {
                    TPD_INFO("Check 0xAA (Fast Read Command) failed, buf[1]=0x%02X, retry=%d\n", buf[1], retry);
                    return -1;
                }
            }
            // Read Checksum (write addr high byte & middle byte)
            buf[0] = 0xFF;
            buf[1] = XDATA_Addr >> 16;
            buf[2] = (XDATA_Addr >> 8) & 0xFF;
            ret = nvt_i2c_write(chip_info->client, I2C_BLDR_Address, buf, 3);
            if (ret < 0) {
                TPD_INFO("Read Checksum (write addr high byte & middle byte) error!\n");
                return ret;
            }
            // Read Checksum
            buf[0] = (XDATA_Addr) & 0xFF;
            buf[1] = 0x00;
            buf[2] = 0x00;
            ret = nvt_i2c_read(chip_info->client, I2C_BLDR_Address, buf, 3);
            if (ret < 0) {
                TPD_INFO("Read Checksum error!!(%d)\n", ret);
                return ret;
            }

            RD_Filechksum[i] = (uint16_t)((buf[2] << 8) | buf[1]);
            if (WR_Filechksum[i] != RD_Filechksum[i]) {
                TPD_INFO("RD_Filechksum[%d]=0x%04X, WR_Filechksum[%d]=0x%04X\n", i, RD_Filechksum[i], i, WR_Filechksum[i]);
                TPD_INFO("firmware checksum not match!!\n");
                return 0;
            }
        }
    }

    TPD_INFO("firmware checksum match!\n");
    return 1;
}

int32_t Check_FW_Ver(struct chip_data_nt36672 *chip_info, const struct firmware *fw)
{
    uint8_t buf[16] = {0};
    int32_t ret = 0;

    //write i2c index to EVENT BUF ADDR
    buf[0] = 0xFF;
    buf[1] = (chip_info->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
    buf[2] = (chip_info->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
    ret = nvt_i2c_write(chip_info->client, I2C_BLDR_Address, buf, 3);
    if (ret < 0) {
        TPD_INFO("%s: i2c write error!\n", __func__);
        return ret;
    }

    //read Firmware Version
    buf[0] = EVENT_MAP_FWINFO;
    buf[1] = 0x00;
    buf[2] = 0x00;
    ret = nvt_i2c_read(chip_info->client, I2C_BLDR_Address, buf, 3);
    if (ret < 0) {
        TPD_INFO("%s: i2c read error!\n", __func__);
        return ret;
    }

    TPD_INFO("IC FW Ver = 0x%02X, FW Ver Bar = 0x%02X\n", buf[1], buf[2]);
    TPD_INFO("Bin FW Ver = 0x%02X, FW ver Bar = 0x%02X\n",
            fw->data[FW_BIN_VER_OFFSET], fw->data[FW_BIN_VER_BAR_OFFSET]);

    // check IC FW_VER + FW_VER_BAR equals 0xFF or not, need to update if not
    if ((buf[1] + buf[2]) != 0xFF) {
        TPD_INFO("IC FW_VER + FW_VER_BAR not equals to 0xFF!\n");
        return 0;
    }

    // compare IC and binary FW version
    if (buf[1] == fw->data[FW_BIN_VER_OFFSET])
        return 1;
    else
        return 0;
}

int32_t Init_BootLoader(struct chip_data_nt36672 *chip_info)
{
    uint8_t buf[64] = {0};
    int32_t ret = 0;
    int32_t retry = 0;

    // SW Reset & Idle
    nvt_sw_reset_idle(chip_info);

    // Initiate Flash Block
    buf[0] = 0x00;
    buf[1] = 0x00;
    buf[2] = I2C_FW_Address;
    ret = nvt_i2c_write(chip_info->client, I2C_HW_Address, buf, 3);
    if (ret < 0) {
        TPD_INFO("Inittial Flash Block error!!\n");
        return ret;
    }

    // Check 0xAA (Initiate Flash Block)
    retry = 0;
    while(1) {
        msleep(1);
        buf[0] = 0x00;
        buf[1] = 0x00;
        ret = nvt_i2c_read(chip_info->client, I2C_HW_Address, buf, 2);
        if (ret < 0) {
            TPD_INFO("Check 0xAA (Inittial Flash Block) error!!\n");
            return ret;
        }
        if (buf[1] == 0xAA) {
            break;
        }
        retry++;
        if (unlikely(retry > 20)) {
            TPD_INFO("Check 0xAA (Inittial Flash Block) error!! status=0x%02X\n", buf[1]);
            return -1;
        }
    }

    TPD_INFO("Init OK \n");
    msleep(20);

    return 0;
}

int32_t Erase_Flash(struct chip_data_nt36672 *chip_info, uint32_t fw_len)
{
    uint8_t buf[64] = {0};
    int32_t ret = 0;
    int32_t count = 0;
    int32_t i = 0;
    int32_t Flash_Address = 0;
    int32_t retry = 0;

    // Write Enable
    buf[0] = 0x00;
    buf[1] = 0x06;
    ret = nvt_i2c_write(chip_info->client, I2C_HW_Address, buf, 2);
    if (ret < 0) {
        TPD_INFO("Write Enable (for Write Status Register) error!\n");
        return ret;
    }
    // Check 0xAA (Write Enable)
    retry = 0;
    while (1) {
        mdelay(1);
        buf[0] = 0x00;
        buf[1] = 0x00;
        ret = nvt_i2c_read(chip_info->client, I2C_HW_Address, buf, 2);
        if (ret < 0) {
            TPD_INFO("Check 0xAA (Write Enable for Write Status Register) error!!\n");
            return ret;
        }
        if (buf[1] == 0xAA) {
            break;
        }
        retry++;
        if (unlikely(retry > 20)) {
            TPD_INFO("Check 0xAA (Write Enable for Write Status Register) error! status=0x%02X\n", buf[1]);
            return -1;
        }
    }

    // Write Status Register
    buf[0] = 0x00;
    buf[1] = 0x01;
    buf[2] = 0x00;
    ret = nvt_i2c_write(chip_info->client, I2C_HW_Address, buf, 3);
    if (ret < 0) {
        TPD_INFO("Write Status Register error!\n");
        return ret;
    }
    // Check 0xAA (Write Status Register)
    retry = 0;
    while (1) {
        mdelay(1);
        buf[0] = 0x00;
        buf[1] = 0x00;
        ret = nvt_i2c_read(chip_info->client, I2C_HW_Address, buf, 2);
        if (ret < 0) {
            TPD_INFO("Check 0xAA (Write Status Register) error!!\n");
            return ret;
        }
        if (buf[1] == 0xAA) {
            break;
        }
        retry++;
        if (unlikely(retry > 20)) {
            TPD_INFO("Check 0xAA (Write Status Register) error! status=0x%02X\n", buf[1]);
            return -1;
        }
    }

    // Read Status
    retry = 0;
    while (1) {
        mdelay(5);
        buf[0] = 0x00;
        buf[1] = 0x05;
        ret = nvt_i2c_write(chip_info->client, I2C_HW_Address, buf, 2);
        if (ret < 0) {
            TPD_INFO("Read Status (for Write Status Register) error!!(%d)\n", ret);
            return ret;
        }

        // Check 0xAA (Read Status)
        buf[0] = 0x00;
        buf[1] = 0x00;
        buf[2] = 0x00;
        ret = nvt_i2c_read(chip_info->client, I2C_HW_Address, buf, 3);
        if (ret < 0) {
            TPD_INFO("Check 0xAA (Read Status for Write Status Register) error!\n");
            return ret;
        }
        if ((buf[1] == 0xAA) && (buf[2] == 0x00)) {
            break;
        }
        retry++;
        if (unlikely(retry > 100)) {
            TPD_INFO("Check 0xAA (Read Status for Write Status Register) failed, buf[1]=0x%02X, buf[2]=0x%02X, retry=%d\n", buf[1], buf[2], retry);
            return -1;
        }
    }

    if (fw_len % FLASH_SECTOR_SIZE)
        count = fw_len / FLASH_SECTOR_SIZE + 1;
    else
        count = fw_len / FLASH_SECTOR_SIZE;

    for(i = 0; i < count; i++) {
        // Write Enable
        buf[0] = 0x00;
        buf[1] = 0x06;
        ret = nvt_i2c_write(chip_info->client, I2C_HW_Address, buf, 2);
        if (ret < 0) {
            TPD_INFO("Write Enable error!!(%d)\n", i);
            return ret;
        }
        // Check 0xAA (Write Enable)
        retry = 0;
        while (1) {
            mdelay(1);
            buf[0] = 0x00;
            buf[1] = 0x00;
            ret = nvt_i2c_read(chip_info->client, I2C_HW_Address, buf, 2);
            if (ret < 0) {
                TPD_INFO("Check 0xAA (Write Enable) error!!(%d)\n", i);
                return ret;
            }
            if (buf[1] == 0xAA) {
                break;
            }
            retry++;
            if (unlikely(retry > 20)) {
                TPD_INFO("Check 0xAA (Write Enable) error!! status=0x%02X\n", buf[1]);
                return -1;
            }
        }

        Flash_Address = i * FLASH_SECTOR_SIZE;

        // Sector Erase
        buf[0] = 0x00;
        buf[1] = 0x20;    // Command : Sector Erase
        buf[2] = ((Flash_Address >> 16) & 0xFF);
        buf[3] = ((Flash_Address >> 8) & 0xFF);
        buf[4] = (Flash_Address & 0xFF);
        ret = nvt_i2c_write(chip_info->client, I2C_HW_Address, buf, 5);
        if (ret < 0) {
            TPD_INFO("Sector Erase error!!(%d)\n", i);
            return ret;
        }
        // Check 0xAA (Sector Erase)
        retry = 0;
        while (1) {
            mdelay(1);
            buf[0] = 0x00;
            buf[1] = 0x00;
            ret = nvt_i2c_read(chip_info->client, I2C_HW_Address, buf, 2);
            if (ret < 0) {
                TPD_INFO("Check 0xAA (Sector Erase) error!\n");
                return ret;
            }
            if (buf[1] == 0xAA) {
                break;
            }
            retry++;
            if (unlikely(retry > 20)) {
                TPD_INFO("Check 0xAA (Sector Erase) failed, buf[1]=0x%02X, retry=%d\n", buf[1], retry);
                return -1;
            }
        }

        // Read Status
        retry = 0;
        while (1) {
            mdelay(5);
            buf[0] = 0x00;
            buf[1] = 0x05;
            ret = nvt_i2c_write(chip_info->client, I2C_HW_Address, buf, 2);
            if (ret < 0) {
                TPD_INFO("Read Status error!!(%d)\n", i);
                return ret;
            }

            // Check 0xAA (Read Status)
            buf[0] = 0x00;
            buf[1] = 0x00;
            buf[2] = 0x00;
            ret = nvt_i2c_read(chip_info->client, I2C_HW_Address, buf, 3);
            if (ret < 0) {
                TPD_INFO("Check 0xAA (Read Status) error!!(%d)\n", i);
                return ret;
            }
            if ((buf[1] == 0xAA) && (buf[2] == 0x00)) {
                break;
            }
            retry++;
            if (unlikely(retry > 100)) {
                TPD_INFO("Check 0xAA (Read Status) failed, buf[1]=0x%02X, buf[2]=0x%02X, retry=%d\n", buf[1], buf[2], retry);
                return -1;
            }
        }
    }

    TPD_INFO("Erase OK \n");
    return 0;
}

int32_t Write_Flash(struct chip_data_nt36672 *chip_info, const struct firmware *fw)
{
    uint8_t buf[64] = {0};
    uint32_t XDATA_Addr = chip_info->mmap->RW_FLASH_DATA_ADDR;
    uint32_t Flash_Address = 0;
    int32_t i = 0, j = 0, k = 0;
    uint8_t tmpvalue = 0;
    int32_t count = 0;
    int32_t ret = 0;
    int32_t retry = 0;

    // change I2C buffer index
    buf[0] = 0xFF;
    buf[1] = XDATA_Addr >> 16;
    buf[2] = (XDATA_Addr >> 8) & 0xFF;
    ret = nvt_i2c_write(chip_info->client, I2C_BLDR_Address, buf, 3);
    if (ret < 0) {
        TPD_INFO("change I2C buffer index error!!\n");
        return ret;
    }

    if (fw->size % 256)
        count = fw->size / 256 + 1;
    else
        count = fw->size / 256;

    for (i = 0; i < count; i++) {
        Flash_Address = i * 256;

        // Write Enable
        buf[0] = 0x00;
        buf[1] = 0x06;
        ret = nvt_i2c_write(chip_info->client, I2C_HW_Address, buf, 2);
        if (ret < 0) {
            TPD_INFO("Write Enable error!\n");
            return ret;
        }
        // Check 0xAA (Write Enable)
        retry = 0;
        while (1) {
            udelay(100);
            buf[0] = 0x00;
            buf[1] = 0x00;
            ret = nvt_i2c_read(chip_info->client, I2C_HW_Address, buf, 2);
            if (ret < 0) {
                TPD_INFO("Check 0xAA (Write Enable) error!\n");
                return ret;
            }
            if (buf[1] == 0xAA) {
                break;
            }
            retry++;
            if (unlikely(retry > 20)) {
                TPD_INFO("Check 0xAA (Write Enable) error! status=0x%02X\n", buf[1]);
                return -1;
            }
        }

        // Write Page : 256 bytes
        for (j = 0; j < min(fw->size - i * 256, (size_t)256); j += 32) {
            buf[0] = (XDATA_Addr + j) & 0xFF;
            for (k = 0; k < 32; k++) {
                buf[1 + k] = fw->data[Flash_Address + j + k];
            }
            ret = nvt_i2c_write(chip_info->client, I2C_BLDR_Address, buf, 33);
            if (ret < 0) {
                TPD_INFO("Write Page error!, j=%d\n", j);
                return ret;
            }
        }
        if (fw->size - Flash_Address >= 256)
            tmpvalue=(Flash_Address >> 16) + ((Flash_Address >> 8) & 0xFF) + (Flash_Address & 0xFF) + 0x00 + (255);
        else
            tmpvalue=(Flash_Address >> 16) + ((Flash_Address >> 8) & 0xFF) + (Flash_Address & 0xFF) + 0x00 + (fw->size - Flash_Address - 1);

        for (k = 0;k < min(fw->size - Flash_Address,(size_t)256); k++)
            tmpvalue += fw->data[Flash_Address + k];

        tmpvalue = 255 - tmpvalue + 1;

        // Page Program
        buf[0] = 0x00;
        buf[1] = 0x02;
        buf[2] = ((Flash_Address >> 16) & 0xFF);
        buf[3] = ((Flash_Address >> 8) & 0xFF);
        buf[4] = (Flash_Address & 0xFF);
        buf[5] = 0x00;
        buf[6] = min(fw->size - Flash_Address,(size_t)256) - 1;
        buf[7] = tmpvalue;
        ret = nvt_i2c_write(chip_info->client, I2C_HW_Address, buf, 8);
        if (ret < 0) {
            TPD_INFO("Page Program error!, i=%d\n", i);
            return ret;
        }
        // Check 0xAA (Page Program)
        retry = 0;
        while (1) {
            mdelay(1);
            buf[0] = 0x00;
            buf[1] = 0x00;
            ret = nvt_i2c_read(chip_info->client, I2C_HW_Address, buf, 2);
            if (ret < 0) {
                TPD_INFO("Page Program error!\n");
                return ret;
            }
            if (buf[1] == 0xAA || buf[1] == 0xEA) {
                break;
            }
            retry++;
            if (unlikely(retry > 20)) {
                TPD_INFO("Check 0xAA (Page Program) failed, buf[1]=0x%02X, retry=%d\n", buf[1], retry);
                return -1;
            }
        }
        if (buf[1] == 0xEA) {
            TPD_INFO("Page Program error! i=%d\n", i);
            return -3;
        }

        // Read Status
        retry = 0;
        while (1) {
            mdelay(5);
            buf[0] = 0x00;
            buf[1] = 0x05;
            ret = nvt_i2c_write(chip_info->client, I2C_HW_Address, buf, 2);
            if (ret < 0) {
                TPD_INFO("Read Status error!\n");
                return ret;
            }

            // Check 0xAA (Read Status)
            buf[0] = 0x00;
            buf[1] = 0x00;
            buf[2] = 0x00;
            ret = nvt_i2c_read(chip_info->client, I2C_HW_Address, buf, 3);
            if (ret < 0) {
                TPD_INFO("Check 0xAA (Read Status) error!\n");
                return ret;
            }
            if (((buf[1] == 0xAA) && (buf[2] == 0x00)) || (buf[1] == 0xEA)) {
                break;
            }
            retry++;
            if (unlikely(retry > 100)) {
                TPD_INFO("Check 0xAA (Read Status) failed, buf[1]=0x%02X, buf[2]=0x%02X, retry=%d\n", buf[1], buf[2], retry);
                return -1;
            }
        }
        if (buf[1] == 0xEA) {
            TPD_INFO("Page Program error! i=%d\n", i);
            return -4;
        }

        TPD_DEBUG("Programming...%2d%%\n", ((i * 100) / count));
    }

    TPD_INFO("Programming...%2d%%\n", 100);
    TPD_INFO("Program OK         \n");
    return 0;
}

int32_t Verify_Flash(struct chip_data_nt36672 *chip_info, const struct firmware *fw)
{
    uint8_t buf[64] = {0};
    uint32_t XDATA_Addr = chip_info->mmap->READ_FLASH_CHECKSUM_ADDR;
    int32_t ret = 0;
    int32_t i = 0;
    int32_t k = 0;
    uint16_t WR_Filechksum[BLOCK_64KB_NUM] = {0};
    uint16_t RD_Filechksum[BLOCK_64KB_NUM] = {0};
    size_t fw_bin_size = fw->size;
    size_t len_in_blk = 0;
    int32_t retry = 0;

    for (i = 0; i < BLOCK_64KB_NUM; i++) {
        if (fw_bin_size > (i * SIZE_64KB)) {
            // Calculate WR_Filechksum of each 64KB block
            len_in_blk = min(fw_bin_size - i * SIZE_64KB, (size_t)SIZE_64KB);
            WR_Filechksum[i] = i + 0x00 + 0x00 + (((len_in_blk - 1) >> 8) & 0xFF) + ((len_in_blk - 1) & 0xFF);
            for (k = 0; k < len_in_blk; k++) {
                WR_Filechksum[i] += fw->data[k + i * SIZE_64KB];
            }
            WR_Filechksum[i] = 65535 - WR_Filechksum[i] + 1;

            // Fast Read Command
            buf[0] = 0x00;
            buf[1] = 0x07;
            buf[2] = i;
            buf[3] = 0x00;
            buf[4] = 0x00;
            buf[5] = ((len_in_blk - 1) >> 8) & 0xFF;
            buf[6] = (len_in_blk - 1) & 0xFF;
            ret = nvt_i2c_write(chip_info->client, I2C_HW_Address, buf, 7);
            if (ret < 0) {
                TPD_INFO("Fast Read Command error!\n");
                return ret;
            }
            // Check 0xAA (Fast Read Command)
            retry = 0;
            while (1) {
                msleep(80);
                buf[0] = 0x00;
                buf[1] = 0x00;
                ret = nvt_i2c_read(chip_info->client, I2C_HW_Address, buf, 2);
                if (ret < 0) {
                    TPD_INFO("Check 0xAA (Fast Read Command) error!\n");
                    return ret;
                }
                if (buf[1] == 0xAA) {
                    break;
                }
                retry++;
                if (unlikely(retry > 5)) {
                    TPD_INFO("Check 0xAA (Fast Read Command) failed, buf[1]=0x%02X, retry=%d\n", buf[1], retry);
                    return -1;
                }
            }
            // Read Checksum (write addr high byte & middle byte)
            buf[0] = 0xFF;
            buf[1] = XDATA_Addr >> 16;
            buf[2] = (XDATA_Addr >> 8) & 0xFF;
            ret = nvt_i2c_write(chip_info->client, I2C_BLDR_Address, buf, 3);
            if (ret < 0) {
                TPD_INFO("Read Checksum (write addr high byte & middle byte) error!\n");
                return ret;
            }
            // Read Checksum
            buf[0] = (XDATA_Addr) & 0xFF;
            buf[1] = 0x00;
            buf[2] = 0x00;
            ret = nvt_i2c_read(chip_info->client, I2C_BLDR_Address, buf, 3);
            if (ret < 0) {
                TPD_INFO("Read Checksum error!\n");
                return ret;
            }

            RD_Filechksum[i] = (uint16_t)((buf[2] << 8) | buf[1]);
            if (WR_Filechksum[i] != RD_Filechksum[i]) {
                TPD_INFO("Verify Fail%d!\n", i);
                TPD_INFO("RD_Filechksum[%d]=0x%04X, WR_Filechksum[%d]=0x%04X\n", i, RD_Filechksum[i], i, WR_Filechksum[i]);
                return -1;
            }
        }
    }

    TPD_INFO("Verify OK \n");
    return 0;
}
/****** End of other functions that work for oppo_touchpanel_operations callbacks*************/

/********* Start of implementation of oppo_touchpanel_operations callbacks********************/
static int nvt_ftm_process(void *chip_data)
{
    int ret = -1;
    struct chip_data_nt36672 *chip_info = (struct chip_data_nt36672 *)chip_data;

    TPD_INFO("%s is called!\n", __func__);
    ret = nvt_get_chip_info(chip_info);
    if (!ret) {
        nvt_bootloader_reset(chip_info);
        nvt_check_fw_reset_state(chip_info, RESET_STATE_NORMAL_RUN);
        ret = nvt_enter_sleep(chip_info, true);
    }

    return ret;
}

static int nvt_get_vendor(void *chip_data, struct panel_info *panel_data)
{
    int len = 0;
    struct chip_data_nt36672 *chip_info = (struct chip_data_nt36672 *)chip_data;

    len = strlen(panel_data->fw_name);
    if ((len > 3) && (panel_data->fw_name[len-3] == 'i') && \
        (panel_data->fw_name[len-2] == 'm') && (panel_data->fw_name[len-1] == 'g')) {
        panel_data->fw_name[len-3] = 'b';
        panel_data->fw_name[len-2] = 'i';
        panel_data->fw_name[len-1] = 'n';
    }
    chip_info->tp_type = panel_data->tp_type;
    TPD_INFO("chip_info->tp_type = %d, panel_data->fw_name = %s\n", chip_info->tp_type, panel_data->fw_name);

    return 0;
}

//return 0--success; return -EVODEV -- failed;
int32_t nvt_detect_chip(struct chip_data_nt36672 *chip_info)
{
        uint8_t buf[2] = {0};

        if (nvt_i2c_read(chip_info->client, I2C_HW_Address, buf, 2) < 0) {
                return -ENODEV;
        }

        return 0;
}

static int nvt_get_chip_info(void *chip_data)
{
    int ret = -1;
    struct chip_data_nt36672 *chip_info = (struct chip_data_nt36672 *)chip_data;

    //check i2c ok, avoid too much i2c error log
    ret = nvt_detect_chip(chip_info);
    if(ret < 0) {
        TPD_INFO("no find novatek chip or i2c fatal error\n");
        return -EINVAL;
    }

    //---check chip version trim---
    ret = nvt_check_chip_ver_trim(chip_info);
    if (ret) {
        TPD_INFO("chip is not identified\n");
        ret = -EINVAL;
    }

    return ret;
}

static int nvt_power_control(void *chip_data, bool enable)
{
    return 0;
}

static int nvt_reset_gpio_control(void *chip_data, bool enable)
{
    struct chip_data_nt36672 *chip_info = (struct chip_data_nt36672 *)chip_data;
    if (gpio_is_valid(chip_info->hw_res->reset_gpio)) {
        TPD_INFO("%s: set reset state %d\n", __func__, enable);
        gpio_set_value(chip_info->hw_res->reset_gpio, enable);
    }

    return 0;
}

static fw_check_state nvt_fw_check(void *chip_data, struct resolution_info *resolution_info, struct panel_info *panel_data)
{
    uint8_t ver_len = 0;
    int ret = 0;
    char dev_version[MAX_DEVICE_VERSION_LENGTH] = {0};
    struct chip_data_nt36672 *chip_info = (struct chip_data_nt36672 *)chip_data;

    nvt_bootloader_reset(chip_info);
    ret = nvt_check_fw_reset_state(chip_info, RESET_STATE_INIT);
    ret |= nvt_get_fw_info(chip_info);
    if (ret < 0) {
        TPD_INFO("%s: get fw info failed\n", __func__);
        return FW_ABNORMAL;
    } else {
        panel_data->TP_FW = chip_info->fw_ver;
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

static fw_update_state nvt_fw_update(void *chip_data, const struct firmware *fw, bool force)
{
    int ret = -1;
    fw_update_state update_state = FW_NO_NEED_UPDATE;
    struct chip_data_nt36672 *chip_info = (struct chip_data_nt36672 *)chip_data;

    TPD_INFO("%s is called\n", __func__);

    if (!chip_info) {
        TPD_INFO("Chip info is NULL\n");
        return FW_NO_NEED_UPDATE;
    }

    if (fw->size != FW_BIN_SIZE) {
        TPD_INFO("bin file size not match, size is %lu\n", fw->size);
        return FW_NO_NEED_UPDATE;
    }

    if (*(fw->data + FW_BIN_VER_OFFSET) + *(fw->data + FW_BIN_VER_BAR_OFFSET) != 0xFF) {
        TPD_INFO("bin file FW_VER(0x%02x) + FW_VER_BAR(0x%02x) should be 0xFF\n", *(fw->data+FW_BIN_VER_OFFSET), *(fw->data+FW_BIN_VER_BAR_OFFSET));
        return FW_NO_NEED_UPDATE;
    }

    nvt_sw_reset_idle(chip_info);
    if (!force) {
        TPD_INFO("no force update!\n");
        ret = Check_CheckSum(chip_info, fw);//-1:error, 0:not match, 1:match
        if (((ret == 0) && (Check_FW_Ver(chip_info, fw) == 1)) || (ret == 1)) {
            TPD_INFO("no need update!\n");
            goto OUT;
        }
    }

    // Step 1: stop crc check
    nvt_stop_crc_reboot(chip_data);

    // Step 2 : initial bootloader
    TPD_INFO("Init BootLoader start\n");
    ret = Init_BootLoader(chip_info);
    if (ret) {
        TPD_INFO("Init BootLoader failed, exit!\n");
        update_state = FW_UPDATE_ERROR;
        goto OUT;
    }

    // Step 3 : Resume PD
    TPD_INFO("Resume PD start\n");
    ret = Resume_PD(chip_info);
    if (ret) {
        TPD_INFO("Resume PD failed, exit!\n");
        update_state = FW_UPDATE_ERROR;
        goto OUT;
    }

    // Step 4 : Erase
    TPD_INFO("Erase Flash start\n");
    ret = Erase_Flash(chip_info, fw->size);
    if (ret) {
        TPD_INFO("Erase Flash failed, exit!\n");
        update_state = FW_UPDATE_ERROR;
        goto OUT;
    }

    // Step 5 : Program
    TPD_INFO("Write Flash start\n");
    ret = Write_Flash(chip_info, fw);
    if (ret) {
        TPD_INFO("Write Flash failed, exit!\n");
        update_state = FW_UPDATE_ERROR;
        goto OUT;
    }

    // Step  6: Verify
    TPD_INFO("Verify Flash start\n");
    ret = Verify_Flash(chip_info, fw);
    if (ret) {
        TPD_INFO("Verify Flash failed, exit!\n");
        update_state = FW_UPDATE_ERROR;
        goto OUT;
    }

    update_state = FW_UPDATE_SUCCESS;

OUT:
    //Step 7 : Bootloader Reset
    nvt_bootloader_reset(chip_info);
    nvt_check_fw_reset_state(chip_info, RESET_STATE_INIT);

    TPD_INFO("update firmware state: %s\n", (update_state == FW_UPDATE_SUCCESS)? "update success!":\
                                            (update_state == FW_UPDATE_ERROR)? "update failed!": "no need update!");
    return update_state;
}

static u8 nvt_trigger_reason(void *chip_data, int gesture_enable, int is_suspended)
{
    //struct chip_data_nt36672 *chip_info = (struct chip_data_nt36672 *)chip_data;

    if ((gesture_enable == 1) && is_suspended) {
        return IRQ_GESTURE;
    } else if (is_suspended) {
        return IRQ_IGNORE;
    }

    return IRQ_TOUCH;
}

static int nvt_get_touch_points(void *chip_data, struct point_info *points, int max_num)
{
    int obj_attention = 0;
    uint8_t pointid = 0;//range from 1 to 10
    uint8_t point_state = 0;
    uint32_t position = 0;
    uint32_t input_x, input_y, input_w, input_p;
    int ret = 0;
    int i = 0;
    struct chip_data_nt36672 *chip_info = (struct chip_data_nt36672 *)chip_data;
    uint8_t buf[POINT_DATA_LEN + 1] = {0};

    memset(buf, 0, sizeof(buf));
    ret = nvt_i2c_read(chip_info->client, I2C_FW_Address, buf, POINT_DATA_LEN + 1);
    if (ret < 0) {
        TPD_INFO("read touch data failed\n");
        return -1;
    }

    for(i = 0; i < max_num; i++) {
        position = 1 + 6 * i;
        pointid = (uint8_t)(buf[position + 0] >> 3) - 1;
        if (pointid >= max_num) {
            continue;
        }

        point_state = buf[position] & 0x07;
        if ((point_state == 0x01) || (point_state == 0x02)) {
            input_x = (uint32_t)(buf[position + 1] << 4) + (uint32_t) (buf[position + 3] >> 4);
            input_y = (uint32_t)(buf[position + 2] << 4) + (uint32_t) (buf[position + 3] & 0x0F);
            input_w = (uint32_t)(buf[position + 4]);
            if (input_w == 0) {
                input_w = 1;
            }
            if (i < 2) {
                input_p = (uint32_t)(buf[position + 5]) + (uint32_t)(buf[i + 63] << 8);
                if (input_p > 1000) {
                    input_p = 1000;
                }
            } else {
                input_p = (uint32_t)(buf[position + 5]);
            }
            if (input_p == 0) {
                input_p = 1;
            }

            obj_attention = obj_attention | (1 << pointid);
            points[pointid].x = input_x;
            points[pointid].y = input_y;
            points[pointid].z = input_p;
            points[pointid].width_major = input_w;
            points[pointid].status = 1;
        }
    }

    return obj_attention;
}

static int nvt_get_gesture_info(void *chip_data, struct gesture_info * gesture)
{
    uint8_t gesture_id = 0;
    uint8_t func_type = 0;
    int ret = -1;
    struct chip_data_nt36672 *chip_info = (struct chip_data_nt36672 *)chip_data;
    uint8_t point_data[POINT_DATA_LEN + 1] = {0};

    memset(point_data, 0, sizeof(point_data));
    ret = nvt_i2c_read(chip_info->client, I2C_FW_Address, point_data, POINT_DATA_LEN + 1);
    if (ret < 0) {
        TPD_INFO("%s: read gesture data failed\n", __func__);
        return -1;
    }

    gesture_id = (uint8_t)(point_data[1] >> 3);
    func_type = (uint8_t)point_data[2];
    if ((gesture_id == 30) && (func_type == 1)) {
        gesture_id = (uint8_t)point_data[3];
    } else if (gesture_id > 30) {
        TPD_INFO("invalid gesture id= %d, no gesture event\n", gesture_id);
        return 0;
    }
    switch (gesture_id)     //judge gesture type
    {
        case RIGHT_SLIDE_DETECT :
            gesture->gesture_type  = Left2RightSwip;
            gesture->Point_start.x = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
            gesture->Point_start.y = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
            gesture->Point_end.x   = (point_data[8] & 0xFF) | (point_data[9] & 0x0F) << 8;
            gesture->Point_end.y   = (point_data[10] & 0xFF) | (point_data[11] & 0x0F) << 8;
            break;

        case LEFT_SLIDE_DETECT :
            gesture->gesture_type  = Right2LeftSwip;
            gesture->Point_start.x = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
            gesture->Point_start.y = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
            gesture->Point_end.x   = (point_data[8] & 0xFF) | (point_data[9] & 0x0F) << 8;
            gesture->Point_end.y   = (point_data[10] & 0xFF) | (point_data[11] & 0x0F) << 8;
            break;

        case DOWN_SLIDE_DETECT  :
            gesture->gesture_type  = Up2DownSwip;
            gesture->Point_start.x = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
            gesture->Point_start.y = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
            gesture->Point_end.x   = (point_data[8] & 0xFF) | (point_data[9] & 0x0F) << 8;
            gesture->Point_end.y   = (point_data[10] & 0xFF) | (point_data[11] & 0x0F) << 8;
            break;

        case UP_SLIDE_DETECT :
            gesture->gesture_type  = Down2UpSwip;
            gesture->Point_start.x = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
            gesture->Point_start.y = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
            gesture->Point_end.x   = (point_data[8] & 0xFF) | (point_data[9] & 0x0F) << 8;
            gesture->Point_end.y   = (point_data[10] & 0xFF) | (point_data[11] & 0x0F) << 8;
            break;

        case DTAP_DETECT:
            gesture->gesture_type  = DouTap;
            gesture->Point_start.x = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
            gesture->Point_start.y = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
            gesture->Point_end     = gesture->Point_start;
            break;

        case UP_VEE_DETECT :
            gesture->gesture_type  = UpVee;
            gesture->Point_start.x = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
            gesture->Point_start.y = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
            gesture->Point_end.x   = (point_data[12] & 0xFF) | (point_data[13] & 0x0F) << 8;
            gesture->Point_end.y   = (point_data[14] & 0xFF) | (point_data[15] & 0x0F) << 8;
            gesture->Point_1st.x   = (point_data[8] & 0xFF) | (point_data[9] & 0x0F) << 8;
            gesture->Point_1st.y   = (point_data[10] & 0xFF) | (point_data[11] & 0x0F) << 8;
            break;

        case DOWN_VEE_DETECT :
            gesture->gesture_type  = DownVee;
            gesture->Point_start.x = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
            gesture->Point_start.y = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
            gesture->Point_end.x   = (point_data[12] & 0xFF) | (point_data[13] & 0x0F) << 8;
            gesture->Point_end.y   = (point_data[14] & 0xFF) | (point_data[15] & 0x0F) << 8;
            gesture->Point_1st.x   = (point_data[8] & 0xFF) | (point_data[9] & 0x0F) << 8;
            gesture->Point_1st.y   = (point_data[10] & 0xFF) | (point_data[11] & 0x0F) << 8;
            break;

        case LEFT_VEE_DETECT:
            gesture->gesture_type = LeftVee;
            gesture->Point_start.x = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
            gesture->Point_start.y = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
            gesture->Point_end.x   = (point_data[12] & 0xFF) | (point_data[13] & 0x0F) << 8;
            gesture->Point_end.y   = (point_data[14] & 0xFF) | (point_data[15] & 0x0F) << 8;
            gesture->Point_1st.x   = (point_data[8] & 0xFF) | (point_data[9] & 0x0F) << 8;
            gesture->Point_1st.y   = (point_data[10] & 0xFF) | (point_data[11] & 0x0F) << 8;
            break;

        case RIGHT_VEE_DETECT :
            gesture->gesture_type  = RightVee;
            gesture->Point_start.x = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
            gesture->Point_start.y = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
            gesture->Point_end.x   = (point_data[12] & 0xFF) | (point_data[13] & 0x0F) << 8;
            gesture->Point_end.y   = (point_data[14] & 0xFF) | (point_data[15] & 0x0F) << 8;
            gesture->Point_1st.x   = (point_data[8] & 0xFF) | (point_data[9] & 0x0F) << 8;
            gesture->Point_1st.y   = (point_data[10] & 0xFF) | (point_data[11] & 0x0F) << 8;
            break;

        case CIRCLE_DETECT  :
            gesture->gesture_type = Circle;
            gesture->clockwise = (point_data[43] == 0x20) ? 1 : 0;
            gesture->Point_start.x = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
            gesture->Point_start.y = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
            gesture->Point_1st.x   = (point_data[8] & 0xFF) | (point_data[9] & 0x0F) << 8;    //ymin
            gesture->Point_1st.y   = (point_data[10] & 0xFF) | (point_data[11] & 0x0F) << 8;
            gesture->Point_2nd.x   = (point_data[12] & 0xFF) | (point_data[13] & 0x0F) << 8;  //xmin
            gesture->Point_2nd.y   = (point_data[14] & 0xFF) | (point_data[15] & 0x0F) << 8;
            gesture->Point_3rd.x   = (point_data[16] & 0xFF) | (point_data[17] & 0x0F) << 8;  //ymax
            gesture->Point_3rd.y   = (point_data[18] & 0xFF) | (point_data[19] & 0x0F) << 8;
            gesture->Point_4th.x   = (point_data[20] & 0xFF) | (point_data[21] & 0x0F) << 8;  //xmax
            gesture->Point_4th.y   = (point_data[22] & 0xFF) | (point_data[23] & 0x0F) << 8;
            gesture->Point_end.x   = (point_data[24] & 0xFF) | (point_data[25] & 0x0F) << 8;
            gesture->Point_end.y   = (point_data[26] & 0xFF) | (point_data[27] & 0x0F) << 8;
            break;

        case DOUSWIP_DETECT  :
            gesture->gesture_type  = DouSwip;
            gesture->Point_start.x = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
            gesture->Point_start.y = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
            gesture->Point_end.x   = (point_data[12] & 0xFF) | (point_data[13] & 0x0F) << 8;
            gesture->Point_end.y   = (point_data[14] & 0xFF) | (point_data[15] & 0x0F) << 8;
            gesture->Point_1st.x   = (point_data[8] & 0xFF) | (point_data[9] & 0x0F) << 8;
            gesture->Point_1st.y   = (point_data[10] & 0xFF) | (point_data[11] & 0x0F) << 8;
            gesture->Point_2nd.x   = (point_data[16] & 0xFF) | (point_data[17] & 0x0F) << 8;
            gesture->Point_2nd.y   = (point_data[18] & 0xFF) | (point_data[19] & 0x0F) << 8;
            break;

        case M_DETECT  :
            gesture->gesture_type  = Mgestrue;
            gesture->Point_start.x = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
            gesture->Point_start.y = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
            gesture->Point_1st.x   = (point_data[8] & 0xFF) | (point_data[9] & 0x0F) << 8;
            gesture->Point_1st.y   = (point_data[10] & 0xFF) | (point_data[11] & 0x0F) << 8;
            gesture->Point_2nd.x   = (point_data[12] & 0xFF) | (point_data[13] & 0x0F) << 8;
            gesture->Point_2nd.y   = (point_data[14] & 0xFF) | (point_data[15] & 0x0F) << 8;
            gesture->Point_3rd.x   = (point_data[16] & 0xFF) | (point_data[17] & 0x0F) << 8;
            gesture->Point_3rd.y   = (point_data[18] & 0xFF) | (point_data[19] & 0x0F) << 8;
            gesture->Point_end.x   = (point_data[20] & 0xFF) | (point_data[21] & 0x0F) << 8;
            gesture->Point_end.y   = (point_data[22] & 0xFF) | (point_data[23] & 0x0F) << 8;
            break;

        case W_DETECT :
            gesture->gesture_type  = Wgestrue;
            gesture->Point_start.x = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
            gesture->Point_start.y = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
            gesture->Point_1st.x   = (point_data[8] & 0xFF) | (point_data[9] & 0x0F) << 8;
            gesture->Point_1st.y   = (point_data[10] & 0xFF) | (point_data[11] & 0x0F) << 8;
            gesture->Point_2nd.x   = (point_data[12] & 0xFF) | (point_data[13] & 0x0F) << 8;
            gesture->Point_2nd.y   = (point_data[14] & 0xFF) | (point_data[15] & 0x0F) << 8;
            gesture->Point_3rd.x   = (point_data[16] & 0xFF) | (point_data[17] & 0x0F) << 8;
            gesture->Point_3rd.y   = (point_data[18] & 0xFF) | (point_data[19] & 0x0F) << 8;
            gesture->Point_end.x   = (point_data[20] & 0xFF) | (point_data[21] & 0x0F) << 8;
            gesture->Point_end.y   = (point_data[22] & 0xFF) | (point_data[23] & 0x0F) << 8;
            break;

        default:
            gesture->gesture_type = UnkownGesture;
            break;
    }

    TPD_INFO("%s, gesture_id: 0x%x, func_type: 0x%x, gesture_type: %d, clockwise: %d, points: (%d, %d)(%d, %d)(%d, %d)(%d, %d)(%d, %d)(%d, %d)\n", \
                __func__, gesture_id, func_type, gesture->gesture_type, gesture->clockwise, \
                gesture->Point_start.x, gesture->Point_start.y, \
                gesture->Point_end.x, gesture->Point_end.y, \
                gesture->Point_1st.x, gesture->Point_1st.y, \
                gesture->Point_2nd.x, gesture->Point_2nd.y, \
                gesture->Point_3rd.x, gesture->Point_3rd.y, \
                gesture->Point_4th.x, gesture->Point_4th.y);

    return 0;
}

static int nvt_enable_jitter_mode(struct chip_data_nt36672 *chip_info, bool enable)
{
    int8_t ret = -1;

    TPD_INFO("%s:enable = %d, chip_info->is_sleep_writed = %d\n", __func__, enable, chip_info->is_sleep_writed);

    if (chip_info->is_sleep_writed) {
        nvt_reset(chip_info);
    }

    if (enable) {
        ret = nvt_cmd_store(chip_info, EVENTBUFFER_JITTER_ON);
    } else {
        ret = nvt_cmd_store(chip_info, EVENTBUFFER_JITTER_OFF);
    }

    return ret;
}

static int nvt_mode_switch(void *chip_data, work_mode mode, bool flag)
{
    int ret = -1;
    struct chip_data_nt36672 *chip_info = (struct chip_data_nt36672 *)chip_data;

    switch(mode) {
        case MODE_NORMAL:
            ret = 0;
            break;

        case MODE_SLEEP:
            ret = nvt_enter_sleep(chip_info, true);
            if (ret < 0) {
                TPD_INFO("%s: nvt enter sleep failed\n", __func__);
            }
            break;

        case MODE_GESTURE:
            ret = nvt_enable_black_gesture(chip_info, flag);
            if (ret < 0) {
                TPD_INFO("%s: nvt enable gesture failed.\n", __func__);
                return ret;
            }
            break;

        case MODE_EDGE:
            ret = nvt_enable_edge_limit(chip_info, flag);
            if (ret < 0) {
                TPD_INFO("%s: nvt enable edg limit failed.\n", __func__);
                return ret;
            }
            break;

        case MODE_CHARGE:
            ret = nvt_enable_charge_mode(chip_info, flag);
            if (ret < 0) {
                TPD_INFO("%s: enable charge mode : %d failed\n", __func__, flag);
            }
            break;

        case MODE_GAME:
            ret = nvt_enable_jitter_mode(chip_info, flag);
            break;

        default:
            TPD_INFO("%s: Wrong mode.\n", __func__);
    }

    return ret;
}

#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
extern unsigned int upmu_get_rgs_chrdet(void);
static int nvt_get_usb_state(void)
{
    return upmu_get_rgs_chrdet();
}
#else
#endif

static void nvt_black_screen_test(void *chip_data, char *message)
{
    int32_t *raw_data = NULL, *raw_data_n = NULL;
    struct chip_data_nt36672 *chip_info = (struct chip_data_nt36672 *)chip_data;
    int tx_num = chip_info->hw_res->TX_NUM;
    int rx_num = chip_info->hw_res->RX_NUM;
    const struct firmware *fw = NULL;
    struct nvt_test_header *ph = NULL;
    int i, j, iArrayIndex, err_cnt = 0;
    int fd = -1, ret = -1;
    mm_segment_t old_fs;
    char buf[128] = {0};
    uint8_t data_buf[128];
    int32_t buf_len = 0;
    struct timespec now_time;
    struct rtc_time rtc_now_time;
    int32_t *lpwg_rawdata_P = NULL, *lpwg_rawdata_N = NULL;
    int32_t *lpwg_diff_rawdata_P = NULL, *lpwg_diff_rawdata_N = NULL;

    if (nvt_switch_FreqHopEnDis(chip_info, FREQ_HOP_DISABLE)) {
        TPD_INFO("switch frequency hopping disable failed!\n");
        sprintf(message, "1 error, switch frequency hopping disable failed!\n");
        return;
    }

    if (nvt_check_fw_reset_state(chip_info, RESET_STATE_NORMAL_RUN)) {
        TPD_INFO("check fw reset state failed!\n");
        sprintf(message, "1 error, check fw reset state failed!\n");
        return;
    }

    msleep(100);

    //---Enter Test Mode---
    if (nvt_clear_fw_status(chip_info)) {
        TPD_INFO("clear fw status failed!\n");
        sprintf(message, "1 error, clear fw status failed!\n");
        return;
    }

    nvt_change_mode(chip_info, TEST_MODE_2);

    if (nvt_check_fw_status(chip_info)) {
        TPD_INFO("check fw status failed!\n");
        sprintf(message, "1 error, check fw status failed!\n");
        return;
    }

    if (nvt_get_fw_info(chip_info)) {
        TPD_INFO("get fw info failed!\n");
        sprintf(message, "1 error, get fw info failed!\n");
        return;
    }

    TPD_INFO("malloc raw_data space\n");
    buf_len = tx_num * rx_num * sizeof(int32_t);
    raw_data = kzalloc(buf_len, GFP_KERNEL);
    raw_data_n = kzalloc(buf_len, GFP_KERNEL);
    if (!(raw_data && raw_data_n)) {
        err_cnt++;
        sprintf(buf, "kzalloc space failed\n");
        goto KFREE_OUT;
    }

    ret = request_firmware(&fw, chip_info->test_limit_name, &chip_info->client->dev);
    if (ret < 0) {
        err_cnt++;
        sprintf(buf, "Request firmware failed: %s\n", chip_info->test_limit_name);
        goto KFREE_OUT;
    }
    ph = (struct nvt_test_header *)(fw->data);

    //create a file to store test data in /sdcard/ScreenOffTpTestReport
    getnstimeofday(&now_time);
    rtc_time_to_tm(now_time.tv_sec, &rtc_now_time);
    snprintf(data_buf, 128, "/sdcard/TpTestReport/screenOff/tp_testlimit_%02d%02d%02d-%02d%02d%02d-utc.csv",
            (rtc_now_time.tm_year + 1900) % 100, rtc_now_time.tm_mon + 1, rtc_now_time.tm_mday,
            rtc_now_time.tm_hour, rtc_now_time.tm_min, rtc_now_time.tm_sec);
    old_fs = get_fs();
    set_fs(KERNEL_DS);
    sys_mkdir("/sdcard/TpTestReport/screenOff", 0666);
    fd = sys_open(data_buf, O_WRONLY | O_CREAT | O_TRUNC, 0);
    if (fd < 0) {
        TPD_INFO("Open log file '%s' failed.\n", data_buf);
        err_cnt++;
        sprintf(buf, "Open log file '%s' failed.\n", data_buf);
        goto OUT;
    }

    lpwg_rawdata_P = (int32_t *)(fw->data + ph->array_LPWG_Rawdata_P_offset);
    lpwg_rawdata_N = (int32_t *)(fw->data + ph->array_LPWG_Rawdata_N_offset);
    lpwg_diff_rawdata_P = (int32_t *)(fw->data + ph->array_LPWG_Diff_P_offset);
    lpwg_diff_rawdata_N = (int32_t *)(fw->data + ph->array_LPWG_Diff_N_offset);

    //---FW Rawdata Test---
    TPD_INFO("LPWG mode FW Rawdata Test \n");
    memset(raw_data, 0, buf_len);
    if (nvt_get_fw_pipe(chip_info) == 0)
        nvt_read_mdata(chip_info, chip_info->mmap->RAW_PIPE0_ADDR, raw_data, buf_len);
    else
        nvt_read_mdata(chip_info, chip_info->mmap->RAW_PIPE1_ADDR, raw_data, buf_len);
    store_to_file(fd, "LPWG mode FW Rawdata:\n");
    if ((ph->config_Lmt_LPWG_Rawdata_P != 0) && (ph->config_Lmt_LPWG_Rawdata_N != 0)) {
        for (j = 0; j < rx_num; j++) {
            for (i = 0; i < tx_num; i++) {
                iArrayIndex = j * tx_num + i;
                TPD_DEBUG_NTAG("%d, ", raw_data[iArrayIndex]);
                if (fd >= 0) {
                    store_to_file(fd, "%d, ", raw_data[iArrayIndex]);
                }
                if((raw_data[iArrayIndex] > ph->config_Lmt_LPWG_Rawdata_P) \
                        || (raw_data[iArrayIndex] < ph->config_Lmt_LPWG_Rawdata_N)) {
                    TPD_INFO("LPWG_Rawdata Test failed at rawdata[%d][%d] = %d\n", i, j, raw_data[iArrayIndex]);
                    if (!err_cnt) {
                        sprintf(buf, "LPWG Rawdata[%d][%d] = %d[%d %d]\n",
                            i, j, raw_data[iArrayIndex], ph->config_Lmt_LPWG_Rawdata_N, ph->config_Lmt_LPWG_Rawdata_P);
                    }
                    err_cnt++;
                }
            }
            if (fd >= 0) {
                store_to_file(fd, "\n");
            }
            TPD_DEBUG_NTAG("\n");
        }
    } else {
        for (j = 0; j < rx_num; j++) {
            for (i = 0; i < tx_num; i++) {
                iArrayIndex = j * tx_num + i;
                TPD_DEBUG_NTAG("%d, ", raw_data[iArrayIndex]);
                if (fd >= 0) {
                    store_to_file(fd, "%d, ", raw_data[iArrayIndex]);
                }
                if((raw_data[iArrayIndex] > lpwg_rawdata_P[iArrayIndex]) \
                        || (raw_data[iArrayIndex] < lpwg_rawdata_N[iArrayIndex])) {
                    TPD_INFO("LPWG_Rawdata Test failed at rawdata[%d][%d] = %d\n", i, j, raw_data[iArrayIndex]);
                    if (!err_cnt) {
                        sprintf(buf, "LPWG Rawdata[%d][%d] = %d[%d %d]\n",
                            i, j, raw_data[iArrayIndex], lpwg_rawdata_N[iArrayIndex], lpwg_rawdata_P[iArrayIndex]);
                    }
                    err_cnt++;
                }
            }
            if (fd >= 0) {
                store_to_file(fd, "\n");
            }
            TPD_DEBUG_NTAG("\n");
        }
    }

    //---Leave Test Mode---
    nvt_change_mode(chip_info, NORMAL_MODE);

    //---Noise Test---
    TPD_INFO("LPWG mode FW Noise Test \n");
    memset(raw_data, 0, buf_len);  //store max
    memset(raw_data_n, 0, buf_len); //store min
    if (nvt_read_fw_noise(chip_info, ph->config_Diff_Test_Frame, raw_data, raw_data_n, buf_len) != 0) {
        TPD_INFO("LPWG mode read Noise data failed!\n");    // 1: ERROR
        sprintf(buf, "LPWG mode read Noise data failed!\n");
        err_cnt++;
        goto OUT;
    }
    TPD_INFO("LPWG Noise RawData_Diff_Max:\n");
    store_to_file(fd, "LPWG Noise RawData_Diff_Max:\n");
    if ((ph->config_Lmt_LPWG_Diff_P != 0) && (ph->config_Lmt_LPWG_Diff_N != 0)) {
        for (j = 0; j < rx_num; j++) {
            for (i = 0; i < tx_num; i++) {
                iArrayIndex = j * tx_num + i;
                TPD_DEBUG_NTAG("%d, ", raw_data[iArrayIndex]);
                if (fd >= 0) {
                    store_to_file(fd, "%d, ", raw_data[iArrayIndex]);
                }
                if((raw_data[iArrayIndex] > ph->config_Lmt_LPWG_Diff_P) \
                        || (raw_data[iArrayIndex] < ph->config_Lmt_LPWG_Diff_N)) {
                    TPD_INFO("LPWG Noise RawData_Diff_Max Test failed at rawdata[%d][%d] = %d\n", i, j, raw_data[iArrayIndex]);
                    if (!err_cnt) {
                        sprintf(buf, "LPWG Noise RawData_Diff_Max[%d][%d] = %d[%d %d]\n",
                            i, j, raw_data[iArrayIndex], ph->config_Lmt_LPWG_Diff_N, ph->config_Lmt_LPWG_Diff_P);
                    }
                    err_cnt++;
                }
            }
            if (fd >= 0) {
                store_to_file(fd, "\n");
            }
            TPD_DEBUG_NTAG("\n");
        }
        TPD_INFO("LPWG Noise RawData_Diff_Min:\n");
        store_to_file(fd, "LPWG Noise RawData_Diff_Min:\n");
        for (j = 0; j < rx_num; j++) {
            for (i = 0; i < tx_num; i++) {
                iArrayIndex = j * tx_num + i;
                TPD_DEBUG_NTAG("%d, ", raw_data_n[iArrayIndex]);
                if (fd >= 0) {
                    store_to_file(fd, "%d, ", raw_data_n[iArrayIndex]);
                }
                if((raw_data_n[iArrayIndex] > ph->config_Lmt_LPWG_Diff_P) \
                        || (raw_data_n[iArrayIndex] < ph->config_Lmt_LPWG_Diff_N)) {
                    TPD_INFO("LPWG Noise RawData_Diff_Min Test failed at rawdata[%d][%d] = %d\n", i, j, raw_data_n[iArrayIndex]);
                    if (!err_cnt) {
                        sprintf(buf, "LPWG Noise RawData_Diff_Min[%d][%d] = %d[%d %d]\n",
                            i, j, raw_data_n[iArrayIndex], ph->config_Lmt_LPWG_Diff_N, ph->config_Lmt_LPWG_Diff_P);
                    }
                    err_cnt++;
                }
            }
            if (fd >= 0) {
                store_to_file(fd, "\n");
            }
            TPD_DEBUG_NTAG("\n");
        }
    } else {
        for (j = 0; j < rx_num; j++) {
            for (i = 0; i < tx_num; i++) {
                iArrayIndex = j * tx_num + i;
                TPD_DEBUG_NTAG("%d, ", raw_data[iArrayIndex]);
                if (fd >= 0) {
                    store_to_file(fd, "%d, ", raw_data[iArrayIndex]);
                }
                if((raw_data[iArrayIndex] > lpwg_diff_rawdata_P[iArrayIndex]) \
                        || (raw_data[iArrayIndex] < lpwg_diff_rawdata_N[iArrayIndex])) {
                    TPD_INFO("LPWG Noise RawData_Diff_Max Test failed at rawdata[%d][%d] = %d\n", i, j, raw_data[iArrayIndex]);
                    if (!err_cnt) {
                        sprintf(buf, "LPWG Noise RawData_Diff_Max[%d][%d] = %d[%d %d]\n",
                            i, j, raw_data[iArrayIndex], lpwg_diff_rawdata_N[iArrayIndex], lpwg_diff_rawdata_P[iArrayIndex]);
                    }
                    err_cnt++;
                }
            }
            if (fd >= 0) {
                store_to_file(fd, "\n");
            }
            TPD_DEBUG_NTAG("\n");
        }
        TPD_INFO("LPWG Noise RawData_Diff_Min:\n");
        store_to_file(fd, "LPWG Noise RawData_Diff_Min:\n");
        for (j = 0; j < rx_num; j++) {
            for (i = 0; i < tx_num; i++) {
                iArrayIndex = j * tx_num + i;
                TPD_DEBUG_NTAG("%d, ", raw_data_n[iArrayIndex]);
                if (fd >= 0) {
                    store_to_file(fd, "%d, ", raw_data_n[iArrayIndex]);
                }
                if((raw_data_n[iArrayIndex] > lpwg_diff_rawdata_P[iArrayIndex]) \
                        || (raw_data_n[iArrayIndex] < lpwg_diff_rawdata_N[iArrayIndex])) {
                    TPD_INFO("LPWG Noise RawData_Diff_Min Test failed at rawdata[%d][%d] = %d\n", i, j, raw_data_n[iArrayIndex]);
                    if (!err_cnt) {
                        sprintf(buf, "LPWG Noise RawData_Diff_Min[%d][%d] = %d[%d %d]\n",
                            i, j, raw_data_n[iArrayIndex], lpwg_diff_rawdata_N[iArrayIndex], lpwg_diff_rawdata_P[iArrayIndex]);
                    }
                    err_cnt++;
                }
            }
            if (fd >= 0) {
                store_to_file(fd, "\n");
            }
            TPD_DEBUG_NTAG("\n");
        }
    }

OUT:
    if (fd >= 0) {
        sys_close(fd);
    }
    set_fs(old_fs);
    release_firmware(fw);

KFREE_OUT:
    if (raw_data)
        kfree(raw_data);
    if (raw_data_n)
        kfree(raw_data_n);
    sprintf(message, "%d errors. %s", err_cnt, buf);
    TPD_INFO("%d errors. %s\n", err_cnt, buf);
}

static struct oppo_touchpanel_operations nvt_ops = {
    .ftm_process                = nvt_ftm_process,
    .get_vendor                 = nvt_get_vendor,
    .get_chip_info              = nvt_get_chip_info,
    .reset                      = nvt_reset,
    .power_control              = nvt_power_control,
    .fw_check                   = nvt_fw_check,
    .fw_update                  = nvt_fw_update,
    .trigger_reason             = nvt_trigger_reason,
    .get_touch_points           = nvt_get_touch_points,
    .get_gesture_info           = nvt_get_gesture_info,
    .mode_switch                = nvt_mode_switch,
    .get_usb_state              = nvt_get_usb_state,
    .black_screen_test          = nvt_black_screen_test,
    .reset_gpio_control         = nvt_reset_gpio_control,
};
/********* End of implementation of oppo_touchpanel_operations callbacks**********************/

/************** Start of function work for debug_info proc callbacks**************************/

static int32_t nvt_clear_fw_status(struct chip_data_nt36672 *chip_info)
{
    uint8_t buf[8] = {0};
    int32_t i = 0;
    const int32_t retry = 20;

    for (i = 0; i < retry; i++) {
        //---set xdata index to EVENT BUF ADDR---
        buf[0] = 0xFF;
        buf[1] = (chip_info->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
        buf[2] = (chip_info->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
        nvt_i2c_write(chip_info->client, I2C_FW_Address, buf, 3);

        //---clear fw status---
        buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
        buf[1] = 0x00;
        nvt_i2c_write(chip_info->client, I2C_FW_Address, buf, 2);

        //---read fw status---
        buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
        buf[1] = 0xFF;
        nvt_i2c_read(chip_info->client, I2C_FW_Address, buf, 2);

        if (buf[1] == 0x00)
            break;

        msleep(10);
    }

    if (i >= retry) {
        TPD_INFO("%s failed, i=%d, buf[1]=0x%02X\n", __func__, i, buf[1]);
        return -1;
    } else {
        return 0;
    }
}

static void nvt_change_mode(struct chip_data_nt36672 *chip_info, uint8_t mode)
{
    uint8_t buf[8] = {0};

    //---set xdata index to EVENT BUF ADDR---
    buf[0] = 0xFF;
    buf[1] = (chip_info->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
    buf[2] = (chip_info->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
    nvt_i2c_write(chip_info->client, I2C_FW_Address, buf, 3);

    //---set mode---
    buf[0] = EVENT_MAP_HOST_CMD;
    buf[1] = mode;
    nvt_i2c_write(chip_info->client, I2C_FW_Address, buf, 2);

    if (mode == NORMAL_MODE) {
        buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
        buf[1] = HANDSHAKING_HOST_READY;
        nvt_i2c_write(chip_info->client, I2C_FW_Address, buf, 2);
        msleep(20);
    }
}

static int32_t nvt_check_fw_status(struct chip_data_nt36672 *chip_info)
{
    uint8_t buf[8] = {0};
    int32_t i = 0;
    int ret = -1;
    const int32_t retry = 50;

    for (i = 0; i < retry; i++) {
        //---set xdata index to EVENT BUF ADDR---
        buf[0] = 0xFF;
        buf[1] = (chip_info->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
        buf[2] = (chip_info->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
        ret = nvt_i2c_write(chip_info->client, I2C_FW_Address, buf, 3);

        //---read fw status---
        buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
        buf[1] = 0x00;
        ret |= nvt_i2c_read(chip_info->client, I2C_FW_Address, buf, 2);
        if (ret < 0) {
            TPD_INFO("%s write or read failed\n", __func__);
        }

        if ((buf[1] & 0xF0) == 0xA0)
            break;

        msleep(10);
    }

    if (i >= retry) {
        TPD_INFO("%s failed, i=%d, buf[1]=0x%02X\n", __func__, i, buf[1]);
        return -1;
    } else {
        return 0;
    }
}

static uint8_t nvt_get_fw_pipe(struct chip_data_nt36672 *chip_info)
{
    int ret = -1;
    uint8_t buf[8]= {0};

    //---set xdata index to EVENT BUF ADDR---
    buf[0] = 0xFF;
    buf[1] = (chip_info->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
    buf[2] = (chip_info->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
    ret = nvt_i2c_write(chip_info->client, I2C_FW_Address, buf, 3);

    //---read fw status---
    buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
    buf[1] = 0x00;
    ret |= nvt_i2c_read(chip_info->client, I2C_FW_Address, buf, 2);
    if (ret < 0) {
        TPD_INFO("%s: read or write failed\n", __func__);
    }

    return (buf[1] & 0x01);
}

void nvt_read_mdata(struct chip_data_nt36672 *chip_info, uint32_t xdata_addr, int32_t *xdata, int32_t xdata_len)
{
    int32_t i = 0;
    int32_t j = 0;
    int32_t k = 0;
    uint8_t buf[I2C_TANSFER_LENGTH + 1] = {0};
    uint32_t head_addr = 0;
    int32_t dummy_len = 0;
    int32_t data_len = 0;
    int32_t residual_len = 0;
    uint8_t *xdata_tmp = NULL;

    //---set xdata sector address & length---
    head_addr = xdata_addr - (xdata_addr % XDATA_SECTOR_SIZE);
    dummy_len = xdata_addr - head_addr;
    data_len = chip_info->hw_res->TX_NUM * chip_info->hw_res->RX_NUM * 2;
    residual_len = (head_addr + dummy_len + data_len) % XDATA_SECTOR_SIZE;

    if (xdata_len/sizeof(int32_t) < data_len/2) {
        TPD_INFO("xdata read buffer(%d) less than max data size(%d), return\n", xdata_len, data_len);
        return;
    }

    //malloc buffer space
    xdata_tmp = kzalloc(2048 ,GFP_KERNEL);
    if (xdata_tmp == NULL) {
        TPD_INFO("%s malloc memory failed\n", __func__);
        return;
    }
    //printk("head_addr=0x%05X, dummy_len=0x%05X, data_len=0x%05X, residual_len=0x%05X\n", head_addr, dummy_len, data_len, residual_len);

    //read xdata : step 1
    for (i = 0; i < ((dummy_len + data_len) / XDATA_SECTOR_SIZE); i++) {
        //---change xdata index---
        buf[0] = 0xFF;
        buf[1] = ((head_addr + XDATA_SECTOR_SIZE * i) >> 16) & 0xFF;
        buf[2] = ((head_addr + XDATA_SECTOR_SIZE * i) >> 8) & 0xFF;
        nvt_i2c_write(chip_info->client, I2C_FW_Address, buf, 3);

        //---read xdata by I2C_TANSFER_LENGTH
        for (j = 0; j < (XDATA_SECTOR_SIZE / I2C_TANSFER_LENGTH); j++) {
            //---read data---
            buf[0] = I2C_TANSFER_LENGTH * j;
            nvt_i2c_read(chip_info->client, I2C_FW_Address, buf, I2C_TANSFER_LENGTH + 1);

            //---copy buf to xdata_tmp---
            for (k = 0; k < I2C_TANSFER_LENGTH; k++) {
                xdata_tmp[XDATA_SECTOR_SIZE * i + I2C_TANSFER_LENGTH * j + k] = buf[k + 1];
                //printk("0x%02X, 0x%04X\n", buf[k+1], (XDATA_SECTOR_SIZE*i + I2C_TANSFER_LENGTH*j + k));
            }
        }
        //printk("addr=0x%05X\n", (head_addr+XDATA_SECTOR_SIZE*i));
    }

    //read xdata : step2
    if (residual_len != 0) {
        //---change xdata index---
        buf[0] = 0xFF;
        buf[1] = ((xdata_addr + data_len - residual_len) >> 16) & 0xFF;
        buf[2] = ((xdata_addr + data_len - residual_len) >> 8) & 0xFF;
        nvt_i2c_write(chip_info->client, I2C_FW_Address, buf, 3);

        //---read xdata by I2C_TANSFER_LENGTH
        for (j = 0; j < (residual_len / I2C_TANSFER_LENGTH + 1); j++) {
            //---read data---
            buf[0] = I2C_TANSFER_LENGTH * j;
            nvt_i2c_read(chip_info->client, I2C_FW_Address, buf, I2C_TANSFER_LENGTH + 1);

            //---copy buf to xdata_tmp---
            for (k = 0; k < I2C_TANSFER_LENGTH; k++) {
                xdata_tmp[(dummy_len + data_len - residual_len) + I2C_TANSFER_LENGTH * j + k] = buf[k + 1];
                //printk("0x%02X, 0x%04x\n", buf[k+1], ((dummy_len+data_len-residual_len) + I2C_TANSFER_LENGTH*j + k));
            }
        }
        //printk("addr=0x%05X\n", (xdata_addr+data_len-residual_len));
    }

    //---remove dummy data and 2bytes-to-1data---
    for (i = 0; i < (data_len / 2); i++) {
        xdata[i] = (int16_t)(xdata_tmp[dummy_len + i * 2] + 256 * xdata_tmp[dummy_len + i * 2 + 1]);
    }

    //---set xdata index to EVENT BUF ADDR---
    buf[0] = 0xFF;
    buf[1] = (chip_info->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
    buf[2] = (chip_info->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
    nvt_i2c_write(chip_info->client, I2C_FW_Address, buf, 3);

    kfree(xdata_tmp);
}

static void nvt_data_read(struct seq_file *s, struct chip_data_nt36672 *chip_info, DEBUG_READ_TYPE read_type)
{
    int ret = -1;
    int i, j;
    uint8_t pipe;
    int32_t *xdata = NULL;
    int32_t buf_len = 0;

    TPD_INFO("nvt clear fw status start\n");
    ret = nvt_clear_fw_status(chip_info);
    if (ret < 0) {
        TPD_INFO("clear_fw_status error, return\n");
        return;
    }

    nvt_change_mode(chip_info, TEST_MODE_2);

    TPD_INFO("nvt check fw status start\n");
    ret = nvt_check_fw_status(chip_info);
    if (ret < 0) {
        TPD_INFO("check_fw_status error, return\n");
        return;
    }

    TPD_INFO("nvt get fw info start");
    ret = nvt_get_fw_info(chip_info);
    if (ret < 0) {
        TPD_INFO("get_fw_info error, return\n");
        return;
    }

    buf_len = chip_info->hw_res->TX_NUM*chip_info->hw_res->RX_NUM*sizeof(int32_t);
    xdata = kzalloc(buf_len, GFP_KERNEL);
    if (!xdata) {
        TPD_INFO("%s, malloc memory failed\n", __func__);
        return;
    }

    pipe = nvt_get_fw_pipe(chip_info);
    TPD_INFO("nvt_get_fw_pipe:%d\n", pipe);
    switch (read_type) {
        case NVT_RAWDATA:
            seq_printf(s, "raw_data:\n");
            if (pipe == 0)
                nvt_read_mdata(chip_info, chip_info->mmap->RAW_PIPE0_ADDR, xdata, buf_len);
            else
                nvt_read_mdata(chip_info, chip_info->mmap->RAW_PIPE1_ADDR, xdata, buf_len);
            break;
        case NVT_DIFFDATA:
            seq_printf(s, "diff_data:\n");
            if (pipe == 0)
                nvt_read_mdata(chip_info, chip_info->mmap->DIFF_PIPE0_ADDR, xdata, buf_len);
            else
                nvt_read_mdata(chip_info, chip_info->mmap->DIFF_PIPE1_ADDR, xdata, buf_len);
            break;
        case NVT_BASEDATA:
            seq_printf(s, "basline_data:\n");
            nvt_read_mdata(chip_info, chip_info->mmap->BASELINE_ADDR, xdata, buf_len);
            break;
        default:
            seq_printf(s, "read type not support\n");
            break;
    }

    nvt_change_mode(chip_info, NORMAL_MODE);
    TPD_INFO("change normal mode end\n");

    //print all data
    for (i = 0; i < chip_info->hw_res->RX_NUM; i++) {
        seq_printf(s, "[%2d]", i);
        for (j = 0; j < chip_info->hw_res->TX_NUM; j++) {
            seq_printf(s, "%5d, ", xdata[i * chip_info->hw_res->TX_NUM + j]);
        }
        seq_printf(s, "\n");
    }

    kfree(xdata);
}

/*************** End of function work for debug_info proc callbacks***************************/


/************** Start of implementation of debug_info proc callbacks**************************/
static void nvt_delta_read(struct seq_file *s, void *chip_data)
{
    struct chip_data_nt36672 *chip_info = (struct chip_data_nt36672 *)chip_data;

    nvt_data_read(s, chip_info, NVT_DIFFDATA);
}

static void nvt_baseline_read(struct seq_file *s, void *chip_data)
{
    struct chip_data_nt36672 *chip_info = (struct chip_data_nt36672 *)chip_data;

    nvt_data_read(s, chip_info, NVT_BASEDATA);
    nvt_data_read(s, chip_info, NVT_RAWDATA);
}

static void nvt_main_register_read(struct seq_file *s, void *chip_data)
{
    uint8_t buf[3];
    struct chip_data_nt36672 *chip_info = (struct chip_data_nt36672 *)chip_data;

    if (!chip_info)
        return ;

    //---set xdata index to EVENT BUF ADDR---
    buf[0] = 0xFF;
    buf[1] = (chip_info->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
    buf[2] = (chip_info->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
    nvt_i2c_write(chip_info->client, I2C_FW_Address, buf, 3);

    //---read cmd status---
    buf[0] = 0x5E;
    buf[1] = 0xFF;
    nvt_i2c_read(chip_info->client, I2C_FW_Address, buf, 2);

    seq_printf(s, "PWR_FLAG:%d\n", (buf[1]>> PWR_FLAG) & 0x01);
    seq_printf(s, "EDGE_REGECT:%d\n", (buf[1]>> EDGE_REGECT) & 0x01);
    seq_printf(s, "JITTER_FLAG:%d\n", (buf[1]>> JITTER_FLAG) & 0x01);
}

static struct debug_info_proc_operations debug_info_proc_ops = {
    .limit_read    = nvt_limit_read,
    .delta_read    = nvt_delta_read,
    .baseline_read = nvt_baseline_read,
    .main_register_read = nvt_main_register_read,
};
/***************** End of implementation of debug_info proc callbacks*************************/

/************** Start of implementation of baseline_test proc callbacks***********************/
static int8_t nvt_switch_FreqHopEnDis(struct chip_data_nt36672 *chip_info, uint8_t FreqHopEnDis)
{
    uint8_t buf[8] = {0};
    uint8_t retry = 0;
    int8_t ret = 0;

    for (retry = 0; retry < 20; retry++) {
        //---set xdata index to EVENT BUF ADDR---
        buf[0] = 0xFF;
        buf[1] = (chip_info->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
        buf[2] = (chip_info->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
        nvt_i2c_write(chip_info->client, I2C_FW_Address, buf, 3);

        //---switch FreqHopEnDis---
        buf[0] = EVENT_MAP_HOST_CMD;
        buf[1] = FreqHopEnDis;
        nvt_i2c_write(chip_info->client, I2C_FW_Address, buf, 2);

        msleep(35);

        buf[0] = EVENT_MAP_HOST_CMD;
        buf[1] = 0xFF;
        nvt_i2c_read(chip_info->client, I2C_FW_Address, buf, 2);

        if (buf[1] == 0x00)
            break;
    }

    if (unlikely(retry == 20)) {
        TPD_INFO("switch FreqHopEnDis 0x%02X failed, buf[1]=0x%02X\n", FreqHopEnDis, buf[1]);
        ret = -1;
    }

    return ret;
}

static void store_to_file(int fd, char* format, ...)
{
    va_list args;
    char buf[64] = {0};

    va_start(args, format);
    vsnprintf(buf, 64, format, args);
    va_end(args);

    if(fd >= 0) {
        sys_write(fd, buf, strlen(buf));
    }
}

static void nvt_enable_noise_collect(struct chip_data_nt36672 *chip_info, int32_t frame_num)
{
    int ret = -1;
    uint8_t buf[8] = {0};

    //---set xdata index to EVENT BUF ADDR---
    buf[0] = 0xFF;
    buf[1] = (chip_info->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
    buf[2] = (chip_info->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
    ret = nvt_i2c_write(chip_info->client, I2C_FW_Address, buf, 3);

    //---enable noise collect---
    buf[0] = EVENT_MAP_HOST_CMD;
    buf[1] = 0x47;
    buf[2] = 0xAA;
    buf[3] = frame_num;
    buf[4] = 0x00;
    ret |= nvt_i2c_write(chip_info->client, I2C_FW_Address, buf, 5);
    if (ret < 0) {
        TPD_INFO("%s failed\n", __func__);
    }
}

static int32_t nvt_polling_hand_shake_status(struct chip_data_nt36672 *chip_info)
{
    uint8_t buf[8] = {0};
    int32_t i = 0;
    const int32_t retry = 50;

    for (i = 0; i < retry; i++) {
        //---set xdata index to EVENT BUF ADDR---
        buf[0] = 0xFF;
        buf[1] = (chip_info->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
        buf[2] = (chip_info->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
        nvt_i2c_write(chip_info->client, I2C_FW_Address, buf, 3);

        //---read fw status---
        buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
        buf[1] = 0x00;
        nvt_i2c_read(chip_info->client, I2C_FW_Address, buf, 2);

        if ((buf[1] == 0xA0) || (buf[1] == 0xA1))
            break;

        msleep(10);
    }

    if (i >= retry) {
        TPD_INFO("polling hand shake status failed, buf[1]=0x%02X\n", buf[1]);
        return -1;
    } else {
        return 0;
    }
}

static int32_t nvt_read_fw_noise(struct chip_data_nt36672 *chip_info, int32_t config_Diff_Test_Frame, int32_t *xdata, int32_t *xdata_n, int32_t xdata_len)
{
    uint32_t x = 0;
    uint32_t y = 0;
    int32_t iArrayIndex = 0;
    int32_t frame_num = 0;

    if (xdata_len/sizeof(int32_t) < chip_info->hw_res->TX_NUM * chip_info->hw_res->RX_NUM) {
        TPD_INFO("read fw nosie buffer(%d) less than data size(%d)\n", xdata_len, chip_info->hw_res->TX_NUM * chip_info->hw_res->RX_NUM);
        return -1;
    }

    //---Enter Test Mode---
    if (nvt_clear_fw_status(chip_info)) {
        return -EAGAIN;
    }

    frame_num = config_Diff_Test_Frame / 10;
    if (frame_num <= 0)
        frame_num = 1;
    TPD_INFO("%s: frame_num=%d\n", __func__, frame_num);
    nvt_enable_noise_collect(chip_info, frame_num);
    // need wait PS_Config_Diff_Test_Frame * 8.3ms
    msleep(frame_num * 83);

    if (nvt_polling_hand_shake_status(chip_info)) {
        return -EAGAIN;
    }

    if (nvt_get_fw_info(chip_info)) {
        return -EAGAIN;
    }

    if (nvt_get_fw_pipe(chip_info) == 0)
        nvt_read_mdata(chip_info, chip_info->mmap->DIFF_PIPE0_ADDR, xdata, xdata_len);
    else
        nvt_read_mdata(chip_info, chip_info->mmap->DIFF_PIPE1_ADDR, xdata, xdata_len);

    for (y = 0; y < chip_info->hw_res->RX_NUM; y++) {
        for (x = 0; x < chip_info->hw_res->TX_NUM; x++) {
            iArrayIndex = y * chip_info->hw_res->TX_NUM + x;
            xdata_n[iArrayIndex] = (int8_t)(xdata[iArrayIndex] & 0xFF);
            xdata[iArrayIndex] = (int8_t)((xdata[iArrayIndex] >> 8) & 0xFF);
        }
    }

    //---Leave Test Mode---
    nvt_change_mode(chip_info, NORMAL_MODE);
    return 0;
}

static void nvt_enable_doze_noise_collect(struct chip_data_nt36672 *chip_info, int32_t frame_num)
{
    int ret = -1;
    uint8_t buf[8] = {0};

    //---set xdata index to EVENT BUF ADDR---
    buf[0] = 0xFF;
    buf[1] = (chip_info->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
    buf[2] = (chip_info->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
    ret = nvt_i2c_write(chip_info->client, I2C_FW_Address, buf, 3);

    //---enable noise collect---
    buf[0] = EVENT_MAP_HOST_CMD;
    buf[1] = 0x4B;
    buf[2] = 0xAA;
    buf[3] = frame_num;
    buf[4] = 0x00;
    ret |= nvt_i2c_write(chip_info->client, I2C_FW_Address, buf, 5);
    if (ret < 0) {
        TPD_INFO("%s failed\n", __func__);
    }
}

static int32_t nvt_read_doze_fw_noise(struct chip_data_nt36672 *chip_info, int32_t config_Doze_Noise_Test_Frame, int32_t doze_X_Channel,int32_t *xdata, int32_t *xdata_n, int32_t xdata_len)
{
    uint8_t buf[128] = {0};
    uint32_t x = 0;
    uint32_t y = 0;
    uint32_t rx_num = chip_info->hw_res->RX_NUM;
    int32_t iArrayIndex = 0;
    int32_t frame_num = 0;

    if (xdata_len/sizeof(int32_t) < rx_num * doze_X_Channel) {
        TPD_INFO("read doze nosie buffer(%d) less than data size(%d)\n", xdata_len, rx_num * doze_X_Channel);
        return -1;
    }

    //---Enter Test Mode---
    if (nvt_clear_fw_status(chip_info)) {
        return -EAGAIN;
    }

    frame_num = config_Doze_Noise_Test_Frame / 10;
    if (frame_num <= 0)
        frame_num = 1;
    TPD_INFO("%s: frame_num=%d\n", __func__, frame_num);
    nvt_enable_doze_noise_collect(chip_info, frame_num);
    // need wait PS_Config_Doze_Noise_Test_Frame * 8.3ms
    msleep(frame_num * 83);

    if (nvt_polling_hand_shake_status(chip_info)) {
        return -EAGAIN;
    }

    for (x = 0; x < doze_X_Channel; x++) {
        //---change xdata index---
        buf[0] = 0xFF;
        buf[1] = ((chip_info->mmap->DIFF_PIPE0_ADDR + rx_num * doze_X_Channel * x) >> 16) & 0xFF;
        buf[2] = ((chip_info->mmap->DIFF_PIPE0_ADDR + rx_num * doze_X_Channel * x) >> 8) & 0xFF;
        nvt_i2c_write(chip_info->client, I2C_FW_Address, buf, 3);

        //---read data---
        buf[0] = (chip_info->mmap->DIFF_PIPE0_ADDR + rx_num * doze_X_Channel * x) & 0xFF;
        nvt_i2c_read(chip_info->client, I2C_FW_Address, buf, rx_num * 2 + 1);

        for (y = 0; y < rx_num; y++) {
            xdata[y * doze_X_Channel + x] = (uint16_t)(buf[y * doze_X_Channel + 1] + 256 * buf[y * doze_X_Channel + 2]);
        }
    }

    for (y = 0; y < rx_num; y++) {
        for (x = 0; x < doze_X_Channel; x++) {
            iArrayIndex = y * doze_X_Channel + x;
            xdata_n[iArrayIndex] = (int8_t)(xdata[iArrayIndex] & 0xFF) * 4;
            xdata[iArrayIndex] = (int8_t)((xdata[iArrayIndex] >> 8) & 0xFF) * 4;    //scaling up
        }
    }

    //---Leave Test Mode---
    //nvt_change_mode(NORMAL_MODE);    //No return to normal mode. Continuous to get doze rawdata

    return 0;
}

static int32_t nvt_read_doze_baseline(struct chip_data_nt36672 *chip_info, int32_t doze_X_Channel, int32_t *xdata, int32_t xdata_len)
{
    uint8_t buf[128] = {0};
    uint32_t x = 0;
    uint32_t y = 0;
    uint32_t rm_num = chip_info->hw_res->RX_NUM;
    int32_t iArrayIndex = 0;

    //---Enter Test Mode---
    //nvt_change_mode(TEST_MODE_2);

    //if (nvt_check_fw_status()) {
    //    return -EAGAIN;
    //}
    if (xdata_len/sizeof(int32_t) < rm_num * doze_X_Channel) {
        TPD_INFO("read doze baseline buffer(%d) less than data size(%d)\n", xdata_len, rm_num * doze_X_Channel);
        return -1;
    }

    for (x = 0; x < doze_X_Channel; x++) {
        //---change xdata index---
        buf[0] = 0xFF;
        buf[1] = ((chip_info->mmap->DOZE_GM_S1D_SCAN_RAW_ADDR + rm_num * doze_X_Channel * x) >> 16) & 0xFF;
        buf[2] = ((chip_info->mmap->DOZE_GM_S1D_SCAN_RAW_ADDR + rm_num * doze_X_Channel * x) >> 8) & 0xFF;
        nvt_i2c_write(chip_info->client, I2C_FW_Address, buf, 3);

        //---read data---
        buf[0] = (chip_info->mmap->DOZE_GM_S1D_SCAN_RAW_ADDR + rm_num * doze_X_Channel * x) & 0xFF;
        nvt_i2c_read(chip_info->client, I2C_FW_Address, buf, rm_num * 2 + 1);
        for (y = 0; y < rm_num; y++) {
            xdata[y * 2 + x] = (uint16_t)(buf[y * 2 + 1] + 256 * buf[y * 2 + 2]);
        }
    }

    for (y = 0; y < rm_num; y++) {
        for (x = 0; x < doze_X_Channel; x++) {
            iArrayIndex = y * doze_X_Channel + x;
            xdata[iArrayIndex] = (int16_t)xdata[iArrayIndex];
        }
    }

    //---Leave Test Mode---
    nvt_change_mode(chip_info, NORMAL_MODE);
    return 0;
}

static void nvt_enable_short_test(struct chip_data_nt36672 *chip_info)
{
    uint8_t buf[8] = {0};

    //---set xdata index to EVENT BUF ADDR---
    buf[0] = 0xFF;
    buf[1] = (chip_info->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
    buf[2] = (chip_info->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
    nvt_i2c_write(chip_info->client, I2C_FW_Address, buf, 3);

    //---enable short test---
    buf[0] = EVENT_MAP_HOST_CMD;
    buf[1] = 0x43;
    buf[2] = 0xAA;
    buf[3] = 0x02;
    buf[4] = 0x00;
    nvt_i2c_write(chip_info->client, I2C_FW_Address, buf, 5);
}

static int32_t nvt_read_fw_short(struct chip_data_nt36672 *chip_info, int32_t *xdata, int32_t xdata_len)
{
    uint32_t raw_pipe_addr = 0;
    uint8_t *rawdata_buf = NULL;
    uint32_t x = 0;
    uint32_t y = 0;
    uint8_t buf[128] = {0};
    int32_t iArrayIndex = 0;

    if (xdata_len/sizeof(int32_t) < chip_info->hw_res->TX_NUM * chip_info->hw_res->RX_NUM) {
        TPD_INFO("read fw short buffer(%d) less than data size(%d)\n", xdata_len, chip_info->hw_res->TX_NUM * chip_info->hw_res->RX_NUM);
        return -1;
    }

    //---Enter Test Mode---
    if (nvt_clear_fw_status(chip_info)) {
        return -EAGAIN;
    }

    nvt_enable_short_test(chip_info);

    if (nvt_polling_hand_shake_status(chip_info)) {
        return -EAGAIN;
    }

    rawdata_buf = (uint8_t *)kzalloc(chip_info->hw_res->TX_NUM * chip_info->hw_res->RX_NUM * 2, GFP_KERNEL);
    if (!rawdata_buf) {
        TPD_INFO("kzalloc for rawdata_buf failed!\n");
        return -ENOMEM;
    }

    if (nvt_get_fw_pipe(chip_info) == 0)
        raw_pipe_addr = chip_info->mmap->RAW_PIPE0_ADDR;
    else
        raw_pipe_addr = chip_info->mmap->RAW_PIPE1_ADDR;

    for (y = 0; y < chip_info->hw_res->RX_NUM; y++) {
        //---change xdata index---
        buf[0] = 0xFF;
        buf[1] = (uint8_t)(((raw_pipe_addr + y * chip_info->hw_res->TX_NUM * 2) >> 16) & 0xFF);
        buf[2] = (uint8_t)(((raw_pipe_addr + y * chip_info->hw_res->TX_NUM * 2) >> 8) & 0xFF);
        nvt_i2c_write(chip_info->client, I2C_FW_Address, buf, 3);
        buf[0] = (uint8_t)((raw_pipe_addr + y * chip_info->hw_res->TX_NUM * 2) & 0xFF);
        nvt_i2c_read(chip_info->client, I2C_FW_Address, buf, chip_info->hw_res->TX_NUM * 2 + 1);
        memcpy(rawdata_buf + y * chip_info->hw_res->TX_NUM * 2, buf + 1, chip_info->hw_res->TX_NUM * 2);
    }

    for (y = 0; y < chip_info->hw_res->RX_NUM; y++) {
        for (x = 0; x < chip_info->hw_res->TX_NUM; x++) {
            iArrayIndex = y * chip_info->hw_res->TX_NUM + x;
            xdata[iArrayIndex] = (int16_t)(rawdata_buf[iArrayIndex * 2] + 256 * rawdata_buf[iArrayIndex * 2 + 1]);
        }
    }

    if (rawdata_buf) {
        kfree(rawdata_buf);
        rawdata_buf = NULL;
    }

    //---Leave Test Mode---
    nvt_change_mode(chip_info, NORMAL_MODE);
    return 0;
}

static void nvt_enable_open_test(struct chip_data_nt36672 *chip_info)
{
    uint8_t buf[8] = {0};

    //---set xdata index to EVENT BUF ADDR---
    buf[0] = 0xFF;
    buf[1] = (chip_info->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
    buf[2] = (chip_info->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
    nvt_i2c_write(chip_info->client, I2C_FW_Address, buf, 3);

    //---enable open test---
    buf[0] = EVENT_MAP_HOST_CMD;
    buf[1] = 0x45;
    buf[2] = 0xAA;
    buf[3] = 0x02;
    buf[4] = 0x00;
    nvt_i2c_write(chip_info->client, I2C_FW_Address, buf, 5);
}

static bool nvt_is_NT36672A(struct chip_data_nt36672 *chip_inf)
{
    int i = 0;
    uint8_t nvt_36672A[NVT_ID_BYTE_MAX] = {0x0A, 0xFF, 0xFF, 0x72, 0x66, 0x03};

    for (i = 0; i < NVT_ID_BYTE_MAX; i++) {
        if (chip_inf->id[i] != nvt_36672A[i]) {
            return false;
        }
    }

    return true;
}

static int32_t nvt_read_fw_open(struct chip_data_nt36672 *chip_info, int32_t *xdata, int32_t xdata_len)
{
    uint32_t raw_pipe_addr = 0;
    uint8_t *rawdata_buf = NULL;
    uint32_t x = 0, y = 0;
    uint32_t tx_num = chip_info->hw_res->TX_NUM;
    uint32_t rx_num = chip_info->hw_res->RX_NUM;
    uint8_t buf[128] = {0};

    if (xdata_len/sizeof(int32_t) < chip_info->hw_res->TX_NUM * chip_info->hw_res->RX_NUM) {
        TPD_INFO("read fw open buffer(%d) less than data size(%d)\n", xdata_len, chip_info->hw_res->TX_NUM * chip_info->hw_res->RX_NUM);
        return -1;
    }

    //---Enter Test Mode---
    if (nvt_clear_fw_status(chip_info)) {
        return -EAGAIN;
    }

    nvt_enable_open_test(chip_info);

    if (nvt_polling_hand_shake_status(chip_info)) {
        return -EAGAIN;
    }

    rawdata_buf = (uint8_t *)kzalloc(tx_num * rx_num * 2, GFP_KERNEL);
    if (!rawdata_buf) {
        TPD_INFO("kzalloc for rawdata_buf failed!\n");
        return -ENOMEM;
    }

    if (nvt_get_fw_pipe(chip_info) == 0)
        raw_pipe_addr = chip_info->mmap->RAW_PIPE0_ADDR;
    else
        raw_pipe_addr = chip_info->mmap->RAW_PIPE1_ADDR;

    for (y = 0; y < rx_num; y++) {
        //---change xdata index---
        buf[0] = 0xFF;
        buf[1] = (uint8_t)(((raw_pipe_addr + y * tx_num * 2) >> 16) & 0xFF);
        buf[2] = (uint8_t)(((raw_pipe_addr + y * tx_num * 2) >> 8) & 0xFF);
        nvt_i2c_write(chip_info->client, I2C_FW_Address, buf, 3);
        buf[0] = (uint8_t)((raw_pipe_addr + y * tx_num * 2) & 0xFF);
        nvt_i2c_read(chip_info->client, I2C_FW_Address, buf, tx_num * 2 + 1);
        memcpy(rawdata_buf + y * tx_num * 2, buf + 1, tx_num * 2);
    }

    for (y = 0; y < rx_num; y++) {
        for (x = 0; x < tx_num; x++) {
            if (nvt_is_NT36672A(chip_info)) { /*for 36672A*/
                xdata[(y*tx_num + x)] = (int16_t)((rawdata_buf[(y*tx_num + x)*2] + 256 * rawdata_buf[(y*tx_num + x)*2 + 1]));
            } else {
                xdata[(rx_num-y-1) * tx_num + (tx_num-x-1)] = (int16_t)((rawdata_buf[(y*tx_num + x)*2] + 256 * rawdata_buf[(y*tx_num + x)*2 + 1]));
            }
        }
    }

    if (rawdata_buf) {
        kfree(rawdata_buf);
        rawdata_buf = NULL;
    }

    //---Leave Test Mode---
    nvt_change_mode(chip_info, NORMAL_MODE);
    return 0;
}

static void nvt_auto_test(struct seq_file *s, void *chip_data, struct nvt_testdata *nvt_testdata)
{
    int32_t *raw_data = NULL;
    int32_t *raw_data_n = NULL;
    int i, j, buf_len = 0;
    int err_cnt = 0;
    int32_t iArrayIndex = 0;
    struct chip_data_nt36672 *chip_info = (struct chip_data_nt36672 *)chip_data;
    struct nvt_test_header *ph = NULL;
    int32_t *fw_rawdata_P = NULL, *fw_rawdata_N = NULL;
    int32_t *open_rawdata_P = NULL, *open_rawdata_N = NULL;
    int32_t *short_rawdata_P = NULL, *short_rawdata_N = NULL;
    int32_t *diff_rawdata_P = NULL, *diff_rawdata_N = NULL;
    int32_t *cc_data_P = NULL, *cc_data_N = NULL;
    int32_t *doze_rawdata_P = NULL, *doze_rawdata_N = NULL;
    int32_t *doze_diff_rawdata_P = NULL, *doze_diff_rawdata_N = NULL;

    if (nvt_switch_FreqHopEnDis(chip_info, FREQ_HOP_DISABLE)) {
        TPD_INFO("switch frequency hopping disable failed!\n");
        store_to_file(nvt_testdata->fd, "switch frequency hopping disable failed!\n");
        seq_printf(s, "switch frequency hopping disable failed!\n");
        return;
    }

    if (nvt_check_fw_reset_state(chip_info, RESET_STATE_NORMAL_RUN)) {
        TPD_INFO("check fw reset state failed!\n");
        store_to_file(nvt_testdata->fd, "check fw reset state failed!\n");
        seq_printf(s, "check fw reset state failed!\n");
        return;
    }

    msleep(100);

    //---Enter Test Mode---
    TPD_INFO("enter test mode\n");
    if (nvt_clear_fw_status(chip_info)) {
        TPD_INFO("clear fw status failed!\n");
        store_to_file(nvt_testdata->fd, "clear fw status failed!\n");
        seq_printf(s, "clear fw status failed!\n");
        return;
    }

    nvt_change_mode(chip_info, MP_MODE_CC);

    if (nvt_check_fw_status(chip_info)) {
        TPD_INFO("check fw status failed!\n");
        store_to_file(nvt_testdata->fd, "check fw status failed!\n");
        seq_printf(s, "check fw status failed!\n");
        return;
    }

    if (nvt_get_fw_info(chip_info)) {
        TPD_INFO("get fw info failed!\n");
        store_to_file(nvt_testdata->fd, "get fw info failed!\n");
        seq_printf(s, "get fw info failed!\n");
        return;
    }

    TPD_INFO("malloc raw_data space\n");
    buf_len = nvt_testdata->TX_NUM * nvt_testdata->RX_NUM * sizeof(int32_t);
    raw_data = kzalloc(buf_len, GFP_KERNEL);
    raw_data_n = kzalloc(buf_len, GFP_KERNEL);
    if (!(raw_data && raw_data_n)) {
        err_cnt++;
        store_to_file(nvt_testdata->fd, "malloc memory failed!\n");
        seq_printf(s, "malloc memory failed!\n");
        goto OUT;
    }

    //get test data
    ph = (struct nvt_test_header *)(nvt_testdata->fw->data);
    fw_rawdata_P = (int32_t *)(nvt_testdata->fw->data + ph->array_fw_rawdata_P_offset);
    fw_rawdata_N = (int32_t *)(nvt_testdata->fw->data + ph->array_fw_rawdata_N_offset);
    open_rawdata_P = (int32_t *)(nvt_testdata->fw->data + ph->array_open_rawdata_P_offset);
    open_rawdata_N = (int32_t *)(nvt_testdata->fw->data + ph->array_open_rawdata_N_offset);
    short_rawdata_P = (int32_t *)(nvt_testdata->fw->data + ph->array_Short_Rawdata_P_offset);
    short_rawdata_N = (int32_t *)(nvt_testdata->fw->data + ph->array_Short_Rawdata_N_offset);
    diff_rawdata_P = (int32_t *)(nvt_testdata->fw->data + ph->array_FW_Diff_P_offset);
    diff_rawdata_N = (int32_t *)(nvt_testdata->fw->data + ph->array_FW_Diff_N_offset);
    cc_data_P = (int32_t *)(nvt_testdata->fw->data + ph->array_FW_CC_P_offset);
    cc_data_N = (int32_t *)(nvt_testdata->fw->data + ph->array_FW_CC_N_offset);
    doze_rawdata_P = (int32_t *)(nvt_testdata->fw->data + ph->array_Doze_Rawdata_P_offset);
    doze_rawdata_N = (int32_t *)(nvt_testdata->fw->data + ph->array_Doze_Rawdata_N_offset);
    doze_diff_rawdata_P = (int32_t *)(nvt_testdata->fw->data + ph->array_Doze_Diff_P_offset);
    doze_diff_rawdata_N = (int32_t *)(nvt_testdata->fw->data + ph->array_Doze_Diff_N_offset);

    //---FW Rawdata Test---
    TPD_INFO("FW Rawdata Test \n");
    memset(raw_data, 0, buf_len);
    nvt_read_mdata(chip_info, chip_info->mmap->BASELINE_ADDR, raw_data, buf_len);
    store_to_file(nvt_testdata->fd, "rawData:\n");
    for (j = 0; j < nvt_testdata->RX_NUM; j++) {
        for (i = 0; i < nvt_testdata->TX_NUM; i++) {
            iArrayIndex = j * nvt_testdata->TX_NUM + i;
            if (nvt_testdata->fd >= 0) {
                store_to_file(nvt_testdata->fd, "%d, ", raw_data[iArrayIndex]);
            }
            if((raw_data[iArrayIndex] > fw_rawdata_P[iArrayIndex]) \
                    || (raw_data[iArrayIndex] < fw_rawdata_N[iArrayIndex])) {
                TPD_INFO("rawdata Test failed at rawdata[%d][%d] = %d [%d,%d]\n", i, j, raw_data[iArrayIndex], fw_rawdata_N[iArrayIndex], fw_rawdata_P[iArrayIndex]);
                if (!err_cnt) {
                    seq_printf(s, "rawdata Test failed at rawdata[%d][%d] = %d [%d,%d]\n", i, j, raw_data[iArrayIndex], fw_rawdata_N[iArrayIndex], fw_rawdata_P[iArrayIndex]);
                }
                err_cnt++;
            }
        }
        if (nvt_testdata->fd >= 0) {
            store_to_file(nvt_testdata->fd, "\n");
        }
    }

    TPD_INFO("FW cc data test \n");
    memset(raw_data, 0, buf_len);
    if (nvt_get_fw_pipe(chip_info) == 0)
        nvt_read_mdata(chip_info, chip_info->mmap->DIFF_PIPE1_ADDR, raw_data, buf_len);
    else
        nvt_read_mdata(chip_info, chip_info->mmap->DIFF_PIPE0_ADDR, raw_data, buf_len);
    store_to_file(nvt_testdata->fd, "ccData:\n");
    if ((ph->config_Lmt_FW_CC_P != 0) && (ph->config_Lmt_FW_CC_N != 0)) {
        for (j = 0; j < nvt_testdata->RX_NUM; j++) {
            for (i = 0; i < nvt_testdata->TX_NUM; i++) {
                iArrayIndex = j * nvt_testdata->TX_NUM + i;
                if (nvt_testdata->fd >= 0) {
                    store_to_file(nvt_testdata->fd, "%d, ", raw_data[iArrayIndex]);
                }
                if((raw_data[iArrayIndex] > ph->config_Lmt_FW_CC_P) \
                        || (raw_data[iArrayIndex] < ph->config_Lmt_FW_CC_N)) {
                    TPD_INFO("cc data Test failed at rawdata[%d][%d] = %d [%d,%d]\n", i, j, raw_data[iArrayIndex], ph->config_Lmt_FW_CC_N, ph->config_Lmt_FW_CC_P);
                    if (!err_cnt) {
                        seq_printf(s, "cc data Test failed at rawdata[%d][%d] = %d [%d,%d]\n", i, j, raw_data[iArrayIndex], ph->config_Lmt_FW_CC_N, ph->config_Lmt_FW_CC_P);
                    }
                    err_cnt++;
                }
            }
            if (nvt_testdata->fd >= 0) {
                store_to_file(nvt_testdata->fd, "\n");
            }
        }
    } else {
        for (j = 0; j < nvt_testdata->RX_NUM; j++) {
            for (i = 0; i < nvt_testdata->TX_NUM; i++) {
                iArrayIndex = j * nvt_testdata->TX_NUM + i;
                if (nvt_testdata->fd >= 0) {
                    store_to_file(nvt_testdata->fd, "%d, ", raw_data[iArrayIndex]);
                }
                if((raw_data[iArrayIndex] > cc_data_P[iArrayIndex]) \
                        || (raw_data[iArrayIndex] < cc_data_N[iArrayIndex])) {
                    TPD_INFO("cc data Test failed at rawdata[%d][%d] = %d [%d,%d]\n", i, j, raw_data[iArrayIndex], cc_data_N[iArrayIndex], cc_data_P[iArrayIndex]);
                    if (!err_cnt) {
                        seq_printf(s, "cc data Test failed at rawdata[%d][%d] = %d [%d,%d]\n", i, j, raw_data[iArrayIndex], cc_data_N[iArrayIndex], cc_data_P[iArrayIndex]);
                    }
                    err_cnt++;
                }
            }
            if (nvt_testdata->fd >= 0) {
                store_to_file(nvt_testdata->fd, "\n");
            }
        }
    }

    //---Leave Test Mode---
    nvt_change_mode(chip_info, NORMAL_MODE);

    //---Noise Test---
    TPD_INFO("FW Noise Test \n");
    memset(raw_data, 0, buf_len);  //store max
    memset(raw_data_n, 0, buf_len); //store min
    if (nvt_read_fw_noise(chip_info, ph->config_Diff_Test_Frame, raw_data, raw_data_n, buf_len) != 0) {
        TPD_INFO("read Noise data failed!\n");
        store_to_file(nvt_testdata->fd, "read Noise data failed!\n");
        seq_printf(s, "read Noise data failed!\n");
        err_cnt++;
        goto OUT;
    }
    store_to_file(nvt_testdata->fd, "RawData_Diff_Max:\n");
    if ((ph->config_Lmt_FW_Diff_P != 0) && (ph->config_Lmt_FW_Diff_N != 0)) {
        for (j = 0; j < nvt_testdata->RX_NUM; j++) {
            for (i = 0; i < nvt_testdata->TX_NUM; i++) {
                iArrayIndex = j * nvt_testdata->TX_NUM + i;
                if (nvt_testdata->fd >= 0) {
                    store_to_file(nvt_testdata->fd, "%d, ", raw_data[iArrayIndex]);
                }
                if((raw_data[iArrayIndex] > ph->config_Lmt_FW_Diff_P) \
                        || (raw_data[iArrayIndex] < ph->config_Lmt_FW_Diff_N)) {
                    TPD_INFO("Noise RawData_Diff_Max Test failed at data[%d][%d] = %d [%d,%d]\n", i, j, raw_data[iArrayIndex], ph->config_Lmt_FW_Diff_N, ph->config_Lmt_FW_Diff_P);
                    if (!err_cnt) {
                        seq_printf(s, "Noise RawData_Diff_Max Test failed at data[%d][%d] = %d [%d,%d]\n", i, j, raw_data[iArrayIndex], ph->config_Lmt_FW_Diff_N, ph->config_Lmt_FW_Diff_P);
                    }
                    err_cnt++;
                }
            }
            if (nvt_testdata->fd >= 0) {
                store_to_file(nvt_testdata->fd, "\n");
            }
        }
        store_to_file(nvt_testdata->fd, "RawData_Diff_Min:\n");
        for (j = 0; j < nvt_testdata->RX_NUM; j++) {
            for (i = 0; i < nvt_testdata->TX_NUM; i++) {
                iArrayIndex = j * nvt_testdata->TX_NUM + i;
                if (nvt_testdata->fd >= 0) {
                    store_to_file(nvt_testdata->fd, "%d, ", raw_data_n[iArrayIndex]);
                }
                if((raw_data_n[iArrayIndex] > ph->config_Lmt_FW_Diff_P) \
                        || (raw_data_n[iArrayIndex] < ph->config_Lmt_FW_Diff_N)) {
                    TPD_INFO("Noise RawData_Diff_Min Test failed at data[%d][%d] = %d [%d,%d]\n", i, j, raw_data_n[iArrayIndex], ph->config_Lmt_FW_Diff_N, ph->config_Lmt_FW_Diff_P);
                    if (!err_cnt) {
                        seq_printf(s, "Noise RawData_Diff_Min Test failed at data[%d][%d] = %d [%d,%d]\n", i, j, raw_data_n[iArrayIndex], ph->config_Lmt_FW_Diff_N, ph->config_Lmt_FW_Diff_P);
                    }
                    err_cnt++;
                }
            }
            if (nvt_testdata->fd >= 0) {
                store_to_file(nvt_testdata->fd, "\n");
            }
        }
    } else {
        for (j = 0; j < nvt_testdata->RX_NUM; j++) {
            for (i = 0; i < nvt_testdata->TX_NUM; i++) {
                iArrayIndex = j * nvt_testdata->TX_NUM + i;
                if (nvt_testdata->fd >= 0) {
                    store_to_file(nvt_testdata->fd, "%d, ", raw_data[iArrayIndex]);
                }
                if((raw_data[iArrayIndex] > diff_rawdata_P[iArrayIndex]) \
                        || (raw_data[iArrayIndex] < diff_rawdata_N[iArrayIndex])) {
                    TPD_INFO("Noise RawData_Diff_Max Test failed at data[%d][%d] = %d [%d,%d]\n", i, j, raw_data[iArrayIndex], diff_rawdata_N[iArrayIndex], diff_rawdata_P[iArrayIndex]);
                    if (!err_cnt) {
                        seq_printf(s, "Noise RawData_Diff_Max Test failed at data[%d][%d] = %d [%d,%d]\n", i, j, raw_data[iArrayIndex], diff_rawdata_N[iArrayIndex], diff_rawdata_P[iArrayIndex]);
                    }
                    err_cnt++;
                }
            }
            if (nvt_testdata->fd >= 0) {
                store_to_file(nvt_testdata->fd, "\n");
            }
        }
        store_to_file(nvt_testdata->fd, "RawData_Diff_Min:\n");
        for (j = 0; j < nvt_testdata->RX_NUM; j++) {
            for (i = 0; i < nvt_testdata->TX_NUM; i++) {
                iArrayIndex = j * nvt_testdata->TX_NUM + i;
                if (nvt_testdata->fd >= 0) {
                    store_to_file(nvt_testdata->fd, "%d, ", raw_data_n[iArrayIndex]);
                }
                if((raw_data_n[iArrayIndex] > diff_rawdata_P[iArrayIndex]) \
                        || (raw_data_n[iArrayIndex] < diff_rawdata_N[iArrayIndex])) {
                    TPD_INFO("Noise RawData_Diff_Min Test failed at data[%d][%d] = %d [%d,%d]\n", i, j, raw_data_n[iArrayIndex], diff_rawdata_N[iArrayIndex], diff_rawdata_P[iArrayIndex]);
                    if (!err_cnt) {
                        seq_printf(s, "Noise RawData_Diff_Min Test failed at data[%d][%d] = %d [%d,%d]\n", i, j, raw_data_n[iArrayIndex], diff_rawdata_N[iArrayIndex], diff_rawdata_P[iArrayIndex]);
                    }
                    err_cnt++;
                }
            }
            if (nvt_testdata->fd >= 0) {
                store_to_file(nvt_testdata->fd, "\n");
            }
        }
    }

    //---Noise Test---
    TPD_INFO("Doze FW Noise Test \n");
    memset(raw_data, 0, buf_len);  //store max
    memset(raw_data_n, 0, buf_len); //store min
    if (nvt_read_doze_fw_noise(chip_info, ph->config_Doze_Noise_Test_Frame, ph->doze_X_Channel, raw_data, raw_data_n, buf_len) != 0) {
        TPD_INFO("read Doze Noise data failed!\n");
        store_to_file(nvt_testdata->fd, "read Doze Noise data failed!\n");
        seq_printf(s, "read Doze Noise data failed!\n");
        err_cnt++;
        goto OUT;
    }
    store_to_file(nvt_testdata->fd, "Doze RawData_Diff_Max:\n");
    if ((ph->config_Lmt_Doze_Diff_P != 0) && (ph->config_Lmt_Doze_Diff_N != 0)) {
        for (j = 0; j < nvt_testdata->RX_NUM; j++) {
            for (i = 0; i < ph->doze_X_Channel; i++) {
                iArrayIndex = j * ph->doze_X_Channel + i;
                if (nvt_testdata->fd >= 0) {
                    store_to_file(nvt_testdata->fd, "%d, ", raw_data[iArrayIndex]);
                }
                if((raw_data[iArrayIndex] > ph->config_Lmt_Doze_Diff_P) \
                        || (raw_data[iArrayIndex] < ph->config_Lmt_Doze_Diff_N)) {
                    TPD_INFO("Doze Noise RawData_Diff_Max Test failed at data[%d][%d] = %d [%d,%d]\n", i, j, raw_data[iArrayIndex], ph->config_Lmt_Doze_Diff_N, ph->config_Lmt_Doze_Diff_P);
                    if (!err_cnt) {
                        seq_printf(s, "Doze Noise RawData_Diff_Max Test failed at data[%d][%d] = %d [%d,%d]\n", i, j, raw_data[iArrayIndex], ph->config_Lmt_Doze_Diff_N, ph->config_Lmt_Doze_Diff_P);
                    }
                    err_cnt++;
                }
            }
            if (nvt_testdata->fd >= 0) {
                store_to_file(nvt_testdata->fd, "\n");
            }
        }
        store_to_file(nvt_testdata->fd, "Doze RawData_Diff_Min:\n");
        for (j = 0; j < nvt_testdata->RX_NUM; j++) {
            for (i = 0; i < ph->doze_X_Channel; i++) {
                iArrayIndex = j * ph->doze_X_Channel + i;
                if (nvt_testdata->fd >= 0) {
                    store_to_file(nvt_testdata->fd, "%d, ", raw_data_n[iArrayIndex]);
                }
                if((raw_data_n[iArrayIndex] > ph->config_Lmt_Doze_Diff_P) \
                        || (raw_data_n[iArrayIndex] < ph->config_Lmt_Doze_Diff_N)) {
                    TPD_INFO("Doze Noise RawData_Diff_Min Test failed at data[%d][%d] = %d [%d,%d]\n", i, j, raw_data_n[iArrayIndex], ph->config_Lmt_Doze_Diff_N, ph->config_Lmt_Doze_Diff_P);
                    if (!err_cnt) {
                        seq_printf(s, "Doze Noise RawData_Diff_Min Test failed at data[%d][%d] = %d [%d,%d]\n", i, j, raw_data_n[iArrayIndex], ph->config_Lmt_Doze_Diff_N, ph->config_Lmt_Doze_Diff_P);
                    }
                    err_cnt++;
                }
            }
            if (nvt_testdata->fd >= 0) {
                store_to_file(nvt_testdata->fd, "\n");
            }
        }
    } else {
        for (j = 0; j < nvt_testdata->RX_NUM; j++) {
            for (i = 0; i < ph->doze_X_Channel; i++) {
                iArrayIndex = j * ph->doze_X_Channel + i;
                if (nvt_testdata->fd >= 0) {
                    store_to_file(nvt_testdata->fd, "%d, ", raw_data[iArrayIndex]);
                }
                if((raw_data[iArrayIndex] > doze_diff_rawdata_P[iArrayIndex]) \
                        || (raw_data[iArrayIndex] < doze_diff_rawdata_N[iArrayIndex])) {
                    TPD_INFO("Doze Noise RawData_Diff_Max Test failed at data[%d][%d] = %d [%d,%d]\n", i, j, raw_data[iArrayIndex], doze_diff_rawdata_N[iArrayIndex], doze_diff_rawdata_P[iArrayIndex]);
                    if (!err_cnt) {
                        seq_printf(s, "Doze Noise RawData_Diff_Max Test failed at data[%d][%d] = %d [%d,%d]\n", i, j, raw_data[iArrayIndex], doze_diff_rawdata_N[iArrayIndex], doze_diff_rawdata_P[iArrayIndex]);
                    }
                    err_cnt++;
                }
            }
            if (nvt_testdata->fd >= 0) {
                store_to_file(nvt_testdata->fd, "\n");
            }
        }
        store_to_file(nvt_testdata->fd, "Doze RawData_Diff_Min:\n");
        for (j = 0; j < nvt_testdata->RX_NUM; j++) {
            for (i = 0; i < ph->doze_X_Channel; i++) {
                iArrayIndex = j * ph->doze_X_Channel + i;
                if (nvt_testdata->fd >= 0) {
                    store_to_file(nvt_testdata->fd, "%d, ", raw_data_n[iArrayIndex]);
                }
                if((raw_data_n[iArrayIndex] > doze_diff_rawdata_P[iArrayIndex]) \
                        || (raw_data_n[iArrayIndex] < doze_diff_rawdata_N[iArrayIndex])) {
                    TPD_INFO("Doze Noise RawData_Diff_Min Test failed at data[%d][%d] = %d [%d,%d]\n", i, j, raw_data_n[iArrayIndex], doze_diff_rawdata_N[iArrayIndex], doze_diff_rawdata_P[iArrayIndex]);
                    if (!err_cnt) {
                        seq_printf(s, "Doze Noise RawData_Diff_Min Test failed at data[%d][%d] = %d [%d,%d]\n", i, j, raw_data_n[iArrayIndex], doze_diff_rawdata_N[iArrayIndex], doze_diff_rawdata_P[iArrayIndex]);
                    }
                    err_cnt++;
                }
            }
            if (nvt_testdata->fd >= 0) {
                store_to_file(nvt_testdata->fd, "\n");
            }
        }
    }

    //---Doze FW Rawdata Test---
    TPD_INFO("Doze FW Rawdata Test \n");
    memset(raw_data, 0, buf_len);
    if(nvt_read_doze_baseline(chip_info, ph->doze_X_Channel, raw_data, buf_len) != 0) {
        TPD_INFO("read Doze FW Rawdata failed!\n");
        store_to_file(nvt_testdata->fd, "read Doze FW Rawdata failed!\n");
        seq_printf(s, "read Doze FW Rawdata failed!\n");
        err_cnt++;
        goto OUT;
    }
    store_to_file(nvt_testdata->fd, "Doze FW Rawdata:\n");
    if ((ph->config_Lmt_Doze_Rawdata_P != 0) && (ph->config_Lmt_Doze_Rawdata_N != 0)) {
        for (j = 0; j < nvt_testdata->RX_NUM; j++) {
            for (i = 0; i < ph->doze_X_Channel; i++) {
                iArrayIndex = j * ph->doze_X_Channel + i;
                if (nvt_testdata->fd >= 0) {
                    store_to_file(nvt_testdata->fd, "%d, ", raw_data[iArrayIndex]);
                }
                if((raw_data[iArrayIndex] > ph->config_Lmt_Doze_Rawdata_P) \
                        || (raw_data[iArrayIndex] < ph->config_Lmt_Doze_Rawdata_N)) {
                    TPD_INFO("Doze FW Rawdata Test failed at data[%d][%d] = %d [%d,%d]\n", i, j, raw_data[iArrayIndex], ph->config_Lmt_Doze_Rawdata_N, ph->config_Lmt_Doze_Rawdata_P);
                    if (!err_cnt) {
                        seq_printf(s, "Doze FW Rawdata Test failed at data[%d][%d] = %d [%d,%d]\n", i, j, raw_data[iArrayIndex], ph->config_Lmt_Doze_Rawdata_N, ph->config_Lmt_Doze_Rawdata_P);
                    }
                    err_cnt++;
                }
            }
            if (nvt_testdata->fd >= 0) {
                store_to_file(nvt_testdata->fd, "\n");
            }
        }
    } else {
        for (j = 0; j < nvt_testdata->RX_NUM; j++) {
            for (i = 0; i < ph->doze_X_Channel; i++) {
                iArrayIndex = j * ph->doze_X_Channel + i;
                if (nvt_testdata->fd >= 0) {
                    store_to_file(nvt_testdata->fd, "%d, ", raw_data[iArrayIndex]);
                }
                if((raw_data[iArrayIndex] > doze_rawdata_P[iArrayIndex]) \
                        || (raw_data[iArrayIndex] < doze_rawdata_N[iArrayIndex])) {
                    TPD_INFO("Doze FW Rawdata Test failed at data[%d][%d] = %d [%d,%d]\n", i, j, raw_data[iArrayIndex], doze_rawdata_N[iArrayIndex], doze_rawdata_P[iArrayIndex]);
                    if (!err_cnt) {
                        seq_printf(s, "Doze FW Rawdata Test failed at data[%d][%d] = %d [%d,%d]\n", i, j, raw_data[iArrayIndex], doze_rawdata_N[iArrayIndex], doze_rawdata_P[iArrayIndex]);
                    }
                    err_cnt++;
                }
            }
            if (nvt_testdata->fd >= 0) {
                store_to_file(nvt_testdata->fd, "\n");
            }
        }
    }

    //--Short Test---
    TPD_INFO("FW Short Test \n");
    memset(raw_data, 0, buf_len);
    if (nvt_read_fw_short(chip_info, raw_data, buf_len) != 0) {
        TPD_INFO("read Short test data failed!\n");
        store_to_file(nvt_testdata->fd, "read Short test data failed!\n");
        seq_printf(s, "read Short test data failed!\n");
        err_cnt++;
        goto OUT;
    }
    store_to_file(nvt_testdata->fd, "RawData_Short:\n");
    if ((ph->config_Lmt_Short_Rawdata_P != 0) && (ph->config_Lmt_Short_Rawdata_N != 0)) {
        for (j = 0; j < nvt_testdata->RX_NUM; j++) {
            for (i = 0; i < nvt_testdata->TX_NUM; i++) {
                iArrayIndex = j * nvt_testdata->TX_NUM + i;
                if (nvt_testdata->fd >= 0) {
                    store_to_file(nvt_testdata->fd, "%d, ", raw_data[iArrayIndex]);
                }
                if((raw_data[iArrayIndex] > ph->config_Lmt_Short_Rawdata_P) \
                        || (raw_data[iArrayIndex] < ph->config_Lmt_Short_Rawdata_N)) {
                    TPD_INFO("Short Test failed at data[%d][%d] = %d [%d,%d]\n", i, j, raw_data[iArrayIndex], ph->config_Lmt_Short_Rawdata_N, ph->config_Lmt_Short_Rawdata_P);
                    if (!err_cnt) {
                        seq_printf(s, "Short Test failed at data[%d][%d] = %d [%d,%d]\n", i, j, raw_data[iArrayIndex], ph->config_Lmt_Short_Rawdata_N, ph->config_Lmt_Short_Rawdata_P);
                    }
                    err_cnt++;
                }
            }
            if (nvt_testdata->fd >= 0) {
                store_to_file(nvt_testdata->fd, "\n");
            }
        }
    } else {
        for (j = 0; j < nvt_testdata->RX_NUM; j++) {
            for (i = 0; i < nvt_testdata->TX_NUM; i++) {
                iArrayIndex = j * nvt_testdata->TX_NUM + i;
                if (nvt_testdata->fd >= 0) {
                    store_to_file(nvt_testdata->fd, "%d, ", raw_data[iArrayIndex]);
                }
                if((raw_data[iArrayIndex] > short_rawdata_P[iArrayIndex]) \
                        || (raw_data[iArrayIndex] < short_rawdata_N[iArrayIndex])) {
                    TPD_INFO("Short Test failed at data[%d][%d] = %d [%d,%d]\n", i, j, raw_data[iArrayIndex], short_rawdata_N[iArrayIndex], short_rawdata_P[iArrayIndex]);
                    if (!err_cnt) {
                        seq_printf(s, "Short Test failed at data[%d][%d] = %d [%d,%d]\n", i, j, raw_data[iArrayIndex], short_rawdata_N[iArrayIndex], short_rawdata_P[iArrayIndex]);
                    }
                    err_cnt++;
                }
            }
            if (nvt_testdata->fd >= 0) {
                store_to_file(nvt_testdata->fd, "\n");
            }
        }
    }

    //---Open Test---
    TPD_INFO("FW Open Test \n");
    memset(raw_data, 0, buf_len);
    if (nvt_read_fw_open(chip_info, raw_data, buf_len) != 0) {
        TPD_INFO("read Open test data failed!\n");
        store_to_file(nvt_testdata->fd, "read Open test data failed!\n");
        seq_printf(s, "read Open test data failed!\n");
        err_cnt++;
        goto OUT;
    }
    store_to_file(nvt_testdata->fd, "RawData_Open:\n");
    for (j = 0; j < nvt_testdata->RX_NUM; j++) {
        for (i = 0; i < nvt_testdata->TX_NUM; i++) {
            iArrayIndex = j * nvt_testdata->TX_NUM + i;
            if (nvt_testdata->fd >= 0) {
                store_to_file(nvt_testdata->fd, "%d, ", raw_data[iArrayIndex]);
            }
            if((raw_data[iArrayIndex] > open_rawdata_P[iArrayIndex]) \
                    || (raw_data[iArrayIndex] < open_rawdata_N[iArrayIndex])) {
                TPD_INFO("Open Test failed at data[%d][%d] = %d [%d,%d]\n", i, j, raw_data[iArrayIndex], open_rawdata_N[iArrayIndex], open_rawdata_P[iArrayIndex]);
                if (!err_cnt) {
                    seq_printf(s, "Open Test failed at data[%d][%d] = %d [%d,%d]\n", i, j, raw_data[iArrayIndex], open_rawdata_N[iArrayIndex], open_rawdata_P[iArrayIndex]);
                }
                err_cnt++;
            }
        }
        if (nvt_testdata->fd >= 0) {
            store_to_file(nvt_testdata->fd, "\n");
        }
    }

OUT:
    //---Reset IC---
    nvt_bootloader_reset(chip_info);
    if (raw_data)
        kfree(raw_data);
    if (raw_data_n)
        kfree(raw_data_n);

    seq_printf(s, "FW:0x%llx\n", nvt_testdata->TP_FW);
    seq_printf(s, "%d error(s). %s\n", err_cnt, err_cnt?"":"All test passed.");
    TPD_INFO(" TP auto test %d error(s). %s\n", err_cnt, err_cnt?"":"All test passed.");
}

static struct nvt_proc_operations nvt_proc_ops = {
    .auto_test     = nvt_auto_test,
};
/*************** End of implementation of baseline_test proc callbacks************************/

/*********** Start of I2C Driver and Implementation of it's callbacks*************************/
static int nvt_tp_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct chip_data_nt36672 *chip_info = NULL;
    struct touchpanel_data *ts = NULL;
    int ret = -1;

    TPD_INFO("%s  is called\n", __func__);

    /* 1. alloc chip info */
    chip_info = kzalloc(sizeof(struct chip_data_nt36672), GFP_KERNEL);
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

    /* 4. file_operations callbacks binding */
    ts->ts_ops = &nvt_ops;

    /* 5. register common touch device*/
    ret = register_common_touch_device(ts);
    if (ret < 0 && (ret != -EFTM)) {
        goto err_register_driver;
    }
    ts->tp_suspend_order = TP_LCD_SUSPEND;
    ts->tp_resume_order = LCD_TP_RESUME;
    chip_info->is_sleep_writed = false;
    chip_info->test_limit_name = ts->panel_data.test_limit_name;

    /* 7. create nvt test files */
    nvt_flash_proc_init(ts, "NVTflash");
    nvt_create_proc(ts, &nvt_proc_ops);

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

static int nvt_tp_remove(struct i2c_client *client)
{
    struct touchpanel_data *ts = i2c_get_clientdata(client);

    TPD_INFO("%s is called\n", __func__);
    kfree(ts);

    return 0;
}

static int nvt_i2c_suspend(struct device *dev)
{
    struct touchpanel_data *ts = dev_get_drvdata(dev);

    TPD_INFO("%s: is called\n", __func__);
    tp_i2c_suspend(ts);

    return 0;
}

static int nvt_i2c_resume(struct device *dev)
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
    .suspend = nvt_i2c_suspend,
    .resume = nvt_i2c_resume,
#endif
};

static struct i2c_driver tp_i2c_driver =
{
    .probe = nvt_tp_probe,
    .remove = nvt_tp_remove,
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

MODULE_AUTHOR("nvtTek Driver");
MODULE_DESCRIPTION("nvtTek Touchscreen Driver");
MODULE_LICENSE("GPL v2");
