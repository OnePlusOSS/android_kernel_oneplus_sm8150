/***************************************************
 * File:novatek_common.c
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

#include "../touchpanel_common.h"
#include "novatek_common.h"

/*******LOG TAG Declear*****************************/

#define TPD_DEVICE "nvt_common"
#define TPD_INFO(a, arg...)  pr_err("[TP]"TPD_DEVICE ": " a, ##arg)
#define TPD_DEBUG(a, arg...)\
    do{\
        if (tp_debug)\
        pr_err("[TP]"TPD_DEVICE ": " a, ##arg);\
    }while(0)
#define TPD_DETAIL(a, arg...)\
    do{\
        if (LEVEL_BASIC != tp_debug)\
            pr_err("[TP]"TPD_DEVICE ": " a, ##arg);\
    }while(0)

/*********** nvt tool operate content***********************/
static ssize_t nvt_flash_read(struct file *filp, char __user *buff, size_t count, loff_t *offp)
{
    uint8_t str[68] = {0};
    int32_t ret = -1;
    int32_t retries = 0;
    int8_t i2c_wr = 0;
    unsigned short addr_tmp = 0;

    struct touchpanel_data *ts = PDE_DATA(file_inode(filp));

    if (count > sizeof(str)) {
        TPD_INFO("error count=%zu\n", count);
        return -EFAULT;
    }

    if (copy_from_user(str, buff, count)) {
        TPD_INFO("copy from user error\n");
        return -EFAULT;
    }

    if (ts->esd_handle_support) {
        esd_handle_switch(&ts->esd_info, false);
    }

    i2c_wr = str[0] >> 7;

    if (i2c_wr == 0) {    //I2C write
        while (retries < 20) {
            addr_tmp = ts->client->addr;
            ts->client->addr = str[0] & 0x7F;
            ret = touch_i2c_write(ts->client, &str[2], str[1]);
            ts->client->addr = addr_tmp;
            if (ret == 1)
                break;
            else
                TPD_INFO("error, retries=%d, ret=%d\n", retries, ret);

            retries++;
        }

        if (unlikely(retries == 20)) {
            TPD_INFO("error, ret = %d\n", ret);
            return -EIO;
        }

        return ret;
    } else if (i2c_wr == 1) {    //I2C read
        while (retries < 20) {
            addr_tmp = ts->client->addr;
            ts->client->addr = str[0] & 0x7F;
            ret = touch_i2c_read(ts->client, &str[2], 1, &str[3], str[1] - 1);
            ts->client->addr = addr_tmp;
            if (ret == 2)
                break;
            else
                TPD_INFO("error, retries=%d, ret=%d\n", retries, ret);

            retries++;
        }

        // copy buff to user if i2c transfer
        if (retries < 20) {
            if (copy_to_user(buff, str, count))
                return -EFAULT;
        }

        if (unlikely(retries == 20)) {
            TPD_INFO("error, ret = %d\n", ret);
            return -EIO;
        }

        return ret;
    } else {
        TPD_INFO("Call error, str[0]=%d\n", str[0]);
        return -EFAULT;
    }
}

static const struct file_operations nvt_flash_fops = {
    .owner = THIS_MODULE,
    .open = simple_open,
    .read = nvt_flash_read,
};

static ssize_t nvt_noflash_read(struct file *filp, char __user *buff, size_t count, loff_t *offp)
{
    uint8_t *str = NULL;
    int32_t ret = -1;
    int32_t retries = 0;
    int8_t spi_wr = 0;
    uint8_t *buf;

    struct touchpanel_data *ts = PDE_DATA(file_inode(filp));

    if (count > SPI_TANSFER_LEN)
        return -EFAULT;

    /* allocate buffer for spi transfer */
    str = (uint8_t *)kzalloc((count), GFP_KERNEL | GFP_DMA);
    if(str == NULL) {
        TPD_INFO("kzalloc for buf failed!\n");
        ret = -ENOMEM;
        goto kzalloc_failed;
    }

    buf = (uint8_t *)kzalloc((count), GFP_KERNEL | GFP_DMA);
    if(buf == NULL) {
        TPD_INFO("kzalloc for buf failed!\n");
        ret = -ENOMEM;
        kfree(str);
        str = NULL;
        goto kzalloc_failed;
    }

    if (copy_from_user(str, buff, count)) {
        TPD_INFO("copy from user error\n");
        ret = -EFAULT;
        goto out;
    }

    spi_wr = str[0] >> 7;
    memcpy(buf, str+2, ((str[0] & 0x7F) << 8) | str[1]);

    if (spi_wr == 0) {    //SPI write
        while (retries < 20) {
            ret = CTP_SPI_WRITE(ts->s_client, buf, ((str[0] & 0x7F) << 8) | str[1]);
            if (!ret)
                break;
            else
                TPD_INFO("error, retries=%d, ret=%d\n", retries, ret);

            retries++;
        }

        if (unlikely(retries == 20)) {
            TPD_INFO("error, ret = %d\n", ret);
            ret = -EIO;
            goto out;
        }
    } else if (spi_wr == 1) {    //SPI read
         while (retries < 20) {
            ret = CTP_SPI_READ(ts->s_client, buf, ((str[0] & 0x7F) << 8) | str[1]);
            if (!ret)
                break;
            else
                TPD_INFO("error, retries=%d, ret=%d\n", retries, ret);

            retries++;
        }

        // copy buff to user if spi transfer
        memcpy(str+2, buf, ((str[0] & 0x7F) << 8) | str[1]);
        if (retries < 20) {
            if (copy_to_user(buff, str, count)) {
                ret = -EFAULT;
                goto out;
            }
        }

        if (unlikely(retries == 20)) {
            TPD_INFO("error, ret = %d\n", ret);
            ret = -EIO;
            goto out;
        }
    } else {
        TPD_INFO("Call error, str[0]=%d\n", str[0]);
        ret = -EFAULT;
        goto out;
    }

out:
    kfree(str);
    str = NULL;
    kfree(buf);
    buf = NULL;
kzalloc_failed:
        return ret;
}

static const struct file_operations nvt_noflash_fops = {
    .owner = THIS_MODULE,
    .open = simple_open,
    .read = nvt_noflash_read,
};

void nvt_flash_proc_init(struct touchpanel_data *ts, const char *name)
{
    struct proc_dir_entry *nvt_proc_entry;

    if (strstr(name, "SPI")) {
        TPD_INFO("create /proc/NVTSPI!\n");
        nvt_proc_entry = proc_create_data(name, 0444, NULL, &nvt_noflash_fops, ts);
        if (nvt_proc_entry == NULL) {
                TPD_INFO("%s Failed!\n", __func__);
                return;
        } else {
                TPD_INFO("%s Succeeded!\n", __func__);
        }
    } else {
        TPD_INFO("create /proc/NVTflash!\n");
        nvt_proc_entry = proc_create_data(name, 0444, NULL,&nvt_flash_fops, ts);
        if (nvt_proc_entry == NULL) {
                TPD_INFO("%s Failed!\n", __func__);
                return;
        } else {
                TPD_INFO("%s Succeeded!\n", __func__);
        }
    }

    return;
}

void nvt_limit_read(struct seq_file *s, struct touchpanel_data *ts)
{
    int ret = 0;
    int32_t *fw_rawdata_p = NULL, *fw_rawdata_n = NULL;
    int32_t *open_rawdata_p = NULL, *open_rawdata_n = NULL;
    int32_t *short_rawdata_p = NULL, *short_rawdata_n = NULL;
    int32_t *diff_rawdata_p = NULL, *diff_rawdata_n = NULL;
    int32_t *cc_data_p = NULL, *cc_data_n = NULL;
    int32_t *doze_rawdata_p = NULL, *doze_rawdata_n = NULL;
    int32_t *doze_diff_rawdata_p = NULL, *doze_diff_rawdata_n = NULL;
    int32_t *lpwg_rawdata_p = NULL, *lpwg_rawdata_n = NULL;
    int32_t *lpwg_diff_rawdata_p = NULL, *lpwg_diff_rawdata_n = NULL;
    const struct firmware *fw = NULL;
    struct nvt_test_header *ph = NULL;
    int i = 0, j = 0;
    int temp = 0;

    ret = request_firmware(&fw, ts->panel_data.test_limit_name, ts->dev);
    if (ret < 0) {
        TPD_INFO("Request firmware failed - %s (%d)\n", ts->panel_data.test_limit_name, ret);
        seq_printf(s, "Request failed, Check the path %d", temp);
        return;
    }

    ph = (struct nvt_test_header *)(fw->data);
    fw_rawdata_p = (int32_t *)(fw->data + ph->array_fw_rawdata_P_offset);
    fw_rawdata_n = (int32_t *)(fw->data + ph->array_fw_rawdata_N_offset);
    open_rawdata_p = (int32_t *)(fw->data + ph->array_open_rawdata_P_offset);
    open_rawdata_n = (int32_t *)(fw->data + ph->array_open_rawdata_N_offset);
    short_rawdata_p = (int32_t *)(fw->data + ph->array_Short_Rawdata_P_offset);
    short_rawdata_n = (int32_t *)(fw->data + ph->array_Short_Rawdata_N_offset);
    diff_rawdata_p = (int32_t *)(fw->data + ph->array_FW_Diff_P_offset);
    diff_rawdata_n = (int32_t *)(fw->data + ph->array_FW_Diff_N_offset);
    cc_data_p = (int32_t *)(fw->data + ph->array_FW_CC_P_offset);
    cc_data_n = (int32_t *)(fw->data + ph->array_FW_CC_N_offset);
    doze_rawdata_p = (int32_t *)(fw->data + ph->array_Doze_Rawdata_P_offset);
    doze_rawdata_n = (int32_t *)(fw->data + ph->array_Doze_Rawdata_N_offset);
    doze_diff_rawdata_p = (int32_t *)(fw->data + ph->array_Doze_Diff_P_offset);
    doze_diff_rawdata_n = (int32_t *)(fw->data + ph->array_Doze_Diff_N_offset);
    lpwg_rawdata_p = (int32_t *)(fw->data + ph->array_LPWG_Rawdata_P_offset);
    lpwg_rawdata_n = (int32_t *)(fw->data + ph->array_LPWG_Rawdata_N_offset);
    lpwg_diff_rawdata_p = (int32_t *)(fw->data + ph->array_LPWG_Diff_P_offset);
    lpwg_diff_rawdata_n = (int32_t *)(fw->data + ph->array_LPWG_Diff_N_offset);

    seq_printf(s, "\n FW_Rawdata_P:");
    for (i = 0 ; i < ts->hw_res.RX_NUM; i++) {
        seq_printf(s, "\n[%2d] ", i);
        for (j = 0 ; j < ts->hw_res.TX_NUM; j++) {
            seq_printf(s, "%4d, ", fw_rawdata_p[i * ts->hw_res.TX_NUM + j]);
        }
    }

    seq_printf(s, "\n FW_Rawdata_N:");
    for (i = 0 ; i < ts->hw_res.RX_NUM; i++) {
        seq_printf(s, "\n[%2d] ", i);
        for (j = 0 ; j < ts->hw_res.TX_NUM; j++) {
            seq_printf(s, "%4d, ", fw_rawdata_n[i * ts->hw_res.TX_NUM + j]);
        }
    }

    seq_printf(s, "\n Open_Rawdata_P:");
    for (i = 0 ; i < ts->hw_res.RX_NUM; i++) {
        seq_printf(s, "\n[%2d] ", i);
        for (j = 0 ; j < ts->hw_res.TX_NUM; j++) {
            seq_printf(s, "%4d, ", open_rawdata_p[i * ts->hw_res.TX_NUM + j]);
        }
    }

    seq_printf(s, "\n Open_Rawdata_N:");
    for (i = 0 ; i < ts->hw_res.RX_NUM; i++) {
        seq_printf(s, "\n[%2d] ", i);
        for (j = 0 ; j < ts->hw_res.TX_NUM; j++) {
            seq_printf(s, "%4d, ", open_rawdata_n[i * ts->hw_res.TX_NUM + j]);
        }
    }

    if (ph->config_Lmt_Short_Rawdata_P != 0) {
        seq_printf(s, "\n config_Lmt_Short_Rawdata_P: %4d", ph->config_Lmt_Short_Rawdata_P);
    } else {
        seq_printf(s, "\n Short_Rawdata_P:");
        for (i = 0 ; i < ts->hw_res.RX_NUM; i++) {
            seq_printf(s, "\n[%2d] ", i);
            for (j = 0 ; j < ts->hw_res.TX_NUM; j++) {
                seq_printf(s, "%4d, ", short_rawdata_p[i * ts->hw_res.TX_NUM + j]);
            }
        }
    }

    if (ph->config_Lmt_Short_Rawdata_N != 0) {
        seq_printf(s, "\n config_Lmt_Short_Rawdata_N: %4d", ph->config_Lmt_Short_Rawdata_N);
    } else {
        seq_printf(s, "\n Short_Rawdata_N:");
        for (i = 0 ; i < ts->hw_res.RX_NUM; i++) {
            seq_printf(s, "\n[%2d] ", i);
            for (j = 0 ; j < ts->hw_res.TX_NUM; j++) {
                seq_printf(s, "%4d, ", short_rawdata_n[i * ts->hw_res.TX_NUM + j]);
            }
        }
    }

    seq_printf(s, "\n config_Diff_Test_Frame: %4d", ph->config_Diff_Test_Frame);

    if (ph->config_Lmt_FW_Diff_P != 0) {
        seq_printf(s, "\n config_Lmt_FW_Diff_P: %4d", ph->config_Lmt_FW_Diff_P);
    } else {
        seq_printf(s, "\n Diff_Rawdata_P:");
        for (i = 0 ; i < ts->hw_res.RX_NUM; i++) {
            seq_printf(s, "\n[%2d] ", i);
            for (j = 0 ; j < ts->hw_res.TX_NUM; j++) {
                seq_printf(s, "%4d, ", diff_rawdata_p[i * ts->hw_res.TX_NUM + j]);
            }
        }
    }

    if (ph->config_Lmt_FW_Diff_N != 0) {
        seq_printf(s, "\n config_Lmt_FW_Diff_N: %4d", ph->config_Lmt_FW_Diff_N);
    } else {
        seq_printf(s, "\n Diff_Rawdata_N:");
        for (i = 0 ; i < ts->hw_res.RX_NUM; i++) {
            seq_printf(s, "\n[%2d] ", i);
            for (j = 0 ; j < ts->hw_res.TX_NUM; j++) {
                seq_printf(s, "%4d, ", diff_rawdata_n[i * ts->hw_res.TX_NUM + j]);
            }
        }
    }

    if (ph->config_Lmt_FW_CC_P != 0) {
        seq_printf(s, "\n config_Lmt_FW_CC_P: %4d", ph->config_Lmt_FW_CC_P);
    } else {
        seq_printf(s, "\n FW_Cc_data_P:");
        for (i = 0 ; i < ts->hw_res.RX_NUM; i++) {
            seq_printf(s, "\n[%2d] ", i);
            for (j = 0 ; j < ts->hw_res.TX_NUM; j++) {
                seq_printf(s, "%4d, ", cc_data_p[i * ts->hw_res.TX_NUM + j]);
            }
        }
    }

    if (ph->config_Lmt_FW_CC_N != 0) {
        seq_printf(s, "\n config_Lmt_FW_CC_N: %4d", ph->config_Lmt_FW_CC_N);
    } else {
        seq_printf(s, "\n FW_Cc_data_N:");
        for (i = 0 ; i < ts->hw_res.RX_NUM; i++) {
            seq_printf(s, "\n[%2d] ", i);
            for (j = 0 ; j < ts->hw_res.TX_NUM; j++) {
                seq_printf(s, "%4d, ", cc_data_n[i * ts->hw_res.TX_NUM + j]);
            }
        }
    }

    seq_printf(s, "\n doze_X_Channel: %4d", ph->doze_X_Channel);

    if (ph->config_Lmt_Doze_Rawdata_P != 0) {
        seq_printf(s, "\n config_Lmt_Doze_Rawdata_P: %4d", ph->config_Lmt_Doze_Rawdata_P);
    } else {
        seq_printf(s, "\n Doze_Rawdata_P:");
        for (i = 0 ; i < ts->hw_res.RX_NUM; i++) {
            seq_printf(s, "\n[%2d] ", i);
            for (j = 0 ; j < ph->doze_X_Channel; j++) {
                seq_printf(s, "%4d, ", doze_rawdata_p[i * ph->doze_X_Channel + j]);
            }
        }
    }

    if (ph->config_Lmt_Doze_Rawdata_N != 0) {
        seq_printf(s, "\n config_Lmt_Doze_Rawdata_N: %4d", ph->config_Lmt_Doze_Rawdata_N);
    } else {
        seq_printf(s, "\n Doze_Rawdata_N:");
        for (i = 0 ; i < ts->hw_res.RX_NUM; i++) {
            seq_printf(s, "\n[%2d] ", i);
            for (j = 0 ; j < ph->doze_X_Channel; j++) {
                seq_printf(s, "%4d, ", doze_rawdata_n[i * ph->doze_X_Channel + j]);
            }
        }
    }

    seq_printf(s, "\n config_Doze_Noise_Test_Frame: %4d", ph->config_Doze_Noise_Test_Frame);

    if (ph->config_Lmt_Doze_Diff_P != 0) {
        seq_printf(s, "\n config_Lmt_Doze_Diff_P: %4d", ph->config_Lmt_Doze_Diff_P);
    } else {
        seq_printf(s, "\n Doze_Diff_P:");
        for (i = 0 ; i < ts->hw_res.RX_NUM; i++) {
            seq_printf(s, "\n[%2d] ", i);
            for (j = 0 ; j < ph->doze_X_Channel; j++) {
                seq_printf(s, "%4d, ", doze_diff_rawdata_p[i * ph->doze_X_Channel + j]);
            }
        }
    }

    if (ph->config_Lmt_Doze_Diff_N != 0) {
        seq_printf(s, "\n config_Lmt_Doze_Diff_N: %4d", ph->config_Lmt_Doze_Diff_N);
    } else {
        seq_printf(s, "\n Doze_Diff_N:");
        for (i = 0 ; i < ts->hw_res.RX_NUM; i++) {
            seq_printf(s, "\n[%2d] ", i);
            for (j = 0 ; j < ph->doze_X_Channel; j++) {
                seq_printf(s, "%4d, ", doze_diff_rawdata_n[i * ph->doze_X_Channel + j]);
            }
        }
    }

    if (ph->config_Lmt_LPWG_Rawdata_P != 0) {
        seq_printf(s, "\n config_Lmt_LPWG_Rawdata_P: %4d", ph->config_Lmt_LPWG_Rawdata_P);
    } else {
        seq_printf(s, "\n Lpwg_Rawdata_P:");
        for (i = 0 ; i < ts->hw_res.RX_NUM; i++) {
            seq_printf(s, "\n[%2d] ", i);
            for (j = 0 ; j < ts->hw_res.TX_NUM; j++) {
                seq_printf(s, "%4d, ", lpwg_rawdata_p[i * ts->hw_res.TX_NUM + j]);
            }
        }
    }

    if (ph->config_Lmt_LPWG_Rawdata_N != 0) {
        seq_printf(s, "\n config_Lmt_LPWG_Rawdata_N: %4d", ph->config_Lmt_LPWG_Rawdata_N);
    } else {
        seq_printf(s, "\n Lpwg_Rawdata_N:");
        for (i = 0 ; i < ts->hw_res.RX_NUM; i++) {
            seq_printf(s, "\n[%2d] ", i);
            for (j = 0 ; j < ts->hw_res.TX_NUM; j++) {
                seq_printf(s, "%4d, ", lpwg_rawdata_n[i * ts->hw_res.TX_NUM + j]);
            }
        }
    }

    if (ph->config_Lmt_LPWG_Diff_P != 0) {
        seq_printf(s, "\n config_Lmt_LPWG_Diff_P: %4d", ph->config_Lmt_LPWG_Diff_P);
    } else {
        seq_printf(s, "\n Lpwg_Diff_P:");
        for (i = 0 ; i < ts->hw_res.RX_NUM; i++) {
            seq_printf(s, "\n[%2d] ", i);
            for (j = 0 ; j < ts->hw_res.TX_NUM; j++) {
                seq_printf(s, "%4d, ", lpwg_diff_rawdata_p[i * ts->hw_res.TX_NUM + j]);
            }
        }
    }

    if (ph->config_Lmt_LPWG_Diff_N != 0) {
        seq_printf(s, "\n config_Lmt_LPWG_Diff_N: %4d", ph->config_Lmt_LPWG_Diff_N);
    } else {
        seq_printf(s, "\n Lpwg_Diff_N:");
        for (i = 0 ; i < ts->hw_res.RX_NUM; i++) {
            seq_printf(s, "\n[%2d] ", i);
            for (j = 0 ; j < ts->hw_res.TX_NUM; j++) {
                seq_printf(s, "%4d, ", lpwg_diff_rawdata_n[i * ts->hw_res.TX_NUM + j]);
            }
        }
    }

    seq_printf(s, "\n");
    release_firmware(fw);
}

/************ nvt auto test content*************************/

static int tp_auto_test_read_func(struct seq_file *s, void *v)
{
    struct touchpanel_data *ts = s->private;
    struct nvt_proc_operations *nvt_ops;
    const struct firmware *fw = NULL;
    struct timespec now_time;
    struct rtc_time rtc_now_time;
    mm_segment_t old_fs;
    uint8_t data_buf[128];
    int fd = -1, ret = -1;

    struct nvt_testdata nvt_testdata =
    {
        .TX_NUM = 0,
        .RX_NUM = 0,
        .fd = -1,
        .irq_gpio = -1,
        .key_TX = 0,
        .key_RX = 0,
        .TP_FW = 0,
        .fw = NULL,
    };

    if (!ts)
        return 0;
    nvt_ops = (struct nvt_proc_operations *)ts->private_data;
    if (!nvt_ops)
        return 0;
    if (!nvt_ops->auto_test) {
        seq_printf(s, "Not support auto-test proc node\n");
        return 0;
    }

    //step1:disable_irq && get mutex locked
    disable_irq_nosync(ts->irq);
    mutex_lock(&ts->mutex);

    if (ts->esd_handle_support) {
        esd_handle_switch(&ts->esd_info, false);
    }

    //step2: create a file to store test data in /sdcard/Tp_Test
    getnstimeofday(&now_time);
    rtc_time_to_tm(now_time.tv_sec, &rtc_now_time);
    snprintf(data_buf, 128, "/sdcard/TpTestReport/screenOn/tp_testlimit_%02d%02d%02d-%02d%02d%02d-utc.csv",
            (rtc_now_time.tm_year + 1900) % 100, rtc_now_time.tm_mon + 1, rtc_now_time.tm_mday,
            rtc_now_time.tm_hour, rtc_now_time.tm_min, rtc_now_time.tm_sec);
    old_fs = get_fs();
    set_fs(KERNEL_DS);
    sys_mkdir("/sdcard/TpTestReport", 0666);
    sys_mkdir("/sdcard/TpTestReport/screenOn", 0666);
    fd = sys_open(data_buf, O_WRONLY | O_CREAT | O_TRUNC, 0);
    if (fd < 0) {
        TPD_INFO("Open log file '%s' failed.\n", data_buf);
        set_fs(old_fs);
        mutex_unlock(&ts->mutex);
        enable_irq(ts->irq);

        return 0;
    }

    //step3:request test limit data from userspace
    ret = request_firmware(&fw, ts->panel_data.test_limit_name, ts->dev);
    if (ret < 0) {
        TPD_INFO("Request firmware failed - %s (%d)\n", ts->panel_data.test_limit_name, ret);
        seq_printf(s, "No limit IMG\n");
        sys_close(fd);
        set_fs(old_fs);
        mutex_unlock(&ts->mutex);
        enable_irq(ts->irq);
        return 0;
    }

    //step3:init syna_testdata
    nvt_testdata.fd = fd;
    nvt_testdata.TX_NUM = ts->hw_res.TX_NUM;
    nvt_testdata.RX_NUM = ts->hw_res.RX_NUM;
    nvt_testdata.irq_gpio = ts->hw_res.irq_gpio;
    nvt_testdata.key_TX = ts->hw_res.key_TX;
    nvt_testdata.key_RX = ts->hw_res.key_RX;
    nvt_testdata.TP_FW = ts->panel_data.TP_FW;
    nvt_testdata.fw = fw;

    nvt_ops->auto_test(s, ts->chip_data, &nvt_testdata);

    //step4: close file && release test limit firmware
    if (fd >= 0) {
        sys_close(fd);
        set_fs(old_fs);
    }
    release_firmware(fw);

    //step5: return to normal mode
    ts->ts_ops->reset(ts->chip_data);
    operate_mode_switch(ts);

    //step6: unlock the mutex && enable irq trigger
    mutex_unlock(&ts->mutex);
    enable_irq(ts->irq);

    return 0;
}

static int baseline_autotest_open(struct inode *inode, struct file *file)
{
    return single_open(file, tp_auto_test_read_func, PDE_DATA(inode));
}

static const struct file_operations tp_auto_test_proc_fops = {
    .owner = THIS_MODULE,
    .open  = baseline_autotest_open,
    .read  = seq_read,
    .release = single_release,
};

//proc/touchpanel/baseline_test
int nvt_create_proc(struct touchpanel_data *ts, struct nvt_proc_operations *nvt_ops)
{
    int ret = 0;

    // touchpanel_auto_test interface
    struct proc_dir_entry *prEntry_tmp = NULL;
    ts->private_data = nvt_ops;
    prEntry_tmp = proc_create_data("baseline_test", 0666, ts->prEntry_tp, &tp_auto_test_proc_fops, ts);
    if (prEntry_tmp == NULL) {
        ret = -ENOMEM;
        TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
    }
    return ret;
}
