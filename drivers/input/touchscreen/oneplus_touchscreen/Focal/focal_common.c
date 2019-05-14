/***************************************************
 * File:synaptics_common.c
 * Copyright (c)  2008- 2030  Oppo Mobile communication Corp.ltd.
 * Description:
 *             focal common driver
 * Version:1.0:
 * Date created:2017/06/18
 * Author: Cong.Dai@Bsp.Driver
 * TAG: BSP.TP.Init
 * *
 * -------------- Revision History: -----------------
 *  <author >  <data>  <version>  <desc>
 ***************************************************/

#include "../touchpanel_common.h"
#include "focal_common.h"

/*******LOG TAG Declear*****************************/

#define TPD_DEVICE "focal_common"
#define TPD_INFO(a, arg...)  pr_err("[TP]"TPD_DEVICE ": " a, ##arg)
#define TPD_DEBUG(a, arg...)\
    do{\
        if (tp_debug)\
        pr_err("[TP]"TPD_DEVICE ": " a, ##arg);\
    }while(0)


/************ Start of other functions work for device attribute file*************************/


static int is_hex_char(const char ch)
{
    if ((ch >= '0' && ch <= '9') || (ch >= 'a' && ch <= 'f') || (ch >= 'A' && ch <= 'F')) {
        return 1;
    }

    return 0;
}

static int hex_char_to_int(const char ch)
{
    int result = 0;
    if (ch >= '0' && ch <= '9') {
        result = (int)(ch - '0');
    } else if (ch >= 'a' && ch <= 'f') {
        result = (int)(ch - 'a') + 10;
    } else if (ch >= 'A' && ch <= 'F') {
        result = (int)(ch - 'A') + 10;
    } else {
        result = -1;
    }

    return result;
}

static int hex_to_str(char *hex, int iHexLen, char *ch, int *iChLen)
{
    int high=0;
    int low=0;
    int tmp = 0;
    int i = 0;
    int iCharLen = 0;
    if (hex == NULL || ch == NULL) {
        return -1;
    }

    TPD_INFO("iHexLen: %d in function:%s!!\n", iHexLen, __func__);

    if (iHexLen %2 == 1) {
        return -2;
    }

    for (i=0; i<iHexLen; i+=2) {
        high = hex_char_to_int(hex[i]);
        if (high < 0) {
            ch[iCharLen] = '\0';
            return -3;
        }

        low = hex_char_to_int(hex[i+1]);
        if (low < 0) {
            ch[iCharLen] = '\0';
            return -3;
        }
        tmp = (high << 4) + low;
        ch[iCharLen++] = (char)tmp;
    }
    ch[iCharLen] = '\0';
    *iChLen = iCharLen;
    TPD_INFO("iCharLen: %d, iChLen: %d in function:%s!!\n", iCharLen, *iChLen, __func__);
    return 0;
}

static void str_to_bytes(char * bufStr, int iLen, char* uBytes, int *iBytesLen)
{
    int i=0;
    int iNumChLen=0;
    *iBytesLen=0;

    for (i=0; i<iLen; i++) {
        //filter illegal chars
        if (is_hex_char(bufStr[i])) {
            bufStr[iNumChLen++] = bufStr[i];
        }
    }

    bufStr[iNumChLen] = '\0';
    hex_to_str(bufStr, iNumChLen, uBytes, iBytesLen);
}
/***************** End of other functions work for device attribute file*********************/


static struct
{
    int op;         // 0: read, 1: write
    int reg;        // register
    int value;      // read: return value, write: op return
    int result;     // 0: success, otherwise: fail
} g_rwreg_result;
/************************** Start of device attribute file***********************************/
static ssize_t focal_hw_reset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    ssize_t count = 0;
    struct touchpanel_data *ts = dev_get_drvdata(dev);
    struct focal_debug_func *focal_debug_ops = (struct focal_debug_func *)ts->private_data;

    if (focal_debug_ops && focal_debug_ops->reset) {
        mutex_lock(&ts->mutex);
        focal_debug_ops->reset(ts->chip_data, 200);
        mutex_unlock(&ts->mutex);
    }

    count = snprintf(buf, PAGE_SIZE, "hw reset executed\n");
    return count;
}

static ssize_t focal_irq_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct touchpanel_data *ts = dev_get_drvdata(dev);

    if ((strcmp(buf, "1")  == 0) || (strcmp(buf, "on") == 0)) {
        if (ts) {
            enable_irq(ts->irq);
        }
        TPD_INFO("[EX-FUN]enable irq\n");
    } else if ((strcmp(buf, "0")  == 0) || (strcmp(buf, "off") == 0)) {
        if (ts) {
            disable_irq_nosync(ts->irq);
        }
        TPD_INFO("[EX-FUN]disable irq\n");
    }
    return count;
}

static ssize_t focal_fw_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    ssize_t num_read_chars = 0;
    u8 fw_version = 0;
    struct touchpanel_data *ts = dev_get_drvdata(dev);
    struct focal_debug_func *focal_debug_ops = (struct focal_debug_func *)ts->private_data;

    if (ts->esd_handle_support) {
        if (focal_debug_ops && focal_debug_ops->esd_check_enable) {
            focal_debug_ops->esd_check_enable(ts->chip_data, false);
        }
    }

    if (focal_debug_ops && focal_debug_ops->get_fw_version) {
        mutex_lock(&ts->mutex);
        fw_version = focal_debug_ops->get_fw_version(ts->chip_data);
        mutex_unlock(&ts->mutex);
        if (fw_version < 0) {
            num_read_chars = snprintf(buf, PAGE_SIZE,"I2c transfer error!\n");
        }

        if (fw_version == 255) {
            num_read_chars = snprintf(buf, PAGE_SIZE,"get tp fw version fail!\n");
        } else {
            num_read_chars = snprintf(buf, PAGE_SIZE, "%02X\n", fw_version);
        }
    }

    if (ts->esd_handle_support) {
        if (focal_debug_ops && focal_debug_ops->esd_check_enable) {
            focal_debug_ops->esd_check_enable(ts->chip_data, true);
        }
    }

    return num_read_chars;
}


static ssize_t focal_rw_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int count;

    if (!g_rwreg_result.op) {
        if (g_rwreg_result.result == 0) {
            count = sprintf(buf, "Read %02X: %02X\n", g_rwreg_result.reg, g_rwreg_result.value);
        } else {
            count = sprintf(buf, "Read %02X failed, ret: %d\n",g_rwreg_result.reg,  g_rwreg_result.result);
        }
    } else {
        if (g_rwreg_result.result == 0) {
            count = sprintf(buf, "Write %02X, %02X success\n",g_rwreg_result.reg,  g_rwreg_result.value);
        } else {
            count = sprintf(buf, "Write %02X failed, ret: %d\n",g_rwreg_result.reg,  g_rwreg_result.result);
        }
    }

    return count;
}

static ssize_t focal_rw_reg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct touchpanel_data *ts = dev_get_drvdata(dev);
    struct focal_debug_func *focal_debug_ops = (struct focal_debug_func *)ts->private_data;
    ssize_t num_read_chars = 0;
    int retval;
    long unsigned int wmreg = 0;
    u8 regaddr = 0xff, regvalue = 0xff;
    u8 valbuf[5] = {0};

    memset(valbuf, 0, sizeof(valbuf));
    num_read_chars = count - 1;
    if ((num_read_chars != 2) && (num_read_chars != 4)) {
        TPD_INFO("please input 2 or 4 character\n");
        goto error_return;
    }

    memcpy(valbuf, buf, num_read_chars);
    retval = kstrtoul(valbuf, 16, &wmreg);
    str_to_bytes((char*)buf, num_read_chars, valbuf, &retval);

    if (1 == retval) {
        regaddr = valbuf[0];
        retval = 0;
    } else if (2 == retval) {
        regaddr = valbuf[0];
        regvalue = valbuf[1];
        retval = 0;
    } else {
        retval =-1;
    }

    if (0 != retval) {
        TPD_INFO("%s() - ERROR: Could not convert the given input to a number. The given input was: %s\n", __func__, buf);
        goto error_return;
    }

    if (ts->esd_handle_support) {
        if (focal_debug_ops && focal_debug_ops->esd_check_enable) {
            focal_debug_ops->esd_check_enable(ts->chip_data, false);
        }
    }

    mutex_lock(&ts->mutex);

    if (2 == num_read_chars) {
        g_rwreg_result.op = 0;
        g_rwreg_result.reg = regaddr;
        /*read register*/
        regaddr = wmreg;
        regvalue = touch_i2c_read_byte(ts->client, regaddr);
        if (regvalue < 0)  {
            TPD_INFO("Could not read the register(0x%02x)\n", regaddr);
            g_rwreg_result.result = -1;
        } else {
            TPD_INFO("the register(0x%02x) is 0x%02x\n", regaddr, regvalue);
            g_rwreg_result.value = regvalue;
            g_rwreg_result.result = 0;
        }
    } else {
        regaddr = wmreg>>8;
        regvalue = wmreg;

        g_rwreg_result.op = 1;
        g_rwreg_result.reg = regaddr;
        g_rwreg_result.value = regvalue;
        g_rwreg_result.result = touch_i2c_write_byte(ts->client, regaddr, regvalue);
        if (g_rwreg_result.result < 0) {
            TPD_INFO("Could not write the register(0x%02x)\n", regaddr);

        } else {
            TPD_INFO("Write 0x%02x into register(0x%02x) successful\n", regvalue, regaddr);
            g_rwreg_result.result = 0;
        }
    }

    mutex_unlock(&ts->mutex);

    if (ts->esd_handle_support) {
        if (focal_debug_ops && focal_debug_ops->esd_check_enable) {
            focal_debug_ops->esd_check_enable(ts->chip_data, true);
        }
    }
error_return:

    return count;
}

static ssize_t focal_esdcheck_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct touchpanel_data *ts = dev_get_drvdata(dev);

    if (ts->esd_handle_support) {
        if ((strcmp(buf, "1")  == 0) || (strcmp(buf, "on") == 0)) {
            esd_handle_switch(&ts->esd_info, true);
        } else if ((strcmp(buf, "0")  == 0) || (strcmp(buf, "off") == 0)) {
            esd_handle_switch(&ts->esd_info, false);
        }
    }

    return count;
}

static ssize_t focal_esdcheck_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int count = 0;
    struct touchpanel_data *ts = dev_get_drvdata(dev);
    struct focal_debug_func *focal_debug_ops = (struct focal_debug_func *)ts->private_data;

    if (ts->esd_handle_support) {
        if (focal_debug_ops && focal_debug_ops->get_esd_check_flag) {
            count = sprintf(buf, "esd_running_flag: %d,  esd_check_need_stop: %d\n", \
                            ts->esd_info.esd_running_flag, focal_debug_ops->get_esd_check_flag(ts->chip_data));
        } else {
            count = sprintf(buf, "esd_running_flag: %d\n", ts->esd_info.esd_running_flag);
        }
    } else {
        count = sprintf(buf, "not support esd handle\n");
    }

    return count;
}

static ssize_t focal_dump_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    char tmp[256];
    int count = 0;
    struct touchpanel_data *ts = dev_get_drvdata(dev);
    struct focal_debug_func *focal_debug_ops = (struct focal_debug_func *)ts->private_data;

    if (ts->esd_handle_support) {
        if (focal_debug_ops && focal_debug_ops->esd_check_enable) {
            focal_debug_ops->esd_check_enable(ts->chip_data, false);
        }
    }

    mutex_lock(&ts->mutex);
    if (focal_debug_ops && focal_debug_ops->dump_reg_sate) {
        count = focal_debug_ops->dump_reg_sate(ts->chip_data, tmp);
    }
    mutex_unlock(&ts->mutex);

    if (ts->esd_handle_support) {
        if (focal_debug_ops && focal_debug_ops->esd_check_enable) {
            focal_debug_ops->esd_check_enable(ts->chip_data, true);
        }
    }
    memcpy(buf, tmp, count);

    return count;
}

static DEVICE_ATTR(fts_fw_version, S_IRUGO|S_IWUSR, focal_fw_version_show, NULL);
/* read and write register
*   read example: echo 88 > rw_reg ---read register 0x88
*   write example:echo 8807 > rw_reg ---write 0x07 into register 0x88
*   note:the number of input must be 2 or 4.if it not enough,please fill in the 0.*/
static DEVICE_ATTR(fts_rw_reg, S_IRUGO|S_IWUSR, focal_rw_reg_show, focal_rw_reg_store);
static DEVICE_ATTR(fts_dump_reg, S_IRUGO|S_IWUSR, focal_dump_reg_show, NULL);
static DEVICE_ATTR(fts_hw_reset, S_IRUGO|S_IWUSR, focal_hw_reset_show, NULL);
static DEVICE_ATTR(fts_irq, S_IRUGO|S_IWUSR, NULL, focal_irq_store);
static DEVICE_ATTR(fts_esd_check, S_IRUGO|S_IWUSR, focal_esdcheck_show, focal_esdcheck_store);

static struct attribute *focal_attributes[] =
{
    &dev_attr_fts_fw_version.attr,
    &dev_attr_fts_rw_reg.attr,
    &dev_attr_fts_dump_reg.attr,
    &dev_attr_fts_hw_reset.attr,
    &dev_attr_fts_irq.attr,
    &dev_attr_fts_esd_check.attr,
    NULL
};

static struct attribute_group focal_attribute_group =
{
    .attrs = focal_attributes
};

int focal_create_sysfs(struct i2c_client * client)
{
    int err = -1;
    err = sysfs_create_group(&client->dev.kobj, &focal_attribute_group);
    if (0 != err) {
        TPD_INFO("[EX]: sysfs_create_group() failed!\n");
        sysfs_remove_group(&client->dev.kobj, &focal_attribute_group);
        return -EIO;
    } else {
        TPD_INFO("[EX]: sysfs_create_group() succeeded!\n");
    }
    return err;
}
/******************************* End of device attribute file******************************************/

/********************Start of apk debug file and it's operation callbacks******************************/
unsigned char proc_operate_mode = 0;

static ssize_t focal_debug_write(struct file *filp, const char __user *buff, size_t count, loff_t *ppos)
{
    unsigned char writebuf[WRITE_BUF_SIZE];
    int buflen = count;
    int writelen = 0;
    int ret = 0;
    char tmp[25];
    struct touchpanel_data *ts = PDE_DATA(file_inode(filp));
    struct focal_debug_func *focal_debug_ops = (struct focal_debug_func *)ts->private_data;

    if (copy_from_user(&writebuf, buff, buflen)) {
        TPD_INFO("[APK]: copy from user error!\n");
        return -EFAULT;
    }

    mutex_lock(&ts->mutex);

    proc_operate_mode = writebuf[0];
    switch (proc_operate_mode)
    {
        case PROC_SET_TEST_FLAG:
            TPD_INFO("[APK]: PROC_SET_TEST_FLAG = %x!\n", writebuf[1]);
            if (ts->esd_handle_support && !ts->is_suspended) {
                if (writebuf[1] == 1) {
                    esd_handle_switch(&ts->esd_info, false);
                } else {
                    esd_handle_switch(&ts->esd_info, true);
                }
            }
            break;
        case PROC_READ_REGISTER:
            writelen = 1;
            ret = touch_i2c_write(ts->client, writebuf + 1, writelen);
            if (ret < 0) {
                TPD_INFO("[APK]: write iic error!\n");
            }
            break;
        case PROC_WRITE_REGISTER:
            writelen = 2;
            ret = touch_i2c_write(ts->client, writebuf + 1, writelen);
            if (ret < 0) {
                TPD_INFO("[APK]: write iic error!\n");
            }
            break;
        case PROC_HW_RESET:
            sprintf(tmp, "%s", writebuf + 1);
            tmp[buflen - 1] = '\0';
            if (strncmp(tmp,"focal_driver",12) == 0) {
                TPD_INFO("Begin HW Reset\n");
                if (focal_debug_ops && focal_debug_ops->reset) {
                    focal_debug_ops->reset(ts->chip_data, 1);
                }
            }
            break;
        case PROC_READ_DATA:
        case PROC_WRITE_DATA:
            writelen = count - 1;
            if (writelen>0)
            {
                ret = touch_i2c_write(ts->client, writebuf + 1, writelen);
                if (ret < 0) {
                    TPD_INFO("[APK]: write iic error!\n");
                }
            }
            break;
        default:
            break;
    }

    mutex_unlock(&ts->mutex);

    if (ret < 0) {
        return ret;
    } else {
        return count;
    }
}

static ssize_t focal_debug_read(struct file *filp, char __user *buff, size_t count, loff_t *ppos)
{
    int ret = 0;
    int num_read_chars = 0;
    int readlen = 0;
    unsigned char buf[READ_BUF_SIZE];
    struct touchpanel_data *ts = PDE_DATA(file_inode(filp));
    struct focal_debug_func *focal_debug_ops = (struct focal_debug_func *)ts->private_data;

    if (ts->esd_handle_support) {
        if (focal_debug_ops && focal_debug_ops->esd_check_enable) {
            focal_debug_ops->esd_check_enable(ts->chip_data, false);
        }
    }

    mutex_lock(&ts->mutex);

    switch (proc_operate_mode)
    {
        case PROC_READ_REGISTER:
            readlen = 1;
            ret = touch_i2c_read(ts->client, NULL, 0, buf, readlen);
            if (ret < 0) {
                if (ts->esd_handle_support) {
                    if (focal_debug_ops && focal_debug_ops->esd_check_enable) {
                        focal_debug_ops->esd_check_enable(ts->chip_data, true);
                    }
                }
                TPD_INFO("[APK]: read i2c error!\n");
                mutex_unlock(&ts->mutex);
                return ret;
            }
            num_read_chars = 1;
            break;
        case PROC_READ_DATA:
            readlen = count;
            ret = touch_i2c_read(ts->client, NULL, 0, buf, readlen);
            if (ret < 0) {
                if (ts->esd_handle_support) {
                    if (focal_debug_ops && focal_debug_ops->esd_check_enable) {
                        focal_debug_ops->esd_check_enable(ts->chip_data, true);
                    }
                }
                TPD_INFO("[APK]: read iic error!\n");
                mutex_unlock(&ts->mutex);
                return ret;
            }

            num_read_chars = readlen;
            break;
        case PROC_WRITE_DATA:
            break;
        default:
            break;
    }

    mutex_unlock(&ts->mutex);

    if (ts->esd_handle_support) {
        if (focal_debug_ops && focal_debug_ops->esd_check_enable) {
            focal_debug_ops->esd_check_enable(ts->chip_data, true);
        }
    }

    if (copy_to_user(buff, buf, num_read_chars))
    {
        TPD_INFO("[APK]: copy to user error!\n");
        return -EFAULT;
    }

    return num_read_chars;
}
static const struct file_operations focal_proc_fops =
{
    .owner  = THIS_MODULE,
    .open  = simple_open,
    .read   = focal_debug_read,
    .write  = focal_debug_write,
};

int focal_create_apk_debug_channel(struct touchpanel_data *ts)
{
    struct proc_dir_entry *focal_proc_entry = NULL;
    focal_proc_entry = proc_create_data("ftxxxx-debug", 0777, NULL, &focal_proc_fops, ts);
    if (NULL == focal_proc_entry) {
        TPD_INFO("%s: Couldn't create proc entry!\n", __func__);
        return -ENOMEM;
    }

    return 0;
}
/**********************End of apk debug file and it's operation callbacks******************************/