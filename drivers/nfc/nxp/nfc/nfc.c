/*
 * The original Work has been changed by NXP Semiconductors.
 * Copyright 2013-2019 NXP
 *
 * Copyright (c) 2015-2017, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
/*
 * Copyright (C) 2010 Trusted Logic S.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/spinlock.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/uaccess.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/i2c.h>
#include <linux/oneplus/boot_mode.h>
#include "nfc.h"
#include "sn1xx.h"
#include "pn8xt.h"

#include <linux/project_info.h>

#define MAX_BUFFER_SIZE         (512)
#define WAKEUP_SRC_TIMEOUT      (5000)
#define MAX_RETRY_COUNT          3
#define MAX_SECURE_SESSIONS      1

#define HWINFO     1

/*Compile time function calls based on the platform selection*/
#define platform_func(prefix, postfix) prefix##postfix
#define func(prefix, postfix) platform_func(prefix, postfix)

#if HWINFO
struct hw_type_info hw_info;
#endif

#if HWINFO
static void check_hw_info(struct nfc_dev *nfc_dev);
#endif

void nfc_disable_irq(struct nfc_dev *nfc_dev)
{
    unsigned long flags;
    spin_lock_irqsave(&nfc_dev->irq_enabled_lock, flags);
    if (nfc_dev->irq_enabled) {
        disable_irq_nosync(nfc_dev->client->irq);
        nfc_dev->irq_enabled = false;
    }
    spin_unlock_irqrestore(&nfc_dev->irq_enabled_lock, flags);
}

void nfc_enable_irq(struct nfc_dev *nfc_dev)
{
    unsigned long flags;
    spin_lock_irqsave(&nfc_dev->irq_enabled_lock, flags);
    if (!nfc_dev->irq_enabled) {
        nfc_dev->irq_enabled = true;
        enable_irq(nfc_dev->client->irq);
    }
    spin_unlock_irqrestore(&nfc_dev->irq_enabled_lock, flags);
}

static irqreturn_t nfc_dev_irq_handler(int irq, void *dev_id)
{
    struct nfc_dev *nfc_dev = dev_id;
    unsigned long flags;
if (device_may_wakeup(&nfc_dev->client->dev))
	pm_wakeup_event(&nfc_dev->client->dev, WAKEUP_SRC_TIMEOUT);

    nfc_disable_irq(nfc_dev);
    spin_lock_irqsave(&nfc_dev->irq_enabled_lock, flags);
    nfc_dev->count_irq++;
    spin_unlock_irqrestore(&nfc_dev->irq_enabled_lock, flags);
    wake_up(&nfc_dev->read_wq);
    return IRQ_HANDLED;
}

static ssize_t nfc_dev_read(struct file *filp, char __user *buf,
        size_t count, loff_t *offset)
{
    struct nfc_dev *nfc_dev = filp->private_data;
    // char tmp[MAX_BUFFER_SIZE];
    unsigned char *tmp = NULL;
    int ret;
    int irq_gpio_val = 0;
    if (!nfc_dev) {
        return -ENODEV;
    }

    if (count > MAX_BUFFER_SIZE)
        count = MAX_BUFFER_SIZE;

    mutex_lock(&nfc_dev->read_mutex);
    irq_gpio_val = gpio_get_value(nfc_dev->irq_gpio);
    if (irq_gpio_val == 0) {
        if (filp->f_flags & O_NONBLOCK) {
            dev_err(&nfc_dev->client->dev,
            ":f_flags has O_NONBLOCK. EAGAIN\n");
            ret = -EAGAIN;
            goto err;
        }
        while (1) {
            ret = 0;
	nfc_enable_irq(nfc_dev);
            if (!gpio_get_value(nfc_dev->irq_gpio)) {
                ret = wait_event_interruptible(nfc_dev->read_wq,
                    !nfc_dev->irq_enabled);
            }
            if (ret)
                goto err;
            nfc_disable_irq(nfc_dev);
            if (gpio_get_value(nfc_dev->irq_gpio))
                break;
            pr_warning("%s: spurious interrupt detected\n", __func__);
        }
    }
    tmp = nfc_dev->kbuf;
    if (!tmp) {
        pr_info("%s: device doesn't exist anymore\n", __func__);
        ret = -ENODEV;
        goto err;
    }
    memset(tmp, 0x00, count);
    /* Read data */
    ret = i2c_master_recv(nfc_dev->client, tmp, count);
    mutex_unlock(&nfc_dev->read_mutex);
    /* delay of 1ms for slow devices*/
    udelay(1000);
    if (ret < 0) {
        pr_err("%s: i2c_master_recv returned %d\n", __func__, ret);
        goto err;
    }
    if (ret > count) {
        pr_err("%s: received too many bytes from i2c (%d)\n",
                __func__, ret);
        ret = -EIO;
        goto err;
    }
    if (copy_to_user(buf, tmp, ret)) {
        pr_warning("%s : failed to copy to user space\n", __func__);
        ret = -EFAULT;
        goto err;
    }
    return ret;
err:
    mutex_unlock(&nfc_dev->read_mutex);
    return ret;
}

static ssize_t nfc_dev_write(struct file *filp, const char __user *buf,
        size_t count, loff_t *offset)
{
    struct nfc_dev *nfc_dev = filp->private_data;
    // char tmp[MAX_BUFFER_SIZE];
    char *tmp = NULL;
    int ret = 0;
    if (!nfc_dev) {
        return -ENODEV;
    }
    if (count > MAX_BUFFER_SIZE) {
        count = MAX_BUFFER_SIZE;
    }

#if 1
    tmp = memdup_user(buf, count);
    if (IS_ERR(tmp)) {
        pr_info("%s: memdup_user failed\n", __func__);
        ret = PTR_ERR(tmp);
        return ret;
    }
#else
    if (copy_from_user(tmp, buf, count)) {
        pr_err("%s : failed to copy from user space\n", __func__);
        return -EFAULT;
    }
#endif
    ret = i2c_master_send(nfc_dev->client, tmp, count);
    if (ret != count) {
        pr_err("%s: i2c_master_send returned %d\n", __func__, ret);
        ret = -EIO;
    }
    /* delay of 1ms for slow devices*/
    udelay(1000);
    kfree(tmp);
    return ret;
}

/* Callback to claim the embedded secure element
 * It is a blocking call, in order to protect the ese
 * from being reset from outside when it is in use.
 */
void nfc_ese_acquire(struct nfc_dev *nfc_dev)
{
    mutex_lock(&nfc_dev->ese_status_mutex);
    pr_debug("%s: ese acquired\n", __func__);
}

/* Callback to release the  embedded secure element
 * it should be released, after completion of any
 * operation (usage or reset) of ese.
 */
void nfc_ese_release(struct nfc_dev *nfc_dev)
{
    mutex_unlock(&nfc_dev->ese_status_mutex);
    pr_debug("%s: ese released\n", __func__);
}

static int nfc_dev_open(struct inode *inode, struct file *filp)
{
    int ret = 0;
    struct nfc_dev *nfc_dev = container_of(filp->private_data,
            struct nfc_dev, nfc_device);

    filp->private_data = nfc_dev;
    pr_info("%s: %d,%d\n", __func__, imajor(inode), iminor(inode));
    return ret;
}

long nfc_dev_ioctl(struct file *filep, unsigned int cmd,
        unsigned long arg)
{
    long ret = 0;
    struct nfc_dev *nfc_dev = filep->private_data;
    ret = func(NFC_PLATFORM, _nfc_ioctl)(nfc_dev, cmd, arg);
    if (ret != 0)
        pr_err("%s: ioctl: cmd = %u, arg = %lu\n", __func__, cmd, arg);
    return ret;
}

static const struct file_operations nfc_dev_fops = {
        .owner  = THIS_MODULE,
        .llseek = no_llseek,
        .read   = nfc_dev_read,
        .write  = nfc_dev_write,
        .open   = nfc_dev_open,
        .unlocked_ioctl  = nfc_dev_ioctl,
};

struct nfc_platform_data {
    unsigned int irq_gpio;
    unsigned int ven_gpio;
    unsigned int firm_gpio;
    unsigned int ese_pwr_gpio;
};

static int nfc_parse_dt(struct device *dev,
    struct nfc_platform_data *data)
{
    int ret = 0;
    struct device_node *np = dev->of_node;

    data->irq_gpio = of_get_named_gpio(np, "nxp,pn544-irq", 0);
    if ((!gpio_is_valid(data->irq_gpio)))
            return -EINVAL;

    data->ven_gpio = of_get_named_gpio(np, "nxp,pn544-ven", 0);
    if ((!gpio_is_valid(data->ven_gpio)))
            return -EINVAL;

    data->firm_gpio = of_get_named_gpio(np, "nxp,pn544-fw-dwnld", 0);
    if ((!gpio_is_valid(data->firm_gpio)))
            return -EINVAL;

    //required for old platform only
    data->ese_pwr_gpio = of_get_named_gpio(np, "nxp,pn544-ese-pwr", 0);
    if ((!gpio_is_valid(data->ese_pwr_gpio)))
        data->ese_pwr_gpio =  -EINVAL;

    pr_info("%s: %d, %d, %d, %d, error:%d\n", __func__,
                data->irq_gpio, data->ven_gpio, data->firm_gpio,
                data->ese_pwr_gpio, ret);
    return ret;
}

static int nfc_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
    int ret = 0;
    int irqn = 0;
    struct nfc_platform_data platform_data;
    struct nfc_dev *nfc_dev;
    pr_debug("%s: enter\n", __func__);

    if (get_second_board_absent() == 1) {
        pr_err("second board absent, don't probe pn5xx\n", __func__);
	ret = -ENODEV;
        goto err;
    }

    ret = nfc_parse_dt(&client->dev, &platform_data);
    if (ret) {
        pr_err("%s : failed to parse\n", __func__);
        goto err;
    }

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        pr_err("%s : need I2C_FUNC_I2C\n", __func__);
        ret = -ENODEV;
        goto err;
    }
    nfc_dev = kzalloc(sizeof(*nfc_dev), GFP_KERNEL);
    if (nfc_dev == NULL) {
        ret = -ENOMEM;
        goto err;
    }
    nfc_dev->client = client;
    if (gpio_is_valid(platform_data.ven_gpio)) {
        ret = gpio_request(platform_data.ven_gpio, "nfc_reset_gpio");
        if (ret) {
            pr_err("%s: unable to request nfc reset gpio [%d]\n",
                        __func__, platform_data.ven_gpio);
            goto err_mem;
        }
        ret = gpio_direction_output(platform_data.ven_gpio, 0);
        if (ret) {
            pr_err("%s: unable to set direction for nfc reset gpio [%d]\n",
                        __func__, platform_data.ven_gpio);
            goto err_en_gpio;
        }
    } else {
        pr_err("%s: nfc reset gpio not provided\n", __func__);
        goto err_mem;
    }
    if (gpio_is_valid(platform_data.irq_gpio)) {
        ret = gpio_request(platform_data.irq_gpio, "nfc_irq_gpio");
        if (ret) {
            pr_err("%s: unable to request nfc irq gpio [%d]\n",
                        __func__, platform_data.irq_gpio);
            goto err_en_gpio;
        }
        ret = gpio_direction_input(platform_data.irq_gpio);
        if (ret) {
            pr_err("%s: unable to set direction for nfc irq gpio [%d]\n",
                        __func__, platform_data.irq_gpio);
            goto err_irq_gpio;
        }
        irqn = gpio_to_irq(platform_data.irq_gpio);
        if (irqn < 0) {
            ret = irqn;
            goto err_irq_gpio;
        }
        client->irq = irqn;
    } else {
        pr_err("%s: irq gpio not provided\n", __func__);
        goto err_en_gpio;
    }
    if (gpio_is_valid(platform_data.firm_gpio)) {
        ret = gpio_request(platform_data.firm_gpio, "nfc_firm_gpio");
        if (ret) {
            pr_err("%s: unable to request nfc firmware gpio [%d]\n",
                        __func__, platform_data.firm_gpio);
            goto err_irq_gpio;
        }
        ret = gpio_direction_output(platform_data.firm_gpio, 0);
        if (ret) {
            pr_err("%s: cannot set direction for nfc firmware gpio [%d]\n",
                    __func__, platform_data.firm_gpio);
            goto err_firm_gpio;
        }
    } else {
        pr_err("%s: firm gpio not provided\n", __func__);
        goto err_irq_gpio;
    }
    if (gpio_is_valid(platform_data.ese_pwr_gpio)) {
        ret = gpio_request(platform_data.ese_pwr_gpio, "nfc-ese_pwr");
        if (ret) {
            pr_err("%s: unable to request nfc ese gpio [%d]\n",
                    __func__, platform_data.ese_pwr_gpio);
            goto err_firm_gpio;
        }
        ret = gpio_direction_output(platform_data.ese_pwr_gpio, 0);
        if (ret) {
            pr_err("%s: cannot set direction for nfc ese gpio [%d]\n",
                    __func__, platform_data.ese_pwr_gpio);
            goto err_ese_pwr_gpio;
        }
    } else {
        /* ese gpio not required for latest platform*/
        pr_info("%s: ese pwr gpio not provided\n", __func__);
    }

    nfc_dev->kbuflen = MAX_BUFFER_SIZE;
    nfc_dev->kbuf = kzalloc(MAX_BUFFER_SIZE, GFP_KERNEL);
    if (!nfc_dev->kbuf) {
        pr_err("failed to allocate memory for pn544_dev->kbuf\n");
        ret = -ENOMEM;
        goto err_ese_pwr_gpio;
    }

    nfc_dev->ven_gpio = platform_data.ven_gpio;
    nfc_dev->irq_gpio = platform_data.irq_gpio;
    nfc_dev->firm_gpio  = platform_data.firm_gpio;
    nfc_dev->ese_pwr_gpio  = platform_data.ese_pwr_gpio;
    /* init mutex and queues */
    init_waitqueue_head(&nfc_dev->read_wq);
    mutex_init(&nfc_dev->read_mutex);
    mutex_init(&nfc_dev->ese_status_mutex);
    spin_lock_init(&nfc_dev->irq_enabled_lock);

    nfc_dev->nfc_device.minor = MISC_DYNAMIC_MINOR;
    nfc_dev->nfc_device.name = "pn553";
    nfc_dev->nfc_device.fops = &nfc_dev_fops;

    ret = misc_register(&nfc_dev->nfc_device);
    if (ret) {
        pr_err("%s: misc_register failed\n", __func__);
        goto err_misc_register;
    }
    /* NFC_INT IRQ */
    nfc_dev->irq_enabled = true;
    ret = request_irq(client->irq, nfc_dev_irq_handler,
            IRQF_TRIGGER_HIGH, client->name, nfc_dev);
    if (ret) {
        pr_err("request_irq failed\n");
        goto err_request_irq_failed;
    }
    device_init_wakeup(&client->dev, true);
    device_set_wakeup_capable(&client->dev, true);
	enable_irq_wake(nfc_dev->client->irq);
    i2c_set_clientdata(client, nfc_dev);
    /*Enable IRQ and VEN*/
    nfc_enable_irq(nfc_dev);
    /*call to platform specific probe*/
    ret = func(NFC_PLATFORM, _nfc_probe)(nfc_dev);
    if (ret != 0) {
        pr_err("%s: probing platform failed\n", __func__);
        goto err_request_irq_failed;
    };
    pr_info("%s: probing NXP NFC exited successfully\n", __func__);
    pr_info("%s: Name of nfcc: %s.\n", __func__, dev_name(&client->dev));

#if HWINFO
    /*
     * This function is used only if
     * hardware info is required during probe*/
    check_hw_info(nfc_dev);
#endif
    return 0;

err_request_irq_failed:
    misc_deregister(&nfc_dev->nfc_device);
err_misc_register:
    mutex_destroy(&nfc_dev->read_mutex);
    mutex_destroy(&nfc_dev->ese_status_mutex);
    kfree(nfc_dev->kbuf);
err_ese_pwr_gpio:
    gpio_free(platform_data.ese_pwr_gpio);
err_firm_gpio:
    gpio_free(platform_data.firm_gpio);
err_irq_gpio:
    gpio_free(platform_data.irq_gpio);
err_en_gpio:
    gpio_free(platform_data.ven_gpio);
err_mem:
    kfree(nfc_dev);
err:
    pr_err("%s: probing NXP NFC driver failed, check hardware\n", __func__);
    return ret;
}

static int nfc_remove(struct i2c_client *client)
{
    int ret = 0;
    struct nfc_dev *nfc_dev;
    pr_info("%s: remove device\n", __func__);
    nfc_dev = i2c_get_clientdata(client);
    if (!nfc_dev) {
        pr_err("%s: device doesn't exist anymore\n", __func__);
        ret = -ENODEV;
        goto err;
    }
    /*call to platform specific remove*/
    ret = func(NFC_PLATFORM, _nfc_remove)(nfc_dev);
    if (ret != 0) {
        pr_err("%s: platform failed\n", __func__);
        goto err;
    }
    free_irq(client->irq, nfc_dev);
    misc_deregister(&nfc_dev->nfc_device);
    mutex_destroy(&nfc_dev->read_mutex);
    mutex_destroy(&nfc_dev->ese_status_mutex);
    gpio_free(nfc_dev->ese_pwr_gpio);
    gpio_free(nfc_dev->firm_gpio);
    gpio_free(nfc_dev->irq_gpio);
    gpio_free(nfc_dev->ven_gpio);
    kfree(nfc_dev->kbuf);
    kfree(nfc_dev);
err:
    return ret;
}

static int nfc_suspend(struct device *device)
{
	struct i2c_client *client = to_i2c_client(device);
	struct nfc_dev *nfc_dev;
	bool ven_enabled = false;
	int err = -1;
	nfc_dev = i2c_get_clientdata(client);
	ven_enabled = func(NFC_PLATFORM, _nfc_ven_enabled)(nfc_dev);
	if (ven_enabled && gpio_get_value(nfc_dev->irq_gpio)) {
		pm_wakeup_event(&nfc_dev->client->dev, WAKEUP_SRC_TIMEOUT);
	return err;
	}
	nfc_enable_irq(nfc_dev);
	return 0;
}

static int nfc_resume(struct device *device)
{
	return 0;
}

static const struct dev_pm_ops nfc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(nfc_suspend, nfc_resume)
};

static const struct i2c_device_id nfc_id[] = {
        { "pn544", 0 },
        { }
};

static struct of_device_id nfc_match_table[] = {
    {.compatible = "nxp,pn544",},
    {}
};
MODULE_DEVICE_TABLE(of, nfc_match_table);

static struct i2c_driver nfc_driver = {
        .id_table   = nfc_id,
        .probe      = nfc_probe,
        .remove     = nfc_remove,
        .driver     = {
                .owner = THIS_MODULE,
                .name  = "pn544",
                .of_match_table = nfc_match_table,
		.pm = &nfc_pm_ops,
	},
};

#if HWINFO
/******************************************************************************
 * Function         check_hw_info
 *
 * Description      This function is called during pn544_probe to retrieve
 *                  HW info.
 *                  Useful get HW information in case of previous FW download is
 *                  interrupted and core reset is not allowed.
 *                  This function checks if core reset  is allowed, if not
 *                  sets DWNLD_REQ(firm_gpio) , ven reset and sends firmware
 *                  get version command.
 *                  In response HW information will be received.
 *
 * Returns          None
 *
 ******************************************************************************/
static void check_hw_info(struct nfc_dev *nfc_dev) {
    //char read_data[20];
    int ret, get_version_len = 8, retry_count = 0;
    // static uint8_t cmd_reset_nci[] = {0x20, 0x00, 0x01, 0x00};
    char get_version_cmd[] =
    {0x00, 0x04, 0xF1, 0x00, 0x00, 0x00, 0x6E, 0xEF};

    char *tmp = kzalloc(MAX_BUFFER_SIZE,GFP_KERNEL);
    if (tmp == NULL) {
        pr_err("%s: failed to allocate memory for transfer data\n", __func__);
        return;
    }

    memcpy(tmp, get_version_cmd, get_version_len);


    pr_info("%s :Enter\n", __func__);

    /*
     * Ven Reset  before sending core Reset
     * This is to check core reset is allowed or not.
     * If not allowed then previous FW download is interrupted in between
     * */

    /*
     * Core reset  failed.
     * set the DWNLD_REQ , do ven reset
     * send firmware download info command
     * */
    // pr_err("%s : write failed\n", __func__);
    pr_info("%s power on with firmware\n", __func__);
    gpio_set_value(nfc_dev->ven_gpio, 1);
    msleep(10);
    if (nfc_dev->firm_gpio) {
        func(NFC_PLATFORM, _update_state_out)(nfc_dev, ST_DN, true);
        gpio_set_value(nfc_dev->firm_gpio, 1);
    }
    msleep(10);
    gpio_set_value(nfc_dev->ven_gpio, 0);
    msleep(10);
    gpio_set_value(nfc_dev->ven_gpio, 1);
    msleep(10);
    ret = i2c_master_send(nfc_dev->client, tmp, get_version_len);
    if (ret != get_version_len) {
        pr_err("%s : write_failed ret=%d\n", __func__,ret);
        kfree(tmp);
        return ;
    } else {
        pr_info("%s :data sent\n", __func__);
    }

    ret = 0;

    while (retry_count < 10) {

        /*
         * Wait for read interrupt
         * If spurious interrupt is received retry again
         * */
        nfc_enable_irq(nfc_dev);
        ret = wait_event_interruptible(
                  nfc_dev->read_wq,
                  !nfc_dev->irq_enabled);

        nfc_disable_irq(nfc_dev);

        if (gpio_get_value(nfc_dev->irq_gpio))
            break;

        pr_warning("%s: spurious interrupt detected\n", __func__);
        retry_count ++;
    }

    if(ret) {
        return;
    }

    memset(tmp, 0x00, MAX_BUFFER_SIZE);
    /*
     * Read response data and copy into hw_type_info
     * */
    ret = i2c_master_recv(nfc_dev->client, tmp, 14);

    if(ret > 0) {
        memcpy(hw_info.data, tmp, ret);
        hw_info.len = ret;
        pr_info("%s :Hardware Version  : %d\n", __func__,hw_info.data[3]);

        switch (hw_info.data[3]) {
        case NFCC_NQ_210:
            push_component_info(NFC, "NQ210", "NXP");
            break;
        case NFCC_NQ_220:
            push_component_info(NFC, "NQ220", "NXP");
            break;
        case NFCC_NQ_310:
            push_component_info(NFC, "NQ310", "NXP");
            break;
        case NFCC_NQ_310_V2:
            push_component_info(NFC, "NQ310", "NXP");
            break;
        case NFCC_NQ_330:
            push_component_info(NFC, "NQ330", "NXP");
            break;
        case NFCC_PN66T:
            push_component_info(NFC, "PN66T", "NXP");
            break;
        default:
            pr_err("%s: spurious interrupt detected\n", __func__);
            break;
        }

    } else {
        pr_err("%s :Read Failed\n", __func__);
    }
    func(NFC_PLATFORM, _update_state_out)(nfc_dev, ST_DN, true);
    gpio_set_value(nfc_dev->firm_gpio, 0);
    msleep(10);
    gpio_set_value(nfc_dev->ven_gpio, 0);
    udelay(1000);
    kfree(tmp);
}
#endif

static int __init nfc_dev_init(void)
{
    pr_info("Loading NXP NFC driver\n");
    return i2c_add_driver(&nfc_driver);
}
module_init(nfc_dev_init);

static void __exit nfc_dev_exit(void)
{
    pr_info("Unloading NXP NFC driver\n");
    i2c_del_driver(&nfc_driver);
}
module_exit(nfc_dev_exit);

MODULE_DESCRIPTION("NXP NFC driver");
MODULE_LICENSE("GPL");
