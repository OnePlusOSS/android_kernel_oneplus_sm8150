/***************************************************
 * File:st_common.c
 * Copyright (c)  2008- 2030  Oppo Mobile communication Corp.ltd.
 * Description:
 *             st common driver
 * Version:1.0:
 * Date created:2018/10/26
 * Author: Zengpeng.Chen@Bsp.Driver
 * TAG: BSP.TP.Init
 * *
 * -------------- Revision History: -----------------
 *  <author >  <data>  <version>  <desc>
 ***************************************************/

#include "../touchpanel_common.h"
#include "st_common.h"
#include <linux/crc32.h>


/*******Part0:LOG TAG Declear********************/

#define TPD_DEVICE "st_common"
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

/* st get usb state */
#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
extern unsigned int upmu_get_rgs_chrdet(void);
int st_get_usb_state(void)
{
    return upmu_get_rgs_chrdet();
}
#else

int st_get_usb_state(void)
{
    return 0;
}
#endif

/* proc/touchpanel/st_auto_test */
static int tp_auto_test_read_func(struct seq_file *s, void *v)
{
    struct touchpanel_data *ts = s->private;
    struct st_proc_operations *st_ops;
    if (!ts)
        return 0;
    st_ops = (struct st_proc_operations *)ts->private_data;
    if (!st_ops)
        return 0;
    if (!st_ops->auto_test || !st_ops->init_test_iterm) {
        seq_printf(s, "Not support auto-test proc node\n");
        return 0;
    }

    /* if resume not completed, do not do screen on test */
    if (ts->suspend_state != TP_SPEEDUP_RESUME_COMPLETE) {
        seq_printf(s, "Not in resume state\n");
        return 0;
    }

    /* step1:disable_irq && get mutex locked */
    if (ts->int_mode == BANNABLE) {
        disable_irq_nosync(ts->irq);
    }
    mutex_lock(&ts->mutex);

    ts->in_test_process = true;

    /* step2:init st_test_data */
    //st_ops->init_test_iterm(ts->panel_data.test_limit_name);

    /* step3:execute st auto test func */
    st_ops->auto_test(s, ts->chip_data, ts->panel_data.test_limit_name);

    /* step4: return to normal mode */
    ts->ts_ops->reset(ts->chip_data);
    operate_mode_switch(ts);

    /* step5: unlock the mutex && enable irq trigger */
    mutex_unlock(&ts->mutex);
    if (ts->int_mode == BANNABLE) {
        enable_irq(ts->irq);
    }

    ts->in_test_process = false;
    return 0;
}



/* common driver full panel auto test open func */
static int st_autotest_open(struct inode *inode, struct file *file)
{
    return single_open(file, tp_auto_test_read_func, PDE_DATA(inode));
}

/* common driver full panel auto test func file ops */
static const struct file_operations tp_auto_test_proc_fops = {
    .owner = THIS_MODULE,
    .open  = st_autotest_open,
    .read  = seq_read,
    .release = single_release,
};


static int calibrate_fops_read_func(struct seq_file *s, void *v)
{
    struct touchpanel_data *ts = s->private;
    struct st_proc_operations *st_ops = (struct st_proc_operations *)ts->private_data;

    if (!st_ops->calibrate)
        return 0;

    disable_irq_nosync(ts->irq);
    mutex_lock(&ts->mutex);
    if (!ts->touch_count) {
        st_ops->calibrate(s, ts->chip_data);
    } else {
        seq_printf(s, "1 error, release touch on the screen\n");
    }
    mutex_unlock(&ts->mutex);
    enable_irq(ts->irq);

    return 0;
}

static int proc_calibrate_fops_open(struct inode *inode, struct file *file)
{
    return single_open(file, calibrate_fops_read_func, PDE_DATA(inode));
}

static const struct file_operations proc_calibrate_fops = {
    .owner = THIS_MODULE,
    .open  = proc_calibrate_fops_open,
    .read  = seq_read,
    .release = single_release,
};

static int verify_fops_read_func(struct seq_file *s, void *v)
{
    struct touchpanel_data *ts = s->private;
    struct st_proc_operations *st_ops = (struct st_proc_operations *)ts->private_data;

    if (!st_ops->verify_calibration)
        return 0;

    disable_irq_nosync(ts->irq);
    mutex_lock(&ts->mutex);
    if (!ts->touch_count) {
        st_ops->verify_calibration(s, ts->chip_data);
    } else {
        seq_printf(s, "1 error, skip verify when touch on screen\n");
    }
    mutex_unlock(&ts->mutex);
    enable_irq(ts->irq);

    return 0;
}

static int proc_verify_fops_open(struct inode *inode, struct file *file)
{
    return single_open(file, verify_fops_read_func, PDE_DATA(inode));
}

static const struct file_operations proc_verify_fops = {
    .owner = THIS_MODULE,
    .open  = proc_verify_fops_open,
    .read  = seq_read,
    .release = single_release,
};


/* st create proc file node func */
int st_create_proc(struct touchpanel_data *ts, struct st_proc_operations *st_ops)
{
    int ret = 0;

    /* touchpanel_auto_test interface */
    struct proc_dir_entry *prEntry_tmp = NULL;
    ts->private_data = st_ops;
    prEntry_tmp = proc_create_data("baseline_test", 0666, ts->prEntry_tp, &tp_auto_test_proc_fops, ts);
    if (prEntry_tmp == NULL) {
        ret = -ENOMEM;
        TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
    }
	prEntry_tmp = proc_create_data("calibration", 0666, ts->prEntry_tp, &proc_calibrate_fops, ts);
    if (prEntry_tmp == NULL) {
        ret = -ENOMEM;
        TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
    }

    prEntry_tmp = proc_create_data("calibration_verify", 0666, ts->prEntry_tp, &proc_verify_fops, ts);
    if (prEntry_tmp == NULL) {
        ret = -ENOMEM;
        TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
    }

    return ret;
}

/* st remove proc file node func */
void st_proc_remove(struct touchpanel_data *ts)
{
    remove_proc_entry("baseline_test", ts->prEntry_tp);
}

