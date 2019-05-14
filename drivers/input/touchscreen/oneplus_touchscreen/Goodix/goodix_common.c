/***************************************************
 * File:goodix_common.c
 * Copyright (c)  2008- 2030  Oppo Mobile communication Corp.ltd.
 * Description:
 *             goodix common driver
 * Version:1.0:
 * Date created:2016/09/02
 * Author: Tong.han@Bsp.Driver
 * TAG: BSP.TP.Init
 * *
 * -------------- Revision History: -----------------
 *  <author >  <data>  <version>  <desc>
 ***************************************************/

#include "goodix_common.h"

#define TPD_DEVICE "goodix_common"
#define TPD_INFO(a, arg...)  pr_err("[TP]"TPD_DEVICE ": " a, ##arg)    
#define TPD_DEBUG(a, arg...)\
    do{\
        if (tp_debug)\
        pr_err("[TP]"TPD_DEVICE ": " a, ##arg);\
    }while(0) 

void GetCirclePoints(struct Coordinate *input_points, int number, struct Coordinate *pPnts)
{
    int i = 0;
    int k = 0, j = 0, m = 0, n = 0;
    int max_y, min_y, max_x, min_x;

    max_y = input_points[0].y;
    min_y = input_points[0].y;
    max_x = input_points[0].x;
    min_x = input_points[0].x;

    for (i = 0; i < number; i++) {
        if (input_points[i].y > max_y) {
            max_y = input_points[i].y;
            k = i;
        }
    }
    pPnts[2] = input_points[k];

    for (i = 0; i < number; i++) {
        if (input_points[i].y < min_y) {
            min_y = input_points[i].y;
            j = i;
        }
    }
    pPnts[0] = input_points[j];

    for (i = 0; i < number; i++) {
        if (input_points[i].x > max_x) {
            max_x = input_points[i].x;
            m = i;
        }
    }
    pPnts[3] = input_points[m];

    for (i = 0; i < number; i++) {
        if (input_points[i].x < min_x) {
            min_x = input_points[i].x;
            n = i;
        }
    }
    pPnts[1] = input_points[n];
}

/**
 * ClockWise -  calculate clockwise for circle gesture
 * @p: coordinate array head point.
 * @n: how many points need to be calculated
 * Return 1--clockwise, 0--anticlockwise, not circle, report 2
 */
int ClockWise(struct Coordinate *p, int n)
{
    int i, j, k;
    int count = 0;
    long int z;

    if (n < 3)
        return 1;
    for (i = 0; i < n; i++) {
        j = (i + 1) % n;
        k = (i + 2) % n;
        if ((p[i].x == p[j].x) && (p[j].x == p[j].y))
            continue;
        z = (p[j].x - p[i].x) * (p[k].y - p[j].y);
        z -= (p[j].y - p[i].y) * (p[k].x - p[j].x);
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

static ssize_t tp_devices_check_read_func(struct file *file, char __user * page, size_t size, loff_t * ppos)
{
    char pagesize[64] = {0};
    int ret = 0;
    struct touchpanel_data *ts = (struct touchpanel_data *)PDE_DATA(file_inode(file));
    if(!ts)
        return 0;

    ret = sprintf(pagesize, "%d\n", ts->panel_data.tp_type);
    ret = simple_read_from_buffer(page, size, ppos, pagesize, strlen(pagesize));
    return ret;
}

static const struct file_operations gt1x_devices_check = {
    .owner = THIS_MODULE,
    .read  = tp_devices_check_read_func,
};

//proc/touchpanel/Goodix/config_version
static int gt1x_tp_config_read_func(struct seq_file *s, void *v)
{
    struct touchpanel_data *ts = s->private;
    struct goodix_proc_operations *goodix_ops;

    if(!ts)
        return 0;
    goodix_ops = (struct goodix_proc_operations *)(ts->private_data);
    if (!goodix_ops) {
        return 0;
    }
    if (!goodix_ops->goodix_config_info_read) {
        seq_printf(s, "Not support auto-test proc node\n");
        return 0;
    }
    disable_irq_nosync(ts->client->irq);
    mutex_lock(&ts->mutex);

    goodix_ops->goodix_config_info_read(s, ts->chip_data);

    mutex_unlock(&ts->mutex);
    enable_irq(ts->client->irq);
    return 0;
}

static int proc_data_config_version_read(struct seq_file *s, void *v)

{
    gt1x_tp_config_read_func(s, v);
    return 0;
}

static int gt1x_data_config_version_open(struct inode *inode, struct file *file)
{
    return single_open(file, proc_data_config_version_read, PDE_DATA(inode));
}

static const struct file_operations gt1x_tp_config_version_proc_fops = {
    .owner = THIS_MODULE,
    .open = gt1x_data_config_version_open,
    .read = seq_read,
    .release = single_release,
};

static ssize_t goodix_water_protect_read(struct file *file, char *buff, size_t len, loff_t *pos)
{
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));
    struct goodix_proc_operations *syna_ops;
    ssize_t ret = 0;

    if (!ts)
        return 0;

    syna_ops = (struct goodix_proc_operations *)ts->private_data;
    if (!syna_ops)
        return 0;

    if (!syna_ops->goodix_water_protect_read) {
        if(copy_to_user(buff, "Not support auto-test proc node\n", strlen("Not support auto-test proc node\n")))
                TPD_INFO("%s,here:%d\n",__func__,__LINE__);
        return 0;
    }

    ret = syna_ops->goodix_water_protect_read(file, buff, len, pos);
    return ret;
}

static ssize_t goodix_water_protect_write(struct file *file, const char *buff, size_t len, loff_t *pos)
{
    struct touchpanel_data *ts = PDE_DATA(file_inode(file));
    struct goodix_proc_operations *syna_ops;
    ssize_t ret = 0;

    if (!ts)
        return 0;

    syna_ops = (struct goodix_proc_operations *)ts->private_data;
    if (!syna_ops)
        return 0;

    if (!syna_ops->goodix_water_protect_write) {
            TPD_INFO("Not support dd proc node %s %d\n",__func__,__LINE__);
        return 0;
    }

    ret = syna_ops->goodix_water_protect_write(file, buff, len, pos);
    return ret;
}

static struct file_operations goodix_water_protect_debug_ops =
{
	.owner = THIS_MODULE,
	.read =  goodix_water_protect_read,
	.write = goodix_water_protect_write,
};

int Goodix_create_proc(struct touchpanel_data *ts, struct goodix_proc_operations *goodix_ops)
{
    int ret = 0;
    struct proc_dir_entry *prEntry_tmp = NULL;
    struct proc_dir_entry *prEntry_gt = NULL;
    ts->private_data = goodix_ops;

    prEntry_gt = proc_mkdir("Goodix", ts->prEntry_tp);
    if (prEntry_gt == NULL) {
        ret = -ENOMEM;
        TPD_INFO("%s: Couldn't create GT TP proc entry\n", __func__);
    }

    prEntry_tmp = proc_create_data("TP_AUTO_TEST_ID", 0777, ts->prEntry_tp, &gt1x_devices_check, ts);
    if (prEntry_tmp == NULL) {
        ret = -ENOMEM;
        TPD_INFO("init_synaptics_proc: Couldn't create proc entry\n");
    }

    //show config and firmware id interface
    prEntry_tmp = proc_create_data("config_version", 0666, prEntry_gt, &gt1x_tp_config_version_proc_fops, ts);
    if (prEntry_tmp == NULL) {
        ret = -ENOMEM;
        TPD_INFO("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
    }

    prEntry_tmp = proc_create_data("water_protect", 0666, prEntry_gt, &goodix_water_protect_debug_ops, ts);
    if (prEntry_tmp == NULL) {
        ret = -ENOMEM;
        TPD_INFO("%s: Couldn't create water_protect proc entry, %d\n", __func__, __LINE__);
    }

    return ret;
}


