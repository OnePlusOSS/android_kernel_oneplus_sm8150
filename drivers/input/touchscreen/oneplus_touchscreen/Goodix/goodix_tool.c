/***************************************************
 * File:goodix_tool.c
 * Copyright (c)  2008- 2030  Oppo Mobile communication Corp.ltd.
 * Description:
 *             goodix debugging tool code
 * Version:1.0:
 * Date created:2016/09/02
 * Author: Tong.han@Bsp.Driver
 * TAG: BSP.TP.Init
 * *
 * -------------- Revision History: -----------------
 *  <author >  <data>  <version>  <desc>
 ***************************************************/

#include "goodix_tool.h"

int gt1x_rawdiff_mode ;

/****************************PART1:Log TAG****************************/

#define TPD_DEVICE "Goodix-TOOL"
#define TPD_INFO(fmt, arg...)        pr_err(TPD_DEVICE ": " fmt, ##arg)
#define TPD_DEBUG(fmt, arg...)       do{\
    if (tp_debug)\
    pr_err(TPD_DEVICE ": " fmt, ##arg);\
}while(0)

#define TPD_DEBUG_ARRAY(array, num)    do{\
    s32 i;\
    u8* a = array;\
    if (tp_debug)\
    {\
        pr_err("<< GTP-TOOL-DBG >>");\
        for (i = 0; i < (num); i++)\
        {\
            pr_err("%02x ", (a)[i]);\
            if ((i + 1) % 10 == 0)\
            {\
                pr_err("\n<< GTP-DBG >>");\
            }\
        }\
        pr_err("\n");\
    }\
}while(0)

/****************************PART2:Code * && function****************************/

static ssize_t gt1x_tool_read     (struct file *filp, char __user * buffer, size_t count, loff_t * ppos);
static ssize_t gt1x_tool_write    (struct file *filp, const char *buffer, size_t count, loff_t * ppos);
static int     gt1x_tool_release(struct inode *inode, struct file *filp);
static int     gt1x_tool_open    (struct inode *inode, struct file *file);

static struct file_operations gt1x_tool_fops = {
    .read = gt1x_tool_read,
    .write = gt1x_tool_write,
    .open = gt1x_tool_open,
    .release = gt1x_tool_release,
    .owner = THIS_MODULE,
};

int gt1x_init_tool_node(struct touchpanel_data *ts, struct fw_update_info *update_info)
{
    struct proc_dir_entry *gt1x_tool_proc_entry;
    struct Goodix_tool_info *gt_tool_info;
    gt_tool_info = kzalloc(sizeof(struct Goodix_tool_info), GFP_KERNEL);

    gt_tool_info->is_suspended = &ts->is_suspended;
    gt_tool_info->esd_handle_support = ts->esd_handle_support;
    gt_tool_info->esd_info = &ts->esd_info;
    gt_tool_info->client = ts->client;
    gt_tool_info->chip_data = ts->chip_data;
    gt_tool_info->hw_res = &(ts->hw_res);
    gt_tool_info->reset = ts->ts_ops->reset;
    gt_tool_info->cmd_head.wr = 1;    //if the first operation is read, will return fail.

    gt_tool_info->cmd_head.data = kzalloc(DATA_LENGTH_UINT, GFP_KERNEL);
    gt_tool_info->update_info = update_info;

    if (NULL == gt_tool_info->cmd_head.data) {
        TPD_INFO("Apply for memory failed.");
        return -1;
    }
    TPD_INFO("Alloc memory size:%d.", DATA_LENGTH_UINT);

    gt1x_tool_proc_entry = proc_create_data("goodix_tool", 0666, NULL, &gt1x_tool_fops, gt_tool_info);
    if (gt1x_tool_proc_entry == NULL) {
        TPD_INFO("CAN't create proc entry /proc/goodix_tool.");
        return -1;
    } else {
        TPD_INFO("Created proc entry /proc/goodix_tool.");
    }

    return 0;
}

#if 0
void gt1x_deinit_tool_node(void)
{
    remove_proc_entry("goodix_tool", NULL);
    kfree(cmd_head.data);
    cmd_head.data = NULL;
}
#endif

static s32 tool_i2c_read(struct i2c_client *client, u8 * buf, u16 len)
{
    int ret = 0;
    u16 addr = (buf[0] << 8) + buf[1];

    ret = touch_i2c_read_block(client, addr, len, &buf[2]);
    if (ret < 0) {
        return -1;
    }

    return 1;
}

static s32 tool_i2c_write(struct i2c_client *client, u8 * buf, u16 len)
{
    int ret = 0;
    u16 addr = (buf[0] << 8) + buf[1];

    ret = touch_i2c_write_block(client, addr, len-2, &buf[2]);
    if (ret < 0) {
        return -1;
    }

    return 1;
}

static u8 relation(u8 src, u8 dst, u8 rlt)
{
    u8 ret = 0;

    switch (rlt) {
        case 0:
            ret = (src != dst) ? true : false;
            break;

        case 1:
            ret = (src == dst) ? true : false;
            TPD_DEBUG("equal:src:0x%02x   dst:0x%02x   ret:%d.", src, dst, (s32) ret);
            break;

        case 2:
            ret = (src > dst) ? true : false;
            break;

        case 3:
            ret = (src < dst) ? true : false;
            break;

        case 4:
            ret = (src & dst) ? true : false;
            break;

        case 5:
            ret = (!(src | dst)) ? true : false;
            break;

        default:
            ret = false;
            break;
    }

    return ret;
}

/*******************************************************
Function:
Comfirm function.
Input:
None.
Output:
Return write length.
 ********************************************************/
static u8 comfirm(struct Goodix_tool_info *gt_tool_info)
{
    s32 i = 0;
    u8 buf[32];

    memcpy(buf, gt_tool_info->cmd_head.flag_addr, gt_tool_info->cmd_head.addr_len);

    for (i = 0; i < gt_tool_info->cmd_head.times; i++) {
        if (tool_i2c_read(gt_tool_info->client, buf, 1) <= 0) {
            TPD_INFO("Read flag data failed!");
            return -1;
        }

        if (true == relation(buf[GTP_ADDR_LENGTH], gt_tool_info->cmd_head.flag_val, gt_tool_info->cmd_head.flag_relation)) {
            TPD_DEBUG("value at flag addr:0x%02x.", buf[GTP_ADDR_LENGTH]);
            TPD_DEBUG("flag value:0x%02x.", gt_tool_info->cmd_head.flag_val);
            break;
        }

        msleep(gt_tool_info->cmd_head.circle);
    }

    if (i >= gt_tool_info->cmd_head.times) {
        TPD_INFO("Didn't get the flag to continue!");
        return -1;
    }

    return 0;
}

/*******************************************************
Function:
Goodix tool write function.
Input:
standard proc write function param.
Output:
Return write length.
 ********************************************************/
static ssize_t gt1x_tool_write(struct file *filp, const char __user * buff, size_t len, loff_t * data)
{
    struct Goodix_tool_info *gt_tool_info = PDE_DATA(file_inode(filp));
    u64 ret = 0;
    struct st_cmd_head *cmd_head = &gt_tool_info->cmd_head;
    struct i2c_client *client = gt_tool_info->client;

    TPD_DEBUG_ARRAY((u8 *) buff, len);

    if (*(gt_tool_info->is_suspended)) {
        TPD_INFO("IC halt");
        gt1x_rawdiff_mode = 0;

        return -1;
    }

    ret = copy_from_user(cmd_head, buff, CMD_HEAD_LENGTH);
    if (ret) {
        TPD_INFO("copy_from_user failed.");
    }

    TPD_DEBUG("wr  :0x%02x.", cmd_head->wr);

    /*
       TPD_DEBUG("flag:0x%02x.", cmd_head.flag);
       TPD_DEBUG("flag addr:0x%02x%02x.", cmd_head.flag_addr[0], cmd_head.flag_addr[1]);
       TPD_DEBUG("flag val:0x%02x.", cmd_head.flag_val);
       TPD_DEBUG("flag rel:0x%02x.", cmd_head.flag_relation);
       TPD_DEBUG("circle  :%d.", (s32)cmd_head.circle);
       TPD_DEBUG("times   :%d.", (s32)cmd_head.times);
       TPD_DEBUG("retry   :%d.", (s32)cmd_head.retry);
       TPD_DEBUG("delay   :%d.", (s32)cmd_head.delay);
       TPD_DEBUG("data len:%d.", (s32)cmd_head.data_len);
       TPD_DEBUG("addr len:%d.", (s32)cmd_head.addr_len);
       TPD_DEBUG("addr:0x%02x%02x.", cmd_head.addr[0], cmd_head.addr[1]);
       TPD_DEBUG("len:%d.", (s32)len);
       TPD_DEBUG("buf[20]:0x%02x.", buff[CMD_HEAD_LENGTH]);
     */

    if (1 == cmd_head->wr) {
        u16 addr, data_len, pos;

        if (1 == cmd_head->flag) {
            if (comfirm(gt_tool_info)) {
                TPD_INFO("[WRITE]Comfirm fail!");
                return -1;
            }
        } else if (2 == cmd_head->flag) {
            //Need interrupt!
        }

        addr = (cmd_head->addr[0] << 8) + cmd_head->addr[1];
        data_len = cmd_head->data_len;
        pos = 0;
        while (data_len > 0) {
            len = data_len > DATA_LENGTH ? DATA_LENGTH : data_len;
            ret = copy_from_user(&cmd_head->data[GTP_ADDR_LENGTH], &buff[CMD_HEAD_LENGTH + pos], len);
            if (ret) {
                TPD_INFO("[WRITE]copy_from_user failed.");
                return -1;
            }
            cmd_head->data[0] = ((addr >> 8) & 0xFF);
            cmd_head->data[1] = (addr & 0xFF);

            TPD_DEBUG_ARRAY(cmd_head->data, len + GTP_ADDR_LENGTH);

            if (tool_i2c_write(client, cmd_head->data, len + GTP_ADDR_LENGTH) <= 0) {
                TPD_INFO("[WRITE]Write data failed!");
                return -1;
            }
            addr += len;
            pos += len;
            data_len -= len;
        }

        if (cmd_head->delay) {
            msleep(cmd_head->delay);
        }

        return cmd_head->data_len + CMD_HEAD_LENGTH;
    } else if (3 == cmd_head->wr) {    //gt1x unused

        //memcpy(IC_TYPE, cmd_head->data, cmd_head->data_len);
        return cmd_head->data_len + CMD_HEAD_LENGTH;
    } else if (5 == cmd_head->wr) {    //?

        //memcpy(IC_TYPE, cmd_head.data, cmd_head.data_len);
        return cmd_head->data_len + CMD_HEAD_LENGTH;
    } else if (7 == cmd_head->wr) {    //disable irq!
        disable_irq_nosync(client->irq);

        if(gt_tool_info->esd_handle_support)
            esd_handle_switch(gt_tool_info->esd_info, false);  //disable esd check

        return CMD_HEAD_LENGTH;
    } else if (9 == cmd_head->wr) {    //enable irq!
        enable_irq(client->irq);
        if(gt_tool_info->esd_handle_support)
            esd_handle_switch(gt_tool_info->esd_info, true);   //enable esd check
        return CMD_HEAD_LENGTH;
    } else if (17 == cmd_head->wr) {
        ret = copy_from_user(&cmd_head->data[GTP_ADDR_LENGTH], &buff[CMD_HEAD_LENGTH], cmd_head->data_len);
        if (ret) {
            TPD_INFO("copy_from_user failed.");
            return -1;
        }

        if (cmd_head->data[GTP_ADDR_LENGTH]) {
            TPD_DEBUG("gtp enter rawdiff.");
            gt1x_rawdiff_mode = true;
        } else {
            gt1x_rawdiff_mode = false;
            TPD_DEBUG("gtp leave rawdiff.");
        }

        return CMD_HEAD_LENGTH;
    } else if (11 == cmd_head->wr) {
        if(gt_tool_info->esd_handle_support)        //add test tool
            esd_handle_switch(gt_tool_info->esd_info, false);  //disable esd check
  //      disable_irq_nosync(client->irq);
    } else if (13 == cmd_head->wr) {
        gt_tool_info->reset(gt_tool_info->chip_data);
        if(gt_tool_info->esd_handle_support)
            esd_handle_switch(gt_tool_info->esd_info, true); 
        gt_tool_info->update_info->status = UPDATE_STATUS_IDLE;
 //       enable_irq(client->irq);
    } else if (15 == cmd_head->wr) {
        
      /*struct task_struct *thrd = NULL;
        memset(cmd_head->data, 0, cmd_head->data_len + 1);
        memcpy(cmd_head->data, &buff[CMD_HEAD_LENGTH], cmd_head->data_len);
        TPD_DEBUG("update firmware, filename: %s", cmd_head->data);
        thrd = kthread_run(gt1x_update_firmware, (void *)cmd_head->data, "GT1x FW Update");
        if (IS_ERR(thrd)) {
        return PTR_ERR(thrd);
        }
      */
    }
    return CMD_HEAD_LENGTH;
}

static int gt1x_tool_open(struct inode *inode, struct file *file)
{
    struct Goodix_tool_info *gt_tool_info = PDE_DATA(inode);

    if (gt_tool_info->devicecount > 0) {
        return -ERESTARTSYS;
        TPD_INFO("tools open failed!");
    }
    gt_tool_info->devicecount++;

    return 0;
}

static int gt1x_tool_release(struct inode *inode, struct file *filp)
{
    struct Goodix_tool_info *gt_tool_info = PDE_DATA(inode);

    gt_tool_info->devicecount--;

    return 0;
}
/*******************************************************
Function:
Goodix tool read function.
Input:
standard proc read function param.
Output:
Return read length.
 ********************************************************/
static ssize_t gt1x_tool_read(struct file *filp, char __user * buffer, size_t count, loff_t * ppos)
{
    struct Goodix_tool_info *gt_tool_info = PDE_DATA(file_inode(filp));
    struct st_cmd_head *cmd_head = &gt_tool_info->cmd_head;

    if (*ppos) {
        TPD_DEBUG("[PARAM]size: %zd, *ppos: %d", count, (int)*ppos);
        *ppos = 0;

        return 0;
    }

    if (*(gt_tool_info->is_suspended)) {
        TPD_INFO("IC halt");
        gt1x_rawdiff_mode = 0;

        return  -1;
    }
    if (cmd_head->wr % 2) {
        TPD_INFO("[READ] invaild operator fail!");
        return -1;
    } else if (!cmd_head->wr) {
        /* general  i2c read  */
        u16 addr, data_len, len, loc;

        if (1 == cmd_head->flag) {
            if (comfirm(gt_tool_info)) {
                TPD_INFO("[READ]Comfirm fail!");
                return -1;
            }
        } else if (2 == cmd_head->flag) {
            //Need interrupt!
        }

        addr = (cmd_head->addr[0] << 8) + cmd_head->addr[1];
        data_len = cmd_head->data_len;
        loc = 0;

        TPD_DEBUG("[READ] ADDR:0x%04X.", addr);
        TPD_DEBUG("[READ] Length: %d", data_len);

        if (cmd_head->delay) {
            msleep(cmd_head->delay);
        }

        while (data_len > 0) {
            len = data_len > DATA_LENGTH ? DATA_LENGTH : data_len;
            cmd_head->data[0] = (addr >> 8) & 0xFF;
            cmd_head->data[1] = (addr & 0xFF);
            if (tool_i2c_read(gt_tool_info->client, cmd_head->data, len) <= 0) {
                TPD_INFO("[READ]Read data failed!");
                return -1;
            }

            memcpy(&buffer[loc], &cmd_head->data[GTP_ADDR_LENGTH], len);
            data_len -= len;
            addr += len;
            loc += len;
            TPD_DEBUG_ARRAY(&cmd_head->data[GTP_ADDR_LENGTH], len);
        }
        *ppos += cmd_head->data_len;
        return cmd_head->data_len;
    } else if (2 == cmd_head->wr) {
        TPD_DEBUG("Return ic type:%s len:%d.", buffer, (s32) cmd_head->data_len);
        return -1;
    } else if (4 == cmd_head->wr) {
        /* read fw update progress */
        buffer[0] = gt_tool_info->update_info->progress >> 8;
        buffer[1] = gt_tool_info->update_info->progress & 0xff;
        buffer[2] = gt_tool_info->update_info->max_progress >> 8;
        buffer[3] = gt_tool_info->update_info->max_progress & 0xff;
        *ppos += 4;

        return 4;
    } else if (6 == cmd_head->wr) {
        //Read error code!
        return -1;
    } else if (8 == cmd_head->wr) {
        /* Read driver version */
        s32 tmp_len;
        tmp_len = strlen(GTP_DRIVER_VERSION);
        memcpy(buffer, GTP_DRIVER_VERSION, tmp_len);
        buffer[tmp_len] = 0;
        *ppos += tmp_len + 1;
        return (tmp_len + 1);
    }
    *ppos += cmd_head->data_len;

    return cmd_head->data_len;
}
