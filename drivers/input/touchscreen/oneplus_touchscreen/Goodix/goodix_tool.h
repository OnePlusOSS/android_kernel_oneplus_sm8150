/***************************************************
 * File:goodix_tool.h
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

#ifndef _TOUCHPANEL_TOOL_GOODIX_H_
#define _TOUCHPANEL_TOOL_GOODIX_H_

#include "../touchpanel_common.h"
#include "goodix_common.h"

#pragma pack(1)
struct st_cmd_head{
    u8  wr;              //write read flag£¬0:R  1:W  2:PID 3:
    u8  flag;            //0:no need flag/int 1: need flag  2:need int
    u8  flag_addr[2];    //flag address
    u8  flag_val;        //flag val
    u8  flag_relation;   //flag_val:flag 0:not equal 1:equal 2:> 3:<
    u16 circle;          //polling cycle
    u8  times;           //plling times
    u8  retry;           //I2C retry times
    u16 delay;           //delay befor read or after write
    u16 data_len;        //data length
    u8  addr_len;        //address length
    u8  addr[2];         //address
    u8  res[3];          //reserved
    u8  *data;           //data pointer
} ;
#pragma pack()

struct Goodix_tool_info{
    u8     devicecount;
    bool   esd_handle_support;            /*esd handle support feature*/
    int    *is_suspended;
    void   *chip_data;
    struct hw_resource *hw_res;
    struct i2c_client  *client;
    struct st_cmd_head cmd_head;
    struct fw_update_info *update_info;
    struct esd_information  *esd_info;

    int  (*reset) (void *chip_data); /*Reset Touchpanel*/
};

int gt1x_init_tool_node(struct touchpanel_data *ts, struct fw_update_info *update_info);

#define GTP_ESD_PROTECT 0
#define GTP_DRIVER_VERSION          "V1.4<2015/07/10>"

#define DATA_LENGTH_UINT    512
#define GTP_ADDR_LENGTH     2
#define DATA_LENGTH         (DATA_LENGTH_UINT - GTP_ADDR_LENGTH)
#define CMD_HEAD_LENGTH     (sizeof(struct st_cmd_head) - sizeof(u8*))

#endif
