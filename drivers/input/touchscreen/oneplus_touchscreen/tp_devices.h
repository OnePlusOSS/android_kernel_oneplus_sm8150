/***************************************************
 * File:tp_devices.h
 * Copyright (c)  2008- 2030  Oppo Mobile communication Corp.ltd.
 * Description:
 *             tp dev
 * Version:1.0:
 * Date created:2016/09/02
 * Author: Tong.han@Bsp.Driver
 * TAG: BSP.TP.Init
 *
 * -------------- Revision History: -----------------
 *  <author >  <data>  <version>  <desc>
 ***************************************************/
#ifndef OPPO_TP_DEVICES_H
#define OPPO_TP_DEVICES_H
//device list define
typedef enum tp_dev{
    TP_OFILM,
    TP_BIEL,
    TP_TRULY,
    TP_BOE,
    TP_G2Y,
    TP_TPK,
    TP_JDI,
    TP_SAMSUNG,
    TP_DSJM,
    TP_BOE_B8,
    TP_INNOLUX,
    TP_HIMAX_DPT,
    TP_AUO,
    TP_DEPUTE,
    TP_UNKNOWN,
}tp_dev;

struct tp_dev_name {
    tp_dev type;
    char name[32];
};

#endif

