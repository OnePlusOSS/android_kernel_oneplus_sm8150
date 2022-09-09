/***************************************************************
** Copyright (C),  2020,  OPLUS Mobile Comm Corp.,  Ltd
** VENDOR_EDIT
** File : oplus_display_panel.h
** Description : oplus display panel char dev  /dev/oplus_panel
** Version : 1.0
** Date : 2020/06/13
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**  Li.Sheng       2020/06/13        1.0           Build this moudle
******************************************************************/
#ifndef _OPLUS_DISPLAY_PANEL_H_
#define _OPLUS_DISPLAY_PANEL_H_

#include <linux/fs.h>
#include <linux/printk.h>
#include <linux/device.h>
#include <asm/ioctl.h>
#include <linux/err.h>
#include <linux/notifier.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>
#include "oplus_display_panel_power.h"
#include "oplus_display_panel_seed.h"
#include "oplus_display_panel_common.h"
#include "oplus_ffl.h"
#include "oplus_aod.h"
#include "oplus_dc_diming.h"
#include "oplus_onscreenfingerprint.h"

#define OPLUS_PANEL_NAME "oplus_display"
#define OPLUS_PANEL_CLASS_NAME "oplus_display_class"

#define OPLUS_PANEL_IOCTL_BASE			'o'

#define PANEL_IO(nr)			_IO(OPLUS_PANEL_IOCTL_BASE, nr)
#define PANEL_IOR(nr, type)		_IOR(OPLUS_PANEL_IOCTL_BASE, nr, type)
#define PANEL_IOW(nr, type)		_IOW(OPLUS_PANEL_IOCTL_BASE, nr, type)
#define PANEL_IOWR(nr, type)		_IOWR(OPLUS_PANEL_IOCTL_BASE, nr, type)

#define PANEL_IOCTL_NR(n)       _IOC_NR(n)
#define PANEL_IOCTL_SIZE(n)		_IOC_SIZE(n)

typedef int oplus_panel_feature(void *data);

static dev_t dev_num = 0;
static struct class *panel_class;
static struct device *panel_dev;
static int panel_ref = 0;
static struct cdev panel_cdev;

struct panel_ioctl_desc {
	unsigned int cmd;
	oplus_panel_feature *func;
	const char *name;
};

#define RAMLESS_AOD_PAYLOAD_SIZE	100
#define RAMLESS_AOD_AREA_NUM		6
struct panel_aod_area {
	int x;
	int y;
	int w;
	int h;
	int color;
	int bitdepth;
	int mono;
	int gray;
};

struct panel_aod_area_para {
	struct panel_aod_area aod_area[RAMLESS_AOD_AREA_NUM];
	uint32_t size;
};
struct hecate_info {
	uint32_t display_id;
	uint32_t kgsl_dump; /* 1 present kgsl fence timeout */
};

/*oplus ioctl case start*/
#define PANEL_COMMOND_BASE 0x00
#define PANEL_COMMOND_MAX  0x55

#define PANEL_IOCTL_SET_POWER				  PANEL_IOW(0x01, struct panel_vol_set)
#define PANEL_IOCTL_GET_POWER				  PANEL_IOWR(0x02, struct panel_vol_get)
#define PANEL_IOCTL_SET_SEED				  PANEL_IOW(0x03, unsigned int)
#define PANEL_IOCTL_GET_SEED				  PANEL_IOWR(0x04, unsigned int)
#define PANEL_IOCTL_GET_PANELID			      PANEL_IOWR(0x05, struct panel_id)
#define PANEL_IOCTL_SET_FFL				      PANEL_IOW(0x06, unsigned int)
#define PANEL_IOCTL_GET_FFL				      PANEL_IOWR(0x07, unsigned int)
#define PANEL_IOCTL_SET_AOD				      PANEL_IOW(0x08, unsigned int)
#define PANEL_IOCTL_GET_AOD				      PANEL_IOWR(0x09, unsigned int)
#define PANEL_IOCTL_SET_MAX_BRIGHTNESS		  PANEL_IOW(0x0A, unsigned int)
#define PANEL_IOCTL_GET_MAX_BRIGHTNESS		  PANEL_IOWR(0x0B, unsigned int)
#define PANEL_IOCTL_GET_PANELINFO			  PANEL_IOWR(0x0C, struct panel_info)
#define PANEL_IOCTL_GET_CCD				      PANEL_IOWR(0x0D, unsigned int)
#define PANEL_IOCTL_GET_SERIAL_NUMBER		  PANEL_IOWR(0x0E, struct panel_serial_number)
#define PANEL_IOCTL_SET_HBM				      PANEL_IOW(0x0F, unsigned int)
#define PANEL_IOCTL_GET_HBM				      PANEL_IOWR(0x10, unsigned int)
#define PANEL_IOCTL_SET_DIM_ALPHA			  PANEL_IOW(0x11, unsigned int)
#define PANEL_IOCTL_GET_DIM_ALPHA			  PANEL_IOWR(0x12, unsigned int)
#define PANEL_IOCTL_SET_DIM_DC_ALPHA		  PANEL_IOW(0x13, unsigned int)
#define PANEL_IOCTL_GET_DIM_DC_ALPHA		  PANEL_IOWR(0x14, unsigned int)
#define PANEL_IOCTL_SET_POWER_STATUS		  PANEL_IOW(0x18, unsigned int)
#define PANEL_IOCTL_GET_POWER_STATUS		  PANEL_IOWR(0x19, unsigned int)
#define PANEL_IOCTL_SET_CLOSEBL_FLAG		  PANEL_IOW(0x1B, unsigned int)
#define PANEL_IOCTL_GET_CLOSEBL_FLAG		  PANEL_IOWR(0x1C, unsigned int)
#define PANEL_IOCTL_SET_DIMLAYER_HBM		  PANEL_IOW(0x1F, unsigned int)
#define PANEL_IOCTL_GET_DIMLAYER_HBM		  PANEL_IOWR(0x20, unsigned int)
#define PANEL_IOCTL_SET_DIMLAYER_BL_EN        PANEL_IOW(0X21, unsigned int)
#define PANEL_IOCTL_GET_OPLUS_BRIGHTNESS      PANEL_IOWR(0x2B, unsigned int)
#define PANEL_IOCTL_SET_AOD_AREA              PANEL_IOW(0x2E, struct panel_aod_area_para)
#define PANEL_IOCTL_SET_HECATE_INFO           PANEL_IOW(0x54, struct hecate_info)

/*oplus ioctl case end*/

#endif /*_OPLUS_DISPLAY_PANEL_H_*/
