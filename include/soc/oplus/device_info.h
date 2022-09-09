/**
 * Copyright 2008-2013 OPLUS Mobile Comm Corp., Ltd, All rights reserved.
 * FileName:devinfo.h
 * ModuleName:devinfo
 * Create Date: 2013-10-23
 * Description:add interface to get device information.
*/

#ifndef _DEVICE_INFO_H
#define _DEVICE_INFO_H

#include <linux/list.h>

#define INFO_LEN  24

struct manufacture_info {
	char name[INFO_LEN];
	char *version;
	char *manufacture;
	char *fw_path;
};

struct o_hw_id {
	const char *label;
	const char *match;
	int id;
	struct list_head list;
};

struct o_ufsplus_status {
   int *hpb_status;
   int *tw_status;
};

int register_device_proc(char *name, char *version, char *vendor);
int register_device_proc_for_ufsplus(char *name, int *hpb_status,int *tw_status);
int register_devinfo(char *name, struct manufacture_info *info);
bool check_id_match(const char *label, const char *id_match, int id);

#endif
