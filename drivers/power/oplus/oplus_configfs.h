// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2022 Oplus. All rights reserved.
 */

#ifndef _OPLUS_CONFIGFS_H_
#define _OPLUS_CONFIGFS_H_

int oplus_chg_configfs_init(struct oplus_chg_chip *chip);
int oplus_chg_configfs_exit(void);
int oplus_ac_node_add(struct device_attribute **ac_attributes);
void oplus_ac_node_delete(struct device_attribute **ac_attributes);
int oplus_usb_node_add(struct device_attribute **usb_attributes);
void oplus_usb_node_delete(struct device_attribute **usb_attributes);
int oplus_battery_node_add(struct device_attribute **battery_attributes);
void oplus_battery_node_delete(struct device_attribute **battery_attributes);
int oplus_wireless_node_add(struct device_attribute **wireless_attributes);
void oplus_wireless_node_delete(struct device_attribute **wireless_attributes);
int oplus_common_node_add(struct device_attribute **common_attributes);
void oplus_common_node_delete(struct device_attribute **common_attributes);

#endif /* _OPLUS_CONFIGFS_H_ */

