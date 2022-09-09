// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2022 Oplus. All rights reserved.
 */

#include <linux/configfs.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/nls.h>
#include <linux/kdev_t.h>

#include "oplus_charger.h"
#include "oplus_gauge.h"
#include "oplus_vooc.h"
#include "oplus_short.h"
#include "oplus_adapter.h"
#include "oplus_wireless.h"
#include "charger_ic/oplus_short_ic.h"
#include "oplus_debug_info.h"

static struct class *oplus_chg_class;
static struct device *oplus_wireless_dir;

/**********************************************************************
* wireless device nodes
**********************************************************************/
static ssize_t max_w_power_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct oplus_chg_chip *chip = NULL;

	chip = (struct oplus_chg_chip *)dev_get_drvdata(oplus_wireless_dir);
	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", oplus_wpc_get_max_wireless_power());
}
static DEVICE_ATTR_RO(max_w_power);

static struct device_attribute *oplus_wireless_attributes[] = {
	&dev_attr_max_w_power,
	NULL
};

static void oplus_wireless_dir_destroy(void)
{
	struct device_attribute **attrs;
	struct device_attribute *attr;

	attrs = oplus_wireless_attributes;
	while ((attr = *attrs++))
		device_remove_file(oplus_wireless_dir, attr);
	device_destroy(oplus_wireless_dir->class, oplus_wireless_dir->devt);
	unregister_chrdev_region(oplus_wireless_dir->devt, 1);
}

int oplus_wireless_node_add(struct device_attribute **wireless_attributes)
{
	struct device_attribute **attrs;
	struct device_attribute *attr;

	attrs = wireless_attributes;
	while ((attr = *attrs++)) {
		int err;

		err = device_create_file(oplus_wireless_dir, attr);
		if (err) {
			chg_err("device_create_file fail!\n");
			device_destroy(oplus_wireless_dir->class, oplus_wireless_dir->devt);
			return err;
		}
	}

	return 0;
}


static int oplus_wireless_dir_create(struct oplus_chg_chip *chip)
{
	int status = 0;
	dev_t devt;
	struct device_attribute **attrs;
	struct device_attribute *attr;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	status = alloc_chrdev_region(&devt, 0, 1, "wireless");
	if (status < 0) {
		chg_err("alloc_chrdev_region wireless fail!\n");
		return -ENOMEM;
	}

	oplus_wireless_dir = device_create(oplus_chg_class, NULL,
			devt, NULL, "%s", "wireless");
	oplus_wireless_dir->devt = devt;
	dev_set_drvdata(oplus_wireless_dir, chip);

	attrs = oplus_wireless_attributes;
	while ((attr = *attrs++)) {
		int err;

		err = device_create_file(oplus_wireless_dir, attr);
		if (err) {
			chg_err("device_create_file fail!\n");
			device_destroy(oplus_wireless_dir->class, oplus_wireless_dir->devt);
			return err;
		}
	}

	return 0;
}

void oplus_wireless_node_delete(struct device_attribute **wireless_attributes)
{
	struct device_attribute **attrs;
	struct device_attribute *attr;

	attrs = wireless_attributes;
	while ((attr = *attrs++))
		device_remove_file(oplus_wireless_dir, attr);
}

int oplus_chg_configfs_init(struct oplus_chg_chip *chip)
{
	int status = 0;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -EINVAL;
	}

	oplus_chg_class = class_create(THIS_MODULE, "oplus_chg");
	if (IS_ERR(oplus_chg_class)) {
		chg_err("oplus_chg_configfs_init fail!\n");
		return PTR_ERR(oplus_chg_class);
	}

	status = oplus_wireless_dir_create(chip);
	if (status < 0)
		chg_err("oplus_wireless_dir_create fail!\n");

	return 0;
}
EXPORT_SYMBOL(oplus_chg_configfs_init);

int oplus_chg_configfs_exit(void)
{
	oplus_wireless_dir_destroy();

	if (!IS_ERR(oplus_chg_class))
		class_destroy(oplus_chg_class);
	return 0;
}
EXPORT_SYMBOL(oplus_chg_configfs_exit);

