/************************************************************************************
** File:  oplus_wireless.c
** VENDOR_EDIT
** Copyright (C), 2008-2012, OPLUS Mobile Comm Corp., Ltd
**
** Description:
**      for wpc charge
**
** Version: 1.0
** Date created: 2019-03-16
** Author: huangtongfeng
**
** --------------------------- Revision History: ------------------------------------------------------------
* <version>       <date>        <author>              			<desc>
* Revision 1.0    2019-03-16   huangtongfeng  		Created for wpc charge
* Revision 2.0    2020-09-10   zhaohang  			changed for wpc charge
************************************************************************************************************/

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/proc_fs.h>

#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/bitops.h>
#include <linux/mutex.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/rtc.h>
#include <soc/oplus/device_info.h>

#include "oplus_charger.h"
#include "oplus_vooc.h"
#include "oplus_gauge.h"
#include "oplus_adapter.h"
#include "oplus_wireless.h"

/* Macro's to report to framework for a workaround to avoid showing wattage
 * information in the animation text
 */
#define MAX_W_POWER_SHOW_NO_WATTAGE_INFO	-3
#define MAX_W_POWER_SHOW_WATTAGE_INFO		-1

static struct oplus_wpc_chip *g_wpc_chip = NULL;

void oplus_wpc_set_otg_en_val(int value)
{
	return;
}

int oplus_wpc_get_otg_en_val(struct oplus_chg_chip *chip)
{
	return 0;
}

void oplus_wpc_set_vbat_en_val(int value)
{
	if (!g_wpc_chip) {
		chg_err("g_wpc_chip null, return\n");
		return;
	}
	if (g_wpc_chip->wpc_ops->wpc_set_vbat_en) {
		g_wpc_chip->wpc_ops->wpc_set_vbat_en(value);
		return;
	} else {
		return;
	}
}

void oplus_wpc_set_booster_en_val(int value)
{
	if (!g_wpc_chip) {
		chg_err("g_wpc_chip null, return\n");
		return;
	}
	if (g_wpc_chip->wpc_ops->wpc_set_booster_en) {
		g_wpc_chip->wpc_ops->wpc_set_booster_en(value);
		return;
	} else {
		return;
	}
}


void oplus_wpc_set_ext1_wired_otg_en_val(int value)
{
	if (!g_wpc_chip) {
		chg_err("g_wpc_chip null, return\n");
		return;
	}
	if (g_wpc_chip->wpc_ops->wpc_set_ext1_wired_otg_en) {
		g_wpc_chip->wpc_ops->wpc_set_ext1_wired_otg_en(value);
		return;
	} else {
		return;
	}
}

void oplus_wpc_set_ext2_wireless_otg_en_val(int value)
{
	if (!g_wpc_chip) {
		chg_err("g_wpc_chip null, return\n");
		return;
	}
	if (g_wpc_chip->wpc_ops->wpc_set_ext2_wireless_otg_en) {
		g_wpc_chip->wpc_ops->wpc_set_ext2_wireless_otg_en(value);
		return;
	} else {
		return;
	}
}

void oplus_wpc_set_rtx_function_prepare(void)
{
	if (!g_wpc_chip) {
		chg_err("g_wpc_chip null, return\n");
		return;
	}
	if (g_wpc_chip->wpc_ops->wpc_set_rtx_function_prepare) {
		g_wpc_chip->wpc_ops->wpc_set_rtx_function_prepare();
		return;
	} else {
		return;
	}
}

void oplus_wpc_set_rtx_function(bool enable)
{
	if (!g_wpc_chip) {
		chg_err("g_wpc_chip null, return\n");
		return;
	}
	if (g_wpc_chip->wpc_ops->wpc_set_rtx_function) {
		g_wpc_chip->wpc_ops->wpc_set_rtx_function(enable);
		return;
	} else {
		return;
	}
}




int oplus_wpc_get_idt_en_val(void)
{
	return 0;
}

bool oplus_wpc_get_wired_otg_online(void)
{
	return false;
}

bool oplus_wpc_get_wired_chg_present(void)
{
	return false;
}

void oplus_wpc_dcin_irq_enable(bool enable)
{
	return;
}



bool oplus_wireless_charge_start(void)
{
	if (!g_wpc_chip) {
		/*chg_err("g_wpc_chip null, return\n");*/
		return false;
	}
	if (g_wpc_chip->wpc_ops->wireless_charge_start) {
		return g_wpc_chip->wpc_ops->wireless_charge_start();
	} else {
		return false;
	}
}

bool oplus_wpc_get_normal_charging(void)
{
	if (!g_wpc_chip) {
		chg_err("g_wpc_chip null, return\n");
		return false;
	}
	if (g_wpc_chip->wpc_ops->wpc_get_normal_charging) {
		return g_wpc_chip->wpc_ops->wpc_get_normal_charging();
	} else {
		return false;
	}
}

bool oplus_wpc_get_fast_charging(void)
{
	if (!g_wpc_chip) {
		chg_err("g_wpc_chip null, return\n");
		return false;
	}
	if (g_wpc_chip->wpc_ops->wpc_get_fast_charging) {
		return g_wpc_chip->wpc_ops->wpc_get_fast_charging();
	} else {
		return false;
	}
}

bool oplus_wpc_get_otg_charging(void)
{
	if (!g_wpc_chip) {
		chg_err("g_wpc_chip null, return\n");
		return false;
	}
	if (g_wpc_chip->wpc_ops->wpc_get_otg_charging) {
		return g_wpc_chip->wpc_ops->wpc_get_otg_charging();
	} else {
		return false;
	}
}



bool oplus_wpc_get_ffc_charging(void)
{
	if (!g_wpc_chip) {
		chg_err("g_wpc_chip null, return\n");
		return false;
	}
	if (g_wpc_chip->wpc_ops->wpc_get_ffc_charging) {
		return g_wpc_chip->wpc_ops->wpc_get_ffc_charging();
	} else {
		return false;
	}
}

bool oplus_wpc_check_chip_is_null(void)
{
	if (!g_wpc_chip) {
		chg_err("g_wpc_chip null, return\n");
		return true;
	}
	return false;
}

bool oplus_wpc_get_fw_updating(void)
{
	if (!g_wpc_chip) {
		chg_err("g_wpc_chip null, return\n");
		return -EINVAL;
	}
	if (g_wpc_chip->wpc_ops->wpc_get_fw_updating) {
		return g_wpc_chip->wpc_ops->wpc_get_fw_updating();
	} else {
		return false;
	}
}


int oplus_wpc_get_adapter_type(void)
{
	if (!g_wpc_chip) {
		chg_err("g_wpc_chip null, return\n");
		return -EINVAL;
	}
	if (g_wpc_chip->wpc_ops->wpc_get_adapter_type) {
		return g_wpc_chip->wpc_ops->wpc_get_adapter_type();
	} else {
		return 0;
	}
}


void oplus_wpc_print_log(void)
{
	if (!g_wpc_chip) {
		chg_err("g_wpc_chip null, return\n");
		return;
	}
	if (g_wpc_chip->wpc_ops->wpc_print_log) {
		g_wpc_chip->wpc_ops->wpc_print_log();
		return;
	} else {
		return;
	}
}



void oplus_wpc_set_wrx_en_value(int value)
{
	struct oplus_wpc_chip *chip = g_wpc_chip;
	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_wpc_chip not ready!\n", __func__);
		return;
	}
	if (chip->wpc_gpios.wrx_en_gpio <= 0) {
		chg_err("wrx_en_gpio not exist, return\n");
		return;
	}
	if (IS_ERR_OR_NULL(chip->wpc_gpios.pinctrl)
		|| IS_ERR_OR_NULL(chip->wpc_gpios.wrx_en_active)
		|| IS_ERR_OR_NULL(chip->wpc_gpios.wrx_en_sleep)) {
		chg_err("pinctrl null, return\n");
		return;
	}
	if (value == 1) {
		gpio_direction_output(chip->wpc_gpios.wrx_en_gpio, 1);
		pinctrl_select_state(chip->wpc_gpios.pinctrl,
			chip->wpc_gpios.wrx_en_active);
	} else {
		pinctrl_select_state(chip->wpc_gpios.pinctrl,
			chip->wpc_gpios.wrx_en_sleep);
	}
	chg_err("set value:%d, gpio_val:%d\n",
		value, gpio_get_value(chip->wpc_gpios.wrx_en_gpio));
}


int oplus_wpc_get_wrx_en_val(void)
{
	struct oplus_wpc_chip *chip = g_wpc_chip;
	if (!chip) {
		chg_err("oplus_wpc_chip not ready!\n", __func__);
		return 0;
	}
	if (chip->wpc_gpios.wrx_en_gpio <= 0) {
		chg_err("wrx_en_gpio not exist, return\n");
		return 0;
	}
	if (IS_ERR_OR_NULL(chip->wpc_gpios.pinctrl)
		|| IS_ERR_OR_NULL(chip->wpc_gpios.wrx_en_active)
		|| IS_ERR_OR_NULL(chip->wpc_gpios.wrx_en_sleep)) {
		chg_err("pinctrl null, return\n");
		return 0;
	}
	return gpio_get_value(chip->wpc_gpios.wrx_en_gpio);
}

int oplus_wpc_get_wrx_otg_en_val(void)
{
	struct oplus_wpc_chip *chip = g_wpc_chip;
	if (!chip) {
		chg_err("oplus_wpc_chip not ready!\n", __func__);
		return 0;
	}
	if (chip->wpc_gpios.wrx_otg_en_gpio <= 0) {
		chg_err("wrx_otg_en_gpio not exist, return\n");
		return 0;
	}
	if (IS_ERR_OR_NULL(chip->wpc_gpios.pinctrl)
		|| IS_ERR_OR_NULL(chip->wpc_gpios.wrx_otg_en_active)
		|| IS_ERR_OR_NULL(chip->wpc_gpios.wrx_otg_en_sleep)) {
		chg_err("pinctrl null, return\n");
		return 0;
	}
	return gpio_get_value(chip->wpc_gpios.wrx_otg_en_gpio);
}

void oplus_wpc_set_wrx_otg_en_value(int value)
{
	struct oplus_wpc_chip *chip = g_wpc_chip;
	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_wpc_chip not ready!\n", __func__);
		return;
	}
	if (chip->wpc_gpios.wrx_otg_en_gpio <= 0) {
		chg_err("wrx_otg_en_gpio not exist, return\n");
		return;
	}
	if (IS_ERR_OR_NULL(chip->wpc_gpios.pinctrl)
		|| IS_ERR_OR_NULL(chip->wpc_gpios.wrx_otg_en_active)
		|| IS_ERR_OR_NULL(chip->wpc_gpios.wrx_otg_en_sleep)) {
		chg_err("pinctrl null, return\n");
		return;
	}
	if (value == 1) {
		gpio_direction_output(chip->wpc_gpios.wrx_otg_en_gpio, 1);
		pinctrl_select_state(chip->wpc_gpios.pinctrl,
			chip->wpc_gpios.wrx_otg_en_active);
	} else {
		pinctrl_select_state(chip->wpc_gpios.pinctrl,
			chip->wpc_gpios.wrx_otg_en_sleep);
	}
	chg_err("set value:%d, gpio_val:%d\n",
		value, gpio_get_value(chip->wpc_gpios.wrx_otg_en_gpio));
}

int oplus_wpc_get_max_wireless_power(void)
{
	/* Workaround code logic to report to framework to avoid showing wattage
	 * information in the animation text as agreed with systemUI team due to
	 * the bug in MCU firmware version(0xa2) which reports same adapter ID for
	 * 33W/50W/65W adapters which makes unable to differentiate them
	 */
	if (is_single_batt_svooc_project() == true)
		return MAX_W_POWER_SHOW_NO_WATTAGE_INFO;
	else
		return MAX_W_POWER_SHOW_WATTAGE_INFO;
}

void oplus_wpc_init(struct oplus_wpc_chip *chip)
{
	g_wpc_chip = chip;
}
