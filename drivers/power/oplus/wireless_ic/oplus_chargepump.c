/************************************************************************************
** File:  oplus_chargepump.c
** OPLUS_FEATURE_CHG_BASIC
** Copyright (C), 2008-2012, OPLUS Mobile Comm Corp., Ltd
** 
** Description: 
**      
** 
** Version: 1.0
** Date created: 21:03:46,09/04/2019
** Author: Lin Shangbo
** 
** --------------------------- Revision History: ------------------------------------------------------------
* <version>       <date>        <author>              			<desc>
* Revision 1.0    2019-04-09    Lin Shangbo    		Created for new charger
************************************************************************************************************/

#include <linux/interrupt.h>
#include <linux/i2c.h>

#ifdef CONFIG_OPLUS_CHARGER_MTK


#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <linux/xlog.h>

#include <linux/module.h>
#include <upmu_common.h>
#include <mt-plat/mtk_gpio.h>
#include <mtk_boot_common.h>
#include <mt-plat/mtk_rtc.h>
#include <mt-plat/charging.h>

#include <soc/oplus/device_info.h>

extern void mt_power_off(void); 
#else

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
#endif
#include "../oplus_vooc.h"
#include "../oplus_gauge.h"
#include "../oplus_charger.h"
#include "oplus_chargepump.h"
#include "../wireless_ic/oplus_p922x.h"

struct chip_chargepump *chargepump_ic = NULL;

int chargepump_reg = 0;

static DEFINE_MUTEX(chargepump_i2c_access);

static int __chargepump_read_reg(int reg, int *returnData)
{
	int ret = 0;
	struct chip_chargepump *chip = chargepump_ic;

	if(chip == NULL) {
		chg_err("chargepump_ic is NULL!\n");
		return -1;
	}
	
	ret = i2c_smbus_read_byte_data(chip->client, (unsigned char)reg);
	if (ret < 0) {
		chg_err("i2c read fail: can't read from %02x: %d\n", reg, ret);
		return ret;
	} else {
		*returnData = ret;
	}

	return 0;
}

static int chargepump_read_reg(int reg, int *returnData)
{
	int ret = 0;

	mutex_lock(&chargepump_i2c_access);
	ret = __chargepump_read_reg(reg, returnData);
	mutex_unlock(&chargepump_i2c_access);
	return ret;
}

static int __chargepump_write_reg(int reg, int val)
{
	int ret = 0;
	struct chip_chargepump *chip = chargepump_ic;

	if(chip == NULL) {
		chg_err("chargepump_ic is NULL!\n");
		return -1;
	}

	ret = i2c_smbus_write_byte_data(chip->client, reg, val);
	if (ret < 0) {
		chg_err("i2c write fail: can't write %02x to %02x: %d\n",
			val, reg, ret);
		return ret;
	}

	return 0;
}

static int chargepump_config_interface(int RegNum, int val, int MASK)
{
	int chargepump_reg = 0;
	int ret = 0;

	mutex_lock(&chargepump_i2c_access);

	
	ret = __chargepump_read_reg(RegNum, &chargepump_reg);

	//chg_err(" Reg[%x]=0x%x\n", RegNum, chargepump_reg);

	chargepump_reg &= ~MASK;
	chargepump_reg |= val;

	ret = __chargepump_write_reg(RegNum, chargepump_reg);

	//chg_err(" write Reg[%x]=0x%x\n", RegNum, chargepump_reg);

	__chargepump_read_reg(RegNum, &chargepump_reg);

	chg_err(" Check Reg[%x]=0x%x\n", RegNum, chargepump_reg);

	mutex_unlock(&chargepump_i2c_access);

	return ret;
}

int chargepump_set_for_otg(char enable)
{
#ifndef OP20A
	int ret;
	
	if (chargepump_ic == NULL) {	
		chg_err(" chargepump_ic is NULL!\n");
		return -1;
	}

	chg_err("<~WPC~> chargepump_set_for_otg: %d\n", enable);

	if(enable) {
		ret = chargepump_config_interface(0x00, 0xFF, 0xFF);
		if (ret) {
			chg_err(" write reg 0x00 error!\n");
			return ret;
		}

		ret = chargepump_config_interface(0x01, 0x40, 0xFF);
		if (ret) {
			chg_err(" write reg 0x01 error!\n");
			return ret;
		}

		ret = chargepump_config_interface(0x02, 0xF9, 0xFF);
		if (ret) {
			chg_err(" write reg 0x02 error!\n");
			return ret;
		}

		ret = chargepump_config_interface(0x03, 0x53, 0xFF);
		if (ret) {
			chg_err(" write reg 0x03 error!\n");
			return ret;
		}

		ret = chargepump_config_interface(0x04, 0x01, 0xFF);
		if (ret) {
			chg_err(" write reg 0x04 error!\n");
			return ret;
		}

		ret = chargepump_config_interface(0x06, 0x10, 0xFF);
		if (ret) {
			chg_err(" write reg 0x06 error!\n");
			return ret;
		}

		ret = chargepump_config_interface(0x07, 0x78, 0xFF);
		if (ret) {
			chg_err(" write reg 0x07 error!\n");
			return ret;
		}

		ret = chargepump_config_interface(0xA7, 0xF9, 0xFF);
		if (ret) {
			chg_err(" write reg 0xA7 error!\n");
			return ret;
		}

		ret = chargepump_config_interface(0x05, 0x61, 0xFF);
		if (ret) {
			chg_err(" write reg 0x05 error!\n");
			return ret;
		}
	} else {
		ret = chargepump_config_interface(0x0A, 0x30, 0xFF);
		if (ret) {
			chg_err(" write reg 0x0A error!\n");
			return ret;
		}
	}
#endif
	
	return 0;
}

int chargepump_set_for_EPP(void)
{
	int ret;
	
	if (chargepump_ic == NULL) {	
		chg_err(" chargepump_ic is NULL!\n");
		return -1;
	}

	chg_err("<~WPC~> chargepump_set_for_EPP!\n");

#ifdef OP20A
	ret = chargepump_config_interface(0x08, 0xFF, 0xFF);
	if (ret) {
		chg_err(" write reg 0x08 error!\n");
		return ret;
	}
	
	msleep(20);
	ret = chargepump_config_interface(0x01, 0x02, 0xFF);
	if (ret) {
		chg_err(" write reg 0x01 error!\n");
		return ret;
	}

	msleep(20);
	ret = chargepump_config_interface(0x02, 0x00, 0xFF);
	if (ret) {
		chg_err(" write reg 0x02 error!\n");
		return ret;
	}

	msleep(20);
	ret = chargepump_config_interface(0x03, 0x00, 0xFF);
	if (ret) {
		chg_err(" write reg 0x03 error!\n");
		return ret;
	}

	msleep(20);
	ret = chargepump_config_interface(0x00, 0xCA, 0xFF);
	if (ret) {
		chg_err(" write reg 0x00 error!\n");
		return ret;
	}
#else
	ret = chargepump_config_interface(0x00, 0xFF, 0xFF);
	if (ret) {
		chg_err(" write reg 0x00 error!\n");
		return ret;
	}

	ret = chargepump_config_interface(0x01, 0xC0, 0xFF);
	if (ret) {
		chg_err(" write reg 0x01 error!\n");
		return ret;
	}

	ret = chargepump_config_interface(0x02, 0xF9, 0xFF);
	if (ret) {
		chg_err(" write reg 0x02 error!\n");
		return ret;
	}

	ret = chargepump_config_interface(0x03, 0x53, 0xFF);
	if (ret) {
		chg_err(" write reg 0x03 error!\n");
		return ret;
	}

	ret = chargepump_config_interface(0x04, 0x01, 0xFF);
	if (ret) {
		chg_err(" write reg 0x04 error!\n");
		return ret;
	}

	ret = chargepump_config_interface(0x06, 0x10, 0xFF);
	if (ret) {
		chg_err(" write reg 0x06 error!\n");
		return ret;
	}

	ret = chargepump_config_interface(0x07, 0x78, 0xFF);
	if (ret) {
		chg_err(" write reg 0x07 error!\n");
		return ret;
	}

	ret = chargepump_config_interface(0xA7, 0xF9, 0xFF);
	if (ret) {
		chg_err(" write reg 0xA7 error!\n");
		return ret;
	}

	ret = chargepump_config_interface(0x0A, 0x30, 0xFF);
	if (ret) {
		chg_err(" write reg 0x0A error!\n");
		return ret;
	}

#endif

	return 0;
}

int chargepump_set_for_LDO(void)
{
#ifndef OP20A
	int ret;

	if (chargepump_ic == NULL) {	
		chg_err(" chargepump_ic is NULL!\n");
		return -1;
	}

	chg_err("<~WPC~> chargepump_set_for_LDO!\n");

	ret = chargepump_config_interface(0x05, 0x61, 0xFF);
	if (ret) {
		chg_err(" write reg 0x05 error!\n");
		return ret;
	}

	ret = chargepump_config_interface(0xA7, 0xF9, 0xFF);
	if (ret) {
		chg_err(" write reg 0xA7 error!\n");
		return ret;
	}

	ret = chargepump_config_interface(0x0A, 0x90, 0xFF);
	if (ret) {
		chg_err(" write reg 0x0A error!\n");
		return ret;
	}
#endif

	return 0;
}

int chargepump_enable_voltage_diff_detect(void)
{
#ifndef OP20A
	int ret;

	if (chargepump_ic == NULL) {	
		chg_err(" chargepump_ic is NULL!\n");
		return -1;
	}

	chg_err("<~WPC~> chargepump_enable_voltage_diff_detect!\n");
	
	ret = chargepump_config_interface(0x03, 0x50, 0xFF);
	if (ret) {
		chg_err(" write reg 0x03 error!\n");
		return ret;
	}
#endif

	return 0;
}

int chargepump_enable_watchdog(void)
{
#ifndef OP20A
	int ret;
	
	chg_err("<~WPC~> chargepump_enable_watchdog!\n");
	
	ret = chargepump_config_interface(0x07, 0x98, 0xFF);
	if (ret) {
		chg_err(" write reg 0x07 error!\n");
		return ret;
	}
#endif
	
	return 0;
}

int chargepump_kick_watchdog(void)
{
#ifndef OP20A
	int ret;
		
	chg_err("<~WPC~> chargepump_kick_watchdog!\n");
		
	ret = chargepump_config_interface(0x07, 0x98, 0xFF);
	if (ret) {
		chg_err(" write reg 0x07 error!\n");
		return ret;
	}
#else
	int ret;
		
	//chg_err("<~WPC~>  chargepump_kick_watchdog!\n");
		
	ret = chargepump_config_interface(0x0A, 0x71, 0xFF);
	if (ret) {
		chg_err(" write reg 0x0A error!\n");
		return ret;
	}
#endif
		
	return 0;
}

int chargepump_check_fastchg_status(void)
{
#ifdef OP20A
	int reg_value = 0;

	if (!chargepump_ic) {
		chg_err("<~WPC~> chargepump_ic is NULL!\n");
		return -1;
	}

	chargepump_read_reg(0x04, &reg_value);
	chg_err(" <~WPC~> chargepump 0x04: 0x%02X\n", reg_value);
	if ((reg_value & 0x01) == 0) {
		chg_err(" <~WPC~> chargepump isn't working properly!\n");
		return -1;
	} else {
		return 0;
	}
#else
	return 0;
#endif
}

int chargepump_check_dwp_status(void)
{
#ifdef OP20A
	int reg_value = 0;

	if (!chargepump_ic) {
		chg_err("<~WPC~> chargepump_ic is NULL!\n");
		return -1;
	}

	chargepump_read_reg(0x04, &reg_value);
	chg_err(" <~WPC~> chargepump 0x04: 0x%02X\n", reg_value);
	if ((reg_value & 0x02) != 0) {
		chg_err(" <~WPC~> chargepump dwp isn't OK!\n");
		return -1;
	} else if ((reg_value & 0xF8) != 0) {
		chg_err(" <~WPC~> chargepump has other errors!\n");
		return -2;
	} else {
		chg_err(" <~WPC~> chargepump dwp is OK!\n");
		return 0;
	}
#else
	return 0;
#endif
}

void chargepump_set_chargepump_en_val(struct chip_chargepump *chip, int value)
{    
	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: chargepump_ic not ready!\n", __func__);
		return;
	}

	if (chip->chargepump_en_gpio <= 0) {
		chg_err("chargepump_en_gpio not exist, return\n");
		return;
	}

	if (IS_ERR_OR_NULL(chip->pinctrl)
		|| IS_ERR_OR_NULL(chip->chargepump_en_active)
		|| IS_ERR_OR_NULL(chip->chargepump_en_sleep)
		|| IS_ERR_OR_NULL(chip->chargepump_en_default)) {
		chg_err("pinctrl null, return\n");
		return;
	}

	if (value) {
		gpio_direction_output(chip->chargepump_en_gpio, 1);
		pinctrl_select_state(chip->pinctrl,
				chip->chargepump_en_default);
	} else {
		gpio_direction_output(chip->chargepump_en_gpio, 0);
		pinctrl_select_state(chip->pinctrl,
				chip->chargepump_en_default);
	}

	chg_err("<~WPC~> set value:%d, gpio_val:%d\n", 
		value, gpio_get_value(chip->chargepump_en_gpio));
}

int chargepump_get_chargepump_en_val(void)
{
	struct chip_chargepump *chip = chargepump_ic;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: chargepump_ic not ready!\n", __func__);
		return -1;
	}

	if (chip->chargepump_en_gpio <= 0) {
		chg_err("chargepump_en_gpio not exist, return\n");
		return -1;
	}

	if (IS_ERR_OR_NULL(chip->pinctrl)
		|| IS_ERR_OR_NULL(chip->chargepump_en_active)
		|| IS_ERR_OR_NULL(chip->chargepump_en_sleep)
		|| IS_ERR_OR_NULL(chip->chargepump_en_default)) {
		chg_err("pinctrl null, return\n");
		return -1;
	}

	return gpio_get_value(chip->chargepump_en_gpio);
}

int chargepump_dwp_enable(void)
{
#ifdef OP20A
	int ret;
		
	chg_err("<~WPC~> chargepump_dwp_enable!\n");
		
	ret = chargepump_config_interface(0x00, 0xCA, 0xFF);
	if (ret) {
		chg_err(" write reg 0x00 error!\n");
		return ret;
	}
#endif

	return 0;
}

int chargepump_dwp_disable(void)
{
#ifdef OP20A
	int ret;
			
	chg_err("<~WPC~> chargepump_dwp_disable!\n");
			
	ret = chargepump_config_interface(0x00, 0xC2, 0xFF);
	if (ret) {
		chg_err(" write reg 0x00 error!\n");
		return ret;
	}
#endif
	
	return 0;
}

int chargepump_enable(void)
{
	int ret = 0;

	chg_err("<~WPC~> chargepump_enable!\n");
	
	if (chargepump_ic != NULL) {	
		chargepump_set_chargepump_en_val(chargepump_ic, 1);
		chargepump_ic->is_chargepump_enable = true;
		return ret;
	} else {
		chg_err(" chargepump_ic is NULL!\n");
		return -1;
	}
}

int chargepump_disable(void)
{
	int ret;

	chg_err("<~WPC~> chargepump_disable!\n");
	
	if (chargepump_ic != NULL) {
		chargepump_set_for_otg(0);
		chargepump_set_chargepump_en_val(chargepump_ic, 0);
		chargepump_ic->is_chargepump_enable = false;
		return ret;
	} else {
		chg_err(" chargepump_ic is NULL!\n");
		return -1;
	}
}

void chargepump_print_log(void)
{
	int reg_value = 0;

	if (!chargepump_ic) {
		chg_err("<~WPC~> chargepump_ic is NULL!\n");
		return;
	}

	if (chargepump_ic->is_chargepump_enable == false)
		return;

#ifdef OP20A
	chargepump_read_reg(0x04, &reg_value);
	chg_err(" <~WPC~> chargepump 0x04: 0x%02X\n", reg_value);	
#else
	chargepump_read_reg(0x08, &reg_value);
	chg_err(" <~WPC~> chargepump 0x08: 0x%02X\n", reg_value);	

	chargepump_read_reg(0x09, &reg_value);
	chg_err(" <~WPC~> chargepump 0x09: 0x%02X\n", reg_value);	
#endif

}

int chargepump_hardware_init(void)
{
	return true;
}

static int chargepump_en_gpio_init(struct chip_chargepump *chip)
{
	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: chip_chargepump not ready!\n", __func__);
		return -EINVAL;
	}

	chip->pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->pinctrl)) {
		chg_err("get chargepump_en pinctrl fail\n");
		return -EINVAL;
	}

	chip->chargepump_en_active = pinctrl_lookup_state(chip->pinctrl, "cp_en_active");
	if (IS_ERR_OR_NULL(chip->chargepump_en_active)) {
		chg_err("get chargepump_en_active fail\n");
		return -EINVAL;
	}

	chip->chargepump_en_sleep = pinctrl_lookup_state(chip->pinctrl, "cp_en_sleep");
	if (IS_ERR_OR_NULL(chip->chargepump_en_sleep)) {
		chg_err("get chargepump_en_sleep fail\n");
		return -EINVAL;
	}

	chip->chargepump_en_default = pinctrl_lookup_state(chip->pinctrl, "cp_en_default");
	if (IS_ERR_OR_NULL(chip->chargepump_en_default)) {
		chg_err("get chargepump_en_default fail\n");
		return -EINVAL;
	}

	gpio_direction_output(chip->chargepump_en_gpio, 0);	
	pinctrl_select_state(chip->pinctrl, chip->chargepump_en_default);

	chg_err("<~WPC~> chargepump_en_gpio: %d \n", gpio_get_value(chip->chargepump_en_gpio));

	return 0;
}

static int chargepump_gpio_init(struct chip_chargepump *chip)
{
	int rc=0;
	struct device_node *node = chip->dev->of_node;

	// Parsing gpio chargepump_en_gpio
	chip->chargepump_en_gpio = of_get_named_gpio(node, "qcom,cp_en-gpio", 0);
	if(chip->chargepump_en_gpio < 0 ){
		pr_err("chip->chargepump_en_gpio not specified\n");	
	}
	else
	{
		if( gpio_is_valid(chip->chargepump_en_gpio) ){
			rc = gpio_request(chip->chargepump_en_gpio, "qcom,cp_en-gpio");
			if(rc){
				pr_err("unable to request gpio [%d]\n", chip->chargepump_en_gpio);
			}
		}
		rc = chargepump_en_gpio_init(chip);
		pr_err("chip->chargepump_en_gpio =%d\n",chip->chargepump_en_gpio);
	}
	

	chg_err(" chargepump_gpio_init FINISH\n");

	return rc;
}

#if 0
struct oplus_wpc_operations      *cp_ops = {
	.cp_hw_init = chargepump_hardware_init;
	.cp_set_for_otg = chargepump_set_for_otg;
	.cp_set_for_EPP = chargepump_set_for_EPP;
	.cp_set_for_LDO = chargepump_set_for_LDO;
	.cp_enable = chargepump_enable;
};
#endif

static int chargepump_driver_probe(struct i2c_client *client, const struct i2c_device_id *id) 
{
	int ret = 0;
	struct chip_chargepump *chg_ic;

	chg_ic = devm_kzalloc(&client->dev,
		sizeof(struct chip_chargepump), GFP_KERNEL);
	if (!chg_ic) {
		chg_err(" kzalloc() failed\n");
		return -ENOMEM;
	}

	chg_debug( " call \n");
	chg_ic->client = client;
	chg_ic->dev = &client->dev;

	chargepump_ic = chg_ic;
	atomic_set(&chg_ic->chargepump_suspended, 0);

	chargepump_hardware_init();
	chargepump_gpio_init(chg_ic);

	chg_debug(" success\n");

	return ret;
}


static struct i2c_driver chargepump_i2c_driver;

static int chargepump_driver_remove(struct i2c_client *client)
{
	int ret=0;

	//ret = i2c_del_driver(&chargepump_i2c_driver);
	chg_debug( "  ret = %d\n", ret);
	return 0;
}

static unsigned long suspend_tm_sec = 0;
static int get_current_time(unsigned long *now_tm_sec)
{
	struct rtc_time tm;
	struct rtc_device *rtc = NULL;
	int rc = 0;

	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (rtc == NULL) {
		chg_err("%s: unable to open rtc device (%s)\n",
			__FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
		return -EINVAL;
	}

	rc = rtc_read_time(rtc, &tm);
	if (rc) {
		chg_err("Error reading rtc device (%s) : %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	rc = rtc_valid_tm(&tm);
	if (rc) {
		chg_err("Invalid RTC time (%s): %d\n",
		CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	rtc_tm_to_time(&tm, now_tm_sec);

close_time:
	rtc_class_close(rtc);

	return rc;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
static int chargepump_pm_resume(struct device *dev)
{
	unsigned long resume_tm_sec = 0;
	unsigned long sleep_time = 0;
	int rc = 0;
	struct chip_chargepump *chip = chargepump_ic;

	if(!chip) {
		return 0;
	}
	
	atomic_set(&chip->chargepump_suspended, 0);
	rc = get_current_time(&resume_tm_sec);
	if (rc || suspend_tm_sec == -1) {
		chg_err("RTC read failed\n");
		sleep_time = 0;
	} else {
		sleep_time = resume_tm_sec - suspend_tm_sec;
	}
	
	/*
	if(sleep_time < 0) {
		sleep_time = 0;
	}
	*/

	return 0;
}

static int chargepump_pm_suspend(struct device *dev)
{
	struct chip_chargepump *chip = chargepump_ic;

	if(!chip) {
		return 0;
	}
	
	atomic_set(&chip->chargepump_suspended, 1);
	if (get_current_time(&suspend_tm_sec)) {
		chg_err("RTC read failed\n");
		suspend_tm_sec = -1;
	}

	return 0;
}

static const struct dev_pm_ops chargepump_pm_ops = {
	.resume		= chargepump_pm_resume,
	.suspend	= chargepump_pm_suspend,
};
#else
static int chargepump_resume(struct i2c_client *client)
{
	unsigned long resume_tm_sec = 0;
	unsigned long sleep_time = 0;
	int rc = 0;
	struct chip_chargepump *chip = chargepump_ic;

	if(!chip) {
		return 0;
	}
	
	atomic_set(&chip->chargepump_suspended, 0);
	rc = get_current_time(&resume_tm_sec);
	if (rc || suspend_tm_sec == -1) {
		chg_err("RTC read failed\n");
		sleep_time = 0;
	} else {
		sleep_time = resume_tm_sec - suspend_tm_sec;
	}

	/*
	if(sleep_time < 0) {
		sleep_time = 0;
	}
	*/	

	return 0;
}

static int chargepump_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct chip_chargepump *chip = chargepump_ic;

	if(!chip) {
		return 0;
	}
	
	atomic_set(&chip->chargepump_suspended, 1);
	if (get_current_time(&suspend_tm_sec)) {
		chg_err("RTC read failed\n");
		suspend_tm_sec = -1;
	}
	
	return 0;
}
#endif

static void chargepump_reset(struct i2c_client *client)
{
}


/**********************************************************
  *
  *   [platform_driver API] 
  *
  *********************************************************/

static const struct of_device_id chargepump_match[] = {
	{ .compatible = "oplus,chgpump-charger"},
	{ },
};

static const struct i2c_device_id chargepump_id[] = {
	{"chgpump-charger", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, chargepump_id);


static struct i2c_driver chargepump_i2c_driver = {
	.driver		= {
		.name = "chgpump-charger",
		.owner	= THIS_MODULE,
		.of_match_table = chargepump_match,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
		.pm 	= &chargepump_pm_ops,
#endif
	},
	.probe		= chargepump_driver_probe,
	.remove		= chargepump_driver_remove,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0))
	.resume		= chargepump_resume,
	.suspend	= chargepump_suspend,
#endif
	.shutdown	= chargepump_reset,
	.id_table	= chargepump_id,
};


module_i2c_driver(chargepump_i2c_driver);
MODULE_DESCRIPTION("Driver for chgpump chip");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:chargepump-charger");
