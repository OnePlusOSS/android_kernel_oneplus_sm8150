/************************************************************************************
** File:  oplus_nu1619.c
** VENDOR_EDIT
** Copyright (C), 2008-2012, OPLUS Mobile Comm Corp., Ltd
** 
** Description: 
**      for wireless charge
** 
** Version: 1.0
** Date created: 21:03:46,06/11/2018
** Author: Lin Shangbo, Li Jiada
** 
** --------------------------- Revision History: ------------------------------------------------------------
* <version>       <date>        <author>              			<desc>
* Revision 1.0    2018-11-06    Lin Shangbo,Li Jiada   		Created for wireless charge
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
#include <linux/random.h>

#include <linux/alarmtimer.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/regmap.h>
#include <linux/iio/consumer.h>
#include <linux/version.h>
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
#include <uapi/linux/qg.h>
#endif
#include <soc/oplus/device_info.h>
#include <soc/oplus/oplus_project.h>

#include "../oplus_vooc.h"
#include "../oplus_gauge.h"
#include "../oplus_charger.h"
#include "../charger_ic/oplus_mp2650.h"
#include "oplus_chargepump.h"
#include <oplus_nu1619.h>
#include <oplus_nu1619_fw.h>
#include <soc/oplus/boot_mode.h>

#define DEBUG_BY_FILE_OPS
#define DEBUG_FASTCHG_BY_ADB

#ifdef FASTCHG_TEST_BY_TIME
#define USE_CHARGEPUMP_SWITCH
static int stop_timer = 0;
#endif

static unsigned long records_seconds = 0;
static bool test_is_charging = false;

struct oplus_nu1619_ic *nu1619_chip = NULL;

extern struct oplus_chg_chip *g_oplus_chip;
extern void oplus_wireless_set_otg_en_val(int value);
extern int oplus_wireless_get_otg_en_val(struct oplus_nu1619_ic *chip);
extern void oplus_set_wrx_en_value(int value);
extern void oplus_set_wrx_otg_value(int value);
extern int oplus_get_idt_en_val(void);
extern int oplus_get_wrx_otg_val(void);
extern int oplus_get_wrx_en_val(void);
extern void oplus_chg_cancel_update_work_sync(void);
extern void oplus_chg_restart_update_work(void);
extern bool oplus_get_wired_otg_online(void);
extern bool oplus_get_wired_chg_present(void);
extern void oplus_dcin_irq_enable(bool enable);
int nu1619_get_idt_con_val(void);
int nu1619_get_idt_int_val(void);
int nu1619_get_vt_sleep_val(void);
static void wlchg_reset_variables(struct oplus_nu1619_ic *chip);

#ifdef VENDOR_EDIT
void __attribute__((weak)) switch_wireless_charger_state(int wireless_state) {return;}
#endif

static DEFINE_MUTEX(nu1619_i2c_access);

/*This array must be consisten with E_FASTCHARGE_LEVEL*/
static int fasctchg_current[FASTCHARGE_LEVEL_NUM] =
{
0,			/*FASTCHARGE_LEVEL_UNKNOW*/
2300,		/*FASTCHARGE_LEVEL_1*/
1250,		/*FASTCHARGE_LEVEL_2*/
1000,		/*FASTCHARGE_LEVEL_3*/
900,		/*FASTCHARGE_LEVEL_4*/
800,		/*FASTCHARGE_LEVEL_5*/
700,		/*FASTCHARGE_LEVEL_6*/
600,		/*FASTCHARGE_LEVEL_7*/
500,		/*FASTCHARGE_LEVEL_8*/
400,		/*FASTCHARGE_LEVEL_9*/
};

static void wpc_battery_update(void)
{
	if (!g_oplus_chip) {
		chg_err("<~WPC~> g_oplus_chip is NULL\n");
		return;
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
	power_supply_changed(g_oplus_chip->batt_psy);
#else
	power_supply_changed(&g_oplus_chip->batt_psy);
#endif
}

static int get_rtc_time(unsigned long *rtc_time)
{
	struct rtc_time tm;
	struct rtc_device *rtc;
	int rc;

	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (rtc == NULL) {
		pr_err("Failed to open rtc device (%s)\n",
				CONFIG_RTC_HCTOSYS_DEVICE);
		return -EINVAL;
	}

	rc = rtc_read_time(rtc, &tm);
	if (rc) {
		pr_err("Failed to read rtc time (%s) : %d\n",
				CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	rc = rtc_valid_tm(&tm);
	if (rc) {
		pr_err("Invalid RTC time (%s): %d\n",
				CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}
	rtc_tm_to_time(&tm, rtc_time);

close_time:
	rtc_class_close(rtc);
	return rc;
}

static int nu1619_test_charging_status(void)
{
	int rc;
	unsigned long now_seconds = 0;

	if (test_is_charging == false) {
		return 0;
	}

	rc = get_rtc_time(&now_seconds);
	if (rc < 0) {
		pr_err("Failed to get RTC time, rc=%d\n", rc);
		return -1;
	}

	chg_err("<~WPC~>[-TEST-]charging time: %ds\n", now_seconds - records_seconds);
	if ((now_seconds - records_seconds) >  270) {
		return 4;
	} else if ((now_seconds - records_seconds) >  180) {
		return 1;
	} else if ((now_seconds - records_seconds) >  20) {
		return 2;
	} else {
		return 0;
	}
}

static void nu1619_test_reset_record_seconds(void)
{
	int rc;

	rc = get_rtc_time(&records_seconds);
	if (rc < 0) {
		pr_err("Failed to get RTC time, rc=%d\n", rc);
	}

	test_is_charging = true;
	/*chg_err("<~WPC~>[-TEST-] record begin seconds: %d\n", records_seconds);*/
}

#define P22X_ADD_COUNT      2
static int __nu1619_read_reg(struct oplus_nu1619_ic *chip, int reg, char *returnData, int count)
{
	/* We have 16-bit i2c addresses - care for endianness */
	char cmd_buf[2]={ reg >> 8, reg & 0xff };
	int ret = 0;
	int i;
	char val_buf[20];

	for (i = 0; i < count; i++) {
		val_buf[i] = 0;
	}

	ret = i2c_master_send(chip->client, cmd_buf, P22X_ADD_COUNT);
	if (ret < P22X_ADD_COUNT) {
		chg_err("%s: i2c read error, reg: %x\n", __func__, reg);
		return ret < 0 ? ret : -EIO;
	}

	ret = i2c_master_recv(chip->client, val_buf, count);
	if (ret < count) {
		chg_err("%s: i2c read error, reg: %x\n", __func__, reg);
		return ret < 0 ? ret : -EIO;
	}

	for (i = 0; i < count; i++) {
		*(returnData + i) = val_buf[i];
	}

	return 0;
}

static int __nu1619_write_reg(struct oplus_nu1619_ic *chip, int reg, int val)
{
	int ret;
	unsigned char data[3] = { reg >> 8, reg & 0xff, val };

	/*chg_err("<~WPC~> tongfeng test reg[0x%x]!\n", reg);*/

	ret = i2c_master_send(chip->client, data, 3);
	if (ret < 3) {
		chg_err("%s: i2c write error, reg: %x\n", __func__, reg);
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}

static int nu1619_write_reg_multi_byte(struct oplus_nu1619_ic *chip, int reg, char *cbuf, int length)
{
	int ret;
	int send_length;
	unsigned char *data_w;

	send_length = length + 2;
	data_w = kzalloc(send_length, GFP_KERNEL);
	if (!data_w) {
		chg_err("can't alloc memory!\n");
		return -1;
	}

	data_w[0] = reg >> 8;
	data_w[1] = reg & 0xff;

	memcpy(data_w + 2, cbuf, length);

	mutex_lock(&nu1619_i2c_access);

	ret = i2c_master_send(chip->client, data_w, send_length);
	if (ret < send_length) {
		chg_err("%s: i2c write error, reg: %x\n", __func__, reg);
		kfree(data_w);
		mutex_unlock(&nu1619_i2c_access);
		return ret < 0 ? ret : -EIO;
	}

	mutex_unlock(&nu1619_i2c_access);

	kfree(data_w);
	return 0;
}

static int nu1619_read_reg(struct oplus_nu1619_ic *chip, int reg, char *returnData, int count)
{
	int ret = 0;

	mutex_lock(&nu1619_i2c_access);
	ret = __nu1619_read_reg(chip, reg, returnData, count);
	mutex_unlock(&nu1619_i2c_access);
	return ret;
}

static int nu1619_config_interface (struct oplus_nu1619_ic *chip, int RegNum, int val, int MASK)
{
	char nu1619_reg = 0;
	int ret = 0;

	mutex_lock(&nu1619_i2c_access);
	ret = __nu1619_read_reg(chip, RegNum, &nu1619_reg, 1);

	nu1619_reg &= ~MASK;
	nu1619_reg |= val;

	ret = __nu1619_write_reg(chip, RegNum, nu1619_reg);

	mutex_unlock(&nu1619_i2c_access);

	return ret;
}

static int nu1619_write_reg(struct oplus_nu1619_ic *chip, int RegNum, int val)
{
	int ret = 0;

	mutex_lock(&nu1619_i2c_access);

	ret = __nu1619_write_reg(chip, RegNum, val);

	mutex_unlock(&nu1619_i2c_access);

	return ret;
}

static int nu1619_write_cmd_D(struct oplus_nu1619_ic *chip, int val)
{
	int ret = 0;
	static int cmd_d = 0;
	cmd_d &= 0x20;
	cmd_d |= val;
	cmd_d ^= 0x20;
	mutex_lock(&nu1619_i2c_access);
	ret = __nu1619_write_reg(chip, 0x000D, cmd_d);
	chg_err("write cmd_D = 0x%x", cmd_d);
	mutex_unlock(&nu1619_i2c_access);
	return ret;
}


static void nu1619_clear_irq(struct oplus_nu1619_ic *chip, char mark0, char mark1)
{
	char write_data[2] = {0, 0};

	return;

	chg_err("<~WPC~>nu1619_clear_irq----------\n");

	write_data[0] = 0x17 | mark0;
	write_data[1] = 0x00 | mark1;
	nu1619_write_reg_multi_byte(chip, 0x0038, write_data, 2);

	write_data[0] = 0x17 | mark0;
	write_data[1] = 0x00 | mark1;
	nu1619_write_reg_multi_byte(chip, 0x0056, write_data, 2);

	write_data[0] = 0x20;
	write_data[1] = 0x00;
	nu1619_write_reg_multi_byte(chip, 0x004E, write_data, 2);
}

static void nu1619_set_FOD_parameter(struct oplus_nu1619_ic *chip, char parameter)
{
	if (parameter == chip->nu1619_chg_status.FOD_parameter) {
		return;
	}

	return;
	if (parameter == 17) {
		chg_err("<~WPC~>set FOD parameter BPP17\n");
		nu1619_config_interface(chip, 0x0068, 0xBE, 0xFF);
		nu1619_config_interface(chip, 0x0069, 0x78, 0xFF);
		nu1619_config_interface(chip, 0x006A, 0x9C, 0xFF);
		nu1619_config_interface(chip, 0x006B, 0x78, 0xFF);
		nu1619_config_interface(chip, 0x006C, 0x96, 0xFF);
		nu1619_config_interface(chip, 0x006D, 0x2D, 0xFF);
		nu1619_config_interface(chip, 0x006E, 0x93, 0xFF);
		nu1619_config_interface(chip, 0x006F, 0x22, 0xFF);
		nu1619_config_interface(chip, 0x0070, 0x90, 0xFF);
		nu1619_config_interface(chip, 0x0071, 0x0E, 0xFF);
		nu1619_config_interface(chip, 0x0072, 0x98, 0xFF);
		nu1619_config_interface(chip, 0x0073, 0xE2, 0xFF);

		chip->nu1619_chg_status.FOD_parameter = parameter;
	} else if (parameter == 12) {
		chg_err("<~WPC~>set FOD parameter BPP12\n");
		nu1619_config_interface(chip, 0x0068, 0xC8, 0xFF);
		nu1619_config_interface(chip, 0x0069, 0x6E, 0xFF);
		nu1619_config_interface(chip, 0x006A, 0xAA, 0xFF);
		nu1619_config_interface(chip, 0x006B, 0x64, 0xFF);
		nu1619_config_interface(chip, 0x006C, 0xA0, 0xFF);
		nu1619_config_interface(chip, 0x006D, 0x2D, 0xFF);
		nu1619_config_interface(chip, 0x006E, 0x93, 0xFF);
		nu1619_config_interface(chip, 0x006F, 0x0E, 0xFF);
		nu1619_config_interface(chip, 0x0070, 0x93, 0xFF);
		nu1619_config_interface(chip, 0x0071, 0x0E, 0xFF);
		nu1619_config_interface(chip, 0x0072, 0x93, 0xFF);
		nu1619_config_interface(chip, 0x0073, 0x17, 0xFF);

		chip->nu1619_chg_status.FOD_parameter = parameter;
	} else if (parameter == 10) {
		chg_err("<~WPC~>set FOD parameter BPP10\n");
		nu1619_config_interface(chip, 0x0068, 0xBE, 0xFF);
		nu1619_config_interface(chip, 0x0069, 0x5F, 0xFF);
		nu1619_config_interface(chip, 0x006A, 0xAA, 0xFF);
		nu1619_config_interface(chip, 0x006B, 0x50, 0xFF);
		nu1619_config_interface(chip, 0x006C, 0xA2, 0xFF);
		nu1619_config_interface(chip, 0x006D, 0x4D, 0xFF);
		nu1619_config_interface(chip, 0x006E, 0x9B, 0xFF);
		nu1619_config_interface(chip, 0x006F, 0x40, 0xFF);
		nu1619_config_interface(chip, 0x0070, 0x9A, 0xFF);
		nu1619_config_interface(chip, 0x0071, 0x40, 0xFF);
		nu1619_config_interface(chip, 0x0072, 0x96, 0xFF);
		nu1619_config_interface(chip, 0x0073, 0x3F, 0xFF);

		chip->nu1619_chg_status.FOD_parameter = parameter;
	} else if (parameter == 1) {
		chg_err("<~WPC~>set FOD parameter BPP1\n");
		nu1619_config_interface(chip, 0x0068, 0x85, 0xFF);
		nu1619_config_interface(chip, 0x0069, 0x76, 0xFF);
		nu1619_config_interface(chip, 0x006A, 0x85, 0xFF);
		nu1619_config_interface(chip, 0x006B, 0x70, 0xFF);
		nu1619_config_interface(chip, 0x006C, 0x85, 0xFF);
		nu1619_config_interface(chip, 0x006D, 0x5C, 0xFF);
		nu1619_config_interface(chip, 0x006E, 0x9C, 0xFF);
		nu1619_config_interface(chip, 0x006F, 0x33, 0xFF);
		nu1619_config_interface(chip, 0x0070, 0x9C, 0xFF);
		nu1619_config_interface(chip, 0x0071, 0x2F, 0xFF);
		nu1619_config_interface(chip, 0x0072, 0xA4, 0xFF);
		nu1619_config_interface(chip, 0x0073, 0x17, 0xFF);

		chip->nu1619_chg_status.FOD_parameter = parameter;
	} else if (parameter == 0) {
		chg_err("<~WPC~>Disable FOD\n");
		nu1619_config_interface(chip, 0x0068, 0xFF, 0xFF);
		nu1619_config_interface(chip, 0x0069, 0x7F, 0xFF);
		nu1619_config_interface(chip, 0x006A, 0xFF, 0xFF);
		nu1619_config_interface(chip, 0x006B, 0x7F, 0xFF);
		nu1619_config_interface(chip, 0x006C, 0xFF, 0xFF);
		nu1619_config_interface(chip, 0x006D, 0x7F, 0xFF);
		nu1619_config_interface(chip, 0x006E, 0xFF, 0xFF);
		nu1619_config_interface(chip, 0x006F, 0x7F, 0xFF);
		nu1619_config_interface(chip, 0x0070, 0xFF, 0xFF);
		nu1619_config_interface(chip, 0x0071, 0x7F, 0xFF);
		nu1619_config_interface(chip, 0x0072, 0xFF, 0xFF);
		nu1619_config_interface(chip, 0x0073, 0x7F, 0xFF);

		chip->nu1619_chg_status.FOD_parameter = parameter;
	}
}

static int nu1619_set_tx_Q_value(struct oplus_nu1619_ic *chip)
{
	chg_err("<~WPC~>nu1619_set_tx_Q_value----------->\n");

	nu1619_write_reg(chip, 0x0000, 0x38);
	nu1619_write_reg(chip, 0x0001, 0x48);
	nu1619_write_reg(chip, 0x0002, 0x00);
	nu1619_write_reg(chip, 0x0003, 0x41);
	nu1619_write_cmd_D(chip, 0x01);
	return 0;
}

static int nu1619_set_tx_charger_dect(struct oplus_nu1619_ic *chip)
{
	chg_err("<~WPC~>nu1619_set_tx_charger_dect----------->\n");
	nu1619_write_reg(chip, 0x0000, 0x48);
	nu1619_write_reg(chip, 0x0001, P9221_CMD_INDENTIFY_ADAPTER);
	nu1619_write_reg(chip, 0x0002, ~P9221_CMD_INDENTIFY_ADAPTER);
	nu1619_write_reg(chip, 0x0003, 0xFF);
	nu1619_write_reg(chip, 0x0004, 0x00);
	nu1619_write_cmd_D(chip, 0x01);
	return 0;
}

static int nu1619_set_tx_charger_fastcharge(struct oplus_nu1619_ic *chip)
{
	chg_err("<~WPC~>nu1619_set_tx_charger_fastcharge----------->\n");
	nu1619_write_reg(chip, 0x0000, 0x48);
	nu1619_write_reg(chip, 0x0001, P9221_CMD_INTO_FASTCHAGE);
	nu1619_write_reg(chip, 0x0002, ~P9221_CMD_INTO_FASTCHAGE);
	nu1619_write_reg(chip, 0x0003, 0xFF);
	nu1619_write_reg(chip, 0x0004, 0x00);
	nu1619_write_cmd_D(chip, 0x01);
	return 0;
}

int nu1619_set_tx_fan_pwm_pulse(struct oplus_nu1619_ic *chip)
{
	int pwm_pulse;
	pwm_pulse = chip->nu1619_chg_status.dock_fan_pwm_pulse;
	chg_err("<~WPC~>nu1619_set_tx_fan_pwm_pulse: %d----->\n", pwm_pulse);

	nu1619_write_reg(chip, 0x0000, 0x48);
	nu1619_write_reg(chip, 0x0001, P9221_CMD_SET_PWM_PULSE);
	nu1619_write_reg(chip, 0x0002, ~P9221_CMD_SET_PWM_PULSE);
	nu1619_write_reg(chip, 0x0003, pwm_pulse);
	nu1619_write_reg(chip, 0x0004, ~pwm_pulse);
	nu1619_write_cmd_D(chip, 0x01);
	return 0;
}

static int nu1619_set_tx_cep_timeout(struct oplus_nu1619_ic *chip)
{
	chg_err("<~WPC~>nu1619_set_tx_cep_timeout----------->\n");
	nu1619_write_reg(chip, 0x0000, 0x48);
	nu1619_write_reg(chip, 0x0001, P9221_CMD_SET_CEP_TIMEOUT);
	nu1619_write_reg(chip, 0x0002, ~P9221_CMD_SET_CEP_TIMEOUT);
	nu1619_write_reg(chip, 0x0003, 0xFF);
	nu1619_write_reg(chip, 0x0004, 0x00);
	nu1619_write_cmd_D(chip, 0x01);
	return 0;
}

static int nu1619_set_tx_led_brightness(struct oplus_nu1619_ic *chip)
{
	int pwm_pulse;
	pwm_pulse = chip->nu1619_chg_status.dock_led_pwm_pulse;
	chg_err("<~WPC~>nu1619_set_tx_led_brightness: %d----->\n", pwm_pulse);
	nu1619_write_reg(chip, 0x0000, 0x48);
	nu1619_write_reg(chip, 0x0001, P9221_CMD_SET_LED_BRIGHTNESS);
	nu1619_write_reg(chip, 0x0002, ~P9221_CMD_SET_LED_BRIGHTNESS);
	nu1619_write_reg(chip, 0x0003, pwm_pulse);
	nu1619_write_reg(chip, 0x0004, ~pwm_pulse);
	nu1619_write_cmd_D(chip, 0x01);
	return 0;
}

static void nu1619_set_rx_charge_voltage(struct oplus_nu1619_ic *chip, int vol)
{
	char write_data_vout[2] = { 0, 0 };
	char write_data_vrect[2] = { 0, 0 };
	chg_err("<~WPC~> nu1619_set_rx_charge_voltage: %d\n", vol);
	chip->nu1619_chg_status.charge_voltage = vol;
	write_data_vout[0] = vol & 0x00FF;
	write_data_vout[1] = (vol & 0xFF00) >> 8;
	nu1619_write_reg(chip, 0x0000, write_data_vout[1]);
	nu1619_write_reg(chip, 0x0001, write_data_vout[0]);
	write_data_vrect[0] = (vol + 100) & 0x00FF;
	write_data_vrect[1] = ((vol + 100) & 0xFF00) >> 8;
	nu1619_write_reg(chip, 0x0002, write_data_vrect[1]);
	nu1619_write_reg(chip, 0x0003, write_data_vrect[0]);
	nu1619_write_cmd_D(chip, 0x02);
}


static void nu1619_set_rx_charge_current(struct oplus_nu1619_ic *chip, int chg_current)
{
	chg_err("<~WPC~> set charge current: %d\n", chg_current);
	chip->nu1619_chg_status.charge_current = chg_current;
	mp2650_charging_current_write_fast(chip->nu1619_chg_status.charge_current);
}

static void nu1619_set_rx_terminate_voltage(struct oplus_nu1619_ic *chip, int vol_value)
{
	chg_err("<~WPC~> set terminate voltage: %d\n", vol_value);
	chip->nu1619_chg_status.terminate_voltage = vol_value;
	mp2650_float_voltage_write(vol_value);
}

static int nu1619_increase_vc_by_step(struct oplus_nu1619_ic *chip, int src_value, int limit_value, int step_value)
{
	int temp_value;

	if (src_value >= limit_value) {
		return 0;
	}

	temp_value = src_value + step_value;
	if (temp_value > limit_value) {
		temp_value = limit_value;
	}

	return temp_value;
}

static int nu1619_decrease_vc_by_step(struct oplus_nu1619_ic *chip, int src_value, int limit_value, int step_value)
{
	int temp_value = 0;

	if (src_value <= limit_value) {
		return 0;
	}

	if (src_value >= step_value) {
		temp_value = src_value - step_value;
	}

	if (temp_value < limit_value) {
		temp_value = limit_value;
	}

	return temp_value;
}

static int nu1619_decrease_vout_to_target(struct oplus_nu1619_ic *chip, int vout_target, int vout_min)
{
	int temp_value;

	chg_err("<~WPC~> vout_setting[%d], vout_sample[%d],target_voltage[%d]\n",
			chip->nu1619_chg_status.charge_voltage,
			chip->nu1619_chg_status.vout,
			vout_target);

	if (chip->nu1619_chg_status.vout > vout_target) {
		if (chip->nu1619_chg_status.charge_voltage > vout_target) {
			temp_value = nu1619_decrease_vc_by_step(chip,
													chip->nu1619_chg_status.charge_voltage,
													vout_target,
													WPC_CHARGE_VOLTAGE_CHANGE_STEP_1V);
		} else {
			temp_value = nu1619_decrease_vc_by_step(chip,
													chip->nu1619_chg_status.charge_voltage,
													vout_min,
													WPC_CHARGE_VOLTAGE_CHANGE_STEP_100MV);
		}

		if (temp_value != 0) {
			nu1619_set_rx_charge_voltage(chip, temp_value);
		} else if (chip->nu1619_chg_status.charge_voltage <= vout_min) {
			return 0;
		}
	} else {
		return 0;
	}

	return -1;
}

static int nu1619_increase_vout_to_target(struct oplus_nu1619_ic *chip, int vout_target, int vout_max)
{
	int temp_value;

	chg_err("<~WPC~> vout_setting[%d], vout_sample[%d],target_voltage[%d]\n",
			chip->nu1619_chg_status.charge_voltage,
			chip->nu1619_chg_status.vout,
			vout_target);

	if (chip->nu1619_chg_status.vout < vout_target) {
		if (chip->nu1619_chg_status.charge_voltage < vout_target) {
			temp_value = nu1619_increase_vc_by_step(chip,
													chip->nu1619_chg_status.charge_voltage,
													vout_target,
													WPC_CHARGE_VOLTAGE_CHANGE_STEP_1V);
		} else {
			temp_value = nu1619_increase_vc_by_step(chip,
													chip->nu1619_chg_status.charge_voltage,
													vout_max,
													WPC_CHARGE_VOLTAGE_CHANGE_STEP_100MV);
		}

		if (temp_value != 0) {
			nu1619_set_rx_charge_voltage(chip, temp_value);
		} else if (chip->nu1619_chg_status.charge_voltage >= vout_max) {
			return 0;
		}
	} else {
		return 0;
	}

	return -1;
}

static void nu1619_reset_variables(struct oplus_nu1619_ic *chip)
{
	chip->nu1619_chg_status.tx_online = false;
	chip->nu1619_chg_status.freq_threshold = 135;
	chip->nu1619_chg_status.freq_check_count = 0;
	chip->nu1619_chg_status.freq_thr_inc = false;
	chip->nu1619_chg_status.is_deviation = false;
	chip->nu1619_chg_status.deviation_check_done = false;
	chip->nu1619_chg_status.charge_status = WPC_CHG_STATUS_DEFAULT;
	chip->nu1619_chg_status.charge_online = false;
	chip->nu1619_chg_status.send_message = P9221_CMD_NULL;
	chip->nu1619_chg_status.send_msg_cnt = 0;
	chip->nu1619_chg_status.adapter_type = ADAPTER_TYPE_UNKNOW;
	chip->nu1619_chg_status.charge_type = WPC_CHARGE_TYPE_DEFAULT;
	chip->nu1619_chg_status.dock_version = 0;
	chip->nu1619_chg_status.charge_voltage = 0;
	chip->nu1619_chg_status.charge_current = 0;
	chip->nu1619_chg_status.CEP_ready = false;
	chip->nu1619_chg_status.vrect = 0;
	chip->nu1619_chg_status.vout = 0;
	chip->nu1619_chg_status.iout = 0;
	chip->nu1619_chg_status.fastchg_ing = false;
	chip->nu1619_chg_status.epp_working = false;
	chip->nu1619_chg_status.idt_fw_updating = false;
	chip->nu1619_chg_status.wpc_reach_stable_charge = false;
	chip->nu1619_chg_status.wpc_reach_4370mv = false;
	chip->nu1619_chg_status.wpc_reach_4500mv = false;
	chip->nu1619_chg_status.wpc_ffc_charge = false;
	chip->nu1619_chg_status.wpc_skewing_proc = false;
	chip->nu1619_chg_status.skewing_info = false;
	chip->nu1619_chg_status.cep_info = 0;
	chip->nu1619_chg_status.rx_runing_mode = RX_RUNNING_MODE_UNKNOW;
	chip->nu1619_chg_status.adapter_power = ADAPTER_POWER_NUKNOW;
	chip->nu1619_chg_status.idt_adc_test_result = false;
	chip->nu1619_chg_status.need_doublecheck_to_cp = false;
	chip->nu1619_chg_status.doublecheck_ok = true;
	chip->nu1619_chg_status.epp_current_limit = 0;
	chip->nu1619_chg_status.BPP_current_limit = 0;
	chip->nu1619_chg_status.wpc_chg_param.pre_input_ma = 0;
#ifdef DEBUG_FASTCHG_BY_ADB
	chip->nu1619_chg_status.vout_debug_mode = false;
	chip->nu1619_chg_status.iout_debug_mode = false;
#endif

	chip->nu1619_chg_status.FOD_parameter = 0;
}

static void nu1619_init(struct oplus_nu1619_ic *chip)
{
	char write_data[2] = {0, 0};
	return;

	write_data[0] = 0x17;
	write_data[1] = 0x00;
	nu1619_write_reg_multi_byte(chip, 0x0038, write_data, 2);

	write_data[0] = 0x30;
	write_data[1] = 0x00;
	nu1619_write_reg_multi_byte(chip, 0x0056, write_data, 2);


	write_data[0] = 0x20;
	write_data[1] = 0x00;
	nu1619_write_reg_multi_byte(chip, 0x004E, write_data, 2);
}

static void nu1619_bpp_current_slow_init(struct oplus_nu1619_ic *chip, int chg_current)
{
	int temp_current = 50;

	while (temp_current < chg_current) {
		mp2650_input_current_limit_without_aicl(temp_current);
		nu1619_set_rx_charge_current(chip, temp_current);
		temp_current += 50;
		msleep(100);
	}
	return;
}

static void nu1619_charger_init(struct oplus_nu1619_ic *chip)
{
	mp2650_reset_charger();

	mp2650_set_chging_term_disable();

	mp2650_float_voltage_write(WPC_TERMINATION_VOLTAGE_CV);

	mp2650_set_enable_volatile_writes();

	mp2650_set_complete_charge_timeout(OVERTIME_DISABLED);

	mp2650_set_prechg_voltage_threshold();

	mp2650_set_prechg_current(WPC_PRECHARGE_CURRENT);

	mp2650_disable_buck_switch();

	if (chip->nu1619_chg_status.rx_runing_mode == RX_RUNNING_MODE_EPP) {
		chg_err("<~WPC~> nu1619_charger_init for EPP\n");

		mp2650_set_vindpm_vol(8700);

		chip->nu1619_chg_status.epp_current_step = WPC_CHARGE_CURRENT_EPP_INIT;
		mp2650_input_current_limit_without_aicl(chip->nu1619_chg_status.epp_current_step);
		nu1619_set_rx_charge_current(chip, WPC_CHARGE_CURRENT_DEFAULT);
	} else {
		chg_err("<~WPC~> nu1619_charger_init for BPP\n");

		mp2650_set_vindpm_vol(4800);

		chip->nu1619_chg_status.BPP_current_limit = 300;
		nu1619_bpp_current_slow_init(chip, chip->nu1619_chg_status.BPP_current_limit);
		chip->nu1619_chg_status.BPP_current_step_cnt = BPP_CURRENT_INCREASE_TIME;
	}

	mp2650_set_termchg_current(WPC_TERMINATION_CURRENT);

	mp2650_set_rechg_voltage(WPC_RECHARGE_VOLTAGE_OFFSET);

	mp2650_set_switching_frequency();

	mp2650_set_mps_otg_current();

	mp2650_set_mps_otg_voltage(false);
	mp2650_set_mps_second_otg_voltage(false);

	mp2650_unsuspend_charger();

	mp2650_enable_charging();

	mp2650_set_wdt_timer(REG09_MP2650_WTD_TIMER_40S);

	chargepump_disable();
}

static void nu1619_self_reset(struct oplus_nu1619_ic *chip, bool to_BPP)
{
	chip->nu1619_chg_status.wpc_self_reset = to_BPP;
	nu1619_set_vt_sleep_val(1);
	schedule_delayed_work(&chip->nu1619_self_reset_work, round_jiffies_relative(msecs_to_jiffies(1000)));
}
/*
static void nu1619_read_debug_registers(struct oplus_nu1619_ic *chip)
{
	char debug_data[10];

	nu1619_read_reg(chip, 0x04B0, debug_data, 10);
	chg_err("<linshangbo> P9415 DEBUG DATA: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
			debug_data[0], debug_data[1], debug_data[2], debug_data[3], debug_data[4], debug_data[5], debug_data[6], debug_data[7], debug_data[8], debug_data[9]);

}
*/
int nu1619_wpc_get_adapter_type(void)
{
	if (!nu1619_chip) {
		chg_err("<~WPC~> nu1619_chip is NULL\n");
		return false;
	} else {
		return nu1619_chip->nu1619_chg_status.adapter_type;
	}
}

bool nu1619_wpc_get_normal_charging(void)
{
	if (!nu1619_chip) {
		chg_err("<~WPC~> nu1619_chip is NULL\n");
		return false;
	}

	if(nu1619_chip->wireless_mode == WIRELESS_MODE_RX) {
		if(nu1619_chip->nu1619_chg_status.fastchg_ing) {
			return false;
		} else {
			return true;
		}
	} else {
		return false;
	}
}

bool nu1619_wpc_get_fast_charging(void)
{
	if (!nu1619_chip) {
		chg_err("<~WPC~> nu1619_chip is NULL\n");
		return false;
	}

	if(nu1619_chip->wireless_mode == WIRELESS_MODE_RX) {
		if(nu1619_chip->nu1619_chg_status.fastchg_ing) {
			return true;
		} else {
			return false;
		}
	} else {
		return false;
	}
}

bool nu1619_wpc_get_ffc_charging(void)
{
	if (!nu1619_chip) {
		chg_err("<~WPC~> nu1619_chip is NULL\n");
		return false;
	}

	if (nu1619_chip->wireless_mode == WIRELESS_MODE_RX) {
		if (nu1619_chip->nu1619_chg_status.wpc_ffc_charge) {
			return true;
		} else {
			return false;
		}
	} else {
		return false;
	}
}

bool nu1619_wpc_get_otg_charging(void)
{
	if (!nu1619_chip) {
		chg_err("<~WPC~> nu1619_chip is NULL\n");
		return false;
	}

	if(nu1619_chip->wireless_mode == WIRELESS_MODE_TX) {
		return true;
	} else {
		return false;
	}
}

int nu1619_get_rx_temperature(void)
{
	char temp_data[2] = {0, 0};
	int temp_value;
	int rc;

	rc = nu1619_read_reg(nu1619_chip, 0x46, temp_data, 2);
	if (rc != 0) {
		chg_err("<~WPC~>can't read 0x46!\n");
		return -1;
	}

	/*chg_err("<~WPC~>0x46 REG: 0x%02X 0x%02X\n", temp_data[0], temp_data[1]);*/
	temp_value = (int)(temp_data[0] | temp_data[1] << 8);
	temp_value = temp_value * 75 / 1000 - 174;

	return temp_value;
}

void nu1619_set_rtx_function_prepare(void)
{
	if (!g_oplus_chip) {
		chg_err("<~WPC~> g_oplus_chip is NULL\n");
		return;
	}

	if (!nu1619_chip) {
		chg_err("<~WPC~> nu1619_chip is NULL\n");
		return;
	}

	/*nu1619_chip->nu1619_chg_status.wpc_dischg_status == WPC_DISCHG_STATUS_OFF;*/
	wlchg_reset_variables(nu1619_chip);
}

static void nu1619_disable_tx_power(void)
{
		chg_err("<~WPC~> nu1619_disable_tx_power\n");
		if (oplus_get_wired_chg_present() == true) {
			chg_err("<~WPC~> wired_chg_present\n");
			nu1619_set_booster_en_val(0);
			nu1619_set_ext2_wireless_otg_en_val(1);
		} else if (oplus_get_wired_otg_online() == true) {
			chg_err("<~WPC~> wired_otg_online\n");
			/*mp2650_wireless_set_mps_otg_en_val(0);in mp2650_otg_disable*/
			mp2650_otg_disable();
			oplus_set_wrx_otg_value(0);
		} else {
			mp2650_otg_disable();
			/*mp2650_wireless_set_mps_otg_en_val(0);in mp2650_otg_disable*/
			oplus_set_wrx_otg_value(0);
			mp2650_enable_charging();
			msleep(100);
			oplus_set_wrx_en_value(0);
		}

		mp2650_set_mps_otg_voltage(false);
		mp2650_set_mps_second_otg_voltage(false);
}

static int nu1619_start_tx(struct oplus_nu1619_ic *chip)
{
	int ret = -1;
	static int cnt = 0;
	mutex_lock(&nu1619_i2c_access);
	while (ret !=0 && ++cnt < 10) {
		ret = __nu1619_write_reg(chip, 0x000D, 0x20);
		msleep(10);
	}
	chg_err("nu1619_start_tx,ret=%d,cnt=%d", ret, cnt);
	cnt = 0;
	mutex_unlock(&nu1619_i2c_access);
	return ret;
}

void nu1619_set_rtx_function(bool is_on)
{
	if (!g_oplus_chip) {
		chg_err("<~WPC~> g_oplus_chip is NULL\n");
		return;
	}

	if (!nu1619_chip) {
		chg_err("<~WPC~> nu1619_chip is NULL\n");
		return;
	}

	if (nu1619_firmware_is_updating() == true) {
		chg_err("<~WPC~> FW is updating, return!\n");
		return;
	}

	if (is_on) {
		if (nu1619_chip->nu1619_chg_status.wpc_dischg_status != WPC_DISCHG_STATUS_OFF) {
			chg_err("<~WPC~> Rtx function has already enabled!\n");
			return;
		}

		chg_err(" !!!!! <~WPC~> Enable rtx function!\n");

		cancel_delayed_work_sync(&nu1619_chip->idt_dischg_work);

		if (oplus_get_wired_chg_present() == true) {
			chg_err("<~WPC~> wired_chg_present\n");
			nu1619_set_booster_en_val(1);
			nu1619_set_ext2_wireless_otg_en_val(0);
		} else if (oplus_get_wired_otg_online() == true) {
			chg_err("<~WPC~> wired_otg_online\n");
			mp2650_disable_charging();
			oplus_set_wrx_en_value(1);
			mp2650_set_mps_otg_voltage(true);
			mp2650_set_mps_second_otg_voltage(false);
			mp2650_set_mps_otg_current();
			oplus_set_wrx_otg_value(1);
			mp2650_set_voltage_slew_rate(1);
			/*mp2650_wireless_set_mps_otg_en_val(1);in mp2650_otg_enable*/
			mp2650_otg_enable();
		} else {
			mp2650_disable_charging();
			oplus_set_wrx_en_value(1);
			mp2650_set_mps_otg_voltage(true);
			mp2650_set_mps_second_otg_voltage(false);
			mp2650_set_mps_otg_current();
			oplus_set_wrx_otg_value(1);
			mp2650_set_voltage_slew_rate(1);
			/*mp2650_wireless_set_mps_otg_en_val(1);in mp2650_otg_enable*/
			mp2650_otg_enable();
		}

		nu1619_chip->wireless_mode = WIRELESS_MODE_TX;

		msleep(1000);
		nu1619_start_tx(nu1619_chip);
		nu1619_chip->nu1619_chg_status.wpc_dischg_status = WPC_DISCHG_STATUS_ON;

		/*cancel_delayed_work_sync(&nu1619_chip->idt_dischg_work);*/
		schedule_delayed_work(&nu1619_chip->idt_dischg_work, round_jiffies_relative(msecs_to_jiffies(200)));
	} else {
		if (nu1619_chip->nu1619_chg_status.wpc_dischg_status == WPC_DISCHG_STATUS_OFF) {
			chg_err("<~WPC~> Rtx function has already disabled!\n");
			return;
		}

		chg_err(" !!!!! <~WPC~> Disable rtx function!\n");

		cancel_delayed_work_sync(&nu1619_chip->idt_dischg_work);

		nu1619_disable_tx_power();

		nu1619_chip->wireless_mode = WIRELESS_MODE_NULL;
		nu1619_chip->nu1619_chg_status.tx_online = false;
		nu1619_chip->nu1619_chg_status.wpc_dischg_status = WPC_DISCHG_STATUS_OFF;
	}
	wpc_battery_update();
}

int nu1619_enable_ftm(bool enable)
{
	if (!nu1619_chip) {
		chg_err("<~WPC~> nu1619_chip is NULL\n");
		return -1;
	}

	chg_err("<~WPC~> nu1619_enable_ftm: %d \n", enable);

	nu1619_chip->nu1619_chg_status.ftm_mode = enable;
	return 0;
}

/*
void nu1619_reg_print(void)
{
	char debug_data[13];

#if 0
	nu1619_read_reg(nu1619_chip, 0x4e0, debug_data, 13);
	chg_err("<~WPC~>0x4E0 REG: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n", 
			debug_data[0],debug_data[1],debug_data[2],debug_data[3],debug_data[4],debug_data[5],debug_data[6],debug_data[7],
			debug_data[8],debug_data[9],debug_data[10],debug_data[11],debug_data[12]);

	nu1619_read_reg(nu1619_chip, 0x6e, debug_data, 2);
	chg_err("<~WPC~>0x6E REG: 0x%02X 0x%02X\n", debug_data[0], debug_data[1]);

	nu1619_read_reg(nu1619_chip, 0x70, debug_data, 2);
	chg_err("<~WPC~>0x70 REG: 0x%02X 0x%02X\n", debug_data[0], debug_data[1]);


	nu1619_read_reg(nu1619_chip, 0x74, debug_data, 2);
	chg_err("<~WPC~>0x74 REG: 0x%02X 0x%02X\n", debug_data[0], debug_data[1]);


	nu1619_read_reg(nu1619_chip, 0x78, debug_data, 2);
	chg_err("<~WPC~>0x78 REG: 0x%02X 0x%02X\n", debug_data[0], debug_data[1]);


	nu1619_read_reg(nu1619_chip, 0x470, debug_data, 2);
	chg_err("<~WPC~>0x470 REG: 0x%02X 0x%02X\n", debug_data[0], debug_data[1]);
#endif

	if (nu1619_chip->nu1619_chg_status.wpc_dischg_status == WPC_DISCHG_STATUS_ON) {
		nu1619_read_reg(nu1619_chip, 0x78, debug_data, 2);
		chg_err("<~WPC~>rtx func 0x78-->[0x%02x]!\n", debug_data[0]);
		chg_err("<~WPC~>rtx func 0x79-->[0x%02x]!\n", debug_data[1]);
		nu1619_read_reg(nu1619_chip, 0x70, debug_data, 1);
		chg_err("<~WPC~>rtx func 0x70-->[0x%02x]!\n", debug_data[0]);
		nu1619_read_reg(nu1619_chip, 0x6E, debug_data, 1);
		chg_err("<~WPC~>rtx func 0x6E-->[0x%02x]!\n", debug_data[0]);
	}
}
*/

/*
static int nu1619_get_vrect_iout_offline(struct oplus_nu1619_ic * chip)
{
	char val_buf[5] = {0,0,0,0,0};
	int vout_value = 0;
	int vrect_value = 0;
	int iout_value = 0;
	int rc;

	rc = nu1619_read_reg(chip, 0x003C, val_buf, 2);
	if (rc != 0) {
		vout_value = 0;
	} else {
		vout_value = val_buf[0] | val_buf[1] << 8;
		vout_value = vout_value * 21000 / 4095;
	}

	rc = nu1619_read_reg(chip, 0x0040, val_buf, 2);
	if (rc != 0) {
		vrect_value = 0;
	} else {
		vrect_value = val_buf[0] | val_buf[1] << 8;
		vrect_value = vrect_value * 26250 / 4095;
	}

	rc = nu1619_read_reg(chip, 0x0044, val_buf, 2);
	if (rc != 0) {
		iout_value = 0;
	} else {
		iout_value = val_buf[0] | val_buf[1] << 8;
	}

	chg_err("<~WPC~> Vout:%d,  Vrect:%d, Iout:%d\n", vout_value, vrect_value, iout_value);

	return 0;
}
*/

void nu1619_wpc_print_log(void)
{
	if (!nu1619_chip) {
		chg_err("<~WPC~> nu1619_chip is NULL!\n");
		return;
	}

	/*nu1619_reg_print();*/

	chg_err("<~WPC~>vt_sleep[%d], idt_connect[%d], idt_int[%d], idt_en[%d], \
booster_en[%d], chargepump_en[%d], wrx_en[%d], wrx_otg_en[%d], mps_otg_en[%d], \
5V_en[%d], EXT2_otg_en[%d], EXT1_otg_en[%d]\n",
							nu1619_get_vt_sleep_val(),
							nu1619_get_idt_con_val(),
							nu1619_get_idt_int_val(),
							oplus_get_idt_en_val(),
							nu1619_get_booster_en_val(),
							chargepump_get_chargepump_en_val(),
							oplus_get_wrx_en_val(),
							oplus_get_wrx_otg_val(),
							mp2650_wireless_get_mps_otg_en_val(),
							nu1619_get_cp_ldo_5v_val(),
							nu1619_get_ext2_wireless_otg_en_val(),
							nu1619_get_ext1_wired_otg_en_val());

	if (!nu1619_chip->nu1619_chg_status.charge_online) {
		chg_err("<~WPC~> charge_online is false\n");
		/*nu1619_get_vrect_iout_offline(nu1619_chip);*/
		return;
	}

	chg_err("<~WPC~> [Chg sta: %d] [Adap type: %d] [Chg type: %d] [Dock Ver: %d] \
[Chg Vol: %d] [Chg Curr: %d] [Curr limit: %d] [ffc: %d] [silent: %d]\n",
			nu1619_chip->nu1619_chg_status.charge_status,
			nu1619_chip->nu1619_chg_status.adapter_type,
			nu1619_chip->nu1619_chg_status.charge_type,
			nu1619_chip->nu1619_chg_status.dock_version,
			nu1619_chip->nu1619_chg_status.charge_voltage,
			nu1619_chip->nu1619_chg_status.charge_current,
			nu1619_chip->nu1619_chg_status.fastchg_current_limit,
			nu1619_chip->nu1619_chg_status.wpc_ffc_charge,
			nu1619_chip->nu1619_chg_status.work_silent_mode);

	chg_err("<~WPC~> [Temp: %d] [Soc: %d] [Chg curr: %d] [Batt vol: %d]\
[Batt vol max: %d] [Batt vol min: %d] [Term Vol: %d] [Term Curr: %d]\n",
			g_oplus_chip->temperature,
			g_oplus_chip->soc,
			g_oplus_chip->icharging,
			g_oplus_chip->batt_volt,
			g_oplus_chip->batt_volt_max,
			g_oplus_chip->batt_volt_min,
			mp2650_get_termchg_voltage(),
			mp2650_get_termchg_current());

	/*chg_err("<~WPC~> P9415 Temperature = %d", nu1619_get_rx_temperature());*/
}

int nu1619_wireless_get_vout(void)
{
	if (!nu1619_chip) {
		return 0;
	}
	return nu1619_chip->nu1619_chg_status.vout;
}

static int nu1619_exit_write_mode(struct oplus_nu1619_ic *chip)
{
	int ret;

	chg_err("[nu1619] enter \n");

	ret = nu1619_write_reg(chip, 0x001a, 0x00);
	if (ret < 0)
		return ret;

	ret = nu1619_write_reg(chip, 0x0017, 0x00);
	if (ret < 0)
		return ret;

	return 1;
}

static int nu1619_enter_write_mode(struct oplus_nu1619_ic *chip)
{
	int ret;

	chg_err("[nu1619] enter \n");

	ret = nu1619_write_reg(chip, 0x0017, 0x01);
	if (ret < 0)
		return ret;

	ret = nu1619_write_reg(chip, 0x1000, 0x30);
	if (ret < 0)
		return ret;

	ret = nu1619_write_reg(chip, 0x001a, 0x5a);
	if (ret < 0)
		return ret;

	return 1;
}


static int nu1619_wirte_firmware_data(struct oplus_nu1619_ic *chip, unsigned short addr, unsigned short length)
{
	const char *fw_data = NULL;
	char addr_h, addr_l;
	int i = 0;
	char j = 0;
	unsigned short count = 0;
	int ret;
	/************write_mtp_addr************/
	if (addr == 0) {
		fw_data = nu1619_fw_data_boot;
	}
	if (addr == 256) {
		fw_data = nu1619_fw_data_rx;
	}
	if (addr == 4864) {
		fw_data = nu1619_fw_data_tx;
	}
	chg_err("[nu1619] addr=%d length=%d \n", addr, length);
	addr_h = (char)(addr >> 8);
	addr_l = (char)(addr & 0xff);
	ret = nu1619_write_reg(chip, 0x0010, addr_h);
	if (ret < 0) {
		return ret;
	}
	ret = nu1619_write_reg(chip, 0x0011, addr_l);
	if (ret < 0) {
		return ret;
	}
	/************write_mtp_addr************/
	ret = nu1619_enter_write_mode(chip);
	if (ret < 0) {
		return ret;
	}
	/************write data************/
	for (i = 0; i < length; i += 4) {
		ret = nu1619_write_reg(chip, 0x0012, fw_data[i + 3]);
		if (ret < 0) {
			return ret;
		}
		for (j = 0; j < 100; j++) {
			if (nu1619_get_idt_con_val() == 1) {
				usleep_range(10, 20);
				if (nu1619_get_idt_con_val() == 0) {
					break;
				}
			} else if (nu1619_get_idt_con_val() == 0) {
				usleep_range(100, 150);
				count++;
				break;
			}
		}
		ret = nu1619_write_reg(chip, 0x0012, fw_data[i + 2]);
		if (ret < 0) {
			return ret;
		}
		for (j = 0; j < 100; j++) {
			if (nu1619_get_idt_con_val() == 1) {
				usleep_range(10, 20);
				if (nu1619_get_idt_con_val() == 0) {
					break;
				}
			} else if (nu1619_get_idt_con_val() == 0) {
				usleep_range(100, 150);
				count++;
				break;
			}
		}
		ret = nu1619_write_reg(chip, 0x0012, fw_data[i + 1]);
		if (ret < 0) {
			return ret;
		}
		for (j = 0; j < 100; j++) {
			if (nu1619_get_idt_con_val() == 1) {
				usleep_range(10, 20);
				if (nu1619_get_idt_con_val() == 0) {
					break;
				}
			} else if (nu1619_get_idt_con_val() == 0) {
				usleep_range(100, 150);
				count++;
				break;
			}
		}
		ret = nu1619_write_reg(chip, 0x0012, fw_data[i + 0]);
		if (ret < 0) {
			return ret;
		}
		for (j = 0; j < 100; j++) {
			if (nu1619_get_idt_con_val() == 1) {
				usleep_range(10, 20);
				if (nu1619_get_idt_con_val() == 0) {
					break;
				}
			} else if (nu1619_get_idt_con_val() == 0) {
				usleep_range(100, 150);
				count++;
				break;
			}
		}
	}
	/************write data************/
	ret = nu1619_exit_write_mode(chip);
	if (ret < 0) {
		return ret;
	}
	chg_err("[nu1619] count=%d \n", count);
	return 1;
}


static int nu1619_config_gpio_1V8(struct oplus_nu1619_ic *chip)
{
	int ret;

	chg_err("[nu1619] enter \n");

	ret = nu1619_write_reg(chip, 0x1000, 0x10);
	if (ret < 0)
		return ret;

	ret = nu1619_write_reg(chip, 0x1130, 0x3e);
	if (ret < 0)
		return ret;

	return 1;
}


static int nu1619_enter_dtm_mode(struct oplus_nu1619_ic *chip)
{
	int ret;

	chg_err("[nu1619] enter \n");

	ret = nu1619_write_reg(chip, 0x2017, 0x69);
	if (ret < 0)
		return ret;

	ret = nu1619_write_reg(chip, 0x2017, 0x96);
	if (ret < 0)
		return ret;

	ret = nu1619_write_reg(chip, 0x2017, 0x66);
	if (ret < 0)
		return ret;

	ret = nu1619_write_reg(chip, 0x2017, 0x99);
	if (ret < 0)
		return ret;

	ret = nu1619_write_reg(chip, 0x2018, 0xff);
	if (ret < 0)
		return ret;

	ret = nu1619_write_reg(chip, 0x2019, 0xff);
	if (ret < 0)
		return ret;

	ret = nu1619_write_reg(chip, 0x0001, 0x5a);
	if (ret < 0)
		return ret;

	ret = nu1619_write_reg(chip, 0x0003, 0xa5);
	if (ret < 0)
		return ret;

	return 1;
}


static int nu1619_exit_dtm_mode(struct oplus_nu1619_ic *chip)
{
	int ret;

	chg_err("[nu1619] enter \n");

	ret = nu1619_write_reg(chip, 0x2018, 0x00);
	if (ret < 0)
		return ret;

	ret = nu1619_write_reg(chip, 0x2019, 0x00);
	if (ret < 0)
		return ret;

	ret = nu1619_write_reg(chip, 0x0001, 0x00);
	if (ret < 0)
		return ret;

	ret = nu1619_write_reg(chip, 0x0003, 0x00);
	if (ret < 0)
		return ret;

	ret = nu1619_write_reg(chip, 0x2017, 0x55);
	if (ret < 0)
		return ret;

	return 1;
}


bool nu1619_check_i2c_is_ok(struct oplus_nu1619_ic *chip)
{
	char read_data = 0;

	nu1619_write_reg(chip, 0x0000, 0x88);
	msleep(10);
	nu1619_read_reg(chip, 0x0000, &read_data, 1);

	if (read_data == 0x88) {
		return true;
	} else {
		return false;
	}
}

static bool nu1619_download_firmware_data(struct oplus_nu1619_ic *chip, char area)
{
	int ret;
	chg_err("[nu1619] [%s] enter \n", __func__);
	ret = nu1619_enter_dtm_mode(chip);
	if (ret < 0) {
		return false;
	}
	ret = nu1619_config_gpio_1V8(chip);
	if (ret < 0) {
		return false;
	}
	msleep(10);
	if (area == BOOT_AREA) {
		ret = nu1619_wirte_firmware_data(chip, 0, sizeof(nu1619_fw_data_boot));
		if (ret < 0) {
			return false;
		}
	}
	if (area == RX_AREA) {
		ret = nu1619_wirte_firmware_data(chip, 256, sizeof(nu1619_fw_data_rx));
		if (ret < 0) {
			return false;
		}
	}
	if (area == TX_AREA) {
		ret = nu1619_wirte_firmware_data(chip, 4864, sizeof(nu1619_fw_data_tx));
		if (ret < 0) {
			return false;
		}
	}
	ret = nu1619_exit_dtm_mode(chip);
	if (ret < 0) {
		return false;
	}
	chg_err("[nu1619] [%s] exit \n", __func__);
	return true;
}

bool nu1619_download_firmware_area(struct oplus_nu1619_ic * chip, char area)
{
	bool status = false;
	if (area == BOOT_AREA) {
		g_fw_data_lenth = sizeof(nu1619_fw_data_boot);
	} else if (area == RX_AREA) {
		g_fw_data_lenth = sizeof(nu1619_fw_data_rx);
	} else if (area == TX_AREA) {
		g_fw_data_lenth = sizeof(nu1619_fw_data_tx);
	}
	status = nu1619_download_firmware_data(chip, area);
	if (!status) {
		return false;
	}
	return true;
}

bool nu1619_download_firmware(struct oplus_nu1619_ic * chip)
{
	bool status = false;
	status = nu1619_download_firmware_area(chip, BOOT_AREA);
	if (!status) {
		chg_err("BOOT_AREA download fail!!! \n");
		return false;
	}
	msleep(20);
	status = nu1619_download_firmware_area(chip, RX_AREA);
	if (!status) {
		chg_err("RX_AREA download fail!!! \n");
		return false;
	}
	msleep(20);
	status = nu1619_download_firmware_area(chip, TX_AREA);
	if (!status) {
		chg_err("TX_AREA download fail!!! \n");
		return false;
	}
	return true;
}

static bool nu1619_req_checksum_and_fw_version(struct oplus_nu1619_ic *chip,
					       char *boot_check_sum, char *rx_check_sum, char *tx_check_sum, char *boot_ver, char *rx_ver, char *tx_ver)
{
	char read_buffer1[4];
	char read_buffer2[5];
	nu1619_write_reg(chip, 0x000d, 0x21);
	msleep(20);
	nu1619_read_reg(chip, 0x0008, read_buffer1, 3);
	chg_err("check sum value: 0x0008,9,a = 0x%x,0x%x,0x%x \n", read_buffer1[0], read_buffer1[1], read_buffer1[2]);
	nu1619_read_reg(chip, 0x0020, read_buffer2, 5);
	chg_err("0x0020,21,22,23,24 = 0x%x,0x%x,0x%x,0x%x,0x%x \n", read_buffer2[0], read_buffer2[1], read_buffer2[2], read_buffer2[3], read_buffer2[4]);
	*boot_check_sum = read_buffer1[0];
	*rx_check_sum = read_buffer1[1];
	*tx_check_sum = read_buffer1[2];
	*boot_ver = read_buffer2[0];
	*rx_ver = read_buffer2[1];
	*tx_ver = read_buffer2[2];
	return true;
}

static bool nu1619_check_firmware_version(struct oplus_nu1619_ic *chip)
{
	char boot_checksum, rx_checksum, tx_checksum, boot_version, rx_version, tx_version;
	bool g_boot_no_need_update = false;
	bool g_rx_no_need_update = false;
	bool g_tx_no_need_update = false;
	nu1619_req_checksum_and_fw_version(chip, &tx_checksum, &rx_checksum, &boot_checksum, &tx_version, &rx_version, &boot_version);
	chg_err("check_sum: 0x%x,0x%x,0x%x  Version:0x%x,0x%x,0x%x \n", tx_checksum, rx_checksum, boot_checksum, tx_version, rx_version, boot_version);
	chg_err("tx (0xff&(~nu1619_fw_data_tx[13308-5]))=0x%x \n", (0xff & (~nu1619_fw_data_tx[13308 - 5])));
	chg_err("rx (0xff&(~nu1619_fw_data_rx[18428-5]))=0x%x \n", (0xff & (~nu1619_fw_data_rx[18428 - 5])));
	chg_err("boot (0xff&(~nu1619_fw_data_boot[1020-5]))=0x%x \n", (0xff & (~nu1619_fw_data_boot[1020 - 5])));
	if ((tx_version == (0xff & (~nu1619_fw_data_tx[13308 - 5]))) && (tx_checksum == 0x66)) {
		g_tx_no_need_update = true;
		chg_err("[nu1619] [%s] g_tx_no_need_update = true \n", __func__);
	}
	if ((rx_version == (0xff & (~nu1619_fw_data_rx[18428 - 5]))) && (rx_checksum == 0x66)) {
		g_rx_no_need_update = true;
		chg_err("[nu1619] [%s] g_rx_no_need_update = true \n", __func__);
	}
	if ((boot_version == (0xff & (~nu1619_fw_data_boot[1020 - 5]))) && (boot_checksum == 0x66)) {
		g_boot_no_need_update = true;
		chg_err("[nu1619] [%s] g_boot_no_need_update = true \n", __func__);
	}
	chg_err(",g_tx_no_need_update=%d, g_rx_no_need_update=%d, g_boot_no_need_update=%d \n",
		g_tx_no_need_update, g_rx_no_need_update, g_boot_no_need_update);
	if ((g_tx_no_need_update) && (g_rx_no_need_update) && (g_boot_no_need_update)) {
		return false;
	}
	return true;
}

static bool nu1619_check_firmware(struct oplus_nu1619_ic *chip)
{
	char boot_checksum, rx_checksum, tx_checksum, boot_version, rx_version, tx_version;
	nu1619_req_checksum_and_fw_version(chip, &tx_checksum, &rx_checksum, &boot_checksum, &tx_version, &rx_version, &boot_version);
	chg_err("check_sum: 0x%x,0x%x,0x%x  Version:0x%x,0x%x,0x%x \n", tx_checksum, rx_checksum, boot_checksum, tx_version, rx_version, boot_version);
	if ((tx_checksum == 0x66) && (rx_checksum == 0x66) && (boot_checksum == 0x66)) {
		return true;
	} else {
		return false;
	}
}


bool nu1619_onekey_download_firmware(struct oplus_nu1619_ic * chip)
{
	bool status = false;
	chg_err("[nu1619] enter \n");
	if (!nu1619_check_i2c_is_ok(chip)) {
		chg_err(" i2c error! \n");
		return false;
	} else {
		chg_err(" i2c success! \n");
	}
	status = nu1619_check_firmware_version(chip);
	if (!status) {
		chg_err("FW is the Same, NO need download fw!!! \n");
		return true;
	}
	chg_err("***********************reset power************************* \n");
	nu1619_set_vbat_en_val(0);
	msleep(5);
	nu1619_set_vbat_en_val(1);
	msleep(20);
	status = nu1619_download_firmware(chip);
	if (!status) {
		chg_err("nu1619_download_firmware fail!!! \n");
		return false;
	}
	chg_err("***********************reset power************************* \n");
	nu1619_set_vbat_en_val(0);
	msleep(5);
	nu1619_set_vbat_en_val(1);
	msleep(20);
	status = nu1619_check_firmware(chip);
	if (!status) {
		chg_err("nu1619_check_firmware fail!!! \n");
		return false;
	}
	chg_err("[nu1619] end \n");
	return true;
}


static int nu1619_check_idt_fw_update(struct oplus_nu1619_ic *chip)
{
	int rc = -1;
/*
//	static int idt_update_retry_cnt = 0;
//	char temp[4] = {0, 0, 0, 0};
//	unsigned char *fw_buf;
//	int fw_size;
//	int fw_ver_start_addr = 0;
*/

	chg_err("<IDT UPDATE> check idt fw <><><><><><><><>\n");

	if (!chip) {
		chg_err("<IDT UPDATE> nu1619 isn't ready!\n");
		return rc;
	}

	if (g_oplus_chip->charger_exist == 1) {
		chg_err("<IDT UPDATE>g_oplus_chip->charger_exist == 1, return!\n");
		chip->nu1619_chg_status.check_fw_update = true;
		return 0;
	}

	if (nu1619_chip->nu1619_chg_status.charge_online) {
		chg_err("<SP UPDATE>nu1619_chip->nu1619_chg_status.charge_online == 1, return!\n");
		chip->nu1619_chg_status.check_fw_update = true;
		return 0;
	}

	chip->nu1619_chg_status.idt_fw_updating = true;
	mp2650_disable_charging();
	mp2650_set_mps_otg_voltage(false);
	mp2650_set_mps_second_otg_voltage(false);
	mp2650_set_mps_otg_current();
	mp2650_otg_enable();

	mp2650_wireless_set_mps_otg_en_val(1);
	oplus_set_wrx_en_value(1);
	oplus_set_wrx_otg_value(1);
	oplus_wireless_set_otg_en_val(0);
	msleep(1000);

	if(nu1619_onekey_download_firmware(chip) == true) {
		chip->nu1619_chg_status.check_fw_update = true;
	}

	mp2650_otg_disable();
	mp2650_otg_wait_vbus_decline();

	mp2650_wireless_set_mps_otg_en_val(0);
	oplus_set_wrx_en_value(0);
	oplus_set_wrx_otg_value(0);
	chip->nu1619_chg_status.idt_fw_updating = false;

	return rc;
}

bool nu1619_wpc_get_fw_updating(void)
{
	if (!nu1619_chip) {
		chg_err("<~WPC~> nu1619_chip is NULL\n");
		return false;
	} else {
		return nu1619_chip->nu1619_chg_status.idt_fw_updating;
	}
}

bool nu1619_firmware_is_updating(void)
{
	if (!nu1619_chip) {
		return 0;
	}

	return nu1619_chip->nu1619_chg_status.idt_fw_updating;
}

static int nu1619_get_vrect_iout(struct oplus_nu1619_ic * chip)
{
	static int iout_error_cnt = 0;
	char val_buf[5] = { 0, 0, 0, 0, 0 };
	int vout_value = 0;
	int vrect_value = 0;
	int iout_value = 0;
	int icharging = 0;
	int retry_count = 0;
READ_VOUT_RETRY:
	nu1619_write_reg(chip, 0x000c, 0xab);
	nu1619_read_reg(chip, 0x0008, val_buf, 3);
	if (val_buf[0] == (0xab ^ 0x80)) {
		vout_value = (val_buf[2] << 8) + val_buf[1];
		retry_count = 0;
	} else {
		if (retry_count < 3) {
			msleep(1);
			retry_count++;
			goto READ_VOUT_RETRY;
		}
		chg_err("<~WPC~> Read Vout ERROR\n");
		return -1;
	}
READ_VRECT_RETRY:
	nu1619_write_reg(chip, 0x000c, 0xad);
	nu1619_read_reg(chip, 0x0008, val_buf, 3);
	if (val_buf[0] == (0xad ^ 0x80)) {
		vrect_value = (val_buf[2] << 8) + val_buf[1];
		retry_count = 0;
	} else {
		if (retry_count < 3) {
			msleep(1);
			retry_count++;
			goto READ_VRECT_RETRY;
		}
		chg_err("<~WPC~> Read Vrect ERROR\n");
		return -1;
	}
READ_IOUT_RETRY:
	nu1619_write_reg(chip, 0x000c, 0xaf);
	nu1619_read_reg(chip, 0x0008, val_buf, 3);
	if (val_buf[0] == (0xaf ^ 0x80)) {
		iout_value = (val_buf[2] << 8) + val_buf[1];
		retry_count = 0;
	} else {
		if (retry_count < 3) {
			msleep(1);
			retry_count++;
			goto READ_IOUT_RETRY;
		}
		chg_err("<~WPC~> Read Iout ERROR\n");
		return -1;
	}
	chg_err("<~WPC~> Vout:%d,  Vrect:%d, Iout:%d\n", vout_value, vrect_value, iout_value);
	chip->nu1619_chg_status.vrect = vrect_value;
	switch (chip->nu1619_chg_status.charge_status) {
	case WPC_CHG_STATUS_FAST_CHARGING_FROM_CHGPUMP:
		if ((nu1619_get_CEP_flag(chip) == 0) && (abs(chip->nu1619_chg_status.charge_voltage - vout_value) > 2000)) {
			chg_err("<~WPC~>ERROR: VoutSet[%d] -- VoutRead[%d]\n", chip->nu1619_chg_status.charge_voltage, vout_value);
			return -1;
		}
		break;
	default:
		break;
	}
	chip->nu1619_chg_status.vout = vout_value;
	icharging = oplus_gauge_get_batt_current();
	if (chip->nu1619_chg_status.charge_status == WPC_CHG_STATUS_FAST_CHARGING_FROM_CHGPUMP) {
		if (abs(iout_value * 2 + icharging) > 1500) {
			iout_error_cnt++;
			chg_err("<~WPC~>ERROR: Iout*2[%d] -- icharging[%d]\n", iout_value * 2, icharging);
			if (iout_error_cnt >= 7) {
				iout_error_cnt = 0;
				return -1;
			} else {
				return 0;
			}
		} else {
			iout_error_cnt = 0;
		}
	}
	chip->nu1619_chg_status.iout = iout_value;
	return 0;
}


static int nu1619_get_vout(struct oplus_nu1619_ic * chip)
{
	char val_buf[5] = { 0, 0, 0, 0, 0 };
	int vout_value = 0;
	nu1619_write_reg(chip, 0x000c, 0xab);
	nu1619_read_reg(chip, 0x0008, val_buf, 3);
	if (val_buf[0] == (0xab ^ 0x80)) {
		vout_value = (val_buf[2] << 8) + val_buf[1];
	}
	return vout_value;
}


static void nu1619_begin_CEP_detect(struct oplus_nu1619_ic * chip)
{
	chip->nu1619_chg_status.CEP_ready = false;
	schedule_delayed_work(&chip->nu1619_CEP_work, P922X_CEP_INTERVAL);
}

static void nu1619_reset_CEP_flag(struct oplus_nu1619_ic * chip)
{
	chip->nu1619_chg_status.CEP_ready = false;
}

int nu1619_get_CEP_flag(struct oplus_nu1619_ic * chip)
{
	if (chip->nu1619_chg_status.CEP_ready == false) {
		chg_err("<~WPC~> CEP value isn't ready!\n");
		return -1;
	}

	if ((chip->nu1619_chg_status.CEP_value == 0) || (chip->nu1619_chg_status.CEP_value == 1)
		|| (chip->nu1619_chg_status.CEP_value == 2) || (chip->nu1619_chg_status.CEP_value == 0xFF)
		|| (chip->nu1619_chg_status.CEP_value == 0xFE)) {
		return 0;
	} else {
		return 1;
	}
}

static void nu1619_print_CEP_flag(struct oplus_nu1619_ic * chip)
{
	if ((nu1619_chip->nu1619_chg_status.charge_online) && (chip->nu1619_chg_status.CEP_ready)) {
		chg_err("<~WPC~> CEP value = %d\n", chip->nu1619_chg_status.CEP_value);
	}
}

static int to_cep_info(char cep_value)
{
	int cep_info = 0;

	cep_info = (0x80 & cep_value) ? (~cep_value + 1) : cep_value;
	return cep_info;
}

static int nu1619_detect_CEP(struct oplus_nu1619_ic * chip)
{
	int rc = -1;
	char temp = 0;
	int cep_info = 0;
	char val_buf[5] = { 0, 0, 0, 0, 0 };
	rc = nu1619_write_reg(chip, 0x000c, 0x95);
	rc = nu1619_read_reg(chip, 0x0008, val_buf, 3);
	if (val_buf[0] == (0x95 ^ 0x80)) {
		temp = val_buf[1];
	}
	chip->nu1619_chg_status.CEP_ready = true;
	chip->nu1619_chg_status.CEP_value = temp;
	if (chip->nu1619_chg_status.skewing_info == true) {
		cep_info = to_cep_info(chip->nu1619_chg_status.CEP_value);
		if (chip->nu1619_chg_status.cep_info < cep_info) {
			chip->nu1619_chg_status.cep_info = cep_info;
		}
	}
	return 0;
}

#ifndef FASTCHG_TEST_BY_TIME
static int nu1619_get_work_freq(struct oplus_nu1619_ic *chip, int *val)
{
	int rc;
	char temp;
return 0;
	rc = nu1619_read_reg(chip, 0x5e, &temp, 1);
	if (rc) {
		chg_err("Couldn't read rx freq val, rc = %d\n", rc);
		return rc;
	}
	*val = (int)temp;
	return rc;
}
#endif

static int nu1619_get_special_ID(struct oplus_nu1619_ic *chip)
{
	int rc;
	char temp[2];

return 0;
	rc = nu1619_read_reg(chip, 0xA2, temp, 2);
	if (rc) {
		chg_err("Couldn't read rx freq val, rc = %d\n", rc);
		return -1;
	}

	chg_err("<~WPC~>TX ID: 0x%02X 0x%02X\n", temp[0], temp[1]);
	if ((temp[0] == 0x5E) && (temp[1] == 0x00)) {
		return -1;
	} else {
		return 0;
	}
}

static bool nu1619_get_self_reset(void)
{
	struct oplus_nu1619_ic *chip = nu1619_chip;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: nu1619_chip not ready!\n", __func__);
		return false;
	}

	if (chip->nu1619_chg_status.wpc_self_reset) {
		return true;
	} else {
		return false;
	}
}

bool nu1619_wireless_charge_start(void)
{
	if (!nu1619_chip) {
		return 0;
	}

	if (nu1619_get_self_reset())
		return true;

	return nu1619_chip->nu1619_chg_status.charge_online;
}

void nu1619_set_wireless_charge_stop(void)
{
	/*ATTENTION: This function can't be used in idt_connect_int_work, nu1619_task_work, idt_event_int_work!*/

	if (!nu1619_chip) {
		return;
	}

	if (nu1619_chip->nu1619_chg_status.charge_online == true) {
		cancel_delayed_work_sync(&nu1619_chip->idt_connect_int_work);
		cancel_delayed_work_sync(&nu1619_chip->nu1619_task_work);
		cancel_delayed_work_sync(&nu1619_chip->idt_event_int_work);

		nu1619_reset_variables(nu1619_chip);

		nu1619_chip->nu1619_chg_status.charge_online = false;
	}
}

void nu1619_restart_charger(struct oplus_nu1619_ic *chip)
{
/*
	nu1619_set_rx_charge_current(chip, WPC_CHARGE_CURRENT_200MA);
	mp2650_input_current_limit_without_aicl(WPC_CHARGE_CURRENT_LIMIT_300MA);
*/
	nu1619_set_rx_terminate_voltage(chip, WPC_TERMINATION_VOLTAGE_CV);
/*
	mp2650_enable_charging();
	msleep(100);
*/
	chargepump_disable();
	nu1619_set_rx_charge_current(chip, WPC_CHARGE_CURRENT_FASTCHG);
/*
	mp2650_input_current_limit_without_aicl(1000);
	nu1619_set_rx_charge_voltage(chip, WPC_CHARGE_VOLTAGE_CHGPUMP_TO_CHARGER);
*/
}

static void nu1619_ready_to_switch_to_charger(struct oplus_nu1619_ic *chip, bool reset_ceptimeout)
{
	if (reset_ceptimeout) {
		nu1619_set_tx_cep_timeout_1500ms();
	}

	chg_err("<~WPC~> turn to mp2650 charge \n");
	chip->nu1619_chg_status.need_doublecheck_to_cp = true;
	chip->nu1619_chg_status.doublecheck_ok = false;

	nu1619_set_FOD_parameter(chip, 0);
	chip->nu1619_chg_status.charge_status = WPC_CHG_STATUS_DECREASE_IOUT_TO_200MA;
}
/*
static void nu1619_ready_to_switch_to_chargepump(struct oplus_nu1619_ic *chip)
{
	chg_err("<~WPC~> turn to chargepump charge \n");
	if (chip->nu1619_chg_status.need_doublecheck_to_cp == true)
		chip->nu1619_chg_status.doublecheck_ok = false;

	nu1619_set_FOD_parameter(chip, 0);
	chip->nu1619_chg_status.charge_status = WPC_CHG_STATUS_READY_FOR_FASTCHG;
}
*/
#ifdef DEBUG_FASTCHG_BY_ADB
static int nu1619_fastcharge_debug_by_adb(struct oplus_nu1619_ic *chip)
{
	int iout_max_value;
	int iout_min_value;
	int temp_value;

	if (!chip->nu1619_chg_status.iout_debug_mode && !chip->nu1619_chg_status.vout_debug_mode) {
		return -1;
	}

	if (chip->nu1619_chg_status.vout_debug_mode) {
		chg_err("<~WPC~> Vout debug mode is running...\n");
		return 0;
	}

	chg_err("<~WPC~> Iout debug mode is running...\n");

	if (nu1619_get_CEP_flag(chip) != 0) {
		return 0;
	}

	iout_max_value = chip->nu1619_chg_status.iout_stated_current + WPC_CHARGE_CURRENT_OFFSET;
	if (chip->nu1619_chg_status.iout_stated_current > WPC_CHARGE_CURRENT_OFFSET) {
		iout_min_value = chip->nu1619_chg_status.iout_stated_current - WPC_CHARGE_CURRENT_OFFSET;
	} else {
		iout_min_value = 0;
	}

	if (chip->nu1619_chg_status.iout > iout_max_value) {
		chg_err("<~WPC~> The Iout > %d.\n", iout_max_value);
		if (chip->nu1619_chg_status.charge_voltage > WPC_CHARGE_VOLTAGE_CHGPUMP_MIN) {
			temp_value = chip->nu1619_chg_status.iout - iout_max_value;
			if (chip->nu1619_chg_status.iout > 2100) {
				nu1619_set_rx_charge_voltage(chip, chip->nu1619_chg_status.charge_voltage - 200);
			} else if (temp_value > 50) {
				nu1619_set_rx_charge_voltage(chip, chip->nu1619_chg_status.charge_voltage - 100);
			} else {
				nu1619_set_rx_charge_voltage(chip, chip->nu1619_chg_status.charge_voltage - 20);
			}
			nu1619_reset_CEP_flag(chip);
		}
	} else if (chip->nu1619_chg_status.iout < iout_min_value) {
		chg_err("<~WPC~> The Iout < %d.\n", iout_min_value);
		if (chip->nu1619_chg_status.charge_voltage < WPC_CHARGE_VOLTAGE_CHGPUMP_MAX) {
			temp_value = iout_min_value - chip->nu1619_chg_status.iout;
			if ((temp_value > 100) && (chip->nu1619_chg_status.iout < 1800)) {
				nu1619_set_rx_charge_voltage(chip, chip->nu1619_chg_status.charge_voltage + 100);
			} else if ((temp_value > 50) && (chip->nu1619_chg_status.iout < 1800)) {
				nu1619_set_rx_charge_voltage(chip, chip->nu1619_chg_status.charge_voltage + 50);
			} else {
				nu1619_set_rx_charge_voltage(chip, chip->nu1619_chg_status.charge_voltage + 20);
			}
			nu1619_reset_CEP_flag(chip);
		}
	} else {
		chg_err("<~WPC~> The Iout is OK!\n");
	}

	return 0;
}
#endif

#ifdef FASTCHG_TEST_BY_TIME
static int nu1619_fastcharge_test_40w(struct oplus_nu1619_ic *chip)
{
	if (nu1619_test_charging_status() == 1) {
		chg_err("<~WPC~>[-TEST-]3min timeout,stop charging!\n");
		nu1619_set_vt_sleep_val(1);
		return -1;
	} else {
		chip->nu1619_chg_status.iout_stated_current = 2000;
	}

	return 0;
}

#else
static void nu1619_fastcharge_current_adjust_40w(struct oplus_nu1619_ic *chip, bool is_init)
{
	static int adjust_current_delay = 0;

	if (is_init) {
		adjust_current_delay = 0;
	}

	chg_err("<~WPC~> ~~~~~~~~40W fastcharg current adjust(delay %d)~~~~~~~~\n", adjust_current_delay);

	if ((g_oplus_chip->batt_volt >= 4500) && (!chip->nu1619_chg_status.wpc_ffc_charge)) {
		chg_err("<~WPC~> batt_volt[%d] >= 4.5V, Not in FFC\n", g_oplus_chip->batt_volt);
		nu1619_ready_to_switch_to_charger(chip, true);
		return;
	} else if (chip->nu1619_chg_status.wpc_ffc_charge && (g_oplus_chip->batt_volt >= 4500)
				&& ((g_oplus_chip->icharging < 0) && ((-1 * g_oplus_chip->icharging) < 900))) {
		chg_err("<~WPC~> batt_volt[%d] >= 4.5V, batt_cur[%d] < 900ma, In FFC\n", g_oplus_chip->batt_volt, (-1 * g_oplus_chip->icharging));
		chip->nu1619_chg_status.wpc_ffc_charge = false;
		nu1619_ready_to_switch_to_charger(chip, true);
		return;
	} else if (!chip->nu1619_chg_status.wpc_ffc_charge && g_oplus_chip->batt_volt >= 4370) {
		chg_err("<~WPC~> batt_volt[%d], Not in FFC\n", g_oplus_chip->batt_volt);
		nu1619_ready_to_switch_to_charger(chip, true);
		return;
	} else if (((g_oplus_chip->temperature < WPC_CHARGE_FFC_TEMP_MIN)
			|| (g_oplus_chip->temperature > WPC_CHARGE_FFC_TEMP_MAX)) && (g_oplus_chip->batt_volt > 4370)) {
		chg_err("<~WPC~> batt_volt[%d], temperature[%d]\n", g_oplus_chip->batt_volt, g_oplus_chip->temperature);
		chip->nu1619_chg_status.wpc_ffc_charge = false;
		nu1619_ready_to_switch_to_charger(chip, true);
		return;
	}
	if (g_oplus_chip->temperature < g_oplus_chip->limits.little_cool_bat_decidegc || g_oplus_chip->temperature > g_oplus_chip->limits.warm_bat_decidegc) {
		chg_err("<~WPC~> temperature[%d]\n", g_oplus_chip->temperature);
		chip->nu1619_chg_status.wpc_ffc_charge = false;
		nu1619_ready_to_switch_to_charger(chip, true);
		return;
	}

	if (g_oplus_chip->batt_volt >= 4370
		&& ((g_oplus_chip->icharging < 0) && ((-1 * g_oplus_chip->icharging) < 800))
		&& chip->nu1619_chg_status.wpc_reach_stable_charge == true) {
		chg_err("<~WPC~> icharging < 800. Exit FFC.\n");
		chip->nu1619_chg_status.wpc_ffc_charge = false;
		nu1619_set_rx_terminate_voltage(chip, WPC_TERMINATION_VOLTAGE_CV);
		nu1619_ready_to_switch_to_charger(chip, true);
		goto ADJUST_FINISH;
	}

	if ((nu1619_get_CEP_flag(chip) != 0) || chip->nu1619_chg_status.wpc_skewing_proc) {
		return;
	}

	if(adjust_current_delay > 0) {
		adjust_current_delay--;
		return;
	}

	if (chip->nu1619_chg_status.wpc_reach_stable_charge) {
		if (!chip->nu1619_chg_status.wpc_reach_4370mv && (g_oplus_chip->batt_volt >= 4370)) {
			chg_err("<~WPC~> First reach 4370mV.\n");
			if(fasctchg_current[chip->nu1619_chg_status.fastcharge_level] > fasctchg_current[FASTCHARGE_LEVEL_3]) {
				chg_err("<~WPC~> turn to %dMA charge\n", fasctchg_current[FASTCHARGE_LEVEL_3]);
				chip->nu1619_chg_status.iout_stated_current = fasctchg_current[FASTCHARGE_LEVEL_3];
				chip->nu1619_chg_status.fastcharge_level = FASTCHARGE_LEVEL_3;
			} else {
				if ((g_oplus_chip->icharging < 0) && ((-1 * g_oplus_chip->icharging) < WPC_CHARGE_CURRENT_FFC_TO_CV)) {
					chg_err("<~WPC~> reach 4370mV, icharging < 1A. Exit FFC.\n");
					chip->nu1619_chg_status.wpc_ffc_charge = false;
					nu1619_set_rx_terminate_voltage(chip, WPC_TERMINATION_VOLTAGE_CV);
					goto ADJUST_FINISH;
				}
			}
			chip->nu1619_chg_status.wpc_reach_4370mv = true;
			adjust_current_delay = 2;
			goto ADJUST_FINISH;
		}/* else if (chip->nu1619_chg_status.wpc_reach_4370mv && (g_oplus_chip->batt_volt >= 4370)) {
		}*/
	}

	if (g_oplus_chip->batt_volt >= 4500) {
		chg_err("<~WPC~> batt_volt[%d]\n", g_oplus_chip->batt_volt);
		chip->nu1619_chg_status.wpc_reach_4500mv = true;

		if (chip->nu1619_chg_status.fastcharge_level < FASTCHARGE_LEVEL_5) {
			chg_err("<~WPC~> turn to %dMA charge\n", fasctchg_current[FASTCHARGE_LEVEL_5]);
			chip->nu1619_chg_status.iout_stated_current = fasctchg_current[FASTCHARGE_LEVEL_5];
			chip->nu1619_chg_status.fastcharge_level = FASTCHARGE_LEVEL_5;
			adjust_current_delay = WPC_ADJUST_CV_DELAY;
			goto ADJUST_FINISH;
		} else if (chip->nu1619_chg_status.fastcharge_level == FASTCHARGE_LEVEL_5) {
			chg_err("<~WPC~> turn to %dMA charge\n", fasctchg_current[FASTCHARGE_LEVEL_6]);
			chip->nu1619_chg_status.iout_stated_current = fasctchg_current[FASTCHARGE_LEVEL_6];
			chip->nu1619_chg_status.fastcharge_level = FASTCHARGE_LEVEL_6;
			adjust_current_delay = WPC_ADJUST_CV_DELAY;
			goto ADJUST_FINISH;
		} else if (chip->nu1619_chg_status.fastcharge_level == FASTCHARGE_LEVEL_6) {
			chg_err("<~WPC~> turn to %dMA charge\n", fasctchg_current[FASTCHARGE_LEVEL_7]);
			chip->nu1619_chg_status.iout_stated_current = fasctchg_current[FASTCHARGE_LEVEL_7];
			chip->nu1619_chg_status.fastcharge_level = FASTCHARGE_LEVEL_7;
			adjust_current_delay = WPC_ADJUST_CV_DELAY;
			goto ADJUST_FINISH;
		} else if (chip->nu1619_chg_status.fastcharge_level == FASTCHARGE_LEVEL_7) {
			chg_err("<~WPC~> turn to %dMA charge\n", fasctchg_current[FASTCHARGE_LEVEL_8]);
			chip->nu1619_chg_status.iout_stated_current = fasctchg_current[FASTCHARGE_LEVEL_8];
			chip->nu1619_chg_status.fastcharge_level = FASTCHARGE_LEVEL_8;
			adjust_current_delay = WPC_ADJUST_CV_DELAY;
			goto ADJUST_FINISH;
		} else if (chip->nu1619_chg_status.fastcharge_level == FASTCHARGE_LEVEL_8) {
			chg_err("<~WPC~> turn to %dMA charge\n", fasctchg_current[FASTCHARGE_LEVEL_9]);
			chip->nu1619_chg_status.iout_stated_current = fasctchg_current[FASTCHARGE_LEVEL_9];
			chip->nu1619_chg_status.fastcharge_level = FASTCHARGE_LEVEL_9;
			adjust_current_delay = WPC_ADJUST_CV_DELAY;
			goto ADJUST_FINISH;
		} else if (chip->nu1619_chg_status.fastcharge_level == FASTCHARGE_LEVEL_9) {
			chip->nu1619_chg_status.wpc_ffc_charge = false;
			nu1619_ready_to_switch_to_charger(chip, true);
			return;
		}
	}

	if (g_oplus_chip->temperature >= 420) {
		chg_err("<~WPC~> The tempearture is >= 42. fastchg current: %d\n", fasctchg_current[chip->nu1619_chg_status.fastcharge_level]);

		if (chip->nu1619_chg_status.fastcharge_level < FASTCHARGE_LEVEL_3) {
			chg_err("<~WPC~> turn to %dMA charge\n", fasctchg_current[FASTCHARGE_LEVEL_3]);
			chip->nu1619_chg_status.iout_stated_current = fasctchg_current[FASTCHARGE_LEVEL_3];
			chip->nu1619_chg_status.fastcharge_level = FASTCHARGE_LEVEL_3;
			adjust_current_delay = 35;
		} else if (chip->nu1619_chg_status.fastcharge_level == FASTCHARGE_LEVEL_3) {
			chg_err("<~WPC~> turn to %dMA charge\n", fasctchg_current[FASTCHARGE_LEVEL_4]);
			chip->nu1619_chg_status.iout_stated_current = fasctchg_current[FASTCHARGE_LEVEL_4];
			chip->nu1619_chg_status.fastcharge_level = FASTCHARGE_LEVEL_4;
			adjust_current_delay = 35;
		} else if (chip->nu1619_chg_status.fastcharge_level == FASTCHARGE_LEVEL_4) {
			chg_err("<~WPC~> turn to %dMA charge\n", fasctchg_current[FASTCHARGE_LEVEL_5]);
			chip->nu1619_chg_status.iout_stated_current = fasctchg_current[FASTCHARGE_LEVEL_5];
			chip->nu1619_chg_status.fastcharge_level = FASTCHARGE_LEVEL_5;
			adjust_current_delay = 35;
		} else {
			chip->nu1619_chg_status.wpc_ffc_charge = false;
			nu1619_ready_to_switch_to_charger(chip, true);
			return;
		}
	} else if (g_oplus_chip->temperature >= 390) {
		chg_err("<~WPC~> The tempearture is >= 39. fastcharge_level: %d\n", chip->nu1619_chg_status.fastcharge_level);
		if (chip->nu1619_chg_status.fastcharge_level > FASTCHARGE_LEVEL_2) {
		} else if (chip->nu1619_chg_status.fastcharge_level != FASTCHARGE_LEVEL_2) {
			chg_err("<~WPC~> turn to %dMA charge\n", fasctchg_current[FASTCHARGE_LEVEL_2]);
			chip->nu1619_chg_status.iout_stated_current = fasctchg_current[FASTCHARGE_LEVEL_2];
			chip->nu1619_chg_status.fastcharge_level = FASTCHARGE_LEVEL_2;
			adjust_current_delay = WPC_ADJUST_CV_DELAY;
		}
	} else {
		chg_err("<~WPC~> The tempearture is < 39. fastcharge_level: %d\n", chip->nu1619_chg_status.fastcharge_level);
		if (chip->nu1619_chg_status.fastcharge_level > FASTCHARGE_LEVEL_1) {
		} else if (chip->nu1619_chg_status.fastcharge_level != FASTCHARGE_LEVEL_1) {
			chg_err("<~WPC~> turn to %dMA charge\n", fasctchg_current[FASTCHARGE_LEVEL_1]);
			chip->nu1619_chg_status.iout_stated_current = fasctchg_current[FASTCHARGE_LEVEL_1];
			chip->nu1619_chg_status.fastcharge_level = FASTCHARGE_LEVEL_1;
			adjust_current_delay = WPC_ADJUST_CV_DELAY;
		}
	}

ADJUST_FINISH:
	chip->nu1619_chg_status.iout_stated_current = fasctchg_current[chip->nu1619_chg_status.fastcharge_level];
	if (chip->nu1619_chg_status.iout_stated_current > chip->nu1619_chg_status.fastchg_current_limit) {
		chg_err("<~WPC~> charge_current[%d] > charge_current_limit[%d]\n",
				chip->nu1619_chg_status.iout_stated_current, chip->nu1619_chg_status.fastchg_current_limit);
		chip->nu1619_chg_status.iout_stated_current = chip->nu1619_chg_status.fastchg_current_limit;
	}
}
#endif

static void nu1619_fastcharge_skewing_proc_40w(struct oplus_nu1619_ic *chip, bool is_init)
{
	static int skewing_proc_delay = 0;

	if (is_init) {
		skewing_proc_delay = 0;
		return;
	}

	chg_err("<~WPC~> ~~~~~~~~40W fastcharg skewing proc(delay %d)~~~~~~~~\n", skewing_proc_delay);

	if(skewing_proc_delay > 0) {
		skewing_proc_delay--;
		return;
	}

	if (chip->nu1619_chg_status.iout <= fasctchg_current[FASTCHARGE_LEVEL_3]) {
		chg_err("<~WPC~> iout = %dmA <= %d\n", chip->nu1619_chg_status.iout, fasctchg_current[FASTCHARGE_LEVEL_3]);
		chip->nu1619_chg_status.wpc_ffc_charge = false;
		nu1619_ready_to_switch_to_charger(chip, false);
	} else {
		chip->nu1619_chg_status.iout_stated_current = chip->nu1619_chg_status.iout;
		nu1619_set_rx_charge_voltage(chip, chip->nu1619_chg_status.vout);
		chg_err("<~WPC~> set Vout: %d set Iout: %d\n", chip->nu1619_chg_status.charge_voltage, chip->nu1619_chg_status.iout_stated_current);
		skewing_proc_delay = 2;
	}

	if (chip->nu1619_chg_status.iout_stated_current > chip->nu1619_chg_status.fastchg_current_limit) {
		chg_err("<~WPC~> charge_current[%d] > charge_current_limit[%d]\n",
				chip->nu1619_chg_status.iout_stated_current, chip->nu1619_chg_status.fastchg_current_limit);
		chip->nu1619_chg_status.iout_stated_current = chip->nu1619_chg_status.fastchg_current_limit;
	}
}

static void nu1619_TX_message_process(struct oplus_nu1619_ic *chip)
{
	chip->nu1619_chg_status.send_msg_timer++;
	if (chip->nu1619_chg_status.send_msg_timer > 2) {
		chip->nu1619_chg_status.send_msg_timer = 0;

		if (chip->nu1619_chg_status.adapter_type == ADAPTER_TYPE_UNKNOW) {
			nu1619_set_tx_charger_dect(chip);
		} else if (chip->nu1619_chg_status.send_message == P9221_CMD_INTO_FASTCHAGE) {
			nu1619_set_tx_charger_fastcharge(chip);
		} else if (chip->nu1619_chg_status.send_message == P9221_CMD_SET_CEP_TIMEOUT) {
			if (chip->nu1619_chg_status.send_msg_cnt > 0) {
				chip->nu1619_chg_status.send_msg_cnt--;
				nu1619_set_tx_cep_timeout(chip);
			} else {
				chip->nu1619_chg_status.send_message = P9221_CMD_NULL;
			}
		} else if (chip->nu1619_chg_status.send_message == P9221_CMD_SET_PWM_PULSE) {
			if (chip->nu1619_chg_status.send_msg_cnt > 0) {
				chip->nu1619_chg_status.send_msg_cnt--;
				nu1619_set_tx_fan_pwm_pulse(chip);
			} else {
				chip->nu1619_chg_status.send_message = P9221_CMD_NULL;
			}
		} else if (chip->nu1619_chg_status.send_message == P9221_CMD_SET_LED_BRIGHTNESS) {
			if (chip->nu1619_chg_status.send_msg_cnt > 0) {
				chip->nu1619_chg_status.send_msg_cnt--;
				nu1619_set_tx_led_brightness(chip);
			} else {
				chip->nu1619_chg_status.send_message = P9221_CMD_NULL;
			}
		}
	}
}

static int nu1619_resume_chargepump_fastchg(struct oplus_nu1619_ic *chip)
{
	if ((chip->nu1619_chg_status.wpc_reach_4370mv == false)
		&& (g_oplus_chip->batt_volt <= 4370)
		&& (g_oplus_chip->temperature < 390)
		&& g_oplus_chip->temperature > g_oplus_chip->limits.little_cool_bat_decidegc) {
		chip->nu1619_chg_status.iout_stated_current = fasctchg_current[chip->nu1619_chg_status.fastcharge_level];
		chg_err("<~WPC~> resume chargepump IOUT: %d\n", chip->nu1619_chg_status.iout_stated_current);
		return 1;
	} else {
		return 0;
	}
}

static int oplus_wpc_chg_parse_chg_dt(struct oplus_nu1619_ic *chip)
{
	int rc;
	struct device_node *node = chip->dev->of_node;

	if (!node) {
		chg_err("device tree info. missing\n");
		return -EINVAL;
	}

	rc = of_property_read_u32(node, "qcom,iout_ma",
			&chip->nu1619_chg_status.wpc_chg_param.iout_ma);
	if (rc) {
		chip->nu1619_chg_status.wpc_chg_param.iout_ma = 1000;
	}
	chg_err("iout_ma[%d]\n", chip->nu1619_chg_status.wpc_chg_param.iout_ma);

	rc = of_property_read_u32(node, "qcom,bpp_input_ma",
			&chip->nu1619_chg_status.wpc_chg_param.bpp_input_ma);
	if (rc) {
		chip->nu1619_chg_status.wpc_chg_param.bpp_input_ma = 750;
	}
	chg_err("bpp_input_ma[%d]\n", chip->nu1619_chg_status.wpc_chg_param.bpp_input_ma);

	rc = of_property_read_u32(node, "qcom,epp_input_ma",
			&chip->nu1619_chg_status.wpc_chg_param.epp_input_ma);
	if (rc) {
		chip->nu1619_chg_status.wpc_chg_param.epp_input_ma = 800;
	}
	chg_err("epp_input_ma[%d]\n", chip->nu1619_chg_status.wpc_chg_param.epp_input_ma);

	rc = of_property_read_u32(node, "qcom,epp_temp_warm_input_ma",
			&chip->nu1619_chg_status.wpc_chg_param.epp_temp_warm_input_ma);
	if (rc) {
		chip->nu1619_chg_status.wpc_chg_param.epp_temp_warm_input_ma = 500;
	}
	chg_err("epp_input_ma[%d]\n", chip->nu1619_chg_status.wpc_chg_param.epp_temp_warm_input_ma);

	rc = of_property_read_u32(node, "qcom,epp_input_step_ma",
			&chip->nu1619_chg_status.wpc_chg_param.epp_input_step_ma);
	if (rc) {
		chip->nu1619_chg_status.wpc_chg_param.epp_input_step_ma = 100;
	}
	chg_err("epp_input_step_ma[%d]\n", chip->nu1619_chg_status.wpc_chg_param.epp_input_step_ma);

	rc = of_property_read_u32(node, "qcom,vooc_input_ma",
			&chip->nu1619_chg_status.wpc_chg_param.vooc_input_ma);
	if (rc) {
		chip->nu1619_chg_status.wpc_chg_param.vooc_input_ma = 1000;
	}
	chg_err("vooc_input_ma[%d]\n", chip->nu1619_chg_status.wpc_chg_param.vooc_input_ma);

	rc = of_property_read_u32(node, "qcom,vooc_iout_ma",
			&chip->nu1619_chg_status.wpc_chg_param.vooc_iout_ma);
	if (rc) {
		chip->nu1619_chg_status.wpc_chg_param.vooc_iout_ma = 1000;
	}
	chg_err("vooc_iout_ma[%d]\n", chip->nu1619_chg_status.wpc_chg_param.vooc_iout_ma);

	rc = of_property_read_u32(node, "qcom,svooc_input_ma",
			&chip->nu1619_chg_status.wpc_chg_param.svooc_input_ma);
	if (rc) {
		chip->nu1619_chg_status.wpc_chg_param.svooc_input_ma = 1000;
	}
	chg_err("svooc_input_ma[%d]\n", chip->nu1619_chg_status.wpc_chg_param.svooc_input_ma);

	rc = of_property_read_u32(node, "qcom,svooc_65w_iout_ma",
			&chip->nu1619_chg_status.wpc_chg_param.svooc_65w_iout_ma);
	if (rc) {
		chip->nu1619_chg_status.wpc_chg_param.svooc_65w_iout_ma = 2000;
	}
	chg_err("svooc_65w_iout_ma[%d]\n", chip->nu1619_chg_status.wpc_chg_param.svooc_65w_iout_ma);

	rc = of_property_read_u32(node, "qcom,svooc_50w_iout_ma",
			&chip->nu1619_chg_status.wpc_chg_param.svooc_50w_iout_ma);
	if (rc) {
		chip->nu1619_chg_status.wpc_chg_param.svooc_50w_iout_ma = 1750;
	}
	chg_err("svooc_50w_iout_ma[%d]\n", chip->nu1619_chg_status.wpc_chg_param.svooc_50w_iout_ma);

	rc = of_property_read_u32(node, "qcom,bpp_temp_cold_fastchg_ma",
			&chip->nu1619_chg_status.wpc_chg_param.bpp_temp_cold_fastchg_ma);
	if (rc) {
		chip->nu1619_chg_status.wpc_chg_param.bpp_temp_cold_fastchg_ma = 360;
	}
	chg_err("temp_cold_fastchg_ma[%d]\n", chip->nu1619_chg_status.wpc_chg_param.bpp_temp_cold_fastchg_ma);

	rc = of_property_read_u32(node, "qcom,vooc_temp_little_cold_fastchg_ma",
			&chip->nu1619_chg_status.wpc_chg_param.vooc_temp_little_cold_fastchg_ma);
	if (rc) {
		chip->nu1619_chg_status.wpc_chg_param.vooc_temp_little_cold_fastchg_ma = 1200;
	}
	chg_err("vooc_temp_little_cold_fastchg_ma[%d]\n", chip->nu1619_chg_status.wpc_chg_param.vooc_temp_little_cold_fastchg_ma);

	rc = of_property_read_u32(node, "qcom,svooc_temp_little_cold_iout_ma",
			&chip->nu1619_chg_status.wpc_chg_param.svooc_temp_little_cold_iout_ma);
	if (rc) {
		chip->nu1619_chg_status.wpc_chg_param.svooc_temp_little_cold_iout_ma = 650;
	}
	chg_err("svooc_temp_little_cold_iout_ma[%d]\n", chip->nu1619_chg_status.wpc_chg_param.svooc_temp_little_cold_iout_ma);

	rc = of_property_read_u32(node, "qcom,svooc_temp_little_cold_fastchg_ma",
			&chip->nu1619_chg_status.wpc_chg_param.svooc_temp_little_cold_fastchg_ma);
	if (rc) {
		chip->nu1619_chg_status.wpc_chg_param.svooc_temp_little_cold_fastchg_ma = 1200;
	}
	chg_err("svooc_temp_little_cold_fastchg_ma[%d]\n", chip->nu1619_chg_status.wpc_chg_param.svooc_temp_little_cold_fastchg_ma);

	rc = of_property_read_u32(node, "qcom,bpp_temp_little_cold_fastchg_ma",
			&chip->nu1619_chg_status.wpc_chg_param.bpp_temp_little_cold_fastchg_ma);
	if (rc) {
		chip->nu1619_chg_status.wpc_chg_param.bpp_temp_little_cold_fastchg_ma = 600;
	}
	chg_err("bpp_temp_little_cold_fastchg_ma[%d]\n", chip->nu1619_chg_status.wpc_chg_param.bpp_temp_little_cold_fastchg_ma);

	rc = of_property_read_u32(node, "qcom,epp_temp_little_cold_fastchg_ma",
			&chip->nu1619_chg_status.wpc_chg_param.epp_temp_little_cold_fastchg_ma);
	if (rc) {
		chip->nu1619_chg_status.wpc_chg_param.epp_temp_little_cold_fastchg_ma = 1400;
	}
	chg_err("epp_temp_little_cold_fastchg_ma[%d]\n", chip->nu1619_chg_status.wpc_chg_param.epp_temp_little_cold_fastchg_ma);

	rc = of_property_read_u32(node, "qcom,vooc_temp_cool_fastchg_ma",
			&chip->nu1619_chg_status.wpc_chg_param.vooc_temp_cool_fastchg_ma);
	if (rc) {
		chip->nu1619_chg_status.wpc_chg_param.vooc_temp_cool_fastchg_ma = 1200;
	}
	chg_err("vooc_temp_cool_fastchg_ma[%d]\n", chip->nu1619_chg_status.wpc_chg_param.vooc_temp_cool_fastchg_ma);

	rc = of_property_read_u32(node, "qcom,svooc_temp_cool_iout_ma",
			&chip->nu1619_chg_status.wpc_chg_param.svooc_temp_cool_iout_ma);
	if (rc) {
		chip->nu1619_chg_status.wpc_chg_param.svooc_temp_cool_iout_ma = 650;
	}
	chg_err("svooc_temp_cool_iout_ma[%d]\n", chip->nu1619_chg_status.wpc_chg_param.svooc_temp_cool_iout_ma);

	rc = of_property_read_u32(node, "qcom,svooc_temp_cool_fastchg_ma",
			&chip->nu1619_chg_status.wpc_chg_param.svooc_temp_cool_fastchg_ma);
	if (rc) {
		chip->nu1619_chg_status.wpc_chg_param.svooc_temp_cool_fastchg_ma = 1200;
	}
	chg_err("svooc_temp_cool_fastchg_ma[%d]\n", chip->nu1619_chg_status.wpc_chg_param.svooc_temp_cool_fastchg_ma);

	rc = of_property_read_u32(node, "qcom,bpp_temp_cool_fastchg_ma",
			&chip->nu1619_chg_status.wpc_chg_param.bpp_temp_cool_fastchg_ma);
	if (rc) {
		chip->nu1619_chg_status.wpc_chg_param.bpp_temp_cool_fastchg_ma = 600;
	}
	chg_err("bpp_temp_cool_fastchg_ma[%d]\n", chip->nu1619_chg_status.wpc_chg_param.bpp_temp_cool_fastchg_ma);

	rc = of_property_read_u32(node, "qcom,epp_temp_cool_fastchg_ma",
			&chip->nu1619_chg_status.wpc_chg_param.epp_temp_cool_fastchg_ma);
	if (rc) {
		chip->nu1619_chg_status.wpc_chg_param.epp_temp_cool_fastchg_ma = 1400;
	}
	chg_err("epp_temp_cool_fastchg_ma[%d]\n", chip->nu1619_chg_status.wpc_chg_param.epp_temp_cool_fastchg_ma);

	rc = of_property_read_u32(node, "qcom,vooc_temp_little_cool_fastchg_ma",
			&chip->nu1619_chg_status.wpc_chg_param.vooc_temp_little_cool_fastchg_ma);
	if (rc) {
		chip->nu1619_chg_status.wpc_chg_param.vooc_temp_little_cool_fastchg_ma = 1200;
	}
	chg_err("vooc_temp_little_cool_fastchg_ma[%d]\n", chip->nu1619_chg_status.wpc_chg_param.vooc_temp_little_cool_fastchg_ma);

	rc = of_property_read_u32(node, "qcom,svooc_temp_little_cool_fastchg_ma",
			&chip->nu1619_chg_status.wpc_chg_param.svooc_temp_little_cool_fastchg_ma);
	if (rc) {
		chip->nu1619_chg_status.wpc_chg_param.svooc_temp_little_cool_fastchg_ma = 1200;
	}
	chg_err("svooc_temp_little_cool_fastchg_ma[%d]\n", chip->nu1619_chg_status.wpc_chg_param.svooc_temp_little_cool_fastchg_ma);

	rc = of_property_read_u32(node, "qcom,bpp_temp_little_cool_fastchg_ma",
			&chip->nu1619_chg_status.wpc_chg_param.bpp_temp_little_cool_fastchg_ma);
	if (rc) {
		chip->nu1619_chg_status.wpc_chg_param.bpp_temp_little_cool_fastchg_ma = 600;
	}
	chg_err("bpp_temp_little_cool_fastchg_ma[%d]\n", chip->nu1619_chg_status.wpc_chg_param.bpp_temp_little_cool_fastchg_ma);

	rc = of_property_read_u32(node, "qcom,epp_temp_little_cool_fastchg_ma",
			&chip->nu1619_chg_status.wpc_chg_param.epp_temp_little_cool_fastchg_ma);
	if (rc) {
		chip->nu1619_chg_status.wpc_chg_param.epp_temp_little_cool_fastchg_ma = 1400;
	}
	chg_err("epp_temp_little_cool_fastchg_ma[%d]\n", chip->nu1619_chg_status.wpc_chg_param.epp_temp_little_cool_fastchg_ma);

	rc = of_property_read_u32(node, "qcom,vooc_temp_normal_fastchg_ma",
			&chip->nu1619_chg_status.wpc_chg_param.vooc_temp_normal_fastchg_ma);
	if (rc) {
		chip->nu1619_chg_status.wpc_chg_param.vooc_temp_normal_fastchg_ma = 1200;
	}
	chg_err("vooc_temp_normal_fastchg_ma[%d]\n", chip->nu1619_chg_status.wpc_chg_param.vooc_temp_normal_fastchg_ma);

	rc = of_property_read_u32(node, "qcom,svooc_temp_normal_fastchg_ma",
			&chip->nu1619_chg_status.wpc_chg_param.svooc_temp_normal_fastchg_ma);
	if (rc) {
		chip->nu1619_chg_status.wpc_chg_param.svooc_temp_normal_fastchg_ma = 1200;
	}
	chg_err("svooc_temp_normal_fastchg_ma[%d]\n", chip->nu1619_chg_status.wpc_chg_param.svooc_temp_normal_fastchg_ma);

	rc = of_property_read_u32(node, "qcom,bpp_temp_normal_fastchg_ma",
			&chip->nu1619_chg_status.wpc_chg_param.bpp_temp_normal_fastchg_ma);
	if (rc) {
		chip->nu1619_chg_status.wpc_chg_param.bpp_temp_normal_fastchg_ma = 1200;
	}
	chg_err("bpp_temp_normal_fastchg_ma[%d]\n", chip->nu1619_chg_status.wpc_chg_param.bpp_temp_normal_fastchg_ma);

	rc = of_property_read_u32(node, "qcom,epp_temp_normal_fastchg_ma",
			&chip->nu1619_chg_status.wpc_chg_param.epp_temp_normal_fastchg_ma);
	if (rc) {
		chip->nu1619_chg_status.wpc_chg_param.epp_temp_normal_fastchg_ma = 1400;
	}
	chg_err("epp_temp_normal_fastchg_ma[%d]\n", chip->nu1619_chg_status.wpc_chg_param.epp_temp_normal_fastchg_ma);

	rc = of_property_read_u32(node, "qcom,vooc_temp_warm_fastchg_ma",
			&chip->nu1619_chg_status.wpc_chg_param.vooc_temp_warm_fastchg_ma);
	if (rc) {
		chip->nu1619_chg_status.wpc_chg_param.vooc_temp_warm_fastchg_ma = 1200;
	}
	chg_err("vooc_temp_warm_fastchg_ma[%d]\n", chip->nu1619_chg_status.wpc_chg_param.vooc_temp_warm_fastchg_ma);

	rc = of_property_read_u32(node, "qcom,svooc_temp_warm_fastchg_ma",
			&chip->nu1619_chg_status.wpc_chg_param.svooc_temp_warm_fastchg_ma);
	if (rc) {
		chip->nu1619_chg_status.wpc_chg_param.svooc_temp_warm_fastchg_ma = 1200;
	}
	chg_err("svooc_temp_warm_fastchg_ma[%d]\n", chip->nu1619_chg_status.wpc_chg_param.svooc_temp_warm_fastchg_ma);

	rc = of_property_read_u32(node, "qcom,bpp_temp_warm_fastchg_ma",
			&chip->nu1619_chg_status.wpc_chg_param.bpp_temp_warm_fastchg_ma);
	if (rc) {
		chip->nu1619_chg_status.wpc_chg_param.bpp_temp_warm_fastchg_ma = 600;
	}
	chg_err("bpp_temp_warm_fastchg_ma[%d]\n", chip->nu1619_chg_status.wpc_chg_param.bpp_temp_warm_fastchg_ma);

	rc = of_property_read_u32(node, "qcom,epp_temp_warm_fastchg_ma",
			&chip->nu1619_chg_status.wpc_chg_param.epp_temp_warm_fastchg_ma);
	if (rc) {
		chip->nu1619_chg_status.wpc_chg_param.epp_temp_warm_fastchg_ma = 1400;
	}
	chg_err("epp_temp_warm_fastchg_ma[%d]\n", chip->nu1619_chg_status.wpc_chg_param.epp_temp_warm_fastchg_ma);

	rc = of_property_read_u32(node, "qcom,curr_cp_to_charger",
			&chip->nu1619_chg_status.wpc_chg_param.curr_cp_to_charger);
	if (rc) {
		chip->nu1619_chg_status.wpc_chg_param.curr_cp_to_charger = 400;
	}
	chg_err("curr_cp_to_charger[%d]\n", chip->nu1619_chg_status.wpc_chg_param.curr_cp_to_charger);

	return 0;
}

static bool oplus_wpc_get_fastchg_allow(struct oplus_nu1619_ic *chip)
{
	int temp = 0;
	int cap = 0;

	temp = oplus_chg_get_chg_temperature();
	cap = oplus_chg_get_ui_soc();

	if(chip->nu1619_chg_status.adapter_type != ADAPTER_TYPE_VOOC
		&& chip->nu1619_chg_status.adapter_type != ADAPTER_TYPE_SVOOC
		&& chip->nu1619_chg_status.adapter_type != ADAPTER_TYPE_SVOOC_50W) {
		chip->nu1619_chg_status.fastchg_allow = false;
		return chip->nu1619_chg_status.fastchg_allow;
	}
/*
	if(cap > 90){
		chip->nu1619_chg_status.fastchg_allow = false;
		return chip->nu1619_chg_status.fastchg_allow;
	}
*/
	if(temp < 0 || temp > 450) {
		chip->nu1619_chg_status.fastchg_allow = false;
		return chip->nu1619_chg_status.fastchg_allow;
	}

	chip->nu1619_chg_status.fastchg_allow = true;
	return chip->nu1619_chg_status.fastchg_allow;
}

int nu1619_charge_set_max_current_by_tbatt(struct oplus_nu1619_ic *chip)
{
	int charging_current = 512;
	static int pre_tbatt = BATTERY_STATUS__NORMAL;

	int now_tbatt = oplus_chg_get_tbatt_status();
	chg_err("<~WPC~> now_tbatt %d, charge_status[%d]\n", now_tbatt, chip->nu1619_chg_status.charge_status);

	switch (now_tbatt) {
	case BATTERY_STATUS__NORMAL:
		chip->nu1619_chg_status.fastchg_current_limit = chip->nu1619_chg_status.wpc_chg_param.svooc_65w_iout_ma;
		/*chip->nu1619_chg_status.epp_current_limit = chip->nu1619_chg_status.wpc_chg_param.epp_input_ma;*/
		chip->nu1619_chg_status.BPP_fastchg_current_ma = chip->nu1619_chg_status.wpc_chg_param.bpp_temp_normal_fastchg_ma;
		switch (chip->nu1619_chg_status.adapter_type) {
		case ADAPTER_TYPE_VOOC:
			chip->nu1619_chg_status.fastchg_current_limit = chip->nu1619_chg_status.wpc_chg_param.vooc_temp_normal_fastchg_ma;
			charging_current = chip->nu1619_chg_status.wpc_chg_param.vooc_temp_normal_fastchg_ma;
			break;
		case ADAPTER_TYPE_SVOOC:
			charging_current = chip->nu1619_chg_status.wpc_chg_param.svooc_65w_iout_ma;
			break;
		case ADAPTER_TYPE_SVOOC_50W:
			charging_current = chip->nu1619_chg_status.wpc_chg_param.svooc_50w_iout_ma;
			break;
		case ADAPTER_TYPE_EPP:
			charging_current = chip->nu1619_chg_status.wpc_chg_param.epp_temp_normal_fastchg_ma;
			break;
		default:
			charging_current = chip->nu1619_chg_status.wpc_chg_param.bpp_temp_normal_fastchg_ma;
			break;
		}
		break;
	case BATTERY_STATUS__COLD_TEMP:
		if (chip->nu1619_chg_status.charge_status != WPC_CHG_STATUS_DEFAULT
				&& chip->nu1619_chg_status.adapter_type != ADAPTER_TYPE_EPP) {
			chg_err("<~WPC~> self reset: because of cold temp\n");
			nu1619_self_reset(chip, true);
		}
		chip->nu1619_chg_status.fastchg_current_limit = 0;
		/*chip->nu1619_chg_status.epp_current_limit = chip->nu1619_chg_status.wpc_chg_param.epp_input_ma;*/
		chip->nu1619_chg_status.BPP_fastchg_current_ma = chip->nu1619_chg_status.wpc_chg_param.bpp_temp_cold_fastchg_ma;
		charging_current = chip->nu1619_chg_status.wpc_chg_param.bpp_temp_cold_fastchg_ma;
		break;
	case BATTERY_STATUS__LITTLE_COLD_TEMP:
		/*chip->nu1619_chg_status.epp_current_limit = chip->nu1619_chg_status.wpc_chg_param.epp_input_ma;*/
		chip->nu1619_chg_status.BPP_fastchg_current_ma = chip->nu1619_chg_status.wpc_chg_param.bpp_temp_little_cold_fastchg_ma;
		switch (chip->nu1619_chg_status.adapter_type) {
		case ADAPTER_TYPE_VOOC:
			chip->nu1619_chg_status.fastchg_current_limit = chip->nu1619_chg_status.wpc_chg_param.vooc_temp_little_cold_fastchg_ma;
			charging_current = chip->nu1619_chg_status.wpc_chg_param.vooc_temp_little_cold_fastchg_ma;
			break;
		case ADAPTER_TYPE_SVOOC:
		case ADAPTER_TYPE_SVOOC_50W:
			chip->nu1619_chg_status.fastchg_current_limit = chip->nu1619_chg_status.wpc_chg_param.svooc_temp_little_cold_fastchg_ma;
			charging_current = chip->nu1619_chg_status.wpc_chg_param.svooc_temp_little_cold_fastchg_ma;
			break;
		case ADAPTER_TYPE_EPP:
			charging_current = chip->nu1619_chg_status.wpc_chg_param.epp_temp_little_cold_fastchg_ma;
			break;
		default:
			charging_current = chip->nu1619_chg_status.wpc_chg_param.bpp_temp_little_cold_fastchg_ma;
			break;
		}
		break;
	case BATTERY_STATUS__COOL_TEMP:
		/*chip->nu1619_chg_status.epp_current_limit = chip->nu1619_chg_status.wpc_chg_param.epp_input_ma;*/
		chip->nu1619_chg_status.BPP_fastchg_current_ma = chip->nu1619_chg_status.wpc_chg_param.bpp_temp_cool_fastchg_ma;
		switch (chip->nu1619_chg_status.adapter_type) {
		case ADAPTER_TYPE_VOOC:
			chip->nu1619_chg_status.fastchg_current_limit = chip->nu1619_chg_status.wpc_chg_param.vooc_temp_cool_fastchg_ma;
			charging_current = chip->nu1619_chg_status.wpc_chg_param.vooc_temp_cool_fastchg_ma;
			break;
		case ADAPTER_TYPE_SVOOC:
		case ADAPTER_TYPE_SVOOC_50W:
			chip->nu1619_chg_status.fastchg_current_limit = chip->nu1619_chg_status.wpc_chg_param.svooc_temp_cool_fastchg_ma;
			charging_current = chip->nu1619_chg_status.wpc_chg_param.svooc_temp_cool_fastchg_ma;
			break;
		case ADAPTER_TYPE_EPP:
			charging_current = chip->nu1619_chg_status.wpc_chg_param.epp_temp_cool_fastchg_ma;
			break;
		default:
			charging_current = chip->nu1619_chg_status.wpc_chg_param.bpp_temp_cool_fastchg_ma;
			break;
		}
		break;
	case BATTERY_STATUS__LITTLE_COOL_TEMP:
		/*chip->nu1619_chg_status.epp_current_limit = chip->nu1619_chg_status.wpc_chg_param.epp_input_ma;*/
		chip->nu1619_chg_status.BPP_fastchg_current_ma = chip->nu1619_chg_status.wpc_chg_param.bpp_temp_little_cool_fastchg_ma;
		switch (chip->nu1619_chg_status.adapter_type) {
		case ADAPTER_TYPE_VOOC:
			chip->nu1619_chg_status.fastchg_current_limit = chip->nu1619_chg_status.wpc_chg_param.vooc_temp_little_cool_fastchg_ma;
			charging_current = chip->nu1619_chg_status.wpc_chg_param.vooc_temp_little_cool_fastchg_ma;
			break;
		case ADAPTER_TYPE_SVOOC:
			chip->nu1619_chg_status.fastchg_current_limit = chip->nu1619_chg_status.wpc_chg_param.svooc_65w_iout_ma;
			charging_current = chip->nu1619_chg_status.wpc_chg_param.svooc_temp_little_cool_fastchg_ma;
			break;
		case ADAPTER_TYPE_SVOOC_50W:
			chip->nu1619_chg_status.fastchg_current_limit = chip->nu1619_chg_status.wpc_chg_param.svooc_50w_iout_ma;
			charging_current = chip->nu1619_chg_status.wpc_chg_param.svooc_temp_little_cool_fastchg_ma;
			break;
		case ADAPTER_TYPE_EPP:
			charging_current = chip->nu1619_chg_status.wpc_chg_param.epp_temp_little_cool_fastchg_ma;
			break;
		default:
			charging_current = chip->nu1619_chg_status.wpc_chg_param.bpp_temp_little_cool_fastchg_ma;
			break;
		}
		break;
	case BATTERY_STATUS__WARM_TEMP:
		if (chip->nu1619_chg_status.charge_status != WPC_CHG_STATUS_DEFAULT
				&& chip->nu1619_chg_status.adapter_type != ADAPTER_TYPE_EPP) {
			chg_err("<~WPC~> self reset: because of warm temp\n");
			nu1619_self_reset(chip, true);
		}
		chip->nu1619_chg_status.fastchg_current_limit = 0;
		/*chip->nu1619_chg_status.epp_current_limit = 0;*/
		chip->nu1619_chg_status.BPP_fastchg_current_ma = chip->nu1619_chg_status.wpc_chg_param.bpp_temp_warm_fastchg_ma;
		charging_current = chip->nu1619_chg_status.wpc_chg_param.bpp_temp_warm_fastchg_ma;
		break;
	case BATTERY_STATUS__REMOVED:
	case BATTERY_STATUS__LOW_TEMP:
	case BATTERY_STATUS__HIGH_TEMP:
		if (chip->nu1619_chg_status.charge_status != WPC_CHG_STATUS_DEFAULT
				&& chip->nu1619_chg_status.adapter_type != ADAPTER_TYPE_EPP) {
			chg_err("<~WPC~> self reset: because of low/high temp\n");
			nu1619_self_reset(chip, true);
		}
		chip->nu1619_chg_status.fastchg_current_limit = 0;
		/*chip->nu1619_chg_status.epp_current_limit = 0;*/
		chip->nu1619_chg_status.BPP_fastchg_current_ma = 0;
		charging_current = 0;
		break;
	default:
		break;
	}
	if(pre_tbatt != now_tbatt) {
		if(pre_tbatt == BATTERY_STATUS__REMOVED
			|| pre_tbatt == BATTERY_STATUS__LOW_TEMP
			|| pre_tbatt == BATTERY_STATUS__HIGH_TEMP
			|| pre_tbatt == BATTERY_STATUS__COLD_TEMP
			|| pre_tbatt == BATTERY_STATUS__WARM_TEMP) {
			if ((g_oplus_chip->temperature > g_oplus_chip->limits.little_cold_bat_decidegc)
				&& (g_oplus_chip->temperature < g_oplus_chip->limits.warm_bat_decidegc)) {
				chip->nu1619_chg_status.adapter_type = ADAPTER_TYPE_UNKNOW;
			}
		}
		pre_tbatt = now_tbatt;
	}
	if(chip->nu1619_chg_status.adapter_type != ADAPTER_TYPE_VOOC
		&& chip->nu1619_chg_status.adapter_type != ADAPTER_TYPE_SVOOC
		&& chip->nu1619_chg_status.adapter_type != ADAPTER_TYPE_SVOOC_50W)
		nu1619_set_rx_charge_current(chip, charging_current);
	return 0;
}

static int oplus_wpc_set_input_current(struct oplus_nu1619_ic *chip)
{
	int current_limit = 0;
	int input_current_threshold = 0;
	int now_tbatt = oplus_chg_get_tbatt_status();

	if (!chip) {
		chg_err("oplus_nu1619_ic is not ready\n");
		return -1;
	}

	switch (chip->nu1619_chg_status.adapter_type) {
	case ADAPTER_TYPE_VOOC:
		current_limit = chip->nu1619_chg_status.wpc_chg_param.vooc_input_ma;
		break;
	case ADAPTER_TYPE_SVOOC:
		current_limit = chip->nu1619_chg_status.wpc_chg_param.svooc_input_ma;
		break;
	case ADAPTER_TYPE_SVOOC_50W:
		current_limit = chip->nu1619_chg_status.wpc_chg_param.svooc_input_ma;
		break;
	case ADAPTER_TYPE_EPP:
		if (now_tbatt == BATTERY_STATUS__WARM_TEMP)
			input_current_threshold = chip->nu1619_chg_status.wpc_chg_param.epp_temp_warm_input_ma;
		else
			input_current_threshold = chip->nu1619_chg_status.wpc_chg_param.epp_input_ma;
		if (nu1619_get_special_ID(chip) != 0) {
			chip->nu1619_chg_status.epp_current_limit = WPC_CHARGE_CURRENT_EPP_SPEC;
		} else {
			if (chip->nu1619_chg_status.epp_current_limit < input_current_threshold) {
				chip->nu1619_chg_status.epp_current_limit +=
					chip->nu1619_chg_status.wpc_chg_param.epp_input_step_ma;
				chg_err("<~WPC~>EPP increase charge current to %dmA\n", chip->nu1619_chg_status.epp_current_limit);
			} else {
				chip->nu1619_chg_status.epp_current_limit = input_current_threshold;
			}
		}
		current_limit = chip->nu1619_chg_status.epp_current_limit;
		break;
	default:
		if (chip->nu1619_chg_status.BPP_current_step_cnt > 0) {
			chip->nu1619_chg_status.BPP_current_step_cnt--;
			if (chip->nu1619_chg_status.BPP_current_step_cnt == 0) {
				if (chip->nu1619_chg_status.BPP_current_limit < 500) {
					chg_err("<~WPC~> set BPP current limit 500ma\n");
					chip->nu1619_chg_status.BPP_current_limit = 500;
					mp2650_set_vindpm_vol(4900);
					chip->nu1619_chg_status.BPP_current_step_cnt = BPP_CURRENT_INCREASE_TIME;
				} else if (chip->nu1619_chg_status.BPP_current_limit < 700) {
					chg_err("<~WPC~> set BPP current limit 700ma\n");
					chip->nu1619_chg_status.BPP_current_limit = 700;
					chip->nu1619_chg_status.BPP_current_step_cnt = BPP_CURRENT_INCREASE_TIME;
				} else if (chip->nu1619_chg_status.BPP_current_limit < chip->nu1619_chg_status.wpc_chg_param.bpp_input_ma) {
					chg_err("<~WPC~> set BPP current limit 750ma\n");
					chip->nu1619_chg_status.BPP_current_limit = chip->nu1619_chg_status.wpc_chg_param.bpp_input_ma;
				}
			}
		}
		if(chip->nu1619_chg_status.BPP_current_limit < 300)
			chip->nu1619_chg_status.BPP_current_limit = 300;
		current_limit = chip->nu1619_chg_status.BPP_current_limit;
		break;
	}
	if(g_oplus_chip->batt_full == true && g_oplus_chip->in_rechging == false) {
		chip->nu1619_chg_status.wpc_chg_param.pre_input_ma = 0;
		current_limit = 200;
	}
	if(chip->nu1619_chg_status.fastchg_ing == true) {
		return 0;
	}
	if(current_limit != chip->nu1619_chg_status.wpc_chg_param.pre_input_ma) {
		chg_err("<~WPC~> current_limit %d, pre_input_ma[%d], adapter_type[%d]\n",
			current_limit,
		chip->nu1619_chg_status.wpc_chg_param.pre_input_ma,
		chip->nu1619_chg_status.adapter_type);
		chip->nu1619_chg_status.wpc_chg_param.pre_input_ma = current_limit;
		oplus_chg_set_input_current_without_aicl(current_limit);
	}
	return 0;
}

int nu1619_charge_set_max_current_by_adapter_power(struct oplus_nu1619_ic *chip)
{
	switch (chip->nu1619_chg_status.adapter_power) {
	case ADAPTER_POWER_65W:
		if(nu1619_test_charging_status() == 4) {
			if(chip->nu1619_chg_status.fastchg_current_limit > fasctchg_current[FASTCHARGE_LEVEL_2])
				chip->nu1619_chg_status.fastchg_current_limit = fasctchg_current[FASTCHARGE_LEVEL_2];
		}
		break;
	case ADAPTER_POWER_50W:
		if(chip->nu1619_chg_status.fastchg_current_limit > chip->nu1619_chg_status.wpc_chg_param.svooc_50w_iout_ma)
			chip->nu1619_chg_status.fastchg_current_limit = chip->nu1619_chg_status.wpc_chg_param.svooc_50w_iout_ma;
		break;
	case ADAPTER_POWER_30W:
	case ADAPTER_POWER_20W:
		if(chip->nu1619_chg_status.fastchg_current_limit > chip->nu1619_chg_status.wpc_chg_param.vooc_temp_normal_fastchg_ma)
			chip->nu1619_chg_status.fastchg_current_limit = chip->nu1619_chg_status.wpc_chg_param.vooc_temp_normal_fastchg_ma;
		break;
	default:
		break;
	}
	return 0;
}

int nu1619_charge_set_max_current_by_led(struct oplus_nu1619_ic *chip)
{
	return 0;
}

int nu1619_charge_set_max_current_by_cool_down(struct oplus_nu1619_ic *chip)
{
	oplus_chg_get_cool_down_status();

	return 0;
}

static int nu1619_charge_status_process(struct oplus_nu1619_ic *chip)
{
	static int work_delay = 0;
	static int cep_nonzero_cnt = 0;
	static int cep_zero_cnt = 0;
	static int self_reset_cnt = 0;
	static int cep_doublecheck_cnt = 0;
	static int shake_temp = 0;
	int temp_value = 0;
	int iout_max_value;
	int iout_min_value;
#ifndef FASTCHG_TEST_BY_TIME
	int work_freq = 0;
	int freq_thr = 0;
	int ret = 0;
#endif
	int vout_value = 0;
	int batt_volt = 0;
	int i = 0;

#ifndef FASTCHG_TEST_BY_TIME
	if ((g_oplus_chip->temperature < g_oplus_chip->limits.cold_bat_decidegc) || (g_oplus_chip->temperature > g_oplus_chip->limits.hot_bat_decidegc)) {
		chg_err("<~WPC~> The temperature is abnormal, stop charge!\n");
		if (chip->nu1619_chg_status.charge_current != WPC_CHARGE_CURRENT_ZERO) {
			nu1619_set_rx_charge_current(chip, WPC_CHARGE_CURRENT_ZERO);
		}

		return 0;
	}
#endif
	nu1619_print_CEP_flag(chip);

	if (work_delay > 0) {
		work_delay--;
		return 0;
	}
	if (chip->nu1619_chg_status.need_doublecheck_to_cp == true && chip->nu1619_chg_status.doublecheck_ok == false) {
		if (nu1619_get_CEP_flag(chip) == 0) {
			cep_doublecheck_cnt++;
			chg_err("<~WPC~> cep_doublecheck_cnt[%d]\n", cep_doublecheck_cnt);
			if (cep_doublecheck_cnt > 10) {
				cep_doublecheck_cnt = 0;
				chip->nu1619_chg_status.doublecheck_ok = true;
			}
		} else {
			cep_doublecheck_cnt = 0;
		}
	} else {
		cep_doublecheck_cnt = 0;
	}
	switch (chip->nu1619_chg_status.charge_status) {
	case WPC_CHG_STATUS_DEFAULT:
		if (chip->nu1619_chg_status.rx_runing_mode == RX_RUNNING_MODE_EPP) {
			chg_err("<~WPC~> Change to EPP charge\n");
			work_delay = 7;
			chip->nu1619_chg_status.charge_status = WPC_CHG_STATUS_READY_FOR_EPP;
			break;
		}
#ifdef FASTCHG_TEST_BY_TIME
		if (chip->nu1619_chg_status.adapter_type == ADAPTER_TYPE_SVOOC) {
			chg_err("<~WPC~>[-TEST-] Go to Fastchg test!\n");
			chip->nu1619_chg_status.fastchg_current_limit = 2000;
			chip->nu1619_chg_status.charge_status = WPC_CHG_STATUS_READY_FOR_FASTCHG;
		}
#else
		/*deviation check*/
		if ((chip->nu1619_chg_status.adapter_type == ADAPTER_TYPE_VOOC
				|| chip->nu1619_chg_status.adapter_type == ADAPTER_TYPE_SVOOC)
				&& !chip->nu1619_chg_status.deviation_check_done) {
			if (g_oplus_chip->batt_volt_max > 4100) {
				freq_thr = chip->nu1619_chg_status.freq_threshold + 2;
			} else if (g_oplus_chip->batt_volt_max > 3700) {
				freq_thr = chip->nu1619_chg_status.freq_threshold + (g_oplus_chip->batt_volt_max - 3700) / 200;
			} else {
				freq_thr = chip->nu1619_chg_status.freq_threshold;
			}
			ret = nu1619_get_work_freq(chip, &work_freq);
			if (ret != 0) {
				chg_err("<~WPC~> nu1619_get_work_freq error, return\n");
				return ret;
			}
			chg_err("<~WPC~> IDT TX Freq = %d, Freq_thr = %d\n", work_freq, freq_thr);
			if (work_freq > freq_thr) {
				chip->nu1619_chg_status.deviation_check_done = true;
				chip->nu1619_chg_status.is_deviation = false;
				chg_err("<~WPC~> deviation_check_done, phone location is correct\n");
			} else {
				chip->nu1619_chg_status.is_deviation = true;
				chg_err("<~WPC~> phone location is deviation, work_freq=%d, freq_thr=%d\n", work_freq, freq_thr);
			}
		}
		if ((chip->nu1619_chg_status.adapter_type == ADAPTER_TYPE_VOOC)
			|| (chip->nu1619_chg_status.adapter_type == ADAPTER_TYPE_SVOOC)) {
			if (chip->nu1619_chg_status.fastchg_allow == true) {
				chip->nu1619_chg_status.charge_status = WPC_CHG_STATUS_READY_FOR_FASTCHG;
				break;
			} else {
				chg_err("<~WPC~> temp < 0 or temp > 45 or  soc > 90\n");
			}
		}
#endif
		break;
	case WPC_CHG_STATUS_READY_FOR_EPP:
		chg_err("<~WPC~> ..........WPC_CHG_STATUS_READY_FOR_EPP..........\n");
		if (chip->nu1619_chg_status.epp_current_step == WPC_CHARGE_CURRENT_EPP_INIT) {
			mp2650_set_vindpm_vol(11500);
		}
		if (chip->nu1619_chg_status.epp_current_limit >= chip->nu1619_chg_status.wpc_chg_param.epp_input_ma
				|| (chip->nu1619_chg_status.epp_current_limit >= chip->nu1619_chg_status.wpc_chg_param.epp_temp_warm_input_ma
					&& oplus_chg_get_tbatt_status() == BATTERY_STATUS__WARM_TEMP)) {
			chg_err("<~WPC~> 2 turn to WPC_CHG_STATUS_EPP_WORKING\n");
			chip->nu1619_chg_status.charge_status = WPC_CHG_STATUS_EPP_WORKING;
		}
		break;
	case WPC_CHG_STATUS_EPP_WORKING:
		break;
	case WPC_CHG_STATUS_READY_FOR_FASTCHG:
		chg_err("<~WPC~> ..........WPC_CHG_STATUS_READY_FOR_FASTCHG..........\n");
		chip->nu1619_chg_status.fastchg_ing = true;
		mp2650_enable_charging();
		nu1619_set_rx_charge_current(chip, 1200);
		mp2650_input_current_limit_without_aicl(WPC_CHARGE_CURRENT_LIMIT_300MA);
		nu1619_set_rx_charge_voltage(chip, WPC_CHARGE_VOLTAGE_FASTCHG_INIT);
		chip->nu1619_chg_status.send_message = P9221_CMD_INTO_FASTCHAGE;
		nu1619_set_tx_charger_fastcharge(chip);
		nu1619_set_cp_ldo_5v_val(1);
		chip->nu1619_chg_status.send_msg_timer = 0;
		nu1619_begin_CEP_detect(chip);
		cep_nonzero_cnt = 0;
		chip->nu1619_chg_status.charge_status = WPC_CHG_STATUS_WAITING_FOR_TX_INTO_FASTCHG;
		break;
	case WPC_CHG_STATUS_WAITING_FOR_TX_INTO_FASTCHG:
		chg_err("<~WPC~> ..........WPC_CHG_STATUS_WAITING_FOR_TX_INTO_FASTCHG..........\n");
		if (chip->nu1619_chg_status.charge_type == WPC_CHARGE_TYPE_FAST) {
			if (nu1619_get_CEP_flag(chip) != 0) {
#ifndef FASTCHG_TEST_BY_TIME
				cep_nonzero_cnt++;
				if (cep_nonzero_cnt > 10) {
					cep_nonzero_cnt = 0;
					chip->nu1619_chg_status.charge_status = WPC_CHG_STATUS_CHARGER_FASTCHG_INIT;
					chip->nu1619_chg_status.skewing_info = true;
					if (chip->nu1619_chg_status.cep_info < to_cep_info(chip->nu1619_chg_status.CEP_value))
						chip->nu1619_chg_status.cep_info = to_cep_info(chip->nu1619_chg_status.CEP_value);
				}
#endif
				break;
			}
			nu1619_set_FOD_parameter(chip, 12);
			mp2650_set_vindpm_vol(8700);
			chip->nu1619_chg_status.idt_adc_test_result = false;
			chip->nu1619_chg_status.charge_status = WPC_CHG_STATUS_WAIT_IOUT_READY;
		}
		break;
	case WPC_CHG_STATUS_WAIT_IOUT_READY:
		chg_err("<~WPC~> ..........WPC_CHG_STATUS_WAIT_IOUT_READY..........\n");
		if (chip->nu1619_chg_status.idt_adc_test_enable) {
			if (chip->nu1619_chg_status.iout < 200) {
				chg_err("<~WPC~> idt_adc_test iout: %dmA < 200mA\n", chip->nu1619_chg_status.iout);
				chip->nu1619_chg_status.idt_adc_test_result = false;
				break;
			} else {
				chg_err("<~WPC~> idt_adc_test iout: %dmA >= 200mA\n", chip->nu1619_chg_status.iout);
				chip->nu1619_chg_status.idt_adc_test_result = true;
			}
		}
		nu1619_test_reset_record_seconds();
		if (chip->nu1619_chg_status.ftm_mode) {
			chip->nu1619_chg_status.charge_status = WPC_CHG_STATUS_READY_FOR_FTM;
		} else {
			if (chip->nu1619_chg_status.adapter_type == ADAPTER_TYPE_VOOC
				|| g_oplus_chip->soc >= 90
				|| g_oplus_chip->temperature < g_oplus_chip->limits.little_cool_bat_decidegc) {
				chip->nu1619_chg_status.charge_status = WPC_CHG_STATUS_CHARGER_FASTCHG_INIT;
			} else {
				cep_nonzero_cnt = 0;
				chip->nu1619_chg_status.charge_status = WPC_CHG_STATUS_INCREASE_VOLTAGE_FOR_CHARGEPUMP;
			}
		}
		break;
	case WPC_CHG_STATUS_INCREASE_VOLTAGE_FOR_CHARGEPUMP:
		chg_err("<~WPC~> ..........WPC_CHG_STATUS_INCREASE_VOLTAGE_FOR_CHARGEPUMP..........\n");
		if (nu1619_get_CEP_flag(chip) == 0) {
			temp_value = nu1619_increase_vout_to_target(chip, g_oplus_chip->batt_volt * 4 + 300, WPC_CHARGE_VOLTAGE_CHGPUMP_MAX);
			if (temp_value != 0) {
				nu1619_reset_CEP_flag(chip);
			} else {
				chip->nu1619_chg_status.charge_status = WPC_CHG_STATUS_CHARGEPUMP_INIT;
			}
			cep_nonzero_cnt = 0;
		} else {
#ifndef FASTCHG_TEST_BY_TIME
			cep_nonzero_cnt++;
			if (cep_nonzero_cnt > 10) {
				cep_nonzero_cnt = 0;
				chip->nu1619_chg_status.charge_status = WPC_CHG_STATUS_CHARGER_FASTCHG_INIT;
				chip->nu1619_chg_status.skewing_info = true;
				if (chip->nu1619_chg_status.cep_info < to_cep_info(chip->nu1619_chg_status.CEP_value))
					chip->nu1619_chg_status.cep_info = to_cep_info(chip->nu1619_chg_status.CEP_value);
			}
#endif
		}
		break;
	case WPC_CHG_STATUS_CHARGER_FASTCHG_INIT:
		chg_err("<~WPC~> ..........WPC_CHG_STATUS_CHARGER_FASTCHG_INIT..........\n");
		nu1619_set_rx_charge_voltage(chip, NU1619_WPC_CHARGE_VOLTAGE_FASTCHG_MIN);
		cep_zero_cnt = 0;
		cep_nonzero_cnt = 0;
		chip->nu1619_chg_status.wpc_ffc_charge = false;
		chip->nu1619_chg_status.has_reach_max_temperature = false;
		nu1619_set_rx_charge_current(chip, 1000);
		chip->nu1619_chg_status.iout_stated_current = 1000;
		mp2650_input_current_limit_without_aicl(2000);
		nu1619_set_tx_cep_timeout_1500ms();
		nu1619_set_rx_terminate_voltage(chip, WPC_TERMINATION_VOLTAGE_CV);
		chip->nu1619_chg_status.charge_status = WPC_CHG_STATUS_FAST_CHARGING_FROM_CHARGER;
		break;
	case WPC_CHG_STATUS_CHARGEPUMP_INIT:
		chg_err("<~WPC~> ..........WPC_CHG_STATUS_CHARGEPUMP_INIT..........\n");
		if (chargepump_set_for_EPP() != 0) {
			chg_err("<~WPC~> init chargepump\n");
			break;
		}
		if (chip->nu1619_chg_status.vout < (g_oplus_chip->batt_volt * 4 + 300)) {
			chg_err("<~WPC~> Vout < (2*Vbatt+300)\n");
			chip->nu1619_chg_status.charge_status = WPC_CHG_STATUS_INCREASE_VOLTAGE_FOR_CHARGEPUMP;
			break;
		}
		chargepump_enable();
		chargepump_check_dwp_status();
		if (chargepump_check_dwp_status() == -1) {
			chg_err("<~WPC~> open chargepump false!\n");
			chargepump_disable();
			if ((chip->nu1619_chg_status.vout >= 19000)
				|| (chip->nu1619_chg_status.vout >= (g_oplus_chip->batt_volt * 4 + 600))) {
				chg_err("<~WPC~> Vout >= 19V or Vout >= 2*Vbatt, turn to MP2762!\n");
				chip->nu1619_chg_status.adapter_type = ADAPTER_TYPE_VOOC;
				chip->nu1619_chg_status.charge_status = WPC_CHG_STATUS_CHARGER_FASTCHG_INIT;
				break;
			} else {
				chg_err("<~WPC~> Increase Vout 100mV\n");
				nu1619_set_rx_charge_voltage(chip, chip->nu1619_chg_status.charge_voltage + 100);
				chip->nu1619_chg_status.charge_status = WPC_CHG_STATUS_CHARGEPUMP_INIT;
				break;
			}
		} else if (chargepump_check_dwp_status() == -2) {
			chargepump_disable();
			chip->nu1619_chg_status.charge_status = WPC_CHG_STATUS_INCREASE_VOLTAGE_FOR_CHARGEPUMP;
			break;
		} else {
			chg_err("<~WPC~> open chargepump successful!\n");
			chargepump_dwp_disable();
			vout_value = nu1619_get_vout(chip);
			batt_volt = oplus_gauge_get_batt_mvolts();
			chg_err("<~WPC~> Vout: %d, Vbatt: %d\n", vout_value, batt_volt);
			if ((chargepump_check_dwp_status() == -2)
				|| (chip->nu1619_chg_status.vout < (batt_volt * 4 - 500))) {
				chg_err("<~WPC~> chargepump fail or Vout < 2*Vbatt - 500!\n");
				chargepump_disable();
				chip->nu1619_chg_status.charge_status = WPC_CHG_STATUS_INCREASE_VOLTAGE_FOR_CHARGEPUMP;
				break;
			}

			for(i = 0; i < CHARGEPUMP_DETECT_CNT; i++) {
				mdelay(5);
				vout_value = nu1619_get_vout(chip);
				if ((chip->nu1619_chg_status.vout < (batt_volt * 4 - 500))
					 || (chargepump_check_dwp_status() == -2)) {
					chg_err("<~WPC~> 40times: chargepump fail or Vout < 2*Vbatt - 500!\n");
					chargepump_disable();
					chip->nu1619_chg_status.charge_status = WPC_CHG_STATUS_INCREASE_VOLTAGE_FOR_CHARGEPUMP;
					break;
				}
				if (chargepump_check_fastchg_status() == 0) {
					chg_err("<~WPC~> 40times: chargepump is fastcharging!\n");
					chargepump_enable_watchdog();
					chip->nu1619_chg_status.charge_status = WPC_CHG_STATUS_CHAREPUMP_FASTCHG_INIT;
					break;
				}
			}
			if (i >= CHARGEPUMP_DETECT_CNT) {
				chg_err("<~WPC~> i >= CHARGEPUMP_DETECT_CNT\n");

				chargepump_disable();
				chip->nu1619_chg_status.charge_status = WPC_CHG_STATUS_INCREASE_VOLTAGE_FOR_CHARGEPUMP;
			}
		}
		break;
	case WPC_CHG_STATUS_CHAREPUMP_FASTCHG_INIT:
		chg_err("<~WPC~> ..........WPC_CHG_STATUS_CHAREPUMP_FASTCHG_INIT..........\n");
		nu1619_set_rx_charge_current(chip, 300);
		nu1619_set_FOD_parameter(chip, 17);
		cep_zero_cnt = 0;
		cep_nonzero_cnt = 0;
		chip->nu1619_chg_status.wpc_ffc_charge = true;
		chip->nu1619_chg_status.has_reach_max_temperature = false;
		chip->nu1619_chg_status.fastcharge_level = FASTCHARGE_LEVEL_UNKNOW;
		nu1619_set_rx_terminate_voltage(chip, WPC_TERMINATION_VOLTAGE_FFC);
#ifndef FASTCHG_TEST_BY_TIME
		nu1619_fastcharge_current_adjust_40w(chip, true);
#else
		nu1619_test_reset_record_seconds();
#endif
		nu1619_fastcharge_skewing_proc_40w(chip, true);
		chip->nu1619_chg_status.charge_status = WPC_CHG_STATUS_FAST_CHARGING_FROM_CHGPUMP;
		break;
	case WPC_CHG_STATUS_FAST_CHARGING_FROM_CHGPUMP:
		chargepump_kick_watchdog();
#ifndef FASTCHG_TEST_BY_TIME
		if (chargepump_check_fastchg_status() != 0) {
			chip->nu1619_chg_status.charge_status = WPC_CHG_STATUS_DECREASE_VOUT_FOR_RESTART;
			break;
		}
#endif
#ifdef DEBUG_FASTCHG_BY_ADB
		if (nu1619_fastcharge_debug_by_adb(chip) == 0) {
			break;
		}
#endif
#ifdef FASTCHG_TEST_BY_TIME
		if (nu1619_fastcharge_test_40w(chip) != 0) {
			break;
		} else {
			if (chargepump_check_fastchg_status() != 0) {
				chip->nu1619_chg_status.charge_status = WPC_CHG_STATUS_DECREASE_VOUT_FOR_RESTART;
				break;
			}
		}
#else
		if ((g_oplus_chip->batt_volt >= 4500) && (!chip->nu1619_chg_status.wpc_ffc_charge)) {
			chg_err("<~WPC~> batt_volt[%d] >= 4.5V, Not in FFC\n", g_oplus_chip->batt_volt);
			cep_nonzero_cnt = 0;
			nu1619_ready_to_switch_to_charger(chip, true);
			break;
		} else if (chip->nu1619_chg_status.wpc_ffc_charge && (g_oplus_chip->batt_volt >= 4500)
					&& ((g_oplus_chip->icharging < 0) && ((-1 * g_oplus_chip->icharging) < 900))) {
			chg_err("<~WPC~> batt_volt[%d] >= 4.5V, batt_cur[%d] < 900ma, In FFC\n", g_oplus_chip->batt_volt, (-1 * g_oplus_chip->icharging));
			cep_nonzero_cnt = 0;
			chip->nu1619_chg_status.wpc_ffc_charge = false;
			nu1619_ready_to_switch_to_charger(chip, true);
			break;
		} else if (!chip->nu1619_chg_status.wpc_ffc_charge && g_oplus_chip->batt_volt >= 4370) {
			chg_err("<~WPC~> batt_volt[%d], Not in FFC\n", g_oplus_chip->batt_volt);
			cep_nonzero_cnt = 0;
			nu1619_ready_to_switch_to_charger(chip, true);
			break;
		} else if (((g_oplus_chip->temperature < WPC_CHARGE_FFC_TEMP_MIN)
				|| (g_oplus_chip->temperature > WPC_CHARGE_FFC_TEMP_MAX)) && (g_oplus_chip->batt_volt > 4370)) {
			chg_err("<~WPC~> batt_volt[%d], temperature[%d]\n", g_oplus_chip->batt_volt, g_oplus_chip->temperature);
			cep_nonzero_cnt = 0;
			chip->nu1619_chg_status.wpc_ffc_charge = false;
			nu1619_ready_to_switch_to_charger(chip, true);
			break;
		}
		if ((nu1619_get_CEP_flag(chip) == 0) && (chip->nu1619_chg_status.wpc_skewing_proc == false)) {
			cep_nonzero_cnt = 0;
			if (chip->nu1619_chg_status.work_silent_mode || chip->nu1619_chg_status.call_mode) {
				cep_nonzero_cnt = 0;
				chip->nu1619_chg_status.wpc_ffc_charge = false;
				chg_err("<~WPC~> Enable silent mode. \n");
				nu1619_ready_to_switch_to_charger(chip, false);
			}
			nu1619_fastcharge_current_adjust_40w(chip, false);
		} else {
			if (nu1619_get_CEP_flag(chip) == 0) {
				cep_nonzero_cnt = 0;
				cep_zero_cnt++;
				if (cep_zero_cnt >= 10) {
					cep_zero_cnt = 0;
					chip->nu1619_chg_status.wpc_skewing_proc = false;
					chg_err("<~WPC~> turn to wpc_skewing_proc = false\n");

					if (nu1619_resume_chargepump_fastchg(chip) == 0) {
						chip->nu1619_chg_status.wpc_ffc_charge = false;
						nu1619_ready_to_switch_to_charger(chip, true);
						break;
					}
				}
			} else {
				cep_zero_cnt = 0;

				if (chip->nu1619_chg_status.wpc_skewing_proc == true) {
					nu1619_fastcharge_skewing_proc_40w(chip, false);
				} else {
					cep_nonzero_cnt++;
					if (cep_nonzero_cnt >= 3) {
						cep_nonzero_cnt = 0;
						chip->nu1619_chg_status.wpc_skewing_proc = true;
						chip->nu1619_chg_status.skewing_info = true;
						chg_err("<~WPC~> turn to wpc_skewing_proc = true\n");
					}
				}
			}
		}
#endif
		if (nu1619_get_CEP_flag(chip) == 0) {
			iout_max_value = chip->nu1619_chg_status.iout_stated_current + WPC_CHARGE_CURRENT_OFFSET;
			if (chip->nu1619_chg_status.iout_stated_current > WPC_CHARGE_CURRENT_OFFSET) {
				iout_min_value = chip->nu1619_chg_status.iout_stated_current - WPC_CHARGE_CURRENT_OFFSET;
			} else {
				iout_min_value = 0;
			}
			if (chip->nu1619_chg_status.iout > iout_max_value) {
				chg_err("<~WPC~> The Iout > %d.\n", iout_max_value);
				if (chip->nu1619_chg_status.charge_voltage > WPC_CHARGE_VOLTAGE_CHGPUMP_MIN) {
					temp_value = chip->nu1619_chg_status.iout - iout_max_value;
					if (chip->nu1619_chg_status.iout > 2100) {
						nu1619_set_rx_charge_voltage(chip, chip->nu1619_chg_status.charge_voltage - 200);
					} else if (temp_value > 50) {
						nu1619_set_rx_charge_voltage(chip, chip->nu1619_chg_status.charge_voltage - 100);
					} else {
						nu1619_set_rx_charge_voltage(chip, chip->nu1619_chg_status.charge_voltage - 20);
					}

					nu1619_reset_CEP_flag(chip);
					work_delay = 0;
					break;
				}
			} else if (chip->nu1619_chg_status.iout < iout_min_value) {
				chg_err("<~WPC~> The Iout < %d.\n", iout_min_value);
				if (chip->nu1619_chg_status.charge_voltage < WPC_CHARGE_VOLTAGE_CHGPUMP_MAX) {
					temp_value = iout_min_value - chip->nu1619_chg_status.iout;
					chg_err("<~WPC~> temp_value = %d.\n", temp_value);
					if ((temp_value > 100) && (chip->nu1619_chg_status.iout < 2200)) {
						nu1619_set_rx_charge_voltage(chip, chip->nu1619_chg_status.charge_voltage + 100);
					} else if ((temp_value > 50) && (chip->nu1619_chg_status.iout < 2200)) {
						nu1619_set_rx_charge_voltage(chip, chip->nu1619_chg_status.charge_voltage + 50);
					} else {
						nu1619_set_rx_charge_voltage(chip, chip->nu1619_chg_status.charge_voltage + 20);
					}
					nu1619_reset_CEP_flag(chip);
					if (chip->nu1619_chg_status.iout > 1500) {
						work_delay = 3;
					} else {
						work_delay = 0;
					}
					break;
				}
			} else {
				if (!chip->nu1619_chg_status.wpc_reach_stable_charge) {
					chg_err("<~WPC~> set wpc_reach_stable_charge = true\n");
					chip->nu1619_chg_status.wpc_reach_stable_charge = true;
					if (g_oplus_chip->batt_volt >= 4370) {
						chg_err("<~WPC~> Reach 4370mV after connected. Exit FFC\n");
						chip->nu1619_chg_status.wpc_reach_4370mv = true;
						chip->nu1619_chg_status.wpc_ffc_charge = false;
						nu1619_set_rx_terminate_voltage(chip, WPC_TERMINATION_VOLTAGE_CV);
					}
				}
			}
		}
		break;
	case WPC_CHG_STATUS_DECREASE_IOUT_TO_200MA:
		chg_err("<~WPC~> decreasing IOUT, IOUT = %d \n", chip->nu1619_chg_status.iout);
		if ((nu1619_get_CEP_flag(chip) == 0) || chip->nu1619_chg_status.wpc_skewing_proc) {
			if (chip->nu1619_chg_status.iout > chip->nu1619_chg_status.wpc_chg_param.curr_cp_to_charger) {
				if (chip->nu1619_chg_status.charge_voltage > WPC_CHARGE_VOLTAGE_CHGPUMP_MIN) {
					temp_value = chip->nu1619_chg_status.iout - chip->nu1619_chg_status.wpc_chg_param.curr_cp_to_charger;
					if (chip->nu1619_chg_status.iout > 2100) {
						nu1619_set_rx_charge_voltage(chip, chip->nu1619_chg_status.charge_voltage - 200);
					} else if (temp_value > 100) {
						nu1619_set_rx_charge_voltage(chip, chip->nu1619_chg_status.charge_voltage - 100);
					} else {
						nu1619_set_rx_charge_voltage(chip, chip->nu1619_chg_status.charge_voltage - 20);
					}

					nu1619_reset_CEP_flag(chip);
					work_delay = 0;
				}  else {
					nu1619_restart_charger(chip);
					chip->nu1619_chg_status.charge_status = WPC_CHG_STATUS_DECREASE_VOUT_TO_12V;
				}
			} else {
				nu1619_restart_charger(chip);
				chip->nu1619_chg_status.charge_status = WPC_CHG_STATUS_DECREASE_VOUT_TO_12V;
			}
		}
		break;
	case WPC_CHG_STATUS_DECREASE_VOUT_TO_12V:
		if ((nu1619_get_CEP_flag(chip) == 0) || chip->nu1619_chg_status.wpc_skewing_proc) {
			temp_value = nu1619_decrease_vout_to_target(chip, WPC_CHARGE_VOLTAGE_CHGPUMP_TO_CHARGER, WPC_CHARGE_VOLTAGE_CHGPUMP_MIN);
			if (temp_value == 0) {
				mp2650_input_current_limit_without_aicl(1000);
				chip->nu1619_chg_status.iout_stated_current = 1000;
				self_reset_cnt = 0;
				chip->nu1619_chg_status.charge_status = WPC_CHG_STATUS_FAST_CHARGING_FROM_CHARGER;
			}
		}
		break;
	case WPC_CHG_STATUS_INCREASE_CURRENT_FOR_CHARGER:
		chg_err("<~WPC~> ..........WPC_CHG_STATUS_INCREASE_CURRENT_FOR_CHARGER..........\n");
		temp_value = nu1619_increase_vc_by_step(chip,
												chip->nu1619_chg_status.charge_current,
												chip->nu1619_chg_status.fastchg_current_limit,
												500);
		if (temp_value != 0) {
			nu1619_set_rx_charge_current(chip, temp_value);
			work_delay = WPC_INCREASE_CURRENT_DELAY;
			chip->nu1619_chg_status.charge_status = WPC_CHG_STATUS_ADJUST_VOL_AFTER_INC_CURRENT;
		} else {
			self_reset_cnt = 0;
			chip->nu1619_chg_status.charge_status = WPC_CHG_STATUS_FAST_CHARGING_FROM_CHARGER;
		}
		break;
	case WPC_CHG_STATUS_ADJUST_VOL_AFTER_INC_CURRENT:
		chg_err("<~WPC~> ..........WPC_CHG_STATUS_ADJUST_VOL_AFTER_INC_CURRENT..........\n");
		if (chip->nu1619_chg_status.iout > (chip->nu1619_chg_status.iout_stated_current + WPC_CHARGE_CURRENT_OFFSET)) {
			chg_err("<~WPC~>  IDT Iout > %dmA!\n", (chip->nu1619_chg_status.iout_stated_current + WPC_CHARGE_CURRENT_OFFSET));
			temp_value = nu1619_increase_vc_by_step(chip,
													chip->nu1619_chg_status.charge_voltage,
													NU1619_WPC_CHARGE_VOLTAGE_FASTCHG_MAX,
													WPC_CHARGE_VOLTAGE_CHANGE_STEP_1V);
			if (temp_value != 0) {
				nu1619_set_rx_charge_voltage(chip, temp_value);
			} else {
				self_reset_cnt = 0;
				chip->nu1619_chg_status.charge_status = WPC_CHG_STATUS_FAST_CHARGING_FROM_CHARGER;
			}
		} else {
			chip->nu1619_chg_status.charge_status = WPC_CHG_STATUS_INCREASE_CURRENT_FOR_CHARGER;
		}
		break;
	case WPC_CHG_STATUS_FAST_CHARGING_FROM_CHARGER:
		if (chip->nu1619_chg_status.vout < 12000) {
			nu1619_set_FOD_parameter(chip, 10);
		} else if (chip->nu1619_chg_status.vout > 12100) {
			nu1619_set_FOD_parameter(chip, 0);
		}
		if (chip->nu1619_chg_status.adapter_type == ADAPTER_TYPE_VOOC) {
			if(chip->nu1619_chg_status.fastchg_current_limit > 3000) {
				chg_err("<~WPC~>fastchg_current_limit > 3000\n");
				chip->nu1619_chg_status.fastchg_current_limit = 3000;
				nu1619_set_rx_charge_current(chip, chip->nu1619_chg_status.fastchg_current_limit);
				work_delay = WPC_ADJUST_CV_DELAY;
				break;
			}
		}
		if (g_oplus_chip->temperature < 410 + shake_temp
				&& chip->nu1619_chg_status.charge_voltage < NU1619_WPC_CHARGE_VOLTAGE_FASTCHG) {
			temp_value = nu1619_increase_vc_by_step(chip,
													chip->nu1619_chg_status.charge_voltage,
													NU1619_WPC_CHARGE_VOLTAGE_FASTCHG,
													WPC_CHARGE_VOLTAGE_CHANGE_STEP_1V);
			if (temp_value != 0) {
				nu1619_set_rx_charge_voltage(chip, temp_value);
				chg_err("<~WPC~> increase charger voltage\n");
				work_delay = WPC_INCREASE_CURRENT_DELAY;
				break;
			}
		}
		if (g_oplus_chip->temperature >= 410 + shake_temp) {
			chg_err("<~WPC~> The tempearture is >= %d\n", 410 + shake_temp);
			nu1619_set_rx_charge_current(chip, 1250);
			nu1619_set_rx_charge_voltage(chip, WPC_CHARGE_VOLTAGE_FASTCHG_MIN);
			work_delay = WPC_ADJUST_CV_DELAY;
			shake_temp = -10;
		} else if ((g_oplus_chip->temperature >= 380 + shake_temp) && (g_oplus_chip->temperature < 410 + shake_temp)) {
			chg_err("<~WPC~> The tempearture is >= %d\n", 380 + shake_temp);
			chip->nu1619_chg_status.fastchg_current_limit = 2000;
			if (chip->nu1619_chg_status.charge_current < chip->nu1619_chg_status.fastchg_current_limit) {
				chg_err("<~WPC~> The tempearture is < 38. Icrease current.\n");
				temp_value = nu1619_increase_vc_by_step(chip,
												chip->nu1619_chg_status.charge_current,
												chip->nu1619_chg_status.fastchg_current_limit,
												500);
				if (temp_value != 0) {
					nu1619_set_rx_charge_current(chip, temp_value);
					work_delay = WPC_INCREASE_CURRENT_DELAY;
				}
			} else {
				temp_value = nu1619_decrease_vc_by_step(chip,
												chip->nu1619_chg_status.charge_current,
												chip->nu1619_chg_status.fastchg_current_limit,
												500);
				if (temp_value != 0) {
					nu1619_set_rx_charge_current(chip, temp_value);
					chg_err("<~WPC~> The tempearture is < 38. Decrease current.\n");
					work_delay = WPC_INCREASE_CURRENT_DELAY;
				}
			}
			shake_temp = 0;
		} else if (g_oplus_chip->temperature < 380 + shake_temp) {
			chg_err("<~WPC~> The tempearture is <= %d\n", 380 + shake_temp);
			chip->nu1619_chg_status.fastchg_current_limit = 3000;
			if (chip->nu1619_chg_status.charge_current < chip->nu1619_chg_status.fastchg_current_limit) {
				chg_err("<~WPC~> The tempearture is < 38. Icrease current.\n");
				temp_value = nu1619_increase_vc_by_step(chip,
												chip->nu1619_chg_status.charge_current,
												chip->nu1619_chg_status.fastchg_current_limit,
												500);
				if (temp_value != 0) {
					nu1619_set_rx_charge_current(chip, temp_value);
					work_delay = WPC_INCREASE_CURRENT_DELAY;
				}
			}
			shake_temp = 10;
		}
		break;
	case WPC_CHG_STATUS_READY_FOR_FTM:
		chg_err("<~WPC~> ..........WPC_CHG_STATUS_READY_FOR_FTM..........\n");
		if (nu1619_get_CEP_flag(chip) == 0) {
			temp_value = nu1619_increase_vc_by_step(chip,
													chip->nu1619_chg_status.charge_voltage,
													WPC_CHARGE_VOLTAGE_FTM,
													WPC_CHARGE_VOLTAGE_CHANGE_STEP_1V);
			if (temp_value != 0) {
				nu1619_set_rx_charge_voltage(chip, temp_value);
				nu1619_reset_CEP_flag(chip);
			} else {
				if (chargepump_set_for_EPP() != 0) {
					chg_err("<~WPC~> init chargepump again\n");
					break;
				}

				chargepump_enable();
				chargepump_set_for_LDO();
				chargepump_enable_watchdog();
				chip->nu1619_chg_status.charge_status = WPC_CHG_STATUS_FTM_WORKING;
			}
		}
		break;
	case WPC_CHG_STATUS_FTM_WORKING:
		chg_err("<~WPC~> ..........WPC_CHG_STATUS_FTM_WORKING..........\n");
		chargepump_kick_watchdog();
		break;
	case WPC_CHG_STATUS_DECREASE_VOUT_FOR_RESTART:
		chg_err("<~WPC~> ..........WPC_CHG_STATUS_DECREASE_VOUT_FOR_RESTART..........\n");
		temp_value = nu1619_decrease_vout_to_target(chip, g_oplus_chip->batt_volt * 4 + 300, WPC_CHARGE_VOLTAGE_CHGPUMP_MIN);
		if (temp_value == 0) {
			chg_err("<~WPC~> restart chargepump!\n");
			chargepump_disable();
			msleep(100);
			/*chargepump_set_for_EPP();*/

			cep_nonzero_cnt = 0;
			chip->nu1619_chg_status.charge_status = WPC_CHG_STATUS_INCREASE_VOLTAGE_FOR_CHARGEPUMP;
		}
		break;
	default:
		break;
	}

	return 0;
}

static void nu1619_idt_dischg_status(struct oplus_nu1619_ic *chip)
{
	char regdata[2] = {0};
	int rc = 0;

	nu1619_read_reg(chip, 0x23, regdata, 2);
	chg_err("<~WPC~>rtx func 0x23-->[0x%x],0x24-->[0x%x]!, wpc_dischg_status[%d]\n",
		regdata[0], regdata[1], chip->nu1619_chg_status.wpc_dischg_status);
	regdata[1] = 0;
	if (regdata[1] != 0) {
		if (P922X_REG_RTX_ERR_TX_RXAC & regdata[1]) {
			chip->nu1619_chg_status.wpc_dischg_status = WPC_DISCHG_IC_ERR_TX_RXAC;
		} else if (P922X_REG_RTX_ERR_TX_OCP & regdata[1]) {
			chip->nu1619_chg_status.wpc_dischg_status = WPC_DISCHG_IC_ERR_TX_OCP;
		} else if (P922X_REG_RTX_ERR_TX_OVP & regdata[1]) {
			chip->nu1619_chg_status.wpc_dischg_status = WPC_DISCHG_IC_ERR_TX_OVP;
		} else if (P922X_REG_RTX_ERR_TX_LVP & regdata[1]) {
			chip->nu1619_chg_status.wpc_dischg_status = WPC_DISCHG_IC_ERR_TX_LVP;
		} else if (P922X_REG_RTX_ERR_TX_FOD & regdata[1]) {
			chip->nu1619_chg_status.wpc_dischg_status = WPC_DISCHG_IC_ERR_TX_FOD;
		} else if (P922X_REG_RTX_ERR_TX_OTP & regdata[1]) {
			chip->nu1619_chg_status.wpc_dischg_status = WPC_DISCHG_IC_ERR_TX_OTP;
		} else if (P922X_REG_RTX_ERR_TX_CEPTIMEOUT & regdata[1]) {
			chip->nu1619_chg_status.wpc_dischg_status = WPC_DISCHG_IC_ERR_TX_CEPTIMEOUT;
		} else if (P922X_REG_RTX_ERR_TX_RXEPT & regdata[1]) {
			chip->nu1619_chg_status.wpc_dischg_status = WPC_DISCHG_IC_ERR_TX_RXEPT;
		}

		switch (chip->nu1619_chg_status.wpc_dischg_status) {
		case WPC_DISCHG_IC_ERR_TX_RXAC:
		case WPC_DISCHG_IC_ERR_TX_OCP:
		case WPC_DISCHG_IC_ERR_TX_OVP:
		case WPC_DISCHG_IC_ERR_TX_LVP:
		case WPC_DISCHG_IC_ERR_TX_FOD:
		case WPC_DISCHG_IC_ERR_TX_OTP:
		case WPC_DISCHG_IC_ERR_TX_RXEPT:
			nu1619_disable_tx_power();
			break;
		case WPC_DISCHG_IC_ERR_TX_CEPTIMEOUT:
			chg_err("<~WPC~>rtx func RTX_ERR_TX_CEPTIMEOUT\n");
			break;
		default:
			break;
		}
/*
		if ((P922X_REG_RTX_ERR_TX_CEPTIMEOUT & regdata[1]) == 0) {
			//chip->nu1619_chg_status.wpc_dischg_status = WPC_DISCHG_STATUS_OFF;
			//g_oplus_chip->otg_switch = 0;
			mp2650_otg_disable();
			mp2650_otg_wait_vbus_decline();
			oplus_set_wrx_otg_value(0);
			oplus_set_wrx_en_value(0);
		}
*/
		if (P922X_RTX_TRANSFER & regdata[0]) {
			chip->nu1619_chg_status.tx_online = true;
			wpc_battery_update();
			chg_err("<~WPC~>rtx func in discharging now, tx_online online!\n");
		} else {
			chip->nu1619_chg_status.tx_online = false;
		}
	} else {
		if (P922X_RTX_READY & regdata[0]) {
			chip->nu1619_chg_status.tx_online = false;
			chip->nu1619_chg_status.wpc_dischg_status = WPC_DISCHG_IC_READY;
			rc = nu1619_write_cmd_D(chip, 0x07);
			schedule_delayed_work(&chip->idt_dischg_work, 0);
		} else if (P922X_RTX_DIGITALPING & regdata[0] || P922X_RTX_ANALOGPING & regdata[0]) {
			chip->nu1619_chg_status.tx_online = false;
			if (WPC_DISCHG_IC_PING_DEVICE == chip->nu1619_chg_status.wpc_dischg_status) {
				chg_err("<~WPC~>rtx func no device to be charged, ping device...\n");
			} else {
				chip->nu1619_chg_status.wpc_dischg_status = WPC_DISCHG_IC_PING_DEVICE;
				schedule_delayed_work(&chip->idt_dischg_work, WPC_DISCHG_WAIT_DEVICE_EVENT);
			}
		} else if (P922X_RTX_TRANSFER & regdata[0]) {
			chip->nu1619_chg_status.tx_online = true;
			chip->nu1619_chg_status.wpc_dischg_status = WPC_DISCHG_IC_TRANSFER;
			wpc_battery_update();
			chg_err("<~WPC~>rtx func in discharging now!\n");
		}
	}

	return;
}

static void nu1619_idt_dischg_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_nu1619_ic *chip = container_of(dwork, struct oplus_nu1619_ic, idt_dischg_work);

	chg_err("<~WPC~>rtx func wpc_dischg_status[%d]\n", chip->nu1619_chg_status.wpc_dischg_status);

	if (chip->nu1619_chg_status.wpc_dischg_status == WPC_DISCHG_STATUS_OFF) {
		mp2650_otg_disable();
		mp2650_otg_wait_vbus_decline();
		oplus_set_wrx_otg_value(0);
		oplus_set_wrx_en_value(0);
	} else {
		/*nu1619_idt_dischg_status(chip);*/
	}

	return;
}

static void nu1619_commu_data_process(struct oplus_nu1619_ic *chip)
{
	int rc = -1;
	char temp[3] = { 0, 0, 0 };
	char val_buf[5] = { 0, 0, 0, 0, 0 };
	char tx_command, tx_command_r;
	char tx_data, tx_data_r;

	chg_err("<~WPC~>rtx func chip->nu1619_chg_status.wpc_dischg_status[%d]\n", chip->nu1619_chg_status.wpc_dischg_status);
	if (chip->nu1619_chg_status.wpc_dischg_status == WPC_DISCHG_STATUS_ON
		|| chip->nu1619_chg_status.wpc_dischg_status == WPC_DISCHG_IC_READY
		|| chip->nu1619_chg_status.wpc_dischg_status == WPC_DISCHG_IC_PING_DEVICE
		|| chip->nu1619_chg_status.wpc_dischg_status == WPC_DISCHG_IC_TRANSFER
		|| chip->nu1619_chg_status.wpc_dischg_status == WPC_DISCHG_IC_ERR_TX_CEPTIMEOUT) {
		cancel_delayed_work_sync(&chip->idt_dischg_work);
		nu1619_idt_dischg_status(chip);
	}

	rc = 0;
	nu1619_write_reg(chip, 0x000c, 0xb3);
	nu1619_read_reg(chip, 0x0008, temp, 3);
	if (temp[0] == (0xb3 ^ 0x80)) {
		rc += 1;
		val_buf[0] = temp[1];
		val_buf[1] = temp[2];
	}
	nu1619_write_reg(chip, 0x000c, 0xb5);
	nu1619_read_reg(chip, 0x0008, temp, 3);
	if (temp[0] == (0xb5 ^ 0x80)) {
		rc += 1;
		val_buf[2] = temp[1];
		val_buf[3] = temp[2];
	}
	nu1619_write_reg(chip, 0x000c, 0xb7);
	nu1619_read_reg(chip, 0x0008, temp, 3);
	if (temp[0] == (0xb7 ^ 0x80)) {
		rc += 1;
		val_buf[4] = temp[1];
	}
	chg_err("rc = %x\n", rc);
	if (rc != 3) {
		chg_err("Couldn't read 0x%04x rc = %x\n", 0x0058, rc);
	} else {
		chg_err("Header = 0x%04x\n", val_buf[0]);
		if (val_buf[0] == 0x4F) {
			tx_command = val_buf[1];
			tx_command_r = ~val_buf[2];
			tx_data = val_buf[3];
			tx_data_r = ~val_buf[4];
			chg_err("<~WPC~> Received TX command: 0x%02X, data: 0x%02X\n", tx_command, tx_data);
			switch (tx_command) {
			case P9237_RESPONE_ADAPTER_TYPE:
				chg_err("<~WPC~> P9237_RESPONE_ADAPTER_TYPE\n");
				if ((tx_command == tx_command_r) && (tx_data == tx_data_r)) {
					nu1619_set_FOD_parameter(chip, 1);
					chip->nu1619_chg_status.adapter_type = (tx_data & 0x07);
					chip->nu1619_chg_status.dock_version = (tx_data & 0xF8) >> 3;
					chip->nu1619_chg_status.adapter_type = ADAPTER_TYPE_VOOC;
					if (chip->nu1619_chg_status.adapter_type == ADAPTER_TYPE_SVOOC) {
						wpc_battery_update();
					}
					chg_err("<~WPC~> get adapter type = 0x%02X\n", chip->nu1619_chg_status.adapter_type);
					chg_err("<~WPC~> get dock hw version = 0x%02X\n", chip->nu1619_chg_status.dock_version);
				}
				chg_err("<~WPC~> P9237_RESPONE_ADAPTER_TYPE data error\n");
				break;
			case P9237_RESPONE_INTO_FASTCHAGE:
				chg_err("<~WPC~> P9237_RESPONE_INTO_FASTCHAGE\n");
				if ((tx_command == tx_command_r) && (tx_data == tx_data_r)) {
					chip->nu1619_chg_status.charge_type = WPC_CHARGE_TYPE_FAST;
					chg_err("<~WPC~> enter charge type = WPC_CHARGE_TYPE_FAST\n");
					if ((tx_data >= 0) && (tx_data <= ADAPTER_POWER_MAX)) {
						chip->nu1619_chg_status.adapter_power = tx_data;
					} else {
						chip->nu1619_chg_status.adapter_power = ADAPTER_POWER_NUKNOW;
					}
					chg_err("<~WPC~> Adapter power type = %d\n", chip->nu1619_chg_status.adapter_power);
					if (chip->nu1619_chg_status.send_message == P9221_CMD_INTO_FASTCHAGE) {
						chip->nu1619_chg_status.send_message = P9221_CMD_NULL;
					}
				}
				break;
			case P9237_RESPONE_CEP_TIMEOUT:
				chg_err("<~WPC~> P9237_RESPONE_CEP_TIMEOUT\n");
				if ((tx_command == tx_command_r) && (tx_data == tx_data_r)) {
					chg_err("<~WPC~> P9237_RESPONE_CEP_TIMEOUT\n");
					if (chip->nu1619_chg_status.send_message == P9221_CMD_SET_CEP_TIMEOUT) {
						chip->nu1619_chg_status.send_message = P9221_CMD_NULL;
					}
				}
				break;
			case P9237_RESPONE_PWM_PULSE:
				chg_err("<~WPC~> P9237_RESPONE_PWM_PULSE\n");
				if ((tx_command == tx_command_r) && (tx_data == tx_data_r)) {
					chg_err("<~WPC~> P9237_RESPONE_PWM_PULSE\n");
					if (chip->nu1619_chg_status.send_message == P9221_CMD_SET_PWM_PULSE) {
						chip->nu1619_chg_status.send_message = P9221_CMD_NULL;
					}
				}
				break;
			case P9237_RESPONE_LED_BRIGHTNESS:
				chg_err("<~WPC~> P9237_RESPONE_LED_BRIGHTNESS\n");
				if ((tx_command == tx_command_r) && (tx_data == tx_data_r)) {
					chg_err("<~WPC~> P9237_RESPONE_LED_BRIGHTNESS\n");
					if (chip->nu1619_chg_status.send_message == P9221_CMD_SET_LED_BRIGHTNESS) {
						chip->nu1619_chg_status.send_message = P9221_CMD_NULL;
					}
				}
				break;
			default:
				chg_err("<~WPC~> default\n");
				break;
			}
		}
	}
	nu1619_clear_irq(chip, temp[0], temp[1]);
	msleep(5);
}


static void nu1619_get_running_mode(struct oplus_nu1619_ic *chip)
{
	int rc = -1;
	char val_buf[5] = { 0, 0, 0, 0, 0 };
	char temp;
	rc = nu1619_write_reg(chip, 0x000c, 0x9d);
	rc = nu1619_read_reg(chip, 0x0008, val_buf, 3);
	if (val_buf[0] == (0x9d ^ 0x80)) {
		temp = val_buf[1];
	}
	if (rc) {
		chg_err("Couldn't read 0x0088 rc = %x\n", rc);
	} else {
		chg_err("<~WPC~>REG 0x0088 = %x\n", temp);
		if (temp == 0x31) {
			chg_err("<~WPC~> RX running in EPP!\n");
			chip->nu1619_chg_status.adapter_type = ADAPTER_TYPE_EPP;
			chip->nu1619_chg_status.rx_runing_mode = RX_RUNNING_MODE_EPP;
		} else if (temp == 0x04) {
			chg_err("<~WPC~> RX running in BPP!\n");
			chip->nu1619_chg_status.rx_runing_mode = RX_RUNNING_MODE_BPP;
		} else {
			chg_err("<~WPC~> RX running in Others!\n");
			chip->nu1619_chg_status.rx_runing_mode = RX_RUNNING_MODE_OTHERS;
		}
	}
}


int nu1619_get_idt_con_val(void)
{
	struct oplus_nu1619_ic *chip = nu1619_chip;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: nu1619_chip not ready!\n", __func__);
		return -1;
	}

	if (chip->idt_con_gpio <= 0) {
		chg_err("idt_con_gpio not exist, return\n");
		return -1;
	}

	if (IS_ERR_OR_NULL(chip->pinctrl)
		|| IS_ERR_OR_NULL(chip->idt_con_active)
		|| IS_ERR_OR_NULL(chip->idt_con_sleep)) {
		chg_err("pinctrl null, return\n");
		return -1;
	}

	return gpio_get_value(chip->idt_con_gpio);
}

int nu1619_get_idt_int_val(void)
{
	struct oplus_nu1619_ic *chip = nu1619_chip;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: nu1619_chip not ready!\n", __func__);
		return -1;
	}

	if (chip->idt_int_gpio <= 0) {
		chg_err("idt_int_gpio not exist, return\n");
		return -1;
	}

	if (IS_ERR_OR_NULL(chip->pinctrl)
		|| IS_ERR_OR_NULL(chip->idt_int_active)
		|| IS_ERR_OR_NULL(chip->idt_int_sleep)) {
		chg_err("pinctrl null, return\n");
		return -1;
	}

	return gpio_get_value(chip->idt_int_gpio);
}

static void nu1619_idt_event_int_func(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_nu1619_ic *chip = container_of(dwork, struct oplus_nu1619_ic, idt_event_int_work);

	chg_err("<~WPC~> action for 22222\n");

	if (nu1619_chip->nu1619_chg_status.charge_online == true
			|| nu1619_wpc_get_otg_charging() == true) {
		nu1619_commu_data_process(chip);
	}
}

static void nu1619_idt_connect_int_func(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_nu1619_ic *chip = container_of(dwork, struct oplus_nu1619_ic, idt_connect_int_work);

	if (nu1619_firmware_is_updating() == true) {
		/*chg_err("<~WPC~> nu1619_firmware_is_updating, return\n");*/
		return;
	}

#ifndef FASTCHG_TEST_BY_TIME
	if (oplus_get_wired_chg_present() == true
			&& (chip->wireless_mode == WIRELESS_MODE_NULL)
			&& chip->nu1619_chg_status.wpc_self_reset == false) {
		nu1619_set_vt_sleep_val(1);
		chg_err("<~WPC~> wired charging, return\n");
		return;
	}
#endif

	chg_err("<~WPC~> action for 11111\n");
	if (nu1619_chip->wireless_mode == WIRELESS_MODE_TX) {
		chg_err("<~WPC~> wireless_mode = WIRELESS_MODE_TX, return\n");
		return;
	}


	if (nu1619_get_idt_con_val() == 1) {
		chg_err(" !!!!! <~WPC~>[-TEST-] wpc dock has connected!>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
		if (nu1619_chip->nu1619_chg_status.charge_online == false) {
			nu1619_chip->nu1619_chg_status.charge_online = true;
			chip->wireless_mode = WIRELESS_MODE_RX;

			oplus_chg_cancel_update_work_sync();
			cancel_delayed_work_sync(&chip->nu1619_task_work);
			cancel_delayed_work_sync(&chip->idt_event_int_work);

			chg_err("<~WPC~> ready for wireless charge\n");
			msleep(1500);

			nu1619_get_running_mode(chip);

			oplus_set_wrx_en_value(1);

			nu1619_init(chip);

			nu1619_set_tx_Q_value(chip);

			nu1619_charger_init(chip);

			msleep(50);
			if (chip->nu1619_chg_status.wpc_self_reset) {
				chg_err("<~WPC~> self reset: wpc_self_reset true, force adapter_type to normal charge\n");
				chip->nu1619_chg_status.adapter_type = ADAPTER_TYPE_NORMAL_CHARGE;
			} else {
				nu1619_set_tx_charger_dect(chip);
			}

			chip->nu1619_chg_status.send_msg_timer = 0;
			chip->nu1619_chg_status.wpc_reach_stable_charge = false;
			chip->nu1619_chg_status.wpc_reach_4370mv = false;
			chip->nu1619_chg_status.wpc_reach_4500mv = false;
			chip->nu1619_chg_status.wpc_ffc_charge = false;
#ifdef DEBUG_FASTCHG_BY_ADB
			chip->nu1619_chg_status.vout_debug_mode = false;
			chip->nu1619_chg_status.iout_debug_mode = false;
#endif

			schedule_delayed_work(&chip->nu1619_task_work, round_jiffies_relative(msecs_to_jiffies(100)));

			oplus_chg_restart_update_work();
#ifdef VENDOR_EDIT
			switch_wireless_charger_state(1);
#endif
		}

	} else {
		chg_err(" !!!!! <~WPC~>[-TEST-] wpc dock has disconnected!< < < < < < < < < < < < <\n");
		if (nu1619_chip->nu1619_chg_status.charge_online == true) {
			nu1619_chip->nu1619_chg_status.charge_online = false;
			if (g_oplus_chip->charger_type == POWER_SUPPLY_TYPE_WIRELESS)
				g_oplus_chip->charger_type = POWER_SUPPLY_TYPE_UNKNOWN;
			chip->wireless_mode = WIRELESS_MODE_NULL;

			chg_err("<~WPC~> cancel delayed work\n");
			oplus_chg_cancel_update_work_sync();
			cancel_delayed_work_sync(&chip->nu1619_task_work);
			cancel_delayed_work_sync(&chip->idt_event_int_work);

			chg_err("<~WPC~> exit wireless charge\n");
			mp2650_set_vindpm_vol(4500);

			if (nu1619_get_vt_sleep_val() == 1) {
				chargepump_disable();
				nu1619_set_cp_ldo_5v_val(0);
				msleep(100);
				if (oplus_get_wired_otg_online() == false)
					oplus_set_wrx_en_value(0);
				nu1619_reset_variables(chip);
			} else {
				nu1619_set_vt_sleep_val(1);
				chargepump_disable();
				nu1619_set_cp_ldo_5v_val(0);
				msleep(100);
				if (oplus_get_wired_otg_online() == false)
					oplus_set_wrx_en_value(0);
				nu1619_reset_variables(chip);
				msleep(400);
				nu1619_set_vt_sleep_val(0);
			}

#ifdef FASTCHG_TEST_BY_TIME
			if (nu1619_test_charging_status() >= 1) {
				chg_err("<~WPC~>[-TEST-]charge test > 22s, stop charge!\n");
				stop_timer = 0;
				nu1619_set_vt_sleep_val(1);
				schedule_delayed_work(&chip->nu1619_test_work, round_jiffies_relative(msecs_to_jiffies(1000)));
			} else {
				chg_err("<~WPC~>[-TEST-]charge test <= 22s!\n");
			}

			test_is_charging = false;
#endif
			oplus_chg_restart_update_work();
#ifdef VENDOR_EDIT
			switch_wireless_charger_state(0);
#endif
		}
	}
}

static void nu1619_idt_event_shedule_work(void)
{
	if (!nu1619_chip) {
		chg_err(" nu1619_chip is NULL\n");
	} else {
		schedule_delayed_work(&nu1619_chip->idt_event_int_work, 0);
	}
}

static void nu1619_idt_connect_shedule_work(void)
{
	if (!nu1619_chip) {
		chg_err(" nu1619_chip is NULL\n");
	} else {
		schedule_delayed_work(&nu1619_chip->idt_connect_int_work, 0);
	}
}

static irqreturn_t irq_idt_event_int_handler(int irq, void *dev_id)
{
	chg_err("<~WPC~> 22222\n");
	nu1619_idt_event_shedule_work();
	return IRQ_HANDLED;
}

static irqreturn_t irq_idt_connect_int_handler(int irq, void *dev_id)
{
	/*chg_err("<~WPC~> 11111-> %d\n", nu1619_get_idt_con_val());*/
	nu1619_idt_connect_shedule_work();
	return IRQ_HANDLED;
}

static void nu1619_set_idt_int_active(struct oplus_nu1619_ic *chip)
{
	gpio_direction_input(chip->idt_int_gpio);
	pinctrl_select_state(chip->pinctrl, chip->idt_int_active);
}

static void nu1619_set_idt_con_active(struct oplus_nu1619_ic *chip)
{
	gpio_direction_input(chip->idt_con_gpio);
	pinctrl_select_state(chip->pinctrl, chip->idt_con_active);
}

static void nu1619_idt_int_irq_init(struct oplus_nu1619_ic *chip)
{
	chip->idt_int_irq = gpio_to_irq(chip->idt_int_gpio);
	pr_err("tongfeng test %s chip->idt_int_irq[%d]\n", __func__, chip->idt_int_irq);
}

static void nu1619_idt_con_irq_init(struct oplus_nu1619_ic *chip)
{
	chip->idt_con_irq = gpio_to_irq(chip->idt_con_gpio);
	pr_err("tongfeng test %s chip->idt_con_irq[%d]\n", __func__, chip->idt_con_irq);
}

static void nu1619_idt_int_eint_register(struct oplus_nu1619_ic *chip)
{
	int retval = 0;

	nu1619_set_idt_int_active(chip);
	retval = request_irq(chip->idt_int_irq, irq_idt_event_int_handler, IRQF_TRIGGER_FALLING, "nu1619_idt_int", chip);
	if (retval < 0) {
		pr_err("%s request idt_int irq failed.\n", __func__);
	}
	retval = enable_irq_wake(chip->idt_int_irq);
	if (retval != 0) {
		chg_err("enable_irq_wake: idt_int_irq failed %d\n", retval);
	}
}

static void nu1619_idt_con_eint_register(struct oplus_nu1619_ic *chip)
{
	int retval = 0;

	pr_err("%s tongfeng test start, irq happened\n", __func__);
	nu1619_set_idt_con_active(chip);
	retval = request_irq(chip->idt_con_irq, irq_idt_connect_int_handler, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "nu1619_con_int", chip);
	if (retval < 0) {
		pr_err("%s request idt_con irq failed.\n", __func__);
	}
}

static int nu1619_idt_con_gpio_init(struct oplus_nu1619_ic *chip)
{
	chip->pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->pinctrl)) {
		chg_err("get pinctrl fail\n");
		return -EINVAL;
	}

	chip->idt_con_active =
			pinctrl_lookup_state(chip->pinctrl, "idt_connect_active");
	if (IS_ERR_OR_NULL(chip->idt_con_active)) {
		chg_err("get idt_con_active fail\n");
		return -EINVAL;
	}

	chip->idt_con_sleep =
			pinctrl_lookup_state(chip->pinctrl, "idt_connect_sleep");
	if (IS_ERR_OR_NULL(chip->idt_con_sleep)) {
		chg_err("get idt_con_sleep fail\n");
		return -EINVAL;
	}

	chip->idt_con_default =
			pinctrl_lookup_state(chip->pinctrl, "idt_connect_default");
	if (IS_ERR_OR_NULL(chip->idt_con_default)) {
		chg_err("get idt_con_default fail\n");
		return -EINVAL;
	}

	if (chip->idt_con_gpio > 0) {
		gpio_direction_input(chip->idt_con_gpio);
	}

	pinctrl_select_state(chip->pinctrl, chip->idt_con_active);

	return 0;
}

static int nu1619_idt_int_gpio_init(struct oplus_nu1619_ic *chip)
{
	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_nu1619_ic not ready!\n", __func__);
		return -EINVAL;
	}

	chip->pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->pinctrl)) {
		chg_err("get pinctrl fail\n");
		return -EINVAL;
	}

	chip->idt_int_active =
			pinctrl_lookup_state(chip->pinctrl, "idt_int_active");
	if (IS_ERR_OR_NULL(chip->idt_int_active)) {
		chg_err("get idt_int_active fail\n");
		return -EINVAL;
	}

	chip->idt_int_sleep =
			pinctrl_lookup_state(chip->pinctrl, "idt_int_sleep");
	if (IS_ERR_OR_NULL(chip->idt_int_sleep)) {
		chg_err("get idt_int_sleep fail\n");
		return -EINVAL;
	}

	chip->idt_int_default =
			pinctrl_lookup_state(chip->pinctrl, "idt_int_default");
	if (IS_ERR_OR_NULL(chip->idt_int_default)) {
		chg_err("get idt_int_default fail\n");
		return -EINVAL;
	}

	if (chip->idt_int_gpio > 0) {
		gpio_direction_input(chip->idt_int_gpio);
	}

	pinctrl_select_state(chip->pinctrl, chip->idt_int_active);

	return 0;
}

static int nu1619_ext1_wired_otg_gpio_init(struct oplus_nu1619_ic *chip)
{
	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_nu1619_ic not ready!\n", __func__);
		return -EINVAL;
	}

	chip->pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->pinctrl)) {
		chg_err("get pinctrl fail\n");
		return -EINVAL;
	}

	chip->ext1_wired_otg_en_active =
			pinctrl_lookup_state(chip->pinctrl, "ext1_wired_otg_en_active");
	if (IS_ERR_OR_NULL(chip->ext1_wired_otg_en_active)) {
		chg_err("get ext1_wired_otg_en_active fail\n");
		return -EINVAL;
	}

	chip->ext1_wired_otg_en_sleep =
			pinctrl_lookup_state(chip->pinctrl, "ext1_wired_otg_en_sleep");
	if (IS_ERR_OR_NULL(chip->ext1_wired_otg_en_sleep)) {
		chg_err("get ext1_wired_otg_en_sleep fail\n");
		return -EINVAL;
	}

	chip->ext1_wired_otg_en_default =
			pinctrl_lookup_state(chip->pinctrl, "ext1_wired_otg_en_default");
	if (IS_ERR_OR_NULL(chip->ext1_wired_otg_en_default)) {
		chg_err("get ext1_wired_otg_en_default fail\n");
		return -EINVAL;
	}

#ifdef OPLUS_FEATURE_CHG_BASIC
	gpio_direction_output(chip->ext1_wired_otg_en_gpio, 1);
	pinctrl_select_state(chip->pinctrl,
			chip->ext1_wired_otg_en_active);
#else
	gpio_direction_output(chip->ext1_wired_otg_en_gpio, 0);
	pinctrl_select_state(chip->pinctrl,
			chip->ext1_wired_otg_en_default);
#endif


	return 0;
}

void nu1619_set_ext1_wired_otg_en_val(int value)
{
	struct oplus_nu1619_ic *chip = nu1619_chip;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_nu1619_ic not ready!\n", __func__);
		return;
	}

	if (chip->ext1_wired_otg_en_gpio <= 0) {
		chg_err("ext1_wired_otg_en_gpio not exist, return\n");
		return;
	}

	if (IS_ERR_OR_NULL(chip->pinctrl)
		|| IS_ERR_OR_NULL(chip->ext1_wired_otg_en_active)
		|| IS_ERR_OR_NULL(chip->ext1_wired_otg_en_sleep)
		|| IS_ERR_OR_NULL(chip->ext1_wired_otg_en_default)) {
		chg_err("pinctrl null, return\n");
		return;
	}

	if (value) {
		gpio_direction_output(chip->ext1_wired_otg_en_gpio, 1);
		pinctrl_select_state(chip->pinctrl,
				chip->ext1_wired_otg_en_default);
	} else {
		gpio_direction_output(chip->ext1_wired_otg_en_gpio, 0);
		pinctrl_select_state(chip->pinctrl,
				chip->ext1_wired_otg_en_sleep);
	}

	chg_err("<~WPC~>set value:%d, gpio_val:%d\n",
		value, gpio_get_value(chip->ext1_wired_otg_en_gpio));
}

int nu1619_get_ext1_wired_otg_en_val(void)
{
	struct oplus_nu1619_ic *chip = nu1619_chip;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: nu1619_chip not ready!\n", __func__);
		return -1;
	}

	if (chip->ext1_wired_otg_en_gpio <= 0) {
		chg_err("ext1_wired_otg_en_gpio not exist, return\n");
		return -1;
	}

	if (IS_ERR_OR_NULL(chip->pinctrl)
		|| IS_ERR_OR_NULL(chip->ext1_wired_otg_en_active)
		|| IS_ERR_OR_NULL(chip->ext1_wired_otg_en_sleep)
		|| IS_ERR_OR_NULL(chip->ext1_wired_otg_en_default)) {
		chg_err("pinctrl null, return\n");
		return -1;
	}

	return gpio_get_value(chip->ext1_wired_otg_en_gpio);
}


static int nu1619_ext2_wireless_otg_gpio_init(struct oplus_nu1619_ic *chip)
{
	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_nu1619_ic not ready!\n", __func__);
		return -EINVAL;
	}

	chip->pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->pinctrl)) {
		chg_err("get pinctrl fail\n");
		return -EINVAL;
	}

	chip->ext2_wireless_otg_en_active =
			pinctrl_lookup_state(chip->pinctrl, "ext2_wireless_otg_en_active");
	if (IS_ERR_OR_NULL(chip->ext2_wireless_otg_en_active)) {
		chg_err("get ext2_wireless_otg_en_active fail\n");
		return -EINVAL;
	}

	chip->ext2_wireless_otg_en_sleep =
			pinctrl_lookup_state(chip->pinctrl, "ext2_wireless_otg_en_sleep");
	if (IS_ERR_OR_NULL(chip->ext2_wireless_otg_en_sleep)) {
		chg_err("get ext2_wireless_otg_en_sleep fail\n");
		return -EINVAL;
	}

	chip->ext2_wireless_otg_en_default =
			pinctrl_lookup_state(chip->pinctrl, "ext2_wireless_otg_en_default");
	if (IS_ERR_OR_NULL(chip->ext2_wireless_otg_en_default)) {
		chg_err("get ext2_wireless_otg_en_default fail\n");
		return -EINVAL;
	}

#ifdef OPLUS_FEATURE_CHG_BASIC
	gpio_direction_output(chip->ext2_wireless_otg_en_gpio, 1);
	pinctrl_select_state(chip->pinctrl,
			chip->ext2_wireless_otg_en_active);
#else
	gpio_direction_output(chip->ext2_wireless_otg_en_gpio, 0);
	pinctrl_select_state(chip->pinctrl,
			chip->ext2_wireless_otg_en_default);
#endif

	return 0;
}

void nu1619_set_ext2_wireless_otg_en_val(int value)
{
	struct oplus_nu1619_ic *chip = nu1619_chip;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_nu1619_ic not ready!\n", __func__);
		return;
	}

	if (chip->ext2_wireless_otg_en_gpio <= 0) {
		chg_err("ext2_wireless_otg_en_gpio not exist, return\n");
		return;
	}

	if (IS_ERR_OR_NULL(chip->pinctrl)
		|| IS_ERR_OR_NULL(chip->ext2_wireless_otg_en_active)
		|| IS_ERR_OR_NULL(chip->ext2_wireless_otg_en_sleep)
		|| IS_ERR_OR_NULL(chip->ext2_wireless_otg_en_default)) {
		chg_err("pinctrl null, return\n");
		return;
	}

	if (value) {
		gpio_direction_output(chip->ext2_wireless_otg_en_gpio, 1);
		pinctrl_select_state(chip->pinctrl,
				chip->ext2_wireless_otg_en_default);
	} else {
		gpio_direction_output(chip->ext2_wireless_otg_en_gpio, 0);
		pinctrl_select_state(chip->pinctrl,
				chip->ext2_wireless_otg_en_sleep);
	}

	chg_err("<~WPC~>set value:%d, gpio_val:%d\n",
		value, gpio_get_value(chip->ext2_wireless_otg_en_gpio));
}

int nu1619_get_ext2_wireless_otg_en_val(void)
{
	struct oplus_nu1619_ic *chip = nu1619_chip;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: nu1619_chip not ready!\n", __func__);
		return -1;
	}

	if (chip->ext2_wireless_otg_en_gpio <= 0) {
		chg_err("ext2_wireless_otg_en_gpio not exist, return\n");
		return -1;
	}

	if (IS_ERR_OR_NULL(chip->pinctrl)
		|| IS_ERR_OR_NULL(chip->ext2_wireless_otg_en_active)
		|| IS_ERR_OR_NULL(chip->ext2_wireless_otg_en_sleep)
		|| IS_ERR_OR_NULL(chip->ext2_wireless_otg_en_default)) {
		chg_err("pinctrl null, return\n");
		return -1;
	}

	return gpio_get_value(chip->ext2_wireless_otg_en_gpio);
}

static int nu1619_vt_sleep_gpio_init(struct oplus_nu1619_ic *chip)
{
	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_nu1619_ic not ready!\n", __func__);
		return -EINVAL;
	}

	chip->pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->pinctrl)) {
		chg_err("get pinctrl fail\n");
		return -EINVAL;
	}

	chip->vt_sleep_active =
			pinctrl_lookup_state(chip->pinctrl, "vt_sleep_active");
	if (IS_ERR_OR_NULL(chip->vt_sleep_active)) {
		chg_err("get vt_sleep_active fail\n");
		return -EINVAL;
	}

	chip->vt_sleep_sleep =
			pinctrl_lookup_state(chip->pinctrl, "vt_sleep_sleep");
	if (IS_ERR_OR_NULL(chip->vt_sleep_sleep)) {
		chg_err("get vt_sleep_sleep fail\n");
		return -EINVAL;
	}

	chip->vt_sleep_default =
			pinctrl_lookup_state(chip->pinctrl, "vt_sleep_default");
	if (IS_ERR_OR_NULL(chip->vt_sleep_default)) {
		chg_err("get vt_sleep_default fail\n");
		return -EINVAL;
	}

	gpio_direction_output(chip->vt_sleep_gpio, 0);
	pinctrl_select_state(chip->pinctrl,
			chip->vt_sleep_sleep);

	return 0;
}

void nu1619_set_vt_sleep_val(int value)
{
	struct oplus_nu1619_ic *chip = nu1619_chip;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_nu1619_ic not ready!\n", __func__);
		return;
	}

	if (chip->vt_sleep_gpio <= 0) {
		chg_err("vt_sleep_gpio not exist, return\n");
		return;
	}

	if (IS_ERR_OR_NULL(chip->pinctrl)
		|| IS_ERR_OR_NULL(chip->vt_sleep_active)
		|| IS_ERR_OR_NULL(chip->vt_sleep_sleep)
		|| IS_ERR_OR_NULL(chip->vt_sleep_default)) {
		chg_err("pinctrl null, return\n");
		return;
	}

	if (value) {
		gpio_direction_output(chip->vt_sleep_gpio, 1);
		pinctrl_select_state(chip->pinctrl,
				chip->vt_sleep_default);
	} else {
		gpio_direction_output(chip->vt_sleep_gpio, 0);
		pinctrl_select_state(chip->pinctrl,
				chip->vt_sleep_sleep);
	}

	chg_err("<~WPC~>set value:%d, gpio_val:%d\n",
		value, gpio_get_value(chip->vt_sleep_gpio));
}

int nu1619_get_vt_sleep_val(void)
{
	struct oplus_nu1619_ic *chip = nu1619_chip;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: nu1619_chip not ready!\n", __func__);
		return -1;
	}

	if (chip->vt_sleep_gpio <= 0) {
		chg_err("vt_sleep_gpio not exist, return\n");
		return -1;
	}

	if (IS_ERR_OR_NULL(chip->pinctrl)
		|| IS_ERR_OR_NULL(chip->vt_sleep_active)
		|| IS_ERR_OR_NULL(chip->vt_sleep_sleep)
		|| IS_ERR_OR_NULL(chip->vt_sleep_default)) {
		chg_err("pinctrl null, return\n");
		return -1;
	}

	return gpio_get_value(chip->vt_sleep_gpio);
}

static int nu1619_vbat_en_gpio_init(struct oplus_nu1619_ic *chip)
{
	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_nu1619_ic not ready!\n", __func__);
		return -EINVAL;
	}

	chip->pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->pinctrl)) {
		chg_err("get pinctrl fail\n");
		return -EINVAL;
	}

	chip->vbat_en_active =
			pinctrl_lookup_state(chip->pinctrl, "vbat_en_active");
	if (IS_ERR_OR_NULL(chip->vbat_en_active)) {
		chg_err("get vbat_en_active fail\n");
		return -EINVAL;
	}

	chip->vbat_en_sleep =
			pinctrl_lookup_state(chip->pinctrl, "vbat_en_sleep");
	if (IS_ERR_OR_NULL(chip->vbat_en_sleep)) {
		chg_err("get vbat_en_sleep fail\n");
		return -EINVAL;
	}

	chip->vbat_en_default =
			pinctrl_lookup_state(chip->pinctrl, "vbat_en_default");
	if (IS_ERR_OR_NULL(chip->vbat_en_default)) {
		chg_err("get vbat_en_default fail\n");
		return -EINVAL;
	}

	gpio_direction_output(chip->vbat_en_gpio, 0);
	pinctrl_select_state(chip->pinctrl,
			chip->vbat_en_sleep);

	return 0;
}

void nu1619_set_vbat_en_val(int value)
{
	struct oplus_nu1619_ic *chip = nu1619_chip;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_nu1619_ic not ready!\n", __func__);
		return;
	}

	if (chip->vbat_en_gpio <= 0) {
		chg_err("vbat_en_gpio not exist, return\n");
		return;
	}

	if (IS_ERR_OR_NULL(chip->pinctrl)
		|| IS_ERR_OR_NULL(chip->vbat_en_active)
		|| IS_ERR_OR_NULL(chip->vbat_en_sleep)
		|| IS_ERR_OR_NULL(chip->vbat_en_default)) {
		chg_err("pinctrl null, return\n");
		return;
	}

	if (value) {
		gpio_direction_output(chip->vbat_en_gpio, 1);
		pinctrl_select_state(chip->pinctrl,
				chip->vbat_en_default);
	} else {
		gpio_direction_output(chip->vbat_en_gpio, 0);
		pinctrl_select_state(chip->pinctrl,
				chip->vbat_en_sleep);
	}

	chg_err("<~WPC~>set value:%d, gpio_val:%d\n",
		value, gpio_get_value(chip->vbat_en_gpio));
}

int nu1619_get_vbat_en_val(void)
{
	struct oplus_nu1619_ic *chip = nu1619_chip;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: nu1619_chip not ready!\n", __func__);
		return -1;
	}

	if (chip->vbat_en_gpio <= 0) {
		chg_err("vbat_en_gpio not exist, return\n");
		return -1;
	}

	if (IS_ERR_OR_NULL(chip->pinctrl)
		|| IS_ERR_OR_NULL(chip->vbat_en_active)
		|| IS_ERR_OR_NULL(chip->vbat_en_sleep)
		|| IS_ERR_OR_NULL(chip->vbat_en_default)) {
		chg_err("pinctrl null, return\n");
		return -1;
	}

	return gpio_get_value(chip->vbat_en_gpio);
}


static int nu1619_booster_en_gpio_init(struct oplus_nu1619_ic *chip)
{
	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_nu1619_ic not ready!\n", __func__);
		return -EINVAL;
	}

	chip->pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->pinctrl)) {
		chg_err("get pinctrl fail\n");
		return -EINVAL;
	}

	chip->booster_en_active =
			pinctrl_lookup_state(chip->pinctrl, "booster_en_active");
	if (IS_ERR_OR_NULL(chip->booster_en_active)) {
		chg_err("get booster_en_active fail\n");
		return -EINVAL;
	}

	chip->booster_en_sleep =
			pinctrl_lookup_state(chip->pinctrl, "booster_en_sleep");
	if (IS_ERR_OR_NULL(chip->booster_en_sleep)) {
		chg_err("get booster_en_sleep fail\n");
		return -EINVAL;
	}

	chip->booster_en_default =
			pinctrl_lookup_state(chip->pinctrl, "booster_en_default");
	if (IS_ERR_OR_NULL(chip->booster_en_default)) {
		chg_err("get booster_en_default fail\n");
		return -EINVAL;
	}

	gpio_direction_output(chip->booster_en_gpio, 0);
	pinctrl_select_state(chip->pinctrl,
			chip->booster_en_sleep);

	chg_err("gpio_val:%d\n", gpio_get_value(chip->booster_en_gpio));

	return 0;
}

void nu1619_set_booster_en_val(int value)
{
	struct oplus_nu1619_ic *chip = nu1619_chip;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_nu1619_ic not ready!\n", __func__);
		return;
	}

	if (chip->booster_en_gpio <= 0) {
		chg_err("booster_en_gpio not exist, return\n");
		return;
	}

	if (IS_ERR_OR_NULL(chip->pinctrl)
		|| IS_ERR_OR_NULL(chip->booster_en_active)
		|| IS_ERR_OR_NULL(chip->booster_en_sleep)
		|| IS_ERR_OR_NULL(chip->booster_en_default)) {
		chg_err("pinctrl null, return\n");
		return;
	}

	if (value) {
		gpio_direction_output(chip->booster_en_gpio, 1);
		pinctrl_select_state(chip->pinctrl,
				chip->booster_en_active);
	} else {
		gpio_direction_output(chip->booster_en_gpio, 0);
		pinctrl_select_state(chip->pinctrl,
				chip->booster_en_sleep);
	}

	chg_err("<~WPC~>set value:%d, gpio_val:%d\n",
		value, gpio_get_value(chip->booster_en_gpio));
}

int nu1619_get_booster_en_val(void)
{
	struct oplus_nu1619_ic *chip = nu1619_chip;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: nu1619_chip not ready!\n", __func__);
		return -1;
	}

	if (chip->booster_en_gpio <= 0) {
		chg_err("booster_en_gpio not exist, return\n");
		return -1;
	}

	if (IS_ERR_OR_NULL(chip->pinctrl)
		|| IS_ERR_OR_NULL(chip->booster_en_active)
		|| IS_ERR_OR_NULL(chip->booster_en_sleep)
		|| IS_ERR_OR_NULL(chip->booster_en_default)) {
		chg_err("pinctrl null, return\n");
		return -1;
	}

	return gpio_get_value(chip->booster_en_gpio);
}


void nu1619_set_cp_ldo_5v_val(int value)
{
	struct oplus_nu1619_ic *chip = nu1619_chip;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_nu1619_ic not ready!\n", __func__);
		return;
	}

	if (chip->cp_ldo_5v_gpio <= 0) {
		chg_err("cp_ldo_5v_gpio not exist, return\n");
		return;
	}

	if (IS_ERR_OR_NULL(chip->pinctrl)
		|| IS_ERR_OR_NULL(chip->cp_ldo_5v_active)
		|| IS_ERR_OR_NULL(chip->cp_ldo_5v_sleep)
		|| IS_ERR_OR_NULL(chip->cp_ldo_5v_default)) {
		chg_err("pinctrl null, return\n");
		return;
	}

	if (value) {
		gpio_direction_output(chip->cp_ldo_5v_gpio, 1);
		pinctrl_select_state(chip->pinctrl,
				chip->cp_ldo_5v_active);
	} else {
		gpio_direction_output(chip->cp_ldo_5v_gpio, 0);
		pinctrl_select_state(chip->pinctrl,
				chip->cp_ldo_5v_sleep);
	}

	chg_err("set value:%d, gpio_val:%d\n",
		value, gpio_get_value(chip->cp_ldo_5v_gpio));
}

int nu1619_get_cp_ldo_5v_val(void)
{
	struct oplus_nu1619_ic *chip = nu1619_chip;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: nu1619_chip not ready!\n", __func__);
		return -1;
	}

	if (chip->cp_ldo_5v_gpio <= 0) {
		chg_err("cp_ldo_5v_gpio not exist, return\n");
		return -1;
	}

	if (IS_ERR_OR_NULL(chip->pinctrl)
		|| IS_ERR_OR_NULL(chip->cp_ldo_5v_active)
		|| IS_ERR_OR_NULL(chip->cp_ldo_5v_sleep)
		|| IS_ERR_OR_NULL(chip->cp_ldo_5v_default)) {
		chg_err("pinctrl null, return\n");
		return -1;
	}

	return gpio_get_value(chip->cp_ldo_5v_gpio);
}

int nu1619_set_tx_cep_timeout_1500ms(void)
{
	struct oplus_nu1619_ic *chip = nu1619_chip;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: nu1619_chip not ready!\n", __func__);
		return -1;
	}

	if (chip->nu1619_chg_status.send_message == P9221_CMD_NULL) {
		chg_err("<~WPC~> nu1619_set_tx_cep_timeout_1.5s\n");
		chip->nu1619_chg_status.send_message = P9221_CMD_SET_CEP_TIMEOUT;
		chip->nu1619_chg_status.send_msg_cnt = 10;
		return 0;
	} else {
		chg_err("<~WPC~> busy, return!\n");
		return 1;
	}
}

static int nu1619_set_dock_led_pwm_pulse(int pwm_pulse)
{
	struct oplus_nu1619_ic *chip = nu1619_chip;

	return 0;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: nu1619_chip not ready!\n", __func__);
		return -1;
	}

	if (pwm_pulse > 100) {
		pwm_pulse = 100;
	} else if (pwm_pulse < 0) {
		pwm_pulse = 0;
	}

	if (chip->nu1619_chg_status.send_message == P9221_CMD_NULL) {
		chg_err("<~WPC~> set fan pwm pulse: %d\n", pwm_pulse);
		chip->nu1619_chg_status.dock_led_pwm_pulse = pwm_pulse;
		chip->nu1619_chg_status.send_message = P9221_CMD_SET_LED_BRIGHTNESS;
		chip->nu1619_chg_status.send_msg_cnt = 10;
		return 0;
	} else {
		chg_err("<~WPC~> busy, return!\n");
		return 1;
	}
}

static int nu1619_set_dock_fan_pwm_pulse(int pwm_pulse)
{
	struct oplus_nu1619_ic *chip = nu1619_chip;

	return 0;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: nu1619_chip not ready!\n", __func__);
		return -1;
	}

	if (pwm_pulse > 100) {
		pwm_pulse = 100;
	} else if (pwm_pulse < 0) {
		pwm_pulse = 0;
	}

	if (chip->nu1619_chg_status.send_message == P9221_CMD_NULL) {
		chg_err("<~WPC~> set fan pwm pulse: %d\n", pwm_pulse);
		chip->nu1619_chg_status.dock_fan_pwm_pulse = pwm_pulse;
		chip->nu1619_chg_status.send_message = P9221_CMD_SET_PWM_PULSE;
		chip->nu1619_chg_status.send_msg_cnt = 10;
		return 0;
	} else {
		chg_err("<~WPC~> busy, return!\n");
		return 1;
	}
}

static int nu1619_set_silent_mode(bool is_silent)
{
	struct oplus_nu1619_ic *chip = nu1619_chip;

	return 0;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: nu1619_chip not ready!\n", __func__);
		return -1;
	}

	if ((chip->nu1619_chg_status.send_message != P9221_CMD_NULL)
		&& (chip->nu1619_chg_status.send_message != P9221_CMD_SET_PWM_PULSE)) {
		return -1;
	}

	if (is_silent) {
		chg_err("<~WPC~> set fan pwm pulse: %d\n", FAN_PWM_PULSE_IN_SILENT_MODE);
		nu1619_set_dock_fan_pwm_pulse(FAN_PWM_PULSE_IN_SILENT_MODE);
		chip->nu1619_chg_status.work_silent_mode = true;
	} else {
		chg_err("<~WPC~> set fan pwm pulse: %d\n", FAN_PWM_PULSE_IN_FASTCHG_MODE);
		nu1619_set_dock_fan_pwm_pulse(FAN_PWM_PULSE_IN_FASTCHG_MODE);
		chip->nu1619_chg_status.work_silent_mode = false;
	}

	return 0;
}

static int nu1619_cp_ldo_5v_gpio_init(struct oplus_nu1619_ic *chip)
{
	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_nu1619_ic not ready!\n", __func__);
		return -EINVAL;
	}

	chip->pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->pinctrl)) {
		chg_err("get pinctrl fail\n");
		return -EINVAL;
	}

	chip->cp_ldo_5v_active =
			pinctrl_lookup_state(chip->pinctrl, "cp_ldo_5v_active");
	if (IS_ERR_OR_NULL(chip->cp_ldo_5v_active)) {
		chg_err("get cp_ldo_5v_active fail\n");
		return -EINVAL;
	}

	chip->cp_ldo_5v_sleep =
			pinctrl_lookup_state(chip->pinctrl, "cp_ldo_5v_sleep");
	if (IS_ERR_OR_NULL(chip->cp_ldo_5v_sleep)) {
		chg_err("get cp_ldo_5v_sleep fail\n");
		return -EINVAL;
	}

	chip->cp_ldo_5v_default =
			pinctrl_lookup_state(chip->pinctrl, "cp_ldo_5v_default");
	if (IS_ERR_OR_NULL(chip->cp_ldo_5v_default)) {
		chg_err("get cp_ldo_5v_default fail\n");
		return -EINVAL;
	}

	gpio_direction_output(chip->cp_ldo_5v_gpio, 0);
	pinctrl_select_state(chip->pinctrl,
			chip->cp_ldo_5v_sleep);

	chg_err("gpio_val:%d\n", gpio_get_value(chip->cp_ldo_5v_gpio));

	return 0;
}

static int nu1619_idt_gpio_init(struct oplus_nu1619_ic *chip)
{
	int rc = 0;
	struct device_node *node = chip->dev->of_node;
	pr_err("tongfeng test %s start\n", __func__);

	chip->idt_int_gpio = of_get_named_gpio(node, "qcom,idt_int-gpio", 0);
	if (chip->idt_int_gpio < 0) {
		pr_err("chip->idt_int_gpio not specified\n");
	} else {
		if (gpio_is_valid(chip->idt_int_gpio)) {
			rc = gpio_request(chip->idt_int_gpio, "idt-idt-gpio");
			if (rc) {
				pr_err("unable to request gpio [%d]\n", chip->idt_int_gpio);
			} else {
				rc = nu1619_idt_int_gpio_init(chip);
				if (rc) {
					chg_err("unable to init idt_int_gpio:%d\n", chip->idt_int_gpio);
				} else {
					nu1619_idt_int_irq_init(chip);
					nu1619_idt_int_eint_register(chip);
				}
			}
		}

		pr_err("chip->idt_int_gpio =%d\n", chip->idt_int_gpio);
	}

	chip->idt_con_gpio = of_get_named_gpio(node, "qcom,idt_connect-gpio", 0);
	if (chip->idt_con_gpio < 0) {
		pr_err("chip->idt_con_gpio not specified\n");
	} else {
		if (gpio_is_valid(chip->idt_con_gpio)) {
			rc = gpio_request(chip->idt_con_gpio, "idt-connect-gpio");
			if (rc) {
				pr_err("unable to request gpio [%d]\n", chip->idt_con_gpio);
			} else {
				rc = nu1619_idt_con_gpio_init(chip);
				if (rc) {
					chg_err("unable to init idt_con_gpio:%d\n", chip->idt_con_gpio);
				} else {
					nu1619_idt_con_irq_init(chip);
					nu1619_idt_con_eint_register(chip);
				}
			}
		}
		pr_err("chip->idt_con_gpio =%d\n", chip->idt_con_gpio);
	}

	chip->vt_sleep_gpio = of_get_named_gpio(node, "qcom,vt_sleep-gpio", 0);
	if (chip->vt_sleep_gpio < 0) {
		pr_err("chip->vt_sleep_gpio not specified\n");
	} else {
		if (gpio_is_valid(chip->vt_sleep_gpio)) {
			rc = gpio_request(chip->vt_sleep_gpio, "vt-sleep-gpio");
			if (rc) {
				pr_err("unable to request gpio [%d]\n", chip->vt_sleep_gpio);
			} else {
				rc = nu1619_vt_sleep_gpio_init(chip);
				if (rc) {
					chg_err("unable to init vt_sleep_gpio:%d\n", chip->vt_sleep_gpio);
				}
			}
		}
		pr_err("chip->vt_sleep_gpio =%d\n", chip->vt_sleep_gpio);
	}

	chip->vbat_en_gpio = of_get_named_gpio(node, "qcom,vbat_en-gpio", 0);
	if (chip->vbat_en_gpio < 0) {
		pr_err("chip->vbat_en_gpio not specified\n");
	} else {
		if (gpio_is_valid(chip->vbat_en_gpio)) {
			rc = gpio_request(chip->vbat_en_gpio, "vbat-en-gpio");
			if (rc) {
				pr_err("unable to request gpio [%d]\n", chip->vbat_en_gpio);
			} else {
				rc = nu1619_vbat_en_gpio_init(chip);
				if (rc) {
					chg_err("unable to init vbat_en_gpio:%d\n", chip->vbat_en_gpio);
				}
			}
		}
		pr_err("chip->vbat_en_gpio =%d\n", chip->vbat_en_gpio);
	}

	chip->ext1_wired_otg_en_gpio = of_get_named_gpio(node, "qcom,ext1_wired_otg_en-gpio", 0);
	if (chip->ext1_wired_otg_en_gpio < 0) {
		pr_err("chip->ext1_wired_otg_en_gpio not specified\n");
	} else {
		if (gpio_is_valid(chip->ext1_wired_otg_en_gpio)) {
			rc = gpio_request(chip->ext1_wired_otg_en_gpio, "ext1_wired_otg_en-gpio");
			if (rc) {
				pr_err("unable to request gpio [%d]\n", chip->ext1_wired_otg_en_gpio);
			} else {
				rc = nu1619_ext1_wired_otg_gpio_init(chip);
				if (rc) {
					chg_err("unable to init ext1_wired_otg_en_gpio:%d\n", chip->ext1_wired_otg_en_gpio);
				}
			}
		}
		pr_err("chip->ext1_wired_otg_en_gpio =%d\n", chip->ext1_wired_otg_en_gpio);
	}

	chip->ext2_wireless_otg_en_gpio = of_get_named_gpio(node, "qcom,ext2_wireless_otg_en-gpio", 0);
	if (chip->ext2_wireless_otg_en_gpio < 0) {
		pr_err("chip->ext2_wireless_otg_en_gpio not specified\n");
	} else {
		if (gpio_is_valid(chip->ext2_wireless_otg_en_gpio)) {
			rc = gpio_request(chip->ext2_wireless_otg_en_gpio, "ext2_wireless_otg_en-gpio");
			if (rc) {
				pr_err("unable to request gpio [%d]\n", chip->ext2_wireless_otg_en_gpio);
			} else {
				rc = nu1619_ext2_wireless_otg_gpio_init(chip);
				if (rc) {
					chg_err("unable to init ext2_wireless_otg_en_gpio:%d\n", chip->ext2_wireless_otg_en_gpio);
				}
			}
		}

		pr_err("chip->ext2_wireless_otg_en_gpio =%d\n", chip->ext2_wireless_otg_en_gpio);
	}

	chip->booster_en_gpio = of_get_named_gpio(node, "qcom,booster_en-gpio", 0);
	if (chip->booster_en_gpio < 0) {
		pr_err("chip->booster_en_gpio not specified\n");
	} else {
		if (gpio_is_valid(chip->booster_en_gpio)) {
			rc = gpio_request(chip->booster_en_gpio, "booster-en-gpio");
			if (rc) {
				pr_err("unable to request gpio [%d]\n", chip->booster_en_gpio);
			} else {
				rc = nu1619_booster_en_gpio_init(chip);
				if (rc) {
					chg_err("unable to init booster_en_gpio:%d\n", chip->booster_en_gpio);
				}
			}
		}
		pr_err("chip->booster_en_gpio =%d\n", chip->booster_en_gpio);
	}

	chip->cp_ldo_5v_gpio = of_get_named_gpio(node, "qcom,cp_ldo_5v-gpio", 0);
	if (chip->cp_ldo_5v_gpio < 0) {
		pr_err("chip->cp_ldo_5v_gpio not specified\n");
	} else {
		if (gpio_is_valid(chip->cp_ldo_5v_gpio)) {
			rc = gpio_request(chip->cp_ldo_5v_gpio, "cp-ldo-5v-gpio");
			if (rc) {
				pr_err("unable to request gpio [%d]\n", chip->cp_ldo_5v_gpio);
			} else {
				rc = nu1619_cp_ldo_5v_gpio_init(chip);
				if (rc) {
					chg_err("unable to init cp_ldo_5v_gpio:%d\n", chip->cp_ldo_5v_gpio);
				}
			}
		}

		pr_err("chip->cp_ldo_5v_gpio =%d\n", chip->cp_ldo_5v_gpio);
	}

	return rc;
}

#ifdef DEBUG_BY_FILE_OPS
static int nu1619_add = 0;
static ssize_t nu1619_reg_store(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	char write_data[32] = {0};
	char val_buf;
	int rc;

	if (copy_from_user(&write_data, buff, len)) {
		pr_err("nu1619_reg_store error.\n");
		return -EFAULT;
	}

	if (len >= ARRAY_SIZE(write_data)) {
		pr_err("data len error.\n");
		return -EFAULT;
	}
	write_data[len] = '\0';
	if (write_data[len - 1] == '\n') {
		write_data[len - 1] = '\0';
	}

	nu1619_add = (int)simple_strtoul(write_data, NULL, 0);

	pr_err("%s:received data=%s, nu1619_register address: 0x%02x\n", __func__, write_data, nu1619_add);

	rc = nu1619_read_reg(nu1619_chip, nu1619_add, &val_buf, 1);
	if (rc) {
		 chg_err("Couldn't read 0x%02x rc = %d\n", nu1619_add, rc);
	} else {
		 chg_err("nu1619_read 0x%02x = 0x%02x\n", nu1619_add, val_buf);
	}

	return len;
}

static ssize_t nu1619_reg_show(struct file *filp, char __user *buff, size_t count, loff_t *off)
{
	char page[256] = {0};
	char val_buf;
	int rc;
	int len = 0;

	rc = nu1619_read_reg(nu1619_chip, nu1619_add, &val_buf, 1);
	if (rc) {
		 chg_err("Couldn't read 0x%02x rc = %d\n", nu1619_add, rc);
	}

	len = sprintf(page, "reg = 0x%x, data = 0x%x\n", nu1619_add, val_buf);
	if (len > *off) {
		len -= *off;
	} else {
		len = 0;
	}

	if (copy_to_user(buff, page, (len < count ? len : count))) {
		return -EFAULT;
	}

	*off += len < count ? len : count;
	return (len < count ? len : count);
}

static const struct file_operations nu1619_add_log_proc_fops = {
	.write = nu1619_reg_store,
	.read = nu1619_reg_show,
};

static void init_nu1619_add_log(void)
{
	struct proc_dir_entry *p = NULL;

	p = proc_create("nu1619_add_log", 0664, NULL, &nu1619_add_log_proc_fops);
	if (!p) {
		pr_err("proc_create init_nu1619_add_log_proc_fops fail!\n");
	}
}

static ssize_t nu1619_data_log_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	char write_data[32] = {0};
	int critical_log = 0;
	int rc;

	if (copy_from_user(&write_data, buff, len)) {
		pr_err("bat_log_write error.\n");
		return -EFAULT;
	}

	if (len >= ARRAY_SIZE(write_data)) {
		pr_err("data len error.\n");
		return -EFAULT;
	}
	write_data[len] = '\0';
	if (write_data[len - 1] == '\n') {
		write_data[len - 1] = '\0';
	}

	critical_log = (int)simple_strtoul(write_data, NULL, 0);
	if (critical_log > 0xFF) {
		critical_log = 0xFF;
	}

	pr_err("%s:received data=%s, nu1619_data=%x\n", __func__, write_data, critical_log);

	rc = nu1619_config_interface(nu1619_chip, nu1619_add, critical_log, 0xFF);
	if (rc) {
		 chg_err("Couldn't write 0x%02x rc = %d\n", nu1619_add, rc);
	}

	return len;
}

static const struct file_operations nu1619_data_log_proc_fops = {
	.write = nu1619_data_log_write,
};

static void init_nu1619_data_log(void)
{
	struct proc_dir_entry *p = NULL;

	p = proc_create("nu1619_data_log", 0664, NULL, &nu1619_data_log_proc_fops);
	if (!p) {
		pr_err("proc_create init_nu1619_data_log_proc_fops fail!\n");
	}
}
#endif /*DEBUG_BY_FILE_OPS*/

static void nu1619_task_work_process(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_nu1619_ic *chip = container_of(dwork, struct oplus_nu1619_ic, nu1619_task_work);
	static int idt_disconnect_cnt = 0;

	chg_err("<~WPC~> in\n");

	if (nu1619_get_idt_con_val() == 0) {
		chg_err("<~WPC~> idt_connect == 0\n");

		idt_disconnect_cnt++;
		if (idt_disconnect_cnt >= 2) {
			if (nu1619_chip->nu1619_chg_status.charge_online) {
				chg_err("<~WPC~> idt_connect has dispeared. exit wpc\n");
				schedule_delayed_work(&nu1619_chip->idt_connect_int_work, 0);
			}
		}

		return;
	} else {
		idt_disconnect_cnt = 0;
	}

	if (nu1619_chip->nu1619_chg_status.charge_online) {
		oplus_wpc_get_fastchg_allow(chip);
		nu1619_charge_set_max_current_by_tbatt(chip);
		nu1619_charge_set_max_current_by_adapter_power(chip);
		nu1619_charge_set_max_current_by_led(chip);
		nu1619_charge_set_max_current_by_cool_down(chip);
		oplus_wpc_set_input_current(chip);
		if (nu1619_get_vrect_iout(chip) != 0) {
			chg_err("<~WPC~> self reset: because of vout jump\n");
			nu1619_self_reset(chip, false);
		}
		nu1619_TX_message_process(chip);
		nu1619_charge_status_process(chip);

		/* run again after interval */
		switch(chip->nu1619_chg_status.charge_status) {
		case WPC_CHG_STATUS_FAST_CHARGING_FROM_CHGPUMP:
		case WPC_CHG_STATUS_INCREASE_CURRENT_FOR_CHARGER:
		case WPC_CHG_STATUS_FAST_CHARGING_FROM_CHARGER:
		case WPC_CHG_STATUS_FTM_WORKING:
		case WPC_CHG_STATUS_DECREASE_VOUT_FOR_RESTART:
			schedule_delayed_work(&chip->nu1619_task_work, round_jiffies_relative(msecs_to_jiffies(500)));
			break;
		default:
			schedule_delayed_work(&chip->nu1619_task_work, round_jiffies_relative(msecs_to_jiffies(100)));
			break;
		}
	}

	chg_err("<~WPC~> out\n");
}

static void nu1619_CEP_work_process(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_nu1619_ic *chip = container_of(dwork, struct oplus_nu1619_ic, nu1619_CEP_work);

	if (nu1619_chip->nu1619_chg_status.charge_online) {
		nu1619_detect_CEP(chip);
		schedule_delayed_work(&chip->nu1619_CEP_work, P922X_CEP_INTERVAL);
	}
}

static void nu1619_update_work_process(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_nu1619_ic *chip = container_of(dwork, struct oplus_nu1619_ic, nu1619_update_work);
	int rc;
	int boot_mode = get_boot_mode();

	if (boot_mode == MSM_BOOT_MODE__FACTORY) {
		chg_err("<IDT UPDATE> MSM_BOOT_MODE__FACTORY do not update\n");
		return;
	}
	chg_err("<IDT UPDATE> nu1619_update_work_process\n");

	if (!chip->nu1619_chg_status.check_fw_update) {
		rc = nu1619_check_idt_fw_update(chip);
	}
}

static void nu1619_self_reset_process(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_nu1619_ic *chip = container_of(dwork, struct oplus_nu1619_ic, nu1619_self_reset_work);

	if (nu1619_get_vt_sleep_val() == 1) {
		chg_err("<~WPC~> self reset: enable connect again!\n");
		nu1619_set_vt_sleep_val(0);
		if (chip->nu1619_chg_status.wpc_self_reset) {
			schedule_delayed_work(&chip->nu1619_self_reset_work, round_jiffies_relative(msecs_to_jiffies(4500)));
		}
	} else {
		chg_err("<~WPC~> self reset: clear wpc_self_reset!\n");
		chip->nu1619_chg_status.wpc_self_reset = false;
	}
}

#ifdef FASTCHG_TEST_BY_TIME
static void nu1619_test_work_process(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_nu1619_ic *chip = container_of(dwork, struct oplus_nu1619_ic, nu1619_test_work);

	if (chip->disable_charge == true) {
		chg_err("<~WPC~>[-TEST-] chip->disable_charge==true, keep disable charge\n");
		stop_timer = 0;
		return;
	}

	stop_timer++;
	chg_err("<~WPC~>[-TEST-] test stop timer: %d\n", stop_timer);
	if (stop_timer >= 420) {
		chg_err("<~WPC~>[-TEST-] enable charge\n");
		nu1619_set_vt_sleep_val(0);
	} else {
		chg_err("<~WPC~>[-TEST-] keep stop\n");
		schedule_delayed_work(&chip->nu1619_test_work, round_jiffies_relative(msecs_to_jiffies(1000)));
	}
}
#endif

static void wlchg_reset_variables(struct oplus_nu1619_ic *chip)
{
	struct wpc_data *chg_status = &chip->nu1619_chg_status;
	/*struct charge_param *chg_param = &chip->chg_param;*/

	/*chip->pmic_high_vol = false;*/

	chg_status->charge_status = WPC_CHG_STATUS_DEFAULT;
	chg_status->fastchg_startup_step = FASTCHG_EN_CHGPUMP1_STEP;
	chg_status->charge_online = false;
	chg_status->tx_online = false;
	chg_status->tx_present = false;
	chg_status->charge_voltage = 0;
	chg_status->charge_current = 0;
	chg_status->temp_region = WLCHG_TEMP_REGION_MAX;
	chg_status->wpc_dischg_status = WPC_DISCHG_STATUS_OFF;
	chg_status->fastchg_ing = false;
	chg_status->max_current = FASTCHG_CURR_MAX_UA / 1000;
	chg_status->target_curr = WPC_CHARGE_CURRENT_DEFAULT;
	chg_status->target_vol = WPC_CHARGE_VOLTAGE_DEFAULT;
	chg_status->vol_set = WPC_CHARGE_VOLTAGE_DEFAULT;
	chg_status->curr_limit_mode = false;
	chg_status->vol_set_ok = true;
	chg_status->curr_set_ok = true;
	chg_status->fastchg_mode = false;
	chg_status->startup_fast_chg = false;
	chg_status->cep_err_flag = false;
	chg_status->cep_exit_flag = false;
	chg_status->ffc_check = false;
	chg_status->curr_need_dec = false;
	chg_status->vol_not_step = false;
	chg_status->is_power_changed = false;
	chg_status->vbat_too_high = false;
	chg_status->freq_threshold = 130;
	chg_status->freq_check_count = 0;
	chg_status->freq_thr_inc = false;
	chg_status->is_deviation = false;
	chg_status->deviation_check_done = false;
/*
	chg_param->mBattTempBoundT0 = chg_param->BATT_TEMP_T0;
	chg_param->mBattTempBoundT1 = chg_param->BATT_TEMP_T1;
	chg_param->mBattTempBoundT2 = chg_param->BATT_TEMP_T2;
	chg_param->mBattTempBoundT3 = chg_param->BATT_TEMP_T3;
	chg_param->mBattTempBoundT4 = chg_param->BATT_TEMP_T4;
	chg_param->mBattTempBoundT5 = chg_param->BATT_TEMP_T5;
	chg_param->mBattTempBoundT6 = chg_param->BATT_TEMP_T6;
*/
	if (nu1619_chip != NULL)
		nu1619_reset_variables(nu1619_chip);
}

static ssize_t proc_wireless_voltage_rect_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	uint8_t ret = 0;
	char page[10];
	int vrect = 0;
	struct oplus_nu1619_ic *chip = nu1619_chip;

	if(chip == NULL) {
		chg_err("%s: nu1619 driver is not ready\n", __func__);
		return 0;
	}
	if (atomic_read(&chip->suspended) == 1) {
		return 0;
	}

	vrect = chip->nu1619_chg_status.vrect;

	chg_err("%s: vrect = %d.\n", __func__, vrect);
	sprintf(page, "%d", vrect);
	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

	return ret;
}

static ssize_t proc_wireless_voltage_rect_write(struct file *file, const char __user *buf, size_t count, loff_t *lo)
{
	return count;
}

static const struct file_operations proc_wireless_voltage_rect_ops =
{
	.read = proc_wireless_voltage_rect_read,
	.write  = proc_wireless_voltage_rect_write,
	.open  = simple_open,
	.owner = THIS_MODULE,
};

static ssize_t proc_wireless_current_out_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	uint8_t ret = 0;
	char page[10];
	int iout = 0;
	struct oplus_nu1619_ic *chip = nu1619_chip;

	if(chip == NULL) {
		chg_err("%s: nu1619 driver is not ready\n", __func__);
		return 0;
	}
	if (atomic_read(&chip->suspended) == 1) {
		return 0;
	}

	iout = chip->nu1619_chg_status.iout;

	chg_err("%s: iout = %d.\n", __func__, iout);
	sprintf(page, "%d", iout);
	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

	return ret;
}

static ssize_t proc_wireless_current_out_write(struct file *file, const char __user *buf, size_t count, loff_t *lo)
{
#ifdef DEBUG_FASTCHG_BY_ADB
	char cur_string[8] = {0};
	int cur = 0;
	int len = count < 8 ? count : 8;

	if (nu1619_chip == NULL) {
		chg_err("%s: nu1619_chip is not ready\n", __func__);
		return -ENODEV;
	}

	copy_from_user(cur_string, buf, len);
	kstrtoint(cur_string, 0, &cur);
	chg_err("set current: cur_string = %s, cur = %d.", cur_string, cur);
	nu1619_chip->nu1619_chg_status.iout_stated_current = cur;

	nu1619_chip->nu1619_chg_status.vout_debug_mode = false;
	nu1619_chip->nu1619_chg_status.iout_debug_mode = true;
#endif

	return count;
}

static const struct file_operations proc_wireless_current_out_ops =
{
	.read = proc_wireless_current_out_read,
	.write  = proc_wireless_current_out_write,
	.open  = simple_open,
	.owner = THIS_MODULE,
};


static ssize_t proc_wireless_ftm_mode_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	return count;
}

static ssize_t proc_wireless_ftm_mode_write(struct file *file, const char __user *buf, size_t len, loff_t *lo)
{
	char buffer[2] = {0};

	chg_err("%s: len[%d] start.\n", __func__, len);
	if (len > 2) {
		return -EFAULT;
	}

	if (copy_from_user(buffer, buf, 2)) {
		chg_err("%s:  error.\n", __func__);
		return -EFAULT;
	}

	chg_err("%s: buffer[%s] .\n", __func__, buffer);
	if (buffer[0] == '0') {
		chg_err("%s:ftm_mode write 0.\n", __func__);
		nu1619_enable_ftm(false);
	} else {
		chg_err("%s:ftm_mode write 1.\n", __func__);
		nu1619_enable_ftm(true);
	}
	chg_err("%s: end.\n", __func__);

	return len;
}

static const struct file_operations proc_wireless_ftm_mode_ops =
{
	.read = proc_wireless_ftm_mode_read,
	.write  = proc_wireless_ftm_mode_write,
	.open  = simple_open,
	.owner = THIS_MODULE,
};

static ssize_t proc_wireless_rx_voltage_read(struct file *file,
					     char __user *buf, size_t count,
					     loff_t *ppos)
{
	char vol_string[8];
	int len = 0;
	len = snprintf(vol_string, 8, "%d",
		       nu1619_chip->nu1619_chg_status.charge_voltage);

	copy_to_user(buf, vol_string, len);

	return 0;
}
static ssize_t proc_wireless_rx_voltage_write(struct file *file,
					      const char __user *buf,
					      size_t count, loff_t *lo)
{
	char vol_string[8] = {0};
	int vol = 0;
	int len = count < 8 ? count : 8;

	if (nu1619_chip == NULL) {
		chg_err("%s: nu1619_chip is not ready\n", __func__);
		return -ENODEV;
	}

	copy_from_user(vol_string, buf, len);
	kstrtoint(vol_string, 0, &vol);
	chg_err("set voltage: vol_string = %s, vol = %d.", vol_string, vol);
	nu1619_set_rx_charge_voltage(nu1619_chip, vol);

#ifdef DEBUG_FASTCHG_BY_ADB
	nu1619_chip->nu1619_chg_status.vout_debug_mode = true;
	nu1619_chip->nu1619_chg_status.iout_debug_mode = false;
#endif

	return count;
}

static const struct file_operations proc_wireless_rx_voltage = {
	.read = proc_wireless_rx_voltage_read,
	.write = proc_wireless_rx_voltage_write,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static ssize_t proc_wireless_tx_read(struct file *file, char __user *buf,
				     size_t count, loff_t *ppos)
{
	uint8_t ret = 0;
	char page[10];
	struct oplus_nu1619_ic *chip = nu1619_chip;

	if (chip == NULL) {
		chg_err("%s: nu1619 driver is not ready\n", __func__);
		return -ENODEV;
	}

	if (chip->wireless_mode == WIRELESS_MODE_TX) {
		if (chip->nu1619_chg_status.tx_online)
			snprintf(page, 10, "%s\n", "charging");
		else
			snprintf(page, 10, "%s\n", "enable");
	} else
		snprintf(page, 10, "%s\n", "disable");

	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));
	return ret;
}

static ssize_t proc_wireless_tx_write(struct file *file, const char __user *buf,
				      size_t count, loff_t *lo)
{
	char buffer[5] = { 0 };
	struct oplus_nu1619_ic *chip = nu1619_chip;
	int val;

	if (chip == NULL) {
		chg_err("%s: nu1619 driver is not ready\n", __func__);
		return -ENODEV;
	}

	if (count > 5) {
		return -EFAULT;
	}

	if (copy_from_user(buffer, buf, count)) {
		chg_err("%s: error.\n", __func__);
		return -EFAULT;
	}

	if (chip->nu1619_chg_status.charge_online == true) {
		chg_err("<~WPC~> charge_online is true, can't set rtx function, return!\n");
		return -EBUSY;
	}

	chg_err("buffer=%s", buffer);
	kstrtoint(buffer, 0, &val);
	chg_err("val = %d", val);

	if (val == 1) {
		wlchg_reset_variables(chip);
		nu1619_set_rtx_function(true);
	} else {
		nu1619_set_rtx_function(false);
	}
	return count;
}

static const struct file_operations proc_wireless_tx_ops = {
	.read = proc_wireless_tx_read,
	.write = proc_wireless_tx_write,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static ssize_t proc_wireless_epp_read(struct file *file, char __user *buf,
				      size_t count, loff_t *ppos)
{
	uint8_t ret = 0;
#ifdef oplus_wireless
	char page[6];
	struct oplus_nu1619_ic *chip = nu1619_chip;
	size_t len = 6;

	if (chip == NULL) {
		chg_err("%s: nu1619 driver is not ready\n", __func__);
		return 0;
	}

	memset(page, 0, 6);
	if (force_epp) {
		len = snprintf(page, len, "epp\n");
	} else if (force_bpp) {
		len = snprintf(page, len, "bpp\n");
	} else if (!auto_mode) {
		len = snprintf(page, len, "manu\n");
	} else {
		len = snprintf(page, len, "auto\n");
	}
	ret = simple_read_from_buffer(buf, count, ppos, page, len);
#endif
	return ret;
}

static ssize_t proc_wireless_epp_write(struct file *file,
				       const char __user *buf, size_t count,
				       loff_t *lo)
{
#ifdef oplus_wireless
	char buffer[5] = { 0 };
	int val = 0;

	chg_err("%s: len[%d] start.\n", __func__, count);
	if (count > 5) {
		return -EFAULT;
	}

	if (copy_from_user(buffer, buf, count)) {
		chg_err("%s: error.\n", __func__);
		return -EFAULT;
	}
	chg_err("buffer=%s", buffer);
	kstrtoint(buffer, 0, &val);
	chg_err("val=%d", val);
	if (val == 1) {
		force_bpp = true;
		force_epp = false;
		auto_mode = true;
	} else if (val == 2) {
		force_bpp = false;
		force_epp = true;
		auto_mode = true;
	} else if (val == 3) {
		force_bpp = false;
		force_epp = false;
		auto_mode = false;
	} else {
		force_bpp = false;
		force_epp = false;
		auto_mode = true;
	}
#endif
	return count;
}

static const struct file_operations proc_wireless_epp_ops = {
	.read = proc_wireless_epp_read,
	.write = proc_wireless_epp_write,
	.open = simple_open,
	.owner = THIS_MODULE,
};
static int proc_charge_pump_status;
static ssize_t proc_wireless_charge_pump_read(struct file *file, char __user *buf,
					   size_t count, loff_t *ppos)
{
	uint8_t ret = 0;
	char page[6];
	struct oplus_nu1619_ic *chip = nu1619_chip;
	size_t len = 6;

	if (chip == NULL) {
		chg_err("%s: nu1619 driver is not ready\n", __func__);
		return 0;
	}

	memset(page, 0, 6);
	len = snprintf(page, len, "%d\n", proc_charge_pump_status);
	ret = simple_read_from_buffer(buf, count, ppos, page, len);
	return ret;
}

static ssize_t proc_wireless_charge_pump_write(struct file *file,
					       const char __user *buf,
					       size_t count, loff_t *lo)
{
	char buffer[2] = { 0 };
	int val = 0;

	chg_err("%s: len[%d] start.\n", __func__, count);
	if (count > 2) {
		return -EFAULT;
	}

	if (copy_from_user(buffer, buf, count)) {
		chg_err("%s: error.\n", __func__);
		return -EFAULT;
	}
	chg_err("buffer=%s", buffer);
	val = buffer[0] - '0';
	chg_err("val=%d", val);
	if (val < 0 || val > 6) {
		return -EINVAL;
	}
	switch (val) {
	case 0:
		chg_err("wkcs: disable all charge pump\n");
		chargepump_disable();
		/*bq2597x_enable_charge_pump(false);*/
		break;
	case 1:
		chg_err("wkcs: disable charge pump 1\n");
		chargepump_disable();
		break;
	case 2:
		chg_err("wkcs: enable charge pump 1\n");
		chargepump_set_for_EPP(); /*enable chargepump*/
		chargepump_enable();
		chargepump_set_for_LDO();
		break;
	case 3:
		chg_err("wkcs: disable charge pump 2\n");
		/*bq2597x_enable_charge_pump(false);*/
		break;
	case 4:
		chg_err("wkcs: enable charge pump 2\n");
		/*bq2597x_enable_charge_pump(true);*/
		break;
	case 5:
		nu1619_set_rx_charge_current(nu1619_chip, 0);
		/*pmic_high_vol_en(g_op_chip, false);*/
		break;
	case 6:
		/*pmic_high_vol_en(g_op_chip, true);*/
		nu1619_set_rx_charge_current(nu1619_chip, 300);
		break;
	default:
		chg_err("wkcs: invalid value.");
		break;
	}
	proc_charge_pump_status = val;
	return count;
}

static const struct file_operations proc_wireless_charge_pump_ops = {
	.read = proc_wireless_charge_pump_read,
	.write = proc_wireless_charge_pump_write,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static ssize_t proc_wireless_bat_mult_read(struct file *file, char __user *buf,
					   size_t count, loff_t *ppos)
{
	uint8_t ret = 0;
	char page[6];
	struct oplus_nu1619_ic *chip = nu1619_chip;
	size_t len = 6;

	if (chip == NULL) {
		chg_err("%s: nu1619 driver is not ready\n", __func__);
		return 0;
	}

	memset(page, 0, 6);
	/*snprintf(page, len, "%d\n", test_bat_val);*/
	ret = simple_read_from_buffer(buf, count, ppos, page, len);
	return ret;
}

static ssize_t proc_wireless_bat_mult_write(struct file *file,
					    const char __user *buf,
					    size_t count, loff_t *lo)
{
#ifdef oplus_wireless
	char buffer[5] = { 0 };
	int val = 0;

	chg_err("%s: len[%d] start.\n", __func__, count);
	if (count > 5) {
		return -EFAULT;
	}

	if (copy_from_user(buffer, buf, count)) {
		chg_err("%s: error.\n", __func__);
		return -EFAULT;
	}
	chg_err("buffer=%s", buffer);
	kstrtoint(buffer, 0, &val);
	chg_err("val=%d", val);
	test_bat_val = val;
#endif
	return count;
}

static const struct file_operations proc_wireless_bat_mult_ops = {
	.read = proc_wireless_bat_mult_read,
	.write = proc_wireless_bat_mult_write,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static ssize_t proc_wireless_deviated_read(struct file *file, char __user *buf,
					   size_t count, loff_t *ppos)
{
	uint8_t ret = 0;
	char page[7];
	struct oplus_nu1619_ic *chip = nu1619_chip;
	size_t len = 7;

	if (chip == NULL) {
		chg_err("%s: nu1619 driver is not ready\n", __func__);
		return 0;
	}

	memset(page, 0, 7);
	if (chip->nu1619_chg_status.is_deviation) {
		len = snprintf(page, len, "%s\n", "true");
	} else {
		len = snprintf(page, len, "%s\n", "false");
	}
	ret = simple_read_from_buffer(buf, count, ppos, page, len);
	return ret;
}

static const struct file_operations proc_wireless_deviated_ops = {
	.read = proc_wireless_deviated_read,
	.write = NULL,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static ssize_t proc_wireless_rx_read(struct file *file, char __user *buf,
					    size_t count, loff_t *ppos)
{
	uint8_t ret = 0;
	char page[3];
	struct oplus_nu1619_ic *chip = nu1619_chip;

	if (chip == NULL) {
		chg_err("<~WPC~> nu1619_chip is NULL!\n");
		return -ENODEV;
	}


	memset(page, 0, 3);
	snprintf(page, 3, "%c\n", !chip->disable_charge ? '1' : '0');
	ret = simple_read_from_buffer(buf, count, ppos, page, 3);
	return ret;
}

static ssize_t proc_wireless_rx_write(struct file *file, const char __user *buf,
				      size_t count, loff_t *lo)
{
	char buffer[5] = { 0 };
	struct oplus_nu1619_ic *chip = nu1619_chip;
	int val;

	if (chip == NULL) {
		chg_err("%s: nu1619 driver is not ready\n", __func__);
		return -ENODEV;
	}

	if (count > 5) {
		return -EFAULT;
	}

	if (copy_from_user(buffer, buf, count)) {
		chg_err("%s: error.\n", __func__);
		return -EFAULT;
	}

	chg_err("buffer=%s", buffer);
	kstrtoint(buffer, 0, &val);
	chg_err("val = %d", val);

	if (val == 0) {
		chg_err("<~WPC~>[-TEST-] chip->disable_charge = true\n");
		chip->disable_charge = true;
		oplus_dcin_irq_enable(false);
		nu1619_set_vt_sleep_val(1);
	} else {
		chg_err("<~WPC~>[-TEST-] chip->disable_charge = false\n");
		chip->disable_charge = false;
		oplus_dcin_irq_enable(true);
		nu1619_set_vt_sleep_val(0);
	}
	return count;
}

static const struct file_operations proc_wireless_rx_ops = {
	.read = proc_wireless_rx_read,
	.write = proc_wireless_rx_write,
	.open = simple_open,
	.owner = THIS_MODULE,
};

#define UPGRADE_START 0
#define UPGRADE_FW    1
#define UPGRADE_END   2
struct idt_fw_head {
	u8 magic[4];
	int size;
};
static ssize_t proc_wireless_upgrade_firmware_write(struct file *file,
					      const char __user *buf,
					      size_t count, loff_t *lo)
{
#ifdef oplus_wireless
	u8 temp_buf[sizeof(struct idt_fw_head)];
	int rc = 0;
	static u8 *fw_buf;
	static int upgrade_step = UPGRADE_START;
	static int fw_index;
	static int fw_size;
	struct idt_fw_head *fw_head;

	if (nu1619_chip == NULL) {
		chg_err("<IDT UPDATE>nu1619_chip not't ready\n");
		return -ENODEV;
	}

start:
	switch (upgrade_step) {
	case UPGRADE_START:
		if (count < sizeof(struct idt_fw_head)) {
			chg_err("<IDT UPDATE>image format error\n");
			return -EINVAL;
		}
		memset(temp_buf, 0, sizeof(struct idt_fw_head));
		copy_from_user(temp_buf, buf, sizeof(struct idt_fw_head));
		fw_head = (struct idt_fw_head *)temp_buf;
		if (fw_head->magic[0] == 0x02 && fw_head->magic[1] == 0x00 &&
		    fw_head->magic[2] == 0x03 && fw_head->magic[3] == 0x00) {
			fw_size = fw_head->size;
			fw_buf = kzalloc(fw_size, GFP_KERNEL);
			if (fw_buf == NULL) {
				chg_err("<IDT UPDATE>alloc fw_buf err\n");
				return -ENOMEM;
			}
			chg_err("<IDT UPDATE>image header verification succeeded, fw_size=%d\n", fw_size);
			copy_from_user(fw_buf, buf + sizeof(struct idt_fw_head), count - sizeof(struct idt_fw_head));
			fw_index = count - sizeof(struct idt_fw_head);
			chg_err("<IDT UPDATE>Receiving image, fw_size=%d, fw_index=%d\n", fw_size, fw_index);
			if (fw_index >= fw_size) {
				upgrade_step = UPGRADE_END;
				goto start;
			} else {
				upgrade_step = UPGRADE_FW;
			}
		} else {
			chg_err("<IDT UPDATE>image format error\n");
			return -EINVAL;
		}
		break;
	case UPGRADE_FW:
		copy_from_user(fw_buf + fw_index, buf, count);
		fw_index += count;
		chg_err("<IDT UPDATE>Receiving image, fw_size=%d, fw_index=%d\n", fw_size, fw_index);
		if (fw_index >= fw_size) {
			upgrade_step = UPGRADE_END;
			goto start;
		}
		break;
	case UPGRADE_END:
		rc = nu1619_upgrade_firmware(nu1619_chip, fw_buf, fw_size);
		kfree(fw_buf);
		fw_buf = NULL;
		upgrade_step = UPGRADE_START;
		if (rc < 0)
			return rc;
		break;
	default:
		upgrade_step = UPGRADE_START;
		chg_err("<IDT UPDATE>status error\n");
		if (fw_buf != NULL) {
			kfree(fw_buf);
			fw_buf = NULL;
		}
		break;
	}
#endif
	return count;
}

static const struct file_operations proc_upgrade_firmware_ops = {
	.read = NULL,
	.write = proc_wireless_upgrade_firmware_write,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static ssize_t proc_wireless_rx_freq_read(struct file *file,
					  char __user *buf, size_t count,
					  loff_t *ppos)
{
	int rc;
	char string[8];
	int len = 0;
	struct oplus_nu1619_ic *chip = nu1619_chip;

	if (chip == NULL) {
		chg_err("nu1619 driver is not ready\n");
		return -ENODEV;
	}

	memset(string, 0, 8);

	len = snprintf(string, 8, "%d\n", chip->nu1619_chg_status.freq_threshold);
	rc = simple_read_from_buffer(buf, count, ppos, string, len);

	return rc;
}
static ssize_t proc_wireless_rx_freq_write(struct file *file,
					   const char __user *buf,
					   size_t count, loff_t *lo)
{
	char string[16];
	int freq = 0;
	struct oplus_nu1619_ic *chip = nu1619_chip;

	if (chip == NULL) {
		chg_err("nu1619 driver is not ready\n");
		return -ENODEV;
	}

	memset(string, 0, 16);
	copy_from_user(string, buf, count);
	chg_err("buf = %s, len = %d\n", string, count);
	kstrtoint(string, 0, &freq);
	chg_err("set freq threshold to %d\n", freq);
	chip->nu1619_chg_status.freq_threshold = freq;

	return count;
}

static const struct file_operations proc_wireless_rx_freq_ops = {
	.read = proc_wireless_rx_freq_read,
	.write = proc_wireless_rx_freq_write,
	.open = simple_open,
	.owner = THIS_MODULE,
};

#ifdef HW_TEST_EDITION
static ssize_t proc_wireless_w30w_time_read(struct file *file, char __user *buf,
					    size_t count, loff_t *ppos)
{
	uint8_t ret = 0;
#ifdef oplus_wireless
	char page[32];
	struct op_chg_chip *chip = g_op_chip;

	if (chip == NULL) {
		chg_err("%s: nu1619 driver is not ready\n", __func__);
		return 0;
	}

	snprintf(page, 32, "w30w_time:%d minutes\n", chip->w30w_time);
	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));
#endif
	return ret;
}

static ssize_t proc_wireless_w30w_time_write(struct file *file,
					     const char __user *buf,
					     size_t count, loff_t *lo)
{
#ifdef oplus_wireless
	char buffer[4] = { 0 };
	int timeminutes = 0;
	struct op_chg_chip *chip = g_op_chip;

	if (chip == NULL) {
		chg_err("%s: nu1619 driver is not ready\n", __func__);
		return 0;
	}

	chg_err("%s: len[%d] start.\n", __func__, count);
	if (count > 3) {
		return -EFAULT;
	}

	if (copy_from_user(buffer, buf, count)) {
		chg_err("%s: error.\n", __func__);
		return -EFAULT;
	}
	chg_err("buffer=%s", buffer);
	kstrtoint(buffer, 0, &timeminutes);
	chg_err("set w30w_time = %dm", timeminutes);
	if (timeminutes >= 0 && timeminutes <= 60)
		chip->w30w_time = timeminutes;
	chip->w30w_work_started = false;
	chip->w30w_timeout = false;
#endif
	return count;
}

static const struct file_operations proc_wireless_w30w_time_ops = {
	.read = proc_wireless_w30w_time_read,
	.write = proc_wireless_w30w_time_write,
	.open = simple_open,
	.owner = THIS_MODULE,
};

#endif

static ssize_t proc_wireless_user_sleep_mode_read(struct file *file, char __user *buf,
					    size_t count, loff_t *ppos)
{
	uint8_t ret = 0;
	char page[10];
	struct oplus_nu1619_ic *chip = nu1619_chip;

	if (chip == NULL) {
		chg_err("nu1619_chip driver is not ready\n");
		return 0;
	}

	chg_err("chip->quiet_mode_need = %d.\n", chip->quiet_mode_need);
	sprintf(page, "%d", chip->quiet_mode_need);
	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

	return ret;
}

#define FASTCHG_MODE		0
#define SILENT_MODE			1
#define BATTERY_FULL_MODE	2
#define CALL_MODE			598
#define EXIT_CALL_MODE		599
static ssize_t proc_wireless_user_sleep_mode_write(struct file *file, const char __user *buf,
				      size_t len, loff_t *lo)
{
	char buffer[4] = {0};
	int pmw_pulse = 0;
	int rc = -1;
	struct oplus_nu1619_ic *chip = nu1619_chip;

	if (chip == NULL) {
		chg_err("nu1619 driver is not ready\n");
		return 0;
	}

	if (len > 4) {
		chg_err("len[%d] -EFAULT\n", len);
		return -EFAULT;
	}

	if (copy_from_user(buffer, buf, len)) {
		chg_err("copy from user error\n");
		return -EFAULT;
	}

	chg_err("user mode: buffer=%s\n", buffer);
	kstrtoint(buffer, 0, &pmw_pulse);
	if (pmw_pulse == FASTCHG_MODE) {
		rc = nu1619_set_silent_mode(false);
		if (rc == 0)
			chip->quiet_mode_need = FASTCHG_MODE;
		chg_err("<~WPC~> set user mode: %d, fastchg mode, rc: %d\n", pmw_pulse, rc);
	} else if (pmw_pulse == SILENT_MODE) {
		rc = nu1619_set_silent_mode(true);
		if (rc == 0)
			chip->quiet_mode_need = SILENT_MODE;
		chg_err("<~WPC~>set user mode: %d, silent mode, rc: %d\n", pmw_pulse, rc);
		nu1619_set_dock_led_pwm_pulse(3);
	} else if (pmw_pulse == BATTERY_FULL_MODE) {
		chg_err("<~WPC~> set user mode: %d, battery full mode\n", pmw_pulse);
		chip->quiet_mode_need = BATTERY_FULL_MODE;
		nu1619_set_dock_fan_pwm_pulse(60);
	} else if (pmw_pulse == CALL_MODE) {
		chg_err("<~WPC~> set user mode: %d, call mode\n", pmw_pulse);
		chip->nu1619_chg_status.call_mode = true;
	} else if (pmw_pulse == EXIT_CALL_MODE) {
		chip->nu1619_chg_status.call_mode = false;
		chg_err("<~WPC~> set user mode: %d, exit call mode\n", pmw_pulse);
	} else {
		chg_err("<~WPC~> user sleep mode: pmw_pulse: %d\n", pmw_pulse);
		chip->quiet_mode_need = pmw_pulse;
		nu1619_set_dock_fan_pwm_pulse(pmw_pulse);
	}

	return len;
}

static const struct file_operations proc_wireless_user_sleep_mode_ops = {
	.read = proc_wireless_user_sleep_mode_read,
	.write = proc_wireless_user_sleep_mode_write,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static ssize_t proc_wireless_idt_adc_test_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	uint8_t ret = 0;
	char page[10];
	int idt_adc_result = 0;
	struct oplus_nu1619_ic *chip = nu1619_chip;

	if (chip == NULL) {
		chg_err("%s: nu1619_chip driver is not ready\n", __func__);
		return 0;
	}

	if (chip->nu1619_chg_status.idt_adc_test_result == true) {
		idt_adc_result = 1;
	} else {
		idt_adc_result = 0;
	}
	chg_err("<~WPC~>idt_adc_test: result %d.\n", idt_adc_result);
	sprintf(page, "%d", idt_adc_result);
	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

	return ret;
}

static ssize_t proc_wireless_idt_adc_test_write(struct file *file, const char __user *buf,
		size_t len, loff_t *lo)
{
	char buffer[4] = {0};
	int idt_adc_cmd = 0;
	struct oplus_nu1619_ic *chip = nu1619_chip;

	if (chip == NULL) {
		chg_err("%s: nu1619 driver is not ready\n", __func__);
		return 0;
	}

	if (len > 4) {
		chg_err("%s: len[%d] -EFAULT.\n", __func__, len);
		return -EFAULT;
	}

	if (copy_from_user(buffer, buf, len)) {
		chg_err("%s:  error.\n", __func__);
		return -EFAULT;
	}

	kstrtoint(buffer, 0, &idt_adc_cmd);
	if (idt_adc_cmd == 0) {
		chg_err("<~WPC~> idt_adc_test: set 0.\n");
		chip->nu1619_chg_status.idt_adc_test_enable = false;
	} else if (idt_adc_cmd == 1) {
		chg_err("<~WPC~> idt_adc_test: set 1.\n");
		chip->nu1619_chg_status.idt_adc_test_enable = true;
	} else {
		chg_err("<~WPC~> idt_adc_test: set %d, invalid.\n", idt_adc_cmd);
	}

	return len;
}

static const struct file_operations proc_wireless_idt_adc_test_ops = {
	.read = proc_wireless_idt_adc_test_read,
	.write = proc_wireless_idt_adc_test_write,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static int init_wireless_charge_proc(struct oplus_nu1619_ic *chip)
{
	int ret = 0;
	struct proc_dir_entry *prEntry_da = NULL;
	struct proc_dir_entry *prEntry_tmp = NULL;

	prEntry_da = proc_mkdir("wireless", NULL);
	if (prEntry_da == NULL) {
		ret = -ENOMEM;
		chg_debug("%s: Couldn't create wireless proc entry\n",
			  __func__);
	}

	prEntry_tmp = proc_create_data("voltage_rect", 0664, prEntry_da,
				       &proc_wireless_voltage_rect_ops, chip);
	if (prEntry_tmp == NULL) {
		ret = -ENOMEM;
		chg_debug("%s: Couldn't create proc entry, %d\n", __func__,
			  __LINE__);
	}

	prEntry_tmp = proc_create_data("rx_voltage", 0664, prEntry_da,
				       &proc_wireless_rx_voltage, chip);
	if (prEntry_tmp == NULL) {
		ret = -ENOMEM;
		chg_debug("%s: Couldn't create proc entry, %d\n", __func__,
			  __LINE__);
	}

	prEntry_tmp = proc_create_data("current_out", 0664, prEntry_da,
				       &proc_wireless_current_out_ops, chip);
	if (prEntry_tmp == NULL) {
		ret = -ENOMEM;
		chg_debug("%s: Couldn't create proc entry, %d\n", __func__,
			  __LINE__);
	}

	prEntry_tmp = proc_create_data("ftm_mode", 0664, prEntry_da,
				       &proc_wireless_ftm_mode_ops, chip);
	if (prEntry_tmp == NULL) {
		ret = -ENOMEM;
		chg_debug("%s: Couldn't create proc entry, %d\n", __func__,
			  __LINE__);
	}

	prEntry_tmp = proc_create_data("enable_tx", 0664, prEntry_da,
				       &proc_wireless_tx_ops, chip);
	if (prEntry_tmp == NULL) {
		ret = -ENOMEM;
		chg_debug("%s: Couldn't create proc entry, %d\n", __func__,
			  __LINE__);
	}

	prEntry_tmp = proc_create_data("epp_or_bpp", 0664, prEntry_da,
				       &proc_wireless_epp_ops, chip);
	if (prEntry_tmp == NULL) {
		ret = -ENOMEM;
		chg_debug("%s: Couldn't create proc entry, %d\n", __func__,
			  __LINE__);
	}

	prEntry_tmp = proc_create_data("charge_pump_en", 0664, prEntry_da,
				       &proc_wireless_charge_pump_ops, chip);
	if (prEntry_tmp == NULL) {
		ret = -ENOMEM;
		chg_debug("%s: Couldn't create proc entry, %d\n", __func__,
			  __LINE__);
	}

	prEntry_tmp = proc_create_data("bat_mult", 0664, prEntry_da,
				       &proc_wireless_bat_mult_ops, chip);
	if (prEntry_tmp == NULL) {
		ret = -ENOMEM;
		chg_debug("%s: Couldn't create proc entry, %d\n", __func__,
			  __LINE__);
	}

	prEntry_tmp = proc_create_data("deviated", 0664, prEntry_da,
				       &proc_wireless_deviated_ops, chip);
	if (prEntry_tmp == NULL) {
		ret = -ENOMEM;
		chg_debug("%s: Couldn't create proc entry, %d\n", __func__,
			  __LINE__);
	}

	prEntry_tmp = proc_create_data("enable_rx", 0664, prEntry_da,
				       &proc_wireless_rx_ops, chip);
	if (prEntry_tmp == NULL) {
		ret = -ENOMEM;
		chg_debug("%s: Couldn't create proc entry, %d\n", __func__,
			  __LINE__);
	}

	prEntry_tmp = proc_create_data("upgrade_firmware", 0664, prEntry_da,
				       &proc_upgrade_firmware_ops, chip);
	if (prEntry_tmp == NULL) {
		ret = -ENOMEM;
		chg_debug("%s: Couldn't create proc entry, %d\n", __func__,
			  __LINE__);
	}

	prEntry_tmp = proc_create_data("rx_freq", 0664, prEntry_da,
				       &proc_wireless_rx_freq_ops, chip);
	if (prEntry_tmp == NULL) {
		ret = -ENOMEM;
		chg_debug("%s: Couldn't create proc entry, %d\n", __func__,
			  __LINE__);
	}

	prEntry_tmp = proc_create_data("user_sleep_mode", 0664, prEntry_da,
				       &proc_wireless_user_sleep_mode_ops, chip);
	if (prEntry_tmp == NULL) {
		ret = -ENOMEM;
		chg_debug("%s: Couldn't create proc entry, %d\n", __func__,
			  __LINE__);
	}

	prEntry_tmp = proc_create_data("idt_adc_test", 0664, prEntry_da,
				       &proc_wireless_idt_adc_test_ops, chip);
	if (prEntry_tmp == NULL) {
		ret = -ENOMEM;
		chg_debug("%s: Couldn't create proc entry, %d\n", __func__,
			  __LINE__);
	}

#ifdef HW_TEST_EDITION
	prEntry_tmp = proc_create_data("w30w_time", 0664, prEntry_da,
				       &proc_wireless_w30w_time_ops, chip);
	if (prEntry_tmp == NULL) {
		ret = -ENOMEM;
		chg_debug("%s: Couldn't create proc entry, %d\n", __func__,
			  __LINE__);
	}
#endif
	return 0;
}


static enum power_supply_property nu1619_wireless_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_REAL_TYPE,
	POWER_SUPPLY_PROP_TX_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TX_CURRENT_NOW,
	POWER_SUPPLY_PROP_CP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CP_CURRENT_NOW,
	POWER_SUPPLY_PROP_WIRELESS_MODE,
	POWER_SUPPLY_PROP_WIRELESS_TYPE,
	POWER_SUPPLY_PROP_CEP_INFO,
};

static int nu1619_wireless_get_prop(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct oplus_nu1619_ic *chip = nu1619_chip;
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = chip->nu1619_chg_status.charge_online;
		if (val->intval == 0 && nu1619_get_self_reset())
			val->intval = 1;
		rc = 0;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = chip->nu1619_chg_status.charge_online;
		if (val->intval == 0 && nu1619_get_self_reset())
			val->intval = 1;
		chg_err("wireless_get_prop online[%d]\n", val->intval);
		rc = 0;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
#ifdef oplus_wireless
		if (chip->wireless_mode == WIRELESS_MODE_RX)
			val->intval = nu1619_wireless_get_vout();
		else
#endif
		val->intval = nu1619_wireless_get_vout();
		rc = 0;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
#ifdef oplus_wireless
		if (chip->wireless_mode == WIRELESS_MODE_RX)
			val->intval = chip->nu1619_chg_status.target_vol;
		else
#endif
		val->intval = 0;
		rc = 0;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
#ifdef oplus_wireless
		if (chip->wireless_mode == WIRELESS_MODE_RX)
			val->intval = nu1619_chip->nu1619_chg_status.iout;
		else
#endif
		val->intval = nu1619_chip->nu1619_chg_status.iout;
		rc = 0;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
#ifdef oplus_wireless
		if (chip->wireless_mode == WIRELESS_MODE_RX)
			val->intval = chip->nu1619_chg_status.max_current;
		else
#endif
		val->intval = 0;
		rc = 0;
		break;
	case POWER_SUPPLY_PROP_TX_VOLTAGE_NOW:
#ifdef oplus_wireless
		if (chip->wireless_mode == WIRELESS_MODE_TX)
			val->intval = nu1619_chip->nu1619_chg_status.tx_vol;
		else
#endif
		val->intval = 0;
		rc = 0;
		break;
	case POWER_SUPPLY_PROP_TX_CURRENT_NOW:
#ifdef oplus_wireless
		if (chip->wireless_mode == WIRELESS_MODE_TX)
			val->intval = nu1619_chip->nu1619_chg_status.tx_curr;
		else
#endif
		val->intval = 0;
		rc = 0;
		break;
	case POWER_SUPPLY_PROP_CP_VOLTAGE_NOW:
#ifdef oplus_wireless
		if (chip->wireless_mode == WIRELESS_MODE_RX) {
			if (exchgpump_bq == NULL)
				return -ENODEV;
			bq2597x_get_adc_data(exchgpump_bq, ADC_VBUS, &tmp);
			val->intval = tmp * 1000;
		} else {
			val->intval = 0;
		}
#else
			val->intval = 0;
#endif
		rc = 0;
		break;
	case POWER_SUPPLY_PROP_CP_CURRENT_NOW:
#ifdef oplus_wireless
		if (chip->wireless_mode == WIRELESS_MODE_RX) {
			if (exchgpump_bq == NULL)
				return -ENODEV;
			bq2597x_get_adc_data(exchgpump_bq, ADC_IBUS, &tmp);
			val->intval = tmp * 1000;
		} else {
			val->intval = 0;
		}
#else
			val->intval = 0;
#endif
		rc = 0;
		break;
	case POWER_SUPPLY_PROP_REAL_TYPE:
		switch (nu1619_chip->nu1619_chg_status.adapter_type) {
		case ADAPTER_TYPE_VOOC:
		case ADAPTER_TYPE_SVOOC:
			val->intval = POWER_SUPPLY_TYPE_USB_DCP;
			break;
		case ADAPTER_TYPE_USB:
			val->intval = POWER_SUPPLY_TYPE_USB;
			break;
		case ADAPTER_TYPE_NORMAL_CHARGE:
			val->intval = POWER_SUPPLY_TYPE_USB_DCP;
			break;
		case ADAPTER_TYPE_EPP:
			val->intval = POWER_SUPPLY_TYPE_USB_PD;
			break;
		case ADAPTER_TYPE_UNKNOW:
			if (chip->nu1619_chg_status.rx_runing_mode == RX_RUNNING_MODE_BPP)
				val->intval = POWER_SUPPLY_TYPE_USB_DCP;
			else
				val->intval = POWER_SUPPLY_TYPE_UNKNOWN;
			break;
		default:
			val->intval = POWER_SUPPLY_TYPE_UNKNOWN;
			break;
		}
		chg_err("wireless_get_prop power supply type[%d], adapter_type[%d]\n", val->intval, nu1619_chip->nu1619_chg_status.adapter_type);
		rc = 0;
		break;
	case POWER_SUPPLY_PROP_WIRELESS_MODE:
#ifdef oplus_wireless
		val->strval = nu1619_wireless_get_mode(chip);
#endif
		rc = 0;
		break;
	case POWER_SUPPLY_PROP_WIRELESS_TYPE:
#ifdef oplus_wireless
		val->intval = chip->wireless_type;
#endif
		rc = 0;
		break;
	case POWER_SUPPLY_PROP_CEP_INFO:
		val->intval = chip->nu1619_chg_status.cep_info;
		rc = 0;
		break;

	default:
		return -EINVAL;
	}
	if (rc < 0) {
		pr_debug("Couldn't get prop %d rc = %d\n", psp, rc);
		return -ENODATA;
	}
	return 0;
}

static int nu1619_wireless_set_prop(struct power_supply *psy,
				   enum power_supply_property psp,
				   const union power_supply_propval *val)
{
#ifdef oplus_wireless
	struct oplus_chg_chip *chip = power_supply_get_drvdata(psy);
#endif
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
#ifdef oplus_wireless
		if (!chip->nu1619_chg_status.fastchg_mode) {
			nu1619_set_rx_target_voltage(chip, val->intval / 1000);
			rc = 0;
		} else {
			chg_err("is fastchg mode, can't set rx voltage\n");
			rc = -EINVAL;
		}
#endif
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
#ifdef oplus_wireless
		chip->nu1619_chg_status.max_current = val->intval / 1000;
		vote(chip->wlcs_fcc_votable, MAX_VOTER, true, val->intval);
#endif
		rc = 0;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
#ifdef oplus_wireless
		vote(chip->wlcs_fcc_votable, USER_VOTER, true, val->intval);
#endif
		rc = 0;
		break;
	case POWER_SUPPLY_PROP_CEP_INFO:
		if (val->intval == 0) {
			chg_err("clear cep info = %d, wpc_skewing_proc = %d\n",
				nu1619_chip->nu1619_chg_status.cep_info, nu1619_chip->nu1619_chg_status.wpc_skewing_proc);
			if (!nu1619_chip->nu1619_chg_status.wpc_skewing_proc) {
				nu1619_chip->nu1619_chg_status.skewing_info = false;
				nu1619_chip->nu1619_chg_status.cep_info = 0;
			}
		}
		rc = 0;
		break;

	default:
		chg_err("set prop %d is not supported\n", psp);
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int nu1619_wireless_prop_is_writeable(struct power_supply *psy,
					    enum power_supply_property psp)
{
	int rc;

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
	case POWER_SUPPLY_PROP_CEP_INFO:
		rc = 1;
		break;
	default:
		rc = 0;
		break;
	}

	return rc;
}


static const struct power_supply_desc wireless_psy_desc = {
	.name = "wireless",
	.type = POWER_SUPPLY_TYPE_WIRELESS,
	.properties = nu1619_wireless_props,
	.num_properties = ARRAY_SIZE(nu1619_wireless_props),
	.get_property = nu1619_wireless_get_prop,
	.set_property = nu1619_wireless_set_prop,
	.property_is_writeable = nu1619_wireless_prop_is_writeable,
};

static int nu1619_init_wireless_psy(struct oplus_nu1619_ic *chip)
{
	struct power_supply_config wireless_cfg = {};

	wireless_cfg.drv_data = chip;
	wireless_cfg.of_node = chip->dev->of_node;
	chip->wireless_psy = devm_power_supply_register(
		chip->dev, &wireless_psy_desc, &wireless_cfg);
	if (IS_ERR(chip->wireless_psy)) {
		chg_err("Couldn't register wireless power supply\n");
		return PTR_ERR(chip->wireless_psy);
	}

	return 0;
}

bool nu1619_check_chip_is_null(void)
{
	if (nu1619_chip)
		return false;
	return true;
}

struct oplus_wpc_operations nu1619_ops = {
	.wireless_charge_start = nu1619_wireless_charge_start,
	.wpc_get_normal_charging = nu1619_wpc_get_normal_charging,
	.wpc_get_fast_charging = nu1619_wpc_get_fast_charging,
	.wpc_get_otg_charging = nu1619_wpc_get_otg_charging,
	.wpc_get_ffc_charging = nu1619_wpc_get_ffc_charging,
	.wpc_get_fw_updating = nu1619_wpc_get_fw_updating,
	.wpc_get_adapter_type = nu1619_wpc_get_adapter_type,
	.wpc_set_vbat_en = nu1619_set_vbat_en_val,
	.wpc_set_booster_en = nu1619_set_booster_en_val,
	.wpc_set_ext1_wired_otg_en = nu1619_set_ext1_wired_otg_en_val,
	.wpc_set_ext2_wireless_otg_en = nu1619_set_ext2_wireless_otg_en_val,
	.wpc_set_rtx_function = nu1619_set_rtx_function,
	.wpc_set_rtx_function_prepare = nu1619_set_rtx_function_prepare,
	.wpc_print_log = nu1619_wpc_print_log,
};

static int nu1619_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct oplus_nu1619_ic	*chip;
	struct oplus_wpc_chip *wpc_chip;
	int rc = 0;

	chg_err(" call \n");

	if (oplus_chg_check_chip_is_null() == true) {
		chg_err(" g_oplus_chg chip is null, probe again \n");
		return -EPROBE_DEFER;
	}

	chip = devm_kzalloc(&client->dev,
		sizeof(struct oplus_nu1619_ic), GFP_KERNEL);
	if (!chip) {
		chg_err(" kzalloc() failed\n");
		return -ENOMEM;
	}

	wpc_chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!wpc_chip) {
		dev_err(&client->dev, "Couldn't allocate memory\n");
		return -ENOMEM;
	}

	wpc_chip->client = client;
	wpc_chip->dev = &client->dev;
	wpc_chip->wpc_ops = &nu1619_ops;
	oplus_wpc_init(wpc_chip);

	chip->client = client;
	chip->dev = &client->dev;

	nu1619_idt_gpio_init(chip);
	oplus_wpc_chg_parse_chg_dt(chip);

#ifdef DEBUG_BY_FILE_OPS
	init_nu1619_add_log();
	init_nu1619_data_log();
#endif

	chip->nu1619_chg_status.check_fw_update = false;
	chip->nu1619_chg_status.wpc_self_reset = false;
	chip->nu1619_chg_status.idt_adc_test_enable = false;
	chip->nu1619_chg_status.wpc_dischg_status = WPC_DISCHG_STATUS_OFF;
	chip->nu1619_chg_status.ftm_mode = false;
	chip->nu1619_chg_status.work_silent_mode = false;
	chip->nu1619_chg_status.call_mode = false;
	chip->quiet_mode_need = -1;

	nu1619_reset_variables(chip);

	INIT_DELAYED_WORK(&chip->idt_event_int_work, nu1619_idt_event_int_func);
	INIT_DELAYED_WORK(&chip->idt_connect_int_work, nu1619_idt_connect_int_func);
	INIT_DELAYED_WORK(&chip->idt_dischg_work, nu1619_idt_dischg_work);

	INIT_DELAYED_WORK(&chip->nu1619_task_work, nu1619_task_work_process);
	INIT_DELAYED_WORK(&chip->nu1619_CEP_work, nu1619_CEP_work_process);
	INIT_DELAYED_WORK(&chip->nu1619_update_work, nu1619_update_work_process);
	INIT_DELAYED_WORK(&chip->nu1619_self_reset_work, nu1619_self_reset_process);
#ifdef FASTCHG_TEST_BY_TIME
	INIT_DELAYED_WORK(&chip->nu1619_test_work, nu1619_test_work_process);
#endif

	nu1619_chip = chip;

	schedule_delayed_work(&chip->nu1619_update_work, P922X_UPDATE_INTERVAL);

	if (g_oplus_chip && !g_oplus_chip->charger_exist)
		nu1619_idt_connect_shedule_work();
	rc = init_wireless_charge_proc(chip);
	if (rc < 0) {
		chg_err("Create wireless charge proc error.");
	}
	nu1619_init_wireless_psy(chip);

	chg_debug(" call end\n");

	return 0;
}


static struct i2c_driver nu1619_i2c_driver;

static int nu1619_driver_remove(struct i2c_client *client)
{
	return 0;
}


#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
static int nu1619_pm_resume(struct device *dev)
{
	return 0;
}

static int nu1619_pm_suspend(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops nu1619_pm_ops = {
	.resume		= nu1619_pm_resume,
	.suspend		= nu1619_pm_suspend,
};
#else
static int nu1619_resume(struct i2c_client *client)
{
	return 0;
}

static int nu1619_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return 0;
}
#endif

static void nu1619_reset(struct i2c_client *client)
{
	int wpc_con_level = 0;
	int wait_wpc_disconn_cnt = 0;

	nu1619_set_vt_sleep_val(1);

	wpc_con_level = nu1619_get_idt_con_val();
	if(wpc_con_level == 1) {
		msleep(100);

		while(wait_wpc_disconn_cnt < 10) {
			wpc_con_level = nu1619_get_idt_con_val();
			if (wpc_con_level == 0) {
				break;
			}
			msleep(150);
			wait_wpc_disconn_cnt++;
		}
		chargepump_disable();
	}
	return;
}

/**********************************************************
  *
  *   [platform_driver API] 
  *
  *********************************************************/

static const struct of_device_id nu1619_match[] = {
	{ .compatible = "oplus,nu1619-charger"},
	{ },
};

static const struct i2c_device_id nu1619_id[] = {
	{"nu1619-charger", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, nu1619_id);


static struct i2c_driver nu1619_i2c_driver = {
	.driver		= {
		.name = "nu1619-charger",
		.owner	= THIS_MODULE,
		.of_match_table = nu1619_match,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
					.pm 	= &nu1619_pm_ops,
#endif
	},
	.probe		= nu1619_driver_probe,
	.remove		= nu1619_driver_remove,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0))
	.resume		= nu1619_resume,
	.suspend	= nu1619_suspend,
#endif
	.shutdown	= nu1619_reset,
	.id_table	= nu1619_id,
};


module_i2c_driver(nu1619_i2c_driver);
MODULE_DESCRIPTION("Driver for nu1619 charger chip");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:nu1619-charger");
