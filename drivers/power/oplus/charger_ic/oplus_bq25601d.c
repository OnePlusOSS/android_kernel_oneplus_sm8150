/************************************************************************************
** OPLUS_FEATURE_CHG_BASIC
** Copyright (C), 2018-2019, OPLUS Mobile Comm Corp., Ltd
**
** Description:
**    For bq25601d charger ic driver
**
** Version: 1.0
** Date created: 2018-09-24
**
** --------------------------- Revision History: ------------------------------------
* <version>       <date>         <author>              			<desc>
*************************************************************************************/

#include <linux/interrupt.h>
#include <linux/i2c.h>

#ifdef CONFIG_OPLUS_CHARGER_MTK
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <linux/xlog.h>
#ifdef CONFIG_OPLUS_CHARGER_6750T
#include <linux/module.h>
#include <upmu_common.h>
#include <mt-plat/mt_gpio.h>
#include <mt_boot_common.h>
#include <mt-plat/mtk_rtc.h>
#elif defined(CONFIG_OPLUS_CHARGER_MTK6763) || defined(CONFIG_OPLUS_CHARGER_MTK6771)
#include <linux/module.h>

#include <mt-plat/mtk_gpio.h>

#include <mt-plat/mtk_rtc.h>

#else
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_boot.h>
#include <mach/mtk_rtc.h>
#endif
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

#include <mach/oplus_boot_mode.h>
#include <soc/oplus/device_info.h>

void (*enable_aggressive_segmentation_fn)(bool);

#endif

#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include "../oplus_vooc.h"
#include "../oplus_gauge.h"
#include <oplus_bq25601d.h>
extern int oplus_get_rtc_ui_soc(void);
extern int oplus_set_rtc_ui_soc(int value);
static struct chip_bq25601d *charger_ic = NULL;
static int aicl_result = 500;
static struct delayed_work charger_modefy_work;

static DEFINE_MUTEX(bq25601d_i2c_access);

static int __bq25601d_read_reg(struct chip_bq25601d *chip, int reg, int *returnData)
{
#ifdef CONFIG_OPLUS_CHARGER_MTK
#if defined(CONFIG_OPLUS_CHARGER_MTK6763) || defined(CONFIG_OPLUS_CHARGER_MTK6771)
	int ret = 0;

	ret = i2c_smbus_read_byte_data(chip->client, reg);
	if (ret < 0) {
		chg_err("i2c read fail: can't read from %02x: %d\n", reg, ret);
		return ret;
	} else {
		*returnData = ret;
	}
#else
	char cmd_buf[1] = {0x00};
	char readData = 0;
	int ret = 0;

	chip->client->ext_flag = ((chip->client->ext_flag) & I2C_MASK_FLAG) | I2C_WR_FLAG | I2C_DIRECTION_FLAG;
	chip->client->timing = 300;
	cmd_buf[0] = reg;
	ret = i2c_master_send(chip->client, &cmd_buf[0], (1 << 8 | 1));
	if (ret < 0) {
		chip->client->ext_flag = 0;
		return ret;
	}

	readData = cmd_buf[0];
	*returnData = readData;

	chip->client->ext_flag = 0;
#endif
#else
	int ret = 0;

	ret = i2c_smbus_read_byte_data(chip->client, reg);
	if (ret < 0) {
		chg_err("i2c read fail: can't read from %02x: %d\n", reg, ret);
		return ret;
	} else {
		*returnData = ret;
	}
#endif /* CONFIG_OPLUS_CHARGER_MTK */

	return 0;
}

static int bq25601d_read_reg(struct chip_bq25601d *chip, int reg, int *returnData)
{
	int ret = 0;

	mutex_lock(&bq25601d_i2c_access);
	ret = __bq25601d_read_reg(chip, reg, returnData);
	mutex_unlock(&bq25601d_i2c_access);
	return ret;
}

static int __bq25601d_write_reg(struct chip_bq25601d *chip, int reg, int val)
{
#ifdef CONFIG_OPLUS_CHARGER_MTK
#if defined(CONFIG_OPLUS_CHARGER_MTK6763) || defined(CONFIG_OPLUS_CHARGER_MTK6771)
	int ret = 0;

	ret = i2c_smbus_write_byte_data(chip->client, reg, val);
	if (ret < 0) {
		chg_err("i2c write fail: can't write %02x to %02x: %d\n", val, reg, ret);
		return ret;
	}
#else
	char write_data[2] = {0};
	int ret = 0;

	write_data[0] = reg;
	write_data[1] = val;

	chip->client->ext_flag = ((chip->client->ext_flag) & I2C_MASK_FLAG) | I2C_DIRECTION_FLAG;
	chip->client->timing = 300;
	ret = i2c_master_send(chip->client, write_data, 2);
	if (ret < 0) {
		chip->client->ext_flag = 0;
		return ret;
	}

	chip->client->ext_flag = 0;
#endif
#else /* CONFIG_OPLUS_CHARGER_MTK */
	int ret = 0;

	ret = i2c_smbus_write_byte_data(chip->client, reg, val);
	if (ret < 0) {
		chg_err("i2c write fail: can't write %02x to %02x: %d\n", val, reg, ret);
		return ret;
	}
#endif /* CONFIG_OPLUS_CHARGER_MTK */

	return 0;
}

static int bq25601d_config_interface (struct chip_bq25601d *chip, int RegNum, int val, int MASK)
{
	int bq25601d_reg = 0;
	int ret = 0;

	mutex_lock(&bq25601d_i2c_access);
	ret = __bq25601d_read_reg(chip, RegNum, &bq25601d_reg);

	//chg_err("Reg[%x]=0x%x\n", RegNum, bq25601d_reg);

	bq25601d_reg &= ~MASK;
	bq25601d_reg |= val;

	ret = __bq25601d_write_reg(chip, RegNum, bq25601d_reg);

	//chg_err("write Reg[%x]=0x%x\n", RegNum, bq25601d_reg);

	__bq25601d_read_reg(chip, RegNum, &bq25601d_reg);

	//chg_err("Check Reg[%x]=0x%x\n", RegNum, bq25601d_reg);
	mutex_unlock(&bq25601d_i2c_access);

	return ret;
}

static int bq25601d_usbin_input_current_limit[] = {
	100,    300,    500,    900,
	1200,   1500,   2000,   3000,
};

#if defined(CONFIG_OPLUS_CHARGER_MTK6763) || defined(CONFIG_OPLUS_CHARGER_MTK6771)
#define AICL_COUNT	10
static int chg_vol_mini(int *chg_vol)
{
	int chg_vol_temp = 0;
	int i = 0;

	chg_vol_temp = chg_vol[0];
	for (i = 0; i < AICL_COUNT; i++) {
		if (chg_vol_temp > chg_vol[i]) {
			chg_vol_temp = chg_vol[i];
		}
	}
	return chg_vol_temp;
}
#endif

static int bq25601d_input_current_limit_write(int value)
{
	int rc = 0;
	int i = 0;
	int j = 0;
	int chg_vol = 0;
	int aicl_point_temp = 0;
	int chg_vol_all[10] = {0};
	struct chip_bq25601d *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		chg_err("charger in suspended.\n");
		return 0;
	}

	chg_debug("usb input max current limit=%d setting %02x\n", value, i);

	aicl_point_temp = chip->sw_aicl_point;

	if (value < 300) {
		j = 0;
		goto aicl_end;
	} else if (value < 500) {
		j = 1;
		goto aicl_end;
	}

	j = 2; /* 500 */
	rc = bq25601d_config_interface(chip, REG00_BQ25601D_ADDRESS, REG00_BQ25601D_INPUT_CURRENT_LIMIT_500MA, REG00_BQ25601D_INPUT_CURRENT_LIMIT_MASK);
	msleep(90);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point_temp) {
		j = 2;
		goto aicl_pre_step;
	} else if (value < 900)
		goto aicl_end;

	j = 3; /* 900 */
	rc = bq25601d_config_interface(chip, REG00_BQ25601D_ADDRESS, REG00_BQ25601D_INPUT_CURRENT_LIMIT_900MA, REG00_BQ25601D_INPUT_CURRENT_LIMIT_MASK);
	msleep(90);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point_temp) {
		j = j - 1;
		goto aicl_pre_step;
	} else if (value < 1200)
		goto aicl_end;

	j = 4; /* 1200 */
	rc = bq25601d_config_interface(chip, REG00_BQ25601D_ADDRESS, REG00_BQ25601D_INPUT_CURRENT_LIMIT_1200MA, REG00_BQ25601D_INPUT_CURRENT_LIMIT_MASK);
	msleep(90);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point_temp) {
		j = j - 1;
		goto aicl_pre_step;
	}

	j = 5; /* 1500 */
	rc = bq25601d_config_interface(chip, REG00_BQ25601D_ADDRESS, REG00_BQ25601D_INPUT_CURRENT_LIMIT_1500MA, REG00_BQ25601D_INPUT_CURRENT_LIMIT_MASK);
	msleep(90);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point_temp) {
		j = j - 2; //We DO NOT use 1.2A here
		goto aicl_pre_step;
	} else if (value < 1500) {
		j = j - 1; //We use 1.2A here
		goto aicl_end;
	} else if (value < 2000)
		goto aicl_end;

	j = 6; /* 2000 */
	rc = bq25601d_config_interface(chip, REG00_BQ25601D_ADDRESS, REG00_BQ25601D_INPUT_CURRENT_LIMIT_2000MA, REG00_BQ25601D_INPUT_CURRENT_LIMIT_MASK);
	msleep(90);
#if defined(CONFIG_OPLUS_CHARGER_MTK6763) || defined(CONFIG_OPLUS_CHARGER_MTK6771)
	for (i = 0; i < AICL_COUNT; i++) {
		chg_vol = battery_meter_get_charger_voltage();
		chg_vol_all[i] = chg_vol;
		usleep_range(5000, 5000);
	}
	chg_vol = chg_vol_mini(chg_vol_all);
#else
	chg_vol = battery_meter_get_charger_voltage();
#endif
	if (chg_vol < aicl_point_temp) {
		j = j - 1;
		goto aicl_pre_step;
	} else if (value < 3000)
		goto aicl_end;

	j = 7; /* 3000 */
	rc = bq25601d_config_interface(chip, REG00_BQ25601D_ADDRESS, REG00_BQ25601D_INPUT_CURRENT_LIMIT_3000MA, REG00_BQ25601D_INPUT_CURRENT_LIMIT_MASK);
	msleep(90);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point_temp) {
		j = j - 1;
		goto aicl_pre_step;
	} else if (value >= 3000)
		goto aicl_end;

aicl_pre_step:		
	chg_debug("usb input max current limit aicl chg_vol=%d j[%d]=%d sw_aicl_point:%d aicl_pre_step\n", chg_vol, j, bq25601d_usbin_input_current_limit[j], aicl_point_temp);
	goto aicl_rerun;
aicl_end:		
	chg_debug("usb input max current limit aicl chg_vol=%d j[%d]=%d sw_aicl_point:%d aicl_end\n", chg_vol, j, bq25601d_usbin_input_current_limit[j], aicl_point_temp);
	goto aicl_rerun;
aicl_rerun:
	if ((j >= 2) && (j <= ARRAY_SIZE(bq25601d_usbin_input_current_limit) - 1))
		aicl_result = bq25601d_usbin_input_current_limit[j];
	switch (j) {
		case 0:
			bq25601d_config_interface(chip, REG00_BQ25601D_ADDRESS, REG00_BQ25601D_INPUT_CURRENT_LIMIT_100MA, REG00_BQ25601D_INPUT_CURRENT_LIMIT_MASK);
			break;
		case 1:
			bq25601d_config_interface(chip, REG00_BQ25601D_ADDRESS, REG00_BQ25601D_INPUT_CURRENT_LIMIT_300MA, REG00_BQ25601D_INPUT_CURRENT_LIMIT_MASK);
			break;
		case 2:
			bq25601d_config_interface(chip, REG00_BQ25601D_ADDRESS, REG00_BQ25601D_INPUT_CURRENT_LIMIT_500MA, REG00_BQ25601D_INPUT_CURRENT_LIMIT_MASK);
			break;
		case 3:
			bq25601d_config_interface(chip, REG00_BQ25601D_ADDRESS, REG00_BQ25601D_INPUT_CURRENT_LIMIT_900MA, REG00_BQ25601D_INPUT_CURRENT_LIMIT_MASK);
			break;
		case 4:
			bq25601d_config_interface(chip, REG00_BQ25601D_ADDRESS, REG00_BQ25601D_INPUT_CURRENT_LIMIT_1200MA, REG00_BQ25601D_INPUT_CURRENT_LIMIT_MASK);
			break;
		case 5:
			bq25601d_config_interface(chip, REG00_BQ25601D_ADDRESS, REG00_BQ25601D_INPUT_CURRENT_LIMIT_1500MA, REG00_BQ25601D_INPUT_CURRENT_LIMIT_MASK);
			break;
		case 6:
			bq25601d_config_interface(chip, REG00_BQ25601D_ADDRESS, REG00_BQ25601D_INPUT_CURRENT_LIMIT_2000MA, REG00_BQ25601D_INPUT_CURRENT_LIMIT_MASK);
			break;
		case 7:
			bq25601d_config_interface(chip, REG00_BQ25601D_ADDRESS, REG00_BQ25601D_INPUT_CURRENT_LIMIT_3000MA, REG00_BQ25601D_INPUT_CURRENT_LIMIT_MASK);
			break;
		default:
			break;
	}
	return rc;
}

static int bq25601d_charging_current_write_fast(int chg_cur)
{
	int rc = 0;
	int value = 0;
	struct chip_bq25601d *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		chg_err("charger in suspended.\n");
		return 0;
	}

	chg_debug("chg_cur = %d\r\n", chg_cur);

	if (chg_cur > REG02_BQ25601D_MAX_FAST_CHG_CURRENT_MA
			|| chg_cur < REG02_BQ25601D_MIN_FAST_CHG_CURRENT_MA)
		chg_cur = REG02_BQ25601D_MAX_FAST_CHG_CURRENT_MA;
	value = (chg_cur - REG02_BQ25601D_MIN_FAST_CHG_CURRENT_MA) / REG02_BQ25601D_FAST_CHG_CURRENT_STEP_MA;
	value <<= REG02_BQ25601D_FAST_CHG_CURRENT_LIMIT_SHIFT;

	rc = bq25601d_config_interface(chip, REG02_BQ25601D_ADDRESS, value,
			REG02_BQ25601D_FAST_CHG_CURRENT_LIMIT_MASK);
	if (rc < 0) {
		chg_err("Couldn't write fast charge current rc = %d\n", rc);
	}

	return rc;
}

static int bq25601d_set_vindpm_vol(int vol)
{
	int rc = 0;
	int value = 0;
	struct chip_bq25601d *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		chg_err("charger in suspended.\n");
		return 0;
	}

	if (vol < REG06_BQ25601D_VINDPM_OFFSET)
		vol = REG06_BQ25601D_VINDPM_OFFSET;
	value = (vol - REG06_BQ25601D_VINDPM_OFFSET) / REG06_BQ25601D_VINDPM_STEP_MV;
	value <<= REG06_BQ25601D_VINDPM_SHIFT;

	rc = bq25601d_config_interface(chip, REG06_BQ25601D_ADDRESS,
			value, REG06_BQ25601D_VINDPM_MASK);
	return rc;
}

#ifdef CONFIG_OPLUS_SHORT_C_BATT_CHECK
/* This function is getting the dynamic aicl result/input limited in mA.
 * If charger was suspended, it must return 0(mA).
 * It meets the requirements in SDM660 platform.
 */
static int bq25601d_chg_get_dyna_aicl_result(void)
{
	return aicl_result;
}
#endif /* CONFIG_OPLUS_SHORT_C_BATT_CHECK */

static void bq25601d_set_aicl_point(int vbatt)
{
	struct chip_bq25601d *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		chg_err("charger in suspended.\n");
		return;
	}

	if (chip->hw_aicl_point == 4400 && vbatt > 4100) {
		chip->hw_aicl_point = 4500;
		chip->sw_aicl_point = 4550;
		bq25601d_set_vindpm_vol(chip->hw_aicl_point);
	} else if (chip->hw_aicl_point == 4500 && vbatt <= 4100) {
		chip->hw_aicl_point = 4400;
		chip->sw_aicl_point = 4500;
		bq25601d_set_vindpm_vol(chip->hw_aicl_point);
	}
}

static int bq25601d_set_enable_volatile_writes(void)
{
	int rc = 0;
	struct chip_bq25601d *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		chg_err("charger in suspended.\n");
		return 0;
	}

	return rc;
}

static int bq25601d_set_complete_charge_timeout(int val)
{
	int rc = 0;
	struct chip_bq25601d *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		chg_err("charger in suspended.\n");
		return 0;
	}

	if (val == OVERTIME_AC) {
		val = REG05_BQ25601D_CHG_SAFETY_TIMER_ENABLE | REG05_BQ25601D_FAST_CHG_TIMEOUT_10H;
	} else if (val == OVERTIME_USB) {
		val = REG05_BQ25601D_CHG_SAFETY_TIMER_ENABLE | REG05_BQ25601D_FAST_CHG_TIMEOUT_10H;
	} else {
		val = REG05_BQ25601D_CHG_SAFETY_TIMER_DISABLE | REG05_BQ25601D_FAST_CHG_TIMEOUT_10H;
	}

	rc = bq25601d_config_interface(chip, REG05_BQ25601D_ADDRESS, val,
			REG05_BQ25601D_CHG_SAFETY_TIMER_MASK | REG05_BQ25601D_FAST_CHG_TIMEOUT_MASK);
	if (rc < 0) {
		chg_err("Couldn't complete charge timeout rc = %d\n", rc);
	}

	return rc;
}

static int bq25601d_float_voltage_write(int vfloat_mv)
{
	int rc = 0;
	int value = 0;
	struct chip_bq25601d *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		chg_err("charger in suspended.\n");
		return 0;
	}

	if (vfloat_mv < REG04_BQ25601D_MIN_FLOAT_MV) {
		chg_err("bad vfloat_mv:%d,return\n", vfloat_mv);
		return 0;
	}
	value = (vfloat_mv - REG04_BQ25601D_MIN_FLOAT_MV) / REG04_BQ25601D_VFLOAT_STEP_MV;
	value <<= REG04_BQ25601D_CHG_VOL_LIMIT_SHIFT;
	chg_debug("bq25601d_set_float_voltage vfloat_mv = %d value=%d\n", vfloat_mv, value);

	rc = bq25601d_config_interface(chip, REG04_BQ25601D_ADDRESS, value, REG04_BQ25601D_CHG_VOL_LIMIT_MASK);
	if (rc < 0) {
		chg_err("Couldn't set float voltage rc = %d\n", rc);
	}

	return rc;
}

static int bq25601d_set_prechg_current(int ipre_ma)
{
	int value = 0;
	struct chip_bq25601d *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		chg_err("charger in suspended.\n");
		return 0;
	}

	if (ipre_ma < REG03_BQ25601D_MIN_PRE_CHG_CURRENT_MA)
		ipre_ma = REG03_BQ25601D_MIN_PRE_CHG_CURRENT_MA;
	if (ipre_ma > REG03_BQ25601D_MAX_PRE_CHG_CURRENT_MA)
		ipre_ma = REG03_BQ25601D_MAX_PRE_CHG_CURRENT_MA;
	value = (ipre_ma - REG03_BQ25601D_MIN_PRE_CHG_CURRENT_MA) / REG03_BQ25601D_PRE_CHG_CURRENT_STEP_MA;
	value <<= REG03_BQ25601D_PRE_CHG_CURRENT_LIMIT_SHIFT;

	return bq25601d_config_interface(chip, REG03_BQ25601D_ADDRESS, value,
			REG03_BQ25601D_PRE_CHG_CURRENT_LIMIT_MASK);
}

static int bq25601d_set_termchg_current(int term_curr)
{
	int value = 0;
	int rc = 0;
	struct chip_bq25601d *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		chg_err("charger in suspended.\n");
		return 0;
	}

	if (term_curr < REG03_BQ25601D_MIN_TERM_CHG_CURRENT_MA) {
		term_curr = REG03_BQ25601D_MIN_TERM_CHG_CURRENT_MA;
	}
	value = (term_curr - REG03_BQ25601D_MIN_TERM_CHG_CURRENT_MA) / REG03_BQ25601D_TERM_CHG_CURRENT_STEP_MA;
	value <<= REG03_BQ25601D_TERM_CHG_CURRENT_LIMIT_SHIFT;
	chg_debug("value=%d\n", value);

	rc = bq25601d_config_interface(chip, REG03_BQ25601D_ADDRESS, value,
			REG03_BQ25601D_TERM_CHG_CURRENT_LIMIT_MASK);
	if (rc < 0) {
		chg_err("Couldn't set termination curren rc = %d\n", rc);
	}

	return rc;
}

static int bq25601d_set_rechg_voltage(int recharge_mv)
{
	int reg = 0;
	int rc = 0;
	struct chip_bq25601d *chip = charger_ic;
	
	if (atomic_read(&chip->charger_suspended) == 1) {
		chg_err("charger in suspended.\n");
		return 0;
	}

	/* set recharge voltage */
	if (recharge_mv >= 200) {
		reg = REG04_BQ25601D_RECHG_THRESHOLD_VOL_200MV;
	} else {
		reg = REG04_BQ25601D_RECHG_THRESHOLD_VOL_100MV;
	}
	rc = bq25601d_config_interface(chip, REG04_BQ25601D_ADDRESS, reg, REG04_BQ25601D_RECHG_THRESHOLD_VOL_MASK);
	if (rc) {
		chg_err("Couldn't set rechg threshold rc = %d\n", rc);
	}

	return rc;
}

static int bq25601d_set_wdt_timer(int reg)
{
	int rc = 0;
	struct chip_bq25601d *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		chg_err("charger in suspended.\n");
		return 0;
	}

	rc = bq25601d_config_interface(chip, REG05_BQ25601D_ADDRESS, reg,
			REG05_BQ25601D_WATCHDOG_TIMER_MASK);

	return rc;
}

static int bq25601d_set_chging_term_disable(void)
{
	int rc = 0;
	struct chip_bq25601d *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		chg_err("charger in suspended.\n");
		return 0;
	}

	rc = bq25601d_config_interface(chip, REG05_BQ25601D_ADDRESS,
			REG05_BQ25601D_TERMINATION_DISABLE, REG05_BQ25601D_TERMINATION_MASK);

	return rc;
}

static int bq25601d_kick_wdt(void)
{
	int rc = 0;
	struct chip_bq25601d *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		chg_err("charger in suspended.\n");
		return 0;
	}

	rc = bq25601d_config_interface(chip, REG01_BQ25601D_ADDRESS,
			REG01_BQ25601D_WDT_TIMER_RESET, REG01_BQ25601D_WDT_TIMER_RESET_MASK);
	if (rc < 0) {
		chg_err("Couldn't bq25601d_kick_wdt rc = %d\n", rc);
	}

	return rc;
}

#ifdef CONFIG_MTK_PMIC_CHIP_MT6353
static bool bq25601d_check_charger_suspend_enable(void);
static int bq25601d_enable_charging(void)
{
	int rc = 0;
	struct chip_bq25601d *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		chg_err("charger in suspended.\n");
		return 0;
	}

#ifdef CONFIG_OPLUS_CHARGER_MTK
	if (get_boot_mode() == META_BOOT || get_boot_mode() == FACTORY_BOOT
		|| get_boot_mode() == ADVMETA_BOOT
		|| get_boot_mode() == ATE_FACTORY_BOOT) {
		return 0;
	}
#endif /* CONFIG_OPLUS_CHARGER_MTK */

	if (bq25601d_check_charger_suspend_enable()) {
		bq25601d_unsuspend_charger();
	}

	rc = bq25601d_config_interface(chip, REG01_BQ25601D_ADDRESS,
			REG01_BQ25601D_CHARGING_ENABLE, REG01_BQ25601D_CHARGING_MASK);
	if (rc < 0) {
		chg_err("Couldn't bq25601d_enable_charging rc = %d\n", rc);
	}
	return rc;
}

static int bq25601d_disable_charging(void)
{
	int rc = 0;
	struct chip_bq25601d *chip = charger_ic;

	/* Only for BATTERY_STATUS__HIGH_TEMP or BATTERY_STATUS__WARM_TEMP, */
	/* system power is from charger but NOT from battery to avoid temperature rise problem */
	if (atomic_read(&chip->charger_suspended) == 1) {
		chg_err("charger in suspended.\n");
		return 0;
	}

	rc = bq25601d_config_interface(chip, REG01_BQ25601D_ADDRESS,
			REG01_BQ25601D_CHARGING_DISABLE, REG01_BQ25601D_CHARGING_MASK);
	if (rc < 0) {
		chg_err("Couldn't bq25601d_disable_charging  rc = %d\n", rc);
	} else {
		chg_err("battery HIGH_TEMP or WARM_TEMP, bq25601d_disable_charging\n");
	}
	return rc;
}
#else /* CONFIG_MTK_PMIC_CHIP_MT6353 */
static int bq25601d_enable_charging(void)
{
	int rc = 0;
	struct chip_bq25601d *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		chg_err("charger in suspended.\n");
		return 0;
	}

	rc = bq25601d_config_interface(chip, REG01_BQ25601D_ADDRESS,
			REG01_BQ25601D_CHARGING_ENABLE, REG01_BQ25601D_CHARGING_MASK);
	if (rc < 0) {
		chg_err("Couldn'tbq25601d_enable_charging rc = %d\n", rc);
	}

	return rc;
}

static int bq25601d_disable_charging(void)
{
	int rc = 0;
	struct chip_bq25601d *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		chg_err("charger in suspended.\n");
		return 0;
	}

	rc = bq25601d_config_interface(chip, REG01_BQ25601D_ADDRESS,
			REG01_BQ25601D_CHARGING_DISABLE, REG01_BQ25601D_CHARGING_MASK);
	if (rc < 0) {
		chg_err("Couldn't bq25601d_disable_charging  rc = %d\n", rc);
	}

	return rc;
}
#endif /*CONFIG_MTK_PMIC_CHIP_MT6353*/

#ifdef CONFIG_MTK_PMIC_CHIP_MT6353
static bool bq25601d_check_charger_suspend_enable(void)
{
	int rc = 0;
	int reg_val = 0;
	bool charger_suspend_enable = false;
	struct chip_bq25601d *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		chg_err("charger in suspended.\n");
		return 0;
	}

	rc = bq25601d_read_reg(chip, REG00_BQ25601D_ADDRESS, &reg_val);
	if (rc) {
		chg_err("Couldn't read REG00_BQ25601D_ADDRESS rc = %d\n", rc);
		return 0;
	}

	charger_suspend_enable = ((reg_val & REG00_BQ25601D_SUSPEND_MODE_MASK) == REG00_BQ25601D_SUSPEND_MODE_ENABLE) ? true : false;

	return charger_suspend_enable;
}

static int bq25601d_check_charging_enable(void)
{
	bool rc = 0;
	struct chip_bq25601d *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		chg_err("charger in suspended.\n");
		return 0;
	}

	rc = bq25601d_check_charger_suspend_enable();
	if (rc)
		return 0;
	else
		return 1;
}
#else /* CONFIG_MTK_PMIC_CHIP_MT6353 */
static int bq25601d_check_charging_enable(void)
{
	int rc = 0;
	int reg_val = 0;
	bool charging_enable = false;
	struct chip_bq25601d *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		chg_err("charger in suspended.\n");
		return 0;
	}

	rc = bq25601d_read_reg(chip, REG01_BQ25601D_ADDRESS, &reg_val);
	if (rc) {
		chg_err("Couldn't read REG01_BQ25601D_ADDRESS rc = %d\n", rc);
		return 0;
	}

	charging_enable = ((reg_val & REG01_BQ25601D_CHARGING_MASK) == REG01_BQ25601D_CHARGING_ENABLE) ? 1 : 0;

	return charging_enable;
}
#endif /*CONFIG_MTK_PMIC_CHIP_MT6353*/

static int bq25601d_registers_read_full(void)
{
	int rc = 0;
	int reg_full = 0;
	int reg_ovp = 0;
	struct chip_bq25601d *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		chg_err("charger in suspended.\n");
		return 0;
	}

	rc = bq25601d_read_reg(chip, REG08_BQ25601D_ADDRESS, &reg_full);
	if (rc) {
		chg_err("Couldn't read REG08_BQ25601D_ADDRESS rc = %d\n", rc);
		return 0;
	}
	reg_full = ((reg_full & REG08_BQ25601D_CHG_STAT_MASK) == REG08_BQ25601D_CHG_STAT_CHG_TERMINATION) ? 1 : 0;

	rc = bq25601d_read_reg(chip, REG09_BQ25601D_ADDRESS, &reg_ovp);
	if (rc) {
		chg_err("Couldn't read REG09_BQ25601D_ADDRESS rc = %d\n", rc);
		return 0;
	}
	reg_ovp = ((reg_ovp & REG09_BQ25601D_BAT_FAULT_MASK) == REG09_BQ25601D_BAT_FAULT_BATOVP) ? 1 : 0;

	//chg_err("bq25601d_registers_read_full, reg_full = %d, reg_ovp = %d\r\n", reg_full, reg_ovp);
	return (reg_full || reg_ovp);
}

static int oplus_otg_online = 0;

int bq25601d_otg_enable(void)
{
	int rc = 0;
	struct chip_bq25601d *chip = charger_ic;

	if (!chip) {
		return 0;
	}
#ifndef CONFIG_OPLUS_CHARGER_MTK
	if (atomic_read(&chip->charger_suspended) == 1) {
		chg_err("charger in suspended.\n");
		return 0;
	}
#endif /* CONFIG_OPLUS_CHARGER_MTK */

	bq25601d_config_interface(chip, REG00_BQ25601D_ADDRESS,
			REG00_BQ25601D_SUSPEND_MODE_DISABLE, REG00_BQ25601D_SUSPEND_MODE_MASK);

	rc = bq25601d_config_interface(chip, REG01_BQ25601D_ADDRESS,
			REG01_BQ25601D_OTG_ENABLE, REG01_BQ25601D_OTG_MASK);
	if (rc) {
		chg_err("Couldn't enable OTG mode rc=%d\n", rc);
	} else {
		chg_debug("bq25601d_otg_enable rc=%d\n", rc);
	}
	
	bq25601d_set_wdt_timer(REG05_BQ25601D_WATCHDOG_TIMER_DISABLE);
	oplus_otg_online = 1;
	oplus_chg_set_otg_online(true);
	return rc;
}

int bq25601d_otg_disable(void)
{
	int rc = 0;
	int val_buf;
	struct chip_bq25601d *chip = charger_ic;

	if (!chip) {
		return 0;
	}
#ifndef CONFIG_OPLUS_CHARGER_MTK
	if (atomic_read(&chip->charger_suspended) == 1) {
		chg_err("charger in suspended.\n");
		return 0;
	}
#endif /* CONFIG_OPLUS_CHARGER_MTK */

	rc = bq25601d_config_interface(chip, REG01_BQ25601D_ADDRESS,
			REG01_BQ25601D_OTG_DISABLE, REG01_BQ25601D_OTG_MASK);
	if (rc)
		chg_err("Couldn't disable OTG mode rc=%d\n", rc);
	else
		chg_debug("bq25601d_otg_disable rc=%d\n", rc);
	
	bq25601d_set_wdt_timer(REG05_BQ25601D_WATCHDOG_TIMER_40S);
	oplus_otg_online = 0;
	oplus_chg_set_otg_online(false);

	return rc;
}

static int bq25601d_suspend_charger(void)
{
	int rc = 0;
	struct chip_bq25601d *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		chg_err("charger in suspended.\n");
		return 0;
	}

	if (oplus_chg_get_otg_online() == true)
		return 0;

	if (oplus_otg_online == 1)
		return 0;

	rc = bq25601d_config_interface(chip, REG00_BQ25601D_ADDRESS,
			REG00_BQ25601D_SUSPEND_MODE_ENABLE, REG00_BQ25601D_SUSPEND_MODE_MASK);
	if (rc < 0) {
		chg_err("Couldn't bq25601d_suspend_charger rc = %d\n", rc);
	} else {
		chg_debug("rc = %d\n", rc);
	}

	return rc;
}

static int bq25601d_unsuspend_charger(void)
{
	int rc = 0;
	struct chip_bq25601d *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		chg_err("charger in suspended.\n");
		return 0;
	}

	rc = bq25601d_config_interface(chip, REG00_BQ25601D_ADDRESS,
			REG00_BQ25601D_SUSPEND_MODE_DISABLE, REG00_BQ25601D_SUSPEND_MODE_MASK);
	if (rc < 0) {
		chg_err("Couldn't bq25601d_unsuspend_charger rc = %d\n", rc);
	} else {
		chg_debug("rc = %d\n", rc);
	}

	return rc;
}

#if defined (CONFIG_MTK_PMIC_CHIP_MT6353) || defined(CONFIG_OPLUS_CHARGER_MTK6763) || defined(CONFIG_OPLUS_CHARGER_MTK6771)
static int bq25601d_reset_charger(void)
{
	int rc = 0;
	struct chip_bq25601d *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		chg_err("charger in suspended.\n");
		return 0;
	}

	rc = bq25601d_config_interface(chip, REG00_BQ25601D_ADDRESS, 0x0, 0xFF);
	if (rc < 0) {
		chg_err("Couldn't reset REG00 rc = %d\n", rc);
	}
	rc = bq25601d_config_interface(chip, REG01_BQ25601D_ADDRESS, 0x1A, 0xFF);
	if (rc < 0) {
		chg_err("Couldn't reset REG01 rc = %d\n", rc);
	}
	rc = bq25601d_config_interface(chip, REG02_BQ25601D_ADDRESS, 0xA2, 0xFF);
	if (rc < 0) {
		chg_err("Couldn't reset REG02 rc = %d\n", rc);
	}
	rc = bq25601d_config_interface(chip, REG03_BQ25601D_ADDRESS, 0x22, 0xFF);
	if (rc < 0) {
		chg_err("Couldn't reset REG03 rc = %d\n", rc);
	}
	rc = bq25601d_config_interface(chip, REG04_BQ25601D_ADDRESS, 0x88, 0xFF);
	if (rc < 0) {
		chg_err("Couldn't reset REG04 rc = %d\n", rc);
	}
	rc = bq25601d_config_interface(chip, REG05_BQ25601D_ADDRESS, 0x1F, 0xFF);
	if (rc < 0) {
		chg_err("Couldn't reset REG05 rc = %d\n", rc);
	}
	rc = bq25601d_config_interface(chip, REG06_BQ25601D_ADDRESS, 0x66, 0xFF);
	if (rc < 0) {
		chg_err("Couldn't reset REG06 rc = %d\n", rc);
	}
	rc = bq25601d_config_interface(chip, REG07_BQ25601D_ADDRESS, 0x4C, 0xFF);
	if (rc < 0) {
		chg_err("Couldn't reset REG07 rc = %d\n", rc);
	}
	rc = bq25601d_config_interface(chip, REG0A_BQ25601D_ADDRESS, 0x0, 0x03);
	if (rc < 0) {
		chg_err("Couldn't reset REG0A rc = %d\n", rc);
	}
	chg_debug("rc = %d\n", rc);

	return rc;
}
#else /*defined (CONFIG_MTK_PMIC_CHIP_MT6353) || defined(CONFIG_OPLUS_CHARGER_MTK6763) || defined(CONFIG_OPLUS_CHARGER_MTK6771)*/
static int bq25601d_reset_charger(void)
{
	int rc = 0;
	struct chip_bq25601d *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		chg_err("charger in suspended.\n");
		return 0;
	}

	rc = bq25601d_config_interface(chip, REG0B_BQ25601D_ADDRESS,
			REG0B_BQ25601D_REG_RST_RESET, REG0B_BQ25601D_REG_RST_MASK);
	if (rc < 0) {
		chg_err("Couldn't bq25601d_reset_charger rc = %d\n", rc);
	} else {
		chg_debug("rc = %d\n", rc);
	}

	return rc;
}
#endif /*defined (CONFIG_MTK_PMIC_CHIP_MT6353) || defined(CONFIG_OPLUS_CHARGER_MTK6763) || defined(CONFIG_OPLUS_CHARGER_MTK6771)*/

static bool bq25601d_check_charger_resume(void)
{
	struct chip_bq25601d *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		return false;
	} else {
		return true;
	}
}

#define DUMP_REG_LOG_CNT_30S  1
static void bq25601d_dump_registers(void)
{
	int rc = 0;
	int addr = 0;
	unsigned int val_buf[BQ25601D_REG_NUMBER] = {0x0};
	static int dump_count = 0;
	struct chip_bq25601d *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		chg_err("charger in suspended.\n");
		return;
	}

	for (addr = BQ25601D_FIRST_REG; addr <= BQ25601D_LAST_REG; addr++) {
		rc = bq25601d_read_reg(chip, addr, &val_buf[addr]);
		if (rc) {
			 chg_err("Couldn't read 0x%02x rc = %d\n", addr, rc);
			 return;
		}
	}

	if (dump_count == DUMP_REG_LOG_CNT_30S) {
		dump_count = 0;
		chg_debug("bq25601d_reg: [0x%02x,0x%02x,0x%02x,0x%02x], [0x%02x,0x%02x,0x%02x,0x%02x], [0x%02x,0x%02x,0x%02x,0x%02x]\n",
			val_buf[0], val_buf[1], val_buf[2], val_buf[3], val_buf[4], val_buf[5], 
			val_buf[6], val_buf[7], val_buf[8], val_buf[9], val_buf[10], val_buf[11]);
	}
	dump_count++;

	return;
}

static int bq25601d_check_registers(void)
{
	int rc = 0;
	int addr = 0;
	unsigned int val_buf[BQ25601D_REG_NUMBER] = {0x0};
	struct chip_bq25601d *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		chg_err("charger in suspended.\n");
		return 0;
	}

	for (addr = BQ25601D_FIRST_REG; addr <= BQ25601D_LAST_REG; addr++) {
		rc = bq25601d_read_reg(chip, addr, &val_buf[addr]);
		if (rc) {
			 chg_err("Couldn't read 0x%02x rc = %d\n", addr, rc);
			 return -1;
		}
	}

  	return 0;
}

static int bq25601d_get_chg_current_step(void)
{
	return 60;
}

static int bq25601d_hardware_init(void)
{
	struct chip_bq25601d *chip = charger_ic;

	/* must be before set_vindpm_vol and set_input_current */
	chip->hw_aicl_point = 4400;
	chip->sw_aicl_point = 4500;

	bq25601d_reset_charger();

	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT) {
#ifndef CONFIG_MTK_PMIC_CHIP_MT6353
		bq25601d_disable_charging();
#endif /* CONFIG_MTK_PMIC_CHIP_MT6353 */
		bq25601d_float_voltage_write(4400);
		msleep(100);
	}

	bq25601d_float_voltage_write(4370);

	bq25601d_set_enable_volatile_writes();

	bq25601d_set_complete_charge_timeout(OVERTIME_DISABLED);

	bq25601d_set_prechg_current(300);

	bq25601d_charging_current_write_fast(512);

	bq25601d_set_termchg_current(150);

	bq25601d_set_rechg_voltage(100);

	bq25601d_set_vindpm_vol(chip->hw_aicl_point);

#ifdef CONFIG_OPLUS_CHARGER_MTK
	if (get_boot_mode() == META_BOOT || get_boot_mode() == FACTORY_BOOT
			|| get_boot_mode() == ADVMETA_BOOT || get_boot_mode() == ATE_FACTORY_BOOT) {
		bq25601d_suspend_charger();
		bq25601d_disable_charging();
	} else {
		bq25601d_unsuspend_charger();
	}
#else /* CONFIG_OPLUS_CHARGER_MTK */
	bq25601d_unsuspend_charger();
#endif /* CONFIG_OPLUS_CHARGER_MTK */

	bq25601d_enable_charging();

	bq25601d_set_wdt_timer(REG05_BQ25601D_WATCHDOG_TIMER_40S);

	return true;
}

#ifdef CONFIG_OPLUS_RTC_DET_SUPPORT
static int rtc_reset_check(void)
{
	struct rtc_time tm;
	struct rtc_device *rtc;
	int rc = 0;

	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (rtc == NULL) {
		pr_err("%s: unable to open rtc device (%s)\n",
			__FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
		return 0;
	}

	rc = rtc_read_time(rtc, &tm);
	if (rc) {
		pr_err("Error reading rtc device (%s) : %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	rc = rtc_valid_tm(&tm);
	if (rc) {
		pr_err("Invalid RTC time (%s): %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	if ((tm.tm_year == 70) && (tm.tm_mon == 0) && (tm.tm_mday <= 1)) {
		chg_debug(": Sec: %d, Min: %d, Hour: %d, Day: %d, Mon: %d, Year: %d  @@@ wday: %d, yday: %d, isdst: %d\n",
			tm.tm_sec, tm.tm_min, tm.tm_hour, tm.tm_mday, tm.tm_mon, tm.tm_year,
			tm.tm_wday, tm.tm_yday, tm.tm_isdst);
		rtc_class_close(rtc);
		return 1;
	}

	chg_debug(": Sec: %d, Min: %d, Hour: %d, Day: %d, Mon: %d, Year: %d  ###  wday: %d, yday: %d, isdst: %d\n",
		tm.tm_sec, tm.tm_min, tm.tm_hour, tm.tm_mday, tm.tm_mon, tm.tm_year,
		tm.tm_wday, tm.tm_yday, tm.tm_isdst);

close_time:
	rtc_class_close(rtc);
	return 0;
}
#endif /* CONFIG_OPLUS_RTC_DET_SUPPORT */

extern bool oplus_chg_get_shortc_hw_gpio_status(void);

struct oplus_chg_operations  bq25601d_chg_ops = {
	.dump_registers = bq25601d_dump_registers,
	.kick_wdt = bq25601d_kick_wdt,
	.hardware_init = bq25601d_hardware_init,
	.charging_current_write_fast = bq25601d_charging_current_write_fast,
	.set_aicl_point = bq25601d_set_aicl_point,
	.input_current_write = bq25601d_input_current_limit_write,
	.float_voltage_write = bq25601d_float_voltage_write,
	.term_current_set = bq25601d_set_termchg_current,
	.charging_enable = bq25601d_enable_charging,
	.charging_disable = bq25601d_disable_charging,
	.get_charging_enable = bq25601d_check_charging_enable,
	.charger_suspend = bq25601d_suspend_charger,
	.charger_unsuspend = bq25601d_unsuspend_charger,
	.set_rechg_vol = bq25601d_set_rechg_voltage,
	.reset_charger = bq25601d_reset_charger,
	.read_full = bq25601d_registers_read_full,
	.otg_enable = bq25601d_otg_enable,
	.otg_disable = bq25601d_otg_disable,
	.set_charging_term_disable = bq25601d_set_chging_term_disable,
	.check_charger_resume = bq25601d_check_charger_resume,
	.get_chg_current_step = bq25601d_get_chg_current_step,
#ifdef CONFIG_OPLUS_CHARGER_MTK
	.get_charger_type = mt_power_supply_type_check,
	.get_charger_volt = battery_meter_get_charger_voltage,
	.check_chrdet_status = (bool (*) (void)) pmic_chrdet_status,
	.get_instant_vbatt = oplus_battery_meter_get_battery_voltage,
	.get_boot_mode = (int (*)(void))get_boot_mode,
	.get_boot_reason = (int (*)(void))get_boot_reason,
	.get_chargerid_volt = mt_get_chargerid_volt,
	.set_chargerid_switch_val = mt_set_chargerid_switch_val ,
	.get_chargerid_switch_val  = mt_get_chargerid_switch_val,
#ifdef CONFIG_MTK_HAFG_20
	.get_rtc_soc = get_rtc_spare_oplus_fg_value,
	.set_rtc_soc = set_rtc_spare_oplus_fg_value,
#elif defined(CONFIG_OPLUS_CHARGER_MTK6771)
	.get_rtc_soc = oplus_get_rtc_ui_soc,
	.set_rtc_soc = oplus_set_rtc_ui_soc,
#elif defined(CONFIG_OPLUS_CHARGER_MTK6763)
	.get_rtc_soc = get_rtc_spare_oplus_fg_value,
	.set_rtc_soc = set_rtc_spare_oplus_fg_value,
#else /* CONFIG_MTK_HAFG_20 */
	.get_rtc_soc = oplus_get_rtc_ui_soc,
	.set_rtc_soc = set_rtc_spare_fg_value,
#endif /* CONFIG_MTK_HAFG_20 */
	.set_power_off = mt_power_off,
	.usb_connect = mt_usb_connect,
	.usb_disconnect = mt_usb_disconnect,
#else /* CONFIG_OPLUS_CHARGER_MTK */
	.get_charger_type = qpnp_charger_type_get,
	.get_charger_volt = qpnp_get_prop_charger_voltage_now,
	.check_chrdet_status = qpnp_lbc_is_usb_chg_plugged_in,
	.get_instant_vbatt = qpnp_get_prop_battery_voltage_now,
	.get_boot_mode = get_boot_mode,
	.get_rtc_soc = qpnp_get_pmic_soc_memory,
	.set_rtc_soc = qpnp_set_pmic_soc_memory,
#endif /* CONFIG_OPLUS_CHARGER_MTK */

#ifdef CONFIG_OPLUS_SHORT_C_BATT_CHECK
	.get_dyna_aicl_result = bq25601d_chg_get_dyna_aicl_result,
#endif
	.get_shortc_hw_gpio_status = oplus_chg_get_shortc_hw_gpio_status,
#ifdef CONFIG_OPLUS_RTC_DET_SUPPORT
	.check_rtc_reset = rtc_reset_check,
#endif
};

static void register_charger_devinfo(void)
{
	int ret = 0;
	char *version = "bq25601d";
	char *manufacture = "TI";
	ret = register_device_proc("charger", version, manufacture);
	if (ret)
		chg_err("register_charger_devinfo fail\n");
}

extern void Charger_Detect_Init(void);
extern void Charger_Detect_Release(void);
extern bool is_usb_rdy(void);
static void hw_bc12_init(void)
{
	int timeout = 350;
	static bool first_connect = true;

	if (first_connect == true) {
		/* add make sure USB Ready */
		if (is_usb_rdy() == false) {
			pr_err("CDP, block\n");
			while (is_usb_rdy() == false && timeout > 0) {
				msleep(100);
				timeout--;
			}
			if (timeout == 0) {
				pr_err("CDP, timeout\n");
			} else {
				pr_err("CDP, free, timeout:%d\n", timeout);
			}
		} else {
			pr_err("CDP, PASS\n");
		}
		first_connect = false;
	}
	Charger_Detect_Init();
}

//static DEVICE_ATTR(bq25601d_access, 0664, show_bq25601d_access, store_bq25601d_access);
#ifdef OPLUS_FEATURE_CHG_BASIC
extern enum charger_type MTK_CHR_Type_num;
extern unsigned int upmu_get_rgs_chrdet(void);

enum charger_type mt_charger_type_detection_bq25601d(void)
{
	int rc = 0;
	int chgr_type_addr = 0;
	int val_buf = 0;
	int count = 0;
	int i = 0;

	if (!charger_ic) {
		pr_err("%s charger_ic null, return\n", __func__);
		return CHARGER_UNKNOWN;
	}

	if (get_boot_mode() == META_BOOT || get_boot_mode() == FACTORY_BOOT
			|| get_boot_mode() == ADVMETA_BOOT
			|| get_boot_mode() == ATE_FACTORY_BOOT) {
		return STANDARD_HOST;
	}

	chgr_type_addr = REG08_BQ25601D_ADDRESS;

	printk("mt_charger_type_detection0\n");
	for (i = 0; i < 15; i++) {
		usleep_range(20000, 20200);
		if (!upmu_get_rgs_chrdet()) {
			MTK_CHR_Type_num = CHARGER_UNKNOWN;
			return MTK_CHR_Type_num;
		}
	}

	hw_bc12_init();
	usleep_range(10000, 10200);

	printk("mt_charger_type_detection1\n");
	bq25601d_config_interface(charger_ic, REG07_BQ25601D_ADDRESS,
			REG07_BQ25601D_IINDET_EN_FORCE_DET, REG07_BQ25601D_IINDET_EN_MASE);
	bq25601d_dump_registers();

	rc = bq25601d_read_reg(charger_ic, REG07_BQ25601D_ADDRESS, &val_buf);
	val_buf &= REG07_BQ25601D_IINDET_EN_MASE;
	while (val_buf == REG07_BQ25601D_IINDET_EN_FORCE_DET && count < 20) {
		count++;
		rc = bq25601d_read_reg(charger_ic, REG07_BQ25601D_ADDRESS, &val_buf);
		val_buf &= REG07_BQ25601D_IINDET_EN_MASE;
		usleep_range(50000, 50200);
		if (!upmu_get_rgs_chrdet()) {
			MTK_CHR_Type_num = CHARGER_UNKNOWN;
			return MTK_CHR_Type_num;
		}
	}

	printk("mt_charger_type_detection2, count=%d\n", count);
	if (count == 20) {
		MTK_CHR_Type_num = CHARGER_UNKNOWN;
		return MTK_CHR_Type_num;
	}

	rc = bq25601d_read_reg(charger_ic, chgr_type_addr, &val_buf);
	bq25601d_dump_registers();
	val_buf = val_buf & REG08_BQ25601D_VBUS_STAT_MASK;
	printk("mt_charger_type_detection3, val_buf=[0x%x]\n", val_buf);

	if (val_buf == REG08_BQ25601D_VBUS_STAT_UNKNOWN) {
		MTK_CHR_Type_num = CHARGER_UNKNOWN;
	} else if (val_buf == REG08_BQ25601D_VBUS_STAT_SDP) {
		MTK_CHR_Type_num = STANDARD_HOST;
	} else if (val_buf == REG08_BQ25601D_VBUS_STAT_CDP) {
		MTK_CHR_Type_num = CHARGING_HOST;
	} else if (val_buf == REG08_BQ25601D_VBUS_STAT_DCP
			|| val_buf == REG08_BQ25601D_VBUS_STAT_OCP
			|| val_buf == REG08_BQ25601D_VBUS_STAT_FLOAT) {
		MTK_CHR_Type_num = APPLE_2_1A_CHARGER;
	} else {
		MTK_CHR_Type_num = CHARGER_UNKNOWN;
	}

	/* the 2nd detection */
	if (MTK_CHR_Type_num == CHARGER_UNKNOWN && upmu_get_rgs_chrdet()) {
		printk("mt_charger_type_detection: 2nd...\n");
		bq25601d_config_interface(charger_ic, REG07_BQ25601D_ADDRESS,
				REG07_BQ25601D_IINDET_EN_FORCE_DET, REG07_BQ25601D_IINDET_EN_MASE);
		bq25601d_dump_registers();

		rc = bq25601d_read_reg(charger_ic, REG07_BQ25601D_ADDRESS, &val_buf);
		val_buf &= REG07_BQ25601D_IINDET_EN_MASE;
		count = 0;
		while (val_buf == REG07_BQ25601D_IINDET_EN_FORCE_DET && count < 20) {
			count++;
			rc = bq25601d_read_reg(charger_ic, REG07_BQ25601D_ADDRESS, &val_buf);
			val_buf &= REG07_BQ25601D_IINDET_EN_MASE;
			usleep_range(50000, 50200);
			if (!upmu_get_rgs_chrdet()) {
				MTK_CHR_Type_num = CHARGER_UNKNOWN;
				return MTK_CHR_Type_num;
			}
		}

		printk("mt_charger_type_detection: 2nd, count=%d\n", count);
		if (count == 20) {
			MTK_CHR_Type_num = APPLE_2_1A_CHARGER;
			return MTK_CHR_Type_num;
		}

		rc = bq25601d_read_reg(charger_ic, chgr_type_addr, &val_buf);
		bq25601d_dump_registers();
		val_buf = val_buf & REG08_BQ25601D_VBUS_STAT_MASK;
		printk("mt_charger_type_detection: 2nd, val_buf=[0x%x]\n", val_buf);
		if (val_buf == REG08_BQ25601D_VBUS_STAT_UNKNOWN) {
			MTK_CHR_Type_num = APPLE_2_1A_CHARGER;
		} else if (val_buf == REG08_BQ25601D_VBUS_STAT_SDP) {
			MTK_CHR_Type_num = STANDARD_HOST;
		} else if (val_buf == REG08_BQ25601D_VBUS_STAT_CDP) {
			MTK_CHR_Type_num = CHARGING_HOST;
		} else if (val_buf == REG08_BQ25601D_VBUS_STAT_DCP
				|| val_buf == REG08_BQ25601D_VBUS_STAT_OCP
				|| val_buf == REG08_BQ25601D_VBUS_STAT_FLOAT) {
			MTK_CHR_Type_num = APPLE_2_1A_CHARGER;
		} else {
			MTK_CHR_Type_num = APPLE_2_1A_CHARGER;
		}
	}

	Charger_Detect_Release();

	return MTK_CHR_Type_num;
}
#if 0
enum charger_type mt_charger_type_detection_bq25601d(void)
{
	int rc =0;
	int addr = 0;
	int val_buf;
	int count = 0;
	int i;

	if (!charger_ic) {
		pr_err("%s charger_ic null, return\n", __func__);
		return CHARGER_UNKNOWN;
	}
	if (get_boot_mode() == META_BOOT || get_boot_mode() == FACTORY_BOOT
			|| get_boot_mode() == ADVMETA_BOOT || get_boot_mode() == ATE_FACTORY_BOOT) {
		return STANDARD_HOST;
	}
	addr = BQ25601D_FIRST_REG + 8;
	printk("mt_charger_type_detection0\n");
	hw_bc12_init();
	usleep_range(40000, 40200);
	printk("mt_charger_type_detection1\n");
	bq25601d_config_interface(charger_ic, REG00_BQ25601D_ADDRESS, 0, 0x80);
	usleep_range(5000, 5200);
	bq25601d_config_interface(charger_ic, REG07_BQ25601D_ADDRESS, 0x80, 0);
	bq25601d_dump_registers();
	rc = bq25601d_read_reg(charger_ic, REG07_BQ25601D_ADDRESS, &val_buf);
	while (val_buf == 0xcb && count < 20) {
		count++;
		rc = bq25601d_read_reg(charger_ic, REG07_BQ25601D_ADDRESS, &val_buf);
		usleep_range(50000, 50200);
		if (!upmu_get_rgs_chrdet()) {
			MTK_CHR_Type_num = CHARGER_UNKNOWN;
			return MTK_CHR_Type_num;
		}
	}
	if (count == 20) {
		MTK_CHR_Type_num = CHARGER_UNKNOWN;
		chg_debug("count time out,return apple adapter\n");
		return MTK_CHR_Type_num;
	}
	printk("mt_charger_type_detection2\n");
	rc = bq25601d_read_reg(charger_ic, addr, &val_buf);
	printk("mt_charger_type_detection\n");
	bq25601d_dump_registers();
	val_buf = val_buf & 0xc0;
	printk("val_buf = 0x%x\n",val_buf);
	if (val_buf == 0) {
		MTK_CHR_Type_num = CHARGER_UNKNOWN;
	}
	else if (val_buf == 0x40) {
		MTK_CHR_Type_num = STANDARD_HOST;
	}
	else if (val_buf == 0x80){
		MTK_CHR_Type_num = APPLE_2_1A_CHARGER;
	}
	else {
		MTK_CHR_Type_num = CHARGER_UNKNOWN;
	}
	if ((MTK_CHR_Type_num == CHARGER_UNKNOWN || MTK_CHR_Type_num == STANDARD_HOST) && upmu_get_rgs_chrdet()) {
		if (MTK_CHR_Type_num == STANDARD_HOST) {
			for (i = 0; i < 4; i++) {
				usleep_range(100000, 100020);
			}
		}
		bq25601d_config_interface(charger_ic, REG00_BQ25601D_ADDRESS, 0, 0x80);
		usleep_range(5000, 5020);
		bq25601d_config_interface(charger_ic, REG07_BQ25601D_ADDRESS, 0x80, 0);
		bq25601d_dump_registers();
		rc = bq25601d_read_reg(charger_ic, REG07_BQ25601D_ADDRESS, &val_buf);
		while (val_buf == 0xcb && count <20) {
			count++;
			rc = bq25601d_read_reg(charger_ic, REG07_BQ25601D_ADDRESS, &val_buf);
			usleep_range(50000, 50200);
			if (!upmu_get_rgs_chrdet()) {
				MTK_CHR_Type_num = CHARGER_UNKNOWN;
				return MTK_CHR_Type_num;
			}
		}
		if (count == 20) {
			MTK_CHR_Type_num = APPLE_2_1A_CHARGER;
			chg_debug("count time out,return apple adapter\n");
			return MTK_CHR_Type_num;
		}
		rc = bq25601d_read_reg(charger_ic, addr, &val_buf);
		bq25601d_dump_registers();
		val_buf = val_buf & 0xc0;
		printk("val_buf =0x%x\n",val_buf);
		if (val_buf == 0) {
			MTK_CHR_Type_num = APPLE_2_1A_CHARGER;
		}
		else if(val_buf == 0x40) {
			MTK_CHR_Type_num = STANDARD_HOST;
		}
		else if(val_buf == 0x80){
			MTK_CHR_Type_num = APPLE_2_1A_CHARGER;
		}
		else {
			MTK_CHR_Type_num = CHARGER_UNKNOWN;
		}
	}
	chg_debug("chr_type:%d\n",MTK_CHR_Type_num);
	Charger_Detect_Release();
	//MTK_CHR_Type_num = APPLE_2_1A_CHARGER;

	return MTK_CHR_Type_num;
}
#endif
#endif

static void do_charger_modefy_work(struct work_struct *data)
{
	/*
	if (charger_ic != NULL) {
		mt_charger_type_detection_bq25601d();
	}
	*/
}
	
static struct delayed_work bq25601d_irq_delay_work;
bool fg_bq25601d_irq_delay_work_running = false;
static void do_bq25601d_irq_delay_work(struct work_struct *data)
{
	int val_buf = 0;
	int val_reg = 0;
	int i = 0;
	int otg_overcurrent_flag = 0;
	struct chip_bq25601d *chip = charger_ic;
	
	if (!chip) {
		pr_err("%s chip null,return\n", __func__);
		return;
	}

	for (i = 0; i < 10; i++) {
		bq25601d_read_reg(chip, REG09_BQ25601D_ADDRESS, &val_reg);
		val_buf = val_reg & 0x40;
		if (val_buf == 0x40) {
			otg_overcurrent_flag++;
		}
		
		usleep_range(10000, 10200);
	}
	printk("[OPLUS_CHG] do_bq25601d_irq_delay_work disable vbus out flag=%d, val_reg[0x%x]\n", otg_overcurrent_flag, val_reg);
	if (otg_overcurrent_flag >= 8) {
		bq25601d_otg_disable();
	}
	fg_bq25601d_irq_delay_work_running = false;
	return; 
}

static irqreturn_t bq25601d_irq_handler_fn(int irq, void *dev_id)
{
	//int val_buf;
	struct chip_bq25601d *chip = charger_ic;

	pr_err("%s start\n", __func__);
	if (!chip) {
		pr_err("%s chip null,return\n", __func__);
		return IRQ_HANDLED;
	}
	if (oplus_otg_online == 0) {
		return IRQ_HANDLED;
	}
	if (fg_bq25601d_irq_delay_work_running == false) {
		fg_bq25601d_irq_delay_work_running = true;
		schedule_delayed_work(&bq25601d_irq_delay_work, round_jiffies_relative(msecs_to_jiffies(50)));
	}

	return IRQ_HANDLED;
}

static int bq25601d_parse_dts(void)
{
	int ret = 0;
	struct chip_bq25601d *chip = charger_ic;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -1;
	}

	chip->irq_gpio = of_get_named_gpio(chip->client->dev.of_node, "chg-irq-gpio", 0);
	if (chip->irq_gpio <= 0) {
		chg_err("Couldn't read chg-irq-gpio:%d\n", chip->irq_gpio);
		return -1;
	} else {
		if (gpio_is_valid(chip->irq_gpio)) {
			ret = gpio_request(chip->irq_gpio, "chg-irq-gpio");
			if (ret) {
				chg_err("unable to request chg-irq-gpio[%d]\n", chip->irq_gpio);
				chip->irq_gpio = -EINVAL;
			} else {
				gpio_direction_input(chip->irq_gpio);
			}
		} else {
			chg_err("gpio_is_valid fail chg-irq-gpio[%d]\n", chip->irq_gpio);
			chip->irq_gpio = -EINVAL;
			return -1;
		}
	}
	chg_err("chg-irq-gpio[%d]\n", chip->irq_gpio);
	return ret;
}

static int bq25601d_irq_registration(void)
{
	int ret = 0;
	struct chip_bq25601d *chip = charger_ic;

	if (chip->irq_gpio <= 0) {
		chg_err("chip->irq_gpio fail\n");
		return -1;
	}

	ret = request_threaded_irq(gpio_to_irq(chip->irq_gpio), NULL,
			bq25601d_irq_handler_fn,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			"BQ25601D-eint", chip);
	if (ret < 0) {
		printk("BQ25601D request_irq IRQ LINE NOT AVAILABLE!");
		return -EFAULT;
	}
	return 0;
}

extern int charger_ic_flag;
static int bq25601d_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int reg = 0;
	struct chip_bq25601d *chip = NULL;

#ifndef CONFIG_OPLUS_CHARGER_MTK
	struct power_supply *usb_psy;
	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		chg_err("USB psy not found; deferring probe\n");
		return -EPROBE_DEFER;
	}
#endif  /* CONFIG_OPLUS_CHARGER_MTK */
	chg_debug("call \n");

	chip = devm_kzalloc(&client->dev, sizeof(struct chip_bq25601d), GFP_KERNEL);
	if (!chip) {
		chg_err("kzalloc() failed\n");
		return -ENOMEM;
	}
	charger_ic = chip;
	chip->client = client;
	chip->dev = &client->dev;
	
	reg = bq25601d_check_registers();
	if (reg < 0) {
		return -ENODEV;
	}
	charger_ic_flag = 2;
	
	INIT_DELAYED_WORK(&bq25601d_irq_delay_work, do_bq25601d_irq_delay_work);
	bq25601d_parse_dts();
	bq25601d_irq_registration();
	atomic_set(&chip->charger_suspended, 0);

	register_charger_devinfo();
	usleep_range(1000, 1200);
	INIT_DELAYED_WORK(&charger_modefy_work, do_charger_modefy_work);
	schedule_delayed_work(&charger_modefy_work, 0);
	chg_debug("call OK!\n");
	return 0;
}

static struct i2c_driver bq25601d_i2c_driver;

static int bq25601d_driver_remove(struct i2c_client *client)
{
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
static int bq25601d_resume(struct device *dev)
{
	unsigned long resume_tm_sec = 0;
	unsigned long sleep_time = 0;
	int rc = 0;

	if (!charger_ic) {
		return 0;
	}
	atomic_set(&charger_ic->charger_suspended, 0);
	rc = get_current_time(&resume_tm_sec);
	if (rc || suspend_tm_sec == -1) {
		chg_err("RTC read failed\n");
		sleep_time = 0;
	} else {
		sleep_time = resume_tm_sec - suspend_tm_sec;
	}

	if (sleep_time < 0) {
		sleep_time = 0;
	}
	chg_err("resume_sec:%ld,sleep_time:%ld\n\n",resume_tm_sec,sleep_time);
	oplus_chg_soc_update_when_resume(sleep_time);
	return 0;
}

static int bq25601d_suspend(struct device *dev)
{
	if (!charger_ic) {
		return 0;
	}
	atomic_set(&charger_ic->charger_suspended, 1);
	if (get_current_time(&suspend_tm_sec)) {
		chg_err("RTC read failed\n");
		suspend_tm_sec = -1;
	}
	chg_err("suspend_sec:%ld\n",suspend_tm_sec);
	return 0;
}

static const struct dev_pm_ops bq25601d_pm_ops = {
	.resume		= bq25601d_resume,
	.suspend		= bq25601d_suspend,
};

#else //(LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))

static int bq25601d_resume(struct i2c_client *client)
{
	unsigned long resume_tm_sec = 0;
	unsigned long sleep_time = 0;
	int rc = 0;

	if (!charger_ic) {
		return 0;
	}
	atomic_set(&charger_ic->charger_suspended, 0);
	rc = get_current_time(&resume_tm_sec);
	if (rc || suspend_tm_sec == -1) {
		chg_err("RTC read failed\n");
		sleep_time = 0;
	} else {
		sleep_time = resume_tm_sec - suspend_tm_sec;
	}

	if (sleep_time < 0) {
		sleep_time = 0;
	}
	oplus_chg_soc_update_when_resume(sleep_time);
	return 0;
}

static int bq25601d_suspend(struct i2c_client *client, pm_message_t mesg)
{
	if (!charger_ic) {
		return 0;
	}
	atomic_set(&charger_ic->charger_suspended, 1);
	if (get_current_time(&suspend_tm_sec)) {
		chg_err("RTC read failed\n");
		suspend_tm_sec = -1;
	}
	return 0;
}
#endif //(LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))

static void bq25601d_reset(struct i2c_client *client)
{
	bq25601d_otg_disable();
}

/**********************************************************
 * *
 * *   [platform_driver API]
 * *
 * *********************************************************/

static const struct of_device_id bq25601d_match[] = {
	{ .compatible = "oplus,bq25601d-charger"},
	{ },
};

static const struct i2c_device_id bq25601d_id[] = {
	{"bq25601d-charger", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, bq25601d_id);

static struct i2c_driver bq25601d_i2c_driver = {
	.driver		= {
		.name			= "bq25601d-charger",
		.owner			= THIS_MODULE,
		.of_match_table	= bq25601d_match,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
		.pm				= &bq25601d_pm_ops,
#endif
	},
	.probe		= bq25601d_driver_probe,
	.remove		= bq25601d_driver_remove,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0))
	.resume		= bq25601d_resume,
	.suspend		= bq25601d_suspend,
#endif

	.shutdown	= bq25601d_reset,
	.id_table		= bq25601d_id,
};

module_i2c_driver(bq25601d_i2c_driver);
MODULE_DESCRIPTION("Driver for bq25601d charger chip");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:bq25601d-charger");
