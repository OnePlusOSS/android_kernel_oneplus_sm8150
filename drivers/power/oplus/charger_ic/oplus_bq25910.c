/************************************************************************************
** File:  \\192.168.144.3\Linux_Share\12015\ics2\development\mediatek\custom\oplus77_12015\kernel\battery\battery
** VENDOR_EDIT
** Copyright (C), 2008-2012, OPLUS Mobile Comm Corp., Ltd
** 
** Description: 
**      for dc-dc sn111008 charg
** 
** Version: 1.0
** Date created: 21:03:46,05/04/2012
** Author: Fanhong.Kong@ProDrv.CHG
** 
** --------------------------- Revision History: ------------------------------------------------------------
* <version>       <date>        <author>              			<desc>
* Revision 1.0    2015-06-22    Fanhong.Kong@ProDrv.CHG   		Created for new architecture
* Revision 2.0    2018-04-14    Fanhong.Kong@ProDrv.CHG     	Upgrade for SVOOC
************************************************************************************************************/

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
#include <linux/module.h>

#include <soc/oplus/device_info.h>

extern void mt_power_off(void); 
#else
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


#endif

#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include "../oplus_vooc.h"
#include "../oplus_gauge.h"
#include <oplus_bq25910.h>

#define DEBUG_BY_FILE_OPS

static int __bq25910_read_reg(int reg, int *returnData);
static int __bq25910_write_reg(int reg, int val);

static struct chip_bq25910 *charger_ic = NULL;
static int reg_access_allow = 0;
int bq25910_reg = 0;
static int aicl_result = 500;
static int bq25910_online = true;

static DEFINE_MUTEX(bq25910_i2c_access);

static ssize_t bq25910_reg_access_show(struct device *dev,
						struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%02x\n", reg_access_allow);
}

static ssize_t bq25910_reg_access_store(struct device *dev,
						struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	pr_err("%s: value=%d\n", __FUNCTION__, val);
	reg_access_allow = val;

	return count;
}

static int bq25910_read_reg(int reg, int *returnData);
static int bq25910_read_reg(int reg, int *returnData);

static ssize_t bq25910_reg_set_show(struct device *dev,
						struct device_attribute *attr, char *buf)
{
	size_t count = 0;
	int reg_val = 0;

	bq25910_read_reg(bq25910_reg, &reg_val);
	count += snprintf(buf+count, PAGE_SIZE-count, "reg[0x%02x]: 0x%02x\n", bq25910_reg, reg_val);

	return count;
}

static ssize_t bq25910_reg_set_store(struct device *dev,
						struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int databuf[2] = {0, 0};

	if(2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
		pr_err("%s:reg[0x%02x]=0x%02x\n", __FUNCTION__, databuf[0], databuf[1]);
		bq25910_reg = databuf[0];
		__bq25910_write_reg((int)databuf[0], (int)databuf[1]);
    }

    return count;
}

static ssize_t bq25910_regs_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
	ssize_t len = 0;
	int i = 0;
	int reg_val = 0;

	for (i = BQ25910_FIRST_REG; i <= BQ25910_LAST_REG; i++) {
		bq25910_read_reg(i, &reg_val);
		len += snprintf(buf+len, PAGE_SIZE-len, "reg:0x%02x=0x%02x \n", i, reg_val);
	}

	return len;
}

static ssize_t bq25910_regs_store(struct device *dev,
					struct device_attribute *attr, const char *buf, size_t count)
{
    return count;
}

static DEVICE_ATTR(reg_access, S_IWUSR | S_IRUGO, bq25910_reg_access_show, bq25910_reg_access_store);
static DEVICE_ATTR(reg_set, S_IWUSR | S_IRUGO, bq25910_reg_set_show, bq25910_reg_set_store);
static DEVICE_ATTR(read_regs, S_IWUSR | S_IRUGO, bq25910_regs_show, bq25910_regs_store);

static struct attribute *bq25910_attributes[] = {
	&dev_attr_reg_access.attr,
	&dev_attr_reg_set.attr,
	&dev_attr_read_regs.attr,
	NULL
};

static struct attribute_group bq25910_attribute_group = {
	.attrs = bq25910_attributes
};

static int __bq25910_read_reg(int reg, int *returnData)
{
	int ret = 0;
	struct chip_bq25910 *chip = charger_ic;

	ret = i2c_smbus_read_byte_data(chip->client, (unsigned char)reg);
	if (ret < 0) {
		chg_err("i2c read fail: can't read from %02x: %d\n", reg, ret);
		return ret;
	} else {
		*returnData = ret;
	}

	return 0;
}

static int bq25910_read_reg(int reg, int *returnData)
{
	int ret = 0;

	mutex_lock(&bq25910_i2c_access);
	ret = __bq25910_read_reg(reg, returnData);
	mutex_unlock(&bq25910_i2c_access);
		
	return ret;
}

static int __bq25910_write_reg(int reg, int val)
{
	int ret = 0;
	struct chip_bq25910 *chip = charger_ic;

	if (reg_access_allow != 0) {
		chg_err("!!! can not access registers\n");
		return 0;
	}

	ret = i2c_smbus_write_byte_data(chip->client, reg, val);
	if (ret < 0) {
		chg_err("i2c write fail: can't write %02x to %02x: %d\n",
			val, reg, ret);
		return ret;
	}

	return 0;
}

/**********************************************************
  *
  *   [Read / Write Function] 
  *
  *********************************************************/

static int bq25910_config_interface(int RegNum, int val, int MASK)
{
	int bq25910_reg = 0;
	int ret = 0;

	mutex_lock(&bq25910_i2c_access);

	ret = __bq25910_read_reg(RegNum, &bq25910_reg);
	chg_err(" Reg[%x]=0x%x\n", RegNum, bq25910_reg);

	bq25910_reg &= ~MASK;
	bq25910_reg |= val;

	ret = __bq25910_write_reg(RegNum, bq25910_reg);
	chg_err(" write Reg[%x]=0x%x\n", RegNum, bq25910_reg);

	__bq25910_read_reg(RegNum, &bq25910_reg);
	chg_err(" Check Reg[%x]=0x%x\n", RegNum, bq25910_reg);

	mutex_unlock(&bq25910_i2c_access);

	return ret;
}


int bq25910_set_vindpm_vol(int vol)
{
	int rc;
	int tmp = 0;

	struct chip_bq25910 *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}

	tmp = (vol - REG02_BQ25910_VINDPM_THRESHOLD_OFFSET) / REG02_BQ25910_VINDPM_THRESHOLD_STEP;
	rc = bq25910_config_interface(REG02_BQ25910_ADDRESS, tmp << REG02_BQ25910_VINDPM_THRESHOLD_SHIFT, REG02_BQ25910_VINDPM_THRESHOLD_MASK);

	return rc;
}

int bq25910_usbin_input_current_limit[] = {
	500,    900,    1200,   1500,   1700,   2000,
};

int bq25910_input_Ilimit_disable(void)
{
	int rc = 0;
	struct chip_bq25910 *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1)
		return 0;

	return rc;
}

int bq25910_get_pre_icl_index(void)
{
	int reg_val = 0;
	int icl_index = 0;
	int rc = 0;
	struct chip_bq25910 *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1)
		return 0;

	rc = bq25910_read_reg(REG03_BQ25910_ADDRESS, &reg_val);
	if (rc) {
		chg_err("Couldn't read REG03_bq25910_ADDRESS rc = %d\n", rc);
		return icl_index;
	}

	reg_val &= REG03_BQ25910_CURRENT_LIMIT_MASK;

	switch (reg_val) {
		case REG03_BQ25910_CURRENT_LIMIT_500MA:
			icl_index = 0;
			break;
		case REG03_BQ25910_CURRENT_LIMIT_900MA:
			icl_index = 1;
			break;
		case REG03_BQ25910_CURRENT_LIMIT_1200MA:
			icl_index = 2;
			break;
		case REG03_BQ25910_CURRENT_LIMIT_1500MA:
			icl_index = 3;
			break;
		case REG03_BQ25910_CURRENT_LIMIT_1700MA:
			icl_index = 4;
			break;
		case REG03_BQ25910_CURRENT_LIMIT_2000MA:
			icl_index = 5;
			break;
		default:
			icl_index = 0;
			break;
	}
	chg_debug( "pre icl=%d setting %02x\n", bq25910_usbin_input_current_limit[icl_index], icl_index);

	return icl_index;
}



int bq25910_get_charger_vol(void)
{
	int chg_vol = 0;

#ifdef CONFIG_OPLUS_CHARGER_MTK
//	chg_vol = battery_meter_get_charger_voltage();
#else
//	chg_vol = qpnp_get_prop_charger_voltage_now();
#endif

	return chg_vol;
}

int bq25910_input_current_limit_write(int current_ma) 
{
	int rc = 0;
	struct chip_bq25910 *chip = charger_ic;

	if (bq25910_is_detected() == false){
		return 0;
	}

	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}

	chg_err("bq25910_input_current_limit_write: %d\n", current_ma);

	rc = bq25910_config_interface(REG03_BQ25910_ADDRESS, (current_ma - REG03_BQ25910_CURRENT_LIMIT_OFFSET) /100, REG03_BQ25910_CURRENT_LIMIT_MASK);
	if (rc < 0) {
                chg_err("Couldn't config current limit, rc = %d\n", rc);
        }

	return rc;
}

int bq25910_charging_current_write_fast(int chg_cur)
{	
	int rc = 0;
	int tmp = 0;
	struct chip_bq25910 *chip = charger_ic;

	if (bq25910_is_detected() == false){
		return 0;
	}

	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}

	chg_err("set charge current = %d\n", chg_cur);

	tmp = chg_cur - REG01_BQ25910_CHARGE_CURRENT_SETTING_OFFSET;
	tmp = tmp / REG01_BQ25910_CHARGE_CURRENT_SETTING_STEP;

	rc = bq25910_config_interface(REG01_BQ25910_ADDRESS, tmp << REG01_BQ25910_CHARGE_CURRENT_SETTING_SHIFT, REG01_BQ25910_CHARGE_CURRENT_SETTING_MASK);

	return rc;
}

#if 1
/* This function is getting the dynamic aicl result/input limited in mA.
 * If charger was suspended, it must return 0(mA).
 * It meets the requirements in SDM660 platform.
 */
int bq25910_chg_get_dyna_aicl_result(void)
{
	return aicl_result;
}
#endif /* CONFIG_OPLUS_SHORT_C_BATT_CHECK */


void bq25910_set_aicl_point(int vbatt)
{
	struct chip_bq25910 *chip = charger_ic;

	if(chip->hw_aicl_point == 4440 && vbatt > 4140) {
		chip->hw_aicl_point = 4520;
		chip->sw_aicl_point = 4535;
		bq25910_set_vindpm_vol(chip->hw_aicl_point);
	} else if(chip->hw_aicl_point == 4520 && vbatt < 4000) {
		chip->hw_aicl_point = 4440;
		chip->sw_aicl_point = 4500;
		bq25910_set_vindpm_vol(chip->hw_aicl_point);
	}
}

int bq25910_set_enable_volatile_writes(void)
{
	int rc = 0;
	struct chip_bq25910 *chip = charger_ic;

	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}
	//need do nothing

	return rc;
}

int bq25910_set_complete_charge_timeout(int val)
{
	int rc = 0;
	struct chip_bq25910 *chip = charger_ic;
	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}

	if (val == BQ_OVERTIME_AC) {
		val = REG05_BQ25910_CHARGE_TIMER_EN_ENABLE | REG05_BQ25910_FAST_CHARGE_TIMER_8H;
	} else if (val == BQ_OVERTIME_USB) {
		val = REG05_BQ25910_CHARGE_TIMER_EN_ENABLE | REG05_BQ25910_FAST_CHARGE_TIMER_12H;
	} else {
		val = REG05_BQ25910_CHARGE_TIMER_EN_DISABLE | REG05_BQ25910_FAST_CHARGE_TIMER_20H;
	}

	rc = bq25910_config_interface(REG05_BQ25910_ADDRESS, val, REG05_BQ25910_FAST_CHARGE_TIMER_MASK | REG05_BQ25910_CHARGE_TIMER_EN_MASK);
	if (rc < 0) {
		chg_err("Couldn't complete charge timeout rc = %d\n", rc);
	}

    return 0;
}

int bq25910_float_voltage_write(int vfloat_mv)
{
	int rc = 0;
	int tmp = 0;
	struct chip_bq25910 *chip = charger_ic;

	if (bq25910_is_detected() == false){
		return 0;
	}

	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}

	chg_err("vfloat_mv = %d\n", vfloat_mv); 

	if (vfloat_mv > 5000) {
		if (vfloat_mv > 9000) {
			vfloat_mv = 9000;
		}
		vfloat_mv = vfloat_mv / 2;
	}

	tmp = vfloat_mv - REG00_BQ25910_CHARGE_FULL_VOL_OFFSET;
	tmp = tmp / REG00_BQ25910_CHARGE_FULL_VOL_STEP;

	rc = bq25910_config_interface(REG00_BQ25910_ADDRESS, tmp << REG00_BQ25910_CHARGE_FULL_VOL_SHIFT, REG00_BQ25910_CHARGE_FULL_VOL_MASK);

	return rc;
}

int bq25910_set_prechg_current( int ipre_mA)
{
	int rc = 0;
	struct chip_bq25910 *chip = charger_ic;

	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}

	//do nothing
	return rc;
}

int bq25910_set_termchg_current(int term_curr)
{
	int rc = 0;
	struct chip_bq25910 *chip = charger_ic;	

	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}

 	//do_nothing
	return 0;
}

int bq25910_set_rechg_voltage(int recharge_mv)
{
	int rc = 0;
	struct chip_bq25910 *chip = charger_ic;

	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}

	//do nothing
	return rc;
}

int bq25910_set_wdt_timer(int reg)
{
	int rc = 0;
	struct chip_bq25910 *chip = charger_ic;

	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}

	rc = bq25910_config_interface(REG05_BQ25910_ADDRESS, reg, REG05_BQ25910_WTD_TIMER_MASK);
	if (rc) {
		chg_err("Couldn't set recharging threshold rc = %d\n", rc);     
	}

	return 0;
}

int bq25910_set_chging_term_disable(void)
{
	int rc = 0;
	struct chip_bq25910 *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}
	rc = bq25910_config_interface(REG05_BQ25910_ADDRESS, REG05_BQ25910_CHARGE_TERMINATION_EN_DISABLE, REG05_BQ25910_CHARGE_TERMINATION_EN_MASK);
	if (rc) {
		chg_err("Couldn't set chging term disable rc = %d\n", rc);     
	}

	return rc;
}

int bq25910_kick_wdt(void)
{
	int rc = 0;
	struct chip_bq25910 *chip = charger_ic;
	
	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}

	rc = bq25910_config_interface(REG05_BQ25910_ADDRESS, REG05_BQ25910_WTD_RST_RESET, REG05_BQ25910_WTD_RST_MASK);
	if (rc) {
		chg_err("Couldn't bq25910 kick wdt rc = %d\n", rc);     
	}

	return rc;
}


int bq25910_enable_charging(void)
{
	int rc;
	struct chip_bq25910 *chip = charger_ic;

	if (bq25910_is_detected() == false){
		return 0;
	}

	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}

	dump_stack();
	rc = bq25910_config_interface(REG06_BQ25910_ADDRESS, 
			REG06_BQ25910_CHG_EN_ENABLE, REG06_BQ25910_CHG_EN_MASK);
	if (rc < 0) {
		chg_err("Couldn'tbq25910_enable_charging rc = %d\n", rc);
	}

	chg_err("bq25910_enable_charging \n");

	return rc;
}

int bq25910_disable_charging(void)
{
	int rc;
	struct chip_bq25910 *chip = charger_ic;

	if (bq25910_is_detected() == false){
		return 0;
	}

	if(atomic_read(&chip->charger_suspended) == 1) {	
		chg_err(" charger_suspended \n");
		return 0;
	}

	rc = bq25910_config_interface(REG06_BQ25910_ADDRESS, 
			REG06_BQ25910_CHG_EN_DISABLE, REG06_BQ25910_CHG_EN_MASK);
	if (rc < 0) {
		chg_err("Couldn't bq25910_disable_charging  rc = %d\n", rc);
	}

	chg_err(" bq25910_disable_charging \n");

	return rc;
}

int bq25910_check_charging_enable(void)
{
	int rc = 0;
	int reg_val = 0;
	struct chip_bq25910 *chip = charger_ic;
	bool charging_enable = false;

	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}

	rc = bq25910_read_reg(REG06_BQ25910_ADDRESS, &reg_val);
	if (rc) {
		chg_err("Couldn't read REG08_bq25910_ADDRESS rc = %d\n", rc);
		return 0;
	}

	charging_enable = ((reg_val & REG06_BQ25910_CHG_EN_MASK) == REG06_BQ25910_CHG_EN_ENABLE) ? 1 : 0;

	return charging_enable;	
}


int bq25910_get_vbus_voltage(void)
{
    return 0;
}

int bq25910_get_ibus_current(void)
{
    return 0;
}

int bq25910_registers_read_full(void)
{
	int rc;
	int reg_full = 0;
	struct chip_bq25910 *chip = charger_ic;

	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}

	rc = bq25910_read_reg(REG09_BQ25910_ADDRESS, &reg_full);
	if (rc) {
		chg_err("Couldn't read STAT_C rc = %d\n", rc);
		return 0;
	}
	
	reg_full = ((reg_full & REG09_BQ25910_CHRG_TERM_FLAG_MASK) == REG09_BQ25910_CHARGING_STATUS_CHARGE_TERMINATION) ? 1 : 0;
	if (reg_full) {
		chg_err("the bq25910 is full");
		bq25910_dump_registers();
	}

	return 0;
}

int bq25910_suspend_charger(void)
{
	return 0;
}

int bq25910_unsuspend_charger(void)
{
	return 0;
}

bool bq25910_check_charger_resume(void)
{
	struct chip_bq25910 *chip = charger_ic;
	if(atomic_read(&chip->charger_suspended) == 1) {
		return false;
	} else {
		return true;
	}
}

int bq25910_reset_charger_for_wired_charge(void)
{
	int rc;
	struct chip_bq25910 *chip = charger_ic;

	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}

	chg_err("reset bq25910 for wired charge! \n");
    
	rc = bq25910_config_interface(REG0D_BQ25910_ADDRESS, 
			REG0D_BQ25910_REG_RST_RESET, REG0D_BQ25910_REG_RST_MASK);
	if (rc < 0) {
		chg_err("Couldn't bq25910_reset_charger  rc = %d\n", rc);
	}

	return rc;
}

int bq25910_otg_enable(void)
{
	return 0;
}

int bq25910_otg_disable(void)
{
	return 0;
}

int bq25910_reset_charger(void)
{
	int rc;
	struct chip_bq25910 *chip = charger_ic;
	
	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}

	rc = bq25910_config_interface(REG0D_BQ25910_ADDRESS, 
			REG0D_BQ25910_REG_RST_RESET, REG0D_BQ25910_REG_RST_MASK);
	if (rc < 0) {
		chg_err("Couldn't bq25910_reset_charger  rc = %d\n", rc);
	}

	return rc;
}

int bq25910_otg_ilim_set(int ilim)
{
    return 0;
}


int bq25910_set_switching_frequency(void)
{
	//default 750K
	return 0;
}

int bq25910_set_mps_otg_current(void)
{
	return 0;
}

int bq25910_set_mps_otg_voltage(void)
{
	return 0;
}

int bq25910_set_mps_otg_enable(void)
{
	return 0;
}

int bq25910_set_mps_otg_disable(void)
{
	return 0;
}


int bq25910_enable_hiz(void)
{
	return 0;
}

int bq25910_disable_hiz(void)
{
	return 0;
}
static int bq25910_vbus_avoid_electric_config(void)
{
	return 0;
}

int bq25910_check_learn_mode(void)
{
	return 0;	
}

int bq25910_other_registers_init(void)
{
	return 0;
}

static int bq25910_detect_device(void)
{
	int rc = 0;
	int reg_val = 0;
	struct chip_bq25910 *chip = charger_ic;

	rc = bq25910_read_reg(BQ25910_FIRST_REG, &reg_val);
	if (rc) {
		chg_err("Couldn't read BQ25910_FIRST_REG, rc = %d\n", rc);
		bq25910_online = false;
		return -1;
	}
	return rc;
}

bool bq25910_is_detected(void)
{
	return bq25910_online;
}

#define DUMP_REG_LOG_CNT_30S             6
void bq25910_dump_registers(void)
{
	int rc;
	int addr;
 	static int dump_count = 0;
	struct chip_bq25910 *chip = charger_ic;

	unsigned int val_buf[BQ25910_DUMP_MAX_REG+1] = {0x0};

	if (bq25910_is_detected() == false){
		return ;
	}

	if(atomic_read(&chip->charger_suspended) == 1) {
		return ;
	}

	if(dump_count == DUMP_REG_LOG_CNT_30S) {
		dump_count = 0;
		for (addr = BQ25910_FIRST_REG; addr <= BQ25910_DUMP_MAX_REG; addr++) {
 			rc = bq25910_read_reg(addr, &val_buf[addr]);
            		if (rc) {
                		chg_err("Couldn't read 0x%02x rc = %d\n", addr, rc);
            		}
        	}
        	printk(KERN_ERR "bq25910[0-0D]: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n", 
			val_buf[0], val_buf[1], val_buf[2], val_buf[3], val_buf[4], val_buf[5], val_buf[6], val_buf[7], val_buf[8], val_buf[9], val_buf[10], val_buf[11], val_buf[12], val_buf[13]);
    }
    dump_count++;
}

bool bq25910_need_to_check_ibatt(void)
{
    return false;
}


int bq25910_get_chg_current_step(void)
{
    int rc = 50;

    return rc;
}

int bq25910_input_current_limit_init(void)
{
	int rc = 0;
	struct chip_bq25910 *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		chg_err("bq25910_input_current_limit_init: in suspended\n");
		return 0;
	}

	rc = bq25910_config_interface(REG03_BQ25910_ADDRESS, REG03_BQ25910_CURRENT_LIMIT_500MA, REG03_BQ25910_CURRENT_LIMIT_MASK);
	if (rc < 0) {
		chg_err("Couldn't bq25910_input_current_limit_init rc = %d\n", rc);
	}

	return rc;
}

int bq25910_input_current_limit_without_aicl(int current_ma)
{
	int rc = 0;
	int reg_val = 0;
	struct chip_bq25910 *chip = charger_ic;

	if (atomic_read(&chip->charger_suspended) == 1) {
		chg_err("bq25910_input_current_limit_init: in suspended\n");
		return 0;
	}

	//chip->pre_current_ma = current_ma;
	reg_val = current_ma / REG03_BQ25910_CURRENT_LIMIT_STEP;
	chg_err(" reg_val current [%d]ma\n", reg_val);
	rc = bq25910_config_interface(REG03_BQ25910_ADDRESS, reg_val, REG03_BQ25910_CURRENT_LIMIT_MASK);
	if (rc < 0) {
		chg_err("Couldn't bq25910_input_current_limit_init rc = %d\n", rc);
	}

	return rc;
}


#ifdef CONFIG_OPLUS_CHARGER_MTK6853
static int bq25910_slave_chg_en_gpio_init(struct chip_bq25910 *chip)
{
	if (!chip) {
		chg_err("chip bq25910 not ready!\n");
		return -EINVAL;
	}

	chip->pinctrl = devm_pinctrl_get(chip->dev);

	if (IS_ERR_OR_NULL(chip->pinctrl)) {
		chg_err("get slave_chg_en_pinctrl fail\n");
		return -EINVAL;
	}

	chip->slave_charger_disable = pinctrl_lookup_state(chip->pinctrl, "slave_charger_disable");
	if (IS_ERR_OR_NULL(chip->slave_charger_disable)) {
		chg_err("get slave_charger_disable fail\n");
		return -EINVAL;
	}

	chip->slave_charger_enable = pinctrl_lookup_state(chip->pinctrl, "slave_charger_enable");
	if (IS_ERR_OR_NULL(chip->slave_charger_enable)) {
		chg_err("get slave_charger_enable fail\n");
		return -EINVAL;
	}

	pinctrl_select_state(chip->pinctrl, chip->slave_charger_enable);


	return 0;
}


static int bq25910_parse_gpio_dts(void)
{
	int ret = 0;
	struct chip_bq25910 *chip = charger_ic;

	if (!chip) {
		chg_err("chip is NULL\n");
		return -1;
	}

	chip->slave_chg_en_gpio = of_get_named_gpio(chip->client->dev.of_node, "qcom,slave-chg-en-gpio", 0);
	if (chip->slave_chg_en_gpio <= 0) {
		chg_err("Couldn't read slave_chg_en_gpio:%d\n", chip->slave_chg_en_gpio);
		return -1;
	} else {
		if (gpio_is_valid(chip->slave_chg_en_gpio)) {
			ret = gpio_request(chip->slave_chg_en_gpio, "slave-chg-en-gpio");
			if (ret) {
				chg_err("unable to request slave-chg-en-gpio[%d]\n", chip->slave_chg_en_gpio);
				chip->slave_chg_en_gpio = -EINVAL;
			} else {
				bq25910_slave_chg_en_gpio_init(chip);
				//gpio_direction_input(chip->irq_gpio);
			}
		}
        }
	return ret;
}
#endif


int bq25910_hardware_init(void)
{
	struct chip_bq25910 *chip = charger_ic;
	
	if (bq25910_is_detected() == false){
		return 0;
	}

	chg_err("init bq25910 hardware init for charge!\n");

	//must be before set_vindpm_vol and set_input_current
	chip->hw_aicl_point = 4440;
	chip->sw_aicl_point = 4500;

	bq25910_reset_charger();

	bq25910_set_chging_term_disable();

	bq25910_input_current_limit_init();

	bq25910_float_voltage_write(WPC_TERMINATION_VOLTAGE);

	bq25910_set_enable_volatile_writes();
    
	bq25910_set_complete_charge_timeout(BQ_OVERTIME_DISABLED);

	bq25910_set_prechg_current(WPC_PRECHARGE_CURRENT);

	bq25910_charging_current_write_fast(WPC_CHARGE_CURRENT_DEFAULT);
    
 	bq25910_set_termchg_current(WPC_TERMINATION_CURRENT);

	bq25910_set_rechg_voltage(WPC_RECHARGE_VOLTAGE_OFFSET);

	bq25910_set_switching_frequency();
    
	bq25910_set_vindpm_vol(chip->hw_aicl_point);

	bq25910_set_mps_otg_current();

	bq25910_set_mps_otg_voltage();

	bq25910_set_mps_otg_enable();

	bq25910_unsuspend_charger();

	bq25910_disable_charging();

	bq25910_set_wdt_timer(REG05_BQ25910_WTD_TIMER_DISABLE);

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

    if ((tm.tm_year == 110) && (tm.tm_mon == 0) && (tm.tm_mday <= 1)) {
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
struct oplus_chg_operations  bq25910_chg_ops = {
	.dump_registers = bq25910_dump_registers,
	.kick_wdt = bq25910_kick_wdt,
	.hardware_init = bq25910_hardware_init,
	.charging_current_write_fast = bq25910_charging_current_write_fast,
	.set_aicl_point = bq25910_set_aicl_point,
	.input_current_write = bq25910_input_current_limit_write,
	.float_voltage_write = bq25910_float_voltage_write,
	.term_current_set = bq25910_set_termchg_current,
	.charging_enable = bq25910_enable_charging,
	.charging_disable = bq25910_disable_charging,
	.get_charging_enable = bq25910_check_charging_enable,
	.charger_suspend = bq25910_suspend_charger,
	.charger_unsuspend = bq25910_unsuspend_charger,
	.set_rechg_vol = bq25910_set_rechg_voltage,
	.reset_charger = bq25910_reset_charger,
	.read_full = bq25910_registers_read_full,
	.otg_enable = bq25910_otg_enable,
	.otg_disable = bq25910_otg_disable,
	.set_charging_term_disable = bq25910_set_chging_term_disable,
	.check_charger_resume = bq25910_check_charger_resume,
#ifdef 		CONFIG_OPLUS_CHARGER_MTK
#else /* CONFIG_OPLUS_CHARGER_MTK */
#endif /* CONFIG_OPLUS_CHARGER_MTK */
#ifdef CONFIG_OPLUS_SHORT_C_BATT_CHECK
	.get_dyna_aicl_result = bq25910_chg_get_dyna_aicl_result,
#endif
#ifdef CONFIG_OPLUS_RTC_DET_SUPPORT
	.check_rtc_reset = rtc_reset_check,
#endif
};

#ifndef CONFIG_OPLUS_CHARGER_MTK6853
struct oplus_chg_operations * oplus_get_chg_ops(void)
{
	chg_err(" oplus_get_chg_ops--bq25910_chg_ops\n");
	return &bq25910_chg_ops;
}
#endif

static void register_charger_devinfo(void)
{
	int ret = 0;
	char *version;
	char *manufacture;

	version = "bq25910";
	manufacture = "MP";
	/****************************************************/
	//lizhijie temp compile
	//ret = register_device_proc("charger", version, manufacture);
	if (ret)
		chg_err("register_charger_devinfo fail\n");
}

#ifndef CONFIG_OPLUS_CHARGER_MTK6853
bool oplus_charger_ic_chip_is_null(void)
{
	if (!charger_ic) {
		return true;
	} else {
		return false;
	}
}
#endif

static int bq25910_gpio_init(struct chip_bq25910 *chip)
{
	return 0;
}

static int bq25910_driver_probe(struct i2c_client *client, const struct i2c_device_id *id) 
{
	int ret = 0;
 	struct chip_bq25910 *chg_ic;

	chg_ic = devm_kzalloc(&client->dev,
				sizeof(struct chip_bq25910), GFP_KERNEL);
	if (!chg_ic) {
		chg_err(" kzalloc() failed\n");
		return -ENOMEM;
	}

	chg_debug( " call \n");
	chg_ic->client = client;
	chg_ic->dev = &client->dev;

	charger_ic = chg_ic;
	atomic_set(&chg_ic->charger_suspended, 0);
#ifdef CONFIG_OPLUS_CHARGER_MTK6853
	bq25910_parse_gpio_dts();
#endif

	ret = bq25910_detect_device();
	if (ret < 0) {
		chg_err("!!!bq25910 is not detected\n");
		return -ENODEV;
	}
	if (bq25910_is_detected() == true){
		chg_err("bq25910 is detected\n");
	}
	bq25910_dump_registers();
	bq25910_vbus_avoid_electric_config();
	bq25910_hardware_init();
	bq25910_gpio_init(chg_ic);

	register_charger_devinfo();
	ret = sysfs_create_group(&chg_ic->dev->kobj, &bq25910_attribute_group);
	if (ret < 0) {
		chg_debug(" sysfs_create_group error fail\n");
		///return ret;
	}

	chg_debug(" success\n");

	return ret;                                                                                       
}


static struct i2c_driver bq25910_i2c_driver;

static int bq25910_driver_remove(struct i2c_client *client)
{
	int ret=0;

	//ret = i2c_del_driver(&bq25910_i2c_driver);
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
static int bq25910_pm_resume(struct device *dev)
{
	unsigned long resume_tm_sec = 0;
	unsigned long sleep_time = 0;
	int rc = 0;
	struct chip_bq25910 *chip = charger_ic;

	if(!chip) {
		return 0;
	}
	atomic_set(&chip->charger_suspended, 0);
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

	oplus_chg_soc_update_when_resume(sleep_time);
	return 0;
}

static int bq25910_pm_suspend(struct device *dev)
{
	struct chip_bq25910 *chip = charger_ic;

	if(!chip) {
		return 0;
	}
	atomic_set(&chip->charger_suspended, 1);
	if (get_current_time(&suspend_tm_sec)) {
		chg_err("RTC read failed\n");
		suspend_tm_sec = -1;
	}

	return 0;
}

static const struct dev_pm_ops bq25910_pm_ops = {
	.resume		= bq25910_pm_resume,
	.suspend		= bq25910_pm_suspend,
};
#else
static int bq25910_resume(struct i2c_client *client)
{	
	unsigned long resume_tm_sec = 0;
	unsigned long sleep_time = 0;
	int rc = 0;
	struct chip_bq25910 *chip = charger_ic;

	if(!chip) {
		return 0;
	}
	atomic_set(&chip->charger_suspended, 0);
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
	oplus_chg_soc_update_when_resume(sleep_time);
	return 0;
}

static int bq25910_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct chip_bq25910 *chip = charger_ic;

	if(!chip) {
		return 0;
	}
	atomic_set(&chip->charger_suspended, 1);
	if (get_current_time(&suspend_tm_sec)) {
		chg_err("RTC read failed\n");
		suspend_tm_sec = -1;
	}
	return 0;
}
#endif

static void bq25910_reset(struct i2c_client *client)
{
	bq25910_otg_disable();
}

/**********************************************************
  *
  *   [platform_driver API] 
  *
  *********************************************************/

static const struct of_device_id bq25910_match[] = {
	{ .compatible = "oplus,bq25910-charger"},
	{ },
};

static const struct i2c_device_id bq25910_id[] = {
	{"bq25910-charger", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, bq25910_id);


static struct i2c_driver bq25910_i2c_driver = {
	.driver		= {
		.name = "bq25910-charger",
		.owner	= THIS_MODULE,
		.of_match_table = bq25910_match,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
		.pm 	= &bq25910_pm_ops,
#endif
	},
	.probe		= bq25910_driver_probe,
	.remove		= bq25910_driver_remove,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0))
	.resume		= bq25910_resume,
	.suspend	= bq25910_suspend,
#endif
	.shutdown	= bq25910_reset,
	.id_table	= bq25910_id,
};


module_i2c_driver(bq25910_i2c_driver);
MODULE_DESCRIPTION("Driver for bq25910 charger chip");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:bq25910-charger");
