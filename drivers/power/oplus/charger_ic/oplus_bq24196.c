/************************************************************************************
** File:  \\192.168.144.3\Linux_Share\12015\ics2\development\mediatek\custom\oplus77_12015\kernel\battery\battery
** OPLUS_FEATURE_CHG_BASIC
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
************************************************************************************************************/

#include <linux/interrupt.h>
#include <linux/i2c.h>

//#ifdef CONFIG_OPLUS_CHARGER_MTK
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


#endif

#include "../oplus_vooc.h"
#include "../oplus_gauge.h"
#include <oplus_bq24196.h>

static struct chip_bq24196 *charger_ic = NULL;
static int aicl_result = 500;

static DEFINE_MUTEX(bq24196_i2c_access);

static int __bq24196_read_reg(int reg, int *returnData)
{
    int ret = 0;
	struct chip_bq24196 *chip = charger_ic;
	
	ret = i2c_smbus_read_byte_data(chip->client, reg);
	if (ret < 0) {
		chg_err("i2c read fail: can't read from %02x: %d\n", reg, ret);
		return ret;
	} else {
		*returnData = ret;
	}

	return 0;
}

static int bq24196_read_reg(int reg, int *returnData)
{
	int ret = 0;
	
	mutex_lock(&bq24196_i2c_access);
	ret = __bq24196_read_reg(reg, returnData);
	mutex_unlock(&bq24196_i2c_access);
	return ret;
}

static int __bq24196_write_reg(int reg, int val)
{

    int ret = 0;
	struct chip_bq24196 *chip = charger_ic;
	
	ret = i2c_smbus_write_byte_data(chip->client, reg, val);
	if (ret < 0) {
		chg_err("i2c write fail: can't write %02x to %02x: %d\n",
			val, reg, ret);
		return ret;
	}

	return 0;
}


static int bq24196_config_interface (int RegNum, int val, int MASK)
{
    int bq24196_reg = 0;
    int ret = 0;
	
	mutex_lock(&bq24196_i2c_access);
    ret = __bq24196_read_reg(RegNum, &bq24196_reg);

    //chg_err(" Reg[%x]=0x%x\n", RegNum, bq24196_reg);
    
    bq24196_reg &= ~MASK;
    bq24196_reg |= val;

    ret = __bq24196_write_reg(RegNum, bq24196_reg);

    //chg_err(" write Reg[%x]=0x%x\n", RegNum, bq24196_reg);

    __bq24196_read_reg(RegNum, &bq24196_reg);

    //chg_err(" Check Reg[%x]=0x%x\n", RegNum, bq24196_reg);
	mutex_unlock(&bq24196_i2c_access);
	
    return ret;
}


//write one register directly
#if 0
int bq24196_reg_config_interface (int RegNum, int val)
{   
    int ret = 0;

	mutex_lock(&bq24196_i2c_access);
    ret = __bq24196_write_reg(RegNum, val);
	mutex_unlock(&bq24196_i2c_access);
	
    return ret;
}
#endif

static int bq24196_usbin_input_current_limit[] = {
    100,    150,    500,    900,
    1200,   1500,   2000,   3000,
};
#if defined(CONFIG_OPLUS_CHARGER_MTK6763) || defined(CONFIG_OPLUS_CHARGER_MTK6771)
#define AICL_COUNT 10
int chg_vol_mini(int *chg_vol)
{
	int chg_vol_temp = 0;
	int i = 0;
	chg_vol_temp = chg_vol[0];
	for(i = 0; i < AICL_COUNT; i++) {
		if(chg_vol_temp > chg_vol[i]){
			chg_vol_temp = chg_vol[i];
		}
	}
	return chg_vol_temp;
}
#endif
#if defined(CONFIG_MTK_PMIC_CHIP_MT6353) || defined(CONFIG_OPLUS_CHARGER_MTK6763) || defined(CONFIG_OPLUS_CHARGER_MTK6771)
static int bq24196_input_current_limit_write(int value)
{
	int rc = 0;
	int i = 0;
	int j = 0;
	int chg_vol = 0;
	int aicl_point_temp = 0;
	int chg_vol_all[10] = {0};
	struct chip_bq24196 *chip = charger_ic;
	
	if (atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}

	chg_debug( "usb input max current limit=%d setting %02x\n", value, i);

	aicl_point_temp = chip->sw_aicl_point;

	if (value < 150) {
		j = 0;
		goto aicl_end;
	} else if (value < 500) {
		j = 1;
		goto aicl_end;
	}

	j = 2; /* 500 */
	rc = bq24196_config_interface(REG00_BQ24196_ADDRESS, j << REG00_BQ24196_INPUT_CURRENT_LIMIT_SHIFT, REG00_BQ24196_INPUT_CURRENT_LIMIT_MASK);
	msleep(90);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point_temp) {
		j = 2;
		goto aicl_pre_step;
	} else if (value < 900)
		goto aicl_end;

	j = 3; /* 900 */
	rc = bq24196_config_interface(REG00_BQ24196_ADDRESS, j << REG00_BQ24196_INPUT_CURRENT_LIMIT_SHIFT, REG00_BQ24196_INPUT_CURRENT_LIMIT_MASK);
	msleep(90);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point_temp) {
		j = j - 1;
		goto aicl_pre_step;
	} else if (value < 1200)
		goto aicl_end;

	j = 5; /* 1500 */
	#if defined(CONFIG_OPLUS_CHARGER_MTK6763)
	aicl_point_temp = chip->sw_aicl_point + 20;
	#else
	aicl_point_temp = chip->sw_aicl_point + 55;
	#endif
	rc = bq24196_config_interface(REG00_BQ24196_ADDRESS, j << REG00_BQ24196_INPUT_CURRENT_LIMIT_SHIFT, REG00_BQ24196_INPUT_CURRENT_LIMIT_MASK);
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
	#if defined(CONFIG_OPLUS_CHARGER_MTK6763)
	aicl_point_temp = chip->sw_aicl_point + 30;
	#else
	aicl_point_temp = chip->sw_aicl_point - 30;
	#endif
	rc = bq24196_config_interface(REG00_BQ24196_ADDRESS, j << REG00_BQ24196_INPUT_CURRENT_LIMIT_SHIFT, REG00_BQ24196_INPUT_CURRENT_LIMIT_MASK);
	msleep(90);
	#if defined(CONFIG_OPLUS_CHARGER_MTK6763)
	for(i = 0; i < AICL_COUNT; i++){
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
	rc = bq24196_config_interface(REG00_BQ24196_ADDRESS, j << REG00_BQ24196_INPUT_CURRENT_LIMIT_SHIFT, REG00_BQ24196_INPUT_CURRENT_LIMIT_MASK);
	msleep(90);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point_temp) {
		j = j - 1;
		goto aicl_pre_step;
	} else if (value >= 3000)
		goto aicl_end;

aicl_pre_step:
	if((j >= 2) && (j <= ARRAY_SIZE(bq24196_usbin_input_current_limit) - 1))
		aicl_result = bq24196_usbin_input_current_limit[j];		
	chg_debug( "usb input max current limit aicl chg_vol=%d j[%d]=%d sw_aicl_point:%d aicl_pre_step\n", chg_vol, j, bq24196_usbin_input_current_limit[j], aicl_point_temp);
	rc = bq24196_config_interface(REG00_BQ24196_ADDRESS, j << REG00_BQ24196_INPUT_CURRENT_LIMIT_SHIFT, REG00_BQ24196_INPUT_CURRENT_LIMIT_MASK);
	return rc;
aicl_end:
if((j >= 2) && (j <= ARRAY_SIZE(bq24196_usbin_input_current_limit) - 1))
		aicl_result = bq24196_usbin_input_current_limit[j];		
	chg_debug( "usb input max current limit aicl chg_vol=%d j[%d]=%d sw_aicl_point:%d aicl_end\n", chg_vol, j, bq24196_usbin_input_current_limit[j], aicl_point_temp);
	rc = bq24196_config_interface(REG00_BQ24196_ADDRESS, j << REG00_BQ24196_INPUT_CURRENT_LIMIT_SHIFT, REG00_BQ24196_INPUT_CURRENT_LIMIT_MASK);
	return rc;
}
#else
static int bq24196_input_current_limit_write(void, int value)
{
	int rc = 0;
	int i = 0;
	int j = 0;
	int chg_vol = 0;
	struct chip_bq24196 *chip = charger_ic;
	
	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}

	for (i = ARRAY_SIZE(bq24196_usbin_input_current_limit) - 1; i >= 0; i--) {
		if (bq24196_usbin_input_current_limit[i] <= value) {
			break;
		}
		else if (i == 0) {
		    break;
		}
	}
    chg_debug( "usb input max current limit=%d setting %02x\n", value, i);
    for(j = 2; j <= i; j++) {
        bq24196_config_interface(REG00_BQ24196_ADDRESS, j<<REG00_BQ24196_INPUT_CURRENT_LIMIT_SHIFT, REG00_BQ24196_INPUT_CURRENT_LIMIT_MASK);
#ifdef CONFIG_OPLUS_CHARGER_MTK
		msleep(90);
		chg_vol = battery_meter_get_charger_voltage();
#else
		msleep(90);
		chg_vol = qpnp_get_prop_charger_voltage_now();
#endif	
        //chg_err("usb input max current limit aicl chg_vol=%d j=%d\n", chg_vol, j);
        if(chg_vol < chip->sw_aicl_point) {
            if (j > 2) {
                j = j-1;
            }
			if((j >= 2) && (j <= ARRAY_SIZE(bq24196_usbin_input_current_limit) - 1))
				aicl_result = bq24196_usbin_input_current_limit[j];		
            chg_debug( "usb input max current limit aicl chg_vol=%d j[%d]=%d sw_aicl_point:%d\n", chg_vol, j, bq24196_usbin_input_current_limit[j], chip->sw_aicl_point);
            bq24196_config_interface(REG00_BQ24196_ADDRESS, j << REG00_BQ24196_INPUT_CURRENT_LIMIT_SHIFT, REG00_BQ24196_INPUT_CURRENT_LIMIT_MASK);
            return 0;
        }
    }
    j = i;

	if((j >= 2) && (j <= ARRAY_SIZE(bq24196_usbin_input_current_limit) - 1))
		aicl_result = bq24196_usbin_input_current_limit[j];		
    chg_debug( "usb input max current limit aicl chg_vol=%d j[%d]=%d\n", chg_vol, j, bq24196_usbin_input_current_limit[j]);
    rc = bq24196_config_interface(REG00_BQ24196_ADDRESS, j << REG00_BQ24196_INPUT_CURRENT_LIMIT_SHIFT, REG00_BQ24196_INPUT_CURRENT_LIMIT_MASK);
    return rc;
}
#endif /*CONFIG_MTK_PMIC_CHIP_MT6353*/


static int bq24196_charging_current_write_fast(int chg_cur)
{
	int ret = 0;
	int value = 0;
	struct chip_bq24196 *chip = charger_ic;
	
	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}
	chg_debug( " chg_cur = %d\r\n", chg_cur);
	if (chg_cur < BQ24196_MIN_FAST_CURRENT_MA_ALLOWED) {
		if (chg_cur > BQ24196_MIN_FAST_CURRENT_MA_20_PERCENT)
			chg_cur = BQ24196_MIN_FAST_CURRENT_MA_20_PERCENT;
	    chg_cur = chg_cur * 5;
	    value = (chg_cur - BQ24196_MIN_FAST_CURRENT_MA)/BQ24196_FAST_CURRENT_STEP_MA;
	    value <<= REG02_BQ24196_FAST_CHARGING_CURRENT_LIMIT_SHIFT;
	    value = value | REG02_BQ24196_FAST_CHARGING_CURRENT_FORCE20PCT_ENABLE;
	} else {
	    value = (chg_cur - BQ24196_MIN_FAST_CURRENT_MA)/BQ24196_FAST_CURRENT_STEP_MA;
	    value <<= REG02_BQ24196_FAST_CHARGING_CURRENT_LIMIT_SHIFT;
	}
	ret = bq24196_config_interface(REG02_BQ24196_ADDRESS, value, 
		REG02_BQ24196_FAST_CHARGING_CURRENT_LIMIT_MASK | REG02_BQ24196_FAST_CHARGING_CURRENT_FORCE20PCT_MASK);
	return ret;
}

static int bq24196_set_vindpm_vol(int vol)
{
	int rc = 0;
	int value = 0;
	struct chip_bq24196 *chip = charger_ic;
	
	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}
	value = (vol - REG00_BQ24196_VINDPM_OFFSET) / REG00_BQ24196_VINDPM_STEP_MV;
	value <<= REG00_BQ24196_VINDPM_SHIFT;
	
    rc = bq24196_config_interface(REG00_BQ24196_ADDRESS, value, REG00_BQ24196_VINDPM_MASK);
	return rc;
}

#ifdef CONFIG_OPLUS_SHORT_C_BATT_CHECK
/* This function is getting the dynamic aicl result/input limited in mA.
 * If charger was suspended, it must return 0(mA).
 * It meets the requirements in SDM660 platform.
 */
static int bq24196_chg_get_dyna_aicl_result(void)
{
	return aicl_result;
	
}
#endif /* CONFIG_OPLUS_SHORT_C_BATT_CHECK */

static void bq24196_set_aicl_point(int vbatt)
{
	struct chip_bq24196 *chip = charger_ic;
	
	if(chip->hw_aicl_point == 4440 && vbatt > 4140) {
		chip->hw_aicl_point = 4520;
		chip->sw_aicl_point = 4535;
		bq24196_set_vindpm_vol(chip->hw_aicl_point);
	} else if(chip->hw_aicl_point == 4520 && vbatt < 4000) {
		chip->hw_aicl_point = 4440;
		chip->sw_aicl_point = 4500;
		bq24196_set_vindpm_vol(chip->hw_aicl_point);
	}
}

static int bq24196_set_enable_volatile_writes(void)
{
    int rc = 0;
	struct chip_bq24196 *chip = charger_ic;
	
	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}
	//need do nothing
	
    return rc;
}

static int bq24196_set_complete_charge_timeout(int val)
{
    int rc = 0;
	struct chip_bq24196 *chip = charger_ic;
	
	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}
    if (val == OVERTIME_AC){
        val = REG05_BQ24196_CHARGING_SAFETY_TIME_ENABLE | REG05_BQ24196_FAST_CHARGING_TIMEOUT_8H;
    } else if (val == OVERTIME_USB){
        val = REG05_BQ24196_CHARGING_SAFETY_TIME_ENABLE | REG05_BQ24196_FAST_CHARGING_TIMEOUT_12H;
    } else {
        val = REG05_BQ24196_CHARGING_SAFETY_TIME_DISABLE | REG05_BQ24196_FAST_CHARGING_TIMEOUT_8H;
    }
    
    rc = bq24196_config_interface(REG05_BQ24196_ADDRESS, 
		val, REG05_BQ24196_CHARGING_SAFETY_TIME_MASK | REG05_BQ24196_FAST_CHARGING_TIMEOUT_MASK);
    if (rc < 0) {
        chg_err("Couldn't complete charge timeout rc = %d\n", rc);
    }
    
    return rc;
}

int bq24196_float_voltage_write(int vfloat_mv)
{
	int rc = 0;
    int value = 0;
	struct chip_bq24196 *chip = charger_ic;
	
	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}
	if(vfloat_mv < BQ24196_MIN_FLOAT_MV) {
		chg_err(" bad vfloat_mv:%d,return\n", vfloat_mv);
		return 0;
	}
	value = (vfloat_mv - BQ24196_MIN_FLOAT_MV)/BQ24196_VFLOAT_STEP_MV;
	value <<= REG04_BQ24196_CHARGING_VOL_LIMIT_SHIFT;
	chg_debug( "bq24196_set_float_voltage vfloat_mv = %d value=%d\n", vfloat_mv, value);

	rc = bq24196_config_interface(REG04_BQ24196_ADDRESS, value, REG04_BQ24196_CHARGING_VOL_LIMIT_MASK);
	return rc;
}

int bq24196_set_prechg_current(int ipre_mA)
{
    int value = 0;
	struct chip_bq24196 *chip = charger_ic;
	
	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}
	value = (ipre_mA - BQ24196_MIN_PRE_CURRENT_MA)/BQ24196_PRE_CURRENT_STEP_MA;
	value <<= REG03_BQ24196_PRE_CHARGING_CURRENT_LIMIT_SHIFT;

	return bq24196_config_interface(REG03_BQ24196_ADDRESS, 
		value, REG03_BQ24196_PRE_CHARGING_CURRENT_LIMIT_MASK);
}

static int bq24196_set_termchg_current(int term_curr)
{
	int value = 0;
	int rc = 0;
	struct chip_bq24196 *chip = charger_ic;
	
	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}
	if (term_curr < BQ24196_MIN_TERM_CURRENT_MA) {
		term_curr = BQ24196_MIN_TERM_CURRENT_MA;
	}
	value = (term_curr - BQ24196_MIN_TERM_CURRENT_MA)/BQ24196_TERM_CURRENT_STEP_MA;
	value <<= REG03_BQ24196_TERM_CHARGING_CURRENT_LIMIT_SHIFT;
	chg_debug( " value=%d\n", value);

	bq24196_config_interface(REG03_BQ24196_ADDRESS, 
		value, REG03_BQ24196_TERM_CHARGING_CURRENT_LIMIT_MASK);
	return rc;
}

int bq24196_set_rechg_voltage(int recharge_mv)
{
	int reg = 0;
	int rc = 0;
	struct chip_bq24196 *chip = charger_ic;
	
	if (atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}
   
   /* set recharge voltage*/
    if (recharge_mv >= 300) {
        reg = REG04_BQ24196_RECHARGING_THRESHOLD_VOL_300MV;
    } else {
        reg = REG04_BQ24196_RECHARGING_THRESHOLD_VOL_100MV;
    }
    rc = bq24196_config_interface(REG04_BQ24196_ADDRESS, reg, REG04_BQ24196_RECHARGING_THRESHOLD_VOL_MASK);
    if (rc) {
        chg_err("Couldn't set recharging threshold rc = %d\n", rc);     
    }
	 return rc;
}

int bq24196_set_wdt_timer(int reg)
{
    int rc = 0;
	struct chip_bq24196 *chip = charger_ic;
	
	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}
    rc = bq24196_config_interface(REG05_BQ24196_ADDRESS, reg, REG05_BQ24196_I2C_WATCHDOG_TIME_MASK);
    
    return rc;
}

static int bq24196_set_chging_term_disable(void)
{
	int rc = 0;
	struct chip_bq24196 *chip = charger_ic;
	
	if (atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}
	
	rc = bq24196_config_interface(REG05_BQ24196_ADDRESS, 
		REG05_BQ24196_TERMINATION_DISABLE, REG05_BQ24196_TERMINATION_MASK);

	return rc;
}

int bq24196_kick_wdt()
{
	int rc = 0;
	struct chip_bq24196 *chip = charger_ic;
	
	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}
    rc = bq24196_config_interface(REG01_BQ24196_ADDRESS,
		REG01_BQ24196_WDT_TIMER_RESET, REG01_BQ24196_WDT_TIMER_RESET_MASK);
    
    return rc;
}

#ifdef CONFIG_MTK_PMIC_CHIP_MT6353
static bool bq24196_check_charger_suspend_enable(void);
int bq24196_enable_charging(void)
{
	int rc = 0;
	struct chip_bq24196 *chip = charger_ic;
	
	#ifdef CONFIG_OPLUS_CHARGER_MTK
	if (get_boot_mode() == META_BOOT || get_boot_mode() == FACTORY_BOOT
		|| get_boot_mode() == ADVMETA_BOOT
		|| get_boot_mode() == ATE_FACTORY_BOOT) {
		return 0;
	}
	#endif /* CONFIG_OPLUS_CHARGER_MTK */

	if (bq24196_check_charger_suspend_enable()) {
		bq24196_unsuspend_charger();
	}
	
	if (atomic_read(&chip->charger_suspended) == 1) {
		chg_err("Couldn't bq24196_enable_charging because charger_suspended\n");
		return 0;
	}
	rc = bq24196_config_interface(REG01_BQ24196_ADDRESS, 
			REG01_BQ24196_CHARGING_ENABLE, REG01_BQ24196_CHARGING_MASK);
	if (rc < 0) {
		chg_err("Couldn'tbq24196_enable_charging rc = %d\n", rc);
	}
	return rc;
}

int bq24196_disable_charging()
{
	int rc = 0;
	struct chip_bq24196 *chip = charger_ic;
	
	/* Only for BATTERY_STATUS__HIGH_TEMP or BATTERY_STATUS__WARM_TEMP, */
	/* system power is from charger but NOT from battery to avoid temperature rise problem */
	if (atomic_read(&chip->charger_suspended) == 1) {
		chg_err("Couldn't bq24196_disable_charging because charger_suspended\n");
		return 0;
	}
	rc = bq24196_config_interface(REG01_BQ24196_ADDRESS, 
			REG01_BQ24196_CHARGING_DISABLE, REG01_BQ24196_CHARGING_MASK);
	if (rc < 0) {
		chg_err("Couldn't bq24196_disable_charging  rc = %d\n", rc);
	} else {
		chg_err("battery HIGH_TEMP or WARM_TEMP, bq24196_disable_charging\n");
	}
	return rc;
}
#else
int bq24196_enable_charging(void)
{
	int rc = 0;
	struct chip_bq24196 *chip = charger_ic;
	
	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}
	
    rc = bq24196_config_interface(REG01_BQ24196_ADDRESS, 
		REG01_BQ24196_CHARGING_ENABLE, REG01_BQ24196_CHARGING_MASK);
    if (rc < 0) {
		chg_err("Couldn'tbq24196_enable_charging rc = %d\n", rc);
	}
	
	return rc;
}

int bq24196_disable_charging()
{
	int rc = 0;
	struct chip_bq24196 *chip = charger_ic;
	
	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}
    rc = bq24196_config_interface(REG01_BQ24196_ADDRESS, 
		REG01_BQ24196_CHARGING_DISABLE, REG01_BQ24196_CHARGING_MASK);
    if (rc < 0) {
		chg_err("Couldn't bq24196_disable_charging  rc = %d\n", rc);
	}
	
	return rc;
}
#endif /*CONFIG_MTK_PMIC_CHIP_MT6353*/

#ifdef CONFIG_MTK_PMIC_CHIP_MT6353
static bool bq24196_check_charger_suspend_enable(void)
{
	int rc = 0;
	int reg_val = 0;
	bool charger_suspend_enable = false;
	struct chip_bq24196 *chip = charger_ic;
	
	if (atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}
	
	rc = bq24196_read_reg(REG00_BQ24196_ADDRESS, &reg_val);
	if (rc) {
		chg_err("Couldn't read REG00_BQ24196_ADDRESS rc = %d\n", rc);
		return 0;
	}
	
	charger_suspend_enable = ((reg_val & REG00_BQ24196_SUSPEND_MODE_MASK) == REG00_BQ24196_SUSPEND_MODE_ENABLE) ? true : false;
		
	return charger_suspend_enable;	
}
static int bq24196_check_charging_enable(void)
{
	bool rc = 0;

	rc = bq24196_check_charger_suspend_enable();
	if(rc)
		return 0;
	else
		return 1;
}
#else
static int bq24196_check_charging_enable(void)
{
	int rc = 0;
	int reg_val = 0;
	bool charging_enable = false;
	struct chip_bq24196 *chip = charger_ic;
	
	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}
	rc = bq24196_read_reg(REG01_BQ24196_ADDRESS, &reg_val);
    if (rc) {
        chg_err("Couldn't read REG01_BQ24196_ADDRESS rc = %d\n", rc);
        return 0;
    }
	
    charging_enable = ((reg_val & REG01_BQ24196_CHARGING_MASK) == REG01_BQ24196_CHARGING_ENABLE) ? 1 : 0;
	
	return charging_enable;	
}
#endif /*CONFIG_MTK_PMIC_CHIP_MT6353*/

int bq24196_registers_read_full(void)
{
	int rc = 0;
    int reg_full = 0;
	int reg_ovp = 0;
	struct chip_bq24196 *chip = charger_ic;
	
	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}
    rc = bq24196_read_reg(REG08_BQ24196_ADDRESS, &reg_full);
    if (rc) {
        chg_err("Couldn't read STAT_C rc = %d\n", rc);
        return 0;
    }
	
    reg_full = ((reg_full & REG08_BQ24196_CHARGING_STATUS_CHARGING_MASK) == REG08_BQ24196_CHARGING_STATUS_TERM_CHARGING) ? 1 : 0;

	rc = bq24196_read_reg(REG09_BQ24196_ADDRESS, &reg_ovp);
	if (rc) {
        chg_err("Couldn't read STAT_D rc = %d\n", rc);
        return 0;
    }

	reg_ovp = ((reg_ovp & REG09_BQ24196_BATTERY_VOLATGE_MASK) == REG09_BQ24196_BATTERY_VOLATGE_HIGH_ERROR) ? 1 : 0;
	//chg_err("bq24196_registers_read_full, reg_full = %d, reg_ovp = %d\r\n", reg_full, reg_ovp);
	return (reg_full || reg_ovp);
}

#if defined(CONFIG_OPLUS_CHARGER_MTK6763) || defined(CONFIG_OPLUS_CHARGER_MTK6771)
int oplus_otg_online = 0;
#endif

int bq24196_otg_enable(void)
{
	int rc = 0;
	struct chip_bq24196 *chip = charger_ic;
	
	if (!chip) {
		return 0;
	}
#ifndef CONFIG_OPLUS_CHARGER_MTK
	if (atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}
#endif	
	//bq24196_unsuspend_charger();
	bq24196_config_interface(REG00_BQ24196_ADDRESS, 
		REG00_BQ24196_SUSPEND_MODE_DISABLE, REG00_BQ24196_SUSPEND_MODE_MASK);
	
	rc = bq24196_config_interface(REG01_BQ24196_ADDRESS, 
		REG01_BQ24196_OTG_ENABLE, REG01_BQ24196_OTG_MASK);
	if (rc) {
		chg_err("Couldn't enable  OTG mode rc=%d\n", rc);
	} else {
		chg_debug( "bq24196_otg_enable rc=%d\n", rc);
	}
#if defined(CONFIG_OPLUS_CHARGER_MTK6763) || defined(CONFIG_OPLUS_CHARGER_MTK6771)
	oplus_otg_online = 1;
#endif
	oplus_chg_set_otg_online(true);
	return rc;
}

int bq24196_otg_disable(void)
{
	int rc = 0;
	struct chip_bq24196 *chip = charger_ic;
	
	if (!chip) {
		return 0;
	}
	
#ifndef CONFIG_OPLUS_CHARGER_MTK	
	if (atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}
#endif
	rc = bq24196_config_interface(REG01_BQ24196_ADDRESS, 
		REG01_BQ24196_OTG_DISABLE, REG01_BQ24196_OTG_MASK);
	if (rc) 
		chg_err("Couldn't disable OTG mode rc=%d\n", rc);
	else
		chg_debug( "bq24196_otg_disable rc=%d\n", rc);
#if defined(CONFIG_OPLUS_CHARGER_MTK6763) || defined(CONFIG_OPLUS_CHARGER_MTK6771)
	oplus_otg_online = 0;
#endif
	oplus_chg_set_otg_online(false);
	return rc;
}

int bq24196_suspend_charger(void)
{
	int rc = 0;
	struct chip_bq24196 *chip = charger_ic;
	
	if (atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}
	
	if (oplus_chg_get_otg_online() == true)
		return 0;
	
#if defined(CONFIG_OPLUS_CHARGER_MTK6763) || defined(CONFIG_OPLUS_CHARGER_MTK6771)
	if (oplus_otg_online == 1)
		return 0;
#endif

	rc = bq24196_config_interface(REG00_BQ24196_ADDRESS, 
		REG00_BQ24196_SUSPEND_MODE_ENABLE, REG00_BQ24196_SUSPEND_MODE_MASK);
	chg_debug( " rc = %d\n", rc);
	if (rc < 0) {
		chg_err("Couldn't bq24196_suspend_charger rc = %d\n", rc);
	}
	
	return rc;
}
int bq24196_unsuspend_charger(void)
{
	int rc = 0;
	struct chip_bq24196 *chip = charger_ic;
	
	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}

    rc = bq24196_config_interface(REG00_BQ24196_ADDRESS, 
		REG00_BQ24196_SUSPEND_MODE_DISABLE, REG00_BQ24196_SUSPEND_MODE_MASK);
	chg_debug( " rc = %d\n", rc);
    if (rc < 0) {
		chg_err("Couldn't bq24196_unsuspend_charger rc = %d\n", rc);
	}	
	return rc;
}

#ifdef CONFIG_MTK_PMIC_CHIP_MT6353
int bq24196_reset_charger()
{
	int rc = 0;
	struct chip_bq24196 *chip = charger_ic;
	
	if (atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}

	rc = bq24196_config_interface(REG00_BQ24196_ADDRESS, 0x32, 0xFF);
	if (rc < 0) {
		chg_err("Couldn't reset REG00 rc = %d\n", rc);
	}
	rc = bq24196_config_interface(REG01_BQ24196_ADDRESS, 0x1B, 0xFF);
	if (rc < 0) {
		chg_err("Couldn't reset REG01 rc = %d\n", rc);
	}
	rc = bq24196_config_interface(REG02_BQ24196_ADDRESS, 0x60, 0xFF);
	if (rc < 0) {
		chg_err("Couldn't reset REG02 rc = %d\n", rc);
	}
	rc = bq24196_config_interface(REG03_BQ24196_ADDRESS, 0x11, 0xFF);
	if (rc < 0) {
		chg_err("Couldn't reset REG03 rc = %d\n", rc);
	}
	rc = bq24196_config_interface(REG04_BQ24196_ADDRESS, 0xCA, 0xFF);
	if (rc < 0) {
		chg_err("Couldn't reset REG04 rc = %d\n", rc);
	}
	rc = bq24196_config_interface(REG05_BQ24196_ADDRESS, 0x1A, 0xFF);
	if (rc < 0) {
		chg_err("Couldn't reset REG05 rc = %d\n", rc);
	}
	rc = bq24196_config_interface(REG06_BQ24196_ADDRESS, 0x03, 0xFF);
	if (rc < 0) {
		chg_err("Couldn't reset REG06 rc = %d\n", rc);
	}
	rc = bq24196_config_interface(REG07_BQ24196_ADDRESS, 0x4B, 0xFF);
	if (rc < 0) {
		chg_err("Couldn't reset REG07 rc = %d\n", rc);
	}

	return rc;
}
#else
int bq24196_reset_charger()
{
	int rc = 0;
	struct chip_bq24196 *chip = charger_ic;
	
	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}
    rc = bq24196_config_interface(REG01_BQ24196_ADDRESS, 
		REG01_BQ24196_REGISTER_RESET, REG01_BQ24196_REGISTER_RESET_MASK);
	chg_debug( " rc = %d\n", rc);
    if (rc < 0) {
		chg_err("Couldn't bq24196_reset_charger rc = %d\n", rc);
	}
	
	return rc;
}
#endif /*CONFIG_MTK_PMIC_CHIP_MT6353*/

static bool bq24196_check_charger_resume(void)
{
	struct chip_bq24196 *chip = charger_ic;
	
	if(atomic_read(&chip->charger_suspended) == 1) {
		return false;
	} else {
		return true;
	}
}

#define DUMP_REG_LOG_CNT_30S             6
#if defined(CONFIG_OPLUS_CHARGER_MTK6763) || defined(CONFIG_OPLUS_CHARGER_MTK6771)
void bq24196_dump_registers(void)
{
	int rc = 0;
	int addr = 0;
	unsigned int val_buf[BQ24196_REG_NUMBER] = {0x0};
	static int dump_count = 0;
	struct chip_bq24196 *chip = charger_ic;	
	
	if(atomic_read(&chip->charger_suspended) == 1) {
		return;
	}
	
	for (addr = BQ24196_FIRST_REG; addr <= BQ24196_LAST_REG; addr++) {
		rc = bq24196_read_reg(addr, &val_buf[addr]);
		if (rc) {
			 chg_err("Couldn't read 0x%02x rc = %d\n", addr, rc);
			 return;
		}
	}	

	if(dump_count == DUMP_REG_LOG_CNT_30S) {			
		dump_count = 0;
		chg_debug( "bq24196_reg[0-a]:0x%02x(0),0x%02x(1),0x%02x(2),0x%02x(3),0x%02x(4),0x%02x(5),0x%02x(6),0x%02x(7),0x%02x(8),0x%02x(9),0x%02x(a)\n",
			val_buf[0],val_buf[1],val_buf[2],val_buf[3],val_buf[4],val_buf[5],val_buf[6],val_buf[7],
			val_buf[8],val_buf[9],val_buf[10]);
	}
	dump_count++;
	return;

}
#else
void bq24196_dump_registers()
{
	int rc = 0;
	int addr = 0;
	unsigned int val_buf[BQ24196_REG_NUMBER] = {0x0};
	static int dump_count = 0;
	struct chip_bq24196 *chip = charger_ic;	
		
	if(atomic_read(&chip->charger_suspended) == 1) {
		return ;
	}
	
	for (addr = BQ24196_FIRST_REG; addr <= BQ24196_LAST_REG; addr++) {
		rc = bq24196_read_reg(addr, &val_buf[addr]);
		if (rc) {
			 chg_err("Couldn't read 0x%02x rc = %d\n", addr, rc);
		}
	}	

	if(dump_count == DUMP_REG_LOG_CNT_30S) {			
		dump_count = 0;
		chg_debug( "bq24196_reg[0-a]:0x%02x(0),0x%02x(1),0x%02x(2),0x%02x(3),0x%02x(4),0x%02x(5),0x%02x(6),0x%02x(7),0x%02x(8),0x%02x(9),0x%02x(a)\n",
			val_buf[0],val_buf[1],val_buf[2],val_buf[3],val_buf[4],val_buf[5],val_buf[6],val_buf[7],
			val_buf[8],val_buf[9],val_buf[10]);
	}
	dump_count++;

}
#endif
static int bq24196_get_chg_current_step(void)
{
	int rc = 0;
	int reg_val = 0;
	struct chip_bq24196 *chip = charger_ic;
		
	rc = bq24196_read_reg(REG02_BQ24196_ADDRESS, &reg_val);
	if (rc) {
		chg_err("Couldn't read REG02_BQ24196_ADDRESS rc = %d\n", rc);
		return 0;
	}

	if(reg_val & REG02_BQ24196_FAST_CHARGING_CURRENT_FORCE20PCT_MASK)
		return 13;
	else 
		return 64;
}


int bq24196_hardware_init(void)
{
	//must be before set_vindpm_vol and set_input_current
	struct chip_bq24196 *chip = charger_ic;
		
	chip->hw_aicl_point = 4440;
	chip->sw_aicl_point = 4500;
	
	bq24196_reset_charger();

	if(get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT) {
#ifndef CONFIG_MTK_PMIC_CHIP_MT6353
		bq24196_disable_charging();
#endif
		bq24196_float_voltage_write(4400);
		msleep(100);
	}
#if defined(CONFIG_OPLUS_CHARGER_MTK6763) || defined(CONFIG_OPLUS_CHARGER_MTK6771)
	bq24196_float_voltage_write(4370);
#else
	bq24196_float_voltage_write(4320);
#endif

	bq24196_set_enable_volatile_writes();
	
	bq24196_set_complete_charge_timeout(OVERTIME_DISABLED);

    bq24196_set_prechg_current(300);

	bq24196_charging_current_write_fast(512);
	
    bq24196_set_termchg_current(150);
    
    bq24196_set_rechg_voltage(100);

	bq24196_set_vindpm_vol(charger_ic->hw_aicl_point);

	#ifdef CONFIG_OPLUS_CHARGER_MTK
	if (get_boot_mode() == META_BOOT || get_boot_mode() == FACTORY_BOOT
		|| get_boot_mode() == ADVMETA_BOOT || get_boot_mode() == ATE_FACTORY_BOOT) {
		bq24196_suspend_charger();
	} else {
		bq24196_unsuspend_charger();
	}
#else
	bq24196_unsuspend_charger();
#endif

    bq24196_enable_charging();

    bq24196_set_wdt_timer(REG05_BQ24196_I2C_WATCHDOG_TIME_40S);

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

struct oplus_chg_operations  bq24196_chg_ops = {
	.dump_registers = bq24196_dump_registers,
	.kick_wdt = bq24196_kick_wdt,
	.hardware_init = bq24196_hardware_init,
	.charging_current_write_fast = bq24196_charging_current_write_fast,
	.set_aicl_point = bq24196_set_aicl_point,
	.input_current_write = bq24196_input_current_limit_write,
	.float_voltage_write = bq24196_float_voltage_write,
	.term_current_set = bq24196_set_termchg_current,
	.charging_enable = bq24196_enable_charging,
	.charging_disable = bq24196_disable_charging,
	.get_charging_enable = bq24196_check_charging_enable,
	.charger_suspend = bq24196_suspend_charger,
	.charger_unsuspend = bq24196_unsuspend_charger,
	.set_rechg_vol = bq24196_set_rechg_voltage,
	.reset_charger = bq24196_reset_charger,
	.read_full = bq24196_registers_read_full,
	.otg_enable = bq24196_otg_enable,
	.otg_disable = bq24196_otg_disable,
	.set_charging_term_disable = bq24196_set_chging_term_disable,
	.check_charger_resume = bq24196_check_charger_resume,
#ifdef 		CONFIG_OPLUS_CHARGER_MTK
	.get_charger_type = mt_power_supply_type_check,
	.get_charger_volt = battery_meter_get_charger_voltage,
	.check_chrdet_status = (bool (*) (void)) pmic_chrdet_status,
#if defined(CONFIG_OPLUS_CHARGER_MTK6763) || defined(CONFIG_OPLUS_CHARGER_MTK6771)
	.get_instant_vbatt = oplus_battery_meter_get_battery_voltage,
#else
	.get_instant_vbatt = battery_meter_get_battery_voltage,
#endif
	.get_boot_mode = (int (*)(void))get_boot_mode,
	.get_boot_reason = (int (*)(void))get_boot_reason,
//	#ifdef CONFIG_MTK_HAFG_20
	.get_chargerid_volt = mt_get_chargerid_volt,

	.set_chargerid_switch_val = mt_set_chargerid_switch_val ,
	.get_chargerid_switch_val  = mt_get_chargerid_switch_val,
//	#endif /* CONFIG_MTK_HAFG_20 */
	#ifdef CONFIG_MTK_HAFG_20
	.get_rtc_soc = get_rtc_spare_oplus_fg_value,
	.set_rtc_soc = set_rtc_spare_oplus_fg_value,
	#elif defined(CONFIG_OPLUS_CHARGER_MTK6763)
    .get_rtc_soc = get_rtc_spare_oplus_fg_value,
    .set_rtc_soc = set_rtc_spare_oplus_fg_value,
	#else /* CONFIG_MTK_HAFG_20 */
	.get_rtc_soc = get_rtc_spare_fg_value,
	.set_rtc_soc = set_rtc_spare_fg_value,
	#endif /* CONFIG_MTK_HAFG_20 */
	.set_power_off = mt_power_off,
	.usb_connect = mt_usb_connect,
	.usb_disconnect = mt_usb_disconnect,
	#else /* CONFIG_OPLUS_CHARGER_MTK */
	.get_charger_type = opchg_get_charger_type,
	.get_charger_volt = qpnp_get_prop_charger_voltage_now,
	.check_chrdet_status = oplus_chg_is_usb_present,
	.get_instant_vbatt = qpnp_get_battery_voltage,
	.get_boot_mode = get_boot_mode,
	.get_boot_reason = smbchg_get_boot_reason,
	.get_rtc_soc = oplus_chg_get_shutdown_soc,
	.set_rtc_soc = oplus_chg_backup_soc,
	#endif /* CONFIG_OPLUS_CHARGER_MTK */
	.get_chg_current_step = bq24196_get_chg_current_step,
#ifdef CONFIG_OPLUS_SHORT_C_BATT_CHECK
	.get_dyna_aicl_result = bq24196_chg_get_dyna_aicl_result,
#endif
#ifdef CONFIG_OPLUS_RTC_DET_SUPPORT
	.check_rtc_reset = rtc_reset_check,
#endif
};
/*
static struct oplus_chg_operations * oplus_get_chg_ops(void)
{
    return bq24196_chg_ops;

}
*/
static void register_charger_devinfo(void)
{
	int ret = 0;
	char *version;
	char *manufacture;
	
	version = "bq24196";
	manufacture = "TI";

	ret = register_device_proc("charger", version, manufacture);
	if (ret)
		chg_err("register_charger_devinfo fail\n");
}
/*
bool oplus_charger_ic_chip_is_null(void)
{
        if (!charger_ic) {
                return true;
        } else {
                return false;
        }
}
*/

static int bq24196_driver_probe(struct i2c_client *client, const struct i2c_device_id *id) 
{
	int reg = 0;
//	struct oplus_chg_chip *chip = NULL;
    struct chip_bq24196 *chg_ic;

	chg_ic = devm_kzalloc(&client->dev,
		sizeof(struct chip_bq24196), GFP_KERNEL);

    if (!chg_ic) {
            dev_err(&client->dev, "failed to allocate device info data\n");
            return -ENOMEM;
    }

#ifndef CONFIG_OPLUS_CHARGER_MTK	
	struct power_supply *usb_psy;
	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
        chg_err("USB psy not found; deferring probe\n");
        return -EPROBE_DEFER;
    }
#endif
    chg_debug( " call \n");
/*
	chip = devm_kzalloc(&client->dev,
		sizeof(struct oplus_chg_chip), GFP_KERNEL);
	if (!chip) {
		chg_err(" kzalloc() failed\n");
		return -ENOMEM;
	}
*/	
	chg_ic->client = client;
    chg_ic->dev = &client->dev;
	charger_ic = chg_ic;
    
    
	//chip->chg_ops = &bq24196_chg_ops;
	/*
	if(oplus_gauge_check_chip_is_null() || (chip->vooc_project && oplus_vooc_check_chip_is_null()) 
		|| oplus_pmic_check_chip_is_null())
	{
		chg_err("[oplus_chg_init] vooc || gauge || pmic not ready, will do after bettery init.\n");
		return -EPROBE_DEFER;
	}
	*/

	//oplus_chg_parse_dt(chip);
	atomic_set(&chg_ic->charger_suspended, 0);
	bq24196_dump_registers();
	
	bq24196_hardware_init();


  	//oplus_chg_init(chip);
	register_charger_devinfo();


    return 0;                                                                                       

}


static struct i2c_driver bq24196_i2c_driver;

static int bq24196_driver_remove(struct i2c_client *client)
{

	int ret=0;
    
	//ret = i2c_del_driver(&bq24196_i2c_driver);
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
static int bq24196_resume(struct device *dev)
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

	if (sleep_time < 1) {
		sleep_time = 0;
	}
	chg_err(" resume_sec:%ld,sleep_time:%ld\n\n",resume_tm_sec,sleep_time);
	oplus_chg_soc_update_when_resume(sleep_time);
	return 0;
}

static int bq24196_suspend(struct device *dev)
{
	if (!charger_ic) {
		return 0;
	}
	atomic_set(&charger_ic->charger_suspended, 1);
	if (get_current_time(&suspend_tm_sec)) {
		chg_err("RTC read failed\n");
		suspend_tm_sec = -1;
	}
	chg_err(" suspend_sec:%ld\n",suspend_tm_sec);
	return 0;
}
static const struct dev_pm_ops bq24196_pm_ops = {
	.resume		= bq24196_resume,
	.suspend		= bq24196_suspend,
};


#else //(LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
static int bq24196_resume(struct i2c_client *client)
{
	unsigned long resume_tm_sec = 0;
	unsigned long sleep_time = 0;
	int rc = 0;

	if(!charger_ic) {
		return 0;
	}
	atomic_set(&the_chip->charger_suspended, 0);
	rc = get_current_time(&resume_tm_sec);
	if (rc || suspend_tm_sec == -1) {
		chg_err("RTC read failed\n");
		sleep_time = 0;
	} else {
		sleep_time = resume_tm_sec - suspend_tm_sec;
	}

	if(sleep_time < 1) {
		sleep_time = 0;
	}
	oplus_chg_soc_update_when_resume(sleep_time);
	return 0;
}

static int bq24196_suspend(struct i2c_client *client, pm_message_t mesg)
{
	if(!charger_ic) {
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

static void bq24196_reset(struct i2c_client *client)
{
	bq24196_otg_disable();
}


/**********************************************************
  *
  *   [platform_driver API] 
  *
  *********************************************************/

static const struct of_device_id bq24196_match[] = {
	{ .compatible = "oplus,bq24196-charger"},
	{ },
};

static const struct i2c_device_id bq24196_id[] = {
	{"bq24196-charger", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, bq24196_id);


static struct i2c_driver bq24196_i2c_driver = {
	.driver		= {
		.name = "bq24196-charger",
		.owner	= THIS_MODULE,
		.of_match_table = bq24196_match,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
		.pm		= &bq24196_pm_ops,
#endif /*CONFIG_OPLUS_CHARGER_MTK6763*/
	},
	.probe		= bq24196_driver_probe,
	.remove		= bq24196_driver_remove,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0))
	.resume		= bq24196_resume,
	.suspend	= bq24196_suspend,
#endif
	.shutdown	= bq24196_reset,
	.id_table	= bq24196_id,
};


module_i2c_driver(bq24196_i2c_driver);
MODULE_DESCRIPTION("Driver for bq24196 charger chip");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:bq24196-charger");
