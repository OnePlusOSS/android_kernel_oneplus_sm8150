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
//#include <upmu_common.h>
#include <mt-plat/mtk_gpio.h>
//#include <mtk_boot_common.h>
#include <mt-plat/mtk_rtc.h>
//#include <mt-plat/charger_type.h>
#else
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_boot.h>
#include <mach/mtk_rtc.h>
#endif

#if defined CONFIG_OPLUS_CHARGER_MTK6763 || defined(CONFIG_OPLUS_CHARGER_MTK6771)
#include <soc/oplus/device_info.h>
#else
#include <soc/oplus/device_info.h>
#endif


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
#include <oplus_bq25890h.h>
extern int oplus_get_rtc_ui_soc(void);
extern int oplus_set_rtc_ui_soc(int value);
static struct chip_bq25890h *charger_ic;
extern int charger_ic_flag;
static int aicl_result = 500;
static struct delayed_work charger_modefy_work;

static DEFINE_MUTEX(bq25890h_i2c_access);

static int __bq25890h_read_reg(struct chip_bq25890h *chip, int reg, int *returnData)
{
    #ifdef 	CONFIG_OPLUS_CHARGER_MTK
    #if defined CONFIG_OPLUS_CHARGER_MTK6763  || defined(CONFIG_OPLUS_CHARGER_MTK6771)
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

    chip->client->ext_flag = ((chip->client->ext_flag ) & I2C_MASK_FLAG ) | I2C_WR_FLAG | I2C_DIRECTION_FLAG;
    chip->client->timing = 300;
    cmd_buf[0] = reg;
    ret = i2c_master_send(chip->client, &cmd_buf[0], (1<<8 | 1));
    if (ret < 0) {
        chip->client->ext_flag=0;
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

static int bq25890h_read_reg(struct chip_bq25890h *chip, int reg, int *returnData)
{
	int ret = 0;

	mutex_lock(&bq25890h_i2c_access);
	ret = __bq25890h_read_reg(chip, reg, returnData);
	mutex_unlock(&bq25890h_i2c_access);
	return ret;
}

static int __bq25890h_write_reg(struct chip_bq25890h *chip, int reg, int val)
{
    #ifdef CONFIG_OPLUS_CHARGER_MTK
    #if defined CONFIG_OPLUS_CHARGER_MTK6763 || defined(CONFIG_OPLUS_CHARGER_MTK6771)
	int ret = 0;

    ret = i2c_smbus_write_byte_data(chip->client, reg, val);
    if (ret < 0) {
        chg_err("i2c write fail: can't write %02x to %02x: %d\n",
        val, reg, ret);
        return ret;
    }
	#else
    char write_data[2] = {0};
    int ret = 0;

    write_data[0] = reg;
    write_data[1] = val;

    chip->client->ext_flag = ((chip->client->ext_flag ) & I2C_MASK_FLAG ) | I2C_DIRECTION_FLAG;
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
        chg_err("i2c write fail: can't write %02x to %02x: %d\n",
        val, reg, ret);
        return ret;
    }
	#endif /* CONFIG_OPLUS_CHARGER_MTK */

	return 0;
}


static int bq25890h_config_interface (struct chip_bq25890h *chip, int RegNum, int val, int MASK)
{
    int bq25890h_reg = 0;
    int ret = 0;

	mutex_lock(&bq25890h_i2c_access);
    ret = __bq25890h_read_reg(chip, RegNum, &bq25890h_reg);

    //chg_err(" Reg[%x]=0x%x\n", RegNum, bq25890h_reg);

    bq25890h_reg &= ~MASK;
    bq25890h_reg |= val;

    ret = __bq25890h_write_reg(chip, RegNum, bq25890h_reg);

    //chg_err(" write Reg[%x]=0x%x\n", RegNum, bq25890h_reg);

    __bq25890h_read_reg(chip, RegNum, &bq25890h_reg);

    //chg_err(" Check Reg[%x]=0x%x\n", RegNum, bq25890h_reg);
	mutex_unlock(&bq25890h_i2c_access);

    return ret;
}

static int bq25890h_usbin_input_current_limit[] = {
    100,    150,    500,    900,
    1200,   1500,   2000,   3000,
};
#if defined(CONFIG_OPLUS_CHARGER_MTK6763)  || defined(CONFIG_OPLUS_CHARGER_MTK6771)
#define AICL_COUNT 10
static int chg_vol_mini(int *chg_vol)
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
#if defined(CONFIG_MTK_PMIC_CHIP_MT6353) || defined(CONFIG_OPLUS_CHARGER_MTK6763)  || defined(CONFIG_OPLUS_CHARGER_MTK6771)
static int bq25890h_input_current_limit_write(int value)
{
	int rc = 0;
	int i = 0;
	int j = 0;
	int chg_vol = 0;
	int aicl_point_temp = 0;
	int chg_vol_all[10] = {0};
	static int low_chg_vol_flag = 0;
/*
	if(bq25890h_registers_read_full()) {
		bq25890h_config_interface(charger_ic, REG00_BQ25890H_ADDRESS, REG00_BQ25890H_INPUT_CURRENT_LIMIT_400MA, REG00_BQ25890H_INPUT_CURRENT_LIMIT_MASK);
		bq25890h_dump_registers();
		printk("bq25890h_input_current_limit_write\n");
		return 0;
	}

	if(charger_ic->charging_state == CHARGING_STATUS_FULL && charger_ic->in_rechging == false){
		bq25890h_config_interface(charger_ic, REG00_BQ25890H_ADDRESS, REG00_BQ25890H_INPUT_CURRENT_LIMIT_400MA, REG00_BQ25890H_INPUT_CURRENT_LIMIT_MASK);
		bq25890h_dump_registers();
		printk("bq25890h_input_current_limit_write, CHARGING_STATUS_FULL\n");
		return 0;
	}
	*/
	if (atomic_read(&charger_ic->charger_suspended) == 1) {
		return 0;
	}

	chg_debug( "usb input max current limit=%d setting %02x\n", value, i);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol <4300 && low_chg_vol_flag == 0 && !oplus_vooc_get_fastchg_started()) {
		bq25890h_disable_charging();
		bq25890h_suspend_charger();
		low_chg_vol_flag = 1;
	}
	if (chg_vol >4400 && low_chg_vol_flag == 1) {
		bq25890h_enable_charging();
		bq25890h_unsuspend_charger();
		low_chg_vol_flag = 0;
	}
	aicl_point_temp = charger_ic->sw_aicl_point;
/*
	for (j = 2; j < ARRAY_SIZE(bq25890h_usbin_input_current_limit); j++) {
		if (j == 4) //We DO NOT use 1.2A here
			continue;
		else if (j == 5)
			aicl_point_temp = chip->sw_aicl_point - 30;
		else if (j == 6)
			aicl_point_temp = chip->sw_aicl_point - 50;
		bq25890h_config_interface(chip, REG00_BQ25890H_ADDRESS, j << REG00_BQ25890H_INPUT_CURRENT_LIMIT_SHIFT, REG00_BQ25890H_INPUT_CURRENT_LIMIT_MASK);
		msleep(90);
		chg_vol = battery_meter_get_charger_voltage();
		if (chg_vol < aicl_point_temp) {
			if (j > 2)
				j = j - 1;
		}
		if (bq25890h_usbin_input_current_limit[j] >= 3000 || value < bq25890h_usbin_input_current_limit[j + 1]) {
			goto aicl_end;
		}
	}
*/
	if (value < 150) {
		j = 0;
		goto aicl_end;
	} else if (value < 500) {
		j = 1;
		goto aicl_end;
	}

	j = 2; /* 500 */
	rc = bq25890h_config_interface(charger_ic, REG00_BQ25890H_ADDRESS, REG00_BQ25890H_INPUT_CURRENT_LIMIT_500MA, REG00_BQ25890H_INPUT_CURRENT_LIMIT_MASK);
	msleep(90);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point_temp) {
		j = 2;
		goto aicl_pre_step;
	} else if (value < 900)
		goto aicl_end;

	j = 3; /* 900 */
	rc = bq25890h_config_interface(charger_ic, REG00_BQ25890H_ADDRESS, REG00_BQ25890H_INPUT_CURRENT_LIMIT_900MA, REG00_BQ25890H_INPUT_CURRENT_LIMIT_MASK);
	msleep(90);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point_temp) {
		j = j - 1;
		goto aicl_pre_step;
	} else if (value < 1200)
		goto aicl_end;

	j = 5; /* 1500 */
	#if defined(CONFIG_OPLUS_CHARGER_MTK6763)  || defined(CONFIG_OPLUS_CHARGER_MTK6771)
	aicl_point_temp = charger_ic->sw_aicl_point;
	#else
	aicl_point_temp = charger_ic->sw_aicl_point;
	#endif
	rc = bq25890h_config_interface(charger_ic, REG00_BQ25890H_ADDRESS, REG00_BQ25890H_INPUT_CURRENT_LIMIT_1500MA, REG00_BQ25890H_INPUT_CURRENT_LIMIT_MASK);
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
	#if defined(CONFIG_OPLUS_CHARGER_MTK6763)  || defined(CONFIG_OPLUS_CHARGER_MTK6771)
	aicl_point_temp = charger_ic->sw_aicl_point;
	#else
	aicl_point_temp = charger_ic->sw_aicl_point;
	#endif
	rc = bq25890h_config_interface(charger_ic, REG00_BQ25890H_ADDRESS, REG00_BQ25890H_INPUT_CURRENT_LIMIT_2000MA, REG00_BQ25890H_INPUT_CURRENT_LIMIT_MASK);
	msleep(90);
	#if defined(CONFIG_OPLUS_CHARGER_MTK6763)  || defined(CONFIG_OPLUS_CHARGER_MTK6771)
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
	rc = bq25890h_config_interface(charger_ic, REG00_BQ25890H_ADDRESS, REG00_BQ25890H_INPUT_CURRENT_LIMIT_3000MA, REG00_BQ25890H_INPUT_CURRENT_LIMIT_MASK);
	msleep(90);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point_temp) {
		j = j - 1;
		goto aicl_pre_step;
	} else if (value >= 3000)
		goto aicl_end;

aicl_pre_step:
	if((j >= 2) && (j <= ARRAY_SIZE(bq25890h_usbin_input_current_limit) - 1))
		aicl_result = bq25890h_usbin_input_current_limit[j];		
	chg_debug( "usb input max current limit aicl chg_vol=%d j[%d]=%d sw_aicl_point:%d aicl_pre_step\n", chg_vol, j, bq25890h_usbin_input_current_limit[j], aicl_point_temp);
	switch (j) {
		case 0:
			rc = bq25890h_config_interface(charger_ic, REG00_BQ25890H_ADDRESS, REG00_BQ25890H_INPUT_CURRENT_LIMIT_100MA, REG00_BQ25890H_INPUT_CURRENT_LIMIT_MASK);
		break;
		case 1:
			rc = bq25890h_config_interface(charger_ic, REG00_BQ25890H_ADDRESS, REG00_BQ25890H_INPUT_CURRENT_LIMIT_150MA, REG00_BQ25890H_INPUT_CURRENT_LIMIT_MASK);
		break;
		case 2:
			rc = bq25890h_config_interface(charger_ic, REG00_BQ25890H_ADDRESS, REG00_BQ25890H_INPUT_CURRENT_LIMIT_500MA, REG00_BQ25890H_INPUT_CURRENT_LIMIT_MASK);
		break;
		case 3:
			rc = bq25890h_config_interface(charger_ic, REG00_BQ25890H_ADDRESS, REG00_BQ25890H_INPUT_CURRENT_LIMIT_900MA, REG00_BQ25890H_INPUT_CURRENT_LIMIT_MASK);
		break;
		case 4:
			rc = bq25890h_config_interface(charger_ic, REG00_BQ25890H_ADDRESS, REG00_BQ25890H_INPUT_CURRENT_LIMIT_1200MA, REG00_BQ25890H_INPUT_CURRENT_LIMIT_MASK);
		break;
		case 5:
			rc = bq25890h_config_interface(charger_ic, REG00_BQ25890H_ADDRESS, REG00_BQ25890H_INPUT_CURRENT_LIMIT_1500MA, REG00_BQ25890H_INPUT_CURRENT_LIMIT_MASK);
		break;
		case 6:
			rc = bq25890h_config_interface(charger_ic, REG00_BQ25890H_ADDRESS, REG00_BQ25890H_INPUT_CURRENT_LIMIT_2000MA, REG00_BQ25890H_INPUT_CURRENT_LIMIT_MASK);
		break;
		case 7:
			rc = bq25890h_config_interface(charger_ic, REG00_BQ25890H_ADDRESS, REG00_BQ25890H_INPUT_CURRENT_LIMIT_3000MA, REG00_BQ25890H_INPUT_CURRENT_LIMIT_MASK);
		break;
		default:
			break;
	}
	return rc;
aicl_end:
	if((j >= 2) && (j <= ARRAY_SIZE(bq25890h_usbin_input_current_limit) - 1))
		aicl_result = bq25890h_usbin_input_current_limit[j];		
	chg_debug( "usb input max current limit aicl chg_vol=%d j[%d]=%d sw_aicl_point:%d aicl_end\n", chg_vol, j, bq25890h_usbin_input_current_limit[j], aicl_point_temp);
	switch (j) {
		case 0:
			rc = bq25890h_config_interface(charger_ic, REG00_BQ25890H_ADDRESS, REG00_BQ25890H_INPUT_CURRENT_LIMIT_100MA, REG00_BQ25890H_INPUT_CURRENT_LIMIT_MASK);
		break;
		case 1:
			rc = bq25890h_config_interface(charger_ic, REG00_BQ25890H_ADDRESS, REG00_BQ25890H_INPUT_CURRENT_LIMIT_150MA, REG00_BQ25890H_INPUT_CURRENT_LIMIT_MASK);
		break;
		case 2:
			rc = bq25890h_config_interface(charger_ic, REG00_BQ25890H_ADDRESS, REG00_BQ25890H_INPUT_CURRENT_LIMIT_500MA, REG00_BQ25890H_INPUT_CURRENT_LIMIT_MASK);
		break;
		case 3:
			rc = bq25890h_config_interface(charger_ic, REG00_BQ25890H_ADDRESS, REG00_BQ25890H_INPUT_CURRENT_LIMIT_900MA, REG00_BQ25890H_INPUT_CURRENT_LIMIT_MASK);
		break;
		case 4:
			rc = bq25890h_config_interface(charger_ic, REG00_BQ25890H_ADDRESS, REG00_BQ25890H_INPUT_CURRENT_LIMIT_1200MA, REG00_BQ25890H_INPUT_CURRENT_LIMIT_MASK);
		break;
		case 5:
			rc = bq25890h_config_interface(charger_ic, REG00_BQ25890H_ADDRESS, REG00_BQ25890H_INPUT_CURRENT_LIMIT_1500MA, REG00_BQ25890H_INPUT_CURRENT_LIMIT_MASK);
		break;
		case 6:
			rc = bq25890h_config_interface(charger_ic, REG00_BQ25890H_ADDRESS, REG00_BQ25890H_INPUT_CURRENT_LIMIT_2000MA, REG00_BQ25890H_INPUT_CURRENT_LIMIT_MASK);
		break;
		case 7:
			rc = bq25890h_config_interface(charger_ic, REG00_BQ25890H_ADDRESS, REG00_BQ25890H_INPUT_CURRENT_LIMIT_3000MA, REG00_BQ25890H_INPUT_CURRENT_LIMIT_MASK);
		break;
		default:
			break;
	}
	bq25890h_dump_registers();
	return rc;
}
#else /*CONFIG_MTK_PMIC_CHIP_MT6353 || CONFIG_OPLUS_CHARGER_MTK6763*/
static int bq25890h_input_current_limit_write(int value)
{
	int rc = 0;
	int i = 0;
	int j = 0;
	int chg_vol = 0;

	if (atomic_read(&charger_ic->charger_suspended) == 1) {
		return 0;
	}

	for (i = ARRAY_SIZE(bq25890h_usbin_input_current_limit) - 1; i >= 0; i--) {
		if (bq25890h_usbin_input_current_limit[i] <= value) {
			break;
		}
		else if (i == 0) {
		    break;
		}
	}
    chg_debug( "usb input max current limit=%d setting %02x\n", value, i);
    for (j = 2; j <= i; j++) {
        bq25890h_config_interface(charger_ic, REG00_BQ25890H_ADDRESS, j<<REG00_BQ25890H_INPUT_CURRENT_LIMIT_SHIFT, REG00_BQ25890H_INPUT_CURRENT_LIMIT_MASK);
		#ifdef CONFIG_OPLUS_CHARGER_MTK
		msleep(90);
		chg_vol = battery_meter_get_charger_voltage();
		#else /* CONFIG_OPLUS_CHARGER_MTK */
		msleep(90);
		chg_vol = qpnp_get_prop_charger_voltage_now();
		#endif /* CONFIG_OPLUS_CHARGER_MTK */
        if(chg_vol < charger_ic->sw_aicl_point) {
            if (j > 2) {
                j = j-1;
            }
			if((j >= 2) && (j <= ARRAY_SIZE(bq25890h_usbin_input_current_limit) - 1))
				aicl_result = bq25890h_usbin_input_current_limit[j];		
            chg_debug( "usb input max current limit aicl chg_vol=%d j[%d]=%d sw_aicl_point:%d\n", chg_vol, j, bq25890h_usbin_input_current_limit[j], charger_ic->sw_aicl_point);
            bq25890h_config_interface(chip, REG00_BQ25890H_ADDRESS, j << REG00_BQ25890H_INPUT_CURRENT_LIMIT_SHIFT, REG00_BQ25890H_INPUT_CURRENT_LIMIT_MASK);
            return 0;
        }
    }
    j = i;
	
	if((j >= 2) && (j <= ARRAY_SIZE(bq25890h_usbin_input_current_limit) - 1))
		aicl_result = bq25890h_usbin_input_current_limit[j];		

    chg_debug( "usb input max current limit aicl chg_vol=%d j[%d]=%d\n", chg_vol, j, bq25890h_usbin_input_current_limit[j]);
    rc = bq25890h_config_interface(charger_ic, REG00_BQ25890H_ADDRESS, j << REG00_BQ25890H_INPUT_CURRENT_LIMIT_SHIFT, REG00_BQ25890H_INPUT_CURRENT_LIMIT_MASK);
    return rc;
}
#endif /*CONFIG_MTK_PMIC_CHIP_MT6353*/


static int bq25890h_charging_current_write_fast(int chg_cur)
{
	int ret = 0;
	int value = 0;

	if (atomic_read(&charger_ic->charger_suspended) == 1) {
		return 0;
	}
	chg_debug( " chg_cur = %d\r\n", chg_cur);

	value = (chg_cur - BQ25890H_MIN_FAST_CURRENT_MA)/BQ25890H_FAST_CURRENT_STEP_MA;
	value <<= REG04_BQ25890H_FAST_CHARGING_CURRENT_LIMIT_SHIFT;

	ret = bq25890h_config_interface(charger_ic, REG04_BQ25890H_ADDRESS, value,
		REG04_BQ25890H_FAST_CHARGING_CURRENT_LIMIT_MASK);
	return ret;
}

static int bq25890h_set_vindpm_vol(struct chip_bq25890h *chip, int vol)
{
	int rc = 0;
	int value = 0;

	if (atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}
	value = (vol - REG0D_BQ25890H_VINDPM_OFFSET) / REG0D_BQ25890H_VINDPM_STEP_MV;
	value <<= REG0D_BQ25890H_VINDPM_SHIFT;

    rc = bq25890h_config_interface(chip,
		REG0D_BQ25890H_ADDRESS, value, REG0D_BQ25890H_VINDPM_MASK);
	return rc;
}

#ifdef CONFIG_OPLUS_SHORT_C_BATT_CHECK
/* This function is getting the dynamic aicl result/input limited in mA.
 * If charger was suspended, it must return 0(mA).
 * It meets the requirements in SDM660 platform.
 */
static int oplus_chg_get_dyna_aicl_result(void)
{
	return aicl_result;
	
}
#endif /* CONFIG_OPLUS_SHORT_C_BATT_CHECK */

static void bq25890h_set_aicl_point(int vbatt)
{
	if (charger_ic->hw_aicl_point == 4440 && vbatt > 4100) {
		charger_ic->hw_aicl_point = 4500;
		charger_ic->sw_aicl_point = 4550;
		bq25890h_set_vindpm_vol(charger_ic, charger_ic->hw_aicl_point);
	} else if (charger_ic->hw_aicl_point == 4520 && vbatt < 4100) {
		charger_ic->hw_aicl_point = 4400;
		charger_ic->sw_aicl_point = 4500;
		bq25890h_set_vindpm_vol(charger_ic, charger_ic->hw_aicl_point);
	}
}

static int bq25890h_set_enable_volatile_writes(struct chip_bq25890h *chip)
{
    int rc = 0;

	if (atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}

    return rc;
}

static int bq25890h_set_complete_charge_timeout(struct chip_bq25890h * chip, int val)
{
    int rc = 0;

	if (atomic_read(&charger_ic->charger_suspended) == 1) {
		return 0;
	}
    if (val == OVERTIME_AC) {
        val = REG07_BQ25890H_CHARGING_SAFETY_TIME_ENABLE | REG07_BQ25890H_FAST_CHARGING_TIMEOUT_8H;
    } else if (val == OVERTIME_USB) {
        val = REG07_BQ25890H_CHARGING_SAFETY_TIME_ENABLE | REG07_BQ25890H_FAST_CHARGING_TIMEOUT_12H;
    } else {
        val = REG07_BQ25890H_CHARGING_SAFETY_TIME_DISABLE | REG07_BQ25890H_FAST_CHARGING_TIMEOUT_8H;
    }

    rc = bq25890h_config_interface(charger_ic, REG07_BQ25890H_ADDRESS,
		val, REG07_BQ25890H_CHARGING_SAFETY_TIME_MASK | REG07_BQ25890H_FAST_CHARGING_TIMEOUT_MASK);
    if (rc < 0) {
        chg_err("Couldn't complete charge timeout rc = %d\n", rc);
    }

    return rc;
}

int bq25890h_float_voltage_write(int vfloat_mv)
{
	int rc = 0;
    int value = 0;

	if (atomic_read(&charger_ic->charger_suspended) == 1) {
		return 0;
	}
	if (vfloat_mv < BQ25890H_MIN_FLOAT_MV) {
		chg_err(" bad vfloat_mv:%d,return\n", vfloat_mv);
		return 0;
	}
	value = (vfloat_mv - BQ25890H_MIN_FLOAT_MV)/BQ25890H_VFLOAT_STEP_MV;
	value <<= REG06_BQ25890H_CHARGING_VOL_LIMIT_SHIFT;
	chg_debug( "bq25890h_set_float_voltage vfloat_mv = %d value=%d\n", vfloat_mv, value);

	rc = bq25890h_config_interface(charger_ic, REG06_BQ25890H_ADDRESS, value, REG06_BQ25890H_CHARGING_VOL_LIMIT_MASK);
	return rc;
}

int bq25890h_set_prechg_current(int ipre_mA)
{
    int value = 0;

	if (atomic_read(&charger_ic->charger_suspended) == 1) {
		return 0;
	}
	value = (ipre_mA - BQ25890H_MIN_PRE_CURRENT_MA)/BQ25890H_PRE_CURRENT_STEP_MA;
	value <<= REG05_BQ25890H_PRE_CHARGING_CURRENT_LIMIT_SHIFT;

	return bq25890h_config_interface(charger_ic, REG05_BQ25890H_ADDRESS,
		value, REG05_BQ25890H_PRE_CHARGING_CURRENT_LIMIT_MASK);
}

static int bq25890h_set_termchg_current(int term_curr)
{
	int value = 0;
	int rc = 0;

	if (atomic_read(&charger_ic->charger_suspended) == 1) {
		return 0;
	}
	if (term_curr < BQ25890H_MIN_TERM_CURRENT_MA) {
		term_curr = BQ25890H_MIN_TERM_CURRENT_MA;
	}
	value = (term_curr - BQ25890H_MIN_TERM_CURRENT_MA)/BQ25890H_TERM_CURRENT_STEP_MA;
	value <<= REG05_BQ25890H_TERM_CHARGING_CURRENT_LIMIT_SHIFT;
	chg_debug( " value=%d\n", value);

	bq25890h_config_interface(charger_ic, REG05_BQ25890H_ADDRESS,
		value, REG05_BQ25890H_TERM_CHARGING_CURRENT_LIMIT_MASK);
	return rc;
}

int bq25890h_set_rechg_voltage(int recharge_mv)
{
	int reg = 0;
	int rc = 0;
	
	if (atomic_read(&charger_ic->charger_suspended) == 1) {
		return 0;
	}

	/* set recharge voltage */
    if (recharge_mv >= 300) {
        reg = REG06_BQ25890H_RECHARGING_THRESHOLD_VOL_200MV;
    } else {
        reg = REG06_BQ25890H_RECHARGING_THRESHOLD_VOL_100MV;
    }
    rc = bq25890h_config_interface(charger_ic, REG06_BQ25890H_ADDRESS, reg, REG06_BQ25890H_RECHARGING_THRESHOLD_VOL_MASK);
    if (rc) {
        chg_err("Couldn't set recharging threshold rc = %d\n", rc);
    }
	 return rc;
}

int bq25890h_set_wdt_timer(int reg)
{
    int rc = 0;

	if (atomic_read(&charger_ic->charger_suspended) == 1) {
		return 0;
	}
    rc = bq25890h_config_interface(charger_ic,
		REG07_BQ25890H_ADDRESS, reg, REG07_BQ25890H_I2C_WATCHDOG_TIME_MASK);

    return rc;
}

static int bq25890h_set_chging_term_disable(void)
{
	int rc = 0;
	if (atomic_read(&charger_ic->charger_suspended) == 1) {
		return 0;
	}

	rc = bq25890h_config_interface(charger_ic, REG07_BQ25890H_ADDRESS,
		REG07_BQ25890H_TERMINATION_DISABLE, REG07_BQ25890H_TERMINATION_MASK);

	return rc;
}

int bq25890h_kick_wdt(void)
{
	int rc = 0;

	if (atomic_read(&charger_ic->charger_suspended) == 1) {
		return 0;
	}
    rc = bq25890h_config_interface(charger_ic, REG03_BQ25890H_ADDRESS,
		REG03_BQ25890H_WDT_TIMER_RESET, REG03_BQ25890H_WDT_TIMER_RESET_MASK);

    return rc;
}

#ifdef CONFIG_MTK_PMIC_CHIP_MT6353
static bool bq25890h_check_charger_suspend_enable(void);
int bq25890h_enable_charging(void)
{
	int rc = 0;

	#ifdef CONFIG_OPLUS_CHARGER_MTK
	if (get_boot_mode() == META_BOOT || get_boot_mode() == FACTORY_BOOT
		|| get_boot_mode() == ADVMETA_BOOT
		|| get_boot_mode() == ATE_FACTORY_BOOT) {
		return 0;
	}
	#endif /* CONFIG_OPLUS_CHARGER_MTK */

	if (bq25890h_check_charger_suspend_enable()) {
		bq25890h_unsuspend_charger();
	}

	if (atomic_read(&charger_ic->charger_suspended) == 1) {
		chg_err("Couldn't bq25890h_enable_charging because charger_suspended\n");
		return 0;
	}
	rc = bq25890h_config_interface(charger_ic, REG03_BQ25890H_ADDRESS,
			REG03_BQ25890H_CHARGING_ENABLE, REG03_BQ25890H_CHARGING_MASK);
	if (rc < 0) {
		chg_err("Couldn'tbq25890h_enable_charging rc = %d\n", rc);
	}
	return rc;
}

int bq25890h_disable_charging(void)
{
	int rc = 0;

	/* Only for BATTERY_STATUS__HIGH_TEMP or BATTERY_STATUS__WARM_TEMP, */
	/* system power is from charger but NOT from battery to avoid temperature rise problem */
	if (atomic_read(&charger_ic->charger_suspended) == 1) {
		chg_err("Couldn't bq25890h_disable_charging because charger_suspended\n");
		return 0;
	}
	rc = bq25890h_config_interface(charger_ic, REG03_BQ25890H_ADDRESS,
			REG03_BQ25890H_CHARGING_DISABLE, REG03_BQ25890H_CHARGING_MASK);
	if (rc < 0) {
		chg_err("Couldn't bq25890h_disable_charging  rc = %d\n", rc);
	} else {
		chg_err("battery HIGH_TEMP or WARM_TEMP, bq25890h_disable_charging\n");
	}
	rc = bq25890h_config_interface(charger_ic, REG07_BQ25890H_ADDRESS,
			REG07_BQ25890H_CHARGING_SAFETY_TIME_DISABLE, REG07_BQ25890H_I2C_WATCHDOG_TIME_MASK);
	if (rc < 0) {
		chg_err("Couldn't bq25890h_disable_watchdog  rc = %d\n", rc);
	} else {
		chg_err("disable_watchdog\n");
	}
	return rc;
}
#else /* CONFIG_MTK_PMIC_CHIP_MT6353 */
int bq25890h_enable_charging(void)
{
	int rc = 0;

	if(atomic_read(&charger_ic->charger_suspended) == 1) {
		return 0;
	}

    rc = bq25890h_config_interface(charger_ic, REG03_BQ25890H_ADDRESS,
		REG03_BQ25890H_CHARGING_ENABLE, REG03_BQ25890H_CHARGING_MASK);
    if (rc < 0) {
		chg_err("Couldn'tbq25890h_enable_charging rc = %d\n", rc);
	}
	
	bq25890h_set_wdt_timer(REG07_BQ25890H_I2C_WATCHDOG_TIME_40S);

	return rc;
}

int bq25890h_disable_charging(void)
{
	int rc = 0;

	if (atomic_read(&charger_ic->charger_suspended) == 1) {
		return 0;
	}
    rc = bq25890h_config_interface(charger_ic, REG03_BQ25890H_ADDRESS,
		REG03_BQ25890H_CHARGING_DISABLE, REG03_BQ25890H_CHARGING_MASK);
    if (rc < 0) {
		chg_err("Couldn't bq25890h_disable_charging  rc = %d\n", rc);
	}
	//if(charger_ic->charging_state == CHARGING_STATUS_FULL)
	//{
	//	bq25890h_input_current_limit_write(400);
	//}
	bq25890h_config_interface(charger_ic, REG00_BQ25890H_ADDRESS, 
		REG00_BQ25890H_INPUT_CURRENT_LIMIT_400MA, REG00_BQ25890H_INPUT_CURRENT_LIMIT_MASK);
	
	bq25890h_set_wdt_timer(REG07_BQ25890H_I2C_WATCHDOG_TIME_DISABLE);
	bq25890h_set_vindpm_vol(charger_ic, 4500);

	return rc;
}
#endif /*CONFIG_MTK_PMIC_CHIP_MT6353*/

#ifdef CONFIG_MTK_PMIC_CHIP_MT6353
static bool bq25890h_check_charger_suspend_enable(void)
{
	int rc = 0;
	int reg_val = 0;
	bool charger_suspend_enable = false;

	if (atomic_read(&charger_ic->charger_suspended) == 1) {
		return 0;
	}

	rc = bq25890h_read_reg(charger_ic, REG00_BQ25890H_ADDRESS, &reg_val);
	if (rc) {
		chg_err("Couldn't read REG00_BQ25890H_ADDRESS rc = %d\n", rc);
		return 0;
	}

	charger_suspend_enable = ((reg_val & REG00_BQ25890H_SUSPEND_MODE_MASK) == REG00_BQ25890H_SUSPEND_MODE_ENABLE) ? true : false;

	return charger_suspend_enable;
}
static int bq25890h_check_charging_enable(void)
{
	bool rc = 0;

	rc = bq25890h_check_charger_suspend_enable();
	if(rc)
		return 0;
	else
		return 1;
}
#else /* CONFIG_MTK_PMIC_CHIP_MT6353 */
static int bq25890h_check_charging_enable(void)
{
	int rc = 0;
	int reg_val = 0;
	bool charging_enable = false;

	if(atomic_read(&charger_ic->charger_suspended) == 1) {
		return 0;
	}
	rc = bq25890h_read_reg(charger_ic, REG03_BQ25890H_ADDRESS, &reg_val);
    if (rc) {
        chg_err("Couldn't read REG01_BQ25890H_ADDRESS rc = %d\n", rc);
        return 0;
    }

    charging_enable = ((reg_val & REG03_BQ25890H_CHARGING_MASK) == REG03_BQ25890H_CHARGING_ENABLE) ? 1 : 0;

	return charging_enable;
}
#endif /*CONFIG_MTK_PMIC_CHIP_MT6353*/

int bq25890h_registers_read_full(void)
{
	int rc = 0;
    int reg_full = 0;
	int reg_ovp = 0;

	if (atomic_read(&charger_ic->charger_suspended) == 1) {
		return 0;
	}
    rc = bq25890h_read_reg(charger_ic, REG0B_BQ25890H_ADDRESS, &reg_full);
    if (rc) {
        chg_err("Couldn't read STAT_C rc = %d\n", rc);
        return 0;
    }

    reg_full = ((reg_full & REG0B_BQ25890H_CHARGING_STATUS_CHARGING_MASK) == REG0B_BQ25890H_CHARGING_STATUS_TERM_CHARGING) ? 1 : 0;

	rc = bq25890h_read_reg(charger_ic, REG0C_BQ25890H_ADDRESS, &reg_ovp);
	if (rc) {
        chg_err("Couldn't read STAT_D rc = %d\n", rc);
        return 0;
    }

	reg_ovp = ((reg_ovp & REG0C_BQ25890H_BATTERY_VOLATGE_MASK) == REG0C_BQ25890H_BATTERY_VOLATGE_HIGH_ERROR) ? 1 : 0;
//	chg_err("bq25890h_registers_read_full, reg_full = %d, reg_ovp = %d\r\n", reg_full, reg_ovp);
	return (reg_full || reg_ovp);
}

static int oplus_otg_online = 0;

int bq25890h_otg_enable(void)
{
	int rc = 0;

	if (!charger_ic) {
		return 0;
	}
	#ifndef CONFIG_OPLUS_CHARGER_MTK
	if (atomic_read(&charger_ic->charger_suspended) == 1) {
		return 0;
	}
	#endif /* CONFIG_OPLUS_CHARGER_MTK */
	bq25890h_config_interface(charger_ic, REG00_BQ25890H_ADDRESS,
		REG00_BQ25890H_SUSPEND_MODE_DISABLE, REG00_BQ25890H_SUSPEND_MODE_MASK);

	rc = bq25890h_config_interface(charger_ic, REG03_BQ25890H_ADDRESS,
		REG03_BQ25890H_OTG_ENABLE, REG03_BQ25890H_OTG_MASK);
	if (rc) {
		chg_err("Couldn't enable  OTG mode rc=%d\n", rc);
	} else {
		chg_debug( "bq25890h_otg_enable rc=%d\n", rc);
	}
	oplus_otg_online = 1;
	bq25890h_set_wdt_timer(REG07_BQ25890H_I2C_WATCHDOG_TIME_40S);
	oplus_chg_set_otg_online(true);
	return rc;
}

int bq25890h_otg_disable(void)
{
	int rc = 0;
	int val_buf;

	if (!charger_ic) {
		return 0;
	}
	#ifndef CONFIG_OPLUS_CHARGER_MTK
	if (atomic_read(&charger_ic->charger_suspended) == 1) {
		return 0;
	}
	#endif /* CONFIG_OPLUS_CHARGER_MTK */
	rc = bq25890h_config_interface(charger_ic, REG03_BQ25890H_ADDRESS,
		REG03_BQ25890H_OTG_DISABLE, REG03_BQ25890H_OTG_MASK);
	if (rc)
		chg_err("Couldn't disable OTG mode rc=%d\n", rc);
	else
		chg_debug( "bq25890h_otg_disable rc=%d\n", rc);

	bq25890h_set_wdt_timer(REG07_BQ25890H_I2C_WATCHDOG_TIME_DISABLE);
	oplus_otg_online = 0;
	oplus_chg_set_otg_online(false);
	bq25890h_read_reg(charger_ic, REG0C_BQ25890H_ADDRESS, &val_buf);
	bq25890h_read_reg(charger_ic, REG0C_BQ25890H_ADDRESS, &val_buf);
	return rc;
}

int bq25890h_suspend_charger(void)
{
	int rc = 0;

	if (atomic_read(&charger_ic->charger_suspended) == 1) {
		return 0;
	}

	if (oplus_chg_get_otg_online() == true)
		return 0;

	if (oplus_otg_online == 1)
		return 0;

	rc = bq25890h_config_interface(charger_ic, REG00_BQ25890H_ADDRESS,
		REG00_BQ25890H_SUSPEND_MODE_ENABLE, REG00_BQ25890H_SUSPEND_MODE_MASK);
	chg_debug( " rc = %d\n", rc);
	if (rc < 0) {
		chg_err("Couldn't bq25890h_suspend_charger rc = %d\n", rc);
	}

	return rc;
}
int bq25890h_unsuspend_charger(void)
{
	int rc = 0;

	if (atomic_read(&charger_ic->charger_suspended) == 1) {
		return 0;
	}

    rc = bq25890h_config_interface(charger_ic, REG00_BQ25890H_ADDRESS,
		REG00_BQ25890H_SUSPEND_MODE_DISABLE, REG00_BQ25890H_SUSPEND_MODE_MASK);
	chg_debug( " rc = %d\n", rc);
    if (rc < 0) {
		chg_err("Couldn't bq25890h_unsuspend_charger rc = %d\n", rc);
	}
	return rc;
}

#if defined CONFIG_MTK_PMIC_CHIP_MT6353 || defined(CONFIG_OPLUS_CHARGER_MTK6763) || defined(CONFIG_OPLUS_CHARGER_MTK6771)
int bq25890h_reset_charger(void)
{
	int rc = 0;

	if (atomic_read(&charger_ic->charger_suspended) == 1) {
		return 0;
	}

	rc = bq25890h_config_interface(charger_ic, REG00_BQ25890H_ADDRESS, 0x08, 0xFF);
	if (rc < 0) {
		chg_err("Couldn't reset REG00 rc = %d\n", rc);
	}
	rc = bq25890h_config_interface(charger_ic, REG01_BQ25890H_ADDRESS, 0x01, 0xFF);
	if (rc < 0) {
		chg_err("Couldn't reset REG01 rc = %d\n", rc);
	}
	rc = bq25890h_config_interface(charger_ic, REG02_BQ25890H_ADDRESS, 0x00, 0xFF);
	if (rc < 0) {
		chg_err("Couldn't reset REG02 rc = %d\n", rc);
	}
	rc = bq25890h_config_interface(charger_ic, REG03_BQ25890H_ADDRESS, 0x1A, 0xFF);
	if (rc < 0) {
		chg_err("Couldn't reset REG03 rc = %d\n", rc);
	}
	rc = bq25890h_config_interface(charger_ic, REG04_BQ25890H_ADDRESS, 0x08, 0xFF);
	if (rc < 0) {
		chg_err("Couldn't reset REG04 rc = %d\n", rc);
	}
	rc = bq25890h_config_interface(charger_ic, REG05_BQ25890H_ADDRESS, 0x13, 0xFF);
	if (rc < 0) {
		chg_err("Couldn't reset REG05 rc = %d\n", rc);
	}
	rc = bq25890h_config_interface(charger_ic, REG06_BQ25890H_ADDRESS, 0x86, 0xFF);
	if (rc < 0) {
		chg_err("Couldn't reset REG06 rc = %d\n", rc);
	}
	rc = bq25890h_config_interface(charger_ic, REG07_BQ25890H_ADDRESS, 0xDD, 0xFF);
	if (rc < 0) {
		chg_err("Couldn't reset REG07 rc = %d\n", rc);
	}
	rc = bq25890h_config_interface(charger_ic, REG08_BQ25890H_ADDRESS, 0x03, 0xFF);
	if (rc < 0) {
		chg_err("Couldn't reset REG08 rc = %d\n", rc);
	}
		rc = bq25890h_config_interface(charger_ic, REG09_BQ25890H_ADDRESS, 0x40, 0xFF);
	if (rc < 0) {
		chg_err("Couldn't reset REG09 rc = %d\n", rc);
	}
	rc = bq25890h_config_interface(charger_ic, REG0A_BQ25890H_ADDRESS, 0x73, 0xFF);
	if (rc < 0) {
		chg_err("Couldn't reset REG0A rc = %d\n", rc);
	}
	rc = bq25890h_config_interface(charger_ic, REG0D_BQ25890H_ADDRESS, 0x91, 0xFF);
	if (rc < 0) {
		chg_err("Couldn't reset REG0D rc = %d\n", rc);
	}
	rc = bq25890h_config_interface(charger_ic, REG14_BQ25890H_ADDRESS, 0x5F, 0xFF);
	if (rc < 0) {
		chg_err("Couldn't reset REG14 rc = %d\n", rc);
	}
	

	return rc;
}
#else /*defined CONFIG_MTK_PMIC_CHIP_MT6353 || defined(CONFIG_OPLUS_CHARGER_MTK6763) || defined(CONFIG_OPLUS_CHARGER_MTK6771)*/
int bq25890h_reset_charger(void)
{
	int rc = 0;

	if (atomic_read(&charger_ic->charger_suspended) == 1) {
		return 0;
	}
    rc = bq25890h_config_interface(charger_ic, REG01_BQ25890H_ADDRESS,
		REG14_BQ25890H_REGISTER_RESET, REG14_BQ25890H_REGISTER_RESET_MASK);
	chg_debug( " rc = %d\n", rc);
    if (rc < 0) {
		chg_err("Couldn't bq25890h_reset_charger rc = %d\n", rc);
	}

	return rc;
}
#endif /*defined CONFIG_MTK_PMIC_CHIP_MT6353 || defined(CONFIG_OPLUS_CHARGER_MTK6763) || defined(CONFIG_OPLUS_CHARGER_MTK6771)*/

static bool bq25890h_check_charger_resume(void)
{
	if (atomic_read(&charger_ic->charger_suspended) == 1) {
		return false;
	} else {
		return true;
	}
}

#define DUMP_REG_LOG_CNT_30S  6

void bq25890h_dump_registers(void)
{
	int rc = 0;
	int addr = 0;
	unsigned int val_buf[BQ25890H_REG_NUMBER] = {0x0};
	static int dump_count = 0;

	if (atomic_read(&charger_ic->charger_suspended) == 1) {
		return;
	}

	for (addr = BQ25890H_FIRST_REG; addr <= BQ25890H_LAST_REG; addr++) {
		rc = bq25890h_read_reg(charger_ic, addr, &val_buf[addr]);
		if (rc) {
			 chg_err("Couldn't read 0x%02x rc = %d\n", addr, rc);
			 return;
		}
	}

	if(dump_count == DUMP_REG_LOG_CNT_30S) {
		///if(1) {
		dump_count = 0;
		printk( "bq25890h_reg[0-a]:0x%02x(0),0x%02x(1),0x%02x(2),0x%02x(3),0x%02x(4),0x%02x(5),0x%02x(6),0x%02x(7),0x%02x(8),0x%02x(9),0x%02x(a)\n",
			val_buf[0],val_buf[1],val_buf[2],val_buf[3],val_buf[4],val_buf[5],val_buf[6],val_buf[7],
			val_buf[8],val_buf[9],val_buf[10]);
		printk( "bq25890h_reg[b-14]:0x%02x(b),0x%02x(c),0x%02x(d),0x%02x(e),0x%02x(f),0x%02x(10),0x%02x(11),0x%02x(12),0x%02x(13),0x%02x(14)\n",
			val_buf[11],val_buf[12],val_buf[13],val_buf[14],val_buf[15],val_buf[16],val_buf[17],val_buf[18],
			val_buf[19],val_buf[20]);

	}
	dump_count++;
	return;

}

int bq25890h_check_registers(void)
{
	int rc = 0;
	int addr = 0;
	unsigned int val_buf[BQ25890H_REG_NUMBER] = {0x0};

	if (atomic_read(&charger_ic->charger_suspended) == 1) {
		return 0;
	}

	for (addr = BQ25890H_FIRST_REG; addr <= BQ25890H_LAST_REG; addr++) {
		rc = bq25890h_read_reg(charger_ic, addr, &val_buf[addr]);
		if (rc) {
			 chg_err("Couldn't read 0x%02x rc = %d\n", addr, rc);
			 return -1;
		}
	}

	return 0;

}


static int bq25890h_get_chg_current_step(void)
{
	int rc = 0;
	int reg_val = 0;

	rc = bq25890h_read_reg(charger_ic, REG02_BQ25890H_ADDRESS, &reg_val);
	if (rc) {
		chg_err("Couldn't read REG02_BQ25890H_ADDRESS rc = %d\n", rc);
		return 0;
	}

	//if(reg_val & REG02_BQ25890H_FAST_CHARGING_CURRENT_FORCE20PCT_MASK)
	//	return 13;
	//else
		return 64;
}


int bq25890h_hardware_init(void)
{
	/* must be before set_vindpm_vol and set_input_current */
	charger_ic->hw_aicl_point = 4440;
	charger_ic->sw_aicl_point = 4500;

	bq25890h_reset_charger();

	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT) {
		#ifndef CONFIG_MTK_PMIC_CHIP_MT6353
		bq25890h_disable_charging();
		#endif /* CONFIG_MTK_PMIC_CHIP_MT6353 */
		bq25890h_float_voltage_write(4400);
		msleep(100);
	}
	#if defined(CONFIG_OPLUS_CHARGER_MTK6763) || defined(CONFIG_OPLUS_CHARGER_MTK6771)
	bq25890h_float_voltage_write(4370);
	#else
	bq25890h_float_voltage_write(4320);
	#endif
	
	bq25890h_set_enable_volatile_writes(charger_ic);

	bq25890h_set_complete_charge_timeout(charger_ic, OVERTIME_DISABLED);

    bq25890h_set_prechg_current(300);

	bq25890h_charging_current_write_fast(512);

    bq25890h_set_termchg_current(150);

    bq25890h_set_rechg_voltage(100);

	bq25890h_set_vindpm_vol(charger_ic, charger_ic->hw_aicl_point);

	#ifdef CONFIG_OPLUS_CHARGER_MTK
	if (get_boot_mode() == META_BOOT || get_boot_mode() == FACTORY_BOOT
		|| get_boot_mode() == ADVMETA_BOOT || get_boot_mode() == ATE_FACTORY_BOOT) {
		bq25890h_suspend_charger();
		bq25890h_disable_charging();
	} else {
		bq25890h_unsuspend_charger();
	}
	#else /* CONFIG_OPLUS_CHARGER_MTK */
	bq25890h_unsuspend_charger();
	#endif /* CONFIG_OPLUS_CHARGER_MTK */

    bq25890h_enable_charging();

    bq25890h_set_wdt_timer(REG07_BQ25890H_I2C_WATCHDOG_TIME_40S);
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

void is_iindpm_mode(void)
{
	int rc = 0;
	int val_buf;
	int reg_ovp;
	int val_buf1;
	int charger_type = 0;

	rc = bq25890h_read_reg(charger_ic, REG13_BQ25890H_ADDRESS, &reg_ovp);
	val_buf = reg_ovp & 0x40;
	rc = bq25890h_read_reg(charger_ic, REG0B_BQ25890H_ADDRESS, &reg_ovp);
	val_buf1 = reg_ovp & 0x18;
	if ((val_buf == 0x40) || (val_buf1 == 0x10)) {
		charger_type = mt_power_supply_type_check();
		if (charger_type == POWER_SUPPLY_TYPE_USB) {
			rc = bq25890h_config_interface(charger_ic, REG00_BQ25890H_ADDRESS, REG00_BQ25890H_INPUT_CURRENT_LIMIT_500MA, REG00_BQ25890H_INPUT_CURRENT_LIMIT_MASK);
			bq25890h_dump_registers();
			printk("is_iindpm_mode_usb\n");
		} else if (charger_type == POWER_SUPPLY_TYPE_USB_DCP){
			rc = bq25890h_config_interface(charger_ic, REG00_BQ25890H_ADDRESS, REG00_BQ25890H_INPUT_CURRENT_LIMIT_2000MA, REG00_BQ25890H_INPUT_CURRENT_LIMIT_MASK);
			bq25890h_dump_registers();
			printk("is_iindpm_mode_dcp\n");
		}
	}
}

extern bool oplus_chg_get_shortc_hw_gpio_status(void);
extern int oplus_chg_shortc_hw_parse_dt(struct chip_bq25890h *chip);
struct oplus_chg_operations  bq25890h_chg_ops = {
	.dump_registers = bq25890h_dump_registers,
	.kick_wdt = bq25890h_kick_wdt,
	.hardware_init = bq25890h_hardware_init,
	.charging_current_write_fast = bq25890h_charging_current_write_fast,
	.set_aicl_point = bq25890h_set_aicl_point,
	.input_current_write = bq25890h_input_current_limit_write,
	.float_voltage_write = bq25890h_float_voltage_write,
	.term_current_set = bq25890h_set_termchg_current,
	.charging_enable = bq25890h_enable_charging,
	.charging_disable = bq25890h_disable_charging,
	.get_charging_enable = bq25890h_check_charging_enable,
	.charger_suspend = bq25890h_suspend_charger,
	.charger_unsuspend = bq25890h_unsuspend_charger,
	.set_rechg_vol = bq25890h_set_rechg_voltage,
	.reset_charger = bq25890h_reset_charger,
	.read_full = bq25890h_registers_read_full,
	.otg_enable = bq25890h_otg_enable,
	.otg_disable = bq25890h_otg_disable,
	.set_charging_term_disable = bq25890h_set_chging_term_disable,
	.check_charger_resume = bq25890h_check_charger_resume,
	.get_chg_current_step = bq25890h_get_chg_current_step,
	#ifdef CONFIG_OPLUS_CHARGER_MTK
	.get_charger_type = mt_power_supply_type_check,
	//.get_chg_pretype = charger_pretype_get,
	.get_charger_volt = battery_meter_get_charger_voltage,
	.check_chrdet_status = (bool (*) (void)) pmic_chrdet_status,
	.get_instant_vbatt = oplus_battery_meter_get_battery_voltage,
	.get_boot_mode = (int (*)(void))get_boot_mode,
	.get_boot_reason = (int (*)(void))get_boot_reason,
//	#ifdef CONFIG_MTK_HAFG_20
	.get_chargerid_volt = mt_get_chargerid_volt,

	.set_chargerid_switch_val = mt_set_chargerid_switch_val ,
	.get_chargerid_switch_val  = mt_get_chargerid_switch_val,
	//.set_usb_shell_ctrl_val = mt_set_usb_shell_ctrl_val,
	//.get_usb_shell_ctrl_val = mt_get_usb_shell_ctrl_val,
//	#endif /* CONFIG_MTK_HAFG_20 */
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

	#ifdef CONFIG_OPLUS_RTC_DET_SUPPORT
	.check_rtc_reset = rtc_reset_check,
	#endif
	#ifdef CONFIG_OPLUS_SHORT_C_BATT_CHECK
	.get_dyna_aicl_result = oplus_chg_get_dyna_aicl_result,
	#endif
	.get_shortc_hw_gpio_status = oplus_chg_get_shortc_hw_gpio_status,
	.check_is_iindpm_mode = is_iindpm_mode,
};

static void register_charger_devinfo(void)
{
	int ret = 0;
	char *version = "bq25890h";
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
			while ((is_usb_rdy() == false && timeout > 0)
					/*keep "Ibus < 200mA" for 1s, so vooc/svooc adapter go into idle and release D+*/
					|| (is_usb_rdy() == true && timeout > 340)) {
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

enum charger_type mt_charger_type_detection_bq25890h(void)
{
	int rc =0;
	int addr = 0;
	int val_buf;
	int count = 0;
	int i = 0;

	if (!charger_ic) {
		pr_err("%s charger_ic null,return\n", __func__);
		return CHARGER_UNKNOWN;
	}

	if (get_boot_mode() == META_BOOT || get_boot_mode() == FACTORY_BOOT
		|| get_boot_mode() == ADVMETA_BOOT || get_boot_mode() == ATE_FACTORY_BOOT) {
		return STANDARD_HOST;
	}

	addr = BQ25890H_FIRST_REG + 0x0b;

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
	//bq25890h_config_interface(charger_ic, REG00_BQ25890H_ADDRESS, 0, 0x80);
	bq25890h_config_interface(charger_ic, REG02_BQ25890H_ADDRESS, 0x01, 0);
	bq25890h_config_interface(charger_ic, REG02_BQ25890H_ADDRESS, 0x02, 0);
	bq25890h_dump_registers();
	
	rc = bq25890h_read_reg(charger_ic, REG02_BQ25890H_ADDRESS, &val_buf);
	val_buf &= 0x02;
	while (val_buf == 0x02 && count < 20) {
		count++;
		rc = bq25890h_read_reg(charger_ic, REG02_BQ25890H_ADDRESS, &val_buf);
		val_buf &= 0x02;
		usleep_range(50000, 50200);
		if (!upmu_get_rgs_chrdet()) {
			MTK_CHR_Type_num = CHARGER_UNKNOWN;
			bq25890h_config_interface(charger_ic, REG02_BQ25890H_ADDRESS, 0, 0xff);
			return MTK_CHR_Type_num;
		}
	}

	printk("mt_charger_type_detection2, count=%d\n", count);
	if (count == 20) {
		MTK_CHR_Type_num = CHARGER_UNKNOWN;
		bq25890h_config_interface(charger_ic, REG02_BQ25890H_ADDRESS, 0, 0xff);
		return MTK_CHR_Type_num;
	}

	rc = bq25890h_read_reg(charger_ic, addr, &val_buf);
	bq25890h_dump_registers();
	val_buf = val_buf & 0xe0;
	printk("mt_charger_type_detection3, val_buf=[0x%x]\n", val_buf);

	if (val_buf == 0x0) {
		MTK_CHR_Type_num = CHARGER_UNKNOWN;
	} else if (val_buf == 0x20) {
		MTK_CHR_Type_num = STANDARD_HOST;
	} else if (val_buf == 0x40) {
		MTK_CHR_Type_num = CHARGING_HOST;
	} else if (val_buf == 0x60 || val_buf == 0x80 || val_buf == 0xA0 || val_buf == 0xC0) {
		MTK_CHR_Type_num = APPLE_2_1A_CHARGER;
	} else {
		MTK_CHR_Type_num = APPLE_2_1A_CHARGER;
	}

	/* the 2nd detection */
	if (MTK_CHR_Type_num == CHARGER_UNKNOWN && upmu_get_rgs_chrdet()) {
		printk("mt_charger_type_detection: 2nd...\n");
		bq25890h_config_interface(charger_ic, REG02_BQ25890H_ADDRESS, 0x01, 0);
		bq25890h_config_interface(charger_ic, REG02_BQ25890H_ADDRESS, 0x02, 0);
		bq25890h_dump_registers();
		
		rc = bq25890h_read_reg(charger_ic, REG02_BQ25890H_ADDRESS, &val_buf);
		val_buf &= 0x02;
		while (val_buf == 0x02 && count < 20) {
			count++;
			rc = bq25890h_read_reg(charger_ic, REG02_BQ25890H_ADDRESS, &val_buf);
			val_buf &= 0x02;
			usleep_range(50000, 50200);
			if (!upmu_get_rgs_chrdet()) {
				MTK_CHR_Type_num = CHARGER_UNKNOWN;
				bq25890h_config_interface(charger_ic, REG02_BQ25890H_ADDRESS, 0, 0xff);
				return MTK_CHR_Type_num;
			}
		}

		printk("mt_charger_type_detection: 2nd, count=%d\n", count);
		if (count == 20) {
			MTK_CHR_Type_num = APPLE_2_1A_CHARGER;
			bq25890h_config_interface(charger_ic, REG02_BQ25890H_ADDRESS, 0, 0xff);
			return MTK_CHR_Type_num;
		}

		rc = bq25890h_read_reg(charger_ic, addr, &val_buf);
		bq25890h_dump_registers();
		val_buf = val_buf & 0xe0;
		printk("mt_charger_type_detection: 2nd, val_buf=[0x%x]\n", val_buf);
		if (val_buf == 0x0) {
			MTK_CHR_Type_num = APPLE_2_1A_CHARGER;
		} else if (val_buf == 0x20) {
			MTK_CHR_Type_num = STANDARD_HOST;
		} else if (val_buf == 0x40) {
			MTK_CHR_Type_num = CHARGING_HOST;
		} else if (val_buf == 0x60 || val_buf == 0x80 || val_buf == 0xA0 || val_buf == 0xC0) {
			MTK_CHR_Type_num = APPLE_2_1A_CHARGER;
		} else {
			MTK_CHR_Type_num = APPLE_2_1A_CHARGER;
		}
	}

	Charger_Detect_Release();
	//MTK_CHR_Type_num = APPLE_2_1A_CHARGER;
	bq25890h_config_interface(charger_ic, REG02_BQ25890H_ADDRESS, 0, 0xff);

	return MTK_CHR_Type_num;
}
#endif
static void do_charger_modefy_work(struct work_struct *data)
{
	/*
	if(charger_ic != NULL) {
		mt_charger_type_detection_bq25890h();
	}
	*/
}

static struct delayed_work bq25890h_irq_delay_work;
bool fg_bq25890h_irq_delay_work_running = false;
static void do_bq25890h_irq_delay_work(struct work_struct *data)
{	
	int val_buf = 0;
	int val_reg = 0;
	int i = 0;
	int otg_overcurrent_flag = 0;

	if (!charger_ic) {
		pr_err("%s charger_ic null,return\n", __func__);
		return ;
	}
	
	for (i = 0; i < 10; i++) {
		bq25890h_read_reg(charger_ic, REG0C_BQ25890H_ADDRESS, &val_reg);
		val_buf = val_reg & 0x40;
		if (val_buf == 0x40) {
			otg_overcurrent_flag++;
		}
		usleep_range(10000, 10200);
	}
	printk("[OPLUS_CHG] do_bq25890h_irq_delay_work flag=%d, val_reg[0x%x]\n", otg_overcurrent_flag, val_reg);
	if (otg_overcurrent_flag >= 8) {
		bq25890h_otg_disable();
	}
	fg_bq25890h_irq_delay_work_running = false;
}


static irqreturn_t bq25890h_irq_handler_fn(int irq, void *dev_id)
{
	//int val_buf;

	pr_err("%s start\n", __func__);
	if (!charger_ic) {
		pr_err("%s charger_ic null,return\n", __func__);
		return IRQ_HANDLED;
	}
	if (oplus_otg_online == 0) {
		return IRQ_HANDLED;
	}
	if(fg_bq25890h_irq_delay_work_running == false)
	{
		fg_bq25890h_irq_delay_work_running = true;
	    schedule_delayed_work(&bq25890h_irq_delay_work, round_jiffies_relative(msecs_to_jiffies(50)));
	}

	return IRQ_HANDLED;
}

static int bq25890h_parse_dts(void)
{
	int ret = 0;
	struct chip_bq25890h *chip = charger_ic;

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

static int bq25890h_irq_registration(void)
{
	int ret = 0;
	struct chip_bq25890h *chip = charger_ic;

	if (!chip || chip->irq_gpio <= 0) {
		chg_err("chip->irq_gpio fail\n");
		return -1;
	}

	ret = request_threaded_irq(gpio_to_irq(chip->irq_gpio), NULL,
		bq25890h_irq_handler_fn,
		IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
		"BQ25890H-eint", chip);
	if (ret < 0) {
		printk("BQ25890H request_irq IRQ LINE NOT AVAILABLE!.");
		return -EFAULT;
	}
	return 0;
}

static int bq25890h_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int reg = 0;
	struct chip_bq25890h *chip = NULL;

#ifndef CONFIG_OPLUS_CHARGER_MTK
	struct power_supply *usb_psy;
	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
        chg_err("USB psy not found; deferring probe\n");
        return -EPROBE_DEFER;
	}
#endif  /* CONFIG_OPLUS_CHARGER_MTK */
	chg_debug( " call \n");

	chip = devm_kzalloc(&client->dev,
		sizeof(struct chip_bq25890h), GFP_KERNEL);
	if (!chip) {
		chg_err(" kzalloc() failed\n");
		return -ENOMEM;
	}
	chip->client = client;
	chip->dev = &client->dev;
	charger_ic = chip;
	reg = bq25890h_check_registers();
	if (reg < 0) {
		return -ENODEV;
	}
	charger_ic_flag = 1;
	//chip->chg_ops = &bq25890h_chg_ops;
	//oplus_chg_shortc_hw_parse_dt(chip);
	
	INIT_DELAYED_WORK(&bq25890h_irq_delay_work, do_bq25890h_irq_delay_work);
	bq25890h_parse_dts();
	bq25890h_irq_registration();
	atomic_set(&chip->charger_suspended, 0);
	register_charger_devinfo();
	usleep_range(1000, 1200);
	INIT_DELAYED_WORK(&charger_modefy_work, do_charger_modefy_work);
	schedule_delayed_work(&charger_modefy_work, 0);

	return 0;

}


static struct i2c_driver bq25890h_i2c_driver;

static int bq25890h_driver_remove(struct i2c_client *client)
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
static int bq25890h_resume(struct device *dev)
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

static int bq25890h_suspend(struct device *dev)
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
static const struct dev_pm_ops bq25890h_pm_ops = {
	.resume		= bq25890h_resume,
	.suspend		= bq25890h_suspend,
};


#else //(LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
static int bq25890h_resume(struct i2c_client *client)
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
	oplus_chg_soc_update_when_resume(sleep_time);
	return 0;
}

static int bq25890h_suspend(struct i2c_client *client, pm_message_t mesg)
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

static void bq25890h_reset(struct i2c_client *client)
{
	bq25890h_otg_disable();
}

/**********************************************************
 * *
 * *   [platform_driver API]
 * *
 * *********************************************************/

static const struct of_device_id bq25890h_match[] = {
	{ .compatible = "oplus,bq25890h-charger"},
	{ },
};

static const struct i2c_device_id bq25890h_id[] = {
	{"bq25890h-charger", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, bq25890h_id);


static struct i2c_driver bq25890h_i2c_driver = {
	.driver		= {
		.name = "bq25890h-charger",
		.owner	= THIS_MODULE,
		.of_match_table = bq25890h_match,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
		.pm		= &bq25890h_pm_ops,
#endif /*CONFIG_OPLUS_CHARGER_MTK6763*/
	},
	.probe		= bq25890h_driver_probe,
	.remove		= bq25890h_driver_remove,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0))
	.resume		= bq25890h_resume,
	.suspend	= bq25890h_suspend,
#endif
	.shutdown	= bq25890h_reset,
	.id_table	= bq25890h_id,
};


module_i2c_driver(bq25890h_i2c_driver);
MODULE_DESCRIPTION("Driver for bq25890h charger chip");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:bq25890h-charger");
