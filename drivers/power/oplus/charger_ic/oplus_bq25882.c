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
//#include <upmu_common.h>
//#include <mt-plat/mtk_gpio.h>
//#include <mtk_boot_common.h>
#include <mt-plat/mtk_rtc.h>
//#include <mt-plat/charger_type.h>
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
#include <oplus_bq25882.h>


static struct chip_bq25882 *charger_ic = NULL;
static int aicl_result = 500;

static DEFINE_MUTEX(bq25882_i2c_access);

static int __bq25882_read_reg(int reg, int *returnData)
{
    int ret = 0;
    struct chip_bq25882 *chip = charger_ic;

    ret = i2c_smbus_read_byte_data(chip->client, reg);
    if (ret < 0) {
        chg_err("i2c read fail: can't read from %02x: %d\n", reg, ret);
        return ret;
    } else {
        *returnData = ret;
    }

    return 0;
}

static int bq25882_read_reg(int reg, int *returnData)
{
    int ret = 0;

    mutex_lock(&bq25882_i2c_access);
    ret = __bq25882_read_reg(reg, returnData);
    mutex_unlock(&bq25882_i2c_access);
    return ret;
}

static int __bq25882_write_reg(int reg, int val)
{
    int ret = 0;
    struct chip_bq25882 *chip = charger_ic;

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
/*
*/

static int bq25882_config_interface (int RegNum, int val, int MASK)
{
    int bq25882_reg = 0;
    int ret = 0;

    mutex_lock(&bq25882_i2c_access);
    ret = __bq25882_read_reg(RegNum, &bq25882_reg);


    bq25882_reg &= ~MASK;
    bq25882_reg |= val;

    ret = __bq25882_write_reg(RegNum, bq25882_reg);


    __bq25882_read_reg(RegNum, &bq25882_reg);

    mutex_unlock(&bq25882_i2c_access);

    return ret;
}


int bq25882_set_vindpm_vol(int vol)
{
    int rc;
    int tmp = 0;
    struct chip_bq25882 *chip = charger_ic;

    if (atomic_read(&chip->charger_suspended) == 1) {
        return 0;
    }

    tmp = (vol - REG02_BQ25882_INPUT_VOLTAGE_LIMIT_OFFSET)/REG02_BQ25882_INPUT_VOLTAGE_LIMIT_STEP;
    rc = bq25882_config_interface(REG02_BQ25882_ADDRESS, tmp << REG02_BQ25882_INPUT_VOLTAGE_LIMIT_SHIFT, REG02_BQ25882_INPUT_VOLTAGE_LIMIT_MASK);

    return rc;
}

static int bq25882_usbin_input_current_limit[] = {
    500,    900,    1200,   1500,   1700,   2000,
};

int bq25882_input_Ilimit_disable(void)
{
    int rc = 0;
    struct chip_bq25882 *chip = charger_ic;

    if (atomic_read(&chip->charger_suspended) == 1)
        return 0;

    rc = bq25882_config_interface(REG01_BQ25882_ADDRESS, REG01_BQ25882_EN_ILIM_DISABLE, REG01_BQ25882_EN_ILIM_MASK);

    return rc;
}

static int bq25882_get_pre_icl_index(void)
{
    int rc = 0;
    int reg_val = 0;
    int icl_index = 0;
    struct chip_bq25882 *chip = charger_ic;

    if (atomic_read(&chip->charger_suspended) == 1)
        return 0;

    rc = bq25882_read_reg(REG03_BQ25882_ADDRESS, &reg_val);
    if (rc) {
        chg_err("Couldn't read REG03_BQ25882_ADDRESS rc = %d\n", rc);
        return icl_index;
    }

    reg_val &= REG03_BQ25882_INPUT_CURRENT_LIMIT_MASK;
    switch (reg_val) {
        case REG03_BQ25882_INPUT_CURRENT_LIMIT_500MA:
            icl_index = 0;
            break;
        case REG03_BQ25882_INPUT_CURRENT_LIMIT_900MA:
            icl_index = 1;
            break;
        case REG03_BQ25882_INPUT_CURRENT_LIMIT_1200MA:
            icl_index = 2;
            break;
        case REG03_BQ25882_INPUT_CURRENT_LIMIT_1500MA:
            icl_index = 3;
            break;
        case REG03_BQ25882_INPUT_CURRENT_LIMIT_1700MA:
            icl_index = 4;
            break;
        case REG03_BQ25882_INPUT_CURRENT_LIMIT_2000MA:
            icl_index = 5;
            break;
        default:
            icl_index = 0;
            break;
    }
    chg_debug( "pre icl=%d setting %02x\n", bq25882_usbin_input_current_limit[icl_index], icl_index);
    return icl_index;
}

static int bq25882_get_charger_vol(void)
{
    int chg_vol = 0;

#ifdef CONFIG_OPLUS_CHARGER_MTK
    chg_vol = battery_meter_get_charger_voltage();
#else
    chg_vol = qpnp_get_prop_charger_voltage_now();
#endif
    return chg_vol;
}

int bq25882_input_current_limit_write(int current_ma)
{
    int i = 0;
    int chg_vol = 0;
    int pre_icl_index = 0;
    struct chip_bq25882 *chip = charger_ic;

    if (atomic_read(&chip->charger_suspended) == 1)
        return 0;

    for (i = ARRAY_SIZE(bq25882_usbin_input_current_limit) - 1; i >= 0; i--) {
        if (bq25882_usbin_input_current_limit[i] <= current_ma) {
            break;
        } else if (i == 0) {
            break;
        }
    }

    chg_debug( "usb input max current limit=%d setting %02x\n", current_ma, i);

    pre_icl_index = bq25882_get_pre_icl_index();
    if (pre_icl_index < 2) {
        //<1.2A, have been set 500 in bq25882_hardware_init
    } else {
        for (i = pre_icl_index - 1; i > 0; i--) {
            switch(i) {
                case 1:
                    bq25882_config_interface(REG03_BQ25882_ADDRESS, REG03_BQ25882_INPUT_CURRENT_LIMIT_900MA, REG03_BQ25882_INPUT_CURRENT_LIMIT_MASK);
                    break;
                case 2:
                    bq25882_config_interface(REG03_BQ25882_ADDRESS, REG03_BQ25882_INPUT_CURRENT_LIMIT_1200MA, REG03_BQ25882_INPUT_CURRENT_LIMIT_MASK);
                    break;
                case 3:
                    bq25882_config_interface(REG03_BQ25882_ADDRESS, REG03_BQ25882_INPUT_CURRENT_LIMIT_1500MA, REG03_BQ25882_INPUT_CURRENT_LIMIT_MASK);
                    break;
                case 4:
                    bq25882_config_interface(REG03_BQ25882_ADDRESS, REG03_BQ25882_INPUT_CURRENT_LIMIT_1700MA, REG03_BQ25882_INPUT_CURRENT_LIMIT_MASK);
                    break;
                case 5:
                    bq25882_config_interface(REG03_BQ25882_ADDRESS, REG03_BQ25882_INPUT_CURRENT_LIMIT_2000MA, REG03_BQ25882_INPUT_CURRENT_LIMIT_MASK);
                    break;
                default:
                    break;
            }
            msleep(50);
        }
    }

    ///bq25882_set_vindpm_vol(3900);

    i = 0; /* 500 */
    bq25882_config_interface(REG03_BQ25882_ADDRESS, REG03_BQ25882_INPUT_CURRENT_LIMIT_500MA, REG03_BQ25882_INPUT_CURRENT_LIMIT_MASK);
    msleep(90);
    chg_vol = bq25882_get_charger_vol();
    if (chg_vol < chip->sw_aicl_point) {
        chg_debug( "use 500 here\n");
        goto aicl_end;
    } else if (current_ma < 900)
        goto aicl_end;

    i = 1; /* 900 */
    bq25882_config_interface(REG03_BQ25882_ADDRESS, REG03_BQ25882_INPUT_CURRENT_LIMIT_900MA, REG03_BQ25882_INPUT_CURRENT_LIMIT_MASK);
    msleep(90);
    chg_vol = bq25882_get_charger_vol();
    if (chg_vol < chip->sw_aicl_point) {
        i = i - 1;
        goto aicl_pre_step;
    } else if (current_ma < 1200)
        goto aicl_end;

    i = 2; /* 1200 */
    bq25882_config_interface(REG03_BQ25882_ADDRESS, REG03_BQ25882_INPUT_CURRENT_LIMIT_1200MA, REG03_BQ25882_INPUT_CURRENT_LIMIT_MASK);
    msleep(90);
    chg_vol = bq25882_get_charger_vol();
    if (chg_vol < chip->sw_aicl_point) {
        i = i - 1;
        goto aicl_pre_step;
    }

    i = 3; /* 1500 */
    bq25882_config_interface(REG03_BQ25882_ADDRESS, REG03_BQ25882_INPUT_CURRENT_LIMIT_1500MA, REG03_BQ25882_INPUT_CURRENT_LIMIT_MASK);
    msleep(50);
    chg_vol = bq25882_get_charger_vol();
    if (chg_vol < chip->sw_aicl_point) {
        i = i - 2; //We DO NOT use 1.2A here
        goto aicl_pre_step;
    } else if (current_ma < 1500) {
        i = i - 1; //We use 1.2A here
        goto aicl_end;
    } else if (current_ma < 2000)
        goto aicl_end;

    i = 4; /* 1700 */
    bq25882_config_interface(REG03_BQ25882_ADDRESS, REG03_BQ25882_INPUT_CURRENT_LIMIT_1700MA, REG03_BQ25882_INPUT_CURRENT_LIMIT_MASK);
    msleep(90);
    chg_vol = bq25882_get_charger_vol();
    if (chg_vol < chip->sw_aicl_point) {
        i = i - 2; //1.2
        goto aicl_pre_step;
    }

    i = 5; /* 2000 */
    bq25882_config_interface(REG03_BQ25882_ADDRESS, REG03_BQ25882_INPUT_CURRENT_LIMIT_2000MA, REG03_BQ25882_INPUT_CURRENT_LIMIT_MASK);
    msleep(90);
    chg_vol = bq25882_get_charger_vol();
    if (chg_vol < chip->sw_aicl_point) {
        i = i - 2; //1.5
        goto aicl_pre_step;
    } else if (current_ma < 3000)
        goto aicl_end;

    aicl_pre_step:
        chg_debug( "usb input max current limit aicl chg_vol=%d j[%d]=%d sw_aicl_point:%d aicl_pre_step\n", chg_vol, i, bq25882_usbin_input_current_limit[i], chip->sw_aicl_point);
        goto aicl_rerun;
    aicl_end:
        chg_debug( "usb input max current limit aicl chg_vol=%d j[%d]=%d sw_aicl_point:%d aicl_end\n", chg_vol, i, bq25882_usbin_input_current_limit[i], chip->sw_aicl_point);
        goto aicl_rerun;
    aicl_rerun:
        aicl_result = bq25882_usbin_input_current_limit[i];
        switch (i) {
            case 0:
                bq25882_config_interface(REG03_BQ25882_ADDRESS, REG03_BQ25882_INPUT_CURRENT_LIMIT_500MA, REG03_BQ25882_INPUT_CURRENT_LIMIT_MASK);
                break;
            case 1:
                bq25882_config_interface(REG03_BQ25882_ADDRESS, REG03_BQ25882_INPUT_CURRENT_LIMIT_900MA, REG03_BQ25882_INPUT_CURRENT_LIMIT_MASK);
                break;
            case 2:
                bq25882_config_interface(REG03_BQ25882_ADDRESS, REG03_BQ25882_INPUT_CURRENT_LIMIT_1200MA, REG03_BQ25882_INPUT_CURRENT_LIMIT_MASK);
                break;
            case 3:
                bq25882_config_interface(REG03_BQ25882_ADDRESS, REG03_BQ25882_INPUT_CURRENT_LIMIT_1500MA, REG03_BQ25882_INPUT_CURRENT_LIMIT_MASK);
                break;
            case 4:
                bq25882_config_interface(REG03_BQ25882_ADDRESS, REG03_BQ25882_INPUT_CURRENT_LIMIT_1700MA, REG03_BQ25882_INPUT_CURRENT_LIMIT_MASK);
                break;
            case 5:
                bq25882_config_interface(REG03_BQ25882_ADDRESS, REG03_BQ25882_INPUT_CURRENT_LIMIT_2000MA, REG03_BQ25882_INPUT_CURRENT_LIMIT_MASK);
                break;
            default:
                break;
        }

    bq25882_set_vindpm_vol(chip->hw_aicl_point);
    return 0;
}

int bq25882_charging_current_write_fast(int chg_cur)
{	
    int rc = 0;
    int tmp = 0;
    struct chip_bq25882 *chip = charger_ic;

    chg_err( "chg_cur = %d\n", chg_cur);	
        if(atomic_read(&chip->charger_suspended) == 1) {
        return 0;
    }

    tmp = chg_cur - REG01_BQ25882_FAST_CURRENT_LIMIT_OFFSET;
    tmp = tmp / REG01_BQ25882_FAST_CURRENT_LIMIT_STEP;
    rc = bq25882_config_interface(REG01_BQ25882_ADDRESS, tmp << REG01_BQ25882_FAST_CURRENT_LIMIT_SHIFT, REG01_BQ25882_FAST_CURRENT_LIMIT_MASK);

    return rc;
}

#ifdef CONFIG_OPLUS_SHORT_C_BATT_CHECK
/* This function is getting the dynamic aicl result/input limited in mA.
 * If charger was suspended, it must return 0(mA).
 * It meets the requirements in SDM660 platform.
 */
static int bq25882_chg_get_dyna_aicl_result(void)
{
    return aicl_result;
}
#endif /* CONFIG_OPLUS_SHORT_C_BATT_CHECK */

void bq25882_set_aicl_point(int vbatt)
{
    struct chip_bq25882 *chip = charger_ic;

    if (chip->hw_aicl_point == 4400 && vbatt > 4100) {
        chip->hw_aicl_point = 4500;
        chip->sw_aicl_point = 4550;
        bq25882_set_vindpm_vol(chip->hw_aicl_point);
    } else if (chip->hw_aicl_point == 4500 && vbatt <= 4100) {
        chip->hw_aicl_point = 4400;
        chip->sw_aicl_point = 4500;
        bq25882_set_vindpm_vol(chip->hw_aicl_point);
    }
}

int bq25882_set_enable_volatile_writes(void)
{
    int rc = 0;
    struct chip_bq25882 *chip = charger_ic;

    if(atomic_read(&chip->charger_suspended) == 1) {
        return 0;
    }
    //need do nothing

    return rc;
}

int bq25882_set_complete_charge_timeout(int val)
{
    int rc = 0;

    struct chip_bq25882 *chip = charger_ic;

    if(atomic_read(&chip->charger_suspended) == 1) {
        return 0;
    }

    if (val == OVERTIME_AC) {
        val = REG05_BQ25882_EN_TIMER_ENABLE | REG05_BQ25882_CHG_TIMER_8H;
    } else if (val == OVERTIME_USB) {
        val = REG05_BQ25882_EN_TIMER_ENABLE | REG05_BQ25882_CHG_TIMER_12H;
    } else {
        val = REG05_BQ25882_EN_TIMER_DISABLE | REG05_BQ25882_CHG_TIMER_8H;
    }

    rc = bq25882_config_interface(REG05_BQ25882_ADDRESS,
    val, REG05_BQ25882_EN_TIMER_MASK | REG05_BQ25882_CHG_TIMER_MASK);
    if (rc < 0) {
        chg_err("Couldn't complete charge timeout rc = %d\n", rc);
    }

    return 0;
}

int bq25882_float_voltage_write(int vfloat_mv)
{
    int rc = 0;
    int tmp = 0;
    struct chip_bq25882 *chip = charger_ic;

    if(atomic_read(&chip->charger_suspended) == 1) {
        return 0;
    }

    tmp = vfloat_mv - REG00_BQ25882_BAT_THRESHOLD_OFFSET;
    tmp = tmp / REG00_BQ25882_BAT_THRESHOLD_STEP;
    rc = bq25882_config_interface(REG00_BQ25882_ADDRESS, tmp << REG00_BQ25882_BAT_THRESHOLD_SHIFT, REG00_BQ25882_BAT_THRESHOLD_MASK);
    return rc;
}

int bq25882_set_prechg_current(int ipre_mA)
{
    int tmp = 0;
    int rc = 0;
    struct chip_bq25882 *chip = charger_ic;

    if(atomic_read(&chip->charger_suspended) == 1) {
        return 0;
    }

    tmp = ipre_mA - REG04_BQ25882_IPRECHG_LIMIT_OFFSET;
    tmp = tmp / REG04_BQ25882_IPRECHG_LIMIT_STEP;
    rc = bq25882_config_interface(REG04_BQ25882_ADDRESS, tmp << REG04_BQ25882_IPRECHG_LIMIT_SHIFT, REG04_BQ25882_IPRECHG_LIMIT_MASK);

    return 0;
}

int bq25882_set_termchg_current(int term_curr)
{
    int rc = 0;
    int tmp = 0;

    struct chip_bq25882 *chip = charger_ic;

    if(atomic_read(&chip->charger_suspended) == 1) {
        return 0;
    }

    tmp = term_curr - REG04_BQ25882_ITERM_LIMIT_OFFSET;
    tmp = tmp / REG04_BQ25882_ITERM_LIMIT_STEP;

    rc = bq25882_config_interface(REG04_BQ25882_ADDRESS, tmp << REG04_BQ25882_ITERM_LIMIT_SHIFT, REG04_BQ25882_ITERM_LIMIT_MASK);
    return rc;
}

int bq25882_set_rechg_voltage(int recharge_mv)
{
    int rc = 0;
    int tmp = 0;
    struct chip_bq25882 *chip = charger_ic;

    if(atomic_read(&chip->charger_suspended) == 1) {
        return 0;
    }

    tmp = recharge_mv - REG06_BQ25882_VRECHG_OFFSET;
    tmp = tmp / REG06_BQ25882_VRECHG_STEP;

    rc = bq25882_config_interface(REG06_BQ25882_ADDRESS, tmp << REG06_BQ25882_VRECHG_SHIFT, REG06_BQ25882_VRECHG_MASK);
    if (rc) {
        chg_err("Couldn't set recharging threshold rc = %d\n", rc);     
    }
    return rc;
}

int bq25882_set_wdt_timer(int reg)
{
    int rc = 0;
    struct chip_bq25882 *chip = charger_ic;

    if(atomic_read(&chip->charger_suspended) == 1) {
        return 0;
    }
    chg_err("bq25882_set_wdt_timer reg = %d\n", reg);

    rc = bq25882_config_interface(REG05_BQ25882_ADDRESS, reg, REG05_BQ25882_WATCHDOG_MASK);
    if (rc) {
        chg_err("bq25882_set_wdt_timer set watchdog fail rc = %d\n", rc);
    }

    return 0;
}

int bq25882_set_chging_term_disable(void)
{
    int rc = 0;
    struct chip_bq25882 *chip = charger_ic;

    if (atomic_read(&chip->charger_suspended) == 1) {
        return 0;
    }

    rc = bq25882_config_interface(REG05_BQ25882_ADDRESS, 
    REG05_BQ25882_EN_TERM_DISABLE, REG05_BQ25882_EN_TERM_MASK);
    if (rc) {
        chg_err("Couldn't set chging term disable rc = %d\n", rc);     
    }
    return rc;
}

int bq25882_kick_wdt(void)
{
    int rc = 0;
    struct chip_bq25882 *chip = charger_ic;

    if(atomic_read(&chip->charger_suspended) == 1) {
        return 0;
    }

    rc = bq25882_config_interface(REG07_BQ25882_ADDRESS, REG07_BQ25882_WD_RST_RESET, REG07_BQ25882_WD_RST_MASK);
    if (rc) {
        chg_err("Couldn't bq25882 kick wdt rc = %d\n", rc);     
    }
    return rc;
}

int bq25882_enable_charging(void)
{
    int rc;
    struct chip_bq25882 *chip = charger_ic;

    if(atomic_read(&chip->charger_suspended) == 1) {
        return 0;
    }

    rc = bq25882_config_interface(REG06_BQ25882_ADDRESS, 
    REG06_BQ25882_EN_CHG_ENABLE, REG06_BQ25882_EN_CHG_MASK);
    if (rc < 0) {
        chg_err("Couldn't bq25882_enable_charging rc = %d\n", rc);
    }

    return rc;
}

int bq25882_disable_charging(void)
{
    int rc;
    struct chip_bq25882 *chip = charger_ic;

    if(atomic_read(&chip->charger_suspended) == 1) {
        return 0;
    }
    rc = bq25882_config_interface(REG06_BQ25882_ADDRESS, 
    REG06_BQ25882_EN_CHG_DISABLE, REG06_BQ25882_EN_CHG_MASK);
    if (rc < 0) {
        chg_err("Couldn't bq25882_disable_charging  rc = %d\n", rc);
    }

    return rc;
}

int bq25882_check_charging_enable(void)
{
    int rc = 0, reg_val = 0;
    bool charging_enable = false;
    struct chip_bq25882 *chip = charger_ic;

    if(atomic_read(&chip->charger_suspended) == 1) {
        return 0;
    }
    rc = bq25882_read_reg(REG06_BQ25882_ADDRESS, &reg_val);
    if (rc) {
        chg_err("Couldn't read REG06_BQ25882_ADDRESS rc = %d\n", rc);
        return 0;
    }

    charging_enable = ((reg_val & REG06_BQ25882_EN_CHG_MASK) == REG06_BQ25882_EN_CHG_ENABLE) ? 1 : 0;

    return charging_enable;	
}

int bq25882_registers_read_full(void)
{
    int rc;
    int reg_full = 0;
    struct chip_bq25882 *chip = charger_ic;

    if(atomic_read(&chip->charger_suspended) == 1) {
        return 0;
    }
    rc = bq25882_read_reg(REG0B_BQ25882_ADDRESS, &reg_full);
    if (rc) {
        chg_err("Couldn't read STAT_C rc = %d\n", rc);
        return 0;
    }

    reg_full = ((reg_full & REG0B_BQ25882_CHGSTAT_MASK) == REG0B_BQ25882_CHGSTAT_CHGDONE) ? 1 : 0;

    return reg_full;
}

#if 1//ndef OPLUS_BQ25882

int bq25882_suspend_charger(void)
{
    int rc = 0;
    struct chip_bq25882 *chip = charger_ic;

    if (atomic_read(&chip->charger_suspended) == 1) {
        return 0;
    }

    rc = bq25882_config_interface(REG01_BQ25882_ADDRESS, REG01_BQ25882_EN_HIZ_ENABLE, REG01_BQ25882_EN_HIZ_MASK);
    if (rc < 0) {
        chg_err("Couldn't bq25882_suspend_charger rc = %d\n", rc);
    } else {
        chg_err("bq25882_suspend_charger\n");
    }

    return rc;
}

int bq25882_unsuspend_charger(void)
{
    int rc = 0;
    struct chip_bq25882 *chip = charger_ic;

    if (atomic_read(&chip->charger_suspended) == 1) {
        return 0;
    }

    rc = bq25882_config_interface(REG01_BQ25882_ADDRESS, REG01_BQ25882_EN_HIZ_DISABLE, REG01_BQ25882_EN_HIZ_MASK);
    if (rc < 0) {
        chg_err("Couldn't bq25882_unsuspend_charger rc = %d\n", rc);
    } else {
        //chg_err("bq25882_unsuspend_charger\n");
    }
    return rc;
}

bool bq25882_check_charger_resume(void)
{
    struct chip_bq25882 *chip = charger_ic;

    if(atomic_read(&chip->charger_suspended) == 1) {
        return false;
    } else {
        return true;
    }
}
#endif

int bq25882_reset_charger(void)
{
    int rc;
    struct chip_bq25882 *chip = charger_ic;

    if(atomic_read(&chip->charger_suspended) == 1) {
        return 0;
    }
    rc = bq25882_config_interface(REG25_BQ25882_ADDRESS, 
    REG25_BQ25882_RESET_DEFAULT, REG25_BQ25882_RESET_MASK);
    if (rc < 0) {
        chg_err("Couldn't bq25882_reset_charger  rc = %d\n", rc);
    }

    return rc;
}

int bq25882_otg_vlim_set(int vlim)
{
    int rc;
    struct chip_bq25882 *chip = charger_ic;

    if(atomic_read(&chip->charger_suspended) == 1) {
        return 0;
    }
    rc = bq25882_config_interface(REG09_BQ25882_ADDRESS,
    vlim, REG09_BQ25882_OTG_VLIM_MASK);
    if (rc < 0) {
        chg_err("Couldn't bq25882_otg_vlim_set  rc = %d\n", rc);
    }

    return rc;
}


int bq25882_otg_ilim_set(int ilim)
{
    int rc;
    struct chip_bq25882 *chip = charger_ic;

    if(atomic_read(&chip->charger_suspended) == 1) {
        return 0;
    }
    rc = bq25882_config_interface(REG09_BQ25882_ADDRESS,
    ilim, REG09_BQ25882_OTG_ILIM_MASK);
    if (rc < 0) {
        chg_err("Couldn't bq25882_otg_ilim_set  rc = %d\n", rc);
    }

    return rc;
}

int bq25882_otg_enable(void)
{
    int rc;
    struct chip_bq25882 *chip = charger_ic;

    if (atomic_read(&chip->charger_suspended) == 1) {
        return 0;
    }

    bq25882_unsuspend_charger();

    bq25882_adc_en_enable(true);

    rc = bq25882_otg_ilim_set(REG09_BQ25882_OTG_ILIM_1300MA);
    if (rc < 0) {
        chg_err("Couldn't bq25882_otg_ilim_set  rc = %d\n", rc);
    }

    rc = bq25882_config_interface(REG06_BQ25882_ADDRESS,
    REG06_BQ25882_EN_OTG_ENABLE, REG06_BQ25882_EN_OTG_MASK);
    if (rc < 0) {
        chg_err("Couldn't bq25882_otg_enable  rc = %d\n", rc);
    }
    bq25882_set_wdt_timer(REG05_BQ25882_WATCHDOG_DISABLE);

    return rc;
}

int bq25882_otg_disable(void)
{
    int rc;
    struct chip_bq25882 *chip = charger_ic;

    if (atomic_read(&chip->charger_suspended) == 1) {
        return 0;
    }
    rc = bq25882_config_interface(REG06_BQ25882_ADDRESS,
    REG06_BQ25882_EN_OTG_DISABLE, REG06_BQ25882_EN_OTG_MASK);
    if (rc < 0) {
        chg_err("Couldn't bq25882_otg_disable  rc = %d\n", rc);
    }

    bq25882_adc_en_enable(false);
    bq25882_set_wdt_timer(REG05_BQ25882_WATCHDOG_40S);

    return rc;
}

int bq25882_ico_disable(void)
{
    int rc;
    struct chip_bq25882 *chip = charger_ic;

    if(atomic_read(&chip->charger_suspended) == 1) {
        return 0;
    }
    rc = bq25882_config_interface(REG03_BQ25882_ADDRESS, 
    REG03_BQ25882_EN_ICO_DISABLE, REG03_BQ25882_EN_ICO_MASK);
    if (rc < 0) {
        chg_err("Couldn't bq25882_ico_disable  rc = %d\n", rc);
    }

    return rc;
}

int bq25882_adc_en_enable(bool enable)
{
    int rc;
    struct chip_bq25882 *chip = charger_ic;

    if (atomic_read(&chip->charger_suspended) == 1) {
        return 0;
    }

    if (enable == false) {
        rc = bq25882_config_interface(REG15_BQ25882_ADDRESS, 0x0, 0xFF);//REG15_BQ25882_ADC_EN_DISABLE, REG15_BQ25882_ADC_EN_MASK);
        if (rc < 0) {
            chg_err("Couldn't bq25882_adc_en_disable  rc = %d\n", rc);
        }
    } else {
        rc = bq25882_config_interface(REG15_BQ25882_ADDRESS, 0x80, 0xFF);//REG15_BQ25882_ADC_EN_ENABLE, REG15_BQ25882_ADC_EN_MASK);
        if (rc < 0) {
            chg_err("Couldn't bq25882_adc_en_enable  rc = %d\n", rc);
        }
    }
    return rc;
}

int bq25882_ibus_adc_dis_enable(void)
{
    int rc;
    struct chip_bq25882 *chip = charger_ic;

    if(atomic_read(&chip->charger_suspended) == 1) {
        return 0;
    }
    rc = bq25882_config_interface(REG16_BQ25882_ADDRESS, 0x00, 0xFF);//REG16_BQ25882_IBUS_ADC_DIS_ENABLE, REG16_BQ25882_IBUS_ADC_DIS_MASK);
    if (rc < 0) {
        chg_err("Couldn't bq25882_ibus_adc_dis_enable  rc = %d\n", rc);
    }

    return rc;
}

int bq25882_ichg_adc_dis_enable(void)
{
    int rc;
    struct chip_bq25882 *chip = charger_ic;

    if(atomic_read(&chip->charger_suspended) == 1) {
        return 0;
    }
    rc = bq25882_config_interface(REG16_BQ25882_ADDRESS, 0x00, 0xFF);//REG16_BQ25882_ICHG_ADC_DIS_ENABLE, REG16_BQ25882_ICHG_ADC_DIS_MASK);
    if (rc < 0) {
        chg_err("Couldn't bq25882_ichg_adc_dis_enable  rc = %d\n", rc);
    }

    return rc;
}

#define DUMP_REG_LOG_CNT_30S             6
void bq25882_dump_registers(void)
{
    int rc;
    int addr;
    static int dump_count = 0;

    struct chip_bq25882 *chip = charger_ic;

    unsigned int val_buf[BQ25882_REG_NUMBER] = {0x0};

    if(atomic_read(&chip->charger_suspended) == 1) {
        return ;
    }

    if(dump_count == DUMP_REG_LOG_CNT_30S) {
        dump_count = 0;
        for (addr = BQ25882_FIRST_REG; addr <= BQ25882_LAST_REG; addr++) {
            rc = bq25882_read_reg(addr, &val_buf[addr]);
            if (rc) {
                chg_err("Couldn't read 0x%02x rc = %d\n", addr, rc);
            }
        }
        printk(KERN_ERR "bq25882[0-0C]: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x,", 
			val_buf[0], val_buf[1], val_buf[2], val_buf[3], val_buf[4], val_buf[5], val_buf[6], val_buf[7], val_buf[8], val_buf[9], val_buf[10], val_buf[11], val_buf[12]);
        printk(KERN_ERR "bq25882[0D-19]: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x,", 
			val_buf[13], val_buf[14], val_buf[15], val_buf[16], val_buf[17], val_buf[18], val_buf[19], val_buf[20], val_buf[21], val_buf[22], val_buf[23], val_buf[24], val_buf[25]);
        printk(KERN_ERR "bq25882[1A-25]: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x \n", 
			val_buf[26], val_buf[27], val_buf[28], val_buf[29], val_buf[30], val_buf[31], val_buf[32], val_buf[33], val_buf[34], val_buf[35], val_buf[36], val_buf[37]);
    }
    dump_count++;
}

bool bq25882_need_to_check_ibatt(void)
{
    return false;
}

static int bq25882_get_chg_current_step(void)
{
    int rc = 50;

    return rc;
}

static int bq25882_input_current_limit_init(void)
{
    int rc = 0;
    struct chip_bq25882 *chip = charger_ic;

    if (atomic_read(&chip->charger_suspended) == 1) {
        chg_err("bq25882_input_current_limit_init: in suspended\n");
        return 0;
    }
    rc = bq25882_config_interface(REG03_BQ25882_ADDRESS,
        REG03_BQ25882_INPUT_CURRENT_LIMIT_500MA,
        REG03_BQ25882_INPUT_CURRENT_LIMIT_MASK);
    if (rc < 0) {
        chg_err("Couldn't bq25882_input_current_limit_init rc = %d\n", rc);
    }
    return rc;
}

int bq25882_hardware_init(void)
{
    //must be before set_vindpm_vol and set_input_current
    struct chip_bq25882 *chip = charger_ic;

    chip->hw_aicl_point = 4400;
    chip->sw_aicl_point = 4500;

    bq25882_reset_charger();

    bq25882_input_current_limit_init();

    bq25882_input_Ilimit_disable();

    bq25882_otg_vlim_set(REG09_BQ25882_OTG_VLIM_5100MV);
    bq25882_otg_ilim_set(REG09_BQ25882_OTG_ILIM_1300MA);

    bq25882_float_voltage_write(8800);

    bq25882_set_enable_volatile_writes();

    bq25882_set_complete_charge_timeout(OVERTIME_DISABLED);

    bq25882_set_prechg_current(300);

    //	bq25882_charging_current_write_fast(1500);
    bq25882_charging_current_write_fast(300);

    bq25882_set_termchg_current(100);

    bq25882_set_rechg_voltage(100);

    bq25882_set_vindpm_vol(chip->hw_aicl_point);

    bq25882_ico_disable();

    //bq25882_adc_en_enable();

    //bq25882_ibus_adc_dis_enable();

    //bq25882_ichg_adc_dis_enable();

#ifdef CONFIG_OPLUS_CHARGER_MTK
    if(get_boot_mode() == META_BOOT || get_boot_mode() == FACTORY_BOOT
    || get_boot_mode() == ADVMETA_BOOT || get_boot_mode() == ATE_FACTORY_BOOT)
    {
        bq25882_suspend_charger();
    } else {
        bq25882_unsuspend_charger();
    }
#else
    bq25882_unsuspend_charger();
#endif

    bq25882_enable_charging();

    bq25882_set_wdt_timer(REG05_BQ25882_WATCHDOG_40S);

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

struct oplus_chg_operations  bq25882_chg_ops = {
    .dump_registers = bq25882_dump_registers,
    .kick_wdt = bq25882_kick_wdt,
    .hardware_init = bq25882_hardware_init,
    .charging_current_write_fast = bq25882_charging_current_write_fast,
    .set_aicl_point = bq25882_set_aicl_point,
    .input_current_write = bq25882_input_current_limit_write,
    .float_voltage_write = bq25882_float_voltage_write,
    .term_current_set = bq25882_set_termchg_current,
    .charging_enable = bq25882_enable_charging,
    .charging_disable = bq25882_disable_charging,
    .get_charging_enable = bq25882_check_charging_enable,
    .charger_suspend = bq25882_suspend_charger,
    .charger_unsuspend = bq25882_unsuspend_charger,
    .set_rechg_vol = bq25882_set_rechg_voltage,
    .reset_charger = bq25882_reset_charger,
    .read_full = bq25882_registers_read_full,
    .otg_enable = bq25882_otg_enable,
    .otg_disable = bq25882_otg_disable,
    .set_charging_term_disable = bq25882_set_chging_term_disable,
    .check_charger_resume = bq25882_check_charger_resume,
    .get_chg_current_step = bq25882_get_chg_current_step,
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
    .get_rtc_soc = get_rtc_spare_oplus_fg_value,
    .set_rtc_soc = set_rtc_spare_oplus_fg_value,
    .set_power_off = mt_power_off,
    .usb_connect = mt_usb_connect,
    .usb_disconnect = mt_usb_disconnect,
#else /* CONFIG_OPLUS_CHARGER_MTK */
    .get_chargerid_volt = smbchg_get_chargerid_volt,
    .set_chargerid_switch_val = smbchg_set_chargerid_switch_val,
    .get_chargerid_switch_val = smbchg_get_chargerid_switch_val,
    .need_to_check_ibatt = bq25882_need_to_check_ibatt,
    .get_charger_type = opchg_get_charger_type,
    .get_charger_volt = qpnp_get_prop_charger_voltage_now,
    .check_chrdet_status = oplus_chg_is_usb_present,
    .get_instant_vbatt = qpnp_get_battery_voltage,
    .get_boot_mode = get_boot_mode,
    .get_boot_reason = smbchg_get_boot_reason,
    .get_rtc_soc = oplus_chg_get_shutdown_soc,
    .set_rtc_soc = oplus_chg_backup_soc,
#endif /* CONFIG_OPLUS_CHARGER_MTK */
#ifdef CONFIG_OPLUS_SHORT_C_BATT_CHECK
    .get_dyna_aicl_result = bq25882_chg_get_dyna_aicl_result,
#endif
#ifdef CONFIG_OPLUS_RTC_DET_SUPPORT
    .check_rtc_reset = rtc_reset_check,
#endif
};

struct oplus_chg_operations * oplus_get_chg_ops(void)
{
    return &bq25882_chg_ops;
}

static void register_charger_devinfo(void)
{
    int ret = 0;
    char *version;
    char *manufacture;

    version = "bq25882";
    manufacture = "MAXIM";

    ret = register_device_proc("charger", version, manufacture);
    if (ret)
        chg_err("register_charger_devinfo fail\n");
}

bool oplus_charger_ic_chip_is_null(void)
{
    if (!charger_ic) {
        return true;
    } else {
        return false;
    }
}

static int bq25882_driver_probe(struct i2c_client *client, const struct i2c_device_id *id) 
{
    int reg = 0;
    struct chip_bq25882 *chg_ic;

    chg_ic = devm_kzalloc(&client->dev,
    sizeof(struct chip_bq25882), GFP_KERNEL);
    if (!chg_ic) {
        dev_err(&client->dev, "failed to allocate device info data\n");
        return -ENOMEM;
    }

    chg_debug( " call \n");
    chg_ic->client = client;
    chg_ic->dev = &client->dev;
    charger_ic = chg_ic;

    atomic_set(&chg_ic->charger_suspended, 0);
    bq25882_dump_registers();

    bq25882_hardware_init();

    register_charger_devinfo();

    chg_debug(" success\n");

    return reg;                                                                                       
}


static struct i2c_driver bq25882_i2c_driver;

static int bq25882_driver_remove(struct i2c_client *client)
{

    int ret=0;

    //ret = i2c_del_driver(&bq25882_i2c_driver);
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
static int bq25882_pm_resume(struct device *dev)
{
    unsigned long resume_tm_sec = 0;
    unsigned long sleep_time = 0;
    int rc = 0;
    struct chip_bq25882 *chip = charger_ic;

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

static int bq25882_pm_suspend(struct device *dev)
{
    struct chip_bq25882 *chip = charger_ic;

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

static const struct dev_pm_ops bq25882_pm_ops = {
    .resume         = bq25882_pm_resume,
    .suspend        = bq25882_pm_suspend,
};
#else
static int bq25882_resume(struct i2c_client *client)
{	
    unsigned long resume_tm_sec = 0;
    unsigned long sleep_time = 0;
    int rc = 0;
    struct chip_bq25882 *chip = charger_ic;

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

static int bq25882_suspend(struct i2c_client *client, pm_message_t mesg)
{
    struct chip_bq25882 *chip = charger_ic;

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

static void bq25882_reset(struct i2c_client *client)
{
    bq25882_otg_disable();
}


/**********************************************************
  *
  *   [platform_driver API] 
  *
  *********************************************************/

static const struct of_device_id bq25882_match[] = {
    { .compatible = "oplus,bq25882-charger"},
    { },
};

static const struct i2c_device_id bq25882_id[] = {
    {"bq25882-charger", 0},
    {},
};
MODULE_DEVICE_TABLE(i2c, bq25882_id);


static struct i2c_driver bq25882_i2c_driver = {
    .driver     = {
        .name = "bq25882-charger",
        .owner	= THIS_MODULE,
        .of_match_table = bq25882_match,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
        .pm     = &bq25882_pm_ops,
#endif
    },
    .probe      = bq25882_driver_probe,
    .remove     = bq25882_driver_remove,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0))
    .resume     = bq25882_resume,
    .suspend    = bq25882_suspend,
#endif
    .shutdown   = bq25882_reset,
    .id_table   = bq25882_id,
};

module_i2c_driver(bq25882_i2c_driver);
MODULE_DESCRIPTION("Driver for bq25882 charger chip");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:bq25882-charger");

