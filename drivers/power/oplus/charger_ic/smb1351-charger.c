/* Copyright (c) 2016-2017 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
//#include <linux/qpnp/qpnp-adc.h>
#include <linux/pinctrl/consumer.h>
#include <smb1351-charger.h>
#include "../oplus_vooc.h"
#include "../oplus_gauge.h"
#include "../oplus_charger.h"
#include <mt-plat/charger_type.h>


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
//#include <mt-plat/battery_common.h>
#else
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_boot.h>
#include <mach/mtk_rtc.h>
#endif
#include <soc/oplus/device_info.h>



//extern void mt_power_off(void);
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

static struct smb1351_charger *smb1351_chip = NULL;
extern int oplus_get_rtc_ui_soc(void);
extern int oplus_set_rtc_ui_soc(int value);
extern int charger_ic_flag;
static int aicl_result = 500;
extern int battery_meter_get_charger_voltage(void);
extern bool pmic_chrdet_status(void);
static int smb1351_registers_read_full(void);
void smb1351_dump_regs(void);
static int smb1351_reset_charger(void);
extern int mt_power_supply_type_check(void);
extern void mt_set_chargerid_switch_val(int value);
extern int charger_pretype_get(void);
extern int mt_get_chargerid_switch_val(void);
extern bool oplus_pmic_check_chip_is_null(void);
extern int mt_get_chargerid_volt (void);
extern int oplus_battery_meter_get_battery_voltage(void);
static int smb1351_read_reg(struct smb1351_charger *chip, int reg, u8 *val)
{
	s32 ret;

	pm_stay_awake(chip->dev);
	ret = i2c_smbus_read_byte_data(chip->client, reg);
	if (ret < 0) {
		pr_err("i2c read fail: can't read from %02x: %d\n", reg, ret);
		pm_relax(chip->dev);
		return ret;
	} else {
		*val = ret;
	}
	pm_relax(chip->dev);
//	pr_err("Reading 0x%02x=0x%02x\n", reg, *val);
	return 0;
}

static int smb1351_write_reg(struct smb1351_charger *chip, int reg, u8 val)
{
	s32 ret;

	pm_stay_awake(chip->dev);
	ret = i2c_smbus_write_byte_data(chip->client, reg, val);
	if (ret < 0) {
		pr_err("i2c write fail: can't write %02x to %02x: %d\n",
			val, reg, ret);
		pm_relax(chip->dev);
		return ret;
	}
	pm_relax(chip->dev);
//	pr_err("Writing 0x%02x=0x%02x\n", reg, val);
	return 0;
}

static int smb1351_masked_write(struct smb1351_charger *chip, int reg,
							u8 mask, u8 val)
{
	s32 rc;
	u8 temp;

	rc = smb1351_read_reg(chip, reg, &temp);
	if (rc) {
		pr_err("read failed: reg=%03X, rc=%d\n", reg, rc);
		return rc;
	}
	temp &= ~mask;
	temp |= val & mask;
	rc = smb1351_write_reg(chip, reg, temp);
	if (rc) {
		pr_err("write failed: reg=%03X, rc=%d\n", reg, rc);
		return rc;
	}
	return 0;
}

static int smb1351_enable_volatile_writes(struct smb1351_charger *chip)
{
	int rc;

	rc = smb1351_masked_write(chip, CMD_I2C_REG, CMD_BQ_CFG_ACCESS_BIT,
							CMD_BQ_CFG_ACCESS_BIT);
	if (rc)
		pr_err("Couldn't write CMD_BQ_CFG_ACCESS_BIT rc=%d\n", rc);

	return rc;
}

static int smbchg_charging_suspend(struct smb1351_charger *chip, bool enable)
{
	int rc = 0;
	u8 reg = 0;
	pr_err("%s, suspend= %d\n", __func__,enable);
	rc = smb1351_masked_write(chip, CMD_INPUT_LIMIT_REG,CMD_SUSPEND_MODE_BIT,
				enable ? CMD_SUSPEND_MODE_BIT : 0);
	if (rc)
		pr_err("Couldn't suspend rc = %d\n", rc);
	
	smb1351_read_reg(chip,CMD_INPUT_LIMIT_REG, &reg);
	pr_err("%s, read reg:0x%x\n",__func__,reg);
	return rc;
}
static int smbchg_charging_enable(struct smb1351_charger *chip, bool enable)
{
	int rc = 0;
	u8 reg = 0;
	pr_err("%s, enable= %d\n", __func__,enable);
	rc = smb1351_masked_write(chip, CMD_CHG_REG, CMD_CHG_EN_BIT,
					enable ? CMD_CHG_ENABLE : 0);
	if (rc)
		pr_err("Couldn't enable rc = %d\n", rc);

	smb1351_read_reg(chip,CMD_CHG_REG, &reg);
	pr_err("%s, read reg:0x%x\n",__func__,reg);
	
	return rc;
}
static int smb1351_check_charging_enable(void)
{
	int rc = 0;
	u8 reg = 0;
	bool charging_enable = false;

	if(atomic_read(&smb1351_chip->charger_suspended) == 1) {
		return 0;
	}
	rc = smb1351_read_reg(smb1351_chip, CMD_CHG_REG, &reg);
    if (rc) {
        chg_err("Couldn't read charging enable rc = %d\n", rc);
        return 0;
    }

    charging_enable = ((reg & CMD_CHG_EN_BIT) == CMD_CHG_EN_BIT) ? 1 : 0;
	chg_debug(" charging_enable:%d\n",charging_enable);

	return charging_enable;
	
}

static int smb1351_set_rechg_voltage(int recharge_mv)
{
	u8 reg = 0;
	int rc = 0;
	
	if (atomic_read(&smb1351_chip->charger_suspended) == 1) {
		return 0;
	}
	 
	
	/* set recharge voltage */
    if (recharge_mv >= 100) {
        reg = AUTO_RECHG_TH_100MV;
    } else {
        reg = AUTO_RECHG_TH_50MV;
    }
    rc = smb1351_masked_write(smb1351_chip, CHG_CTRL_REG, AUTO_RECHG_TH_BIT,reg);
    if (rc) {
        chg_err("Couldn't set recharging threshold rc = %d\n", rc);
    }

	rc = smb1351_masked_write(smb1351_chip, CHG_CTRL_REG, AUTO_RECHG_BIT,AUTO_RECHG_DISABLE);
	chg_debug(" hw rechg disable\n");

	return rc;
	
}

static int smb1351_set_chging_term_disable(void)
{
	int rc = 0;
	u8 reg =0;
	if (atomic_read(&smb1351_chip->charger_suspended) == 1) {
		return 0;
	}

	rc = smb1351_masked_write(smb1351_chip, CHG_CTRL_REG, ITERM_EN_BIT, ITERM_DISABLE);
	if (rc) {
        chg_err("Couldn't set chging_term disable rc = %d\n", rc);
    }

	smb1351_read_reg(smb1351_chip,CHG_CTRL_REG, &reg);
	pr_err("%s, read reg:0x%x\n",__func__,reg);
	
	return rc;
}

static bool smb1351_check_charger_resume(void)
{
	if (atomic_read(&smb1351_chip->charger_suspended) == 1) {
		return false;
	} else {
		return true;
	}
}



static int smb1351_fastchg_current_set(int fastchg_current)
{
	int i, rc;
	bool is_pre_chg = false;


	if ((fastchg_current < SMB1351_CHG_PRE_MIN_MA) ||
		(fastchg_current > SMB1351_CHG_FAST_MAX_MA)) {
		pr_err("bad pre_fastchg current mA=%d asked to set\n",
					fastchg_current);
		return -EINVAL;
	}

	/*
	 * fast chg current could not support less than 1000mA
	 * use pre chg to instead for the parallel charging
	 */
	if (fastchg_current < SMB1351_CHG_FAST_MIN_MA) {
		is_pre_chg = true;
		pr_err("is_pre_chg true, current is %d\n", fastchg_current);
	}

	if (is_pre_chg) {
		/* set prechg current */
		for (i = ARRAY_SIZE(pre_chg_current) - 1; i >= 0; i--) {
			if (pre_chg_current[i] <= fastchg_current)
				break;
		}
		if (i < 0)
			i = 0;
		smb1351_chip->fastchg_current_max_ma = pre_chg_current[i];
		pr_err("prechg setting %02x\n", i);

		i = i << SMB1351_CHG_PRE_SHIFT;

		rc = smb1351_masked_write(smb1351_chip, CHG_OTH_CURRENT_CTRL_REG,
				PRECHG_CURRENT_MASK, i);
		if (rc)
			pr_err("Couldn't write CHG_OTH_CURRENT_CTRL_REG rc=%d\n",
									rc);

		return smb1351_masked_write(smb1351_chip, VARIOUS_FUNC_2_REG,
				PRECHG_TO_FASTCHG_BIT, PRECHG_TO_FASTCHG_BIT);
	} else {
		if (smb1351_chip->version == SMB_UNKNOWN)
			return -EINVAL;

		/* SMB1350 supports FCC upto 2600 mA */
		if (smb1351_chip->version == SMB1350 && fastchg_current > 2600)
			fastchg_current = 2600;

		/* set fastchg current */
		for (i = ARRAY_SIZE(fast_chg_current) - 1; i >= 0; i--) {
			if (fast_chg_current[i] <= fastchg_current)
				break;
		}
		if (i < 0)
			i = 0;
		smb1351_chip->fastchg_current_max_ma = fast_chg_current[i];

		i = i << SMB1351_CHG_FAST_SHIFT;
		pr_err("fastchg limit=%d setting %02x\n",
					smb1351_chip->fastchg_current_max_ma, i);

		/* make sure pre chg mode is disabled */
		rc = smb1351_masked_write(smb1351_chip, VARIOUS_FUNC_2_REG,
					PRECHG_TO_FASTCHG_BIT, 0);
		if (rc)
			pr_err("Couldn't write VARIOUS_FUNC_2_REG rc=%d\n", rc);

		return smb1351_masked_write(smb1351_chip, CHG_CURRENT_CTRL_REG,
					FAST_CHG_CURRENT_MASK, i);
	}
}

#define MIN_FLOAT_MV		3500
#define MAX_FLOAT_MV		4500
#define VFLOAT_STEP_MV		20

static int smb1351_float_voltage_set(int vfloat_mv)
{
	u8 temp;

	if ((vfloat_mv < MIN_FLOAT_MV) || (vfloat_mv > MAX_FLOAT_MV)) {
		pr_err("bad float voltage mv =%d asked to set\n", vfloat_mv);
		return -EINVAL;
	}

	temp = (vfloat_mv - MIN_FLOAT_MV) / VFLOAT_STEP_MV;

	return smb1351_masked_write(smb1351_chip, VFLOAT_REG, VFLOAT_MASK, temp);
}

static int smb1351_iterm_set(int iterm_ma)
{
	int rc;
	u8 reg;

	if (iterm_ma <= 200)
		reg = CHG_ITERM_200MA;
	else if (iterm_ma <= 300)
		reg = CHG_ITERM_300MA;
	else if (iterm_ma <= 400)
		reg = CHG_ITERM_400MA;
	else if (iterm_ma <= 500)
		reg = CHG_ITERM_500MA;
	else if (iterm_ma <= 600)
		reg = CHG_ITERM_600MA;
	else
		reg = CHG_ITERM_700MA;

	rc = smb1351_masked_write(smb1351_chip, CHG_OTH_CURRENT_CTRL_REG,
				ITERM_MASK, reg);
	if (rc) {
		pr_err("Couldn't set iterm rc = %d\n", rc);
		return rc;
	}
	/* enable the iterm */
	rc = smb1351_masked_write(smb1351_chip, CHG_CTRL_REG,
				ITERM_EN_BIT, ITERM_ENABLE);
	if (rc) {
		pr_err("Couldn't enable iterm rc = %d\n", rc);
		return rc;
	}
	return 0;
}

static int smb1351_chg_otg_regulator_enable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct smb1351_charger *chip = rdev_get_drvdata(rdev);

	rc = smb1351_masked_write(chip, CMD_CHG_REG, CMD_OTG_EN_BIT,
							CMD_OTG_EN_BIT);
	if (rc)
		pr_err("Couldn't enable  OTG mode rc=%d\n", rc);
	pr_err("%s \n",__func__);
	return rc;
}

static int smb1351_chg_otg_regulator_disable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct smb1351_charger *chip = rdev_get_drvdata(rdev);

	rc = smb1351_masked_write(chip, CMD_CHG_REG, CMD_OTG_EN_BIT, 0);
	if (rc)
		pr_err("Couldn't disable OTG mode rc=%d\n", rc);
	pr_err("%s\n",__func__);
	return rc;
}

static int smb1351_chg_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	int rc = 0;
	u8 reg = 0;
	struct smb1351_charger *chip = rdev_get_drvdata(rdev);

	rc = smb1351_read_reg(chip, CMD_CHG_REG, &reg);
	if (rc) {
		pr_err("Couldn't read OTG enable bit rc=%d\n", rc);
		return rc;
	}

	return (reg & CMD_OTG_EN_BIT) ? 1 : 0;
}

static struct regulator_ops smb1351_chg_otg_reg_ops = {
	.enable		= smb1351_chg_otg_regulator_enable,
	.disable	= smb1351_chg_otg_regulator_disable,
	.is_enabled	= smb1351_chg_otg_regulator_is_enable,
};
/*
static int smb1351_regulator_init(struct smb1351_charger *chip)
{
	int rc = 0;
	struct regulator_config cfg = {};

	chip->otg_vreg.rdesc.owner = THIS_MODULE;
	chip->otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
	chip->otg_vreg.rdesc.ops = &smb1351_chg_otg_reg_ops;
	chip->otg_vreg.rdesc.name =
		chip->dev->of_node->name;
	chip->otg_vreg.rdesc.of_match =
		chip->dev->of_node->name;

	cfg.dev = chip->dev;
	cfg.driver_data = chip;

	chip->otg_vreg.rdev = regulator_register(
					&chip->sotg_vreg.rdesc, &cfg);
	if (IS_ERR(chip->otg_vreg.rdev)) {
		rc = PTR_ERR(chip->otg_vreg.rdev);
		chip->otg_vreg.rdev = NULL;
		if (rc != -EPROBE_DEFER)
			pr_err("OTG reg failed, rc=%d\n", rc);
	}
	return rc;
}
*/
static int smb_chip_get_version(void)
{
	u8 ver;
	int rc = 0;

	if (smb1351_chip->version == SMB_UNKNOWN) {
		rc = smb1351_read_reg(smb1351_chip, VERSION_REG, &ver);
		if (rc) {
			pr_err("Couldn't read version rc=%d\n", rc);
			return rc;
		}

		/* If bit 1 is set, it is SMB1350 */
		if (ver & VERSION_MASK)
			smb1351_chip->version = SMB1350;
		else
			smb1351_chip->version = SMB1351;
	}

	return rc;
}

static int smb1351_hw_init(void)
{
	int rc;
	u8 reg = 0, mask = 0;

	/* configure smb_pinctrl to enable irqs */
	if (smb1351_chip->pinctrl_state_name) {
		smb1351_chip->smb_pinctrl = pinctrl_get_select(smb1351_chip->dev,
						smb1351_chip->pinctrl_state_name);
		if (IS_ERR(smb1351_chip->smb_pinctrl)) {
			pr_err("Could not get/set %s pinctrl state rc = %ld\n",
						smb1351_chip->pinctrl_state_name,
						PTR_ERR(smb1351_chip->smb_pinctrl));
//			return PTR_ERR(smb1351_chip->smb1351.smb_pinctrl);
		}
	}


	/*
	 * If the charger is pre-configured for autonomous operation,
	 * do not apply additional settings
	 */
	if (smb1351_chip->chg_autonomous_mode) {
		pr_err("Charger configured for autonomous mode\n");
		return 0;
	}

	rc = smb_chip_get_version();
	if (rc) {
		pr_err("Couldn't get version rc = %d\n", rc);
		return rc;
	}

	rc = smb1351_enable_volatile_writes(smb1351_chip);
	if (rc) {
		pr_err("Couldn't configure volatile writes rc=%d\n", rc);
		return rc;
	}
	rc = smb1351_reset_charger();
	if (rc) {
		pr_err("Couldn't configure reg rc=%d\n", rc);
		return rc;
	}
	/* setup battery missing source */
	reg = BATT_MISSING_THERM_PIN_SOURCE_BIT;
	mask = BATT_MISSING_THERM_PIN_SOURCE_BIT;
	rc = smb1351_masked_write(smb1351_chip, HVDCP_BATT_MISSING_CTRL_REG,
								mask, reg);
	if (rc) {
		pr_err("Couldn't set HVDCP_BATT_MISSING_CTRL_REG rc=%d\n", rc);
		return rc;
	}

	/* disable hvdcp */
	reg = 0;
	mask = HVDCP_EN_BIT;
	rc = smb1351_masked_write(smb1351_chip, HVDCP_BATT_MISSING_CTRL_REG,
								mask, reg);
	if (rc) {
		pr_err("Couldn't disable hvdcp rc=%d\n", rc);
		return rc;
	}

	/*setup  FlexCharge+ 5v*/
	reg = 0;
	mask = ADAPTER_CONFIG_MASK;
	rc = smb1351_masked_write(smb1351_chip, OTG_MODE_POWER_OPTIONS_REG,
								mask, reg);
	if (rc) {
		pr_err("Couldn't set  adapter_config 0x14h , rc=%d\n", rc);
		return rc;
	}
	reg = 0;
	mask = CHG_CONFIG_MASK;
	rc = smb1351_masked_write(smb1351_chip, FLEXCHARGER_REG,
								mask, reg);
	if (rc) {
		pr_err("Couldn't set FlexCharge 5V rc=%d\n", rc);
		return rc;
	}
	

	/*setup dcd timetout time*/
	reg = DCD_DISABLE | TIMEOUT_SEL_330MS;
	mask = TIMEOUT_SEL_FOR_APSD_BIT | DCD_EN_BIT;
	rc = smb1351_masked_write(smb1351_chip, VARIOUS_FUNC_3_REG,
								mask, reg);
	if (rc) {
		pr_err("Couldn't SET DCD  rc=%d\n", rc);
		return rc;
	}
	
	/* setup defaults for CHG_PIN_EN_CTRL_REG */
	reg = EN_BY_I2C_0_DISABLE | USBCS_CTRL_BY_I2C | CHG_ERR_BIT |
		APSD_DONE_BIT | LED_BLINK_FUNC_BIT;
	mask = EN_PIN_CTRL_MASK | USBCS_CTRL_BIT | CHG_ERR_BIT |
		APSD_DONE_BIT | LED_BLINK_FUNC_BIT;
	rc = smb1351_masked_write(smb1351_chip, CHG_PIN_EN_CTRL_REG, mask, reg);
	if (rc) {
		pr_err("Couldn't set CHG_PIN_EN_CTRL_REG rc=%d\n", rc);
		return rc;
	}
	/* setup USB 2.0/3.0 detection and USB 500/100 command polarity */
	reg = USB_2_3_MODE_SEL_BY_I2C | USB_CMD_POLARITY_500_1_100_0;
	mask = USB_2_3_MODE_SEL_BIT | USB_5_1_CMD_POLARITY_BIT;
	rc = smb1351_masked_write(smb1351_chip, CHG_OTH_CURRENT_CTRL_REG, mask, reg);
	if (rc) {
		pr_err("Couldn't set CHG_OTH_CURRENT_CTRL_REG rc=%d\n", rc);
		return rc;
	}
	/* setup USB suspend, AICL and APSD  */
	reg = SUSPEND_MODE_CTRL_BY_I2C | AICL_EN_BIT;
	if (!smb1351_chip->disable_apsd)
		reg |= APSD_EN_BIT;
	mask = SUSPEND_MODE_CTRL_BIT | AICL_EN_BIT | APSD_EN_BIT;
	rc = smb1351_masked_write(smb1351_chip, VARIOUS_FUNC_REG, mask, reg);
	if (rc) {
		pr_err("Couldn't set VARIOUS_FUNC_REG rc=%d\n",	rc);
		return rc;
	}
	/* Fault and Status IRQ configuration */
	reg = HOT_COLD_HARD_LIMIT_BIT | HOT_COLD_SOFT_LIMIT_BIT
		| INPUT_OVLO_BIT | INPUT_UVLO_BIT | AICL_DONE_FAIL_BIT;
	rc = smb1351_write_reg(smb1351_chip, FAULT_INT_REG, reg);
	if (rc) {
		pr_err("Couldn't set FAULT_INT_REG rc=%d\n", rc);
		return rc;
	}
	reg = CHG_OR_PRECHG_TIMEOUT_BIT | BATT_OVP_BIT |
		FAST_TERM_TAPER_RECHG_INHIBIT_BIT |
		BATT_MISSING_BIT | BATT_LOW_BIT;
	rc = smb1351_write_reg(smb1351_chip, STATUS_INT_REG, reg);
	if (rc) {
		pr_err("Couldn't set STATUS_INT_REG rc=%d\n", rc);
		return rc;
	}
	/* setup THERM Monitor */
	if (!smb1351_chip->using_pmic_therm) {
		rc = smb1351_masked_write(smb1351_chip, THERM_A_CTRL_REG,
			THERM_MONITOR_BIT, THERM_MONITOR_EN);
		if (rc) {
			pr_err("Couldn't set THERM_A_CTRL_REG rc=%d\n",	rc);
			return rc;
		}
	}
	/* set the fast charge current limit */
	rc = smb1351_fastchg_current_set(2000);
	if (rc) {
		pr_err("Couldn't set fastchg current rc=%d\n", rc);
		return rc;
	}

	/* set the float voltage */
	if (smb1351_chip->vfloat_mv != -EINVAL) {
		rc = smb1351_float_voltage_set(smb1351_chip->vfloat_mv);
		if (rc) {
			pr_err("Couldn't set float voltage rc = %d\n", rc);
			return rc;
		}
	}

	/* disable AFVC*/
	reg = 0;
	mask = AFCV_MASK;
	rc = smb1351_masked_write(smb1351_chip, CHG_CTRL_REG,
								mask, reg);
	if (rc) {
		pr_err("Couldn't SET DCD  rc=%d\n", rc);
		return rc;
	}
	
#if 0
	/* set iterm */
	if (smb1351_chip->iterm_ma != -EINVAL) {
		if (smb1351_chip->iterm_disabled) {
			pr_err("Error: Both iterm_disabled and iterm_ma set\n");
			return -EINVAL;
		} else {
			rc = smb1351_iterm_set(smb1351_chip->iterm_ma);
			if (rc) {
				pr_err("Couldn't set iterm rc = %d\n", rc);
				return rc;
			}
		}
	} else  
#endif
	if (smb1351_chip->iterm_disabled) {
		rc = smb1351_masked_write(smb1351_chip, CHG_CTRL_REG,
					ITERM_EN_BIT, ITERM_DISABLE);
		if (rc) {
			pr_err("Couldn't set iterm rc = %d\n", rc);
			return rc;
		}
	}

	/* set recharge-threshold */
	if (smb1351_chip->recharge_mv != -EINVAL) {
		if (smb1351_chip->recharge_disabled) {
			pr_err("Error: Both recharge_disabled and recharge_mv set\n");
			return -EINVAL;
		} else {
			reg = AUTO_RECHG_ENABLE;
			if (smb1351_chip->recharge_mv > 50)
				reg |= AUTO_RECHG_TH_100MV;
			else
				reg |= AUTO_RECHG_TH_50MV;

			rc = smb1351_masked_write(smb1351_chip, CHG_CTRL_REG,
					AUTO_RECHG_BIT |
					AUTO_RECHG_TH_BIT, reg);
			if (rc) {
				pr_err("Couldn't set rechg-cfg rc = %d\n", rc);
				return rc;
			}
		}
	} else if (smb1351_chip->recharge_disabled) {
		rc = smb1351_masked_write(smb1351_chip, CHG_CTRL_REG,
				AUTO_RECHG_BIT,
				AUTO_RECHG_DISABLE);
		if (rc) {
			pr_err("Couldn't disable auto-rechg rc = %d\n", rc);
			return rc;
		}
	}

	/* enable/disable charging by suspending usb */
	rc = smbchg_charging_suspend(smb1351_chip,false);
	if (rc) {
		pr_err("Unable to %s battery charging. rc=%d\n",
			smb1351_chip->usb_suspended_status ? "disable" : "enable",
									rc);
	}

	return rc;

}

static int smb1351_suspend_charger(void)
{
	return smbchg_charging_suspend(smb1351_chip, true);
}

static int smb1351_unsuspend_charger(void)
{
	return smbchg_charging_suspend(smb1351_chip, false);
}
static int smb1351_enable_charging(void)
{
	return smbchg_charging_enable(smb1351_chip, true);
}

static int smb1351_disable_charging(void)
{
	return smbchg_charging_enable(smb1351_chip, false);
}
static int smb1351_usbin_input_current_limit[] = {
    100,    150,    500,    900,
    1200,   1500,   2000,   3000,
};
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

static int smb1351_input_current_limit_write(struct smb1351_charger *chip, int value)
{
		int rc = 0;
		int i = 0;
		int j = 0;
		int chg_vol = 0;
		int aicl_point_temp = 0;
		int chg_vol_all[10] = {0};
		if(smb1351_registers_read_full()) {
			smb1351_masked_write(chip, CHG_CURRENT_CTRL_REG, AC_INPUT_CURRENT_LIMIT_MASK, REG00_SMB1351_INPUT_CURRENT_LIMIT_500MA);
			smb1351_dump_regs();
			printk("smb1351_masked_write_input_current_limit_write\n");
			return 0;
		}
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
		rc = smb1351_masked_write(chip, CHG_CURRENT_CTRL_REG, AC_INPUT_CURRENT_LIMIT_MASK, REG00_SMB1351_INPUT_CURRENT_LIMIT_500MA);
		msleep(90);
		chg_vol = battery_meter_get_charger_voltage();
		if (chg_vol < aicl_point_temp) {
			j = 2;
			goto aicl_pre_step;
		} else if (value < 900)
			goto aicl_end;
	
		j = 3; /* 900 */
		rc = smb1351_masked_write(chip, CHG_CURRENT_CTRL_REG, AC_INPUT_CURRENT_LIMIT_MASK, REG00_SMB1351_INPUT_CURRENT_LIMIT_900MA);
		msleep(90);
		chg_vol = battery_meter_get_charger_voltage();
		if (chg_vol < aicl_point_temp) {
			j = j - 1;
			goto aicl_pre_step;
		} else if (value < 1200)
			goto aicl_end;
	
		j = 5; /* 1500 */
	#if defined(CONFIG_OPLUS_CHARGER_MTK6763)  || defined(CONFIG_OPLUS_CHARGER_MTK6771)
		aicl_point_temp = chip->sw_aicl_point;
	#else
		aicl_point_temp = chip->sw_aicl_point;
	#endif
		rc = smb1351_masked_write(chip, CHG_CURRENT_CTRL_REG, AC_INPUT_CURRENT_LIMIT_MASK, REG00_SMB1351_INPUT_CURRENT_LIMIT_1500MA);
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
		aicl_point_temp = chip->sw_aicl_point;
	#else
		aicl_point_temp = chip->sw_aicl_point;
	#endif
		rc = smb1351_masked_write(chip, CHG_CURRENT_CTRL_REG, AC_INPUT_CURRENT_LIMIT_MASK, REG00_SMB1351_INPUT_CURRENT_LIMIT_2000MA);
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
		rc = smb1351_masked_write(chip, CHG_CURRENT_CTRL_REG, AC_INPUT_CURRENT_LIMIT_MASK, REG00_SMB1351_INPUT_CURRENT_LIMIT_2200MA);
		msleep(90);
		chg_vol = battery_meter_get_charger_voltage();
		if (chg_vol < aicl_point_temp) {
			j = j - 1;
			goto aicl_pre_step;
		} else if (value >= 3000)
			goto aicl_end;
	
	aicl_pre_step:
		if((j >= 2) && (j <= ARRAY_SIZE(smb1351_usbin_input_current_limit) - 1))
			aicl_result = smb1351_usbin_input_current_limit[j]; 	
		chg_debug( "usb input max current limit aicl chg_vol=%d j[%d]=%d sw_aicl_point:%d aicl_pre_step\n", chg_vol, j, smb1351_usbin_input_current_limit[j], aicl_point_temp);
		switch (j) {
			case 0:
				rc = smb1351_masked_write(chip, CHG_CURRENT_CTRL_REG, AC_INPUT_CURRENT_LIMIT_MASK, REG00_SMB1351_INPUT_CURRENT_LIMIT_500MA);
			break;
			case 1:
				rc = smb1351_masked_write(chip, CHG_CURRENT_CTRL_REG, AC_INPUT_CURRENT_LIMIT_MASK, REG00_SMB1351_INPUT_CURRENT_LIMIT_500MA);
			break;
			case 2:
				rc = smb1351_masked_write(chip, CHG_CURRENT_CTRL_REG, AC_INPUT_CURRENT_LIMIT_MASK, REG00_SMB1351_INPUT_CURRENT_LIMIT_500MA);
			break;
			case 3:
				rc = smb1351_masked_write(chip, CHG_CURRENT_CTRL_REG, AC_INPUT_CURRENT_LIMIT_MASK, REG00_SMB1351_INPUT_CURRENT_LIMIT_900MA);
			break;
			case 4:
				rc = smb1351_masked_write(chip, CHG_CURRENT_CTRL_REG, AC_INPUT_CURRENT_LIMIT_MASK, REG00_SMB1351_INPUT_CURRENT_LIMIT_1200MA);
			break;
			case 5:
				rc = smb1351_masked_write(chip, CHG_CURRENT_CTRL_REG, AC_INPUT_CURRENT_LIMIT_MASK, REG00_SMB1351_INPUT_CURRENT_LIMIT_1500MA);
			break;
			case 6:
				rc = smb1351_masked_write(chip, CHG_CURRENT_CTRL_REG, AC_INPUT_CURRENT_LIMIT_MASK, REG00_SMB1351_INPUT_CURRENT_LIMIT_2000MA);
			break;
			case 7:
				rc = smb1351_masked_write(chip, CHG_CURRENT_CTRL_REG, AC_INPUT_CURRENT_LIMIT_MASK, REG00_SMB1351_INPUT_CURRENT_LIMIT_2200MA);
			break;
			default:
				break;

		}
		smb1351_masked_write(chip, CHG_PIN_EN_CTRL_REG, USBCS_CTRL_BIT, USBCS_CTRL_BIT);
		smb1351_masked_write(chip, CMD_INPUT_LIMIT_REG, CMD_INPUT_CURRENT_MODE_BIT, CMD_INPUT_CURRENT_MODE_BIT);
		mdelay(30);
		smb1351_masked_write(chip, CMD_INPUT_LIMIT_REG, CMD_USB_AC_MODE_MASK, CMD_USB_AC_MODE);
		mdelay(30);
		smb1351_masked_write(chip, CHG_PIN_EN_CTRL_REG, USBCS_CTRL_BIT, 0x0);
	//	smb1351_masked_write(chip, CMD_INPUT_LIMIT_REG, CMD_INPUT_CURRENT_MODE_BIT, 0x0);
	//	smb1351_masked_write(chip, CMD_INPUT_LIMIT_REG, CMD_USB_AC_MODE_MASK, 0x0);
		
		return rc;
	aicl_end:
		if((j >= 2) && (j <= ARRAY_SIZE(smb1351_usbin_input_current_limit) - 1))
			aicl_result = smb1351_usbin_input_current_limit[j]; 	
		chg_debug( "usb input max current limit aicl chg_vol=%d j[%d]=%d sw_aicl_point:%d aicl_end\n", chg_vol, j, smb1351_usbin_input_current_limit[j], aicl_point_temp);
		switch (j) {
			case 0:
				rc = smb1351_masked_write(chip, CHG_CURRENT_CTRL_REG, AC_INPUT_CURRENT_LIMIT_MASK, REG00_SMB1351_INPUT_CURRENT_LIMIT_500MA);
			break;
			case 1:
				rc = smb1351_masked_write(chip, CHG_CURRENT_CTRL_REG, AC_INPUT_CURRENT_LIMIT_MASK, REG00_SMB1351_INPUT_CURRENT_LIMIT_500MA);
			break;
			case 2:
				rc = smb1351_masked_write(chip, CHG_CURRENT_CTRL_REG, AC_INPUT_CURRENT_LIMIT_MASK, REG00_SMB1351_INPUT_CURRENT_LIMIT_500MA);
			break;
			case 3:
				rc = smb1351_masked_write(chip, CHG_CURRENT_CTRL_REG, AC_INPUT_CURRENT_LIMIT_MASK, REG00_SMB1351_INPUT_CURRENT_LIMIT_900MA);
			break;
			case 4:
				rc = smb1351_masked_write(chip, CHG_CURRENT_CTRL_REG, AC_INPUT_CURRENT_LIMIT_MASK, REG00_SMB1351_INPUT_CURRENT_LIMIT_1200MA);
			break;
			case 5:
				rc = smb1351_masked_write(chip, CHG_CURRENT_CTRL_REG, AC_INPUT_CURRENT_LIMIT_MASK, REG00_SMB1351_INPUT_CURRENT_LIMIT_1500MA);
			break;
			case 6:
				rc = smb1351_masked_write(chip, CHG_CURRENT_CTRL_REG, AC_INPUT_CURRENT_LIMIT_MASK, REG00_SMB1351_INPUT_CURRENT_LIMIT_2000MA);
			break;
			case 7:
				rc = smb1351_masked_write(chip, CHG_CURRENT_CTRL_REG, AC_INPUT_CURRENT_LIMIT_MASK, REG00_SMB1351_INPUT_CURRENT_LIMIT_2200MA);
			break;
			default:
				break;
		}
		smb1351_masked_write(chip, CHG_PIN_EN_CTRL_REG, USBCS_CTRL_BIT, USBCS_CTRL_BIT);
		smb1351_masked_write(chip, CMD_INPUT_LIMIT_REG, CMD_INPUT_CURRENT_MODE_BIT, CMD_INPUT_CURRENT_MODE_BIT);
		mdelay(30);
		smb1351_masked_write(chip, CMD_INPUT_LIMIT_REG, CMD_USB_AC_MODE_MASK, CMD_USB_AC_MODE);
		mdelay(30);
		smb1351_masked_write(chip, CHG_PIN_EN_CTRL_REG, USBCS_CTRL_BIT, 0x0);
	//	smb1351_masked_write(chip, CMD_INPUT_LIMIT_REG, CMD_INPUT_CURRENT_MODE_BIT, 0x0);
	//	smb1351_masked_write(chip, CMD_INPUT_LIMIT_REG, CMD_USB_AC_MODE_MASK, 0x0);
		smb1351_dump_regs();
		return rc;

}
int smb1351_get_charger_type(void);
static int smb1351_set_usb_chg_current(int current_ma)
{
	int i, rc = 0;
	u8 reg = 0, mask = 0;

	pr_err("USB current_ma = %d\n", current_ma);

	if (smb1351_chip->chg_autonomous_mode) {
		pr_err("Charger in autonomous mode\n");
		return 0;
	}

	/* set suspend bit when urrent_ma <= 2 */
	if (current_ma <= SUSPEND_CURRENT_MA) {
		smb1351_suspend_charger();
		pr_err("USB suspend\n");
		return 0;
	}
	switch(smb1351_get_charger_type()) {
	//case POWER_SUPPLY_TYPE_USB:
    case STANDARD_HOST:
		if (current_ma > SUSPEND_CURRENT_MA &&
				current_ma < USB2_MIN_CURRENT_MA)
			current_ma = USB2_MIN_CURRENT_MA;

		if (current_ma == USB2_MIN_CURRENT_MA) {
			/* USB 2.0 - 100mA */
			reg = CMD_USB_2_MODE | CMD_USB_100_MODE;
		} else if (current_ma == USB3_MIN_CURRENT_MA) {
			/* USB 3.0 - 150mA */
			reg = CMD_USB_3_MODE | CMD_USB_100_MODE;
		} else if (current_ma == USB2_MAX_CURRENT_MA) {
			/* USB 2.0 - 500mA */
			reg = CMD_USB_2_MODE | CMD_USB_500_MODE;
		} 
		/* control input current mode by command */
		reg |= CMD_INPUT_CURRENT_MODE_CMD;
		mask = CMD_INPUT_CURRENT_MODE_BIT | CMD_USB_2_3_SEL_BIT |
			CMD_USB_1_5_AC_CTRL_MASK;
		rc = smb1351_masked_write(smb1351_chip, CMD_INPUT_LIMIT_REG, mask, reg);
		if (rc) {
			pr_err("Couldn't set charging mode rc = %d\n", rc);
			return rc;
		}

		if (current_ma > USB2_MAX_CURRENT_MA) {
			rc = smb1351_input_current_limit_write(smb1351_chip,current_ma);
		}
		if (rc < 0)
			pr_err("Couldn't set %dmA rc = %d\n", current_ma, rc);
		break;

	default:
		rc = smb1351_input_current_limit_write(smb1351_chip, current_ma);
		if (rc < 0)
			pr_err("Couldn't set %dmA rc = %d\n", current_ma, rc);
		break;


	}


	return rc;
}
#if 0

static int smb1351_get_closest_usb_setpoint(int val)
{
	int i;

	for (i = ARRAY_SIZE(usb_chg_current) - 1; i >= 0; i--) {
		if (usb_chg_current[i] <= val)
			break;
	}
	if (i < 0)
		i = 0;

	if (i >= ARRAY_SIZE(usb_chg_current) - 1)
		return ARRAY_SIZE(usb_chg_current) - 1;

	/* check what is closer, i or i + 1 */
	if (abs(usb_chg_current[i] - val) < abs(usb_chg_current[i + 1] - val))
		return i;
	else
		return i + 1;
}

static bool smb1351_is_input_current_limited(struct smb1351_charger *chip)
{
	int rc;
	u8 reg;

	rc = smb1351_read_reg(chip, IRQ_H_REG, &reg);
	if (rc) {
		pr_err("Failed to read IRQ_H_REG for ICL status: %d\n", rc);
		return false;
	}

	return !!(reg & IRQ_IC_LIMIT_STATUS_BIT);
}
#endif

static int rerun_apsd(struct smb1351_charger *chip)
{
	int rc;

	pr_err("Reruning APSD\n\n");

	rc = smb1351_masked_write(chip, CMD_HVDCP_REG, CMD_APSD_RE_RUN_BIT,
						CMD_APSD_RE_RUN_BIT);
	if (rc)
		pr_err("Couldn't re-run APSD algo\n");

	return 0;
}


static int smb1351_apsd_complete_handler(struct smb1351_charger *chip,
						u8 status)
{
	int rc;
	u8 reg = 0;
	pr_err("%s, status:%d\n",__func__,status);
	rc = smb1351_read_reg(chip, STATUS_5_REG, &reg);
	if (rc) {
		pr_err("Couldn't read STATUS_5 rc = %d\n", rc);
		return rc;
	}

	pr_err("%s, STATUS_5_REG(0x3B)=%x\n", __func__,reg);
	return 0;
	
}
extern void Charger_Detect_Init(void);
extern void Charger_Detect_Release(void);
extern bool is_usb_rdy(void);
static void hw_bc12_init(void)
{
	int timeout = 40;
	static bool first_connect = true;
	if (first_connect == true) {
		/* add make sure USB Ready */
		if (is_usb_rdy() == false) {
			pr_err("CDP, block\n");
			while (is_usb_rdy() == false && timeout > 0) {
				msleep(100);
				timeout--;
			}
			if (timeout == 0)
				pr_err("CDP, timeout\n");
			else {
				pr_err("CDP, free, timeout:%d\n", timeout);
			}
		} else
			pr_err("CDP, PASS\n");
		first_connect = false;
	}
	Charger_Detect_Init();
}

int smb1351_get_charger_type(void)
{
	int rc;
	u8 reg = 0;
	int type = 0;
	int count = 0;
	if (!smb1351_chip) {
		pr_err("%s smb1351_chip null,return\n", __func__);
		return CHARGER_UNKNOWN;
	}
	hw_bc12_init();
	mdelay(40);
	printk("mt_charger_type_detection1\n");
	rerun_apsd(smb1351_chip);
	
	mdelay(800);
	rc = smb1351_read_reg(smb1351_chip, IRQ_G_REG, &reg);
	while(!(reg & IRQ_SOURCE_DET_BIT) && count < 40){
		count++;
		rc = smb1351_read_reg(smb1351_chip, IRQ_G_REG, &reg);
		mdelay(50);
	}
	if(count == 40) {
		type = APPLE_2_1A_CHARGER;
		chg_debug(" call resun apsd timeout\n");
		if(!pmic_chrdet_status()) {
			type = CHARGER_UNKNOWN;
			chg_debug(" already plug out\n");
		}
		return type;
	}

	rc = smb1351_read_reg(smb1351_chip, STATUS_6_REG, &reg);
	if (rc) {
		pr_err("Couldn't read STATUS_6 rc = %d\n", rc);
	}
	pr_err("dcd result(0x3C)=%x\n", reg);
	
	rc = smb1351_read_reg(smb1351_chip, STATUS_5_REG, &reg);
	if (rc) {
		pr_err("Couldn't read STATUS_5 rc = %d\n", rc);
	}

	pr_err("apsd result(0x3B)=%x, count:%d\n", reg,count);

	switch (reg) {
	case STATUS_PORT_ACA_DOCK:
	case STATUS_PORT_ACA_C:
	case STATUS_PORT_ACA_B:
	case STATUS_PORT_ACA_A:
		type = NONSTANDARD_CHARGER;
		break;
	case STATUS_PORT_CDP:
		type = CHARGING_HOST;
		break;
	case STATUS_PORT_DCP:
		type = STANDARD_CHARGER;
		break;
	case STATUS_PORT_SDP:
		type = STANDARD_HOST;
		break;
	case STATUS_PORT_OTHER:
		type = NONSTANDARD_CHARGER;
		break;
	default:
		type = CHARGER_UNKNOWN;
		break;
	}
	Charger_Detect_Release();
	return type;
}


/*
 * As source detect interrupt is not triggered on the falling edge,
 * we need to schedule a work for checking source detect status after
 * charger UV interrupt fired.
 */
#define FIRST_CHECK_DELAY	100
#define SECOND_CHECK_DELAY	1000
/*
static void smb1351_chg_remove_work(struct work_struct *work)
{
	int rc;
	u8 reg;
	struct smb1351_charger *pmic_chip = container_of(work,struct smb1351_charger, chg_remove_work.work);
	struct smb1351_charger *chip = container_of(pmic_chip,struct smb1351_charger,smb1351);

	rc = smb1351_read_reg(chip, IRQ_G_REG, &reg);
	if (rc) {
		pr_err("Couldn't read IRQ_G_REG rc = %d\n", rc);
		goto end;
	}

	if (!(reg & IRQ_SOURCE_DET_BIT)) {
		pr_err("chg removed\n");
		smb1351_apsd_complete_handler(chip, 0);
	} else if (!chip->chg_remove_work_scheduled) {
		chip->chg_remove_work_scheduled = true;
		goto reschedule;
	} else {
		pr_err("charger is present\n");
	}
end:
	chip->chg_remove_work_scheduled = false;
	pm_relax(chip->dev);
	return;

reschedule:
	pr_err("reschedule after 1s\n");
	schedule_delayed_work(&chip->chg_remove_work,
				msecs_to_jiffies(SECOND_CHECK_DELAY));
}
*/
static int smb1351_usbin_uv_handler(struct smb1351_charger *chip, u8 status)
{
	pr_err("%s, status:%d\n",__func__,status);

	return 0;
}

static int smb1351_usbin_ov_handler(struct smb1351_charger *chip, u8 status)
{
	int rc;
	u8 reg;
	union power_supply_propval pval = {0, };

	rc = smb1351_read_reg(chip, IRQ_E_REG, &reg);
	if (rc)
		pr_err("Couldn't read IRQ_E rc = %d\n", rc);
	pr_err("%s, status:%d\n",__func__,status);

	return 0;
}

static int smb1351_fast_chg_handler(struct smb1351_charger *chip, u8 status)
{
	pr_err("enter\n");
	return 0;
}

static int smb1351_chg_term_handler(struct smb1351_charger *chip, u8 status)
{
	pr_err("enter\n");
	if (!chip->bms_controlled_charging)
		chip->batt_full = !!status;
	return 0;
}

static int smb1351_safety_timeout_handler(struct smb1351_charger *chip,
						u8 status)
{
	pr_err("safety_timeout triggered\n");
	return 0;
}

static int smb1351_aicl_done_handler(struct smb1351_charger *chip, u8 status)
{
	pr_err("aicl_done triggered\n");
	return 0;
}

static int smb1351_hot_hard_handler(struct smb1351_charger *chip, u8 status)
{
	pr_err("status = 0x%02x\n", status);
	chip->batt_hot = !!status;
	return 0;
}
static int smb1351_cold_hard_handler(struct smb1351_charger *chip, u8 status)
{
	pr_err("status = 0x%02x\n", status);
	chip->batt_cold = !!status;
	return 0;
}
static int smb1351_hot_soft_handler(struct smb1351_charger *chip, u8 status)
{
	pr_err("status = 0x%02x\n", status);
	chip->batt_warm = !!status;
	return 0;
}
static int smb1351_cold_soft_handler(struct smb1351_charger *chip, u8 status)
{
	pr_err("status = 0x%02x\n", status);
	chip->batt_cool = !!status;
	return 0;
}

static int smb1351_battery_missing_handler(struct smb1351_charger *chip,
						u8 status)
{
	if (status)
		chip->battery_missing = true;
	else
		chip->battery_missing = false;

	return 0;
}
#ifdef CONFIG_OPLUS_CHARGER_MTK6771
struct smb_irq_info {
	const char		*name;
	int (*smb_irq)(struct smb1351_charger *chip, u8 rt_stat);
	int			high;
	int			low;
};

struct irq_handler_info {
		u8			stat_reg;
		u8			val;
		u8			prev_val;
		struct smb_irq_info	irq_info[4];
};
#endif
static struct irq_handler_info handlers[] = {
	[0] = {
		.stat_reg	= IRQ_A_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{	.name	 = "cold_soft",
				.smb_irq = smb1351_cold_soft_handler,
			},
			{	.name	 = "hot_soft",
				.smb_irq = smb1351_hot_soft_handler,
			},
			{	.name	 = "cold_hard",
				.smb_irq = smb1351_cold_hard_handler,
			},
			{	.name	 = "hot_hard",
				.smb_irq = smb1351_hot_hard_handler,
			},
		},
	},
	[1] = {
		.stat_reg	= IRQ_B_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{	.name	 = "internal_temp_limit",
			},
			{	.name	 = "vbatt_low",
			},
			{	.name	 = "battery_missing",
				.smb_irq = smb1351_battery_missing_handler,
			},
			{	.name	 = "batt_therm_removed",
			},
		},
	},
	[2] = {
		.stat_reg	= IRQ_C_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{	.name	 = "chg_term",
				.smb_irq = smb1351_chg_term_handler,
			},
			{	.name	 = "taper",
			},
			{	.name	 = "recharge",
			},
			{	.name	 = "fast_chg",
				.smb_irq = smb1351_fast_chg_handler,
			},
		},
	},
	[3] = {
		.stat_reg	= IRQ_D_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{	.name	 = "prechg_timeout",
			},
			{	.name	 = "safety_timeout",
				.smb_irq = smb1351_safety_timeout_handler,
			},
			{	.name	 = "chg_error",
			},
			{	.name	 = "batt_ov",
			},
		},
	},
	[4] = {
		.stat_reg	= IRQ_E_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{	.name	 = "power_ok",
			},
			{	.name	 = "afvc",
			},
			{	.name	 = "usbin_uv",
				.smb_irq = smb1351_usbin_uv_handler,
			},
			{	.name	 = "usbin_ov",
				.smb_irq = smb1351_usbin_ov_handler,
			},
		},
	},
	[5] = {
		.stat_reg	= IRQ_F_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{	.name	 = "otg_oc_retry",
			},
			{	.name	 = "rid",
			},
			{	.name	 = "otg_fail",
			},
			{	.name	 = "otg_oc",
			},
		},
	},
	[6] = {
		.stat_reg	= IRQ_G_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{	.name	 = "chg_inhibit",
			},
			{	.name	 = "aicl_fail",
			},
			{	.name	 = "aicl_done",
				.smb_irq = smb1351_aicl_done_handler,
			},
			{	.name	 = "apsd_complete",
				.smb_irq = smb1351_apsd_complete_handler,
			},
		},
	},
	[7] = {
		.stat_reg	= IRQ_H_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{	.name	 = "wdog_timeout",
			},
			{	.name	 = "hvdcp_auth_done",
			},
		},
	},
};

#define IRQ_LATCHED_MASK	0x02
#define IRQ_STATUS_MASK		0x01
#define BITS_PER_IRQ		2
static irqreturn_t smb1351_chg_stat_handler(int irq, void *dev_id)
{
	struct smb1351_charger *chip = dev_id;
	int i, j;
	u8 triggered;
	u8 changed;
	u8 rt_stat, prev_rt_stat;
	int rc;
	int handler_count = 0;

	mutex_lock(&chip->irq_complete);

	chip->irq_waiting = true;
	if (!chip->resume_completed) {
		pr_err("IRQ triggered before device-resume\n");
		disable_irq_nosync(irq);
		mutex_unlock(&chip->irq_complete);
		return IRQ_HANDLED;
	}
	chip->irq_waiting = false;

	for (i = 0; i < ARRAY_SIZE(handlers); i++) {
		rc = smb1351_read_reg(chip, handlers[i].stat_reg,
						&handlers[i].val);
		if (rc) {
			pr_err("Couldn't read %d rc = %d\n",
					handlers[i].stat_reg, rc);
			continue;
		}

		for (j = 0; j < ARRAY_SIZE(handlers[i].irq_info); j++) {
			triggered = handlers[i].val
			       & (IRQ_LATCHED_MASK << (j * BITS_PER_IRQ));
			rt_stat = handlers[i].val
				& (IRQ_STATUS_MASK << (j * BITS_PER_IRQ));
			prev_rt_stat = handlers[i].prev_val
				& (IRQ_STATUS_MASK << (j * BITS_PER_IRQ));
			changed = prev_rt_stat ^ rt_stat;

			if (triggered || changed)
				rt_stat ? handlers[i].irq_info[j].high++ :
						handlers[i].irq_info[j].low++;

			if ((triggered || changed)
				&& handlers[i].irq_info[j].smb_irq != NULL) {
				handler_count++;
				rc = handlers[i].irq_info[j].smb_irq(chip,
								rt_stat);
				if (rc)
					pr_err("Couldn't handle %d irq for reg 0x%02x rc = %d\n",
						j, handlers[i].stat_reg, rc);
			}
		}
		handlers[i].prev_val = handlers[i].val;
	}

	pr_err("handler count = %d\n", handler_count);
	if (handler_count) {
		pr_err("batt psy changed\n");
	}

	mutex_unlock(&chip->irq_complete);

	return IRQ_HANDLED;
}

#define LAST_CNFG_REG	0x16
static int show_cnfg_regs(struct seq_file *m, void *data)
{
	struct smb1351_charger *chip = m->private;
	int rc;
	u8 reg;
	u8 addr;

	for (addr = 0; addr <= LAST_CNFG_REG; addr++) {
		rc = smb1351_read_reg(chip, addr, &reg);
		if (!rc)
			seq_printf(m, "0x%02x = 0x%02x\n", addr, reg);
	}

	return 0;
}

static int cnfg_debugfs_open(struct inode *inode, struct file *file)
{
	struct smb1351_charger *chip = inode->i_private;

	return single_open(file, show_cnfg_regs, chip);
}

static const struct file_operations cnfg_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= cnfg_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#define FIRST_CMD_REG	0x30
#define LAST_CMD_REG	0x34
static int show_cmd_regs(struct seq_file *m, void *data)
{
	struct smb1351_charger *chip = m->private;
	int rc;
	u8 reg;
	u8 addr;

	for (addr = FIRST_CMD_REG; addr <= LAST_CMD_REG; addr++) {
		rc = smb1351_read_reg(chip, addr, &reg);
		if (!rc)
			seq_printf(m, "0x%02x = 0x%02x\n", addr, reg);
	}

	return 0;
}

static int cmd_debugfs_open(struct inode *inode, struct file *file)
{
	struct smb1351_charger *chip = inode->i_private;

	return single_open(file, show_cmd_regs, chip);
}

static const struct file_operations cmd_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= cmd_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#define FIRST_STATUS_REG	0x36
#define LAST_STATUS_REG		0x3F
static int show_status_regs(struct seq_file *m, void *data)
{
	struct smb1351_charger *chip = m->private;
	int rc;
	u8 reg;
	u8 addr;

	for (addr = FIRST_STATUS_REG; addr <= LAST_STATUS_REG; addr++) {
		rc = smb1351_read_reg(chip, addr, &reg);
		if (!rc)
			seq_printf(m, "0x%02x = 0x%02x\n", addr, reg);
	}

	return 0;
}

static int status_debugfs_open(struct inode *inode, struct file *file)
{
	struct smb1351_charger *chip = inode->i_private;

	return single_open(file, show_status_regs, chip);
}

static const struct file_operations status_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= status_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int show_irq_count(struct seq_file *m, void *data)
{
	int i, j, total = 0;

	for (i = 0; i < ARRAY_SIZE(handlers); i++)
		for (j = 0; j < 4; j++) {
			seq_printf(m, "%s=%d\t(high=%d low=%d)\n",
						handlers[i].irq_info[j].name,
						handlers[i].irq_info[j].high
						+ handlers[i].irq_info[j].low,
						handlers[i].irq_info[j].high,
						handlers[i].irq_info[j].low);
			total += (handlers[i].irq_info[j].high
					+ handlers[i].irq_info[j].low);
		}

	seq_printf(m, "\n\tTotal = %d\n", total);

	return 0;
}

static int irq_count_debugfs_open(struct inode *inode, struct file *file)
{
	struct smb1351_charger *chip = inode->i_private;

	return single_open(file, show_irq_count, chip);
}

static const struct file_operations irq_count_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= irq_count_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int get_reg(void *data, u64 *val)
{
	struct smb1351_charger *chip = data;
	int rc;
	u8 temp;

	rc = smb1351_read_reg(chip, chip->peek_poke_address, &temp);
	if (rc) {
		pr_err("Couldn't read reg %x rc = %d\n",
			chip->peek_poke_address, rc);
		return -EAGAIN;
	}
	*val = temp;
	return 0;
}

static int set_reg(void *data, u64 val)
{
	struct smb1351_charger *chip = data;
	int rc;
	u8 temp;

	temp = (u8) val;
	rc = smb1351_write_reg(chip, chip->peek_poke_address, temp);
	if (rc) {
		pr_err("Couldn't write 0x%02x to 0x%02x rc= %d\n",
			temp, chip->peek_poke_address, rc);
		return -EAGAIN;
	}
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(poke_poke_debug_ops, get_reg, set_reg, "0x%02llx\n");

static int force_irq_set(void *data, u64 val)
{
	struct smb1351_charger *chip = data;

	smb1351_chg_stat_handler(chip->client->irq, data);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(force_irq_ops, NULL, force_irq_set, "0x%02llx\n");

#if 1
#define DUMP_REG_LEN 64
void smb1351_dump_regs(void)
{
	int rc;
	u8 reg;
	u8 addr;
	u8 dump_reg[DUMP_REG_LEN] = {0x0};

	for (addr = 0; addr <= LAST_CNFG_REG; addr++) {
		rc = smb1351_read_reg(smb1351_chip, addr, &reg);
		if (rc){
			pr_err("Couldn't read 0x%02x rc = %d\n", addr, rc);
			return ;
		}
		else {
			dump_reg[addr] = reg;
		}
			
	}
	pr_err("[%s]:smb1351_cnfg_reg[0x00-0x16]:0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n",__func__,
		dump_reg[0],dump_reg[1], dump_reg[2], dump_reg[3], dump_reg[4], dump_reg[5], dump_reg[6],
		dump_reg[7], dump_reg[8], dump_reg[9], dump_reg[10], dump_reg[11], dump_reg[12], dump_reg[13], dump_reg[14], dump_reg[15],
		dump_reg[16], dump_reg[17], dump_reg[18], dump_reg[19], dump_reg[20], dump_reg[21], dump_reg[22]);

	
	for (addr = FIRST_STATUS_REG; addr <= LAST_STATUS_REG; addr++) {
		rc = smb1351_read_reg(smb1351_chip, addr, &reg);
		if (rc)
			pr_err("Couldn't read 0x%02x rc = %d\n", addr, rc);
		else {
			dump_reg[addr-FIRST_STATUS_REG] = reg;
		}
	}
	pr_err("[%s]:smb1351_status_reg[0x36-0x3f]:0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n",__func__,
		dump_reg[0], dump_reg[1], dump_reg[2], dump_reg[3], dump_reg[4], dump_reg[5], dump_reg[6], dump_reg[7], dump_reg[8],
		dump_reg[9]);

	for (addr = FIRST_CMD_REG; addr <= LAST_CMD_REG; addr++) {
		rc = smb1351_read_reg(smb1351_chip, addr, &reg);
		if (rc)
			pr_err("Couldn't read 0x%02x rc = %d\n", addr, rc);
		else {
			dump_reg[addr-FIRST_CMD_REG] = reg;
		}
	}
	pr_err("[%s]:smb1351_cmd_reg[0x30-0x34]:0x%x,0x%x,0x%x,0x%x,0x%x\n",__func__,dump_reg[0], dump_reg[1], dump_reg[2], dump_reg[3], dump_reg[4]);
    charger_ic_flag = 3;
	return ;
}
#else
static void dump_regs(struct smb1351_charger *chip)
{
}
#endif

static int smb1351_parse_dt(struct smb1351_charger *chip)
{
	int rc;
	struct device_node *node = chip->dev->of_node;

	if (!node) {
		pr_err("device tree info. missing\n");
		return -EINVAL;
	}


	chip->disable_apsd = of_property_read_bool(node, "qcom,disable-apsd");

	chip->using_pmic_therm = of_property_read_bool(node,
						"qcom,using-pmic-therm");
	chip->bms_controlled_charging  = of_property_read_bool(node,
					"qcom,bms-controlled-charging");
	chip->force_hvdcp_2p0 = of_property_read_bool(node,
					"qcom,force-hvdcp-2p0");

	rc = of_property_read_string(node, "qcom,bms-psy-name",
						&chip->bms_psy_name);
	if (rc)
		chip->bms_psy_name = NULL;

	rc = of_property_read_u32(node, "qcom,fastchg-current-max-ma",
					&chip->target_fastchg_current_max_ma);
	if (rc)
		chip->target_fastchg_current_max_ma = SMB1351_CHG_FAST_MAX_MA;

	chip->iterm_disabled = of_property_read_bool(node,
					"qcom,iterm-disabled");



	rc = of_property_read_u32(node, "qcom,float-voltage-mv",
						&chip->vfloat_mv);
	if (rc)
		chip->vfloat_mv = -EINVAL;

	rc = of_property_read_u32(node, "qcom,recharge-mv",
						&chip->recharge_mv);
	if (rc)
		chip->recharge_mv = -EINVAL;

	chip->recharge_disabled = of_property_read_bool(node,
					"qcom,recharge-disabled");

	/* thermal and jeita support */
	rc = of_property_read_u32(node, "qcom,batt-cold-decidegc",
						&chip->batt_cold_decidegc);
	if (rc < 0)
		chip->batt_cold_decidegc = -EINVAL;

	rc = of_property_read_u32(node, "qcom,batt-hot-decidegc",
						&chip->batt_hot_decidegc);
	if (rc < 0)
		chip->batt_hot_decidegc = -EINVAL;

	rc = of_property_read_u32(node, "qcom,batt-warm-decidegc",
						&chip->batt_warm_decidegc);

	rc |= of_property_read_u32(node, "qcom,batt-cool-decidegc",
						&chip->batt_cool_decidegc);

	if (!rc) {
		rc = of_property_read_u32(node, "qcom,batt-cool-mv",
						&chip->batt_cool_mv);

		rc |= of_property_read_u32(node, "qcom,batt-warm-mv",
						&chip->batt_warm_mv);

		rc |= of_property_read_u32(node, "qcom,batt-cool-ma",
						&chip->batt_cool_ma);

		rc |= of_property_read_u32(node, "qcom,batt-warm-ma",
						&chip->batt_warm_ma);
		if (rc)
			chip->jeita_supported = false;
		else
			chip->jeita_supported = true;
	}

	pr_err("jeita_supported = %d\n", chip->jeita_supported);

	rc = of_property_read_u32(node, "qcom,batt-missing-decidegc",
						&chip->batt_missing_decidegc);

	chip->pinctrl_state_name = of_get_property(node, "pinctrl-names", NULL);

	return 0;
}

static int smb1351_determine_initial_state(struct smb1351_charger *chip)
{
	int rc;
	u8 reg = 0;

	/*
	 * It is okay to read the interrupt status here since
	 * interrupts aren't requested. Reading interrupt status
	 * clears the interrupt so be careful to read interrupt
	 * status only in interrupt handling code
	 */

	rc = smb1351_read_reg(chip, IRQ_B_REG, &reg);
	if (rc) {
		pr_err("Couldn't read IRQ_B rc = %d\n", rc);
		goto fail_init_status;
	}

	chip->battery_missing = (reg & IRQ_BATT_MISSING_BIT) ? true : false;

	rc = smb1351_read_reg(chip, IRQ_C_REG, &reg);
	if (rc) {
		pr_err("Couldn't read IRQ_C rc = %d\n", rc);
		goto fail_init_status;
	}
	chip->batt_full = (reg & IRQ_TERM_BIT) ? true : false;

	rc = smb1351_read_reg(chip, IRQ_A_REG, &reg);
	if (rc) {
		pr_err("Couldn't read irq A rc = %d\n", rc);
		return rc;
	}

	if (reg & IRQ_HOT_HARD_BIT)
		chip->batt_hot = true;
	if (reg & IRQ_COLD_HARD_BIT)
		chip->batt_cold = true;
	if (reg & IRQ_HOT_SOFT_BIT)
		chip->batt_warm = true;
	if (reg & IRQ_COLD_SOFT_BIT)
		chip->batt_cool = true;

	rc = smb1351_read_reg(chip, IRQ_E_REG, &reg);
	if (rc) {
		pr_err("Couldn't read IRQ_E rc = %d\n", rc);
		goto fail_init_status;
	}

	if (reg & IRQ_USBIN_UV_BIT) {
		smb1351_usbin_uv_handler(chip, 1);
	} else {
		smb1351_usbin_uv_handler(chip, 0);
		smb1351_apsd_complete_handler(chip, 1);
	}

	rc = smb1351_read_reg(chip, IRQ_G_REG, &reg);
	if (rc) {
		pr_err("Couldn't read IRQ_G rc = %d\n", rc);
		goto fail_init_status;
	}

	if (reg & IRQ_SOURCE_DET_BIT)
		smb1351_apsd_complete_handler(chip, 1);

	return 0;

fail_init_status:
	pr_err("Couldn't determine initial status\n");
	return rc;
}
static int create_debugfs_entries(struct smb1351_charger *chip)
{
	struct dentry *ent;

	chip->debug_root = debugfs_create_dir("smb1351", NULL);
	if (!chip->debug_root) {
		pr_err("Couldn't create debug dir\n");
	} else {
		ent = debugfs_create_file("config_registers", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &cnfg_debugfs_ops);
		if (!ent)
			pr_err("Couldn't create cnfg debug file\n");

		ent = debugfs_create_file("status_registers", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &status_debugfs_ops);
		if (!ent)
			pr_err("Couldn't create status debug file\n");

		ent = debugfs_create_file("cmd_registers", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &cmd_debugfs_ops);
		if (!ent)
			pr_err("Couldn't create cmd debug file\n");

		ent = debugfs_create_x32("address", S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root,
					  &(chip->peek_poke_address));
		if (!ent)
			pr_err("Couldn't create address debug file\n");

		ent = debugfs_create_file("data", S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root, chip,
					  &poke_poke_debug_ops);
		if (!ent)
			pr_err("Couldn't create data debug file\n");

		ent = debugfs_create_file("force_irq",
					  S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root, chip,
					  &force_irq_ops);
		if (!ent)
			pr_err("Couldn't create data debug file\n");

		ent = debugfs_create_file("irq_count", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &irq_count_debugfs_ops);
		if (!ent)
			pr_err("Couldn't create count debug file\n");
	}
	return 0;
}


static int oplus_chg_hw_init(void)
{
	
#ifdef CONFIG_OPLUS_CHARGER_MTK
	if (get_boot_mode() == META_BOOT || get_boot_mode() == FACTORY_BOOT
		|| get_boot_mode() == ADVMETA_BOOT || get_boot_mode() == ATE_FACTORY_BOOT) {
		smb1351_suspend_charger();
	} else {
		smb1351_unsuspend_charger();
	}
#else /* CONFIG_OPLUS_CHARGER_MTK */
	smb1351_unsuspend_charger();
#endif /* CONFIG_OPLUS_CHARGER_MTK */

	smb1351_enable_charging();
	return 0;
}
static int smb1351_kick_wdt(void)
{
	int rc;
	return 0;
	if (atomic_read(&smb1351_chip->charger_suspended) == 1) {
		return 0;
	}
	rc = smb1351_masked_write(smb1351_chip, WDOG_SAFETY_TIMER_CTRL_REG, WDOG_TIMER_EN_BIT, WDOG_TIMER_EN_BIT);
	return 0;

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

static void smb1351_set_aicl_point(int vol)
{
	return ;
}

static int smb1351_reset_charger(void)
{
	int rc;
	u8 reg1;
	u8 reg2;
	u8 reg3;
	rc = smb1351_masked_write(smb1351_chip, CHG_CURRENT_CTRL_REG, 0xff, 0x5a);
	rc = smb1351_masked_write(smb1351_chip, CHG_OTH_CURRENT_CTRL_REG, 0xff, 0x01);
	rc = smb1351_masked_write(smb1351_chip, VARIOUS_FUNC_REG, 0xff, 0x97);
	rc = smb1351_masked_write(smb1351_chip, VFLOAT_REG, 0xff, 0xeb);
	rc = smb1351_masked_write(smb1351_chip, CHG_CTRL_REG, 0xff, 0x81);
	rc = smb1351_masked_write(smb1351_chip, CHG_STAT_TIMERS_CTRL_REG, 0xff, 0x11);
	rc = smb1351_masked_write(smb1351_chip, CHG_PIN_EN_CTRL_REG, 0xff, 0x18);
	rc = smb1351_masked_write(smb1351_chip, THERM_A_CTRL_REG, 0xff, 0xd5);
	rc = smb1351_masked_write(smb1351_chip, WDOG_SAFETY_TIMER_CTRL_REG, 0xff, 0x80);
	rc = smb1351_masked_write(smb1351_chip, OTG_USBIN_AICL_CTRL_REG, 0xff, 0x18);
	rc = smb1351_masked_write(smb1351_chip, OTG_TLIM_CTRL_REG, 0xff, 0x2c);
	rc = smb1351_masked_write(smb1351_chip, HARD_SOFT_LIMIT_CELL_TEMP_REG, 0xff, 0x40);
	rc = smb1351_masked_write(smb1351_chip, FAULT_INT_REG, 0xff, 0x00);
	rc = smb1351_masked_write(smb1351_chip, STATUS_INT_REG, 0xff, 0x00);
	rc = smb1351_masked_write(smb1351_chip, VARIOUS_FUNC_2_REG, 0xff, 0x98);
	rc = smb1351_masked_write(smb1351_chip, FLEXCHARGER_REG, 0xff, 0x77);
	rc = smb1351_masked_write(smb1351_chip, VARIOUS_FUNC_3_REG, 0xff, 0xa2);
	rc = smb1351_masked_write(smb1351_chip, HVDCP_BATT_MISSING_CTRL_REG, 0xff, 0x60);
	rc = smb1351_masked_write(smb1351_chip, PON_OPTIONS_REG, 0xff, 0x4c);
	rc = smb1351_masked_write(smb1351_chip, OTG_MODE_POWER_OPTIONS_REG, 0xff, 0x1c);
	smb1351_read_reg(smb1351_chip, 0x42, &reg1);
	smb1351_read_reg(smb1351_chip, 0x44, &reg2);
	smb1351_read_reg(smb1351_chip, 0x46, &reg3);
	printk("smb1351_reset_charger reg[0]=0x%x,reg[1]=0x%x,reg[2]=0x%x\n",reg1,reg2,reg3);
	
	return 0;
}
static int smb1351_registers_read_full(void)
{
	return 0;
}
static int smb1351_get_chg_current_step(void)
{
	return 100;
	
}

int smbchg_otg_enable(void)
{
	int rc = 0;
	if (!smb1351_chip) {
		pr_err("%s smb1351_chip null,return\n", __func__);
		return 0;
	}
	rc = smb1351_masked_write(smb1351_chip, CMD_CHG_REG, CMD_OTG_EN_BIT,
							CMD_OTG_EN_BIT);
	if (rc)
		pr_err("Couldn't enable  OTG mode rc=%d\n", rc);
	pr_err("%s \n",__func__);
	return rc;
	
}

int smbchg_otg_disable(void)
{
	int rc = 0;
	if (!smb1351_chip) {
		pr_err("%s smb1351_chip null,return\n", __func__);
		return 0;
	}
	rc = smb1351_masked_write(smb1351_chip, CMD_CHG_REG, CMD_OTG_EN_BIT, 0);
	if (rc)
		pr_err("Couldn't disable OTG mode rc=%d\n", rc);
	pr_err("%s \n",__func__);
	return rc;
}

static void register_charger_devinfo(void)
{
	int ret = 0;
	char *version = "smb1351";
	char *manufacture = "smb";
	ret = register_device_proc("charger", version, manufacture);
	if (ret)
		chg_err("register_charger_devinfo fail\n");
}

extern int oplus_chg_shortc_hw_parse_dt(struct smb1351_charger *chip);
extern bool oplus_chg_get_shortc_hw_gpio_status(void);
//struct mutex charger_type_mutex;
struct oplus_chg_operations  smb1351_chg_ops = {
	.dump_registers = smb1351_dump_regs,
	.kick_wdt = smb1351_kick_wdt,
	.hardware_init = oplus_chg_hw_init,
	.charging_current_write_fast = smb1351_fastchg_current_set,
	.set_aicl_point = smb1351_set_aicl_point,
	.input_current_write = smb1351_set_usb_chg_current,
	.float_voltage_write = smb1351_float_voltage_set,
	.term_current_set = smb1351_iterm_set,
	.charging_enable = smb1351_enable_charging,
	.charging_disable = smb1351_disable_charging,
	.get_charging_enable = smb1351_check_charging_enable,
	.charger_suspend = smb1351_suspend_charger,
	.charger_unsuspend = smb1351_unsuspend_charger,
	.set_rechg_vol = smb1351_set_rechg_voltage,
	.reset_charger = smb1351_reset_charger,
	.read_full = smb1351_registers_read_full,
	.otg_enable = smbchg_otg_enable,
	.otg_disable = smbchg_otg_disable,
	.set_charging_term_disable = smb1351_set_chging_term_disable,
	.check_charger_resume = smb1351_check_charger_resume,
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
	.get_chg_current_step = smb1351_get_chg_current_step,
	#ifdef CONFIG_OPLUS_SHORT_C_BATT_CHECK
	.get_dyna_aicl_result = oplus_chg_get_dyna_aicl_result,
	#endif
	.get_shortc_hw_gpio_status = oplus_chg_get_shortc_hw_gpio_status,
	#ifdef CONFIG_OPLUS_RTC_DET_SUPPORT
	.check_rtc_reset = rtc_reset_check,
	#endif
};


static int smb1351_charger_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int rc;
	struct smb1351_charger *chip = NULL;
	struct power_supply *usb_psy;
	struct power_supply_config batt_psy_cfg = {};
	u8 reg = 0;
	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		pr_err("Couldn't allocate memory\n");
		return -ENOMEM;
	}
	chg_debug( "SMB1351: call \n");
	chip->client = client;
	chip->dev = &client->dev;
	//chip->usb_psy = usb_psy;
	chip->fake_battery_soc = -EINVAL;
	smb1351_chip = chip;
	smb1351_dump_regs();
	if(charger_ic_flag != 3) {
		pr_err("%s, read error,return \n",__func__);
		return -ENODEV;
	}
	//oplus_chg_parse_dt(chip);
	//oplus_chg_shortc_hw_parse_dt(chip);
	atomic_set(&chip->charger_suspended, 0);
	
	//INIT_DELAYED_WORK(&chip->chg_remove_work, smb1351_chg_remove_work);
	device_init_wakeup(chip->dev, true);

	//mutex_init(&charger_type_mutex);
	/* probe the device to check if its actually connected */
	rc = smb1351_read_reg(chip, CHG_REVISION_REG, &reg);
	if (rc) {
		pr_err("Failed to detect smb1351, device may be absent\n");
		return -ENODEV;
	}
	pr_err("smb1351 chip revision is %d\n", reg);

	rc = smb1351_parse_dt(chip);
	if (rc) {
		pr_err("Couldn't parse DT nodes rc=%d\n", rc);
		return rc;
	}


	i2c_set_clientdata(client, chip);

	chip->resume_completed = true;
	mutex_init(&chip->irq_complete);

	//smb1351_dump_regs(chip);

	//rc = smb1351_regulator_init(chip);
	if (rc) {
		pr_err("Couldn't initialize smb1351 ragulator rc=%d\n", rc);
	}

	rc = smb1351_hw_init();
	if (rc) {
		pr_err("Couldn't intialize hardware rc=%d\n", rc);
	}

	rc = smb1351_determine_initial_state(chip);
	if (rc) {
		pr_err("Couldn't determine initial state rc=%d\n", rc);
	}
	/* STAT irq configuration */
	if (client->irq) {
		rc = devm_request_threaded_irq(&client->dev, client->irq, NULL,
				smb1351_chg_stat_handler,
				IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				"smb1351_chg_stat_irq", chip);
		if (rc) {
			pr_err("Failed STAT irq=%d request rc = %d\n",
				client->irq, rc);
		}
		enable_irq_wake(client->irq);
	}

	register_charger_devinfo();

	create_debugfs_entries(chip);

	pr_err("smb1351 successfully probed. charger=%d,version=%s\n",
			chip->chg_present,
			smb1351_version_str[chip->version]);
	return 0;

}

static int smb1351_charger_remove(struct i2c_client *client)
{

	return 0;
}

static void smb1351_shutdown(struct i2c_client *client)
{
	return;
}

static int smb1351_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct smb1351_charger *chip = i2c_get_clientdata(client);

	/* no suspend resume activities for parallel charger */
	if (chip->parallel_charger)
		return 0;

	mutex_lock(&chip->irq_complete);
	chip->resume_completed = false;
	mutex_unlock(&chip->irq_complete);

	return 0;
}

static int smb1351_suspend_noirq(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct smb1351_charger *chip = i2c_get_clientdata(client);

	/* no suspend resume activities for parallel charger */
	if (chip->parallel_charger)
		return 0;

	if (chip->irq_waiting) {
		pr_err_ratelimited("Aborting suspend, an interrupt was detected while suspending\n");
		return -EBUSY;
	}
	return 0;
}

static int smb1351_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct smb1351_charger *chip = i2c_get_clientdata(client);

	/* no suspend resume activities for parallel charger */
	if (chip->parallel_charger)
		return 0;

	mutex_lock(&chip->irq_complete);
	chip->resume_completed = true;
	if (chip->irq_waiting) {
		mutex_unlock(&chip->irq_complete);
		smb1351_chg_stat_handler(client->irq, chip);
		enable_irq(client->irq);
	} else {
		mutex_unlock(&chip->irq_complete);
	}
	return 0;
}

static const struct dev_pm_ops smb1351_pm_ops = {
	.suspend	= smb1351_suspend,
	.suspend_noirq	= smb1351_suspend_noirq,
	.resume		= smb1351_resume,
};

static struct of_device_id smb1351_match_table[] = {
	{ .compatible = "qcom,smb1351-charger",},
	{ },
};

static const struct i2c_device_id smb1351_charger_id[] = {
	{"smb1351-charger", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, smb1351_charger_id);

static struct i2c_driver smb1351_charger_driver = {
	.driver		= {
		.name		= "smb1351-charger",
		.owner		= THIS_MODULE,
		.of_match_table	= smb1351_match_table,
//		.pm		= &smb1351_pm_ops,
	},
	.probe		= smb1351_charger_probe,
	.remove		= smb1351_charger_remove,
	.shutdown	= smb1351_shutdown,
	.id_table	= smb1351_charger_id,
};

module_i2c_driver(smb1351_charger_driver);

MODULE_DESCRIPTION("smb1351 Charger");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:smb1351-charger");
