/* Copyright (c) 2014-2016 The Linux Foundation. All rights reserved.
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
#define pr_fmt(fmt) "SMBCHG: %s: " fmt, __func__

#include <linux/spmi.h>
#include <linux/spinlock.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/bitops.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/spmi.h>
#include <linux/printk.h>
#include <linux/ratelimit.h>
#include <linux/debugfs.h>
#include <linux/leds.h>
#include <linux/rtc.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/batterydata-lib.h>
#include <linux/of_batterydata.h>
#include <linux/msm_bcl.h>
#include <linux/ktime.h>
//#include "../../pmic-voter.h"
#include "../../../../kernel/drivers/power/pmic-voter.h"
#include <soc/oplus/system/boot_mode.h>
#include "../oplus_vooc.h"
#include "../oplus_gauge.h"
#include "../oplus_charger.h"

/* Mask/Bit helpers */
#define _SMB_MASK(BITS, POS) \
	((unsigned char)(((1 << (BITS)) - 1) << (POS)))
#define SMB_MASK(LEFT_BIT_POS, RIGHT_BIT_POS) \
		_SMB_MASK((LEFT_BIT_POS) - (RIGHT_BIT_POS) + 1, \
				(RIGHT_BIT_POS))
				
static struct oplus_chg_chip *the_chip = NULL;

enum qpnp_schg {
	QPNP_SCHG,
	QPNP_SCHG_LITE,
};

static char *version_str[] = {
	[QPNP_SCHG]		= "SCHG",
	[QPNP_SCHG_LITE]	= "SCHG_LITE",
};

enum pmic_subtype {
	PMI8994		= 10,
	PMI8950		= 17,
	PMI8996		= 19,
	PMI8937		= 55,
};

enum smbchg_wa {
	SMBCHG_AICL_DEGLITCH_WA = BIT(0),
	SMBCHG_HVDCP_9V_EN_WA	= BIT(1),
	SMBCHG_USB100_WA = BIT(2),
	SMBCHG_BATT_OV_WA = BIT(3),
	SMBCHG_CC_ESR_WA = BIT(4),
	SMBCHG_FLASH_ICL_DISABLE_WA = BIT(5),
	SMBCHG_RESTART_WA = BIT(6),
	SMBCHG_FLASH_BUCK_SWITCH_FREQ_WA = BIT(7),
};

enum print_reason {
	PR_REGISTER	= BIT(0),
	PR_INTERRUPT	= BIT(1),
	PR_STATUS	= BIT(2),
	PR_DUMP		= BIT(3),
	PR_PM		= BIT(4),
	PR_MISC		= BIT(5),
	PR_WIPOWER	= BIT(6),
	PR_TYPEC	= BIT(7),
};

enum wake_reason {
	PM_PARALLEL_CHECK = BIT(0),
	PM_REASON_VFLOAT_ADJUST = BIT(1),
	PM_ESR_PULSE = BIT(2),
	PM_PARALLEL_TAPER = BIT(3),
	PM_DETECT_HVDCP = BIT(4),
	PM_USB_PRESENT = BIT(5),
};

enum fcc_voters {
	ESR_PULSE_FCC_VOTER,
	BATT_TYPE_FCC_VOTER,
	RESTRICTED_CHG_FCC_VOTER,
	NUM_FCC_VOTER,
};

enum icl_voters {
	PSY_ICL_VOTER,
	THERMAL_ICL_VOTER,
	HVDCP_ICL_VOTER,
	USER_ICL_VOTER,
	WEAK_CHARGER_ICL_VOTER,
	SW_AICL_ICL_VOTER,
	CHG_SUSPEND_WORKAROUND_ICL_VOTER,
	NUM_ICL_VOTER,
};

enum enable_voters {
	/* userspace has suspended charging altogether */
	USER_EN_VOTER,
	/*
	 * this specific path has been suspended through the power supply
	 * framework
	 */
	POWER_SUPPLY_EN_VOTER,
	/*
	 * the usb driver has suspended this path by setting a current limit
	 * of < 2MA
	 */
	USB_EN_VOTER,
	/*
	 * when a wireless charger comes online,
	 * the dc path is suspended for a second
	 */
	WIRELESS_EN_VOTER,
	/*
	 * the thermal daemon can suspend a charge path when the system
	 * temperature levels rise
	 */
	THERMAL_EN_VOTER,
	/*
	 * an external OTG supply is being used, suspend charge path so the
	 * charger does not accidentally try to charge from the external supply.
	 */
	OTG_EN_VOTER,
	/*
	 * the charger is very weak, do not draw any current from it
	 */
	WEAK_CHARGER_EN_VOTER,
	/*
	 * fake battery voter, if battery id-resistance around 7.5 Kohm
	 */
	FAKE_BATTERY_EN_VOTER,
	NUM_EN_VOTERS,
};

enum battchg_enable_voters {
	/* userspace has disabled battery charging */
	BATTCHG_USER_EN_VOTER,
	/* battery charging disabled while loading battery profiles */
	BATTCHG_UNKNOWN_BATTERY_EN_VOTER,
	NUM_BATTCHG_EN_VOTERS,
};

enum hw_aicl_rerun_enable_indirect_voters {
	/* enabled via device tree */
	DEFAULT_CONFIG_HW_AICL_VOTER,
	/* Varb workaround voter */
	VARB_WORKAROUND_VOTER,
	/* SHUTDOWN workaround voter */
	SHUTDOWN_WORKAROUND_VOTER,
	NUM_HW_AICL_RERUN_ENABLE_INDIRECT_VOTERS,
};

enum hw_aicl_rerun_disable_voters {
	/* the results from enabling clients */
	HW_AICL_RERUN_ENABLE_INDIRECT_VOTER,
	/* Weak charger voter */
	WEAK_CHARGER_HW_AICL_VOTER,
	NUM_HW_AICL_DISABLE_VOTERS,
};

enum aicl_short_deglitch_voters {
	/* Varb workaround voter */
	VARB_WORKAROUND_SHORT_DEGLITCH_VOTER,
	/* QC 2.0 */
	HVDCP_SHORT_DEGLITCH_VOTER,
	NUM_HW_SHORT_DEGLITCH_VOTERS,
};
static int smbchg_debug_mask;
module_param_named(
	debug_mask, smbchg_debug_mask, int, S_IRUSR | S_IWUSR
);

static int smbchg_parallel_en = 1;
module_param_named(
	parallel_en, smbchg_parallel_en, int, S_IRUSR | S_IWUSR
);

static int smbchg_main_chg_fcc_percent = 50;
module_param_named(
	main_chg_fcc_percent, smbchg_main_chg_fcc_percent,
	int, S_IRUSR | S_IWUSR
);

static int smbchg_main_chg_icl_percent = 60;
module_param_named(
	main_chg_icl_percent, smbchg_main_chg_icl_percent,
	int, S_IRUSR | S_IWUSR
);

static int smbchg_default_hvdcp_icl_ma = 1800;
module_param_named(
	default_hvdcp_icl_ma, smbchg_default_hvdcp_icl_ma,
	int, S_IRUSR | S_IWUSR
);

static int smbchg_default_hvdcp3_icl_ma = 3000;
module_param_named(
	default_hvdcp3_icl_ma, smbchg_default_hvdcp3_icl_ma,
	int, S_IRUSR | S_IWUSR
);

static int smbchg_default_dcp_icl_ma = 1200;	//1800
module_param_named(
	default_dcp_icl_ma, smbchg_default_dcp_icl_ma,
	int, S_IRUSR | S_IWUSR
);

static int wipower_dyn_icl_en;
module_param_named(
	dynamic_icl_wipower_en, wipower_dyn_icl_en,
	int, S_IRUSR | S_IWUSR
);

static int wipower_dcin_interval = ADC_MEAS1_INTERVAL_2P0MS;
module_param_named(
	wipower_dcin_interval, wipower_dcin_interval,
	int, S_IRUSR | S_IWUSR
);

#define WIPOWER_DEFAULT_HYSTERISIS_UV	250000
static int wipower_dcin_hyst_uv = WIPOWER_DEFAULT_HYSTERISIS_UV;
module_param_named(
	wipower_dcin_hyst_uv, wipower_dcin_hyst_uv,
	int, S_IRUSR | S_IWUSR
);

#if 1
#define pr_smb(reason, fmt, ...)				\
	do {							\
		if (smbchg_debug_mask & (reason))		\
			pr_err(fmt, ##__VA_ARGS__);		\
		else						\
			pr_debug(fmt, ##__VA_ARGS__);		\
	} while (0)

#define pr_smb_rt(reason, fmt, ...)					\
	do {								\
		if (smbchg_debug_mask & (reason))			\
			pr_err_ratelimited(fmt, ##__VA_ARGS__);	\
		else							\
			pr_debug_ratelimited(fmt, ##__VA_ARGS__);	\
	} while (0)
#else
#define pr_smb(reason, fmt, ...)				\
	do {							\
		pr_err(fmt, ##__VA_ARGS__);		\
	} while (0)
#define pr_smb_rt(reason, fmt, ...)					\
	do {								\
		pr_err_ratelimited(fmt, ##__VA_ARGS__);	\
	} while (0)	
#endif

static int smbchg_read(struct oplus_chg_chip *chip, u8 *val,
			u16 addr, int count)
{
	int rc = 0;
	struct spmi_device *spmi = chip->pmic_spmi.spmi;

	if (addr == 0) {
		dev_err(chip->dev, "addr cannot be zero addr=0x%02x sid=0x%02x rc=%d\n",
			addr, spmi->sid, rc);
		return -EINVAL;
	}

	rc = spmi_ext_register_readl(spmi->ctrl, spmi->sid, addr, val, count);
	if (rc) {
		dev_err(chip->dev, "spmi read failed addr=0x%02x sid=0x%02x rc=%d\n",
				addr, spmi->sid, rc);
		return rc;
	}
	return 0;
}

/*
 * Writes an arbitrary number of bytes to a specified register
 *
 * Do not use this function for register writes if possible. Instead use the
 * smbchg_masked_write function.
 *
 * The sec_access_lock must be held for all register writes and this function
 * does not do that. If this function is used, please hold the spinlock or
 * random secure access writes may fail.
 */
static int smbchg_write(struct oplus_chg_chip *chip, u8 *val,
			u16 addr, int count)
{
	int rc = 0;
	struct spmi_device *spmi = chip->pmic_spmi.spmi;

	if (addr == 0) {
		dev_err(chip->dev, "addr cannot be zero addr=0x%02x sid=0x%02x rc=%d\n",
			addr, spmi->sid, rc);
		return -EINVAL;
	}

	rc = spmi_ext_register_writel(spmi->ctrl, spmi->sid, addr, val, count);
	if (rc) {
		dev_err(chip->dev, "write failed addr=0x%02x sid=0x%02x rc=%d\n",
			addr, spmi->sid, rc);
		return rc;
	}

	return 0;
}

/*
 * Writes a register to the specified by the base and limited by the bit mask
 *
 * Do not use this function for register writes if possible. Instead use the
 * smbchg_masked_write function.
 *
 * The sec_access_lock must be held for all register writes and this function
 * does not do that. If this function is used, please hold the spinlock or
 * random secure access writes may fail.
 */
static int smbchg_masked_write_raw(struct oplus_chg_chip *chip, u16 base, u8 mask,
									u8 val)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, base, 1);
	if (rc) {
		dev_err(chip->dev, "spmi read failed: addr=%03X, rc=%d\n",
				base, rc);
		return rc;
	}

	reg &= ~mask;
	reg |= val & mask;

	pr_smb(PR_REGISTER, "addr = 0x%x writing 0x%x\n", base, reg);

	rc = smbchg_write(chip, &reg, base, 1);
	if (rc) {
		dev_err(chip->dev, "spmi write failed: addr=%03X, rc=%d\n",
				base, rc);
		return rc;
	}

	return 0;
}

/*
 * Writes a register to the specified by the base and limited by the bit mask
 *
 * This function holds a spin lock to ensure secure access register writes goes
 * through. If the secure access unlock register is armed, any old register
 * write can unarm the secure access unlock, causing the next write to fail.
 *
 * Note: do not use this for sec_access registers. Instead use the function
 * below: smbchg_sec_masked_write
 */
static int smbchg_masked_write(struct oplus_chg_chip *chip, u16 base, u8 mask,
								u8 val)
{
	unsigned long flags;
	int rc;

	spin_lock_irqsave(&chip->pmic_spmi.sec_access_lock, flags);
	rc = smbchg_masked_write_raw(chip, base, mask, val);
	spin_unlock_irqrestore(&chip->pmic_spmi.sec_access_lock, flags);

	return rc;
}

/*
 * Unlocks sec access and writes to the register specified.
 *
 * This function holds a spin lock to exclude other register writes while
 * the two writes are taking place.
 */
#define SEC_ACCESS_OFFSET	0xD0
#define SEC_ACCESS_VALUE	0xA5
#define PERIPHERAL_MASK		0xFF
static int smbchg_sec_masked_write(struct oplus_chg_chip *chip, u16 base, u8 mask,
									u8 val)
{
	unsigned long flags;
	int rc;
	u16 peripheral_base = base & (~PERIPHERAL_MASK);

	spin_lock_irqsave(&chip->pmic_spmi.sec_access_lock, flags);

	rc = smbchg_masked_write_raw(chip, peripheral_base + SEC_ACCESS_OFFSET,
				SEC_ACCESS_VALUE, SEC_ACCESS_VALUE);
	if (rc) {
		dev_err(chip->dev, "Unable to unlock sec_access: %d", rc);
		goto out;
	}

	rc = smbchg_masked_write_raw(chip, base, mask, val);

out:
	spin_unlock_irqrestore(&chip->pmic_spmi.sec_access_lock, flags);
	return rc;
}

static void smbchg_stay_awake(struct oplus_chg_chip *chip, int reason)
{
	int reasons;

	mutex_lock(&chip->pmic_spmi.pm_lock);
	reasons = chip->pmic_spmi.wake_reasons | reason;
	if (reasons != 0 && chip->pmic_spmi.wake_reasons == 0) {
		pr_smb(PR_PM, "staying awake: 0x%02x (bit %d)\n",
				reasons, reason);
		pm_stay_awake(chip->dev);
	}
	chip->pmic_spmi.wake_reasons = reasons;
	mutex_unlock(&chip->pmic_spmi.pm_lock);
}

static void smbchg_relax(struct oplus_chg_chip *chip, int reason)
{
	int reasons;

	mutex_lock(&chip->pmic_spmi.pm_lock);
	reasons = chip->pmic_spmi.wake_reasons & (~reason);
	if (reasons == 0 && chip->pmic_spmi.wake_reasons != 0) {
		pr_smb(PR_PM, "relaxing: 0x%02x (bit %d)\n",
				reasons, reason);
		pm_relax(chip->dev);
	}
	chip->pmic_spmi.wake_reasons = reasons;
	mutex_unlock(&chip->pmic_spmi.pm_lock);
};

#if 0
enum pwr_path_type {
	UNKNOWN = 0,
	PWR_PATH_BATTERY = 1,
	PWR_PATH_USB = 2,
	PWR_PATH_DC = 3,
};

#define PWR_PATH		0x08
#define PWR_PATH_MASK		0x03
static enum pwr_path_type smbchg_get_pwr_path(struct oplus_chg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->pmic_spmi.usb_chgpth_base + PWR_PATH, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read PWR_PATH rc = %d\n", rc);
		return PWR_PATH_BATTERY;
	}

	return reg & PWR_PATH_MASK;
}
#endif

static int smbchg_chargerid_switch_gpio_init(struct oplus_chg_chip *chip)
{
    chip->normalchg_gpio.pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->normalchg_gpio.pinctrl)) {
		chg_err("get normalchg_gpio.pinctrl fail\n");
		return -EINVAL;
	}

	chip->normalchg_gpio.chargerid_switch_active = 
			pinctrl_lookup_state(chip->normalchg_gpio.pinctrl, 
				"chargerid_switch_active");
	if (IS_ERR_OR_NULL(chip->normalchg_gpio.chargerid_switch_active)) {
		chg_err("get chargerid_switch_active fail\n");
		return -EINVAL;
	}

	chip->normalchg_gpio.chargerid_switch_sleep = 
			pinctrl_lookup_state(chip->normalchg_gpio.pinctrl, 
				"chargerid_switch_sleep");
	if (IS_ERR_OR_NULL(chip->normalchg_gpio.chargerid_switch_sleep)) {
		chg_err("get chargerid_switch_sleep fail\n");
		return -EINVAL;
	}

	chip->normalchg_gpio.chargerid_switch_default = 
			pinctrl_lookup_state(chip->normalchg_gpio.pinctrl, 
				"chargerid_switch_default");
	if (IS_ERR_OR_NULL(chip->normalchg_gpio.chargerid_switch_default)) {
		chg_err("get chargerid_switch_default fail\n");
		return -EINVAL;
	}

	if(chip->normalchg_gpio.chargerid_switch_gpio > 0) {
		gpio_direction_output(chip->normalchg_gpio.chargerid_switch_gpio, 0);
	}
	pinctrl_select_state(chip->normalchg_gpio.pinctrl,
		chip->normalchg_gpio.chargerid_switch_default);
	
	return 0;
}


static bool is_usb_present(struct oplus_chg_chip *chip);
static void smbchg_set_chargerid_switch_val(
				struct oplus_chg_chip *chip, int value)
{
	if(chip->normalchg_gpio.chargerid_switch_gpio <= 0) {
		chg_err("chargerid_switch_gpio not exist, return\n");
		return;
	}
	if(IS_ERR_OR_NULL(chip->normalchg_gpio.pinctrl)
		|| IS_ERR_OR_NULL(chip->normalchg_gpio.chargerid_switch_active)
		|| IS_ERR_OR_NULL(chip->normalchg_gpio.chargerid_switch_sleep)
		|| IS_ERR_OR_NULL(chip->normalchg_gpio.chargerid_switch_default)) {
		chg_err("pinctrl null, return\n");
		return;
	}

	if(oplus_vooc_get_adapter_update_real_status() == ADAPTER_FW_NEED_UPDATE
		|| oplus_vooc_get_btb_temp_over() == true) {
		chg_err("adapter update or btb_temp_over, return\n");
		return;
	}
	
	if(chip->pmic_spmi.not_support_1200ma && !value && !is_usb_present(chip)) {
	/* BugID 879716 : Solve some situatuion ChargerID is not 0 mV when usb is not present */
		chip->chargerid_volt = 0;
		chip->chargerid_volt_got = false;
	}
	
	if(value) {
		gpio_direction_output(chip->normalchg_gpio.chargerid_switch_gpio, 1);
		pinctrl_select_state(chip->normalchg_gpio.pinctrl,
				chip->normalchg_gpio.chargerid_switch_default);
	} else {
		gpio_direction_output(chip->normalchg_gpio.chargerid_switch_gpio, 0);
		pinctrl_select_state(chip->normalchg_gpio.pinctrl,
				chip->normalchg_gpio.chargerid_switch_default);
	}
	chg_err("set value:%d, gpio_val:%d\n", 
		value, gpio_get_value(chip->normalchg_gpio.chargerid_switch_gpio));
}

static int smbchg_get_chargerid_switch_val(struct oplus_chg_chip *chip)
{
	if(chip->normalchg_gpio.chargerid_switch_gpio <= 0) {
		chg_err("chargerid_switch_gpio not exist, return\n");
		return -1;
	}
	if(IS_ERR_OR_NULL(chip->normalchg_gpio.pinctrl)
		|| IS_ERR_OR_NULL(chip->normalchg_gpio.chargerid_switch_active)
		|| IS_ERR_OR_NULL(chip->normalchg_gpio.chargerid_switch_sleep)) {
		chg_err("pinctrl null, return\n");
		return -1;
	}

	return gpio_get_value(chip->normalchg_gpio.chargerid_switch_gpio);
}

static int smbchg_get_chargerid_volt(struct oplus_chg_chip *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;
	int chargerid_volt = 0;

	if(!chip->pmic_spmi.pm8953_vadc_dev) {
		chg_err("pm8953_vadc_dev NULL\n");
		return 0;
	}

	rc = qpnp_vadc_read(chip->pmic_spmi.pm8953_vadc_dev, P_MUX2_1_1, &results);
	if (rc) {
		chg_err("unable to read P_MUX2_1_1 rc = %d\n", rc);
		return 0;
	}
	chargerid_volt = (int)results.physical/1000;
	chg_err("chargerid_volt:%d\n", chargerid_volt);
	return chargerid_volt;
}


#define RID_STS				0xB
#define RID_MASK			0xF
#define IDEV_STS			0x8
#define RT_STS				0x10
#define USBID_MSB			0xE
#define USBIN_UV_BIT			BIT(0)
#define USBIN_OV_BIT			BIT(1)
#define USBIN_SRC_DET_BIT		BIT(2)
#define FMB_STS_MASK			SMB_MASK(3, 0)
#define USBID_GND_THRESHOLD		0x495
static bool is_otg_present_schg(struct oplus_chg_chip *chip)
{
	int rc;
	u8 reg;
	u8 usbid_reg[2];
	u16 usbid_val;
	/*
	 * After the falling edge of the usbid change interrupt occurs,
	 * there may still be some time before the ADC conversion for USB RID
	 * finishes in the fuel gauge. In the worst case, this could be up to
	 * 15 ms.
	 *
	 * Sleep for 20 ms (minimum msleep time) to wait for the conversion to
	 * finish and the USB RID status register to be updated before trying
	 * to detect OTG insertions.
	 */

	msleep(20);

	/*
	 * There is a problem with USBID conversions on PMI8994 revisions
	 * 2.0.0. As a workaround, check that the cable is not
	 * detected as factory test before enabling OTG.
	 */
	rc = smbchg_read(chip, &reg, chip->pmic_spmi.misc_base + IDEV_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read IDEV_STS rc = %d\n", rc);
		return false;
	}

	if ((reg & FMB_STS_MASK) != 0) {
		pr_smb(PR_STATUS, "IDEV_STS = %02x, not ground\n", reg);
		return false;
	}

	rc = smbchg_read(chip, usbid_reg, chip->pmic_spmi.usb_chgpth_base + USBID_MSB, 2);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read USBID rc = %d\n", rc);
		return false;
	}
	usbid_val = (usbid_reg[0] << 8) | usbid_reg[1];

	if (usbid_val > USBID_GND_THRESHOLD) {
		pr_smb(PR_STATUS, "USBID = 0x%04x, too high to be ground\n",
				usbid_val);
		return false;
	}

	rc = smbchg_read(chip, &reg, chip->pmic_spmi.usb_chgpth_base + RID_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev,
				"Couldn't read usb rid status rc = %d\n", rc);
		return false;
	}

	pr_smb(PR_STATUS, "RID_STS = %02x\n", reg);

	return (reg & RID_MASK) == 0;
}

#define RID_GND_DET_STS			BIT(2)
static bool is_otg_present_schg_lite(struct oplus_chg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->pmic_spmi.otg_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't read otg RT status rc = %d\n", rc);
		return false;
	}

	return !!(reg & RID_GND_DET_STS);
}

static bool is_otg_present(struct oplus_chg_chip *chip)
{
	if (chip->pmic_spmi.schg_version == QPNP_SCHG_LITE)
		return is_otg_present_schg_lite(chip);

	return is_otg_present_schg(chip);
}

#define USBIN_9V			BIT(5)
#define USBIN_UNREG			BIT(4)
#define USBIN_LV			BIT(3)
#define DCIN_9V				BIT(2)
#define DCIN_UNREG			BIT(1)
#define DCIN_LV				BIT(0)
#define INPUT_STS			0x0D
#define DCIN_UV_BIT			BIT(0)
#define DCIN_OV_BIT			BIT(1)
static bool is_dc_present(struct oplus_chg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->pmic_spmi.dc_chgpth_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read dc status rc = %d\n", rc);
		return false;
	}

	if ((reg & DCIN_UV_BIT) || (reg & DCIN_OV_BIT))
		return false;

	return true;
}

static bool is_usb_present(struct oplus_chg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->pmic_spmi.usb_chgpth_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb rt status rc = %d\n", rc);
		return false;
	}
	if (!(reg & USBIN_SRC_DET_BIT) || (reg & USBIN_OV_BIT))
		return false;

	rc = smbchg_read(chip, &reg, chip->pmic_spmi.usb_chgpth_base + INPUT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb status rc = %d\n", rc);
		return false;
	}

	return !!(reg & (USBIN_9V | USBIN_UNREG | USBIN_LV));
}

static bool oplus_chg_is_usb_present(void)
{
	if(!the_chip) {
		return false;
	} else {
		return is_usb_present(the_chip);
	}
}

static char *usb_type_str[] = {
	"SDP",		/* bit 0 */
	"OTHER",	/* bit 1 */
	"DCP",		/* bit 2 */
	"CDP",		/* bit 3 */
	"NONE",		/* bit 4 error case */
};

#define N_TYPE_BITS		4
#define TYPE_BITS_OFFSET	4

static int get_type(u8 type_reg)
{
	unsigned long type = type_reg;
	type >>= TYPE_BITS_OFFSET;
	return find_first_bit(&type, N_TYPE_BITS);
}

/* helper to return the string of USB type */
static inline char *get_usb_type_name(int type)
{
	return usb_type_str[type];
}

static enum power_supply_type usb_type_enum[] = {
	POWER_SUPPLY_TYPE_USB,		/* bit 0 */
	POWER_SUPPLY_TYPE_USB_DCP,	/* bit 1 */
	POWER_SUPPLY_TYPE_USB_DCP,	/* bit 2 */
	POWER_SUPPLY_TYPE_USB_CDP,	/* bit 3 */
	POWER_SUPPLY_TYPE_USB_DCP,	/* bit 4 error case, report DCP */
};

/* helper to return enum power_supply_type of USB type */
static inline enum power_supply_type get_usb_supply_type(int type)
{
	return usb_type_enum[type];
}

static bool is_src_detect_high(struct oplus_chg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->pmic_spmi.usb_chgpth_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb rt status rc = %d\n", rc);
		return false;
	}
	return reg &= USBIN_SRC_DET_BIT;
}

static void read_usb_type(struct oplus_chg_chip *chip, char **usb_type_name,
				enum power_supply_type *usb_supply_type)
{
	int rc, type;
	u8 reg;

	if (!is_src_detect_high(chip)) {
		pr_smb(PR_MISC, "src det low\n");
		*usb_type_name = "Absent";
		*usb_supply_type = POWER_SUPPLY_TYPE_UNKNOWN;
		return;
	}

	rc = smbchg_read(chip, &reg, chip->pmic_spmi.misc_base + IDEV_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read status 5 rc = %d\n", rc);
		*usb_type_name = "Other";
		*usb_supply_type = POWER_SUPPLY_TYPE_UNKNOWN;
		return;
	}
	type = get_type(reg);
	chip->pmic_spmi.primal_chgtype = type;
	*usb_type_name = get_usb_type_name(type);
	*usb_supply_type = get_usb_supply_type(type);
}

static void qpnp_get_charger_type(struct oplus_chg_chip *chip, char **usb_type_name,
				enum power_supply_type *usb_supply_type, bool need_ap_detect)
{
	union power_supply_propval ret = {0,}; 

	read_usb_type(chip, usb_type_name, usb_supply_type);
	if (need_ap_detect &&
		(*usb_supply_type == POWER_SUPPLY_TYPE_USB 
			|| *usb_supply_type == POWER_SUPPLY_TYPE_USB_CDP)){
		chip->usb_psy->get_property(chip->usb_psy, 
                  POWER_SUPPLY_PROP_TYPE, &ret); 
		*usb_supply_type = ret.intval;
		chg_err("sdp/cdp, ap detect again usb_supply_type:%d\n", *usb_supply_type);
	}
	chip->pmic_spmi.usb_supply_type = *usb_supply_type;
}

#define CHGR_STS			0x0E
#define BATT_LESS_THAN_2V		BIT(4)
#define CHG_HOLD_OFF_BIT		BIT(3)
#define CHG_TYPE_MASK			SMB_MASK(2, 1)
#define CHG_TYPE_SHIFT			1
#define BATT_NOT_CHG_VAL		0x0
#define BATT_PRE_CHG_VAL		0x1
#define BATT_FAST_CHG_VAL		0x2
#define BATT_TAPER_CHG_VAL		0x3
#define CHG_INHIBIT_BIT			BIT(1)
#define BAT_TCC_REACHED_BIT		BIT(7)
static int get_prop_batt_status(struct oplus_chg_chip *chip)
{
	int rc, status = POWER_SUPPLY_STATUS_DISCHARGING;
	u8 reg = 0, chg_type;
	bool charger_present, chg_inhibit;

	charger_present = is_usb_present(chip) | is_dc_present(chip) |
			  chip->pmic_spmi.hvdcp_3_det_ignore_uv;
	if (!charger_present)
		return POWER_SUPPLY_STATUS_DISCHARGING;

	rc = smbchg_read(chip, &reg, chip->pmic_spmi.chgr_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Unable to read RT_STS rc = %d\n", rc);
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

	if (reg & BAT_TCC_REACHED_BIT)
		return POWER_SUPPLY_STATUS_FULL;

	chg_inhibit = reg & CHG_INHIBIT_BIT;
	if (chg_inhibit)
		return POWER_SUPPLY_STATUS_FULL;

	rc = smbchg_read(chip, &reg, chip->pmic_spmi.chgr_base + CHGR_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Unable to read CHGR_STS rc = %d\n", rc);
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

	if (reg & CHG_HOLD_OFF_BIT) {
		/*
		 * when chg hold off happens the battery is
		 * not charging
		 */
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		goto out;
	}

	chg_type = (reg & CHG_TYPE_MASK) >> CHG_TYPE_SHIFT;

	if (chg_type == BATT_NOT_CHG_VAL && !chip->pmic_spmi.hvdcp_3_det_ignore_uv)
		status = POWER_SUPPLY_STATUS_DISCHARGING;
	else
		status = POWER_SUPPLY_STATUS_CHARGING;
out:
	pr_smb_rt(PR_MISC, "CHGR_STS = 0x%02x\n", reg);
	return status;
}

static int smbchg_read_full(struct oplus_chg_chip *chip)
{
	int rc,status = 0;
	u8 reg = 0;


	rc = smbchg_read(chip, &reg, chip->pmic_spmi.chgr_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Unable to read RT_STS rc = %d\n", rc);
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

	if (reg & (BAT_TCC_REACHED_BIT | CHG_INHIBIT_BIT)){
		status = 1;
		pr_err("[%s]oplus_status_reg=0x%x\n",__func__,reg);
	}

	return status;		
}

#define BAT_PRES_STATUS			0x08
#define BAT_PRES_BIT			BIT(7)
static int get_prop_batt_present(struct oplus_chg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->pmic_spmi.bat_if_base + BAT_PRES_STATUS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Unable to read CHGR_STS rc = %d\n", rc);
		return 0;
	}

	return !!(reg & BAT_PRES_BIT);
}

static int get_prop_charge_type(struct oplus_chg_chip *chip)
{
	int rc;
	u8 reg, chg_type;

	rc = smbchg_read(chip, &reg, chip->pmic_spmi.chgr_base + CHGR_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Unable to read CHGR_STS rc = %d\n", rc);
		return 0;
	}

	chg_type = (reg & CHG_TYPE_MASK) >> CHG_TYPE_SHIFT;
	if (chg_type == BATT_NOT_CHG_VAL)
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	else if (chg_type == BATT_TAPER_CHG_VAL)
		return POWER_SUPPLY_CHARGE_TYPE_TAPER;
	else if (chg_type == BATT_FAST_CHG_VAL)
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	else if (chg_type == BATT_PRE_CHG_VAL)
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;

	return POWER_SUPPLY_CHARGE_TYPE_NONE;
}

static int set_property_on_fg(struct oplus_chg_chip *chip,
		enum power_supply_property prop, int val)
{
	int rc;
	union power_supply_propval ret = {0, };

	if (!chip->pmic_spmi.bms_psy && chip->pmic_spmi.bms_psy_name)
		chip->pmic_spmi.bms_psy =
			power_supply_get_by_name((char *)chip->pmic_spmi.bms_psy_name);
	if (!chip->pmic_spmi.bms_psy) {
		pr_smb(PR_STATUS, "no bms psy found\n");
		return -EINVAL;
	}

	ret.intval = val;
	rc = chip->pmic_spmi.bms_psy->set_property(chip->pmic_spmi.bms_psy, prop, &ret);
	if (rc)
		pr_smb(PR_STATUS,
			"bms psy does not allow updating prop %d rc = %d\n",
			prop, rc);

	return rc;
}

static int get_property_from_fg(struct oplus_chg_chip *chip,
		enum power_supply_property prop, int *val)
{
	int rc;
	union power_supply_propval ret = {0, };

	if (!chip->pmic_spmi.bms_psy && chip->pmic_spmi.bms_psy_name)
		chip->pmic_spmi.bms_psy =
			power_supply_get_by_name((char *)chip->pmic_spmi.bms_psy_name);
	if (!chip->pmic_spmi.bms_psy) {
		pr_smb(PR_STATUS, "no bms psy found\n");
		return -EINVAL;
	}

	rc = chip->pmic_spmi.bms_psy->get_property(chip->pmic_spmi.bms_psy, prop, &ret);
	if (rc) {
		pr_smb(PR_STATUS,
			"bms psy doesn't support reading prop %d rc = %d\n",
			prop, rc);
		return rc;
	}

	*val = ret.intval;
	return rc;
}
#if 0 //deleted by PengNan  12.13
#define DEFAULT_BATT_CAPACITY	50
static int get_prop_batt_capacity(struct oplus_chg_chip *chip)
{
	int capacity, rc;

	if (chip->pmic_spmi.fake_battery_soc >= 0)
		return chip->pmic_spmi.fake_battery_soc;

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_CAPACITY, &capacity);
	if (rc) {
		pr_smb(PR_STATUS, "Couldn't get capacity rc = %d\n", rc);
		capacity = DEFAULT_BATT_CAPACITY;
	}
	return capacity;
}

#define DEFAULT_BATT_TEMP		200
static int get_prop_batt_temp(struct oplus_chg_chip *chip)
{
	int temp, rc;

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_TEMP, &temp);
	if (rc) {
		pr_smb(PR_STATUS, "Couldn't get temperature rc = %d\n", rc);
		temp = DEFAULT_BATT_TEMP;
	}
	return temp;
}

#define DEFAULT_BATT_CURRENT_NOW	0
static int get_prop_batt_current_now(struct oplus_chg_chip *chip)
{
	int ua, rc;

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_CURRENT_NOW, &ua);
	if (rc) {
		pr_smb(PR_STATUS, "Couldn't get current rc = %d\n", rc);
		ua = DEFAULT_BATT_CURRENT_NOW;
	}
	return ua;
}

#define DEFAULT_BATT_VOLTAGE_NOW	0
static int get_prop_batt_voltage_now(struct oplus_chg_chip *chip)
{
	int uv, rc;

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_VOLTAGE_NOW, &uv);
	if (rc) {
		pr_smb(PR_STATUS, "Couldn't get voltage rc = %d\n", rc);
		uv = DEFAULT_BATT_VOLTAGE_NOW;
	}
	return uv;
}
#endif

static int qpnp_get_prop_charger_voltage_now(void)
{
	int rc = 0;
	int V_charger = 0;
	struct qpnp_vadc_result results;	

	if(!the_chip) {
		return 0;
	}
	if(!the_chip->pmic_spmi.vadc_dev) {
		pr_err("%s vadc_dev NULL\n", __func__);
		return 0;
	}
	// board version_B
	rc = qpnp_vadc_read(the_chip->pmic_spmi.vadc_dev, USBIN, &results);
	if (rc) {
		pr_err("Unable to read vchg (USBIN)rc=%d\n", rc);
		return the_chip->charger_volt_pre;
	}
	V_charger = (int)results.physical/1000;
	the_chip->charger_volt_pre = V_charger;
	  	  
	return V_charger;//return (int)results.physical/1000;
	
}


static int qpnp_get_battery_voltage(void)
{
	int rc;
	struct qpnp_vadc_result adc_result;

	if(!the_chip) {
		return 0;
	}
	if(!the_chip->pmic_spmi.vadc_dev) {
		pr_err("%s vadc_dev NULL\n", __func__);
		return 0;
	}
	rc = qpnp_vadc_read(the_chip->pmic_spmi.vadc_dev, VBAT_SNS, &adc_result);
	if (rc) {
		pr_err("error reading adc channel = %d, rc = %d\n",
							VBAT_SNS, rc);
		return 0;
	}
	pr_debug("mvolts phy=%lld meas=0x%llx\n", adc_result.physical,
						adc_result.measurement);
	
	return (int)adc_result.physical/1000;
}

#if 0
#define DEFAULT_BATT_VOLTAGE_MAX_DESIGN	4200000
static int get_prop_batt_voltage_max_design(struct oplus_chg_chip *chip)
{
	int uv, rc;

	rc = get_property_from_fg(chip,
			POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN, &uv);
	if (rc) {
		pr_smb(PR_STATUS, "Couldn't get voltage rc = %d\n", rc);
		uv = DEFAULT_BATT_VOLTAGE_MAX_DESIGN;
	}
	return uv;
}
#endif

static int get_prop_batt_health(struct oplus_chg_chip *chip)
{
	if (chip->pmic_spmi.batt_hot)
		return POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (chip->pmic_spmi.batt_cold)
		return POWER_SUPPLY_HEALTH_COLD;
	else if (chip->pmic_spmi.batt_warm)
		return POWER_SUPPLY_HEALTH_WARM;
	else if (chip->pmic_spmi.batt_cool)
		return POWER_SUPPLY_HEALTH_COOL;
	else
		return POWER_SUPPLY_HEALTH_GOOD;
}

static void get_property_from_typec(struct oplus_chg_chip *chip,
				enum power_supply_property property,
				union power_supply_propval *prop)
{
	int rc;

	rc = chip->pmic_spmi.typec_psy->get_property(chip->pmic_spmi.typec_psy, property, prop);
	if (rc)
		pr_smb(PR_TYPEC,
			"typec psy doesn't support reading prop %d rc = %d\n",
			property, rc);
}

static void update_typec_status(struct oplus_chg_chip *chip)
{
	union power_supply_propval type = {0, };
	union power_supply_propval capability = {0, };
	int rc;

	get_property_from_typec(chip, POWER_SUPPLY_PROP_TYPE, &type);
	if (type.intval != POWER_SUPPLY_TYPE_UNKNOWN) {
		get_property_from_typec(chip,
				POWER_SUPPLY_PROP_CURRENT_CAPABILITY,
				&capability);
		chip->pmic_spmi.typec_current_ma = capability.intval;

		if (!chip->pmic_spmi.skip_usb_notification) {
			rc = chip->usb_psy->set_property(chip->usb_psy,
				POWER_SUPPLY_PROP_INPUT_CURRENT_MAX,
				&capability);
			if (rc)
				pr_err("typec failed to set current max rc=%d\n",
					rc);
			pr_smb(PR_TYPEC, "SMB Type-C mode = %d, current=%d\n",
					type.intval, capability.intval);
		}
	} else {
		pr_smb(PR_TYPEC,
			"typec detection not completed continuing with USB update\n");
	}
}

/*
 * finds the index of the closest value in the array. If there are two that
 * are equally close, the lower index will be returned
 */
static int find_closest_in_array(const int *arr, int len, int val)
{
	int i, closest = 0;

	if (len == 0)
		return closest;
	for (i = 0; i < len; i++)
		if (abs(val - arr[i]) < abs(val - arr[closest]))
			closest = i;

	return closest;
}

/* finds the index of the closest smaller value in the array. */
static int find_smaller_in_array(const int *table, int val, int len)
{
	int i;

	for (i = len - 1; i >= 0; i--) {
		if (val >= table[i])
			break;
	}

	return i;
}

static const int iterm_ma_table_8994[] = {
	300,
	50,
	100,
	150,
	200,
	250,
	500,
	600
};

static const int iterm_ma_table_8996[] = {
	300,
	50,
	100,
	150,
	200,
	250,
	400,
	500
};

static const int usb_ilim_ma_table_8994[] = {
	300,
	400,
	450,
	475,
	500,
	550,
	600,
	650,
	700,
	900,
	950,
	1000,
	1100,
	1200,
	1400,
	1450,
	1500,
	1600,
	1800,
	1850,
	1880,
	1910,
	1930,
	1950,
	1970,
	2000,
	2050,
	2100,
	2300,
	2400,
	2500,
	3000
};

//oplus_usbin_current_ma_table[] must match with oplus_usbin_current_regval_table[]
static const int oplus_usbin_current_ma_table[] = {
	500,
	950,
	1200,
	1500,
	2000,
};

static const int oplus_usbin_current_regval_table[] = {
	0x4,		//500ma,
	0xa,		//950ma,
	0xd,		//1200ma,
	0x10,		//1500ma,
	0x19,		//2000ma,
};

static int find_usb_current_in_oplus_array
			(const int *table, int val)
{
	int i;
	int len = ARRAY_SIZE(oplus_usbin_current_ma_table);
		
	for (i = len - 1; i >= 0; i--) {
		if (val >= table[i])
			break;
	}
	if(i < 0)
		i = 0;
	else if(i > (len - 1)) 
		i = len - 1;
	
	return i;
}

static const int usb_ilim_ma_table_8996[] = {
	300,
	400,
	500,
	600,
	700,
	800,
	900,
	1000,
	1100,
	1200,
	1300,
	1400,
	1450,
	1500,
	1550,
	1600,
	1700,
	1800,
	1900,
	1950,
	2000,
	2050,
	2100,
	2200,
	2300,
	2400,
	2500,
	2600,
	2700,
	2800,
	2900,
	3000
};

static int dc_ilim_ma_table_8994[] = {
	300,
	400,
	450,
	475,
	500,
	550,
	600,
	650,
	700,
	900,
	950,
	1000,
	1100,
	1200,
	1400,
	1450,
	1500,
	1600,
	1800,
	1850,
	1880,
	1910,
	1930,
	1950,
	1970,
	2000,
};

static int dc_ilim_ma_table_8996[] = {
	300,
	400,
	500,
	600,
	700,
	800,
	900,
	1000,
	1100,
	1200,
	1300,
	1400,
	1450,
	1500,
	1550,
	1600,
	1700,
	1800,
	1900,
	1950,
	2000,
	2050,
	2100,
	2200,
	2300,
	2400,
};

static const int fcc_comp_table_8994[] = {
	250,
	700,
	900,
	1200,
};

static const int fcc_comp_table_8996[] = {
	250,
	1100,
	1200,
	1500,
};

static const int aicl_rerun_period[] = {
	45,
	90,
	180,
	360,
};

static const int aicl_rerun_period_schg_lite[] = {
	3,	/* 2.8s  */
	6,	/* 5.6s  */
	11,	/* 11.3s */
	23,	/* 22.5s */
	45,
	90,
	180,
	360,
};

static void use_pmi8994_tables(struct oplus_chg_chip *chip)
{
	chip->pmic_spmi.tables.usb_ilim_ma_table = usb_ilim_ma_table_8994;
	chip->pmic_spmi.tables.usb_ilim_ma_len = ARRAY_SIZE(usb_ilim_ma_table_8994);
	chip->pmic_spmi.tables.dc_ilim_ma_table = dc_ilim_ma_table_8994;
	chip->pmic_spmi.tables.dc_ilim_ma_len = ARRAY_SIZE(dc_ilim_ma_table_8994);
	chip->pmic_spmi.tables.iterm_ma_table = iterm_ma_table_8994;
	chip->pmic_spmi.tables.iterm_ma_len = ARRAY_SIZE(iterm_ma_table_8994);
	chip->pmic_spmi.tables.fcc_comp_table = fcc_comp_table_8994;
	chip->pmic_spmi.tables.fcc_comp_len = ARRAY_SIZE(fcc_comp_table_8994);
	chip->pmic_spmi.tables.rchg_thr_mv = 200;
	chip->pmic_spmi.tables.aicl_rerun_period_table = aicl_rerun_period;
	chip->pmic_spmi.tables.aicl_rerun_period_len = ARRAY_SIZE(aicl_rerun_period);
}

static void use_pmi8996_tables(struct oplus_chg_chip *chip)
{
	chip->pmic_spmi.tables.usb_ilim_ma_table = usb_ilim_ma_table_8996;
	chip->pmic_spmi.tables.usb_ilim_ma_len = ARRAY_SIZE(usb_ilim_ma_table_8996);
	chip->pmic_spmi.tables.dc_ilim_ma_table = dc_ilim_ma_table_8996;
	chip->pmic_spmi.tables.dc_ilim_ma_len = ARRAY_SIZE(dc_ilim_ma_table_8996);
	chip->pmic_spmi.tables.iterm_ma_table = iterm_ma_table_8996;
	chip->pmic_spmi.tables.iterm_ma_len = ARRAY_SIZE(iterm_ma_table_8996);
	chip->pmic_spmi.tables.fcc_comp_table = fcc_comp_table_8996;
	chip->pmic_spmi.tables.fcc_comp_len = ARRAY_SIZE(fcc_comp_table_8996);
	chip->pmic_spmi.tables.rchg_thr_mv = 150;
	chip->pmic_spmi.tables.aicl_rerun_period_table = aicl_rerun_period;
	chip->pmic_spmi.tables.aicl_rerun_period_len = ARRAY_SIZE(aicl_rerun_period);
}

#define CMD_CHG_REG	0x42
#define EN_BAT_CHG_BIT		BIT(1)
static int smbchg_charging_en(struct oplus_chg_chip *chip, bool en)
{
	chg_err("en:%d\n", en);
	/* The en bit is configured active low */
	return smbchg_masked_write(chip, chip->pmic_spmi.bat_if_base + CMD_CHG_REG,
			EN_BAT_CHG_BIT, en ? 0 : EN_BAT_CHG_BIT);
}

#ifdef OPLUS_FEATURE_CHG_BASIC
static int smbchg_charging_enble(struct oplus_chg_chip *chip)
{
	return smbchg_charging_en(chip, true);
}

static int smbchg_charging_disble(struct oplus_chg_chip *chip)
{
	return smbchg_charging_en(chip, false);
}

static int smbchg_get_charge_enable(struct oplus_chg_chip *chip)
{
	int rc = 0;
	u8 reg = 0;
	
	rc = smbchg_read(chip, &reg, chip->pmic_spmi.bat_if_base + CMD_CHG_REG, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Unable to read RT_STS rc = %d\n", rc);
		return 0;
	}
	if(reg & EN_BAT_CHG_BIT)
		return 0;
	else
		return 1;
}


int qpnp_fg_set_charge_enble(bool enable)
{	
	if(!the_chip)
		return -EINVAL;
	return smbchg_charging_en(the_chip, enable);
}

#endif

#define CMD_IL			0x40
#define USBIN_SUSPEND_BIT	BIT(4)
#define CURRENT_100_MA		100
#define CURRENT_150_MA		150
#define CURRENT_500_MA		500
#define CURRENT_900_MA		900
#define CURRENT_950_MA		950
#define CURRENT_1500_MA		1500
#define SUSPEND_CURRENT_MA	2
#define ICL_OVERRIDE_BIT	BIT(2)

#ifndef OPLUS_FEATURE_CHG_BASIC	//Fuchun.Liao 2016/06/17 modify
static int smbchg_usb_suspend(struct oplus_chg_chip *chip, bool suspend)
{
	int rc;

	rc = smbchg_masked_write(chip, chip->pmic_spmi.usb_chgpth_base + CMD_IL,
			USBIN_SUSPEND_BIT, suspend ? USBIN_SUSPEND_BIT : 0);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set usb suspend rc = %d\n", rc);
	return rc;
}
#else
static int smbchg_usb_suspend(struct oplus_chg_chip *chip, bool suspend)
{
	int rc;
	bool suspend_to_set = false;
	int boot_mode = get_boot_mode();

	if((boot_mode == MSM_BOOT_MODE__RF) || (boot_mode == MSM_BOOT_MODE__WLAN)){
		suspend_to_set = true;
	} else {
		suspend_to_set = suspend;
	}
	chg_err("suspend:%d, suspend_to_set:%d\n", suspend, suspend_to_set);
	rc = smbchg_masked_write(chip, chip->pmic_spmi.usb_chgpth_base + CMD_IL,
			USBIN_SUSPEND_BIT, suspend_to_set ? USBIN_SUSPEND_BIT : 0);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set usb suspend rc = %d\n", rc);
	return rc;
}
#endif /* OPLUS_FEATURE_CHG_BASIC */

static int smbchg_usb_suspend_enable(struct oplus_chg_chip *chip)
{
	return smbchg_usb_suspend(chip, true);
}

static int smbchg_usb_suspend_disable(struct oplus_chg_chip *chip)
{
	return smbchg_usb_suspend(chip, false);
}

#define DCIN_SUSPEND_BIT	BIT(3)
static int smbchg_dc_suspend(struct oplus_chg_chip *chip, bool suspend)
{
	int rc = 0;

	rc = smbchg_masked_write(chip, chip->pmic_spmi.usb_chgpth_base + CMD_IL,
			DCIN_SUSPEND_BIT, suspend ? DCIN_SUSPEND_BIT : 0);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set dc suspend rc = %d\n", rc);
	return rc;
}

#define IL_CFG			0xF2
#define DCIN_INPUT_MASK	SMB_MASK(4, 0)
static int smbchg_set_dc_current_max(struct oplus_chg_chip *chip, int current_ma)
{
	int i;
	u8 dc_cur_val;

	i = find_smaller_in_array(chip->pmic_spmi.tables.dc_ilim_ma_table,
			current_ma, chip->pmic_spmi.tables.dc_ilim_ma_len);

	if (i < 0) {
		dev_err(chip->dev, "Cannot find %dma current_table\n",
				current_ma);
		return -EINVAL;
	}

	chip->pmic_spmi.dc_max_current_ma = chip->pmic_spmi.tables.dc_ilim_ma_table[i];
	dc_cur_val = i & DCIN_INPUT_MASK;

	pr_smb(PR_STATUS, "dc current set to %d mA\n",
			chip->pmic_spmi.dc_max_current_ma);
	return smbchg_sec_masked_write(chip, chip->pmic_spmi.dc_chgpth_base + IL_CFG,
				DCIN_INPUT_MASK, dc_cur_val);
}

#define AICL_WL_SEL_CFG			0xF5
#define AICL_WL_SEL_MASK		SMB_MASK(1, 0)
#define AICL_WL_SEL_SCHG_LITE_MASK	SMB_MASK(2, 0)
static int smbchg_set_aicl_rerun_period_s(struct oplus_chg_chip *chip,
								int period_s)
{
	int i;
	u8 reg, mask;

	i = find_smaller_in_array(chip->pmic_spmi.tables.aicl_rerun_period_table,
			period_s, chip->pmic_spmi.tables.aicl_rerun_period_len);

	if (i < 0) {
		dev_err(chip->dev, "Cannot find %ds in aicl rerun period\n",
				period_s);
		return -EINVAL;
	}

	if (chip->pmic_spmi.schg_version == QPNP_SCHG_LITE)
		mask = AICL_WL_SEL_SCHG_LITE_MASK;
	else
		mask = AICL_WL_SEL_MASK;

	reg = i & mask;

	pr_smb(PR_STATUS, "aicl rerun period set to %ds\n",
			chip->pmic_spmi.tables.aicl_rerun_period_table[i]);
	return smbchg_sec_masked_write(chip,
			chip->pmic_spmi.dc_chgpth_base + AICL_WL_SEL_CFG,
			mask, reg);
}

static struct power_supply *get_parallel_psy(struct oplus_chg_chip *chip)
{
	if (!chip->pmic_spmi.parallel.avail)
		return NULL;
	if (chip->pmic_spmi.parallel.psy)
		return chip->pmic_spmi.parallel.psy;
	chip->pmic_spmi.parallel.psy = power_supply_get_by_name("usb-parallel");
	if (!chip->pmic_spmi.parallel.psy)
		pr_smb(PR_STATUS, "parallel charger not found\n");
	return chip->pmic_spmi.parallel.psy;
}

static void smbchg_usb_update_online_work(struct work_struct *work)
{
	struct qcom_pmic *pmic_chip = container_of(work,
				struct qcom_pmic,
				usb_set_online_work);
	struct oplus_chg_chip *chip = container_of(pmic_chip,
				struct oplus_chg_chip,
				pmic_spmi);
	bool user_enabled = !get_client_vote(chip->pmic_spmi.usb_suspend_votable,
						USER_EN_VOTER);
	int online;

	online = user_enabled && chip->pmic_spmi.usb_present && !chip->pmic_spmi.very_weak_charger;

	mutex_lock(&chip->pmic_spmi.usb_set_online_lock);
	if (chip->pmic_spmi.usb_online != online) {
		pr_smb(PR_MISC, "setting usb psy online = %d\n", online);
		power_supply_set_online(chip->usb_psy, online);
		chip->pmic_spmi.usb_online = online;
	}
	mutex_unlock(&chip->pmic_spmi.usb_set_online_lock);
}

#define CHGPTH_CFG		0xF4
#define CFG_USB_2_3_SEL_BIT	BIT(7)
#define CFG_USB_2		0
#define CFG_USB_3		BIT(7)
#define USBIN_INPUT_MASK	SMB_MASK(4, 0)
#define USBIN_MODE_CHG_BIT	BIT(0)
#define USBIN_LIMITED_MODE	0
#define USBIN_HC_MODE		BIT(0)
#define USB51_MODE_BIT		BIT(1)
#define USB51_100MA		0
#define USB51_500MA		BIT(1)
#if 0
static int smbchg_set_high_usb_chg_current(struct oplus_chg_chip *chip,
							int current_ma)
{
	int i, rc;
	u8 usb_cur_val;

	if (current_ma == CURRENT_100_MA) {
		rc = smbchg_sec_masked_write(chip,
					chip->pmic_spmi.usb_chgpth_base + CHGPTH_CFG,
					CFG_USB_2_3_SEL_BIT, CFG_USB_2);
		if (rc < 0) {
			pr_err("Couldn't set CFG_USB_2 rc=%d\n", rc);
			return rc;
		}

		rc = smbchg_masked_write(chip, chip->pmic_spmi.usb_chgpth_base + CMD_IL,
			USBIN_MODE_CHG_BIT | USB51_MODE_BIT | ICL_OVERRIDE_BIT,
			USBIN_LIMITED_MODE | USB51_100MA | ICL_OVERRIDE_BIT);
		if (rc < 0) {
			pr_err("Couldn't set ICL_OVERRIDE rc=%d\n", rc);
			return rc;
		}

		pr_smb(PR_STATUS,
			"Forcing 100mA current limit\n");
		chip->pmic_spmi.usb_max_current_ma = CURRENT_100_MA;
		return rc;
	}

	i = find_smaller_in_array(chip->pmic_spmi.tables.usb_ilim_ma_table,
			current_ma, chip->pmic_spmi.tables.usb_ilim_ma_len);
	if (i < 0) {
		dev_err(chip->dev,
			"Cannot find %dma current_table using %d\n",
			current_ma, CURRENT_150_MA);

		rc = smbchg_sec_masked_write(chip,
					chip->pmic_spmi.usb_chgpth_base + CHGPTH_CFG,
					CFG_USB_2_3_SEL_BIT, CFG_USB_3);
		rc |= smbchg_masked_write(chip, chip->pmic_spmi.usb_chgpth_base + CMD_IL,
					USBIN_MODE_CHG_BIT | USB51_MODE_BIT,
					USBIN_LIMITED_MODE | USB51_100MA);
		if (rc < 0)
			dev_err(chip->dev, "Couldn't set %dmA rc=%d\n",
					CURRENT_150_MA, rc);
		else
			chip->pmic_spmi.usb_max_current_ma = 150;
		return rc;
	}

	usb_cur_val = i & USBIN_INPUT_MASK;
	rc = smbchg_sec_masked_write(chip, chip->pmic_spmi.usb_chgpth_base + IL_CFG,
				USBIN_INPUT_MASK, usb_cur_val);
	if (rc < 0) {
		dev_err(chip->dev, "cannot write to config c rc = %d\n", rc);
		return rc;
	}

#if 0
	rc = smbchg_masked_write(chip, chip->pmic_spmi.usb_chgpth_base + CMD_IL,
				USBIN_MODE_CHG_BIT, USBIN_HC_MODE);
#else
	rc = smbchg_masked_write(chip, chip->pmic_spmi.usb_chgpth_base + CMD_IL,
				USBIN_MODE_CHG_BIT|ICL_OVERRIDE_BIT, USBIN_HC_MODE|ICL_OVERRIDE_BIT);
#endif
	if (rc < 0)
		dev_err(chip->dev, "Couldn't write cfg 5 rc = %d\n", rc);
	chip->pmic_spmi.usb_max_current_ma = chip->pmic_spmi.tables.usb_ilim_ma_table[i];
	chg_err("current_ma = %d, usb_cur_val = 0x%x, max_current_ma = %d\n", 
		current_ma, usb_cur_val, chip->pmic_spmi.usb_max_current_ma);
	return rc;
}
#else

static void smbchg_rerun_aicl(struct oplus_chg_chip *chip);
static void smbchg_aicl_enable(struct oplus_chg_chip *chip, bool enable);

static void smbchg_write_usb_input_current_reg
				(struct oplus_chg_chip *chip, int reg_ma)
{
	int rc = 0, usb_cur_val;

	usb_cur_val = reg_ma & USBIN_INPUT_MASK;
	rc = smbchg_sec_masked_write(chip, chip->pmic_spmi.usb_chgpth_base + IL_CFG,
				USBIN_INPUT_MASK, usb_cur_val);
	if (rc < 0) {
		dev_err(chip->dev, "cannot write usb_chg_current reg = %d\n", rc);
		return;
	}
	rc = smbchg_masked_write(chip, chip->pmic_spmi.usb_chgpth_base + CMD_IL,
				USBIN_MODE_CHG_BIT|ICL_OVERRIDE_BIT, USBIN_HC_MODE|ICL_OVERRIDE_BIT);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't write cfg 5 rc = %d\n", rc);
	}
	//smbchg_rerun_aicl(chip);
	chg_err("reg_ma:0x%x\n", reg_ma);
}

static bool smbchg_check_soft_aicl_pass_ornot(struct oplus_chg_chip *chip, 
								int reg_ma)
{
	int chg_vol = 0;
	
	smbchg_write_usb_input_current_reg(chip ,reg_ma);
	msleep(80);
	chg_vol = qpnp_get_prop_charger_voltage_now();
	if(chg_vol < 4520 && chg_vol > 0) {
		chg_err("aicl end reg_ma:0x%x, chg_vol:%d\n", reg_ma, chg_vol);
		return false;
	} else {
		return true;
	}
}
#ifdef CONFIG_OPLUS_SHORT_C_BATT_CHECK
/* This function is getting the dynamic aicl result/input limited in mA.
 * If charger was suspended, it must return 0(mA).
 * It meets the requirements in SDM660 platform.
 */
#define USB_ICL_STS_1 0x07
#define USB_ICL_STS_2 0x09
#define USB_ICL_STS_MASK (BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0))
#define USBIN_SUSPEND_STS_MASK BIT(3)
#define DEFAULT_CURRENT 1000    /* abnormal status return 1A */
static int oplus_chg_get_dyna_aicl_result(struct oplus_chg_chip *chip)
{
	int rc = 0, array_num = 0, usb_icl_ma = 0;
	u8 usb_icl_reg = 0;
	u8 usb_suspend_reg = 0;
	int usb_suspend_sts = 0;

	rc = smbchg_read(chip, &usb_suspend_reg, chip->pmic_spmi.usb_chgpth_base + USB_ICL_STS_2, 1);
	if (rc < 0) {
		chg_err("Unable to read ICL_STS rc = %d\n", rc);
		return DEFAULT_CURRENT;
	} else {
		usb_suspend_sts = (usb_suspend_reg & USBIN_SUSPEND_STS_MASK) >> 3;
		if (usb_suspend_sts) {
			chg_debug("Chg suspend Reg = 0x%02x, STS = %d\n", usb_suspend_reg, usb_suspend_sts);
			return 0;
		} 
	}
	rc = smbchg_read(chip, &usb_icl_reg, chip->pmic_spmi.usb_chgpth_base + USB_ICL_STS_1, 1);
	if (rc < 0) {
		chg_err("Unable to read ICL_STS rc = %d\n", rc);
		return DEFAULT_CURRENT;
	}
	array_num = usb_icl_reg & USB_ICL_STS_MASK;
	if (array_num >= 0 && array_num <= 31) {
		usb_icl_ma = usb_ilim_ma_table_8994[array_num];
		chg_err("usb_icl_reg is 0x%02x, usb_icl_ma = %d mA\n", usb_icl_reg, usb_icl_ma);
		return usb_icl_ma;
	} else {
		chg_err("array_num is out of range => %d\n", array_num);
		return DEFAULT_CURRENT;
	}
}
#endif /* CONFIG_OPLUS_SHORT_C_BATT_CHECK */

extern int oplus_gauge_get_batt_temperature(void);		// add for avoid aicl when temp < 5 degree
static int smbchg_set_high_usb_chg_current(struct oplus_chg_chip *chip,
							int current_ma)
{
	int rc = 0, i, icl_ma, aicl_max_ma;
	bool aicl_pass = false;
	int temp = oplus_gauge_get_batt_temperature();

	if (current_ma == CURRENT_100_MA) {
		rc = smbchg_sec_masked_write(chip,
					chip->pmic_spmi.usb_chgpth_base + CHGPTH_CFG,
					CFG_USB_2_3_SEL_BIT, CFG_USB_2);
		if (rc < 0) {
			pr_err("Couldn't set CFG_USB_2 rc=%d\n", rc);
			return rc;
		}

		rc = smbchg_masked_write(chip, chip->pmic_spmi.usb_chgpth_base + CMD_IL,
			USBIN_MODE_CHG_BIT | USB51_MODE_BIT | ICL_OVERRIDE_BIT,
			USBIN_LIMITED_MODE | USB51_100MA | ICL_OVERRIDE_BIT);
		if (rc < 0) {
			pr_err("Couldn't set ICL_OVERRIDE rc=%d\n", rc);
			return rc;
		}

		pr_smb(PR_STATUS,
			"Forcing 100mA current limit\n");
		chip->pmic_spmi.usb_max_current_ma = CURRENT_100_MA;
		return rc;
	}

	i = find_smaller_in_array(chip->pmic_spmi.tables.usb_ilim_ma_table,
			current_ma, chip->pmic_spmi.tables.usb_ilim_ma_len);
	if (i < 0) {
		dev_err(chip->dev,
			"Cannot find %dma current_table using %d\n",
			current_ma, CURRENT_150_MA);

		rc = smbchg_sec_masked_write(chip,
					chip->pmic_spmi.usb_chgpth_base + CHGPTH_CFG,
					CFG_USB_2_3_SEL_BIT, CFG_USB_3);
		rc |= smbchg_masked_write(chip, chip->pmic_spmi.usb_chgpth_base + CMD_IL,
					USBIN_MODE_CHG_BIT | USB51_MODE_BIT,
					USBIN_LIMITED_MODE | USB51_100MA);
		if (rc < 0)
			dev_err(chip->dev, "Couldn't set %dmA rc=%d\n",
					CURRENT_150_MA, rc);
		else
			chip->pmic_spmi.usb_max_current_ma = 150;
		return rc;
	}
	
	smbchg_aicl_enable(chip, false);

	if(chip->pmic_spmi.not_support_1200ma && (chip->pmic_spmi.usb_supply_type != POWER_SUPPLY_TYPE_USB)) {
		if(temp <= 50) {
			/* when temp is < 5 degree, do not aicl and set input current max = 900mA */
			smbchg_write_usb_input_current_reg(chip, oplus_usbin_current_regval_table[1]);
			smbchg_rerun_aicl(chip);
			chip->pmic_spmi.usb_max_current_ma = oplus_usbin_current_ma_table[1];
			chg_err("Low Temp,No aicl and Input_Current_limit = %d (0x%02x), rc = %d\n",
				chip->pmic_spmi.usb_max_current_ma,oplus_usbin_current_regval_table[1],rc);
			return rc;
		} 
	}
	
	icl_ma = find_usb_current_in_oplus_array(oplus_usbin_current_ma_table, current_ma);
	if(icl_ma > 0x3) {			// > 1500ma, aicl to 2000ma
		aicl_max_ma = 0x4;		// 2000ma
	} else if(icl_ma > 0) {		// 950ma/1200ma/1500ma, aicl to 1500ma
		aicl_max_ma = 0x3;		// 1500ma
	} else {
		aicl_max_ma = 0x0;		// 500ma
	}
	/* aicl begin */
	for(i = 0; i <= aicl_max_ma; i++) {
		aicl_pass = smbchg_check_soft_aicl_pass_ornot(chip, oplus_usbin_current_regval_table[i]);
		if(aicl_pass == false) {
			if(i > 0){
				i--;
			}
			smbchg_write_usb_input_current_reg(chip, oplus_usbin_current_regval_table[i]);
			break;
		}
	}
	if(i == 2) {	// skip 1200ma
		i--;
	}
	/* aicl end, aicl result is i */
	if(i > icl_ma) {
		i = icl_ma;
	}
	smbchg_write_usb_input_current_reg(chip, oplus_usbin_current_regval_table[i]);
	smbchg_rerun_aicl(chip);
	chip->pmic_spmi.usb_max_current_ma = oplus_usbin_current_ma_table[i];
	chg_err("current_ma = %d, reg_ma = 0x%x, max_current_ma = %d\n",
		current_ma, oplus_usbin_current_regval_table[i], chip->pmic_spmi.usb_max_current_ma);
	return rc;
}

#endif

/* if APSD results are used
 *	if SDP is detected it will look at 500mA setting
 *		if set it will draw 500mA
 *		if unset it will draw 100mA
 *	if CDP/DCP it will look at 0x0C setting
 *		i.e. values in 0x41[1, 0] does not matter
 */
static int smbchg_set_usb_current_max(struct oplus_chg_chip *chip,
							int current_ma)
{
	int rc = 0;

	/*
	 * if the battery is not present, do not allow the usb ICL to lower in
	 * order to avoid browning out the device during a hotswap.
	 */
	if (!chip->pmic_spmi.batt_present && current_ma < chip->pmic_spmi.usb_max_current_ma) {
		pr_info_ratelimited("Ignoring usb current->%d, battery is absent\n",
				current_ma);
		return 0;
	}
	pr_smb(PR_STATUS, "USB current_ma = %d\n", current_ma);

	if (current_ma <= SUSPEND_CURRENT_MA) {
		/* suspend the usb if current <= 2mA */
//		rc = vote(chip->pmic_spmi.usb_suspend_votable, USB_EN_VOTER, true, 0);
//		chip->pmic_spmi.usb_max_current_ma = 0;
		chg_err("Avoid to set usb suspend for 2mA\n");
		goto out;
	} else {
		rc = vote(chip->pmic_spmi.usb_suspend_votable, USB_EN_VOTER, false, 0);
	}

	switch (chip->pmic_spmi.usb_supply_type) {
	case POWER_SUPPLY_TYPE_USB:
		if ((current_ma < CURRENT_150_MA) &&
				(chip->pmic_spmi.wa_flags & SMBCHG_USB100_WA))
			current_ma = CURRENT_150_MA;

		/* handle special SDP case when USB reports high current */
		if (current_ma > CURRENT_900_MA) {
			if (chip->pmic_spmi.cfg_override_usb_current) {
				/*
				 * allow setting the current value as reported
				 * by USB driver.
				 */
				rc = smbchg_set_high_usb_chg_current(chip,
							current_ma);
				if (rc < 0) {
					pr_err("Couldn't set %dmA rc = %d\n",
							current_ma, rc);
					goto out;
				}
				rc = smbchg_masked_write(chip,
					chip->pmic_spmi.usb_chgpth_base + CMD_IL,
					ICL_OVERRIDE_BIT, ICL_OVERRIDE_BIT);
				if (rc < 0)
					pr_err("Couldn't set ICL override rc = %d\n",
							rc);
			} else {
				/* default to 500mA */
				current_ma = CURRENT_500_MA;
			}
			pr_smb(PR_STATUS,
				"override_usb_current=%d current_ma set to %d\n",
				chip->pmic_spmi.cfg_override_usb_current, current_ma);
		}

		if (current_ma < CURRENT_150_MA) {
			/* force 100mA */
			rc = smbchg_sec_masked_write(chip,
					chip->pmic_spmi.usb_chgpth_base + CHGPTH_CFG,
					CFG_USB_2_3_SEL_BIT, CFG_USB_2);
			if (rc < 0) {
				pr_err("Couldn't set CHGPTH_CFG rc = %d\n", rc);
				goto out;
			}
			rc = smbchg_masked_write(chip,
					chip->pmic_spmi.usb_chgpth_base + CMD_IL,
					USBIN_MODE_CHG_BIT | USB51_MODE_BIT,
					USBIN_LIMITED_MODE | USB51_100MA);
			if (rc < 0) {
				pr_err("Couldn't set CMD_IL rc = %d\n", rc);
				goto out;
			}
			chip->pmic_spmi.usb_max_current_ma = 100;
		}
		/* specific current values */
		if (current_ma == CURRENT_150_MA) {
			rc = smbchg_sec_masked_write(chip,
					chip->pmic_spmi.usb_chgpth_base + CHGPTH_CFG,
					CFG_USB_2_3_SEL_BIT, CFG_USB_3);
			if (rc < 0) {
				pr_err("Couldn't set CHGPTH_CFG rc = %d\n", rc);
				goto out;
			}
			rc = smbchg_masked_write(chip,
					chip->pmic_spmi.usb_chgpth_base + CMD_IL,
					USBIN_MODE_CHG_BIT | USB51_MODE_BIT,
					USBIN_LIMITED_MODE | USB51_100MA);
			if (rc < 0) {
				pr_err("Couldn't set CMD_IL rc = %d\n", rc);
				goto out;
			}
			chip->pmic_spmi.usb_max_current_ma = 150;
		}
		if (current_ma == CURRENT_500_MA) {
			/*
			 * allow setting the current value as reported
			 * by USB driver.
			 */
			if (chip->pmic_spmi.not_support_1200ma) {
				/* For 8940 */
				if (chip->pmic_spmi.id_value_en) {
					/* Otg_switch on, Use USB51_MODE */
					rc = smbchg_sec_masked_write(chip,
							chip->pmic_spmi.usb_chgpth_base + CHGPTH_CFG,
							CFG_USB_2_3_SEL_BIT, CFG_USB_2);
					if (rc < 0) {
						pr_err("Couldn't set CHGPTH_CFG rc = %d\n", rc);
						goto out;
					}
					rc = smbchg_masked_write(chip,
							chip->pmic_spmi.usb_chgpth_base + CMD_IL,
							USBIN_MODE_CHG_BIT | USB51_MODE_BIT,
							USBIN_LIMITED_MODE | USB51_500MA);
					if (rc < 0) {
						pr_err("Couldn't set CMD_IL rc = %d\n", rc);
						goto out;
					}
				} else {
				/* Otg_switch off, Use HC500_MODE */
					rc = smbchg_set_high_usb_chg_current(chip,
									current_ma);
					if (rc < 0) {
						pr_err("Couldn't set %dmA rc = %d\n",
								current_ma, rc);
						goto out;
					}
					rc = smbchg_masked_write(chip,
						chip->pmic_spmi.usb_chgpth_base + CMD_IL,
						ICL_OVERRIDE_BIT, ICL_OVERRIDE_BIT);
					if (rc < 0)
						pr_err("Couldn't set ICL override rc = %d\n",
								rc);

					chg_err("override_usb_current=%d current_ma set to %d\n",
						chip->pmic_spmi.cfg_override_usb_current, current_ma);
				}
			} else {
				/* For 8953 USE HC500_MODE */
				rc = smbchg_set_high_usb_chg_current(chip,
								current_ma);
				if (rc < 0) {
					pr_err("Couldn't set %dmA rc = %d\n",
							current_ma, rc);
					goto out;
				}
				rc = smbchg_masked_write(chip,
					chip->pmic_spmi.usb_chgpth_base + CMD_IL,
					ICL_OVERRIDE_BIT, ICL_OVERRIDE_BIT);
				if (rc < 0)
					pr_err("Couldn't set ICL override rc = %d\n",
							rc);

				chg_err("override_usb_current=%d current_ma set to %d\n",
					chip->pmic_spmi.cfg_override_usb_current, current_ma);
			}
			chip->pmic_spmi.usb_max_current_ma = 500;
		}
		if (current_ma == CURRENT_900_MA) {
			rc = smbchg_sec_masked_write(chip,
					chip->pmic_spmi.usb_chgpth_base + CHGPTH_CFG,
					CFG_USB_2_3_SEL_BIT, CFG_USB_3);
			if (rc < 0) {
				pr_err("Couldn't set CHGPTH_CFG rc = %d\n", rc);
				goto out;
			}
			rc = smbchg_masked_write(chip,
					chip->pmic_spmi.usb_chgpth_base + CMD_IL,
					USBIN_MODE_CHG_BIT | USB51_MODE_BIT,
					USBIN_LIMITED_MODE | USB51_500MA);
			if (rc < 0) {
				pr_err("Couldn't set CMD_IL rc = %d\n", rc);
				goto out;
			}
			chip->pmic_spmi.usb_max_current_ma = 900;
		}
		break;
	case POWER_SUPPLY_TYPE_USB_CDP:
		if (current_ma < CURRENT_1500_MA) {
			/* use override for CDP */
			rc = smbchg_masked_write(chip,
					chip->pmic_spmi.usb_chgpth_base + CMD_IL,
					ICL_OVERRIDE_BIT, ICL_OVERRIDE_BIT);
			if (rc < 0)
				pr_err("Couldn't set override rc = %d\n", rc);
		}
		if(current_ma > CURRENT_950_MA) {
			current_ma = CURRENT_950_MA;
		}
		/* fall through */
	default:
		rc = smbchg_set_high_usb_chg_current(chip, current_ma);
		if (rc < 0)
			pr_err("Couldn't set %dmA rc = %d\n", current_ma, rc);
		break;
	}

out:
	chg_err("usb type = %d current set to %d mA, current_ma %d mA\n",
			chip->pmic_spmi.usb_supply_type, chip->pmic_spmi.usb_max_current_ma, current_ma);
	return rc;
}

#define USBIN_HVDCP_STS				0x0C
#define USBIN_HVDCP_SEL_BIT			BIT(4)
#define USBIN_HVDCP_SEL_9V_BIT			BIT(1)
#define SCHG_LITE_USBIN_HVDCP_SEL_9V_BIT	BIT(2)
#define SCHG_LITE_USBIN_HVDCP_SEL_BIT		BIT(0)
static int smbchg_get_min_parallel_current_ma(struct oplus_chg_chip *chip)
{
	int rc;
	u8 reg, hvdcp_sel, hvdcp_sel_9v;

	rc = smbchg_read(chip, &reg,
			chip->pmic_spmi.usb_chgpth_base + USBIN_HVDCP_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb status rc = %d\n", rc);
		return 0;
	}
	if (chip->pmic_spmi.schg_version == QPNP_SCHG_LITE) {
		hvdcp_sel = SCHG_LITE_USBIN_HVDCP_SEL_BIT;
		hvdcp_sel_9v = SCHG_LITE_USBIN_HVDCP_SEL_9V_BIT;
	} else {
		hvdcp_sel = USBIN_HVDCP_SEL_BIT;
		hvdcp_sel_9v = USBIN_HVDCP_SEL_9V_BIT;
	}

	if ((reg & hvdcp_sel) && (reg & hvdcp_sel_9v))
		return chip->pmic_spmi.parallel.min_9v_current_thr_ma;
	return chip->pmic_spmi.parallel.min_current_thr_ma;
}

static bool is_hvdcp_present(struct oplus_chg_chip *chip)
{
	int rc;
	u8 reg, hvdcp_sel;

	rc = smbchg_read(chip, &reg,
			chip->pmic_spmi.usb_chgpth_base + USBIN_HVDCP_STS, 1);
	if (rc < 0) {
		pr_err("Couldn't read hvdcp status rc = %d\n", rc);
		return false;
	}

	pr_smb(PR_STATUS, "HVDCP_STS = 0x%02x\n", reg);
	/*
	 * If a valid HVDCP is detected, notify it to the usb_psy only
	 * if USB is still present.
	 */
	if (chip->pmic_spmi.schg_version == QPNP_SCHG_LITE)
		hvdcp_sel = SCHG_LITE_USBIN_HVDCP_SEL_BIT;
	else
		hvdcp_sel = USBIN_HVDCP_SEL_BIT;

	if ((reg & hvdcp_sel) && is_usb_present(chip))
		return true;

	return false;
}

#define FCC_CFG			0xF2
#define FCC_500MA_VAL		0x4
#define FCC_MASK		SMB_MASK(4, 0)
static int smbchg_set_fastchg_current_raw(struct oplus_chg_chip *chip,
							int current_ma)
{
	int i, rc;
	u8 cur_val;

	/* the fcc enumerations are the same as the usb currents */
	i = find_smaller_in_array(chip->pmic_spmi.tables.usb_ilim_ma_table,
			current_ma, chip->pmic_spmi.tables.usb_ilim_ma_len);
	if (i < 0) {
		dev_err(chip->dev,
			"Cannot find %dma current_table using %d\n",
			current_ma, CURRENT_500_MA);

		rc = smbchg_sec_masked_write(chip, chip->pmic_spmi.chgr_base + FCC_CFG,
					FCC_MASK,
					FCC_500MA_VAL);
		if (rc < 0)
			dev_err(chip->dev, "Couldn't set %dmA rc=%d\n",
					CURRENT_500_MA, rc);
		else
			chip->pmic_spmi.fastchg_current_ma = 500;
		return rc;
	}

	if (chip->pmic_spmi.tables.usb_ilim_ma_table[i] == chip->pmic_spmi.fastchg_current_ma) {
		pr_smb(PR_STATUS, "skipping fastchg current request: %d\n",
				chip->pmic_spmi.fastchg_current_ma);
		return 0;
	}

	cur_val = i & FCC_MASK;
	rc = smbchg_sec_masked_write(chip, chip->pmic_spmi.chgr_base + FCC_CFG,
				FCC_MASK, cur_val);
	if (rc < 0) {
		dev_err(chip->dev, "cannot write to fcc cfg rc = %d\n", rc);
		return rc;
	}
	chg_err("fastcharge current requested %d, set to %d\n",
			current_ma, chip->pmic_spmi.tables.usb_ilim_ma_table[cur_val]);

	chip->pmic_spmi.fastchg_current_ma = chip->pmic_spmi.tables.usb_ilim_ma_table[cur_val];
	return rc;
}

#define ICL_STS_1_REG			0x7
#define ICL_STS_2_REG			0x9
#define ICL_STS_MASK			0x1F
#define AICL_SUSP_BIT			BIT(6)
#define AICL_STS_BIT			BIT(5)
#define USBIN_SUSPEND_STS_BIT		BIT(3)
#define USBIN_ACTIVE_PWR_SRC_BIT	BIT(1)
#define DCIN_ACTIVE_PWR_SRC_BIT		BIT(0)
#define PARALLEL_REENABLE_TIMER_MS	1000
#define PARALLEL_CHG_THRESHOLD_CURRENT	1800
static bool smbchg_is_usbin_active_pwr_src(struct oplus_chg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg,
			chip->pmic_spmi.usb_chgpth_base + ICL_STS_2_REG, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Could not read usb icl sts 2: %d\n", rc);
		return false;
	}

	return !(reg & USBIN_SUSPEND_STS_BIT)
		&& (reg & USBIN_ACTIVE_PWR_SRC_BIT);
}

static void smbchg_detect_parallel_charger(struct oplus_chg_chip *chip)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);

	if (parallel_psy)
		chip->pmic_spmi.parallel_charger_detected =
			power_supply_set_present(parallel_psy, true) ?
								false : true;
}

static int smbchg_parallel_usb_charging_en(struct oplus_chg_chip *chip, bool en)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);
	union power_supply_propval pval = {0, };

	if (!parallel_psy || !chip->pmic_spmi.parallel_charger_detected)
		return 0;

	pval.intval = en;
	return parallel_psy->set_property(parallel_psy,
		POWER_SUPPLY_PROP_CHARGING_ENABLED, &pval);
}

#define ESR_PULSE_CURRENT_DELTA_MA	200
static int smbchg_sw_esr_pulse_en(struct oplus_chg_chip *chip, bool en)
{
	int rc, fg_current_now, icl_ma;

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_CURRENT_NOW,
						&fg_current_now);
	if (rc) {
		pr_smb(PR_STATUS, "bms psy does not support OCV\n");
		return 0;
	}

	icl_ma = max(chip->pmic_spmi.iterm_ma + ESR_PULSE_CURRENT_DELTA_MA,
				fg_current_now - ESR_PULSE_CURRENT_DELTA_MA);
	rc = vote(chip->pmic_spmi.fcc_votable, ESR_PULSE_FCC_VOTER, en, icl_ma);
	if (rc < 0) {
		pr_err("Couldn't Vote FCC en = %d rc = %d\n", en, rc);
		return rc;
	}
	rc = smbchg_parallel_usb_charging_en(chip, !en);
	return rc;
}

#define USB_AICL_CFG				0xF3
#define AICL_EN_BIT				BIT(2)
static void smbchg_aicl_enable(struct oplus_chg_chip *chip, bool enable)
{
	smbchg_sec_masked_write(chip, chip->pmic_spmi.usb_chgpth_base + USB_AICL_CFG,
		AICL_EN_BIT, enable ? AICL_EN_BIT : 0);
}

static void smbchg_rerun_aicl(struct oplus_chg_chip *chip)
{
	pr_smb(PR_STATUS, "Rerunning AICL...\n");
	smbchg_sec_masked_write(chip, chip->pmic_spmi.usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, 0);
	/* Add a delay so that AICL successfully clears */
	msleep(50);
	smbchg_sec_masked_write(chip, chip->pmic_spmi.usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, AICL_EN_BIT);
}

static void taper_irq_en(struct oplus_chg_chip *chip, bool en)
{
	mutex_lock(&chip->pmic_spmi.taper_irq_lock);
	if (en != chip->pmic_spmi.taper_irq_enabled) {
		if (en) {
			enable_irq(chip->pmic_spmi.taper_irq);
			enable_irq_wake(chip->pmic_spmi.taper_irq);
		} else {
			disable_irq_wake(chip->pmic_spmi.taper_irq);
			disable_irq_nosync(chip->pmic_spmi.taper_irq);
		}
		chip->pmic_spmi.taper_irq_enabled = en;
	}
	mutex_unlock(&chip->pmic_spmi.taper_irq_lock);
}

static int smbchg_get_aicl_level_ma(struct oplus_chg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg,
			chip->pmic_spmi.usb_chgpth_base + ICL_STS_1_REG, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Could not read usb icl sts 1: %d\n", rc);
		return 0;
	}
	if (reg & AICL_SUSP_BIT) {
		pr_warn("AICL suspended: %02x\n", reg);
		return 0;
	}
	reg &= ICL_STS_MASK;
	if (reg >= chip->pmic_spmi.tables.usb_ilim_ma_len) {
		pr_warn("invalid AICL value: %02x\n", reg);
		return 0;
	}
	return chip->pmic_spmi.tables.usb_ilim_ma_table[reg];
}

static void smbchg_parallel_usb_disable(struct oplus_chg_chip *chip)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);

	if (!parallel_psy || !chip->pmic_spmi.parallel_charger_detected)
		return;
	pr_smb(PR_STATUS, "disabling parallel charger\n");
	chip->pmic_spmi.parallel.last_disabled = ktime_get_boottime();
	taper_irq_en(chip, false);
	chip->pmic_spmi.parallel.initial_aicl_ma = 0;
	chip->pmic_spmi.parallel.current_max_ma = 0;
	power_supply_set_current_limit(parallel_psy,
				SUSPEND_CURRENT_MA * 1000);
	power_supply_set_present(parallel_psy, false);
	smbchg_set_fastchg_current_raw(chip,
			get_effective_result_locked(chip->pmic_spmi.fcc_votable));
	smbchg_set_usb_current_max(chip,
			get_effective_result_locked(chip->pmic_spmi.usb_icl_votable));
	smbchg_rerun_aicl(chip);
}

#define PARALLEL_TAPER_MAX_TRIES		3
#define PARALLEL_FCC_PERCENT_REDUCTION		75
#define MINIMUM_PARALLEL_FCC_MA			500
#define CHG_ERROR_BIT		BIT(0)
#define BAT_TAPER_MODE_BIT	BIT(6)
static void smbchg_parallel_usb_taper(struct oplus_chg_chip *chip)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);
	union power_supply_propval pval = {0, };
	int parallel_fcc_ma, tries = 0;
	u8 reg = 0;

	smbchg_detect_parallel_charger(chip);
	if (!chip->pmic_spmi.parallel_charger_detected)
		return;

	smbchg_stay_awake(chip, PM_PARALLEL_TAPER);
try_again:
	mutex_lock(&chip->pmic_spmi.parallel.lock);
	if (chip->pmic_spmi.parallel.current_max_ma == 0) {
		pr_smb(PR_STATUS, "Not parallel charging, skipping\n");
		goto done;
	}
	parallel_psy->get_property(parallel_psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &pval);
	tries += 1;
	parallel_fcc_ma = pval.intval / 1000;
	pr_smb(PR_STATUS, "try #%d parallel charger fcc = %d\n",
			tries, parallel_fcc_ma);
	if (parallel_fcc_ma < MINIMUM_PARALLEL_FCC_MA
				|| tries > PARALLEL_TAPER_MAX_TRIES) {
		smbchg_parallel_usb_disable(chip);
		goto done;
	}
	pval.intval = ((parallel_fcc_ma
			* PARALLEL_FCC_PERCENT_REDUCTION) / 100);
	pr_smb(PR_STATUS, "reducing FCC of parallel charger to %d\n",
		pval.intval);
	/* Change it to uA */
	pval.intval *= 1000;
	parallel_psy->set_property(parallel_psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &pval);
	/*
	 * sleep here for 100 ms in order to make sure the charger has a chance
	 * to go back into constant current charging
	 */
	mutex_unlock(&chip->pmic_spmi.parallel.lock);
	msleep(100);

	mutex_lock(&chip->pmic_spmi.parallel.lock);
	if (chip->pmic_spmi.parallel.current_max_ma == 0) {
		pr_smb(PR_STATUS, "Not parallel charging, skipping\n");
		goto done;
	}
	smbchg_read(chip, &reg, chip->pmic_spmi.chgr_base + RT_STS, 1);
	if (reg & BAT_TAPER_MODE_BIT) {
		mutex_unlock(&chip->pmic_spmi.parallel.lock);
		goto try_again;
	}
	taper_irq_en(chip, true);
done:
	mutex_unlock(&chip->pmic_spmi.parallel.lock);
	smbchg_relax(chip, PM_PARALLEL_TAPER);
}

static void smbchg_parallel_usb_enable(struct oplus_chg_chip *chip,
		int total_current_ma)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);
	union power_supply_propval pval = {0, };
	int new_parallel_cl_ma, set_parallel_cl_ma, new_pmi_cl_ma, rc;
	int current_table_index, target_icl_ma;
	int fcc_ma, main_fastchg_current_ma;
	int target_parallel_fcc_ma, supplied_parallel_fcc_ma;
	int parallel_chg_fcc_percent;

	if (!parallel_psy || !chip->pmic_spmi.parallel_charger_detected)
		return;

	pr_smb(PR_STATUS, "Attempting to enable parallel charger\n");

	rc = power_supply_set_voltage_limit(parallel_psy, chip->pmic_spmi.vfloat_mv + 50);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't set Vflt on parallel psy rc: %d\n", rc);
		return;
	}
	/* Set USB ICL */
	target_icl_ma = get_effective_result_locked(chip->pmic_spmi.usb_icl_votable);
	new_parallel_cl_ma = total_current_ma
			* (100 - smbchg_main_chg_icl_percent) / 100;
	taper_irq_en(chip, true);
	power_supply_set_present(parallel_psy, true);
	power_supply_set_current_limit(parallel_psy,
				new_parallel_cl_ma * 1000);
	/* read back the real amount of current we are getting */
	parallel_psy->get_property(parallel_psy,
			POWER_SUPPLY_PROP_CURRENT_MAX, &pval);
	set_parallel_cl_ma = pval.intval / 1000;
	chip->pmic_spmi.parallel.current_max_ma = new_parallel_cl_ma;
	pr_smb(PR_MISC, "Requested ICL = %d from parallel, got %d\n",
		new_parallel_cl_ma, set_parallel_cl_ma);
	new_pmi_cl_ma = max(0, target_icl_ma - set_parallel_cl_ma);
	pr_smb(PR_STATUS, "New Total USB current = %d[%d, %d]\n",
		total_current_ma, new_pmi_cl_ma,
		set_parallel_cl_ma);
	smbchg_set_usb_current_max(chip, new_pmi_cl_ma);

	/* begin splitting the fast charge current */
	fcc_ma = get_effective_result_locked(chip->pmic_spmi.fcc_votable);
	parallel_chg_fcc_percent =
		100 - smbchg_main_chg_fcc_percent;
	target_parallel_fcc_ma =
		(fcc_ma * parallel_chg_fcc_percent) / 100;
	pval.intval = target_parallel_fcc_ma * 1000;
	parallel_psy->set_property(parallel_psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &pval);
	/* check how much actual current is supplied by the parallel charger */
	parallel_psy->get_property(parallel_psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &pval);
	supplied_parallel_fcc_ma = pval.intval / 1000;
	pr_smb(PR_MISC, "Requested FCC = %d from parallel, got %d\n",
		target_parallel_fcc_ma, supplied_parallel_fcc_ma);

	/* then for the main charger, use the left over FCC */
	current_table_index = find_smaller_in_array(
			chip->pmic_spmi.tables.usb_ilim_ma_table,
			fcc_ma - supplied_parallel_fcc_ma,
			chip->pmic_spmi.tables.usb_ilim_ma_len);
	main_fastchg_current_ma =
		chip->pmic_spmi.tables.usb_ilim_ma_table[current_table_index];
	smbchg_set_fastchg_current_raw(chip, main_fastchg_current_ma);
	pr_smb(PR_STATUS, "FCC = %d[%d, %d]\n", fcc_ma, main_fastchg_current_ma,
					supplied_parallel_fcc_ma);

	chip->pmic_spmi.parallel.enabled_once = true;

	return;
}

static bool smbchg_is_parallel_usb_ok(struct oplus_chg_chip *chip,
		int *ret_total_current_ma)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);
	union power_supply_propval pval = {0, };
	int min_current_thr_ma, rc, type;
	int total_current_ma, current_limit_ma, parallel_cl_ma;
	ktime_t kt_since_last_disable;
	u8 reg;
	int fcc_ma = get_effective_result_locked(chip->pmic_spmi.fcc_votable);
	int fcc_voter_id = get_effective_client_id_locked(chip->pmic_spmi.fcc_votable);
	int usb_icl_ma = get_effective_result_locked(chip->pmic_spmi.usb_icl_votable);

	if (!parallel_psy || !smbchg_parallel_en
			|| !chip->pmic_spmi.parallel_charger_detected) {
		pr_smb(PR_STATUS, "Parallel charging not enabled\n");
		return false;
	}

	kt_since_last_disable = ktime_sub(ktime_get_boottime(),
					chip->pmic_spmi.parallel.last_disabled);
	if (chip->pmic_spmi.parallel.current_max_ma == 0
		&& chip->pmic_spmi.parallel.enabled_once
		&& ktime_to_ms(kt_since_last_disable)
			< PARALLEL_REENABLE_TIMER_MS) {
		pr_smb(PR_STATUS, "Only been %lld since disable, skipping\n",
				ktime_to_ms(kt_since_last_disable));
		return false;
	}

	/*
	 * If the battery is not present, try not to change parallel charging
	 * from OFF to ON or from ON to OFF, as it could cause the device to
	 * brown out in the instant that the USB settings are changed.
	 *
	 * Only allow parallel charging check to report false (thereby turnin
	 * off parallel charging) if the battery is still there, or if parallel
	 * charging is disabled in the first place.
	 */
	if (get_prop_charge_type(chip) != POWER_SUPPLY_CHARGE_TYPE_FAST
			&& (get_prop_batt_present(chip)
				|| chip->pmic_spmi.parallel.current_max_ma == 0)) {
		pr_smb(PR_STATUS, "Not in fast charge, skipping\n");
		return false;
	}

	if (get_prop_batt_health(chip) != POWER_SUPPLY_HEALTH_GOOD) {
		pr_smb(PR_STATUS, "JEITA active, skipping\n");
		return false;
	}

	rc = smbchg_read(chip, &reg, chip->pmic_spmi.misc_base + IDEV_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read status 5 rc = %d\n", rc);
		return false;
	}

	type = get_type(reg);
	if (get_usb_supply_type(type) == POWER_SUPPLY_TYPE_USB_CDP) {
		pr_smb(PR_STATUS, "CDP adapter, skipping\n");
		return false;
	}

	if (get_usb_supply_type(type) == POWER_SUPPLY_TYPE_USB) {
		pr_smb(PR_STATUS, "SDP adapter, skipping\n");
		return false;
	}

	/*
	 * If USBIN is suspended or not the active power source, do not enable
	 * parallel charging. The device may be charging off of DCIN.
	 */
	if (!smbchg_is_usbin_active_pwr_src(chip)) {
		pr_smb(PR_STATUS, "USB not active power source: %02x\n", reg);
		return false;
	}

	min_current_thr_ma = smbchg_get_min_parallel_current_ma(chip);
	if (min_current_thr_ma <= 0) {
		pr_smb(PR_STATUS, "parallel charger unavailable for thr: %d\n",
				min_current_thr_ma);
		return false;
	}

	if (usb_icl_ma < min_current_thr_ma) {
		pr_smb(PR_STATUS, "Weak USB chg skip enable: %d < %d\n",
			usb_icl_ma, min_current_thr_ma);
		return false;
	}

	/*
	 * Suspend the parallel charger if the charging current is < 1800 mA
	 * and is not because of an ESR pulse.
	 */
	if (fcc_voter_id != ESR_PULSE_FCC_VOTER
			&& fcc_ma < PARALLEL_CHG_THRESHOLD_CURRENT) {
		pr_smb(PR_STATUS, "FCC %d lower than %d\n",
			fcc_ma,
			PARALLEL_CHG_THRESHOLD_CURRENT);
		return false;
	}

	current_limit_ma = smbchg_get_aicl_level_ma(chip);
	if (current_limit_ma <= 0)
		return false;

	if (chip->pmic_spmi.parallel.initial_aicl_ma == 0) {
		if (current_limit_ma < min_current_thr_ma) {
			pr_smb(PR_STATUS, "Initial AICL very low: %d < %d\n",
				current_limit_ma, min_current_thr_ma);
			return false;
		}
		chip->pmic_spmi.parallel.initial_aicl_ma = current_limit_ma;
	}

	parallel_psy->get_property(parallel_psy,
			POWER_SUPPLY_PROP_CURRENT_MAX, &pval);
	parallel_cl_ma = pval.intval / 1000;
	/*
	 * Read back the real amount of current we are getting
	 * Treat 2mA as 0 because that is the suspend current setting
	 */
	if (parallel_cl_ma <= SUSPEND_CURRENT_MA)
		parallel_cl_ma = 0;

	/*
	 * Set the parallel charge path's input current limit (ICL)
	 * to the total current / 2
	 */
	total_current_ma = min(current_limit_ma + parallel_cl_ma, usb_icl_ma);

	if (total_current_ma < chip->pmic_spmi.parallel.initial_aicl_ma
			- chip->pmic_spmi.parallel.allowed_lowering_ma) {
		pr_smb(PR_STATUS,
			"Total current reduced a lot: %d (%d + %d) < %d - %d\n",
			total_current_ma,
			current_limit_ma, parallel_cl_ma,
			chip->pmic_spmi.parallel.initial_aicl_ma,
			chip->pmic_spmi.parallel.allowed_lowering_ma);
		return false;
	}

	*ret_total_current_ma = total_current_ma;
	return true;
}

#define PARALLEL_CHARGER_EN_DELAY_MS	500
static void smbchg_parallel_usb_en_work(struct work_struct *work)
{
	struct qcom_pmic *pmic_chip = container_of(work,
				struct qcom_pmic,
				parallel_en_work.work);
	struct oplus_chg_chip *chip = container_of(pmic_chip,
				struct oplus_chg_chip ,
				pmic_spmi);
	
	int previous_aicl_ma, total_current_ma, aicl_ma;
	bool in_progress;

	/* do a check to see if the aicl is stable */
	previous_aicl_ma = smbchg_get_aicl_level_ma(chip);
	msleep(PARALLEL_CHARGER_EN_DELAY_MS);
	aicl_ma = smbchg_get_aicl_level_ma(chip);
	if (previous_aicl_ma == aicl_ma) {
		pr_smb(PR_STATUS, "AICL at %d\n", aicl_ma);
	} else {
		pr_smb(PR_STATUS,
			"AICL changed [%d -> %d], recheck %d ms\n",
			previous_aicl_ma, aicl_ma,
			PARALLEL_CHARGER_EN_DELAY_MS);
		goto recheck;
	}

	mutex_lock(&chip->pmic_spmi.parallel.lock);
	in_progress = (chip->pmic_spmi.parallel.current_max_ma != 0);
	if (smbchg_is_parallel_usb_ok(chip, &total_current_ma)) {
		smbchg_parallel_usb_enable(chip, total_current_ma);
	} else {
		if (in_progress) {
			pr_smb(PR_STATUS, "parallel charging unavailable\n");
			smbchg_parallel_usb_disable(chip);
		}
	}
	mutex_unlock(&chip->pmic_spmi.parallel.lock);
	smbchg_relax(chip, PM_PARALLEL_CHECK);
	return;

recheck:
	schedule_delayed_work(&chip->pmic_spmi.parallel_en_work, 0);
}

static void smbchg_parallel_usb_check_ok(struct oplus_chg_chip *chip)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);

	if (!parallel_psy || !chip->pmic_spmi.parallel_charger_detected)
		return;

	smbchg_stay_awake(chip, PM_PARALLEL_CHECK);
	schedule_delayed_work(&chip->pmic_spmi.parallel_en_work, 0);
}

static int charging_suspend_vote_cb(struct device *dev, int suspend,
						int client, int last_suspend,
						int last_client)
{
	int rc;
	struct oplus_chg_chip *chip = dev_get_drvdata(dev);

	rc = smbchg_charging_en(chip, !suspend);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't configure batt chg: 0x%x rc = %d\n",
			!suspend, rc);
	}

	return rc;
}

static int usb_suspend_vote_cb(struct device *dev, int suspend,
						int client, int last_suspend,
						int last_client)
{
	int rc;
	struct oplus_chg_chip *chip = dev_get_drvdata(dev);

	rc = smbchg_usb_suspend(chip, suspend);
	if (rc < 0)
		return rc;

	if (client == THERMAL_EN_VOTER || client == POWER_SUPPLY_EN_VOTER ||
				client == USER_EN_VOTER ||
				client == FAKE_BATTERY_EN_VOTER)
		smbchg_parallel_usb_check_ok(chip);

	return rc;
}

static int dc_suspend_vote_cb(struct device *dev, int suspend,
						int client, int last_suspend,
						int last_client)
{
	int rc;
	struct oplus_chg_chip *chip = dev_get_drvdata(dev);

	rc = smbchg_dc_suspend(chip, suspend);
	if (rc < 0)
		return rc;

	if (chip->pmic_spmi.dc_psy_type != -EINVAL && chip->pmic_spmi.psy_registered)
		power_supply_changed(&chip->pmic_spmi.dc_psy);

	return rc;
}

static int set_fastchg_current_vote_cb(struct device *dev,
						int fcc_ma,
						int client,
						int last_fcc_ma,
						int last_client)
{
	struct oplus_chg_chip *chip = dev_get_drvdata(dev);
	int rc;

	if (chip->pmic_spmi.parallel.current_max_ma == 0) {
		rc = smbchg_set_fastchg_current_raw(chip, fcc_ma);
		if (rc < 0) {
			pr_err("Can't set FCC fcc_ma=%d rc=%d\n", fcc_ma, rc);
			return rc;
		}
	}
	/*
	 * check if parallel charging can be enabled, and if enabled,
	 * distribute the fcc
	 */
	smbchg_parallel_usb_check_ok(chip);
	return 0;
}

#if 0
static int smbchg_set_fastchg_current_user(struct oplus_chg_chip *chip,
							int current_ma)
{
	int rc = 0;

	pr_smb(PR_STATUS, "User setting FCC to %d\n", current_ma);

	rc = vote(chip->pmic_spmi.fcc_votable, BATT_TYPE_FCC_VOTER, true, current_ma);
	if (rc < 0)
		pr_err("Couldn't vote en rc %d\n", rc);
	return rc;
}
#endif

static struct ilim_entry *smbchg_wipower_find_entry(struct oplus_chg_chip *chip,
				struct ilim_map *map, int uv)
{
	int i;
	struct ilim_entry *ret = &(chip->pmic_spmi.wipower_default.entries[0]);

	for (i = 0; i < map->num; i++) {
		if (is_between(map->entries[i].vmin_uv, map->entries[i].vmax_uv,
			uv))
			ret = &map->entries[i];
	}
	return ret;
}

#define ZIN_ICL_PT	0xFC
#define ZIN_ICL_LV	0xFD
#define ZIN_ICL_HV	0xFE
#define ZIN_ICL_MASK	SMB_MASK(4, 0)
static int smbchg_dcin_ilim_config(struct oplus_chg_chip *chip, int offset, int ma)
{
	int i, rc;

	i = find_smaller_in_array(chip->pmic_spmi.tables.dc_ilim_ma_table,
			ma, chip->pmic_spmi.tables.dc_ilim_ma_len);

	if (i < 0)
		i = 0;

	rc = smbchg_sec_masked_write(chip, chip->pmic_spmi.bat_if_base + offset,
			ZIN_ICL_MASK, i);
	if (rc)
		dev_err(chip->dev, "Couldn't write bat if offset %d value = %d rc = %d\n",
				offset, i, rc);
	return rc;
}

static int smbchg_wipower_ilim_config(struct oplus_chg_chip *chip,
						struct ilim_entry *ilim)
{
	int rc = 0;

	if (chip->pmic_spmi.current_ilim.icl_pt_ma != ilim->icl_pt_ma) {
		rc = smbchg_dcin_ilim_config(chip, ZIN_ICL_PT, ilim->icl_pt_ma);
		if (rc)
			dev_err(chip->dev, "failed to write batif offset %d %dma rc = %d\n",
					ZIN_ICL_PT, ilim->icl_pt_ma, rc);
		else
			chip->pmic_spmi.current_ilim.icl_pt_ma =  ilim->icl_pt_ma;
	}

	if (chip->pmic_spmi.current_ilim.icl_lv_ma !=  ilim->icl_lv_ma) {
		rc = smbchg_dcin_ilim_config(chip, ZIN_ICL_LV, ilim->icl_lv_ma);
		if (rc)
			dev_err(chip->dev, "failed to write batif offset %d %dma rc = %d\n",
					ZIN_ICL_LV, ilim->icl_lv_ma, rc);
		else
			chip->pmic_spmi.current_ilim.icl_lv_ma =  ilim->icl_lv_ma;
	}

	if (chip->pmic_spmi.current_ilim.icl_hv_ma !=  ilim->icl_hv_ma) {
		rc = smbchg_dcin_ilim_config(chip, ZIN_ICL_HV, ilim->icl_hv_ma);
		if (rc)
			dev_err(chip->dev, "failed to write batif offset %d %dma rc = %d\n",
					ZIN_ICL_HV, ilim->icl_hv_ma, rc);
		else
			chip->pmic_spmi.current_ilim.icl_hv_ma =  ilim->icl_hv_ma;
	}
	return rc;
}

static void btm_notify_dcin(enum qpnp_tm_state state, void *ctx);
static int smbchg_wipower_dcin_btm_configure(struct oplus_chg_chip *chip,
		struct ilim_entry *ilim)
{
	int rc;

	if (ilim->vmin_uv == chip->pmic_spmi.current_ilim.vmin_uv
			&& ilim->vmax_uv == chip->pmic_spmi.current_ilim.vmax_uv)
		return 0;

	chip->pmic_spmi.param.channel = DCIN;
	chip->pmic_spmi.param.btm_ctx = chip;
	if (wipower_dcin_interval < ADC_MEAS1_INTERVAL_0MS)
		wipower_dcin_interval = ADC_MEAS1_INTERVAL_0MS;

	if (wipower_dcin_interval > ADC_MEAS1_INTERVAL_16S)
		wipower_dcin_interval = ADC_MEAS1_INTERVAL_16S;

	chip->pmic_spmi.param.timer_interval = wipower_dcin_interval;
	chip->pmic_spmi.param.threshold_notification = &btm_notify_dcin;
	chip->pmic_spmi.param.high_thr = ilim->vmax_uv + wipower_dcin_hyst_uv;
	chip->pmic_spmi.param.low_thr = ilim->vmin_uv - wipower_dcin_hyst_uv;
	chip->pmic_spmi.param.state_request = ADC_TM_HIGH_LOW_THR_ENABLE;
	rc = qpnp_vadc_channel_monitor(chip->pmic_spmi.vadc_dev, &chip->pmic_spmi.param);
	if (rc) {
		dev_err(chip->dev, "Couldn't configure btm for dcin rc = %d\n",
				rc);
	} else {
		chip->pmic_spmi.current_ilim.vmin_uv = ilim->vmin_uv;
		chip->pmic_spmi.current_ilim.vmax_uv = ilim->vmax_uv;
		pr_smb(PR_STATUS, "btm ilim = (%duV %duV %dmA %dmA %dmA)\n",
			ilim->vmin_uv, ilim->vmax_uv,
			ilim->icl_pt_ma, ilim->icl_lv_ma, ilim->icl_hv_ma);
	}
	return rc;
}

static int smbchg_wipower_icl_configure(struct oplus_chg_chip *chip,
						int dcin_uv, bool div2)
{
	int rc = 0;
	struct ilim_map *map = div2 ? &chip->pmic_spmi.wipower_div2 : &chip->pmic_spmi.wipower_pt;
	struct ilim_entry *ilim = smbchg_wipower_find_entry(chip, map, dcin_uv);

	rc = smbchg_wipower_ilim_config(chip, ilim);
	if (rc) {
		dev_err(chip->dev, "failed to config ilim rc = %d, dcin_uv = %d , div2 = %d, ilim = (%duV %duV %dmA %dmA %dmA)\n",
			rc, dcin_uv, div2,
			ilim->vmin_uv, ilim->vmax_uv,
			ilim->icl_pt_ma, ilim->icl_lv_ma, ilim->icl_hv_ma);
		return rc;
	}

	rc = smbchg_wipower_dcin_btm_configure(chip, ilim);
	if (rc) {
		dev_err(chip->dev, "failed to config btm rc = %d, dcin_uv = %d , div2 = %d, ilim = (%duV %duV %dmA %dmA %dmA)\n",
			rc, dcin_uv, div2,
			ilim->vmin_uv, ilim->vmax_uv,
			ilim->icl_pt_ma, ilim->icl_lv_ma, ilim->icl_hv_ma);
		return rc;
	}
	chip->pmic_spmi.wipower_configured = true;
	return 0;
}

static void smbchg_wipower_icl_deconfigure(struct oplus_chg_chip *chip)
{
	int rc;
	struct ilim_entry *ilim = &(chip->pmic_spmi.wipower_default.entries[0]);

	if (!chip->pmic_spmi.wipower_configured)
		return;

	rc = smbchg_wipower_ilim_config(chip, ilim);
	if (rc)
		dev_err(chip->dev, "Couldn't config default ilim rc = %d\n",
				rc);

	rc = qpnp_vadc_end_channel_monitor(chip->pmic_spmi.vadc_dev);
	if (rc)
		dev_err(chip->dev, "Couldn't de configure btm for dcin rc = %d\n",
				rc);

	chip->pmic_spmi.wipower_configured = false;
	chip->pmic_spmi.current_ilim.vmin_uv = 0;
	chip->pmic_spmi.current_ilim.vmax_uv = 0;
	chip->pmic_spmi.current_ilim.icl_pt_ma = ilim->icl_pt_ma;
	chip->pmic_spmi.current_ilim.icl_lv_ma = ilim->icl_lv_ma;
	chip->pmic_spmi.current_ilim.icl_hv_ma = ilim->icl_hv_ma;
	pr_smb(PR_WIPOWER, "De config btm\n");
}

#define FV_STS		0x0C
#define DIV2_ACTIVE	BIT(7)
static void __smbchg_wipower_check(struct oplus_chg_chip *chip)
{
	int chg_type;
	bool usb_present, dc_present;
	int rc;
	int dcin_uv;
	bool div2;
	struct qpnp_vadc_result adc_result;
	u8 reg;

	if (!wipower_dyn_icl_en) {
		smbchg_wipower_icl_deconfigure(chip);
		return;
	}

	chg_type = get_prop_charge_type(chip);
	usb_present = is_usb_present(chip);
	dc_present = is_dc_present(chip);
	if (chg_type != POWER_SUPPLY_CHARGE_TYPE_NONE
			 && !usb_present
			&& dc_present
			&& chip->pmic_spmi.dc_psy_type == POWER_SUPPLY_TYPE_WIPOWER) {
		rc = qpnp_vadc_read(chip->pmic_spmi.vadc_dev, DCIN, &adc_result);
		if (rc) {
			pr_smb(PR_STATUS, "error DCIN read rc = %d\n", rc);
			return;
		}
		dcin_uv = adc_result.physical;

		/* check div_by_2 */
		rc = smbchg_read(chip, &reg, chip->pmic_spmi.chgr_base + FV_STS, 1);
		if (rc) {
			pr_smb(PR_STATUS, "error DCIN read rc = %d\n", rc);
			return;
		}
		div2 = !!(reg & DIV2_ACTIVE);

		pr_smb(PR_WIPOWER,
			"config ICL chg_type = %d usb = %d dc = %d dcin_uv(adc_code) = %d (0x%x) div2 = %d\n",
			chg_type, usb_present, dc_present, dcin_uv,
			adc_result.adc_code, div2);
		smbchg_wipower_icl_configure(chip, dcin_uv, div2);
	} else {
		pr_smb(PR_WIPOWER,
			"deconfig ICL chg_type = %d usb = %d dc = %d\n",
			chg_type, usb_present, dc_present);
		smbchg_wipower_icl_deconfigure(chip);
	}
}

static void smbchg_wipower_check(struct oplus_chg_chip *chip)
{
	if (!chip->pmic_spmi.wipower_dyn_icl_avail)
		return;

	mutex_lock(&chip->pmic_spmi.wipower_config);
	__smbchg_wipower_check(chip);
	mutex_unlock(&chip->pmic_spmi.wipower_config);
}

static void btm_notify_dcin(enum qpnp_tm_state state, void *ctx)
{
	struct oplus_chg_chip *chip = ctx;

	mutex_lock(&chip->pmic_spmi.wipower_config);
	pr_smb(PR_WIPOWER, "%s state\n",
			state  == ADC_TM_LOW_STATE ? "low" : "high");
	chip->pmic_spmi.current_ilim.vmin_uv = 0;
	chip->pmic_spmi.current_ilim.vmax_uv = 0;
	__smbchg_wipower_check(chip);
	mutex_unlock(&chip->pmic_spmi.wipower_config);
}

static int force_dcin_icl_write(void *data, u64 val)
{
	struct oplus_chg_chip *chip = data;

	smbchg_wipower_check(chip);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(force_dcin_icl_ops, NULL,
		force_dcin_icl_write, "0x%02llx\n");

/*
 * set the dc charge path's maximum allowed current draw
 * that may be limited by the system's thermal level
 */
static int set_dc_current_limit_vote_cb(struct device *dev,
						int icl_ma,
						int client,
						int last_icl_ma,
						int last_client)
{
	struct oplus_chg_chip *chip = dev_get_drvdata(dev);

	return smbchg_set_dc_current_max(chip, icl_ma);
}

/*
 * set the usb charge path's maximum allowed current draw
 * that may be limited by the system's thermal level
 */
static int set_usb_current_limit_vote_cb(struct device *dev,
						int icl_ma,
						int client,
						int last_icl_ma,
						int last_client)
{
	struct oplus_chg_chip *chip = dev_get_drvdata(dev);
	int rc, aicl_ma, effective_id;

	effective_id = get_effective_client_id_locked(chip->pmic_spmi.usb_icl_votable);

	/* disable parallel charging if HVDCP is voting for 300mA */
	if (effective_id == HVDCP_ICL_VOTER)
		smbchg_parallel_usb_disable(chip);

	if (chip->pmic_spmi.parallel.current_max_ma == 0) {
		rc = smbchg_set_usb_current_max(chip, icl_ma);
		if (rc) {
			pr_err("Failed to set usb current max: %d\n", rc);
			return rc;
		}
	}

	/* skip the aicl rerun if hvdcp icl voter is active */
	if (effective_id == HVDCP_ICL_VOTER)
		return 0;

	aicl_ma = smbchg_get_aicl_level_ma(chip);
	chg_err("icl_ma:%d,aicl_ma:%d\n", icl_ma, aicl_ma);
#if 0
	if (icl_ma > aicl_ma)
		smbchg_rerun_aicl(chip);
#else
	if(chip->pmic_spmi.usb_supply_type == POWER_SUPPLY_TYPE_USB
		&& icl_ma > aicl_ma) {
		smbchg_rerun_aicl(chip);
	}
#endif
	smbchg_parallel_usb_check_ok(chip);
	return 0;
}

static int oplus_chg_set_input_current(struct oplus_chg_chip *chip,
							int current_ma)
{
	int rc;
	
	rc = vote(chip->pmic_spmi.usb_icl_votable, PSY_ICL_VOTER,
			true, current_ma);
	if (rc < 0) {
		pr_err("Can't vote oplus_chg %d current limit rc=%d\n",
			current_ma, rc);
	}
		
	return 0;
}

static int smbchg_system_temp_level_set(struct oplus_chg_chip *chip,
								int lvl_sel)
{
	int rc = 0;
	int prev_therm_lvl;
	int thermal_icl_ma;

	if (!chip->pmic_spmi.thermal_mitigation) {
		dev_err(chip->dev, "Thermal mitigation not supported\n");
		return -EINVAL;
	}

	if (lvl_sel < 0) {
		dev_err(chip->dev, "Unsupported level selected %d\n", lvl_sel);
		return -EINVAL;
	}

	if (lvl_sel >= chip->pmic_spmi.thermal_levels) {
		dev_err(chip->dev, "Unsupported level selected %d forcing %d\n",
				lvl_sel, chip->pmic_spmi.thermal_levels - 1);
		lvl_sel = chip->pmic_spmi.thermal_levels - 1;
	}

	if (lvl_sel == chip->pmic_spmi.therm_lvl_sel)
		return 0;

	mutex_lock(&chip->pmic_spmi.therm_lvl_lock);
	prev_therm_lvl = chip->pmic_spmi.therm_lvl_sel;
	chip->pmic_spmi.therm_lvl_sel = lvl_sel;
	if (chip->pmic_spmi.therm_lvl_sel == (chip->pmic_spmi.thermal_levels - 1)) {
		/*
		 * Disable charging if highest value selected by
		 * setting the DC and USB path in suspend
		 */
		rc = vote(chip->pmic_spmi.dc_suspend_votable, THERMAL_EN_VOTER, true, 0);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set dc suspend rc %d\n", rc);
			goto out;
		}
		rc = vote(chip->pmic_spmi.usb_suspend_votable, THERMAL_EN_VOTER, true, 0);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set usb suspend rc %d\n", rc);
			goto out;
		}
		goto out;
	}

	if (chip->pmic_spmi.therm_lvl_sel == 0) {
		rc = vote(chip->pmic_spmi.usb_icl_votable, THERMAL_ICL_VOTER, false, 0);
		if (rc < 0)
			pr_err("Couldn't disable USB thermal ICL vote rc=%d\n",
				rc);

		rc = vote(chip->pmic_spmi.dc_icl_votable, THERMAL_ICL_VOTER, false, 0);
		if (rc < 0)
			pr_err("Couldn't disable DC thermal ICL vote rc=%d\n",
				rc);
	} else {
		thermal_icl_ma =
			(int)chip->pmic_spmi.thermal_mitigation[chip->pmic_spmi.therm_lvl_sel];
		rc = vote(chip->pmic_spmi.usb_icl_votable, THERMAL_ICL_VOTER, true,
					thermal_icl_ma);
		if (rc < 0)
			pr_err("Couldn't vote for USB thermal ICL rc=%d\n", rc);

		rc = vote(chip->pmic_spmi.dc_icl_votable, THERMAL_ICL_VOTER, true,
					thermal_icl_ma);
		if (rc < 0)
			pr_err("Couldn't vote for DC thermal ICL rc=%d\n", rc);
	}

	if (prev_therm_lvl == chip->pmic_spmi.thermal_levels - 1) {
		/*
		 * If previously highest value was selected charging must have
		 * been disabed. Enable charging by taking the DC and USB path
		 * out of suspend.
		 */
		rc = vote(chip->pmic_spmi.dc_suspend_votable, THERMAL_EN_VOTER, false, 0);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set dc suspend rc %d\n", rc);
			goto out;
		}
		rc = vote(chip->pmic_spmi.usb_suspend_votable, THERMAL_EN_VOTER,
								false, 0);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set usb suspend rc %d\n", rc);
			goto out;
		}
	}
out:
	mutex_unlock(&chip->pmic_spmi.therm_lvl_lock);
	return rc;
}

static int smbchg_ibat_ocp_threshold_ua = 4500000;
module_param(smbchg_ibat_ocp_threshold_ua, int, 0644);

#define UCONV			1000000LL
#define MCONV			1000LL
#define FLASH_V_THRESHOLD	3000000
#define FLASH_VDIP_MARGIN	100000
#define VPH_FLASH_VDIP		(FLASH_V_THRESHOLD + FLASH_VDIP_MARGIN)
#define BUCK_EFFICIENCY		800LL
static int smbchg_calc_max_flash_current(struct oplus_chg_chip *chip)
{
	int ocv_uv, esr_uohm, rbatt_uohm, ibat_now, rc;
	int64_t ibat_flash_ua, avail_flash_ua, avail_flash_power_fw;
	int64_t ibat_safe_ua, vin_flash_uv, vph_flash_uv;

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_VOLTAGE_OCV, &ocv_uv);
	if (rc) {
		pr_smb(PR_STATUS, "bms psy does not support OCV\n");
		return 0;
	}

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_RESISTANCE,
			&esr_uohm);
	if (rc) {
		pr_smb(PR_STATUS, "bms psy does not support resistance\n");
		return 0;
	}

	rc = msm_bcl_read(BCL_PARAM_CURRENT, &ibat_now);
	if (rc) {
		pr_smb(PR_STATUS, "BCL current read failed: %d\n", rc);
		return 0;
	}

	rbatt_uohm = esr_uohm + chip->pmic_spmi.rpara_uohm + chip->pmic_spmi.rslow_uohm;
	/*
	 * Calculate the maximum current that can pulled out of the battery
	 * before the battery voltage dips below a safe threshold.
	 */
	ibat_safe_ua = div_s64((ocv_uv - VPH_FLASH_VDIP) * UCONV,
				rbatt_uohm);

	if (ibat_safe_ua <= smbchg_ibat_ocp_threshold_ua) {
		/*
		 * If the calculated current is below the OCP threshold, then
		 * use it as the possible flash current.
		 */
		ibat_flash_ua = ibat_safe_ua - ibat_now;
		vph_flash_uv = VPH_FLASH_VDIP;
	} else {
		/*
		 * If the calculated current is above the OCP threshold, then
		 * use the ocp threshold instead.
		 *
		 * Any higher current will be tripping the battery OCP.
		 */
		ibat_flash_ua = smbchg_ibat_ocp_threshold_ua - ibat_now;
		vph_flash_uv = ocv_uv - div64_s64((int64_t)rbatt_uohm
				* smbchg_ibat_ocp_threshold_ua, UCONV);
	}
	/* Calculate the input voltage of the flash module. */
	vin_flash_uv = max((chip->pmic_spmi.vled_max_uv + 500000LL),
				div64_s64((vph_flash_uv * 1200), 1000));
	/* Calculate the available power for the flash module. */
	avail_flash_power_fw = BUCK_EFFICIENCY * vph_flash_uv * ibat_flash_ua;
	/*
	 * Calculate the available amount of current the flash module can draw
	 * before collapsing the battery. (available power/ flash input voltage)
	 */
	avail_flash_ua = div64_s64(avail_flash_power_fw, vin_flash_uv * MCONV);
	pr_smb(PR_MISC,
		"avail_iflash=%lld, ocv=%d, ibat=%d, rbatt=%d\n",
		avail_flash_ua, ocv_uv, ibat_now, rbatt_uohm);
	return (int)avail_flash_ua;
}

#define FCC_CMP_CFG	0xF3
#define FCC_COMP_MASK	SMB_MASK(1, 0)
static int smbchg_fastchg_current_comp_set(struct oplus_chg_chip *chip,
					int comp_current)
{
	int rc;
	u8 i;

	for (i = 0; i < chip->pmic_spmi.tables.fcc_comp_len; i++)
		if (comp_current == chip->pmic_spmi.tables.fcc_comp_table[i])
			break;

	if (i >= chip->pmic_spmi.tables.fcc_comp_len)
		return -EINVAL;

	rc = smbchg_sec_masked_write(chip, chip->pmic_spmi.chgr_base + FCC_CMP_CFG,
			FCC_COMP_MASK, i);

	if (rc)
		dev_err(chip->dev, "Couldn't set fastchg current comp rc = %d\n",
			rc);

	return rc;
}

#define CFG_TCC_REG			0xF9
#define CHG_ITERM_MASK			SMB_MASK(2, 0)
static int smbchg_iterm_set(struct oplus_chg_chip *chip, int iterm_ma)
{
	int rc;
	u8 reg;

	reg = find_closest_in_array(
			chip->pmic_spmi.tables.iterm_ma_table,
			chip->pmic_spmi.tables.iterm_ma_len,
			iterm_ma);
	chg_err("iterm_ma:%d, reg:%d\n", iterm_ma, reg);
	rc = smbchg_sec_masked_write(chip,
			chip->pmic_spmi.chgr_base + CFG_TCC_REG,
			CHG_ITERM_MASK, reg);
	if (rc) {
		dev_err(chip->dev,
			"Couldn't set iterm rc = %d\n", rc);
		return rc;
	}
	pr_smb(PR_STATUS, "set tcc (%d) to 0x%02x\n",
			iterm_ma, reg);
	chip->pmic_spmi.iterm_ma = iterm_ma;

	return 0;
}

#define FV_CMP_CFG	0xF5
#define FV_COMP_MASK	SMB_MASK(5, 0)
static int smbchg_float_voltage_comp_set(struct oplus_chg_chip *chip, int code)
{
	int rc;
	u8 val;

	val = code & FV_COMP_MASK;
	rc = smbchg_sec_masked_write(chip, chip->pmic_spmi.chgr_base + FV_CMP_CFG,
			FV_COMP_MASK, val);

	if (rc)
		dev_err(chip->dev, "Couldn't set float voltage comp rc = %d\n",
			rc);

	return rc;
}

#define VFLOAT_CFG_REG			0xF4
#define MIN_FLOAT_MV			3600
#define MAX_FLOAT_MV			4500
#define VFLOAT_MASK			SMB_MASK(5, 0)

#define MID_RANGE_FLOAT_MV_MIN		3600
#define MID_RANGE_FLOAT_MIN_VAL		0x05
#define MID_RANGE_FLOAT_STEP_MV		20

#define HIGH_RANGE_FLOAT_MIN_MV		4340
#define HIGH_RANGE_FLOAT_MIN_VAL	0x2A
#define HIGH_RANGE_FLOAT_STEP_MV	10

#define VHIGH_RANGE_FLOAT_MIN_MV	4360
#define VHIGH_RANGE_FLOAT_MIN_VAL	0x2C
#define VHIGH_RANGE_FLOAT_STEP_MV	20
static int smbchg_float_voltage_set(struct oplus_chg_chip *chip, int vfloat_mv)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);
	int rc, delta;
	u8 temp;

	if ((vfloat_mv < MIN_FLOAT_MV) || (vfloat_mv > MAX_FLOAT_MV)) {
		dev_err(chip->dev, "bad float voltage mv =%d asked to set\n",
					vfloat_mv);
		return -EINVAL;
	}
	chg_err("vfloat_mv:%d\n", vfloat_mv);
	if (vfloat_mv <= HIGH_RANGE_FLOAT_MIN_MV) {
		/* mid range */
		delta = vfloat_mv - MID_RANGE_FLOAT_MV_MIN;
		temp = MID_RANGE_FLOAT_MIN_VAL + delta
				/ MID_RANGE_FLOAT_STEP_MV;
		vfloat_mv -= delta % MID_RANGE_FLOAT_STEP_MV;
	} else if (vfloat_mv <= VHIGH_RANGE_FLOAT_MIN_MV) {
		/* high range */
		delta = vfloat_mv - HIGH_RANGE_FLOAT_MIN_MV;
		temp = HIGH_RANGE_FLOAT_MIN_VAL + delta
				/ HIGH_RANGE_FLOAT_STEP_MV;
		vfloat_mv -= delta % HIGH_RANGE_FLOAT_STEP_MV;
	} else {
		/* very high range */
		delta = vfloat_mv - VHIGH_RANGE_FLOAT_MIN_MV;
		temp = VHIGH_RANGE_FLOAT_MIN_VAL + delta
				/ VHIGH_RANGE_FLOAT_STEP_MV;
		vfloat_mv -= delta % VHIGH_RANGE_FLOAT_STEP_MV;
	}

	if (parallel_psy) {
		rc = power_supply_set_voltage_limit(parallel_psy,
				vfloat_mv + 50);
		if (rc)
			dev_err(chip->dev, "Couldn't set float voltage on parallel psy rc: %d\n",
				rc);
	}

	rc = smbchg_sec_masked_write(chip, chip->pmic_spmi.chgr_base + VFLOAT_CFG_REG,
			VFLOAT_MASK, temp);

	if (rc)
		dev_err(chip->dev, "Couldn't set float voltage rc = %d\n", rc);
	else
		chip->pmic_spmi.vfloat_mv = vfloat_mv;

	return rc;
}

#if 0
static int smbchg_float_voltage_get(struct oplus_chg_chip *chip)
{
	return chip->pmic_spmi.vfloat_mv;
}
#endif

#define SFT_CFG				0xFD
#define SFT_EN_MASK			SMB_MASK(5, 4)
#define SFT_TO_MASK			SMB_MASK(3, 2)
#define PRECHG_SFT_TO_MASK		SMB_MASK(1, 0)
#define SFT_TIMER_DISABLE_BIT		BIT(5)
#define PRECHG_SFT_TIMER_DISABLE_BIT	BIT(4)
#define SAFETY_TIME_MINUTES_SHIFT	2

#if 0
static int smbchg_safety_timer_enable(struct oplus_chg_chip *chip, bool enable)
{
	int rc;
	u8 reg;

	if (enable == chip->pmic_spmi.safety_timer_en)
		return 0;

	if (enable)
		reg = 0;
	else
		reg = SFT_TIMER_DISABLE_BIT | PRECHG_SFT_TIMER_DISABLE_BIT;

	rc = smbchg_sec_masked_write(chip, chip->pmic_spmi.chgr_base + SFT_CFG,
			SFT_EN_MASK, reg);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't %s safety timer rc = %d\n",
			enable ? "enable" : "disable", rc);
		return rc;
	}
	chip->pmic_spmi.safety_timer_en = enable;
	return 0;
}
#endif

#define BAT_IF_TRIM7_REG	0xF7
#define CFG_750KHZ_BIT		BIT(1)
#define MISC_CFG_NTC_VOUT_REG	0xF3
#define CFG_NTC_VOUT_FSW_BIT	BIT(0)
static int smbchg_switch_buck_frequency(struct oplus_chg_chip *chip,
				bool flash_active)
{
	int rc;

	if (!(chip->pmic_spmi.wa_flags & SMBCHG_FLASH_BUCK_SWITCH_FREQ_WA))
		return 0;

	if (chip->pmic_spmi.flash_active == flash_active) {
		pr_smb(PR_STATUS, "Fsw not changed, flash_active: %d\n",
			flash_active);
		return 0;
	}

	/*
	 * As per the systems team recommendation, before the flash fires,
	 * buck switching frequency(Fsw) needs to be increased to 1MHz. Once the
	 * flash is disabled, Fsw needs to be set back to 750KHz.
	 */
	rc = smbchg_sec_masked_write(chip, chip->pmic_spmi.misc_base +
				MISC_CFG_NTC_VOUT_REG, CFG_NTC_VOUT_FSW_BIT,
				flash_active ? CFG_NTC_VOUT_FSW_BIT : 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set switching frequency multiplier rc=%d\n",
				rc);
		return rc;
	}

	rc = smbchg_sec_masked_write(chip, chip->pmic_spmi.bat_if_base + BAT_IF_TRIM7_REG,
			CFG_750KHZ_BIT, flash_active ? 0 : CFG_750KHZ_BIT);
	if (rc < 0) {
		dev_err(chip->dev, "Cannot set switching freq: %d\n", rc);
		return rc;
	}

	pr_smb(PR_STATUS, "Fsw @ %sHz\n", flash_active ? "1M" : "750K");
	chip->pmic_spmi.flash_active = flash_active;
	return 0;
}

#define OTG_TRIM6		0xF6
#define TR_ENB_SKIP_BIT		BIT(2)
#define OTG_EN_BIT		BIT(0)
static int smbchg_otg_pulse_skip_disable(struct oplus_chg_chip *chip,
				enum skip_reason reason, bool disable)
{
	int rc;
	bool disabled;

	disabled = !!chip->pmic_spmi.otg_pulse_skip_dis;
	pr_smb(PR_STATUS, "%s pulse skip, reason %d\n",
			disable ? "disabling" : "enabling", reason);
	if (disable)
		chip->pmic_spmi.otg_pulse_skip_dis |= reason;
	else
		chip->pmic_spmi.otg_pulse_skip_dis &= ~reason;
	if (disabled == !!chip->pmic_spmi.otg_pulse_skip_dis)
		return 0;
	disabled = !!chip->pmic_spmi.otg_pulse_skip_dis;

	rc = smbchg_sec_masked_write(chip, chip->pmic_spmi.otg_base + OTG_TRIM6,
			TR_ENB_SKIP_BIT, disabled ? TR_ENB_SKIP_BIT : 0);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't %s otg pulse skip rc = %d\n",
			disabled ? "disable" : "enable", rc);
		return rc;
	}
	pr_smb(PR_STATUS, "%s pulse skip\n", disabled ? "disabled" : "enabled");
	return 0;
}

static int smbchg_set_prop_flash_active(struct oplus_chg_chip *chip,
				enum skip_reason reason, bool disable)
{
	int rc = 0;
	
	rc = smbchg_switch_buck_frequency(chip, disable);
	if (rc) {
		pr_err("Couldn't switch buck frequency, rc=%d\n", rc);
		/*
		 * Trigger a panic if there is an error while switching
		 * buck frequency. This will prevent LS FET damage.
		 */
		BUG_ON(1);
	}
	rc = smbchg_otg_pulse_skip_disable(chip, REASON_FLASH_ENABLED, disable);
	return rc;
}	

#define LOW_PWR_OPTIONS_REG	0xFF
#define FORCE_TLIM_BIT		BIT(4)
static int smbchg_force_tlim_en(struct oplus_chg_chip *chip, bool enable)
{
	int rc;

	rc = smbchg_sec_masked_write(chip, chip->pmic_spmi.otg_base + LOW_PWR_OPTIONS_REG,
			FORCE_TLIM_BIT, enable ? FORCE_TLIM_BIT : 0);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't %s otg force tlim rc = %d\n",
			enable ? "enable" : "disable", rc);
		return rc;
	}
	return rc;
}

static void smbchg_vfloat_adjust_check(struct oplus_chg_chip *chip)
{
	if (!chip->pmic_spmi.use_vfloat_adjustments)
		return;

	smbchg_stay_awake(chip, PM_REASON_VFLOAT_ADJUST);
	pr_smb(PR_STATUS, "Starting vfloat adjustments\n");
	schedule_delayed_work(&chip->pmic_spmi.vfloat_adjust_work, 0);
}

#define FV_STS_REG			0xC
#define AICL_INPUT_STS_BIT		BIT(6)
static bool smbchg_is_input_current_limited(struct oplus_chg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->pmic_spmi.chgr_base + FV_STS_REG, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read FV_STS rc=%d\n", rc);
		return false;
	}

	return !!(reg & AICL_INPUT_STS_BIT);
}

#define SW_ESR_PULSE_MS			1500
static void smbchg_cc_esr_wa_check(struct oplus_chg_chip *chip)
{
	int rc, esr_count;

	if (!(chip->pmic_spmi.wa_flags & SMBCHG_CC_ESR_WA))
		return;

	if (!is_usb_present(chip) && !is_dc_present(chip)) {
		pr_smb(PR_STATUS, "No inputs present, skipping\n");
		return;
	}

	if (get_prop_charge_type(chip) != POWER_SUPPLY_CHARGE_TYPE_FAST) {
		pr_smb(PR_STATUS, "Not in fast charge, skipping\n");
		return;
	}

	if (!smbchg_is_input_current_limited(chip)) {
		pr_smb(PR_STATUS, "Not input current limited, skipping\n");
		return;
	}

	set_property_on_fg(chip, POWER_SUPPLY_PROP_UPDATE_NOW, 1);
	rc = get_property_from_fg(chip,
			POWER_SUPPLY_PROP_ESR_COUNT, &esr_count);
	if (rc) {
		pr_smb(PR_STATUS,
			"could not read ESR counter rc = %d\n", rc);
		return;
	}

	/*
	 * The esr_count is counting down the number of fuel gauge cycles
	 * before a ESR pulse is needed.
	 *
	 * After a successful ESR pulse, this count is reset to some
	 * high number like 28. If this reaches 0, then the fuel gauge
	 * hardware should force a ESR pulse.
	 *
	 * However, if the device is in constant current charge mode while
	 * being input current limited, the ESR pulse will not affect the
	 * battery current, so the measurement will fail.
	 *
	 * As a failsafe, force a manual ESR pulse if this value is read as
	 * 0.
	 */
	if (esr_count != 0) {
		pr_smb(PR_STATUS, "ESR count is not zero, skipping\n");
		return;
	}

	pr_smb(PR_STATUS, "Lowering charge current for ESR pulse\n");
	smbchg_stay_awake(chip, PM_ESR_PULSE);
	smbchg_sw_esr_pulse_en(chip, true);
	msleep(SW_ESR_PULSE_MS);
	pr_smb(PR_STATUS, "Raising charge current for ESR pulse\n");
	smbchg_relax(chip, PM_ESR_PULSE);
	smbchg_sw_esr_pulse_en(chip, false);
}

static void smbchg_soc_changed(struct oplus_chg_chip *chip)
{
	smbchg_cc_esr_wa_check(chip);
}

#define DC_AICL_CFG			0xF3
#define MISC_TRIM_OPT_15_8		0xF5
#define USB_AICL_DEGLITCH_MASK		(BIT(5) | BIT(4) | BIT(3))
#define USB_AICL_DEGLITCH_SHORT		(BIT(5) | BIT(4) | BIT(3))
#define USB_AICL_DEGLITCH_LONG		0
#define DC_AICL_DEGLITCH_MASK		(BIT(5) | BIT(4) | BIT(3))
#define DC_AICL_DEGLITCH_SHORT		(BIT(5) | BIT(4) | BIT(3))
#define DC_AICL_DEGLITCH_LONG		0
#define AICL_RERUN_MASK			(BIT(5) | BIT(4))
#define AICL_RERUN_ON			(BIT(5) | BIT(4))
#define AICL_RERUN_OFF			0

static int smbchg_hw_aicl_rerun_enable_indirect_cb(struct device *dev,
						int enable,
						int client, int last_enable,
						int last_client)
{
	int rc = 0;
	struct oplus_chg_chip *chip = dev_get_drvdata(dev);

	/*
	 * If the indirect voting result of all the clients is to enable hw aicl
	 * rerun, then remove our vote to disable hw aicl rerun
	 */
	rc = vote(chip->pmic_spmi.hw_aicl_rerun_disable_votable,
		HW_AICL_RERUN_ENABLE_INDIRECT_VOTER, !enable, 0);
	if (rc < 0) {
		pr_err("Couldn't vote for hw rerun rc= %d\n", rc);
		return rc;
	}

	return rc;
}

static int smbchg_hw_aicl_rerun_disable_cb(struct device *dev, int disable,
						int client, int last_disable,
						int last_client)
{
	int rc = 0;
	struct oplus_chg_chip *chip = dev_get_drvdata(dev);

	rc = smbchg_sec_masked_write(chip,
		chip->pmic_spmi.misc_base + MISC_TRIM_OPT_15_8,
		AICL_RERUN_MASK, disable ? AICL_RERUN_OFF : AICL_RERUN_ON);
	if (rc < 0)
		pr_err("Couldn't write to MISC_TRIM_OPTIONS_15_8 rc=%d\n", rc);

	return rc;
}

static int smbchg_aicl_deglitch_config_cb(struct device *dev, int shorter,
						int client, int last_result,
						int last_client)
{
	int rc = 0;
	struct oplus_chg_chip *chip = dev_get_drvdata(dev);

	rc = smbchg_sec_masked_write(chip,
		chip->pmic_spmi.usb_chgpth_base + USB_AICL_CFG,
		USB_AICL_DEGLITCH_MASK,
		shorter ? USB_AICL_DEGLITCH_SHORT : USB_AICL_DEGLITCH_LONG);
	if (rc < 0) {
		pr_err("Couldn't write to USB_AICL_CFG rc=%d\n", rc);
		return rc;
	}
	rc = smbchg_sec_masked_write(chip,
		chip->pmic_spmi.dc_chgpth_base + DC_AICL_CFG,
		DC_AICL_DEGLITCH_MASK,
		shorter ? DC_AICL_DEGLITCH_SHORT : DC_AICL_DEGLITCH_LONG);
	if (rc < 0) {
		pr_err("Couldn't write to DC_AICL_CFG rc=%d\n", rc);
		return rc;
	}
	return rc;
}

static void smbchg_aicl_deglitch_wa_en(struct oplus_chg_chip *chip, bool en)
{
	int rc;

	rc = vote(chip->pmic_spmi.aicl_deglitch_short_votable,
		VARB_WORKAROUND_VOTER, en, 0);
	if (rc < 0) {
		pr_err("Couldn't vote %s deglitch rc=%d\n",
				en ? "short" : "long", rc);
		return;
	}
	pr_smb(PR_STATUS, "AICL deglitch set to %s\n", en ? "short" : "long");

	rc = vote(chip->pmic_spmi.hw_aicl_rerun_enable_indirect_votable,
			VARB_WORKAROUND_VOTER, en, 0);
	if (rc < 0) {
		pr_err("Couldn't vote hw aicl rerun rc= %d\n", rc);
		return;
	}
	chip->pmic_spmi.aicl_deglitch_short = en;
}

static void smbchg_aicl_deglitch_wa_check(struct oplus_chg_chip *chip)
{
	union power_supply_propval prop = {0,};
	int rc;
	bool low_volt_chgr = true;

	if (!(chip->pmic_spmi.wa_flags & SMBCHG_AICL_DEGLITCH_WA))
		return;

	if (!is_usb_present(chip) && !is_dc_present(chip)) {
		pr_smb(PR_STATUS, "Charger removed\n");
		smbchg_aicl_deglitch_wa_en(chip, false);
		return;
	}

	if (!chip->pmic_spmi.bms_psy)
		return;

	if (is_usb_present(chip)) {
		if (is_hvdcp_present(chip))
			low_volt_chgr = false;
	} else if (is_dc_present(chip)) {
		if (chip->pmic_spmi.dc_psy_type == POWER_SUPPLY_TYPE_WIPOWER)
			low_volt_chgr = false;
		else
			low_volt_chgr = chip->pmic_spmi.low_volt_dcin;
	}

	if (!low_volt_chgr) {
		pr_smb(PR_STATUS, "High volt charger! Don't set deglitch\n");
		smbchg_aicl_deglitch_wa_en(chip, false);
		return;
	}

	/* It is possible that battery voltage went high above threshold
	 * when the charger is inserted and can go low because of system
	 * load. We shouldn't be reconfiguring AICL deglitch when this
	 * happens as it will lead to oscillation again which is being
	 * fixed here. Do it once when the battery voltage crosses the
	 * threshold (e.g. 4.2 V) and clear it only when the charger
	 * is removed.
	 */
	if (!chip->pmic_spmi.vbat_above_headroom) {
		rc = chip->pmic_spmi.bms_psy->get_property(chip->pmic_spmi.bms_psy,
				POWER_SUPPLY_PROP_VOLTAGE_MIN, &prop);
		if (rc < 0) {
			pr_err("could not read voltage_min, rc=%d\n", rc);
			return;
		}
		chip->pmic_spmi.vbat_above_headroom = !prop.intval;
	}
	smbchg_aicl_deglitch_wa_en(chip, chip->pmic_spmi.vbat_above_headroom);
}

#define MISC_TEST_REG		0xE2
#define BB_LOOP_DISABLE_ICL	BIT(2)
static int smbchg_icl_loop_disable_check(struct oplus_chg_chip *chip)
{
	bool icl_disabled = !chip->pmic_spmi.chg_otg_enabled && chip->pmic_spmi.flash_triggered;
	int rc = 0;

	if ((chip->pmic_spmi.wa_flags & SMBCHG_FLASH_ICL_DISABLE_WA)
			&& icl_disabled != chip->pmic_spmi.icl_disabled) {
		rc = smbchg_sec_masked_write(chip,
				chip->pmic_spmi.misc_base + MISC_TEST_REG,
				BB_LOOP_DISABLE_ICL,
				icl_disabled ? BB_LOOP_DISABLE_ICL : 0);
		chip->pmic_spmi.icl_disabled = icl_disabled;
	}

	return rc;
}

#define UNKNOWN_BATT_TYPE	"Unknown Battery"
#define LOADING_BATT_TYPE	"Loading Battery Data"
static int smbchg_config_chg_battery_type(struct oplus_chg_chip *chip)
{
	int rc = 0, max_voltage_uv = 0, fastchg_ma = 0, ret = 0, iterm_ua = 0;
	struct device_node *batt_node, *profile_node;
	struct device_node *node = chip->pmic_spmi.spmi->dev.of_node;
	union power_supply_propval prop = {0,};

	if(chip->pmic_spmi.skip_fg_control_chg) {
		return 0;
	}
	rc = chip->pmic_spmi.bms_psy->get_property(chip->pmic_spmi.bms_psy,
			POWER_SUPPLY_PROP_BATTERY_TYPE, &prop);
	if (rc) {
		pr_smb(PR_STATUS, "Unable to read battery-type rc=%d\n", rc);
		return 0;
	}
	if (!strcmp(prop.strval, UNKNOWN_BATT_TYPE) ||
		!strcmp(prop.strval, LOADING_BATT_TYPE)) {
		pr_smb(PR_MISC, "Battery-type not identified\n");
		return 0;
	}
	/* quit if there is no change in the battery-type from previous */
	if (chip->pmic_spmi.battery_type && !strcmp(prop.strval, chip->pmic_spmi.battery_type))
		return 0;

	batt_node = of_parse_phandle(node, "qcom,battery-data", 0);
	if (!batt_node) {
		pr_smb(PR_MISC, "No batterydata available\n");
		return 0;
	}

	profile_node = of_batterydata_get_best_profile(batt_node,
							"bms", NULL);
	if (!profile_node) {
		pr_err("couldn't find profile handle\n");
		return -EINVAL;
	}
	chip->pmic_spmi.battery_type = prop.strval;

	/* change vfloat */
	rc = of_property_read_u32(profile_node, "qcom,max-voltage-uv",
						&max_voltage_uv);
	if (rc) {
		pr_warn("couldn't find battery max voltage rc=%d\n", rc);
		ret = rc;
	} else {
		if (chip->pmic_spmi.vfloat_mv != (max_voltage_uv / 1000)) {
			pr_info("Vfloat changed from %dmV to %dmV for battery-type %s\n",
				chip->pmic_spmi.vfloat_mv, (max_voltage_uv / 1000),
				chip->pmic_spmi.battery_type);
			rc = smbchg_float_voltage_set(chip,
						(max_voltage_uv / 1000));
			if (rc < 0) {
				dev_err(chip->dev,
				"Couldn't set float voltage rc = %d\n", rc);
				return rc;
			}
		}
	}

	/* change chg term */
	rc = of_property_read_u32(profile_node, "qcom,chg-term-ua",
						&iterm_ua);
	if (rc && rc != -EINVAL) {
		pr_warn("couldn't read battery term current=%d\n", rc);
		ret = rc;
	} else if (!rc) {
		if (chip->pmic_spmi.iterm_ma != (iterm_ua / 1000)
				&& !chip->pmic_spmi.iterm_disabled) {
			pr_info("Term current changed from %dmA to %dmA for battery-type %s\n",
				chip->pmic_spmi.iterm_ma, (iterm_ua / 1000),
				chip->pmic_spmi.battery_type);
			rc = smbchg_iterm_set(chip,
						(iterm_ua / 1000));
			if (rc < 0) {
				dev_err(chip->dev,
				"Couldn't set iterm rc = %d\n", rc);
				return rc;
			}
		}
		chip->pmic_spmi.iterm_ma = iterm_ua / 1000;
	}

	/*
	 * Only configure from profile if fastchg-ma is not defined in the
	 * charger device node.
	 */
	if (!of_find_property(chip->pmic_spmi.spmi->dev.of_node,
				"qcom,fastchg-current-ma", NULL)) {
		rc = of_property_read_u32(profile_node,
				"qcom,fastchg-current-ma", &fastchg_ma);
		if (rc) {
			ret = rc;
		} else {
			pr_smb(PR_MISC,
				"fastchg-ma changed from to %dma for battery-type %s\n",
				fastchg_ma, chip->pmic_spmi.battery_type);
			rc = vote(chip->pmic_spmi.fcc_votable, BATT_TYPE_FCC_VOTER, true,
							fastchg_ma);
			if (rc < 0) {
				dev_err(chip->dev,
					"Couldn't vote for fastchg current rc=%d\n",
					rc);
				return rc;
			}
		}
	}

	return ret;
}

#define MAX_INV_BATT_ID		7700
#define MIN_INV_BATT_ID		7300
static void check_battery_type(struct oplus_chg_chip *chip)
{
	union power_supply_propval prop = {0,};
	bool en;

	if (!chip->pmic_spmi.bms_psy && chip->pmic_spmi.bms_psy_name)
		chip->pmic_spmi.bms_psy =
			power_supply_get_by_name((char *)chip->pmic_spmi.bms_psy_name);
	if(chip->pmic_spmi.skip_fg_control_chg) {
		return;
	}
	if (chip->pmic_spmi.bms_psy) {
		chip->pmic_spmi.bms_psy->get_property(chip->pmic_spmi.bms_psy,
				POWER_SUPPLY_PROP_BATTERY_TYPE, &prop);
		en = (strcmp(prop.strval, UNKNOWN_BATT_TYPE) != 0
				|| chip->pmic_spmi.charge_unknown_battery)
			&& (strcmp(prop.strval, LOADING_BATT_TYPE) != 0);
		vote(chip->pmic_spmi.battchg_suspend_votable,
				BATTCHG_UNKNOWN_BATTERY_EN_VOTER, !en, 0);

		if (!chip->pmic_spmi.skip_usb_suspend_for_fake_battery) {
			chip->pmic_spmi.bms_psy->get_property(chip->pmic_spmi.bms_psy,
				POWER_SUPPLY_PROP_RESISTANCE_ID, &prop);
			/* suspend USB path for invalid battery-id */
			en = (prop.intval <= MAX_INV_BATT_ID &&
				prop.intval >= MIN_INV_BATT_ID) ? 1 : 0;
			vote(chip->pmic_spmi.usb_suspend_votable, FAKE_BATTERY_EN_VOTER,
				en, 0);
		}
	}
}

void oplus_chg_external_power_changed(struct power_supply *psy)
{
	struct oplus_chg_chip *chip = container_of(psy,
				struct oplus_chg_chip, batt_psy);
	union power_supply_propval prop = {0,};
	int rc, current_limit = 0, soc;
	static int current_limit_pre = 0;
	enum power_supply_type usb_supply_type;
	static int usb_supply_type_pre = POWER_SUPPLY_TYPE_UNKNOWN;
	char *usb_type_name = "null";

	if (chip->pmic_spmi.bms_psy_name)
		chip->pmic_spmi.bms_psy =
			power_supply_get_by_name((char *)chip->pmic_spmi.bms_psy_name);

	smbchg_aicl_deglitch_wa_check(chip);
	if (chip->pmic_spmi.bms_psy) {
		check_battery_type(chip);
		//soc = get_prop_batt_capacity(chip);
		soc = oplus_gauge_get_batt_soc();
		if (chip->pmic_spmi.previous_soc != soc) {
			chip->pmic_spmi.previous_soc = soc;
			smbchg_soc_changed(chip);
		}

		rc = smbchg_config_chg_battery_type(chip);
		if (rc)
			pr_smb(PR_MISC,
				"Couldn't update charger configuration rc=%d\n",
									rc);
	}

	rc = chip->usb_psy->get_property(chip->usb_psy,
				POWER_SUPPLY_PROP_CHARGING_ENABLED, &prop);
	if (rc == 0)
		vote(chip->pmic_spmi.usb_suspend_votable, POWER_SUPPLY_EN_VOTER,
				!prop.intval, 0);

	rc = chip->usb_psy->get_property(chip->usb_psy,
				POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
	if (rc == 0)
		current_limit = prop.intval / 1000;

#if 0
	read_usb_type(chip, &usb_type_name, &usb_supply_type);
	if(usb_supply_type == POWER_SUPPLY_TYPE_USB || usb_supply_type == POWER_SUPPLY_TYPE_USB_CDP){
		usb_supply_type = qpnp_charger_type_get(chip);
		chip->pmic_spmi.usb_supply_type = usb_supply_type;
		chg_err("donot use pmic,use ap detect charger,usb_supply_type=%d\n",usb_supply_type);
	}
#else
	qpnp_get_charger_type(chip, &usb_type_name, &usb_supply_type, true);
#endif
	if (usb_supply_type != POWER_SUPPLY_TYPE_USB
		&& usb_supply_type != POWER_SUPPLY_TYPE_USB_CDP)
		goto  skip_current_for_non_sdp;

	rc = vote(chip->pmic_spmi.usb_icl_votable, PSY_ICL_VOTER, true,
				current_limit);
	if (rc < 0)
		pr_err("Couldn't update USB PSY ICL vote rc=%d\n", rc);

skip_current_for_non_sdp:
	if(current_limit != current_limit_pre || usb_supply_type != usb_supply_type_pre) {
		if (is_usb_present(chip) || is_dc_present(chip)) {
			if (current_limit > 2) {
				chip->charger_type = POWER_SUPPLY_TYPE_UNKNOWN;
				oplus_chg_wake_update_work();
			}
		} else {
			oplus_chg_wake_update_work();
		}	
	}
	chg_err("current_limit:%d,current_pre:%d,usb_type:%d,usb_type_pre:%d\n",
		current_limit, current_limit_pre, usb_supply_type, usb_supply_type_pre);
	current_limit_pre = current_limit;
	usb_supply_type_pre = usb_supply_type;
	smbchg_vfloat_adjust_check(chip);

	power_supply_changed(&chip->batt_psy);
}

static int smbchg_otg_regulator_enable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct oplus_chg_chip *chip = rdev_get_drvdata(rdev);

	chip->pmic_spmi.otg_retries = 0;
	chip->pmic_spmi.chg_otg_enabled = true;
	smbchg_icl_loop_disable_check(chip);
	smbchg_otg_pulse_skip_disable(chip, REASON_OTG_ENABLED, true);

	/* If pin control mode then return from here */
	if (chip->pmic_spmi.otg_pinctrl)
		return rc;

	/* sleep to make sure the pulse skip is actually disabled */
	msleep(20);
	rc = smbchg_masked_write(chip, chip->pmic_spmi.bat_if_base + CMD_CHG_REG,
			OTG_EN_BIT, OTG_EN_BIT);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't enable OTG mode rc=%d\n", rc);
	else
		chip->pmic_spmi.otg_enable_time = ktime_get();
	pr_smb(PR_STATUS, "Enabling OTG Boost\n");
	return rc;
}

static int smbchg_otg_regulator_disable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct oplus_chg_chip *chip = rdev_get_drvdata(rdev);

	if (!chip->pmic_spmi.otg_pinctrl) {
		rc = smbchg_masked_write(chip, chip->pmic_spmi.bat_if_base + CMD_CHG_REG,
				OTG_EN_BIT, 0);
		if (rc < 0)
			dev_err(chip->dev, "Couldn't disable OTG mode rc=%d\n",
					rc);
	}

	chip->pmic_spmi.chg_otg_enabled = false;
	smbchg_otg_pulse_skip_disable(chip, REASON_OTG_ENABLED, false);
	smbchg_icl_loop_disable_check(chip);
	pr_smb(PR_STATUS, "Disabling OTG Boost\n");
	return rc;
}

static int smbchg_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	int rc = 0;
	u8 reg = 0;
	struct oplus_chg_chip *chip = rdev_get_drvdata(rdev);

	rc = smbchg_read(chip, &reg, chip->pmic_spmi.bat_if_base + CMD_CHG_REG, 1);
	if (rc < 0) {
		dev_err(chip->dev,
				"Couldn't read OTG enable bit rc=%d\n", rc);
		return rc;
	}

	return (reg & OTG_EN_BIT) ? 1 : 0;
}

struct regulator_ops smbchg_otg_reg_ops = {
	.enable		= smbchg_otg_regulator_enable,
	.disable	= smbchg_otg_regulator_disable,
	.is_enabled	= smbchg_otg_regulator_is_enable,
};

#define USBIN_CHGR_CFG			0xF1
#define ADAPTER_ALLOWANCE_MASK		0x7
#define USBIN_ADAPTER_9V		0x3
#define USBIN_ADAPTER_5V_9V_CONT	0x2
#define USBIN_ADAPTER_5V_UNREGULATED_9V	0x5
#define HVDCP_EN_BIT			BIT(3)
static int smbchg_external_otg_regulator_enable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct oplus_chg_chip *chip = rdev_get_drvdata(rdev);

	rc = vote(chip->pmic_spmi.usb_suspend_votable, OTG_EN_VOTER, true, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't suspend charger rc=%d\n", rc);
		return rc;
	}

	rc = smbchg_read(chip, &chip->pmic_spmi.original_usbin_allowance,
			chip->pmic_spmi.usb_chgpth_base + USBIN_CHGR_CFG, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb allowance rc=%d\n", rc);
		return rc;
	}

	/*
	 * To disallow source detect and usbin_uv interrupts, set the adapter
	 * allowance to 9V, so that the audio boost operating in reverse never
	 * gets detected as a valid input
	 */
	rc = smbchg_sec_masked_write(chip,
				chip->pmic_spmi.usb_chgpth_base + CHGPTH_CFG,
				HVDCP_EN_BIT, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't disable HVDCP rc=%d\n", rc);
		return rc;
	}

	rc = smbchg_sec_masked_write(chip,
				chip->pmic_spmi.usb_chgpth_base + USBIN_CHGR_CFG,
				0xFF, USBIN_ADAPTER_9V);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't write usb allowance rc=%d\n", rc);
		return rc;
	}

	pr_smb(PR_STATUS, "Enabling OTG Boost\n");
	return rc;
}

static int smbchg_external_otg_regulator_disable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct oplus_chg_chip *chip = rdev_get_drvdata(rdev);

	rc = vote(chip->pmic_spmi.usb_suspend_votable, OTG_EN_VOTER, false, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't unsuspend charger rc=%d\n", rc);
		return rc;
	}

	/*
	 * Reenable HVDCP and set the adapter allowance back to the original
	 * value in order to allow normal USBs to be recognized as a valid
	 * input.
	 */
	rc = smbchg_sec_masked_write(chip,
				chip->pmic_spmi.usb_chgpth_base + CHGPTH_CFG,
				HVDCP_EN_BIT, HVDCP_EN_BIT);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't enable HVDCP rc=%d\n", rc);
		return rc;
	}

	rc = smbchg_sec_masked_write(chip,
				chip->pmic_spmi.usb_chgpth_base + USBIN_CHGR_CFG,
				0xFF, chip->pmic_spmi.original_usbin_allowance);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't write usb allowance rc=%d\n", rc);
		return rc;
	}

	pr_smb(PR_STATUS, "Disabling OTG Boost\n");
	return rc;
}

static int smbchg_external_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	struct oplus_chg_chip *chip = rdev_get_drvdata(rdev);

	return get_client_vote(chip->pmic_spmi.usb_suspend_votable, OTG_EN_VOTER);
}

struct regulator_ops smbchg_external_otg_reg_ops = {
	.enable		= smbchg_external_otg_regulator_enable,
	.disable	= smbchg_external_otg_regulator_disable,
	.is_enabled	= smbchg_external_otg_regulator_is_enable,
};

static int smbchg_regulator_init(struct oplus_chg_chip *chip)
{
	int rc = 0;
	struct regulator_init_data *init_data;
	struct regulator_config cfg = {};
	struct device_node *regulator_node;

	regulator_node = of_get_child_by_name(chip->dev->of_node,
			"qcom,smbcharger-boost-otg");

	init_data = of_get_regulator_init_data(chip->dev, regulator_node);
	if (!init_data) {
		dev_err(chip->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	if (init_data->constraints.name) {
		chip->pmic_spmi.otg_vreg.rdesc.owner = THIS_MODULE;
		chip->pmic_spmi.otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		chip->pmic_spmi.otg_vreg.rdesc.ops = &smbchg_otg_reg_ops;
		chip->pmic_spmi.otg_vreg.rdesc.name = init_data->constraints.name;

		cfg.dev = chip->dev;
		cfg.init_data = init_data;
		cfg.driver_data = chip;
		cfg.of_node = regulator_node;

		init_data->constraints.valid_ops_mask
			|= REGULATOR_CHANGE_STATUS;

		chip->pmic_spmi.otg_vreg.rdev = devm_regulator_register(chip->dev,
						&chip->pmic_spmi.otg_vreg.rdesc, &cfg);
		if (IS_ERR(chip->pmic_spmi.otg_vreg.rdev)) {
			rc = PTR_ERR(chip->pmic_spmi.otg_vreg.rdev);
			chip->pmic_spmi.otg_vreg.rdev = NULL;
			if (rc != -EPROBE_DEFER)
				dev_err(chip->dev,
					"OTG reg failed, rc=%d\n", rc);
		}
	}

	if (rc)
		return rc;

	regulator_node = of_get_child_by_name(chip->dev->of_node,
			"qcom,smbcharger-external-otg");
	if (!regulator_node) {
		dev_dbg(chip->dev, "external-otg node absent\n");
		return 0;
	}
	init_data = of_get_regulator_init_data(chip->dev, regulator_node);
	if (!init_data) {
		dev_err(chip->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	if (init_data->constraints.name) {
		if (of_get_property(chip->dev->of_node,
					"otg-parent-supply", NULL))
			init_data->supply_regulator = "otg-parent";
		chip->pmic_spmi.ext_otg_vreg.rdesc.owner = THIS_MODULE;
		chip->pmic_spmi.ext_otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		chip->pmic_spmi.ext_otg_vreg.rdesc.ops = &smbchg_external_otg_reg_ops;
		chip->pmic_spmi.ext_otg_vreg.rdesc.name = init_data->constraints.name;

		cfg.dev = chip->dev;
		cfg.init_data = init_data;
		cfg.driver_data = chip;
		cfg.of_node = regulator_node;

		init_data->constraints.valid_ops_mask
			|= REGULATOR_CHANGE_STATUS;

		chip->pmic_spmi.ext_otg_vreg.rdev = devm_regulator_register(chip->dev,
						&chip->pmic_spmi.ext_otg_vreg.rdesc,
						&cfg);
		if (IS_ERR(chip->pmic_spmi.ext_otg_vreg.rdev)) {
			rc = PTR_ERR(chip->pmic_spmi.ext_otg_vreg.rdev);
			chip->pmic_spmi.ext_otg_vreg.rdev = NULL;
			if (rc != -EPROBE_DEFER)
				dev_err(chip->dev,
					"external OTG reg failed, rc=%d\n", rc);
		}
	}

	return rc;
}

#define CMD_CHG_LED_REG		0x43
#define CHG_LED_CTRL_BIT		BIT(0)
#define LED_SW_CTRL_BIT		0x1
#define LED_CHG_CTRL_BIT		0x0
#define CHG_LED_ON		0x03
#define CHG_LED_OFF		0x00
#define LED_BLINKING_PATTERN1		0x01
#define LED_BLINKING_PATTERN2		0x02
#define LED_BLINKING_CFG_MASK		SMB_MASK(2, 1)
#define CHG_LED_SHIFT		1
static int smbchg_chg_led_controls(struct oplus_chg_chip *chip)
{
	u8 reg, mask;
	int rc;

	if (chip->pmic_spmi.cfg_chg_led_sw_ctrl) {
		/* turn-off LED by default for software control */
		mask = CHG_LED_CTRL_BIT | LED_BLINKING_CFG_MASK;
		reg = LED_SW_CTRL_BIT;
	} else {
		mask = CHG_LED_CTRL_BIT;
		reg = LED_CHG_CTRL_BIT;
	}

	rc = smbchg_masked_write(chip, chip->pmic_spmi.bat_if_base + CMD_CHG_LED_REG,
			mask, reg);
	if (rc < 0)
		dev_err(chip->dev,
				"Couldn't write LED_CTRL_BIT rc=%d\n", rc);
	return rc;
}

static void smbchg_chg_led_brightness_set(struct led_classdev *cdev,
		enum led_brightness value)
{
	struct qcom_pmic *pmic_chip = container_of(cdev,
			struct qcom_pmic, led_cdev);
	struct oplus_chg_chip *chip = container_of(pmic_chip,
			struct oplus_chg_chip, pmic_spmi);
	u8 reg;
	int rc;

	reg = (value > LED_OFF) ? CHG_LED_ON << CHG_LED_SHIFT :
		CHG_LED_OFF << CHG_LED_SHIFT;

	if (value > LED_OFF)
		power_supply_set_hi_power_state(chip->pmic_spmi.bms_psy, 1);
	else
		power_supply_set_hi_power_state(chip->pmic_spmi.bms_psy, 0);

	pr_smb(PR_STATUS,
			"set the charger led brightness to value=%d\n",
			value);
	rc = smbchg_sec_masked_write(chip,
			chip->pmic_spmi.bat_if_base + CMD_CHG_LED_REG,
			LED_BLINKING_CFG_MASK, reg);
	if (rc)
		dev_err(chip->dev, "Couldn't write CHG_LED rc=%d\n",
				rc);
}

static enum
led_brightness smbchg_chg_led_brightness_get(struct led_classdev *cdev)
{
	struct qcom_pmic *pmic_chip = container_of(cdev,
			struct qcom_pmic, led_cdev);
	struct oplus_chg_chip *chip = container_of(pmic_chip,
			struct oplus_chg_chip, pmic_spmi);
	u8 reg_val, chg_led_sts;
	int rc;

	rc = smbchg_read(chip, &reg_val, chip->pmic_spmi.bat_if_base + CMD_CHG_LED_REG,
			1);
	if (rc < 0) {
		dev_err(chip->dev,
				"Couldn't read CHG_LED_REG sts rc=%d\n",
				rc);
		return rc;
	}

	chg_led_sts = (reg_val & LED_BLINKING_CFG_MASK) >> CHG_LED_SHIFT;

	pr_smb(PR_STATUS, "chg_led_sts = %02x\n", chg_led_sts);

	return (chg_led_sts == CHG_LED_OFF) ? LED_OFF : LED_FULL;
}

static void smbchg_chg_led_blink_set(struct oplus_chg_chip *chip,
		unsigned long blinking)
{
	u8 reg;
	int rc;

	if (blinking == 0) {
		reg = CHG_LED_OFF << CHG_LED_SHIFT;
		power_supply_set_hi_power_state(chip->pmic_spmi.bms_psy, 0);
	} else {
		power_supply_set_hi_power_state(chip->pmic_spmi.bms_psy, 1);
		if (blinking == 1)
			reg = LED_BLINKING_PATTERN1 << CHG_LED_SHIFT;
		else if (blinking == 2)
			reg = LED_BLINKING_PATTERN2 << CHG_LED_SHIFT;
		else
			reg = LED_BLINKING_PATTERN1 << CHG_LED_SHIFT;
	}

	rc = smbchg_sec_masked_write(chip,
			chip->pmic_spmi.bat_if_base + CMD_CHG_LED_REG,
			LED_BLINKING_CFG_MASK, reg);
	if (rc)
		dev_err(chip->dev, "Couldn't write CHG_LED rc=%d\n",
				rc);
}

static ssize_t smbchg_chg_led_blink_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct qcom_pmic *pmic_chip = container_of(cdev,
			struct qcom_pmic, led_cdev);
	struct oplus_chg_chip *chip = container_of(pmic_chip,
			struct oplus_chg_chip, pmic_spmi);
	unsigned long blinking;
	ssize_t rc = -EINVAL;

	rc = kstrtoul(buf, 10, &blinking);
	if (rc)
		return rc;

	smbchg_chg_led_blink_set(chip, blinking);

	return len;
}

static DEVICE_ATTR(blink, 0664, NULL, smbchg_chg_led_blink_store);

static struct attribute *led_blink_attributes[] = {
	&dev_attr_blink.attr,
	NULL,
};

static struct attribute_group smbchg_led_attr_group = {
	.attrs = led_blink_attributes
};

static int smbchg_register_chg_led(struct oplus_chg_chip *chip)
{
	int rc;

	chip->pmic_spmi.led_cdev.name = "red";
	chip->pmic_spmi.led_cdev.brightness_set = smbchg_chg_led_brightness_set;
	chip->pmic_spmi.led_cdev.brightness_get = smbchg_chg_led_brightness_get;

	rc = led_classdev_register(chip->dev, &chip->pmic_spmi.led_cdev);
	if (rc) {
		dev_err(chip->dev, "unable to register charger led, rc=%d\n",
				rc);
		return rc;
	}

	rc = sysfs_create_group(&chip->pmic_spmi.led_cdev.dev->kobj,
			&smbchg_led_attr_group);
	if (rc) {
		dev_err(chip->dev, "led sysfs rc: %d\n", rc);
		return rc;
	}

	return rc;
}

static int vf_adjust_low_threshold = 5;
module_param(vf_adjust_low_threshold, int, 0644);

static int vf_adjust_high_threshold = 7;
module_param(vf_adjust_high_threshold, int, 0644);

static int vf_adjust_n_samples = 10;
module_param(vf_adjust_n_samples, int, 0644);

static int vf_adjust_max_delta_mv = 40;
module_param(vf_adjust_max_delta_mv, int, 0644);

static int vf_adjust_trim_steps_per_adjust = 1;
module_param(vf_adjust_trim_steps_per_adjust, int, 0644);

#define CENTER_TRIM_CODE		7
#define MAX_LIN_CODE			14
#define MAX_TRIM_CODE			15
#define SCALE_SHIFT			4
#define VF_TRIM_OFFSET_MASK		SMB_MASK(3, 0)
#define VF_STEP_SIZE_MV			10
#define SCALE_LSB_MV			17
static int smbchg_trim_add_steps(int prev_trim, int delta_steps)
{
	int scale_steps;
	int linear_offset, linear_scale;
	int offset_code = prev_trim & VF_TRIM_OFFSET_MASK;
	int scale_code = (prev_trim & ~VF_TRIM_OFFSET_MASK) >> SCALE_SHIFT;

	if (abs(delta_steps) > 1) {
		pr_smb(PR_STATUS,
			"Cant trim multiple steps delta_steps = %d\n",
			delta_steps);
		return prev_trim;
	}
	if (offset_code <= CENTER_TRIM_CODE)
		linear_offset = offset_code + CENTER_TRIM_CODE;
	else if (offset_code > CENTER_TRIM_CODE)
		linear_offset = MAX_TRIM_CODE - offset_code;

	if (scale_code <= CENTER_TRIM_CODE)
		linear_scale = scale_code + CENTER_TRIM_CODE;
	else if (scale_code > CENTER_TRIM_CODE)
		linear_scale = scale_code - (CENTER_TRIM_CODE + 1);

	/* check if we can accomodate delta steps with just the offset */
	if (linear_offset + delta_steps >= 0
			&& linear_offset + delta_steps <= MAX_LIN_CODE) {
		linear_offset += delta_steps;

		if (linear_offset > CENTER_TRIM_CODE)
			offset_code = linear_offset - CENTER_TRIM_CODE;
		else
			offset_code = MAX_TRIM_CODE - linear_offset;

		return (prev_trim & ~VF_TRIM_OFFSET_MASK) | offset_code;
	}

	/* changing offset cannot satisfy delta steps, change the scale bits */
	scale_steps = delta_steps > 0 ? 1 : -1;

	if (linear_scale + scale_steps < 0
			|| linear_scale + scale_steps > MAX_LIN_CODE) {
		pr_smb(PR_STATUS,
			"Cant trim scale_steps = %d delta_steps = %d\n",
			scale_steps, delta_steps);
		return prev_trim;
	}

	linear_scale += scale_steps;

	if (linear_scale > CENTER_TRIM_CODE)
		scale_code = linear_scale - CENTER_TRIM_CODE;
	else
		scale_code = linear_scale + (CENTER_TRIM_CODE + 1);
	prev_trim = (prev_trim & VF_TRIM_OFFSET_MASK)
		| scale_code << SCALE_SHIFT;

	/*
	 * now that we have changed scale which is a 17mV jump, change the
	 * offset bits (10mV) too so the effective change is just 7mV
	 */
	delta_steps = -1 * delta_steps;

	linear_offset = clamp(linear_offset + delta_steps, 0, MAX_LIN_CODE);
	if (linear_offset > CENTER_TRIM_CODE)
		offset_code = linear_offset - CENTER_TRIM_CODE;
	else
		offset_code = MAX_TRIM_CODE - linear_offset;

	return (prev_trim & ~VF_TRIM_OFFSET_MASK) | offset_code;
}

#define TRIM_14		0xFE
#define VF_TRIM_MASK	0xFF
static int smbchg_adjust_vfloat_mv_trim(struct oplus_chg_chip *chip,
						int delta_mv)
{
	int sign, delta_steps, rc = 0;
	u8 prev_trim, new_trim;
	int i;

	sign = delta_mv > 0 ? 1 : -1;
	delta_steps = (delta_mv + sign * VF_STEP_SIZE_MV / 2)
			/ VF_STEP_SIZE_MV;

	rc = smbchg_read(chip, &prev_trim, chip->pmic_spmi.misc_base + TRIM_14, 1);
	if (rc) {
		dev_err(chip->dev, "Unable to read trim 14: %d\n", rc);
		return rc;
	}

	for (i = 1; i <= abs(delta_steps)
			&& i <= vf_adjust_trim_steps_per_adjust; i++) {
		new_trim = (u8)smbchg_trim_add_steps(prev_trim,
				delta_steps > 0 ? 1 : -1);
		if (new_trim == prev_trim) {
			pr_smb(PR_STATUS,
				"VFloat trim unchanged from %02x\n", prev_trim);
			/* treat no trim change as an error */
			return -EINVAL;
		}

		rc = smbchg_sec_masked_write(chip, chip->pmic_spmi.misc_base + TRIM_14,
				VF_TRIM_MASK, new_trim);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't change vfloat trim rc=%d\n", rc);
		}
		pr_smb(PR_STATUS,
			"VFlt trim %02x to %02x, delta steps: %d\n",
			prev_trim, new_trim, delta_steps);
		prev_trim = new_trim;
	}

	return rc;
}

#define VFLOAT_RESAMPLE_DELAY_MS	10000
static void smbchg_vfloat_adjust_work(struct work_struct *work)
{
		struct qcom_pmic *pmic_chip = container_of(work,
				struct qcom_pmic,
				vfloat_adjust_work.work);
	struct oplus_chg_chip *chip = container_of(pmic_chip,
				struct oplus_chg_chip,
				pmic_spmi);
	int vbat_uv, vbat_mv, ibat_ua, rc, delta_vfloat_mv;
	bool taper, enable;

	smbchg_stay_awake(chip, PM_REASON_VFLOAT_ADJUST);
	taper = (get_prop_charge_type(chip)
		== POWER_SUPPLY_CHARGE_TYPE_TAPER);
	enable = taper && (chip->pmic_spmi.parallel.current_max_ma == 0);

	if (!enable) {
		pr_smb(PR_MISC,
			"Stopping vfloat adj taper=%d parallel_ma = %d\n",
			taper, chip->pmic_spmi.parallel.current_max_ma);
		goto stop;
	}

	if (get_prop_batt_health(chip) != POWER_SUPPLY_HEALTH_GOOD) {
		pr_smb(PR_STATUS, "JEITA active, skipping\n");
		goto stop;
	}

	set_property_on_fg(chip, POWER_SUPPLY_PROP_UPDATE_NOW, 1);
	rc = get_property_from_fg(chip,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &vbat_uv);
	if (rc) {
		pr_smb(PR_STATUS,
			"bms psy does not support voltage rc = %d\n", rc);
		goto stop;
	}
	vbat_mv = vbat_uv / 1000;

	if ((vbat_mv - chip->pmic_spmi.vfloat_mv) < -1 * vf_adjust_max_delta_mv) {
		pr_smb(PR_STATUS, "Skip vbat out of range: %d\n", vbat_mv);
		goto reschedule;
	}

	rc = get_property_from_fg(chip,
			POWER_SUPPLY_PROP_CURRENT_NOW, &ibat_ua);
	if (rc) {
		pr_smb(PR_STATUS,
			"bms psy does not support current_now rc = %d\n", rc);
		goto stop;
	}

	if (ibat_ua / 1000 > -chip->pmic_spmi.iterm_ma) {
		pr_smb(PR_STATUS, "Skip ibat too high: %d\n", ibat_ua);
		goto reschedule;
	}

	pr_smb(PR_STATUS, "sample number = %d vbat_mv = %d ibat_ua = %d\n",
		chip->pmic_spmi.n_vbat_samples,
		vbat_mv,
		ibat_ua);

	chip->pmic_spmi.max_vbat_sample = max(chip->pmic_spmi.max_vbat_sample, vbat_mv);
	chip->pmic_spmi.n_vbat_samples += 1;
	if (chip->pmic_spmi.n_vbat_samples < vf_adjust_n_samples) {
		pr_smb(PR_STATUS, "Skip %d samples; max = %d\n",
			chip->pmic_spmi.n_vbat_samples, chip->pmic_spmi.max_vbat_sample);
		goto reschedule;
	}
	/* if max vbat > target vfloat, delta_vfloat_mv could be negative */
	delta_vfloat_mv = chip->pmic_spmi.vfloat_mv - chip->pmic_spmi.max_vbat_sample;
	pr_smb(PR_STATUS, "delta_vfloat_mv = %d, samples = %d, mvbat = %d\n",
		delta_vfloat_mv, chip->pmic_spmi.n_vbat_samples, chip->pmic_spmi.max_vbat_sample);
	/*
	 * enough valid samples has been collected, adjust trim codes
	 * based on maximum of collected vbat samples if necessary
	 */
	if (delta_vfloat_mv > vf_adjust_high_threshold
			|| delta_vfloat_mv < -1 * vf_adjust_low_threshold) {
		rc = smbchg_adjust_vfloat_mv_trim(chip, delta_vfloat_mv);
		if (rc) {
			pr_smb(PR_STATUS,
				"Stopping vfloat adj after trim adj rc = %d\n",
				 rc);
			goto stop;
		}
		chip->pmic_spmi.max_vbat_sample = 0;
		chip->pmic_spmi.n_vbat_samples = 0;
		goto reschedule;
	}

stop:
	chip->pmic_spmi.max_vbat_sample = 0;
	chip->pmic_spmi.n_vbat_samples = 0;
	smbchg_relax(chip, PM_REASON_VFLOAT_ADJUST);
	return;

reschedule:
	schedule_delayed_work(&chip->pmic_spmi.vfloat_adjust_work,
			msecs_to_jiffies(VFLOAT_RESAMPLE_DELAY_MS));
	return;
}

static int smbchg_charging_status_change(struct oplus_chg_chip *chip)
{
	smbchg_vfloat_adjust_check(chip);
	set_property_on_fg(chip, POWER_SUPPLY_PROP_STATUS,
			get_prop_batt_status(chip));
	return 0;
}

#define BB_CLMP_SEL		0xF8
#define BB_CLMP_MASK		SMB_MASK(1, 0)
#define BB_CLMP_VFIX_3338MV	0x1
#define BB_CLMP_VFIX_3512MV	0x2
static int smbchg_set_optimal_charging_mode(struct oplus_chg_chip *chip, int type)
{
	int rc;
	bool hvdcp2 = (type == POWER_SUPPLY_TYPE_USB_HVDCP
			&& smbchg_is_usbin_active_pwr_src(chip));

	/*
	 * Set the charger switching freq to 1MHZ if HVDCP 2.0,
	 * or 750KHZ otherwise
	 */
	rc = smbchg_sec_masked_write(chip,
			chip->pmic_spmi.bat_if_base + BAT_IF_TRIM7_REG,
			CFG_750KHZ_BIT, hvdcp2 ? 0 : CFG_750KHZ_BIT);
	if (rc) {
		dev_err(chip->dev, "Cannot set switching freq: %d\n", rc);
		return rc;
	}

	/*
	 * Set the charger switch frequency clamp voltage threshold to 3.338V
	 * if HVDCP 2.0, or 3.512V otherwise.
	 */
	rc = smbchg_sec_masked_write(chip, chip->pmic_spmi.bat_if_base + BB_CLMP_SEL,
			BB_CLMP_MASK,
			hvdcp2 ? BB_CLMP_VFIX_3338MV : BB_CLMP_VFIX_3512MV);
	if (rc) {
		dev_err(chip->dev, "Cannot set switching freq: %d\n", rc);
		return rc;
	}

	return 0;
}

static int smbchg_set_primal_charger_type
				(struct power_supply *psy,int primal_type)
{
	const union power_supply_propval ret = {primal_type,};

	if (psy->set_property)
		return psy->set_property(psy, POWER_SUPPLY_PROP_PRIMAL_PROPERTY, &ret);
	else
		return -EINVAL;
}

#define DEFAULT_SDP_MA		100
#define DEFAULT_CDP_MA		1500
#define TYPE_UNKNOWN_MA		350
static int smbchg_change_usb_supply_type(struct oplus_chg_chip *chip,
						enum power_supply_type type)
{
	int rc, current_limit_ma;

	/*
	 * if the type is not unknown, set the type before changing ICL vote
	 * in order to ensure that the correct current limit registers are
	 * used
	 */
	if (type != POWER_SUPPLY_TYPE_UNKNOWN)
		chip->pmic_spmi.usb_supply_type = type;

	/*
	 * Type-C only supports STD(900), MEDIUM(1500) and HIGH(3000) current
	 * modes, skip all BC 1.2 current if external typec is supported.
	 * Note: for SDP supporting current based on USB notifications.
	 */
	if (chip->pmic_spmi.typec_psy && (type != POWER_SUPPLY_TYPE_USB))
		current_limit_ma = chip->pmic_spmi.typec_current_ma;
	else if (type == POWER_SUPPLY_TYPE_USB)
		current_limit_ma = DEFAULT_SDP_MA;
	else if (type == POWER_SUPPLY_TYPE_USB)
		current_limit_ma = DEFAULT_SDP_MA;
	else if (type == POWER_SUPPLY_TYPE_USB_CDP)
		current_limit_ma = DEFAULT_CDP_MA;
	else if (type == POWER_SUPPLY_TYPE_USB_HVDCP)
		current_limit_ma = smbchg_default_hvdcp_icl_ma;
	else if (type == POWER_SUPPLY_TYPE_USB_HVDCP_3)
		current_limit_ma = smbchg_default_hvdcp3_icl_ma;
	else if (type == POWER_SUPPLY_TYPE_UNKNOWN)		//lfc add for plug in usb -> phone reboot -> plug out usb -> plug in dcp, current is only 700ma
		current_limit_ma = TYPE_UNKNOWN_MA;
	else
		current_limit_ma = smbchg_default_dcp_icl_ma;

	pr_smb(PR_STATUS, "Type %d: setting mA = %d\n",
		type, current_limit_ma);
#if 0	//lfc delete because charge_en false and fastchg_current not set here
	rc = vote(chip->pmic_spmi.usb_icl_votable, PSY_ICL_VOTER, true,
				current_limit_ma);
	if (rc < 0) {
		pr_err("Couldn't vote for new USB ICL rc=%d\n", rc);
		goto out;
	}
#endif
	if (!chip->pmic_spmi.skip_usb_notification) {
		power_supply_set_supply_type(chip->usb_psy, type);
		smbchg_set_primal_charger_type(chip->usb_psy, chip->pmic_spmi.primal_chgtype);
	}
	/*
	 * otherwise if it is unknown, remove vote
	 * and set type after the vote
	 */
	if (type == POWER_SUPPLY_TYPE_UNKNOWN) {
#if 0
		rc = vote(chip->pmic_spmi.usb_icl_votable, PSY_ICL_VOTER, false,
				current_limit_ma);
#else
		rc = vote(chip->pmic_spmi.usb_icl_votable, PSY_ICL_VOTER, true,
				current_limit_ma);
#endif
		if (rc < 0)
			pr_err("Couldn't remove ICL vote rc=%d\n", rc);

		chip->pmic_spmi.usb_supply_type = type;
	}

	/* set the correct buck switching frequency */
	rc = smbchg_set_optimal_charging_mode(chip, type);
	if (rc < 0)
		pr_err("Couldn't set charger optimal mode rc=%d\n", rc);

//out:
	return rc;
}

#define HVDCP_ADAPTER_SEL_MASK	SMB_MASK(5, 4)
#define HVDCP_5V		0x00
#define HVDCP_9V		0x10
#define USB_CMD_HVDCP_1		0x42
#define FORCE_HVDCP_2p0		BIT(3)

static int force_9v_hvdcp(struct oplus_chg_chip *chip)
{
	int rc;

	/* Force 5V HVDCP */
	rc = smbchg_sec_masked_write(chip,
			chip->pmic_spmi.usb_chgpth_base + CHGPTH_CFG,
			HVDCP_ADAPTER_SEL_MASK, HVDCP_5V);
	if (rc) {
		pr_err("Couldn't set hvdcp config in chgpath_chg rc=%d\n", rc);
		return rc;
	}

	/* Force QC2.0 */
	rc = smbchg_masked_write(chip,
			chip->pmic_spmi.usb_chgpth_base + USB_CMD_HVDCP_1,
			FORCE_HVDCP_2p0, FORCE_HVDCP_2p0);
	rc |= smbchg_masked_write(chip,
			chip->pmic_spmi.usb_chgpth_base + USB_CMD_HVDCP_1,
			FORCE_HVDCP_2p0, 0);
	if (rc < 0) {
		pr_err("Couldn't force QC2.0 rc=%d\n", rc);
		return rc;
	}

	/* Delay to switch into HVDCP 2.0 and avoid UV */
	msleep(500);

	/* Force 9V HVDCP */
	rc = smbchg_sec_masked_write(chip,
			chip->pmic_spmi.usb_chgpth_base + CHGPTH_CFG,
			HVDCP_ADAPTER_SEL_MASK, HVDCP_9V);
	if (rc)
		pr_err("Couldn't set hvdcp config in chgpath_chg rc=%d\n", rc);

	return rc;
}

static void smbchg_hvdcp_det_work(struct work_struct *work)
{
	struct qcom_pmic *pmic_chip = container_of(work,
				struct qcom_pmic,
				hvdcp_det_work.work);
	struct oplus_chg_chip *chip = container_of(pmic_chip,
				struct oplus_chg_chip,
				pmic_spmi);
	int rc;

	if (is_hvdcp_present(chip)) {
		if (!chip->pmic_spmi.hvdcp3_supported &&
			(chip->pmic_spmi.wa_flags & SMBCHG_HVDCP_9V_EN_WA)) {
			/* force HVDCP 2.0 */
			rc = force_9v_hvdcp(chip);
			if (rc)
				pr_err("could not force 9V HVDCP continuing rc=%d\n",
						rc);
		}
		smbchg_change_usb_supply_type(chip,
				POWER_SUPPLY_TYPE_USB_HVDCP);
		if (chip->pmic_spmi.psy_registered)
			power_supply_changed(&chip->batt_psy);
		smbchg_aicl_deglitch_wa_check(chip);
	}
	smbchg_relax(chip, PM_DETECT_HVDCP);
}

static int set_usb_psy_dp_dm(struct oplus_chg_chip *chip, int state)
{
	int rc;
	u8 reg;

	/*
	 * ensure that we are not in the middle of an insertion where usbin_uv
	 * is low and src_detect hasnt gone high. If so force dp=F dm=F
	 * which guarantees proper type detection
	 */
	rc = smbchg_read(chip, &reg, chip->pmic_spmi.usb_chgpth_base + RT_STS, 1);
	if (!rc && !(reg & USBIN_UV_BIT) && !(reg & USBIN_SRC_DET_BIT)) {
		pr_smb(PR_MISC, "overwriting state = %d with %d\n",
				state, POWER_SUPPLY_DP_DM_DPF_DMF);
		state = POWER_SUPPLY_DP_DM_DPF_DMF;
	}
	pr_smb(PR_MISC, "setting usb psy dp dm = %d\n", state);
	return power_supply_set_dp_dm(chip->usb_psy, state);
}

#define APSD_CFG		0xF5
#define AUTO_SRC_DETECT_EN_BIT	BIT(0)
#define APSD_TIMEOUT_MS		1500
static void restore_from_hvdcp_detection(struct oplus_chg_chip *chip)
{
	int rc;

	/* switch to 9V HVDCP */
	rc = smbchg_sec_masked_write(chip, chip->pmic_spmi.usb_chgpth_base + CHGPTH_CFG,
				HVDCP_ADAPTER_SEL_MASK, HVDCP_9V);
	if (rc < 0)
		pr_err("Couldn't configure HVDCP 9V rc=%d\n", rc);

	/* enable HVDCP */
	rc = smbchg_sec_masked_write(chip,
				chip->pmic_spmi.usb_chgpth_base + CHGPTH_CFG,
				HVDCP_EN_BIT, HVDCP_EN_BIT);
	if (rc < 0)
		pr_err("Couldn't enable HVDCP rc=%d\n", rc);

	/* enable APSD */
	rc = smbchg_sec_masked_write(chip,
				chip->pmic_spmi.usb_chgpth_base + APSD_CFG,
				AUTO_SRC_DETECT_EN_BIT, AUTO_SRC_DETECT_EN_BIT);
	if (rc < 0)
		pr_err("Couldn't enable APSD rc=%d\n", rc);

	/* Reset back to 5V unregulated */
	rc = smbchg_sec_masked_write(chip,
		chip->pmic_spmi.usb_chgpth_base + USBIN_CHGR_CFG,
		ADAPTER_ALLOWANCE_MASK, USBIN_ADAPTER_5V_UNREGULATED_9V);
	if (rc < 0)
		pr_err("Couldn't write usb allowance rc=%d\n", rc);

	rc = smbchg_sec_masked_write(chip, chip->pmic_spmi.usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, AICL_EN_BIT);
	if (rc < 0)
		pr_err("Couldn't enable AICL rc=%d\n", rc);

	chip->pmic_spmi.hvdcp_3_det_ignore_uv = false;
	chip->pmic_spmi.pulse_cnt = 0;

	if ((chip->pmic_spmi.schg_version == QPNP_SCHG_LITE)
				&& is_hvdcp_present(chip)) {
		pr_smb(PR_MISC, "Forcing 9V HVDCP 2.0\n");
		rc = force_9v_hvdcp(chip);
		if (rc)
			pr_err("Failed to force 9V HVDCP=%d\n",	rc);
	}

	pr_smb(PR_MISC, "Retracting HVDCP vote for ICL\n");
	rc = vote(chip->pmic_spmi.usb_icl_votable, HVDCP_ICL_VOTER, false, 0);
	if (rc < 0)
		pr_err("Couldn't retract HVDCP ICL vote rc=%d\n", rc);
}

#if 0
#define RESTRICTED_CHG_FCC_PERCENT	50
static int smbchg_restricted_charging(struct oplus_chg_chip *chip, bool enable)
{
	int current_table_index, fastchg_current;
	int rc = 0;

	/* If enable, set the fcc to the set point closest
	 * to 50% of the configured fcc while remaining below it
	 */
	current_table_index = find_smaller_in_array(
			chip->pmic_spmi.tables.usb_ilim_ma_table,
			chip->pmic_spmi.cfg_fastchg_current_ma
				* RESTRICTED_CHG_FCC_PERCENT / 100,
			chip->pmic_spmi.tables.usb_ilim_ma_len);
	fastchg_current =
		chip->pmic_spmi.tables.usb_ilim_ma_table[current_table_index];
	rc = vote(chip->pmic_spmi.fcc_votable, RESTRICTED_CHG_FCC_VOTER, enable,
			fastchg_current);

	pr_smb(PR_STATUS, "restricted_charging set to %d\n", enable);
	chip->pmic_spmi.restricted_charging = enable;

	return rc;
}
#endif

static void handle_usb_removal(struct oplus_chg_chip *chip)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);
	int rc;

	pr_smb(PR_STATUS, "triggered\n");
	chg_err("%s\n", __func__);
	smbchg_aicl_deglitch_wa_check(chip);
	/* Clear the OV detected status set before */
	if (chip->pmic_spmi.usb_ov_det)
		chip->pmic_spmi.usb_ov_det = false;
	/* Clear typec current status */
	if (chip->pmic_spmi.typec_psy)
		chip->pmic_spmi.typec_current_ma = 0;
	smbchg_change_usb_supply_type(chip, POWER_SUPPLY_TYPE_UNKNOWN);
	if (!chip->pmic_spmi.skip_usb_notification) {
		chg_err("setting usb psy present = %d\n",
				chip->pmic_spmi.usb_present);
		power_supply_set_present(chip->usb_psy, chip->pmic_spmi.usb_present);
	}
	set_usb_psy_dp_dm(chip, POWER_SUPPLY_DP_DM_DPR_DMR);
	schedule_work(&chip->pmic_spmi.usb_set_online_work);
	pr_smb(PR_MISC, "setting usb psy health UNKNOWN\n");
	rc = power_supply_set_health_state(chip->usb_psy,
			POWER_SUPPLY_HEALTH_UNKNOWN);
	if (rc < 0)
		pr_smb(PR_STATUS,
			"usb psy does not allow updating prop %d rc = %d\n",
			POWER_SUPPLY_HEALTH_UNKNOWN, rc);

	if (parallel_psy && chip->pmic_spmi.parallel_charger_detected)
		power_supply_set_present(parallel_psy, false);
	if (chip->pmic_spmi.parallel.avail && chip->pmic_spmi.aicl_done_irq
			&& chip->pmic_spmi.enable_aicl_wake) {
		disable_irq_wake(chip->pmic_spmi.aicl_done_irq);
		chip->pmic_spmi.enable_aicl_wake = false;
	}
	chip->pmic_spmi.parallel.enabled_once = false;
	chip->pmic_spmi.vbat_above_headroom = false;
	rc = smbchg_masked_write(chip, chip->pmic_spmi.usb_chgpth_base + CMD_IL,
			ICL_OVERRIDE_BIT, 0);
	if (rc < 0)
		pr_err("Couldn't set override rc = %d\n", rc);

	vote(chip->pmic_spmi.usb_icl_votable, WEAK_CHARGER_ICL_VOTER, false, 0);
	chip->pmic_spmi.usb_icl_delta = 0;
	vote(chip->pmic_spmi.usb_icl_votable, SW_AICL_ICL_VOTER, false, 0);
	vote(chip->pmic_spmi.aicl_deglitch_short_votable,
		HVDCP_SHORT_DEGLITCH_VOTER, false, 0);
	if (!chip->pmic_spmi.hvdcp_not_supported)
		restore_from_hvdcp_detection(chip);
}

static bool is_usbin_uv_high(struct oplus_chg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->pmic_spmi.usb_chgpth_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb rt status rc = %d\n", rc);
		return false;
	}
	return reg &= USBIN_UV_BIT;
}

#define HVDCP_NOTIFY_MS		2500
static void handle_usb_insertion(struct oplus_chg_chip *chip)
{
	enum power_supply_type usb_supply_type;
	int rc;
	char *usb_type_name = "null";

	pr_smb(PR_STATUS, "triggered\n");
	/* usb inserted */
#if 0
	read_usb_type(chip, &usb_type_name, &usb_supply_type);
#else
	qpnp_get_charger_type(chip, &usb_type_name, &usb_supply_type, false);
#endif
	chg_err("inserted type = %d (%s)", usb_supply_type, usb_type_name);

	smbchg_aicl_deglitch_wa_check(chip);
	if (chip->pmic_spmi.typec_psy)
		update_typec_status(chip);
	smbchg_change_usb_supply_type(chip, usb_supply_type);
	if (!chip->pmic_spmi.skip_usb_notification) {
		chg_err("setting usb psy present = %d\n",
				chip->pmic_spmi.usb_present);
		power_supply_set_present(chip->usb_psy, chip->pmic_spmi.usb_present);
	}

	/* Notify the USB psy if OV condition is not present */
	if (!chip->pmic_spmi.usb_ov_det) {
		/*
		 * Note that this could still be a very weak charger
		 * if the handle_usb_insertion was triggered from
		 * the falling edge of an USBIN_OV interrupt
		 */
		pr_smb(PR_MISC, "setting usb psy health %s\n",
				chip->pmic_spmi.very_weak_charger
				? "UNSPEC_FAILURE" : "GOOD");
		rc = power_supply_set_health_state(chip->usb_psy,
				chip->pmic_spmi.very_weak_charger
				? POWER_SUPPLY_HEALTH_UNSPEC_FAILURE
				: POWER_SUPPLY_HEALTH_GOOD);
		if (rc < 0)
			pr_smb(PR_STATUS,
				"usb psy does not allow updating prop %d rc = %d\n",
				POWER_SUPPLY_HEALTH_GOOD, rc);
	}
	schedule_work(&chip->pmic_spmi.usb_set_online_work);

	if (!chip->pmic_spmi.hvdcp_not_supported &&
			(usb_supply_type == POWER_SUPPLY_TYPE_USB_DCP)) {
		cancel_delayed_work_sync(&chip->pmic_spmi.hvdcp_det_work);
		smbchg_stay_awake(chip, PM_DETECT_HVDCP);
		schedule_delayed_work(&chip->pmic_spmi.hvdcp_det_work,
					msecs_to_jiffies(HVDCP_NOTIFY_MS));
	}

	smbchg_detect_parallel_charger(chip);

	if (chip->pmic_spmi.parallel.avail && chip->pmic_spmi.aicl_done_irq
			&& !chip->pmic_spmi.enable_aicl_wake) {
		rc = enable_irq_wake(chip->pmic_spmi.aicl_done_irq);
		chip->pmic_spmi.enable_aicl_wake = true;
	}
}

void update_usb_status(struct oplus_chg_chip *chip, bool usb_present, bool force)
{
	mutex_lock(&chip->pmic_spmi.usb_status_lock);
	if (force) {
		chip->pmic_spmi.usb_present = usb_present;
		chip->pmic_spmi.usb_present ? handle_usb_insertion(chip)
			: handle_usb_removal(chip);
		goto unlock;
	}
	if (!chip->pmic_spmi.usb_present && usb_present) {
		smbchg_stay_awake(chip, PM_USB_PRESENT);
		chip->pmic_spmi.usb_present = usb_present;
		handle_usb_insertion(chip);
		oplus_chg_variables_reset(chip, true);
	} else if (chip->pmic_spmi.usb_present && !usb_present) {
		chip->pmic_spmi.usb_present = usb_present;
		handle_usb_removal(chip);
		oplus_chg_variables_reset(chip, false);
		smbchg_relax(chip, PM_USB_PRESENT);
	}

	/* update FG */
	set_property_on_fg(chip, POWER_SUPPLY_PROP_STATUS,
			get_prop_batt_status(chip));
unlock:
	mutex_unlock(&chip->pmic_spmi.usb_status_lock);
}

static int otg_oc_reset(struct oplus_chg_chip *chip)
{
	int rc;

	rc = smbchg_masked_write(chip, chip->pmic_spmi.bat_if_base + CMD_CHG_REG,
						OTG_EN_BIT, 0);
	if (rc)
		pr_err("Failed to disable OTG rc=%d\n", rc);

	msleep(20);

	/*
	 * There is a possibility that an USBID interrupt might have
	 * occurred notifying USB power supply to disable OTG. We
	 * should not enable OTG in such cases.
	 */
	if (!is_otg_present(chip)) {
		pr_smb(PR_STATUS,
			"OTG is not present, not enabling OTG_EN_BIT\n");
		goto out;
	}

	rc = smbchg_masked_write(chip, chip->pmic_spmi.bat_if_base + CMD_CHG_REG,
						OTG_EN_BIT, OTG_EN_BIT);
	if (rc)
		pr_err("Failed to re-enable OTG rc=%d\n", rc);

out:
	return rc;
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

static int get_current_time(unsigned long *now_tm_sec)
{
	struct rtc_time tm;
	struct rtc_device *rtc;
	int rc;

	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (rtc == NULL) {
		pr_err("%s: unable to open rtc device (%s)\n",
			__FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
		return -EINVAL;
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
	rtc_tm_to_time(&tm, now_tm_sec);

close_time:
	rtc_class_close(rtc);
	return rc;
}

#define AICL_IRQ_LIMIT_SECONDS	60
#define AICL_IRQ_LIMIT_COUNT	25
static void increment_aicl_count(struct oplus_chg_chip *chip)
{
	bool bad_charger = false;
	int max_aicl_count, rc;
	u8 reg;
	long elapsed_seconds;
	unsigned long now_seconds;

	pr_smb(PR_INTERRUPT, "aicl count c:%d dgltch:%d first:%ld\n",
			chip->pmic_spmi.aicl_irq_count, chip->pmic_spmi.aicl_deglitch_short,
			chip->pmic_spmi.first_aicl_seconds);

	rc = smbchg_read(chip, &reg,
			chip->pmic_spmi.usb_chgpth_base + ICL_STS_1_REG, 1);
	if (!rc)
		chip->pmic_spmi.aicl_complete = reg & AICL_STS_BIT;
	else
		chip->pmic_spmi.aicl_complete = false;

	if (chip->pmic_spmi.aicl_deglitch_short || chip->pmic_spmi.force_aicl_rerun) {
		if (!chip->pmic_spmi.aicl_irq_count)
			get_current_time(&chip->pmic_spmi.first_aicl_seconds);
		get_current_time(&now_seconds);
		elapsed_seconds = now_seconds
				- chip->pmic_spmi.first_aicl_seconds;

		if (elapsed_seconds > AICL_IRQ_LIMIT_SECONDS) {
			pr_smb(PR_INTERRUPT,
				"resetting: elp:%ld first:%ld now:%ld c=%d\n",
				elapsed_seconds, chip->pmic_spmi.first_aicl_seconds,
				now_seconds, chip->pmic_spmi.aicl_irq_count);
			chip->pmic_spmi.aicl_irq_count = 1;
			get_current_time(&chip->pmic_spmi.first_aicl_seconds);
			return;
		}
		/*
		 * Double the amount of AICLs allowed if parallel charging is
		 * enabled.
		 */
		max_aicl_count = AICL_IRQ_LIMIT_COUNT
			* (chip->pmic_spmi.parallel.avail ? 2 : 1);
		chip->pmic_spmi.aicl_irq_count++;

		if (chip->pmic_spmi.aicl_irq_count > max_aicl_count) {
			pr_smb(PR_INTERRUPT, "elp:%ld first:%ld now:%ld c=%d\n",
				elapsed_seconds, chip->pmic_spmi.first_aicl_seconds,
				now_seconds, chip->pmic_spmi.aicl_irq_count);
			pr_smb(PR_INTERRUPT, "Disable AICL rerun\n");
			chip->pmic_spmi.very_weak_charger = true;
			bad_charger = true;

			/*
			 * Disable AICL rerun since many interrupts were
			 * triggered in a short time
			 */
			/* disable hw aicl */
			rc = vote(chip->pmic_spmi.hw_aicl_rerun_disable_votable,
				WEAK_CHARGER_HW_AICL_VOTER, true, 0);
			if (rc < 0) {
				pr_err("Couldn't disable hw aicl rerun rc=%d\n",
					rc);
				return;
			}

			/* Vote 100mA current limit */
			rc = vote(chip->pmic_spmi.usb_icl_votable, WEAK_CHARGER_ICL_VOTER,
					true, CURRENT_100_MA);
			if (rc < 0) {
				pr_err("Can't vote %d current limit rc=%d\n",
					CURRENT_100_MA, rc);
			}

			chip->pmic_spmi.aicl_irq_count = 0;
		} else if ((get_prop_charge_type(chip) ==
				POWER_SUPPLY_CHARGE_TYPE_FAST) &&
					(reg & AICL_SUSP_BIT)) {
			/*
			 * If the AICL_SUSP_BIT is on, then AICL reruns have
			 * already been disabled. Set the very weak charger
			 * flag so that the driver reports a bad charger
			 * and does not reenable AICL reruns.
			 */
			chip->pmic_spmi.very_weak_charger = true;
			bad_charger = true;
		}
		if (bad_charger) {
			pr_smb(PR_MISC,
				"setting usb psy health UNSPEC_FAILURE\n");
			rc = power_supply_set_health_state(chip->usb_psy,
					POWER_SUPPLY_HEALTH_UNSPEC_FAILURE);
			if (rc)
				pr_err("Couldn't set health on usb psy rc:%d\n",
					rc);
			schedule_work(&chip->pmic_spmi.usb_set_online_work);
		}
	}
}

static int wait_for_usbin_uv(struct oplus_chg_chip *chip, bool high)
{
	int rc;
	int tries = 3;
	struct completion *completion = &chip->pmic_spmi.usbin_uv_lowered;
	bool usbin_uv;

	if (high)
		completion = &chip->pmic_spmi.usbin_uv_raised;

	while (tries--) {
		rc = wait_for_completion_interruptible_timeout(
				completion,
				msecs_to_jiffies(APSD_TIMEOUT_MS));
		if (rc >= 0)
			break;
	}

	usbin_uv = is_usbin_uv_high(chip);

	if (high == usbin_uv)
		return 0;

	pr_err("usbin uv didnt go to a %s state, still at %s, tries = %d, rc = %d\n",
			high ? "risen" : "lowered",
			usbin_uv ? "high" : "low",
			tries, rc);
	return -EINVAL;
}

static int wait_for_src_detect(struct oplus_chg_chip *chip, bool high)
{
	int rc;
	int tries = 3;
	struct completion *completion = &chip->pmic_spmi.src_det_lowered;
	bool src_detect;

	if (high)
		completion = &chip->pmic_spmi.src_det_raised;

	while (tries--) {
		rc = wait_for_completion_interruptible_timeout(
				completion,
				msecs_to_jiffies(APSD_TIMEOUT_MS));
		if (rc >= 0)
			break;
	}

	src_detect = is_src_detect_high(chip);

	if (high == src_detect)
		return 0;

	pr_err("src detect didnt go to a %s state, still at %s, tries = %d, rc = %d\n",
			high ? "risen" : "lowered",
			src_detect ? "high" : "low",
			tries, rc);
	return -EINVAL;
}

static int fake_insertion_removal(struct oplus_chg_chip *chip, bool insertion)
{
	int rc;
	bool src_detect;
	bool usbin_uv;

	if (insertion) {
		reinit_completion(&chip->pmic_spmi.src_det_raised);
		reinit_completion(&chip->pmic_spmi.usbin_uv_lowered);
	} else {
		reinit_completion(&chip->pmic_spmi.src_det_lowered);
		reinit_completion(&chip->pmic_spmi.usbin_uv_raised);
	}

	/* ensure that usbin uv real time status is in the right state */
	usbin_uv = is_usbin_uv_high(chip);
	if (usbin_uv != insertion) {
		pr_err("Skip faking, usbin uv is already %d\n", usbin_uv);
		return -EINVAL;
	}

	/* ensure that src_detect real time status is in the right state */
	src_detect = is_src_detect_high(chip);
	if (src_detect == insertion) {
		pr_err("Skip faking, src detect is already %d\n", src_detect);
		return -EINVAL;
	}

	pr_smb(PR_MISC, "Allow only %s charger\n",
			insertion ? "5-9V" : "9V only");
	rc = smbchg_sec_masked_write(chip,
			chip->pmic_spmi.usb_chgpth_base + USBIN_CHGR_CFG,
			ADAPTER_ALLOWANCE_MASK,
			insertion ?
			USBIN_ADAPTER_5V_9V_CONT : USBIN_ADAPTER_9V);
	if (rc < 0) {
		pr_err("Couldn't write usb allowance rc=%d\n", rc);
		return rc;
	}

	pr_smb(PR_MISC, "Waiting on %s usbin uv\n",
			insertion ? "falling" : "rising");
	rc = wait_for_usbin_uv(chip, !insertion);
	if (rc < 0) {
		pr_err("wait for usbin uv failed rc = %d\n", rc);
		return rc;
	}

	pr_smb(PR_MISC, "Waiting on %s src det\n",
			insertion ? "rising" : "falling");
	rc = wait_for_src_detect(chip, insertion);
	if (rc < 0) {
		pr_err("wait for src detect failed rc = %d\n", rc);
		return rc;
	}

	return 0;
}

static int smbchg_prepare_for_pulsing(struct oplus_chg_chip *chip)
{
	int rc = 0;
	u8 reg;

	/* switch to 5V HVDCP */
	pr_smb(PR_MISC, "Switch to 5V HVDCP\n");
	rc = smbchg_sec_masked_write(chip, chip->pmic_spmi.usb_chgpth_base + CHGPTH_CFG,
				HVDCP_ADAPTER_SEL_MASK, HVDCP_5V);
	if (rc < 0) {
		pr_err("Couldn't configure HVDCP 5V rc=%d\n", rc);
		goto out;
	}

	/* wait for HVDCP to lower to 5V */
	msleep(500);
	/*
	 * Check if the same hvdcp session is in progress. src_det should be
	 * high and that we are still in 5V hvdcp
	 */
	if (!is_src_detect_high(chip)) {
		pr_smb(PR_MISC, "src det low after 500mS sleep\n");
		goto out;
	}

	/* disable HVDCP */
	pr_smb(PR_MISC, "Disable HVDCP\n");
	rc = smbchg_sec_masked_write(chip, chip->pmic_spmi.usb_chgpth_base + CHGPTH_CFG,
			HVDCP_EN_BIT, 0);
	if (rc < 0) {
		pr_err("Couldn't disable HVDCP rc=%d\n", rc);
		goto out;
	}

	pr_smb(PR_MISC, "HVDCP voting for 300mA ICL\n");
	rc = vote(chip->pmic_spmi.usb_icl_votable, HVDCP_ICL_VOTER, true, 300);
	if (rc < 0) {
		pr_err("Couldn't vote for 300mA HVDCP ICL rc=%d\n", rc);
		goto out;
	}

	pr_smb(PR_MISC, "Disable AICL\n");
	smbchg_sec_masked_write(chip, chip->pmic_spmi.usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, 0);

	chip->pmic_spmi.hvdcp_3_det_ignore_uv = true;
	/* fake a removal */
	pr_smb(PR_MISC, "Faking Removal\n");
	rc = fake_insertion_removal(chip, false);
	if (rc < 0) {
		pr_err("Couldn't fake removal HVDCP Removed rc=%d\n", rc);
		goto handle_removal;
	}

	/* disable APSD */
	pr_smb(PR_MISC, "Disabling APSD\n");
	rc = smbchg_sec_masked_write(chip,
				chip->pmic_spmi.usb_chgpth_base + APSD_CFG,
				AUTO_SRC_DETECT_EN_BIT, 0);
	if (rc < 0) {
		pr_err("Couldn't disable APSD rc=%d\n", rc);
		goto out;
	}

	/* fake an insertion */
	pr_smb(PR_MISC, "Faking Insertion\n");
	rc = fake_insertion_removal(chip, true);
	if (rc < 0) {
		pr_err("Couldn't fake insertion rc=%d\n", rc);
		goto handle_removal;
	}
	chip->pmic_spmi.hvdcp_3_det_ignore_uv = false;

	pr_smb(PR_MISC, "Enable AICL\n");
	smbchg_sec_masked_write(chip, chip->pmic_spmi.usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, AICL_EN_BIT);

	set_usb_psy_dp_dm(chip, POWER_SUPPLY_DP_DM_DP0P6_DMF);
	/*
	 * DCP will switch to HVDCP in this time by removing the short
	 * between DP DM
	 */
	msleep(HVDCP_NOTIFY_MS);
	/*
	 * Check if the same hvdcp session is in progress. src_det should be
	 * high and the usb type should be none since APSD was disabled
	 */
	if (!is_src_detect_high(chip)) {
		pr_smb(PR_MISC, "src det low after 2s sleep\n");
		rc = -EINVAL;
		goto out;
	}

	smbchg_read(chip, &reg, chip->pmic_spmi.misc_base + IDEV_STS, 1);
	if ((reg >> TYPE_BITS_OFFSET) != 0) {
		pr_smb(PR_MISC, "type bits set after 2s sleep - abort\n");
		rc = -EINVAL;
		goto out;
	}

	set_usb_psy_dp_dm(chip, POWER_SUPPLY_DP_DM_DP0P6_DM3P3);
	/* Wait 60mS after entering continuous mode */
	msleep(60);

	return 0;
out:
	chip->pmic_spmi.hvdcp_3_det_ignore_uv = false;
	restore_from_hvdcp_detection(chip);
	return rc;
handle_removal:
	chip->pmic_spmi.hvdcp_3_det_ignore_uv = false;
	update_usb_status(chip, 0, 0);
	return rc;
}

static int smbchg_unprepare_for_pulsing(struct oplus_chg_chip *chip)
{
	int rc = 0;

	set_usb_psy_dp_dm(chip, POWER_SUPPLY_DP_DM_DPF_DMF);

	/* switch to 9V HVDCP */
	pr_smb(PR_MISC, "Switch to 9V HVDCP\n");
	rc = smbchg_sec_masked_write(chip, chip->pmic_spmi.usb_chgpth_base + CHGPTH_CFG,
				HVDCP_ADAPTER_SEL_MASK, HVDCP_9V);
	if (rc < 0) {
		pr_err("Couldn't configure HVDCP 9V rc=%d\n", rc);
		return rc;
	}

	/* enable HVDCP */
	pr_smb(PR_MISC, "Enable HVDCP\n");
	rc = smbchg_sec_masked_write(chip,
				chip->pmic_spmi.usb_chgpth_base + CHGPTH_CFG,
				HVDCP_EN_BIT, HVDCP_EN_BIT);
	if (rc < 0) {
		pr_err("Couldn't enable HVDCP rc=%d\n", rc);
		return rc;
	}

	/* enable APSD */
	pr_smb(PR_MISC, "Enabling APSD\n");
	rc = smbchg_sec_masked_write(chip,
				chip->pmic_spmi.usb_chgpth_base + APSD_CFG,
				AUTO_SRC_DETECT_EN_BIT, AUTO_SRC_DETECT_EN_BIT);
	if (rc < 0) {
		pr_err("Couldn't enable APSD rc=%d\n", rc);
		return rc;
	}

	/* Disable AICL */
	pr_smb(PR_MISC, "Disable AICL\n");
	rc = smbchg_sec_masked_write(chip, chip->pmic_spmi.usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, 0);
	if (rc < 0) {
		pr_err("Couldn't disable AICL rc=%d\n", rc);
		return rc;
	}

	/* fake a removal */
	chip->pmic_spmi.hvdcp_3_det_ignore_uv = true;
	pr_smb(PR_MISC, "Faking Removal\n");
	rc = fake_insertion_removal(chip, false);
	if (rc < 0) {
		pr_err("Couldn't fake removal rc=%d\n", rc);
		goto out;
	}

	/*
	 * reset the enabled once flag for parallel charging so
	 * parallel charging can immediately restart after the HVDCP pulsing
	 * is complete
	 */
	chip->pmic_spmi.parallel.enabled_once = false;

	/* fake an insertion */
	pr_smb(PR_MISC, "Faking Insertion\n");
	rc = fake_insertion_removal(chip, true);
	if (rc < 0) {
		pr_err("Couldn't fake insertion rc=%d\n", rc);
		goto out;
	}
	chip->pmic_spmi.hvdcp_3_det_ignore_uv = false;

	/* Enable AICL */
	pr_smb(PR_MISC, "Enable AICL\n");
	rc = smbchg_sec_masked_write(chip, chip->pmic_spmi.usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, 0);
	if (rc < 0) {
		pr_err("Couldn't enable AICL rc=%d\n", rc);
		return rc;
	}

out:
	/*
	 * There are many QC 2.0 chargers that collapse before the aicl deglitch
	 * timer can mitigate. Hence set the aicl deglitch time to a shorter
	 * period.
	 */

	rc = vote(chip->pmic_spmi.aicl_deglitch_short_votable,
		HVDCP_SHORT_DEGLITCH_VOTER, true, 0);
	if (rc < 0)
		pr_err("Couldn't reduce aicl deglitch rc=%d\n", rc);

	pr_smb(PR_MISC, "Retracting HVDCP vote for ICL\n");
	rc = vote(chip->pmic_spmi.usb_icl_votable, HVDCP_ICL_VOTER, false, 0);
	if (rc < 0)
		pr_err("Couldn't retract HVDCP ICL vote rc=%d\n", rc);

	chip->pmic_spmi.hvdcp_3_det_ignore_uv = false;
	if (!is_src_detect_high(chip)) {
		pr_smb(PR_MISC, "HVDCP removed\n");
		update_usb_status(chip, 0, 0);
	}
	return rc;
}

#define USB_CMD_APSD		0x41
#define APSD_RERUN		BIT(0)
static int rerun_apsd(struct oplus_chg_chip *chip)
{
	int rc;

	reinit_completion(&chip->pmic_spmi.src_det_raised);
	reinit_completion(&chip->pmic_spmi.usbin_uv_lowered);
	reinit_completion(&chip->pmic_spmi.src_det_lowered);
	reinit_completion(&chip->pmic_spmi.usbin_uv_raised);

	/* re-run APSD */
	rc = smbchg_masked_write(chip, chip->pmic_spmi.usb_chgpth_base + USB_CMD_APSD,
					APSD_RERUN, APSD_RERUN);
	if (rc) {
		pr_err("Couldn't re-run APSD rc=%d\n", rc);
		return rc;
	}

	pr_smb(PR_MISC, "Waiting on rising usbin uv\n");
	rc = wait_for_usbin_uv(chip, true);
	if (rc < 0) {
		pr_err("wait for usbin uv failed rc = %d\n", rc);
		return rc;
	}

	pr_smb(PR_MISC, "Waiting on falling src det\n");
	rc = wait_for_src_detect(chip, false);
	if (rc < 0) {
		pr_err("wait for src detect failed rc = %d\n", rc);
		return rc;
	}

	pr_smb(PR_MISC, "Waiting on falling usbin uv\n");
	rc = wait_for_usbin_uv(chip, false);
	if (rc < 0) {
		pr_err("wait for usbin uv failed rc = %d\n", rc);
		return rc;
	}

	pr_smb(PR_MISC, "Waiting on rising src det\n");
	rc = wait_for_src_detect(chip, true);
	if (rc < 0) {
		pr_err("wait for src detect failed rc = %d\n", rc);
		return rc;
	}

	return rc;
}

#define SCHG_LITE_USBIN_HVDCP_5_9V		0x8
#define SCHG_LITE_USBIN_HVDCP_5_9V_SEL_MASK	0x38
#define SCHG_LITE_USBIN_HVDCP_SEL_IDLE		BIT(3)
static bool is_hvdcp_5v_cont_mode(struct oplus_chg_chip *chip)
{
	int rc;
	u8 reg = 0;

	rc = smbchg_read(chip, &reg,
		chip->pmic_spmi.usb_chgpth_base + USBIN_HVDCP_STS, 1);
	if (rc) {
		pr_err("Unable to read HVDCP status rc=%d\n", rc);
		return false;
	}

	pr_smb(PR_STATUS, "HVDCP status = %x\n", reg);

	if (reg & SCHG_LITE_USBIN_HVDCP_SEL_IDLE) {
		rc = smbchg_read(chip, &reg,
			chip->pmic_spmi.usb_chgpth_base + INPUT_STS, 1);
		if (rc) {
			pr_err("Unable to read INPUT status rc=%d\n", rc);
			return false;
		}
		pr_smb(PR_STATUS, "INPUT status = %x\n", reg);
		if ((reg & SCHG_LITE_USBIN_HVDCP_5_9V_SEL_MASK) ==
					SCHG_LITE_USBIN_HVDCP_5_9V)
			return true;
	}
	return false;
}

static int smbchg_prepare_for_pulsing_lite(struct oplus_chg_chip *chip)
{
	int rc = 0;

	pr_smb(PR_MISC, "HVDCP voting for 300mA ICL\n");
	rc = vote(chip->pmic_spmi.usb_icl_votable, HVDCP_ICL_VOTER, true, 300);
	if (rc < 0) {
		pr_err("Couldn't vote for 300mA HVDCP ICL rc=%d\n", rc);
		return rc;
	}

	/* check if HVDCP is already in 5V continuous mode */
	if (is_hvdcp_5v_cont_mode(chip)) {
		pr_smb(PR_MISC, "HVDCP by default is in 5V continuous mode\n");
		return 0;
	}

	/* switch to 5V HVDCP */
	pr_smb(PR_MISC, "Switch to 5V HVDCP\n");
	rc = smbchg_sec_masked_write(chip, chip->pmic_spmi.usb_chgpth_base + CHGPTH_CFG,
				HVDCP_ADAPTER_SEL_MASK, HVDCP_5V);
	if (rc < 0) {
		pr_err("Couldn't configure HVDCP 5V rc=%d\n", rc);
		goto out;
	}

	/* wait for HVDCP to lower to 5V */
	msleep(500);
	/*
	 * Check if the same hvdcp session is in progress. src_det should be
	 * high and that we are still in 5V hvdcp
	 */
	if (!is_src_detect_high(chip)) {
		pr_smb(PR_MISC, "src det low after 500mS sleep\n");
		goto out;
	}

	pr_smb(PR_MISC, "Disable AICL\n");
	smbchg_sec_masked_write(chip, chip->pmic_spmi.usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, 0);

	chip->pmic_spmi.hvdcp_3_det_ignore_uv = true;

	/* re-run APSD */
	rc = rerun_apsd(chip);
	if (rc) {
		pr_err("APSD rerun failed\n");
		goto out;
	}

	chip->pmic_spmi.hvdcp_3_det_ignore_uv = false;

	pr_smb(PR_MISC, "Enable AICL\n");
	smbchg_sec_masked_write(chip, chip->pmic_spmi.usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, AICL_EN_BIT);
	/*
	 * DCP will switch to HVDCP in this time by removing the short
	 * between DP DM
	 */
	msleep(HVDCP_NOTIFY_MS);
	/*
	 * Check if the same hvdcp session is in progress. src_det should be
	 * high and the usb type should be none since APSD was disabled
	 */
	if (!is_src_detect_high(chip)) {
		pr_smb(PR_MISC, "src det low after 2s sleep\n");
		rc = -EINVAL;
		goto out;
	}

	/* We are set if HVDCP in 5V continuous mode */
	if (!is_hvdcp_5v_cont_mode(chip)) {
		pr_err("HVDCP could not be set in 5V continuous mode\n");
		goto out;
	}

	return 0;
out:
	chip->pmic_spmi.hvdcp_3_det_ignore_uv = false;
	restore_from_hvdcp_detection(chip);
	return rc;
}

static int smbchg_unprepare_for_pulsing_lite(struct oplus_chg_chip *chip)
{
	int rc = 0;

	pr_smb(PR_MISC, "Forcing 9V HVDCP 2.0\n");
	rc = force_9v_hvdcp(chip);
	if (rc) {
		pr_err("Failed to force 9V HVDCP=%d\n",	rc);
		return rc;
	}

	pr_smb(PR_MISC, "Retracting HVDCP vote for ICL\n");
	rc = vote(chip->pmic_spmi.usb_icl_votable, HVDCP_ICL_VOTER, false, 0);
	if (rc < 0)
		pr_err("Couldn't retract HVDCP ICL vote rc=%d\n", rc);

	return rc;
}

#define CMD_HVDCP_2		0x43
#define SINGLE_INCREMENT	BIT(0)
#define SINGLE_DECREMENT	BIT(1)
static int smbchg_dp_pulse_lite(struct oplus_chg_chip *chip)
{
	int rc = 0;

	pr_smb(PR_MISC, "Increment DP\n");
	rc = smbchg_masked_write(chip, chip->pmic_spmi.usb_chgpth_base + CMD_HVDCP_2,
				SINGLE_INCREMENT, SINGLE_INCREMENT);
	if (rc)
		pr_err("Single-increment failed rc=%d\n", rc);

	return rc;
}

static int smbchg_dm_pulse_lite(struct oplus_chg_chip *chip)
{
	int rc = 0;

	pr_smb(PR_MISC, "Decrement DM\n");
	rc = smbchg_masked_write(chip, chip->pmic_spmi.usb_chgpth_base + CMD_HVDCP_2,
				SINGLE_DECREMENT, SINGLE_DECREMENT);
	if (rc)
		pr_err("Single-decrement failed rc=%d\n", rc);

	return rc;
}

static int smbchg_hvdcp3_confirmed(struct oplus_chg_chip *chip)
{
	int rc = 0;

	/*
	 * reset the enabled once flag for parallel charging because this is
	 * effectively a new insertion.
	 */
	chip->pmic_spmi.parallel.enabled_once = false;

	pr_smb(PR_MISC, "Retracting HVDCP vote for ICL\n");
	rc = vote(chip->pmic_spmi.usb_icl_votable, HVDCP_ICL_VOTER, false, 0);
	if (rc < 0)
		pr_err("Couldn't retract HVDCP ICL vote rc=%d\n", rc);

	smbchg_change_usb_supply_type(chip, POWER_SUPPLY_TYPE_USB_HVDCP_3);

	return rc;
}

static int smbchg_dp_dm(struct oplus_chg_chip *chip, int val)
{
	int rc = 0;
	int target_icl_vote_ma;

	switch (val) {
	case POWER_SUPPLY_DP_DM_PREPARE:
		if (!is_hvdcp_present(chip)) {
			pr_err("No pulsing unless HVDCP\n");
			return -ENODEV;
		}
		if (chip->pmic_spmi.schg_version == QPNP_SCHG_LITE)
			rc = smbchg_prepare_for_pulsing_lite(chip);
		else
			rc = smbchg_prepare_for_pulsing(chip);
		break;
	case POWER_SUPPLY_DP_DM_UNPREPARE:
		if (chip->pmic_spmi.schg_version == QPNP_SCHG_LITE)
			rc = smbchg_unprepare_for_pulsing_lite(chip);
		else
			rc = smbchg_unprepare_for_pulsing(chip);
		break;
	case POWER_SUPPLY_DP_DM_CONFIRMED_HVDCP3:
		rc = smbchg_hvdcp3_confirmed(chip);
		break;
	case POWER_SUPPLY_DP_DM_DP_PULSE:
		if (chip->pmic_spmi.schg_version == QPNP_SCHG)
			rc = set_usb_psy_dp_dm(chip,
					POWER_SUPPLY_DP_DM_DP_PULSE);
		else
			rc = smbchg_dp_pulse_lite(chip);
		if (!rc)
			chip->pmic_spmi.pulse_cnt++;
		pr_smb(PR_MISC, "pulse_cnt = %d\n", chip->pmic_spmi.pulse_cnt);
		break;
	case POWER_SUPPLY_DP_DM_DM_PULSE:
		if (chip->pmic_spmi.schg_version == QPNP_SCHG)
			rc = set_usb_psy_dp_dm(chip,
					POWER_SUPPLY_DP_DM_DM_PULSE);
		else
			rc = smbchg_dm_pulse_lite(chip);
		if (!rc && chip->pmic_spmi.pulse_cnt)
			chip->pmic_spmi.pulse_cnt--;
		pr_smb(PR_MISC, "pulse_cnt = %d\n", chip->pmic_spmi.pulse_cnt);
		break;
	case POWER_SUPPLY_DP_DM_HVDCP3_SUPPORTED:
		chip->pmic_spmi.hvdcp3_supported = true;
		pr_smb(PR_MISC, "HVDCP3 supported\n");
		break;
	case POWER_SUPPLY_DP_DM_ICL_DOWN:
		chip->pmic_spmi.usb_icl_delta -= 100;
		target_icl_vote_ma = get_client_vote(chip->pmic_spmi.usb_icl_votable,
						PSY_ICL_VOTER);
		vote(chip->pmic_spmi.usb_icl_votable, SW_AICL_ICL_VOTER, true,
				target_icl_vote_ma + chip->pmic_spmi.usb_icl_delta);
		break;
	case POWER_SUPPLY_DP_DM_ICL_UP:
		chip->pmic_spmi.usb_icl_delta += 100;
		target_icl_vote_ma = get_client_vote(chip->pmic_spmi.usb_icl_votable,
						PSY_ICL_VOTER);
		vote(chip->pmic_spmi.usb_icl_votable, SW_AICL_ICL_VOTER, true,
				target_icl_vote_ma + chip->pmic_spmi.usb_icl_delta);
		break;
	default:
		break;
	}

	return rc;
}

#if 0
static void update_typec_capability_status(struct oplus_chg_chip *chip,
					const union power_supply_propval *val)
{
	int rc;

	pr_smb(PR_TYPEC, "typec capability = %dma\n", val->intval);

	if (!chip->pmic_spmi.skip_usb_notification) {
		rc = chip->usb_psy->set_property(chip->usb_psy,
				POWER_SUPPLY_PROP_INPUT_CURRENT_MAX, val);
		if (rc)
			pr_err("typec failed to set current max rc=%d\n", rc);
	}

	pr_debug("changing ICL from %dma to %dma\n", chip->pmic_spmi.typec_current_ma,
			val->intval);
	chip->pmic_spmi.typec_current_ma = val->intval;
	smbchg_change_usb_supply_type(chip, chip->pmic_spmi.usb_supply_type);
}
#endif

static void update_typec_otg_status(struct oplus_chg_chip *chip, int mode,
					bool force)
{
	pr_smb(PR_TYPEC, "typec mode = %d\n", mode);

	if (mode == POWER_SUPPLY_TYPE_DFP) {
		chip->pmic_spmi.typec_dfp = true;
		power_supply_set_usb_otg(chip->usb_psy, chip->pmic_spmi.typec_dfp);
		/* update FG */
		set_property_on_fg(chip, POWER_SUPPLY_PROP_STATUS,
				get_prop_batt_status(chip));
	} else if (force || chip->pmic_spmi.typec_dfp) {
		chip->pmic_spmi.typec_dfp = false;
		power_supply_set_usb_otg(chip->usb_psy, chip->pmic_spmi.typec_dfp);
		/* update FG */
		set_property_on_fg(chip, POWER_SUPPLY_PROP_STATUS,
				get_prop_batt_status(chip));
	}
}

#if 0
#define CHARGE_OUTPUT_VTG_RATIO		840
static int smbchg_get_iusb(struct oplus_chg_chip *chip)
{
	int rc, iusb_ua = -EINVAL;
	struct qpnp_vadc_result adc_result;

	if (!is_usb_present(chip) && !is_dc_present(chip))
		return 0;

	if (chip->pmic_spmi.vchg_vadc_dev && chip->pmic_spmi.vchg_adc_channel != -EINVAL) {
		rc = qpnp_vadc_read(chip->pmic_spmi.vchg_vadc_dev,
				chip->pmic_spmi.vchg_adc_channel, &adc_result);
		if (rc) {
			pr_smb(PR_STATUS,
				"error in VCHG (channel-%d) read rc = %d\n",
						chip->pmic_spmi.vchg_adc_channel, rc);
			return 0;
		}
		iusb_ua = div_s64(adc_result.physical * 1000,
						CHARGE_OUTPUT_VTG_RATIO);
	}

	return iusb_ua;
}
#endif

#if 0
static enum power_supply_property smbchg_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL,
	POWER_SUPPLY_PROP_FLASH_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_SAFETY_TIMER_ENABLE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED,
	POWER_SUPPLY_PROP_INPUT_CURRENT_NOW,
	POWER_SUPPLY_PROP_FLASH_ACTIVE,
	POWER_SUPPLY_PROP_FLASH_TRIGGER,
	POWER_SUPPLY_PROP_DP_DM,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMITED,
	POWER_SUPPLY_PROP_RERUN_AICL,
	POWER_SUPPLY_PROP_RESTRICTED_CHARGING,
};

static int smbchg_battery_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	int rc = 0;
	struct oplus_chg_chip *chip = container_of(psy,
				struct oplus_chg_chip, batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		vote(chip->pmic_spmi.battchg_suspend_votable, BATTCHG_USER_EN_VOTER,
				!val->intval, 0);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		rc = vote(chip->pmic_spmi.usb_suspend_votable, USER_EN_VOTER,
				!val->intval, 0);
		rc = vote(chip->pmic_spmi.dc_suspend_votable, USER_EN_VOTER,
				!val->intval, 0);
		chip->pmic_spmi.chg_enabled = val->intval;
		schedule_work(&chip->pmic_spmi.usb_set_online_work);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		chip->pmic_spmi.fake_battery_soc = val->intval;
		power_supply_changed(&chip->batt_psy);
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		smbchg_system_temp_level_set(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		rc = smbchg_set_fastchg_current_user(chip, val->intval / 1000);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		rc = smbchg_float_voltage_set(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_SAFETY_TIMER_ENABLE:
		rc = smbchg_safety_timer_enable(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_FLASH_ACTIVE:
		rc = smbchg_switch_buck_frequency(chip, val->intval);
		if (rc) {
			pr_err("Couldn't switch buck frequency, rc=%d\n", rc);
			/*
			 * Trigger a panic if there is an error while switching
			 * buck frequency. This will prevent LS FET damage.
			 */
			BUG_ON(1);
		}

		rc = smbchg_otg_pulse_skip_disable(chip,
				REASON_FLASH_ENABLED, val->intval);
		break;
	case POWER_SUPPLY_PROP_FLASH_TRIGGER:
		chip->pmic_spmi.flash_triggered = !!val->intval;
		smbchg_icl_loop_disable_check(chip);
		break;
	case POWER_SUPPLY_PROP_FORCE_TLIM:
		rc = smbchg_force_tlim_en(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_DP_DM:
		rc = smbchg_dp_dm(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_RERUN_AICL:
		smbchg_rerun_aicl(chip);
		break;
	case POWER_SUPPLY_PROP_RESTRICTED_CHARGING:
		rc = smbchg_restricted_charging(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_CURRENT_CAPABILITY:
		if (chip->pmic_spmi.typec_psy)
			update_typec_capability_status(chip, val);
		break;
	case POWER_SUPPLY_PROP_TYPEC_MODE:
		if (chip->pmic_spmi.typec_psy)
			update_typec_otg_status(chip, val->intval, false);
		break;
	default:
		return -EINVAL;
	}

	return rc;
}

static int smbchg_battery_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	int rc;

	switch (prop) {
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_SAFETY_TIMER_ENABLE:
	case POWER_SUPPLY_PROP_DP_DM:
	case POWER_SUPPLY_PROP_RERUN_AICL:
	case POWER_SUPPLY_PROP_RESTRICTED_CHARGING:
		rc = 1;
		break;
	default:
		rc = 0;
		break;
	}
	return rc;
}

static int smbchg_battery_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct oplus_chg_chip *chip = container_of(psy,
				struct oplus_chg_chip, batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = get_prop_batt_status(chip);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = get_prop_batt_present(chip);
		break;
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		val->intval =
			!get_effective_result(chip->pmic_spmi.battchg_suspend_votable);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = chip->pmic_spmi.chg_enabled;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = get_prop_charge_type(chip);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = smbchg_float_voltage_get(chip);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = get_prop_batt_health(chip);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_FLASH_CURRENT_MAX:
		val->intval = smbchg_calc_max_flash_current(chip);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = chip->pmic_spmi.fastchg_current_ma * 1000;
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		val->intval = chip->pmic_spmi.therm_lvl_sel;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
		val->intval = smbchg_get_aicl_level_ma(chip) * 1000;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED:
		val->intval = (int)chip->pmic_spmi.aicl_complete;
		break;
	case POWER_SUPPLY_PROP_RESTRICTED_CHARGING:
		val->intval = (int)chip->pmic_spmi.restricted_charging;
		break;
	/* properties from fg */
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = get_prop_batt_capacity(chip);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = get_prop_batt_current_now(chip);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = get_prop_batt_voltage_now(chip);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = get_prop_batt_temp(chip);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = get_prop_batt_voltage_max_design(chip);
		break;
	case POWER_SUPPLY_PROP_SAFETY_TIMER_ENABLE:
		val->intval = chip->pmic_spmi.safety_timer_en;
		break;
	case POWER_SUPPLY_PROP_FLASH_ACTIVE:
		val->intval = chip->pmic_spmi.otg_pulse_skip_dis;
		break;
	case POWER_SUPPLY_PROP_FLASH_TRIGGER:
		val->intval = chip->pmic_spmi.flash_triggered;
		break;
	case POWER_SUPPLY_PROP_DP_DM:
		val->intval = chip->pmic_spmi.pulse_cnt;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMITED:
		val->intval = smbchg_is_input_current_limited(chip);
		break;
	case POWER_SUPPLY_PROP_RERUN_AICL:
		val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_NOW:
		val->intval = smbchg_get_iusb(chip);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static char *smbchg_dc_supplicants[] = {
	"bms",
};

static enum power_supply_property smbchg_dc_properties[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static int smbchg_dc_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	int rc = 0;
	struct qcom_pmic *pmic_chip = container_of(psy,
				struct qcom_pmic, dc_psy);
	struct oplus_chg_chip *chip = container_of(pmic_chip,
				struct oplus_chg_chip, pmic_spmi);

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		rc = vote(chip->pmic_spmi.dc_suspend_votable, POWER_SUPPLY_EN_VOTER,
					!val->intval, 0);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		rc = vote(chip->pmic_spmi.dc_icl_votable, USER_ICL_VOTER, true,
				val->intval / 1000);
		break;
	default:
		return -EINVAL;
	}

	return rc;
}

static int smbchg_dc_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct qcom_pmic *pmic_chip = container_of(psy,
				struct qcom_pmic, dc_psy);
	struct oplus_chg_chip *chip = container_of(pmic_chip,
				struct oplus_chg_chip, pmic_spmi);

	switch (prop) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = is_dc_present(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = !get_effective_result(chip->pmic_spmi.dc_suspend_votable);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		/* return if dc is charging the battery */
		val->intval = (smbchg_get_pwr_path(chip) == PWR_PATH_DC)
				&& (get_prop_batt_status(chip)
					== POWER_SUPPLY_STATUS_CHARGING);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = chip->pmic_spmi.dc_max_current_ma * 1000;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int smbchg_dc_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	int rc;

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		rc = 1;
		break;
	default:
		rc = 0;
		break;
	}
	return rc;
}
#endif

#define HOT_BAT_HARD_BIT	BIT(0)
#define HOT_BAT_SOFT_BIT	BIT(1)
#define COLD_BAT_HARD_BIT	BIT(2)
#define COLD_BAT_SOFT_BIT	BIT(3)
#define BAT_OV_BIT		BIT(4)
#define BAT_LOW_BIT		BIT(5)
#define BAT_MISSING_BIT		BIT(6)
#define BAT_TERM_MISSING_BIT	BIT(7)
static irqreturn_t batt_hot_handler(int irq, void *_chip)
{
	struct oplus_chg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->pmic_spmi.bat_if_base + RT_STS, 1);
	chip->pmic_spmi.batt_hot = !!(reg & HOT_BAT_HARD_BIT);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	smbchg_parallel_usb_check_ok(chip);
	if (chip->pmic_spmi.psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	smbchg_wipower_check(chip);
	set_property_on_fg(chip, POWER_SUPPLY_PROP_HEALTH,
			get_prop_batt_health(chip));
	return IRQ_HANDLED;
}

static irqreturn_t batt_cold_handler(int irq, void *_chip)
{
	struct oplus_chg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->pmic_spmi.bat_if_base + RT_STS, 1);
	chip->pmic_spmi.batt_cold = !!(reg & COLD_BAT_HARD_BIT);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	smbchg_parallel_usb_check_ok(chip);
	if (chip->pmic_spmi.psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	smbchg_wipower_check(chip);
	set_property_on_fg(chip, POWER_SUPPLY_PROP_HEALTH,
			get_prop_batt_health(chip));
	return IRQ_HANDLED;
}

static irqreturn_t batt_warm_handler(int irq, void *_chip)
{
	struct oplus_chg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->pmic_spmi.bat_if_base + RT_STS, 1);
	chip->pmic_spmi.batt_warm = !!(reg & HOT_BAT_SOFT_BIT);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	smbchg_parallel_usb_check_ok(chip);
	if (chip->pmic_spmi.psy_registered)
		power_supply_changed(&chip->batt_psy);
	set_property_on_fg(chip, POWER_SUPPLY_PROP_HEALTH,
			get_prop_batt_health(chip));
	return IRQ_HANDLED;
}

static irqreturn_t batt_cool_handler(int irq, void *_chip)
{
	struct oplus_chg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->pmic_spmi.bat_if_base + RT_STS, 1);
	chip->pmic_spmi.batt_cool = !!(reg & COLD_BAT_SOFT_BIT);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	smbchg_parallel_usb_check_ok(chip);
	if (chip->pmic_spmi.psy_registered)
		power_supply_changed(&chip->batt_psy);
	set_property_on_fg(chip, POWER_SUPPLY_PROP_HEALTH,
			get_prop_batt_health(chip));
	return IRQ_HANDLED;
}

static irqreturn_t batt_pres_handler(int irq, void *_chip)
{
	struct oplus_chg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->pmic_spmi.bat_if_base + RT_STS, 1);
	chip->pmic_spmi.batt_present = !(reg & BAT_MISSING_BIT);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	if (chip->pmic_spmi.psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	set_property_on_fg(chip, POWER_SUPPLY_PROP_HEALTH,
			get_prop_batt_health(chip));
	return IRQ_HANDLED;
}

static irqreturn_t vbat_low_handler(int irq, void *_chip)
{
	pr_warn_ratelimited("vbat low\n");
	return IRQ_HANDLED;
}

#define CHG_COMP_SFT_BIT	BIT(3)
static irqreturn_t pr_error_handler(int irq, void *_chip)
{
	struct oplus_chg_chip *chip = _chip;
	int rc = 0;
	u8 reg;

	pr_smb(PR_INTERRUPT, "chg-error triggered\n");

	rc = smbchg_read(chip, &reg, chip->pmic_spmi.chgr_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Unable to read RT_STS rc = %d\n", rc);
	} else {
		pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
		if (reg & CHG_COMP_SFT_BIT)
			set_property_on_fg(chip,
					POWER_SUPPLY_PROP_SAFETY_TIMER_EXPIRED,
					1);
	}

	smbchg_parallel_usb_check_ok(chip);
	if (chip->pmic_spmi.psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	smbchg_wipower_check(chip);
	return IRQ_HANDLED;
}

static irqreturn_t fastchg_handler(int irq, void *_chip)
{
	struct oplus_chg_chip *chip = _chip;

	pr_smb(PR_INTERRUPT, "p2f triggered\n");
	smbchg_detect_parallel_charger(chip);
	smbchg_parallel_usb_check_ok(chip);
	if (chip->pmic_spmi.psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	smbchg_wipower_check(chip);
	return IRQ_HANDLED;
}

static irqreturn_t chg_hot_handler(int irq, void *_chip)
{
	pr_warn_ratelimited("chg hot\n");
	smbchg_wipower_check(_chip);
	return IRQ_HANDLED;
}

static irqreturn_t chg_term_handler(int irq, void *_chip)
{
	struct oplus_chg_chip *chip = _chip;

	pr_smb(PR_INTERRUPT, "tcc triggered\n");
	/*
	 * Charge termination is a pulse and not level triggered. That means,
	 * TCC bit in RT_STS can get cleared by the time this interrupt is
	 * handled. Instead of relying on that to determine whether the
	 * charge termination had happened, we've to simply notify the FG
	 * about this as long as the interrupt is handled.
	 */
	set_property_on_fg(chip, POWER_SUPPLY_PROP_CHARGE_DONE, 1);

	smbchg_parallel_usb_check_ok(chip);
	if (chip->pmic_spmi.psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);

	return IRQ_HANDLED;
}

static irqreturn_t taper_handler(int irq, void *_chip)
{
	struct oplus_chg_chip *chip = _chip;
	u8 reg = 0;

	taper_irq_en(chip, false);
	smbchg_read(chip, &reg, chip->pmic_spmi.chgr_base + RT_STS, 1);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	smbchg_parallel_usb_taper(chip);
	if (chip->pmic_spmi.psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	smbchg_wipower_check(chip);
	return IRQ_HANDLED;
}

static irqreturn_t recharge_handler(int irq, void *_chip)
{
	struct oplus_chg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->pmic_spmi.chgr_base + RT_STS, 1);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	smbchg_parallel_usb_check_ok(chip);
	if (chip->pmic_spmi.psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	return IRQ_HANDLED;
}

static irqreturn_t wdog_timeout_handler(int irq, void *_chip)
{
	struct oplus_chg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->pmic_spmi.misc_base + RT_STS, 1);
	pr_warn_ratelimited("wdog timeout rt_stat = 0x%02x\n", reg);
	if (chip->pmic_spmi.psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	return IRQ_HANDLED;
}

/**
 * power_ok_handler() - called when the switcher turns on or turns off
 * @chip: pointer to oplus_chg_chip
 * @rt_stat: the status bit indicating switcher turning on or off
 */
static irqreturn_t power_ok_handler(int irq, void *_chip)
{
	struct oplus_chg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->pmic_spmi.misc_base + RT_STS, 1);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	return IRQ_HANDLED;
}

/**
 * dcin_uv_handler() - called when the dc voltage crosses the uv threshold
 * @chip: pointer to oplus_chg_chip
 * @rt_stat: the status bit indicating whether dc voltage is uv
 */
#define DCIN_UNSUSPEND_DELAY_MS		1000
static irqreturn_t dcin_uv_handler(int irq, void *_chip)
{
	struct oplus_chg_chip *chip = _chip;
	bool dc_present = is_dc_present(chip);

	pr_smb(PR_STATUS, "chip->pmic_spmi.dc_present = %d dc_present = %d\n",
			chip->pmic_spmi.dc_present, dc_present);

	if (chip->pmic_spmi.dc_present != dc_present) {
		/* dc changed */
		chip->pmic_spmi.dc_present = dc_present;
		if (chip->pmic_spmi.dc_psy_type != -EINVAL && chip->pmic_spmi.psy_registered)
			power_supply_changed(&chip->pmic_spmi.dc_psy);
		smbchg_charging_status_change(chip);
		smbchg_aicl_deglitch_wa_check(chip);
		chip->pmic_spmi.vbat_above_headroom = false;
	}

	smbchg_wipower_check(chip);
	return IRQ_HANDLED;
}

/**
 * usbin_ov_handler() - this is called when an overvoltage condition occurs
 * @chip: pointer to oplus_chg_chip chip
 */
static irqreturn_t usbin_ov_handler(int irq, void *_chip)
{
	struct oplus_chg_chip *chip = _chip;
	int rc;
	u8 reg;
	bool usb_present;

	rc = smbchg_read(chip, &reg, chip->pmic_spmi.usb_chgpth_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb rt status rc = %d\n", rc);
		goto out;
	}

	/* OV condition is detected. Notify it to USB psy */
	if (reg & USBIN_OV_BIT) {
		chip->pmic_spmi.usb_ov_det = true;
		if (chip->usb_psy) {
			pr_smb(PR_MISC, "setting usb psy health OV\n");
			rc = power_supply_set_health_state(chip->usb_psy,
					POWER_SUPPLY_HEALTH_OVERVOLTAGE);
			if (rc)
				pr_smb(PR_STATUS,
					"usb psy does not allow updating prop %d rc = %d\n",
					POWER_SUPPLY_HEALTH_OVERVOLTAGE, rc);
		}
	} else {
		chip->pmic_spmi.usb_ov_det = false;
		/* If USB is present, then handle the USB insertion */
		usb_present = is_usb_present(chip);
		if (usb_present)
			update_usb_status(chip, usb_present, false);
	}
out:
	return IRQ_HANDLED;
}

/**
 * usbin_uv_handler() - this is called when USB charger is removed
 * @chip: pointer to oplus_chg_chip chip
 * @rt_stat: the status bit indicating chg insertion/removal
 */
#define ICL_MODE_MASK		SMB_MASK(5, 4)
#define ICL_MODE_HIGH_CURRENT	0
static irqreturn_t usbin_uv_handler(int irq, void *_chip)
{
	struct oplus_chg_chip *chip = _chip;
	int aicl_level = smbchg_get_aicl_level_ma(chip);
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->pmic_spmi.usb_chgpth_base + RT_STS, 1);
	if (rc) {
		pr_err("could not read rt sts: %d", rc);
		goto out;
	}

	pr_smb(PR_STATUS,
		"%s chip->pmic_spmi.usb_present = %d rt_sts = 0x%02x hvdcp_3_det_ignore_uv = %d aicl = %d\n",
		chip->pmic_spmi.hvdcp_3_det_ignore_uv ? "Ignoring":"",
		chip->pmic_spmi.usb_present, reg, chip->pmic_spmi.hvdcp_3_det_ignore_uv,
		aicl_level);
#ifdef OPLUS_CMCC_TEST
	return IRQ_HANDLED;
#endif
	/*
	 * set usb_psy's dp=f dm=f if this is a new insertion, i.e. it is
	 * not already src_detected and usbin_uv is seen falling
	 */
	if (!(reg & USBIN_UV_BIT) && !(reg & USBIN_SRC_DET_BIT)) {
		pr_smb(PR_MISC, "setting usb psy dp=f dm=f\n");
		power_supply_set_dp_dm(chip->usb_psy,
				POWER_SUPPLY_DP_DM_DPF_DMF);
	}

	if (reg & USBIN_UV_BIT)
		complete_all(&chip->pmic_spmi.usbin_uv_raised);
	else
		complete_all(&chip->pmic_spmi.usbin_uv_lowered);

	if (chip->pmic_spmi.hvdcp_3_det_ignore_uv)
		goto out;

	if ((reg & USBIN_UV_BIT) && (reg & USBIN_SRC_DET_BIT)) {
		pr_smb(PR_STATUS, "Very weak charger detected\n");
		chip->pmic_spmi.very_weak_charger = true;
		rc = smbchg_read(chip, &reg,
				chip->pmic_spmi.usb_chgpth_base + ICL_STS_2_REG, 1);
		if (rc) {
			dev_err(chip->dev, "Could not read usb icl sts 2: %d\n",
					rc);
			goto out;
		}
		if ((reg & ICL_MODE_MASK) != ICL_MODE_HIGH_CURRENT) {
			/*
			 * If AICL is not even enabled, this is either an
			 * SDP or a grossly out of spec charger. Do not
			 * draw any current from it.
			 */
			rc = vote(chip->pmic_spmi.usb_suspend_votable,
					WEAK_CHARGER_EN_VOTER, true, 0);
			if (rc < 0)
				pr_err("could not disable charger: %d", rc);
		} else if (aicl_level == chip->pmic_spmi.tables.usb_ilim_ma_table[0]) {
			/*
			 * we are in a situation where the adapter is not able
			 * to supply even 300mA. Disable hw aicl reruns else it
			 * is only a matter of time when we get back here again
			 */
			rc = vote(chip->pmic_spmi.hw_aicl_rerun_disable_votable,
				WEAK_CHARGER_HW_AICL_VOTER, true, 0);
			if (rc < 0)
				pr_err("Couldn't disable hw aicl rerun rc=%d\n",
						rc);
		}
		pr_smb(PR_MISC, "setting usb psy health UNSPEC_FAILURE\n");
		rc = power_supply_set_health_state(chip->usb_psy,
				POWER_SUPPLY_HEALTH_UNSPEC_FAILURE);
		if (rc)
			pr_err("Couldn't set health on usb psy rc:%d\n", rc);
		schedule_work(&chip->pmic_spmi.usb_set_online_work);
	}

	smbchg_wipower_check(chip);
out:
	return IRQ_HANDLED;
}

/**
 * src_detect_handler() - this is called on rising edge when USB charger type
 *			is detected and on falling edge when USB voltage falls
 *			below the coarse detect voltage(1V), use it for
 *			handling USB charger insertion and removal.
 * @chip: pointer to oplus_chg_chip
 * @rt_stat: the status bit indicating chg insertion/removal
 */
static irqreturn_t src_detect_handler(int irq, void *_chip)
{
	struct oplus_chg_chip *chip = _chip;
	bool usb_present = is_usb_present(chip);
	bool src_detect = is_src_detect_high(chip);
	int rc;

	if(chip->pmic_spmi.hvdcp_3_det_ignore_uv == false) {
		if(chip->pmic_spmi.usb_present && !usb_present) {
			oplus_vooc_reset_fastchg_after_usbout();
			if(oplus_vooc_get_fastchg_started() == false) {
				smbchg_set_chargerid_switch_val(chip, 0);
				chip->chargerid_volt = 0;
				chip->chargerid_volt_got = false;
			}
		}
	}
	
	chg_err("%s chip->pmic_spmi.usb_present = %d usb_present = %d src_detect = %d hvdcp_3_det_ignore_uv=%d\n",
		chip->pmic_spmi.hvdcp_3_det_ignore_uv ? "Ignoring":"",
		chip->pmic_spmi.usb_present, usb_present, src_detect,
		chip->pmic_spmi.hvdcp_3_det_ignore_uv);

	if (src_detect)
		complete_all(&chip->pmic_spmi.src_det_raised);
	else
		complete_all(&chip->pmic_spmi.src_det_lowered);

	if (chip->pmic_spmi.hvdcp_3_det_ignore_uv)
		goto out;

	/*
	 * When VBAT is above the AICL threshold (4.25V) - 180mV (4.07V),
	 * an input collapse due to AICL will actually cause an USBIN_UV
	 * interrupt to fire as well.
	 *
	 * Handle USB insertions and removals in the source detect handler
	 * instead of the USBIN_UV handler since the latter is untrustworthy
	 * when the battery voltage is high.
	 */
	chip->pmic_spmi.very_weak_charger = false;
	/*
	 * a src detect marks a new insertion or a real removal,
	 * vote for enable aicl hw reruns
	 */
	rc = vote(chip->pmic_spmi.hw_aicl_rerun_disable_votable,
		WEAK_CHARGER_HW_AICL_VOTER, false, 0);
	if (rc < 0)
		pr_err("Couldn't enable hw aicl rerun rc=%d\n", rc);

	rc = vote(chip->pmic_spmi.usb_suspend_votable, WEAK_CHARGER_EN_VOTER, false, 0);
	if (rc < 0)
		pr_err("could not enable charger: %d\n", rc);

	if (src_detect) {
		update_usb_status(chip, usb_present, 0);
	} else {
		update_usb_status(chip, 0, false);
		chip->pmic_spmi.aicl_irq_count = 0;
	}
out:
	return IRQ_HANDLED;
}

/**
 * otg_oc_handler() - called when the usb otg goes over current
 */
#define NUM_OTG_RETRIES			5
#define OTG_OC_RETRY_DELAY_US		50000
static irqreturn_t otg_oc_handler(int irq, void *_chip)
{
	int rc;
	struct oplus_chg_chip *chip = _chip;
	s64 elapsed_us = ktime_us_delta(ktime_get(), chip->pmic_spmi.otg_enable_time);

	pr_smb(PR_INTERRUPT, "triggered\n");

	if (chip->pmic_spmi.schg_version == QPNP_SCHG_LITE) {
		pr_warn("OTG OC triggered - OTG disabled\n");
		return IRQ_HANDLED;
	}

	if (elapsed_us > OTG_OC_RETRY_DELAY_US)
		chip->pmic_spmi.otg_retries = 0;

	/*
	 * Due to a HW bug in the PMI8994 charger, the current inrush that
	 * occurs when connecting certain OTG devices can cause the OTG
	 * overcurrent protection to trip.
	 *
	 * The work around is to try reenabling the OTG when getting an
	 * overcurrent interrupt once.
	 */
	if (chip->pmic_spmi.otg_retries < NUM_OTG_RETRIES) {
		chip->pmic_spmi.otg_retries += 1;
		pr_smb(PR_STATUS,
			"Retrying OTG enable. Try #%d, elapsed_us %lld\n",
						chip->pmic_spmi.otg_retries, elapsed_us);
		rc = otg_oc_reset(chip);
		if (rc)
			pr_err("Failed to reset OTG OC state rc=%d\n", rc);
		chip->pmic_spmi.otg_enable_time = ktime_get();
	}
	return IRQ_HANDLED;
}

/**
 * otg_fail_handler() - called when the usb otg fails
 * (when vbat < OTG UVLO threshold)
 */
static irqreturn_t otg_fail_handler(int irq, void *_chip)
{
	pr_smb(PR_INTERRUPT, "triggered\n");
	return IRQ_HANDLED;
}

/**
 * aicl_done_handler() - called when the usb AICL algorithm is finished
 *			and a current is set.
 */
static irqreturn_t aicl_done_handler(int irq, void *_chip)
{
	struct oplus_chg_chip *chip = _chip;
	bool usb_present = is_usb_present(chip);
	int aicl_level = smbchg_get_aicl_level_ma(chip);

	pr_smb(PR_INTERRUPT, "triggered, aicl: %d\n", aicl_level);

	increment_aicl_count(chip);

	if (usb_present)
		smbchg_parallel_usb_check_ok(chip);

	if (chip->pmic_spmi.aicl_complete)
		power_supply_changed(&chip->batt_psy);

	return IRQ_HANDLED;
}

/**
 * usbid_change_handler() - called when the usb RID changes.
 * This is used mostly for detecting OTG
 */
static irqreturn_t usbid_change_handler(int irq, void *_chip)
{
	struct oplus_chg_chip *chip = _chip;
	bool otg_present;

	otg_present = is_otg_present(chip);
	chg_err("triggered, otg_present:%d\n", otg_present);
	if (chip->usb_psy) {
		pr_smb(PR_MISC, "setting usb psy OTG = %d\n",
				otg_present ? 1 : 0);
		power_supply_set_usb_otg(chip->usb_psy, otg_present ? 1 : 0);
	}
	if (otg_present)
		pr_smb(PR_STATUS, "OTG detected\n");

	/* update FG */
	set_property_on_fg(chip, POWER_SUPPLY_PROP_STATUS,
			get_prop_batt_status(chip));

	return IRQ_HANDLED;
}

static int determine_initial_status(struct oplus_chg_chip *chip)
{
	union power_supply_propval type = {0, };
	int boot_mode = 0;

	/*
	 * It is okay to read the interrupt status here since
	 * interrupts aren't requested. reading interrupt status
	 * clears the interrupt so be careful to read interrupt
	 * status only in interrupt handling code
	 */

	batt_pres_handler(0, chip);
	batt_hot_handler(0, chip);
	batt_warm_handler(0, chip);
	batt_cool_handler(0, chip);
	batt_cold_handler(0, chip);
	if (chip->pmic_spmi.typec_psy) {
		get_property_from_typec(chip, POWER_SUPPLY_PROP_TYPE, &type);
		update_typec_otg_status(chip, type.intval, true);
	} else {
		usbid_change_handler(0, chip);
	}
	src_detect_handler(0, chip);
	boot_mode = get_boot_mode();
	if((boot_mode == MSM_BOOT_MODE__RF) || (boot_mode == MSM_BOOT_MODE__WLAN)){
		smbchg_usb_suspend(chip, true);
	} else {
		smbchg_usb_suspend(chip, false);
	}
	chip->pmic_spmi.usb_present = is_usb_present(chip);
	chip->pmic_spmi.dc_present = is_dc_present(chip);

	if (chip->pmic_spmi.usb_present) {
		pr_smb(PR_MISC, "setting usb psy dp=f dm=f\n");
		power_supply_set_dp_dm(chip->usb_psy,
				POWER_SUPPLY_DP_DM_DPF_DMF);
		handle_usb_insertion(chip);
	} else {
		handle_usb_removal(chip);
	}

	return 0;
}

static int prechg_time[] = {
	24,
	48,
	96,
	192,
};
static int chg_time[] = {
	192,
	384,
	768,
	1536,
};

enum bpd_type {
	BPD_TYPE_BAT_NONE,
	BPD_TYPE_BAT_ID,
	BPD_TYPE_BAT_THM,
	BPD_TYPE_BAT_THM_BAT_ID,
	BPD_TYPE_DEFAULT,
};

static const char * const bpd_label[] = {
	[BPD_TYPE_BAT_NONE]		= "bpd_none",
	[BPD_TYPE_BAT_ID]		= "bpd_id",
	[BPD_TYPE_BAT_THM]		= "bpd_thm",
	[BPD_TYPE_BAT_THM_BAT_ID]	= "bpd_thm_id",
};

static inline int get_bpd(const char *name)
{
	int i = 0;
	for (i = 0; i < ARRAY_SIZE(bpd_label); i++) {
		if (strcmp(bpd_label[i], name) == 0)
			return i;
	}
	return -EINVAL;
}

#define REVISION1_REG			0x0
#define DIG_MINOR			0
#define DIG_MAJOR			1
#define ANA_MINOR			2
#define ANA_MAJOR			3
#define CHGR_CFG1			0xFB
#define RECHG_THRESHOLD_SRC_BIT		BIT(1)
#define TERM_I_SRC_BIT			BIT(2)
#define TERM_SRC_FG			BIT(2)
#define CHG_INHIB_CFG_REG		0xF7
#define CHG_INHIBIT_50MV_VAL		0x00
#define CHG_INHIBIT_100MV_VAL		0x01
#define CHG_INHIBIT_200MV_VAL		0x02
#define CHG_INHIBIT_300MV_VAL		0x03
#define CHG_INHIBIT_MASK		0x03
#define USE_REGISTER_FOR_CURRENT	BIT(2)
#define CHGR_CFG2			0xFC
#define CHG_EN_SRC_BIT			BIT(7)
#define CHG_EN_POLARITY_BIT		BIT(6)
#define P2F_CHG_TRAN			BIT(5)
#define CHG_BAT_OV_ECC			BIT(4)
#define I_TERM_BIT			BIT(3)
#define AUTO_RECHG_BIT			BIT(2)
#define CHARGER_INHIBIT_BIT		BIT(0)
#define USB51_COMMAND_POL		BIT(2)
#define USB51AC_CTRL			BIT(1)
#define TR_8OR32B			0xFE
#define BUCK_8_16_FREQ_BIT		BIT(0)
#define BM_CFG				0xF3
#define BATT_MISSING_ALGO_BIT		BIT(2)
#define BMD_PIN_SRC_MASK		SMB_MASK(1, 0)
#define PIN_SRC_SHIFT			0
#define CHGR_CFG			0xFF
#define RCHG_LVL_BIT			BIT(0)
#define VCHG_EN_BIT			BIT(1)
#define VCHG_INPUT_CURRENT_BIT		BIT(3)
#define CFG_AFVC			0xF6
#define VFLOAT_COMP_ENABLE_MASK		SMB_MASK(2, 0)
#define TR_RID_REG			0xFA
#define FG_INPUT_FET_DELAY_BIT		BIT(3)
#define TRIM_OPTIONS_7_0		0xF6
#define INPUT_MISSING_POLLER_EN_BIT	BIT(3)
#define CHGR_CCMP_CFG			0xFA
#define JEITA_TEMP_HARD_LIMIT_BIT	BIT(5)
#define HVDCP_ADAPTER_SEL_MASK		SMB_MASK(5, 4)
#define HVDCP_ADAPTER_SEL_9V_BIT	BIT(4)
#define HVDCP_AUTH_ALG_EN_BIT		BIT(6)
#define CMD_APSD			0x41
#define APSD_RERUN_BIT			BIT(0)
#define OTG_CFG				0xF1
#define HICCUP_ENABLED_BIT		BIT(6)
#define OTG_PIN_POLARITY_BIT		BIT(4)
#define OTG_PIN_ACTIVE_LOW		BIT(4)
#define OTG_EN_CTRL_MASK		SMB_MASK(3, 2)
#define OTG_PIN_CTRL_RID_DIS		0x04
#define OTG_CMD_CTRL_RID_EN		0x08
#define AICL_ADC_BIT			BIT(6)
#ifdef OPLUS_FEATURE_CHG_BASIC
// Add for otg id value
#define OTG_CMD_CTRL_RID_DIS	0x00
#endif /*OPLUS_FEATURE_CHG_BASIC*/
static void batt_ov_wa_check(struct oplus_chg_chip *chip)
{
	int rc;
	u8 reg;

	/* disable-'battery OV disables charging' feature */
	rc = smbchg_sec_masked_write(chip, chip->pmic_spmi.chgr_base + CHGR_CFG2,
			CHG_BAT_OV_ECC, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set chgr_cfg2 rc=%d\n", rc);
		return;
	}

	/*
	 * if battery OV is set:
	 * restart charging by disable/enable charging
	 */
	rc = smbchg_read(chip, &reg, chip->pmic_spmi.bat_if_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't read Battery RT status rc = %d\n", rc);
		return;
	}

	if (reg & BAT_OV_BIT) {
		rc = smbchg_charging_en(chip, false);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't disable charging: rc = %d\n", rc);
			return;
		}

		/* delay for charging-disable to take affect */
		msleep(200);

		rc = smbchg_charging_en(chip, true);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't enable charging: rc = %d\n", rc);
			return;
		}
	}
}

static int smbchg_hw_init(struct oplus_chg_chip *chip)
{
	int rc, i;
	u8 reg, mask;

	rc = smbchg_read(chip, chip->pmic_spmi.revision,
			chip->pmic_spmi.misc_base + REVISION1_REG, 4);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read revision rc=%d\n",
				rc);
		return rc;
	}
	pr_smb(PR_STATUS, "Charger Revision DIG: %d.%d; ANA: %d.%d\n",
			chip->pmic_spmi.revision[DIG_MAJOR], chip->pmic_spmi.revision[DIG_MINOR],
			chip->pmic_spmi.revision[ANA_MAJOR], chip->pmic_spmi.revision[ANA_MINOR]);

	/* Setup 9V HVDCP */
	if (!chip->pmic_spmi.hvdcp_not_supported) {
		rc = smbchg_sec_masked_write(chip,
				chip->pmic_spmi.usb_chgpth_base + CHGPTH_CFG,
				HVDCP_ADAPTER_SEL_MASK, HVDCP_9V);
		if (rc < 0) {
			pr_err("Couldn't set hvdcp config in chgpath_chg rc=%d\n",
					rc);
			return rc;
		}
	} else {
		/* disable HVDCP */
		pr_smb(PR_MISC, "Disable HVDCP\n");
		rc = smbchg_sec_masked_write(chip, chip->pmic_spmi.usb_chgpth_base + CHGPTH_CFG,
				HVDCP_EN_BIT, 0);
		if (rc < 0)
			pr_err("Couldn't disable HVDCP rc=%d\n", rc);
	}

	if (chip->pmic_spmi.aicl_rerun_period_s > 0) {
		rc = smbchg_set_aicl_rerun_period_s(chip,
				chip->pmic_spmi.aicl_rerun_period_s);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set AICL rerun timer rc=%d\n",
					rc);
			return rc;
		}
	}

	rc = smbchg_sec_masked_write(chip, chip->pmic_spmi.usb_chgpth_base + TR_RID_REG,
			FG_INPUT_FET_DELAY_BIT, FG_INPUT_FET_DELAY_BIT);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't disable fg input fet delay rc=%d\n",
				rc);
		return rc;
	}

	rc = smbchg_sec_masked_write(chip, chip->pmic_spmi.misc_base + TRIM_OPTIONS_7_0,
			INPUT_MISSING_POLLER_EN_BIT,
			INPUT_MISSING_POLLER_EN_BIT);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't enable input missing poller rc=%d\n",
				rc);
		return rc;
	}

	/*
	 * Do not force using current from the register i.e. use auto
	 * power source detect (APSD) mA ratings for the initial current values.
	 *
	 * If this is set, AICL will not rerun at 9V for HVDCPs
	 */
	rc = smbchg_masked_write(chip, chip->pmic_spmi.usb_chgpth_base + CMD_IL,
			USE_REGISTER_FOR_CURRENT, 0);

	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set input limit cmd rc=%d\n", rc);
		return rc;
	}

	/*
	 * set chg en by cmd register, set chg en by writing bit 1,
	 * enable auto pre to fast, enable auto recharge by default.
	 * enable current termination and charge inhibition based on
	 * the device tree configuration.
	 */
	rc = smbchg_sec_masked_write(chip, chip->pmic_spmi.chgr_base + CHGR_CFG2,
			CHG_EN_SRC_BIT | CHG_EN_POLARITY_BIT | P2F_CHG_TRAN
			| I_TERM_BIT | AUTO_RECHG_BIT | CHARGER_INHIBIT_BIT,
			CHG_EN_POLARITY_BIT
			| (chip->pmic_spmi.chg_inhibit_en ? CHARGER_INHIBIT_BIT : 0)
			| (chip->pmic_spmi.iterm_disabled ? I_TERM_BIT : 0));
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set chgr_cfg2 rc=%d\n", rc);
		return rc;
	}

	/*
	 * enable battery charging to make sure it hasn't been changed earlier
	 * by the bootloader.
	 */
	rc = smbchg_charging_en(chip, true);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't enable battery charging=%d\n", rc);
		return rc;
	}

	/*
	 * Based on the configuration, use the analog sensors or the fuelgauge
	 * adc for recharge threshold source.
	 */

	if (chip->pmic_spmi.chg_inhibit_source_fg)
		rc = smbchg_sec_masked_write(chip, chip->pmic_spmi.chgr_base + CHGR_CFG1,
			TERM_I_SRC_BIT | RECHG_THRESHOLD_SRC_BIT,
			TERM_SRC_FG | RECHG_THRESHOLD_SRC_BIT);
	else
		rc = smbchg_sec_masked_write(chip, chip->pmic_spmi.chgr_base + CHGR_CFG1,
			TERM_I_SRC_BIT | RECHG_THRESHOLD_SRC_BIT, 0);

	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set chgr_cfg2 rc=%d\n", rc);
		return rc;
	}

	/*
	 * control USB suspend via command bits and set correct 100/500mA
	 * polarity on the usb current
	 */
	rc = smbchg_sec_masked_write(chip, chip->pmic_spmi.usb_chgpth_base + CHGPTH_CFG,
		USB51_COMMAND_POL | USB51AC_CTRL, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set usb_chgpth cfg rc=%d\n", rc);
		return rc;
	}

	check_battery_type(chip);

	/* set the float voltage */
	if (chip->pmic_spmi.vfloat_mv != -EINVAL) {
		rc = smbchg_float_voltage_set(chip, chip->pmic_spmi.vfloat_mv);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set float voltage rc = %d\n", rc);
			return rc;
		}
		pr_smb(PR_STATUS, "set vfloat to %d\n", chip->pmic_spmi.vfloat_mv);
	}

	/* set the fast charge current compensation */
	if (chip->pmic_spmi.fastchg_current_comp != -EINVAL) {
		rc = smbchg_fastchg_current_comp_set(chip,
			chip->pmic_spmi.fastchg_current_comp);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set fastchg current comp rc = %d\n",
				rc);
			return rc;
		}
		pr_smb(PR_STATUS, "set fastchg current comp to %d\n",
			chip->pmic_spmi.fastchg_current_comp);
	}

	/* set the float voltage compensation */
	if (chip->pmic_spmi.float_voltage_comp != -EINVAL) {
		rc = smbchg_float_voltage_comp_set(chip,
			chip->pmic_spmi.float_voltage_comp);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set float voltage comp rc = %d\n",
				rc);
			return rc;
		}
		pr_smb(PR_STATUS, "set float voltage comp to %d\n",
			chip->pmic_spmi.float_voltage_comp);
	}

	/* set iterm */
	if (chip->pmic_spmi.iterm_ma != -EINVAL) {
		if (chip->pmic_spmi.iterm_disabled) {
			dev_err(chip->dev, "Error: Both iterm_disabled and iterm_ma set\n");
			return -EINVAL;
		} else {
			smbchg_iterm_set(chip, chip->pmic_spmi.iterm_ma);
		}
	}

	/* set the safety time voltage */
	if (chip->pmic_spmi.safety_time != -EINVAL) {
		reg = (chip->pmic_spmi.safety_time > 0 ? 0 : SFT_TIMER_DISABLE_BIT) |
			(chip->pmic_spmi.prechg_safety_time > 0
			? 0 : PRECHG_SFT_TIMER_DISABLE_BIT);

		for (i = 0; i < ARRAY_SIZE(chg_time); i++) {
			if (chip->pmic_spmi.safety_time <= chg_time[i]) {
				reg |= i << SAFETY_TIME_MINUTES_SHIFT;
				break;
			}
		}
		for (i = 0; i < ARRAY_SIZE(prechg_time); i++) {
			if (chip->pmic_spmi.prechg_safety_time <= prechg_time[i]) {
				reg |= i;
				break;
			}
		}

		rc = smbchg_sec_masked_write(chip,
				chip->pmic_spmi.chgr_base + SFT_CFG,
				SFT_EN_MASK | SFT_TO_MASK |
				(chip->pmic_spmi.prechg_safety_time > 0
				? PRECHG_SFT_TO_MASK : 0), reg);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set safety timer rc = %d\n",
				rc);
			return rc;
		}
		chip->pmic_spmi.safety_timer_en = true;
	} else {
		rc = smbchg_read(chip, &reg, chip->pmic_spmi.chgr_base + SFT_CFG, 1);
		if (rc < 0)
			dev_err(chip->dev, "Unable to read SFT_CFG rc = %d\n",
				rc);
		else if (!(reg & SFT_EN_MASK))
			chip->pmic_spmi.safety_timer_en = true;
	}

	/* configure jeita temperature hard limit */
	if (chip->pmic_spmi.jeita_temp_hard_limit >= 0) {
		rc = smbchg_sec_masked_write(chip,
			chip->pmic_spmi.chgr_base + CHGR_CCMP_CFG,
			JEITA_TEMP_HARD_LIMIT_BIT,
			chip->pmic_spmi.jeita_temp_hard_limit
			? 0 : JEITA_TEMP_HARD_LIMIT_BIT);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set jeita temp hard limit rc = %d\n",
				rc);
			return rc;
		}
	}

	/* make the buck switch faster to prevent some vbus oscillation */
	rc = smbchg_sec_masked_write(chip,
			chip->pmic_spmi.usb_chgpth_base + TR_8OR32B,
			BUCK_8_16_FREQ_BIT, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set buck frequency rc = %d\n", rc);
		return rc;
	}

	/* battery missing detection */
	mask =  BATT_MISSING_ALGO_BIT;
	reg = chip->pmic_spmi.bmd_algo_disabled ? 0 : BATT_MISSING_ALGO_BIT;
	if (chip->pmic_spmi.bmd_pin_src < BPD_TYPE_DEFAULT) {
		mask |= BMD_PIN_SRC_MASK;
		reg |= chip->pmic_spmi.bmd_pin_src << PIN_SRC_SHIFT;
	}
	rc = smbchg_sec_masked_write(chip,
			chip->pmic_spmi.bat_if_base + BM_CFG, mask, reg);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set batt_missing config = %d\n",
									rc);
		return rc;
	}

	if (chip->pmic_spmi.vchg_adc_channel != -EINVAL) {
		/* configure and enable VCHG */
		rc = smbchg_sec_masked_write(chip, chip->pmic_spmi.chgr_base + CHGR_CFG,
				VCHG_INPUT_CURRENT_BIT | VCHG_EN_BIT,
				VCHG_INPUT_CURRENT_BIT | VCHG_EN_BIT);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set recharge rc = %d\n",
					rc);
			return rc;
		}
	}

	smbchg_charging_status_change(chip);

	vote(chip->pmic_spmi.usb_suspend_votable, USER_EN_VOTER, !chip->pmic_spmi.chg_enabled, 0);
	vote(chip->pmic_spmi.dc_suspend_votable, USER_EN_VOTER, !chip->pmic_spmi.chg_enabled, 0);
	/* resume threshold */
	if (chip->pmic_spmi.resume_delta_mv != -EINVAL) {

		/*
		 * Configure only if the recharge threshold source is not
		 * fuel gauge ADC.
		 */
		if (!chip->pmic_spmi.chg_inhibit_source_fg) {
			if (chip->pmic_spmi.resume_delta_mv < 100)
				reg = CHG_INHIBIT_50MV_VAL;
			else if (chip->pmic_spmi.resume_delta_mv < 200)
				reg = CHG_INHIBIT_100MV_VAL;
			else if (chip->pmic_spmi.resume_delta_mv < 300)
				reg = CHG_INHIBIT_200MV_VAL;
			else
				reg = CHG_INHIBIT_300MV_VAL;

			rc = smbchg_sec_masked_write(chip,
					chip->pmic_spmi.chgr_base + CHG_INHIB_CFG_REG,
					CHG_INHIBIT_MASK, reg);
			if (rc < 0) {
				dev_err(chip->dev, "Couldn't set inhibit val rc = %d\n",
						rc);
				return rc;
			}
		}

		rc = smbchg_sec_masked_write(chip,
				chip->pmic_spmi.chgr_base + CHGR_CFG,
				RCHG_LVL_BIT,
				(chip->pmic_spmi.resume_delta_mv
				 < chip->pmic_spmi.tables.rchg_thr_mv)
				? 0 : RCHG_LVL_BIT);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set recharge rc = %d\n",
					rc);
			return rc;
		}
	}

	/* DC path current settings */
	if (chip->pmic_spmi.dc_psy_type != -EINVAL) {
		rc = vote(chip->pmic_spmi.dc_icl_votable, PSY_ICL_VOTER, true,
					chip->pmic_spmi.dc_target_current_ma);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't vote for initial DC ICL rc=%d\n", rc);
			return rc;
		}
	}


	/*
	 * on some devices the battery is powered via external sources which
	 * could raise its voltage above the float voltage. smbchargers go
	 * in to reverse boost in such a situation and the workaround is to
	 * disable float voltage compensation (note that the battery will appear
	 * hot/cold when powered via external source).
	 */
	if (chip->pmic_spmi.soft_vfloat_comp_disabled) {
		rc = smbchg_sec_masked_write(chip, chip->pmic_spmi.chgr_base + CFG_AFVC,
				VFLOAT_COMP_ENABLE_MASK, 0);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't disable soft vfloat rc = %d\n",
					rc);
			return rc;
		}
	}

	rc = vote(chip->pmic_spmi.fcc_votable, BATT_TYPE_FCC_VOTER, true,
			chip->pmic_spmi.cfg_fastchg_current_ma);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't vote fastchg ma rc = %d\n", rc);
		return rc;
	}

	rc = smbchg_read(chip, &chip->pmic_spmi.original_usbin_allowance,
			chip->pmic_spmi.usb_chgpth_base + USBIN_CHGR_CFG, 1);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't read usb allowance rc=%d\n", rc);

	if (chip->pmic_spmi.wipower_dyn_icl_avail) {
		rc = smbchg_wipower_ilim_config(chip,
				&(chip->pmic_spmi.wipower_default.entries[0]));
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set default wipower ilim = %d\n",
				rc);
			return rc;
		}
	}
	/* unsuspend dc path, it could be suspended by the bootloader */
	rc = smbchg_dc_suspend(chip, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't unsuspend dc path= %d\n", rc);
		return rc;
	}

	if (chip->pmic_spmi.force_aicl_rerun) {
		/* vote to enable hw aicl */
		rc = vote(chip->pmic_spmi.hw_aicl_rerun_enable_indirect_votable,
			DEFAULT_CONFIG_HW_AICL_VOTER, true, 0);
		if (rc < 0) {
			pr_err("Couldn't vote enable hw aicl rerun rc=%d\n",
				rc);
			return rc;
		}
	}

	if (chip->pmic_spmi.schg_version == QPNP_SCHG_LITE) {
		/* enable OTG hiccup mode */
		rc = smbchg_sec_masked_write(chip, chip->pmic_spmi.otg_base + OTG_CFG,
					HICCUP_ENABLED_BIT, HICCUP_ENABLED_BIT);
		if (rc < 0)
			dev_err(chip->dev, "Couldn't set OTG OC config rc = %d\n",
				rc);
	}

	if (chip->pmic_spmi.otg_pinctrl) {
		/* configure OTG enable to pin control active low */
		rc = smbchg_sec_masked_write(chip, chip->pmic_spmi.otg_base + OTG_CFG,
				OTG_PIN_POLARITY_BIT | OTG_EN_CTRL_MASK,
				OTG_PIN_ACTIVE_LOW | OTG_PIN_CTRL_RID_DIS);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set OTG EN config rc = %d\n",
				rc);
			return rc;
		}
	}
#ifndef OPLUS_FEATURE_CHG_BASIC
// Add for disable otg RID EN
		/* configure OTG enable to register command control*/
	rc = smbchg_sec_masked_write(chip, chip->pmic_spmi.otg_base + OTG_CFG,
			OTG_EN_CTRL_MASK,OTG_CMD_CTRL_RID_EN);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set OTG CMD CTRL config rc = %d\n",rc);
		return rc;
	}
#else
	rc = smbchg_sec_masked_write(chip, chip->pmic_spmi.otg_base + OTG_CFG,
			OTG_EN_CTRL_MASK,OTG_CMD_CTRL_RID_DIS);
	chg_err("[id_value] Disable OTG_CFG RID_DIS\n");
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set OTG CMD CTRL config rc = %d\n",rc);
		return rc;
	}
	chip->pmic_spmi.id_value_en = false;
#endif /*OPLUS_FEATURE_CHG_BASIC*/

	if (chip->pmic_spmi.wa_flags & SMBCHG_BATT_OV_WA)
		batt_ov_wa_check(chip);

	/* turn off AICL adc for improved accuracy */
	rc = smbchg_sec_masked_write(chip,
		chip->pmic_spmi.misc_base + MISC_TRIM_OPT_15_8, AICL_ADC_BIT, 0);
	if (rc)
		pr_err("Couldn't write to MISC_TRIM_OPTIONS_15_8 rc=%d\n",
			rc);

	return rc;
}

static struct of_device_id smbchg_match_table[] = {
	{
		.compatible     = "qcom,qpnp-smbcharger",
	},
	{ },
};

#define DC_MA_MIN 300
#define DC_MA_MAX 2000
#define OF_PROP_READ(chip, prop, dt_property, retval, optional)		\
do {									\
	if (retval)							\
		break;							\
	if (optional)							\
		prop = -EINVAL;						\
									\
	retval = of_property_read_u32(chip->pmic_spmi.spmi->dev.of_node,		\
					"qcom," dt_property	,	\
					&prop);				\
									\
	if ((retval == -EINVAL) && optional)				\
		retval = 0;						\
	else if (retval)						\
		dev_err(chip->dev, "Error reading " #dt_property	\
				" property rc = %d\n", rc);		\
} while (0)

#define ILIM_ENTRIES		3
#define VOLTAGE_RANGE_ENTRIES	2
#define RANGE_ENTRY		(ILIM_ENTRIES + VOLTAGE_RANGE_ENTRIES)
static int smb_parse_wipower_map_dt(struct oplus_chg_chip *chip,
		struct ilim_map *map, char *property)
{
	struct device_node *node = chip->dev->of_node;
	int total_elements, size;
	struct property *prop;
	const __be32 *data;
	int num, i;

	prop = of_find_property(node, property, &size);
	if (!prop) {
		dev_err(chip->dev, "%s missing\n", property);
		return -EINVAL;
	}

	total_elements = size / sizeof(int);
	if (total_elements % RANGE_ENTRY) {
		dev_err(chip->dev, "%s table not in multiple of %d, total elements = %d\n",
				property, RANGE_ENTRY, total_elements);
		return -EINVAL;
	}

	data = prop->value;
	num = total_elements / RANGE_ENTRY;
	map->entries = devm_kzalloc(chip->dev,
			num * sizeof(struct ilim_entry), GFP_KERNEL);
	if (!map->entries) {
		dev_err(chip->dev, "kzalloc failed for default ilim\n");
		return -ENOMEM;
	}
	for (i = 0; i < num; i++) {
		map->entries[i].vmin_uv =  be32_to_cpup(data++);
		map->entries[i].vmax_uv =  be32_to_cpup(data++);
		map->entries[i].icl_pt_ma =  be32_to_cpup(data++);
		map->entries[i].icl_lv_ma =  be32_to_cpup(data++);
		map->entries[i].icl_hv_ma =  be32_to_cpup(data++);
	}
	map->num = num;
	return 0;
}

static int smb_parse_wipower_dt(struct oplus_chg_chip *chip)
{
	int rc = 0;

	chip->pmic_spmi.wipower_dyn_icl_avail = false;

	if (!chip->pmic_spmi.vadc_dev)
		goto err;

	rc = smb_parse_wipower_map_dt(chip, &chip->pmic_spmi.wipower_default,
					"qcom,wipower-default-ilim-map");
	if (rc) {
		dev_err(chip->dev, "failed to parse wipower-pt-ilim-map rc = %d\n",
				rc);
		goto err;
	}

	rc = smb_parse_wipower_map_dt(chip, &chip->pmic_spmi.wipower_pt,
					"qcom,wipower-pt-ilim-map");
	if (rc) {
		dev_err(chip->dev, "failed to parse wipower-pt-ilim-map rc = %d\n",
				rc);
		goto err;
	}

	rc = smb_parse_wipower_map_dt(chip, &chip->pmic_spmi.wipower_div2,
					"qcom,wipower-div2-ilim-map");
	if (rc) {
		dev_err(chip->dev, "failed to parse wipower-div2-ilim-map rc = %d\n",
				rc);
		goto err;
	}
	chip->pmic_spmi.wipower_dyn_icl_avail = true;
	return 0;
err:
	chip->pmic_spmi.wipower_default.num = 0;
	chip->pmic_spmi.wipower_pt.num = 0;
	chip->pmic_spmi.wipower_default.num = 0;
	if (chip->pmic_spmi.wipower_default.entries)
		devm_kfree(chip->dev, chip->pmic_spmi.wipower_default.entries);
	if (chip->pmic_spmi.wipower_pt.entries)
		devm_kfree(chip->dev, chip->pmic_spmi.wipower_pt.entries);
	if (chip->pmic_spmi.wipower_div2.entries)
		devm_kfree(chip->dev, chip->pmic_spmi.wipower_div2.entries);
	chip->pmic_spmi.wipower_default.entries = NULL;
	chip->pmic_spmi.wipower_pt.entries = NULL;
	chip->pmic_spmi.wipower_div2.entries = NULL;
	chip->pmic_spmi.vadc_dev = NULL;
	return rc;
}

#define DEFAULT_VLED_MAX_UV		3500000
#define DEFAULT_FCC_MA			2000
static int smb_parse_dt(struct oplus_chg_chip *chip)
{
	int rc = 0, ocp_thresh = -EINVAL;
	struct device_node *node = chip->dev->of_node;
	const char *dc_psy_type, *bpd;

	if (!node) {
		dev_err(chip->dev, "device tree info. missing\n");
		return -EINVAL;
	}

	/* read optional u32 properties */
	OF_PROP_READ(chip, ocp_thresh,
			"ibat-ocp-threshold-ua", rc, 1);
	if (ocp_thresh >= 0)
		smbchg_ibat_ocp_threshold_ua = ocp_thresh;
	OF_PROP_READ(chip, chip->pmic_spmi.iterm_ma, "iterm-ma", rc, 1);
	OF_PROP_READ(chip, chip->pmic_spmi.cfg_fastchg_current_ma,
			"fastchg-current-ma", rc, 1);
	if (chip->pmic_spmi.cfg_fastchg_current_ma == -EINVAL)
		chip->pmic_spmi.cfg_fastchg_current_ma = DEFAULT_FCC_MA;
	OF_PROP_READ(chip, chip->pmic_spmi.vfloat_mv, "float-voltage-mv", rc, 1);
	OF_PROP_READ(chip, chip->pmic_spmi.safety_time, "charging-timeout-mins", rc, 1);
	OF_PROP_READ(chip, chip->pmic_spmi.vled_max_uv, "vled-max-uv", rc, 1);
	if (chip->pmic_spmi.vled_max_uv < 0)
		chip->pmic_spmi.vled_max_uv = DEFAULT_VLED_MAX_UV;
	OF_PROP_READ(chip, chip->pmic_spmi.rpara_uohm, "rparasitic-uohm", rc, 1);
	if (chip->pmic_spmi.rpara_uohm < 0)
		chip->pmic_spmi.rpara_uohm = 0;
	OF_PROP_READ(chip, chip->pmic_spmi.prechg_safety_time, "precharging-timeout-mins",
			rc, 1);
	OF_PROP_READ(chip, chip->pmic_spmi.fastchg_current_comp, "fastchg-current-comp",
			rc, 1);
	OF_PROP_READ(chip, chip->pmic_spmi.float_voltage_comp, "float-voltage-comp",
			rc, 1);
	if (chip->pmic_spmi.safety_time != -EINVAL &&
		(chip->pmic_spmi.safety_time > chg_time[ARRAY_SIZE(chg_time) - 1])) {
		dev_err(chip->dev, "Bad charging-timeout-mins %d\n",
						chip->pmic_spmi.safety_time);
		return -EINVAL;
	}
	if (chip->pmic_spmi.prechg_safety_time != -EINVAL &&
		(chip->pmic_spmi.prechg_safety_time >
		 prechg_time[ARRAY_SIZE(prechg_time) - 1])) {
		dev_err(chip->dev, "Bad precharging-timeout-mins %d\n",
						chip->pmic_spmi.prechg_safety_time);
		return -EINVAL;
	}
	OF_PROP_READ(chip, chip->pmic_spmi.resume_delta_mv, "resume-delta-mv", rc, 1);
	OF_PROP_READ(chip, chip->pmic_spmi.parallel.min_current_thr_ma,
			"parallel-usb-min-current-ma", rc, 1);
	OF_PROP_READ(chip, chip->pmic_spmi.parallel.min_9v_current_thr_ma,
			"parallel-usb-9v-min-current-ma", rc, 1);
	OF_PROP_READ(chip, chip->pmic_spmi.parallel.allowed_lowering_ma,
			"parallel-allowed-lowering-ma", rc, 1);
	if (chip->pmic_spmi.parallel.min_current_thr_ma != -EINVAL
			&& chip->pmic_spmi.parallel.min_9v_current_thr_ma != -EINVAL)
		chip->pmic_spmi.parallel.avail = true;
	/*
	 * use the dt values if they exist, otherwise do not touch the params
	 */
	of_property_read_u32(chip->pmic_spmi.spmi->dev.of_node,
					"qcom,parallel-main-chg-fcc-percent",
					&smbchg_main_chg_fcc_percent);
	of_property_read_u32(chip->pmic_spmi.spmi->dev.of_node,
					"qcom,parallel-main-chg-icl-percent",
					&smbchg_main_chg_icl_percent);
	pr_smb(PR_STATUS, "parallel usb thr: %d, 9v thr: %d\n",
			chip->pmic_spmi.parallel.min_current_thr_ma,
			chip->pmic_spmi.parallel.min_9v_current_thr_ma);
	OF_PROP_READ(chip, chip->pmic_spmi.jeita_temp_hard_limit,
			"jeita-temp-hard-limit", rc, 1);
	OF_PROP_READ(chip, chip->pmic_spmi.aicl_rerun_period_s,
			"aicl-rerun-period-s", rc, 1);
	OF_PROP_READ(chip, chip->pmic_spmi.vchg_adc_channel,
			"vchg-adc-channel-id", rc, 1);

	/* read boolean configuration properties */
	chip->pmic_spmi.use_vfloat_adjustments = of_property_read_bool(node,
						"qcom,autoadjust-vfloat");
	chip->pmic_spmi.bmd_algo_disabled = of_property_read_bool(node,
						"qcom,bmd-algo-disabled");
	chip->pmic_spmi.iterm_disabled = of_property_read_bool(node,
						"qcom,iterm-disabled");
	chip->pmic_spmi.soft_vfloat_comp_disabled = of_property_read_bool(node,
					"qcom,soft-vfloat-comp-disabled");
	chip->pmic_spmi.chg_enabled = !(of_property_read_bool(node,
						"qcom,charging-disabled"));
	chip->pmic_spmi.charge_unknown_battery = of_property_read_bool(node,
						"qcom,charge-unknown-battery");
	chip->pmic_spmi.chg_inhibit_en = of_property_read_bool(node,
					"qcom,chg-inhibit-en");
	chip->pmic_spmi.chg_inhibit_source_fg = of_property_read_bool(node,
						"qcom,chg-inhibit-fg");
	chip->pmic_spmi.low_volt_dcin = of_property_read_bool(node,
					"qcom,low-volt-dcin");
	chip->pmic_spmi.force_aicl_rerun = of_property_read_bool(node,
					"qcom,force-aicl-rerun");
	chip->pmic_spmi.skip_usb_suspend_for_fake_battery = of_property_read_bool(node,
				"qcom,skip-usb-suspend-for-fake-battery");
	chip->pmic_spmi.skip_fg_control_chg = of_property_read_bool(node,
				"qcom,skip-fg-control-chg");
	/* parse the battery missing detection pin source */
	rc = of_property_read_string(chip->pmic_spmi.spmi->dev.of_node,
		"qcom,bmd-pin-src", &bpd);
	if (rc) {
		/* Select BAT_THM as default BPD scheme */
		chip->pmic_spmi.bmd_pin_src = BPD_TYPE_DEFAULT;
		rc = 0;
	} else {
		chip->pmic_spmi.bmd_pin_src = get_bpd(bpd);
		if (chip->pmic_spmi.bmd_pin_src < 0) {
			dev_err(chip->dev,
				"failed to determine bpd schema %d\n", rc);
			return rc;
		}
	}

	/* parse the dc power supply configuration */
	rc = of_property_read_string(node, "qcom,dc-psy-type", &dc_psy_type);
	if (rc) {
		chip->pmic_spmi.dc_psy_type = -EINVAL;
		rc = 0;
	} else {
		if (strcmp(dc_psy_type, "Mains") == 0)
			chip->pmic_spmi.dc_psy_type = POWER_SUPPLY_TYPE_MAINS;
		else if (strcmp(dc_psy_type, "Wireless") == 0)
			chip->pmic_spmi.dc_psy_type = POWER_SUPPLY_TYPE_WIRELESS;
		else if (strcmp(dc_psy_type, "Wipower") == 0)
			chip->pmic_spmi.dc_psy_type = POWER_SUPPLY_TYPE_WIPOWER;
	}
	if (chip->pmic_spmi.dc_psy_type != -EINVAL) {
		OF_PROP_READ(chip, chip->pmic_spmi.dc_target_current_ma,
				"dc-psy-ma", rc, 0);
		if (rc)
			return rc;
		if (chip->pmic_spmi.dc_target_current_ma < DC_MA_MIN
				|| chip->pmic_spmi.dc_target_current_ma > DC_MA_MAX) {
			dev_err(chip->dev, "Bad dc mA %d\n",
					chip->pmic_spmi.dc_target_current_ma);
			return -EINVAL;
		}
	}

	if (chip->pmic_spmi.dc_psy_type == POWER_SUPPLY_TYPE_WIPOWER)
		smb_parse_wipower_dt(chip);

	/* read the bms power supply name */
	rc = of_property_read_string(node, "qcom,bms-psy-name",
						&chip->pmic_spmi.bms_psy_name);
	if (rc)
		chip->pmic_spmi.bms_psy_name = NULL;

	/* read the battery power supply name */
	rc = of_property_read_string(node, "qcom,battery-psy-name",
						&chip->pmic_spmi.battery_psy_name);
	if (rc)
		chip->pmic_spmi.battery_psy_name = "battery";

	/* Get the charger led support property */
	chip->pmic_spmi.cfg_chg_led_sw_ctrl =
		of_property_read_bool(node, "qcom,chg-led-sw-controls");
	chip->pmic_spmi.cfg_chg_led_support =
		of_property_read_bool(node, "qcom,chg-led-support");

	if (of_find_property(node, "qcom,thermal-mitigation",
					&chip->pmic_spmi.thermal_levels)) {
		chip->pmic_spmi.thermal_mitigation = devm_kzalloc(chip->dev,
			chip->pmic_spmi.thermal_levels,
			GFP_KERNEL);

		if (chip->pmic_spmi.thermal_mitigation == NULL) {
			dev_err(chip->dev, "thermal mitigation kzalloc() failed.\n");
			return -ENOMEM;
		}

		chip->pmic_spmi.thermal_levels /= sizeof(int);
		rc = of_property_read_u32_array(node,
				"qcom,thermal-mitigation",
				chip->pmic_spmi.thermal_mitigation, chip->pmic_spmi.thermal_levels);
		if (rc) {
			dev_err(chip->dev,
				"Couldn't read threm limits rc = %d\n", rc);
			return rc;
		}
	}

	chip->pmic_spmi.skip_usb_notification
		= of_property_read_bool(node,
				"qcom,skip-usb-notification");

	chip->pmic_spmi.otg_pinctrl = of_property_read_bool(node, "qcom,otg-pinctrl");

	chip->pmic_spmi.cfg_override_usb_current = of_property_read_bool(node,
				"qcom,override-usb-current");
	
	chip->normalchg_gpio.chargerid_switch_gpio = 
		of_get_named_gpio(chip->pmic_spmi.spmi->dev.of_node, "qcom,chargerid_switch-gpio", 0);
	if(chip->normalchg_gpio.chargerid_switch_gpio <= 0) {
		dev_err(chip->dev,
			"Couldn't read chargerid_switch-gpio rc = %d, chargerid_switch_gpio:%d\n", 
				rc, chip->normalchg_gpio.chargerid_switch_gpio);
	} else {
		if(gpio_is_valid(chip->normalchg_gpio.chargerid_switch_gpio)){
			rc = gpio_request(chip->normalchg_gpio.chargerid_switch_gpio, "charging-switch1-gpio");
			if(rc){
				chg_err("unable to request chargerid_switch_gpio:%d\n", 
					chip->normalchg_gpio.chargerid_switch_gpio);
			} else {
				smbchg_chargerid_switch_gpio_init(chip);
			}
		}
		chg_err("chargerid_switch_gpio:%d\n", chip->normalchg_gpio.chargerid_switch_gpio);
	}
	chip->pmic_spmi.not_support_1200ma =
		of_property_read_bool(node, "qcom,not-support-1200ma");
	
	return 0;
}

#define SUBTYPE_REG			0x5
#define SMBCHG_CHGR_SUBTYPE		0x1
#define SMBCHG_OTG_SUBTYPE		0x8
#define SMBCHG_BAT_IF_SUBTYPE		0x3
#define SMBCHG_USB_CHGPTH_SUBTYPE	0x4
#define SMBCHG_DC_CHGPTH_SUBTYPE	0x5
#define SMBCHG_MISC_SUBTYPE		0x7
#define SMBCHG_LITE_CHGR_SUBTYPE	0x51
#define SMBCHG_LITE_OTG_SUBTYPE		0x58
#define SMBCHG_LITE_BAT_IF_SUBTYPE	0x53
#define SMBCHG_LITE_USB_CHGPTH_SUBTYPE	0x54
#define SMBCHG_LITE_DC_CHGPTH_SUBTYPE	0x55
#define SMBCHG_LITE_MISC_SUBTYPE	0x57
#define REQUEST_IRQ(chip, resource, irq_num, irq_name, irq_handler, flags, rc)\
do {									\
	irq_num = spmi_get_irq_byname(chip->pmic_spmi.spmi,			\
					resource, irq_name);		\
	if (irq_num < 0) {						\
		dev_err(chip->dev, "Unable to get " irq_name " irq\n");	\
		return -ENXIO;						\
	}								\
	rc = devm_request_threaded_irq(chip->dev,			\
			irq_num, NULL, irq_handler, flags, irq_name,	\
			chip);						\
	if (rc < 0) {							\
		dev_err(chip->dev, "Unable to request " irq_name " irq: %d\n",\
				rc);					\
		return -ENXIO;						\
	}								\
} while (0)

static int smbchg_request_irqs(struct oplus_chg_chip *chip)
{
	int rc = 0;
	struct resource *resource;
	struct spmi_resource *spmi_resource;
	u8 subtype;
	struct spmi_device *spmi = chip->pmic_spmi.spmi;
	unsigned long flags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING
							| IRQF_ONESHOT;

	spmi_for_each_container_dev(spmi_resource, chip->pmic_spmi.spmi) {
		if (!spmi_resource) {
				dev_err(chip->dev, "spmi resource absent\n");
			return rc;
		}

		resource = spmi_get_resource(spmi, spmi_resource,
						IORESOURCE_MEM, 0);
		if (!(resource && resource->start)) {
			dev_err(chip->dev, "node %s IO resource absent!\n",
				spmi->dev.of_node->full_name);
			return rc;
		}

		rc = smbchg_read(chip, &subtype,
				resource->start + SUBTYPE_REG, 1);
		if (rc) {
			dev_err(chip->dev, "Peripheral subtype read failed rc=%d\n",
					rc);
			return rc;
		}

		switch (subtype) {
		case SMBCHG_CHGR_SUBTYPE:
		case SMBCHG_LITE_CHGR_SUBTYPE:
			REQUEST_IRQ(chip, spmi_resource, chip->pmic_spmi.pr_error_irq,
				"chg-error", pr_error_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->pmic_spmi.taper_irq,
				"chg-taper-thr", taper_handler,
				(IRQF_TRIGGER_RISING | IRQF_ONESHOT), rc);
			disable_irq_nosync(chip->pmic_spmi.taper_irq);
			REQUEST_IRQ(chip, spmi_resource, chip->pmic_spmi.chg_term_irq,
				"chg-tcc-thr", chg_term_handler,
				(IRQF_TRIGGER_RISING | IRQF_ONESHOT), rc);
			REQUEST_IRQ(chip, spmi_resource, chip->pmic_spmi.recharge_irq,
				"chg-rechg-thr", recharge_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->pmic_spmi.fastchg_irq,
				"chg-p2f-thr", fastchg_handler, flags, rc);
			enable_irq_wake(chip->pmic_spmi.chg_term_irq);
			enable_irq_wake(chip->pmic_spmi.pr_error_irq);
			enable_irq_wake(chip->pmic_spmi.fastchg_irq);
			break;
		case SMBCHG_BAT_IF_SUBTYPE:
		case SMBCHG_LITE_BAT_IF_SUBTYPE:
			REQUEST_IRQ(chip, spmi_resource, chip->pmic_spmi.batt_hot_irq,
				"batt-hot", batt_hot_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->pmic_spmi.batt_warm_irq,
				"batt-warm", batt_warm_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->pmic_spmi.batt_cool_irq,
				"batt-cool", batt_cool_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->pmic_spmi.batt_cold_irq,
				"batt-cold", batt_cold_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->pmic_spmi.batt_missing_irq,
				"batt-missing", batt_pres_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->pmic_spmi.vbat_low_irq,
				"batt-low", vbat_low_handler, flags, rc);
			enable_irq_wake(chip->pmic_spmi.batt_hot_irq);
			enable_irq_wake(chip->pmic_spmi.batt_warm_irq);
			enable_irq_wake(chip->pmic_spmi.batt_cool_irq);
			enable_irq_wake(chip->pmic_spmi.batt_cold_irq);
			enable_irq_wake(chip->pmic_spmi.batt_missing_irq);
			enable_irq_wake(chip->pmic_spmi.vbat_low_irq);
			break;
		case SMBCHG_USB_CHGPTH_SUBTYPE:
		case SMBCHG_LITE_USB_CHGPTH_SUBTYPE:
			REQUEST_IRQ(chip, spmi_resource, chip->pmic_spmi.usbin_uv_irq,
				"usbin-uv", usbin_uv_handler,
				flags | IRQF_EARLY_RESUME, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->pmic_spmi.usbin_ov_irq,
				"usbin-ov", usbin_ov_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->pmic_spmi.src_detect_irq,
				"usbin-src-det",
				src_detect_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->pmic_spmi.aicl_done_irq,
				"aicl-done",
				aicl_done_handler,
				(IRQF_TRIGGER_RISING | IRQF_ONESHOT),
				rc);
			if (chip->pmic_spmi.schg_version != QPNP_SCHG_LITE) {
				REQUEST_IRQ(chip, spmi_resource,
					chip->pmic_spmi.otg_fail_irq, "otg-fail",
					otg_fail_handler, flags, rc);
				REQUEST_IRQ(chip, spmi_resource,
					chip->pmic_spmi.otg_oc_irq, "otg-oc",
					otg_oc_handler,
					(IRQF_TRIGGER_RISING | IRQF_ONESHOT),
					rc);
				REQUEST_IRQ(chip, spmi_resource,
					chip->pmic_spmi.usbid_change_irq, "usbid-change",
					usbid_change_handler,
					(IRQF_TRIGGER_FALLING | IRQF_ONESHOT),
					rc);
				enable_irq_wake(chip->pmic_spmi.otg_oc_irq);
				enable_irq_wake(chip->pmic_spmi.usbid_change_irq);
				enable_irq_wake(chip->pmic_spmi.otg_fail_irq);
			}
			enable_irq_wake(chip->pmic_spmi.usbin_uv_irq);
			enable_irq_wake(chip->pmic_spmi.usbin_ov_irq);
			enable_irq_wake(chip->pmic_spmi.src_detect_irq);
			if (chip->pmic_spmi.parallel.avail && chip->pmic_spmi.usb_present) {
				rc = enable_irq_wake(chip->pmic_spmi.aicl_done_irq);
				chip->pmic_spmi.enable_aicl_wake = true;
			}
			break;
		case SMBCHG_DC_CHGPTH_SUBTYPE:
		case SMBCHG_LITE_DC_CHGPTH_SUBTYPE:
			REQUEST_IRQ(chip, spmi_resource, chip->pmic_spmi.dcin_uv_irq,
				"dcin-uv", dcin_uv_handler, flags, rc);
			enable_irq_wake(chip->pmic_spmi.dcin_uv_irq);
			break;
		case SMBCHG_MISC_SUBTYPE:
		case SMBCHG_LITE_MISC_SUBTYPE:
			REQUEST_IRQ(chip, spmi_resource, chip->pmic_spmi.power_ok_irq,
				"power-ok", power_ok_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->pmic_spmi.chg_hot_irq,
				"temp-shutdown", chg_hot_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource,
				chip->pmic_spmi.wdog_timeout_irq,
				"wdog-timeout",
				wdog_timeout_handler, flags, rc);
			enable_irq_wake(chip->pmic_spmi.chg_hot_irq);
			enable_irq_wake(chip->pmic_spmi.wdog_timeout_irq);
			break;
		case SMBCHG_OTG_SUBTYPE:
			break;
		case SMBCHG_LITE_OTG_SUBTYPE:
			REQUEST_IRQ(chip, spmi_resource,
				chip->pmic_spmi.usbid_change_irq, "usbid-change",
				usbid_change_handler,
				(IRQF_TRIGGER_FALLING | IRQF_ONESHOT),
				rc);
			REQUEST_IRQ(chip, spmi_resource,
				chip->pmic_spmi.otg_oc_irq, "otg-oc",
				otg_oc_handler,
				(IRQF_TRIGGER_RISING | IRQF_ONESHOT), rc);
			REQUEST_IRQ(chip, spmi_resource,
				chip->pmic_spmi.otg_fail_irq, "otg-fail",
				otg_fail_handler, flags, rc);
			enable_irq_wake(chip->pmic_spmi.usbid_change_irq);
			enable_irq_wake(chip->pmic_spmi.otg_oc_irq);
			enable_irq_wake(chip->pmic_spmi.otg_fail_irq);
			break;
		}
	}

	return rc;
}

#define REQUIRE_BASE(chip, base, rc)					\
do {									\
	if (!rc && !chip->pmic_spmi.base) {					\
		dev_err(chip->dev, "Missing " #base "\n");		\
		rc = -EINVAL;						\
	}								\
} while (0)

static int smbchg_parse_peripherals(struct oplus_chg_chip *chip)
{
	int rc = 0;
	struct resource *resource;
	struct spmi_resource *spmi_resource;
	u8 subtype;
	struct spmi_device *spmi = chip->pmic_spmi.spmi;

	spmi_for_each_container_dev(spmi_resource, chip->pmic_spmi.spmi) {
		if (!spmi_resource) {
				dev_err(chip->dev, "spmi resource absent\n");
			return rc;
		}

		resource = spmi_get_resource(spmi, spmi_resource,
						IORESOURCE_MEM, 0);
		if (!(resource && resource->start)) {
			dev_err(chip->dev, "node %s IO resource absent!\n",
				spmi->dev.of_node->full_name);
			return rc;
		}

		rc = smbchg_read(chip, &subtype,
				resource->start + SUBTYPE_REG, 1);
		if (rc) {
			dev_err(chip->dev, "Peripheral subtype read failed rc=%d\n",
					rc);
			return rc;
		}

		switch (subtype) {
		case SMBCHG_CHGR_SUBTYPE:
		case SMBCHG_LITE_CHGR_SUBTYPE:
			chip->pmic_spmi.chgr_base = resource->start;
			break;
		case SMBCHG_BAT_IF_SUBTYPE:
		case SMBCHG_LITE_BAT_IF_SUBTYPE:
			chip->pmic_spmi.bat_if_base = resource->start;
			break;
		case SMBCHG_USB_CHGPTH_SUBTYPE:
		case SMBCHG_LITE_USB_CHGPTH_SUBTYPE:
			chip->pmic_spmi.usb_chgpth_base = resource->start;
			break;
		case SMBCHG_DC_CHGPTH_SUBTYPE:
		case SMBCHG_LITE_DC_CHGPTH_SUBTYPE:
			chip->pmic_spmi.dc_chgpth_base = resource->start;
			break;
		case SMBCHG_MISC_SUBTYPE:
		case SMBCHG_LITE_MISC_SUBTYPE:
			chip->pmic_spmi.misc_base = resource->start;
			break;
		case SMBCHG_OTG_SUBTYPE:
		case SMBCHG_LITE_OTG_SUBTYPE:
			chip->pmic_spmi.otg_base = resource->start;
			break;
		}
	}

	REQUIRE_BASE(chip, chgr_base, rc);
	REQUIRE_BASE(chip, bat_if_base, rc);
	REQUIRE_BASE(chip, usb_chgpth_base, rc);
	REQUIRE_BASE(chip, dc_chgpth_base, rc);
	REQUIRE_BASE(chip, misc_base, rc);

	return rc;
}

#if 0
static inline void dump_reg(struct oplus_chg_chip *chip, u16 addr,
		const char *name)
{
	u8 reg;

	smbchg_read(chip, &reg, addr, 1);
	pr_smb(PR_DUMP, "%s - %04X = %02X\n", name, addr, reg);
}

/* dumps useful registers for debug */
static void dump_regs(struct oplus_chg_chip *chip)
{
	u16 addr;

	/* charger peripheral */
	for (addr = 0xB; addr <= 0x10; addr++)
		dump_reg(chip, chip->pmic_spmi.chgr_base + addr, "CHGR Status");
	for (addr = 0xF0; addr <= 0xFF; addr++)
		dump_reg(chip, chip->pmic_spmi.chgr_base + addr, "CHGR Config");
	/* battery interface peripheral */
	dump_reg(chip, chip->pmic_spmi.bat_if_base + RT_STS, "BAT_IF Status");
	dump_reg(chip, chip->pmic_spmi.bat_if_base + CMD_CHG_REG, "BAT_IF Command");
	for (addr = 0xF0; addr <= 0xFB; addr++)
		dump_reg(chip, chip->pmic_spmi.bat_if_base + addr, "BAT_IF Config");
	/* usb charge path peripheral */
	for (addr = 0x7; addr <= 0x10; addr++)
		dump_reg(chip, chip->pmic_spmi.usb_chgpth_base + addr, "USB Status");
	dump_reg(chip, chip->pmic_spmi.usb_chgpth_base + CMD_IL, "USB Command");
	for (addr = 0xF0; addr <= 0xF5; addr++)
		dump_reg(chip, chip->pmic_spmi.usb_chgpth_base + addr, "USB Config");
	/* dc charge path peripheral */
	dump_reg(chip, chip->pmic_spmi.dc_chgpth_base + RT_STS, "DC Status");
	for (addr = 0xF0; addr <= 0xF6; addr++)
		dump_reg(chip, chip->pmic_spmi.dc_chgpth_base + addr, "DC Config");
	/* misc peripheral */
	dump_reg(chip, chip->pmic_spmi.misc_base + IDEV_STS, "MISC Status");
	dump_reg(chip, chip->pmic_spmi.misc_base + RT_STS, "MISC Status");
	for (addr = 0xF0; addr <= 0xF3; addr++)
		dump_reg(chip, chip->pmic_spmi.misc_base + addr, "MISC CFG");
}
#else

#define REG_BUF_LEN		20
static void copy_val_to_buf(u8 *reg_buf, u8 reg_val, int *count)
{
	int *p = count;

	if((*p) > (REG_BUF_LEN - 1)) {
		chg_err("reg_buf overflow\n");
		return;
	} 
	
	reg_buf[*p] = reg_val;
	(*p)++;
	count = p;
}

static void dump_regs(struct oplus_chg_chip *chip)
{
	u16 addr;
	u8 reg_val = 0x0;
	u8 reg_buf[REG_BUF_LEN] = {0};
	int count = 0;

	if((smbchg_debug_mask & PR_DUMP) == 0) {
		return;
	}
	/* charger peripheral */
	count = 0;
	for (addr = 0xB; addr <= 0x10; addr++) {	//6
		smbchg_read(chip, &reg_val, chip->pmic_spmi.chgr_base + addr, 1);
		copy_val_to_buf(reg_buf, reg_val, &count);
	}
	pr_smb(PR_DUMP, "CHG_STAT[0x100B-0x1010]:0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n",
		reg_buf[0],reg_buf[1],reg_buf[2],reg_buf[3],reg_buf[4],reg_buf[5]);
	
	count = 0;
	for (addr = 0xF0; addr <= 0xFF; addr++)	{ //16
		smbchg_read(chip, &reg_val, chip->pmic_spmi.chgr_base + addr, 1);
		copy_val_to_buf(reg_buf, reg_val, &count);
	}
	pr_smb(PR_DUMP, "CHG_CFG[0x10F0-0x10FF]:0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,\
		0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n",
		reg_buf[0],reg_buf[1],reg_buf[2],reg_buf[3],reg_buf[4],reg_buf[5],
		reg_buf[6],reg_buf[7],reg_buf[8],reg_buf[9],reg_buf[10],reg_buf[11],
		reg_buf[12],reg_buf[13],reg_buf[14],reg_buf[15]);

	/* battery interface peripheral */
	count = 0;
	smbchg_read(chip, &reg_val, chip->pmic_spmi.bat_if_base + RT_STS, 1);
	copy_val_to_buf(reg_buf, reg_val, &count);
	smbchg_read(chip, &reg_val, chip->pmic_spmi.bat_if_base + CMD_CHG_REG, 1);
	copy_val_to_buf(reg_buf, reg_val, &count);
	for (addr = 0xF0; addr <= 0xFB; addr++)	{	//12
		smbchg_read(chip, &reg_val, chip->pmic_spmi.bat_if_base + addr, 1);
		copy_val_to_buf(reg_buf, reg_val, &count);
	}
	pr_smb(PR_DUMP, "BAT_CFG[0x1210,42,F0-0x12FB]:0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,\
		0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n",
		reg_buf[0],reg_buf[1],reg_buf[2],reg_buf[3],reg_buf[4],reg_buf[5],
		reg_buf[6],reg_buf[7],reg_buf[8],reg_buf[9],reg_buf[10],reg_buf[11],
		reg_buf[12],reg_buf[13]);

	/* usb charge path peripheral */
	count = 0;
	for (addr = 0x7; addr <= 0x10; addr++){	//10
		smbchg_read(chip, &reg_val, chip->pmic_spmi.usb_chgpth_base + addr, 1);
		copy_val_to_buf(reg_buf, reg_val, &count);
	}
	pr_smb(PR_DUMP, "USB_STAT[0x1307-0x1310]:0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n",
		reg_buf[0],reg_buf[1],reg_buf[2],reg_buf[3],reg_buf[4],reg_buf[5],
		reg_buf[6],reg_buf[7],reg_buf[8],reg_buf[9]);

	count = 0;
	smbchg_read(chip, &reg_val, chip->pmic_spmi.usb_chgpth_base + CMD_IL, 1);
	copy_val_to_buf(reg_buf, reg_val, &count);
	for (addr = 0xF0; addr <= 0xF5; addr++)	{	//6
		smbchg_read(chip, &reg_val, chip->pmic_spmi.usb_chgpth_base + addr, 1);
		copy_val_to_buf(reg_buf, reg_val, &count);
	}
	pr_smb(PR_DUMP, "USB_CFG[0x1340,0x13F0-0x13F5]:0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n",
		reg_buf[0],reg_buf[1],reg_buf[2],reg_buf[3],reg_buf[4],reg_buf[5],reg_buf[6]);

	/* dc charge path peripheral */
	count = 0;
	smbchg_read(chip, &reg_val, chip->pmic_spmi.dc_chgpth_base + RT_STS, 1);
	copy_val_to_buf(reg_buf, reg_val, &count);
	for (addr = 0xF0; addr <= 0xF6; addr++)	{	//7
		smbchg_read(chip, &reg_val, chip->pmic_spmi.dc_chgpth_base + addr, 1);
		copy_val_to_buf(reg_buf, reg_val, &count);
	}
	pr_smb(PR_DUMP, "DC[0x1410,0x14F0-0x14F6]:0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n",
		reg_buf[0],reg_buf[1],reg_buf[2],reg_buf[3],reg_buf[4],reg_buf[5],reg_buf[6],
		reg_buf[7]);
	
	/* misc peripheral */
	count = 0;
	smbchg_read(chip, &reg_val, chip->pmic_spmi.misc_base + IDEV_STS, 1);
	copy_val_to_buf(reg_buf, reg_val, &count);
	smbchg_read(chip, &reg_val, chip->pmic_spmi.misc_base + RT_STS, 1);
	copy_val_to_buf(reg_buf, reg_val, &count);
	for (addr = 0xF0; addr <= 0xF3; addr++)	{	//4/
		smbchg_read(chip, &reg_val, chip->pmic_spmi.misc_base + addr, 1);
		copy_val_to_buf(reg_buf, reg_val, &count);
	}
	pr_smb(PR_DUMP, "MISC[0x1608,10,F0-0x16F3]:0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n",
		reg_buf[0],reg_buf[1],reg_buf[2],reg_buf[3],reg_buf[4],reg_buf[5]);
	count = 0;
	
}

#endif

static int create_debugfs_entries(struct oplus_chg_chip *chip)
{
	struct dentry *ent;

	chip->pmic_spmi.debug_root = debugfs_create_dir("qpnp-smbcharger", NULL);
	if (!chip->pmic_spmi.debug_root) {
		dev_err(chip->dev, "Couldn't create debug dir\n");
		return -EINVAL;
	}

	ent = debugfs_create_file("force_dcin_icl_check",
				  S_IFREG | S_IWUSR | S_IRUGO,
				  chip->pmic_spmi.debug_root, chip,
				  &force_dcin_icl_ops);
	if (!ent) {
		dev_err(chip->dev,
			"Couldn't create force dcin icl check file\n");
		return -EINVAL;
	}
	return 0;
}

static int smbchg_check_chg_version(struct oplus_chg_chip *chip)
{
	struct pmic_revid_data *pmic_rev_id;
	struct device_node *revid_dev_node;
	int rc;

	revid_dev_node = of_parse_phandle(chip->pmic_spmi.spmi->dev.of_node,
					"qcom,pmic-revid", 0);
	if (!revid_dev_node) {
		pr_err("Missing qcom,pmic-revid property - driver failed\n");
		return -EINVAL;
	}

	pmic_rev_id = get_revid_data(revid_dev_node);
	if (IS_ERR(pmic_rev_id)) {
		rc = PTR_ERR(revid_dev_node);
		if (rc != -EPROBE_DEFER)
			pr_err("Unable to get pmic_revid rc=%d\n", rc);
		return rc;
	}

	switch (pmic_rev_id->pmic_subtype) {
	case PMI8994:
		chip->pmic_spmi.wa_flags |= SMBCHG_AICL_DEGLITCH_WA
				| SMBCHG_BATT_OV_WA
				| SMBCHG_CC_ESR_WA
				| SMBCHG_RESTART_WA;
		use_pmi8994_tables(chip);
		chip->pmic_spmi.schg_version = QPNP_SCHG;
		break;
	case PMI8950:
		chip->pmic_spmi.wa_flags |= SMBCHG_RESTART_WA;
	case PMI8937:
		chip->pmic_spmi.wa_flags |= SMBCHG_BATT_OV_WA;
		if (pmic_rev_id->rev4 < 2) /* PMI8950 1.0 */ {
			chip->pmic_spmi.wa_flags |= SMBCHG_AICL_DEGLITCH_WA;
		} else	{ /* rev > PMI8950 v1.0 */
			chip->pmic_spmi.wa_flags |= SMBCHG_HVDCP_9V_EN_WA
					| SMBCHG_USB100_WA;
		}
		use_pmi8994_tables(chip);
		chip->pmic_spmi.tables.aicl_rerun_period_table =
				aicl_rerun_period_schg_lite;
		chip->pmic_spmi.tables.aicl_rerun_period_len =
			ARRAY_SIZE(aicl_rerun_period_schg_lite);

		chip->pmic_spmi.schg_version = QPNP_SCHG_LITE;
#ifndef OPLUS_FEATURE_CHG_BASIC
		if (pmic_rev_id->pmic_subtype == PMI8937)
			chip->pmic_spmi.hvdcp_not_supported = true;
#else
		chip->pmic_spmi.hvdcp_not_supported = true;
#endif /* OPLUS_FEATURE_CHG_BASIC */
		break;
	case PMI8996:
		chip->pmic_spmi.wa_flags |= SMBCHG_CC_ESR_WA
				| SMBCHG_FLASH_ICL_DISABLE_WA
				| SMBCHG_RESTART_WA
				| SMBCHG_FLASH_BUCK_SWITCH_FREQ_WA;
		use_pmi8996_tables(chip);
		chip->pmic_spmi.schg_version = QPNP_SCHG;
		break;
	default:
		pr_err("PMIC subtype %d not supported, WA flags not set\n",
				pmic_rev_id->pmic_subtype);
	}

	pr_smb(PR_STATUS, "pmic=%s, wa_flags=0x%x, hvdcp_supported=%s\n",
			pmic_rev_id->pmic_name, chip->pmic_spmi.wa_flags,
			chip->pmic_spmi.hvdcp_not_supported ? "false" : "true");

	return 0;
}

static void rerun_hvdcp_det_if_necessary(struct oplus_chg_chip *chip)
{
	enum power_supply_type usb_supply_type;
	char *usb_type_name;
	int rc;

#ifdef OPLUS_FEATURE_CHG_BASIC
// Add for dead battery  charging if sometimes jump into kernel may lead APSD abnormal status
	if(chip->pmic_spmi.hvdcp_not_supported)
		return;
#endif /*OPLUS_FEATURE_CHG_BASIC*/

	if (!(chip->pmic_spmi.wa_flags & SMBCHG_RESTART_WA))
		return;

#if 1
	read_usb_type(chip, &usb_type_name, &usb_supply_type);
#else
	qpnp_get_charger_type(chip, &usb_type_name, &usb_supply_type, false);
#endif
	if (usb_supply_type == POWER_SUPPLY_TYPE_USB_DCP
		&& !is_hvdcp_present(chip)) {
		pr_smb(PR_STATUS, "DCP found rerunning APSD\n");
		rc = vote(chip->pmic_spmi.usb_icl_votable,
				CHG_SUSPEND_WORKAROUND_ICL_VOTER, true, 300);
		if (rc < 0)
			pr_err("Couldn't vote for 300mA for suspend wa, going ahead rc=%d\n",
					rc);

		pr_smb(PR_STATUS, "Faking Removal\n");
		fake_insertion_removal(chip, false);
		msleep(500);
		pr_smb(PR_STATUS, "Faking Insertion\n");
		fake_insertion_removal(chip, true);
#if 1
		read_usb_type(chip, &usb_type_name, &usb_supply_type);
#else
		qpnp_get_charger_type(chip, &usb_type_name, &usb_supply_type, false);
#endif
		if (usb_supply_type != POWER_SUPPLY_TYPE_USB_DCP) {
			msleep(500);
			pr_smb(PR_STATUS, "Fake Removal again as type!=DCP\n");
			fake_insertion_removal(chip, false);
			msleep(500);
			pr_smb(PR_STATUS, "Fake Insert again as type!=DCP\n");
			fake_insertion_removal(chip, true);
		}

		rc = vote(chip->pmic_spmi.usb_icl_votable,
				CHG_SUSPEND_WORKAROUND_ICL_VOTER, false, 0);
		if (rc < 0)
			pr_err("Couldn't vote for 0 for suspend wa, going ahead rc=%d\n",
					rc);
	}
}

#if 0
static int qpnp_charger_type_get(struct oplus_chg_chip *chip) 
{ 
    union power_supply_propval ret = {0,}; 
         
    chip->usb_psy->get_property(chip->usb_psy, 
                  POWER_SUPPLY_PROP_TYPE, &ret); 

    return ret.intval; 
}
#endif

static int opchg_get_charger_type(void)
{
	enum power_supply_type usb_supply_type;
	char *usb_type_name = "null";

	if(!the_chip) {
		return POWER_SUPPLY_TYPE_UNKNOWN;
	}
#if 0
	read_usb_type(the_chip, &usb_type_name, &usb_supply_type);
	if(usb_supply_type == POWER_SUPPLY_TYPE_USB){
		usb_supply_type = qpnp_charger_type_get(the_chip);
		chip->pmic_spmi.usb_supply_type = usb_supply_type;
		pr_err("POWER_SUPPLY_TYPE_USB, use ap charger detect type:%d\n", usb_supply_type);
	}
#else
	qpnp_get_charger_type(the_chip, 
		&usb_type_name, &usb_supply_type, true);
#endif
	return usb_supply_type;
}

int opchg_get_prop_charge_type(struct oplus_chg_chip *chip)
{
	int rc;

    if (chip->pmic_spmi.suspending) {
		return rc;
	}
	rc = get_prop_charge_type(chip);
	chip->pmic_spmi.bat_charging_state = rc;

    return rc;
}

static int oplus_chg_hw_init(struct oplus_chg_chip *chip)
{
	int boot_mode = get_boot_mode();
	
	if((boot_mode != MSM_BOOT_MODE__RF) && (boot_mode != MSM_BOOT_MODE__WLAN)){
		smbchg_usb_suspend_disable(chip);
	}
	smbchg_charging_enble(chip);
	return 0;
}

#define OTG_CTRL_MASK		BIT(2)|BIT(3)
#define OTG_RID_DIS	0x60
#define OTG_RID_EN	0x68
void otg_enable_id_value (void) 
{
	int rc;
	u8 reg = 0;
	if(!the_chip) {
		pr_err("[OPLUS_CHG][%s]: smb_chg not ready!\n",__func__);
		return;
	}

	rc = smbchg_sec_masked_write(the_chip, the_chip->pmic_spmi.otg_base + OTG_CFG,
			OTG_CTRL_MASK, OTG_RID_EN);
	if(rc < 0) 
		pr_err("[OPLUS_CHG][%s]:  Can't enable otg_id value, rc = %d\n",__func__,rc);
	else
		pr_err("[OPLUS_CHG][%s]:  RC = %d\n",__func__,rc);

	rc = smbchg_read(the_chip, &reg, the_chip->pmic_spmi.otg_base + OTG_CFG, 1);
	if (rc < 0) {
		pr_err("[OPLUS_CHG][%s]:Unable to read OTG_CFG rc = %d\n",__func__,rc);
	} else {
		pr_err("[OPLUS_CHG][%s]:  Reg = 0x%02x\n",__func__,reg);
	}
	the_chip->pmic_spmi.id_value_en = true;
}

void otg_disable_id_value (void)
{
	int rc;
	u8 reg = 0;	
	if(!the_chip) {
		pr_err("[OPLUS_CHG][%s]: smb_chg not ready!\n",__func__);
		return;
	}	

	rc = smbchg_sec_masked_write(the_chip, the_chip->pmic_spmi.otg_base + OTG_CFG,
			OTG_CTRL_MASK, OTG_RID_DIS);
	if(rc < 0)
		pr_err("[OPLUS_CHG][%s]:  Can't disable otg_id value, rc = %d\n",__func__,rc);
	else
		pr_err("[OPLUS_CHG][%s]:  RC = %d\n",__func__,rc);

	rc = smbchg_read(the_chip, &reg, the_chip->pmic_spmi.otg_base + OTG_CFG, 1);
	if (rc < 0) {
		pr_err("[OPLUS_CHG][%s]:Unable to read OTG_CFG rc = %d\n",__func__,rc);
	} else {
		pr_err("[OPLUS_CHG][%s]:  Reg = 0x%02x\n",__func__,reg);
	}
	the_chip->pmic_spmi.id_value_en = false;
}


static int smbchg_kick_wdt(struct oplus_chg_chip *chip)
{
	return 0;
}

static void smbchg_set_aicl_point(struct oplus_chg_chip *chip, int vol)
{

}

static int smbchg_reset_charger(struct oplus_chg_chip *chip)
{
	return 0;
}

static int smbchg_term_current_set(struct oplus_chg_chip *chip, int term_current)
{
	return smbchg_iterm_set(chip, term_current);
}

static int smbchg_set_rechg_vol(struct oplus_chg_chip *chip, int rechg_vol)
{
	return 0;
}

static int smbchg_otg_enable(void)
{
	return 0;
}

static int smbchg_otg_disable(void)
{
	return 0;
}

static int smbchg_set_chging_term_disable(struct oplus_chg_chip *chip)
{
	return 0;
}

static int oplus_chg_get_shutdown_soc(void)
{
	return 0;
}

static int oplus_chg_backup_soc(int backup_soc)
{
	return 0;
}

static int smbchg_get_boot_reason(void)
{
	return 0;
}

static bool qcom_check_charger_resume(struct oplus_chg_chip *chip)
{
	return true;
}

static bool smbchg_need_to_check_ibatt(struct oplus_chg_chip *chip)
{
	return false;
}

static int smbchg_get_chg_current_step(struct oplus_chg_chip *chip)
{
	return 50;
}

#define SHIP_MODE_CONFIG		0x40
#define SHIP_MODE_MASK			BIT(0)
#define SHIP_MODE_ENABLE		0
static void smbchg_enter_shipmode(struct oplus_chg_chip *chip)
{
	chg_err("enter\n");
	smbchg_sec_masked_write(chip, chip->pmic_spmi.bat_if_base + SHIP_MODE_CONFIG, 
		SHIP_MODE_MASK, SHIP_MODE_ENABLE);
}

static struct oplus_chg_operations  smb_chg_ops = {
	.dump_registers = dump_regs,
	.kick_wdt = smbchg_kick_wdt,
	.hardware_init = oplus_chg_hw_init,
	.charging_current_write_fast = smbchg_set_fastchg_current_raw,
	.set_aicl_point = smbchg_set_aicl_point,
	.input_current_write = oplus_chg_set_input_current,
	.float_voltage_write = smbchg_float_voltage_set,
	.term_current_set = smbchg_term_current_set,
	.charging_enable = smbchg_charging_enble,
	.charging_disable = smbchg_charging_disble,
	.get_charging_enable = smbchg_get_charge_enable,
	.charger_suspend = smbchg_usb_suspend_enable,
	.charger_unsuspend = smbchg_usb_suspend_disable,
	.set_rechg_vol = smbchg_set_rechg_vol,
	.reset_charger = smbchg_reset_charger,
	.read_full = smbchg_read_full,
	.otg_enable = smbchg_otg_enable,
	.otg_disable = smbchg_otg_disable,
	.set_charging_term_disable = smbchg_set_chging_term_disable,
	.check_charger_resume = qcom_check_charger_resume,
	.get_chargerid_volt = smbchg_get_chargerid_volt,
	.set_chargerid_switch_val = smbchg_set_chargerid_switch_val,
	.get_chargerid_switch_val = smbchg_get_chargerid_switch_val,
	.need_to_check_ibatt = smbchg_need_to_check_ibatt,
	.get_chg_current_step = smbchg_get_chg_current_step,
//#ifdef CONFIG_OPLUS_CHARGER_MTK	
#ifdef 	CONFIG_OPLUS_CHARGER_MTK
	.get_charger_type = mt_power_supply_type_check,
	.get_charger_volt = battery_meter_get_charger_voltage,
	.check_chrdet_status = pmic_chrdet_status,
	.get_instant_vbatt = battery_meter_get_battery_voltage,
	.get_boot_mode = get_boot_mode,
	.get_boot_reason = get_boot_reason,
#ifdef CONFIG_MTK_HAFG_20
	.get_rtc_soc = get_rtc_spare_oplus_fg_value,
	.set_rtc_soc = set_rtc_spare_oplus_fg_value,
#else
	.get_rtc_soc = get_rtc_spare_fg_value,
	.set_rtc_soc = set_rtc_spare_fg_value,
#endif	/* CONFIG_MTK_HAFG_20 */
	.set_power_off = mt_power_off,
	.usb_connect = mt_usb_connect,
	.usb_disconnect = mt_usb_disconnect,
#else
	.get_charger_type = opchg_get_charger_type,
	.get_charger_volt = qpnp_get_prop_charger_voltage_now,
	.check_chrdet_status = oplus_chg_is_usb_present,
	.get_instant_vbatt = qpnp_get_battery_voltage,
	.get_boot_mode = get_boot_mode,
	.get_boot_reason = smbchg_get_boot_reason,
	.get_rtc_soc = oplus_chg_get_shutdown_soc,
	.set_rtc_soc = oplus_chg_backup_soc,
	.get_aicl_ma = smbchg_get_aicl_level_ma,
	.rerun_aicl = smbchg_rerun_aicl,
	.tlim_en = smbchg_force_tlim_en,
	.set_system_temp_level = smbchg_system_temp_level_set,
	.otg_pulse_skip_disable = smbchg_set_prop_flash_active,
	.set_dp_dm = smbchg_dp_dm,
	.calc_flash_current = smbchg_calc_max_flash_current,
#endif	/* CONFIG_OPLUS_CHARGER_MTK */
#ifdef CONFIG_OPLUS_RTC_DET_SUPPORT
	.check_rtc_reset = rtc_reset_check,
#endif
#ifdef CONFIG_OPLUS_SHORT_C_BATT_CHECK
	.get_dyna_aicl_result = oplus_chg_get_dyna_aicl_result,
#endif /* CONFIG_OPLUS_SHORT_C_BATT_CHECK */

};

static int smbchg_probe(struct spmi_device *spmi)
{
	int rc;
	struct oplus_chg_chip *chip;
	struct power_supply *usb_psy, *typec_psy = NULL;
	struct qpnp_vadc_chip *vadc_dev = NULL, *vchg_vadc_dev = NULL, *pm8953_vadc_dev = NULL;
	const char *typec_psy_name;
	
	chg_err("SMBCHG:       smbchg_probe Start  \n");
	
	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		pr_smb(PR_STATUS, "USB supply not found, deferring probe\n");
		return -EPROBE_DEFER;
	}

	if(oplus_gauge_check_chip_is_null()) {
		chg_err("gauge chip null, will do after bettery init.\n");
		return -EPROBE_DEFER;
	}

	if (of_property_read_bool(spmi->dev.of_node, "qcom,external-typec")) {
		/* read the type power supply name */
		rc = of_property_read_string(spmi->dev.of_node,
				"qcom,typec-psy-name", &typec_psy_name);
		if (rc) {
			pr_err("failed to get prop typec-psy-name rc=%d\n",
				rc);
			return rc;
		}

		typec_psy = power_supply_get_by_name(typec_psy_name);
		if (!typec_psy) {
			pr_smb(PR_STATUS,
				"Type-C supply not found, deferring probe\n");
			return -EPROBE_DEFER;
		}
	}

	if (of_find_property(spmi->dev.of_node, "qcom,dcin-vadc", NULL)) {
		vadc_dev = qpnp_get_vadc(&spmi->dev, "dcin");
		if (IS_ERR(vadc_dev)) {
			rc = PTR_ERR(vadc_dev);
			if (rc != -EPROBE_DEFER)
				dev_err(&spmi->dev, "Couldn't get vadc rc=%d\n",
						rc);
			return rc;
		}
	}

	if (of_find_property(spmi->dev.of_node, "qcom,vchg_sns-vadc", NULL)) {
		vchg_vadc_dev = qpnp_get_vadc(&spmi->dev, "vchg_sns");
		if (IS_ERR(vchg_vadc_dev)) {
			rc = PTR_ERR(vchg_vadc_dev);
			if (rc != -EPROBE_DEFER)
				dev_err(&spmi->dev, "Couldn't get vadc 'vchg' rc=%d\n",
						rc);
			return rc;
		}
	}

	if (of_find_property(spmi->dev.of_node, "qcom,pm8953chg-vadc", NULL)) {
		pm8953_vadc_dev = qpnp_get_vadc(&spmi->dev, "pm8953chg");
		if (IS_ERR(pm8953_vadc_dev)) {
			rc = PTR_ERR(pm8953_vadc_dev);
			if (rc != -EPROBE_DEFER)
				dev_err(&spmi->dev, "Couldn't get vadc 'pm8953chg' rc=%d\n",
						rc);
			return rc;
		}
	}
	
	chip = devm_kzalloc(&spmi->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&spmi->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	chip->pmic_spmi.fcc_votable = create_votable(&spmi->dev,
			"SMBCHG: fcc",
			VOTE_MIN, NUM_FCC_VOTER, 2000,
			set_fastchg_current_vote_cb);
	if (IS_ERR(chip->pmic_spmi.fcc_votable))
		return PTR_ERR(chip->pmic_spmi.fcc_votable);

	chip->pmic_spmi.usb_icl_votable = create_votable(&spmi->dev,
			"SMBCHG: usb_icl",
			VOTE_MIN, NUM_ICL_VOTER, 3000,
			set_usb_current_limit_vote_cb);
	if (IS_ERR(chip->pmic_spmi.usb_icl_votable))
		return PTR_ERR(chip->pmic_spmi.usb_icl_votable);

	chip->pmic_spmi.dc_icl_votable = create_votable(&spmi->dev,
			"SMBCHG: dcl_icl",
			VOTE_MIN, NUM_ICL_VOTER, 3000,
			set_dc_current_limit_vote_cb);
	if (IS_ERR(chip->pmic_spmi.dc_icl_votable))
		return PTR_ERR(chip->pmic_spmi.dc_icl_votable);

	chip->pmic_spmi.usb_suspend_votable = create_votable(&spmi->dev,
			"SMBCHG: usb_suspend",
			VOTE_SET_ANY, NUM_EN_VOTERS, 0,
			usb_suspend_vote_cb);
	if (IS_ERR(chip->pmic_spmi.usb_suspend_votable))
		return PTR_ERR(chip->pmic_spmi.usb_suspend_votable);

	chip->pmic_spmi.dc_suspend_votable = create_votable(&spmi->dev,
			"SMBCHG: dc_suspend",
			VOTE_SET_ANY, NUM_EN_VOTERS, 0,
			dc_suspend_vote_cb);
	if (IS_ERR(chip->pmic_spmi.dc_suspend_votable))
		return PTR_ERR(chip->pmic_spmi.dc_suspend_votable);

	chip->pmic_spmi.battchg_suspend_votable = create_votable(&spmi->dev,
			"SMBCHG: battchg_suspend",
			VOTE_SET_ANY, NUM_BATTCHG_EN_VOTERS, 0,
			charging_suspend_vote_cb);
	if (IS_ERR(chip->pmic_spmi.battchg_suspend_votable))
		return PTR_ERR(chip->pmic_spmi.battchg_suspend_votable);

	chip->pmic_spmi.hw_aicl_rerun_disable_votable = create_votable(&spmi->dev,
			"SMBCHG: hwaicl_disable",
			VOTE_SET_ANY, NUM_HW_AICL_DISABLE_VOTERS, 0,
			smbchg_hw_aicl_rerun_disable_cb);
	if (IS_ERR(chip->pmic_spmi.hw_aicl_rerun_disable_votable))
		return PTR_ERR(chip->pmic_spmi.hw_aicl_rerun_disable_votable);

	chip->pmic_spmi.hw_aicl_rerun_enable_indirect_votable = create_votable(&spmi->dev,
			"SMBCHG: hwaicl_enable_indirect",
			VOTE_SET_ANY, NUM_HW_AICL_RERUN_ENABLE_INDIRECT_VOTERS,
			0, smbchg_hw_aicl_rerun_enable_indirect_cb);
	if (IS_ERR(chip->pmic_spmi.hw_aicl_rerun_enable_indirect_votable))
		return PTR_ERR(chip->pmic_spmi.hw_aicl_rerun_enable_indirect_votable);

	chip->pmic_spmi.aicl_deglitch_short_votable = create_votable(&spmi->dev,
			"SMBCHG: hwaicl_short_deglitch",
			VOTE_SET_ANY, NUM_HW_SHORT_DEGLITCH_VOTERS, 0,
			smbchg_aicl_deglitch_config_cb);
	if (IS_ERR(chip->pmic_spmi.aicl_deglitch_short_votable))
		return PTR_ERR(chip->pmic_spmi.aicl_deglitch_short_votable);

	INIT_WORK(&chip->pmic_spmi.usb_set_online_work, smbchg_usb_update_online_work);
	INIT_DELAYED_WORK(&chip->pmic_spmi.parallel_en_work,
			smbchg_parallel_usb_en_work);
	INIT_DELAYED_WORK(&chip->pmic_spmi.vfloat_adjust_work, smbchg_vfloat_adjust_work);
	INIT_DELAYED_WORK(&chip->pmic_spmi.hvdcp_det_work, smbchg_hvdcp_det_work);
	init_completion(&chip->pmic_spmi.src_det_lowered);
	init_completion(&chip->pmic_spmi.src_det_raised);
	init_completion(&chip->pmic_spmi.usbin_uv_lowered);
	init_completion(&chip->pmic_spmi.usbin_uv_raised);
	chip->pmic_spmi.vadc_dev = vadc_dev;
	chip->pmic_spmi.vchg_vadc_dev = vchg_vadc_dev;
	chip->pmic_spmi.pm8953_vadc_dev = pm8953_vadc_dev;
	chip->pmic_spmi.spmi = spmi;
	chip->dev = &spmi->dev;
	chip->usb_psy = usb_psy;
	chip->pmic_spmi.typec_psy = typec_psy;
	chip->pmic_spmi.fake_battery_soc = -EINVAL;
	chip->pmic_spmi.usb_online = -EINVAL;
	chip->chg_ops = &smb_chg_ops;
	dev_set_drvdata(&spmi->dev, chip);

	spin_lock_init(&chip->pmic_spmi.sec_access_lock);
	mutex_init(&chip->pmic_spmi.therm_lvl_lock);
	mutex_init(&chip->pmic_spmi.usb_set_online_lock);
	mutex_init(&chip->pmic_spmi.parallel.lock);
	mutex_init(&chip->pmic_spmi.taper_irq_lock);
	mutex_init(&chip->pmic_spmi.pm_lock);
	mutex_init(&chip->pmic_spmi.wipower_config);
	mutex_init(&chip->pmic_spmi.usb_status_lock);
	device_init_wakeup(chip->dev, true);

	rc = smbchg_parse_peripherals(chip);
	if (rc) {
		dev_err(chip->dev, "Error parsing DT peripherals: %d\n", rc);
		return rc;
	}

	rc = smbchg_check_chg_version(chip);
	if (rc) {
		pr_err("Unable to check schg version rc=%d\n", rc);
		return rc;
	}

	rc = smb_parse_dt(chip);
	if (rc < 0) {
		dev_err(&spmi->dev, "Unable to parse DT nodes: %d\n", rc);
		return rc;
	}

	rc = smbchg_regulator_init(chip);
	if (rc) {
		dev_err(&spmi->dev,
			"Couldn't initialize regulator rc=%d\n", rc);
		return rc;
	}

	rc = smbchg_hw_init(chip);
	if (rc < 0) {
		dev_err(&spmi->dev,
			"Unable to intialize hardware rc = %d\n", rc);
		goto out;
	}

	rc = determine_initial_status(chip);
	if (rc < 0) {
		dev_err(&spmi->dev,
			"Unable to determine init status rc = %d\n", rc);
		goto out;
	}

#if 0
	chip->pmic_spmi.previous_soc = -EINVAL;
	chip->batt_psy.name		= chip->pmic_spmi.battery_psy_name;
	chip->batt_psy.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->batt_psy.get_property	= smbchg_battery_get_property;
	chip->batt_psy.set_property	= smbchg_battery_set_property;
	chip->batt_psy.properties	= smbchg_battery_properties;
	chip->batt_psy.num_properties	= ARRAY_SIZE(smbchg_battery_properties);
	chip->batt_psy.external_power_changed = smbchg_external_power_changed;
	chip->batt_psy.property_is_writeable = smbchg_battery_is_writeable;

	rc = power_supply_register(chip->dev, &chip->batt_psy);
	if (rc < 0) {
		dev_err(&spmi->dev,
			"Unable to register batt_psy rc = %d\n", rc);
		goto out;
	}
	if (chip->pmic_spmi.dc_psy_type != -EINVAL) {
		chip->pmic_spmi.dc_psy.name		= "dc";
		chip->pmic_spmi.dc_psy.type		= chip->pmic_spmi.dc_psy_type;
		chip->pmic_spmi.dc_psy.get_property	= smbchg_dc_get_property;
		chip->pmic_spmi.dc_psy.set_property	= smbchg_dc_set_property;
		chip->pmic_spmi.dc_psy.property_is_writeable = smbchg_dc_is_writeable;
		chip->pmic_spmi.dc_psy.properties		= smbchg_dc_properties;
		chip->pmic_spmi.dc_psy.num_properties = ARRAY_SIZE(smbchg_dc_properties);
		chip->pmic_spmi.dc_psy.supplied_to = smbchg_dc_supplicants;
		chip->pmic_spmi.dc_psy.num_supplicants
			= ARRAY_SIZE(smbchg_dc_supplicants);
		rc = power_supply_register(chip->dev, &chip->pmic_spmi.dc_psy);
		if (rc < 0) {
			dev_err(&spmi->dev,
				"Unable to register dc_psy rc = %d\n", rc);
			goto unregister_batt_psy;
		}
	}
	chip->pmic_spmi.psy_registered = true;
#endif

	if (chip->pmic_spmi.cfg_chg_led_support &&
			chip->pmic_spmi.schg_version == QPNP_SCHG_LITE) {
		rc = smbchg_register_chg_led(chip);
		if (rc) {
			dev_err(chip->dev,
					"Unable to register charger led: %d\n",
					rc);
			goto unregister_dc_psy;
		}

		rc = smbchg_chg_led_controls(chip);
		if (rc) {
			dev_err(chip->dev,
					"Failed to set charger led controld bit: %d\n",
					rc);
			goto unregister_led_class;
		}
	}
#ifdef OPLUS_FEATURE_CHG_BASIC
	oplus_chg_parse_dt(chip);
	oplus_chg_init(chip);
#endif

	rc = smbchg_request_irqs(chip);
	if (rc < 0) {
		dev_err(&spmi->dev, "Unable to request irqs rc = %d\n", rc);
		goto unregister_led_class;
	}

	if (!chip->pmic_spmi.skip_usb_notification) {
		pr_smb(PR_MISC, "setting usb psy present = %d\n",
			chip->pmic_spmi.usb_present);
		power_supply_set_present(chip->usb_psy, chip->pmic_spmi.usb_present);
	}

	rerun_hvdcp_det_if_necessary(chip);

	dump_regs(chip);
	create_debugfs_entries(chip);
	chip->authenticate = oplus_gauge_get_batt_authenticate();
	if(!chip->authenticate)
		smbchg_charging_disble(chip);
	the_chip = chip;

	oplus_chg_wake_update_work();
	dev_info(chip->dev,
		"SMBCHG: successfully probe Charger version=%s Revision DIG:%d.%d ANA:%d.%d batt=%d dc=%d usb=%d batt_authen=%d\n",
			version_str[chip->pmic_spmi.schg_version],
			chip->pmic_spmi.revision[DIG_MAJOR], chip->pmic_spmi.revision[DIG_MINOR],
			chip->pmic_spmi.revision[ANA_MAJOR], chip->pmic_spmi.revision[ANA_MINOR],
			get_prop_batt_present(chip),
			chip->pmic_spmi.dc_present, chip->pmic_spmi.usb_present, chip->authenticate);
	return 0;

unregister_led_class:
	if (chip->pmic_spmi.cfg_chg_led_support && chip->pmic_spmi.schg_version == QPNP_SCHG_LITE)
		led_classdev_unregister(&chip->pmic_spmi.led_cdev);
unregister_dc_psy:
	power_supply_unregister(&chip->pmic_spmi.dc_psy);
#if 0
unregister_batt_psy:
	power_supply_unregister(&chip->batt_psy);
#else
	power_supply_unregister(&chip->batt_psy);
#endif
out:
	handle_usb_removal(chip);
	return rc;
}

static int smbchg_remove(struct spmi_device *spmi)
{
	struct oplus_chg_chip *chip = dev_get_drvdata(&spmi->dev);

	debugfs_remove_recursive(chip->pmic_spmi.debug_root);

	if (chip->pmic_spmi.dc_psy_type != -EINVAL)
		power_supply_unregister(&chip->pmic_spmi.dc_psy);

	power_supply_unregister(&chip->batt_psy);

	return 0;
}

static unsigned long suspend_tm_sec = 0;

static int smbchg_resume(struct spmi_device *spmi)
{	
	unsigned long resume_tm_sec = 0, sleep_time = 0;
	int rc = 0;

	if(!the_chip) {
		return 0;
	}
	rc = get_current_time(&resume_tm_sec);
	if (rc || suspend_tm_sec == -1) {
		chg_err("RTC read failed\n");
		sleep_time = 0;
	} else {
		sleep_time = resume_tm_sec - suspend_tm_sec;
	}

	if(sleep_time < 0) {
		sleep_time = 0;
	}
	oplus_chg_soc_update_when_resume(sleep_time);
	return 0;
}

static int smbchg_suspend(struct spmi_device *spmi, pm_message_t mesg)
{
	if(!the_chip) {
		return 0;
	}
	if (get_current_time(&suspend_tm_sec)) {
		chg_err("RTC read failed\n");
		suspend_tm_sec = -1;
	}
	return 0;
}

static void smbchg_shutdown(struct spmi_device *spmi)
{
	struct oplus_chg_chip *chip = dev_get_drvdata(&spmi->dev);
	int i, rc;

	smbchg_set_chargerid_switch_val(chip, 0);
	msleep(30);
	
	if(chip->pmic_spmi.hvdcp_not_supported)
		goto ship_mode;
	
	if (!(chip->pmic_spmi.wa_flags & SMBCHG_RESTART_WA))
		goto ship_mode;

	if (!is_hvdcp_present(chip))
		goto ship_mode;

	pr_smb(PR_MISC, "Disable Parallel\n");
	mutex_lock(&chip->pmic_spmi.parallel.lock);
	smbchg_parallel_en = 0;
	smbchg_parallel_usb_disable(chip);
	mutex_unlock(&chip->pmic_spmi.parallel.lock);

	pr_smb(PR_MISC, "Disable all interrupts\n");
	disable_irq(chip->pmic_spmi.aicl_done_irq);
	disable_irq(chip->pmic_spmi.batt_cold_irq);
	disable_irq(chip->pmic_spmi.batt_cool_irq);
	disable_irq(chip->pmic_spmi.batt_hot_irq);
	disable_irq(chip->pmic_spmi.batt_missing_irq);
	disable_irq(chip->pmic_spmi.batt_warm_irq);
	disable_irq(chip->pmic_spmi.pr_error_irq);
	disable_irq(chip->pmic_spmi.chg_hot_irq);
	disable_irq(chip->pmic_spmi.chg_term_irq);
	disable_irq(chip->pmic_spmi.dcin_uv_irq);
	disable_irq(chip->pmic_spmi.fastchg_irq);
	disable_irq(chip->pmic_spmi.otg_fail_irq);
	disable_irq(chip->pmic_spmi.otg_oc_irq);
	disable_irq(chip->pmic_spmi.power_ok_irq);
	disable_irq(chip->pmic_spmi.recharge_irq);
	disable_irq(chip->pmic_spmi.src_detect_irq);
	disable_irq(chip->pmic_spmi.taper_irq);
	disable_irq(chip->pmic_spmi.usbid_change_irq);
	disable_irq(chip->pmic_spmi.usbin_ov_irq);
	disable_irq(chip->pmic_spmi.usbin_uv_irq);
	disable_irq(chip->pmic_spmi.vbat_low_irq);
	disable_irq(chip->pmic_spmi.wdog_timeout_irq);

	/* remove all votes for short deglitch */
	for (i = 0; i < NUM_HW_SHORT_DEGLITCH_VOTERS; i++)
		vote(chip->pmic_spmi.aicl_deglitch_short_votable, i, false, 0);

	/* vote to ensure AICL rerun is enabled */
	rc = vote(chip->pmic_spmi.hw_aicl_rerun_enable_indirect_votable,
			SHUTDOWN_WORKAROUND_VOTER, true, 0);
	if (rc < 0)
		pr_err("Couldn't vote to enable indirect AICL rerun\n");
	rc = vote(chip->pmic_spmi.hw_aicl_rerun_disable_votable,
		WEAK_CHARGER_HW_AICL_VOTER, false, 0);
	if (rc < 0)
		pr_err("Couldn't vote to enable AICL rerun\n");

	/* switch to 5V HVDCP */
	pr_smb(PR_MISC, "Switch to 5V HVDCP\n");
	rc = smbchg_sec_masked_write(chip, chip->pmic_spmi.usb_chgpth_base + CHGPTH_CFG,
				HVDCP_ADAPTER_SEL_MASK, HVDCP_5V);
	if (rc < 0) {
		pr_err("Couldn't configure HVDCP 5V rc=%d\n", rc);
		goto ship_mode;
	}

	pr_smb(PR_MISC, "Wait 500mS to lower to 5V\n");
	/* wait for HVDCP to lower to 5V */
	msleep(500);
	/*
	 * Check if the same hvdcp session is in progress. src_det should be
	 * high and that we are still in 5V hvdcp
	 */
	if (!is_src_detect_high(chip)) {
		pr_smb(PR_MISC, "src det low after 500mS sleep\n");
		goto ship_mode;
	}

	/* disable HVDCP */
	pr_smb(PR_MISC, "Disable HVDCP\n");
	rc = smbchg_sec_masked_write(chip, chip->pmic_spmi.usb_chgpth_base + CHGPTH_CFG,
			HVDCP_EN_BIT, 0);
	if (rc < 0)
		pr_err("Couldn't disable HVDCP rc=%d\n", rc);

	chip->pmic_spmi.hvdcp_3_det_ignore_uv = true;
	/* fake a removal */
	pr_smb(PR_MISC, "Faking Removal\n");
	rc = fake_insertion_removal(chip, false);
	if (rc < 0)
		pr_err("Couldn't fake removal HVDCP Removed rc=%d\n", rc);

	/* fake an insertion */
	pr_smb(PR_MISC, "Faking Insertion\n");
	rc = fake_insertion_removal(chip, true);
	if (rc < 0)
		pr_err("Couldn't fake insertion rc=%d\n", rc);

	pr_smb(PR_MISC, "Wait 1S to settle\n");
	msleep(1000);
	chip->pmic_spmi.hvdcp_3_det_ignore_uv = false;

	pr_smb(PR_STATUS, "wrote power off configurations\n");
 ship_mode:
 	if(chip->enable_shipmode) {
		msleep(1000);		// lwb add for avoid i2c write fail
		smbchg_enter_shipmode(chip);
	}
}

#if 0
static const struct dev_pm_ops smbchg_pm_ops = {
};
#endif

MODULE_DEVICE_TABLE(spmi, smbchg_id);

static struct spmi_driver smbchg_driver = {
	.driver		= {
		.name		= "qpnp-smbcharger",
		.owner		= THIS_MODULE,
		.of_match_table	= smbchg_match_table,
		//.pm		= &smbchg_pm_ops,
	},
	.probe		= smbchg_probe,
	.resume		= smbchg_resume,
	.suspend	= smbchg_suspend,
	.remove		= smbchg_remove,
	.shutdown	= smbchg_shutdown,
};

static int __init smbchg_init(void)
{
	return spmi_driver_register(&smbchg_driver);
}

static void __exit smbchg_exit(void)
{
	return spmi_driver_unregister(&smbchg_driver);
}

module_init(smbchg_init);
module_exit(smbchg_exit);

MODULE_DESCRIPTION("QPNP SMB Charger");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:qpnp-smbcharger");
