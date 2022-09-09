/************************************************************************************
** File: 
** OPLUS_FEATURE_CHG_BASIC
** Copyright (C), 2008-2016, OPLUS Mobile Comm Corp., Ltd
** 
** Description: 
**      for MSM8998 charger
** 
** Version: 1.0
** Date created: 26/12/2016
** 
** --------------------------- Revision History: ------------------------------------------------------------
* <version>       <date>        <author>              			<desc>
************************************************************************************************************/

#include <linux/device.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/iio/consumer.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/qpnp/qpnp-revid.h>
#include <linux/input/qpnp-power-on.h>
#include <linux/irq.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/qpnp/qpnp-revid.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>

#ifdef OPLUS_FEATURE_CHG_BASIC
#include <linux/gpio.h>
#include <soc/oplus/system/boot_mode.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/iio/consumer.h>
#include <linux/rtc.h>
#include <linux/proc_fs.h>
#include <soc/oplus/system/boot_mode.h>
#include <soc/oplus/device_info.h>
#include <soc/oplus/system/oplus_project.h>

#include "../oplus_charger.h"
#include "../oplus_gauge.h"
#include "../oplus_vooc.h"
#include "../oplus_adapter.h"
#include "../oplus_short.h"
#include "oplus_bq25882.h"
#include "../gauge_ic/oplus_bq27541.h"



struct oplus_chg_chip *g_oplus_chip = NULL;
void smbchg_set_chargerid_switch_val(int value);
static int smbchg_chargerid_switch_gpio_init(struct oplus_chg_chip *chip);
bool oplus_usbid_check_is_gpio(struct oplus_chg_chip *chip);
static int oplus_usbid_gpio_init(struct oplus_chg_chip *chip);
static void oplus_usbid_irq_init(struct oplus_chg_chip *chip);
static void oplus_set_usbid_active(struct oplus_chg_chip *chip);
static void oplus_set_usbid_sleep(struct oplus_chg_chip *chip);
#define OPLUS_CHG_MONITOR_INTERVAL round_jiffies_relative(msecs_to_jiffies(5000))
#endif



#define smblib_err(chg, fmt, ...)		\
	pr_err("%s: %s: " fmt, chg->name,	\
		__func__, ##__VA_ARGS__)	\

#ifndef OPLUS_FEATURE_CHG_BASIC
#define smblib_dbg(chg, reason, fmt, ...)			\
	do {							\
		if (*chg->debug_mask & (reason))		\
			pr_err("%s: %s: " fmt, chg->name,	\
				__func__, ##__VA_ARGS__);	\
		else						\
			pr_debug("%s: %s: " fmt, chg->name,	\
				__func__, ##__VA_ARGS__);	\
	} while (0)
#else
#define smblib_dbg(chg, reason, fmt, ...)			\
	do {							\
		if (*chg->debug_mask & (reason))		\
			pr_info("%s: %s: " fmt, chg->name,	\
				__func__, ##__VA_ARGS__);	\
		else						\
			pr_debug("%s: %s: " fmt, chg->name,	\
				__func__, ##__VA_ARGS__);	\
	} while (0)
#endif

#define SMB2_DEFAULT_WPWR_UW	8000000

static struct smb_params v1_params = {
	.fcc			= {
		.name	= "fast charge current",
		.reg	= FAST_CHARGE_CURRENT_CFG_REG,
		.min_u	= 0,
		.max_u	= 4500000,
		.step_u	= 25000,
	},
	.fv			= {
		.name	= "float voltage",
		.reg	= FLOAT_VOLTAGE_CFG_REG,
		.min_u	= 3487500,
		.max_u	= 4920000,
		.step_u	= 7500,
	},
	.usb_icl		= {
		.name	= "usb input current limit",
		.reg	= USBIN_CURRENT_LIMIT_CFG_REG,
		.min_u	= 0,
		.max_u	= 4800000,
		.step_u	= 25000,
	},
	.icl_stat		= {
		.name	= "input current limit status",
		.reg	= ICL_STATUS_REG,
		.min_u	= 0,
		.max_u	= 4800000,
		.step_u	= 25000,
	},
	.otg_cl			= {
		.name	= "usb otg current limit",
		.reg	= OTG_CURRENT_LIMIT_CFG_REG,
		.min_u	= 250000,
		.max_u	= 2000000,
		.step_u	= 250000,
	},
	.dc_icl			= {
		.name	= "dc input current limit",
		.reg	= DCIN_CURRENT_LIMIT_CFG_REG,
		.min_u	= 0,
		.max_u	= 6000000,
		.step_u	= 25000,
	},
	.dc_icl_pt_lv		= {
		.name	= "dc icl PT <8V",
		.reg	= ZIN_ICL_PT_REG,
		.min_u	= 0,
		.max_u	= 3000000,
		.step_u	= 25000,
	},
	.dc_icl_pt_hv		= {
		.name	= "dc icl PT >8V",
		.reg	= ZIN_ICL_PT_HV_REG,
		.min_u	= 0,
		.max_u	= 3000000,
		.step_u	= 25000,
	},
	.dc_icl_div2_lv		= {
		.name	= "dc icl div2 <5.5V",
		.reg	= ZIN_ICL_LV_REG,
		.min_u	= 0,
		.max_u	= 3000000,
		.step_u	= 25000,
	},
	.dc_icl_div2_mid_lv	= {
		.name	= "dc icl div2 5.5-6.5V",
		.reg	= ZIN_ICL_MID_LV_REG,
		.min_u	= 0,
		.max_u	= 3000000,
		.step_u	= 25000,
	},
	.dc_icl_div2_mid_hv	= {
		.name	= "dc icl div2 6.5-8.0V",
		.reg	= ZIN_ICL_MID_HV_REG,
		.min_u	= 0,
		.max_u	= 3000000,
		.step_u	= 25000,
	},
	.dc_icl_div2_hv		= {
		.name	= "dc icl div2 >8.0V",
		.reg	= ZIN_ICL_HV_REG,
		.min_u	= 0,
		.max_u	= 3000000,
		.step_u	= 25000,
	},
	.jeita_cc_comp		= {
		.name	= "jeita fcc reduction",
		.reg	= JEITA_CCCOMP_CFG_REG,
		.min_u	= 0,
		.max_u	= 1575000,
		.step_u	= 25000,
	},
	.step_soc_threshold[0]		= {
		.name	= "step charge soc threshold 1",
		.reg	= STEP_CHG_SOC_OR_BATT_V_TH1_REG,
		.min_u	= 0,
		.max_u	= 100,
		.step_u	= 1,
	},
	.step_soc_threshold[1]		= {
		.name	= "step charge soc threshold 2",
		.reg	= STEP_CHG_SOC_OR_BATT_V_TH2_REG,
		.min_u	= 0,
		.max_u	= 100,
		.step_u	= 1,
	},
	.step_soc_threshold[2]         = {
		.name	= "step charge soc threshold 3",
		.reg	= STEP_CHG_SOC_OR_BATT_V_TH3_REG,
		.min_u	= 0,
		.max_u	= 100,
		.step_u	= 1,
	},
	.step_soc_threshold[3]         = {
		.name	= "step charge soc threshold 4",
		.reg	= STEP_CHG_SOC_OR_BATT_V_TH4_REG,
		.min_u	= 0,
		.max_u	= 100,
		.step_u	= 1,
	},
	.step_soc			= {
		.name	= "step charge soc",
		.reg	= STEP_CHG_SOC_VBATT_V_REG,
		.min_u	= 0,
		.max_u	= 100,
		.step_u	= 1,
		.set_proc	= smblib_mapping_soc_from_field_value,
	},
	.step_cc_delta[0]	= {
		.name	= "step charge current delta 1",
		.reg	= STEP_CHG_CURRENT_DELTA1_REG,
		.min_u	= 100000,
		.max_u	= 3200000,
		.step_u	= 100000,
		.get_proc	= smblib_mapping_cc_delta_to_field_value,
		.set_proc	= smblib_mapping_cc_delta_from_field_value,
	},
	.step_cc_delta[1]	= {
		.name	= "step charge current delta 2",
		.reg	= STEP_CHG_CURRENT_DELTA2_REG,
		.min_u	= 100000,
		.max_u	= 3200000,
		.step_u	= 100000,
		.get_proc	= smblib_mapping_cc_delta_to_field_value,
		.set_proc	= smblib_mapping_cc_delta_from_field_value,
	},
	.step_cc_delta[2]	= {
		.name	= "step charge current delta 3",
		.reg	= STEP_CHG_CURRENT_DELTA3_REG,
		.min_u	= 100000,
		.max_u	= 3200000,
		.step_u	= 100000,
		.get_proc	= smblib_mapping_cc_delta_to_field_value,
		.set_proc	= smblib_mapping_cc_delta_from_field_value,
	},
	.step_cc_delta[3]	= {
		.name	= "step charge current delta 4",
		.reg	= STEP_CHG_CURRENT_DELTA4_REG,
		.min_u	= 100000,
		.max_u	= 3200000,
		.step_u	= 100000,
		.get_proc	= smblib_mapping_cc_delta_to_field_value,
		.set_proc	= smblib_mapping_cc_delta_from_field_value,
	},
	.step_cc_delta[4]	= {
		.name	= "step charge current delta 5",
		.reg	= STEP_CHG_CURRENT_DELTA5_REG,
		.min_u	= 100000,
		.max_u	= 3200000,
		.step_u	= 100000,
		.get_proc	= smblib_mapping_cc_delta_to_field_value,
		.set_proc	= smblib_mapping_cc_delta_from_field_value,
	},
	.freq_buck		= {
		.name	= "buck switching frequency",
		.reg	= CFG_BUCKBOOST_FREQ_SELECT_BUCK_REG,
		.min_u	= 600,
		.max_u	= 2000,
		.step_u	= 200,
	},
	.freq_boost		= {
		.name	= "boost switching frequency",
		.reg	= CFG_BUCKBOOST_FREQ_SELECT_BOOST_REG,
		.min_u	= 600,
		.max_u	= 2000,
		.step_u	= 200,
	},
};

static struct smb_params pm660_params = {
	.freq_buck		= {
		.name	= "buck switching frequency",
		.reg	= FREQ_CLK_DIV_REG,
		.min_u	= 600,
		.max_u	= 1600,
		.set_proc = smblib_set_chg_freq,
	},
	.freq_boost		= {
		.name	= "boost switching frequency",
		.reg	= FREQ_CLK_DIV_REG,
		.min_u	= 600,
		.max_u	= 1600,
		.set_proc = smblib_set_chg_freq,
	},
};


#ifndef OPLUS_FEATURE_CHG_BASIC
#define STEP_CHARGING_MAX_STEPS	5
struct smb_dt_props {
	int	fcc_ua;
	int	usb_icl_ua;
#ifndef OPLUS_FEATURE_CHG_BASIC
	int	otg_cl_ua;
#endif
	int	dc_icl_ua;
	int	boost_threshold_ua;
	int	fv_uv;
	int	wipower_max_uw;
	u32	step_soc_threshold[STEP_CHARGING_MAX_STEPS - 1];
	s32	step_cc_delta[STEP_CHARGING_MAX_STEPS];
	struct	device_node *revid_dev_node;
	int	float_option;
	int	chg_inhibit_thr_mv;
	bool	no_battery;
	bool	hvdcp_disable;
	bool	auto_recharge_soc;
};

struct smb2 {
	struct smb_charger	chg;
	struct dentry		*dfs_root;
	struct smb_dt_props	dt;
	bool			bad_part;
};
#endif /* OPLUS_FEATURE_CHG_BASIC */


static bool is_secure(struct smb_charger *chg, int addr)
{
	if (addr == SHIP_MODE_REG || addr == FREQ_CLK_DIV_REG)
		return true;
	/* assume everything above 0xA0 is secure */
	return (bool)((addr & 0xFF) >= 0xA0);
}

int smblib_read(struct smb_charger *chg, u16 addr, u8 *val)
{
	unsigned int temp;
	int rc = 0;

	rc = regmap_read(chg->regmap, addr, &temp);
	if (rc >= 0)
		*val = (u8)temp;

	return rc;
}

int smblib_multibyte_read(struct smb_charger *chg, u16 addr, u8 *val,
				int count)
{
	return regmap_bulk_read(chg->regmap, addr, val, count);
}

int smblib_masked_write(struct smb_charger *chg, u16 addr, u8 mask, u8 val)
{
	int rc = 0;

	mutex_lock(&chg->write_lock);
	if (is_secure(chg, addr)) {
		rc = regmap_write(chg->regmap, (addr & 0xFF00) | 0xD0, 0xA5);
		if (rc < 0)
			goto unlock;
	}

	rc = regmap_update_bits(chg->regmap, addr, mask, val);

unlock:
	mutex_unlock(&chg->write_lock);
	return rc;
}

int smblib_write(struct smb_charger *chg, u16 addr, u8 val)
{
	int rc = 0;

	mutex_lock(&chg->write_lock);

	if (is_secure(chg, addr)) {
		rc = regmap_write(chg->regmap, (addr & ~(0xFF)) | 0xD0, 0xA5);
		if (rc < 0)
			goto unlock;
	}

	rc = regmap_write(chg->regmap, addr, val);

unlock:
	mutex_unlock(&chg->write_lock);
	return rc;
}

static int smblib_get_step_cc_delta(struct smb_charger *chg, int *cc_delta_ua)
{
	int rc, step_state;
	u8 stat;

	if (!chg->step_chg_enabled) {
		*cc_delta_ua = 0;
		return 0;
	}

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
			rc);
		return rc;
	}

	step_state = (stat & STEP_CHARGING_STATUS_MASK) >>
				STEP_CHARGING_STATUS_SHIFT;
	rc = smblib_get_charge_param(chg, &chg->param.step_cc_delta[step_state],
				     cc_delta_ua);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get step cc delta rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static int smblib_get_jeita_cc_delta(struct smb_charger *chg, int *cc_delta_ua)
{
	int rc, cc_minus_ua;
	u8 stat;

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_2_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_2 rc=%d\n",
			rc);
		return rc;
	}

	if (!(stat & BAT_TEMP_STATUS_SOFT_LIMIT_MASK)) {
		*cc_delta_ua = 0;
		return 0;
	}

	rc = smblib_get_charge_param(chg, &chg->param.jeita_cc_comp,
				     &cc_minus_ua);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get jeita cc minus rc=%d\n", rc);
		return rc;
	}

	*cc_delta_ua = -cc_minus_ua;
	return 0;
}

int smblib_icl_override(struct smb_charger *chg, bool override)
{
	int rc;
	bool override_status;
	u8 stat;
	u16 reg;

	switch (chg->smb_version) {
	case PMI8998_SUBTYPE:
		reg = APSD_RESULT_STATUS_REG;
		break;
	case PM660_SUBTYPE:
		reg = AICL_STATUS_REG;
		break;
	default:
		smblib_dbg(chg, PR_MISC, "Unknown chip version=%x\n",
				chg->smb_version);
		return -EINVAL;
	}

	rc = smblib_read(chg, reg, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read reg=%x rc=%d\n", reg, rc);
		return rc;
	}
	override_status = (bool)(stat & ICL_OVERRIDE_LATCH_BIT);

	if (override != override_status) {
		rc = smblib_masked_write(chg, CMD_APSD_REG,
				ICL_OVERRIDE_BIT, ICL_OVERRIDE_BIT);
		if (rc < 0) {
			smblib_err(chg, "Couldn't override ICL rc=%d\n", rc);
			return rc;
		}
	}
	return 0;
}

/********************
 * REGISTER GETTERS *
 ********************/

int smblib_get_charge_param(struct smb_charger *chg,
			    struct smb_chg_param *param, int *val_u)
{
	int rc = 0;
	u8 val_raw;

	rc = smblib_read(chg, param->reg, &val_raw);
	if (rc < 0) {
		smblib_err(chg, "%s: Couldn't read from 0x%04x rc=%d\n",
			param->name, param->reg, rc);
		return rc;
	}

	if (param->get_proc)
		*val_u = param->get_proc(param, val_raw);
	else
		*val_u = val_raw * param->step_u + param->min_u;
	smblib_dbg(chg, PR_REGISTER, "%s = %d (0x%02x)\n",
		   param->name, *val_u, val_raw);

	return rc;
}

int smblib_get_usb_suspend(struct smb_charger *chg, int *suspend)
{
	int rc = 0;
	u8 temp;

	rc = smblib_read(chg, USBIN_CMD_IL_REG, &temp);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read USBIN_CMD_IL rc=%d\n", rc);
		return rc;
	}
	*suspend = temp & USBIN_SUSPEND_BIT;

	return rc;
}

struct apsd_result {
	const char * const name;
	const u8 bit;
	const enum power_supply_type pst;
};

enum {
	UNKNOWN,
	SDP,
	CDP,
	DCP,
	OCP,
	FLOAT,
	HVDCP2,
	HVDCP3,
	MAX_TYPES
};

static const struct apsd_result const smblib_apsd_results[] = {
	[UNKNOWN] = {
		.name	= "UNKNOWN",
		.bit	= 0,
		.pst	= POWER_SUPPLY_TYPE_UNKNOWN
	},
	[SDP] = {
		.name	= "SDP",
		.bit	= SDP_CHARGER_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB
	},
	[CDP] = {
		.name	= "CDP",
		.bit	= CDP_CHARGER_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB_CDP
	},
	[DCP] = {
		.name	= "DCP",
		.bit	= DCP_CHARGER_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB_DCP
	},
	[OCP] = {
		.name	= "OCP",
		.bit	= OCP_CHARGER_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB_DCP
	},
	[FLOAT] = {
		.name	= "FLOAT",
		.bit	= FLOAT_CHARGER_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB_DCP
	},
	[HVDCP2] = {
		.name	= "HVDCP2",
		.bit	= DCP_CHARGER_BIT | QC_2P0_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB_HVDCP
	},
	[HVDCP3] = {
		.name	= "HVDCP3",
		.bit	= DCP_CHARGER_BIT | QC_3P0_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB_HVDCP_3,
	},
};

static const struct apsd_result *smblib_get_apsd_result(struct smb_charger *chg)
{
	int rc, i;
	u8 apsd_stat, stat;
	const struct apsd_result *result = &smblib_apsd_results[UNKNOWN];

	rc = smblib_read(chg, APSD_STATUS_REG, &apsd_stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read APSD_STATUS rc=%d\n", rc);
		return result;
	}
	smblib_dbg(chg, PR_REGISTER, "APSD_STATUS = 0x%02x\n", apsd_stat);

	if (!(apsd_stat & APSD_DTC_STATUS_DONE_BIT))
		return result;

	rc = smblib_read(chg, APSD_RESULT_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read APSD_RESULT_STATUS rc=%d\n",
			rc);
		return result;
	}
	stat &= APSD_RESULT_STATUS_MASK;

	for (i = 0; i < ARRAY_SIZE(smblib_apsd_results); i++) {
		if (smblib_apsd_results[i].bit == stat)
			result = &smblib_apsd_results[i];
	}

	if (apsd_stat & QC_CHARGER_BIT) {
		/* since its a qc_charger, either return HVDCP3 or HVDCP2 */
#ifndef OPLUS_FEATURE_CHG_BASIC
		if (result != &smblib_apsd_results[HVDCP3])
			result = &smblib_apsd_results[HVDCP2];
#else
		if (result != &smblib_apsd_results[HVDCP3] && result->bit == (DCP_CHARGER_BIT | QC_2P0_BIT))
			result = &smblib_apsd_results[HVDCP2];
#endif
	}

	return result;
}

/********************
 * REGISTER SETTERS *
 ********************/

static int chg_freq_list[] = {
	9600, 9600, 6400, 4800, 3800, 3200, 2700, 2400, 2100, 1900, 1700,
	1600, 1500, 1400, 1300, 1200,
};

int smblib_set_chg_freq(struct smb_chg_param *param,
				int val_u, u8 *val_raw)
{
	u8 i;

	if (val_u > param->max_u || val_u < param->min_u)
		return -EINVAL;

	/* Charger FSW is the configured freqency / 2 */
	val_u *= 2;
	for (i = 0; i < ARRAY_SIZE(chg_freq_list); i++) {
		if (chg_freq_list[i] == val_u)
			break;
	}
	if (i == ARRAY_SIZE(chg_freq_list)) {
		pr_err("Invalid frequency %d Hz\n", val_u / 2);
		return -EINVAL;
	}

	*val_raw = i;

	return 0;
}

static int smblib_set_opt_freq_buck(struct smb_charger *chg, int fsw_khz)
{
	union power_supply_propval pval = {0, };
	int rc = 0;

	rc = smblib_set_charge_param(chg, &chg->param.freq_buck, fsw_khz);
	if (rc < 0)
		dev_err(chg->dev, "Error in setting freq_buck rc=%d\n", rc);

	if (chg->mode == PARALLEL_MASTER && chg->pl.psy) {
		pval.intval = fsw_khz;
		/*
		 * Some parallel charging implementations may not have
		 * PROP_BUCK_FREQ property - they could be running
		 * with a fixed frequency
		 */
		power_supply_set_property(chg->pl.psy,
				POWER_SUPPLY_PROP_BUCK_FREQ, &pval);
	}

	return rc;
}

int smblib_set_charge_param(struct smb_charger *chg,
			    struct smb_chg_param *param, int val_u)
{
	int rc = 0;
	u8 val_raw;

	if (param->set_proc) {
		rc = param->set_proc(param, val_u, &val_raw);
		if (rc < 0)
			return -EINVAL;
	} else {
		if (val_u > param->max_u || val_u < param->min_u) {
			smblib_err(chg, "%s: %d is out of range [%d, %d]\n",
				param->name, val_u, param->min_u, param->max_u);
			return -EINVAL;
		}

		val_raw = (val_u - param->min_u) / param->step_u;
	}

	rc = smblib_write(chg, param->reg, val_raw);
	if (rc < 0) {
		smblib_err(chg, "%s: Couldn't write 0x%02x to 0x%04x rc=%d\n",
			param->name, val_raw, param->reg, rc);
		return rc;
	}

	smblib_dbg(chg, PR_REGISTER, "%s = %d (0x%02x)\n",
		   param->name, val_u, val_raw);

	return rc;
}

static int step_charge_soc_update(struct smb_charger *chg, int capacity)
{
	int rc = 0;

	rc = smblib_set_charge_param(chg, &chg->param.step_soc, capacity);
	if (rc < 0) {
		smblib_err(chg, "Error in updating soc, rc=%d\n", rc);
		return rc;
	}

	rc = smblib_write(chg, STEP_CHG_SOC_VBATT_V_UPDATE_REG,
			STEP_CHG_SOC_VBATT_V_UPDATE_BIT);
	if (rc < 0) {
		smblib_err(chg,
			"Couldn't set STEP_CHG_SOC_VBATT_V_UPDATE_REG rc=%d\n",
			rc);
		return rc;
	}

	return rc;
}

int smblib_set_usb_suspend(struct smb_charger *chg, bool suspend)
{
	int rc = 0;

	rc = smblib_masked_write(chg, USBIN_CMD_IL_REG, USBIN_SUSPEND_BIT,
				 suspend ? USBIN_SUSPEND_BIT : 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't write %s to USBIN_SUSPEND_BIT rc=%d\n",
			suspend ? "suspend" : "resume", rc);

	return rc;
}

int smblib_set_dc_suspend(struct smb_charger *chg, bool suspend)
{
	int rc = 0;

	rc = smblib_masked_write(chg, DCIN_CMD_IL_REG, DCIN_SUSPEND_BIT,
				 suspend ? DCIN_SUSPEND_BIT : 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't write %s to DCIN_SUSPEND_BIT rc=%d\n",
			suspend ? "suspend" : "resume", rc);

	return rc;
}

static int smblib_set_adapter_allowance(struct smb_charger *chg,
					u8 allowed_voltage)
{
	int rc = 0;

	switch (allowed_voltage) {
	case USBIN_ADAPTER_ALLOW_12V:
	case USBIN_ADAPTER_ALLOW_5V_OR_12V:
	case USBIN_ADAPTER_ALLOW_9V_TO_12V:
	case USBIN_ADAPTER_ALLOW_5V_OR_9V_TO_12V:
	case USBIN_ADAPTER_ALLOW_5V_TO_12V:
		/* PM660 only support max. 9V */
		if (chg->smb_version == PM660_SUBTYPE) {
			smblib_dbg(chg, PR_MISC, "voltage not supported=%d\n",
					allowed_voltage);
			allowed_voltage = USBIN_ADAPTER_ALLOW_5V_OR_9V;
		}
		break;
	}

	rc = smblib_write(chg, USBIN_ADAPTER_ALLOW_CFG_REG, allowed_voltage);
	if (rc < 0) {
		smblib_err(chg, "Couldn't write 0x%02x to USBIN_ADAPTER_ALLOW_CFG rc=%d\n",
			allowed_voltage, rc);
		return rc;
	}

	return rc;
}

#define MICRO_5V	5000000
#define MICRO_9V	9000000
#define MICRO_12V	12000000
static int smblib_set_usb_pd_allowed_voltage(struct smb_charger *chg,
					int min_allowed_uv, int max_allowed_uv)
{
	int rc;
	u8 allowed_voltage;

	if (min_allowed_uv == MICRO_5V && max_allowed_uv == MICRO_5V) {
		allowed_voltage = USBIN_ADAPTER_ALLOW_5V;
		smblib_set_opt_freq_buck(chg, chg->chg_freq.freq_5V);
	} else if (min_allowed_uv == MICRO_9V && max_allowed_uv == MICRO_9V) {
		allowed_voltage = USBIN_ADAPTER_ALLOW_9V;
		smblib_set_opt_freq_buck(chg, chg->chg_freq.freq_9V);
	} else if (min_allowed_uv == MICRO_12V && max_allowed_uv == MICRO_12V) {
		allowed_voltage = USBIN_ADAPTER_ALLOW_12V;
		smblib_set_opt_freq_buck(chg, chg->chg_freq.freq_12V);
	} else if (min_allowed_uv < MICRO_9V && max_allowed_uv <= MICRO_9V) {
		allowed_voltage = USBIN_ADAPTER_ALLOW_5V_TO_9V;
	} else if (min_allowed_uv < MICRO_9V && max_allowed_uv <= MICRO_12V) {
		allowed_voltage = USBIN_ADAPTER_ALLOW_5V_TO_12V;
	} else if (min_allowed_uv < MICRO_12V && max_allowed_uv <= MICRO_12V) {
		allowed_voltage = USBIN_ADAPTER_ALLOW_9V_TO_12V;
	} else {
		smblib_err(chg, "invalid allowed voltage [%d, %d]\n",
			min_allowed_uv, max_allowed_uv);
		return -EINVAL;
	}

	rc = smblib_set_adapter_allowance(chg, allowed_voltage);
	if (rc < 0) {
		smblib_err(chg, "Couldn't configure adapter allowance rc=%d\n",
				rc);
		return rc;
	}

	return rc;
}

/********************
 * HELPER FUNCTIONS *
 ********************/

static void smblib_rerun_apsd(struct smb_charger *chg)
{
	int rc;

	smblib_dbg(chg, PR_MISC, "re-running APSD\n");
	if (chg->wa_flags & QC_AUTH_INTERRUPT_WA_BIT) {
		rc = smblib_masked_write(chg,
				USBIN_SOURCE_CHANGE_INTRPT_ENB_REG,
				AUTH_IRQ_EN_CFG_BIT, AUTH_IRQ_EN_CFG_BIT);
		if (rc < 0)
			smblib_err(chg, "Couldn't enable HVDCP auth IRQ rc=%d\n",
									rc);
	}

	rc = smblib_masked_write(chg, CMD_APSD_REG,
				APSD_RERUN_BIT, APSD_RERUN_BIT);
	if (rc < 0)
		smblib_err(chg, "Couldn't re-run APSD rc=%d\n", rc);
}

static int try_rerun_apsd_for_hvdcp(struct smb_charger *chg)
{
	const struct apsd_result *apsd_result;

	/*
	 * PD_INACTIVE_VOTER on hvdcp_disable_votable indicates whether
	 * apsd rerun was tried earlier
	 */
	if (get_client_vote(chg->hvdcp_disable_votable_indirect,
						PD_INACTIVE_VOTER)) {
		vote(chg->hvdcp_disable_votable_indirect,
				PD_INACTIVE_VOTER, false, 0);
		/* ensure hvdcp is enabled */
		if (!get_effective_result(
				chg->hvdcp_disable_votable_indirect)) {
			apsd_result = smblib_get_apsd_result(chg);
			if (apsd_result->bit & (QC_2P0_BIT | QC_3P0_BIT)) {
				smblib_rerun_apsd(chg);
			}
		}
	}
	return 0;
}

static const struct apsd_result *smblib_update_usb_type(struct smb_charger *chg)
{
	const struct apsd_result *apsd_result = smblib_get_apsd_result(chg);

	/* if PD is active, APSD is disabled so won't have a valid result */
	if (chg->pd_active)
		chg->usb_psy_desc.type = POWER_SUPPLY_TYPE_USB_PD;
	else
		chg->usb_psy_desc.type = apsd_result->pst;

	smblib_dbg(chg, PR_MISC, "APSD=%s PD=%d\n",
					apsd_result->name, chg->pd_active);
	return apsd_result;
}

static int smblib_notifier_call(struct notifier_block *nb,
		unsigned long ev, void *v)
{
	struct power_supply *psy = v;
	struct smb_charger *chg = container_of(nb, struct smb_charger, nb);

	if (!strcmp(psy->desc->name, "bms")) {
		if (!chg->bms_psy)
			chg->bms_psy = psy;
		if (ev == PSY_EVENT_PROP_CHANGED)
			schedule_work(&chg->bms_update_work);
	}

	if (!chg->pl.psy && !strcmp(psy->desc->name, "parallel"))
		chg->pl.psy = psy;

	return NOTIFY_OK;
}

static int smblib_register_notifier(struct smb_charger *chg)
{
	int rc;

	chg->nb.notifier_call = smblib_notifier_call;
	rc = power_supply_reg_notifier(&chg->nb);
	if (rc < 0) {
		smblib_err(chg, "Couldn't register psy notifier rc = %d\n", rc);
		return rc;
	}

	return 0;
}

int smblib_mapping_soc_from_field_value(struct smb_chg_param *param,
					     int val_u, u8 *val_raw)
{
	if (val_u > param->max_u || val_u < param->min_u)
		return -EINVAL;

	*val_raw = val_u << 1;

	return 0;
}

int smblib_mapping_cc_delta_to_field_value(struct smb_chg_param *param,
					   u8 val_raw)
{
	int val_u  = val_raw * param->step_u + param->min_u;

	if (val_u > param->max_u)
		val_u -= param->max_u * 2;

	return val_u;
}

int smblib_mapping_cc_delta_from_field_value(struct smb_chg_param *param,
					     int val_u, u8 *val_raw)
{
	if (val_u > param->max_u || val_u < param->min_u - param->max_u)
		return -EINVAL;

	val_u += param->max_u * 2 - param->min_u;
	val_u %= param->max_u * 2;
	*val_raw = val_u / param->step_u;

	return 0;
}

static void smblib_uusb_removal(struct smb_charger *chg)
{
	int rc;

	/* reset both usbin current and voltage votes */
	vote(chg->pl_enable_votable_indirect, USBIN_I_VOTER, false, 0);
	vote(chg->pl_enable_votable_indirect, USBIN_V_VOTER, false, 0);
	vote(chg->pl_disable_votable, PL_DELAY_HVDCP_VOTER, true, 0);

	cancel_delayed_work_sync(&chg->hvdcp_detect_work);

	if (chg->wa_flags & QC_AUTH_INTERRUPT_WA_BIT) {
		/* re-enable AUTH_IRQ_EN_CFG_BIT */
		rc = smblib_masked_write(chg,
				USBIN_SOURCE_CHANGE_INTRPT_ENB_REG,
				AUTH_IRQ_EN_CFG_BIT, AUTH_IRQ_EN_CFG_BIT);
		if (rc < 0)
			smblib_err(chg,
				"Couldn't enable QC auth setting rc=%d\n", rc);
	}

	/* reconfigure allowed voltage for HVDCP */
	rc = smblib_set_adapter_allowance(chg,
			USBIN_ADAPTER_ALLOW_5V_OR_9V_TO_12V);
	if (rc < 0)
		smblib_err(chg, "Couldn't set USBIN_ADAPTER_ALLOW_5V_OR_9V_TO_12V rc=%d\n",
			rc);

	chg->voltage_min_uv = MICRO_5V;
	chg->voltage_max_uv = MICRO_5V;
	chg->usb_icl_delta_ua = 0;
	chg->pulse_cnt = 0;
	chg->uusb_apsd_rerun_done = false;

	/* clear USB ICL vote for USB_PSY_VOTER */
	rc = vote(chg->usb_icl_votable, USB_PSY_VOTER, false, 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't un-vote for USB ICL rc=%d\n", rc);

	/* clear USB ICL vote for DCP_VOTER */
	rc = vote(chg->usb_icl_votable, DCP_VOTER, false, 0);
	if (rc < 0)
		smblib_err(chg,
			"Couldn't un-vote DCP from USB ICL rc=%d\n", rc);

	/* clear USB ICL vote for PL_USBIN_USBIN_VOTER */
	rc = vote(chg->usb_icl_votable, PL_USBIN_USBIN_VOTER, false, 0);
	if (rc < 0)
		smblib_err(chg,
			"Couldn't un-vote PL_USBIN_USBIN from USB ICL rc=%d\n",
			rc);
}

static bool smblib_sysok_reason_usbin(struct smb_charger *chg)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, SYSOK_REASON_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get SYSOK_REASON_STATUS rc=%d\n", rc);
		/* assuming 'not usbin' in case of read failure */
		return false;
	}

	return stat & SYSOK_REASON_USBIN_BIT;
}

void smblib_suspend_on_debug_battery(struct smb_charger *chg)
{
	int rc;
	union power_supply_propval val;

	if (!chg->suspend_input_on_debug_batt)
		return;

	rc = power_supply_get_property(chg->bms_psy,
			POWER_SUPPLY_PROP_DEBUG_BATTERY, &val);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get debug battery prop rc=%d\n", rc);
		return;
	}

	vote(chg->usb_icl_votable, DEBUG_BOARD_VOTER, val.intval, 0);
	vote(chg->dc_suspend_votable, DEBUG_BOARD_VOTER, val.intval, 0);
	if (val.intval)
		pr_info("Input suspended: Fake battery\n");
}

int smblib_rerun_apsd_if_required(struct smb_charger *chg)
{
	const struct apsd_result *apsd_result;
	union power_supply_propval val;
	int rc;

	rc = smblib_get_prop_usb_present(chg, &val);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get usb present rc = %d\n", rc);
		return rc;
	}

	if (!val.intval)
		return 0;

	apsd_result = smblib_get_apsd_result(chg);

#ifndef OPLUS_FEATURE_CHG_BASIC
	if ((apsd_result->pst != POWER_SUPPLY_TYPE_UNKNOWN)
		&& (apsd_result->pst != POWER_SUPPLY_TYPE_USB))
		/* if type is not usb or unknown no need to rerun apsd */
		return 0;
#else
	if ((apsd_result->pst != POWER_SUPPLY_TYPE_UNKNOWN)
		&& (apsd_result->pst != POWER_SUPPLY_TYPE_USB)
		&& (apsd_result->pst != POWER_SUPPLY_TYPE_USB_CDP))
		/* if type is not usb or unknown no need to rerun apsd */
		return 0;
#endif
	/* fetch the DPDM regulator */
	if (!chg->dpdm_reg && of_get_property(chg->dev->of_node,
						"dpdm-supply", NULL)) {
		chg->dpdm_reg = devm_regulator_get(chg->dev, "dpdm");
		if (IS_ERR(chg->dpdm_reg)) {
			smblib_err(chg, "Couldn't get dpdm regulator rc=%ld\n",
				PTR_ERR(chg->dpdm_reg));
			chg->dpdm_reg = NULL;
		}
	}

	if (chg->dpdm_reg && !regulator_is_enabled(chg->dpdm_reg)) {
		smblib_dbg(chg, PR_MISC, "enabling DPDM regulator\n");
		rc = regulator_enable(chg->dpdm_reg);
		if (rc < 0)
			smblib_err(chg, "Couldn't enable dpdm regulator rc=%d\n",
				rc);
	}

	chg->uusb_apsd_rerun_done = true;
	smblib_rerun_apsd(chg);

	return 0;
}

static int smblib_get_pulse_cnt(struct smb_charger *chg, int *count)
{
	int rc;
	u8 val[2];

	switch (chg->smb_version) {
	case PMI8998_SUBTYPE:
		rc = smblib_read(chg, QC_PULSE_COUNT_STATUS_REG, val);
		if (rc) {
			pr_err("failed to read QC_PULSE_COUNT_STATUS_REG rc=%d\n",
					rc);
			return rc;
		}
		*count = val[0] & QC_PULSE_COUNT_MASK;
		break;
	case PM660_SUBTYPE:
		rc = smblib_multibyte_read(chg,
				QC_PULSE_COUNT_STATUS_1_REG, val, 2);
		if (rc) {
			pr_err("failed to read QC_PULSE_COUNT_STATUS_1_REG rc=%d\n",
					rc);
			return rc;
		}
		*count = (val[1] << 8) | val[0];
		break;
	default:
		smblib_dbg(chg, PR_PARALLEL, "unknown SMB chip %d\n",
				chg->smb_version);
		return -EINVAL;
	}

	return 0;
}

/*********************
 * VOTABLE CALLBACKS *
 *********************/

static int smblib_dc_suspend_vote_callback(struct votable *votable, void *data,
			int suspend, const char *client)
{
	struct smb_charger *chg = data;

	/* resume input if suspend is invalid */
	if (suspend < 0)
		suspend = 0;

	return smblib_set_dc_suspend(chg, (bool)suspend);
}

#define USBIN_25MA	25000
#define USBIN_100MA	100000
#define USBIN_150MA	150000
#define USBIN_500MA	500000
#define USBIN_900MA	900000


static int set_sdp_current(struct smb_charger *chg, int icl_ua)
{
	int rc;
	u8 icl_options;

	/* power source is SDP */
	switch (icl_ua) {
	case USBIN_100MA:
		/* USB 2.0 100mA */
		icl_options = 0;
		break;
	case USBIN_150MA:
		/* USB 3.0 150mA */
		icl_options = CFG_USB3P0_SEL_BIT;
		break;
	case USBIN_500MA:
		/* USB 2.0 500mA */
		icl_options = USB51_MODE_BIT;
		break;
	case USBIN_900MA:
		/* USB 3.0 900mA */
		icl_options = CFG_USB3P0_SEL_BIT | USB51_MODE_BIT;
		break;
	default:
		smblib_err(chg, "ICL %duA isn't supported for SDP\n", icl_ua);
		return -EINVAL;
	}
#ifdef OPLUS_FEATURE_CHG_BASIC
	if (icl_ua <= USBIN_150MA)
		icl_options = 0;
	else
		icl_options = USB51_MODE_BIT;
#endif

	rc = smblib_masked_write(chg, USBIN_ICL_OPTIONS_REG,
		CFG_USB3P0_SEL_BIT | USB51_MODE_BIT, icl_options);
	if (rc < 0) {
		smblib_err(chg, "Couldn't set ICL options rc=%d\n", rc);
		return rc;
	}

	return rc;
}

static int smblib_usb_icl_vote_callback(struct votable *votable, void *data,
			int icl_ua, const char *client)
{
	struct smb_charger *chg = data;
	int rc = 0;
	bool override;
	union power_supply_propval pval;

#ifdef OPLUS_FEATURE_CHG_BASIC
	int boot_mode = get_boot_mode();
	if (boot_mode == MSM_BOOT_MODE__RF || boot_mode == MSM_BOOT_MODE__WLAN) {
		icl_ua = 0;
	}
#endif

	/* suspend and return if 25mA or less is requested */
	if (client && (icl_ua < USBIN_25MA))
		return smblib_set_usb_suspend(chg, true);

	disable_irq_nosync(chg->irq_info[USBIN_ICL_CHANGE_IRQ].irq);
	if (!client)
		goto override_suspend_config;

	rc = smblib_get_prop_typec_mode(chg, &pval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get typeC mode rc = %d\n", rc);
		goto enable_icl_changed_interrupt;
	}

	/* configure current */
	if (pval.intval == POWER_SUPPLY_TYPEC_SOURCE_DEFAULT
		&& (chg->usb_psy_desc.type == POWER_SUPPLY_TYPE_USB)) {
		rc = set_sdp_current(chg, icl_ua);
		if (rc < 0) {
			smblib_err(chg, "Couldn't set SDP ICL rc=%d\n", rc);
			goto enable_icl_changed_interrupt;
		}
	} else {
		rc = smblib_set_charge_param(chg, &chg->param.usb_icl,
				icl_ua - chg->icl_reduction_ua);
		if (rc < 0) {
			smblib_err(chg, "Couldn't set HC ICL rc=%d\n", rc);
			goto enable_icl_changed_interrupt;
		}
	}

override_suspend_config:
	/* determine if override needs to be enforced */
	override = true;
	if (client == NULL) {
		/* remove override if no voters - hw defaults is desired */
		override = false;
	} else if (pval.intval == POWER_SUPPLY_TYPEC_SOURCE_DEFAULT) {
		if (chg->usb_psy_desc.type == POWER_SUPPLY_TYPE_USB)
			/* For std cable with type = SDP never override */
			override = false;
		else if (chg->usb_psy_desc.type == POWER_SUPPLY_TYPE_USB_CDP
			&& icl_ua - chg->icl_reduction_ua == 1500000)
			/*
			 * For std cable with type = CDP override only if
			 * current is not 1500mA
			 */
			override = false;
	}

	/* enforce override */
	rc = smblib_masked_write(chg, USBIN_ICL_OPTIONS_REG,
		USBIN_MODE_CHG_BIT, override ? USBIN_MODE_CHG_BIT : 0);

	rc = smblib_icl_override(chg, override);
	if (rc < 0) {
		smblib_err(chg, "Couldn't set ICL override rc=%d\n", rc);
		goto enable_icl_changed_interrupt;
	}

	/* unsuspend after configuring current and override */
	rc = smblib_set_usb_suspend(chg, false);
	if (rc < 0) {
		smblib_err(chg, "Couldn't resume input rc=%d\n", rc);
		goto enable_icl_changed_interrupt;
	}

enable_icl_changed_interrupt:
	enable_irq(chg->irq_info[USBIN_ICL_CHANGE_IRQ].irq);
	return rc;
}

static int smblib_dc_icl_vote_callback(struct votable *votable, void *data,
			int icl_ua, const char *client)
{
	struct smb_charger *chg = data;
	int rc = 0;
	bool suspend;

	if (icl_ua < 0) {
		smblib_dbg(chg, PR_MISC, "No Voter hence suspending\n");
		icl_ua = 0;
	}

	suspend = (icl_ua < USBIN_25MA);
	if (suspend)
		goto suspend;

	rc = smblib_set_charge_param(chg, &chg->param.dc_icl, icl_ua);
	if (rc < 0) {
		smblib_err(chg, "Couldn't set DC input current limit rc=%d\n",
			rc);
		return rc;
	}

suspend:
	rc = vote(chg->dc_suspend_votable, USER_VOTER, suspend, 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't vote to %s DC rc=%d\n",
			suspend ? "suspend" : "resume", rc);
		return rc;
	}
	return rc;
}

static int smblib_pd_disallowed_votable_indirect_callback(
	struct votable *votable, void *data, int disallowed, const char *client)
{
	struct smb_charger *chg = data;
	int rc;

	rc = vote(chg->pd_allowed_votable, PD_DISALLOWED_INDIRECT_VOTER,
		!disallowed, 0);

	return rc;
}

static int smblib_awake_vote_callback(struct votable *votable, void *data,
			int awake, const char *client)
{
	struct smb_charger *chg = data;

	if (awake)
		pm_stay_awake(chg->dev);
	else
		pm_relax(chg->dev);

	return 0;
}

static int smblib_chg_disable_vote_callback(struct votable *votable, void *data,
			int chg_disable, const char *client)
{
	struct smb_charger *chg = data;
	int rc;

	rc = smblib_masked_write(chg, CHARGING_ENABLE_CMD_REG,
				 CHARGING_ENABLE_CMD_BIT,
				 chg_disable ? 0 : CHARGING_ENABLE_CMD_BIT);
	if (rc < 0) {
		smblib_err(chg, "Couldn't %s charging rc=%d\n",
			chg_disable ? "disable" : "enable", rc);
		return rc;
	}

	return 0;
}

static int smblib_pl_enable_indirect_vote_callback(struct votable *votable,
			void *data, int chg_enable, const char *client)
{
	struct smb_charger *chg = data;

	vote(chg->pl_disable_votable, PL_INDIRECT_VOTER, !chg_enable, 0);

	return 0;
}

static int smblib_hvdcp_enable_vote_callback(struct votable *votable,
			void *data,
			int hvdcp_enable, const char *client)
{
	struct smb_charger *chg = data;
	int rc;
	u8 val = HVDCP_AUTH_ALG_EN_CFG_BIT | HVDCP_EN_BIT;

#ifdef OPLUS_FEATURE_CHG_BASIC
	hvdcp_enable = 0;
#endif

	/* vote to enable/disable HW autonomous INOV */
	vote(chg->hvdcp_hw_inov_dis_votable, client, !hvdcp_enable, 0);

	/*
	 * Disable the autonomous bit and auth bit for disabling hvdcp.
	 * This ensures only qc 2.0 detection runs but no vbus
	 * negotiation happens.
	 */
#ifdef OPLUS_FEATURE_CHG_BASIC
	val = 0;
#else
	if (!hvdcp_enable)
		val = HVDCP_EN_BIT;
#endif

	rc = smblib_masked_write(chg, USBIN_OPTIONS_1_CFG_REG,
				 HVDCP_EN_BIT | HVDCP_AUTH_ALG_EN_CFG_BIT,
				 val);
	if (rc < 0) {
		smblib_err(chg, "Couldn't %s hvdcp rc=%d\n",
			hvdcp_enable ? "enable" : "disable", rc);
		return rc;
	}

	return 0;
}

static int smblib_hvdcp_disable_indirect_vote_callback(struct votable *votable,
			void *data, int hvdcp_disable, const char *client)
{
	struct smb_charger *chg = data;

	vote(chg->hvdcp_enable_votable, HVDCP_INDIRECT_VOTER,
			!hvdcp_disable, 0);

	return 0;
}

static int smblib_apsd_disable_vote_callback(struct votable *votable,
			void *data,
			int apsd_disable, const char *client)
{
	struct smb_charger *chg = data;
	int rc;

	if (apsd_disable) {
		/* Don't run APSD on CC debounce when APSD is disabled */
		rc = smblib_masked_write(chg, TYPE_C_CFG_REG,
							APSD_START_ON_CC_BIT,
							0);
		if (rc < 0) {
			smblib_err(chg, "Couldn't disable APSD_START_ON_CC rc=%d\n",
									rc);
			return rc;
		}

		rc = smblib_masked_write(chg, USBIN_OPTIONS_1_CFG_REG,
							AUTO_SRC_DETECT_BIT,
							0);
		if (rc < 0) {
			smblib_err(chg, "Couldn't disable APSD rc=%d\n", rc);
			return rc;
		}
	} else {
		rc = smblib_masked_write(chg, USBIN_OPTIONS_1_CFG_REG,
							AUTO_SRC_DETECT_BIT,
							AUTO_SRC_DETECT_BIT);
		if (rc < 0) {
			smblib_err(chg, "Couldn't enable APSD rc=%d\n", rc);
			return rc;
		}

		rc = smblib_masked_write(chg, TYPE_C_CFG_REG,
							APSD_START_ON_CC_BIT,
							APSD_START_ON_CC_BIT);
		if (rc < 0) {
			smblib_err(chg, "Couldn't enable APSD_START_ON_CC rc=%d\n",
									rc);
			return rc;
		}
	}

	return 0;
}

static int smblib_hvdcp_hw_inov_dis_vote_callback(struct votable *votable,
				void *data, int disable, const char *client)
{
	struct smb_charger *chg = data;
	int rc;

	if (disable) {
		/*
		 * the pulse count register get zeroed when autonomous mode is
		 * disabled. Track that in variables before disabling
		 */
		rc = smblib_get_pulse_cnt(chg, &chg->pulse_cnt);
		if (rc < 0) {
			pr_err("failed to read QC_PULSE_COUNT_STATUS_REG rc=%d\n",
					rc);
			return rc;
		}
	}

	rc = smblib_masked_write(chg, USBIN_OPTIONS_1_CFG_REG,
			HVDCP_AUTONOMOUS_MODE_EN_CFG_BIT,
			disable ? 0 : HVDCP_AUTONOMOUS_MODE_EN_CFG_BIT);
	if (rc < 0) {
		smblib_err(chg, "Couldn't %s hvdcp rc=%d\n",
				disable ? "disable" : "enable", rc);
		return rc;
	}

	return rc;
}

/*******************
 * VCONN REGULATOR *
 * *****************/

#define MAX_OTG_SS_TRIES 2
static int _smblib_vconn_regulator_enable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	u8 otg_stat, stat4;
	int rc = 0, i;

	if (!chg->external_vconn) {
		/*
		 * Hardware based OTG soft start should complete within 1ms, so
		 * wait for 2ms in the worst case.
		 */
		for (i = 0; i < MAX_OTG_SS_TRIES; ++i) {
			usleep_range(1000, 1100);
			rc = smblib_read(chg, OTG_STATUS_REG, &otg_stat);
			if (rc < 0) {
				smblib_err(chg, "Couldn't read OTG status rc=%d\n",
									rc);
				return rc;
			}

			if (otg_stat & BOOST_SOFTSTART_DONE_BIT)
				break;
		}

		if (!(otg_stat & BOOST_SOFTSTART_DONE_BIT)) {
			smblib_err(chg, "Couldn't enable VCONN; OTG soft start failed\n");
			return -EAGAIN;
		}
	}

	/*
	 * VCONN_EN_ORIENTATION is overloaded with overriding the CC pin used
	 * for Vconn, and it should be set with reverse polarity of CC_OUT.
	 */
	rc = smblib_read(chg, TYPE_C_STATUS_4_REG, &stat4);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_4 rc=%d\n", rc);
		return rc;
	}

	smblib_dbg(chg, PR_OTG, "enabling VCONN\n");
	stat4 = stat4 & CC_ORIENTATION_BIT ? 0 : VCONN_EN_ORIENTATION_BIT;
	rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 VCONN_EN_VALUE_BIT | VCONN_EN_ORIENTATION_BIT,
				 VCONN_EN_VALUE_BIT | stat4);
	if (rc < 0) {
		smblib_err(chg, "Couldn't enable vconn setting rc=%d\n", rc);
		return rc;
	}

	return rc;
}

int smblib_vconn_regulator_enable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc = 0;

	mutex_lock(&chg->otg_oc_lock);
	if (chg->vconn_en)
		goto unlock;

	rc = _smblib_vconn_regulator_enable(rdev);
	if (rc >= 0)
		chg->vconn_en = true;

unlock:
	mutex_unlock(&chg->otg_oc_lock);
	return rc;
}

static int _smblib_vconn_regulator_disable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc = 0;

	smblib_dbg(chg, PR_OTG, "disabling VCONN\n");
	rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 VCONN_EN_VALUE_BIT, 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't disable vconn regulator rc=%d\n", rc);

	return rc;
}

int smblib_vconn_regulator_disable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc = 0;

	mutex_lock(&chg->otg_oc_lock);
	if (!chg->vconn_en)
		goto unlock;

	rc = _smblib_vconn_regulator_disable(rdev);
	if (rc >= 0)
		chg->vconn_en = false;

unlock:
	mutex_unlock(&chg->otg_oc_lock);
	return rc;
}

int smblib_vconn_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int ret;

	mutex_lock(&chg->otg_oc_lock);
	ret = chg->vconn_en;
	mutex_unlock(&chg->otg_oc_lock);
	return ret;
}

/*****************
 * OTG REGULATOR *
 *****************/
#ifdef OPLUS_FEATURE_CHG_BASIC
#define MAX_RETRY		30
#else
#define MAX_RETRY		15
#endif

#define MIN_DELAY_US		2000
#define MAX_DELAY_US		9000
#ifdef OPLUS_FEATURE_CHG_BASIC
static int otg_current[] = {250000, 500000, 1000000};
static int smblib_enable_otg_wa(struct smb_charger *chg)
{
	u8 stat;
	int rc, i, retry_count = 0, min_delay = MIN_DELAY_US;

	for (i = 0; i < ARRAY_SIZE(otg_current); i++) {
		smblib_dbg(chg, PR_OTG, "enabling OTG with %duA\n",
						otg_current[i]);
		rc = smblib_set_charge_param(chg, &chg->param.otg_cl,
						otg_current[i]);
		if (rc < 0) {
			smblib_err(chg,	"Couldn't set otg limit rc=%d\n", rc);
			return rc;
		}

		rc = smblib_write(chg, CMD_OTG_REG, OTG_EN_BIT);
		if (rc < 0) {
			smblib_err(chg, "Couldn't enable OTG rc=%d\n", rc);
			return rc;
		}

		retry_count = 0;
		do {
			usleep_range(min_delay, min_delay + 100);
			rc = smblib_read(chg, OTG_STATUS_REG, &stat);
			if (rc < 0) {
				smblib_err(chg, "Couldn't read OTG status rc=%d\n",
							rc);
				goto out;
			}

			if (stat & BOOST_SOFTSTART_DONE_BIT) {
				rc = smblib_set_charge_param(chg,
					&chg->param.otg_cl, chg->otg_cl_ua);
				if (rc < 0) {
					smblib_err(chg, "Couldn't set otg limit rc=%d\n",
							rc);
					goto out;
				}
				break;
			}

			/* increase the delay for following iterations */
			if (retry_count > 5)
				min_delay = MAX_DELAY_US;
		} while (retry_count++ < MAX_RETRY);

		if (retry_count >= MAX_RETRY) {
			smblib_dbg(chg, PR_OTG, "OTG enable failed with %duA\n",
								otg_current[i]);
			rc = smblib_write(chg, CMD_OTG_REG, 0);
			if (rc < 0) {
				smblib_err(chg, "disable OTG rc=%d\n", rc);
				goto out;
			}
		} else {
			smblib_dbg(chg, PR_OTG, "OTG enabled\n");
			return 0;
		}
	}

	if (i == ARRAY_SIZE(otg_current)) {
		rc = -EINVAL;
		goto out;
	}

	return 0;
out:
	smblib_write(chg, CMD_OTG_REG, 0);
	return rc;
}

static int _smblib_vbus_regulator_enable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc;

#ifdef OPLUS_FEATURE_CHG_BASIC
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (chip && oplus_usbid_check_is_gpio(chip) && chip->chg_ops->check_chrdet_status())
		return -EINVAL;
#endif

	smblib_dbg(chg, PR_OTG, "halt 1 in 8 mode\n");
	rc = smblib_masked_write(chg, OTG_ENG_OTG_CFG_REG,
				 ENG_BUCKBOOST_HALT1_8_MODE_BIT,
				 ENG_BUCKBOOST_HALT1_8_MODE_BIT);
	if (rc < 0) {
		smblib_err(chg, "Couldn't set OTG_ENG_OTG_CFG_REG rc=%d\n",
			rc);
		return rc;
	}

	smblib_dbg(chg, PR_OTG, "enabling OTG\n");

	if (chg->wa_flags & OTG_WA) {
		rc = smblib_enable_otg_wa(chg);
		if (rc < 0)
			smblib_err(chg, "Couldn't enable OTG rc=%d\n", rc);
	} else {
		rc = smblib_write(chg, CMD_OTG_REG, OTG_EN_BIT);
		if (rc < 0)
			smblib_err(chg, "Couldn't enable OTG rc=%d\n", rc);
	}

	return rc;
}
#else
static int _smblib_vbus_regulator_enable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc, retry_count = 0, min_delay = MIN_DELAY_US;
	u8 stat;

	smblib_dbg(chg, PR_OTG, "halt 1 in 8 mode\n");
	rc = smblib_masked_write(chg, OTG_ENG_OTG_CFG_REG,
				 ENG_BUCKBOOST_HALT1_8_MODE_BIT,
				 ENG_BUCKBOOST_HALT1_8_MODE_BIT);
	if (rc < 0) {
		smblib_err(chg, "Couldn't set OTG_ENG_OTG_CFG_REG rc=%d\n",
			rc);
		return rc;
	}

	smblib_dbg(chg, PR_OTG, "enabling OTG\n");
	rc = smblib_write(chg, CMD_OTG_REG, OTG_EN_BIT);
	if (rc < 0) {
		smblib_err(chg, "Couldn't enable OTG regulator rc=%d\n", rc);
		return rc;
	}

	if (chg->wa_flags & OTG_WA) {
		/* check for softstart */
		do {
			usleep_range(min_delay, min_delay + 100);
			rc = smblib_read(chg, OTG_STATUS_REG, &stat);
			if (rc < 0) {
				smblib_err(chg,
					"Couldn't read OTG status rc=%d\n",
					rc);
				goto out;
			}

			if (stat & BOOST_SOFTSTART_DONE_BIT) {
				rc = smblib_set_charge_param(chg,
					&chg->param.otg_cl, chg->otg_cl_ua);
				if (rc < 0)
					smblib_err(chg,
						"Couldn't set otg limit\n");
				break;
			}

			/* increase the delay for following iterations */
			if (retry_count > 5)
				min_delay = MAX_DELAY_US;
		} while (retry_count++ < MAX_RETRY);

		if (retry_count >= MAX_RETRY) {
			smblib_dbg(chg, PR_OTG, "Boost Softstart not done\n");
			goto out;
		}
	}

	return 0;
out:
	/* disable OTG if softstart failed */
	smblib_write(chg, CMD_OTG_REG, 0);
	return rc;
}
#endif /* OPLUS_FEATURE_CHG_BASIC */

int smblib_vbus_regulator_enable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc = 0;

	mutex_lock(&chg->otg_oc_lock);
	if (chg->otg_en)
		goto unlock;

	rc = _smblib_vbus_regulator_enable(rdev);
	if (rc >= 0)
		chg->otg_en = true;

unlock:
	mutex_unlock(&chg->otg_oc_lock);
	return rc;
}

static int _smblib_vbus_regulator_disable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc;

	if (!chg->external_vconn && chg->vconn_en) {
		smblib_dbg(chg, PR_OTG, "Killing VCONN before disabling OTG\n");
		rc = _smblib_vconn_regulator_disable(rdev);
		if (rc < 0)
			smblib_err(chg, "Couldn't disable VCONN rc=%d\n", rc);
	}

	if (chg->wa_flags & OTG_WA) {
		/* set OTG current limit to minimum value */
		rc = smblib_set_charge_param(chg, &chg->param.otg_cl,
						chg->param.otg_cl.min_u);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't set otg current limit rc=%d\n", rc);
			return rc;
		}
	}

	smblib_dbg(chg, PR_OTG, "disabling OTG\n");
	rc = smblib_write(chg, CMD_OTG_REG, 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't disable OTG regulator rc=%d\n", rc);
		return rc;
	}

	smblib_dbg(chg, PR_OTG, "start 1 in 8 mode\n");
#ifndef OPLUS_FEATURE_CHG_BASIC
	rc = smblib_write(chg, CMD_OTG_REG, 0);
#endif
	rc = smblib_masked_write(chg, OTG_ENG_OTG_CFG_REG,
				 ENG_BUCKBOOST_HALT1_8_MODE_BIT, 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't set OTG_ENG_OTG_CFG_REG rc=%d\n", rc);
		return rc;
	}

	return 0;
}

int smblib_vbus_regulator_disable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc = 0;

	mutex_lock(&chg->otg_oc_lock);
	if (!chg->otg_en)
		goto unlock;

	rc = _smblib_vbus_regulator_disable(rdev);
	if (rc >= 0)
		chg->otg_en = false;

unlock:
	mutex_unlock(&chg->otg_oc_lock);
	return rc;
}

int smblib_vbus_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int ret;

	mutex_lock(&chg->otg_oc_lock);
	ret = chg->otg_en;
	mutex_unlock(&chg->otg_oc_lock);
	return ret;
}

/********************
 * BATT PSY GETTERS *
 ********************/

int smblib_get_prop_input_suspend(struct smb_charger *chg,
				  union power_supply_propval *val)
{
	val->intval
		= (get_client_vote(chg->usb_icl_votable, USER_VOTER) == 0)
		 && get_client_vote(chg->dc_suspend_votable, USER_VOTER);
	return 0;
}

int smblib_get_prop_batt_present(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, BATIF_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATIF_INT_RT_STS rc=%d\n", rc);
		return rc;
	}

	val->intval = !(stat & (BAT_THERM_OR_ID_MISSING_RT_STS_BIT
					| BAT_TERMINAL_MISSING_RT_STS_BIT));

	return rc;
}

int smblib_get_prop_batt_capacity(struct smb_charger *chg,
				  union power_supply_propval *val)
{
	int rc = -EINVAL;

	if (chg->fake_capacity >= 0) {
		val->intval = chg->fake_capacity;
		return 0;
	}

	if (chg->bms_psy)
		rc = power_supply_get_property(chg->bms_psy,
				POWER_SUPPLY_PROP_CAPACITY, val);
	return rc;
}

int smblib_get_prop_batt_status(struct smb_charger *chg,
				union power_supply_propval *val)
{
	union power_supply_propval pval = {0, };
	bool usb_online, dc_online;
	u8 stat;
	int rc;

	rc = smblib_get_prop_usb_online(chg, &pval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get usb online property rc=%d\n",
			rc);
		return rc;
	}
	usb_online = (bool)pval.intval;

	rc = smblib_get_prop_dc_online(chg, &pval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get dc online property rc=%d\n",
			rc);
		return rc;
	}
	dc_online = (bool)pval.intval;

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
			rc);
		return rc;
	}
	stat = stat & BATTERY_CHARGER_STATUS_MASK;

	if (!usb_online && !dc_online) {
		switch (stat) {
		case TERMINATE_CHARGE:
		case INHIBIT_CHARGE:
			val->intval = POWER_SUPPLY_STATUS_FULL;
			break;
		default:
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			break;
		}
		return rc;
	}

	switch (stat) {
	case TRICKLE_CHARGE:
	case PRE_CHARGE:
	case FAST_CHARGE:
	case FULLON_CHARGE:
	case TAPER_CHARGE:
		val->intval = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case TERMINATE_CHARGE:
	case INHIBIT_CHARGE:
		val->intval = POWER_SUPPLY_STATUS_FULL;
		break;
	case DISABLE_CHARGE:
		val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	default:
		val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		break;
	}

	return 0;
}

int smblib_get_prop_batt_charge_type(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
			rc);
		return rc;
	}

	switch (stat & BATTERY_CHARGER_STATUS_MASK) {
	case TRICKLE_CHARGE:
	case PRE_CHARGE:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		break;
	case FAST_CHARGE:
	case FULLON_CHARGE:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
		break;
	case TAPER_CHARGE:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_TAPER;
		break;
	default:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
	}

	return rc;
}

int smblib_get_prop_batt_health(struct smb_charger *chg,
				union power_supply_propval *val)
{
	union power_supply_propval pval;
	int rc;
	u8 stat;

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_2_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_2 rc=%d\n",
			rc);
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "BATTERY_CHARGER_STATUS_2 = 0x%02x\n",
		   stat);

	if (stat & CHARGER_ERROR_STATUS_BAT_OV_BIT) {
		rc = smblib_get_prop_batt_voltage_now(chg, &pval);
		if (!rc) {
			/*
			 * If Vbatt is within 40mV above Vfloat, then don't
			 * treat it as overvoltage.
			 */
			if (pval.intval >=
				get_effective_result(chg->fv_votable) + 40000) {
				val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
				smblib_err(chg, "battery over-voltage\n");
				goto done;
			}
		}
	}

	if (stat & BAT_TEMP_STATUS_TOO_COLD_BIT)
		val->intval = POWER_SUPPLY_HEALTH_COLD;
	else if (stat & BAT_TEMP_STATUS_TOO_HOT_BIT)
		val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (stat & BAT_TEMP_STATUS_COLD_SOFT_LIMIT_BIT)
		val->intval = POWER_SUPPLY_HEALTH_COOL;
	else if (stat & BAT_TEMP_STATUS_HOT_SOFT_LIMIT_BIT)
		val->intval = POWER_SUPPLY_HEALTH_WARM;
	else
		val->intval = POWER_SUPPLY_HEALTH_GOOD;

done:
	return rc;
}

int smblib_get_prop_system_temp_level(struct smb_charger *chg,
				union power_supply_propval *val)
{
	val->intval = chg->system_temp_level;
	return 0;
}

int smblib_get_prop_input_current_limited(struct smb_charger *chg,
				union power_supply_propval *val)
{
	u8 stat;
	int rc;

	rc = smblib_read(chg, AICL_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read AICL_STATUS rc=%d\n", rc);
		return rc;
	}
	val->intval = (stat & SOFT_ILIMIT_BIT) || chg->is_hdc;
	return 0;
}

int smblib_get_prop_batt_voltage_now(struct smb_charger *chg,
				     union power_supply_propval *val)
{
	int rc;

	if (!chg->bms_psy)
		return -EINVAL;

	rc = power_supply_get_property(chg->bms_psy,
				       POWER_SUPPLY_PROP_VOLTAGE_NOW, val);
	return rc;
}

int smblib_get_prop_batt_current_now(struct smb_charger *chg,
				     union power_supply_propval *val)
{
	int rc;

	if (!chg->bms_psy)
		return -EINVAL;

	rc = power_supply_get_property(chg->bms_psy,
				       POWER_SUPPLY_PROP_CURRENT_NOW, val);
	return rc;
}

int smblib_get_prop_batt_temp(struct smb_charger *chg,
			      union power_supply_propval *val)
{
	int rc;

	if (!chg->bms_psy)
		return -EINVAL;

	rc = power_supply_get_property(chg->bms_psy,
				       POWER_SUPPLY_PROP_TEMP, val);
	return rc;
}

int smblib_get_prop_step_chg_step(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc;
	u8 stat;

	if (!chg->step_chg_enabled) {
		val->intval = -1;
		return 0;
	}

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
			rc);
		return rc;
	}

	val->intval = (stat & STEP_CHARGING_STATUS_MASK) >>
				STEP_CHARGING_STATUS_SHIFT;

	return rc;
}

int smblib_get_prop_batt_charge_done(struct smb_charger *chg,
					union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
			rc);
		return rc;
	}

	stat = stat & BATTERY_CHARGER_STATUS_MASK;
	val->intval = (stat == TERMINATE_CHARGE);
	return 0;
}

/***********************
 * BATTERY PSY SETTERS *
 ***********************/

int smblib_set_prop_input_suspend(struct smb_charger *chg,
				  const union power_supply_propval *val)
{
	int rc;

	/* vote 0mA when suspended */
	rc = vote(chg->usb_icl_votable, USER_VOTER, (bool)val->intval, 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't vote to %s USB rc=%d\n",
			(bool)val->intval ? "suspend" : "resume", rc);
		return rc;
	}

	rc = vote(chg->dc_suspend_votable, USER_VOTER, (bool)val->intval, 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't vote to %s DC rc=%d\n",
			(bool)val->intval ? "suspend" : "resume", rc);
		return rc;
	}

	power_supply_changed(chg->batt_psy);
	return rc;
}

int smblib_set_prop_batt_capacity(struct smb_charger *chg,
				  const union power_supply_propval *val)
{
	chg->fake_capacity = val->intval;

	power_supply_changed(chg->batt_psy);

	return 0;
}

int smblib_set_prop_system_temp_level(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	if (val->intval < 0)
		return -EINVAL;

	if (chg->thermal_levels <= 0)
		return -EINVAL;

	if (val->intval > chg->thermal_levels)
		return -EINVAL;

	chg->system_temp_level = val->intval;
	if (chg->system_temp_level == chg->thermal_levels)
		return vote(chg->chg_disable_votable,
			THERMAL_DAEMON_VOTER, true, 0);

	vote(chg->chg_disable_votable, THERMAL_DAEMON_VOTER, false, 0);
	if (chg->system_temp_level == 0)
		return vote(chg->fcc_votable, THERMAL_DAEMON_VOTER, false, 0);

	vote(chg->fcc_votable, THERMAL_DAEMON_VOTER, true,
			chg->thermal_mitigation[chg->system_temp_level]);
	return 0;
}

int smblib_rerun_aicl(struct smb_charger *chg)
{
	int rc, settled_icl_ua;
	u8 stat;

	rc = smblib_read(chg, POWER_PATH_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read POWER_PATH_STATUS rc=%d\n",
								rc);
		return rc;
	}

	/* USB is suspended so skip re-running AICL */
	if (stat & USBIN_SUSPEND_STS_BIT)
		return rc;

	smblib_dbg(chg, PR_MISC, "re-running AICL\n");
	switch (chg->smb_version) {
	case PMI8998_SUBTYPE:
		rc = smblib_get_charge_param(chg, &chg->param.icl_stat,
							&settled_icl_ua);
		if (rc < 0) {
			smblib_err(chg, "Couldn't get settled ICL rc=%d\n", rc);
			return rc;
		}

		vote(chg->usb_icl_votable, AICL_RERUN_VOTER, true,
				max(settled_icl_ua - chg->param.usb_icl.step_u,
				chg->param.usb_icl.step_u));
		vote(chg->usb_icl_votable, AICL_RERUN_VOTER, false, 0);
		break;
	case PM660_SUBTYPE:
		/*
		 * Use restart_AICL instead of trigger_AICL as it runs the
		 * complete AICL instead of starting from the last settled
		 * value.
		 */
		rc = smblib_masked_write(chg, CMD_HVDCP_2_REG,
					RESTART_AICL_BIT, RESTART_AICL_BIT);
		if (rc < 0)
			smblib_err(chg, "Couldn't write to CMD_HVDCP_2_REG rc=%d\n",
									rc);
		break;
	default:
		smblib_dbg(chg, PR_PARALLEL, "unknown SMB chip %d\n",
				chg->smb_version);
		return -EINVAL;
	}

	return 0;
}

static int smblib_dp_pulse(struct smb_charger *chg)
{
	int rc;

	/* QC 3.0 increment */
	rc = smblib_masked_write(chg, CMD_HVDCP_2_REG, SINGLE_INCREMENT_BIT,
			SINGLE_INCREMENT_BIT);
	if (rc < 0)
		smblib_err(chg, "Couldn't write to CMD_HVDCP_2_REG rc=%d\n",
				rc);

	return rc;
}

static int smblib_dm_pulse(struct smb_charger *chg)
{
	int rc;

	/* QC 3.0 decrement */
	rc = smblib_masked_write(chg, CMD_HVDCP_2_REG, SINGLE_DECREMENT_BIT,
			SINGLE_DECREMENT_BIT);
	if (rc < 0)
		smblib_err(chg, "Couldn't write to CMD_HVDCP_2_REG rc=%d\n",
				rc);

	return rc;
}

int smblib_dp_dm(struct smb_charger *chg, int val)
{
	int target_icl_ua, rc = 0;

	switch (val) {
	case POWER_SUPPLY_DP_DM_DP_PULSE:
		rc = smblib_dp_pulse(chg);
		if (!rc)
			chg->pulse_cnt++;
		smblib_dbg(chg, PR_PARALLEL, "DP_DM_DP_PULSE rc=%d cnt=%d\n",
				rc, chg->pulse_cnt);
		break;
	case POWER_SUPPLY_DP_DM_DM_PULSE:
		rc = smblib_dm_pulse(chg);
		if (!rc && chg->pulse_cnt)
			chg->pulse_cnt--;
		smblib_dbg(chg, PR_PARALLEL, "DP_DM_DM_PULSE rc=%d cnt=%d\n",
				rc, chg->pulse_cnt);
		break;
	case POWER_SUPPLY_DP_DM_ICL_DOWN:
		chg->usb_icl_delta_ua -= 100000;
		target_icl_ua = get_effective_result(chg->usb_icl_votable);
		vote(chg->usb_icl_votable, SW_QC3_VOTER, true,
				target_icl_ua + chg->usb_icl_delta_ua);
		break;
	case POWER_SUPPLY_DP_DM_ICL_UP:
	default:
		break;
	}

	return rc;
}

/*******************
 * DC PSY GETTERS *
 *******************/

int smblib_get_prop_dc_present(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, DCIN_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read DCIN_RT_STS rc=%d\n", rc);
		return rc;
	}

	val->intval = (bool)(stat & DCIN_PLUGIN_RT_STS_BIT);
	return 0;
}

int smblib_get_prop_dc_online(struct smb_charger *chg,
			       union power_supply_propval *val)
{
	int rc = 0;
	u8 stat;

	if (get_client_vote(chg->dc_suspend_votable, USER_VOTER)) {
		val->intval = false;
		return rc;
	}

	rc = smblib_read(chg, POWER_PATH_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read POWER_PATH_STATUS rc=%d\n",
			rc);
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "POWER_PATH_STATUS = 0x%02x\n",
		   stat);

	val->intval = (stat & USE_DCIN_BIT) &&
		      (stat & VALID_INPUT_POWER_SOURCE_STS_BIT);

	return rc;
}

int smblib_get_prop_dc_current_max(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	val->intval = get_effective_result_locked(chg->dc_icl_votable);
	return 0;
}

/*******************
 * DC PSY SETTERS *
 * *****************/

int smblib_set_prop_dc_current_max(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	int rc;

	rc = vote(chg->dc_icl_votable, USER_VOTER, true, val->intval);
	return rc;
}

/*******************
 * USB PSY GETTERS *
 *******************/

int smblib_get_prop_usb_present(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, USBIN_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read USBIN_RT_STS rc=%d\n", rc);
		return rc;
	}

	val->intval = (bool)(stat & USBIN_PLUGIN_RT_STS_BIT);
	return 0;
}

#ifdef OPLUS_FEATURE_CHG_BASIC
/*  when set USBIN_SUSPEND_BIT, use present instead of online */
static bool usb_online_status = false;
#endif
int smblib_get_prop_usb_online(struct smb_charger *chg,
			       union power_supply_propval *val)
{
	int rc = 0;
	u8 stat;

#ifdef OPLUS_FEATURE_CHG_BASIC
	if (usb_online_status == true) {
		rc = smblib_read(chg, USBIN_BASE + INT_RT_STS_OFFSET, &stat);
		if (rc < 0) {
			smblib_err(chg, "usb_online_status: Couldn't read USBIN_RT_STS rc=%d\n", rc);
			return rc;
		}

		val->intval = (bool)(stat & USBIN_PLUGIN_RT_STS_BIT);
		return rc;
	}
#endif /*OPLUS_FEATURE_CHG_BASIC*/

	if (get_client_vote(chg->usb_icl_votable, USER_VOTER) == 0) {
		val->intval = false;
#ifdef OPLUS_FEATURE_CHG_BASIC
		printk(KERN_ERR "smblib_get_prop_usb_online false\n");
#endif
		return rc;
	}

	rc = smblib_read(chg, POWER_PATH_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read POWER_PATH_STATUS rc=%d\n",
			rc);
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "POWER_PATH_STATUS = 0x%02x\n",
		   stat);

	val->intval = (stat & USE_USBIN_BIT) &&
		      (stat & VALID_INPUT_POWER_SOURCE_STS_BIT);
	return rc;
}

int smblib_get_prop_usb_voltage_now(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	int rc = 0;

	rc = smblib_get_prop_usb_present(chg, val);
	if (rc < 0 || !val->intval)
		return rc;

	if (!chg->iio.usbin_v_chan ||
		PTR_ERR(chg->iio.usbin_v_chan) == -EPROBE_DEFER)
		chg->iio.usbin_v_chan = iio_channel_get(chg->dev, "usbin_v");

	if (IS_ERR(chg->iio.usbin_v_chan))
		return PTR_ERR(chg->iio.usbin_v_chan);

	return iio_read_channel_processed(chg->iio.usbin_v_chan, &val->intval);
}

int smblib_get_prop_pd_current_max(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	val->intval = get_client_vote_locked(chg->usb_icl_votable, PD_VOTER);
	return 0;
}

int smblib_get_prop_usb_current_max(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	val->intval = get_client_vote_locked(chg->usb_icl_votable,
			USB_PSY_VOTER);
	return 0;
}

int smblib_get_prop_usb_current_now(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	int rc = 0;

	rc = smblib_get_prop_usb_present(chg, val);
	if (rc < 0 || !val->intval)
		return rc;

	if (!chg->iio.usbin_i_chan ||
		PTR_ERR(chg->iio.usbin_i_chan) == -EPROBE_DEFER)
		chg->iio.usbin_i_chan = iio_channel_get(chg->dev, "usbin_i");

	if (IS_ERR(chg->iio.usbin_i_chan))
		return PTR_ERR(chg->iio.usbin_i_chan);

	return iio_read_channel_processed(chg->iio.usbin_i_chan, &val->intval);
}

int smblib_get_prop_charger_temp(struct smb_charger *chg,
				 union power_supply_propval *val)
{
	int rc;

	if (!chg->iio.temp_chan ||
		PTR_ERR(chg->iio.temp_chan) == -EPROBE_DEFER)
		chg->iio.temp_chan = iio_channel_get(chg->dev, "charger_temp");

	if (IS_ERR(chg->iio.temp_chan))
		return PTR_ERR(chg->iio.temp_chan);

	rc = iio_read_channel_processed(chg->iio.temp_chan, &val->intval);
	val->intval /= 100;
	return rc;
}

int smblib_get_prop_charger_temp_max(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	int rc;

	if (!chg->iio.temp_max_chan ||
		PTR_ERR(chg->iio.temp_max_chan) == -EPROBE_DEFER)
		chg->iio.temp_max_chan = iio_channel_get(chg->dev,
							 "charger_temp_max");
	if (IS_ERR(chg->iio.temp_max_chan))
		return PTR_ERR(chg->iio.temp_max_chan);

	rc = iio_read_channel_processed(chg->iio.temp_max_chan, &val->intval);
	val->intval /= 100;
	return rc;
}

int smblib_get_prop_typec_cc_orientation(struct smb_charger *chg,
					 union power_supply_propval *val)
{
	int rc = 0;
	u8 stat;

	rc = smblib_read(chg, TYPE_C_STATUS_4_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_4 rc=%d\n", rc);
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_STATUS_4 = 0x%02x\n",
		   stat);

	if (stat & CC_ATTACHED_BIT)
		val->intval = (bool)(stat & CC_ORIENTATION_BIT) + 1;
	else
		val->intval = 0;

	return rc;
}

static const char * const smblib_typec_mode_name[] = {
	[POWER_SUPPLY_TYPEC_NONE]		  = "NONE",
	[POWER_SUPPLY_TYPEC_SOURCE_DEFAULT]	  = "SOURCE_DEFAULT",
	[POWER_SUPPLY_TYPEC_SOURCE_MEDIUM]	  = "SOURCE_MEDIUM",
	[POWER_SUPPLY_TYPEC_SOURCE_HIGH]	  = "SOURCE_HIGH",
	[POWER_SUPPLY_TYPEC_NON_COMPLIANT]	  = "NON_COMPLIANT",
	[POWER_SUPPLY_TYPEC_SINK]		  = "SINK",
	[POWER_SUPPLY_TYPEC_SINK_POWERED_CABLE]   = "SINK_POWERED_CABLE",
	[POWER_SUPPLY_TYPEC_SINK_DEBUG_ACCESSORY] = "SINK_DEBUG_ACCESSORY",
	[POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER]   = "SINK_AUDIO_ADAPTER",
	[POWER_SUPPLY_TYPEC_POWERED_CABLE_ONLY]   = "POWERED_CABLE_ONLY",
};

static int smblib_get_prop_ufp_mode(struct smb_charger *chg)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, TYPE_C_STATUS_1_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_1 rc=%d\n", rc);
		return POWER_SUPPLY_TYPEC_NONE;
	}
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_STATUS_1 = 0x%02x\n", stat);

	switch (stat) {
	case 0:
		return POWER_SUPPLY_TYPEC_NONE;
	case UFP_TYPEC_RDSTD_BIT:
		return POWER_SUPPLY_TYPEC_SOURCE_DEFAULT;
	case UFP_TYPEC_RD1P5_BIT:
		return POWER_SUPPLY_TYPEC_SOURCE_MEDIUM;
	case UFP_TYPEC_RD3P0_BIT:
		return POWER_SUPPLY_TYPEC_SOURCE_HIGH;
	default:
		break;
	}

	return POWER_SUPPLY_TYPEC_NON_COMPLIANT;
}

static int smblib_get_prop_dfp_mode(struct smb_charger *chg)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, TYPE_C_STATUS_2_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_2 rc=%d\n", rc);
		return POWER_SUPPLY_TYPEC_NONE;
	}
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_STATUS_2 = 0x%02x\n", stat);

	switch (stat & DFP_TYPEC_MASK) {
	case DFP_RA_RA_BIT:
		return POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER;
	case DFP_RD_RD_BIT:
		return POWER_SUPPLY_TYPEC_SINK_DEBUG_ACCESSORY;
	case DFP_RD_RA_VCONN_BIT:
		return POWER_SUPPLY_TYPEC_SINK_POWERED_CABLE;
	case DFP_RD_OPEN_BIT:
		return POWER_SUPPLY_TYPEC_SINK;
	case DFP_RA_OPEN_BIT:
		return POWER_SUPPLY_TYPEC_POWERED_CABLE_ONLY;
	default:
		break;
	}

	return POWER_SUPPLY_TYPEC_NONE;
}

int smblib_get_prop_typec_mode(struct smb_charger *chg,
			       union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, TYPE_C_STATUS_4_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_4 rc=%d\n", rc);
		val->intval = POWER_SUPPLY_TYPEC_NONE;
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_STATUS_4 = 0x%02x\n", stat);

	if (!(stat & TYPEC_DEBOUNCE_DONE_STATUS_BIT)) {
		val->intval = POWER_SUPPLY_TYPEC_NONE;
		return rc;
	}

	if (stat & UFP_DFP_MODE_STATUS_BIT)
		val->intval = smblib_get_prop_dfp_mode(chg);
	else
		val->intval = smblib_get_prop_ufp_mode(chg);

	return rc;
}

int smblib_get_prop_typec_power_role(struct smb_charger *chg,
				     union power_supply_propval *val)
{
	int rc = 0;
	u8 ctrl;

	rc = smblib_read(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG, &ctrl);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_INTRPT_ENB_SOFTWARE_CTRL rc=%d\n",
			rc);
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_INTRPT_ENB_SOFTWARE_CTRL = 0x%02x\n",
		   ctrl);

	if (ctrl & TYPEC_DISABLE_CMD_BIT) {
		val->intval = POWER_SUPPLY_TYPEC_PR_NONE;
		return rc;
	}

	switch (ctrl & (DFP_EN_CMD_BIT | UFP_EN_CMD_BIT)) {
	case 0:
		val->intval = POWER_SUPPLY_TYPEC_PR_DUAL;
		break;
	case DFP_EN_CMD_BIT:
		val->intval = POWER_SUPPLY_TYPEC_PR_SOURCE;
		break;
	case UFP_EN_CMD_BIT:
		val->intval = POWER_SUPPLY_TYPEC_PR_SINK;
		break;
	default:
		val->intval = POWER_SUPPLY_TYPEC_PR_NONE;
		smblib_err(chg, "unsupported power role 0x%02lx\n",
			ctrl & (DFP_EN_CMD_BIT | UFP_EN_CMD_BIT));
		return -EINVAL;
	}

	return rc;
}

int smblib_get_prop_pd_allowed(struct smb_charger *chg,
			       union power_supply_propval *val)
{
	val->intval = get_effective_result(chg->pd_allowed_votable);
	return 0;
}

int smblib_get_prop_input_current_settled(struct smb_charger *chg,
					  union power_supply_propval *val)
{
	return smblib_get_charge_param(chg, &chg->param.icl_stat, &val->intval);
}

#define HVDCP3_STEP_UV	200000
int smblib_get_prop_input_voltage_settled(struct smb_charger *chg,
						union power_supply_propval *val)
{
	const struct apsd_result *apsd_result = smblib_get_apsd_result(chg);
	int rc, pulses;
	u8 stat;

	val->intval = MICRO_5V;
	if (apsd_result == NULL) {
		smblib_err(chg, "APSD result is NULL\n");
		return 0;
	}

	switch (apsd_result->pst) {
	case POWER_SUPPLY_TYPE_USB_HVDCP_3:
		rc = smblib_read(chg, QC_PULSE_COUNT_STATUS_REG, &stat);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't read QC_PULSE_COUNT rc=%d\n", rc);
			return 0;
		}
		pulses = (stat & QC_PULSE_COUNT_MASK);
		val->intval = MICRO_5V + HVDCP3_STEP_UV * pulses;
		break;
	default:
		val->intval = MICRO_5V;
		break;
	}

	return 0;
}

int smblib_get_prop_pd_in_hard_reset(struct smb_charger *chg,
			       union power_supply_propval *val)
{
	int rc;
	u8 ctrl;

	rc = smblib_read(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG, &ctrl);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG rc=%d\n",
			rc);
		return rc;
	}
	val->intval = ctrl & EXIT_SNK_BASED_ON_CC_BIT;
	return 0;
}

int smblib_get_pe_start(struct smb_charger *chg,
			       union power_supply_propval *val)
{
	/*
	 * hvdcp timeout voter is the last one to allow pd. Use its vote
	 * to indicate start of pe engine
	 */
	val->intval
		= !get_client_vote_locked(chg->pd_disallowed_votable_indirect,
			HVDCP_TIMEOUT_VOTER);
	return 0;
}

int smblib_get_prop_die_health(struct smb_charger *chg,
						union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, TEMP_RANGE_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TEMP_RANGE_STATUS_REG rc=%d\n",
									rc);
		return rc;
	}

	/* TEMP_RANGE bits are mutually exclusive */
	switch (stat & TEMP_RANGE_MASK) {
	case TEMP_BELOW_RANGE_BIT:
		val->intval = POWER_SUPPLY_HEALTH_COOL;
		break;
	case TEMP_WITHIN_RANGE_BIT:
		val->intval = POWER_SUPPLY_HEALTH_WARM;
		break;
	case TEMP_ABOVE_RANGE_BIT:
		val->intval = POWER_SUPPLY_HEALTH_HOT;
		break;
	case ALERT_LEVEL_BIT:
		val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
		break;
	default:
		val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;
	}

	return 0;
}

/*******************
 * USB PSY SETTERS *
 * *****************/

int smblib_set_prop_pd_current_max(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	int rc;

	if (chg->pd_active)
		rc = vote(chg->usb_icl_votable, PD_VOTER, true, val->intval);
	else
		rc = -EPERM;

	return rc;
}

int smblib_set_prop_usb_current_max(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	int rc = 0;

	if (!chg->pd_active) {
		rc = vote(chg->usb_icl_votable, USB_PSY_VOTER,
				true, val->intval);
	} else if (chg->system_suspend_supported) {
		if (val->intval <= USBIN_25MA)
			rc = vote(chg->usb_icl_votable,
				PD_SUSPEND_SUPPORTED_VOTER, true, val->intval);
		else
			rc = vote(chg->usb_icl_votable,
				PD_SUSPEND_SUPPORTED_VOTER, false, 0);
	}
	return rc;
}

int smblib_set_prop_boost_current(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	int rc = 0;

	rc = smblib_set_charge_param(chg, &chg->param.freq_boost,
				val->intval <= chg->boost_threshold_ua ?
				chg->chg_freq.freq_below_otg_threshold :
				chg->chg_freq.freq_above_otg_threshold);
	if (rc < 0) {
		dev_err(chg->dev, "Error in setting freq_boost rc=%d\n", rc);
		return rc;
	}

	chg->boost_current_ua = val->intval;
	return rc;
}

int smblib_set_prop_typec_power_role(struct smb_charger *chg,
				     const union power_supply_propval *val)
{
	int rc = 0;
	u8 power_role;

	switch (val->intval) {
	case POWER_SUPPLY_TYPEC_PR_NONE:
		power_role = TYPEC_DISABLE_CMD_BIT;
		break;
	case POWER_SUPPLY_TYPEC_PR_DUAL:
		power_role = 0;
		break;
	case POWER_SUPPLY_TYPEC_PR_SINK:
		power_role = UFP_EN_CMD_BIT;
		break;
	case POWER_SUPPLY_TYPEC_PR_SOURCE:
		power_role = DFP_EN_CMD_BIT;
		break;
	default:
		smblib_err(chg, "power role %d not supported\n", val->intval);
		return -EINVAL;
	}

	rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 TYPEC_POWER_ROLE_CMD_MASK, power_role);
	if (rc < 0) {
		smblib_err(chg, "Couldn't write 0x%02x to TYPE_C_INTRPT_ENB_SOFTWARE_CTRL rc=%d\n",
			power_role, rc);
		return rc;
	}

	return rc;
}

int smblib_set_prop_usb_voltage_min(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	int rc, min_uv;

	min_uv = min(val->intval, chg->voltage_max_uv);
	rc = smblib_set_usb_pd_allowed_voltage(chg, min_uv,
					       chg->voltage_max_uv);
	if (rc < 0) {
		smblib_err(chg, "invalid max voltage %duV rc=%d\n",
			val->intval, rc);
		return rc;
	}

	if (chg->mode == PARALLEL_MASTER)
		vote(chg->pl_enable_votable_indirect, USBIN_V_VOTER,
		     min_uv > MICRO_5V, 0);

	chg->voltage_min_uv = min_uv;
	return rc;
}

int smblib_set_prop_usb_voltage_max(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	int rc, max_uv;

	max_uv = max(val->intval, chg->voltage_min_uv);
	rc = smblib_set_usb_pd_allowed_voltage(chg, chg->voltage_min_uv,
					       max_uv);
	if (rc < 0) {
		smblib_err(chg, "invalid min voltage %duV rc=%d\n",
			val->intval, rc);
		return rc;
	}

	chg->voltage_max_uv = max_uv;
	rc = smblib_rerun_aicl(chg);
	if (rc < 0)
		smblib_err(chg, "Couldn't re-run AICL rc=%d\n", rc);

	return rc;
}

int smblib_set_prop_pd_active(struct smb_charger *chg,
			      const union power_supply_propval *val)
{
	int rc;
	u8 stat = 0;
	bool cc_debounced;
	bool orientation;
	bool pd_active = val->intval;

	if (!get_effective_result(chg->pd_allowed_votable)) {
		smblib_err(chg, "PD is not allowed\n");
		return -EINVAL;
	}

	vote(chg->apsd_disable_votable, PD_VOTER, pd_active, 0);
	vote(chg->pd_allowed_votable, PD_VOTER, pd_active, 0);

	/*
	 * VCONN_EN_ORIENTATION_BIT controls whether to use CC1 or CC2 line
	 * when TYPEC_SPARE_CFG_BIT (CC pin selection s/w override) is set
	 * or when VCONN_EN_VALUE_BIT is set.
	 */
	rc = smblib_read(chg, TYPE_C_STATUS_4_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_4 rc=%d\n", rc);
		return rc;
	}

	if (pd_active) {
		orientation = stat & CC_ORIENTATION_BIT;
		rc = smblib_masked_write(chg,
				TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				VCONN_EN_ORIENTATION_BIT,
				orientation ? 0 : VCONN_EN_ORIENTATION_BIT);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't enable vconn on CC line rc=%d\n", rc);
			return rc;
		}
		/*
		 * Enforce 500mA for PD until the real vote comes in later.
		 * It is guaranteed that pd_active is set prior to
		 * pd_current_max
		 */
		rc = vote(chg->usb_icl_votable, PD_VOTER, true, USBIN_500MA);
		if (rc < 0) {
			smblib_err(chg, "Couldn't vote for USB ICL rc=%d\n",
					rc);
			return rc;
		}

		/* clear USB ICL vote for DCP_VOTER */
		rc = vote(chg->usb_icl_votable, DCP_VOTER, false, 0);
		if (rc < 0)
			smblib_err(chg,
				"Couldn't un-vote DCP from USB ICL rc=%d\n",
				rc);

		/* clear USB ICL vote for PL_USBIN_USBIN_VOTER */
		rc = vote(chg->usb_icl_votable, PL_USBIN_USBIN_VOTER, false, 0);
		if (rc < 0)
			smblib_err(chg,
					"Couldn't un-vote PL_USBIN_USBIN from USB ICL rc=%d\n",
					rc);

		/* remove USB_PSY_VOTER */
		rc = vote(chg->usb_icl_votable, USB_PSY_VOTER, false, 0);
		if (rc < 0) {
			smblib_err(chg, "Couldn't unvote USB_PSY rc=%d\n", rc);
			return rc;
		}

		/* pd active set, parallel charger can be enabled now */
		rc = vote(chg->pl_disable_votable, PL_DELAY_HVDCP_VOTER,
				false, 0);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't unvote PL_DELAY_HVDCP_VOTER rc=%d\n",
				rc);
			return rc;
		}
	}

	/* CC pin selection s/w override in PD session; h/w otherwise. */
	rc = smblib_masked_write(chg, TAPER_TIMER_SEL_CFG_REG,
				 TYPEC_SPARE_CFG_BIT,
				 pd_active ? TYPEC_SPARE_CFG_BIT : 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't change cc_out ctrl to %s rc=%d\n",
			pd_active ? "SW" : "HW", rc);
		return rc;
	}

	cc_debounced = (bool)(stat & TYPEC_DEBOUNCE_DONE_STATUS_BIT);
	if (!pd_active && cc_debounced)
		try_rerun_apsd_for_hvdcp(chg);

	chg->pd_active = pd_active;
	smblib_update_usb_type(chg);
	power_supply_changed(chg->usb_psy);

	return rc;
}

int smblib_set_prop_ship_mode(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	int rc;

	smblib_dbg(chg, PR_MISC, "Set ship mode: %d!!\n", !!val->intval);

	rc = smblib_masked_write(chg, SHIP_MODE_REG, SHIP_MODE_EN_BIT,
			!!val->intval ? SHIP_MODE_EN_BIT : 0);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't %s ship mode, rc=%d\n",
				!!val->intval ? "enable" : "disable", rc);

	return rc;
}

int smblib_reg_block_update(struct smb_charger *chg,
				struct reg_info *entry)
{
	int rc = 0;

	while (entry && entry->reg) {
		rc = smblib_read(chg, entry->reg, &entry->bak);
		if (rc < 0) {
			dev_err(chg->dev, "Error in reading %s rc=%d\n",
				entry->desc, rc);
			break;
		}
		entry->bak &= entry->mask;

		rc = smblib_masked_write(chg, entry->reg,
					 entry->mask, entry->val);
		if (rc < 0) {
			dev_err(chg->dev, "Error in writing %s rc=%d\n",
				entry->desc, rc);
			break;
		}
		entry++;
	}

	return rc;
}

int smblib_reg_block_restore(struct smb_charger *chg,
				struct reg_info *entry)
{
	int rc = 0;

	while (entry && entry->reg) {
		rc = smblib_masked_write(chg, entry->reg,
					 entry->mask, entry->bak);
		if (rc < 0) {
			dev_err(chg->dev, "Error in writing %s rc=%d\n",
				entry->desc, rc);
			break;
		}
		entry++;
	}

	return rc;
}

static struct reg_info cc2_detach_settings[] = {
	{
		.reg	= TYPE_C_CFG_REG,
		.mask	= APSD_START_ON_CC_BIT,
		.val	= 0,
		.desc	= "TYPE_C_CFG_REG",
	},
	{
		.reg	= TYPE_C_CFG_2_REG,
		.mask	= TYPE_C_UFP_MODE_BIT | EN_TRY_SOURCE_MODE_BIT,
		.val	= TYPE_C_UFP_MODE_BIT,
		.desc	= "TYPE_C_CFG_2_REG",
	},
	{
		.reg	= TYPE_C_CFG_3_REG,
		.mask	= EN_TRYSINK_MODE_BIT,
		.val	= 0,
		.desc	= "TYPE_C_CFG_3_REG",
	},
	{
		.reg	= TAPER_TIMER_SEL_CFG_REG,
		.mask	= TYPEC_SPARE_CFG_BIT,
		.val	= TYPEC_SPARE_CFG_BIT,
		.desc	= "TAPER_TIMER_SEL_CFG_REG",
	},
	{
		.reg	= TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
		.mask	= VCONN_EN_ORIENTATION_BIT,
		.val	= 0,
		.desc	= "TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG",
	},
	{
		.reg	= MISC_CFG_REG,
		.mask	= TCC_DEBOUNCE_20MS_BIT,
		.val	= TCC_DEBOUNCE_20MS_BIT,
		.desc	= "Tccdebounce time"
	},
	{
	},
};

static int smblib_cc2_sink_removal_enter(struct smb_charger *chg)
{
	int rc = 0;
	union power_supply_propval cc2_val = {0, };

	if ((chg->wa_flags & TYPEC_CC2_REMOVAL_WA_BIT) == 0)
		return rc;

	if (chg->cc2_sink_detach_flag != CC2_SINK_NONE)
		return rc;

	rc = smblib_get_prop_typec_cc_orientation(chg, &cc2_val);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get cc orientation rc=%d\n", rc);
		return rc;
	}
	if (cc2_val.intval == 1)
		return rc;

	rc = smblib_get_prop_typec_mode(chg, &cc2_val);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get prop typec mode rc=%d\n", rc);
		return rc;
	}

	switch (cc2_val.intval) {
	case POWER_SUPPLY_TYPEC_SOURCE_DEFAULT:
		smblib_reg_block_update(chg, cc2_detach_settings);
		chg->cc2_sink_detach_flag = CC2_SINK_STD;
		schedule_work(&chg->rdstd_cc2_detach_work);
		break;
	case POWER_SUPPLY_TYPEC_SOURCE_MEDIUM:
	case POWER_SUPPLY_TYPEC_SOURCE_HIGH:
		chg->cc2_sink_detach_flag = CC2_SINK_MEDIUM_HIGH;
		break;
	default:
		break;
	}

	return rc;
}

static int smblib_cc2_sink_removal_exit(struct smb_charger *chg)
{
	int rc = 0;

	if ((chg->wa_flags & TYPEC_CC2_REMOVAL_WA_BIT) == 0)
		return rc;

	if (chg->cc2_sink_detach_flag == CC2_SINK_STD) {
		cancel_work_sync(&chg->rdstd_cc2_detach_work);
		smblib_reg_block_restore(chg, cc2_detach_settings);
	}

	chg->cc2_sink_detach_flag = CC2_SINK_NONE;

	return rc;
}

int smblib_set_prop_pd_in_hard_reset(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	int rc;

	rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 EXIT_SNK_BASED_ON_CC_BIT,
				 (val->intval) ? EXIT_SNK_BASED_ON_CC_BIT : 0);
	if (rc < 0) {
		smblib_err(chg, "Could not set EXIT_SNK_BASED_ON_CC rc=%d\n",
				rc);
		return rc;
	}

	vote(chg->apsd_disable_votable, PD_HARD_RESET_VOTER, val->intval, 0);

	if (val->intval)
		rc = smblib_cc2_sink_removal_enter(chg);
	else
		rc = smblib_cc2_sink_removal_exit(chg);

	if (rc < 0) {
		smblib_err(chg, "Could not detect cc2 removal rc=%d\n", rc);
		return rc;
	}

	return rc;
}

/***********************
* USB MAIN PSY GETTERS *
*************************/
int smblib_get_prop_fcc_delta(struct smb_charger *chg,
			       union power_supply_propval *val)
{
	int rc, jeita_cc_delta_ua, step_cc_delta_ua, hw_cc_delta_ua = 0;

	rc = smblib_get_step_cc_delta(chg, &step_cc_delta_ua);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get step cc delta rc=%d\n", rc);
		step_cc_delta_ua = 0;
	} else {
		hw_cc_delta_ua = step_cc_delta_ua;
	}

	rc = smblib_get_jeita_cc_delta(chg, &jeita_cc_delta_ua);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get jeita cc delta rc=%d\n", rc);
		jeita_cc_delta_ua = 0;
	} else if (jeita_cc_delta_ua < 0) {
		/* HW will take the min between JEITA and step charge */
		hw_cc_delta_ua = min(hw_cc_delta_ua, jeita_cc_delta_ua);
	}

	val->intval = hw_cc_delta_ua;
	return 0;
}

/***********************
* USB MAIN PSY SETTERS *
*************************/

#define SDP_CURRENT_MA			500000
#define CDP_CURRENT_MA			1500000
#define DCP_CURRENT_MA			1500000
#define HVDCP_CURRENT_MA		3000000
#define TYPEC_DEFAULT_CURRENT_MA	900000
#define TYPEC_MEDIUM_CURRENT_MA		1500000
#define TYPEC_HIGH_CURRENT_MA		3000000
static int smblib_get_charge_current(struct smb_charger *chg,
				int *total_current_ua)
{
	const struct apsd_result *apsd_result = smblib_update_usb_type(chg);
	union power_supply_propval val = {0, };
	int rc, typec_source_rd, current_ua;
	bool non_compliant;
	u8 stat5;

	rc = smblib_read(chg, TYPE_C_STATUS_5_REG, &stat5);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_5 rc=%d\n", rc);
		return rc;
	}
	non_compliant = stat5 & TYPEC_NONCOMP_LEGACY_CABLE_STATUS_BIT;

	/* get settled ICL */
	rc = smblib_get_prop_input_current_settled(chg, &val);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get settled ICL rc=%d\n", rc);
		return rc;
	}

	typec_source_rd = smblib_get_prop_ufp_mode(chg);

	/* QC 2.0/3.0 adapter */
	if (apsd_result->bit & (QC_3P0_BIT | QC_2P0_BIT)) {
		*total_current_ua = HVDCP_CURRENT_MA;
		return 0;
	}

	if (non_compliant) {
		switch (apsd_result->bit) {
		case CDP_CHARGER_BIT:
			current_ua = CDP_CURRENT_MA;
			break;
		case DCP_CHARGER_BIT:
		case OCP_CHARGER_BIT:
		case FLOAT_CHARGER_BIT:
			current_ua = DCP_CURRENT_MA;
			break;
		default:
			current_ua = 0;
			break;
		}

		*total_current_ua = max(current_ua, val.intval);
		return 0;
	}

	switch (typec_source_rd) {
	case POWER_SUPPLY_TYPEC_SOURCE_DEFAULT:
		switch (apsd_result->bit) {
		case CDP_CHARGER_BIT:
			current_ua = CDP_CURRENT_MA;
			break;
		case DCP_CHARGER_BIT:
		case OCP_CHARGER_BIT:
		case FLOAT_CHARGER_BIT:
			current_ua = chg->default_icl_ua;
			break;
		default:
			current_ua = 0;
			break;
		}
		break;
	case POWER_SUPPLY_TYPEC_SOURCE_MEDIUM:
		current_ua = TYPEC_MEDIUM_CURRENT_MA;
		break;
	case POWER_SUPPLY_TYPEC_SOURCE_HIGH:
		current_ua = TYPEC_HIGH_CURRENT_MA;
		break;
	case POWER_SUPPLY_TYPEC_NON_COMPLIANT:
	case POWER_SUPPLY_TYPEC_NONE:
	default:
		current_ua = 0;
		break;
	}

	*total_current_ua = max(current_ua, val.intval);
	return 0;
}

int smblib_set_icl_reduction(struct smb_charger *chg, int reduction_ua)
{
	int current_ua, rc;

	if (reduction_ua == 0) {
		vote(chg->usb_icl_votable, PL_USBIN_USBIN_VOTER, false, 0);
	} else {
		/*
		 * No usb_icl voter means we are defaulting to hw chosen
		 * max limit. We need a vote from s/w to enforce the reduction.
		 */
		if (get_effective_result(chg->usb_icl_votable) == -EINVAL) {
			rc = smblib_get_charge_current(chg, &current_ua);
			if (rc < 0) {
				pr_err("Failed to get ICL rc=%d\n", rc);
				return rc;
			}
			vote(chg->usb_icl_votable, PL_USBIN_USBIN_VOTER, true,
					current_ua);
		}
	}

	chg->icl_reduction_ua = reduction_ua;

	return rerun_election(chg->usb_icl_votable);
}

/************************
 * PARALLEL PSY GETTERS *
 ************************/

int smblib_get_prop_slave_current_now(struct smb_charger *chg,
				      union power_supply_propval *pval)
{
	if (IS_ERR_OR_NULL(chg->iio.batt_i_chan))
		chg->iio.batt_i_chan = iio_channel_get(chg->dev, "batt_i");

	if (IS_ERR(chg->iio.batt_i_chan))
		return PTR_ERR(chg->iio.batt_i_chan);

	return iio_read_channel_processed(chg->iio.batt_i_chan, &pval->intval);
}

/**********************
 * INTERRUPT HANDLERS *
 **********************/

irqreturn_t smblib_handle_debug(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	//smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s, DEBUG\n", irq_data->name);
	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_otg_overcurrent(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int rc;
	u8 stat;

	rc = smblib_read(chg, OTG_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't read OTG_INT_RT_STS rc=%d\n", rc);
		return IRQ_HANDLED;
	}

	if (chg->wa_flags & OTG_WA) {
		if (stat & OTG_OC_DIS_SW_STS_RT_STS_BIT)
			smblib_err(chg, "OTG disabled by hw\n");

		/* not handling software based hiccups for PM660 */
		return IRQ_HANDLED;
	}

	if (stat & OTG_OVERCURRENT_RT_STS_BIT)
		schedule_work(&chg->otg_oc_work);

	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_chg_state_change(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	u8 stat;
	int rc;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
			rc);
		return IRQ_HANDLED;
	}

	stat = stat & BATTERY_CHARGER_STATUS_MASK;
	power_supply_changed(chg->batt_psy);
	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_step_chg_state_change(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	if (chg->step_chg_enabled)
		rerun_election(chg->fcc_votable);

	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_step_chg_soc_update_fail(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	if (chg->step_chg_enabled)
		rerun_election(chg->fcc_votable);

	return IRQ_HANDLED;
}

#define STEP_SOC_REQ_MS	3000
irqreturn_t smblib_handle_step_chg_soc_update_request(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int rc;
	union power_supply_propval pval = {0, };

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	if (!chg->bms_psy) {
		schedule_delayed_work(&chg->step_soc_req_work,
				      msecs_to_jiffies(STEP_SOC_REQ_MS));
		return IRQ_HANDLED;
	}

	rc = smblib_get_prop_batt_capacity(chg, &pval);
	if (rc < 0)
		smblib_err(chg, "Couldn't get batt capacity rc=%d\n", rc);
	else
		step_charge_soc_update(chg, pval.intval);

	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_batt_temp_changed(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	rerun_election(chg->fcc_votable);
	power_supply_changed(chg->batt_psy);
	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_batt_psy_changed(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);
	power_supply_changed(chg->batt_psy);
	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_usb_psy_changed(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);
	power_supply_changed(chg->usb_psy);
	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_usbin_uv(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	struct storm_watch *wdata;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);
	if (!chg->irq_info[SWITCH_POWER_OK_IRQ].irq_data)
		return IRQ_HANDLED;

	wdata = &chg->irq_info[SWITCH_POWER_OK_IRQ].irq_data->storm_data;
	reset_storm_count(wdata);
	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_usb_plugin(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int rc;
	u8 stat;
	bool vbus_rising;

	rc = smblib_read(chg, USBIN_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't read USB_INT_RT_STS rc=%d\n", rc);
		return IRQ_HANDLED;
	}

	vbus_rising = (bool)(stat & USBIN_PLUGIN_RT_STS_BIT);
	smblib_set_opt_freq_buck(chg,
		vbus_rising ? chg->chg_freq.freq_5V :
			chg->chg_freq.freq_removal);
#ifdef OPLUS_FEATURE_CHG_BASIC
	printk(KERN_ERR "!!!!! smblib_handle_usb_plugin: [%d]\n", vbus_rising);
#endif
	/* fetch the DPDM regulator */
	if (!chg->dpdm_reg && of_get_property(chg->dev->of_node,
						"dpdm-supply", NULL)) {
		chg->dpdm_reg = devm_regulator_get(chg->dev, "dpdm");
		if (IS_ERR(chg->dpdm_reg)) {
			smblib_err(chg, "Couldn't get dpdm regulator rc=%ld\n",
				PTR_ERR(chg->dpdm_reg));
			chg->dpdm_reg = NULL;
		}
	}

	if (vbus_rising) {
		if (chg->dpdm_reg && !regulator_is_enabled(chg->dpdm_reg)) {
			smblib_dbg(chg, PR_MISC, "enabling DPDM regulator\n");
			rc = regulator_enable(chg->dpdm_reg);
			if (rc < 0)
				smblib_err(chg, "Couldn't enable dpdm regulator rc=%d\n",
					rc);
		}
	} else {
#ifdef OPLUS_FEATURE_CHG_BASIC
		oplus_vooc_reset_fastchg_after_usbout();
		if (oplus_vooc_get_fastchg_started() == false && g_oplus_chip) {
			smbchg_set_chargerid_switch_val(0);
			g_oplus_chip->chargerid_volt = 0;
			g_oplus_chip->chargerid_volt_got = false;
			g_oplus_chip->charger_type = POWER_SUPPLY_TYPE_UNKNOWN;
			oplus_chg_wake_update_work();
		}
		chg->pre_current_ma = -1;
#endif

		if (chg->wa_flags & BOOST_BACK_WA)
			vote(chg->usb_icl_votable, BOOST_BACK_VOTER, false, 0);

		if (chg->dpdm_reg && regulator_is_enabled(chg->dpdm_reg)) {
			smblib_dbg(chg, PR_MISC, "disabling DPDM regulator\n");
			rc = regulator_disable(chg->dpdm_reg);
			if (rc < 0)
				smblib_err(chg, "Couldn't disable dpdm regulator rc=%d\n",
					rc);
		}

		if (chg->micro_usb_mode) {
			smblib_update_usb_type(chg);
			extcon_set_cable_state_(chg->extcon, EXTCON_USB, false);
			smblib_uusb_removal(chg);
		}
	}

#ifdef OPLUS_FEATURE_CHG_BASIC
	if (vbus_rising) {
		cancel_delayed_work_sync(&chg->chg_monitor_work);
		schedule_delayed_work(&chg->chg_monitor_work, OPLUS_CHG_MONITOR_INTERVAL);
	} else {
		cancel_delayed_work_sync(&chg->chg_monitor_work);
	}
#endif

	power_supply_changed(chg->usb_psy);
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s %s\n",
		irq_data->name, vbus_rising ? "attached" : "detached");
	return IRQ_HANDLED;
}

#define USB_WEAK_INPUT_UA	1400000
#define ICL_CHANGE_DELAY_MS	1000
irqreturn_t smblib_handle_icl_change(int irq, void *data)
{
	u8 stat;
	int rc, settled_ua, delay = ICL_CHANGE_DELAY_MS;
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	if (chg->mode == PARALLEL_MASTER) {
		rc = smblib_read(chg, AICL_STATUS_REG, &stat);
		if (rc < 0) {
			smblib_err(chg, "Couldn't read AICL_STATUS rc=%d\n",
					rc);
			return IRQ_HANDLED;
		}

		rc = smblib_get_charge_param(chg, &chg->param.icl_stat,
				&settled_ua);
		if (rc < 0) {
			smblib_err(chg, "Couldn't get ICL status rc=%d\n", rc);
			return IRQ_HANDLED;
		}

		/* If AICL settled then schedule work now */
		if ((settled_ua == get_effective_result(chg->usb_icl_votable))
				|| (stat & AICL_DONE_BIT))
			delay = 0;

		cancel_delayed_work_sync(&chg->icl_change_work);
		schedule_delayed_work(&chg->icl_change_work,
						msecs_to_jiffies(delay));
	}

	return IRQ_HANDLED;
}

static void smblib_handle_slow_plugin_timeout(struct smb_charger *chg,
					      bool rising)
{
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: slow-plugin-timeout %s\n",
		   rising ? "rising" : "falling");
}

static void smblib_handle_sdp_enumeration_done(struct smb_charger *chg,
					       bool rising)
{
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: sdp-enumeration-done %s\n",
		   rising ? "rising" : "falling");
}

#define QC3_PULSES_FOR_6V	5
#define QC3_PULSES_FOR_9V	20
#define QC3_PULSES_FOR_12V	35
static void smblib_hvdcp_adaptive_voltage_change(struct smb_charger *chg)
{
	int rc;
	u8 stat;
	int pulses;

	power_supply_changed(chg->usb_main_psy);
	if (chg->usb_psy_desc.type == POWER_SUPPLY_TYPE_USB_HVDCP) {
		rc = smblib_read(chg, QC_CHANGE_STATUS_REG, &stat);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't read QC_CHANGE_STATUS rc=%d\n", rc);
			return;
		}

		switch (stat & QC_2P0_STATUS_MASK) {
		case QC_5V_BIT:
			smblib_set_opt_freq_buck(chg,
					chg->chg_freq.freq_5V);
			break;
		case QC_9V_BIT:
			smblib_set_opt_freq_buck(chg,
					chg->chg_freq.freq_9V);
			break;
		case QC_12V_BIT:
			smblib_set_opt_freq_buck(chg,
					chg->chg_freq.freq_12V);
			break;
		default:
			smblib_set_opt_freq_buck(chg,
					chg->chg_freq.freq_removal);
			break;
		}
	}

	if (chg->usb_psy_desc.type == POWER_SUPPLY_TYPE_USB_HVDCP_3) {
		rc = smblib_read(chg, QC_PULSE_COUNT_STATUS_REG, &stat);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't read QC_PULSE_COUNT rc=%d\n", rc);
			return;
		}
		pulses = (stat & QC_PULSE_COUNT_MASK);

		if (pulses < QC3_PULSES_FOR_6V)
			smblib_set_opt_freq_buck(chg,
				chg->chg_freq.freq_5V);
		else if (pulses < QC3_PULSES_FOR_9V)
			smblib_set_opt_freq_buck(chg,
				chg->chg_freq.freq_6V_8V);
		else if (pulses < QC3_PULSES_FOR_12V)
			smblib_set_opt_freq_buck(chg,
				chg->chg_freq.freq_9V);
		else
			smblib_set_opt_freq_buck(chg,
				chg->chg_freq.freq_12V);
	}
}

/* triggers when HVDCP 3.0 authentication has finished */
static void smblib_handle_hvdcp_3p0_auth_done(struct smb_charger *chg,
					      bool rising)
{
	const struct apsd_result *apsd_result;
	int rc;

	if (!rising)
		return;

	if (chg->wa_flags & QC_AUTH_INTERRUPT_WA_BIT) {
		/*
		 * Disable AUTH_IRQ_EN_CFG_BIT to receive adapter voltage
		 * change interrupt.
		 */
		rc = smblib_masked_write(chg,
				USBIN_SOURCE_CHANGE_INTRPT_ENB_REG,
				AUTH_IRQ_EN_CFG_BIT, 0);
		if (rc < 0)
			smblib_err(chg,
				"Couldn't enable QC auth setting rc=%d\n", rc);
	}

	if (chg->mode == PARALLEL_MASTER)
		vote(chg->pl_enable_votable_indirect, USBIN_V_VOTER, true, 0);

	/* the APSD done handler will set the USB supply type */
	apsd_result = smblib_get_apsd_result(chg);
	if (get_effective_result(chg->hvdcp_hw_inov_dis_votable)) {
		if (apsd_result->pst == POWER_SUPPLY_TYPE_USB_HVDCP) {
			/* force HVDCP2 to 9V if INOV is disabled */
			rc = smblib_masked_write(chg, CMD_HVDCP_2_REG,
					FORCE_9V_BIT, FORCE_9V_BIT);
			if (rc < 0)
				smblib_err(chg,
					"Couldn't force 9V HVDCP rc=%d\n", rc);
		}
	}

	/* QC authentication done, parallel charger can be enabled now */
	vote(chg->pl_disable_votable, PL_DELAY_HVDCP_VOTER, false, 0);

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: hvdcp-3p0-auth-done rising; %s detected\n",
		   apsd_result->name);
}

static void smblib_handle_hvdcp_check_timeout(struct smb_charger *chg,
					      bool rising, bool qc_charger)
{
	const struct apsd_result *apsd_result = smblib_update_usb_type(chg);

	/* Hold off PD only until hvdcp 2.0 detection timeout */
	if (rising) {
		vote(chg->pd_disallowed_votable_indirect, HVDCP_TIMEOUT_VOTER,
								false, 0);
		if (get_effective_result(chg->pd_disallowed_votable_indirect))
			/* could be a legacy cable, try doing hvdcp */
			try_rerun_apsd_for_hvdcp(chg);

		/*
		 * HVDCP detection timeout done
		 * If adapter is not QC2.0/QC3.0 - it is a plain old DCP.
		 */
		if (!qc_charger && (apsd_result->bit & DCP_CHARGER_BIT))
			/* enforce DCP ICL if specified */
			vote(chg->usb_icl_votable, DCP_VOTER,
				chg->dcp_icl_ua != -EINVAL, chg->dcp_icl_ua);
		/*
		 * If adapter is not QC2.0/QC3.0 remove vote for parallel
		 * disable.
		 * Otherwise if adapter is QC2.0/QC3.0 wait for authentication
		 * to complete.
		 */
		if (!qc_charger)
			vote(chg->pl_disable_votable, PL_DELAY_HVDCP_VOTER,
					false, 0);
	}

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: smblib_handle_hvdcp_check_timeout %s\n",
		   rising ? "rising" : "falling");
}

/* triggers when HVDCP is detected */
static void smblib_handle_hvdcp_detect_done(struct smb_charger *chg,
					    bool rising)
{
	if (!rising)
		return;

	/* the APSD done handler will set the USB supply type */
	cancel_delayed_work_sync(&chg->hvdcp_detect_work);
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: hvdcp-detect-done %s\n",
		   rising ? "rising" : "falling");
}

#define HVDCP_DET_MS 2500
static void smblib_handle_apsd_done(struct smb_charger *chg, bool rising)
{
	const struct apsd_result *apsd_result;

	if (!rising)
		return;

	apsd_result = smblib_update_usb_type(chg);
	switch (apsd_result->bit) {
	case SDP_CHARGER_BIT:
	case CDP_CHARGER_BIT:
		if (chg->micro_usb_mode)
			extcon_set_cable_state_(chg->extcon, EXTCON_USB,
					true);
	case OCP_CHARGER_BIT:
	case FLOAT_CHARGER_BIT:
		/*
		 * if not DCP then no hvdcp timeout happens. Enable
		 * pd/parallel here.
		 */
		vote(chg->pd_disallowed_votable_indirect, HVDCP_TIMEOUT_VOTER,
				false, 0);
		vote(chg->pl_disable_votable, PL_DELAY_HVDCP_VOTER, false, 0);
		break;
	case DCP_CHARGER_BIT:
		if (chg->wa_flags & QC_CHARGER_DETECTION_WA_BIT)
			schedule_delayed_work(&chg->hvdcp_detect_work,
					      msecs_to_jiffies(HVDCP_DET_MS));
		break;
	default:
		break;
	}
#ifdef OPLUS_FEATURE_CHG_BASIC
	printk(KERN_ERR "!!!IRQ: apsd-done rising; %s detected\n", apsd_result->name);
#endif
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: apsd-done rising; %s detected\n",
		   apsd_result->name);
}

irqreturn_t smblib_handle_usb_source_change(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int rc = 0;
	u8 stat;
#ifdef OPLUS_FEATURE_CHG_BASIC
	u8 reg_value;
#endif

	rc = smblib_read(chg, APSD_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read APSD_STATUS rc=%d\n", rc);
		return IRQ_HANDLED;
	}
	smblib_dbg(chg, PR_REGISTER, "APSD_STATUS = 0x%02x\n", stat);

#ifdef OPLUS_FEATURE_CHG_BASIC
	if (chg->micro_usb_mode && (stat & APSD_DTC_STATUS_DONE_BIT)
			&& !chg->uusb_apsd_rerun_done) {
		/*
		 * Force re-run APSD to handle slow insertion related
		 * charger-mis-detection.
		 */
#ifndef OPLUS_FEATURE_CHG_BASIC
		chg->uusb_apsd_rerun_done = true;
		smblib_rerun_apsd(chg);
		return IRQ_HANDLED;
#else
		smblib_read(chg, APSD_RESULT_STATUS_REG, &reg_value);
		if (reg_value & (CDP_CHARGER_BIT | SDP_CHARGER_BIT)) {
			chg->uusb_apsd_rerun_done = true;
			smblib_rerun_apsd(chg);
			return IRQ_HANDLED;
		}
#endif
	}
#endif

	smblib_handle_apsd_done(chg,
		(bool)(stat & APSD_DTC_STATUS_DONE_BIT));

#ifdef OPLUS_FEATURE_CHG_BASIC
	if ((bool)(stat & APSD_DTC_STATUS_DONE_BIT))
		oplus_chg_wake_update_work();
#endif

	smblib_handle_hvdcp_detect_done(chg,
		(bool)(stat & QC_CHARGER_BIT));

	smblib_handle_hvdcp_check_timeout(chg,
		(bool)(stat & HVDCP_CHECK_TIMEOUT_BIT),
		(bool)(stat & QC_CHARGER_BIT));

	smblib_handle_hvdcp_3p0_auth_done(chg,
		(bool)(stat & QC_AUTH_DONE_STATUS_BIT));

	smblib_handle_sdp_enumeration_done(chg,
		(bool)(stat & ENUMERATION_DONE_BIT));

	smblib_handle_slow_plugin_timeout(chg,
		(bool)(stat & SLOW_PLUGIN_TIMEOUT_BIT));

	smblib_hvdcp_adaptive_voltage_change(chg);

	power_supply_changed(chg->usb_psy);

	rc = smblib_read(chg, APSD_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read APSD_STATUS rc=%d\n", rc);
		return IRQ_HANDLED;
	}
	smblib_dbg(chg, PR_REGISTER, "APSD_STATUS = 0x%02x\n", stat);

	return IRQ_HANDLED;
}

static void typec_source_removal(struct smb_charger *chg)
{
	int rc;

	/* reset both usbin current and voltage votes */
	vote(chg->pl_enable_votable_indirect, USBIN_I_VOTER, false, 0);
	vote(chg->pl_enable_votable_indirect, USBIN_V_VOTER, false, 0);

	cancel_delayed_work_sync(&chg->hvdcp_detect_work);

	if (chg->wa_flags & QC_AUTH_INTERRUPT_WA_BIT) {
		/* re-enable AUTH_IRQ_EN_CFG_BIT */
		rc = smblib_masked_write(chg,
				USBIN_SOURCE_CHANGE_INTRPT_ENB_REG,
				AUTH_IRQ_EN_CFG_BIT, AUTH_IRQ_EN_CFG_BIT);
		if (rc < 0)
			smblib_err(chg,
				"Couldn't enable QC auth setting rc=%d\n", rc);
	}

	/* reconfigure allowed voltage for HVDCP */
	rc = smblib_set_adapter_allowance(chg,
			USBIN_ADAPTER_ALLOW_5V_OR_9V_TO_12V);
	if (rc < 0)
		smblib_err(chg, "Couldn't set USBIN_ADAPTER_ALLOW_5V_OR_9V_TO_12V rc=%d\n",
			rc);

	chg->voltage_min_uv = MICRO_5V;
	chg->voltage_max_uv = MICRO_5V;

	/* clear USB ICL vote for PD_VOTER */
	rc = vote(chg->usb_icl_votable, PD_VOTER, false, 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't un-vote PD from USB ICL rc=%d\n", rc);

	/* clear USB ICL vote for USB_PSY_VOTER */
	rc = vote(chg->usb_icl_votable, USB_PSY_VOTER, false, 0);
	if (rc < 0)
		smblib_err(chg,
			"Couldn't un-vote USB_PSY from USB ICL rc=%d\n", rc);

	/* clear USB ICL vote for DCP_VOTER */
	rc = vote(chg->usb_icl_votable, DCP_VOTER, false, 0);
	if (rc < 0)
		smblib_err(chg,
			"Couldn't un-vote DCP from USB ICL rc=%d\n", rc);

	/* clear USB ICL vote for PL_USBIN_USBIN_VOTER */
	rc = vote(chg->usb_icl_votable, PL_USBIN_USBIN_VOTER, false, 0);
	if (rc < 0)
		smblib_err(chg,
			"Couldn't un-vote PL_USBIN_USBIN from USB ICL rc=%d\n",
			rc);
}

static void typec_source_insertion(struct smb_charger *chg)
{
}

static void typec_sink_insertion(struct smb_charger *chg)
{
	/* when a sink is inserted we should not wait on hvdcp timeout to
	 * enable pd
	 */
	vote(chg->pd_disallowed_votable_indirect, HVDCP_TIMEOUT_VOTER,
			false, 0);
}

static void typec_sink_removal(struct smb_charger *chg)
{
	smblib_set_charge_param(chg, &chg->param.freq_boost,
			chg->chg_freq.freq_above_otg_threshold);
	chg->boost_current_ua = 0;
}

static void smblib_handle_typec_removal(struct smb_charger *chg)
{
	vote(chg->pd_disallowed_votable_indirect, CC_DETACHED_VOTER, true, 0);
	vote(chg->pd_disallowed_votable_indirect, HVDCP_TIMEOUT_VOTER, true, 0);
	vote(chg->pd_disallowed_votable_indirect, LEGACY_CABLE_VOTER, true, 0);
	vote(chg->pd_disallowed_votable_indirect, VBUS_CC_SHORT_VOTER, true, 0);
	vote(chg->pl_disable_votable, PL_DELAY_HVDCP_VOTER, true, 0);

	/* reset votes from vbus_cc_short */
	vote(chg->hvdcp_disable_votable_indirect, VBUS_CC_SHORT_VOTER,
			true, 0);
	vote(chg->hvdcp_disable_votable_indirect, PD_INACTIVE_VOTER,
			true, 0);
	/*
	 * cable could be removed during hard reset, remove its vote to
	 * disable apsd
	 */
	vote(chg->apsd_disable_votable, PD_HARD_RESET_VOTER, false, 0);

	chg->vconn_attempts = 0;
	chg->otg_attempts = 0;
	chg->pulse_cnt = 0;
	chg->usb_icl_delta_ua = 0;

	chg->usb_ever_removed = true;

	smblib_update_usb_type(chg);

	typec_source_removal(chg);
	typec_sink_removal(chg);
}

static void smblib_handle_typec_insertion(struct smb_charger *chg,
		bool sink_attached, bool legacy_cable)
{
	int rp;
	bool vbus_cc_short = false;
	bool valid_legacy_cable;

	vote(chg->pd_disallowed_votable_indirect, CC_DETACHED_VOTER, false, 0);

	if (sink_attached) {
		typec_source_removal(chg);
		typec_sink_insertion(chg);
	} else {
		typec_source_insertion(chg);
		typec_sink_removal(chg);
	}

	valid_legacy_cable = legacy_cable &&
		(chg->usb_ever_removed || !smblib_sysok_reason_usbin(chg));
	vote(chg->pd_disallowed_votable_indirect, LEGACY_CABLE_VOTER,
			valid_legacy_cable, 0);

	if (valid_legacy_cable) {
		rp = smblib_get_prop_ufp_mode(chg);
		if (rp == POWER_SUPPLY_TYPEC_SOURCE_HIGH
				|| rp == POWER_SUPPLY_TYPEC_NON_COMPLIANT) {
			vbus_cc_short = true;
			smblib_err(chg, "Disabling PD and HVDCP, VBUS-CC shorted, rp = %d found\n",
					rp);
		}
	}

	vote(chg->hvdcp_disable_votable_indirect, VBUS_CC_SHORT_VOTER,
			vbus_cc_short, 0);
	vote(chg->pd_disallowed_votable_indirect, VBUS_CC_SHORT_VOTER,
			vbus_cc_short, 0);
}

static void smblib_handle_typec_debounce_done(struct smb_charger *chg,
			bool rising, bool sink_attached, bool legacy_cable)
{
	int rc;
	union power_supply_propval pval = {0, };

	if (rising)
		smblib_handle_typec_insertion(chg, sink_attached, legacy_cable);
	else
		smblib_handle_typec_removal(chg);

	rc = smblib_get_prop_typec_mode(chg, &pval);
	if (rc < 0)
		smblib_err(chg, "Couldn't get prop typec mode rc=%d\n", rc);

	/*
	 * HW BUG - after cable is removed, medium or high rd reading
	 * falls to std. Use it for signal of typec cc detachment in
	 * software WA.
	 */
	if (chg->cc2_sink_detach_flag == CC2_SINK_MEDIUM_HIGH
		&& pval.intval == POWER_SUPPLY_TYPEC_SOURCE_DEFAULT) {

		chg->cc2_sink_detach_flag = CC2_SINK_WA_DONE;

		rc = smblib_masked_write(chg,
				TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				EXIT_SNK_BASED_ON_CC_BIT, 0);
		if (rc < 0)
			smblib_err(chg, "Couldn't get prop typec mode rc=%d\n",
				rc);
	}

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: debounce-done %s; Type-C %s detected\n",
		   rising ? "rising" : "falling",
		   smblib_typec_mode_name[pval.intval]);
}

#ifdef OPLUS_FEATURE_CHG_BASIC
irqreturn_t usbid_change_handler(int irq, void *data)
{
	struct oplus_chg_chip *chip = data;
	struct smb_charger *chg = &chip->pmic_spmi.smb2_chip->chg;
#if 0
	union power_supply_propval ret = {0,};

	chg->usb_psy_desc.get_property(chg->usb_psy, POWER_SUPPLY_PROP_OTG_SWITCH, &ret);
	if (ret.intval == false) {
		chg_err("usbid_change_handler, get_otg_switch == false!\n");
		return IRQ_HANDLED;
	}
#endif
	if (oplus_usbid_check_is_gpio(chip) == true) {
		if (chg->micro_usb_mode) {
			cancel_delayed_work_sync(&chg->uusb_otg_work);
			vote(chg->awake_votable, OTG_DELAY_VOTER, true, 0);
			smblib_dbg(chg, PR_INTERRUPT, "Scheduling OTG work\n");
			schedule_delayed_work(&chg->uusb_otg_work,
					msecs_to_jiffies(chg->otg_delay_ms));
			return IRQ_HANDLED;
		}
	} else {
		chg_err("usbid_change_handler, oplus_usbid_check_is_gpio == false!\n");
	}

	return IRQ_HANDLED;
}
#endif

irqreturn_t smblib_handle_usb_typec_change(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int rc;
	u8 stat4, stat5;
	bool debounce_done, sink_attached, legacy_cable;

	if (chg->micro_usb_mode) {
		cancel_delayed_work_sync(&chg->uusb_otg_work);
		vote(chg->awake_votable, OTG_DELAY_VOTER, true, 0);
		smblib_dbg(chg, PR_INTERRUPT, "Scheduling OTG work\n");
		schedule_delayed_work(&chg->uusb_otg_work,
				msecs_to_jiffies(chg->otg_delay_ms));
		return IRQ_HANDLED;
	}

	/* WA - not when PD hard_reset WIP on cc2 in sink mode */
	if (chg->cc2_sink_detach_flag == CC2_SINK_STD)
		return IRQ_HANDLED;

	rc = smblib_read(chg, TYPE_C_STATUS_4_REG, &stat4);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_4 rc=%d\n", rc);
		return IRQ_HANDLED;
	}

	rc = smblib_read(chg, TYPE_C_STATUS_5_REG, &stat5);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_5 rc=%d\n", rc);
		return IRQ_HANDLED;
	}

	debounce_done = (bool)(stat4 & TYPEC_DEBOUNCE_DONE_STATUS_BIT);
	sink_attached = (bool)(stat4 & UFP_DFP_MODE_STATUS_BIT);
	legacy_cable = (bool)(stat5 & TYPEC_LEGACY_CABLE_STATUS_BIT);

	smblib_handle_typec_debounce_done(chg,
			debounce_done, sink_attached, legacy_cable);

	if (stat4 & TYPEC_VBUS_ERROR_STATUS_BIT)
		smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s vbus-error\n",
			irq_data->name);

	if (stat4 & TYPEC_VCONN_OVERCURR_STATUS_BIT)
		schedule_work(&chg->vconn_oc_work);

	power_supply_changed(chg->usb_psy);
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_STATUS_4 = 0x%02x\n", stat4);
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_STATUS_5 = 0x%02x\n", stat5);
	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_dc_plugin(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

#ifdef OPLUS_FEATURE_CHG_BASIC
	if (chg->dc_psy)
#endif
	power_supply_changed(chg->dc_psy);
	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_high_duty_cycle(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	chg->is_hdc = true;
	schedule_delayed_work(&chg->clear_hdc_work, msecs_to_jiffies(60));

	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_switcher_power_ok(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int rc;
#ifdef OPLUS_FEATURE_CHG_BASIC
	int usb_icl;
#endif
	u8 stat;

	if (!(chg->wa_flags & BOOST_BACK_WA))
		return IRQ_HANDLED;

	rc = smblib_read(chg, POWER_PATH_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read POWER_PATH_STATUS rc=%d\n", rc);
		return IRQ_HANDLED;
	}

#ifndef OPLUS_FEATURE_CHG_BASIC
	if ((stat & USE_USBIN_BIT) &&
			get_effective_result(chg->usb_icl_votable) < USBIN_25MA)
		return IRQ_HANDLED;
#else
	/* skip suspending input if its already suspended by some other voter */
	usb_icl = get_effective_result(chg->usb_icl_votable);
	if ((stat & USE_USBIN_BIT) && usb_icl >= 0 && usb_icl < USBIN_25MA)
		return IRQ_HANDLED;
#endif /*OPLUS_FEATURE_CHG_BASIC*/

	if (stat & USE_DCIN_BIT)
		return IRQ_HANDLED;

	if (is_storming(&irq_data->storm_data)) {
#ifdef OPLUS_FEATURE_CHG_BASIC
		if (printk_ratelimit())
			smblib_err(chg, "Reverse boost detected: voting 0mA to suspend input\n");
		if (chg->usb_psy_desc.type != POWER_SUPPLY_TYPE_USB_CDP
				&& chg->usb_psy_desc.type != POWER_SUPPLY_TYPE_USB)
			vote(chg->usb_icl_votable, BOOST_BACK_VOTER, true, 0);
#else
		smblib_err(chg, "Reverse boost detected: voting 0mA to suspend input\n");
		vote(chg->usb_icl_votable, BOOST_BACK_VOTER, true, 0);
#endif
	}

	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_wdog_bark(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int rc;

	rc = smblib_write(chg, BARK_BITE_WDOG_PET_REG, BARK_BITE_WDOG_PET_BIT);
	if (rc < 0)
		smblib_err(chg, "Couldn't pet the dog rc=%d\n", rc);

	return IRQ_HANDLED;
}

/***************
 * Work Queues *
 ***************/
static void smblib_uusb_otg_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						uusb_otg_work.work);
	int rc;
	u8 stat;
	bool otg;
#ifdef OPLUS_FEATURE_CHG_BASIC
	static bool otg_status = 0;
#endif

#ifdef OPLUS_FEATURE_CHG_BASIC
	struct oplus_chg_chip *chip = g_oplus_chip;
	union power_supply_propval ret = {0,};

	chg->usb_psy_desc.get_property(chg->usb_psy, POWER_SUPPLY_PROP_OTG_SWITCH, &ret);

	if (oplus_usbid_check_is_gpio(chip) == true) {
		if (ret.intval == false)
			otg = 0;
		else
			otg = !gpio_get_value(chip->normalchg_gpio.usbid_gpio);
		/*the following 3 lines are just for compling*/
		rc = stat = 0;
		if (rc != 0)
			goto out;
	} else {
		rc = smblib_read(chg, TYPE_C_STATUS_3_REG, &stat);
		if (rc < 0) {
			smblib_err(chg, "Couldn't read TYPE_C_STATUS_3 rc=%d\n", rc);
			goto out;
		}

		otg = !!(stat & (U_USB_GND_NOVBUS_BIT | U_USB_GND_BIT));
	}
#else
	rc = smblib_read(chg, TYPE_C_STATUS_3_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_3 rc=%d\n", rc);
		goto out;
	}

	otg = !!(stat & (U_USB_GND_NOVBUS_BIT | U_USB_GND_BIT));
#endif

#ifdef OPLUS_FEATURE_CHG_BASIC
	if (otg_status ^ otg) {
		otg_status = otg;
		printk(KERN_ERR "!!!!! smblib_handle_usb_typec_change_for_uusb: [%d]\n", otg_status);
	}
#endif
	extcon_set_cable_state_(chg->extcon, EXTCON_USB_HOST, otg);
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_STATUS_3 = 0x%02x OTG=%d\n",
			stat, otg);
	power_supply_changed(chg->usb_psy);

out:
	vote(chg->awake_votable, OTG_DELAY_VOTER, false, 0);
}

static void smblib_hvdcp_detect_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
					       hvdcp_detect_work.work);

	vote(chg->pd_disallowed_votable_indirect, HVDCP_TIMEOUT_VOTER,
				false, 0);
	if (get_effective_result(chg->pd_disallowed_votable_indirect))
		/* pd is still disabled, try hvdcp */
		try_rerun_apsd_for_hvdcp(chg);
	else
		/* notify pd now that pd is allowed */
		power_supply_changed(chg->usb_psy);
}

static void bms_update_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						bms_update_work);

	smblib_suspend_on_debug_battery(chg);

	if (chg->batt_psy)
		power_supply_changed(chg->batt_psy);
}

static void step_soc_req_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						step_soc_req_work.work);
	union power_supply_propval pval = {0, };
	int rc;

	rc = smblib_get_prop_batt_capacity(chg, &pval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get batt capacity rc=%d\n", rc);
		return;
	}

	step_charge_soc_update(chg, pval.intval);
}

static void clear_hdc_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						clear_hdc_work.work);

	chg->is_hdc = 0;
}

static void rdstd_cc2_detach_work(struct work_struct *work)
{
	int rc;
	u8 stat;
	struct smb_irq_data irq_data = {NULL, "cc2-removal-workaround"};
	struct smb_charger *chg = container_of(work, struct smb_charger,
						rdstd_cc2_detach_work);

	/*
	 * WA steps -
	 * 1. Enable both UFP and DFP, wait for 10ms.
	 * 2. Disable DFP, wait for 30ms.
	 * 3. Removal detected if both TYPEC_DEBOUNCE_DONE_STATUS
	 *    and TIMER_STAGE bits are gone, otherwise repeat all by
	 *    work rescheduling.
	 * Note, work will be cancelled when pd_hard_reset is 0.
	 */

	rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 UFP_EN_CMD_BIT | DFP_EN_CMD_BIT,
				 UFP_EN_CMD_BIT | DFP_EN_CMD_BIT);
	if (rc < 0) {
		smblib_err(chg, "Couldn't write TYPE_C_CTRL_REG rc=%d\n", rc);
		return;
	}

	usleep_range(10000, 11000);

	rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 UFP_EN_CMD_BIT | DFP_EN_CMD_BIT,
				 UFP_EN_CMD_BIT);
	if (rc < 0) {
		smblib_err(chg, "Couldn't write TYPE_C_CTRL_REG rc=%d\n", rc);
		return;
	}

	usleep_range(30000, 31000);

	rc = smblib_read(chg, TYPE_C_STATUS_4_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_4 rc=%d\n",
			rc);
		return;
	}
	if (stat & TYPEC_DEBOUNCE_DONE_STATUS_BIT)
		goto rerun;

	rc = smblib_read(chg, TYPE_C_STATUS_5_REG, &stat);
	if (rc < 0) {
		smblib_err(chg,
			"Couldn't read TYPE_C_STATUS_5_REG rc=%d\n", rc);
		return;
	}
	if (stat & TIMER_STAGE_2_BIT)
		goto rerun;

	/* Bingo, cc2 removal detected */
	smblib_reg_block_restore(chg, cc2_detach_settings);
	chg->cc2_sink_detach_flag = CC2_SINK_WA_DONE;
	irq_data.parent_data = chg;
	smblib_handle_usb_typec_change(0, &irq_data);

	return;

rerun:
	schedule_work(&chg->rdstd_cc2_detach_work);
}

static void smblib_otg_oc_exit(struct smb_charger *chg, bool success)
{
	int rc;

	chg->otg_attempts = 0;
	if (!success) {
		smblib_err(chg, "OTG soft start failed\n");
		chg->otg_en = false;
	}

	smblib_dbg(chg, PR_OTG, "enabling VBUS < 1V check\n");
	rc = smblib_masked_write(chg, OTG_CFG_REG,
					QUICKSTART_OTG_FASTROLESWAP_BIT, 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't enable VBUS < 1V check rc=%d\n", rc);

	if (!chg->external_vconn && chg->vconn_en) {
		chg->vconn_attempts = 0;
		if (success) {
			rc = _smblib_vconn_regulator_enable(
							chg->vconn_vreg->rdev);
			if (rc < 0)
				smblib_err(chg, "Couldn't enable VCONN rc=%d\n",
									rc);
		} else {
			chg->vconn_en = false;
		}
	}
}

#define MAX_OC_FALLING_TRIES 10
static void smblib_otg_oc_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
								otg_oc_work);
	int rc, i;
	u8 stat;

	if (!chg->vbus_vreg || !chg->vbus_vreg->rdev)
		return;

	smblib_err(chg, "over-current detected on VBUS\n");
	mutex_lock(&chg->otg_oc_lock);
	if (!chg->otg_en)
		goto unlock;

	smblib_dbg(chg, PR_OTG, "disabling VBUS < 1V check\n");
	smblib_masked_write(chg, OTG_CFG_REG,
					QUICKSTART_OTG_FASTROLESWAP_BIT,
					QUICKSTART_OTG_FASTROLESWAP_BIT);

	/*
	 * If 500ms has passed and another over-current interrupt has not
	 * triggered then it is likely that the software based soft start was
	 * successful and the VBUS < 1V restriction should be re-enabled.
	 */
	schedule_delayed_work(&chg->otg_ss_done_work, msecs_to_jiffies(500));

	rc = _smblib_vbus_regulator_disable(chg->vbus_vreg->rdev);
	if (rc < 0) {
		smblib_err(chg, "Couldn't disable VBUS rc=%d\n", rc);
		goto unlock;
	}

	if (++chg->otg_attempts > OTG_MAX_ATTEMPTS) {
		cancel_delayed_work_sync(&chg->otg_ss_done_work);
		smblib_err(chg, "OTG failed to enable after %d attempts\n",
			   chg->otg_attempts - 1);
		smblib_otg_oc_exit(chg, false);
		goto unlock;
	}

	/*
	 * The real time status should go low within 10ms. Poll every 1-2ms to
	 * minimize the delay when re-enabling OTG.
	 */
	for (i = 0; i < MAX_OC_FALLING_TRIES; ++i) {
		usleep_range(1000, 2000);
		rc = smblib_read(chg, OTG_BASE + INT_RT_STS_OFFSET, &stat);
		if (rc >= 0 && !(stat & OTG_OVERCURRENT_RT_STS_BIT))
			break;
	}

	if (i >= MAX_OC_FALLING_TRIES) {
		cancel_delayed_work_sync(&chg->otg_ss_done_work);
		smblib_err(chg, "OTG OC did not fall after %dms\n",
						2 * MAX_OC_FALLING_TRIES);
		smblib_otg_oc_exit(chg, false);
		goto unlock;
	}

	smblib_dbg(chg, PR_OTG, "OTG OC fell after %dms\n", 2 * i + 1);
	rc = _smblib_vbus_regulator_enable(chg->vbus_vreg->rdev);
	if (rc < 0) {
		smblib_err(chg, "Couldn't enable VBUS rc=%d\n", rc);
		goto unlock;
	}

unlock:
	mutex_unlock(&chg->otg_oc_lock);
}

static void smblib_vconn_oc_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
								vconn_oc_work);
	int rc, i;
	u8 stat;

	if (chg->micro_usb_mode)
		return;

	smblib_err(chg, "over-current detected on VCONN\n");
	if (!chg->vconn_vreg || !chg->vconn_vreg->rdev)
		return;

	mutex_lock(&chg->otg_oc_lock);
	rc = _smblib_vconn_regulator_disable(chg->vconn_vreg->rdev);
	if (rc < 0) {
		smblib_err(chg, "Couldn't disable VCONN rc=%d\n", rc);
		goto unlock;
	}

	if (++chg->vconn_attempts > VCONN_MAX_ATTEMPTS) {
		smblib_err(chg, "VCONN failed to enable after %d attempts\n",
			   chg->otg_attempts - 1);
		chg->vconn_en = false;
		chg->vconn_attempts = 0;
		goto unlock;
	}

	/*
	 * The real time status should go low within 10ms. Poll every 1-2ms to
	 * minimize the delay when re-enabling OTG.
	 */
	for (i = 0; i < MAX_OC_FALLING_TRIES; ++i) {
		usleep_range(1000, 2000);
		rc = smblib_read(chg, TYPE_C_STATUS_4_REG, &stat);
		if (rc >= 0 && !(stat & TYPEC_VCONN_OVERCURR_STATUS_BIT))
			break;
	}

	if (i >= MAX_OC_FALLING_TRIES) {
		smblib_err(chg, "VCONN OC did not fall after %dms\n",
						2 * MAX_OC_FALLING_TRIES);
		chg->vconn_en = false;
		chg->vconn_attempts = 0;
		goto unlock;
	}

	smblib_dbg(chg, PR_OTG, "VCONN OC fell after %dms\n", 2 * i + 1);
	if (++chg->vconn_attempts > VCONN_MAX_ATTEMPTS) {
		smblib_err(chg, "VCONN failed to enable after %d attempts\n",
			   chg->vconn_attempts - 1);
		chg->vconn_en = false;
		goto unlock;
	}

	rc = _smblib_vconn_regulator_enable(chg->vconn_vreg->rdev);
	if (rc < 0) {
		smblib_err(chg, "Couldn't enable VCONN rc=%d\n", rc);
		goto unlock;
	}

unlock:
	mutex_unlock(&chg->otg_oc_lock);
}

static void smblib_otg_ss_done_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
							otg_ss_done_work.work);
	int rc;
	bool success = false;
	u8 stat;

	mutex_lock(&chg->otg_oc_lock);
	rc = smblib_read(chg, OTG_STATUS_REG, &stat);
	if (rc < 0)
		smblib_err(chg, "Couldn't read OTG status rc=%d\n", rc);
	else if (stat & BOOST_SOFTSTART_DONE_BIT)
		success = true;

	smblib_otg_oc_exit(chg, success);
	mutex_unlock(&chg->otg_oc_lock);
}

static void smblib_icl_change_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
							icl_change_work.work);
	int rc, settled_ua;

	rc = smblib_get_charge_param(chg, &chg->param.icl_stat, &settled_ua);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get ICL status rc=%d\n", rc);
		return;
	}

	power_supply_changed(chg->usb_main_psy);
	vote(chg->pl_enable_votable_indirect, USBIN_I_VOTER,
				settled_ua >= USB_WEAK_INPUT_UA, 0);

	smblib_dbg(chg, PR_INTERRUPT, "icl_settled=%d\n", settled_ua);
}

#ifdef OPLUS_FEATURE_CHG_BASIC
static int oplus_chg_get_fv_monitor(struct oplus_chg_chip *chip)
{
	int default_fv = chip->limits.temp_cold_vfloat_mv;

	if (!chip)
		return 0;

	switch(chip->tbatt_status) {
		case BATTERY_STATUS__INVALID:
		case BATTERY_STATUS__REMOVED:
		case BATTERY_STATUS__LOW_TEMP:
		case BATTERY_STATUS__HIGH_TEMP:
			break;
		case BATTERY_STATUS__COLD_TEMP:
			default_fv = chip->limits.temp_cold_vfloat_mv;
			break;
		case BATTERY_STATUS__LITTLE_COLD_TEMP:
			default_fv = chip->limits.temp_little_cold_vfloat_mv;
			break;
		case BATTERY_STATUS__COOL_TEMP:
			default_fv = chip->limits.temp_cool_vfloat_mv;
			break;
		case BATTERY_STATUS__LITTLE_COOL_TEMP:
			default_fv = chip->limits.temp_little_cool_vfloat_mv;
			break;
		case BATTERY_STATUS__NORMAL:
			if (oplus_vooc_get_fastchg_to_normal() && chip->charging_state != CHARGING_STATUS_FULL)
				default_fv = chip->limits.temp_normal_vfloat_mv_voocchg;
			else
				default_fv = chip->limits.temp_normal_vfloat_mv_normalchg;
			break;
		case BATTERY_STATUS__WARM_TEMP:
			default_fv = chip->limits.temp_warm_vfloat_mv;
			break;
		default:
			break;
	}
#ifdef CONFIG_OPLUS_SHORT_C_BATT_CHECK
	if (oplus_short_c_batt_is_prohibit_chg(chip) && default_fv > chip->limits.short_c_bat_vfloat_mv)
		default_fv = chip->limits.short_c_bat_vfloat_mv;
#endif
	return default_fv;
}

static int oplus_chg_get_vbatt_full_vol_sw(struct oplus_chg_chip *chip)
{
	int default_fv = chip->limits.cold_vfloat_sw_limit;

	if (!chip)
		return 0;

	switch(chip->tbatt_status) {
		case BATTERY_STATUS__INVALID:
		case BATTERY_STATUS__REMOVED:
		case BATTERY_STATUS__LOW_TEMP:
		case BATTERY_STATUS__HIGH_TEMP:
			break;
		case BATTERY_STATUS__COLD_TEMP:
			default_fv = chip->limits.cold_vfloat_sw_limit;
			break;
		case BATTERY_STATUS__LITTLE_COLD_TEMP:
			default_fv = chip->limits.little_cold_vfloat_sw_limit;
			break;
		case BATTERY_STATUS__COOL_TEMP:
			default_fv = chip->limits.cool_vfloat_sw_limit;
			break;
		case BATTERY_STATUS__LITTLE_COOL_TEMP:
			default_fv = chip->limits.little_cool_vfloat_sw_limit;
			break;
		case BATTERY_STATUS__NORMAL:
			default_fv = chip->limits.normal_vfloat_sw_limit;
			break;
		case BATTERY_STATUS__WARM_TEMP:
			default_fv = chip->limits.warm_vfloat_sw_limit;
			break;
		default:
			break;
	}
#ifdef CONFIG_OPLUS_SHORT_C_BATT_CHECK
	if (oplus_short_c_batt_is_prohibit_chg(chip) && default_fv > chip->limits.short_c_bat_vfloat_sw_limit)
		default_fv = chip->limits.short_c_bat_vfloat_sw_limit;
#endif
	return default_fv;
}

/* When charger voltage is setting to < 4.3V and then resume to 5V, cannot charge, so... */
static void oplus_chg_monitor_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
							chg_monitor_work.work);
	struct oplus_chg_chip *chip = g_oplus_chip;
	int boot_mode = get_boot_mode();
	static int counts = 0;
	int rechg_vol;
	int rc;
	u8 stat;

	if (!chip || !chip->charger_exist || !chip->batt_exist || !chip->mmi_chg) {
		counts = 0;
		goto rerun_work;
	}
	if (chg->usb_psy_desc.type == POWER_SUPPLY_TYPE_USB || chg->usb_psy_desc.type == POWER_SUPPLY_TYPE_USB_CDP)
		return;
	if (boot_mode == MSM_BOOT_MODE__RF || boot_mode == MSM_BOOT_MODE__WLAN)
		return;
#ifdef CONFIG_OPLUS_SHORT_C_BATT_CHECK
	if (oplus_short_c_batt_is_disable_rechg(chip)) {
		counts = 0;
		goto rerun_work;
	}
#endif
	if (oplus_vooc_get_fastchg_started() == true || chip->charger_volt < 4400) {
		counts = 0;
		goto rerun_work;
	}
	if (chip->tbatt_status == BATTERY_STATUS__COLD_TEMP)
		rechg_vol = oplus_chg_get_fv_monitor(chip) - 300;
	else if (chip->tbatt_status == BATTERY_STATUS__LITTLE_COLD_TEMP)
		rechg_vol = oplus_chg_get_fv_monitor(chip) - 200;
	else
		rechg_vol = oplus_chg_get_fv_monitor(chip) - 100;
	if ((chip->batt_volt > rechg_vol - 10) && chip->batt_full) {
		counts = 0;
		goto rerun_work;
	} else if (chip->batt_volt > oplus_chg_get_vbatt_full_vol_sw(chip) - 10) {
		counts = 0;
		goto rerun_work;
	}

	if (chip->icharging >= 0) {
		counts++;
	} else if (chip->icharging < 0 && (chip->icharging * -1) <= chip->limits.iterm_ma / 2) {
		counts++;
	} else {
		counts = 0;
	}
	if (counts > 10)
		counts = 10;

	if (counts >= (chip->batt_full ? 8 : 3)) {//because rechg counts=6
		rc = smblib_read(chg, BATTERY_CHARGER_STATUS_8_REG, &stat);
		if (rc < 0) {
			printk(KERN_ERR "oplus_chg_monitor_work: Couldn't get BATTERY_CHARGER_STATUS_8_REG status rc=%d\n", rc);
			goto rerun_work;
		}
		if (get_client_vote(chg->usb_icl_votable, BOOST_BACK_VOTER) == 0
				&& get_effective_result(chg->usb_icl_votable) < USBIN_25MA) {
			printk(KERN_ERR "oplus_chg_monitor_work: boost back\n");
			if (chg->wa_flags & BOOST_BACK_WA)
				vote(chg->usb_icl_votable, BOOST_BACK_VOTER, false, 0);
		}
		if (chip->charging_state == CHARGING_STATUS_FAIL) {//for TEMP > 55 or < -3
			counts = 0;
			goto rerun_work;
		}
		if (stat & PRE_TERM_BIT) {
			usb_online_status = true;
			printk(KERN_ERR "oplus_chg_monitor_work: PRE_TERM_BIT is set[0x%x], clear it\n", stat);
			rc = smblib_masked_write(chg, USBIN_CMD_IL_REG, USBIN_SUSPEND_BIT, 1);
			if (rc < 0) {
				printk(KERN_ERR "oplus_chg_monitor_work: Couldn't set USBIN_SUSPEND_BIT rc=%d\n", rc);
				goto rerun_work;
			}
			msleep(50);
			rc = smblib_masked_write(chg, USBIN_CMD_IL_REG, USBIN_SUSPEND_BIT, 0);
			if (rc < 0) {
				printk(KERN_ERR "oplus_chg_monitor_work: Couldn't clear USBIN_SUSPEND_BIT rc=%d\n", rc);
				goto rerun_work;
			}
			msleep(10);
			rc = smblib_masked_write(chg, CMD_HVDCP_2_REG, RESTART_AICL_BIT, RESTART_AICL_BIT);
			if (rc < 0) {
				printk(KERN_ERR "oplus_chg_monitor_work: Couldn't set RESTART_AICL_BIT rc=%d\n", rc);
				goto rerun_work;
			}
			printk(KERN_ERR "oplus_chg_monitor_work: ichg[%d], fv[%d]\n", chip->icharging, oplus_chg_get_fv_monitor(chip));
		}
		counts = 0;
	}

rerun_work:
	usb_online_status = false;
	schedule_delayed_work(&chg->chg_monitor_work, OPLUS_CHG_MONITOR_INTERVAL);
}
#endif /* OPLUS_FEATURE_CHG_BASIC */

static int smblib_create_votables(struct smb_charger *chg)
{
	int rc = 0;
	
	
	chg->fcc_votable = find_votable("FCC");
	if (!chg->fcc_votable) {
		rc = -EPROBE_DEFER;
		return rc;
	}

	chg->fv_votable = find_votable("FV");
	if (!chg->fv_votable) {
		rc = -EPROBE_DEFER;
		return rc;
	}

	chg->pl_disable_votable = find_votable("PL_DISABLE");
	if (!chg->pl_disable_votable) {
		rc = -EPROBE_DEFER;
		return rc;
	}
	vote(chg->pl_disable_votable, PL_INDIRECT_VOTER, true, 0);
	vote(chg->pl_disable_votable, PL_DELAY_HVDCP_VOTER, true, 0);

	chg->dc_suspend_votable = create_votable("DC_SUSPEND", VOTE_SET_ANY,
					smblib_dc_suspend_vote_callback,
					chg);
	if (IS_ERR(chg->dc_suspend_votable)) {
		rc = PTR_ERR(chg->dc_suspend_votable);
		return rc;
	}

	chg->usb_icl_votable = create_votable("USB_ICL", VOTE_MIN,
					smblib_usb_icl_vote_callback,
					chg);
	if (IS_ERR(chg->usb_icl_votable)) {
		rc = PTR_ERR(chg->usb_icl_votable);
		return rc;
	}

	chg->dc_icl_votable = create_votable("DC_ICL", VOTE_MIN,
					smblib_dc_icl_vote_callback,
					chg);
	if (IS_ERR(chg->dc_icl_votable)) {
		rc = PTR_ERR(chg->dc_icl_votable);
		return rc;
	}

	chg->pd_disallowed_votable_indirect
		= create_votable("PD_DISALLOWED_INDIRECT", VOTE_SET_ANY,
			smblib_pd_disallowed_votable_indirect_callback, chg);
	if (IS_ERR(chg->pd_disallowed_votable_indirect)) {
		rc = PTR_ERR(chg->pd_disallowed_votable_indirect);
		return rc;
	}

	chg->pd_allowed_votable = create_votable("PD_ALLOWED",
					VOTE_SET_ANY, NULL, NULL);
	if (IS_ERR(chg->pd_allowed_votable)) {
		rc = PTR_ERR(chg->pd_allowed_votable);
		return rc;
	}

	chg->awake_votable = create_votable("AWAKE", VOTE_SET_ANY,
					smblib_awake_vote_callback,
					chg);
	if (IS_ERR(chg->awake_votable)) {
		rc = PTR_ERR(chg->awake_votable);
		return rc;
	}

	chg->chg_disable_votable = create_votable("CHG_DISABLE", VOTE_SET_ANY,
					smblib_chg_disable_vote_callback,
					chg);
	if (IS_ERR(chg->chg_disable_votable)) {
		rc = PTR_ERR(chg->chg_disable_votable);
		return rc;
	}

	chg->pl_enable_votable_indirect = create_votable("PL_ENABLE_INDIRECT",
					VOTE_SET_ANY,
					smblib_pl_enable_indirect_vote_callback,
					chg);
	if (IS_ERR(chg->pl_enable_votable_indirect)) {
		rc = PTR_ERR(chg->pl_enable_votable_indirect);
		return rc;
	}

	chg->hvdcp_disable_votable_indirect = create_votable(
				"HVDCP_DISABLE_INDIRECT",
				VOTE_SET_ANY,
				smblib_hvdcp_disable_indirect_vote_callback,
				chg);
	if (IS_ERR(chg->hvdcp_disable_votable_indirect)) {
		rc = PTR_ERR(chg->hvdcp_disable_votable_indirect);
		return rc;
	}

	chg->hvdcp_enable_votable = create_votable("HVDCP_ENABLE",
					VOTE_SET_ANY,
					smblib_hvdcp_enable_vote_callback,
					chg);
	if (IS_ERR(chg->hvdcp_enable_votable)) {
		rc = PTR_ERR(chg->hvdcp_enable_votable);
		return rc;
	}

	chg->apsd_disable_votable = create_votable("APSD_DISABLE",
					VOTE_SET_ANY,
					smblib_apsd_disable_vote_callback,
					chg);
	if (IS_ERR(chg->apsd_disable_votable)) {
		rc = PTR_ERR(chg->apsd_disable_votable);
		return rc;
	}

	chg->hvdcp_hw_inov_dis_votable = create_votable("HVDCP_HW_INOV_DIS",
					VOTE_SET_ANY,
					smblib_hvdcp_hw_inov_dis_vote_callback,
					chg);
	if (IS_ERR(chg->hvdcp_hw_inov_dis_votable)) {
		rc = PTR_ERR(chg->hvdcp_hw_inov_dis_votable);
		return rc;
	}

	return rc;
}

static void smblib_destroy_votables(struct smb_charger *chg)
{
	if (chg->dc_suspend_votable)
		destroy_votable(chg->dc_suspend_votable);
	if (chg->usb_icl_votable)
		destroy_votable(chg->usb_icl_votable);
	if (chg->dc_icl_votable)
		destroy_votable(chg->dc_icl_votable);
	if (chg->pd_disallowed_votable_indirect)
		destroy_votable(chg->pd_disallowed_votable_indirect);
	if (chg->pd_allowed_votable)
		destroy_votable(chg->pd_allowed_votable);
	if (chg->awake_votable)
		destroy_votable(chg->awake_votable);
	if (chg->chg_disable_votable)
		destroy_votable(chg->chg_disable_votable);
	if (chg->pl_enable_votable_indirect)
		destroy_votable(chg->pl_enable_votable_indirect);
	if (chg->apsd_disable_votable)
		destroy_votable(chg->apsd_disable_votable);
	if (chg->hvdcp_hw_inov_dis_votable)
		destroy_votable(chg->hvdcp_hw_inov_dis_votable);
}

static void smblib_iio_deinit(struct smb_charger *chg)
{
	if (!IS_ERR_OR_NULL(chg->iio.temp_chan))
		iio_channel_release(chg->iio.temp_chan);
	if (!IS_ERR_OR_NULL(chg->iio.temp_max_chan))
		iio_channel_release(chg->iio.temp_max_chan);
	if (!IS_ERR_OR_NULL(chg->iio.usbin_i_chan))
		iio_channel_release(chg->iio.usbin_i_chan);
	if (!IS_ERR_OR_NULL(chg->iio.usbin_v_chan))
		iio_channel_release(chg->iio.usbin_v_chan);
	if (!IS_ERR_OR_NULL(chg->iio.batt_i_chan))
		iio_channel_release(chg->iio.batt_i_chan);
}

int smblib_init(struct smb_charger *chg)
{
	int rc = 0;

	mutex_init(&chg->write_lock);
	mutex_init(&chg->otg_oc_lock);
	INIT_WORK(&chg->bms_update_work, bms_update_work);
	INIT_WORK(&chg->rdstd_cc2_detach_work, rdstd_cc2_detach_work);
	INIT_DELAYED_WORK(&chg->hvdcp_detect_work, smblib_hvdcp_detect_work);
	INIT_DELAYED_WORK(&chg->step_soc_req_work, step_soc_req_work);
	INIT_DELAYED_WORK(&chg->clear_hdc_work, clear_hdc_work);
	INIT_WORK(&chg->otg_oc_work, smblib_otg_oc_work);
	INIT_WORK(&chg->vconn_oc_work, smblib_vconn_oc_work);
	INIT_DELAYED_WORK(&chg->otg_ss_done_work, smblib_otg_ss_done_work);
	INIT_DELAYED_WORK(&chg->icl_change_work, smblib_icl_change_work);
#ifdef OPLUS_FEATURE_CHG_BASIC
	INIT_DELAYED_WORK(&chg->chg_monitor_work, oplus_chg_monitor_work);
#endif
	INIT_DELAYED_WORK(&chg->uusb_otg_work, smblib_uusb_otg_work);
	chg->fake_capacity = -EINVAL;

	switch (chg->mode) {
	case PARALLEL_MASTER:
		chg->qnovo_fcc_ua = -EINVAL;
		chg->qnovo_fv_uv = -EINVAL;
		rc = smblib_create_votables(chg);
		if (rc < 0) {
			smblib_err(chg, "Couldn't create votables rc=%d\n",
				rc);
			return rc;
		}

		rc = smblib_register_notifier(chg);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't register notifier rc=%d\n", rc);
			return rc;
		}

		chg->bms_psy = power_supply_get_by_name("bms");
		chg->pl.psy = power_supply_get_by_name("parallel");
		break;
	case PARALLEL_SLAVE:
		break;
	default:
		smblib_err(chg, "Unsupported mode %d\n", chg->mode);
		return -EINVAL;
	}

	return rc;
}

int smblib_deinit(struct smb_charger *chg)
{
	switch (chg->mode) {
	case PARALLEL_MASTER:
		cancel_delayed_work_sync(&chg->uusb_otg_work);
		power_supply_unreg_notifier(&chg->nb);
		smblib_destroy_votables(chg);
		break;
	case PARALLEL_SLAVE:
		break;
	default:
		smblib_err(chg, "Unsupported mode %d\n", chg->mode);
		return -EINVAL;
	}

	smblib_iio_deinit(chg);

	return 0;
}

#ifndef OPLUS_FEATURE_CHG_BASIC
static int __debug_mask = PR_MISC | PR_OTG | PR_REGISTER;
#else
static int __debug_mask;
#endif
module_param_named(
	debug_mask, __debug_mask, int, S_IRUSR | S_IWUSR
);

#ifndef OPLUS_FEATURE_CHG_BASIC
#define MICRO_1P5A	1500000
#else
#define MICRO_1P5A	1000000
#endif
#define MICRO_P1A	100000
#define OTG_DEFAULT_DEGLITCH_TIME_MS	50
static int smb2_parse_dt(struct smb2 *chip)
{
	struct smb_charger *chg = &chip->chg;
	struct device_node *node = chg->dev->of_node;
	int rc, byte_len;

	if (!node) {
		pr_err("device tree node missing\n");
		return -EINVAL;
	}

	chg->step_chg_enabled = true;

	if (of_property_count_u32_elems(node, "qcom,step-soc-thresholds")
			!= STEP_CHARGING_MAX_STEPS - 1)
		chg->step_chg_enabled = false;

	rc = of_property_read_u32_array(node, "qcom,step-soc-thresholds",
			chip->dt.step_soc_threshold,
			STEP_CHARGING_MAX_STEPS - 1);
	if (rc < 0)
		chg->step_chg_enabled = false;

	if (of_property_count_u32_elems(node, "qcom,step-current-deltas")
			!= STEP_CHARGING_MAX_STEPS)
		chg->step_chg_enabled = false;

	rc = of_property_read_u32_array(node, "qcom,step-current-deltas",
			chip->dt.step_cc_delta,
			STEP_CHARGING_MAX_STEPS);
	if (rc < 0)
		chg->step_chg_enabled = false;

	chip->dt.no_battery = of_property_read_bool(node,
						"qcom,batteryless-platform");

	chg->external_vconn = of_property_read_bool(node,
						"qcom,external-vconn");

	rc = of_property_read_u32(node,
				"qcom,fcc-max-ua", &chip->dt.fcc_ua);
	if (rc < 0)
		chip->dt.fcc_ua = -EINVAL;

	rc = of_property_read_u32(node,
				"qcom,fv-max-uv", &chip->dt.fv_uv);
	if (rc < 0)
		chip->dt.fv_uv = -EINVAL;

	rc = of_property_read_u32(node,
				"qcom,usb-icl-ua", &chip->dt.usb_icl_ua);
	if (rc < 0)
		chip->dt.usb_icl_ua = -EINVAL;

#ifdef OPLUS_FEATURE_CHG_BASIC
	rc = of_property_read_u32(node,
				"qcom,otg-cl-ua", &chg->otg_cl_ua);
	if (rc < 0)
		chg->otg_cl_ua = MICRO_1P5A;
#else
	rc = of_property_read_u32(node,
				"qcom,otg-cl-ua", &chg->otg_cl_ua);
	if (rc < 0)
		chg->otg_cl_ua = MICRO_1P5A;
#endif

	rc = of_property_read_u32(node,
				"qcom,dc-icl-ua", &chip->dt.dc_icl_ua);
	if (rc < 0)
		chip->dt.dc_icl_ua = -EINVAL;

	rc = of_property_read_u32(node,
				"qcom,boost-threshold-ua",
				&chip->dt.boost_threshold_ua);
	if (rc < 0)
		chip->dt.boost_threshold_ua = MICRO_P1A;

	rc = of_property_read_u32(node, "qcom,wipower-max-uw",
				&chip->dt.wipower_max_uw);
	if (rc < 0)
		chip->dt.wipower_max_uw = -EINVAL;

	if (of_find_property(node, "qcom,thermal-mitigation", &byte_len)) {
		chg->thermal_mitigation = devm_kzalloc(chg->dev, byte_len,
			GFP_KERNEL);

		if (chg->thermal_mitigation == NULL)
			return -ENOMEM;

		chg->thermal_levels = byte_len / sizeof(u32);
		rc = of_property_read_u32_array(node,
				"qcom,thermal-mitigation",
				chg->thermal_mitigation,
				chg->thermal_levels);
		if (rc < 0) {
			dev_err(chg->dev,
				"Couldn't read threm limits rc = %d\n", rc);
			return rc;
		}
	}

	of_property_read_u32(node, "qcom,float-option", &chip->dt.float_option);
	if (chip->dt.float_option < 0 || chip->dt.float_option > 4) {
		pr_err("qcom,float-option is out of range [0, 4]\n");
		return -EINVAL;
	}

	chip->dt.hvdcp_disable = of_property_read_bool(node,
						"qcom,hvdcp-disable");

	of_property_read_u32(node, "qcom,chg-inhibit-threshold-mv",
				&chip->dt.chg_inhibit_thr_mv);
	if ((chip->dt.chg_inhibit_thr_mv < 0 ||
		chip->dt.chg_inhibit_thr_mv > 300)) {
		pr_err("qcom,chg-inhibit-threshold-mv is incorrect\n");
		return -EINVAL;
	}

	chip->dt.auto_recharge_soc = of_property_read_bool(node,
						"qcom,auto-recharge-soc");

	chg->micro_usb_mode = of_property_read_bool(node, "qcom,micro-usb");

#ifdef OPLUS_FEATURE_CHG_BASIC
	if (g_oplus_chip) {
		g_oplus_chip->normalchg_gpio.chargerid_switch_gpio = 
				of_get_named_gpio(node, "qcom,chargerid_switch-gpio", 0);
		if (g_oplus_chip->normalchg_gpio.chargerid_switch_gpio <= 0) {
			chg_err("Couldn't read chargerid_switch-gpio rc = %d, chargerid_switch_gpio:%d\n", 
					rc, g_oplus_chip->normalchg_gpio.chargerid_switch_gpio);
		} else {
			if (gpio_is_valid(g_oplus_chip->normalchg_gpio.chargerid_switch_gpio)) {
				rc = gpio_request(g_oplus_chip->normalchg_gpio.chargerid_switch_gpio, "charging-switch1-gpio");
				if (rc) {
					chg_err("unable to request chargerid_switch_gpio:%d\n", g_oplus_chip->normalchg_gpio.chargerid_switch_gpio);
				} else {
					smbchg_chargerid_switch_gpio_init(g_oplus_chip);
				}
			}
			chg_err("chargerid_switch_gpio:%d\n", g_oplus_chip->normalchg_gpio.chargerid_switch_gpio);
		}
	}
#endif /*OPLUS_FEATURE_CHG_BASIC*/

#ifdef OPLUS_FEATURE_CHG_BASIC
	if (!chg->pm660l_bob_reg && of_get_property(node, "pm660l_bob-supply", NULL)) {
		chg->pm660l_bob_reg = devm_regulator_get(chg->dev, "pm660l_bob");
		if (IS_ERR(chg->pm660l_bob_reg)) {
			chg_err("Couldn't get pm660l_bob regulator rc=%ld\n", PTR_ERR(chg->pm660l_bob_reg));
			chg->pm660l_bob_reg = NULL;
		}
	}
#endif /*OPLUS_FEATURE_CHG_BASIC*/

#ifdef OPLUS_FEATURE_CHG_BASIC
	if (g_oplus_chip) {
		g_oplus_chip->normalchg_gpio.usbid_gpio =
				of_get_named_gpio(node, "qcom,usbid-gpio", 0);
		if (g_oplus_chip->normalchg_gpio.usbid_gpio <= 0) {
			chg_err("Couldn't read qcom,usbid-gpio rc=%d, qcom,usbid-gpio:%d\n",
					rc, g_oplus_chip->normalchg_gpio.usbid_gpio);
		} else {
			if (oplus_usbid_check_is_gpio(g_oplus_chip) == true) {
				rc = gpio_request(g_oplus_chip->normalchg_gpio.usbid_gpio, "usbid-gpio");
				if (rc) {
					chg_err("unable to request usbid-gpio:%d\n",
							g_oplus_chip->normalchg_gpio.usbid_gpio);
				} else {
					oplus_usbid_gpio_init(g_oplus_chip);
					oplus_usbid_irq_init(g_oplus_chip);
				}
			}
			chg_err("usbid-gpio:%d\n", g_oplus_chip->normalchg_gpio.usbid_gpio);
		}
	}
#endif /*OPLUS_FEATURE_CHG_BASIC*/

#ifndef OPLUS_FEATURE_CHG_BASIC
	chg->dcp_icl_ua = chip->dt.usb_icl_ua;
#else
	chg->dcp_icl_ua = -EINVAL;
#endif

	chg->suspend_input_on_debug_batt = of_property_read_bool(node,
					"qcom,suspend-input-on-debug-batt");

	rc = of_property_read_u32(node, "qcom,otg-deglitch-time-ms",
					&chg->otg_delay_ms);
	if (rc < 0)
		chg->otg_delay_ms = OTG_DEFAULT_DEGLITCH_TIME_MS;

	return 0;
}

/************************
 * USB PSY REGISTRATION *
 ************************/

static enum power_supply_property smb2_usb_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_PD_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_TYPEC_MODE,
	POWER_SUPPLY_PROP_TYPEC_POWER_ROLE,
	POWER_SUPPLY_PROP_TYPEC_CC_ORIENTATION,
	POWER_SUPPLY_PROP_PD_ALLOWED,
	POWER_SUPPLY_PROP_PD_ACTIVE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED,
	POWER_SUPPLY_PROP_INPUT_CURRENT_NOW,
	POWER_SUPPLY_PROP_BOOST_CURRENT,
	POWER_SUPPLY_PROP_PE_START,
	POWER_SUPPLY_PROP_CTM_CURRENT_MAX,
/*oplus own usb props*/
        POWER_SUPPLY_PROP_OTG_SWITCH,
        POWER_SUPPLY_PROP_OTG_ONLINE,
};

#ifdef OPLUS_FEATURE_CHG_BASIC
bool __attribute__((weak)) oplus_get_otg_switch_status(void)
{
	return false;
}

bool __attribute__((weak)) oplus_get_otg_online_status(void)
{
	return false;
}

void __attribute__((weak)) oplus_set_otg_switch_status(bool value)
{
	printk(KERN_ERR "USE weak oplus_set_otg_switch_status\n");
}
#endif /* OPLUS_FEATURE_CHG_BASIC */
#ifdef OPLUS_FEATURE_CHG_BASIC
static bool use_present_status = false;
#endif

static int smb2_usb_get_prop(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct smb2 *chip = power_supply_get_drvdata(psy);
	struct smb_charger *chg = &chip->chg;
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		if (chip->bad_part)
			val->intval = 1;
		else
			rc = smblib_get_prop_usb_present(chg, val);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
#ifdef OPLUS_FEATURE_CHG_BASIC
		if (use_present_status)
			rc = smblib_get_prop_usb_present(chg, val);
		else
			rc = smblib_get_prop_usb_online(chg, val);
#else
		rc = smblib_get_prop_usb_online(chg, val);
#endif
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
		val->intval = chg->voltage_min_uv;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = chg->voltage_max_uv;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		rc = smblib_get_prop_usb_voltage_now(chg, val);
		break;
	case POWER_SUPPLY_PROP_PD_CURRENT_MAX:
		rc = smblib_get_prop_pd_current_max(chg, val);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		rc = smblib_get_prop_usb_current_max(chg, val);
		break;
	case POWER_SUPPLY_PROP_TYPE:
		if (chip->bad_part)
			val->intval = POWER_SUPPLY_TYPE_USB;
		else
			val->intval = chg->usb_psy_desc.type;
		break;
	case POWER_SUPPLY_PROP_TYPEC_MODE:
		if (chg->micro_usb_mode)
			val->intval = POWER_SUPPLY_TYPEC_NONE;
		else if (chip->bad_part)
			val->intval = POWER_SUPPLY_TYPEC_SOURCE_DEFAULT;
		else
			rc = smblib_get_prop_typec_mode(chg, val);
		break;
	case POWER_SUPPLY_PROP_TYPEC_POWER_ROLE:
		if (chg->micro_usb_mode)
			val->intval = POWER_SUPPLY_TYPEC_PR_NONE;
		else
			rc = smblib_get_prop_typec_power_role(chg, val);
		break;
	case POWER_SUPPLY_PROP_TYPEC_CC_ORIENTATION:
		if (chg->micro_usb_mode)
			val->intval = 0;
		else
			rc = smblib_get_prop_typec_cc_orientation(chg, val);
		break;
	case POWER_SUPPLY_PROP_PD_ALLOWED:
		rc = smblib_get_prop_pd_allowed(chg, val);
		break;
	case POWER_SUPPLY_PROP_PD_ACTIVE:
		val->intval = chg->pd_active;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED:
		rc = smblib_get_prop_input_current_settled(chg, val);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_NOW:
		rc = smblib_get_prop_usb_current_now(chg, val);
		break;
	case POWER_SUPPLY_PROP_BOOST_CURRENT:
		val->intval = chg->boost_current_ua;
		break;
	case POWER_SUPPLY_PROP_PD_IN_HARD_RESET:
		rc = smblib_get_prop_pd_in_hard_reset(chg, val);
		break;
	case POWER_SUPPLY_PROP_PD_USB_SUSPEND_SUPPORTED:
		val->intval = chg->system_suspend_supported;
		break;
	case POWER_SUPPLY_PROP_PE_START:
		rc = smblib_get_pe_start(chg, val);
		break;
	case POWER_SUPPLY_PROP_CTM_CURRENT_MAX:
		val->intval = get_client_vote(chg->usb_icl_votable, CTM_VOTER);
		break;
#ifdef OPLUS_FEATURE_CHG_BASIC
	case POWER_SUPPLY_PROP_OTG_SWITCH:
		val->intval = oplus_get_otg_switch_status();
		break;
	case POWER_SUPPLY_PROP_OTG_ONLINE:
		val->intval = oplus_get_otg_online_status();
		break;
#endif /*OPLUS_FEATURE_CHG_BASIC*/
	default:
        //rc = oplus_usb_get_property(psy, psp, val);
		pr_err("get prop %d is not supported in usb\n", psp);
		rc = -EINVAL;
		break;
	}
	if (rc < 0) {
		pr_debug("Couldn't get prop %d rc = %d\n", psp, rc);
		return -ENODATA;
	}
	return 0;
}

static int smb2_usb_set_prop(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{
	struct smb2 *chip = power_supply_get_drvdata(psy);
	struct smb_charger *chg = &chip->chg;
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
		rc = smblib_set_prop_usb_voltage_min(chg, val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		rc = smblib_set_prop_usb_voltage_max(chg, val);
		break;
	case POWER_SUPPLY_PROP_PD_CURRENT_MAX:
		rc = smblib_set_prop_pd_current_max(chg, val);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		rc = smblib_set_prop_usb_current_max(chg, val);
		break;
	case POWER_SUPPLY_PROP_TYPEC_POWER_ROLE:
		rc = smblib_set_prop_typec_power_role(chg, val);
		break;
	case POWER_SUPPLY_PROP_PD_ACTIVE:
		rc = smblib_set_prop_pd_active(chg, val);
		break;
	case POWER_SUPPLY_PROP_PD_IN_HARD_RESET:
		rc = smblib_set_prop_pd_in_hard_reset(chg, val);
		break;
	case POWER_SUPPLY_PROP_PD_USB_SUSPEND_SUPPORTED:
		chg->system_suspend_supported = val->intval;
		break;
	case POWER_SUPPLY_PROP_BOOST_CURRENT:
		rc = smblib_set_prop_boost_current(chg, val);
		break;
#ifdef OPLUS_FEATURE_CHG_BASIC
	case POWER_SUPPLY_PROP_OTG_SWITCH:
		oplus_set_otg_switch_status(!!val->intval);
		break;
#endif
	case POWER_SUPPLY_PROP_CTM_CURRENT_MAX:
		rc = vote(chg->usb_icl_votable, CTM_VOTER,
						val->intval >= 0, val->intval);
		break;
	default:
		pr_err("set prop %d is not supported\n", psp);
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int smb2_usb_prop_is_writeable(struct power_supply *psy,
		enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case POWER_SUPPLY_PROP_TYPEC_POWER_ROLE:
#ifdef OPLUS_FEATURE_CHG_BASIC
	case POWER_SUPPLY_PROP_OTG_SWITCH:
#endif
	case POWER_SUPPLY_PROP_CTM_CURRENT_MAX:
		return 1;
	default:
		break;
	}

	return 0;
}

static int smb2_init_usb_psy(struct smb2 *chip)
{
	struct power_supply_config usb_cfg = {};
	struct smb_charger *chg = &chip->chg;

	chg->usb_psy_desc.name			= "usb";
	chg->usb_psy_desc.type			= POWER_SUPPLY_TYPE_UNKNOWN;
	chg->usb_psy_desc.properties		= smb2_usb_props;
	chg->usb_psy_desc.num_properties	= ARRAY_SIZE(smb2_usb_props);
	chg->usb_psy_desc.get_property		= smb2_usb_get_prop;
	chg->usb_psy_desc.set_property		= smb2_usb_set_prop;
	chg->usb_psy_desc.property_is_writeable	= smb2_usb_prop_is_writeable;

	usb_cfg.drv_data = chip;
	usb_cfg.of_node = chg->dev->of_node;
	chg->usb_psy = devm_power_supply_register(chg->dev,
						  &chg->usb_psy_desc,
						  &usb_cfg);
	if (IS_ERR(chg->usb_psy)) {
		pr_err("Couldn't register USB power supply\n");
		return PTR_ERR(chg->usb_psy);
	}

	return 0;
}

/*****************************
 * USB MAIN PSY REGISTRATION *
 *****************************/

static enum power_supply_property smb2_usb_main_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_ICL_REDUCTION,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED,
	POWER_SUPPLY_PROP_INPUT_VOLTAGE_SETTLED,
	POWER_SUPPLY_PROP_FCC_DELTA,
	/*
	 * TODO move the TEMP and TEMP_MAX properties here,
	 * and update the thermal balancer to look here
	 */
};

static int smb2_usb_main_get_prop(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct smb2 *chip = power_supply_get_drvdata(psy);
	struct smb_charger *chg = &chip->chg;
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		rc = smblib_get_charge_param(chg, &chg->param.fv, &val->intval);
		break;
	case POWER_SUPPLY_PROP_ICL_REDUCTION:
		val->intval = chg->icl_reduction_ua;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		rc = smblib_get_charge_param(chg, &chg->param.fcc,
							&val->intval);
		break;
	case POWER_SUPPLY_PROP_TYPE:
		val->intval = POWER_SUPPLY_TYPE_MAIN;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED:
		rc = smblib_get_prop_input_current_settled(chg, val);
		break;
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_SETTLED:
		rc = smblib_get_prop_input_voltage_settled(chg, val);
		break;
	case POWER_SUPPLY_PROP_FCC_DELTA:
		rc = smblib_get_prop_fcc_delta(chg, val);
		break;
	default:
		pr_debug("get prop %d is not supported in usb-main\n", psp);
		rc = -EINVAL;
		break;
	}
	if (rc < 0) {
		pr_debug("Couldn't get prop %d rc = %d\n", psp, rc);
		return -ENODATA;
	}
	return 0;
}

static int smb2_usb_main_set_prop(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{
	struct smb2 *chip = power_supply_get_drvdata(psy);
	struct smb_charger *chg = &chip->chg;
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		rc = smblib_set_charge_param(chg, &chg->param.fv, val->intval);
		break;
	case POWER_SUPPLY_PROP_ICL_REDUCTION:
		rc = smblib_set_icl_reduction(chg, val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		rc = smblib_set_charge_param(chg, &chg->param.fcc, val->intval);
		break;
	default:
		pr_err("set prop %d is not supported\n", psp);
		rc = -EINVAL;
		break;
	}

	return rc;
}

static const struct power_supply_desc usb_main_psy_desc = {
	.name		= "main",
	.type		= POWER_SUPPLY_TYPE_MAIN,
	.properties	= smb2_usb_main_props,
	.num_properties	= ARRAY_SIZE(smb2_usb_main_props),
	.get_property	= smb2_usb_main_get_prop,
	.set_property	= smb2_usb_main_set_prop,
};

static int smb2_init_usb_main_psy(struct smb2 *chip)
{
	struct power_supply_config usb_main_cfg = {};
	struct smb_charger *chg = &chip->chg;

	usb_main_cfg.drv_data = chip;
	usb_main_cfg.of_node = chg->dev->of_node;
	chg->usb_main_psy = devm_power_supply_register(chg->dev,
						  &usb_main_psy_desc,
						  &usb_main_cfg);
	if (IS_ERR(chg->usb_main_psy)) {
		pr_err("Couldn't register USB main power supply\n");
		return PTR_ERR(chg->usb_main_psy);
	}

	return 0;
}

/*************************
 * DC PSY REGISTRATION   *
 *************************/

#ifndef OPLUS_FEATURE_CHG_BASIC
static enum power_supply_property smb2_dc_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static int smb2_dc_get_prop(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct smb2 *chip = power_supply_get_drvdata(psy);
	struct smb_charger *chg = &chip->chg;
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		rc = smblib_get_prop_dc_present(chg, val);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		rc = smblib_get_prop_dc_online(chg, val);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		rc = smblib_get_prop_dc_current_max(chg, val);
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

static int smb2_dc_set_prop(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{
	struct smb2 *chip = power_supply_get_drvdata(psy);
	struct smb_charger *chg = &chip->chg;
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		rc = smblib_set_prop_dc_current_max(chg, val);
		break;
	default:
		return -EINVAL;
	}

	return rc;
}

static int smb2_dc_prop_is_writeable(struct power_supply *psy,
		enum power_supply_property psp)
{
	int rc;

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		rc = 1;
		break;
	default:
		rc = 0;
		break;
	}

	return rc;
}

static const struct power_supply_desc dc_psy_desc = {
	.name = "dc",
	.type = POWER_SUPPLY_TYPE_WIPOWER,
	.properties = smb2_dc_props,
	.num_properties = ARRAY_SIZE(smb2_dc_props),
	.get_property = smb2_dc_get_prop,
	.set_property = smb2_dc_set_prop,
	.property_is_writeable = smb2_dc_prop_is_writeable,
};

static int smb2_init_dc_psy(struct smb2 *chip)
{
	struct power_supply_config dc_cfg = {};
	struct smb_charger *chg = &chip->chg;

	dc_cfg.drv_data = chip;
	dc_cfg.of_node = chg->dev->of_node;
	chg->dc_psy = devm_power_supply_register(chg->dev,
						  &dc_psy_desc,
						  &dc_cfg);
	if (IS_ERR(chg->dc_psy)) {
		pr_err("Couldn't register USB power supply\n");
		return PTR_ERR(chg->dc_psy);
	}

	return 0;
}
#endif /* OPLUS_FEATURE_CHG_BASIC */

#ifdef OPLUS_FEATURE_CHG_BASIC
/*

static enum power_supply_property platform_ac_props[] = {
};

static enum power_supply_property platform_batt_props[] = {
	POWER_SUPPLY_PROP_INPUT_SUSPEND,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGER_TEMP,
	POWER_SUPPLY_PROP_CHARGER_TEMP_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMITED,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_QNOVO,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_QNOVO,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_STEP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_SW_JEITA_ENABLED,
	POWER_SUPPLY_PROP_CHARGE_DONE,
	POWER_SUPPLY_PROP_PARALLEL_DISABLE,
	POWER_SUPPLY_PROP_SET_SHIP_MODE,
	POWER_SUPPLY_PROP_DIE_HEALTH,
	POWER_SUPPLY_PROP_RERUN_AICL,
	POWER_SUPPLY_PROP_DP_DM,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
};

static enum power_supply_property platform_usb_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_PD_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_TYPEC_MODE,
	POWER_SUPPLY_PROP_TYPEC_POWER_ROLE,
	POWER_SUPPLY_PROP_TYPEC_CC_ORIENTATION,
	POWER_SUPPLY_PROP_PD_ALLOWED,
	POWER_SUPPLY_PROP_PD_ACTIVE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED,
	POWER_SUPPLY_PROP_INPUT_CURRENT_NOW,
	POWER_SUPPLY_PROP_BOOST_CURRENT,
	POWER_SUPPLY_PROP_PE_START,
	POWER_SUPPLY_PROP_CTM_CURRENT_MAX,
	POWER_SUPPLY_PROP_HW_CURRENT_MAX,
	POWER_SUPPLY_PROP_REAL_TYPE,
	POWER_SUPPLY_PROP_PR_SWAP,
	POWER_SUPPLY_PROP_PD_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_PD_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_SDP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONNECTOR_TYPE,
	POWER_SUPPLY_PROP_MOISTURE_DETECTED,
};



#define PSPNUM_PLAFORM_AC_PROPS     ARRAY_SIZE(platform_ac_props);
#define PSPNUM_OPLUS_AC_PROPS        ARRAY_SIZE(oplus_ac_props);


static enum power_supply_property ac_props[ARRAY_SIZE(platform_ac_props) + ARRAY_SIZE(oplus_ac_props)] = {0};
static enum power_supply_property battery_props[ARRAY_SIZE(platform_batt_props) + ARRAY_SIZE(oplus_batt_props)] = {0};
static enum power_supply_property usb_props[ARRAY_SIZE(platform_usb_props) + ARRAY_SIZE(oplus_usb_props)] = {0};




void power_supply_property_merge(power_supply_property platform_props[], power_supply_property oplus_props[]) {
    int i;
    for(i = 0; i < ARRAY_SIZE(platform_props); i++) {
        ac_props[i] = platform_props[i];
    }
    for(i = 0; i < ARRAY_SIZE(oplus_props); i++) {
        ac_props[ARRAY_SIZE(platform_props) + i]= oplus_props[i];
    }
}
*/
 static enum power_supply_property ac_props[] = {
/*oplus own ac props*/    
        POWER_SUPPLY_PROP_ONLINE,
};

static int smb2_ac_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	int rc = 0;

    rc = oplus_ac_get_property(psy, psp, val);

	return rc;
}

static const struct power_supply_desc ac_psy_desc = {
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = ac_props,
	.num_properties = ARRAY_SIZE(ac_props),
	.get_property = smb2_ac_get_property,
};

/*
void oplus_chg_ac_psy_init(void)
{
		chip->ac_psd.name = "ac";
		chip->ac_psd.type = POWER_SUPPLY_TYPE_MAINS;
		chip->ac_psd.properties = ac_props;
		chip->ac_psd.num_properties = ARRAY_SIZE(ac_props);
		chip->ac_psd.get_property = ac_get_property;
		chip->ac_cfg.drv_data = chip->dev;
}
*/

static int smb2_init_ac_psy(struct smb2 *chip)
{
	struct power_supply_config ac_cfg = {};
	struct smb_charger *chg = &chip->chg;

	ac_cfg.drv_data = chip;
	ac_cfg.of_node = chg->dev->of_node;
    //power_supply_property_merge(platform_ac_props[],oplus_ac_props[]);
 //   g_oplus_chip->ac_cfg = ac_cfg;
//	g_oplus_chip->ac_psd = ac_psy_desc;
	
	chg->ac_psy = devm_power_supply_register(chg->dev,
						  &ac_psy_desc,
						  &ac_cfg);
	if (IS_ERR(chg->ac_psy)) {
		pr_err("Couldn't register AC power supply\n");
		return PTR_ERR(chg->ac_psy);
	}

	return 0;
}

static enum power_supply_property smb2_batt_props[] = {
	POWER_SUPPLY_PROP_INPUT_SUSPEND,
//	POWER_SUPPLY_PROP_STATUS,
//	POWER_SUPPLY_PROP_HEALTH,
//	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
//	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL,
	POWER_SUPPLY_PROP_CHARGER_TEMP,
	POWER_SUPPLY_PROP_CHARGER_TEMP_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMITED,
//	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
//	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
//	POWER_SUPPLY_PROP_TEMP,
//	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_STEP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_STEP_CHARGING_STEP,
	POWER_SUPPLY_PROP_CHARGE_DONE,
	POWER_SUPPLY_PROP_PARALLEL_DISABLE,
	POWER_SUPPLY_PROP_SET_SHIP_MODE,
	POWER_SUPPLY_PROP_DIE_HEALTH,
	POWER_SUPPLY_PROP_RERUN_AICL,
	POWER_SUPPLY_PROP_DP_DM,
#ifdef OPLUS_FEATURE_CHG_BASIC
/*oplus own battery props*/
    POWER_SUPPLY_PROP_STATUS,
    POWER_SUPPLY_PROP_HEALTH,
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_TECHNOLOGY,
    POWER_SUPPLY_PROP_CAPACITY,
    POWER_SUPPLY_PROP_TEMP,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_CURRENT_NOW,

	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_AUTHENTICATE,
	POWER_SUPPLY_PROP_CHARGE_TIMEOUT,
    POWER_SUPPLY_PROP_BATTERY_REQUEST_POWEROFF,	
	POWER_SUPPLY_PROP_CHARGE_TECHNOLOGY,
	POWER_SUPPLY_PROP_FAST_CHARGE,
	POWER_SUPPLY_PROP_MMI_CHARGING_ENABLE,
	POWER_SUPPLY_PROP_BATTERY_FCC,
	POWER_SUPPLY_PROP_BATTERY_SOH,
	POWER_SUPPLY_PROP_BATTERY_CC,
	POWER_SUPPLY_PROP_BATTERY_RM,
	POWER_SUPPLY_PROP_BATTERY_NOTIFY_CODE,
	POWER_SUPPLY_PROP_ADAPTER_FW_UPDATE,
	POWER_SUPPLY_PROP_VOOCCHG_ING,

/*CTS*/	
    POWER_SUPPLY_PROP_CHARGE_COUNTER,
    POWER_SUPPLY_PROP_CURRENT_MAX,

#if defined(CONFIG_OPLUS_CHARGER_MTK6763) || defined(CONFIG_OPLUS_CHARGER_MTK6771)
    POWER_SUPPLY_PROP_STOP_CHARGING_ENABLE,
    POWER_SUPPLY_PROP_adjust_power,
#endif
#if defined(CONFIG_OPLUS_CHARGER_MTK6771)
            POWER_SUPPLY_PROP_CHARGE_COUNTER,
            POWER_SUPPLY_PROP_CURRENT_MAX,
#endif
#ifdef CONFIG_OPLUS_CHECK_CHARGERID_VOLT
	POWER_SUPPLY_PROP_CHARGERID_VOLT,
#endif
#ifdef CONFIG_OPLUS_SHIP_MODE_SUPPORT
	POWER_SUPPLY_PROP_SHIP_MODE,
#endif
#ifdef CONFIG_OPLUS_SHORT_C_BATT_CHECK
	POWER_SUPPLY_PROP_SHORT_C_BATT_UPDATE_CHANGE,
	POWER_SUPPLY_PROP_SHORT_C_BATT_IN_IDLE,
	POWER_SUPPLY_PROP_SHORT_C_BATT_CV_STATUS,
#endif
#ifdef CONFIG_OPLUS_SHORT_HW_CHECK
	POWER_SUPPLY_PROP_SHORT_C_HW_FEATURE,
	POWER_SUPPLY_PROP_SHORT_C_HW_STATUS,
#endif
#endif
};


static int smb2_batt_get_prop(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct smb_charger *chg = power_supply_get_drvdata(psy);
	int rc = 0;

	switch (psp) {



	case POWER_SUPPLY_PROP_INPUT_SUSPEND:
		rc = smblib_get_prop_input_suspend(chg, val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		rc = smblib_get_prop_batt_charge_type(chg, val);
		break;


	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		rc = smblib_get_prop_system_temp_level(chg, val);
		break;
#ifndef OPLUS_FEATURE_CHG_BASIC
/* CHARGER_TEMP and CHARGER_TEMP_MAX is dependent on FG and only for HVDCP */
	case POWER_SUPPLY_PROP_CHARGER_TEMP:
		/* do not query RRADC if charger is not present */
		rc = smblib_get_prop_usb_present(chg, &pval);
		if (rc < 0)
			pr_err("Couldn't get usb present rc=%d\n", rc);

		rc = -ENODATA;
		if (pval.intval)
			rc = smblib_get_prop_charger_temp(chg, val);
		break;
	case POWER_SUPPLY_PROP_CHARGER_TEMP_MAX:
		rc = smblib_get_prop_charger_temp_max(chg, val);
		break;
#else
	case POWER_SUPPLY_PROP_CHARGER_TEMP:
	case POWER_SUPPLY_PROP_CHARGER_TEMP_MAX:
		val->intval = -1;
		break;
#endif
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMITED:
		rc = smblib_get_prop_input_current_limited(chg, val);
		break;
	case POWER_SUPPLY_PROP_STEP_CHARGING_ENABLED:
		val->intval = chg->step_chg_enabled;
		break;
	case POWER_SUPPLY_PROP_STEP_CHARGING_STEP:
		rc = smblib_get_prop_step_chg_step(chg, val);
		break;



	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = get_client_vote(chg->fv_votable, DEFAULT_VOTER);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_QNOVO:
		val->intval = chg->qnovo_fv_uv;
		break;

		break;
	case POWER_SUPPLY_PROP_CURRENT_QNOVO:
		val->intval = chg->qnovo_fcc_ua;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = get_client_vote(chg->fcc_votable,
					      DEFAULT_VOTER);
		break;


		break;
	case POWER_SUPPLY_PROP_CHARGE_DONE:
		rc = smblib_get_prop_batt_charge_done(chg, val);
		break;
	case POWER_SUPPLY_PROP_PARALLEL_DISABLE:
		val->intval = get_client_vote(chg->pl_disable_votable,
					      USER_VOTER);
		break;
	case POWER_SUPPLY_PROP_SET_SHIP_MODE:
		/* Not in ship mode as long as device is active */
		val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_DIE_HEALTH:
		rc = smblib_get_prop_die_health(chg, val);
		break;
	case POWER_SUPPLY_PROP_DP_DM:
		val->intval = chg->pulse_cnt;
		break;
	case POWER_SUPPLY_PROP_RERUN_AICL:
		val->intval = 0;
		break;
	default:
        rc = oplus_battery_get_property(psy, psp, val);
	}

	if (rc < 0) {
		pr_debug("Couldn't get prop %d rc = %d\n", psp, rc);
		return -ENODATA;
	}

	return 0;
}

static int smb2_batt_set_prop(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *val)
{
	int rc = 0;
	struct smb_charger *chg = power_supply_get_drvdata(psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_SUSPEND:
		rc = smblib_set_prop_input_suspend(chg, val);
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		rc = smblib_set_prop_system_temp_level(chg, val);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		rc = smblib_set_prop_batt_capacity(chg, val);
		break;
	case POWER_SUPPLY_PROP_PARALLEL_DISABLE:
		vote(chg->pl_disable_votable, USER_VOTER, (bool)val->intval, 0);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		vote(chg->fv_votable, DEFAULT_VOTER, true, val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_QNOVO:
		chg->qnovo_fv_uv = val->intval;
		rc = rerun_election(chg->fv_votable);
		break;
	case POWER_SUPPLY_PROP_CURRENT_QNOVO:
		chg->qnovo_fcc_ua = val->intval;
		rc = rerun_election(chg->fcc_votable);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		vote(chg->fcc_votable, DEFAULT_VOTER, true, val->intval);
		break;
	case POWER_SUPPLY_PROP_SET_SHIP_MODE:
		/* Not in ship mode as long as the device is active */
		if (!val->intval)
			break;
		if (chg->pl.psy)
			power_supply_set_property(chg->pl.psy,
				POWER_SUPPLY_PROP_SET_SHIP_MODE, val);
		rc = smblib_set_prop_ship_mode(chg, val);
		break;
	case POWER_SUPPLY_PROP_RERUN_AICL:
		rc = smblib_rerun_aicl(chg);
		break;
	case POWER_SUPPLY_PROP_DP_DM:
		rc = smblib_dp_dm(chg, val->intval);
		break;
	default:
        rc = oplus_battery_set_property(psy, prop, val);
	}

	return rc;
}

static int smb2_batt_prop_is_writeable(struct power_supply *psy,
		enum power_supply_property psp)
{
	int rc = 0;
    
	switch (psp) {
	case POWER_SUPPLY_PROP_INPUT_SUSPEND:
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_PARALLEL_DISABLE:
	case POWER_SUPPLY_PROP_DP_DM:
	case POWER_SUPPLY_PROP_RERUN_AICL:
		return 1;
	default:
        rc = oplus_battery_property_is_writeable(psy, psp);
		break;
	}

	return rc;
}

static const struct power_supply_desc batt_psy_desc = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = smb2_batt_props,
	.num_properties = ARRAY_SIZE(smb2_batt_props),
	.get_property = smb2_batt_get_prop,
	.set_property = smb2_batt_set_prop,
	.property_is_writeable = smb2_batt_prop_is_writeable,
};

static int smb2_init_batt_psy(struct smb2 *chip)
{
	struct power_supply_config batt_cfg = {};
	struct smb_charger *chg = &chip->chg;
	int rc = 0;

	batt_cfg.drv_data = chg;
	batt_cfg.of_node = chg->dev->of_node;

   // g_oplus_chip->battery_psd= batt_psy_desc;
    //g_oplus_chip->battery_cfg = battery_cfg;

	
	chg->batt_psy = devm_power_supply_register(chg->dev,
						   &batt_psy_desc,
						   &batt_cfg);
	if (IS_ERR(chg->batt_psy)) {
		pr_err("Couldn't register battery power supply\n");
		return PTR_ERR(chg->batt_psy);
	}

	return rc;
}

static int oplus_power_supply_init(struct smb2 *chip)
{
    int rc = 0;
    
    rc = smb2_init_ac_psy(chip);
    if (rc < 0) {
        pr_err("Couldn't initialize ac psy rc=%d\n", rc);
        return rc;
    }
//kong
    rc = smb2_init_batt_psy(chip);
    if (rc < 0) {
        pr_err("Couldn't initialize batt psy rc=%d\n", rc);
        return rc;
    }

//kong
    rc = smb2_init_usb_psy(chip);
    if (rc < 0) {
        pr_err("Couldn't initialize usb psy rc=%d\n", rc);
       return rc;
    }
    
    return rc;
}
#endif /* OPLUS_FEATURE_CHG_BASIC */


/******************************
 * VBUS REGULATOR REGISTRATION *
 ******************************/

struct regulator_ops smb2_vbus_reg_ops = {
	.enable = smblib_vbus_regulator_enable,
	.disable = smblib_vbus_regulator_disable,
	.is_enabled = smblib_vbus_regulator_is_enabled,
};

static int smb2_init_vbus_regulator(struct smb2 *chip)
{
	struct smb_charger *chg = &chip->chg;
	struct regulator_config cfg = {};
	int rc = 0;

	chg->vbus_vreg = devm_kzalloc(chg->dev, sizeof(*chg->vbus_vreg),
				      GFP_KERNEL);
	if (!chg->vbus_vreg)
		return -ENOMEM;

	cfg.dev = chg->dev;
	cfg.driver_data = chip;

	chg->vbus_vreg->rdesc.owner = THIS_MODULE;
	chg->vbus_vreg->rdesc.type = REGULATOR_VOLTAGE;
	chg->vbus_vreg->rdesc.ops = &smb2_vbus_reg_ops;
	chg->vbus_vreg->rdesc.of_match = "qcom,smb2-vbus";
	chg->vbus_vreg->rdesc.name = "qcom,smb2-vbus";

	chg->vbus_vreg->rdev = devm_regulator_register(chg->dev,
						&chg->vbus_vreg->rdesc, &cfg);
	if (IS_ERR(chg->vbus_vreg->rdev)) {
		rc = PTR_ERR(chg->vbus_vreg->rdev);
		chg->vbus_vreg->rdev = NULL;
		if (rc != -EPROBE_DEFER)
			pr_err("Couldn't register VBUS regualtor rc=%d\n", rc);
	}

	return rc;
}

/******************************
 * VCONN REGULATOR REGISTRATION *
 ******************************/

struct regulator_ops smb2_vconn_reg_ops = {
	.enable = smblib_vconn_regulator_enable,
	.disable = smblib_vconn_regulator_disable,
	.is_enabled = smblib_vconn_regulator_is_enabled,
};

static int smb2_init_vconn_regulator(struct smb2 *chip)
{
	struct smb_charger *chg = &chip->chg;
	struct regulator_config cfg = {};
	int rc = 0;

	if (chg->micro_usb_mode)
		return 0;

	chg->vconn_vreg = devm_kzalloc(chg->dev, sizeof(*chg->vconn_vreg),
				      GFP_KERNEL);
	if (!chg->vconn_vreg)
		return -ENOMEM;

	cfg.dev = chg->dev;
	cfg.driver_data = chip;

	chg->vconn_vreg->rdesc.owner = THIS_MODULE;
	chg->vconn_vreg->rdesc.type = REGULATOR_VOLTAGE;
	chg->vconn_vreg->rdesc.ops = &smb2_vconn_reg_ops;
	chg->vconn_vreg->rdesc.of_match = "qcom,smb2-vconn";
	chg->vconn_vreg->rdesc.name = "qcom,smb2-vconn";

	chg->vconn_vreg->rdev = devm_regulator_register(chg->dev,
						&chg->vconn_vreg->rdesc, &cfg);
	if (IS_ERR(chg->vconn_vreg->rdev)) {
		rc = PTR_ERR(chg->vconn_vreg->rdev);
		chg->vconn_vreg->rdev = NULL;
		if (rc != -EPROBE_DEFER)
			pr_err("Couldn't register VCONN regualtor rc=%d\n", rc);
	}

	return rc;
}

/***************************
 * HARDWARE INITIALIZATION *
 ***************************/
static int smb2_config_step_charging(struct smb2 *chip)
{
	struct smb_charger *chg = &chip->chg;
	int rc = 0;
	int i;

	if (!chg->step_chg_enabled)
		return rc;

	for (i = 0; i < STEP_CHARGING_MAX_STEPS - 1; i++) {
		rc = smblib_set_charge_param(chg,
					     &chg->param.step_soc_threshold[i],
					     chip->dt.step_soc_threshold[i]);
		if (rc < 0) {
			pr_err("Couldn't configure soc thresholds rc = %d\n",
				rc);
			goto err_out;
		}
	}

	for (i = 0; i < STEP_CHARGING_MAX_STEPS; i++) {
		rc = smblib_set_charge_param(chg, &chg->param.step_cc_delta[i],
					     chip->dt.step_cc_delta[i]);
		if (rc < 0) {
			pr_err("Couldn't configure cc delta rc = %d\n",
				rc);
			goto err_out;
		}
	}

	rc = smblib_write(chg, STEP_CHG_UPDATE_REQUEST_TIMEOUT_CFG_REG,
			  STEP_CHG_UPDATE_REQUEST_TIMEOUT_40S);
	if (rc < 0) {
		dev_err(chg->dev,
			"Couldn't configure soc request timeout reg rc=%d\n",
			 rc);
		goto err_out;
	}

	rc = smblib_write(chg, STEP_CHG_UPDATE_FAIL_TIMEOUT_CFG_REG,
			  STEP_CHG_UPDATE_FAIL_TIMEOUT_120S);
	if (rc < 0) {
		dev_err(chg->dev,
			"Couldn't configure soc fail timeout reg rc=%d\n",
			rc);
		goto err_out;
	}

	/*
	 *  enable step charging, source soc, standard mode, go to final
	 *  state in case of failure.
	 */
	rc = smblib_write(chg, CHGR_STEP_CHG_MODE_CFG_REG,
			       STEP_CHARGING_ENABLE_BIT |
			       STEP_CHARGING_SOURCE_SELECT_BIT |
			       STEP_CHARGING_SOC_FAIL_OPTION_BIT);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't configure charger rc=%d\n", rc);
		goto err_out;
	}

	return 0;
err_out:
	chg->step_chg_enabled = false;
	return rc;
}

static int smb2_config_wipower_input_power(struct smb2 *chip, int uw)
{
	int rc;
	int ua;
	struct smb_charger *chg = &chip->chg;
	s64 nw = (s64)uw * 1000;

	if (uw < 0)
		return 0;

	ua = div_s64(nw, ZIN_ICL_PT_MAX_MV);
	rc = smblib_set_charge_param(chg, &chg->param.dc_icl_pt_lv, ua);
	if (rc < 0) {
		pr_err("Couldn't configure dc_icl_pt_lv rc = %d\n", rc);
		return rc;
	}

	ua = div_s64(nw, ZIN_ICL_PT_HV_MAX_MV);
	rc = smblib_set_charge_param(chg, &chg->param.dc_icl_pt_hv, ua);
	if (rc < 0) {
		pr_err("Couldn't configure dc_icl_pt_hv rc = %d\n", rc);
		return rc;
	}

	ua = div_s64(nw, ZIN_ICL_LV_MAX_MV);
	rc = smblib_set_charge_param(chg, &chg->param.dc_icl_div2_lv, ua);
	if (rc < 0) {
		pr_err("Couldn't configure dc_icl_div2_lv rc = %d\n", rc);
		return rc;
	}

	ua = div_s64(nw, ZIN_ICL_MID_LV_MAX_MV);
	rc = smblib_set_charge_param(chg, &chg->param.dc_icl_div2_mid_lv, ua);
	if (rc < 0) {
		pr_err("Couldn't configure dc_icl_div2_mid_lv rc = %d\n", rc);
		return rc;
	}

	ua = div_s64(nw, ZIN_ICL_MID_HV_MAX_MV);
	rc = smblib_set_charge_param(chg, &chg->param.dc_icl_div2_mid_hv, ua);
	if (rc < 0) {
		pr_err("Couldn't configure dc_icl_div2_mid_hv rc = %d\n", rc);
		return rc;
	}

	ua = div_s64(nw, ZIN_ICL_HV_MAX_MV);
	rc = smblib_set_charge_param(chg, &chg->param.dc_icl_div2_hv, ua);
	if (rc < 0) {
		pr_err("Couldn't configure dc_icl_div2_hv rc = %d\n", rc);
		return rc;
	}

	return 0;
}

static int smb2_configure_typec(struct smb_charger *chg)
{
	int rc;

	/*
	 * trigger the usb-typec-change interrupt only when the CC state
	 * changes
	 */
	rc = smblib_write(chg, TYPE_C_INTRPT_ENB_REG,
			  TYPEC_CCSTATE_CHANGE_INT_EN_BIT);
	if (rc < 0) {
		dev_err(chg->dev,
			"Couldn't configure Type-C interrupts rc=%d\n", rc);
		return rc;
	}

	/* configure power role for dual-role */
	rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 TYPEC_POWER_ROLE_CMD_MASK, 0);
	if (rc < 0) {
		dev_err(chg->dev,
			"Couldn't configure power role for DRP rc=%d\n", rc);
		return rc;
	}

	/*
	 * disable Type-C factory mode and stay in Attached.SRC state when VCONN
	 * over-current happens
	 */
	rc = smblib_masked_write(chg, TYPE_C_CFG_REG,
			FACTORY_MODE_DETECTION_EN_BIT | VCONN_OC_CFG_BIT, 0);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't configure Type-C rc=%d\n", rc);
		return rc;
	}

	/* increase VCONN softstart */
	rc = smblib_masked_write(chg, TYPE_C_CFG_2_REG,
			VCONN_SOFTSTART_CFG_MASK, VCONN_SOFTSTART_CFG_MASK);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't increase VCONN softstart rc=%d\n",
			rc);
		return rc;
	}

	/* disable try.SINK mode */
	rc = smblib_masked_write(chg, TYPE_C_CFG_3_REG, EN_TRYSINK_MODE_BIT, 0);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't set TRYSINK_MODE rc=%d\n", rc);
		return rc;
	}

	return rc;
}

#ifdef OPLUS_FEATURE_CHG_BASIC
static int smb2_disable_typec(struct smb_charger *chg)
{
	int rc;

	/* Move to typeC mode */
	/* configure FSM in idle state and disable UFP_ENABLE bit */
	rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
			TYPEC_DISABLE_CMD_BIT | UFP_EN_CMD_BIT,
			TYPEC_DISABLE_CMD_BIT);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't put FSM in idle rc=%d\n", rc);
		return rc;
	}

	/* wait for FSM to enter idle state */
	msleep(200);
	/* configure TypeC mode */
	rc = smblib_masked_write(chg, TYPE_C_CFG_REG,
			TYPE_C_OR_U_USB_BIT, 0);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't enable micro USB mode rc=%d\n", rc);
		return rc;
	}

	/* wait for mode change before enabling FSM */
	usleep_range(10000, 11000);
	/* release FSM from idle state */
	rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
			TYPEC_DISABLE_CMD_BIT, 0);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't release FSM rc=%d\n", rc);
		return rc;
	}

	/* wait for FSM to start */
	msleep(100);
	/* move to uUSB mode */
	/* configure FSM in idle state */
	rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
			TYPEC_DISABLE_CMD_BIT, TYPEC_DISABLE_CMD_BIT);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't put FSM in idle rc=%d\n", rc);
		return rc;
	}

	/* wait for FSM to enter idle state */
	msleep(200);
	/* configure micro USB mode */
	rc = smblib_masked_write(chg, TYPE_C_CFG_REG,
			TYPE_C_OR_U_USB_BIT, TYPE_C_OR_U_USB_BIT);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't enable micro USB mode rc=%d\n", rc);
		return rc;
	}

	/* wait for mode change before enabling FSM */
	usleep_range(10000, 11000);
	/* release FSM from idle state */
	rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
			TYPEC_DISABLE_CMD_BIT, 0);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't release FSM rc=%d\n", rc);
		return rc;
	}

	return rc;
}
#endif /* OPLUS_FEATURE_CHG_BASIC */

#ifdef OPLUS_FEATURE_CHG_BASIC
static void otg_enable_pmic_id_value (void)
{
	int rc;
	u8 stat;
	struct smb_charger *chg = NULL;

	if (!g_oplus_chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: smb2_chg not ready!\n", __func__);
		return;
	}
	chg = &g_oplus_chip->pmic_spmi.smb2_chip->chg;

	/* ENABLE TYPE-C MODULE */
	rc = smblib_masked_write(chg, 0x1368, 0x01, 0x0);//bit[0]=0
	if (rc < 0) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: Couldn't clear 0x1368[0] rc=%d\n", __func__, rc);
	}

	rc = smblib_read(chg, 0x1368, &stat);
	if (rc < 0) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: Couldn't read 0x1368 rc=%d\n", __func__, rc);
	} else {
		printk(KERN_ERR "[OPLUS_CHG][%s]: reg0x1368[0x%x], bit[0]=0\n", __func__, stat);
	}
}

static void otg_disable_pmic_id_value (void)
{
	int rc;
	u8 stat;
	struct smb_charger *chg = NULL;

	if (!g_oplus_chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: smb2_chg not ready!\n", __func__);
		return;
	}
	chg = &g_oplus_chip->pmic_spmi.smb2_chip->chg;

	/* DISABLE TYPE-C MODULE */
	rc = smblib_masked_write(chg, 0x1368, 0x01, 0x1);//bit[0]=1
	if (rc < 0) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: Couldn't set 0x1368[0] rc=%d\n", __func__, rc);
	}

	rc = smblib_read(chg, 0x1368, &stat);
	if (rc < 0) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: Couldn't read 0x1368 rc=%d\n", __func__, rc);
	} else {
		printk(KERN_ERR "[OPLUS_CHG][%s]: reg0x1368[0x%x], bit[0]=1\n", __func__, stat);
	}

	/* go to type-c mode */
	rc = smblib_masked_write(chg, 0x1358, 0x01, 0x0);//bit[0]=0
	if (rc < 0) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: Couldn't clear 0x1358[0] rc=%d\n", __func__, rc);
	}

	rc = smblib_read(chg, 0x1358, &stat);
	if (rc < 0) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: Couldn't read 0x1358 rc=%d\n", __func__, rc);
	} else {
		printk(KERN_ERR "[OPLUS_CHG][%s]: reg0x1358[0x%x], bit[0]=0\n", __func__, stat);
	}

	/* wait for mode change */
	usleep_range(5000, 5100);
	/* go to micro USB mode */
	rc = smblib_masked_write(chg, 0x1358, 0x01, 0x1);//bit[0]=1
	if (rc < 0) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: Couldn't set 0x1358[0] rc=%d\n", __func__, rc);
	}

	rc = smblib_read(chg, 0x1358, &stat);
	if (rc < 0) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: Couldn't read 0x1358 rc=%d\n", __func__, rc);
	} else {
		printk(KERN_ERR "[OPLUS_CHG][%s]: reg0x1358[0x%x], bit[0]=1\n", __func__, stat);
	}
}

void otg_enable_id_value (void)
{
	if (oplus_usbid_check_is_gpio(g_oplus_chip) == true) {
		oplus_set_usbid_active(g_oplus_chip);
		usbid_change_handler(0, g_oplus_chip);
		printk(KERN_ERR "[OPLUS_CHG][%s]: usbid_gpio=%d\n",
				__func__, gpio_get_value(g_oplus_chip->normalchg_gpio.usbid_gpio));
	} else {
		otg_enable_pmic_id_value();
	}
}

void otg_disable_id_value (void)
{
	if (oplus_usbid_check_is_gpio(g_oplus_chip) == true) {
		oplus_set_usbid_sleep(g_oplus_chip);
		usbid_change_handler(0, g_oplus_chip);
		printk(KERN_ERR "[OPLUS_CHG][%s]: usbid_gpio=%d\n",
				__func__, gpio_get_value(g_oplus_chip->normalchg_gpio.usbid_gpio));
	} else {
		otg_disable_pmic_id_value();
	}
}
#endif /* OPLUS_FEATURE_CHG_BASIC */

static int smb2_init_hw(struct smb2 *chip)
{
	struct smb_charger *chg = &chip->chg;
	int rc;
	u8 stat;

	if (chip->dt.no_battery)
		chg->fake_capacity = 50;

	if (chip->dt.fcc_ua < 0)
		smblib_get_charge_param(chg, &chg->param.fcc, &chip->dt.fcc_ua);

	if (chip->dt.fv_uv < 0)
		smblib_get_charge_param(chg, &chg->param.fv, &chip->dt.fv_uv);

	smblib_get_charge_param(chg, &chg->param.usb_icl,
				&chg->default_icl_ua);
	if (chip->dt.usb_icl_ua < 0)
		chip->dt.usb_icl_ua = chg->default_icl_ua;

	if (chip->dt.dc_icl_ua < 0)
		smblib_get_charge_param(chg, &chg->param.dc_icl,
					&chip->dt.dc_icl_ua);

	/* set a slower soft start setting for OTG */
	rc = smblib_masked_write(chg, DC_ENG_SSUPPLY_CFG2_REG,
				ENG_SSUPPLY_IVREF_OTG_SS_MASK, OTG_SS_SLOW);
	if (rc < 0) {
		pr_err("Couldn't set otg soft start rc=%d\n", rc);
		return rc;
	}

	/* set OTG current limit */
#ifdef OPLUS_FEATURE_CHG_BASIC
	rc = smblib_set_charge_param(chg, &chg->param.otg_cl,
				(chg->wa_flags & OTG_WA) ?
				chg->param.otg_cl.min_u : chg->otg_cl_ua);
#else
	rc = smblib_set_charge_param(chg, &chg->param.otg_cl,
				(chg->wa_flags & OTG_WA) ?
				chg->param.otg_cl.min_u : chg->otg_cl_ua);
#endif
	if (rc < 0) {
		pr_err("Couldn't set otg current limit rc=%d\n", rc);
		return rc;
	}

#ifdef OPLUS_FEATURE_CHG_BASIC
	smblib_masked_write(chg, BAT_UVLO_THRESHOLD_CFG_REG, BAT_UVLO_THRESHOLD_MASK, 0x3);
#endif

	chg->boost_threshold_ua = chip->dt.boost_threshold_ua;

	rc = smblib_read(chg, APSD_RESULT_STATUS_REG, &stat);
	if (rc < 0) {
		pr_err("Couldn't read APSD_RESULT_STATUS rc=%d\n", rc);
		return rc;
	}

	smblib_rerun_apsd_if_required(chg);

	/* clear the ICL override if it is set */
	if (smblib_icl_override(chg, false) < 0) {
		pr_err("Couldn't disable ICL override rc=%d\n", rc);
		return rc;
	}

	/* votes must be cast before configuring software control */
	/* vote 0mA on usb_icl for non battery platforms */
	vote(chg->usb_icl_votable,
		DEFAULT_VOTER, chip->dt.no_battery, 0);
	vote(chg->dc_suspend_votable,
		DEFAULT_VOTER, chip->dt.no_battery, 0);
	vote(chg->fcc_votable,
		DEFAULT_VOTER, true, chip->dt.fcc_ua);
	vote(chg->fv_votable,
		DEFAULT_VOTER, true, chip->dt.fv_uv);
	vote(chg->dc_icl_votable,
		DEFAULT_VOTER, true, chip->dt.dc_icl_ua);
	vote(chg->hvdcp_disable_votable_indirect, PD_INACTIVE_VOTER,
			true, 0);
	vote(chg->hvdcp_disable_votable_indirect, DEFAULT_VOTER,
		chip->dt.hvdcp_disable, 0);
	vote(chg->pd_disallowed_votable_indirect, CC_DETACHED_VOTER,
			true, 0);
	vote(chg->pd_disallowed_votable_indirect, HVDCP_TIMEOUT_VOTER,
			true, 0);
	vote(chg->pd_disallowed_votable_indirect, MICRO_USB_VOTER,
			chg->micro_usb_mode, 0);
#ifndef OPLUS_FEATURE_CHG_BASIC
	vote(chg->hvdcp_enable_votable, MICRO_USB_VOTER,
			chg->micro_usb_mode, 0);
#else
	vote(chg->hvdcp_enable_votable, MICRO_USB_VOTER,
			false, 0);
#endif

	/*
	 * AICL configuration:
	 * start from min and AICL ADC disable
	 */
#ifdef OPLUS_FEATURE_CHG_BASIC
	rc = smblib_masked_write(chg, USBIN_AICL_OPTIONS_CFG_REG,
			SUSPEND_ON_COLLAPSE_USBIN_BIT | USBIN_AICL_START_AT_MAX_BIT
				| USBIN_AICL_ADC_EN_BIT | USBIN_AICL_RERUN_EN_BIT, USBIN_AICL_RERUN_EN_BIT);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't configure AICL rc=%d\n", rc);
		return rc;
	}
#else
	rc = smblib_masked_write(chg, USBIN_AICL_OPTIONS_CFG_REG,
			USBIN_AICL_START_AT_MAX_BIT
				| USBIN_AICL_ADC_EN_BIT, 0);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't configure AICL rc=%d\n", rc);
		return rc;
	}
#endif /* OPLUS_FEATURE_CHG_BASIC */

	/* Configure charge enable for software control; active high */
	rc = smblib_masked_write(chg, CHGR_CFG2_REG,
				 CHG_EN_POLARITY_BIT |
				 CHG_EN_SRC_BIT, 0);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't configure charger rc=%d\n", rc);
		return rc;
	}

	/* enable the charging path */
	rc = vote(chg->chg_disable_votable, DEFAULT_VOTER, false, 0);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't enable charging rc=%d\n", rc);
		return rc;
	}

#ifdef OPLUS_FEATURE_CHG_BASIC
	if (chg->micro_usb_mode)
		rc = smb2_disable_typec(chg);
	else
		rc = smb2_configure_typec(chg);
	if (rc < 0) {
		dev_err(chg->dev,
			"Couldn't configure Type-C interrupts rc=%d\n", rc);
		return rc;
	}
#else
	if (!chg->micro_usb_mode) {
		rc = smb2_configure_typec(chg);
		if (rc < 0) {
			dev_err(chg->dev,
				"Couldn't configure Type-C rc=%d\n", rc);
			return rc;
		}
	}
#endif /* OPLUS_FEATURE_CHG_BASIC */

	/* configure VCONN for software control */
	rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 VCONN_EN_SRC_BIT | VCONN_EN_VALUE_BIT,
				 VCONN_EN_SRC_BIT);
	if (rc < 0) {
		dev_err(chg->dev,
			"Couldn't configure VCONN for SW control rc=%d\n", rc);
		return rc;
	}

	/* configure VBUS for software control */
	rc = smblib_masked_write(chg, OTG_CFG_REG, OTG_EN_SRC_CFG_BIT, 0);
	if (rc < 0) {
		dev_err(chg->dev,
			"Couldn't configure VBUS for SW control rc=%d\n", rc);
		return rc;
	}

#ifndef OPLUS_FEATURE_CHG_BASIC
	rc = smblib_masked_write(chg, QNOVO_PT_ENABLE_CMD_REG,
			QNOVO_PT_ENABLE_CMD_BIT, QNOVO_PT_ENABLE_CMD_BIT);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't enable qnovo rc=%d\n", rc);
		return rc;
	}
#else
	rc = smblib_masked_write(chg, QNOVO_PT_ENABLE_CMD_REG,
			QNOVO_PT_ENABLE_CMD_BIT, 0);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't enable qnovo rc=%d\n", rc);
		return rc;
	}
#endif

	/* configure step charging */
	rc = smb2_config_step_charging(chip);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't configure step charging rc=%d\n",
			rc);
		return rc;
	}

	/* configure wipower watts */
	rc = smb2_config_wipower_input_power(chip, chip->dt.wipower_max_uw);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't configure wipower rc=%d\n", rc);
		return rc;
	}

	/* disable SW STAT override */
	rc = smblib_masked_write(chg, STAT_CFG_REG,
				 STAT_SW_OVERRIDE_CFG_BIT, 0);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't disable SW STAT override rc=%d\n",
			rc);
		return rc;
	}

#ifdef OPLUS_FEATURE_CHG_BASIC
	/* disable FG default iterm */
	rc = smblib_masked_write(chg, FG_UPDATE_CFG_2_SEL_REG,
				IBT_LT_CHG_TERM_THRESH_SEL_BIT, 1);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't disable FG iterm override rc=%d\n",
			rc);
	}

	rc = smblib_masked_write(chg, CHGR_CFG2_REG, I_TERM_BIT, 1);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't disable PM660 iterm override rc=%d\n",
			rc);
	}
#endif

#ifdef OPLUS_FEATURE_CHG_BASIC
	/* disable FG ESR for GPS, NOTE: this is not final solution */
	rc = smblib_write(chg, 0x4169, 0x0);
#endif

#ifdef OPLUS_FEATURE_CHG_BASIC
	smblib_masked_write(chg, 0x1380, 0x03, 0x3);
	smblib_masked_write(chg, 0x1365, 0x03, 0x3);
#endif

#ifdef OPLUS_FEATURE_CHG_BASIC
	smblib_masked_write(chg, 0x1363, 0x20, 0);
#endif

#ifdef OPLUS_FEATURE_CHG_BASIC
	smblib_masked_write(chg, 0x1052, 0x02, 0);
	smblib_masked_write(chg, 0x1053, 0x40, 0);
#endif

#ifdef OPLUS_FEATURE_CHG_BASIC
	smblib_masked_write(chg, 0x1670, 0xff, 0);
#endif

#ifdef OPLUS_FEATURE_CHG_BASIC
	otg_disable_pmic_id_value();
#endif

	/* configure float charger options */
	switch (chip->dt.float_option) {
	case 1:
		rc = smblib_masked_write(chg, USBIN_OPTIONS_2_CFG_REG,
				FLOAT_OPTIONS_MASK, 0);
		break;
	case 2:
		rc = smblib_masked_write(chg, USBIN_OPTIONS_2_CFG_REG,
				FLOAT_OPTIONS_MASK, FORCE_FLOAT_SDP_CFG_BIT);
		break;
	case 3:
		rc = smblib_masked_write(chg, USBIN_OPTIONS_2_CFG_REG,
				FLOAT_OPTIONS_MASK, FLOAT_DIS_CHGING_CFG_BIT);
		break;
	case 4:
		rc = smblib_masked_write(chg, USBIN_OPTIONS_2_CFG_REG,
				FLOAT_OPTIONS_MASK, SUSPEND_FLOAT_CFG_BIT);
		break;
	default:
		rc = 0;
		break;
	}

	if (rc < 0) {
		dev_err(chg->dev, "Couldn't configure float charger options rc=%d\n",
			rc);
		return rc;
	}

	switch (chip->dt.chg_inhibit_thr_mv) {
	case 50:
		rc = smblib_masked_write(chg, CHARGE_INHIBIT_THRESHOLD_CFG_REG,
				CHARGE_INHIBIT_THRESHOLD_MASK,
				CHARGE_INHIBIT_THRESHOLD_50MV);
		break;
	case 100:
		rc = smblib_masked_write(chg, CHARGE_INHIBIT_THRESHOLD_CFG_REG,
				CHARGE_INHIBIT_THRESHOLD_MASK,
				CHARGE_INHIBIT_THRESHOLD_100MV);
		break;
	case 200:
		rc = smblib_masked_write(chg, CHARGE_INHIBIT_THRESHOLD_CFG_REG,
				CHARGE_INHIBIT_THRESHOLD_MASK,
				CHARGE_INHIBIT_THRESHOLD_200MV);
		break;
	case 300:
		rc = smblib_masked_write(chg, CHARGE_INHIBIT_THRESHOLD_CFG_REG,
				CHARGE_INHIBIT_THRESHOLD_MASK,
				CHARGE_INHIBIT_THRESHOLD_300MV);
		break;
	case 0:
		rc = smblib_masked_write(chg, CHGR_CFG2_REG,
				CHARGER_INHIBIT_BIT, 0);
	default:
		break;
	}

	if (rc < 0) {
		dev_err(chg->dev, "Couldn't configure charge inhibit threshold rc=%d\n",
			rc);
		return rc;
	}

	if (chip->dt.auto_recharge_soc) {
		rc = smblib_masked_write(chg, FG_UPDATE_CFG_2_SEL_REG,
				SOC_LT_CHG_RECHARGE_THRESH_SEL_BIT |
				VBT_LT_CHG_RECHARGE_THRESH_SEL_BIT,
				VBT_LT_CHG_RECHARGE_THRESH_SEL_BIT);
		if (rc < 0) {
			dev_err(chg->dev, "Couldn't configure FG_UPDATE_CFG2_SEL_REG rc=%d\n",
				rc);
			return rc;
		}
	} else {
		rc = smblib_masked_write(chg, FG_UPDATE_CFG_2_SEL_REG,
				SOC_LT_CHG_RECHARGE_THRESH_SEL_BIT |
				VBT_LT_CHG_RECHARGE_THRESH_SEL_BIT,
				SOC_LT_CHG_RECHARGE_THRESH_SEL_BIT);
		if (rc < 0) {
			dev_err(chg->dev, "Couldn't configure FG_UPDATE_CFG2_SEL_REG rc=%d\n",
				rc);
			return rc;
		}
	}

	return rc;
}

static int smb2_chg_config_init(struct smb2 *chip)
{
	struct smb_charger *chg = &chip->chg;
	struct pmic_revid_data *pmic_rev_id;
	struct device_node *revid_dev_node;

	revid_dev_node = of_parse_phandle(chip->chg.dev->of_node,
					  "qcom,pmic-revid", 0);
	if (!revid_dev_node) {
		pr_err("Missing qcom,pmic-revid property\n");
		return -EINVAL;
	}

	pmic_rev_id = get_revid_data(revid_dev_node);
	if (IS_ERR_OR_NULL(pmic_rev_id)) {
		/*
		 * the revid peripheral must be registered, any failure
		 * here only indicates that the rev-id module has not
		 * probed yet.
		 */
		return -EPROBE_DEFER;
	}

	switch (pmic_rev_id->pmic_subtype) {
	case PMI8998_SUBTYPE:
		chip->chg.smb_version = PMI8998_SUBTYPE;
		chip->chg.wa_flags |= BOOST_BACK_WA | QC_AUTH_INTERRUPT_WA_BIT;
		if (pmic_rev_id->rev4 == PMI8998_V1P1_REV4) /* PMI rev 1.1 */
			chg->wa_flags |= QC_CHARGER_DETECTION_WA_BIT;
		if (pmic_rev_id->rev4 == PMI8998_V2P0_REV4) /* PMI rev 2.0 */
			chg->wa_flags |= TYPEC_CC2_REMOVAL_WA_BIT;
		chg->chg_freq.freq_5V		= 600;
		chg->chg_freq.freq_6V_8V	= 800;
		chg->chg_freq.freq_9V		= 1000;
		chg->chg_freq.freq_12V		= 1200;
		chg->chg_freq.freq_removal	= 1000;
		chg->chg_freq.freq_below_otg_threshold = 2000;
		chg->chg_freq.freq_above_otg_threshold = 800;
		break;
	case PM660_SUBTYPE:
		chip->chg.smb_version = PM660_SUBTYPE;
		chip->chg.wa_flags |= BOOST_BACK_WA | OTG_WA;
		chg->param.freq_buck = pm660_params.freq_buck;
		chg->param.freq_boost = pm660_params.freq_boost;
		chg->chg_freq.freq_5V		= 600;
		chg->chg_freq.freq_6V_8V	= 800;
		chg->chg_freq.freq_9V		= 1050;
		chg->chg_freq.freq_12V		= 1200;
		chg->chg_freq.freq_removal	= 1050;
		chg->chg_freq.freq_below_otg_threshold = 1600;
		chg->chg_freq.freq_above_otg_threshold = 800;
		break;
	default:
		pr_err("PMIC subtype %d not supported\n",
				pmic_rev_id->pmic_subtype);
		return -EINVAL;
	}

	return 0;
}

/****************************
 * DETERMINE INITIAL STATUS *
 ****************************/

static int smb2_determine_initial_status(struct smb2 *chip)
{
	struct smb_irq_data irq_data = {chip, "determine-initial-status"};
	struct smb_charger *chg = &chip->chg;

	if (chg->bms_psy)
		smblib_suspend_on_debug_battery(chg);
	smblib_handle_usb_plugin(0, &irq_data);
#ifdef OPLUS_FEATURE_CHG_BASIC
	if (oplus_usbid_check_is_gpio(g_oplus_chip) == false)
		smblib_handle_usb_typec_change(0, &irq_data);
#else
	smblib_handle_usb_typec_change(0, &irq_data);
#endif
	smblib_handle_usb_source_change(0, &irq_data);
	smblib_handle_chg_state_change(0, &irq_data);
	smblib_handle_icl_change(0, &irq_data);
	smblib_handle_step_chg_state_change(0, &irq_data);
	smblib_handle_step_chg_soc_update_request(0, &irq_data);

	return 0;
}

/**************************
 * INTERRUPT REGISTRATION *
 **************************/

static struct smb_irq_info smb2_irqs[] = {
/* CHARGER IRQs */
	[CHG_ERROR_IRQ] = {
		.name		= "chg-error",
		.handler	= smblib_handle_debug,
	},
	[CHG_STATE_CHANGE_IRQ] = {
		.name		= "chg-state-change",
		.handler	= smblib_handle_chg_state_change,
		.wake		= true,
	},
	[STEP_CHG_STATE_CHANGE_IRQ] = {
		.name		= "step-chg-state-change",
		.handler	= smblib_handle_step_chg_state_change,
		.wake		= true,
	},
	[STEP_CHG_SOC_UPDATE_FAIL_IRQ] = {
		.name		= "step-chg-soc-update-fail",
		.handler	= smblib_handle_step_chg_soc_update_fail,
		.wake		= true,
	},
	[STEP_CHG_SOC_UPDATE_REQ_IRQ] = {
		.name		= "step-chg-soc-update-request",
		.handler	= smblib_handle_step_chg_soc_update_request,
		.wake		= true,
	},
/* OTG IRQs */
	[OTG_FAIL_IRQ] = {
		.name		= "otg-fail",
		.handler	= smblib_handle_debug,
	},
	[OTG_OVERCURRENT_IRQ] = {
		.name		= "otg-overcurrent",
		.handler	= smblib_handle_otg_overcurrent,
	},
	[OTG_OC_DIS_SW_STS_IRQ] = {
		.name		= "otg-oc-dis-sw-sts",
		.handler	= smblib_handle_debug,
	},
	[TESTMODE_CHANGE_DET_IRQ] = {
		.name		= "testmode-change-detect",
		.handler	= smblib_handle_debug,
	},
/* BATTERY IRQs */
	[BATT_TEMP_IRQ] = {
		.name		= "bat-temp",
		.handler	= smblib_handle_batt_temp_changed,
	},
	[BATT_OCP_IRQ] = {
		.name		= "bat-ocp",
		.handler	= smblib_handle_batt_psy_changed,
	},
	[BATT_OV_IRQ] = {
		.name		= "bat-ov",
		.handler	= smblib_handle_batt_psy_changed,
	},
	[BATT_LOW_IRQ] = {
		.name		= "bat-low",
		.handler	= smblib_handle_batt_psy_changed,
	},
	[BATT_THERM_ID_MISS_IRQ] = {
		.name		= "bat-therm-or-id-missing",
		.handler	= smblib_handle_batt_psy_changed,
	},
	[BATT_TERM_MISS_IRQ] = {
		.name		= "bat-terminal-missing",
		.handler	= smblib_handle_batt_psy_changed,
	},
/* USB INPUT IRQs */
	[USBIN_COLLAPSE_IRQ] = {
		.name		= "usbin-collapse",
		.handler	= smblib_handle_debug,
	},
	[USBIN_LT_3P6V_IRQ] = {
		.name		= "usbin-lt-3p6v",
		.handler	= smblib_handle_debug,
	},
	[USBIN_UV_IRQ] = {
		.name		= "usbin-uv",
		.handler	= smblib_handle_usbin_uv,
	},
	[USBIN_OV_IRQ] = {
		.name		= "usbin-ov",
		.handler	= smblib_handle_debug,
	},
	[USBIN_PLUGIN_IRQ] = {
		.name		= "usbin-plugin",
		.handler	= smblib_handle_usb_plugin,
		.wake		= true,
	},
	[USBIN_SRC_CHANGE_IRQ] = {
		.name		= "usbin-src-change",
		.handler	= smblib_handle_usb_source_change,
		.wake		= true,
	},
	[USBIN_ICL_CHANGE_IRQ] = {
		.name		= "usbin-icl-change",
		.handler	= smblib_handle_icl_change,
		.wake		= true,
	},
	[TYPE_C_CHANGE_IRQ] = {
		.name		= "type-c-change",
		.handler	= smblib_handle_usb_typec_change,
		.wake		= true,
	},
/* DC INPUT IRQs */
	[DCIN_COLLAPSE_IRQ] = {
		.name		= "dcin-collapse",
		.handler	= smblib_handle_debug,
	},
	[DCIN_LT_3P6V_IRQ] = {
		.name		= "dcin-lt-3p6v",
		.handler	= smblib_handle_debug,
	},
	[DCIN_UV_IRQ] = {
		.name		= "dcin-uv",
		.handler	= smblib_handle_debug,
	},
	[DCIN_OV_IRQ] = {
		.name		= "dcin-ov",
		.handler	= smblib_handle_debug,
	},
	[DCIN_PLUGIN_IRQ] = {
		.name		= "dcin-plugin",
		.handler	= smblib_handle_dc_plugin,
		.wake		= true,
	},
	[DIV2_EN_DG_IRQ] = {
		.name		= "div2-en-dg",
		.handler	= smblib_handle_debug,
	},
	[DCIN_ICL_CHANGE_IRQ] = {
		.name		= "dcin-icl-change",
		.handler	= smblib_handle_debug,
	},
/* MISCELLANEOUS IRQs */
	[WDOG_SNARL_IRQ] = {
		.name		= "wdog-snarl",
		.handler	= NULL,
	},
	[WDOG_BARK_IRQ] = {
		.name		= "wdog-bark",
		.handler	= NULL,
	},
	[AICL_FAIL_IRQ] = {
		.name		= "aicl-fail",
		.handler	= smblib_handle_debug,
	},
	[AICL_DONE_IRQ] = {
		.name		= "aicl-done",
		.handler	= smblib_handle_debug,
	},
	[HIGH_DUTY_CYCLE_IRQ] = {
		.name		= "high-duty-cycle",
		.handler	= smblib_handle_high_duty_cycle,
		.wake		= true,
	},
	[INPUT_CURRENT_LIMIT_IRQ] = {
		.name		= "input-current-limiting",
		.handler	= smblib_handle_debug,
	},
	[TEMPERATURE_CHANGE_IRQ] = {
		.name		= "temperature-change",
		.handler	= smblib_handle_debug,
	},
	[SWITCH_POWER_OK_IRQ] = {
		.name		= "switcher-power-ok",
		.handler	= smblib_handle_switcher_power_ok,
		.storm_data	= {true, 1000, 3},
	},
};

static int smb2_get_irq_index_byname(const char *irq_name)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(smb2_irqs); i++) {
		if (strcmp(smb2_irqs[i].name, irq_name) == 0)
			return i;
	}

	return -ENOENT;
}

static int smb2_request_interrupt(struct smb2 *chip,
				struct device_node *node, const char *irq_name)
{
	struct smb_charger *chg = &chip->chg;
	int rc, irq, irq_index;
	struct smb_irq_data *irq_data;

	irq = of_irq_get_byname(node, irq_name);
	if (irq < 0) {
		pr_err("Couldn't get irq %s byname\n", irq_name);
		return irq;
	}

	irq_index = smb2_get_irq_index_byname(irq_name);
	if (irq_index < 0) {
		pr_err("%s is not a defined irq\n", irq_name);
		return irq_index;
	}

	if (!smb2_irqs[irq_index].handler)
		return 0;

#ifdef OPLUS_FEATURE_CHG_BASIC
	if (irq_index == TYPE_C_CHANGE_IRQ/* || strcmp(irq_name, "type-c-change") == 0*/) {
		if (g_oplus_chip && oplus_usbid_check_is_gpio(g_oplus_chip) == true) {
			printk(KERN_ERR "[OPLUS_CHG] usbid is gpio\n");
			return 0;
		} else {
			printk(KERN_ERR "[OPLUS_CHG] usbid is PMIC\n");
		}
	}
#endif

	irq_data = devm_kzalloc(chg->dev, sizeof(*irq_data), GFP_KERNEL);
	if (!irq_data)
		return -ENOMEM;

	irq_data->parent_data = chip;
	irq_data->name = irq_name;
	irq_data->storm_data = smb2_irqs[irq_index].storm_data;
	mutex_init(&irq_data->storm_data.storm_lock);

	rc = devm_request_threaded_irq(chg->dev, irq, NULL,
					smb2_irqs[irq_index].handler,
					IRQF_ONESHOT, irq_name, irq_data);
	if (rc < 0) {
		pr_err("Couldn't request irq %d\n", irq);
		return rc;
	}

	smb2_irqs[irq_index].irq = irq;
	smb2_irqs[irq_index].irq_data = irq_data;
	if (smb2_irqs[irq_index].wake)
		enable_irq_wake(irq);

	return rc;
}

static int smb2_request_interrupts(struct smb2 *chip)
{
	struct smb_charger *chg = &chip->chg;
	struct device_node *node = chg->dev->of_node;
	struct device_node *child;
	int rc = 0;
	const char *name;
	struct property *prop;

	for_each_available_child_of_node(node, child) {
		of_property_for_each_string(child, "interrupt-names",
					    prop, name) {
			rc = smb2_request_interrupt(chip, child, name);
			if (rc < 0)
				return rc;
		}
	}

	return rc;
}

#if defined(CONFIG_DEBUG_FS)

static int force_batt_psy_update_write(void *data, u64 val)
{
	struct smb_charger *chg = data;

	power_supply_changed(chg->batt_psy);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(force_batt_psy_update_ops, NULL,
			force_batt_psy_update_write, "0x%02llx\n");

static int force_usb_psy_update_write(void *data, u64 val)
{
	struct smb_charger *chg = data;

	power_supply_changed(chg->usb_psy);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(force_usb_psy_update_ops, NULL,
			force_usb_psy_update_write, "0x%02llx\n");

static int force_dc_psy_update_write(void *data, u64 val)
{
	struct smb_charger *chg = data;

#ifdef OPLUS_FEATURE_CHG_BASIC
	if (chg->dc_psy)
#endif
	power_supply_changed(chg->dc_psy);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(force_dc_psy_update_ops, NULL,
			force_dc_psy_update_write, "0x%02llx\n");

static void smb2_create_debugfs(struct smb2 *chip)
{
	struct dentry *file;

	chip->dfs_root = debugfs_create_dir("charger", NULL);
	if (IS_ERR_OR_NULL(chip->dfs_root)) {
		pr_err("Couldn't create charger debugfs rc=%ld\n",
			(long)chip->dfs_root);
		return;
	}

	file = debugfs_create_file("force_batt_psy_update", S_IRUSR | S_IWUSR,
			    chip->dfs_root, chip, &force_batt_psy_update_ops);
	if (IS_ERR_OR_NULL(file))
		pr_err("Couldn't create force_batt_psy_update file rc=%ld\n",
			(long)file);

	file = debugfs_create_file("force_usb_psy_update", S_IRUSR | S_IWUSR,
			    chip->dfs_root, chip, &force_usb_psy_update_ops);
	if (IS_ERR_OR_NULL(file))
		pr_err("Couldn't create force_usb_psy_update file rc=%ld\n",
			(long)file);

	file = debugfs_create_file("force_dc_psy_update", S_IRUSR | S_IWUSR,
			    chip->dfs_root, chip, &force_dc_psy_update_ops);
	if (IS_ERR_OR_NULL(file))
		pr_err("Couldn't create force_dc_psy_update file rc=%ld\n",
			(long)file);
}

#else

static void smb2_create_debugfs(struct smb2 *chip)
{}

#endif


#ifdef OPLUS_FEATURE_CHG_BASIC
static bool d_reg_mask = false;
static ssize_t dump_registers_mask_write(struct file *file, const char __user *buff, size_t count, loff_t *ppos)
{
	char mask[16];

	if (copy_from_user(&mask, buff, count)) {
		printk(KERN_ERR "dump_registers_mask_write error.\n");
		return -EFAULT;
	}

	if (strncmp(mask, "dump808", 7) == 0) {
		d_reg_mask = true;
		printk(KERN_ERR "dump registers mask enable.\n");
	} else {
		d_reg_mask = false;
		return -EFAULT;
	}

	return count;
}

static const struct file_operations dump_registers_mask_fops = {
	.write = dump_registers_mask_write,
	.llseek = noop_llseek,
};

static void init_proc_dump_registers_mask(void)
{
	if (!proc_create("d_reg_mask", S_IWUSR | S_IWGRP | S_IWOTH, NULL, &dump_registers_mask_fops)) {
		printk(KERN_ERR "proc_create dump_registers_mask_fops fail\n");
	}
}
#endif /* OPLUS_FEATURE_CHG_BASIC */

#ifdef OPLUS_FEATURE_CHG_BASIC
#ifndef USE_8998_FILE
//static int get_boot_mode(void);
static int smbchg_usb_suspend_disable(void);
static int smbchg_usb_suspend_enable(void);
static int smbchg_charging_enble(void);
bool oplus_chg_is_usb_present(void);
int qpnp_get_prop_charger_voltage_now(void);

static void dump_regs(void)
{
	int i;
	int j;
	int rc;
	u8 stat;
	int base[] = {0x1000, 0x1100, 0x1200, 0x1300, 0x1400, 0x1600, 0x1800, 0x1900};
	struct smb_charger *chg = NULL;

	if (!g_oplus_chip || !d_reg_mask)
		return;

	chg = &g_oplus_chip->pmic_spmi.smb2_chip->chg;
	if (!chg)
		return;

	pr_err("================= %s: begin ======================\n", __func__);

	for (j = 0; j < 8; j++) {
		for (i = 0; i < 255; i++) {
			rc = smblib_read(chg, base[j] + i, &stat);
			if (rc < 0) {
				pr_err("Couldn't read %x rc=%d\n", base[j] + i, rc);
			} else {
				pr_err("%x : %x\n", base[j] + i, stat);
			}
		}

		msleep(1000);
	}

	pr_err("================= %s: end ======================\n", __func__);

	d_reg_mask = false;
}

static int smbchg_kick_wdt(void)
{
	return 0;
}

static int oplus_chg_hw_init(void)
{
	int boot_mode = get_boot_mode();

	if (boot_mode != MSM_BOOT_MODE__RF && boot_mode != MSM_BOOT_MODE__WLAN) {
		smbchg_usb_suspend_disable();
	} else {
		smbchg_usb_suspend_enable();
	}
	smbchg_charging_enble();

	return 0;
}

static int smbchg_set_fastchg_current_raw(int current_ma)
{
	int rc = 0;

	rc = vote(g_oplus_chip->pmic_spmi.smb2_chip->chg.fcc_votable, DEFAULT_VOTER,
			true, current_ma * 1000);
	if (rc < 0)
		chg_err("Couldn't vote fcc_votable[%d], rc=%d\n", current_ma, rc);

	return rc;
}

static void smbchg_set_aicl_point(int vol)
{
	//DO Nothing
}

static void smbchg_aicl_enable(bool enable)
{
	int rc = 0;
	struct oplus_chg_chip *chip = g_oplus_chip;
	
	rc = smblib_masked_write(&chip->pmic_spmi.smb2_chip->chg, USBIN_AICL_OPTIONS_CFG_REG,
			USBIN_AICL_EN_BIT, enable ? USBIN_AICL_EN_BIT : 0);
	if (rc < 0)
		chg_err("Couldn't write USBIN_AICL_OPTIONS_CFG_REG rc=%d\n", rc);
}

static void smbchg_rerun_aicl(void)
{
	smbchg_aicl_enable(false);
	/* Add a delay so that AICL successfully clears */
	msleep(50);
	smbchg_aicl_enable(true);
}

static bool  oplus_chg_is_normal_mode(void)
{
	int boot_mode = get_boot_mode();

	if (boot_mode == MSM_BOOT_MODE__RF || boot_mode == MSM_BOOT_MODE__WLAN)
		return false;
	return true;
}

static bool oplus_chg_is_suspend_status(void)
{
	int rc = 0;
	u8 stat;
	struct smb_charger *chg = NULL;

	if (!g_oplus_chip)
		return false;

	chg = &g_oplus_chip->pmic_spmi.smb2_chip->chg;

	rc = smblib_read(chg, POWER_PATH_STATUS_REG, &stat);
	if (rc < 0) {
		printk(KERN_ERR "oplus_chg_is_suspend_status: Couldn't read POWER_PATH_STATUS rc=%d\n", rc);
		return false;
	}

	return (bool)(stat & USBIN_SUSPEND_STS_BIT);
}

static void oplus_chg_clear_suspend(void)
{
	int rc;
	struct smb_charger *chg = NULL;

	if (!g_oplus_chip)
		return;

	chg = &g_oplus_chip->pmic_spmi.smb2_chip->chg;

	rc = smblib_masked_write(chg, USBIN_CMD_IL_REG, USBIN_SUSPEND_BIT, 1);
	if (rc < 0) {
		printk(KERN_ERR "oplus_chg_monitor_work: Couldn't set USBIN_SUSPEND_BIT rc=%d\n", rc);
	}
	msleep(50);
	rc = smblib_masked_write(chg, USBIN_CMD_IL_REG, USBIN_SUSPEND_BIT, 0);
	if (rc < 0) {
		printk(KERN_ERR "oplus_chg_monitor_work: Couldn't clear USBIN_SUSPEND_BIT rc=%d\n", rc);
	}
}

static void oplus_chg_check_clear_suspend(void)
{
	use_present_status = true;
	oplus_chg_clear_suspend();
	use_present_status = false;
}

static int usb_icl[] = {
	300, 500, 900, 1200, 1500, 1750, 2000, 3000,
};

#define USBIN_25MA	25000
static int oplus_chg_set_input_current(int current_ma)
{
	int rc = 0, i = 0;
	int chg_vol = 0;
	int aicl_point = 0;
	int aicl_point_temp = 0;
	struct oplus_chg_chip *chip = g_oplus_chip;
	if (get_client_vote(chip->pmic_spmi.smb2_chip->chg.usb_icl_votable, BOOST_BACK_VOTER) == 0
			&& get_effective_result(chip->pmic_spmi.smb2_chip->chg.usb_icl_votable) < USBIN_25MA) {
		vote(chip->pmic_spmi.smb2_chip->chg.usb_icl_votable, BOOST_BACK_VOTER, false, 0);
	}

	if (chip->pmic_spmi.smb2_chip->chg.pre_current_ma == current_ma)
		return rc;
	else
		chip->pmic_spmi.smb2_chip->chg.pre_current_ma = current_ma;

	chg_debug( "usb input max current limit=%d setting %02x\n", current_ma, i);

	aicl_point_temp = aicl_point = 4500;

	smbchg_aicl_enable(false);

	if (current_ma < 500) {
		i = 0;
		goto aicl_end;
	}

	i = 1; /* 500 */
	rc = vote(chip->pmic_spmi.smb2_chip->chg.usb_icl_votable, USB_PSY_VOTER, true, usb_icl[i] * 1000);
	msleep(90);

	if (get_client_vote(chip->pmic_spmi.smb2_chip->chg.usb_icl_votable, BOOST_BACK_VOTER) == 0
			&& get_effective_result(chip->pmic_spmi.smb2_chip->chg.usb_icl_votable) < USBIN_25MA) {
		chg_debug( "use 500 here\n");
		goto aicl_boost_back;
	}

	chg_vol = qpnp_get_prop_charger_voltage_now();
	if (get_client_vote(chip->pmic_spmi.smb2_chip->chg.usb_icl_votable, BOOST_BACK_VOTER) == 0
			&& get_effective_result(chip->pmic_spmi.smb2_chip->chg.usb_icl_votable) < USBIN_25MA) {
		chg_debug( "use 500 here\n");
		goto aicl_boost_back;
	}
	if (chg_vol < aicl_point_temp) {
		chg_debug( "use 500 here\n");
		goto aicl_end;
	} else if (current_ma < 900)
		goto aicl_end;

	i = 2; /* 900 */
	rc = vote(chip->pmic_spmi.smb2_chip->chg.usb_icl_votable, USB_PSY_VOTER, true, usb_icl[i] * 1000);
	msleep(90);

	if (get_client_vote(chip->pmic_spmi.smb2_chip->chg.usb_icl_votable, BOOST_BACK_VOTER) == 0
			&& get_effective_result(chip->pmic_spmi.smb2_chip->chg.usb_icl_votable) < USBIN_25MA) {
		i = i - 1;
		goto aicl_boost_back;
	}
	if (oplus_chg_is_suspend_status() && oplus_chg_is_usb_present() && oplus_chg_is_normal_mode()) {
		i = i - 1;
		goto aicl_suspend;
	}

	chg_vol = qpnp_get_prop_charger_voltage_now();
	if (get_client_vote(chip->pmic_spmi.smb2_chip->chg.usb_icl_votable, BOOST_BACK_VOTER) == 0
			&& get_effective_result(chip->pmic_spmi.smb2_chip->chg.usb_icl_votable) < USBIN_25MA) {
		i = i - 1;
		goto aicl_boost_back;
	}
	if (chg_vol < aicl_point_temp) {
		i = i - 1;
		goto aicl_pre_step;
	} else if (current_ma < 1200)
		goto aicl_end;

	i = 3; /* 1200 */
	rc = vote(chip->pmic_spmi.smb2_chip->chg.usb_icl_votable, USB_PSY_VOTER, true, usb_icl[i] * 1000);
	msleep(90);

	if (get_client_vote(chip->pmic_spmi.smb2_chip->chg.usb_icl_votable, BOOST_BACK_VOTER) == 0
			&& get_effective_result(chip->pmic_spmi.smb2_chip->chg.usb_icl_votable) < USBIN_25MA) {
		i = i - 1;
		goto aicl_boost_back;
	}
	if (oplus_chg_is_suspend_status() && oplus_chg_is_usb_present() && oplus_chg_is_normal_mode()) {
		i = i - 1;
		goto aicl_suspend;
	}

	chg_vol = qpnp_get_prop_charger_voltage_now();
	if (get_client_vote(chip->pmic_spmi.smb2_chip->chg.usb_icl_votable, BOOST_BACK_VOTER) == 0
			&& get_effective_result(chip->pmic_spmi.smb2_chip->chg.usb_icl_votable) < USBIN_25MA) {
		i = i - 1;
		goto aicl_boost_back;
	}
	if (chg_vol < aicl_point_temp) {
		i = i - 1;
		goto aicl_pre_step;
	}

	i = 4; /* 1500 */
	aicl_point_temp = aicl_point + 50;
	rc = vote(chip->pmic_spmi.smb2_chip->chg.usb_icl_votable, USB_PSY_VOTER, true, usb_icl[i] * 1000);
	msleep(120);

	if (get_client_vote(chip->pmic_spmi.smb2_chip->chg.usb_icl_votable, BOOST_BACK_VOTER) == 0
			&& get_effective_result(chip->pmic_spmi.smb2_chip->chg.usb_icl_votable) < USBIN_25MA) {
		i = i - 2;
		goto aicl_boost_back;
	}
	if (oplus_chg_is_suspend_status() && oplus_chg_is_usb_present() && oplus_chg_is_normal_mode()) {
		i = i - 2;
		goto aicl_suspend;
	}

	chg_vol = qpnp_get_prop_charger_voltage_now();
	if (get_client_vote(chip->pmic_spmi.smb2_chip->chg.usb_icl_votable, BOOST_BACK_VOTER) == 0
			&& get_effective_result(chip->pmic_spmi.smb2_chip->chg.usb_icl_votable) < USBIN_25MA) {
		i = i - 2;
		goto aicl_boost_back;
	}
	if (chg_vol < aicl_point_temp) {
		i = i - 2; //We DO NOT use 1.2A here
		goto aicl_pre_step;
	} else if (current_ma < 1500) {
		i = i - 1; //We use 1.2A here
		goto aicl_end;
	} else if (current_ma < 2000)
		goto aicl_end;

	i = 5; /* 1750 */
	aicl_point_temp = aicl_point + 50;
	rc = vote(chip->pmic_spmi.smb2_chip->chg.usb_icl_votable, USB_PSY_VOTER, true, usb_icl[i] * 1000);
	msleep(120);

	if (get_client_vote(chip->pmic_spmi.smb2_chip->chg.usb_icl_votable, BOOST_BACK_VOTER) == 0
			&& get_effective_result(chip->pmic_spmi.smb2_chip->chg.usb_icl_votable) < USBIN_25MA) {
		i = i - 2;
		goto aicl_boost_back;
	}
	if (oplus_chg_is_suspend_status() && oplus_chg_is_usb_present() && oplus_chg_is_normal_mode()) {
		i = i - 2;
		goto aicl_suspend;
	}

	chg_vol = qpnp_get_prop_charger_voltage_now();
	if (get_client_vote(chip->pmic_spmi.smb2_chip->chg.usb_icl_votable, BOOST_BACK_VOTER) == 0
			&& get_effective_result(chip->pmic_spmi.smb2_chip->chg.usb_icl_votable) < USBIN_25MA) {
		i = i - 2;
		goto aicl_boost_back;
	}
	if (chg_vol < aicl_point_temp) {
		i = i - 2; //1.2
		goto aicl_pre_step;
	}

	i = 6; /* 2000 */
	aicl_point_temp = aicl_point;
	rc = vote(chip->pmic_spmi.smb2_chip->chg.usb_icl_votable, USB_PSY_VOTER, true, usb_icl[i] * 1000);
	msleep(90);

	if (get_client_vote(chip->pmic_spmi.smb2_chip->chg.usb_icl_votable, BOOST_BACK_VOTER) == 0
			&& get_effective_result(chip->pmic_spmi.smb2_chip->chg.usb_icl_votable) < USBIN_25MA) {
		i = i - 2;
		goto aicl_boost_back;
	}
	if (oplus_chg_is_suspend_status() && oplus_chg_is_usb_present() && oplus_chg_is_normal_mode()) {
		i = i - 2;
		goto aicl_suspend;
	}

	chg_vol = qpnp_get_prop_charger_voltage_now();
	if (get_client_vote(chip->pmic_spmi.smb2_chip->chg.usb_icl_votable, BOOST_BACK_VOTER) == 0
			&& get_effective_result(chip->pmic_spmi.smb2_chip->chg.usb_icl_votable) < USBIN_25MA) {
		i = i - 2;
		goto aicl_boost_back;
	}
	if (chg_vol < aicl_point_temp) {
		i =  i - 2;//1.5
		goto aicl_pre_step;
	} else if (current_ma < 3000)
		goto aicl_end;

	i = 7; /* 3000 */
	rc = vote(chip->pmic_spmi.smb2_chip->chg.usb_icl_votable, USB_PSY_VOTER, true, usb_icl[i] * 1000);
	msleep(90);

	if (get_client_vote(chip->pmic_spmi.smb2_chip->chg.usb_icl_votable, BOOST_BACK_VOTER) == 0
			&& get_effective_result(chip->pmic_spmi.smb2_chip->chg.usb_icl_votable) < USBIN_25MA) {
		i = i - 1;
		goto aicl_boost_back;
	}
	if (oplus_chg_is_suspend_status() && oplus_chg_is_usb_present() && oplus_chg_is_normal_mode()) {
		i = i - 1;
		goto aicl_suspend;
	}

	chg_vol = qpnp_get_prop_charger_voltage_now();
	if (chg_vol < aicl_point_temp) {
		i = i - 1;
		goto aicl_pre_step;
	} else if (current_ma >= 3000)
		goto aicl_end;

aicl_pre_step:
	rc = vote(chip->pmic_spmi.smb2_chip->chg.usb_icl_votable, USB_PSY_VOTER, true, usb_icl[i] * 1000);
	chg_debug( "usb input max current limit aicl chg_vol=%d j[%d]=%d sw_aicl_point:%d aicl_pre_step\n", chg_vol, i, usb_icl[i], aicl_point_temp);
	smbchg_rerun_aicl();
	return rc;
aicl_end:
	rc = vote(chip->pmic_spmi.smb2_chip->chg.usb_icl_votable, USB_PSY_VOTER, true, usb_icl[i] * 1000);
	chg_debug( "usb input max current limit aicl chg_vol=%d j[%d]=%d sw_aicl_point:%d aicl_end\n", chg_vol, i, usb_icl[i], aicl_point_temp);
	smbchg_rerun_aicl();
	return rc;
aicl_boost_back:
	rc = vote(chip->pmic_spmi.smb2_chip->chg.usb_icl_votable, USB_PSY_VOTER, true, usb_icl[i] * 1000);
	chg_debug( "usb input max current limit aicl chg_vol=%d j[%d]=%d sw_aicl_point:%d aicl_boost_back\n", chg_vol, i, usb_icl[i], aicl_point_temp);
	if (chip->pmic_spmi.smb2_chip->chg.wa_flags & BOOST_BACK_WA)
		vote(chip->pmic_spmi.smb2_chip->chg.usb_icl_votable, BOOST_BACK_VOTER, false, 0);
	smbchg_rerun_aicl();
	return rc;
aicl_suspend:
	rc = vote(chip->pmic_spmi.smb2_chip->chg.usb_icl_votable, USB_PSY_VOTER, true, usb_icl[i] * 1000);
	chg_debug( "usb input max current limit aicl chg_vol=%d j[%d]=%d sw_aicl_point:%d aicl_suspend\n", chg_vol, i, usb_icl[i], aicl_point_temp);
	oplus_chg_check_clear_suspend();
	smbchg_rerun_aicl();
	return rc;
}

static int smbchg_float_voltage_set(int vfloat_mv)
{
	int rc = 0;

	rc = vote(g_oplus_chip->pmic_spmi.smb2_chip->chg.fv_votable, DEFAULT_VOTER,
			true, vfloat_mv * 1000);
	if (rc < 0)
		chg_err("Couldn't vote fv_votable[%d], rc=%d\n", vfloat_mv, rc);

	return rc;
}

static int smbchg_term_current_set(int term_current)
{
	int rc = 0;
	u8 val_raw = 0;
	
	if (term_current < 0 || term_current > 750)
		term_current = 150;

	val_raw = term_current / 50;
	rc = smblib_masked_write(&g_oplus_chip->pmic_spmi.smb2_chip->chg, TCCC_CHARGE_CURRENT_TERMINATION_CFG_REG,
			TCCC_CHARGE_CURRENT_TERMINATION_SETTING_MASK, val_raw);
	if (rc < 0)
		chg_err("Couldn't write TCCC_CHARGE_CURRENT_TERMINATION_CFG_REG rc=%d\n", rc);

	return rc;
}

static int smbchg_charging_enble(void)
{
	int rc = 0;

	rc = vote(g_oplus_chip->pmic_spmi.smb2_chip->chg.chg_disable_votable, DEFAULT_VOTER,
			false, 0);
	if (rc < 0)
		chg_err("Couldn't enable charging, rc=%d\n", rc);

	return rc;
}

static int smbchg_charging_disble(void)
{
	int rc = 0;

	rc = vote(g_oplus_chip->pmic_spmi.smb2_chip->chg.chg_disable_votable, DEFAULT_VOTER,
			true, 0);
	if (rc < 0)
		chg_err("Couldn't disable charging, rc=%d\n", rc);

	g_oplus_chip->pmic_spmi.smb2_chip->chg.pre_current_ma = -1;

	return rc;
}

static int smbchg_get_charge_enable(void)
{
	int rc = 0;
	u8 temp = 0;

	rc = smblib_read(&g_oplus_chip->pmic_spmi.smb2_chip->chg, CHARGING_ENABLE_CMD_REG, &temp);
	if (rc < 0) {
		chg_err("Couldn't read CHARGING_ENABLE_CMD_REG rc=%d\n", rc);
		return 0;
	}
	rc = temp & CHARGING_ENABLE_CMD_BIT;

	return rc;
}

static int smbchg_usb_suspend_enable(void)
{
	int rc = 0;

	rc = smblib_set_usb_suspend(&g_oplus_chip->pmic_spmi.smb2_chip->chg, true);
	if (rc < 0)
		chg_err("Couldn't write enable to USBIN_SUSPEND_BIT rc=%d\n", rc);

	g_oplus_chip->pmic_spmi.smb2_chip->chg.pre_current_ma = -1;

	return rc;
}

static int smbchg_usb_suspend_disable(void)
{
	int rc = 0;
	int boot_mode = get_boot_mode();

	if (boot_mode == MSM_BOOT_MODE__RF || boot_mode == MSM_BOOT_MODE__WLAN) {
		chg_err("RF/WLAN, suspending...\n");
		rc = smblib_set_usb_suspend(&g_oplus_chip->pmic_spmi.smb2_chip->chg, true);
		if (rc < 0)
			chg_err("Couldn't write enable to USBIN_SUSPEND_BIT rc=%d\n", rc);
		return rc;
	}

	rc = smblib_set_usb_suspend(&g_oplus_chip->pmic_spmi.smb2_chip->chg, false);
	if (rc < 0)
		chg_err("Couldn't write disable to USBIN_SUSPEND_BIT rc=%d\n", rc);

	return rc;
}

static int smbchg_set_rechg_vol(int rechg_vol)
{
	return 0;
}

static int smbchg_reset_charger(void)
{
	return 0;
}

static int smbchg_read_full(void)
{
	int rc = 0;
	u8 stat = 0;

	if (!oplus_chg_is_usb_present())
		return 0;

	rc = smblib_read(&g_oplus_chip->pmic_spmi.smb2_chip->chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		chg_err("Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n", rc);
		return 0;
	}
	stat = stat & BATTERY_CHARGER_STATUS_MASK;

	if (stat == TERMINATE_CHARGE || stat == INHIBIT_CHARGE)
		return 1;
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

static int oplus_set_chging_term_disable(void)
{
	return 0;
}

static bool qcom_check_charger_resume(void)
{
	return true;
}

int smbchg_get_chargerid_volt(void)
{
	int rc = 0;
	int chargerid_volt = 0;
	struct qpnp_vadc_result results;
	struct oplus_chg_chip *chip = g_oplus_chip;
	
	if (!chip->pmic_spmi.pm660_vadc_dev) {
		chg_err("pm660_vadc_dev NULL\n");
		return 0;
	}

	rc = qpnp_vadc_read(chip->pmic_spmi.pm660_vadc_dev, P_MUX3_1_1, &results);
	if (rc) {
		chg_err("unable to read pm660_vadc_dev P_MUX3_1_1 rc = %d\n", rc);
		return 0;
	}
	chargerid_volt = (int)results.physical / 1000;
	chg_err("chargerid_volt: %d\n", chargerid_volt);

	return chargerid_volt;
}

static int smbchg_chargerid_switch_gpio_init(struct oplus_chg_chip *chip)
{
	
	chip->normalchg_gpio.pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->normalchg_gpio.pinctrl)) {
		chg_err("get normalchg_gpio.pinctrl fail\n");
		return -EINVAL;
	}

	chip->normalchg_gpio.chargerid_switch_active =
			pinctrl_lookup_state(chip->normalchg_gpio.pinctrl, "chargerid_switch_active");
	if (IS_ERR_OR_NULL(chip->normalchg_gpio.chargerid_switch_active)) {
		chg_err("get chargerid_switch_active fail\n");
		return -EINVAL;
	}

	chip->normalchg_gpio.chargerid_switch_sleep =
			pinctrl_lookup_state(chip->normalchg_gpio.pinctrl, "chargerid_switch_sleep");
	if (IS_ERR_OR_NULL(chip->normalchg_gpio.chargerid_switch_sleep)) {
		chg_err("get chargerid_switch_sleep fail\n");
		return -EINVAL;
	}

	chip->normalchg_gpio.chargerid_switch_default =
			pinctrl_lookup_state(chip->normalchg_gpio.pinctrl, "chargerid_switch_default");
	if (IS_ERR_OR_NULL(chip->normalchg_gpio.chargerid_switch_default)) {
		chg_err("get chargerid_switch_default fail\n");
		return -EINVAL;
	}

	if (chip->normalchg_gpio.chargerid_switch_gpio > 0) {
		gpio_direction_output(chip->normalchg_gpio.chargerid_switch_gpio, 0);
	}
	pinctrl_select_state(chip->normalchg_gpio.pinctrl, chip->normalchg_gpio.chargerid_switch_default);

	return 0;
}

void smbchg_set_chargerid_switch_val(int value)
{
	struct oplus_chg_chip *chip = g_oplus_chip;
	
	if (chip->normalchg_gpio.chargerid_switch_gpio <= 0) {
		chg_err("chargerid_switch_gpio not exist, return\n");
		return;
	}

	if (IS_ERR_OR_NULL(chip->normalchg_gpio.pinctrl)
		|| IS_ERR_OR_NULL(chip->normalchg_gpio.chargerid_switch_active)
		|| IS_ERR_OR_NULL(chip->normalchg_gpio.chargerid_switch_sleep)
		|| IS_ERR_OR_NULL(chip->normalchg_gpio.chargerid_switch_default)) {
		chg_err("pinctrl null, return\n");
		return;
	}

	if (oplus_vooc_get_adapter_update_real_status() == ADAPTER_FW_NEED_UPDATE
		|| oplus_vooc_get_btb_temp_over() == true) {
		chg_err("adapter update or btb_temp_over, return\n");
		return;
	}
#if 0
	if (chip->pmic_spmi.not_support_1200ma && !value && !is_usb_present(chip)) {
	/* BugID 879716 : Solve some situatuion ChargerID is not 0 mV when usb is not present */
		chip->chargerid_volt = 0;
		chip->chargerid_volt_got = false;
	}
#endif
	if (value) {
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

int smbchg_get_chargerid_switch_val(void)
{
	struct oplus_chg_chip *chip = g_oplus_chip;
	
	if (chip->normalchg_gpio.chargerid_switch_gpio <= 0) {
		chg_err("chargerid_switch_gpio not exist, return\n");
		return -1;
	}

	if (IS_ERR_OR_NULL(chip->normalchg_gpio.pinctrl)
		|| IS_ERR_OR_NULL(chip->normalchg_gpio.chargerid_switch_active)
		|| IS_ERR_OR_NULL(chip->normalchg_gpio.chargerid_switch_sleep)) {
		chg_err("pinctrl null, return\n");
		return -1;
	}

	return gpio_get_value(chip->normalchg_gpio.chargerid_switch_gpio);
}

#ifdef OPLUS_FEATURE_CHG_BASIC
static int oplus_usbid_gpio_init(struct oplus_chg_chip *chip)
{
	chip->normalchg_gpio.pinctrl = devm_pinctrl_get(chip->dev);

	if (IS_ERR_OR_NULL(chip->normalchg_gpio.pinctrl)) {
		chg_err("get normalchg_gpio.pinctrl fail\n");
		return -EINVAL;
	}

	chip->normalchg_gpio.usbid_active =
			pinctrl_lookup_state(chip->normalchg_gpio.pinctrl,
				"usbid_active");
	if (IS_ERR_OR_NULL(chip->normalchg_gpio.usbid_active)) {
		chg_err("get usbid_active fail\n");
		return -EINVAL;
	}

	chip->normalchg_gpio.usbid_sleep =
			pinctrl_lookup_state(chip->normalchg_gpio.pinctrl,
				"usbid_sleep");
	if (IS_ERR_OR_NULL(chip->normalchg_gpio.usbid_sleep)) {
		chg_err("get usbid_sleep fail\n");
		return -EINVAL;
	}

	if (chip->normalchg_gpio.usbid_gpio > 0) {
		gpio_direction_output(chip->normalchg_gpio.usbid_gpio, 0);
	}

	pinctrl_select_state(chip->normalchg_gpio.pinctrl,
		chip->normalchg_gpio.usbid_sleep);

	return 0;
}

static void oplus_set_usbid_active(struct oplus_chg_chip *chip)
{
	gpio_direction_input(chip->normalchg_gpio.usbid_gpio);// in
	pinctrl_select_state(chip->normalchg_gpio.pinctrl, chip->normalchg_gpio.usbid_active);// PULL_UP
}

static void oplus_set_usbid_sleep(struct oplus_chg_chip *chip)
{
	gpio_direction_output(chip->normalchg_gpio.usbid_gpio, 0);
	pinctrl_select_state(chip->normalchg_gpio.pinctrl, chip->normalchg_gpio.usbid_sleep);// no_PULL
}

static void oplus_usbid_irq_init(struct oplus_chg_chip *chip)
{
	chip->normalchg_gpio.usbid_irq = gpio_to_irq(chip->normalchg_gpio.usbid_gpio);
}

static void oplus_usbid_irq_register(struct oplus_chg_chip *chip)
{
	int retval = 0;
	union power_supply_propval ret = {0,};

	oplus_set_usbid_active(chip);

	retval = devm_request_threaded_irq(chip->dev, chip->normalchg_gpio.usbid_irq, NULL,
			usbid_change_handler, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			"usbid-change", chip);
	if (retval < 0) {
		chg_err("Unable to request usbid-change irq: %d\n", retval);
	}

	power_supply_get_property(chip->usb_psy, POWER_SUPPLY_PROP_OTG_SWITCH, &ret);
	if (ret.intval == false) {
		oplus_set_usbid_sleep(chip);
	}
}

bool oplus_usbid_check_is_gpio(struct oplus_chg_chip *chip)
{
	unsigned int project = get_project();
	unsigned char pcb_version = get_PCB_Version();

	if (gpio_is_valid(chip->normalchg_gpio.usbid_gpio)) {
		if (project == OPLUS_17011 && (pcb_version == HW_VERSION__11//DVT/PVT/MP
				|| pcb_version == HW_VERSION__12 || pcb_version == HW_VERSION__13)) {
			return true;
		}
		if (project == OPLUS_17021 && (pcb_version == HW_VERSION__12//DVT/PVT/MP
				|| pcb_version == HW_VERSION__13 || pcb_version == HW_VERSION__14)) {
			return true;
		}
	}
	return false;
}
#endif /* OPLUS_FEATURE_CHG_BASIC */

bool smbchg_need_to_check_ibatt(void)
{
	return true;
}

int smbchg_get_chg_current_step(void)
{
	return 25;
}

int opchg_get_charger_type(void)
{
	u8 apsd_stat;
	int rc;
	struct smb_charger *chg = NULL;
	struct oplus_chg_chip *chip = g_oplus_chip;
	
	if (!chip)
		return POWER_SUPPLY_TYPE_UNKNOWN;

	chg = &chip->pmic_spmi.smb2_chip->chg;

	/* reset for fastchg to normal */
	if (chip->charger_type == POWER_SUPPLY_TYPE_UNKNOWN)
		chg->pre_current_ma = -1;

	rc = smblib_read(chg, APSD_STATUS_REG, &apsd_stat);
	if (rc < 0) {
		chg_err("Couldn't read APSD_STATUS rc=%d\n", rc);
		return POWER_SUPPLY_TYPE_UNKNOWN;
	}
	chg_debug("APSD_STATUS = 0x%02x\n", apsd_stat);

	if (!(apsd_stat & APSD_DTC_STATUS_DONE_BIT))
		return POWER_SUPPLY_TYPE_UNKNOWN;

	if (chg->usb_psy_desc.type == POWER_SUPPLY_TYPE_USB
			|| chg->usb_psy_desc.type == POWER_SUPPLY_TYPE_USB_CDP
			|| chg->usb_psy_desc.type == POWER_SUPPLY_TYPE_USB_DCP) {
		oplus_chg_soc_update();
	}

	if (chg->usb_psy_desc.type == POWER_SUPPLY_TYPE_USB_CDP)
		return POWER_SUPPLY_TYPE_USB;
	return chg->usb_psy_desc.type;
}

int qpnp_get_prop_charger_voltage_now(void)
{
	int val = 0;
	struct smb_charger *chg = NULL;
	struct oplus_chg_chip *chip = g_oplus_chip;
	
	if (!chip)
		return 0;

	//if (!oplus_chg_is_usb_present())
	//	return 0;

	chg = &chip->pmic_spmi.smb2_chip->chg;
	if (!chg->iio.usbin_v_chan || PTR_ERR(chg->iio.usbin_v_chan) == -EPROBE_DEFER)
		chg->iio.usbin_v_chan = iio_channel_get(chg->dev, "usbin_v");

	if (IS_ERR(chg->iio.usbin_v_chan))
		return PTR_ERR(chg->iio.usbin_v_chan);

	iio_read_channel_processed(chg->iio.usbin_v_chan, &val);

	if (val < 2000 * 1000)
		chg->pre_current_ma = -1;

	return val / 1000;
}

bool oplus_chg_is_usb_present(void)
{
	int rc = 0;
	u8 stat = 0;
	bool vbus_rising = false;
	struct oplus_chg_chip *chip = g_oplus_chip;
	
	if (!chip)
		return false;

	rc = smblib_read(&chip->pmic_spmi.smb2_chip->chg, USBIN_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		chg_err("Couldn't read USB_INT_RT_STS, rc=%d\n", rc);
		return false;
	}
	vbus_rising = (bool)(stat & USBIN_PLUGIN_RT_STS_BIT);

	if (vbus_rising == false && oplus_vooc_get_fastchg_started() == true) {
		if (qpnp_get_prop_charger_voltage_now() > 2000) {
			chg_err("USBIN_PLUGIN_RT_STS_BIT low but fastchg started true and chg vol > 2V\n");
			vbus_rising = true;
		}
	}

	if (vbus_rising == false)
		chip->pmic_spmi.smb2_chip->chg.pre_current_ma = -1;

	return vbus_rising;
}

int qpnp_get_battery_voltage(void)
{
	return 3800;//Not use anymore
}

/*static int get_boot_mode(void)
{
	return 0;
}*/

int smbchg_get_boot_reason(void)
{
	return 0;
}

int oplus_chg_get_shutdown_soc(void)
{
	return 0;
}

int oplus_chg_backup_soc(int backup_soc)
{
	return 0;
}

static int smbchg_get_aicl_level_ma(void)
{
	return 0;
}

static int smbchg_force_tlim_en(bool enable)
{
	return 0;
}

static int smbchg_system_temp_level_set(int lvl_sel)
{
	return 0;
}

static int smbchg_set_prop_flash_active(enum skip_reason reason, bool disable)
{
	return 0;
}

static int smbchg_dp_dm(int val)
{
	return 0;
}

static int smbchg_calc_max_flash_current(void)
{
	return 0;
}

static int oplus_chg_get_fv(struct oplus_chg_chip *chip)
{
	int flv = chip->limits.temp_normal_vfloat_mv_normalchg;
	int batt_temp = chip->temperature;

	if (batt_temp > chip->limits.hot_bat_decidegc) {//53C
		//default
	} else if (batt_temp >= chip->limits.warm_bat_decidegc) {//45C
		flv = chip->limits.temp_warm_vfloat_mv;
	} else if (batt_temp >= chip->limits.normal_bat_decidegc) {//16C
		flv = chip->limits.temp_normal_vfloat_mv_normalchg;
	} else if (batt_temp >= chip->limits.little_cool_bat_decidegc) {//12C
		flv = chip->limits.temp_little_cool_vfloat_mv;
	} else if (batt_temp >= chip->limits.cool_bat_decidegc) {//5C
		flv = chip->limits.temp_cool_vfloat_mv;
	} else if (batt_temp >= chip->limits.little_cold_bat_decidegc) {//0C
		flv = chip->limits.temp_little_cold_vfloat_mv;
	} else if (batt_temp >= chip->limits.cold_bat_decidegc) {//-3C
		flv = chip->limits.temp_cold_vfloat_mv;
	} else {
		//default
	}

	return flv;
}

static int oplus_chg_get_charging_current(struct oplus_chg_chip *chip)
{
	int charging_current = 0;
	
	int batt_temp = chip->temperature;

	if (batt_temp > chip->limits.hot_bat_decidegc) {//53C
		charging_current = 0;
	} else if (batt_temp >= chip->limits.warm_bat_decidegc) {//45C
		charging_current = chip->limits.temp_warm_fastchg_current_ma;
	} else if (batt_temp >= chip->limits.normal_bat_decidegc) {//16C
		charging_current = chip->limits.temp_normal_fastchg_current_ma;
	} else if (batt_temp >= chip->limits.little_cool_bat_decidegc) {//12C
		charging_current = chip->limits.temp_little_cool_fastchg_current_ma;
	} else if (batt_temp >= chip->limits.cool_bat_decidegc) {//5C
		if (chip->batt_volt > 4180)
			charging_current = chip->limits.temp_cool_fastchg_current_ma_low;
		else
			charging_current = chip->limits.temp_cool_fastchg_current_ma_high;
	} else if (batt_temp >= chip->limits.little_cold_bat_decidegc) {//0C
		charging_current = chip->limits.temp_little_cold_fastchg_current_ma;
	} else if (batt_temp >= chip->limits.cold_bat_decidegc) {//-3C
		charging_current = chip->limits.temp_cold_fastchg_current_ma;
	} else {
		charging_current = 0;
	}

	return charging_current;
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

#ifdef CONFIG_OPLUS_SHORT_C_BATT_CHECK
/* This function is getting the dynamic aicl result/input limited in mA.
 * If charger was suspended, it must return 0(mA).
 * It meets the requirements in SDM660 platform.
 */
static int oplus_chg_get_dyna_aicl_result(void)
{
	struct power_supply *usb_psy = NULL;
	union power_supply_propval pval = {0, };

	usb_psy = power_supply_get_by_name("usb");
	if (usb_psy) {
		power_supply_get_property(usb_psy,
				POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED,
				&pval);
		return pval.intval / 1000;
	}

	return 1000;
}
#endif /* CONFIG_OPLUS_SHORT_C_BATT_CHECK */

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

static unsigned long suspend_tm_sec = 0;
static int smb2_pm_resume(struct device *dev)
{
	int rc = 0;
	unsigned long resume_tm_sec = 0;
	unsigned long sleep_time = 0;

	if (!g_oplus_chip)
		return 0;

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

static int smb2_pm_suspend(struct device *dev)
{
	if (!g_oplus_chip)
		return 0;

	if (get_current_time(&suspend_tm_sec)) {
		chg_err("RTC read failed\n");
		suspend_tm_sec = -1;
	}

	return 0;
}

static const struct dev_pm_ops smb2_pm_ops = {
	.resume		= smb2_pm_resume,
	.suspend		= smb2_pm_suspend,
};

struct oplus_chg_operations  smb2_chg_ops = {
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
	.set_charging_term_disable = oplus_set_chging_term_disable,
	.check_charger_resume = qcom_check_charger_resume,
	.get_chargerid_volt = smbchg_get_chargerid_volt,
	.set_chargerid_switch_val = smbchg_set_chargerid_switch_val,
	.get_chargerid_switch_val = smbchg_get_chargerid_switch_val,
	.need_to_check_ibatt = smbchg_need_to_check_ibatt,
	.get_chg_current_step = smbchg_get_chg_current_step,
#ifdef CONFIG_OPLUS_CHARGER_MTK
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
#endif
};
#endif
#endif /* OPLUS_FEATURE_CHG_BASIC */

#ifdef OPLUS_FEATURE_CHG_BASIC
#define REGULATOR_MODE_FAST			0x1
#define REGULATOR_MODE_NORMAL			0x2
#define REGULATOR_MODE_IDLE			0x4
#define REGULATOR_MODE_STANDBY			0x8

int pm660l_bob_regulator_get_mode(unsigned int *mode)
{
	int rc;
	unsigned int bob_mode;
	struct smb_charger *chg = NULL;

	if (!g_oplus_chip) {
		printk(KERN_ERR "pm660l_bob_regulator_get_mode: g_oplus_chip NULL\n");
		return -1;
	}
	chg = &g_oplus_chip->pmic_spmi.smb2_chip->chg;

	if (!chg || !chg->pm660l_bob_reg) {
		printk(KERN_ERR "%s: pm660l_bob_reg NULL\n", __func__);
		return -1;
	}

	rc = regulator_enable(chg->pm660l_bob_reg);
	if (rc < 0) {
		printk(KERN_ERR "%s: Couldn't enable regulator rc=%d\n", __func__, rc);
		return -1;
	}

	bob_mode = regulator_get_mode(chg->pm660l_bob_reg);
	if (bob_mode != REGULATOR_MODE_FAST && bob_mode != REGULATOR_MODE_NORMAL
			&& bob_mode != REGULATOR_MODE_IDLE && bob_mode != REGULATOR_MODE_STANDBY) {
		printk(KERN_ERR "%s: Couldn't get regulator mode=%d\n", __func__, bob_mode);
		*mode = 0;
		goto err;
	}
	*mode = bob_mode;

err:
	rc = regulator_disable(chg->pm660l_bob_reg);
	if (rc < 0) {
		printk(KERN_ERR "%s: Couldn't disable regulator rc=%d\n", __func__, rc);
		return -1;
	}

	return rc;
}
EXPORT_SYMBOL_GPL(pm660l_bob_regulator_get_mode);

/*return -1 for error, return 0 for ok, return 1 for invalid*/
int pm660l_bob_regulator_set_mode(unsigned int mode)
{
	int rc;
	int ua_load;
	static unsigned int pre_mode = 0;
	struct smb_charger *chg = NULL;
	struct oplus_chg_chip *chip = g_oplus_chip;
	
	if (!chip) {
		printk(KERN_ERR "%s: chip NULL\n", __func__);
		return -1;
	}
	chg = &chip->pmic_spmi.smb2_chip->chg;

	if (!chg || !chg->pm660l_bob_reg) {
		printk(KERN_ERR "%s: pm660l_bob_reg NULL\n", __func__);
		return -1;
	}

	if (mode != REGULATOR_MODE_FAST && mode != REGULATOR_MODE_NORMAL) {
		printk(KERN_ERR "%s: Invalid mode: %d", __func__, mode);
		return -1;
	}

	if (pre_mode == mode) {
		printk(KERN_ERR "%s: pre_mode[%d], mode[%d], return\n", __func__, pre_mode, mode);
		return 1;
	}

	if (mode == REGULATOR_MODE_FAST) {
			ua_load = 2000000;
		rc = regulator_set_load(chg->pm660l_bob_reg, ua_load);
		if (rc < 0) {
			printk(KERN_ERR "%s: Couldn't set regulator load=%d rc=%d\n", __func__, ua_load, rc);
			return -1;
		}
		pre_mode = mode;
	} else if (mode == REGULATOR_MODE_NORMAL) {
		ua_load = 0;
		rc = regulator_set_load(chg->pm660l_bob_reg, ua_load);
		if (rc < 0) {
			printk(KERN_ERR "%s: Couldn't set regulator load=%d rc=%d\n", __func__, ua_load, rc);
			return -1;
		}
		pre_mode = mode;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(pm660l_bob_regulator_set_mode);
#endif /* OPLUS_FEATURE_CHG_BASIC */

static int smb2_probe(struct platform_device *pdev)
{
#ifdef OPLUS_FEATURE_CHG_BASIC
	struct oplus_chg_chip *oplus_chip;
	struct power_supply *main_psy = NULL;
	union power_supply_propval pval = {0, };
#endif
	struct smb2 *chip;
	struct smb_charger *chg;
	int rc = 0;
	union power_supply_propval val;
	int usb_present, batt_present, batt_health, batt_charge_type;

#ifdef OPLUS_FEATURE_CHG_BASIC
	if(oplus_gauge_ic_chip_is_null() || oplus_vooc_check_chip_is_null() 
		|| oplus_charger_ic_chip_is_null())
	{
		chg_err("[oplus_chg_init] vooc || gauge || chg not ready, will do after bettery init.\n");
		return -EPROBE_DEFER;
	}

	chg_debug("SMB2_Probe Start----\n");
#endif

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

#ifdef OPLUS_FEATURE_CHG_BASIC
	oplus_chip = devm_kzalloc(&pdev->dev, sizeof(*oplus_chip), GFP_KERNEL);
	if (!oplus_chip)
		return -ENOMEM;

	oplus_chip->pmic_spmi.smb2_chip = chip;
	oplus_chip->chg_ops = (oplus_get_chg_ops());
	//oplus_chip->chg_ops = &smb2_chg_ops;
	oplus_chip->dev = &pdev->dev;
	g_oplus_chip = oplus_chip;
#endif
	chg = &chip->chg;
	chg->dev = &pdev->dev;
	chg->param = v1_params;
	chg->debug_mask = &__debug_mask;
	chg->mode = PARALLEL_MASTER;
	chg->irq_info = smb2_irqs;
	chg->name = "PMI";

#ifdef OPLUS_FEATURE_CHG_BASIC
	chg->pre_current_ma = -1;
#endif

#ifdef OPLUS_FEATURE_CHG_BASIC
	if (of_find_property(oplus_chip->dev->of_node, "qcom,pm660chg-vadc", NULL)) {
		oplus_chip->pmic_spmi.pm660_vadc_dev = qpnp_get_vadc(oplus_chip->dev, "pm660chg");
		if (IS_ERR(oplus_chip->pmic_spmi.pm660_vadc_dev)) {
			rc = PTR_ERR(oplus_chip->pmic_spmi.pm660_vadc_dev);
			oplus_chip->pmic_spmi.pm660_vadc_dev = NULL;
			if (rc != -EPROBE_DEFER)
				chg_err("Couldn't get vadc rc=%d\n", rc);
			else {
				chg_err("Couldn't get vadc, try again...\n");
				return -EPROBE_DEFER;
			}
		}
	}
#endif

	chg->regmap = dev_get_regmap(chg->dev->parent, NULL);
	if (!chg->regmap) {
		pr_err("parent regmap is missing\n");
		return -EINVAL;
	}

	rc = smb2_chg_config_init(chip);
	if (rc < 0) {
		if (rc != -EPROBE_DEFER)
			pr_err("Couldn't setup chg_config rc=%d\n", rc);
		return rc;
	}

	rc = smblib_init(chg);
	if (rc < 0) {
		pr_err("Smblib_init failed rc=%d\n", rc);
		goto cleanup;
	}

	rc = smb2_parse_dt(chip);
	if (rc < 0) {
		pr_err("Couldn't parse device tree rc=%d\n", rc);
		goto cleanup;
	}

	/* set driver data before resources request it */
	platform_set_drvdata(pdev, chip);

	rc = smb2_init_vbus_regulator(chip);
	if (rc < 0) {
		pr_err("Couldn't initialize vbus regulator rc=%d\n",
			rc);
		goto cleanup;
	}

	rc = smb2_init_vconn_regulator(chip);
	if (rc < 0) {
		pr_err("Couldn't initialize vconn regulator rc=%d\n",
				rc);
		goto cleanup;
	}

	/* extcon registration */
	chg->extcon = devm_extcon_dev_allocate(chg->dev, smblib_extcon_cable);
	if (IS_ERR(chg->extcon)) {
		rc = PTR_ERR(chg->extcon);
		dev_err(chg->dev, "failed to allocate extcon device rc=%d\n",
				rc);
		goto cleanup;
	}

	rc = devm_extcon_dev_register(chg->dev, chg->extcon);
	if (rc < 0) {
		dev_err(chg->dev, "failed to register extcon device rc=%d\n",
				rc);
		goto cleanup;
	}

	rc = smb2_init_hw(chip);
	if (rc < 0) {
		pr_err("Couldn't initialize hardware rc=%d\n", rc);
		goto cleanup;
	}

#ifndef OPLUS_FEATURE_CHG_BASIC
	rc = smb2_init_dc_psy(chip);
	if (rc < 0) {
		pr_err("Couldn't initialize dc psy rc=%d\n", rc);
		goto cleanup;
	}
#endif

#ifndef OPLUS_FEATURE_CHG_BASIC
	rc = smb2_init_ac_psy(chip);
	if (rc < 0) {
		pr_err("Couldn't initialize ac psy rc=%d\n", rc);
		goto cleanup;
	}
#endif
#ifndef OPLUS_FEATURE_CHG_BASIC
	rc = smb2_init_usb_psy(chip);
	if (rc < 0) {
		pr_err("Couldn't initialize usb psy rc=%d\n", rc);
		goto cleanup;
	}
#endif
//kong
    rc = oplus_power_supply_init(chip);
    if (rc < 0) {
            pr_err("Couldn't initialize usb main psy rc=%d\n", rc);
            goto cleanup;
    }

	rc = smb2_init_usb_main_psy(chip);
	if (rc < 0) {
		pr_err("Couldn't initialize usb psy rc=%d\n", rc);
		goto cleanup;
	}
#ifndef OPLUS_FEATURE_CHG_BASIC
	rc = smb2_init_batt_psy(chip);
	if (rc < 0) {
		pr_err("Couldn't initialize batt psy rc=%d\n", rc);
		goto cleanup;
	}
#endif
#ifdef OPLUS_FEATURE_CHG_BASIC
	if (oplus_chg_is_usb_present()) {
		rc = smblib_masked_write(chg, CHARGING_ENABLE_CMD_REG,
				CHARGING_ENABLE_CMD_BIT, 0);
		if (rc < 0)
			pr_err("Couldn't disable at bootup rc=%d\n", rc);
		msleep(100);
		rc = smblib_masked_write(chg, CHARGING_ENABLE_CMD_REG,
				CHARGING_ENABLE_CMD_BIT, CHARGING_ENABLE_CMD_BIT);
		if (rc < 0)
			pr_err("Couldn't enable at bootup rc=%d\n", rc);
	}
#endif

#ifdef OPLUS_FEATURE_CHG_BASIC
	oplus_chg_parse_dt(oplus_chip);
	oplus_chg_init(oplus_chip);
	main_psy = power_supply_get_by_name("main");
	if (main_psy) {
		pval.intval = 1000 * oplus_chg_get_fv(oplus_chip);
		power_supply_set_property(main_psy,
				POWER_SUPPLY_PROP_VOLTAGE_MAX,
				&pval);
		pval.intval = 1000 * oplus_chg_get_charging_current(oplus_chip);
		power_supply_set_property(main_psy,
				POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
				&pval);
	}
#endif

	rc = smb2_determine_initial_status(chip);
	if (rc < 0) {
		pr_err("Couldn't determine initial status rc=%d\n",
			rc);
		goto cleanup;
	}

	rc = smb2_request_interrupts(chip);
	if (rc < 0) {
		pr_err("Couldn't request interrupts rc=%d\n", rc);
		goto cleanup;
	}

#ifdef OPLUS_FEATURE_CHG_BASIC
	if (oplus_usbid_check_is_gpio(oplus_chip) == true)
		oplus_usbid_irq_register(oplus_chip);
#endif

	smb2_create_debugfs(chip);


#ifdef OPLUS_FEATURE_CHG_BASIC
//	g_oplus_chip->authenticate = oplus_gauge_get_batt_authenticate();
//	if(!g_oplus_chip->authenticate)
//		smbchg_charging_disble();
	oplus_chg_wake_update_work();
#endif

	rc = smblib_get_prop_usb_present(chg, &val);
	if (rc < 0) {
		pr_err("Couldn't get usb present rc=%d\n", rc);
		goto cleanup;
	}
	usb_present = val.intval;

	rc = smblib_get_prop_batt_present(chg, &val);
	if (rc < 0) {
		pr_err("Couldn't get batt present rc=%d\n", rc);
		goto cleanup;
	}
	batt_present = val.intval;

#ifndef OPLUS_FEATURE_CHG_BASIC
	rc = smblib_get_prop_batt_health(chg, &val);
	if (rc < 0) {
		pr_err("Couldn't get batt health rc=%d\n", rc);
		goto cleanup;
	}
	batt_health = val.intval;
#else
	batt_health = oplus_chg_get_prop_batt_health(oplus_chip);
#endif

	rc = smblib_get_prop_batt_charge_type(chg, &val);
	if (rc < 0) {
		pr_err("Couldn't get batt charge type rc=%d\n", rc);
		goto cleanup;
	}
	batt_charge_type = val.intval;

#ifdef OPLUS_FEATURE_CHG_BASIC
	init_proc_dump_registers_mask();
#endif

	pr_info("QPNP SMB2 probed successfully usb:present=%d type=%d batt:present = %d health = %d charge = %d\n",
		usb_present, chg->usb_psy_desc.type,
		batt_present, batt_health, batt_charge_type);

	return rc;

cleanup:
	smblib_deinit(chg);
	if (chg->usb_psy)
		power_supply_unregister(chg->usb_psy);
	if (chg->batt_psy)
		power_supply_unregister(chg->batt_psy);
	if (chg->ac_psy)
		power_supply_unregister(chg->ac_psy);
	if (chg->vconn_vreg && chg->vconn_vreg->rdev)
		regulator_unregister(chg->vconn_vreg->rdev);
	if (chg->vbus_vreg && chg->vbus_vreg->rdev)
		regulator_unregister(chg->vbus_vreg->rdev);
	platform_set_drvdata(pdev, NULL);
	return rc;
}

static int smb2_remove(struct platform_device *pdev)
{
	struct smb2 *chip = platform_get_drvdata(pdev);
	struct smb_charger *chg = &chip->chg;

	power_supply_unregister(chg->batt_psy);
	power_supply_unregister(chg->usb_psy);
	regulator_unregister(chg->vconn_vreg->rdev);
	regulator_unregister(chg->vbus_vreg->rdev);

	platform_set_drvdata(pdev, NULL);
	return 0;
}

static void smb2_shutdown(struct platform_device *pdev)
{
	struct smb2 *chip = platform_get_drvdata(pdev);
	struct smb_charger *chg = &chip->chg;

#ifdef OPLUS_FEATURE_CHG_BASIC
	if (g_oplus_chip) {
		smbchg_set_chargerid_switch_val(0);
		msleep(30);
	}
#endif

	/* configure power role for UFP */
	smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				TYPEC_POWER_ROLE_CMD_MASK, UFP_EN_CMD_BIT);

	/* force HVDCP to 5V */
	smblib_masked_write(chg, USBIN_OPTIONS_1_CFG_REG,
				HVDCP_AUTONOMOUS_MODE_EN_CFG_BIT, 0);
	smblib_write(chg, CMD_HVDCP_2_REG, FORCE_5V_BIT);

	/* force enable APSD */
	smblib_masked_write(chg, USBIN_OPTIONS_1_CFG_REG,
				 AUTO_SRC_DETECT_BIT, AUTO_SRC_DETECT_BIT);
}

static const struct of_device_id match_table[] = {
	{ .compatible = "qcom,qpnp-smb2", },
	{ },
};

static struct platform_driver smb2_driver = {
	.driver		= {
		.name		= "qcom,qpnp-smb2",
		.owner		= THIS_MODULE,
		.of_match_table	= match_table,
#ifdef OPLUS_FEATURE_CHG_BASIC
		.pm		= &smb2_pm_ops,
#endif
	},
	.probe		= smb2_probe,
	.remove		= smb2_remove,
	.shutdown	= smb2_shutdown,
};
module_platform_driver(smb2_driver);

MODULE_DESCRIPTION("QPNP SMB2 Charger Driver");
MODULE_LICENSE("GPL v2");
