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

#define pr_fmt(fmt)	"CHG: %s: " fmt, __func__

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/spmi.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/alarmtimer.h>
#include <linux/bitops.h>
#ifdef OPLUS_FEATURE_CHG_BASIC	/*Fuchun.Liao 2014-09-19 add*/
#include <mach/oplus_project.h>
#endif
#include "../oplus_vooc.h"
#include "../oplus_gauge.h"
#include "../oplus_charger.h"


#define CREATE_MASK(NUM_BITS, POS) \
	((unsigned char) (((1 << (NUM_BITS)) - 1) << (POS)))
#define LBC_MASK(MSB_BIT, LSB_BIT) \
	CREATE_MASK(MSB_BIT - LSB_BIT + 1, LSB_BIT)

/* Interrupt offsets */
#define INT_RT_STS_REG				0x10
#define FAST_CHG_ON_IRQ                         BIT(5)
#define OVERTEMP_ON_IRQ				BIT(4)
#define BAT_TEMP_OK_IRQ                         BIT(1)
#define BATT_PRES_IRQ                           BIT(0)

/* USB CHARGER PATH peripheral register offsets */
#define USB_PTH_STS_REG				0x09
#define USB_IN_VALID_MASK			LBC_MASK(7, 6)
#define USB_SUSP_REG				0x47
#define USB_SUSPEND_BIT				BIT(0)

/* CHARGER peripheral register offset */
#define CHG_OPTION_REG				0x08
#define CHG_OPTION_MASK				BIT(7)
#define CHG_STATUS_REG				0x09
#define CHG_VDD_LOOP_BIT			BIT(1)
#define CHG_VDD_MAX_REG				0x40
#define CHG_VDD_SAFE_REG			0x41
#define CHG_IBAT_MAX_REG			0x44
#define CHG_IBAT_SAFE_REG			0x45
#define CHG_VIN_MIN_REG				0x47
#define CHG_CTRL_REG				0x49
#define CHG_ENABLE				BIT(7)
#define CHG_FORCE_BATT_ON			BIT(0)
#define CHG_EN_MASK				(BIT(7) | BIT(0))
#define CHG_FAILED_REG				0x4A
#define CHG_FAILED_BIT				BIT(7)
#define CHG_VBAT_WEAK_REG			0x52
#define CHG_IBATTERM_EN_REG			0x5B
#define CHG_USB_ENUM_T_STOP_REG			0x4E
#define CHG_TCHG_MAX_EN_REG			0x60
#define CHG_TCHG_MAX_EN_BIT			BIT(7)
#define CHG_TCHG_MAX_MASK			LBC_MASK(6, 0)
#define CHG_TCHG_MAX_REG			0x61
#define CHG_WDOG_EN_REG				0x65
#define CHG_PERPH_RESET_CTRL3_REG		0xDA
#define CHG_COMP_OVR1				0xEE
#define CHG_VBAT_DET_OVR_MASK			LBC_MASK(1, 0)
#define OVERRIDE_0				0x2
#define OVERRIDE_NONE				0x0

/* BATTIF peripheral register offset */
#define BAT_IF_PRES_STATUS_REG			0x08
#define BATT_PRES_MASK				BIT(7)
#define BAT_IF_TEMP_STATUS_REG			0x09
#define BATT_TEMP_HOT_MASK			BIT(6)
#define BATT_TEMP_COLD_MASK			LBC_MASK(7, 6)
#define BATT_TEMP_OK_MASK			BIT(7)
#define BAT_IF_VREF_BAT_THM_CTRL_REG		0x4A
#define VREF_BATT_THERM_FORCE_ON		LBC_MASK(7, 6)
#define VREF_BAT_THM_ENABLED_FSM		BIT(7)
#define BAT_IF_BPD_CTRL_REG			0x48
#define BATT_BPD_CTRL_SEL_MASK			LBC_MASK(1, 0)
#define BATT_BPD_OFFMODE_EN			BIT(3)
#define BATT_THM_EN				BIT(1)
#define BATT_ID_EN				BIT(0)
#define BAT_IF_BTC_CTRL				0x49
#define BTC_COMP_EN_MASK			BIT(7)
#define BTC_COLD_MASK				BIT(1)
#define BTC_HOT_MASK				BIT(0)

/* MISC peripheral register offset */
#define MISC_REV2_REG				0x01
#define MISC_BOOT_DONE_REG			0x42
#define MISC_BOOT_DONE				BIT(7)
#define MISC_TRIM3_REG				0xF3
#define MISC_TRIM3_VDD_MASK			LBC_MASK(5, 4)
#define MISC_TRIM4_REG				0xF4
#define MISC_TRIM4_VDD_MASK			BIT(4)

#define PERP_SUBTYPE_REG			0x05
#define SEC_ACCESS                              0xD0

/* Linear peripheral subtype values */
#define LBC_CHGR_SUBTYPE			0x15
#define LBC_BAT_IF_SUBTYPE			0x16
#define LBC_USB_PTH_SUBTYPE			0x17
#define LBC_MISC_SUBTYPE			0x18

#define QPNP_CHG_I_MAX_MIN_90                   90

/* Feature flags */
#define VDD_TRIM_SUPPORTED			BIT(0)

#define QPNP_CHARGER_DEV_NAME	"qcom,qpnp-linear-charger"

#ifdef OPLUS_FEATURE_CHG_BASIC
struct qpnp_lbc_chip *qpnp_chip = NULL;
#endif /*OPLUS_FEATURE_CHG_BASIC*/

/* usb_interrupts */

struct qpnp_lbc_irq {
	int		irq;
	unsigned long	disabled;
	bool            is_wake;
};

enum {
	USBIN_VALID = 0,
	USB_OVER_TEMP,
	USB_CHG_GONE,
	BATT_PRES,
	BATT_TEMPOK,
	CHG_DONE,
	CHG_FAILED,
	CHG_FAST_CHG,
	CHG_VBAT_DET_LO,
	MAX_IRQS,
};

enum {
	USER	= BIT(0),
	THERMAL = BIT(1),
	CURRENT = BIT(2),
	SOC	= BIT(3),
};

enum bpd_type {
	BPD_TYPE_BAT_ID,
	BPD_TYPE_BAT_THM,
	BPD_TYPE_BAT_THM_BAT_ID,
};

static const char * const bpd_label[] = {
	[BPD_TYPE_BAT_ID] = "bpd_id",
	[BPD_TYPE_BAT_THM] = "bpd_thm",
	[BPD_TYPE_BAT_THM_BAT_ID] = "bpd_thm_id",
};

enum btc_type {
	HOT_THD_25_PCT = 25,
	HOT_THD_35_PCT = 35,
	COLD_THD_70_PCT = 70,
	COLD_THD_80_PCT = 80,
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


/*
 * struct qpnp_lbc_chip - device information
 * @dev:			device pointer to access the parent
 * @spmi:			spmi pointer to access spmi information
 * @chgr_base:			charger peripheral base address
 * @bat_if_base:		battery interface  peripheral base address
 * @usb_chgpth_base:		USB charge path peripheral base address
 * @misc_base:			misc peripheral base address
 * @bat_is_cool:		indicates that battery is cool
 * @bat_is_warm:		indicates that battery is warm
 * @chg_done:			indicates that charging is completed
 * @usb_present:		present status of USB
 * @batt_present:		present status of battery
 * @cfg_charging_disabled:	disable drawing current from USB.
 * @cfg_use_fake_battery:	flag to report default battery properties
 * @fastchg_on:			indicate charger in fast charge mode
 * @cfg_btc_disabled:		flag to disable btc (disables hot and cold
 *				irqs)
 * @cfg_max_voltage_mv:		the max volts the batt should be charged up to
 * @cfg_min_voltage_mv:		VIN_MIN configuration
 * @cfg_batt_weak_voltage_uv:	weak battery voltage threshold
 * @cfg_warm_bat_chg_ma:	warm battery maximum charge current in mA
 * @cfg_cool_bat_chg_ma:	cool battery maximum charge current in mA
 * @cfg_safe_voltage_mv:	safe voltage to which battery can charge
 * @cfg_warm_bat_mv:		warm temperature battery target voltage
 * @cfg_warm_bat_mv:		warm temperature battery target voltage
 * @cfg_cool_bat_mv:		cool temperature battery target voltage
 * @cfg_soc_resume_limit:	SOC at which battery resumes charging
 * @cfg_float_charge:		enable float charging
 * @charger_disabled:		maintain USB path state.
 * @cfg_charger_detect_eoc:	charger can detect end of charging
 * @cfg_disable_vbatdet_based_recharge:	keep VBATDET comparator overriden to
 *				low and VBATDET irq disabled.
 * @cfg_safe_current:		battery safety current setting
 * @cfg_hot_batt_p:		hot battery threshold setting
 * @cfg_cold_batt_p:		eold battery threshold setting
 * @cfg_warm_bat_decidegc:	warm battery temperature in degree Celsius
 * @cfg_cool_bat_decidegc:	cool battery temperature in degree Celsius
 * @fake_battery_soc:		SOC value to be reported to userspace
 * @cfg_tchg_mins:		maximum allowed software initiated charge time
 * @chg_failed_count:		counter to maintained number of times charging
 *				failed
 * @cfg_disable_follow_on_reset	charger ignore PMIC reset signal
 * @cfg_bpd_detection:		battery present detection mechanism selection
 * @cfg_thermal_levels:		amount of thermal mitigation levels
 * @cfg_thermal_mitigation:	thermal mitigation level values
 * @therm_lvl_sel:		thermal mitigation level selection
 * @jeita_configure_lock:	lock to serialize jeita configuration request
 * @hw_access_lock:		lock to serialize access to charger registers
 * @ibat_change_lock:		lock to serialize ibat change requests from
 *				USB and thermal.
 * @irq_lock			lock to serialize enabling/disabling of irq
 * @supported_feature_flag	bitmask for all supported features
 * @vddtrim_alarm		alarm to schedule trim work at regular
 *				interval
 * @vddtrim_work		work to perform actual vddmax trimming
 * @init_trim_uv		initial trim voltage at bootup
 * @delta_vddmax_uv		current vddmax trim voltage
 * @chg_enable_lock:		lock to serialize charging enable/disable for
 *				SOC based resume charging
 * @usb_psy:			power supply to export information to
 *				userspace
 * @bms_psy:			power supply to export information to
 *				userspace
 * @batt_psy:			power supply to export information to
 *				userspace
 */
struct qpnp_lbc_chip {
	struct device			*dev;
	struct spmi_device		*spmi;
	u16				chgr_base;
	u16				bat_if_base;
	u16				usb_chgpth_base;
	u16				misc_base;
	bool				bat_is_cool;
	bool				bat_is_warm;
	bool				chg_done;
	bool				usb_present;
	bool				batt_present;
	bool				cfg_charging_disabled;
	bool				cfg_btc_disabled;
	bool				cfg_use_fake_battery;
	bool				fastchg_on;
	bool				cfg_use_external_charger;
	unsigned int			cfg_warm_bat_chg_ma;
	unsigned int			cfg_cool_bat_chg_ma;
	unsigned int			cfg_safe_voltage_mv;
	unsigned int			cfg_max_voltage_mv;
	unsigned int			cfg_min_voltage_mv;
	unsigned int			cfg_charger_detect_eoc;
	unsigned int			cfg_disable_vbatdet_based_recharge;
	unsigned int			cfg_batt_weak_voltage_uv;
	unsigned int			cfg_warm_bat_mv;
	unsigned int			cfg_cool_bat_mv;
	unsigned int			cfg_hot_batt_p;
	unsigned int			cfg_cold_batt_p;
	unsigned int			cfg_thermal_levels;
	unsigned int			therm_lvl_sel;
	unsigned int			*thermal_mitigation;
	unsigned int			cfg_safe_current;
	unsigned int			cfg_tchg_mins;
	unsigned int			chg_failed_count;
	unsigned int			cfg_disable_follow_on_reset;
	unsigned int			supported_feature_flag;
	int				cfg_bpd_detection;
	int				cfg_warm_bat_decidegc;
	int				cfg_cool_bat_decidegc;
	int				fake_battery_soc;
	int				cfg_soc_resume_limit;
	int				cfg_float_charge;
	int				charger_disabled;
	int				prev_max_ma;
	int				usb_psy_ma;
	int				delta_vddmax_uv;
	int				init_trim_uv;
	struct alarm			vddtrim_alarm;
	struct work_struct		vddtrim_work;
	struct qpnp_lbc_irq		irqs[MAX_IRQS];
	struct mutex			jeita_configure_lock;
	struct mutex			chg_enable_lock;
	spinlock_t			ibat_change_lock;
	spinlock_t			hw_access_lock;
	spinlock_t			irq_lock;
	struct power_supply		*usb_psy;
	struct power_supply		*bms_psy;
	struct power_supply		batt_psy;
	struct qpnp_adc_tm_btm_param	adc_param;
	struct qpnp_vadc_chip		*vadc_dev;
	struct qpnp_adc_tm_chip		*adc_tm_dev;
};

static int __qpnp_lbc_read(struct spmi_device *spmi, u16 base,
			u8 *val, int count)
{
	int rc = 0;

	rc = spmi_ext_register_readl(spmi->ctrl, spmi->sid, base, val, count);
	if (rc)
		pr_err("SPMI read failed base=0x%02x sid=0x%02x rc=%d\n",
				base, spmi->sid, rc);

	return rc;
}

static int __qpnp_lbc_write(struct spmi_device *spmi, u16 base,
			u8 *val, int count)
{
	int rc;

	rc = spmi_ext_register_writel(spmi->ctrl, spmi->sid, base, val,
					count);
	if (rc)
		pr_err("SPMI write failed base=0x%02x sid=0x%02x rc=%d\n",
				base, spmi->sid, rc);

	return rc;
}

/*
static int __qpnp_lbc_secure_write(struct spmi_device *spmi, u16 base,
				u16 offset, u8 *val, int count)
{
	int rc;
	u8 reg_val;

	reg_val = 0xA5;
	rc = __qpnp_lbc_write(spmi, base + SEC_ACCESS, &reg_val, 1);
	if (rc) {
		pr_err("SPMI read failed base=0x%02x sid=0x%02x rc=%d\n",
				base + SEC_ACCESS, spmi->sid, rc);
		return rc;
	}

	rc = __qpnp_lbc_write(spmi, base + offset, val, 1);
	if (rc)
		pr_err("SPMI write failed base=0x%02x sid=0x%02x rc=%d\n",
				base + SEC_ACCESS, spmi->sid, rc);

	return rc;
}
*/

static int qpnp_lbc_read(struct qpnp_lbc_chip *chip, u16 base,
			u8 *val, int count)
{
	int rc = 0;
	struct spmi_device *spmi = chip->spmi;
	unsigned long flags;

	if (base == 0) {
		pr_err("base cannot be zero base=0x%02x sid=0x%02x rc=%d\n",
				base, spmi->sid, rc);
		return -EINVAL;
	}

	spin_lock_irqsave(&chip->hw_access_lock, flags);
	rc = __qpnp_lbc_read(spmi, base, val, count);
	spin_unlock_irqrestore(&chip->hw_access_lock, flags);

	return rc;
}

static int qpnp_lbc_write(struct qpnp_lbc_chip *chip, u16 base,
			u8 *val, int count)
{
	int rc = 0;
	struct spmi_device *spmi = chip->spmi;
	unsigned long flags;

	if (base == 0) {
		pr_err("base cannot be zero base=0x%02x sid=0x%02x rc=%d\n",
				base, spmi->sid, rc);
		return -EINVAL;
	}

	spin_lock_irqsave(&chip->hw_access_lock, flags);
	rc = __qpnp_lbc_write(spmi, base, val, count);
	spin_unlock_irqrestore(&chip->hw_access_lock, flags);

	return rc;
}

static int qpnp_lbc_masked_write(struct qpnp_lbc_chip *chip, u16 base,
				u8 mask, u8 val)
{
	int rc;
	u8 reg_val;
	struct spmi_device *spmi = chip->spmi;
	unsigned long flags;

	spin_lock_irqsave(&chip->hw_access_lock, flags);
	rc = __qpnp_lbc_read(spmi, base, &reg_val, 1);
	if (rc) {
		pr_err("spmi read failed: addr=%03X, rc=%d\n", base, rc);
		goto out;
	}
	pr_debug("addr = 0x%x read 0x%x\n", base, reg_val);

	reg_val &= ~mask;
	reg_val |= val & mask;

	pr_debug("writing to base=%x val=%x\n", base, reg_val);

	rc = __qpnp_lbc_write(spmi, base, &reg_val, 1);
	if (rc)
		pr_err("spmi write failed: addr=%03X, rc=%d\n", base, rc);

out:
	spin_unlock_irqrestore(&chip->hw_access_lock, flags);
	return rc;
}

/*
static int __qpnp_lbc_secure_masked_write(struct spmi_device *spmi, u16 base,
				u16 offset, u8 mask, u8 val)
{
	int rc;
	u8 reg_val, reg_val1;

	rc = __qpnp_lbc_read(spmi, base + offset, &reg_val, 1);
	if (rc) {
		pr_err("spmi read failed: addr=%03X, rc=%d\n", base, rc);
		return rc;
	}
	pr_debug("addr = 0x%x read 0x%x\n", base, reg_val);

	reg_val &= ~mask;
	reg_val |= val & mask;
	pr_debug("writing to base=%x val=%x\n", base, reg_val);

	reg_val1 = 0xA5;
	rc = __qpnp_lbc_write(spmi, base + SEC_ACCESS, &reg_val1, 1);
	if (rc) {
		pr_err("SPMI read failed base=0x%02x sid=0x%02x rc=%d\n",
				base + SEC_ACCESS, spmi->sid, rc);
		return rc;
	}

	rc = __qpnp_lbc_write(spmi, base + offset, &reg_val, 1);
	if (rc) {
		pr_err("SPMI write failed base=0x%02x sid=0x%02x rc=%d\n",
				base + offset, spmi->sid, rc);
		return rc;
	}

	return rc;
}
*/

//int qpnp_lbc_is_usb_chg_plugged_in(struct qpnp_lbc_chip *chip)
bool qpnp_lbc_is_usb_chg_plugged_in(void)
{
	u8 usbin_valid_rt_sts;
	int rc;
	struct qpnp_lbc_chip *chip = qpnp_chip;
#ifndef OPLUS_FEATURE_CHG_BASIC
	rc = qpnp_lbc_read(chip, chip->usb_chgpth_base + USB_PTH_STS_REG,
				&usbin_valid_rt_sts, 1);
	if (rc) {
		pr_err("spmi read failed: addr=%03X, rc=%d\n",
				chip->usb_chgpth_base + USB_PTH_STS_REG, rc);
		return rc;
	}

	pr_debug("usb_path_sts 0x%x\n", usbin_valid_rt_sts);

	return (usbin_valid_rt_sts & USB_IN_VALID_MASK) ? 1 : 0;
#else
	rc = qpnp_lbc_read(chip, chip->usb_chgpth_base + INT_RT_STS_REG,
				&usbin_valid_rt_sts, 1);
	if (rc) {
		pr_err("spmi read failed: addr=%03X, rc=%d\n",
				chip->usb_chgpth_base + INT_RT_STS_REG, rc);
		return rc;
	}

	pr_err("usb_path_sts 0x%x\n", usbin_valid_rt_sts);

	return (usbin_valid_rt_sts & 2) ? 1 : 0;
#endif
}

int qpnp_get_prop_charger_voltage_now(void)
{
	int rc = 0;
	int V_charger = 0;
	struct qpnp_vadc_result results;			
	// board version_B
	rc = qpnp_vadc_read(qpnp_chip->vadc_dev, USBIN, &results);
	if (rc) {
		pr_err("Unable to read vchg rc=%d\n", rc);
		return 0;
	}
	V_charger = (int)results.physical/1000;
	  	  
	return V_charger;//return (int)results.physical/1000;
}

int qpnp_get_prop_battery_voltage_now(void)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	rc = qpnp_vadc_read(qpnp_chip->vadc_dev, VBAT_SNS, &results);
	if (rc) {
		pr_err("Unable to read vbat rc=%d\n", rc);
		return 0;
	}

	return results.physical;
}

int qpnp_charger_type_get(void)
{
	union power_supply_propval ret = {0,};
	if(!qpnp_chip->usb_psy)	
		return POWER_SUPPLY_TYPE_UNKNOWN;
		
	qpnp_chip->usb_psy->get_property(qpnp_chip->usb_psy,
				  POWER_SUPPLY_PROP_TYPE, &ret);

	return ret.intval;
}


#if 0
static int qpnp_lbc_is_batt_present(struct qpnp_lbc_chip *chip)
{
	u8 batt_pres_rt_sts;
	int rc;

	rc = qpnp_lbc_read(chip, chip->bat_if_base + INT_RT_STS_REG,
				&batt_pres_rt_sts, 1);
	if (rc) {
		pr_err("spmi read failed: addr=%03X, rc=%d\n",
				chip->bat_if_base + INT_RT_STS_REG, rc);
		return rc;
	}

	return (batt_pres_rt_sts & BATT_PRES_IRQ) ? 1 : 0;
}






static int get_prop_batt_present(struct qpnp_lbc_chip *chip)
{
	u8 reg_val;
	int rc;

	rc = qpnp_lbc_read(chip, chip->bat_if_base + BAT_IF_PRES_STATUS_REG,
				&reg_val, 1);
	if (rc) {
		pr_err("Failed to read battery status read failed rc=%d\n",
				rc);
		return 0;
	}

	return (reg_val & BATT_PRES_MASK) ? 1 : 0;
}

static int get_prop_batt_health(struct qpnp_lbc_chip *chip)
{
	u8 reg_val;
	int rc;

	rc = qpnp_lbc_read(chip, chip->bat_if_base + BAT_IF_TEMP_STATUS_REG,
				&reg_val, 1);
	if (rc) {
		pr_err("Failed to read battery health rc=%d\n", rc);
		return POWER_SUPPLY_HEALTH_UNKNOWN;
	}

	if (BATT_TEMP_HOT_MASK & reg_val)
		return POWER_SUPPLY_HEALTH_OVERHEAT;
	if (!(BATT_TEMP_COLD_MASK & reg_val))
		return POWER_SUPPLY_HEALTH_COLD;
	if (chip->bat_is_cool)
		return POWER_SUPPLY_HEALTH_COOL;
	if (chip->bat_is_warm)
		return POWER_SUPPLY_HEALTH_WARM;

	return POWER_SUPPLY_HEALTH_GOOD;
}

static int get_prop_charge_type(struct qpnp_lbc_chip *chip)
{
	int rc;
	u8 reg_val;

	if (!get_prop_batt_present(chip))
		return POWER_SUPPLY_CHARGE_TYPE_NONE;

	rc = qpnp_lbc_read(chip, chip->chgr_base + INT_RT_STS_REG,
				&reg_val, 1);
	if (rc) {
		pr_err("Failed to read interrupt sts %d\n", rc);
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	}

	if (reg_val & FAST_CHG_ON_IRQ)
		return POWER_SUPPLY_CHARGE_TYPE_FAST;

	return POWER_SUPPLY_CHARGE_TYPE_NONE;
}

static int get_prop_batt_status(struct qpnp_lbc_chip *chip)
{
	int rc;
	u8 reg_val;

	if (qpnp_lbc_is_usb_chg_plugged_in(chip) && chip->chg_done)
		return POWER_SUPPLY_STATUS_FULL;

	rc = qpnp_lbc_read(chip, chip->chgr_base + INT_RT_STS_REG,
				&reg_val, 1);
	if (rc) {
		pr_err("Failed to read interrupt sts rc= %d\n", rc);
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	}

	if (reg_val & FAST_CHG_ON_IRQ)
		return POWER_SUPPLY_STATUS_CHARGING;

	return POWER_SUPPLY_STATUS_DISCHARGING;
}

static int get_prop_current_now(struct qpnp_lbc_chip *chip)
{
	union power_supply_propval ret = {0,};

	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
			  POWER_SUPPLY_PROP_CURRENT_NOW, &ret);
		return ret.intval;
	} else {
		pr_debug("No BMS supply registered return 0\n");
	}

	return 0;
}

#define DEFAULT_CAPACITY	50
static int get_prop_capacity(struct qpnp_lbc_chip *chip)
{
	union power_supply_propval ret = {0,};
	int soc ;

	if (chip->fake_battery_soc >= 0)
		return chip->fake_battery_soc;

	if (chip->cfg_use_fake_battery || !get_prop_batt_present(chip))
		return DEFAULT_CAPACITY;

	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_CAPACITY, &ret);
	

		soc = ret.intval;
		return soc;
	} else {
		pr_debug("No BMS supply registered return %d\n",
							DEFAULT_CAPACITY);
	}

	/*
	 * Return default capacity to avoid userspace
	 * from shutting down unecessarily
	 */
	return DEFAULT_CAPACITY;
}

#define DEFAULT_TEMP		250
static int get_prop_batt_temp(struct qpnp_lbc_chip *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	if (chip->cfg_use_fake_battery || !get_prop_batt_present(chip))
		return DEFAULT_TEMP;

	rc = qpnp_vadc_read(chip->vadc_dev, LR_MUX1_BATT_THERM, &results);
	if (rc) {
		pr_debug("Unable to read batt temperature rc=%d\n", rc);
		return DEFAULT_TEMP;
	}
	pr_debug("get_bat_temp %d, %lld\n", results.adc_code,
							results.physical);

	return (int)results.physical;
}
#endif

#define OF_PROP_READ(chip, prop, qpnp_dt_property, retval, optional)	\
do {									\
	if (retval)							\
		break;							\
									\
	retval = of_property_read_u32(chip->spmi->dev.of_node,		\
					"qcom," qpnp_dt_property,	\
					&chip->prop);			\
									\
	if ((retval == -EINVAL) && optional)				\
		retval = 0;						\
	else if (retval)						\
		pr_err("Error reading " #qpnp_dt_property		\
				" property rc = %d\n", rc);		\
} while (0)

static int qpnp_charger_read_dt_props(struct qpnp_lbc_chip *chip)
{
	int rc = 0;
	const char *bpd;

	OF_PROP_READ(chip, cfg_max_voltage_mv, "vddmax-mv", rc, 0);
	OF_PROP_READ(chip, cfg_safe_voltage_mv, "vddsafe-mv", rc, 0);
	OF_PROP_READ(chip, cfg_min_voltage_mv, "vinmin-mv", rc, 0);
	OF_PROP_READ(chip, cfg_safe_current, "ibatsafe-ma", rc, 0);
	if (rc)
		pr_err("Error reading required property rc=%d\n", rc);

	OF_PROP_READ(chip, cfg_tchg_mins, "tchg-mins", rc, 1);
	OF_PROP_READ(chip, cfg_warm_bat_decidegc, "warm-bat-decidegc", rc, 1);
	OF_PROP_READ(chip, cfg_cool_bat_decidegc, "cool-bat-decidegc", rc, 1);
	OF_PROP_READ(chip, cfg_hot_batt_p, "batt-hot-percentage", rc, 1);
	OF_PROP_READ(chip, cfg_cold_batt_p, "batt-cold-percentage", rc, 1);
	OF_PROP_READ(chip, cfg_batt_weak_voltage_uv, "vbatweak-uv", rc, 1);
	OF_PROP_READ(chip, cfg_soc_resume_limit, "resume-soc", rc, 1);
	if (rc) {
		pr_err("Error reading optional property rc=%d\n", rc);
		return rc;
	}

	rc = of_property_read_string(chip->spmi->dev.of_node,
						"qcom,bpd-detection", &bpd);
	if (rc) {

		chip->cfg_bpd_detection = BPD_TYPE_BAT_THM;
		rc = 0;
	} else {
		chip->cfg_bpd_detection = get_bpd(bpd);
		if (chip->cfg_bpd_detection < 0) {
			pr_err("Failed to determine bpd schema rc=%d\n", rc);
			return -EINVAL;
		}
	}

	/*
	 * Look up JEITA compliance parameters if cool and warm temp
	 * provided
	 */
	if (chip->cfg_cool_bat_decidegc || chip->cfg_warm_bat_decidegc) {
		chip->adc_tm_dev = qpnp_get_adc_tm(chip->dev, "chg");
		if (IS_ERR(chip->adc_tm_dev)) {
			rc = PTR_ERR(chip->adc_tm_dev);
			if (rc != -EPROBE_DEFER)
				pr_err("Failed to get adc-tm rc=%d\n", rc);
			return rc;
		}

		OF_PROP_READ(chip, cfg_warm_bat_chg_ma, "ibatmax-warm-ma",
				rc, 1);
		OF_PROP_READ(chip, cfg_cool_bat_chg_ma, "ibatmax-cool-ma",
				rc, 1);
		OF_PROP_READ(chip, cfg_warm_bat_mv, "warm-bat-mv", rc, 1);
		OF_PROP_READ(chip, cfg_cool_bat_mv, "cool-bat-mv", rc, 1);
		if (rc) {
			pr_err("Error reading battery temp prop rc=%d\n", rc);
			return rc;
		}
	}

	/* Get the btc-disabled property */
	chip->cfg_btc_disabled = of_property_read_bool(
			chip->spmi->dev.of_node, "qcom,btc-disabled");

	/* Get the charging-disabled property */
	chip->cfg_charging_disabled =
		of_property_read_bool(chip->spmi->dev.of_node,
					"qcom,charging-disabled");

	/* Get the fake-batt-values property */
	chip->cfg_use_fake_battery =
			of_property_read_bool(chip->spmi->dev.of_node,
					"qcom,use-default-batt-values");

	/* Get peripheral reset configuration property */
	chip->cfg_disable_follow_on_reset =
			of_property_read_bool(chip->spmi->dev.of_node,
					"qcom,disable-follow-on-reset");

	/* Get the float charging property */
	chip->cfg_float_charge =
			of_property_read_bool(chip->spmi->dev.of_node,
					"qcom,float-charge");

	/* Get the charger EOC detect property */
	chip->cfg_charger_detect_eoc =
			of_property_read_bool(chip->spmi->dev.of_node,
					"qcom,charger-detect-eoc");

	/* Get the vbatdet disable property */
	chip->cfg_disable_vbatdet_based_recharge =
			of_property_read_bool(chip->spmi->dev.of_node,
					"qcom,disable-vbatdet-based-recharge");
	/* Disable charging when faking battery values */
	if (chip->cfg_use_fake_battery)
		chip->cfg_charging_disabled = true;

	chip->cfg_use_external_charger = of_property_read_bool(
			chip->spmi->dev.of_node, "qcom,use-external-charger");

	if (of_find_property(chip->spmi->dev.of_node,
					"qcom,thermal-mitigation",
					&chip->cfg_thermal_levels)) {
		chip->thermal_mitigation = devm_kzalloc(chip->dev,
			chip->cfg_thermal_levels,
			GFP_KERNEL);

		if (chip->thermal_mitigation == NULL) {
			pr_err("thermal mitigation kzalloc() failed.\n");
			return -ENOMEM;
		}

		chip->cfg_thermal_levels /= sizeof(int);
		rc = of_property_read_u32_array(chip->spmi->dev.of_node,
				"qcom,thermal-mitigation",
				chip->thermal_mitigation,
				chip->cfg_thermal_levels);
		if (rc) {
			pr_err("Failed to read threm limits rc = %d\n", rc);
			return rc;
		}
	}

	return rc;
}

#ifdef OPLUS_FEATURE_CHG_BASIC
void opchg_usbin_valid_irq_handler(void)
{
	int usb_present;
	usb_present = qpnp_lbc_is_usb_chg_plugged_in();
	if(oplus_gauge_check_chip_is_null() || oplus_vooc_check_chip_is_null() || oplus_chg_check_chip_is_null())
	{
		pr_err("[do_chrdet_int_task] vooc || gauge || charger not ready, will do after bettery init.\n");
		return ;
	}
	
	if(oplus_vooc_get_fastchg_started() == true){
		pr_err("[do_chrdet_int_task] opchg_get_prop_fast_chg_started = true!\n");
		return ;
	}
	power_supply_set_present(qpnp_chip->usb_psy, usb_present);
	//oplus_charger_detect_check();
	//oplus_chg_wake_update_work();
}
#endif /*OPLUS_FEATURE_CHG_BASIC*/

static irqreturn_t qpnp_lbc_usbin_valid_irq_handler(int irq, void *_chip)
{
	int usb_present;
	//struct qpnp_lbc_chip *chip = _chip;
	usb_present = qpnp_lbc_is_usb_chg_plugged_in();
	pr_err("usbin-valid triggered: %d\n", usb_present);
	
	
	opchg_usbin_valid_irq_handler();
	//power_supply_set_supply_type(chip->usb_psy, POWER_SUPPLY_TYPE_USB);
	//power_supply_set_present(chip->usb_psy, usb_present);
	return IRQ_HANDLED;
}

/*
static int qpnp_lbc_is_batt_temp_ok(struct qpnp_lbc_chip *chip)
{
	u8 reg_val;
	int rc;

	rc = qpnp_lbc_read(chip, chip->bat_if_base + INT_RT_STS_REG,
				&reg_val, 1);
	if (rc) {
		pr_err("spmi read failed: addr=%03X, rc=%d\n",
				chip->bat_if_base + INT_RT_STS_REG, rc);
		return rc;
	}

	return (reg_val & BAT_TEMP_OK_IRQ) ? 1 : 0;
}
*/



static int qpnp_disable_lbc_charger(struct qpnp_lbc_chip *chip)
{
	int rc;
	u8 reg;

	reg = CHG_FORCE_BATT_ON;
	rc = qpnp_lbc_masked_write(chip, chip->chgr_base + CHG_CTRL_REG,
							CHG_EN_MASK, reg);
	/* disable BTC */
	rc |= qpnp_lbc_masked_write(chip, chip->bat_if_base + BAT_IF_BTC_CTRL,
							BTC_COMP_EN_MASK, 0);
	/* Enable BID and disable THM based BPD */
	reg = BATT_ID_EN | BATT_BPD_OFFMODE_EN;
	rc |= qpnp_lbc_write(chip, chip->bat_if_base + BAT_IF_BPD_CTRL_REG,
								&reg, 1);
	return rc;
}

#define SPMI_REQUEST_IRQ(chip, idx, rc, irq_name, threaded, flags, wake)\
do {									\
	if (rc)								\
		break;							\
	if (chip->irqs[idx].irq) {					\
		if (threaded)						\
			rc = devm_request_threaded_irq(chip->dev,	\
				chip->irqs[idx].irq, NULL,		\
				qpnp_lbc_##irq_name##_irq_handler,	\
				flags, #irq_name, chip);		\
		else							\
			rc = devm_request_irq(chip->dev,		\
				chip->irqs[idx].irq,			\
				qpnp_lbc_##irq_name##_irq_handler,	\
				flags, #irq_name, chip);		\
		if (rc < 0) {						\
			pr_err("Unable to request " #irq_name " %d\n",	\
								rc);	\
		} else {						\
			rc = 0;						\
			if (wake) {					\
				enable_irq_wake(chip->irqs[idx].irq);	\
				chip->irqs[idx].is_wake = true;		\
			}						\
		}							\
	}								\
} while (0)

#define SPMI_GET_IRQ_RESOURCE(chip, rc, resource, idx, name)		\
do {									\
	if (rc)								\
		break;							\
									\
	rc = spmi_get_irq_byname(chip->spmi, resource, #name);		\
	if (rc < 0) {							\
		pr_err("Unable to get irq resource " #name "%d\n", rc);	\
	} else {							\
		chip->irqs[idx].irq = rc;				\
		rc = 0;							\
	}								\
} while (0)

/*
static int qpnp_lbc_request_irqs(struct qpnp_lbc_chip *chip)
{
	int rc = 0;

	SPMI_REQUEST_IRQ(chip, CHG_FAILED, rc, chg_failed, 0,
			IRQF_TRIGGER_RISING, 1);

	SPMI_REQUEST_IRQ(chip, CHG_FAST_CHG, rc, fastchg, 1,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING
			| IRQF_ONESHOT, 1);

	SPMI_REQUEST_IRQ(chip, CHG_DONE, rc, chg_done, 0,
			IRQF_TRIGGER_RISING, 0);

	SPMI_REQUEST_IRQ(chip, CHG_VBAT_DET_LO, rc, vbatdet_lo, 0,
			IRQF_TRIGGER_FALLING, 1);

	SPMI_REQUEST_IRQ(chip, BATT_PRES, rc, batt_pres, 1,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING
			| IRQF_ONESHOT, 1);

	SPMI_REQUEST_IRQ(chip, BATT_TEMPOK, rc, batt_temp, 0,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, 1);

	SPMI_REQUEST_IRQ(chip, USB_OVER_TEMP, rc, usb_overtemp, 0,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, 0);

	SPMI_REQUEST_IRQ(chip, USBIN_VALID, rc, usbin_valid, 0,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, 1);

	

	return 0;
}
*/


#ifdef OPLUS_FEATURE_CHG_BASIC
static int qpnp_lbc_request_usbin_valid_irq(struct qpnp_lbc_chip *chip)
{
	int rc = 0;

	SPMI_REQUEST_IRQ(chip, USBIN_VALID, rc, usbin_valid, 0,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, 1);
	return 0;
}
#endif	/*OPLUS_FEATURE_CHG_BASIC*/
static int qpnp_lbc_get_irqs(struct qpnp_lbc_chip *chip, u8 subtype,
					struct spmi_resource *spmi_resource)
{
	int rc = 0;

	switch (subtype) {
	case LBC_CHGR_SUBTYPE:
		SPMI_GET_IRQ_RESOURCE(chip, rc, spmi_resource,
						CHG_FAST_CHG, fast-chg-on);
		SPMI_GET_IRQ_RESOURCE(chip, rc, spmi_resource,
						CHG_FAILED, chg-failed);
		if (!chip->cfg_disable_vbatdet_based_recharge)
			SPMI_GET_IRQ_RESOURCE(chip, rc, spmi_resource,
						CHG_VBAT_DET_LO, vbat-det-lo);
		if (chip->cfg_charger_detect_eoc)
			SPMI_GET_IRQ_RESOURCE(chip, rc, spmi_resource,
						CHG_DONE, chg-done);
		break;
	case LBC_BAT_IF_SUBTYPE:
		SPMI_GET_IRQ_RESOURCE(chip, rc, spmi_resource,
						BATT_PRES, batt-pres);
		SPMI_GET_IRQ_RESOURCE(chip, rc, spmi_resource,
						BATT_TEMPOK, bat-temp-ok);
		break;
	case LBC_USB_PTH_SUBTYPE:
		SPMI_GET_IRQ_RESOURCE(chip, rc, spmi_resource,
						USBIN_VALID, usbin-valid);
		SPMI_GET_IRQ_RESOURCE(chip, rc, spmi_resource,
						USB_OVER_TEMP, usb-over-temp);
		break;
	};

	return 0;
}

/* Get/Set initial state of charger */
/*
static void determine_initial_status(struct qpnp_lbc_chip *chip)
{

	chip->usb_present = qpnp_lbc_is_usb_chg_plugged_in();
	pr_err("%s usb_present:%d\n",__func__,chip->usb_present);

	opchg_usbin_valid_irq_handler();

}
*/
#ifdef OPLUS_FEATURE_CHG_BASIC
#define BMS_VM_BMS_DATA_REG_0			0x40B0
int qpnp_set_pmic_soc_memory(int soc)
{
	int rc = 0;
	u8 soc_temp = 0;
	
	if(qpnp_chip == NULL){
		pr_debug("%s qpnp_chip is NULL\n",__func__);
		return rc;
	}
	soc_temp = soc;
	rc = qpnp_lbc_write(qpnp_chip, BMS_VM_BMS_DATA_REG_0, &soc_temp, 1);
	if(rc)
		pr_err("%s fail,rc:%d\n",__func__,rc);
	return rc;
}

int qpnp_get_pmic_soc_memory(void)
{
	int rc = 0;
	u8 reg = 0;
	
	if(qpnp_chip == NULL){
		pr_debug("%s qpnp_chip is NULL\n",__func__);
		return reg;
	}
	rc = qpnp_lbc_read(qpnp_chip, BMS_VM_BMS_DATA_REG_0, &reg, 1);
	if(rc)
		pr_err("%s fail,rc:%d\n",__func__,rc);
	
	return reg;
}

int opchg_get_charger_inout(void)
{
	int charger_in=0;
	
	charger_in= qpnp_lbc_is_usb_chg_plugged_in();	
	return charger_in;
}
#endif /*OPLUS_FEATURE_CHG_BASIC*/

bool oplus_pmic_check_chip_is_null(void)
{
	if(!qpnp_chip) 
		return true;
	else
		return false;
}

static int qpnp_lbc_probe(struct spmi_device *spmi)
{
	u8 subtype;
	struct qpnp_lbc_chip *chip;
	struct resource *resource;
	struct spmi_resource *spmi_resource;
	struct power_supply *usb_psy;
	//struct power_supply *batt_psy;
	int rc = 0;

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		pr_err("usb supply not found deferring probe\n");
		return -EPROBE_DEFER;
	}/*
	batt_psy = power_supply_get_by_name("battery");
	if (!batt_psy) {
		pr_err("battery supply not found deferring probe\n");
		return -EPROBE_DEFER;
	}*/
	chip = devm_kzalloc(&spmi->dev, sizeof(struct qpnp_lbc_chip),
				GFP_KERNEL);
	if (!chip) {
		pr_err("memory allocation failed.\n");
		return -ENOMEM;
	}

	chip->usb_psy = usb_psy;
	chip->dev = &spmi->dev;
	chip->spmi = spmi;
	chip->fake_battery_soc = -EINVAL;
	dev_set_drvdata(&spmi->dev, chip);
	device_init_wakeup(&spmi->dev, 1);
	spin_lock_init(&chip->irq_lock);



	/* Get all device-tree properties */
	rc = qpnp_charger_read_dt_props(chip);
	if (rc) {
		pr_err("Failed to read DT properties rc=%d\n", rc);
		return rc;
	}

	spmi_for_each_container_dev(spmi_resource, spmi) {
		if (!spmi_resource) {
			pr_err("spmi resource absent\n");
			rc = -ENXIO;
			goto fail_chg_enable;
		}

		resource = spmi_get_resource(spmi, spmi_resource,
							IORESOURCE_MEM, 0);
		if (!(resource && resource->start)) {
			pr_err("node %s IO resource absent!\n",
						spmi->dev.of_node->full_name);
			rc = -ENXIO;
			goto fail_chg_enable;
		}

		rc = qpnp_lbc_read(chip, resource->start + PERP_SUBTYPE_REG,
					&subtype, 1);
		if (rc) {
			pr_err("Peripheral subtype read failed rc=%d\n", rc);
			goto fail_chg_enable;
		}

		switch (subtype) {
		case LBC_CHGR_SUBTYPE:
			chip->chgr_base = resource->start;

			/* Get Charger peripheral irq numbers */
			rc = qpnp_lbc_get_irqs(chip, subtype, spmi_resource);
			if (rc) {
				pr_err("Failed to get CHGR irqs rc=%d\n", rc);
				goto fail_chg_enable;
			}
			break;
		case LBC_USB_PTH_SUBTYPE:
			chip->usb_chgpth_base = resource->start;
			rc = qpnp_lbc_get_irqs(chip, subtype, spmi_resource);
			if (rc) {
				pr_err("Failed to get USB_PTH irqs rc=%d\n",
						rc);
				goto fail_chg_enable;
			}
			break;
		case LBC_BAT_IF_SUBTYPE:
			chip->bat_if_base = resource->start;
			chip->vadc_dev = qpnp_get_vadc(chip->dev, "chg");
			if (IS_ERR(chip->vadc_dev)) {
				rc = PTR_ERR(chip->vadc_dev);
				if (rc != -EPROBE_DEFER)
					pr_err("vadc prop missing rc=%d\n",
							rc);
				goto fail_chg_enable;
			}
			/* Get Charger Batt-IF peripheral irq numbers */
			rc = qpnp_lbc_get_irqs(chip, subtype, spmi_resource);
			if (rc) {
				pr_err("Failed to get BAT_IF irqs rc=%d\n", rc);
				goto fail_chg_enable;
			}
			break;
		case LBC_MISC_SUBTYPE:
			chip->misc_base = resource->start;
			break;
		default:
			pr_err("Invalid peripheral subtype=0x%x\n", subtype);
			rc = -EINVAL;
		}
	}


	//determine_initial_status(chip);
	rc = qpnp_lbc_request_usbin_valid_irq(chip);
	if (rc) {
		pr_err("unable to initialize LBC MISC rc=%d\n", rc);
		goto fail_chg_enable;
	}
	rc = qpnp_disable_lbc_charger(chip);
	if (rc)
		pr_err("Unable to disable charger rc=%d\n", rc);
	
	#ifdef OPLUS_FEATURE_CHG_BASIC
	qpnp_chip = chip;
#endif /*OPLUS_FEATURE_CHG_BASIC*/
	pr_err("%s success\n",__func__);
	
	return 0;

/*
unregister_batt:
	if (chip->bat_if_base)
		power_supply_unregister(&chip->batt_psy);
*/
fail_chg_enable:
	dev_set_drvdata(&spmi->dev, NULL);
	return rc;
}

static int qpnp_lbc_remove(struct spmi_device *spmi)
{
	/*struct qpnp_lbc_chip *chip = dev_get_drvdata(&spmi->dev);
	
	if (chip->bat_if_base)
		power_supply_unregister(&chip->batt_psy);
	*/
	dev_set_drvdata(&spmi->dev, NULL);
	return 0;
}

static struct of_device_id qpnp_lbc_match_table[] = {
	{ .compatible = QPNP_CHARGER_DEV_NAME, },
	{}
};

static struct spmi_driver qpnp_lbc_driver = {
	.probe		= qpnp_lbc_probe,
	.remove		= qpnp_lbc_remove,
	.driver		= {
		.name		= QPNP_CHARGER_DEV_NAME,
		.owner		= THIS_MODULE,
		.of_match_table	= qpnp_lbc_match_table,
	},
};

/*
 * qpnp_lbc_init() - register spmi driver for qpnp-chg
 */
static int __init qpnp_lbc_init(void)
{
	return spmi_driver_register(&qpnp_lbc_driver);
}
module_init(qpnp_lbc_init);

static void __exit qpnp_lbc_exit(void)
{
	spmi_driver_unregister(&qpnp_lbc_driver);
}
module_exit(qpnp_lbc_exit);

MODULE_DESCRIPTION("QPNP Linear charger driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" QPNP_CHARGER_DEV_NAME);
