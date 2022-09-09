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

#ifndef __OPLUS_BQ25601D_H__

#define __OPLUS_BQ25601D_H__

#include <linux/power_supply.h>
#ifdef CONFIG_OPLUS_CHARGER_MTK
#include <mt-plat/charger_type.h>
#endif

#include "../oplus_charger.h"


/* Address:00h */
#define REG00_BQ25601D_ADDRESS							0x00

#define REG00_BQ25601D_SUSPEND_MODE_MASK				BIT(7)
#define REG00_BQ25601D_SUSPEND_MODE_DISABLE				0x00
#define REG00_BQ25601D_SUSPEND_MODE_ENABLE				BIT(7)

#define REG00_BQ25601D_INPUT_CURRENT_LIMIT_MASK			(BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0))
#define REG00_BQ25601D_INPUT_CURRENT_LIMIT_SHIFT			0
#define REG00_BQ25601D_INPUT_CURRENT_LIMIT_100MA		0x00
#define REG00_BQ25601D_INPUT_CURRENT_LIMIT_300MA		BIT(1)
#define REG00_BQ25601D_INPUT_CURRENT_LIMIT_500MA		BIT(2)
#define REG00_BQ25601D_INPUT_CURRENT_LIMIT_900MA		BIT(3)
#define REG00_BQ25601D_INPUT_CURRENT_LIMIT_1200MA		(BIT(3) | BIT(1) | BIT(0))
#define REG00_BQ25601D_INPUT_CURRENT_LIMIT_1500MA		(BIT(3) | BIT(2) | BIT(1))
#define REG00_BQ25601D_INPUT_CURRENT_LIMIT_2000MA		(BIT(4) | BIT(1) | BIT(0))
#define REG00_BQ25601D_INPUT_CURRENT_LIMIT_3000MA		(BIT(4) | BIT(3) | BIT(2) | BIT(0))


/* Address:01h */
#define REG01_BQ25601D_ADDRESS							0x01

#define REG01_BQ25601D_PFM_DIS_MASK						BIT(7)
#define REG01_BQ25601D_PFM_DIS_ENABLE					0x00
#define REG01_BQ25601D_PFM_DIS_DISABLE					BIT(7)

#define REG01_BQ25601D_WDT_TIMER_RESET_MASK				BIT(6)
#define REG01_BQ25601D_WDT_TIMER_NORMAL					0x00
#define REG01_BQ25601D_WDT_TIMER_RESET					BIT(6)

#define REG01_BQ25601D_OTG_MASK							BIT(5)
#define REG01_BQ25601D_OTG_DISABLE						0x00
#define REG01_BQ25601D_OTG_ENABLE						BIT(5)

#define REG01_BQ25601D_CHARGING_MASK					BIT(4)
#define REG01_BQ25601D_CHARGING_DISABLE					0x00
#define REG01_BQ25601D_CHARGING_ENABLE					BIT(4)

#define REG01_BQ25601D_SYS_VOL_LIMIT_MASK				(BIT(3) | BIT(2) | BIT(1))
#define REG01_BQ25601D_SYS_VOL_LIMIT_2600MV				0x00
#define REG01_BQ25601D_SYS_VOL_LIMIT_2800MV				BIT(1)
#define REG01_BQ25601D_SYS_VOL_LIMIT_3000MV				BIT(2)
#define REG01_BQ25601D_SYS_VOL_LIMIT_3200MV				(BIT(2) | BIT(1))
#define REG01_BQ25601D_SYS_VOL_LIMIT_3400MV				BIT(3)
#define REG01_BQ25601D_SYS_VOL_LIMIT_3500MV				(BIT(3) | BIT(1))
#define REG01_BQ25601D_SYS_VOL_LIMIT_3600MV				(BIT(3) | BIT(2))
#define REG01_BQ25601D_SYS_VOL_LIMIT_3700MV				(BIT(3) | BIT(2) | BIT(1))

#define REG01_BQ25601D_VBAT_FALLING_MASK				BIT(0)
#define REG01_BQ25601D_VBAT_FALLING_2800MV				0x00
#define REG01_BQ25601D_VBAT_FALLING_2500MV				BIT(0)


/* Address:02h */
#define REG02_BQ25601D_ADDRESS							0x02

#define REG02_BQ25601D_OTG_CURRENT_LIMIT_MASK			BIT(7)
#define REG02_BQ25601D_OTG_CURRENT_LIMIT_500MA			0x00
#define REG02_BQ25601D_OTG_CURRENT_LIMIT_1200MA			BIT(7)

#define REG02_BQ25601D_FAST_CHG_CURRENT_LIMIT_MASK		(BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0))
#define REG02_BQ25601D_FAST_CHG_CURRENT_LIMIT_SHIFT		0
#define REG02_BQ25601D_MIN_FAST_CHG_CURRENT_MA			0
#define REG02_BQ25601D_MAX_FAST_CHG_CURRENT_MA		3000
#define REG02_BQ25601D_FAST_CHG_CURRENT_STEP_MA		60


/* Address:03h */
#define REG03_BQ25601D_ADDRESS							0x03

#define REG03_BQ25601D_PRE_CHG_CURRENT_LIMIT_MASK		(BIT(7) | BIT(6) | BIT(5) | BIT(4))
#define REG03_BQ25601D_PRE_CHG_CURRENT_LIMIT_SHIFT		4
#define REG03_BQ25601D_MIN_PRE_CHG_CURRENT_MA			60
#define REG03_BQ25601D_MAX_PRE_CHG_CURRENT_MA			960
#define REG03_BQ25601D_PRE_CHG_CURRENT_STEP_MA			60

#define REG03_BQ25601D_TERM_CHG_CURRENT_LIMIT_MASK		(BIT(3) | BIT(2) | BIT(1) | BIT(0))
#define REG03_BQ25601D_TERM_CHG_CURRENT_LIMIT_SHIFT	0
#define REG03_BQ25601D_MIN_TERM_CHG_CURRENT_MA			60
#define REG03_BQ25601D_MAX_TERM_CHG_CURRENT_MA		960
#define REG03_BQ25601D_TERM_CHG_CURRENT_STEP_MA		60


/* Address:04h */
#define REG04_BQ25601D_ADDRESS							0x04

#define REG04_BQ25601D_CHG_VOL_LIMIT_MASK				(BIT(7) | BIT(6) | BIT(5) | BIT(4) | BIT(3))
#define REG04_BQ25601D_CHG_VOL_LIMIT_SHIFT				3
#define REG04_BQ25601D_MIN_FLOAT_MV						3847
#define REG04_BQ25601D_MAX_FLOAT_MV						4615
#define REG04_BQ25601D_VFLOAT_STEP_MV					32

#define REG04_BQ25601D_RECHG_THRESHOLD_VOL_MASK		BIT(0)
#define REG04_BQ25601D_RECHG_THRESHOLD_VOL_100MV		0x00
#define REG04_BQ25601D_RECHG_THRESHOLD_VOL_200MV		BIT(0)


/* Address:05h */
#define REG05_BQ25601D_ADDRESS							0x05

#define REG05_BQ25601D_TERMINATION_MASK					BIT(7)
#define REG05_BQ25601D_TERMINATION_DISABLE				0x00
#define REG05_BQ25601D_TERMINATION_ENABLE				BIT(7)

#define REG05_BQ25601D_WATCHDOG_TIMER_MASK				(BIT(5) | BIT(4))
#define REG05_BQ25601D_WATCHDOG_TIMER_DISABLE			0x00
#define REG05_BQ25601D_WATCHDOG_TIMER_40S				BIT(4)
#define REG05_BQ25601D_WATCHDOG_TIMER_80S				BIT(5)
#define REG05_BQ25601D_WATCHDOG_TIMER_160S				(BIT(5) | BIT(4))

#define REG05_BQ25601D_CHG_SAFETY_TIMER_MASK			BIT(3)
#define REG05_BQ25601D_CHG_SAFETY_TIMER_DISABLE			0x00
#define REG05_BQ25601D_CHG_SAFETY_TIMER_ENABLE			BIT(3)

#define REG05_BQ25601D_FAST_CHG_TIMEOUT_MASK			BIT(2)
#define REG05_BQ25601D_FAST_CHG_TIMEOUT_5H				0x00
#define REG05_BQ25601D_FAST_CHG_TIMEOUT_10H				BIT(2)


/* Address:06h */
#define REG06_BQ25601D_ADDRESS							0x06

#define REG06_BQ25601D_OTG_VLIM_MASK						(BIT(5) | BIT(4))
#define REG06_BQ25601D_OTG_VLIM_5150MV					BIT(5)

#define REG06_BQ25601D_VINDPM_MASK						(BIT(3) | BIT(2) | BIT(1) | BIT(0))
#define REG06_BQ25601D_VINDPM_STEP_MV					100
#define REG06_BQ25601D_VINDPM_OFFSET						3900
#define REG06_BQ25601D_VINDPM_SHIFT						0


/* Address:07h */
#define REG07_BQ25601D_ADDRESS							0x07

#define REG07_BQ25601D_IINDET_EN_MASE					BIT(7)
#define REG07_BQ25601D_IINDET_EN_DET_COMPLETE			0x00
#define REG07_BQ25601D_IINDET_EN_FORCE_DET				BIT(7)

/* Address:08h */
#define REG08_BQ25601D_ADDRESS							0x08

#define REG08_BQ25601D_VBUS_STAT_MASK					(BIT(7) | BIT(6) | BIT(5))
#define REG08_BQ25601D_VBUS_STAT_UNKNOWN				0x00
#define REG08_BQ25601D_VBUS_STAT_SDP						BIT(5)
#define REG08_BQ25601D_VBUS_STAT_CDP						BIT(6)
#define REG08_BQ25601D_VBUS_STAT_DCP						(BIT(6) | BIT(5))
#define REG08_BQ25601D_VBUS_STAT_OCP						(BIT(7) | BIT(5))
#define REG08_BQ25601D_VBUS_STAT_FLOAT					(BIT(7) | BIT(6))
#define REG08_BQ25601D_VBUS_STAT_OTG_MODE				(BIT(7) | BIT(6) | BIT(5))

#define REG08_BQ25601D_CHG_STAT_MASK						(BIT(4) | BIT(3))
#define REG08_BQ25601D_CHG_STAT_NO_CHARGING			0x00
#define REG08_BQ25601D_CHG_STAT_PRE_CHARGING			BIT(3)
#define REG08_BQ25601D_CHG_STAT_FAST_CHARGING			BIT(4)
#define REG08_BQ25601D_CHG_STAT_CHG_TERMINATION		(BIT(4) | BIT(3))

#define REG08_BQ25601D_POWER_GOOD_STAT_MASK			BIT(2)
#define REG08_BQ25601D_POWER_GOOD_STAT_NOT_GOOD		0x00
#define REG08_BQ25601D_POWER_GOOD_STAT_GOOD			BIT(2)


/* Address:09h */
#define REG09_BQ25601D_ADDRESS							0x09

#define REG09_BQ25601D_WDT_FAULT_MASK					BIT(7)
#define REG09_BQ25601D_WDT_FAULT_NORMAL					0x00
#define REG09_BQ25601D_WDT_FAULT_EXPIRATION				BIT(7)

#define REG09_BQ25601D_OTG_FAULT_MASK					BIT(6)
#define REG09_BQ25601D_OTG_FAULT_NORMAL					0x00
#define REG09_BQ25601D_OTG_FAULT_ABNORMAL				BIT(6)

#define REG09_BQ25601D_CHG_FAULT_MASK					(BIT(5) | BIT(4))
#define REG09_BQ25601D_CHG_FAULT_NORMAL					0x00
#define REG09_BQ25601D_CHG_FAULT_INPUT_ERROR			BIT(4)
#define REG09_BQ25601D_CHG_FAULT_THERM_SHUTDOWN		BIT(5)
#define REG09_BQ25601D_CHG_FAULT_TIMEOUT_ERROR			(BIT(5) | BIT(4))

#define REG09_BQ25601D_BAT_FAULT_MASK					BIT(3)
#define REG09_BQ25601D_BAT_FAULT_NORMAL					0x00	
#define REG09_BQ25601D_BAT_FAULT_BATOVP					BIT(3)

#define REG09_BQ25601D_NTC_FAULT_MASK					(BIT(2) | BIT(1) | BIT(0))
#define REG09_BQ25601D_NTC_FAULT_NORMAL					0x00
#define REG09_BQ25601D_NTC_FAULT_WARM					BIT(1)
#define REG09_BQ25601D_NTC_FAULT_COOL					(BIT(1) | BIT(0))
#define REG09_BQ25601D_NTC_FAULT_COLD					(BIT(2) | BIT(0))
#define REG09_BQ25601D_NTC_FAULT_HOT						(BIT(2) | BIT(1))


/* Address:0Ah */
#define REG0A_BQ25601D_ADDRESS							0x0A

#define REG0A_BQ25601D_VINDPM_INT_MASK					BIT(1)
#define REG0A_BQ25601D_VINDPM_INT_ALLOW					0x00
#define REG0A_BQ25601D_VINDPM_INT_NOT_ALLOW			BIT(1)

#define REG0A_BQ25601D_IINDPM_INT_MASK					BIT(0)
#define REG0A_BQ25601D_IINDPM_INT_ALLOW					0x00
#define REG0A_BQ25601D_IINDPM_INT_NOT_ALLOW				BIT(0)


/* Address:0Bh */
#define REG0B_BQ25601D_ADDRESS							0x0B

#define REG0B_BQ25601D_REG_RST_MASK						BIT(7)
#define REG0B_BQ25601D_REG_RST_KEEP						0x00
#define REG0B_BQ25601D_REG_RST_RESET						BIT(7)

/* Other */
#define BQ25601D_FIRST_REG									0x00
#define BQ25601D_LAST_REG									0x0B
#define BQ25601D_REG_NUMBER								12

enum {
	OVERTIME_AC = 0,
	OVERTIME_USB,
	OVERTIME_DISABLED,
};

struct chip_bq25601d {
	struct i2c_client	*client;
	struct device		*dev;
	int				hw_aicl_point;
	int				sw_aicl_point;
	atomic_t			charger_suspended;
	int				irq_gpio;
};

int bq25601d_otg_enable(void);
int bq25601d_otg_disable(void);

extern volatile bool chargin_hw_init_done_bq25601d;

#ifdef CONFIG_OPLUS_CHARGER_MTK
//extern CHARGER_TYPE mt_charger_type_detection(void);
extern int mt_power_supply_type_check(void);
extern bool pmic_chrdet_status(void);
extern int battery_meter_get_charger_voltage(void);
extern int charger_pretype_get(void);

extern int get_rtc_spare_fg_value(void);
extern int set_rtc_spare_fg_value(int val);
extern int get_rtc_spare_oplus_fg_value(void);
extern int set_rtc_spare_oplus_fg_value(int val);

extern void mt_usb_connect(void);
extern void mt_usb_disconnect(void);

extern int mt_get_chargerid_volt (void);

//#ifdef CONFIG_MTK_HAFG_20
extern void mt_set_chargerid_switch_val(int value);
extern int mt_get_chargerid_switch_val(void);
extern int oplus_usb_switch_gpio_gpio_init(void);
//#endif /* CONFIG_OPLUS_CHARGER_MTK */

#if defined(CONFIG_OPLUS_CHARGER_MTK6763) || defined(CONFIG_OPLUS_CHARGER_MTK6771)
extern int oplus_battery_meter_get_battery_voltage(void);
extern bool meter_fg_30_get_battery_authenticate(void);
#endif /* CONFIG_OPLUS_CHARGER_MTK6763 */

#else /* CONFIG_OPLUS_CHARGER_MTK */
extern int qpnp_charger_type_get(void);
extern bool qpnp_lbc_is_usb_chg_plugged_in(void);
extern int qpnp_get_prop_charger_voltage_now(void);
extern int qpnp_get_prop_battery_voltage_now(void);
extern int qpnp_set_pmic_soc_memory(int soc);
extern int qpnp_get_pmic_soc_memory(void);
#endif /* CONFIG_OPLUS_CHARGER_MTK */

bool oplus_pmic_check_chip_is_null(void);

#endif /* __OPLUS_BQ25601D_H__ */
