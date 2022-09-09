/************************************************************************************
** OPLUS_FEATURE_CHG_BASIC
** Copyright (C), 2018-2019, OPLUS Mobile Comm Corp., Ltd
**
** Description:
**    For P80 charger ic driver
**
** Version: 1.0
** Date created: 2018-11-09
**
** --------------------------- Revision History: ------------------------------------
* <version>       <date>         <author>              			<desc>
*************************************************************************************/
#ifndef __OPLUS_BATTERY_MTK6779_H__
#define __OPLUS_BATTERY_MTK6779_H__

#include <linux/device.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/wait.h>
#include <mt-plat/charger_type.h>
#include <mt-plat/mtk_charger.h>
#include <mt-plat/mtk_battery.h>

#include "../../../../kernel-4.9/drivers/power/supply/mediatek/misc/mtk_gauge_time_service.h"

#include "../../../../kernel-4.9/drivers/power/supply/mediatek/charger/charger_class.h"

/* PD */
#include "../../../../kernel-4.9/drivers/misc/mediatek/typec/tcpc/inc/tcpm.h"

#include "../../../../kernel-4.9/drivers/misc/mediatek/typec/tcpc/inc/mtk_direct_charge_vdm.h"

struct charger_manager;
#include "../../../../kernel-4.9/drivers/power/supply/mediatek/charger/mtk_pe_intf.h"
#include "../../../../kernel-4.9/drivers/power/supply/mediatek/charger/mtk_pe20_intf.h"
#include "../../../../kernel-4.9/drivers/power/supply/mediatek/charger/mtk_pdc_intf.h"

//====================================================================//
/* mtk_pe40_intf begin */
#define CONFIG_MTK_PUMP_EXPRESS_PLUS_40_SUPPORT

#define VBUS_IDX_MAX 5

struct pe40_power_cap {
	uint8_t selected_cap_idx;
	uint8_t nr;
	uint8_t pwr_limit[PDO_MAX_NR];
	int max_mv[PDO_MAX_NR];
	int min_mv[PDO_MAX_NR];
	int ma[PDO_MAX_NR];
	int maxwatt[PDO_MAX_NR];
	int minwatt[PDO_MAX_NR];
};

struct pe4_pps_status {
	int output_mv;	/* 0xffff means no support */
	int output_ma;	/* 0xff means no support */
	uint8_t real_time_flags;
};

struct mtk_pe40 {
	bool is_connect;
	bool is_enabled;
	bool can_query;
	struct pe40_power_cap cap;

	int avbus;
	int vbus;
	int ibus;
	int watt;

	int r_sw;
	int r_cable;
	int r_cable_1;
	int r_cable_2;

	int pmic_vbus;
	int TA_vbus;
	int vbus_cali;

	int max_charger_ibus;
	int max_vbus;
	int max_ibus;

	int pe4_input_current_limit;
	int pe4_input_current_limit_setting;
	int input_current_limit;

};

#ifdef CONFIG_MTK_PUMP_EXPRESS_PLUS_40_SUPPORT
extern bool mtk_pe40_init(struct charger_manager *pinfo);
extern bool mtk_is_TA_support_pd_pps(struct charger_manager *info);
extern bool mtk_pe40_is_ready(struct charger_manager *pinfo);
extern bool mtk_pe40_get_is_connect(struct charger_manager *pinfo);
extern void mtk_pe40_set_is_enable(struct charger_manager *pinfo, bool enable);
extern bool mtk_pe40_get_is_enable(struct charger_manager *pinfo);
extern int mtk_pe40_init_state(struct charger_manager *pinfo);
extern int mtk_pe40_tune1_state(struct charger_manager *pinfo);
extern int mtk_pe40_tune2_state(struct charger_manager *pinfo);
extern int mtk_pe40_cc_state(struct charger_manager *pinfo);
extern void mtk_pe40_plugout_reset(struct charger_manager *pinfo);
extern void mtk_pe40_end(struct charger_manager *pinfo, int type, bool retry);
#else

#endif
/* mtk_pe40_intf end */
//====================================================================//


//====================================================================//
/* mtk_charger_init.h begin */
#define BATTERY_CV 4350000
#define V_CHARGER_MAX 6500000 /* 6.5 V */
#define V_CHARGER_MIN 4600000 /* 4.6 V */

#define USB_CHARGER_CURRENT_SUSPEND		0 /* def CONFIG_USB_IF */
#define USB_CHARGER_CURRENT_UNCONFIGURED	70000 /* 70mA */
#define USB_CHARGER_CURRENT_CONFIGURED		500000 /* 500mA */
#define USB_CHARGER_CURRENT			500000 /* 500mA */
#define AC_CHARGER_CURRENT			2050000
#define AC_CHARGER_INPUT_CURRENT		3200000
#define NON_STD_AC_CHARGER_CURRENT		500000
#define CHARGING_HOST_CHARGER_CURRENT		650000
#define APPLE_1_0A_CHARGER_CURRENT		650000
#define APPLE_2_1A_CHARGER_CURRENT		800000
#define TA_AC_CHARGING_CURRENT	3000000

/* sw jeita */
#define JEITA_TEMP_ABOVE_T4_CV	4240000
#define JEITA_TEMP_T3_TO_T4_CV	4240000
#define JEITA_TEMP_T2_TO_T3_CV	4340000
#define JEITA_TEMP_T1_TO_T2_CV	4240000
#define JEITA_TEMP_T0_TO_T1_CV	4040000
#define JEITA_TEMP_BELOW_T0_CV	4040000
#define TEMP_T4_THRES  50
#define TEMP_T4_THRES_MINUS_X_DEGREE 47
#define TEMP_T3_THRES  45
#define TEMP_T3_THRES_MINUS_X_DEGREE 39
#define TEMP_T2_THRES  10
#define TEMP_T2_THRES_PLUS_X_DEGREE 16
#define TEMP_T1_THRES  0
#define TEMP_T1_THRES_PLUS_X_DEGREE 6
#define TEMP_T0_THRES  0
#define TEMP_T0_THRES_PLUS_X_DEGREE  0
#define TEMP_NEG_10_THRES 0

/* Battery Temperature Protection */
#define MIN_CHARGE_TEMP  0
#define MIN_CHARGE_TEMP_PLUS_X_DEGREE	6
#define MAX_CHARGE_TEMP  50
#define MAX_CHARGE_TEMP_MINUS_X_DEGREE	47

/* pe */
#define PE_ICHG_LEAVE_THRESHOLD 1000000 /* uA */
#define TA_AC_12V_INPUT_CURRENT 3200000
#define TA_AC_9V_INPUT_CURRENT	3200000
#define TA_AC_7V_INPUT_CURRENT	3200000
#define TA_9V_SUPPORT
#define TA_12V_SUPPORT

/* pe2.0 */
#define PE20_ICHG_LEAVE_THRESHOLD 1000000 /* uA */
#define TA_START_BATTERY_SOC	0
#define TA_STOP_BATTERY_SOC	85

/* dual charger */
#define TA_AC_MASTER_CHARGING_CURRENT 1500000
#define TA_AC_SLAVE_CHARGING_CURRENT 1500000


/* cable measurement impedance */
#define CABLE_IMP_THRESHOLD 699
#define VBAT_CABLE_IMP_THRESHOLD 3900000 /* uV */

/* bif */
#define BIF_THRESHOLD1 4250000	/* UV */
#define BIF_THRESHOLD2 4300000	/* UV */
#define BIF_CV_UNDER_THRESHOLD2 4450000	/* UV */
#define BIF_CV BATTERY_CV /* UV */

#define R_SENSE 56 /* mohm */

#define MAX_CHARGING_TIME (12 * 60 * 60) /* 12 hours */

/* battery warning */
#define BATTERY_NOTIFY_CASE_0001_VCHARGER
#define BATTERY_NOTIFY_CASE_0002_VBATTEMP
/* mtk_charger_init.h end */
//====================================================================//


//====================================================================//
/* mtk_charger_intf.h begin */
#define CHARGING_INTERVAL 10
#define CHARGING_FULL_INTERVAL 20

#define CHRLOG_ERROR_LEVEL   1
#define CHRLOG_DEBUG_LEVEL   2

extern int chr_get_debug_level(void);

#define chr_err(fmt, args...)					\
do {								\
	if (chr_get_debug_level() >= CHRLOG_ERROR_LEVEL) {	\
		pr_notice(fmt, ##args);				\
	}							\
} while (0)

#define chr_info(fmt, args...)					\
do {								\
	if (chr_get_debug_level() >= CHRLOG_ERROR_LEVEL) {	\
		pr_notice_ratelimited(fmt, ##args);		\
	}							\
} while (0)

#define chr_debug(fmt, args...)					\
do {								\
	if (chr_get_debug_level() >= CHRLOG_DEBUG_LEVEL) {	\
		pr_notice(fmt, ##args);				\
	}							\
} while (0)

#define CHR_CC		(0x0001)
#define CHR_TOPOFF	(0x0002)
#define CHR_TUNING	(0x0003)
#define CHR_POSTCC	(0x0004)
#define CHR_BATFULL	(0x0005)
#define CHR_ERROR	(0x0006)
#define	CHR_PE40_INIT	(0x0007)
#define	CHR_PE40_CC	(0x0008)
#define	CHR_PE40_TUNING	(0x0009)
#define	CHR_PE40_POSTCC	(0x000A)
#define CHR_PE30	(0x000B)

/* charging abnormal status */
#define CHG_VBUS_OV_STATUS	(1 << 0)
#define CHG_BAT_OT_STATUS	(1 << 1)
#define CHG_OC_STATUS		(1 << 2)
#define CHG_BAT_OV_STATUS	(1 << 3)
#define CHG_ST_TMO_STATUS	(1 << 4)
#define CHG_BAT_LT_STATUS	(1 << 5)
#define CHG_TYPEC_WD_STATUS	(1 << 6)

/* charger_algorithm notify charger_dev */
enum {
	EVENT_EOC,
	EVENT_RECHARGE,
};

/* charger_dev notify charger_manager */
enum {
	CHARGER_DEV_NOTIFY_VBUS_OVP,
	CHARGER_DEV_NOTIFY_BAT_OVP,
	CHARGER_DEV_NOTIFY_EOC,
	CHARGER_DEV_NOTIFY_RECHG,
	CHARGER_DEV_NOTIFY_SAFETY_TIMEOUT,
};

/*
 * Software JEITA
 * T0: -10 degree Celsius
 * T1: 0 degree Celsius
 * T2: 10 degree Celsius
 * T3: 45 degree Celsius
 * T4: 50 degree Celsius
 */
enum sw_jeita_state_enum {
	TEMP_BELOW_T0 = 0,
	TEMP_T0_TO_T1,
	TEMP_T1_TO_T2,
	TEMP_T2_TO_T3,
	TEMP_T3_TO_T4,
	TEMP_ABOVE_T4
};

struct sw_jeita_data {
	int sm;
	int pre_sm;
	int cv;
	bool charging;
	bool error_recovery_flag;
};

/* battery thermal protection */
enum bat_temp_state_enum {
	BAT_TEMP_LOW = 0,
	BAT_TEMP_NORMAL,
	BAT_TEMP_HIGH
};

struct battery_thermal_protection_data {
	int sm;
	bool enable_min_charge_temp;
	int min_charge_temp;
	int min_charge_temp_plus_x_degree;
	int max_charge_temp;
	int max_charge_temp_minus_x_degree;
};

struct charger_custom_data {
	int battery_cv;	/* uv */
	int max_charger_voltage;
	int max_charger_voltage_setting;
	int min_charger_voltage;

	int usb_charger_current_suspend;
	int usb_charger_current_unconfigured;
	int usb_charger_current_configured;
	int usb_charger_current;
	int ac_charger_current;
	int ac_charger_input_current;
	int non_std_ac_charger_current;
	int charging_host_charger_current;
	int apple_1_0a_charger_current;
	int apple_2_1a_charger_current;
	int ta_ac_charger_current;
	int pd_charger_current;

	/* sw jeita */
	int jeita_temp_above_t4_cv;
	int jeita_temp_t3_to_t4_cv;
	int jeita_temp_t2_to_t3_cv;
	int jeita_temp_t1_to_t2_cv;
	int jeita_temp_t0_to_t1_cv;
	int jeita_temp_below_t0_cv;
	int temp_t4_thres;
	int temp_t4_thres_minus_x_degree;
	int temp_t3_thres;
	int temp_t3_thres_minus_x_degree;
	int temp_t2_thres;
	int temp_t2_thres_plus_x_degree;
	int temp_t1_thres;
	int temp_t1_thres_plus_x_degree;
	int temp_t0_thres;
	int temp_t0_thres_plus_x_degree;
	int temp_neg_10_thres;

	/* battery temperature protection */
	int mtk_temperature_recharge_support;
	int max_charge_temp;
	int max_charge_temp_minus_x_degree;
	int min_charge_temp;
	int min_charge_temp_plus_x_degree;

	/* pe */
	int pe_ichg_level_threshold;	/* ma */
	int ta_ac_12v_input_current;
	int ta_ac_9v_input_current;
	int ta_ac_7v_input_current;
	bool ta_12v_support;
	bool ta_9v_support;

	/* pe2.0 */
	int pe20_ichg_level_threshold;	/* ma */
	int ta_start_battery_soc;
	int ta_stop_battery_soc;

	/* pe4.0 */
	int pe40_single_charger_input_current;	/* ma */
	int pe40_single_charger_current;
	int pe40_dual_charger_input_current;
	int pe40_dual_charger_chg1_current;
	int pe40_dual_charger_chg2_current;
	int pe40_stop_battery_soc;

	/* pe4.0 cable impedance threshold (mohm) */
	u32 pe40_r_cable_1a_lower;
	u32 pe40_r_cable_2a_lower;
	u32 pe40_r_cable_3a_lower;

	/* dual charger */
	u32 chg1_ta_ac_charger_current;
	u32 chg2_ta_ac_charger_current;

	/* cable measurement impedance */
	int cable_imp_threshold;
	int vbat_cable_imp_threshold;

	/* bif */
	int bif_threshold1;	/* uv */
	int bif_threshold2;	/* uv */
	int bif_cv_under_threshold2;	/* uv */

	/* power path */
	bool power_path_support;

	int max_charging_time; /* second */
};

struct charger_data {
	int force_charging_current;
	int thermal_input_current_limit;
	int thermal_charging_current_limit;
	int input_current_limit;
	int charging_current_limit;
	int disable_charging_count;
	int input_current_limit_by_aicl;
	int junction_temp_min;
	int junction_temp_max;
};

struct charger_manager {
	bool init_done;
	const char *algorithm_name;
	struct platform_device *pdev;
	void	*algorithm_data;
	int usb_state;
	bool usb_unlimited;
	bool disable_charger;

	struct charger_device *chg1_dev;
	struct notifier_block chg1_nb;
	struct charger_data chg1_data;
	struct charger_consumer *chg1_consumer;

	struct charger_device *chg2_dev;
	struct notifier_block chg2_nb;
	struct charger_data chg2_data;

	enum charger_type chr_type;
	bool can_charging;

	int (*do_algorithm)(struct charger_manager *);
	int (*plug_in)(struct charger_manager *);
	int (*plug_out)(struct charger_manager *);
	int (*do_charging)(struct charger_manager *, bool en);
	int (*do_event)(struct notifier_block *nb, unsigned long ev, void *v);
	int (*change_current_setting)(struct charger_manager *);

	/* notify charger user */
	struct srcu_notifier_head evt_nh;
	/* receive from battery */
	struct notifier_block psy_nb;

	/* common info */
	int battery_temp;

	/* sw jeita */
	bool enable_sw_jeita;
	struct sw_jeita_data sw_jeita;

	/* dynamic_cv */
	bool enable_dynamic_cv;

	bool cmd_discharging;
	bool safety_timeout;
	bool vbusov_stat;

	/* battery warning */
	unsigned int notify_code;
	unsigned int notify_test_mode;

	/* battery thermal protection */
	struct battery_thermal_protection_data thermal;

	/* dtsi custom data */
	struct charger_custom_data data;

	bool enable_sw_safety_timer;
	bool sw_safety_timer_setting;

	/* High voltage charging */
	bool enable_hv_charging;

	/* pe */
	bool enable_pe_plus;
	struct mtk_pe pe;

	/* pe 2.0 */
	bool enable_pe_2;
	struct mtk_pe20 pe2;

	/* pe 4.0 */
	bool enable_pe_4;
	struct mtk_pe40 pe4;

	/* type-C*/
	bool enable_type_c;

	/* water detection */
	bool water_detected;

	/* pd */
	struct mtk_pdc pdc;

	int pd_type;
	struct tcpc_device *tcpc;
	struct notifier_block pd_nb;
	bool pd_reset;

	/* thread related */
	struct hrtimer charger_kthread_timer;
	struct gtimer charger_kthread_fgtimer;

	struct wakeup_source charger_wakelock;
	struct mutex charger_lock;
	struct mutex charger_pd_lock;
	spinlock_t slock;
	unsigned int polling_interval;
	bool charger_thread_timeout;
	wait_queue_head_t  wait_que;
	bool charger_thread_polling;

	/* kpoc */
	atomic_t enable_kpoc_shdn;

	/* ATM */
	bool atm_enabled;
};

#ifdef OPLUS_FEATURE_CHG_BASIC

struct mtk_pmic {
	struct charger_manager* oplus_info;
};

//extern int mt_power_supply_type_check(void);
extern enum charger_type mt_get_charger_type(void);
//int battery_meter_get_charger_voltage(void);
extern int mt6360_get_vbus_rising(void);
extern int mt6360_check_charging_enable(void);
extern int mt6360_suspend_charger(bool suspend);
extern int mt6360_set_rechg_voltage(int rechg_mv);
extern int mt6360_reset_charger(void);
extern int mt6360_set_chging_term_disable(bool disable);
extern int mt6360_aicl_enable(bool enable);
extern int mt6360_set_register(u8 addr, u8 mask, u8 data);
//extern int oplus_battery_meter_get_battery_voltage(void);
//extern int charger_pretype_get(void);

//extern int get_rtc_spare_fg_value(void);
//extern int set_rtc_spare_fg_value(int val);
//extern int get_rtc_spare_oplus_fg_value(void);
//extern int set_rtc_spare_oplus_fg_value(int val);

//extern int mt_get_chargerid_volt(void);
//extern void mt_set_chargerid_switch_val(int value);
//extern int mt_get_chargerid_switch_val(void);


extern void mt_power_off(void);
extern void mt_usb_connect(void);
extern void mt_usb_disconnect(void);
//#ifdef CONFIG_MTK_HAFG_20

//extern int oplus_usb_switch_gpio_gpio_init(void);
//#endif /* CONFIG_OPLUS_CHARGER_MTK */

//#if defined(CONFIG_OPLUS_CHARGER_MTK6763) || defined(CONFIG_OPLUS_CHARGER_MTK6771)
//extern int oplus_battery_meter_get_battery_voltage(void);
//extern bool meter_fg_30_get_battery_authenticate(void);
//#endif /* CONFIG_OPLUS_CHARGER_MTK6763 */

//#else /* CONFIG_OPLUS_CHARGER_MTK */
//extern int qpnp_charger_type_get(void);
//extern bool qpnp_lbc_is_usb_chg_plugged_in(void);
//extern int qpnp_get_prop_charger_voltage_now(void);
//extern int qpnp_get_prop_battery_voltage_now(void);
//extern int qpnp_set_pmic_soc_memory(int soc);
//extern int qpnp_get_pmic_soc_memory(void);
//#endif /* CONFIG_OPLUS_CHARGER_MTK */

bool oplus_pmic_check_chip_is_null(void);
extern int oplus_get_typec_sbu_voltage(void);
extern void oplus_set_water_detect(bool enable);
extern int oplus_get_water_detect(void);
#endif /* OPLUS_FEATURE_CHG_BASIC */

/* charger related module interface */
extern int charger_manager_notifier(struct charger_manager *info, int event);
extern int mtk_switch_charging_init(struct charger_manager *info);
extern int mtk_dual_switch_charging_init(struct charger_manager *info);
extern int mtk_linear_charging_init(struct charger_manager *info);
extern void _wake_up_charger(struct charger_manager *info);
extern int mtk_get_dynamic_cv(struct charger_manager *info, unsigned int *cv);
extern bool is_dual_charger_supported(struct charger_manager *info);
extern int charger_enable_vbus_ovp(struct charger_manager *pinfo, bool enable);
extern bool is_typec_adapter(struct charger_manager *info);

/* pmic API */
extern unsigned int upmu_get_rgs_chrdet(void);
extern int pmic_get_vbus(void);
extern int pmic_get_charging_current(void);
extern int pmic_get_battery_voltage(void);
extern int pmic_get_bif_battery_voltage(int *vbat);
extern int pmic_is_bif_exist(void);
extern int pmic_enable_hw_vbus_ovp(bool enable);
extern bool pmic_is_battery_exist(void);

/* FIXME */
enum usb_state_enum {
	USB_SUSPEND = 0,
	USB_UNCONFIGURED,
	USB_CONFIGURED
};

bool __attribute__((weak)) is_usb_rdy(void)
{
	return true;
}

/* procfs */
#define PROC_FOPS_RW(name)						\
static int mtk_chg_##name##_open(struct inode *node, struct file *file)	\
{									\
	return single_open(file, mtk_chg_##name##_show, PDE_DATA(node));\
}									\
static const struct file_operations mtk_chg_##name##_fops = {		\
	.owner = THIS_MODULE,						\
	.open = mtk_chg_##name##_open,					\
	.read = seq_read,						\
	.llseek = seq_lseek,						\
	.release = single_release,					\
	.write = mtk_chg_##name##_write,				\
}
/* mtk_charger_intf.h end */
//====================================================================//


#endif /* __OPLUS_BATTERY_MTK6779_H__ */
