#ifndef _OPLUS_BATTERY_QCOM_MSM8953_N_H_
#define _OPLUS_BATTERY_QCOM_MSM8953_N_H_

#include <linux/power_supply.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/qpnp/qpnp-adc.h>

struct smbchg_regulator {
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
};

enum skip_reason {
	REASON_OTG_ENABLED	= BIT(0),
	REASON_FLASH_ENABLED	= BIT(1)
};

struct ilim_entry {
	int vmin_uv;
	int vmax_uv;
	int icl_pt_ma;
	int icl_lv_ma;
	int icl_hv_ma;
};

struct ilim_map {
	int			num;
	struct ilim_entry	*entries;
};

struct parallel_usb_cfg {
	struct power_supply		*psy;
	int				min_current_thr_ma;
	int				min_9v_current_thr_ma;
	int				allowed_lowering_ma;
	int				current_max_ma;
	bool				avail;
	struct mutex			lock;
	int				initial_aicl_ma;
	ktime_t				last_disabled;
	bool				enabled_once;
};

struct smbchg_version_tables {
	const int			*dc_ilim_ma_table;
	int				dc_ilim_ma_len;
	const int			*usb_ilim_ma_table;
	int				usb_ilim_ma_len;
	const int			*iterm_ma_table;
	int				iterm_ma_len;
	const int			*fcc_comp_table;
	int				fcc_comp_len;
	const int			*aicl_rerun_period_table;
	int				aicl_rerun_period_len;
	int				rchg_thr_mv;
};

struct qcom_pmic {
	struct spmi_device		*spmi;
	int				schg_version;

	/* peripheral register address bases */
	u16				chgr_base;
	u16				bat_if_base;
	u16				usb_chgpth_base;
	u16				dc_chgpth_base;
	u16				otg_base;
	u16				misc_base;

	int				fake_battery_soc;
	u8				revision[4];

	/* configuration parameters */
	int				iterm_ma;
	int				usb_max_current_ma;
	int				typec_current_ma;
	int				dc_max_current_ma;
	int				dc_target_current_ma;
	int				cfg_fastchg_current_ma;
	int				fastchg_current_ma;
	int				vfloat_mv;
	int				fastchg_current_comp;
	int				float_voltage_comp;
	int				resume_delta_mv;
	int				safety_time;
	int				prechg_safety_time;
	int				bmd_pin_src;
	int				jeita_temp_hard_limit;
	int				aicl_rerun_period_s;
	bool				use_vfloat_adjustments;
	bool				iterm_disabled;
	bool				bmd_algo_disabled;
	bool				soft_vfloat_comp_disabled;
	bool				chg_enabled;
	bool				charge_unknown_battery;
	bool				chg_inhibit_en;
	bool				chg_inhibit_source_fg;
	bool				low_volt_dcin;
	bool				cfg_chg_led_support;
	bool				cfg_chg_led_sw_ctrl;
	bool				vbat_above_headroom;
	bool				force_aicl_rerun;
	bool				hvdcp3_supported;
	bool				restricted_charging;
	bool				skip_usb_suspend_for_fake_battery;
	bool				skip_fg_control_chg;
	bool				hvdcp_not_supported;
	bool				otg_pinctrl;
	bool				cfg_override_usb_current;
	u8				original_usbin_allowance;
	struct parallel_usb_cfg		parallel;
	struct delayed_work		parallel_en_work;
	struct dentry			*debug_root;
	struct smbchg_version_tables	tables;

	/* wipower params */
	struct ilim_map			wipower_default;
	struct ilim_map			wipower_pt;
	struct ilim_map			wipower_div2;
	struct qpnp_vadc_chip		*vadc_dev;
	struct qpnp_vadc_chip		*pm8953_vadc_dev;
	bool				wipower_dyn_icl_avail;
	struct ilim_entry		current_ilim;
	struct mutex			wipower_config;
	bool				wipower_configured;
	struct qpnp_adc_tm_btm_param	param;

	/* flash current prediction */
	int				rpara_uohm;
	int				rslow_uohm;
	int				vled_max_uv;

	/* vfloat adjustment */
	int				max_vbat_sample;
	int				n_vbat_samples;

	/* status variables */
	int				wake_reasons;
	int				previous_soc;
	int				usb_online;
	bool				dc_present;
	bool				usb_present;
	bool				batt_present;
	int				otg_retries;
	ktime_t				otg_enable_time;
	bool				aicl_deglitch_short;
	bool				safety_timer_en;
	bool				aicl_complete;
	bool				usb_ov_det;
	bool				otg_pulse_skip_dis;
	const char			*battery_type;
	enum power_supply_type		usb_supply_type;
	bool				very_weak_charger;
	bool				parallel_charger_detected;
	bool				chg_otg_enabled;
	bool				flash_triggered;
	bool				flash_active;
	bool				icl_disabled;
	u32				wa_flags;
	int				usb_icl_delta;
	bool				typec_dfp;

	/* jeita and temperature */
	bool				batt_hot;
	bool				batt_cold;
	bool				batt_warm;
	bool				batt_cool;
	unsigned int			thermal_levels;
	unsigned int			therm_lvl_sel;
	unsigned int			*thermal_mitigation;

	/* irqs */
	int				batt_hot_irq;
	int				batt_warm_irq;
	int				batt_cool_irq;
	int				batt_cold_irq;
	int				batt_missing_irq;
	int				vbat_low_irq;
	int				chg_hot_irq;
	int				chg_term_irq;
	int				taper_irq;
	bool				taper_irq_enabled;
	struct mutex			taper_irq_lock;
	int				recharge_irq;
	int				fastchg_irq;
	int				wdog_timeout_irq;
	int				power_ok_irq;
	int				dcin_uv_irq;
	int				usbin_uv_irq;
	int				usbin_ov_irq;
	int				src_detect_irq;
	int				otg_fail_irq;
	int				otg_oc_irq;
	int				aicl_done_irq;
	int				usbid_change_irq;
	int				pr_error_irq;
	bool				enable_aicl_wake;

	/* psy */
	struct power_supply		dc_psy;
	struct power_supply		*bms_psy;
	struct power_supply		*typec_psy;
	int				dc_psy_type;
	const char			*bms_psy_name;
	const char			*battery_psy_name;
	bool				psy_registered;

	struct smbchg_regulator		otg_vreg;
	struct smbchg_regulator		ext_otg_vreg;
	struct work_struct		usb_set_online_work;
	struct delayed_work		vfloat_adjust_work;
	struct delayed_work		hvdcp_det_work;
	struct delayed_work 	dump_reg_work;
	struct delayed_work 			update_opchg_thread_work;
	struct delayed_work 			opchg_delayed_wakeup_work;
	spinlock_t			sec_access_lock;
	struct mutex			therm_lvl_lock;
	struct mutex			usb_set_online_lock;
	struct mutex			pm_lock;
	/* aicl deglitch workaround */
	unsigned long			first_aicl_seconds;
	int				aicl_irq_count;
	struct mutex			usb_status_lock;
	bool				hvdcp_3_det_ignore_uv;
	struct completion		src_det_lowered;
	struct completion		src_det_raised;
	struct completion		usbin_uv_lowered;
	struct completion		usbin_uv_raised;
	int				pulse_cnt;
	struct led_classdev		led_cdev;
	bool				skip_usb_notification;
	u32				vchg_adc_channel;
	struct qpnp_vadc_chip		*vchg_vadc_dev;

	/* voters */
	struct votable			*fcc_votable;
	struct votable			*usb_icl_votable;
	struct votable			*dc_icl_votable;
	struct votable			*usb_suspend_votable;
	struct votable			*dc_suspend_votable;
	struct votable			*battchg_suspend_votable;
	struct votable			*hw_aicl_rerun_disable_votable;
	struct votable			*hw_aicl_rerun_enable_indirect_votable;
	struct votable			*aicl_deglitch_short_votable;
	bool 					aicl_suspended;
	/* copy from msm8976_pmic begin */
	int 					bat_charging_state;
	bool	 				suspending;
	bool					aicl_suspend;
	bool					usb_hc_mode;
	int    					usb_hc_count;
	bool					hc_mode_flag;
	/* copy form msm8976_pmic end */
	int						primal_chgtype;
	bool					not_support_1200ma;
	/* wenbin.liu add for restore soc */
	bool					soc_restore_support;
	/* wenbin.liu add for project 17151 */
	bool					usbid_gpio_support;
};

#endif

