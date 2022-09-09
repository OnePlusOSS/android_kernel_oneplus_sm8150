#ifndef _OPLUS_BATTERY_QCOM_MSM8976_H_
#define _OPLUS_BATTERY_QCOM_MSM8976_H_

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

struct qcom_pmic {
	struct spmi_device		*spmi;
	int 			schg_version;
	
	/* peripheral register address bases */
	u16 			chgr_base;
	u16 			bat_if_base;
	u16 			usb_chgpth_base;
	u16 			dc_chgpth_base;
	u16 			otg_base;
	u16 			misc_base;

	int 			fake_battery_soc;
	u8				revision[4];

	/* configuration parameters */
	int 			iterm_ma;
	int 			usb_max_current_ma;
	int 			dc_max_current_ma;
	int 			usb_target_current_ma;
	int 			usb_tl_current_ma;
	int 			dc_target_current_ma;
	int 			target_fastchg_current_ma;
	int 			cfg_fastchg_current_ma;
	int 			fastchg_current_ma;
	int 			vfloat_mv;
	int 			fastchg_current_comp;
	int 			float_voltage_comp;
	int 			resume_delta_mv;
	int 			safety_time;
	int 			prechg_safety_time;
	int 			bmd_pin_src;
	int 			jeita_temp_hard_limit;
	bool			use_vfloat_adjustments;
	bool			iterm_disabled;
	bool			bmd_algo_disabled;
	bool			soft_vfloat_comp_disabled;
	bool			chg_enabled;
	bool			charge_unknown_battery;
	bool			chg_inhibit_en;
	bool			chg_inhibit_source_fg;
	bool			low_volt_dcin;
	bool			cfg_chg_led_support;
	bool			cfg_chg_led_sw_ctrl;
	bool			vbat_above_headroom;
	bool			force_aicl_rerun;
	bool			hvdcp3_supported;
	bool			skip_fg_control_chg;
	u8				original_usbin_allowance;
	struct parallel_usb_cfg 	parallel;
		
	struct dentry			*debug_root;

	/* wipower params */
	struct ilim_map 		wipower_default;
	struct ilim_map 		wipower_pt;
	struct ilim_map 		wipower_div2;
	struct qpnp_vadc_chip		*vadc_dev;
	struct qpnp_vadc_chip		*pm8950_vadc_dev;
	bool				wipower_dyn_icl_avail;
	struct ilim_entry		current_ilim;
	struct mutex			wipower_config;
	bool				wipower_configured;
	struct qpnp_adc_tm_btm_param	param;

	/* flash current prediction */
	int 			rpara_uohm;
	int 			rslow_uohm;
	int 			vled_max_uv;

	/* vfloat adjustment */
	int 			max_vbat_sample;
	int 			n_vbat_samples;

	/* status variables */
	int 			battchg_disabled;
	int 			usb_suspended;
	int 			dc_suspended;
	int 			wake_reasons;
	int 			previous_soc;
	int 			usb_online;
	bool				dc_present;
	bool				usb_present;
	bool				batt_present;
	int 			otg_retries;
	ktime_t 			otg_enable_time;
	bool				aicl_deglitch_short;
	bool				sw_esr_pulse_en;
	bool				safety_timer_en;
	bool				aicl_complete;
	bool				usb_ov_det;
	bool				otg_pulse_skip_dis;
	const char			*battery_type;
	bool				very_weak_charger;
	bool				parallel_charger_detected;
	u32 			wa_flags;
	/* jeita and temperature */
	bool				batt_hot;
	bool				batt_cold;
	bool				batt_warm;
	bool				batt_cool;
	unsigned int			thermal_levels;
	unsigned int			therm_lvl_sel;
	unsigned int			*thermal_mitigation;

	/* irqs */
	int 			batt_hot_irq;
	int 			batt_warm_irq;
	int 			batt_cool_irq;
	int 			batt_cold_irq;
	int 			batt_missing_irq;
	int 			vbat_low_irq;
	int 			chg_hot_irq;
	int 			chg_term_irq;
	int 			taper_irq;
	bool				taper_irq_enabled;
	struct mutex			taper_irq_lock;
	int 			recharge_irq;
	int 			fastchg_irq;
	int 			safety_timeout_irq;
	int 			power_ok_irq;
	int 			dcin_uv_irq;
	int 			usbin_uv_irq;
	int 			usbin_ov_irq;
	int 			src_detect_irq;
	int 			otg_fail_irq;
	int 			otg_oc_irq;
	int 			aicl_done_irq;
	int 			usbid_change_irq;
	int 			chg_error_irq;
	bool				enable_aicl_wake;

	/* psy */
	struct power_supply 	dc_psy;
	struct power_supply 	*bms_psy;
	int 			dc_psy_type;
	const char			*bms_psy_name;
	const char			*battery_psy_name;
	bool				psy_registered;

	bool			other_sdp;
	int				pmic_type;
	struct smbchg_regulator 	otg_vreg;
	struct smbchg_regulator 	ext_otg_vreg;
	struct work_struct		usb_set_online_work;
	struct delayed_work 	vfloat_adjust_work;
		struct delayed_work 	parallel_en_work;
	struct delayed_work 	hvdcp_det_work;
	struct delayed_work 	dump_reg_work;
	struct delayed_work 			update_opchg_thread_work;
	struct delayed_work 			opchg_delayed_wakeup_work;
//	  struct wakeup_source			  source;
	spinlock_t			sec_access_lock;
	struct mutex			current_change_lock;
	struct mutex			usb_set_online_lock;
	struct mutex			battchg_disabled_lock;
	struct mutex			usb_en_lock;
	struct mutex			dc_en_lock;
	struct mutex			fcc_lock;
	struct mutex			pm_lock;
	/* aicl deglitch workaround */
	unsigned long			first_aicl_seconds;
	int 			aicl_irq_count;
	struct mutex			usb_status_lock;
	bool				hvdcp_3_det_ignore_uv;
	struct completion		src_det_lowered;
	struct completion		src_det_raised;
	struct completion		usbin_uv_lowered;
	struct completion		usbin_uv_raised;
	int 			pulse_cnt;
	struct led_classdev 	led_cdev;
	u8		 bat_status;
	bool	 suspending;
	int 	bat_instant_vol;
	int 	charging_current;
	int 	input_limit_flags;
	int 	input_current_max;
	int 	voltage_max;
	int 	charging_enable;
	int 	capacity;
	char	*charger_name;
	int 	bat_charging_state;
		bool		aicl_suspend;

		bool		usb_hc_mode;
		int    		usb_hc_count;
		bool		hc_mode_flag;
		bool		not_support_1200ma;

};

#endif

