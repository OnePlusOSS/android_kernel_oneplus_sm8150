/* Copyright (c) 2012-2015, 2017-2018, The Linux Foundation.
 * All rights reserved.
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

#ifndef QPNP_PON_H
#define QPNP_PON_H

#include <dt-bindings/input/qcom,qpnp-power-on.h>
#include <linux/errno.h>

/**
 * enum pon_trigger_source: List of PON trigger sources
 * %PON_SMPL:		PON triggered by Sudden Momentary Power Loss (SMPL)
 * %PON_RTC:		PON triggered by real-time clock (RTC) alarm
 * %PON_DC_CHG:		PON triggered by insertion of DC charger
 * %PON_USB_CHG:	PON triggered by insertion of USB
 * %PON_PON1:		PON triggered by other PMIC (multi-PMIC option)
 * %PON_CBLPWR_N:	PON triggered by power-cable insertion
 * %PON_KPDPWR_N:	PON triggered by long press of the power-key
 */
enum pon_trigger_source {
	PON_SMPL = 1,
	PON_RTC,
	PON_DC_CHG,
	PON_USB_CHG,
	PON_PON1,
	PON_CBLPWR_N,
	PON_KPDPWR_N,
};

/**
 * enum pon_power_off_type: Possible power off actions to perform
 * %PON_POWER_OFF_RESERVED:          Reserved, not used
 * %PON_POWER_OFF_WARM_RESET:        Reset the MSM but not all PMIC peripherals
 * %PON_POWER_OFF_SHUTDOWN:          Shutdown the MSM and PMIC completely
 * %PON_POWER_OFF_HARD_RESET:        Reset the MSM and all PMIC peripherals
 * %PON_POWER_OFF_MAX_TYPE:          Reserved, not used
 */
enum pon_power_off_type {
	PON_POWER_OFF_RESERVED		= 0x00,
	PON_POWER_OFF_WARM_RESET	= PON_POWER_OFF_TYPE_WARM_RESET,
	PON_POWER_OFF_SHUTDOWN		= PON_POWER_OFF_TYPE_SHUTDOWN,
	PON_POWER_OFF_HARD_RESET	= PON_POWER_OFF_TYPE_HARD_RESET,
	PON_POWER_OFF_MAX_TYPE		= 0x10,
};

enum pon_restart_reason {
	PON_RESTART_REASON_UNKNOWN		= 0x00,
	PON_RESTART_REASON_RECOVERY		= 0x01,
	PON_RESTART_REASON_BOOTLOADER		= 0x02,
	PON_RESTART_REASON_RTC			= 0x03,
	PON_RESTART_REASON_DMVERITY_CORRUPTED	= 0x04,
	PON_RESTART_REASON_DMVERITY_ENFORCE	= 0x05,
	PON_RESTART_REASON_KEYS_CLEAR		= 0x06,
#ifdef VENDOR_EDIT
	PON_RESTART_REASON_SILENCE			= 0x21,
	PON_RESTART_REASON_SAU				= 0x22,
	PON_RESTART_REASON_RF				= 0x23,
	PON_RESTART_REASON_WLAN				= 0x24,
	PON_RESTART_REASON_MOS				= 0x25,
	PON_RESTART_REASON_FACTORY			= 0x26,
	PON_RESTART_REASON_KERNEL			= 0x27,
	PON_RESTART_REASON_MODEM			= 0x28,
	PON_RESTART_REASON_ANDROID			= 0x29,
	PON_RESTART_REASON_SAFE				= 0x2A,
	#ifdef OPLUS_FEATURE_AGINGTEST
	PON_RESTART_REASON_SBL_DDRTEST			= 0x2B,
	PON_RESTART_REASON_SBL_DDR_CUS			= 0x2C,
	PON_RESTART_REASON_MEM_AGING			= 0x2D,
	PON_RESTART_REASON_BOOT_NO_VIBRATION            = 0x2F,
	//0x2E is SBLTEST FAIL, just happen in ddrtest fail when xbl setup
	#endif
	PON_RESTART_REASON_NORMAL			= 0x3E,
#endif
};

#ifdef OPLUS_FEATURE_QCOM_PMICWD
struct qpnp_pon {
        struct device           *dev;
        struct regmap           *regmap;
        struct input_dev        *pon_input;
        struct qpnp_pon_config  *pon_cfg;
        struct pon_regulator    *pon_reg_cfg;
        struct list_head        restore_regs;
        struct list_head        list;
        struct mutex            restore_lock;
        struct delayed_work     bark_work;
        struct dentry           *debugfs;
        struct device_node      *pbs_dev_node;
        struct task_struct      *wd_task;
        struct mutex		wd_task_mutex;
        unsigned int		pmicwd_state;//|reserver|rst type|timeout|enable|
        u8                      suspend_state;//record the suspend state
        u16                     base;
        u8                      subtype;
        u8                      pon_ver;
        u8                      warm_reset_reason1;
        u8                      warm_reset_reason2;
        u8                      twm_state;
        int                     num_pon_config;
        int                     num_pon_reg;
        int                     pon_trigger_reason;
        int                     pon_power_off_reason;
        u32                     dbc_time_us;
        u32                     uvlo;
        int                     warm_reset_poff_type;
        int                     hard_reset_poff_type;
        int                     shutdown_poff_type;
        int                     resin_warm_reset_type;
        int                     resin_hard_reset_type;
        int                     resin_shutdown_type;
        bool                    is_spon;
        bool                    store_hard_reset_reason;
        bool                    resin_hard_reset_disable;
        bool                    resin_shutdown_disable;
        bool                    ps_hold_hard_reset_disable;
        bool                    ps_hold_shutdown_disable;
        bool                    kpdpwr_dbc_enable;
        bool                    support_twm_config;
        bool                    resin_pon_reset;
        ktime_t                 kpdpwr_last_release_time;
        struct notifier_block	pon_nb;
        bool			legacy_hard_reset_offset;
};

extern const struct dev_pm_ops qpnp_pm_ops;
extern struct qpnp_pon *sys_reset_dev;
int qpnp_pon_masked_write(struct qpnp_pon *pon, u16 addr, u8 mask, u8 val);
void pmicwd_init(struct platform_device *pdev, struct qpnp_pon *pon, bool sys_reset);
void kpdpwr_init(struct qpnp_pon *pon,  bool sys_reset);
#endif /* OPLUS_FEATURE_QCOM_PMICWD */

#ifdef CONFIG_INPUT_QPNP_POWER_ON
int qpnp_pon_system_pwr_off(enum pon_power_off_type type);
int qpnp_pon_is_warm_reset(void);
int qpnp_pon_trigger_config(enum pon_trigger_source pon_src, bool enable);
int qpnp_pon_wd_config(bool enable);
int qpnp_pon_set_restart_reason(enum pon_restart_reason reason);
bool qpnp_pon_check_hard_reset_stored(void);

#else
static int qpnp_pon_system_pwr_off(enum pon_power_off_type type)
{
	return -ENODEV;
}
static inline int qpnp_pon_is_warm_reset(void)
{
	return -ENODEV;
}
static inline int qpnp_pon_trigger_config(enum pon_trigger_source pon_src,
							bool enable)
{
	return -ENODEV;
}
int qpnp_pon_wd_config(bool enable)
{
	return -ENODEV;
}
static inline int qpnp_pon_set_restart_reason(enum pon_restart_reason reason)
{
	return -ENODEV;
}
static inline bool qpnp_pon_check_hard_reset_stored(void)
{
	return false;
}
#endif

#endif
