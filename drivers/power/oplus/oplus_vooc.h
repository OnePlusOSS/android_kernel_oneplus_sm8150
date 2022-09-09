/**********************************************************************************
* Copyright (c)  2008-2015  Guangdong OPLUS Mobile Comm Corp., Ltd
* OPLUS_FEATURE_CHG_BASIC
* Description: Charger IC management module for charger system framework.
*             Manage all charger IC and define abstarct function flow.
* Version   : 1.0
* Date      : 2015-05-22
*           : Fanhong.Kong@ProDrv.CHG
* ------------------------------ Revision History: --------------------------------
* <version>           <date>                <author>                               <desc>
* Revision 1.0        2015-05-22        Fanhong.Kong@ProDrv.CHG            Created for new architecture
* Revision 2.0        2018-04-14        Fanhong.Kong@ProDrv.CHG            Upgrade for SVOOC
***********************************************************************************/

#ifndef _OPLUS_VOOC_H_
#define _OPLUS_VOOC_H_

#include <linux/workqueue.h>
#include <linux/version.h>
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
#include <linux/wakelock.h>
#endif
#include <linux/timer.h>
#include <linux/slab.h>
#include <soc/oplus/device_info.h>
#include <soc/oplus/system/oplus_project.h>
#include <linux/firmware.h>

#define OPLUS_VOOC_MCU_HWID_UNKNOW   -1
#define OPLUS_VOOC_MCU_HWID_STM8S	0
#define OPLUS_VOOC_MCU_HWID_N76E		1
#define OPLUS_VOOC_ASIC_HWID_RK826	2
#define OPLUS_VOOC_ASIC_HWID_OP10	3
#define OPLUS_VOOC_ASIC_HWID_RT5125   4
#define OPLUS_VOOC_ASIC_HWID_NON_EXIST 5

enum {
	VOOC_CHARGER_MODE,
	HEADPHONE_MODE,
	NORMAL_CHARGER_MODE,
};

enum {
	FW_ERROR_DATA_MODE,
	FW_NO_CHECK_MODE,
	FW_CHECK_MODE,
};

enum {
	VOOC_MAX_CURRENT_NO_LIMIT,
	VOOC_MAX_CURRENT_LIMIT_2A,
	VOOC_MAX_CURRENT_LIMIT_OTHER,
};
enum {
	FASTCHG_CHARGER_TYPE_UNKOWN,
	PORTABLE_PIKAQIU_1 = 0x31,
	PORTABLE_PIKAQIU_2 = 0x32,
	PORTABLE_50W = 0x33,
	PORTABLE_20W_1 = 0X34,
	PORTABLE_20W_2 = 0x35,
	PORTABLE_20W_3 = 0x36,
};

enum e_fastchg_power{
	FASTCHG_POWER_UNKOWN,
	FASTCHG_POWER_5V4A_5V6A_VOOC,
	FASTCHG_POWER_11V3A_FLASHCHARGER,
	FASTCHG_POWER_10V5A_SINGLE_BAT_SVOOC,
	FASTCHG_POWER_10V5A_TWO_BAT_SVOOC,
	FASTCHG_POWER_10V6P5A_TWO_BAT_SVOOC,
	FASTCHG_POWER_OTHER,
};

enum {
	BAT_TEMP_NATURAL = 0,
	BAT_TEMP_HIGH0,
	BAT_TEMP_HIGH1,
	BAT_TEMP_HIGH2,
	BAT_TEMP_HIGH3,
	BAT_TEMP_HIGH4,
	BAT_TEMP_HIGH5,
	BAT_TEMP_LOW0,
	BAT_TEMP_LOW1,
	BAT_TEMP_LOW2,
	BAT_TEMP_LITTLE_COOL,
	BAT_TEMP_COOL,
	BAT_TEMP_NORMAL_LOW,
	BAT_TEMP_NORMAL_HIGH,
	BAT_TEMP_LITTLE_COLD,
	BAT_TEMP_EXIT,
};

enum {
	FASTCHG_TEMP_RANGE_INIT = 0,
	FASTCHG_TEMP_RANGE_LITTLE_COLD,/*0 ~ 5*/
	FASTCHG_TEMP_RANGE_COOL,/*5 ~ 12*/
	FASTCHG_TEMP_RANGE_LITTLE_COOL, /*12 `16*/
	FASTCHG_TEMP_RANGE_NORMAL_LOW, /*16-25*/
	FASTCHG_TEMP_RANGE_NORMAL_HIGH, /*25-43*/
};


struct vooc_gpio_control {
	int switch1_gpio;
	int switch1_ctr1_gpio;
	int switch2_gpio;
	int switch3_gpio;
	int reset_gpio;
	int clock_gpio;
	int data_gpio;
	int vooc_mcu_id_gpio;
	int vooc_asic_id_gpio;
	int data_irq;
	struct pinctrl *pinctrl;

	struct pinctrl_state *gpio_switch1_act_switch2_act;
	struct pinctrl_state *gpio_switch1_sleep_switch2_sleep;
	struct pinctrl_state *gpio_switch1_act_switch2_sleep;
	struct pinctrl_state *gpio_switch1_sleep_switch2_act;
	struct pinctrl_state *gpio_switch1_ctr1_act;
	struct pinctrl_state *gpio_switch1_ctr1_sleep;

	struct pinctrl_state *gpio_clock_active;
	struct pinctrl_state *gpio_clock_sleep;
	struct pinctrl_state *gpio_data_active;
	struct pinctrl_state *gpio_data_sleep;
	struct pinctrl_state *gpio_reset_active;
	struct pinctrl_state *gpio_reset_sleep;
	struct pinctrl_state *gpio_vooc_mcu_id_default;
	struct pinctrl_state *gpio_vooc_asic_id_active;
	struct pinctrl_state *gpio_vooc_asic_id_sleep;
};

struct oplus_vooc_chip {
	struct i2c_client *client;
	struct device *dev;
	struct oplus_vooc_operations *vops;
	struct vooc_gpio_control vooc_gpio;
	struct delayed_work fw_update_work;
	struct delayed_work fastchg_work;
	struct delayed_work delay_reset_mcu_work;
	struct delayed_work check_charger_out_work;
	struct work_struct vooc_watchdog_work;
	struct timer_list watchdog;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
	struct wake_lock vooc_wake_lock;
#else
	struct wakeup_source *vooc_ws;
#endif

	struct power_supply *batt_psy;
	int pcb_version;
	bool allow_reading;
	bool fastchg_started;
	bool fastchg_ing;
	bool fastchg_allow;
	bool fastchg_to_normal;
	bool fastchg_to_warm;
	bool fastchg_low_temp_full;
	bool btb_temp_over;
	bool fastchg_dummy_started;
	bool need_to_up;
	bool have_updated;
	bool mcu_update_ing;
	bool mcu_update_finish;
	bool mcu_boot_by_gpio;
	const unsigned char *firmware_data;
	unsigned int fw_data_count;
	int fw_mcu_version;
	int fw_data_version;
	int adapter_update_real;
	int adapter_update_report;
	int dpdm_switch_mode;
	bool support_vooc_by_normal_charger_path;
/* Add for vooc batt 4.40*/
	bool batt_type_4400mv;
	bool vooc_fw_check;
	bool support_single_batt_svooc;
	int vooc_fw_type;
	int fw_update_flag;
	struct manufacture_info manufacture_info;
	bool vooc_fw_update_newmethod;
	char *fw_path;
	struct mutex pinctrl_mutex;
	int vooc_temp_cur_range;
	int vooc_little_cool_temp;
	int vooc_cool_temp;
	int vooc_little_cold_temp;
	int vooc_normal_low_temp;
	int vooc_little_cool_temp_default;
	int vooc_cool_temp_default;
	int vooc_little_cold_temp_default;
	int vooc_normal_low_temp_default;
	int vooc_low_temp;
	int vooc_high_temp;
	int vooc_low_soc;
	int vooc_high_soc;
	int vooc_cool_bat_volt;
	int vooc_little_cool_bat_volt;
	int vooc_normal_bat_volt;
	int vooc_warm_bat_volt;
	int vooc_cool_bat_suspend_volt;
	int vooc_little_cool_bat_suspend_volt;
	int vooc_normal_bat_suspend_volt;
	int vooc_warm_bat_suspend_volt;
	int vooc_chg_current_now;
	int fast_chg_type;
	int gaugue_fast_chg_type;
	bool disable_adapter_output;// 0--vooc adapter output normal,  1--disable vooc adapter output
	int set_vooc_current_limit;///0--no limit;  1--max current limit 2A
	bool vooc_multistep_adjust_current_support;
	int vooc_reply_mcu_bits;
	int vooc_multistep_initial_batt_temp;
	int vooc_strategy_normal_current;
	int vooc_strategy1_batt_high_temp0;
	int vooc_strategy1_batt_high_temp1;
	int vooc_strategy1_batt_high_temp2;
	int vooc_strategy1_batt_low_temp2;
	int vooc_strategy1_batt_low_temp1;
	int vooc_strategy1_batt_low_temp0;
	int vooc_strategy1_high_current0;
	int vooc_strategy1_high_current1;
	int vooc_strategy1_high_current2;
	int vooc_strategy1_low_current2;
	int vooc_strategy1_low_current1;
	int vooc_strategy1_low_current0;
	int vooc_strategy2_batt_up_temp1;
	int vooc_strategy2_batt_up_down_temp2;
	int vooc_strategy2_batt_up_temp3;
	int vooc_strategy2_batt_up_down_temp4;
	int vooc_strategy2_batt_up_temp5;
	int vooc_strategy2_batt_up_temp6;
	int vooc_strategy2_high0_current;
	int vooc_strategy2_high1_current;
	int vooc_strategy2_high2_current;
	int vooc_strategy2_high3_current;
	int fastchg_batt_temp_status;
	int vooc_batt_over_high_temp;
	int vooc_batt_over_low_temp;
	int vooc_over_high_or_low_current;
	int vooc_strategy_change_count;
	bool w_soc_temp_to_mcu;
	bool temp_range_init;
	int soc_range;
	int fastcharge_fail_count;
	int *vooc_current_lvl;
	int vooc_current_lvl_cnt;
	int detach_unexpectly;
	bool reset_adapter;
	bool suspend_charger;
	bool parse_fw_from_dt;
#ifdef OPLUS_CUSTOM_OP_DEF
	bool hiz_gnd_cable;
#endif
	int *abnormal_adapter_current;
	int abnormal_adapter_current_cnt;
	int allowed_current_max;
	int skin_thermal_high_threshold;
	int skin_thermal_normal_threshold;
	bool ctrl_by_skin_therm;
};

struct oplus_vooc_cp {
	void (*hardware_init_cp)(void);
	void (*vooc_enable_cp)(void);
	void (*vooc_disable_cp)(void);
	void (*cp_dump_reg)(void);
	int (*cp_hardware_init_svooc)(void);
	int (*cp_hardware_init_vooc)(void);
	int (*oplus_reset_cp)(void);
	int (*enable_cp_for_otg)(int en);
	int (*enalbe_ovp)(int en);
	int (*cp_hardware_init_pdqc)(void);
};

#define MAX_FW_NAME_LENGTH	60
#define MAX_DEVICE_VERSION_LENGTH 16
#define MAX_DEVICE_MANU_LENGTH    60
struct oplus_vooc_operations {
	int (*fw_update)(struct oplus_vooc_chip *chip);
	int (*fw_check_then_recover)(struct oplus_vooc_chip *chip);
	void (*eint_regist)(struct oplus_vooc_chip *chip);
	void (*eint_unregist)(struct oplus_vooc_chip *chip);
	void (*set_data_active)(struct oplus_vooc_chip *chip);
	void (*set_data_sleep)(struct oplus_vooc_chip *chip);
	void (*set_clock_active)(struct oplus_vooc_chip *chip);
	void (*set_clock_sleep)(struct oplus_vooc_chip *chip);
	void (*set_switch_mode)(struct oplus_vooc_chip *chip, int mode);
	int (*get_gpio_ap_data)(struct oplus_vooc_chip *chip);
	int (*read_ap_data)(struct oplus_vooc_chip *chip);
	void (*reply_mcu_data)(struct oplus_vooc_chip *chip, int ret_info, int device_type);
	void (*reply_mcu_data_4bits)(struct oplus_vooc_chip *chip,
		int ret_info, int device_type);
	void (*reset_fastchg_after_usbout)(struct oplus_vooc_chip *chip);
	void (*switch_fast_chg)(struct oplus_vooc_chip *chip);
	void (*reset_mcu)(struct oplus_vooc_chip *chip);
	void (*set_mcu_sleep)(struct oplus_vooc_chip *chip);
	void (*set_vooc_chargerid_switch_val)(struct oplus_vooc_chip *chip, int value);
	bool (*is_power_off_charging)(struct oplus_vooc_chip *chip);
	int (*get_reset_gpio_val)(struct oplus_vooc_chip *chip);
	int (*get_switch_gpio_val)(struct oplus_vooc_chip *chip);
	int (*get_ap_clk_gpio_val)(struct oplus_vooc_chip *chip);
	int (*get_fw_version)(struct oplus_vooc_chip *chip);
	int (*get_clk_gpio_num)(struct oplus_vooc_chip *chip);
	int (*get_data_gpio_num)(struct oplus_vooc_chip *chip);
	void (*update_temperature_soc)(void);
};

void oplus_vooc_init(struct oplus_vooc_chip *chip);
void oplus_vooc_init_cp(struct oplus_vooc_cp *cp);
void oplus_vooc_shedule_fastchg_work(void);
void oplus_vooc_read_fw_version_init(struct oplus_vooc_chip *chip);
void oplus_vooc_fw_update_work_init(struct oplus_vooc_chip *chip);
bool oplus_vooc_wake_fastchg_work(struct oplus_vooc_chip *chip);
void oplus_vooc_print_log(void);
void oplus_vooc_switch_mode(int mode);
bool oplus_vooc_get_allow_reading(void);
bool oplus_vooc_get_fastchg_started(void);
bool oplus_vooc_get_fastchg_ing(void);
bool oplus_vooc_get_fastchg_allow(void);
void oplus_vooc_set_fastchg_allow(int enable);
bool oplus_vooc_get_fastchg_to_normal(void);
void oplus_vooc_set_fastchg_to_normal_false(void);
bool oplus_vooc_get_fastchg_to_warm(void);
void oplus_vooc_set_fastchg_to_warm_false(void);
void oplus_vooc_set_fastchg_type_unknow(void);
bool oplus_vooc_get_fastchg_low_temp_full(void);
void oplus_vooc_set_fastchg_low_temp_full_false(void);
bool oplus_vooc_get_vooc_multistep_adjust_current_support(void);
bool oplus_vooc_get_fastchg_dummy_started(void);
void oplus_vooc_set_fastchg_dummy_started_false(void);
int oplus_vooc_get_adapter_update_status(void);
int oplus_vooc_get_adapter_update_real_status(void);
bool oplus_vooc_get_btb_temp_over(void);
void oplus_vooc_reset_fastchg_after_usbout(void);
void oplus_vooc_switch_fast_chg(void);
void oplus_vooc_reset_mcu(void);
int oplus_vooc_get_reset_gpio_status(void);
void oplus_vooc_set_mcu_sleep(void);
void oplus_vooc_set_vooc_chargerid_switch_val(int value);
void oplus_vooc_set_ap_clk_high(void);
int oplus_vooc_get_vooc_switch_val(void);
bool oplus_vooc_check_chip_is_null(void);
void oplus_vooc_battery_update(void);
void vooc_reset_cp(void);
void vooc_enable_cp_for_otg(int en);
void vooc_enable_cp_ovp(int en);
int is_vooc_support_single_batt_svooc(void);
int vooc_enable_cp_for_pdqc(void);
int vooc_get_fastcharge_fail_count(void);

int oplus_vooc_get_uart_tx(void);
int oplus_vooc_get_uart_rx(void);
void oplus_vooc_uart_init(void);
void oplus_vooc_uart_reset(void);
void oplus_vooc_set_adapter_update_real_status(int real);
void oplus_vooc_set_adapter_update_report_status(int report);
int oplus_vooc_get_fast_chg_type(void);
int oplus_gauge_get_fast_chg_type(void);
void oplus_vooc_set_disable_adapter_output(bool disable);
void oplus_vooc_set_vooc_max_current_limit(int current_level);
bool oplus_vooc_get_detach_unexpectly(void);
void oplus_vooc_set_detach_unexpectly(bool val);
void oplus_vooc_turn_off_fastchg(void);
int oplus_vooc_get_reply_bits(void);
extern int get_vooc_mcu_type(struct oplus_vooc_chip *chip);
bool opchg_get_mcu_update_state(void);
void oplus_vooc_get_vooc_chip_handle(struct oplus_vooc_chip **chip);
void oplus_vooc_reset_temp_range(struct oplus_vooc_chip *chip);
void oplus_vooc_check_set_mcu_sleep(void);
bool oplus_vooc_get_reset_adapter_st(void);
bool oplus_vooc_get_mcu_update_finish_status(void);
void oplus_vooc_set_update_finish_status_false(void);
int oplus_vooc_get_reset_active_status(void);
int oplus_vooc_get_abnormal_adapter_current_cnt(void);
#endif /* _OPLUS_VOOC_H */
