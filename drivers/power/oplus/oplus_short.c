/**********************************************************************************
* Copyright (c)  2017-2019  Guangdong OPLUS Mobile Comm Corp., Ltd
* OPLUS_FEATURE_CHG_BASIC
* Description: For short circuit battery check
* Version   : 1.0
* Date      : 2017-10-01
* ------------------------------ Revision History: --------------------------------
* <version>       <date>        	<author>              		<desc>
***********************************************************************************/
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>

#ifdef CONFIG_OPLUS_CHARGER_MTK
#include <linux/slab.h>
#else
#include <linux/spinlock.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/sched.h>
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
#include <linux/version.h>
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0))
#include <linux/qpnp/qpnp-adc.h>
#endif
#include <linux/batterydata-lib.h>
#include <linux/of_batterydata.h>
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0))
#include <linux/msm_bcl.h>
#endif
#include <linux/ktime.h>
#include <linux/kernel.h>
#include <soc/oplus/system/boot_mode.h>
#endif

#include "oplus_charger.h"
#include "oplus_short.h"
#include "oplus_vooc.h"
#include "charger_ic/oplus_short_ic.h"

#define OPLUS_CHG_UPDATE_INTERVAL_SEC	5

#ifdef CONFIG_OPLUS_SHORT_C_BATT_CHECK
static bool short_d_on = false;
#define short_debug(fmt,...) \
	do { \
		if (short_d_on == true) { \
		printk(KERN_CRIT pr_fmt("=====[OPLUS_CHG]"fmt),##__VA_ARGS__); \
		} \
	} while (0)

static int short_c_battery_status = SHORT_C_BATT_STATUS__NORMAL;
static int short_c_switch_status = SHORT_C_BATT_SW_STATUS__OFF;
static int short_c_feature_sw_status = SHORT_C_BATT_FEATURE_SW_STATUS__ON;
static int short_c_feature_hw_status = SHORT_C_BATT_FEATURE_HW_STATUS__OFF;

static int __init oplus_short_c_battery_status_init(char *str)
{
	sscanf(str, "%d", &short_c_battery_status);
	chg_err("short_c_battery_status[%d]\n", short_c_battery_status);
	return 0;
}
__setup("short_c_battery_status=", oplus_short_c_battery_status_init);

static int __init oplus_short_c_switch_status_init(char *str)
{
	sscanf(str, "%d", &short_c_switch_status);
	chg_err("short_c_switch_status[%d]\n", short_c_switch_status);
	return 0;
}
__setup("short_c_switch_status=", oplus_short_c_switch_status_init);

static int __init oplus_short_c_feature_sw_status_init(char *str)
{
	sscanf(str, "%d", &short_c_feature_sw_status);
	chg_err("short_c_feature_sw_status[%d]\n", short_c_feature_sw_status);
	return 0;
}
__setup("short_c_feature_sw_status=", oplus_short_c_feature_sw_status_init);

static int __init oplus_short_c_feature_hw_status_init(char *str)
{
	sscanf(str, "%d", &short_c_feature_hw_status);
	chg_err("short_c_feature_hw_status[%d]\n", short_c_feature_hw_status);
	return 0;
}
__setup("short_c_feature_hw_status=", oplus_short_c_feature_hw_status_init);

int oplus_short_c_batt_err_code_init(void)
{
	return short_c_battery_status;
}

int oplus_short_c_batt_chg_switch_init(void)
{
	return short_c_switch_status;
}

int oplus_short_c_batt_feature_sw_status_init(void)
{
	return short_c_feature_sw_status;
}

int oplus_short_c_batt_feature_hw_status_init(void)
{
	return short_c_feature_hw_status;
}

static struct short_c_batt_item short_c_batt_items[] =  {
	/*batt chging cycle*/
	{
		.name = "batt_chging_cycle_threshold",
		.value = 0
	},
	{
		.name = "batt_chging_cycles",
		.value = 0
	},
	/*timer*/
	{
		.name = "cv_timer1",
		.value = 0
	},
	{
		.name = "full_timer2",
		.value = 0
	},
	{
		.name = "full_timer3",
		.value = 0
	},
	{
		.name = "full_timer4",
		.value = 0
	},
	{
		.name = "full_timer5",
		.value = 0
	},
	/*rbatt*/
	{
		.name = "cool_temp_rbatt",
		.value = 0
	},
	{
		.name = "little_cool_temp_rbatt",
		.value = 0
	},
	{
		.name = "normal_temp_rbatt",
		.value = 0
	},
	/*delta vbatt*/
	{
		.name = "full_delta_vbatt1_mv",
		.value = 0
	},
	{
		.name = "full_delta_vbatt2_mv",
		.value = 0
	},
	/*exception2*/
	{
		.name = "ex2_lower_ibatt_ma",
		.value = 0
	},
	{
		.name = "ex2_low_ibatt_ma",
		.value = 0
	},
	{
		.name = "ex2_high_ibatt_ma",
		.value = 0
	},
	{
		.name = "ex2_lower_ibatt_count",
		.value = 0
	},
	{
		.name = "ex2_low_ibatt_count",
		.value = 0
	},
	{
		.name = "ex2_high_ibatt_count",
		.value = 0
	},
	/*dynamic delta vbatt*/
	{
		.name = "dyna1_low_avg_dv_mv",
		.value = 0
	},
	{
		.name = "dyna1_high_avg_dv_mv",
		.value = 0
	},
	{
		.name = "dyna1_delta_dv_mv",
		.value = 0
	},
	{
		.name = "dyna2_low_avg_dv_mv",
		.value = 0
	},
	{
		.name = "dyna2_high_avg_dv_mv",
		.value = 0
	},
	{
		.name = "dyna2_delta_dv_mv",
		.value = 0
	},
	/*control switch*/
	{
		.name = "is_recheck_on",
		.value = 0
	},
	{
		.name = "is_switch_on",
		.value = 0
	},
	{
		.name = "is_feature_on",
		.value = 0
	},
};

#ifdef CONFIG_OPLUS_CHARGER_MTK
static bool oplus_is_power_off_chg(struct oplus_chg_chip *chip)
{
#ifdef CONFIG_MTK_KERNEL_POWER_OFF_CHARGING
	if (chip->boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT)
		return true;
	else
#endif
		return false;
}

static bool oplus_is_normal_boot(struct oplus_chg_chip *chip)
{
	if (chip->boot_mode == NORMAL_BOOT)
		return true;
	return false;
}
#else /* CONFIG_OPLUS_CHARGER_MTK */
extern bool qpnp_is_power_off_charging(void);

static bool oplus_is_power_off_chg(struct oplus_chg_chip *chip)
{
	return qpnp_is_power_off_charging();
}

static bool oplus_is_normal_boot(struct oplus_chg_chip *chip)
{
	if (chip->boot_mode == MSM_BOOT_MODE__NORMAL)
		return true;
	return false;
}
#endif /* CONFIG_OPLUS_CHARGER_MTK */

extern void sort(void *base, size_t num, size_t size,
	int (*cmp_func)(const void *, const void *),
	void (*swap_func)(void *, void *, int size));

static int oplus_digit_num_len(const char *str)
{
	int len = 0;

	while (*str) {
		if (*str >= '0' && *str <= '9') {
			len++;
		} else {
			break;
		}
		str++;
	}
	return len;
}

static int oplus_cmpint(const void *a, const void *b)
{
	return *(int *)a - *(int *)b;
}

bool oplus_short_c_batt_is_prohibit_chg(struct oplus_chg_chip *chip)
{
	if (chip->short_c_batt.limit_chg) {
		chg_err("limit chg!\n");
		return true;
	}
	if (chip->short_c_batt.is_feature_hw_on
			== SHORT_C_BATT_FEATURE_HW_STATUS__ON) {
		if (chip->short_c_batt.shortc_gpio_status == 0) {
			return true;
		}
	}
	if (chip->short_c_batt.is_feature_hw_on
			== SHORT_C_BATT_FEATURE_HW_STATUS__ON) {
		if (chip->short_c_batt.ic_short_otp_st == false) {
			return true;
		}
	}
	if (chip->short_c_batt.is_feature_sw_on
			== SHORT_C_BATT_FEATURE_SW_STATUS__OFF) {
		return false;
	}
	if (chip->short_c_batt.is_switch_on) {
		if (chip->short_c_batt.err_code == SHORT_C_BATT_STATUS__CV_ERR_CODE1
				|| chip->short_c_batt.err_code == SHORT_C_BATT_STATUS__FULL_ERR_CODE2
				|| chip->short_c_batt.err_code == SHORT_C_BATT_STATUS__FULL_ERR_CODE3
				|| chip->short_c_batt.err_code == SHORT_C_BATT_STATUS__DYNAMIC_ERR_CODE4
				|| chip->short_c_batt.err_code == SHORT_C_BATT_STATUS__DYNAMIC_ERR_CODE5) {
			return true;
		}
	}
	return false;
}

bool oplus_short_c_batt_is_disable_rechg(struct oplus_chg_chip *chip)
{
	if (chip->short_c_batt.limit_rechg) {
		chg_err("limit rechg!\n");
		return true;
	}
	if (chip->short_c_batt.is_feature_sw_on == SHORT_C_BATT_FEATURE_SW_STATUS__OFF) {
		return false;
	}
	return chip->short_c_batt.disable_rechg;
}

static void oplus_short_c_batt_set_disable_rechg(struct oplus_chg_chip *chip, bool disable)
{
	chip->short_c_batt.disable_rechg = disable;
}

bool oplus_short_c_batt_get_cv_status(struct oplus_chg_chip *chip)
{
	return chip->short_c_batt.cv_satus;
}

static bool oplus_short_c_batt_data_is_mounted(void)
{
	struct file *fp;

	fp = filp_open(BAD_CONFIG_FILE, O_RDONLY, 0644);
	if (IS_ERR(fp)) {
		return false;
	} else {
		filp_close(fp, NULL);
		return true;
	}
}

static int oplus_short_c_batt_read_file(struct oplus_chg_chip *chip)
{
	struct file *fp;
	mm_segment_t old_fs;
	loff_t pos;
	char *buf;
	char *p;
	char buf_tmp[32] = {'\0'};
	int ret;
	int i;
	ssize_t nread;

	fp = filp_open(BAD_CONFIG_FILE, O_RDONLY, 0644);
	if (IS_ERR(fp)) {
		chg_err("open %s file error\n", BAD_CONFIG_FILE);
		return -1;
	}

	buf = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (buf == NULL) {
		chg_err("failed to allocate buffer\n");
		filp_close(fp, NULL);
		return -ENOMEM;
	}
	memset(buf, '\0', PAGE_SIZE - 1);

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	nread = vfs_read(fp, buf, PAGE_SIZE - 1, &pos);
	filp_close(fp, NULL);
	set_fs(old_fs);
	if (nread < 0) {
		chg_err("read %s file error\n", BAD_CONFIG_FILE);
		goto err;
	}

	for (i = 0; i < ARRAY_SIZE(short_c_batt_items); i++) {
		p = strstr(buf, short_c_batt_items[i].name);
		if (p != 0) {
			p = p + strlen(short_c_batt_items[i].name) + strlen(":");
			strncpy(buf_tmp, p, oplus_digit_num_len(p));
			buf_tmp[oplus_digit_num_len(p)] = '\0';
			ret = kstrtouint(buf_tmp, 10, &(short_c_batt_items[i].value));
			if (ret != 0) {
				short_c_batt_items[i].value = INVALID_DATA;
				chg_err("err: %s[%d]\n", short_c_batt_items[i].name, short_c_batt_items[i].value);
			} else {
				/*userspace write abs(INVALID_DATA) as invalid data*/
				if (short_c_batt_items[i].value == -INVALID_DATA)
					short_c_batt_items[i].value = INVALID_DATA;
				chg_err("%s[%d]\n", short_c_batt_items[i].name, short_c_batt_items[i].value);
				short_debug("%s[%d]\n", short_c_batt_items[i].name, short_c_batt_items[i].value);
			}
		} else {
			short_c_batt_items[i].value = INVALID_DATA;
			chg_err("CAN NOT find %s, set to INVALID_DATA\n", short_c_batt_items[i].name);
		}
	}

	chip->short_c_batt.batt_chging_cycle_threshold = short_c_batt_items[0].value;
	chip->short_c_batt.batt_chging_cycles = short_c_batt_items[1].value;
	chip->short_c_batt.cv_timer1
		= short_c_batt_items[2].value / OPLUS_CHG_UPDATE_INTERVAL_SEC;
	chip->short_c_batt.full_timer2
		= short_c_batt_items[3].value / OPLUS_CHG_UPDATE_INTERVAL_SEC;
	chip->short_c_batt.full_timer3
		= short_c_batt_items[4].value / OPLUS_CHG_UPDATE_INTERVAL_SEC;
	chip->short_c_batt.full_timer4
		= short_c_batt_items[5].value / OPLUS_CHG_UPDATE_INTERVAL_SEC;
	chip->short_c_batt.full_timer5
		= short_c_batt_items[6].value / OPLUS_CHG_UPDATE_INTERVAL_SEC;
	chip->short_c_batt.cool_temp_rbatt = short_c_batt_items[7].value;
	chip->short_c_batt.little_cool_temp_rbatt = short_c_batt_items[8].value;
	chip->short_c_batt.normal_temp_rbatt = short_c_batt_items[9].value;

	chip->short_c_batt.full_delta_vbatt1_mv = short_c_batt_items[10].value;
	chip->short_c_batt.full_delta_vbatt2_mv = short_c_batt_items[11].value;

	chip->short_c_batt.ex2_lower_ibatt_ma = short_c_batt_items[12].value;
	chip->short_c_batt.ex2_low_ibatt_ma = short_c_batt_items[13].value;
	chip->short_c_batt.ex2_high_ibatt_ma = short_c_batt_items[14].value;
	chip->short_c_batt.ex2_lower_ibatt_count = short_c_batt_items[15].value;
	chip->short_c_batt.ex2_low_ibatt_count = short_c_batt_items[16].value;
	chip->short_c_batt.ex2_high_ibatt_count = short_c_batt_items[17].value;

	chip->short_c_batt.dyna1_low_avg_dv_mv = short_c_batt_items[18].value;
	chip->short_c_batt.dyna1_high_avg_dv_mv = short_c_batt_items[19].value;
	chip->short_c_batt.dyna1_delta_dv_mv = short_c_batt_items[20].value;
	chip->short_c_batt.dyna2_low_avg_dv_mv = short_c_batt_items[21].value;
	chip->short_c_batt.dyna2_high_avg_dv_mv = short_c_batt_items[22].value;
	chip->short_c_batt.dyna2_delta_dv_mv = short_c_batt_items[23].value;

	chip->short_c_batt.is_recheck_on = short_c_batt_items[24].value;
	chip->short_c_batt.is_switch_on = short_c_batt_items[25].value;
	chip->short_c_batt.is_feature_sw_on = short_c_batt_items[26].value;

err:
	kfree(buf);
	return 0;
}

static bool oplus_is_algorithm_parameters_valid(struct oplus_chg_chip *chip)
{
	if (chip->short_c_batt.batt_chging_cycle_threshold < 1
			|| chip->short_c_batt.batt_chging_cycles < 0
			|| chip->short_c_batt.cv_timer1 < 1
			|| chip->short_c_batt.cv_timer1 > chip->limits.max_chg_time_sec
			|| chip->short_c_batt.full_timer2 < 1
			|| chip->short_c_batt.full_timer2 > chip->limits.max_chg_time_sec
			|| chip->short_c_batt.full_timer3 < 1
			|| chip->short_c_batt.full_timer3 > chip->limits.max_chg_time_sec
			|| chip->short_c_batt.full_timer4 < 1
			|| chip->short_c_batt.full_timer4 > chip->limits.max_chg_time_sec
			|| chip->short_c_batt.full_timer5 < 1
			|| chip->short_c_batt.full_timer5 > chip->limits.max_chg_time_sec
			|| chip->short_c_batt.cool_temp_rbatt < 1
			|| chip->short_c_batt.little_cool_temp_rbatt < 1
			|| chip->short_c_batt.normal_temp_rbatt < 1
			|| chip->short_c_batt.full_delta_vbatt1_mv < 1
			|| chip->short_c_batt.full_delta_vbatt2_mv < 1
			|| chip->short_c_batt.ex2_lower_ibatt_ma < 1
			|| chip->short_c_batt.ex2_lower_ibatt_ma > 2000
			|| chip->short_c_batt.ex2_low_ibatt_ma < 1
			|| chip->short_c_batt.ex2_low_ibatt_ma > 2000
			|| chip->short_c_batt.ex2_high_ibatt_ma < 1
			|| chip->short_c_batt.ex2_high_ibatt_ma > 2000
			|| chip->short_c_batt.ex2_lower_ibatt_count < 1
			|| chip->short_c_batt.ex2_low_ibatt_count < 1
			|| chip->short_c_batt.ex2_high_ibatt_count < 1
			|| chip->short_c_batt.dyna1_delta_dv_mv < 1
			|| chip->short_c_batt.dyna2_delta_dv_mv < 1) {
		return false;
	}

	return true;
}

/* return < 0 for error, return >= 0 for OK */
/* err_code1:T1,T2,-1,-1,-1
 * err_code2:T1,Tn,VBAT1,VBAT2,TIMER3_COUNTS
 * err_code3:T1,Tn,VBAT1,VBAT2,TIMER4_COUNTS
 */
static int oplus_short_c_batt_write_err_code(int err_code,
		int temp1,int temp2, int vbatt1_mv, int vbatt2_mv, int timer_counts)
{
	struct file *fp;
	mm_segment_t old_fs;
	loff_t pos;
	ssize_t len;
	char buf[256] = {'\0'};
	char buf_log[256] = {'\0'};

	fp = filp_open(ERR_CODE_FILE, O_RDWR | O_CREAT, 0644);
	if (IS_ERR(fp)) {
		chg_err("create %s file error\n", ERR_CODE_FILE);
		return -1;
	}

	/*add ",#" and the end of the string for userspace*/
	sprintf(buf, "err_code%d:%d,%d,%d,%d,%d,#", err_code,
		temp1, temp2, vbatt1_mv, vbatt2_mv, timer_counts);
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		pos = 0;
		len = vfs_write(fp, buf, strlen(buf), &pos);
	if (len < 0) {
		chg_err("write %s file error\n", ERR_CODE_FILE);
	}
	pos = 0;
	vfs_read(fp, buf_log, sizeof(buf), &pos);
	chg_err("%s\n", buf_log);

	filp_close(fp, NULL);
	set_fs(old_fs);

	return len;
}

#define INVALID_CODE								INVALID_DATA
#define EXIT_CODE__OVER_DELTA_TEMP_IN_TIMER3		1
#define EXIT_CODE__TIMER4_OVERFLOW					2
#define EXIT_CODE__OVER_DELTA_TEMP_IN_TIMER4		3
#define EXIT_CODE__EX2_OVER_LOW_IBATT				5
#define EXIT_CODE__EX2_OVER_HIGH_IBATT				6
#define EXIT_CODE__EX2_OVER_LOWER_IBATT				7
#define EXIT_CODE__EX5_OUT_OF_TEMP_0_45				11
#define EXIT_CODE__OVER_RBATT						12
#define EXIT_CODE__NOT_OVER_RBATT					100
#define EXIT_CODE__TIMER3_OVERFLOW_INFO				101

/* exit_code1:T1,Tn,VBAT1,VBAT2 */
static int oplus_short_c_batt_write_exit_code(int exit_code,
		int temp1, int temp2, int vbatt1, int vbatt2)
{
	struct file *fp;
	mm_segment_t old_fs;
	loff_t pos;
	ssize_t len;
	char buf[256] = {'\0'};
	char buf_log[256] = {'\0'};

	fp = filp_open(EXIT_CODE_FILE, O_RDWR | O_CREAT, 0644);
	if (IS_ERR(fp)) {
		chg_err("create %s file error\n", EXIT_CODE_FILE);
		return -1;
	}

	/*add ",#" and the end of the string for userspace*/
	sprintf(buf, "exit_code%d:%d,%d,%d,%d,#", exit_code, temp1, temp2, vbatt1, vbatt2);

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	len = vfs_write(fp, buf, strlen(buf), &pos);
	if (len < 0) {
		chg_err("write %s file error\n", EXIT_CODE_FILE);
	}
	pos = 0;
	vfs_read(fp, buf_log, sizeof(buf), &pos);
	chg_err("%s\n", buf_log);

	filp_close(fp, NULL);
	set_fs(old_fs);

	return len;
}

static int oplus_short_c_batt_write_chg_data(int current_ma, int volt_mv,
		int lower_count, int low_count, int high_count)
{
	struct file *fp;
	mm_segment_t old_fs;
	loff_t pos;
	ssize_t len;
	char buf[256] = {'\0'};
	char buf_log[256] = {'\0'};
	static int pre_lower_count = -1;
	static int pre_low_count = -1;
	static int pre_high_count = -1;

	fp = filp_open(CHG_DATA_FILE, O_RDWR | O_CREAT, 0644);
	if (IS_ERR(fp)) {
		chg_err("create %s file error\n", CHG_DATA_FILE);
		return -1;
	}

	/*add ",#" and the end of the string for userspace*/
	sprintf(buf, "chg_data:%d,%d,%d,%d,%d,#", current_ma, volt_mv,
			lower_count, low_count, high_count);

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	len = vfs_write(fp, buf, strlen(buf), &pos);
	if (len < 0)
		chg_err("write %s file error\n", CHG_DATA_FILE);

	pos = 0;
	vfs_read(fp, buf_log, sizeof(buf), &pos);
	if (pre_lower_count != lower_count || pre_low_count != low_count
			|| pre_high_count != high_count) {
		short_debug("%s\n", buf_log);
		pre_lower_count = lower_count;
		pre_low_count = low_count;
		pre_high_count = high_count;
	}

	filp_close(fp, NULL);
	set_fs(old_fs);

	return len;
}

#define SHORT_C_BATT_UPDATE_RESET		0
#define SHORT_C_BATT_UPDATE_PARM		1
#define SHORT_C_BATT_CLR_ERR_CODE		2
#define SHORT_C_BATT_DEBUG_ON			98
void oplus_short_c_batt_update_change(struct oplus_chg_chip *chip, int update_value)
{
	if (update_value == SHORT_C_BATT_UPDATE_PARM) {
		oplus_short_c_batt_read_file(chip);
	} else if (update_value == SHORT_C_BATT_CLR_ERR_CODE) {
		oplus_short_c_batt_write_err_code(0, 0, 0, 0, 0, 0);
	}else if (update_value == SHORT_C_BATT_DEBUG_ON) {
		short_d_on = true;
	}
}

/* when get 10 valid vbatt, return avg_vbatt_mv, or return 0 */
static int oplus_short_c_batt_get_average_vbatt_mv(struct oplus_chg_chip *chip, bool reset)
{
	static int valid_vbatt_count = 0;
	static int buf[30] = {0};
	static int avg_vbatt_mv = 0;
	int i;

	if (reset == true) {
		valid_vbatt_count = 0;
		avg_vbatt_mv = 0;
		return 0;
	}

	if (abs(chip->icharging) < 10) {
		if (valid_vbatt_count == 0) {
			for (i = 0; i < ARRAY_SIZE(buf); i++) {
				buf[i] = chip->batt_volt;
			}
		}
		if (valid_vbatt_count < ARRAY_SIZE(buf)) {
			buf[valid_vbatt_count] = chip->batt_volt;
		}
		valid_vbatt_count++;
	}
#if 0
	if (valid_vbatt_count == ARRAY_SIZE(buf)) {
		avg_vbatt_mv = 0;
		sort(buf, ARRAY_SIZE(buf), sizeof(buf[0]), oplus_cmpint, NULL);
		for (i = 3; i < ARRAY_SIZE(buf) - 3; i++)
			avg_vbatt_mv = avg_vbatt_mv + buf[i];
		valid_vbatt_count = 0;
	}

	return avg_vbatt_mv / 4;
#else
	/* 2017-12-18 use the max value instead of avg value */
	if (valid_vbatt_count == ARRAY_SIZE(buf)) {
		avg_vbatt_mv = 0;
		sort(buf, ARRAY_SIZE(buf), sizeof(buf[0]), oplus_cmpint, NULL);
		avg_vbatt_mv = buf[ARRAY_SIZE(buf) - 1];
		valid_vbatt_count = 0;
	}

	return avg_vbatt_mv;
#endif
}


#define D_SHORTC_HW_CNT				3
void oplus_chg_short_c_hw_check(struct oplus_chg_chip *chip)
{
	static int short_c_hw_check_counts = 0;
	bool shortc_gpio_status = true;
	if (!chip || !chip->chg_ops->get_shortc_hw_gpio_status) {
		return;
	}
	shortc_gpio_status = chip->short_c_batt.shortc_gpio_status;
	if (chip->chg_ops->get_shortc_hw_gpio_status() == 0) {
		short_c_hw_check_counts ++;
		chg_debug("[Shortc_HW] shorc_HW gpio low, count = %d, status = %d\n",
				short_c_hw_check_counts, chip->short_c_batt.shortc_gpio_status);
		if(short_c_hw_check_counts >= D_SHORTC_HW_CNT) {
			short_c_hw_check_counts = 0;
			chip->short_c_batt.shortc_gpio_status = 0;
		}
	} else {
		short_c_hw_check_counts = 0;
		chip->short_c_batt.shortc_gpio_status = 1;
	}
	if (chip->short_c_batt.is_feature_hw_on
			== SHORT_C_BATT_FEATURE_HW_STATUS__OFF) {
		return;
	}
	if (chip->charger_exist && (shortc_gpio_status
				!= chip->short_c_batt.shortc_gpio_status)) {
		if (oplus_vooc_get_fastchg_started() == true) {
			oplus_chg_set_chargerid_switch_val(0);
			oplus_vooc_switch_mode(NORMAL_CHARGER_MODE);
		}
		oplus_chg_turn_on_charging(chip);
		chg_debug("[Shortc_HW] shorc_HW gpio status changed! \
			gpio_stat_pre = %d, gpio_stat = %d\n",
			shortc_gpio_status, chip->short_c_batt.shortc_gpio_status);
	}
}

void oplus_chg_short_c_ic_check(struct oplus_chg_chip *chip)
{
	bool short_ic_otp_st = true;

	if (!chip) {
		return;
	}

	short_ic_otp_st = chip->short_c_batt.ic_short_otp_st;
	chip->short_c_batt.ic_short_otp_st = oplus_short_ic_otp_check();
	if (chip->short_c_batt.is_feature_hw_on
			== SHORT_C_BATT_FEATURE_HW_STATUS__OFF) {
		return;
	}
	if (chip->charger_exist && (short_ic_otp_st
				!= chip->short_c_batt.ic_short_otp_st)) {
		if (oplus_vooc_get_fastchg_started() == true) {
			oplus_chg_set_chargerid_switch_val(0);
			oplus_vooc_switch_mode(NORMAL_CHARGER_MODE);
		}
		oplus_chg_turn_on_charging(chip);
		chg_debug("[Shortc_ic] oplus_shorc_ic status changed! \
			ic_otp_pre = %d, ic_otp = %d\n",
			short_ic_otp_st, chip->short_c_batt.ic_short_otp_st);
	}
}

#define INPUT_LIMIT_MA_THRESHOLD	900
//#define CV_VBATT_MV_THRESHOLD	4350
#define CV_IBATT_MA_THRESHOLD		300
#define CV_PRE_CHECK_COUNT			5
#define CV_DELTA_IBATT_COUNT		10
#define CV_DELTA_IBATT_MA_THRESHOLD	50

#define CV_ITERM_COUNT				3
#define CV_VBATT_COUNT				10

#define LITTLE_COLD_TEMP_THRESHOLD	20
#define COOL_TEMP_THRESHOLD			120
#define NORMAL_TEMP_THRESHOLD		250
#define WARM_TEMP_THRESHOLD			430
#define DELTA_TEMP_THRESHOLD		50

//#define FULL_TIMER2_30MIN_COUNT	360 //1800/5
//#define FULL_TIMER3_20MIN_COUNT	240 //1200/5
//#define FULL_TIMER4_40MIN_COUNT	480 //2400/5

#define INVALID_TEMP				INVALID_DATA

void oplus_chg_short_c_battery_check(struct oplus_chg_chip *chip)
{
	static int cv_pre_check_count = 0;
	static int cv_timer1_count = 0;
	//static int cv_timer1_count_limit = 0;
	static int cv_delta_ibatt_count = 0;
	static int cv_temp = INVALID_TEMP;

	static int cv_iterm_count = 0;
	static int cv_avg_vbatt = 0;
	static int cv_vbatt_count = 0;

	static int full_over_10ma_count = 0;
	static int full_over_20ma_count = 0;
	static int full_over_50ma_count = 0;
	static int full_timer2_count = 0;
	static int full_timer3_count = 0;
	static int full_timer4_count = 0;
	static int full_timer5_count = 0;
	static int full_vbatt1 = 0;
	static int full_temp1 = INVALID_TEMP;
	int full_vbatt_n;
	int full_vbatt_2;
	int rbatt;

	static int pre_icharging = 0;
	static bool exit_check_status = false;

	static int data_mounted_count = 0;

	oplus_chg_short_c_hw_check(chip);
	oplus_chg_short_c_ic_check(chip);

	if (chip->short_c_batt.short_c_bat_cv_mv == -EINVAL
			|| chip->limits.short_c_bat_vfloat_mv == -EINVAL
			|| chip->limits.short_c_bat_fastchg_current_ma == -EINVAL) {
		return;
	}
	/*If this feature is OFF, clear anything about charging control and goto cv_reset for doing nothing*/
	if (chip->short_c_batt.is_feature_sw_on == SHORT_C_BATT_FEATURE_SW_STATUS__OFF) {
		chip->short_c_batt.err_code = SHORT_C_BATT_STATUS__NORMAL;
		goto cv_reset;
	}

	if (chip->charger_type == POWER_SUPPLY_TYPE_USB_DCP) {
		if (oplus_is_power_off_chg(chip) == true) {
			data_mounted_count++;
			if (data_mounted_count < 30
					&& oplus_short_c_batt_data_is_mounted() == true) {//3min
				oplus_short_c_batt_read_file(chip);
				data_mounted_count = 30;
			}
			if (data_mounted_count > 30) {
				data_mounted_count = 30;
			}
		} else if (oplus_is_normal_boot(chip) == false) {
			chg_err("oplus_is_normal_boot[false]\n");
			return;
		}
	}

	if (chip->charger_type == POWER_SUPPLY_TYPE_UNKNOWN
			&& chip->short_c_batt.is_recheck_on != 0) {
		if (chip->short_c_batt.err_code != 0) {
			chg_err("clear errcode\n");
			chip->short_c_batt.err_code = 0;
		}
	}

	if (chip->short_c_batt.update_change == SHORT_C_BATT_UPDATE_PARM) {
		chip->short_c_batt.update_change = SHORT_C_BATT_UPDATE_RESET;
		/*bootup phone and algorithm parameters change, userspace will set update_change
		 *to 1, if is_recheck_on==1 we need to reset err_code for userspace to count err_codes
		 */
		if (chip->short_c_batt.is_recheck_on != 0) {
			chg_err("clear errcode\n");
			chip->short_c_batt.err_code = 0;
		}
		goto cv_reset;
	}

	if (chip->charger_type != POWER_SUPPLY_TYPE_USB_DCP) {
		short_debug("chip->charger_type != POWER_SUPPLY_TYPE_USB_DCP\n");
		goto exit_check_reset;
	}

	/* complete this check */
	if (exit_check_status == true) {
		goto cv_reset;
	}

	/* check algorithm parameters */
	if (oplus_is_algorithm_parameters_valid(chip) == false) {
		chg_err("invalid parameters\n");
		goto cv_reset;
	}

	if ((chip->chg_ops->get_dyna_aicl_result)
			&& (chip->chg_ops->get_dyna_aicl_result() < INPUT_LIMIT_MA_THRESHOLD)) {
		short_debug("get_dyna_aicl_result[%d]\n", chip->chg_ops->get_dyna_aicl_result());
		goto cv_reset;
	}

	if (!chip->batt_exist || !chip->mmi_chg) {
		goto cv_reset;
	}

	/* known the battery status, no need to check */
	if (chip->short_c_batt.err_code == SHORT_C_BATT_STATUS__CV_ERR_CODE1
			|| (chip->short_c_batt.err_code == SHORT_C_BATT_STATUS__FULL_ERR_CODE2
				&& chip->batt_full != true) //set errcode2 at the 1st time, need to check errcode3
			|| chip->short_c_batt.err_code == SHORT_C_BATT_STATUS__FULL_ERR_CODE3
			|| (chip->short_c_batt.err_code == SHORT_C_BATT_STATUS__DYNAMIC_ERR_CODE4
				&& chip->batt_full != true) //set errcode4 at the 1st time, need to check errcode3
			|| chip->short_c_batt.err_code == SHORT_C_BATT_STATUS__DYNAMIC_ERR_CODE5) {
		chg_err("exit_check, errcode[%d]\n", chip->short_c_batt.err_code);
		short_debug("exit_check, errcode[%d]\n", chip->short_c_batt.err_code);
		exit_check_status = true;
		return;
	}

	if (chip->temperature <= LITTLE_COLD_TEMP_THRESHOLD || chip->temperature > WARM_TEMP_THRESHOLD) {//0~45
		short_debug("temperature[%d]\n", chip->temperature);
		if (oplus_short_c_batt_get_cv_status(chip) == true) {
			oplus_short_c_batt_write_exit_code(EXIT_CODE__EX5_OUT_OF_TEMP_0_45,
					INVALID_CODE, chip->temperature, INVALID_CODE, INVALID_CODE);
			chg_err("don't exit_check, temperature[%d]\n", chip->temperature);
			short_debug("don't exit_check, out of temp\n");
			goto cv_reset;
		} else {
			goto cv_reset;
		}
	}

	if (chip->short_c_batt.in_idle == false) {
		short_debug("don't exit_check, idle false\n");
		chg_err("don't exit_check, idle false\n");
		goto cv_reset;
	}

	/* delta vbatt check after full */
	if (chip->batt_full == true && chip->chging_on == false
			&& chip->charging_state == CHARGING_STATUS_FULL) {
		chip->short_c_batt.cv_satus = true;
		if (chip->icharging > chip->short_c_batt.ex2_lower_ibatt_ma) {
			full_over_10ma_count++;
		} else {
			full_over_10ma_count = 0;
		}
		if (chip->icharging > chip->short_c_batt.ex2_low_ibatt_ma) {
			full_over_20ma_count++;
		}
		if (chip->icharging > chip->short_c_batt.ex2_high_ibatt_ma) {
			full_over_50ma_count++;
		} else {
			full_over_50ma_count = 0;
		}
		oplus_short_c_batt_write_chg_data(chip->icharging, chip->batt_volt,
			full_over_10ma_count, full_over_20ma_count, full_over_50ma_count);

		if (full_over_10ma_count >= chip->short_c_batt.ex2_lower_ibatt_count) {
			oplus_short_c_batt_write_exit_code(EXIT_CODE__EX2_OVER_LOWER_IBATT,
				INVALID_CODE, INVALID_CODE, INVALID_CODE, INVALID_CODE);
			chg_err("don't exit_check, over [%d]mA [%d]times\n",
				chip->short_c_batt.ex2_lower_ibatt_ma, chip->short_c_batt.ex2_lower_ibatt_count);
			short_debug("don't exit_check, full_over_10ma_count[%d]\n", full_over_10ma_count);
			goto cv_reset;
		}
		if (full_over_20ma_count >= chip->short_c_batt.ex2_low_ibatt_count) {
			oplus_short_c_batt_write_exit_code(EXIT_CODE__EX2_OVER_LOW_IBATT,
				INVALID_CODE, INVALID_CODE, INVALID_CODE, INVALID_CODE);
			chg_err("don't exit_check, over [%d]mA [%d]times\n",
				chip->short_c_batt.ex2_low_ibatt_ma, chip->short_c_batt.ex2_low_ibatt_count);
			short_debug("don't exit_check, full_over_20ma_count[%d]\n", full_over_20ma_count);
			goto cv_reset;
		}
		if (full_over_50ma_count >= chip->short_c_batt.ex2_high_ibatt_count) {
			oplus_short_c_batt_write_exit_code(EXIT_CODE__EX2_OVER_HIGH_IBATT,
				INVALID_CODE, INVALID_CODE, INVALID_CODE, INVALID_CODE);
			chg_err("don't exit_check, over [%d]mA [%d]times\n",
				chip->short_c_batt.ex2_high_ibatt_ma, chip->short_c_batt.ex2_high_ibatt_count);
			short_debug("don't exit_check, full_over_50ma_count[%d]\n", full_over_50ma_count);
			goto cv_reset;
		}

		/* start timer5 */
		full_timer5_count++;
		/* timer5 not overflow */
		if (full_timer5_count <= chip->short_c_batt.full_timer5) {
			short_debug("full_timer5_count[%d]\n", full_timer5_count);
			oplus_short_c_batt_set_disable_rechg(chip, true);
			oplus_short_c_batt_get_average_vbatt_mv(chip, true);//reset
			return;
		}
		/* timer5 overflow then get VBAT2 */
		if (full_timer5_count > chip->short_c_batt.full_timer5
				&& full_timer5_count < chip->short_c_batt.full_timer5 + 2) {
			full_timer5_count = chip->short_c_batt.full_timer5;
			full_vbatt_2 = oplus_short_c_batt_get_average_vbatt_mv(chip, false);
			short_debug("full_timer5_count overflow, full_vbatt_2[%d]\n", full_vbatt_2);
			if (full_vbatt_2 > 0 && chip->limits.iterm_ma != 0) {
				full_timer5_count = chip->short_c_batt.full_timer5 + 2;//goto the next step
				if (cv_avg_vbatt > chip->short_c_batt.short_c_bat_cv_mv + 1000
						|| cv_avg_vbatt < chip->short_c_batt.short_c_bat_cv_mv - 1000) {
					cv_avg_vbatt = -1 * chip->short_c_batt.short_c_bat_cv_mv;
				}
				if (chip->limits.iterm_ma != 0) {
					rbatt = (cv_avg_vbatt - full_vbatt_2) * 1000 / chip->limits.iterm_ma;
				} else {
					rbatt = -1;
				}
				if (chip->temperature <= COOL_TEMP_THRESHOLD) {			//2~12
					if (rbatt > chip->short_c_batt.cool_temp_rbatt) {
						exit_check_status = true;
					}
				} else if (chip->temperature <= NORMAL_TEMP_THRESHOLD) {//12~25
					if (rbatt > chip->short_c_batt.little_cool_temp_rbatt) {
						exit_check_status = true;
					}
				} else {												//25~43
					if (rbatt > chip->short_c_batt.normal_temp_rbatt) {
						exit_check_status = true;
					}
				}
				short_debug("cv_avg_vbatt[%d], full_vbatt_2[%d], rbatt[%d]\n", cv_avg_vbatt, full_vbatt_2, rbatt);
				if (exit_check_status == true) {
					oplus_short_c_batt_write_exit_code(EXIT_CODE__OVER_RBATT,
						chip->temperature, chip->temperature, full_vbatt_2, rbatt);
					chg_err("exit_check, temp [%d] full_vbatt_2[%d] rbatt[%d]\n",
						chip->temperature, full_vbatt_2, rbatt);
					short_debug("exit_check, rbatt[%d]\n", rbatt);
					return;
				} else {
					oplus_short_c_batt_write_exit_code(EXIT_CODE__NOT_OVER_RBATT,
						chip->temperature, chip->temperature, full_vbatt_2, rbatt);
					short_debug("exit_code: EXIT_CODE__NOT_OVER_RBATT\n");
				}
			}
			return;
		}

		/* start timer2 */
		full_timer2_count++;
		/* timer2 not overflow */
		if (full_timer2_count <= chip->short_c_batt.full_timer2) {
			short_debug("full_timer2_count[%d]\n", full_timer2_count);
			oplus_short_c_batt_get_average_vbatt_mv(chip, true);//reset
			return;
		}
		/* timer2 overflow then get VBAT1 and T1 */
		if (full_timer2_count > chip->short_c_batt.full_timer2
				&& full_timer2_count < chip->short_c_batt.full_timer2 + 2) {
			full_timer2_count = chip->short_c_batt.full_timer2;
			full_vbatt_n = oplus_short_c_batt_get_average_vbatt_mv(chip, false);
			short_debug("full_timer2_count overflow, full_vbatt_1[%d]\n", full_vbatt_n);
			if (full_vbatt_n > 0 && full_vbatt1 == 0) {
				full_timer2_count = chip->short_c_batt.full_timer2 + 2;//goto the next step
				full_vbatt1 = full_vbatt_n;
				full_temp1 = chip->temperature;
				chg_err("full_vbatt1[%d], full_temp1[%d]\n", full_vbatt1, full_temp1);
			}
			return;
		}

		/* start timer3 */
		full_timer3_count++;
		/* timer3 not overflow */
		if (full_timer3_count <= chip->short_c_batt.full_timer3) {
			short_debug("full_timer3_count[%d]\n", full_timer3_count);
			oplus_short_c_batt_get_average_vbatt_mv(chip, true);//reset
			if (full_temp1 != INVALID_TEMP && abs(full_temp1 - chip->temperature) > DELTA_TEMP_THRESHOLD) {
				oplus_short_c_batt_write_exit_code(EXIT_CODE__OVER_DELTA_TEMP_IN_TIMER3,
					full_temp1, chip->temperature, INVALID_CODE, INVALID_CODE);
				chg_err("don't exit_check, temperature[%d]\n", chip->temperature);
				short_debug("don't exit_check, T1[%d], T2[%d]\n", full_temp1, chip->temperature);
				goto cv_reset;
			}
			return;
		}
		/* timer3 overflow then check err_code2 and dynamic err_code4 */
		if (full_timer3_count > chip->short_c_batt.full_timer3
				&& full_timer3_count < chip->short_c_batt.full_timer3 + 3) {
			full_vbatt_n = oplus_short_c_batt_get_average_vbatt_mv(chip, false);
			short_debug("full_timer3_count overflow, full_vbatt_n[%d]\n", full_vbatt_n);
			if (full_vbatt_n > 0 && full_vbatt1 > 0) {
				if (full_timer3_count == chip->short_c_batt.full_timer3 + 1) {
					if ((chip->short_c_batt.batt_chging_cycles < chip->short_c_batt.batt_chging_cycle_threshold)
							&& (full_vbatt1 - full_vbatt_n > chip->short_c_batt.full_delta_vbatt1_mv)) {
						//set err_code2;
						short_debug("set err_code[%d]\n", SHORT_C_BATT_STATUS__FULL_ERR_CODE2);
						chip->short_c_batt.err_code = SHORT_C_BATT_STATUS__FULL_ERR_CODE2;
						oplus_short_c_batt_write_err_code(chip->short_c_batt.err_code, full_temp1,
							chip->temperature, full_vbatt1, full_vbatt_n, full_timer3_count * 5);
					}
				}
				/* dynamic */
				if (full_timer3_count == chip->short_c_batt.full_timer3 + 2) {
					full_timer3_count = chip->short_c_batt.full_timer3 + 3;//goto the next step
					if (chip->temperature <= COOL_TEMP_THRESHOLD) {//2~12
						if (chip->short_c_batt.dyna1_low_avg_dv_mv != INVALID_DATA
								&& chip->short_c_batt.dyna1_low_avg_dv_mv >= 0) {
							if ((full_vbatt1 - full_vbatt_n - chip->short_c_batt.dyna1_low_avg_dv_mv > chip->short_c_batt.dyna1_delta_dv_mv)
									&& (full_vbatt1 - full_vbatt_n > chip->short_c_batt.full_delta_vbatt1_mv)) {
								//set err_code4;
								short_debug("set err_code[%d]\n", SHORT_C_BATT_STATUS__DYNAMIC_ERR_CODE4);
								chip->short_c_batt.err_code = SHORT_C_BATT_STATUS__DYNAMIC_ERR_CODE4;
								oplus_short_c_batt_write_err_code(chip->short_c_batt.err_code, full_temp1,
									chip->temperature, full_vbatt1, full_vbatt_n, INVALID_CODE);
							}
						}
					} else {//12~43
						if (chip->short_c_batt.dyna1_high_avg_dv_mv != INVALID_DATA
								&& chip->short_c_batt.dyna1_high_avg_dv_mv >= 0) {
							if ((full_vbatt1 - full_vbatt_n - chip->short_c_batt.dyna1_high_avg_dv_mv > chip->short_c_batt.dyna1_delta_dv_mv)
									&& (full_vbatt1 - full_vbatt_n > chip->short_c_batt.full_delta_vbatt1_mv)) {
								//set err_code4;
								short_debug("set err_code[%d]\n", SHORT_C_BATT_STATUS__DYNAMIC_ERR_CODE4);
								chip->short_c_batt.err_code = SHORT_C_BATT_STATUS__DYNAMIC_ERR_CODE4;
								oplus_short_c_batt_write_err_code(chip->short_c_batt.err_code, full_temp1,
									chip->temperature, full_vbatt1, full_vbatt_n, INVALID_CODE);
							}
						}
					}
					if (chip->short_c_batt.err_code != SHORT_C_BATT_STATUS__DYNAMIC_ERR_CODE4) {
						short_debug("exit_code, EXIT_CODE__TIMER3_OVERFLOW_INFO\n");
						oplus_short_c_batt_write_exit_code(EXIT_CODE__TIMER3_OVERFLOW_INFO,
							full_temp1, chip->temperature, full_vbatt1, full_vbatt_n);
					}
				}
			} else {
				full_timer3_count = chip->short_c_batt.full_timer3;
			}
			return;
		}

		/* start timer4 */
		full_timer4_count++;
		/* timer4 not overflow */
		if (full_timer4_count <= chip->short_c_batt.full_timer4) {
			short_debug("full_timer4_count[%d]\n", full_timer4_count);
			oplus_short_c_batt_get_average_vbatt_mv(chip, true);//reset
			if (full_temp1 != INVALID_TEMP && abs(full_temp1 - chip->temperature) > DELTA_TEMP_THRESHOLD) {
				oplus_short_c_batt_write_exit_code(EXIT_CODE__OVER_DELTA_TEMP_IN_TIMER4,
					full_temp1, chip->temperature, INVALID_CODE, INVALID_CODE);
				chg_err("don't exit_check, temperature[%d]\n", chip->temperature);
				short_debug("don't exit_check, T1[%d], T2[%d]\n", full_temp1, chip->temperature);
				goto cv_reset;
			}
			return;
		}
		/* timer4 overflow then check err_code3 and dynamic err_code5 */
		if (full_timer4_count > chip->short_c_batt.full_timer4
				&& full_timer4_count < chip->short_c_batt.full_timer4 + 3) {
			full_vbatt_n = oplus_short_c_batt_get_average_vbatt_mv(chip, false);
			short_debug("full_timer4_count overflow, full_vbatt_n[%d]\n", full_vbatt_n);
			if (full_vbatt_n > 0 && full_vbatt1 > 0) {
				if (full_timer4_count == chip->short_c_batt.full_timer4 + 1) {
					if ((chip->short_c_batt.batt_chging_cycles < chip->short_c_batt.batt_chging_cycle_threshold)
							&& (full_vbatt1 - full_vbatt_n > chip->short_c_batt.full_delta_vbatt2_mv)) {
						//set err_code3;
						short_debug("set err_code[%d]\n", SHORT_C_BATT_STATUS__FULL_ERR_CODE3);
						chip->short_c_batt.err_code = SHORT_C_BATT_STATUS__FULL_ERR_CODE3;
						oplus_short_c_batt_write_err_code(chip->short_c_batt.err_code, full_temp1,
							chip->temperature, full_vbatt1, full_vbatt_n, full_timer4_count * 5);
					}
				}
				/* dynamic */
				if (full_timer4_count == chip->short_c_batt.full_timer4 + 2) {
					full_timer4_count = chip->short_c_batt.full_timer4 + 3;//goto the next step
					if (chip->temperature <= COOL_TEMP_THRESHOLD) {//2~12
						if (chip->short_c_batt.dyna2_low_avg_dv_mv != INVALID_DATA
								&& chip->short_c_batt.dyna2_low_avg_dv_mv >= 0) {
							if ((full_vbatt1 - full_vbatt_n - chip->short_c_batt.dyna2_low_avg_dv_mv > chip->short_c_batt.dyna2_delta_dv_mv)
									&& (full_vbatt1 - full_vbatt_n > chip->short_c_batt.full_delta_vbatt2_mv)) {
								//set err_code5;
								short_debug("set err_code[%d]\n", SHORT_C_BATT_STATUS__DYNAMIC_ERR_CODE5);
								chip->short_c_batt.err_code = SHORT_C_BATT_STATUS__DYNAMIC_ERR_CODE5;
								oplus_short_c_batt_write_err_code(chip->short_c_batt.err_code, full_temp1,
									chip->temperature, full_vbatt1, full_vbatt_n, INVALID_CODE);
							}
						}
					} else {//12~43
						if (chip->short_c_batt.dyna2_high_avg_dv_mv != INVALID_DATA
								&& chip->short_c_batt.dyna2_high_avg_dv_mv >= 0) {
							if ((full_vbatt1 - full_vbatt_n - chip->short_c_batt.dyna2_high_avg_dv_mv > chip->short_c_batt.dyna2_delta_dv_mv)
									&& (full_vbatt1 - full_vbatt_n > chip->short_c_batt.full_delta_vbatt2_mv)) {
								//set err_code5;
								short_debug("set err_code[%d]\n", SHORT_C_BATT_STATUS__DYNAMIC_ERR_CODE5);
								chip->short_c_batt.err_code = SHORT_C_BATT_STATUS__DYNAMIC_ERR_CODE5;
								oplus_short_c_batt_write_err_code(chip->short_c_batt.err_code, full_temp1,
									chip->temperature, full_vbatt1, full_vbatt_n, INVALID_CODE);
							}
						}
					}
					if (chip->short_c_batt.err_code != SHORT_C_BATT_STATUS__DYNAMIC_ERR_CODE5) {
						short_debug("exit_check, EXIT_CODE__TIMER4_OVERFLOW\n");
						oplus_short_c_batt_write_exit_code(EXIT_CODE__TIMER4_OVERFLOW,
							full_temp1, chip->temperature, full_vbatt1, full_vbatt_n);
					}
					//timer4 overflow, exit check
					exit_check_status = true;
					return;
				}
			} else {
				full_timer4_count = chip->short_c_batt.full_timer4;
			}
			return;
		}
		return;
	}

	/* pre CV check */
	if (chip->batt_volt > chip->short_c_batt.short_c_bat_cv_mv && chip->icharging < 0
			&& (-1 * chip->icharging) < CV_IBATT_MA_THRESHOLD) {
		cv_pre_check_count++;
		if (chip->short_c_batt.cv_satus == false) {
			short_debug("cv_pre_check_count[%d]\n", cv_pre_check_count);
		}
		if (cv_pre_check_count == CV_PRE_CHECK_COUNT) {
			if (chip->short_c_batt.cv_satus == false) {
				//reset exit code
				oplus_short_c_batt_write_exit_code(0, 0, 0, 0, 0);
				chip->short_c_batt.cv_satus = true;
				cv_temp = chip->temperature;
			}
		}
		if (cv_pre_check_count > CV_PRE_CHECK_COUNT) {
			cv_pre_check_count = CV_PRE_CHECK_COUNT + 1;
		}
	} else {
		if (chip->short_c_batt.cv_satus == false) {
			short_debug("short_c_batt.cv_satus == false\n");
			goto cv_reset;
		}
	}

	/* CV check */
	if (chip->short_c_batt.cv_satus == true) {
		cv_timer1_count++;
		short_debug("cv_timer1_count[%d]\n", cv_timer1_count);
		if (chip->batt_volt <= chip->short_c_batt.short_c_bat_cv_mv
				|| (chip->icharging < 0 && (-1 * chip->icharging) >= CV_IBATT_MA_THRESHOLD)
				|| abs(pre_icharging - chip->icharging) >= CV_DELTA_IBATT_MA_THRESHOLD) {
			cv_delta_ibatt_count++;
			if (cv_delta_ibatt_count == CV_DELTA_IBATT_COUNT) {
				goto cv_reset;
			}
		} else {
			if ((-1 * chip->icharging) < chip->limits.iterm_ma + 50) {
				cv_iterm_count++;
				if (cv_vbatt_count < CV_VBATT_COUNT) {
					short_debug("cv_iterm_count[%d], cv_avg_vbatt[%d]\n", cv_iterm_count, cv_avg_vbatt);
				}
				if (cv_iterm_count > CV_ITERM_COUNT) {
					cv_iterm_count = CV_ITERM_COUNT;
				}
			} else if (cv_iterm_count < CV_ITERM_COUNT) {
				cv_iterm_count = 0;
			}

			if (cv_iterm_count == CV_ITERM_COUNT && cv_vbatt_count < CV_VBATT_COUNT) {
#if 0
				cv_avg_vbatt = cv_avg_vbatt + chip->batt_volt;
				cv_vbatt_count++;
				if (cv_vbatt_count == CV_VBATT_COUNT)
					cv_avg_vbatt = cv_avg_vbatt / CV_VBATT_COUNT;
				short_debug("cv_vbatt_count[%d], cv_avg_vbatt[%d]\n", cv_vbatt_count, cv_avg_vbatt);
#else
				/* 2017-12-18 use the max value instead of avg value */
				cv_avg_vbatt = max(cv_avg_vbatt, chip->batt_volt);
				cv_vbatt_count++;
				short_debug("cv_vbatt_count[%d], max cv_avg_vbatt[%d]\n", cv_vbatt_count, cv_avg_vbatt);
#endif
			}
		}

		if (cv_timer1_count == chip->short_c_batt.cv_timer1) {
			//set err_code1
			short_debug("set err_code[%d]\n", SHORT_C_BATT_STATUS__CV_ERR_CODE1);
			chip->short_c_batt.err_code = SHORT_C_BATT_STATUS__CV_ERR_CODE1;
			oplus_short_c_batt_write_err_code(SHORT_C_BATT_STATUS__CV_ERR_CODE1,
				cv_temp, chip->temperature, INVALID_CODE, INVALID_CODE, INVALID_CODE);
			//turnoff charging
			if (chip->short_c_batt.is_switch_on) {
				chip->chg_ops->charging_disable();
			}
		}
	}

	pre_icharging = chip->icharging;
	return;

exit_check_reset:
	exit_check_status = false;

cv_reset:
	cv_pre_check_count = 0;
	chip->short_c_batt.cv_satus = false;
	cv_temp = INVALID_TEMP;
	cv_timer1_count = 0;
	cv_delta_ibatt_count = 0;
	cv_iterm_count = 0;
	cv_avg_vbatt = 0;
	cv_vbatt_count = 0;

//full_reset:
	full_over_10ma_count = 0;
	full_over_20ma_count = 0;
	full_over_50ma_count = 0;
	full_timer2_count = 0;
	full_timer3_count = 0;
	full_vbatt1 = 0;
	full_temp1 = INVALID_TEMP;
	full_timer4_count = 0;
	full_timer5_count = 0;

	pre_icharging = chip->icharging;

	oplus_short_c_batt_set_disable_rechg(chip, false);
	oplus_short_c_batt_get_average_vbatt_mv(chip, true);

	return;
}
#else /* CONFIG_OPLUS_SHORT_C_BATT_CHECK */
int oplus_short_c_batt_err_code_init(void)
{
	return SHORT_C_BATT_STATUS__NORMAL;
}

int oplus_short_c_batt_chg_switch_init(void)
{
	return SHORT_C_BATT_SW_STATUS__OFF;
}

int oplus_short_c_batt_feature_sw_status_init(void)
{
	return SHORT_C_BATT_FEATURE_SW_STATUS__ON;
}

int oplus_short_c_batt_feature_hw_status_init(void)
{
	return SHORT_C_BATT_FEATURE_HW_STATUS__ON;
}

bool oplus_short_c_batt_is_prohibit_chg(struct oplus_chg_chip *chip)
{
	return false;
}

bool oplus_short_c_batt_is_disable_rechg(struct oplus_chg_chip *chip)
{
	return false;
}

bool oplus_short_c_batt_get_cv_status(struct oplus_chg_chip *chip)
{
	return false;
}

void oplus_short_c_batt_update_change(struct oplus_chg_chip *chip, int update_value)
{
	return;
}

void oplus_chg_short_c_battery_check(struct oplus_chg_chip *chip)
{
	return;
}
#endif /* CONFIG_OPLUS_SHORT_C_BATT_CHECK */
