/************************************************************************************
** Copyright (C), 2008-2018, OPLUS Mobile Comm Corp., Ltd
** VENDOR_EDIT
** File: oplus_motor.c
**
** Description:
**	Definitions for motor control layer.
**
** Version: 1.0
** Date created: 2018/01/14,20:27
**
** --------------------------- Revision History: ------------------------------------
* <version>		<date>		<author>		<desc>
**************************************************************************************/
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>
#include <linux/proc_fs.h>
#include <linux/hrtimer.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/sysfs.h>
#ifdef CONFIG_DRM_MSM
#include <linux/msm_drm_notify.h>
#endif
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/pm_qos.h>
#include "oplus_motor.h"
#include "oplus_motor_notifier.h"

static struct oplus_motor_chip *g_chip = NULL;
static DEFINE_MUTEX(motor_running_mutex);
static DEFINE_MUTEX(position_detect_mutex);
static DECLARE_COMPLETION(shutdown_comp);

#ifdef CONFIG_MOTOR_DETECT_QOS
static struct pm_qos_request motor_qos_req;
static int motor_qos_value = PM_QOS_DEFAULT_VALUE;
static int motor_qos_state = 0;
#define PM_QOS_MOTOR_WAKEUP_VALUE 400
#endif

static void motor_run_work(struct work_struct *work);
static void oplus_motor_reset_check(struct oplus_motor_chip *chip);

static bool in_cali = false;

__attribute__((weak)) void oplus_parse_motor_info(struct oplus_motor_chip *chip)
{
	chip->info.type = MOTOR_FI5;
	chip->info.motor_ic = DRV8834;
	chip->dir_sign = POSITIVE;
	chip->is_support_ffd = false;
	chip->have_mode = true;
	chip->boot_mode = MOTOR_NORMAL_MODE;
}

static unsigned int support_speeds[] = {
	SPEED_100KHZ, /*High*/
	SPEED_80KHZ, /*High*/
	SPEED_64KHZ,
	SPEED_60_8KHZ,
	SPEED_57_6KHZ,
	SPEED_54_4KHZ,
	SPEED_51_2KHZ,
	SPEED_38_4KHZ,
	SPEED_32KHZ,
	SPEED_25_6KHZ,
	SPEED_22_4KHZ,
	SPEED_19_2KHZ,
	SPEED_16_6KHZ,
	SPEED_12_8KHZ,
	SPEED_9_6KHZ,
	SPEED_6_4KHZ,
	SPEED_3_2KHZ,
	SPEED_1_6KHZ, /*LOW*/
};

__attribute__((weak)) int check_charge_temp(int *temp)
{
	return -EINVAL;
}

static int oplus_motor_get_speed(int index, unsigned int *speed)
{
	if (index < sizeof(support_speeds) / sizeof(support_speeds[0])) {
		*speed = support_speeds[index];
		return 0;
	}

	return -EINVAL;
}

int oplus_register_dhall(const char *name, struct oplus_dhall_operations *ops)
{
	if (!g_chip) {
		struct oplus_motor_chip *chip = kzalloc(sizeof(struct oplus_motor_chip),
						       GFP_KERNEL);

		if (!chip) {
			MOTOR_ERR("kzalloc err \n");
			return -ENOMEM;
		}

		g_chip = chip;
	}

	if (!g_chip->dhall_ops) {
		if (ops) {
			g_chip->dhall_ops = ops;
			g_chip->info.d_name = name;
		} else {
			MOTOR_ERR("dhall_ops NULL \n");
			return -EINVAL;
		}

	} else {
		MOTOR_ERR("dhall_ops has been register \n");
		return -EINVAL;
	}

	return 0;
}

int oplus_unregister_dhall(const char *name)
{
	if (g_chip) {
		if (!strcmp(g_chip->info.d_name, name)) {
			g_chip->dhall_ops = NULL;
			g_chip->info.d_name = NULL;
			kfree(g_chip);
			g_chip = NULL;
			MOTOR_ERR("success \n");
		}
	}

	return 0;
}

int oplus_register_motor(const char *name, struct oplus_motor_operations *ops)
{
	if (!g_chip) {
		struct oplus_motor_chip *chip = kzalloc(sizeof(struct oplus_motor_chip),
						       GFP_KERNEL);

		if (!chip) {
			MOTOR_ERR("kzalloc err \n");
			return -ENOMEM;
		}

		g_chip = chip;
	}

	if (!g_chip->motor_ops) {
		if (ops) {
			g_chip->motor_ops = ops;
			g_chip->info.m_name = name;

		} else {
			MOTOR_ERR("motor_ops NULL \n");
			return -EINVAL;
		}

	} else {
		MOTOR_ERR("motor_ops has been register \n");
		return -EINVAL;
	}

	return 0;
}

int oplus_unregister_motor(const char *name)
{
	if (g_chip) {
		if (!strcmp(g_chip->info.m_name, name)) {
			g_chip->motor_ops = NULL;
			g_chip->info.m_name = NULL;
			kfree(g_chip);
			g_chip = NULL;
			MOTOR_ERR("success \n");
		}
	}

	return 0;
}

static int oplus_input_dev_init(struct oplus_motor_chip *chip)
{
	struct input_dev *dev;
	int err;

	dev = input_allocate_device();

	if (!dev) {
		MOTOR_ERR("input_dev null \n");
		return -ENOMEM;
	}

	dev->name = "motor";
	dev->id.bustype = BUS_I2C;

	set_bit(MOTOR_EVENT_TYPE, dev->evbit);
	set_bit(MOTOR_EVENT_MANUAL_TO_UP, dev->keybit);
	set_bit(MOTOR_EVENT_MANUAL_TO_DOWN, dev->keybit);
	set_bit(MOTOR_EVENT_UP, dev->keybit);
	set_bit(MOTOR_EVENT_UP_ABNORMAL, dev->keybit);
	set_bit(MOTOR_EVENT_UP_NORMAL, dev->keybit);
	set_bit(MOTOR_EVENT_DOWN, dev->keybit);
	set_bit(MOTOR_EVENT_DOWN_ABNORMAL, dev->keybit);
	set_bit(MOTOR_EVENT_DOWN_NORMAL, dev->keybit);

	err = input_register_device(dev);

	if (err < 0) {
		input_free_device(dev);
		return err;
	}

	chip->i_dev = dev;

	return 0;
}

static void report_positon_state_manual_to_up(struct oplus_motor_chip *chip)
{
	input_report_key(chip->i_dev, MOTOR_EVENT_MANUAL_TO_UP, 1);
	input_sync(chip->i_dev);
	input_report_key(chip->i_dev, MOTOR_EVENT_MANUAL_TO_UP, 0);
	input_sync(chip->i_dev);
}

static void report_positon_state_manual_to_down(struct oplus_motor_chip *chip)
{
	input_report_key(chip->i_dev, MOTOR_EVENT_MANUAL_TO_DOWN, 1);
	input_sync(chip->i_dev);
	input_report_key(chip->i_dev, MOTOR_EVENT_MANUAL_TO_DOWN, 0);
	input_sync(chip->i_dev);
}

static void report_positon_state_up(struct oplus_motor_chip *chip)
{
	input_report_key(chip->i_dev, MOTOR_EVENT_UP, 1);
	input_sync(chip->i_dev);
	input_report_key(chip->i_dev, MOTOR_EVENT_UP, 0);
	input_sync(chip->i_dev);
}

static void report_positon_state_up_abnormal(struct oplus_motor_chip *chip)
{
	input_report_key(chip->i_dev, MOTOR_EVENT_UP_ABNORMAL, 1);
	input_sync(chip->i_dev);
	input_report_key(chip->i_dev, MOTOR_EVENT_UP_ABNORMAL, 0);
	input_sync(chip->i_dev);
}

static void report_positon_state_up_normal(struct oplus_motor_chip *chip)
{
	input_report_key(chip->i_dev, MOTOR_EVENT_UP_NORMAL, 1);
	input_sync(chip->i_dev);
	input_report_key(chip->i_dev, MOTOR_EVENT_UP_NORMAL, 0);
	input_sync(chip->i_dev);
}

static void report_positon_state_down(struct oplus_motor_chip *chip)
{
	input_report_key(chip->i_dev, MOTOR_EVENT_DOWN, 1);
	input_sync(chip->i_dev);
	input_report_key(chip->i_dev, MOTOR_EVENT_DOWN, 0);
	input_sync(chip->i_dev);
}

static void report_positon_state_down_abnormal(struct oplus_motor_chip *chip)
{
	input_report_key(chip->i_dev, MOTOR_EVENT_DOWN_ABNORMAL, 1);
	input_sync(chip->i_dev);
	input_report_key(chip->i_dev, MOTOR_EVENT_DOWN_ABNORMAL, 0);
	input_sync(chip->i_dev);
}

static void report_positon_state_down_normal(struct oplus_motor_chip *chip)
{
	input_report_key(chip->i_dev, MOTOR_EVENT_DOWN_NORMAL, 1);
	input_sync(chip->i_dev);
	input_report_key(chip->i_dev, MOTOR_EVENT_DOWN_NORMAL, 0);
	input_sync(chip->i_dev);
}


/*should use in irq handle*/
int oplus_dhall_enable_irq(unsigned int id, bool enable)
{
	if (!g_chip || !g_chip->dhall_ops || !g_chip->dhall_ops->enable_irq) {
		return -EINVAL;

	} else {
		return g_chip->dhall_ops->enable_irq(id, enable);
	}
}

int oplus_dhall_clear_irq(unsigned int id)
{
	if (!g_chip || !g_chip->dhall_ops || !g_chip->dhall_ops->enable_irq) {
		return -EINVAL;

	} else {
		return g_chip->dhall_ops->clear_irq(id);
	}
}

int oplus_dhall_set_SRS(unsigned int id, char *value)
{
	if (!g_chip || !g_chip->dhall_ops || !g_chip->dhall_ops->set_sensitivity) {
		return -EINVAL;
	}

	if (id != DHALL_0 && id != DHALL_1) {
		return -EINVAL;
	}

	g_chip->dhall_ops->set_sensitivity(id, value);

	return 0;
}

int oplus_dhall_get_data(unsigned int id, short *hall_val, bool raw)
{
	if (!g_chip || !g_chip->dhall_ops || !g_chip->dhall_ops->get_data) {
		return -EINVAL;
	}

	if (id != DHALL_0 && id != DHALL_1) {
		return -EINVAL;
	}

	g_chip->dhall_ops->get_data(id, hall_val);

	if (!raw) {
		*hall_val = g_chip->sign * (*hall_val);
	}

	return 0;
}

static void oplus_dhall_clear_max_data(struct oplus_motor_chip *chip)
{
	chip->hall_max.data0 = -512;
	chip->hall_max.data1 = -512;
}

static void oplus_dhall_record_max_data(short hall0_val, short hall1_val)
{
	if (!g_chip) {
		return;
	}

	g_chip->hall_max.data0 = g_chip->hall_max.data0 > hall0_val ?
				 g_chip->hall_max.data0 : hall0_val;

	g_chip->hall_max.data1 = g_chip->hall_max.data1 > hall1_val ?
				 g_chip->hall_max.data1 : hall1_val;
}

bool oplus_dhall_update_threshold(unsigned int id, int position, short thresh)
{
	short thresh_low = 0;
	short thresh_high = 0;

	if (!g_chip || !g_chip->dhall_ops || !g_chip->dhall_ops->update_threshold) {
		return false;
	}

	/*means disable hall interupt detect*/
	if (thresh == HALL_THRESH_HIGH) {
		thresh_low = HALL_THRESH_HIGH;
		thresh_high = HALL_THRESH_HIGH;

	} else {
		if (g_chip->sign == POSITIVE) {
			thresh_low = HALL_THRESH_LOW;
			thresh_high = thresh;

		} else {
			thresh_low = thresh * g_chip->sign;
			thresh_high = HALL_THRESH_HIGH;
		}
	}

	return g_chip->dhall_ops->update_threshold(id, position, thresh_low,
			thresh_high);
}

int oplus_dhall_set_detection_mode(unsigned int id, u8 mode)
{
	if (!g_chip || !g_chip->dhall_ops || !g_chip->dhall_ops->set_detection_mode) {
		return -EINVAL;

	} else {
		return g_chip->dhall_ops->set_detection_mode(id, mode);
	}
}

int oplus_dhall_get_irq_state(unsigned int id)
{
	if (!g_chip || !g_chip->dhall_ops || !g_chip->dhall_ops->get_irq_state) {
		return -EINVAL;

	} else {
		return g_chip->dhall_ops->get_irq_state(id);
	}
}

void oplus_dhall_dump_regs(unsigned int id, u8 *buf)
{
	if (!g_chip || !g_chip->dhall_ops || !g_chip->dhall_ops->dump_regs) {
		return;

	} else {
		g_chip->dhall_ops->dump_regs(id, buf);
	}
}

int oplus_dhall_set_reg(unsigned int id, int reg, int val)
{
	if (!g_chip || !g_chip->dhall_ops || !g_chip->dhall_ops->set_reg) {
		return -EINVAL;

	} else {
		return g_chip->dhall_ops->set_reg(id, reg, val);
	}
}

bool oplus_dhall_is_power_on(void)
{
	if (!g_chip || !g_chip->dhall_ops || !g_chip->dhall_ops->is_power_on) {
		return false;

	} else {
		if (g_chip->dhall_ops->is_power_on(DHALL_0)
				|| g_chip->dhall_ops->is_power_on(DHALL_1)) {
			return true;

		} else {
			return false;
		}
	}
}

int oplus_motor_set_power(enum motor_state state, int mode)
{
	static enum motor_state pre_state = STATE_INVALID;

	if (!g_chip || !g_chip->motor_ops || !g_chip->motor_ops->set_power) {
		return -EINVAL;
	}

	mutex_lock(&g_chip->power_mutex);

	if (state != pre_state) {
		if (g_chip->motor_ops->pctrl_config) {
			/*config power, set gpio input/pull up/pull down*/
			if ((state == STAY_ACTIVE) || (state == WAKE_ACTIVE)) {
				g_chip->motor_ops->pctrl_config(true);

			} else {
				g_chip->motor_ops->pctrl_config(false); /*config sleep*/
			}
		}
	}

	/*means motor have enter deep suspend before, this should never happen*/
	if ((pre_state == DEEP_SLEEP) && (state == STAY_ACTIVE)) {
		MOTOR_LOG("Resume from motor force power on\n");
		mdelay(10);
	}

	if (mode != MOTOR_POWER_IGNORE) {
		g_chip->motor_ops->set_power(mode);
	}

	pre_state = state;

	mutex_unlock(&g_chip->power_mutex);

	return 0;
}

int oplus_motor_set_direction(int dir)
{
	if (!g_chip || !g_chip->motor_ops->set_direction) {
		return -EINVAL;

	} else {
		return g_chip->motor_ops->set_direction(dir);
	}
}

int oplus_motor_set_working_mode(int mode)
{
	if (!g_chip || !g_chip->motor_ops || !g_chip->motor_ops->set_working_mode) {
		return -EINVAL;

	} else {
		return g_chip->motor_ops->set_working_mode(mode);
	}
}

int oplus_motor_calculate_pwm_count(int L, int mode)
{
	if (!g_chip || !g_chip->motor_ops || !g_chip->motor_ops->calculate_pwm_count) {
		return 0;

	} else {
		return g_chip->motor_ops->calculate_pwm_count(L, mode);
	}
}

int oplus_motor_pwm_config(int duty_ns, int period_ns, uint32_t wave_num)
{
	if (!g_chip || !g_chip->motor_ops || !g_chip->motor_ops->pwm_config) {
		return -EINVAL;

	} else {
		return g_chip->motor_ops->pwm_config(duty_ns, period_ns, wave_num);
	}
}

int oplus_motor_pwm_enable(void)
{
	if (!g_chip || !g_chip->motor_ops || !g_chip->motor_ops->pwm_enable) {
		return -EINVAL;

	} else {
		return g_chip->motor_ops->pwm_enable();
	}
}

int oplus_motor_pwm_disable(void)
{
	if (!g_chip || !g_chip->motor_ops || !g_chip->motor_ops->pwm_disable) {
		return -EINVAL;

	} else {
		return g_chip->motor_ops->pwm_disable();
	}
}

int oplus_motor_get_all_config(int *config , int count)
{
	if (!g_chip || !g_chip->motor_ops || !g_chip->motor_ops->get_all_config) {
		return -EINVAL;

	} else {
		return g_chip->motor_ops->get_all_config(config, count);
	}
}

int oplus_get_motor_type(void)
{
	if (!g_chip) {
		return MOTOR_FI5;

	} else {
		if (g_chip->info.type == MOTOR_UNKNOWN) {
			oplus_parse_motor_info(g_chip);
		}

		return g_chip->info.type;
	}
}

bool oplus_is_support_mode(void)
{
	if (!g_chip) {
		return false;

	} else {
		if (g_chip->info.motor_ic == UNKNOWN) {
			oplus_parse_motor_info(g_chip);
		}

		return g_chip->have_mode;
	}
}

int oplus_get_driver_ic_id(void)
{
	if (!g_chip) {
		return DRV8834;

	} else {
		if (g_chip->info.motor_ic == UNKNOWN) {
			oplus_parse_motor_info(g_chip);
		}

		return g_chip->info.motor_ic;
	}
}

int oplus_get_dir_sign(void)
{
	if (!g_chip) {
		return POSITIVE;

	} else {
		return g_chip->dir_sign;
	}
}

static void oplus_change_dhall_sign(short hall0, short hall1)
{
	if (!g_chip) {
		MOTOR_LOG("g_chip null\n");
		return;
	}

	/*two halls sign should be the same*/
	if ((hall0 * hall1) > 0) {
		if (hall0 > 0) {
			g_chip->sign = POSITIVE;

		} else {
			g_chip->sign = NEGATIVE;
		}
	}

	MOTOR_LOG("sign %s\n", g_chip->sign == POSITIVE ? "POSITIVE" : "NEGATIVE");
}

static void oplus_motor_notify_state(unsigned long val)
{
	if (val > MOTOR_BLOCK_EVENT) {
		return;
	}

	motor_notifier_call_chain(val);
}

static void oplus_motor_awake_init(struct oplus_motor_chip *chip)
{
	if (!chip) {
		return;
	}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
	wake_lock_init(&chip->suspend_lock, WAKE_LOCK_SUSPEND, "motor wakelock");
#else
	chip->suspend_ws = wakeup_source_register(chip->dev, "motor wakelock");
#endif
}

static void oplus_motor_set_awake(struct oplus_motor_chip *chip, int id ,
				 bool awake)
{
	static int wakelock_holder = 0;

	if (id >= MAX_LOCK) {
		return;
	}

	if (awake) {
		if (!((wakelock_holder) & (1 << id)) && (!wakelock_holder)) {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
			wake_lock(&chip->suspend_lock);
#else
			__pm_stay_awake(chip->suspend_ws);
#endif
			MOTOR_LOG("wakelock hold\n");
		}

		wakelock_holder |= 1 << id;

	} else {
		wakelock_holder &= ~(1 << id);

		if (!wakelock_holder) {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
			wake_unlock(&chip->suspend_lock);
#else
			__pm_relax(chip->suspend_ws);
#endif
			MOTOR_LOG("wakelock release\n");
		}
	}
}

static bool check_normal_strong_mag(enum motor_direction dir, short hall0_val,
				    short hall1_val)
{
	if (!g_chip->noise.support) {
		return false;
	}

	MOTOR_LOG("hall0:%d, hall1:%d \n", hall0_val, hall1_val);

	/*condition with weak mag*/
	if ((hall0_val < g_chip->noise.hall0_weak
			&& hall0_val > -g_chip->noise.hall0_weak) &&
			(hall1_val < g_chip->noise.hall1_weak
			 && hall1_val > -g_chip->noise.hall1_weak)) {
		return false;
	}

	if (dir == MOTOR_UPWARD) {
		if (hall0_val < hall1_val ||
				hall0_val > g_chip->noise.normal_max ||
				hall0_val < g_chip->noise.normal_min) {
			return true;
		}
	}

	if (dir == MOTOR_DOWN) {
		if (hall1_val < hall0_val ||
				hall1_val > g_chip->noise.normal_max ||
				hall1_val < g_chip->noise.normal_min) {
			return true;
		}
	}

	if ((hall0_val > g_chip->noise.average || hall0_val < -g_chip->noise.average) &&
			(hall1_val > g_chip->noise.average || hall1_val < -g_chip->noise.average)) {
		return true;
	}

	return false;
}

static bool check_irq_strong_mag(enum motor_pull_push push_pull,
				 short hall0_val, short hall1_val, short pre_hall0, short pre_hall1)
{
	if (!g_chip->noise.support) {
		return false;
	}

	MOTOR_LOG("hall0:%d, hall1:%d pre_hall0:%d, pre_hall1:%d\n",
		  hall0_val, hall1_val, pre_hall0, pre_hall1);

	if (hall0_val > g_chip->noise.irq_max ||
			hall0_val < g_chip->noise.irq_min ||
			hall1_val > g_chip->noise.irq_max ||
			hall1_val < g_chip->noise.irq_min) {
		return true;
	}

	if (push_pull == MOTOR_PULL) {
		if (hall0_val > g_chip->cali_data.irq_pos_hall0 ||
				(hall0_val > pre_hall0) || (hall1_val + 1 < pre_hall1)) {
			return true;
		}
	}

	return false;
}

static void irq_pos_update(struct oplus_motor_chip *chip, unsigned int id,
			   bool makeup)
{
	if (!chip) {
		MOTOR_LOG("g_chip is null.\n");
		return;
	}

	if (id == DHALL_0) {
		if (makeup) {
			chip->cali_data.irq_pos_hall0 = chip->cali_data.irq_pos_hall0_bak +
							chip->cali_data.irq_makeup_hall0;

		} else {
			chip->cali_data.irq_pos_hall0 = chip->cali_data.irq_pos_hall0_bak;
		}

	} else if (id == DHALL_1) {
		if (makeup) {
			chip->cali_data.irq_pos_hall1 = chip->cali_data.irq_pos_hall1_bak +
							chip->cali_data.irq_makeup_hall1;

		} else {
			chip->cali_data.irq_pos_hall1 = chip->cali_data.irq_pos_hall1_bak;
		}
	}

	MOTOR_LOG("update irq_hall0:%d, irq_hall1:%d.\n", chip->cali_data.irq_pos_hall0,
		  chip->cali_data.irq_pos_hall1);
}

/*note:work in irq context*/
int oplus_dhall_irq_handler(unsigned int id)
{
	if (!g_chip) {
		MOTOR_LOG("g_chip null \n");
		return -EINVAL;
	}

	oplus_dhall_get_data(DHALL_0, &g_chip->noise.pre_hall0, false);
	oplus_dhall_get_data(DHALL_1, &g_chip->noise.pre_hall1, false);

	g_chip->irq_count[id]++;

	if (id == DHALL_0) {
		queue_delayed_work(g_chip->manual2auto_wq, &g_chip->up_work,
				   msecs_to_jiffies(50));

	} else if (id == DHALL_1) {
		queue_delayed_work(g_chip->manual2auto_wq, &g_chip->down_work, 0);
	}

	return 0;
}

static void oplus_set_md_mode_para(int md_mode)
{
	int mode = 0;

	if (!g_chip) {
		MOTOR_LOG("g_chip null \n ");
		return;
	}

	if ((md_mode < MOTOR_MODE_FULL) || (md_mode > MOTOR_MODE_1_32)) {
		return;
	}

	switch (md_mode) {
	case MOTOR_MODE_FULL:
		oplus_motor_set_working_mode(MOTOR_MODE_FULL);
		mode = 1;
		break;

	case MOTOR_MODE_1_16:
		oplus_motor_set_working_mode(MOTOR_MODE_1_16);
		mode = 16;
		break;

	case MOTOR_MODE_1_32:
		oplus_motor_set_working_mode(MOTOR_MODE_1_32);
		mode = 32;
		break;

	default:
		oplus_motor_set_working_mode(MOTOR_MODE_1_32);
		mode = 32;
		break;
	}

	g_chip->pwm_param.md_mode = md_mode;
	g_chip->pwm_param.normal.full.pwm = oplus_motor_calculate_pwm_count(
			g_chip->pwm_param.normal.full.l, mode);
	g_chip->pwm_param.normal.nd.pwm = oplus_motor_calculate_pwm_count(
			g_chip->pwm_param.normal.nd.l, mode);
	g_chip->pwm_param.normal.speed_up.pwm = oplus_motor_calculate_pwm_count(
			g_chip->pwm_param.normal.speed_up.l, mode);
	g_chip->pwm_param.normal.speed_down.pwm = oplus_motor_calculate_pwm_count(
				g_chip->pwm_param.normal.speed_down.l, mode);
	MOTOR_LOG("Normal: full[%d, %d], speed_up [%d %d], nd [%d %d], speed_down [%d %d], mode %d\n",
		  g_chip->pwm_param.normal.full.pwm,
		  g_chip->pwm_param.normal.full.l,
		  g_chip->pwm_param.normal.speed_up.pwm,
		  g_chip->pwm_param.normal.speed_up.l,
		  g_chip->pwm_param.normal.nd.pwm,
		  g_chip->pwm_param.normal.nd.l,
		  g_chip->pwm_param.normal.speed_down.pwm,
		  g_chip->pwm_param.normal.speed_down.l,
		  mode);

	g_chip->pwm_param.slow.full.pwm = oplus_motor_calculate_pwm_count(
			g_chip->pwm_param.slow.full.l, mode);
	g_chip->pwm_param.slow.nd.pwm = oplus_motor_calculate_pwm_count(
						g_chip->pwm_param.slow.nd.l, mode);
	g_chip->pwm_param.slow.speed_up.pwm = oplus_motor_calculate_pwm_count(
			g_chip->pwm_param.slow.speed_up.l, mode);
	g_chip->pwm_param.slow.speed_down.pwm = oplus_motor_calculate_pwm_count(
			g_chip->pwm_param.slow.speed_down.l, mode);
	MOTOR_LOG("Slow: full[%d, %d], speed_up [%d %d], nd [%d %d], speed_down [%d %d], mode %d\n",
		  g_chip->pwm_param.slow.full.pwm,
		  g_chip->pwm_param.slow.full.l,
		  g_chip->pwm_param.slow.speed_up.pwm,
		  g_chip->pwm_param.slow.speed_up.l,
		  g_chip->pwm_param.slow.nd.pwm,
		  g_chip->pwm_param.slow.nd.l,
		  g_chip->pwm_param.slow.speed_down.pwm,
		  g_chip->pwm_param.slow.speed_down.l,
		  mode);
}


static int oplus_update_speed_para(bool slow)
{
	if (slow) {
		g_chip->pwm_param.run.full.l = g_chip->pwm_param.slow.full.l;
		g_chip->pwm_param.run.full.speed = g_chip->pwm_param.slow.full.speed;
		g_chip->pwm_param.run.full.pwm = g_chip->pwm_param.slow.full.pwm;
		g_chip->pwm_param.run.speed_up.l = g_chip->pwm_param.slow.speed_up.l;
		g_chip->pwm_param.run.speed_up.speed = g_chip->pwm_param.slow.speed_up.speed;
		g_chip->pwm_param.run.speed_up.pwm = g_chip->pwm_param.slow.speed_up.pwm;
		g_chip->pwm_param.run.speed_down.l = g_chip->pwm_param.slow.speed_down.l;
		g_chip->pwm_param.run.speed_down.speed =
			g_chip->pwm_param.slow.speed_down.speed;
		g_chip->pwm_param.run.speed_down.pwm = g_chip->pwm_param.slow.speed_down.pwm;
		g_chip->pwm_param.run.nd.l = g_chip->pwm_param.slow.nd.l;
		g_chip->pwm_param.run.nd.speed = g_chip->pwm_param.slow.nd.speed;
		g_chip->pwm_param.run.nd.pwm = g_chip->pwm_param.slow.nd.pwm;
		g_chip->pwm_param.run.delay = g_chip->pwm_param.slow.delay;
		g_chip->pwm_param.run.up_brake_delay = g_chip->pwm_param.slow.up_brake_delay;
		g_chip->pwm_param.run.down_brake_delay =
			g_chip->pwm_param.slow.down_brake_delay;

	} else {
		g_chip->pwm_param.run.full.l = g_chip->pwm_param.normal.full.l;
		g_chip->pwm_param.run.full.speed = g_chip->pwm_param.normal.full.speed;
		g_chip->pwm_param.run.full.pwm = g_chip->pwm_param.normal.full.pwm;
		g_chip->pwm_param.run.speed_up.l = g_chip->pwm_param.normal.speed_up.l;
		g_chip->pwm_param.run.speed_up.speed = g_chip->pwm_param.normal.speed_up.speed;
		g_chip->pwm_param.run.speed_up.pwm = g_chip->pwm_param.normal.speed_up.pwm;
		g_chip->pwm_param.run.speed_down.l = g_chip->pwm_param.normal.speed_down.l;
		g_chip->pwm_param.run.speed_down.speed =
			g_chip->pwm_param.normal.speed_down.speed;
		g_chip->pwm_param.run.speed_down.pwm = g_chip->pwm_param.normal.speed_down.pwm;
		g_chip->pwm_param.run.nd.l = g_chip->pwm_param.normal.nd.l;
		g_chip->pwm_param.run.nd.speed = g_chip->pwm_param.normal.nd.speed;
		g_chip->pwm_param.run.nd.pwm = g_chip->pwm_param.normal.nd.pwm;
		g_chip->pwm_param.run.delay = g_chip->pwm_param.normal.delay;
		g_chip->pwm_param.run.up_brake_delay = g_chip->pwm_param.normal.up_brake_delay;
		g_chip->pwm_param.run.down_brake_delay =
			g_chip->pwm_param.normal.down_brake_delay;
	}

	return 0;
}

static int oplus_update_motor_speed(unsigned int speed_ns)
{
	static unsigned int pre_speed = 0;

	if (pre_speed == speed_ns) {
		return -EINVAL;
	}

	g_chip->pwm_param.pwm_duty = speed_ns / 2;
	g_chip->pwm_param.pwm_period = speed_ns;

	pre_speed = speed_ns;

	return 0;
}

static void oplus_set_direction_para(int direction)
{
	if (!g_chip) {
		MOTOR_LOG("g_chip null \n ");
		return;
	}

	if (direction >= 0) {
		g_chip->md_dir = !!direction;
	}
}

static void oplus_set_motor_move_state(int move_state)
{
	if (!g_chip) {
		MOTOR_LOG("g_chip null \n ");
		return;
	}

	if ((move_state >= MOTOR_STOP) && (move_state <= MOTOR_DOWNWARD_STOP)) {
		g_chip->move_state = move_state;
	}
}

static void oplus_parameter_init(struct oplus_motor_chip *chip)
{
	atomic_set(&chip->in_suspend, 0);
	chip->position = MID_STATE;
	chip->move_state = MOTOR_STOP;
	chip->motor_switch = 1;/*switch open defaut*/
	chip->hall_detect_switch = true;
	chip->manual2auto_down_switch = 1;
	chip->manual2auto_up_switch = 0;
	chip->is_motor_test = 0;
	chip->is_skip_pos_check = 0;
	chip->irq_monitor_started = false;
	chip->is_irq_abnormal = false;
	chip->is_in_calibration = false;
	chip->info.type = MOTOR_UNKNOWN;
	chip->info.motor_ic = UNKNOWN;
	chip->is_support_ffd = false;

	oplus_dhall_clear_max_data(chip);
	oplus_parse_motor_info(chip);

	if (chip->boot_mode != MOTOR_NORMAL_MODE) {
		MOTOR_ERR("no normal mode, skip parameter init.\n");
		return;
	}

	oplus_dhall_update_threshold(DHALL_0, DOWN_STATE, HALL_THRESH_HIGH);
	oplus_dhall_update_threshold(DHALL_1, DOWN_STATE, HALL_THRESH_HIGH);
	oplus_dhall_set_detection_mode(DHALL_0, DETECTION_MODE_INTERRUPT);
	oplus_dhall_set_detection_mode(DHALL_1, DETECTION_MODE_INTERRUPT);

	/* initailize pwm*/
	oplus_motor_pwm_disable();
	oplus_update_motor_speed(chip->pwm_param.normal.speed_up.speed);
	oplus_motor_pwm_config(g_chip->pwm_param.pwm_duty, g_chip->pwm_param.pwm_period,
			      0);

	oplus_set_md_mode_para(MOTOR_MODE_1_32);

	oplus_set_direction_para(MOTOR_UPWARD);
	oplus_motor_awake_init(chip);
}

static int oplus_motor_run_check(struct oplus_motor_chip *chip,
				enum start_mode mode)
{
	short hall0_val = 0, hall1_val = 0;

	chip->strong_mag = false;

	if (chip->is_skip_pos_check) {
		MOTOR_ERR("skip_pos_check \n");
		return 1;
	}

	if ((chip->position == UP_STATE) && (chip->md_dir == MOTOR_UPWARD)) {
		MOTOR_LOG("has been in up_state, return false\n");
		return 0;

	} else if ((chip->position == DOWN_STATE) && (chip->md_dir == MOTOR_DOWN)) {
		MOTOR_LOG("has been in down_state, return false\n");
		return 0;
	}

	oplus_dhall_get_data(DHALL_0, &hall0_val, false);
	oplus_dhall_get_data(DHALL_1, &hall1_val, false);

	if (chip->md_dir == MOTOR_DOWN) {
		chip->run_check_hall1 = hall1_val;

	} else if (chip->md_dir == MOTOR_UPWARD) {
		chip->run_check_hall0 = hall0_val;
	}

	if ((mode == USER_START)
			&& check_normal_strong_mag(chip->md_dir, hall0_val, hall1_val)) {
		chip->strong_mag = true;
		MOTOR_LOG("strong mag detect\n");
		return 1;
	}

	return 1;
}

static void oplus_motor_control(enum start_mode mode, int dir)
{
	int ret = 0;

	if (!g_chip) {
		MOTOR_LOG("g_chip null \n ");
		return;
	}

	if (mode) {
		if (g_chip->motor_switch == 0) {
			return;
		}

		if (!g_chip->motor_started) {
			g_chip->motor_enable = 1;
			oplus_set_direction_para(dir);
			ret = oplus_motor_run_check(g_chip, mode);

			if (!ret) {
				g_chip->motor_enable = 0;

			} else {
				schedule_work(&g_chip->motor_work);
			}
		}

	} else {
		if (g_chip->motor_started) {
			g_chip->motor_enable = 0;
			schedule_work(&g_chip->motor_work);
		}
	}
}

static void oplus_motor_start(enum start_mode mode, int dir)
{
	bool slow = false;
	static int retry = 0;

	/*if stuck, update speed para*/
	if (mode == ABNORMAL_START) {
		retry++;

		if (retry >= RETRY_TIMES) {
			slow = true;
		}

	} else {
		retry = 0;
	}

	if (mode == IRQ_START) {
		irq_pos_update(g_chip, DHALL_0, true);

	} else {
		irq_pos_update(g_chip, DHALL_0, false);
	}

	oplus_update_speed_para(slow);

	oplus_motor_control(mode, dir);
}

static void oplus_motor_stop(void)
{
	oplus_motor_control(STOP, 0);
}

static void oplus_motor_change_speed(L_param_t *speed_param)
{
	unsigned int speed = 0;

	if (!g_chip) {
		MOTOR_LOG("g_chip null \n ");
		return;
	}

	if (!speed_param->l) {
		MOTOR_ERR("no need to change speed\n");
		return;
	}

	speed = speed_param->speed;

	if (oplus_update_motor_speed(speed)) {
		MOTOR_LOG("speed not change \n");
		return;
	}

	if (g_chip->motor_started) {
		oplus_motor_pwm_disable();
		oplus_motor_pwm_config(g_chip->pwm_param.pwm_duty, g_chip->pwm_param.pwm_period,
				      0);
		oplus_motor_pwm_enable();
	}
}

static void oplus_motor_upward(void)
{
	if (!g_chip) {
		MOTOR_LOG("g_chip null \n ");
		return;
	}

	oplus_motor_start(USER_START, MOTOR_UPWARD);
}

void oplus_motor_downward(enum start_mode mode)
{
	if (!g_chip) {
		MOTOR_LOG("g_chip null \n ");
		return;
	}

	oplus_motor_start(mode, MOTOR_DOWN);
}

static irqreturn_t oplus_free_fall_detect_handler(int irq, void *dev_id)
{
	if (!g_chip) {
		MOTOR_LOG("g_chip NULL \n");
		return -EINVAL;
	}

	MOTOR_LOG("call \n");

	disable_irq_nosync(g_chip->irq);
	oplus_motor_downward(USER_START);
	enable_irq(g_chip->irq);

	return IRQ_HANDLED;
}

#define MAG_NOISE_PATTERN 7
static void oplus_motor_parse_dts(struct oplus_motor_chip *chip)
{
	struct device_node *node = NULL;
	int rc = 0;
	int value;
	int pattern[MAG_NOISE_PATTERN] = {0};
	int irq_makeup[2] = {0};
	int speed_pat[2] = {0};
	int delay_pat[3] = {0};
	int offset[2] = {0};
	int stop_range[2] = {0};
	int mono_offset = 0;
	int change_thrd[2] = {0};
	int stop_retry = 0;
	int offset_stop_retry = 0;
	int stop_val[2] = {0};
	int cal_stop_val[2] = {0};
	int small_irq_tmp[3] = {0};

	node = chip->dev->of_node;

	chip->pctrl = devm_pinctrl_get(chip->dev);

	if (IS_ERR_OR_NULL(chip->pctrl)) {
		MOTOR_ERR("failed to get pinctrl\n");
		chip->pctrl = NULL;
	}

	MOTOR_LOG("start \n");

	rc = of_property_read_u32(node, "hall-data-sign", &value);

	if (rc) {
		chip->sign = POSITIVE;

	} else {
		if (!value) {
			chip->sign = NEGATIVE;

		} else {
			chip->sign = POSITIVE;
		}
	}

	rc = of_property_read_u32(node, "hall-irq-pos", &value);

	if (rc) {
		chip->cali_data.irq_pos_hall0 = DHALL_DETECT_RANGE_HIGH;
		chip->cali_data.irq_pos_hall1 = DHALL_DETECT_RANGE_HIGH;
		chip->cali_data.irq_pos_hall0_bak = DHALL_DETECT_RANGE_HIGH;
		chip->cali_data.irq_pos_hall1_bak = DHALL_DETECT_RANGE_HIGH;

	} else {
		chip->cali_data.irq_pos_hall0 = value;
		chip->cali_data.irq_pos_hall1 = value;
		chip->cali_data.irq_pos_hall0_bak = value;
		chip->cali_data.irq_pos_hall1_bak = value;
	}

	rc = of_property_read_u32_array(node, "press-irq-makeup", irq_makeup, 2);

	if (rc) {
		chip->cali_data.irq_makeup_hall1 = 0;
		chip->cali_data.irq_makeup_hall0 = 0;

	} else {
		chip->cali_data.irq_makeup_hall1 = irq_makeup[1];
		chip->cali_data.irq_makeup_hall0 = irq_makeup[0];
	}

	rc = of_property_read_u32(node, "motor-retard-pos", &value);

	if (rc) {
		chip->cali_data.up_retard_hall1 = MOTOR_STOP_RETARD_VALUE;
		chip->cali_data.down_retard_hall0 = MOTOR_STOP_RETARD_VALUE;

	} else {
		chip->cali_data.up_retard_hall1 = value;
		chip->cali_data.down_retard_hall0 = value;
	}

	chip->cali_data.up_retard_hall0 = 0;
	chip->cali_data.down_retard_hall1 = 0;

	rc = of_property_read_u32(node, "monotonicity-offset", &mono_offset);

	if (rc) {
		chip->mono_offset = 0;

	} else {
		chip->mono_offset = mono_offset;
	}

	rc = of_property_read_u32_array(node, "delay-cycle",
					delay_pat, 3);

	if (rc) {
		chip->pwm_param.normal.delay = 15000;
		chip->pwm_param.normal.up_brake_delay = 15000;
		chip->pwm_param.normal.down_brake_delay = 15000;

	} else {
		chip->pwm_param.normal.delay = delay_pat[0] * 1000;
		chip->pwm_param.normal.up_brake_delay = delay_pat[1] * 1000;
		chip->pwm_param.normal.down_brake_delay = delay_pat[2] * 1000;
	}

	rc = of_property_read_u32_array(node, "slow-delay-cycle",
					delay_pat, 2);

	if (rc) {
		chip->pwm_param.slow.delay = 30000;
		chip->pwm_param.slow.up_brake_delay = 30000;
		chip->pwm_param.slow.down_brake_delay = 30000;

	} else {
		chip->pwm_param.slow.delay = delay_pat[0] * 1000;
		chip->pwm_param.slow.up_brake_delay = delay_pat[1] * 1000;
		chip->pwm_param.slow.down_brake_delay = delay_pat[2] * 1000;
	}

	rc = of_property_read_u32_array(node, "full-speed-param",
					speed_pat, 2);

	if (!rc) {
		chip->pwm_param.normal.full.l = speed_pat[0];
		chip->pwm_param.normal.full.speed = speed_pat[1];

	} else {
		chip->pwm_param.normal.full.l = 87;
		chip->pwm_param.normal.full.speed = SPEED_80KHZ;
	}

	rc = of_property_read_u32_array(node, "up-speed-param",
					speed_pat, 2);

	if (!rc) {
		chip->pwm_param.normal.speed_up.l = speed_pat[0];
		chip->pwm_param.normal.speed_up.speed = speed_pat[1];

	} else {
		chip->pwm_param.normal.speed_up.l = 5;
		chip->pwm_param.normal.speed_up.speed = SPEED_32KHZ;
	}

	rc = of_property_read_u32_array(node, "down-speed-param",
					speed_pat, 2);

	if (!rc) {
		chip->pwm_param.normal.speed_down.l = speed_pat[0];
		chip->pwm_param.normal.speed_down.speed = speed_pat[1];

	} else {
		chip->pwm_param.normal.speed_down.l = 2;
		chip->pwm_param.normal.speed_down.speed = SPEED_16_6KHZ;
	}

	rc = of_property_read_u32_array(node, "nodetect-speed-param",
					speed_pat, 2);

	if (!rc) {
		chip->pwm_param.normal.nd.l = speed_pat[0];
		chip->pwm_param.normal.nd.speed = speed_pat[1];

	} else {
		chip->pwm_param.normal.nd.l = 0;
		chip->pwm_param.normal.nd.speed = SPEED_80KHZ;
	}

	rc = of_property_read_u32_array(node, "slow-full-speed-param",
					speed_pat, 2);

	if (!rc) {
		chip->pwm_param.slow.full.l = speed_pat[0];
		chip->pwm_param.slow.full.speed = speed_pat[1];

	} else {
		chip->pwm_param.slow.full.l = 87;
		chip->pwm_param.slow.full.speed = SPEED_32KHZ;
	}

	rc = of_property_read_u32_array(node, "slow-up-speed-param",
					speed_pat, 2);

	if (!rc) {
		chip->pwm_param.slow.speed_up.l = speed_pat[0];
		chip->pwm_param.slow.speed_up.speed = speed_pat[1];

	} else {
		chip->pwm_param.slow.speed_up.l = 5;
		chip->pwm_param.slow.speed_up.speed = SPEED_9_6KHZ;
	}

	rc = of_property_read_u32_array(node, "slow-down-speed-param",
					speed_pat, 2);

	if (!rc) {
		chip->pwm_param.slow.speed_down.l = speed_pat[0];
		chip->pwm_param.slow.speed_down.speed = speed_pat[1];

	} else {
		chip->pwm_param.slow.speed_down.l = 2;
		chip->pwm_param.slow.speed_down.speed = SPEED_16_6KHZ;
	}

	rc = of_property_read_u32_array(node, "slow-nodetect-speed-param",
					speed_pat, 2);

	if (!rc) {
		chip->pwm_param.slow.nd.l = speed_pat[0];
		chip->pwm_param.slow.nd.speed = speed_pat[1];

	} else {
		chip->pwm_param.slow.nd.l = 0;
		chip->pwm_param.slow.nd.speed = SPEED_80KHZ;
	}

	rc = of_property_read_u32_array(node, "stop-neg-point", stop_range, 2);

	if (rc) {
		chip->stop.neg[0] = MOTOR_STOP_DEFAULT_NEG_VALUE;
		chip->stop.neg[1] = MOTOR_STOP_DEFAULT_NEG_VALUE;

	} else {
		chip->stop.neg[0] = -stop_range[0];
		chip->stop.neg[1] = -stop_range[1];
	}

	rc = of_property_read_u32_array(node, "stop-pos-point", stop_range, 2);

	if (rc) {
		chip->stop.pos[0] = MOTOR_STOP_DEFAULT_POS_VALUE;
		chip->stop.pos[1] = MOTOR_STOP_DEFAULT_POS_VALUE;

	} else {
		chip->stop.pos[0] = stop_range[0];
		chip->stop.pos[1] = stop_range[1];
	}

	rc = of_property_read_u32_array(node, "terminal-offset", offset, 2);

	if (rc) {
		chip->stop.offset[0] = 0;
		chip->stop.offset[1] = 0;

	} else {
		chip->stop.offset[0] = offset[0];
		chip->stop.offset[1] = offset[1];
	}

	rc = of_property_read_u32_array(node, "pos-change-thrd", change_thrd, 2);

	if (rc) {
		chip->stop.pos_change_thrd[0] = 0;
		chip->stop.pos_change_thrd[1] = 0;

	} else {
		chip->stop.pos_change_thrd[0] = change_thrd[0];
		chip->stop.pos_change_thrd[1] = change_thrd[1];
	}

	memset(&offset, 0, sizeof(offset));
	rc = of_property_read_u32_array(node, "cali-retard-offset", offset, 2);

	if (rc) {
		chip->stop.cali_retard_offset[0] = 0;
		chip->stop.cali_retard_offset[1] = 0;

	} else {
		chip->stop.cali_retard_offset[0] = offset[0];
		chip->stop.cali_retard_offset[1] = offset[1];
	}

	memset(&offset, 0, sizeof(offset));
	rc = of_property_read_u32_array(node, "hall-max-thrd", offset, 2);

	if (rc) {
		chip->stop.hall_max_thrd[0] = 999;
		chip->stop.hall_max_thrd[1] = 999;

	} else {
		chip->stop.hall_max_thrd[0] = offset[0];
		chip->stop.hall_max_thrd[1] = offset[1];
	}

	rc = of_property_read_u32(node, "stop-retry", &stop_retry);

	if (rc) {
		chip->stop_retry = 2;

	} else {
		chip->stop_retry = stop_retry;
	}

	rc = of_property_read_u32(node, "offset-stop-retry", &offset_stop_retry);

	if (rc) {
		chip->offset_stop_retry = 1;

	} else {
		chip->offset_stop_retry = offset_stop_retry;
	}

	chip->stop_in_advance = of_property_read_bool(node, "stop_in_advance");
	chip->motor_retard_offset_support = of_property_read_bool(node,
					    "motor_retard_offset_support");
	chip->extre_stop_support = of_property_read_bool(node, "extre_stop_support");
	chip->motor_stop_val_support = of_property_read_bool(node,
				       "motor_stop_val_support");

	if (chip->motor_stop_val_support) {
		rc = of_property_read_u32_array(node, "motor_stop_val", stop_val, 2);

		if (rc) {
			chip->stop_rang_a = 3;
			chip->stop_rang_b = 2;

		} else {
			chip->stop_rang_a = stop_val[0];
			chip->stop_rang_b = stop_val[1];
		}
	}

	chip->motor_small_irq_later_speed_down_support = of_property_read_bool(node,
			"motor_small_irq_later_speed_down_support");

	rc = of_property_read_u32_array(node, "small-irq-later-speed-down",
					small_irq_tmp, 3);

	if (rc) {
		chip->small_irq = 0;
		chip->later_speed_down = 0;
		chip->later_speed_down_stop_offset = chip->stop.offset[1];

	} else {
		chip->small_irq = small_irq_tmp[0];
		chip->later_speed_down = small_irq_tmp[1];
		chip->later_speed_down_stop_offset = small_irq_tmp[2];
	}

	chip->cal_fix_stop_retry_support = of_property_read_bool(node,
					   "cal_fix_stop_retry_support");
	rc = of_property_read_u32(node, "cal_fix_stop_retry", &value);

	if (rc) {
		chip->cal_fix_stop_retry = 3;

	} else {
		chip->cal_fix_stop_retry = value;
	}

	rc = of_property_read_u32_array(node, "cal_stop_val", cal_stop_val, 2);

	if (rc) {
		chip->cal_stop_pos_val = 2;
		chip->cal_stop_neg_val = -2;

	} else {
		chip->cal_stop_pos_val = cal_stop_val[0];
		chip->cal_stop_neg_val = -cal_stop_val[1];
	}

	rc = of_property_read_u32_array(node, "full_stop_val", cal_stop_val, 2);

	if (rc) {
		chip->full_stop_pos_val = 3;
		chip->full_stop_neg_val = -20;

	} else {
		chip->full_stop_pos_val = cal_stop_val[0];
		chip->full_stop_neg_val = -cal_stop_val[1];
	}

	chip->motor_down_stop_support = of_property_read_bool(node,
					"motor_down_stop_support");
	rc = of_property_read_u32(node, "motor_down_stop_val", &value);

	if (rc) {
		chip->motor_down_stop_val = 0;

	} else {
		chip->motor_down_stop_val = value;
	}

	rc = of_property_read_u32(node, "retard-range-end", &value);

	if (rc) {
		chip->retard_range_end = 0;

	} else {
		chip->retard_range_end = value;
	}

	chip->noise.support = of_property_read_bool(node, "mag_noise_support");

	if (chip->noise.support) {
		rc = of_property_read_u32_array(node, "mag_noise_pattern",
						pattern, MAG_NOISE_PATTERN);

		if (rc < 0) {
			chip->noise.support = false;

		} else {
			chip->noise.irq_max = HALL_THRESH_HIGH - pattern[0];
			chip->noise.irq_min = HALL_THRESH_LOW + pattern[1];
			chip->noise.normal_max = HALL_THRESH_HIGH - pattern[2];
			chip->noise.normal_min = HALL_THRESH_LOW + pattern[3];
			chip->noise.average = pattern[4];
			chip->noise.hall0_weak = pattern[5];
			chip->noise.hall1_weak = pattern[6];
		}
	}

	chip->down_state_high_thresh_support = of_property_read_bool(node,
					       "down_state_high_thresh_support");

	MOTOR_LOG("support %d, irq [%d, %d], normal [%d, %d], aver %d, weak [%d, %d]\n",
		  chip->noise.support,
		  chip->noise.irq_max,
		  chip->noise.irq_min,
		  chip->noise.normal_max,
		  chip->noise.normal_min,
		  chip->noise.average,
		  chip->noise.hall0_weak,
		  chip->noise.hall1_weak);
}

static void oplus_motor_free_fall_register(struct oplus_motor_chip *chip)
{
	struct device_node *np = NULL;
	int rc = 0;
	int ret = 0;
	np = chip->dev->of_node;

	if (IS_ERR_OR_NULL(chip->pctrl)) {
		MOTOR_ERR("failed to get pinctrl\n");
		return;
	}

	chip->free_fall_state = pinctrl_lookup_state(chip->pctrl, "free_fall_input");

	if (IS_ERR_OR_NULL(chip->free_fall_state)) {
		rc = PTR_ERR(chip->free_fall_state);
		MOTOR_ERR("pinctrl_lookup_state, err:%d\n", rc);
		return;
	}

	pinctrl_select_state(chip->pctrl, chip->free_fall_state);

	chip->free_fall_gpio = of_get_named_gpio(np, "motor,irq-gpio", 0);

	if (!gpio_is_valid(chip->free_fall_gpio)) {
		MOTOR_LOG("qcom,hall-power-gpio gpio not specified\n");

	} else {
		rc = gpio_request(chip->free_fall_gpio, "motor-irq-gpio");

		if (rc) {
			MOTOR_LOG("request free_fall_gpio gpio failed, rc=%d\n", rc);
		}

		rc = gpio_direction_input(chip->free_fall_gpio);
		msleep(50);
		chip->irq = gpio_to_irq(chip->free_fall_gpio);

		ret = request_threaded_irq(chip->irq, NULL, &oplus_free_fall_detect_handler,
					   IRQ_TYPE_EDGE_RISING | IRQF_ONESHOT, "free_fall", NULL);

		if (ret < 0) {
			MOTOR_ERR("IRQ LINE NOT AVAILABLE ret %d!!\n", ret);
			return;
		}

		irq_set_irq_wake(chip->irq, 1);
	}

	MOTOR_ERR("GPIO %d irq:%d \n", chip->free_fall_gpio, chip->irq);
}

static enum hrtimer_restart motor_stop_timer_func(struct hrtimer *hrtimer)
{
	if (!g_chip) {
		MOTOR_LOG("g_chip null \n ");
		return HRTIMER_NORESTART;
	}

	MOTOR_LOG("call \n");
	g_chip->stop_timer_trigger = 1;

	oplus_motor_stop();
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart motor_speed_up_timer_func(struct hrtimer *hrtimer)
{
	if (!g_chip) {
		MOTOR_LOG("g_chip null \n ");
		return HRTIMER_NORESTART;
	}

	MOTOR_LOG("call \n");

	mod_delayed_work(system_highpri_wq, &g_chip->detect_work, 0);

	return HRTIMER_NORESTART;
}

static enum alarmtimer_restart motor_reset_timer_func(struct alarm *alrm,
		ktime_t now)
{
	if (!g_chip) {
		MOTOR_LOG("g_chip null \n ");
		return ALARMTIMER_NORESTART;
	}

	MOTOR_LOG("irq_count [%d %d] \n", g_chip->irq_count[DHALL_0],
		  g_chip->irq_count[DHALL_1]);

	if ((g_chip->irq_count[DHALL_0] >= MOTOR_IRQ_MONITOR_COUNT) ||
			(g_chip->irq_count[DHALL_1] >= MOTOR_IRQ_MONITOR_COUNT)) {
		MOTOR_LOG("irq abnormal,reset \n");
		g_chip->is_irq_abnormal = true;
		g_chip->motor_switch = 0;
		g_chip->irq_count[DHALL_0] = 0;
		g_chip->irq_count[DHALL_1] = 0;
	}

	g_chip->irq_monitor_started = false;

	return ALARMTIMER_NORESTART;
}

static void oplus_motor_irq_monitor(struct oplus_motor_chip *chip)
{
	if (!chip->irq_monitor_started) {
		MOTOR_LOG("start \n");
		chip->irq_monitor_started = true;
		alarm_start_relative(&chip->reset_timer,
				     ktime_set(MOTOR_IRQ_MONITOR_TIME / 1000,
					       (MOTOR_IRQ_MONITOR_TIME % 1000) * 1000000));
	}
}

#ifdef CONFIG_DRM_MSM
static int fb_notifier_callback(struct notifier_block *nb, unsigned long event,
				void *data)
{
	int blank;
	struct msm_drm_notifier *evdata = data;

	if (!g_chip) {
		return 0;
	}

	if (!evdata || (evdata->id != 0)) {
		return 0;
	}

	if (event == MSM_DRM_EARLY_EVENT_BLANK) {
		blank = *(int *)(evdata->data);

		if (blank == MSM_DRM_BLANK_UNBLANK) {
			MOTOR_LOG("blank %d event %ld\n", blank, event);
			oplus_motor_set_power(WAKE_ACTIVE, MOTOR_POWER_IGNORE);
		}
	}

	return 0;
}
#else
static int fb_notifier_callback(struct notifier_block *nb, unsigned long event,
				void *data)
{
	int blank;
	struct fb_event *evdata = data;

	if (!g_chip) {
		return 0;
	}

	if (evdata && evdata->data) {
		if (event == FB_EARLY_EVENT_BLANK) {
			blank = *(int *)evdata->data;

			if (blank == FB_BLANK_UNBLANK) {
				MOTOR_LOG("blank %d event %ld\n", blank, event);
				oplus_motor_set_power(WAKE_ACTIVE, MOTOR_POWER_IGNORE);
			}
		}
	}

	return 0;
}
#endif /*CONFIG_DRM_MSM*/

static ktime_t motor_cal_timer(int pwm, int speed)
{
	long long value = 0;
	unsigned long intsecond = 0;
	unsigned long nsecond = 0;

	value = (long long)pwm * (long long)speed;
	nsecond = do_div(value, 1000000000);
	intsecond = (unsigned long)value;

	MOTOR_LOG("timer [%lu s ,%lu us]\n", intsecond, nsecond / 1000);
	return ktime_set(intsecond, nsecond);
}

static ktime_t motor_cal_full_timer(bool mag)
{
	long long value = 0;
	unsigned long intsecond = 0;
	unsigned long nsecond = 0;

	if (!g_chip) {
		return ktime_set(0, 0);
	}

	if (g_chip->pwm_param.run.speed_up.l) {
		value += (long long)g_chip->pwm_param.run.speed_up.pwm *
			 (long long)g_chip->pwm_param.run.speed_up.speed;
	}

	if (g_chip->pwm_param.run.nd.l) {
		value += (long long)g_chip->pwm_param.run.nd.pwm * (long long)
			 g_chip->pwm_param.run.nd.speed;
	}

	if (g_chip->pwm_param.run.full.l) {
		value += (long long)g_chip->pwm_param.run.full.pwm * (long long)
			 g_chip->pwm_param.run.full.speed;
	}

	if (g_chip->pwm_param.run.speed_down.l) {
		value += (long long)g_chip->pwm_param.run.speed_down.pwm *
			 (long long)g_chip->pwm_param.run.speed_down.speed;
	}

	if (mag) {
		value = value * 11 / 10;

	} else {
		value += 800000000;
	}

	nsecond = do_div(value, 1000000000);
	intsecond = (unsigned long)value;

	MOTOR_LOG("timer [%lu s ,%lu us]\n", intsecond, nsecond / 1000);
	return ktime_set(intsecond, nsecond);
}

static void motor_run_work(struct work_struct *work)
{
	struct oplus_motor_chip *chip = container_of(work, struct oplus_motor_chip,
				       motor_work);
	short hall0_val = 0, hall1_val = 0;
	unsigned int speed;
	int pwm;
	int ret = 0;

	mutex_lock(&motor_running_mutex);

	if (chip->motor_enable && (chip->motor_started == 0)) {
		MOTOR_ERR("start motor, dir %d\n", chip->md_dir);

		chip->motor_started = 1;
		oplus_motor_set_awake(chip, MOTOR_RUN_LOCK, true);
		oplus_motor_set_power(STAY_ACTIVE, MOTOR_POWER_ON);
		oplus_motor_set_direction(chip->md_dir);

		/*disable hall interrupt detect*/
		if (!g_chip->is_motor_test) {
			if (chip->md_dir == MOTOR_UPWARD) {
				oplus_dhall_update_threshold(DHALL_0, DOWN_STATE, HALL_THRESH_HIGH);

			} else if (chip->md_dir == MOTOR_DOWN) {
				oplus_dhall_update_threshold(DHALL_1, DOWN_STATE, HALL_THRESH_HIGH);
			}
		}

		/*update speed-up speed*/
		oplus_update_motor_speed(chip->pwm_param.run.speed_up.speed);

		ret = oplus_motor_pwm_config(chip->pwm_param.pwm_duty,
					    chip->pwm_param.pwm_period, 0);

		if (ret < 0) {
			MOTOR_ERR("pwm_config fail ret =  %d \n", ret);
			chip->motor_started = 0;
			mutex_unlock(&motor_running_mutex);
			return;
		}

		ret = oplus_motor_pwm_enable();

		if (ret < 0) {
			MOTOR_ERR("pwm_enable fail ret = %d \n", ret);
			chip->motor_started = 0;
			mutex_unlock(&motor_running_mutex);
			return;

		} else {
			if (chip->md_dir == MOTOR_UPWARD) {
				report_positon_state_up(chip);
				oplus_set_motor_move_state(MOTOR_UPWARD_ING);

			} else if (chip->md_dir == MOTOR_DOWN) {
				report_positon_state_down(chip);
				oplus_set_motor_move_state(MOTOR_DOWNWARD_ING);
			}

			MOTOR_ERR("pwm_enable success ret = %d \n", ret);
		}

		speed = chip->pwm_param.run.full.speed;
		hrtimer_start(&chip->stop_timer, motor_cal_full_timer(chip->strong_mag),
			      HRTIMER_MODE_REL);

		/*calculate when the motor should speed up*/
		pwm = chip->pwm_param.run.speed_up.pwm;
		speed = chip->pwm_param.pwm_period;
		hrtimer_start(&chip->speed_up_timer, motor_cal_timer(pwm, speed),
			      HRTIMER_MODE_REL);

	} else if (!chip->motor_enable && (chip->motor_started == 1)) {
		MOTOR_ERR("stop motor\n");

		if (!chip->stop_timer_trigger) {
			hrtimer_cancel(&chip->stop_timer);

		} else {
			chip->stop_timer_trigger = 0;
		}

		oplus_motor_pwm_disable();
		oplus_motor_set_power(STAY_ACTIVE, MOTOR_POWER_OFF);

		chip->motor_started = 0;

		if (chip->move_state == MOTOR_UPWARD_ING) {
			oplus_set_motor_move_state(MOTOR_UPWARD_STOP);

			oplus_dhall_get_data(DHALL_1, &hall1_val, false);

			if (hall1_val > chip->cali_data.irq_pos_hall1
					|| chip->strong_mag) { /* if strong mag, set normal*/
				chip->position = UP_STATE;
				MOTOR_ERR("POS_NORMAL, irq_count1 %d\n", chip->irq_count[DHALL_1]);
				chip->irq_count[DHALL_1] = 0;
				oplus_dhall_update_threshold(DHALL_1, UP_STATE, chip->cali_data.irq_pos_hall1);
				oplus_motor_notify_state(MOTOR_UP_EVENT);
				report_positon_state_up_normal(chip);

			} else {
				chip->position = MID_STATE;
				report_positon_state_up_abnormal(chip);
				MOTOR_ERR("POS_ABNORMAL %d %d\n", hall1_val, chip->cali_data.irq_pos_hall1);
				oplus_motor_notify_state(MOTOR_BLOCK_EVENT);
			}

		} else if (chip->move_state ==
				MOTOR_DOWNWARD_ING) { /* if strong mag, set normal*/
			oplus_set_motor_move_state(MOTOR_DOWNWARD_STOP);

			oplus_dhall_get_data(DHALL_0, &hall0_val, false);

			if (hall0_val > chip->cali_data.irq_pos_hall0 || chip->strong_mag) {
				chip->position = DOWN_STATE;
				MOTOR_ERR("POS_NORMAL, irq_count0 %d\n", chip->irq_count[DHALL_0]);
				chip->irq_count[DHALL_0] = 0;

				if (chip->down_state_high_thresh_support
						&& hall0_val > chip->cali_data.down_retard_hall0) {
					MOTOR_ERR("update threshold to HALL_THRESH_HIGH\n");
					oplus_dhall_update_threshold(DHALL_0, DOWN_STATE, HALL_THRESH_HIGH);

				} else {
					oplus_dhall_update_threshold(DHALL_0, DOWN_STATE, chip->cali_data.irq_pos_hall0);
				}

				oplus_motor_notify_state(MOTOR_DOWN_EVENT);
				report_positon_state_down_normal(chip);

			} else {
				chip->position = MID_STATE;
				report_positon_state_down_abnormal(chip);
				MOTOR_ERR("POS_ABNORMAL %d %d\n", hall0_val, chip->cali_data.irq_pos_hall0);
				oplus_motor_notify_state(MOTOR_BLOCK_EVENT);
			}
		}

		if (chip->shutting) {
			complete(&shutdown_comp);
		}

		oplus_motor_set_awake(chip, MOTOR_RUN_LOCK, false);
	}

	mutex_unlock(&motor_running_mutex);
}

static void manual_to_auto_up_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_motor_chip *chip = container_of(dwork, struct oplus_motor_chip,
				       up_work);
	short hall0_val = 0, hall1_val = 0;
	const unsigned long msecond = 1000;/*1s*/
	const int threshold = 3;
	static unsigned long current_ts  = 0;
	static int strong_mag_count = 0;

	oplus_motor_set_awake(chip, HALL_DATA_LOCK, true); /*dont sleep*/

	if (atomic_read(&chip->in_suspend)) {
		MOTOR_ERR("in_suspend delay 20 ms \n");
		queue_delayed_work(chip->manual2auto_wq, &chip->up_work, msecs_to_jiffies(20));
		return;
	}

	oplus_dhall_get_data(DHALL_0, &hall0_val, false);
	oplus_dhall_get_data(DHALL_1, &hall1_val, false);

	/*strong mag check*/
	if (check_irq_strong_mag(MOTOR_PULL,
				 hall0_val, hall1_val, chip->noise.pre_hall0, chip->noise.pre_hall1)) {
		chip->irq_count[DHALL_0] = 0;
		strong_mag_count++;

		if (!current_ts) {
			MOTOR_ERR("start timer \n");
			current_ts = jiffies;
		}

		/*strong mag count will clear after 1s and reset timer*/
		if (time_after(current_ts, current_ts + msecs_to_jiffies(msecond))) {
			strong_mag_count = 1;
			current_ts = 0;
			MOTOR_ERR("timer out\n");
		}

		/*need to disable hall0 irq for power comsumption*/
		if ((strong_mag_count > threshold) && (chip->position == DOWN_STATE)) {
			MOTOR_ERR("disable hall0 irq for strong mag \n");
			strong_mag_count = 0;
			current_ts = 0;
			oplus_dhall_update_threshold(DHALL_0, DOWN_STATE, HALL_THRESH_HIGH);
		}

		goto out;

	} else {
		strong_mag_count = 0;
		current_ts = 0;
	}

	oplus_motor_irq_monitor(chip);

	oplus_dhall_update_threshold(DHALL_0, MID_STATE, HALL_THRESH_HIGH);

	if (chip->manual2auto_up_switch) {
		if (!chip->motor_started) {
			report_positon_state_manual_to_up(chip);
			oplus_motor_upward();
		}

	} else {
		if (!chip->is_motor_test) {
			schedule_work(&chip->manual_position_work);
		}
	}

out:
	oplus_dhall_clear_irq(DHALL_0);

	if (!chip->is_irq_abnormal) {
		oplus_dhall_enable_irq(DHALL_0, true);
	}

	oplus_motor_set_awake(chip, HALL_DATA_LOCK, false);
}

static void manual_to_auto_down_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_motor_chip *chip = container_of(dwork, struct oplus_motor_chip,
				       down_work);
	short hall0_val = 0, hall1_val = 0;

	oplus_motor_set_awake(chip, HALL_DATA_LOCK, true);/*dont sleep*/

	oplus_dhall_get_data(DHALL_0, &hall0_val, false);
	oplus_dhall_get_data(DHALL_1, &hall1_val, false);

	if (check_irq_strong_mag(MOTOR_PUSH,
				 hall0_val, hall1_val, chip->noise.pre_hall0, chip->noise.pre_hall1)) {
		chip->irq_count[DHALL_1] = 0;
		goto out;
	}

	oplus_motor_irq_monitor(chip);

	if (atomic_read(&chip->in_suspend)) {
		MOTOR_ERR("in_suspend delay 20 ms \n");
		queue_delayed_work(chip->manual2auto_wq, &chip->down_work,
				   msecs_to_jiffies(20));
		return;
	}

	oplus_dhall_update_threshold(DHALL_1, MID_STATE, HALL_THRESH_HIGH);

	if (chip->manual2auto_down_switch) {
		if (!chip->motor_started) {
			report_positon_state_manual_to_down(chip);
			oplus_motor_downward(IRQ_START);
		}
	}

out:
	oplus_dhall_clear_irq(DHALL_1);

	if (!chip->is_irq_abnormal) {
		oplus_dhall_enable_irq(DHALL_1, true);
	}

	oplus_motor_set_awake(chip, HALL_DATA_LOCK, false);
}

static bool extre_stop_check(struct oplus_motor_chip *chip, short hall1_val,
			     short hall1_val_pre, short hall0_val, short hall0_val_pre)
{
	if ((chip->move_state == MOTOR_UPWARD_ING)
			&& (abs(hall1_val_pre) > abs(hall1_val))) {
		MOTOR_ERR("moving up stop by hall1 < hall1_pre.\n");
		return true;
	}

	if ((chip->move_state == MOTOR_DOWNWARD_ING)
			&& (abs(hall0_val_pre) > abs(hall0_val))
			&& (hall0_val > (chip->cali_data.down_retard_hall0 + chip->mono_offset))) {
		MOTOR_ERR("moving down stop by hall0 < hall0_pre.\n");
		return true;
	}

	return false;
}

#define VALID_CHECK_THD (100)
static bool hall_valid_check(struct oplus_motor_chip *chip, short hall1_val,
			     short hall1_val_pre, short hall0_val, short hall0_val_pre)
{
	/*prevent one byte to two byte, hall value can't sync*/
	if ((chip->move_state == MOTOR_DOWNWARD_ING) && (hall0_val > 2 * hall0_val_pre)
			&& (hall0_val_pre > VALID_CHECK_THD)) {
		return false;
	}

	if ((chip->move_state == MOTOR_UPWARD_ING) && (hall1_val > 2 * hall1_val_pre)
			&& (hall1_val_pre > VALID_CHECK_THD)) {
		return false;
	}

	return true;
}

static void position_detect_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oplus_motor_chip *chip = container_of(dwork, struct oplus_motor_chip,
				       detect_work);
	int hall_delta_pre = 0;
	int hall_delta = 0;
	int delta_d = 0;
	int speed_down = 0;
	int should_stop_count = 0;
	int pwm_count = 0;
	short hall0_val_pre = 0, hall1_val_pre = 0;
	short hall0_val = 0, hall1_val = 0;
	int stop_neg_val = HALL_STOP_NEG_DL;
	int stop_pos_val = HALL_STOP_POS_DL;
	int stop_retry = chip->stop_retry;
	unsigned long usecond = 0;
	unsigned long time_current = jiffies, time_start = 0;
	int len = 0;
	bool switch2fullspeed = false;
	bool reset_retry = false;
	bool first_time_change_speed = false;
	int delay_cycle = 0;
	short hall0_cali_diff = 0, cur_down_offset = 0;
	int down_retard_hall0 = chip->cali_data.down_retard_hall0;
	int down_stop_offset = chip->stop.offset[1];
	int retard_range_end_offset = 0;

	mutex_lock(&position_detect_mutex);
	oplus_motor_set_awake(chip, POSITION_DETECT_LOCK,
			     true); /*should not be sleep during detecting*/

	oplus_dhall_clear_max_data(chip);

	/*change to no position detect speed*/
	oplus_motor_change_speed(&chip->pwm_param.run.nd);

	/*delay position detect*/
	pwm_count = oplus_motor_calculate_pwm_count(chip->pwm_param.run.nd.l, 0);
	usecond = pwm_count * chip->pwm_param.run.nd.speed / 1000;
	time_start = jiffies;

	MOTOR_LOG("start position detect, after %ld us \n", usecond);

#ifdef CONFIG_MOTOR_DETECT_QOS

	if (motor_qos_state) {
		motor_qos_value = PM_QOS_MOTOR_WAKEUP_VALUE;
		pm_qos_update_request(&motor_qos_req, motor_qos_value);
	}

#endif

	while (chip->motor_started) {
		oplus_dhall_get_data(DHALL_0, &hall0_val, false);
		oplus_dhall_get_data(DHALL_1, &hall1_val, false);

		if (usecond && ((chip->md_dir == MOTOR_DOWN
				 && hall0_val < chip->cali_data.irq_pos_hall0) ||
				(chip->md_dir == MOTOR_UPWARD && hall1_val < chip->cali_data.irq_pos_hall1))) {
			time_current = jiffies;

			if (!time_after(time_current, time_start + usecs_to_jiffies(usecond))) {
				usleep_range(15000, 15000);
				continue;

			} else {
				usecond = 0;
			}

		} else {
			usecond = 0;
		}

		/*change to full speed*/
		if (!switch2fullspeed) {
			switch2fullspeed = true;
			oplus_motor_change_speed(&chip->pwm_param.run.full);
			delay_cycle = chip->pwm_param.run.delay;
		}

		hall_delta = hall0_val - hall1_val;
		delta_d = hall_delta - hall_delta_pre;
		hall_delta_pre = hall_delta;

		MOTOR_ERR("dhall_0:[%4d], dhall_1:[%4d], hall_delta:[%4d], delta_d:[%4d]\n",
			  hall0_val, hall1_val, hall_delta, delta_d);

		/*record hall data during running*/
		if (chip->down_data_buf != NULL && chip->up_data_buf != NULL) {
			if ((chip->md_dir == MOTOR_DOWN) && (MAX_BUF_LEN - len - 1 > 0)) {
				len += snprintf(chip->down_data_buf + len, MAX_BUF_LEN - len - 1,
						"[%4d]\t[%4d]\t[%4d]\t[%4d]\t[DOWN]\n",
						hall0_val, hall1_val, hall_delta, delta_d);

			} else if ((chip->md_dir == MOTOR_UPWARD) && (MAX_BUF_LEN - len - 1 > 0)) {
				len += snprintf(chip->up_data_buf + len, MAX_BUF_LEN - len - 1,
						"[%4d]\t[%4d]\t[%4d]\t[%4d]\t[UPWARD]\n",
						hall0_val, hall1_val, hall_delta, delta_d);
			}
		}

		oplus_dhall_record_max_data(hall0_val, hall1_val);

		if (chip->strong_mag) {
			usleep_range(delay_cycle, delay_cycle);
			continue;
		}

		/*only set one time, when reach full range after slow down, reset stop_retry to 5*/
		if (speed_down && !reset_retry && !chip->stop.offset[0]
				&& !chip->stop.offset[1]) {
			reset_retry = true;

			if ((hall0_val == HALL_FULL_RANGE) || (hall1_val == HALL_FULL_RANGE)) {
				stop_retry = 5;
			}
		}

		if (!in_cali) {
			retard_range_end_offset = -g_chip->retard_range_end;

		} else {
			retard_range_end_offset = 0;
		}

		if (chip->motor_small_irq_later_speed_down_support
				&& chip->cali_data.irq_pos_hall0 < chip->small_irq) {
			down_retard_hall0 = chip->cali_data.down_retard_hall0 + chip->later_speed_down;
			down_stop_offset = chip->later_speed_down_stop_offset;

		} else {
			down_retard_hall0 = chip->cali_data.down_retard_hall0;
			down_stop_offset = chip->stop.offset[1];
		}

		/*speed down , shoud fix magnetic noise*/
		if (hall0_val > down_retard_hall0) {
			if ((chip->move_state == MOTOR_DOWNWARD_ING) && (speed_down == 0) && !in_cali) {
				speed_down = 1;

				if (chip->motor_retard_offset_support) {
					oplus_motor_change_speed(&chip->pwm_param.run.full);

				} else {
					oplus_motor_change_speed(&chip->pwm_param.run.speed_down);
				}

				/*oplus_motor_change_speed(&chip->pwm_param.run.speed_down);*/
				usleep_range(5000, 5000);

				if (chip->motor_retard_offset_support) {
					delay_cycle = chip->pwm_param.run.down_brake_delay - 6000;

				} else {
					delay_cycle = chip->pwm_param.run.down_brake_delay;
				}

				first_time_change_speed = true;
			}

		} else if (hall1_val > chip->cali_data.up_retard_hall1 +
				retard_range_end_offset) {
			if ((chip->move_state == MOTOR_UPWARD_ING) && (speed_down == 0)) {
				speed_down = 1;
				oplus_motor_change_speed(&chip->pwm_param.run.speed_down);
				usleep_range(5000, 5000);
				delay_cycle = chip->pwm_param.run.up_brake_delay;
				first_time_change_speed = true;

				if (!in_cali && retard_range_end_offset) {
					if (chip->hall_detect_switch) {
						oplus_motor_stop();
					}

					break;
				}
			}
		}

		/* check if need to change stop strategy*/
		if (hall0_val < chip->cali_data.irq_pos_hall0
				&& hall1_val < chip->cali_data.irq_pos_hall1) {
			stop_neg_val = HALL_STOP_NEG_DL;
			stop_pos_val = HALL_STOP_POS_DL;

		} else {
			/*
			 * 1) terminal_offset > 0 && pos_change_thrd = 0,
			 *  set retry as once times when pos > retard point + terminal_offset
			 * 2) terminal_offset = 0 && pos_change_thrd > 0
			 *  set stop range according to pos_change_thrd
			 * 3) do nothing special about  stop range
			 */
			if ((chip->stop.offset[0] > 0) && (down_stop_offset > 0)) {
				hall0_cali_diff = chip->run_check_hall0 - down_retard_hall0 -
						  chip->stop.cali_retard_offset[1];

				if ((hall0_cali_diff > chip->stop.hall_max_thrd[1])
						&& (chip->move_state == MOTOR_DOWNWARD_ING)) {
					cur_down_offset = down_stop_offset + hall0_cali_diff;

					if (hall0_val > (down_retard_hall0 + cur_down_offset)) {
						stop_retry = 1;
						stop_neg_val = chip->stop.neg[1];
						stop_pos_val = chip->stop.pos[1];
					}

					MOTOR_ERR("down stop offset changed to %d(run_check:%d).\n", cur_down_offset,
						  chip->run_check_hall0);

				} else if (hall0_val >= (down_retard_hall0 + down_stop_offset)) {
					if (chip->move_state == MOTOR_DOWNWARD_ING) {
						stop_retry = chip->offset_stop_retry;
						stop_neg_val = chip->stop.neg[1];
						stop_pos_val = chip->stop.pos[1];
					}

				} else if (hall1_val >= (chip->cali_data.up_retard_hall1 +
							 chip->stop.offset[0])) {
					if (chip->move_state == MOTOR_UPWARD_ING) {
						stop_retry = chip->offset_stop_retry;
						stop_neg_val = chip->stop.neg[0];
						stop_pos_val = chip->stop.pos[0];
					}
				}

			} else if ((chip->stop.pos_change_thrd[0] > 0)
					&& (chip->stop.pos_change_thrd[1] > 0)) {
#define STOP_RANG_A (3)
#define STOP_RANG_B (2)
				stop_retry = 2;

				if (chip->move_state == MOTOR_DOWNWARD_ING) {
					if (hall0_val > chip->stop.pos_change_thrd[1]) {
						if (chip->motor_stop_val_support) {
							stop_neg_val = -chip->stop_rang_a;
							stop_pos_val = chip->stop_rang_a;

						} else {
							stop_neg_val = -STOP_RANG_A;
							stop_pos_val = STOP_RANG_A;
						}

					} else {
						if (chip->motor_stop_val_support) {
							stop_neg_val = -chip->stop_rang_b;
							stop_pos_val = chip->stop_rang_b;

						} else {
							stop_neg_val = -STOP_RANG_B;
							stop_pos_val = STOP_RANG_B;
						}
					}

				} else if (chip->move_state == MOTOR_UPWARD_ING) {
					if (hall1_val > chip->stop.pos_change_thrd[0]) {
						if (chip->motor_stop_val_support) {
							stop_neg_val = -chip->stop_rang_a;
							stop_pos_val = chip->stop_rang_a;

						} else {
							stop_neg_val = -STOP_RANG_A;
							stop_pos_val = STOP_RANG_A;
						}

					} else {
						if (chip->motor_stop_val_support) {
							stop_neg_val = -chip->stop_rang_b;
							stop_pos_val = chip->stop_rang_b;

						} else {
							stop_neg_val = -STOP_RANG_B;
							stop_pos_val = STOP_RANG_B;
						}
					}
				}

				MOTOR_ERR("down stop val stop_neg_val = %d, stop_pos_val = %d.\n", stop_neg_val,
					  stop_pos_val);

			} else {
				if (chip->move_state == MOTOR_UPWARD_ING) {
					stop_neg_val = chip->stop.neg[0];
					stop_pos_val = chip->stop.pos[0];

				} else if (chip->move_state == MOTOR_DOWNWARD_ING) {
					stop_neg_val = chip->stop.neg[1];
					stop_pos_val = chip->stop.pos[1];
				}
			}
		}

		if ((chip->move_state == MOTOR_DOWNWARD_ING)
				&& chip->cal_fix_stop_retry_support && hall0_val >= (down_retard_hall0)) {
			stop_retry = 1;
			stop_neg_val = chip->full_stop_neg_val;
			stop_pos_val = chip->full_stop_pos_val;
		}

		if (chip->is_in_calibration && chip->cal_fix_stop_retry_support) {
			stop_retry = chip->cal_fix_stop_retry;
			stop_neg_val = chip->cal_stop_neg_val;
			stop_pos_val = chip->cal_stop_pos_val;
		}

		/*stop motor*/
		if (chip->stop_in_advance) {
			/* this strategy need to sync with silence calibration */
			if (hall1_val > (chip->cali_data.up_retard_hall1 + STOP_DELTA)) {
				if (chip->move_state == MOTOR_UPWARD_ING) {
					stop_retry = 1;
					stop_neg_val = -STOP_IN_ADVANCE_RANG;
					stop_pos_val = STOP_IN_ADVANCE_RANG;
				}
			}
		}

		if ((delta_d >= stop_neg_val) && (delta_d <= stop_pos_val)) {
			if (++should_stop_count >= stop_retry) {
				if (chip->hall_detect_switch) {
					MOTOR_LOG("Retry over %d times and STOP.\n", stop_retry);
					oplus_motor_stop();
					break;
				}
			}

		} else {
			should_stop_count = 0;
		}

		/*if ((chip->motor_down_stop_support) && speed_down && (chip->move_state == MOTOR_DOWNWARD_ING)) {
			if (hall0_val >= (down_retard_hall0 + down_stop_offset)) {
				if (delta_d <= -chip->motor_down_stop_val) {
					MOTOR_LOG("motor_down_stop_support retry over 1 times and STOP.\n");
					oplus_motor_stop();
					break;
				}
			}
		}*/
		/*will check hall data's monotonicity in speed down condition*/
		if (chip->extre_stop_support && !chip->is_in_calibration && speed_down
				&& extre_stop_check(chip, hall1_val, hall1_val_pre, hall0_val, hall0_val_pre)) {
			if (chip->hall_detect_switch) {
				oplus_motor_stop();
				break;
			}
		}

		if (hall_valid_check(chip, hall1_val, hall1_val_pre, hall0_val,
				     hall0_val_pre)) {
			hall0_val_pre = hall0_val;
			hall1_val_pre = hall1_val;

		} else {
			MOTOR_ERR("stop update hall previous value.\n");
		}

		if (first_time_change_speed && delay_cycle > 5000) {
			usleep_range(delay_cycle - 5000, delay_cycle - 5000);
			first_time_change_speed = false;

		} else {
			usleep_range(delay_cycle, delay_cycle);
		}
	}

#ifdef CONFIG_MOTOR_DETECT_QOS

	if (PM_QOS_MOTOR_WAKEUP_VALUE == motor_qos_value) {
		motor_qos_value = PM_QOS_DEFAULT_VALUE;
		pm_qos_update_request(&motor_qos_req, motor_qos_value);
	}

#endif

	oplus_motor_set_awake(chip, POSITION_DETECT_LOCK, false);
	mutex_unlock(&position_detect_mutex);
}

static void manual_position_detect_work(struct work_struct *work)
{
	struct oplus_motor_chip *chip = container_of(work, struct oplus_motor_chip,
				       manual_position_work);
	int hall_delta_pre = 0;
	int hall_delta = 0;
	int delta_d = 0;
	int should_stop_count = 0;
	short hall0_val, hall1_val;
	int stop_neg_val = HALL_STOP_NEG_DL;
	int stop_pos_val = HALL_STOP_POS_DL;

	mutex_lock(&position_detect_mutex);
	oplus_motor_set_awake(chip, POSITION_DETECT_LOCK,
			     true); /*should not be sleep during detecting*/

	while (1) {
		oplus_dhall_get_data(DHALL_0, &hall0_val, false);
		oplus_dhall_get_data(DHALL_1, &hall1_val, false);

		hall_delta = hall0_val - hall1_val;
		delta_d = hall_delta - hall_delta_pre;
		hall_delta_pre = hall_delta;

		MOTOR_ERR("dhall_0:[%4d], dhall_1:[%4d], hall_delta:[%4d], delta_d:[%4d]\n",
			  hall0_val, hall1_val, hall_delta, delta_d);

		if (chip->move_state == MOTOR_DOWNWARD_ING) {
			stop_neg_val = chip->stop.neg[1];
			stop_pos_val = chip->stop.pos[1];

		} else if (chip->move_state == MOTOR_UPWARD_ING) {
			stop_neg_val = chip->stop.neg[0];
			stop_pos_val = chip->stop.pos[0];
		}

		/*stop motor*/
		if ((delta_d >= stop_neg_val) && (delta_d <= stop_pos_val)) {
			should_stop_count++;

			if (should_stop_count >= 2) {
				should_stop_count = 0;
				hall_delta_pre = 0;

				if (hall1_val > chip->cali_data.irq_pos_hall1) {
					chip->position = UP_STATE;
					oplus_motor_notify_state(MOTOR_UP_EVENT);
					oplus_dhall_update_threshold(DHALL_1, UP_STATE, chip->cali_data.irq_pos_hall1);

				} else {
					chip->position = MID_STATE;
				}

				report_positon_state_manual_to_up(chip);
				MOTOR_LOG("up_retard_hall1 %d\n", chip->cali_data.up_retard_hall1);
				oplus_motor_set_awake(chip, POSITION_DETECT_LOCK, false);
				break;
			}

		} else {
			should_stop_count = 0;
		}

		usleep_range(chip->pwm_param.run.delay, chip->pwm_param.run.delay);
	}

	mutex_unlock(&position_detect_mutex);
}

static void oplus_motor_reset_check(struct oplus_motor_chip *chip)
{
	short hall0_val = 0;
	oplus_dhall_get_data(DHALL_0, &hall0_val, false);

	MOTOR_LOG("hall0 data %d  irq_pos_hall0 %d \n", hall0_val,
		  chip->cali_data.irq_pos_hall0);

	if (hall0_val < chip->cali_data.irq_pos_hall0) {
		MOTOR_LOG("reset motor \n");
		oplus_motor_downward(USER_START);

	} else {
		chip->position = DOWN_STATE;
		oplus_motor_set_power(STAY_ACTIVE, MOTOR_POWER_OFF);
	}
}

/*user space interface*/
static ssize_t motor_direction_store(struct device *pdev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	unsigned long direction = 0;

	if (!g_chip) {
		MOTOR_ERR("g_chip null\n");
		return count;
	}

	if (g_chip->motor_started) {
		return count;
	}

	if (sscanf(buf, "%lu", &direction) == 1) {
		oplus_set_direction_para(direction);
	}

	return count;
}

static ssize_t motor_direction_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	if (!g_chip) {
		MOTOR_ERR("g_chip null\n");
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", g_chip->md_dir);
}

static ssize_t motor_speed_store(struct device *pdev,
				 struct device_attribute *attr,
				 const char *buff, size_t count)
{
	int md_index = 0, brake_index = 0, pulse_index = 0;

	if (!g_chip) {
		MOTOR_ERR("g_chip null\n");
		return count;
	}

	if (g_chip->motor_started) {
		return count;
	}

	if (sscanf(buff, "%d,%d,%d", &md_index, &brake_index, &pulse_index) == 3) {
		if (oplus_motor_get_speed(md_index, &g_chip->pwm_param.normal.full.speed)) {
			MOTOR_ERR("failed to set md speed\n");
		}

		if (oplus_motor_get_speed(md_index, &g_chip->pwm_param.normal.nd.speed)) {
			MOTOR_ERR("failed to set nd speed\n");
		}

		if (oplus_motor_get_speed(brake_index,
					 &g_chip->pwm_param.normal.speed_down.speed)) {
			MOTOR_ERR("failed to set brake speed\n");
		}

		if (oplus_motor_get_speed(pulse_index,
					 &g_chip->pwm_param.normal.speed_up.speed)) {
			MOTOR_ERR("failed to set pulse speed\n");
		}
	}

	MOTOR_LOG("pwm_param.full.speed:%d pwm_param.speed_down.speed:%d pwm_param.speed_up.speed:%d\n",
		  g_chip->pwm_param.normal.full.speed,
		  g_chip->pwm_param.normal.speed_down.speed,
		  g_chip->pwm_param.normal.speed_up.speed);

	return count;
}

static ssize_t motor_speed_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int index = 0, c = 0;
	unsigned int speed;

	if (!g_chip) {
		MOTOR_ERR("g_chip null\n");
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);
	}

	do {
		if (!oplus_motor_get_speed(index, &speed)) {
			c += snprintf(buf + c, PAGE_SIZE, "%d.%dkHZ:%d ", 1000000 / speed,
				      (100 * 1000000 / speed) % 100, index);

		} else {
			break;
		}
	} while (index++ < 20);

	c += snprintf(buf + c, PAGE_SIZE,
		      "\n\npwm_param.full.speed:%d pwm_param.speed_up.speed:%d pwm_param.speed_down.speed:%d\n",
		      g_chip->pwm_param.run.full.speed,
		      g_chip->pwm_param.run.speed_up.speed,
		      g_chip->pwm_param.run.speed_down.speed);

	return c;
}

static ssize_t motor_mode_store(struct device *pdev,
				struct device_attribute *attr,
				const char *buff, size_t count)
{
	int mdmode = 0;

	if (!g_chip) {
		MOTOR_ERR("g_chip null\n");
		return count;
	}

	if (g_chip->motor_started) {
		return count;
	}

	if (sscanf(buff, "%d", &mdmode) == 1) {
		MOTOR_LOG("mdmode = %d\n", mdmode);
		oplus_set_md_mode_para(mdmode);
	}

	return count;
}

static ssize_t	motor_mode_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	if (!g_chip) {
		MOTOR_ERR("g_chip null\n");
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", g_chip->pwm_param.md_mode);
}

static ssize_t motor_enable_store(struct device *pdev,
				  struct device_attribute *attr,
				  const char *buff, size_t count)
{
	int enable = 0;

	if (!g_chip) {
		MOTOR_ERR("g_chip null\n");
		return count;
	}

	if (sscanf(buff, "%d", &enable) == 1) {
		if (enable) {
			MOTOR_LOG("oplus_motor_start %d\n", enable);
			oplus_motor_start(enable, g_chip->md_dir);

		} else {
			oplus_motor_stop();
		}
	}

	return count;
}


static ssize_t motor_sw_switch_store(struct device *pdev,
				     struct device_attribute *attr,
				     const char *buff, size_t count)
{
	unsigned long sw_switch = 0;

	if (!g_chip) {
		MOTOR_ERR("g_chip null\n");
		return count;
	}

	if (sscanf(buff, "%lu", &sw_switch) == 1) {
		g_chip->motor_switch = sw_switch;
	}

	MOTOR_LOG("sw_switch %d\n", g_chip->motor_switch);
	return count;
}

static ssize_t	motor_sw_switch_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	if (!g_chip) {
		MOTOR_ERR("g_chip null\n");
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", g_chip->motor_switch);
}


static ssize_t dhall_detect_switch_store(struct device *pdev,
		struct device_attribute *attr,
		const char *buff, size_t count)
{
	unsigned long sw_switch = 0;

	if (!g_chip) {
		MOTOR_ERR("g_chip null\n");
		return count;
	}

	if (sscanf(buff, "%lu", &sw_switch) == 1) {
		if (sw_switch) {
			g_chip->hall_detect_switch = true;

		} else {
			g_chip->hall_detect_switch = false;
		}
	}

	return count;
}

static ssize_t	dhall_detect_switch_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	if (!g_chip) {
		MOTOR_ERR("g_chip null\n");
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", g_chip->hall_detect_switch);
}

static ssize_t dhall_get_data_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	if (!g_chip || !g_chip->up_data_buf || !g_chip->down_data_buf) {
		MOTOR_ERR("g_chip null\n");
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);
	}

	return snprintf(buf, PAGE_SIZE, "%s\n%s\n", g_chip->up_data_buf,
			g_chip->down_data_buf);
}

static ssize_t	motor_all_config_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	int config[6] = {0};

	if (!g_chip) {
		MOTOR_ERR("g_chip null\n");
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);
	}

	oplus_motor_get_all_config(config, 6);

	return snprintf(buf, PAGE_SIZE,
			"config {sleep:%d step:%d m0:%d m1:%d vref:%d dir:%d}\n",
			config[0], config[1], config[2],
			config[3], config[4], config[5]);
}

static ssize_t	motor_position_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	int config[6] = {0};

	if (!g_chip) {
		MOTOR_ERR("g_chip null\n");
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", g_chip->position);
}

static ssize_t	motor_test_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	if (!g_chip) {
		MOTOR_ERR("g_chip null\n");
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);
	}

	MOTOR_LOG("is_motor_test %d\n", g_chip->is_motor_test);

	return snprintf(buf, PAGE_SIZE, "%d\n", g_chip->is_motor_test);
}

static ssize_t motor_test_store(struct device *pdev,
				struct device_attribute *attr,
				const char *buff, size_t count)
{
	int test = 0;

	if (!g_chip) {
		MOTOR_ERR("g_chip null\n");
		return count;
	}

	if (sscanf(buff, "%d", &test) == 1) {
		g_chip->is_motor_test = test;
	}

	MOTOR_LOG("test %d ,is_motor_test %d\n", test, g_chip->is_motor_test);
	return count;
}

static void force_move_work(struct work_struct *work)
{
	struct oplus_motor_chip *chip = container_of(work, struct oplus_motor_chip,
				       force_work);
	MOTOR_LOG("force stop motor\n");

	oplus_motor_pwm_disable();
	oplus_motor_set_power(STAY_ACTIVE, MOTOR_POWER_OFF);

	chip->force_moving = false;
	chip->manual2auto_down_switch = true;
	chip->is_motor_test = false;
}

static enum hrtimer_restart motor_force_move_stop_timer_func(
	struct hrtimer *hrtimer)
{
	if (!g_chip) {
		MOTOR_ERR("g_chip null\n");
		return HRTIMER_NORESTART;
	}

	schedule_work(&g_chip->force_work);

	return HRTIMER_NORESTART;
}

static ssize_t motor_force_move_store(struct device *pdev,
				      struct device_attribute *attr, const char *buff, size_t count)
{
	int enable = 0;
	int pwm_count = 0;

	if (!g_chip) {
		MOTOR_ERR("g_chip null\n");
		return count;
	}

	if (sscanf(buff, "%d", &enable) == 1) {
		if (!enable) {
			return count;
		}

		if (!g_chip->force_moving) {
			g_chip->force_moving = true;
			g_chip->is_motor_test = true;
			g_chip->manual2auto_down_switch = false;

			oplus_update_motor_speed(g_chip->pwm_param.normal.full.speed);
			pwm_count = g_chip->pwm_param.normal.full.pwm +
				    g_chip->pwm_param.normal.nd.pwm +
				    g_chip->pwm_param.normal.speed_down.pwm +
				    g_chip->pwm_param.normal.speed_up.pwm;
			pwm_count *= 22 / 20;

			oplus_motor_set_power(STAY_ACTIVE, MOTOR_POWER_ON);
			oplus_motor_set_direction(g_chip->md_dir);
			oplus_motor_pwm_config(g_chip->pwm_param.pwm_duty, g_chip->pwm_param.pwm_period,
					      pwm_count);
			oplus_motor_pwm_enable();

			hrtimer_start(&g_chip->force_move_timer, motor_cal_timer(pwm_count,
					g_chip->pwm_param.pwm_period), HRTIMER_MODE_REL);
		}
	}

	return count;
}

static ssize_t dhall_data_store(struct device *pdev,
				struct device_attribute *attr, const char *buff, size_t count)
{
	int id = 0;
	char value[16] = {0};

	if (!g_chip) {
		return count;
	}

	strncpy(value, buff, 15);

	if (value[0] == '0') {
		id = DHALL_0;

	} else if (value[0] == '1') {
		id = DHALL_1;

	} else {
		MOTOR_ERR("invalid id\n");
		return count;
	}

	oplus_dhall_set_SRS(id, &value[2]);

	return count;
}

static ssize_t dhall_data_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	short hall0_val, hall1_val;

	if (!g_chip) {
		MOTOR_ERR("g_chip null\n");
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);
	}

	oplus_dhall_get_data(DHALL_0, &hall0_val, true);
	oplus_dhall_get_data(DHALL_1, &hall1_val, true);

	MOTOR_ERR("dhall0:%d dhall1:%d \n", hall0_val, hall1_val);

	return sprintf(buf, "%d,%d\n", hall0_val, hall1_val);
}

static ssize_t dhall_max_data_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	short hall0_val, hall1_val;

	if (!g_chip) {
		MOTOR_ERR("g_chip null\n");
		return snprintf(buf, PAGE_SIZE, "%d,%d\n", 0, 0);
	}

	oplus_dhall_get_data(DHALL_0, &hall0_val, true);
	oplus_dhall_get_data(DHALL_1, &hall1_val, true);

	MOTOR_ERR("[%d,%d,%d,%d]\n", hall0_val, hall1_val, g_chip->hall_max.data0,
		  g_chip->hall_max.data1);

	if (g_chip->position == DOWN_STATE) {
		return sprintf(buf, "%d,%d\n", g_chip->hall_max.data0 * g_chip->sign,
			       hall1_val);

	} else if (g_chip->position == UP_STATE) {
		return sprintf(buf, "%d,%d\n", hall0_val,
			       g_chip->hall_max.data1 * g_chip->sign);

	} else {
		return snprintf(buf, PAGE_SIZE, "%d,%d\n", 0, 0);
	}
}

static ssize_t dhall_all_reg_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	u8 _buf0[512] = {0};
	u8 _buf1[512] = {0};

	oplus_dhall_dump_regs(DHALL_0, _buf0);
	oplus_dhall_dump_regs(DHALL_1, _buf1);

	return sprintf(buf, "Hall0:\n%s \nHall1:\n%s\n", _buf0, _buf1);
}

static ssize_t motor_move_state_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	if (!g_chip) {
		MOTOR_ERR("g_chip null\n");
		return snprintf(buf, PAGE_SIZE, "%d\n", -1);
	}

	return sprintf(buf, "%d\n", g_chip->move_state);
}

static ssize_t dhall_irq_count_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	if (!g_chip) {
		MOTOR_ERR("g_chip null\n");
		return snprintf(buf, PAGE_SIZE, "%d,%d\n", -1, -1);
	}

	return sprintf(buf, "%d,%d\n", g_chip->irq_count[DHALL_0],
		       g_chip->irq_count[DHALL_1]);
}

static ssize_t motor_manual2auto_switch_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int manual2auto_switch = 0;

	if (!g_chip) {
		MOTOR_ERR("g_chip null\n");
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);
	}

	if ((g_chip->manual2auto_up_switch == 1)
			|| (g_chip->manual2auto_down_switch == 1)) {
		manual2auto_switch = 1;
	}

	return sprintf(buf, "%d\n", manual2auto_switch);
}

static ssize_t motor_manual2auto_switch_store(struct device *pdev,
		struct device_attribute *attr,
		const char *buff, size_t count)
{
	int data[1] = {0};

	if (!g_chip) {
		MOTOR_ERR("g_chip null\n");
		return count;
	}

	if (sscanf(buff, "%d", &data[0]) == 1) {
		g_chip->manual2auto_down_switch = data[0];
		/*g_chip->manual2auto_up_switch = data[0];*/
		MOTOR_ERR("data[%d]\n", data[0]);

		if (data[0] == 1) {
			if (g_chip->position == UP_STATE) {
				oplus_dhall_update_threshold(DHALL_1, MID_STATE,
							    g_chip->cali_data.irq_pos_hall1);

			} else if (g_chip->position == DOWN_STATE) {
				oplus_dhall_update_threshold(DHALL_0, MID_STATE,
							    g_chip->cali_data.irq_pos_hall0);
			}

			g_chip->is_skip_pos_check = 0;
			in_cali = false;

		} else {
			in_cali = true;
			g_chip->is_skip_pos_check = 1;
		}

	} else {
		MOTOR_ERR("fail\n");
	}

	return count;
}

static ssize_t dhall_stop_val_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	if (!g_chip) {
		MOTOR_ERR("g_chip null\n");
		return snprintf(buf, PAGE_SIZE, "%d\n%d\n", -1, -1);
	}

	return sprintf(buf, "%d,%d\n", g_chip->stop_rang_a, g_chip->stop_rang_b);
}

static ssize_t dhall_stop_val_store(struct device *pdev,
				    struct device_attribute *attr, const char *buff, size_t count)
{
	int stop_value[2] = {0};

	if (sscanf(buff, "%d,%d,", &stop_value[0], &stop_value[1]) == 2) {
		g_chip->stop_rang_a = stop_value[0];
		g_chip->stop_rang_b = stop_value[1];

	} else {
		MOTOR_ERR("fail\n");
	}

	MOTOR_LOG("stop_rang_a = %d, stop_rang_a = %d.\n", g_chip->stop_rang_a,
		  g_chip->stop_rang_b);
	return count;
}

static ssize_t dhall_in_calibration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	if (!g_chip) {
		MOTOR_ERR("g_chip null\n");
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);
	}

	return sprintf(buf, "%d\n", g_chip->is_in_calibration);
}

static ssize_t dhall_in_calibration_store(struct device *pdev,
		struct device_attribute *attr, const char *buff, size_t count)
{
	int in_calibration = 0;

	if (!g_chip) {
		return count;
	}

	if (sscanf(buff, "%d", &in_calibration) == 1) {
		/*check if it is defaut parameter*/
		if (in_calibration == 1) {
			MOTOR_LOG("in calibration \n");
			g_chip->is_in_calibration = true;

		} else {
			MOTOR_LOG("out of calibration \n");
			g_chip->is_in_calibration = false;
		}

	} else {
		g_chip->is_in_calibration = false;
		MOTOR_LOG("invalid cmd. out of calibration \n");
	}

	return count;
}

static ssize_t dhall_calibration_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	if (!g_chip) {
		MOTOR_ERR("g_chip null\n");
		return snprintf(buf, PAGE_SIZE, "%d\n%d\n", -1, -1);
	}

	return sprintf(buf, "%d,%d,%d,%d,%d,%d\n",
		       g_chip->cali_data.irq_pos_hall0 * g_chip->sign,
		       g_chip->cali_data.irq_pos_hall1 * g_chip->sign,
		       g_chip->cali_data.down_retard_hall0 * g_chip->sign,
		       g_chip->cali_data.down_retard_hall1 * g_chip->sign,
		       g_chip->cali_data.up_retard_hall0 * g_chip->sign,
		       g_chip->cali_data.up_retard_hall1 * g_chip->sign);
}

static ssize_t dhall_calibration_store(struct device *pdev,
				       struct device_attribute *attr,
				       const char *buff, size_t count)
{
	int data[6] = {0};
	static bool first_check = true;

	if (!g_chip) {
		MOTOR_ERR("g_chip null\n");
		return count;
	}

	if (sscanf(buff, "%d,%d,%d,%d,%d,%d", &data[0], &data[1], &data[2], &data[3],
			&data[4], &data[5]) == 6) {
		if ((data[0] == 0) && (data[1] == 0) && (data[2] == 0) &&
				(data[3] == 0) && (data[4] == 0) && (data[5] == 0)) {
			first_check = false;
			MOTOR_ERR("invalid data\n");
			return count;
		}

		/*determine sign first*/
		oplus_change_dhall_sign(data[2], data[5]);

		if (g_chip->retard_range_end) {
			g_chip->cali_data.irq_pos_hall0 = data[0] * g_chip->sign -
							  (g_chip->retard_range_end - 5);
			g_chip->cali_data.irq_pos_hall1 = data[1] * g_chip->sign -
							  (g_chip->retard_range_end - 5);

		} else {
			g_chip->cali_data.irq_pos_hall0 = data[0] * g_chip->sign;
			g_chip->cali_data.irq_pos_hall1 = data[1] * g_chip->sign;
		}

		g_chip->cali_data.irq_pos_hall0_bak = data[0] * g_chip->sign;
		g_chip->cali_data.irq_pos_hall1_bak = data[1] * g_chip->sign;
		g_chip->cali_data.down_retard_hall0 = data[2] * g_chip->sign;
		g_chip->cali_data.down_retard_hall1 = data[3] * g_chip->sign;
		g_chip->cali_data.up_retard_hall0 = data[4] * g_chip->sign;
		g_chip->cali_data.up_retard_hall1 = data[5] * g_chip->sign;

		MOTOR_ERR("data[%d %d %d %d %d %d]\n",
			  g_chip->cali_data.irq_pos_hall0,
			  g_chip->cali_data.irq_pos_hall1,
			  g_chip->cali_data.down_retard_hall0,
			  g_chip->cali_data.down_retard_hall1,
			  g_chip->cali_data.up_retard_hall0,
			  g_chip->cali_data.up_retard_hall1);

		if (first_check) {
			first_check = false;
			oplus_motor_reset_check(g_chip);
		}

#ifdef CONFIG_MOTOR_DETECT_QOS

		if (!motor_qos_state) {
			pm_qos_add_request(&motor_qos_req, PM_QOS_CPU_DMA_LATENCY, motor_qos_value);
			motor_qos_state = 1;
			MOTOR_ERR("motor add qos request.\n");
		}

#endif

	} else {
		MOTOR_ERR("fail\n");
	}

	return count;
}

static ssize_t dhall_distance_store(struct device *pdev,
				    struct device_attribute *attr,
				    const char *buff, size_t count)
{
	int up_distance = 0, down_distance = 0;

	if (sscanf(buff, "%d,%d", &up_distance, &down_distance) == 2) {
		g_chip->pwm_param.normal.speed_up.l = up_distance;
		g_chip->pwm_param.normal.speed_down.l  = down_distance;
		oplus_set_md_mode_para(g_chip->pwm_param.md_mode);
	}

	MOTOR_LOG("up/down distance <%d, %d>\n",
		  g_chip->pwm_param.normal.speed_up.l,
		  g_chip->pwm_param.normal.speed_down.l);
	return count;
}

static ssize_t dhall_stop_range_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	if (!g_chip) {
		MOTOR_ERR("g_chip null\n");
		return snprintf(buf, PAGE_SIZE, "up range <%d, %d> down range <%d, %d>", 0, 0,
				0, 0);
	}

	return sprintf(buf, "%d,%d,%d,%d\n",
		       g_chip->stop.neg[0], g_chip->stop.pos[0],
		       g_chip->stop.neg[1], g_chip->stop.pos[1]);
}

static ssize_t dhall_stop_range_store(struct device *pdev,
				      struct device_attribute *attr,
				      const char *buff, size_t count)
{
	int neg_value[2] = {0}, pos_value[2] = {0};

	if (sscanf(buff, "-%d,%d,-%d,%d", &neg_value[0], &pos_value[0],
			&neg_value[1], &pos_value[1]) == 4) {
		g_chip->stop.neg[0] = -neg_value[0];
		g_chip->stop.pos[0] = pos_value[0];
		g_chip->stop.neg[1] = -neg_value[1];
		g_chip->stop.pos[1] = pos_value[1];
	}

	MOTOR_LOG("up range <%d, %d> down range <%d, %d>\n", g_chip->stop.neg[0],
		  g_chip->stop.pos[0],
		  g_chip->stop.neg[1], g_chip->stop.pos[1]);
	return count;
}

static ssize_t dhall_retard_range_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	if (!g_chip) {
		MOTOR_ERR("g_chip null\n");
		return snprintf(buf, PAGE_SIZE, "up range <%d, %d> ", 0, 0);
	}

	return sprintf(buf, "%d,%d retard_range_end=%d\n", g_chip->retard_range,
		       g_chip->retard_sign, g_chip->retard_range_end);
}

static ssize_t dhall_retard_range_store(struct device *pdev,
					struct device_attribute *attr,
					const char *buff, size_t count)
{
	int neg_value[2] = {0};

	if (sscanf(buff, "%d,%d", &neg_value[0], &neg_value[1])) {
		g_chip->retard_range = neg_value[0];
		g_chip->retard_sign = neg_value[1];
	}

	MOTOR_LOG("retard_range %d, retard_sign %d, cali_data.up_retard_hall1 %d\n",
		  g_chip->retard_range, g_chip->retard_sign, g_chip->cali_data.up_retard_hall1);

	if (g_chip->retard_sign) {
		g_chip->retard_range_end = g_chip->retard_range * 5;

	} else {
		g_chip->retard_range_end = -g_chip->retard_range * 5;
	}

	MOTOR_LOG("retard_range %d, retard_sign %d, cali_data.up_retard_hall1 %d\n",
		  g_chip->retard_range, g_chip->retard_sign, g_chip->cali_data.up_retard_hall1);
	return count;
}

static ssize_t motor_detect_delay_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	if (!g_chip) {
		MOTOR_ERR("g_chip null\n");
		return snprintf(buf, PAGE_SIZE, "%d\n%d\n%d\n", -1, -1, -1);
	}

	return sprintf(buf, "delay <%d, %d, %d> us\n",
		       g_chip->pwm_param.normal.delay,
		       g_chip->pwm_param.normal.up_brake_delay,
		       g_chip->pwm_param.normal.down_brake_delay);
}

static ssize_t motor_detect_delay_store(struct device *pdev,
					struct device_attribute *attr,
					const char *buff, size_t count)
{
	int delay[3] = {0};

	if (sscanf(buff, "%d,%d,%d", &delay[0], &delay[1], &delay[2]) == 3) {
		g_chip->pwm_param.normal.delay = delay[0];
		g_chip->pwm_param.normal.up_brake_delay = delay[1];
		g_chip->pwm_param.normal.down_brake_delay = delay[2];
	}

	MOTOR_LOG("delay <%d, %d, %d>\n",
		  g_chip->pwm_param.normal.delay,
		  g_chip->pwm_param.normal.up_brake_delay,
		  g_chip->pwm_param.normal.down_brake_delay);
	return count;
}

static ssize_t motor_terminal_offset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	if (!g_chip) {
		MOTOR_ERR("g_chip null\n");
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);
	}

	return sprintf(buf, "<%d,%d>\n", g_chip->stop.offset[0],
		       g_chip->stop.offset[1]);
}

static ssize_t motor_terminal_offset_store(struct device *pdev,
		struct device_attribute *attr,
		const char *buff, size_t count)
{
	int offset[2] = {0};

	if (!g_chip) {
		MOTOR_ERR("g_chip null\n");
		return count;
	}

	if (sscanf(buff, "%d,%d", &offset[0], &offset[1]) == 2) {
		g_chip->stop.offset[0] = offset[0];
		g_chip->stop.offset[1] = offset[1];
	}

	MOTOR_LOG("offset <%d,%d>\n", g_chip->stop.offset[0], g_chip->stop.offset[1]);
	return count;
}

static ssize_t motor_stop_in_advance_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	if (!g_chip) {
		MOTOR_ERR("g_chip null\n");
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);
	}

	return sprintf(buf, "%d \n", g_chip->stop_in_advance);
}

static ssize_t motor_stop_in_advance_store(struct device *pdev,
		struct device_attribute *attr,
		const char *buff, size_t count)
{
	int val = 0;

	if (!g_chip) {
		MOTOR_ERR("g_chip null\n");
		return count;
	}

	if (sscanf(buff, "%d", &val) == 1) {
		if (val) {
			g_chip->stop_in_advance = true;

		} else {
			g_chip->stop_in_advance = false;
		}
	}

	MOTOR_LOG("stop_in_advance %d\n", g_chip->stop_in_advance);
	return count;
}


static ssize_t small_irq_later_speed_down_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	if (!g_chip) {
		MOTOR_ERR("g_chip null\n");
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);
	}

	return sprintf(buf, "<%d,%d,%d>\n", g_chip->small_irq, g_chip->later_speed_down,
		       g_chip->later_speed_down_stop_offset);
}

static ssize_t small_irq_later_speed_down_store(struct device *pdev,
		struct device_attribute *attr,
		const char *buff, size_t count)
{
	int tmp[3] = {0};

	if (!g_chip) {
		MOTOR_ERR("g_chip null\n");
		return count;
	}

	if (sscanf(buff, "%d,%d,%d", &tmp[0], &tmp[1], &tmp[2]) == 3) {
		g_chip->small_irq = tmp[0];
		g_chip->later_speed_down = tmp[1];
		g_chip->later_speed_down_stop_offset = tmp[2];
	}

	MOTOR_LOG("small_irq_later_speed_down <%d,%d,%d>\n", g_chip->small_irq,
		  g_chip->later_speed_down, g_chip->later_speed_down_stop_offset);
	return count;
}

static DEVICE_ATTR(direction, S_IRUGO | S_IWUSR, motor_direction_show,
		   motor_direction_store);
static DEVICE_ATTR(speed,   S_IRUGO | S_IWUSR, motor_speed_show,
		   motor_speed_store);
static DEVICE_ATTR(mode,	S_IRUGO | S_IWUSR, motor_mode_show, motor_mode_store);
static DEVICE_ATTR(enable,  S_IRUGO | S_IWUSR, NULL, motor_enable_store);
static DEVICE_ATTR(move_state,	  S_IRUGO | S_IWUSR, motor_move_state_show,
		   NULL);
static DEVICE_ATTR(config,   S_IRUGO | S_IWUSR, motor_all_config_show, NULL);
static DEVICE_ATTR(stop_range,	 S_IRUGO | S_IWUSR, dhall_stop_range_show,
		   dhall_stop_range_store);
static DEVICE_ATTR(distance,	 S_IRUGO | S_IWUSR, NULL, dhall_distance_store);
static DEVICE_ATTR(sw_switch,	 S_IRUGO | S_IWUSR, motor_sw_switch_show,
		   motor_sw_switch_store);
static DEVICE_ATTR(position,   S_IRUGO | S_IWUSR, motor_position_show, NULL);
static DEVICE_ATTR(manual2auto_switch,	 S_IRUGO | S_IWUSR,
		   motor_manual2auto_switch_show, motor_manual2auto_switch_store);
static DEVICE_ATTR(motor_test,	 S_IRUGO | S_IWUSR, motor_test_show,
		   motor_test_store);
static DEVICE_ATTR(force_move, S_IRUGO | S_IWUSR, NULL, motor_force_move_store);
static DEVICE_ATTR(delay, S_IRUGO | S_IWUSR, motor_detect_delay_show,
		   motor_detect_delay_store);
static DEVICE_ATTR(terminal_offset, S_IRUGO | S_IWUSR,
		   motor_terminal_offset_show, motor_terminal_offset_store);
static DEVICE_ATTR(hall_data,	S_IRUGO | S_IWUSR, dhall_data_show,
		   dhall_data_store);
static DEVICE_ATTR(hall_reg,   S_IRUGO | S_IWUSR, dhall_all_reg_show, NULL);
static DEVICE_ATTR(hall_max_data,   S_IRUGO | S_IWUSR, dhall_max_data_show,
		   NULL);
static DEVICE_ATTR(hall_irq_count,   S_IRUGO | S_IWUSR, dhall_irq_count_show,
		   NULL);
static DEVICE_ATTR(hall_calibration,   S_IRUGO | S_IWUSR,
		   dhall_calibration_show, dhall_calibration_store);
static DEVICE_ATTR(retard_range,   S_IRUGO | S_IWUSR, dhall_retard_range_show,
		   dhall_retard_range_store);
static DEVICE_ATTR(hall_detect_switch,	 S_IRUGO | S_IWUSR,
		   dhall_detect_switch_show, dhall_detect_switch_store);
static DEVICE_ATTR(hall_get_data,	 S_IRUGO | S_IWUSR, dhall_get_data_show,
		   NULL);
static DEVICE_ATTR(in_calibration,   S_IRUGO | S_IWUSR,
		   dhall_in_calibration_show, dhall_in_calibration_store);
static DEVICE_ATTR(stop_in_advance,   S_IRUGO | S_IWUSR,
		   motor_stop_in_advance_show, motor_stop_in_advance_store);
static DEVICE_ATTR(stop_val,   S_IRUGO | S_IWUSR, dhall_stop_val_show,
		   dhall_stop_val_store);
static DEVICE_ATTR(small_irq_later_speed_down,   S_IRUGO | S_IWUSR,
		   small_irq_later_speed_down_show, small_irq_later_speed_down_store);

static struct attribute *motor_class_attrs[] = {
	&dev_attr_direction.attr,
	&dev_attr_speed.attr,
	&dev_attr_mode.attr,
	&dev_attr_enable.attr,
	&dev_attr_move_state.attr,
	&dev_attr_config.attr,
	&dev_attr_stop_range.attr,
	&dev_attr_distance.attr,
	&dev_attr_sw_switch.attr,
	&dev_attr_position.attr,
	&dev_attr_manual2auto_switch.attr,
	&dev_attr_motor_test.attr,
	&dev_attr_force_move.attr,
	&dev_attr_delay.attr,
	&dev_attr_terminal_offset.attr,
	&dev_attr_hall_data.attr,
	&dev_attr_hall_max_data.attr,
	&dev_attr_hall_reg.attr,
	&dev_attr_hall_irq_count.attr,
	&dev_attr_hall_calibration.attr,
	&dev_attr_hall_detect_switch.attr,
	&dev_attr_hall_get_data.attr,
	&dev_attr_in_calibration.attr,
	&dev_attr_retard_range.attr,
	&dev_attr_stop_in_advance.attr,
	&dev_attr_stop_val.attr,
	&dev_attr_small_irq_later_speed_down.attr,
	NULL
};

#ifdef CONFIG_MOTOR_CLASS_INTERFACE
ATTRIBUTE_GROUPS(motor_class);
#else
static struct attribute_group __attribute_group = {
	.attrs = motor_class_attrs,
};
#endif

/*-----misc-------*/
static int motor_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

static int motor_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static long motor_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	int data = 0;
	short raw = 0;
	void __user *u_arg = (void __user *)arg;
	cali_data_t  cali_data;
	dhall_data_t dhall_data;
	short hall0_val, hall1_val;

	if (NULL == g_chip) {
		MOTOR_ERR("g_chip null!!\n");
		return -EFAULT;
	}

	if (_IOC_DIR(cmd) & _IOC_READ) {
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));

	} else if (_IOC_DIR(cmd) & _IOC_WRITE) {
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}

	if (err) {
		MOTOR_ERR("access error: %08X \n", cmd);
		return -EFAULT;
	}

	switch (cmd) {
	case MOTOR_IOCTL_START_MOTOR:
		if (copy_from_user(&data, u_arg, sizeof(data))) {
			MOTOR_ERR("start motor fail \n");
			return -EFAULT;
		}

		if (data) {
			oplus_motor_start(data, g_chip->md_dir);
		}

		break;

	case MOTOR_IOCTL_STOP_MOTOR:
		if (copy_from_user(&data, u_arg, sizeof(data))) {
			MOTOR_ERR("stop motor fail \n");
			return -EFAULT;
		}

		if (data) {
			oplus_motor_stop();
		}

		break;

	case MOTOR_IOCTL_MOTOR_UPWARD:
		if (copy_from_user(&data, u_arg, sizeof(data))) {
			MOTOR_ERR("motor upward fail \n");
			return -EFAULT;
		}

		if (data) {
			oplus_motor_upward();
		}

		break;

	case MOTOR_IOCTL_MOTOR_DOWNWARD:
		if (copy_from_user(&data, u_arg, sizeof(data))) {
			MOTOR_ERR("motor downward fail \n");
			return -EFAULT;
		}

		if (data) {
			oplus_motor_downward(USER_START);
		}

		break;

	case MOTOR_IOCTL_GET_POSITION:
		if (u_arg == NULL) {
			MOTOR_ERR("get position fail0 \n");
			break;
		}

		data = g_chip->position;

		if (copy_to_user(u_arg, &data, sizeof(data))) {
			MOTOR_ERR("get position fail1 \n");
			return -EFAULT;
		}

		break;

	case MOTOR_IOCTL_SET_DIRECTION:
		if (copy_from_user(&data, u_arg, sizeof(data))) {
			MOTOR_ERR("set direction fail \n");
			return -EFAULT;
		}

		oplus_set_direction_para(data);
		break;

	case MOTOR_IOCTL_SET_SPEED:
		if (copy_from_user(&data, u_arg, sizeof(data))) {
			MOTOR_ERR("set speed fail \n");
			return -EFAULT;
		}

		oplus_update_motor_speed(data);
		break;

	case MOTOR_IOCTL_SET_DELAY:
	case MOTOR_IOCTL_GET_DHALL_DATA:
		if (u_arg == NULL) {
			MOTOR_ERR("get dhall data fail0 \n");
			break;
		}

		oplus_dhall_get_data(DHALL_0, &hall0_val, true);
		oplus_dhall_get_data(DHALL_1, &hall1_val, true);
		dhall_data.data0 = hall0_val;
		dhall_data.data1 = hall1_val;

		if (copy_to_user(u_arg, &dhall_data, sizeof(dhall_data))) {
			MOTOR_ERR("get position fail1 \n");
			return -EFAULT;
		}

	case MOTOR_IOCTL_SET_CALIBRATION:
		if (copy_from_user(&cali_data, u_arg, sizeof(cali_data))) {
			MOTOR_ERR("set calibration fail \n");
			return -EFAULT;
		}

		g_chip->cali_data.irq_pos_hall0	 = cali_data.irq_pos_hall0;
		g_chip->cali_data.irq_pos_hall1	 = cali_data.irq_pos_hall1;
		g_chip->cali_data.down_retard_hall0	= cali_data.down_retard_hall0;
		g_chip->cali_data.down_retard_hall1 = cali_data.down_retard_hall1;
		g_chip->cali_data.up_retard_hall0 = cali_data.up_retard_hall0;
		g_chip->cali_data.up_retard_hall1 = cali_data.up_retard_hall1;
		break;

	case MOTOR_IOCTL_GET_INTERRUPT:
	default:
		return -ENOTTY;
	}

	return err;
}

#ifdef CONFIG_MOTOR_CLASS_INTERFACE
static struct class motor_class = {
	.name = "motor",
	.owner = THIS_MODULE,
	.class_groups = motor_class_groups,
};
#endif

static struct file_operations motor_fops = {
	.owner = THIS_MODULE,
	.open = motor_open,
	.unlocked_ioctl = motor_ioctl,
	.release = motor_release,
};

static struct miscdevice motor_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "motor",
	.fops = &motor_fops,
};

static int motor_platform_probe(struct platform_device *pdev)
{
	struct oplus_motor_chip *chip = NULL;
	int err = 0;

	MOTOR_LOG("call \n");

	if (!g_chip) {
		chip = kzalloc(sizeof(struct oplus_motor_chip), GFP_KERNEL);

		if (!chip) {
			MOTOR_ERR("kzalloc err \n");
			return -ENOMEM;
		}

		g_chip = chip;

	} else {
		chip = g_chip;
	}

	mutex_init(&chip->power_mutex);

	chip->dev = &pdev->dev;

	if (!chip->dhall_ops) {
		MOTOR_ERR("no dhall available \n");
		goto fail;
	}

	if (!chip->motor_ops) {
		MOTOR_ERR("no motor driver available \n");
		goto fail;
	}

#ifdef CONFIG_MOTOR_CLASS_INTERFACE
	err = class_register(&motor_class);
	if (err) {
		MOTOR_ERR("class register was failed(%d) \n", err);
		goto sysfs_create_fail;
	}
#else
	err = sysfs_create_group(&pdev->dev.kobj, &__attribute_group);

	if (err) {
		MOTOR_ERR("sysfs_create_group was failed(%d) \n", err);
		goto sysfs_create_fail;
	}
#endif

	err = misc_register(&motor_device);

	if (err) {
		MOTOR_ERR("misc_register was failed(%d)", err);
		goto misc_fail;
	}

	err = oplus_input_dev_init(chip);

	if (err < 0) {
		MOTOR_ERR("oplus_input_dev_init fail \n");
		goto input_fail;
	}

	chip->manual2auto_wq = create_singlethread_workqueue("manual2auto_wq");

	if (!chip->manual2auto_wq) {
		MOTOR_ERR("manual2auto_wq NULL \n");
		goto create_workqueue_fail;
	}

	oplus_motor_parse_dts(chip);
	oplus_parameter_init(chip);

	if (chip->boot_mode != MOTOR_NORMAL_MODE) {
		MOTOR_LOG("boot_mode is %d ,not support motor \n", chip->boot_mode);
		goto boot_mode_fail;
	}

	if (chip->is_support_ffd) {
		oplus_motor_free_fall_register(chip);
	}

	hrtimer_init(&chip->stop_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	chip->stop_timer.function = motor_stop_timer_func;

	hrtimer_init(&chip->speed_up_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	chip->speed_up_timer.function = motor_speed_up_timer_func;

	hrtimer_init(&chip->force_move_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	chip->force_move_timer.function = motor_force_move_stop_timer_func;

	INIT_WORK(&chip->motor_work, motor_run_work);
	INIT_WORK(&chip->force_work, force_move_work);
	INIT_WORK(&chip->manual_position_work, manual_position_detect_work);

	alarm_init(&chip->reset_timer, ALARM_BOOTTIME, motor_reset_timer_func);

	INIT_DELAYED_WORK(&chip->detect_work, position_detect_work);
	INIT_DELAYED_WORK(&chip->up_work, manual_to_auto_up_work);
	INIT_DELAYED_WORK(&chip->down_work, manual_to_auto_down_work);

	chip->fb_notify.notifier_call = fb_notifier_callback;
#ifdef CONFIG_DRM_MSM
	msm_drm_register_client(&chip->fb_notify);
#else
	fb_register_client(&chip->fb_notify);
#endif

	chip->up_data_buf = kzalloc(MAX_BUF_LEN * (sizeof(uint8_t)), GFP_KERNEL);

	if (!chip->up_data_buf) {
		MOTOR_ERR("up_data_buf kzalloc error\n");
	}

	chip->down_data_buf = kzalloc(MAX_BUF_LEN * (sizeof(uint8_t)), GFP_KERNEL);

	if (!chip->down_data_buf) {
		MOTOR_ERR("down_data_buf kzalloc error\n");

		if (chip->up_data_buf != NULL) {
			kfree(chip->up_data_buf);
			chip->up_data_buf = NULL;
		}
	}

	MOTOR_LOG("success. \n");
	return 0;

boot_mode_fail:
	destroy_workqueue(chip->manual2auto_wq);
create_workqueue_fail:
	input_unregister_device(chip->i_dev);
input_fail:
	misc_deregister(&motor_device);
misc_fail:
#ifdef CONFIG_MOTOR_CLASS_INTERFACE
	class_unregister(&motor_class);
#else
	sysfs_remove_group(&pdev->dev.kobj, &__attribute_group);
#endif
sysfs_create_fail:
fail:
	kfree(chip);
	g_chip = NULL;
	MOTOR_LOG("fail \n");
	return -EINVAL;
}

static int motor_platform_remove(struct platform_device *pdev)
{
	if (g_chip) {
#ifdef CONFIG_MOTOR_CLASS_INTERFACE
		class_unregister(&motor_class);
#else
		sysfs_remove_group(&pdev->dev.kobj, &__attribute_group);
#endif
		misc_deregister(&motor_device);
		input_unregister_device(g_chip->i_dev);
		input_free_device(g_chip->i_dev);

		if (g_chip->up_data_buf) {
			kfree(g_chip->up_data_buf);
			g_chip->up_data_buf = NULL;
		}

		if (g_chip->down_data_buf) {
			kfree(g_chip->down_data_buf);
			g_chip->down_data_buf = NULL;
		}

		kfree(g_chip);
		g_chip = NULL;
	}

	return 0;
}

static int motor_platform_suspend(struct platform_device *pdev,
				  pm_message_t state)
{
	if (g_chip) {
		atomic_set(&g_chip->in_suspend, 1);
	}

	return 0;
}

static int motor_platform_resume(struct platform_device *pdev)
{
	if (g_chip) {
		atomic_set(&g_chip->in_suspend, 0);
	}

	return 0;
}

static void motor_platform_shutdown(struct platform_device *pdev)
{
	if (g_chip) {
		oplus_motor_reset_check(g_chip);
		g_chip->shutting = true;

		if (g_chip->motor_switch && g_chip->motor_enable) {
			wait_for_completion_timeout(&shutdown_comp, MOTOR_SHUTDOWN_TIMEOUT);
		}
	}

	return;
}

static const struct of_device_id of_motor_match[] = {
	{ .compatible = "oplus-motor"},
	{},
};

static int motor_pm_resume(struct device *dev)
{
	MOTOR_LOG("call \n");
	return 0;
}

static int motor_pm_suspend(struct device *dev)
{
	MOTOR_LOG("call \n");
	oplus_motor_set_power(DEEP_SLEEP, MOTOR_POWER_IGNORE);

	return 0;
}

static const struct dev_pm_ops motor_pm_ops = {
#ifdef CONFIG_FB
	.suspend = motor_pm_suspend,
	.resume = motor_pm_resume,
#endif
}

MODULE_DEVICE_TABLE(of, of_motor_match);

static struct platform_driver motor_platform_driver = {
	.probe		= motor_platform_probe,
	.remove		= motor_platform_remove,
	.suspend	= motor_platform_suspend,
	.resume		= motor_platform_resume,
	.shutdown   = motor_platform_shutdown,
	.driver		= {
		.name	= "oplus_motor",
		.of_match_table = of_motor_match,
		.pm		= &motor_pm_ops,
	},
};

static int __init motor_platform_init(void)
{
	MOTOR_LOG("call \n");

	platform_driver_register(&motor_platform_driver);
	return 0;
}

late_initcall(motor_platform_init);
MODULE_DESCRIPTION("camera motor platform driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("mofei");
