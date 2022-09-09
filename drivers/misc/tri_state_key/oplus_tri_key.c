// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 * File: oplus_tri_key.c
 *
 * Description:
 *      Definitions for m1120 tri_state_key data process.
 *
 * Version: 1.0
 */

#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/hrtimer.h>
#include <linux/alarmtimer.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/extcon.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/time.h>


#include <linux/string.h>
#include <linux/version.h>

#include "oplus_tri_key.h"
#include "../../extcon/extcon.h"

#define TRI_KEY_DEVICE "oplus,hall_tri_state_key"
#define TRI_KEY_TAG                  "[tri_state_key] "
#define TRI_KEY_ERR(fmt, args...)\
	pr_info(TRI_KEY_TAG" %s : "fmt, __func__, ##args)
#define TRI_KEY_LOG(fmt, args...)\
	pr_info(TRI_KEY_TAG" %s : "fmt, __func__, ##args)
#define TRI_KEY_DEBUG(fmt, args...)\
	do {\
		if (tri_key_debug == LEVEL_DEBUG)\
			pr_info(TRI_KEY_TAG " %s: " fmt, __func__, ##args);\
	} while (0)
enum {
	MODE_UNKNOWN,
	MODE_MUTE,
	MODE_DO_NOT_DISTURB,
	MODE_NORMAL,
	MODE_MAX_NUM
	} tri_mode;


unsigned int tristate_extcon_tab[] = {
		MODE_MUTE,
		MODE_DO_NOT_DISTURB,
		MODE_NORMAL,
	};

static struct hrtimer tri_key_timer;
struct work_struct tri_key_timeout_work;



static struct extcon_dev_data *g_the_chip;
static struct extcon_dev_data *g_hall_dev;
static int last_d0;
static int last_d1;
static int last_position = -1;
static int last_interf = -1;
static int interf_count;
static int time = 1;
unsigned int tri_key_debug;

static short tol0 = 15;
static short tol2 = 40;
static short up_mid_tol = 15;
static short up_tolerance = 15;
static short down_tolerance = 15;
static short mid_down_tol = 15;
static short up_mid_distance;
static short mid_down_distance;
static short calib_UpValueSum, calib_MdValueSum, calib_DnValueSum;
static short calib_UpValueMin, calib_MdValueMin, calib_DnValueMin;
static short calib_dnHall_UM_distance, calib_dnHall_MD_distance;
static short calib_upHall_UM_distance, calib_upHall_MD_distance;
static short calib_upHall_UD_distance, calib_dnHall_UD_distance;

void oplus_hall_disable_irq(bool enable)
{
	oplus_hall_clear_irq(DHALL_0);
	oplus_hall_clear_irq(DHALL_1);

	g_the_chip->dhall_down_ops->enable_irq(enable);
	g_the_chip->dhall_up_ops->enable_irq(enable);
}

int oplus_hall_enable_irq(unsigned int id, bool enable)
{
	if (!g_the_chip)
		return -EINVAL;

	switch (id) {
	case DHALL_0:
		if (!g_the_chip->dhall_down_ops ||
			!g_the_chip->dhall_down_ops->enable_irq)
			TRI_KEY_ERR("enable hall0 irq error\n");
		else {
			oplus_hall_clear_irq(DHALL_0);
			oplus_hall_clear_irq(DHALL_1);
			return g_the_chip->dhall_down_ops->enable_irq(enable);
		}
	case DHALL_1:
		if (!g_the_chip->dhall_up_ops ||
			!g_the_chip->dhall_up_ops->enable_irq)
			TRI_KEY_ERR("enable hall1 irq error\n");
		else {
			oplus_hall_clear_irq(DHALL_0);
			oplus_hall_clear_irq(DHALL_1);
			return g_the_chip->dhall_up_ops->enable_irq(enable);
		}
	default:
		TRI_KEY_ERR("id : %d is not correct\n", id);
		return -EINVAL;
	}

	return -EINVAL;
}

int oplus_hall_clear_irq(unsigned int id)
{
	if (!g_the_chip)
		return -EINVAL;

	TRI_KEY_DEBUG("dhall_clear_irq\n");
	switch (id) {
	case DHALL_0:
		if (!g_the_chip->dhall_down_ops ||
			!g_the_chip->dhall_down_ops->enable_irq)
			return -EINVAL;
		else
			return g_the_chip->dhall_down_ops->clear_irq();

	case DHALL_1:
		if (!g_the_chip->dhall_up_ops ||
			!g_the_chip->dhall_up_ops->enable_irq)
			return -EINVAL;
		else
			return g_the_chip->dhall_up_ops->clear_irq();
	default:
		TRI_KEY_ERR("id : %d is not correct\n", id);
		return -EINVAL;
	}

	return -EINVAL;
}

int oplus_hall_get_data(unsigned int id)
{
	if (!g_the_chip)
		return -EINVAL;

	switch (id) {
	case DHALL_0:
		if (!g_the_chip->dhall_down_ops ||
				!g_the_chip->dhall_down_ops->get_data)
			TRI_KEY_ERR("get hall0 data error\n");
		else {
			g_the_chip->dhall_down_ops->get_data(
				&g_the_chip->dhall_data0);
			return true;
		}
	case DHALL_1:
		if (!g_the_chip->dhall_up_ops ||
				!g_the_chip->dhall_up_ops->get_data)
			TRI_KEY_ERR("get hall1 data error\n");
		else {
			g_the_chip->dhall_up_ops->get_data(
				&g_the_chip->dhall_data1);
			return true;
		}
	default:
		TRI_KEY_ERR("id : %d is not correct\n", id);
		return -EINVAL;
	}
}

bool oplus_hall_update_threshold(unsigned int id, int position,
					short lowthd, short highthd)
{
	if (!g_the_chip)
		return -EINVAL;

	switch (id) {
	case DHALL_0:
		if (!g_the_chip->dhall_down_ops ||
				!g_the_chip->dhall_down_ops->update_threshold)
			TRI_KEY_ERR("update hall0 threshold error\n");
		else {
			g_the_chip->dhall_down_ops->update_threshold(position,
					lowthd, highthd);
			return true;
		}
	case DHALL_1:
		if (!g_the_chip->dhall_up_ops ||
			!g_the_chip->dhall_up_ops->update_threshold)
			TRI_KEY_ERR("update hall1 threshold error\n");
		else {
			g_the_chip->dhall_up_ops->update_threshold(position,
					lowthd, highthd);
			return true;
		}
	default:
		TRI_KEY_ERR("id : %d is not correct\n", id);
		return -EINVAL;
	}
}

int oplus_hall_set_detection_mode(unsigned int id, u8 mode)
{
	if (!g_the_chip)
		return -EINVAL;

	switch (id) {
	case DHALL_0:
		if (!g_the_chip->dhall_down_ops ||
				!g_the_chip->dhall_down_ops->set_detection_mode)
			TRI_KEY_ERR("set hall 0 error\n");
		else {
			g_the_chip->dhall_down_ops->set_detection_mode(mode);
			return true;
		}
	case DHALL_1:
		if (!g_the_chip->dhall_up_ops ||
				!g_the_chip->dhall_up_ops->set_detection_mode)
			TRI_KEY_ERR("set error\n");
		else {
			g_the_chip->dhall_up_ops->set_detection_mode(mode);
			return true;
		}
	default:
		TRI_KEY_ERR("id : %d is not correct\n", id);
		return -EINVAL;
	}
}

int oplus_hall_get_irq_state(unsigned int id)
{
	if (!g_the_chip)
		return -EINVAL;

	switch (id) {
	case DHALL_0:
		if (!g_the_chip->dhall_down_ops ||
				!g_the_chip->dhall_down_ops->get_irq_state)
			return -EINVAL;
		else
			return g_the_chip->dhall_down_ops->get_irq_state();
	case DHALL_1:
		if (!g_the_chip->dhall_up_ops ||
				!g_the_chip->dhall_up_ops->get_irq_state)
			return -EINVAL;
		else
			return g_the_chip->dhall_up_ops->get_irq_state();
	default:
		TRI_KEY_ERR("id : %d is not correct\n", id);
		return -EINVAL;
	}
}



void oplus_hall_dump_regs(unsigned int id, u8 *buf)
{
	if (!g_the_chip)
		return;

	switch (id) {
	case DHALL_0:
		if (!g_the_chip->dhall_down_ops ||
				!g_the_chip->dhall_down_ops->dump_regs)
			TRI_KEY_ERR("dump hall0 error\n");
		else
			g_the_chip->dhall_down_ops->dump_regs(buf);
		break;
	case DHALL_1:
		if (!g_the_chip->dhall_up_ops ||
				!g_the_chip->dhall_up_ops->dump_regs)
			TRI_KEY_ERR("dump hall1 error\n");
		else
			g_the_chip->dhall_up_ops->dump_regs(buf);
		break;
	default:
		TRI_KEY_ERR("id : %d is not correct\n", id);
		return;
	}
}

int oplus_hall_set_reg(unsigned int id, int reg, int val)
{
	if (!g_the_chip)
		return -EINVAL;

	switch (id) {
	case DHALL_0:
		if (!g_the_chip->dhall_down_ops ||
				!g_the_chip->dhall_down_ops->set_reg)
			return -EINVAL;
		else
			return g_the_chip->dhall_down_ops->set_reg(reg, val);
	case DHALL_1:
		if (!g_the_chip->dhall_up_ops ||
				!g_the_chip->dhall_up_ops->set_reg)
			return -EINVAL;
		else
			return g_the_chip->dhall_up_ops->set_reg(reg, val);
	default:
		TRI_KEY_ERR("id : %d is not correct\n", id);
		return -EINVAL;
	}
}

bool oplus_hall_is_power_on(void)
{
	if (!g_the_chip || !g_the_chip->dhall_down_ops ||
		!g_the_chip->dhall_down_ops->is_power_on ||
		!g_the_chip->dhall_up_ops ||
				!g_the_chip->dhall_up_ops->is_power_on) {
	} else {
		if (g_the_chip->dhall_down_ops->is_power_on() ||
			g_the_chip->dhall_up_ops->is_power_on())
			return true;
		else
			return false;
	}
	return false;
}
static void reboot_get_position(struct extcon_dev_data *chip)
{
	short delta;
	short up_data1;
	short down_data1;

	if (chip->dhall_data1 < 0 || chip->dhall_data0 < 0) {
		up_data1 = -chip->dhall_data1;
		down_data1 = -chip->dhall_data0;
		delta = up_data1 - down_data1;
	} else
		delta = chip->dhall_data1 - chip->dhall_data0;
	if (delta > 30)
		chip->position = UP_STATE;
	else if (-delta > 30)
		chip->position = DOWN_STATE;
	else
		chip->position = MID_STATE;
	last_position = chip->position;
}

static int interf_get_position(struct extcon_dev_data *chip)
{
	short delta0;
	short delta1;

	delta0 = chip->dhall_data0 - last_d0;
	delta1 = chip->dhall_data1 - last_d1;
	TRI_KEY_LOG("tri_key: delta0 is %d ,delta1 is %d,last_postion is %d\n",
			delta0, delta1, last_position);
	if ((delta1 > calib_upHall_UM_distance - tol0 &&
			delta1 < calib_upHall_UM_distance + tol0) &&
			((delta0 > calib_dnHall_UM_distance - tol0) &&
			(delta0 < calib_dnHall_UM_distance + tol0))) {
		if (last_position == MID_STATE)
			return UP_STATE;
			}
	if ((delta1 > calib_upHall_UD_distance - tol0 &&
		delta1 < calib_upHall_UD_distance + tol0) &&
		((delta0 > calib_dnHall_UD_distance - tol0) &&
		(delta0 < calib_dnHall_UD_distance + tol0)))
		return UP_STATE;
	if ((delta1 > -calib_upHall_MD_distance - tol0 &&
		delta1 < -calib_upHall_MD_distance + tol0) &&
		((delta0 > -calib_dnHall_MD_distance - tol0) &&
		(delta0 < -calib_dnHall_MD_distance + tol0))) {
		if (last_position == MID_STATE)
			return DOWN_STATE;
			}
	if ((delta1 > -calib_upHall_UD_distance - tol0 &&
		delta1 < -calib_upHall_UD_distance + tol0) &&
		((delta0 > -calib_dnHall_UD_distance - tol0) &&
		(delta0 < -calib_dnHall_UD_distance + tol0)))
		return DOWN_STATE;
	if ((delta1 > -calib_upHall_UM_distance - tol0 &&
		delta1 < -calib_upHall_UM_distance + tol0) &&
		((delta0 > -calib_dnHall_UM_distance - tol0) &&
		(delta0 < -calib_dnHall_UM_distance + tol0))) {
		if (last_position == UP_STATE)
			return MID_STATE;
			}
	if ((delta1 > calib_upHall_MD_distance - tol0 &&
		delta1 < calib_upHall_MD_distance + tol0) &&
		((delta0 > calib_dnHall_MD_distance - tol0) &&
		(delta0 < calib_dnHall_MD_distance + tol0))) {
		if (last_position == DOWN_STATE)
			return MID_STATE;
			}
	return -EINVAL;
}

static int get_position(struct extcon_dev_data *chip)
{
	short diff;

	diff = chip->dhall_data1 - chip->dhall_data0;
	if (chip->dhall_data0 > 0) {
		if (diff > calib_UpValueMin - up_mid_tol &&
			diff < calib_UpValueMin + up_tolerance)
			chip->position = UP_STATE;
		if (calib_MdValueMin < 0) {
			if (diff > calib_MdValueMin - mid_down_tol &&
				diff < calib_MdValueMin + up_mid_tol)
				chip->position = MID_STATE;
			}
		if (calib_MdValueMin > 0 || calib_MdValueMin == 0) {
			if (diff > calib_MdValueMin - mid_down_tol &&
				diff < calib_MdValueMin + up_mid_tol)
				chip->position = MID_STATE;
			}
		if (diff > calib_DnValueMin - down_tolerance &&
			diff < calib_DnValueMin + mid_down_tol)
			chip->position = DOWN_STATE;
	} else {
		if (diff > calib_UpValueMin - up_tolerance &&
			diff < calib_UpValueMin + up_mid_tol)
			chip->position = UP_STATE;
		if (calib_MdValueMin < 0) {
			if (diff > calib_MdValueMin - mid_down_tol &&
				diff < calib_MdValueMin + up_mid_tol)
				chip->position = MID_STATE;
			}
		if (calib_MdValueMin > 0 || calib_MdValueMin == 0) {
			if (diff > calib_MdValueMin - mid_down_tol &&
				diff < calib_MdValueMin + up_mid_tol)
				chip->position = MID_STATE;
			}
		if (diff > calib_DnValueMin - mid_down_tol &&
			diff < calib_DnValueMin + down_tolerance)
			chip->position = DOWN_STATE;
	}
	return 0;
}

static int judge_interference(struct extcon_dev_data *chip)
{
	short delta;
	short sum;

	delta = chip->dhall_data1 - chip->dhall_data0;
	TRI_KEY_LOG("tri_key:delta is %d\n", delta);
	sum = chip->dhall_data0 + chip->dhall_data1;
	TRI_KEY_LOG("tri_key:sum is %d\n", sum);
	if (chip->dhall_data1 > 0) {/*positive number*/
		if (delta > calib_UpValueMin - up_mid_tol &&
			delta < calib_UpValueMin + up_tolerance) {
			TRI_KEY_LOG("calib_Min:%d,calib_Sum:%d\n",
				calib_UpValueMin, calib_UpValueSum);
			if (sum < calib_UpValueSum - tol2 ||
				sum > calib_UpValueSum + tol2) {
				chip->interf = 1;
				chip->state = 1;
			} else {
				chip->interf = 0;
				chip->state = 1;
			}
			return 0;
		}
		if (calib_MdValueMin < 0) {
			if (delta > calib_MdValueMin - mid_down_tol &&
				delta < calib_MdValueMin + up_mid_tol) {
				TRI_KEY_LOG("calibMin:%d,calib_Sum:%d\n",
					calib_MdValueMin, calib_MdValueSum);

				if (sum > calib_MdValueSum + tol2 ||
					sum < calib_MdValueSum - tol2) {
					chip->interf = 1;
					chip->state = 2;
				} else {
					chip->interf = 0;
					chip->state = 2;
				}
				return 0;
			}
		}
		if (calib_MdValueMin > 0 || calib_MdValueMin == 0) {
			if (delta > calib_MdValueMin - mid_down_tol &&
				delta < calib_MdValueMin + up_mid_tol) {
				TRI_KEY_LOG("calib_Min:%d,calib_Sum:%d\n",
					calib_MdValueMin, calib_MdValueSum);

				if (sum > calib_MdValueSum + tol2 ||
					sum < calib_MdValueSum - tol2) {
					chip->interf = 1;
					chip->state = 2;
				} else {
					chip->interf = 0;
					chip->state = 2;
					}
				return 0;
				}
			}
		if (delta > calib_DnValueMin - down_tolerance &&
			delta < calib_DnValueMin + mid_down_tol) {
			TRI_KEY_LOG("calib_Min:%d,calib_Sum:%d\n",
				calib_DnValueMin, calib_DnValueSum);

			if (sum < calib_DnValueSum - tol2 ||
				sum > calib_DnValueSum + tol2) {
				chip->interf = 1;
				chip->state = 3;
			} else {
				chip->interf = 0;
				chip->state = 3;
			}
			return 0;
		}
		chip->interf = 1;
		chip->state = 0;
	} else {/*the hall data is negative number*/
		if (delta > calib_UpValueMin - up_tolerance &&
			delta < calib_UpValueMin + up_mid_tol) {
			TRI_KEY_LOG("calib_Min:%d,calib_Sum:%d\n",
				calib_UpValueMin, calib_UpValueSum);

			if (sum < calib_UpValueSum - tol2 ||
				sum > calib_UpValueSum + tol2) {
				chip->interf = 1;
				chip->state = 1;
			} else {
				chip->interf = 0;
				chip->state = 1;
			}
			return 0;
		}
		if (calib_MdValueMin < 0) {
			if (delta > calib_MdValueMin - mid_down_tol &&
				delta < calib_MdValueMin + up_mid_tol) {
				TRI_KEY_LOG("calib_Min:%d,calib_Sum:%d\n",
					calib_MdValueMin, calib_MdValueSum);

				if (sum > calib_MdValueSum + tol2 ||
					sum < calib_MdValueSum - tol2) {
					chip->interf = 1;
					chip->state = 2;
				} else {
					chip->interf = 0;
					chip->state = 2;
				}
				return 0;
			}
		}
		if (calib_MdValueMin > 0 || calib_MdValueMin == 0) {
			if (delta > calib_MdValueMin - mid_down_tol &&
				delta < calib_MdValueMin + up_mid_tol) {
				TRI_KEY_LOG("calib_Min:%d,calib_Sum:%d\n",
					calib_MdValueMin, calib_MdValueSum);

				if (sum > calib_MdValueSum + tol2 ||
						sum < calib_MdValueSum - tol2) {
					chip->interf = 1;
					chip->state = 2;
				} else {
					chip->interf = 0;
					chip->state = 2;
					}
				return 0;
				}
			}
		if (delta > calib_DnValueMin - mid_down_tol &&
			delta < calib_DnValueMin + down_tolerance) {
			TRI_KEY_LOG("tri_key:calib_Min:%d,calib_Sum:%d\n",
				calib_DnValueMin, calib_DnValueSum);

			if (sum < calib_DnValueSum - tol2 ||
				sum > calib_DnValueSum + tol2) {
				chip->interf = 1;
				chip->state = 3;
			} else {
				chip->interf = 0;
				chip->state = 3;
			}
			return 0;
		}
		chip->interf = 1;
		chip->state = 0;
	}
	return -EINVAL;
}


static int oplus_get_data(struct extcon_dev_data *chip)
{
	int res = 0;

	res = oplus_hall_get_data(DHALL_0);
	if (res < 0) {
		TRI_KEY_LOG("tri_key:get DHALL_0 data failed,res =%d\n", res);
		return res;
	}
	res = oplus_hall_get_data(DHALL_1);
	if (res < 0) {
		TRI_KEY_LOG("tri_key:get DHALL_1 data failed,res =%d\n", res);
		return res;
	}

	return res;
}

static int reupdata_threshold(struct extcon_dev_data *chip)
{
	int res = 0;
	int tolen = 22;

	switch (chip->position) {
	case UP_STATE:
			res = oplus_hall_update_threshold(DHALL_1, UP_STATE,
			chip->dhall_data1-tolen, chip->dhall_data1+tolen);
			if (res < 0) {
				TRI_KEY_LOG("updata_threshold fail:%d\n", res);
				goto fail;
			}
			res = oplus_hall_update_threshold(DHALL_0, UP_STATE,
				-500, 500);
			if (res < 0) {
				TRI_KEY_LOG("updata_threshold fail:%d\n", res);
				goto fail;
			}
		TRI_KEY_LOG("tri_key:updata_threshold up:low:%d,high: %d\n",
			chip->dhall_data1-tolen, chip->dhall_data1+tolen);
		oplus_hall_clear_irq(DHALL_1);
		oplus_hall_clear_irq(DHALL_0);
		break;
	case MID_STATE:
		if (chip->dhall_data0 < 0 || chip->dhall_data1 < 0) {
			res = oplus_hall_update_threshold(DHALL_1, MID_STATE,
			chip->dhall_data1 - tolen, chip->dhall_data1 + tolen);
			if (res < 0) {
				TRI_KEY_LOG("updata_threshold fail:%d\n", res);
				goto fail;
			}
		TRI_KEY_LOG("tri_key:updata_threshold up:low:%d,high:%d\n",
			chip->dhall_data1 - tolen, chip->dhall_data1 + tolen);
		} else {
			res = oplus_hall_update_threshold(DHALL_1, MID_STATE,
			chip->dhall_data1 - tolen, chip->dhall_data1 + tolen);
			if (res < 0) {
				TRI_KEY_LOG("updata_threshold fail:%d\n", res);
				goto fail;
			}
		TRI_KEY_LOG("tri_key:updata_threshold up:low:%d,high:%d\n",
			chip->dhall_data1 - tolen, chip->dhall_data1 + tolen);
		}
		oplus_hall_clear_irq(DHALL_1);
		if (chip->dhall_data0 < 0 || chip->dhall_data1 < 0) {
			res = oplus_hall_update_threshold(DHALL_0, MID_STATE,
			chip->dhall_data0 - tolen, chip->dhall_data0 + tolen);
			if (res < 0) {
				TRI_KEY_LOG("updata_threshold fail:%d\n", res);
				goto fail;
			}
		TRI_KEY_LOG("tri_key:updata_threshold down:low:%d,high:%d\n",
		chip->dhall_data0 - tolen, chip->dhall_data0 + tolen);
		} else {
			res = oplus_hall_update_threshold(DHALL_0, MID_STATE,
			chip->dhall_data0 - tolen, chip->dhall_data0 + tolen);
			if (res < 0) {
				TRI_KEY_LOG("updata_threshold fail:%d\n", res);
				goto fail;
			}
		TRI_KEY_LOG("tri_key:updata_threshold down:low:%d,high:%d\n",
			chip->dhall_data0 - tolen, chip->dhall_data0 + tolen);
		}
		oplus_hall_clear_irq(DHALL_0);
		break;
	case DOWN_STATE:
		res = oplus_hall_update_threshold(DHALL_0, DOWN_STATE,
			chip->dhall_data0 - tolen, chip->dhall_data0 + tolen);
		if (res < 0) {
			TRI_KEY_LOG("updata_threshold fail:%d\n", res);
			goto fail;
		}
		TRI_KEY_LOG("tri_key:updata_threshold down:low:%d,high:%d\n",
			chip->dhall_data0 - tolen, chip->dhall_data0 + tolen);
		res = oplus_hall_update_threshold(DHALL_1, DOWN_STATE,
			-500, 500);
		if (res < 0) {
			TRI_KEY_LOG("updata_threshold fail:%d\n", res);
			goto fail;
		}

		oplus_hall_clear_irq(DHALL_0);
		oplus_hall_clear_irq(DHALL_1);
		break;
		}
fail:
	last_d0 = chip->dhall_data0;
	last_d1 = chip->dhall_data1;
	last_interf = chip->interf;
	TRI_KEY_LOG("tri_key:last_d0 is %d ,last_d1 is %d\n",
		last_d0, last_d1);
	oplus_hall_clear_irq(DHALL_0);
	oplus_hall_clear_irq(DHALL_1);
	return res;
}

static void report_key_value(struct extcon_dev_data *chip)
{
	if (chip->position == DOWN_STATE) {
		chip->state = 3;
		input_report_key(chip->input_dev, KEY_F3, 3);
		input_sync(chip->input_dev);
		input_report_key(chip->input_dev, KEY_F3, 0);
		input_sync(chip->input_dev);
		TRI_KEY_LOG("tri_key: report down key successful!\n");
	}
	if (chip->position == UP_STATE) {
		chip->state = 1;
		TRI_KEY_LOG("tri_key: report up key successful!\n");
		input_report_key(chip->input_dev, KEY_F3, 1);
		input_sync(chip->input_dev);
		input_report_key(chip->input_dev, KEY_F3, 0);
		input_sync(chip->input_dev);
	}
	if (chip->position == MID_STATE) {
		chip->state = 2;
		TRI_KEY_LOG("tri_key: report mid key successful!\n");
		input_report_key(chip->input_dev, KEY_F3, 2);
		input_sync(chip->input_dev);
		input_report_key(chip->input_dev, KEY_F3, 0);
		input_sync(chip->input_dev);
	} else
		TRI_KEY_LOG("no report\n");
}

static int report_calibration_location(struct extcon_dev_data *chip)
{
	oplus_get_data(chip);
	get_position(chip);
	reupdata_threshold(chip);
	if (chip->position == last_position) {
		TRI_KEY_LOG("no report\n");
		goto err;
	} else
		report_key_value(chip);
	last_position = chip->position;
	return 0;
err:
	return -EINVAL;
}

static int judge_calibration_data(struct extcon_dev_data *chip)
{
	if (calib_UpValueMin == 0 || calib_UpValueSum == 0 ||
	calib_MdValueSum == 0 || calib_DnValueMin == 0
						|| calib_DnValueSum == 0) {
		oplus_get_data(chip);
		reboot_get_position(chip);
		if (chip->position == UP_STATE) {
			calib_UpValueMin = chip->dhall_data1 -
						chip->dhall_data0;
			calib_UpValueSum = chip->dhall_data1 +
						chip->dhall_data0;
			TRI_KEY_LOG("UP_MIN is%d,UP_SUM is %d\n",
				calib_UpValueMin, calib_UpValueSum);
		}
		if (chip->position == MID_STATE) {
			calib_MdValueMin = chip->dhall_data1 -
						chip->dhall_data0;
			calib_MdValueSum = chip->dhall_data1 +
						chip->dhall_data0;
			TRI_KEY_LOG("MID_MIN is%d,MID_SUM is %d\n",
				calib_MdValueMin, calib_MdValueSum);
		}
		if (chip->position == DOWN_STATE) {
			calib_DnValueMin = chip->dhall_data1 -
						chip->dhall_data0;
			calib_DnValueSum = chip->dhall_data1 +
						chip->dhall_data0;
			TRI_KEY_LOG("UP_MIN is%d,UP_SUM is %d\n",
				calib_DnValueMin, calib_DnValueSum);
		}
		report_key_value(chip);
		reupdata_threshold(chip);
		return -EINVAL;
	}
	return 0;
}


int oplus_hall_irq_handler(unsigned int id)
{
	TRI_KEY_LOG("%d tri_key:call :%s\n", id, __func__);
	if (!g_the_chip)
		TRI_KEY_LOG("g_the_chip null\n ");
	else
		schedule_work(&g_the_chip->dwork);
	return IRQ_HANDLED;
}
EXPORT_SYMBOL(oplus_hall_irq_handler);

static void tri_key_dev_work(struct work_struct *work)
{
	struct extcon_dev_data *chip = container_of(work,
			struct extcon_dev_data, dwork);
	int res = 0;
	int position = -1;
	int diff0 = 0;
	int diff1 = 0;
	int count = 0;
	int dhall0_sum = 0;
	int dhall1_sum = 0;
	int aver0 = 0;
	int aver1 = 0;
	ktime_t starttime, endtime;
	u64 usecs64;
	int usecs;

	mutex_lock(&chip->mtx);

	starttime = ktime_get();
	msleep(50);
	res = judge_calibration_data(chip);
	if (res < 0)
		goto FINAL;

/*get data*/
	res = oplus_get_data(chip);
	if (res < 0) {
		TRI_KEY_LOG("tri_key:get hall data failed!\n");
		goto fail;
	}
	TRI_KEY_LOG("tri_key:data1 is %d, data0 is %d\n",
				chip->dhall_data1, chip->dhall_data0);

/*judge interference*/
	res = judge_interference(chip);
	TRI_KEY_LOG("tri_key:chip->interf is %d ,chip->state is %d\n",
					chip->interf, chip->state);
	if (!last_interf && chip->interf) {
		msleep(200);
		oplus_get_data(chip);
		TRI_KEY_LOG("tri_key:data1 is %d, data0 is %d\n",
					chip->dhall_data1, chip->dhall_data0);

		judge_interference(chip);
	}
/*get position*/
	if (!chip->interf) {
		hrtimer_cancel(&tri_key_timer);
		time = 1;
		if (!last_interf) {
			interf_count = 0;
			get_position(chip);
		TRI_KEY_LOG("tri_key:the position is %d\n", chip->position);
		} else {
			msleep(50);
			oplus_get_data(chip);
			judge_interference(chip);
			if (chip->interf)
				goto FINAL;
			else
				get_position(chip);
			}
		}
	else {
		hrtimer_cancel(&tri_key_timer);
		TRI_KEY_LOG("tri_key:time0 is %d\n", time);
		hrtimer_start(&tri_key_timer, ktime_set(time, 0),
			HRTIMER_MODE_REL);
		while (count < 4) {
			msleep(35);
			oplus_hall_get_data(DHALL_0);
			oplus_hall_get_data(DHALL_1);
			dhall0_sum += chip->dhall_data0;
			dhall1_sum += chip->dhall_data1;
			count++;
		}
		aver0 = dhall0_sum / 4;
		aver1 = dhall1_sum / 4;
		if (!last_interf) {
			diff0 = aver0 - chip->dhall_data0;
			diff1 = aver1 - chip->dhall_data1;
			TRI_KEY_LOG("tri_key:diff0 is %d,diff1 is %d\n",
				diff0, diff1);
			if ((diff0 > -10 && diff0 < 10) &&
					(diff1 > -10 && diff1 < 10)) {
				chip->position = last_position;
				goto UPDATA_HTRES;
			} else {/*inconstant interference*/
				last_interf = chip->interf;
				goto FINAL;
			}
		}
		diff0 = aver0 - chip->dhall_data0;
		diff1 = aver1 - chip->dhall_data1;
		TRI_KEY_LOG("tri_key:diff0 is %d,diff1 is %d\n",
			diff0, diff1);

/*inconstantly interference*/
		if ((diff0 < -10 || diff0 > 10) &&
				(diff1 < -10 || diff1 > 10)) {
			interf_count++;
			if (interf_count == 15) {
				TRI_KEY_LOG("tri_key:count = 15,msleep 5s\n");
				msleep(5000);
				interf_count = 0;
				goto FINAL;
			}
			TRI_KEY_LOG("tri_key:inconstantlt interference\n");
			reupdata_threshold(chip);
			goto FINAL;
		}

		chip->dhall_data0 = aver0;
		chip->dhall_data1 = aver1;
		position = interf_get_position(chip);
		if (position == -22)
			TRI_KEY_LOG("tri_key:get position failed\n");
		else
			chip->position = position;
	}
	TRI_KEY_LOG("tri_key:t_diff0 is %d,t_diff1 is %d\n",
	chip->dhall_data0 - last_d0, chip->dhall_data1 - last_d1);
/*updata threshold*/
UPDATA_HTRES:
	res = reupdata_threshold(chip);
	if (res < 0) {
		TRI_KEY_LOG("tri_key:updata_threshold failed!\n");
		goto fail;
		}

/*report key value*/
	if (chip->position == last_position)
		goto FINAL;
	else {
		report_key_value(chip);
		last_position = chip->position;
		endtime = ktime_get();
		usecs64 = ktime_to_ns(ktime_sub(endtime, starttime));
		do_div(usecs64, NSEC_PER_USEC);
		usecs = usecs64;
		if (usecs == 0)
			usecs = 1;
		TRI_KEY_LOG("report key after %ld.%03ld msecs\n",
			usecs / USEC_PER_MSEC, usecs % USEC_PER_MSEC);
	}
fail:
	if (res < 0)
		TRI_KEY_LOG("tri_key:dev_work failed,res =%d\n", res);
FINAL:
	oplus_hall_disable_irq(1);

	TRI_KEY_LOG("%s achieve\n", __func__);
	mutex_unlock(&chip->mtx);
}

static enum hrtimer_restart tri_key_status_timeout(struct hrtimer *timer)
{
	schedule_work(&tri_key_timeout_work);
	return HRTIMER_NORESTART;
}

static void tri_key_timeout_work_func(struct work_struct *work)
{
	oplus_get_data(g_the_chip);
	judge_interference(g_the_chip);
	if (g_the_chip->interf) {
		time = time * 2;
		TRI_KEY_LOG("tri_key:time1 is %d\n", time);
		if (time > 2)
			time = 2;
		}
	else {
		get_position(g_the_chip);
		if (g_the_chip->position == last_position)
			return;
		reupdata_threshold(g_the_chip);
		report_key_value(g_the_chip);
		last_position = g_the_chip->position;
		time = 1;
		}
}


static short Sum(short value0, short value1)
{
	short sum = 0;

	sum = value0 + value1;
	return sum;
}
static short Minus(short value0, short value1)
{
	short minus = 0;

	minus = value0 - value1;
	return minus;
}

void initialCalibValue(short calib_dnHall_UpV, short calib_dnHall_MdV,
			short calib_dnHall_DnV, short calib_upHall_UpV,
			short calib_upHall_MdV, short calib_upHall_DnV)
{
	calib_UpValueSum = Sum(calib_dnHall_UpV, calib_upHall_UpV);
	calib_MdValueSum = Sum(calib_dnHall_MdV, calib_upHall_MdV);
	calib_DnValueSum = Sum(calib_dnHall_DnV, calib_upHall_DnV);
	calib_UpValueMin = Minus(calib_upHall_UpV, calib_dnHall_UpV);
	calib_MdValueMin = Minus(calib_upHall_MdV, calib_dnHall_MdV);
	calib_DnValueMin = Minus(calib_upHall_DnV, calib_dnHall_DnV);
	calib_upHall_UM_distance = Minus(calib_upHall_UpV, calib_upHall_MdV);
	calib_upHall_MD_distance = Minus(calib_upHall_MdV, calib_upHall_DnV);
	calib_dnHall_UM_distance = Minus(calib_dnHall_UpV, calib_dnHall_MdV);
	calib_dnHall_MD_distance = Minus(calib_dnHall_MdV, calib_dnHall_DnV);
	calib_upHall_UD_distance = Minus(calib_upHall_UpV, calib_upHall_DnV);
	calib_dnHall_UD_distance = Minus(calib_dnHall_UpV, calib_dnHall_DnV);
	up_mid_tol = (short)(abs(calib_UpValueMin - calib_MdValueMin)
		* 4 / 10);
	up_tolerance = (short)(abs(calib_UpValueMin - calib_MdValueMin)
		* 11 / 10);
	mid_down_tol = (short)(abs(calib_MdValueMin - calib_DnValueMin)
		* 4 / 10);
	down_tolerance = (short)(abs(calib_MdValueMin -
		calib_DnValueMin) * 11 / 10);
	up_mid_distance = (short)(abs(calib_UpValueMin -
		calib_MdValueMin) * 2 / 10);
	mid_down_distance = (short)(abs(calib_MdValueMin -
		calib_DnValueMin) * 2 / 10);
	TRI_KEY_LOG("Upmin:%d, Mdmin:%d, Dnmin:%d\n",
		calib_UpValueMin, calib_MdValueMin, calib_DnValueMin);
	TRI_KEY_LOG("up_mid_tol:%d, mid_down_tol:%d\n",
		up_mid_tol, mid_down_tol);
}

static ssize_t proc_hall_data_read(struct file *file, char __user *user_buf,
			size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[20] = {0};

	if (!g_the_chip) {
		TRI_KEY_ERR("g_the_chip null\n");
		snprintf(page, sizeof(page), "%d\n", -1);
	} else {
		oplus_hall_get_data(DHALL_0);
		oplus_hall_get_data(DHALL_1);

		snprintf(page, sizeof(page), "%d, %d\n",
			g_the_chip->dhall_data0, g_the_chip->dhall_data1);
	}
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static const struct file_operations proc_hall_data_ops = {
	.read  = proc_hall_data_read,
	.open  = simple_open,
	.owner = THIS_MODULE,
};

static ssize_t proc_tri_state_read(struct file *file, char __user *user_buf,
		size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[6] = {0};

	if (!g_the_chip) {
		TRI_KEY_ERR("g_the_chip null\n");
		snprintf(page, sizeof(page), "%d\n", -1);
	} else {
		oplus_hall_get_data(DHALL_0);
		oplus_hall_get_data(DHALL_1);
		snprintf(page, sizeof(page), "%d\n", g_the_chip->state);
	}
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static const struct file_operations proc_tri_state_ops = {
	.read  = proc_tri_state_read,
	.open  = simple_open,
	.owner = THIS_MODULE,
};

static ssize_t proc_hall_data_calib_read(struct file *file, char __user *user_buf,
			size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[50] = {0};

	if (!g_the_chip) {
		TRI_KEY_ERR("g_the_chip null\n");
		snprintf(page, sizeof(page), "%d\n", -1);
	} else {
		snprintf(page, sizeof(page), "%d,%d,%d,%d,%d,%d,%d,%d\n",
		g_the_chip->dnHall_UpV, g_the_chip->upHall_UpV,
		g_the_chip->dnHall_MdV, g_the_chip->upHall_MdV,
		g_the_chip->dnHall_DnV, g_the_chip->upHall_DnV,
		up_mid_tol, mid_down_tol);
	}
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static ssize_t proc_hall_data_calib_write(struct file *file, const char __user *buffer,
			size_t count, loff_t *ppos)
{
	int data[6] = {0};
	char temp[35] = {0};
	int ret = -1;

	if (!g_the_chip) {
		TRI_KEY_ERR("g_the_chip null\n");
		return count;
	}
	ret = copy_from_user(temp, buffer, count);
	if (ret) {
		TRI_KEY_ERR("%s: read proc input error.\n", __func__);
		return count;
	}
	TRI_KEY_LOG("temp is %s:\n", temp);
	if (sscanf(temp, "%d,%d,%d,%d,%d,%d", &data[0], &data[1], &data[2],
		&data[3], &data[4], &data[5]) == 6) {
		g_the_chip->dnHall_UpV = data[0];
		g_the_chip->upHall_UpV = data[1];
		g_the_chip->dnHall_MdV = data[2];
		g_the_chip->upHall_MdV = data[3];
		g_the_chip->dnHall_DnV = data[4];
		g_the_chip->upHall_DnV = data[5];
		TRI_KEY_ERR("data[%d %d %d %d %d %d]\n", data[0], data[1],
				data[2], data[3], data[4], data[5]);
	} else
		TRI_KEY_ERR("fail\n");

	initialCalibValue(g_the_chip->dnHall_UpV, g_the_chip->dnHall_MdV,
			g_the_chip->dnHall_DnV, g_the_chip->upHall_UpV,
			g_the_chip->upHall_MdV, g_the_chip->upHall_DnV);
	report_calibration_location(g_the_chip);
	return count;
}

static const struct file_operations proc_hall_data_calib_ops = {
	.write = proc_hall_data_calib_write,
	.read  = proc_hall_data_calib_read,
	.open  = simple_open,
	.owner = THIS_MODULE,
};

static ssize_t proc_hall_debug_info_write(struct file *file, const char __user *buffer,
			size_t count, loff_t *ppos)
{
	int tmp = 0;
	char buf[4] = {0};

	if (count > 2)
		return count;
	copy_from_user(buf, buffer, count);
	if (!kstrtoint(buf, 0, &tmp))
		tri_key_debug = tmp;
	else
		TRI_KEY_DEBUG("invalid content: '%s', length = %zd\n",
		buf, count);

	return count;
}
static ssize_t proc_hall_debug_info_read(struct file *file, char __user *user_buf,
			 size_t count, loff_t *ppos)
{
	int ret = -1;
	char page[6] = {0};

	if (!g_the_chip) {
		TRI_KEY_ERR("g_the_chip null\n");
		snprintf(page, sizeof(page), "%d\n", -1);
	} else
		snprintf(page, sizeof(page), "%d\n", tri_key_debug);

	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static const struct file_operations proc_hall_debug_info_ops = {
	.write = proc_hall_debug_info_write,
	.read  = proc_hall_debug_info_read,
	.open  = simple_open,
	.owner = THIS_MODULE,
};

static int init_trikey_proc(struct extcon_dev_data *hall_dev)
{
	int ret = 0;
	struct proc_dir_entry *prEntry_trikey = NULL;
	struct proc_dir_entry *prEntry_tmp = NULL;

	TRI_KEY_LOG("%s entry\n", __func__);
	prEntry_trikey = proc_mkdir("tristatekey", NULL);
	if (prEntry_trikey == NULL) {
		ret = -ENOMEM;
		TRI_KEY_ERR("%s: Couldn't create trikey proc entry\n", __func__);
	}

	prEntry_tmp = proc_create("hall_data", 0644, prEntry_trikey, &proc_hall_data_ops);
	if (prEntry_tmp == NULL) {
		ret = -ENOMEM;
		TRI_KEY_ERR("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
	}

	prEntry_tmp = proc_create("tri_state", 0644, prEntry_trikey, &proc_tri_state_ops);
	if (prEntry_tmp == NULL) {
		ret = -ENOMEM;
		TRI_KEY_ERR("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
	}

	prEntry_tmp = proc_create("hall_data_calib", 0666, prEntry_trikey,
			&proc_hall_data_calib_ops);
	if (prEntry_tmp == NULL) {
		ret = -ENOMEM;
		TRI_KEY_ERR("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
	}

	prEntry_tmp = proc_create("hall_debug_info", 0666, prEntry_trikey,
			&proc_hall_debug_info_ops);
	if (prEntry_tmp == NULL) {
		ret = -ENOMEM;
		TRI_KEY_ERR("%s: Couldn't create proc entry, %d\n", __func__, __LINE__);
	}

	return ret;
}
static void register_tri_key_dev_work(struct work_struct *work)
{
	struct extcon_dev_data *chip = container_of(work, struct extcon_dev_data,
			register_work);
	struct extcon_dev_data *pdev;
	int err = 0;
	int res = 0;

	TRI_KEY_LOG("call %s\n", __func__);

	if (!g_the_chip) {
		chip = kzalloc(sizeof(struct extcon_dev_data), GFP_KERNEL);
		if (!chip) {
			TRI_KEY_ERR("kzalloc err\n");
			return;
		}
		g_the_chip = chip;
	} else {
		chip = g_the_chip;
	}
	pdev = g_hall_dev;
	chip->input_dev = pdev->input_dev;
	chip->input_dev = input_allocate_device();
	if (chip->input_dev == NULL) {
		res = -ENOMEM;
		TRI_KEY_ERR("Failed to allocate input device\n");
		goto fail;
	}
	chip->input_dev->name = TRI_KEY_DEVICE;

	set_bit(EV_SYN, chip->input_dev->evbit);
	set_bit(EV_KEY, chip->input_dev->evbit);
	set_bit(KEY_F3, chip->input_dev->keybit);
	res = input_register_device(chip->input_dev);
	if (res) {
		TRI_KEY_ERR("%s: Failed to register input device\n", __func__);
		input_free_device(chip->input_dev);
		goto fail;
	}
	res = init_trikey_proc(pdev);
	if (res < 0) {
		TRI_KEY_ERR("create trikey proc fail\n");
		goto fail;
	}

	if (0 && (!chip->dhall_up_ops || !chip->dhall_down_ops)) {
		TRI_KEY_ERR("no dhall available\n");
		goto fail;
	}



	INIT_WORK(&chip->dwork, tri_key_dev_work);
	hrtimer_init(&tri_key_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	tri_key_timer.function = tri_key_status_timeout;
	INIT_WORK(&tri_key_timeout_work, tri_key_timeout_work_func);
/*get data when reboot*/
	res = oplus_get_data(chip);
	if (res < 0) {
		TRI_KEY_LOG("tri_key:get hall data failed!\n");
		goto fail;
	}
	TRI_KEY_LOG("tri_key:data1 is %d, data0 is %d\n",
				chip->dhall_data1, chip->dhall_data0);

/*get position when reboot*/
	reboot_get_position(chip);
/*set threshold when reboot;*/
	err = reupdata_threshold(chip);
	if (err < 1) {
		TRI_KEY_ERR("%s reupdata_threshold failed\n", __func__);
		goto fail;
	}
/*report key value*/
	report_key_value(chip);
	last_position = chip->position;
	err = oplus_hall_set_detection_mode(DHALL_0,
			DETECTION_MODE_INTERRUPT);
	TRI_KEY_LOG("tri_key:set 0 detection mode\n");
	if (err < 0) {
		TRI_KEY_ERR("%s set HALL0 detection mode failed %d\n",
			__func__, err);
		goto fail;
	}
	err = oplus_hall_set_detection_mode(DHALL_1,
			DETECTION_MODE_INTERRUPT);
	TRI_KEY_LOG("tri_key:set 1 detection mode\n");
	if (err < 0) {
		TRI_KEY_ERR("%s set HALL1 detection mode failed %d\n",
			__func__, err);
		goto fail;
	}
	TRI_KEY_LOG("%s probe success.\n", __func__);
	return;

fail:
	kfree(chip);
	g_the_chip = NULL;
	TRI_KEY_LOG("fail\n");
}

int oplus_register_hall(const char *name, struct dhall_operations *ops,
		struct extcon_dev_data *hall_dev_t)
{
	static int hall_count;
	struct extcon_dev_data *hall_dev = hall_dev_t;

	if (!name || !ops) {
		TRI_KEY_ERR("name is NULL or ops is NULL,",
			"would not register digital hall\n");
		return -EINVAL;
	}

	if (!g_the_chip) {
		struct extcon_dev_data *chip = kzalloc(sizeof(struct
				extcon_dev_data), GFP_KERNEL);
		if (!chip) {
			TRI_KEY_ERR("kzalloc err\n");
			return -ENOMEM;
		}
		g_the_chip = chip;
		g_hall_dev = hall_dev;
	}
	TRI_KEY_LOG("name : %s\n", name);
	if (strcmp(name, "hall_down") == 0) {
		TRI_KEY_LOG("name == hall_down");
		if (!g_the_chip->dhall_down_ops) {
			if (ops) {
				g_the_chip->dhall_down_ops = ops;
				g_the_chip->d_name = name;
				hall_count++;
			} else {
				TRI_KEY_ERR("dhall_down_ops NULL\n");
				return -EINVAL;
			}
		} else {
			TRI_KEY_ERR("dhall_down_ops has been register\n");
			return -EINVAL;
		}
	}
	if (strcmp(name, "hall_up") == 0) {
		TRI_KEY_LOG("name == hall_up");
		if (!g_the_chip->dhall_up_ops) {
			if (ops) {
				g_the_chip->dhall_up_ops = ops;
				g_the_chip->d_name = name;
				hall_count++;
			}  else {
				TRI_KEY_ERR("dhall_up_ops NULL\n");
				return -EINVAL;
			}
		} else {
			TRI_KEY_ERR("dhall_up_ops has been register\n");
			return -EINVAL;
		}
	}
	if (hall_count > 1) {
		INIT_WORK(&g_the_chip->register_work, register_tri_key_dev_work);
		schedule_work(&g_the_chip->register_work);
	}
	return 0;
}
EXPORT_SYMBOL(oplus_register_hall);


