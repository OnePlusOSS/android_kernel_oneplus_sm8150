/************************************************************************************
** Copyright (C), 2008-2017, OPLUS Mobile Comm Corp., Ltd
** VENDOR_EDIT
** File: oplus_drv8834.c
**
** Description:
**	Definitions for motor driver ic drv8834.
**
** Version: 1.0
** Date created: 2018/01/14,20:27
**
** --------------------------- Revision History: ------------------------------------
* <version>		<date>		<author>		<desc>
**************************************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>

#ifdef CONFIG_MTK_PLATFORM
#include <mt-plat/mtk_pwm.h>
#elif defined(CONFIG_ARCH_SM8150) || defined(CONFIG_ARCH_SM6150)
#include <linux/pwm.h>
#else
#include <linux/qpnp/pin.h>
#include <linux/qpnp/pwm.h>
#endif

#include "oplus_drv8834.h"
#include "../oplus_motor.h"

#define oplus_gpio_set_value(gpio, val) do { \
	if (gpio_is_valid(gpio))\
		gpio_set_value(gpio, val);\
} while (0)

#define oplus_gpio_free(gpio) do { \
	if (gpio_is_valid(gpio))\
		gpio_free(gpio);\
} while (0)

#define oplus_gpio_get_value(gpio) \
	((gpio_is_valid(gpio)) ? (gpio_get_value(gpio)) : ERROR_NO)

static struct oplus_mdrv_chip *g_chip = NULL;
static void drv8834_check_motor_type(struct oplus_mdrv_chip *chip);

static int drv8834_parse_dts(struct oplus_mdrv_chip *chip)
{
	struct device_node *np = chip->dev->of_node;
	int rc = 0;

	chip->pctrl = devm_pinctrl_get(chip->dev);

	if (IS_ERR_OR_NULL(chip->pctrl)) {
		MOTOR_ERR("failed to get pinctrl\n");
	}

#ifndef CONFIG_MTK_PLATFORM
	chip->pwm_dev = of_pwm_get(np, NULL);

	if (IS_ERR_OR_NULL(chip->pwm_dev)) {
		MOTOR_LOG("pwm_dev not specified \n");
		return -EINVAL;

	} else {
		MOTOR_LOG("request pwm seccess\n");
	}

#endif

	chip->boost_gpio = of_get_named_gpio(np, "boost-gpio", 0);

	if (!gpio_is_valid(chip->boost_gpio)) {
		MOTOR_LOG("md-boost-gpio gpio not specified\n");

	} else {
		rc = gpio_request(chip->boost_gpio, "boost-gpio");

		if (rc) {
			MOTOR_LOG("request md-boost-gpio gpio failed, rc=%d\n", rc);
		}
	}

	chip->step_gpio = of_get_named_gpio(np, "step-gpio", 0);

	if (!gpio_is_valid(chip->step_gpio)) {
		MOTOR_LOG("md-step-gpio gpio not specified\n");

	} else {
		rc = gpio_request(chip->step_gpio, "step-gpio");

		if (rc) {
			MOTOR_LOG("request md-step-gpio failed, rc=%d\n", rc);
		}
	}

	chip->m1_gpio = of_get_named_gpio(np, "m1-gpio", 0);

	if (!gpio_is_valid(chip->m1_gpio)) {
		MOTOR_LOG("md-m1-gpio gpio not specified\n");

	} else {
		rc = gpio_request(chip->m1_gpio, "m1-gpio");

		if (rc) {
			MOTOR_LOG("request md-m1-gpio failed, rc=%d\n", rc);

		} else {
			oplus_gpio_set_value(chip->m1_gpio, 0);
		}
	}

	chip->vref_gpio = of_get_named_gpio(np, "vref-gpio", 0);

	if (!gpio_is_valid(chip->vref_gpio)) {
		MOTOR_LOG("md-vref-gpio gpio not specified\n");

	} else {
		rc = gpio_request(chip->vref_gpio, "vref-gpio");

		if (rc) {
			MOTOR_LOG("request md_vref-gpio gpio failed, rc=%d\n", rc);

		} else {
			oplus_gpio_set_value(chip->vref_gpio, 0);
		}
	}

	chip->sleep_gpio = of_get_named_gpio(np, "sleep-gpio", 0);

	if (!gpio_is_valid(chip->sleep_gpio)) {
		MOTOR_LOG("md-sleep-gpio gpio not specified\n");

	} else {
		rc = gpio_request(chip->sleep_gpio, "sleep-gpio");

		if (rc) {
			MOTOR_LOG("request md-sleep-gpio gpio failed, rc=%d\n", rc);

		} else {
			oplus_gpio_set_value(chip->sleep_gpio, 0);
		}
	}

	chip->sleep1_gpio = of_get_named_gpio(np, "sleep1-gpio", 0);

	if (!gpio_is_valid(chip->sleep1_gpio)) {
		MOTOR_LOG("md-sleep-gpio1 gpio not specified\n");

	} else {
		rc = gpio_request(chip->sleep1_gpio, "sleep-gpio1");

		if (rc) {
			MOTOR_LOG("request md-sleep-gpio1 gpio failed, rc=%d\n", rc);

		} else {
			oplus_gpio_set_value(chip->sleep1_gpio, 0);
		}
	}

	chip->dir_gpio = of_get_named_gpio(np, "dir-gpio", 0);

	if (!gpio_is_valid(chip->dir_gpio)) {
		MOTOR_LOG("md-dir-gpio gpio not specified\n");

	} else {
		rc = gpio_request(chip->dir_gpio, "dir-gpio");

		if (rc) {
			MOTOR_LOG("request md-dir-gpio gpio failed, rc=%d\n", rc);

		} else {
			oplus_gpio_set_value(chip->dir_gpio, 0);
		}
	}

	chip->m0_gpio = of_get_named_gpio(np, "m0-gpio", 0);

	if (!gpio_is_valid(chip->m0_gpio)) {
		MOTOR_LOG("md-m0-gpio gpio not specified\n");

	} else {
		rc = gpio_request(chip->m0_gpio, "m0-gpio");

		if (rc) {
			MOTOR_LOG("request md-m0-gpio failed, rc=%d\n", rc);

		} else {
			oplus_gpio_set_value(chip->m0_gpio, 0);
		}
	}

	chip->dir_switch_gpio = of_get_named_gpio(np, "dir_switch-gpio", 0);

	if (!gpio_is_valid(chip->dir_switch_gpio)) {
		MOTOR_LOG("dir_switch_gpio not specified\n");

	} else {
		rc = gpio_request(chip->dir_switch_gpio, "dir_switch-gpio");

		if (rc) {
			MOTOR_LOG("request md-m0-gpio failed, rc=%d\n", rc);
		}
	}

	rc = of_property_read_u32(np, "ratio-a", &chip->ratio_a);

	if (rc) {
		chip->ratio_a = RATIO_A;
	}

	rc = of_property_read_u32(np, "ratio-b", &chip->ratio_b);

	if (rc) {
		chip->ratio_b = RATIO_B;
	}

	rc = of_property_read_u32(np, "ratio-c", &chip->ratio_c);

	if (rc) {
		chip->ratio_c = RATIO_C;
	}

	rc = of_property_read_u32(np, "ratio-d", &chip->ratio_d);

	if (rc) {
		chip->ratio_d = RATIO_D;
	}

	MOTOR_LOG("%d %d %d %d %d %d %d %d %d [%d %d %d %d]\n", chip->boost_gpio,
		  chip->vref_gpio, chip->sleep_gpio, chip->sleep1_gpio,
		  chip->dir_gpio, chip->m0_gpio, chip->m1_gpio, chip->step_gpio,
		  chip->dir_switch_gpio,
		  chip->ratio_a, chip->ratio_b, chip->ratio_c, chip->ratio_d);

	return 0;
}

static int drv8834_init_dir_switch(struct oplus_mdrv_chip *chip)
{
	int ret = 0;

	if (IS_ERR_OR_NULL(chip->pctrl)) {
		ret = PTR_ERR(chip->pctrl);
		MOTOR_ERR("failed to get pinctrl\n");
		return ret;
	}

	chip->dir_switch_state = pinctrl_lookup_state(chip->pctrl,
				 "dir_switch_gpio_input_high");

	if (IS_ERR_OR_NULL(chip->dir_switch_state)) {
		ret = PTR_ERR(chip->dir_switch_state);
		MOTOR_ERR("pinctrl_lookup_state, err:%d\n", ret);
		return ret;
	}

	pinctrl_select_state(chip->pctrl, chip->dir_switch_state);

	msleep(10);

	chip->dir_switch = oplus_gpio_get_value(chip->dir_switch_gpio);

	chip->dir_switch_state = pinctrl_lookup_state(chip->pctrl,
				 "dir_switch_gpio_input_low");

	if (IS_ERR_OR_NULL(chip->dir_switch_state)) {
		ret = PTR_ERR(chip->dir_switch_state);
		MOTOR_ERR("pinctrl_lookup_state, err:%d\n", ret);
		return ret;
	}

	pinctrl_select_state(chip->pctrl, chip->dir_switch_state);

	return 0;
}

static int drv8834_init_default(struct oplus_mdrv_chip *chip)
{
	int ret = 0;

	if (IS_ERR_OR_NULL(chip->pctrl)) {
		ret = PTR_ERR(chip->pctrl);
		MOTOR_ERR("failed to get pinctrl\n");
		return ret;
	}

	chip->default_state = pinctrl_lookup_state(chip->pctrl, "default");

	if (IS_ERR_OR_NULL(chip->default_state)) {
		ret = PTR_ERR(chip->default_state);
		MOTOR_ERR("%s err:%d\n", __func__, ret);
		return ret;
	}

	pinctrl_select_state(chip->pctrl, chip->default_state);

	return 0;
}

static int drv8834_init_m1_gpio(struct oplus_mdrv_chip *chip)
{
	int ret = 0;

	if (IS_ERR_OR_NULL(chip->pctrl)) {
		ret = PTR_ERR(chip->pctrl);
		MOTOR_ERR("failed to get pinctrl\n");
		return ret;
	}

	chip->m1_state = pinctrl_lookup_state(chip->pctrl, "m1_gpio");

	if (IS_ERR_OR_NULL(chip->m1_state)) {
		ret = PTR_ERR(chip->m1_state);
		MOTOR_ERR("pinctrl_lookup_state, err:%d\n", ret);
		return ret;
	}

	pinctrl_select_state(chip->pctrl, chip->m1_state);

	return 0;
}

static int drv8834_init_active_config(struct oplus_mdrv_chip *chip)
{
	int ret = 0;

	if (IS_ERR_OR_NULL(chip->pctrl)) {
		ret = PTR_ERR(chip->pctrl);
		MOTOR_ERR("failed to get pinctrl\n");
		return ret;
	}

	chip->active_state = pinctrl_lookup_state(chip->pctrl, "motor_active");

	if (IS_ERR_OR_NULL(chip->active_state)) {
		ret = PTR_ERR(chip->active_state);
		MOTOR_ERR("%s err:%d\n", __func__, ret);
		return ret;
	}

	return 0;
}

static int drv8834_init_sleep_config(struct oplus_mdrv_chip *chip)
{
	int ret = 0;

	if (IS_ERR_OR_NULL(chip->pctrl)) {
		ret = PTR_ERR(chip->pctrl);
		MOTOR_ERR("failed to get pinctrl\n");
		return ret;
	}

	chip->sleep_state = pinctrl_lookup_state(chip->pctrl, "motor_sleep");

	if (IS_ERR_OR_NULL(chip->sleep_state)) {
		ret = PTR_ERR(chip->sleep_state);
		MOTOR_ERR("%s err:%d\n", __func__, ret);
		return ret;
	}

	return 0;
}

static int drv8834_pctrl_config(bool active)
{
	if (IS_ERR_OR_NULL(g_chip->pctrl) ||
			IS_ERR_OR_NULL(g_chip->active_state) ||
			IS_ERR_OR_NULL(g_chip->sleep_state)) {
		return -1;
	}

	MOTOR_LOG("active %d\n", active);

	if (active) {
		pinctrl_select_state(g_chip->pctrl, g_chip->active_state);

	} else {
		pinctrl_select_state(g_chip->pctrl, g_chip->sleep_state);
	}

	return 0;
}

static int drv8834_change_m0_config(struct oplus_mdrv_chip *chip , int config)
{
	int ret = 0;

	if (IS_ERR_OR_NULL(chip->pctrl)) {
		ret = PTR_ERR(chip->pctrl);
		MOTOR_ERR("failed to get pinctrl\n");
		return ret;
	}

	if (config == GPIO_MODE) {
		chip->m0_state = pinctrl_lookup_state(chip->pctrl, "m0_gpio");

		if (IS_ERR_OR_NULL(chip->m0_state)) {
			ret = PTR_ERR(chip->m0_state);
			MOTOR_ERR("pinctrl_lookup_state, err:%d\n", ret);
			return ret;
		}

		pinctrl_select_state(chip->pctrl, chip->m0_state);

	} else {
		chip->m0_state = pinctrl_lookup_state(chip->pctrl, "m0_high_impedance");

		if (IS_ERR_OR_NULL(chip->m0_state)) {
			ret = PTR_ERR(chip->m0_state);
			MOTOR_ERR("pinctrl_lookup_state, err:%d\n", ret);
			return ret;
		}

		pinctrl_select_state(chip->pctrl, chip->m0_state);
	}

	return 0;
}

static int drv8834_change_vref_config(struct oplus_mdrv_chip *chip , int config)
{
	int ret = 0;

	if (IS_ERR_OR_NULL(chip->pctrl)) {
		ret = PTR_ERR(chip->pctrl);
		MOTOR_ERR("failed to get pinctrl\n");
		return ret;
	}

	if (config == GPIO_MODE) {
		chip->vref_state = pinctrl_lookup_state(chip->pctrl, "vref_gpio");

		if (IS_ERR_OR_NULL(chip->vref_state)) {
			ret = PTR_ERR(chip->vref_state);
			MOTOR_ERR("pinctrl_lookup_state, err:%d\n", ret);
			return ret;
		}

		pinctrl_select_state(chip->pctrl, chip->vref_state);

	} else {
		chip->vref_state = pinctrl_lookup_state(chip->pctrl, "vref_high_impedance");

		if (IS_ERR_OR_NULL(chip->vref_state)) {
			ret = PTR_ERR(chip->vref_state);
			MOTOR_ERR("pinctrl_lookup_state, err:%d\n", ret);
			return ret;
		}

		pinctrl_select_state(chip->pctrl, chip->vref_state);
	}

	return 0;
}

static int drv8834_hardware_init(struct oplus_mdrv_chip *chip)
{
	int ret = 0;

	/*default state, only set when driver probe*/
	ret = drv8834_init_default(chip);

	if (ret < 0) {
		MOTOR_ERR("failed to init default\n");
	}

	/*sleep config, set when driver power on*/
	ret = drv8834_init_sleep_config(chip);

	if (ret < 0) {
		MOTOR_ERR("failed to init sleep config %d\n", ret);
	}

	/*active config, set when power on*/
	ret = drv8834_init_active_config(chip);

	if (ret < 0) {
		MOTOR_ERR("failed to init active config %d\n", ret);
	}

	/*set active when probe*/
	drv8834_pctrl_config(true);

	/*config m1_gpio*/
	ret = drv8834_init_m1_gpio(chip);

	if (ret < 0) {
		MOTOR_ERR("drv8834_init_m1_gpio %d \n", ret);
	}

	/*config m0_gpio as GPIO_MODE*/
	ret = drv8834_change_m0_config(chip, GPIO_MODE);

	if (ret < 0) {
		MOTOR_ERR("drv8834_change_m0_config %d \n", ret);
	}

	/*config vref_gpio as GPIO_MODE*/
	ret = drv8834_change_vref_config(chip, GPIO_MODE);

	if (ret < 0) {
		MOTOR_ERR("drv8834_change_vref_config %d \n", ret);
	}

	/*config dir_switch _gpio*/
	ret = drv8834_init_dir_switch(chip);

	if (ret < 0) {
		MOTOR_ERR("drv8834_init_dir_switch %d \n", ret);
	}

	drv8834_check_motor_type(chip);
	return 0;
}

static int drv8834_set_power(int mode)
{
	if (g_chip == NULL) {
		MOTOR_LOG("g_chip null \n ");
		return -EINVAL;
	}

	MOTOR_ERR("drv8834_set_power call mode: %d power on:%d\n", mode,
		  MOTOR_POWER_ON);

	if (mode == MOTOR_POWER_ON) {
		oplus_gpio_set_value(g_chip->boost_gpio, 1);/*boost should be always on*/
		oplus_gpio_set_value(g_chip->sleep_gpio, 1);
		oplus_gpio_set_value(g_chip->sleep1_gpio, 1);

	} else {
		oplus_gpio_set_value(g_chip->sleep_gpio, 0);
		oplus_gpio_set_value(g_chip->sleep1_gpio, 0);
		oplus_gpio_set_value(g_chip->step_gpio, 0);
		oplus_gpio_set_value(g_chip->dir_gpio, 0);
	}

	return 0;
}

static int drv8834_set_direction(int dir)
{
	if (g_chip == NULL) {
		MOTOR_LOG("g_chip null \n ");
		return -EINVAL;
	}

	if ((g_chip->dir_switch != ERROR_NO) && (g_chip->dir_switch)) {/*for findx*/
		if (dir == MOTOR_UPWARD) {
			oplus_gpio_set_value(g_chip->dir_gpio, 1);

		} else {
			oplus_gpio_set_value(g_chip->dir_gpio, 0);
		}

	} else {
		if (oplus_get_dir_sign() == POSITIVE) {
			if (dir == MOTOR_UPWARD) {
				oplus_gpio_set_value(g_chip->dir_gpio, 1);

			} else {
				oplus_gpio_set_value(g_chip->dir_gpio, 0);
			}

		} else {
			if (dir == MOTOR_UPWARD) {
				oplus_gpio_set_value(g_chip->dir_gpio, 0);

			} else {
				oplus_gpio_set_value(g_chip->dir_gpio, 1);
			}
		}
	}

	return 0;
}

static int drv8834_set_working_mode(int mode)
{
	int ret = 0;

	if (g_chip == NULL) {
		MOTOR_LOG("g_chip null \n ");
		return -EINVAL;
	}

	switch (mode) {
	case MOTOR_MODE_FULL:
		MOTOR_ERR("MOTOR_MODE_FULL call \n");
		ret = drv8834_change_vref_config(g_chip, GPIO_MODE);

		if (ret < 0) {
			MOTOR_ERR("drv8834_change_vref_config %d \n", ret);
			return -EINVAL;
		}

		ret = drv8834_change_m0_config(g_chip, GPIO_MODE);

		if (ret < 0) {
			MOTOR_ERR("drv8834_change_m0_config %d \n", ret);
			return -EINVAL;
		}

		oplus_gpio_set_value(g_chip->vref_gpio, 0);
		oplus_gpio_set_value(g_chip->m0_gpio, 0);
		oplus_gpio_set_value(g_chip->m1_gpio, 0);
		break;

	case MOTOR_MODE_1_16:
		MOTOR_ERR("MOTOR_MODE_1_16 call \n");
		ret = drv8834_change_vref_config(g_chip, HIGH_IMPEDANCE_MODE);

		if (ret < 0) {
			MOTOR_ERR("drv8834_change_vref_config %d \n", ret);
			return -EINVAL;
		}

		ret = drv8834_change_m0_config(g_chip, GPIO_MODE);

		if (ret < 0) {
			MOTOR_ERR("drv8834_change_m0_config %d \n", ret);
			return -EINVAL;
		}

		oplus_gpio_set_value(g_chip->m0_gpio, 1);
		oplus_gpio_set_value(g_chip->m1_gpio, 1);
		break;

	case MOTOR_MODE_1_32:
		MOTOR_ERR("MOTOR_MODE_1_32 call \n");
		ret = drv8834_change_vref_config(g_chip, HIGH_IMPEDANCE_MODE);

		if (ret < 0) {
			MOTOR_ERR("drv8834_change_vref_config %d \n", ret);
			return -EINVAL;
		}

		ret = drv8834_change_m0_config(g_chip, HIGH_IMPEDANCE_MODE);

		if (ret < 0) {
			MOTOR_ERR("drv8834_change_m0_config %d \n", ret);
			return -EINVAL;
		}

		oplus_gpio_set_value(g_chip->m1_gpio, 1);
		break;

	default:
		MOTOR_ERR("MOTOR_MODE_default call \n");
		ret = drv8834_change_vref_config(g_chip, HIGH_IMPEDANCE_MODE);

		if (ret < 0) {
			MOTOR_ERR("drv8834_change_vref_config %d \n", ret);
			return -EINVAL;
		}

		ret = drv8834_change_m0_config(g_chip, HIGH_IMPEDANCE_MODE);

		if (ret < 0) {
			MOTOR_ERR("drv8834_change_m0_config %d \n", ret);
			return -EINVAL;
		}

		oplus_gpio_set_value(g_chip->m1_gpio, 1);
		break;
	}

	MOTOR_ERR("config change %d %d %d %d %d %d\n",
		  oplus_gpio_get_value(g_chip->sleep_gpio),
		  oplus_gpio_get_value(g_chip->step_gpio),
		  oplus_gpio_get_value(g_chip->m0_gpio),
		  oplus_gpio_get_value(g_chip->m1_gpio),
		  oplus_gpio_get_value(g_chip->vref_gpio),
		  oplus_gpio_get_value(g_chip->dir_gpio));
	return 0;
}


static int drv8834_calculate_pwm_count(int L, int mode)
{
	int pwm_count = 0;
	int mdmode = 0;

	if (g_chip == NULL) {
		MOTOR_LOG("g_chip null \n");
		return 0;
	}

	switch (mode) {
	case MOTOR_MODE_FULL:
		mdmode = 1;
		break;

	case MOTOR_MODE_1_16:
		mdmode = 16;
		break;

	case MOTOR_MODE_1_32:
		mdmode = 32;
		break;

	default:
		mdmode = 32;
		break;
	}

	pwm_count = (L * g_chip->ratio_b * g_chip->ratio_c * g_chip->ratio_d) /
		    (g_chip->ratio_a * 100);

	return pwm_count;
}

#ifdef CONFIG_MTK_PLATFORM
static int mtk_pwm_config(int duty_ns, int period_ns, uint32_t wave_num)
{
	int l_duration, h_duration;

	l_duration = h_duration = 13 * period_ns / (2 * 1000) -
				  1; /* (26*period_ns)/(2*2*1000) - 1 */
	MOTOR_LOG("period %d, l_duration %d, h_duration %d, wave %d\n",
		  period_ns, l_duration, h_duration, wave_num);

	g_chip->pwm_setting.pwm_no = PWM1;
	g_chip->pwm_setting.mode = PWM_MODE_FIFO;
	g_chip->pwm_setting.clk_div = CLK_DIV2;
	g_chip->pwm_setting.clk_src = PWM_CLK_NEW_MODE_BLOCK;  /*26MHZ*/
	g_chip->pwm_setting.pmic_pad = false;
	g_chip->pwm_setting.PWM_MODE_FIFO_REGS.IDLE_VALUE = false;
	g_chip->pwm_setting.PWM_MODE_FIFO_REGS.GUARD_VALUE = false;
	g_chip->pwm_setting.PWM_MODE_FIFO_REGS.STOP_BITPOS_VALUE = 1;
	g_chip->pwm_setting.PWM_MODE_FIFO_REGS.HDURATION = h_duration;
	g_chip->pwm_setting.PWM_MODE_FIFO_REGS.LDURATION = l_duration;
	g_chip->pwm_setting.PWM_MODE_FIFO_REGS.GDURATION = 0;
	g_chip->pwm_setting.PWM_MODE_FIFO_REGS.SEND_DATA0 = 0xAAAAAAAA;
	g_chip->pwm_setting.PWM_MODE_FIFO_REGS.SEND_DATA1 = 0xAAAAAAAA;
	g_chip->pwm_setting.PWM_MODE_FIFO_REGS.WAVE_NUM = wave_num;

	return 0;
}

static int mtk_pwm_enable(void)
{
	return pwm_set_spec_config(&g_chip->pwm_setting);
}

static void mtk_pwm_disable(void)
{
	mt_pwm_disable(g_chip->pwm_setting.pwm_no, false);
}
#endif

static int drv8834_pwm_config(int duty_ns, int period_ns, uint32_t wave_num)
{
	int pwm_count = 0;
	int mdmode = 0;

	if (g_chip == NULL) {
		MOTOR_LOG("g_chip null \n");
		return -EINVAL;
	}

#ifdef CONFIG_MTK_PLATFORM
	return mtk_pwm_config(duty_ns, period_ns, wave_num);
#else
	return pwm_config(g_chip->pwm_dev, duty_ns, period_ns);
#endif
}

static int drv8834_pwm_enable(void)
{
	if (g_chip == NULL) {
		MOTOR_LOG("g_chip null \n");
		return -EINVAL;
	}

#ifdef CONFIG_MTK_PLATFORM
	return mtk_pwm_enable();
#else
	return pwm_enable(g_chip->pwm_dev);
#endif
}

static int drv8834_pwm_disable(void)
{
	if (g_chip == NULL) {
		MOTOR_LOG("g_chip null \n");
		return -EINVAL;
	}

#ifdef CONFIG_MTK_PLATFORM
	mtk_pwm_disable();
#else
	pwm_disable(g_chip->pwm_dev);
#endif
	return 0;
}


static void drv8834_check_motor_type(struct oplus_mdrv_chip *chip)
{
	chip->motor_type = oplus_get_motor_type();

	MOTOR_LOG("motor_type %d \n", chip->motor_type);
}

static int  drv8834_get_all_config(int *config , int count)
{
	if (g_chip == NULL || count > 6) {
		MOTOR_LOG("g_chip null \n");
		return -EINVAL;
	}

	config[0] = oplus_gpio_get_value(g_chip->sleep_gpio);
	config[1] = oplus_gpio_get_value(g_chip->step_gpio);
	config[2] = oplus_gpio_get_value(g_chip->m0_gpio);
	config[3] = oplus_gpio_get_value(g_chip->m1_gpio);
	config[4] = oplus_gpio_get_value(g_chip->vref_gpio);
	config[5] = oplus_gpio_get_value(g_chip->dir_gpio);

	MOTOR_ERR("config change %d %d %d %d %d %d\n", config[0], config[1], config[2],
		  config[3], config[4], config[5]);

	return 0;
}

struct oplus_motor_operations  drv8834_ops = {
	.set_power = drv8834_set_power,
	.set_direction = drv8834_set_direction,
	.set_working_mode = drv8834_set_working_mode,
	.get_all_config = drv8834_get_all_config,
	.calculate_pwm_count = drv8834_calculate_pwm_count,
	.pwm_config = drv8834_pwm_config,
	.pwm_enable = drv8834_pwm_enable,
	.pwm_disable = drv8834_pwm_disable,
	.pctrl_config = drv8834_pctrl_config,
};

static int drv8834_platform_probe(struct platform_device *pdev)
{
	struct oplus_mdrv_chip *chip = NULL;
	int ret = 0;
	int id = -1;

	MOTOR_LOG("call\n");

	chip = devm_kzalloc(&pdev->dev, sizeof(struct oplus_mdrv_chip), GFP_KERNEL);

	if (!chip) {
		MOTOR_ERR("kernel memory alocation was failed");
		return -ENOMEM;
	}

	chip->dev = &pdev->dev;

	oplus_register_motor("drv8834", &drv8834_ops);
	id = oplus_get_driver_ic_id();

	if (id != DRV8834) {
		MOTOR_LOG("drv8834_init fail,id %d\n", id);
		goto fail;
	}

	g_chip = chip;

	ret = drv8834_parse_dts(chip);

	if (ret < 0) {
		MOTOR_ERR("drv8834_hardware_init %d \n", ret);
		goto fail;
	}

	ret = drv8834_hardware_init(chip);

	if (ret < 0) {
		MOTOR_ERR("drv8834_hardware_init %d \n", ret);
		goto fail;
	}

	MOTOR_LOG("success \n");
	return 0;

fail:
	MOTOR_ERR("fail \n");
	oplus_unregister_motor("drv8834");
	return -EINVAL;
}

static int drv8834_platform_remove(struct platform_device *pdev)
{
	if (g_chip) {
		oplus_gpio_free(g_chip->boost_gpio);
		oplus_gpio_free(g_chip->vref_gpio);
		oplus_gpio_free(g_chip->dir_gpio);
		oplus_gpio_free(g_chip->m0_gpio);
		oplus_gpio_free(g_chip->m1_gpio);
		oplus_gpio_free(g_chip->sleep_gpio);
		oplus_gpio_free(g_chip->sleep1_gpio);
		kfree(g_chip);
		g_chip = NULL;
	}

	return 0;
}

static const struct of_device_id of_motor_drv_match[] = {
	{ .compatible = "motor_drv-8834"},
	{},
};
MODULE_DEVICE_TABLE(of, of_motor_match);

static struct platform_driver motor_drv_driver = {
	.probe		= drv8834_platform_probe,
	.remove		= drv8834_platform_remove,
	.driver		= {
		.name	= "drv8834",
		.of_match_table = of_motor_drv_match,
	},
};

static int __init drv8834_init(void)
{
	MOTOR_LOG("call\n");
	platform_driver_register(&motor_drv_driver);
	return 0;
}

static void __exit drv8834_exit(void)
{
	MOTOR_LOG("call\n");
}

module_init(drv8834_init);
module_exit(drv8834_exit);
MODULE_DESCRIPTION("camera motor driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("mofei");

