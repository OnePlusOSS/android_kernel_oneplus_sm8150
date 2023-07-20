/************************************************************************************
** Copyright (C), 2008-2017, OPLUS Mobile Comm Corp., Ltd
** VENDOR_EDIT
** File: oplus_stspin220.c
**
** Description:
**	Definitions for motor driver ic stspin220.
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

#include "oplus_stspin220.h"
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
static struct oplus_sts_chip *g_chip = NULL;
static void stspin220_check_motor_type(struct oplus_sts_chip *chip);
static int stspin220_set_working_mode(int mode);
static int stspin220_set_direction(int dir);

static int stspin220_parse_dts(struct oplus_sts_chip *chip)
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

		/*else
			oplus_gpio_set_value(chip->step_gpio, 0);*/
	}

	chip->enable_gpio = of_get_named_gpio(np, "enable-gpio", 0);

	if (!gpio_is_valid(chip->enable_gpio)) {
		MOTOR_LOG("md-enable-gpio gpio not specified\n");

	} else {
		rc = gpio_request(chip->enable_gpio, "enable-gpio");

		if (rc) {
			MOTOR_LOG("request md-enavle-gpio gpio failed, rc=%d\n", rc);

		} else {
			oplus_gpio_set_value(chip->enable_gpio, 0);
		}
	}

	chip->standby_gpio = of_get_named_gpio(np, "standby-gpio", 0);

	if (!gpio_is_valid(chip->standby_gpio)) {
		MOTOR_LOG("standby-gpio gpio not specified\n");

	} else {
		rc = gpio_request(chip->standby_gpio, "standby-gpio");

		if (rc) {
			MOTOR_LOG("request standby-gpio failed, rc=%d\n", rc);

		} else {
			oplus_gpio_set_value(chip->standby_gpio, 0);
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

	if (oplus_is_support_mode()) {
		chip->mode1_gpio = of_get_named_gpio(np, "mode1-gpio", 0);

		if (!gpio_is_valid(chip->mode1_gpio)) {
			MOTOR_LOG("mode1 gpio not specified\n");

		} else {
			rc = gpio_request(chip->mode1_gpio, "mode1-gpio");

			if (rc) {
				MOTOR_LOG("request mode1 gpio failed, rc=%d\n", rc);

			} else {
				oplus_gpio_set_value(chip->mode1_gpio, 0);
			}
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

	MOTOR_LOG("%d %d %d %d %d [%d %d %d %d]\n", chip->boost_gpio,
		  chip->standby_gpio,
		  chip->dir_gpio, chip->step_gpio, chip->enable_gpio,
		  chip->ratio_a, chip->ratio_b, chip->ratio_c, chip->ratio_d);

	return 0;
}

static int stspin220_init_boost(struct oplus_sts_chip *chip)
{
	int ret = 0;

	if (IS_ERR_OR_NULL(chip->pctrl)) {
		ret = PTR_ERR(chip->pctrl);
		MOTOR_ERR("failed to get pinctrl\n");
		return ret;
	}

	chip->boost_state = pinctrl_lookup_state(chip->pctrl, "boost");

	if (IS_ERR_OR_NULL(chip->boost_state)) {
		ret = PTR_ERR(chip->boost_state);
		MOTOR_ERR("pinctrl_lookup_state, err:%d\n", ret);
		return ret;
	}

	pinctrl_select_state(chip->pctrl, chip->boost_state);

	return 0;
}

static int stspin220_init_standby_gpio(struct oplus_sts_chip *chip)
{
	int ret = 0;

	if (IS_ERR_OR_NULL(chip->pctrl)) {
		ret = PTR_ERR(chip->pctrl);
		MOTOR_ERR("failed to get pinctrl\n");
		return ret;
	}

	chip->standby_state = pinctrl_lookup_state(chip->pctrl, "standby_gpio");

	if (IS_ERR_OR_NULL(chip->standby_state)) {
		ret = PTR_ERR(chip->standby_state);
		MOTOR_ERR("pinctrl_lookup_state, err:%d\n", ret);
		return ret;
	}

	pinctrl_select_state(chip->pctrl, chip->standby_state);

	return 0;
}

static int stspin220_init_enable_gpio(struct oplus_sts_chip *chip)
{
	int ret = 0;

	if (IS_ERR_OR_NULL(chip->pctrl)) {
		ret = PTR_ERR(chip->pctrl);
		MOTOR_ERR("failed to get pinctrl\n");
		return ret;
	}

	chip->enable_state = pinctrl_lookup_state(chip->pctrl, "enable_gpio");

	if (IS_ERR_OR_NULL(chip->enable_state)) {
		ret = PTR_ERR(chip->enable_state);
		MOTOR_ERR("pinctrl_lookup_state, err:%d\n", ret);
		return ret;
	}

	pinctrl_select_state(chip->pctrl, chip->enable_state);

	return 0;
}

static int stspin220_init_pwm_config(struct oplus_sts_chip *chip ,
				     int is_pwm_mode)
{
	int ret = 0;

	if (IS_ERR_OR_NULL(chip->pctrl)) {
		ret = PTR_ERR(chip->pctrl);
		MOTOR_ERR("failed to get pinctrl\n");
		return ret;
	}

	if (is_pwm_mode) {
		chip->pwm_state = pinctrl_lookup_state(chip->pctrl, "pwm_config");

	} else {
		chip->pwm_state = pinctrl_lookup_state(chip->pctrl, "pwm_config_as_gpio");
	}

	if (IS_ERR_OR_NULL(chip->pwm_state)) {
		ret = PTR_ERR(chip->pwm_state);
		MOTOR_ERR("pinctrl_lookup_state, err:%d\n", ret);
		return ret;
	}

	pinctrl_select_state(chip->pctrl, chip->pwm_state);

	return 0;
}

static int stspin220_init_dir_gpio(struct oplus_sts_chip *chip)
{
	int ret = 0;

	if (IS_ERR_OR_NULL(chip->pctrl)) {
		ret = PTR_ERR(chip->pctrl);
		MOTOR_ERR("failed to get pinctrl\n");
		return ret;
	}

	chip->dir_state = pinctrl_lookup_state(chip->pctrl, "dir_gpio");

	if (IS_ERR_OR_NULL(chip->dir_state)) {
		ret = PTR_ERR(chip->dir_state);
		MOTOR_ERR("pinctrl_lookup_state, err:%d\n", ret);
		return ret;
	}

	pinctrl_select_state(chip->pctrl, chip->dir_state);

	return 0;
}

static int stspin220_init_mode1_gpio(struct oplus_sts_chip *chip)
{
	int ret = 0;

	if (oplus_is_support_mode() == false) {
		return 0;
	}

	if (IS_ERR_OR_NULL(chip->pctrl)) {
		ret = PTR_ERR(chip->pctrl);
		MOTOR_ERR("failed to get pinctrl\n");
		return ret;
	}

	chip->mode1_state = pinctrl_lookup_state(chip->pctrl, "mode1_gpio");

	if (IS_ERR_OR_NULL(chip->mode1_state)) {
		ret = PTR_ERR(chip->mode1_state);
		MOTOR_ERR("pinctrl_lookup_state, err:%d\n", ret);
		return ret;
	}

	pinctrl_select_state(chip->pctrl, chip->mode1_state);

	return 0;
}

static int stspin220_init_active_config(struct oplus_sts_chip *chip)
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

static int stspin220_init_sleep_config(struct oplus_sts_chip *chip)
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

static int stspin220_pctrl_config(bool active)
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

static int stspin220_hardware_init(struct oplus_sts_chip *chip)
{
	int ret = 0;

	/*sleep config, set when driver power on*/
	ret = stspin220_init_sleep_config(chip);

	if (ret < 0) {
		MOTOR_ERR("failed to init sleep config %d\n", ret);
	}

	/*active config, set when power on*/
	ret = stspin220_init_active_config(chip);

	if (ret < 0) {
		MOTOR_ERR("failed to init active config %d\n", ret);
	}

	/*init boost*/
	ret = stspin220_init_boost(chip);

	if (ret < 0) {
		MOTOR_ERR("stspin220_init_boost %d \n", ret);
	}

	oplus_gpio_set_value(chip->boost_gpio, 0);

	/*config pwm for standby-gpio*/
	ret = stspin220_init_standby_gpio(chip);

	if (ret < 0) {
		MOTOR_ERR("stspin220_init_standby_gpio %d \n", ret);
	}

	/*config enable_gpio _gpio*/
	ret = stspin220_init_enable_gpio(chip);

	if (ret < 0) {
		MOTOR_ERR("stspin220_init_enable_gpio %d \n", ret);
	}

	/*config pwm for step-gpio*/
	ret = stspin220_init_pwm_config(chip, 0);

	if (ret < 0) {
		MOTOR_ERR("stspin220_init_pwm_config %d \n", ret);
	}

	/*config dir_gpio*/
	ret = stspin220_init_dir_gpio(chip);

	if (ret < 0) {
		MOTOR_ERR("stspin220_init_dir_gpio %d \n", ret);
	}

	/*config mode1*/
	ret = stspin220_init_mode1_gpio(chip);

	if (ret < 0) {
		MOTOR_ERR("stspin220_init_mode1_gpio %d \n", ret);
	}

	stspin220_check_motor_type(chip);

	return 0;
}

static int stspin220_set_power(int mode)
{
	static bool first_init = true;

	if (g_chip == NULL) {
		MOTOR_LOG("g_chip null \n ");
		return -EINVAL;
	}

	MOTOR_ERR("stspin220_set_power call mode: %d power on:%d\n", mode,
		  MOTOR_POWER_ON);

	if (mode == MOTOR_POWER_ON) {
		if (first_init) {
			first_init = false;
			oplus_gpio_set_value(g_chip->boost_gpio, 1);/*boost should be always on*/
			msleep(10);
		}

		stspin220_set_working_mode(MOTOR_MODE_1_32);
		mdelay(5);
		stspin220_init_pwm_config(g_chip, 1);
		oplus_gpio_set_value(g_chip->enable_gpio, 1);

	} else {
		oplus_gpio_set_value(g_chip->enable_gpio, 0);
		oplus_gpio_set_value(g_chip->standby_gpio, 0);
	}

	return 0;
}

static int stspin220_set_direction(int dir)
{
	if (g_chip == NULL) {
		MOTOR_LOG("g_chip null \n ");
		return -EINVAL;
	}

	if (g_chip->md_dir != dir) {
		g_chip->md_dir = dir;
	}

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

	return 0;
}

static int stspin220_set_working_mode(int mode)
{
	int ret = 0;

	if (g_chip == NULL) {
		MOTOR_LOG("g_chip null \n ");
		return -EINVAL;
	}

	oplus_gpio_set_value(g_chip->enable_gpio, 0);
	oplus_gpio_set_value(g_chip->standby_gpio, 0);

	switch (mode) {
	case MOTOR_MODE_FULL:
		MOTOR_ERR("MOTOR_MODE_FULL call \n");
		stspin220_init_pwm_config(g_chip, 0);
		oplus_gpio_set_value(g_chip->dir_gpio, 0);
		break;

	case MOTOR_MODE_1_16:
		MOTOR_ERR("MOTOR_MODE_1_16 call \n");
		stspin220_init_pwm_config(g_chip, 0);
		oplus_gpio_set_value(g_chip->dir_gpio, 0);
		break;

	case MOTOR_MODE_1_32:
		MOTOR_ERR("MOTOR_MODE_1_32 call \n");
		stspin220_init_pwm_config(g_chip, 0);
		oplus_gpio_set_value(g_chip->dir_gpio, 0);
		break;

	default:
		MOTOR_ERR("MOTOR_MODE_default call \n");
		stspin220_init_pwm_config(g_chip, 0);
		oplus_gpio_set_value(g_chip->dir_gpio, 0);
		break;
	}

	usleep_range(14, 15);
	oplus_gpio_set_value(g_chip->standby_gpio, 1);

	MOTOR_ERR("config change %d %d\n",
		  oplus_gpio_get_value(g_chip->standby_gpio),
		  oplus_gpio_get_value(g_chip->dir_gpio));
	return 0;
}


static int stspin220_calculate_pwm_count(int L, int mode)
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
	MOTOR_LOG("period %d, l_duration %d, h_duration %d\n", period_ns, l_duration,
		  h_duration);
	MOTOR_LOG("wave number %d\n", wave_num);

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

static int stspin220_pwm_config(int duty_ns, int period_ns, uint32_t wave_num)
{
	int pwm_count = 0;
	int mdmode = 0;

	if (g_chip == NULL) {
		MOTOR_LOG("g_chip null \n");
		return -EINVAL;
	}

	MOTOR_LOG("duty_ns %d period_ns %d \n", duty_ns, period_ns);

#ifdef CONFIG_MTK_PLATFORM
	return mtk_pwm_config(duty_ns, period_ns, wave_num);
#else
	return pwm_config(g_chip->pwm_dev, duty_ns, period_ns);
#endif
}

static int stspin220_pwm_enable(void)
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

static int stspin220_pwm_disable(void)
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

static void stspin220_check_motor_type(struct oplus_sts_chip *chip)
{
	chip->motor_type = oplus_get_motor_type();

	MOTOR_LOG("motor_type %d \n", chip->motor_type);
}

static int  stspin220_get_all_config(int *config , int count)
{
	if (g_chip == NULL || count > 6) {
		MOTOR_LOG("g_chip null \n");
		return -EINVAL;
	}

	if (oplus_is_support_mode()) {
		config[0] = oplus_gpio_get_value(g_chip->mode1_gpio);

	} else {
		config[0] = 0;
	}

	config[1] = oplus_gpio_get_value(g_chip->step_gpio);
	config[2] = oplus_gpio_get_value(g_chip->standby_gpio);
	config[3] = oplus_gpio_get_value(g_chip->enable_gpio);
	config[4] = oplus_gpio_get_value(g_chip->dir_gpio);
	config[5] = g_chip->motor_type;

	MOTOR_ERR("config change %d %d %d %d %d %d\n", config[0], config[1], config[2],
		  config[3], config[4], config[5]);

	return 0;
}


struct oplus_motor_operations  stspin220_ops = {
	.set_power = stspin220_set_power,
	.set_direction = stspin220_set_direction,
	.set_working_mode = stspin220_set_working_mode,
	.get_all_config = stspin220_get_all_config,
	.calculate_pwm_count = stspin220_calculate_pwm_count,
	.pwm_config = stspin220_pwm_config,
	.pwm_enable = stspin220_pwm_enable,
	.pwm_disable = stspin220_pwm_disable,
	.pctrl_config = stspin220_pctrl_config,
};

static int stspin220_platform_probe(struct platform_device *pdev)
{
	struct oplus_sts_chip *chip = NULL;
	int ret = 0;
	int id = -1;

	MOTOR_LOG("call\n");

	chip = devm_kzalloc(&pdev->dev, sizeof(struct oplus_sts_chip), GFP_KERNEL);

	if (!chip) {
		MOTOR_ERR("kernel memory alocation was failed");
		return -ENOMEM;
	}

	chip->dev = &pdev->dev;

	oplus_register_motor("stspin220", &stspin220_ops);
	id = oplus_get_driver_ic_id();

	if (id != STSPIN220) {
		MOTOR_LOG("stspin220_init fail,id %d\n", id);
		goto fail;
	}

	ret = stspin220_parse_dts(chip);

	if (ret < 0) {
		MOTOR_ERR("stspin220_parse_dts %d \n", ret);
		goto fail;
	}

	ret = stspin220_hardware_init(chip);

	if (ret < 0) {
		MOTOR_ERR("stspin220_hardware_init %d \n", ret);
		goto fail;
	}

	g_chip = chip;

	MOTOR_LOG("success \n");
	return 0;

fail:
	MOTOR_ERR("fail \n");
	oplus_unregister_motor("stspin220");
	devm_kfree(&pdev->dev, chip);
	g_chip = NULL;
	return -EINVAL;
}

static int stspin220_platform_remove(struct platform_device *pdev)
{
	if (g_chip) {
		oplus_gpio_free(g_chip->boost_gpio);
		oplus_gpio_free(g_chip->dir_gpio);
		oplus_gpio_free(g_chip->enable_gpio);

		if (oplus_is_support_mode()) {
			oplus_gpio_free(g_chip->mode1_gpio);
		}

		oplus_gpio_free(g_chip->standby_gpio);
		kfree(g_chip);
		g_chip = NULL;
	}

	return 0;
}

static const struct of_device_id of_motor_drv_match[] = {
	{ .compatible = "motor_drv-220"},
	{},
};
MODULE_DEVICE_TABLE(of, of_motor_match);

static struct platform_driver motor_drv_driver = {
	.probe		= stspin220_platform_probe,
	.remove		= stspin220_platform_remove,
	.driver		= {
		.name	= "stspin220",
		.of_match_table = of_motor_drv_match,
	},
};

static int __init stspin220_init(void)
{
	MOTOR_LOG("call\n");
	platform_driver_register(&motor_drv_driver);
	return 0;
}

static void __exit stspin220_exit(void)
{
	MOTOR_LOG("call\n");
}

module_init(stspin220_init);
module_exit(stspin220_exit);
MODULE_DESCRIPTION("camera motor driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("mofei");

