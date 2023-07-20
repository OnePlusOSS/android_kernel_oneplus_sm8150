/************************************************************************************
** Copyright (C), 2008-2017, OPLUS Mobile Comm Corp., Ltd
** VENDOR_EDIT
** File: oplus_lt3572.c
**
** Description:
**	Definitions for motor driver ic lt3572.
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

#include "oplus_lt3572.h"
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
static struct oplus_lt_chip *g_chip = NULL;
static void lt3572_check_motor_type(struct oplus_lt_chip *chip);
static int lt3572_set_working_mode(int mode);
static int lt3572_set_direction(int dir);

static int lt3572_parse_dts(struct oplus_lt_chip *chip)
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

	chip->step_gpio = of_get_named_gpio(np, "step-gpio", 0);

	if (!gpio_is_valid(chip->step_gpio)) {
		MOTOR_LOG("md-step-gpio gpio not specified\n");

	} else {
		rc = gpio_request(chip->step_gpio, "step-gpio");

		if (rc) {
			MOTOR_LOG("request md-step-gpio failed, rc=%d\n", rc);
		}

		/*else
			oplus_gpio_set_value(chip->step_gpio, 0);
		*/
	}

	chip->shdn_gpio = of_get_named_gpio(np, "shdn-gpio", 0);

	if (!gpio_is_valid(chip->shdn_gpio)) {
		MOTOR_LOG("shdn_gpio gpio not specified\n");

	} else {
		rc = gpio_request(chip->shdn_gpio, "shdn_gpio");

		if (rc) {
			MOTOR_LOG("request shdn_gpio failed, rc=%d\n", rc);

		} else {
			oplus_gpio_set_value(chip->shdn_gpio, 0);
		}
	}

	chip->shdna_gpio = of_get_named_gpio(np, "shdna-gpio", 0);

	if (!gpio_is_valid(chip->shdna_gpio)) {
		MOTOR_LOG("shdna_gpio gpio not specified\n");

	} else {
		rc = gpio_request(chip->shdna_gpio, "shdna_gpio");

		if (rc) {
			MOTOR_LOG("request shdna_gpio failed, rc=%d\n", rc);

		} else {
			oplus_gpio_set_value(chip->shdna_gpio, 0);
		}
	}

	chip->shdnb_gpio = of_get_named_gpio(np, "shdnb-gpio", 0);

	if (!gpio_is_valid(chip->shdnb_gpio)) {
		MOTOR_LOG("shdnb_gpio gpio not specified\n");

	} else {
		rc = gpio_request(chip->shdnb_gpio, "shdnb_gpio");

		if (rc) {
			MOTOR_LOG("request shdnb_gpio failed, rc=%d\n", rc);

		} else {
			oplus_gpio_set_value(chip->shdnb_gpio, 0);
		}
	}

	rc = of_property_read_u32(np, "ratio-a", &chip->ratio_a);

	if (rc) {
		chip->ratio_a = RATIO_A;
	}

	MOTOR_LOG("%d %d %d %d [%d]\n", chip->step_gpio, chip->shdn_gpio,
		  chip->shdna_gpio, chip->shdnb_gpio, chip->ratio_a);

	return 0;
}

static int lt3572_init_pwm_config(struct oplus_lt_chip *chip)
{
	int ret = 0;

	if (IS_ERR_OR_NULL(chip->pctrl)) {
		ret = PTR_ERR(chip->pctrl);
		MOTOR_ERR("failed to get pinctrl\n");
		return ret;
	}

	chip->pwm_state = pinctrl_lookup_state(chip->pctrl, "pwm_config");

	if (IS_ERR_OR_NULL(chip->pwm_state)) {
		ret = PTR_ERR(chip->pwm_state);
		MOTOR_ERR("pinctrl_lookup_state, err:%d\n", ret);
		return ret;
	}

	pinctrl_select_state(chip->pctrl, chip->pwm_state);

	return 0;
}

static int lt3572_init_shdn_gpio(struct oplus_lt_chip *chip)
{
	int ret = 0;

	if (IS_ERR_OR_NULL(chip->pctrl)) {
		ret = PTR_ERR(chip->pctrl);
		MOTOR_ERR("failed to get pinctrl\n");
		return ret;
	}

	chip->shdn_state = pinctrl_lookup_state(chip->pctrl, "shdn_gpio");

	if (IS_ERR_OR_NULL(chip->shdn_state)) {
		ret = PTR_ERR(chip->shdn_state);
		MOTOR_ERR("pinctrl_lookup_state, err:%d\n", ret);
		return ret;
	}

	pinctrl_select_state(chip->pctrl, chip->shdn_state);

	return 0;
}

static int lt3572_init_shdna_gpio(struct oplus_lt_chip *chip)
{
	int ret = 0;

	if (IS_ERR_OR_NULL(chip->pctrl)) {
		ret = PTR_ERR(chip->pctrl);
		MOTOR_ERR("failed to get pinctrl\n");
		return ret;
	}

	chip->shdna_state = pinctrl_lookup_state(chip->pctrl, "shdna_gpio");

	if (IS_ERR_OR_NULL(chip->shdna_state)) {
		ret = PTR_ERR(chip->shdna_state);
		MOTOR_ERR("pinctrl_lookup_state, err:%d\n", ret);
		return ret;
	}

	pinctrl_select_state(chip->pctrl, chip->shdna_state);

	return 0;
}

static int lt3572_init_shdnb_gpio(struct oplus_lt_chip *chip)
{
	int ret = 0;

	if (IS_ERR_OR_NULL(chip->pctrl)) {
		ret = PTR_ERR(chip->pctrl);
		MOTOR_ERR("failed to get pinctrl\n");
		return ret;
	}

	chip->shdnb_state = pinctrl_lookup_state(chip->pctrl, "shdnb_gpio");

	if (IS_ERR_OR_NULL(chip->shdnb_state)) {
		ret = PTR_ERR(chip->shdnb_state);
		MOTOR_ERR("pinctrl_lookup_state, err:%d\n", ret);
		return ret;
	}

	pinctrl_select_state(chip->pctrl, chip->shdnb_state);

	return 0;
}

static int lt3572_hardware_init(struct oplus_lt_chip *chip)
{
	int ret = 0;

	chip->duty_ratio = RATIO_0_25;

	/*config pwm for step-gpio*/
	ret = lt3572_init_pwm_config(chip);

	if (ret < 0) {
		MOTOR_ERR("lt3572_init_pwm_config %d \n", ret);
	}

	/*config dir_gpio*/
	ret = lt3572_init_shdn_gpio(chip);

	if (ret < 0) {
		MOTOR_ERR("lt3572_init_shdn_gpio %d \n", ret);
	}

	/*config mode1*/
	ret = lt3572_init_shdna_gpio(chip);

	if (ret < 0) {
		MOTOR_ERR("lt3572_init_shdna_gpio %d \n", ret);
	}

	/*config mode1*/
	ret = lt3572_init_shdnb_gpio(chip);

	if (ret < 0) {
		MOTOR_ERR("lt3572_init_shdnb_gpio %d \n", ret);
	}

	lt3572_check_motor_type(chip);

	return 0;
}

static int lt3572_set_power(int mode)
{
	if (g_chip == NULL) {
		MOTOR_LOG("g_chip null \n ");
		return -EINVAL;
	}

	MOTOR_ERR("lt3572_set_power call mode: %d power on:%d\n", mode, MOTOR_POWER_ON);

	if (mode == MOTOR_POWER_ON) {
		lt3572_init_pwm_config(g_chip);
		oplus_gpio_set_value(g_chip->shdn_gpio, 1);
		oplus_gpio_set_value(g_chip->shdna_gpio, 1);
		oplus_gpio_set_value(g_chip->shdnb_gpio, 1);

	} else {
		oplus_gpio_set_value(g_chip->shdn_gpio, 0);
		oplus_gpio_set_value(g_chip->shdna_gpio, 0);
		oplus_gpio_set_value(g_chip->shdnb_gpio, 0);
	}

	return 0;
}

static void lt3572_set_duty_ratio(struct oplus_lt_chip *chip,
				  enum DUTY_RATIO ratio)
{
	chip->duty_ratio = ratio;
}

static int lt3572_set_direction(int dir)
{
	if (g_chip == NULL) {
		MOTOR_LOG("g_chip null \n ");
		return -EINVAL;
	}

	if (oplus_get_dir_sign() == POSITIVE) {
		if (dir == MOTOR_UPWARD) {
			lt3572_set_duty_ratio(g_chip, RATIO_0_25);

		} else {
			lt3572_set_duty_ratio(g_chip, RATIO_0_75);
		}

	} else {
		if (dir == MOTOR_UPWARD) {
			lt3572_set_duty_ratio(g_chip, RATIO_0_75);

		} else {
			lt3572_set_duty_ratio(g_chip, RATIO_0_25);
		}
	}

	return 0;
}

static int lt3572_set_working_mode(int mode)
{
	return 0;
}

static int lt3572_calculate_pwm_count(int L, int mode)
{
	int pwm_count = 0;

	if (g_chip == NULL) {
		MOTOR_LOG("g_chip null \n");
		return 0;
	}

	pwm_count = (L * 100000) / (g_chip->ratio_a);

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

static int lt3572_pwm_config(int duty_ns, int period_ns, uint32_t wave_num)
{
	int pwm_count = 0;
	int mdmode = 0;

	if (g_chip == NULL) {
		MOTOR_LOG("g_chip null \n");
		return -EINVAL;
	}

#if 1

	if (g_chip->duty_ratio == RATIO_0_25) {
		duty_ns = period_ns / 4;

	} else if (g_chip->duty_ratio == RATIO_0_75) {
		duty_ns = period_ns * 3 / 4;

	} else {
		duty_ns = period_ns / 2;
	}

#endif

	MOTOR_LOG("duty_ns %d period_ns %d \n", duty_ns, period_ns);

#ifdef CONFIG_MTK_PLATFORM
	return mtk_pwm_config(duty_ns, period_ns, wave_num);
#else
	return pwm_config(g_chip->pwm_dev, duty_ns, period_ns);
#endif
}

static int lt3572_pwm_enable(void)
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

static int lt3572_pwm_disable(void)
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

static void lt3572_check_motor_type(struct oplus_lt_chip *chip)
{
	chip->motor_type = oplus_get_motor_type();

	MOTOR_LOG("motor_type %d \n", chip->motor_type);
}

static int  lt3572_get_all_config(int *config , int count)
{
	if (g_chip == NULL || count > 6) {
		MOTOR_LOG("g_chip null \n");
		return -EINVAL;
	}

	config[0] = oplus_gpio_get_value(g_chip->step_gpio);
	config[1] = oplus_gpio_get_value(g_chip->shdn_gpio);
	config[2] = oplus_gpio_get_value(g_chip->shdna_gpio);
	config[3] = oplus_gpio_get_value(g_chip->shdnb_gpio);
	config[4] = g_chip->motor_type;

	MOTOR_ERR("config change %d %d %d %d %d %d\n", config[0], config[1], config[2],
		  config[3], config[4], config[5]);

	return 0;
}


struct oplus_motor_operations  lt3572_ops = {
	.set_power = lt3572_set_power,
	.set_direction = lt3572_set_direction,
	.set_working_mode = lt3572_set_working_mode,
	.get_all_config = lt3572_get_all_config,
	.calculate_pwm_count = lt3572_calculate_pwm_count,
	.pwm_config = lt3572_pwm_config,
	.pwm_enable = lt3572_pwm_enable,
	.pwm_disable = lt3572_pwm_disable,
};

static int lt3572_platform_probe(struct platform_device *pdev)
{
	struct oplus_lt_chip *chip = NULL;
	int ret = 0;
	int id = -1;

	MOTOR_LOG("call\n");

	chip = devm_kzalloc(&pdev->dev, sizeof(struct oplus_lt_chip), GFP_KERNEL);

	if (!chip) {
		MOTOR_ERR("kernel memory alocation was failed");
		return -ENOMEM;
	}

	chip->dev = &pdev->dev;

	oplus_register_motor("lt3572", &lt3572_ops);
	id = oplus_get_driver_ic_id();

	if (id != LT3572) {
		MOTOR_LOG("lt3572_init fail,id %d\n", id);
		goto fail;
	}

	ret = lt3572_parse_dts(chip);

	if (ret < 0) {
		MOTOR_ERR("lt3572_hardware_init %d \n", ret);
		goto fail;
	}

	ret = lt3572_hardware_init(chip);

	if (ret < 0) {
		MOTOR_ERR("lt3572_hardware_init %d \n", ret);
		goto fail;
	}

	g_chip = chip;

	MOTOR_LOG("success \n");
	return 0;

fail:
	MOTOR_ERR("fail \n");
	oplus_unregister_motor("lt3572");
	devm_kfree(&pdev->dev, chip);
	g_chip = NULL;
	return -EINVAL;
}

static int lt3572_platform_remove(struct platform_device *pdev)
{
	if (g_chip) {
		oplus_gpio_free(g_chip->step_gpio);
		oplus_gpio_free(g_chip->shdn_gpio);
		oplus_gpio_free(g_chip->shdna_gpio);
		oplus_gpio_free(g_chip->shdnb_gpio);
		kfree(g_chip);
		g_chip = NULL;
	}

	return 0;
}

static const struct of_device_id of_motor_drv_match[] = {
	{ .compatible = "motor_drv-3572"},
	{},
};
MODULE_DEVICE_TABLE(of, of_motor_match);

static struct platform_driver motor_drv_driver = {
	.probe		= lt3572_platform_probe,
	.remove		= lt3572_platform_remove,
	.driver		= {
		.name	= "lt3572",
		.of_match_table = of_motor_drv_match,
	},
};

static int __init lt3572_init(void)
{
	MOTOR_LOG("call\n");
	platform_driver_register(&motor_drv_driver);
	return 0;
}

static void __exit lt3572_exit(void)
{
	MOTOR_LOG("call\n");
}

module_init(lt3572_init);
module_exit(lt3572_exit);
MODULE_DESCRIPTION("camera motor driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("mofei");

