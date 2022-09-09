/*
 * TI LM3697 Backlight Driver
 *
 * Copyright 2014 Texas Instruments
 *
 * Author: Milo Kim <milo.kim@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_data/lm3697_bl.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <soc/oplus/boot_mode.h>
#include <soc/oplus/oplus_project.h>

/* Registers */
#define LM3697_REG_OUTPUT_CFG       0x10

#define LM3697_REG_BRT_CFG      0x16

#define LM3697_REG_BOOST_CTL    0x1A
#define LM3697_REG_BOOST        0x06

#define LM3697_REG_PWM_CFG      0x1C

#define LM3697_REG_IMAX_A       0x17
#define LM3697_REG_IMAX_B       0x18

#define LM3697_REG_BRT_A_LSB        0x20
#define LM3697_REG_BRT_A_MSB        0x21
#define LM3697_REG_BRT_B_LSB        0x22
#define LM3697_REG_BRT_B_MSB        0x23
#define LM3697_BRT_LSB_MASK     (BIT(0) | BIT(1) | BIT(2))
#define LM3697_BRT_MSB_SHIFT        3

#define LM3697_REG_ENABLE       0x24

/* Other definitions */
#define LM3697_PWM_ID           1
#define LM3697_MAX_REGISTERS        0xB4
#define LM3697_MAX_STRINGS      3
#define LM3697_MAX_BRIGHTNESS       2047 //2047
#define LM3697_MIN_BRIGHTNESS       4
#define LM3697_PWM_CLOSE_BRIGHTNESS 150
#define LM3697_IMAX_OFFSET      6
#define LM3697_DEFAULT_NAME     "lcd-backlight"
#define LM3697_DEFAULT_PWM      "lm3697-backlight"

#define LM3697_EXPONENTIAL      1
#define LM3697_LINEAR           0

#define LM3697_EXPONENTIAL_FTM_TEST_BRIGHTNESS                  1800
#define LM3697_EXPONENTIAL_FTM_TEST_LCM_CURRENT_BRIGHTNESS      180
#define LM3697_LINEAR_MAX_BRIGHTNESS                            255
#define BL_FULL_SCALE_LED_CURRENT (25)
#define BL_CURRENT_MAPPING_LINEAR (1)
#define KTD3136_BL_CURRENT_MAPPING_LINEAR_SHIFT (1)
#define LM3697_BL_CURRENT_MAPPING_LINEAR_SHIFT (0)

#define BL_IC_KTD3136 (1)
#define BRIGHTNESS_MAX       (4095)
enum {
	KTD3136_DEV_ID_REG = 0x00,
	KTD3136_SW_RESET_REG,
	KTD3136_MODE_REG,
	KTD3136_CTRL_REG,
	KTD3136_LED_CUR_RATIO_LSB_REG,
	KTD3136_LED_CUR_RATIO_MSB_REG,
	KTD3136_PWM_REG,
	KTD3136_TURNING_ON_OFF_RAMP_REG,
	KTD3136_TRANSITION_RAMP_REG,
	KTD3136_STATUS_REG = 0x0a
};

enum lm3697_bl_ctrl_mode {
	BL_REGISTER_BASED,
	BL_PWM_BASED,
};

/*
 * struct lm3697_bl_chip
 * @dev: Parent device structure
 * @regmap: Used for I2C register access
 * @pdata: LM3697 platform data
 */
struct lm3697_bl_chip {
	struct lm3697_platform_data *pdata;
	struct regmap *regmap;
	struct backlight_device *bl_dev;
	int ic_type; /* 1: ktd3136 */
	int en_gpio_state;
	int brightness;
};

/*
 * struct lm3697_bl
 * @bank_id: Control bank ID. BANK A or BANK A and B
 * @bl_dev: Backlight device structure
 * @chip: LM3697 backlight chip structure for low level access
 * @bl_pdata: LM3697 backlight platform data
 * @mode: Backlight control mode
 * @pwm: PWM device structure. Only valid in PWM control mode
 * @pwm_name: PWM device name
 */
struct lm3697_bl {
	int bank_id;
	struct backlight_device *bl_dev;
	struct lm3697_bl_chip *chip;
	struct lm3697_backlight_platform_data *bl_pdata;
	enum lm3697_bl_ctrl_mode mode;
	struct pwm_device *pwm;
	char pwm_name[20];
};

static struct i2c_client *lm3697_i2c_client = NULL;

static int lm3697_chip_init(struct lm3697_bl_chip *pchip);

#define CHIP_INIT_COUNT_MAX (3)
static int set_lm3697_backlight_gpio(int value)
{
	struct lm3697_bl_chip *pchip;
	int ctr = 0;
	int ret = 0;

	if(!lm3697_i2c_client) {
		pr_err("%s, lm3697_i2c_client = NULL\n", __func__);
		return -1;
	}
	pchip = i2c_get_clientdata(lm3697_i2c_client);

	if ((!pchip) || (!pchip->pdata)) {
		dev_err(&lm3697_i2c_client->dev, \
				"%s, pchip = NULL or pchip->pdata = NULL\n", __func__);
		return -1;
	}

	if (gpio_is_valid(pchip->pdata->en_gpio)) {
		if(value!=pchip->en_gpio_state) {
			dev_info(&lm3697_i2c_client->dev, "%s: bl_en_gpio = %d, value = %d\n", \
					__func__, pchip->pdata->en_gpio, value);

			gpio_direction_output(pchip->pdata->en_gpio, value);
			pchip->en_gpio_state = value;

			usleep_range(150, 150);

			if(!!(pchip->en_gpio_state)) {
				msleep(27);
				do {
					ctr++;
					ret = lm3697_chip_init(pchip);
					if(0 <= ret) {
						break;
					}
				} while(CHIP_INIT_COUNT_MAX >= ctr);
				if(CHIP_INIT_COUNT_MAX < ctr) {
					dev_err(&lm3697_i2c_client->dev, \
							"%s: lm3697_chip_init failed, count = %d\n", __func__, \
							ctr);
				}
			}
		}
	} else {
		dev_err(&lm3697_i2c_client->dev, \
				"%s: pchip->pdata->en_gpio is invalid!\n", __func__);
	}

	return ret;
}

static int lm3697_dt(struct device *dev, struct lm3697_platform_data *pdata)
{
	struct device_node *np = dev->of_node;

	pdata->en_gpio = of_get_named_gpio(np, "ti,bl-enable-gpio", 0);

	dev_info(dev, "%s bl_en_gpio = %d\n", __func__, pdata->en_gpio);

	if (!gpio_is_valid(pdata->en_gpio))
		dev_err(dev, "%s, Backlight_en gpio not specified\n", __func__);

	return 0;
}

static int lm3697_chip_init(struct lm3697_bl_chip *pchip)
{
	int ret = 0;

	dev_info(&lm3697_i2c_client->dev, "%s\n", __func__);

	if (BL_IC_KTD3136 == pchip->ic_type) {
		ret = regmap_write(pchip->regmap, 0x02, (BL_FULL_SCALE_LED_CURRENT << 3));  //OVP 32V, Freq 500kh
		if (ret < 0)
			goto out;

		ret = regmap_write(pchip->regmap, 0x03, (0x28 | (BL_CURRENT_MAPPING_LINEAR << KTD3136_BL_CURRENT_MAPPING_LINEAR_SHIFT)));    //Exponential Mapping Mode
		if (ret < 0)
			goto out;

		ret = regmap_write(pchip->regmap, 0x06, 0x23);  //Linear Mapping Mode
		if (ret < 0)
			goto out;

		ret = regmap_write(pchip->regmap, 0x07, 0x00);  //Bank A Full-scale current (17.8mA)
		if (ret < 0)
			goto out;

		ret = regmap_write(pchip->regmap, 0x08, 0x10);    //Bank B Full-scale current (17.8mA)
		if (ret < 0)
			goto out;
	} else {
		ret = regmap_write(pchip->regmap, 0x10, 0x04);  //HVLED1, 2, 3 enable
		if (ret < 0)
			goto out;

		ret = regmap_write(pchip->regmap, 0x1A, 0x04);  //OVP 32V, Freq 500kh
		if (ret < 0)
			goto out;

		ret = regmap_write(pchip->regmap, 0x16, BL_CURRENT_MAPPING_LINEAR << LM3697_BL_CURRENT_MAPPING_LINEAR_SHIFT);    //Exponential Mapping Mode
		if (ret < 0)
			goto out;

		ret = regmap_write(pchip->regmap, 0x19, 0x03);  //Linear Mapping Mode
		if (ret < 0)
			goto out;

		ret = regmap_write(pchip->regmap, 0x17, BL_FULL_SCALE_LED_CURRENT);  //Bank A Full-scale current (20.2mA)
		if (ret < 0)
			goto out;

		ret = regmap_write(pchip->regmap, 0x18, BL_FULL_SCALE_LED_CURRENT);    //Bank B Full-scale current (20.2mA)
		if (ret < 0)
			goto out;

		ret = regmap_write(pchip->regmap, 0x1C, 0x0D);	//Set PWM Open
		if (ret < 0)
			goto out;
	}

	return ret;
out:
	dev_err(&lm3697_i2c_client->dev, "%s, i2c failed to access register\n", \
			__func__);
	return ret;
}

static struct regmap_config lm3697_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = LM3697_MAX_REGISTERS,
};

struct _bl_map_rang {
	int start;
	int end;
};
struct _bl_map_fac {
	int fac;
	int mul;
};
struct _bl_map {
	struct _bl_map_rang user;
	struct _bl_map_rang ic;
	struct _bl_map_fac fac;
};
#define BL_MAP_NUM (6)
static struct _bl_map bl_map[BL_MAP_NUM] = {
	{{1, 411},     {7, 86},      {193009, 1000000}},
	{{411, 820},   {86, 226},    {341015, 1000000}},
	{{820, 1229},  {226, 484},   {630878, 1000000}},
	{{1229, 1638}, {484, 930},   {109125, 100000}},
	{{1638, 2047}, {930, 1646},  {175067, 100000}},
	{{2047, 4095}, {1646, 2047}, {195905, 1000000}},
};
static int calc_backlightness(int bl_lvl)
{
	int ret = 0;
	int i = 0;
	int find = -1;

	if(0 != bl_lvl) {
		for(i = 0; i < BL_MAP_NUM; i++) {
			if((bl_lvl>=bl_map[i].user.start) && (bl_lvl<bl_map[i].user.end)) {
				find = i;
				break;
			}
		}
		if(i >= BL_MAP_NUM) {
			ret = bl_map[BL_MAP_NUM-1].ic.end;
		} else {
			ret = bl_map[i].ic.start + (((bl_lvl-bl_map[i].user.start) * bl_map[i].fac.fac) /  bl_map[i].fac.mul);
			ret = min(ret, bl_map[i].ic.end);
		}
	}

	return ret;
}

/* update and get brightness */
static int lm3697_wled_update_status(struct backlight_device *bl)
{
	int ret = 0;
	struct lm3697_bl_chip *pchip = bl_get_data(bl);
	int bl_msb = 0;
	int bl_lsb = 0;

	if(!lm3697_i2c_client) {
		pr_err("%s, lm3697_i2c_client = NULL\n", __func__);
		return -1;
	}

	if ((!pchip) || (!pchip->pdata)) {
		dev_err(&lm3697_i2c_client->dev, \
				"%s, pchip = NULL or pchip->pdata = NULL\n", __func__);
		return -1;
	}

	if (!pchip->regmap) {
		dev_err(&lm3697_i2c_client->dev, "%s, pchip->regmap is NULL.\n", \
				__func__);
		return -1;
	}

	pchip->brightness = calc_backlightness(bl->props.brightness);
	bl_msb = (pchip->brightness >> 3) & 0xFF;
	bl_lsb = pchip->brightness & 0x07;

	if (BL_IC_KTD3136 == pchip->ic_type) {
		/* brightness 0 means disable */
		if (0 == bl->props.brightness) {
			if(pchip->en_gpio_state) {
				ret = regmap_write(pchip->regmap, \
						KTD3136_LED_CUR_RATIO_LSB_REG, bl_lsb);
				if (ret < 0)
					goto out;
				ret = regmap_write(pchip->regmap, \
						KTD3136_LED_CUR_RATIO_MSB_REG, bl_msb);
				if (ret < 0)
					goto out;

				ret = regmap_update_bits(pchip->regmap, KTD3136_MODE_REG, \
						0x01, 0x00);
				if (ret < 0)
					goto out;
			}

			msleep(2);

			ret = set_lm3697_backlight_gpio(0);
			if (ret < 0)
				goto out;
		} else {
			ret = set_lm3697_backlight_gpio(1);
			if (ret < 0)
				goto out;

			ret = regmap_write(pchip->regmap, KTD3136_LED_CUR_RATIO_LSB_REG, \
					bl_lsb);
			if (ret < 0)
				goto out;
			ret = regmap_write(pchip->regmap, KTD3136_LED_CUR_RATIO_MSB_REG, \
					bl_msb);
			if (ret < 0)
				goto out;

			ret = regmap_update_bits(pchip->regmap, KTD3136_MODE_REG, 0x01, \
					0x01);
			if (ret < 0)
				goto out;
		}
	} else {
		/* brightness 0 means disable */
		if (bl->props.brightness==0) {
			if(pchip->en_gpio_state) {
				ret = regmap_write(pchip->regmap, 0x20, bl_lsb);
				if (ret < 0)
					goto out;
				ret = regmap_write(pchip->regmap, 0x21, bl_msb);
				if (ret < 0)
					goto out;
				ret = regmap_write(pchip->regmap, 0x24, 0);
				if (ret < 0)
					goto out;
			}

			msleep(2);

			ret = set_lm3697_backlight_gpio(0);
			if (ret < 0)
				goto out;
		} else {
			ret = set_lm3697_backlight_gpio(1);
			if (ret < 0)
				goto out;

			ret = regmap_write(pchip->regmap, 0x20, bl_lsb);
			if (ret < 0)
				goto out;
			ret = regmap_write(pchip->regmap, 0x21, bl_msb);
			if (ret < 0)
				goto out;
			ret = regmap_write(pchip->regmap, 0x24, 1);
			if (ret < 0)
				goto out;
		}
	}

	dev_info(&lm3697_i2c_client->dev, "%s, target_brightness = %d, actual_brightness = %d\n", \
			__func__, bl->props.brightness, pchip->brightness);

	return 0;
out:
	dev_err(&lm3697_i2c_client->dev, "%s, i2c failed to access registers\n", __func__);
	return ret;
}

static int lm3697_wled_get_brightness(struct backlight_device *bl)
{
	int ret = 0;
	struct lm3697_bl_chip *pchip = bl_get_data(bl);
	unsigned int bl_msb;
	unsigned int bl_lsb;

	if(!lm3697_i2c_client) {
		pr_err("%s, lm3697_i2c_client = NULL\n", __func__);
		return 0;
	}

	if ((!pchip) || (!pchip->pdata)) {
		dev_err(&lm3697_i2c_client->dev, \
				"%s, pchip = NULL or pchip->pdata = NULL\n", __func__);
		return 0;
	}

	if (!pchip->regmap) {
		dev_err(&lm3697_i2c_client->dev, "%s, pchip->regmap is NULL.\n", \
				__func__);
		return 0;
	}

	if(!pchip->en_gpio_state) {
		dev_info(&lm3697_i2c_client->dev, "%s, en_gpio is off.\n", __func__);
		return 0;
	}

	if (-1 == pchip->brightness) {
		if(BL_IC_KTD3136 == pchip->ic_type) {
			ret = regmap_read(pchip->regmap, KTD3136_LED_CUR_RATIO_LSB_REG, \
					&bl_lsb);
			if (ret < 0)
				goto out;

			ret = regmap_read(pchip->regmap, KTD3136_LED_CUR_RATIO_MSB_REG, \
					&bl_msb);
			if (ret < 0)
				goto out;
		} else {
			ret = regmap_read(pchip->regmap, 0x20, \
					&bl_lsb);
			if (ret < 0)
				goto out;

			ret = regmap_read(pchip->regmap, 0x21, \
					&bl_msb);
			if (ret < 0)
				goto out;
		}

		pchip->brightness = ((bl_msb&0xff)<<3) | (bl_lsb&0x07);
	}

	bl->props.brightness = pchip->brightness;

	dev_info(&lm3697_i2c_client->dev, "%s, brightness = %d\n", __func__, \
			bl->props.brightness);

	return bl->props.brightness;
out:
	dev_err(&lm3697_i2c_client->dev, "%s, i2c failed to access register\n", __func__);
	return bl->props.brightness;
}

static const struct backlight_ops lm3697_wled_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = lm3697_wled_update_status,
	.get_brightness = lm3697_wled_get_brightness,
};

static int lm3697_bl_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct lm3697_platform_data *pdata = client->dev.platform_data;
	struct lm3697_bl_chip *pchip;
	unsigned int revision;
	int ret = 0;
	int ctr = 0;
	struct backlight_properties props;

	dev_info(&client->dev, "%s Enter\n", __func__);

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				sizeof(struct lm3697_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		ret = lm3697_dt(&client->dev, pdata);
		if (ret)
			return ret;
	} else
		pdata = client->dev.platform_data;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "fail : i2c functionality check...\n");
		return -EOPNOTSUPP;
	}

	if (pdata == NULL) {
		dev_err(&client->dev, "fail : no platform data.\n");
		return -ENODATA;
	}

	pchip = devm_kzalloc(&client->dev, sizeof(struct lm3697_bl_chip),
			GFP_KERNEL);
	if (!pchip) {
		dev_err(&client->dev, "no memory to alloc struct lm3697_bl_chip\n");
		return -ENOMEM;
	}

	pchip->pdata = pdata;
	lm3697_i2c_client = client;

	pchip->regmap = devm_regmap_init_i2c(client, &lm3697_regmap);
	if (IS_ERR(pchip->regmap)) {
		ret = PTR_ERR(pchip->regmap);
		dev_err(&client->dev, "fail : allocate register map: %d\n",
				ret);
		return ret;
	}

	i2c_set_clientdata(client, pchip);

	pchip->brightness = -1;
	pchip->en_gpio_state = 1; /* this gpio have been set in bootloader */
	ret = gpio_request(pdata->en_gpio, "backlight_enable");
	if (ret) {
		dev_err(&client->dev, "request enable gpio failed, ret = %d\n", ret);
		goto error_enable;
	}

	do {
		ctr++;
		ret = regmap_read(pchip->regmap, 0x00, &revision);
		if(0 <= ret) {
			if (0x18 == revision) {
				pchip->ic_type = BL_IC_KTD3136;
			} else {
				pchip->ic_type = 0;
			}
			dev_info(&client->dev, "%s, revision = 0x%x, ic_type = %d\n", \
					__func__, revision, pchip->ic_type);
			break;
		}
	} while(CHIP_INIT_COUNT_MAX >= ctr);
	if(CHIP_INIT_COUNT_MAX < ctr) {
		dev_err(&client->dev, "read revision failed! count = %d\n", __func__, \
				ctr);
		goto probe_out_gpio;
	}

	props.type = BACKLIGHT_RAW;
	props.brightness = 0;
	props.max_brightness = BRIGHTNESS_MAX;
	pchip->bl_dev =
		devm_backlight_device_register(&client->dev, "lm3697_wled",
				&client->dev, pchip, &lm3697_wled_ops,
				&props);
	if (IS_ERR(pchip->bl_dev)) {
		dev_err(&client->dev, "fail : backlight register\n");
		ret = PTR_ERR(pchip->bl_dev);
		goto probe_out_gpio;
	}

	//register_device_proc("backlight", temp, "LM3697");

	dev_info(&client->dev, "%s probe done\n", __func__);

	return 0;

probe_out_gpio:
	gpio_free(pdata->en_gpio);
error_enable:
	devm_kfree(&client->dev, pchip->pdata);
	devm_kfree(&client->dev, pchip);
	dev_err(&client->dev, "%s probe failed\n", __func__);
	return ret;
}

static int lm3697_bl_remove(struct i2c_client *client){
	struct lm3697_bl_chip *pchip = i2c_get_clientdata(client);
	int ret = 0;

	dev_info(&client->dev, "%s failed\n", __func__);

	ret = regmap_write(pchip->regmap, LM3697_REG_BRT_A_LSB, 0);
	if (ret < 0)
		dev_err(&client->dev, "i2c failed to access register\n");

	ret = regmap_write(pchip->regmap, LM3697_REG_BRT_A_MSB, 0);
	if (ret < 0)
		dev_err(&client->dev, "i2c failed to access register\n");

	if (gpio_is_valid(pchip->pdata->en_gpio)){
		gpio_set_value(pchip->pdata->en_gpio, 0);
		gpio_free(pchip->pdata->en_gpio);
	}

	return 0;
}

static const struct i2c_device_id lm3697_bl_ids[] = {
	{ "lm3697", 0 },
	{ }
};


static struct i2c_driver lm3697_i2c_driver = {
	.probe = lm3697_bl_probe,
	.remove = lm3697_bl_remove,
	.driver = {
		.name = "lm3697",
		.owner = THIS_MODULE,
	},
	.id_table = lm3697_bl_ids,
};

module_i2c_driver(lm3697_i2c_driver);


MODULE_DESCRIPTION("TI LM3697 Backlight Driver");
MODULE_AUTHOR("Milo Kim");
MODULE_LICENSE("GPL");
