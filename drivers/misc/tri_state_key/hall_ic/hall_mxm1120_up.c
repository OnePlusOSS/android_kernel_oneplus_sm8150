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
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include "hall_mxm1120.h"
#include "../oplus_tri_key.h"

#define M1120_DBG_ENABLE
#define M1120_DETECTION_MODE	M1120_DETECTION_MODE_INTERRUPT
#define M1120_INTERRUPT_TYPE	M1120_VAL_INTSRS_INTTYPE_BESIDE
/*#define M1120_INTERRUPT_TYPE	M1120_VAL_INTSRS_INTTYPE_WITHIN*/
#define M1120_SENSITIVITY_TYPE	M1120_VAL_INTSRS_SRS_10BIT_0_068mT
#define M1120_PERSISTENCE_COUNT	M1120_VAL_PERSINT_COUNT(15)
#define M1120_OPERATION_FREQUENCY	M1120_VAL_OPF_FREQ_80HZ
#define M1120_OPERATION_RESOLUTION	M1120_VAL_OPF_BIT_10
#define M1120_DETECT_RANGE_HIGH	(60)/*Need change via test.*/
#define M1120_DETECT_RANGE_LOW	(50)/*Need change via test.*/
#define M1120_RESULT_STATUS_A	(0x01)/*result status A 180Degree*/
#define M1120_RESULT_STATUS_B	(0x02)/*result status B 180Degree*/
#define M1120_EVENT_TYPE	EV_ABS
#define M1120_EVENT_CODE	ABS_X
#define M1120_EVENT_DATA_CAPABILITY_MIN  (-32768)
#define M1120_EVENT_DATA_CAPABILITY_MAX  (32767)

#define M1120_VDD_MIN_UV	2700000
#define M1120_VDD_MAX_UV	3600000
#define M1120_VIO_MIN_UV	1650000
#define M1120_VIO_MAX_UV	3600000

#define HALL_MXM1120_UP "oplus,hall-mxm1120,up"

#define TRI_KEY_TAG                  "[tri_state_key] "
#define TRI_KEY_ERR(fmt, args...)\
	pr_info(TRI_KEY_TAG" %s : "fmt, __func__, ##args)
#define TRI_KEY_LOG(fmt, args...)\
	pr_info(TRI_KEY_TAG" %s : "fmt, __func__, ##args)

static struct m1120_data_t *p_m1120_data;

static DEFINE_MUTEX(hall_m1120_up_i2c_mutex);

/*i2c interface*/
static int m1120_i2c_read_block(struct m1120_data_t *m1120_data, u8 addr, u8 *data, u8 len);
static int m1120_i2c_write_block(struct m1120_data_t *m1120_data, u8 addr, u8 *data, u8 len);
static void m1120_short_to_2byte(struct m1120_data_t *m1120_data, short x, u8 *hbyte, u8 *lbyte);
static short m1120_2byte_to_short(struct m1120_data_t *m1120_data, u8 hbyte, u8 lbyte);
/*vdd vid power control*/
static int m1120_set_power(bool on);


static int  m1120_get_enable(struct m1120_data_t *p_data);
static void m1120_set_enable(struct m1120_data_t *p_data, int enable);
static void m1120_set_delay(struct m1120_data_t *p_data, int delay);
static void m1120_set_debug(struct m1120_data_t *p_data, int debug);
static int  m1120_clear_interrupt(struct m1120_data_t *p_data);
static int  m1120_set_operation_mode(struct m1120_data_t *p_data, int mode);
static int  m1120_init_device(struct m1120_data_t *p_data);
static int  m1120_reset_device(struct m1120_data_t *p_data);
static int  m1120_power_ctl(struct m1120_data_t *data, bool on);
static int m1120_get_data(short *data);

/*functions for i2c interface*/

#define M1120_I2C_BUF_SIZE		(17)


static int m1120_i2c_read_block(struct m1120_data_t *m1120_data, u8 addr, u8 *data, u8 len)
{
	u8 reg_addr = addr;
	int err = 0;
	struct i2c_client *client = NULL;
	struct i2c_msg msgs[2] = {{0}, {0}};

	if (!m1120_data) {
		TRI_KEY_ERR("m1120_data == NULL\n");
		return -EINVAL;
	}
	client = m1120_data->client;

	if (!client) {
		TRI_KEY_ERR("client null\n");
		return -EINVAL;
	} else if (len >= M1120_I2C_BUF_SIZE) {
		TRI_KEY_ERR(" length %d exceeds %d\n", len, M1120_I2C_BUF_SIZE);
		return -EINVAL;
	}
	mutex_lock(&hall_m1120_up_i2c_mutex);

	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &reg_addr;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = data;

	err = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));

	if (err < 0) {
		TRI_KEY_ERR("i2c_transfer error: (%d %p %d) %d\n", addr, data, len, err);
		err = -EIO;
	} else  {
		err = 0;
	}
	mutex_unlock(&hall_m1120_up_i2c_mutex);

	return err;
}

static int m1120_i2c_write_block(struct m1120_data_t *m1120_data, u8 addr, u8 *data, u8 len)
{
	int err = 0;
	int idx = 0;
	int num = 0;
	char buf[M1120_I2C_BUF_SIZE] = {0};
	struct i2c_client *client = NULL;

	if (!m1120_data) {
		TRI_KEY_ERR("m1120_data == NULL\n");
		return -EINVAL;
	}
	client = m1120_data->client;

	if (!client) {
		TRI_KEY_ERR("client null\n");
		return -EINVAL;
	} else if (len >= M1120_I2C_BUF_SIZE) {
		TRI_KEY_ERR(" length %d exceeds %d\n", len, M1120_I2C_BUF_SIZE);
		return -EINVAL;
	}

	mutex_lock(&hall_m1120_up_i2c_mutex);

	buf[num++] = addr;
	for (idx = 0; idx < len; idx++)
		buf[num++] = data[idx];

	err = i2c_master_send(client, buf, num);
	if (err < 0)
		TRI_KEY_ERR("send command error!! %d\n", err);

	/* store reg written */
	if (len == 1) {
		switch (addr) {
		case M1120_REG_PERSINT:
			m1120_data->reg.map.persint = data[0];
			break;
		case M1120_REG_INTSRS:
			m1120_data->reg.map.intsrs = data[0];
			break;
		case M1120_REG_LTHL:
			m1120_data->reg.map.lthl = data[0];
			break;
		case M1120_REG_LTHH:
			m1120_data->reg.map.lthh = data[0];
			break;
		case M1120_REG_HTHL:
			m1120_data->reg.map.hthl = data[0];
			break;
		case M1120_REG_HTHH:
			m1120_data->reg.map.hthh = data[0];
			break;
		case M1120_REG_I2CDIS:
			m1120_data->reg.map.i2cdis = data[0];
			break;
		case M1120_REG_SRST:
			m1120_data->reg.map.srst = data[0];
			break;
		case M1120_REG_OPF:
			m1120_data->reg.map.opf = data[0];
			break;
		}
	}

	mutex_unlock(&hall_m1120_up_i2c_mutex);
	return err;
}

static void m1120_short_to_2byte(struct m1120_data_t *m1120_data, short x, u8 *hbyte, u8 *lbyte)
{
	if (!m1120_data) {
		TRI_KEY_ERR("m1120_data == NULL\n");
		return;
	}

	if ((m1120_data->reg.map.opf & M1120_VAL_OPF_BIT_8) == M1120_VAL_OPF_BIT_8) {
		/* 8 bit resolution */
		if (x < -128)
			x = -128;
		else if (x > 127)
			x = 127;

		if (x >= 0)
			*lbyte = x & 0x7F;
		else
			*lbyte = ((0x80 - (x * (-1))) & 0x7F) | 0x80;
		*hbyte = 0x00;
	} else {
		/* 10 bit resolution */
		if (x < -512)
			x = -512;
		else if (x > 511)
			x = 511;

		if (x >= 0) {
			*lbyte = x & 0xFF;
			*hbyte = (((x & 0x100) >> 8) & 0x01) << 6;
		} else {
			*lbyte = (0x0200 - (x*(-1))) & 0xFF;
			*hbyte = ((((0x0200 - (x*(-1))) & 0x100) >> 8) << 6) | 0x80;
		}
	}
}

static short m1120_2byte_to_short(struct m1120_data_t *m1120_data, u8 hbyte, u8 lbyte)
{
	short x = 0;

	if (!m1120_data) {
		TRI_KEY_ERR("m1120_data == NULL\n");
		return -EINVAL;
	}

	if ((m1120_data->reg.map.opf & M1120_VAL_OPF_BIT_8) == M1120_VAL_OPF_BIT_8) {
		/* 8 bit resolution */
		x = lbyte & 0x7F;
		if (lbyte & 0x80)
			x -= 0x80;
	} else {
		/* 10 bit resolution */
		x = (((hbyte & 0x40) >> 6) << 8) | lbyte;
		if (hbyte & 0x80)
			x -= 0x200;
	}

	return x;
}

static int m1120_set_power(bool on)
{
	m1120_power_ctl(p_m1120_data, on);

	return 0;
}

static irqreturn_t m1120_up_irq_handler(int irq, void *dev_id)
{
	TRI_KEY_LOG("call\n");

	if (!p_m1120_data) {
		TRI_KEY_LOG("p_m1120_data NULL\n");
		return -EINVAL;
	}

	disable_irq_nosync(p_m1120_data->irq);
	__pm_wakeup_event(&p_m1120_data->source, 2000);
	oplus_hall_irq_handler(0);

	return IRQ_HANDLED;
}


static int m1120_get_enable(struct m1120_data_t *p_data)
{
	return atomic_read(&p_data->atm.enable);
}


static void m1120_set_enable(struct m1120_data_t *p_data, int enable)
{
	mutex_lock(&p_data->mtx.enable);
	TRI_KEY_LOG("enable : %d\n", enable);
	if (enable) {/*enable if state will be changed*/
		if (!atomic_cmpxchg(&p_data->atm.enable, 0, 1))
			m1120_set_operation_mode(p_data, OPERATION_MODE_MEASUREMENT_M);
	} else {/*disable if state will be changed*/
		if (atomic_cmpxchg(&p_data->atm.enable, 1, 0))
			m1120_set_operation_mode(p_data, OPERATION_MODE_POWERDOWN_M);
	}
	atomic_set(&p_data->atm.enable, enable);

	mutex_unlock(&p_data->mtx.enable);
}


static void m1120_set_delay(struct m1120_data_t *p_data, int delay)
{
	if (delay < M1120_DELAY_MIN)
		delay = M1120_DELAY_MIN;
	atomic_set(&p_data->atm.delay, delay);

	mutex_lock(&p_data->mtx.enable);

	if (m1120_get_enable(p_data)) {
		if (!(p_data->reg.map.intsrs & M1120_DETECTION_MODE_INTERRUPT)) {
			cancel_delayed_work_sync(&p_data->work);
			schedule_delayed_work(&p_data->work, msecs_to_jiffies(delay));
		}
	}

	mutex_unlock(&p_data->mtx.enable);
}


static void m1120_set_debug(struct m1120_data_t *p_data, int debug)
{
	atomic_set(&p_data->atm.debug, debug);
}

static int m1120_clear_interrupt(struct m1120_data_t *p_data)
{
	int ret = 0;
	u8 data = 0x00;

	data = p_data->reg.map.persint | 0x01;
	ret =  m1120_i2c_write_block(p_data, M1120_REG_PERSINT, &data, 1);

	return ret;
}

static int m1120_set_operation_mode(struct m1120_data_t *p_data, int mode)
{
	u8 opf = p_data->reg.map.opf;
	int err = -1;

	switch (mode) {
	case OPERATION_MODE_POWERDOWN_M:
		opf &= (0xFF - M1120_VAL_OPF_HSSON_ON);
		err = m1120_i2c_write_block(p_data, M1120_REG_OPF, &opf, 1);
		TRI_KEY_LOG("operation mode was OPERATION_MODE_POWERDOWN_M");
		break;
	case OPERATION_MODE_MEASUREMENT_M:
		opf &= (0xFF - M1120_VAL_OPF_EFRD_ON);
		opf |= M1120_VAL_OPF_HSSON_ON;
		err = m1120_i2c_write_block(p_data, M1120_REG_OPF, &opf, 1);

		TRI_KEY_LOG("operation mode was OPERATION_MODE_MEASUREMENT_M");
		break;
	case OPERATION_MODE_FUSEROMACCESS_M:
		opf |= M1120_VAL_OPF_EFRD_ON;
		opf |= M1120_VAL_OPF_HSSON_ON;
		err = m1120_i2c_write_block(p_data, M1120_REG_OPF, &opf, 1);
		TRI_KEY_LOG("operation mode was OPERATION_MODE_FUSEROMACCESS_M");
		break;
	}
	TRI_KEY_LOG("opf = ox%x\n", opf);
	return err;
}

static int m1120_init_device(struct m1120_data_t *p_data)
{
	int err = -1;

	/*(1) vdd and vid power up*/
	err = m1120_set_power(1);
	if (err) {
		TRI_KEY_LOG("m1120 power-on was failed (%d)", err);
		return err;
	}

	/*(2) init variables*/
	atomic_set(&p_data->atm.enable, 0);
	atomic_set(&p_data->atm.delay, M1120_DELAY_MIN);
#ifdef M1120_DBG_ENABLE
	atomic_set(&p_data->atm.debug, 1);
#else
	atomic_set(&p_data->atm.debug, 0);
#endif
	p_data->calibrated_data = 0;
	p_data->last_data = 0;
	p_data->irq_enabled = 0;
	p_data->irq_first = 1;
	p_data->thrhigh = M1120_DETECT_RANGE_HIGH;
	p_data->thrlow = M1120_DETECT_RANGE_LOW;
	m1120_set_delay(p_data, M1120_DELAY_MAX);
	m1120_set_debug(p_data, 0);

	/*(3) reset registers*/
	err = m1120_reset_device(p_data);
	if (err < 0) {
		TRI_KEY_ERR("m1120_reset_device was failed (%d)", err);
		return err;
	}

	TRI_KEY_LOG("initializing device was success");

	return 0;
}

static int m1120_reset_device(struct m1120_data_t *p_data)
{
	int err = 0;
	u8  id = 0xFF, data = 0x00;

	if ((p_data == NULL) || (p_data->client == NULL))
		return -ENODEV;

	/*(1) sw reset*/
	data = M1120_VAL_SRST_RESET;
	err = m1120_i2c_write_block(p_data, M1120_REG_SRST, &data, 1);
	if (err < 0) {
		TRI_KEY_ERR("sw-reset was failed(%d)", err);
		return err;
	}
	msleep(20);

	/*(2) check id*/
	err = m1120_i2c_read_block(p_data, M1120_REG_DID, &id, 1);
	if (err < 0)
		return err;
	if (id != M1120_VAL_DID) {
		TRI_KEY_ERR("current device id(0x%02X) is not M1120 device id(0x%02X)",
			id, M1120_VAL_DID);
		return -ENXIO;
	}

	/*(3) init variables*/
	/*(3-1) persint*/
	data = M1120_PERSISTENCE_COUNT;
	err =   m1120_i2c_write_block(p_data, M1120_REG_PERSINT, &data, 1);
	if (err < 0) {
		TRI_KEY_ERR("cm1120_i2c_write_block error, data : %d", data);
		return err;
	}
	/*(3-2) intsrs*/
	data = M1120_DETECTION_MODE | M1120_SENSITIVITY_TYPE;
	if (data & M1120_DETECTION_MODE_INTERRUPT)
		data |= M1120_INTERRUPT_TYPE;

	err = m1120_i2c_write_block(p_data, M1120_REG_INTSRS, &data, 1);
	if (err < 0) {
		TRI_KEY_LOG("cm1120_i2c_write_block error, data : %d", data);
		return err;
	}
	/*(3-3) opf*/
	data = M1120_OPERATION_FREQUENCY | M1120_OPERATION_RESOLUTION;
	err = m1120_i2c_write_block(p_data, M1120_REG_OPF, &data, 1);
	if (err < 0) {
		TRI_KEY_LOG("cm1120_i2c_write_block error, data : %d", data);
		return err;
	}

	/*(5) set power-on mode*/
	err = m1120_set_operation_mode(p_data, OPERATION_MODE_MEASUREMENT_M);
	if (err < 0) {
		TRI_KEY_ERR("m1120_set_detection_mode was failed(%d)", err);
		return err;
	}

	return err;
}

static int m1120_input_dev_init(struct m1120_data_t *p_data)
{
	struct input_dev *dev;
	int err;

	dev = input_allocate_device();
	if (!dev)
		return -ENOMEM;

	dev->name = M1120_DRIVER_NAME_UP;
	dev->id.bustype = BUS_I2C;

#if (M1120_EVENT_TYPE == EV_ABS)
	input_set_drvdata(dev, p_data);
	input_set_capability(dev, M1120_EVENT_TYPE, ABS_MISC);
	input_set_abs_params(dev, M1120_EVENT_CODE, M1120_EVENT_DATA_CAPABILITY_MIN,
		M1120_EVENT_DATA_CAPABILITY_MAX, 0, 0);
#elif (M1120_EVENT_TYPE == EV_KEY)
	input_set_drvdata(dev, p_data);
	input_set_capability(dev, M1120_EVENT_TYPE, M1120_EVENT_CODE);
#else
#error	"[ERR] M1120_EVENT_TYPE is not defined."
#endif

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		return err;
	}

	p_data->input_dev = dev;

	return 0;
}

static void m1120_input_dev_terminate(struct m1120_data_t *p_data)
{
	struct input_dev *dev = p_data->input_dev;

	input_unregister_device(dev);
	input_free_device(dev);
}

static int m1120_power_ctl(struct m1120_data_t *data, bool on)
{
	int ret = 0;
	int err = 0;

	if (!on && data->power_enabled) {
		ret = regulator_disable(data->vdd);
		if (ret) {
			TRI_KEY_ERR("Regulator vdd disable failed ret=%d\n", ret);
			return ret;
		}

		ret = regulator_disable(data->vio);
		if (ret) {
			TRI_KEY_ERR("Regulator vio disable failed ret=%d\n", ret);
			err = regulator_enable(data->vdd);
			return ret;
		}
		data->power_enabled = on;
	} else if (on && !data->power_enabled) {
		ret = regulator_enable(data->vdd);
		if (ret) {
			TRI_KEY_ERR("Regulator vdd enable failed ret=%d\n", ret);
			return ret;
		}
		msleep(20);
		ret = regulator_enable(data->vio);
		if (ret) {
			TRI_KEY_ERR("Regulator vio enable failed ret=%d\n", ret);
			err = regulator_disable(data->vdd);
			return ret;
		}
		msleep(20);
		data->power_enabled = on;
	} else {
		dev_info(&data->client->dev,
			"Power on=%d. enabled=%d\n",
			on, data->power_enabled);
}

return ret;
}

static int m1120_power_init(struct m1120_data_t *data)
{
	int ret;

	data->vdd = regulator_get(&data->client->dev, "vdd");
	if (IS_ERR(data->vdd)) {
		ret = PTR_ERR(data->vdd);
		TRI_KEY_ERR("Regulator get failed vdd ret=%d\n", ret);
		return ret;
	}

	if (regulator_count_voltages(data->vdd) > 0) {
		ret = regulator_set_voltage(data->vdd,
				M1120_VDD_MIN_UV,
				M1120_VDD_MAX_UV);
		if (ret) {
			TRI_KEY_ERR("Regulator set failed vdd ret=%d\n", ret);
			goto reg_vdd_put;
		}
	}

	data->vio = regulator_get(&data->client->dev, "vio");
	if (IS_ERR(data->vio)) {
		ret = PTR_ERR(data->vio);
		TRI_KEY_ERR("Regulator get failed vio ret=%d\n", ret);
		goto reg_vdd_set;
	}

	if (regulator_count_voltages(data->vio) > 0) {
		ret = regulator_set_voltage(data->vio,
				M1120_VIO_MIN_UV,
				M1120_VIO_MAX_UV);
		if (ret) {
			TRI_KEY_ERR("Regulator set failed vio ret=%d\n", ret);
			goto reg_vio_put;
		}
	}

	return 0;

reg_vio_put:
	regulator_put(data->vio);
reg_vdd_set:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, M1120_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return ret;
}


static int tri_key_m1120_parse_dt(struct device *dev,
		struct m1120_data_t *p_data)
{
	struct device_node *np = dev->of_node;
	struct pinctrl *key_pinctrl;
	struct pinctrl_state *set_state;
	u32 temp_val;
	int rc;

	TRI_KEY_LOG("%s", __func__);
	rc = of_property_read_u32(np, "magnachip,init-interval", &temp_val);
	if (rc && (rc != -EINVAL))
		TRI_KEY_ERR("Unable to read init-interval\n");
	else {
		if (temp_val < M1120_DELAY_MIN)
			temp_val = M1120_DELAY_MIN;
		atomic_set(&p_data->atm.delay, temp_val);
	}

	p_data->intprt_en = of_property_read_bool(np, "magnachip,use-interrupt");

	p_data->igpio = of_get_named_gpio_flags(dev->of_node,
			"magnachip,gpio-int", 0, NULL);

	p_data->irq_gpio =  of_get_named_gpio(np, "dhall,irq-gpio", 0);
	TRI_KEY_LOG("irq_gpio : %d", p_data->irq_gpio);

	p_data->use_hrtimer = of_property_read_bool(np, "magnachip,use-hrtimer");

	key_pinctrl = devm_pinctrl_get(dev);

	if (IS_ERR_OR_NULL(key_pinctrl))
		TRI_KEY_ERR("Failed to get pinctrl\n");

	set_state = pinctrl_lookup_state(key_pinctrl,
			"uphall_tri_state_key_active");
	if (IS_ERR_OR_NULL(set_state))
		TRI_KEY_ERR("Failed to lookup_state\n");

	pinctrl_select_state(key_pinctrl, set_state);


	return 0;
}

static int m1120_get_data(short *data)
{
	int err = 0;
	u8 buf[3] = {0};
	short value = 0;

	if (!p_m1120_data) {
		TRI_KEY_ERR("p_m1120_data == NULL");
		return -EINVAL;
	}

	err = m1120_i2c_read_block(p_m1120_data, M1120_REG_ST1, buf, sizeof(buf));
	if (err < 0) {
		TRI_KEY_LOG("fail %d\n", err);
		return err;
	}

	if (buf[0] & 0x01)
		value = m1120_2byte_to_short(p_m1120_data, buf[2], buf[1]);
	else
		return err;
	*data = value;

	return 0;
}

static int m1120_enable_irq(bool enable)
{
	TRI_KEY_LOG("%s", __func__);

	if (p_m1120_data == NULL) {
		TRI_KEY_ERR("p_m1120_data == NULL");
		return -EINVAL;
	}

	if (enable)
		enable_irq(p_m1120_data->irq);
	else
		disable_irq_nosync(p_m1120_data->irq);

	return 0;
}

static int m1120_clear_irq(void)
{
	TRI_KEY_LOG("%s", __func__);
	if (p_m1120_data == NULL) {
		TRI_KEY_ERR("p_m1120_data == NULL");
		return -EINVAL;
	}

	m1120_clear_interrupt(p_m1120_data);
	return 0;
}

static int m1120_get_irq_state(void)
{
	TRI_KEY_LOG("%s", __func__);
	if (p_m1120_data == NULL) {
		TRI_KEY_ERR("p_m1120_data == NULL");
		return -EINVAL;
	}

	return ((p_m1120_data->reg.map.intsrs & M1120_DETECTION_MODE_INTERRUPT) ? 1 : 0);
}

static bool m1120_update_threshold(int position, short lowthd, short highthd)
{
	u8 lthh, lthl, hthh, hthl;
	int err = 0;

	TRI_KEY_LOG("%s", __func__);
	if (p_m1120_data == NULL) {
		TRI_KEY_LOG("p_m1120_data == NULL\n");
		return -EINVAL;
	}

	TRI_KEY_LOG("m1120_up ,low:%d, high:%d\n", lowthd, highthd);

	err = m1120_clear_interrupt(p_m1120_data);

	if (p_m1120_data->reg.map.intsrs & M1120_DETECTION_MODE_INTERRUPT) {
		TRI_KEY_LOG("up hallthreshold, lowthd=%d, highthd=%d.\n", lowthd, highthd);
		m1120_short_to_2byte(p_m1120_data, highthd, &hthh, &hthl);
		m1120_short_to_2byte(p_m1120_data, lowthd, &lthh, &lthl);

		err |= m1120_i2c_write_block(p_m1120_data, M1120_REG_HTHH, &hthh, 1);
		err |= m1120_i2c_write_block(p_m1120_data, M1120_REG_HTHL, &hthl, 1);
		err |= m1120_i2c_write_block(p_m1120_data, M1120_REG_LTHH, &lthh, 1);
		err |= m1120_i2c_write_block(p_m1120_data, M1120_REG_LTHL, &lthl, 1);
	}

	if (err < 0) {
		TRI_KEY_ERR("tri_key:fail %d\n", err);
		return false;
	} else {
		return true;
	}

	return true;
}

static void m1120_dump_reg(u8 *buf)
{
	int i, err;
	u8 val;
	u8 buffer[512] = {0};
	u8 _buf[20] = {0};

	TRI_KEY_LOG("%s", __func__);
	if (p_m1120_data == NULL) {
		TRI_KEY_LOG("p_m1120_data == NULL\n");
		return;
	}

	for (i = 0; i <= 0x12; i++) {
		memset(_buf, 0, sizeof(_buf));

		err = m1120_i2c_read_block(p_m1120_data, i, &val, 1);
		if (err < 0) {
			snprintf(buf, PAGE_SIZE, "read reg error!\n");
			return;
		}

		snprintf(_buf, sizeof(_buf), "reg 0x%x:0x%x\n", i, val);
		strcat(buffer, _buf);
	}
	snprintf(buf, PAGE_SIZE, "%s\n", buffer);
	TRI_KEY_LOG("%s\n", buf);
}

static bool m1120_is_power_on(void)
{
	TRI_KEY_LOG("%s", __func__);
	if (p_m1120_data == NULL) {
		TRI_KEY_LOG("p_m1120_data == NULL\n");
		return false;
	}

	return p_m1120_data->power_enabled > 0 ? true : false;
}

static int m1120_set_detection_mode_1(u8 mode)
{
	u8 data = 0;
	int err = 0;

	TRI_KEY_LOG("======> %s", __func__);
	if (p_m1120_data == NULL) {
		TRI_KEY_ERR("p_m1120_data == NULL");
		return -EINVAL;
	}

	TRI_KEY_LOG("m1120 up detection mode : %s\n", (mode == 0) ? "POLLING":"INTERRUPT");

	if (mode & DETECTION_MODE_INTERRUPT) { /*interrupt mode*/
		if (!p_m1120_data->irq_enabled) {
			data = p_m1120_data->reg.map.intsrs | M1120_DETECTION_MODE_INTERRUPT;

			err = m1120_i2c_write_block(p_m1120_data, M1120_REG_INTSRS, &data, 1);
			if (err < 0) {
				TRI_KEY_ERR("config interrupt fail %d\n", err);
				return err;
			}

			err = m1120_clear_interrupt(p_m1120_data);
			if (err < 0) {
				TRI_KEY_ERR("clear interrupt fail %d\n", err);
				return err;
			}

			/* request irq */
			TRI_KEY_LOG("m1120 down enter irq handler\n");
			if (request_irq(p_m1120_data->irq, &m1120_up_irq_handler,
				IRQ_TYPE_LEVEL_LOW, "hall_m1120_up",
					(void *)p_m1120_data->client)) {
				TRI_KEY_ERR("IRQ LINE NOT AVAILABLE!!\n");
				return -EINVAL;
			}
			irq_set_irq_wake(p_m1120_data->irq, 1);

			p_m1120_data->irq_enabled = 1;
		}
	} else { /*polling mode*/
		if (p_m1120_data->irq_enabled) {
			data = p_m1120_data->reg.map.intsrs &
				(0xFF - M1120_DETECTION_MODE_INTERRUPT);

			err = m1120_i2c_write_block(p_m1120_data, M1120_REG_INTSRS, &data, 1);
			if (err < 0) {
				TRI_KEY_ERR("config interrupt fail %d\n", err);
				return err;
			}

			disable_irq(p_m1120_data->irq);
			free_irq(p_m1120_data->irq, NULL);

			p_m1120_data->irq_enabled = 0;
		}
	}

	return 0;
}

static int m1120_set_reg_1(int reg, int val)
{
	u8 data = (u8)val;

	TRI_KEY_LOG("======> %s", __func__);
	if (p_m1120_data == NULL) {
		TRI_KEY_ERR("p_m1120_data == NULL");
		return -EINVAL;
	}

	m1120_i2c_write_block(p_m1120_data, (u8)reg, &data, 1);
	return 0;
}

struct dhall_operations  m1120_ups_ops = {
	.get_data  = m1120_get_data,
	.enable_irq = m1120_enable_irq,
	.clear_irq = m1120_clear_irq,
	.get_irq_state = m1120_get_irq_state,
	.set_detection_mode = m1120_set_detection_mode_1,
	.update_threshold = m1120_update_threshold,
	.dump_regs = m1120_dump_reg,
	.set_reg = m1120_set_reg_1,
	.is_power_on = m1120_is_power_on
};

static int tri_key_m1120_i2c_drv_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct m1120_data_t            *p_data;
	struct extcon_dev_data	*hall_dev = NULL;
	int                  err = 0;

	TRI_KEY_LOG("allocation memory for p_m1120_data up %s\n", __func__);
	/*(1) allocation memory for p_m1120_data*/
	p_data = kzalloc(sizeof(struct m1120_data_t), GFP_KERNEL);
	if (!p_data) {
		TRI_KEY_ERR("kernel memory alocation was failed");
		err = -ENOMEM;
		goto error_0;
	}
	hall_dev = kzalloc(sizeof(struct extcon_dev_data), GFP_KERNEL);
	if (!hall_dev) {
		TRI_KEY_ERR("kernel memory alocation was failed\n");
		kfree(p_data);
		return -ENOMEM;
	}
	/*(2) init mutex variable*/
	mutex_init(&p_data->mtx.enable);
	mutex_init(&p_data->mtx.data);
	p_data->power_enabled = false;
	TRI_KEY_LOG("init mutex variable\n");
	/*(3) config i2c client*/
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		TRI_KEY_ERR("i2c_check_functionality was failed");
		err = -ENODEV;
		goto error_1;
	}
	hall_dev->client = client;
	i2c_set_clientdata(client, hall_dev);
	hall_dev->dev = &client->dev;
	p_data->client = client;
	p_data->dev = &client->dev;
	p_m1120_data = p_data;

	if (client->dev.of_node) {
		TRI_KEY_LOG("Use client->dev.of_node\n");
		err = tri_key_m1120_parse_dt(&client->dev, p_data);
		if (err) {
			TRI_KEY_ERR("Failed to parse device tree\n");
			err = -EINVAL;
			goto error_1;
		}
	}

	if (gpio_is_valid(p_data->irq_gpio)) {
		err = gpio_request(p_data->irq_gpio, "m1120_up_irq");
		if (err) {
			TRI_KEY_LOG("unable to request gpio [%d]", p_data->irq_gpio);
		} else {
			err = gpio_direction_input(p_data->irq_gpio);
			msleep(50);
			p_data->irq = gpio_to_irq(p_data->irq_gpio);
			TRI_KEY_LOG("======> irq : %d", p_data->irq);
		}
	}

	err = m1120_power_init(p_data);
	if (err) {
		TRI_KEY_ERR("Failed to get sensor regulators\n");
		err = -EINVAL;
		goto error_1;
	}
	err = m1120_power_ctl(p_data, true);
	if (err) {
		TRI_KEY_ERR("Failed to enable sensor power\n");
		err = -EINVAL;
		goto error_1;
	}


	/*(6) reset and init device*/
	err = m1120_init_device(p_data);
	if (err) {
		TRI_KEY_ERR("m1120_init_device was failed(%d)", err);
		goto error_1;
	}
	TRI_KEY_LOG("%s was found", id->name);

	/*(8) init input device*/
	err = m1120_input_dev_init(p_data);
	if (err) {
		TRI_KEY_ERR("m1120_input_dev_init was failed(%d)", err);
		goto error_1;
	}
	TRI_KEY_LOG("%s was initialized", M1120_DRIVER_NAME_UP);

	wakeup_source_add(&p_m1120_data->source);

	/*(12) imigrate p_data to p_m1120_data*/
	TRI_KEY_LOG("%s : %s was probed.\n", __func__, M1120_DRIVER_NAME_UP);

	oplus_register_hall("hall_up", &m1120_ups_ops, hall_dev);

	return 0;

error_1:
	if (gpio_is_valid(p_data->irq_gpio))
		gpio_free(p_data->irq_gpio);

	kfree(p_data);
	kfree(hall_dev);

error_0:
	p_m1120_data = NULL;
	return err;
}

static int m1120_i2c_drv_remove(struct i2c_client *client)
{
	struct m1120_data_t *p_data = i2c_get_clientdata(client);

	m1120_set_enable(p_data, 0);
	m1120_input_dev_terminate(p_data);
	if (p_data->igpio != -1)
		gpio_free(p_data->igpio);

	kfree(p_data);

	return 0;
}

static const struct i2c_device_id m1120_i2c_drv_id_table[] = {
	{"hall_m1120_up", 0 },
	{ }
};


static const struct of_device_id m1120_of_match[] = {
	{ .compatible = HALL_MXM1120_UP, },
	{ },
};

struct i2c_driver m1120_i2c_up_driver = {
	.driver = {
		.owner  = THIS_MODULE,
		.name   = M1120_DRIVER_NAME_UP,
		.of_match_table = m1120_of_match,
	},
	.probe    = tri_key_m1120_i2c_drv_probe,
	.remove  = m1120_i2c_drv_remove,
	.id_table   = m1120_i2c_drv_id_table,
};


static int __init tri_key_m1120_driver_init_up(void)
{
	int res = 0;

	TRI_KEY_LOG("log %s\n", __func__);
	res = i2c_add_driver(&m1120_i2c_up_driver);
	TRI_KEY_LOG("log %s, res : %d\n", __func__, res);
	return res;
}
late_initcall(tri_key_m1120_driver_init_up);

static void __exit m1120_driver_exit_up(void)
{
	TRI_KEY_LOG("%s\n", __func__);
	i2c_del_driver(&m1120_i2c_up_driver);
}
module_exit(m1120_driver_exit_up);

MODULE_DESCRIPTION("M1120 hallswitch driver");
MODULE_LICENSE("GPL");

