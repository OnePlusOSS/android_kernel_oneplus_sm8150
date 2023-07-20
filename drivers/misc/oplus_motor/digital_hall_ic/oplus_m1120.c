/************************************************************************************
** Copyright (C), 2008-2017, OPLUS Mobile Comm Corp., Ltd
** VENDOR_EDIT
** File: oplus_m1120.c
**
** Description:
**	Definitions for m1120 digital hall chip.
**
** Version: 1.0
** Date created: 2018/01/14,20:27
**
** --------------------------- Revision History: -------------------------------------
* <version>		<date>		<author>		<desc>
**************************************************************************************/

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
#include <linux/regulator/consumer.h>

#include "oplus_m1120.h"
#include "../oplus_motor.h"

#define DHALL_NUM 2

static char *dhall_irq_name[DHALL_NUM] = {
	"m1120-irq0",
	"m1120-irq1"
};

static struct oplus_dhall_chip *g_chip[DHALL_NUM] = {NULL, NULL};

static struct hall_srs m1120_ranges[] = {
	{"40mT", M1120_VAL_INTSRS_SRS_0_08mT, false},
	{"20mT", M1120_VAL_INTSRS_SRS_0_04mT, false},
	{"10mT", M1120_VAL_INTSRS_SRS_0_02mT, false},
	{"5mT", M1120_VAL_INTSRS_SRS_0_01mT, false},
	{"2_05mT", M1120_VAL_INTSRS_SRS_0_005mT, false},
};

static DEFINE_MUTEX(m1120_i2c_mutex);
__attribute__((weak)) void oplus_m1120_reconfig(struct oplus_dhall_chip *chip)
{
	return;
}

static int m1120_i2c_read_block(struct oplus_dhall_chip *chip, u8 addr, u8 *data,
				u8 len)
{
	u8 reg_addr = addr;
	int err = 0;
	struct i2c_client *client = chip->client;
	struct i2c_msg msgs[2] = {{0}, {0}};

	if (!client) {
		MOTOR_ERR("client null\n");
		return -EINVAL;

	} else if (len >= M1120_I2C_BUF_SIZE) {
		MOTOR_ERR(" length %d exceeds %d\n", len, M1120_I2C_BUF_SIZE);
		return -EINVAL;
	}

	mutex_lock(&m1120_i2c_mutex);

	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &reg_addr;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = data;

	err = i2c_transfer(client->adapter, msgs, (sizeof(msgs) / sizeof(msgs[0])));

	if (err < 0) {
		MOTOR_ERR("i2c_transfer error: (%d %p %d) %d\n", addr, data, len, err);
		err = -EIO;
	} else {
		err = 0;
	}

	mutex_unlock(&m1120_i2c_mutex);

	return err;
}

static int m1120_i2c_write_block(struct oplus_dhall_chip *chip, u8 addr,
				 u8 *data, u8 len)
{
	int err = 0;
	int idx = 0;
	int num = 0;
	char buf[M1120_I2C_BUF_SIZE] = {0};
	struct i2c_client *client = chip->client;

	if (!client) {
		MOTOR_ERR("client null\n");
		return -EINVAL;

	} else if (len >= M1120_I2C_BUF_SIZE) {
		MOTOR_ERR(" length %d exceeds %d\n", len, M1120_I2C_BUF_SIZE);
		return -EINVAL;
	}

	mutex_lock(&m1120_i2c_mutex);

	buf[num++] = addr;

	for (idx = 0; idx < len; idx++) {
		buf[num++] = data[idx];
	}

	err = i2c_master_send(client, buf, num);

	if (err < 0) {
		MOTOR_ERR("send command error!! %d\n", err);
	}

	/*store reg written*/
	if (len == 1) {
		switch (addr) {
		case M1120_REG_PERSINT:
			chip->reg.map.persint = data[0];
			break;

		case M1120_REG_INTSRS:
			chip->reg.map.intsrs = data[0];
			break;

		case M1120_REG_LTHL:
			chip->reg.map.lthl = data[0];
			break;

		case M1120_REG_LTHH:
			chip->reg.map.lthh = data[0];
			break;

		case M1120_REG_HTHL:
			chip->reg.map.hthl = data[0];
			break;

		case M1120_REG_HTHH:
			chip->reg.map.hthh = data[0];
			break;

		case M1120_REG_I2CDIS:
			chip->reg.map.i2cdis = data[0];
			break;

		case M1120_REG_SRST:
			chip->reg.map.srst = data[0];
			break;

		case M1120_REG_OPF:
			chip->reg.map.opf = data[0];
			break;
		}
	}

	mutex_unlock(&m1120_i2c_mutex);
	return err;
}

static void m1120_short_to_2byte(struct oplus_dhall_chip *chip, short x,
				 u8 *hbyte, u8 *lbyte)
{
	if ((chip->reg.map.opf & M1120_VAL_OPF_BIT_8) == M1120_VAL_OPF_BIT_8) {
		/* 8 bit resolution */
		if (x < -128) {
			x = -128;

		} else if (x > 127) {
			x = 127;
		}

		if (x >= 0) {
			*lbyte = x & 0x7F;

		} else {
			*lbyte = ((0x80 - (x * (-1))) & 0x7F) | 0x80;
		}

		*hbyte = 0x00;

	} else {
		/* 10 bit resolution */
		if (x < -512) {
			x = -512;

		} else if (x > 511) {
			x = 511;
		}

		if (x >= 0) {
			*lbyte = x & 0xFF;
			*hbyte = (((x & 0x100) >> 8) & 0x01) << 6;

		} else {
			*lbyte = (0x0200 - (x * (-1))) & 0xFF;
			*hbyte = ((((0x0200 - (x * (-1))) & 0x100) >> 8) << 6) | 0x80;
		}
	}
}

static short m1120_2byte_to_short(struct oplus_dhall_chip *chip, u8 hbyte,
				  u8 lbyte)
{
	short x = 0;

	if ((chip->reg.map.opf & M1120_VAL_OPF_BIT_8) == M1120_VAL_OPF_BIT_8) {
		/* 8 bit resolution */
		x = lbyte & 0x7F;

		if (lbyte & 0x80) {
			x -= 0x80;
		}

	} else {
		/* 10 bit resolution */
		x = (((hbyte & 0x40) >> 6) << 8) | lbyte;

		if (hbyte & 0x80) {
			x -= 0x200;
		}
	}

	return x;
}

static int m1120_get_id(struct oplus_dhall_chip *chip)
{
	u8 data = 0;
	m1120_i2c_read_block(chip, M1120_REG_DID, &data, 1);

	MOTOR_LOG("id = 0x%x \n", data);

	return data;
}

static int m1120_get_data(unsigned int id, short *data)
{
	int err = 0;
	u8 buf[3] = {0};
	short value = 0;
	static short pre_value[DHALL_NUM] = {0};

	if ((id >= DHALL_NUM) || (g_chip[id] == NULL)) {
		MOTOR_LOG("g_chip NULL \n");
		return -EINVAL;
	}

	/* (1) read data*/
	err = m1120_i2c_read_block(g_chip[id], M1120_REG_ST1, buf, sizeof(buf));

	if (err < 0) {
		MOTOR_LOG(" fail %d \n", err);
		return err;
	}

	/* (2) collect data*/
	if (buf[0] & 0x01) {
		value = m1120_2byte_to_short(g_chip[id], buf[2], buf[1]);

	} else {
		MOTOR_LOG("m1120: st1(0x%02X) is not DRDY.\n", buf[0]);
		*data = pre_value[id];
		return err;
	}

	*data = value;
	pre_value[id] = value;

	return 0;
}

static void m1120_dump_reg(unsigned int id, u8 *buf)
{
	int i, err;
	u8 val;
	u8 buffer[512] = {0};
	u8 _buf[20] = {0};

	if ((id >= DHALL_NUM) || (g_chip[id] == NULL)) {
		MOTOR_LOG("g_chip NULL \n");
		return;
	}

	for (i = 0; i <= 0x12; i++) {
		memset(_buf, 0, sizeof(_buf));

		err = m1120_i2c_read_block(g_chip[id], i, &val, 1);

		if (err < 0) {
			sprintf(buf, "read reg error!\n");
			return;
		}

		sprintf(_buf,  "reg 0x%x:0x%x\n", i, val);
		strcat(buffer, _buf);
	}

	sprintf(buf, "%s\n", buffer);
	MOTOR_LOG("%s \n", buf);
}

static int m1120_set_reg(unsigned int id, int reg, int val)
{
	u8 data = (u8)val;

	if ((id >= DHALL_NUM) || (g_chip[id] == NULL)) {
		MOTOR_LOG("g_chip NULL \n");
		return -EINVAL;
	}

	m1120_i2c_write_block(g_chip[id], (u8)reg, &data, 1);

	return 0;
}


static bool m1120_is_power_on(unsigned int id)
{
	if ((id >= DHALL_NUM) || (g_chip[id] == NULL)) {
		MOTOR_LOG("g_chip NULL \n");
		return false;
	}

	return g_chip[id]->is_power_on;
}

static int m1120_set_power_gpio_down(struct oplus_dhall_chip *chip)
{
	int ret = 0;

	if (IS_ERR_OR_NULL(chip->pctrl)) {
		ret = PTR_ERR(chip->pctrl);
		MOTOR_ERR("failed to get pinctrl\n");
		return ret;
	}

	chip->power_state = pinctrl_lookup_state(chip->pctrl, "hall_power_down");

	if (IS_ERR_OR_NULL(chip->power_state)) {
		ret = PTR_ERR(chip->power_state);
		MOTOR_ERR("pinctrl_lookup_state, err:%d\n", ret);
		return ret;
	}

	pinctrl_select_state(chip->pctrl, chip->power_state);

	return 0;
}

static int m1120_set_power_gpio_up(struct oplus_dhall_chip *chip)
{
	int ret = 0;

	if (IS_ERR_OR_NULL(chip->pctrl)) {
		ret = PTR_ERR(chip->pctrl);
		MOTOR_ERR("failed to get pinctrl\n");
		return ret;
	}

	chip->power_state = pinctrl_lookup_state(chip->pctrl, "hall_power_up");

	if (IS_ERR_OR_NULL(chip->power_state)) {
		ret = PTR_ERR(chip->power_state);
		MOTOR_ERR("pinctrl_lookup_state, err:%d\n", ret);
		return ret;
	}

	pinctrl_select_state(chip->pctrl, chip->power_state);

	return 0;
}


/* vdd / vid power control */
static int m1120_set_power(struct oplus_dhall_chip *chip, bool on)
{
	int ret = 0;

	if (IS_ERR_OR_NULL(chip->power_2v8)) {
		MOTOR_ERR("vdd_2v8 invalid\n");
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(chip->power_1v8)) {
		MOTOR_ERR("vdd1v8 invalid\n");
		return -EINVAL;
	}

	if (on) {
		m1120_set_power_gpio_up(chip);

		if (regulator_count_voltages(chip->power_2v8) > 0) {
			ret = regulator_set_voltage(chip->power_2v8, 2856000, 3104000);

			if (ret) {
				MOTOR_LOG("Regulator set_vtg failed vdd ret=%d\n", ret);
				return ret;
			}

			ret = regulator_set_load(chip->power_2v8, CURRENT_LOAD_UA);

			if (ret) {
				MOTOR_LOG("Regulator set_vtg failed vdd ret=%d\n", ret);
				return ret;
			}
		}

		if (regulator_count_voltages(chip->power_1v8) > 0) {
			ret = regulator_set_voltage(chip->power_1v8, 1800000, 1800000);

			if (ret) {
				MOTOR_LOG("Regulator set_vtg failed vcc_i2c ret=%d\n", ret);
				return ret;
			}
		}

		/*enable the 2v8 power*/
		ret = regulator_enable(chip->power_2v8);

		if (ret) {
			MOTOR_LOG("Regulator vdd enable failed ret=%d\n", ret);
			return ret;
		}

		/*should enable the 1v8 power*/
		msleep(5);
		m1120_set_power_gpio_down(chip);

		ret = regulator_enable(chip->power_1v8);

		if (ret) {
			MOTOR_LOG("Regulator vcc_i2c enable failed ret=%d\n", ret);
			regulator_disable(chip->power_2v8);
			return ret;
		}

		chip->is_power_on = true;
	} else {
		ret = regulator_disable(chip->power_1v8);

		if (ret) {
			MOTOR_LOG("Regulator vcc_i2c disable failed ret=%d\n", ret);
			ret = regulator_enable(chip->power_2v8);
			return ret;
		}

		msleep(1);
		ret = regulator_disable(chip->power_2v8);

		if (ret) {
			MOTOR_LOG("Regulator vdd disable failed ret=%d\n", ret);
			return ret;
		}
	}

	return 0;
}

static int m1120_set_frequency(struct oplus_dhall_chip *chip, int frequency)
{
	int err = 0;
	u8 rdata = 0;

	m1120_i2c_read_block(chip, M1120_REG_OPF, &rdata, 1);

	rdata &= 0x0F;
	rdata |= frequency;

	MOTOR_LOG("M1120_REG_OPF register : 0x%x\n", rdata);

	err = m1120_i2c_write_block(chip, M1120_REG_OPF, &rdata, 1);

	if (err < 0) {
		MOTOR_LOG("sw-reset was failed(%d)", err);
		return err;
	}

	return 0;
}

static int m1120_clear_interrupt(struct oplus_dhall_chip *chip)
{
	int ret = 0;
	u8 data = chip->reg.map.persint | 0x01;

	ret = m1120_i2c_write_block(chip, M1120_REG_PERSINT, &data, 1);

	return ret;
}

static bool m1120_update_threshold(unsigned int id, int position, short lowthd,
				   short highthd)
{
	u8 lthh, lthl, hthh, hthl;
	int err = 0;

	if ((id >= DHALL_NUM) || (g_chip[id] == NULL)) {
		MOTOR_LOG("g_chip NULL \n");
		return -EINVAL;
	}

	MOTOR_LOG("id %d ,low:%d, high:%d  \n", id , lowthd, highthd);

	err = m1120_clear_interrupt(g_chip[id]);

	if (g_chip[id]->reg.map.intsrs & M1120_VAL_INTSRS_INTTYPE_WITHIN) {
		m1120_short_to_2byte(g_chip[id], highthd, &hthh, &hthl);
		m1120_short_to_2byte(g_chip[id], lowthd, &lthh, &lthl);

		err |= m1120_i2c_write_block(g_chip[id], M1120_REG_HTHH, &hthh, 1);
		err |= m1120_i2c_write_block(g_chip[id], M1120_REG_HTHL, &hthl, 1);
		err |= m1120_i2c_write_block(g_chip[id], M1120_REG_LTHH, &lthh, 1);
		err |= m1120_i2c_write_block(g_chip[id], M1120_REG_LTHL, &lthl, 1);
	}

	if (err < 0) {
		MOTOR_ERR("fail %d\n", err);
		return false;

	} else {
		return true;
	}
}

static int m1120_set_operation_mode(struct oplus_dhall_chip *chip, int mode)
{
	u8 opf = M1120_OPERATION_RESOLUTION;
	int ret = 0;

	switch (mode) {
	case OPERATION_MODE_POWERDOWN:
		opf &= (0xFF - M1120_VAL_OPF_HSSON_ON);
		ret = m1120_i2c_write_block(chip, M1120_REG_OPF, &opf, 1);
		MOTOR_ERR("operation mode :OPERATION_MODE_POWERDOWN \n");
		break;

	case OPERATION_MODE_MEASUREMENT:
		opf &= (0xFF - M1120_VAL_OPF_EFRD_ON);
		opf |= M1120_VAL_OPF_HSSON_ON;
		ret = m1120_i2c_write_block(chip, M1120_REG_OPF, &opf, 1);
		MOTOR_ERR("operation mode :OPERATION_MODE_MEASUREMENT \n");
		break;

	case OPERATION_MODE_FUSEROMACCESS:
		opf |= M1120_VAL_OPF_EFRD_ON;
		opf |= M1120_VAL_OPF_HSSON_ON;
		ret = m1120_i2c_write_block(chip, M1120_REG_OPF, &opf, 1);
		MOTOR_ERR("operation mode :OPERATION_MODE_FUSEROMACCESS \n");
		break;
	}

	MOTOR_ERR("opf = 0x%x\n", opf);

	return ret;
}

/* functions for interrupt handler */
static irqreturn_t m1120_irq0_handler(int irq, void *dev_id)
{
	MOTOR_LOG("call \n");

	if (!g_chip[DHALL_0]) {
		MOTOR_LOG("g_chip NULL \n");
		return -EINVAL;
	}

	disable_irq_nosync(g_chip[DHALL_0]->irq);
	oplus_dhall_irq_handler(DHALL_0);

	return IRQ_HANDLED;
}

static irqreturn_t m1120_irq1_handler(int irq, void *dev_id)
{
	MOTOR_LOG("call \n");

	if (!g_chip[DHALL_1]) {
		MOTOR_LOG("g_chip NULL \n");
		return -EINVAL;
	}

	disable_irq_nosync(g_chip[DHALL_1]->irq);
	oplus_dhall_irq_handler(DHALL_1);

	return IRQ_HANDLED;
}

int m1120_setup_eint(struct oplus_dhall_chip *chip)
{
	int ret = 0;

	if (gpio_is_valid(chip->irq_gpio)) {
		if (chip->id < DHALL_NUM) {
			ret = gpio_request(chip->irq_gpio, dhall_irq_name[chip->id]);

			if (ret) {
				MOTOR_LOG("unable to request gpio [%d]\n", chip->irq_gpio);
				return -EINVAL;
			}

			ret = gpio_direction_input(chip->irq_gpio);

			msleep(50);

			chip->irq = gpio_to_irq(chip->irq_gpio);

		} else {
			MOTOR_ERR("hall id is invalid %d.\n", chip->id);
		}

	} else {
		chip->irq = -EINVAL;
		MOTOR_ERR("irq_gpio is invalid\n");
	}

	MOTOR_ERR("GPIO %d irq:%d \n", chip->irq_gpio, chip->irq);

	return 0;
}


static int m1120_set_detection_mode(unsigned int id, u8 mode)
{
	u8 data = 0;
	int err = 0;

	if ((id >= DHALL_NUM) || (g_chip[id] == NULL)) {
		MOTOR_LOG("g_chip NULL \n");
		return -EINVAL;
	}

	MOTOR_LOG("m1120 detection mode : %s\n", (mode == 0) ? "POLLING" : "INTERRUPT");

	if (mode & DETECTION_MODE_INTERRUPT) { /*interrupt mode*/
		if (!g_chip[id]->irq_enabled) {
			data = g_chip[id]->reg.map.intsrs | M1120_DETECTION_MODE_INTERRUPT;

			err = m1120_i2c_write_block(g_chip[id], M1120_REG_INTSRS, &data, 1);

			if (err < 0) {
				MOTOR_ERR("config interupt fail %d \n", err);
				return err;
			}

			err = m1120_clear_interrupt(g_chip[id]);

			if (err < 0) {
				MOTOR_ERR("clear interupt fail %d \n", err);
				return err;
			}

			/* requst irq */
			if (id == 0 && g_chip[id]->irq > 0) {
				if ((err = request_threaded_irq(g_chip[id]->irq, NULL,
								&m1120_irq0_handler,
								IRQ_TYPE_LEVEL_LOW | IRQF_ONESHOT,
								"m1120_0", (void *)g_chip[id]->client)) < 0) {
					MOTOR_ERR("IRQ LINE NOT AVAILABLE!!\n");
					return -EINVAL;
				}

			} else if (id == 1 && g_chip[id]->irq > 0) {
				if ((err = request_threaded_irq(g_chip[id]->irq, NULL,
								&m1120_irq1_handler,
								IRQ_TYPE_LEVEL_LOW | IRQF_ONESHOT,
								"m1120_1", (void *)g_chip[id]->client)) < 0) {
					MOTOR_ERR("IRQ LINE NOT AVAILABLE!!\n");
					return -EINVAL;
				}

			} else {
				MOTOR_ERR("IRQ LINE NOT AVAILABLE!! id:%d \n", id);
				return -EINVAL;
			}

			irq_set_irq_wake(g_chip[id]->irq, 1);

			g_chip[id]->irq_enabled = 1;
		}

	} else { /* polling mode*/
		if (g_chip[id]->irq_enabled) {
			data = g_chip[id]->reg.map.intsrs & (0xFF - M1120_DETECTION_MODE_INTERRUPT);

			err = m1120_i2c_write_block(g_chip[id], M1120_REG_INTSRS, &data, 1);

			if (err < 0) {
				MOTOR_ERR("config interupt fail %d \n", err);
				return err;
			}

			disable_irq(g_chip[id]->irq);
			free_irq(g_chip[id]->irq, NULL);

			g_chip[id]->irq_enabled = 0;
		}
	}

	return 0;
}

static int m1120_enable_irq(unsigned int id, bool enable)
{
	if ((id >= DHALL_NUM) || (g_chip[id] == NULL)) {
		MOTOR_LOG("g_chip NULL \n");
		return -EINVAL;
	}

	if (enable) {
		enable_irq(g_chip[id]->irq);

	} else {
		disable_irq_nosync(g_chip[id]->irq);
	}

	return 0;
}

static int m1120_clear_irq(unsigned int id)
{
	if ((id >= DHALL_NUM) || (g_chip[id] == NULL)) {
		MOTOR_LOG("g_chip NULL \n");
		return -EINVAL;
	}

	m1120_clear_interrupt(g_chip[id]);

	return 0;
}

static int m1120_get_irq_state(unsigned int id)
{
	if ((id >= DHALL_NUM) || (g_chip[id] == NULL)) {
		MOTOR_LOG("g_chip NULL \n");
		return -EINVAL;
	}

	return ((g_chip[id]->reg.map.intsrs & M1120_DETECTION_MODE_INTERRUPT) ? 1 : 0);
}

static int m1120_enable_hidden(struct oplus_dhall_chip *chip, bool enable,
			       uint32_t ratio)
{
	int err = 0;
	u8 data = M1120_VAL_HIDDEN_EANBLE;
	u8 rdata = 0, tmp = 0;

	MOTOR_LOG("set hidden enable/disable %d\n", enable);

	if (!enable) {
		/*fix me, need disable hidden reg*/
		return 0;
	}

	err = m1120_i2c_write_block(chip, M1120_REG_HIDDEN, &data,
				    1); /*enable hidden register*/

	if (err < 0) {
		MOTOR_ERR("failed to enable hidden\n");
		return err;
	}

	err = m1120_i2c_read_block(chip, M1120_REG_BIAS, &rdata, 1);

	if (err < 0) {
		MOTOR_ERR("failed to read bias data\n");
		return err;
	}

	tmp = (rdata * ratio) / 100;

	if (tmp > 31) {
		tmp = 31;
	}

	MOTOR_LOG("Bias data rdata %d, set %d\n", tmp, rdata);

	err = m1120_i2c_write_block(chip, M1120_REG_BIAS, &tmp, 1);

	if (err < 0) {
		return err;
	}

	return 0;
}

static void m1120_set_sensitivity(unsigned int id, char *value)
{
	int i = 0;
	uint8_t rwdata;
	struct oplus_dhall_chip *chip = g_chip[id];
	struct hall_srs *srs = NULL;

	MOTOR_LOG("%s\n", __func__);

	for (i = 0; i < sizeof(m1120_ranges) / sizeof(struct hall_srs); i++) {
		srs = &m1120_ranges[i];

		if (!strncmp(srs->name, value, strlen(srs->name))) {
			break;

		} else {
			srs = NULL;
		}
	}

	if (!srs) {
		MOTOR_ERR("%s not match\n", value);
		return;
	}

	/*first check whether the value */
	m1120_i2c_read_block(chip, M1120_REG_INTSRS, &rwdata, 1);

	if (rwdata != chip->reg.map.intsrs) {
		MOTOR_ERR("warning: data not match which write before\n");
	}

	rwdata = ((chip->reg.map.intsrs & 0xF8) | srs->value);
	m1120_i2c_write_block(chip, M1120_REG_INTSRS, &rwdata, 1);

	if (srs->bias && srs->ratio) {
		m1120_enable_hidden(chip, srs->bias, srs->ratio);

	} else {
		/*need to disable bias*/
		m1120_enable_hidden(chip, false, 0);
	}
}

static int m1120_reset_device(struct oplus_dhall_chip *chip)
{
	int err = 0;
	u8 data = 0;

	data = M1120_VAL_SRST_RESET;
	err = m1120_i2c_write_block(chip, M1120_REG_SRST, &data, 1);

	if (err < 0) {
		MOTOR_ERR("sw-reset failed(%d)", err);
		return err;
	}

	msleep(5); /* wait 5ms*/

	chip->reg.map.persint = M1120_PERSISTENCE_COUNT;
	chip->reg.map.intsrs = M1120_DETECTION_MODE | chip->reg.map.range;

	if (chip->reg.map.intsrs & M1120_DETECTION_MODE_INTERRUPT) {
		chip->reg.map.intsrs |= M1120_INTERRUPT_TYPE;
	}

	err = m1120_set_operation_mode(chip, OPERATION_MODE_MEASUREMENT);

	if (err < 0) {
		MOTOR_ERR("m1120_set_operation_mode was failed(%d)", err);
		return err;
	}

	err = m1120_set_frequency(chip, M1120_OPERATION_FREQUENCY);

	err = m1120_enable_hidden(chip, chip->enable_hidden, chip->bias_ratio);

	if (err < 0) {
		MOTOR_ERR("m1120 failed to set hidden\n");
	}

	return err;
}

static void m1120_parse_dts(struct oplus_dhall_chip *chip)
{
	struct device_node *np = NULL;
	int rc = 0;
	char hall_intr_name[32] = {0};
	uint32_t data_range;
	uint32_t value;

	np = chip->client->dev.of_node;

	of_property_read_u32(np, "dhall,id", &chip->id);

	chip->power_gpio = of_get_named_gpio(np, "qcom,hall-power-gpio", 0);

	if (!gpio_is_valid(chip->power_gpio)) {
		MOTOR_LOG("qcom,hall-power-gpio gpio not specified\n");

	} else {
		rc = gpio_request(chip->power_gpio, "hall-power-gpio");

		if (rc) {
			MOTOR_LOG("request hall-power gpio failed, rc=%d\n", rc);
		}
	}

	rc = of_property_read_u32(np, "data-range", &data_range);

	if (rc) {
		chip->reg.map.range = M1120_SENSITIVITY_TYPE;
		MOTOR_LOG("data-range is not specified, use default value:0x%x\n",
			  chip->reg.map.range);

	} else {
		chip->reg.map.range = (uint8_t)data_range;
		MOTOR_LOG("data-range is 0x%x\n", chip->reg.map.range);
	}

	chip->irq_gpio = of_get_named_gpio(np, "dhall,irq-gpio", 0);

	chip->power_2v8 = regulator_get(&chip->client->dev, "vdd_2v8");

	if (IS_ERR_OR_NULL(chip->power_2v8)) {
		MOTOR_ERR("Regulator get failed vdd_2v8\n");
	}

	chip->power_1v8 = regulator_get(&chip->client->dev, "vcc_1v8");

	if (IS_ERR_OR_NULL(chip->power_1v8)) {
		MOTOR_ERR("Regulator get failed vcc_1v8\n");
	}

	chip->pctrl = devm_pinctrl_get(&chip->client->dev);

	if (IS_ERR_OR_NULL(chip->pctrl)) {
		MOTOR_ERR("failed to get pinctrl\n");
		return;
	}

	snprintf(hall_intr_name, 32, "hall%d_interrupt_input", chip->id);
	chip->irq_state = pinctrl_lookup_state(chip->pctrl, hall_intr_name);

	if (IS_ERR_OR_NULL(chip->irq_state)) {
		rc = PTR_ERR(chip->irq_state);
		MOTOR_ERR("pinctrl_lookup_state, err:%d\n", rc);

	} else {
		pinctrl_select_state(chip->pctrl, chip->irq_state);
	}

	chip->enable_hidden = of_property_read_bool(np, "hall,bias_support");

	if (chip->enable_hidden) {
		rc = of_property_read_u32(np, "hall,bias-ratio", &value);

		if (rc) {
			chip->bias_ratio = 100;

		} else {
			chip->bias_ratio = value;
		}
	}
};

struct oplus_dhall_operations  m1120_ops = {
	.get_data  = m1120_get_data,
	.enable_irq = m1120_enable_irq,
	.clear_irq = m1120_clear_irq,
	.get_irq_state = m1120_get_irq_state,
	.set_detection_mode = m1120_set_detection_mode,
	.update_threshold = m1120_update_threshold,
	.dump_regs = m1120_dump_reg,
	.set_reg = m1120_set_reg,
	.is_power_on = m1120_is_power_on,
	.set_sensitivity = m1120_set_sensitivity,
};

static int m1120_i2c_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct oplus_dhall_chip *chip = NULL;
	u8 dev_id = 0xFF;
	int err = 0;

	MOTOR_LOG("call \n");

	chip = devm_kzalloc(&client->dev, sizeof(struct oplus_dhall_chip), GFP_KERNEL);

	if (!chip) {
		MOTOR_ERR("kernel memory alocation was failed \n");
		return -ENOMEM;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		MOTOR_LOG("i2c unsupported\n");
		return -EOPNOTSUPP;
	}

	chip->client = client;

	m1120_parse_dts(chip);

	oplus_m1120_reconfig(chip);

	m1120_set_power(chip, 1);

	dev_id = m1120_get_id(chip);

	if (dev_id != M1120_VAL_DID) {
		MOTOR_ERR("current device id(0x%02x) is not M1120 device id(0x%02x) \n", dev_id,
			  M1120_VAL_DID);
		goto fail;
	}

	err = m1120_reset_device(chip);

	if (err < 0) {
		MOTOR_ERR("m1120_reset_device fail \n");
		goto fail;
	}

	err = m1120_setup_eint(chip);

	if (err < 0) {
		MOTOR_ERR("m1120_setup_eint failed, %d\n", chip->id);
	}

	oplus_register_dhall("m1120", &m1120_ops);

	if (chip->id < DHALL_NUM) {
		g_chip[chip->id] = chip;
		MOTOR_LOG("dhall id[%d], client addr [0x%02x]\n", chip->id, client->addr);

	} else {
		MOTOR_ERR("dhall id[%d] >= DHALL_NUM[%d],fail!\n", chip->id, DHALL_NUM);
		goto fail;
	}

	MOTOR_LOG("success. \n");
	return 0;
fail:
	MOTOR_LOG("fail. \n");
	devm_kfree(&client->dev, chip);
	return -ENXIO;
}

static int m1120_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static int m1120_i2c_suspend(struct device *dev)
{
	return 0;
}

static int m1120_i2c_resume(struct device *dev)
{
	return 0;
}

static const struct of_device_id m1120_match[] = {
	{ .compatible = "oplus,dhall-m1120"},
	{},
};

static const struct i2c_device_id m1120_id[] = {
	{"dhall-m1120", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, m1120_id);

static const struct dev_pm_ops m1120_pm_ops = {
	.suspend = m1120_i2c_suspend,
	.resume = m1120_i2c_resume,
};

static struct i2c_driver m1120_i2c_driver = {
	.driver = {
		.name	= "dhall-m1120",
		.of_match_table =  m1120_match,
		.pm = &m1120_pm_ops,
	},
	.probe		= m1120_i2c_probe,
	.remove		= m1120_i2c_remove,
	.id_table	= m1120_id,
};

static int __init m1120_init(void)
{
	MOTOR_LOG("call\n");
	i2c_add_driver(&m1120_i2c_driver);
	return 0;
}

static void __exit m1120_exit(void)
{
	MOTOR_LOG("call\n");
}

module_init(m1120_init);
module_exit(m1120_exit);

MODULE_DESCRIPTION("M1120 hallswitch driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("mofei");

