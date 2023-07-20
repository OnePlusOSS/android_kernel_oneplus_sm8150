/************************************************************************************
** Copyright (C), 2008-2017, OPLUS Mobile Comm Corp., Ltd
** VENDOR_EDIT
** File: ist8801.c
**
** Description:
**	Definitions for ist8801 digital hall chip.
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

#include "oplus_ist8801.h"
#include "../oplus_motor.h"

#define DHALL_NUM 2
#define IST8801_I2C_BUF_SIZE				(17)
static char *dhall_irq_name[DHALL_NUM] = {
	"ist8801-irq0",
	"ist8801-irq1"
};

static struct oplus_dhall_chip *g_chip[DHALL_NUM] = {NULL, NULL};
/*for otp info reg is 0x01*/
static struct hall_srs ist8801_ranges_1[] = {
	{"40mT", GAIN_2_TIME, false , 0},
	{"35mT", GAIN_2_TIME, false , 10},
	{"20mT", GAIN_2_TIME, false , 28},
	{"15mT", GAIN_4_TIME, false , 17},
	{"10mT", GAIN_8_TIME, false , 0},
};
/*for otp info reg is else*/
static struct hall_srs ist8801_ranges_2[] = {
	{"40mT", GAIN_2_TIME, false , 0},
	{"35mT", GAIN_2_TIME, false , 6},
	{"20mT", GAIN_2_TIME, false , 24},
	{"15mT", GAIN_4_TIME, false , 13},
	{"10mT", GAIN_8_TIME, false , 0},
};


static DEFINE_MUTEX(ist8801_i2c_mutex);
__attribute__((weak)) void oplus_ist8801_reconfig(struct oplus_dhall_chip *chip)
{
	return;
}

static int probe_count = 0;
static int ist8801_i2c_read_block(struct oplus_dhall_chip *chip, u8 addr,
				  u8 *data, u8 len)
{
	u8 reg_addr = addr;
	int err = 0;
	struct i2c_client *client = chip->client;
	struct i2c_msg msgs[2] = {{0}, {0}};

	if (!client) {
		MOTOR_ERR("client null\n");
		return -EINVAL;

	} else if (len >= IST8801_I2C_BUF_SIZE) {
		MOTOR_ERR(" length %d exceeds %d\n", len, IST8801_I2C_BUF_SIZE);
		return -EINVAL;
	}

	mutex_lock(&ist8801_i2c_mutex);

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

	mutex_unlock(&ist8801_i2c_mutex);

	return err;
}

static int ist8801_i2c_write_block(struct oplus_dhall_chip *chip, u8 addr,
				   u8 *data, u8 len)
{
	int err = 0;
	int idx = 0;
	int num = 0;
	char buf[IST8801_I2C_BUF_SIZE] = {0};
	struct i2c_client *client = chip->client;

	if (!client) {
		MOTOR_ERR("client null\n");
		return -EINVAL;

	} else if (len >= IST8801_I2C_BUF_SIZE) {
		MOTOR_ERR(" length %d exceeds %d\n", len, IST8801_I2C_BUF_SIZE);
		return -EINVAL;
	}

	mutex_lock(&ist8801_i2c_mutex);

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
		case IST8801_REG_PERSINT:
			chip->reg.map.persint = data[0];
			break;

		case IST8801_REG_INTSRS:
			chip->reg.map.intsrs = data[0];
			break;

		case IST8801_REG_LTHL:
			chip->reg.map.lthl = data[0];
			break;

		case IST8801_REG_LTHH:
			chip->reg.map.lthh = data[0];
			break;

		case IST8801_REG_HTHL:
			chip->reg.map.hthl = data[0];
			break;

		case IST8801_REG_HTHH:
			chip->reg.map.hthh = data[0];
			break;

		case IST8801_REG_I2CDIS:
			chip->reg.map.i2cdis = data[0];
			break;

		case IST8801_REG_SRST:
			chip->reg.map.srst = data[0];
			break;

		case IST8801_REG_OPF:
			chip->reg.map.opf = data[0];
			break;
		}
	}

	mutex_unlock(&ist8801_i2c_mutex);
	return err;
}

static void ist8801_short_to_2byte(struct oplus_dhall_chip *chip, short x,
				   u8 *hbyte, u8 *lbyte)
{
	unsigned short temp;

	if (x >= 0) {
		temp  = x;

	} else {
		temp  = 65536 + x;
	}

	*lbyte = temp & 0x00ff;
	*hbyte = (temp & 0xff00) >> 8;
}

static short ist8801_2byte_to_short(struct oplus_dhall_chip *chip, u8 hbyte,
				    u8 lbyte)
{
	short x = 0;
	x = (short)((hbyte << 8) | lbyte);

	return x;
}
#if ENABLE_FILTER
static void moving_average_0(u8 *data_hi, u8 *data_lo, u8 mode)
{
	static int first_0 = 0;
	int x, y;
	static int temp_0 = 0;
	x = 0;
	y = 0;
	x = (int) ist8801_2byte_to_short(NULL, *data_hi, *data_lo);

	if (!first_0) {
		if (mode == 0) {
			y = x;
			temp_0 = 4 * x;

		} else {
			y = x;
			temp_0 = 2 * x;
		}

	} else {
		if (mode == 0) {
			temp_0 = (temp_0 >> 2) + 3 * x;
			y = temp_0 >> 2;

		} else {
			temp_0 = 2 * x + temp_0;
			y = temp_0 >> 2;
			temp_0 =  temp_0 >> 1;
		}
	}

	first_0 = 1;

	if (y > 32767) {
		y = 32767;

	} else if (y <= -32768) {
		y = -32768;
	}

	ist8801_short_to_2byte(NULL, (short) y, data_hi, data_lo);
}

static void moving_average_1(u8 *data_hi, u8 *data_lo, u8 mode)
{
	static int first_1 = 0;
	int x, y;
	static int temp_1 = 0;
	x = 0;
	y = 0;
	x = (int) ist8801_2byte_to_short(NULL, *data_hi, *data_lo);

	if (!first_1) {
		if (mode == 0) {
			y = x;
			temp_1 = 4 * x;

		} else {
			y = x;
			temp_1 = 2 * x;
		}

	} else {
		if (mode == 0) {
			temp_1 = (temp_1 >> 2) + 3 * x;
			y = temp_1 >> 2;

		} else {
			temp_1 = 2 * x + temp_1;
			y = temp_1 >> 2;
			temp_1 =  temp_1 >> 1;
		}
	}

	first_1 = 1;

	if (y > 32767) {
		y = 32767;

	} else if (y <= -32768) {
		y = -32768;
	}

	ist8801_short_to_2byte(NULL, (short) y, data_hi, data_lo);
}
#endif

static int ist8801_get_id(struct oplus_dhall_chip *chip)
{
	u8 data = 0;
	ist8801_i2c_read_block(chip, IST8801_REG_DID, &data, 1);

	MOTOR_LOG("id = 0x%x \n", data);

	return data;
}

static int ist8801_get_data(unsigned int id, short *data)
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
	err = ist8801_i2c_read_block(g_chip[id], IST8801_REG_ST1, buf, sizeof(buf));

	if (err < 0) {
		MOTOR_LOG(" fail %d \n", err);
		return err;
	}

	/* (2) collect data*/
	if ((buf[0] & 0x01) | (buf[0] & 0x4)) { /*buf[2] for data over run status*/
#if ENABLE_FILTER
		if (id == 0) {
			moving_average_0(&buf[2], &buf[1], FILTER_MODE);

		} else {
			moving_average_1(&buf[2], &buf[1], FILTER_MODE);
		}

#endif

		value = ist8801_2byte_to_short(g_chip[id], buf[2], buf[1]);

	} else {
		MOTOR_LOG("ist8801: st1(0x%02X) is not DRDY.\n", buf[0]);
		*data = pre_value[id];
		return err;
	}

	*data = value;
	pre_value[id] = value;

	return 0;
}

static void ist8801_dump_reg(unsigned int id, u8 *buf)
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

		err = ist8801_i2c_read_block(g_chip[id], i, &val, 1);

		if (err < 0) {
			sprintf(buf, "read reg error!\n");
			return;
		}

		sprintf(_buf,  "reg 0x%x:0x%x\n", i, val);
		strcat(buffer, _buf);
	}

	err = ist8801_i2c_read_block(g_chip[id], 0x54, &val, 1);

	if (err < 0) {
		sprintf(buf, "read reg error!\n");
		return;
	}

	sprintf(_buf,  "reg 0x%x:0x%x\n", 0x54, val);
	strcat(buffer, _buf);

	sprintf(buf, "%s\n", buffer);
	MOTOR_LOG("%s \n", buf);
}

static int ist8801_set_reg(unsigned int id, int reg, int val)
{
	u8 data = (u8)val;

	if ((id >= DHALL_NUM) || (g_chip[id] == NULL)) {
		MOTOR_LOG("g_chip NULL \n");
		return -EINVAL;
	}

	ist8801_i2c_write_block(g_chip[id], (u8)reg, &data, 1);

	return 0;
}


static bool ist8801_is_power_on(unsigned int id)
{
	if ((id >= DHALL_NUM) || (g_chip[id] == NULL)) {
		MOTOR_LOG("g_chip NULL \n");
		return false;
	}

	return g_chip[id]->is_power_on;
}

static int ist8801_set_power_gpio_down(struct oplus_dhall_chip *chip)
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

static int ist8801_set_power_gpio_up(struct oplus_dhall_chip *chip)
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
static int ist8801_set_power(struct oplus_dhall_chip *chip, bool on)
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
		ist8801_set_power_gpio_up(chip);

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
		ist8801_set_power_gpio_down(chip);

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
/*use the function change odr,you can use it optionally*/
/*
static int ist8801_set_frequency(struct oplus_dhall_chip *chip,int frequency)
{
        int err = 0;
        u8 rdata = 0,ifcntl = 0;

        ist8801_i2c_write_block(chip, IST8801_REG_OPF, &rdata,1);

        ist8801_i2c_read_block(chip, IST8801_REG_IFCNTL, &ifcntl,1);
        ifcntl |= 0x04;
        ist8801_i2c_write_block(chip, IST8801_REG_IFCNTL, &ifcntl,1);

        rdata = frequency;
        MOTOR_LOG("IST8801_REG_OPF register : 0x%x\n", rdata);

        err = ist8801_i2c_write_block(chip, IST8801_REG_OPF, &rdata,1);
        if(err < 0) {
                MOTOR_LOG("set-opf was failed(%d)", err);
                return err;
        }
        return 0;
}
*/
static int ist8801_clear_interrupt(struct oplus_dhall_chip *chip)
{
	int ret = 0;

	u8 data = chip->reg.map.persint | 0x01;
	MOTOR_LOG("step1- id:%d   ist8801_clear_interrupt chip->reg.map.persint register : 0x%x,data:0x%x\n",
		  chip->id, chip->reg.map.persint, data);

	ret = ist8801_i2c_write_block(chip, IST8801_REG_PERSINT, &data, 1);

	chip->reg.map.persint = chip->reg.map.persint & 0xfe;
	data = chip->reg.map.persint;
	MOTOR_LOG("step2- id:%d   ist8801_clear_interrupt chip->reg.map.persint register : 0x%x,data:0x%x\n",
		  chip->id, chip->reg.map.persint, data);
	ret = ist8801_i2c_write_block(chip, IST8801_REG_PERSINT, &data, 1);

	return ret;
}

/*
IST8801_ADC_BIT_NUM
8-bit:0x10 : threshold range: 127~-128
9-bit:0x0e : threshold range: 255~-256
10-bit:0x0c : threshold range: 511~-512
11-bit:0x0a : threshold range: 1023~-1024
12-bit:0x08 : threshold range: 2047~-2048
13-bit:0x06 : threshold range: 4095~-4096
14-bit:0x04 : threshold range: 8191~-8192
15-bit:0x02 : threshold range: 16383~-16384
16-bit: other : threshold range: 32767~-32768
*/
static bool ist8801_update_threshold(unsigned int id, int position,
				     short lowthd, short highthd)
{
	u8 lthh, lthl, hthh, hthl;
	int err = 0;

	if ((id >= DHALL_NUM) || (g_chip[id] == NULL)) {
		MOTOR_LOG("g_chip NULL \n");
		return -EINVAL;
	}

	MOTOR_LOG("id %d ,low:%d, high:%d  \n", id , lowthd, highthd);

	err = ist8801_clear_interrupt(g_chip[id]);

	if (g_chip[id]->reg.map.intsrs & IST8801_VAL_INTSRS_INTTYPE_WITHIN) {
		ist8801_short_to_2byte(g_chip[id], highthd, &hthh, &hthl);
		ist8801_short_to_2byte(g_chip[id], lowthd, &lthh, &lthl);

		err |= ist8801_i2c_write_block(g_chip[id], IST8801_REG_HTHH, &hthh, 1);
		err |= ist8801_i2c_write_block(g_chip[id], IST8801_REG_HTHL, &hthl, 1);
		err |= ist8801_i2c_write_block(g_chip[id], IST8801_REG_LTHH, &lthh, 1);
		err |= ist8801_i2c_write_block(g_chip[id], IST8801_REG_LTHL, &lthl, 1);
	}

	if (err < 0) {
		MOTOR_ERR("fail %d\n", err);
		return false;

	} else {
		return true;
	}
}

static int ist8801_set_operation_mode(struct oplus_dhall_chip *chip, int mode)
{
	u8 opf = 0, ifcntl = 0;
	int ret = 0;

	switch (mode) {
	case OPERATION_MODE_POWERDOWN:
		opf = 0;
		ifcntl = 0;
		ret = ist8801_i2c_write_block(chip, IST8801_REG_OPF, &opf, 1);
		ret = ist8801_i2c_read_block(chip, IST8801_REG_IFCNTL, &ifcntl, 1);
		ifcntl |= 0x04;
		ret = ist8801_i2c_write_block(chip, IST8801_REG_IFCNTL, &ifcntl, 1);
		MOTOR_ERR("operation mode :OPERATION_MODE_POWERDOWN \n");
		break;

	case OPERATION_MODE_MEASUREMENT:
		opf = 0x00;
		MOTOR_ERR("opf = 0x%x\n", opf);
		/*IST8801_REG_ACTION is 0x20*/
		ret = ist8801_i2c_write_block(chip, IST8801_REG_ACTION, &opf, 1);

		/*delay for 5 ms*/
		usleep_range(5000, 5000);

		opf = FREQUENCY;
		MOTOR_ERR("opf = 0x%x\n", opf);
		ret = ist8801_i2c_write_block(chip, IST8801_REG_OPF, &opf, 1);
		MOTOR_ERR("operation mode :OPERATION_MODE_MEASUREMENT \n");
		break;

	case OPERATION_MODE_SUSPEND :
		opf = 0x00;
		MOTOR_ERR("opf = 0x%x\n", opf);
		ret = ist8801_i2c_write_block(chip, IST8801_REG_OPF, &opf, 1);

		ret = ist8801_i2c_read_block(chip, IST8801_REG_IFCNTL, &ifcntl, 1);
		ifcntl |= 0x04;
		ret = ist8801_i2c_write_block(chip, IST8801_REG_IFCNTL, &ifcntl, 1);

		opf = 0x02;
		MOTOR_ERR("opf = 0x%x\n", opf);
		/*IST8801_REG_ACTION is 0x20*/
		ret = ist8801_i2c_write_block(chip, IST8801_REG_ACTION, &opf, 1);
		MOTOR_ERR("operation mode :OPERATION_MODE_SUSPEND \n");

		/*delay for 5 ms*/
		usleep_range(5000, 5000);
		break;
	}

	MOTOR_ERR("opf = 0x%x\n", opf);

	return ret;
}


/* functions for interrupt handler */
static irqreturn_t ist8801_irq0_handler(int irq, void *dev_id)
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

static irqreturn_t ist8801_irq1_handler(int irq, void *dev_id)
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

int ist8801_setup_eint(struct oplus_dhall_chip *chip)
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


static int ist8801_set_detection_mode(unsigned int id, u8 mode)
{
	u8 data = 0;
	int err = 0;

	if ((id >= DHALL_NUM) || (g_chip[id] == NULL)) {
		MOTOR_LOG("g_chip NULL \n");
		return -EINVAL;
	}

	MOTOR_LOG("ist8801 detection mode : %s\n",
		  (mode == 0) ? "POLLING" : "INTERRUPT");

	if (mode & DETECTION_MODE_INTERRUPT) { /*interrupt mode*/
		if (!g_chip[id]->irq_enabled) {
			data = g_chip[id]->reg.map.intsrs | IST8801_DETECTION_MODE_INTERRUPT;
			err = ist8801_i2c_write_block(g_chip[id], IST8801_REG_INTSRS, &data, 1);

			if (err < 0) {
				MOTOR_ERR("config interupt fail %d \n", err);
				return err;
			}

			err = ist8801_clear_interrupt(g_chip[id]);

			if (err < 0) {
				MOTOR_ERR("clear interupt fail %d \n", err);
				return err;
			}

			/* requst irq */
			if (id == 0 && g_chip[id]->irq > 0) {
				if ((err = request_threaded_irq(g_chip[id]->irq, NULL,
								&ist8801_irq0_handler,
								IRQ_TYPE_LEVEL_LOW | IRQF_ONESHOT,
								"ist8801_0", (void *)g_chip[id]->client)) < 0) {
					MOTOR_ERR("IRQ LINE NOT AVAILABLE!!\n");
					return -EINVAL;
				}

			} else if (id == 1 && g_chip[id]->irq > 0) {
				if ((err = request_threaded_irq(g_chip[id]->irq, NULL,
								&ist8801_irq1_handler,
								IRQ_TYPE_LEVEL_LOW | IRQF_ONESHOT,
								"ist8801_1", (void *)g_chip[id]->client)) < 0) {
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
			data = g_chip[id]->reg.map.intsrs & (0xFF - IST8801_DETECTION_MODE_INTERRUPT);

			err = ist8801_i2c_write_block(g_chip[id], IST8801_REG_INTSRS, &data, 1);

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

static int ist8801_enable_irq(unsigned int id, bool enable)
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

static int ist8801_clear_irq(unsigned int id)
{
	if ((id >= DHALL_NUM) || (g_chip[id] == NULL)) {
		MOTOR_LOG("g_chip NULL \n");
		return -EINVAL;
	}

	ist8801_clear_interrupt(g_chip[id]);

	return 0;
}

static int ist8801_get_irq_state(unsigned int id)
{
	if ((id >= DHALL_NUM) || (g_chip[id] == NULL)) {
		MOTOR_LOG("g_chip NULL \n");
		return -EINVAL;
	}

	return ((g_chip[id]->reg.map.intsrs & IST8801_DETECTION_MODE_INTERRUPT) ? 1 :
		0);
}

static void ist8801_set_sensitivity(unsigned int id, char *value)
{
	int i = 0;
	uint8_t rwdata;
	struct oplus_dhall_chip *chip = g_chip[id];
	struct hall_srs *srs = NULL, *ist8801_ranges = NULL;
	int len1 = 0, len2 = 0, len = 0;
	uint8_t temp_opf, err;

	MOTOR_LOG("ist8801_set_sensitivity, id=%d\n", id);

	if ((id >= DHALL_NUM) || (g_chip[id] == NULL)) {
		MOTOR_LOG("g_chip NULL \n");
		return;
	}

	MOTOR_LOG("%s\n", __func__);

	len1 = sizeof(ist8801_ranges_1) / sizeof(struct hall_srs);
	len2 = sizeof(ist8801_ranges_2) / sizeof(struct hall_srs);

	if (0x01 == chip->origin_info) {
		len = len1;
		ist8801_ranges = ist8801_ranges_1;

	} else {
		len = len2;
		ist8801_ranges = ist8801_ranges_2;
	}

	for (i = 0; i < len; i++) {
		srs = &ist8801_ranges[i];

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

	/*backup the data of IST8801_REG_OPF*/
	temp_opf = 0x00;
	err = ist8801_i2c_read_block(chip, IST8801_REG_OPF, &temp_opf, 1);

	/*write IST8801_REG_OPF to 0x00*/
	rwdata = 0x00;
	ist8801_i2c_write_block(chip, IST8801_REG_OPF, &rwdata, 1);

	/*mask 0x40 reg with 0x04*/
	err = ist8801_i2c_read_block(chip, IST8801_REG_IFCNTL, &rwdata, 1);
	rwdata |= 0x04;
	ist8801_i2c_write_block(chip, IST8801_REG_IFCNTL, &rwdata, 1);

	/*Just change dynamic range and keep bit resolution*/
	/*(DYNAMIC_GAIN_ADC_BIT & 0x1E) -> clean up the dynamic field*/
	/* srs->value | (DYNAMIC_GAIN_ADC_BIT & 0x1E) -> update the dynamic field setting*/
	rwdata = ((DYNAMIC_GAIN_ADC_BIT & 0x1E) | srs->value);
	ist8801_i2c_write_block(chip, IST8801_REG_CNTL2, &rwdata, 1);

	MOTOR_LOG("set sensitivity IST8801_REG_CNTL2 = 0x%x \n", rwdata);

	/*check the IST8801_REG_CNTL2 data*/
	rwdata = 0;
	ist8801_i2c_read_block(chip, IST8801_REG_CNTL2, &rwdata, 1);

	MOTOR_LOG("get sensitivity IST8801_REG_CNTL2 = 0x%x \n", rwdata);

	/*compensate reg 0x54*/
	rwdata = ((uint8_t) srs->ratio) + chip->origin_gain;
	MOTOR_LOG("set sensitivity IST8801_REG_GAINCNTL = %d \n", rwdata);

	ist8801_i2c_write_block(chip, IST8801_REG_GAINCNTL, &rwdata, 1);

	/*check data is correct*/
	rwdata = 0;
	err = ist8801_i2c_read_block(chip, IST8801_REG_GAINCNTL, &rwdata, 1);

	MOTOR_LOG("get sensitivity IST8801_REG_GAINCNTL = %d \n", rwdata);

	/*recovery IST8801_REG_OPF*/
	rwdata = temp_opf;
	ist8801_i2c_write_block(chip, IST8801_REG_OPF, &rwdata, 1);
}
/*
IST8801_ADC_BIT_NUM
8-bit:0x10
9-bit:0x0e
10-bit:0x0c
11-bit:0x0a
12-bit:0x08
13-bit:0x06
14-bit:0x04
15-bit:0x02
16-bit: other
*/
static int ist8801_reset_device(struct oplus_dhall_chip *chip)
{
	int err = 0;
	u8 data = 0;

	data = 0;
	err = ist8801_i2c_write_block(chip, IST8801_REG_ACTION, &data, 1);

	usleep_range(5000, 5000);

	data = IST8801_VAL_SRST_RESET;
	err = ist8801_i2c_write_block(chip, IST8801_REG_SRST, &data, 1);

	if (err < 0) {
		MOTOR_ERR("sw-reset failed(%d)", err);
		return err;
	}

	msleep(20); /* wait 20ms*/

	err = ist8801_i2c_read_block(chip, IST8801_REG_DID, &data, 1);

	if (err < 0) {
		MOTOR_ERR("read IST8801_REG_DID failed(%d)", err);
		return err;
	}

	if (data != IST8801_VAL_DID) {
		MOTOR_ERR("current device id(0x%02X) is not IST8801 device id(0x%02X)", data,
			  IST8801_VAL_DID);
		/* TODO: unitl DID defined*/
		/*return -ENXIO;*/
	}

	/*Disable TST PAD voltage*/
	data = 0x04;
	err = ist8801_i2c_write_block(chip, IST8801_REG_TSTCNTL, &data, 1);

	/*backup the gain data*/
	data = 0x00;
	ist8801_i2c_read_block(chip, IST8801_REG_GAINCNTL, &data, 1);
	chip->origin_gain = data;

	MOTOR_LOG("chip->origin_gain = %d \n", chip->origin_gain);

	/*backup the osr data*/
	data = 0x00;
	ist8801_i2c_read_block(chip, IST8801_REG_OSRCNTL, &data, 1);
	chip->origin_osr = data;

	MOTOR_LOG("chip->origin_osr = %d \n", chip->origin_osr);

	/*backup the info data*/
	data = 0x00;
	ist8801_i2c_read_block(chip, IST8801_REG_INFO, &data, 1);
	chip->origin_info = data;

	MOTOR_LOG("chip->origin_info = %d \n", chip->origin_info);

	chip->reg.map.persint = IST8801_PERSISTENCE_COUNT;
	data = chip->reg.map.persint;
	err = ist8801_i2c_write_block(chip, IST8801_REG_PERSINT, &data, 1);


	chip->reg.map.intsrs = IST8801_DETECTION_MODE | chip->reg.map.range;

	if (chip->reg.map.intsrs & IST8801_DETECTION_MODE_INTERRUPT) {
		chip->reg.map.intsrs |= IST8801_INTERRUPT_TYPE;
	}

	data = chip->reg.map.intsrs;
	err = ist8801_i2c_write_block(chip, IST8801_REG_INTSRS, &data, 1);

#if DISABLE_TEMP_CONPEN
	data = 0x01;
	err = ist8801_i2c_write_block(chip, IST8801_REG_IFCNTL, &data, 1);

	if (err < 0) {
		MOTOR_ERR("IST8801_REG_IFCNTL failed(%d)", err);
		return err;
	}

#endif
	err = ist8801_set_operation_mode(chip, OPERATION_MODE_MEASUREMENT);

	if (err < 0) {
		MOTOR_ERR("ist8801_set_operation_mode was failed(%d)", err);
		return err;
	}

	return err;
}

static void ist8801_parse_dts(struct oplus_dhall_chip *chip)
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
		chip->reg.map.range = IST8801_SENSITIVITY_TYPE;
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

struct oplus_dhall_operations  ist8801_ops = {
	.get_data  = ist8801_get_data,
	.enable_irq = ist8801_enable_irq,
	.clear_irq = ist8801_clear_irq,
	.get_irq_state = ist8801_get_irq_state,
	.set_detection_mode = ist8801_set_detection_mode,
	.update_threshold = ist8801_update_threshold,
	.dump_regs = ist8801_dump_reg,
	.set_reg = ist8801_set_reg,
	.is_power_on = ist8801_is_power_on,
	.set_sensitivity = ist8801_set_sensitivity,
};

static int ist8801_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	struct oplus_dhall_chip *chip = NULL;
	u8 dev_id = 0xFF;
	int err = 0;

	if (client->addr == 0x0D || client->addr == 0x0E) {
		if (probe_count == 0) {
			client->addr = 0x0F;
			probe_count++;

		} else {
			client->addr = 0x0C;
		}
	}

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

	ist8801_parse_dts(chip);

	oplus_ist8801_reconfig(chip);

	ist8801_set_power(chip, 1);

	dev_id = ist8801_get_id(chip);

	if (dev_id != IST8801_VAL_DID) {
		MOTOR_ERR("current device id(0x%02x) is not ist8801 device id(0x%02x) \n",
			  dev_id, IST8801_VAL_DID);
		goto fail;
	}

	err = ist8801_reset_device(chip);

	if (err < 0) {
		MOTOR_ERR("ist8801_reset_device fail \n");
		goto fail;
	}

	err = ist8801_setup_eint(chip);

	if (err < 0) {
		MOTOR_ERR("ist8801_setup_eint failed, %d\n", chip->id);
	}

	oplus_register_dhall("ist8801", &ist8801_ops);

	if (chip->id < DHALL_NUM) {
		g_chip[chip->id] = chip;
		MOTOR_LOG("dhall id[%d], client addr [0x%02x]\n", chip->id, client->addr);

	} else {
		MOTOR_ERR("dhall id[%d] >= DHALL_NUM[%d],fail!\n", chip->id, DHALL_NUM);
		goto fail;
	}

	ist8801_set_sensitivity(chip->id, "20mT");

	MOTOR_LOG("success. \n");
	return 0;
fail:
	MOTOR_LOG("fail. \n");
	devm_kfree(&client->dev, chip);
	return -ENXIO;
}

static int ist8801_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static int ist8801_i2c_suspend(struct device *dev)
{
	int ret = 0;
	int i = 0;

	for (i = 0 ; i < DHALL_NUM ; i++) {
		if (g_chip[i] != NULL) {
			ret = ist8801_set_operation_mode(g_chip[i], OPERATION_MODE_SUSPEND);
		}
	}

	return 0;
}

static int ist8801_i2c_resume(struct device *dev)
{
	int ret = 0;
	int i = 0;

	for (i = 0 ; i < DHALL_NUM ; i++) {
		if (g_chip[i] != NULL) {
			ret = ist8801_set_operation_mode(g_chip[i], OPERATION_MODE_MEASUREMENT);
		}
	}

	return 0;
}

static const struct of_device_id ist8801_match[] = {
	{ .compatible = "oplus,dhall-ist8801"},
	{},
};

static const struct i2c_device_id ist8801_id[] = {
	{"dhall-ist8801", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, ist8801_id);

static const struct dev_pm_ops ist8801_pm_ops = {
	.suspend = ist8801_i2c_suspend,
	.resume = ist8801_i2c_resume,
};

static struct i2c_driver ist8801_i2c_driver = {
	.driver = {
		.name	= "dhall-ist8801",
		.of_match_table =  ist8801_match,
		.pm = &ist8801_pm_ops,
	},
	.probe		= ist8801_i2c_probe,
	.remove		= ist8801_i2c_remove,
	.id_table	= ist8801_id,
};

static int __init ist8801_init(void)
{
	MOTOR_LOG("call\n");
	i2c_add_driver(&ist8801_i2c_driver);
	return 0;
}

static void __exit ist8801_exit(void)
{
	MOTOR_LOG("call\n");
}

module_init(ist8801_init);
module_exit(ist8801_exit);

MODULE_DESCRIPTION("ist8801 hallswitch driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("zengchao");

