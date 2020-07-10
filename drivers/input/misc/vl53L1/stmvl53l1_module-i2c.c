/**************************************************************************
 * Copyright (c) 2016, STMicroelectronics - All Rights Reserved

 License terms: BSD 3-clause "New" or "Revised" License.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 3. Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software
 without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************************/

/**
 * @file stmvl53l1_module-i2c.c
 *
 *  implement STM VL53L1 module interface i2c wrapper + control
 *  using linux native i2c + gpio + reg api
 */
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/time.h>
#include <linux/platform_device.h>
#include <linux/version.h>

/*
 * power specific includes
 */
#include <linux/pwm.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/clk.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>

#include "stmvl53l1-i2c.h"
#include "stmvl53l1.h"
#include "cam_cci_ctrl_interface.h"

#define STMVL53L1_SLAVE_ADDR	(0x52>>1)
#define IGNORE_IRQ  0       //songyt add
/** @ingroup drv_port
 * @{
 */

/**
 * control specific debug message echo
 *
 * Set to 0/1 do not remove
 *
 * most dbg warn err messages goes true main driver macro
 * this one permit some specific debug without activating all main dbg
 */
#define MODI2C_DEBUG	0

/*
 * mutex to handle device i2c address changes. It allow to avoid multiple
 * device active with same i2c addresses at the same time. Note that we don't
 * support case where boot_reg has the same value as a final i2c address of
 * another device.
 */
static DEFINE_MUTEX(dev_addr_change_mutex);

/**
 * i2c client assigned to our driver
 *
 * this is use for stm test purpose as we fake client create and regstration
 * we stores the i2c client for release in clean-up overwise we wan't reload
 * the module multiple time
 *
 * in a normal dev tree prod system this is not required
 */
static struct i2c_client *stm_test_i2c_client;

/*
 * pi3:
 * insmod stmvl53l1.ko force_device=1 adapter_nb=1 xsdn_gpio_nb=19
 * intr_gpio_nb=16 pwren_gpio_nb=12
 *
 * panda
 * insmod stmvl53l1.ko force_device=1 adapter_nb=4 xsdn_gpio_nb=56
 * intr_gpio_nb=59 pwren_gpio_nb=55
*/

static int force_device;
static int adapter_nb = -1;
static int xsdn_gpio_nb = -1;
static int pwren_gpio_nb = -1;
static int intr_gpio_nb = -1;

module_param(force_device, int, 0000);
MODULE_PARM_DESC(force_device, "force device insertion at module init");

module_param(adapter_nb, int, 0000);
MODULE_PARM_DESC(adapter_nb, "i2c adapter to use");

module_param(xsdn_gpio_nb, int, 0000);
MODULE_PARM_DESC(xsdn_gpio_nb, "select gpio numer to use for vl53l1 reset");

module_param(pwren_gpio_nb, int, 0000);
MODULE_PARM_DESC(pwren_gpio_nb, "select gpio numer to use for vl53l1 power");

module_param(intr_gpio_nb, int, 0000);
MODULE_PARM_DESC(intr_gpio_nb, "select gpio numer to use for vl53l1 interrupt");

/**
 * warn message
 *
 * @warning use only in scope where i2c_data ptr is present
 **/
#define modi2c_warn(fmt, ...)\
	dev_WARN(&i2c_data->client->dev, fmt, ##__VA_ARGS__)

/**
 * err message
 *
 * @warning use only in scope where i2c_data ptr is present
 */
#define modi2c_err(fmt, ...)\
	dev_err(&i2c_data->client->dev, fmt, ##__VA_ARGS__)



#if MODI2C_DEBUG
#	define modi2c_dbg(fmt, ...)\
		pr_devel("%s "fmt"\n", __func__, ##__VA_ARGS__)
#else
#	define modi2c_dbg(...)	(void)0
#endif

static int insert_device(void)
{
	int ret = 0;
	struct i2c_adapter *adapter;
	struct i2c_board_info info = {
		.type = "stmvl53l1",
		.addr = STMVL53L1_SLAVE_ADDR,
	};

	memset(&info, 0, sizeof(info));
	strcpy(info.type, "stmvl53l1");
	info.addr = STMVL53L1_SLAVE_ADDR;
	adapter = i2c_get_adapter(adapter_nb);
	if (!adapter) {
		ret = -EINVAL;
		goto done;
	}
	stm_test_i2c_client = i2c_new_device(adapter, &info);
	if (!stm_test_i2c_client)
		ret = -EINVAL;

done:
	return ret;
}

static int get_xsdn(struct device *dev, struct i2c_data *i2c_data)
{
	int rc = 0;

	i2c_data->io_flag.xsdn_owned = 0;
	if (i2c_data->xsdn_gpio == -1) {
		vl53l1_errmsg("reset gpio is required");
		rc = -ENODEV;
		goto no_gpio;
	}

	vl53l1_dbgmsg("request xsdn_gpio %d", i2c_data->xsdn_gpio);
	rc = gpio_request(i2c_data->xsdn_gpio, "vl53l1_xsdn");
	if (rc) {
		vl53l1_errmsg("fail to acquire xsdn %d", rc);
		goto request_failed;
	}

	rc = gpio_direction_output(i2c_data->xsdn_gpio, 0);
	if (rc) {
		vl53l1_errmsg("fail to configure xsdn as output %d", rc);
		goto direction_failed;
	}
	i2c_data->io_flag.xsdn_owned = 1;

	return rc;

direction_failed:
	gpio_free(i2c_data->xsdn_gpio);

request_failed:
no_gpio:
	return rc;
}

static void put_xsdn(struct i2c_data *i2c_data)
{
	if (i2c_data->io_flag.xsdn_owned) {
		vl53l1_dbgmsg("release xsdn_gpio %d", i2c_data->xsdn_gpio);
		gpio_free(i2c_data->xsdn_gpio);
		i2c_data->io_flag.xsdn_owned = 0;
		i2c_data->xsdn_gpio = -1;
	}
	i2c_data->xsdn_gpio = -1;
}

static int get_pwren(struct device *dev, struct i2c_data *i2c_data)
{
	int rc = 0;

	i2c_data->io_flag.pwr_owned = 0;
	if (i2c_data->pwren_gpio == -1) {
		vl53l1_wanrmsg("pwren gpio disable");
		goto no_gpio;
	}

	vl53l1_dbgmsg("request pwren_gpio %d", i2c_data->pwren_gpio);
	rc = gpio_request(i2c_data->pwren_gpio, "vl53l1_pwren");
	if (rc) {
		vl53l1_errmsg("fail to acquire pwren %d", rc);
		goto request_failed;
	}

	rc = gpio_direction_output(i2c_data->pwren_gpio, 0);
	if (rc) {
		vl53l1_errmsg("fail to configure pwren as output %d", rc);
		goto direction_failed;
	}
	i2c_data->io_flag.pwr_owned = 1;

	return rc;

direction_failed:
	gpio_free(i2c_data->xsdn_gpio);

request_failed:
no_gpio:
	return rc;
}

static void put_pwren(struct i2c_data *i2c_data)
{
	if (i2c_data->io_flag.pwr_owned) {
		vl53l1_dbgmsg("release pwren_gpio %d", i2c_data->pwren_gpio);
		gpio_free(i2c_data->pwren_gpio);
		i2c_data->io_flag.pwr_owned = 0;
		i2c_data->pwren_gpio = -1;
	}
	i2c_data->pwren_gpio = -1;
}

static int get_intr(struct device *dev, struct i2c_data *i2c_data)
{
	int rc = 0;

	i2c_data->io_flag.intr_owned = 0;
	if (i2c_data->intr_gpio == -1) {
		vl53l1_wanrmsg("no interrupt gpio");
		goto no_gpio;
	}

	vl53l1_dbgmsg("request intr_gpio %d", i2c_data->intr_gpio);
	rc = gpio_request(i2c_data->intr_gpio, "vl53l1_intr");
	if (rc) {
		vl53l1_errmsg("fail to acquire intr %d", rc);
		goto request_failed;
	}

	rc = gpio_direction_input(i2c_data->intr_gpio);
	if (rc) {
		vl53l1_errmsg("fail to configure intr as input %d", rc);
		goto direction_failed;
	}

	i2c_data->irq = gpio_to_irq(i2c_data->intr_gpio);
	if (i2c_data->irq < 0) {
		vl53l1_errmsg("fail to map GPIO: %d to interrupt:%d\n",
				i2c_data->intr_gpio, i2c_data->irq);
		goto irq_failed;
	}
	i2c_data->io_flag.intr_owned = 1;

	return rc;

irq_failed:
direction_failed:
	gpio_free(i2c_data->intr_gpio);

request_failed:
no_gpio:
	return rc;
}

static void put_intr(struct i2c_data *i2c_data)
{
	if (i2c_data->io_flag.intr_owned) {
		if (i2c_data->io_flag.intr_started) {
			free_irq(i2c_data->irq, i2c_data);
			i2c_data->io_flag.intr_started = 0;
		}
		vl53l1_dbgmsg("release intr_gpio %d", i2c_data->intr_gpio);
		gpio_free(i2c_data->intr_gpio);
		i2c_data->io_flag.intr_owned = 0;
	}
	i2c_data->intr_gpio = -1;
}

/**
 *  parse dev tree for all platform specific input
 */
int stmvl53l1_parse_tree(struct device *dev, struct i2c_data *i2c_data)
{
	int rc = 0;
	enum of_gpio_flags flags;
	/* if force device is in use then gpio nb comes from module param else
	 * we use devicetree.
	 */
	i2c_data->vdd = NULL;
	i2c_data->pwren_gpio = -1;
	i2c_data->xsdn_gpio = -1;
	i2c_data->intr_gpio = -1;
	i2c_data->boot_reg = STMVL53L1_SLAVE_ADDR;
	if (force_device) {
		i2c_data->xsdn_gpio = xsdn_gpio_nb;
		i2c_data->pwren_gpio = pwren_gpio_nb;
		i2c_data->intr_gpio = intr_gpio_nb;
	} else if (dev->of_node) {
		/* power : either vdd or pwren_gpio. try reulator first */
		i2c_data->vdd = regulator_get(dev, "laser_vdd");
		if (IS_ERR(i2c_data->vdd) || i2c_data->vdd == NULL) {
			i2c_data->vdd = NULL;
			vl53l1_wanrmsg(
        		"no laser_vdd, fatal.");
		}
		i2c_data->pwren_gpio = of_get_named_gpio_flags(dev->of_node, "pwren-gpio", 0,
						  &flags);
		if (gpio_is_valid(i2c_data->pwren_gpio)) {
			vl53l1_dbgmsg("pwren-gpio %d",
				i2c_data->pwren_gpio);
		} else {
			vl53l1_wanrmsg(
        		"no regulator, nor power gpio => power ctrl disabled");
			i2c_data->pwren_gpio = -1;
		}
		i2c_data->xsdn_gpio = of_get_named_gpio_flags(dev->of_node, "xsdn-gpio", 0,
						  &flags);
		if (gpio_is_valid(i2c_data->xsdn_gpio)) {
			vl53l1_dbgmsg("xsdn-gpio %d",
				i2c_data->xsdn_gpio);
		} else {
			vl53l1_wanrmsg("Unable to find xsdn-gpio %d",
				i2c_data->xsdn_gpio);
			i2c_data->xsdn_gpio = -1;
		}
#if IGNORE_IRQ
        i2c_data->intr_gpio = -1;
        vl53l1_dbgmsg("do not use intr-gpio!");
#else
		i2c_data->intr_gpio = of_get_named_gpio_flags(dev->of_node, "intr-gpio", 0,&flags);
		if (gpio_is_valid(i2c_data->intr_gpio)) {
			vl53l1_dbgmsg("intr-gpio %d",
				i2c_data->intr_gpio);
		} else {
			vl53l1_wanrmsg("Unable to find intr-gpio %d",
				i2c_data->intr_gpio);
			i2c_data->intr_gpio = -1;
		}
#endif
	}

	/* configure gpios */
	rc = get_xsdn(dev, i2c_data);
	if (rc)
		goto no_xsdn;
	rc = get_pwren(dev, i2c_data);
	if (rc)
		goto no_pwren;
	rc = get_intr(dev, i2c_data);
	if (rc){
        vl53l1_errmsg("get_intr failed.");
		goto no_intr;
    }
	return rc;

no_intr:
#if IGNORE_IRQ
    return 0;
#else
	if (i2c_data->vdd) {
		regulator_put(i2c_data->vdd);
		i2c_data->vdd = NULL;
	}
	put_pwren(i2c_data);
#endif
no_pwren:
	put_xsdn(i2c_data);
no_xsdn:
	return rc;
}

void stmvl53l1_release_gpios(struct i2c_data *i2c_data)
{
	put_xsdn(i2c_data);
	if (i2c_data->vdd) {
		regulator_put(i2c_data->vdd);
		i2c_data->vdd = NULL;
	}
	put_pwren(i2c_data);
	put_intr(i2c_data);
}

static int stmvl53l1_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int rc = 0;
	struct stmvl53l1_data *vl53l1_data = NULL;
	struct i2c_data *i2c_data = NULL;

	vl53l1_dbgmsg("Enter %s : 0x%02x\n", client->name, client->addr);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE)) {
		rc = -EIO;
		return rc;
	}

	vl53l1_data = kzalloc(sizeof(struct stmvl53l1_data), GFP_KERNEL);
	if (!vl53l1_data) {
		rc = -ENOMEM;
		return rc;
	}
	if (vl53l1_data) {
		vl53l1_data->client_object =
				kzalloc(sizeof(struct i2c_data), GFP_KERNEL);
		if (!vl53l1_data)
			goto done_freemem;
		i2c_data = (struct i2c_data *)vl53l1_data->client_object;
	}
	i2c_data->client = client;
	i2c_data->vl53l1_data = vl53l1_data;
	i2c_data->irq = -1 ; /* init to no irq */

	/* parse and configure hardware */
	rc = stmvl53l1_parse_tree(&i2c_data->client->dev, i2c_data);
	if (rc)
		goto done_freemem;

	/* setup device name */
	/* vl53l1_data->dev_name = dev_name(&client->dev); */

	/* setup client data */
	i2c_set_clientdata(client, vl53l1_data);

	/* end up by core driver setup */
	rc = stmvl53l1_setup(vl53l1_data);
	if (rc)
		goto release_gpios;
	vl53l1_dbgmsg("End\n");

	kref_init(&i2c_data->ref);

	return rc;

release_gpios:
	stmvl53l1_release_gpios(i2c_data);

done_freemem:
	/* kfree safe against NULL */
	kfree(vl53l1_data);
	kfree(i2c_data);

	return -1;
}

static int stmvl53l1_remove(struct i2c_client *client)
{
	struct stmvl53l1_data *data = i2c_get_clientdata(client);
	struct i2c_data *i2c_data = (struct i2c_data *)data->client_object;

	vl53l1_dbgmsg("Enter\n");
	mutex_lock(&data->work_mutex);
	/* main driver cleanup */
	stmvl53l1_cleanup(data);

	/* release gpios */
	stmvl53l1_release_gpios(i2c_data);

	mutex_unlock(&data->work_mutex);

	stmvl53l1_put(data->client_object);

	vl53l1_dbgmsg("End\n");

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int stmvl53l1_suspend(struct device *dev)
{
	struct stmvl53l1_data *data = i2c_get_clientdata(to_i2c_client(dev));

	vl53l1_dbgmsg("Enter\n");
	mutex_lock(&data->work_mutex);
	/* Stop ranging */
	stmvl53l1_pm_suspend_stop(data);

	mutex_unlock(&data->work_mutex);

	vl53l1_dbgmsg("End\n");

	return 0;
}

static int stmvl53l1_resume(struct device *dev)
{
#if 0
	struct stmvl53l1_data *data = i2c_get_clientdata(to_i2c_client(dev));

	vl53l1_dbgmsg("Enter\n");

	mutex_lock(&data->work_mutex);

	/* do nothing user will restart measurements */

	mutex_unlock(&data->work_mutex);

	vl53l1_dbgmsg("End\n");
#else
	vl53l1_dbgmsg("Enter\n");
	vl53l1_dbgmsg("End\n");
#endif
	return 0;
}
#endif


static SIMPLE_DEV_PM_OPS(stmvl53l1_pm_ops, stmvl53l1_suspend, stmvl53l1_resume);

static const struct i2c_device_id stmvl53l1_id[] = {
	{ STMVL53L1_DRV_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, stmvl53l1_id);

static const struct of_device_id st_stmvl53l1_dt_match[] = {
	{ .compatible = "st,"STMVL53L1_DRV_NAME, },
	{ },
};

static struct i2c_driver stmvl53l1_driver = {
	.driver = {
		.name	= STMVL53L1_DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = st_stmvl53l1_dt_match,
		.pm	= &stmvl53l1_pm_ops,
	},
	.probe	= stmvl53l1_probe,
	.remove	= stmvl53l1_remove,
	.id_table = stmvl53l1_id,

};

/**
 * give power to device
 *
 * @param object  the i2c layer object
 * @return
 */
int stmvl53l1_power_up_i2c(void *object)
{
	int rc = 0;
	struct i2c_data *data = (struct i2c_data *) object;

	vl53l1_dbgmsg("Enter\n");

	/* turn on power */
	if (data->vdd) {
		rc = regulator_enable(data->vdd);
		if (rc) {
			vl53l1_errmsg("fail to turn on regulator");
			return rc;
		}
	}
    if (data->pwren_gpio != -1) {
		gpio_set_value(data->pwren_gpio, 1);
		vl53l1_info("slow power on");
	} else
		vl53l1_wanrmsg("no power control");

	return rc;
}

/**
 * remove power to device (reset it)
 *
 * @param i2c_object the i2c layer object
 * @return 0 on success
 */
int stmvl53l1_power_down_i2c(void *i2c_object)
{
	struct i2c_data *data = (struct i2c_data *) i2c_object;
	int rc = 0;

	vl53l1_dbgmsg("Enter\n");

	/* turn off power */
	if (data->vdd) {
		rc = regulator_disable(data->vdd);
		if (rc)
			vl53l1_errmsg("reg disable failed. rc=%d\n",
				rc);
	}
    if (data->pwren_gpio != -1) {
		gpio_set_value(data->pwren_gpio, 0);
	}
	vl53l1_dbgmsg("power off");

	vl53l1_dbgmsg("End\n");

	return rc;
}

static int handle_i2c_address_device_change_lock(struct i2c_data *data)
{
	return 0;
}

/* reset release will also handle device address change. It will avoid state
 * where multiple stm53l1 are bring out of reset at the same time with the
 * same boot address.
 * Note that we don't manage case where boot_reg has the same value as a final
 * i2c address of another device. This case is not supported and will lead
 * to unpredictable behavior.
 */
static int release_reset(struct i2c_data *data)
{
	//struct i2c_client *client = (struct i2c_client *) data->client;
	int rc = 0;
	bool is_address_change = false;//client->addr != data->boot_reg;//songyt modify

	if (is_address_change)
		mutex_lock(&dev_addr_change_mutex);
    //vl53l1_dbgmsg("reset xsdn pin to 1\n");
	gpio_set_value(data->xsdn_gpio, 1);
	if (is_address_change) {
		rc = handle_i2c_address_device_change_lock(data);
		if (rc){
			gpio_set_value(data->xsdn_gpio, 0);
            //vl53l1_dbgmsg("reset xsdn pin to 0\n");
        }
	}

	if (is_address_change)
		mutex_unlock(&dev_addr_change_mutex);

	return rc;
}

/**
 * release device reset
 *
 * @param i2c_object the i2c layer object
 * @return 0 on success
 */
int stmvl53l1_reset_release_i2c(void *i2c_object)
{
	int rc;
	struct i2c_data *data = (struct i2c_data *) i2c_object;

	vl53l1_dbgmsg("Enter\n");

	rc = release_reset(data);
	if (rc)
		goto error;

	/* and now wait for device end of boot */
	data->vl53l1_data->is_delay_allowed = true;
	rc = VL53L1_WaitDeviceBooted(&data->vl53l1_data->stdev);
	data->vl53l1_data->is_delay_allowed = false;
	if (rc) {
		gpio_set_value(data->xsdn_gpio, 0);
		vl53l1_errmsg("boot fail with error %d,change xsdn pin to 0", rc);
		data->vl53l1_data->last_error = rc;
		rc = -EIO;
	}

error:
	vl53l1_dbgmsg("End\n");

	return rc;
}

/**
 * put device under reset
 *
 * @param i2c_object the i2c layer object
 * @return 0 on success
 */
int stmvl53l1_reset_hold_i2c(void *i2c_object)
{
	struct i2c_data *data = (struct i2c_data *) i2c_object;

	vl53l1_dbgmsg("Enter\n");

	gpio_set_value(data->xsdn_gpio, 0);

	vl53l1_dbgmsg("End\n");

	return 0;
}

int stmvl53l1_init_i2c(void)
{
	int ret = 0;

	vl53l1_dbgmsg("Enter\n");

	/* register as a i2c client device */
	ret = i2c_add_driver(&stmvl53l1_driver);
	if (ret)
		vl53l1_errmsg("%d erro ret:%d\n", __LINE__, ret);

	if (!ret && force_device)
		ret = insert_device();

	if (ret)
		i2c_del_driver(&stmvl53l1_driver);

	vl53l1_dbgmsg("End with rc:%d\n", ret);

	return ret;
}


void stmvl53l1_clean_up_i2c(void)
{
	if (stm_test_i2c_client) {
		vl53l1_dbgmsg("to unregister i2c client\n");
		i2c_unregister_device(stm_test_i2c_client);
	}
}

static irqreturn_t stmvl53l1_irq_handler_i2c(int vec, void *info)
{
	struct i2c_data *i2c_data = (struct i2c_data *)info;

	if (i2c_data->irq == vec) {
		modi2c_dbg("irq");
		stmvl53l1_intr_handler(i2c_data->vl53l1_data);
		modi2c_dbg("over");
	} else {
		if (!i2c_data->msg_flag.unhandled_irq_vec) {
			modi2c_warn("unmatching vec %d != %d\n",
					vec, i2c_data->irq);
			i2c_data->msg_flag.unhandled_irq_vec = 1;
		}
	}

	return IRQ_HANDLED;
}

/**
 * enable and start intr handling
 *
 * @param object  our i2c_data specific object
 * @param poll_mode [in/out] set to force mode clear to use irq
 * @return 0 on success and set ->poll_mode if it faill ranging wan't start
 */
int stmvl53l1_start_intr(void *object, int *poll_mode)
{
	struct i2c_data *i2c_data;
	int rc;

	i2c_data = (struct i2c_data *)object;
	/* irq and gpio acquire config done in parse_tree */
	if (i2c_data->irq < 0) {
		/* the i2c tree as no intr force polling mode */
		*poll_mode = -1;
		return 0;
	}
	/* clear irq warning report enabe it again for this session */
	i2c_data->msg_flag.unhandled_irq_vec = 0;
	/* if started do no nothing */
	if (i2c_data->io_flag.intr_started) {
		/* nothing to do */
		*poll_mode = 0;
		return 0;
	}

	vl53l1_dbgmsg("to register_irq:%d\n", i2c_data->irq);
	rc = request_threaded_irq(i2c_data->irq, NULL,
			stmvl53l1_irq_handler_i2c,
			IRQF_TRIGGER_FALLING|IRQF_ONESHOT,
			"vl53l1_interrupt",
			(void *)i2c_data);
	if (rc) {
		vl53l1_errmsg("fail to req threaded irq rc=%d\n", rc);
		*poll_mode = 0;
	} else {
		vl53l1_dbgmsg("irq %d now handled\n", i2c_data->irq);
		i2c_data->io_flag.intr_started = 1;
		*poll_mode = 0;
	}
	return rc;
}

void *stmvl53l1_get(void *object)
{
	struct i2c_data *data = (struct i2c_data *) object;

	vl53l1_dbgmsg("Enter\n");
	kref_get(&data->ref);
	vl53l1_dbgmsg("End\n");

	return object;
}

static void memory_release(struct kref *kref)
{
	struct i2c_data *data = container_of(kref, struct i2c_data, ref);

	vl53l1_dbgmsg("Enter\n");
	kfree(data->vl53l1_data);
	kfree(data);
	vl53l1_dbgmsg("End\n");
}

void stmvl53l1_put(void *object)
{
	struct i2c_data *data = (struct i2c_data *) object;

	vl53l1_dbgmsg("Enter\n");
	kref_put(&data->ref, memory_release);
	vl53l1_dbgmsg("End\n");
}

void __exit stmvl53l1_exit_i2c(void *i2c_object)
{
	vl53l1_dbgmsg("Enter\n");
	i2c_del_driver(&stmvl53l1_driver);
	vl53l1_dbgmsg("End\n");
}
