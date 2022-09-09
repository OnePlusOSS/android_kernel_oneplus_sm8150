// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2021 Oplus. All rights reserved.
 * File: oplus_tri_key.c
 *
 * Description:
 *      Mechanical tri_state_key driver code.
 *
 * Version: 1.0
 */

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
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>

#define DRV_NAME	"tri_state_key"
#define TRI_KEY_DEVICE "oplus,tri-state-key"

#define TRI_KEY_TAG    "[tri_state_key] "
#define TRI_KEY_ERR(fmt, args...)\
	pr_info(TRI_KEY_TAG" %s : "fmt, __func__, ##args)
#define TRI_KEY_LOG(fmt, args...)\
	pr_info(TRI_KEY_TAG" %s : "fmt, __func__, ##args)
#define TRI_KEY_DEBUG(fmt, args...)\
	do {\
		if (tri_key_debug == LEVEL_DEBUG)\
			pr_info(TRI_KEY_TAG " %s: " fmt, __func__, ##args);\
	} while (0)

/*
 *	    KEY1(GPIO10)	KEY2(GPIO83)  KEY3(GPIO69)
 *		1	            1         	  0				| Silent
 *		1	            0         	  1				| Vibrate
 *		0	            1         	  1				| Ring
 */

struct trikey_dev_data {
	int irq_key3;
	int irq_key2;
	int irq_key1;
	int key1_gpio;
	int key2_gpio;
	int key3_gpio;
	short state;

	struct regulator *vdd_io;

	struct work_struct work;
	struct device *dev;
	struct input_dev *input_dev;

	struct timer_list s_timer;
	struct pinctrl *key_pinctrl;
	struct pinctrl_state *set_state;
};

static struct trikey_dev_data *trikey_data;

static int set_gpio_by_pinctrl(void)
{
	return pinctrl_select_state(trikey_data->key_pinctrl,
		trikey_data->set_state);
}
/*op add to fix GCE-7551 begin
extern int aw8697_op_haptic_stop(void);
op add to fix GCE-7551 end*/

static void tri_key_dev_work(struct work_struct *work)
{
	int key[3] = {0, 0, 0};
	/*op add to fix ISTRACKING-34823 begin*/
	static int pre_key0, pre_key1, pre_key2;
	/*op add to fix ISTRACKING-34823 end*/

	key[0] = gpio_get_value(trikey_data->key1_gpio);
	key[1] = gpio_get_value(trikey_data->key2_gpio);
	key[2] = gpio_get_value(trikey_data->key3_gpio);
	TRI_KEY_LOG("key[0]=%d,key[1]=%d,key[2]=%d\n",
		key[0], key[1], key[2]);
	/*op add to fix ISTRACKING-34823 begin*/
	if (!key[0] || !key[1] || !key[2]) {
		if (pre_key0 == key[0] && pre_key1 == key[1]
				&& pre_key2 == key[2]) {
			pre_key0 = key[0];
			pre_key1 = key[1];
			pre_key2 = key[2];
			return;
		}
	}
	/*op add to fix ISTRACKING-34823 end*/
	/*op add to fix GCE-7551 begin*/
	if (key[0] && key[1] && key[2])
		return;
	if (!key[0] && !key[1] && !key[2])
		return;
	if (!key[0] && !key[1] && key[2])
		return;
	if (!key[0] && key[1] && !key[2])
		return;
	if (key[0] && !key[1] && !key[2])
		return;
	/*op add to fix GCE-7551 end*/
	if (key[0] == 1 && key[1] == 1 && key[2] == 0) {
		trikey_data->state = 1;
		input_report_key(trikey_data->input_dev, KEY_F3, 1);
		input_sync(trikey_data->input_dev);
		input_report_key(trikey_data->input_dev, KEY_F3, 0);
		input_sync(trikey_data->input_dev);
		TRI_KEY_LOG("report up key successful!\n");
	}

	else if (key[0] == 1 && key[1] == 0 && key[2] == 1) {
		trikey_data->state = 2;
		input_report_key(trikey_data->input_dev, KEY_F3, 2);
		input_sync(trikey_data->input_dev);
		input_report_key(trikey_data->input_dev, KEY_F3, 0);
		input_sync(trikey_data->input_dev);
		TRI_KEY_LOG("report mid key successful!\n");
	}

	else if (key[0] == 0 && key[1] == 1 && key[2] == 1) {
		trikey_data->state = 3;
		input_report_key(trikey_data->input_dev, KEY_F3, 3);
		input_sync(trikey_data->input_dev);
		input_report_key(trikey_data->input_dev, KEY_F3, 0);
		input_sync(trikey_data->input_dev);
		TRI_KEY_LOG("report down key successful!\n");
	} else
		TRI_KEY_LOG("not report any key!\n");

	/*op add to fix GCE-7551 begin
	if (!key[2] ||  !key[1])
		aw8697_op_haptic_stop();
	op add to fix GCE-7551 end*/
	/*op add to fix ISTRACKING-34823 begin*/
	if (!key[0] || !key[1] || !key[2]) {
		pre_key0 = key[0];
		pre_key1 = key[1];
		pre_key2 = key[2];
	}
	/*op add to fix ISTRACKING-34823 end*/
}


static irqreturn_t tri_key_dev_interrupt(int irq, void *_dev)
{
	schedule_work(&trikey_data->work);
	return IRQ_HANDLED;
}

static void timer_handle(unsigned long arg)
{
	schedule_work(&trikey_data->work);
}

#ifdef CONFIG_OF
static int dev_get_devtree_pdata(struct device *dev)
{
	struct device_node *node;

	node = dev->of_node;
	if (!node)
		return -EINVAL;

	trikey_data->key3_gpio =
	of_get_named_gpio(node, "tristate,gpio_key3", 0);
	if ((!gpio_is_valid(trikey_data->key3_gpio)))
		return -EINVAL;
	TRI_KEY_LOG("trikey_data->key3_gpio=%d\n", trikey_data->key3_gpio);

	trikey_data->key2_gpio =
		of_get_named_gpio(node, "tristate,gpio_key2", 0);
	if ((!gpio_is_valid(trikey_data->key2_gpio)))
		return -EINVAL;
	TRI_KEY_LOG("trikey_data->key2_gpio=%d\n", trikey_data->key2_gpio);

	trikey_data->key1_gpio =
		of_get_named_gpio(node, "tristate,gpio_key1", 0);
	if ((!gpio_is_valid(trikey_data->key1_gpio)))
		return -EINVAL;
	TRI_KEY_LOG("trikey_data->key1_gpio=%d\n", trikey_data->key1_gpio);

	return 0;
}
#else
static inline int dev_get_devtree_pdata(struct device *dev)
{
	KEY_LOG("inline function\n");
	return 0;
}
#endif

static ssize_t proc_tri_state_read(struct file *file, char __user *user_buf,
		size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[6] = {0};
	TRI_KEY_LOG("Called\n");
	snprintf(page, sizeof(page), "%d\n", trikey_data->state);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static const struct file_operations proc_tri_state_ops = {
	.read  = proc_tri_state_read,
	.open  = simple_open,
	.owner = THIS_MODULE,
};

static int init_trikey_proc(struct trikey_dev_data *trikey_data)
{
	int ret = 0;
	struct proc_dir_entry *prEntry_trikey = NULL;
	struct proc_dir_entry *prEntry_tmp = NULL;
	TRI_KEY_LOG("entry\n");

	prEntry_trikey = proc_mkdir("tristatekey", NULL);
	if (prEntry_trikey == NULL) {
		ret = -ENOMEM;
		TRI_KEY_ERR("Couldn't create trikey proc entry\n");
	}

	prEntry_tmp = proc_create("tri_state", 0644, prEntry_trikey, &proc_tri_state_ops);
	if (prEntry_tmp == NULL) {
		ret = -ENOMEM;
		TRI_KEY_ERR("Couldn't create proc entry, %d\n");
	}

	return ret;
}

static int tri_key_dev_probe(struct platform_device *pdev)
{
	struct device *dev;
	int ret = 0;

	TRI_KEY_LOG("called\n");

	dev = &pdev->dev;

	trikey_data = kzalloc(sizeof(struct trikey_dev_data), GFP_KERNEL);
	if (!trikey_data) {
		TRI_KEY_ERR("kzalloc err\n");
		return -ENOMEM;
	}

	trikey_data->dev = dev;

	trikey_data->key_pinctrl = devm_pinctrl_get(trikey_data->dev);

	if (IS_ERR_OR_NULL(trikey_data->key_pinctrl)) {
		TRI_KEY_ERR("Failed to get pinctrl\n");
		goto fail;
	}
	trikey_data->set_state = pinctrl_lookup_state(trikey_data->key_pinctrl,
		"pmx_tri_state_key_active");

	if (IS_ERR_OR_NULL(trikey_data->set_state)) {
		TRI_KEY_ERR("Failed to lookup_state\n");
		goto fail;
	}

	set_gpio_by_pinctrl();

	ret = dev_get_devtree_pdata(dev);
	if (ret) {
		TRI_KEY_ERR("parse device tree fail!!!\n");
		goto fail;
	}

	/* input registration */
	trikey_data->input_dev = input_allocate_device();
	if (trikey_data->input_dev == NULL) {
		ret = -ENOMEM;
		TRI_KEY_ERR("Failed to allocate input device\n");
		goto fail;
	}
	trikey_data->input_dev->name = TRI_KEY_DEVICE;

	set_bit(EV_SYN, trikey_data->input_dev->evbit);
	set_bit(EV_KEY, trikey_data->input_dev->evbit);
	set_bit(KEY_F3, trikey_data->input_dev->keybit);

	ret = input_register_device(trikey_data->input_dev);
	if (ret) {
		TRI_KEY_ERR("Failed to register input device\n");
		input_free_device(trikey_data->input_dev);
		goto fail;
	}

	ret = init_trikey_proc(trikey_data);
	if (ret < 0) {
		TRI_KEY_ERR("create trikey proc fail\n");
		goto proc_fail;
	}

	/*config irq gpio and request irq */
	ret = gpio_request(trikey_data->key1_gpio, "tristate_key1");
	if (ret < 0)
		goto err_request_gpio;

	ret = gpio_direction_input(trikey_data->key1_gpio);
	if (ret < 0)
		goto err_set_gpio_input;

	trikey_data->irq_key1 = gpio_to_irq(trikey_data->key1_gpio);
	if (trikey_data->irq_key1 < 0) {
		ret = trikey_data->irq_key1;
		goto err_detect_irq_num_failed;
	}

	ret = request_irq(trikey_data->irq_key1, tri_key_dev_interrupt,
		IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING,
		"tristate_key1", trikey_data);
	if (ret < 0)
		goto err_request_irq;

	ret = gpio_request(trikey_data->key2_gpio,
		"tristate_key2");
	if (ret < 0)
		goto err_request_gpio;

	ret = gpio_direction_input(trikey_data->key2_gpio);
	if (ret < 0)
		goto err_set_gpio_input;

	trikey_data->irq_key2 = gpio_to_irq(trikey_data->key2_gpio);
	if (trikey_data->irq_key2 < 0) {
		ret = trikey_data->irq_key2;
		goto err_detect_irq_num_failed;
	}

	ret = request_irq(trikey_data->irq_key2, tri_key_dev_interrupt,
		IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING,
		"tristate_key2", trikey_data);
	if (ret < 0)
		goto err_request_irq;

	ret = gpio_request(trikey_data->key3_gpio,
		"tristate_key3");
	if (ret < 0)
		goto err_request_gpio;

	ret = gpio_direction_input(trikey_data->key3_gpio);
	if (ret < 0)
		goto err_set_gpio_input;

	trikey_data->irq_key3 = gpio_to_irq(trikey_data->key3_gpio);
	if (trikey_data->irq_key3 < 0) {
		ret = trikey_data->irq_key3;
		goto err_detect_irq_num_failed;
	}

	ret = request_irq(trikey_data->irq_key3, tri_key_dev_interrupt,
		IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING,
		"tristate_key3", trikey_data);
	if (ret < 0)
		goto err_request_irq;

	INIT_WORK(&trikey_data->work, tri_key_dev_work);

	trikey_data->s_timer.function = &timer_handle;
	trikey_data->s_timer.expires = jiffies + 5*HZ;

	add_timer(&trikey_data->s_timer);

	enable_irq_wake(trikey_data->irq_key1);
	enable_irq_wake(trikey_data->irq_key2);
	enable_irq_wake(trikey_data->irq_key3);

	return 0;

err_request_gpio:
err_request_irq:
err_detect_irq_num_failed:
err_set_gpio_input:
	gpio_free(trikey_data->key2_gpio);
	gpio_free(trikey_data->key1_gpio);
	gpio_free(trikey_data->key3_gpio);
proc_fail:
	input_unregister_device(trikey_data->input_dev);
fail:
	kfree(trikey_data);

	return ret;
}

static int tri_key_dev_remove(struct platform_device *pdev)
{
	cancel_work_sync(&trikey_data->work);
	gpio_free(trikey_data->key1_gpio);
	gpio_free(trikey_data->key2_gpio);
	gpio_free(trikey_data->key3_gpio);
	input_unregister_device(trikey_data->input_dev);
	kfree(trikey_data);

	return 0;
}

static const struct of_device_id tri_key_dev_of_match[] = {
	{ .compatible = "oplus, tri-state-key", },
	{ },
};
MODULE_DEVICE_TABLE(of, tri_key_dev_of_match);

static struct platform_driver tristate_dev_driver = {
	.probe		= tri_key_dev_probe,
	.remove		= tri_key_dev_remove,
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = tri_key_dev_of_match,
	},
};

static int __init tri_state_key_init(void)
{
	return platform_driver_register(&tristate_dev_driver);
}
module_init(tri_state_key_init);

static void __exit tri_state_key_exit(void)
{
	platform_driver_unregister(&tristate_dev_driver);
}
module_exit(tri_state_key_exit);
MODULE_DESCRIPTION("mechanical tri_state_key driver");
MODULE_LICENSE("GPL v2");


