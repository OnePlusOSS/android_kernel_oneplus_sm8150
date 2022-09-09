/* Copyright (c) 2017-2018, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#ifdef VENDOR_EDIT
#include <linux/proc_fs.h>
#include <linux/time.h>
#include <linux/rtc.h>
struct cam_flash_ctrl *vendor_flash_ctrl = NULL;
struct cam_flash_ctrl *front_flash_ctrl = NULL;
#endif
#include "cam_flash_dev.h"
#include "cam_flash_soc.h"
#include "cam_flash_core.h"
#include "cam_common_util.h"
#ifdef VENDOR_EDIT
#include "cam_res_mgr_api.h"
#endif

static int32_t cam_flash_driver_cmd(struct cam_flash_ctrl *fctrl,
		void *arg, struct cam_flash_private_soc *soc_private)
{
	int rc = 0;
	int i = 0;
	struct cam_control *cmd = (struct cam_control *)arg;

	if (!fctrl || !arg) {
		CAM_ERR(CAM_FLASH, "fctrl/arg is NULL with arg:%pK fctrl%pK",
			fctrl, arg);
		return -EINVAL;
	}

	if (cmd->handle_type != CAM_HANDLE_USER_POINTER) {
		CAM_ERR(CAM_FLASH, "Invalid handle type: %d",
			cmd->handle_type);
		return -EINVAL;
	}

	mutex_lock(&(fctrl->flash_mutex));
	switch (cmd->op_code) {
	case CAM_ACQUIRE_DEV: {
		struct cam_sensor_acquire_dev flash_acq_dev;
		struct cam_create_dev_hdl bridge_params;

		CAM_DBG(CAM_FLASH, "CAM_ACQUIRE_DEV");

		if (fctrl->flash_state != CAM_FLASH_STATE_INIT) {
			CAM_ERR(CAM_FLASH,
				"Cannot apply Acquire dev: Prev state: %d",
				fctrl->flash_state);
			rc = -EINVAL;
			goto release_mutex;
		}

		if (fctrl->bridge_intf.device_hdl != -1) {
			CAM_ERR(CAM_FLASH, "Device is already acquired");
			rc = -EINVAL;
			goto release_mutex;
		}

		rc = copy_from_user(&flash_acq_dev,
			u64_to_user_ptr(cmd->handle),
			sizeof(flash_acq_dev));
		if (rc) {
			CAM_ERR(CAM_FLASH, "Failed Copying from User");
			goto release_mutex;
		}

		bridge_params.session_hdl = flash_acq_dev.session_handle;
		bridge_params.ops = &fctrl->bridge_intf.ops;
		bridge_params.v4l2_sub_dev_flag = 0;
		bridge_params.media_entity_flag = 0;
		bridge_params.priv = fctrl;
		bridge_params.dev_id = CAM_FLASH;
		flash_acq_dev.device_handle =
			cam_create_device_hdl(&bridge_params);
		fctrl->bridge_intf.device_hdl =
			flash_acq_dev.device_handle;
		fctrl->bridge_intf.session_hdl =
			flash_acq_dev.session_handle;

		rc = copy_to_user(u64_to_user_ptr(cmd->handle),
			&flash_acq_dev,
			sizeof(struct cam_sensor_acquire_dev));
		if (rc) {
			CAM_ERR(CAM_FLASH, "Failed Copy to User with rc = %d",
				rc);
			rc = -EFAULT;
			goto release_mutex;
		}
		fctrl->flash_state = CAM_FLASH_STATE_ACQUIRE;
		break;
	}
	case CAM_RELEASE_DEV: {
		CAM_DBG(CAM_FLASH, "CAM_RELEASE_DEV");
		if ((fctrl->flash_state == CAM_FLASH_STATE_INIT) ||
			(fctrl->flash_state == CAM_FLASH_STATE_START)) {
			CAM_WARN(CAM_FLASH,
				"Wrong state for Release dev: Prev state:%d",
				fctrl->flash_state);
		}

		if (fctrl->bridge_intf.device_hdl == -1 &&
			fctrl->flash_state == CAM_FLASH_STATE_ACQUIRE) {
			CAM_ERR(CAM_FLASH,
				"Invalid Handle: Link Hdl: %d device hdl: %d",
				fctrl->bridge_intf.device_hdl,
				fctrl->bridge_intf.link_hdl);
			rc = -EINVAL;
			goto release_mutex;
		}

		if (fctrl->bridge_intf.link_hdl != -1) {
			CAM_ERR(CAM_SENSOR,
				"Device [%d] still active on link 0x%x",
				fctrl->flash_state,
				fctrl->bridge_intf.link_hdl);
			rc = -EAGAIN;
			goto release_mutex;
		}

		if ((fctrl->flash_state == CAM_FLASH_STATE_CONFIG) ||
			(fctrl->flash_state == CAM_FLASH_STATE_START))
			fctrl->func_tbl.flush_req(fctrl, FLUSH_ALL, 0);

		if (cam_flash_release_dev(fctrl))
			CAM_WARN(CAM_FLASH,
				"Failed in destroying the device Handle");

		if (fctrl->func_tbl.power_ops(fctrl, false))
			CAM_WARN(CAM_FLASH, "Power Down Failed");

		fctrl->flash_state = CAM_FLASH_STATE_INIT;
		break;
	}
	case CAM_QUERY_CAP: {
		struct cam_flash_query_cap_info flash_cap = {0};

		CAM_DBG(CAM_FLASH, "CAM_QUERY_CAP");
		flash_cap.slot_info = fctrl->soc_info.index;
		for (i = 0; i < fctrl->flash_num_sources; i++) {
			flash_cap.max_current_flash[i] =
				soc_private->flash_max_current[i];
			flash_cap.max_duration_flash[i] =
				soc_private->flash_max_duration[i];
		}

		for (i = 0; i < fctrl->torch_num_sources; i++)
			flash_cap.max_current_torch[i] =
				soc_private->torch_max_current[i];

		if (copy_to_user(u64_to_user_ptr(cmd->handle),
			&flash_cap, sizeof(struct cam_flash_query_cap_info))) {
			CAM_ERR(CAM_FLASH, "Failed Copy to User");
			rc = -EFAULT;
			goto release_mutex;
		}
		break;
	}
	case CAM_START_DEV: {
		CAM_DBG(CAM_FLASH, "CAM_START_DEV");
		if ((fctrl->flash_state == CAM_FLASH_STATE_INIT) ||
			(fctrl->flash_state == CAM_FLASH_STATE_START)) {
			CAM_WARN(CAM_FLASH,
				"Cannot apply Start Dev: Prev state: %d",
				fctrl->flash_state);
			rc = -EINVAL;
			goto release_mutex;
		}

		fctrl->flash_state = CAM_FLASH_STATE_START;
		break;
	}
	case CAM_STOP_DEV: {
		CAM_DBG(CAM_FLASH, "CAM_STOP_DEV ENTER");
		if (fctrl->flash_state != CAM_FLASH_STATE_START) {
			CAM_WARN(CAM_FLASH,
				"Cannot apply Stop dev: Prev state is: %d",
				fctrl->flash_state);
			rc = -EINVAL;
			goto release_mutex;
		}

		cam_flash_off(fctrl);
		fctrl->func_tbl.flush_req(fctrl, FLUSH_ALL, 0);
		fctrl->last_flush_req = 0;
		fctrl->flash_state = CAM_FLASH_STATE_ACQUIRE;
		break;
	}
	case CAM_CONFIG_DEV: {
		CAM_DBG(CAM_FLASH, "CAM_CONFIG_DEV");
		rc = fctrl->func_tbl.parser(fctrl, arg);
		if (rc) {
			CAM_ERR(CAM_FLASH, "Failed Flash Config: rc=%d\n", rc);
			goto release_mutex;
		}
		break;
	}
	default:
		CAM_ERR(CAM_FLASH, "Invalid Opcode: %d", cmd->op_code);
		rc = -EINVAL;
	}

release_mutex:
	mutex_unlock(&(fctrl->flash_mutex));
	return rc;
}

#ifdef VENDOR_EDIT
volatile static int flash_mode;
volatile static int pre_flash_mode;
static ssize_t flash_on_off(struct cam_flash_ctrl *flash_ctrl)
{
	int rc = 1;
	struct timespec ts;
	struct rtc_time tm;
	struct cam_flash_frame_setting flash_data;
	memset(&flash_data, 0, sizeof(flash_data));

	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);
	pr_info("flash_mode %d,%d-%02d-%02d %02d:%02d:%02d.%09lu UTC\n",
		flash_mode,
		tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
		tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
	if(pre_flash_mode == flash_mode)
		return rc;

	if(pre_flash_mode == 5 && flash_mode == 0){
		pr_err("camera is opened,not to set flashlight off");
		return rc;
	}

	pre_flash_mode = flash_mode;

	switch (flash_mode)
	{
		case 0:
			flash_data.led_current_ma[0] = 0;
			flash_data.led_current_ma[1] = 0;
			cam_flash_off(flash_ctrl);
			flash_ctrl->flash_state = CAM_FLASH_STATE_INIT;
			break;
		case 1:
			flash_data.led_current_ma[0] = 55;
			flash_data.led_current_ma[1] = 55;
			cam_flash_on(flash_ctrl, &flash_data, 0);
			break;
		case 2:
			flash_data.led_current_ma[0] = 1000;
			flash_data.led_current_ma[1] = 1000;
			cam_flash_on(flash_ctrl, &flash_data, 1);
			break;
		case 3:
			flash_data.led_current_ma[0] = 60;
			flash_data.led_current_ma[1] = 60;
			cam_flash_on(flash_ctrl, &flash_data, 0);
			break;
		default:
			break;
	}
	return rc;
}

static ssize_t flash_proc_write(struct file *filp, const char __user *buff,
                        	size_t len, loff_t *data)
{
	char buf[8] = {0};
	int rc = 0;
	if (len > 8)
		len = 8;
	if (copy_from_user(buf, buff, len)) {
		pr_err("proc write error.\n");
		return -EFAULT;
	}
	flash_mode = simple_strtoul(buf, NULL, 10);
	rc = flash_on_off(vendor_flash_ctrl);
	if(rc < 0)
		pr_err("%s flash write failed %d\n", __func__, __LINE__);
	return len;
}
static ssize_t flash_proc_read(struct file *filp, char __user *buff,
                        	size_t len, loff_t *data)
{
	char value[2] = {0};
	snprintf(value, sizeof(value), "%d", flash_mode);
	return simple_read_from_buffer(buff, len, data, value,1);
}

static const struct file_operations led_fops = {
    .owner		= THIS_MODULE,
    .read		= flash_proc_read,
    .write		= flash_proc_write,
};
static int flash_proc_init(struct cam_flash_ctrl *flash_ctl)
{
	int ret = 0;
	char proc_flash[16] = "qcom_flash";
	char strtmp[] = "0";
	struct proc_dir_entry *proc_entry;
	if (flash_ctl->flash_name == NULL) {
		pr_err("%s get flash name is NULL %d\n", __func__, __LINE__);
		return -1;
	} else {
		if (strcmp(flash_ctl->flash_name, "pmic") != 0) {
			pr_err("%s get flash name is PMIC ,so return\n", __func__);
			return -1;
		}
	}
	if (flash_ctl->soc_info.index > 0) {
		sprintf(strtmp, "%d", flash_ctl->soc_info.index);
		strcat(proc_flash, strtmp);
	}
	proc_entry = proc_create_data(proc_flash, 0666, NULL,&led_fops, NULL);
	if (proc_entry == NULL) {
		ret = -ENOMEM;
		pr_err("[%s]: Error! Couldn't create qcom_flash proc entry\n", __func__);
	}
	vendor_flash_ctrl = flash_ctl;
	return ret;
}

static ssize_t cam_flash_switch_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	int rc = 0;
	struct cam_flash_ctrl *data = dev_get_drvdata(dev);

	int enable = 0;

	if (kstrtoint(buf, 0, &enable)) {
		pr_err("get val error.\n");
		rc = -EINVAL;
	}
	CAM_ERR(CAM_FLASH, "echo data = %d ", enable);

	flash_mode = enable;
	rc = flash_on_off(data);

	return count;
}

static ssize_t cam_flash_switch_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 5, "%d\n", flash_mode);
	//return simple_read_from_buffer(buff, 10, buf, value,1);
}

static DEVICE_ATTR(fswitch, 0660, cam_flash_switch_show,cam_flash_switch_store);
#endif

static int32_t cam_flash_init_default_params(struct cam_flash_ctrl *fctrl)
{
	/* Validate input parameters */
	if (!fctrl) {
		CAM_ERR(CAM_FLASH, "failed: invalid params fctrl %pK",
			fctrl);
		return -EINVAL;
	}

	CAM_DBG(CAM_FLASH,
		"master_type: %d", fctrl->io_master_info.master_type);
	/* Initialize cci_client */
	if (fctrl->io_master_info.master_type == CCI_MASTER) {
		fctrl->io_master_info.cci_client = kzalloc(sizeof(
			struct cam_sensor_cci_client), GFP_KERNEL);
		if (!(fctrl->io_master_info.cci_client))
			return -ENOMEM;
	} else if (fctrl->io_master_info.master_type == I2C_MASTER) {
		if (!(fctrl->io_master_info.client))
			return -EINVAL;
	} else {
		CAM_ERR(CAM_FLASH,
			"Invalid master / Master type Not supported");
		return -EINVAL;
	}

	return 0;
}

static const struct of_device_id cam_flash_dt_match[] = {
	{.compatible = "qcom,camera-flash", .data = NULL},
	{}
};

static long cam_flash_subdev_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd, void *arg)
{
	int rc = 0;
	struct cam_flash_ctrl *fctrl = NULL;
	struct cam_flash_private_soc *soc_private = NULL;

	CAM_DBG(CAM_FLASH, "Enter");

	fctrl = v4l2_get_subdevdata(sd);
	soc_private = fctrl->soc_info.soc_private;

	switch (cmd) {
	case VIDIOC_CAM_CONTROL: {
		rc = cam_flash_driver_cmd(fctrl, arg,
			soc_private);
		break;
	}
	default:
		CAM_ERR(CAM_FLASH, "Invalid ioctl cmd type");
		rc = -EINVAL;
		break;
	}

	CAM_DBG(CAM_FLASH, "Exit");
	return rc;
}

#ifdef CONFIG_COMPAT
static long cam_flash_subdev_do_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd, unsigned long arg)
{
	struct cam_control cmd_data;
	int32_t rc = 0;

	if (copy_from_user(&cmd_data, (void __user *)arg,
		sizeof(cmd_data))) {
		CAM_ERR(CAM_FLASH,
			"Failed to copy from user_ptr=%pK size=%zu",
			(void __user *)arg, sizeof(cmd_data));
		return -EFAULT;
	}

	switch (cmd) {
	case VIDIOC_CAM_CONTROL: {
		rc = cam_flash_subdev_ioctl(sd, cmd, &cmd_data);
		if (rc)
			CAM_ERR(CAM_FLASH, "cam_flash_ioctl failed");
		break;
	}
	default:
		CAM_ERR(CAM_FLASH, "Invalid compat ioctl cmd_type:%d",
			cmd);
		rc = -EINVAL;
	}

	if (!rc) {
		if (copy_to_user((void __user *)arg, &cmd_data,
			sizeof(cmd_data))) {
			CAM_ERR(CAM_FLASH,
				"Failed to copy to user_ptr=%pK size=%zu",
				(void __user *)arg, sizeof(cmd_data));
			rc = -EFAULT;
		}
	}

	return rc;
}
#endif

static int cam_flash_platform_remove(struct platform_device *pdev)
{
	struct cam_flash_ctrl *fctrl;

	fctrl = platform_get_drvdata(pdev);
	if (!fctrl) {
		CAM_ERR(CAM_FLASH, "Flash device is NULL");
		return 0;
	}

	CAM_INFO(CAM_FLASH, "Platform remove invoked");
	mutex_lock(&fctrl->flash_mutex);
	cam_flash_shutdown(fctrl);
	mutex_unlock(&fctrl->flash_mutex);
	cam_unregister_subdev(&(fctrl->v4l2_dev_str));
	platform_set_drvdata(pdev, NULL);
	v4l2_set_subdevdata(&fctrl->v4l2_dev_str.sd, NULL);
	kfree(fctrl);

	return 0;
}

static int32_t cam_flash_i2c_driver_remove(struct i2c_client *client)
{
	int32_t rc = 0;
	struct cam_flash_ctrl *fctrl = i2c_get_clientdata(client);
	/* Handle I2C Devices */
	if (!fctrl) {
		CAM_ERR(CAM_FLASH, "Flash device is NULL");
		return -EINVAL;
	}

	CAM_INFO(CAM_FLASH, "i2c driver remove invoked");
	/*Free Allocated Mem */
	kfree(fctrl->i2c_data.per_frame);
	fctrl->i2c_data.per_frame = NULL;
	kfree(fctrl);
	return rc;
}

static int cam_flash_subdev_close(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh)
{
	struct cam_flash_ctrl *fctrl =
		v4l2_get_subdevdata(sd);

	if (!fctrl) {
		CAM_ERR(CAM_FLASH, "Flash ctrl ptr is NULL");
		return -EINVAL;
	}

	mutex_lock(&fctrl->flash_mutex);
	cam_flash_shutdown(fctrl);
	mutex_unlock(&fctrl->flash_mutex);

	return 0;
}

static struct v4l2_subdev_core_ops cam_flash_subdev_core_ops = {
	.ioctl = cam_flash_subdev_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = cam_flash_subdev_do_ioctl
#endif
};

static struct v4l2_subdev_ops cam_flash_subdev_ops = {
	.core = &cam_flash_subdev_core_ops,
};

static const struct v4l2_subdev_internal_ops cam_flash_internal_ops = {
	.close = cam_flash_subdev_close,
};

static int cam_flash_init_subdev(struct cam_flash_ctrl *fctrl)
{
	int rc = 0;

	strlcpy(fctrl->device_name, CAM_FLASH_NAME,
		sizeof(fctrl->device_name));
	fctrl->v4l2_dev_str.internal_ops =
		&cam_flash_internal_ops;
	fctrl->v4l2_dev_str.ops = &cam_flash_subdev_ops;
	fctrl->v4l2_dev_str.name = CAMX_FLASH_DEV_NAME;
	fctrl->v4l2_dev_str.sd_flags =
		V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	fctrl->v4l2_dev_str.ent_function = CAM_FLASH_DEVICE_TYPE;
	fctrl->v4l2_dev_str.token = fctrl;

	rc = cam_register_subdev(&(fctrl->v4l2_dev_str));
	if (rc)
		CAM_ERR(CAM_FLASH, "Fail to create subdev with %d", rc);

	return rc;
}

static int32_t cam_flash_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0, i = 0;
	struct cam_flash_ctrl *fctrl = NULL;

	CAM_DBG(CAM_FLASH, "Enter");
	if (!pdev->dev.of_node) {
		CAM_ERR(CAM_FLASH, "of_node NULL");
		return -EINVAL;
	}

	fctrl = kzalloc(sizeof(struct cam_flash_ctrl), GFP_KERNEL);
	if (!fctrl)
		return -ENOMEM;

	fctrl->pdev = pdev;
	fctrl->soc_info.pdev = pdev;
	fctrl->soc_info.dev = &pdev->dev;
	fctrl->soc_info.dev_name = pdev->name;

	platform_set_drvdata(pdev, fctrl);

	rc = cam_flash_get_dt_data(fctrl, &fctrl->soc_info);
	if (rc) {
		CAM_ERR(CAM_FLASH, "cam_flash_get_dt_data failed with %d", rc);
		kfree(fctrl);
		return -EINVAL;
	}

	if (of_find_property(pdev->dev.of_node, "cci-master", NULL)) {
		/* Get CCI master */
		rc = of_property_read_u32(pdev->dev.of_node, "cci-master",
			&fctrl->cci_i2c_master);
		CAM_DBG(CAM_FLASH, "cci-master %d, rc %d",
			fctrl->cci_i2c_master, rc);
		if (rc < 0) {
			/* Set default master 0 */
			fctrl->cci_i2c_master = MASTER_0;
			rc = 0;
		}

		fctrl->io_master_info.master_type = CCI_MASTER;
		rc = cam_flash_init_default_params(fctrl);
		if (rc) {
			CAM_ERR(CAM_FLASH,
				"failed: cam_flash_init_default_params rc %d",
				rc);
			return rc;
		}

		fctrl->i2c_data.per_frame = (struct i2c_settings_array *)
			kzalloc(sizeof(struct i2c_settings_array) *
			MAX_PER_FRAME_ARRAY, GFP_KERNEL);
		if (fctrl->i2c_data.per_frame == NULL) {
			CAM_ERR(CAM_FLASH, "No Memory");
			rc = -ENOMEM;
			goto free_cci_resource;
		}

		INIT_LIST_HEAD(&(fctrl->i2c_data.init_settings.list_head));
		INIT_LIST_HEAD(&(fctrl->i2c_data.config_settings.list_head));
		for (i = 0; i < MAX_PER_FRAME_ARRAY; i++)
			INIT_LIST_HEAD(
				&(fctrl->i2c_data.per_frame[i].list_head));

		fctrl->func_tbl.parser = cam_flash_i2c_pkt_parser;
		fctrl->func_tbl.apply_setting = cam_flash_i2c_apply_setting;
		fctrl->func_tbl.power_ops = cam_flash_i2c_power_ops;
		fctrl->func_tbl.flush_req = cam_flash_i2c_flush_request;
	} else {
		/* PMIC Flash */
		fctrl->func_tbl.parser = cam_flash_pmic_pkt_parser;
		fctrl->func_tbl.apply_setting = cam_flash_pmic_apply_setting;
		fctrl->func_tbl.power_ops = cam_flash_pmic_power_ops;
		fctrl->func_tbl.flush_req = cam_flash_pmic_flush_request;
	}

	rc = cam_flash_init_subdev(fctrl);
	if (rc) {
		if (fctrl->io_master_info.cci_client != NULL)
			goto free_cci_resource;
		else
			goto free_resource;
	}

	fctrl->bridge_intf.device_hdl = -1;
	fctrl->bridge_intf.link_hdl = -1;
	fctrl->bridge_intf.ops.get_dev_info = cam_flash_publish_dev_info;
	fctrl->bridge_intf.ops.link_setup = cam_flash_establish_link;
	fctrl->bridge_intf.ops.apply_req = cam_flash_apply_request;
	fctrl->bridge_intf.ops.flush_req = cam_flash_flush_request;
	fctrl->last_flush_req = 0;

	mutex_init(&(fctrl->flash_mutex));

	fctrl->flash_state = CAM_FLASH_STATE_INIT;
	#ifdef VENDOR_EDIT
	if (flash_proc_init(fctrl) < 0) {
		device_create_file(&pdev->dev, &dev_attr_fswitch);
	}
	#endif
	CAM_DBG(CAM_FLASH, "Probe success");
	return rc;

free_cci_resource:
	kfree(fctrl->io_master_info.cci_client);
	fctrl->io_master_info.cci_client = NULL;
free_resource:
	kfree(fctrl->i2c_data.per_frame);
	kfree(fctrl->soc_info.soc_private);
	cam_soc_util_release_platform_resource(&fctrl->soc_info);
	fctrl->i2c_data.per_frame = NULL;
	fctrl->soc_info.soc_private = NULL;
	kfree(fctrl);
	fctrl = NULL;
	return rc;
}

static int32_t cam_flash_i2c_driver_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int32_t rc = 0, i = 0;
	struct cam_flash_ctrl *fctrl;

	if (client == NULL || id == NULL) {
		CAM_ERR(CAM_FLASH, "Invalid Args client: %pK id: %pK",
			client, id);
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		CAM_ERR(CAM_FLASH, "%s :: i2c_check_functionality failed",
			 client->name);
		return -EFAULT;
	}

	/* Create sensor control structure */
	fctrl = kzalloc(sizeof(*fctrl), GFP_KERNEL);
	if (!fctrl)
		return -ENOMEM;

	i2c_set_clientdata(client, fctrl);

	fctrl->io_master_info.client = client;
	fctrl->soc_info.dev = &client->dev;
	fctrl->soc_info.dev_name = client->name;
	fctrl->io_master_info.master_type = I2C_MASTER;

	rc = cam_flash_get_dt_data(fctrl, &fctrl->soc_info);
	if (rc) {
		CAM_ERR(CAM_FLASH, "failed: cam_sensor_parse_dt rc %d", rc);
		goto free_ctrl;
	}

	rc = cam_flash_init_subdev(fctrl);
	if (rc)
		goto free_ctrl;

	fctrl->i2c_data.per_frame =
		(struct i2c_settings_array *)
		kzalloc(sizeof(struct i2c_settings_array) *
		MAX_PER_FRAME_ARRAY, GFP_KERNEL);
	if (fctrl->i2c_data.per_frame == NULL) {
		rc = -ENOMEM;
		goto unreg_subdev;
	}

	INIT_LIST_HEAD(&(fctrl->i2c_data.init_settings.list_head));
	INIT_LIST_HEAD(&(fctrl->i2c_data.config_settings.list_head));
	for (i = 0; i < MAX_PER_FRAME_ARRAY; i++)
		INIT_LIST_HEAD(&(fctrl->i2c_data.per_frame[i].list_head));

	fctrl->func_tbl.parser = cam_flash_i2c_pkt_parser;
	fctrl->func_tbl.apply_setting = cam_flash_i2c_apply_setting;
	fctrl->func_tbl.power_ops = cam_flash_i2c_power_ops;
	fctrl->func_tbl.flush_req = cam_flash_i2c_flush_request;

	fctrl->bridge_intf.device_hdl = -1;
	fctrl->bridge_intf.link_hdl = -1;
	fctrl->bridge_intf.ops.get_dev_info = cam_flash_publish_dev_info;
	fctrl->bridge_intf.ops.link_setup = cam_flash_establish_link;
	fctrl->bridge_intf.ops.apply_req = cam_flash_apply_request;
	fctrl->bridge_intf.ops.flush_req = cam_flash_flush_request;
	fctrl->last_flush_req = 0;

	mutex_init(&(fctrl->flash_mutex));
	fctrl->flash_state = CAM_FLASH_STATE_INIT;

	return rc;

unreg_subdev:
	cam_unregister_subdev(&(fctrl->v4l2_dev_str));
free_ctrl:
	kfree(fctrl);
	fctrl = NULL;
	return rc;
}

MODULE_DEVICE_TABLE(of, cam_flash_dt_match);

static struct platform_driver cam_flash_platform_driver = {
	.probe = cam_flash_platform_probe,
	.remove = cam_flash_platform_remove,
	.driver = {
		.name = "CAM-FLASH-DRIVER",
		.owner = THIS_MODULE,
		.of_match_table = cam_flash_dt_match,
		.suppress_bind_attrs = true,
	},
};

static const struct i2c_device_id i2c_id[] = {
	{FLASH_DRIVER_I2C, (kernel_ulong_t)NULL},
	{ }
};

static struct i2c_driver cam_flash_i2c_driver = {
	.id_table = i2c_id,
	.probe  = cam_flash_i2c_driver_probe,
	.remove = cam_flash_i2c_driver_remove,
	.driver = {
		.name = FLASH_DRIVER_I2C,
	},
};

static int32_t __init cam_flash_init_module(void)
{
	int32_t rc = 0;

	rc = platform_driver_register(&cam_flash_platform_driver);
	if (rc == 0) {
		CAM_DBG(CAM_FLASH, "platform probe success");
		return 0;
	}

	rc = i2c_add_driver(&cam_flash_i2c_driver);
	if (rc)
		CAM_ERR(CAM_FLASH, "i2c_add_driver failed rc: %d", rc);
	return rc;
}

static void __exit cam_flash_exit_module(void)
{
	platform_driver_unregister(&cam_flash_platform_driver);
	i2c_del_driver(&cam_flash_i2c_driver);
}

module_init(cam_flash_init_module);
module_exit(cam_flash_exit_module);
MODULE_DESCRIPTION("CAM FLASH");
MODULE_LICENSE("GPL v2");
